// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/syscore_ops.h>
#include <linux/sipc.h>

#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
#include <linux/sprd_mailbox.h>
#endif

#include "sipc_priv.h"
#include "smsg_uart.h"

#if defined(CONFIG_DEBUG_FS)
#include "sipc_debugfs.h"
#endif

#define SMSG_TXBUF_ADDR (0)
#define SMSG_TXBUF_SIZE (SZ_1K)
#define SMSG_RXBUF_ADDR (SMSG_TXBUF_SIZE)
#define SMSG_RXBUF_SIZE (SZ_1K)

#define SMSG_RINGHDR (SMSG_TXBUF_SIZE + SMSG_RXBUF_SIZE)
#define SMSG_TXBUF_RDPTR (SMSG_RINGHDR + 0)
#define SMSG_TXBUF_WRPTR (SMSG_RINGHDR + 4)
#define SMSG_RXBUF_RDPTR (SMSG_RINGHDR + 8)
#define SMSG_RXBUF_WRPTR (SMSG_RINGHDR + 12)

#define SIPC_READL(addr) readl((__force void __iomem *)(addr))
#define SIPC_WRITEL(b, addr) writel(b, (__force void __iomem *)(addr))

#define HIGH_OFFSET_FLAG (0xFEFE)

static u8 g_wakeup_flag;

struct smsg_ipc *smsg_ipcs[SIPC_ID_NR];
EXPORT_SYMBOL_GPL(smsg_ipcs);

static ushort debug_enable;

module_param_named(debug_enable, debug_enable, ushort, 0644);
static u8 channel2index[SMSG_CH_NR + 1];

/* The UART tuple is a one-shot boot-lifetime reservation. Never publish its
 * cache in legacy channels[]: the legacy busy/open/close scheme cannot pin it.
 */
enum smsg_uart_slot_state {
	SMSG_UART_FREE,
	SMSG_UART_OWNED,
	SMSG_UART_QUARANTINED,
};

struct smsg_uart_channel {
	struct smsg_ipc *ipc;
	enum smsg_uart_slot_state state;
	bool canceled;
	bool overflow;
	bool tx_attempted;
	u32 head;
	u32 tail;
	wait_queue_head_t rxwait;
	struct smsg cache[SMSG_CACHE_NR];
};

static DEFINE_SPINLOCK(smsg_uart_lock);
static struct smsg_uart_channel smsg_uart_slot = {
	.rxwait = __WAIT_QUEUE_HEAD_INITIALIZER(smsg_uart_slot.rxwait),
};
static unsigned int smsg_uart_legacy_inflight;
static bool smsg_uart_legacy_channel;
static bool smsg_uart_legacy_sbuf;

enum smsg_uart_legacy_op {
	SMSG_UART_LEGACY_IO,
	SMSG_UART_LEGACY_OPEN,
	SMSG_UART_LEGACY_CLOSE,
};

static bool smsg_uart_tuple(u8 dst, u8 channel)
{
	return dst == SIPC_ID_PM_SYS && channel == SMSG_CH_COMM;
}

static int smsg_uart_legacy_enter(u8 dst, u8 channel,
				  enum smsg_uart_legacy_op op)
{
	unsigned long flags;
	int ret = 0;

	if (!smsg_uart_tuple(dst, channel))
		return 0;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (smsg_uart_slot.state != SMSG_UART_FREE ||
	    (op == SMSG_UART_LEGACY_OPEN && smsg_uart_legacy_channel)) {
		ret = -EBUSY;
	} else {
		smsg_uart_legacy_inflight++;
		if (op == SMSG_UART_LEGACY_OPEN)
			smsg_uart_legacy_channel = true;
	}
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return ret;
}

static int smsg_uart_legacy_exit(u8 dst, u8 channel,
				 enum smsg_uart_legacy_op op, int result)
{
	unsigned long flags;

	if (!smsg_uart_tuple(dst, channel))
		return result;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (WARN_ON_ONCE(!smsg_uart_legacy_inflight)) {
		spin_unlock_irqrestore(&smsg_uart_lock, flags);
		return result;
	}
	if (op == SMSG_UART_LEGACY_OPEN)
		smsg_uart_legacy_channel = !result;
	else if (op == SMSG_UART_LEGACY_CLOSE && !result)
		smsg_uart_legacy_channel = false;
	smsg_uart_legacy_inflight--;
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return result;
}

int smsg_uart_legacy_sbuf_acquire(u8 dst, u8 channel)
{
	unsigned long flags;
	int ret = 0;

	if (!smsg_uart_tuple(dst, channel))
		return 0;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (smsg_uart_slot.state != SMSG_UART_FREE || smsg_uart_legacy_sbuf)
		ret = -EBUSY;
	else
		smsg_uart_legacy_sbuf = true;
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return ret;
}

void smsg_uart_legacy_sbuf_release(u8 dst, u8 channel)
{
	unsigned long flags;

	if (!smsg_uart_tuple(dst, channel))
		return;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (!WARN_ON_ONCE(!smsg_uart_legacy_sbuf))
		smsg_uart_legacy_sbuf = false;
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
}

/* Called only after validating channel/type. If legacy routing is selected,
 * count admission before releasing this lock, then pair it at exit_msg_proc.
 */
static bool smsg_uart_ingress(struct smsg_ipc *ipc, const struct smsg *msg)
{
	struct smsg_uart_channel *uart = &smsg_uart_slot;
	unsigned long flags;
	bool handled = true;

	if (!smsg_uart_tuple(ipc->dst, msg->channel))
		return false;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (uart->state == SMSG_UART_FREE) {
		smsg_uart_legacy_inflight++;
		handled = false;
	} else if (uart->state == SMSG_UART_OWNED && !uart->canceled &&
		   uart->ipc == ipc) {
		if (uart->head - uart->tail == SMSG_CACHE_NR) {
			/* A lost control message cannot become a successful handshake. */
			WRITE_ONCE(uart->overflow, true);
			WRITE_ONCE(uart->canceled, true);
			uart->state = SMSG_UART_QUARANTINED;
		} else {
			uart->cache[uart->head & (SMSG_CACHE_NR - 1)] = *msg;
			WRITE_ONCE(uart->head, uart->head + 1);
		}
		wake_up_interruptible_all(&uart->rxwait);
	}
	/* Canceled/quarantined messages never reach legacy CLIENT_OPENED. */
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return handled;
}

static int smsg_uart_status_locked(struct smsg_uart_channel *uart)
{
	lockdep_assert_held(&smsg_uart_lock);
	if (uart != &smsg_uart_slot || uart->state == SMSG_UART_FREE)
		return -EINVAL;
	if (uart->overflow)
		return -EOVERFLOW;
	if (uart->canceled || uart->state != SMSG_UART_OWNED || !uart->ipc)
		return -ESHUTDOWN;
	return 0;
}

int smsg_uart_reserve(struct smsg_uart_channel **out)
{
	struct smsg_ipc *ipc;
	unsigned long flags;
	u8 index;
	int ret = 0;

	if (!out)
		return -EINVAL;
	*out = NULL;
	if (!IS_ENABLED(CONFIG_SPRD_SIPC_MBOX_CLIENT))
		return -EOPNOTSUPP;
	/* The enclosing endpoint must be externally serialized and retained. */
	ipc = smsg_ipcs[SIPC_ID_PM_SYS];
	if (!ipc)
		return -EPROBE_DEFER;
	if (ipc->dst != SIPC_ID_PM_SYS || ipc->client ||
	    ipc->type != SIPC_BASE_MBOX || ipc->smem_type != SMEM_LOCAL ||
	    ipc->ring_base || !ipc->smem_inited ||
	    IS_ERR_OR_NULL(ipc->mbox_link))
		return -EOPNOTSUPP;
	index = channel2index[SMSG_CH_COMM];
	if (index >= SMSG_VALID_CH_NR)
		return -EINVAL;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (smsg_uart_slot.state != SMSG_UART_FREE || smsg_uart_legacy_sbuf ||
	    smsg_uart_legacy_channel || smsg_uart_legacy_inflight ||
	    ipc->channels[index] || atomic_read(&ipc->busy[index])) {
		ret = -EBUSY;
	} else {
		smsg_uart_slot.ipc = ipc;
		/* Legacy ingress retained an early validated OPEN as this latch.
		 * Admission is drained under this same lock: transfer it exactly
		 * once instead of waiting for an unproven peer retransmission.
		 */
		if (ipc->states[index] == CHAN_STATE_CLIENT_OPENED) {
			smsg_uart_slot.cache[0].channel = SMSG_CH_COMM;
			smsg_uart_slot.cache[0].type = SMSG_TYPE_OPEN;
			smsg_uart_slot.cache[0].flag = SMSG_OPEN_MAGIC;
			smsg_uart_slot.cache[0].value = 0;
			WRITE_ONCE(smsg_uart_slot.head, 1);
			ipc->states[index] = CHAN_STATE_UNUSED;
		}
		smsg_uart_slot.state = SMSG_UART_OWNED;
		*out = &smsg_uart_slot;
	}
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return ret;
}

int smsg_uart_try_send(struct smsg_uart_channel *uart, const struct smsg *msg)
{
#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
	struct udx710_smsg message;
	unsigned long flags;
	int ret;

	if (!msg || msg->channel != SMSG_CH_COMM || msg->type >= SMSG_TYPE_NR)
		return -EINVAL;
	message.channel = msg->channel;
	message.type = msg->type;
	message.flag = msg->flag;
	message.value = msg->value;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	ret = smsg_uart_status_locked(uart);
	if (ret)
		goto unlock;
	if (READ_ONCE(uart->ipc->suspend)) {
		ret = -EAGAIN;
		goto unlock;
	}
	/* Cancel cannot pass this boundary until enqueue returns. The target
	 * mailbox client/provider path is atomic-safe; no consumer callback runs
	 * synchronously from its TX completion. Never wait for remote completion.
	 */
	uart->tx_attempted = true;
	ret = udx710_smsg_link_send(uart->ipc->mbox_link, &message);
unlock:
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return ret;
#else
	return -EOPNOTSUPP;
#endif
}

int smsg_uart_try_recv(struct smsg_uart_channel *uart, struct smsg *msg)
{
	unsigned long flags;
	int ret;

	if (!msg)
		return -EINVAL;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	ret = smsg_uart_status_locked(uart);
	if (!ret) {
		if (uart->head == uart->tail) {
			ret = -ENODATA;
		} else {
			*msg = uart->cache[uart->tail & (SMSG_CACHE_NR - 1)];
			WRITE_ONCE(uart->tail, uart->tail + 1);
		}
	}
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return ret;
}

int smsg_uart_recv_until(struct smsg_uart_channel *uart, struct smsg *msg,
			 unsigned long deadline)
{
	long left;
	int ret;

	if (uart != &smsg_uart_slot || !msg)
		return -EINVAL;
	might_sleep();
	for (;;) {
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		ret = smsg_uart_try_recv(uart, msg);
		if (ret != -ENODATA)
			return ret;
		left = deadline - jiffies;
		if (left <= 0)
			return -ETIMEDOUT;
		left = wait_event_interruptible_timeout(
			uart->rxwait,
			READ_ONCE(uart->canceled) ||
				READ_ONCE(uart->head) != READ_ONCE(uart->tail),
			left);
		if (left < 0)
			return left;
		if (!left)
			return -ETIMEDOUT;
	}
}

int smsg_uart_wait_retry(struct smsg_uart_channel *uart, unsigned long deadline,
			 unsigned long interval)
{
	unsigned long flags;
	long left;
	int ret;

	if (uart != &smsg_uart_slot || !interval || interval > MAX_JIFFY_OFFSET)
		return -EINVAL;
	might_sleep();
	spin_lock_irqsave(&smsg_uart_lock, flags);
	ret = smsg_uart_status_locked(uart);
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	if (ret)
		return ret;
	if (time_after_eq(jiffies, deadline))
		return -ETIMEDOUT;
	left = deadline - jiffies;
	if (left <= 0)
		return -ETIMEDOUT;
	left = wait_event_interruptible_timeout(
		uart->rxwait, READ_ONCE(uart->canceled),
		min_t(unsigned long, left, interval));
	if (left < 0)
		return left;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	ret = smsg_uart_status_locked(uart);
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	if (ret)
		return ret;
	return time_after_eq(jiffies, deadline) ? -ETIMEDOUT : 0;
}

void smsg_uart_cancel(struct smsg_uart_channel *uart)
{
	unsigned long flags;

	if (uart != &smsg_uart_slot)
		return;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (uart->state != SMSG_UART_FREE) {
		WRITE_ONCE(uart->canceled, true);
		uart->state = SMSG_UART_QUARANTINED;
		wake_up_interruptible_all(&uart->rxwait);
	}
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
}

int smsg_uart_release(struct smsg_uart_channel *uart)
{
	unsigned long flags;
	int ret = 0;

	if (uart != &smsg_uart_slot)
		return -EINVAL;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	if (uart->state == SMSG_UART_FREE) {
		ret = -EINVAL;
	} else {
		WRITE_ONCE(uart->canceled, true);
		uart->state = SMSG_UART_QUARANTINED;
		/* IRQ never retains this pointer outside the same lock. */
		uart->ipc = NULL;
		wake_up_interruptible_all(&uart->rxwait);
	}
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return ret;
}

bool smsg_uart_may_have_sent(struct smsg_uart_channel *uart)
{
	unsigned long flags;
	bool attempted;

	if (uart != &smsg_uart_slot)
		return true;
	spin_lock_irqsave(&smsg_uart_lock, flags);
	attempted = uart->tx_attempted;
	spin_unlock_irqrestore(&smsg_uart_lock, flags);
	return attempted;
}

static int smsg_ipc_smem_init(struct smsg_ipc *ipc);

void smsg_init_channel2index(void)
{
	u16 i, j;

	for (i = 0; i < ARRAY_SIZE(channel2index); i++) {
		for (j = 0; j < SMSG_VALID_CH_NR; j++) {
			/* find the index of channel i */
			if (sipc_cfg[j].channel == i)
				break;
		}

		/* if not find, init with INVALID_CHANEL_INDEX,
		 * else init whith j
		 */
		if (j == SMSG_VALID_CH_NR)
			channel2index[i] = INVALID_CHANEL_INDEX;
		else
			channel2index[i] = j;
	}
}

static void get_channel_status(u8 dst, char *status, int size)
{
	u8 ch_index;
	int i, len;
	struct smsg_channel *ch;

	len = strlen(status);
	for (i = 0; i < SMSG_VALID_CH_NR && len < size; i++) {
		ch_index = channel2index[i];
		if (ch_index == INVALID_CHANEL_INDEX)
			continue;
		ch = smsg_ipcs[dst]->channels[ch_index];
		if (!ch)
			continue;
		if (SIPC_READL(ch->rdptr) < SIPC_READL(ch->wrptr))
			snprintf(status + len, size - len,
				 "dst-%d-ch-%d: rd = %u, wr = %u.\n", dst, i,
				 SIPC_READL(ch->rdptr), SIPC_READL(ch->wrptr));
	}
}

static void smsg_wakeup_print(struct smsg_ipc *ipc, struct smsg *msg)
{
	/* if the first msg come after the irq wake up by sipc,
	 * use prin_fo to output log
	 */
	if (g_wakeup_flag) {
		g_wakeup_flag = 0;
		pr_info("irq read smsg: dst=%d, channel=%d,type=%d, flag=0x%04x, value=0x%08x\n",
			ipc->dst, msg->channel, msg->type, msg->flag,
			msg->value);
	} else {
		pr_debug(
			"irq read smsg: dst=%d, channel=%d,type=%d, flag=0x%04x, value=0x%08x\n",
			ipc->dst, msg->channel, msg->type, msg->flag,
			msg->value);
	}
}

static void smsg_die_process(struct smsg_ipc *ipc, struct smsg *msg)
{
	if (msg->type == SMSG_TYPE_DIE) {
		if (debug_enable) {
			char sipc_status[100] = { 0 };

			get_channel_status(ipc->dst, sipc_status,
					   sizeof(sipc_status));
			sbuf_get_status(ipc->dst, sipc_status,
					sizeof(sipc_status));
			panic("cpcrash: %s", sipc_status);
			while (1)
				;
		}
	}
}

static void smsg_msg_process(struct smsg_ipc *ipc, struct smsg *msg)
{
	struct smsg_channel *ch = NULL;
	u32 wr;
	u8 ch_index;

	/* Validate peer-controlled indices before any array access or action. */
	if (msg->channel >= SMSG_CH_NR || msg->type >= SMSG_TYPE_NR)
		return;
	ch_index = channel2index[msg->channel];
	if (ch_index >= SMSG_VALID_CH_NR)
		return;

	if (smsg_uart_ingress(ipc, msg))
		return;

	smsg_wakeup_print(ipc, msg);
	smsg_die_process(ipc, msg);
	atomic_inc(&ipc->busy[ch_index]);

	pr_debug(
		"smsg:get dst=%d msg channel=%d, type=%d, flag=0x%04x, value=0x%08x\n",
		ipc->dst, msg->channel, msg->type, msg->flag, msg->value);

	if (msg->type >= SMSG_TYPE_NR) {
		/* invalid msg */
		pr_info("invalid smsg: channel=%d, type=%d, flag=0x%04x, value=0x%08x\n",
			msg->channel, msg->type, msg->flag, msg->value);
		goto exit_msg_proc;
	}

	if (ipc->client && msg->type == SMSG_TYPE_HIGH_OFFSET &&
	    msg->flag == HIGH_OFFSET_FLAG) {
		/* high offset msg */
		ipc->high_offset = msg->value;
		pr_info("smsg:  high_offset = 0x%x\n", msg->value);
		goto exit_msg_proc;
	}

	ch = ipc->channels[ch_index];
	if (!ch) {
		if (ipc->states[ch_index] == CHAN_STATE_UNUSED &&
		    msg->type == SMSG_TYPE_OPEN && msg->flag == SMSG_OPEN_MAGIC)
			ipc->states[ch_index] = CHAN_STATE_CLIENT_OPENED;
		else
			/* drop this bad msg since channel
			 * is not opened
			 */
			pr_info("smsg channel %d not opened! drop smsg: type=%d, flag=0x%04x, value=0x%08x\n",
				msg->channel, msg->type, msg->flag, msg->value);

		goto exit_msg_proc;
	}

	if ((int)(SIPC_READL(ch->wrptr) - SIPC_READL(ch->rdptr)) >=
	    SMSG_CACHE_NR) {
		/* msg cache is full, drop this msg */
		pr_info("smsg channel %d recv cache is full! drop smsg: type=%d, flag=0x%04x, value=0x%08x\n",
			msg->channel, msg->type, msg->flag, msg->value);
	} else {
		/* write smsg to cache */
		wr = SIPC_READL(ch->wrptr) & (SMSG_CACHE_NR - 1);
		memcpy(&ch->caches[wr], msg, sizeof(struct smsg));
		SIPC_WRITEL(SIPC_READL(ch->wrptr) + 1, ch->wrptr);
	}

	wake_up_interruptible_all(&ch->rxwait);
	__pm_wakeup_event(ch->sipc_wake_lock, jiffies_to_msecs(HZ / 2));

exit_msg_proc:
	atomic_dec(&ipc->busy[ch_index]);
	smsg_uart_legacy_exit(ipc->dst, msg->channel, SMSG_UART_LEGACY_IO, 0);
}

#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
static irqreturn_t smsg_mbox_irq_handler(void *ptr, void *private)
{
	struct smsg_ipc *ipc = (struct smsg_ipc *)private;
	struct smsg *msg;

	msg = ptr;
	smsg_msg_process(ipc, msg);

	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
static void smsg_std_mbox_receive(void *private,
				  const struct udx710_smsg *message)
{
	struct smsg msg;

	msg.channel = message->channel;
	msg.type = message->type;
	msg.flag = message->flag;
	msg.value = message->value;
	smsg_msg_process(private, &msg);
}
#endif

#ifndef CONFIG_SPRD_SIPC_MBOX_CLIENT
static irqreturn_t smsg_irq_handler(int irq, void *private)
{
	struct smsg_ipc *ipc = (struct smsg_ipc *)private;
	struct smsg *msg;
	uintptr_t rxpos;

	pr_debug("%s: %s, irq = %d\n", __func__, ipc->name, irq);

	if (ipc->rxirq_status(ipc->dst))
		ipc->rxirq_clear(ipc->dst);

	if (!ipc->smem_inited) {
		pr_info("%s: %s smem have not been inited!", __func__,
			ipc->name);
		return IRQ_HANDLED;
	}

	while (SIPC_READL(ipc->rxbuf_wrptr) != SIPC_READL(ipc->rxbuf_rdptr)) {
		rxpos = (SIPC_READL(ipc->rxbuf_rdptr) & (ipc->rxbuf_size - 1)) *
				sizeof(struct smsg) +
			ipc->rxbuf_addr;
		msg = (struct smsg *)rxpos;
		pr_debug("irq get smsg: wrptr=%u, rdptr=%u, rxpos=0x%lx\n",
			 SIPC_READL(ipc->rxbuf_wrptr),
			 SIPC_READL(ipc->rxbuf_rdptr), rxpos);

		smsg_msg_process(ipc, msg);
		/* update smsg rdptr */
		SIPC_WRITEL(SIPC_READL(ipc->rxbuf_rdptr) + 1, ipc->rxbuf_rdptr);
	}

	return IRQ_HANDLED;
}

#endif /* legacy shared-ring interrupt */

static int smsg_send_high_offset_thread(void *data)
{
	struct smsg msg;
	struct smsg_ipc *ipc = (struct smsg_ipc *)data;

	smsg_set(&msg, SMSG_CH_CTRL, SMSG_TYPE_HIGH_OFFSET, HIGH_OFFSET_FLAG,
		 ipc->dst_high_offset);
	smsg_send(ipc->dst, &msg, 0);

	pr_info("smsg: send high offset(%d) to client!\n",
		ipc->dst_high_offset);
	ipc->thread = NULL;

	return 0;
}

static void smsg_send_high_offset(struct smsg_ipc *ipc)
{
	u8 dst = ipc->dst;

	/* host sipc, if offset > 0, send high offset to dst */
	if (!ipc->client && ipc->dst_high_offset && !ipc->thread) {
		ipc->thread = kthread_create(smsg_send_high_offset_thread, ipc,
					     "sipc-send-%d", dst);
		if (!IS_ERR(ipc->thread))
			wake_up_process(ipc->thread);
	}
}

#ifndef CONFIG_SPRD_SIPC_MBOX_CLIENT
#if defined(CONFIG_SPRD_PCIE_EP_DEVICE) || defined(CONFIG_PCIE_EPF_SPRD)
static void smsg_resume_work(struct smsg_ipc *ipc)
{
	unsigned long flags;

	pr_debug("%s: %s\n", __func__, ipc->name);

	spin_lock_irqsave(&ipc->suspend_pinlock, flags);
	ipc->suspend = 0;
	wake_up_interruptible_all(&ipc->suspend_wait);
	spin_unlock_irqrestore(&ipc->suspend_pinlock, flags);
	smsg_send_high_offset(ipc);
}

static void smsg_suspend_work(struct smsg_ipc *ipc)
{
	unsigned long flags;

	pr_debug("%s: %s, task %s!\n", __func__, ipc->name, current->comm);

	spin_lock_irqsave(&ipc->suspend_pinlock, flags);
	ipc->suspend = 1;
	spin_unlock_irqrestore(&ipc->suspend_pinlock, flags);
}
#endif

#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
static void smsg_pcie_ep_dev_notify(int event, void *data)
{
	struct smsg_ipc *ipc = (struct smsg_ipc *)data;

	pr_debug("%s: %s, event = %d\n", __func__, ipc->name, event);

	if (event == PCIE_EP_PROBE)
		smsg_resume_work(ipc);
	else if (event == PCIE_EP_REMOVE)
		smsg_suspend_work(ipc);
}
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
static void smsg_pcie_epf_notify(int event, void *data)
{
	struct smsg_ipc *ipc = (struct smsg_ipc *)data;

	pr_debug("%s: %s, event = %d\n", __func__, ipc->name, event);

	if (event == SPRD_EPF_BIND) {
		smsg_ipc_smem_init(ipc);
		smsg_resume_work(ipc);
	} else if (event == SPRD_EPF_UNBIND || event == SPRD_EPF_REMOVE)
		smsg_suspend_work(ipc);
}
#endif

static void smsg_ipc_init_irq_callback(struct smsg_ipc *ipc)
{
	int ret1 = 0, ret2 = 0;

#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
	if (ipc->type == SIPC_BASE_MBOX) {
		ret1 = mbox_register_irq_handle(ipc->core_id,
						smsg_mbox_irq_handler, ipc);

		if ((ipc->dst == SIPC_ID_PM_SYS) &&
		    (ipc->core_sensor_id != MBOX_INVALID_CORE))
			ret2 = mbox_register_irq_handle(ipc->core_sensor_id,
							smsg_mbox_irq_handler,
							ipc);
		return;
	}
#endif

	if (ipc->type == SIPC_BASE_PCIE) {
#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
		ipc->irq = PCIE_EP_SIPC_IRQ;
		ret1 = sprd_ep_dev_register_irq_handler(ipc->ep_dev, ipc->irq,
							smsg_irq_handler, ipc);
		ret2 = sprd_ep_dev_register_notify(
			ipc->ep_dev, smsg_pcie_ep_dev_notify, ipc);
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
		sprd_pci_epf_set_irq_number(ipc->ep_fun, ipc->irq);
		ret1 = sprd_pci_epf_register_irq_handler(
			ipc->ep_fun, SPRD_EPF_SIPC_IRQ, smsg_irq_handler, ipc);
		ret2 = sprd_pci_epf_register_notify(ipc->ep_fun,
						    smsg_pcie_epf_notify, ipc);
#endif
	} else if (ipc->type == SIPC_BASE_IPI) {
		/* explicitly call irq handler in case of missing irq on boot */
		smsg_irq_handler(ipc->irq, ipc);

		/* register IPI irq */
		ret2 = ret1 = request_irq(ipc->irq, smsg_irq_handler,
					  IRQF_NO_SUSPEND, ipc->name, ipc);
	}
	pr_debug("%s:%s ret1=%d, ret2=%d\n", __func__, ipc->name, ret1, ret2);
}

#endif /* legacy transport registration */

static int smsg_ipc_smem_init(struct smsg_ipc *ipc)
{
	void __iomem *base, *p;
	int ret;

	if (ipc->smem_inited)
		return 0;

	pr_debug("%s: %s!\n", __func__, ipc->name);
	ret = smem_init(ipc->smem_base, ipc->smem_size, ipc->dst,
			ipc->smem_type);
	if (ret)
		return ret;

	if (ipc->type == SIPC_BASE_PCIE) {
		ipc->ring_base = smem_alloc(ipc->dst, SZ_4K);
		ipc->ring_size = SZ_4K;
		pr_info("%s: ring_base = 0x%x, ring_size = 0x%x\n", __func__,
			ipc->ring_base, ipc->ring_size);
	}

	if (ipc->ring_base) {
		base = (void __iomem *)shmem_ram_vmap_nocache(
			ipc->dst, ipc->ring_base + ipc->high_offset,
			ipc->ring_size);
		if (!base) {
			pr_err("%s: ioremap failed!\n", __func__);
			return -ENOMEM;
		}

		/* assume client is boot later than host */
		if (!ipc->client) {
			/**
			 * memset(base, 0, ipc->ring_size);
			 * the instruction dc avz
			 * will abort for nocache memory
			 */
			for (p = base; p < base + ipc->ring_size;) {
#ifdef CONFIG_64BIT
				*(uint64_t *)p = 0x0;
				p += sizeof(uint64_t);
#else
				*(u32 *)p = 0x0;
				p += sizeof(u32);
#endif
			}
		}

		if (ipc->client) {
			/* clent mode, tx is host rx , rx is host tx*/
			ipc->smem_vbase = (void *)base;
			ipc->txbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);
			ipc->txbuf_addr = (uintptr_t)base + SMSG_RXBUF_ADDR;
			ipc->txbuf_rdptr = (uintptr_t)base + SMSG_RXBUF_RDPTR;
			ipc->txbuf_wrptr = (uintptr_t)base + SMSG_RXBUF_WRPTR;
			ipc->rxbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
			ipc->rxbuf_addr = (uintptr_t)base + SMSG_TXBUF_ADDR;
			ipc->rxbuf_rdptr = (uintptr_t)base + SMSG_TXBUF_RDPTR;
			ipc->rxbuf_wrptr = (uintptr_t)base + SMSG_TXBUF_WRPTR;
		} else {
			ipc->smem_vbase = (void *)base;
			ipc->txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
			ipc->txbuf_addr = (uintptr_t)base + SMSG_TXBUF_ADDR;
			ipc->txbuf_rdptr = (uintptr_t)base + SMSG_TXBUF_RDPTR;
			ipc->txbuf_wrptr = (uintptr_t)base + SMSG_TXBUF_WRPTR;
			ipc->rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);
			ipc->rxbuf_addr = (uintptr_t)base + SMSG_RXBUF_ADDR;
			ipc->rxbuf_rdptr = (uintptr_t)base + SMSG_RXBUF_RDPTR;
			ipc->rxbuf_wrptr = (uintptr_t)base + SMSG_RXBUF_WRPTR;
		}
	}

	smsg_send_high_offset(ipc);
	ipc->smem_inited = 1;

	return 0;
}

int smsg_ipc_create(struct smsg_ipc *ipc)
{
#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
	int ret;

	if (smsg_ipcs[ipc->dst])
		return -EBUSY;
	/* No clients may be created until probe returns successfully. */
	smsg_ipcs[ipc->dst] = ipc;
	ipc->mbox_link = udx710_smsg_link_open(ipc->dev, "smsg",
					       smsg_std_mbox_receive, ipc);
	if (IS_ERR(ipc->mbox_link)) {
		ret = PTR_ERR(ipc->mbox_link);
		ipc->mbox_link = NULL;
		smsg_ipcs[ipc->dst] = NULL;
		return ret;
	}
	ret = smsg_ipc_smem_init(ipc);
	if (ret) {
		udx710_smsg_link_close(ipc->mbox_link);
		ipc->mbox_link = NULL;
		smsg_ipcs[ipc->dst] = NULL;
	}
	return ret;
#else
	pr_info("%s: %s\n", __func__, ipc->name);
	smsg_ipc_init_irq_callback(ipc);
	smsg_ipcs[ipc->dst] = ipc;
	if (ipc->smem_type == SMEM_PCIE)
		pr_info("%s: will init smem until pcie driver probe!",
			ipc->name);
	else
		return smsg_ipc_smem_init(ipc);
	return 0;
#endif
}

void smsg_ipc_destroy(struct smsg_ipc *ipc)
{
	if (!ipc)
		return;
#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
	udx710_smsg_link_close(ipc->mbox_link);
	ipc->mbox_link = NULL;
#endif
	if (ipc->smem_vbase)
		shmem_ram_unmap(ipc->dst, ipc->smem_vbase);
	if (ipc->ring_base)
		smem_free(ipc->dst, ipc->ring_base, ipc->ring_size);

#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
	if (ipc->type == SIPC_BASE_MBOX) {
		mbox_unregister_irq_handle(ipc->core_id);

		if ((ipc->dst == SIPC_ID_PM_SYS) &&
		    (ipc->core_sensor_id != MBOX_INVALID_CORE))
			mbox_unregister_irq_handle(ipc->core_sensor_id);
	}
#endif

	if (ipc->type == SIPC_BASE_PCIE) {
#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
		sprd_ep_dev_unregister_irq_handler(ipc->ep_dev, ipc->irq);
		sprd_ep_dev_unregister_notify(ipc->ep_dev);
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
		free_irq(ipc->irq, ipc);
		sprd_pci_epf_unregister_notify(ipc->ep_fun);
#endif
	} else if (ipc->type == SIPC_BASE_IPI) {
		free_irq(ipc->irq, ipc);
	}

	if (!IS_ERR_OR_NULL(ipc->thread))
		kthread_stop(ipc->thread);

	smsg_ipcs[ipc->dst] = NULL;
}

int sipc_get_wakeup_flag(void)
{
	return (int)g_wakeup_flag;
}
EXPORT_SYMBOL_GPL(sipc_get_wakeup_flag);

void sipc_set_wakeup_flag(void)
{
	g_wakeup_flag = 1;
}
EXPORT_SYMBOL_GPL(sipc_set_wakeup_flag);

void sipc_clear_wakeup_flag(void)
{
	g_wakeup_flag = 0;
}
EXPORT_SYMBOL_GPL(sipc_clear_wakeup_flag);

static int smsg_ch_wake_unlock_legacy(u8 dst, u8 channel)
{
	struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	struct smsg_channel *ch;
	u8 ch_index;

	ch_index = channel2index[channel];
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	if (!ipc)
		return -ENODEV;

	ch = ipc->channels[ch_index];
	if (!ch)
		return -ENODEV;

	__pm_relax(ch->sipc_wake_lock);
	return 0;
}
int smsg_ch_wake_unlock(u8 dst, u8 channel)
{
	int ret;

	ret = smsg_uart_legacy_enter(dst, channel, SMSG_UART_LEGACY_IO);
	if (ret)
		return ret;
	ret = smsg_ch_wake_unlock_legacy(dst, channel);
	return smsg_uart_legacy_exit(dst, channel, SMSG_UART_LEGACY_IO, ret);
}
EXPORT_SYMBOL_GPL(smsg_ch_wake_unlock);

static int smsg_ch_open_legacy(u8 dst, u8 channel, int timeout)
{
	struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	struct smsg_channel *ch;
	struct smsg mopen;
	struct smsg mrecv;
	int rval = 0;
	u8 ch_index;

	ch_index = channel2index[channel];
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	if (!ipc)
		return -ENODEV;

	ch = kzalloc_obj(*ch, GFP_KERNEL);
	if (!ch)
		return -ENOMEM;

	sprintf(ch->wake_lock_name, "smsg-%d-%d", dst, channel);
	ch->sipc_wake_lock =
		wakeup_source_register(ipc->dev, ch->wake_lock_name);
	if (IS_ENABLED(CONFIG_PM_SLEEP) && !ch->sipc_wake_lock) {
		kfree(ch);
		return -ENOMEM;
	}

	atomic_set(&ipc->busy[ch_index], 1);
	init_waitqueue_head(&ch->rxwait);
	mutex_init(&ch->rxlock);
	ipc->channels[ch_index] = ch;

	pr_info("%s: channel %d-%d send open msg!\n", __func__, dst, channel);

	smsg_set(&mopen, channel, SMSG_TYPE_OPEN, SMSG_OPEN_MAGIC, 0);
	rval = smsg_send(dst, &mopen, timeout);
	if (rval != 0) {
		pr_err("%s: channel %d-%d send open msg error = %d!\n",
		       __func__, dst, channel, rval);
		ipc->states[ch_index] = CHAN_STATE_UNUSED;
		ipc->channels[ch_index] = NULL;
		atomic_dec(&ipc->busy[ch_index]);
		/* guarantee that channel resource isn't used in irq handler  */
		while (atomic_read(&ipc->busy[ch_index]))
			;

		wakeup_source_unregister(ch->sipc_wake_lock);
		kfree(ch);

		return rval;
	}

	/* open msg might be got before */
	if (ipc->states[ch_index] == CHAN_STATE_CLIENT_OPENED)
		goto open_done;

	ipc->states[ch_index] = CHAN_STATE_HOST_OPENED;

	do {
		smsg_set(&mrecv, channel, 0, 0, 0);
		rval = smsg_recv(dst, &mrecv, timeout);
		if (rval != 0) {
			pr_err("%s: channel %d-%d smsg receive error = %d!\n",
			       __func__, dst, channel, rval);
			ipc->states[ch_index] = CHAN_STATE_UNUSED;
			ipc->channels[ch_index] = NULL;
			atomic_dec(&ipc->busy[ch_index]);
			/* guarantee that channel resource isn't used
			 * in irq handler
			 */
			while (atomic_read(&ipc->busy[ch_index]))
				;

			wakeup_source_unregister(ch->sipc_wake_lock);
			kfree(ch);
			return rval;
		}
	} while (mrecv.type != SMSG_TYPE_OPEN || mrecv.flag != SMSG_OPEN_MAGIC);

	pr_info("%s: channel %d-%d receive open msg!\n", __func__, dst,
		channel);

open_done:
	pr_info("%s: channel %d-%d success\n", __func__, dst, channel);
	ipc->states[ch_index] = CHAN_STATE_OPENED;
	atomic_dec(&ipc->busy[ch_index]);

	return 0;
}
int smsg_ch_open(u8 dst, u8 channel, int timeout)
{
	int ret;

	ret = smsg_uart_legacy_enter(dst, channel, SMSG_UART_LEGACY_OPEN);
	if (ret)
		return ret;
	ret = smsg_ch_open_legacy(dst, channel, timeout);
	return smsg_uart_legacy_exit(dst, channel, SMSG_UART_LEGACY_OPEN, ret);
}
EXPORT_SYMBOL_GPL(smsg_ch_open);

static int smsg_ch_close_legacy(u8 dst, u8 channel, int timeout)
{
	struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	struct smsg_channel *ch;
	struct smsg mclose;
	u8 ch_index;

	ch_index = channel2index[channel];
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	if (!ipc)
		return -ENODEV;
	ch = ipc->channels[ch_index];
	if (!ch)
		return 0;

	smsg_set(&mclose, channel, SMSG_TYPE_CLOSE, SMSG_CLOSE_MAGIC, 0);
	smsg_send(dst, &mclose, timeout);

	ipc->states[ch_index] = CHAN_STATE_FREE;
	wake_up_interruptible_all(&ch->rxwait);

	/* wait for the channel being unused */
	while (atomic_read(&ipc->busy[ch_index]))
		;

	/* maybe channel has been free for smsg_ch_open failed */
	if (ipc->channels[ch_index]) {
		ipc->channels[ch_index] = NULL;
		/* guarantee that channel resource isn't used in irq handler */
		while (atomic_read(&ipc->busy[ch_index]))
			;

		wakeup_source_unregister(ch->sipc_wake_lock);
		kfree(ch);
	}

	/* finally, update the channel state*/
	ipc->states[ch_index] = CHAN_STATE_UNUSED;

	return 0;
}
int smsg_ch_close(u8 dst, u8 channel, int timeout)
{
	int ret;

	ret = smsg_uart_legacy_enter(dst, channel, SMSG_UART_LEGACY_CLOSE);
	if (ret)
		return ret;
	ret = smsg_ch_close_legacy(dst, channel, timeout);
	return smsg_uart_legacy_exit(dst, channel, SMSG_UART_LEGACY_CLOSE, ret);
}
EXPORT_SYMBOL_GPL(smsg_ch_close);

int smsg_senddie(u8 dst)
{
	struct smsg msg;
	struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	uintptr_t txpos;
	int rval = 0;

	if (!ipc)
		return -ENODEV;

#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
	/* Panic/crash injection needs a reviewed poll/flush path. */
	return -EOPNOTSUPP;
#endif
	msg.channel = SMSG_CH_CTRL;
	msg.type = SMSG_TYPE_DIE;
	msg.flag = 0;
	msg.value = 0;

#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
	if (ipc->type == SIPC_BASE_MBOX) {
		mbox_just_sent(ipc->core_id, *((u64 *)&msg));
		return 0;
	}
#endif

	if (ipc->ring_base) {
		if ((int)(SIPC_READL(ipc->txbuf_wrptr) -
			  SIPC_READL(ipc->txbuf_rdptr)) >= ipc->txbuf_size) {
			pr_info("smsg_send: smsg txbuf is full!\n");
			rval = -EBUSY;
			goto send_failed;
		}

		/* calc txpos and write smsg */
		txpos = (SIPC_READL(ipc->txbuf_wrptr) & (ipc->txbuf_size - 1)) *
				sizeof(struct smsg) +
			ipc->txbuf_addr;
		memcpy((void *)txpos, &msg, sizeof(struct smsg));

		pr_debug("write smsg: wrptr=%u, rdptr=%u, txpos=0x%lx\n",
			 SIPC_READL(ipc->txbuf_wrptr),
			 SIPC_READL(ipc->txbuf_rdptr), txpos);

		/* update wrptr */
		SIPC_WRITEL(SIPC_READL(ipc->txbuf_wrptr) + 1, ipc->txbuf_wrptr);
	}

	rval = ipc->txirq_trigger(ipc->dst, *((u64 *)&msg));
send_failed:

	return rval;
}
EXPORT_SYMBOL_GPL(smsg_senddie);

static int smsg_send_legacy(u8 dst, struct smsg *msg, int timeout)
{
	struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	uintptr_t txpos;
	int rval = 0;
	unsigned long flags;
	u8 ch_index;

	if (!msg || msg->channel >= SMSG_CH_NR || msg->type >= SMSG_TYPE_NR)
		return -EINVAL;
	ch_index = channel2index[msg->channel];
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, msg->channel);
		return -EINVAL;
	}

	if (!ipc)
		return -ENODEV;

	if (!ipc->channels[ch_index]) {
		pr_err("%s: channel %d not inited!\n", __func__, msg->channel);
		return -ENODEV;
	}

	if (ipc->states[ch_index] != CHAN_STATE_OPENED &&
	    msg->type != SMSG_TYPE_OPEN && msg->type != SMSG_TYPE_CLOSE) {
		pr_err("%s: channel %d not opened!\n", __func__, msg->channel);
		return -EINVAL;
	}

	pr_debug("%s: dst=%d, channel=%d, timeout=%d, suspend=%d\n", __func__,
		 dst, msg->channel, timeout, ipc->suspend);

	if (ipc->suspend) {
		if (!timeout)
			return -EAGAIN;
		if (timeout < 0) {
			rval = wait_event_interruptible(
				ipc->suspend_wait, !READ_ONCE(ipc->suspend));
		} else {
			rval = wait_event_interruptible_timeout(
				ipc->suspend_wait, !READ_ONCE(ipc->suspend),
				timeout);
			if (!rval)
				return -ETIME;
			if (rval > 0)
				timeout = rval;
		}
		if (rval < 0)
			return rval;
	}
	pr_debug("send smsg: channel=%d, type=%d, flag=0x%04x, value=0x%08x\n",
		 msg->channel, msg->type, msg->flag, msg->value);

#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
	if (!ipc->ring_base) {
		struct udx710_smsg message = {
			.channel = msg->channel,
			.type = msg->type,
			.flag = msg->flag,
			.value = msg->value,
		};

		/* Local mailbox payload has no shared SMSG ring to serialize. */
		return udx710_smsg_link_send_timeout(ipc->mbox_link, &message,
						     timeout);
	}
#endif
	spin_lock_irqsave(&ipc->txpinlock, flags);

	if (ipc->ring_base) {
		if (((int)(SIPC_READL(ipc->txbuf_wrptr) -
			   SIPC_READL(ipc->txbuf_rdptr)) >= ipc->txbuf_size)) {
			pr_info("%s: smsg txbuf is full!\n", __func__);
			rval = -EBUSY;
			goto send_failed;
		}

		/* calc txpos and write smsg */
		txpos = (SIPC_READL(ipc->txbuf_wrptr) & (ipc->txbuf_size - 1)) *
				sizeof(struct smsg) +
			ipc->txbuf_addr;
		memcpy((void *)txpos, msg, sizeof(struct smsg));

		pr_debug("write smsg: wrptr=0x%x, rdptr=0x%x, txpos=0x%lx\n",
			 SIPC_READL(ipc->txbuf_wrptr),
			 SIPC_READL(ipc->txbuf_rdptr), txpos);

		/* update wrptr */
		SIPC_WRITEL(SIPC_READL(ipc->txbuf_wrptr) + 1, ipc->txbuf_wrptr);
	}

	rval = ipc->txirq_trigger(ipc->dst, *(u64 *)msg);

send_failed:
	spin_unlock_irqrestore(&ipc->txpinlock, flags);

	return rval;
}
int smsg_send(u8 dst, struct smsg *msg, int timeout)
{
	int ret;

	if (!msg)
		return -EINVAL;
	ret = smsg_uart_legacy_enter(dst, msg->channel, SMSG_UART_LEGACY_IO);
	if (ret)
		return ret;
	ret = smsg_send_legacy(dst, msg, timeout);
	return smsg_uart_legacy_exit(dst, msg->channel, SMSG_UART_LEGACY_IO,
				     ret);
}
EXPORT_SYMBOL_GPL(smsg_send);

static int smsg_recv_legacy(u8 dst, struct smsg *msg, int timeout)
{
	struct smsg_ipc *ipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	struct smsg_channel *ch;
	u32 rd;
	int rval = 0;
	u8 ch_index;

	if (!msg || msg->channel >= SMSG_CH_NR)
		return -EINVAL;
	ch_index = channel2index[msg->channel];
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, msg->channel);
		return -EINVAL;
	}

	if (!ipc)
		return -ENODEV;

	atomic_inc(&ipc->busy[ch_index]);

	ch = ipc->channels[ch_index];

	if (!ch) {
		pr_err("%s: channel %d not opened!\n", __func__, msg->channel);
		atomic_dec(&ipc->busy[ch_index]);
		return -ENODEV;
	}

	pr_debug("%s: dst=%d, channel=%d, timeout=%d, ch_index = %d\n",
		 __func__, dst, msg->channel, timeout, ch_index);

	if (timeout == 0) {
		if (!mutex_trylock(&ch->rxlock)) {
			pr_err("dst=%d, channel=%d recv smsg busy!\n", dst,
			       msg->channel);
			atomic_dec(&ipc->busy[ch_index]);

			return -EBUSY;
		}

		/* no wait */
		if (SIPC_READL(ch->wrptr) == SIPC_READL(ch->rdptr)) {
			pr_info("dst=%d, channel=%d smsg rx cache is empty!\n",
				dst, msg->channel);

			rval = -ENODATA;

			goto recv_failed;
		}
	} else if (timeout < 0) {
		mutex_lock(&ch->rxlock);
		/* wait forever */
		rval = wait_event_interruptible(
			ch->rxwait,
			(SIPC_READL(ch->wrptr) != SIPC_READL(ch->rdptr)) ||
				(ipc->states[ch_index] == CHAN_STATE_FREE));
		if (rval < 0) {
			pr_debug("%s: dst=%d, channel=%d wait interrupted!\n",
				 __func__, dst, msg->channel);

			goto recv_failed;
		}

		if (ipc->states[ch_index] == CHAN_STATE_FREE) {
			pr_info("%s: dst=%d, channel=%d channel is free!\n",
				__func__, dst, msg->channel);

			rval = -EIO;

			goto recv_failed;
		}
	} else {
		mutex_lock(&ch->rxlock);
		/* wait timeout */
		rval = wait_event_interruptible_timeout(
			ch->rxwait,
			(SIPC_READL(ch->wrptr) != SIPC_READL(ch->rdptr)) ||
				(ipc->states[ch_index] == CHAN_STATE_FREE),
			timeout);
		if (rval < 0) {
			pr_debug("%s: dst=%d, channel=%d wait interrupted!\n",
				 __func__, dst, msg->channel);

			goto recv_failed;
		} else if (rval == 0) {
			pr_debug("%s: dst=%d, channel=%d wait timeout!\n",
				 __func__, dst, msg->channel);

			rval = -ETIME;

			goto recv_failed;
		}

		if (ipc->states[ch_index] == CHAN_STATE_FREE) {
			pr_info("%s: dst=%d, channel=%d channel is free!\n",
				__func__, dst, msg->channel);

			rval = -EIO;

			goto recv_failed;
		}
	}

	/* A completed receive returns zero; timeout wait returns remaining ticks. */
	rval = 0;
	/* read smsg from cache */
	rd = SIPC_READL(ch->rdptr) & (SMSG_CACHE_NR - 1);
	memcpy(msg, &ch->caches[rd], sizeof(struct smsg));
	SIPC_WRITEL(SIPC_READL(ch->rdptr) + 1, ch->rdptr);

	if (ipc->ring_base)
		pr_debug(
			"read smsg: dst=%d, channel=%d, wrptr=%d, rdptr=%d, rd=%d\n",
			dst, msg->channel, SIPC_READL(ch->wrptr),
			SIPC_READL(ch->rdptr), rd);

	pr_debug(
		"recv smsg: dst=%d, channel=%d, type=%d, flag=0x%04x, value=0x%08x, rval = %d\n",
		dst, msg->channel, msg->type, msg->flag, msg->value, rval);

recv_failed:
	mutex_unlock(&ch->rxlock);
	atomic_dec(&ipc->busy[ch_index]);
	return rval;
}
int smsg_recv(u8 dst, struct smsg *msg, int timeout)
{
	int ret;

	if (!msg)
		return -EINVAL;
	ret = smsg_uart_legacy_enter(dst, msg->channel, SMSG_UART_LEGACY_IO);
	if (ret)
		return ret;
	ret = smsg_recv_legacy(dst, msg, timeout);
	return smsg_uart_legacy_exit(dst, msg->channel, SMSG_UART_LEGACY_IO,
				     ret);
}
EXPORT_SYMBOL_GPL(smsg_recv);

u8 sipc_channel2index(u8 channel)
{
	return channel2index[channel];
}
EXPORT_SYMBOL_GPL(sipc_channel2index);

#if defined(CONFIG_DEBUG_FS)
static int smsg_debug_show(struct seq_file *m, void *private)
{
	struct smsg_ipc *ipc = NULL;
	struct smsg_channel *ch;

	int i, j, cnt, ch_index;

	for (i = 0; i < SIPC_ID_NR; i++) {
		ipc = smsg_ipcs[i];
		if (!ipc)
			continue;

		sipc_debug_putline(m, '*', 120);
		seq_printf(m, "sipc: %s:\n", ipc->name);
		seq_printf(m, "dst: 0x%0x, irq: 0x%0x\n", ipc->dst, ipc->irq);
		if (ipc->ring_base) {
			seq_printf(
				m,
				"txbufAddr: 0x%p, txbufsize: 0x%x, txbufrdptr: [0x%p]=%d, txbufwrptr: [0x%p]=%d\n",
				(void *)ipc->txbuf_addr, ipc->txbuf_size,
				(void *)ipc->txbuf_rdptr,
				SIPC_READL(ipc->txbuf_rdptr),
				(void *)ipc->txbuf_wrptr,
				SIPC_READL(ipc->txbuf_wrptr));
			seq_printf(
				m,
				"rxbufAddr: 0x%p, rxbufsize: 0x%x, rxbufrdptr: [0x%p]=%d, rxbufwrptr: [0x%p]=%d\n",
				(void *)ipc->rxbuf_addr, ipc->rxbuf_size,
				(void *)ipc->rxbuf_rdptr,
				SIPC_READL(ipc->rxbuf_rdptr),
				(void *)ipc->rxbuf_wrptr,
				SIPC_READL(ipc->rxbuf_wrptr));
		}
		sipc_debug_putline(m, '-', 80);
		seq_puts(m, "1. all channel state list:\n");

		for (j = 0; j < SMSG_VALID_CH_NR; j++)
			seq_printf(m,
				   "%2d. channel[%3d] states: %d, name: %s\n",
				   j, sipc_cfg[j].channel, ipc->states[j],
				   sipc_cfg[j].name);

		sipc_debug_putline(m, '-', 80);
		seq_puts(m, "2. channel rdpt < wrpt list:\n");

		cnt = 1;
		for (j = 0; j < SMSG_VALID_CH_NR; j++) {
			ch_index = channel2index[i];
			ch = ipc->channels[ch_index];
			if (!ch)
				continue;

			if (SIPC_READL(ch->rdptr) < SIPC_READL(ch->wrptr))
				seq_printf(
					m,
					"%2d. channel[%3d] rd: %d, wt: %d, name: %s\n",
					cnt++, sipc_cfg[j].channel,
					SIPC_READL(ch->rdptr),
					SIPC_READL(ch->wrptr),
					sipc_cfg[j].name);
		}
	}
	return 0;
}

static int smsg_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, smsg_debug_show, inode->i_private);
}

static const struct file_operations smsg_debug_fops = {
	.open = smsg_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int smsg_init_debugfs(void *root)
{
	if (!root)
		return -ENXIO;
	debugfs_create_file("smsg", 0444, (struct dentry *)root, NULL,
			    &smsg_debug_fops);
	return 0;
}
EXPORT_SYMBOL_GPL(smsg_init_debugfs);

#endif /* CONFIG_DEBUG_FS */

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SMSG driver");
MODULE_LICENSE("GPL");
