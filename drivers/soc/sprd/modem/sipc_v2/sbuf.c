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

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/overflow.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/refcount.h>
#include <linux/sipc.h>
#include <asm/pgtable.h>
#include <uapi/linux/sched/types.h>

#include "sipc_priv.h"
#include "sbuf.h"
#include "smsg_uart.h"

#if defined(CONFIG_DEBUG_FS)
#include "sipc_debugfs.h"
#endif


struct name_node {
	struct list_head list;
	char comm[TASK_COMM_LEN];
	pid_t pid;
	u8 latest;
};

union sbuf_buf {
	void *buf;
	void __user *ubuf;
};

enum task_type { TASK_RXWAIT = 0, TASK_TXWAIT, TASK_SELECT };

static struct sbuf_mgr *sbufs[SIPC_ID_NR][SMSG_VALID_CH_NR];

/* Separate creator lifetime: never published in the legacy sbufs[][] table.
 * The SMSG tuple is consumed for this boot, even after local release.
 */
struct sbuf_uart_backend {
	struct sbuf_mgr sbuf;
	struct smsg_uart_channel *channel;
	struct completion exited;
	wait_queue_head_t status_wait;
	struct mutex join_lock;
	unsigned long open_deadline;
	int result;
	bool started;
	bool joined;
};

/* One fixed boot-lifetime quarantine record. Physical SMEM is deliberately
 * not returned to its allocator, even after all AP objects have been joined.
 */
static struct {
	u32 smem_addr, smem_size, peer_addr;
	bool valid;
} sbuf_uart_quarantine;

/* UART-only opt-in ownership. The creator must serialize manager/endpoint
 * lifetime against attach/release; the retained pin also rejects destroy.
 * No UART callback or I/O path takes control_lock while holding state_lock.
 */
struct sbuf_uart_notifier {
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring;
	void (*notify)(int event, u32 generation, void *data);
	void *data;
	refcount_t users; /* owner + admitted callbacks/I/O/lease */
	wait_queue_head_t idle;
	struct list_head callbacks;
	struct task_struct *lease_task; /* state_lock; mutex owner task */
	bool published;
	bool canceled;
};

struct sbuf_uart_call {
	struct list_head node;
	struct task_struct *task;
	struct sbuf_uart_notifier *uart;
	u32 generation;
};

static int sbuf_uart_status_locked(struct sbuf_uart_notifier *uart,
				   u32 generation)
{
	struct sbuf_mgr *sbuf = uart->sbuf;

	lockdep_assert_held(&sbuf->state_lock);
	if (uart->canceled || sbuf->stopping || sbuf->uart_owner != uart)
		return -ESHUTDOWN;
	if (sbuf->state != SBUF_STATE_READY ||
	    generation != atomic_read(&sbuf->generation))
		return -EPIPE;
	return 0;
}

static bool sbuf_uart_call_get_locked(struct sbuf_mgr *sbuf,
				      struct sbuf_ring *ring, u32 generation,
				      struct sbuf_uart_call *call)
{
	struct sbuf_uart_notifier *uart = sbuf->uart_owner;

	lockdep_assert_held(&sbuf->state_lock);
	if (!uart || uart->ring != ring || !uart->published ||
	    sbuf_uart_status_locked(uart, generation))
		return false;
	call->uart = uart;
	call->generation = generation;
	call->task = current;
	list_add(&call->node, &uart->callbacks);
	refcount_inc(&uart->users);
	return true;
}

static void sbuf_uart_call(struct sbuf_uart_call *call, int event)
{
	struct sbuf_uart_notifier *uart = call->uart;
	struct sbuf_mgr *sbuf = uart->sbuf;
	unsigned long flags;

	/* Callback may run under control_lock: enqueue only, never wait/I/O. */
	uart->notify(event, call->generation, uart->data);
	spin_lock_irqsave(&sbuf->state_lock, flags);
	list_del(&call->node);
	refcount_dec(&uart->users);
	/* Wake before dropping state_lock; synchronize/release acquires it. */
	wake_up_all(&uart->idle);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
}

static void sbuf_uart_notify(struct sbuf_mgr *sbuf, struct sbuf_ring *ring,
			     int event, u32 generation)
{
	struct sbuf_uart_call call;
	unsigned long flags;
	bool acquired;

	spin_lock_irqsave(&sbuf->state_lock, flags);
	acquired = sbuf_uart_call_get_locked(sbuf, ring, generation, &call);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	if (acquired)
		sbuf_uart_call(&call, event);
}

static int sbuf_uart_io_get(struct sbuf_uart_notifier *uart, u32 generation)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&uart->sbuf->state_lock, flags);
	ret = sbuf_uart_status_locked(uart, generation);
	if (!ret)
		refcount_inc(&uart->users);
	spin_unlock_irqrestore(&uart->sbuf->state_lock, flags);
	return ret;
}

static void sbuf_uart_io_put(struct sbuf_uart_notifier *uart)
{
	struct sbuf_mgr *sbuf = uart->sbuf;
	unsigned long flags;

	spin_lock_irqsave(&sbuf->state_lock, flags);
	refcount_dec(&uart->users);
	wake_up_all(&uart->idle);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
}

static int sbuf_uart_commit(struct sbuf_uart_notifier *uart, u32 generation,
			    u32 *pointer, u32 count, bool tx)
{
	unsigned long flags;
	int ret;

	/* Pair the final check and pointer publication with UART reset/cancel. */
	spin_lock_irqsave(&uart->sbuf->state_lock, flags);
	ret = sbuf_uart_status_locked(uart, generation);
	if (!ret) {
		if (tx) {
			/* Publish payload before the peer sees the new TX pointer. */
			wmb();
		} else {
			/* Finish payload reads before releasing RX ring space. */
			mb();
		}
		WRITE_ONCE(*pointer, READ_ONCE(*pointer) + count);
	}
	spin_unlock_irqrestore(&uart->sbuf->state_lock, flags);
	return ret;
}

static struct task_struct *sbuf_wait_get_task(wait_queue_entry_t *pos,
					      u32 *b_select)
{
	*b_select = 0;
	if (pos->func == autoremove_wake_function ||
	    pos->func == default_wake_function)
		return pos->private;
	return NULL;
}

#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
static void sbuf_record_rdwt_owner(struct sbuf_ring *ring, int b_rx)
{
	int b_add;
	int cnt = 0;
	struct name_node *pos = NULL;
	struct name_node *temp = NULL;
	struct list_head *owner_list;
	unsigned long flags;

	b_add = 1;
	owner_list = b_rx ? (&ring->rx_list) : (&ring->tx_list);

	spin_lock_irqsave(&ring->rxwait.lock, flags);
	list_for_each_entry(pos, owner_list, list) {
		cnt++;
		if (pos->pid == current->pid) {
			b_add = 0;
			pos->latest = 1;
			continue;
		}
		if (pos->latest)
			pos->latest = 0;
	}
	spin_unlock_irqrestore(&ring->rxwait.lock, flags);

	if (b_add) {
		/* delete head next */
		if (cnt == MAX_RECORD_CNT) {
			temp = list_first_entry(owner_list, struct name_node,
						list);
			list_del(&temp->list);
			kfree(temp);
		}

		pos = kzalloc_obj(*pos, GFP_KERNEL);
		if (pos) {
			memcpy(pos->comm, current->comm, TASK_COMM_LEN);
			pos->pid = current->pid;
			pos->latest = 1;
			spin_lock_irqsave(&ring->rxwait.lock, flags);
			list_add_tail(&pos->list, owner_list);
			spin_unlock_irqrestore(&ring->rxwait.lock, flags);
		}
	}
}

static void sbuf_destroy_rdwt_owner(struct sbuf_ring *ring)
{
	struct name_node *pos, *temp;
	unsigned long flags;

	spin_lock_irqsave(&ring->rxwait.lock, flags);
	/* free task node */
	list_for_each_entry_safe(pos, temp, &ring->rx_list, list) {
		list_del(&pos->list);
		kfree(pos);
	}

	list_for_each_entry_safe(pos, temp, &ring->tx_list, list) {
		list_del(&pos->list);
		kfree(pos);
	}
	spin_unlock_irqrestore(&ring->rxwait.lock, flags);
}
#endif

/* SBUF EVENT messages are level notifications of the current shared pointers.
 * Keep two preallocated bits per ring so mailbox congestion cannot lose the
 * only wakeup after bytes have already been committed to shared memory.
 */
static void sbuf_notify_work(struct work_struct *work)
{
	struct sbuf_mgr *sbuf = container_of(to_delayed_work(work),
					     struct sbuf_mgr, notify_work);
	struct smsg msg;
	unsigned long flags, pending;
	u32 i, bit;
	int ret, generation;
	bool again = false, ready = false;
	u8 ack;

	mutex_lock(&sbuf->control_lock);
	if (READ_ONCE(sbuf->stopping))
		goto unlock;
	generation = atomic_read(&sbuf->generation);
	ack = sbuf->ack_pending;
	if (ack) {
		smsg_set(&msg, sbuf->channel, ack,
			 ack == SMSG_TYPE_OPEN ? SMSG_OPEN_MAGIC :
						 SMSG_CLOSE_MAGIC,
			 0);
		ret = smsg_send(sbuf->dst, &msg, 0);
		if (ret) {
			again = true;
			goto unlock;
		}
		sbuf->ack_pending = 0;
	}
	if (READ_ONCE(sbuf->init_pending)) {
		smsg_set(&msg, sbuf->channel, SMSG_TYPE_DONE,
			 SMSG_DONE_SBUF_INIT, sbuf->dst_smem_addr);
		ret = smsg_send(sbuf->dst, &msg, 0);
		spin_lock_irqsave(&sbuf->state_lock, flags);
		if (generation == atomic_read(&sbuf->generation) &&
		    sbuf->init_pending && !sbuf->stopping) {
			if (!ret) {
				sbuf->init_pending = false;
				WRITE_ONCE(sbuf->state, SBUF_STATE_READY);
				ready = true;
			} else {
				again = true;
			}
		}
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		if (ready) {
			blocking_notifier_call_chain(&sbuf->events,
						     SBUF_NOTIFY_READY, NULL);
			for (i = 0; i < sbuf->ringnr; i++) {
				struct sbuf_ring *ring = &sbuf->rings[i];

				wake_up_interruptible_all(&ring->txwait);
				wake_up_interruptible_all(&ring->rxwait);
				sbuf_uart_notify(sbuf, ring, SBUF_NOTIFY_READY,
						 generation);
				if (ring->handler)
					ring->handler(SBUF_NOTIFY_READY,
						      ring->data);
			}
		}
	}
	if (READ_ONCE(sbuf->state) == SBUF_STATE_READY) {
		for (i = 0; i < sbuf->ringnr; i++) {
			pending = xchg(&sbuf->rings[i].notify_pending, 0);
			for (bit = 0; bit < 2; bit++) {
				if (!(pending & BIT(bit)))
					continue;
				smsg_set(&msg, sbuf->channel, SMSG_TYPE_EVENT,
					 bit ? SMSG_EVENT_SBUF_RDPTR :
					       SMSG_EVENT_SBUF_WRPTR,
					 i);
				ret = smsg_send(sbuf->dst, &msg, 0);
				if (ret) {
					set_bit(bit,
						&sbuf->rings[i].notify_pending);
					again = true;
				}
			}
		}
	}

unlock:
	mutex_unlock(&sbuf->control_lock);
	if (again && !READ_ONCE(sbuf->stopping))
		mod_delayed_work(system_percpu_wq, &sbuf->notify_work,
				 msecs_to_jiffies(10));
}

static void sbuf_queue_notify(struct sbuf_mgr *sbuf, u32 ring, unsigned int bit)
{
	if (READ_ONCE(sbuf->stopping))
		return;
	set_bit(bit, &sbuf->rings[ring].notify_pending);
	/* Private UART thread services these bits within its finite RX wait. */
	if (sbuf->uart_backend)
		return;
	mod_delayed_work(system_percpu_wq, &sbuf->notify_work, 0);
}

/* A peer OPEN/CLOSE is the existing SBUF session transition, not a claim that
 * all modem hardware DMA is stopped. Fence AP ring accesses and discard only
 * the old session's bytes before acknowledging the peer's reinitialization.
 */
static void sbuf_reset_session(struct sbuf_mgr *sbuf)
{
	unsigned long flags;
	u32 i;

	mutex_lock(&sbuf->control_lock);
	spin_lock_irqsave(&sbuf->state_lock, flags);
	WRITE_ONCE(sbuf->state, SBUF_STATE_IDLE);
	sbuf->ack_pending = 0;
	sbuf->init_pending = false;
	atomic_inc(&sbuf->generation);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	mutex_unlock(&sbuf->control_lock);
	for (i = 0; i < sbuf->ringnr; i++) {
		wake_up_interruptible_all(&sbuf->rings[i].txwait);
		wake_up_interruptible_all(&sbuf->rings[i].rxwait);
	}
	blocking_notifier_call_chain(&sbuf->events, SBUF_NOTIFY_RESET, NULL);
	cancel_delayed_work_sync(&sbuf->notify_work);
	for (i = 0; i < sbuf->ringnr; i++) {
		struct sbuf_ring *ring = &sbuf->rings[i];
		struct sbuf_ring_header_op *hd = &ring->header_op;

		mutex_lock(&ring->txlock);
		mutex_lock(&ring->rxlock);
		ring->notify_pending = 0;
		/* Preserve the GPL recovery contract: discard old received bytes.
		 * Do not invent a TX rewind or modify a peer-owned pointer.
		 */
		WRITE_ONCE(*hd->rx_rd_p, READ_ONCE(*hd->rx_wt_p));
		/* Publish the RX discard before the subsequent OPEN/CLOSE ACK. */
		wmb();
		mutex_unlock(&ring->rxlock);
		mutex_unlock(&ring->txlock);
	}
}

static int sbuf_comm_init(struct sbuf_mgr *sbuf)
{
	u32 bufnum = sbuf->ringnr;
	int i;
	struct sbuf_ring *ring;

	for (i = 0; i < bufnum; i++) {
		ring = &sbuf->rings[i];
		init_waitqueue_head(&ring->txwait);
		init_waitqueue_head(&ring->rxwait);
#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
		INIT_LIST_HEAD(&ring->tx_list);
		INIT_LIST_HEAD(&ring->rx_list);
#endif
		mutex_init(&ring->txlock);
		mutex_init(&ring->rxlock);
		ring->need_wake_lock = 1;
		ring->tx_wakelock_state = 0;
		ring->rx_wakelock_state = 0;

		sprintf(ring->tx_wakelock_name, "sbuf-%d-%d-%d-tx", sbuf->dst,
			sbuf->channel, i);
		ring->tx_wake_lock =
			wakeup_source_register(NULL, ring->tx_wakelock_name);
		sprintf(ring->rx_wakelock_name, "sbuf-%d-%d-%d-rx", sbuf->dst,
			sbuf->channel, i);
		ring->rx_wake_lock =
			wakeup_source_register(NULL, ring->rx_wakelock_name);
		if (IS_ENABLED(CONFIG_PM_SLEEP) &&
		    (!ring->tx_wake_lock || !ring->rx_wake_lock))
			goto fail;
	}
	return 0;
fail:
	for (; i >= 0; i--) {
		ring = &sbuf->rings[i];
		wakeup_source_unregister(ring->tx_wake_lock);
		wakeup_source_unregister(ring->rx_wake_lock);
		ring->tx_wake_lock = NULL;
		ring->rx_wake_lock = NULL;
	}
	return -ENOMEM;
}

static int sbuf_host_init(struct smsg_ipc *sipc, struct sbuf_mgr *sbuf,
			  u32 bufnum, u32 txbufsize, u32 rxbufsize)
{
	struct sbuf_smem_header *smem;
	struct sbuf_ring_header *ringhd;
	struct sbuf_ring_header_op *hd_op;
	int hsize, i, ret;
	phys_addr_t offset = 0;
	u8 dst = sbuf->dst;
	struct sbuf_ring *ring;

	sbuf->ringnr = bufnum;

	/* allocate smem */
	if (!bufnum || bufnum > 256 || (!txbufsize && !rxbufsize) ||
	    txbufsize > INT_MAX || rxbufsize > INT_MAX ||
	    (u64)sizeof(struct sbuf_smem_header) +
			    ((u64)sizeof(struct sbuf_ring_header) + txbufsize +
			     rxbufsize) *
				    bufnum >
		    U32_MAX)
		return -EINVAL;
	hsize = sizeof(struct sbuf_smem_header) +
		sizeof(struct sbuf_ring_header) * bufnum;
	sbuf->smem_size = hsize + (txbufsize + rxbufsize) * bufnum;
	sbuf->smem_addr = smem_alloc(dst, sbuf->smem_size);
	if (!sbuf->smem_addr) {
		pr_err("%s: channel %d-%d, Failed to allocate smem for sbuf\n",
		       __func__, sbuf->dst, sbuf->channel);
		return -ENOMEM;
	}
	sbuf->dst_smem_addr =
		sbuf->smem_addr - sipc->smem_base + sipc->dst_smem_base;

	pr_debug(
		"%s: channel %d-%d, smem_addr=0x%x, smem_size=0x%x, dst_smem_addr=0x%x\n",
		__func__, sbuf->dst, sbuf->channel, sbuf->smem_addr,
		sbuf->smem_size, sbuf->dst_smem_addr);

#ifdef CONFIG_PHYS_ADDR_T_64BIT
	offset = sipc->high_offset;
	offset = offset << 32;
#endif

	pr_info("%s: channel %d-%d, offset = 0x%llx!\n", __func__, sbuf->dst,
		sbuf->channel, offset);
	sbuf->smem_virt = shmem_ram_vmap_nocache(dst, sbuf->smem_addr + offset,
						 sbuf->smem_size);
	if (!sbuf->smem_virt) {
		pr_err("%s: channel %d-%d, Failed to map smem for sbuf\n",
		       __func__, sbuf->dst, sbuf->channel);
		smem_free(dst, sbuf->smem_addr, sbuf->smem_size);
		return -EFAULT;
	}

	/* allocate rings description */
	sbuf->rings = kcalloc(bufnum, sizeof(struct sbuf_ring), GFP_KERNEL);
	if (!sbuf->rings) {
		smem_free(dst, sbuf->smem_addr, sbuf->smem_size);
		shmem_ram_unmap(dst, sbuf->smem_virt);
		return -ENOMEM;
	}

	/* initialize all ring bufs */
	smem = (struct sbuf_smem_header *)sbuf->smem_virt;
	WRITE_ONCE(smem->ringnr, bufnum);
	for (i = 0; i < bufnum; i++) {
		ringhd = (struct sbuf_ring_header *)&smem->headers[i];
		WRITE_ONCE(ringhd->txbuf_addr,
			   sbuf->dst_smem_addr + hsize +
				   (txbufsize + rxbufsize) * i);
		WRITE_ONCE(ringhd->txbuf_size, txbufsize);
		WRITE_ONCE(ringhd->txbuf_rdptr, 0);
		WRITE_ONCE(ringhd->txbuf_wrptr, 0);
		WRITE_ONCE(ringhd->rxbuf_addr,
			   READ_ONCE(smem->headers[i].txbuf_addr) + txbufsize);
		WRITE_ONCE(ringhd->rxbuf_size, rxbufsize);
		WRITE_ONCE(ringhd->rxbuf_rdptr, 0);
		WRITE_ONCE(ringhd->rxbuf_wrptr, 0);

		ring = &sbuf->rings[i];
		ring->header = ringhd;
		ring->txbuf_virt =
			sbuf->smem_virt + hsize + (txbufsize + rxbufsize) * i;
		ring->rxbuf_virt = ring->txbuf_virt + txbufsize;
		/* init header op */
		hd_op = &ring->header_op;
		hd_op->rx_rd_p = &ringhd->rxbuf_rdptr;
		hd_op->rx_wt_p = &ringhd->rxbuf_wrptr;
		hd_op->rx_size = READ_ONCE(ringhd->rxbuf_size);
		hd_op->tx_rd_p = &ringhd->txbuf_rdptr;
		hd_op->tx_wt_p = &ringhd->txbuf_wrptr;
		hd_op->tx_size = READ_ONCE(ringhd->txbuf_size);
	}

	ret = sbuf_comm_init(sbuf);
	if (ret) {
		kfree(sbuf->rings);
		sbuf->rings = NULL;
		shmem_ram_unmap(dst, sbuf->smem_virt);
		sbuf->smem_virt = NULL;
		smem_free(dst, sbuf->smem_addr, sbuf->smem_size);
	}
	return ret;
}

#define SBUF_UART_RETRY max_t(unsigned long, 1, msecs_to_jiffies(10))

static bool sbuf_uart_backpressure(int ret)
{
	return ret == -EAGAIN || ret == -EBUSY || ret == -ENOBUFS;
}

/* Cancel has no sleeping operation and never transmits CLOSE. It closes AP
 * admission; it does not stop an already admitted MMIO lease or the peer.
 */
void sbuf_uart_backend_cancel(struct sbuf_uart_backend *backend)
{
	struct sbuf_mgr *sbuf;
	unsigned long flags;

	if (!backend)
		return;
	sbuf = &backend->sbuf;
	spin_lock_irqsave(&sbuf->state_lock, flags);
	WRITE_ONCE(sbuf->stopping, true);
	if (sbuf->uart_owner) {
		sbuf->uart_owner->canceled = true;
		sbuf->uart_owner->published = false;
	}
	/* Pair task wake with join's last-reference transfer. */
	if (!backend->joined)
		wake_up_process(sbuf->thread);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	smsg_uart_cancel(backend->channel);
	wake_up_all(&backend->status_wait);
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_cancel);

static int sbuf_uart_send_until(struct sbuf_uart_backend *backend,
				const struct smsg *msg)
{
	int ret;

	for (;;) {
		if (READ_ONCE(backend->sbuf.stopping))
			return -ESHUTDOWN;
		if (time_after_eq(jiffies, backend->open_deadline))
			return -ETIMEDOUT;
		ret = smsg_uart_try_send(backend->channel, msg);
		if (!sbuf_uart_backpressure(ret))
			return ret;
		ret = smsg_uart_wait_retry(backend->channel,
					   backend->open_deadline,
					   SBUF_UART_RETRY);
		if (ret)
			return ret;
	}
}

/* At most two queued EVENT submissions, no sleeping mutex or transport wait.
 * The same pending bits/data-plane helper are used by legacy SBUF.
 */
static int sbuf_uart_flush_notify(struct sbuf_uart_backend *backend)
{
	struct sbuf_mgr *sbuf = &backend->sbuf;
	struct sbuf_ring *ring = &sbuf->rings[0];
	struct smsg msg;
	unsigned long pending;
	unsigned int bit;
	int ret = 0;

	if (!mutex_trylock(&sbuf->control_lock))
		return -EAGAIN;
	if (READ_ONCE(sbuf->stopping)) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	pending = xchg(&ring->notify_pending, 0);
	for (bit = 0; bit < 2; bit++) {
		if (!(pending & BIT(bit)))
			continue;
		smsg_set(&msg, sbuf->channel, SMSG_TYPE_EVENT,
			 bit ? SMSG_EVENT_SBUF_RDPTR : SMSG_EVENT_SBUF_WRPTR,
			 0);
		ret = smsg_uart_try_send(backend->channel, &msg);
		if (ret) {
			set_bit(bit, &ring->notify_pending);
			if (!sbuf_uart_backpressure(ret))
				break;
		}
	}
unlock:
	mutex_unlock(&sbuf->control_lock);
	return ret;
}

static int sbuf_uart_backend_thread(void *data)
{
	struct sbuf_uart_backend *backend = data;
	struct sbuf_mgr *sbuf = &backend->sbuf;
	struct sbuf_ring *ring = &sbuf->rings[0];
	struct smsg msg;
	struct sbuf_uart_call terminal;
	bool notify_stop;
	unsigned long flags;
	int ret;

	if (READ_ONCE(sbuf->stopping)) {
		ret = -ESHUTDOWN;
		goto exit;
	}
	smsg_set(&msg, sbuf->channel, SMSG_TYPE_OPEN, SMSG_OPEN_MAGIC, 0);
	ret = sbuf_uart_send_until(backend, &msg);
	if (ret)
		goto exit;
	/* Preserve the proven host OPEN -> peer OPEN -> CMD_INIT -> DONE order.
	 * One absolute budget covers all messages and transport retries.
	 */
	do {
		ret = smsg_uart_recv_until(backend->channel, &msg,
					   backend->open_deadline);
		if (ret)
			goto exit;
		if (msg.type == SMSG_TYPE_CLOSE) {
			ret = -EPIPE;
			goto exit;
		}
		cond_resched();
	} while (msg.type != SMSG_TYPE_OPEN || msg.flag != SMSG_OPEN_MAGIC);
	for (;;) {
		ret = smsg_uart_recv_until(backend->channel, &msg,
					   backend->open_deadline);
		if (ret)
			goto exit;
		if (msg.type == SMSG_TYPE_CMD && msg.flag == SMSG_CMD_SBUF_INIT)
			break;
		if (msg.type == SMSG_TYPE_OPEN || msg.type == SMSG_TYPE_CLOSE) {
			ret = -EPIPE;
			goto exit;
		}
		cond_resched();
	}
	/* The peer can learn the allocation asynchronously after this enqueue.
	 * release retains it even on timeout/cancel or later protocol failure.
	 */
	wmb();
	smsg_set(&msg, sbuf->channel, SMSG_TYPE_DONE, SMSG_DONE_SBUF_INIT,
		 sbuf->dst_smem_addr);
	ret = sbuf_uart_send_until(backend, &msg);
	if (ret)
		goto exit;
	/* No UART lease can be admitted before this first READY. */
	if (!mutex_trylock(&sbuf->control_lock)) {
		ret = -EBUSY;
		goto exit;
	}
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (sbuf->stopping || time_after_eq(jiffies, backend->open_deadline)) {
		ret = sbuf->stopping ? -ESHUTDOWN : -ETIMEDOUT;
	} else {
		WRITE_ONCE(sbuf->state, SBUF_STATE_READY);
		ret = 0;
	}
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	if (!ret)
		sbuf_uart_notify(sbuf, ring, SBUF_NOTIFY_READY,
				 atomic_read(&sbuf->generation));
	mutex_unlock(&sbuf->control_lock);
	wake_up_all(&backend->status_wait);
	if (ret)
		goto exit;

	while (!READ_ONCE(sbuf->stopping)) {
		ret = sbuf_uart_flush_notify(backend);
		if (ret && !sbuf_uart_backpressure(ret))
			goto exit;
		/* A bounded sleep also services locally queued pointer notices. */
		ret = smsg_uart_recv_until(backend->channel, &msg,
					   jiffies + SBUF_UART_RETRY);
		if (ret == -ETIMEDOUT)
			continue;
		if (ret)
			goto exit;
		/* Autonomous reset/reopen/close is not implemented or acknowledged.
		 * No generation relabel, ring rewind, or remote-silence inference.
		 */
		if (msg.type == SMSG_TYPE_OPEN || msg.type == SMSG_TYPE_CLOSE ||
		    msg.type == SMSG_TYPE_CMD) {
			ret = -EPIPE;
			goto exit;
		}
		if (msg.type == SMSG_TYPE_EVENT && !msg.value) {
			if (msg.flag == SMSG_EVENT_SBUF_RDPTR)
				sbuf_uart_notify(
					sbuf, ring, SBUF_NOTIFY_WRITE,
					atomic_read(&sbuf->generation));
			else if (msg.flag == SMSG_EVENT_SBUF_WRPTR)
				sbuf_uart_notify(
					sbuf, ring, SBUF_NOTIFY_READ,
					atomic_read(&sbuf->generation));
		}
		cond_resched();
	}
	ret = -ESHUTDOWN;
exit:
	/* Admit one terminal callback before closing publication. Its existing
	 * owner reference covers delivery after cancel and any concurrent detach.
	 * External cancel itself never invokes a callback or waits.
	 */
	spin_lock_irqsave(&sbuf->state_lock, flags);
	notify_stop = sbuf_uart_call_get_locked(
		sbuf, ring, atomic_read(&sbuf->generation), &terminal);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	sbuf_uart_backend_cancel(backend);
	if (notify_stop)
		sbuf_uart_call(&terminal, SBUF_UART_NOTIFY_STOP);
	WRITE_ONCE(backend->result, ret);
	wake_up_all(&backend->status_wait);
	/* The thread never modifies sbuf->thread. Completion is its last access
	 * to backend; the owner retains task_struct until successful join.
	 */
	kthread_complete_and_exit(&backend->exited, ret);
}

int sbuf_uart_backend_create(struct sbuf_uart_backend **out)
{
	struct sbuf_uart_backend *backend;
	struct sbuf_mgr *sbuf;
	struct smsg_ipc *sipc;
	int ret;
	u32 i;

	if (!out)
		return -EINVAL;
	*out = NULL;
	backend = kzalloc_obj(*backend, GFP_KERNEL);
	if (!backend)
		return -ENOMEM;
	ret = smsg_uart_reserve(&backend->channel);
	if (ret)
		goto free_backend;
	/* The caller pins the enclosing endpoint; the private reservation does
	 * not invent driver lifetime protection for the global lookup.
	 */
	sipc = smsg_ipcs[SIPC_ID_PM_SYS];
	if (!sipc || sipc->client) {
		ret = -EOPNOTSUPP;
		goto release_channel;
	}
	sbuf = &backend->sbuf;
	sbuf->dst = SIPC_ID_PM_SYS;
	sbuf->channel = SMSG_CH_COMM;
	sbuf->uart_backend = backend;
	spin_lock_init(&sbuf->state_lock);
	kref_init(&sbuf->ref);
	BLOCKING_INIT_NOTIFIER_HEAD(&sbuf->events);
	mutex_init(&sbuf->control_lock);
	atomic_set(&sbuf->generation, 0);
	init_completion(&backend->exited);
	init_waitqueue_head(&backend->status_wait);
	mutex_init(&backend->join_lock);
	backend->result = -EINPROGRESS;
	ret = sbuf_host_init(sipc, sbuf, 1, 512, 512);
	if (ret)
		goto release_channel;
	sbuf->thread = kthread_create(sbuf_uart_backend_thread, backend,
				      "sbuf-uart-6-1");
	if (IS_ERR(sbuf->thread)) {
		ret = PTR_ERR(sbuf->thread);
		for (i = 0; i < sbuf->ringnr; i++) {
			wakeup_source_unregister(sbuf->rings[i].tx_wake_lock);
			wakeup_source_unregister(sbuf->rings[i].rx_wake_lock);
		}
		kfree(sbuf->rings);
		shmem_ram_unmap(sbuf->dst, sbuf->smem_virt);
		/* No thread or transport ever saw this allocation. */
		smem_free(sbuf->dst, sbuf->smem_addr, sbuf->smem_size);
		goto release_channel;
	}
	get_task_struct(sbuf->thread);
	*out = backend;
	return 0;
release_channel:
	smsg_uart_release(backend->channel);
free_backend:
	kfree(backend);
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_create);

int sbuf_uart_backend_start(struct sbuf_uart_backend *backend,
			    unsigned long timeout)
{
	struct sbuf_mgr *sbuf;
	unsigned long flags;
	int ret = 0;

	if (!backend || !timeout || timeout > MAX_JIFFY_OFFSET)
		return -EINVAL;
	sbuf = &backend->sbuf;
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (sbuf->stopping)
		ret = -ESHUTDOWN;
	else if (backend->started)
		ret = -EALREADY;
	else {
		backend->open_deadline = jiffies + timeout;
		backend->started = true;
		wake_up_process(sbuf->thread);
	}
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_start);

int sbuf_uart_backend_wait_ready(struct sbuf_uart_backend *backend,
				 unsigned long timeout)
{
	long ret;

	if (!backend || !timeout || timeout > MAX_JIFFY_OFFSET)
		return -EINVAL;
	ret = wait_event_interruptible_timeout(
		backend->status_wait,
		READ_ONCE(backend->sbuf.stopping) ||
			READ_ONCE(backend->sbuf.state) == SBUF_STATE_READY,
		timeout);
	if (ret < 0)
		return ret;
	if (!ret)
		return -ETIMEDOUT;
	return READ_ONCE(backend->sbuf.stopping) ? -ESHUTDOWN : 0;
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_wait_ready);

int sbuf_uart_backend_join(struct sbuf_uart_backend *backend,
			   unsigned long timeout)
{
	struct sbuf_mgr *sbuf;
	struct task_struct *task;
	unsigned long flags;
	int ret = 0;

	if (!backend || !timeout || timeout > MAX_JIFFY_OFFSET)
		return -EINVAL;
	sbuf = &backend->sbuf;
	if (!mutex_trylock(&backend->join_lock))
		return -EBUSY;
	if (backend->joined)
		goto unlock;
	if (current == sbuf->thread) {
		ret = -EDEADLK;
		goto unlock;
	}
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (!sbuf->stopping || sbuf->uart_owner)
		ret = -EBUSY;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	if (ret)
		goto unlock;
	if (!wait_for_completion_timeout(&backend->exited, timeout)) {
		ret = -ETIMEDOUT;
		goto unlock;
	}
	spin_lock_irqsave(&sbuf->state_lock, flags);
	WRITE_ONCE(backend->joined, true);
	task = sbuf->thread;
	sbuf->thread = NULL;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	put_task_struct(task);
unlock:
	mutex_unlock(&backend->join_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_join);

int sbuf_uart_backend_release(struct sbuf_uart_backend *backend)
{
	struct sbuf_mgr *sbuf;
	unsigned long flags;
	int ret;

	if (!backend)
		return -EINVAL;
	sbuf = &backend->sbuf;
	/* Caller has joined all of its external API callers, not merely thread. */
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (!backend->joined || sbuf->uart_owner) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	ret = smsg_uart_release(backend->channel);
	if (ret)
		return ret;
	WRITE_ONCE(sbuf_uart_quarantine.smem_addr, sbuf->smem_addr);
	WRITE_ONCE(sbuf_uart_quarantine.smem_size, sbuf->smem_size);
	WRITE_ONCE(sbuf_uart_quarantine.peer_addr, sbuf->dst_smem_addr);
	WRITE_ONCE(sbuf_uart_quarantine.valid, true);
	wakeup_source_unregister(sbuf->rings[0].tx_wake_lock);
	wakeup_source_unregister(sbuf->rings[0].rx_wake_lock);
	shmem_ram_unmap(sbuf->dst, sbuf->smem_virt);
	kfree(sbuf->rings);
	/* No smem_free: queued DONE may still reach a live peer after cancel. */
	kfree(backend);
	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_release);

static int sbuf_client_init(struct smsg_ipc *sipc, struct sbuf_mgr *sbuf)
{
	struct sbuf_smem_header *smem;
	struct sbuf_ring_header *ringhd;
	struct sbuf_ring_header_op *hd_op;
	struct sbuf_ring *ring;
	int hsize, i, ret;
	u32 txbufsize, rxbufsize;
	phys_addr_t offset = 0;
	u32 bufnum;
	u8 dst = sbuf->dst;

#ifdef CONFIG_PHYS_ADDR_T_64BIT
	offset = sipc->high_offset;
	offset = offset << 32;
	pr_info("%s: channel %d-%d, offset = 0x%llx!\n", __func__, sbuf->dst,
		sbuf->channel, offset);
#endif

	/* get bufnum and bufsize */
	hsize = sizeof(struct sbuf_smem_header) +
		sizeof(struct sbuf_ring_header) * 1;
	sbuf->smem_virt =
		shmem_ram_vmap_nocache(dst, sbuf->smem_addr + offset, hsize);
	if (!sbuf->smem_virt) {
		pr_err("%s: channel %d-%d, Failed to map smem for sbuf head\n",
		       __func__, sbuf->dst, sbuf->channel);
		return -EFAULT;
	}
	smem = (struct sbuf_smem_header *)sbuf->smem_virt;
	sbuf->ringnr = READ_ONCE(smem->ringnr);
	bufnum = sbuf->ringnr;
	ringhd = (struct sbuf_ring_header *)&smem->headers[0];
	txbufsize = READ_ONCE(ringhd->rxbuf_size);
	rxbufsize = READ_ONCE(ringhd->txbuf_size);
	hsize = sizeof(struct sbuf_smem_header) +
		sizeof(struct sbuf_ring_header) * bufnum;
	sbuf->smem_size = hsize + (txbufsize + rxbufsize) * bufnum;
	pr_debug("%s: channel %d-%d, txbufsize = 0x%x, rxbufsize = 0x%x!\n",
		 __func__, sbuf->dst, sbuf->channel, txbufsize, rxbufsize);
	pr_debug("%s: channel %d-%d, smem_size = 0x%x, ringnr = %d!\n",
		 __func__, sbuf->dst, sbuf->channel, sbuf->smem_size, bufnum);
	shmem_ram_unmap(dst, sbuf->smem_virt);

	/* alloc debug smem */
	sbuf->smem_addr_debug = smem_alloc(dst, sbuf->smem_size);
	if (!sbuf->smem_addr_debug) {
		pr_err("%s: channel %d-%d,Failed to allocate debug smem for sbuf\n",
		       __func__, sbuf->dst, sbuf->channel);
		return -ENOMEM;
	}

	/* get smem virtual address */
	sbuf->smem_virt = shmem_ram_vmap_nocache(dst, sbuf->smem_addr + offset,
						 sbuf->smem_size);
	if (!sbuf->smem_virt) {
		pr_err("%s: channel %d-%d,Failed to map smem for sbuf\n",
		       __func__, sbuf->dst, sbuf->channel);
		smem_free(dst, sbuf->smem_addr_debug, sbuf->smem_size);
		return -EFAULT;
	}

	/* allocate rings description */
	sbuf->rings = kcalloc(bufnum, sizeof(struct sbuf_ring), GFP_KERNEL);
	if (!sbuf->rings) {
		smem_free(dst, sbuf->smem_addr_debug, sbuf->smem_size);
		shmem_ram_unmap(dst, sbuf->smem_virt);
		return -ENOMEM;
	}
	pr_info("%s: channel %d-%d, ringns = 0x%p!\n", __func__, sbuf->dst,
		sbuf->channel, sbuf->rings);

	/* initialize all ring bufs */
	smem = (struct sbuf_smem_header *)sbuf->smem_virt;
	for (i = 0; i < bufnum; i++) {
		ringhd = (struct sbuf_ring_header *)&smem->headers[i];
		ring = &sbuf->rings[i];
		ring->header = ringhd;
		/* host txbuf_addr */
		ring->rxbuf_virt =
			sbuf->smem_virt + hsize + (txbufsize + rxbufsize) * i;
		/* host rxbuf_addr */
		ring->txbuf_virt = ring->rxbuf_virt + rxbufsize;
		/* init header op , client mode, rx <==> tx */
		hd_op = &ring->header_op;
		hd_op->rx_rd_p = &ringhd->txbuf_rdptr;
		hd_op->rx_wt_p = &ringhd->txbuf_wrptr;
		hd_op->rx_size = READ_ONCE(ringhd->txbuf_size);
		hd_op->tx_rd_p = &ringhd->rxbuf_rdptr;
		hd_op->tx_wt_p = &ringhd->rxbuf_wrptr;
		hd_op->tx_size = READ_ONCE(ringhd->rxbuf_size);
	}

	ret = sbuf_comm_init(sbuf);
	if (ret) {
		kfree(sbuf->rings);
		sbuf->rings = NULL;
		shmem_ram_unmap(dst, sbuf->smem_virt);
		sbuf->smem_virt = NULL;
		smem_free(dst, sbuf->smem_addr_debug, sbuf->smem_size);
	}
	return ret;
}

static int sbuf_thread(void *data)
{
	struct sbuf_mgr *sbuf = data;
	struct sbuf_ring *ring;
	struct smsg mcmd, mrecv;
	int rval;
	struct smsg_ipc *sipc;

	/* since the channel open may hang, we call it in the sbuf thread */
	rval = smsg_ch_open(sbuf->dst, sbuf->channel, -1);
	if (rval != 0) {
		pr_err("Failed to open channel %d\n", sbuf->channel);
		/* assign NULL to thread poniter as failed to open channel */
		sbuf->thread = NULL;
		return rval;
	}

	/* if client, send SMSG_CMD_SBUF_INIT, wait sbuf SMSG_DONE_SBUF_INIT */
	sipc = smsg_ipcs[sbuf->dst];
	if (sipc->client) {
		unsigned long flags;

		smsg_set(&mcmd, sbuf->channel, SMSG_TYPE_CMD,
			 SMSG_CMD_SBUF_INIT, 0);
		smsg_send(sbuf->dst, &mcmd, -1);
		do {
			smsg_set(&mrecv, sbuf->channel, 0, 0, 0);
			rval = smsg_recv(sbuf->dst, &mrecv, -1);
			if (rval != 0) {
				sbuf->thread = NULL;
				return rval;
			}
		} while (mrecv.type != SMSG_TYPE_DONE ||
			 mrecv.flag != SMSG_DONE_SBUF_INIT);
		sbuf->smem_addr = mrecv.value;
		pr_info("%s: channel %d-%d, done_sbuf_init, address = 0x%x!\n",
			__func__, sbuf->dst, sbuf->channel, sbuf->smem_addr);
		if (sbuf_client_init(sipc, sbuf)) {
			sbuf->thread = NULL;
			return 0;
		}
		spin_lock_irqsave(&sbuf->state_lock, flags);
		WRITE_ONCE(sbuf->state, SBUF_STATE_READY);
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		blocking_notifier_call_chain(&sbuf->events, SBUF_NOTIFY_READY,
					     NULL);
	}

	/* sbuf init done, handle the ring rx events */
	while (!kthread_should_stop()) {
		/* monitor sbuf rdptr/wrptr update smsg */
		smsg_set(&mrecv, sbuf->channel, 0, 0, 0);
		rval = smsg_recv(sbuf->dst, &mrecv, -1);
		if (rval) {
			/* Never inspect a stale message after an interrupted receive. */
			if (kthread_should_stop())
				break;
			/* channel state is free */
			msleep(20);
			continue;
		}

		pr_debug(
			"sbuf thread recv msg: dst=%d, channel=%d, type=%d, flag=0x%04x, value=0x%08x\n",
			sbuf->dst, sbuf->channel, mrecv.type, mrecv.flag,
			mrecv.value);

		switch (mrecv.type) {
		case SMSG_TYPE_OPEN:
			if (mrecv.flag != SMSG_OPEN_MAGIC)
				break;
			pr_info("%s: channel %d-%d, state=%d, recv open msg!\n",
				__func__, sbuf->dst, sbuf->channel,
				sbuf->state);
			if (sipc->client)
				break;

			/* if channel state is already reay, reopen it
			 * (such as modem reset), we must skip the old
			 * buf data , than give open ack and reset state
			 * to idle
			 */
			sbuf_reset_session(sbuf);
			mutex_lock(&sbuf->control_lock);
			sbuf->ack_pending = SMSG_TYPE_OPEN;
			mutex_unlock(&sbuf->control_lock);
			mod_delayed_work(system_percpu_wq, &sbuf->notify_work,
					 0);
			break;
		case SMSG_TYPE_CLOSE:
			if (mrecv.flag != SMSG_CLOSE_MAGIC)
				break;
			/* handle channel close */
			sbuf_reset_session(sbuf);
			mutex_lock(&sbuf->control_lock);
			sbuf->ack_pending = SMSG_TYPE_CLOSE;
			mutex_unlock(&sbuf->control_lock);
			mod_delayed_work(system_percpu_wq, &sbuf->notify_work,
					 0);
			break;
		case SMSG_TYPE_CMD:
			pr_info("%s: channel %d-%d state = %d, recv cmd msg, flag = %d!\n",
				__func__, sbuf->dst, sbuf->channel, sbuf->state,
				mrecv.flag);
			if (sipc->client)
				break;

			/* respond cmd done for sbuf init only state is idle */
			if (READ_ONCE(sbuf->state) == SBUF_STATE_IDLE &&
			    mrecv.flag == SMSG_CMD_SBUF_INIT) {
				unsigned long flags;

				spin_lock_irqsave(&sbuf->state_lock, flags);
				if (sbuf->state == SBUF_STATE_IDLE &&
				    !sbuf->stopping)
					sbuf->init_pending = true;
				spin_unlock_irqrestore(&sbuf->state_lock,
						       flags);
				mod_delayed_work(system_percpu_wq,
						 &sbuf->notify_work, 0);
			}
			break;
		case SMSG_TYPE_EVENT:
			if (mrecv.value >= sbuf->ringnr) {
				pr_warn_ratelimited(
					"sbuf invalid ring %u on %u-%u\n",
					mrecv.value, sbuf->dst, sbuf->channel);
				break;
			}
			ring = &sbuf->rings[mrecv.value];
			switch (mrecv.flag) {
			case SMSG_EVENT_SBUF_RDPTR:
				wake_up_interruptible_all(&ring->txwait);
				sbuf_uart_notify(
					sbuf, ring, SBUF_NOTIFY_WRITE,
					atomic_read(&sbuf->generation));
				blocking_notifier_call_chain(&sbuf->events,
							     SBUF_NOTIFY_WRITE,
							     &mrecv.value);
				if (ring->handler)
					ring->handler(SBUF_NOTIFY_WRITE,
						      ring->data);
				__pm_wakeup_event(ring->tx_wake_lock,
						  jiffies_to_msecs(HZ / 2));
				ring->tx_wakelock_state = 1;
				pr_debug("sbuf %s : wake_lock hz/2!",
					 ring->tx_wakelock_name);
				break;
			case SMSG_EVENT_SBUF_WRPTR:
				wake_up_interruptible_all(&ring->rxwait);
				sbuf_uart_notify(
					sbuf, ring, SBUF_NOTIFY_READ,
					atomic_read(&sbuf->generation));
				blocking_notifier_call_chain(&sbuf->events,
							     SBUF_NOTIFY_READ,
							     &mrecv.value);
				if (ring->handler)
					ring->handler(SBUF_NOTIFY_READ,
						      ring->data);
				__pm_wakeup_event(ring->rx_wake_lock,
						  jiffies_to_msecs(HZ / 2));
				ring->rx_wakelock_state = 1;
				pr_debug("sbuf %s : wake_lock hz/2!",
					 ring->rx_wakelock_name);

				break;
			default:
				rval = 1;
				break;
			}
			break;
		default:
			rval = 1;
			break;
		};

		if (rval) {
			pr_info("non-handled sbuf msg: %d-%d, %d, %d, %d\n",
				sbuf->dst, sbuf->channel, mrecv.type,
				mrecv.flag, mrecv.value);
			rval = 0;
		}
		/* unlock sipc channel wake lock */
		smsg_ch_wake_unlock(sbuf->dst, sbuf->channel);
	}

	return 0;
}

static int sbuf_create_legacy(u8 dst, u8 channel, u32 bufnum, u32 txbufsize,
			      u32 rxbufsize)
{
	struct sbuf_mgr *sbuf;
	u8 ch_index;
	int ret, i;
	struct smsg_ipc *sipc = NULL;
	struct sched_param param = { .sched_priority = 10 };
	struct sbuf_ring *ring;

	sipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	pr_debug(
		"%s dst=%d, chanel=%d, bufnum=%d, txbufsize=0x%x, rxbufsize=0x%x\n",
		__func__, dst, channel, bufnum, txbufsize, rxbufsize);

	if (dst >= SIPC_ID_NR)
		return -EINVAL;
	if (!sipc_local_peer_supported(dst))
		return -EOPNOTSUPP;
	if (!sipc)
		return -EPROBE_DEFER;

	if (sbufs[dst][ch_index])
		return -EBUSY;
	sbuf = kzalloc_obj(*sbuf, GFP_KERNEL);
	if (!sbuf)
		return -ENOMEM;

	sbuf->state = SBUF_STATE_IDLE;
	atomic_set(&sbuf->generation, 0);
	spin_lock_init(&sbuf->state_lock);
	kref_init(&sbuf->ref);
	BLOCKING_INIT_NOTIFIER_HEAD(&sbuf->events);
	mutex_init(&sbuf->control_lock);
	INIT_DELAYED_WORK(&sbuf->notify_work, sbuf_notify_work);
	sbuf->dst = dst;
	sbuf->channel = channel;
	if (!sipc->client) {
		ret = sbuf_host_init(sipc, sbuf, bufnum, txbufsize, rxbufsize);
		if (ret) {
			kfree(sbuf);
			return ret;
		}
	}

	sbuf->thread =
		kthread_create(sbuf_thread, sbuf, "sbuf-%d-%d", dst, channel);
	if (IS_ERR(sbuf->thread)) {
		pr_err("Failed to create kthread: sbuf-%d-%d\n", dst, channel);
		if (!sipc->client) {
			for (i = 0; i < bufnum; i++) {
				ring = &sbuf->rings[i];
				wakeup_source_unregister(ring->tx_wake_lock);
				wakeup_source_unregister(ring->rx_wake_lock);
			}
			kfree(sbuf->rings);
			shmem_ram_unmap(dst, sbuf->smem_virt);
			smem_free(dst, sbuf->smem_addr, sbuf->smem_size);
		}
		ret = PTR_ERR(sbuf->thread);
		kfree(sbuf);
		return ret;
	}

	get_task_struct(sbuf->thread);
	sbufs[dst][ch_index] = sbuf;

	/*set the thread as a real time thread, and its priority is 10*/
	sched_setscheduler(sbuf->thread, SCHED_FIFO, &param);
	wake_up_process(sbuf->thread);

	return 0;
}
int sbuf_create(u8 dst, u8 channel, u32 bufnum, u32 txbufsize, u32 rxbufsize)
{
	int ret;

	ret = smsg_uart_legacy_sbuf_acquire(dst, channel);
	if (ret)
		return ret;
	ret = sbuf_create_legacy(dst, channel, bufnum, txbufsize, rxbufsize);
	if (ret)
		smsg_uart_legacy_sbuf_release(dst, channel);
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_create);

void sbuf_set_no_need_wake_lock(u8 dst, u8 channel, u32 bufnum)
{
	u8 ch_index;
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring = NULL;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return;
	}

	sbuf = dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL;
	if (!sbuf || sbuf->ringnr <= bufnum)
		return;

	ring = &sbuf->rings[bufnum];
	ring->need_wake_lock = 0;
}
EXPORT_SYMBOL_GPL(sbuf_set_no_need_wake_lock);

static void sbuf_manager_release(struct kref *ref)
{
	struct sbuf_mgr *sbuf = container_of(ref, struct sbuf_mgr, ref);

	kfree(sbuf->rings);
	kfree(sbuf);
}

struct sbuf_mgr *sbuf_get_manager(u8 dst, u8 channel)
{
	struct sbuf_mgr *sbuf = sbufs[dst][sipc_channel2index(channel)];

	kref_get(&sbuf->ref);
	return sbuf;
}

void sbuf_put_manager(struct sbuf_mgr *sbuf)
{
	kref_put(&sbuf->ref, sbuf_manager_release);
}

static int sbuf_session_status(struct sbuf_mgr *sbuf, u32 generation,
			       const bool *dead)
{
	if (READ_ONCE(sbuf->stopping))
		return -ENODEV;
	if ((dead && READ_ONCE(*dead)) ||
	    READ_ONCE(sbuf->state) != SBUF_STATE_READY ||
	    generation != atomic_read(&sbuf->generation))
		return -EPIPE;
	return 0;
}

int sbuf_session_state(struct sbuf_mgr *sbuf, u32 *generation)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&sbuf->state_lock, flags);
	*generation = atomic_read(&sbuf->generation);
	ret = sbuf_session_status(sbuf, *generation, NULL);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return ret;
}

/* Frontend registration rollback can revoke an opened cdev without changing
 * the peer's communication session. Serialize revocation with ring commits.
 */
void sbuf_session_cancel(struct sbuf_mgr *sbuf, u32 id, bool *dead)
{
	unsigned long flags;

	spin_lock_irqsave(&sbuf->state_lock, flags);
	WRITE_ONCE(*dead, true);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	wake_up_interruptible_all(&sbuf->rings[id].txwait);
	wake_up_interruptible_all(&sbuf->rings[id].rxwait);
}

static int sbuf_session_commit(struct sbuf_mgr *sbuf, u32 generation,
			       const bool *dead, u32 *pointer, u32 count,
			       bool tx)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&sbuf->state_lock, flags);
	ret = sbuf_session_status(sbuf, generation, dead);
	if (!ret) {
		if (tx)
			wmb();
		else
			mb();
		WRITE_ONCE(*pointer, READ_ONCE(*pointer) + count);
	}
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return ret;
}

void sbuf_destroy(u8 dst, u8 channel)
{
	struct sbuf_mgr *sbuf;
	int i;
	u8 ch_index;
	struct smsg_ipc *sipc;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return;
	}

	sbuf = dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL;
	if (!sbuf)
		return;

	spin_lock_irq(&sbuf->state_lock);
	/* A canceled subscription still pins the manager until final release.
	 * Refusal is a safety guard, not successful completion of this void API.
	 */
	if (WARN_ON_ONCE(sbuf->uart_owner)) {
		spin_unlock_irq(&sbuf->state_lock);
		return;
	}
	WRITE_ONCE(sbuf->stopping, true);
	spin_unlock_irq(&sbuf->state_lock);
	sbuf_reset_session(sbuf);
	smsg_ch_close(dst, channel, -1);

	/* stop sbuf thread if it's created successfully and still alive */
	if (!IS_ERR_OR_NULL(sbuf->thread)) {
		kthread_stop(sbuf->thread);
		put_task_struct(sbuf->thread);
	}
	/* The receive thread may have queued an ACK while it was being stopped. */
	cancel_delayed_work_sync(&sbuf->notify_work);

	if (sbuf->rings) {
		for (i = 0; i < sbuf->ringnr; i++) {
			wake_up_interruptible_all(&sbuf->rings[i].txwait);
			wake_up_interruptible_all(&sbuf->rings[i].rxwait);
#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
			sbuf_destroy_rdwt_owner(&sbuf->rings[i]);
#endif
			wakeup_source_unregister(sbuf->rings[i].tx_wake_lock);
			wakeup_source_unregister(sbuf->rings[i].rx_wake_lock);
		}
	}

	if (sbuf->smem_virt)
		shmem_ram_unmap(dst, sbuf->smem_virt);

	sipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	if (sipc->client)
		smem_free(dst, sbuf->smem_addr_debug, sbuf->smem_size);
	else
		smem_free(dst, sbuf->smem_addr, sbuf->smem_size);

	sbuf_put_manager(sbuf);

	sbufs[dst][ch_index] = NULL;
	smsg_uart_legacy_sbuf_release(dst, channel);
}
EXPORT_SYMBOL_GPL(sbuf_destroy);

static int sbuf_write_common(u8 dst, u8 channel, u32 bufid, void *buf, u32 len,
			     int timeout, bool user_buffer,
			     bool check_generation, u32 expected_generation,
			     struct sbuf_uart_notifier *uart,
			     struct sbuf_mgr *managed, const bool *dead)
{
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring = NULL;
	struct sbuf_ring_header_op *hd_op;
	void *txpos;
	int rval, left, tail, txsize;
	u8 ch_index;
	int generation;
	union sbuf_buf u_buf;

	if (len > INT_MAX || (!buf && len))
		return -EINVAL;
	if (!len)
		return 0;
	u_buf.buf = buf;
	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	sbuf = managed ? managed :
	       uart    ? uart->sbuf :
			 (dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL);
	if (!sbuf)
		return -ENODEV;
	/* Exclusive UART ownership also rejects mixed legacy data calls. */
	if (!uart && READ_ONCE(sbuf->uart_owner))
		return -EBUSY;
	generation = atomic_read(&sbuf->generation);
	if ((dead && READ_ONCE(*dead)) ||
	    (check_generation && generation != expected_generation))
		return -EPIPE;
	if (bufid >= sbuf->ringnr || !sbuf->rings)
		return -EINVAL;
	ring = &sbuf->rings[bufid];
	hd_op = &ring->header_op;
	if (!hd_op->tx_size)
		return -EOPNOTSUPP;
	if (READ_ONCE(sbuf->stopping))
		return -ENODEV;
	if (sbuf->state != SBUF_STATE_READY) {
		pr_info("sbuf-%d-%d not ready to write!\n", dst, channel);
		return check_generation ? -EPIPE : -ENODEV;
	}

	pr_debug("%s: dst=%d, channel=%d, bufid=%d, len=%d, timeout=%d\n",
		 __func__, dst, channel, bufid, len, timeout);

	rval = 0;
	left = len;

	if (timeout) {
		if (mutex_lock_interruptible(&ring->txlock))
			return -ERESTARTSYS;
	} else {
		if (!mutex_trylock(&ring->txlock)) {
			pr_debug(
				"sbuf_read busy, dst=%d, channel=%d, bufid=%d\n",
				dst, channel, bufid);
			return -EBUSY;
		}
	}

	/* reset marks the session dead before joining these locks. */
	rval = sbuf_session_status(sbuf, generation, dead);
	if (rval) {
		mutex_unlock(&ring->txlock);
		return rval;
	}

#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
	/* Legacy recorder allocates GFP_KERNEL; UART try calls never use it. */
	if (!uart)
		sbuf_record_rdwt_owner(ring, 0);
#endif

	if (timeout == 0) {
		/* no wait */
		if ((int)(READ_ONCE(*hd_op->tx_wt_p) -
			  READ_ONCE(*hd_op->tx_rd_p)) >= hd_op->tx_size) {
			pr_info("%s: %d-%d ring %d txbuf is full!\n", __func__,
				dst, channel, bufid);
			rval = -EBUSY;
		}
	} else if (timeout < 0) {
		/* wait forever */
		rval = wait_event_interruptible(
			ring->txwait,
			(int)(READ_ONCE(*hd_op->tx_wt_p) -
			      READ_ONCE(*hd_op->tx_rd_p)) < hd_op->tx_size ||
				((dead && READ_ONCE(*dead)) ||
				 READ_ONCE(sbuf->state) == SBUF_STATE_IDLE ||
				 (generation !=
				  atomic_read(&sbuf->generation))));
		if (rval < 0)
			pr_debug("%s: wait interrupted!\n", __func__);

		if (((dead && READ_ONCE(*dead)) ||
		     READ_ONCE(sbuf->state) == SBUF_STATE_IDLE ||
		     (generation != atomic_read(&sbuf->generation)))) {
			pr_err("%s: sbuf state is idle!\n", __func__);
			rval = -EPIPE;
		}
	} else {
		/* wait timeout */
		rval = wait_event_interruptible_timeout(
			ring->txwait,
			(int)(READ_ONCE(*hd_op->tx_wt_p) -
			      READ_ONCE(*hd_op->tx_rd_p)) < hd_op->tx_size ||
				((dead && READ_ONCE(*dead)) ||
				 READ_ONCE(sbuf->state) == SBUF_STATE_IDLE ||
				 (generation != atomic_read(&sbuf->generation))),
			timeout);
		if (rval < 0) {
			pr_debug("%s: wait interrupted!\n", __func__);
		} else if (rval == 0) {
			pr_info("%s: wait timeout!\n", __func__);
			rval = -ETIME;
		}

		if (((dead && READ_ONCE(*dead)) ||
		     READ_ONCE(sbuf->state) == SBUF_STATE_IDLE ||
		     (generation != atomic_read(&sbuf->generation)))) {
			pr_err("%s: sbuf state is idle!\n", __func__);
			rval = -EPIPE;
		}
	}

	while (rval >= 0 && left &&
	       (int)(READ_ONCE(*hd_op->tx_wt_p) - READ_ONCE(*hd_op->tx_rd_p)) <
		       hd_op->tx_size &&
	       ((!dead || !READ_ONCE(*dead)) &&
		READ_ONCE(sbuf->state) == SBUF_STATE_READY &&
		(generation == atomic_read(&sbuf->generation)))) {
		if ((u32)(READ_ONCE(*hd_op->tx_wt_p) -
			  READ_ONCE(*hd_op->tx_rd_p)) > hd_op->tx_size) {
			rval = -EIO;
			break;
		}
		/* calc txpos & txsize */
		txpos = ring->txbuf_virt +
			READ_ONCE(*hd_op->tx_wt_p) % hd_op->tx_size;
		txsize = hd_op->tx_size - (int)(READ_ONCE(*hd_op->tx_wt_p) -
						READ_ONCE(*hd_op->tx_rd_p));
		txsize = min(txsize, left);

		tail = txpos + txsize - (ring->txbuf_virt + hd_op->tx_size);
		if (tail > 0) {
			/* ring buffer is rounded */
			if (!user_buffer) {
				unalign_memcpy(txpos, u_buf.buf, txsize - tail);
				unalign_memcpy(ring->txbuf_virt,
					       u_buf.buf + txsize - tail, tail);
			} else {
				if (unalign_copy_from_user(txpos, u_buf.ubuf,
							   txsize - tail) ||
				    unalign_copy_from_user(
					    ring->txbuf_virt,
					    u_buf.ubuf + txsize - tail, tail)) {
					pr_err("%s:failed to copy from user!\n",
					       __func__);
					rval = -EFAULT;
					break;
				}
			}
		} else {
			if (!user_buffer) {
				unalign_memcpy(txpos, u_buf.buf, txsize);
			} else {
				/* handle the user space address */
				if (unalign_copy_from_user(txpos, u_buf.ubuf,
							   txsize)) {
					pr_err("%s:failed to copy from user!\n",
					       __func__);
					rval = -EFAULT;
					break;
				}
			}
		}

		pr_debug("%s: channel=%d, txpos=%p, txsize=%d\n", __func__,
			 channel, txpos, txsize);

		rval = uart ? sbuf_uart_commit(uart, generation, hd_op->tx_wt_p,
					       txsize, true) :
			      sbuf_session_commit(sbuf, generation, dead,
						  hd_op->tx_wt_p, txsize, true);
		if (rval)
			break;
		/* tx ringbuf is empty, so need to notify peer side */
		if (READ_ONCE(*hd_op->tx_wt_p) - READ_ONCE(*hd_op->tx_rd_p) ==
		    txsize)
			sbuf_queue_notify(sbuf, bufid, 0);

		left -= txsize;
		u_buf.buf += txsize;
	}

	/* must tx_wakelock_state is 1(means have got a long wake lock) ,
	 * we can do below actions, ortherwise system can't enter to sleep
	 */
	if (ring->tx_wakelock_state) {
		ring->tx_wakelock_state = 0;

		/* if need wake lock , update wake lock time to 20ms,
		 * else unlock it immediately
		 */
		if (ring->need_wake_lock) {
			pr_debug("sbuf %s : wake_lock hz/50!",
				 ring->tx_wakelock_name);
			__pm_wakeup_event(ring->tx_wake_lock,
					  jiffies_to_msecs(HZ / 50));
		} else {
			pr_debug("sbuf %s : wake_unlock!",
				 ring->tx_wakelock_name);
			__pm_relax(ring->tx_wake_lock);
		}
	}
	if (len == left && ((dead && READ_ONCE(*dead)) ||
			    generation != atomic_read(&sbuf->generation) ||
			    READ_ONCE(sbuf->state) != SBUF_STATE_READY))
		rval = -EPIPE;
	mutex_unlock(&ring->txlock);

	pr_debug("%s: done, channel=%d, len=%d\n", __func__, channel,
		 len - left);

	if (len == left)
		return rval;
	else
		return (len - left);
}
/* Kernel callers must supply kernel memory. User pointers use the explicit API. */
int sbuf_write(u8 dst, u8 channel, u32 bufid, void *buf, u32 len, int timeout)
{
	return sbuf_write_common(dst, channel, bufid, buf, len, timeout, false,
				 false, 0, NULL, NULL, NULL);
}
EXPORT_SYMBOL_GPL(sbuf_write);

int sbuf_write_user(u8 dst, u8 channel, u32 bufid, const void __user *buf,
		    u32 len, int timeout)
{
	return sbuf_write_common(dst, channel, bufid, (void __force *)buf, len,
				 timeout, true, false, 0, NULL, NULL, NULL);
}
EXPORT_SYMBOL_GPL(sbuf_write_user);

static int sbuf_read_common(u8 dst, u8 channel, u32 bufid, void *buf, u32 len,
			    int timeout, bool user_buffer,
			    bool check_generation, u32 expected_generation,
			    struct sbuf_uart_notifier *uart,
			    struct sbuf_mgr *managed, const bool *dead)
{
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring = NULL;
	struct sbuf_ring_header_op *hd_op;
	void *rxpos;
	int rval, left, tail, rxsize;
	u8 ch_index;
	int generation;
	union sbuf_buf u_buf;

	if (len > INT_MAX || (!buf && len))
		return -EINVAL;
	if (!len)
		return 0;
	u_buf.buf = buf;
	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}
	sbuf = managed ? managed :
	       uart    ? uart->sbuf :
			 (dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL);
	if (!sbuf)
		return -ENODEV;
	/* Exclusive UART ownership also rejects mixed legacy data calls. */
	if (!uart && READ_ONCE(sbuf->uart_owner))
		return -EBUSY;
	generation = atomic_read(&sbuf->generation);
	if ((dead && READ_ONCE(*dead)) ||
	    (check_generation && generation != expected_generation))
		return -EPIPE;
	if (bufid >= sbuf->ringnr || !sbuf->rings)
		return -EINVAL;
	ring = &sbuf->rings[bufid];
	hd_op = &ring->header_op;
	if (!hd_op->rx_size)
		return -EOPNOTSUPP;

	if (READ_ONCE(sbuf->stopping))
		return -ENODEV;
	if (sbuf->state != SBUF_STATE_READY) {
		pr_debug("sbuf-%d-%d not ready to read!\n", dst, channel);
		return check_generation ? -EPIPE : -ENODEV;
	}

	pr_debug("%s:dst=%d, channel=%d, bufid=%d, len=%d, timeout=%d\n",
		 __func__, dst, channel, bufid, len, timeout);

	rval = 0;
	left = len;

	if (timeout) {
		if (mutex_lock_interruptible(&ring->rxlock))
			return -ERESTARTSYS;
	} else {
		if (!mutex_trylock(&ring->rxlock)) {
			pr_debug("%s: busy!,dst=%d, channel=%d, bufid=%d\n",
				 __func__, dst, channel, bufid);
			return -EBUSY;
		}
	}

	/* reset marks the session dead before joining these locks. */
	rval = sbuf_session_status(sbuf, generation, dead);
	if (rval) {
		mutex_unlock(&ring->rxlock);
		return rval;
	}

#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
	/* Legacy recorder allocates GFP_KERNEL; UART try calls never use it. */
	if (!uart)
		sbuf_record_rdwt_owner(ring, 1);
#endif

	if (READ_ONCE(*hd_op->rx_wt_p) == READ_ONCE(*hd_op->rx_rd_p)) {
		if (timeout == 0) {
			/* no wait */
			pr_debug("%s: %d-%d ring %d rxbuf is empty!\n",
				 __func__, dst, channel, bufid);
			rval = -ENODATA;
		} else if (timeout < 0) {
			/* wait forever */
			rval = wait_event_interruptible(
				ring->rxwait,
				READ_ONCE(*hd_op->rx_wt_p) !=
						READ_ONCE(*hd_op->rx_rd_p) ||
					((dead && READ_ONCE(*dead)) ||
					 READ_ONCE(sbuf->state) ==
						 SBUF_STATE_IDLE ||
					 (generation !=
					  atomic_read(&sbuf->generation))));
			if (rval < 0)
				pr_debug("%s: wait interrupted!\n", __func__);

			if (((dead && READ_ONCE(*dead)) ||
			     READ_ONCE(sbuf->state) == SBUF_STATE_IDLE ||
			     (generation != atomic_read(&sbuf->generation)))) {
				pr_err("%s: sbuf state is idle!\n", __func__);
				rval = -EPIPE;
			}
		} else {
			/* wait timeout */
			rval = wait_event_interruptible_timeout(
				ring->rxwait,
				READ_ONCE(*hd_op->rx_wt_p) !=
						READ_ONCE(*hd_op->rx_rd_p) ||
					((dead && READ_ONCE(*dead)) ||
					 READ_ONCE(sbuf->state) ==
						 SBUF_STATE_IDLE ||
					 (generation !=
					  atomic_read(&sbuf->generation))),
				timeout);
			if (rval < 0) {
				pr_debug("%s: wait interrupted!\n", __func__);
			} else if (rval == 0) {
				pr_info("%s: wait timeout!\n", __func__);
				rval = -ETIME;
			}

			if (((dead && READ_ONCE(*dead)) ||
			     READ_ONCE(sbuf->state) == SBUF_STATE_IDLE ||
			     (generation != atomic_read(&sbuf->generation)))) {
				pr_err("%s: state is idle!\n", __func__);
				rval = -EPIPE;
			}
		}
	}

	while (rval >= 0 && left &&
	       (READ_ONCE(*hd_op->rx_wt_p) != READ_ONCE(*hd_op->rx_rd_p)) &&
	       ((!dead || !READ_ONCE(*dead)) &&
		READ_ONCE(sbuf->state) == SBUF_STATE_READY &&
		(generation == atomic_read(&sbuf->generation)))) {
		/* Observe the CP producer pointer before reading its payload. */
		rmb();
		/* calc rxpos & rxsize */
		rxpos = ring->rxbuf_virt +
			READ_ONCE(*hd_op->rx_rd_p) % hd_op->rx_size;
		rxsize = (int)(READ_ONCE(*hd_op->rx_wt_p) -
			       READ_ONCE(*hd_op->rx_rd_p));
		/* check overrun */
		if (rxsize < 0 || rxsize > hd_op->rx_size) {
			rval = -EIO;
			break;
		}
		if (rxsize > hd_op->rx_size)
			pr_err("%s: bufid = %d, channel= %d rxsize=0x%x, rdptr=%d, wrptr=%d",
			       __func__, bufid, channel, rxsize,
			       READ_ONCE(*hd_op->rx_wt_p),
			       READ_ONCE(*hd_op->rx_rd_p));

		rxsize = min(rxsize, left);

		pr_debug("%s: channel=%d, buf=%p, rxpos=%p, rxsize=%d\n",
			 __func__, channel, u_buf.buf, rxpos, rxsize);

		tail = rxpos + rxsize - (ring->rxbuf_virt + hd_op->rx_size);

		if (tail > 0) {
			/* ring buffer is rounded */
			if (!user_buffer) {
				unalign_memcpy(u_buf.buf, rxpos, rxsize - tail);
				unalign_memcpy(u_buf.buf + rxsize - tail,
					       ring->rxbuf_virt, tail);
			} else {
				/* handle the user space address */
				if (unalign_copy_to_user(u_buf.ubuf, rxpos,
							 rxsize - tail) ||
				    unalign_copy_to_user(
					    u_buf.ubuf + rxsize - tail,
					    ring->rxbuf_virt, tail)) {
					pr_err("%s: failed to copy to user!\n",
					       __func__);
					rval = -EFAULT;
					break;
				}
			}
		} else {
			if (!user_buffer) {
				unalign_memcpy(u_buf.buf, rxpos, rxsize);
			} else {
				/* handle the user space address */
				if (unalign_copy_to_user(u_buf.ubuf, rxpos,
							 rxsize)) {
					pr_err("%s: failed to copy to user!\n",
					       __func__);
					rval = -EFAULT;
					break;
				}
			}
		}

		rval = uart ? sbuf_uart_commit(uart, generation, hd_op->rx_rd_p,
					       rxsize, false) :
			      sbuf_session_commit(sbuf, generation, dead,
						  hd_op->rx_rd_p, rxsize,
						  false);
		if (rval)
			break;
		/* rx ringbuf is full ,so need to notify peer side */
		if (READ_ONCE(*hd_op->rx_wt_p) - READ_ONCE(*hd_op->rx_rd_p) ==
		    hd_op->rx_size - rxsize) {
			sbuf_queue_notify(sbuf, bufid, 1);
		}

		left -= rxsize;
		u_buf.buf += rxsize;
	}

	/* must rx_wakelock_state is 1(means have got a long wake lock) ,
	 * we can do below actions, ortherwise system can't enter to sleep
	 */
	if (ring->rx_wakelock_state) {
		ring->rx_wakelock_state = 0;

		/* if need wake lock , update wake lock time to 20ms,
		 * else unlock it immediately
		 */
		if (ring->need_wake_lock) {
			pr_debug("sbuf %s : wake_lock hz/50!",
				 ring->rx_wakelock_name);
			__pm_wakeup_event(ring->rx_wake_lock,
					  jiffies_to_msecs(HZ / 50));
		} else {
			pr_debug("sbuf %s : wake_unlock!",
				 ring->rx_wakelock_name);
			__pm_relax(ring->rx_wake_lock);
		}
	}

	if (len == left && ((dead && READ_ONCE(*dead)) ||
			    generation != atomic_read(&sbuf->generation) ||
			    READ_ONCE(sbuf->state) != SBUF_STATE_READY))
		rval = -EPIPE;
	mutex_unlock(&ring->rxlock);

	pr_debug("%s: done, channel=%d, len=%d", __func__, channel, len - left);

	if (len == left)
		return rval;
	else
		return (len - left);
}
/* Kernel callers must supply kernel memory. User pointers use the explicit API. */
int sbuf_read(u8 dst, u8 channel, u32 bufid, void *buf, u32 len, int timeout)
{
	return sbuf_read_common(dst, channel, bufid, buf, len, timeout, false,
				false, 0, NULL, NULL, NULL);
}
EXPORT_SYMBOL_GPL(sbuf_read);

int sbuf_read_user(u8 dst, u8 channel, u32 bufid, void __user *buf, u32 len,
		   int timeout)
{
	return sbuf_read_common(dst, channel, bufid, (void __force *)buf, len,
				timeout, true, false, 0, NULL, NULL, NULL);
}
EXPORT_SYMBOL_GPL(sbuf_read_user);

int sbuf_session_read(struct sbuf_mgr *sbuf, u32 generation, const bool *dead,
		      u32 ring, void *buf, u32 len, int timeout, bool user)
{
	return sbuf_read_common(sbuf->dst, sbuf->channel, ring, buf, len,
				timeout, user, true, generation, NULL, sbuf,
				dead);
}

int sbuf_session_write(struct sbuf_mgr *sbuf, u32 generation, const bool *dead,
		       u32 ring, const void *buf, u32 len, int timeout,
		       bool user)
{
	return sbuf_write_common(sbuf->dst, sbuf->channel, ring, (void *)buf,
				 len, timeout, user, true, generation, NULL,
				 sbuf, dead);
}

__poll_t sbuf_session_poll(struct sbuf_mgr *sbuf, u32 generation,
			   const bool *dead, u32 id, struct file *file,
			   poll_table *wait)
{
	struct sbuf_ring *ring = &sbuf->rings[id];
	struct sbuf_ring_header_op *hd = &ring->header_op;
	unsigned long flags;
	__poll_t mask = 0;

	poll_wait(file, &ring->rxwait, wait);
	poll_wait(file, &ring->txwait, wait);
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (sbuf_session_status(sbuf, generation, dead)) {
		mask = EPOLLERR | EPOLLHUP;
	} else {
		if (READ_ONCE(*hd->rx_wt_p) != READ_ONCE(*hd->rx_rd_p))
			mask |= EPOLLIN | EPOLLRDNORM;
		if (READ_ONCE(*hd->tx_wt_p) - READ_ONCE(*hd->tx_rd_p) <
		    hd->tx_size)
			mask |= EPOLLOUT | EPOLLWRNORM;
	}
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return mask;
}

int sbuf_get_session(u8 dst, u8 channel, u32 *generation, u32 *state)
{
	struct sbuf_mgr *sbuf;
	u8 index = sipc_channel2index(channel);
	unsigned long flags;

	if (!generation || !state || dst >= SIPC_ID_NR ||
	    index >= SMSG_VALID_CH_NR)
		return -EINVAL;
	sbuf = sbufs[dst][index];
	if (!sbuf)
		return -ENODEV;
	/*
	 * The owning SPIPE device removes its sysfs nodes before sbuf_destroy.
	 * Snapshot only software state, without shared-memory or mailbox reads.
	 */
	spin_lock_irqsave(&sbuf->state_lock, flags);
	*generation = atomic_read(&sbuf->generation);
	*state = sbuf->stopping ? 2 : sbuf->state;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_get_session);

int sbuf_poll_wait(u8 dst, u8 channel, u32 bufid, struct file *filp,
		   poll_table *wait)
{
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring = NULL;
	struct sbuf_ring_header_op *hd_op;
	unsigned int mask = 0;
	u8 ch_index;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return mask;
	}
	sbuf = dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL;
	if (!sbuf || !sbuf->rings || bufid >= sbuf->ringnr)
		return POLLERR | POLLHUP;
	ring = &sbuf->rings[bufid];
	hd_op = &ring->header_op;
	poll_wait(filp, &ring->txwait, wait);
	poll_wait(filp, &ring->rxwait, wait);
	if (READ_ONCE(sbuf->state) != SBUF_STATE_READY)
		return POLLERR | POLLHUP;

	if (READ_ONCE(*hd_op->rx_wt_p) != READ_ONCE(*hd_op->rx_rd_p))
		mask |= POLLIN | POLLRDNORM;

	if (READ_ONCE(*hd_op->tx_wt_p) - READ_ONCE(*hd_op->tx_rd_p) <
	    hd_op->tx_size)
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}
EXPORT_SYMBOL_GPL(sbuf_poll_wait);

int sbuf_status(u8 dst, u8 channel)
{
	struct sbuf_mgr *sbuf;
	u8 ch_index;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}
	sbuf = dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL;

	if (!sbuf)
		return -ENODEV;
	if (sbuf->state != SBUF_STATE_READY)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_status);

int sbuf_register_notifier(u8 dst, u8 channel, u32 bufid,
			   void (*handler)(int event, void *data), void *data)
{
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring = NULL;
	u8 ch_index;
	unsigned long flags;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}
	sbuf = dst < SIPC_ID_NR ? sbufs[dst][ch_index] : NULL;
	if (!sbuf)
		return -ENODEV;
	if (bufid >= sbuf->ringnr || !sbuf->rings)
		return -EINVAL;
	ring = &sbuf->rings[bufid];
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (sbuf->uart_owner && sbuf->uart_owner->ring == ring) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		return -EBUSY;
	}
	ring->handler = handler;
	ring->data = data;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);

	if (handler && sbuf->state == SBUF_STATE_READY)
		handler(SBUF_NOTIFY_READ, data);

	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_register_notifier);

/* The caller exclusively owns a successful sbuf_create(6, 1, 1, 512, 512)
 * and the enclosing SIPC endpoint throughout this handle's lifetime.
 */
static int
sbuf_uart_register_common(struct sbuf_mgr *managed, u8 dst, u8 channel,
			  u32 bufid,
			  void (*notify)(int event, u32 generation, void *data),
			  void *data, struct sbuf_uart_notifier **out)
{
	struct sbuf_uart_notifier *uart;
	struct sbuf_uart_call call;
	struct sbuf_mgr *sbuf;
	struct sbuf_ring *ring;
	unsigned long flags;
	bool acquired;
	int ret = 0;
	u8 index;

	if (!out)
		return -EINVAL;
	*out = NULL;
	if (!notify)
		return -EINVAL;
	if (dst != SIPC_ID_PM_SYS || channel != SMSG_CH_COMM || bufid)
		return -EOPNOTSUPP;
	index = sipc_channel2index(channel);
	if (index == INVALID_CHANEL_INDEX)
		return -EINVAL;
	/* Lookup itself requires the documented exclusive creator lifetime. */
	sbuf = managed ? managed : sbufs[dst][index];
	if (!sbuf)
		return -ENODEV;
	if (!sbuf->rings)
		return -EPROBE_DEFER;
	if (sbuf->ringnr != 1)
		return -EINVAL;
	ring = &sbuf->rings[0];
	if (ring->header_op.tx_size != 512 || ring->header_op.rx_size != 512)
		return -EINVAL;
	uart = kzalloc_obj(*uart, GFP_KERNEL);
	if (!uart)
		return -ENOMEM;
	uart->sbuf = sbuf;
	uart->ring = ring;
	uart->notify = notify;
	uart->data = data;
	refcount_set(&uart->users, 1);
	init_waitqueue_head(&uart->idle);
	INIT_LIST_HEAD(&uart->callbacks);

	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (sbuf->stopping)
		ret = -ESHUTDOWN;
	else if (sbuf->uart_owner || ring->handler)
		ret = -EBUSY;
	if (ret) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		kfree(uart);
		return ret;
	}
	/* Publish out first so even a synchronous callback may request cancel.
	 * READY decision, reference and generation are one locked operation.
	 */
	*out = uart;
	uart->published = true;
	sbuf->uart_owner = uart;
	acquired = sbuf_uart_call_get_locked(
		sbuf, ring, atomic_read(&sbuf->generation), &call);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	if (acquired)
		sbuf_uart_call(&call, SBUF_NOTIFY_READ);
	return 0;
}
int sbuf_uart_register(u8 dst, u8 channel, u32 bufid,
		       void (*notify)(int event, u32 generation, void *data),
		       void *data, struct sbuf_uart_notifier **out)
{
	return sbuf_uart_register_common(NULL, dst, channel, bufid, notify,
					 data, out);
}
EXPORT_SYMBOL_GPL(sbuf_uart_register);

int sbuf_uart_backend_register(struct sbuf_uart_backend *backend,
			       void (*notify)(int event, u32 generation,
					      void *data),
			       void *data, struct sbuf_uart_notifier **out)
{
	if (!backend) {
		if (out)
			*out = NULL;
		return -EINVAL;
	}
	return sbuf_uart_register_common(&backend->sbuf, SIPC_ID_PM_SYS,
					 SMSG_CH_COMM, 0, notify, data, out);
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_register);

int sbuf_uart_backend_get_generation(struct sbuf_uart_backend *backend,
				     u32 *generation)
{
	struct sbuf_mgr *sbuf;
	unsigned long flags;
	int ret = 0;

	if (!backend || !generation)
		return -EINVAL;
	sbuf = &backend->sbuf;
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (sbuf->stopping)
		ret = -ESHUTDOWN;
	else if (sbuf->state != SBUF_STATE_READY)
		ret = -EPIPE;
	else
		*generation = atomic_read(&sbuf->generation);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_uart_backend_get_generation);

void sbuf_uart_cancel(struct sbuf_uart_notifier *uart)
{
	unsigned long flags;

	if (!uart)
		return;
	spin_lock_irqsave(&uart->sbuf->state_lock, flags);
	/* Retain uart_owner: unpublishing must not permit manager destruction. */
	uart->published = false;
	uart->canceled = true;
	spin_unlock_irqrestore(&uart->sbuf->state_lock, flags);
}
EXPORT_SYMBOL_GPL(sbuf_uart_cancel);

int sbuf_uart_synchronize(struct sbuf_uart_notifier *uart)
{
	struct sbuf_uart_call *call;
	struct sbuf_mgr *sbuf;
	unsigned long flags;

	if (!uart)
		return -EINVAL;
	sbuf = uart->sbuf;
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (uart->lease_task == current) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		return -EDEADLK;
	}
	if (!uart->canceled) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		return -EBUSY;
	}
	list_for_each_entry(call, &uart->callbacks, node) {
		if (call->task == current) {
			spin_unlock_irqrestore(&sbuf->state_lock, flags);
			return -EDEADLK;
		}
	}
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	might_sleep();
	wait_event(uart->idle, refcount_read(&uart->users) == 1);
	/* Pair with last put's locked wake before permitting owner release. */
	spin_lock_irqsave(&sbuf->state_lock, flags);
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_uart_synchronize);

int sbuf_uart_unregister(struct sbuf_uart_notifier *uart)
{
	struct sbuf_mgr *sbuf;
	unsigned long flags;
	int ret;

	if (!uart)
		return -EINVAL;
	sbuf = uart->sbuf;
	sbuf_uart_cancel(uart);
	ret = sbuf_uart_synchronize(uart);
	if (ret)
		return ret;
	/* Caller has also joined queued workers and all external API callers.
	 * This barrier accounts admitted backend calls, not caller-owned work.
	 */
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (WARN_ON_ONCE(sbuf->uart_owner != uart ||
			 refcount_read(&uart->users) != 1)) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		return -EBUSY;
	}
	sbuf->uart_owner = NULL;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	WARN_ON_ONCE(!refcount_dec_and_test(&uart->users));
	kfree(uart);
	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_uart_unregister);

/* Admit a bounded process-context transaction against AP session reset.
 * This mutex cannot fence remote hardware activity before its reset message.
 */
int sbuf_uart_lease_try_begin(struct sbuf_uart_notifier *uart, u32 generation)
{
	struct sbuf_uart_call *call;
	struct sbuf_mgr *sbuf;
	unsigned long flags;
	int ret = 0;

	if (!uart)
		return -EINVAL;
	might_sleep(); /* Process context only; mutex acquisition never waits. */
	sbuf = uart->sbuf;
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (uart->lease_task == current) {
		ret = -EDEADLK;
		goto unlock;
	}
	/* READY callbacks may already own control_lock. All callbacks enqueue. */
	list_for_each_entry(call, &uart->callbacks, node) {
		if (call->task == current) {
			ret = -EDEADLK;
			goto unlock;
		}
	}
	ret = sbuf_uart_status_locked(uart, generation);
	if (!ret)
		refcount_inc(&uart->users);
unlock:
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	if (ret)
		return ret;

	if (!mutex_trylock(&sbuf->control_lock)) {
		sbuf_uart_io_put(uart);
		return -EAGAIN;
	}
	/* Reset/cancel may have won after the reference was admitted. */
	spin_lock_irqsave(&sbuf->state_lock, flags);
	ret = sbuf_uart_status_locked(uart, generation);
	if (!ret && WARN_ON_ONCE(uart->lease_task))
		ret = -EBUSY;
	if (!ret)
		uart->lease_task = current;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	if (ret) {
		mutex_unlock(&sbuf->control_lock);
		sbuf_uart_io_put(uart);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_uart_lease_try_begin);

int sbuf_uart_lease_end(struct sbuf_uart_notifier *uart)
{
	struct sbuf_mgr *sbuf;
	unsigned long flags;

	if (!uart)
		return -EINVAL;
	might_sleep(); /* Drop the UART port spinlock before ending the lease. */
	sbuf = uart->sbuf;
	spin_lock_irqsave(&sbuf->state_lock, flags);
	if (uart->lease_task != current) {
		spin_unlock_irqrestore(&sbuf->state_lock, flags);
		return -EPERM;
	}
	uart->lease_task = NULL;
	spin_unlock_irqrestore(&sbuf->state_lock, flags);
	/* A synchronizer can release the owner after the last put: unlock first. */
	mutex_unlock(&sbuf->control_lock);
	sbuf_uart_io_put(uart);
	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_uart_lease_end);

int sbuf_uart_try_read(struct sbuf_uart_notifier *uart, u32 generation,
		       void *buf, u32 len)
{
	int ret;

	if (!uart || !buf || !len || len > 128)
		return -EINVAL;
	might_sleep(); /* Process context, but no transport/mutex wait. */
	ret = sbuf_uart_io_get(uart, generation);
	if (ret)
		return ret;
	ret = sbuf_read_common(uart->sbuf->dst, uart->sbuf->channel, 0, buf,
			       len, 0, false, true, generation, uart, NULL,
			       NULL);
	sbuf_uart_io_put(uart);
	return ret;
}
EXPORT_SYMBOL_GPL(sbuf_uart_try_read);

int sbuf_uart_try_ack(struct sbuf_uart_notifier *uart, u32 generation)
{
	u8 ack = 0x01;
	int ret;

	if (!uart)
		return -EINVAL;
	might_sleep(); /* Caller must not hold the UART port spinlock. */
	ret = sbuf_uart_io_get(uart, generation);
	if (ret)
		return ret;
	ret = sbuf_write_common(uart->sbuf->dst, uart->sbuf->channel, 0, &ack,
				1, 0, false, true, generation, uart, NULL,
				NULL);
	sbuf_uart_io_put(uart);
	return ret; /* 1 committed, <0 not committed; never automatically retry. */
}
EXPORT_SYMBOL_GPL(sbuf_uart_try_ack);

void sbuf_get_status(u8 dst, char *status_info, int size)
{
	struct sbuf_mgr *sbuf = NULL;
	struct sbuf_ring *ring = NULL;
	struct sbuf_ring_header_op *hd_op;
	wait_queue_entry_t *pos;
	struct task_struct *task;
	unsigned long flags;
	int i, n, len, cnt;
	u32 b_select;
	char *phead;
#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
	struct name_node *node = NULL;
#endif

	if (!status_info || size < 0 || dst >= SIPC_ID_NR)
		return;
	len = strlen(status_info);

	for (i = 0; i < SMSG_VALID_CH_NR; i++) {
		sbuf = sbufs[dst][i];
		if (!sbuf)
			continue;

		for (n = 0; n < sbuf->ringnr && len < size; n++) {
			ring = &sbuf->rings[n];
			hd_op = &ring->header_op;

			if ((READ_ONCE(*hd_op->rx_wt_p) -
			     READ_ONCE(*hd_op->rx_rd_p)) < hd_op->rx_size)
				continue;

			snprintf(status_info + len, size - len,
				 "ch-%d-ring-%d is full.\n", sbuf->channel, n);
			len = strlen(status_info);

			/*  show all rxwait task */
			spin_lock_irqsave(&ring->rxwait.lock, flags);
			cnt = 0;
			list_for_each_entry(pos, &ring->rxwait.head, entry) {
				task = sbuf_wait_get_task(pos, &b_select);
				if (!task)
					continue;

				if (b_select)
					phead = "rxwait task";
				else
					phead = "select task";

				snprintf(
					status_info + len, size - len,
					"%s %d: %s, state=0x%lx, pid=%d.\n",
					phead, cnt, task->comm,
					(unsigned long)READ_ONCE(task->__state),
					task->pid);
				cnt++;
				len = strlen(status_info);
			}
			spin_unlock_irqrestore(&ring->rxwait.lock, flags);

			/* only show the latest ever read task */
#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
			spin_lock_irqsave(&ring->rxwait.lock, flags);
			list_for_each_entry(node, &ring->rx_list, list) {
				if (node->latest) {
					snprintf(status_info + len, size - len,
						 "read task: %s, pid = %d.\n",
						 node->comm, node->pid);
					break;
				}
			}
			spin_unlock_irqrestore(&ring->rxwait.lock, flags);
#endif
		}
	}
}
EXPORT_SYMBOL_GPL(sbuf_get_status);

#if defined(CONFIG_DEBUG_FS)
static void sbuf_debug_task_show(struct seq_file *m, struct sbuf_mgr *sbuf,
				 int task_type)
{
	int n, cnt;
	u32 b_select;
	unsigned long flags;
	struct sbuf_ring *ring = NULL;
	wait_queue_head_t *phead;
	char *buf;
	wait_queue_entry_t *pos;
	struct task_struct *task;

	for (n = 0; n < sbuf->ringnr; n++) {
		ring = &sbuf->rings[n];
		cnt = 0;

		if (task_type == TASK_RXWAIT) {
			phead = &ring->rxwait;
			buf = "rxwait task";
		} else if (task_type == TASK_TXWAIT) {
			phead = &ring->txwait;
			buf = "txwait task";
		} else {
			phead = &ring->rxwait;
			buf = "select task";
		}

		spin_lock_irqsave(&phead->lock, flags);
		list_for_each_entry(pos, &phead->head, entry) {
			task = sbuf_wait_get_task(pos, &b_select);
			if (!task)
				continue;

			if (b_select && (task_type != TASK_SELECT))
				continue;

			seq_printf(m, "  ring[%2d]: %s %d ", n, buf, cnt);
			seq_printf(m, ": %s, state = 0x%lx, pid = %d\n",
				   task->comm,
				   (unsigned long)READ_ONCE(task->__state),
				   task->pid);
			cnt++;
		}
		spin_unlock_irqrestore(&phead->lock, flags);
	}
}

#if defined(SIPC_DEBUG_SBUF_RDWT_OWNER)
static void sbuf_debug_list_show(struct seq_file *m, struct sbuf_mgr *sbuf,
				 int b_rx)
{
	int n, cnt;
	struct sbuf_ring *ring = NULL;
	struct list_head *plist;
	char *buf;
	struct name_node *node = NULL;
	unsigned long flags;

	/* list all  sbuf task list */
	for (n = 0; n < sbuf->ringnr; n++) {
		ring = &sbuf->rings[n];
		cnt = 0;

		if (b_rx) {
			plist = &ring->rx_list;
			buf = "read task";
		} else {
			plist = &ring->tx_list;
			buf = "write task";
		}

		spin_lock_irqsave(&ring->rxwait.lock, flags);
		list_for_each_entry(node, plist, list) {
			seq_printf(
				m,
				"  ring[%2d]: %s %d : %s, pid = %d, latest = %d\n",
				n, buf, cnt, node->comm, node->pid,
				node->latest);
			cnt++;
		}
		spin_unlock_irqrestore(&ring->rxwait.lock, flags);
	}
}
#endif

static int sbuf_debug_show(struct seq_file *m, void *private)
{
	struct sbuf_mgr *sbuf = NULL;
	struct sbuf_ring *ring = NULL;
	struct sbuf_ring_header_op *hd_op;
	int i, j, n, cnt;
	struct smsg_ipc *sipc = NULL;

	for (i = 0; i < SIPC_ID_NR; i++) {
		sipc = smsg_ipcs[i];
		if (!sipc)
			continue;
		sipc_debug_putline(m, '*', 120);
		seq_printf(m, "dst: 0x%0x, sipc: %s:\n", i, sipc->name);
		sipc_debug_putline(m, '*', 120);

		for (j = 0; j < SMSG_VALID_CH_NR; j++) {
			sbuf = sbufs[i][j];
			if (!sbuf)
				continue;
			/* list a sbuf channel */
			sipc_debug_putline(m, '-', 100);
			seq_printf(m, "sbuf_%d_%03d, state: %d, ", sbuf->dst,
				   sbuf->channel, sbuf->state);
			seq_printf(m, "virt: 0x%lx, phy: 0x%0x, map: 0x%x",
				   (unsigned long)sbuf->smem_virt,
				   sbuf->smem_addr, sbuf->dst_smem_addr);
			seq_printf(m, " size: 0x%0x, ringnr: %d\n",
				   sbuf->smem_size, sbuf->ringnr);
			sipc_debug_putline(m, '-', 100);

			/* list all  sbuf ring info list in a chanel */
			sipc_debug_putline(m, '-', 80);
			seq_puts(m, "  1. all sbuf ring info list:\n");
			for (n = 0; n < sbuf->ringnr; n++) {
				ring = &sbuf->rings[n];
				hd_op = &ring->header_op;
				if (READ_ONCE(*hd_op->tx_wt_p) == 0 &&
				    READ_ONCE(*hd_op->rx_wt_p) == 0)
					continue;

				seq_printf(m, "  rx ring[%2d]: addr: 0x%0x, ",
					   n, hd_op->rx_size);
				seq_printf(
					m,
					"rp: 0x%0x, wp: 0x%0x, size: 0x%0x\n",
					READ_ONCE(*hd_op->rx_rd_p),
					READ_ONCE(*hd_op->rx_wt_p),
					hd_op->rx_size);

				seq_printf(m, "  tx ring[%2d]: addr: 0x%0x, ",
					   n, hd_op->tx_size);
				seq_printf(
					m,
					"rp: 0x%0x, wp: 0x%0x, size: 0x%0x\n",
					READ_ONCE(*hd_op->tx_rd_p),
					READ_ONCE(*hd_op->tx_wt_p),
					hd_op->tx_size);
			}

			/* list all sbuf rxwait/txwait in a chanel */;
			sipc_debug_putline(m, '-', 80);
			seq_puts(m, "  2. all waittask list:\n");
			sbuf_debug_task_show(m, sbuf, TASK_RXWAIT);
			sbuf_debug_task_show(m, sbuf, TASK_TXWAIT);
			sbuf_debug_task_show(m, sbuf, TASK_SELECT);

#ifdef SIPC_DEBUG_SBUF_RDWT_OWNER
			/* list all sbuf ever read task list in a chanel */;
			sipc_debug_putline(m, '-', 80);
			seq_puts(m, "  3. all ever rdwt list:\n");
			sbuf_debug_list_show(m, sbuf, 1);
			sbuf_debug_list_show(m, sbuf, 0);
#endif

			/* list all  rx full ring list in a chanel */
			cnt = 0;
			for (n = 0; n < sbuf->ringnr; n++) {
				ring = &sbuf->rings[n];
				hd_op = &ring->header_op;
				if ((READ_ONCE(*hd_op->rx_wt_p) -
				     READ_ONCE(*hd_op->rx_rd_p)) ==
				    hd_op->rx_size) {
					if (cnt == 0) {
						sipc_debug_putline(m, '-', 80);
						seq_puts(
							m,
							"  x. all rx full ring list:\n");
					}
					cnt++;
					seq_printf(m, "  ring[%2d]\n", n);
				}
			}
		}
	}

	return 0;
}

static int sbuf_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sbuf_debug_show, inode->i_private);
}

static const struct file_operations sbuf_debug_fops = {
	.open = sbuf_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int sbuf_init_debugfs(void *root)
{
	if (!root)
		return -ENXIO;
	debugfs_create_file("sbuf", 0444, (struct dentry *)root, NULL,
			    &sbuf_debug_fops);
	return 0;
}
EXPORT_SYMBOL_GPL(sbuf_init_debugfs);

#endif /* CONFIG_DEBUG_FS */

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SBUF driver");
MODULE_LICENSE("GPL");
