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
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sipc.h>
#include <uapi/linux/sched/types.h>

#include "sipc_priv.h"
#include "sipc_debugfs.h"
#include "sblock.h"

static struct sblock_mgr *sblocks[SIPC_ID_NR][SMSG_VALID_CH_NR];

/* Serialize AP ownership and reset; the CP still owns its producer pointers. */
#define SBLOCK_NOTIFY_SEND_PENDING BIT(0)
#define SBLOCK_NOTIFY_RELEASE_PENDING BIT(1)
#define SBLOCK_NOTIFY_INIT_PENDING BIT(2)
#define SBLOCK_NOTIFY_OPEN_PENDING BIT(3)
#define SBLOCK_NOTIFY_CLOSE_PENDING BIT(4)

static struct sblock_mgr *sblock_lookup(u8 dst, u8 channel)
{
	u8 index = sipc_channel2index(channel);

	if (dst >= SIPC_ID_NR || index == INVALID_CHANEL_INDEX)
		return NULL;
	return READ_ONCE(sblocks[dst][index]);
}

static void sblock_wake(struct sblock_mgr *sblock)
{
	wake_up_interruptible_all(&sblock->ring->getwait);
	wake_up_interruptible_all(&sblock->ring->recvwait);
}

/* Called with lock held; a new epoch always invalidates sleeping requests. */
static void sblock_idle_locked(struct sblock_mgr *sblock)
{
	WRITE_ONCE(sblock->state, SBLOCK_STATE_IDLE);
	WRITE_ONCE(sblock->generation, sblock->generation + 1);
	sblock->notify_pending = 0;
	sblock->pool_ready = false;
}

/* Never allocate after publishing a shared descriptor. */
static void sblock_notify_locked(struct sblock_mgr *sblock, unsigned long bits)
{
	if (sblock->stopping)
		return;
	sblock->notify_pending |= bits;
	mod_delayed_work(system_wq, &sblock->notify_work, 0);
}

static bool sblock_retryable(int ret)
{
	return ret == -EAGAIN || ret == -EBUSY || ret == -ENOSPC ||
	       ret == -ENOBUFS || ret == -ETIMEDOUT || ret == -ETIME;
}

static void sblock_notify_work(struct work_struct *work)
{
	struct sblock_mgr *sblock = container_of(
		to_delayed_work(work), struct sblock_mgr, notify_work);
	struct smsg msg;
	unsigned long flags, bit;
	bool opened = false, failed = false;
	int ret;

	/* A reset cannot overtake an in-flight old-epoch EVENT or DONE. */
	mutex_lock(&sblock->control_lock);
	for (;;) {
		spin_lock_irqsave(&sblock->lock, flags);
		if (sblock->stopping || !sblock->notify_pending) {
			spin_unlock_irqrestore(&sblock->lock, flags);
			break;
		}
		if (sblock->notify_pending & SBLOCK_NOTIFY_CLOSE_PENDING)
			bit = SBLOCK_NOTIFY_CLOSE_PENDING;
		else if (sblock->notify_pending & SBLOCK_NOTIFY_OPEN_PENDING)
			bit = SBLOCK_NOTIFY_OPEN_PENDING;
		else if (sblock->notify_pending & SBLOCK_NOTIFY_INIT_PENDING)
			bit = SBLOCK_NOTIFY_INIT_PENDING;
		else if (sblock->state == SBLOCK_STATE_READY)
			bit = BIT(__ffs(sblock->notify_pending));
		else {
			spin_unlock_irqrestore(&sblock->lock, flags);
			break;
		}
		sblock->notify_pending &= ~bit;
		spin_unlock_irqrestore(&sblock->lock, flags);

		if (bit == SBLOCK_NOTIFY_OPEN_PENDING)
			smsg_set(&msg, sblock->channel, SMSG_TYPE_OPEN,
				 SMSG_OPEN_MAGIC, 0);
		else if (bit == SBLOCK_NOTIFY_CLOSE_PENDING)
			smsg_set(&msg, sblock->channel, SMSG_TYPE_CLOSE,
				 SMSG_CLOSE_MAGIC, 0);
		else if (bit == SBLOCK_NOTIFY_INIT_PENDING)
			smsg_set(&msg, sblock->channel, SMSG_TYPE_DONE,
				 SMSG_DONE_SBLOCK_INIT, sblock->dst_smem_addr);
		else
			smsg_set(&msg, sblock->channel, SMSG_TYPE_EVENT,
				 bit == SBLOCK_NOTIFY_SEND_PENDING ?
					 SMSG_EVENT_SBLOCK_SEND :
					 SMSG_EVENT_SBLOCK_RELEASE,
				 0);
		/* No manager spinlock held, and never sleep for mailbox capacity. */
		ret = smsg_send(sblock->dst, &msg, 0);
		spin_lock_irqsave(&sblock->lock, flags);
		if (ret) {
			if (sblock_retryable(ret) && !sblock->stopping) {
				sblock->notify_pending |= bit;
				mod_delayed_work(system_wq,
						 &sblock->notify_work,
						 msecs_to_jiffies(20));
			} else {
				sblock_idle_locked(sblock);
				failed = true;
			}
		} else if (!sblock->stopping &&
			   (bit == SBLOCK_NOTIFY_INIT_PENDING ||
			    (bit == SBLOCK_NOTIFY_OPEN_PENDING &&
			     sblock->pre_cfg))) {
			/* Dynamic: DONE accepted. Preconfigured: OPEN ACK accepted. */
			WRITE_ONCE(sblock->state, SBLOCK_STATE_READY);
			sblock->ever_ready = true;
			opened = true;
		}
		spin_unlock_irqrestore(&sblock->lock, flags);
		if (ret)
			break;
	}
	if (opened || failed)
		sblock_wake(sblock);
	if (opened && sblock->handler)
		sblock->handler(SBLOCK_NOTIFY_OPEN, sblock->data);
	if (failed && sblock->handler)
		sblock->handler(SBLOCK_NOTIFY_CLOSE, sblock->data);
	mutex_unlock(&sblock->control_lock);
}

/* A returned old-epoch loan is valid for recycling, never for sending data. */
static int sblock_owned_index(struct sblock_mgr *sblock, struct sblock *blk,
			      bool tx)
{
	struct sblock_ring *ring = sblock->ring;
	struct sblock_record *records = tx ? ring->txrecord : ring->rxrecord;
	unsigned long base =
		(unsigned long)(tx ? ring->txblk_virt : ring->rxblk_virt);
	unsigned long address, offset;
	u32 size = tx ? sblock->txblksz : sblock->rxblksz;
	u32 count = tx ? sblock->txblknum : sblock->rxblknum;
	u32 index;

	if (!blk || !size || !count)
		return -EINVAL;
	address = (unsigned long)blk->addr;
	if (address < base)
		return -EINVAL;
	offset = address - base;
	if (offset % size || offset / size >= count || blk->length > size)
		return -EINVAL;
	index = offset / size;
	if (!records[index].pending || records[index].cookie != blk->cookie ||
	    records[index].generation != blk->generation)
		return -ESTALE;
	return index;
}

void sblock_put(u8 dst, u8 channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	struct sblock_ring *ring;
	struct sblock_ring_header_op *hd;
	unsigned long flags;
	u32 rd, count, pos;
	int index;

	if (!sblock)
		return;
	ring = sblock->ring;
	hd = &ring->header_op.poolhd_op;
	spin_lock_irqsave(&sblock->lock, flags);
	index = sblock_owned_index(sblock, blk, true);
	if (index < 0)
		goto unlock;
	/* Before OPEN recovery, local ownership suffices; recovery includes it. */
	if (!sblock->pool_ready) {
		ring->txrecord[index].pending = false;
		goto unlock;
	}
	rd = READ_ONCE(*hd->tx_rd_p);
	count = READ_ONCE(*hd->tx_wt_p) - rd;
	if (count >= hd->tx_count)
		goto unlock; /* Keep ownership if corrupt/full; never overwrite a loan. */
	pos = sblock_get_ringpos(rd - 1, hd->tx_count);
	ring->p_txblks[pos].addr = hd->tx_addr + index * sblock->txblksz;
	ring->p_txblks[pos].length = sblock->txblksz;
	/* Publish the complete descriptor before its producer/consumer pointer. */
	wmb();
	WRITE_ONCE(*hd->tx_rd_p, rd - 1);
	ring->txrecord[index].pending = false;
unlock:
	spin_unlock_irqrestore(&sblock->lock, flags);
	sblock_wake(sblock);
}
EXPORT_SYMBOL_GPL(sblock_put);

/* Remote OPEN is the established peer ring-recovery protocol, not a DMA fence. */
static void sblock_recover_locked(struct sblock_mgr *sblock)
{
	struct sblock_ring *ring = sblock->ring;
	struct sblock_ring_header_op *pool = &ring->header_op.poolhd_op;
	struct sblock_ring_header_op *hd = &ring->header_op.ringhd_op;
	u32 base, n, i, pos;

	sblock_idle_locked(sblock);
	WRITE_ONCE(*hd->tx_wt_p, READ_ONCE(*hd->tx_rd_p));
	base = READ_ONCE(*pool->tx_wt_p);
	WRITE_ONCE(*pool->tx_rd_p, base);
	for (i = 0, n = 0; i < pool->tx_count; i++) {
		if (ring->txrecord[i].pending)
			continue;
		pos = sblock_get_ringpos(base + n++, pool->tx_count);
		ring->p_txblks[pos].addr = pool->tx_addr + i * sblock->txblksz;
		ring->p_txblks[pos].length = sblock->txblksz;
	}
	/* Publish the complete descriptor before its producer/consumer pointer. */
	wmb();
	WRITE_ONCE(*pool->tx_wt_p, base + n);
	WRITE_ONCE(*hd->rx_rd_p, READ_ONCE(*hd->rx_wt_p));
	base = READ_ONCE(*pool->rx_rd_p);
	for (i = 0, n = 0; i < pool->rx_count; i++) {
		if (ring->rxrecord[i].pending)
			continue;
		pos = sblock_get_ringpos(base + n++, pool->rx_count);
		ring->p_rxblks[pos].addr = pool->rx_addr + i * sblock->rxblksz;
		ring->p_rxblks[pos].length = sblock->rxblksz;
	}
	/* Publish the complete descriptor before its producer/consumer pointer. */
	wmb();
	WRITE_ONCE(*pool->rx_wt_p, base + n);
	sblock->pool_ready = true;
}

static int sblock_host_init(struct smsg_ipc *sipc, struct sblock_mgr *sblock,
			    u32 txblocknum, u32 txblocksize, u32 rxblocknum,
			    u32 rxblocksize)
{
	struct sblock_ring_header *ringhd;
	struct sblock_ring_header *poolhd;
	struct sblock_ring_header_op *ringhd_op;
	struct sblock_ring_header_op *poolhd_op;

	u32 hsize;
	int i;
	phys_addr_t offset = 0;
	u8 dst = sblock->dst;

	if ((!txblocknum && !rxblocknum) || (!!txblocknum != !!txblocksize) ||
	    (!!rxblocknum != !!rxblocksize) ||
	    txblocksize > INT_MAX - SBLOCK_ALIGN_BYTES ||
	    rxblocksize > INT_MAX - SBLOCK_ALIGN_BYTES)
		return -EINVAL;
	txblocksize = ALIGN(txblocksize, SBLOCK_ALIGN_BYTES);
	rxblocksize = ALIGN(rxblocksize, SBLOCK_ALIGN_BYTES);
	if ((u64)sizeof(struct sblock_header) + (u64)txblocknum * txblocksize +
		    (u64)rxblocknum * rxblocksize +
		    ((u64)txblocknum + rxblocknum) * 2 *
			    sizeof(struct sblock_blks) >
	    U32_MAX)
		return -EINVAL;
	sblock->txblksz = txblocksize;
	sblock->rxblksz = rxblocksize;
	sblock->txblknum = txblocknum;
	sblock->rxblknum = rxblocknum;

	pr_debug("%s: channel %d-%d, txblocksize=%#x, rxblocksize=%#x!\n",
		 __func__, sblock->dst, sblock->channel, txblocksize,
		 rxblocksize);

	pr_debug("%s: channel %d-%d, txblocknum=%#x, rxblocknum=%#x!\n",
		 __func__, sblock->dst, sblock->channel, txblocknum,
		 rxblocknum);

	/* allocate smem */
	hsize = sizeof(struct sblock_header);
	/* for header*/
	sblock->smem_size =
		hsize +
		/* for blks */
		txblocknum * txblocksize + rxblocknum * rxblocksize +
		/* for ring */
		(txblocknum + rxblocknum) * sizeof(struct sblock_blks) +
		/* for pool */
		(txblocknum + rxblocknum) * sizeof(struct sblock_blks);

	sblock->smem_addr = smem_alloc(dst, sblock->smem_size);
	if (!sblock->smem_addr) {
		pr_err("%s: channel %d-%d, Failed to alloc smem for sblock\n",
		       __func__, sblock->dst, sblock->channel);
		return -ENOMEM;
	}

	pr_debug("%s: channel %d-%d, smem_addr=%#llx, smem_size=%#x!\n",
		 __func__, sblock->dst, sblock->channel,
		 sblock->smem_addr + offset, sblock->smem_size);

	sblock->dst_smem_addr =
		sblock->smem_addr - sipc->smem_base + sipc->dst_smem_base;

	/* in host mode,  it is client physial address. */
	sblock->stored_smem_addr = sblock->dst_smem_addr;

#ifdef CONFIG_PHYS_ADDR_T_64BIT
	offset = sipc->high_offset;
	offset = offset << 32;
#endif
	pr_debug("%s: channel %d-%d, offset = 0x%llx!\n", __func__, sblock->dst,
		 sblock->channel, offset);
	sblock->smem_virt = shmem_ram_vmap_nocache(
		dst, sblock->smem_addr + offset, sblock->smem_size);
	if (!sblock->smem_virt) {
		pr_err("%s: channel %d-%d, Failed to map smem for sblock\n",
		       __func__, sblock->dst, sblock->channel);
		goto sblock_host_smem_free;
	}

	/* initialize header */
	ringhd = (struct sblock_ring_header *)(sblock->smem_virt);
	WRITE_ONCE(ringhd->txblk_addr, sblock->stored_smem_addr + hsize);
	WRITE_ONCE(ringhd->txblk_count, txblocknum);
	WRITE_ONCE(ringhd->txblk_size, txblocksize);
	WRITE_ONCE(ringhd->txblk_rdptr, 0);
	WRITE_ONCE(ringhd->txblk_wrptr, 0);
	WRITE_ONCE(ringhd->txblk_blks, READ_ONCE(ringhd->txblk_addr) +
					       txblocknum * txblocksize +
					       rxblocknum * rxblocksize);
	WRITE_ONCE(ringhd->rxblk_addr,
		   READ_ONCE(ringhd->txblk_addr) + txblocknum * txblocksize);
	WRITE_ONCE(ringhd->rxblk_count, rxblocknum);
	WRITE_ONCE(ringhd->rxblk_size, rxblocksize);
	WRITE_ONCE(ringhd->rxblk_rdptr, 0);
	WRITE_ONCE(ringhd->rxblk_wrptr, 0);
	WRITE_ONCE(ringhd->rxblk_blks,
		   READ_ONCE(ringhd->txblk_blks) +
			   txblocknum * sizeof(struct sblock_blks));

	poolhd = ringhd + 1;
	WRITE_ONCE(poolhd->txblk_addr, sblock->stored_smem_addr + hsize);
	WRITE_ONCE(poolhd->txblk_count, txblocknum);
	WRITE_ONCE(poolhd->txblk_size, txblocksize);
	WRITE_ONCE(poolhd->txblk_rdptr, 0);
	WRITE_ONCE(poolhd->txblk_wrptr, 0);
	WRITE_ONCE(poolhd->txblk_blks,
		   READ_ONCE(ringhd->rxblk_blks) +
			   rxblocknum * sizeof(struct sblock_blks));
	WRITE_ONCE(poolhd->rxblk_addr,
		   READ_ONCE(ringhd->txblk_addr) + txblocknum * txblocksize);
	WRITE_ONCE(poolhd->rxblk_count, rxblocknum);
	WRITE_ONCE(poolhd->rxblk_size, rxblocksize);
	WRITE_ONCE(poolhd->rxblk_rdptr, 0);
	WRITE_ONCE(poolhd->rxblk_wrptr, 0);
	WRITE_ONCE(poolhd->rxblk_blks,
		   READ_ONCE(poolhd->txblk_blks) +
			   txblocknum * sizeof(struct sblock_blks));

	pr_debug("%s: channel %d-%d, int ring!\n", __func__, sblock->dst,
		 sblock->channel);

	/* initialize ring */
	sblock->ring->txrecord = kcalloc(
		txblocknum, sizeof(*sblock->ring->txrecord), GFP_KERNEL);
	if (!sblock->ring->txrecord)
		goto sblock_host_unmap;

	sblock->ring->rxrecord = kcalloc(
		rxblocknum, sizeof(*sblock->ring->rxrecord), GFP_KERNEL);
	if (!sblock->ring->rxrecord)
		goto sblock_host_tx_free;

	sblock->ring->header = sblock->smem_virt;
	sblock->ring->txblk_virt =
		sblock->smem_virt +
		(READ_ONCE(ringhd->txblk_addr) - sblock->stored_smem_addr);
	sblock->ring->r_txblks =
		sblock->smem_virt +
		(READ_ONCE(ringhd->txblk_blks) - sblock->stored_smem_addr);
	sblock->ring->rxblk_virt =
		sblock->smem_virt +
		(READ_ONCE(ringhd->rxblk_addr) - sblock->stored_smem_addr);
	sblock->ring->r_rxblks =
		sblock->smem_virt +
		(READ_ONCE(ringhd->rxblk_blks) - sblock->stored_smem_addr);
	sblock->ring->p_txblks =
		sblock->smem_virt +
		(READ_ONCE(poolhd->txblk_blks) - sblock->stored_smem_addr);
	sblock->ring->p_rxblks =
		sblock->smem_virt +
		(READ_ONCE(poolhd->rxblk_blks) - sblock->stored_smem_addr);

	for (i = 0; i < txblocknum; i++) {
		sblock->ring->p_txblks[i].addr =
			READ_ONCE(poolhd->txblk_addr) + i * txblocksize;
		sblock->ring->p_txblks[i].length = txblocksize;
		sblock->ring->txrecord[i].pending = false;
		WRITE_ONCE(poolhd->txblk_wrptr,
			   READ_ONCE(poolhd->txblk_wrptr) + 1);
	}
	for (i = 0; i < rxblocknum; i++) {
		sblock->ring->p_rxblks[i].addr =
			READ_ONCE(poolhd->rxblk_addr) + i * rxblocksize;
		sblock->ring->p_rxblks[i].length = rxblocksize;
		sblock->ring->rxrecord[i].pending = false;
		WRITE_ONCE(poolhd->rxblk_wrptr,
			   READ_ONCE(poolhd->rxblk_wrptr) + 1);
	}

	/* init header op */
	ringhd_op = &((sblock->ring->header_op).ringhd_op);
	ringhd_op->tx_rd_p = &ringhd->txblk_rdptr;
	ringhd_op->tx_wt_p = &ringhd->txblk_wrptr;
	ringhd_op->rx_rd_p = &ringhd->rxblk_rdptr;
	ringhd_op->rx_wt_p = &ringhd->rxblk_wrptr;
	ringhd_op->tx_addr = READ_ONCE(ringhd->txblk_addr);
	ringhd_op->tx_count = READ_ONCE(ringhd->txblk_count);
	ringhd_op->tx_size = READ_ONCE(ringhd->txblk_size);
	ringhd_op->tx_blks = READ_ONCE(ringhd->txblk_blks);
	ringhd_op->rx_addr = READ_ONCE(ringhd->rxblk_addr);
	ringhd_op->rx_count = READ_ONCE(ringhd->rxblk_count);
	ringhd_op->rx_size = READ_ONCE(ringhd->rxblk_size);
	ringhd_op->rx_blks = READ_ONCE(ringhd->rxblk_blks);
	poolhd_op = &((sblock->ring->header_op).poolhd_op);
	poolhd_op->tx_rd_p = &poolhd->txblk_rdptr;
	poolhd_op->tx_wt_p = &poolhd->txblk_wrptr;
	poolhd_op->rx_rd_p = &poolhd->rxblk_rdptr;
	poolhd_op->rx_wt_p = &poolhd->rxblk_wrptr;
	poolhd_op->tx_addr = READ_ONCE(poolhd->txblk_addr);
	poolhd_op->tx_count = READ_ONCE(poolhd->txblk_count);
	poolhd_op->tx_size = READ_ONCE(poolhd->txblk_size);
	poolhd_op->tx_blks = READ_ONCE(poolhd->txblk_blks);
	poolhd_op->rx_addr = READ_ONCE(poolhd->rxblk_addr);
	poolhd_op->rx_count = READ_ONCE(poolhd->rxblk_count);
	poolhd_op->rx_size = READ_ONCE(poolhd->rxblk_size);
	poolhd_op->rx_blks = READ_ONCE(poolhd->rxblk_blks);

	return 0;

sblock_host_tx_free:
	kfree(sblock->ring->txrecord);
sblock_host_unmap:
	shmem_ram_unmap(dst, sblock->smem_virt);
sblock_host_smem_free:
	smem_free(dst, sblock->smem_addr, sblock->smem_size);

	pr_err("%s: channel %d-%d, failed, ENOMEM!\n", __func__, sblock->dst,
	       sblock->channel);

	return -ENOMEM;
}

static int sblock_thread(void *data)
{
	struct sblock_mgr *sblock = data;
	struct smsg msg;
	unsigned long flags;
	bool closed;
	int ret;

	/* Preconfigured peers use OPEN, not the dynamic CMD/DONE handshake. */
	ret = smsg_ch_open(sblock->dst, sblock->channel, -1);
	if (ret) {
		sblock_wake(sblock);
		if (sblock->handler)
			sblock->handler(SBLOCK_NOTIFY_OPEN_FAILED,
					sblock->data);
		return ret;
	}
	if (sblock->pre_cfg) {
		spin_lock_irqsave(&sblock->lock, flags);
		if (sblock->stopping) {
			spin_unlock_irqrestore(&sblock->lock, flags);
			return -EPIPE;
		}
		WRITE_ONCE(sblock->state, SBLOCK_STATE_READY);
		sblock->ever_ready = true;
		spin_unlock_irqrestore(&sblock->lock, flags);
		sblock_wake(sblock);
		if (sblock->handler)
			sblock->handler(SBLOCK_NOTIFY_OPEN, sblock->data);
	}

	while (!kthread_should_stop()) {
		smsg_set(&msg, sblock->channel, 0, 0, 0);
		ret = smsg_recv(sblock->dst, &msg, -1);
		if (ret) {
			if (kthread_should_stop())
				break;
			/* Never interpret an unfilled message after an interrupted receive. */
			if (ret == -EIO || ret == -ENODEV) {
				mutex_lock(&sblock->control_lock);
				spin_lock_irqsave(&sblock->lock, flags);
				if (sblock->state == SBLOCK_STATE_READY)
					sblock_idle_locked(sblock);
				spin_unlock_irqrestore(&sblock->lock, flags);
				mutex_unlock(&sblock->control_lock);
				sblock_wake(sblock);
			}
			msleep(20);
			continue;
		}
		closed = false;
		mutex_lock(&sblock->control_lock);
		spin_lock_irqsave(&sblock->lock, flags);
		if (sblock->stopping)
			goto unlock;
		switch (msg.type) {
		case SMSG_TYPE_OPEN:
			if (msg.flag != SMSG_OPEN_MAGIC)
				break;
			if (sblock->ever_ready || !sblock->pool_ready) {
				sblock_recover_locked(sblock);
				closed = true;
			}
			sblock_notify_locked(sblock,
					     SBLOCK_NOTIFY_OPEN_PENDING);
			break;
		case SMSG_TYPE_CLOSE:
			if (msg.flag != SMSG_CLOSE_MAGIC)
				break;
			sblock_idle_locked(sblock);
			closed = true;
			sblock_notify_locked(sblock,
					     SBLOCK_NOTIFY_CLOSE_PENDING);
			break;
		case SMSG_TYPE_CMD:
			if (sblock->pre_cfg ||
			    msg.flag != SMSG_CMD_SBLOCK_INIT ||
			    !sblock->pool_ready)
				break;
			sblock_notify_locked(sblock,
					     SBLOCK_NOTIFY_INIT_PENDING);
			break;
		case SMSG_TYPE_EVENT:
			if (sblock->state != SBLOCK_STATE_READY)
				break;
			if (msg.flag == SMSG_EVENT_SBLOCK_SEND ||
			    msg.flag == SMSG_EVENT_SBLOCK_RELEASE) {
				spin_unlock_irqrestore(&sblock->lock, flags);
				mutex_unlock(&sblock->control_lock);
				sblock_wake(sblock);
				if (sblock->handler)
					sblock->handler(
						msg.flag == SMSG_EVENT_SBLOCK_SEND ?
							SBLOCK_NOTIFY_RECV :
							SBLOCK_NOTIFY_GET,
						sblock->data);
				continue;
			}
			break;
		default:
			break;
		}
unlock:
		spin_unlock_irqrestore(&sblock->lock, flags);
		if (closed) {
			sblock_wake(sblock);
			if (sblock->handler)
				sblock->handler(SBLOCK_NOTIFY_CLOSE,
						sblock->data);
		}
		mutex_unlock(&sblock->control_lock);
	}
	return 0;
}

static int sblock_mgr_create(uint8_t dst, uint8_t channel, int pre_cfg,
			     uint32_t tx_blk_num, uint32_t tx_blk_sz,
			     uint32_t rx_blk_num, uint32_t rx_blk_sz,
			     struct sblock_mgr **sb_mgr)
{
	struct sblock_mgr *sblock = NULL;
	struct smsg_ipc *sipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	int ret;

	pr_debug("%s: dst=%d channel=%d\n", __func__, dst, channel);

	if (!sipc_local_peer_supported(dst))
		return -EOPNOTSUPP;
	if (!sipc)
		return -EPROBE_DEFER;
	if (sipc->client)
		return -EOPNOTSUPP;

	sblock = kzalloc_obj(struct sblock_mgr, GFP_KERNEL);
	if (!sblock)
		return -ENOMEM;

	sblock->ring = kzalloc_obj(struct sblock_ring, GFP_KERNEL);
	if (!sblock->ring) {
		kfree(sblock);
		return -ENOMEM;
	}

	sblock->dst = dst;
	sblock->channel = channel;
	sblock->pre_cfg = pre_cfg;
	sblock->state = SBLOCK_STATE_IDLE;
	sblock->generation = 1;
	spin_lock_init(&sblock->lock);
	mutex_init(&sblock->control_lock);
	INIT_DELAYED_WORK(&sblock->notify_work, sblock_notify_work);

	if (!sipc->client) {
		ret = sblock_host_init(sipc, sblock, tx_blk_num, tx_blk_sz,
				       rx_blk_num, rx_blk_sz);
		if (ret) {
			kfree(sblock->ring);
			kfree(sblock);
			return ret;
		}
	}

	init_waitqueue_head(&sblock->ring->getwait);
	init_waitqueue_head(&sblock->ring->recvwait);
	spin_lock_init(&sblock->ring->r_txlock);
	spin_lock_init(&sblock->ring->r_rxlock);
	spin_lock_init(&sblock->ring->p_txlock);
	spin_lock_init(&sblock->ring->p_rxlock);

	sblock->pool_ready = true;
	*sb_mgr = sblock;

	return 0;
}

int sblock_create_ex(u8 dst, u8 channel, u32 txblocknum, u32 txblocksize,
		     u32 rxblocknum, u32 rxblocksize,
		     void (*handler)(int event, void *data), void *data)
{
	struct sblock_mgr *sblock = NULL;
	int result;
	u8 ch_index;
	struct smsg_ipc *sipc;
	struct sched_param param = { .sched_priority = 11 };

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	if (dst >= SIPC_ID_NR) {
		pr_err("%s: dst = %d is invalid\n", __func__, dst);
		return -EINVAL;
	}

	pr_debug("%s: dst=%d channel=%d\n", __func__, dst, channel);

	if (sblocks[dst][ch_index])
		return -EBUSY;
	result = sblock_mgr_create(dst, channel, 0, txblocknum, txblocksize,
				   rxblocknum, rxblocksize, &sblock);
	if (!result) {
		sblock->thread = kthread_create(sblock_thread, sblock,
						"sblock-%d-%d", dst, channel);
		if (IS_ERR(sblock->thread)) {
			pr_err("Failed to create kthread: sblock-%d-%d\n", dst,
			       channel);
			sipc = smsg_ipcs[sblock->dst];
			if (!sipc->client) {
				shmem_ram_unmap(dst, sblock->smem_virt);
				smem_free(dst, sblock->smem_addr,
					  sblock->smem_size);
				kfree(sblock->ring->txrecord);
				kfree(sblock->ring->rxrecord);
			}
			kfree(sblock->ring);
			result = PTR_ERR(sblock->thread);
			kfree(sblock);
			return result;
		}

		/* Prevent the thread task_struct from being destroyed. */
		get_task_struct(sblock->thread);

		sblocks[dst][ch_index] = sblock;
		if ((handler != NULL) && (data != NULL)) {
			result = sblock_register_notifier(dst, channel, handler,
							  data);
			if (result < 0) {
				sblock_destroy(dst, channel);
				return result;
			}
		}
		/*set the thread as a real time thread, and its priority is 11*/
		sched_setscheduler(sblock->thread, SCHED_RR, &param);
		wake_up_process(sblock->thread);
	}

	pr_debug("%s: sblock-%d-%d create over, result = %d\n", __func__, dst,
		 channel, result);

	return result;
}
EXPORT_SYMBOL_GPL(sblock_create_ex);

int sblock_create(u8 dst, u8 channel, u32 txblocknum, u32 txblocksize,
		  u32 rxblocknum, u32 rxblocksize)
{
	return sblock_create_ex(dst, channel, txblocknum, txblocksize,
				rxblocknum, rxblocksize, NULL, NULL);
}
EXPORT_SYMBOL_GPL(sblock_create);

int sblock_pcfg_create(u8 dst, u8 channel, u32 tx_blk_num, u32 tx_blk_sz,
		       u32 rx_blk_num, u32 rx_blk_sz)
{
	struct sblock_mgr *sblock = NULL;
	int result;
	u8 ch_index;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}
	if (dst >= SIPC_ID_NR) {
		pr_err("sblock_create_ex: dst = %d is invalid\n", dst);
		return -EINVAL;
	}

	pr_debug("%s: dst=%d channel=%d\n", __func__, dst, channel);

	if (sblocks[dst][ch_index])
		return -EBUSY;
	result = sblock_mgr_create(dst, channel, 1, tx_blk_num, tx_blk_sz,
				   rx_blk_num, rx_blk_sz, &sblock);
	if (!result) {
		struct sched_param param = { .sched_priority = 11 };

		sblock->thread = kthread_create(sblock_thread, sblock,
						"sblock-%d-%d", dst, channel);
		if (IS_ERR(sblock->thread)) {
			struct smsg_ipc *sipc;

			pr_err("Failed to create kthread: sblock-%d-%d\n", dst,
			       channel);
			sipc = smsg_ipcs[sblock->dst];
			if (!sipc->client) {
				shmem_ram_unmap(dst, sblock->smem_virt);
				smem_free(dst, sblock->smem_addr,
					  sblock->smem_size);
				kfree(sblock->ring->txrecord);
				kfree(sblock->ring->rxrecord);
			}
			kfree(sblock->ring);
			result = PTR_ERR(sblock->thread);
			kfree(sblock);
			return result;
		}

		/* Prevent the thread task_struct from being destroyed. */
		get_task_struct(sblock->thread);

		sblocks[dst][ch_index] = sblock;
		/*
		 * Set the thread as a real time thread, and its priority
		 * is 11.
		 */
		sched_setscheduler(sblock->thread, SCHED_RR, &param);
		wake_up_process(sblock->thread);
	}

	return result;
}
EXPORT_SYMBOL_GPL(sblock_pcfg_create);

void sblock_destroy(u8 dst, u8 channel)
{
	struct sblock_mgr *sblock;
	u8 ch_index;
	struct smsg_ipc *sipc;
	unsigned long flags;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return;
	}

	sblock = dst < SIPC_ID_NR ? sblocks[dst][ch_index] : NULL;
	if (sblock == NULL)
		return;

	mutex_lock(&sblock->control_lock);
	spin_lock_irqsave(&sblock->lock, flags);
	sblock->stopping = true;
	sblock_idle_locked(sblock);
	spin_unlock_irqrestore(&sblock->lock, flags);
	mutex_unlock(&sblock->control_lock);
	sblock_wake(sblock);
	cancel_delayed_work_sync(&sblock->notify_work);
	smsg_ch_close(dst, channel, -1);

	/* stop sblock thread if it's created successfully and still alive */
	if (!IS_ERR_OR_NULL(sblock->thread)) {
		kthread_stop(sblock->thread);
		put_task_struct(sblock->thread);
		sblock->thread = NULL;
	}

	if (sblock->ring) {
		wake_up_interruptible_all(&sblock->ring->recvwait);
		wake_up_interruptible_all(&sblock->ring->getwait);
		/* kfree(NULL) is safe */
		/* if (sblock->ring->txrecord) */
		kfree(sblock->ring->txrecord);
		/* if (sblock->ring->rxrecord) */
		kfree(sblock->ring->rxrecord);
		kfree(sblock->ring);
	}
	if (sblock->smem_virt)
		shmem_ram_unmap(dst, sblock->smem_virt);

	sipc = dst < SIPC_ID_NR ? smsg_ipcs[dst] : NULL;
	if (sipc->client)
		smem_free(dst, sblock->smem_addr_debug, sblock->smem_size);
	else
		smem_free(dst, sblock->smem_addr, sblock->smem_size);
	kfree(sblock);

	sblocks[dst][ch_index] = NULL;
}
EXPORT_SYMBOL_GPL(sblock_destroy);

int sblock_pcfg_open(uint8_t dest, uint8_t channel,
		     void (*notifier)(int event, void *client), void *client)
{
	struct sblock_mgr *sblock;
	uint8_t idx;
	int ret;
	struct sched_param param = { .sched_priority = 11 };

	pr_debug("%s: dst=%d channel=%d\n", __func__, dest, channel);

	if (!notifier)
		return -EINVAL;

	idx = sipc_channel2index(channel);
	if (idx == INVALID_CHANEL_INDEX) {
		pr_err("%s: invalid channel %d!\n", __func__, channel);
		return -ENODEV;
	}

	sblock = dest < SIPC_ID_NR ? sblocks[dest][idx] : NULL;
	if (!sblock)
		return -ENODEV;

	if (!sblock->pre_cfg)
		return -EINVAL;

	if (sblock->thread) {
		pr_err("%s: SBLOCK %u/%u already open", __func__,
		       (unsigned int)sblock->dst,
		       (unsigned int)sblock->channel);
		return -EPROTO;
	}

	ret = 0;
	sblock->thread = kthread_create(sblock_thread, sblock, "sblock-%d-%d",
					dest, channel);
	if (IS_ERR(sblock->thread)) {
		pr_err("%s: create thread error\n", __func__);
		sblock->thread = NULL;
		ret = -EBUSY;
	} else {
		/* Prevent the thread task_struct from being destroyed. */
		get_task_struct(sblock->thread);

		sblock->handler = notifier;
		sblock->data = client;
		/*set the thread as a real time thread, and its priority is 11*/
		sched_setscheduler(sblock->thread, SCHED_RR, &param);
		wake_up_process(sblock->thread);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sblock_pcfg_open);

int sblock_register_notifier(u8 dst, u8 channel,
			     void (*handler)(int event, void *data), void *data)
{
	struct sblock_mgr *sblock;
	u8 ch_index;

	ch_index = sipc_channel2index(channel);
	if (ch_index == INVALID_CHANEL_INDEX) {
		pr_err("%s:channel %d invalid!\n", __func__, channel);
		return -EINVAL;
	}

	sblock = dst < SIPC_ID_NR ? sblocks[dst][ch_index] : NULL;

	if (!sblock) {
		pr_err("%s:sblock-%d-%d not ready!\n", __func__, dst, channel);
		return -ENODEV;
	}
#ifndef CONFIG_SPRD_SIPC_WCN
	if (sblock->handler) {
		pr_err("sblock handler already registered\n");
		return -EBUSY;
	}
#endif
	sblock->handler = handler;
	sblock->data = data;

	return 0;
}
EXPORT_SYMBOL_GPL(sblock_register_notifier);

int sblock_get_smem_cp_addr(uint8_t dest, uint8_t channel, uint32_t *paddr)
{
	struct sblock_mgr *sblock;
	uint8_t idx;

	if (!paddr)
		return -EINVAL;

	idx = sipc_channel2index(channel);
	if (idx == INVALID_CHANEL_INDEX) {
		pr_err("%s: invalid channel %d!\n", __func__, channel);
		return -ENODEV;
	}

	sblock = dest < SIPC_ID_NR ? sblocks[dest][idx] : NULL;
	if (!sblock)
		return -ENODEV;

	*paddr = sblock->dst_smem_addr;

	return 0;
}
EXPORT_SYMBOL_GPL(sblock_get_smem_cp_addr);

int sblock_get_generation(u8 dst, u8 channel, u32 *generation)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	unsigned long flags;
	int ret;

	if (!sblock)
		return -ENODEV;
	if (!generation)
		return -EINVAL;
	spin_lock_irqsave(&sblock->lock, flags);
	*generation = sblock->generation;
	ret = sblock->state == SBLOCK_STATE_READY && !sblock->stopping ? 0 :
									 -EPIPE;
	spin_unlock_irqrestore(&sblock->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sblock_get_generation);

static bool sblock_available(struct sblock_mgr *sblock, bool tx, u32 generation)
{
	struct sblock_ring_header_op *hd =
		tx ? &sblock->ring->header_op.poolhd_op :
		     &sblock->ring->header_op.ringhd_op;

	return READ_ONCE(sblock->generation) != generation ||
	       READ_ONCE(sblock->state) != SBLOCK_STATE_READY ||
	       (tx ? READ_ONCE(*hd->tx_wt_p) != READ_ONCE(*hd->tx_rd_p) :
		     READ_ONCE(*hd->rx_wt_p) != READ_ONCE(*hd->rx_rd_p));
}

static int sblock_take(u8 dst, u8 channel, struct sblock *blk, int timeout,
		       bool tx, u32 generation)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	struct sblock_ring *ring;
	struct sblock_ring_header_op *hd;
	struct sblock_record *record;
	struct sblock_blks *descriptor;
	wait_queue_head_t *wait;
	unsigned long flags;
	u32 rd, wr, nr, pos, address, base, length, size, offset, index;
	long ret;

	if (!sblock)
		return -ENODEV;
	if (!blk)
		return -EINVAL;
	ring = sblock->ring;
	size = tx ? sblock->txblksz : sblock->rxblksz;
	nr = tx ? sblock->txblknum : sblock->rxblknum;
	if (!size || !nr)
		return -EOPNOTSUPP;
	hd = tx ? &ring->header_op.poolhd_op : &ring->header_op.ringhd_op;
	wait = tx ? &ring->getwait : &ring->recvwait;
	if (timeout < 0) {
		ret = wait_event_interruptible(
			*wait, sblock_available(sblock, tx, generation));
		if (ret)
			return ret;
	} else if (timeout > 0) {
		ret = wait_event_interruptible_timeout(
			*wait, sblock_available(sblock, tx, generation),
			timeout);
		if (ret <= 0)
			return ret ? ret : -ETIME;
	}

	spin_lock_irqsave(&sblock->lock, flags);
	ret = -EPIPE;
	if (sblock->stopping || sblock->state != SBLOCK_STATE_READY ||
	    sblock->generation != generation)
		goto unlock;
	rd = tx ? READ_ONCE(*hd->tx_rd_p) : READ_ONCE(*hd->rx_rd_p);
	wr = tx ? READ_ONCE(*hd->tx_wt_p) : READ_ONCE(*hd->rx_wt_p);
	ret = -EAGAIN;
	if (wr == rd)
		goto unlock;
	ret = -EIO;
	if (wr - rd > nr)
		goto unlock;
	/* Observe the CP descriptor after observing its producer pointer. */
	rmb();
	pos = sblock_get_ringpos(rd, nr);
	descriptor = tx ? &ring->p_txblks[pos] : &ring->r_rxblks[pos];
	address = READ_ONCE(descriptor->addr);
	length = READ_ONCE(descriptor->length);
	base = tx ? hd->tx_addr : hd->rx_addr;
	if (address < base)
		goto unlock;
	offset = address - base;
	if (offset % size || offset / size >= nr || length > size ||
	    (tx && length != size))
		goto unlock;
	index = offset / size;
	record = tx ? &ring->txrecord[index] : &ring->rxrecord[index];
	if (record->pending)
		goto unlock;
	record->pending = true;
	record->generation = generation;
	record->cookie = ++sblock->next_cookie;
	if (!record->cookie)
		record->cookie = ++sblock->next_cookie;
	blk->addr = (tx ? ring->txblk_virt : ring->rxblk_virt) + offset;
	blk->length = length;
	blk->generation = generation;
	blk->cookie = record->cookie;
	/* Finish descriptor/payload accesses before returning shared ring space. */
	mb();
	if (tx)
		WRITE_ONCE(*hd->tx_rd_p, rd + 1);
	else
		WRITE_ONCE(*hd->rx_rd_p, rd + 1);
	ret = 0;
unlock:
	spin_unlock_irqrestore(&sblock->lock, flags);
	return ret;
}

int sblock_get_for_generation(u8 dst, u8 channel, struct sblock *blk,
			      int timeout, u32 generation)
{
	return sblock_take(dst, channel, blk, timeout, true, generation);
}
EXPORT_SYMBOL_GPL(sblock_get_for_generation);

int sblock_receive_for_generation(u8 dst, u8 channel, struct sblock *blk,
				  int timeout, u32 generation)
{
	return sblock_take(dst, channel, blk, timeout, false, generation);
}
EXPORT_SYMBOL_GPL(sblock_receive_for_generation);

int sblock_get(u8 dst, u8 channel, struct sblock *blk, int timeout)
{
	u32 generation;
	int ret = sblock_get_generation(dst, channel, &generation);

	return ret ? ret :
		     sblock_get_for_generation(dst, channel, blk, timeout,
					       generation);
}
EXPORT_SYMBOL_GPL(sblock_get);

int sblock_receive(u8 dst, u8 channel, struct sblock *blk, int timeout)
{
	u32 generation;
	int ret = sblock_get_generation(dst, channel, &generation);

	return ret ? ret :
		     sblock_receive_for_generation(dst, channel, blk, timeout,
						   generation);
}
EXPORT_SYMBOL_GPL(sblock_receive);

static int sblock_send_ex(u8 dst, u8 channel, struct sblock *blk, bool notify)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	struct sblock_ring *ring;
	struct sblock_ring_header_op *hd;
	unsigned long flags;
	u32 wr, rd, pos;
	int ret, index;

	if (!sblock)
		return -ENODEV;
	ring = sblock->ring;
	hd = &ring->header_op.ringhd_op;
	spin_lock_irqsave(&sblock->lock, flags);
	ret = -EPIPE;
	if (!blk || sblock->stopping || sblock->state != SBLOCK_STATE_READY ||
	    blk->generation != sblock->generation)
		goto unlock;
	index = sblock_owned_index(sblock, blk, true);
	ret = index;
	if (index < 0)
		goto unlock;
	wr = READ_ONCE(*hd->tx_wt_p);
	rd = READ_ONCE(*hd->tx_rd_p);
	ret = -EIO;
	if (wr - rd > hd->tx_count)
		goto unlock;
	ret = -EAGAIN;
	if (wr - rd == hd->tx_count)
		goto unlock;
	pos = sblock_get_ringpos(wr, hd->tx_count);
	ring->r_txblks[pos].addr = hd->tx_addr + index * sblock->txblksz;
	ring->r_txblks[pos].length = blk->length;
	/* Publish the complete descriptor before its producer/consumer pointer. */
	wmb();
	WRITE_ONCE(*hd->tx_wt_p, wr + 1);
	ring->txrecord[index].pending = false;
	if (notify)
		sblock_notify_locked(sblock, SBLOCK_NOTIFY_SEND_PENDING);
	/* Once published, the transport owns the block and retries its doorbell. */
	ret = 0;
unlock:
	spin_unlock_irqrestore(&sblock->lock, flags);
	return ret;
}

int sblock_send(u8 dst, u8 channel, struct sblock *blk)
{
	return sblock_send_ex(dst, channel, blk, true);
}
EXPORT_SYMBOL_GPL(sblock_send);

int sblock_send_prepare(u8 dst, u8 channel, struct sblock *blk)
{
	return sblock_send_ex(dst, channel, blk, false);
}
EXPORT_SYMBOL_GPL(sblock_send_prepare);

int sblock_send_finish(u8 dst, u8 channel)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	unsigned long flags;
	int ret = 0;

	if (!sblock)
		return -ENODEV;
	spin_lock_irqsave(&sblock->lock, flags);
	if (sblock->stopping || sblock->state != SBLOCK_STATE_READY)
		ret = -EPIPE;
	else
		sblock_notify_locked(sblock, SBLOCK_NOTIFY_SEND_PENDING);
	spin_unlock_irqrestore(&sblock->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sblock_send_finish);

int sblock_release(u8 dst, u8 channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	struct sblock_ring *ring;
	struct sblock_ring_header_op *hd;
	unsigned long flags;
	u32 wr, rd, pos;
	int ret, index;

	if (!sblock)
		return -ENODEV;
	ring = sblock->ring;
	hd = &ring->header_op.poolhd_op;
	spin_lock_irqsave(&sblock->lock, flags);
	index = sblock_owned_index(sblock, blk, false);
	ret = index;
	if (index < 0)
		goto unlock;
	if (!sblock->pool_ready) {
		ring->rxrecord[index].pending = false;
		ret = 0;
		goto unlock;
	}
	wr = READ_ONCE(*hd->rx_wt_p);
	rd = READ_ONCE(*hd->rx_rd_p);
	ret = -EIO;
	if (wr - rd >= hd->rx_count)
		goto unlock;
	pos = sblock_get_ringpos(wr, hd->rx_count);
	ring->p_rxblks[pos].addr = hd->rx_addr + index * sblock->rxblksz;
	ring->p_rxblks[pos].length = sblock->rxblksz;
	/* Finish payload reads and publish the pool descriptor before its pointer. */
	mb();
	WRITE_ONCE(*hd->rx_wt_p, wr + 1);
	ring->rxrecord[index].pending = false;
	sblock_notify_locked(sblock, SBLOCK_NOTIFY_RELEASE_PENDING);
	ret = 0;
unlock:
	spin_unlock_irqrestore(&sblock->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sblock_release);

static int sblock_count(u8 dst, u8 channel, bool tx)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	struct sblock_ring_header_op *hd;
	unsigned long flags;
	u32 n, max;
	int ret;

	if (!sblock)
		return -ENODEV;
	hd = tx ? &sblock->ring->header_op.poolhd_op :
		  &sblock->ring->header_op.ringhd_op;
	spin_lock_irqsave(&sblock->lock, flags);
	ret = -EPIPE;
	if (sblock->state != SBLOCK_STATE_READY)
		goto unlock;
	n = tx ? READ_ONCE(*hd->tx_wt_p) - READ_ONCE(*hd->tx_rd_p) :
		 READ_ONCE(*hd->rx_wt_p) - READ_ONCE(*hd->rx_rd_p);
	max = tx ? hd->tx_count : hd->rx_count;
	ret = n > max ? -EIO : (int)n;
unlock:
	spin_unlock_irqrestore(&sblock->lock, flags);
	return ret;
}

int sblock_get_arrived_count(u8 dst, u8 channel)
{
	return sblock_count(dst, channel, false);
}
EXPORT_SYMBOL_GPL(sblock_get_arrived_count);

int sblock_get_free_count(u8 dst, u8 channel)
{
	return sblock_count(dst, channel, true);
}
EXPORT_SYMBOL_GPL(sblock_get_free_count);

unsigned int sblock_poll_wait(u8 dst, u8 channel, struct file *filp,
			      poll_table *wait)
{
	struct sblock_mgr *sblock = sblock_lookup(dst, channel);
	struct sblock_ring_header_op *pool, *hd;
	unsigned long flags;
	u32 tx, rx;
	__poll_t mask = 0;

	if (!sblock)
		return EPOLLERR | EPOLLHUP;
	/* Register first, including while IDLE, so CLOSE/OPEN cannot be missed. */
	poll_wait(filp, &sblock->ring->getwait, wait);
	poll_wait(filp, &sblock->ring->recvwait, wait);
	pool = &sblock->ring->header_op.poolhd_op;
	hd = &sblock->ring->header_op.ringhd_op;
	spin_lock_irqsave(&sblock->lock, flags);
	if (sblock->stopping || sblock->state != SBLOCK_STATE_READY) {
		mask = EPOLLERR | EPOLLHUP;
		goto unlock;
	}
	tx = READ_ONCE(*pool->tx_wt_p) - READ_ONCE(*pool->tx_rd_p);
	rx = READ_ONCE(*hd->rx_wt_p) - READ_ONCE(*hd->rx_rd_p);
	if (tx > pool->tx_count || rx > hd->rx_count) {
		mask = EPOLLERR;
		goto unlock;
	}
	if (tx)
		mask |= EPOLLOUT | EPOLLWRNORM;
	if (rx)
		mask |= EPOLLIN | EPOLLRDNORM;
unlock:
	spin_unlock_irqrestore(&sblock->lock, flags);
	return mask;
}
EXPORT_SYMBOL_GPL(sblock_poll_wait);

int sblock_query(u8 dst, u8 channel)
{
	u32 generation;

	return sblock_get_generation(dst, channel, &generation);
}
EXPORT_SYMBOL_GPL(sblock_query);

#if defined(CONFIG_DEBUG_FS)
static int sblock_debug_show(struct seq_file *m, void *private)
{
	struct sblock_mgr *sblock;
	struct sblock_ring *ring;
	struct sblock_ring_header_op *poolhd_op;
	struct sblock_ring_header_op *ringhd_op;
	int i, j;

	for (i = 0; i < SIPC_ID_NR; i++) {
		for (j = 0; j < SMSG_VALID_CH_NR; j++) {
			sblock = sblocks[i][j];
			if (!sblock)
				continue;

			sipc_debug_putline(m, '*', 170);
			seq_printf(
				m,
				"sblock dst %d, channel: %3d, state: %d, smem_virt: 0x%lx, smem_addr: 0x%0x, dst_smem_addr: 0x%0x, smem_size: 0x%0x, txblksz: %d, rxblksz: %d\n",
				sblock->dst, sblock->channel, sblock->state,
				(unsigned long)sblock->smem_virt,
				sblock->smem_addr, sblock->dst_smem_addr,
				sblock->smem_size, sblock->txblksz,
				sblock->rxblksz);

			/*
			 * in precfg channel,  the ring pinter can be null
			 * before the block manager has been created
			 * and ring->header pointer can also be null
			 * before the block handshake with host,
			 * so must add null pointer protect here.
			 */
			ring = sblock->ring;
			if (!ring || !ring->header)
				continue;

			poolhd_op = &(ring->header_op.poolhd_op);
			ringhd_op = &(ring->header_op.ringhd_op);
			seq_printf(
				m,
				"sblock ring: txblk_virt :0x%lx, rxblk_virt :0x%lx\n",
				(unsigned long)ring->txblk_virt,
				(unsigned long)ring->rxblk_virt);
			seq_printf(
				m,
				"sblock ring header: rxblk_addr :0x%0x, rxblk_rdptr :0x%0x, rxblk_wrptr :0x%0x, rxblk_size :%d, rxblk_count :%d, rxblk_blks: 0x%0x\n",
				ringhd_op->rx_addr,
				READ_ONCE(*ringhd_op->rx_rd_p),
				READ_ONCE(*ringhd_op->rx_wt_p),
				ringhd_op->rx_size, ringhd_op->rx_count,
				ringhd_op->rx_blks);
			seq_printf(
				m,
				"sblock ring header: txblk_addr :0x%0x, txblk_rdptr :0x%0x, txblk_wrptr :0x%0x, txblk_size :%d, txblk_count :%d, txblk_blks: 0x%0x\n",
				ringhd_op->tx_addr,
				READ_ONCE(*ringhd_op->tx_rd_p),
				READ_ONCE(*ringhd_op->tx_wt_p),
				ringhd_op->tx_size, ringhd_op->tx_count,
				ringhd_op->tx_blks);
			seq_printf(
				m,
				"sblock pool header: rxblk_addr :0x%0x, rxblk_rdptr :0x%0x, rxblk_wrptr :0x%0x, rxblk_size :%d, rxpool_count :%d, rxblk_blks: 0x%0x\n",
				poolhd_op->rx_addr,
				READ_ONCE(*poolhd_op->rx_rd_p),
				READ_ONCE(*poolhd_op->rx_wt_p),
				poolhd_op->rx_size, poolhd_op->rx_count,
				poolhd_op->rx_blks);
			seq_printf(
				m,
				"sblock pool header: txblk_addr :0x%0x, txblk_rdptr :0x%0x, txblk_wrptr :0x%0x, txblk_size :%d, txpool_count :%d, txblk_blks: 0x%0x\n",
				poolhd_op->tx_addr,
				READ_ONCE(*poolhd_op->tx_rd_p),
				READ_ONCE(*poolhd_op->tx_wt_p),
				poolhd_op->tx_size, poolhd_op->tx_count,
				poolhd_op->tx_blks);
		}
	}
	return 0;
}

static int sblock_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sblock_debug_show, inode->i_private);
}

static const struct file_operations sblock_debug_fops = {
	.open = sblock_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int sblock_init_debugfs(void *root)
{
	if (!root)
		return -ENXIO;
	debugfs_create_file("sblock", 0444, (struct dentry *)root, NULL,
			    &sblock_debug_fops);
	return 0;
}
EXPORT_SYMBOL_GPL(sblock_init_debugfs);

#endif /* CONFIG_DEBUG_FS */

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SBLOCK driver");
MODULE_LICENSE("GPL");
