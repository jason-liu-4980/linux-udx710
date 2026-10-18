// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/icmp.h>
#include <linux/icmpv6.h>
#include <linux/if_arp.h>
#include <asm/byteorder.h>
#include <linux/tty.h>
#include <linux/platform_device.h>
#include <uapi/linux/sched/types.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of_device.h>
#include <linux/sipa.h>
#include "sipa_priv.h"
#include "sipa_hal.h"

#define SIPA_RECV_BUF_LEN 1600
#define SIPA_RECV_RSVD_LEN 64
#define SIPA_RECV_BATCH 64
#define SIPA_REFILL_RETRY_MS 2

/* One refill producer and one completion consumer. Publish the complete
 * skb/DMA pair before wp, and return the slot only after validating ownership.
 */
static int put_recv_array_node(struct sipa_skb_array *p, struct sk_buff *skb,
			       dma_addr_t dma_addr)
{
	/* Reuse a slot only after the consumer releases its ownership. */
	u32 wp = READ_ONCE(p->wp), rp = smp_load_acquire(&p->rp);

	if (wp - rp >= p->depth)
		return -ENOSPC;
	p->array[wp & (p->depth - 1)].skb = skb;
	p->array[wp & (p->depth - 1)].dma_addr = dma_addr;
	/* Publish the complete skb/DMA pair to the consumer. */
	smp_store_release(&p->wp, wp + 1);
	return 0;
}

static int get_recv_array_node(struct sipa_skb_array *p, dma_addr_t expected,
			       struct sk_buff **skb, dma_addr_t *dma_addr)
{
	/* Read the skb/DMA pair only after the producer publishes it. */
	u32 rp = READ_ONCE(p->rp), wp = smp_load_acquire(&p->wp);
	u32 pos = rp & (p->depth - 1);

	if (rp == wp)
		return -ENOENT;
	*skb = p->array[pos].skb;
	*dma_addr = p->array[pos].dma_addr;
	if (!expected || !*skb || expected != *dma_addr)
		return -EBADMSG;
	/* Finish ownership validation before the producer reuses this slot. */
	smp_store_release(&p->rp, rp + 1);
	return 0;
}

static int sipa_cache_rx_buffer(struct sipa_skb_receiver *receiver,
				struct sk_buff *skb, dma_addr_t dma,
				struct sipa_hal_fifo_item *item)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&receiver->lock, flags);
	if (READ_ONCE(receiver->faulted)) {
		ret = -EIO;
		goto out;
	}
	ret = put_recv_array_node(&receiver->recv_array, skb, dma);
	if (ret)
		goto out;
	ret = sipa_hal_cache_rx_fifo_item(receiver->ctx->hdl,
					  receiver->ep->recv_fifo.idx, item);
	if (ret)
		/* No descriptor was cached/published and the completion consumer
		 * cannot observe this unsubmitted tail while this lock is held.
		 */
		WRITE_ONCE(receiver->recv_array.wp,
			   receiver->recv_array.wp - 1);
out:
	spin_unlock_irqrestore(&receiver->lock, flags);
	return ret;
}

static struct sk_buff *alloc_recv_skb(u32 req_len, u8 rsvd)
{
	struct sk_buff *skb;
	u32 hr;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	skb = __dev_alloc_skb(req_len + rsvd, GFP_KERNEL | GFP_NOWAIT);
	if (!skb) {
		dev_err_ratelimited(ctrl->ctx->pdev, "failed to alloc skb!\n");
		return NULL;
	}

	/* save skb ptr to skb->data */
	hr = skb_headroom(skb);
	if (hr < rsvd)
		skb_reserve(skb, rsvd - hr);

	return skb;
}

static void sipa_prepare_free_node_init(struct sipa_skb_receiver *receiver,
					u32 cnt)
{
	struct sk_buff *skb;
	u32 fail_cnt = 0;
	int i;
	u32 success_cnt = 0;
	struct sipa_hal_fifo_item item = {};
	dma_addr_t dma_addr;

	for (i = 0; i < cnt; i++) {
		skb = alloc_recv_skb(SIPA_RECV_BUF_LEN, receiver->rsvd);
		if (skb) {
			skb_put(skb, SIPA_RECV_BUF_LEN);
			dma_addr = dma_map_single(
				receiver->ctx->pdev, skb->head,
				SIPA_RECV_BUF_LEN + skb_headroom(skb),
				DMA_FROM_DEVICE);
			if (dma_mapping_error(receiver->ctx->pdev, dma_addr)) {
				dev_err(receiver->ctx->pdev,
					"prepare free node dma map err\n");
				dev_kfree_skb_any(skb);
				fail_cnt++;
				break;
			}

			item.addr = dma_addr;
			item.len = skb->len;
			item.offset = skb_headroom(skb);
			item.dst = receiver->ep->recv_fifo.dst_id;
			item.src = receiver->ep->recv_fifo.src_id;
			item.intr = 0;
			item.netid = 0;
			item.err_code = 0;
			if (sipa_cache_rx_buffer(receiver, skb, dma_addr,
						 &item)) {
				dma_unmap_single(receiver->ctx->pdev, dma_addr,
						 SIPA_RECV_BUF_LEN +
							 skb_headroom(skb),
						 DMA_FROM_DEVICE);
				dev_kfree_skb_any(skb);
				fail_cnt++;
				break;
			}
			success_cnt++;
		} else {
			fail_cnt++;
			break;
		}
	}

	if (fail_cnt)
		dev_err(receiver->ctx->pdev,
			"ep->id = %d fail_cnt = %d success_cnt = %d\n",
			receiver->ep->id, fail_cnt, success_cnt);
	/* Initial allocation may be partial; arrange actual later refill. */
	atomic_set(&receiver->need_fill_cnt, cnt - success_cnt);
}

/* A failed publish may already have moved some descriptors to hardware.
 * Retain every DMA owner, stop admission, and never replay an ambiguous batch.
 */
static int sipa_publish_free_nodes(struct sipa_skb_receiver *receiver)
{
	int ret = sipa_hal_put_rx_fifo_items(receiver->ctx->hdl,
					     receiver->ep->recv_fifo.idx);

	if (ret) {
		receiver->refill_publish_failures++;
		WRITE_ONCE(receiver->faulted, true);
		sipa_hal_cmn_fifo_set_receive(
			receiver->ctx->hdl, receiver->ep->recv_fifo.idx, true);
		dev_err(receiver->ctx->pdev,
			"RX refill publish failed fifo %u: %d; DMA owners retained\n",
			receiver->ep->recv_fifo.idx, ret);
	}
	return ret;
}

static int fill_free_fifo(struct sipa_skb_receiver *receiver, u32 cnt)
{
	struct sk_buff *skb;
	u32 success_cnt = 0, depth, i;
	struct sipa_hal_fifo_item item = {};
	dma_addr_t dma_addr;
	int failure = 0, ret;

	if (READ_ONCE(receiver->faulted))
		return -EIO;
	depth = receiver->ep->recv_fifo.rx_fifo.fifo_depth;
	receiver->max_need_fill = max(receiver->max_need_fill, cnt);
	if (cnt > depth - depth / 4) {
		dev_warn_ratelimited(
			receiver->ctx->pdev,
			"ep id = %d free node is not enough,need fill %u\n",
			receiver->ep->id, cnt);
		receiver->rx_danger_cnt++;
	}
	/*
	 * Give the hardware each small batch without waiting for the whole
	 * deficit to be allocated and DMA-mapped.
	 */
	cnt = min_t(u32, cnt, SIPA_RECV_BATCH);
	for (i = 0; i < cnt && !READ_ONCE(receiver->faulted); i++) {
		skb = alloc_recv_skb(SIPA_RECV_BUF_LEN, receiver->rsvd);
		if (!skb) {
			receiver->refill_alloc_failures++;
			failure = -ENOMEM;
			break;
		}
		skb_put(skb, SIPA_RECV_BUF_LEN);
		dma_addr = dma_map_single(receiver->ctx->pdev, skb->head,
					  SIPA_RECV_BUF_LEN + skb_headroom(skb),
					  DMA_FROM_DEVICE);
		if (dma_mapping_error(receiver->ctx->pdev, dma_addr)) {
			dev_kfree_skb_any(skb);
			receiver->refill_map_failures++;
			failure = -ENOMEM;
			break;
		}
		item.addr = dma_addr;
		item.len = skb->len;
		item.offset = skb_headroom(skb);
		item.dst = receiver->ep->recv_fifo.dst_id;
		item.src = receiver->ep->recv_fifo.src_id;
		ret = sipa_cache_rx_buffer(receiver, skb, dma_addr, &item);
		if (ret) {
			dma_unmap_single(receiver->ctx->pdev, dma_addr,
					 SIPA_RECV_BUF_LEN + skb_headroom(skb),
					 DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			receiver->refill_cache_failures++;
			failure = ret;
			break;
		}
		success_cnt++;
	}
	if (success_cnt) {
		ret = sipa_publish_free_nodes(receiver);
		if (ret)
			return ret;
		/* Only hardware-accepted descriptors reduce the deficit. */
		atomic_sub(success_cnt, &receiver->need_fill_cnt);
		receiver->refill_batches++;
		receiver->refilled_buffers += success_cnt;
	}
	return failure;
}

int sipa_fill_free_node(struct sipa_skb_receiver *receiver, u32 cnt)
{
	if (READ_ONCE(receiver->faulted))
		return -EIO;
	if (cnt != receiver->ep->recv_fifo.rx_fifo.fifo_depth)
		return -EINVAL;
	/*
	 * Initial/resume descriptors already own their buffers. They are not
	 * newly consumed RX completions, so do not subtract need_fill_cnt.
	 */
	return sipa_publish_free_nodes(receiver);
}
EXPORT_SYMBOL(sipa_fill_free_node);

static void sipa_receiver_notify_cb(void *priv, enum sipa_hal_evt_type evt,
				    unsigned long data)
{
	struct sipa_skb_receiver *receiver = (struct sipa_skb_receiver *)priv;

	if (evt & SIPA_RECV_EVT)
		wake_up(&receiver->recv_waitq);

	if (evt & SIPA_RECV_WARN_EVT) {
		dev_err(receiver->ctx->pdev,
			"sipa maybe poor resources evt = 0x%x\n", evt);
		receiver->tx_danger_cnt++;
		wake_up(&receiver->recv_waitq);
	}
}

static void trigger_nics_recv(struct sipa_skb_receiver *receiver)
{
	int i;

	for (i = 0; i < receiver->nic_cnt; i++)
		sipa_nic_try_notify_recv(receiver->nic_array[i]);
}

static int dispath_to_nic(struct sipa_skb_receiver *receiver,
			  struct sipa_hal_fifo_item *item, struct sk_buff *skb)
{
	u32 i;
	struct sipa_nic *nic;
	struct sipa_nic *dst_nic = NULL;

	for (i = 0; i < receiver->nic_cnt; i++) {
		nic = receiver->nic_array[i];
		if (atomic_read(&nic->status) != NIC_OPEN)
			continue;
		if ((nic->src_mask & BIT(item->src)) &&
		    (nic->netid == -1 || nic->netid == item->netid)) {
			dst_nic = nic;
			break;
		}
	}

	if (dst_nic) {
		sipa_nic_push_skb(dst_nic, skb);
	} else {
		dev_err(receiver->ctx->pdev,
			"dispath to nic src:0x%x, netid:%d no nic matched\n",
			item->src, item->netid);
		dev_kfree_skb_any(skb);
	}

	return 0;
}

static int do_recv(struct sipa_skb_receiver *receiver)
{
	int ret;
	u32 i;
	unsigned long flags;
	u32 num = 0, depth = 0;
	dma_addr_t addr;
	struct sk_buff *recv_skb = NULL;
	struct sipa_hal_fifo_item item = {};
	enum sipa_cmn_fifo_index id = receiver->ep->recv_fifo.idx;

	depth = receiver->ep->recv_fifo.tx_fifo.fifo_depth;
	num = sipa_hal_get_tx_fifo_items(receiver->ctx->hdl,
					 receiver->ep->recv_fifo.idx);

	if (num > depth) {
		WRITE_ONCE(receiver->faulted, true);
		sipa_hal_cmn_fifo_set_receive(receiver->ctx->hdl, id, true);
		dev_err(receiver->ctx->pdev,
			"RX count %u exceeds depth %u; quarantined\n", num,
			depth);
		return 0;
	}

	if (num > (depth - depth / 4)) {
		dev_warn(receiver->ctx->pdev,
			 "ep id %d tx fifo not read in time num = %d\n",
			 receiver->ep->id, num);
		receiver->tx_danger_cnt++;
	}

	/*
	 * Bound completion processing so refill and NAPI are notified before
	 * a large backlog has been completely drained.
	 */
	num = min_t(u32, num, SIPA_RECV_BATCH);
	receiver->max_recv_batch = max(receiver->max_recv_batch, num);
	for (i = 0; i < num; i++) {
		sipa_hal_recv_conversion_node_to_item(receiver->ctx->hdl, id,
						      &item, i);

		spin_lock_irqsave(&receiver->lock, flags);
		ret = get_recv_array_node(&receiver->recv_array, item.addr,
					  &recv_skb, &addr);
		if (ret)
			WRITE_ONCE(receiver->faulted, true);
		spin_unlock_irqrestore(&receiver->lock, flags);
		if (ret) {
			/* Do not pop a different skb, unmap an unproven DMA owner,
			 * or acknowledge the corrupt slot. Preserve the remaining
			 * bounded queue until a future verified reset/drain path.
			 */
			WRITE_ONCE(receiver->faulted, true);
			dev_err(receiver->ctx->pdev,
				"RX ownership fault fifo %u slot %u addr %llx: %d\n",
				id, i, item.addr, ret);
			ret = sipa_hal_cmn_fifo_set_receive(receiver->ctx->hdl,
							    id, true);
			if (ret)
				dev_err(receiver->ctx->pdev,
					"RX stop request failed: %d; DMA remains quarantined\n",
					ret);
			break;
		}

		dma_unmap_single(receiver->ctx->pdev, addr,
				 SIPA_RECV_BUF_LEN + skb_headroom(recv_skb),
				 DMA_FROM_DEVICE);

		if (item.len > SIPA_RECV_BUF_LEN) {
			dev_err_ratelimited(receiver->ctx->pdev,
					    "invalid RX length %u\n", item.len);
			dev_kfree_skb_any(recv_skb);
			continue;
		}
		skb_trim(recv_skb, item.len);

		skb_reset_network_header(recv_skb);

		dispath_to_nic(receiver, &item, recv_skb);
	}

	/* Acknowledge only completions whose DMA ownership was established. */
	if (i)
		sipa_hal_set_tx_fifo_rptr(receiver->ctx->hdl, id, i);
	return i;
}

static int fill_recv_thread(void *data)
{
	int ret;
	struct sipa_skb_receiver *receiver = (struct sipa_skb_receiver *)data;
	struct sched_param param = { .sched_priority = 92 };

	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			receiver->fill_recv_waitq,
			kthread_should_stop() ||
				(!READ_ONCE(receiver->faulted) &&
				 atomic_read(&receiver->need_fill_cnt) > 0));
		if (kthread_should_stop())
			break;
		if (!ret) {
			ret = fill_free_fifo(
				receiver,
				atomic_read(&receiver->need_fill_cnt));
			/*
			 * Allocation/cache pressure must not turn this priority-92
			 * worker into an unbounded retry loop.
			 */
			if (ret && !READ_ONCE(receiver->faulted))
				msleep_interruptible(SIPA_REFILL_RETRY_MS);
		}
	}

	return 0;
}

static int recv_thread(void *data)
{
	struct sipa_skb_receiver *receiver = (struct sipa_skb_receiver *)data;
	struct sched_param param = { .sched_priority = 90 };

	/*set the thread as a real time thread, and its priority is 90*/
	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		u32 recv_cnt = 0;

		if (wait_event_interruptible(
			    receiver->recv_waitq,
			    kthread_should_stop() ||
				    (!READ_ONCE(receiver->faulted) &&
				     !sipa_hal_is_tx_fifo_empty(
					     receiver->ctx->hdl,
					     receiver->ep->recv_fifo.idx))))
			continue;
		if (kthread_should_stop())
			break;
		recv_cnt = do_recv(receiver);
		atomic_add(recv_cnt, &receiver->need_fill_cnt);
		if (atomic_read(&receiver->need_fill_cnt) > 0x30)
			wake_up(&receiver->fill_recv_waitq);

		trigger_nics_recv(receiver);
	}

	return 0;
}

int sipa_receiver_prepare_suspend(struct sipa_skb_receiver *receiver)
{
	if (!sipa_hal_is_tx_fifo_empty(receiver->ctx->hdl,
				       receiver->ep->recv_fifo.idx)) {
		pr_err("sipa recv fifo %d tx fifo is not empty\n",
		       receiver->ep->recv_fifo.idx);
		wake_up(&receiver->recv_waitq);
		return -EAGAIN;
	}

	if (atomic_read(&receiver->need_fill_cnt)) {
		pr_err("sipa recv fifo %d need_fill_cnt = %d\n",
		       receiver->ep->recv_fifo.idx,
		       atomic_read(&receiver->need_fill_cnt));
		wake_up(&receiver->fill_recv_waitq);
		return -EAGAIN;
	}

	return sipa_hal_cmn_fifo_set_receive(receiver->ctx->hdl,
					     receiver->ep->recv_fifo.idx, true);
}
EXPORT_SYMBOL(sipa_receiver_prepare_suspend);

int sipa_receiver_prepare_resume(struct sipa_skb_receiver *receiver)
{
	if (READ_ONCE(receiver->faulted))
		return -EIO;
	if (unlikely(receiver->init_flag)) {
		dev_info(receiver->ctx->pdev, "receiver %d wake up thread\n",
			 receiver->ep->id);
		wake_up_process(receiver->thread);
		wake_up_process(receiver->fill_thread);
		receiver->init_flag = false;
	}

	return sipa_hal_cmn_fifo_set_receive(
		receiver->ctx->hdl, receiver->ep->recv_fifo.idx, false);
}
EXPORT_SYMBOL(sipa_receiver_prepare_resume);

static void sipa_receiver_init(struct sipa_skb_receiver *receiver, u32 rsvd)
{
	u32 depth;
	struct sipa_comm_fifo_params attr = {};

	/* timeout = 1 / ipa_sys_clk * 1024 * value */
	attr.tx_intr_delay_us = 0x64;
	attr.tx_intr_threshold = 0x30;
	attr.flowctrl_in_tx_full = true;
	attr.flow_ctrl_cfg = flow_ctrl_rx_empty;
	attr.flow_ctrl_irq_mode = enter_exit_flow_ctrl;
	attr.rx_enter_flowctrl_watermark =
		receiver->ep->recv_fifo.rx_fifo.fifo_depth / 4;
	attr.rx_leave_flowctrl_watermark =
		receiver->ep->recv_fifo.rx_fifo.fifo_depth / 2;
	attr.tx_enter_flowctrl_watermark = 0;
	attr.tx_leave_flowctrl_watermark = 0;

	dev_info(receiver->ctx->pdev,
		 "ep_id = %d fifo_id = %d rx_fifo depth = 0x%x\n",
		 receiver->ep->id, receiver->ep->recv_fifo.idx,
		 receiver->ep->recv_fifo.rx_fifo.fifo_depth);
	dev_info(receiver->ctx->pdev, "recv status is %d\n",
		 receiver->ep->recv_fifo.is_receiver);
	sipa_open_common_fifo(receiver->ctx->hdl, receiver->ep->recv_fifo.idx,
			      &attr, NULL, true, sipa_receiver_notify_cb,
			      receiver);

	/* reserve space for dma flushing cache issue */
	receiver->rsvd = rsvd;
	receiver->init_flag = true;
	depth = receiver->ep->recv_fifo.rx_fifo.fifo_depth;

	sipa_prepare_free_node_init(receiver, depth);
}

void sipa_receiver_add_nic(struct sipa_skb_receiver *receiver,
			   struct sipa_nic *nic)
{
	int i;
	unsigned long flags;

	for (i = 0; i < receiver->nic_cnt; i++)
		if (receiver->nic_array[i] == nic)
			return;
	spin_lock_irqsave(&receiver->lock, flags);
	if (receiver->nic_cnt < SIPA_NIC_MAX)
		receiver->nic_array[receiver->nic_cnt++] = nic;
	spin_unlock_irqrestore(&receiver->lock, flags);
}
EXPORT_SYMBOL(sipa_receiver_add_nic);

static int create_recv_array(struct sipa_skb_array *p, u32 depth)
{
	if (!is_power_of_2(depth))
		return -EINVAL;
	p->array = kzalloc_objs(*p->array, depth, GFP_KERNEL);
	if (!p->array)
		return -ENOMEM;
	p->rp = 0;
	p->wp = 0;
	p->depth = depth;

	return 0;
}

static void destroy_recv_array(struct sipa_skb_array *p)
{
	kfree(p->array);

	p->array = NULL;
	p->rp = 0;
	p->wp = 0;
	p->depth = 0;
}

int create_sipa_skb_receiver(struct sipa_context *ipa, struct sipa_endpoint *ep,
			     struct sipa_skb_receiver **receiver_pp)
{
	int ret;
	struct sipa_skb_receiver *receiver = NULL;

	dev_info(ipa->pdev, "ep->id = %d start\n", ep->id);
	receiver = kzalloc_obj(struct sipa_skb_receiver, GFP_KERNEL);
	if (!receiver)
		return -ENOMEM;

	receiver->ctx = ipa;
	receiver->ep = ep;
	receiver->rsvd = SIPA_RECV_RSVD_LEN;

	atomic_set(&receiver->need_fill_cnt, 0);

	ret = create_recv_array(&receiver->recv_array,
				receiver->ep->recv_fifo.rx_fifo.fifo_depth);
	if (ret) {
		dev_err(ipa->pdev,
			"create_sipa_sipa_receiver: recv_array kzalloc err.\n");
		kfree(receiver);
		return -ENOMEM;
	}

	spin_lock_init(&receiver->lock);
	init_waitqueue_head(&receiver->recv_waitq);
	init_waitqueue_head(&receiver->fill_recv_waitq);

	/* Create both dormant workers before publishing FIFO callbacks. */
	receiver->thread =
		kthread_create(recv_thread, receiver, "sipa-recv-%d", ep->id);
	if (IS_ERR(receiver->thread)) {
		dev_err(ipa->pdev, "Failed to create kthread: ipa-recv-%d\n",
			ep->id);
		ret = PTR_ERR(receiver->thread);
		kfree(receiver->recv_array.array);
		kfree(receiver);
		return ret;
	}

	receiver->fill_thread = kthread_create(fill_recv_thread, receiver,
					       "sipa-fill-%d", ep->id);
	if (IS_ERR(receiver->fill_thread)) {
		kthread_stop(receiver->thread);
		dev_err(ipa->pdev, "Failed to create kthread: ipa-fill-%d\n",
			ep->id);
		ret = PTR_ERR(receiver->fill_thread);
		kfree(receiver->recv_array.array);
		kfree(receiver);
		return ret;
	}

	sipa_receiver_init(receiver, SIPA_RECV_RSVD_LEN);
	*receiver_pp = receiver;
	return 0;
}
EXPORT_SYMBOL(create_sipa_skb_receiver);

void destroy_sipa_skb_receiver(struct sipa_skb_receiver *receiver)
{
	if (receiver->recv_array.array)
		destroy_recv_array(&receiver->recv_array);

	kfree(receiver);
}
EXPORT_SYMBOL(destroy_sipa_skb_receiver);
