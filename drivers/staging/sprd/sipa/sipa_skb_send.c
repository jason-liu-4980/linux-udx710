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
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of_device.h>
#include <linux/sipa.h>
#include <uapi/linux/sched/types.h>

#include "sipa_priv.h"
#include "sipa_hal.h"

#define SIPA_RECEIVER_BUF_LEN 1600

static void sipa_inform_evt_to_nics(struct sipa_skb_sender *sender,
				    enum sipa_evt_type evt)
{
	unsigned long flags;
	struct sipa_nic *nic;

	spin_lock_irqsave(&sender->nic_lock, flags);
	list_for_each_entry(nic, &sender->nic_list, list) {
		if (nic->flow_ctrl_status) {
			nic->flow_ctrl_status = false;
			sipa_nic_notify_evt(nic, evt);
		}
	}
	spin_unlock_irqrestore(&sender->nic_lock, flags);
}

static void sipa_sender_notify_cb(void *priv, enum sipa_hal_evt_type evt,
				  unsigned long data)
{
	struct sipa_skb_sender *sender = (struct sipa_skb_sender *)priv;

	if (evt & SIPA_RECV_EVT)
		wake_up(&sender->free_waitq);

	if (evt & SIPA_HAL_TXFIFO_OVERFLOW)
		dev_err(sender->ctx->pdev, "sipa overflow on ep:%d\n",
			sender->ep->id);

	if (evt & SIPA_HAL_ENTER_FLOW_CTRL)
		sender->enter_flow_ctrl_cnt++;

	if (evt & SIPA_HAL_EXIT_FLOW_CTRL)
		sender->exit_flow_ctrl_cnt++;
}

static void sipa_free_sent_items(struct sipa_skb_sender *sender)
{
	unsigned long flags;
	u32 i = 0, num, success_cnt = 0;
	struct sipa_skb_dma_addr_node *iter, *found;
	struct sipa_hal_fifo_item item;
	int ret;

	num = sipa_hal_get_tx_fifo_items(sender->ctx->hdl,
					 sender->ep->send_fifo.idx);
	if (num > sender->ep->send_fifo.tx_fifo.fifo_depth) {
		WRITE_ONCE(sender->faulted, true);
		goto fault;
	}
	for (i = 0; i < num; i++) {
		sipa_hal_recv_conversion_node_to_item(
			sender->ctx->hdl, sender->ep->send_fifo.idx, &item, i);
		found = NULL;
		spin_lock_irqsave(&sender->send_lock, flags);
		list_for_each_entry(iter, &sender->sending_list, list) {
			if (iter->dma_addr == item.addr) {
				found = iter;
				list_del(&found->list);
				break;
			}
		}
		if (!found)
			WRITE_ONCE(sender->faulted, true);
		spin_unlock_irqrestore(&sender->send_lock, flags);
		if (!found)
			break;

		/* This pair remains private until both its mapping and skb have
		 * been released. Publishing it on pair_free_list earlier lets a
		 * parallel retry overwrite fields still used by this completion.
		 */
		dma_unmap_single(sender->ctx->pdev, found->dma_addr,
				 found->dma_len, DMA_TO_DEVICE);
		dev_kfree_skb_any(found->skb);
		/* Keep the endpoint referenced while completion is outside the lock. */
		if (found->pending)
			atomic_dec_return_release(found->pending);
		spin_lock_irqsave(&sender->send_lock, flags);
		found->skb = NULL;
		found->pending = NULL;
		found->dma_addr = 0;
		found->dma_len = 0;
		list_add_tail(&found->list, &sender->pair_free_list);
		atomic_inc(&sender->left_cnt);
		spin_unlock_irqrestore(&sender->send_lock, flags);
		success_cnt++;
	}
	if (i) {
		ret = sipa_hal_set_tx_fifo_rptr(sender->ctx->hdl,
						sender->ep->send_fifo.idx, i);
		if (ret)
			WRITE_ONCE(sender->faulted, true);
	}
	if (READ_ONCE(sender->faulted))
		goto fault;
	if (sender->free_notify_net &&
	    atomic_read(&sender->left_cnt) >
		    sender->ep->send_fifo.rx_fifo.fifo_depth / 4) {
		sender->free_notify_net = false;
		sipa_inform_evt_to_nics(sender, SIPA_LEAVE_FLOWCTRL);
	}
	return;
fault:
	dev_err(sender->ctx->pdev,
		"TX ownership/pointer fault fifo %u completed %u/%u; mappings quarantined\n",
		sender->ep->send_fifo.idx, success_cnt, num);
	ret = sipa_hal_cmn_fifo_set_receive(sender->ctx->hdl,
					    sender->ep->send_fifo.idx, true);
	if (ret)
		dev_err(sender->ctx->pdev, "TX stop request failed: %d\n", ret);
}

static int sipa_send_thread(void *data)
{
	int ret;
	struct sipa_skb_sender *sender = data;
	struct sched_param param = { .sched_priority = 90 };

	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			sender->send_waitq,
			kthread_should_stop() ||
				(!READ_ONCE(sender->faulted) &&
				 !sipa_hal_check_rx_priv_fifo_is_empty(
					 sender->ctx->hdl,
					 sender->ep->send_fifo.idx)));
		if (kthread_should_stop())
			break;
		if (!ret) {
			ret = sipa_hal_put_rx_fifo_items(
				sender->ctx->hdl, sender->ep->send_fifo.idx);
			if (ret) {
				WRITE_ONCE(sender->faulted, true);
				sipa_hal_cmn_fifo_set_receive(
					sender->ctx->hdl,
					sender->ep->send_fifo.idx, true);
				dev_err(sender->ctx->pdev,
					"TX publish failed fifo %u: %d; DMA owners retained\n",
					sender->ep->send_fifo.idx, ret);
			}
		}

		if (sender->free_notify_net)
			wake_up(&sender->free_waitq);
	}

	return 0;
}

/*
 * Wait for an actual completion, or notify a queue which already recovered.
 * Merely requesting free space must not create an RT90 busy-poll loop.
 */
static bool sipa_free_work_pending(struct sipa_skb_sender *sender)
{
	return !READ_ONCE(sender->faulted) &&
	       (!sipa_hal_is_tx_fifo_empty(sender->ctx->hdl,
					   sender->ep->send_fifo.idx) ||
		(READ_ONCE(sender->free_notify_net) &&
		 atomic_read(&sender->left_cnt) >
			 (int)(sender->ep->send_fifo.rx_fifo.fifo_depth / 4)));
}

static int sipa_free_thread(void *data)
{
	int ret;
	struct sipa_skb_sender *sender = (struct sipa_skb_sender *)data;
	struct sched_param param = { .sched_priority = 90 };

	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			sender->free_waitq,
			kthread_should_stop() ||
				sipa_free_work_pending(sender));
		if (kthread_should_stop())
			break;
		if (!ret)
			sipa_free_sent_items(sender);
	}

	return 0;
}

static int sipa_skb_sender_init(struct sipa_skb_sender *sender)
{
	struct sipa_comm_fifo_params attr = {};

	attr.tx_intr_delay_us = 500;
	attr.tx_intr_threshold = 32;
	attr.flow_ctrl_cfg = flow_ctrl_tx_full;
	attr.flowctrl_in_tx_full = true;
	attr.flow_ctrl_irq_mode = enter_exit_flow_ctrl;
	attr.rx_enter_flowctrl_watermark = 0;
	attr.rx_leave_flowctrl_watermark = 0;
	attr.tx_enter_flowctrl_watermark = 0;
	attr.tx_leave_flowctrl_watermark = 0;

	sipa_open_common_fifo(sender->ctx->hdl, sender->ep->send_fifo.idx,
			      &attr, NULL, true, sipa_sender_notify_cb, sender);
	sender->init_flag = true;

	return 0;
}

int sipa_sender_prepare_suspend(struct sipa_skb_sender *sender)
{
	if (!list_empty(&sender->sending_list)) {
		pr_err("pkt_type = %d sending list have unsend node\n",
		       sender->type);
		wake_up(&sender->free_waitq);
		return -EAGAIN;
	}

	if (!sipa_hal_check_rx_priv_fifo_is_empty(sender->ctx->hdl,
						  sender->ep->send_fifo.idx)) {
		pr_err("pkt_type = %d rx priv fifo is not empty\n",
		       sender->type);
		wake_up(&sender->send_waitq);
		return -EAGAIN;
	}

	if (!sipa_hal_check_send_cmn_fifo_com(sender->ctx->hdl,
					      sender->ep->send_fifo.idx)) {
		pr_err("pkt_type = %d sender have something to handle\n",
		       sender->type);
		return -EAGAIN;
	}

	return 0;
}
EXPORT_SYMBOL(sipa_sender_prepare_suspend);

int sipa_sender_prepare_resume(struct sipa_skb_sender *sender)
{
	if (READ_ONCE(sender->faulted))
		return -EIO;
	if (unlikely(sender->init_flag)) {
		wake_up_process(sender->send_thread);
		wake_up_process(sender->free_thread);
		sender->init_flag = false;
	}

	return 0;
}
EXPORT_SYMBOL(sipa_sender_prepare_resume);

int create_sipa_skb_sender(struct sipa_context *ipa, struct sipa_endpoint *ep,
			   enum sipa_xfer_pkt_type type,
			   struct sipa_skb_sender **sender_pp)
{
	int i, ret;
	struct sipa_skb_sender *sender = NULL;

	dev_info(ipa->pdev, "%s ep->id = %d start\n", __func__, ep->id);
	sender = kzalloc_obj(*sender, GFP_KERNEL);
	if (!sender)
		return -ENOMEM;

	sender->pair_cache = kcalloc(ep->send_fifo.rx_fifo.fifo_depth,
				     sizeof(struct sipa_skb_dma_addr_node),
				     GFP_KERNEL);
	if (!sender->pair_cache) {
		kfree(sender);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&sender->nic_list);
	INIT_LIST_HEAD(&sender->sending_list);
	INIT_LIST_HEAD(&sender->pair_free_list);
	spin_lock_init(&sender->nic_lock);
	spin_lock_init(&sender->send_lock);

	for (i = 0; i < ep->send_fifo.rx_fifo.fifo_depth; i++)
		list_add_tail(&((sender->pair_cache + i)->list),
			      &sender->pair_free_list);

	sender->ctx = ipa;
	sender->ep = ep;
	sender->type = type;

	atomic_set(&sender->left_cnt, ep->send_fifo.rx_fifo.fifo_depth);

	init_waitqueue_head(&sender->send_waitq);
	init_waitqueue_head(&sender->free_waitq);

	/* create sender thread */
	sender->send_thread = kthread_create(sipa_send_thread, sender,
					     "sipa-send-%d", ep->id);
	if (IS_ERR(sender->send_thread)) {
		dev_err(ipa->pdev, "Failed to create kthread: ipa-send-%d\n",
			ep->id);
		ret = PTR_ERR(sender->send_thread);
		kfree(sender->pair_cache);
		kfree(sender);
		return ret;
	}

	sender->free_thread = kthread_create(sipa_free_thread, sender,
					     "sipa-free-%d", ep->id);
	if (IS_ERR(sender->free_thread)) {
		kthread_stop(sender->send_thread);
		dev_err(ipa->pdev, "Failed to create kthread: ipa-free-%d\n",
			ep->id);
		ret = PTR_ERR(sender->free_thread);
		kfree(sender->pair_cache);
		kfree(sender);
		return ret;
	}

	/* All waitqueues and both dormant threads must exist before IRQ
	 * callback registration. Otherwise thread allocation failure leaves
	 * hardware callbacks pointing to a freed sender.
	 */
	sipa_skb_sender_init(sender);
	*sender_pp = sender;
	return 0;
}
EXPORT_SYMBOL(create_sipa_skb_sender);

void destroy_sipa_skb_sender(struct sipa_skb_sender *sender)
{
	kfree(sender->pair_cache);
	kfree(sender);
}
EXPORT_SYMBOL(destroy_sipa_skb_sender);

void sipa_skb_sender_add_nic(struct sipa_skb_sender *sender,
			     struct sipa_nic *nic)
{
	unsigned long flags;

	spin_lock_irqsave(&sender->nic_lock, flags);
	list_add_tail(&nic->list, &sender->nic_list);
	spin_unlock_irqrestore(&sender->nic_lock, flags);
}
EXPORT_SYMBOL(sipa_skb_sender_add_nic);

void sipa_skb_sender_remove_nic(struct sipa_skb_sender *sender,
				struct sipa_nic *nic)
{
	unsigned long flags;

	spin_lock_irqsave(&sender->nic_lock, flags);
	list_del(&nic->list);
	spin_unlock_irqrestore(&sender->nic_lock, flags);
}
EXPORT_SYMBOL(sipa_skb_sender_remove_nic);

int sipa_skb_sender_send_data(struct sipa_skb_sender *sender,
			      struct sk_buff *skb, enum sipa_term_type dst,
			      u8 netid, atomic_t *pending)
{
	unsigned long flags;
	int ret;
	dma_addr_t dma_addr;
	size_t dma_len;
	struct sipa_skb_dma_addr_node *node;
	struct sipa_hal_fifo_item item;
	bool counted = false;

	if (READ_ONCE(sender->faulted))
		return -EIO;
	if (!atomic_read(&sender->left_cnt)) {
		sender->no_free_cnt++;
		return -EAGAIN;
	}
	/* Reserve one slot atomically across concurrent netdev queues. */
	if (atomic_dec_if_positive(&sender->left_cnt) < 0)
		return -EAGAIN;

	/* This descriptor describes one contiguous linear DMA mapping. */
	if (skb_is_nonlinear(skb) || skb->len >= BIT(20) ||
	    skb_headroom(skb) >= BIT(12)) {
		atomic_inc(&sender->left_cnt);
		return -EINVAL;
	}
	/* PAM rounds the final payload TRB up to align the NTB's NDP. A
	 * snapshot of seven 1514-byte frames showed its last TRB reading 1516
	 * bytes. Reserve and zero all possible tail bytes without changing the
	 * Ethernet length carried by the SIPA descriptor. Any frame can become
	 * the final one, and preceding odd-sized frames affect the padding.
	 */
	if (dst == SIPA_TERM_USB) {
		ret = __skb_pad(skb, 3, false);
		if (ret) {
			atomic_inc(&sender->left_cnt);
			return ret; /* The caller still owns the skb on failure. */
		}
	}
	dma_len = skb_headroom(skb) + skb->len + (dst == SIPA_TERM_USB ? 3 : 0);

	dma_addr = dma_map_single(sender->ctx->pdev, skb->head, dma_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(sender->ctx->pdev, dma_addr)) {
		atomic_inc(&sender->left_cnt);
		return -ENOMEM;
	}

	memset(&item, 0, sizeof(item));
	item.addr = dma_addr;
	item.len = skb->len;
	item.offset = skb_headroom(skb);
	item.netid = netid;
	item.dst = dst;
	item.src = sender->ep->send_fifo.src_id;

	spin_lock_irqsave(&sender->send_lock, flags);
	if (READ_ONCE(sender->faulted) || list_empty(&sender->pair_free_list)) {
		ret = -EIO;
		goto unmap;
	}
	node = list_first_entry(&sender->pair_free_list,
				struct sipa_skb_dma_addr_node, list);
	list_del(&node->list);
	node->skb = skb;
	node->dma_addr = dma_addr;
	node->dma_len = dma_len;
	node->pending = pending;
	/* Count before cache publication: the flush thread can run immediately. */
	if (pending) {
		atomic_inc(pending);
		counted = true;
	}
	list_add_tail(&node->list, &sender->sending_list);
	ret = sipa_hal_cache_rx_fifo_item(sender->ctx->hdl,
					  sender->ep->send_fifo.idx, &item);
	if (ret) {
		list_move_tail(&node->list, &sender->pair_free_list);
		node->skb = NULL;
		node->dma_addr = 0;
		node->dma_len = 0;
		node->pending = NULL;
		ret = -EAGAIN;
		goto unmap;
	}
	spin_unlock_irqrestore(&sender->send_lock, flags);

	wake_up(&sender->send_waitq);

	return 0;
unmap:
	spin_unlock_irqrestore(&sender->send_lock, flags);
	dma_unmap_single(sender->ctx->pdev, dma_addr, dma_len, DMA_TO_DEVICE);
	if (counted)
		atomic_dec_return_release(pending);
	atomic_inc(&sender->left_cnt);
	return ret;
}
EXPORT_SYMBOL(sipa_skb_sender_send_data);

bool sipa_skb_sender_check_send_complete(struct sipa_skb_sender *sender)
{
	return sipa_hal_check_send_cmn_fifo_com(sender->ctx->hdl,
						sender->ep->send_fifo.idx);
}
EXPORT_SYMBOL(sipa_skb_sender_check_send_complete);
