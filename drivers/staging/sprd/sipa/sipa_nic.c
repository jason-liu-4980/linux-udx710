// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/sipa.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rcupdate.h>
#include <linux/mutex.h>
#include "sipa_priv.h"
#include "sipa_hal.h"

#define SIPA_CP_SRC                                                            \
	((1 << SIPA_TERM_VAP0) | (1 << SIPA_TERM_VAP1) |                       \
	 (1 << SIPA_TERM_VAP2) | (1 << SIPA_TERM_CP0) | (1 << SIPA_TERM_CP1) | \
	 (1 << SIPA_TERM_VCP))

#define SIPA_NIC_RM_INACTIVE_TIMER 1000

/* Open/close are process-context operations; callbacks must not close. */
static DEFINE_MUTEX(sipa_nic_lifecycle_lock);
struct sipa_nic_statics_info {
	enum sipa_ep_id send_ep;
	enum sipa_xfer_pkt_type pkt_type;
	u32 src_mask;
	int netid;
	enum sipa_rm_res_id cons;
};

static struct sipa_nic_statics_info s_spia_nic_statics[SIPA_NIC_MAX] = {
	{
		.send_ep = SIPA_EP_AP_ETH,
		.pkt_type = SIPA_PKT_ETH,
		.src_mask = (1 << SIPA_TERM_USB),
		.netid = -1,
		.cons = SIPA_RM_RES_CONS_USB,
	},
	{
		.send_ep = SIPA_EP_AP_ETH,
		.pkt_type = SIPA_PKT_ETH,
		.src_mask = (1 << SIPA_TERM_WIFI),
		.netid = -1,
		.cons = SIPA_RM_RES_CONS_WLAN,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 0,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 1,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 2,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 3,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 4,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 5,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 6,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
	{
		.send_ep = SIPA_EP_AP_IP,
		.pkt_type = SIPA_PKT_IP,
		.src_mask = SIPA_CP_SRC,
		.netid = 7,
		.cons = SIPA_RM_RES_CONS_WWAN_UL,
	},
};

/* The built-in SIPA device owns these registered consumers for its lifetime.
 * Every request reaching consumer_request adds one reference, even on a hard
 * producer error; consumer_release consumes it even when returning EINPROGRESS.
 */
static int sipa_nic_drop_lease_locked(struct sipa_nic_cons_res *res)
{
	int ret;

	if (!res->lease_held)
		return 0;
	ret = sipa_rm_release_resource(res->cons);
	res->lease_held = false;
	res->need_request = true;
	res->request_in_progress = false;
	if (ret && ret != -EINPROGRESS) {
		/* Never retry a consumed reference, or invent a successful grant. */
		res->error = ret;
		pr_err("sipa NIC consumer %d release fault %d\n", res->cons,
		       ret);
		return ret;
	}
	return 0;
}

static bool sipa_nic_send_complete(struct sipa_nic_cons_res *res)
{
	struct sipa_skb_sender *sender = res->chk_priv;

	/* Pointer equality alone does not retire quarantined skb/DMA pairs. */
	return !READ_ONCE(sender->faulted) &&
	       atomic_read(&sender->left_cnt) ==
		       sender->ep->send_fifo.rx_fifo.fifo_depth &&
	       res->chk_func(res->chk_priv);
}

static void sipa_nic_schedule_release_locked(struct sipa_nic_cons_res *res)
{
	res->release_in_progress = true;
	mod_delayed_work(system_unbound_wq, &res->work, res->jiffies);
}

static void sipa_nic_grant_work(struct work_struct *work)
{
	struct sipa_nic_cons_res *res =
		container_of(work, struct sipa_nic_cons_res, grant_work);
	struct sipa_nic *nic = container_of(res, struct sipa_nic, rm_res);
	unsigned long flags;
	bool notify = false;
	int ret, release_ret;

	spin_lock_irqsave(&res->lock, flags);
	if (!res->lease_held || res->closing || res->error)
		goto unlock;
	/* RM queues unversioned notifications. Revalidate a possibly stale grant
	 * by a temporary counted request, then balance exactly that reference.
	 * Our original lease prevents this probe from releasing a real grant.
	 */
	ret = sipa_rm_request_resource(res->cons);
	release_ret = sipa_rm_release_resource(res->cons);
	if (release_ret && release_ret != -EINPROGRESS) {
		res->error = release_ret;
		res->closing = true;
		sipa_nic_schedule_release_locked(res);
		goto unlock;
	}
	if (ret == -EINPROGRESS)
		goto unlock;
	if (ret) {
		res->error = ret;
		res->closing = true;
		sipa_nic_schedule_release_locked(res);
		goto unlock;
	}
	res->request_in_progress = false;
	notify = true;
	sipa_nic_schedule_release_locked(res);
unlock:
	spin_unlock_irqrestore(&res->lock, flags);
	if (notify)
		sipa_nic_notify_evt(nic, SIPA_LEAVE_FLOWCTRL);
}

static void sipa_nic_rm_notify_cb(void *user_data, enum sipa_rm_event event,
				  unsigned long data)
{
	struct sipa_nic *nic = user_data;

	/* Invoked under RM's spinlock: never acquire res->lock or call a client. */
	if (event == SIPA_RM_EVT_GRANTED &&
	    atomic_read(&nic->status) == NIC_OPEN)
		schedule_work(&nic->rm_res.grant_work);
}

static int sipa_nic_register_rm(struct sipa_nic *nic, enum sipa_nic_id id)
{
	struct sipa_rm_register_params param = {
		.user_data = nic,
		.notify_cb = sipa_nic_rm_notify_cb,
	};

	return sipa_rm_register(s_spia_nic_statics[id].cons, &param);
}

static int sipa_nic_deregister_rm(struct sipa_nic *nic, enum sipa_nic_id id)
{
	struct sipa_rm_register_params param = {
		.user_data = nic,
		.notify_cb = sipa_nic_rm_notify_cb,
	};

	return sipa_rm_deregister(s_spia_nic_statics[id].cons, &param);
}

static void sipa_nic_rm_timer_func(struct work_struct *work)
{
	struct sipa_nic_cons_res *res = container_of(
		to_delayed_work(work), struct sipa_nic_cons_res, work);
	unsigned long flags;

	spin_lock_irqsave(&res->lock, flags);
	if (!res->lease_held) {
		res->closing = false;
		res->release_in_progress = false;
	} else if (res->active_users || (!res->request_in_progress &&
					 !sipa_nic_send_complete(res))) {
		/* This persistent NIC/sender owns the lease after netdev detachment.
		 * No unknown hardware DMA is freed to make close appear complete.
		 */
		sipa_nic_schedule_release_locked(res);
	} else if (res->request_in_progress && !res->closing) {
		/* A waiting TX is woken only by a real, revalidated grant. */
		res->release_in_progress = false;
	} else {
		sipa_nic_drop_lease_locked(res);
		res->closing = false;
		res->release_in_progress = false;
	}
	spin_unlock_irqrestore(&res->lock, flags);
}

static void sipa_nic_rm_init(struct sipa_nic_cons_res *res,
			     struct sipa_skb_sender *sender,
			     enum sipa_rm_res_id cons)
{
	spin_lock_init(&res->lock);
	res->cons = cons;
	res->chk_func =
		(sipa_check_send_completed)sipa_skb_sender_check_send_complete;
	res->chk_priv = sender;
	res->jiffies = msecs_to_jiffies(SIPA_NIC_RM_INACTIVE_TIMER);
	res->need_request = true;
	INIT_DELAYED_WORK(&res->work, sipa_nic_rm_timer_func);
	INIT_WORK(&res->grant_work, sipa_nic_grant_work);
	res->initied = true;
}

static int sipa_nic_rm_res_request(struct sipa_nic *nic)
{
	struct sipa_nic_cons_res *res = &nic->rm_res;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&res->lock, flags);
	if (res->error || res->closing) {
		ret = res->error ?: -ESHUTDOWN;
		goto out;
	}
	if (res->need_request) {
		ret = sipa_rm_request_resource(res->cons);
		res->lease_held = true;
		res->need_request = false;
		res->request_in_progress = ret == -EINPROGRESS;
		if (ret && ret != -EINPROGRESS) {
			int release_ret = sipa_nic_drop_lease_locked(res);

			/* The next TX must perform a new request, never return fake 0. */
			if (release_ret)
				ret = release_ret;
			goto out;
		}
	} else {
		ret = res->request_in_progress ? -EINPROGRESS : 0;
	}
	if (!ret)
		res->active_users++;
out:
	spin_unlock_irqrestore(&res->lock, flags);
	return ret;
}

static void sipa_nic_rm_res_release(struct sipa_nic *nic)
{
	struct sipa_nic_cons_res *res = &nic->rm_res;
	unsigned long flags;

	spin_lock_irqsave(&res->lock, flags);
	if (WARN_ON(!res->active_users)) {
		spin_unlock_irqrestore(&res->lock, flags);
		return;
	}
	res->active_users--;
	sipa_nic_schedule_release_locked(res);
	spin_unlock_irqrestore(&res->lock, flags);
}

int sipa_nic_open(enum sipa_term_type src, int netid, sipa_notify_cb cb,
		  void *priv)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_skb_receiver *receiver;
	struct sipa_skb_sender *sender;
	struct sipa_nic *nic;
	unsigned long flags;
	int id, ret;

	if (!ctrl || !sipa_get_pdev())
		return -EPROBE_DEFER;
	if (src < 0 || src >= 32)
		return -EINVAL;
	for (id = 0; id < SIPA_NIC_MAX; id++)
		if ((s_spia_nic_statics[id].src_mask & BIT(src)) &&
		    netid == s_spia_nic_statics[id].netid)
			break;
	if (id == SIPA_NIC_MAX)
		return -EINVAL;
	sender = ctrl->sender[s_spia_nic_statics[id].pkt_type];
	receiver = ctrl->receiver[s_spia_nic_statics[id].pkt_type];
	if (!sender || !receiver ||
	    (s_spia_nic_statics[id].pkt_type == SIPA_PKT_IP &&
	     !ctrl->receiver[SIPA_PKT_ETH]))
		return -ENODEV;
	if (s_spia_nic_statics[id].cons == SIPA_RM_RES_CONS_USB) {
		/* This is the optional hardware USB NIC, not the WWAN path. */
		if (!IS_ENABLED(CONFIG_SPRD_SIPA_USB_ACCEL))
			return -EOPNOTSUPP;
		ret = sipa_rm_enable_usb_tether();
		if (ret)
			return ret;
	}
	mutex_lock(&sipa_nic_lifecycle_lock);
	nic = READ_ONCE(ctrl->nic[id]);
	if (!nic) {
		nic = kzalloc_obj(*nic, GFP_KERNEL);
		if (!nic) {
			ret = -ENOMEM;
			goto unlock;
		}
		atomic_set(&nic->status, NIC_CLOSE);
		skb_queue_head_init(&nic->rx_skb_q);
		INIT_LIST_HEAD(&nic->list);
		nic->nic_id = id;
		nic->send_ep = ctrl->eps[s_spia_nic_statics[id].send_ep];
		nic->src_mask = s_spia_nic_statics[id].src_mask;
		nic->netid = netid;
		sipa_nic_rm_init(&nic->rm_res, sender,
				 s_spia_nic_statics[id].cons);
		/* NIC storage is driver-lifetime, including deferred DMA retirement. */
		smp_store_release(&ctrl->nic[id], nic);
	}
	if (atomic_read(&nic->status) == NIC_OPEN || nic->rm_registered) {
		ret = -EBUSY;
		goto unlock;
	}
	spin_lock_irqsave(&nic->rm_res.lock, flags);
	ret = nic->rm_res.error;
	if (!ret && (nic->rm_res.closing || nic->rm_res.lease_held ||
		     nic->rm_res.active_users))
		ret = -EBUSY;
	spin_unlock_irqrestore(&nic->rm_res.lock, flags);
	if (ret)
		goto unlock;
	/* No live callback/private context remains from a completed close. */
	skb_queue_purge(&nic->rx_skb_q);
	ret = sipa_nic_register_rm(nic, id);
	if (ret)
		goto unlock;
	nic->rm_registered = true;
	WRITE_ONCE(nic->cb_priv, priv);
	rcu_assign_pointer(nic->cb, cb);
	WRITE_ONCE(nic->need_notify, 0);
	nic->flow_ctrl_status = false;
	sipa_receiver_add_nic(receiver, nic);
	if (s_spia_nic_statics[id].pkt_type == SIPA_PKT_IP)
		sipa_receiver_add_nic(ctrl->receiver[SIPA_PKT_ETH], nic);
	sipa_skb_sender_add_nic(sender, nic);
	/* Work, RM listener, callback and list membership are fully initialized. */
	atomic_set_release(&nic->status, NIC_OPEN);
	ret = id;
unlock:
	mutex_unlock(&sipa_nic_lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL(sipa_nic_open);

/* This void legacy API fences client callbacks and software TX/RX admission.
 * It does not assert that DMA has drained. The persistent NIC retains its RM
 * lease and remains non-reopenable until the completion worker proves that.
 * Must not be called from an atomic NIC callback/RCU read-side section.
 */
void sipa_nic_close(enum sipa_nic_id id)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_nic *nic;
	unsigned long flags;
	int ret;

	if (!ctrl || id < 0 || id >= SIPA_NIC_MAX)
		return;
	mutex_lock(&sipa_nic_lifecycle_lock);
	/* Pair with the initialized NIC publication in sipa_nic_open(). */
	nic = smp_load_acquire(&ctrl->nic[id]);
	if (!nic || atomic_read(&nic->status) != NIC_OPEN)
		goto unlock;
	atomic_set_release(&nic->status, NIC_CLOSE);
	rcu_assign_pointer(nic->cb, NULL);
	/* Deregistration takes the RM lock and fences its callback invocation. */
	ret = sipa_nic_deregister_rm(nic, id);
	if (!ret)
		nic->rm_registered = false;
	else
		pr_err("sipa NIC %d RM deregistration failed: %d\n", id, ret);
	cancel_work_sync(&nic->rm_res.grant_work);
	/* Includes TX publication and all callback/priv readers. */
	synchronize_rcu();
	WRITE_ONCE(nic->cb_priv, NULL);
	cancel_delayed_work_sync(&nic->rm_res.work);
	sipa_skb_sender_remove_nic(
		ctrl->sender[s_spia_nic_statics[id].pkt_type], nic);
	skb_queue_purge(&nic->rx_skb_q);
	WRITE_ONCE(nic->need_notify, 0);
	spin_lock_irqsave(&nic->rm_res.lock, flags);
	nic->rm_res.closing = nic->rm_res.lease_held;
	if (nic->rm_res.lease_held)
		mod_delayed_work(system_unbound_wq, &nic->rm_res.work, 0);
	spin_unlock_irqrestore(&nic->rm_res.lock, flags);
unlock:
	mutex_unlock(&sipa_nic_lifecycle_lock);
}
EXPORT_SYMBOL(sipa_nic_close);

void sipa_nic_notify_evt(struct sipa_nic *nic, enum sipa_evt_type evt)
{
	sipa_notify_cb cb;

	rcu_read_lock();
	if (atomic_read_acquire(&nic->status) == NIC_OPEN) {
		cb = rcu_dereference(nic->cb);
		if (cb)
			cb(READ_ONCE(nic->cb_priv), evt, 0);
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL(sipa_nic_notify_evt);

void sipa_nic_try_notify_recv(struct sipa_nic *nic)
{
	if (xchg(&nic->need_notify, 0))
		sipa_nic_notify_evt(nic, SIPA_RECEIVE);
}
EXPORT_SYMBOL(sipa_nic_try_notify_recv);

void sipa_nic_push_skb(struct sipa_nic *nic, struct sk_buff *skb)
{
	unsigned long flags;

	spin_lock_irqsave(&nic->rx_skb_q.lock, flags);
	if (atomic_read_acquire(&nic->status) != NIC_OPEN) {
		spin_unlock_irqrestore(&nic->rx_skb_q.lock, flags);
		dev_kfree_skb_any(skb);
		return;
	}
	__skb_queue_tail(&nic->rx_skb_q, skb);
	if (nic->rx_skb_q.qlen == 1)
		WRITE_ONCE(nic->need_notify, 1);
	spin_unlock_irqrestore(&nic->rx_skb_q.lock, flags);
}
EXPORT_SYMBOL(sipa_nic_push_skb);

int sipa_nic_tx(enum sipa_nic_id id, enum sipa_term_type dst, int netid,
		struct sk_buff *skb)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_skb_sender *sender;
	struct sipa_nic *nic;
	int ret = -ENODEV;

	if (!ctrl || id < 0 || id >= SIPA_NIC_MAX || !skb || dst < 0 ||
	    dst >= 32 || netid < -1 || netid > U8_MAX ||
	    (netid == -1 && s_spia_nic_statics[id].netid != -1))
		return -EINVAL;
	rcu_read_lock();
	/* Pair with the initialized NIC publication in sipa_nic_open(). */
	nic = smp_load_acquire(&ctrl->nic[id]);
	if (!nic || atomic_read_acquire(&nic->status) != NIC_OPEN)
		goto out;
	sender = ctrl->sender[s_spia_nic_statics[id].pkt_type];
	if (!sender)
		goto out;

	/* START closes this gate and waits for this RCU publication section. */
	if (dst == SIPA_TERM_USB && smp_load_acquire(&ctrl->usb_tx_blocked)) {
		ret = -ESHUTDOWN;
		goto out;
	}
	ret = sipa_nic_rm_res_request(nic);
	if (ret)
		goto out;
	/* Ethernet NICs use the existing -1/no-netid sentinel (wire byte 0xff). */
	ret = sipa_skb_sender_send_data(
		sender, skb, dst, netid == -1 ? U8_MAX : netid,
		dst == SIPA_TERM_USB ? &ctrl->usb_tx_pending : NULL);
	if (ret == -EAGAIN)
		WRITE_ONCE(nic->flow_ctrl_status, true);
	sipa_nic_rm_res_release(nic);
out:
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL(sipa_nic_tx);

int sipa_nic_rx(enum sipa_nic_id nic_id, struct sk_buff **out_skb)
{
	struct sk_buff *skb;
	struct sipa_nic *nic;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	if (!out_skb)
		return -EINVAL;
	*out_skb = NULL;
	if (!ctrl || nic_id < 0 || nic_id >= SIPA_NIC_MAX)
		return -EINVAL;
	if (!READ_ONCE(ctrl->nic[nic_id]) ||
	    atomic_read(&ctrl->nic[nic_id]->status) == NIC_CLOSE)
		return -ENODEV;

	nic = ctrl->nic[nic_id];
	skb = skb_dequeue(&nic->rx_skb_q);

	*out_skb = skb;

	return (skb) ? 0 : -ENODATA;
}
EXPORT_SYMBOL(sipa_nic_rx);

int sipa_nic_rx_has_data(enum sipa_nic_id nic_id)
{
	struct sipa_nic *nic;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	if (!ctrl || nic_id < 0 || nic_id >= SIPA_NIC_MAX)
		return -EINVAL;
	if (!READ_ONCE(ctrl->nic[nic_id]) ||
	    atomic_read(&ctrl->nic[nic_id]->status) == NIC_CLOSE)
		return 0;

	nic = ctrl->nic[nic_id];

	return (!!nic->rx_skb_q.qlen);
}
EXPORT_SYMBOL(sipa_nic_rx_has_data);

int sipa_nic_trigger_flow_ctrl_work(enum sipa_nic_id nic_id, int err)
{
	struct sipa_skb_sender *sender;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	if (!ctrl || nic_id < 0 || nic_id >= SIPA_NIC_MAX)
		return -EINVAL;
	sender = ctrl->sender[s_spia_nic_statics[nic_id].pkt_type];
	if (!sender)
		return -ENODEV;

	switch (err) {
	case -EAGAIN:
		sender->free_notify_net = true;
		schedule_work(&ctrl->flow_ctrl_work);
		break;
	default:
		dev_warn(ctrl->ctx->pdev,
			 "don't have this flow ctrl err type\n");
		break;
	}

	return 0;
}
EXPORT_SYMBOL(sipa_nic_trigger_flow_ctrl_work);
