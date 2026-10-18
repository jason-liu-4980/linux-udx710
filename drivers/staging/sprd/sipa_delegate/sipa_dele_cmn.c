// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2019 Unisoc Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#define pr_fmt(fmt) "sipa_dele: %s " fmt, __func__

#include <linux/device.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/sipa.h>
#include "sipa_dele_priv.h"

static int conn_thread(void *data)
{
	struct smsg mrecv;
	struct sipa_delegator *delegator = data;
	int ret;

	/* since the channel open may hang, we call it in the thread context */
	ret = smsg_ch_open(delegator->dst, delegator->chan, -1);
	if (ret != 0) {
		pr_err("sipa_delegator failed to open dst %d channel %d\n",
		       delegator->dst, delegator->chan);
		/* assign NULL to thread poniter as failed to open channel */
		delegator->thread = NULL;
		return ret;
	}

	/* set connect status */
	delegator->on_open(delegator, 0, 0);

	/* start listen the smsg events */
	while (!kthread_should_stop()) {
		/* monitor seblock recv smsg */
		smsg_set(&mrecv, delegator->chan, 0, 0, 0);
		ret = smsg_recv(delegator->dst, &mrecv, -1);
		if (ret) {
			/* Never dispatch an uninitialized/stale message after an error. */
			if (kthread_should_stop())
				break;
			usleep_range(5000, 10000);
			continue;
		}

		pr_debug("dst=%d, chan=%d, type=%d, flag=0x%x, value=0x%08x\n",
			 delegator->dst, delegator->chan, mrecv.type,
			 mrecv.flag, mrecv.value);

		switch (mrecv.type) {
		case SMSG_TYPE_OPEN:
			if (mrecv.flag != SMSG_OPEN_MAGIC)
				break;
			/* OEM repeated OPEN preserves a pending REQUEST and ACKs it. */
			smsg_open_ack(delegator->dst, delegator->chan);
			delegator->on_open(delegator, mrecv.flag, mrecv.value);
			break;
		case SMSG_TYPE_CLOSE:
			if (mrecv.flag != SMSG_CLOSE_MAGIC)
				break;
			/* Fence TX before the ACK can wait for mailbox space. */
			delegator->on_close(delegator, mrecv.flag, mrecv.value);
			smsg_close_ack(delegator->dst, delegator->chan);
			break;
		case SMSG_TYPE_CMD:
			/* handle commads */
			delegator->on_cmd(delegator, mrecv.flag, mrecv.value);
			break;
		case SMSG_TYPE_DONE:
			/* handle cmd done */
			delegator->on_done(delegator, mrecv.flag, mrecv.value);
			break;
		case SMSG_TYPE_EVENT:
			/* handle events */
			delegator->on_evt(delegator, mrecv.flag, mrecv.value);
			break;
		default:
			ret = 1;
			break;
		};

		if (ret) {
			pr_info("unknown msg in conn_thrd: %d-%d, %d, %d, %d\n",
				delegator->dst, delegator->chan, mrecv.type,
				mrecv.flag, mrecv.value);
			ret = 0;
		}
	}

	pr_err("sipa dele thread %d-%d stop", delegator->dst, delegator->chan);

	return ret;
}

static void sipa_dele_fault_handler(struct work_struct *work)
{
	struct sipa_delegator *delegator =
		container_of(work, struct sipa_delegator, fault_work);

	dev_err(delegator->pdev, "SIPA control session fault: %d\n",
		atomic_read(&delegator->control_error));
	/* A failed queue reservation cannot silently lose a DONE/REQUEST.
	 * Use the same conservative fencing as CLOSE, without an on-wire ACK.
	 */
	delegator->on_close(delegator, 0, 0);
}

static int sipa_dele_control_fault(struct sipa_delegator *delegator, int error)
{
	/* May be called with delegator->lock held or from an RM callback. */
	if (!atomic_cmpxchg(&delegator->control_error, 0, error))
		schedule_work(&delegator->fault_work);
	return error;
}

static void sipa_dele_wq_handler(struct work_struct *work)
{
	struct sipa_dele_smsg_work_type *msg =
		container_of(work, struct sipa_dele_smsg_work_type, work);
	struct sipa_delegator *delegator = msg->delegator;
	unsigned long flags;
	int ret;

	for (;;) {
		/* One nonblocking publication is serialized with CLOSE. Never hold
		 * this spinlock across a sleepable smsg_send timeout. The ordered
		 * worker retains this immutable message while the mailbox is full.
		 */
		spin_lock_irqsave(&delegator->lock, flags);
		if (!delegator->connected ||
		    atomic_read(&delegator->control_error) ||
		    msg->generation != delegator->generation) {
			spin_unlock_irqrestore(&delegator->lock, flags);
			break;
		}
		ret = smsg_send(delegator->dst, &msg->msg, 0);
		spin_unlock_irqrestore(&delegator->lock, flags);
		if (!ret)
			break;
		/* Error is not successful delivery. Keep the accepted operation
		 * pending until it is sent or this connection epoch is invalidated.
		 * Bounded sleeps let CLOSE cancel a congested/broken session.
		 */
		if (ret != -ENOBUFS && ret != -EAGAIN && ret != -EBUSY) {
			sipa_dele_control_fault(delegator, ret);
			break;
		}
		msleep(20);
	}
	atomic_dec(&delegator->pending_messages);
	kfree(msg);
}

static int sipa_dele_queue_message(struct sipa_delegator *delegator, u8 type,
				   u16 flag, u32 value)
{
	struct sipa_dele_smsg_work_type *work;
	int error = atomic_read(&delegator->control_error);

	if (error)
		return error;
	if (atomic_inc_return(&delegator->pending_messages) > 64) {
		atomic_dec(&delegator->pending_messages);
		return sipa_dele_control_fault(delegator, -ENOSPC);
	}
	work = kzalloc_obj(*work, GFP_ATOMIC);
	if (!work) {
		atomic_dec(&delegator->pending_messages);
		return sipa_dele_control_fault(delegator, -ENOMEM);
	}
	INIT_WORK(&work->work, sipa_dele_wq_handler);
	work->delegator = delegator;
	work->generation = READ_ONCE(delegator->generation);
	smsg_set(&work->msg, delegator->chan, type, flag, value);
	queue_work(delegator->smsg_wq, &work->work);
	return 0;
}

static int sipa_dele_start_req_work(struct sipa_delegator *delegator)
{
	return sipa_dele_queue_message(delegator, SMSG_TYPE_CMD,
				       SMSG_FLG_DELE_REQUEST, 0);
}

static int sipa_dele_start_rls_work(struct sipa_delegator *delegator)
{
	return sipa_dele_queue_message(delegator, SMSG_TYPE_CMD,
				       SMSG_FLG_DELE_RELEASE, 0);
}

void sipa_dele_start_done_work(struct sipa_delegator *delegator, u16 flag,
			       u32 val)
{
	int ret = sipa_dele_queue_message(delegator, SMSG_TYPE_DONE, flag, val);

	if (ret)
		dev_err_ratelimited(delegator->pdev,
				    "SIPA DONE queue failed: %d\n", ret);
}

static void sipa_dele_r_user_req_cons(struct sipa_delegator *delegator)
{
	int ret;

	atomic_set(&delegator->requesting_cons, 1);
	ret = sipa_rm_request_resource(delegator->cons_user);
	switch (ret) {
	case 0:
		delegator->cons_ref_cnt++;
		if (atomic_cmpxchg(&delegator->requesting_cons, 1, 0))
			sipa_dele_start_done_work(delegator,
						  SMSG_FLG_DELE_REQUEST,
						  SMSG_VAL_DELE_REQ_SUCCESS);
		break;
	case -EINPROGRESS:
		delegator->cons_ref_cnt++;
		break;
	default:
		atomic_set(&delegator->requesting_cons, 0);
		/* The built-in, registered consumer is stable for our lifetime.
		 * RM counts every request reaching consumer_request, including a
		 * hard producer failure. Balance that reference before reporting
		 * FAIL; release consumes it even when returning EINPROGRESS.
		 */
		ret = sipa_rm_release_resource(delegator->cons_user);
		if (ret && ret != -EINPROGRESS)
			dev_err(delegator->pdev,
				"failed consumer request release: %d\n", ret);
		sipa_dele_start_done_work(delegator, SMSG_FLG_DELE_REQUEST,
					  SMSG_VAL_DELE_REQ_FAIL);
		break;
	}
}

static void sipa_dele_r_user_rls_cons(struct sipa_delegator *delegator)
{
	if (!delegator->cons_ref_cnt)
		return;

	delegator->cons_ref_cnt--;
	sipa_rm_release_resource(delegator->cons_user);
}

void sipa_dele_on_open(void *priv, u16 flag, u32 data)
{
	struct sipa_delegator *delegator = priv;
	unsigned long flags;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	/* Retire an old connection's deferred error before accepting the OPEN. */
	cancel_work_sync(&delegator->fault_work);
	spin_lock_irqsave(&delegator->lock, flags);
	atomic_set(&delegator->control_error, 0);
	WRITE_ONCE(delegator->connected, true);
	if (delegator->stat == SIPA_DELE_REQUESTING)
		sipa_dele_start_req_work(delegator);
	spin_unlock_irqrestore(&delegator->lock, flags);
}

void sipa_dele_on_close(void *priv, u16 flag, u32 data)
{
	struct sipa_delegator *delegator = priv;
	unsigned long flags;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	spin_lock_irqsave(&delegator->lock, flags);
	WRITE_ONCE(delegator->connected, false);
	delegator->generation++;
	atomic_set(&delegator->requesting_cons, 0);
	/*
	 * OEM CLOSE releases the software connection; the next OPEN can
	 * acknowledge and request the peer resource again.
	 */
	delegator->stat = SIPA_DELE_RELEASED;
	spin_unlock_irqrestore(&delegator->lock, flags);
}

void sipa_dele_on_commad(void *priv, u16 flag, u32 data)
{
	struct sipa_delegator *delegator = priv;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	if (!READ_ONCE(delegator->connected) ||
	    atomic_read(&delegator->control_error))
		return;
	switch (flag) {
	case SMSG_FLG_DELE_REQUEST:
		sipa_dele_r_user_req_cons(delegator);
		break;
	case SMSG_FLG_DELE_RELEASE:
		sipa_dele_r_user_rls_cons(delegator);
		break;
	default:
		break;
	}
}

void sipa_dele_on_done(void *priv, u16 flag, u32 val)
{
	struct sipa_delegator *delegator = priv;
	enum sipa_dele_state last_stat;
	unsigned long flags;
	int ret;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	if (flag != SMSG_FLG_DELE_REQUEST)
		return;
	spin_lock_irqsave(&delegator->lock, flags);
	last_stat = delegator->stat;
	if (!delegator->connected || atomic_read(&delegator->control_error))
		goto unlock;
	switch (delegator->stat) {
	case SIPA_DELE_ACTIVE:
		break;
	case SIPA_DELE_REQUESTING:
		if (val == SMSG_VAL_DELE_REQ_FAIL) {
			sipa_dele_start_req_work(delegator);
		} else {
			ret = sipa_rm_notify_completion(SIPA_RM_EVT_GRANTED,
							delegator->prod_id);
			if (ret)
				sipa_dele_control_fault(delegator, ret);
			else
				delegator->stat = SIPA_DELE_ACTIVE;
		}
		break;
	case SIPA_DELE_RELEASING:
		sipa_dele_start_rls_work(delegator);
		delegator->stat = SIPA_DELE_RELEASED;
		break;
	case SIPA_DELE_RELEASED:
		break;
	default:
		break;
	}
unlock:
	spin_unlock_irqrestore(&delegator->lock, flags);

	pr_debug("smsg req done stat:%d -> %d\n", last_stat, delegator->stat);
}

void sipa_dele_on_event(void *priv, u16 flag, u32 data)
{
	pr_debug("flag:%d data:%d\n", flag, data);
}

int sipa_dele_local_rls_r_prod(void *user_data)
{
	unsigned long flags;
	struct sipa_delegator *delegator = user_data;
	int ret;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	spin_lock_irqsave(&delegator->lock, flags);
	switch (delegator->stat) {
	case SIPA_DELE_ACTIVE:
		ret = sipa_dele_start_rls_work(delegator);
		if (!ret)
			delegator->stat = SIPA_DELE_RELEASED;
		break;
	case SIPA_DELE_REQUESTING:
		delegator->stat = SIPA_DELE_RELEASING;
		ret = -EINPROGRESS;
		break;
	case SIPA_DELE_RELEASING:
		ret = -EINPROGRESS;
		break;
	case SIPA_DELE_RELEASED:
		ret = 0;
		break;
	default:
		ret = -EPERM;
		break;
	}
	spin_unlock_irqrestore(&delegator->lock, flags);

	return ret;
}

int sipa_dele_local_req_r_prod(void *user_data)
{
	unsigned long flags;
	struct sipa_delegator *delegator = user_data;
	int ret;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	spin_lock_irqsave(&delegator->lock, flags);
	if (atomic_read(&delegator->control_error)) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	switch (delegator->stat) {
	case SIPA_DELE_ACTIVE:
		ret = 0;
		break;
	case SIPA_DELE_REQUESTING:
		ret = -EINPROGRESS;
		break;
	case SIPA_DELE_RELEASING:
		delegator->stat = SIPA_DELE_REQUESTING;
		ret = -EINPROGRESS;
		break;
	case SIPA_DELE_RELEASED:
		ret = 0;
		if (READ_ONCE(delegator->connected))
			ret = sipa_dele_start_req_work(delegator);
		if (!ret) {
			delegator->stat = SIPA_DELE_REQUESTING;
			ret = -EINPROGRESS;
		}
		break;
	default:
		ret = -EPERM;
		break;
	}
unlock:
	spin_unlock_irqrestore(&delegator->lock, flags);

	return ret;
}

static void sipa_dele_cons_notify_cb(void *user_data, enum sipa_rm_event event,
				     unsigned long data)
{
	struct sipa_delegator *delegator = user_data;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	if (event != SIPA_RM_EVT_GRANTED)
		return;
	if (atomic_cmpxchg(&delegator->requesting_cons, 1, 0))
		sipa_dele_start_done_work(delegator, SMSG_FLG_DELE_REQUEST,
					  SMSG_VAL_DELE_REQ_SUCCESS);
}

int sipa_delegator_init(struct sipa_delegator *delegator,
			struct sipa_delegator_create_params *params)
{
	/* value init */
	delegator->pdev = params->pdev;
	delegator->cfg = params->cfg;
	delegator->prod_id = params->prod_id;
	delegator->cons_prod = params->cons_prod;
	delegator->cons_user = params->cons_user;
	delegator->stat = SIPA_DELE_RELEASED;
	delegator->cons_ref_cnt = 0;
	delegator->generation = 0;
	delegator->dst = params->dst;
	delegator->chan = params->chan;
	WRITE_ONCE(delegator->connected, false);
	atomic_set(&delegator->requesting_cons, 0);
	atomic_set(&delegator->pending_messages, 0);
	atomic_set(&delegator->control_error, 0);
	INIT_WORK(&delegator->fault_work, sipa_dele_fault_handler);
	delegator->on_open = sipa_dele_on_open;
	delegator->on_close = sipa_dele_on_close;
	delegator->on_cmd = sipa_dele_on_commad;
	delegator->on_done = sipa_dele_on_done;
	delegator->on_evt = sipa_dele_on_event;
	delegator->local_request_prod = sipa_dele_local_req_r_prod;
	delegator->local_release_prod = sipa_dele_local_rls_r_prod;
	spin_lock_init(&delegator->lock);

	delegator->smsg_wq = create_singlethread_workqueue("dele_smsg_wq");
	if (!delegator->smsg_wq) {
		pr_err("create workqueue failed\n");
		return -ENOMEM;
	}

	return 0;
}

int sipa_delegator_start(struct sipa_delegator *delegator)
{
	struct sipa_rm_create_params rm_params = {};
	struct sipa_rm_register_params reg_params;
	int ret;

	pr_debug("prod_id:%d\n", delegator->prod_id);

	/* sipa resource manage operations */
	rm_params.name = delegator->prod_id;
	rm_params.floor_voltage = 0;
	rm_params.reg_params.notify_cb = NULL;
	rm_params.reg_params.user_data = delegator;
	rm_params.request_resource = delegator->local_request_prod;
	rm_params.release_resource = delegator->local_release_prod;

	ret = sipa_rm_create_resource(&rm_params);
	if (ret)
		goto destroy_wq;

	ret = sipa_rm_add_dependency(delegator->cons_prod, delegator->prod_id);
	/* The edge is installed while an active consumer awaits this producer.
	 * Keep it and start the channel so OPEN can complete the pending grant.
	 */
	if (ret && ret != -EINPROGRESS)
		goto del_res;

	reg_params.notify_cb = sipa_dele_cons_notify_cb;
	reg_params.user_data = delegator;
	ret = sipa_rm_register(delegator->cons_user, &reg_params);
	if (ret)
		goto del_dep;

	/* No CP event can arrive before the real resource graph exists. */
	delegator->thread = kthread_run(conn_thread, delegator, "dele-%d-%d",
					delegator->prod_id, delegator->dst);
	if (IS_ERR(delegator->thread)) {
		ret = PTR_ERR(delegator->thread);
		delegator->thread = NULL;
		sipa_rm_deregister(delegator->cons_user, &reg_params);
		goto del_dep;
	}
	return 0;
del_dep:
	sipa_rm_delete_dependency(delegator->cons_prod, delegator->prod_id);
del_res:
	sipa_rm_delete_resource(delegator->prod_id);
destroy_wq:
	destroy_workqueue(delegator->smsg_wq);
	cancel_work_sync(&delegator->fault_work);
	delegator->smsg_wq = NULL;
	return ret;
}
