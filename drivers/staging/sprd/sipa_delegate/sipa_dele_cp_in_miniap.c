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

#include <linux/sipa.h>
#include <linux/sipc.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "sipa_dele_priv.h"

static struct cp_delegator *s_cp_delegator;

static ssize_t cp_session_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sipa_delegator *delegator = dev_get_drvdata(dev);
	unsigned long flags;
	u32 generation;
	bool connected;
	int error;

	/* Diagnostic software connection state only; no DMA/MMIO access. */
	spin_lock_irqsave(&delegator->lock, flags);
	generation = delegator->generation;
	connected = delegator->connected;
	error = atomic_read(&delegator->control_error);
	spin_unlock_irqrestore(&delegator->lock, flags);
	return sysfs_emit(buf, "1 %u %u %u %u %u %d\n", delegator->dst,
			  delegator->chan, generation, connected, 0, error);
}
static DEVICE_ATTR_RO(cp_session);

static struct attribute *cp_session_attrs[] = {
	&dev_attr_cp_session.attr,
	NULL,
};
static const struct attribute_group cp_session_group = {
	.attrs = cp_session_attrs,
};

static void cp_dele_on_commad(void *priv, u16 flag, u32 data)
{
	struct sipa_delegator *delegator = priv;
	struct cp_delegator *cp =
		container_of(delegator, struct cp_delegator, delegator);
	int ret;

	pr_debug("prod_id:%d\n", delegator->prod_id);
	if (!READ_ONCE(delegator->connected) ||
	    atomic_read(&delegator->control_error))
		return;
	/* The connection thread serializes the session's enable ownership. */
	switch (flag) {
	case SMSG_FLG_DELE_ENABLE:
		ret = cp->enable_held ? 0 : sipa_set_enabled(true);
		if (!ret)
			cp->enable_held = true;
		sipa_dele_start_done_work(delegator, SMSG_FLG_DELE_ENABLE,
					  ret ? SMSG_VAL_DELE_REQ_FAIL :
						SMSG_VAL_DELE_REQ_SUCCESS);
		break;
	case SMSG_FLG_DELE_DISABLE:
		if (!cp->enable_held)
			break;
		ret = sipa_set_enabled(false);
		if (!ret)
			cp->enable_held = false;
		else
			/* Keep ownership so a repeated DISABLE can retry safely. */
			dev_err(delegator->pdev,
				"CP disable still pending: %d\n", ret);
		break;
	default:
		break;
	}
	/* do default operations */
	sipa_dele_on_commad(priv, flag, data);
}

static void cp_dele_on_done(void *priv, u16 flag, u32 value)
{
	struct sipa_delegator *delegator = priv;
	struct cp_delegator *cp =
		container_of(delegator, struct cp_delegator, delegator);
	unsigned long flags;
	bool install;
	int ret;

	/* Recovered stock AP callback: REQUEST DONE, REQUESTING state,
	 * value != FAIL, producer CP, and a not-yet-installed edge. The
	 * connection thread serializes DONE processing. Do not take the RM
	 * graph lock under delegator->lock (request callbacks take reverse).
	 */
	spin_lock_irqsave(&delegator->lock, flags);
	install = IS_ENABLED(CONFIG_SPRD_SIPA_USB_ACCEL) &&
		  delegator->connected && flag == SMSG_FLG_DELE_REQUEST &&
		  value != SMSG_VAL_DELE_REQ_FAIL &&
		  delegator->stat == SIPA_DELE_REQUESTING &&
		  delegator->prod_id == SIPA_RM_RES_PROD_CP &&
		  !cp->usb_dependency_added;
	spin_unlock_irqrestore(&delegator->lock, flags);
	if (install) {
		ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_USB,
					     SIPA_RM_RES_PROD_CP);
		/* EINPROGRESS means the edge was installed and is waiting for
		 * this very CP grant, not that graph insertion failed.
		 */
		if (ret && ret != -EEXIST && ret != -EINPROGRESS) {
			dev_err(delegator->pdev,
				"USB->CP dependency failed: %d\n", ret);
			/* No grant on a failed graph update; a later DONE may retry. */
			return;
		}
		cp->usb_dependency_added = true;
	}
	sipa_dele_on_done(priv, flag, value);
}

static int cp_dele_local_req_r_prod(void *user_data)
{
	/* do enable ipa  operation */

	return sipa_dele_local_req_r_prod(user_data);
}

int cp_delegator_init(struct sipa_delegator_create_params *params)
{
	int ret;

	s_cp_delegator =
		devm_kzalloc(params->pdev, sizeof(*s_cp_delegator), GFP_KERNEL);
	if (!s_cp_delegator)
		return -ENOMEM;
	ret = sipa_delegator_init(&s_cp_delegator->delegator, params);
	if (ret)
		return ret;

	s_cp_delegator->delegator.on_cmd = cp_dele_on_commad;
	s_cp_delegator->delegator.on_close = sipa_dele_on_close;
	s_cp_delegator->delegator.on_done = cp_dele_on_done;
	s_cp_delegator->delegator.local_request_prod = cp_dele_local_req_r_prod;
	/*
	 * Publish read-only state before starting the channel thread, so group
	 * allocation failure cannot strand a live worker during probe unwind.
	 */
	dev_set_drvdata(params->pdev, &s_cp_delegator->delegator);
	ret = devm_device_add_group(params->pdev, &cp_session_group);
	if (ret) {
		destroy_workqueue(s_cp_delegator->delegator.smsg_wq);
		dev_set_drvdata(params->pdev, NULL);
		s_cp_delegator = NULL;
		return ret;
	}
	ret = sipa_delegator_start(&s_cp_delegator->delegator);
	if (ret) {
		/*
		 * Probe unwind removes the managed group before freeing its
		 * devm context. Keep drvdata valid for a concurrent sysfs read.
		 */
		s_cp_delegator = NULL;
	} else if (IS_ENABLED(CONFIG_SPRD_SIPA_USB_ACCEL))
		sipa_usb_cp_resource_online(params->dst);
	return ret;
}
