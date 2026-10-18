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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/sipa.h>
#include "sipa_priv.h"
#include "sipa_hal.h"

static void sipa_usb_rm_notify_cb(void *user_data, enum sipa_rm_event event,
				  unsigned long data)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	pr_debug("%s: event %d\n", __func__, event);
	switch (event) {
	case SIPA_RM_EVT_GRANTED:
		complete(&ctrl->usb_rm_comp);
		break;
	case SIPA_RM_EVT_RELEASED:
		break;
	default:
		pr_err("%s: unknown event %d\n", __func__, event);
		break;
	}
}

int sipa_rm_usb_cons_init(void)
{
	struct sipa_rm_register_params r_param;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	init_completion(&ctrl->usb_rm_comp);

	r_param.user_data = ctrl;
	r_param.notify_cb = sipa_usb_rm_notify_cb;
	return sipa_rm_register(SIPA_RM_RES_CONS_USB, &r_param);
}
EXPORT_SYMBOL(sipa_rm_usb_cons_init);

void sipa_rm_usb_cons_deinit(void)
{
	struct sipa_rm_register_params r_param;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	r_param.user_data = ctrl;
	r_param.notify_cb = sipa_usb_rm_notify_cb;
	sipa_rm_deregister(SIPA_RM_RES_CONS_USB, &r_param);
}
EXPORT_SYMBOL(sipa_rm_usb_cons_deinit);

int sipa_rm_set_usb_eth_up(void)
{
	int ret;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	reinit_completion(&ctrl->usb_rm_comp);
	ret = sipa_rm_request_resource(SIPA_RM_RES_CONS_USB);
	if (ret == -EINPROGRESS) {
		long done = wait_for_completion_interruptible_timeout(
			&ctrl->usb_rm_comp, msecs_to_jiffies(5000));
		if (done <= 0) {
			sipa_rm_release_resource(SIPA_RM_RES_CONS_USB);
			return done ? (int)done : -ETIMEDOUT;
		}
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(sipa_rm_set_usb_eth_up);

void sipa_rm_set_usb_eth_down(void)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	reinit_completion(&ctrl->usb_rm_comp);

	sipa_rm_release_resource(SIPA_RM_RES_CONS_USB);

	/* Transport dependencies are shared device-lifetime graph policy. */
}
EXPORT_SYMBOL(sipa_rm_set_usb_eth_down);

/* The graph edges are device-lifetime policy, not owned by a USB function.
 * Do not delete another client's dependency when one USB request is released.
 */
int sipa_usb_remote_producer(void)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	if (!ctrl || !sipa_get_pdev())
		return -EPROBE_DEFER;
	/* Acquire the producer/listener setup published by the CP delegate. */
	if (ctrl->params_cfg.usb_remote_prod == SIPA_RM_RES_PROD_CP &&
	    !smp_load_acquire(&ctrl->params_cfg.usb_cp_resource_online))
		return -EPROBE_DEFER;
	return ctrl->params_cfg.usb_remote_prod;
}
EXPORT_SYMBOL_GPL(sipa_usb_remote_producer);

void sipa_usb_cp_resource_online(u8 dst)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	/* Called only after CP producer, listeners and connection thread exist.
	 * This does not mark its RM state granted or invent a firmware response.
	 */
	/* Publish the completed producer/listener setup to USB consumers. */
	if (ctrl && dst == 5 &&
	    ctrl->params_cfg.usb_remote_prod == SIPA_RM_RES_PROD_CP)
		smp_store_release(&ctrl->params_cfg.usb_cp_resource_online,
				  true);
}
EXPORT_SYMBOL_GPL(sipa_usb_cp_resource_online);

int sipa_rm_enable_usb_tether(void)
{
	int prod = sipa_usb_remote_producer();
	int ret;

	if (prod < 0)
		return prod;
	/* IPA0 is permanent from core init.  Local mailbox uses the real
	 * CP13 request/DONE protocol; only a declared PCIe peer needs PCIE3.
	 * Success here means the edge exists, never that a grant was observed.
	 */
	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_USB, prod);
	return ret == -EEXIST || ret == -EINPROGRESS ? 0 : ret;
}
EXPORT_SYMBOL(sipa_rm_enable_usb_tether);
