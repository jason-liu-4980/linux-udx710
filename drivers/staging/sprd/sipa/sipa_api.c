// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
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
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/pm_wakeup.h>
#include <linux/rcupdate.h>
#include <linux/regulator/consumer.h>

#include "sipa_hal.h"
#include "sipa_api.h"
#include "sipa_priv.h"
#include "sipa_rm.h"
#include "sipa_debug.h"

#define DRV_NAME "sipa"
#define DRV_LOCAL_NAME "local_ipa"
#define DRV_REMOTE_NAME "remote_ipa"

static const int s_ep_src_term_map[SIPA_EP_MAX] = { SIPA_TERM_USB,
						    SIPA_TERM_AP_IP,
						    SIPA_TERM_AP_ETH,
						    SIPA_TERM_VCP,
						    SIPA_TERM_PCIE0,
						    SIPA_TERM_PCIE_LOCAL_CTRL0,
						    SIPA_TERM_PCIE_LOCAL_CTRL1,
						    SIPA_TERM_PCIE_LOCAL_CTRL2,
						    SIPA_TERM_PCIE_LOCAL_CTRL3,
						    SIPA_TERM_PCIE_REMOTE_CTRL0,
						    SIPA_TERM_PCIE_REMOTE_CTRL1,
						    SIPA_TERM_PCIE_REMOTE_CTRL2,
						    SIPA_TERM_PCIE_REMOTE_CTRL3,
						    SIPA_TERM_SDIO0,
						    SIPA_TERM_WIFI };

struct sipa_common_fifo_info sipa_common_fifo_statics[SIPA_FIFO_MAX] = {
	{
		.tx_fifo = "sprd,usb-ul-tx",
		.rx_fifo = "sprd,usb-ul-rx",
		.relate_ep = SIPA_EP_USB,
		.src_id = SIPA_TERM_USB,
		.dst_id = SIPA_TERM_AP_ETH,
		.is_to_ipa = 1,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,sdio-ul-tx",
		.rx_fifo = "sprd,sdio-ul-rx",
		.relate_ep = SIPA_EP_SDIO,
		.src_id = SIPA_TERM_SDIO0,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,ap-ip-ul-tx",
		.rx_fifo = "sprd,ap-ip-ul-rx",
		.relate_ep = SIPA_EP_AP_IP,
		.src_id = SIPA_TERM_AP_IP,
		.dst_id = SIPA_TERM_VAP0,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,pcie-ul-tx",
		.rx_fifo = "sprd,pcie-ul-rx",
		.relate_ep = SIPA_EP_PCIE,
		.src_id = SIPA_TERM_PCIE0,
		.dst_id = SIPA_TERM_VCP,
		.is_to_ipa = 1,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,remote-pcie0-ul-tx",
		.rx_fifo = "sprd,remote-pcie0-ul-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL0,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL0,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,remote-pcie1-ul-tx",
		.rx_fifo = "sprd,remote-pcie1-ul-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL1,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL1,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,remote-pcie2-ul-tx",
		.rx_fifo = "sprd,remote-pcie2-ul-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL2,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL2,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,remote-pcie3-ul-tx",
		.rx_fifo = "sprd,remote-pcie3-ul-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL3,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL3,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,ap-eth-dl-tx",
		.rx_fifo = "sprd,ap-eth-dl-rx",
		.relate_ep = SIPA_EP_AP_ETH,
		.src_id = SIPA_TERM_AP_ETH,
		.dst_id = SIPA_TERM_USB,
		.is_to_ipa = 1,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie0-dl-tx",
		.rx_fifo = "sprd,local-pcie0-dl-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL0,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL0,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie1-dl-tx",
		.rx_fifo = "sprd,local-pcie1-dl-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL1,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL1,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie2-dl-tx",
		.rx_fifo = "sprd,local-pcie2-dl-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL2,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL2,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie3-dl-tx",
		.rx_fifo = "sprd,local-pcie3-dl-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL3,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL3,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,wifi-ul-tx",
		.rx_fifo = "sprd,wifi-ul-rx",
		.relate_ep = SIPA_EP_WIFI,
		.src_id = SIPA_TERM_WIFI,
		.dst_id = SIPA_TERM_AP_ETH,
		.is_to_ipa = 1,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,cp-dl-tx",
		.rx_fifo = "sprd,cp-dl-rx",
		.relate_ep = SIPA_EP_VCP,
		.src_id = SIPA_TERM_VAP0,
		.dst_id = SIPA_TERM_PCIE0,
		.is_to_ipa = 1,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,usb-dl-tx",
		.rx_fifo = "sprd,usb-dl-rx",
		.relate_ep = SIPA_EP_USB,
		.src_id = SIPA_TERM_USB,
		.dst_id = SIPA_TERM_AP_ETH,
		.is_to_ipa = 0,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,sdio-dl-tx",
		.rx_fifo = "sprd,sdio-dl-rx",
		.relate_ep = SIPA_EP_SDIO,
		.src_id = SIPA_TERM_SDIO0,
		.dst_id = SIPA_TERM_AP_ETH,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,ap-ip-dl-tx",
		.rx_fifo = "sprd,ap-ip-dl-rx",
		.relate_ep = SIPA_EP_AP_IP,
		.src_id = SIPA_TERM_AP_IP,
		.dst_id = SIPA_TERM_VAP0,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,pcie-dl-tx",
		.rx_fifo = "sprd,pcie-dl-rx",
		.relate_ep = SIPA_EP_PCIE,
		.src_id = SIPA_TERM_PCIE0,
		.dst_id = SIPA_TERM_VCP,
		.is_to_ipa = 0,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,remote-pcie0-dl-tx",
		.rx_fifo = "sprd,remote-pcie0-dl-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL0,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL0,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,remote-pcie1-dl-tx",
		.rx_fifo = "sprd,remote-pcie1-dl-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL1,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL1,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,remote-pcie2-dl-tx",
		.rx_fifo = "sprd,remote-pcie2-dl-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL2,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL2,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,remote-pcie3-dl-tx",
		.rx_fifo = "sprd,remote-pcie3-dl-rx",
		.relate_ep = SIPA_EP_REMOTE_PCIE_CTRL3,
		.src_id = SIPA_TERM_PCIE_REMOTE_CTRL3,
		.dst_id = SIPA_TERM_AP_IP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,ap-eth-ul-tx",
		.rx_fifo = "sprd,ap-eth-ul-rx",
		.relate_ep = SIPA_EP_AP_ETH,
		.src_id = SIPA_TERM_AP_ETH,
		.dst_id = SIPA_TERM_PCIE0,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie0-ul-tx",
		.rx_fifo = "sprd,local-pcie0-ul-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL0,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL0,
		.dst_id = SIPA_TERM_VCP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie1-ul-tx",
		.rx_fifo = "sprd,local-pcie1-ul-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL1,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL1,
		.dst_id = SIPA_TERM_VCP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie2-ul-tx",
		.rx_fifo = "sprd,local-pcie2-ul-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL2,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL2,
		.dst_id = SIPA_TERM_VCP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,local-pcie3-ul-tx",
		.rx_fifo = "sprd,local-pcie3-ul-rx",
		.relate_ep = SIPA_EP_PCIE_CTRL3,
		.src_id = SIPA_TERM_PCIE_LOCAL_CTRL3,
		.dst_id = SIPA_TERM_VCP,
		.is_to_ipa = 0,
		.is_pam = 0,
	},
	{
		.tx_fifo = "sprd,wifi-dl-tx",
		.rx_fifo = "sprd,wifi-dl-rx",
		.relate_ep = SIPA_EP_WIFI,
		.src_id = SIPA_TERM_WIFI,
		.dst_id = SIPA_TERM_AP_ETH,
		.is_to_ipa = 0,
		.is_pam = 1,
	},
	{
		.tx_fifo = "sprd,cp-ul-tx",
		.rx_fifo = "sprd,cp-ul-rx",
		.relate_ep = SIPA_EP_VCP,
		.src_id = SIPA_TERM_VAP0,
		.dst_id = SIPA_TERM_PCIE0,
		.is_to_ipa = 0,
		.is_pam = 1,
	},
};

static const struct file_operations sipa_local_drv_fops = {
	.owner = THIS_MODULE,
	.open = NULL,
	.read = NULL,
	.write = NULL,
	.unlocked_ioctl = NULL,
#ifdef CONFIG_COMPAT
	.compat_ioctl = NULL,
#endif
};

struct sipa_control *s_sipa_ctrl;
static DEFINE_MUTEX(sipa_pam_lock);

static int sipa_resume_for_pamu3(struct sipa_control *ctrl)
{
	int i, ret = 0;
	sipa_hal_hdl hdl = ctrl->ctx->hdl;
	struct sipa_plat_drv_cfg *cfg = &ctrl->params_cfg;
	struct sipa_common_fifo_cfg *fifo = cfg->common_fifo_cfg;
	struct sipa_skb_receiver **receiver = ctrl->receiver;

	mutex_lock(&ctrl->resume_lock);
	if (!ctrl->suspend_stage) {
		mutex_unlock(&ctrl->resume_lock);
		return 0;
	}

	if (sipa_hal_get_pause_status() || sipa_hal_get_resume_status())
		goto early_resume;

	if (ctrl->suspend_stage & SIPA_BACKUP_SUSPEND) {
		for (i = 0; i < SIPA_FIFO_MAX; i++)
			if (fifo[i].tx_fifo.in_iram && fifo[i].rx_fifo.in_iram)
				sipa_hal_resume_fifo_node(hdl, i);

		ret = sipa_resume_glb_reg_cfg(cfg);
		if (ret)
			goto out;
		if (cfg->tft_mode) {
			ret = sipa_tft_mode_init(hdl);
			if (ret)
				goto out;
		}
		ret = sipa_resume_common_fifo(hdl, &ctrl->params_cfg);
		if (ret)
			goto out;
		ctrl->suspend_stage &= ~SIPA_BACKUP_SUSPEND;
	}

early_resume:
#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
	/* The legacy software-status shortcut must not skip punt restoration. */
	ret = sipa_hal_ft_punt_restore();
	if (ret)
		goto out;
#endif
	if (!cfg->is_bypass && (ctrl->suspend_stage & SIPA_THREAD_SUSPEND)) {
		ret = sipa_receiver_prepare_resume(receiver[SIPA_PKT_IP]);
		if (ret)
			goto out;
		ret = sipa_receiver_prepare_resume(receiver[SIPA_PKT_ETH]);
		if (ret)
			goto out;
		ctrl->suspend_stage &= ~SIPA_THREAD_SUSPEND;
	}

	if (ctrl->suspend_stage & SIPA_ACTION_SUSPEND) {
		if (sipa_hal_ctrl_action(true) != 1) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ctrl->suspend_stage &= ~SIPA_ACTION_SUSPEND;
	}

out:
	mutex_unlock(&ctrl->resume_lock);
	return ret;
}

/**
 * sipa_prepare_resume() - Restore IPA related configuration
 * @ctrl: s_sipa_ctrl
 *
 * IPA EB bit enable, restore IPA glb reg and common fifo, resume
 * ep and receiver recv function.
 *
 */
static int sipa_prepare_resume(struct sipa_control *ctrl)
{
	int i, ret = 0;
	struct sipa_endpoint *ep;
	sipa_hal_hdl hdl = ctrl->ctx->hdl;
	struct sipa_plat_drv_cfg *cfg = &ctrl->params_cfg;
	struct sipa_common_fifo_cfg *fifo = cfg->common_fifo_cfg;
	struct sipa_skb_receiver **receiver = ctrl->receiver;

	mutex_lock(&ctrl->resume_lock);
	if (!ctrl->suspend_stage) {
		mutex_unlock(&ctrl->resume_lock);
		return 0;
	}

	if (ctrl->suspend_stage & SIPA_EB_SUSPEND) {
		ret = sipa_set_enabled(true);
		if (ret)
			goto out;
		ctrl->suspend_stage &= ~SIPA_EB_SUSPEND;
	}

	if (sipa_hal_get_pause_status() || sipa_hal_get_resume_status())
		goto early_resume;

	if (ctrl->suspend_stage & SIPA_BACKUP_SUSPEND) {
		for (i = 0; i < SIPA_FIFO_MAX; i++)
			if (fifo[i].tx_fifo.in_iram && fifo[i].rx_fifo.in_iram)
				sipa_hal_resume_fifo_node(hdl, i);

		ret = sipa_resume_glb_reg_cfg(cfg);
		if (ret)
			goto out;
		if (cfg->tft_mode) {
			ret = sipa_tft_mode_init(hdl);
			if (ret)
				goto out;
		}
		ret = sipa_resume_common_fifo(hdl, &ctrl->params_cfg);
		if (ret)
			goto out;
		ctrl->suspend_stage &= ~SIPA_BACKUP_SUSPEND;
	}

early_resume:
#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
	/* The legacy software-status shortcut must not skip punt restoration. */
	ret = sipa_hal_ft_punt_restore();
	if (ret)
		goto out;
#endif
	ep = ctrl->eps[SIPA_EP_USB];
	if (ep && READ_ONCE(ep->connected) && !READ_ONCE(ep->pam_stopping)) {
		ret = sipa_hal_cmn_fifo_set_receive(hdl, ep->recv_fifo.idx,
						    false);
		if (ret)
			goto out;
	}
	ep = ctrl->eps[SIPA_EP_VCP];
	if (ep && READ_ONCE(ep->connected) && !READ_ONCE(ep->pam_stopping)) {
		ret = sipa_hal_cmn_fifo_set_receive(hdl, ep->recv_fifo.idx,
						    false);
		if (ret)
			goto out;
	}
	ep = ctrl->eps[SIPA_EP_WIFI];
	if (ep && READ_ONCE(ep->connected) && !READ_ONCE(ep->pam_stopping)) {
		ret = sipa_hal_cmn_fifo_set_receive(hdl, ep->recv_fifo.idx,
						    false);
		if (ret)
			goto out;
	}

	ctrl->suspend_stage &= ~SIPA_EP_SUSPEND;

	if (!cfg->is_bypass && (ctrl->suspend_stage & SIPA_THREAD_SUSPEND)) {
		ret = sipa_receiver_prepare_resume(receiver[SIPA_PKT_IP]);
		if (ret)
			goto out;
		ret = sipa_receiver_prepare_resume(receiver[SIPA_PKT_ETH]);
		if (ret)
			goto out;
		ctrl->suspend_stage &= ~SIPA_THREAD_SUSPEND;
	}

	if (ctrl->suspend_stage & SIPA_ACTION_SUSPEND) {
		if (sipa_hal_ctrl_action(true) != 1) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ctrl->suspend_stage &= ~SIPA_ACTION_SUSPEND;
	}

out:
	mutex_unlock(&ctrl->resume_lock);
	return ret;
}

/**
 * sipa_resume_work() - resume ipa all profile
 * @work: &s_sipa_ctrl->resume_work
 *
 * resume ipa all profile, after this function finished,
 * ipa will work normally.
 *
 */
static void sipa_resume_work(struct work_struct *work)
{
	struct sipa_control *ctrl = container_of(
		(struct delayed_work *)work, struct sipa_control, resume_work);

	int ret;

	if (!ctrl)
		return;

	if (ctrl->suspend_stage & SIPA_FORCE_SUSPEND) {
		ctrl->suspend_stage &= ~SIPA_FORCE_SUSPEND;
		sipa_force_wakeup(&ctrl->params_cfg, true);
	}

	ret = sipa_prepare_resume(ctrl);
	if (ret) {
		dev_err(ctrl->ctx->pdev, "SIPA resume failed: %d\n", ret);
		return;
	}

	if (!ctrl->params_cfg.is_bypass) {
		sipa_sender_prepare_resume(ctrl->sender[SIPA_PKT_ETH]);
		sipa_sender_prepare_resume(ctrl->sender[SIPA_PKT_IP]);
	}

	sipa_rm_notify_completion(SIPA_RM_EVT_GRANTED, SIPA_RM_RES_PROD_IPA);

	ctrl->suspend_stage = 0;
}

/**
 * sipa_check_ep_suspend() - Check ep whether have the conditions for sleep.
 * @dev: Sipa driver device.
 * @id: The endpoint id that need to be checked.
 *
 * Determine if the node description sent out is completely free,
 * if not free completely, wake lock 500ms, return -EAGAIN.
 *
 * Return:
 *	0: succeed.
 *	-EAGAIN: check err.
 */
static int sipa_check_ep_suspend(struct device *dev, enum sipa_ep_id id)
{
	bool s;
	struct sipa_control *ctrl = dev_get_drvdata(dev);
	struct sipa_endpoint *ep = ctrl->eps[id];

	if (!ep)
		return 0;

	if (id == SIPA_EP_VCP && !ep->connected)
		return 0;

	sipa_hal_cmn_fifo_set_receive(ctrl->ctx->hdl, ep->recv_fifo.idx, true);
	s = sipa_hal_check_send_cmn_fifo_com(ctrl->ctx->hdl, ep->send_fifo.idx);
	if (!s) {
		dev_err(dev,
			"check send cmn fifo finish status fail fifo id = %d\n",
			ep->send_fifo.idx);
		pm_wakeup_dev_event(dev, 500, true);
		return -EAGAIN;
	}

	return 0;
}

/**
 * sipa_ep_prepare_suspend() - Check usb/wifi/vcp ep suspend conditions
 * @dev: sipa driver device
 *
 * Check usb/wifi/vcp end pointer suspend conditions, if conditions
 * satisfaction, turn off its receiving function.
 *
 * Return:
 *	0: success.
 *	-EAGAIN: suspend fail.
 */
static int sipa_ep_prepare_suspend(struct device *dev)
{
	struct sipa_control *ctrl = dev_get_drvdata(dev);

	if (ctrl->suspend_stage & SIPA_EP_SUSPEND)
		return 0;

	if (sipa_check_ep_suspend(dev, SIPA_EP_USB) ||
	    sipa_check_ep_suspend(dev, SIPA_EP_WIFI) ||
	    sipa_check_ep_suspend(dev, SIPA_EP_VCP))
		return -EAGAIN;

	ctrl->suspend_stage |= SIPA_EP_SUSPEND;

	return 0;
}

/**
 * sipa_thread_prepare_suspend() - Check sender/receiver suspend conditions.
 * @dev: sipa driver device.
 *
 * Check sender/receiver suspend conditions. if conditions not satisfaction,
 * wake lock 500ms.
 *
 * Return:
 *	0: success.
 *	-EAGAIN: check fail.
 */
static int sipa_thread_prepare_suspend(struct device *dev)
{
	struct sipa_control *ctrl = dev_get_drvdata(dev);
	struct sipa_skb_sender **sender = ctrl->sender;
	struct sipa_skb_receiver **receiver = ctrl->receiver;

	if (ctrl->suspend_stage & SIPA_THREAD_SUSPEND)
		return 0;

	if (ctrl->params_cfg.is_bypass)
		return 0;

	if (!sipa_sender_prepare_suspend(sender[SIPA_PKT_IP]) &&
	    !sipa_sender_prepare_suspend(sender[SIPA_PKT_ETH]) &&
	    !sipa_receiver_prepare_suspend(receiver[SIPA_PKT_IP]) &&
	    !sipa_receiver_prepare_suspend(receiver[SIPA_PKT_ETH])) {
		ctrl->suspend_stage |= SIPA_THREAD_SUSPEND;
	} else {
		dev_err(dev, "thread prepare suspend err\n");
		pm_wakeup_dev_event(dev, 500, true);
		return -EAGAIN;
	}

	return 0;
}

static int sipa_fifo_prepare_suspend(struct device *dev)
{
	int i;
	struct sipa_control *ctrl = dev_get_drvdata(dev);
	struct sipa_common_fifo_cfg *fifo = ctrl->params_cfg.common_fifo_cfg;

	if (ctrl->suspend_stage & SIPA_BACKUP_SUSPEND)
		return 0;

	for (i = 0; i < SIPA_FIFO_MAX; i++)
		if (fifo[i].tx_fifo.in_iram && fifo[i].rx_fifo.in_iram)
			sipa_hal_bk_fifo_node(ctrl->ctx->hdl, i);

	ctrl->suspend_stage |= SIPA_BACKUP_SUSPEND;

	return 0;
}

static int sipa_prepare_suspend(struct device *dev)
{
	struct sipa_control *ctrl = dev_get_drvdata(dev);

	if (ctrl->power_flag)
		return 0;

	if (sipa_ep_prepare_suspend(dev))
		return -EAGAIN;

	if (sipa_thread_prepare_suspend(dev))
		return -EAGAIN;

	if (sipa_fifo_prepare_suspend(dev))
		return -EAGAIN;

	if (!(ctrl->suspend_stage & SIPA_ACTION_SUSPEND)) {
		sipa_hal_ctrl_action(false);
		ctrl->suspend_stage |= SIPA_ACTION_SUSPEND;
	}

	if (!(ctrl->suspend_stage & SIPA_EB_SUSPEND) &&
	    !sipa_set_enabled(false))
		ctrl->suspend_stage |= SIPA_EB_SUSPEND;

	if (!(ctrl->suspend_stage & SIPA_FORCE_SUSPEND) &&
	    !sipa_force_wakeup(&ctrl->params_cfg, false))
		ctrl->suspend_stage |= SIPA_FORCE_SUSPEND;

	dev_info(dev, "sipa prepare suspend finish\n");

	return 0;
}

static int sipa_rm_prepare_release(void *priv)
{
	struct sipa_context *ipa = priv;
	struct sipa_control *ctrl = dev_get_drvdata(ipa->pdev);

	ctrl->params_cfg.suspend_cnt++;
	ctrl->power_flag = false;
	cancel_delayed_work(&ctrl->resume_work);
	queue_delayed_work(ctrl->power_wq, &ctrl->suspend_work, 0);

	return 0;
}

static int sipa_rm_prepare_resume(void *priv)
{
	struct sipa_context *ipa = priv;
	struct sipa_control *ctrl = dev_get_drvdata(ipa->pdev);

	ctrl->params_cfg.resume_cnt++;
	ctrl->power_flag = true;
	cancel_delayed_work(&ctrl->suspend_work);
	queue_delayed_work(ctrl->power_wq, &ctrl->resume_work, 0);

	/* TODO: will remove the error code in future */
	return -EINPROGRESS;
}

int sipa_get_ep_info(enum sipa_ep_id id, struct sipa_to_pam_info *out)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_endpoint *ep;

	if (!out || id < 0 || id >= SIPA_EP_MAX)
		return -EINVAL;
	if (!ctrl || !ctrl->ctx)
		return -EPROBE_DEFER;
	ep = ctrl->eps[id];
	if (!ep) {
		dev_err(ctrl->ctx->pdev, "ep id:%d not create!", id);
		return -EPROBE_DEFER;
	}
	if (SIPA_EP_USB == id || SIPA_EP_WIFI == id || SIPA_EP_PCIE == id)
		sipa_hal_init_pam_param(ep->recv_fifo.idx, ep->send_fifo.idx,
					out);
	else
		sipa_hal_init_pam_param(ep->send_fifo.idx, ep->recv_fifo.idx,
					out);

	return 0;
}
EXPORT_SYMBOL(sipa_get_ep_info);

/* Process context. The caller owns its RM grant and keeps the PAM producer
 * disabled until this returns zero. -EUCLEAN means ownership is retained:
 * preserve DMA/power and complete START, external DMA drain, then END.
 */
int sipa_pam_connect(const struct sipa_connect_params *in)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_endpoint *ep;
	int ret, rollback;

	if (!in || in->id < 0 || in->id >= SIPA_EP_MAX)
		return -EINVAL;
	/* Pair with context publication after endpoint initialization. */
	if (!ctrl || !smp_load_acquire(&ctrl->ctx))
		return -EPROBE_DEFER;
	ep = ctrl->eps[in->id];
	if (!ep)
		return -EPROBE_DEFER;
	if (in->id != SIPA_EP_USB && in->id != SIPA_EP_WIFI &&
	    in->id != SIPA_EP_VCP)
		return -EOPNOTSUPP;
	if (in->id == SIPA_EP_USB && in->usb_format != SIPA_USB_RNDIS &&
	    in->usb_format != SIPA_USB_MBIM)
		return -EINVAL;
	/* No safe partial-publication rollback exists for caller-owned pools. */
	if (in->send_param.data_ptr || in->send_param.data_ptr_cnt ||
	    in->recv_param.data_ptr || in->recv_param.data_ptr_cnt)
		return -EOPNOTSUPP;

	mutex_lock(&sipa_pam_lock);
	if (ep->pam_enabled || ep->connected) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = sipa_set_enabled(true);
	if (ret)
		goto unlock;
	ep->pam_enabled = true;
	ret = sipa_resume_for_pamu3(ctrl);
	if (ret) {
		rollback = sipa_set_enabled(false);
		if (rollback)
			ret = -EUCLEAN;
		else
			ep->pam_enabled = false;
		goto unlock;
	}

	mutex_lock(&ctrl->resume_lock);
	if (in->id == SIPA_EP_USB) {
		/* Enabled and resumed above, before publishing any USB descriptor. */
		ret = sipa_hal_set_usb_format(in->usb_format);
		if (ret)
			goto retain;
		ctrl->params_cfg.usb_format = in->usb_format;
	}
	ep->send_fifo_param = in->send_param;
	ep->recv_fifo_param = in->recv_param;
	ep->send_notify = in->send_notify;
	ep->recv_notify = in->recv_notify;
	ep->send_priv = in->send_priv;
	ep->recv_priv = in->recv_priv;
	/* Stop new SIPA ingress before any FIFO configuration can be published. */
	ret = sipa_hal_cmn_fifo_set_receive(ctrl->ctx->hdl, ep->recv_fifo.idx,
					    true);
	if (ret)
		goto retain;
	ret = sipa_open_common_fifo(ep->sipa_ctx->hdl, ep->send_fifo.idx,
				    &ep->send_fifo_param, NULL, false,
				    (sipa_hal_notify_cb)in->send_notify, ep);
	if (ret)
		goto retain;
	ret = sipa_open_common_fifo(ep->sipa_ctx->hdl, ep->recv_fifo.idx,
				    &ep->recv_fifo_param, NULL, false,
				    (sipa_hal_notify_cb)in->recv_notify, ep);
	if (ret)
		goto retain;
	ret = sipa_hal_cmn_fifo_set_receive(ctrl->ctx->hdl, ep->recv_fifo.idx,
					    false);
	if (ret)
		goto retain;
	ep->suspended = false;
	ep->pam_stopping = false;
	ep->pam_idle_verified = false;
	WRITE_ONCE(ep->connected, true);
	/* Publish the configured endpoint before admitting USB transmitters. */
	if (in->id == SIPA_EP_USB)
		smp_store_release(&ctrl->usb_tx_blocked, false);
	mutex_unlock(&ctrl->resume_lock);
	goto unlock;
retain:
	/* We cannot prove that a partially programmed FIFO has no DMA owner. */
	dev_err(ctrl->ctx->pdev,
		"PAM ep %d connect failed (%d), retaining ownership\n", in->id,
		ret);
	ret = -EUCLEAN;
	mutex_unlock(&ctrl->resume_lock);
unlock:
	mutex_unlock(&sipa_pam_lock);
	return ret;
}
EXPORT_SYMBOL(sipa_pam_connect);

int sipa_ext_open_pcie(struct sipa_pcie_open_params *in)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_endpoint *ep = ctrl->eps[SIPA_EP_PCIE];

	if (ep) {
		dev_err(ctrl->ctx->pdev, "pcie already create!");
		return -EINVAL;
	}
	ep = kzalloc_obj(*ep, GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ctrl->eps[SIPA_EP_PCIE] = ep;

	ep->id = SIPA_EP_PCIE;

	ep->sipa_ctx = ctrl->ctx;
	ep->send_fifo.idx = SIPA_FIFO_PCIE_UL;
	ep->send_fifo.rx_fifo.fifo_depth = in->ext_send_param.rx_depth;
	ep->send_fifo.tx_fifo.fifo_depth = in->ext_send_param.tx_depth;
	ep->send_fifo.src_id = SIPA_TERM_PCIE0;
	ep->send_fifo.dst_id = SIPA_TERM_VCP;

	ep->recv_fifo.idx = SIPA_FIFO_PCIE_DL;
	ep->recv_fifo.rx_fifo.fifo_depth = in->ext_recv_param.rx_depth;
	ep->recv_fifo.tx_fifo.fifo_depth = in->ext_recv_param.tx_depth;
	ep->recv_fifo.src_id = SIPA_TERM_PCIE0;
	ep->recv_fifo.dst_id = SIPA_TERM_VCP;

	ep->send_notify = in->send_notify;
	ep->recv_notify = in->recv_notify;
	ep->send_priv = in->send_priv;
	ep->recv_priv = in->recv_priv;
	ep->connected = true;
	ep->suspended = false;
	memcpy(&ep->send_fifo_param, &in->send_param,
	       sizeof(struct sipa_comm_fifo_params));
	memcpy(&ep->recv_fifo_param, &in->recv_param,
	       sizeof(struct sipa_comm_fifo_params));

	sipa_open_common_fifo(ep->sipa_ctx->hdl, ep->send_fifo.idx,
			      &ep->send_fifo_param, &in->ext_send_param, false,
			      (sipa_hal_notify_cb)ep->send_notify, ep);

	sipa_open_common_fifo(ep->sipa_ctx->hdl, ep->recv_fifo.idx,
			      &ep->recv_fifo_param, &in->ext_recv_param, false,
			      (sipa_hal_notify_cb)ep->recv_notify, ep);
	return 0;
}
EXPORT_SYMBOL(sipa_ext_open_pcie);

int sipa_sw_connect(const struct sipa_connect_params *in)
{
	/* The GPL BSP never implemented a software endpoint connection.
	 * Do not expose its unconditional success as a working AP API.
	 */
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(sipa_sw_connect);

/* START prevents new SIPA ingress but is not a DMA-drain acknowledgment.
 * Before END, the caller must complete its PAM RELEASE/DONE handshake, disable
 * its producer, prove TX/RX DMA idle, and stop/release its external endpoints.
 * On any error, all caller-owned resources and the RM grant must remain live.
 */
int sipa_disconnect(enum sipa_ep_id ep_id, enum sipa_disconnect_id stage)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_endpoint *ep;
	int ret = 0;

	if (ep_id < 0 || ep_id >= SIPA_EP_MAX ||
	    (stage != SIPA_DISCONNECT_START && stage != SIPA_DISCONNECT_END &&
	     stage != SIPA_DISCONNECT_RECLAIM))
		return -EINVAL;
	/* Pair with context publication after endpoint initialization. */
	if (!ctrl || !smp_load_acquire(&ctrl->ctx))
		return -EPROBE_DEFER;
	ep = ctrl->eps[ep_id];
	if (!ep)
		return -ENODEV;
	mutex_lock(&sipa_pam_lock);
	mutex_lock(&ctrl->resume_lock);
	if (!ep->pam_enabled) {
		ret = ep->connected ? -EOPNOTSUPP : 0;
		goto out;
	}
	if (stage == SIPA_DISCONNECT_RECLAIM) {
		int resumed;

		/* The caller has ended DWC transfers and reset the exclusive PAM
		 * master while retaining every allocation. Return published USB DL
		 * nodes through the normal completion FIFO, not by editing counts
		 * or directly freeing an skb still owned by SIPA.
		 */
		if (ep_id != SIPA_EP_USB ||
		    !smp_load_acquire(&ctrl->usb_tx_blocked)) {
			ret = -EINVAL;
			goto out;
		}
		ret = sipa_hal_cmn_fifo_set_receive(ctrl->ctx->hdl,
						    ep->recv_fifo.idx, true);
		if (ret)
			goto out;
		if (sipa_hal_ctrl_action(false) != 1) {
			sipa_hal_ctrl_action(true);
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = sipa_hal_reclaim_unuse_node(ctrl->ctx->hdl,
						  ep->recv_fifo.idx);
		resumed = sipa_hal_ctrl_action(true);
		if (resumed != 1)
			ret = -ETIMEDOUT;
		if (!ret) {
			ep->pam_stopping = true;
			ep->pam_idle_verified = false;
		}
		goto out;
	}
	if (stage == SIPA_DISCONNECT_START) {
		if (ep_id == SIPA_EP_USB) {
			/* No new AP_ETH/AP_IP TX may reach USB after this grace period.
			 * PAM stays active while already-published packets complete.
			 */
			smp_store_release(&ctrl->usb_tx_blocked, true);
			synchronize_rcu();
			if (atomic_read_acquire(&ctrl->usb_tx_pending)) {
				ret = -EAGAIN;
				goto out;
			}
		}
		ret = sipa_hal_cmn_fifo_set_receive(ctrl->ctx->hdl,
						    ep->recv_fifo.idx, true);
		if (!ret)
			WRITE_ONCE(ep->pam_stopping, true);
		goto out;
	}
	/* Observe the disconnect gate before testing in-flight transmitters. */
	if (ep_id == SIPA_EP_USB &&
	    (!smp_load_acquire(&ctrl->usb_tx_blocked) ||
	     atomic_read_acquire(&ctrl->usb_tx_pending))) {
		ret = -EAGAIN;
		goto out;
	}
	if (!ep->pam_stopping) {
		ret = -EINVAL;
		goto out;
	}
	if (!ep->pam_idle_verified) {
		if (!sipa_hal_check_send_cmn_fifo_com(ctrl->ctx->hdl,
						      ep->send_fifo.idx)) {
			ret = -EAGAIN;
			goto out;
		}
		/* Pause is only the SIPA half of quiescence; external drain is above. */
		if (sipa_hal_ctrl_action(false) != 1) {
			ret = -ETIMEDOUT;
			goto out;
		}
		/* Do not force-copy unmatched descriptors: no evidence says that
		 * HW_READY_FOR_CHECK drains outstanding AXI writes. Only a closed
		 * ownership ledger can retire the connection without guessing.
		 */
		ret = sipa_hal_pam_fifo_idle(ctrl->ctx->hdl, ep->recv_fifo.idx);
		if (ret) {
			if (sipa_hal_ctrl_action(true) != 1)
				ret = -ETIMEDOUT;
			goto out;
		}
		ep->pam_idle_verified = true;
	}
	if (sipa_hal_ctrl_action(true) != 1) {
		ret = -ETIMEDOUT;
		goto out;
	}
	/* Drain the IRQ callback before the last enable reference can gate MMIO. */
	sipa_hal_detach_fifo_callback(ctrl->ctx->hdl, ep->send_fifo.idx);
	sipa_hal_detach_fifo_callback(ctrl->ctx->hdl, ep->recv_fifo.idx);
	ret = sipa_set_enabled(false);
	if (ret)
		goto out;
	ep->pam_enabled = false;
	ep->suspended = true;
	WRITE_ONCE(ep->connected, false);
	ep->send_notify = NULL;
	ep->recv_notify = NULL;
	ep->send_priv = NULL;
	ep->recv_priv = NULL;
	ep->pam_stopping = false;
	ep->pam_idle_verified = false;
out:
	mutex_unlock(&ctrl->resume_lock);
	mutex_unlock(&sipa_pam_lock);
	return ret;
}
EXPORT_SYMBOL(sipa_disconnect);

struct sipa_control *sipa_get_ctrl_pointer(void)
{
	return s_sipa_ctrl;
}
EXPORT_SYMBOL(sipa_get_ctrl_pointer);

u32 sipa_get_suspend_status(void)
{
	return s_sipa_ctrl->suspend_stage;
}
EXPORT_SYMBOL(sipa_get_suspend_status);

int sipa_enable_receive(enum sipa_ep_id ep_id, bool enabled)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_endpoint *ep;
	int ret;

	if (ep_id < 0 || ep_id >= SIPA_EP_MAX)
		return -EINVAL;
	/* Pair with context publication after endpoint initialization. */
	if (!ctrl || !smp_load_acquire(&ctrl->ctx))
		return -EPROBE_DEFER;
	ep = ctrl->eps[ep_id];
	if (!ep)
		return -ENODEV;
	mutex_lock(&sipa_pam_lock);
	if (!ep->connected || (enabled && ep->pam_stopping))
		ret = -ESHUTDOWN;
	else
		ret = sipa_hal_cmn_fifo_set_receive(
			ep->sipa_ctx->hdl, ep->recv_fifo.idx, !enabled);
	mutex_unlock(&sipa_pam_lock);
	return ret;
}
EXPORT_SYMBOL(sipa_enable_receive);

int sipa_set_enabled(bool enable)
{
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_plat_drv_cfg *cfg;
	unsigned long flags;
	int ret = 0;

	if (!ctrl)
		return -EPROBE_DEFER;
	cfg = &ctrl->params_cfg;
	spin_lock_irqsave(&cfg->enable_lock, flags);
	if (enable) {
		if (cfg->enable_cnt == U32_MAX) {
			ret = -EOVERFLOW;
			goto out;
		}
		if (!cfg->enable_cnt)
			ret = sipa_hal_set_enabled(cfg, true);
		if (!ret)
			cfg->enable_cnt++;
	} else {
		if (!cfg->enable_cnt) {
			ret = -EINVAL;
			goto out;
		}
		if (cfg->enable_cnt == 1)
			ret = sipa_hal_set_enabled(cfg, false);
		if (!ret)
			cfg->enable_cnt--;
	}
out:
	spin_unlock_irqrestore(&cfg->enable_lock, flags);
	return ret;
}
EXPORT_SYMBOL(sipa_set_enabled);

/* Bind USB resource policy to the actual modem transport node.  A missing
 * declaration keeps cellular/AP transport usable but rejects USB offload.
 */
static int sipa_parse_usb_transport(struct device *dev,
				    struct sipa_plat_drv_cfg *cfg)
{
	struct device_node *peer, *rc;
	struct of_phandle_args mbox;
	const char *type;
	u32 dst, core, ep;
	int ret = -EINVAL;

	cfg->usb_remote_prod = -ENODEV;
	peer = of_parse_phandle(dev->of_node, "sprd,cp-transport", 0);
	if (!peer)
		return 0;
	if (!of_device_is_available(peer) ||
	    !of_device_is_compatible(peer, "sprd,sipc") ||
	    of_property_read_string(peer, "sprd,type", &type) ||
	    of_property_read_u32(peer, "sprd,dst", &dst))
		goto put;
	if (!strcmp(type, "mbox")) {
		/* The local CP delegate is explicitly PSCP dst5/core2. */
		if (cfg->need_through_pcie || cfg->standalone_subsys ||
		    dst != 5 ||
		    of_property_read_u32(peer, "sprd,core", &core) || core != 2)
			goto put;
		ret = of_parse_phandle_with_args(peer, "mboxes", "#mbox-cells",
						 0, &mbox);
		if (ret)
			goto put;
		ret = mbox.args_count == 1 && mbox.args[0] == core &&
				      of_device_is_available(mbox.np) &&
				      of_device_is_compatible(
					      mbox.np, "sprd,orca-mailbox") ?
			      0 :
			      -EINVAL;
		of_node_put(mbox.np);
		if (!ret)
			cfg->usb_remote_prod = SIPA_RM_RES_PROD_CP;
	} else if (!strcmp(type, "pcie")) {
		/* This only identifies the dependency.  A real host-resource
		 * producer must still be registered and grant every request.
		 */
		if (!cfg->need_through_pcie ||
		    of_property_read_u32(peer, "sprd,ep-dev", &ep) || ep != 0)
			goto put;
		rc = of_parse_phandle(peer, "sprd,rc-ctrl", 0);
		if (!rc)
			goto put;
		ret = of_device_is_available(rc) &&
				      of_device_is_compatible(
					      rc, "sprd,orca-pcie") ?
			      0 :
			      -ENODEV;
		of_node_put(rc);
		if (!ret)
			cfg->usb_remote_prod = SIPA_RM_RES_PROD_PCIE3;
	}
put:
	of_node_put(peer);
	return ret;
}

static int sipa_parse_dts_configuration(struct platform_device *pdev,
					struct sipa_plat_drv_cfg *cfg)
{
	int i, ret;
	u32 fifo_info[2];
	u32 reg_info[2];
	struct resource *resource;
	const struct sipa_hw_data *pdata;

	/* get IPA  global  register  offset */
	pdata = of_device_get_match_data(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "No matching driver data found\n");
		return -EINVAL;
	}
#ifdef CONFIG_DEBUG_FS
	cfg->debugfs_data = pdata;
#endif
	cfg->standalone_subsys = pdata->standalone_subsys;
	/* get IPA global register base  address */
	resource =
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "glb-base");
	if (!resource) {
		dev_err(&pdev->dev, "get resource failed for glb-base!\n");
		return -ENODEV;
	}
	cfg->glb_phy = resource->start;
	cfg->glb_size = resource_size(resource);

	/* get IPA iram base  address */
	resource =
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "iram-base");
	if (!resource) {
		dev_err(&pdev->dev, "get resource failed for iram-base!\n");
		return -ENODEV;
	}
	cfg->iram_phy = resource->start;
	cfg->iram_size = resource_size(resource);

	/* get IRQ numbers */
	cfg->ipa_intr = platform_get_irq_byname(pdev, "local_ipa_irq");
	if (cfg->ipa_intr < 0) {
		return dev_err_probe(&pdev->dev, cfg->ipa_intr,
				     "get IPA IRQ\n");
	}
	dev_info(&pdev->dev, "ipa intr num = %d\n", cfg->ipa_intr);

	/* get IPA bypass mode */
	ret = of_property_read_u32(pdev->dev.of_node, "sprd,sipa-bypass-mode",
				   &cfg->is_bypass);
	if (ret)
		dev_info(&pdev->dev, "using non-bypass mode by default\n");
	else
		dev_info(&pdev->dev, "using bypass mode =%d", cfg->is_bypass);

	/* get through pcie flag */
	cfg->need_through_pcie = of_property_read_bool(
		pdev->dev.of_node, "sprd,need-through-pcie");

	ret = sipa_parse_usb_transport(&pdev->dev, cfg);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "invalid CP transport topology\n");

	/* get wiap ul dma flag */
	cfg->wiap_ul_dma =
		of_property_read_bool(pdev->dev.of_node, "sprd,wiap-ul-dma");

	/* get tft mode flag */
	cfg->tft_mode =
		of_property_read_bool(pdev->dev.of_node, "sprd,tft-mode");

	/* init enable register locks */
	spin_lock_init(&cfg->enable_lock);
	cfg->enable_cnt = 0;
	/* get enable register information */
	cfg->sys_regmap = syscon_regmap_lookup_by_phandle_args(
		pdev->dev.of_node, "sprd,enable-syscon", 2, reg_info);
	if (IS_ERR(cfg->sys_regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(cfg->sys_regmap),
				     "missing SIPA enable syscon\n");
	if (!reg_info[1])
		return -EINVAL;
	cfg->enable_reg = reg_info[0];
	cfg->enable_mask = reg_info[1];

	/* get wakeup register information */
	if (cfg->standalone_subsys) {
		cfg->vpower = devm_regulator_get(&pdev->dev, "vpower");
		if (IS_ERR(cfg->vpower)) {
			ret = PTR_ERR(cfg->vpower);
			dev_err(&pdev->dev, "unable to get vpower supply %d\n",
				ret);
			return ret;
		}
	}

	/* get IPA fifo memory settings */
	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		/* free fifo info */
		ret = of_property_read_u32_array(
			pdev->dev.of_node, sipa_common_fifo_statics[i].tx_fifo,
			(u32 *)fifo_info, 2);
		if (!ret) {
			cfg->common_fifo_cfg[i].tx_fifo.in_iram = fifo_info[0];
			cfg->common_fifo_cfg[i].tx_fifo.fifo_size =
				fifo_info[1];
		}
		/* filled fifo info */
		ret = of_property_read_u32_array(
			pdev->dev.of_node, sipa_common_fifo_statics[i].rx_fifo,
			(u32 *)fifo_info, 2);
		if (!ret) {
			cfg->common_fifo_cfg[i].rx_fifo.in_iram = fifo_info[0];
			cfg->common_fifo_cfg[i].rx_fifo.fifo_size =
				fifo_info[1];
		}
		if (sipa_common_fifo_statics[i].is_to_ipa)
			cfg->common_fifo_cfg[i].is_recv = false;
		else
			cfg->common_fifo_cfg[i].is_recv = true;

		cfg->common_fifo_cfg[i].src =
			sipa_common_fifo_statics[i].src_id;
		cfg->common_fifo_cfg[i].dst =
			sipa_common_fifo_statics[i].dst_id;
		cfg->common_fifo_cfg[i].is_pam =
			sipa_common_fifo_statics[i].is_pam;
	}

	return 0;
}

static int ipa_pre_init(struct sipa_plat_drv_cfg *cfg)
{
	int ret;

	cfg->name = DRV_LOCAL_NAME;
	cfg->class = class_create(cfg->name);
	if (IS_ERR(cfg->class))
		return PTR_ERR(cfg->class);
	ret = alloc_chrdev_region(&cfg->dev_num, 0, 1, cfg->name);
	if (ret)
		goto destroy_class;
	cdev_init(&cfg->cdev, &sipa_local_drv_fops);
	cfg->cdev.owner = THIS_MODULE;
	ret = cdev_add(&cfg->cdev, cfg->dev_num, 1);
	if (ret)
		goto unregister_region;
	cfg->dev = device_create(cfg->class, NULL, cfg->dev_num, cfg,
				 DRV_LOCAL_NAME);
	if (IS_ERR(cfg->dev)) {
		ret = PTR_ERR(cfg->dev);
		cdev_del(&cfg->cdev);
		goto unregister_region;
	}
	return 0;
unregister_region:
	unregister_chrdev_region(cfg->dev_num, 1);
destroy_class:
	class_destroy(cfg->class);
	return ret;
}

static int create_sipa_ep_from_fifo_idx(struct device *dev,
					enum sipa_cmn_fifo_index fifo_idx,
					struct sipa_plat_drv_cfg *cfg,
					struct sipa_context *ipa)
{
	enum sipa_ep_id ep_id;
	struct sipa_common_fifo *fifo;
	struct sipa_endpoint *ep = NULL;
	struct sipa_common_fifo_info *fifo_info;
	struct sipa_control *ctrl = dev_get_drvdata(dev);

	fifo_info = (struct sipa_common_fifo_info *)sipa_common_fifo_statics;
	ep_id = (fifo_info + fifo_idx)->relate_ep;

	ep = ctrl->eps[ep_id];
	if (!ep) {
		ep = kzalloc_obj(*ep, GFP_KERNEL);
		if (!ep)
			return -ENOMEM;

		ctrl->eps[ep_id] = ep;
	}

	ep->sipa_ctx = ipa;
	ep->id = (fifo_info + fifo_idx)->relate_ep;
	dev_info(dev, "idx = %d ep = %d ep_id = %d is_to_ipa = %d\n", fifo_idx,
		 ep->id, ep_id, (fifo_info + fifo_idx)->is_to_ipa);

	ep->connected = false;
	ep->suspended = true;

	if (!(fifo_info + fifo_idx)->is_to_ipa) {
		fifo = &ep->recv_fifo;
		fifo->is_receiver = true;
		fifo->rx_fifo.fifo_depth =
			cfg->common_fifo_cfg[fifo_idx].rx_fifo.fifo_size;
		fifo->tx_fifo.fifo_depth =
			cfg->common_fifo_cfg[fifo_idx].tx_fifo.fifo_size;
	} else {
		fifo = &ep->send_fifo;
		fifo->is_receiver = false;
		fifo->rx_fifo.fifo_depth =
			cfg->common_fifo_cfg[fifo_idx].rx_fifo.fifo_size;
		fifo->tx_fifo.fifo_depth =
			cfg->common_fifo_cfg[fifo_idx].tx_fifo.fifo_size;
	}
	fifo->dst_id = (fifo_info + fifo_idx)->dst_id;
	fifo->src_id = (fifo_info + fifo_idx)->src_id;

	fifo->idx = fifo_idx;

	return 0;
}

static void destroy_sipa_ep_from_fifo_idx(struct device *dev,
					  enum sipa_cmn_fifo_index fifo_idx,
					  struct sipa_plat_drv_cfg *cfg,
					  struct sipa_context *ipa)
{
	struct sipa_endpoint *ep = NULL;
	struct sipa_control *ctrl = dev_get_drvdata(dev);
	enum sipa_ep_id ep_id = sipa_common_fifo_statics[fifo_idx].relate_ep;

	ep = ctrl->eps[ep_id];
	if (!ep)
		return;

	kfree(ep);
	ctrl->eps[ep_id] = NULL;
}

static void destroy_sipa_eps(struct device *dev, struct sipa_plat_drv_cfg *cfg,
			     struct sipa_context *ipa)
{
	int i;

	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		if (cfg->common_fifo_cfg[i].tx_fifo.fifo_size > 0)
			destroy_sipa_ep_from_fifo_idx(dev, i, cfg, ipa);
	}
}

static int create_sipa_eps(struct device *dev, struct sipa_plat_drv_cfg *cfg,
			   struct sipa_context *ipa)
{
	int i;
	int ret = 0;

	dev_info(dev, "create sipa eps start\n");
	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		if (cfg->common_fifo_cfg[i].tx_fifo.fifo_size > 0) {
			ret = create_sipa_ep_from_fifo_idx(dev, i, cfg, ipa);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int sipa_create_skb_xfer(struct device *dev, struct sipa_context *ipa)
{
	int ret = 0;
	struct sipa_control *ctrl = dev_get_drvdata(dev);

	ret = create_sipa_skb_sender(ipa, ctrl->eps[SIPA_EP_AP_ETH],
				     SIPA_PKT_ETH, &ctrl->sender[SIPA_PKT_ETH]);
	if (ret) {
		ret = -EFAULT;
		goto sender_fail;
	}

	ret = create_sipa_skb_sender(ipa, ctrl->eps[SIPA_EP_AP_IP], SIPA_PKT_IP,
				     &ctrl->sender[SIPA_PKT_IP]);
	if (ret) {
		ret = -EFAULT;
		goto receiver_fail;
	}

	ret = create_sipa_skb_receiver(ipa, ctrl->eps[SIPA_EP_AP_ETH],
				       &ctrl->receiver[SIPA_PKT_ETH]);

	if (ret) {
		ret = -EFAULT;
		goto receiver_fail;
	}

	ret = create_sipa_skb_receiver(ipa, ctrl->eps[SIPA_EP_AP_IP],
				       &ctrl->receiver[SIPA_PKT_IP]);

	if (ret) {
		ret = -EFAULT;
		goto receiver_fail;
	}

	ret = sipa_rm_inactivity_timer_init(SIPA_RM_RES_CONS_WWAN_UL,
					    SIPA_WWAN_CONS_TIMER);
	if (ret)
		goto receiver_fail;

	return 0;

receiver_fail:
	if (ctrl->receiver[SIPA_PKT_IP])
		ctrl->receiver[SIPA_PKT_IP] = NULL;

	if (ctrl->receiver[SIPA_PKT_ETH])
		ctrl->receiver[SIPA_PKT_ETH] = NULL;

sender_fail:
	if (ctrl->sender[SIPA_PKT_IP])
		ctrl->sender[SIPA_PKT_IP] = NULL;

	if (ctrl->sender[SIPA_PKT_ETH])
		ctrl->sender[SIPA_PKT_ETH] = NULL;

	return ret;
}

static int sipa_create_rm_cons(void)
{
	int ret;
	struct sipa_rm_create_params rm_params = {};

	/* WWAN UL */
	rm_params.name = SIPA_RM_RES_CONS_WWAN_UL;

	ret = sipa_rm_create_resource(&rm_params);
	if (ret)
		return ret;

	/* WWAN DL */
	rm_params.name = SIPA_RM_RES_CONS_WWAN_DL;

	ret = sipa_rm_create_resource(&rm_params);
	if (ret) {
		sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_UL);
		return ret;
	}

	/* USB */
	rm_params.name = SIPA_RM_RES_CONS_USB;

	ret = sipa_rm_create_resource(&rm_params);
	if (ret) {
		sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_UL);
		sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_DL);
		return ret;
	}

	/* WLAN */
	rm_params.name = SIPA_RM_RES_CONS_WLAN;

	ret = sipa_rm_create_resource(&rm_params);
	if (ret) {
		sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_UL);
		sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_DL);
		sipa_rm_delete_resource(SIPA_RM_RES_CONS_USB);
		return ret;
	}

	return 0;
}

static void sipa_destroy_rm_cons(void)
{
	sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_UL);
	sipa_rm_delete_resource(SIPA_RM_RES_CONS_WWAN_DL);
	sipa_rm_delete_resource(SIPA_RM_RES_CONS_USB);
	sipa_rm_delete_resource(SIPA_RM_RES_CONS_WLAN);
}

static int sipa_create_ipa_prod(struct sipa_context *ipa)
{
	int ret;
	struct sipa_rm_create_params rm_params = {};

	/* create prod */
	rm_params.name = SIPA_RM_RES_PROD_IPA;
	rm_params.floor_voltage = 0;
	rm_params.reg_params.notify_cb = NULL;
	rm_params.reg_params.user_data = ipa;
	rm_params.request_resource = sipa_rm_prepare_resume;
	rm_params.release_resource = sipa_rm_prepare_release;
	ret = sipa_rm_create_resource(&rm_params);
	if (ret)
		return ret;

	/* add dependencys */
	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_WWAN_UL,
				     SIPA_RM_RES_PROD_IPA);
	if (ret < 0 && ret != -EINPROGRESS) {
		dev_err(ipa->pdev, "sipa_init: add_dependency WWAN_UL fail.\n");
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_IPA);
		return ret;
	}
	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_WWAN_DL,
				     SIPA_RM_RES_PROD_IPA);
	if (ret < 0 && ret != -EINPROGRESS) {
		dev_err(ipa->pdev, "sipa_init: add_dependency WWAN_DL fail.\n");
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WWAN_UL,
					  SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_IPA);
		return ret;
	}
	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_USB,
				     SIPA_RM_RES_PROD_IPA);
	if (ret < 0 && ret != -EINPROGRESS) {
		dev_err(ipa->pdev, "sipa_init: add_dependency USB fail.\n");
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WWAN_UL,
					  SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WWAN_DL,
					  SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_IPA);
		return ret;
	}
	return 0;
}

static void sipa_destroy_ipa_prod(void)
{
	sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WWAN_UL,
				  SIPA_RM_RES_PROD_IPA);
	sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WWAN_DL,
				  SIPA_RM_RES_PROD_IPA);
	sipa_rm_delete_dependency(SIPA_RM_RES_CONS_USB, SIPA_RM_RES_PROD_IPA);
	sipa_rm_delete_resource(SIPA_RM_RES_PROD_IPA);
}

static int sipa_init(struct sipa_context **ipa_pp,
		     struct sipa_plat_drv_cfg *cfg, struct device *ipa_dev)
{
	int ret = 0;
	struct sipa_context *ipa = NULL;

	ipa = kzalloc_obj(struct sipa_context, GFP_KERNEL);
	if (!ipa)
		return -ENOMEM;

	ipa->pdev = ipa_dev;
	ipa->bypass_mode = cfg->is_bypass;

	ipa->hdl = sipa_hal_init(ipa_dev, cfg);
	if (!ipa->hdl) {
		dev_err(ipa_dev, "sipa_hal_init fail!\n");
		kfree(ipa);
		return -ENODEV;
	}

	/* init sipa eps */
	ret = create_sipa_eps(ipa_dev, cfg, ipa);
	if (ret)
		goto ep_fail;

	/* init resource manager */
	ret = sipa_rm_init();
	if (ret)
		goto ep_fail;

	/* create basic cons */
	ret = sipa_create_rm_cons();
	if (ret)
		goto cons_fail;

#ifdef CONFIG_SPRD_SIPA_USB_ACCEL
	/* USB is optional; local CP/WWAN resource dependencies stay active. */
	ret = sipa_rm_usb_cons_init();
	if (ret)
		goto usb_fail;
#endif

	/* create basic prod */
	ret = sipa_create_ipa_prod(ipa);
	if (ret)
		goto prod_fail;

	/* init sipa skb transfer layer */
	if (!cfg->is_bypass) {
		ret = sipa_create_skb_xfer(ipa_dev, ipa);
		if (ret) {
			ret = -EFAULT;
			goto xfer_fail;
		}
	}

	/* Make the fully initialized endpoint context visible to API users. */
	smp_store_release(ipa_pp, ipa);

	return 0;

xfer_fail:
	sipa_destroy_ipa_prod();
prod_fail:
#ifdef CONFIG_SPRD_SIPA_USB_ACCEL
	sipa_rm_usb_cons_deinit();
usb_fail:
#endif
	sipa_destroy_rm_cons();
cons_fail:
	sipa_rm_exit();
ep_fail:
	destroy_sipa_eps(ipa_dev, cfg, ipa);

	kfree(ipa);
	return ret;
}

static void sipa_notify_sender_flow_ctrl(struct work_struct *work)
{
	int i;
	struct sipa_control *sipa_ctrl =
		container_of(work, struct sipa_control, flow_ctrl_work);

	for (i = 0; i < SIPA_PKT_TYPE_MAX; i++)
		if (sipa_ctrl->sender[i] &&
		    sipa_ctrl->sender[i]->free_notify_net)
			wake_up(&sipa_ctrl->sender[i]->free_waitq);
}

static void sipa_prepare_suspend_work(struct work_struct *work)
{
	struct sipa_control *ctrl = container_of(
		(struct delayed_work *)work, struct sipa_control, suspend_work);

	if (sipa_prepare_suspend(ctrl->ctx->pdev) && !ctrl->power_flag) {
		/* 200ms can ensure that the skb data has been recycled */
		queue_delayed_work(ctrl->power_wq, &ctrl->suspend_work,
				   msecs_to_jiffies(200));
		dev_info(ctrl->ctx->pdev, "sipa schedule_delayed_work\n");
	}
}

static int sipa_plat_drv_probe(struct platform_device *pdev_p)
{
	int ret;
	struct device *dev = &pdev_p->dev;
	struct sipa_control *ctrl;
	/*
	 * SIPA probe function can be called for multiple times as the same probe
	 * function handles multiple compatibilities
	 */
	dev_dbg(dev, "sipa: IPA driver probing started for %s\n",
		pdev_p->dev.of_node->name);

	if (READ_ONCE(s_sipa_ctrl))
		return -EBUSY;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	ctrl = kzalloc_obj(*ctrl, GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->usb_tx_blocked = true;
	atomic_set(&ctrl->usb_tx_pending, 0);
	s_sipa_ctrl = ctrl;
	dev_set_drvdata(&pdev_p->dev, ctrl);

	ret = sipa_parse_dts_configuration(pdev_p, &ctrl->params_cfg);
	if (ret) {
		dev_err(dev, "sipa: dts parsing failed\n");
		WRITE_ONCE(s_sipa_ctrl, NULL);
		dev_set_drvdata(dev, NULL);
		kfree(ctrl);
		return ret;
	}

	ctrl->suspend_stage = SIPA_SUSPEND_MASK;
	ret = ipa_pre_init(&ctrl->params_cfg);
	if (ret) {
		dev_err(dev, "sipa: pre init failed\n");
		WRITE_ONCE(s_sipa_ctrl, NULL);
		dev_set_drvdata(dev, NULL);
		kfree(ctrl);
		return ret;
	}

	mutex_init(&ctrl->resume_lock);
	INIT_WORK(&ctrl->flow_ctrl_work, sipa_notify_sender_flow_ctrl);
	INIT_DELAYED_WORK(&ctrl->resume_work, sipa_resume_work);
	INIT_DELAYED_WORK(&ctrl->suspend_work, sipa_prepare_suspend_work);
	ctrl->power_wq = create_workqueue("sipa_power_wq");
	if (!ctrl->power_wq) {
		dev_err(dev, "sipa power wq create failed\n");
		return -ENOMEM;
	}

	ret = sipa_init(&ctrl->ctx, &ctrl->params_cfg, dev);
	if (ret) {
		dev_err(dev, "sipa: sipa_init failed %d\n", ret);
		return ret;
	}

	device_init_wakeup(dev, true);
	sipa_init_debugfs(&ctrl->params_cfg, ctrl);

	return 0;
}

/* Since different sipa of orca/roc1 series can have different register
 * offset address and register , we should save offset and names
 * in the device data structure.
 */
static struct sipa_hw_data roc1_defs_data = {
	.ahb_reg = sipa_roc1_ahb_regmap,
	.ahb_regnum = ROC1_AHB_MAX_REG,
	.standalone_subsys = true,
};

static struct sipa_hw_data orca_defs_data = {
	.ahb_reg = sipa_orca_ahb_regmap,
	.ahb_regnum = ORCA_AHB_MAX_REG,
	.standalone_subsys = false,
};

static const struct of_device_id sipa_plat_drv_match[] = {
	{ .compatible = "sprd,roc1-sipa", .data = &roc1_defs_data },
	{ .compatible = "sprd,orca-sipa", .data = &orca_defs_data },
	{
		.compatible = "sprd,remote-sipa",
	},
	{}
};

/*
 * sipa_ap_suspend() - suspend callback for runtime_pm
 * @dev: pointer to device
 *
 * This callback will be invoked by the runtime_pm framework when an AP suspend
 * operation is invoked.
 *
 * Returns -EAGAIN to runtime_pm framework in case IPA is in use by AP.
 * This will postpone the suspend operation until IPA is no longer used by AP.
 */
static int sipa_ap_suspend(struct device *dev)
{
	/* Queue/DMA restore has not yet been validated for system sleep. */
	return -EBUSY;
}

/*
 * sipa_ap_resume() - resume callback for runtime_pm
 * @dev: pointer to device
 *
 * This callback will be invoked by the runtime_pm framework when an AP resume
 * operation is invoked.
 *
 * Always returns 0 since resume should always succeed.
 */
static int sipa_ap_resume(struct device *dev)
{
	return 0;
}

/**
 * sipa_get_pdev() - return a pointer to IPA dev struct
 *
 * Return value: a pointer to IPA dev struct
 *
 */
struct device *sipa_get_pdev(void)
{
	struct sipa_control *ctrl = READ_ONCE(s_sipa_ctrl);

	/* Pair with the context publication at the end of initialization. */
	struct sipa_context *ctx = ctrl ? smp_load_acquire(&ctrl->ctx) : NULL;

	return ctx ? ctx->pdev : NULL;
}
EXPORT_SYMBOL(sipa_get_pdev);

static const struct dev_pm_ops sipa_pm_ops = {
	.suspend_noirq = sipa_ap_suspend,
	.resume_noirq = sipa_ap_resume,
};

static struct platform_driver sipa_plat_drv = {
	.probe = sipa_plat_drv_probe,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &sipa_pm_ops,
		.of_match_table = sipa_plat_drv_match,
		/* Built-in backend; full remove/reset ownership is not ready. */
		.suppress_bind_attrs = true,
	},
};

static int __init sipa_module_init(void)
{
	return platform_driver_register(&sipa_plat_drv);
}
module_init(sipa_module_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Spreadtrum IPA HW device driver");
