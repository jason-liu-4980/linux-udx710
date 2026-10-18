// SPDX-License-Identifier: GPL-2.0-only
/*
 * Orca USB PHY AP-side register sequences, adapted to the generic PHY API.
 * Copyright (c) 2019 Spreadtrum Co., Ltd.
 * Original author: Jiayong Yang <jiayong.yang@unisoc.com>
 * Based on phy{0,}-sprd-orca.c at d834a127aa46356c63bd5a59bc6d86c66ea371be.
 * No BC1.2 emulation, speculative host VBUS supply, or undocumented tuning.
 */
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include "phy-sprd-orca-usb-regs.h"

enum orca_usb_bank { ANA, AON, CTRL, GLB, NUM_BANKS };
struct orca_usb_step {
	u32 bank, reg, mask, val;
};
struct orca_usb_data {
	const struct orca_usb_step *on, *glb, *device, *host;
	unsigned int num_on, num_glb, num_device, num_host;
	unsigned int port;
	const char *ctrl_property;
};
struct orca_usb {
	struct device *dev;
	struct regmap *map[NUM_BANKS];
	struct regulator *vdd;
	struct reset_control *reset;
	const struct orca_usb_data *data;
	u32 original[16];
	u32 glb_original[2];
	u32 mode_original[4];
	enum phy_mode mode;
	bool shared_held, glb_set;
	bool active, mode_set, reset_held, supply_on;
};
/*
 * Fixed DWC3 roles use GCTL.PRTCAPDIR. Keep the shared ID input at the
 * established peripheral value; per-port Host pull-downs are independent.
 * Public BSP also tunes both USB2 PHYs from PHY1, so these writes share one
 * reference epoch and are never restored while either port is active.
 */
static DEFINE_MUTEX(orca_usb_shared_lock);
static struct orca_usb *orca_usb_owner[2];
static unsigned int orca_usb_shared_users;
static u32 orca_usb_shared_original[4];

static const struct orca_usb_step orca_usb_common[] = {
	{ AON, REG_AON_APB_RF_OTG_PHY_TEST,
	  MASK_AON_APB_RF_OTG_VBUS_VALID_PHYREG,
	  MASK_AON_APB_RF_OTG_VBUS_VALID_PHYREG },
	{ AON, REG_AON_APB_RF_OTG_PHY_CTRL, MASK_AON_APB_RF_USB2_PHY_IDDIG,
	  MASK_AON_APB_RF_USB2_PHY_IDDIG },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_TRIMMING,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_TUNEHSAMP |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_TFREGRES,
	  (0x11 << 23) | (1 << 29) },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_TRIMMING,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_TUNEHSAMP |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_TFREGRES,
	  (0x11 << 23) | (1 << 29) },
};

static const struct orca_usb_step orca_usb0_on[] = {
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_0_ANA_USB30_CTRL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_0_USB30_PS_PD_S |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_0_USB30_PS_PD_L |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_0_USB30_ISO_SW_EN,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_BATTER_PLL,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_PS_PD_S |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_PS_PD_L,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_ISO_SW,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_ISO_SW_EN, 0 },
	{ CTRL, REG_AP_AHB_RF_USB0_CTRL,
	  MASK_AP_AHB_RF_UTMISRP_BVALID_REG0 |
		  MASK_AP_AHB_RF_OTG_VBUS_VALID_PHYREG0 |
		  MASK_AP_AHB_RF_PIPE3_POWERPRESENT0,
	  MASK_AP_AHB_RF_UTMISRP_BVALID_REG0 |
		  MASK_AP_AHB_RF_OTG_VBUS_VALID_PHYREG0 |
		  MASK_AP_AHB_RF_PIPE3_POWERPRESENT0 },
	{ CTRL, REG_AP_AHB_RF_USB0_CTRL,
	  MASK_AP_AHB_RF_OTG_VBUS_VALID_PHYREG_SEL0 | MASK_AP_AHB_RF_PME_EN0,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_UTMI_CTL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DATABUS16_8 |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_VBUSVLDEXT,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DATABUS16_8 |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_VBUSVLDEXT },
};

/* Read and write these only after the analog on sequence has completed.
 * Exact board stock kernel PHY0 init at 0xffffff800845a818.
 */
static const struct orca_usb_step orca_usb0_glb[] = {
	{ GLB, 0x9c, BIT(0), 0 },
	{ GLB, 0xb4, GENMASK(17, 16), 0 },
};

static const struct orca_usb_step orca_usb0_device[] = {
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_REG_SEL_CFG_0,
	  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_0_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_0_USB20_DMPULLDOWN,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_UTMI_CTL2,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DMPULLDOWN,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_UTMI_CTL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_RESERVED, 0 },
};

static const struct orca_usb_step orca_usb0_host[] = {
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_REG_SEL_CFG_0,
	  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_0_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_0_USB20_DMPULLDOWN,
	  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_0_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_0_USB20_DMPULLDOWN },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_UTMI_CTL2,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DMPULLDOWN,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_DMPULLDOWN },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_UTMI_CTL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_0_USB20_RESERVED, 0x200 },
};

static const struct orca_usb_step orca_usb1_on[] = {
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_1_ANA_USB30_CTRL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_1_USB30_PS_PD_S |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_1_USB30_PS_PD_L |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB3_TYPEC_1_USB30_ISO_SW_EN,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_BATTER_PLL,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_PS_PD_S |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_PS_PD_L,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_ISO_SW,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_ISO_SW_EN, 0 },
	{ CTRL, REG_AP_IPA_AHB_RF_USB1_CTRL,
	  MASK_AP_IPA_AHB_RF_UTMISRP_BVALID_REG1 |
		  MASK_AP_IPA_AHB_RF_OTG_VBUS_VALID_PHYREG1 |
		  MASK_AP_IPA_AHB_RF_PIPE3_POWERPRESENT1,
	  MASK_AP_IPA_AHB_RF_UTMISRP_BVALID_REG1 |
		  MASK_AP_IPA_AHB_RF_OTG_VBUS_VALID_PHYREG1 |
		  MASK_AP_IPA_AHB_RF_PIPE3_POWERPRESENT1 },
	{ CTRL, REG_AP_IPA_AHB_RF_USB1_CTRL,
	  MASK_AP_IPA_AHB_RF_OTG_VBUS_VALID_PHYREG_SEL1 |
		  MASK_AP_IPA_AHB_RF_PME_EN1,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_UTMI_CTL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DATABUS16_8 |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_VBUSVLDEXT,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DATABUS16_8 |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_VBUSVLDEXT },
};

static const struct orca_usb_step orca_usb1_device[] = {
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_REG_SEL_CFG_0,
	  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_1_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_1_USB20_DMPULLDOWN,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_UTMI_CTL2,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DMPULLDOWN,
	  0 },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_UTMI_CTL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_RESERVED, 0 },
};

static const struct orca_usb_step orca_usb1_host[] = {
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_REG_SEL_CFG_0,
	  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_1_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_1_USB20_DMPULLDOWN,
	  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_1_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_DBG_SEL_ANALOG_USB20_1_USB20_DMPULLDOWN },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_UTMI_CTL2,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DMPULLDOWN,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DPPULLDOWN |
		  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_DMPULLDOWN },
	{ ANA, REG_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_UTMI_CTL1,
	  MASK_ANLG_PHY_G4_RF_ANALOG_USB20_1_USB20_RESERVED, 0x200 },
};

static int orca_usb_apply(struct orca_usb *usb, const struct orca_usb_step *s,
			  unsigned int count, u32 *saved)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = regmap_read(usb->map[s[i].bank], s[i].reg, &saved[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < count; i++) {
		ret = regmap_update_bits(usb->map[s[i].bank], s[i].reg,
					 s[i].mask, s[i].val);
		if (ret) {
			/* The failing bus operation may have partially taken effect. */
			do {
				int undo = regmap_update_bits(
					usb->map[s[i].bank], s[i].reg,
					s[i].mask, saved[i]);
				if (undo) {
					dev_err(usb->dev,
						"PHY register rollback failed: %d\n",
						undo);
				}
			} while (i--);
			return ret;
		}
	}
	return 0;
}

static int orca_usb_restore(struct orca_usb *usb)
{
	const struct orca_usb_data *d = usb->data;
	int i, ret, err = 0;

	if (usb->glb_set) {
		for (i = d->num_glb - 1; i >= 0; i--) {
			const struct orca_usb_step *s = &d->glb[i];

			ret = regmap_update_bits(usb->map[s->bank], s->reg,
						 s->mask, usb->glb_original[i]);
			if (ret && !err)
				err = ret;
		}
		usb->glb_set = false;
	}
	for (i = d->num_on - 1; i >= 0; i--) {
		const struct orca_usb_step *s = &d->on[i];

		ret = regmap_update_bits(usb->map[s->bank], s->reg, s->mask,
					 usb->original[i]);
		if (ret && !err)
			err = ret;
	}
	return err;
}

static int orca_usb_shared_get(struct orca_usb *usb)
{
	int ret;

	if (!orca_usb_shared_users) {
		ret = orca_usb_apply(usb, orca_usb_common,
				     ARRAY_SIZE(orca_usb_common),
				     orca_usb_shared_original);
		if (ret)
			return ret;
	}
	orca_usb_shared_users++;
	usb->shared_held = true;
	return 0;
}

static int orca_usb_shared_put(struct orca_usb *usb)
{
	int i, ret, first = 0;

	if (!usb->shared_held)
		return 0;
	if (orca_usb_shared_users == 1) {
		for (i = ARRAY_SIZE(orca_usb_common) - 1; i >= 0; i--) {
			const struct orca_usb_step *s = &orca_usb_common[i];

			ret = regmap_update_bits(usb->map[s->bank], s->reg,
						 s->mask,
						 orca_usb_shared_original[i]);
			if (ret && !first)
				first = ret;
		}
	}
	orca_usb_shared_users--;
	usb->shared_held = false;
	return first;
}

static int orca_usb_init(struct phy *phy)
{
	struct orca_usb *usb = phy_get_drvdata(phy);
	int ret;

	mutex_lock(&orca_usb_shared_lock);
	if (orca_usb_owner[usb->data->port] &&
	    orca_usb_owner[usb->data->port] != usb) {
		ret = -EBUSY;
		goto out;
	}
	if (usb->active) {
		ret = 0;
		goto out;
	}
	if (!usb->supply_on) {
		ret = regulator_enable(usb->vdd);
		if (ret)
			goto out;
		usb->supply_on = true;
	}
	ret = orca_usb_shared_get(usb);
	if (ret)
		goto disable;
	ret = orca_usb_apply(usb, usb->data->on, usb->data->num_on,
			     usb->original);
	if (ret)
		goto disable;
	if (usb->data->num_glb) {
		ret = orca_usb_apply(usb, usb->data->glb, usb->data->num_glb,
				     usb->glb_original);
		if (ret) {
			int undo = orca_usb_restore(usb);

			if (undo)
				dev_err(usb->dev, "PHY cleanup failed: %d\n",
					undo);
			goto disable;
		}
		usb->glb_set = true;
	}
	/* Shared USB1/PAM reset is pulsed only once in the current ownership epoch. */
	ret = reset_control_reset(usb->reset);
	if (ret) {
		int undo = orca_usb_restore(usb);

		if (undo)
			dev_err(usb->dev, "PHY reset rollback failed: %d\n",
				undo);
		goto disable;
	}
	usb->reset_held = true;
	usb->active = true;
	orca_usb_owner[usb->data->port] = usb;
	goto out;
disable:
	{
		int cleanup = orca_usb_shared_put(usb);

		if (cleanup)
			dev_err(usb->dev, "PHY shared cleanup failed: %d\n",
				cleanup);
		if (usb->supply_on) {
			cleanup = regulator_disable(usb->vdd);
			if (cleanup)
				dev_err(usb->dev,
					"PHY regulator disable failed: %d\n",
					cleanup);
			else
				usb->supply_on = false;
		}
	}
out:
	mutex_unlock(&orca_usb_shared_lock);
	return ret;
}

static int orca_usb_exit(struct phy *phy)
{
	struct orca_usb *usb = phy_get_drvdata(phy);
	int ret, err;

	mutex_lock(&orca_usb_shared_lock);
	ret = 0;
	if (usb->mode_set) {
		int i;

		for (i = (usb->mode == PHY_MODE_USB_HOST ?
				  usb->data->num_host :
				  usb->data->num_device) -
			 1;
		     i >= 0; i--) {
			const struct orca_usb_step *s =
				usb->mode == PHY_MODE_USB_HOST ?
					&usb->data->host[i] :
					&usb->data->device[i];
			err = regmap_update_bits(usb->map[s->bank], s->reg,
						 s->mask,
						 usb->mode_original[i]);
			if (err && !ret)
				ret = err;
		}
	}
	err = orca_usb_restore(usb);
	if (!ret)
		ret = err;
	usb->mode_set = false;
	err = orca_usb_shared_put(usb);
	if (!ret)
		ret = err;
	if (usb->supply_on) {
		err = regulator_disable(usb->vdd);
		if (!err)
			usb->supply_on = false;
		if (!ret)
			ret = err;
	}
	if (usb->reset_held) {
		err = reset_control_rearm(usb->reset);
		if (!err)
			usb->reset_held = false;
		if (!ret)
			ret = err;
	}
	usb->active = false;
	orca_usb_owner[usb->data->port] = NULL;
	mutex_unlock(&orca_usb_shared_lock);
	return ret;
}

static int orca_usb_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct orca_usb *usb = phy_get_drvdata(phy);
	int ret;
	/* No OTG role changes after a controller has acquired its fixed mode. */
	if (mode != PHY_MODE_USB_DEVICE && mode != PHY_MODE_USB_HOST)
		return -EOPNOTSUPP;
	mutex_lock(&orca_usb_shared_lock);
	if (!usb->active)
		ret = -EIO;
	else if (usb->mode_set)
		ret = usb->mode == mode ? 0 : -EBUSY;
	else {
		ret = orca_usb_apply(
			usb,
			mode == PHY_MODE_USB_HOST ? usb->data->host :
						    usb->data->device,
			mode == PHY_MODE_USB_HOST ? usb->data->num_host :
						    usb->data->num_device,
			usb->mode_original);
		if (!ret) {
			usb->mode = mode;
			usb->mode_set = true;
		}
	}
	mutex_unlock(&orca_usb_shared_lock);
	return ret;
}

static const struct phy_ops orca_usb_ops = {
	.init = orca_usb_init,
	.exit = orca_usb_exit,
	.set_mode = orca_usb_set_mode,
	.owner = THIS_MODULE,
};

static const struct orca_usb_data orca_usb0_data = {
	.on = orca_usb0_on,
	.num_on = ARRAY_SIZE(orca_usb0_on),
	.glb = orca_usb0_glb,
	.num_glb = ARRAY_SIZE(orca_usb0_glb),
	.device = orca_usb0_device,
	.num_device = ARRAY_SIZE(orca_usb0_device),
	.host = orca_usb0_host,
	.num_host = ARRAY_SIZE(orca_usb0_host),
	.port = 0,
	.ctrl_property = "sprd,syscon-ap-ahb",
};
static const struct orca_usb_data orca_usb1_data = {
	.on = orca_usb1_on,
	.num_on = ARRAY_SIZE(orca_usb1_on),
	.device = orca_usb1_device,
	.num_device = ARRAY_SIZE(orca_usb1_device),
	.host = orca_usb1_host,
	.num_host = ARRAY_SIZE(orca_usb1_host),
	.port = 1,
	.ctrl_property = "sprd,syscon-ipa-ahb",
};

static const struct regmap_config orca_usb_glb_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xfffc,
};

static int orca_usb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct orca_usb *usb;
	struct phy *phy;
	void __iomem *base;
	u32 microvolt;
	int ret, i;

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;
	usb->dev = dev;
	usb->data = of_device_get_match_data(dev);
	usb->map[ANA] = syscon_regmap_lookup_by_phandle(dev->of_node,
							"sprd,syscon-anag4");
	usb->map[AON] = syscon_regmap_lookup_by_phandle(dev->of_node,
							"sprd,syscon-aon-apb");
	usb->map[CTRL] = syscon_regmap_lookup_by_phandle(
		dev->of_node, usb->data->ctrl_property);
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);
	usb->map[GLB] = devm_regmap_init_mmio(dev, base, &orca_usb_glb_config);
	for (i = 0; i < NUM_BANKS; i++)
		if (IS_ERR(usb->map[i]))
			return dev_err_probe(dev, PTR_ERR(usb->map[i]),
					     "Missing PHY syscon\n");
	usb->reset = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(usb->reset))
		return dev_err_probe(dev, PTR_ERR(usb->reset),
				     "Missing shared PHY reset\n");
	/* _optional forbids a dummy supply; every board port requires its real rail. */
	usb->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(usb->vdd))
		return dev_err_probe(dev, PTR_ERR(usb->vdd),
				     "Missing PHY VDD\n");
	ret = of_property_read_u32(dev->of_node, "sprd,vdd-voltage",
				   &microvolt);
	if (ret)
		return ret;
	if (microvolt < 1200000 || microvolt > 3750000)
		return -EINVAL;
	ret = regulator_set_voltage(usb->vdd, microvolt, microvolt);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot set PHY VDD\n");
	phy = devm_phy_create(dev, NULL, &orca_usb_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, usb);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id orca_usb_match[] = {
	{ .compatible = "sprd,orca1-ssphy0", .data = &orca_usb0_data },
	{ .compatible = "sprd,orca1-ssphy1", .data = &orca_usb1_data },
	{}
};
MODULE_DEVICE_TABLE(of, orca_usb_match);
static struct platform_driver orca_usb_driver = {
	.probe = orca_usb_probe,
	.driver = { .name = "sprd-orca-usb-phy",
		    .of_match_table = orca_usb_match,
		    .suppress_bind_attrs = true },
};
module_platform_driver(orca_usb_driver);
MODULE_DESCRIPTION("Orca AP USB PHY");
MODULE_AUTHOR("Jiayong Yang <jiayong.yang@unisoc.com>");
MODULE_LICENSE("GPL");
