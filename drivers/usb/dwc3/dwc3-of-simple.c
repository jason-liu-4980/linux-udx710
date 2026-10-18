// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-of-simple.c - OF glue layer for simple integrations
 *
 * Copyright (c) 2015 Texas Instruments Incorporated - https://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This is a combination of the old dwc3-qcom.c by Ivan T. Ivanov
 * <iivanov@mm-sol.com> and the original patch adding support for Xilinx' SoC
 * by Subbaraya Sundeep Bhatta <subbaraya.sundeep.bhatta@xilinx.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/phy/phy.h>
#include <linux/mutex.h>

struct dwc3_of_simple_data {
	const char *const *required_clocks;
	unsigned int num_required_clocks;
	bool early_fixed_phy;
};

static const char *const orca_clocks[] = { "core_clk", "ref_clk", "susp_clk" };
static const struct dwc3_of_simple_data orca_data = {
	.required_clocks = orca_clocks,
	.num_required_clocks = ARRAY_SIZE(orca_clocks),
	/* Its combined USB/PAM reset belongs to the PHY's shared-reset epoch. */
	.early_fixed_phy = true,
};

struct dwc3_of_simple {
	struct device		*dev;
	struct clk_bulk_data	*clks;
	int			num_clocks;
	struct reset_control	*resets;
	bool			need_reset;
	const struct dwc3_of_simple_data *data;
	struct phy *phy;
};

static int dwc3_simple_fixed_mode(struct device_node *np,
				  enum phy_mode *phy_mode)
{
	struct device_node *child = of_get_compatible_child(np, "snps,dwc3");
	const char *mode;
	int ret;

	if (!child)
		return -ENODEV;
	ret = of_property_read_string(child, "dr_mode", &mode);
	if (!ret) {
		if (!strcmp(mode, "peripheral"))
			*phy_mode = PHY_MODE_USB_DEVICE;
		else if (!strcmp(mode, "host"))
			*phy_mode = PHY_MODE_USB_HOST;
		else
			ret = -EOPNOTSUPP;
	}
	of_node_put(child);
	return ret;
}

static int dwc3_simple_check_clocks(struct dwc3_of_simple *simple)
{
	const struct dwc3_of_simple_data *data = simple->data;
	unsigned int i;
	int j;

	if (!data || !data->num_required_clocks)
		return 0;
	if (simple->num_clocks != data->num_required_clocks)
		return -EINVAL;
	for (i = 0; i < data->num_required_clocks; i++) {
		for (j = 0; j < simple->num_clocks; j++)
			if (simple->clks[j].id &&
			    !strcmp(simple->clks[j].id,
				    data->required_clocks[i]))
				break;
		if (j == simple->num_clocks)
			return -EINVAL;
	}
	return 0;
}

static int dwc3_of_simple_probe(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple;
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;

	const struct dwc3_of_simple_data *data = of_device_get_match_data(dev);
	bool early_phy = data && data->early_fixed_phy;
	int ret, cleanup;
	enum phy_mode phy_mode = PHY_MODE_INVALID;

	if (early_phy) {
		/* The fixed-role Orca glue is built in. */
		if (IS_MODULE(CONFIG_USB_DWC3_OF_SIMPLE))
			return dev_err_probe(
				dev, -EOPNOTSUPP,
				"Orca shared ownership requires built-in glue\n");
		ret = dwc3_simple_fixed_mode(np, &phy_mode);
		if (ret)
			return dev_err_probe(
				dev, ret,
				"Explicit host or peripheral mode required\n");
	}
	simple = devm_kzalloc(dev, sizeof(*simple), GFP_KERNEL);
	if (!simple)
		return -ENOMEM;

	platform_set_drvdata(pdev, simple);
	simple->dev = dev;
	simple->data = data;

	/*
	 * Some controllers need to toggle the usb3-otg reset before trying to
	 * initialize the PHY, otherwise the PHY times out.
	 */
	if (of_device_is_compatible(np, "rockchip,rk3399-dwc3"))
		simple->need_reset = true;

	/*
	 * An early PHY owns the shared reset reference; do not acquire a second
	 * exclusive reset or pulse it after the DWC3 child has been initialized.
	 */
	simple->resets =
		early_phy ? NULL :
			    of_reset_control_array_get_optional_exclusive(np);
	if (IS_ERR(simple->resets)) {
		ret = PTR_ERR(simple->resets);
		dev_err(dev, "failed to get device resets, err=%d\n", ret);
		goto err_free;
	}

	ret = reset_control_deassert(simple->resets);
	if (ret)
		goto err_resetc_put;

	ret = clk_bulk_get_all(simple->dev, &simple->clks);
	if (ret < 0)
		goto err_resetc_assert;

	simple->num_clocks = ret;
	ret = dwc3_simple_check_clocks(simple);
	if (ret)
		goto err_clk_put_all;
	if (early_phy) {
		simple->phy = phy_get(dev, "usb");
		if (IS_ERR(simple->phy)) {
			ret = PTR_ERR(simple->phy);
			simple->phy = NULL;
			goto err_clk_put_all;
		}
	}
	ret = clk_bulk_prepare_enable(simple->num_clocks, simple->clks);
	if (ret)
		goto err_phy_put;
	if (early_phy) {
		ret = phy_init(simple->phy);
		if (ret)
			goto err_clk_disable;
		ret = phy_set_mode(simple->phy, phy_mode);
		if (ret)
			goto err_phy_exit;
	}

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		if (early_phy)
			of_platform_depopulate(dev);
		goto err_phy_exit;
	}
	/*
	 * Keep PHY init and gates held. Unsupported system/runtime sleep must not
	 * silently cut a shared PAM epoch while a different owner can use it.
	 */
	if (early_phy)
		return 0;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err_phy_exit:
	if (early_phy) {
		cleanup = phy_exit(simple->phy);
		if (cleanup)
			dev_err(dev, "PHY cleanup failed: %d\n", cleanup);
	}
err_clk_disable:
	clk_bulk_disable_unprepare(simple->num_clocks, simple->clks);
err_phy_put:
	if (simple->phy)
		phy_put(dev, simple->phy);
err_clk_put_all:
	clk_bulk_put_all(simple->num_clocks, simple->clks);

err_resetc_assert:
	reset_control_assert(simple->resets);

err_resetc_put:
	reset_control_put(simple->resets);
err_free:
	return ret;
}

static bool __dwc3_of_simple_teardown(struct dwc3_of_simple *simple)
{
	of_platform_depopulate(simple->dev);
	if (simple->phy) {
		int ret = phy_exit(simple->phy);

		if (ret)
			dev_err(simple->dev, "PHY exit failed: %d\n", ret);
		phy_put(simple->dev, simple->phy);
		simple->phy = NULL;
	}

	clk_bulk_disable_unprepare(simple->num_clocks, simple->clks);
	clk_bulk_put_all(simple->num_clocks, simple->clks);
	simple->num_clocks = 0;

	reset_control_assert(simple->resets);

	reset_control_put(simple->resets);

	if (!simple->data || !simple->data->early_fixed_phy) {
		pm_runtime_disable(simple->dev);
		pm_runtime_put_noidle(simple->dev);
		pm_runtime_set_suspended(simple->dev);
	}
	return true;
}

static void dwc3_of_simple_remove(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);

	__dwc3_of_simple_teardown(simple);
}

static void dwc3_of_simple_shutdown(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);

	/*
	 * The Orca child shuts down first; retain the shared analog/reset epoch
	 * until system power-off, as in its explicit no-power-management contract.
	 */
	if (!simple->data || !simple->data->early_fixed_phy)
		__dwc3_of_simple_teardown(simple);
}

static int __maybe_unused dwc3_of_simple_runtime_suspend(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);

	if (simple->data && simple->data->early_fixed_phy)
		return -EBUSY;
	clk_bulk_disable(simple->num_clocks, simple->clks);

	return 0;
}

static int __maybe_unused dwc3_of_simple_runtime_resume(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);

	if (simple->data && simple->data->early_fixed_phy)
		return -EBUSY;
	return clk_bulk_enable(simple->num_clocks, simple->clks);
}

static int __maybe_unused dwc3_of_simple_suspend(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->data && simple->data->early_fixed_phy)
		return -EBUSY;
	if (simple->need_reset)
		reset_control_assert(simple->resets);

	return 0;
}

static int __maybe_unused dwc3_of_simple_resume(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_deassert(simple->resets);

	return 0;
}

static const struct dev_pm_ops dwc3_of_simple_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_of_simple_suspend, dwc3_of_simple_resume)
	SET_RUNTIME_PM_OPS(dwc3_of_simple_runtime_suspend,
			dwc3_of_simple_runtime_resume, NULL)
};

static const struct of_device_id of_dwc3_simple_match[] = {
	{ .compatible = "sprd,orca-dwc3", .data = &orca_data },
	{ .compatible = "rockchip,rk3399-dwc3" },
	{ .compatible = "sprd,sc9860-dwc3" },
	{ .compatible = "allwinner,sun50i-h6-dwc3" },
	{ .compatible = "hisilicon,hi3670-dwc3" },
	{ .compatible = "hisilicon,hi3798mv200-dwc3" },
	{ .compatible = "intel,keembay-dwc3" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_simple_match);

static struct platform_driver dwc3_of_simple_driver = {
	.probe		= dwc3_of_simple_probe,
	.remove		= dwc3_of_simple_remove,
	.shutdown	= dwc3_of_simple_shutdown,
	.driver		= {
		.name	= "dwc3-of-simple",
		.of_match_table = of_dwc3_simple_match,
		.pm	= &dwc3_of_simple_dev_pm_ops,
		/* Supplier unbind must not bypass the shared PAM/PHY ownership gate. */
		.suppress_bind_attrs = IS_ENABLED(CONFIG_PHY_SPRD_ORCA_USB),
	},
};

module_platform_driver(dwc3_of_simple_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DesignWare USB3 OF Simple Glue Layer");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
