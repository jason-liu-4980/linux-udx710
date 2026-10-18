// SPDX-License-Identifier: GPL-2.0-only
/* Orca USB reset groups from Spreadtrum BSP d834a127 (2019).
 * The PHY uses the combined USB1/PAM startup group. PAM can then reset
 * itself independently while holding the controller runtime-PM reference.
 */
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

struct orca_usb_reset {
	struct reset_controller_dev rcdev;
	struct regmap *ap, *ipa;
	struct mutex lock;
};

static int orca_usb_reset_pulse(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct orca_usb_reset *rst =
		container_of(rcdev, struct orca_usb_reset, rcdev);
	struct regmap *map;
	u32 mask;
	int ret;

	if (id > 2)
		return -EINVAL;
	map = id ? rst->ipa : rst->ap;
	/* BSP AP_AHB_USB0=BIT(1), IPA_AHB_USB1=BIT(0), USB_PAM=BIT(5). */
	mask = id == 2 ? BIT(5) : id ? BIT(0) | BIT(5) : BIT(1);
	mutex_lock(&rst->lock);
	ret = regmap_update_bits(map, 0x4, mask, mask);
	if (!ret) {
		if (id == 2)
			udelay(5);
		else
			usleep_range(20000, 30000);
		ret = regmap_update_bits(map, 0x4, mask, 0);
	}
	mutex_unlock(&rst->lock);
	return ret;
}

static const struct reset_control_ops orca_usb_reset_ops = {
	.reset = orca_usb_reset_pulse,
};

static int orca_usb_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct orca_usb_reset *rst;

	rst = devm_kzalloc(dev, sizeof(*rst), GFP_KERNEL);
	if (!rst)
		return -ENOMEM;
	rst->ap = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,ap-ahb");
	if (IS_ERR(rst->ap))
		return PTR_ERR(rst->ap);
	rst->ipa =
		syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,ipa-ahb");
	if (IS_ERR(rst->ipa))
		return PTR_ERR(rst->ipa);
	mutex_init(&rst->lock);
	rst->rcdev.owner = THIS_MODULE;
	rst->rcdev.dev = dev;
	rst->rcdev.of_node = dev->of_node;
	rst->rcdev.ops = &orca_usb_reset_ops;
	rst->rcdev.nr_resets = 3;
	return devm_reset_controller_register(dev, &rst->rcdev);
}

static const struct of_device_id orca_usb_reset_match[] = {
	{ .compatible = "sprd,orca-usb-reset" },
	{}
};
MODULE_DEVICE_TABLE(of, orca_usb_reset_match);
static struct platform_driver orca_usb_reset_driver = {
	.probe = orca_usb_reset_probe,
	.driver = { .name = "sprd-orca-usb-reset",
		    .of_match_table = orca_usb_reset_match,
		    .suppress_bind_attrs = true },
};
module_platform_driver(orca_usb_reset_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Orca USB reset groups and independent PAM reset");
