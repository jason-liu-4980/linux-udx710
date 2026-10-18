// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 * Copyright (C) 2018 Linaro Ltd.
 */

#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/syscore_ops.h>

#define SC27XX_PWR_PD_HW	0xc2c
#define SC27XX_PWR_OFF_EN	BIT(0)
#define SC27XX_SLP_CTRL		0xdf0
#define SC27XX_LDO_XTL_EN	BIT(3)

static struct regmap *regmap;

/*
 * On Spreadtrum platform, we need power off system through external SC27xx
 * series PMICs, and it is one similar SPI bus mapped by regmap to access PMIC,
 * which is not fast io access.
 *
 * So before stopping other cores, we need release other cores' resource by
 * taking cpus down to avoid racing regmap or spi mutex lock when poweroff
 * system through PMIC.
 */
static void sc27xx_poweroff_shutdown(void *data)
{
#ifdef CONFIG_HOTPLUG_CPU
	int cpu;

	for_each_online_cpu(cpu) {
		if (cpu != smp_processor_id())
			remove_cpu(cpu);
	}
#endif
}

static const struct syscore_ops poweroff_syscore_ops = {
	.shutdown = sc27xx_poweroff_shutdown,
};

static struct syscore poweroff_syscore = {
	.ops = &poweroff_syscore_ops,
};

static void sc27xx_poweroff_do_poweroff(void)
{
	/* Disable the external subsys connection's power firstly */
	regmap_write(regmap, SC27XX_SLP_CTRL, SC27XX_LDO_XTL_EN);

	regmap_write(regmap, SC27XX_PWR_PD_HW, SC27XX_PWR_OFF_EN);
}

struct sc27xx_poweroff_variant {
	unsigned int sleep_reg, sleep_value, power_reg, power_value;
};
static const struct sc27xx_poweroff_variant sc2730_data = {
	.sleep_reg = 0x1a48,
	.sleep_value = BIT(2),
	.power_reg = 0x1820,
	.power_value = BIT(0),
};
struct sc27xx_poweroff_context {
	struct device *dev;
	struct regmap *map;
	const struct sc27xx_poweroff_variant *variant;
};

static int sc27xx_poweroff_prepare(struct sys_off_data *data)
{
	struct sc27xx_poweroff_context *off = data->cb_data;
	const struct sc27xx_poweroff_variant *v = off->variant;
	int ret;

	/*
	 * SC2730 ADI/SPI may sleep: after device shutdown, before CPU migration
	 * and atomic syscore shutdown. PSCI remains the genuine final fallback.
	 */
	ret = regmap_write(off->map, v->sleep_reg, v->sleep_value);
	if (!ret)
		ret = regmap_write(off->map, v->power_reg, v->power_value);
	if (ret) {
		dev_emerg(off->dev, "PMIC power-off write failed: %d\n", ret);
		return NOTIFY_DONE;
	}
	msleep(1000);
	dev_emerg(
		off->dev,
		"PMIC power-off request returned; trying firmware fallback\n");
	return NOTIFY_DONE;
}

static int
sc27xx_register_prepare(struct platform_device *pdev,
			const struct sc27xx_poweroff_variant *variant)
{
	struct sc27xx_poweroff_context *off;

	off = devm_kzalloc(&pdev->dev, sizeof(*off), GFP_KERNEL);
	if (!off)
		return -ENOMEM;
	off->dev = &pdev->dev;
	off->variant = variant;
	off->map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!off->map)
		return -ENODEV;
	return devm_register_sys_off_handler(&pdev->dev,
					     SYS_OFF_MODE_POWER_OFF_PREPARE,
					     SYS_OFF_PRIO_PLATFORM,
					     sc27xx_poweroff_prepare, off);
}

static int sc27xx_poweroff_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_id(pdev);

	if (id && id->driver_data)
		return sc27xx_register_prepare(pdev,
					       (const void *)id->driver_data);
	/* Existing SC2731 path is unchanged; its shutdown contract is separate. */
	if (regmap)
		return -EINVAL;

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap)
		return -ENODEV;

	pm_power_off = sc27xx_poweroff_do_poweroff;
	register_syscore(&poweroff_syscore);
	return 0;
}

static const struct platform_device_id sc27xx_poweroff_id_table[] = {
	{ "sc2731-poweroff" },
	{ "sc2730-poweroff", (kernel_ulong_t)&sc2730_data },
	{}
};
MODULE_DEVICE_TABLE(platform, sc27xx_poweroff_id_table);

static struct platform_driver sc27xx_poweroff_driver = {
	.probe = sc27xx_poweroff_probe,
	.driver = {
		.name = "sc27xx-poweroff",
	},
	.id_table = sc27xx_poweroff_id_table,
};
module_platform_driver(sc27xx_poweroff_driver);

MODULE_DESCRIPTION("Power off driver for SC27XX PMIC Device");
MODULE_AUTHOR("Baolin Wang <baolin.wang@unisoc.com>");
MODULE_LICENSE("GPL");
