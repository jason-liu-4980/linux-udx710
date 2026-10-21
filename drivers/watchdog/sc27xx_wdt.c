// SPDX-License-Identifier: GPL-2.0-only
/* SC2730 PMIC watchdog, accessed through the parent ADI regmap. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

#define SC27XX_WDT_LOAD_LOW	0x00
#define SC27XX_WDT_LOAD_HIGH	0x04
#define SC27XX_WDT_CTRL		0x08
#define SC27XX_WDT_RAW		0x10
#define SC27XX_WDT_COUNT_LOW	0x18
#define SC27XX_WDT_COUNT_HIGH	0x1c
#define SC27XX_WDT_LOCK		0x20
#define SC27XX_WDT_INT_EN	BIT(0)
#define SC27XX_WDT_COUNT_EN	BIT(1)
#define SC27XX_WDT_NEW_VERSION	BIT(2)
#define SC27XX_WDT_RESET_EN	BIT(3)
#define SC27XX_WDT_RESET_RAW	BIT(3)
#define SC27XX_WDT_LOAD_BUSY	BIT(4)
#define SC27XX_WDT_UNLOCK	0xe551
#define SC27XX_WDT_RELOCK	0x1aae
#define SC27XX_WDT_RATE		32768U
#define SC2730_MODULE_EN		0x1808
#define SC2730_RTC_CLK_EN	0x1810
#define SC2730_WDT_GATE		BIT(2)

struct sc27xx_wdt {
	struct regmap *regmap;
	unsigned int base;
	struct watchdog_device wdd;
};

static int sc27xx_wdt_wait_load(struct sc27xx_wdt *wdt)
{
	unsigned int raw;

	return regmap_read_poll_timeout(wdt->regmap, wdt->base + SC27XX_WDT_RAW,
				       raw, !(raw & SC27XX_WDT_LOAD_BUSY), 10, 640);
}

static int sc27xx_wdt_load(struct sc27xx_wdt *wdt, unsigned int seconds)
{
	u32 ticks = seconds * SC27XX_WDT_RATE;
	int ret, lock_ret;

	ret = sc27xx_wdt_wait_load(wdt);
	if (ret)
		return ret;
	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOCK,
			   SC27XX_WDT_UNLOCK);
	if (ret)
		return ret;
	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOAD_HIGH,
			   ticks >> 16);
	if (!ret)
		ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOAD_LOW,
				   ticks & 0xffff);
	/* Relock even if an ADI write fails; do not replay a partial load. */
	lock_ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOCK,
				SC27XX_WDT_RELOCK);
	if (ret || lock_ret)
		return ret ? ret : lock_ret;
	udelay(100);
	return sc27xx_wdt_wait_load(wdt);
}

static int sc27xx_wdt_ping(struct watchdog_device *wdd)
{
	return sc27xx_wdt_load(watchdog_get_drvdata(wdd), wdd->timeout);
}

static int sc27xx_wdt_start(struct watchdog_device *wdd)
{
	struct sc27xx_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret, lock_ret;

	ret = sc27xx_wdt_ping(wdd);
	if (ret)
		return ret;
	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOCK,
			   SC27XX_WDT_UNLOCK);
	if (ret)
		return ret;
	ret = regmap_update_bits(wdt->regmap, wdt->base + SC27XX_WDT_CTRL,
				SC27XX_WDT_INT_EN | SC27XX_WDT_COUNT_EN |
				SC27XX_WDT_NEW_VERSION | SC27XX_WDT_RESET_EN,
				SC27XX_WDT_COUNT_EN | SC27XX_WDT_NEW_VERSION |
				SC27XX_WDT_RESET_EN);
	lock_ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOCK,
				SC27XX_WDT_RELOCK);
	return ret ? ret : lock_ret;
}

static int sc27xx_wdt_stop(struct watchdog_device *wdd)
{
	/* This PMIC watchdog is deliberately nowayout, including during shutdown. */
	return -EBUSY;
}

static int sc27xx_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	int ret = sc27xx_wdt_load(watchdog_get_drvdata(wdd), timeout);

	if (!ret)
		wdd->timeout = timeout;
	return ret;
}

static unsigned int sc27xx_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct sc27xx_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int high, low, again;
	int ret;

	/* ADI exposes two 16-bit halves; retry only an observed rollover. */
	for (unsigned int i = 0; i < 4; i++) {
		ret = regmap_read(wdt->regmap, wdt->base + SC27XX_WDT_COUNT_HIGH, &high);
		if (ret)
			return 0;
		ret = regmap_read(wdt->regmap, wdt->base + SC27XX_WDT_COUNT_LOW, &low);
		if (ret)
			return 0;
		ret = regmap_read(wdt->regmap, wdt->base + SC27XX_WDT_COUNT_HIGH, &again);
		if (ret)
			return 0;
		if (high == again)
			return ((high << 16) | (low & 0xffff)) / SC27XX_WDT_RATE;
	}
	return 0;
}

static const struct watchdog_info sc27xx_wdt_info = {
	.identity = "SC2730 PMIC Watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_CARDRESET,
};

static const struct watchdog_ops sc27xx_wdt_ops = {
	.owner = THIS_MODULE,
	.start = sc27xx_wdt_start,
	.stop = sc27xx_wdt_stop,
	.ping = sc27xx_wdt_ping,
	.set_timeout = sc27xx_wdt_set_timeout,
	.get_timeleft = sc27xx_wdt_get_timeleft,
};

static int sc27xx_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc27xx_wdt *wdt;
	unsigned int control, raw;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	wdt->regmap = dev_get_regmap(dev->parent, NULL);
	if (!wdt->regmap)
		return -ENODEV;
	ret = of_property_read_u32(dev->of_node, "reg", &wdt->base);
	if (ret)
		return ret;
	ret = regmap_update_bits(wdt->regmap, SC2730_MODULE_EN,
				SC2730_WDT_GATE, SC2730_WDT_GATE);
	if (ret)
		return ret;
	ret = regmap_update_bits(wdt->regmap, SC2730_RTC_CLK_EN,
				SC2730_WDT_GATE, SC2730_WDT_GATE);
	if (ret)
		return ret;
	ret = regmap_read(wdt->regmap, wdt->base + SC27XX_WDT_CTRL, &control);
	if (ret)
		return ret;
	ret = regmap_read(wdt->regmap, wdt->base + SC27XX_WDT_RAW, &raw);
	if (ret)
		return ret;

	wdt->wdd.info = &sc27xx_wdt_info;
	wdt->wdd.ops = &sc27xx_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = U32_MAX / SC27XX_WDT_RATE;
	wdt->wdd.timeout = 60;
	/* Firmware may already have consumed the reset latch; do not invent history. */
	wdt->wdd.bootstatus = raw & SC27XX_WDT_RESET_RAW ? WDIOF_CARDRESET : 0;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_set_nowayout(&wdt->wdd, true);
	ret = watchdog_init_timeout(&wdt->wdd, 0, dev);
	if (ret)
		return ret;
	if ((control & (SC27XX_WDT_COUNT_EN | SC27XX_WDT_NEW_VERSION |
			SC27XX_WDT_RESET_EN)) ==
	    (SC27XX_WDT_COUNT_EN | SC27XX_WDT_NEW_VERSION | SC27XX_WDT_RESET_EN))
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
	/* Standard bounded boot handling may ping inherited hardware until first open. */
	return devm_watchdog_register_device(dev, &wdt->wdd);
}

static const struct of_device_id sc27xx_wdt_match[] = {
	{ .compatible = "sprd,sc2730-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc27xx_wdt_match);

static struct platform_driver sc27xx_wdt_driver = {
	.probe = sc27xx_wdt_probe,
	.driver = {
		.name = "sc27xx-wdt",
		.of_match_table = sc27xx_wdt_match,
	},
};
module_platform_driver(sc27xx_wdt_driver);
MODULE_DESCRIPTION("SC2730 PMIC watchdog driver");
MODULE_LICENSE("GPL");
