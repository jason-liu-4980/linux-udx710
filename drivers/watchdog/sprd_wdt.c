// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spreadtrum watchdog driver
 * Copyright (C) 2017 Spreadtrum - http://www.spreadtrum.com
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#define SPRD_WDT_LOAD_LOW		0x0
#define SPRD_WDT_LOAD_HIGH		0x4
#define SPRD_WDT_CTRL			0x8
#define SPRD_WDT_INT_CLR		0xc
#define SPRD_WDT_INT_RAW		0x10
#define SPRD_WDT_INT_MSK		0x14
#define SPRD_WDT_CNT_LOW		0x18
#define SPRD_WDT_CNT_HIGH		0x1c
#define SPRD_WDT_LOCK			0x20
#define SPRD_WDT_IRQ_LOAD_LOW		0x2c
#define SPRD_WDT_IRQ_LOAD_HIGH		0x30

/* WDT_CTRL */
#define SPRD_WDT_INT_EN_BIT		BIT(0)
#define SPRD_WDT_CNT_EN_BIT		BIT(1)
#define SPRD_WDT_NEW_VER_EN		BIT(2)
#define SPRD_WDT_RST_EN_BIT		BIT(3)

/* WDT_INT_CLR */
#define SPRD_WDT_INT_CLEAR_BIT		BIT(0)
#define SPRD_WDT_RST_CLEAR_BIT		BIT(3)

/* WDT_INT_RAW */
#define SPRD_WDT_INT_RAW_BIT		BIT(0)
#define SPRD_WDT_RST_RAW_BIT		BIT(3)
#define SPRD_WDT_LD_BUSY_BIT		BIT(4)

/* 1s equal to 32768 counter steps */
#define SPRD_WDT_CNT_STEP		32768

#define SPRD_WDT_UNLOCK_KEY		0xe551
#define SPRD_WDT_MIN_TIMEOUT		3
#define SPRD_WDT_MAX_TIMEOUT		60

#define SPRD_WDT_CNT_HIGH_SHIFT		16
#define SPRD_WDT_LOW_VALUE_MASK		GENMASK(15, 0)
#define SPRD_WDT_LOAD_POLLS 12

struct sprd_wdt {
	void __iomem *base;
	struct watchdog_device wdd;
	struct clk *enable;
	struct clk *rtc_enable;
	int irq;
	struct mutex mutex;
	spinlock_t reg_lock;
	bool clocks_on;
	bool suspended;
	bool strict_lifecycle;
	bool resume_running;
};

static inline struct sprd_wdt *to_sprd_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct sprd_wdt, wdd);
}

static void sprd_wdt_update_ctrl(struct sprd_wdt *wdt, u32 clear, u32 set)
{
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&wdt->reg_lock, flags);
	writel(SPRD_WDT_UNLOCK_KEY, wdt->base + SPRD_WDT_LOCK);
	value = readl(wdt->base + SPRD_WDT_CTRL);
	writel((value & ~clear) | set, wdt->base + SPRD_WDT_CTRL);
	writel(0, wdt->base + SPRD_WDT_LOCK);
	spin_unlock_irqrestore(&wdt->reg_lock, flags);
}

static int sprd_wdt_wait_load(struct sprd_wdt *wdt)
{
	int i;

	/* 2-3 RTC ticks normally suffice. Check the final allowed sample too. */
	for (i = 0; i < SPRD_WDT_LOAD_POLLS; i++) {
		if (!(readl(wdt->base + SPRD_WDT_INT_RAW) &
		      SPRD_WDT_LD_BUSY_BIT))
			return 0;
		if (i + 1 < SPRD_WDT_LOAD_POLLS)
			usleep_range(10, 100);
	}
	return -EBUSY;
}

static int sprd_wdt_validate(u32 timeout, u32 pretimeout)
{
	/* Validate before multiplication; even UINT_MAX cannot wrap into range. */
	if (timeout < SPRD_WDT_MIN_TIMEOUT || timeout > SPRD_WDT_MAX_TIMEOUT ||
	    pretimeout >= timeout)
		return -EINVAL;
	return 0;
}

static irqreturn_t sprd_wdt_isr(int irq, void *data)
{
	struct sprd_wdt *wdt = data;
	unsigned long flags;
	bool notify;

	spin_lock_irqsave(&wdt->reg_lock, flags);
	if (!(readl(wdt->base + SPRD_WDT_INT_RAW) & SPRD_WDT_INT_EN_BIT)) {
		spin_unlock_irqrestore(&wdt->reg_lock, flags);
		return IRQ_NONE;
	}
	notify = readl(wdt->base + SPRD_WDT_CTRL) & SPRD_WDT_INT_EN_BIT;
	writel(SPRD_WDT_UNLOCK_KEY, wdt->base + SPRD_WDT_LOCK);
	writel(SPRD_WDT_INT_EN_BIT, wdt->base + SPRD_WDT_INT_CLR);
	writel(0, wdt->base + SPRD_WDT_LOCK);
	spin_unlock_irqrestore(&wdt->reg_lock, flags);
	if (notify)
		watchdog_notify_pretimeout(&wdt->wdd);
	return IRQ_HANDLED;
}

static u32 sprd_wdt_get_cnt_value(struct sprd_wdt *wdt)
{
	u32 val;

	val = readl_relaxed(wdt->base + SPRD_WDT_CNT_HIGH) <<
		SPRD_WDT_CNT_HIGH_SHIFT;
	val |= readl_relaxed(wdt->base + SPRD_WDT_CNT_LOW) &
		SPRD_WDT_LOW_VALUE_MASK;

	return val;
}

static int sprd_wdt_load_value(struct sprd_wdt *wdt, u32 timeout,
			       u32 pretimeout)
{
	unsigned long flags;
	u32 ticks, irq_ticks;
	int ret;

	ret = sprd_wdt_validate(timeout, pretimeout);
	if (ret)
		return ret;
	if (!wdt->clocks_on)
		return -EIO;
	ret = sprd_wdt_wait_load(wdt);
	if (ret)
		return ret;

	ticks = timeout * SPRD_WDT_CNT_STEP;
	irq_ticks = pretimeout * SPRD_WDT_CNT_STEP;
	spin_lock_irqsave(&wdt->reg_lock, flags);
	writel(SPRD_WDT_UNLOCK_KEY, wdt->base + SPRD_WDT_LOCK);
	writel(ticks >> 16, wdt->base + SPRD_WDT_LOAD_HIGH);
	writel(ticks & 0xffff, wdt->base + SPRD_WDT_LOAD_LOW);
	writel(irq_ticks >> 16, wdt->base + SPRD_WDT_IRQ_LOAD_HIGH);
	writel(irq_ticks & 0xffff, wdt->base + SPRD_WDT_IRQ_LOAD_LOW);
	writel(0, wdt->base + SPRD_WDT_LOCK);
	spin_unlock_irqrestore(&wdt->reg_lock, flags);

	/* A failure here means the hardware outcome is uncertain, not rolled back. */
	return sprd_wdt_wait_load(wdt);
}

static int sprd_wdt_enable(struct sprd_wdt *wdt)
{
	int ret;

	if (wdt->clocks_on)
		return 0;
	ret = clk_prepare_enable(wdt->enable);
	if (ret)
		return ret;
	ret = clk_prepare_enable(wdt->rtc_enable);
	if (ret) {
		clk_disable_unprepare(wdt->enable);
		return ret;
	}
	wdt->clocks_on = true;
	return 0;
}

static void sprd_wdt_disable(void *data)
{
	struct sprd_wdt *wdt = data;

	/* Never stop firmware-started or nowayout hardware during error unwind. */
	if (!wdt->clocks_on || watchdog_hw_running(&wdt->wdd))
		return;
	clk_disable_unprepare(wdt->rtc_enable);
	clk_disable_unprepare(wdt->enable);
	wdt->clocks_on = false;
}

static int sprd_wdt_start(struct watchdog_device *wdd)
{
	struct sprd_wdt *wdt = to_sprd_wdt(wdd);
	int ret;

	mutex_lock(&wdt->mutex);
	ret = sprd_wdt_load_value(wdt, wdd->timeout, wdd->pretimeout);
	if (!ret) {
		sprd_wdt_update_ctrl(
			wdt, SPRD_WDT_INT_EN_BIT,
			SPRD_WDT_NEW_VER_EN | SPRD_WDT_CNT_EN_BIT |
				SPRD_WDT_RST_EN_BIT |
				(wdd->pretimeout ? SPRD_WDT_INT_EN_BIT : 0));
		set_bit(WDOG_HW_RUNNING, &wdd->status);
	}
	mutex_unlock(&wdt->mutex);
	return ret;
}

static int sprd_wdt_stop(struct watchdog_device *wdd)
{
	struct sprd_wdt *wdt = to_sprd_wdt(wdd);
	int ret = 0;

	mutex_lock(&wdt->mutex);
	if (test_bit(WDOG_NO_WAY_OUT, &wdd->status)) {
		ret = -EBUSY;
	} else if (!wdt->clocks_on) {
		ret = -EIO;
	} else {
		sprd_wdt_update_ctrl(wdt,
				     SPRD_WDT_CNT_EN_BIT | SPRD_WDT_RST_EN_BIT |
					     SPRD_WDT_INT_EN_BIT,
				     0);
		clear_bit(WDOG_HW_RUNNING, &wdd->status);
	}
	mutex_unlock(&wdt->mutex);
	return ret;
}

static int sprd_wdt_set_timeout(struct watchdog_device *wdd, u32 timeout)
{
	struct sprd_wdt *wdt = to_sprd_wdt(wdd);
	int ret;

	mutex_lock(&wdt->mutex);
	ret = sprd_wdt_load_value(wdt, timeout, wdd->pretimeout);
	if (!ret)
		wdd->timeout = timeout;
	mutex_unlock(&wdt->mutex);
	return ret;
}

static int sprd_wdt_set_pretimeout(struct watchdog_device *wdd, u32 pretimeout)
{
	struct sprd_wdt *wdt = to_sprd_wdt(wdd);
	int ret;

	mutex_lock(&wdt->mutex);
	ret = sprd_wdt_load_value(wdt, wdd->timeout, pretimeout);
	if (!ret) {
		sprd_wdt_update_ctrl(wdt, SPRD_WDT_INT_EN_BIT,
				     watchdog_hw_running(wdd) && pretimeout ?
					     SPRD_WDT_INT_EN_BIT :
					     0);
		wdd->pretimeout = pretimeout;
	}
	mutex_unlock(&wdt->mutex);
	return ret;
}

static u32 sprd_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct sprd_wdt *wdt = to_sprd_wdt(wdd);
	u32 val;

	val = sprd_wdt_get_cnt_value(wdt);
	return val / SPRD_WDT_CNT_STEP;
}

static const struct watchdog_ops sprd_wdt_ops = {
	.owner = THIS_MODULE,
	.start = sprd_wdt_start,
	.stop = sprd_wdt_stop,
	.set_timeout = sprd_wdt_set_timeout,
	.set_pretimeout = sprd_wdt_set_pretimeout,
	.get_timeleft = sprd_wdt_get_timeleft,
};

/* Orca counter sampling has not been established: do not expose timeleft. */
static const struct watchdog_ops sprd_wdt_orca_ops = {
	.owner = THIS_MODULE,
	.start = sprd_wdt_start,
	.stop = sprd_wdt_stop,
	.set_timeout = sprd_wdt_set_timeout,
	.set_pretimeout = sprd_wdt_set_pretimeout,
};

static const struct watchdog_info sprd_wdt_info = {
	.options = WDIOF_SETTIMEOUT |
		   WDIOF_PRETIMEOUT |
		   WDIOF_MAGICCLOSE |
		   WDIOF_KEEPALIVEPING,
	.identity = "Spreadtrum Watchdog Timer",
};

static int sprd_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sprd_wdt *wdt;
	u32 ctrl;
	int irq, ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;
	mutex_init(&wdt->mutex);
	spin_lock_init(&wdt->reg_lock);
	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);
	wdt->enable = devm_clk_get(dev, "enable");
	if (IS_ERR(wdt->enable))
		return PTR_ERR(wdt->enable);
	wdt->rtc_enable = devm_clk_get(dev, "rtc_enable");
	if (IS_ERR(wdt->rtc_enable))
		return PTR_ERR(wdt->rtc_enable);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	wdt->irq = irq;

	wdt->wdd.info = &sprd_wdt_info;
	wdt->strict_lifecycle = !!of_device_get_match_data(dev);
	wdt->wdd.ops = wdt->strict_lifecycle ? &sprd_wdt_orca_ops :
					       &sprd_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = SPRD_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = SPRD_WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = SPRD_WDT_MAX_TIMEOUT;
	watchdog_set_nowayout(&wdt->wdd, WATCHDOG_NOWAYOUT);
	ret = watchdog_init_timeout(&wdt->wdd, 0, dev);
	if (ret)
		return ret;
	ret = sprd_wdt_enable(wdt);
	if (ret)
		return ret;
	ctrl = readl(wdt->base + SPRD_WDT_CTRL);
	if (ctrl & SPRD_WDT_CNT_EN_BIT)
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
	ret = devm_add_action_or_reset(dev, sprd_wdt_disable, wdt);
	if (ret)
		return ret;
	/* Do not reinterpret an inherited legacy or interrupt-only counter. */
	if (watchdog_hw_running(&wdt->wdd) &&
	    (ctrl & (SPRD_WDT_NEW_VER_EN | SPRD_WDT_RST_EN_BIT)) !=
		    (SPRD_WDT_NEW_VER_EN | SPRD_WDT_RST_EN_BIT))
		return dev_err_probe(
			dev, -EOPNOTSUPP,
			"firmware watchdog uses unsupported counter/reset mode\n");
	ret = devm_request_irq(dev, irq, sprd_wdt_isr,
			       wdt->strict_lifecycle ? 0 : IRQF_NO_SUSPEND,
			       dev_name(dev), wdt);
	if (ret)
		return ret;

	/* Establish a known deadline before handing a running timer to the core. */
	if (watchdog_hw_running(&wdt->wdd)) {
		ret = sprd_wdt_start(&wdt->wdd);
		if (ret)
			return dev_err_probe(
				dev, ret,
				"firmware watchdog takeover failed\n");
	} else {
		sprd_wdt_update_ctrl(wdt,
				     SPRD_WDT_INT_EN_BIT | SPRD_WDT_RST_EN_BIT,
				     SPRD_WDT_NEW_VER_EN);
	}
	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, wdt);
	return 0;
}

static int __maybe_unused sprd_wdt_pm_suspend(struct device *dev)
{
	struct sprd_wdt *wdt = dev_get_drvdata(dev);
	int ret = 0;

	if (!wdt->strict_lifecycle && watchdog_active(&wdt->wdd)) {
		ret = sprd_wdt_stop(&wdt->wdd);
		if (ret)
			return ret;
		wdt->resume_running = true;
	}
	mutex_lock(&wdt->mutex);
	/* No proven autonomous feeding/clock retention contract across sleep. */
	if (watchdog_hw_running(&wdt->wdd) ||
	    (wdt->strict_lifecycle && watchdog_active(&wdt->wdd))) {
		ret = -EBUSY;
	} else if (!wdt->suspended) {
		/* Drain an in-flight handler before making its registers inaccessible. */
		disable_irq(wdt->irq);
		sprd_wdt_disable(wdt);
		wdt->suspended = true;
	}
	mutex_unlock(&wdt->mutex);
	return ret;
}

static int __maybe_unused sprd_wdt_pm_resume(struct device *dev)
{
	struct sprd_wdt *wdt = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&wdt->mutex);
	ret = sprd_wdt_enable(wdt);
	if (!ret && wdt->suspended) {
		wdt->suspended = false;
		enable_irq(wdt->irq);
	}
	mutex_unlock(&wdt->mutex);
	if (!ret && wdt->resume_running) {
		ret = sprd_wdt_start(&wdt->wdd);
		if (!ret)
			wdt->resume_running = false;
	}
	return ret;
}

static const struct dev_pm_ops sprd_wdt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sprd_wdt_pm_suspend,
				sprd_wdt_pm_resume)
};

static const bool sprd_wdt_orca_strict_lifecycle = true;

static const struct of_device_id sprd_wdt_match_table[] = {
	{
		.compatible = "sprd,sp9860-wdt",
	},
	{ .compatible = "sprd,orca-wdt",
	  .data = &sprd_wdt_orca_strict_lifecycle },
	{},
};
MODULE_DEVICE_TABLE(of, sprd_wdt_match_table);

static struct platform_driver sprd_watchdog_driver = {
	.probe	= sprd_wdt_probe,
	.driver	= {
		.name = "sprd-wdt",
		.of_match_table = sprd_wdt_match_table,
		.pm = &sprd_wdt_pm_ops,
		/* A running timer must not lose its feeding owner through sysfs. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sprd_watchdog_driver);

MODULE_AUTHOR("Eric Long <eric.long@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum Watchdog Timer Controller Driver");
MODULE_LICENSE("GPL");
