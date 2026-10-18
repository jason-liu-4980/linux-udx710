// SPDX-License-Identifier: GPL-2.0-only
/* RTC checkpoints before the ADI SPI driver takes ownership. */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/timekeeping.h>
#include <linux/udx710-trial.h>

#define ADI_BASE 0x63a00000u
#define ADI_SLAVE 0x63a20000u

static int __init trial_idle(void __iomem *base)
{
	for (unsigned int i = 0; i < 2000; i++)
		if ((readl(base + 0x30) & (BIT(11) | BIT(10))) == BIT(10))
			return 0;
	return -ETIMEDOUT;
}

static int __init trial_read(void __iomem *base, unsigned int reg, u16 *value)
{
	int ret = trial_idle(base);
	unsigned int address = ADI_SLAVE + reg;

	if (ret)
		return ret;
	writel(address, base + 0x28);
	for (unsigned int i = 0; i < 2000; i++) {
		u32 raw = readl(base + 0x2c);

		if (raw & BIT(31))
			continue;
		/* Orca R3 echoes the physical command address bits 16:2. */
		if (((raw >> 16) & 0x7fff) != ((address & 0x1fffc) >> 2))
			return -EIO;
		*value = raw;
		return 0;
	}
	return -ETIMEDOUT;
}

static int __init trial_write(void __iomem *base, unsigned int reg, u16 value)
{
	int ret = trial_idle(base);

	if (ret)
		return ret;
	writel(value, base + ADI_SLAVE - ADI_BASE + reg);
	return trial_idle(base);
}

static int __init trial_stage(void __iomem *base, u16 target)
{
	u64 start;
	u16 value;
	int ret;

	ret = trial_write(base, 0x238, 0x80);
	if (ret)
		return ret;
	ret = trial_write(base, 0x254, target);
	if (ret)
		return ret;
	start = ktime_get_ns();
	for (unsigned int i = 0; i < 50000; i++) {
		ret = trial_read(base, 0x234, &value);
		if (ret)
			return ret;
		if (value & 0x80) {
			ret = trial_read(base, 0x250, &value);
			if (ret)
				return ret;
			if (value != target)
				return -EIO;
			return trial_write(base, 0x238, 0x80);
		}
		if (ktime_get_ns() - start >= 200 * NSEC_PER_MSEC)
			break;
	}
	return -ETIMEDOUT;
}

static int __init trial_checkpoint(unsigned int from, unsigned int to)
{
	void __iomem *base;
	u16 stage;
	int ret;

	if (!of_property_read_bool(of_chosen, "udx,trial-rtc-stages"))
		return 0;
	base = ioremap(ADI_BASE, 0x23000);
	if (!base)
		return -ENOMEM;
	ret = trial_read(base, 0x250, &stage);
	if (!ret)
		ret = stage == from ? trial_stage(base, to) : -EPROTO;
	iounmap(base);
	pr_info("udx710 trial: kernel checkpoint %04x -> %04x result=%d\n",
		from, to, ret);
	return ret;
}
static int __init udx710_trial_early(void)
{
	return trial_checkpoint(UDX_RTC_IMAGE_ENTRY, UDX_RTC_KERNEL_EARLY);
}
early_initcall(udx710_trial_early);
/* Runs after SMP and driver_init, before the ADI device_initcall. */
static int __init udx710_trial_core(void)
{
	return trial_checkpoint(UDX_RTC_KERNEL_EARLY, UDX_RTC_DRIVER_FRAMEWORK);
}
core_initcall(udx710_trial_core);
