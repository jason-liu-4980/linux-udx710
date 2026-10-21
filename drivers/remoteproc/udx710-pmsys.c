// SPDX-License-Identifier: GPL-2.0-only
/* UDX710 PM_SYS CM4: original raw firmware, no resource table. */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/sched.h>
#include <linux/unaligned.h>

#define PMSYS_SRAM_BASE 0xb000
#define PMSYS_SRAM_SIZE 0xdc00
#define PMSYS_IMAGE_SIZE 0xd000
#define PMSYS_RESET 0x210
#define PMSYS_CORE_RESET BIT(0)
#define PMSYS_SYS_RESET BIT(4)

struct sprd_pmsys {
	struct regmap *aon;
	void __iomem *sram;
};

static int pmsys_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	u32 stack, entry;

	if (fw->size != PMSYS_IMAGE_SIZE)
		return -ENOEXEC;
	stack = get_unaligned_le32(fw->data);
	entry = get_unaligned_le32(fw->data + 4);
	/* Original CM4 vector table: aligned stack inside its SRAM and a
	 * Thumb reset entry inside the image.
	 */
	if (!stack || stack > PMSYS_SRAM_SIZE ||
	    stack & 3 || !(entry & 1) || (entry & ~1U) >= PMSYS_IMAGE_SIZE)
		return -ENOEXEC;
	return 0;
}

static int pmsys_hold_reset(struct sprd_pmsys *pm)
{
	unsigned int reset;
	int ret;

	/* Stop the old firmware before SIPC initializes shared rings. */
	ret = regmap_update_bits(pm->aon, PMSYS_RESET, PMSYS_CORE_RESET,
				 PMSYS_CORE_RESET);
	if (ret)
		return ret;
	udelay(2);
	ret = regmap_update_bits(pm->aon, PMSYS_RESET, PMSYS_SYS_RESET, 0);
	if (ret)
		return ret;
	ret = regmap_read(pm->aon, PMSYS_RESET, &reset);
	if (ret)
		return ret;
	return (reset & (PMSYS_CORE_RESET | PMSYS_SYS_RESET)) ==
		PMSYS_CORE_RESET ? 0 : -EIO;
}

static int pmsys_load(struct rproc *rproc, const struct firmware *fw)
{
	struct sprd_pmsys *pm = rproc->priv;
	u32 i;

	/* AON SRAM requires individual 32-bit accesses, not wide memcpy_toio. */
	for (i = 0; i < PMSYS_IMAGE_SIZE; i += sizeof(u32)) {
		writel_relaxed(get_unaligned_le32(fw->data + i), pm->sram + i);
		if (!(i & 0xfff))
			cond_resched();
	}
	wmb();
	return 0;
}

static int pmsys_start(struct rproc *rproc)
{
	struct sprd_pmsys *pm = rproc->priv;
	unsigned int reset;
	int ret;

	ret = regmap_update_bits(pm->aon, PMSYS_RESET, PMSYS_CORE_RESET, 0);
	if (ret)
		return ret;
	ret = regmap_read(pm->aon, PMSYS_RESET, &reset);
	if (ret)
		return ret;
	if (reset & (PMSYS_CORE_RESET | PMSYS_SYS_RESET))
		return -EIO;
	dev_info(&rproc->dev, "CM4 released after SIPC receiver registration\n");
	return 0;
}

static int pmsys_stop(struct rproc *rproc)
{
	/* UART and DDR depend on the running firmware for the entire boot. */
	return -EOPNOTSUPP;
}

static const struct rproc_ops pmsys_ops = {
	.sanity_check = pmsys_sanity_check,
	.load = pmsys_load,
	.start = pmsys_start,
	.stop = pmsys_stop,
};

static int sprd_pmsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sprd_pmsys *pm;
	struct resource *res;
	struct rproc *rproc;
	const char *firmware;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != PMSYS_SRAM_BASE ||
	    resource_size(res) != PMSYS_SRAM_SIZE)
		return -EINVAL;
	ret = of_property_read_string(dev->of_node, "firmware-name", &firmware);
	if (ret)
		return ret;
	rproc = devm_rproc_alloc(dev, "udx710-pmsys", &pmsys_ops, firmware,
				sizeof(*pm));
	if (!rproc)
		return -ENOMEM;
	pm = rproc->priv;
	pm->aon = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,aon-apb");
	if (IS_ERR(pm->aon))
		return PTR_ERR(pm->aon);
	pm->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm->sram))
		return PTR_ERR(pm->sram);
	ret = pmsys_hold_reset(pm);
	if (ret)
		return ret;
	/* Only SIPC may start PM_SYS, after publishing its receive endpoint. */
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	rproc->sysfs_read_only = true;
	platform_set_drvdata(pdev, rproc);
	return devm_rproc_add(dev, rproc);
}

static const struct of_device_id sprd_pmsys_of_match[] = {
	{ .compatible = "sprd,udx710-pmsys" },
	{}
};
MODULE_DEVICE_TABLE(of, sprd_pmsys_of_match);

static struct platform_driver sprd_pmsys_driver = {
	.probe = sprd_pmsys_probe,
	.driver = {
		.name = "udx710-pmsys",
		.of_match_table = sprd_pmsys_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sprd_pmsys_driver);
MODULE_DESCRIPTION("UDX710 PM_SYS CM4 remote processor");
MODULE_LICENSE("GPL");
