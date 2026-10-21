// SPDX-License-Identifier: GPL-2.0-only
/* UDX710 modem subsystem: load every raw image before releasing any core. */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/unaligned.h>

#define MODEM_CORES 3
#define MODEM_IMAGES 9
#define IRAM_WORDS 49

struct modem_core {
	u32 shutdown, sleep, reset, reset_mask, sys_mask, extra;
	bool reset_rmw;
};

static const struct modem_core modem_cores[MODEM_CORES] = {
	{ 0x58, BIT(3), 0x214, 0x3000, BIT(1), 0, false },
	{ 0x54, BIT(2), 0x220, 0x7, BIT(0), 0x48, true },
	{ 0x18, BIT(0), 0x218, 0x7e00, BIT(17), 0, false },
};

static const char * const iram_names[] = { "cp-iram", "v3-iram", "nr-iram" };
static const char * const iram_properties[] = {
	"sprd,cp-iram-data", "sprd,v3-iram-data", "sprd,nr-iram-data",
};
static const char * const image_names[MODEM_IMAGES] = {
	"modem", "v3phy", "nrphy", "nrdsp1", "nrdsp2",
	"deltanv", "cpcmdline", "fixnv", "runnv",
};

struct udx710_modem {
	struct regmap *aon, *pmu;
	void __iomem *iram[MODEM_CORES];
	u32 iram_data[MODEM_CORES][IRAM_WORDS];
	struct resource image[MODEM_IMAGES];
	const char *firmware[MODEM_IMAGES];
};

static int modem_core_reset(struct udx710_modem *m, unsigned int i, bool hold)
{
	const struct modem_core *c = &modem_cores[i];

	if (c->reset_rmw)
		return regmap_update_bits(m->aon, c->reset, c->reset_mask,
					  hold ? c->reset_mask : 0);
	return regmap_write(m->aon, c->reset + (hold ? 0x1000 : 0x2000),
			    c->reset_mask);
}

static int modem_hold(struct rproc *rproc)
{
	struct udx710_modem *m = rproc->priv;
	unsigned int i;
	int ret;

	for (i = 0; i < MODEM_CORES; i++) {
		const struct modem_core *c = &modem_cores[i];

		ret = modem_core_reset(m, i, true);
		if (ret)
			return ret;
		msleep(50);
		ret = regmap_write(m->pmu, 0x10b0, c->sys_mask);
		if (ret)
			return ret;
		msleep(50);
		ret = regmap_write(m->pmu, 0x1544, c->sleep);
		if (ret)
			return ret;
		msleep(50);
		if (c->extra) {
			ret = regmap_write(m->pmu, c->extra + 0x1000, BIT(25));
			if (ret)
				return ret;
		}
		ret = regmap_write(m->pmu, c->shutdown + 0x1000, BIT(25));
		if (ret)
			return ret;
	}
	return 0;
}

static int modem_check_image(struct udx710_modem *m, unsigned int i,
			     const struct firmware *fw)
{
	u32 entry;

	if (!fw->size || fw->size > resource_size(&m->image[i]))
		return -EFBIG;
	/* FIXNV and RUNNV are complete raw payloads, including unused bytes. */
	if (i >= 7 && fw->size != resource_size(&m->image[i]))
		return -ENOEXEC;
	if (i >= MODEM_CORES)
		return 0;
	/* Original MECP header and ARM reset vectors, after SCI extraction. */
	if (fw->size < 0x60c || memcmp(fw->data, "MECPV1.0", 8) ||
	    get_unaligned_le32(fw->data + 8) != 0x600 ||
	    get_unaligned_le32(fw->data + 0x600) != 0xe59ff000 ||
	    get_unaligned_le32(fw->data + 0x604) != 0xe59ff000)
		return -ENOEXEC;
	entry = get_unaligned_le32(fw->data + 0x608);
	if (entry < m->image[i].start ||
	    entry - m->image[i].start >= fw->size || (entry & 3))
		return -ENOEXEC;
	return 0;
}

static int modem_sanity(struct rproc *rproc, const struct firmware *fw)
{
	return modem_check_image(rproc->priv, 0, fw);
}

static int modem_load(struct rproc *rproc, const struct firmware *primary)
{
	struct udx710_modem *m = rproc->priv;
	size_t lengths[MODEM_IMAGES] = {};
	unsigned int i, j;
	int ret;

	for (i = 0; i < MODEM_IMAGES; i++) {
		const struct firmware *fw = primary;
		void __iomem *dst;

		/* remoteproc already checked primary through .sanity_check. */
		if (i) {
			ret = request_firmware(&fw, m->firmware[i], rproc->dev.parent);
			if (ret)
				return ret;
			ret = modem_check_image(m, i, fw);
			if (ret)
				goto release;
		}
		/* NRPHY's capacity overlaps its DSP windows. Actual extents may not. */
		for (j = 0; j < i; j++)
			if (m->image[i].start < m->image[j].start + lengths[j] &&
			    m->image[j].start < m->image[i].start + fw->size) {
				ret = -EINVAL;
				goto release;
			}
		lengths[i] = fw->size;
		dst = ioremap_np(m->image[i].start, fw->size);
		if (!dst) {
			ret = -ENOMEM;
			goto release;
		}
		memcpy_toio(dst, fw->data, fw->size);
		wmb();
		iounmap(dst);
		ret = 0;
release:
		if (i)
			release_firmware(fw);
		if (ret)
			return ret;
	}
	return 0;
}

static int modem_start_core(struct udx710_modem *m, unsigned int i)
{
	const struct modem_core *c = &modem_cores[i];
	unsigned int value, j;
	int ret;

	/* Preserve byte accesses used by the original IRAM loader. */
	for (j = 0; j < IRAM_WORDS * sizeof(u32); j++)
		writeb((m->iram_data[i][j / 4] >> (8 * (j % 4))) & 0xff,
		       m->iram[i] + j);
	wmb();
	ret = regmap_write(m->pmu, c->shutdown + 0x2000, BIT(25));
	if (ret)
		return ret;
	if (c->extra) {
		ret = regmap_write(m->pmu, c->extra + 0x2000, BIT(25));
		if (ret)
			return ret;
	}
	ret = regmap_write(m->pmu, 0x2544, c->sleep);
	if (ret)
		return ret;
	msleep(50);
	ret = modem_core_reset(m, i, false);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(m->aon, c->reset, value,
				       !(value & c->reset_mask), 10, 1000000);
	if (ret)
		return ret;
	msleep(50);
	ret = regmap_write(m->pmu, 0x20b0, c->sys_mask);
	if (ret)
		return ret;
	return regmap_read_poll_timeout(m->pmu, 0xb0, value,
					!(value & c->sys_mask), 10, 1000000);
}

static int modem_start(struct rproc *rproc)
{
	struct udx710_modem *m = rproc->priv;
	unsigned int i;
	int ret;

	for (i = 0; i < MODEM_CORES; i++) {
		ret = modem_start_core(m, i);
		if (ret) {
			/* A partial release is not a running modem subsystem. */
			int hold_ret = modem_hold(rproc);

			if (hold_ret)
				dev_err(&rproc->dev, "reset after start failure: %d\n",
					hold_ret);
			return ret;
		}
	}
	return 0;
}

static int modem_stop(struct rproc *rproc)
{
	/* SIPC/SIPA do not yet have a complete live shutdown contract. */
	return -EOPNOTSUPP;
}

static const struct rproc_ops modem_ops = {
	.sanity_check = modem_sanity,
	.load = modem_load,
	.start = modem_start,
	.stop = modem_stop,
};

static int udx710_modem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct reserved_mem *reserved;
	struct udx710_modem *m;
	struct rproc *rproc;
	const char *primary;
	unsigned int i;
	int ret;

	ret = of_property_read_string_index(dev->of_node, "firmware-name", 0, &primary);
	if (ret)
		return ret;
	rproc = devm_rproc_alloc(dev, "udx710-modem", &modem_ops, primary, sizeof(*m));
	if (!rproc)
		return -ENOMEM;
	m = rproc->priv;
	m->aon = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,aon-apb");
	if (IS_ERR(m->aon))
		return PTR_ERR(m->aon);
	m->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,pmu-apb");
	if (IS_ERR(m->pmu))
		return PTR_ERR(m->pmu);
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return -EINVAL;
	reserved = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!reserved)
		return -EINVAL;
	for (i = 0; i < MODEM_CORES; i++) {
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
								   iram_names[i]);

		if (!res || resource_size(res) < IRAM_WORDS * sizeof(u32) ||
		    of_property_count_u32_elems(dev->of_node, iram_properties[i]) != IRAM_WORDS)
			return -EINVAL;
		m->iram[i] = devm_platform_ioremap_resource_byname(pdev, iram_names[i]);
		if (IS_ERR(m->iram[i]))
			return PTR_ERR(m->iram[i]);
		ret = of_property_read_u32_array(dev->of_node, iram_properties[i],
						m->iram_data[i], IRAM_WORDS);
		if (ret)
			return ret;
	}
	for (i = 0; i < MODEM_IMAGES; i++) {
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
								   image_names[i]);

		if (!res || res->start < reserved->base ||
		    res->end >= reserved->base + reserved->size)
			return -EINVAL;
		m->image[i] = *res;
		ret = of_property_read_string_index(dev->of_node, "firmware-name", i,
						    &m->firmware[i]);
		if (ret)
			return ret;
	}
	ret = modem_hold(rproc);
	if (ret)
		return ret;
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	platform_set_drvdata(pdev, rproc);
	return devm_rproc_add(dev, rproc);
}

static const struct of_device_id udx710_modem_match[] = {
	{ .compatible = "sprd,udx710-modem" },
	{}
};
MODULE_DEVICE_TABLE(of, udx710_modem_match);

static struct platform_driver udx710_modem_driver = {
	.probe = udx710_modem_probe,
	.driver = {
		.name = "udx710-modem",
		.of_match_table = udx710_modem_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(udx710_modem_driver);
MODULE_DESCRIPTION("UDX710 modem subsystem remote processor");
MODULE_LICENSE("GPL");
