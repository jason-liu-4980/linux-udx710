// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019 Spreadtrum Communications Inc.

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SPRD_EFUSE_ENABLE		0x20
#define SPRD_EFUSE_ERR_FLAG		0x24
#define SPRD_EFUSE_ERR_CLR		0x28
#define SPRD_EFUSE_MAGIC_NUM		0x2c
#define SPRD_EFUSE_FW_CFG		0x50
#define SPRD_EFUSE_PW_SWT		0x54
#define SPRD_EFUSE_MEM(val)		(0x1000 + ((val) << 2))

#define SPRD_EFUSE_VDD_EN		BIT(0)
#define SPRD_EFUSE_AUTO_CHECK_EN	BIT(1)
#define SPRD_EFUSE_DOUBLE_EN		BIT(2)
#define SPRD_EFUSE_MARGIN_RD_EN		BIT(3)
#define SPRD_EFUSE_LOCK_WR_EN		BIT(4)

#define SPRD_EFUSE_ERR_CLR_MASK		GENMASK(13, 0)

#define SPRD_EFUSE_ENK1_ON		BIT(0)
#define SPRD_EFUSE_ENK2_ON		BIT(1)
#define SPRD_EFUSE_PROG_EN		BIT(2)

#define SPRD_EFUSE_MAGIC_NUMBER		0x8810

/* Block width (bytes) definitions */
#define SPRD_EFUSE_BLOCK_WIDTH		4

/*
 * The Spreadtrum AP efuse contains 2 parts: normal efuse and secure efuse,
 * and we can only access the normal efuse in kernel. So define the normal
 * block offset index and normal block numbers.
 */
#define SPRD_EFUSE_NORMAL_BLOCK_NUMS	24
#define SPRD_EFUSE_NORMAL_BLOCK_OFFSET	72

/* Timeout (ms) for the trylock of hardware spinlocks */
#define SPRD_EFUSE_HWLOCK_TIMEOUT	5000

/*
 * Since different Spreadtrum SoC chip can have different normal block numbers
 * and offset. And some SoC can support block double feature, which means
 * when reading or writing data to efuse memory, the controller can save double
 * data in case one data become incorrect after a long period.
 *
 * Thus we should save them in the device data structure.
 */
struct sprd_efuse_variant_data {
	u32 blk_nums;
	u32 blk_offset;
	bool blk_double;
	bool read_only;
};

struct sprd_efuse {
	struct device *dev;
	struct clk *clk;
	struct hwspinlock *hwlock;
	struct mutex mutex;
	void __iomem *base;
	const struct sprd_efuse_variant_data *data;
};

static const struct sprd_efuse_variant_data ums312_data = {
	.blk_nums = SPRD_EFUSE_NORMAL_BLOCK_NUMS,
	.blk_offset = SPRD_EFUSE_NORMAL_BLOCK_OFFSET,
	.blk_double = false,
};

/* The Orca public aperture excludes secure blocks and is never programmable. */
static const struct sprd_efuse_variant_data orca_data = {
	.blk_nums = 45,
	.blk_offset = 2,
	.blk_double = true,
	.read_only = true,
};

/*
 * On Spreadtrum platform, we have multi-subsystems will access the unique
 * efuse controller, so we need one hardware spinlock to synchronize between
 * the multiple subsystems.
 */
static int sprd_efuse_lock(struct sprd_efuse *efuse)
{
	int ret;

	mutex_lock(&efuse->mutex);

	ret = hwspin_lock_timeout_raw(efuse->hwlock,
				      SPRD_EFUSE_HWLOCK_TIMEOUT);
	if (ret) {
		dev_err(efuse->dev, "timeout get the hwspinlock\n");
		mutex_unlock(&efuse->mutex);
		return ret;
	}

	return 0;
}

static void sprd_efuse_unlock(struct sprd_efuse *efuse)
{
	hwspin_unlock_raw(efuse->hwlock);
	mutex_unlock(&efuse->mutex);
}

static void sprd_efuse_set_prog_power(struct sprd_efuse *efuse, bool en)
{
	u32 val = readl(efuse->base + SPRD_EFUSE_PW_SWT);

	if (en)
		val &= ~SPRD_EFUSE_ENK2_ON;
	else
		val &= ~SPRD_EFUSE_ENK1_ON;

	writel(val, efuse->base + SPRD_EFUSE_PW_SWT);

	/* Open or close efuse power need wait 1000us to make power stable. */
	usleep_range(1000, 1200);

	if (en)
		val |= SPRD_EFUSE_ENK1_ON;
	else
		val |= SPRD_EFUSE_ENK2_ON;

	writel(val, efuse->base + SPRD_EFUSE_PW_SWT);

	/* Open or close efuse power need wait 1000us to make power stable. */
	usleep_range(1000, 1200);
}

static void sprd_efuse_set_read_power(struct sprd_efuse *efuse, bool en)
{
	u32 val = readl(efuse->base + SPRD_EFUSE_ENABLE);

	if (en)
		val |= SPRD_EFUSE_VDD_EN;
	else
		val &= ~SPRD_EFUSE_VDD_EN;

	writel(val, efuse->base + SPRD_EFUSE_ENABLE);

	/* Open or close efuse power need wait 1000us to make power stable. */
	usleep_range(1000, 1200);
}

static void sprd_efuse_set_prog_lock(struct sprd_efuse *efuse, bool en)
{
	u32 val = readl(efuse->base + SPRD_EFUSE_ENABLE);

	if (en)
		val |= SPRD_EFUSE_LOCK_WR_EN;
	else
		val &= ~SPRD_EFUSE_LOCK_WR_EN;

	writel(val, efuse->base + SPRD_EFUSE_ENABLE);
}

static void sprd_efuse_set_auto_check(struct sprd_efuse *efuse, bool en)
{
	u32 val = readl(efuse->base + SPRD_EFUSE_ENABLE);

	if (en)
		val |= SPRD_EFUSE_AUTO_CHECK_EN;
	else
		val &= ~SPRD_EFUSE_AUTO_CHECK_EN;

	writel(val, efuse->base + SPRD_EFUSE_ENABLE);
}

static void sprd_efuse_set_data_double(struct sprd_efuse *efuse, bool en)
{
	u32 val = readl(efuse->base + SPRD_EFUSE_ENABLE);

	if (en)
		val |= SPRD_EFUSE_DOUBLE_EN;
	else
		val &= ~SPRD_EFUSE_DOUBLE_EN;

	writel(val, efuse->base + SPRD_EFUSE_ENABLE);
}

static void sprd_efuse_set_prog_en(struct sprd_efuse *efuse, bool en)
{
	u32 val = readl(efuse->base + SPRD_EFUSE_PW_SWT);

	if (en)
		val |= SPRD_EFUSE_PROG_EN;
	else
		val &= ~SPRD_EFUSE_PROG_EN;

	writel(val, efuse->base + SPRD_EFUSE_PW_SWT);
}

static int sprd_efuse_raw_prog(struct sprd_efuse *efuse, u32 blk, bool doub,
			       bool lock, u32 *data)
{
	u32 status;
	int ret = 0;

	/*
	 * We need set the correct magic number before writing the efuse to
	 * allow programming, and block other programming until we clear the
	 * magic number.
	 */
	writel(SPRD_EFUSE_MAGIC_NUMBER,
	       efuse->base + SPRD_EFUSE_MAGIC_NUM);

	/*
	 * Power on the efuse, enable programme and enable double data
	 * if asked.
	 */
	sprd_efuse_set_prog_power(efuse, true);
	sprd_efuse_set_prog_en(efuse, true);
	sprd_efuse_set_data_double(efuse, doub);

	/*
	 * Enable the auto-check function to validate if the programming is
	 * successful.
	 */
	if (lock)
		sprd_efuse_set_auto_check(efuse, true);

	writel(*data, efuse->base + SPRD_EFUSE_MEM(blk));

	/* Disable auto-check and data double after programming */
	if (lock)
		sprd_efuse_set_auto_check(efuse, false);
	sprd_efuse_set_data_double(efuse, false);

	/*
	 * Check the efuse error status, if the programming is successful,
	 * we should lock this efuse block to avoid programming again.
	 */
	status = readl(efuse->base + SPRD_EFUSE_ERR_FLAG);
	if (status) {
		dev_err(efuse->dev,
			"write error status %u of block %d\n", status, blk);

		writel(SPRD_EFUSE_ERR_CLR_MASK,
		       efuse->base + SPRD_EFUSE_ERR_CLR);
		ret = -EBUSY;
	} else if (lock) {
		sprd_efuse_set_prog_lock(efuse, lock);
		writel(0, efuse->base + SPRD_EFUSE_MEM(blk));
		sprd_efuse_set_prog_lock(efuse, false);
	}

	sprd_efuse_set_prog_power(efuse, false);
	writel(0, efuse->base + SPRD_EFUSE_MAGIC_NUM);

	return ret;
}

static int sprd_efuse_raw_read(struct sprd_efuse *efuse, int blk, u32 *val,
			       bool doub)
{
	u32 status;

	/*
	 * Need power on the efuse before reading data from efuse, and will
	 * power off the efuse after reading process.
	 */
	sprd_efuse_set_read_power(efuse, true);

	/* Enable double data if asked */
	sprd_efuse_set_data_double(efuse, doub);

	/* Start to read data from efuse block */
	*val = readl(efuse->base + SPRD_EFUSE_MEM(blk));

	/* Disable double data */
	sprd_efuse_set_data_double(efuse, false);

	/* Power off the efuse */
	sprd_efuse_set_read_power(efuse, false);

	/*
	 * Check the efuse error status and clear them if there are some
	 * errors occurred.
	 */
	status = readl(efuse->base + SPRD_EFUSE_ERR_FLAG);
	/* The Orca read protocol clears status after every transaction. */
	if (efuse->data->read_only)
		writel(SPRD_EFUSE_ERR_CLR_MASK,
		       efuse->base + SPRD_EFUSE_ERR_CLR);
	if (status) {
		dev_err_ratelimited(efuse->dev,
				    "eFuse controller read error\n");

		writel(SPRD_EFUSE_ERR_CLR_MASK,
		       efuse->base + SPRD_EFUSE_ERR_CLR);
		return -EBUSY;
	}

	return 0;
}

static int sprd_efuse_read(void *context, u32 offset, void *val, size_t bytes)
{
	struct sprd_efuse *efuse = context;
	u32 size = efuse->data->blk_nums * SPRD_EFUSE_BLOCK_WIDTH;
	u8 *out = val;
	int ret;

	if (offset > size || bytes > size - offset)
		return -EINVAL;
	if (!bytes)
		return 0;
	ret = sprd_efuse_lock(efuse);
	if (ret)
		return ret;
	ret = clk_prepare_enable(efuse->clk);
	if (ret)
		goto unlock;

	/* NVMEM permits unaligned reads spanning more than one physical block. */
	while (bytes) {
		u32 skip = offset % SPRD_EFUSE_BLOCK_WIDTH;
		u32 index = offset / SPRD_EFUSE_BLOCK_WIDTH +
			    efuse->data->blk_offset;
		size_t count =
			min_t(size_t, bytes, SPRD_EFUSE_BLOCK_WIDTH - skip);
		u32 data;
		size_t i;

		ret = sprd_efuse_raw_read(efuse, index, &data,
					  efuse->data->blk_double);
		if (ret)
			break;
		for (i = 0; i < count; i++)
			out[i] = data >> ((skip + i) * BITS_PER_BYTE);
		out += count;
		offset += count;
		bytes -= count;
	}
	clk_disable_unprepare(efuse->clk);
unlock:
	sprd_efuse_unlock(efuse);
	return ret;
}

/* The board's original READ_UID handler reads physical blocks 0 and 1 in
 * double mode, then clears bit 31 in each word. These two UID blocks precede
 * the existing Orca calibration aperture; keep its offsets unchanged.
 */
static int sprd_efuse_read_uid(struct sprd_efuse *efuse, u8 uid[8])
{
	u32 words[2];
	int ret, i;

	ret = sprd_efuse_lock(efuse);
	if (ret)
		return ret;
	ret = clk_prepare_enable(efuse->clk);
	if (ret)
		goto unlock;

	writel(SPRD_EFUSE_ERR_CLR_MASK, efuse->base + SPRD_EFUSE_ERR_CLR);
	for (i = 0; i < 2; i++) {
		ret = sprd_efuse_raw_read(efuse, i, &words[i], true);
		if (ret)
			goto clock_off;
		words[i] &= 0x7fffffff;
	}
	for (i = 0; i < 8; i++)
		uid[i] = words[i / 4] >> ((i % 4) * 8);

clock_off:
	clk_disable_unprepare(efuse->clk);
unlock:
	sprd_efuse_unlock(efuse);
	return ret;
}

/* Keep calibration offsets at hardware block 2 and append the preceding
 * two UID blocks to the read-only logical view. No calibration cell moves.
 */
static int sprd_orca_read(void *context, u32 offset, void *val, size_t bytes)
{
	struct sprd_efuse *efuse = context;
	u32 public_size = efuse->data->blk_nums * SPRD_EFUSE_BLOCK_WIDTH;
	u8 uid[8], *out = val;
	size_t count;
	int ret;

	if (offset > public_size + sizeof(uid) ||
	    bytes > public_size + sizeof(uid) - offset)
		return -EINVAL;
	if (offset < public_size && bytes) {
		count = min_t(size_t, bytes, public_size - offset);
		ret = sprd_efuse_read(efuse, offset, out, count);
		if (ret)
			return ret;
		offset += count;
		out += count;
		bytes -= count;
	}
	if (!bytes)
		return 0;
	ret = sprd_efuse_read_uid(efuse, uid);
	if (ret)
		return ret;
	memcpy(out, uid + offset - public_size, bytes);
	return 0;
}

static int sprd_efuse_write(void *context, u32 offset, void *val, size_t bytes)
{
	struct sprd_efuse *efuse = context;
	bool blk_double = efuse->data->blk_double;
	bool lock;
	int ret;

	if (efuse->data->read_only)
		return -EROFS;

	ret = sprd_efuse_lock(efuse);
	if (ret)
		return ret;

	ret = clk_prepare_enable(efuse->clk);
	if (ret)
		goto unlock;

	/*
	 * If the writing bytes are equal with the block width, which means the
	 * whole block will be programmed. For this case, we should not allow
	 * this block to be programmed again by locking this block.
	 *
	 * If the block was programmed partially, we should allow this block to
	 * be programmed again.
	 */
	if (bytes < SPRD_EFUSE_BLOCK_WIDTH)
		lock = false;
	else
		lock = true;

	ret = sprd_efuse_raw_prog(efuse, offset, blk_double, lock, val);

	clk_disable_unprepare(efuse->clk);

unlock:
	sprd_efuse_unlock(efuse);
	return ret;
}

static int sprd_efuse_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct nvmem_device *nvmem;
	struct nvmem_config econfig = { };
	struct sprd_efuse *efuse;
	const struct sprd_efuse_variant_data *pdata;
	struct resource *res;
	int ret;

	pdata = of_device_get_match_data(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "No matching driver data found\n");
		return -EINVAL;
	}

	efuse = devm_kzalloc(&pdev->dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < SPRD_EFUSE_MEM(pdata->blk_offset +
							pdata->blk_nums - 1) +
						 sizeof(u32))
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "eFuse register window too small\n");
	efuse->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(efuse->base))
		return PTR_ERR(efuse->base);

	ret = of_hwspin_lock_get_id(np, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get hwlock id\n");
		return ret;
	}

	efuse->hwlock = devm_hwspin_lock_request_specific(&pdev->dev, ret);
	if (!efuse->hwlock) {
		dev_err(&pdev->dev, "failed to request hwlock\n");
		return -ENXIO;
	}

	efuse->clk = devm_clk_get(&pdev->dev, "enable");
	if (IS_ERR(efuse->clk)) {
		dev_err(&pdev->dev, "failed to get enable clock\n");
		return PTR_ERR(efuse->clk);
	}

	mutex_init(&efuse->mutex);
	efuse->dev = &pdev->dev;
	efuse->data = pdata;

	econfig.stride = 1;
	econfig.word_size = 1;
	econfig.read_only = pdata->read_only;
	econfig.root_only = pdata->read_only;
	econfig.type = NVMEM_TYPE_OTP;
	econfig.owner = THIS_MODULE;
	econfig.name = pdata->read_only ? "orca-public-efuse" : "sprd-efuse";
	econfig.size = efuse->data->blk_nums * SPRD_EFUSE_BLOCK_WIDTH;
	econfig.add_legacy_fixed_of_cells = pdata != &orca_data;
	econfig.reg_read = pdata == &orca_data ? sprd_orca_read : sprd_efuse_read;
	if (pdata == &orca_data)
		econfig.size += 8;
	if (!pdata->read_only)
		econfig.reg_write = sprd_efuse_write;
	econfig.priv = efuse;
	econfig.dev = &pdev->dev;
	nvmem = devm_nvmem_register(&pdev->dev, &econfig);
	if (IS_ERR(nvmem)) {
		dev_err(&pdev->dev, "failed to register nvmem\n");
		return PTR_ERR(nvmem);
	}
	platform_set_drvdata(pdev, efuse);

	return 0;
}

static const struct of_device_id sprd_efuse_of_match[] = {
	{ .compatible = "sprd,ums312-efuse", .data = &ums312_data },
	{ .compatible = "sprd,orca-efuse", .data = &orca_data },
	{}
};
MODULE_DEVICE_TABLE(of, sprd_efuse_of_match);

static struct platform_driver sprd_efuse_driver = {
	.probe = sprd_efuse_probe,
	.driver = {
		.name = "sprd-efuse",
		.of_match_table = sprd_efuse_of_match,
	},
};

module_platform_driver(sprd_efuse_driver);

MODULE_AUTHOR("Freeman Liu <freeman.liu@spreadtrum.com>");
MODULE_DESCRIPTION("Spreadtrum AP efuse driver");
MODULE_LICENSE("GPL");
