// SPDX-License-Identifier: GPL-2.0-only
/* UDX710 CM4 startup through its SRAM firmware window and reset controls.
 */
#include <linux/delay.h>
#include <linux/mtd/ubi.h>
#include <linux/notifier.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/sprd_pmsys.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define PMSYS_SRAM_BASE 0xb000
#define PMSYS_SRAM_SIZE 0xdc00
#define PMSYS_IMAGE_SIZE 0xd000
#define PMSYS_RESET 0x210
#define PMSYS_CORE_RESET BIT(0)
#define PMSYS_SYS_RESET BIT(4)

struct sprd_pmsys {
	struct device *dev;
	struct regmap *aon;
	void __iomem *sram;
	struct mutex lock;
	bool released;
};

/* Volume notifications run under UBI locks. Only schedule work here; neither
 * opening a volume nor probing a platform device is allowed in the callback.
 */
static void pmsys_reprobe(struct work_struct *work)
{
	struct device_node *np;
	struct platform_device *pdev;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "sprd,udx710-pmsys");
	if (!np)
		return;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return;
	ret = device_attach(&pdev->dev);
	if (ret < 0)
		dev_dbg(&pdev->dev, "PM_SYS deferred attach: %d\n", ret);
	put_device(&pdev->dev);
}
static DECLARE_WORK(pmsys_reprobe_work, pmsys_reprobe);

static int pmsys_ubi_event(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct ubi_notification *notice = data;

	if (event == UBI_VOLUME_ADDED && notice->di.ubi_num == 0 &&
	    !strcmp(notice->vi.name, "nr_pmsys"))
		schedule_work(&pmsys_reprobe_work);
	return NOTIFY_OK;
}

static struct notifier_block pmsys_ubi_notifier = {
	.notifier_call = pmsys_ubi_event,
};

static int pmsys_read_original(struct device *dev, u8 **image)
{
	struct ubi_volume_desc *desc;
	struct ubi_volume_info volume;
	const char *name;
	u8 *data;
	int ret;

	ret = of_property_read_string(dev->of_node, "sprd,firmware-volume",
				      &name);
	if (ret || strcmp(name, "nr_pmsys"))
		return -EINVAL;
	desc = ubi_open_volume_nm(0, name, UBI_READONLY);
	if (IS_ERR(desc)) {
		ret = PTR_ERR(desc);
		if (ret == -ENODEV || ret == -ENOENT)
			ret = -EPROBE_DEFER;
		return ret;
	}
	ubi_get_volume_info(desc, &volume);
	ret = -EINVAL;
	if (volume.corrupted || volume.upd_marker ||
	    volume.used_bytes < PMSYS_IMAGE_SIZE ||
	    volume.usable_leb_size < PMSYS_IMAGE_SIZE)
		goto close;
	data = kmalloc(PMSYS_IMAGE_SIZE, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto close;
	}
	ret = ubi_leb_read(desc, 0, data, 0, PMSYS_IMAGE_SIZE, 1);
	if (ret)
		goto free;
	/* Original CM4 vector table: aligned stack inside its SRAM and a Thumb
	 * reset entry inside the image.
	 */
	if (get_unaligned_le32(data) <= PMSYS_IMAGE_SIZE ||
	    get_unaligned_le32(data) > PMSYS_SRAM_SIZE ||
	    get_unaligned_le32(data) & 3 ||
	    !(get_unaligned_le32(data + 4) & 1) ||
	    (get_unaligned_le32(data + 4) & ~1U) >= PMSYS_IMAGE_SIZE) {
		ret = -ENOEXEC;
		goto free;
	}
	*image = data;
	ubi_close_volume(desc);
	return 0;
free:
	kfree(data);
close:
	ubi_close_volume(desc);
	return ret;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct sprd_pmsys *pm = dev_get_drvdata(dev);
	unsigned int reset;
	int ret = regmap_read(pm->aon, PMSYS_RESET, &reset);

	if (ret)
		return ret;
	return sysfs_emit(buf, "%s core_reset=%u system_reset=%u\n",
			  READ_ONCE(pm->released) ? "released" : "prepared",
			  !!(reset & PMSYS_CORE_RESET),
			  !!(reset & PMSYS_SYS_RESET));
}
static DEVICE_ATTR_RO(state);
static struct attribute *pmsys_attrs[] = { &dev_attr_state.attr, NULL };
ATTRIBUTE_GROUPS(pmsys);

static void pmsys_put_device(void *data)
{
	put_device(data);
}

struct sprd_pmsys *sprd_pmsys_get(struct device *consumer)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct sprd_pmsys *pm;
	int ret;

	np = of_parse_phandle(consumer->of_node, "sprd,pm-sys", 0);
	if (!np)
		return ERR_PTR(-ENODEV);
	if (!of_device_is_compatible(np, "sprd,udx710-pmsys")) {
		of_node_put(np);
		return ERR_PTR(-EINVAL);
	}
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	device_lock(&pdev->dev);
	pm = device_is_bound(&pdev->dev) ? dev_get_drvdata(&pdev->dev) : NULL;
	device_unlock(&pdev->dev);
	if (!pm) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	/* Both drivers are built-in and do not expose manual unbind. */
	if (!device_link_add(consumer, &pdev->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&pdev->dev);
		return ERR_PTR(-ENOMEM);
	}
	ret = devm_add_action_or_reset(consumer, pmsys_put_device, &pdev->dev);
	return ret ? ERR_PTR(ret) : pm;
}
EXPORT_SYMBOL_GPL(sprd_pmsys_get);

int sprd_pmsys_start(struct sprd_pmsys *pm)
{
	unsigned int reset;
	int ret = 0;

	mutex_lock(&pm->lock);
	if (pm->released)
		goto out;
	ret = regmap_read(pm->aon, PMSYS_RESET, &reset);
	if (ret)
		goto out;
	if ((reset & (PMSYS_CORE_RESET | PMSYS_SYS_RESET)) !=
	    PMSYS_CORE_RESET) {
		ret = -EIO;
		goto out;
	}
	/* SRAM writes completed in probe, before SIPC registered
	 * its receiver; release the CM4 after both are ready.
	 */
	/* Complete SRAM image writes before the start callback releases reset. */
	wmb();
	ret = regmap_update_bits(pm->aon, PMSYS_RESET, PMSYS_CORE_RESET, 0);
	if (ret)
		goto out;
	ret = regmap_read(pm->aon, PMSYS_RESET, &reset);
	if (!ret && (reset & PMSYS_CORE_RESET))
		ret = -EIO;
	if (!ret) {
		pm->released = true;
		dev_info(
			pm->dev,
			"CM4 released after SIPC receiver registration; reset=%08x\n",
			reset);
	}
out:
	mutex_unlock(&pm->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pmsys_start);

static int sprd_pmsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sprd_pmsys *pm;
	u8 *image;
	struct resource *res;
	unsigned int reset;
	u32 i;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != PMSYS_SRAM_BASE ||
	    resource_size(res) != PMSYS_SRAM_SIZE)
		return -EINVAL;
	pm = devm_kzalloc(dev, sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return -ENOMEM;
	pm->dev = dev;
	mutex_init(&pm->lock);
	pm->aon = syscon_regmap_lookup_by_phandle(dev->of_node, "sprd,aon-apb");
	if (IS_ERR(pm->aon))
		return PTR_ERR(pm->aon);
	pm->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm->sram))
		return PTR_ERR(pm->sram);
	ret = pmsys_read_original(dev, &image);
	if (ret)
		return dev_err_probe(
			dev, ret, "Original NAND PM_SYS image unavailable\n");

	/* The original UDX710 boot path holds CM4 core reset and releases its
	 * system reset. Re-establish that state for every kernel boot before
	 * replacing the old resume vector (0x1a9) with the cold entry (0x275).
	 */
	ret = regmap_update_bits(pm->aon, PMSYS_RESET, PMSYS_CORE_RESET,
				 PMSYS_CORE_RESET);
	if (ret)
		goto release_fw;
	udelay(2);
	ret = regmap_update_bits(pm->aon, PMSYS_RESET, PMSYS_SYS_RESET, 0);
	if (ret)
		goto release_fw;
	ret = regmap_read(pm->aon, PMSYS_RESET, &reset);
	if (ret)
		goto release_fw;
	if ((reset & (PMSYS_CORE_RESET | PMSYS_SYS_RESET)) !=
	    PMSYS_CORE_RESET) {
		ret = -EIO;
		goto release_fw;
	}
	/* AON SRAM requires individual 32-bit accesses: do not substitute a
	 * wide memcpy_toio implementation (same constraint as stock cproc).
	 */
	for (i = 0; i < PMSYS_IMAGE_SIZE; i += sizeof(u32)) {
		writel_relaxed(get_unaligned_le32(image + i), pm->sram + i);
		if (!(i & 0xfff))
			cond_resched();
	}
	/* Complete SRAM image writes before the start callback releases reset. */
	wmb();

	platform_set_drvdata(pdev, pm);
	dev_info(
		dev,
		"original NAND PM_SYS image prepared (%u bytes), core held for SIPC\n",
		PMSYS_IMAGE_SIZE);
	ret = 0;
release_fw:
	kfree(image);
	return ret;
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
		.dev_groups = pmsys_groups,
		.suppress_bind_attrs = true,
	},
};
static int __init sprd_pmsys_init(void)
{
	int ret = platform_driver_register(&sprd_pmsys_driver);

	if (ret)
		return ret;
	ret = ubi_register_volume_notifier(&pmsys_ubi_notifier, 0);
	if (ret)
		platform_driver_unregister(&sprd_pmsys_driver);
	return ret;
}
module_init(sprd_pmsys_init);

static void __exit sprd_pmsys_exit(void)
{
	ubi_unregister_volume_notifier(&pmsys_ubi_notifier);
	cancel_work_sync(&pmsys_reprobe_work);
	platform_driver_unregister(&sprd_pmsys_driver);
}
module_exit(sprd_pmsys_exit);
MODULE_DESCRIPTION("UDX710 PM_SYS CM4 image and startup provider");
MODULE_LICENSE("GPL");
