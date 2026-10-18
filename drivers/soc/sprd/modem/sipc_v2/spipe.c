// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sipc.h>

#include "sipc_priv.h"
#include "spipe.h"

struct spipe_device {
	struct spipe_init_data *init;
	int major;
	int minor;
	struct cdev cdev;
};

struct spipe_sbuf {
	u8 dst;
	u8 channel;
	u32 bufid;
};

static struct class *spipe_class;

static ssize_t session_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct spipe_device *spipe = dev_get_drvdata(dev);
	u32 generation, state;
	int ret;

	ret = sbuf_get_session(spipe->init->dst, spipe->init->channel,
			       &generation, &state);
	if (ret)
		return ret;
	return sysfs_emit(buf, "1 %u %u %u %u\n", spipe->init->dst,
			  spipe->init->channel, generation, state);
}
static DEVICE_ATTR_RO(session);

static struct attribute *spipe_session_attrs[] = {
	&dev_attr_session.attr,
	NULL,
};
ATTRIBUTE_GROUPS(spipe_session);

static int spipe_open(struct inode *inode, struct file *filp)
{
	int minor = iminor(filp->f_path.dentry->d_inode);
	struct spipe_device *spipe;
	struct spipe_sbuf *sbuf;
	int ret;

	spipe = container_of(inode->i_cdev, struct spipe_device, cdev);
	ret = sbuf_status(spipe->init->dst, spipe->init->channel);
	if (ret)
		return ret;

	sbuf = kmalloc_obj(struct spipe_sbuf, GFP_KERNEL);
	if (!sbuf)
		return -ENOMEM;
	filp->private_data = sbuf;

	sbuf->dst = spipe->init->dst;
	sbuf->channel = spipe->init->channel;
	sbuf->bufid = minor - spipe->minor;

	return stream_open(inode, filp);
}

static int spipe_release(struct inode *inode, struct file *filp)
{
	struct spipe_sbuf *sbuf = filp->private_data;

	kfree(sbuf);

	return 0;
}

static ssize_t spipe_read(struct file *filp, char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct spipe_sbuf *sbuf = filp->private_data;
	int ret, timeout = -1;

	if (filp->f_flags & O_NONBLOCK)
		timeout = 0;

	ret = sbuf_read_user(sbuf->dst, sbuf->channel, sbuf->bufid, buf,
			     min_t(size_t, count, MAX_RW_COUNT), timeout);
	if (!timeout && (ret == -ENODATA || ret == -EBUSY))
		return -EAGAIN;
	return ret;
}

static ssize_t spipe_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct spipe_sbuf *sbuf = filp->private_data;
	int ret, timeout = -1;

	if (filp->f_flags & O_NONBLOCK)
		timeout = 0;

	ret = sbuf_write_user(sbuf->dst, sbuf->channel, sbuf->bufid, buf,
			      min_t(size_t, count, MAX_RW_COUNT), timeout);
	if (!timeout && (ret == -ENODATA || ret == -EBUSY))
		return -EAGAIN;
	return ret;
}

static __poll_t spipe_poll(struct file *filp, poll_table *wait)
{
	struct spipe_sbuf *sbuf = filp->private_data;

	return sbuf_poll_wait(sbuf->dst, sbuf->channel, sbuf->bufid, filp,
			      wait);
}

static long spipe_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -ENOTTY;
}

static const struct file_operations spipe_fops = {
	.open = spipe_open,
	.release = spipe_release,
	.read = spipe_read,
	.write = spipe_write,
	.poll = spipe_poll,
	.unlocked_ioctl = spipe_ioctl,
	.owner = THIS_MODULE,
};

static int spipe_parse_dt(struct spipe_init_data **init, struct device_node *np,
			  struct device *dev)
{
	struct spipe_init_data *pdata = NULL;
	int ret;
	u32 data;

	pdata = devm_kzalloc(dev, sizeof(struct spipe_init_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_property_read_string(np, "sprd,name",
				      (const char **)&pdata->name);
	if (ret)
		goto error;

	ret = of_property_read_u32(np, "sprd,dst", (u32 *)&data);
	if (ret)
		goto error;
	if (data >= SIPC_ID_NR) {
		ret = -EINVAL;
		goto error;
	}
	pdata->dst = (u8)data;

	ret = of_property_read_u32(np, "sprd,channel", (u32 *)&data);
	if (ret)
		goto error;
	if (data >= SMSG_CH_NR ||
	    sipc_channel2index(data) == INVALID_CHANEL_INDEX) {
		ret = -EINVAL;
		goto error;
	}
	pdata->channel = (u8)data;

	ret = of_property_read_u32(np, "sprd,ringnr", (u32 *)&pdata->ringnr);
	if (ret)
		goto error;

	ret = of_property_read_u32(np, "sprd,size-rxbuf",
				   (u32 *)&pdata->rxbuf_size);
	if (ret)
		goto error;

	ret = of_property_read_u32(np, "sprd,size-txbuf",
				   (u32 *)&pdata->txbuf_size);
	if (ret)
		goto error;

	*init = pdata;
	return ret;
error:
	devm_kfree(dev, pdata);
	*init = NULL;
	return ret;
}

static inline void spipe_destroy_pdata(struct spipe_init_data **init,
				       struct device *dev)
{
	struct spipe_init_data *pdata = *init;

	devm_kfree(dev, pdata);

	*init = NULL;
}

static int spipe_probe(struct platform_device *pdev)
{
	struct spipe_init_data *init;
	struct spipe_device *spipe;
	struct device *dev;
	dev_t devid;
	int i, ret;

	if (!pdev->dev.of_node)
		return -ENODEV;
	ret = spipe_parse_dt(&init, pdev->dev.of_node, &pdev->dev);
	if (ret)
		return ret;
	if (!init->ringnr || init->ringnr > MINORMASK + 1U) {
		ret = -EINVAL;
		goto free_pdata;
	}

	spipe = devm_kzalloc(&pdev->dev, sizeof(*spipe), GFP_KERNEL);
	if (!spipe) {
		ret = -ENOMEM;
		goto free_pdata;
	}
	spipe->init = init;
	ret = sbuf_create(init->dst, init->channel, init->ringnr,
			  init->txbuf_size, init->rxbuf_size);
	if (ret)
		goto free_pdata;
	ret = alloc_chrdev_region(&devid, 0, init->ringnr, init->name);
	if (ret)
		goto destroy_sbuf;
	spipe->major = MAJOR(devid);
	spipe->minor = MINOR(devid);

	for (i = 0; i < init->ringnr; i++) {
		if (init->ringnr > 1)
			dev = device_create_with_groups(spipe_class, &pdev->dev,
							devid + i, spipe,
							spipe_session_groups,
							"%s%d", init->name, i);
		else
			dev = device_create_with_groups(spipe_class, &pdev->dev,
							devid, spipe,
							spipe_session_groups,
							"%s", init->name);
		if (IS_ERR(dev)) {
			ret = PTR_ERR(dev);
			goto destroy_devices;
		}
	}

	/* Initialize all open-time data before the cdev becomes visible. */
	cdev_init(&spipe->cdev, &spipe_fops);
	spipe->cdev.owner = THIS_MODULE;
	ret = cdev_add(&spipe->cdev, devid, init->ringnr);
	if (ret)
		goto destroy_devices;
	platform_set_drvdata(pdev, spipe);
	return 0;

destroy_devices:
	while (i--)
		device_destroy(spipe_class, devid + i);
	unregister_chrdev_region(devid, init->ringnr);
destroy_sbuf:
	sbuf_destroy(init->dst, init->channel);
free_pdata:
	spipe_destroy_pdata(&init, &pdev->dev);
	return ret;
}

static void spipe_remove(struct platform_device *pdev)
{
	struct spipe_device *spipe = platform_get_drvdata(pdev);
	int i;

	if (spipe) {
		for (i = 0; i < spipe->init->ringnr; i++) {
			device_destroy(spipe_class,
				       MKDEV(spipe->major, spipe->minor + i));
		}
		cdev_del(&(spipe->cdev));
		unregister_chrdev_region(MKDEV(spipe->major, spipe->minor),
					 spipe->init->ringnr);

		sbuf_destroy(spipe->init->dst, spipe->init->channel);

		spipe_destroy_pdata(&spipe->init, &pdev->dev);

		devm_kfree(&pdev->dev, spipe);

		platform_set_drvdata(pdev, NULL);
	}

}

static const struct of_device_id spipe_match_table[] = {
	{
		.compatible = "sprd,spipe",
	},
	{},
};

static struct platform_driver spipe_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "spipe",
		.suppress_bind_attrs = true,
		.of_match_table = spipe_match_table,
	},
	.probe = spipe_probe,
	.remove = spipe_remove,
};

static int __init spipe_init(void)
{
	int ret;

	spipe_class = class_create("spipe");
	if (IS_ERR(spipe_class))
		return PTR_ERR(spipe_class);

	ret = platform_driver_register(&spipe_driver);
	if (ret)
		class_destroy(spipe_class);
	return ret;
}

static void __exit spipe_exit(void)
{
	platform_driver_unregister(&spipe_driver);
	class_destroy(spipe_class);
}

module_init(spipe_init);
module_exit(spipe_exit);

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SPIPE driver");
MODULE_LICENSE("GPL");
