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
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/sipc.h>

#include "spool.h"

struct spool_device;

struct spool_device {
	struct spool_init_data *init;
	int major;
	int minor;
	struct cdev cdev;
	struct platform_device *plt_dev;

	struct device *sys_dev; /* Device object in sysfs */
};

struct spool_sblock {
	u8 dst;
	u8 channel;
	u32 generation;
	/* Serializes reads through duplicated descriptors. */
	struct mutex rxlock;
	struct sblock hold;
	bool owns_hold;
	u32 offset;
	/* Published independently for poll; hold and offset need rxlock. */
	u32 remaining;
};

static struct class *spool_class;

static int spool_open(struct inode *inode, struct file *filp)
{
	struct spool_device *spool;
	struct spool_sblock *sblock;
	int ret;
	u32 generation;

	spool = container_of(inode->i_cdev, struct spool_device, cdev);
	ret = sblock_get_generation(spool->init->dst, spool->init->channel,
				    &generation);
	if (ret)
		return ret;
	sblock = kzalloc_obj(*sblock, GFP_KERNEL);
	if (!sblock)
		return -ENOMEM;
	filp->private_data = sblock;

	sblock->dst = spool->init->dst;
	sblock->channel = spool->init->channel;
	sblock->generation = generation;
	mutex_init(&sblock->rxlock);

	return stream_open(inode, filp);
}

static int spool_release(struct inode *inode, struct file *filp)
{
	struct spool_sblock *sblock = filp->private_data;

	if (sblock->owns_hold) {
		if (sblock_release(sblock->dst, sblock->channel, &sblock->hold))
			pr_debug("failed to release block!\n");
	}
	kfree(sblock);

	return 0;
}

/* Old file descriptions stay hung up; userspace must close and reopen. */
static int spool_check_generation(struct spool_sblock *sblock)
{
	u32 generation;
	int ret = sblock_get_generation(sblock->dst, sblock->channel,
					&generation);

	return ret || generation != sblock->generation ? -EPIPE : 0;
}

static void spool_drop_hold(struct spool_sblock *sblock)
{
	if (sblock->owns_hold) {
		/* The token permits recycling a genuine old loan, not sending it. */
		if (!sblock_release(sblock->dst, sblock->channel,
				    &sblock->hold)) {
			sblock->owns_hold = false;
			WRITE_ONCE(sblock->remaining, 0);
		}
	}
}

static ssize_t spool_read(struct file *filp, char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct spool_sblock *sblock = filp->private_data;
	int timeout = filp->f_flags & O_NONBLOCK ? 0 : -1;
	u32 bytes, copied, left;
	int ret;

	if (!count)
		return 0;
	if (!timeout) {
		if (!mutex_trylock(&sblock->rxlock))
			return -EAGAIN;
	} else if (mutex_lock_interruptible(&sblock->rxlock)) {
		return -ERESTARTSYS;
	}

	/* A held block must not bypass an observed transport shutdown. */
	ret = spool_check_generation(sblock);
	if (ret) {
		spool_drop_hold(sblock);
		goto unlock;
	}
	if (!sblock->remaining) {
		if (sblock->owns_hold) {
			/* Keep the token if a prior release failed; never overwrite it. */
			ret = sblock_release(sblock->dst, sblock->channel,
					     &sblock->hold);
			if (ret)
				goto unlock;
			sblock->owns_hold = false;
		}
		ret = sblock_receive_for_generation(sblock->dst,
						    sblock->channel,
						    &sblock->hold, timeout,
						    sblock->generation);
		if (ret)
			goto unlock;
		sblock->owns_hold = true;
		sblock->offset = 0;
		WRITE_ONCE(sblock->remaining, sblock->hold.length);
		/* An empty block is still owned and must be returned once. */
		if (!sblock->remaining) {
			ret = sblock_release(sblock->dst, sblock->channel,
					     &sblock->hold);
			if (!ret) {
				sblock->owns_hold = false;
				ret = -EIO;
			}
			goto unlock;
		}
	}

	bytes = min_t(size_t, count, sblock->remaining);
	left = unalign_copy_to_user(buf, sblock->hold.addr + sblock->offset,
				    bytes);
	/* Reset may race a faulting user copy; never report stale bytes as valid. */
	if (spool_check_generation(sblock)) {
		spool_drop_hold(sblock);
		ret = -EPIPE;
		goto unlock;
	}
	copied = bytes - left;
	sblock->offset += copied;
	WRITE_ONCE(sblock->remaining, sblock->remaining - copied);
	ret = copied ? copied : -EFAULT;

	/* Preserve both the copied count and the unread suffix on user faults. */
	if (!sblock->remaining) {
		if (!sblock_release(sblock->dst, sblock->channel,
				    &sblock->hold))
			sblock->owns_hold = false;
		else
			pr_debug("failed to release consumed block!\n");
	}
unlock:
	mutex_unlock(&sblock->rxlock);
	if (!timeout && (ret == -ENODATA || ret == -EBUSY))
		return -EAGAIN;
	return ret;
}

static ssize_t spool_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct spool_sblock *sblock = filp->private_data;
	int timeout = filp->f_flags & O_NONBLOCK ? 0 : -1;
	struct sblock blk;
	size_t done = 0, bytes;
	int ret = 0;

	count = min_t(size_t, count, MAX_RW_COUNT);
	while (done < count) {
		ret = sblock_get_for_generation(sblock->dst, sblock->channel,
						&blk, timeout,
						sblock->generation);
		if (ret)
			break;
		bytes = min_t(size_t, blk.length, count - done);
		if (!bytes) {
			sblock_put(sblock->dst, sblock->channel, &blk);
			ret = -EIO;
			break;
		}
		if (unalign_copy_from_user(blk.addr, buf + done, bytes)) {
			sblock_put(sblock->dst, sblock->channel, &blk);
			ret = -EFAULT;
			break;
		}
		blk.length = bytes;
		ret = sblock_send(sblock->dst, sblock->channel, &blk);
		/* Failure is guaranteed to precede publication; ownership is ours. */
		if (ret) {
			sblock_put(sblock->dst, sblock->channel, &blk);
			break;
		}
		done += bytes;
	}
	if (!done && !timeout && (ret == -ENODATA || ret == -EBUSY))
		ret = -EAGAIN;
	return done ? done : ret;
}

static __poll_t spool_poll(struct file *filp, poll_table *wait)
{
	struct spool_sblock *sblock = filp->private_data;

	__poll_t mask;

	/* Register transport waiters even when this file has a partial block. */
	mask = sblock_poll_wait(sblock->dst, sblock->channel, filp, wait);
	if (spool_check_generation(sblock))
		return EPOLLERR | EPOLLHUP;
	if (READ_ONCE(sblock->remaining))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static long spool_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -ENOTTY;
}

static const struct file_operations spool_fops = {
	.open = spool_open,
	.release = spool_release,
	.read = spool_read,
	.write = spool_write,
	.poll = spool_poll,
	.unlocked_ioctl = spool_ioctl,
	.owner = THIS_MODULE,
};

static int spool_parse_dt(struct spool_init_data **init, struct device *dev,
			  struct device_node *np)
{
	struct spool_init_data *pdata = NULL;
	int ret;
	u32 data;

	pdata = devm_kzalloc(dev, sizeof(struct spool_init_data), GFP_KERNEL);
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

	ret = of_property_read_u32(np, "sprd,preconfigured", (u32 *)&data);
	if (!ret)
		pdata->pre_cfg = (int)data;

	ret = of_property_read_u32(np, "sprd,tx-blksize",
				   (u32 *)&pdata->txblocksize);
	if (ret)
		goto error;
	ret = of_property_read_u32(np, "sprd,tx-blknum",
				   (u32 *)&pdata->txblocknum);
	if (ret)
		goto error;
	ret = of_property_read_u32(np, "sprd,rx-blksize",
				   (u32 *)&pdata->rxblocksize);
	if (ret)
		goto error;
	ret = of_property_read_u32(np, "sprd,rx-blknum",
				   (u32 *)&pdata->rxblocknum);
	if (ret)
		goto error;

	if (!of_property_read_u32(np, "sprd,nodev", (u32 *)&data))
		pdata->nodev = (u8)data;

	*init = pdata;
	return ret;
error:
	devm_kfree(dev, pdata);
	*init = NULL;
	return ret;
}

static ssize_t base_addr_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct spool_device *spool =
		(struct spool_device *)dev_get_drvdata(dev);
	struct spool_init_data *init = spool->init;
	uint32_t addr;
	int ret;

	ret = sblock_get_smem_cp_addr(init->dst, init->channel, &addr);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%u %u 0x%08X %d %u %u %u %u\n",
			(unsigned int)init->dst, (unsigned int)init->channel,
			addr, init->pre_cfg, (unsigned int)init->txblocknum,
			(unsigned int)init->txblocksize,
			(unsigned int)init->rxblocknum,
			(unsigned int)init->rxblocksize);
}

static DEVICE_ATTR(base_addr, 0440, base_addr_show, NULL);

static int create_spool(struct platform_device *pdev,
			struct spool_init_data *init, struct spool_device **out)
{
	int rval;
	struct spool_device *spool;
	dev_t dev_no;
	char sp_name[16];

	snprintf(sp_name, sizeof(sp_name), "spool-%u-%u",
		 (unsigned int)init->dst, (unsigned int)init->channel);
	rval = alloc_chrdev_region(&dev_no, 0, 1, sp_name);
	if (rval)
		return rval;

	if (init->pre_cfg)
		rval = sblock_pcfg_create(init->dst, init->channel,
					  init->txblocknum, init->txblocksize,
					  init->rxblocknum, init->rxblocksize);
	else
		rval = sblock_create(init->dst, init->channel, init->txblocknum,
				     init->txblocksize, init->rxblocknum,
				     init->rxblocksize);
	if (rval) {
		pr_info("Failed to create sblock: %d\n", rval);
		goto free_devno;
	}

	spool = devm_kzalloc(&pdev->dev, sizeof(struct spool_device),
			     GFP_KERNEL);
	if (!spool) {
		rval = -ENOMEM;
		goto free_sblock;
	}

	spool->init = init;
	spool->major = MAJOR(dev_no);
	spool->minor = MINOR(dev_no);
	spool->plt_dev = pdev;

	platform_set_drvdata(pdev, spool);
	spool->sys_dev = device_create(spool_class, &pdev->dev, dev_no, spool,
				       "%s", init->name);
	if (IS_ERR(spool->sys_dev)) {
		rval = PTR_ERR(spool->sys_dev);
		goto clear_drvdata;
	}
	rval = device_create_file(&pdev->dev, &dev_attr_base_addr);
	if (rval)
		goto destroy_device;

	/* No failing step may follow publication of the openable cdev. */
	if (!init->nodev) {
		cdev_init(&spool->cdev, &spool_fops);
		spool->cdev.owner = THIS_MODULE;
		rval = cdev_add(&spool->cdev, dev_no, 1);
		if (rval)
			goto remove_attr;
	}

	*out = spool;

	return 0;

remove_attr:
	device_remove_file(&pdev->dev, &dev_attr_base_addr);
destroy_device:
	device_destroy(spool_class, dev_no);
clear_drvdata:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, spool);

free_sblock:
	sblock_destroy(init->dst, init->channel);

free_devno:
	unregister_chrdev_region(dev_no, 1);
	return rval;
}

static int destroy_spool(struct spool_device *spool)
{
	dev_t dev_no = MKDEV(spool->major, spool->minor);
	struct spool_init_data *init = spool->init;

	device_remove_file(&spool->plt_dev->dev, &dev_attr_base_addr);
	if (spool->sys_dev) {
		device_destroy(spool_class, dev_no);
		spool->sys_dev = NULL;
	}
	if (!init->nodev)
		cdev_del(&spool->cdev);
	sblock_destroy(init->dst, init->channel);
	unregister_chrdev_region(dev_no, 1);
	devm_kfree(&spool->plt_dev->dev, init);
	devm_kfree(&spool->plt_dev->dev, spool);

	return 0;
}

static int spool_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int rval;
	struct spool_init_data *init;
	struct spool_device *spool;

	if (!np)
		return -ENODEV;

	rval = spool_parse_dt(&init, &pdev->dev, np);
	if (rval) {
		pr_err("Failed to parse spool device tree, ret=%d\n", rval);
		return rval;
	}

	pr_info("spool: name=%s, dst=%u, channel=%u, pre_cfg=%u\n", init->name,
		init->dst, init->channel, init->pre_cfg);

	pr_info("spool: tx_num=%u, tx_size=%u, rx_num=%u, rx_size=%u\n",
		init->txblocknum, init->txblocksize, init->rxblocknum,
		init->rxblocksize);

	rval = create_spool(pdev, init, &spool);
	if (rval) {
		pr_err("Failed to create spool device %u:%u, ret=%d\n",
		       (unsigned int)init->dst, (unsigned int)init->channel,
		       rval);
		devm_kfree(&pdev->dev, init);
	}

	return rval;
}

static void spool_remove(struct platform_device *pdev)
{
	struct spool_device *priv =
		(struct spool_device *)platform_get_drvdata(pdev);

	if (priv)
		destroy_spool(priv);

	platform_set_drvdata(pdev, NULL);

}

static const struct of_device_id spool_match_table[] = {
	{
		.compatible = "sprd,spool",
	},
	{},
};
static struct platform_driver spool_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "spool",
		.suppress_bind_attrs = true,
		.of_match_table = spool_match_table,
	},
	.probe = spool_probe,
	.remove = spool_remove,
};

static int __init spool_init(void)
{
	int ret;

	spool_class = class_create("spool");
	if (IS_ERR(spool_class))
		return PTR_ERR(spool_class);

	ret = platform_driver_register(&spool_driver);
	if (ret)
		class_destroy(spool_class);
	return ret;
}

static void __exit spool_exit(void)
{
	platform_driver_unregister(&spool_driver);
	class_destroy(spool_class);
}

module_init(spool_init);
module_exit(spool_exit);

MODULE_AUTHOR("Qiu Yi");
MODULE_DESCRIPTION("SIPC/SPOOL driver");
MODULE_LICENSE("GPL");
