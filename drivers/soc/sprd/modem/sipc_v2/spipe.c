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
#include "sbuf.h"

struct spipe_endpoint {
	struct device dev;
	struct cdev cdev;
	struct sbuf_mgr *sbuf;
	u32 generation;
	u32 ring;
	bool dead;
};

struct spipe_device {
	struct spipe_init_data *init;
	struct device *parent;
	struct sbuf_mgr *sbuf;
	struct notifier_block notifier;
	struct work_struct work;
	struct spipe_endpoint **endpoints;
	dev_t devid;
	u32 generation;
	bool attempted;
};

static struct class *spipe_class;

static int spipe_open(struct inode *inode, struct file *filp)
{
	struct spipe_endpoint *ep =
		container_of(inode->i_cdev, struct spipe_endpoint, cdev);
	u32 generation;
	int ret = sbuf_session_state(ep->sbuf, &generation);

	if (ret)
		return ret;
	if (READ_ONCE(ep->dead) || generation != ep->generation)
		return -EPIPE;
	get_device(&ep->dev);
	filp->private_data = ep;
	return stream_open(inode, filp);
}

static int spipe_release(struct inode *inode, struct file *filp)
{
	struct spipe_endpoint *ep = filp->private_data;

	put_device(&ep->dev);
	return 0;
}

static ssize_t spipe_read(struct file *filp, char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct spipe_endpoint *ep = filp->private_data;
	int ret, timeout = filp->f_flags & O_NONBLOCK ? 0 : -1;

	ret = sbuf_session_read(ep->sbuf, ep->generation, &ep->dead, ep->ring,
				(void __force *)buf,
				min_t(size_t, count, MAX_RW_COUNT), timeout,
				true);
	if (!timeout && (ret == -ENODATA || ret == -EBUSY))
		return -EAGAIN;
	return ret;
}

static ssize_t spipe_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct spipe_endpoint *ep = filp->private_data;
	int ret, timeout = filp->f_flags & O_NONBLOCK ? 0 : -1;

	ret = sbuf_session_write(ep->sbuf, ep->generation, &ep->dead, ep->ring,
				 (const void __force *)buf,
				 min_t(size_t, count, MAX_RW_COUNT), timeout,
				 true);
	if (!timeout && (ret == -ENODATA || ret == -EBUSY))
		return -EAGAIN;
	return ret;
}

static __poll_t spipe_poll(struct file *filp, poll_table *wait)
{
	struct spipe_endpoint *ep = filp->private_data;

	return sbuf_session_poll(ep->sbuf, ep->generation, &ep->dead, ep->ring,
				 filp, wait);
}

static const struct file_operations spipe_fops = {
	.open = spipe_open,
	.release = spipe_release,
	.read = spipe_read,
	.write = spipe_write,
	.poll = spipe_poll,
	.owner = THIS_MODULE,
};

static void spipe_endpoint_release(struct device *dev)
{
	struct spipe_endpoint *ep =
		container_of(dev, struct spipe_endpoint, dev);

	sbuf_put_manager(ep->sbuf);
	kfree(ep);
}

static void spipe_unregister_endpoints(struct spipe_device *spipe)
{
	u32 i;

	for (i = 0; i < spipe->init->ringnr; i++) {
		struct spipe_endpoint *ep = spipe->endpoints[i];

		if (!ep)
			continue;
		sbuf_session_cancel(ep->sbuf, ep->ring, &ep->dead);
		cdev_device_del(&ep->cdev, &ep->dev);
		put_device(&ep->dev);
		spipe->endpoints[i] = NULL;
	}
}

static int spipe_register_endpoint(struct spipe_device *spipe, u32 ring)
{
	struct spipe_endpoint *ep;
	int ret;

	ep = kzalloc(sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;
	ep->sbuf = spipe->sbuf;
	kref_get(&ep->sbuf->ref);
	ep->generation = spipe->generation;
	ep->ring = ring;
	device_initialize(&ep->dev);
	ep->dev.release = spipe_endpoint_release;
	ep->dev.class = spipe_class;
	ep->dev.parent = spipe->parent;
	ep->dev.devt = spipe->devid + ring;
	if (spipe->init->ringnr > 1)
		ret = dev_set_name(&ep->dev, "%s%u", spipe->init->name, ring);
	else
		ret = dev_set_name(&ep->dev, "%s", spipe->init->name);
	if (ret)
		goto put;
	cdev_init(&ep->cdev, &spipe_fops);
	ep->cdev.owner = THIS_MODULE;
	ret = cdev_device_add(&ep->cdev, &ep->dev);
	if (ret)
		goto put;
	spipe->endpoints[ring] = ep;
	return 0;
put:
	sbuf_session_cancel(ep->sbuf, ep->ring, &ep->dead);
	put_device(&ep->dev);
	return ret;
}

static void spipe_session_work(struct work_struct *work)
{
	struct spipe_device *spipe =
		container_of(work, struct spipe_device, work);
	u32 generation, i;
	int ret = sbuf_session_state(spipe->sbuf, &generation);

	/* The existing generation also catches RESET/READY coalesced by work. */
	if (ret || generation != spipe->generation) {
		spipe_unregister_endpoints(spipe);
		spipe->attempted = false;
	}
	if (ret || spipe->attempted)
		return;
	spipe->generation = generation;
	spipe->attempted = true;
	for (i = 0; i < spipe->init->ringnr; i++) {
		/* Preserve all 15 firmware rings, export only actual consumers. */
		if (spipe->init->dst == 5 && spipe->init->channel == 4 &&
		    i != 2 && i != 3 && i != 8)
			continue;
		ret = spipe_register_endpoint(spipe, i);
		if (ret) {
			dev_err(spipe->parent,
				"session port registration: %d\n", ret);
			spipe_unregister_endpoints(spipe);
			break;
		}
	}
}

static int spipe_event(struct notifier_block *nb, unsigned long event,
		       void *data)
{
	struct spipe_device *spipe =
		container_of(nb, struct spipe_device, notifier);

	if (event == SBUF_NOTIFY_READY || event == SBUF_NOTIFY_RESET)
		schedule_work(&spipe->work);
	return NOTIFY_OK;
}

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

static int spipe_probe(struct platform_device *pdev)
{
	struct spipe_init_data *init;
	struct spipe_device *spipe;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;
	ret = spipe_parse_dt(&init, pdev->dev.of_node, &pdev->dev);
	if (ret)
		return ret;
	if (!init->ringnr || init->ringnr > MINORMASK + 1U)
		return -EINVAL;
	spipe = devm_kzalloc(&pdev->dev, sizeof(*spipe), GFP_KERNEL);
	if (!spipe)
		return -ENOMEM;
	spipe->endpoints = devm_kcalloc(&pdev->dev, init->ringnr,
					sizeof(*spipe->endpoints), GFP_KERNEL);
	if (!spipe->endpoints)
		return -ENOMEM;
	spipe->init = init;
	spipe->parent = &pdev->dev;
	INIT_WORK(&spipe->work, spipe_session_work);
	spipe->notifier.notifier_call = spipe_event;
	ret = sbuf_create(init->dst, init->channel, init->ringnr,
			  init->txbuf_size, init->rxbuf_size);
	if (ret)
		return ret;
	spipe->sbuf = sbuf_get_manager(init->dst, init->channel);
	ret = alloc_chrdev_region(&spipe->devid, 0, init->ringnr, init->name);
	if (ret)
		goto destroy;
	ret = blocking_notifier_chain_register(&spipe->sbuf->events,
					       &spipe->notifier);
	if (ret)
		goto unregister;
	platform_set_drvdata(pdev, spipe);
	/* Covers READY before the notifier was attached. */
	schedule_work(&spipe->work);
	return 0;
unregister:
	unregister_chrdev_region(spipe->devid, init->ringnr);
destroy:
	sbuf_destroy(init->dst, init->channel);
	sbuf_put_manager(spipe->sbuf);
	return ret;
}

static void spipe_remove(struct platform_device *pdev)
{
	struct spipe_device *spipe = platform_get_drvdata(pdev);

	blocking_notifier_chain_unregister(&spipe->sbuf->events,
					   &spipe->notifier);
	cancel_work_sync(&spipe->work);
	/* Invalidate and join ring I/O before removing nodes. Old files keep only
	 * their endpoint and the dead manager, never this devm controller.
	 */
	sbuf_destroy(spipe->init->dst, spipe->init->channel);
	spipe_unregister_endpoints(spipe);
	unregister_chrdev_region(spipe->devid, spipe->init->ringnr);
	sbuf_put_manager(spipe->sbuf);
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
