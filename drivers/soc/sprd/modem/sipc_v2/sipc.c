// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Spreadtrum Communications Inc.
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

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sipc.h>
#include <linux/sprd_pmsys.h>
#include <linux/sizes.h>

#include "sipc_priv.h"

#define MBOX_BAMK "mbox"
#define PCIE_BAMK "pcie"

#if defined(CONFIG_DEBUG_FS)
void sipc_debug_putline(struct seq_file *m, char c, int n)
{
	char buf[300];
	int i, max, len;

	/* buf will end with '\n' and 0 */
	max = ARRAY_SIZE(buf) - 2;
	len = (n > max) ? max : n;

	for (i = 0; i < len; i++)
		buf[i] = c;

	buf[i] = '\n';
	buf[i + 1] = 0;

	seq_puts(m, buf);
}
EXPORT_SYMBOL_GPL(sipc_debug_putline);
#endif

static u32 sipc_rxirq_status(u8 dst)
{
	return 0;
}

static void sipc_rxirq_clear(u8 dst)
{
}

static int sipc_txirq_trigger(u8 dst, u64 msg)
{
	struct smsg_ipc *ipc;

	ipc = smsg_ipcs[dst];

	if (ipc) {
#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
		if (ipc->type == SIPC_BASE_MBOX) {
			struct udx710_smsg message;

			message.channel = msg;
			message.type = msg >> 8;
			message.flag = msg >> 16;
			message.value = msg >> 32;
			return udx710_smsg_link_send(ipc->mbox_link, &message);
		}
#endif
#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
		if (ipc->type == SIPC_BASE_MBOX)
			return mbox_raw_sent(ipc->core_id, msg);
#endif

		if (ipc->type == SIPC_BASE_PCIE) {
#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
			sprd_ep_dev_raise_irq(ipc->ep_dev, PCIE_EP_SIPC_IRQ);
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
			sprd_pci_epf_raise_irq(ipc->ep_fun, SPRD_EPF_SIPC_IRQ);
#endif
			return 0;
		}
	}
	return -EOPNOTSUPP;
}

static int sipc_parse_dt(struct smsg_ipc *ipc, struct device_node *np,
			 struct device *dev)
{
	u32 val[3];
	int ret;
	const char *type;

	/* get name */
	ret = of_property_read_string(np, "sprd,name", &ipc->name);
	if (ret)
		return ret;

	pr_debug("sipc: name=%s\n", ipc->name);

	/* get sipc type, optional */
	if (of_property_read_string(np, "sprd,type", &type) == 0) {
		pr_debug("sipc: type=%s\n", type);
		if (strcmp(MBOX_BAMK, type) == 0)
			ipc->type = SIPC_BASE_MBOX;
		else if (strcmp(PCIE_BAMK, type) == 0)
			ipc->type = SIPC_BASE_PCIE;
		else
			return -EINVAL;
	}

	/* get sipc client, optional */
	if (of_property_read_u32_array(np, "sprd,client", val, 1) == 0) {
		if (val[0] > 1)
			return -EINVAL;
		ipc->client = (u8)val[0];
		pr_debug("sipc: client=%d\n", ipc->client);
	}

	/* get sipc dst */
	ret = of_property_read_u32_array(np, "sprd,dst", val, 1);
	if (!ret) {
		if (val[0] >= SIPC_ID_NR)
			return -EINVAL;
		ipc->dst = (u8)val[0];
		pr_debug("sipc: dst =%d\n", ipc->dst);
	}

	if (ret || ipc->dst >= SIPC_ID_NR) {
		pr_err("sipc: dst err, ret =%d.\n", ret);
		return ret;
	}

#if defined(CONFIG_SPRD_MAILBOX) && !defined(CONFIG_SPRD_SIPC_MBOX_CLIENT)
	if (ipc->type == SIPC_BASE_MBOX) {
		/* get core id */
		ipc->core_id = (u8)MBOX_INVALID_CORE;
		ret = of_property_read_u32_array(np, "sprd,core", val, 1);
		if (!ret) {
			ipc->core_id = (u8)val[0];
			pr_debug("sipc: core=%d\n", ipc->core_id);
		} else {
			pr_err("sipc: core err, ret =%d.\n", ret);
			return ret;
		}

		/* get core sensor id, optional*/
		ipc->core_sensor_id = (u8)MBOX_INVALID_CORE;
		if (of_property_read_u32_array(np, "sprd,core_sensor", val,
					       1) == 0) {
			ipc->core_sensor_id = (u8)val[0];
			pr_debug("sipc: core_sensor=%d\n", ipc->core_sensor_id);
		}
	}
#endif

#ifdef CONFIG_SPRD_PCIE_EP_DEVICE
	if (ipc->type == SIPC_BASE_PCIE) {
		ipc->irq = PCIE_EP_SIPC_IRQ;
		ret = of_property_read_u32_array(np, "sprd,ep-dev",
						 &ipc->ep_dev, 1);
		pr_debug("sipc: ep_dev=%d\n", ipc->ep_dev);
		if (ret || ipc->ep_dev >= PCIE_EP_NR) {
			pr_err("sipc: ep_dev err, ret =%d.\n", ret);
			return ret;
		}
	}
#endif

#ifdef CONFIG_PCIE_EPF_SPRD
	if (ipc->type == SIPC_BASE_PCIE) {
		ret = of_property_read_u32_array(np, "sprd,ep-fun",
						 &ipc->ep_fun, 1);
		pr_debug("sipc: ep_fun=%d\n", ipc->ep_fun);
		if (ret || ipc->ep_fun >= SPRD_FUNCTION_MAX) {
			pr_err("sipc: ep_fun err, ret =%d.\n", ret);
			return ret;
		}

		/* parse doolbell irq */
		ret = of_irq_get(np, 0);
		if (ret < 0) {
			pr_err("sipc: doorbell irq err, ret=%d\n", ret);
			return -EINVAL;
		}
		ipc->irq = ret;
		pr_debug("sipc: irq=%d\n", ipc->irq);
	}
#endif

	/* get smem type */
	ret = of_property_read_u32_array(np, "sprd,smem-type", &ipc->smem_type,
					 1);
	if (ret)
		ipc->smem_type = SMEM_LOCAL;

	pr_debug("sipc: smem_type = %d, ret =%d\n", ipc->smem_type, ret);

	/* get smem info */
	ret = of_property_read_u32_array(np, "sprd,smem-info", val, 3);
	if (ret) {
		pr_err("sipc: parse smem info failed.\n");
		return ret;
	}
	ipc->smem_base = val[0];
	ipc->dst_smem_base = val[1];
	ipc->smem_size = val[2];
	pr_debug("sipc: smem_base=0x%x, dst_smem_base=0x%x, smem_size=0x%x\n",
		 ipc->smem_base, ipc->dst_smem_base, ipc->smem_size);

	/* try to get high_offset */
	ret = of_property_read_u32_array(np, "sprd,high-offset", val, 2);
	if (!ret) {
		ipc->high_offset = val[0];
		ipc->dst_high_offset = val[1];
	}
	pr_debug("sipc:  high_offset=0x%x, dst_high_offset=0x%x\n",
		 ipc->high_offset, ipc->dst_high_offset);

	if (ipc->type == SIPC_BASE_PCIE) {
		/* need wait pcie linkup */
		ipc->suspend = 1;
		/* pcie sipc, the host must use loacal SMEM_LOCAL */
		if (!ipc->client && ipc->smem_type != SMEM_LOCAL) {
			pr_err("sipc: host must use local smem!");
			return -EINVAL;
		}
	}

	return 0;
}

#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
static int sipc_validate_local_peer(struct smsg_ipc *ipc,
				    struct device_node *np)
{
	struct of_phandle_args args;
	u32 core;
	int index, ret;

	if (ipc->type != SIPC_BASE_MBOX || ipc->client ||
	    ipc->smem_type != SMEM_LOCAL ||
	    !sipc_local_peer_supported(ipc->dst))
		return -EOPNOTSUPP;
	/* These captured board peers all use the low 32-bit AP address domain. */
	if (ipc->high_offset || ipc->dst_high_offset)
		return -EOPNOTSUPP;
	if (ipc->dst == SIPC_ID_PM_SYS) {
		/* The PM peer uses AON SRAM, not the DDR-backed modem pool. */
		if (ipc->smem_base != 0x8400 ||
		    ipc->dst_smem_base != 0x60008400 ||
		    ipc->smem_size != 0x2000)
			return -EOPNOTSUPP;
	} else if (ipc->smem_base < 0x87800000 || !ipc->smem_size ||
		   ipc->smem_size > 0x800000 ||
		   (u64)ipc->smem_base + ipc->smem_size > 0x88000000 ||
		   ipc->dst_smem_base != ipc->smem_base) {
		/* DDR subpools must remain inside the boot-DT SIPC reservation. */
		return -EINVAL;
	}

	index = of_property_match_string(np, "mbox-names", "smsg");
	if (index < 0)
		return index;
	ret = of_parse_phandle_with_args(np, "mboxes", "#mbox-cells", index,
					 &args);
	if (ret)
		return ret;
	ret = args.args_count == 1 && args.args[0] < 9 ? 0 : -EINVAL;
	if (!ret) {
		ipc->core_id = args.args[0];
		if (!of_property_read_u32(np, "sprd,core", &core) &&
		    (core >= 9 || core != ipc->core_id))
			ret = -EINVAL;
	}
	of_node_put(args.np);
	return ret;
}
#endif

static int sipc_probe(struct platform_device *pdev)
{
	struct sprd_pmsys *pmsys = NULL;
	struct smsg_ipc *ipc;
	struct device_node *np;
	int ret;

	if (pdev->dev.of_node) {
		np = pdev->dev.of_node;
		ipc = devm_kzalloc(&pdev->dev, sizeof(struct smsg_ipc),
				   GFP_KERNEL);
		if (!ipc)
			return -ENOMEM;

		ret = sipc_parse_dt(ipc, np, &pdev->dev);
		if (ret)
			return ret;
		ipc->dev = &pdev->dev;
		if (ipc->dst == SIPC_ID_PM_SYS &&
		    of_find_property(np, "sprd,pm-sys", NULL)) {
			pmsys = sprd_pmsys_get(&pdev->dev);
			if (IS_ERR(pmsys))
				return PTR_ERR(pmsys);
		}
#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
		ret = sipc_validate_local_peer(ipc, np);
		if (ret)
			return ret;
#endif

		ipc->rxirq_status = sipc_rxirq_status;
		ipc->rxirq_clear = sipc_rxirq_clear;
		ipc->txirq_trigger = sipc_txirq_trigger;
		init_waitqueue_head(&ipc->suspend_wait);
		spin_lock_init(&ipc->suspend_pinlock);
		spin_lock_init(&ipc->txpinlock);

		ret = smsg_ipc_create(ipc);
		if (ret)
			return ret;
		/* PM_SYS sends OPEN during startup. Publish the mailbox receiver
		 * first; UART/DDR consumers follow this completed endpoint probe.
		 */
		if (pmsys) {
			ret = sprd_pmsys_start(pmsys);
			if (ret) {
				smsg_ipc_destroy(ipc);
				return ret;
			}
		}
		platform_set_drvdata(pdev, ipc);
	}
	return 0;
}

static void sipc_remove(struct platform_device *pdev)
{
	struct smsg_ipc *ipc = platform_get_drvdata(pdev);

	smsg_ipc_destroy(ipc);

	devm_kfree(&pdev->dev, ipc);
}

static const struct of_device_id sipc_match_table[] = {
	{
		.compatible = "sprd,sipc",
	},
	{},
};

static struct platform_driver sipc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sipc",
#ifdef CONFIG_SPRD_SIPC_MBOX_CLIENT
		.suppress_bind_attrs = true,
#endif
		.of_match_table = sipc_match_table,
	},
	.probe = sipc_probe,
	.remove = sipc_remove,
};

static int __init sipc_init(void)
{
	smsg_init_channel2index();
	return platform_driver_register(&sipc_driver);
}

static void __exit sipc_exit(void)
{
	platform_driver_unregister(&sipc_driver);
}

subsys_initcall_sync(sipc_init);
module_exit(sipc_exit);

MODULE_AUTHOR("Wenping Zhou");
MODULE_DESCRIPTION("SIPC module driver");
MODULE_LICENSE("GPL");
