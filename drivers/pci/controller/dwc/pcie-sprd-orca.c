// SPDX-License-Identifier: GPL-2.0-only
/*
 * Orca PCIe root complex glue.
 * Derived from the GPL Spreadtrum 2018-2019 pcie-sprd.c/pcie-sprd-misc.c.
 * Copyright (C) 2018-2019 Spreadtrum corporation.
 */
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/mutex.h>
#include <linux/pcie-rc-sprd.h>
#include <linux/soc/sprd/sprd_pcie_ep_device.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "pcie-designware.h"

#define ORCA_PCIE_GEN_CTRL3 0xe58
#define ORCA_PCIE_LTSSM_EN BIT(0)
#define ORCA_PCIE_L1_AUXCLK_EN BIT(8)
#define ORCA_PCIE_PM_CTRL 0xe60
#define ORCA_PCIE_APP_CLK_PM_EN BIT(21)
#define ORCA_PCIE_TX_MSG 0xe80
#define ORCA_PCIE_PME_TURN_OFF BIT(19)
#define ORCA_PCIE_MAX_STEPS 64
#define ORCA_IPA_AHB_BASE 0x29000000
#define ORCA_PCIE_CORE_GATE 0
#define ORCA_PCIE_AUX_GATE 1
#define ORCA_PCIE_GATE_COUNT 2

enum orca_pcie_sequence {
	ORCA_STARTUP,
	ORCA_SHUTDOWN,
	ORCA_SUSPEND,
	ORCA_RESUME,
	ORCA_ASPM,
	ORCA_SEQUENCE_COUNT,
};

static const char *const orca_sequence_names[] = {
	"sprd,pcie-startup-syscons",  "sprd,pcie-shutdown-syscons",
	"sprd,pcie-suspend-syscons",  "sprd,pcie-resume-syscons",
	"sprd,pcie-aspml1p2-syscons",
};

struct orca_pcie_step {
	struct regmap *map;
	u32 type, delay_ms, reg, mask, value;
	int gate;
};

struct orca_pcie_sequence_data {
	struct orca_pcie_step *steps;
	unsigned int count;
};

struct orca_pcie {
	struct dw_pcie pci;
	struct mutex lifecycle_lock;
	bool manually_offline;
	struct orca_pcie_sequence_data sequence[ORCA_SEQUENCE_COUNT];
	u32 msi_enable[MAX_MSI_CTRLS];
	bool initialized;
	bool powered;
	bool suspending;
	bool msi_saved;
	int power_error;
	struct clk *gates[ORCA_PCIE_GATE_COUNT];
	bool gate_enabled[ORCA_PCIE_GATE_COUNT];
};

static struct orca_pcie *to_orca(struct dw_pcie *pci)
{
	return container_of(pci, struct orca_pcie, pci);
}

/* Resolve and validate every resource before executing any sequence. */
static int orca_pcie_parse_sequence(struct orca_pcie *pcie, unsigned int which)
{
	struct device *dev = pcie->pci.dev;
	struct device_node *np = dev->of_node;
	struct orca_pcie_sequence_data *seq = &pcie->sequence[which];
	const char *property = orca_sequence_names[which];
	struct of_phandle_args args;
	struct resource res;
	int count, i, ret;

	if (!of_property_present(np, property))
		return which == ORCA_ASPM ? 0 : -EINVAL;
	count = of_property_count_elems_of_size(np, property, 6 * sizeof(u32));
	if (count <= 0 || count > ORCA_PCIE_MAX_STEPS)
		return -EINVAL;
	seq->steps = devm_kcalloc(dev, count, sizeof(*seq->steps), GFP_KERNEL);
	if (!seq->steps)
		return -ENOMEM;
	for (i = 0; i < count; i++) {
		struct orca_pcie_step *step = &seq->steps[i];

		ret = of_parse_phandle_with_fixed_args(np, property, 5, i,
						       &args);
		if (ret)
			return ret;
		step->type = args.args[0];
		step->delay_ms = args.args[1];
		step->reg = args.args[2];
		step->mask = args.args[3];
		step->value = args.args[4];
		step->gate = -1;
		ret = of_address_to_resource(args.np, 0, &res);
		if (ret || step->type > 2 || step->delay_ms > 10000 ||
		    (step->reg & 3) || !step->mask ||
		    (step->value & ~step->mask) ||
		    (!ret && (u64)step->reg + 4 > resource_size(&res))) {
			of_node_put(args.np);
			return ret ?: -EINVAL;
		}
		step->map = syscon_node_to_regmap(args.np);
		of_node_put(args.np);
		if (IS_ERR(step->map))
			return PTR_ERR(step->map);
		/* The captured power sequences place the core/aux clock writes
		 * between PHY/domain operations. Preserve that ordering, while
		 * making these two gates visible to CCF instead of bypassing it.
		 */
		if (res.start == ORCA_IPA_AHB_BASE && !step->reg &&
		    step->type != 2) {
			if (step->mask == BIT(5))
				step->gate = ORCA_PCIE_CORE_GATE;
			else if (step->mask == BIT(6))
				step->gate = ORCA_PCIE_AUX_GATE;
		}
	}
	seq->count = count;
	return 0;
}

static int orca_pcie_gate(struct orca_pcie *pcie, unsigned int gate,
			  bool enable)
{
	int ret;

	if (pcie->gate_enabled[gate] == enable)
		return 0;
	if (enable) {
		ret = clk_prepare_enable(pcie->gates[gate]);
		if (ret)
			return ret;
	} else {
		clk_disable_unprepare(pcie->gates[gate]);
	}
	pcie->gate_enabled[gate] = enable;
	return 0;
}

static void orca_pcie_release_gates(void *data)
{
	struct orca_pcie *pcie = data;

	orca_pcie_gate(pcie, ORCA_PCIE_CORE_GATE, false);
	orca_pcie_gate(pcie, ORCA_PCIE_AUX_GATE, false);
}

static int orca_pcie_run(struct orca_pcie *pcie, unsigned int which)
{
	struct orca_pcie_sequence_data *seq = &pcie->sequence[which];
	unsigned int i;
	int ret;
	u32 value;

	for (i = 0; i < seq->count; i++) {
		struct orca_pcie_step *step = &seq->steps[i];

		if (step->gate >= 0) {
			ret = orca_pcie_gate(pcie, step->gate, !!step->value);
			goto step_done;
		}
		switch (step->type) {
		case 0:
			ret = regmap_update_bits(step->map, step->reg,
						 step->mask, step->value);
			break;
		case 1:
			/* BSP forced RMW write; keep atomic regmap locking. */
			ret = regmap_write_bits(step->map, step->reg,
						step->mask, step->value);
			break;
		case 2:
			ret = regmap_read_poll_timeout(
				step->map, step->reg, value,
				(value & step->mask) == step->value, 50, 10000);
			break;
		default:
			return -EINVAL;
		}
step_done:
		if (ret)
			return ret;
		if (step->delay_ms)
			msleep(step->delay_ms);
	}
	return 0;
}

static int orca_pcie_start_link(struct dw_pcie *pci)
{
	struct orca_pcie *pcie = to_orca(pci);
	unsigned int i;

	/* Exact BSP application-control write, not a fabricated PHY sequence. */
	dw_pcie_writel_dbi(pci, ORCA_PCIE_GEN_CTRL3,
			   ORCA_PCIE_LTSSM_EN | ORCA_PCIE_L1_AUXCLK_EN);
	/* Preserve endpoint-client hardware-owned MSI enables across resume. */
	if (pcie->msi_saved)
		for (i = 0; i < pci->pp.num_vectors / MAX_MSI_IRQS_PER_CTRL;
		     i++)
			dw_pcie_writel_dbi(pci,
					   PCIE_MSI_INTR0_ENABLE +
						   i * MSI_REG_CTRL_BLOCK_SIZE,
					   pcie->msi_enable[i]);
	return 0;
}

static void orca_pcie_stop_link(struct dw_pcie *pci)
{
	u32 value = dw_pcie_readl_dbi(pci, ORCA_PCIE_GEN_CTRL3);

	value &= ~ORCA_PCIE_LTSSM_EN;
	dw_pcie_writel_dbi(pci, ORCA_PCIE_GEN_CTRL3, value);
}

static int orca_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct orca_pcie *pcie = to_orca(pci);
	int ret;

	ret = orca_pcie_run(pcie,
			    pcie->initialized ? ORCA_RESUME : ORCA_STARTUP);
	if (!ret)
		ret = orca_pcie_run(pcie, ORCA_ASPM);
	if (ret) {
		int cleanup = orca_pcie_run(pcie, ORCA_SHUTDOWN);

		if (cleanup)
			dev_warn(pci->dev, "PCIe cleanup failed: %d\n",
				 cleanup);
		orca_pcie_release_gates(pcie);
		pcie->initialized = false;
		return ret;
	}
	pcie->initialized = true;
	pcie->powered = true;
	pcie->suspending = false;
	pcie->power_error = 0;
	return orca_pcie_start_link(pci);
}

static void orca_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct orca_pcie *pcie = to_orca(pci);
	unsigned int i;

	if (!pcie->powered)
		return;
	if (pcie->suspending) {
		for (i = 0; i < pp->num_vectors / MAX_MSI_IRQS_PER_CTRL; i++)
			pcie->msi_enable[i] = dw_pcie_readl_dbi(
				pci, PCIE_MSI_INTR0_ENABLE +
					     i * MSI_REG_CTRL_BLOCK_SIZE);
		pcie->msi_saved = true;
	}
	pcie->power_error = orca_pcie_run(
		pcie, pcie->suspending ? ORCA_SUSPEND : ORCA_SHUTDOWN);
	if (pcie->power_error)
		dev_err(pci->dev, "PCIe power sequence failed: %d\n",
			pcie->power_error);
	/* Suspend keeps AUX running for the captured resume/wake sequence.
	 * Full shutdown has no remaining link/wake consumer and releases it.
	 */
	if (!pcie->suspending) {
		orca_pcie_release_gates(pcie);
		pcie->initialized = false;
	}
	pcie->powered = false;
}

static void orca_pcie_pme_turn_off(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u32 value;

	value = dw_pcie_readl_dbi(pci, ORCA_PCIE_PM_CTRL);
	dw_pcie_writel_dbi(pci, ORCA_PCIE_PM_CTRL,
			   value | ORCA_PCIE_APP_CLK_PM_EN);
	value = dw_pcie_readl_dbi(pci, ORCA_PCIE_TX_MSG);
	dw_pcie_writel_dbi(pci, ORCA_PCIE_TX_MSG,
			   value | ORCA_PCIE_PME_TURN_OFF);
}

static const struct dw_pcie_host_ops orca_pcie_host_ops = {
	.init = orca_pcie_host_init,
	.deinit = orca_pcie_host_deinit,
	.pme_turn_off = orca_pcie_pme_turn_off,
};

static const struct dw_pcie_ops orca_pcie_ops = {
	.start_link = orca_pcie_start_link,
	.stop_link = orca_pcie_stop_link,
};

static int orca_pcie_msi_irq(struct platform_device *pdev)
{
	struct device_node *child;
	const char *label;
	int irq;

	irq = platform_get_irq_byname_optional(pdev, "msi");
	if (irq == -EPROBE_DEFER || irq > 0)
		return irq;
	/* Stock DT put the GIC MSI line in a labelled child. */
	for_each_available_child_of_node(pdev->dev.of_node, child) {
		if (of_property_read_string(child, "label", &label) ||
		    strcmp(label, "msi_int"))
			continue;
		irq = of_irq_get(child, 0);
		of_node_put(child);
		return irq;
	}
	return -ENXIO;
}

/* Only accept our own bridge; other hosts may have unrelated driver data. */
static struct orca_pcie *orca_from_pdev(struct platform_device *pdev)
{
	if (!pdev || !pdev->dev.driver ||
	    strcmp(pdev->dev.driver->name, "sprd-orca-pcie"))
		return NULL;
	return platform_get_drvdata(pdev);
}

int sprd_pcie_set_msi_ap_target(unsigned int irq, bool enable)
{
	struct msi_desc *desc = irq_get_msi_desc(irq);
	struct pci_host_bridge *bridge;
	struct irq_data *data;
	struct orca_pcie *pcie;
	struct dw_pcie_rp *pp;
	struct pci_dev *ep;
	unsigned long flags;
	u32 reg, value, bit;
	unsigned int group;

	if (!desc || !desc->dev || !dev_is_pci(desc->dev))
		return -EINVAL;
	ep = msi_desc_to_pci_dev(desc);
	bridge = pci_find_host_bridge(ep->bus);
	if (!bridge || !bridge->dev.parent ||
	    !dev_is_platform(bridge->dev.parent))
		return -ENODEV;
	pcie = orca_from_pdev(to_platform_device(bridge->dev.parent));
	if (!pcie || !READ_ONCE(pcie->powered))
		return -ENODEV;
	pp = &pcie->pci.pp;
	for (data = irq_get_irq_data(irq); data; data = data->parent_data)
		if (data->domain == pp->irq_domain)
			break;
	if (!data || data->hwirq >= pp->num_vectors)
		return -EINVAL;
	group = data->hwirq / MAX_MSI_IRQS_PER_CTRL;
	bit = BIT(data->hwirq % MAX_MSI_IRQS_PER_CTRL);
	reg = PCIE_MSI_INTR0_ENABLE + group * MSI_REG_CTRL_BLOCK_SIZE;
	raw_spin_lock_irqsave(&pp->lock, flags);
	value = dw_pcie_readl_dbi(&pcie->pci, reg);
	value = enable ? value | bit : value & ~bit;
	dw_pcie_writel_dbi(&pcie->pci, reg, value);
	if (pcie->msi_saved)
		pcie->msi_enable[group] = value;
	raw_spin_unlock_irqrestore(&pp->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_pcie_set_msi_ap_target);

void sprd_pcie_teardown_msi_irq(unsigned int irq)
{
	WARN_ON_ONCE(sprd_pcie_set_msi_ap_target(irq, false));
}
EXPORT_SYMBOL_GPL(sprd_pcie_teardown_msi_irq);

int sprd_pcie_unconfigure_device(struct platform_device *pdev)
{
	struct orca_pcie *pcie = orca_from_pdev(pdev);
	struct pci_dev *child, *tmp;
	int ret = 0;

	if (!pcie)
		return -ENODEV;
	pci_lock_rescan_remove();
	mutex_lock(&pcie->lifecycle_lock);
	if (pcie->manually_offline)
		goto out;
#if IS_ENABLED(CONFIG_SPRD_PCIE_EP_DEVICE)
	ret = sprd_ep_dev_prepare_remove(&pdev->dev);
	if (ret)
		goto out;
#endif
	/* No active registered consumer or BAR lease may survive this point. */
	list_for_each_entry_safe(child, tmp, &pcie->pci.pp.bridge->bus->devices,
				 bus_list)
		pci_stop_and_remove_bus_device(child);
	pcie->suspending = true;
	ret = dw_pcie_suspend_noirq(&pcie->pci);
	if (!ret)
		ret = pcie->power_error;
	/* Even partial power-down cannot be advertised as usable. */
	pcie->manually_offline = true;
out:
	mutex_unlock(&pcie->lifecycle_lock);
	pci_unlock_rescan_remove();
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pcie_unconfigure_device);

int sprd_pcie_configure_device(struct platform_device *pdev)
{
	struct orca_pcie *pcie = orca_from_pdev(pdev);
	int ret;

	if (!pcie)
		return -ENODEV;
	pci_lock_rescan_remove();
	mutex_lock(&pcie->lifecycle_lock);
	if (pcie->power_error) {
		ret = pcie->power_error;
		goto out;
	}
	ret = dw_pcie_resume_noirq(&pcie->pci);
	if (ret)
		goto out;
	if (!dw_pcie_link_up(&pcie->pci)) {
		ret = -ENOLINK;
		goto out;
	}
	pci_rescan_bus(pcie->pci.pp.bridge->bus);
	pcie->manually_offline = false;
out:
	mutex_unlock(&pcie->lifecycle_lock);
	pci_unlock_rescan_remove();
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pcie_configure_device);

static int orca_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct orca_pcie *pcie;
	unsigned int i;
	int ret;
	u32 vectors;

	if (of_property_read_bool(dev->of_node, "no-pcie"))
		return -ENODEV;
	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;
	mutex_init(&pcie->lifecycle_lock);
	pcie->pci.dev = dev;
	pcie->pci.ops = &orca_pcie_ops;
	pcie->pci.pp.ops = &orca_pcie_host_ops;
	pcie->gates[ORCA_PCIE_CORE_GATE] = devm_clk_get(dev, "core");
	if (IS_ERR(pcie->gates[ORCA_PCIE_CORE_GATE]))
		return dev_err_probe(dev,
				     PTR_ERR(pcie->gates[ORCA_PCIE_CORE_GATE]),
				     "missing PCIe core clock\n");
	pcie->gates[ORCA_PCIE_AUX_GATE] = devm_clk_get(dev, "aux");
	if (IS_ERR(pcie->gates[ORCA_PCIE_AUX_GATE]))
		return dev_err_probe(dev,
				     PTR_ERR(pcie->gates[ORCA_PCIE_AUX_GATE]),
				     "missing PCIe auxiliary clock\n");
	ret = devm_add_action_or_reset(dev, orca_pcie_release_gates, pcie);
	if (ret)
		return ret;
	for (i = 0; i < ORCA_SEQUENCE_COUNT; i++) {
		ret = orca_pcie_parse_sequence(pcie, i);
		if (ret)
			return dev_err_probe(dev, ret, "invalid %s\n",
					     orca_sequence_names[i]);
	}
	ret = of_property_read_u32(dev->of_node, "num-vectors", &vectors);
	if (ret || !vectors || vectors > MAX_MSI_IRQS ||
	    vectors % MAX_MSI_IRQS_PER_CTRL)
		return dev_err_probe(dev, -EINVAL,
				     "invalid MSI vector count\n");
	pcie->pci.pp.num_vectors = vectors;
	ret = orca_pcie_msi_irq(pdev);
	if (ret <= 0)
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "missing MSI interrupt\n");
	pcie->pci.pp.msi_irq[0] = ret;
	platform_set_drvdata(pdev, pcie);
	return dw_pcie_host_init(&pcie->pci.pp);
}

static int orca_pcie_suspend_noirq(struct device *dev)
{
	struct orca_pcie *pcie = dev_get_drvdata(dev);
	int ret;

	if (pcie->manually_offline)
		return 0;
	pcie->suspending = true;
	ret = dw_pcie_suspend_noirq(&pcie->pci);
	return ret ?: pcie->power_error;
}

static int orca_pcie_resume_noirq(struct device *dev)
{
	struct orca_pcie *pcie = dev_get_drvdata(dev);

	if (pcie->manually_offline)
		return 0;
	return dw_pcie_resume_noirq(&pcie->pci);
}

static void orca_pcie_shutdown(struct platform_device *pdev)
{
	struct orca_pcie *pcie = platform_get_drvdata(pdev);

	if (!pcie->powered) {
		/* A manually suspended controller still owns its AUX reference. */
		orca_pcie_release_gates(pcie);
		return;
	}
	pcie->suspending = false;
	orca_pcie_stop_link(&pcie->pci);
	orca_pcie_host_deinit(&pcie->pci.pp);
}

static const struct dev_pm_ops orca_pcie_pm_ops = { NOIRQ_SYSTEM_SLEEP_PM_OPS(
	orca_pcie_suspend_noirq, orca_pcie_resume_noirq) };

static const struct of_device_id orca_pcie_of_match[] = {
	{ .compatible = "sprd,orca-pcie" },
	{}
};

static struct platform_driver orca_pcie_driver = {
	.probe = orca_pcie_probe,
	.shutdown = orca_pcie_shutdown,
	.driver = {
		.name = "sprd-orca-pcie",
		.of_match_table = orca_pcie_of_match,
		.pm = pm_sleep_ptr(&orca_pcie_pm_ops),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(orca_pcie_driver);
