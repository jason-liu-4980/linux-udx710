// SPDX-License-Identifier: GPL-2.0-only
/*
 * AP-side Orca endpoint client. Derived from GPL sprd_pcie_ep_device.c.
 * Copyright (C) 2018-2019 Spreadtrum Co., Ltd.
 * Mainline lifecycle/locking adaptation; does not fabricate a SIPA producer.
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/pci.h>
#include <linux/pcie-rc-sprd.h>
#include <linux/rcupdate.h>
#include <linux/soc/sprd/sprd_pcie_ep_device.h>

#define ORCA_VECTORS 32
#define ORCA_IRQ_BASE 16
#define ORCA_HW_VECTORS 4
#define ORCA_CFG_BAR 4
#define ORCA_ATU_BASE 0x10000
#define ORCA_ATU_VIEWPORT 0x900
#define ORCA_ATU_CR1 0x904
#define ORCA_ATU_CR2 0x908
#define ORCA_ATU_TARGET_LO 0x918
#define ORCA_ATU_TARGET_HI 0x91c
#define ORCA_ATU_ENABLE BIT(31)
#define ORCA_ATU_BAR_MATCH BIT(30)
#define ORCA_DOORBELL_STATUS 0x14

struct orca_irq_callback {
	irq_handler_t handler;
	void *data;
};

struct orca_bar {
	bool used;
	int ipa_type;
	void __iomem *mapping, *address;
	phys_addr_t ipa_address;
};

struct orca_endpoint;
struct orca_irq_context {
	struct orca_endpoint *endpoint;
	int index;
};

struct orca_endpoint {
	struct pci_dev *pdev;
	void __iomem *cfg;
	struct orca_bar bar[4];
	bool unrolled, dying, suspended, ready;
	struct orca_irq_context irq_context[PCIE_EP_MAX_IRQ];
	unsigned int requested_irqs, redirected_irqs;
};

/* Lock order: notify_lock -> callback_lock -> endpoint_lock -> doorbell_lock. */
static DEFINE_MUTEX(endpoint_lock);
static DEFINE_MUTEX(notify_lock);
static DEFINE_MUTEX(callback_lock);
static DEFINE_SPINLOCK(doorbell_lock);
static struct orca_endpoint *endpoint;
static struct orca_irq_callback __rcu *callbacks[PCIE_EP_MAX_IRQ];
static void (*endpoint_notify)(int, void *);
static void *notify_data;
static bool probe_notified;

static void orca_notify(bool present)
{
	mutex_lock(&notify_lock);
	if (endpoint_notify && probe_notified != present) {
		probe_notified = present;
		endpoint_notify(present ? PCIE_EP_PROBE : PCIE_EP_REMOVE,
				notify_data);
	}
	mutex_unlock(&notify_lock);
}

int sprd_ep_dev_register_notify(int ep, void (*notify)(int, void *), void *data)
{
	bool present;
	int ret = 0;

	if (ep != PCIE_EP_MODEM || !notify)
		return -EINVAL;
	mutex_lock(&notify_lock);
	if (endpoint_notify) {
		ret = -EBUSY;
		goto out;
	}
	mutex_lock(&endpoint_lock);
	if (endpoint && (endpoint->dying || endpoint->suspended)) {
		mutex_unlock(&endpoint_lock);
		ret = -ENODEV;
		goto out;
	}
	endpoint_notify = notify;
	notify_data = data;
	present = endpoint && !endpoint->dying && !endpoint->suspended;
	mutex_unlock(&endpoint_lock);
	probe_notified = present;
	if (present)
		notify(PCIE_EP_PROBE, data);
out:
	mutex_unlock(&notify_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ep_dev_register_notify);

int sprd_ep_dev_unregister_notify(int ep)
{
	if (ep != PCIE_EP_MODEM)
		return -EINVAL;
	mutex_lock(&notify_lock);
	endpoint_notify = NULL;
	notify_data = NULL;
	probe_notified = false;
	mutex_unlock(&notify_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_ep_dev_unregister_notify);

int sprd_ep_dev_register_irq_handler(int ep, int irq, irq_handler_t handler,
				     void *data)
{
	struct orca_irq_callback *cb;
	int ret = 0;

	if (ep != PCIE_EP_MODEM || irq < 0 || irq >= PCIE_EP_MAX_IRQ ||
	    !handler)
		return -EINVAL;
	cb = kmalloc_obj(*cb, GFP_KERNEL);
	if (!cb)
		return -ENOMEM;
	cb->handler = handler;
	cb->data = data;
	mutex_lock(&callback_lock);
	mutex_lock(&endpoint_lock);
	if (endpoint && (endpoint->dying || endpoint->suspended))
		ret = -ENODEV;
	else if (rcu_access_pointer(callbacks[irq]))
		ret = -EBUSY;
	else
		rcu_assign_pointer(callbacks[irq], cb);
	mutex_unlock(&endpoint_lock);
	mutex_unlock(&callback_lock);
	if (ret)
		kfree(cb);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ep_dev_register_irq_handler);

int sprd_ep_dev_unregister_irq_handler(int ep, int irq)
{
	struct orca_irq_callback *old;

	if (ep != PCIE_EP_MODEM || irq < 0 || irq >= PCIE_EP_MAX_IRQ)
		return -EINVAL;
	mutex_lock(&callback_lock);
	old = rcu_replace_pointer(callbacks[irq], NULL,
				  lockdep_is_held(&callback_lock));
	mutex_unlock(&callback_lock);
	synchronize_rcu();
	kfree(old);
	return 0;
}
EXPORT_SYMBOL_GPL(sprd_ep_dev_unregister_irq_handler);

static irqreturn_t orca_endpoint_irq(int irq, void *data)
{
	struct orca_irq_callback *cb;
	struct orca_irq_context *context = data;
	int index = context->index;
	irqreturn_t ret = IRQ_HANDLED;

	if (!READ_ONCE(context->endpoint->ready))
		return IRQ_HANDLED;
	rcu_read_lock();
	cb = rcu_dereference(callbacks[index]);
	if (cb)
		ret = cb->handler(irq, cb->data);
	rcu_read_unlock();
	return ret;
}

int sprd_ep_dev_raise_irq(int ep, int irq)
{
	struct orca_endpoint *e;
	unsigned long flags;
	int ret = 0;
	u32 value;

	if (ep != PCIE_EP_MODEM || irq < 0 || irq >= 32)
		return -EINVAL;
	spin_lock_irqsave(&doorbell_lock, flags);
	e = endpoint;
	if (!e || e->dying || e->suspended) {
		ret = -ENODEV;
		goto out;
	}
	value = readl(e->cfg + ORCA_DOORBELL_STATUS);
	writel(value | BIT(irq), e->cfg + ORCA_DOORBELL_STATUS);
out:
	spin_unlock_irqrestore(&doorbell_lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ep_dev_raise_irq);

/* endpoint_lock covers selection, viewport programming AND the enable poll. */
static void __iomem *orca_atu_ctrl(struct orca_endpoint *e, int bar)
{
	void __iomem *base = e->cfg + ORCA_ATU_BASE;

	if (e->unrolled)
		return base + (bar << 9) + BIT(8) + 4;
	writel(BIT(31) | bar, base + ORCA_ATU_VIEWPORT);
	return base + ORCA_ATU_CR2;
}

static void orca_disable_bar(struct orca_endpoint *e, int bar)
{
	void __iomem *ctrl = orca_atu_ctrl(e, bar);

	writel(readl(ctrl) & ~ORCA_ATU_ENABLE, ctrl);
	readl(ctrl);
}

static int orca_enable_bar(struct orca_endpoint *e, int bar, phys_addr_t target)
{
	void __iomem *base = e->cfg + ORCA_ATU_BASE, *ctrl;
	int retry;

	ctrl = orca_atu_ctrl(e, bar);
	if (e->unrolled) {
		base += (bar << 9) + BIT(8);
		writel(lower_32_bits(target), base + 0x14);
		writel(upper_32_bits(target), base + 0x18);
		writel(0, base);
	} else {
		writel(lower_32_bits(target), base + ORCA_ATU_TARGET_LO);
		writel(upper_32_bits(target), base + ORCA_ATU_TARGET_HI);
		writel(0, base + ORCA_ATU_CR1);
	}
	writel(ORCA_ATU_ENABLE | ORCA_ATU_BAR_MATCH | (bar << 8), ctrl);
	for (retry = 0; retry < 5; retry++) {
		if (readl(ctrl) & ORCA_ATU_ENABLE)
			return 0;
		usleep_range(9000, 10000);
	}
	orca_disable_bar(e, bar);
	return -ETIMEDOUT;
}

static int orca_select_bar(struct orca_endpoint *e, int type)
{
	bool hardware = IS_ENABLED(CONFIG_SPRD_IPA_PCIE_WORKROUND);
	int first = hardware ? 2 : 0, bar;

	if (hardware && type >= 0) {
		if (e->bar[type].used)
			return -EBUSY;
		return type;
	}
	for (bar = first; bar < ORCA_CFG_BAR; bar++)
		if (!e->bar[bar].used &&
		    (pci_resource_flags(e->pdev, bar) & IORESOURCE_MEM) &&
		    pci_resource_len(e->pdev, bar))
			return bar;
	return -ENOSPC;
}

static int orca_map_bar(struct orca_endpoint *e, phys_addr_t target,
			size_t size, int type)
{
	resource_size_t len, offset, base, span, end;
	struct orca_bar *lease;
	int bar, ret;

	if (!size)
		return -EINVAL;
	bar = orca_select_bar(e, type);
	if (bar < 0)
		return bar;
	len = pci_resource_len(e->pdev, bar);
	if (!(pci_resource_flags(e->pdev, bar) & IORESOURCE_MEM) || !len ||
	    !is_power_of_2(len) || len > SIZE_MAX)
		return -EINVAL;
	base = target & ~(len - 1);
	offset = target - base;
	if (check_add_overflow((resource_size_t)target,
			       (resource_size_t)size - 1, &end) ||
	    check_add_overflow(offset, (resource_size_t)size, &span) ||
	    span > len || span > SIZE_MAX - (PAGE_SIZE - 1))
		return -ERANGE;
	span = PAGE_ALIGN(span);
	if (span > len)
		return -ERANGE;
	lease = &e->bar[bar];
	if (type < 0) {
		lease->mapping =
			ioremap(pci_resource_start(e->pdev, bar), span);
		if (!lease->mapping)
			return -ENOMEM;
	}
	ret = orca_enable_bar(e, bar, base);
	if (ret) {
		if (lease->mapping)
			iounmap(lease->mapping);
		memset(lease, 0, sizeof(*lease));
		return ret;
	}
	lease->used = true;
	lease->ipa_type = type;
	if (type < 0)
		lease->address = lease->mapping + offset;
	else
		lease->ipa_address = pci_resource_start(e->pdev, bar) + offset;
	return bar;
}

static void orca_unmap_bar(struct orca_endpoint *e, int bar)
{
	struct orca_bar *lease = &e->bar[bar];

	orca_disable_bar(e, bar);
	if (lease->mapping)
		iounmap(lease->mapping);
	memset(lease, 0, sizeof(*lease));
}

void __iomem *sprd_ep_map_memory(int ep, phys_addr_t target, size_t size)
{
	void __iomem *result = NULL;
	int bar;

	if (ep != PCIE_EP_MODEM)
		return NULL;
	mutex_lock(&endpoint_lock);
	if (endpoint && !endpoint->dying && !endpoint->suspended) {
		bar = orca_map_bar(endpoint, target, size, -1);
		if (bar >= 0)
			result = endpoint->bar[bar].address;
	}
	mutex_unlock(&endpoint_lock);
	return result;
}
EXPORT_SYMBOL_GPL(sprd_ep_map_memory);

void sprd_ep_unmap_memory(int ep, const void __iomem *address)
{
	int bar;

	if (ep != PCIE_EP_MODEM || !address)
		return;
	mutex_lock(&endpoint_lock);
	if (endpoint)
		for (bar = 0; bar < ORCA_CFG_BAR; bar++)
			if (endpoint->bar[bar].used &&
			    endpoint->bar[bar].ipa_type < 0 &&
			    endpoint->bar[bar].address == address) {
				orca_unmap_bar(endpoint, bar);
				break;
			}
	mutex_unlock(&endpoint_lock);
}
EXPORT_SYMBOL_GPL(sprd_ep_unmap_memory);

phys_addr_t sprd_ep_ipa_map(int type, phys_addr_t target, size_t size)
{
	phys_addr_t result = 0;
	int bar;

	if (type != PCIE_IPA_TYPE_MEM && type != PCIE_IPA_TYPE_REG)
		return 0;
	mutex_lock(&endpoint_lock);
	if (endpoint && !endpoint->dying && !endpoint->suspended) {
		bar = orca_map_bar(endpoint, target, size, type);
		if (bar >= 0)
			result = endpoint->bar[bar].ipa_address;
	}
	mutex_unlock(&endpoint_lock);
	return result;
}
EXPORT_SYMBOL_GPL(sprd_ep_ipa_map);

int sprd_ep_ipa_unmap(int type, phys_addr_t address)
{
	int bar, ret = -ENOENT;

	if ((type != PCIE_IPA_TYPE_MEM && type != PCIE_IPA_TYPE_REG) ||
	    !address)
		return -EINVAL;
	mutex_lock(&endpoint_lock);
	if (endpoint)
		for (bar = 0; bar < ORCA_CFG_BAR; bar++)
			if (endpoint->bar[bar].used &&
			    endpoint->bar[bar].ipa_type == type &&
			    endpoint->bar[bar].ipa_address == address) {
				orca_unmap_bar(endpoint, bar);
				ret = 0;
				break;
			}
	mutex_unlock(&endpoint_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ep_ipa_unmap);

/* Dynamic RC removal is refused until clients explicitly release ownership. */
int sprd_ep_dev_prepare_remove(struct device *host)
{
	struct pci_host_bridge *bridge;
	unsigned long flags;
	int bar, ret = 0;

	mutex_lock(&notify_lock);
	mutex_lock(&callback_lock);
	mutex_lock(&endpoint_lock);
	if (!endpoint)
		goto out;
	bridge = pci_find_host_bridge(endpoint->pdev->bus);
	if (bridge->dev.parent != host)
		goto out;
	if (endpoint_notify || rcu_access_pointer(callbacks[0]) ||
	    rcu_access_pointer(callbacks[1])) {
		ret = -EBUSY;
		goto out;
	}
	for (bar = 0; bar < ORCA_CFG_BAR; bar++)
		if (endpoint->bar[bar].used) {
			ret = -EBUSY;
			goto out;
		}
	spin_lock_irqsave(&doorbell_lock, flags);
	endpoint->dying = true;
	spin_unlock_irqrestore(&doorbell_lock, flags);
out:
	mutex_unlock(&endpoint_lock);
	mutex_unlock(&callback_lock);
	mutex_unlock(&notify_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_ep_dev_prepare_remove);

static void orca_free_irqs(struct orca_endpoint *e)
{
	unsigned int i;

	for (i = 0; i < e->redirected_irqs; i++)
		sprd_pcie_set_msi_ap_target(pci_irq_vector(e->pdev, i), true);
	for (i = 0; i < e->requested_irqs; i++)
		free_irq(pci_irq_vector(e->pdev, ORCA_IRQ_BASE + i),
			 &e->irq_context[i]);
	pci_free_irq_vectors(e->pdev);
}

static int orca_client_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	struct orca_endpoint *e;
	unsigned long flags;
	u32 viewport;
	int ret, i;

	if (pci_is_bridge(pdev))
		return -ENODEV;
	e = kzalloc_obj(*e, GFP_KERNEL);
	if (!e)
		return -ENOMEM;
	e->pdev = pdev;
	ret = pci_enable_device_mem(pdev);
	if (ret)
		goto free;
	ret = pci_request_regions(pdev, "sprd-orca-client");
	if (ret)
		goto disable;
	if (!(pci_resource_flags(pdev, ORCA_CFG_BAR) & IORESOURCE_MEM) ||
	    pci_resource_len(pdev, ORCA_CFG_BAR) < ORCA_ATU_BASE + 0x920) {
		ret = -EINVAL;
		goto regions;
	}
	e->cfg = pci_ioremap_bar(pdev, ORCA_CFG_BAR);
	if (!e->cfg) {
		ret = -ENOMEM;
		goto regions;
	}
	ret = pci_read_config_dword(pdev, ORCA_ATU_VIEWPORT, &viewport);
	if (ret) {
		ret = pcibios_err_to_errno(ret);
		goto unmap;
	}
	e->unrolled = viewport == U32_MAX;
	ret = pci_alloc_irq_vectors(pdev, ORCA_VECTORS, ORCA_VECTORS,
				    PCI_IRQ_MSI);
	if (ret < 0)
		goto unmap;
	for (i = 0; i < PCIE_EP_MAX_IRQ; i++) {
		e->irq_context[i].index = i;
		e->irq_context[i].endpoint = e;
		ret = request_irq(pci_irq_vector(pdev, ORCA_IRQ_BASE + i),
				  orca_endpoint_irq, 0, "sprd-orca-client",
				  &e->irq_context[i]);
		if (ret)
			goto irqs;
		e->requested_irqs++;
	}
	if (IS_ENABLED(CONFIG_SPRD_IPA_PCIE_WORKROUND)) {
		/* Exact BSP fixed BAR and MSI indices; opt-in, not board-inferred. */
		for (i = 0; i < 2; i++)
			if (!(pci_resource_flags(pdev, i) & IORESOURCE_MEM) ||
			    !pci_resource_len(pdev, i)) {
				ret = -ENODEV;
				goto irqs;
			}
		for (i = 0; i < ORCA_HW_VECTORS; i++) {
			ret = sprd_pcie_set_msi_ap_target(
				pci_irq_vector(pdev, i), false);
			if (ret)
				goto irqs;
			e->redirected_irqs++;
		}
	}
	mutex_lock(&endpoint_lock);
	if (endpoint) {
		mutex_unlock(&endpoint_lock);
		ret = -EBUSY;
		goto irqs;
	}
	/* BSP's status initialisation; DOOR_BELL_ENABLE is not invented here. */
	writel(U32_MAX, e->cfg + ORCA_DOORBELL_STATUS);
	pci_set_master(pdev);
	pci_set_drvdata(pdev, e);
	spin_lock_irqsave(&doorbell_lock, flags);
	e->ready = true;
	endpoint = e;
	spin_unlock_irqrestore(&doorbell_lock, flags);
	mutex_unlock(&endpoint_lock);
	orca_notify(true);
	return 0;
irqs:
	orca_free_irqs(e);
unmap:
	pci_iounmap(pdev, e->cfg);
regions:
	pci_release_regions(pdev);
disable:
	pci_disable_device(pdev);
free:
	kfree(e);
	return ret;
}

static void orca_client_remove(struct pci_dev *pdev)
{
	struct orca_endpoint *e = pci_get_drvdata(pdev);
	unsigned long flags;
	int bar;

	mutex_lock(&endpoint_lock);
	spin_lock_irqsave(&doorbell_lock, flags);
	e->dying = true;
	spin_unlock_irqrestore(&doorbell_lock, flags);
	mutex_unlock(&endpoint_lock);
	/* Consumers still have valid BARs while synchronously quiescing. */
	orca_notify(false);
	WRITE_ONCE(e->ready, false);
	pci_clear_master(pdev);
	orca_free_irqs(e);
	mutex_lock(&endpoint_lock);
	for (bar = 0; bar < ORCA_CFG_BAR; bar++)
		if (e->bar[bar].used) {
			dev_warn(&pdev->dev,
				 "BAR%d lease survived REMOVE notification\n",
				 bar);
			orca_unmap_bar(e, bar);
		}
	spin_lock_irqsave(&doorbell_lock, flags);
	endpoint = NULL;
	spin_unlock_irqrestore(&doorbell_lock, flags);
	pci_iounmap(pdev, e->cfg);
	mutex_unlock(&endpoint_lock);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	kfree(e);
}

static int orca_client_suspend(struct device *dev)
{
	struct orca_endpoint *e = pci_get_drvdata(to_pci_dev(dev));
	unsigned long flags;
	int bar, ret = 0;

	mutex_lock(&notify_lock);
	mutex_lock(&callback_lock);
	mutex_lock(&endpoint_lock);
	/* Legacy callback ABI has no sleep/quiesce acknowledgment. */
	if (endpoint_notify || rcu_access_pointer(callbacks[0]) ||
	    rcu_access_pointer(callbacks[1])) {
		ret = -EBUSY;
		goto out;
	}
	/* Do not invalidate live AP/IPA apertures without consumer quiescence. */
	for (bar = 0; bar < ORCA_CFG_BAR; bar++)
		if (e->bar[bar].used) {
			ret = -EBUSY;
			goto out;
		}
	spin_lock_irqsave(&doorbell_lock, flags);
	e->suspended = true;
	spin_unlock_irqrestore(&doorbell_lock, flags);
out:
	mutex_unlock(&endpoint_lock);
	mutex_unlock(&callback_lock);
	mutex_unlock(&notify_lock);
	return ret;
}

static int orca_client_resume(struct device *dev)
{
	struct orca_endpoint *e = pci_get_drvdata(to_pci_dev(dev));
	unsigned long flags;

	mutex_lock(&endpoint_lock);
	spin_lock_irqsave(&doorbell_lock, flags);
	e->suspended = false;
	spin_unlock_irqrestore(&doorbell_lock, flags);
	mutex_unlock(&endpoint_lock);
	return 0;
}

static const struct dev_pm_ops orca_client_pm = { SYSTEM_SLEEP_PM_OPS(
	orca_client_suspend, orca_client_resume) };
static const struct pci_device_id orca_client_ids[] = { { PCI_DEVICE(0x16c3,
								     0xabcd) },
							{} };
MODULE_DEVICE_TABLE(pci, orca_client_ids);
static struct pci_driver orca_client_driver = {
	.name = "sprd-orca-client",
	.id_table = orca_client_ids,
	.probe = orca_client_probe,
	.remove = orca_client_remove,
	.driver.pm = pm_sleep_ptr(&orca_client_pm),
	/* Legacy BAR consumers cannot acknowledge removal or drain accesses. */
	.driver.suppress_bind_attrs = true,
};
module_pci_driver(orca_client_driver);
MODULE_DESCRIPTION("AP-side Spreadtrum Orca PCI endpoint client");
MODULE_LICENSE("GPL");
