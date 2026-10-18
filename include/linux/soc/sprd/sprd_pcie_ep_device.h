/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2019 Spreadtrum Co., Ltd. */
#ifndef __SPRD_PCIE_EP_DEVICE_H
#define __SPRD_PCIE_EP_DEVICE_H
#include <linux/interrupt.h>
#include <linux/types.h>
enum { PCIE_EP_SIPC_IRQ, PCIE_EP_OTHER_IRQ, PCIE_EP_MAX_IRQ };
enum { PCIE_EP_MODEM, PCIE_EP_NR };
enum { PCIE_EP_PROBE, PCIE_EP_REMOVE };
enum { PCIE_IPA_TYPE_MEM, PCIE_IPA_TYPE_REG };
struct device;
int sprd_ep_dev_prepare_remove(struct device *host);
#define MINI_REGION_SIZE 0x10000
/*
 * Notification callbacks may sleep/map/unmap. REMOVE must stop endpoint DMA,
 * drain work and release all BAR leases before returning. Registration APIs
 * must not be called recursively from these callbacks. No RM grant is implied.
 * Mapping and registration APIs are process-context only; raise_irq is atomic.
 * IRQ handlers must not unregister themselves. They run under RCU read lock.
 */
int sprd_ep_dev_register_notify(int ep, void (*notify)(int, void *),
				void *data);
int sprd_ep_dev_unregister_notify(int ep);
int sprd_ep_dev_register_irq_handler(int ep, int irq, irq_handler_t handler,
				     void *data);
int sprd_ep_dev_unregister_irq_handler(int ep, int irq);
int sprd_ep_dev_raise_irq(int ep, int irq);
void __iomem *sprd_ep_map_memory(int ep, phys_addr_t addr, size_t size);
void sprd_ep_unmap_memory(int ep, const void __iomem *addr);
phys_addr_t sprd_ep_ipa_map(int type, phys_addr_t target, size_t size);
int sprd_ep_ipa_unmap(int type, phys_addr_t cpu_addr);
#endif
