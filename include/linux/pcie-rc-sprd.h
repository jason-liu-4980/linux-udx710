/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _PCIE_RC_SPRD_H
#define _PCIE_RC_SPRD_H
#include <linux/types.h>
struct platform_device;
/* Process context only; do not call recursively from PCI probe/remove callbacks. */
int sprd_pcie_configure_device(struct platform_device *pdev);
int sprd_pcie_unconfigure_device(struct platform_device *pdev);
/* Redirecting hardware MSI requires retaining the allocated Linux vector. */
int sprd_pcie_set_msi_ap_target(unsigned int irq, bool enable);
void sprd_pcie_teardown_msi_irq(unsigned int irq);
#endif
