/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_USB_SPRD_PAMU3_H
#define _LINUX_USB_SPRD_PAMU3_H

#include <linux/types.h>

struct device;
struct usb_ep;

/* DWC3 owns these coherent allocations. Valid only between attach/detach. */
struct dwc3_pamu3_resources {
	struct device *dma_dev;
	phys_addr_t regs_phys;
	dma_addr_t event_dma[2]; /* event 1 = ep3 IN, event 2 = ep2 OUT */
	u32 event_size[2];
	u32 generation;
};

/* Process context. attach pins runtime PM; caller must establish a device link. */
int dwc3_pamu3_attach(struct device *controller, struct device *owner,
		      struct dwc3_pamu3_resources *resources);
int dwc3_pamu3_detach(struct device *controller, struct device *owner);
/* Claim both disabled, request-free endpoints before the USB function enables
 * them. Only physical ep2 OUT and ep3 IN are accepted. Software queue is then
 * prohibited. Release is legal only after PAM and SIPA are stopped/drained.
 */
int dwc3_pamu3_claim(struct device *controller, struct device *owner,
		     struct usb_ep *out, struct usb_ep *in);
int dwc3_pamu3_drain(struct device *controller, struct device *owner);
int dwc3_pamu3_release(struct device *controller, struct device *owner);

enum sprd_pamu3_protocol {
	SPRD_PAMU3_RNDIS,
	SPRD_PAMU3_MBIM_NTB16,
	SPRD_PAMU3_NCM_NTB16,
};

struct sprd_pamu3_config {
	enum sprd_pamu3_protocol protocol;
	u32 max_transfer, max_packets, netid;
	u32 nth_signature, ndp_signature;
	u8 host_mac[6], dev_mac[6];
};

/* Exclusive function lease; put drops the device reference even on error. */
struct device *sprd_pamu3_get(struct device *controller);
int sprd_pamu3_put(struct device *dev);
/* Before controller pullup, acquire/wait/release a real network resource. */
int sprd_pamu3_wait_ready(struct device *dev);
int sprd_pamu3_arm(struct device *dev, struct usb_ep *out, struct usb_ep *in);
int sprd_pamu3_pause(struct device *dev);
void sprd_pamu3_quiesce_atomic(struct device *dev);
/* Preparation allocates and programs disabled hardware; it is not a start. */
int sprd_pamu3_prepare(struct device *dev);
int sprd_pamu3_start(struct device *dev,
		     const struct sprd_pamu3_config *config);
int sprd_pamu3_stop(struct device *dev);

#endif
