// SPDX-License-Identifier: GPL-2.0-only
/* UDX710 PAMU3 resource/lifetime port. No vendor usb_phy pseudo-PHY ABI. */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/usb/gadget.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sipa.h>
#include <linux/usb/sprd-pamu3.h>
#include "sprd-pamu3.h"

#define PAMU3_TX_BYTES (sizeof(struct pamu3_dwc3_trb) * PAMU3_TX_TRB_NUM * 2)
#define PAMU3_RX_BYTES (sizeof(struct pamu3_dwc3_trb) * PAMU3_RX_TRB_NUM)
#define PAMU3_DATA_BYTES (PAMU3_RX_TRBBUF_SIZE * PAMU3_RX_TRBBUF_NUM)

/* Callers use both loop-index and direction expressions. These must have
 * exactly the same register offsets as literal event-buffer indices.
 */

static LIST_HEAD(pamu3_devices);
static DEFINE_MUTEX(pamu3_devices_lock);

struct sprd_pamu3 {
	struct device *dev;
	struct device *controller;
	struct device *sipa;
	struct mutex lock;
	spinlock_t hw_lock;
	bool abort;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct dwc3_pamu3_resources dwc;
	struct sipa_to_pam_info fifo;
	void *tx_trbs, *rx_trbs, *header, *data;
	dma_addr_t tx_dma, rx_dma, header_dma, data_dma;
	u64 common_dma_mask;
	bool clock_on, reset_held, attached, prepared, claimed;
	bool rm_registered, dependency, requested, connected, stopping, running;
	bool function_owned, foreign_active;
	int rm_error;
	struct completion grant;
	struct sipa_rm_register_params rm;
	struct usb_ep *out, *in;
	struct usb_endpoint_descriptor out_desc, in_desc;
	struct usb_ss_ep_comp_descriptor out_comp, in_comp;
	struct list_head node;
};

static bool pamu3_address_valid(u64 addr, u64 bytes)
{
	return bytes && addr <= DMA_BIT_MASK(40) &&
	       bytes - 1 <= DMA_BIT_MASK(40) - addr;
}

/* Equal masks are insufficient: both masters must interpret DMA addresses in
 * the same translation domain. Different domains/ranges are refused.
 */
static bool pamu3_same_dma_domain(struct device *a, struct device *b)
{
	const struct bus_dma_region *ra = a->dma_range_map,
				    *rb = b->dma_range_map;

	/* The captured platform uses direct identity DMA. Shared IOVA domains
	 * alone do not map PAM's accesses to the peers' MMIO apertures.
	 */
	if (iommu_get_domain_for_dev(a) || iommu_get_domain_for_dev(b) ||
	    get_dma_ops(a) || get_dma_ops(b) ||
	    dev_is_dma_coherent(a) != dev_is_dma_coherent(b))
		return false;
	if (!ra || !rb)
		return !ra && !rb;
	while (ra->size && rb->size) {
		if (ra->cpu_start != ra->dma_start ||
		    rb->cpu_start != rb->dma_start ||
		    ra->cpu_start != rb->cpu_start ||
		    ra->dma_start != rb->dma_start || ra->size != rb->size)
			return false;
		ra++;
		rb++;
	}
	return !ra->size && !rb->size;
}

static void pamu3_write_addr(struct sprd_pamu3 *pam, u32 offset, u64 addr)
{
	writel(lower_32_bits(addr), pam->base + offset);
	writel(upper_32_bits(addr), pam->base + offset + 4);
}

static void pamu3_free_buffers(struct sprd_pamu3 *pam)
{
	if (pam->data)
		dma_free_coherent(pam->dev, PAMU3_DATA_BYTES, pam->data,
				  pam->data_dma);
	if (pam->header)
		dma_free_coherent(pam->dev, PAMU3_RNDIS_HEADER_SIZE,
				  pam->header, pam->header_dma);
	if (pam->rx_trbs)
		dma_free_coherent(pam->dev, PAMU3_RX_BYTES, pam->rx_trbs,
				  pam->rx_dma);
	if (pam->tx_trbs)
		dma_free_coherent(pam->dev, PAMU3_TX_BYTES, pam->tx_trbs,
				  pam->tx_dma);
	pam->data = pam->header = pam->rx_trbs = pam->tx_trbs = NULL;
}

static int pamu3_alloc_buffers(struct sprd_pamu3 *pam)
{
	pam->tx_trbs = dma_alloc_coherent(pam->dev, PAMU3_TX_BYTES,
					  &pam->tx_dma, GFP_KERNEL);
	pam->rx_trbs = dma_alloc_coherent(pam->dev, PAMU3_RX_BYTES,
					  &pam->rx_dma, GFP_KERNEL);
	pam->header = dma_alloc_coherent(pam->dev, PAMU3_RNDIS_HEADER_SIZE,
					 &pam->header_dma, GFP_KERNEL);
	pam->data = dma_alloc_coherent(pam->dev, PAMU3_DATA_BYTES,
				       &pam->data_dma, GFP_KERNEL);
	if (!pam->tx_trbs || !pam->rx_trbs || !pam->header || !pam->data)
		return -ENOMEM;
	if (!pamu3_address_valid(pam->tx_dma, PAMU3_TX_BYTES) ||
	    !pamu3_address_valid(pam->rx_dma, PAMU3_RX_BYTES) ||
	    !pamu3_address_valid(pam->header_dma, PAMU3_RNDIS_HEADER_SIZE) ||
	    !pamu3_address_valid(pam->data_dma, PAMU3_DATA_BYTES))
		return -ERANGE;
	return 0;
}

/* This microcode addresses controller command registers with 32-bit operands;
 * coherent payload and event addresses have separate full 40-bit fields.
 */
static void pamu3_load_code(struct sprd_pamu3 *pam, bool in)
{
	u32 code[20] = { 0,	 0x8100, 0,	 0x8200, 0,	 0x9f00, 0,
			 0x8f00, 0,	 0x8300, 0xcafe, 0xcafe, 0,	 0xa000,
			 0xcafe, 0xcafe, 0,	 0xa000, 0xcafe, 0xcafe };
	u32 endpoint = pam->dwc.regs_phys + REG_DWC3_DEP_BASE(in ? 3 : 2);
	u32 iram = in ? PAM_U3_TXDBG_IRAM : PAM_U3_RXDBG_IRAM;
	u32 entry = in ? PAM_U3_TXPCMDENTRY_ADDR0 : PAM_U3_RXPCMDENTRY_ADDR0;
	unsigned int i;

	code[6] = endpoint;
	code[2] = endpoint + 4;
	code[0] = endpoint + 8;
	code[8] = code[12] = endpoint + 12;
	code[16] = ((endpoint + 12) & PAMU3_MASK_TRIGHDRCNT) |
		   REG_DWC3_GEVNTCOUNT(in ? 1 : 2);
	for (i = 0; i < ARRAY_SIZE(code); i++)
		writel(code[i], pam->base + iram + 4 * i);
	writel(PAMU3_CMDENTER_ADDR0, pam->base + entry);
	writel(PAMU3_CMDENTER_ADDR1, pam->base + entry + 4);
}

static int pamu3_program_resources(struct sprd_pamu3 *pam)
{
	struct pamu3_dwc3_trb *trb = pam->rx_trbs;
	const struct sipa_pam_attrs *dl = &pam->fifo.dl_fifo,
				    *ul = &pam->fifo.ul_fifo;
	u64 regs = pam->dwc.regs_phys;
	unsigned int i;
	u32 value;

	if (regs + REG_DWC3_DEP_BASE(3) + 15 > U32_MAX ||
	    regs + REG_DWC3_DEP_BASE(3) + 15 > pam->common_dma_mask)
		return -ERANGE;
	if (!dl->fifo_depth || !ul->fifo_depth ||
	    !pamu3_address_valid(dl->rx_fifo_base_addr,
				 (u64)dl->fifo_depth * 16) ||
	    !pamu3_address_valid(dl->tx_fifo_base_addr,
				 (u64)dl->fifo_depth * 16) ||
	    !pamu3_address_valid(ul->rx_fifo_base_addr,
				 (u64)ul->fifo_depth * 16) ||
	    !pamu3_address_valid(ul->tx_fifo_base_addr,
				 (u64)ul->fifo_depth * 16) ||
	    !pamu3_address_valid(dl->fifo_sts_addr, 4) ||
	    !pamu3_address_valid(ul->fifo_sts_addr, 4) ||
	    dl->rx_fifo_base_addr + (u64)dl->fifo_depth * 16 - 1 >
		    pam->common_dma_mask ||
	    dl->tx_fifo_base_addr + (u64)dl->fifo_depth * 16 - 1 >
		    pam->common_dma_mask ||
	    ul->rx_fifo_base_addr + (u64)ul->fifo_depth * 16 - 1 >
		    pam->common_dma_mask ||
	    ul->tx_fifo_base_addr + (u64)ul->fifo_depth * 16 - 1 >
		    pam->common_dma_mask ||
	    dl->fifo_sts_addr + 3 > pam->common_dma_mask ||
	    ul->fifo_sts_addr + 3 > pam->common_dma_mask)
		return -ERANGE;
	for (i = 0; i < 2; i++) {
		u32 offset = i ? PAM_U3_RXEVTBUFFER_ADDRL :
				 PAM_U3_TXEVTBUFFER_ADDRL;
		u64 address = regs + REG_DWC3_GEVNTSIZ(i + 1);

		if (!pamu3_address_valid(pam->dwc.event_dma[i],
					 pam->dwc.event_size[i]) ||
		    pam->dwc.event_size[i] > PAMU3_MASK_EVTSZ ||
		    (u64)pam->dwc.event_dma[i] + pam->dwc.event_size[i] - 1 >
			    pam->common_dma_mask)
			return -ERANGE;
		pamu3_write_addr(pam,
				 i ? PAM_U3_RXEVENTTBUFFER_ADDRL :
				     PAM_U3_TXEVENTTBUFFER_ADDRL,
				 pam->dwc.event_dma[i]);
		writel(lower_32_bits(address), pam->base + offset);
		writel(upper_32_bits(address) | (pam->dwc.event_size[i]
						 << PAMU3_BITS_STARTEVTSZ),
		       pam->base + offset + 4);
		pamu3_write_addr(pam,
				 i ? PAM_U3_RXEVTCOUNT_ADDRL :
				     PAM_U3_TXEVTCOUNT_ADDRL,
				 regs + REG_DWC3_GEVNTCOUNT(i + 1));
	}
	pamu3_write_addr(pam, PAM_U3_DLFIFOFILL_ADDRL, dl->tx_fifo_base_addr);
	pamu3_write_addr(pam, PAM_U3_DLFIFOFREE_ADDRL, dl->rx_fifo_base_addr);
	pamu3_write_addr(pam, PAM_U3_ULFIFOFILL_ADDRL, ul->rx_fifo_base_addr);
	pamu3_write_addr(pam, PAM_U3_ULFIFOFREE_ADDRL, ul->tx_fifo_base_addr);
	pamu3_write_addr(pam, PAM_U3_DLGETIPAFIFO_ADDRL, dl->fifo_sts_addr);
	pamu3_write_addr(pam, PAM_U3_DLPUTIPAFIFO_ADDRL, dl->fifo_sts_addr);
	pamu3_write_addr(pam, PAM_U3_ULGETIPAFIFO_ADDRL, ul->fifo_sts_addr);
	pamu3_write_addr(pam, PAM_U3_ULPUTIPAFIFO_ADDRL, ul->fifo_sts_addr);
	for (i = 0; i < PAMU3_RX_TRBBUF_NUM; i++) {
		u64 address = pam->data_dma + i * PAMU3_RX_TRBBUF_SIZE;

		trb[i].bpl = cpu_to_le32(lower_32_bits(address));
		trb[i].bph = cpu_to_le32(upper_32_bits(address));
		trb[i].size = cpu_to_le32(PAMU3_RX_TRBBUF_SIZE);
		trb[i].ctrl = cpu_to_le32(PAMU3_TRB_CTRL_DEFVAL);
		pamu3_write_addr(pam, PAM_U3_RXBUF0_ADDRL + 8 * i, address);
		pamu3_write_addr(pam, PAM_U3_RXTRBBUF0_ADDRL + 8 * i,
				 pam->rx_dma + i * sizeof(*trb));
	}
	value = readl(pam->base + PAM_U3_RXTRBBUF1_ADDRH) &
		PAMU3_MASK_ADDR32_LSB;
	writel(value | (PAMU3_RX_TRBBUF_SIZE << PAMU3_BIT_RXTRBBUFSIZE_SHIFT),
	       pam->base + PAM_U3_RXTRBBUF1_ADDRH);
	for (i = 0; i < PAMU3_TX_TRBBUF_NUM; i++)
		pamu3_write_addr(pam, PAM_U3_TXTRBBUF0_ADDRL + 8 * i,
				 pam->tx_dma +
					 i * sizeof(*trb) * PAMU3_TX_TRB_NUM);
	pamu3_write_addr(pam, PAM_U3_HEADER_ENDBASE_ADDRL, pam->header_dma);
	writel(PAM_U3_MBIM_DEFNTH, pam->base + PAM_U3_MBIM_NCM_NTH);
	writel(PAM_U3_MBIM_DEFNDP, pam->base + PAM_U3_MBIM_NCM_NDP);
	pamu3_load_code(pam, true);
	pamu3_load_code(pam, false);
	dma_wmb();
	/* No PAM_EN/USB_EN/TX_START/RX_START is asserted by preparation. */
	return 0;
}

static int pamu3_release_resources(struct sprd_pamu3 *pam)
{
	int ret;

	if (pam->claimed) {
		ret = dwc3_pamu3_release(pam->controller, pam->dev);
		if (ret)
			return ret;
		pam->claimed = false;
		if (pam->out) {
			pam->out->desc = NULL;
			pam->out->comp_desc = NULL;
			pam->in->desc = NULL;
			pam->in->comp_desc = NULL;
		}
	}
	if (pam->attached) {
		ret = dwc3_pamu3_detach(pam->controller, pam->dev);
		if (ret)
			return ret;
		pam->attached = false;
	}
	pamu3_free_buffers(pam);
	if (pam->reset_held) {
		ret = reset_control_rearm(pam->reset);
		if (ret)
			return ret;
		pam->reset_held = false;
	}
	if (pam->clock_on) {
		spin_lock_irq(&pam->hw_lock);
		pam->clock_on = false;
		spin_unlock_irq(&pam->hw_lock);
		clk_disable_unprepare(pam->clk);
	}
	pam->prepared = false;
	return 0;
}

int sprd_pamu3_prepare(struct device *dev)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	int ret, cleanup;

	if (!pam)
		return -ENODEV;
	if (WARN_ON_ONCE(in_interrupt()))
		return -EWOULDBLOCK;
	mutex_lock(&pam->lock);
	if (pam->prepared || pam->attached || pam->reset_held) {
		ret = -EBUSY;
		goto out;
	}
	ret = clk_prepare_enable(pam->clk);
	if (ret)
		goto out;
	spin_lock_irq(&pam->hw_lock);
	pam->clock_on = true;
	spin_unlock_irq(&pam->hw_lock);
	ret = dwc3_pamu3_attach(pam->controller, pam->dev, &pam->dwc);
	if (ret)
		goto fail;
	pam->attached = true;
	if (!pamu3_same_dma_domain(pam->dev, pam->dwc.dma_dev) ||
	    !pamu3_same_dma_domain(pam->dev, pam->sipa)) {
		ret = -EXDEV;
		goto fail;
	}
	if (readl(pam->base + PAM_U3_CTL0) &
	    (PAMU3_CTL0_BIT_USB_EN | PAMU3_CTL0_BIT_PAM_EN)) {
		/* Existing DMA is not ours to tear down. Keep supplier/reset/clock
		 * references pinned until a successful explicit stop/drain.
		 */
		pam->foreign_active = true;
		ret = -EBUSY;
		goto out;
	}
	/* The controller is now pinned and no foreign PAM DMA is active.
	 * Reset PAM's internal state on every prepare, without resetting USB1.
	 * A shared USB1/PAM trigger alone is a no-op while the PHY holds it;
	 * that left RX unable to issue STARTTRANSFER after a successful stop.
	 */
	ret = reset_control_reset(pam->reset);
	if (ret)
		goto fail;
	pam->reset_held = true;
	pam->common_dma_mask = min_t(u64, DMA_BIT_MASK(40),
				     min(dma_get_mask(pam->dev),
					 min(dma_get_mask(pam->dwc.dma_dev),
					     dma_get_mask(pam->sipa))));
	ret = dma_set_mask_and_coherent(pam->dev, pam->common_dma_mask);
	if (ret)
		goto fail;
	ret = sipa_get_ep_info(SIPA_EP_USB, &pam->fifo);
	if (ret)
		goto fail;
	ret = pamu3_alloc_buffers(pam);
	if (ret)
		goto fail;
	ret = pamu3_program_resources(pam);
	if (ret)
		goto fail;
	pam->prepared = true;
	goto out;
fail:
	cleanup = pamu3_release_resources(pam);
	if (cleanup)
		dev_err(dev, "PAM prepare cleanup remains pinned: %d\n",
			cleanup);
out:
	mutex_unlock(&pam->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_prepare);

/* No reverse DT edge is needed: provider lookup follows its existing
 * PAM -> DWC3 link. Returned device reference belongs to the caller.
 */
struct device *sprd_pamu3_get(struct device *controller)
{
	struct sprd_pamu3 *pam;
	struct device *dev = ERR_PTR(-EPROBE_DEFER);

	mutex_lock(&pamu3_devices_lock);
	list_for_each_entry(pam, &pamu3_devices, node) {
		if (pam->controller == controller) {
			if (pam->function_owned)
				dev = ERR_PTR(-EBUSY);
			else {
				pam->function_owned = true;
				dev = get_device(pam->dev);
			}
			break;
		}
	}
	mutex_unlock(&pamu3_devices_lock);
	return dev;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_get);

int sprd_pamu3_put(struct device *dev)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&pamu3_devices_lock);
	mutex_lock(&pam->lock);
	if (pam->attached || pam->claimed || pam->connected || pam->requested ||
	    pam->reset_held || pam->clock_on || pam->rm_registered ||
	    pam->dependency)
		ret = -EBUSY; /* failed drain keeps the provider unavailable */
	else
		pam->function_owned = false;
	mutex_unlock(&pam->lock);
	mutex_unlock(&pamu3_devices_lock);
	put_device(dev);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_put);

static void pamu3_rm_notify(void *data, enum sipa_rm_event event,
			    unsigned long unused)
{
	struct sprd_pamu3 *pam = data;

	if (event == SIPA_RM_EVT_GRANTED)
		complete(&pam->grant);
}

static int pamu3_rm_get(struct sprd_pamu3 *pam)
{
	unsigned long deadline;
	long done;
	int ret, release_ret;

	if (!pam->rm_registered) {
		pam->rm.user_data = pam;
		pam->rm.notify_cb = pamu3_rm_notify;
		ret = sipa_rm_register(SIPA_RM_RES_CONS_USB, &pam->rm);
		if (ret)
			return ret;
		pam->rm_registered = true;
	}
	if (pam->rm_error)
		return pam->rm_error;
	if (!pam->dependency) {
		ret = sipa_rm_enable_usb_tether();
		if (ret)
			return ret;
		pam->dependency = true;
	}
	if (pam->requested)
		return -EBUSY;
	reinit_completion(&pam->grant);
	/* The legacy consumer increments ref_count even on a producer's hard
	 * request error. register above proves this consumer exists.
	 */
	pam->requested = true;
	ret = sipa_rm_request_resource(SIPA_RM_RES_CONS_USB);
	if (ret && ret != -EINPROGRESS)
		return ret;
	if (ret == -EINPROGRESS) {
		/* The caller waits before pullup or after acknowledging endpoint
		 * activation. Wait for the real firmware grant without holding
		 * EP0, and let any USB disable cancel within 100 ms.
		 */
		deadline = jiffies + msecs_to_jiffies(30000);
		for (;;) {
			if (READ_ONCE(pam->abort))
				return -ECANCELED;
			if (time_after_eq(jiffies, deadline))
				return -ETIMEDOUT;
			done = wait_for_completion_interruptible_timeout(
				&pam->grant, msecs_to_jiffies(100));
			if (done < 0)
				return (int)done;
			if (done)
				break;
		}
	}
	if (READ_ONCE(pam->abort))
		return -ECANCELED;
	/* Completion events have no generation tag. A fresh counted query
	 * confirms the current graph is granted; the original lease stays held.
	 */
	ret = sipa_rm_request_resource(SIPA_RM_RES_CONS_USB);
	release_ret = sipa_rm_release_resource(SIPA_RM_RES_CONS_USB);
	if (release_ret && release_ret != -EINPROGRESS) {
		pam->rm_error = release_ret;
		return release_ret;
	}
	return ret;
}

static int pamu3_rm_put(struct sprd_pamu3 *pam)
{
	int ret;

	if (pam->rm_error)
		return pam->rm_error;
	if (pam->requested) {
		ret = sipa_rm_release_resource(SIPA_RM_RES_CONS_USB);
		/* Registered static consumer: release consumed this reference even
		 * on a hard producer error.  Never retry and decrement another's.
		 */
		pam->requested = false;
		if (ret && ret != -EINPROGRESS) {
			pam->rm_error = ret;
			return ret;
		}
	}
	/* CP/PCIe and IPA edges are shared topology, not this function's lease. */
	pam->dependency = false;
	if (pam->rm_registered) {
		ret = sipa_rm_deregister(SIPA_RM_RES_CONS_USB, &pam->rm);
		if (ret)
			return ret;
		pam->rm_registered = false;
	}
	return 0;
}

/* Configfs bind runs before the controller pullup, so this cold-boot
 * firmware dependency can settle without consuming the host's EP0 timeout.
 * This is a real, temporary consumer lease, not a synthetic ready flag.
 * Release it before returning; normal activation owns its own lease.
 */
int sprd_pamu3_wait_ready(struct device *dev)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	int ret, cleanup;

	if (!pam || in_interrupt())
		return -EINVAL;
	mutex_lock(&pam->lock);
	if (!pam->function_owned || pam->attached || pam->prepared ||
	    pam->claimed || pam->connected || pam->requested || pam->stopping ||
	    pam->reset_held || pam->clock_on || pam->rm_registered ||
	    pam->dependency) {
		ret = -EBUSY;
		goto unlock;
	}
	/* No endpoint or worker exists yet; a previous clean unbind may have
	 * left the cancellation flag set.
	 */
	WRITE_ONCE(pam->abort, false);
	ret = pamu3_rm_get(pam);
	cleanup = pamu3_rm_put(pam);
	if (cleanup) {
		dev_err(dev, "prebind resource cleanup failed: %d\n", cleanup);
		ret = cleanup;
	}
	if (!ret)
		dev_info(dev, "network resource ready before USB bind\n");
unlock:
	mutex_unlock(&pam->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_wait_ready);

/* Separate endpoint ownership from traffic enable. Called by the function's
 * process-context SET_INTERFACE/SET_CONFIGURATION worker, never under the UDC
 * spinlock. Descriptor copies remain alive even if a failed drain pins PAM.
 */
int sprd_pamu3_arm(struct device *dev, struct usb_ep *out, struct usb_ep *in)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	int ret;

	if (!pam || !out || !in || !out->desc || !in->desc)
		return -EINVAL;
	if (in_interrupt())
		return -EWOULDBLOCK;
	mutex_lock(&pam->lock);
	if (!pam->prepared || pam->claimed) {
		ret = -EBUSY;
		goto unlock;
	}
	if (!usb_endpoint_xfer_bulk(out->desc) ||
	    !usb_endpoint_xfer_bulk(in->desc) ||
	    usb_endpoint_num(out->desc) != 1 ||
	    usb_endpoint_num(in->desc) != 1 || usb_endpoint_dir_in(out->desc) ||
	    !usb_endpoint_dir_in(in->desc)) {
		ret = -EINVAL;
		goto unlock;
	}
	ret = dwc3_pamu3_claim(pam->controller, dev, out, in);
	if (ret)
		goto unlock;
	pam->claimed = true;
	spin_lock_irq(&pam->hw_lock);
	pam->abort = false;
	spin_unlock_irq(&pam->hw_lock);
	pam->out = out;
	pam->in = in;
	pam->out_desc = *out->desc;
	pam->in_desc = *in->desc;
	out->desc = &pam->out_desc;
	in->desc = &pam->in_desc;
	if (out->comp_desc) {
		pam->out_comp = *out->comp_desc;
		out->comp_desc = &pam->out_comp;
	}
	if (in->comp_desc) {
		pam->in_comp = *in->comp_desc;
		in->comp_desc = &pam->in_comp;
	}
	ret = usb_ep_enable(in);
	if (!ret)
		ret = usb_ep_enable(out);
	/* On failure retain ownership for stop(), which is also the rollback
	 * entry point. No ordinary queue/disable may race this pair.
	 */
unlock:
	mutex_unlock(&pam->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_arm);

static int pamu3_program_framing(struct sprd_pamu3 *pam,
				 const struct sprd_pamu3_config *cfg)
{
	u32 value;

	if (!cfg || !cfg->max_packets || cfg->max_packets > MAX_PACKET_NUM ||
	    cfg->max_transfer != PAMU3_RX_TRBBUF_SIZE || cfg->netid > U8_MAX)
		return -EINVAL;
	switch (cfg->protocol) {
	case SPRD_PAMU3_NCM_NTB16:
		/* The BSP names mode 1 MBIM and mode 2 NCM, but wire captures
		 * establish that they select NTH16 and NTH32 respectively. NCM16
		 * therefore uses mode 1 with the NCM signatures and Ethernet IPA
		 * routing; MBIM keeps its separate raw-IP routing selection.
		 */
		if (cfg->nth_signature != 0x484d434e ||
		    cfg->ndp_signature != 0x304d434e)
			return -EOPNOTSUPP;
		value = PAMU3_PROTOCOL_MBIM << PAMU3_CTL0_SHIFT_PROTOCOL;
		break;
	case SPRD_PAMU3_MBIM_NTB16:
		/* Captured NTB16 framing template: NCMH + IPS0.
		 * NTB32, DSS and multiple sessions need different templates.
		 */
		if (cfg->nth_signature != PAM_U3_MBIM_DEFNTH ||
		    cfg->ndp_signature != PAM_U3_MBIM_DEFNDP)
			return -EOPNOTSUPP;
		value = PAMU3_PROTOCOL_MBIM << PAMU3_CTL0_SHIFT_PROTOCOL;
		break;
	case SPRD_PAMU3_RNDIS:
		/* VN007 AP start 84942b0 uses protocol 0 with the same DMA/ring
		 * template. It does not require invented MAC or length-register
		 * writes; the PAM consumes/generates the RNDIS Ethernet framing.
		 */
		value = PAMU3_PROTOCOL_RNDIS << PAMU3_CTL0_SHIFT_PROTOCOL;
		break;
	default:
		/* Protocol NULL is not established as CDC ECM. */
		return -EOPNOTSUPP;
	}
	writel(value, pam->base + PAM_U3_CTL0); /* ORCA project bit = 0 */
	if ((readl(pam->base + PAM_U3_CTL0) & PAMU3_CTL0_MASK_PROTOCOL) !=
	    value)
		return -EIO;
	/* Reference RNDIS and MBIM setup both retain these reset templates. */
	writel(cfg->protocol == SPRD_PAMU3_NCM_NTB16 ? cfg->nth_signature :
						       PAM_U3_MBIM_DEFNTH,
	       pam->base + PAM_U3_MBIM_NCM_NTH);
	writel(cfg->protocol == SPRD_PAMU3_NCM_NTB16 ? cfg->ndp_signature :
						       PAM_U3_MBIM_DEFNDP,
	       pam->base + PAM_U3_MBIM_NCM_NDP);
	value = readl(pam->base + PAM_U3_SRC_MACH) & PAMU3_MASK_SRCMAC_ADDRH;
	writel(value | (cfg->max_packets << MAX_PACKET_NUM_SHIFT_BIT),
	       pam->base + PAM_U3_SRC_MACH);
	value = readl(pam->base + PAM_U3_UL_NODE_HEADER);
	value &= ~(PAMU3_MASK_NETID | PAMU3_MASK_ULNODE_SRC);
	value |= cfg->netid | (SIPA_TERM_USB << PAMU3_SHIFT_ULNODE_SRC);
	writel(value, pam->base + PAM_U3_UL_NODE_HEADER);
	writel(readl(pam->base + PAM_U3_TRB_HEADER) | PAMU3_TRB_LAST,
	       pam->base + PAM_U3_TRB_HEADER);
	return 0;
}

int sprd_pamu3_start(struct device *dev, const struct sprd_pamu3_config *cfg)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	struct sipa_connect_params params = { .id = SIPA_EP_USB };
	u32 value, depth;
	int ret;

	if (!pam || !cfg)
		return -EINVAL;
	if (in_interrupt())
		return -EWOULDBLOCK;
	mutex_lock(&pam->lock);
	if (!pam->prepared || !pam->claimed || !pam->out->enabled ||
	    !pam->in->enabled || pam->running || pam->connected ||
	    pam->stopping) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = pamu3_program_framing(pam, cfg);
	if (ret)
		goto unlock;
	ret = pamu3_rm_get(pam);
	if (ret)
		goto unlock;
	params.usb_format = cfg->protocol == SPRD_PAMU3_MBIM_NTB16 ?
				    SIPA_USB_MBIM :
				    SIPA_USB_RNDIS;
	depth = pam->fifo.dl_fifo.fifo_depth;
	params.send_param.tx_intr_threshold = PAM_U3_MAX_DLPKTS_DEF;
	params.send_param.tx_intr_delay_us = 5;
	params.send_param.flow_ctrl_irq_mode = 2;
	params.recv_param.tx_intr_threshold = PAM_U3_MAX_ULPKTS_DEF;
	params.recv_param.tx_intr_delay_us = 5;
	params.recv_param.tx_enter_flowctrl_watermark = depth - depth / 4;
	params.recv_param.tx_leave_flowctrl_watermark = depth / 2;
	params.recv_param.flow_ctrl_cfg = 1;
	/* Treat connect as potentially partially effective until checked stop.
	 * The SIPA checked lifecycle patch is a required dependency.
	 */
	ret = sipa_pam_connect(&params);
	pam->connected = !ret || ret == -EUCLEAN;
	if (ret)
		goto unlock;
	spin_lock_irq(&pam->hw_lock);
	if (pam->abort) {
		spin_unlock_irq(&pam->hw_lock);
		ret = -ECANCELED;
		goto unlock;
	}
	dma_wmb();
	writel(0, pam->base + PAM_U3_INR_EN); /* polling; no unhandled IRQ */
	value = readl(pam->base + PAM_U3_CTL0) | PAMU3_CTL0_BIT_PAM_EN;
	writel(value, pam->base + PAM_U3_CTL0);
	value |= PAMU3_CTL0_BIT_RX_START | PAMU3_CTL0_BIT_TX_START;
	writel(value, pam->base + PAM_U3_CTL0);
	writel(value | PAMU3_CTL0_BIT_USB_EN, pam->base + PAM_U3_CTL0);
	pam->running = true;
	spin_unlock_irq(&pam->hw_lock);
	ret = 0;
unlock:
	mutex_unlock(&pam->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_start);

/* Request the hardware RELEASE handshake immediately on USB disable/reset.
 * Keep the micro-engines running: clearing START before RELEASE prevents DONE.
 * DMA buffers, endpoints and clocks remain owned until the worker verifies
 * DONE and idle. hw_lock fences clock-off and a racing enable transaction.
 */
void sprd_pamu3_quiesce_atomic(struct device *dev)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	unsigned long flags;
	u32 value;

	if (!pam)
		return;
	spin_lock_irqsave(&pam->hw_lock, flags);
	pam->abort = true;
	if (pam->clock_on && READ_ONCE(pam->claimed) && !pam->foreign_active) {
		value = readl(pam->base + PAM_U3_CTL0);
		value = (value & ~PAMU3_CTL0_BITS_STOPMODE) | BIT(3) |
			PAMU3_CTL0_BIT_RELEASE;
		writel(value, pam->base + PAM_U3_CTL0);
	}
	spin_unlock_irqrestore(&pam->hw_lock, flags);
}
EXPORT_SYMBOL_GPL(sprd_pamu3_quiesce_atomic);

/* A closed host cannot consume the final NTB. End both external DMA
 * engines before returning those nodes through SIPA's normal completion path.
 * Allocations and grants remain owned on every error.
 */
static int pamu3_abort_locked(struct sprd_pamu3 *pam)
{
	u32 value;
	int ret, status;

	if (pam->foreign_active || !pam->clock_on || !pam->claimed)
		return -EBUSY;
	if (pam->running) {
		spin_lock_irq(&pam->hw_lock);
		pam->abort = true;
		value = readl(pam->base + PAM_U3_CTL0);
		value = (value & ~PAMU3_CTL0_BITS_STOPMODE) | BIT(3) |
			PAMU3_CTL0_BIT_RELEASE;
		writel(value, pam->base + PAM_U3_CTL0);
		spin_unlock_irq(&pam->hw_lock);
		ret = readl_poll_timeout(
			pam->base + PAM_U3_CTL1, value,
			!(value & (PAMU3_CTL1_TXHWBUSY | PAMU3_CTL1_RXHWBUSY)),
			10, 100000);
		if (ret)
			return ret;
		spin_lock_irq(&pam->hw_lock);
		value = readl(pam->base + PAM_U3_CTL0);
		value &= ~(PAMU3_CTL0_BIT_USB_EN | PAMU3_CTL0_BIT_PAM_EN |
			   PAMU3_CTL0_BIT_TX_START | PAMU3_CTL0_BIT_RX_START);
		writel(value, pam->base + PAM_U3_CTL0);
		spin_unlock_irq(&pam->hw_lock);
		ret = dwc3_pamu3_drain(pam->controller, pam->dev);
		if (ret)
			return ret;
		if (pam->reset_held) {
			ret = reset_control_rearm(pam->reset);
			if (ret)
				return ret;
			pam->reset_held = false;
		}
		ret = reset_control_reset(pam->reset);
		if (ret)
			return ret;
		pam->reset_held = true;
		/* Reset leaves HWBUSY at its uninitialized default, 0x300.
		 * The independently reset master is stopped; do not wait for an
		 * initialized running-engine status after reset has completed.
		 */
		pam->running = false;
	}
	if (readl(pam->base + PAM_U3_CTL0) &
	    (PAMU3_CTL0_BIT_PAM_EN | PAMU3_CTL0_BIT_USB_EN))
		return -EIO;
	if (!pam->connected)
		return 0;
	ret = sipa_disconnect(SIPA_EP_USB, SIPA_DISCONNECT_RECLAIM);
	if (ret)
		return ret;
	ret = read_poll_timeout(sipa_disconnect, status, status != -EAGAIN,
				1000, 500000, false, SIPA_EP_USB,
				SIPA_DISCONNECT_START);
	if (ret || status)
		return ret ?: status;
	pam->stopping = true;
	dev_info(pam->dev,
		 "Aborted USB traffic returned through SIPA completion\n");
	return 0;
}

static int pamu3_pause_locked(struct sprd_pamu3 *pam)
{
	u32 value;
	int ret;

	if (pam->foreign_active)
		return -EBUSY; /* unknown DMA must not be stopped or reclaimed as ours */
	if (pam->connected && !pam->stopping) {
		int status;

		/* Fence new AP submissions and first allow normal completion. A
		 * peer disconnect may strand a partial NTB; cancel its DMA before
		 * reclaiming descriptors, then verify the ownership ledger again.
		 */
		ret = read_poll_timeout(sipa_disconnect, status,
					status != -EAGAIN, 1000, 500000, false,
					SIPA_EP_USB, SIPA_DISCONNECT_START);
		if (ret == -ETIMEDOUT && status == -EAGAIN) {
			ret = pamu3_abort_locked(pam);
			if (ret)
				return ret;
		} else if (ret || status) {
			return ret ?: status;
		}
		pam->stopping = true;
	}
	if (pam->clock_on) {
		value = readl(pam->base + PAM_U3_CTL0);
		if (value & (PAMU3_CTL0_BIT_USB_EN | PAMU3_CTL0_BIT_PAM_EN)) {
			/* On ORCA, STOPMODE=1 is required to acknowledge RELEASE after
			 * the host closes the bulk interface. Mode 0 remained idle with
			 * DONE clear; selecting mode 1 returned DONE on real hardware.
			 * Keep USB_EN/START until DONE, then verify both engines idle
			 * and drain DWC3 before retiring any DMA allocation.
			 */
			value = (value & ~PAMU3_CTL0_BITS_STOPMODE) | BIT(3);
			writel(value | PAMU3_CTL0_BIT_RELEASE,
			       pam->base + PAM_U3_CTL0);
			ret = readl_poll_timeout(pam->base + PAM_U3_CTL0, value,
						 value & PAMU3_CTL0_BIT_DONE,
						 10, 500000);
			if (ret) {
				ret = pamu3_abort_locked(pam);
				if (ret)
					return ret;
				value = readl(pam->base + PAM_U3_CTL0);
			}
			value &= ~(PAMU3_CTL0_BIT_USB_EN |
				   PAMU3_CTL0_BIT_PAM_EN |
				   PAMU3_CTL0_BIT_RELEASE |
				   PAMU3_CTL0_BIT_TX_START |
				   PAMU3_CTL0_BIT_RX_START);
			writel(value, pam->base + PAM_U3_CTL0);
		}
		if (pam->running) {
			ret = readl_poll_timeout(
				pam->base + PAM_U3_CTL1, value,
				!(value &
				  (PAMU3_CTL1_TXHWBUSY | PAMU3_CTL1_RXHWBUSY)),
				10, 500000);
			if (ret)
				return ret;
		}
		writel(0, pam->base + PAM_U3_INR_EN);
	}
	if (pam->claimed) {
		ret = dwc3_pamu3_drain(pam->controller, pam->dev);
		if (ret)
			return ret;
	}
	/* END is legal only now: the external DMA master is proven idle. */
	if (pam->connected) {
		ret = sipa_disconnect(SIPA_EP_USB, SIPA_DISCONNECT_END);
		if (ret)
			return ret;
		pam->connected = false;
		pam->stopping = false;
	}
	pam->running = false;
	return pamu3_rm_put(pam);
}

int sprd_pamu3_stop(struct device *dev)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	int ret;

	if (!pam)
		return -ENODEV;
	if (in_interrupt())
		return -EWOULDBLOCK;
	mutex_lock(&pam->lock);
	ret = pamu3_pause_locked(pam);
	if (!ret)
		ret = pamu3_release_resources(pam);
	/* Any error preserves every not-yet-proven-safe resource. */
	mutex_unlock(&pam->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sprd_pamu3_stop);

static void pamu3_put_device(void *data)
{
	put_device(data);
}

static int sprd_pamu3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *controller, *sipa;
	struct sprd_pamu3 *pam;
	struct device_node *np;
	struct resource *res;
	int ret;

	pam = devm_kzalloc(dev, sizeof(*pam), GFP_KERNEL);
	if (!pam)
		return -ENOMEM;
	pam->dev = dev;
	mutex_init(&pam->lock);
	spin_lock_init(&pam->hw_lock);
	init_completion(&pam->grant);
	INIT_LIST_HEAD(&pam->node);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "pamu3_glb_regs");
	if (!res || resource_size(res) < PAM_U3_RXDBG_IRAM + 20 * sizeof(u32))
		return -EINVAL;
	pam->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pam->base))
		return PTR_ERR(pam->base);
	pam->clk = devm_clk_get(dev, "pamu3_clk");
	if (IS_ERR(pam->clk))
		return PTR_ERR(pam->clk);
	pam->reset = devm_reset_control_get_shared(dev, "pamu3");
	if (IS_ERR(pam->reset))
		return PTR_ERR(pam->reset);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;
	np = of_parse_phandle(dev->of_node, "sprd,dwc3", 0);
	if (!np)
		return -EINVAL;
	if (!of_device_is_compatible(np, "snps,dwc3")) {
		of_node_put(np);
		return -EINVAL;
	}
	controller = of_find_device_by_node(np);
	of_node_put(np);
	if (!controller)
		return -EPROBE_DEFER;
	pam->controller = &controller->dev;
	ret = devm_add_action_or_reset(dev, pamu3_put_device, pam->controller);
	if (ret)
		return ret;
	if (!dev_get_drvdata(pam->controller))
		return -EPROBE_DEFER;
	np = of_parse_phandle(dev->of_node, "sprd,sipa", 0);
	if (!np)
		return -EINVAL;
	if (!of_device_is_available(np)) {
		of_node_put(np);
		return -ENODEV;
	}
	sipa = of_find_device_by_node(np);
	of_node_put(np);
	if (!sipa)
		return -EPROBE_DEFER;
	pam->sipa = &sipa->dev;
	ret = devm_add_action_or_reset(dev, pamu3_put_device, pam->sipa);
	if (ret)
		return ret;
	if (!device_is_bound(pam->sipa) || !dev_get_drvdata(pam->sipa))
		return -EPROBE_DEFER;
	if (!device_link_add(dev, pam->controller,
			     DL_FLAG_AUTOREMOVE_CONSUMER) ||
	    !device_link_add(dev, pam->sipa, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	platform_set_drvdata(pdev, pam);
	mutex_lock(&pamu3_devices_lock);
	list_add_tail(&pam->node, &pamu3_devices);
	mutex_unlock(&pamu3_devices_lock);
	return 0;
}

static void sprd_pamu3_remove(struct platform_device *pdev)
{
	struct sprd_pamu3 *pam = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&pamu3_devices_lock);
	list_del(&pam->node);
	mutex_unlock(&pamu3_devices_lock);
	ret = sprd_pamu3_stop(&pdev->dev);

	if (ret)
		dev_err(&pdev->dev,
			"unsupported forced removal after PAM drain failure (%d); devres cannot preserve the retained DMA ownership\n",
			ret);
}

static ssize_t offload_state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sprd_pamu3 *pam = dev_get_drvdata(dev);
	u32 ctl0 = 0;
	ssize_t len;

	mutex_lock(&pam->lock);
	/* Never access the register window while its clock is disabled. */
	if (pam->clock_on)
		ctl0 = readl(pam->base + PAM_U3_CTL0);
	len = sysfs_emit(
		buf,
		"clock=%u prepared=%u claimed=%u running=%u connected=%u requested=%u ctl0=0x%08x\n",
		pam->clock_on, pam->prepared, pam->claimed, pam->running,
		pam->connected, pam->requested, ctl0);
	mutex_unlock(&pam->lock);
	return len;
}
static DEVICE_ATTR_RO(offload_state);

static struct attribute *pamu3_attrs[] = {
	&dev_attr_offload_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pamu3);

static const struct of_device_id sprd_pamu3_match[] = {
	{ .compatible = "sprd,orca-pamu3" },
	{}
};
MODULE_DEVICE_TABLE(of, sprd_pamu3_match);

static struct platform_driver sprd_pamu3_driver = {
	.probe = sprd_pamu3_probe,
	.remove = sprd_pamu3_remove,
	.driver = {
		.name = "sprd-pamu3",
		.of_match_table = sprd_pamu3_match,
		.suppress_bind_attrs = true,
		.dev_groups = pamu3_groups,
	},
};
module_platform_driver(sprd_pamu3_driver);
MODULE_DESCRIPTION("UDX710 PAMU3 checked resources and DWC3 ownership boundary");
MODULE_LICENSE("GPL");
