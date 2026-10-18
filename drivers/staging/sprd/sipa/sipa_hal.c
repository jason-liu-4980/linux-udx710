#include <linux/sipa.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "sipa_hal.h"
#include "sipa_hal_priv.h"
#include "sipa_priv.h"

static bool fast_descriptor_decode = true;
module_param(fast_descriptor_decode, bool, 0644);
MODULE_PARM_DESC(fast_descriptor_decode,
		 "Snapshot DMA completion descriptors before decoding");

static void sipa_decode_node_words(struct sipa_hal_fifo_item *item, u64 low,
				   u64 high)
{
	item->addr = low & GENMASK_ULL(39, 0);
	item->len = (low >> 40) & GENMASK_ULL(19, 0);
	item->offset = (low >> 60) | ((high & 0xff) << 4);
	item->netid = (high >> 8) & 0xff;
	item->src = (high >> 16) & 0x1f;
	item->dst = (high >> 21) & 0x1f;
	item->intr = (high >> 36) & 1;
	item->err_code = (high >> 38) & 0xf;
}

struct sipa_hal_context sipa_hal_ctx;

static int alloc_tx_fifo_ram(struct device *dev, struct sipa_hal_context *cfg,
			     enum sipa_cmn_fifo_index index)
{
	ssize_t size, node_size = sizeof(struct sipa_node_description_tag);
	dma_addr_t phy_addr;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = &cfg->cmn_fifo_cfg[index];

	if (!fifo_cfg->tx_fifo.depth)
		return 0;
	size = fifo_cfg->tx_fifo.depth * node_size;
	if (fifo_cfg->tx_fifo.in_iram) {
		if (cfg->phy_virt_res.iram_allocated_size >
			    cfg->phy_virt_res.iram_size ||
		    size > cfg->phy_virt_res.iram_size -
				    cfg->phy_virt_res.iram_allocated_size)
			return -ENOMEM;

		fifo_cfg->tx_fifo.virtual_addr =
			cfg->phy_virt_res.iram_base +
			cfg->phy_virt_res.iram_allocated_size;

		phy_addr = cfg->phy_virt_res.iram_phy +
			   cfg->phy_virt_res.iram_allocated_size;

		cfg->phy_virt_res.iram_allocated_size += size;
	} else {
		/* Descriptor ownership changes at 16-byte granularity, smaller
		 * than noncoherent cache lines. Use coherent rings for every
		 * DDR FIFO; packet payloads retain streaming DMA mappings.
		 */
		fifo_cfg->tx_fifo.virtual_addr =
			dma_alloc_coherent(dev, size, &phy_addr, GFP_KERNEL);
		if (!fifo_cfg->tx_fifo.virtual_addr)
			return -ENOMEM;
		memset(fifo_cfg->tx_fifo.virtual_addr, 0, size);
	}
	fifo_cfg->tx_fifo.size = size;
	fifo_cfg->tx_fifo.fifo_base_addr_l = IPA_GET_LOW32(phy_addr);
	fifo_cfg->tx_fifo.fifo_base_addr_h = IPA_GET_HIGH32(phy_addr);

	return 0;
}

static int alloc_rx_fifo_ram(struct device *dev, struct sipa_hal_context *cfg,
			     enum sipa_cmn_fifo_index index)
{
	dma_addr_t phy_addr;
	ssize_t size, node_size = sizeof(struct sipa_node_description_tag);
	struct sipa_common_fifo_cfg_tag *fifo_cfg = &cfg->cmn_fifo_cfg[index];

	if (!fifo_cfg->rx_fifo.depth)
		return 0;
	size = fifo_cfg->rx_fifo.depth * node_size;
	if (fifo_cfg->rx_fifo.in_iram) {
		if (cfg->phy_virt_res.iram_allocated_size >
			    cfg->phy_virt_res.iram_size ||
		    size > cfg->phy_virt_res.iram_size -
				    cfg->phy_virt_res.iram_allocated_size) {
			dev_err(dev, "fifo id = %d don't have iram\n", index);
			return -ENOMEM;
		}

		fifo_cfg->rx_fifo.virtual_addr =
			cfg->phy_virt_res.iram_base +
			cfg->phy_virt_res.iram_allocated_size;

		phy_addr = cfg->phy_virt_res.iram_phy +
			   cfg->phy_virt_res.iram_allocated_size;

		cfg->phy_virt_res.iram_allocated_size += size;
	} else {
		/* Descriptor ownership changes at 16-byte granularity, smaller
		 * than noncoherent cache lines. Use coherent rings for every
		 * DDR FIFO; packet payloads retain streaming DMA mappings.
		 */
		fifo_cfg->rx_fifo.virtual_addr =
			dma_alloc_coherent(dev, size, &phy_addr, GFP_KERNEL);
		if (!fifo_cfg->rx_fifo.virtual_addr)
			return -ENOMEM;
		memset(fifo_cfg->rx_fifo.virtual_addr, 0, size);
	}
	fifo_cfg->rx_fifo.size = size;
	fifo_cfg->rx_fifo.fifo_base_addr_l = IPA_GET_LOW32(phy_addr);
	fifo_cfg->rx_fifo.fifo_base_addr_h = IPA_GET_HIGH32(phy_addr);

	return 0;
}

static void free_tx_fifo_ram(struct device *dev, struct sipa_hal_context *cfg,
			     enum sipa_cmn_fifo_index index)
{
	dma_addr_t phy_addr;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = &cfg->cmn_fifo_cfg[index];

	if (!fifo_cfg->tx_fifo.virtual_addr)
		return;
	phy_addr = IPA_STI_64BIT(fifo_cfg->tx_fifo.fifo_base_addr_l,
				 fifo_cfg->tx_fifo.fifo_base_addr_h);
	if (!fifo_cfg->tx_fifo.in_iram)
		dma_free_coherent(dev, fifo_cfg->tx_fifo.size,
				  fifo_cfg->tx_fifo.virtual_addr, phy_addr);
}

static void free_rx_fifo_ram(struct device *dev, struct sipa_hal_context *cfg,
			     enum sipa_cmn_fifo_index index)
{
	dma_addr_t phy_addr;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = &cfg->cmn_fifo_cfg[index];

	if (!fifo_cfg->rx_fifo.virtual_addr)
		return;
	phy_addr = IPA_STI_64BIT(fifo_cfg->rx_fifo.fifo_base_addr_l,
				 fifo_cfg->rx_fifo.fifo_base_addr_h);
	if (!fifo_cfg->rx_fifo.in_iram)
		dma_free_coherent(dev, fifo_cfg->rx_fifo.size,
				  fifo_cfg->rx_fifo.virtual_addr, phy_addr);
}

static void sipa_free_fifo_addr(struct device *dev,
				struct sipa_hal_context *cfg)
{
	int i;

	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		free_rx_fifo_ram(dev, cfg, i);
		free_tx_fifo_ram(dev, cfg, i);
		cfg->cmn_fifo_cfg[i].rx_fifo.virtual_addr = NULL;
		cfg->cmn_fifo_cfg[i].tx_fifo.virtual_addr = NULL;
	}
	cfg->phy_virt_res.iram_allocated_size = 0;
}

static int sipa_init_fifo_addr(struct device *dev, struct sipa_hal_context *cfg)
{
	int i, ret;

	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		ret = alloc_rx_fifo_ram(dev, cfg, i);
		if (ret)
			goto fail;
		ret = alloc_tx_fifo_ram(dev, cfg, i);
		if (ret)
			goto fail;
	}
	return 0;
fail:
	sipa_free_fifo_addr(dev, cfg);
	return ret;
}

static u32 sipa_init_fifo_reg_base(struct sipa_hal_context *cfg)
{
	int i;

	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		cfg->cmn_fifo_cfg[i].fifo_reg_base =
			cfg->phy_virt_res.glb_base +
			((i + 1) * SIPA_FIFO_REG_SIZE);
		cfg->cmn_fifo_cfg[i].fifo_phy_addr =
			cfg->phy_virt_res.glb_phy +
			((i + 1) * SIPA_FIFO_REG_SIZE);
	}

	return 0;
}

static void siap_init_hal_cfg(struct sipa_plat_drv_cfg *cfg)
{
	u32 i;
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;

	hal_cfg->ipa_intr = cfg->ipa_intr;

	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		hal_cfg->cmn_fifo_cfg[i].cur = cfg->common_fifo_cfg[i].src;
		hal_cfg->cmn_fifo_cfg[i].dst = cfg->common_fifo_cfg[i].dst;
		hal_cfg->cmn_fifo_cfg[i].is_recv =
			cfg->common_fifo_cfg[i].is_recv;
		hal_cfg->cmn_fifo_cfg[i].tx_fifo.depth =
			cfg->common_fifo_cfg[i].tx_fifo.fifo_size;
		hal_cfg->cmn_fifo_cfg[i].tx_fifo.in_iram =
			cfg->common_fifo_cfg[i].tx_fifo.in_iram;
		hal_cfg->cmn_fifo_cfg[i].rx_fifo.depth =
			cfg->common_fifo_cfg[i].rx_fifo.fifo_size;
		hal_cfg->cmn_fifo_cfg[i].rx_fifo.in_iram =
			cfg->common_fifo_cfg[i].rx_fifo.in_iram;
		hal_cfg->cmn_fifo_cfg[i].is_pam =
			cfg->common_fifo_cfg[i].is_pam;
		hal_cfg->cmn_fifo_cfg[i].fifo_id = i;
	}
}

static void sipa_remove_fifo_params(sipa_hal_hdl hdl,
				    enum sipa_cmn_fifo_index id)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_open_fifo_param *fifo_param = &hal_cfg->fifo_param[id];

	kfree(fifo_param->attr);
	kfree(fifo_param->ext_attr);
	fifo_param->attr = NULL;
	fifo_param->ext_attr = NULL;

	fifo_param->open_flag = false;
}

sipa_hal_hdl sipa_hal_init(struct device *dev, struct sipa_plat_drv_cfg *cfg)
{
	int ret;
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;

	hal_cfg->dev = dev;

	sipa_glb_ops_init(&hal_cfg->glb_ops);
	sipa_fifo_ops_init(&hal_cfg->fifo_ops);

	siap_init_hal_cfg(cfg);

	hal_cfg->phy_virt_res.glb_phy = cfg->glb_phy;
	hal_cfg->phy_virt_res.glb_size = cfg->glb_size;
	hal_cfg->phy_virt_res.glb_base =
		devm_ioremap(dev, cfg->glb_phy, cfg->glb_size);

	if (!hal_cfg->phy_virt_res.glb_base) {
		dev_err(dev, "remap glb_base fail\n");
		return NULL;
	}

	hal_cfg->phy_virt_res.iram_phy = cfg->iram_phy;
	hal_cfg->phy_virt_res.iram_size = cfg->iram_size;
	hal_cfg->phy_virt_res.iram_base =
		memremap((resource_size_t)cfg->iram_phy, (size_t)cfg->iram_size,
			 MEMREMAP_WC);
	if (!hal_cfg->phy_virt_res.iram_base) {
		dev_err(dev, "remap iram_base fail\n");
		return NULL;
	}

	ret = sipa_init_fifo_addr(dev, hal_cfg);
	if (ret)
		goto unmap_iram;
	sipa_init_fifo_reg_base(hal_cfg);
	/* The handler dereferences all of the mappings initialized above. */
	ret = devm_request_irq(dev, hal_cfg->ipa_intr, sipa_int_callback_func,
			       IRQF_NO_SUSPEND, "sprd,sipa", hal_cfg);
	if (ret) {
		sipa_free_fifo_addr(dev, hal_cfg);
		goto unmap_iram;
	}

	return ((sipa_hal_hdl)hal_cfg);
unmap_iram:
	memunmap(hal_cfg->phy_virt_res.iram_base);
	hal_cfg->phy_virt_res.iram_base = NULL;
	return NULL;
}
EXPORT_SYMBOL(sipa_hal_init);

int sipa_hal_set_enabled(struct sipa_plat_drv_cfg *cfg, bool enable)
{
	int ret = 0;
	u32 val = enable ? cfg->enable_mask : 0;

	if (cfg->sys_regmap) {
		ret = regmap_update_bits(cfg->sys_regmap, cfg->enable_reg,
					 cfg->enable_mask, val);
		if (ret < 0)
			pr_err("sipa: regmap update bits failed: %d\n", ret);
	} else {
		ret = -ENODEV;
	}
	return ret;
}
EXPORT_SYMBOL(sipa_hal_set_enabled);

int sipa_force_wakeup(struct sipa_plat_drv_cfg *cfg, bool wake)
{
	int ret = 0;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	if (!cfg->vpower)
		return ret;

	if (wake)
		ret = regulator_enable(cfg->vpower);
	else
		ret = regulator_disable(cfg->vpower);

	if (ret < 0)
		dev_err(ctrl->ctx->pdev, "enable vpower failed");

	return ret;
}
EXPORT_SYMBOL(sipa_force_wakeup);

/* A powered-off non-PAM open stages the saved configuration for resume.
 * PAM callers must own power; a hardware failure leaves rings allocated and
 * returns -EUCLEAN because resetting a partially live FIFO is not a rollback.
 */
int sipa_open_common_fifo(sipa_hal_hdl hdl, enum sipa_cmn_fifo_index fifo,
			  struct sipa_comm_fifo_params *attr,
			  struct sipa_ext_fifo_params *ext_attr,
			  bool force_sw_intr, sipa_hal_notify_cb cb, void *priv)
{
	struct sipa_hal_context *hal = hdl;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_common_fifo_cfg_tag *fc;
	struct sipa_open_fifo_param saved = {};
	u32 rx_depth, tx_depth;
	int ret;
	bool tx_allocated = false;

	if (!hal || !ctrl || !attr || fifo < 0 || fifo >= SIPA_FIFO_MAX)
		return -EINVAL;
	fc = &hal->cmn_fifo_cfg[fifo];
	rx_depth = ext_attr ? ext_attr->rx_depth : fc->rx_fifo.depth;
	tx_depth = ext_attr ? ext_attr->tx_depth : fc->tx_fifo.depth;
	if (!is_power_of_2(rx_depth) || !is_power_of_2(tx_depth) ||
	    rx_depth > 0x8000 || tx_depth > 0x8000 ||
	    attr->tx_intr_threshold > tx_depth ||
	    attr->tx_intr_delay_us > U16_MAX || attr->flow_ctrl_cfg > 3 ||
	    attr->flow_ctrl_irq_mode > 2 ||
	    attr->tx_enter_flowctrl_watermark > tx_depth ||
	    attr->tx_leave_flowctrl_watermark > tx_depth ||
	    attr->rx_enter_flowctrl_watermark > rx_depth ||
	    attr->rx_leave_flowctrl_watermark > rx_depth ||
	    (ext_attr &&
	     (ext_attr->rx_fifo_pah > 0xff || ext_attr->tx_fifo_pah > 0xff)))
		return -EINVAL;
	if (ext_attr && fc->state &&
	    (rx_depth != fc->rx_fifo.depth || tx_depth != fc->tx_fifo.depth ||
	     ext_attr->rx_fifo_pal != fc->rx_fifo.fifo_base_addr_l ||
	     ext_attr->rx_fifo_pah != fc->rx_fifo.fifo_base_addr_h ||
	     ext_attr->tx_fifo_pal != fc->tx_fifo.fifo_base_addr_l ||
	     ext_attr->tx_fifo_pah != fc->tx_fifo.fifo_base_addr_h))
		return -EBUSY;
	if (fc->is_pam && !READ_ONCE(ctrl->params_cfg.enable_cnt))
		return -EHOSTDOWN;
	/* Allocate every fallible software object before publishing callbacks. */
	saved.attr = kmemdup(attr, sizeof(*attr), GFP_KERNEL);
	if (!saved.attr)
		return -ENOMEM;
	if (ext_attr) {
		saved.ext_attr =
			kmemdup(ext_attr, sizeof(*ext_attr), GFP_KERNEL);
		if (!saved.ext_attr) {
			ret = -ENOMEM;
			goto free_saved;
		}
	}
	if (!fc->is_pam || (fc->tx_fifo.in_iram && fc->rx_fifo.in_iram)) {
		if (!kfifo_initialized(&fc->tx_priv_fifo)) {
			ret = kfifo_alloc(
				&fc->tx_priv_fifo,
				tx_depth *
					sizeof(struct sipa_node_description_tag),
				GFP_KERNEL);
			if (ret)
				goto free_saved;
			tx_allocated = true;
		}
		if (!kfifo_initialized(&fc->rx_priv_fifo)) {
			ret = kfifo_alloc(
				&fc->rx_priv_fifo,
				rx_depth *
					sizeof(struct sipa_node_description_tag),
				GFP_KERNEL);
			if (ret) {
				if (tx_allocated)
					kfifo_free(&fc->tx_priv_fifo);
				goto free_saved;
			}
		}
	}
	if ((kfifo_initialized(&fc->tx_priv_fifo) &&
	     kfifo_size(&fc->tx_priv_fifo) <
		     tx_depth * sizeof(struct sipa_node_description_tag)) ||
	    (kfifo_initialized(&fc->rx_priv_fifo) &&
	     kfifo_size(&fc->rx_priv_fifo) <
		     rx_depth * sizeof(struct sipa_node_description_tag))) {
		ret = -EINVAL;
		goto free_saved;
	}
	if (ext_attr) {
		fc->rx_fifo.depth = rx_depth;
		fc->rx_fifo.fifo_base_addr_l = ext_attr->rx_fifo_pal;
		fc->rx_fifo.fifo_base_addr_h = ext_attr->rx_fifo_pah;
		fc->rx_fifo.virtual_addr = ext_attr->rx_fifo_va;
		fc->tx_fifo.depth = tx_depth;
		fc->tx_fifo.fifo_base_addr_l = ext_attr->tx_fifo_pal;
		fc->tx_fifo.fifo_base_addr_h = ext_attr->tx_fifo_pah;
		fc->tx_fifo.virtual_addr = ext_attr->tx_fifo_va;
	}
	if (!READ_ONCE(ctrl->params_cfg.enable_cnt))
		goto publish;
	ret = -EUCLEAN;
	if (!hal->fifo_ops.open(fifo, hal->cmn_fifo_cfg, NULL))
		goto free_saved;
	if (!force_sw_intr && fc->is_pam) {
		if (!hal->fifo_ops.set_hw_interrupt_threshold(
			    fifo, hal->cmn_fifo_cfg, !!attr->tx_intr_threshold,
			    attr->tx_intr_threshold, NULL) ||
		    !hal->fifo_ops.set_hw_interrupt_timeout(
			    fifo, hal->cmn_fifo_cfg, !!attr->tx_intr_delay_us,
			    attr->tx_intr_delay_us, NULL))
			goto free_saved;
	} else {
		if (!hal->fifo_ops.set_interrupt_threshold(
			    fifo, hal->cmn_fifo_cfg, !!attr->tx_intr_threshold,
			    attr->tx_intr_threshold, NULL) ||
		    !hal->fifo_ops.set_interrupt_timeout(
			    fifo, hal->cmn_fifo_cfg, !!attr->tx_intr_delay_us,
			    attr->tx_intr_delay_us, NULL))
			goto free_saved;
	}
	if (fc->is_recv) {
		if (!hal->fifo_ops.enable_remote_flowctrl_interrupt(
			    fifo, hal->cmn_fifo_cfg, attr->flow_ctrl_cfg,
			    attr->tx_enter_flowctrl_watermark,
			    attr->tx_leave_flowctrl_watermark,
			    attr->rx_enter_flowctrl_watermark,
			    attr->rx_leave_flowctrl_watermark))
			goto free_saved;
	} else if (!hal->fifo_ops.enable_local_flowctrl_interrupt(
			   fifo, hal->cmn_fifo_cfg, 1, attr->flow_ctrl_irq_mode,
			   NULL)) {
		goto free_saved;
	}
	if (!hal->fifo_ops.set_interrupt_txfifo_full(
		    fifo, hal->cmn_fifo_cfg, attr->flowctrl_in_tx_full, NULL))
		goto free_saved;

publish:
	saved.force_sw_intr = force_sw_intr;
	saved.cb = cb;
	saved.priv = priv;
	saved.open_flag = true;
	sipa_remove_fifo_params(hdl, fifo);
	hal->fifo_param[fifo] = saved;
	/* Readers cannot observe the callback before its private data. */
	sipa_hal_detach_fifo_callback(hdl, fifo);
	/* Restore the saved callback after detaching the previous IRQ user. */
	hal->fifo_param[fifo].cb = cb;
	hal->fifo_param[fifo].priv = priv;
	WRITE_ONCE(fc->priv, priv);
	/* Publish callback private data before the IRQ handler can call it. */
	smp_store_release(&fc->fifo_irq_callback, cb);
	return 0;
free_saved:
	kfree(saved.ext_attr);
	kfree(saved.attr);
	return ret;
}
EXPORT_SYMBOL(sipa_open_common_fifo);

void sipa_hal_detach_fifo_callback(sipa_hal_hdl hdl,
				   enum sipa_cmn_fifo_index fifo)
{
	struct sipa_hal_context *hal = hdl;
	struct sipa_common_fifo_cfg_tag *fc;

	if (!hal || fifo < 0 || fifo >= SIPA_FIFO_MAX)
		return;
	fc = &hal->cmn_fifo_cfg[fifo];
	WRITE_ONCE(fc->fifo_irq_callback, NULL);
	synchronize_irq(hal->ipa_intr);
	WRITE_ONCE(fc->priv, NULL);
	hal->fifo_param[fifo].cb = NULL;
	hal->fifo_param[fifo].priv = NULL;
}

int sipa_close_common_fifo(sipa_hal_hdl hdl, enum sipa_cmn_fifo_index fifo)
{
	struct sipa_hal_context *hal = hdl;
	struct sipa_common_fifo_cfg_tag *fc;

	if (!hal || fifo < 0 || fifo >= SIPA_FIFO_MAX)
		return -EINVAL;
	fc = &hal->cmn_fifo_cfg[fifo];
	/* Caller must first stop its producer and prove external DMA idle. */
	if (!hal->fifo_ops.close(fifo, hal->cmn_fifo_cfg))
		return -EIO;
	WRITE_ONCE(fc->fifo_irq_callback, NULL);
	synchronize_irq(hal->ipa_intr);
	WRITE_ONCE(fc->priv, NULL);
	sipa_remove_fifo_params(hdl, fifo);
	return 0;
}
EXPORT_SYMBOL(sipa_close_common_fifo);

static int sipa_resume_cmn_fifo_ptr(sipa_hal_hdl hdl,
				    enum sipa_cmn_fifo_index id,
				    struct sipa_plat_drv_cfg *cfg)
{
	u32 depth;
	struct sipa_hal_context *hal_cfg = hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();

	switch (id) {
	case SIPA_FIFO_CP_UL:
		if (!cfg->wiap_ul_dma)
			return 0;
		depth = fifo_cfg[id].rx_fifo.depth;
		if (!hal_cfg->fifo_ops.set_rx_fifo_wptr(id, fifo_cfg, depth))
			return -EIO;
		break;
	case SIPA_FIFO_CP_DL:
		depth = fifo_cfg[id].tx_fifo.depth;
		if (!hal_cfg->fifo_ops.set_tx_fifo_wptr(id, fifo_cfg, depth))
			return -EIO;
		break;
	case SIPA_FIFO_AP_IP_DL:
		depth = fifo_cfg[id].rx_fifo.depth;
		if (sipa_hal_check_rx_priv_fifo_is_full(&sipa_hal_ctx, id) &&
		    sipa_fill_free_node(ctrl->receiver[SIPA_PKT_IP], depth))
			return -EIO;
		if (!hal_cfg->fifo_ops.set_rx_fifo_wptr(id, fifo_cfg, depth))
			return -EIO;
		break;
	case SIPA_FIFO_AP_ETH_UL:
		depth = fifo_cfg[id].rx_fifo.depth;
		if (sipa_hal_check_rx_priv_fifo_is_full(&sipa_hal_ctx, id) &&
		    sipa_fill_free_node(ctrl->receiver[SIPA_PKT_ETH], depth))
			return -EIO;
		if (!hal_cfg->fifo_ops.set_rx_fifo_wptr(id, fifo_cfg, depth))
			return -EIO;
		break;
	default:
		dev_info(ctrl->ctx->pdev,
			 "this fifo %d do not need to update ptr", id);
		break;
	}

	return 0;
}

int sipa_resume_common_fifo(sipa_hal_hdl hdl, struct sipa_plat_drv_cfg *cfg)
{
	int i, ret;
	struct sipa_open_fifo_param *iter;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;

	for (i = 0; i < SIPA_FIFO_MAX; i++) {
		iter = &hal_cfg->fifo_param[i];

		if (!iter->open_flag)
			continue;

		if (hal_cfg->fifo_ops.get_tx_depth(i, hal_cfg->cmn_fifo_cfg))
			continue;

		hal_cfg->cmn_fifo_cfg[i].state = false;
		ret = sipa_open_common_fifo(hdl, i, iter->attr, iter->ext_attr,
					    iter->force_sw_intr, iter->cb,
					    iter->priv);
		if (ret)
			return ret;
		ret = sipa_resume_cmn_fifo_ptr(hdl, i, cfg);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(sipa_resume_common_fifo);

bool sipa_hal_cmn_fifo_open_status(sipa_hal_hdl hdl,
				   enum sipa_cmn_fifo_index fifo)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg =
		&hal_cfg->cmn_fifo_cfg[fifo];

	return fifo_cfg->state;
}
EXPORT_SYMBOL(sipa_hal_cmn_fifo_open_status);

int sipa_tft_mode_init(sipa_hal_hdl hdl)
{
	int ret;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;
	void __iomem *glb_addr;

	if (!hal_cfg)
		return -EINVAL;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;
	glb_addr = hal_cfg->phy_virt_res.glb_base;

	ret = hal_cfg->fifo_ops.set_cur_dst_term(
		SIPA_FIFO_PCIE_UL, fifo_cfg, SIPA_TERM_PCIE0, SIPA_TERM_VCP);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->fifo_ops.set_cur_dst_term(
		SIPA_FIFO_PCIE_DL, fifo_cfg, SIPA_TERM_PCIE0, SIPA_TERM_VCP);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->glb_ops.set_cp_ul_cur_num(glb_addr, SIPA_TERM_VCP);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->glb_ops.set_cp_ul_dst_num(glb_addr, SIPA_TERM_PCIE0);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->glb_ops.set_cp_dl_cur_num(glb_addr, SIPA_TERM_VCP);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->glb_ops.set_cp_dl_dst_num(glb_addr, SIPA_TERM_PCIE0);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->glb_ops.enable_from_pcie_no_mac(glb_addr, 1);
	if (unlikely(!ret))
		return -EIO;

	ret = hal_cfg->glb_ops.enable_to_pcie_no_mac(glb_addr, 1);
	if (unlikely(!ret))
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(sipa_tft_mode_init);

/*
 * stop : true : stop recv false : start receive
 */
int sipa_hal_cmn_fifo_set_receive(sipa_hal_hdl hdl,
				  enum sipa_cmn_fifo_index fifo_id, bool stop)
{
	u32 ret;
	struct sipa_hal_context *hal_cfg;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	if (!hdl || fifo_id < 0 || fifo_id >= SIPA_FIFO_MAX)
		return -EINVAL;
	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	ret = hal_cfg->fifo_ops.ctrl_receive(fifo_id, fifo_cfg, stop);

	if (ret)
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL(sipa_hal_cmn_fifo_set_receive);

int sipa_hal_init_set_tx_fifo(sipa_hal_hdl hdl,
			      enum sipa_cmn_fifo_index fifo_id,
			      struct sipa_hal_fifo_item *items, u32 num)
{
	u32 ret, i;
	struct sipa_hal_context *hal_cfg;
	struct sipa_node_description_tag node;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	memset(&node, 0, sizeof(node));
	for (i = 0; i < num; i++) {
		node.address = (items + i)->addr;
		node.length = (items + i)->len;
		node.dst = (items + i)->dst;
		node.offset = (items + i)->offset;
		ret = hal_cfg->fifo_ops.put_node_to_tx_fifo(
			hal_cfg->dev, fifo_id, fifo_cfg, &node, 0, 1);
		if (ret == 0) {
			dev_err(hal_cfg->dev, "put node to tx fifo %d fail\n",
				fifo_id);
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(sipa_hal_init_set_tx_fifo);

int sipa_hal_get_tx_fifo_item(sipa_hal_hdl hdl,
			      enum sipa_cmn_fifo_index fifo_id,
			      struct sipa_hal_fifo_item *item)
{
	u32 ret;
	struct sipa_hal_context *hal_cfg;
	struct sipa_node_description_tag node;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	ret = hal_cfg->fifo_ops.recv_node_from_tx_fifo(hal_cfg->dev, fifo_id,
						       fifo_cfg, &node, 0, 1);

	if (ret == 0) {
		dev_err(hal_cfg->dev, "get node from tx fifo %d fail\n",
			fifo_id);
		return -1;
	}

	item->addr = node.address;
	item->len = node.length;
	item->dst = node.dst;
	item->offset = node.offset;
	item->src = node.src;
	item->err_code = node.err_code;
	item->netid = node.net_id;
	item->intr = node.intr;

	return 0;
}
EXPORT_SYMBOL(sipa_hal_get_tx_fifo_item);

u32 sipa_hal_get_tx_fifo_items(sipa_hal_hdl hdl,
			       enum sipa_cmn_fifo_index fifo_id)
{
	u32 num;
	struct sipa_node_description_tag node;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;

	num = hal_cfg->fifo_ops.recv_node_from_tx_fifo(hal_cfg->dev, fifo_id,
						       fifo_cfg, &node, 0, -1);
	if (!num)
		dev_warn(hal_cfg->dev, "fifo id %d tx fifo don't have node\n",
			 fifo_id);

	return num;
}
EXPORT_SYMBOL(sipa_hal_get_tx_fifo_items);

int sipa_hal_recv_conversion_node_to_item(sipa_hal_hdl hdl,
					  enum sipa_cmn_fifo_index fifo_id,
					  struct sipa_hal_fifo_item *item,
					  u32 index)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;
	struct sipa_node_description_tag *node =
		hal_cfg->fifo_ops.get_tx_fifo_node(fifo_id, fifo_cfg, index);

	if (unlikely(!node))
		return -EINVAL;

	if (READ_ONCE(fast_descriptor_decode)) {
		/* The coherent ring is page aligned and every descriptor is 16
		 * bytes. Its producer pointer/barrier was already consumed before
		 * this lookup. Snapshot the two little-endian words once: packed
		 * bitfields otherwise emit many slow byte loads from DMA memory.
		 */
		const __le64 *words = (const void *)node;
		u64 low, high;

		BUILD_BUG_ON(sizeof(*node) != 16);
		low = le64_to_cpu(READ_ONCE(words[0]));
		high = le64_to_cpu(READ_ONCE(words[1]));
		sipa_decode_node_words(item, low, high);
		return 0;
	}

	item->addr = node->address;
	item->len = node->length;
	item->dst = node->dst;
	item->offset = node->offset;
	item->src = node->src;
	item->err_code = node->err_code;
	item->netid = node->net_id;
	item->intr = node->intr;

	return 0;
}
EXPORT_SYMBOL(sipa_hal_recv_conversion_node_to_item);

int sipa_hal_conversion_node_to_item(sipa_hal_hdl hdl,
				     enum sipa_cmn_fifo_index fifo_id,
				     struct sipa_hal_fifo_item *item)
{
	struct sipa_node_description_tag node;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;

	if (!kfifo_out(&((fifo_cfg + fifo_id)->tx_priv_fifo), &node,
		       sizeof(node)))
		return 0;

	item->addr = node.address;
	item->len = node.length;
	item->dst = node.dst;
	item->offset = node.offset;
	item->src = node.src;
	item->err_code = node.err_code;
	item->netid = node.net_id;
	item->intr = node.intr;

	return 0;
}
EXPORT_SYMBOL(sipa_hal_conversion_node_to_item);

int sipa_hal_set_tx_fifo_rptr(sipa_hal_hdl hdl, enum sipa_cmn_fifo_index id,
			      u32 num)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;

	return hal_cfg->fifo_ops.update_tx_fifo_rptr(id, fifo_cfg, num);
}
EXPORT_SYMBOL(sipa_hal_set_tx_fifo_rptr);

int sipa_hal_get_cmn_fifo_filled_depth(sipa_hal_hdl hdl,
				       enum sipa_cmn_fifo_index fifo_id,
				       u32 *rx_filled, u32 *tx_filled)
{
	u32 ret;
	struct sipa_hal_context *hal_cfg;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	if (rx_filled)
		*rx_filled = 0;
	if (tx_filled)
		*tx_filled = 0;
	if (!hdl || fifo_id < 0 || fifo_id >= SIPA_FIFO_MAX ||
	    (!rx_filled && !tx_filled))
		return -EINVAL;
	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	/* The common FIFO operation returns TRUE/FALSE, not Linux errno. */
	ret = hal_cfg->fifo_ops.get_filled_depth(fifo_id, fifo_cfg, rx_filled,
						 tx_filled);

	return ret ? 0 : -EIO;
}
EXPORT_SYMBOL(sipa_hal_get_cmn_fifo_filled_depth);

int sipa_hal_enable_wiap_dma(sipa_hal_hdl hdl, bool dma)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;

	return hal_cfg->glb_ops.enable_wiap_ul_dma(
		hal_cfg->phy_virt_res.glb_base, dma);
}
EXPORT_SYMBOL(sipa_hal_enable_wiap_dma);

/* After the peer is drained and SIPA is paused, every consumed free-buffer
 * descriptor must have produced a packet descriptor, and every produced
 * packet must have been consumed by the peer. Unconsumed RX free buffers stay
 * in the allocated ring. This does not infer DMA idle from the pause bit.
 */
int sipa_hal_pam_fifo_idle(sipa_hal_hdl hdl, enum sipa_cmn_fifo_index id)
{
	struct sipa_hal_context *hal = hdl;
	struct sipa_common_fifo_cfg_tag *fc;
	u32 tx_rd, tx_wr, rx_rd, rx_wr, mask;

	if (!hal || id < 0 || id >= SIPA_FIFO_MAX)
		return -EINVAL;
	fc = &hal->cmn_fifo_cfg[id];
	if (!fc->is_pam || !is_power_of_2(fc->rx_fifo.depth) ||
	    fc->rx_fifo.depth > 0x8000 ||
	    fc->rx_fifo.depth != fc->tx_fifo.depth)
		return -EINVAL;
	if (!hal->fifo_ops.get_tx_fifo_wr_rd_ptr(id, hal->cmn_fifo_cfg, &tx_wr,
						 &tx_rd) ||
	    !hal->fifo_ops.get_rx_fifo_wr_rd_ptr(id, hal->cmn_fifo_cfg, &rx_wr,
						 &rx_rd))
		return -EIO;
	mask = (fc->rx_fifo.depth << 1) - 1;
	if (((tx_rd | tx_wr | rx_rd | rx_wr) & ~mask) ||
	    ((rx_wr - rx_rd) & mask) > fc->rx_fifo.depth)
		return -EIO;
	return tx_rd == tx_wr && rx_rd == tx_wr ? 0 : -EAGAIN;
}

/**
 * sipa_hal_reclaim_unuse_node() - reclaim that unfree description.
 * @hdl: &sipa_hal_ctx.
 * @fifo_id: the cmn fifo that need to reclaim.
 *
 * Some node descriptions that sent out may not be free normally,
 * so wo need use soft method to reclaim these node description.
 *
 * Return:
 *	0: reclaim success.
 *	negative value: reclaim fail.
 */
int sipa_hal_reclaim_unuse_node(sipa_hal_hdl hdl,
				enum sipa_cmn_fifo_index fifo_id)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	if (!hal_cfg || fifo_id < 0 || fifo_id >= SIPA_FIFO_MAX)
		return -EINVAL;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	return hal_cfg->fifo_ops.reclaim_node_desc(fifo_id, fifo_cfg);
}
EXPORT_SYMBOL(sipa_hal_reclaim_unuse_node);

int sipa_hal_put_rx_fifo_item(sipa_hal_hdl hdl,
			      enum sipa_cmn_fifo_index fifo_id,
			      struct sipa_hal_fifo_item *item)
{
	u32 ret;
	struct sipa_hal_context *hal_cfg;
	struct sipa_node_description_tag node;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	memset(&node, 0, sizeof(node));
	node.address = item->addr;
	node.length = item->len;
	node.dst = item->dst;
	node.offset = item->offset;
	node.src = item->src;
	node.err_code = item->err_code;
	node.net_id = item->netid;
	node.intr = item->intr;

	ret = hal_cfg->fifo_ops.put_node_to_rx_fifo(hal_cfg->dev, fifo_id,
						    fifo_cfg, &node, 0, 1);
	if (ret == 0) {
		dev_err(hal_cfg->dev, "put node to rx fifo %d fail\n", fifo_id);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(sipa_hal_put_rx_fifo_item);

int sipa_hal_put_rx_fifo_items(sipa_hal_hdl hdl,
			       enum sipa_cmn_fifo_index fifo_id)
{
	int ret;
	u32 num;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;

	num = kfifo_len(&((fifo_cfg + fifo_id)->rx_priv_fifo));
	num /= sizeof(struct sipa_node_description_tag);
	if (num) {
		ret = hal_cfg->fifo_ops.put_node_to_rx_fifo(
			hal_cfg->dev, fifo_id, fifo_cfg, NULL, 0, num);
		if (ret < 0 || (u32)ret != num) {
			dev_err(hal_cfg->dev, "put node to rx fifo %d fail\n",
				fifo_id);
			return ret < 0 ? ret : -ENOSPC;
		}
	} else {
		dev_err(hal_cfg->dev, "fifo id %d rx priv fifo is empty",
			fifo_id);
	}

	return 0;
}
EXPORT_SYMBOL(sipa_hal_put_rx_fifo_items);

int sipa_hal_cache_rx_fifo_item(sipa_hal_hdl hdl,
				enum sipa_cmn_fifo_index fifo_id,
				struct sipa_hal_fifo_item *item)
{
	u32 ret;
	struct sipa_node_description_tag node;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg =
		&hal_cfg->cmn_fifo_cfg[fifo_id];

	memset(&node, 0, sizeof(node));
	node.address = item->addr;
	node.length = item->len;
	node.dst = item->dst;
	node.offset = item->offset;
	node.src = item->src;
	node.err_code = item->err_code;
	node.net_id = item->netid;
	node.intr = item->intr;
	node.reserved = item->reserved;

	/* A byte FIFO must never accept a partial hardware descriptor. */
	if (kfifo_avail(&fifo_cfg->rx_priv_fifo) < sizeof(node))
		return -ENOSPC;
	ret = kfifo_in(&fifo_cfg->rx_priv_fifo, &node, sizeof(node));
	if (ret != sizeof(node)) {
		dev_err(hal_cfg->dev, "fifo id %d rx priv fifo is full\n",
			fifo_id);
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(sipa_hal_cache_rx_fifo_item);

bool sipa_hal_is_rx_fifo_empty(sipa_hal_hdl hdl,
			       enum sipa_cmn_fifo_index fifo_id)
{
	u32 ret;
	struct sipa_hal_context *hal_cfg;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	ret = hal_cfg->fifo_ops.get_rx_empty_status(fifo_id, fifo_cfg);

	return ret;
}
EXPORT_SYMBOL(sipa_hal_is_rx_fifo_empty);

bool sipa_hal_check_rx_priv_fifo_is_empty(sipa_hal_hdl hdl,
					  enum sipa_cmn_fifo_index fifo_id)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg =
		&hal_cfg->cmn_fifo_cfg[fifo_id];

	return kfifo_is_empty(&fifo_cfg->rx_priv_fifo);
}
EXPORT_SYMBOL(sipa_hal_check_rx_priv_fifo_is_empty);

bool sipa_hal_check_rx_priv_fifo_is_full(sipa_hal_hdl hdl,
					 enum sipa_cmn_fifo_index fifo_id)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg =
		&hal_cfg->cmn_fifo_cfg[fifo_id];

	return kfifo_is_full(&fifo_cfg->rx_priv_fifo);
}
EXPORT_SYMBOL(sipa_hal_check_rx_priv_fifo_is_full);

bool sipa_hal_is_rx_fifo_full(sipa_hal_hdl hdl,
			      enum sipa_cmn_fifo_index fifo_id)
{
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg = hal_cfg->cmn_fifo_cfg;

	return hal_cfg->fifo_ops.get_rx_full_status(fifo_id, fifo_cfg);
}
EXPORT_SYMBOL(sipa_hal_is_rx_fifo_full);

bool sipa_hal_bk_fifo_node(sipa_hal_hdl hdl, enum sipa_cmn_fifo_index fifo_id)
{
	u32 ret;
	ssize_t node_size = sizeof(struct sipa_node_description_tag);
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg =
		&hal_cfg->cmn_fifo_cfg[fifo_id];

	if (!fifo_cfg->state || !kfifo_initialized(&fifo_cfg->rx_priv_fifo) ||
	    !kfifo_initialized(&fifo_cfg->tx_priv_fifo) || !fifo_cfg->is_pam)
		return false;

	kfifo_reset(&fifo_cfg->tx_priv_fifo);
	kfifo_reset(&fifo_cfg->rx_priv_fifo);
	ret = kfifo_in(&fifo_cfg->rx_priv_fifo, fifo_cfg->rx_fifo.virtual_addr,
		       fifo_cfg->rx_fifo.depth * node_size);
	if (ret != fifo_cfg->rx_fifo.depth * node_size)
		pr_err("backup %d rx fifo failed ret = %d\n", fifo_id, ret);

	ret = kfifo_in(&fifo_cfg->tx_priv_fifo, fifo_cfg->tx_fifo.virtual_addr,
		       fifo_cfg->tx_fifo.depth * node_size);
	if (ret != fifo_cfg->tx_fifo.depth * node_size)
		pr_err("backup %d tx fifo failed ret = %d\n", fifo_id, ret);

	return true;
}
EXPORT_SYMBOL(sipa_hal_bk_fifo_node);

bool sipa_hal_resume_fifo_node(sipa_hal_hdl hdl,
			       enum sipa_cmn_fifo_index fifo_id)
{
	u32 ret;
	ssize_t node_size = sizeof(struct sipa_node_description_tag);
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg =
		&hal_cfg->cmn_fifo_cfg[fifo_id];

	if (!kfifo_initialized(&fifo_cfg->rx_priv_fifo) ||
	    !kfifo_initialized(&fifo_cfg->tx_priv_fifo) || !fifo_cfg->is_pam)
		return false;

	if (!kfifo_is_full(&fifo_cfg->rx_priv_fifo) ||
	    !kfifo_is_full(&fifo_cfg->tx_priv_fifo))
		return false;

	if (!hal_cfg->fifo_ops.get_tx_depth(fifo_id, hal_cfg->cmn_fifo_cfg))
		return false;

	ret = kfifo_out(&fifo_cfg->rx_priv_fifo, fifo_cfg->rx_fifo.virtual_addr,
			fifo_cfg->rx_fifo.depth * node_size);
	if (ret != fifo_cfg->rx_fifo.depth * node_size)
		pr_err("resume %d rx fifo node failed\n", fifo_id);

	ret = kfifo_out(&fifo_cfg->tx_priv_fifo, fifo_cfg->tx_fifo.virtual_addr,
			fifo_cfg->tx_fifo.depth * node_size);
	if (ret != fifo_cfg->tx_fifo.depth * node_size)
		pr_err("resume %d tx fifo node failed\n", fifo_id);

	return true;
}
EXPORT_SYMBOL(sipa_hal_resume_fifo_node);

bool sipa_hal_is_tx_fifo_empty(sipa_hal_hdl hdl,
			       enum sipa_cmn_fifo_index fifo_id)
{
	u32 ret;
	struct sipa_hal_context *hal_cfg;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	hal_cfg = (struct sipa_hal_context *)hdl;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	ret = hal_cfg->fifo_ops.get_tx_empty_status(fifo_id, fifo_cfg);

	return ret;
}
EXPORT_SYMBOL(sipa_hal_is_tx_fifo_empty);

/**
 * sipa_hal_check_send_cmn_fifo_com() - Check cmn fifo send and free whether
 * completion
 * @hdl: &sipa_hal_ctx
 * @fifo_id: The cmn fifo id that need to be check.
 *
 * If fifo is not CP_DL, its tx/rx fifo's wptr/rptr should be equal,
 * if fifo is CP_DL, its wptr of tx fifo should be greated than rx fifo
 * wptr and rptr total depth.
 *
 * Return:
 *	true: send and free complete.
 *	false: send or free not complete.
 */
bool sipa_hal_check_send_cmn_fifo_com(sipa_hal_hdl hdl,
				      enum sipa_cmn_fifo_index fifo_id)
{
	bool status;
	u32 ret, tx_rptr, tx_wptr, rx_rptr, rx_wptr, depth;
	struct sipa_hal_context *hal_cfg = (struct sipa_hal_context *)hdl;
	struct sipa_common_fifo_cfg_tag *fifo_cfg;

	if (!hal_cfg || fifo_id < 0 || fifo_id >= SIPA_FIFO_MAX)
		return false;
	fifo_cfg = hal_cfg->cmn_fifo_cfg;

	depth = (fifo_cfg + fifo_id)->tx_fifo.depth;
	ret = hal_cfg->fifo_ops.get_tx_empty_status(fifo_id, fifo_cfg);
	if (!hal_cfg->fifo_ops.get_tx_fifo_wr_rd_ptr(fifo_id, fifo_cfg,
						     &tx_wptr, &tx_rptr) ||
	    !hal_cfg->fifo_ops.get_rx_fifo_wr_rd_ptr(fifo_id, fifo_cfg,
						     &rx_wptr, &rx_rptr))
		return false;
	if (fifo_id != SIPA_FIFO_CP_DL) {
		status = (tx_wptr == tx_rptr) && (tx_rptr == rx_wptr) &&
			 (rx_wptr == rx_rptr);
	} else {
		status = (rx_wptr == rx_rptr) &&
			 (((depth | (depth - 1)) & (rx_wptr + depth)) ==
			  tx_wptr);
		ret = true;
	}

	if (status && ret)
		return true;

	return false;
}
EXPORT_SYMBOL(sipa_hal_check_send_cmn_fifo_com);

int sipa_hal_init_pam_param(enum sipa_cmn_fifo_index dl_idx,
			    enum sipa_cmn_fifo_index ul_idx,
			    struct sipa_to_pam_info *out)
{
	struct sipa_common_fifo_cfg_tag *dl_fifo_cfg, *ul_fifo_cfg;
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;

	dl_fifo_cfg = &hal_cfg->cmn_fifo_cfg[dl_idx];
	ul_fifo_cfg = &hal_cfg->cmn_fifo_cfg[ul_idx];

	out->dl_fifo.tx_fifo_base_addr =
		IPA_STI_64BIT(dl_fifo_cfg->tx_fifo.fifo_base_addr_l,
			      dl_fifo_cfg->tx_fifo.fifo_base_addr_h);
	out->dl_fifo.rx_fifo_base_addr =
		IPA_STI_64BIT(dl_fifo_cfg->rx_fifo.fifo_base_addr_l,
			      dl_fifo_cfg->rx_fifo.fifo_base_addr_h);

	out->ul_fifo.tx_fifo_base_addr =
		IPA_STI_64BIT(ul_fifo_cfg->tx_fifo.fifo_base_addr_l,
			      ul_fifo_cfg->tx_fifo.fifo_base_addr_h);
	out->ul_fifo.rx_fifo_base_addr =
		IPA_STI_64BIT(ul_fifo_cfg->rx_fifo.fifo_base_addr_l,
			      ul_fifo_cfg->rx_fifo.fifo_base_addr_h);

	out->dl_fifo.fifo_sts_addr = dl_fifo_cfg->fifo_phy_addr;
	out->ul_fifo.fifo_sts_addr = ul_fifo_cfg->fifo_phy_addr;

	out->dl_fifo.fifo_depth = dl_fifo_cfg->tx_fifo.depth;
	out->ul_fifo.fifo_depth = ul_fifo_cfg->tx_fifo.depth;

	return 0;
}
EXPORT_SYMBOL(sipa_hal_init_pam_param);

int sipa_swap_hash_table(struct sipa_hash_table *new_tbl,
			 struct sipa_hash_table *old_tbl)
{
	u32 len, addrl, addrh;
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;

	if (old_tbl) {
		hal_cfg->glb_ops.get_hash_table(hal_cfg->phy_virt_res.glb_base,
						&addrl, &addrh, &len);
		old_tbl->tbl_phy_addr = IPA_STI_64BIT(addrl, addrh);
		old_tbl->depth = len;
	}

	if (new_tbl) {
		addrl = IPA_GET_LOW32(new_tbl->tbl_phy_addr);
		addrh = IPA_GET_HIGH32(new_tbl->tbl_phy_addr);
		/* Order coherent table stores before publishing its DMA address.
		 * This does not establish old-table retirement after DONE.
		 */
		dma_wmb();
		if (!hal_cfg->glb_ops.hash_table_switch(
			    hal_cfg->phy_virt_res.glb_base, addrl, addrh,
			    new_tbl->depth))
			return -ETIMEDOUT;
		/*
		 * Because return done does not mean that the hash table is
		 * actually written to ddr, so add delay ensure that IPA gets
		 * the latest hash table.
		 */
		udelay(500);
	}

	return 0;
}
EXPORT_SYMBOL(sipa_swap_hash_table);

int sipa_hal_ctrl_action(u32 enable)
{
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;

	return hal_cfg->glb_ops.ctrl_ipa_action(hal_cfg->phy_virt_res.glb_base,
						enable);
}
EXPORT_SYMBOL(sipa_hal_ctrl_action);

bool sipa_hal_get_resume_status(void)
{
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;
	void __iomem *base = hal_cfg->phy_virt_res.glb_base;

	return hal_cfg->glb_ops.get_resume_status(base);
}
EXPORT_SYMBOL(sipa_hal_get_resume_status);

bool sipa_hal_get_pause_status(void)
{
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;
	void __iomem *base = hal_cfg->phy_virt_res.glb_base;

	return hal_cfg->glb_ops.get_pause_status(base);
}
EXPORT_SYMBOL(sipa_hal_get_pause_status);

/* Mode_n_Flow_Ctrl [30:29]: 00 RNDIS, 01 MBIM; 10/11 reserved.
 * This-board AP setters (85134c8/8513990) call the same recovered glb operation.
 * Only USB-format bits change; callers serialize all global restoration.
 */
int sipa_hal_set_usb_format(enum sipa_usb_format format)
{
	struct sipa_hal_context *hal = &sipa_hal_ctx;
	void __iomem *base = hal->phy_virt_res.glb_base;
	u32 mask = GENMASK(30, 29), value;

	if (format != SIPA_USB_RNDIS && format != SIPA_USB_MBIM)
		return -EINVAL;
	if (!base)
		return -ENODEV;
	value = (readl(base) & ~mask) | ((u32)format << 29);
	writel(value, base);
	return (readl(base) & mask) == ((u32)format << 29) ? 0 : -EIO;
}

int sipa_resume_glb_reg_cfg(struct sipa_plat_drv_cfg *cfg)
{
	struct sipa_hal_context *hal_cfg = &sipa_hal_ctx;
	void __iomem *glb_base = hal_cfg->phy_virt_res.glb_base;

	if (!hal_cfg->glb_ops.enable_wiap_ul_dma(glb_base, cfg->wiap_ul_dma) ||
	    !hal_cfg->glb_ops.enable_cp_through_pcie(glb_base,
						     cfg->need_through_pcie) ||
	    !hal_cfg->glb_ops.enable_def_flowctrl_to_src_blk(glb_base) ||
	    !hal_cfg->glb_ops.set_mode(glb_base, cfg->is_bypass))
		return -EIO;
	if (sipa_hal_set_usb_format(cfg->usb_format))
		return -EIO;
	if (cfg->is_bypass)
		hal_cfg->glb_ops.enable_pcie_intr_write_reg_mode(glb_base, 1);
#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
	return sipa_hal_ft_punt_restore();
#else
	return 0;
#endif
}
EXPORT_SYMBOL(sipa_resume_glb_reg_cfg);

#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
/* sipa_glb_phy.h: TCP_FIN/RST leave-to-AP bits in INTERNAL_FIFO_CLR. */
#define SIPA_FT_PUNT_REG 0x38
#define SIPA_FT_TCP_PUNT_BITS (BIT(31) | BIT(29))
static u32 sipa_ft_required_punt;

/* Caller serializes register restoration with ctrl->resume_lock. */
int sipa_hal_ft_punt_restore(void)
{
	struct sipa_hal_context *hal = &sipa_hal_ctx;
	u32 required = READ_ONCE(sipa_ft_required_punt);

	if (!required)
		return 0; /* No FT has requested ownership of these global bits. */
	if (!hal->phy_virt_res.glb_base || !hal->glb_ops.set_force_to_ap)
		return -ENODEV;
	if (!hal->glb_ops.set_force_to_ap(hal->phy_virt_res.glb_base, 1,
					  required) ||
	    (readl(hal->phy_virt_res.glb_base + SIPA_FT_PUNT_REG) & required) !=
		    required)
		return -EIO;
	return 0;
}

/*
 * Checked register transaction only. HW_READY_FOR_CHECK is bit 26, not the
 * software PAUSE request at bit 27. A caller still needs a proven contract
 * that the acknowledgment drains every old-table read/write before reuse.
 * On any uncertain outcome, keep both DMA tables and do not claim deletion.
 */
int sipa_hal_ft_transaction(const struct sipa_hash_table *next,
			    const struct sipa_hash_table *expected,
			    bool *uncertain)
{
	struct sipa_hal_context *hal = &sipa_hal_ctx;
	struct sipa_control *ctrl = sipa_get_ctrl_pointer();
	struct sipa_hash_table observed;
	u32 low, high, depth;
	int ret = 0;

	if (!next || !expected || !uncertain)
		return -EINVAL;
	*uncertain = false;
	if (!ctrl || !ctrl->ctx || !hal->phy_virt_res.glb_base ||
	    !hal->glb_ops.get_hash_table || !hal->glb_ops.hash_table_switch ||
	    !hal->glb_ops.get_hw_ready_to_check_sts)
		return -ENODEV;
	if (next->depth != 2048 || next->tbl_phy_addr > DMA_BIT_MASK(32))
		return -ERANGE;

	mutex_lock(&ctrl->resume_lock);
	if (!READ_ONCE(ctrl->power_flag) || READ_ONCE(ctrl->suspend_stage)) {
		ret = -EHOSTDOWN;
		goto out;
	}
	if (!hal->glb_ops.get_hash_table(hal->phy_virt_res.glb_base, &low,
					 &high, &depth)) {
		ret = -EIO;
		goto out;
	}
	observed.tbl_phy_addr = IPA_STI_64BIT(low, high);
	observed.depth = depth;
	if (observed.tbl_phy_addr != expected->tbl_phy_addr ||
	    observed.depth != expected->depth) {
		/* A foreign/firmware-owned table must never be overwritten. */
		ret = -EBUSY;
		goto out;
	}
	*uncertain = true;
	/* The vendor API returns ONE on success and ZERO on failure. */
	if (sipa_hal_ctrl_action(false) != 1 ||
	    !hal->glb_ops.get_hw_ready_to_check_sts(
		    hal->phy_virt_res.glb_base)) {
		ret = -ETIMEDOUT;
		goto out;
	}
	WRITE_ONCE(sipa_ft_required_punt, SIPA_FT_TCP_PUNT_BITS);
	ret = sipa_hal_ft_punt_restore();
	if (ret)
		goto out;
	dma_wmb();
	if (!hal->glb_ops.hash_table_switch(hal->phy_virt_res.glb_base,
					    lower_32_bits(next->tbl_phy_addr),
					    upper_32_bits(next->tbl_phy_addr),
					    next->depth)) {
		ret = -ETIMEDOUT;
		goto out;
	}
	if (!hal->glb_ops.get_hash_table(hal->phy_virt_res.glb_base, &low,
					 &high, &depth) ||
	    IPA_STI_64BIT(low, high) != next->tbl_phy_addr ||
	    depth != next->depth ||
	    !hal->glb_ops.get_hw_ready_to_check_sts(
		    hal->phy_virt_res.glb_base)) {
		ret = -EIO;
		goto out;
	}
	/* No fixed delay is treated as retirement evidence. */
	if (sipa_hal_ctrl_action(true) != 1 ||
	    hal->glb_ops.get_hw_ready_to_check_sts(
		    hal->phy_virt_res.glb_base)) {
		ret = -ETIMEDOUT;
		goto out;
	}
	*uncertain = false;
out:
	mutex_unlock(&ctrl->resume_lock);
	return ret;
}
#endif
