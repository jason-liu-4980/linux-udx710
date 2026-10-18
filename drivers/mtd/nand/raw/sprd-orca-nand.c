// SPDX-License-Identifier: GPL-2.0-only
/*
 * UDX710/Orca raw NAND controller, mainline API adaptation.
 * Hardware sequencer derived from Spreadtrum Communications Inc. (C) 2019,
 * GPL v2, strongtz/linux-sprd d834a127aa46356c63bd5a59bc6d86c66ea371be.
 * New raw-NAND frontend: no vendor nfc_base MTD implementation is retained.
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include "sprd-orca-nand-params.h"
#include "sprd-orca-nand-regs.h"
#define NFC_MAX_ID_LEN 8

struct sprd_nand_param {
	u8 id[NFC_MAX_ID_LEN];
	u32 nblkcnt;
	u32 npage_per_blk;
	u32 nsect_per_page;
	u32 nsect_size;
	u32 nspare_size;

	u32 page_size; /* no contain spare */
	u32 main_size; /* page_size */
	u32 spare_size;

	u32 badflag_pos;
	u32 badflag_len;
	u32 ecc_mode;
	u32 ecc_pos;
	u32 ecc_size;
	u32 info_pos;
	u32 info_size;

	u8 nbus_width;
	u8 ncycle;

	/* ACS */
	u32 t_als;
	u32 t_cls;
	/* ACE */
	u32 t_clh;
	u32 t_alh;
	/* RWS */
	u32 t_rr;
	u32 t_adl;
	/* RWE */

	/* RWH & RWL */
	u32 t_wh; /* we high when write */
	u32 t_wp; /* we low  when write */
	u32 t_reh; /* re high when read */
	u32 t_rp; /* re low when read */
};

struct nand_inst {
	u8 *program_name;
	u32 cnt;
	u32 step_tag;
	u32 int_bits;
	u32 inst[NFC_MAX_MC_INST_NUM];
};

struct sprd_nand_host {
	struct nand_controller controller;
	struct nand_chip chip;
	struct device_node *flash_node;
	bool clocks_on;
	/* resource0: interrupt id & clock name &  register bass address */
	int irq;
	u32 frequency;
	struct device *dev;
	void __iomem *ioaddr;
	struct clk *clk_ahb_enable_gate;
	struct clk *clk_nand;
	struct clk *clk_ecc;
	struct clk *clk_parent_sdr;
	struct clk *clk_parent_ddr;
	struct clk *clk_ahb_ecc;
	struct clk *clk_ahb_26m;
	struct clk *clk_nand_1x;
	struct clk *clk_nand_2x;

	/* resource1: nand param & mtd ecclayout & risk threshold */
	struct sprd_nand_param param;
	u32 eccmode_reg;
	u32 risk_threshold;
	u32 cs[CFG0_CS_MAX];
	u32 csnum;

	/* u32 csDis; */
	u32 sect_perpage_shift;
	u32 sectshift;
	u32 sectmsk;
	u32 pageshift;
	u32 blkshift;
	u32 bufshift;
	u32 badblkcnt;
	/* resource3: local DMA buffer */
	dma_addr_t mbuf_p;
	u8 *mbuf_v;
	dma_addr_t sbuf_p;
	u8 *sbuf_v;
	dma_addr_t stsbuf_p;
	u8 *stsbuf_v;
	dma_addr_t seedbuf_p;
	u32 *seedbuf_v;
	u32 mbuf_size;
	u32 sbuf_size;
	u32 stsbuf_size;
	u32 seedbuf_size;
	/*
	 * resource4: register base value. some value is const
	 * while operate on nand flash, we store it to local valuable.
	 * Time & cfg & status mach. It is different with operation R W E.
	 */
	u32 nfc_time0_r;
	u32 nfc_time0_w;
	u32 nfc_time0_e;

	u32 nfc_start;
	u32 nfc_cfg0;
	u32 nfc_cfg1;
	u32 nfc_cfg2;
	u32 nfc_cfg3;
	u32 nfc_cfg4;

	u32 nfc_time0_val;
	u32 nfc_cfg0_val;
	u32 nfc_sts_mach_val;

	struct nand_inst inst_read_main_spare;
	struct nand_inst inst_read_main_raw;
	struct nand_inst inst_read_spare_raw;
	struct nand_inst inst_write_main_spare;
	struct nand_inst inst_write_main_raw;
	struct nand_inst inst_write_spare_raw;
	struct nand_inst inst_erase;

	bool randomizer;
};

static inline void sprd_nand_cmd_init(struct nand_inst *cmd_inst, u32 position)
{
	cmd_inst->int_bits = position;
}

static inline void sprd_nand_cmd_add(struct nand_inst *cmd_inst, u16 inst_p)
{
	cmd_inst->inst[cmd_inst->cnt] = inst_p;
	cmd_inst->cnt++;
}

static inline void sprd_nand_cmd_tag(struct nand_inst *cmd_inst)
{
	cmd_inst->step_tag = cmd_inst->cnt;
}

static inline void sprd_nand_cmd_change(struct sprd_nand_host *host,
					struct nand_inst *cmd_inst, u32 pg)
{
	cmd_inst->inst[cmd_inst->step_tag] =
		INST_ADDR(SPRD_NAND_PAGE_SHIFT_0(pg), 1);
	cmd_inst->inst[cmd_inst->step_tag + 1] =
		INST_ADDR(SPRD_NAND_PAGE_SHIFT_8(pg), 0);
	if (host->param.ncycle == 5)
		cmd_inst->inst[cmd_inst->step_tag + 2] =
			INST_ADDR(SPRD_NAND_PAGE_SHIFT_16(pg), 0);
}

struct sprd_ecc_dma_status {
	u16 ecc_stats[16];
	u32 layout4_ecc_stats;
	u32 freecount[5];
};

static struct nand_inst inst_reset = { "_inst_reset",
				       3,
				       0,
				       INT_TO | INT_DONE,
				       { INST_CMD(0xFF), INST_WRB0(),
					 INST_DONE() } };

static struct nand_inst inst_readid = { "_inst_readid",
					5,
					0,
					INT_TO | INT_DONE,
					{ INST_CMD(0x90), INST_ADDR(0, 0),
					  INST_INOP(10), INST_IDST(0x08),
					  INST_DONE() } };

static struct sprd_nand_timing default_timing = { 10, 25, 15 };

static const u32 seedtbl[64] = {
	0x056c, 0x1bc77, 0x5d356, 0x1f645d, 0x0fbc, 0x0090c, 0x7f880, 0x3d9e86,
	0x1717, 0x1e1ad, 0x6db67, 0x7d7ea0, 0x0a52, 0x0d564, 0x6fbac, 0x6823dd,
	0x07cf, 0x1cb3b, 0x37cd1, 0x5c91f0, 0x064e, 0x167a7, 0x0f1d2, 0x506be8,
	0x098c, 0x1bd54, 0x2c2af, 0x4b5fb7, 0x1399, 0x11690, 0x1d310, 0x27e53b,
	0x1246, 0x14794, 0x0f34f, 0x347bc4, 0x0150, 0x00787, 0x73450, 0x3d8927,
	0x11f1, 0x17bad, 0x46eaa, 0x5403f5, 0x1026, 0x173ab, 0x79634, 0x01b987,
	0x1c45, 0x08b63, 0x42924, 0x4bf708, 0x012a, 0x03a3a, 0x435d5, 0x1a7baa,
	0x0849, 0x1cb9b, 0x28350, 0x1e8309, 0x1d4c, 0x0af6e, 0x0949e, 0x00193a,
};

static inline void sprd_nand_writel(struct sprd_nand_host *host,
				    unsigned long val, unsigned long reg)
{
	writel_relaxed(val, host->ioaddr + reg);
}

static inline unsigned long sprd_nand_readl(struct sprd_nand_host *host,
					    unsigned long reg)
{
	return readl_relaxed(host->ioaddr + reg);
}

static void sprd_nand_pageseed(struct sprd_nand_host *host, u32 page)
{
	u32 i, j, remain, shift, mask, numbit;
	u32 offset = page >> 4;

	if (page & ~SPRD_NAND_PAGE_MASK)
		return;

	memset(host->seedbuf_v, 0, SEED_BUF_SIZE * 4);
	for (i = 0; i < SEED_TBL_SIZE; i++) {
		switch (i & 0x3) {
		case 0:
			numbit = 13;
			break;
		case 1:
			numbit = 17;
			break;
		case 2:
			numbit = 19;
			break;
		case 3:
			numbit = 23;
			break;
		}
		for (j = 0; j <= numbit - 1; j++) {
			if (seedtbl[i] & BIT(j))
				host->seedbuf_v[i] |= BIT(numbit - 1 - j);
		}
		if (offset) {
			if (offset > numbit - 1)
				shift = offset - numbit;
			else
				shift = offset;
			mask = (BIT(numbit) - 1) >> shift;
			remain = host->seedbuf_v[i] & ~mask;
			remain >>= numbit - shift;
			host->seedbuf_v[i] &= mask;
			host->seedbuf_v[i] <<= shift;
			host->seedbuf_v[i] |= remain;
		}
	}
	host->seedbuf_v[SEED_TBL_SIZE] = host->seedbuf_v[0];
	host->seedbuf_v[SEED_TBL_SIZE + 1] = host->seedbuf_v[1];
	host->seedbuf_v[SEED_TBL_SIZE + 2] = host->seedbuf_v[2];
	host->seedbuf_v[SEED_TBL_SIZE + 3] = host->seedbuf_v[3];
}

static void sprd_nand_enable_randomizer(struct sprd_nand_host *host, u32 page,
					u32 mode)
{
	u32 spl_page = SPL_MAX_SIZE / host->param.page_size;
	dma_addr_t seedaddr =
		host->seedbuf_p + PAGE2SEED_ADDR_OFFSET(page) * sizeof(u32);

	if (host->randomizer)
		return;
	if (page < spl_page && mode != MTD_OPS_RAW) {
		sprd_nand_pageseed(host, page);

		sprd_nand_writel(host, NFC_POLYNOMIALS0, NFC_POLY0_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS1, NFC_POLY1_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS2, NFC_POLY2_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS3, NFC_POLY3_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS0, NFC_POLY4_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS1, NFC_POLY5_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS2, NFC_POLY6_REG);
		sprd_nand_writel(host, NFC_POLYNOMIALS3, NFC_POLY7_REG);
		sprd_nand_writel(host, 0x0, NFC_SEED_ADDRH_REG);
		sprd_nand_writel(host, (unsigned long)seedaddr,
				 NFC_SEED_ADDRL_REG);
		host->nfc_cfg3 |= CFG3_RANDOM_EN | CFG3_POLY_4R1_EN;
		sprd_nand_writel(host, host->nfc_cfg3, NFC_CFG3_REG);
	}
}

static void sprd_nand_disable_randomizer(struct sprd_nand_host *host, u32 page,
					 u32 mode)
{
	u32 spl_page = SPL_MAX_SIZE / host->param.page_size;

	if (host->randomizer)
		return;
	if (page < spl_page && mode != MTD_OPS_RAW) {
		host->nfc_cfg3 &= ~(CFG3_POLY_4R1_EN | CFG3_RANDOM_EN);
		sprd_nand_writel(host, host->nfc_cfg3, NFC_CFG3_REG);
	}
}

static void sprd_nand_cmd_exec(struct sprd_nand_host *host,
			       struct nand_inst *program, u32 repeat,
			       u32 if_use_int)
{
	u32 i;

	host->nfc_cfg0_val |= CFG0_SET_REPEAT_NUM(repeat);
	sprd_nand_writel(host, host->nfc_cfg0_val, NFC_CFG0_REG);
	sprd_nand_writel(host, host->nfc_time0_val, NFC_TIMING0_REG);
	sprd_nand_writel(host, host->nfc_sts_mach_val, NFC_STAT_STSMCH_REG);
	for (i = 0; i < program->cnt; i += 2) {
		sprd_nand_writel(
			host, (program->inst[i] | (program->inst[i + 1] << 16)),
			NFC_INST00_REG + (i << 1));
	}

	sprd_nand_writel(host, 0, NFC_INT_REG);
	sprd_nand_writel(host, GENMASK(11, 8), NFC_INT_REG);
	if (if_use_int)
		sprd_nand_writel(host, (program->int_bits & 0xF), NFC_INT_REG);

	dma_wmb();
	sprd_nand_writel(host, host->nfc_start | CTRL_NFC_CMD_START,
			 NFC_START_REG);
}

static int sprd_nand_cmd_wait(struct sprd_nand_host *host,
			      struct nand_inst *program)
{
	int ret = -EIO;
	u32 nfc_timeout_val, regval;

	if (strcmp(program->program_name, "_inst_reset") == 0)
		nfc_timeout_val = NFC_RESET_TIMEOUT_VAL;
	else
		nfc_timeout_val = NFC_TIMEOUT_VAL;

	ret = readl_relaxed_poll_timeout(
		host->ioaddr + NFC_INT_REG, regval,
		(SPRD_NAND_GET_INT_VAL(regval) &
		 ((INT_TO | INT_STSMCH | INT_WP) & program->int_bits)) ||
			(SPRD_NAND_GET_INT_VAL(regval) &
			 (INT_DONE & program->int_bits)),
		1, nfc_timeout_val);
	dma_rmb();
	if (ret) {
		dev_err(host->dev, "command %s wait timeout.\n",
			program->program_name);
		return ret;
	}

	if (SPRD_NAND_GET_INT_VAL(regval) &
	    ((INT_TO | INT_STSMCH | INT_WP) & program->int_bits))
		ret = -EIO;

	if (ret)
		pr_err("sprd nand cmd %s fail", program->program_name);

	return ret;
}

static void sprd_nand_init_reg_state0(struct sprd_nand_host *host)
{
	host->nfc_cfg0 = CFG0_DEF0_MAST_ENDIAN | CFG0_DEF0_SECT_NUM_IN_INST |
			 CFG0_DEF0_DETECT_ALL_FF;
	host->nfc_cfg3 = CFG3_DETECT_ALL_FF;
	host->nfc_cfg4 = CFG4_SLICE_CLK_EN | CFG4_PHY_DLL_CLK_2X_EN;
	sprd_nand_writel(host, host->nfc_cfg0, NFC_CFG0_REG);
	sprd_nand_writel(host, host->nfc_cfg3, NFC_CFG3_REG);
	sprd_nand_writel(host, host->nfc_cfg4, NFC_CFG4_REG);

	sprd_nand_writel(host, RAM_MAIN_ADDR(-1), NFC_MAIN_ADDRH_REG);
	sprd_nand_writel(host, RAM_MAIN_ADDR(-1), NFC_MAIN_ADDRL_REG);
	sprd_nand_writel(host, RAM_SPAR_ADDR(-1), NFC_SPAR_ADDRH_REG);
	sprd_nand_writel(host, RAM_SPAR_ADDR(-1), NFC_SPAR_ADDRL_REG);
	sprd_nand_writel(host, RAM_STAT_ADDR(-1), NFC_STAT_ADDRH_REG);
	sprd_nand_writel(host, RAM_STAT_ADDR(-1), NFC_STAT_ADDRL_REG);
	/* need delay time after set NFC_CFG0_REG */
	usleep_range(1000, 1500);
}

static void sprd_nand_select_cs(struct sprd_nand_host *host, int cs)
{
	host->nfc_cfg0 = (host->nfc_cfg0 & CFG0_CS_MSKCLR) |
			 CFG0_SET_CS_SEL(cs);
}

static int sprd_nand_reset(struct sprd_nand_host *host)
{
	struct nand_inst *inst = &inst_reset;

	host->nfc_sts_mach_val = DEF0_MATCH;
	host->nfc_time0_val = host->nfc_time0_r;
	host->nfc_cfg0_val = host->nfc_cfg0 | CFG0_SET_NFC_MODE(0x2);
	sprd_nand_cmd_exec(host, inst, 1, 0);

	return sprd_nand_cmd_wait(host, inst);
}

static void sprd_nand_delect_cs(struct sprd_nand_host *host, int ret)
{
	if (ret == -EIO) {
		sprd_nand_writel(host, CTRL_NFC_CMD_CLR, NFC_START_REG);
		sprd_nand_reset(host);
	}
}

static u32 sprd_nand_timing_field(u32 ns, u32 clock_hz, u32 mask)
{
	/*
	 * The input clock is 2x the NAND clock; fields encode cycles minus one.
	 * Saturate before subtraction: at26MHz the old unsigned subtraction
	 * wrapped sub-cycle requirements to the longest31/63 encodings.
	 */
	u64 cycles = DIV_ROUND_UP_ULL((u64)ns * clock_hz, 2000000000ULL);

	return clamp_val(cycles, 1, mask + 1) - 1;
}

static void sprd_nand_set_timing_config(struct sprd_nand_host *host,
					struct sprd_nand_timing *timing,
					u32 clk_hz)
{
	u32 reg_val = (2u << NFC_ACS_OFFSET) | (2u << NFC_RWE_OFFSET);

	reg_val |= sprd_nand_timing_field(timing->ace_ns, clk_hz, 0x1f)
		   << NFC_ACE_OFFSET;
	reg_val |= sprd_nand_timing_field(20, clk_hz, 0x3f) << NFC_RWS_OFFSET;
	reg_val |= sprd_nand_timing_field(timing->rwh_ns, clk_hz, 0x1f)
		   << NFC_RWH_OFFSET;
	reg_val |= sprd_nand_timing_field(timing->rwl_ns, clk_hz, 0x3f);
	host->nfc_time0_r = reg_val;
	host->nfc_time0_w = reg_val;
	host->nfc_time0_e = reg_val;
	dev_info(host->dev, "NAND timing %08x at %u Hz\n", reg_val, clk_hz);
}

static struct sprd_nand_vendor_param *sprd_get_nand_param(u8 *id)
{
	struct sprd_nand_vendor_param *param = sprd_nand_vendor_param_table;
	u32 i;

	for (i = 0; i < 5; i++)
		pr_info("nand device id[%d] is %x\n", i, id[i]);

	while (param->idmaker != 0) {
		if (!memcmp(param->id, id, i))
			return param;
		param++;
	}
	pr_info("Nand params unconfig, please check it. Halt on booting!!!\n");

	return NULL;
}

static int sprd_nand_param_init_nandhw(struct sprd_nand_host *host)
{
	struct sprd_nand_vendor_param *param;

	/* get base param info */
	param = sprd_get_nand_param(host->param.id);
	if (!param)
		return -EINVAL;

	sprd_nand_set_timing_config(host, &param->s_timing, host->frequency);
	host->param.nblkcnt = param->blk_num;
	host->param.npage_per_blk = param->blk_size / param->page_size;
	host->param.nsect_per_page = param->page_size / param->nsect_size;
	host->param.nsect_size = param->nsect_size;
	host->param.nspare_size = param->s_oob.oob_size;
	host->param.page_size = param->page_size; /* no contain spare */
	host->param.main_size = param->page_size;
	host->param.spare_size = param->nspare_size;
	host->param.badflag_pos = 0; /* --- default:need fixed */
	host->param.badflag_len = 2; /* --- default:need fixed */
	host->param.ecc_mode = param->s_oob.ecc_bits;
	host->param.ecc_pos = param->s_oob.ecc_pos;
	host->param.ecc_size = param->s_oob.ecc_size;
	host->param.info_pos = param->s_oob.info_pos;
	host->param.info_size = param->s_oob.info_size;

	host->param.nbus_width = param->nbus_width;
	host->param.ncycle = param->ncycles;

	dev_dbg(host->dev, "nand:nBlkcnt = [%d], npageperblk = [%d]\n",
		host->param.nblkcnt, host->param.npage_per_blk);
	dev_dbg(host->dev, "nand:nsectperpage = [%d], nsecsize = [%d]\n",
		host->param.nsect_per_page, host->param.nsect_size);
	dev_dbg(host->dev, "nand:nspare_size = [%d], page_size = [%d]\n",
		host->param.nspare_size, host->param.page_size);
	dev_dbg(host->dev, "nand:page_size =  [%d], spare_size = [%d]\n",
		host->param.page_size, host->param.spare_size);
	dev_dbg(host->dev, "nand:ecc_mode = [%d], ecc_pos = [%d]\n",
		host->param.ecc_mode, host->param.ecc_pos);
	dev_dbg(host->dev,
		"nand:ecc_size = [%d], info_pos = [%d], info_size = [%d]\n",
		host->param.ecc_size, host->param.info_pos,
		host->param.info_size);

	/* setting globe param seting */
	switch (host->param.ecc_mode) {
	case 1:
		host->risk_threshold = 1;
		host->eccmode_reg = 0;
		break;
	case 2:
		host->risk_threshold = 1;
		host->eccmode_reg = 1;
		break;
	case 4:
		host->risk_threshold = 2;
		host->eccmode_reg = 2;
		break;
	case 8:
		host->risk_threshold = 4;
		host->eccmode_reg = 3;
		break;
	case 12:
		host->risk_threshold = 6;
		host->eccmode_reg = 4;
		break;
	case 16:
		host->risk_threshold = 8;
		host->eccmode_reg = 5;
		break;
	case 24:
		host->risk_threshold = 12;
		host->eccmode_reg = 6;
		break;
	case 40:
		host->risk_threshold = 20;
		host->eccmode_reg = 7;
		break;
	case 60:
		host->risk_threshold = 30;
		host->eccmode_reg = 8;
		break;
	default:
		dev_err(host->dev, "nand:sprd nand ecc mode not support!\n");
		return -EINVAL;
	}

	host->sect_perpage_shift = ffs(host->param.nsect_per_page) - 1;
	host->sectshift = ffs(host->param.nsect_size) - 1;
	host->sectmsk = BIT(host->sectshift) - 1;
	host->pageshift =
		ffs(host->param.nsect_per_page << host->sectshift) - 1;
	host->blkshift = ffs(host->param.npage_per_blk << host->pageshift) - 1;
	host->mbuf_size = max_t(u32, NFC_MBUF_SIZE, host->param.page_size);
	host->bufshift = ffs(host->mbuf_size) - 1;
	host->bufshift = min(host->bufshift, host->blkshift);
	dev_dbg(host->dev, "nand: sectperpgshift %d, sectshift %d\n",
		host->sect_perpage_shift, host->sectshift);
	dev_dbg(host->dev,
		"nand:secmsk %d, pageshift %d,blkshift %d, bufshift %d\n",
		host->sectmsk, host->pageshift, host->blkshift, host->bufshift);

	return 0;
}

static void sprd_nand_param_init_inst(struct sprd_nand_host *host)
{
	u32 column = BIT(host->pageshift);

	dev_dbg(host->dev,
		"nand:param init inst column %d,host->pageshift %d\n", column,
		host->pageshift);
	if (host->param.nbus_width == BW_16) {
		pr_debug("nand:buswidth = 16\n");
		column >>= 1;
	}
	/* erase */
	host->inst_erase.program_name = "_inst_erase";
	sprd_nand_cmd_init(&host->inst_erase,
			   INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(&host->inst_erase, INST_CMD(0x60));
	sprd_nand_cmd_tag(&host->inst_erase);
	sprd_nand_cmd_add(&host->inst_erase, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_erase, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_erase, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_erase, INST_CMD(0xD0));
	sprd_nand_cmd_add(&host->inst_erase, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_erase, INST_CMD(0x70));
	sprd_nand_cmd_add(&host->inst_erase, INST_IDST(1));
	sprd_nand_cmd_add(&host->inst_erase, INST_DONE());
	/* read main+spare(info)+Ecc or Raw */
	host->inst_read_main_spare.program_name = "_inst_read_main_spare";
	sprd_nand_cmd_init(&host->inst_read_main_spare, INT_TO | INT_DONE);
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_CMD(0x00));
	sprd_nand_cmd_add(&host->inst_read_main_spare,
			  INST_ADDR((0xFF & (u8)column), 0));
	sprd_nand_cmd_add(&host->inst_read_main_spare,
			  INST_ADDR((0xFF & (u8)(column >> 8)), 0));
	sprd_nand_cmd_tag(&host->inst_read_main_spare);
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_read_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_CMD(0x30));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_SRDT());

	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_CMD(0x05));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_CMD(0xE0));
	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_MRDT());

	sprd_nand_cmd_add(&host->inst_read_main_spare, INST_DONE());
	/* read main raw */
	host->inst_read_main_raw.program_name = "_inst_read_main_raw";
	sprd_nand_cmd_init(&host->inst_read_main_raw, INT_TO | INT_DONE);
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_CMD(0x00));
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_tag(&host->inst_read_main_raw);
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_read_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_CMD(0x30));
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_MRDT());
	sprd_nand_cmd_add(&host->inst_read_main_raw, INST_DONE());
	/*
	 * read spare raw: read only main or only spare data,
	 * it is read to main addr.
	 */
	host->inst_read_spare_raw.program_name = "_inst_read_spare_raw";
	sprd_nand_cmd_init(&host->inst_read_spare_raw, INT_TO | INT_DONE);
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_CMD(0x00));
	sprd_nand_cmd_add(&host->inst_read_spare_raw,
			  INST_ADDR((0xFF & (u8)column), 0));
	sprd_nand_cmd_add(&host->inst_read_spare_raw,
			  INST_ADDR((0xFF & (u8)(column >> 8)), 0));
	sprd_nand_cmd_tag(&host->inst_read_spare_raw);
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_CMD(0x30));
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_MRDT());
	sprd_nand_cmd_add(&host->inst_read_spare_raw, INST_DONE());
	/* write main+spare(info)+ecc */
	host->inst_write_main_spare.program_name = "_inst_write_main_spare";
	sprd_nand_cmd_init(&host->inst_write_main_spare,
			   INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_CMD(0x80));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_ADDR(0, 0));
	sprd_nand_cmd_tag(&host->inst_write_main_spare);
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_write_main_spare,
				  INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_MWDT());

	/* just need input column addr */
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_CMD(0x85));
	sprd_nand_cmd_add(&host->inst_write_main_spare,
			  INST_ADDR((0xFF & (u8)column), 0));
	sprd_nand_cmd_add(&host->inst_write_main_spare,
			  INST_ADDR((0xFF & (u8)(column >> 8)), 0));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_SWDT());

	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_CMD(0x10));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_CMD(0x70));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_IDST(1));
	sprd_nand_cmd_add(&host->inst_write_main_spare, INST_DONE());
	/* write main raw */
	host->inst_write_main_raw.program_name = "_inst_write_main_raw";
	sprd_nand_cmd_init(&host->inst_write_main_raw,
			   INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_CMD(0x80));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_tag(&host->inst_write_main_raw);
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_write_main_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_MWDT());
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_CMD(0x10));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_CMD(0x70));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_IDST(1));
	sprd_nand_cmd_add(&host->inst_write_main_raw, INST_DONE());
	/* write spare raw */
	host->inst_write_spare_raw.program_name = "_inst_write_spare_raw";
	sprd_nand_cmd_init(&host->inst_write_spare_raw,
			   INT_TO | INT_DONE | INT_WP | INT_STSMCH);
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_CMD(0x80));
	sprd_nand_cmd_add(&host->inst_write_spare_raw,
			  INST_ADDR((0xFF & column), 0));
	sprd_nand_cmd_add(&host->inst_write_spare_raw,
			  INST_ADDR((0xFF & (column >> 8)), 0));
	sprd_nand_cmd_tag(&host->inst_write_spare_raw);
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_ADDR(0, 1));
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_ADDR(0, 0));
	if (host->param.ncycle == 5)
		sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_ADDR(0, 0));
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_SWDT());
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_CMD(0x10));
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_WRB0());
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_CMD(0x70));
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_IDST(1));
	sprd_nand_cmd_add(&host->inst_write_spare_raw, INST_DONE());
}

static void sprd_nand_init_reg_state1(struct sprd_nand_host *host)
{
	host->nfc_start |= CTRL_DEF1_ECC_MODE(host->eccmode_reg);
	host->nfc_cfg0 |= CFG0_DEF1_SECT_NUM(host->param.nsect_per_page) |
			  CFG0_DEF1_BUS_WIDTH(host->param.nbus_width) |
			  CFG0_DEF1_MAIN_SPAR_APT(host->param.nsect_per_page);
	host->nfc_cfg1 = CFG1_DEF1_SPAR_INFO_SIZE(host->param.info_size) |
			 CFG1_DEF1_SPAR_SIZE(host->param.nspare_size) |
			 CFG1_DEF_MAIN_SIZE(host->param.nsect_size);
	host->nfc_cfg2 = CFG2_DEF1_SPAR_SECTOR_NUM(host->param.nsect_per_page) |
			 CFG2_DEF1_SPAR_INFO_POS(host->param.info_pos) |
			 CFG2_DEF1_ECC_POSITION(host->param.ecc_pos);

	sprd_nand_writel(host, host->nfc_cfg0, NFC_CFG0_REG);
	sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
	sprd_nand_writel(host, host->nfc_cfg2, NFC_CFG2_REG);
	sprd_nand_writel(host, 0xFFFFFFFF, NFC_TIMEOUT_REG);

	sprd_nand_writel(host, 0x0, NFC_STAT_ADDRH_REG);
	sprd_nand_writel(host, RAM_STAT_ADDR(host->stsbuf_p),
			 NFC_STAT_ADDRL_REG);
	sprd_nand_writel(host, 0x0, NFC_MAIN_ADDRH_REG);
	sprd_nand_writel(host, RAM_MAIN_ADDR(host->mbuf_p), NFC_MAIN_ADDRL_REG);
	sprd_nand_writel(host, 0x0, NFC_SPAR_ADDRH_REG);
	sprd_nand_writel(host, RAM_SPAR_ADDR(host->sbuf_p), NFC_SPAR_ADDRL_REG);
	/* need delay time after set NFC_CFG0_REG */
	usleep_range(1000, 1500);
}

/* DMA status is one bank per page. Core handles aggregate failed statistics. */
static int sprd_nand_ecc_analyze(struct sprd_nand_host *host, u32 num,
				 struct mtd_ecc_stats *stats, u32 mode)
{
	struct sprd_ecc_dma_status *st = (void *)host->stsbuf_v;
	int bitflips = 0, ret;
	unsigned int i;

	for (i = 0; i < num; i++) {
		unsigned int bank = i / host->param.nsect_per_page;
		unsigned int slot = i % host->param.nsect_per_page;
		u16 value = st[bank].ecc_stats[slot];
		unsigned int state = ECC_STAT(value), count = ECC_NUM(value);

		if (!state)
			continue;
		if (state == 1 && count != 0x1ff &&
		    count <= host->param.ecc_mode) {
			stats->corrected += count;
			bitflips = max_t(int, bitflips, count);
			continue;
		}
		ret = nand_check_erased_ecc_chunk(
			host->mbuf_v + i * host->param.nsect_size,
			host->param.nsect_size,
			host->sbuf_v + i * host->param.nspare_size,
			host->param.nspare_size, NULL, 0, host->param.ecc_mode);
		if (ret < 0)
			stats->failed++;
		else {
			stats->corrected += ret;
			bitflips = max(bitflips, ret);
		}
	}
	return bitflips;
}

static int sprd_nand_alloc_dma(struct sprd_nand_host *host)
{
	host->sbuf_size = host->mbuf_size / host->param.nsect_size *
			  host->param.nspare_size;
	host->stsbuf_size = host->mbuf_size / host->param.page_size *
			    sizeof(struct sprd_ecc_dma_status);
	host->seedbuf_size = SEED_BUF_SIZE * sizeof(u32);
	host->mbuf_v = dmam_alloc_coherent(host->dev, host->mbuf_size,
					   &host->mbuf_p, GFP_KERNEL);
	host->sbuf_v = dmam_alloc_coherent(host->dev, host->sbuf_size,
					   &host->sbuf_p, GFP_KERNEL);
	host->stsbuf_v = dmam_alloc_coherent(host->dev, host->stsbuf_size,
					     &host->stsbuf_p, GFP_KERNEL);
	host->seedbuf_v = dmam_alloc_coherent(host->dev, host->seedbuf_size,
					      &host->seedbuf_p, GFP_KERNEL);
	if (!host->mbuf_v || !host->sbuf_v || !host->stsbuf_v ||
	    !host->seedbuf_v)
		return -ENOMEM;
	return 0;
}
static int sprd_nand_read_page_in_block(struct sprd_nand_host *host, u32 page,
					u32 num, u32 *ret_num, u32 if_has_mbuf,
					u32 if_has_sbuf, u32 mode,
					struct mtd_ecc_stats *ecc_sts)
{
	struct nand_inst *inst = NULL;
	int ret;
	u32 if_change_buf = 0;

	struct nand_inst inst_read_main_spare = host->inst_read_main_spare;
	struct nand_inst inst_read_main_raw = host->inst_read_main_raw;
	struct nand_inst inst_read_spare_raw = host->inst_read_spare_raw;

	sprd_nand_select_cs(host, 0);
	host->nfc_sts_mach_val = MACH_READ;
	host->nfc_time0_val = host->nfc_time0_r;
	host->nfc_cfg0_val = (host->nfc_cfg0 | CFG0_SET_NFC_MODE(0));
	sprd_nand_enable_randomizer(host, page, mode);

	switch (mode) {
	case MTD_OPS_AUTO_OOB:
		host->nfc_cfg0_val |= CFG0_SET_SPARE_ONLY_INFO_PROD_EN;
		fallthrough;
	case MTD_OPS_PLACE_OOB:
		host->nfc_cfg0_val |= (CFG0_SET_ECC_EN | CFG0_SET_MAIN_USE |
				       CFG0_SET_SPAR_USE);
		inst = &inst_read_main_spare;
		break;
	case MTD_OPS_RAW:
		if (if_has_mbuf & if_has_sbuf) {
			host->nfc_cfg0_val |=
				(CFG0_SET_MAIN_USE | CFG0_SET_SPAR_USE);
			inst = &inst_read_main_spare;
		} else if (if_has_mbuf) {
			host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
			inst = &inst_read_main_raw;
		} else if (if_has_sbuf) {
			host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
			inst = &inst_read_spare_raw;
			/*
			 * 0 nand controller use mainAddr to send spare data,
			 * So i have to change some globe config here
			 * if_change_buf = 1;
			 * 1 change to main buf
			 */
			if_change_buf = 1;
			sprd_nand_writel(host, 0x0, NFC_MAIN_ADDRH_REG);
			sprd_nand_writel(host, RAM_MAIN_ADDR(host->sbuf_p),
					 NFC_MAIN_ADDRL_REG);
			/* 2 change page_size */
			host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
			host->nfc_cfg1 |=
				CFG1_DEF_MAIN_SIZE(host->param.nspare_size
						   << host->sect_perpage_shift);
			sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
			/* 3 change sect number */
			host->nfc_cfg0_val &= (~CFG0_SET_SECT_NUM_MSK);
			host->nfc_cfg0_val |= CFG0_DEF1_SECT_NUM(1);
		} else {
			sprd_nand_delect_cs(host, -EINVAL);
			return -EINVAL;
		}
		break;
	default:
		pr_err("nand:sprd nand ops mode error!\n");
		break;
	}

	sprd_nand_cmd_change(host, inst, page);
	sprd_nand_cmd_exec(host, inst, num, 0);
	ret = sprd_nand_cmd_wait(host, inst);
	if (if_change_buf) {
		/* 1 change to main buf */
		sprd_nand_writel(host, 0x0, NFC_MAIN_ADDRH_REG);
		sprd_nand_writel(host, RAM_MAIN_ADDR(host->mbuf_p),
				 NFC_MAIN_ADDRL_REG);
		/* 2 change page_size */
		host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
		host->nfc_cfg1 |= CFG1_DEF_MAIN_SIZE(host->param.nsect_size);
		sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
		/* 3 change sect number */
		sprd_nand_writel(host, host->nfc_cfg0, NFC_CFG0_REG);
	}
	sprd_nand_disable_randomizer(host, page, mode);

	if (ret) {
		*ret_num = 0;
		sprd_nand_delect_cs(host, -EIO);
		return -EIO;
	}
	*ret_num = num;

	switch (mode) {
	case MTD_OPS_AUTO_OOB:
	case MTD_OPS_PLACE_OOB:
		ret = sprd_nand_ecc_analyze(
			host, num * host->param.nsect_per_page, ecc_sts, mode);
		if (ret < 0)
			pr_err("nand:page %d, ecc error ret %d\n", page, ret);
		break;
	case MTD_OPS_RAW:
		break;
	default:
		dev_err(host->dev, "nand:sprd nand ops mode error!\n");
		break;
	}
	sprd_nand_delect_cs(host, ret);
	return ret;
}

static void sprd_nand_disable_wp(struct sprd_nand_host *host)
{
	u32 reg_nfc_cfg0;

	reg_nfc_cfg0 = sprd_nand_readl(host, NFC_CFG0_REG);
	reg_nfc_cfg0 |= CFG0_SET_WPN;
	sprd_nand_writel(host, reg_nfc_cfg0, NFC_CFG0_REG);
	/* need delay time after set NFC_CFG0_REG */
	udelay(1);
}

static void sprd_nand_enable_wp(struct sprd_nand_host *host)
{
	u32 reg_nfc_cfg0;

	reg_nfc_cfg0 = sprd_nand_readl(host, NFC_CFG0_REG);
	reg_nfc_cfg0 &= ~CFG0_SET_WPN;
	sprd_nand_writel(host, reg_nfc_cfg0, NFC_CFG0_REG);
	/* need delay time after set NFC_CFG0_REG */
	udelay(1);
}

static int sprd_nand_write_page_in_blk(struct sprd_nand_host *host, u32 page,
				       u32 num, u32 *ret_num, u32 if_has_mbuf,
				       u32 if_has_sbuf, u32 mode)
{
	struct nand_inst *inst = NULL;
	int ret;
	u32 if_change_buf = 0;
	struct nand_inst inst_write_main_spare = host->inst_write_main_spare;
	struct nand_inst inst_write_spare_raw = host->inst_write_spare_raw;
	struct nand_inst inst_write_main_raw = host->inst_write_main_raw;

	sprd_nand_select_cs(host, 0);
	sprd_nand_disable_wp(host);
	host->nfc_cfg0 |= CFG0_SET_WPN;

	host->nfc_sts_mach_val = MACH_WRITE;
	host->nfc_time0_val = host->nfc_time0_w;
	host->nfc_cfg0_val =
		(host->nfc_cfg0 | CFG0_SET_NFC_MODE(0) | CFG0_SET_NFC_RW);
	sprd_nand_enable_randomizer(host, page, mode);

	switch (mode) {
	case MTD_OPS_AUTO_OOB:
		host->nfc_cfg0_val |= CFG0_SET_SPARE_ONLY_INFO_PROD_EN;
		fallthrough;
	case MTD_OPS_PLACE_OOB:
		host->nfc_cfg0_val |= (CFG0_SET_ECC_EN | CFG0_SET_MAIN_USE |
				       CFG0_SET_SPAR_USE);
		inst = &inst_write_main_spare;
		break;
	case MTD_OPS_RAW:
		if (if_has_mbuf & if_has_sbuf) {
			host->nfc_cfg0_val |=
				(CFG0_SET_MAIN_USE | CFG0_SET_SPAR_USE);
			inst = &inst_write_main_spare;
		} else if (if_has_mbuf) {
			host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
			inst = &inst_write_main_raw;
		} else if (if_has_sbuf) {
			/* use main to write spare area */
			host->nfc_cfg0_val |= (CFG0_SET_MAIN_USE);
			inst = &inst_write_spare_raw;
			/*
			 * 0 nand controller use mainAddr to send spare data,
			 * So i have to change some globe config here
			 * if_change_buf = 1;
			 * 1 change to main buf
			 */
			if_change_buf = 1;
			sprd_nand_writel(host, 0x0, NFC_MAIN_ADDRH_REG);
			sprd_nand_writel(host, RAM_MAIN_ADDR(host->sbuf_p),
					 NFC_MAIN_ADDRL_REG);
			/* 2 change page_size */
			host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
			host->nfc_cfg1 |=
				CFG1_DEF_MAIN_SIZE(host->param.nspare_size
						   << host->sect_perpage_shift);
			sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
			/* 3 change sect number */
			host->nfc_cfg0_val &= (~CFG0_SET_SECT_NUM_MSK);
			host->nfc_cfg0_val |= CFG0_DEF1_SECT_NUM(1);
		} else {
			sprd_nand_enable_wp(host);
			host->nfc_cfg0 &= ~(CFG0_SET_WPN);
			sprd_nand_delect_cs(host, -EINVAL);
			return -EINVAL;
		}
		break;
	default:
		pr_err("nand:sprd nand ops mode error!\n");
		break;
	}

	sprd_nand_cmd_change(host, inst, page);
	sprd_nand_cmd_exec(host, inst, num, 0);
	ret = sprd_nand_cmd_wait(host, inst);
	if (if_change_buf) {
		/* 1 change to main buf */
		sprd_nand_writel(host, 0x0, NFC_MAIN_ADDRH_REG);
		sprd_nand_writel(host, RAM_MAIN_ADDR(host->mbuf_p),
				 NFC_MAIN_ADDRL_REG);
		/* 2 change page_size */
		host->nfc_cfg1 &= (~CFG1_temp_MIAN_SIZE_MSK);
		host->nfc_cfg1 |= CFG1_DEF_MAIN_SIZE(host->param.nsect_size);
		sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
		/* 3 change sect number */
		sprd_nand_writel(host, host->nfc_cfg0, NFC_CFG0_REG);
	}
	sprd_nand_disable_randomizer(host, page, mode);

	if (ret) {
		*ret_num = 0;
		sprd_nand_enable_wp(host);
		host->nfc_cfg0 &= ~(CFG0_SET_WPN);
		sprd_nand_delect_cs(host, -EIO);
		return ret;
	}
	*ret_num = num;
	sprd_nand_enable_wp(host);
	host->nfc_cfg0 &= ~(CFG0_SET_WPN);
	sprd_nand_delect_cs(host, 0);

	return 0;
}

static void sprd_nand_clk_disable(struct sprd_nand_host *host)
{
	if (!host->clocks_on)
		return;
	host->clocks_on = false;
	clk_disable_unprepare(host->clk_ecc);
	clk_disable_unprepare(host->clk_nand);
	clk_disable_unprepare(host->clk_nand_2x);
	clk_disable_unprepare(host->clk_nand_1x);
	clk_disable_unprepare(host->clk_ahb_26m);
	clk_disable_unprepare(host->clk_ahb_ecc);
	clk_disable_unprepare(host->clk_ahb_enable_gate);
}

static int sprd_nand_ctrl_en(struct sprd_nand_host *host, u32 en)
{
	int ret = 0;

	if (!en) {
		sprd_nand_clk_disable(host);
		return ret;
	}

	if (host->clocks_on)
		return 0;
	ret = clk_prepare_enable(host->clk_ahb_enable_gate);
	if (ret)
		return ret;
	ret = clk_prepare_enable(host->clk_ahb_ecc);
	if (ret)
		goto err_ahb_ecc_clk;
	ret = clk_prepare_enable(host->clk_ahb_26m);
	if (ret)
		goto err_ahb_26m_clk;
	ret = clk_prepare_enable(host->clk_nand_1x);
	if (ret)
		goto err_nand_1x_clk;
	ret = clk_prepare_enable(host->clk_nand_2x);
	if (ret)
		goto err_nand_2x_clk;
	ret = clk_set_parent(host->clk_nand, host->clk_parent_sdr);
	if (ret) {
		dev_err(host->dev, "NAND SDR parent selection failed: %d\n",
			ret);
		goto err_nand_clk;
	}
	ret = clk_set_parent(host->clk_ecc, host->clk_parent_ddr);
	if (ret) {
		dev_err(host->dev, "NAND ECC parent selection failed: %d\n",
			ret);
		goto err_nand_clk;
	}
	host->frequency = clk_get_rate(host->clk_nand);
	if (!host->frequency) {
		ret = -EINVAL;
		goto err_nand_clk;
	}
	ret = clk_prepare_enable(host->clk_nand);
	if (ret)
		goto err_nand_clk;
	ret = clk_prepare_enable(host->clk_ecc);
	if (ret)
		goto err_ecc_clk;

	host->clocks_on = true;
	dev_info(host->dev, "NAND controller clock: %u Hz\n", host->frequency);
	return 0;

err_ecc_clk:
	clk_disable_unprepare(host->clk_nand);
err_nand_clk:
	clk_disable_unprepare(host->clk_nand_2x);
err_nand_2x_clk:
	clk_disable_unprepare(host->clk_nand_1x);
err_nand_1x_clk:
	clk_disable_unprepare(host->clk_ahb_26m);
err_ahb_26m_clk:
	clk_disable_unprepare(host->clk_ahb_ecc);
err_ahb_ecc_clk:
	clk_disable_unprepare(host->clk_ahb_enable_gate);

	return ret;
}

static void sprd_nand_set_mode(struct sprd_nand_host *host, u8 infi_type,
			       u32 delay)
{
	host->nfc_cfg1 &= ~(CFG1_INTF_TYPE(7));
	host->nfc_cfg1 |= CFG1_INTF_TYPE(infi_type);
	sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
	sprd_nand_writel(host, delay, NFC_DLL0_CFG);
	sprd_nand_writel(host, delay, NFC_DLL1_CFG);
	sprd_nand_writel(host, delay, NFC_DLL2_CFG);
}

static int sprd_nand_parse_dt(struct platform_device *pdev,
			      struct sprd_nand_host *host)
{
	struct resource *res;
	struct device *dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->ioaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->ioaddr))
		return PTR_ERR(host->ioaddr);

	host->clk_nand = devm_clk_get(dev, "nandc_clk");
	if (IS_ERR(host->clk_nand))
		return PTR_ERR(host->clk_nand);
	host->clk_ecc = devm_clk_get(dev, "nandc_ecc_clk");
	if (IS_ERR(host->clk_ecc))
		return PTR_ERR(host->clk_ecc);
	host->clk_parent_sdr = devm_clk_get(dev, "nandc_parent_sdr");
	if (IS_ERR(host->clk_parent_sdr))
		return PTR_ERR(host->clk_parent_sdr);
	host->clk_parent_ddr = devm_clk_get(dev, "nandc_parent_ddr");
	if (IS_ERR(host->clk_parent_ddr))
		return PTR_ERR(host->clk_parent_ddr);
	host->clk_ahb_enable_gate = devm_clk_get(dev, "nandc_ahb_enable");
	if (IS_ERR(host->clk_ahb_enable_gate))
		return PTR_ERR(host->clk_ahb_enable_gate);
	host->clk_ahb_ecc = devm_clk_get(dev, "nandc_ecc_enable");
	if (IS_ERR(host->clk_ahb_ecc))
		return PTR_ERR(host->clk_ahb_ecc);
	host->clk_ahb_26m = devm_clk_get(dev, "nandc_26m_enable");
	if (IS_ERR(host->clk_ahb_26m))
		return PTR_ERR(host->clk_ahb_26m);
	host->clk_nand_1x = devm_clk_get(dev, "nandc_1x_enable");
	if (IS_ERR(host->clk_nand_1x))
		return PTR_ERR(host->clk_nand_1x);
	host->clk_nand_2x = devm_clk_get(dev, "nandc_2x_enable");
	if (IS_ERR(host->clk_nand_2x))
		return PTR_ERR(host->clk_nand_2x);

	return 0;
}

/* Check-only builds exactly the same bounded program without touching MMIO. */
static int sprd_orca_add_inst(struct nand_inst *program, u16 ins)
{
	if (program->cnt >= NFC_MAX_MC_INST_NUM - 1)
		return -ENOTSUPP;
	program->inst[program->cnt++] = ins;
	return 0;
}

static int sprd_orca_exec_op(struct nand_chip *chip,
			     const struct nand_operation *op, bool check_only)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);
	struct nand_inst prog = { .program_name = "raw_nand_exec_op",
				  .int_bits = INT_DONE | INT_TO | INT_WP };
	const struct nand_op_instr *data = NULL;
	bool status_in = false, writing = false, have_cmd = false;
	unsigned int i, j, last_cmd = 0, len = 0;
	u32 cfg1, cfg0, delay_cycles;
	int ret;

	if (op->cs != 0 || !op->ninstrs || !host->frequency)
		return -ENOTSUPP;
	for (i = 0; i < op->ninstrs; i++) {
		const struct nand_op_instr *in = &op->instrs[i];

		switch (in->type) {
		case NAND_OP_CMD_INSTR:
			last_cmd = in->ctx.cmd.opcode;
			/* Cache-read pipelines are not implemented by this page-ECC frontend. */
			if (last_cmd == NAND_CMD_READCACHESEQ ||
			    last_cmd == NAND_CMD_READCACHEEND)
				return -ENOTSUPP;
			have_cmd = true;
			writing |= last_cmd == NAND_CMD_ERASE1 ||
				   last_cmd == NAND_CMD_ERASE2 ||
				   last_cmd == NAND_CMD_PAGEPROG ||
				   last_cmd == NAND_CMD_SEQIN;
			ret = sprd_orca_add_inst(&prog, INST_CMD(last_cmd));
			if (ret)
				return ret;
			break;
		case NAND_OP_ADDR_INSTR:
			if (!in->ctx.addr.naddrs || in->ctx.addr.naddrs > 8)
				return -ENOTSUPP;
			for (j = 0; j < in->ctx.addr.naddrs; j++) {
				ret = sprd_orca_add_inst(
					&prog,
					INST_ADDR(in->ctx.addr.addrs[j], 0));
				if (ret)
					return ret;
			}
			break;
		case NAND_OP_DATA_IN_INSTR:
		case NAND_OP_DATA_OUT_INSTR:
			if (data || !have_cmd || !in->ctx.data.len)
				return -ENOTSUPP;
			data = in;
			len = in->ctx.data.len;
			status_in = in->type == NAND_OP_DATA_IN_INSTR &&
				    (last_cmd == NAND_CMD_READID ||
				     last_cmd == NAND_CMD_STATUS);
			if ((status_in && len > NFC_MAX_ID_LEN) ||
			    (!status_in && len > 2048))
				return -ENOTSUPP;
			if (status_in) {
				ret = sprd_orca_add_inst(&prog, INST_INOP(10));
				if (!ret)
					ret = sprd_orca_add_inst(
						&prog, INST_IDST(len));
			} else {
				writing |= in->type == NAND_OP_DATA_OUT_INSTR;
				ret = sprd_orca_add_inst(
					&prog,
					in->type == NAND_OP_DATA_OUT_INSTR ?
						INST_MWDT() :
						INST_MRDT());
			}
			if (ret)
				return ret;
			break;
		case NAND_OP_WAITRDY_INSTR:
			ret = sprd_orca_add_inst(&prog, INST_WRB0());
			if (ret)
				return ret;
			break;
		default:
			return -ENOTSUPP;
		}
		delay_cycles = DIV_ROUND_UP_ULL(
			(u64)in->delay_ns * host->frequency, 2000000000ULL);
		while (delay_cycles) {
			unsigned int cycles =
				min_t(unsigned int, delay_cycles, 256);
			ret = sprd_orca_add_inst(&prog, INST_INOP(cycles));
			if (ret)
				return ret;
			delay_cycles -= cycles;
		}
	}
	prog.inst[prog.cnt++] = INST_DONE();
	if (check_only)
		return 0;
	if (!host->clocks_on || (data && !status_in && !host->mbuf_v))
		return -EIO;

	/* Raw operation is never allowed to inherit ECC or randomizer configuration. */
	cfg0 = host->nfc_cfg0;
	cfg0 &= ~(CFG0_SET_SECT_NUM_MSK | CFG0_SET_NFC_RW | CFG0_SET_ECC_EN |
		  CFG0_SET_SPAR_USE | CFG0_SET_MAIN_USE |
		  CFG0_SET_SPARE_ONLY_INFO_PROD_EN |
		  CFG0_DEF1_MAIN_SPAR_APT(2) | 3);
	cfg0 &= ~GENMASK(23, 16);
	cfg0 = (cfg0 & CFG0_CS_MSKCLR) | CFG0_SET_CS_SEL(op->cs);
	cfg1 = host->nfc_cfg1;
	if (data && !status_in) {
		cfg0 |= CFG0_SET_MAIN_USE;
		cfg1 = (cfg1 & ~CFG1_temp_MIAN_SIZE_MSK) |
		       CFG1_DEF_MAIN_SIZE(len);
		if (data->type == NAND_OP_DATA_OUT_INSTR) {
			cfg0 |= CFG0_SET_NFC_RW;
			memcpy(host->mbuf_v, data->ctx.data.buf.out, len);
		}
		sprd_nand_writel(host, lower_32_bits(host->mbuf_p),
				 NFC_MAIN_ADDRL_REG);
		sprd_nand_writel(host, 0, NFC_MAIN_ADDRH_REG);
	} else {
		cfg0 |= CFG0_SET_NFC_MODE(2);
	}
	if (writing)
		cfg0 |= CFG0_SET_WPN;
	else
		cfg0 &= ~CFG0_SET_WPN;
	host->nfc_cfg0_val = cfg0;
	host->nfc_time0_val = status_in ? NFC_READID_TIMING : host->nfc_time0_r;
	host->nfc_sts_mach_val = DEF0_MATCH;
	sprd_nand_writel(host,
			 host->nfc_cfg3 & ~(CFG3_RANDOM_EN | CFG3_POLY_4R1_EN),
			 NFC_CFG3_REG);
	sprd_nand_writel(host, cfg1, NFC_CFG1_REG);
	sprd_nand_cmd_exec(host, &prog, 1, 0);
	ret = sprd_nand_cmd_wait(host, &prog);
	if (!ret && data && data->type == NAND_OP_DATA_IN_INSTR) {
		if (status_in) {
			u8 id[NFC_MAX_ID_LEN];

			put_unaligned_le32(
				sprd_nand_readl(host, NFC_STATUS0_REG), id);
			put_unaligned_le32(
				sprd_nand_readl(host, NFC_STATUS1_REG), id + 4);
			memcpy(data->ctx.data.buf.in, id, len);
		} else {
			memcpy(data->ctx.data.buf.in, host->mbuf_v, len);
		}
	}
	sprd_nand_writel(host, host->nfc_cfg1, NFC_CFG1_REG);
	sprd_nand_writel(host, host->nfc_cfg0 & ~CFG0_SET_WPN, NFC_CFG0_REG);
	sprd_nand_writel(host, host->nfc_cfg3, NFC_CFG3_REG);
	if (ret)
		sprd_nand_delect_cs(host, -EIO);
	return ret;
}

static int sprd_orca_read_page(struct nand_chip *chip, u8 *buf,
			       int oob_required, int page, bool raw)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 done;
	int ret;

	if (!host->clocks_on || chip->cur_cs != 0)
		return -EIO;
	memset(host->stsbuf_v, 0, host->stsbuf_size);
	ret = sprd_nand_read_page_in_block(
		host, page, 1, &done, 1, 1,
		raw ? MTD_OPS_RAW : MTD_OPS_PLACE_OOB, &mtd->ecc_stats);
	if (ret < 0)
		return ret;
	if (buf)
		memcpy(buf, host->mbuf_v, mtd->writesize);
	if (oob_required)
		memcpy(chip->oob_poi, host->sbuf_v, mtd->oobsize);
	return ret;
}

static int sprd_orca_read_ecc(struct nand_chip *chip, u8 *buf, int oob,
			      int page)
{
	return sprd_orca_read_page(chip, buf, oob, page, false);
}

static int sprd_orca_read_raw(struct nand_chip *chip, u8 *buf, int oob,
			      int page)
{
	return sprd_orca_read_page(chip, buf, oob, page, true);
}

static int sprd_orca_write_page(struct nand_chip *chip, const u8 *buf,
				int oob_required, int page, bool raw)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 done;

	if (!host->clocks_on || chip->cur_cs != 0)
		return -EIO;
	if (buf)
		memcpy(host->mbuf_v, buf, mtd->writesize);
	else
		memset(host->mbuf_v, 0xff, mtd->writesize);
	if (oob_required)
		memcpy(host->sbuf_v, chip->oob_poi, mtd->oobsize);
	else
		memset(host->sbuf_v, 0xff, mtd->oobsize);
	/* BSP boot compatibility uses two zero marker bytes, even on its x8 device. */
	if (!buf && oob_required && !host->sbuf_v[0])
		host->sbuf_v[1] = 0;
	return sprd_nand_write_page_in_blk(host, page, 1, &done, 1, 1,
					   raw ? MTD_OPS_RAW :
						 MTD_OPS_PLACE_OOB);
}

static int sprd_orca_write_ecc(struct nand_chip *chip, const u8 *buf, int oob,
			       int page)
{
	return sprd_orca_write_page(chip, buf, oob, page, false);
}

static int sprd_orca_write_raw(struct nand_chip *chip, const u8 *buf, int oob,
			       int page)
{
	return sprd_orca_write_page(chip, buf, oob, page, true);
}

static int sprd_orca_read_oob(struct nand_chip *chip, int page)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u32 done;
	int ret;

	if (!host->clocks_on || chip->cur_cs != 0)
		return -EIO;
	/*
	 * Match the BSP bad-block reader: one raw spare-only transfer, not a
	 * complete page through the combined main/spare sequencer.
	 */
	ret = sprd_nand_read_page_in_block(host, page, 1, &done, 0, 1,
					   MTD_OPS_RAW, &mtd->ecc_stats);
	if (ret < 0)
		return ret;
	memcpy(chip->oob_poi, host->sbuf_v, mtd->oobsize);
	return ret;
}

static int sprd_orca_write_oob(struct nand_chip *chip, int page)
{
	/* Programming FF to the main area cannot restore programmed zero bits. */
	return sprd_orca_write_raw(chip, NULL, 1, page);
}

static int sprd_orca_oob_ecc(struct mtd_info *mtd, int section,
			     struct mtd_oob_region *region)
{
	struct sprd_nand_host *host =
		nand_get_controller_data(mtd_to_nand(mtd));
	if (section < 0 || section >= host->param.nsect_per_page)
		return -ERANGE;
	region->offset =
		section * host->param.nspare_size + host->param.ecc_pos;
	region->length = host->param.ecc_size;
	return 0;
}

static int sprd_orca_oob_free(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *region)
{
	struct sprd_nand_host *host =
		nand_get_controller_data(mtd_to_nand(mtd));
	if (section < 0 || section >= host->param.nsect_per_page ||
	    !host->param.info_size)
		return -ERANGE;
	region->offset =
		section * host->param.nspare_size + host->param.info_pos;
	region->length = host->param.info_size;
	return 0;
}

static const struct mtd_ooblayout_ops sprd_orca_ooblayout = {
	.ecc = sprd_orca_oob_ecc,
	.free = sprd_orca_oob_free,
};

/*
 * Stock callback_isBadBlk reads the first page and classifies two bytes
 * by their combined population count. SPL ECC data may occupy these bytes;
 * exact-FF matching would incorrectly discard both intact SPL blocks.
 */
static u8 sprd_orca_good_marker[] = { 0xff, 0xff };
static struct nand_bbt_descr sprd_orca_badblock_pattern = {
	.offs = 0,
	.len = 2,
	.pattern = sprd_orca_good_marker,
	.min_good_bits = 8,
};

static int sprd_orca_attach_chip(struct nand_chip *chip)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);

	if (mtd->writesize != host->param.page_size ||
	    mtd->oobsize !=
		    host->param.nspare_size * host->param.nsect_per_page ||
	    mtd->erasesize !=
		    host->param.page_size * host->param.npage_per_blk ||
	    mtd->size != (u64)mtd->erasesize * host->param.nblkcnt)
		return -EINVAL;
	if ((chip->ecc.size && chip->ecc.size != host->param.nsect_size) ||
	    (chip->ecc.strength && chip->ecc.strength != host->param.ecc_mode))
		return -EINVAL;
	if (chip->bbt_options & NAND_BBT_USE_FLASH)
		return -EOPNOTSUPP;
	chip->badblock_pattern = &sprd_orca_badblock_pattern;
	chip->options &= ~(NAND_BBM_SECONDPAGE | NAND_BBM_LASTPAGE);
	chip->options |= NAND_BBM_FIRSTPAGE;
	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_ON_HOST;
	chip->ecc.placement = NAND_ECC_PLACEMENT_OOB;
	chip->ecc.algo = NAND_ECC_ALGO_BCH;
	chip->ecc.size = host->param.nsect_size;
	chip->ecc.strength = host->param.ecc_mode;
	chip->ecc.bytes = host->param.ecc_size;
	chip->ecc.read_page = sprd_orca_read_ecc;
	chip->ecc.write_page = sprd_orca_write_ecc;
	chip->ecc.read_page_raw = sprd_orca_read_raw;
	chip->ecc.write_page_raw = sprd_orca_write_raw;
	chip->ecc.read_oob = sprd_orca_read_oob;
	chip->ecc.read_oob_raw = sprd_orca_read_oob;
	chip->ecc.write_oob = sprd_orca_write_oob;
	chip->ecc.write_oob_raw = sprd_orca_write_oob;
	mtd_set_ooblayout(mtd, &sprd_orca_ooblayout);
	mtd->bitflip_threshold = host->risk_threshold;
	return 0;
}

static const struct nand_controller_ops sprd_orca_controller_ops = {
	.attach_chip = sprd_orca_attach_chip,
	.exec_op = sprd_orca_exec_op,
	/* NAND_KEEP_TIMINGS: only the recovered fixed SDR profile is programmed. */
};

static int sprd_orca_suspend(struct nand_chip *chip)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);

	sprd_nand_clk_disable(host);
	return 0;
}

static void sprd_orca_resume(struct nand_chip *chip)
{
	struct sprd_nand_host *host = nand_get_controller_data(chip);
	int ret = sprd_nand_ctrl_en(host, 1);

	if (ret) {
		dev_err(host->dev, "resume clocks failed: %d\n", ret);
		return;
	}
	sprd_nand_set_mode(host, 0, 0);
	sprd_nand_init_reg_state0(host);
	ret = sprd_nand_reset(host);
	sprd_nand_init_reg_state1(host);
	if (ret) {
		dev_err(host->dev, "resume NAND reset failed: %d\n", ret);
		sprd_nand_clk_disable(host);
	}
}

static int sprd_orca_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sprd_nand_host *host;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct device_node *child;
	struct nand_flash_dev ids[2] = {};
	u32 cs;
	int ret;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->dev = dev;
	chip = &host->chip;
	mtd = nand_to_mtd(chip);
	nand_controller_init(&host->controller);
	host->controller.ops = &sprd_orca_controller_ops;
	host->controller.controller_wp = true;
	chip->controller = &host->controller;
	nand_set_controller_data(chip, host);
	child = of_get_next_available_child(dev->of_node, NULL);
	if (child) {
		if (of_property_read_u32(child, "reg", &cs) || cs) {
			of_node_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "only NAND CS0 is supported\n");
		}
		host->flash_node = child;
	} else {
		/* Accept the original flat BSP node as well as standard nand@0 children. */
		host->flash_node = of_node_get(dev->of_node);
	}
	nand_set_flash_node(chip, host->flash_node);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto put_node;
	ret = sprd_nand_parse_dt(pdev, host);
	if (ret)
		goto put_node;
	ret = sprd_nand_ctrl_en(host, 1);
	if (ret)
		goto put_node;
	host->randomizer =
		of_property_read_bool(dev->of_node, "sprd,random-mode");
	sprd_nand_set_mode(host, 0, 0);
	sprd_nand_init_reg_state0(host);
	sprd_nand_set_timing_config(host, &default_timing, host->frequency);
	ret = sprd_nand_reset(host);
	if (ret)
		goto clocks_off;
	host->nfc_sts_mach_val = DEF0_MATCH;
	host->nfc_cfg0_val = host->nfc_cfg0 | CFG0_SET_NFC_MODE(2);
	host->nfc_time0_val = NFC_READID_TIMING;
	sprd_nand_cmd_exec(host, &inst_readid, 1, 0);
	ret = sprd_nand_cmd_wait(host, &inst_readid);
	if (ret)
		goto clocks_off;
	put_unaligned_le32(sprd_nand_readl(host, NFC_STATUS0_REG),
			   host->param.id);
	put_unaligned_le32(sprd_nand_readl(host, NFC_STATUS1_REG),
			   host->param.id + 4);
	ret = sprd_nand_param_init_nandhw(host);
	if (ret)
		goto clocks_off;
	ret = sprd_nand_alloc_dma(host);
	if (ret)
		goto clocks_off;
	sprd_nand_param_init_inst(host);
	sprd_nand_init_reg_state1(host);
	chip->options |= NAND_NO_SUBPAGE_WRITE | NAND_USES_DMA |
			 NAND_KEEP_TIMINGS;
	chip->ops.suspend = sprd_orca_suspend;
	chip->ops.resume = sprd_orca_resume;
	mtd->dev.parent = dev;
	mtd->owner = THIS_MODULE;
	mtd->name = "sprd-nand";
	/*
	 * Supply the complete matched record to NAND core; short IDs can have
	 * several geometries and OOB layouts in the firmware parameter table.
	 */
	ids[0].name = "Orca NAND";
	memcpy(ids[0].id, host->param.id,
	       sizeof(sprd_nand_vendor_param_table[0].id));
	ids[0].id_len = sizeof(sprd_nand_vendor_param_table[0].id);
	ids[0].pagesize = host->param.page_size;
	ids[0].erasesize = host->param.page_size * host->param.npage_per_blk;
	ids[0].chipsize = ((u64)ids[0].erasesize * host->param.nblkcnt) >> 20;
	ids[0].oobsize = host->param.nspare_size * host->param.nsect_per_page;
	ids[0].ecc.strength_ds = host->param.ecc_mode;
	ids[0].ecc.step_ds = host->param.nsect_size;
	if (host->param.nbus_width == BW_16) {
		ids[0].options = NAND_BUSWIDTH_16;
		chip->options |= NAND_BUSWIDTH_16;
	}
	ret = nand_scan_with_ids(chip, 1, ids);
	if (ret)
		goto clocks_off;
	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		nand_cleanup(chip);
		goto clocks_off;
	}
	platform_set_drvdata(pdev, host);
	dev_info(dev, "Orca NAND: ECC%u/%u, page %u, OOB %u\n",
		 chip->ecc.strength, chip->ecc.size, mtd->writesize,
		 mtd->oobsize);
	return 0;
clocks_off:
	sprd_nand_clk_disable(host);
put_node:
	of_node_put(host->flash_node);
	return ret;
}

static void sprd_orca_remove(struct platform_device *pdev)
{
	struct sprd_nand_host *host = platform_get_drvdata(pdev);
	int ret = mtd_device_unregister(nand_to_mtd(&host->chip));

	if (ret)
		dev_warn(host->dev, "MTD unregister returned %d\n", ret);
	nand_cleanup(&host->chip);
	sprd_nand_clk_disable(host);
	of_node_put(host->flash_node);
}

static const struct of_device_id sprd_orca_of_match[] = {
	{ .compatible = "sprd,orca-nandc" },
	{}
};
MODULE_DEVICE_TABLE(of, sprd_orca_of_match);

static struct platform_driver sprd_orca_driver = {
	.probe = sprd_orca_probe,
	.remove = sprd_orca_remove,
	.driver = {
		.name = "sprd-orca-nand",
		.of_match_table = sprd_orca_of_match,
		/* Static root-flash controller: no live UBI/MTD teardown contract yet. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sprd_orca_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Spreadtrum UDX710 Orca raw NAND controller");
