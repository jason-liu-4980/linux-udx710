// SPDX-License-Identifier: GPL-2.0
//
// Spreadtrum pll clock driver
//
// Copyright (C) 2015~2017 Spreadtrum, Inc.
// Author: Chunyan Zhang <chunyan.zhang@spreadtrum.com>

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "pll.h"

#define CLK_PLL_1M	1000000
#define CLK_PLL_10M	(CLK_PLL_1M * 10)

#define pindex(pll, member)		\
	(pll->factors[member].shift / (8 * sizeof(pll->regs_num)))

#define pshift(pll, member)		\
	(pll->factors[member].shift % (8 * sizeof(pll->regs_num)))

#define pwidth(pll, member)		\
	pll->factors[member].width

#define pmask(pll, member)					\
	((pwidth(pll, member)) ?				\
	GENMASK(pwidth(pll, member) + pshift(pll, member) - 1,	\
	pshift(pll, member)) : 0)

#define pinternal(pll, cfg, member)	\
	(cfg[pindex(pll, member)] & pmask(pll, member))

#define pinternal_val(pll, cfg, member)	\
	(pinternal(pll, cfg, member) >> pshift(pll, member))

static int sprd_pll_read(const struct sprd_pll *pll, u8 index, u32 *val)
{
	const struct sprd_clk_common *common = &pll->common;
	if (WARN_ON(index >= pll->regs_num))
		return -EINVAL;
	return regmap_read(common->regmap, common->reg + index * 4, val);
}

static int sprd_pll_write(const struct sprd_pll *pll, u8 index, u32 mask,
			  u32 val)
{
	const struct sprd_clk_common *common = &pll->common;
	u32 reg;
	int ret;
	if (WARN_ON(index >= pll->regs_num))
		return -EINVAL;
	ret = regmap_read(common->regmap, common->reg + index * 4, &reg);
	if (ret)
		return ret;
	return regmap_write(common->regmap, common->reg + index * 4,
			    (reg & ~mask) | val);
}

static int pll_get_refin(const struct sprd_pll *pll, u64 *refin)
{
	u32 shift, mask, index, reg, refin_id = 3;
	const unsigned long inputs[4] = { 2, 4, 13, 26 };
	int ret;
	if (pwidth(pll, PLL_REFIN)) {
		index = pindex(pll, PLL_REFIN);
		shift = pshift(pll, PLL_REFIN);
		mask = pmask(pll, PLL_REFIN);
		ret = sprd_pll_read(pll, index, &reg);
		if (ret)
			return ret;
		refin_id = (reg & mask) >> shift;
		if (refin_id > 3)
			refin_id = 3;
	}
	*refin = inputs[refin_id];
	return 0;
}

static u32 pll_get_ibias(u64 rate, const struct sprd_pll *pll)
{
	u32 i, num;

	if (pll->rate_mode == SPRD_PLL_RATE_INTEGER_FRACTIONAL) {
		for (i = 0; pll->ftable[i].ibias < SPRD_PLL_IBIAS_INVALID; i++)
			if (rate <= pll->ftable[i].max_freq)
				break;
		return pll->ftable[i].ibias;
	}
	/* Preserve the legacy implicit index-to-bias table contract. */
	num = pll->itable[0];
	for (i = 0; i < num; i++)
		if (rate <= pll->itable[i + 1])
			break;
	return i == num ? num - 1 : i;
}

static unsigned long _sprd_pll_recalc_rate(const struct sprd_pll *pll,
					   unsigned long parent_rate)
{
	u32 *cfg;
	u32 i, mask, regs_num = pll->regs_num;
	unsigned long rate, nint, kint = 0;
	u64 refin;
	u16 k1, k2;

	cfg = kcalloc(regs_num, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return parent_rate;

	for (i = 0; i < regs_num; i++) {
		if (sprd_pll_read(pll, i, &cfg[i])) {
			kfree(cfg);
			return parent_rate; /* error fallback, not a measured PLL rate */
		}
	}

	if (pll_get_refin(pll, &refin)) {
		kfree(cfg);
		return parent_rate;
	}

	if (pinternal(pll, cfg, PLL_PREDIV))
		refin = refin * 2;

	if (pwidth(pll, PLL_POSTDIV) &&
	    ((pll->fflag == 1 && pinternal(pll, cfg, PLL_POSTDIV)) ||
	     (!pll->fflag && !pinternal(pll, cfg, PLL_POSTDIV))))
		refin = refin / 2;

	if (!pinternal(pll, cfg, PLL_DIV_S)) {
		rate = refin * pinternal_val(pll, cfg, PLL_N) *
		       (pll->rate_mode == SPRD_PLL_RATE_INTEGER_FRACTIONAL ?
				CLK_PLL_1M :
				CLK_PLL_10M);
	} else {
		nint = pinternal_val(pll, cfg, PLL_NINT);
		if (pinternal(pll, cfg, PLL_SDM_EN))
			kint = pinternal_val(pll, cfg, PLL_KINT);

		mask = pmask(pll, PLL_KINT);

		k1 = pll->k1;
		k2 = pll->k2;
		rate = DIV_ROUND_CLOSEST_ULL(refin * kint * k1,
					 ((mask >> __ffs(mask)) + 1)) *
					 k2 + refin * nint * CLK_PLL_1M;
	}

	kfree(cfg);
	return rate;
}

static int _sprd_pll_set_rate(const struct sprd_pll *pll,
			      unsigned long rate,
			      unsigned long parent_rate)
{
	struct reg_cfg *cfg;
	int ret = 0;
	u32 mask, shift, width, ibias_val, index;
	u32 regs_num = pll->regs_num, i = 0;
	unsigned long kint = 0, nint;
	u64 tmp, refin, fvco = rate;

	if (!rate)
		return -EINVAL;
	cfg = kcalloc(regs_num, sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	ret = pll_get_refin(pll, &refin);
	if (ret)
		goto out;

	mask = pmask(pll, PLL_PREDIV);
	index = pindex(pll, PLL_PREDIV);
	width = pwidth(pll, PLL_PREDIV);
	if (width) {
		u32 reg;

		ret = sprd_pll_read(pll, index, &reg);
		if (ret)
			goto out;
		if (reg & mask)
			refin = refin * 2;
	}

	mask = pmask(pll, PLL_POSTDIV);
	index = pindex(pll, PLL_POSTDIV);
	width = pwidth(pll, PLL_POSTDIV);
	cfg[index].msk |= mask;
	if (width && ((pll->fflag == 1 && fvco <= pll->fvco) ||
		      (pll->fflag == 0 && fvco > pll->fvco)))
		cfg[index].val |= mask;

	if (width && fvco <= pll->fvco)
		fvco = fvco * 2;

	if (pll->rate_mode == SPRD_PLL_RATE_INTEGER_FRACTIONAL) {
		/* A sentinel or an integer-only PLL must never silently program garbage. */
		ibias_val = pll_get_ibias(fvco, pll);
		if (ibias_val == SPRD_PLL_IBIAS_INVALID ||
		    (!pwidth(pll, PLL_DIV_S) && fvco % (refin * CLK_PLL_1M))) {
			kfree(cfg);
			return -EINVAL;
		}

		mask = pmask(pll, PLL_IBIAS);
		index = pindex(pll, PLL_IBIAS);
		shift = pshift(pll, PLL_IBIAS);
		cfg[index].val |= ibias_val << shift & mask;
		cfg[index].msk |= mask;

		tmp = do_div(fvco, refin * CLK_PLL_1M);
		nint = fvco;
		if (tmp == 0) {
			mask = pmask(pll, PLL_N);
			index = pindex(pll, PLL_N);
			shift = pshift(pll, PLL_N);
			cfg[index].val |= (nint << shift) & mask;
			cfg[index].msk |= mask;

			mask = pmask(pll, PLL_DIV_S);
			index = pindex(pll, PLL_DIV_S);
			cfg[index].val &= ~mask;
			cfg[index].msk |= mask;

			mask = pmask(pll, PLL_SDM_EN);
			index = pindex(pll, PLL_SDM_EN);
			cfg[index].val &= ~mask;
			cfg[index].msk |= mask;
		} else {
			mask = pmask(pll, PLL_DIV_S);
			index = pindex(pll, PLL_DIV_S);
			cfg[index].val |= mask;
			cfg[index].msk |= mask;

			mask = pmask(pll, PLL_SDM_EN);
			index = pindex(pll, PLL_SDM_EN);
			cfg[index].val |= mask;
			cfg[index].msk |= mask;

			mask = pmask(pll, PLL_NINT);
			index = pindex(pll, PLL_NINT);
			shift = pshift(pll, PLL_NINT);
			cfg[index].val |= (nint << shift) & mask;
			cfg[index].msk |= mask;

			mask = pmask(pll, PLL_KINT);
			index = pindex(pll, PLL_KINT);
			width = pwidth(pll, PLL_KINT);
			shift = pshift(pll, PLL_KINT);
			do_div(tmp, 10000);
			tmp = tmp * ((mask >> shift) + 1);
			kint = DIV_ROUND_CLOSEST_ULL(tmp, refin * 100);
			cfg[index].val |= (kint << shift) & mask;
			cfg[index].msk |= mask;
		}

		/* Reject nonzero requests quantized to an all-zero BSP N/NINT/KINT code. */
		if (!nint && !kint) {
			ret = -EINVAL;
			goto out;
		}
	} else {
		mask = pmask(pll, PLL_DIV_S);
		index = pindex(pll, PLL_DIV_S);
		cfg[index].val |= mask;
		cfg[index].msk |= mask;

		mask = pmask(pll, PLL_SDM_EN);
		index = pindex(pll, PLL_SDM_EN);
		cfg[index].val |= mask;
		cfg[index].msk |= mask;

		nint = do_div(fvco, refin * CLK_PLL_1M);
		mask = pmask(pll, PLL_NINT);
		index = pindex(pll, PLL_NINT);
		shift = pshift(pll, PLL_NINT);
		cfg[index].val |= (nint << shift) & mask;
		cfg[index].msk |= mask;

		mask = pmask(pll, PLL_KINT);
		index = pindex(pll, PLL_KINT);
		width = pwidth(pll, PLL_KINT);
		shift = pshift(pll, PLL_KINT);
		tmp = fvco - refin * nint * CLK_PLL_1M;
		tmp = do_div(tmp, 10000) * ((mask >> shift) + 1);
		kint = DIV_ROUND_CLOSEST_ULL(tmp, refin * 100);
		cfg[index].val |= (kint << shift) & mask;
		cfg[index].msk |= mask;

		ibias_val = pll_get_ibias(fvco, pll);

		mask = pmask(pll, PLL_IBIAS);
		index = pindex(pll, PLL_IBIAS);
		shift = pshift(pll, PLL_IBIAS);
		cfg[index].val |= ibias_val << shift & mask;
		cfg[index].msk |= mask;
	}

	for (i = 0; i < regs_num; i++) {
		u32 actual;

		if (!cfg[i].msk)
			continue;
		ret = sprd_pll_write(pll, i, cfg[i].msk, cfg[i].val);
		if (ret)
			goto out;
		ret = sprd_pll_read(pll, i, &actual);
		if (ret)
			goto out;
		if ((actual & cfg[i].msk) != cfg[i].val) {
			ret = -EFAULT;
			goto out;
		}
	}

	if (!ret)
		udelay(pll->udelay);

out:
	kfree(cfg);
	return ret;
}

static unsigned long sprd_pll_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct sprd_pll *pll = hw_to_sprd_pll(hw);

	return _sprd_pll_recalc_rate(pll, parent_rate);
}

static int sprd_pll_set_rate(struct clk_hw *hw,
			     unsigned long rate,
			     unsigned long parent_rate)
{
	struct sprd_pll *pll = hw_to_sprd_pll(hw);

	return _sprd_pll_set_rate(pll, rate, parent_rate);
}

static int sprd_pll_clk_prepare(struct clk_hw *hw)
{
	struct sprd_pll *pll = hw_to_sprd_pll(hw);

	udelay(pll->udelay);

	return 0;
}

const struct clk_ops sprd_pll_ops = {
	.prepare = sprd_pll_clk_prepare,
	.recalc_rate = sprd_pll_recalc_rate,
	.determine_rate = clk_determine_rate_noop,
	.set_rate = sprd_pll_set_rate,
};
EXPORT_SYMBOL_GPL(sprd_pll_ops);
