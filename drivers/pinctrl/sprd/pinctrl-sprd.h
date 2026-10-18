/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Driver header file for pin controller driver
 * Copyright (C) 2017 Spreadtrum  - http://www.spreadtrum.com
 */

#ifndef __PINCTRL_SPRD_H__
#define __PINCTRL_SPRD_H__

struct platform_device;

#define NUM_OFFSET	(20)
#define TYPE_OFFSET	(16)
#define BIT_OFFSET	(8)
#define WIDTH_OFFSET	(4)

#define SPRD_PIN_INFO(num, type, offset, width, reg)	\
		(((num) & 0xFFF) << NUM_OFFSET |	\
		 ((type) & 0xF) << TYPE_OFFSET |	\
		 ((offset) & 0xFF) << BIT_OFFSET |	\
		 ((width) & 0xF) << WIDTH_OFFSET |	\
		 ((reg) & 0xF))

#define SPRD_PINCTRL_PIN(pin)	SPRD_PINCTRL_PIN_DATA(pin, #pin)

#define SPRD_PINCTRL_PIN_DATA(a, b)				\
	{							\
		.name = b,					\
		.num = (((a) >> NUM_OFFSET) & 0xfff),		\
		.type = (((a) >> TYPE_OFFSET) & 0xf),		\
		.bit_offset = (((a) >> BIT_OFFSET) & 0xff),	\
		.bit_width = ((a) >> WIDTH_OFFSET & 0xf),	\
		.reg = ((a) & 0xf)				\
	}

enum pin_type {
	GLOBAL_CTRL_PIN,
	COMMON_PIN,
	MISC_PIN,
};

struct sprd_pins_info {
	const char *name;
	unsigned int num;
	enum pin_type type;

	/* for global control pins configuration */
	unsigned long bit_offset;
	unsigned long bit_width;
	unsigned int reg;
};

/**
 * struct sprd_pinctrl_soc_data - SoC-specific register layout and encoding
 * @common_pin_offset: first common pin register, relative to controller base
 * @misc_pin_offset: first miscellaneous pin register
 * @pull_up_20k: register value selecting the 20 kiloohm pull-up
 * @pull_up_4_7k: register value selecting the 4.7 kiloohm pull-up
 * @sleep_mode_mask: supported sleep-domain bits, before SLEEP_MODE_SHIFT
 * @use_dt_path_names: key groups by full DT path instead of node basename
 * @legacy_pinconf: keep SC9860 raw readback and legacy config behavior
 *
 * Pin-table ordering remains global, common, then miscellaneous pins.
 */
struct sprd_pinctrl_soc_data {
	unsigned int common_pin_offset;
	unsigned int misc_pin_offset;
	unsigned int pull_up_20k;
	unsigned int pull_up_4_7k;
	unsigned int sleep_mode_mask;
	unsigned int legacy_pinconf;
	unsigned int use_dt_path_names;
};

int sprd_pinctrl_core_probe_with_data(
	struct platform_device *pdev,
	const struct sprd_pins_info *sprd_soc_pin_info, int pins_cnt,
	const struct sprd_pinctrl_soc_data *soc_data);
int sprd_pinctrl_core_probe(struct platform_device *pdev,
			    struct sprd_pins_info *sprd_soc_pin_info,
			    int pins_cnt);
void sprd_pinctrl_remove(struct platform_device *pdev);
void sprd_pinctrl_shutdown(struct platform_device *pdev);

#endif /* __PINCTRL_SPRD_H__ */
