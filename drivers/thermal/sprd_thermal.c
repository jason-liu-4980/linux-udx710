// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Spreadtrum Communications Inc.

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include "thermal_hwmon.h"

#define SPRD_THM_CTL			0x0
#define SPRD_THM_INT_EN			0x4
#define SPRD_THM_INT_STS		0x8
#define SPRD_THM_INT_RAW_STS		0xc
#define SPRD_THM_DET_PERIOD		0x10
#define SPRD_THM_INT_CLR		0x14
#define SPRD_THM_INT_CLR_ST		0x18
#define SPRD_THM_MON_PERIOD		0x4c
#define SPRD_THM_MON_CTL		0x50
#define SPRD_THM_INTERNAL_STS1		0x54
#define SPRD_THM_RAW_READ_MSK		0x3ff

#define SPRD_THM_OFFSET(id)		((id) * 0x4)
#define SPRD_THM_TEMP(id)		(SPRD_THM_OFFSET(id) + 0x5c)
#define SPRD_THM_THRES(id)		(SPRD_THM_OFFSET(id) + 0x2c)

#define SPRD_THM_SEN(id)		BIT((id) + 2)
#define SPRD_THM_SEN_OVERHEAT_EN(id)	BIT((id) + 8)
#define SPRD_THM_SEN_OVERHEAT_ALARM_EN(id)	BIT((id) + 0)

/* bits definitions for register THM_CTL */
#define SPRD_THM_SET_RDY_ST		BIT(13)
#define SPRD_THM_SET_RDY		BIT(12)
#define SPRD_THM_MON_EN			BIT(1)
#define SPRD_THM_EN			BIT(0)

/* bits definitions for register THM_INT_CTL */
#define SPRD_THM_BIT_INT_EN		BIT(26)
#define SPRD_THM_OVERHEAT_EN		BIT(25)
#define SPRD_THM_OTP_TRIP_SHIFT		10

/* bits definitions for register SPRD_THM_INTERNAL_STS1 */
#define SPRD_THM_TEMPER_RDY		BIT(0)

#define SPRD_THM_DET_PERIOD_DATA	0x800
#define SPRD_THM_DET_PERIOD_MASK	GENMASK(19, 0)
#define SPRD_THM_MON_MODE		0x7
#define SPRD_THM_MON_MODE_MASK		GENMASK(3, 0)
#define SPRD_THM_MON_PERIOD_DATA	0x10
#define SPRD_THM_MON_PERIOD_MASK	GENMASK(15, 0)
#define SPRD_THM_THRES_MASK		GENMASK(19, 0)
#define SPRD_THM_INT_CLR_MASK		GENMASK(24, 0)

/* thermal sensor calibration parameters */
#define SPRD_THM_TEMP_LOW		-40000
#define SPRD_THM_TEMP_HIGH		120000
#define SPRD_THM_OTP_TEMP		120000
#define SPRD_THM_HOT_TEMP		75000
#define SPRD_THM_RAW_DATA_LOW		0
#define SPRD_THM_RAW_DATA_HIGH		1000
#define SPRD_THM_SEN_NUM		8
#define SPRD_THM_DT_OFFSET		24
#define SPRD_THM_RATION_OFFSET		17
#define SPRD_THM_RATION_SIGN		16

#define SPRD_THM_RDYST_POLLING_TIME	10
#define SPRD_THM_RDYST_TIMEOUT		700
#define SPRD_THM_TEMP_READY_POLL_TIME	10000
#define SPRD_THM_TEMP_READY_TIMEOUT	600000
#define SPRD_THM_MAX_SENSOR		8

struct sprd_thermal_sensor {
	struct thermal_zone_device *tzd;
	struct sprd_thermal_data *data;
	struct device *dev;
	int cal_slope;
	int cal_offset;
	int id;
};

struct sprd_thermal_data {
	const struct sprd_thm_variant_data *var_data;
	struct sprd_thermal_sensor *sensor[SPRD_THM_MAX_SENSOR];
	struct clk *clk;
	struct clk *functional_clk;
	struct device *dev;
	struct mutex lock;
	bool ready;
	bool clock_enabled;
	u32 sensor_mask;
	void __iomem *base;
	u32 ratio_off;
	int ratio_sign;
};

/*
 * The conversion between ADC and temperature is based on linear relationship,
 * and use idea_k to specify the slope and ideal_b to specify the offset.
 *
 * Since different Spreadtrum SoCs have different ideal_k and ideal_b,
 * we should save ideal_k and ideal_b in the device data structure.
 */
struct sprd_thm_variant_data {
	u32 ideal_k;
	u32 ideal_b;
	u32 detect_period;
	u32 monitor_period;
	bool strict_calibration;
};

static const struct sprd_thm_variant_data ums512_data = {
	.ideal_k = 262,
	.ideal_b = 66400,
	/* Preserve the existing UMS512 programming until separately verified. */
	.detect_period = SPRD_THM_DET_PERIOD,
	.monitor_period = SPRD_THM_MON_PERIOD,
};

static const struct sprd_thm_variant_data orca_data = {
	.ideal_k = 262,
	.ideal_b = 66400,
	.detect_period = SPRD_THM_DET_PERIOD_DATA,
	.monitor_period = SPRD_THM_MON_PERIOD_DATA,
	.strict_calibration = true,
};

static inline void sprd_thm_update_bits(void __iomem *reg, u32 mask, u32 val)
{
	u32 tmp, orig;

	orig = readl(reg);
	tmp = orig & ~mask;
	tmp |= val & mask;
	writel(tmp, reg);
}

static int sprd_thm_cal_read(struct device_node *np, const char *cell_id,
			     u32 *val)
{
	struct nvmem_cell *cell;
	u8 *buf;
	size_t len, i;

	cell = of_nvmem_cell_get(np, cell_id);
	if (IS_ERR(cell))
		return PTR_ERR(cell);
	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return PTR_ERR(buf);
	if (!len || len > sizeof(*val)) {
		kfree(buf);
		return -EINVAL;
	}
	*val = 0;
	for (i = 0; i < len; i++)
		*val |= (u32)buf[i] << (i * 8);
	kfree(buf);
	return 0;
}

static int sprd_thm_sensor_calibration(struct device_node *np,
				       struct sprd_thermal_data *thm,
				       struct sprd_thermal_sensor *sen)
{
	int ret;
	/*
	 * According to thermal datasheet, the default calibration offset is 64,
	 * and the default ratio is 1000.
	 */
	int dt_offset = 64, ratio = 1000;

	ret = sprd_thm_cal_read(np, "sen_delta_cal", &dt_offset);
	if (ret)
		return ret;

	if (thm->var_data->strict_calibration) {
		if (dt_offset < 0 || dt_offset > 255 || thm->ratio_off > 127)
			return -ERANGE;

	}
	ratio += thm->ratio_sign * thm->ratio_off;

	/*
	 * According to the ideal slope K and ideal offset B, combined with
	 * calibration value of thermal from efuse, then calibrate the real
	 * slope k and offset b:
	 * k_cal = (k * ratio) / 1000.
	 * b_cal = b + (dt_offset - 64) * 500.
	 */
	sen->cal_slope = (thm->var_data->ideal_k * ratio) / 1000;
	sen->cal_offset = thm->var_data->ideal_b + (dt_offset - 128) * 250;
	if (sen->cal_slope <= 0)
		return -ERANGE;

	return 0;
}

static int sprd_thm_rawdata_to_temp(struct sprd_thermal_sensor *sen,
				    u32 rawdata)
{
	rawdata = clamp(rawdata, SPRD_THM_RAW_DATA_LOW, SPRD_THM_RAW_DATA_HIGH);

	/*
	 * According to thermal datasheet, the formula of converting
	 * adc value to the temperature value should be:
	 * T_final = k_cal * x - b_cal.
	 */
	return sen->cal_slope * rawdata - sen->cal_offset;
}

static int sprd_thm_temp_to_rawdata(int temp, struct sprd_thermal_sensor *sen)
{
	int val;

	temp = clamp(temp, SPRD_THM_TEMP_LOW, SPRD_THM_TEMP_HIGH);

	/*
	 * According to thermal datasheet, the formula of converting
	 * adc value to the temperature value should be:
	 * T_final = k_cal * x - b_cal.
	 */
	val = (temp + sen->cal_offset) / sen->cal_slope;

	return clamp(val, 0, SPRD_THM_RAW_DATA_HIGH - 1);
}

static int sprd_thm_read_temp(struct thermal_zone_device *tz, int *temp)
{
	struct sprd_thermal_sensor *sen = thermal_zone_device_priv(tz);
	struct sprd_thermal_data *thm = sen->data;
	u32 data;
	int ret = 0;

	mutex_lock(&thm->lock);
	if (!thm->ready) {
		ret = -EAGAIN;
		goto out;
	}
	data = readl(thm->base + SPRD_THM_TEMP(sen->id)) &
	       SPRD_THM_RAW_READ_MSK;
	if (thm->var_data->strict_calibration &&
	    data > SPRD_THM_RAW_DATA_HIGH) {
		ret = -ERANGE;
		goto out;
	}
	*temp = sprd_thm_rawdata_to_temp(sen, data);
out:
	mutex_unlock(&thm->lock);
	return ret;
}

static const struct thermal_zone_device_ops sprd_thm_ops = {
	.get_temp = sprd_thm_read_temp,
};

static int sprd_thm_poll_ready_status(struct sprd_thermal_data *thm)
{
	u32 val;
	int ret;

	/*
	 * Wait for thermal ready status before configuring thermal parameters.
	 */
	ret = readl_poll_timeout(thm->base + SPRD_THM_CTL, val,
				 !(val & SPRD_THM_SET_RDY_ST),
				 SPRD_THM_RDYST_POLLING_TIME,
				 SPRD_THM_RDYST_TIMEOUT);
	if (ret)
		return ret;

	sprd_thm_update_bits(thm->base + SPRD_THM_CTL, SPRD_THM_MON_EN,
			     SPRD_THM_MON_EN);
	sprd_thm_update_bits(thm->base + SPRD_THM_CTL, SPRD_THM_SET_RDY,
			     SPRD_THM_SET_RDY);
	return 0;
}

static int sprd_thm_wait_temp_ready(struct sprd_thermal_data *thm)
{
	u32 val;

	/* Wait for first temperature data ready before reading temperature */
	return readl_poll_timeout(thm->base + SPRD_THM_INTERNAL_STS1, val,
				  !(val & SPRD_THM_TEMPER_RDY),
				  SPRD_THM_TEMP_READY_POLL_TIME,
				  SPRD_THM_TEMP_READY_TIMEOUT);
}

static int sprd_thm_set_ready(struct sprd_thermal_data *thm)
{
	int ret;

	ret = sprd_thm_poll_ready_status(thm);
	if (ret)
		return ret;

	/*
	 * Clear interrupt status, enable thermal interrupt and enable thermal.
	 *
	 * The SPRD thermal controller integrates a hardware interrupt signal,
	 * which means if the temperature is overheat, it will generate an
	 * interrupt and notify the event to PMIC automatically to shutdown the
	 * system. So here we should enable the interrupt bits, though we have
	 * not registered an irq handler.
	 */
	writel(SPRD_THM_INT_CLR_MASK, thm->base + SPRD_THM_INT_CLR);
	sprd_thm_update_bits(thm->base + SPRD_THM_INT_EN,
			     SPRD_THM_BIT_INT_EN, SPRD_THM_BIT_INT_EN);
	sprd_thm_update_bits(thm->base + SPRD_THM_CTL,
			     SPRD_THM_EN, SPRD_THM_EN);
	return 0;
}

static void sprd_thm_sensor_init(struct sprd_thermal_data *thm,
				 struct sprd_thermal_sensor *sen)
{
	u32 otp_rawdata, hot_rawdata;

	otp_rawdata = sprd_thm_temp_to_rawdata(SPRD_THM_OTP_TEMP, sen);
	hot_rawdata = sprd_thm_temp_to_rawdata(SPRD_THM_HOT_TEMP, sen);

	/* Enable the sensor' overheat temperature protection interrupt */
	sprd_thm_update_bits(thm->base + SPRD_THM_INT_EN,
			     SPRD_THM_SEN_OVERHEAT_ALARM_EN(sen->id),
			     SPRD_THM_SEN_OVERHEAT_ALARM_EN(sen->id));

	/* Set the sensor' overheat and hot threshold temperature */
	sprd_thm_update_bits(thm->base + SPRD_THM_THRES(sen->id),
			     SPRD_THM_THRES_MASK,
			     (otp_rawdata << SPRD_THM_OTP_TRIP_SHIFT) |
			     hot_rawdata);

	/* Enable the corresponding sensor */
	sprd_thm_update_bits(thm->base + SPRD_THM_CTL, SPRD_THM_SEN(sen->id),
			     SPRD_THM_SEN(sen->id));
}

static void sprd_thm_para_config(struct sprd_thermal_data *thm)
{
	/* Set the period of two valid temperature detection action */
	sprd_thm_update_bits(thm->base + SPRD_THM_DET_PERIOD,
			     SPRD_THM_DET_PERIOD_MASK,
			     thm->var_data->detect_period);

	/* Set the sensors' monitor mode */
	sprd_thm_update_bits(thm->base + SPRD_THM_MON_CTL,
			     SPRD_THM_MON_MODE_MASK, SPRD_THM_MON_MODE);

	/* Set the sensors' monitor period */
	sprd_thm_update_bits(thm->base + SPRD_THM_MON_PERIOD,
			     SPRD_THM_MON_PERIOD_MASK,
			     thm->var_data->monitor_period);
}

static void sprd_thm_toggle_sensor(struct sprd_thermal_sensor *sen, bool on)
{
	struct thermal_zone_device *tzd = sen->tzd;

	if (on)
		thermal_zone_device_enable(tzd);
	else
		thermal_zone_device_disable(tzd);
}

static int sprd_thm_clocks_enable(struct sprd_thermal_data *thm)
{
	int ret;

	ret = clk_prepare_enable(thm->functional_clk);
	if (ret)
		return ret;
	if (thm->functional_clk && !clk_get_rate(thm->functional_clk)) {
		ret = -EINVAL;
		goto disable_function;
	}
	ret = clk_prepare_enable(thm->clk);
	if (!ret)
		return 0;
disable_function:
	clk_disable_unprepare(thm->functional_clk);
	return ret;
}

static void sprd_thm_clocks_disable(struct sprd_thermal_data *thm)
{
	clk_disable_unprepare(thm->clk);
	clk_disable_unprepare(thm->functional_clk);
}

static int sprd_thm_hw_start(struct sprd_thermal_data *thm)
{
	u32 val;
	int ret, i;

	ret = readl_poll_timeout(thm->base + SPRD_THM_CTL, val,
				 !(val & SPRD_THM_SET_RDY_ST),
				 SPRD_THM_RDYST_POLLING_TIME,
				 SPRD_THM_RDYST_TIMEOUT);
	if (ret)
		return ret;
	sprd_thm_para_config(thm);
	/* Only the explicitly configured sensors may remain enabled. */
	sprd_thm_update_bits(thm->base + SPRD_THM_CTL, GENMASK(9, 2), 0);
	sprd_thm_update_bits(thm->base + SPRD_THM_INT_EN, GENMASK(7, 0), 0);
	for (i = 0; i < SPRD_THM_MAX_SENSOR; i++)
		if (thm->sensor[i])
			sprd_thm_sensor_init(thm, thm->sensor[i]);
	ret = sprd_thm_set_ready(thm);
	return ret ? ret : sprd_thm_wait_temp_ready(thm);
}

static int sprd_thm_hw_stop(struct sprd_thermal_data *thm)
{
	int ret;

	sprd_thm_update_bits(thm->base + SPRD_THM_CTL, GENMASK(9, 2), 0);
	/* Latch sensor disable before clearing the controller enable. */
	ret = sprd_thm_poll_ready_status(thm);
	sprd_thm_update_bits(thm->base + SPRD_THM_CTL, SPRD_THM_EN, 0);
	return ret;
}

static void sprd_thm_disable(void *data)
{
	struct sprd_thermal_data *thm = data;

	mutex_lock(&thm->lock);
	thm->ready = false;
	if (thm->clock_enabled) {
		if (sprd_thm_hw_stop(thm))
			dev_warn(thm->dev,
				 "failed to stop thermal controller\n");
		sprd_thm_clocks_disable(thm);
		thm->clock_enabled = false;
	}
	mutex_unlock(&thm->lock);
}

static int sprd_thm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *sen_child;
	struct sprd_thermal_data *thm;
	struct sprd_thermal_sensor *sen;
	int ret, i;
	u32 val, id;

	thm = devm_kzalloc(dev, sizeof(*thm), GFP_KERNEL);
	if (!thm)
		return -ENOMEM;
	thm->var_data = of_device_get_match_data(dev);
	if (!thm->var_data)
		return -EINVAL;
	thm->dev = dev;
	mutex_init(&thm->lock);
	thm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(thm->base))
		return PTR_ERR(thm->base);
	thm->clk = devm_clk_get(dev, "enable");
	if (IS_ERR(thm->clk))
		return dev_err_probe(dev, PTR_ERR(thm->clk), "enable clock\n");
	if (of_property_match_string(np, "clock-names", "functional") >= 0) {
		thm->functional_clk = devm_clk_get(dev, "functional");
		if (IS_ERR(thm->functional_clk))
			return dev_err_probe(dev, PTR_ERR(thm->functional_clk),
					     "functional clock\n");
	}
	/* Calibrate every sensor before enabling clocks or writing registers. */
	ret = sprd_thm_cal_read(np, "thm_sign_cal", &val);
	if (ret)
		return dev_err_probe(dev, ret, "sign calibration\n");
	if (thm->var_data->strict_calibration && val > 1)
		return -ERANGE;
	thm->ratio_sign = val ? -1 : 1;
	ret = sprd_thm_cal_read(np, "thm_ratio_cal", &thm->ratio_off);
	if (ret)
		return dev_err_probe(dev, ret, "ratio calibration\n");
	for_each_available_child_of_node(np, sen_child) {
		ret = of_property_read_u32(sen_child, "reg", &id);
		if (ret)
			goto of_put;
		if (id >= SPRD_THM_MAX_SENSOR || thm->sensor[id]) {
			ret = -EINVAL;
			goto of_put;
		}
		sen = devm_kzalloc(dev, sizeof(*sen), GFP_KERNEL);
		if (!sen) {
			ret = -ENOMEM;
			goto of_put;
		}
		sen->id = id;
		sen->data = thm;
		sen->dev = dev;
		ret = sprd_thm_sensor_calibration(sen_child, thm, sen);
		if (ret)
			goto of_put;
		thm->sensor[id] = sen;
		thm->sensor_mask |= BIT(id);
	}
	if (!thm->sensor_mask)
		return -EINVAL;
	ret = sprd_thm_clocks_enable(thm);
	if (ret)
		return ret;
	thm->clock_enabled = true;
	ret = devm_add_action_or_reset(dev, sprd_thm_disable, thm);
	if (ret)
		return ret;
	ret = sprd_thm_hw_start(thm);
	if (ret)
		return dev_err_probe(dev, ret, "controller not ready\n");
	thm->ready = true;
	for (i = 0; i < SPRD_THM_MAX_SENSOR; i++) {
		sen = thm->sensor[i];
		if (!sen)
			continue;
		sen->tzd = devm_thermal_of_zone_register(dev, i, sen,
							 &sprd_thm_ops);
		if (IS_ERR(sen->tzd))
			return dev_err_probe(dev, PTR_ERR(sen->tzd),
					     "thermal zone %d\n", i);
		ret = devm_thermal_add_hwmon_sysfs(dev, sen->tzd);
		if (ret)
			return dev_err_probe(dev, ret, "hwmon sensor %d\n", i);
	}
	platform_set_drvdata(pdev, thm);
	return 0;
of_put:
	of_node_put(sen_child);
	return dev_err_probe(dev, ret, "invalid sensor or calibration\n");
}

static int __maybe_unused sprd_thm_suspend(struct device *dev)
{
	struct sprd_thermal_data *thm = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&thm->lock);
	thm->ready = false;
	ret = thm->clock_enabled ? sprd_thm_hw_stop(thm) : 0;
	if (ret) {
		if (!sprd_thm_hw_start(thm))
			thm->ready = true;
		else
			dev_warn(dev, "failed to restore thermal controller\n");
	} else if (thm->clock_enabled) {
		sprd_thm_clocks_disable(thm);
		thm->clock_enabled = false;
	}
	mutex_unlock(&thm->lock);
	return ret;
}

static int __maybe_unused sprd_thm_resume(struct device *dev)
{
	struct sprd_thermal_data *thm = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&thm->lock);
	thm->ready = false;
	if (!thm->clock_enabled) {
		ret = sprd_thm_clocks_enable(thm);
		if (ret)
			goto out;
		thm->clock_enabled = true;
	}
	ret = sprd_thm_hw_start(thm);
	if (ret) {
		sprd_thm_hw_stop(thm);
		sprd_thm_clocks_disable(thm);
		thm->clock_enabled = false;
	} else {
		thm->ready = true;
	}
out:
	mutex_unlock(&thm->lock);
	return ret;
}

static void sprd_thm_remove(struct platform_device *pdev)
{
	struct sprd_thermal_data *thm = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < SPRD_THM_MAX_SENSOR; i++) {
		if (!thm->sensor[i])
			continue;
		sprd_thm_toggle_sensor(thm->sensor[i], false);
		devm_thermal_of_zone_unregister(&pdev->dev,
						thm->sensor[i]->tzd);
	}
}

static const struct of_device_id sprd_thermal_of_match[] = {
	{ .compatible = "sprd,ums512-thermal", .data = &ums512_data },
	{ .compatible = "sprd,orca-thermal", .data = &orca_data },
	{},
};
MODULE_DEVICE_TABLE(of, sprd_thermal_of_match);

static const struct dev_pm_ops sprd_thermal_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sprd_thm_suspend, sprd_thm_resume)
};

static struct platform_driver sprd_thermal_driver = {
	.probe = sprd_thm_probe,
	.remove = sprd_thm_remove,
	.driver = {
		.name = "sprd-thermal",
		.pm = &sprd_thermal_pm_ops,
		.of_match_table = sprd_thermal_of_match,
	},
};

module_platform_driver(sprd_thermal_driver);

MODULE_AUTHOR("Freeman Liu <freeman.liu@unisoc.com>");
MODULE_DESCRIPTION("Spreadtrum thermal driver");
MODULE_LICENSE("GPL");
