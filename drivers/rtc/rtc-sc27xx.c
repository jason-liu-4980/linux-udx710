// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/udx710-trial.h>
#include <linux/pid.h>
#include <linux/sched/signal.h>
#define UDX_LEASE_RATE 32768u
#define UDX_LEASE_TICKS (60u * UDX_LEASE_RATE)
struct udx_lease_state {
	uint64_t last_renew_ms;
	unsigned int renewals, closed, ready_stage;
};

#define SPRD_RTC_SEC_CNT_VALUE 0x0
#define SPRD_RTC_MIN_CNT_VALUE 0x4
#define SPRD_RTC_HOUR_CNT_VALUE 0x8
#define SPRD_RTC_DAY_CNT_VALUE 0xc
#define SPRD_RTC_SEC_CNT_UPD 0x10
#define SPRD_RTC_MIN_CNT_UPD 0x14
#define SPRD_RTC_HOUR_CNT_UPD 0x18
#define SPRD_RTC_DAY_CNT_UPD 0x1c
#define SPRD_RTC_SEC_ALM_UPD 0x20
#define SPRD_RTC_MIN_ALM_UPD 0x24
#define SPRD_RTC_HOUR_ALM_UPD 0x28
#define SPRD_RTC_DAY_ALM_UPD 0x2c
#define SPRD_RTC_INT_EN 0x30
#define SPRD_RTC_INT_RAW_STS 0x34
#define SPRD_RTC_INT_CLR 0x38
#define SPRD_RTC_INT_MASK_STS 0x3C
#define SPRD_RTC_SEC_ALM_VALUE 0x40
#define SPRD_RTC_MIN_ALM_VALUE 0x44
#define SPRD_RTC_HOUR_ALM_VALUE 0x48
#define SPRD_RTC_DAY_ALM_VALUE 0x4c
#define SPRD_RTC_SPG_VALUE 0x50
#define SPRD_RTC_SPG_UPD 0x54
#define SPRD_RTC_PWR_CTRL 0x58
#define SPRD_RTC_PWR_STS 0x5c
#define SPRD_RTC_SEC_AUXALM_UPD 0x60
#define SPRD_RTC_MIN_AUXALM_UPD 0x64
#define SPRD_RTC_HOUR_AUXALM_UPD 0x68
#define SPRD_RTC_DAY_AUXALM_UPD 0x6c

/* BIT & MASK definition for SPRD_RTC_INT_* registers */
#define SPRD_RTC_SEC_EN BIT(0)
#define SPRD_RTC_MIN_EN BIT(1)
#define SPRD_RTC_HOUR_EN BIT(2)
#define SPRD_RTC_DAY_EN BIT(3)
#define SPRD_RTC_ALARM_EN BIT(4)
#define SPRD_RTC_HRS_FORMAT_EN BIT(5)
#define SPRD_RTC_AUXALM_EN BIT(6)
#define SPRD_RTC_SPG_UPD_EN BIT(7)
#define SPRD_RTC_SEC_UPD_EN BIT(8)
#define SPRD_RTC_MIN_UPD_EN BIT(9)
#define SPRD_RTC_HOUR_UPD_EN BIT(10)
#define SPRD_RTC_DAY_UPD_EN BIT(11)
#define SPRD_RTC_ALMSEC_UPD_EN BIT(12)
#define SPRD_RTC_ALMMIN_UPD_EN BIT(13)
#define SPRD_RTC_ALMHOUR_UPD_EN BIT(14)
#define SPRD_RTC_ALMDAY_UPD_EN BIT(15)
#define SPRD_RTC_INT_MASK GENMASK(15, 0)

#define SPRD_RTC_TIME_INT_MASK                                              \
	(SPRD_RTC_SEC_UPD_EN | SPRD_RTC_MIN_UPD_EN | SPRD_RTC_HOUR_UPD_EN | \
	 SPRD_RTC_DAY_UPD_EN)

#define SPRD_RTC_ALMTIME_INT_MASK                          \
	(SPRD_RTC_ALMSEC_UPD_EN | SPRD_RTC_ALMMIN_UPD_EN | \
	 SPRD_RTC_ALMHOUR_UPD_EN | SPRD_RTC_ALMDAY_UPD_EN)

#define SPRD_RTC_ALM_INT_MASK                                   \
	(SPRD_RTC_SEC_EN | SPRD_RTC_MIN_EN | SPRD_RTC_HOUR_EN | \
	 SPRD_RTC_DAY_EN | SPRD_RTC_ALARM_EN | SPRD_RTC_AUXALM_EN)

/* second/minute/hour/day values mask definition */
#define SPRD_RTC_SEC_MASK GENMASK(5, 0)
#define SPRD_RTC_MIN_MASK GENMASK(5, 0)
#define SPRD_RTC_HOUR_MASK GENMASK(4, 0)
#define SPRD_RTC_DAY_MASK GENMASK(15, 0)

/* alarm lock definition for SPRD_RTC_SPG_UPD register */
#define SPRD_RTC_ALMLOCK_MASK GENMASK(7, 0)
#define SPRD_RTC_ALM_UNLOCK 0xa5
#define SPRD_RTC_ALM_LOCK (~SPRD_RTC_ALM_UNLOCK & SPRD_RTC_ALMLOCK_MASK)

/* SPG values definition for SPRD_RTC_SPG_UPD register */
#define SPRD_RTC_POWEROFF_ALM_FLAG BIT(8)

/* power control/status definition */
#define SPRD_RTC_POWER_RESET_VALUE 0x96
#define SPRD_RTC_POWER_STS_CLEAR GENMASK(7, 0)
#define SPRD_RTC_POWER_STS_SHIFT 8
#define SPRD_RTC_POWER_STS_VALID \
	(~SPRD_RTC_POWER_RESET_VALUE << SPRD_RTC_POWER_STS_SHIFT)

/* timeout of synchronizing time and alarm registers (us) */
#define SPRD_RTC_POLL_TIMEOUT 200000
#define SPRD_RTC_POLL_DELAY_US 20000

struct sprd_rtc {
	struct rtc_device *rtc;
	struct regmap *regmap;
	struct device *dev;
	u32 base;
	int irq;
	bool valid;
	struct udx_lease_state trial_lease;
	struct pid *trial_local_owner;
	bool trial_owner_cleanup;
};

/*
 * The Spreadtrum RTC controller has 3 groups registers, including time, normal
 * alarm and auxiliary alarm. The time group registers are used to set RTC time,
 * the normal alarm registers are used to set normal alarm, and the auxiliary
 * alarm registers are used to set auxiliary alarm. Both alarm event and
 * auxiliary alarm event can wake up system from deep sleep, but only alarm
 * event can power up system from power down status.
 */
enum sprd_rtc_reg_types {
	SPRD_RTC_TIME,
	SPRD_RTC_ALARM,
	SPRD_RTC_AUX_ALARM,
};

static int sprd_rtc_clear_alarm_ints(struct sprd_rtc *rtc)
{
	return regmap_write(rtc->regmap, rtc->base + SPRD_RTC_INT_CLR,
			    SPRD_RTC_ALM_INT_MASK);
}

static int sprd_rtc_lock_alarm(struct sprd_rtc *rtc, bool lock)
{
	int ret;
	u32 val;

	ret = regmap_read(rtc->regmap, rtc->base + SPRD_RTC_SPG_VALUE, &val);
	if (ret)
		return ret;

	val &= ~SPRD_RTC_ALMLOCK_MASK;
	if (lock)
		val |= SPRD_RTC_ALM_LOCK;
	else
		val |= SPRD_RTC_ALM_UNLOCK | SPRD_RTC_POWEROFF_ALM_FLAG;

	ret = regmap_write(rtc->regmap, rtc->base + SPRD_RTC_SPG_UPD, val);
	if (ret)
		return ret;

	/* wait until the SPG value is updated successfully */
	ret = regmap_read_poll_timeout(rtc->regmap,
				       rtc->base + SPRD_RTC_INT_RAW_STS, val,
				       (val & SPRD_RTC_SPG_UPD_EN),
				       SPRD_RTC_POLL_DELAY_US,
				       SPRD_RTC_POLL_TIMEOUT);
	if (ret) {
		dev_err(rtc->dev, "failed to update SPG value:%d\n", ret);
		return ret;
	}

	return regmap_write(rtc->regmap, rtc->base + SPRD_RTC_INT_CLR,
			    SPRD_RTC_SPG_UPD_EN);
}

static int sprd_rtc_get_secs(struct sprd_rtc *rtc, enum sprd_rtc_reg_types type,
			     time64_t *secs)
{
	u32 sec_reg, min_reg, hour_reg, day_reg;
	u32 val, sec, min, hour, day;
	int ret;

	switch (type) {
	case SPRD_RTC_TIME:
		sec_reg = SPRD_RTC_SEC_CNT_VALUE;
		min_reg = SPRD_RTC_MIN_CNT_VALUE;
		hour_reg = SPRD_RTC_HOUR_CNT_VALUE;
		day_reg = SPRD_RTC_DAY_CNT_VALUE;
		break;
	case SPRD_RTC_ALARM:
		sec_reg = SPRD_RTC_SEC_ALM_VALUE;
		min_reg = SPRD_RTC_MIN_ALM_VALUE;
		hour_reg = SPRD_RTC_HOUR_ALM_VALUE;
		day_reg = SPRD_RTC_DAY_ALM_VALUE;
		break;
	case SPRD_RTC_AUX_ALARM:
		sec_reg = SPRD_RTC_SEC_AUXALM_UPD;
		min_reg = SPRD_RTC_MIN_AUXALM_UPD;
		hour_reg = SPRD_RTC_HOUR_AUXALM_UPD;
		day_reg = SPRD_RTC_DAY_AUXALM_UPD;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(rtc->regmap, rtc->base + sec_reg, &val);
	if (ret)
		return ret;

	sec = val & SPRD_RTC_SEC_MASK;

	ret = regmap_read(rtc->regmap, rtc->base + min_reg, &val);
	if (ret)
		return ret;

	min = val & SPRD_RTC_MIN_MASK;

	ret = regmap_read(rtc->regmap, rtc->base + hour_reg, &val);
	if (ret)
		return ret;

	hour = val & SPRD_RTC_HOUR_MASK;

	ret = regmap_read(rtc->regmap, rtc->base + day_reg, &val);
	if (ret)
		return ret;

	day = val & SPRD_RTC_DAY_MASK;
	*secs = (((time64_t)(day * 24) + hour) * 60 + min) * 60 + sec;
	return 0;
}

static int sprd_rtc_set_secs(struct sprd_rtc *rtc, enum sprd_rtc_reg_types type,
			     time64_t secs)
{
	u32 sec_reg, min_reg, hour_reg, day_reg, sts_mask;
	u32 sec, min, hour, day, val;
	int ret, rem;

	/* convert seconds to RTC time format */
	day = div_s64_rem(secs, 86400, &rem);
	hour = rem / 3600;
	rem -= hour * 3600;
	min = rem / 60;
	sec = rem - min * 60;

	switch (type) {
	case SPRD_RTC_TIME:
		sec_reg = SPRD_RTC_SEC_CNT_UPD;
		min_reg = SPRD_RTC_MIN_CNT_UPD;
		hour_reg = SPRD_RTC_HOUR_CNT_UPD;
		day_reg = SPRD_RTC_DAY_CNT_UPD;
		sts_mask = SPRD_RTC_TIME_INT_MASK;
		break;
	case SPRD_RTC_ALARM:
		sec_reg = SPRD_RTC_SEC_ALM_UPD;
		min_reg = SPRD_RTC_MIN_ALM_UPD;
		hour_reg = SPRD_RTC_HOUR_ALM_UPD;
		day_reg = SPRD_RTC_DAY_ALM_UPD;
		sts_mask = SPRD_RTC_ALMTIME_INT_MASK;
		break;
	case SPRD_RTC_AUX_ALARM:
		sec_reg = SPRD_RTC_SEC_AUXALM_UPD;
		min_reg = SPRD_RTC_MIN_AUXALM_UPD;
		hour_reg = SPRD_RTC_HOUR_AUXALM_UPD;
		day_reg = SPRD_RTC_DAY_AUXALM_UPD;
		sts_mask = 0;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(rtc->regmap, rtc->base + sec_reg, sec);
	if (ret)
		return ret;

	ret = regmap_write(rtc->regmap, rtc->base + min_reg, min);
	if (ret)
		return ret;

	ret = regmap_write(rtc->regmap, rtc->base + hour_reg, hour);
	if (ret)
		return ret;

	ret = regmap_write(rtc->regmap, rtc->base + day_reg, day);
	if (ret)
		return ret;

	if (type == SPRD_RTC_AUX_ALARM)
		return 0;

	/*
	 * Since the time and normal alarm registers are put in always-power-on
	 * region supplied by VDDRTC, then these registers changing time will
	 * be very long, about 125ms. Thus here we should wait until all
	 * values are updated successfully.
	 */
	ret = regmap_read_poll_timeout(rtc->regmap,
				       rtc->base + SPRD_RTC_INT_RAW_STS, val,
				       ((val & sts_mask) == sts_mask),
				       SPRD_RTC_POLL_DELAY_US,
				       SPRD_RTC_POLL_TIMEOUT);
	if (ret < 0) {
		dev_err(rtc->dev, "set time/alarm values timeout\n");
		return ret;
	}

	return regmap_write(rtc->regmap, rtc->base + SPRD_RTC_INT_CLR,
			    sts_mask);
}

static int sprd_rtc_set_aux_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	time64_t secs = rtc_tm_to_time64(&alrm->time);
	int ret;

	/* clear the auxiliary alarm interrupt status */
	ret = regmap_write(rtc->regmap, rtc->base + SPRD_RTC_INT_CLR,
			   SPRD_RTC_AUXALM_EN);
	if (ret)
		return ret;

	ret = sprd_rtc_set_secs(rtc, SPRD_RTC_AUX_ALARM, secs);
	if (ret)
		return ret;

	if (alrm->enabled) {
		ret = regmap_update_bits(rtc->regmap,
					 rtc->base + SPRD_RTC_INT_EN,
					 SPRD_RTC_AUXALM_EN,
					 SPRD_RTC_AUXALM_EN);
	} else {
		ret = regmap_update_bits(rtc->regmap,
					 rtc->base + SPRD_RTC_INT_EN,
					 SPRD_RTC_AUXALM_EN, 0);
	}

	return ret;
}

static int sprd_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	time64_t secs;
	int ret;

	if (!rtc->valid) {
		dev_warn(dev, "RTC values are invalid\n");
		return -EINVAL;
	}

	ret = sprd_rtc_get_secs(rtc, SPRD_RTC_TIME, &secs);
	if (ret)
		return ret;

	rtc_time64_to_tm(secs, tm);
	return 0;
}

static int sprd_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	time64_t secs = rtc_tm_to_time64(tm);
	int ret;

	ret = sprd_rtc_set_secs(rtc, SPRD_RTC_TIME, secs);
	if (ret)
		return ret;

	if (!rtc->valid) {
		/* Clear RTC power status firstly */
		ret = regmap_write(rtc->regmap, rtc->base + SPRD_RTC_PWR_CTRL,
				   SPRD_RTC_POWER_STS_CLEAR);
		if (ret)
			return ret;

		/*
		 * Set RTC power status to indicate now RTC has valid time
		 * values.
		 */
		ret = regmap_write(rtc->regmap, rtc->base + SPRD_RTC_PWR_CTRL,
				   SPRD_RTC_POWER_STS_VALID);
		if (ret)
			return ret;

		rtc->valid = true;
	}

	return 0;
}

static int sprd_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	time64_t secs;
	int ret;
	u32 val;

	/*
	 * The RTC core checks to see if there is an alarm already set in RTC
	 * hardware, and we always read the normal alarm at this time.
	 */
	ret = sprd_rtc_get_secs(rtc, SPRD_RTC_ALARM, &secs);
	if (ret)
		return ret;

	rtc_time64_to_tm(secs, &alrm->time);

	ret = regmap_read(rtc->regmap, rtc->base + SPRD_RTC_INT_EN, &val);
	if (ret)
		return ret;

	alrm->enabled = !!(val & SPRD_RTC_ALARM_EN);

	ret = regmap_read(rtc->regmap, rtc->base + SPRD_RTC_INT_RAW_STS, &val);
	if (ret)
		return ret;

	alrm->pending = !!(val & SPRD_RTC_ALARM_EN);
	return 0;
}

static int sprd_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	time64_t secs = rtc_tm_to_time64(&alrm->time);
	struct rtc_time aie_time =
		rtc_ktime_to_tm(rtc->rtc->aie_timer.node.expires);
	int ret;

	/*
	 * We have 2 groups alarms: normal alarm and auxiliary alarm. Since
	 * both normal alarm event and auxiliary alarm event can wake up system
	 * from deep sleep, but only alarm event can power up system from power
	 * down status. Moreover we do not need to poll about 125ms when
	 * updating auxiliary alarm registers. Thus we usually set auxiliary
	 * alarm when wake up system from deep sleep, and for other scenarios,
	 * we should set normal alarm with polling status.
	 *
	 * So here we check if the alarm time is set by aie_timer, if yes, we
	 * should set normal alarm, if not, we should set auxiliary alarm which
	 * means it is just a wake event.
	 */
	if (!rtc->rtc->aie_timer.enabled || rtc_tm_sub(&aie_time, &alrm->time))
		return sprd_rtc_set_aux_alarm(dev, alrm);

	/* clear the alarm interrupt status firstly */
	ret = regmap_write(rtc->regmap, rtc->base + SPRD_RTC_INT_CLR,
			   SPRD_RTC_ALARM_EN);
	if (ret)
		return ret;

	ret = sprd_rtc_set_secs(rtc, SPRD_RTC_ALARM, secs);
	if (ret)
		return ret;

	if (alrm->enabled) {
		ret = regmap_update_bits(rtc->regmap,
					 rtc->base + SPRD_RTC_INT_EN,
					 SPRD_RTC_ALARM_EN, SPRD_RTC_ALARM_EN);
		if (ret)
			return ret;

		/* unlock the alarm to enable the alarm function. */
		ret = sprd_rtc_lock_alarm(rtc, false);
	} else {
		regmap_update_bits(rtc->regmap, rtc->base + SPRD_RTC_INT_EN,
				   SPRD_RTC_ALARM_EN, 0);

		/*
		 * Lock the alarm function in case fake alarm event will power
		 * up systems.
		 */
		ret = sprd_rtc_lock_alarm(rtc, true);
	}

	return ret;
}

static int sprd_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	int ret;

	if (enabled) {
		ret = regmap_update_bits(
			rtc->regmap, rtc->base + SPRD_RTC_INT_EN,
			SPRD_RTC_ALARM_EN | SPRD_RTC_AUXALM_EN,
			SPRD_RTC_ALARM_EN | SPRD_RTC_AUXALM_EN);
		if (ret)
			return ret;

		ret = sprd_rtc_lock_alarm(rtc, false);
	} else {
		regmap_update_bits(rtc->regmap, rtc->base + SPRD_RTC_INT_EN,
				   SPRD_RTC_ALARM_EN | SPRD_RTC_AUXALM_EN, 0);

		ret = sprd_rtc_lock_alarm(rtc, true);
	}

	return ret;
}

static const struct rtc_class_ops sprd_rtc_ops = {
	.read_time = sprd_rtc_read_time,
	.set_time = sprd_rtc_set_time,
	.read_alarm = sprd_rtc_read_alarm,
	.set_alarm = sprd_rtc_set_alarm,
	.alarm_irq_enable = sprd_rtc_alarm_irq_enable,
};

static irqreturn_t sprd_rtc_handler(int irq, void *dev_id)
{
	struct sprd_rtc *rtc = dev_id;
	int ret;

	ret = sprd_rtc_clear_alarm_ints(rtc);
	if (ret)
		return IRQ_RETVAL(ret);

	rtc_update_irq(rtc->rtc, 1, RTC_AF | RTC_IRQF);
	return IRQ_HANDLED;
}

static int sprd_rtc_check_power_down(struct sprd_rtc *rtc)
{
	u32 val;
	int ret;

	ret = regmap_read(rtc->regmap, rtc->base + SPRD_RTC_PWR_STS, &val);
	if (ret)
		return ret;

	/*
	 * If the RTC power status value is SPRD_RTC_POWER_RESET_VALUE, which
	 * means the RTC has been powered down, so the RTC time values are
	 * invalid.
	 */
	rtc->valid = val != SPRD_RTC_POWER_RESET_VALUE;
	return 0;
}

static int sprd_rtc_check_alarm_int(struct sprd_rtc *rtc)
{
	u32 val;
	int ret;

	ret = regmap_read(rtc->regmap, rtc->base + SPRD_RTC_SPG_VALUE, &val);
	if (ret)
		return ret;

	/*
	 * The SPRD_RTC_INT_EN register is not put in always-power-on region
	 * supplied by VDDRTC, so we should check if we need enable the alarm
	 * interrupt when system booting.
	 *
	 * If we have set SPRD_RTC_POWEROFF_ALM_FLAG which is saved in
	 * always-power-on region, that means we have set one alarm last time,
	 * so we should enable the alarm interrupt to help RTC core to see if
	 * there is an alarm already set in RTC hardware.
	 */
	if (!(val & SPRD_RTC_POWEROFF_ALM_FLAG))
		return 0;

	return regmap_update_bits(rtc->regmap, rtc->base + SPRD_RTC_INT_EN,
				  SPRD_RTC_ALARM_EN, SPRD_RTC_ALARM_EN);
}

enum udx_boot_target { UDX_BOOT_NONE, UDX_BOOT_OPENWRT, UDX_BOOT_RECOVERY };
static int trial_boot_target(void)
{
	const char *policy, *target;

	if (!of_find_property(of_chosen, "udx,boot-policy", NULL))
		return of_find_property(of_chosen, "udx,boot-target", NULL) ?
			       -EINVAL :
			       UDX_BOOT_NONE;
	if (of_property_read_string(of_chosen, "udx,boot-policy", &policy) ||
	    strcmp(policy, "openwrt-v1") ||
	    of_property_read_string(of_chosen, "udx,boot-target", &target) ||
	    !of_property_read_bool(of_chosen, "udx,trial-local-lease"))
		return -EINVAL;
	if (!strcmp(target, "openwrt"))
		return UDX_BOOT_OPENWRT;
	if (!strcmp(target, "recovery"))
		return UDX_BOOT_RECOVERY;
	return -EINVAL;
}

static unsigned int trial_ready_stage(void)
{
	return trial_boot_target() == UDX_BOOT_RECOVERY ?
		       UDX_RTC_RECOVERY_READY :
		       UDX_RTC_RUNTIME_VERIFIED;
}

static bool trial_local_enabled(struct sprd_rtc *rtc)
{
	return rtc->base == 0x200 && trial_boot_target() > UDX_BOOT_NONE &&
	       of_property_read_bool(of_chosen, "udx,trial-rtc-stages");
}

/*
 * The trial profile exposes forward-only userspace checkpoints.
 * The regmap owns the ADI channel here; the raw early writer has finished.
 */
static int udx_trial_stage_set(struct sprd_rtc *rtc, unsigned int from,
			       unsigned int to)
{
	unsigned int spg, raw;
	int ret;

	if (rtc->base != 0x200 ||
	    !of_property_read_bool(of_chosen, "udx,trial-rtc-stages"))
		return -ENODEV;
	ret = regmap_read(rtc->regmap, rtc->base + 0x50, &spg);
	if (ret)
		return ret;
	if (spg != from || !udx_rtc_stage_value(from) ||
	    !udx_rtc_stage_value(to))
		return -EPROTO;
	ret = regmap_write(rtc->regmap, rtc->base + 0x38, 0x80);
	if (ret)
		return ret;
	ret = regmap_read(rtc->regmap, rtc->base + 0x34, &raw);
	if (ret)
		return ret;
	if (raw & 0x80)
		return -EIO;
	ret = regmap_write(rtc->regmap, rtc->base + 0x54, to);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(rtc->regmap, rtc->base + 0x34, raw,
				       raw & 0x80, 1000, 200000);
	if (ret)
		return ret;
	ret = regmap_read(rtc->regmap, rtc->base + 0x50, &spg);
	if (ret)
		return ret;
	if (spg != to)
		return -EIO;
	ret = regmap_write(rtc->regmap, rtc->base + 0x38, 0x80);
	if (ret)
		return ret;
	dev_info(rtc->dev,
		 "trial checkpoint %04x -> %04x; PMIC deadline untouched\n",
		 from, to);
	return 0;
}

static ssize_t udx_trial_stage_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	unsigned int value;
	int ret = regmap_read(rtc->regmap, rtc->base + 0x50, &value);

	return ret ? ret : sysfs_emit(buf, "%04x\n", value);
}

static ssize_t udx_trial_stage_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	unsigned int from, to;
	int ret;

	if (!trial_local_enabled(rtc))
		return -EOPNOTSUPP;
	if (!sysfs_streq(buf, "initramfs"))
		return -EINVAL;
	from = UDX_RTC_DRIVER_FRAMEWORK;
	to = UDX_RTC_INITRAMFS;
	mutex_lock(&rtc->rtc->ops_lock);
	ret = udx_trial_stage_set(rtc, from, to);
	mutex_unlock(&rtc->rtc->ops_lock);
	return ret ? ret : count;
}

static DEVICE_ATTR_RW(udx_trial_stage);

static int lease_read(struct sprd_rtc *rtc, unsigned int reg,
		      unsigned int *value)
{
	return regmap_read(rtc->regmap, reg, value);
}

static int udx_lease_peek(struct sprd_rtc *rtc, unsigned int *count)
{
	if (!count)
		return -EINVAL;
	for (unsigned int i = 0; i < 4; i++) {
		unsigned int hi, lo, again;
		int ret;

		ret = lease_read(rtc, 0x5c, &hi);
		if (ret)
			return ret;
		ret = lease_read(rtc, 0x58, &lo);
		if (ret)
			return ret;
		ret = lease_read(rtc, 0x5c, &again);
		if (ret)
			return ret;
		if (hi == again) {
			*count = (hi << 16) | lo;
			return 0;
		}
	}
	return -EAGAIN;
}

static int udx_lease_claim(struct sprd_rtc *rtc, struct udx_lease_state *s)
{
	unsigned int control;
	int ret;

	if (!s || !s->ready_stage)
		return -EINVAL;
	if (s->closed)
		return -ESHUTDOWN;
	ret = lease_read(rtc, 0x48, &control);
	if (ret)
		return ret;
	return (control & 14u) == 14u ? 0 : -EPROTO;
}

static int lease_idle(struct sprd_rtc *rtc)
{
	for (unsigned int i = 0; i < 64; i++) {
		unsigned int raw;
		int ret = lease_read(rtc, 0x50, &raw);

		if (ret)
			return ret;
		if (!(raw & 0x10))
			return 0;
		if (i + 1 < 64)
			udelay(10);
	}
	return -ETIMEDOUT;
}

static int lease_load(struct sprd_rtc *rtc, unsigned int high)
{
	const unsigned int regs[] = { 0x60, 0x44, 0x40, 0x60 };
	const unsigned int values[] = { 0xe551, high, 0, 0x1aae };
	int ret = lease_idle(rtc);

	if (ret)
		return ret;
	for (unsigned int i = 0; i < 4; i++) {
		ret = regmap_write(rtc->regmap, regs[i], values[i]);
		if (ret)
			return ret;
	}
	udelay(100);
	return lease_idle(rtc);
}

static void udx_lease_close(struct udx_lease_state *s)
{
	if (s)
		s->closed = 1;
}

static int udx_lease_renew(struct sprd_rtc *rtc, struct udx_lease_state *s)
{
	int ret;

	if (!s)
		return -EINVAL;
	if (s->closed)
		return -ESHUTDOWN;
	ret = lease_load(rtc, 30);

	if (!ret) {
		s->renewals++;
		s->last_renew_ms = ktime_get_boottime_ns() / 1000000;
	}
	return ret;
}

static int lease_reason(struct sprd_rtc *rtc, unsigned int value)
{
	unsigned int prior;
	int ret = lease_read(rtc, 0x1bac, &prior);

	if (ret)
		return ret;
	return regmap_write(rtc->regmap, 0x1bac, (prior & 0x200u) | value);
}

static int lease_boot_window(struct sprd_rtc *rtc, struct udx_lease_state *s,
			     unsigned int recovery, unsigned int high)
{
	int ret;

	if (!s || recovery > 1 || !s->ready_stage)
		return -EINVAL;
	if (s->closed)
		return -ESHUTDOWN;
	udx_lease_close(s);
	ret = udx_trial_stage_set(rtc, s->ready_stage,
				  recovery ? 0xbc5a : 0xb65a);
	if (ret)
		return ret;
	ret = lease_reason(rtc, recovery ? 0x20 : 0x40);
	if (ret)
		return ret;
	return lease_load(rtc, high);
}

static int udx_lease_boot_reboot(struct sprd_rtc *rtc,
				 struct udx_lease_state *s,
				 unsigned int recovery)
{
	return lease_boot_window(rtc, s, recovery, 6);
}

static int udx_lease_boot_upgrade(struct sprd_rtc *rtc,
				  struct udx_lease_state *s)
{
	if (!s || (s->ready_stage != 0xbe5a && s->ready_stage != 0xbc5a))
		return -EINVAL;
	return lease_boot_window(rtc, s, 0, 30);
}

static int trial_lease_admit(struct sprd_rtc *rtc)
{
	if (!trial_local_enabled(rtc))
		return -ENODEV;
	return rtc->trial_local_owner == task_tgid(current) ? 0 : -EPERM;
}

static void trial_owner_put(void *data)
{
	struct sprd_rtc *rtc = data;

	put_pid(rtc->trial_local_owner);
}

static ssize_t udx_trial_lease_owner_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	struct task_struct *owner;
	unsigned int stage;
	int ret;

	if (!sysfs_streq(buf, "claim"))
		return -EINVAL;
	if (!trial_local_enabled(rtc))
		return -EOPNOTSUPP;
	mutex_lock(&rtc->rtc->ops_lock);
	if (rtc->trial_lease.closed) {
		ret = -EBUSY;
		goto out;
	}
	owner = rtc->trial_local_owner ?
			get_pid_task(rtc->trial_local_owner, PIDTYPE_TGID) :
			NULL;
	if (owner) {
		bool active = !READ_ONCE(owner->exit_state);

		put_task_struct(owner);
		if (active) {
			ret = -EBUSY;
			goto out;
		}
	}
	rtc->trial_lease.ready_stage = trial_ready_stage();
	ret = regmap_read(rtc->regmap, rtc->base + 0x50, &stage);
	if (ret)
		goto out;
	if (stage == UDX_RTC_INITRAMFS)
		ret = udx_trial_stage_set(rtc, stage, trial_ready_stage());
	else if (stage != trial_ready_stage())
		ret = -EPROTO;
	if (ret)
		goto out;
	ret = udx_lease_claim(rtc, &rtc->trial_lease);
	if (ret)
		goto out;
	put_pid(rtc->trial_local_owner);
	rtc->trial_local_owner = get_task_pid(current, PIDTYPE_TGID);
	if (!rtc->trial_owner_cleanup) {
		ret = devm_add_action_or_reset(rtc->dev, trial_owner_put, rtc);
		if (ret) {
			rtc->trial_local_owner = NULL;
			goto out;
		}
		rtc->trial_owner_cleanup = true;
	}
	dev_info(dev,
		 "runtime watchdog owner pid %d; existing countdown retained\n",
		 pid_nr(rtc->trial_local_owner));
out:
	mutex_unlock(&rtc->rtc->ops_lock);
	return ret ? ret : count;
}

static DEVICE_ATTR_WO(udx_trial_lease_owner);

static ssize_t udx_trial_lease_status_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	unsigned int count, stage;
	int ret;

	if (!trial_local_enabled(rtc))
		return -ENODEV;
	mutex_lock(&rtc->rtc->ops_lock);
	ret = udx_lease_peek(rtc, &count);
	if (!ret)
		ret = regmap_read(rtc->regmap, rtc->base + 0x50, &stage);
	if (!ret)
		ret = sysfs_emit(
			buf,
			"mode=standalone owner=%d stage=%04x remaining_ticks=%u renewals=%u closed=%u last_renew_ms=%llu\n",
			rtc->trial_local_owner ?
				pid_nr(rtc->trial_local_owner) :
				0,
			stage, count, rtc->trial_lease.renewals,
			rtc->trial_lease.closed,
			(unsigned long long)rtc->trial_lease.last_renew_ms);
	mutex_unlock(&rtc->rtc->ops_lock);
	return ret;
}

static DEVICE_ATTR_RO(udx_trial_lease_status);
static ssize_t udx_boot_policy_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int target = trial_boot_target();

	if (target <= UDX_BOOT_NONE)
		return -ENODEV;
	return sysfs_emit(buf, "policy=openwrt-v1 target=%s ready_stage=%04x\n",
			  target == UDX_BOOT_RECOVERY ? "recovery" : "openwrt",
			  trial_ready_stage());
}

static DEVICE_ATTR_RO(udx_boot_policy);
static ssize_t udx_trial_lease_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	bool close = sysfs_streq(buf, "close");
	int ret;

	if (!close && !sysfs_streq(buf, "renew"))
		return -EINVAL;
	mutex_lock(&rtc->rtc->ops_lock);
	ret = trial_lease_admit(rtc);
	if (!ret) {
		if (close)
			udx_lease_close(&rtc->trial_lease);
		else
			ret = udx_lease_renew(rtc, &rtc->trial_lease);
	}
	mutex_unlock(&rtc->rtc->ops_lock);
	return ret ? ret : count;
}

static DEVICE_ATTR_WO(udx_trial_lease);
static ssize_t udx_trial_reboot_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sprd_rtc *rtc = dev_get_drvdata(dev);
	char target[16], extra;
	int ret, boot_target = trial_boot_target();
	bool recovery = false, upgrade = false;

	if (sscanf(buf, "%15s %c", target, &extra) != 1)
		return -EINVAL;
	if (boot_target <= UDX_BOOT_NONE)
		return -ENODEV;
	if (!strcmp(target, "recovery"))
		recovery = true;
	else if (!strcmp(target, "upgrade"))
		upgrade = true;
	else if (strcmp(target, "openwrt"))
		return -EINVAL;
	mutex_lock(&rtc->rtc->ops_lock);
	ret = trial_lease_admit(rtc);
	if (ret) {
		mutex_unlock(&rtc->rtc->ops_lock);
		return ret;
	}
	if (upgrade)
		ret = udx_lease_boot_upgrade(rtc, &rtc->trial_lease);
	else
		ret = udx_lease_boot_reboot(rtc, &rtc->trial_lease, recovery);
	mutex_unlock(&rtc->rtc->ops_lock);
	if (ret)
		dev_warn(dev, "trial reboot %s refused: %d; renewals closed\n",
			 target, ret);
	else
		dev_info(dev, "trial reboot %s queued; CTRL remains14\n",
			 target);
	return ret ? ret : count;
}

static DEVICE_ATTR_WO(udx_trial_reboot);
static struct attribute *udx_trial_attrs[] = {
	&dev_attr_udx_trial_stage.attr,
	&dev_attr_udx_trial_lease.attr,
	&dev_attr_udx_trial_lease_owner.attr,
	&dev_attr_udx_trial_lease_status.attr,
	&dev_attr_udx_boot_policy.attr,
	&dev_attr_udx_trial_reboot.attr,
	NULL,
};
static const struct attribute_group udx_trial_group = {
	.attrs = udx_trial_attrs,
};

static int sprd_rtc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct sprd_rtc *rtc;
	int ret;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!rtc->regmap)
		return -ENODEV;

	ret = of_property_read_u32(node, "reg", &rtc->base);
	if (ret) {
		dev_err(&pdev->dev, "failed to get RTC base address\n");
		return ret;
	}

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq < 0)
		return rtc->irq;

	rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc->rtc))
		return PTR_ERR(rtc->rtc);

	rtc->dev = &pdev->dev;
	platform_set_drvdata(pdev, rtc);

	/* check if we need set the alarm interrupt */
	ret = sprd_rtc_check_alarm_int(rtc);
	if (ret) {
		dev_err(&pdev->dev, "failed to check RTC alarm interrupt\n");
		return ret;
	}

	/* check if RTC time values are valid */
	ret = sprd_rtc_check_power_down(rtc);
	if (ret) {
		dev_err(&pdev->dev, "failed to check RTC time values\n");
		return ret;
	}

	ret = devm_request_threaded_irq(&pdev->dev, rtc->irq, NULL,
					sprd_rtc_handler,
					IRQF_ONESHOT | IRQF_EARLY_RESUME,
					pdev->name, rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request RTC irq\n");
		return ret;
	}

	device_init_wakeup(&pdev->dev, true);

	rtc->rtc->ops = &sprd_rtc_ops;
	rtc->rtc->range_min = 0;
	rtc->rtc->range_max = 5662310399LL;
	ret = devm_rtc_register_device(rtc->rtc);
	if (ret) {
		device_init_wakeup(&pdev->dev, false);
		return ret;
	}
	if (of_property_read_bool(of_chosen, "udx,trial-rtc-stages")) {
		ret = udx_trial_stage_set(rtc, UDX_RTC_DRIVER_FRAMEWORK,
					  UDX_RTC_DRIVER_FRAMEWORK);
		if (!ret)
			ret = devm_device_add_group(&pdev->dev,
						    &udx_trial_group);
		if (ret)
			dev_warn(&pdev->dev,
				 "trial checkpoint setup failed: %d\n", ret);
	}

	return 0;
}

static const struct of_device_id sprd_rtc_of_match[] = {
	{
		.compatible = "sprd,sc2731-rtc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sprd_rtc_of_match);

static struct platform_driver sprd_rtc_driver = {
	.driver = {
		.name = "sprd-rtc",
		.of_match_table = sprd_rtc_of_match,
	},
	.probe	= sprd_rtc_probe,
};
module_platform_driver(sprd_rtc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Spreadtrum RTC Device Driver");
MODULE_AUTHOR("Baolin Wang <baolin.wang@spreadtrum.com>");
