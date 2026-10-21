// SPDX-License-Identifier: GPL-2.0-only
/* Orca DDR frequency votes through the retained PM_SYS firmware.
 * The wire protocol is from sprd/sprd_dfs_auto_drv.c in the public BSP.
 * Firmware owns DDR transitions and arbitrates AP/CP demand; an AP vote is
 * not a command to change the EMC mux or program an invented voltage table.
 */
#include <linux/devfreq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/sipc.h>
#include <linux/sort.h>

#define ORCA_DFS_VOTE 0x0000
#define ORCA_DFS_ENABLE 0x0300
#define ORCA_DFS_AUTO_ENABLE 0x0310
#define ORCA_DFS_GET_FREQ 0x0500
#define ORCA_DFS_GET_AP 0x0502
#define ORCA_DFS_GET_TABLE 0x0505
#define ORCA_DFS_GET_STATUS 0x050a
#define ORCA_DFS_GET_AUTO 0x050b
#define ORCA_DFS_QUERY_OK 0x0500
#define ORCA_DFS_ENABLE_OK 0x0300
#define ORCA_DFS_VOTE_OK 0x0000
#define ORCA_DFS_VOTE_UNCHANGED 0x0003
#define ORCA_DFS_MAX_FREQS 32
#define ORCA_MHZ 1000000UL

struct orca_ddr {
	struct device *dev;
	struct mutex lock;
	struct devfreq_dev_profile profile;
	struct devfreq *devfreq;
	bool connected;
	unsigned int added_opps;
};

/* Serialize the whole request/reply exchange, including sysfs readers. */
static int orca_dfs_request(struct orca_ddr *d, u16 command, u32 value,
			    u32 *result)
{
	struct smsg message;
	bool switch_command = command == ORCA_DFS_ENABLE ||
			      command == ORCA_DFS_AUTO_ENABLE;
	u16 success = switch_command	       ? ORCA_DFS_ENABLE_OK :
		      command == ORCA_DFS_VOTE ? ORCA_DFS_VOTE_OK :
						 ORCA_DFS_QUERY_OK;
	int ret;

	lockdep_assert_held(&d->lock);
	smsg_set(&message, SMSG_CH_PM_CTRL, SMSG_TYPE_DFS, command, value);
	ret = smsg_send(SIPC_ID_PM_SYS, &message, msecs_to_jiffies(100));
	if (ret)
		return ret;
	ret = smsg_recv(SIPC_ID_PM_SYS, &message,
			msecs_to_jiffies(switch_command ? 2000 : 500));
	if (ret)
		return ret;
	if (message.channel != SMSG_CH_PM_CTRL ||
	    message.type != SMSG_TYPE_DFS_RSP)
		return -EPROTO;
	if (message.flag != success &&
	    !(command == ORCA_DFS_VOTE &&
	      message.flag == ORCA_DFS_VOTE_UNCHANGED)) {
		dev_err(d->dev,
			"DDR command %#x rejected: status %#x, value %u\n",
			command, message.flag, message.value);
		return message.flag == 2 ? -EBUSY : -EIO;
	}
	*result = message.value;
	return 0;
}

static int orca_ddr_read_hz(struct orca_ddr *d, u16 command, unsigned long *hz)
{
	u32 mhz;
	int ret = orca_dfs_request(d, command, 0, &mhz);

	if (!ret) {
		if (mhz > ULONG_MAX / ORCA_MHZ)
			return -ERANGE;
		*hz = (unsigned long)mhz * ORCA_MHZ;
	}
	return ret;
}

static int orca_ddr_get_freq(struct device *dev, unsigned long *hz)
{
	struct orca_ddr *d = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&d->lock);
	ret = orca_ddr_read_hz(d, ORCA_DFS_GET_FREQ, hz);
	mutex_unlock(&d->lock);
	return ret;
}

static int orca_ddr_target(struct device *dev, unsigned long *hz, u32 flags)
{
	struct orca_ddr *d = dev_get_drvdata(dev);
	unsigned long *table = d->profile.freq_table;
	unsigned int i, chosen = 0;
	u32 status;
	int ret;

	/* devfreq table is sorted ascending. Match the standard LUB convention. */
	if (flags & DEVFREQ_FLAG_LEAST_UPPER_BOUND) {
		for (i = 0; i < d->profile.max_state && table[i] <= *hz; i++)
			chosen = i;
	} else {
		chosen = d->profile.max_state - 1;
		for (i = 0; i < d->profile.max_state; i++)
			if (table[i] >= *hz) {
				chosen = i;
				break;
			}
	}
	mutex_lock(&d->lock);
	ret = orca_dfs_request(d, ORCA_DFS_GET_STATUS, 0, &status);
	if (!ret && !status)
		ret = orca_dfs_request(d, ORCA_DFS_ENABLE, 0, &status);
	if (!ret) {
		ret = orca_dfs_request(d, ORCA_DFS_VOTE,
				       table[chosen] / ORCA_MHZ, &status);
	}
	if (!ret)
		ret = orca_ddr_read_hz(d, ORCA_DFS_GET_FREQ, hz);
	mutex_unlock(&d->lock);
	return ret;
}

static int orca_freq_compare(const void *a, const void *b)
{
	unsigned long x = *(const unsigned long *)a;
	unsigned long y = *(const unsigned long *)b;

	return (x > y) - (x < y);
}

static void orca_ddr_remove_opps(void *data)
{
	struct orca_ddr *d = data;

	while (d->added_opps)
		dev_pm_opp_remove(d->dev,
				  d->profile.freq_table[--d->added_opps]);
}

static int orca_ddr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct orca_ddr *d;
	unsigned long *table;
	u32 count, mhz, enabled, automatic, ap_vote;
	int ret;
	unsigned int i;

	ret = of_property_read_u32(dev->of_node, "freq-num", &count);
	if (ret || !count || count > ORCA_DFS_MAX_FREQS)
		return dev_err_probe(
			dev, -EINVAL,
			"DDR freq-num must describe the firmware table\n");
	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	table = devm_kcalloc(dev, count, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	d->dev = dev;
	mutex_init(&d->lock);
	platform_set_drvdata(pdev, d);
	ret = smsg_ch_open(SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL,
			   msecs_to_jiffies(500));
	if (ret == -ENODEV)
		ret = -EPROBE_DEFER;
	if (ret)
		return dev_err_probe(dev, ret,
				     "PM_SYS DDR channel open failed\n");
	d->connected = true;
	mutex_lock(&d->lock);
	for (i = 0; i < count; i++) {
		ret = orca_dfs_request(d, ORCA_DFS_GET_TABLE, i, &mhz);
		if (ret)
			goto unlock;
		if (!mhz || mhz > ULONG_MAX / ORCA_MHZ) {
			ret = -ERANGE;
			goto unlock;
		}
		table[i] = (unsigned long)mhz * ORCA_MHZ;
	}
	sort(table, count, sizeof(*table), orca_freq_compare, NULL);
	for (i = 1; i < count; i++)
		if (table[i] == table[i - 1]) {
			ret = -EINVAL;
			goto unlock;
		}
	/* Product default: firmware controls DDR; no AP frequency request. */
	ret = orca_dfs_request(d, ORCA_DFS_ENABLE, 0, &enabled);
	if (ret)
		goto unlock;
	ret = orca_dfs_request(d, ORCA_DFS_AUTO_ENABLE, 0, &automatic);
	if (ret)
		goto unlock;
	ret = orca_dfs_request(d, ORCA_DFS_VOTE, 0, &ap_vote);
	if (ret)
		goto unlock;
	ret = orca_dfs_request(d, ORCA_DFS_GET_STATUS, 0, &enabled);
	if (ret)
		goto unlock;
	ret = orca_dfs_request(d, ORCA_DFS_GET_AUTO, 0, &automatic);
	if (ret)
		goto unlock;
	ret = orca_dfs_request(d, ORCA_DFS_GET_AP, 0, &ap_vote);
	if (ret)
		goto unlock;
	if (enabled != 1 || automatic != 1 || ap_vote) {
		ret = -EIO;
		goto unlock;
	}
	ret = orca_ddr_read_hz(d, ORCA_DFS_GET_FREQ, &d->profile.initial_freq);
	if (!ret && !d->profile.initial_freq)
		ret = -ERANGE;
unlock:
	mutex_unlock(&d->lock);
	if (ret)
		goto close;
	d->profile.target = orca_ddr_target;
	d->profile.get_cur_freq = orca_ddr_get_freq;
	d->profile.freq_table = table;
	d->profile.max_state = count;
	ret = devm_add_action_or_reset(dev, orca_ddr_remove_opps, d);
	if (ret)
		goto close;
	for (i = 0; i < count; i++) {
		/* Frequency-only OPP: voltage transitions stay inside PM firmware. */
		ret = dev_pm_opp_add(dev, table[i], 0);
		if (ret)
			goto close;
		d->added_opps++;
	}
	/*
	 * Keep normal devfreq monitoring and governor semantics. The product
	 * does not request frequencies; root can still use standard debug controls.
	 */
	dev_info(
		dev,
		"PM_SYS DDR: %u frequencies, current %lu Hz, DFS %u, auto %u, AP request %u MHz\n",
		count, d->profile.initial_freq, enabled, automatic, ap_vote);
	for (i = 0; i < count; i++)
		dev_info(dev, "DDR firmware frequency %u: %lu Hz\n", i,
			 table[i]);
	d->devfreq = devfreq_add_device(dev, &d->profile, DEVFREQ_GOV_USERSPACE,
					NULL);
	if (IS_ERR(d->devfreq)) {
		ret = PTR_ERR(d->devfreq);
		goto close;
	}
	return 0;
close:
	/* Release the existing channel after a failed probe. */
	smsg_ch_close(SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL, msecs_to_jiffies(100));
	d->connected = false;
	return dev_err_probe(dev, ret, "PM_SYS DDR discovery failed\n");
}

static void orca_ddr_remove(struct platform_device *pdev)
{
	struct orca_ddr *d = platform_get_drvdata(pdev);
	u32 status;
	int ret;

	devfreq_remove_device(d->devfreq);
	mutex_lock(&d->lock);
	ret = orca_dfs_request(d, ORCA_DFS_VOTE, 0, &status);
	if (ret)
		dev_warn(d->dev, "Could not release AP DDR vote: %d\n", ret);
	if (d->connected)
		smsg_ch_close(SIPC_ID_PM_SYS, SMSG_CH_PM_CTRL,
			      msecs_to_jiffies(100));
	mutex_unlock(&d->lock);
	/* Preserve firmware auto mode and other processors' votes. */
}

static const struct of_device_id orca_ddr_of_match[] = {
	{ .compatible = "sprd,orca-ddr-dfs" },
	{ .compatible = "sprd,dfs" },
	{}
};
MODULE_DEVICE_TABLE(of, orca_ddr_of_match);
static struct platform_driver orca_ddr_driver = {
	.probe = orca_ddr_probe,
	.remove = orca_ddr_remove,
	.driver = {
		.name = "sprd-orca-ddr",
		.of_match_table = orca_ddr_of_match,
	},
};
module_platform_driver(orca_ddr_driver);
MODULE_DESCRIPTION("Orca PM_SYS firmware DDR devfreq");
MODULE_LICENSE("GPL");
