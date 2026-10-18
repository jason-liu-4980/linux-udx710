// SPDX-License-Identifier: GPL-2.0-only
/*
 * Orca secure-firmware CPU DVFS client.
 * ABI reconstructed from the supplied AP Linux Image's SIP wrappers and
 * sprd-cpufreq caller, not from an invented register or voltage table.
 */
#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/topology.h>

#define ORCA_DVFS_REV 0x82000200
#define ORCA_DVFS_ENABLE 0x82000202
#define ORCA_DVFS_TABLE_UPDATE 0x82000203
#define ORCA_DVFS_CONFIG 0x82000204
#define ORCA_DVFS_SET_INDEX 0x82000205
#define ORCA_DVFS_GET_FREQ 0x82000206
#define ORCA_DVFS_GET_PAIR 0x82000207
/* Allocation/latency defensive bounds, not claimed silicon frequency limits. */
#define ORCA_MAX_OPPS 256
#define ORCA_MAX_LATENCY_US 10000

struct orca_dvfs {
	struct device *dev;
	struct mutex lock;
};
struct orca_policy {
	struct orca_dvfs *dvfs;
	struct device *cpu_dev;
	struct cpufreq_frequency_table *table;
	unsigned int count, added, cluster, latency_us;
};
static DEFINE_MUTEX(orca_registration_lock);
static struct orca_dvfs *orca_dvfs_instance;

/* Caller serializes all calls. Only a0 == 0 is success in the stock ABI. */
static int orca_smc(u32 fid, u32 cluster, u32 index, u32 voltage_step,
		    struct arm_smccc_res *res)
{
	/* CONFIG's voltage step is x4, NOT x2. All other unused arguments zero. */
	arm_smccc_smc(fid, cluster, index, 0, voltage_step, 0, 0, 0, res);
	if (!res->a0)
		return 0;
	if ((s32)res->a0 == SMCCC_RET_NOT_SUPPORTED)
		return -EOPNOTSUPP;
	return -EIO;
}

static int orca_read_frequency(struct orca_policy *p, unsigned long *hz)
{
	struct arm_smccc_res res;
	int ret = orca_smc(ORCA_DVFS_GET_FREQ, p->cluster, 0, 0, &res);

	if (ret)
		return ret;
	if (!res.a1 || res.a1 > U32_MAX || res.a1 % 1000)
		return -ERANGE;
	*hz = res.a1;
	return 0;
}

static unsigned int orca_get(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct orca_policy *p;
	unsigned long hz;
	int ret;

	if (!policy || !policy->driver_data)
		return 0;
	p = policy->driver_data;
	mutex_lock(&p->dvfs->lock);
	ret = orca_read_frequency(p, &hz);
	mutex_unlock(&p->dvfs->lock);
	return ret ? 0 : hz / 1000;
}

static int orca_set_index(struct orca_policy *p, unsigned int index)
{
	struct arm_smccc_res res;
	int ret = orca_smc(ORCA_DVFS_SET_INDEX, p->cluster, index, 0, &res);

	if (!ret && p->latency_us)
		udelay(p->latency_us);
	return ret;
}

static int orca_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct orca_policy *p = policy->driver_data;
	int ret;

	if (index >= p->count)
		return -EINVAL;
	mutex_lock(&p->dvfs->lock);
	ret = orca_set_index(p, policy->freq_table[index].driver_data);
	mutex_unlock(&p->dvfs->lock);
	return ret;
}

static void orca_free_opps(struct orca_policy *p)
{
	while (p->added) {
		p->added--;
		dev_pm_opp_remove(p->cpu_dev,
				  (unsigned long)p->table[p->added].frequency *
					  1000);
	}
	kfree(p->table);
	kfree(p);
}

static int orca_init_policy(struct cpufreq_policy *policy)
{
	struct orca_dvfs *d = cpufreq_get_driver_data();
	struct device_node *np, *other;
	struct arm_smccc_res res;
	struct orca_policy *p;
	struct device *cpu_dev;
	unsigned long hz;
	u32 step, boost;
	const char *stage = "CPU topology and DT";
	int cpu, cluster, ret;
	unsigned int i, j, firmware_count;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev)
		return -ENODEV;
	cluster = topology_physical_package_id(policy->cpu);
	/* Supplied Orca DT has one cluster0. Other domain encodings are unproven. */
	if (cluster != 0)
		return dev_err_probe(d->dev, -EOPNOTSUPP,
				     "CPU%u has unsupported package %d\n",
				     policy->cpu, cluster);
	np = of_parse_phandle(cpu_dev->of_node, "cpufreq-data-v1", 0);
	if (!np)
		return dev_err_probe(d->dev, -EINVAL,
				     "CPU%u lacks cpufreq-data-v1\n",
				     policy->cpu);
	p = kzalloc_obj(*p, GFP_KERNEL);
	if (!p) {
		of_node_put(np);
		return -ENOMEM;
	}
	p->dvfs = d;
	p->cpu_dev = cpu_dev;
	p->cluster = cluster;
	ret = of_property_read_u32(np, "voltage-step", &step);
	if (ret || !step) {
		ret = -EINVAL;
		goto put;
	}
	ret = of_property_read_u32(np, "tuning-latency-us", &p->latency_us);
	if (ret || p->latency_us > ORCA_MAX_LATENCY_US) {
		ret = -EINVAL;
		goto put;
	}
	ret = of_property_read_u32(np, "boost-duration", &boost);
	if (ret || boost) {
		/* No false-success boot/boost target suppression in this driver. */
		ret = -EOPNOTSUPP;
		goto put;
	}
	cpumask_clear(policy->cpus);
	for_each_possible_cpu(cpu) {
		struct device *dev = get_cpu_device(cpu);

		if (topology_physical_package_id(cpu) != cluster)
			continue;
		if (!dev) {
			ret = -EPROBE_DEFER;
			goto put;
		}
		other = of_parse_phandle(dev->of_node, "cpufreq-data-v1", 0);
		if (other != np) {
			of_node_put(other);
			ret = -EINVAL;
			goto put;
		}
		of_node_put(other);
		cpumask_set_cpu(cpu, policy->cpus);
	}
	of_node_put(np);
	np = NULL;
	mutex_lock(&d->lock);
	stage = "CONFIG";
	ret = orca_smc(ORCA_DVFS_CONFIG, cluster, 0, step, &res);
	if (ret)
		goto unlock;
	stage = "TABLE_UPDATE";
	ret = orca_smc(ORCA_DVFS_TABLE_UPDATE, cluster, 0, 0, &res);
	if (ret)
		goto unlock;
	if (!res.a1 || res.a1 > ORCA_MAX_OPPS) {
		ret = -ERANGE;
		goto unlock;
	}
	firmware_count = res.a1;
	p->table = kcalloc(firmware_count + 1, sizeof(*p->table), GFP_KERNEL);
	if (!p->table) {
		ret = -ENOMEM;
		goto unlock;
	}
	for (i = 0; i < firmware_count; i++) {
		stage = "GET_PAIR / OPP registration";
		ret = orca_smc(ORCA_DVFS_GET_PAIR, cluster, i, 0, &res);
		if (ret)
			goto unlock;
		if (!res.a1 || res.a1 > U32_MAX || res.a1 % 1000 || !res.a2 ||
		    res.a2 > U32_MAX) {
			ret = -ERANGE;
			goto unlock;
		}
		/* Firmware pads its table with repeated maximum-frequency rows.
		 * Linux needs unique OPPs, but SET_INDEX still takes the original
		 * firmware row number. Keep the first row for each frequency.
		 */
		for (j = 0; j < p->count; j++)
			if (p->table[j].frequency == res.a1 / 1000)
				break;
		if (j < p->count)
			continue;
		p->table[p->count].frequency = res.a1 / 1000;
		p->table[p->count].driver_data = i;
		ret = dev_pm_opp_add(cpu_dev, res.a1, res.a2);
		if (ret)
			goto unlock;
		p->added++;
		p->count++;
	}
	p->table[p->count].frequency = CPUFREQ_TABLE_END;
	stage = "GET_FREQ / initial OPP";
	ret = orca_read_frequency(p, &hz);
	if (ret)
		goto unlock;
	for (i = 0; i < p->count; i++)
		if ((unsigned long)p->table[i].frequency * 1000 == hz)
			break;
	if (i == p->count) {
		ret = -ERANGE;
		goto unlock;
	}
	stage = "ENABLE";
	ret = orca_smc(ORCA_DVFS_ENABLE, cluster, 0, 0, &res);
	if (ret)
		goto unlock;
	policy->driver_data = p;
	policy->freq_table = p->table;
	policy->cur = hz / 1000;
	policy->cpuinfo.transition_latency = p->latency_us * 1000;
	policy->transition_delay_us = p->latency_us;
	dev_info(d->dev,
		 "CPU%u: %u OPPs from %u firmware rows, current %lu Hz\n",
		 policy->cpu, p->count, firmware_count, hz);
	mutex_unlock(&d->lock);
	return 0;
unlock:
	mutex_unlock(&d->lock);
put:
	dev_err(d->dev, "CPU%u policy initialization failed at %s: %d\n",
		policy->cpu, stage, ret);
	of_node_put(np);
	orca_free_opps(p);
	return ret;
}

static void orca_exit_policy(struct cpufreq_policy *policy)
{
	struct orca_policy *p = policy->driver_data;

	policy->driver_data = NULL;
	policy->freq_table = NULL;
	orca_free_opps(p);
	/* No undocumented disable command or direct regulator write. */
}

static struct cpufreq_driver orca_cpufreq_driver = {
	.name = "sprd-orca-sip",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY | CPUFREQ_IS_COOLING_DEV,
	.init = orca_init_policy,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = orca_target,
	.get = orca_get,
	.exit = orca_exit_policy,
};

static int orca_cpufreq_probe(struct platform_device *pdev)
{
	struct arm_smccc_res rev;
	struct orca_dvfs *d;
	int ret;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = &pdev->dev;
	mutex_init(&d->lock);
	mutex_lock(&orca_registration_lock);
	if (orca_dvfs_instance) {
		ret = -EBUSY;
		goto out;
	}
	/* Revision has major/minor in a0/a1, unlike status-bearing operations. */
	arm_smccc_smc(ORCA_DVFS_REV, 0, 0, 0, 0, 0, 0, 0, &rev);
	if ((s32)rev.a0 < 0 || rev.a0 > U32_MAX || rev.a1 > U32_MAX) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	dev_info(&pdev->dev, "Orca SIP DVFS ABI %lu.%lu\n", rev.a0, rev.a1);
	orca_cpufreq_driver.driver_data = d;
	ret = cpufreq_register_driver(&orca_cpufreq_driver);
	if (!ret) {
		orca_dvfs_instance = d;
		platform_set_drvdata(pdev, d);
	} else {
		orca_cpufreq_driver.driver_data = NULL;
		dev_err(&pdev->dev,
			"CPU frequency driver registration failed: %d\n", ret);
	}
out:
	mutex_unlock(&orca_registration_lock);
	return ret;
}

static void orca_cpufreq_remove(struct platform_device *pdev)
{
	mutex_lock(&orca_registration_lock);
	cpufreq_unregister_driver(&orca_cpufreq_driver);
	orca_cpufreq_driver.driver_data = NULL;
	orca_dvfs_instance = NULL;
	mutex_unlock(&orca_registration_lock);
}

static const struct of_device_id orca_cpufreq_match[] = {
	{ .compatible = "sprd,orca-cpufreq" },
	{}
};
MODULE_DEVICE_TABLE(of, orca_cpufreq_match);
static struct platform_driver orca_cpufreq_platform_driver = {
	.probe = orca_cpufreq_probe,
	.remove = orca_cpufreq_remove,
	.driver = {
		.name = "sprd-orca-sip-cpufreq",
		.of_match_table = orca_cpufreq_match,
	},
};
module_platform_driver(orca_cpufreq_platform_driver);
MODULE_DESCRIPTION("Orca firmware-backed CPU frequency scaling");
MODULE_LICENSE("GPL");
