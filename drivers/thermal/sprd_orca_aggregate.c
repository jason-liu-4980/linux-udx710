// SPDX-License-Identifier: GPL-2.0-only
/*
 * Orca aggregate temperature providers.
 * Algorithm: GPL sprd_cluster0_thm.c/sprd_soc_thm.c, Spreadtrum 2015.
 * Source lifetimes are managed with supplier device links, not global raw
 * thermal-zone pointers that can outlive their provider driver.
 */
#include <linux/device.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include "thermal_hwmon.h"

#define ORCA_AGG_MAX_SOURCES 16
struct orca_aggregate_source {
	struct device *supplier;
	struct thermal_zone_device *zone;
};
struct orca_aggregate {
	unsigned int count;
	struct orca_aggregate_source sources[ORCA_AGG_MAX_SOURCES];
};
struct orca_aggregate_variant {
	const char *source_compatible;
};
static const struct orca_aggregate_variant orca_cluster = {
	.source_compatible = "sprd,orca-thermal",
};
static const struct orca_aggregate_variant orca_soc = {
	.source_compatible = "sprd,cluster0-thermal",
};

static int orca_aggregate_get_temp(struct thermal_zone_device *zone, int *temp)
{
	struct orca_aggregate *agg = thermal_zone_device_priv(zone);
	int maximum = INT_MIN, value, ret;
	s64 sum = 0;
	unsigned int i;

	for (i = 0; i < agg->count; i++) {
		ret = thermal_zone_get_temp(agg->sources[i].zone, &value);
		if (ret)
			return ret;
		maximum = max(maximum, value);
		sum += value;
	}
	/* Preserve the BSP's 30C crossover and integer truncation semantics. */
	*temp = maximum >= 30000 ? maximum : div_s64(sum, agg->count);
	return 0;
}
static const struct thermal_zone_device_ops orca_aggregate_ops = {
	.get_temp = orca_aggregate_get_temp,
};

/* The ID is carried by the actual zone reference, not guessed from unit address. */
static int orca_aggregate_id(struct device_node *zones,
			     struct device_node *provider)
{
	struct of_phandle_args args;
	struct device_node *zone;
	int id = -ENODEV, ret;

	for_each_available_child_of_node(zones, zone) {
		ret = of_parse_phandle_with_args(zone, "thermal-sensors",
						 "#thermal-sensor-cells", 0,
						 &args);
		if (ret)
			continue;
		if (args.np == provider) {
			if (args.args_count != 1 || args.args[0] > INT_MAX ||
			    (id >= 0 && id != args.args[0])) {
				of_node_put(args.np);
				of_node_put(zone);
				return -EINVAL;
			}
			id = args.args[0];
		}
		of_node_put(args.np);
	}
	return id;
}

static int orca_aggregate_source(struct platform_device *pdev,
				 struct device_node *zones,
				 struct orca_aggregate *agg,
				 const struct orca_aggregate_variant *variant,
				 unsigned int index, const char *name)
{
	struct device_node *node;
	struct of_phandle_args args;
	struct platform_device *supplier;
	struct thermal_zone_device *zone;
	struct device_link *link;
	unsigned int i;
	int ret;

	node = of_get_child_by_name(zones, name);
	if (!node)
		return -EINVAL;
	if (!of_device_is_available(node)) {
		of_node_put(node);
		return -ENODEV;
	}
	ret = of_parse_phandle_with_args(node, "thermal-sensors",
					 "#thermal-sensor-cells", 0, &args);
	of_node_put(node);
	if (ret)
		return ret;
	/* Enforce the proven DAG: raw Orca -> cluster aggregate -> SoC aggregate. */
	if (args.np == pdev->dev.of_node ||
	    !of_device_is_compatible(args.np, variant->source_compatible)) {
		of_node_put(args.np);
		return -EINVAL;
	}
	supplier = of_find_device_by_node(args.np);
	of_node_put(args.np);
	if (!supplier)
		return -EPROBE_DEFER;
	for (i = 0; i < index; i++)
		if (agg->sources[i].supplier == &supplier->dev)
			break;
	if (i == index) {
		/* Managed link unbinds this consumer before the supplier's devres. */
		link = device_link_add(&pdev->dev, &supplier->dev,
				       DL_FLAG_AUTOREMOVE_CONSUMER);
		if (!link) {
			ret = -ENOMEM;
			goto put;
		}
	}
	/* Exclude supplier remove/probe while acquiring its registered zone. */
	device_lock(&supplier->dev);
	if (!device_is_bound(&supplier->dev)) {
		ret = -EPROBE_DEFER;
	} else {
		zone = thermal_zone_get_zone_by_name(name);
		if (IS_ERR(zone))
			ret = PTR_ERR(zone) == -ENODEV ? -EPROBE_DEFER :
							 PTR_ERR(zone);
		else {
			agg->sources[index].supplier = &supplier->dev;
			agg->sources[index].zone = zone;
			ret = 0;
		}
	}
	device_unlock(&supplier->dev);
put:
	put_device(&supplier->dev);
	return ret;
}

static int orca_aggregate_probe(struct platform_device *pdev)
{
	const struct orca_aggregate_variant *variant =
		of_device_get_match_data(&pdev->dev);
	struct device_node *np = pdev->dev.of_node, *zones;
	struct thermal_zone_device *zone;
	struct orca_aggregate *agg;
	const char *name, *prior;
	int count, id, i, j, ret;

	count = of_property_count_strings(np, "sensor-names");
	if (count < 1 || count > ORCA_AGG_MAX_SOURCES)
		return -EINVAL;
	zones = of_find_node_by_path("/thermal-zones");
	if (!zones)
		return -ENODEV;
	id = orca_aggregate_id(zones, np);
	if (id < 0) {
		ret = id;
		goto put;
	}
	agg = devm_kzalloc(&pdev->dev, sizeof(*agg), GFP_KERNEL);
	if (!agg) {
		ret = -ENOMEM;
		goto put;
	}
	agg->count = count;
	for (i = 0; i < count; i++) {
		ret = of_property_read_string_index(np, "sensor-names", i,
						    &name);
		if (ret)
			goto put;
		for (j = 0; j < i; j++) {
			of_property_read_string_index(np, "sensor-names", j,
						      &prior);
			if (!strcmp(prior, name)) {
				ret = -EINVAL;
				goto put;
			}
		}
		ret = orca_aggregate_source(pdev, zones, agg, variant, i, name);
		if (ret)
			goto put;
	}
	zone = devm_thermal_of_zone_register(&pdev->dev, id, agg,
					     &orca_aggregate_ops);
	ret = PTR_ERR_OR_ZERO(zone);
	if (!ret)
		ret = devm_thermal_add_hwmon_sysfs(&pdev->dev, zone);
put:
	of_node_put(zones);
	return ret ? dev_err_probe(&pdev->dev, ret,
				   "aggregate thermal sources\n") :
		     0;
}

static const struct of_device_id orca_aggregate_match[] = {
	{ .compatible = "sprd,cluster0-thermal", .data = &orca_cluster },
	{ .compatible = "sprd,soc-thermal", .data = &orca_soc },
	{}
};
MODULE_DEVICE_TABLE(of, orca_aggregate_match);
static struct platform_driver orca_aggregate_driver = {
	.probe = orca_aggregate_probe,
	.driver = {
		.name = "sprd-orca-aggregate-thermal",
		.of_match_table = orca_aggregate_match,
	},
};
module_platform_driver(orca_aggregate_driver);
MODULE_DESCRIPTION("Orca cluster and SoC aggregate temperatures");
MODULE_LICENSE("GPL");
