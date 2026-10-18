/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SIPA_FLOWTABLE_H
#define _SIPA_FLOWTABLE_H

#include <linux/netdevice.h>

#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
int sipa_flowtable_setup_tc(struct net_device *dev, enum tc_setup_type type,
			    void *type_data);
int sipa_eth_flowtable_port(struct net_device *dev, u8 *port);
#endif

#endif
