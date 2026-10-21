#ifndef _SIPA_ETH_H_
#define _SIPA_ETH_H_

#include <linux/sipa.h>
#include <linux/if.h>

/* Struct of data transfer statistics */
struct sipa_eth_dtrans_stats {
	u32 rx_sum;
	u32 rx_cnt;
	u32 rx_fail;

	u32 tx_sum;
	u32 tx_cnt;
	u32 tx_fail;
};

/* Device instance data. */
struct SIPA_ETH {
	int state;
	struct net_device *netdev; /* Linux net device */
	struct dentry *debugfs_dir;
	enum sipa_nic_id nic_id;
	struct napi_struct napi; /* Napi instance */
	/* Record data_transfer statistics */
	struct sipa_eth_dtrans_stats dt_stats;
	struct net_device_stats stats; /* Net statistics */
	u32 term_type;
	s32 netid;
	bool mac_h;
};
#endif
