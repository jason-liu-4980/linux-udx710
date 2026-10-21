// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "sipa_eth: " fmt

#include "sipa_flowtable.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/if_arp.h>
#include <linux/wwan.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include "sipa_eth.h"
#include "sipa_priv.h"

/* Device status */
#define DEV_ON 1
#define DEV_OFF 0

#define SIPA_ETH_NAPI_WEIGHT 64
#define SIPA_ETH_IFACE_PREF "sipa_eth"

static struct dentry *root;
static int sipa_eth_debugfs_mknod(void *root, void *data);

static u64 gro_enable;

static inline void sipa_eth_dt_stats_init(struct sipa_eth_dtrans_stats *stats)
{
	memset(stats, 0, sizeof(*stats));
}

static inline void sipa_eth_rx_stats_update(struct sipa_eth_dtrans_stats *stats,
					    u32 len)
{
	stats->rx_sum += len;
	stats->rx_cnt++;
}

static inline void sipa_eth_tx_stats_update(struct sipa_eth_dtrans_stats *stats,
					    u32 len)
{
	stats->tx_sum += len;
	stats->tx_cnt++;
}

static int sipa_eth_prepare_skb(struct SIPA_ETH *sipa_eth, struct sk_buff *skb)
{
	struct iphdr *iph;
	struct ethhdr *peth;
	struct net_device *dev;
	unsigned int offset = sipa_eth->mac_h ? ETH_HLEN : 0;
	u8 version;

	dev = sipa_eth->netdev;
	/* Descriptor length validation does not prove an IP header is present. */
	if (!pskb_may_pull(skb, offset + 1))
		return -EINVAL;
	version = skb->data[offset] >> 4;
	if (version == 4) {
		if (!pskb_may_pull(skb, offset + sizeof(struct iphdr)))
			return -EINVAL;
		iph = (struct iphdr *)(skb->data + offset);
		if (iph->ihl < 5 || !pskb_may_pull(skb, offset + iph->ihl * 4))
			return -EINVAL;
	} else if (version == 6) {
		if (!pskb_may_pull(skb, offset + sizeof(struct ipv6hdr)))
			return -EINVAL;
	} else {
		return -EPROTONOSUPPORT;
	}
	if (!sipa_eth->mac_h && skb_cow_head(skb, ETH_HLEN))
		return -ENOMEM;

	/* eth is rawip */
	/*
	 * The purpose of adding a fake mac header is
	 * to prevent a "out-of order" issue
	 * when doing napi_gro_receive.
	 *
	 * On ORCA CPE, ASIC has set a 14byte offset
	 * in a skb for mac header,
	 * but the content of this mac header is not initialized,
	 * so we need to fulfill this offset with a fake mac header.
	 *
	 * On Roc1+Orca, a skb arrives at sipa_eth
	 * has no mac header or a offset.
	 * But it has a 64 byte header room,
	 * We have to take advantage of the 64-byte header room,
	 * because the information in the header room
	 * is no longer useful for IP layer.
	 */
	if (sipa_eth->mac_h) {
		skb_reset_mac_header(skb);
		peth = (struct ethhdr *)skb->data;
	} else {
		peth = (struct ethhdr *)skb_push(skb, ETH_HLEN);
		skb_reset_mac_header(skb);
	}
	ether_addr_copy(peth->h_source, "12345");
	ether_addr_copy(peth->h_dest, "54321");

	skb_set_network_header(skb, ETH_HLEN);
	iph = ip_hdr(skb);
	if (iph->version == 4)
		skb->protocol = htons(ETH_P_IP);
	else
		skb->protocol = htons(ETH_P_IPV6);

	peth->h_proto = skb->protocol;
	skb_pull_inline(skb, ETH_HLEN);

	/* TODO chechsum ... */
	skb->ip_summed = CHECKSUM_NONE;
	skb->dev = dev;
	return 0;
}

static int sipa_eth_rx(struct SIPA_ETH *sipa_eth, int budget)
{
	struct sk_buff *skb;
	struct net_device *dev;
	struct sipa_eth_dtrans_stats *dt_stats;
	int skb_cnt = 0;
	int ret;

	if (!sipa_eth) {
		pr_err("no sipa_eth device\n");
		return -EINVAL;
	}

	dt_stats = &sipa_eth->dt_stats;
	dev = sipa_eth->netdev;

	while (skb_cnt < budget) {
		ret = sipa_nic_rx(sipa_eth->nic_id, &skb);

		if (ret) {
			switch (ret) {
			case -ENODEV:
				pr_err("fail to find dev");
				sipa_eth->stats.rx_errors++;
				dt_stats->rx_fail++;
				break;
			case -ENODATA:
				pr_debug("no more skb to recv");
				break;
			}
			break;
		}

		if (!skb) {
			sipa_eth->stats.rx_errors++;
			break;
		}
		skb_cnt++;
		ret = sipa_eth_prepare_skb(sipa_eth, skb);
		if (ret) {
			sipa_eth->stats.rx_errors++;
			sipa_eth->stats.rx_dropped++;
			dt_stats->rx_fail++;
			dev_kfree_skb_any(skb);
			continue;
		}

		sipa_eth->stats.rx_packets++;
		sipa_eth->stats.rx_bytes += skb->len;
		sipa_eth_rx_stats_update(dt_stats, skb->len);

		if (gro_enable)
			napi_gro_receive(&sipa_eth->napi, skb);
		else
			netif_receive_skb(skb);
	}

	return skb_cnt;
}

static int sipa_eth_rx_poll_handler(struct napi_struct *napi, int budget)
{
	struct SIPA_ETH *sipa_eth = container_of(napi, struct SIPA_ETH, napi);
	int pkts;

	pkts = sipa_eth_rx(sipa_eth, budget);

	/* If the number of pkt is more than weight(64),
	 * we cannot read them all with a single poll.
	 * When the return value of poll func equals to weight(64),
	 * napi structure invokes the poll func one more time by
	 * __raise_softirq_irqoff.(See napi_poll for details)
	 * So do not do napi_complete in that case.
	 */
	if (pkts < budget)
		napi_complete_done(napi, pkts);
	return pkts;
}

static void sipa_eth_rx_handler(void *priv)
{
	struct SIPA_ETH *sipa_eth = (struct SIPA_ETH *)priv;

	if (!sipa_eth) {
		pr_err("data is NULL\n");
		return;
	}

	/* NAPI's MISSED bit closes the final-empty-read/completion race. */
	napi_schedule(&sipa_eth->napi);
}

static void sipa_eth_flowctrl_handler(void *priv, int flowctrl)
{
	struct SIPA_ETH *sipa_eth = (struct SIPA_ETH *)priv;
	struct net_device *dev = sipa_eth->netdev;

	if (flowctrl)
		netif_stop_queue(dev);
	else if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
}

static void sipa_eth_notify_cb(void *priv, enum sipa_evt_type evt,
			       unsigned long data)
{
	switch (evt) {
	case SIPA_RECEIVE:
		sipa_eth_rx_handler(priv);
		break;
	case SIPA_LEAVE_FLOWCTRL:
		pr_info("SIPA LEAVE FLOWCTRL\n");
		sipa_eth_flowctrl_handler(priv, 0);
		break;
	case SIPA_ENTER_FLOWCTRL:
		pr_info("SIPA ENTER FLOWCTRL\n");
		sipa_eth_flowctrl_handler(priv, 1);
		break;
	default:
		break;
	}
}

static int sipa_eth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);
	struct sipa_eth_dtrans_stats *dt_stats;
	int ret = 0;
	int netid;
	unsigned int payload_len;

	dt_stats = &sipa_eth->dt_stats;
	if (sipa_eth->state != DEV_ON) {
		pr_err("called when %s is down\n", dev->name);
		dt_stats->tx_fail++;
		netif_carrier_off(dev);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	netid = sipa_eth->netid;
	/* eth is rawip, so pull 14 bytes */
	if (!sipa_eth->mac_h) {
		if (!pskb_may_pull(skb, ETH_HLEN))
			goto err;
		skb_pull_inline(skb, ETH_HLEN);
	}
	/* Success transfers ownership: do not read skb after the call. */
	payload_len = skb->len;

	ret = sipa_nic_tx(sipa_eth->nic_id, sipa_eth->term_type, netid, skb);
	if (unlikely(ret != 0)) {
		pr_err("fail to send skb, ret %d\n", ret);
		if (ret == -EAGAIN || ret == -EINPROGRESS) {
			/* NETDEV_TX_BUSY leaves ownership with the networking stack. */
			if (!sipa_eth->mac_h)
				skb_push(skb, ETH_HLEN);
			dt_stats->tx_fail++;
			sipa_eth->stats.tx_errors++;
			netif_stop_queue(dev);
			sipa_nic_trigger_flow_ctrl_work(sipa_eth->nic_id, ret);
			return NETDEV_TX_BUSY;
		}
		goto err;
	}

	/* update netdev statistics */
	sipa_eth->stats.tx_packets++;
	sipa_eth->stats.tx_bytes += payload_len;
	sipa_eth_tx_stats_update(dt_stats, payload_len);

	return NETDEV_TX_OK;

err:
	sipa_eth->netdev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/* Open interface */
static int sipa_eth_open(struct net_device *dev)
{
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);
	int ret = 0;

	pr_info("open %s netid %d term_type %d mac_h %d\n", dev->name,
		sipa_eth->netid, sipa_eth->term_type, sipa_eth->mac_h);
	ret = sipa_nic_open(sipa_eth->term_type, sipa_eth->netid, sipa_eth_notify_cb,
			    (void *)sipa_eth);

	if (ret < 0)
		return ret;

	sipa_eth->nic_id = ret;
	sipa_eth->state = DEV_ON;

	sipa_eth_dt_stats_init(&sipa_eth->dt_stats);
	memset(&sipa_eth->stats, 0, sizeof(sipa_eth->stats));

	if (!netif_carrier_ok(sipa_eth->netdev)) {
		pr_info("set netif_carrier_on\n");
		netif_carrier_on(sipa_eth->netdev);
	}

	/* Open publishes a callback before returning its NIC id. Keep NAPI
	 * disabled until the id is installed, then poll once for that window.
	 */
	napi_enable(&sipa_eth->napi);
	napi_schedule(&sipa_eth->napi);
	netif_start_queue(dev);

	return 0;
}

/* Close interface */
static int sipa_eth_close(struct net_device *dev)
{
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);

	pr_info("close %s!\n", dev->name);

	netif_stop_queue(dev);
	sipa_nic_close(sipa_eth->nic_id);
	sipa_eth->state = DEV_OFF;
	napi_disable(&sipa_eth->napi);
	netif_carrier_off(dev);

	return 0;
}

static struct net_device_stats *sipa_eth_get_stats(struct net_device *dev)
{
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);

	return &sipa_eth->stats;
}

static const struct net_device_ops sipa_eth_ops = {
	.ndo_open = sipa_eth_open,
	.ndo_stop = sipa_eth_close,
	.ndo_start_xmit = sipa_eth_start_xmit,
	.ndo_get_stats = sipa_eth_get_stats,
#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
	.ndo_setup_tc = sipa_flowtable_setup_tc,
#endif
};

#if IS_ENABLED(CONFIG_SPRD_SIPA_NF_FLOWTABLE)
/* BSP sfp_mgr maps a genuine SIPA cellular netdev to VCP terminal 6.
 * Do not infer hardware endpoints from mutable names or Linux ifindex.
 * Descriptor lacks netid; admission remains blocked until mux semantics
 * are established. USB/WLAN need their own verified endpoint adapters.
 */
int sipa_eth_flowtable_port(struct net_device *dev, u8 *port)
{
	if (!dev || !port || dev->netdev_ops != &sipa_eth_ops)
		return -EOPNOTSUPP;
	*port = SIPA_TERM_VCP;
	return 0;
}
#endif

static void sipa_wwan_setup(struct net_device *dev)
{
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);

	ether_setup(dev);
	dev->flags &= ~(IFF_BROADCAST | IFF_MULTICAST);
	dev->flags |= IFF_NOARP;
	/* Preserve the 14-byte DMA framing while presenting cellular raw IP. */
	dev->type = ARPHRD_RAWIP;
	dev->netdev_ops = &sipa_eth_ops;
	dev->watchdog_timeo = HZ;
	dev->needs_free_netdev = true;
	eth_hw_addr_random(dev);
	sipa_eth->netdev = dev;
	sipa_eth->term_type = SIPA_TERM_VCP;
	sipa_eth->mac_h = true;
	sipa_eth->state = DEV_OFF;
	sipa_eth_dt_stats_init(&sipa_eth->dt_stats);
	netif_napi_add_weight(dev, &sipa_eth->napi, sipa_eth_rx_poll_handler,
			     SIPA_ETH_NAPI_WEIGHT);
	netif_carrier_off(dev);
}

static int sipa_wwan_newlink(void *ctxt, struct net_device *dev, u32 link_id,
			   struct netlink_ext_ack *extack)
{
	struct sipa_control *ctrl = ctxt;
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);
	int ret;

	if (link_id >= 8)
		return -EINVAL;
	/* Each CP netid has one receive queue; WWAN does not reject duplicates. */
	if (ctrl->wwan_links & BIT(link_id))
		return -EBUSY;
	sipa_eth->netid = link_id;
	ret = register_netdevice(dev);
	if (ret)
		return ret;
	ctrl->wwan_links |= BIT(link_id);
	sipa_eth_debugfs_mknod(root, sipa_eth);
	return 0;
}

static void sipa_wwan_dellink(void *ctxt, struct net_device *dev,
			    struct list_head *head)
{
	struct sipa_control *ctrl = ctxt;
	struct SIPA_ETH *sipa_eth = wwan_netdev_drvpriv(dev);

	debugfs_remove_recursive(sipa_eth->debugfs_dir);
	ctrl->wwan_links &= ~BIT(sipa_eth->netid);
	unregister_netdevice_queue(dev, head);
}

static const struct wwan_ops sipa_wwan_ops = {
	.priv_size = sizeof(struct SIPA_ETH),
	.setup = sipa_wwan_setup,
	.newlink = sipa_wwan_newlink,
	.dellink = sipa_wwan_dellink,
};

static int sipa_eth_debug_show(struct seq_file *m, void *v)
{
	struct SIPA_ETH *sipa_eth = (struct SIPA_ETH *)(m->private);
	struct sipa_eth_dtrans_stats *stats;

	if (!sipa_eth) {
		pr_err("invalid data, sipa_eth is NULL\n");
		return -EINVAL;
	}
	stats = &sipa_eth->dt_stats;

	seq_puts(m, "*************************************************\n");
	seq_printf(m, "DEVICE: %s, term_type %d, netid %d, state %s mac_h %d\n",
		   sipa_eth->netdev->name, sipa_eth->term_type, sipa_eth->netid,
		   sipa_eth->state == DEV_ON ? "UP" : "DOWN", sipa_eth->mac_h);
	seq_puts(m, "\nRX statistics:\n");
	seq_printf(m, "rx_sum=%u, rx_cnt=%u\n", stats->rx_sum, stats->rx_cnt);
	seq_printf(m, "rx_fail=%u\n", stats->rx_fail);

	seq_printf(m, "napi_scheduled=%d\n",
		   test_bit(NAPI_STATE_SCHED, &sipa_eth->napi.state));
	seq_puts(m, "\nTX statistics:\n");
	seq_printf(m, "tx_sum=%u, tx_cnt=%u\n", stats->tx_sum, stats->tx_cnt);
	seq_printf(m, "tx_fail=%u\n", stats->tx_fail);

	seq_puts(m, "*************************************************\n");

	return 0;
}

static int sipa_eth_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sipa_eth_debug_show, inode->i_private);
}

static const struct file_operations sipa_eth_debug_fops = {
	.open = sipa_eth_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int debugfs_gro_enable_get(void *data, u64 *val)
{
	*val = *(u64 *)data;
	return 0;
}

static int debugfs_gro_enable_set(void *data, u64 val)
{
	*(u64 *)data = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_gro_enable, debugfs_gro_enable_get,
			debugfs_gro_enable_set, "%llu\n");

static int sipa_eth_debugfs_mknod(void *root, void *data)
{
	struct SIPA_ETH *sipa_eth = (struct SIPA_ETH *)data;

	if (!sipa_eth)
		return -ENODEV;

	if (IS_ERR_OR_NULL(root))
		return -ENXIO;

	sipa_eth->debugfs_dir =
		debugfs_create_dir(sipa_eth->netdev->name, root);
	if (IS_ERR(sipa_eth->debugfs_dir))
		return PTR_ERR(sipa_eth->debugfs_dir);

	debugfs_create_file("stats", 0444, sipa_eth->debugfs_dir, data,
			    &sipa_eth_debug_fops);

	return 0;
}

static void sipa_eth_debugfs_init(void)
{
	root = debugfs_create_dir(SIPA_ETH_IFACE_PREF, NULL);
	if (IS_ERR(root))
		return;
	/* This setting is shared by every interface, so create it only once. */
	debugfs_create_file("gro_enable", 0600, root, &gro_enable,
			    &fops_gro_enable);
}

int sipa_eth_init(struct device *dev, struct sipa_control *ctrl)
{
	int ret;

	sipa_eth_debugfs_init();
	ret = wwan_register_ops(dev, &sipa_wwan_ops, ctrl, 0);
	if (ret)
		debugfs_remove_recursive(root);
	return ret;
}

void sipa_eth_exit(struct device *dev)
{
	wwan_unregister_ops(dev);
	debugfs_remove_recursive(root);
}

MODULE_AUTHOR("Wade.Shu");
MODULE_DESCRIPTION("Unisoc SIPA WWAN data links");
MODULE_LICENSE("GPL");
