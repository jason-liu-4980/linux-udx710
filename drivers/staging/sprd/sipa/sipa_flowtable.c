// SPDX-License-Identifier: GPL-2.0-only
/*
 * UDX710 AP nft flowtable adapter and SIPA table builder.
 * Wire format and hash follow the vendor SFP descriptor ABI, not vendor
 * netfilter hooks. No flow is admitted until all hardware semantics below
 * are established. The current revision has no qualified capability set.
 */
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/jhash.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/sipa.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/flow_offload.h>
#include <net/net_namespace.h>
#include <net/pkt_cls.h>
#include <uapi/linux/tc_act/tc_csum.h>

#include "sipa_flowtable.h"
#include "sipa_hal.h"

#define SIPA_FT_BUCKETS 2048
#define SIPA_FT_ENTRIES 4096
#define SIPA_FT_DESC_BYTES 96
#define SIPA_FT_BUCKET_BYTES 8
#define SIPA_FT_TABLE_BYTES                       \
	(SIPA_FT_BUCKETS * SIPA_FT_BUCKET_BYTES + \
	 SIPA_FT_ENTRIES * SIPA_FT_DESC_BYTES)

/* These are proof obligations, never user or DT feature override switches. */
#define SIPA_FT_CAP_INGRESS_ISOLATION BIT(0)
#define SIPA_FT_CAP_TCP_CONTROL_TRAP BIT(1)
#define SIPA_FT_CAP_TTL_CSUM BIT(2)
#define SIPA_FT_CAP_FRAGMENT_TRAP BIT(3)
#define SIPA_FT_CAP_DMA_DRAIN BIT(4)
#define SIPA_FT_CAP_DELETE_FAIL_CLOSED BIT(5)
#define SIPA_FT_CAP_POWER_LIFETIME BIT(6)
#define SIPA_FT_CAP_VCP_NETID BIT(7)

struct sipa_ft_tuple {
	__be32 dst[4];
	__be32 src[4];
	__be16 sport;
	__be16 dport;
	u8 family;
	u8 protocol;
} __packed;

struct sipa_ft_desc {
	struct sipa_ft_tuple original;
	struct sipa_ft_tuple translated;
	u8 dmac[ETH_ALEN];
	u8 smac[ETH_ALEN];
	u8 output_port;
	u8 flags;
	__be32 timestamp;
	__be16 reserved;
} __packed;

struct sipa_ft_bucket {
	u8 pad1;
	__be16 count;
	u8 pad2;
	__be32 address;
} __packed;

static_assert(sizeof(struct sipa_ft_tuple) == 38);
static_assert(sizeof(struct sipa_ft_desc) == SIPA_FT_DESC_BYTES);
static_assert(offsetof(struct sipa_ft_desc, output_port) == 88);
static_assert(offsetof(struct sipa_ft_desc, timestamp) == 90);
static_assert(sizeof(struct sipa_ft_bucket) == SIPA_FT_BUCKET_BYTES);

struct sipa_ft_table {
	void *cpu;
	dma_addr_t dma;
};

struct sipa_ft_block;
struct sipa_ft_entry {
	struct list_head list;
	struct sipa_ft_block *owner;
	unsigned long cookie;
	struct sipa_ft_desc desc;
	u16 bucket;
	int ingress_ifindex;
	u16 tcp_flags_mask;
};

struct sipa_ft_context {
	struct device *dma_dev;
	struct list_head entries;
	struct sipa_ft_table tables[2];
	u32 verified_caps;
	unsigned int count;
	unsigned int active;
	bool published;
	bool quarantine;
};

struct sipa_ft_block {
	struct net_device *dev;
	struct sipa_ft_context *ctx;
};

static DEFINE_MUTEX(sipa_ft_lock);
static LIST_HEAD(sipa_ft_block_list);
static struct sipa_ft_context *sipa_ft_context;

static int sipa_ft_error(struct netlink_ext_ack *extack, const char *msg)
{
	NL_SET_ERR_MSG_FMT(extack, "%s", msg);
	return -EOPNOTSUPP;
}

/* Same 36-byte key layout and jhash2 seed as vendor sfp_hash_conntrack(). */
static u16 sipa_ft_hash(const struct sipa_ft_tuple *tuple)
{
	struct {
		__be32 src[4];
		__be16 sport;
		u16 family;
		__be32 dst[4];
	} key;
	u32 words[9];
	u32 seed = ((u32)(__force u16)tuple->dport << 16) | tuple->protocol;

	static_assert(sizeof(key) == sizeof(words));
	memcpy(key.src, tuple->src, sizeof(key.src));
	key.sport = tuple->sport;
	key.family = tuple->family;
	memcpy(key.dst, tuple->dst, sizeof(key.dst));
	memcpy(words, &key, sizeof(words));
	return jhash2(words, ARRAY_SIZE(words), seed) & (SIPA_FT_BUCKETS - 1);
}

static int sipa_ft_alloc_tables(struct sipa_ft_context *ctx)
{
	int i;

	for (i = 0; i < 2; i++) {
		ctx->tables[i].cpu = dma_alloc_coherent(ctx->dma_dev,
							SIPA_FT_TABLE_BYTES,
							&ctx->tables[i].dma,
							GFP_KERNEL | GFP_DMA32);
		if (!ctx->tables[i].cpu)
			goto nomem;
		/* GFP_DMA32 alone does not constrain an IOMMU's DMA address. */
		if (ctx->tables[i].dma > U32_MAX ||
		    SIPA_FT_TABLE_BYTES - 1 > U32_MAX - ctx->tables[i].dma)
			goto range;
		memset(ctx->tables[i].cpu, 0, SIPA_FT_TABLE_BYTES);
	}
	return 0;
range:
	while (i >= 0) {
		dma_free_coherent(ctx->dma_dev, SIPA_FT_TABLE_BYTES,
				  ctx->tables[i].cpu, ctx->tables[i].dma);
		ctx->tables[i--].cpu = NULL;
	}
	return -ERANGE;
nomem:
	while (--i >= 0) {
		dma_free_coherent(ctx->dma_dev, SIPA_FT_TABLE_BYTES,
				  ctx->tables[i].cpu, ctx->tables[i].dma);
		ctx->tables[i].cpu = NULL;
	}
	return -ENOMEM;
}

static bool sipa_ft_skip(const struct sipa_ft_entry *entry,
			 const struct sipa_ft_entry *replace,
			 const struct sipa_ft_block *flush)
{
	return entry == replace || (flush && entry->owner == flush);
}

/* Build the inactive coherent table from a candidate software state. */
static int sipa_ft_build(struct sipa_ft_context *ctx, unsigned int id,
			 struct sipa_ft_entry *add,
			 struct sipa_ft_entry *replace,
			 struct sipa_ft_block *flush)
{
	struct sipa_ft_table *table = &ctx->tables[id];
	struct sipa_ft_bucket *buckets = table->cpu;
	struct sipa_ft_desc *descs =
		table->cpu + SIPA_FT_BUCKETS * sizeof(*buckets);
	struct sipa_ft_entry *entry;
	u16 *counts, *cursor;
	u32 used = 0;
	unsigned int i;
	int ret = 0;

	counts = kcalloc(SIPA_FT_BUCKETS * 2, sizeof(*counts), GFP_KERNEL);
	if (!counts)
		return -ENOMEM;
	cursor = counts + SIPA_FT_BUCKETS;
	list_for_each_entry(entry, &ctx->entries, list) {
		if (!sipa_ft_skip(entry, replace, flush))
			counts[entry->bucket]++;
	}
	if (add)
		counts[add->bucket]++;
	memset(table->cpu, 0, SIPA_FT_TABLE_BYTES);
	for (i = 0; i < SIPA_FT_BUCKETS; i++) {
		dma_addr_t address;

		if (used + counts[i] > SIPA_FT_ENTRIES) {
			ret = -ENOSPC;
			goto out;
		}
		cursor[i] = used;
		if (counts[i]) {
			address = table->dma +
				  SIPA_FT_BUCKETS * sizeof(*buckets) +
				  used * sizeof(*descs);
			if (address > U32_MAX) {
				ret = -ERANGE;
				goto out;
			}
			buckets[i].address = cpu_to_be32(address);
			buckets[i].count = cpu_to_be16(counts[i]);
		}
		used += counts[i];
	}
	list_for_each_entry(entry, &ctx->entries, list) {
		if (!sipa_ft_skip(entry, replace, flush))
			descs[cursor[entry->bucket]++] = entry->desc;
	}
	if (add)
		descs[cursor[add->bucket]++] = add->desc;
	dma_wmb();
out:
	kfree(counts);
	return ret;
}

static int sipa_ft_commit(struct sipa_ft_context *ctx,
			  struct sipa_ft_entry *add,
			  struct sipa_ft_entry *replace,
			  struct sipa_ft_block *flush)
{
	struct sipa_hash_table next, expected = {};
	unsigned int id = ctx->published ? ctx->active ^ 1 : 0;
	bool uncertain = false;
	int ret;

	if (ctx->quarantine)
		return -EIO;
	/* Both drain and failure containment must be proven before publication. */
	if ((ctx->verified_caps &
	     (SIPA_FT_CAP_DMA_DRAIN | SIPA_FT_CAP_DELETE_FAIL_CLOSED |
	      SIPA_FT_CAP_POWER_LIFETIME)) !=
	    (SIPA_FT_CAP_DMA_DRAIN | SIPA_FT_CAP_DELETE_FAIL_CLOSED |
	     SIPA_FT_CAP_POWER_LIFETIME))
		return -EOPNOTSUPP;
	if (!ctx->tables[0].cpu) {
		ret = sipa_ft_alloc_tables(ctx);
		if (ret)
			return ret;
	}
	ret = sipa_ft_build(ctx, id, add, replace, flush);
	if (ret)
		return ret;
	next.depth = SIPA_FT_BUCKETS;
	next.tbl_phy_addr = ctx->tables[id].dma;
	if (ctx->published) {
		expected.depth = SIPA_FT_BUCKETS;
		expected.tbl_phy_addr = ctx->tables[ctx->active].dma;
	}
	ret = sipa_hal_ft_transaction(&next, &expected, &uncertain);
	if (ret) {
		/* Never free or reuse either candidate after an uncertain switch. */
		ctx->quarantine |= uncertain || ctx->published;
		return ret;
	}
	ctx->published = true;
	ctx->active = id;
	return 0;
}

static int sipa_ft_parse_match(struct net_device *dev,
			       struct flow_cls_offload *f,
			       struct sipa_ft_entry *entry)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct netlink_ext_ack *ack = f->common.extack;
	struct flow_match_meta meta;
	struct flow_match_control control;
	struct flow_match_basic basic;
	struct flow_match_ports ports;
	u64 required = BIT_ULL(FLOW_DISSECTOR_KEY_META) |
		       BIT_ULL(FLOW_DISSECTOR_KEY_CONTROL) |
		       BIT_ULL(FLOW_DISSECTOR_KEY_BASIC) |
		       BIT_ULL(FLOW_DISSECTOR_KEY_PORTS);
	u64 used;

	if (!rule || f->common.chain_index || f->classid)
		return sipa_ft_error(
			ack, "SIPA accepts only exact nft flowtable rules");
	used = rule->match.dissector->used_keys;
	if ((used & required) != required)
		return sipa_ft_error(
			ack,
			"SIPA requires ingress, protocol and complete ports");
	flow_rule_match_meta(rule, &meta);
	if (meta.mask->ingress_ifindex != -1 || meta.mask->ingress_iftype ||
	    meta.mask->l2_miss || meta.key->ingress_ifindex != dev->ifindex)
		return sipa_ft_error(
			ack,
			"SIPA does not support wildcard or foreign ingress");
	entry->ingress_ifindex = meta.key->ingress_ifindex;
	flow_rule_match_control(rule, &control);
	if (control.mask->addr_type != U16_MAX || control.mask->thoff ||
	    control.mask->flags)
		return sipa_ft_error(
			ack,
			"SIPA cannot represent control masks or fragments");
	flow_rule_match_basic(rule, &basic);
	if (basic.mask->n_proto != cpu_to_be16(U16_MAX) ||
	    basic.mask->ip_proto != U8_MAX || basic.mask->padding ||
	    (basic.key->ip_proto != IPPROTO_TCP &&
	     basic.key->ip_proto != IPPROTO_UDP))
		return sipa_ft_error(
			ack, "SIPA supports exact IPv4/IPv6 TCP/UDP only");
	entry->desc.original.protocol = basic.key->ip_proto;
	if (basic.key->n_proto == htons(ETH_P_IP) &&
	    control.key->addr_type == FLOW_DISSECTOR_KEY_IPV4_ADDRS) {
		struct flow_match_ipv4_addrs ipv4;

		required |= BIT_ULL(FLOW_DISSECTOR_KEY_IPV4_ADDRS);
		if (!(used & BIT_ULL(FLOW_DISSECTOR_KEY_IPV4_ADDRS)))
			return -EOPNOTSUPP;
		flow_rule_match_ipv4_addrs(rule, &ipv4);
		if (ipv4.mask->src != cpu_to_be32(U32_MAX) ||
		    ipv4.mask->dst != cpu_to_be32(U32_MAX))
			return sipa_ft_error(
				ack, "SIPA requires exact IPv4 addresses");
		entry->desc.original.family = AF_INET;
		entry->desc.original.src[0] = ipv4.key->src;
		entry->desc.original.dst[0] = ipv4.key->dst;
	} else if (basic.key->n_proto == htons(ETH_P_IPV6) &&
		   control.key->addr_type == FLOW_DISSECTOR_KEY_IPV6_ADDRS) {
		struct flow_match_ipv6_addrs ipv6;

		required |= BIT_ULL(FLOW_DISSECTOR_KEY_IPV6_ADDRS);
		if (!(used & BIT_ULL(FLOW_DISSECTOR_KEY_IPV6_ADDRS)))
			return -EOPNOTSUPP;
		flow_rule_match_ipv6_addrs(rule, &ipv6);
		if (memchr_inv(&ipv6.mask->src, 0xff, sizeof(ipv6.mask->src)) ||
		    memchr_inv(&ipv6.mask->dst, 0xff, sizeof(ipv6.mask->dst)))
			return sipa_ft_error(
				ack, "SIPA requires exact IPv6 addresses");
		entry->desc.original.family = AF_INET6;
		memcpy(entry->desc.original.src, &ipv6.key->src,
		       sizeof(ipv6.key->src));
		memcpy(entry->desc.original.dst, &ipv6.key->dst,
		       sizeof(ipv6.key->dst));
	} else {
		return sipa_ft_error(
			ack, "SIPA address family and EtherType must agree");
	}
	flow_rule_match_ports(rule, &ports);
	if (ports.mask->src != cpu_to_be16(U16_MAX) ||
	    ports.mask->dst != cpu_to_be16(U16_MAX))
		return sipa_ft_error(
			ack,
			"SIPA requires exact source and destination ports");
	entry->desc.original.sport = ports.key->src;
	entry->desc.original.dport = ports.key->dst;
	if (entry->desc.original.protocol == IPPROTO_TCP) {
		struct flow_match_tcp tcp;

		required |= BIT_ULL(FLOW_DISSECTOR_KEY_TCP);
		if (!(used & BIT_ULL(FLOW_DISSECTOR_KEY_TCP)))
			return sipa_ft_error(
				ack,
				"SIPA needs the nft TCP FIN/RST exclusion");
		flow_rule_match_tcp(rule, &tcp);
		if (tcp.key->flags || tcp.mask->flags != htons(0x05))
			return sipa_ft_error(
				ack,
				"SIPA cannot represent these TCP flag masks");
		entry->tcp_flags_mask = ntohs(tcp.mask->flags);
	}
	if (used != required)
		return sipa_ft_error(
			ack,
			"SIPA cannot represent VLAN, tunnel or extra match keys");
	entry->desc.translated = entry->desc.original;
	return 0;
}

/* Apply native-endian tc pedit values bytewise, checking every changed byte. */
static int sipa_ft_mangle(const struct flow_action_entry *act,
			  struct sipa_ft_desc *desc, u8 *eth_seen)
{
	const u8 *mask = (const u8 *)&act->mangle.mask;
	const u8 *value = (const u8 *)&act->mangle.val;
	u8 *target;
	u32 offset = act->mangle.offset;
	unsigned int size, i;

	if (act->mangle.val & act->mangle.mask)
		return -EOPNOTSUPP;
	switch (act->mangle.htype) {
	case FLOW_ACT_MANGLE_HDR_TYPE_ETH:
		target = desc->dmac;
		size = ETH_ALEN * 2;
		break;
	case FLOW_ACT_MANGLE_HDR_TYPE_IP4:
		if (desc->original.family != AF_INET || act->mangle.mask ||
		    (offset != offsetof(struct iphdr, saddr) &&
		     offset != offsetof(struct iphdr, daddr)))
			return -EOPNOTSUPP;
		target = offset == offsetof(struct iphdr, saddr) ?
				 (u8 *)desc->translated.src :
				 (u8 *)desc->translated.dst;
		offset = 0;
		size = 4;
		break;
	case FLOW_ACT_MANGLE_HDR_TYPE_IP6:
		if (desc->original.family != AF_INET6 || act->mangle.mask ||
		    offset < offsetof(struct ipv6hdr, saddr) ||
		    offset > offsetof(struct ipv6hdr, daddr) + 12 ||
		    (offset & 3))
			return -EOPNOTSUPP;
		if (offset < offsetof(struct ipv6hdr, daddr)) {
			target = (u8 *)desc->translated.src;
			offset -= offsetof(struct ipv6hdr, saddr);
		} else {
			target = (u8 *)desc->translated.dst;
			offset -= offsetof(struct ipv6hdr, daddr);
		}
		size = 16;
		break;
	case FLOW_ACT_MANGLE_HDR_TYPE_TCP:
	case FLOW_ACT_MANGLE_HDR_TYPE_UDP:
		if (offset ||
		    (act->mangle.htype == FLOW_ACT_MANGLE_HDR_TYPE_TCP &&
		     desc->original.protocol != IPPROTO_TCP) ||
		    (act->mangle.htype == FLOW_ACT_MANGLE_HDR_TYPE_UDP &&
		     desc->original.protocol != IPPROTO_UDP) ||
		    (act->mangle.mask &&
		     act->mangle.mask != (__force u32)htons(0xffff) &&
		     act->mangle.mask != htonl(0xffff0000) &&
		     act->mangle.mask != htonl(0x0000ffff)))
			return -EOPNOTSUPP;
		target = (u8 *)&desc->translated.sport;
		size = 4;
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (offset > size || size - offset < 4)
		return -EOPNOTSUPP;
	for (i = 0; i < 4; i++) {
		if (mask[i] != 0 && mask[i] != 0xff)
			return -EOPNOTSUPP;
		if (!mask[i]) {
			target[offset + i] = value[i];
			if (act->mangle.htype == FLOW_ACT_MANGLE_HDR_TYPE_ETH)
				eth_seen[offset + i] = 1;
		}
	}
	return 0;
}

static int sipa_ft_parse_actions(struct net_device *dev,
				 struct flow_cls_offload *f,
				 struct sipa_ft_entry *entry)
{
	const struct flow_action_entry *act;
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	u8 eth_seen[ETH_ALEN * 2] = {};
	bool redirected = false;
	unsigned int csum = 0, required_csum;
	int i, ret;

	flow_action_for_each(i, act, &rule->action) {
		if (redirected)
			return sipa_ft_error(
				f->common.extack,
				"SIPA requires redirect to be last");
		switch (act->id) {
		case FLOW_ACTION_MANGLE:
			ret = sipa_ft_mangle(act, &entry->desc, eth_seen);
			if (ret)
				return sipa_ft_error(
					f->common.extack,
					"SIPA cannot encode this mangle");
			break;
		case FLOW_ACTION_CSUM:
			if (csum)
				return -EOPNOTSUPP;
			csum = act->csum_flags;
			break;
		case FLOW_ACTION_REDIRECT:
			if (!act->dev ||
			    !net_eq(dev_net(dev), dev_net(act->dev)))
				return -EOPNOTSUPP;
			ret = sipa_eth_flowtable_port(act->dev,
						      &entry->desc.output_port);
			if (ret)
				return sipa_ft_error(
					f->common.extack,
					"SIPA output port is not a verified endpoint");
			redirected = true;
			break;
		default:
			return sipa_ft_error(f->common.extack,
					     "SIPA action is unsupported");
		}
	}
	if (!redirected || memchr_inv(eth_seen, 1, sizeof(eth_seen)) ||
	    !is_valid_ether_addr(entry->desc.dmac) ||
	    !is_valid_ether_addr(entry->desc.smac))
		return sipa_ft_error(
			f->common.extack,
			"SIPA requires complete unicast MAC rewrite and redirect");
	required_csum = entry->desc.original.protocol == IPPROTO_TCP ?
				TCA_CSUM_UPDATE_FLAG_TCP :
				TCA_CSUM_UPDATE_FLAG_UDP;
	if (entry->desc.original.family == AF_INET)
		required_csum |= TCA_CSUM_UPDATE_FLAG_IPV4HDR;
	/* IPv6 nf_flowtable may omit explicit CSUM even for address/port NAT. */
	if ((csum && csum != required_csum) ||
	    (entry->desc.original.family == AF_INET &&
	     memcmp(&entry->desc.original, &entry->desc.translated,
		    sizeof(entry->desc.original)) &&
	     csum != required_csum))
		return sipa_ft_error(
			f->common.extack,
			"SIPA checksum action flags are unsupported");
	entry->bucket = sipa_ft_hash(&entry->desc.original);
	return 0;
}

static int sipa_ft_admit(struct sipa_ft_context *ctx,
			 struct sipa_ft_entry *entry,
			 struct netlink_ext_ack *ack)
{
	if (!(ctx->verified_caps & SIPA_FT_CAP_INGRESS_ISOLATION))
		return sipa_ft_error(
			ack,
			"SIPA descriptor has no ingress match; isolation is unproven");
	if (entry->tcp_flags_mask &&
	    !(ctx->verified_caps & SIPA_FT_CAP_TCP_CONTROL_TRAP))
		return sipa_ft_error(
			ack,
			"SIPA cannot prove TCP FIN/RST packets return to nft");
	if (!(ctx->verified_caps & SIPA_FT_CAP_TTL_CSUM))
		return sipa_ft_error(
			ack,
			"SIPA TTL/hop-limit and NAT checksum behavior is unproven");
	if (!(ctx->verified_caps & SIPA_FT_CAP_FRAGMENT_TRAP))
		return sipa_ft_error(
			ack,
			"SIPA fragment/extension-header fallback is unproven");
	if (!(ctx->verified_caps & SIPA_FT_CAP_VCP_NETID))
		return sipa_ft_error(
			ack,
			"SIPA VCP output does not encode the cellular netid");
	if ((ctx->verified_caps &
	     (SIPA_FT_CAP_DMA_DRAIN | SIPA_FT_CAP_DELETE_FAIL_CLOSED |
	      SIPA_FT_CAP_POWER_LIFETIME)) !=
	    (SIPA_FT_CAP_DMA_DRAIN | SIPA_FT_CAP_DELETE_FAIL_CLOSED |
	     SIPA_FT_CAP_POWER_LIFETIME))
		return sipa_ft_error(
			ack,
			"SIPA table retirement, delete failure and power lifetime are unproven");
	return 0;
}

static struct sipa_ft_entry *sipa_ft_lookup(struct sipa_ft_context *ctx,
					    struct sipa_ft_block *owner,
					    unsigned long cookie)
{
	struct sipa_ft_entry *entry;

	list_for_each_entry(entry, &ctx->entries, list)
		if (entry->owner == owner && entry->cookie == cookie)
			return entry;
	return NULL;
}

static int sipa_ft_replace(struct sipa_ft_block *block,
			   struct flow_cls_offload *f)
{
	struct sipa_ft_context *ctx = block->ctx;
	struct sipa_ft_entry *entry, *old, *other;
	int ret;

	entry = kzalloc_obj(*entry, GFP_KERNEL);
	if (!entry)
		return -ENOMEM;
	entry->owner = block;
	entry->cookie = f->cookie;
	ret = sipa_ft_parse_match(block->dev, f, entry);
	if (ret)
		goto out;
	ret = sipa_ft_parse_actions(block->dev, f, entry);
	if (ret)
		goto out;
	ret = sipa_ft_admit(ctx, entry, f->common.extack);
	if (ret)
		goto out;
	old = sipa_ft_lookup(ctx, block, f->cookie);
	if (!old && ctx->count >= SIPA_FT_ENTRIES) {
		ret = -ENOSPC;
		goto out;
	}
	list_for_each_entry(other, &ctx->entries, list) {
		if (other != old &&
		    !memcmp(&other->desc.original, &entry->desc.original,
			    sizeof(entry->desc.original))) {
			ret = -EEXIST;
			goto out;
		}
	}
	ret = sipa_ft_commit(ctx, entry, old, NULL);
	if (ret)
		goto out;
	if (old) {
		list_replace(&old->list, &entry->list);
		kfree(old);
	} else {
		list_add_tail(&entry->list, &ctx->entries);
		ctx->count++;
	}
	return 0;
out:
	kfree(entry);
	return ret;
}

static int sipa_ft_destroy(struct sipa_ft_block *block, unsigned long cookie)
{
	struct sipa_ft_context *ctx = block->ctx;
	struct sipa_ft_entry *entry = sipa_ft_lookup(ctx, block, cookie);
	int ret;

	if (!entry)
		return -ENOENT;
	ret = sipa_ft_commit(ctx, NULL, entry, NULL);
	if (ret)
		return ret;
	list_del(&entry->list);
	ctx->count--;
	kfree(entry);
	return 0;
}

static int sipa_ft_block_cb(enum tc_setup_type type, void *type_data,
			    void *priv)
{
	struct sipa_ft_block *block = priv;
	struct flow_cls_offload *f = type_data;
	int ret;

	if (type != TC_SETUP_CLSFLOWER)
		return -EOPNOTSUPP;
	mutex_lock(&sipa_ft_lock);
	if (block->ctx->quarantine) {
		ret = -EIO;
		goto out;
	}
	switch (f->command) {
	case FLOW_CLS_REPLACE:
		ret = sipa_ft_replace(block, f);
		break;
	case FLOW_CLS_DESTROY:
		ret = sipa_ft_destroy(block, f->cookie);
		break;
	case FLOW_CLS_STATS:
		if (!sipa_ft_lookup(block->ctx, block, f->cookie)) {
			ret = -ENOENT;
			break;
		}
		/* No packet/byte counters; timestamp tick/epoch are not established. */
		ret = sipa_ft_error(
			f->common.extack,
			"SIPA has no counters; timestamp epoch/read consistency is unqualified");
		break;
	default:
		ret = -EOPNOTSUPP;
	}
out:
	mutex_unlock(&sipa_ft_lock);
	return ret;
}

static void sipa_ft_block_release(void *priv)
{
	struct sipa_ft_block *block = priv;

	dev_put(block->dev);
	kfree(block);
}

static int sipa_ft_flush(struct sipa_ft_block *block)
{
	struct sipa_ft_entry *entry, *tmp;
	bool found = false;
	int ret;

	list_for_each_entry(entry, &block->ctx->entries, list)
		found |= entry->owner == block;
	if (!found)
		return 0;
	ret = sipa_ft_commit(block->ctx, NULL, NULL, block);
	if (ret)
		return ret;
	list_for_each_entry_safe(entry, tmp, &block->ctx->entries, list) {
		if (entry->owner != block)
			continue;
		list_del(&entry->list);
		block->ctx->count--;
		kfree(entry);
	}
	return 0;
}

int sipa_flowtable_setup_tc(struct net_device *dev, enum tc_setup_type type,
			    void *type_data)
{
	struct flow_block_offload *f = type_data;
	struct flow_block_cb *cb;
	struct sipa_ft_block *block;
	struct device *dma_dev;
	int ret = 0;

	/* Register only through nft FT; ordinary TC_SETUP_BLOCK is unsupported. */
	if (type != TC_SETUP_FT ||
	    f->binder_type != FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS ||
	    !net_eq(dev_net(dev), &init_net))
		return -EOPNOTSUPP;
	mutex_lock(&sipa_ft_lock);
	f->driver_block_list = &sipa_ft_block_list;
	cb = flow_block_cb_lookup(f->block, sipa_ft_block_cb, dev);
	switch (f->command) {
	case FLOW_BLOCK_BIND:
		if (cb) {
			flow_block_cb_incref(cb);
			break;
		}
		dma_dev = sipa_get_pdev();
		if (!dma_dev) {
			ret = -EPROBE_DEFER;
			break;
		}
		if (!sipa_ft_context) {
			sipa_ft_context =
				kzalloc_obj(*sipa_ft_context, GFP_KERNEL);
			if (!sipa_ft_context) {
				ret = -ENOMEM;
				break;
			}
			sipa_ft_context->dma_dev = get_device(dma_dev);
			INIT_LIST_HEAD(&sipa_ft_context->entries);
			/*
			 * FIN/RST global punt is documented and checked at every switch.
			 * Other contracts are unqualified; no DT/module-param bypass.
			 */
			sipa_ft_context->verified_caps =
				SIPA_FT_CAP_TCP_CONTROL_TRAP;
		}
		if (sipa_ft_context->dma_dev != dma_dev ||
		    sipa_ft_context->quarantine) {
			ret = -EBUSY;
			break;
		}
		block = kzalloc_obj(*block, GFP_KERNEL);
		if (!block) {
			ret = -ENOMEM;
			break;
		}
		block->dev = dev;
		block->ctx = sipa_ft_context;
		dev_hold(dev);
		cb = flow_block_cb_alloc(sipa_ft_block_cb, dev, block,
					 sipa_ft_block_release);
		if (IS_ERR(cb)) {
			ret = PTR_ERR(cb);
			sipa_ft_block_release(block);
			break;
		}
		flow_block_cb_incref(cb);
		flow_block_cb_add(cb, f);
		list_add_tail(&cb->driver_list, &sipa_ft_block_list);
		break;
	case FLOW_BLOCK_UNBIND:
		if (!cb) {
			ret = -ENOENT;
			break;
		}
		if (cb->refcnt > 1) {
			flow_block_cb_decref(cb);
			break;
		}
		ret = sipa_ft_flush(cb->cb_priv);
		if (ret)
			break;
		if (!flow_block_cb_decref(cb)) {
			flow_block_cb_remove(cb, f);
			list_del(&cb->driver_list);
		}
		break;
	default:
		ret = -EOPNOTSUPP;
	}
	mutex_unlock(&sipa_ft_lock);
	return ret;
}
