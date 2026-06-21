// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL9602C "Luna" hardware L3/L4 forwarding + NAPT engine.
 *
 * Indirect table-access plumbing + NAPT flow programming for the SoC switch
 * core's L3/L4 accelerator. Established connections handed down by the kernel
 * flow-offload path are written into the hardware NAPT tables so they are
 * NAT'd/forwarded by the switch instead of the CPU.
 *
 * Clean-room: the register/table behaviour is hardware fact; all expression
 * (names, packing helpers, the access sequence) is original. Endian-safe: the
 * packed entry words are built with explicit shift/mask on a local u32 array,
 * never struct overlays, so the same code is correct on big-endian MIPS now and
 * little-endian ARM later.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "rtl9602c_l34.h"

#define L34_EXE_POLL_US		2000	/* engine clears EXE well under this */

static inline u32 l34_rd(struct rtl9602c_l34 *l, u32 off)
{
	return readl(l->sw + off);
}

static inline void l34_wr(struct rtl9602c_l34 *l, u32 off, u32 val)
{
	writel(val, l->sw + off);
}

/*
 * Pack/unpack a field that may straddle 32-bit word boundaries within the
 * entry's word array. @lsp is the bit position from bit 0 of word[0].
 */
static void l34_field_set(u32 *w, unsigned int lsp, unsigned int width, u32 val)
{
	unsigned int word = lsp / 32, bit = lsp % 32, take;

	val &= (width >= 32) ? ~0u : ((1u << width) - 1);
	while (width) {
		take = min(width, 32 - bit);
		w[word] &= ~((((take >= 32) ? ~0u : ((1u << take) - 1))) << bit);
		w[word] |= (val & ((take >= 32) ? ~0u : ((1u << take) - 1))) << bit;
		val >>= take;
		width -= take;
		word++;
		bit = 0;
	}
}

/* Inverse of l34_field_set: extract a (possibly word-straddling) field. */
static u32 l34_field_get(const u32 *w, unsigned int lsp, unsigned int width)
{
	unsigned int word = lsp / 32, bit = lsp % 32, take, got = 0;
	u32 val = 0;

	while (width) {
		take = min(width, 32 - bit);
		val |= ((w[word] >> bit) & ((take >= 32) ? ~0u : ((1u << take) - 1)))
		       << got;
		got += take;
		width -= take;
		word++;
		bit = 0;
	}
	return val;
}

/* Issue one indirect table op. The data bank is significance-ordered: WRDATA[i]
 * (and RDDATA[i]) carry entry bits [32*i+31 : 32*i], so w[i] maps 1:1 onto data
 * slot i (w[0] = least-significant word). */
static int l34_tbl_op(struct rtl9602c_l34 *l, enum l34_tbl type, u16 idx,
		      u32 *w, unsigned int n, bool write)
{
	u32 cmd, exe = write ? L34_CMD_WR_EXE : L34_CMD_RD_EXE;
	unsigned int i, t;

	if (write)
		for (i = 0; i < n; i++)
			l34_wr(l, L34_WDATA + 4 * i, w[i]);

	cmd = exe | ((type & L34_CMD_TYPE_MASK) << L34_CMD_TYPE_SHIFT) |
	      (idx & L34_CMD_IDX_MASK);
	l34_wr(l, L34_CMD, cmd);

	for (t = 0; t < L34_EXE_POLL_US; t++) {
		if (!(l34_rd(l, L34_CMD) & exe))
			break;
		udelay(1);
	}
	if (l34_rd(l, L34_CMD) & exe)
		return -ETIMEDOUT;

	if (!write)
		for (i = 0; i < n; i++)
			w[i] = l34_rd(l, L34_RDATA + 4 * i);
	return 0;
}

static int l34_tbl_write(struct rtl9602c_l34 *l, enum l34_tbl type, u16 idx,
			 u32 *w, unsigned int n)
{
	return l34_tbl_op(l, type, idx, w, n, true);
}

static int l34_tbl_read(struct rtl9602c_l34 *l, enum l34_tbl type, u16 idx,
			u32 *w, unsigned int n)
{
	return l34_tbl_op(l, type, idx, w, n, false);
}

/*
 * Enable the L34 NAT engine. DEFERRED from init: writing the engine's NAT-mode /
 * lookup-mode / master-enable bits BEFORE the OLT has provisioned the GPON GEM
 * datapath corrupts the upstream-GEM setup on a cold range (the US data GEM
 * stops egressing -> no DHCP -> no WAN). So it is enabled lazily the first time
 * the offload is programmed, once the datapath is up. Caller holds l->lock.
 */
static void l34_engine_on(struct rtl9602c_l34 *l)
{
	u32 v;

	l34_wr(l, L34_CLR, 0xffff);		/* reset the NAT table types we use */
	usleep_range(50, 100);

	v = l34_rd(l, L34_SWTCR0);
	v &= ~(0x3u << L34_SWTCR0_NATMODE_SH);
	v |=  (0x3u << L34_SWTCR0_NATMODE_SH);		/* L3 + L4 NAT enable */
	v &= ~(0x3u << L34_SWTCR0_LOOKUP_SH);
	v |=  (0x2u << L34_SWTCR0_LOOKUP_SH);		/* MAC-base lookup */
	l34_wr(l, L34_SWTCR0, v);

	l34_wr(l, L34_GLB_CFG, l34_rd(l, L34_GLB_CFG) | BIT(0));	/* master enable */

	l->engine_on = true;
}

int rtl9602c_l34_init(struct rtl9602c_l34 *l, void __iomem *sw)
{
	if (!sw)
		return -EINVAL;
	l->sw = sw;
	mutex_init(&l->lock);
	l->ready = true;	/* table-access ready; the engine is enabled lazily */
	return 0;
}

/*
 * NAPT bucket hashes (clean-room: the arithmetic is re-expressed from observed
 * behaviour). Each folds its key to a 10-bit bucket (0..1023); the 4096-entry
 * NAPT/NAPTR tables are 4-way, so an entry index is (bucket << 2) + way, with
 * way 0..3. The is_tcp argument is a 1-bit flag (1=TCP, 0=UDP) — NOT the IP
 * protocol number; only its bit 0 feeds the hash.
 */
static u16 l34_hash_out(bool is_tcp, u32 sip, u16 sport, u32 dip, u16 dport)
{
	/* low 16 bits of both addresses + both ports, summed then folded 18->10 */
	u32 lo   = (sip & 0xffff) + (dip & 0xffff) + sport + dport;
	u32 fold = (lo & 0x3ff) + ((lo >> 10) & 0xff);

	/* the address upper halves and the TCP/UDP selector, mixed in by XOR */
	u32 mix  = ((sip >> 16) & 0x3ff) ^ ((dip >> 16) & 0x3ff);

	mix ^= ((sip >> 26) & 0x3f) + ((u32)(is_tcp & 1) << 9);
	mix ^= ((dip >> 26) & 0x3f) << 4;

	return (fold ^ mix) & 0x3ff;
}

/* Inbound NAPTR bucket: keyed only on the post-NAT (WAN-side) dest IP + port. */
static u16 l34_hash_in(bool is_tcp, u32 dip, u16 dport)
{
	u32 lo   = dport + (dip & 0xffff);
	u32 fold = (lo & 0x3ff) + ((lo >> 10) & 0x7f);
	u32 mix  = ((dip >> 16) & 0x3ff)
		 ^ (((dip >> 26) & 0x3f) + ((u32)(is_tcp & 1) << 9));

	return (fold ^ mix) & 0x3ff;
}

/* Scan the 4 ways of a hash bucket for the first slot whose VALID field is 0;
 * returns the entry index, a negative table error, or -ENOSPC if full. */
static int l34_free_way(struct rtl9602c_l34 *l, enum l34_tbl type, u16 bucket,
			unsigned int words, unsigned int valid_lsp,
			unsigned int valid_w)
{
	u32 probe[L34_WORDS_NAPTR_IN];	/* sized to the widest entry */
	unsigned int way;
	int ret;

	for (way = 0; way < L34_NAPT_WAYS; way++) {
		ret = l34_tbl_read(l, type, (bucket << 2) + way, probe, words);
		if (ret)
			return ret;
		if (!l34_field_get(probe, valid_lsp, valid_w))
			return (bucket << 2) + way;
	}
	return -ENOSPC;
}

/*
 * NAPT flow programming. A 4-way hashed pair: the outbound slot
 * (L34_TBL_NAPT_OUT, keyed by l34_hash_out() of the original tuple) points at an
 * inbound rewrite entry (L34_TBL_NAPTR_IN, keyed by l34_hash_in() of the
 * translated WAN tuple) holding the internal host addr/port + post-NAT port.
 * The rewrite's WAN source IP and egress next-hop come from the EXTIP slot named
 * by f->egress_netif, programmed once at WAN bring-up. Written full-cone
 * (remHash unused) so the return path matches on the WAN addr/port alone.
 */
int rtl9602c_l34_flow_add(struct rtl9602c_l34 *l, struct l34_flow *f)
{
	u32 naptr[L34_WORDS_NAPTR_IN] = { 0 };
	u32 napt[L34_WORDS_NAPT_OUT] = { 0 };
	int naptr_idx, napt_idx, ret;
	u16 out_bucket, in_bucket;
	bool is_tcp;

	if (!l->ready)
		return -ENODEV;
	if (f->l4proto != IPPROTO_TCP && f->l4proto != IPPROTO_UDP)
		return -EOPNOTSUPP;
	is_tcp = (f->l4proto == IPPROTO_TCP);

	out_bucket = l34_hash_out(is_tcp, f->orig_sip, f->orig_sport,
				  f->orig_dip, f->orig_dport);
	in_bucket  = l34_hash_in(is_tcp, f->nat_sip, f->nat_sport);

	mutex_lock(&l->lock);
	if (!l->engine_on)
		l34_engine_on(l);

	naptr_idx = l34_free_way(l, L34_TBL_NAPTR_IN, in_bucket, L34_WORDS_NAPTR_IN,
				 L34_NAPTR_VALID_LSP, L34_NAPTR_VALID_W);
	if (naptr_idx < 0) {
		ret = naptr_idx;
		goto out;
	}
	napt_idx = l34_free_way(l, L34_TBL_NAPT_OUT, out_bucket, L34_WORDS_NAPT_OUT,
				L34_NAPT_VALID_LSP, L34_NAPT_VALID_W);
	if (napt_idx < 0) {
		ret = napt_idx;
		goto out;
	}

	l34_field_set(naptr, L34_NAPTR_INTIP_LSP,    L34_NAPTR_INTIP_W,    f->orig_sip);
	l34_field_set(naptr, L34_NAPTR_INTPORT_LSP,  L34_NAPTR_INTPORT_W,  f->orig_sport);
	l34_field_set(naptr, L34_NAPTR_EXTIPIDX_LSP, L34_NAPTR_EXTIPIDX_W, f->egress_netif);
	l34_field_set(naptr, L34_NAPTR_EXTPORT_LSP,  L34_NAPTR_EXTPORT_W,  f->nat_sport);
	l34_field_set(naptr, L34_NAPTR_TCP_LSP,      L34_NAPTR_TCP_W,      is_tcp);
	l34_field_set(naptr, L34_NAPTR_VALID_LSP,    L34_NAPTR_VALID_W,    L34_NAPTR_TYPE_FULLCONE);
	ret = l34_tbl_write(l, L34_TBL_NAPTR_IN, naptr_idx, naptr, L34_WORDS_NAPTR_IN);
	if (ret)
		goto out;

	l34_field_set(napt, L34_NAPT_HASHIN_IDX_LSP, L34_NAPT_HASHIN_IDX_W, naptr_idx);
	l34_field_set(napt, L34_NAPT_VALID_LSP,      L34_NAPT_VALID_W,      1);
	ret = l34_tbl_write(l, L34_TBL_NAPT_OUT, napt_idx, napt, L34_WORDS_NAPT_OUT);
	if (ret) {
		memset(naptr, 0, sizeof(naptr));	/* roll back the rewrite entry */
		l34_tbl_write(l, L34_TBL_NAPTR_IN, naptr_idx, naptr, L34_WORDS_NAPTR_IN);
		goto out;
	}

	f->hw_index = napt_idx;
	ret = 0;
out:
	mutex_unlock(&l->lock);
	return ret;
}

int rtl9602c_l34_flow_del(struct rtl9602c_l34 *l, struct l34_flow *f)
{
	u32 zero[L34_WORDS_NAPTR_IN] = { 0 };
	u32 napt[L34_WORDS_NAPT_OUT];
	int ret;

	if (!l->ready)
		return -ENODEV;
	mutex_lock(&l->lock);
	/* follow the outbound slot's pointer to also clear the rewrite entry */
	ret = l34_tbl_read(l, L34_TBL_NAPT_OUT, f->hw_index, napt, L34_WORDS_NAPT_OUT);
	if (!ret && l34_field_get(napt, L34_NAPT_VALID_LSP, L34_NAPT_VALID_W)) {
		u16 naptr_idx = l34_field_get(napt, L34_NAPT_HASHIN_IDX_LSP,
					      L34_NAPT_HASHIN_IDX_W);

		l34_tbl_write(l, L34_TBL_NAPT_OUT, f->hw_index, zero,
			      L34_WORDS_NAPT_OUT);
		l34_tbl_write(l, L34_TBL_NAPTR_IN, naptr_idx, zero,
			      L34_WORDS_NAPTR_IN);
	}
	mutex_unlock(&l->lock);
	return ret;
}

int rtl9602c_l34_flow_hit(struct rtl9602c_l34 *l, u16 hw_index, bool *active)
{
	u32 word;

	if (!l->ready)
		return -ENODEV;
	word = l34_rd(l, L34_NAPT_HIT + 4 * (hw_index / 32));
	*active = !!(word & BIT(hw_index % 32));
	return 0;
}

/*
 * WAN forwarding bring-up: the per-interface state a NAPT flow egresses through.
 * The L2 unicast table sits behind its own indirect block (poll BUSY, write the
 * data words, issue a command, poll BUSY, read the assigned index back).
 */
static int l2_busy_wait(struct rtl9602c_l34 *l)
{
	unsigned int t;

	for (t = 0; t < L34_EXE_POLL_US; t++) {
		if (!(l34_rd(l, L2_STS) & L2_STS_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

/* Insert a static unicast MAC by the MAC-hash method; the engine picks a free
 * way and reports the entry index (used as NEXTHOP/ARP nhIdx). */
static int l2uc_add_static(struct rtl9602c_l34 *l, const u8 *mac, u8 port)
{
	u32 w[L34_WORDS_L2UC] = { 0 };
	u32 sts;
	int ret;

	/* 48-bit MAC: octet[0] sits at the field's most-significant byte. */
	l34_field_set(w, L2UC_MAC_LSP, 32,
		      ((u32)mac[2] << 24) | ((u32)mac[3] << 16) |
		      ((u32)mac[4] << 8)  |  (u32)mac[5]);
	l34_field_set(w, L2UC_MAC_LSP + 32, 16,
		      ((u32)mac[0] << 8) | (u32)mac[1]);
	l34_field_set(w, L2UC_STATIC_LSP,  L2UC_STATIC_W,  1);
	l34_field_set(w, L2UC_SPA_LSP,     L2UC_SPA_W,     port);
	l34_field_set(w, L2UC_AGE_LSP,     L2UC_AGE_W,     1);
	l34_field_set(w, L2UC_ARPUSED_LSP, L2UC_ARPUSED_W, 1);
	l34_field_set(w, L2UC_VALID_LSP,   L2UC_VALID_W,   1);

	ret = l2_busy_wait(l);
	if (ret)
		return ret;
	l34_wr(l, L2_WDATA + 0, w[0]);
	l34_wr(l, L2_WDATA + 4, w[1]);
	l34_wr(l, L2_WDATA + 8, w[2]);
	l34_wr(l, L2_CMD, L2_CMD_WR | (L2_METHOD_MAC << L2_CMD_METHOD_SH));

	ret = l2_busy_wait(l);
	if (ret)
		return ret;
	sts = l34_rd(l, L2_STS);
	if (!(sts & L2_STS_HIT))
		return -ENOSPC;
	return ((sts & L2_STS_CAM) ? (1 << 10) : 0) | (sts & L2_STS_ADDR_MASK);
}

int rtl9602c_l34_wan_setup(struct rtl9602c_l34 *l, u8 idx, u32 wan_ip,
			   const u8 *wan_mac, u32 gw_ip, const u8 *gw_mac,
			   u8 wan_port, u16 vlan)
{
	u32 netif[L34_WORDS_NETIF] = { 0 };
	u32 rt[L34_WORDS_L3ROUTE] = { 0 };
	u32 extip[L34_WORDS_EXTIP] = { 0 };
	u32 nh[L34_WORDS_NEXTHOP] = { 0 };
	u32 arp[L34_WORDS_ARP] = { 0 };
	int l2idx, ret;

	if (!l->ready)
		return -ENODEV;
	if (idx >= L34_EXTIP_SLOTS)	/* NAPTR EXTIP_IDX is only 3 bits wide */
		return -EINVAL;

	mutex_lock(&l->lock);
	if (!l->engine_on)
		l34_engine_on(l);

	/* gateway dst MAC -> L2 unicast table; capture the assigned index */
	l2idx = l2uc_add_static(l, gw_mac, wan_port);
	if (l2idx < 0) {
		ret = l2idx;
		goto out;
	}

	/* NETIF[idx]: egress source MAC + VLAN/MTU/IP, routing enabled */
	l34_field_set(netif, L34_NETIF_VALID_LSP,   L34_NETIF_VALID_W,   1);
	l34_field_set(netif, L34_NETIF_VLANID_LSP,  L34_NETIF_VLANID_W,  vlan);
	l34_field_set(netif, L34_NETIF_GMAC_LSP, 32,
		      ((u32)wan_mac[2] << 24) | ((u32)wan_mac[3] << 16) |
		      ((u32)wan_mac[4] << 8)  |  (u32)wan_mac[5]);
	l34_field_set(netif, L34_NETIF_GMAC_LSP + 32, 16,
		      ((u32)wan_mac[0] << 8) | (u32)wan_mac[1]);
	l34_field_set(netif, L34_NETIF_MACMASK_LSP, L34_NETIF_MACMASK_W, L34_NETIF_DEF_MACMASK);
	l34_field_set(netif, L34_NETIF_ENRTR_LSP,   L34_NETIF_ENRTR_W,   1);
	l34_field_set(netif, L34_NETIF_MTU_LSP,     L34_NETIF_MTU_W,     L34_NETIF_DEF_MTU);
	l34_field_set(netif, L34_NETIF_L34_LSP,     L34_NETIF_L34_W,     1);
	l34_field_set(netif, L34_NETIF_IP_LSP,      L34_NETIF_IP_W,      wan_ip);
	ret = l34_tbl_write(l, L34_TBL_NETIF, idx, netif, L34_WORDS_NETIF);
	if (ret)
		goto out;

	/* LOCAL ROUTE[idx]: classify L34-domain ingress on this WAN netif and set the
	 * US/DS direction the NAPT path keys on (the vendor leaves valid=0 here, which
	 * is why offload silently fails — we set valid=1). IP/MASK/INT left 0 (WAN). */
	l34_field_set(rt, L34_RT_PROCESS_LSP,   L34_RT_PROCESS_W,   L34_RT_PROCESS_ARP);
	l34_field_set(rt, L34_RT_DENTIF_LSP,    L34_RT_DENTIF_W,    idx);
	l34_field_set(rt, L34_RT_RT2WANINF_LSP, L34_RT_RT2WANINF_W, 1);
	l34_field_set(rt, L34_RT_VALID_LSP,     L34_RT_VALID_W,     1);
	ret = l34_tbl_write(l, L34_TBL_L3ROUTE, idx, rt, L34_WORDS_L3ROUTE);
	if (ret)
		goto out;

	/* NEXTHOP[idx]: ethernet next-hop via NETIF[idx], dst MAC = L2[l2idx] */
	l34_field_set(nh, L34_NH_TYPE_LSP,  L34_NH_TYPE_W,  0);	/* ETHER */
	l34_field_set(nh, L34_NH_IFIDX_LSP, L34_NH_IFIDX_W, idx);
	l34_field_set(nh, L34_NH_NHIDX_LSP, L34_NH_NHIDX_W, l2idx);
	ret = l34_tbl_write(l, L34_TBL_NEXTHOP, idx, nh, L34_WORDS_NEXTHOP);
	if (ret)
		goto out;

	/* EXTIP[idx]: the WAN source IP a NAPT rewrite applies, via NEXTHOP[idx] */
	l34_field_set(extip, L34_EXTIP_EXTIP_LSP, L34_EXTIP_EXTIP_W, wan_ip);
	l34_field_set(extip, L34_EXTIP_VALID_LSP, L34_EXTIP_VALID_W, 1);
	l34_field_set(extip, L34_EXTIP_TYPE_LSP,  L34_EXTIP_TYPE_W,  0);	/* NAPT */
	l34_field_set(extip, L34_EXTIP_NHIDX_LSP, L34_EXTIP_NHIDX_W, idx);
	ret = l34_tbl_write(l, L34_TBL_EXTIP, idx, extip, L34_WORDS_EXTIP);
	if (ret)
		goto out;

	/* ARP entry: gateway IP -> the same L2 entry (placed in the WAN half). */
	l34_field_set(arp, L34_ARP_IP_LSP,    L34_ARP_IP_W,    gw_ip);
	l34_field_set(arp, L34_ARP_VALID_LSP, L34_ARP_VALID_W, 1);
	l34_field_set(arp, L34_ARP_NHIDX_LSP, L34_ARP_NHIDX_W, l2idx);
	ret = l34_tbl_write(l, L34_TBL_ARP, 64 + idx, arp, L34_WORDS_ARP);
out:
	mutex_unlock(&l->lock);
	return ret;
}

/*
 * Program a LAN-side interface: a NETIF (the ONU's LAN MAC/IP, in the L34
 * domain) plus a local route (process=ARP, internal=1) classifying the LAN
 * subnet — so a US (LAN->WAN) frame's MAC-base lookup resolves to a netif and
 * the DS return is routed back to the LAN.
 */
int rtl9602c_l34_lan_setup(struct rtl9602c_l34 *l, u8 idx, u32 lan_ip,
			   const u8 *lan_mac, u32 lan_net, u8 prefix, u16 vlan)
{
	u32 netif[L34_WORDS_NETIF] = { 0 };
	u32 rt[L34_WORDS_L3ROUTE] = { 0 };
	int ret;

	if (!l->ready)
		return -ENODEV;
	if (idx >= L34_EXTIP_SLOTS || prefix < 1 || prefix > 32)
		return -EINVAL;

	mutex_lock(&l->lock);
	if (!l->engine_on)
		l34_engine_on(l);

	l34_field_set(netif, L34_NETIF_VALID_LSP,   L34_NETIF_VALID_W,   1);
	l34_field_set(netif, L34_NETIF_VLANID_LSP,  L34_NETIF_VLANID_W,  vlan);
	l34_field_set(netif, L34_NETIF_GMAC_LSP, 32,
		      ((u32)lan_mac[2] << 24) | ((u32)lan_mac[3] << 16) |
		      ((u32)lan_mac[4] << 8)  |  (u32)lan_mac[5]);
	l34_field_set(netif, L34_NETIF_GMAC_LSP + 32, 16,
		      ((u32)lan_mac[0] << 8) | (u32)lan_mac[1]);
	l34_field_set(netif, L34_NETIF_MACMASK_LSP, L34_NETIF_MACMASK_W, L34_NETIF_DEF_MACMASK);
	l34_field_set(netif, L34_NETIF_ENRTR_LSP,   L34_NETIF_ENRTR_W,   1);
	l34_field_set(netif, L34_NETIF_MTU_LSP,     L34_NETIF_MTU_W,     L34_NETIF_DEF_MTU);
	l34_field_set(netif, L34_NETIF_L34_LSP,     L34_NETIF_L34_W,     1);
	l34_field_set(netif, L34_NETIF_IP_LSP,      L34_NETIF_IP_W,      lan_ip);
	ret = l34_tbl_write(l, L34_TBL_NETIF, idx, netif, L34_WORDS_NETIF);
	if (ret)
		goto out;

	l34_field_set(rt, L34_RT_IP_LSP,      L34_RT_IP_W,      lan_net);
	l34_field_set(rt, L34_RT_MASK_LSP,    L34_RT_MASK_W,    prefix - 1);
	l34_field_set(rt, L34_RT_PROCESS_LSP, L34_RT_PROCESS_W, L34_RT_PROCESS_ARP);
	l34_field_set(rt, L34_RT_INT_LSP,     L34_RT_INT_W,     1);	/* LAN */
	l34_field_set(rt, L34_RT_DENTIF_LSP,  L34_RT_DENTIF_W,  idx);
	l34_field_set(rt, L34_RT_VALID_LSP,   L34_RT_VALID_W,   1);
	ret = l34_tbl_write(l, L34_TBL_L3ROUTE, idx, rt, L34_WORDS_L3ROUTE);
	if (ret)
		goto out;

	/* A more-specific /32 CPU route for the interface's OWN IP, so traffic
	 * addressed to the ONU itself (management) terminates locally instead of
	 * being ARP-routed by the LAN subnet route above (which black-holes the
	 * mgmt path once the LAN netif is in the L34 domain). */
	memset(rt, 0, sizeof(rt));
	l34_field_set(rt, L34_RT_IP_LSP,      L34_RT_IP_W,      lan_ip);
	l34_field_set(rt, L34_RT_MASK_LSP,    L34_RT_MASK_W,    31);	/* /32 */
	l34_field_set(rt, L34_RT_PROCESS_LSP, L34_RT_PROCESS_W, L34_RT_PROCESS_CPU);
	l34_field_set(rt, L34_RT_INT_LSP,     L34_RT_INT_W,     1);
	l34_field_set(rt, L34_RT_DENTIF_LSP,  L34_RT_DENTIF_W,  idx);
	l34_field_set(rt, L34_RT_VALID_LSP,   L34_RT_VALID_W,   1);
	ret = l34_tbl_write(l, L34_TBL_L3ROUTE, idx + 8, rt, L34_WORDS_L3ROUTE);
out:
	mutex_unlock(&l->lock);
	return ret;
}

/*
 * Bring-up test harness: /proc/rtl9602c_l34, write-driven so the live WAN
 * parameters (DHCP address, gateway MAC learned via ARP) can be fed in from
 * userspace once the WAN is up:
 *   w <wanMAC> <wanIPhex> <gwIPhex> <gwMAC> <port> <vlan>             -> wan_setup
 *   f <proto> <sIPhex> <sPort> <dIPhex> <dPort> <natIPhex> <natPort>  -> flow_add
 * Reading the file dumps the outbound NAPT hit bitmap. A developer aid gated
 * behind the engine being enabled, not part of the production datapath.
 */
static ssize_t l34_proc_write(struct file *fp, const char __user *ub,
			      size_t n, loff_t *off)
{
	struct rtl9602c_l34 *l = pde_data(file_inode(fp));
	u8 wmac[6], gmac[6];
	char buf[192];
	int ret = 0;

	if (n >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ub, n))
		return -EFAULT;
	buf[n] = '\0';

	if (buf[0] == 'w') {
		unsigned int port, vlan;
		u32 wip, gip;

		if (sscanf(buf, "w %2hhx%2hhx%2hhx%2hhx%2hhx%2hhx %x %x %2hhx%2hhx%2hhx%2hhx%2hhx%2hhx %u %u",
			   &wmac[0], &wmac[1], &wmac[2], &wmac[3], &wmac[4], &wmac[5],
			   &wip, &gip,
			   &gmac[0], &gmac[1], &gmac[2], &gmac[3], &gmac[4], &gmac[5],
			   &port, &vlan) != 16)
			return -EINVAL;
		ret = rtl9602c_l34_wan_setup(l, 0, wip, wmac, gip, gmac, port, vlan);
		pr_info("rtl9602c_l34: wan_setup -> %d\n", ret);
	} else if (buf[0] == 'l') {
		unsigned int prefix, vlan;
		u32 lip, lnet;

		if (sscanf(buf, "l %2hhx%2hhx%2hhx%2hhx%2hhx%2hhx %x %x %u %u",
			   &wmac[0], &wmac[1], &wmac[2], &wmac[3], &wmac[4], &wmac[5],
			   &lip, &lnet, &prefix, &vlan) != 10)
			return -EINVAL;
		ret = rtl9602c_l34_lan_setup(l, 1, lip, wmac, lnet, prefix, vlan);
		pr_info("rtl9602c_l34: lan_setup -> %d\n", ret);
	} else if (buf[0] == 'f') {
		struct l34_flow f = { 0 };
		unsigned int proto, sp, dp, nsp;

		if (sscanf(buf, "f %u %x %u %x %u %x %u",
			   &proto, &f.orig_sip, &sp, &f.orig_dip, &dp,
			   &f.nat_sip, &nsp) != 7)
			return -EINVAL;
		f.l4proto = proto;
		f.orig_sport = sp;
		f.orig_dport = dp;
		f.nat_sport = nsp;
		ret = rtl9602c_l34_flow_add(l, &f);
		pr_info("rtl9602c_l34: flow_add -> %d hw_index=%u\n", ret, f.hw_index);
	} else {
		return -EINVAL;
	}
	return ret ? ret : n;
}

static int l34_proc_show(struct seq_file *sf, void *v)
{
	struct rtl9602c_l34 *l = sf->private;
	unsigned int i;

	seq_puts(sf, "outbound NAPT hit bitmap (set bits = recently matched slots):\n");
	for (i = 0; i < L34_NAPT_ENTRIES / 32; i++) {
		u32 w = l34_rd(l, L34_NAPT_HIT + 4 * i);

		if (w)
			seq_printf(sf, "  [%4u] 0x%08x\n", i * 32, w);
	}
	return 0;
}

static int l34_proc_open(struct inode *ino, struct file *fp)
{
	return single_open(fp, l34_proc_show, pde_data(ino));
}

static const struct proc_ops l34_proc_ops = {
	.proc_open	= l34_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= l34_proc_write,
};

void rtl9602c_l34_proc_init(struct rtl9602c_l34 *l)
{
	proc_create_data("rtl9602c_l34", 0600, NULL, &l34_proc_ops, l);
}
