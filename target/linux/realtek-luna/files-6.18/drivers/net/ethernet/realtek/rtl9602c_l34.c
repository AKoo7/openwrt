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
#include <linux/string.h>

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

int rtl9602c_l34_init(struct rtl9602c_l34 *l, void __iomem *sw)
{
	u32 v;

	if (!sw)
		return -EINVAL;
	l->sw = sw;
	mutex_init(&l->lock);

	/* Reset every table type we use, then arm the engine: L3+L4 NAT on,
	 * MAC-base lookup. The hashed NAPT path is used (flow-routing left off). */
	l34_wr(l, L34_CLR, 0xffff);
	usleep_range(50, 100);

	v = l34_rd(l, L34_SWTCR0);
	v &= ~(0x3u << L34_SWTCR0_NATMODE_SH);
	v |=  (0x3u << L34_SWTCR0_NATMODE_SH);		/* L3 + L4 NAT enable */
	v &= ~(0x3u << L34_SWTCR0_LOOKUP_SH);
	v |=  (0x2u << L34_SWTCR0_LOOKUP_SH);		/* MAC-base lookup */
	v &= ~(L34_SWTCR0_V4RT_EN | L34_SWTCR0_V6RT_EN);
	l34_wr(l, L34_SWTCR0, v);

	l34_wr(l, L34_GLB_CFG, l34_rd(l, L34_GLB_CFG) | BIT(0));	/* master enable */

	l->ready = true;
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
