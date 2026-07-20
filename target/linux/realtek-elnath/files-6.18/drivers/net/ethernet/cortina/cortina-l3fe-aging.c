// SPDX-License-Identifier: GPL-2.0-only
/*
 * cortina-l3fe-aging.c — Cortina CA8277C (RTL9607F "Elnath") L3FE main-hash
 * flow-engine AGING / GC / HIT-FEED sub-block for the nf_flow_table HW offload.
 *
 * This is the sub-block that keeps conntrack/flowtable timeouts refreshed from
 * hardware traffic at scale, and evicts dead flows out of the 64K main hash:
 *
 *   - autonomous HW hit-rearm: a packet match re-arms an entry's 2-bit age to
 *     START(2).  We keep the HW auto-countdown (AGING_GRANULARITY) DISABLED, to
 *     match stock, so hardware never ages a live offloaded flow out from under
 *     the Linux flowtable.
 *   - SW keepalive sweep: a batch "had-traffic" read+clear of a whole 32-entry
 *     bucket in ONE indirect access (l3fe_traffic_status_get) — bit set == HW saw
 *     a packet since the last sweep; the same access clears every live slot back
 *     to IDLE(1).  This feeds FLOW_CLS_STATS -> f->stats.lastused.
 *   - AQM byte/packet MIB (up to 2048 metered flows) for real counters.
 *   - go-live / kill via the age field; explicit on-chip cache invalidate on del.
 *
 * Clean-room re-expression of the shipped stock ca-ne.ko (aal-07f) register
 * sequences + the Cortina aal HAL semantics.  No vendor source text reproduced.
 * Facts, bit layouts and Ghidra citations:
 *   dev/x411axf/HW_FLOW_OFFLOAD_AGING_GC.md
 *
 * ★ Elnath-specific: the main-hash age is 2-BIT (0..3), START=2, STATIC=3 —
 *   NOT the 3-bit/START=6 model in the aal-77c HAL header.  Proven by the
 *   ca-ne.ko decomp (age masked &3, /0x2000 in aging_timer_set, 0x7ffffffe000 in
 *   hash_init).  A wrong-width age RMW corrupts the neighbouring slot.
 *
 * NOTE: built under CONFIG_CORTINA_NI_FLOWOFFLOAD as the documented aging/
 *       AQM reference.  The production liveness sweep is the copy folded into
 *       cortina-ni-flowoffload.c (integrated with its SW shadow); the AQM MIB
 *       reader here becomes the phase-4 per-flow byte/packet counter option.
 *       Register offsets verified decomp==rtl8277c header.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

/* -------------------------------------------------------------------------
 * NE register base + L3FE hash-engine (HS) aging/cache register file.
 * NE APB0 base = 0xf4300000 (proven: our RX driver uses the rtl8277c L3FE map,
 * ILPB_LDPID @0xf43030d8; the aal-07f decomp offsets match rtl8277c exactly).
 * Offsets below are relative to the ioremap of NE_REG_BASE.
 * ------------------------------------------------------------------------- */
#define CL3A_NE_REG_BASE		0xf4300000UL

/* --- indirect age memory (main-hash + overflow) --- */
#define CL3A_AGING_GRANULARITY		0x3924	/* timer[29:0]; 0 = auto-countdown OFF */
#define CL3A_AGE_ACCESS			0x3928	/* address[11:0] r0w1[30] go/busy[31] */
#define CL3A_AGE_DATA0			0x3938	/* slots 0..15  (2-bit age each) */
#define CL3A_AGE_DATA1			0x3934	/* slots 16..31 (2-bit age each) */
#define  CL3A_AGE_TBL_HASH		(0u << 11)	/* address bit11 = age-table select */
#define  CL3A_AGE_TBL_OVERFLOW		(1u << 11)

/* --- on-chip action-cache control (invalidate/allocate) --- */
#define CL3A_CACHE_CTRL			0x38ac	/* slot[4:0] crc16[20:5] loc[24] age[27:26] pri[29:28] cmd[31:30] */
#define CL3A_CACHE_CTRL_REQ		0x38b0	/* req_sts[0] = go/busy */
#define CL3A_CACHE_CTRL_STS		0x38b4	/* bsy0 err_hash1 err_free2 err_nch3 match4 match_loc5 evict6 */
#define  CL3A_CACHE_CMD_INVALIDATE	(1u << 30)	/* cmd = 01 */
#define  CL3A_CACHE_STS_ERR_NCH		BIT(3)		/* "entry not cached" (non-fatal) */
#define  CL3A_CACHE_STS_EVICT		BIT(6)
#define  CL3A_CACHE_STS_CACHED		BIT(4)
#define  CL3A_CACHE_STS_NO_WAY		BIT(2)		/* alloc error */
#define  CL3A_CACHE_STS_INVALID		BIT(1)		/* alloc error */

/* --- AQM byte/packet MIB: shares the cache indirect access reg, table=2 --- */
#define CL3A_CACHE_HASH_ACCESS		0x38e4	/* AQM/cache indirect access reg */
#define  CL3A_AQM_TBL_SEL		(2u << 11)
#define CL3A_AQM_DATA_REGS		7	/* data word i at ACCESS + ((7-i)<<2) */

#define CL3A_GO				BIT(31)
#define CL3A_WRITE			BIT(30)
#define CL3A_POLL_TRIES			1000	/* == vendor TABLE_TRY_TIMEOUT */

/* --- geometry --- */
#define CL3A_ENTRIES			65536
#define CL3A_WAYS			32	/* slots per bucket */
#define CL3A_BUCKETS			(CL3A_ENTRIES / CL3A_WAYS)	/* 2048 */
#define CL3A_AQM_MAX			2048	/* L3FE_AQM_FLOW_STAT_MAX */

/* --- 2-bit age codes (Elnath 07f) --- */
#define CL3A_AGE_FREE			0	/* aged-out / invalid, stops matching */
#define CL3A_AGE_IDLE			1	/* keepalive floor written by the sweep */
#define CL3A_AGE_START			2	/* on add; HW re-arms here on a hit */
#define CL3A_AGE_STATIC			3	/* never ages out */

struct cl3a_ctx {
	void __iomem *ne;		/* ioremap of NE_REG_BASE */
	bool cache_enabled;		/* on-chip action cache in use */
};

/* ------------------------------------------------------------------ */
/* Indirect-access helpers — every op is a bounded poll on the GO bit. */
/* ------------------------------------------------------------------ */
static inline u32 cl3a_rd(struct cl3a_ctx *c, u32 off)
{
	return readl(c->ne + off);
}

static inline void cl3a_wr(struct cl3a_ctx *c, u32 off, u32 val)
{
	/* dmb before every register write, mirroring the stock DataMemoryBarrier */
	dma_wmb();
	writel(val, c->ne + off);
}

/* Poll a self-clearing GO/busy bit; returns 0 on done, -ETIMEDOUT on cap. */
static int cl3a_poll_go(struct cl3a_ctx *c, u32 off, u32 gobit)
{
	int i;

	for (i = 0; i < CL3A_POLL_TRIES; i++) {
		if (!(cl3a_rd(c, off) & gobit))
			return 0;
	}
	return -ETIMEDOUT;
}

/* Map a slot to (age data-reg offset, in-word bit shift).  2-bit ages,
 * 16 slots per 32-bit word: word0 = slots 0..15, word1 = slots 16..31. */
static inline u32 cl3a_age_dataoff(u32 slot)
{
	return (slot & 0x1f) < 16 ? CL3A_AGE_DATA0 : CL3A_AGE_DATA1;
}
static inline u32 cl3a_age_shift(u32 slot)
{
	return 2u * (slot & 0xf);
}

/* ------------------------------------------------------------------ */
/* 3a. Read one entry's 2-bit age.                                     */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_age_get(struct cl3a_ctx *c, u32 idx, u8 *age)
{
	u32 addr = ((idx >> 5) & 0x7ff) | CL3A_AGE_TBL_HASH;
	u32 word;
	int ret;

	cl3a_wr(c, CL3A_AGE_ACCESS, addr | CL3A_GO);		/* start read */
	ret = cl3a_poll_go(c, CL3A_AGE_ACCESS, CL3A_GO);
	if (ret)
		return ret;

	word = cl3a_rd(c, cl3a_age_dataoff(idx));
	*age = (word >> cl3a_age_shift(idx)) & 0x3;
	return 0;
}

/* ------------------------------------------------------------------ */
/* 3b. Write one entry's 2-bit age.  age>0 = GO-LIVE, age==0 = KILL.   */
/*     RMW: ages share a 32-bit word, so read-modify the row first.    */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_age_set(struct cl3a_ctx *c, u32 idx, u8 age)
{
	u32 addr = ((idx >> 5) & 0x7ff) | CL3A_AGE_TBL_HASH;
	u32 dataoff = cl3a_age_dataoff(idx);
	u32 shift = cl3a_age_shift(idx);
	u32 word;
	int ret;

	age &= 0x3;

	/* 1. read the bucket's age row */
	cl3a_wr(c, CL3A_AGE_ACCESS, addr | CL3A_GO);
	ret = cl3a_poll_go(c, CL3A_AGE_ACCESS, CL3A_GO);
	if (ret)
		return ret;

	/* 2. patch just this slot's 2 bits */
	word = cl3a_rd(c, dataoff);
	word = (word & ~(0x3u << shift)) | ((u32)age << shift);
	cl3a_wr(c, dataoff, word);

	/* 3. commit = GO-LIVE (bit31 go + bit30 write) */
	cl3a_wr(c, CL3A_AGE_ACCESS, addr | CL3A_GO | CL3A_WRITE);
	return cl3a_poll_go(c, CL3A_AGE_ACCESS, CL3A_GO);
}

/* ------------------------------------------------------------------ */
/* 3c. Batch had-traffic read+clear of ONE 32-entry bucket.            */
/*     bucket_base_idx must be 32-aligned (slot 0 of the group).       */
/*     *trf = 32-bit bitmap, bit k set iff slot k had traffic          */
/*     (age > IDLE) since the last sweep; the same op resets every      */
/*     live slot back to IDLE(1) so only a fresh HW hit lifts it again. */
/*     This is the scale path: ONE indirect read+clear per 32 flows,    */
/*     not one age_get per flow.                                        */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_traffic_status_get(struct cl3a_ctx *c, u32 bucket_base_idx,
				   u32 *trf)
{
	u32 addr = ((bucket_base_idx >> 5) & 0x7ff) | CL3A_AGE_TBL_HASH;
	static const u32 dregs[2] = { CL3A_AGE_DATA0, CL3A_AGE_DATA1 };
	int ret, w, k;

	if (bucket_base_idx & 0x1f)		/* only the first index of a group */
		return -EINVAL;

	*trf = 0;

	/* read the whole 32-slot age row */
	cl3a_wr(c, CL3A_AGE_ACCESS, addr | CL3A_GO);
	ret = cl3a_poll_go(c, CL3A_AGE_ACCESS, CL3A_GO);
	if (ret)
		return ret;

	for (w = 0; w < 2; w++) {		/* 2 words x 16 slots x 2 bits */
		u32 word = cl3a_rd(c, dregs[w]);

		for (k = 0; k < 16; k++) {
			u32 age = (word >> (2 * k)) & 0x3;
			u32 slot = (w << 4) + k;

			if (age > CL3A_AGE_IDLE)	/* HW re-armed -> had traffic */
				*trf |= (1u << slot);
			/* keepalive: nonzero -> IDLE(1), zero stays 0 */
			word &= ~(0x3u << (2 * k));
			word |= (age ? CL3A_AGE_IDLE : CL3A_AGE_FREE) << (2 * k);
		}
		cl3a_wr(c, dregs[w], word);
	}

	/* commit the cleared row (GO-LIVE write) */
	cl3a_wr(c, CL3A_AGE_ACCESS, addr | CL3A_GO | CL3A_WRITE);
	return cl3a_poll_go(c, CL3A_AGE_ACCESS, CL3A_GO);
}

/* ------------------------------------------------------------------ */
/* 3e. On-chip action-cache invalidate (part of flow delete).         */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_cache_invalidate(struct cl3a_ctx *c, u32 idx, u16 crc16)
{
	u32 ctrl, sts;
	int ret;

	ctrl = (idx & (CL3A_WAYS - 1))			/* slot within bucket */
	     | ((u32)crc16 << 5)			/* crc16[20:5] */
	     | CL3A_CACHE_CMD_INVALIDATE;		/* cmd = 01 */

	/* decomp order: write CTRL params, wait REQ idle, pulse REQ GO, wait done */
	cl3a_wr(c, CL3A_CACHE_CTRL, ctrl);
	ret = cl3a_poll_go(c, CL3A_CACHE_CTRL_REQ, BIT(0));	/* wait not-busy */
	if (ret)
		return ret;
	cl3a_wr(c, CL3A_CACHE_CTRL_REQ, cl3a_rd(c, CL3A_CACHE_CTRL_REQ) | 1);	/* GO */
	ret = cl3a_poll_go(c, CL3A_CACHE_CTRL_REQ, BIT(0));
	if (ret)
		return ret;

	sts = cl3a_rd(c, CL3A_CACHE_CTRL_STS);
	if (sts & CL3A_CACHE_STS_ERR_NCH)
		pr_debug("l3fe cache invalidate idx %u: entry was not cached (non-fatal)\n",
			 idx);
	return 0;
}

/* ------------------------------------------------------------------ */
/* 3d. Flow delete — order is load-bearing (stop-matching FIRST).      */
/*     Caller supplies the recovered crc16 (from the key row) and the  */
/*     FIB/key row clear callbacks (in the sibling flow file).         */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_flow_del(struct cl3a_ctx *c, u32 idx, u16 crc16,
			 int (*clear_fib)(struct cl3a_ctx *, u32),
			 int (*clear_key)(struct cl3a_ctx *, u32))
{
	int ret;

	ret = cl3a_age_set(c, idx, CL3A_AGE_FREE);	/* 1. STOP MATCHING FIRST */
	if (ret)
		return ret;
	if (clear_fib) {
		ret = clear_fib(c, idx);		/* 2. clear action/FIB row */
		if (ret)
			return ret;
	}
	if (clear_key) {
		ret = clear_key(c, idx);		/* 3. clear key/hash row */
		if (ret)
			return ret;
	}
	if (c->cache_enabled)				/* 4. flush on-chip cache */
		cl3a_cache_invalidate(c, idx, crc16);
	return 0;
}

/* ------------------------------------------------------------------ */
/* 3g. AQM byte/packet MIB read (up to 2048 metered flows).           */
/*     MIB mode (AQM_SET.cnt_md=1): word0=byteCnt, word1=pktCnt[17:0]. */
/* ------------------------------------------------------------------ */
static int __maybe_unused cl3a_aqm_mib_get(struct cl3a_ctx *c, u32 idx, u64 *bytes, u32 *pkts)
{
	u32 words[CL3A_AQM_DATA_REGS];
	int ret, i;

	if (idx >= CL3A_AQM_MAX)
		return -EINVAL;

	cl3a_wr(c, CL3A_CACHE_HASH_ACCESS,
		(idx & 0x7ff) | CL3A_GO | CL3A_AQM_TBL_SEL);	/* read, table=2 */
	ret = cl3a_poll_go(c, CL3A_CACHE_HASH_ACCESS, CL3A_GO);
	if (ret)
		return ret;

	for (i = 0; i < CL3A_AQM_DATA_REGS; i++)
		words[i] = cl3a_rd(c, CL3A_CACHE_HASH_ACCESS + ((CL3A_AQM_DATA_REGS - i) << 2));

	if (bytes)
		*bytes = words[0];			/* byteCnt[31:0] */
	if (pkts)
		*pkts = words[1] & 0x3ffff;		/* pktCnt[17:0] */
	return 0;
}

/* ------------------------------------------------------------------ */
/* Aging-init: keep the HW auto-countdown DISABLED (match stock).      */
/* HW hit-rearm + the SW sweep drive expiry; the flowtable's own       */
/* teardown timer decides the actual delete.                           */
/* ------------------------------------------------------------------ */
static void __maybe_unused cl3a_aging_init(struct cl3a_ctx *c)
{
	cl3a_wr(c, CL3A_AGING_GRANULARITY, 0);	/* auto age-countdown OFF */
}

/*
 * GC sweep (to be driven by a ~4s delayed_work in the wiring phase):
 *
 *   for each OCCUPIED bucket b (walk the SW shadow's per-bucket valid mask,
 *   NOT all 2048 blindly):
 *       cl3a_traffic_status_get(c, b << 5, &trf);
 *       for each valid slot k in b:
 *           if (trf & (1<<k)) entry[b*32+k]->last_hit = jiffies;   // fed to
 *                                                                  // FLOW_CLS_STATS
 *   Do NOT delete here on idle — report lastused and let the Linux flowtable
 *   teardown timer call cl3a_flow_del().  One indirect read+clear per 32 flows
 *   is what makes >10k flows affordable (2048 ops max vs 10k+ per-flow reads).
 */
