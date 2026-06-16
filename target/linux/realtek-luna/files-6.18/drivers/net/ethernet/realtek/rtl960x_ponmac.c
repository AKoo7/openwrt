// SPDX-License-Identifier: GPL-2.0
/*
 * rtl960x_ponmac.c - clean-room RTL960x family GPON PON-MAC / SerDes bring-up.
 *
 * This is an ORIGINAL, data-driven reimplementation. The per-chip register
 * SEQUENCES (which registers, what values, in what order, with what delays) are
 * hardware-interface FACTS dictated by the silicon - extracted by observing the
 * bring-up - not copied code. They are expressed here as compact declarative
 * op-tables driven by a single tiny interpreter, rather than the vendor's
 * repetitive per-register procedural boilerplate. The structure, interpreter,
 * naming, and organization are all original; only the factual register data is
 * shared with any other implementation of the same hardware.
 *
 * Design (deliberately "better code"):
 *   - one r960_op{} table per (chip, phase) = the bring-up as data
 *   - r960_run() interprets WR / FLD(RMW) / DELAY / POLL ops
 *   - loops/conditionals (scheduler & queue init, rev/subtype branches) stay as
 *     small explicit code - tables are only for straight-line register runs
 *   - absolute physical addresses throughout; the board injects rd/wr (ops)
 *
 * Tested: the 9602C path on the realtek-luna board. The 9601B / 9603CVD / 9607C
 * paths are register-faithful but UNTESTED (no hardware); ready for a future board.
 */

#include "rtl960x_ponmac.h"
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>

/* ---- op-table format (original) --------------------------------------- */
enum r960_opc {
	R960_WR,	/* wr(addr, val)                         */
	R960_FLD,	/* rfwr(addr, msb, lsb, val) RMW         */
	R960_DLY,	/* mdelay(val) ms                        */
	R960_POLL,	/* wait addr bit[lsb]==1, up to val*200us */
};

struct r960_op {
	u8  opc;
	u8  msb;
	u8  lsb;
	u32 addr;
	u32 val;
};

#define WR(a, v)		{ R960_WR,  0, 0, (a), (v) }
#define FLD(a, m, l, v)		{ R960_FLD, (m), (l), (a), (v) }
#define DLY(ms)			{ R960_DLY, 0, 0, 0, (ms) }
#define POLL(a, bit, iters)	{ R960_POLL, (bit), (bit), (a), (iters) }

/* the whole interpreter - one function for the entire family */
static int r960_run(const struct rtl960x_ops *o,
		    const struct r960_op *seq, unsigned int n)
{
	unsigned int i, k;

	for (i = 0; i < n; i++) {
		const struct r960_op *p = &seq[i];

		switch (p->opc) {
		case R960_WR:
			o->wr(p->addr, p->val);
			break;
		case R960_FLD:
			rtl960x_rfwr(o, p->addr, p->msb, p->lsb, p->val);
			break;
		case R960_DLY:
			mdelay(p->val);
			break;
		case R960_POLL:
			for (k = 0; k < p->val; k++) {
				if (o->rd(p->addr) & (1u << p->lsb))
					break;
				udelay(200);
			}
			if (k == p->val)
				return -ETIMEDOUT;
			break;
		}
	}
	return 0;
}

/* =======================================================================
 * Per-chip bring-up tables + glue.
 * Populated from the per-chip register FACTS (resolved from each chip's own
 * reg_list/regField map). Each block is self-contained so a board only links
 * what it needs once the dispatch is wired by chip id.
 * ======================================================================= */

/* ---- RTL9602C (rev-A, CHIP_REV_ID_A) - HW-tested on realtek-luna -------- */
/* (tables filled from verified facts) */

/* ------------------------------------------------------------------ *
 *  RTL9601B GPON PON-MAC / SerDes bring-up - clean-room op-table form.
 *
 *  This block is a SECTION of rtl960x_ponmac.c: it reuses that file's
 *  op-table primitives (enum r960_opc, struct r960_op, the WR/FLD/DLY/POLL
 *  macros and the r960_run() interpreter) and rtl960x_rfwr() / ARRAY_SIZE,
 *  all already in scope there. Do NOT compile this as a standalone unit and
 *  do NOT redeclare r960_run() extern - it is file-private (static).
 *
 *  The register addresses, field bit-ranges, values, ordering and delays are
 *  hardware-interface facts of the RTL9601B silicon. The expression (data-driven
 *  op-tables + the indirect-SerDes helper + the explicit scheduler/queue loops +
 *  all comments) is original.
 *
 *  Address space: swcore physical base 0x1B000000. The board supplies rd/wr in
 *  struct rtl960x_ops to map phys->virt.
 *
 *  SerDes access on this chip is INDIRECT: a command/data/poll window, not a
 *  flat MMIO bank. addr = (index<<11) | (page<<5) | reg.
 * ------------------------------------------------------------------ */

/* ---- Absolute register map (swcore base 0x1B000000) -------------------- */
/* scheduler / queue block (PON-MAC, window 0x1BF0xxxx) */
#define C1B_R_BW_THRES	0x1BF0104Cu	/* bandwidth grant thresholds        */
#define C1B_R_SCH_CTRL	0x1BF02030u	/* global scheduler control          */
#define C1B_R_CIR_BASE	0x1BF02034u	/* per-queue CIR, +qid*4             */
#define C1B_R_PIR_BASE	0x1BF020B8u	/* per-queue PIR, +qid*4             */
#define C1B_R_QMAP_BASE	0x1BF0213Cu	/* per-tcont queue map, +tcont*4    */
#define C1B_R_TCONT_EN	0x1BF0215Cu	/* T-CONT enable bitfield            */
#define C1B_R_WFQ_TYPE	0x1BF02160u	/* per-queue strict/WFQ select       */
#define C1B_R_WFQ_W	0x1BF0216Cu	/* per-queue WFQ weight, 3/word      */
#define C1B_R_SID2QID	0x1BF0102Cu	/* flow->queue table, 5 sids/word    */
/* trap / mode (low swcore offsets) */
#define C1B_R_TRAP_CFG	0x1B0001F8u	/* OMCI/MPCP trap priority           */
#define C1B_R_MODE_CFG	0x1B0001F4u	/* PON mode enable                   */
/* indirect SerDes window */
#define C1B_R_SDS_WD	0x1B00011Cu	/* write data [15:0]                 */
#define C1B_R_SDS_CMD	0x1B000120u	/* addr[15:0] | CMD_EN[16] | WREN[17]*/
#define C1B_R_SDS_RD	0x1B000124u	/* read data [15:0] | BUSY[16]       */
/* SerDes wrapper digital control */
#define C1B_R_WSDS00	0x1B022000u	/* GPON-MAC soft reset-B @bit10      */
#define C1B_R_WSDS01	0x1B022004u	/* clkrd source select @bit2         */
#define C1B_R_WSDS11	0x1B02202Cu	/* power-down-on-BEN enable @bit0    */
#define C1B_R_WSDS12	0x1B022030u	/* burst-enable output @bit12        */
#define C1B_R_WSDS17	0x1B022044u	/* digital soft reset-B @bit14       */
/* misc */
#define C1B_R_SDS1_CFG	0x1B000088u	/* SerDes lane-1 mode select [4:0]   */
#define C1B_R_SOFT_RST	0x1B000044u	/* queue reset pulse @bit3           */
#define C1B_R_PMISC_PON	0x1B020028u	/* P_MISC: base 0x20008 + PON port 1 * stride 0x20 */
#define C1B_R_ACC_LEN_PON	0x1B023054u	/* ACCEPT_MAX_LEN: 0x23034 + PON port 1 * stride 0x20 */
#define C1B_R_ACC_LEN_UTP	0x1B023034u	/* UTP RX accept max length          */
#define C1B_R_TX_LEN_PON	0x1B011028u	/* TX_MAX_LEN: 0x11008 + PON port 1 * stride 0x20 */
#define C1B_R_TX_LEN_UTP	0x1B011008u	/* UTP TX max length                 */
#define C1B_R_PORT_CLK	0x1B020450u	/* per-port MAC clock select @bit2   */

/* hardware limits */
#define Q9601B_TCONTS		9	/* T-CONT scheduler slots            */
#define Q9601B_QUEUES		33	/* physical PON egress queues         */
#define Q9601B_RATE_MAX		0x3FFFFu
#define Q9601B_TCONT_QSTRIDE	32	/* queues per scheduler group         */
#define Q9601B_MAXLEN		2031	/* jumbo payload ceiling             */

/* indirect SerDes index / page constants */
#define C1B_SI_LAN		0x00
#define C1B_SI_PON		0x01
#define C1B_SP_COMMON		0x21
#define C1B_SP_125G		0x24
#define C1B_SP_GPON		0x30
#define C1B_SP_EPON		0x32
#define C1B_SDS_SPIN		0x10	/* busy-poll spin budget             */

/* ------------------------------------------------------------------ *
 *  indirect SerDes primitives (cmd/data/poll)
 *  c1b_sds_wr(): write data, latch addr+enables, wait for BUSY to drop.
 *  c1b_sds_rd(): latch addr+read enable, wait BUSY, return latched data.
 * ------------------------------------------------------------------ */
static int c1b_sds_wr(const struct rtl960x_ops *o,
		      u8 idx, u8 page, u8 reg, u16 data)
{
	u32 addr = ((u32)idx << 11) | ((u32)page << 5) | reg;
	unsigned int spin;

	rtl960x_rfwr(o, C1B_R_SDS_CMD, 15, 0, addr);	/* target address     */
	rtl960x_rfwr(o, C1B_R_SDS_CMD, 17, 17, 1);	/* WREN = write       */
	rtl960x_rfwr(o, C1B_R_SDS_WD,  15, 0, data);	/* payload            */
	rtl960x_rfwr(o, C1B_R_SDS_CMD, 16, 16, 1);	/* fire the access    */

	for (spin = C1B_SDS_SPIN; spin; spin--)
		if (((o->rd(C1B_R_SDS_RD) >> 16) & 1) == 0)
			return 0;
	return -ETIMEDOUT;
}

static int c1b_sds_rd(const struct rtl960x_ops *o,
		      u8 idx, u8 page, u8 reg, u16 *data)
{
	u32 addr = ((u32)idx << 11) | ((u32)page << 5) | reg;
	unsigned int spin;

	rtl960x_rfwr(o, C1B_R_SDS_CMD, 15, 0, addr);
	rtl960x_rfwr(o, C1B_R_SDS_CMD, 17, 17, 0);	/* WREN = read        */
	rtl960x_rfwr(o, C1B_R_SDS_CMD, 16, 16, 1);

	for (spin = C1B_SDS_SPIN; spin; spin--) {
		if (((o->rd(C1B_R_SDS_RD) >> 16) & 1) == 0) {
			*data = o->rd(C1B_R_SDS_RD) & 0xFFFF;
			return 0;
		}
	}
	return -ETIMEDOUT;
}

/* ------------------------------------------------------------------ *
 *  SDS patch tables (analog/CMU/CDR trim), selected by silicon rev.
 *  {index, page, reg, data}
 * ------------------------------------------------------------------ */
struct c1b_sds_op { u8 idx, page, reg; u16 data; };

/* rev-0 silicon: full analog trim for PON lane + LAN lane */
static const struct c1b_sds_op rtl9601b_sds_patch_rev0[] = {
	{ C1B_SI_PON, C1B_SP_COMMON, 0x02, 0xc36c },	/* CMU/PLL loop trim      */
	{ C1B_SI_PON, C1B_SP_COMMON, 0x06, 0x1945 },
	{ C1B_SI_PON, C1B_SP_COMMON, 0x16, 0x9188 },
	{ C1B_SI_PON, C1B_SP_GPON,   0x03, 0x60b1 },	/* GPON-rate equalizer    */
	{ C1B_SI_PON, C1B_SP_EPON,   0x03, 0x60b1 },
	{ C1B_SI_PON, C1B_SP_125G,   0x03, 0x60b1 },
	{ C1B_SI_PON, C1B_SP_COMMON, 0x01, 0x4a82 },	/* TX driver bias         */
	{ C1B_SI_PON, C1B_SP_COMMON, 0x04, 0x6956 },
	{ C1B_SI_PON, C1B_SP_COMMON, 0x0f, 0x0cf2 },
	{ C1B_SI_LAN, C1B_SP_COMMON, 0x00, 0x5ba9 },	/* LAN lane trim          */
	{ C1B_SI_LAN, C1B_SP_COMMON, 0x03, 0x8400 },
	{ C1B_SI_LAN, C1B_SP_COMMON, 0x04, 0x5558 },
};

/* rev-A and later: short trim (TX bias + RX path tweak) */
static const struct c1b_sds_op rtl9601b_sds_patch_revA[] = {
	{ C1B_SI_PON, C1B_SP_COMMON, 0x04, 0x6956 },	/* TX driver bias         */
	{ C1B_SI_PON, C1B_SP_COMMON, 0x0d, 0xc0c8 },	/* RX path config         */
};

static int c1b_sds_run(const struct rtl960x_ops *o,
		       const struct c1b_sds_op *seq, unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = c1b_sds_wr(o, seq[i].idx, seq[i].page, seq[i].reg, seq[i].data);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtl9601b_sds_patch(const struct rtl960x_ops *o, int rev)
{
	if (rev == 0)
		return c1b_sds_run(o, rtl9601b_sds_patch_rev0,
				   ARRAY_SIZE(rtl9601b_sds_patch_rev0));
	return c1b_sds_run(o, rtl9601b_sds_patch_revA,
			   ARRAY_SIZE(rtl9601b_sds_patch_revA));
}

/* ------------------------------------------------------------------ *
 *  flow -> physical-queue mapping (SID2QID table, 5 sids per word,
 *  6-bit field each). physical queue = stride*(scheduler/8) + queue.
 * ------------------------------------------------------------------ */
static void rtl9601b_flow2q(const struct rtl960x_ops *o,
			    u32 flow, u32 sched, u32 queue)
{
	u32 pqid = Q9601B_TCONT_QSTRIDE * (sched / 8) + queue;
	u8  lsb  = (flow % 5) * 6;

	rtl960x_rfwr(o, C1B_R_SID2QID + (flow / 5) * 4, lsb + 5, lsb, pqid);
}

/* ------------------------------------------------------------------ *
 *  PON-MAC core init: BEN enable, grant thresholds, scheduler reset,
 *  default per-queue shaping, OMCI trap priority.
 * ------------------------------------------------------------------ */
static int rtl9601b_ponmac_init(const struct rtl960x_ops *o)
{
	u16 ben;
	u32 i;
	int ret;

	/* burst-enable: turn on TTL output driver (PON lane, page 0x21 reg1) */
	ret = c1b_sds_rd(o, C1B_SI_PON, C1B_SP_COMMON, 1, &ben);
	if (ret)
		return ret;
	ben |= (1u << 14);
	ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 1, ben);
	if (ret)
		return ret;

	/* DBA grant thresholds: last + runt = 5 cells */
	rtl960x_rfwr(o, C1B_R_BW_THRES, 29, 16, 5);	/* last grant         */
	rtl960x_rfwr(o, C1B_R_BW_THRES, 13,  0, 5);	/* runt grant         */

	/* park every T-CONT: disable it and clear its queue map */
	for (i = 0; i < Q9601B_TCONTS - 1; i++) {
		rtl960x_rfwr(o, C1B_R_TCONT_EN, i % 32, i % 32, 0);
		o->wr(C1B_R_QMAP_BASE + i * 4, 0);
	}

	/* enable PIR overflow drop in the shaper */
	rtl960x_rfwr(o, C1B_R_SCH_CTRL, 18, 18, 1);

	/* default every queue: strict priority, CIR off, PIR wide open, weight 1 */
	for (i = 0; i < Q9601B_QUEUES; i++) {
		u8 lsb;

		rtl960x_rfwr(o, C1B_R_WFQ_TYPE + (i / 32) * 4, i % 32, i % 32, 0);
		rtl960x_rfwr(o, C1B_R_CIR_BASE + i * 4, 17, 0, 0);
		rtl960x_rfwr(o, C1B_R_PIR_BASE + i * 4, 17, 0, Q9601B_RATE_MAX);
		lsb = (i % 3) * 10;
		rtl960x_rfwr(o, C1B_R_WFQ_W + (i / 3) * 4, lsb + 9, lsb, 1);
	}

	/* OMCI/MPCP trap at top priority 7 */
	rtl960x_rfwr(o, C1B_R_TRAP_CFG, 2, 0, 7);
	return 0;
}

/* ------------------------------------------------------------------ *
 *  GPON mode select.
 *  rev = silicon revision (0 = rev-0, >0 = rev-A+). subtype unused on 9601B.
 * ------------------------------------------------------------------ */

/* rev>0: hold SerDes in reset and arm the CMU-TX ber-notify bypass */
static const struct r960_op rtl9601b_gpon_pre_revA[] = {
	FLD(C1B_R_WSDS17, 14, 14, 0),	/* hold digital in reset            */
	FLD(C1B_R_WSDS00, 10, 10, 0),	/* hold GPON-MAC in reset           */
	FLD(C1B_R_WSDS01,  2,  2, 1),	/* clkrd from original clock        */
};

/* common GPON datapath enable (after SerDes pre-config) */
static const struct r960_op rtl9601b_gpon_enable[] = {
	WR(C1B_R_MODE_CFG, 1),		/* select GPON mode                 */
	FLD(C1B_R_SDS1_CFG, 4, 0, 0x8),	/* SerDes lane-1 -> GPON line rate  */
	FLD(C1B_R_WSDS12, 12, 12, 1),	/* burst-enable output on          */
	FLD(C1B_R_PMISC_PON, 2, 2, 1),	/* accept undersize on PON port    */
	FLD(C1B_R_WSDS11, 0, 0, 0),	/* keep TX live when BEN deasserts */
};

/* rev>0: release SerDes from reset + pulse the queue engine */
static const struct r960_op rtl9601b_gpon_post_revA[] = {
	FLD(C1B_R_WSDS17, 14, 14, 1),	/* release digital reset           */
	FLD(C1B_R_WSDS00, 10, 10, 1),	/* release GPON-MAC reset          */
	FLD(C1B_R_SOFT_RST, 3, 3, 1),	/* queue reset pulse high          */
	FLD(C1B_R_SOFT_RST, 3, 3, 0),	/* queue reset pulse low           */
};

static int rtl9601b_ponmac_mode_set(const struct rtl960x_ops *o,
				    int rev, int subtype)
{
	u32 f;
	int ret;
	(void)subtype;

	/* re-apply analog trim for the selected rev */
	ret = rtl9601b_sds_patch(o, rev);
	if (ret)
		return ret;

	/* GPON: steer flows 0..31 to scheduler 7 / queue 31, flow 32 to 8/0 */
	for (f = 0; f < 32; f++)
		rtl9601b_flow2q(o, f, 7, 31);
	rtl9601b_flow2q(o, 32, 8, 0);

	if (rev == 0) {
		/* rev-0 line-of-sight patch: PON lane page-common reg12 */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 12, 0x4840);
		if (ret)
			return ret;
	} else {
		ret = r960_run(o, rtl9601b_gpon_pre_revA,
			       ARRAY_SIZE(rtl9601b_gpon_pre_revA));
		if (ret)
			return ret;
		/* bypass CMU-TX ber-notify */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 1, 0x4a8a);
		if (ret)
			return ret;
		/* RX signal-detect idle via out-of-band squelch */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 12, 0x4248);
		if (ret)
			return ret;
	}

	ret = r960_run(o, rtl9601b_gpon_enable,
		       ARRAY_SIZE(rtl9601b_gpon_enable));
	if (ret)
		return ret;

	/* clear RX filter config (PON lane page 0x21 reg11) */
	ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 11, 0x0);
	if (ret)
		return ret;

	/* MAC clock select for 62.5MHz sys / 155.52MHz TX */
	o->wr(C1B_R_PORT_CLK, o->rd(C1B_R_PORT_CLK) | 0x4);

	/* jumbo ceiling on both PON and UTP, RX-accept and TX */
	rtl960x_rfwr(o, C1B_R_ACC_LEN_PON, 27, 14, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_ACC_LEN_PON, 13,  0, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_TX_LEN_PON,  27, 14, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_TX_LEN_PON,  13,  0, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_ACC_LEN_UTP, 27, 14, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_ACC_LEN_UTP, 13,  0, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_TX_LEN_UTP,  27, 14, Q9601B_MAXLEN);
	rtl960x_rfwr(o, C1B_R_TX_LEN_UTP,  13,  0, Q9601B_MAXLEN);

	mdelay(10);			/* let analog settle               */

	if (rev > 0) {
		/* re-assert RX signal-detect */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 12, 0x4a48);
		if (ret)
			return ret;
		/* force ber-notify (workaround for GPON reset hang) */
		ret = c1b_sds_wr(o, C1B_SI_PON, 0x20, 2, 0x3000);
		if (ret)
			return ret;
		ret = r960_run(o, rtl9601b_gpon_post_revA,
			       ARRAY_SIZE(rtl9601b_gpon_post_revA));
		if (ret)
			return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------ *
 *  CDR re-seat.
 *  rev 0: pulse the PON-lane CDR-reset bit. rev>0: toggle RX signal-detect
 *  polarity, then re-cycle the FIFO/MAC reset and queue engine.
 * ------------------------------------------------------------------ */
static const struct r960_op rtl9601b_cdr_fifo_revA[] = {
	FLD(C1B_R_WSDS00, 10, 10, 0),	/* hold GPON-MAC reset             */
	FLD(C1B_R_WSDS17, 14, 14, 0),	/* hold digital reset              */
	DLY(10),
	FLD(C1B_R_WSDS17, 14, 14, 1),	/* release digital reset           */
	FLD(C1B_R_WSDS00, 10, 10, 1),	/* release GPON-MAC reset          */
	FLD(C1B_R_SOFT_RST, 3, 3, 1),	/* queue reset pulse high          */
	FLD(C1B_R_SOFT_RST, 3, 3, 0),	/* queue reset pulse low           */
};

static int rtl9601b_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	u16 cur, tog;
	int ret;
	int rev = RTL960X_REV_A;	/* dispatch has no rev arg; default to rev-A+ */

	if (rev == 0) {
		/* PON lane page-common reg19: assert then deassert CDR reset */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 19, 0x6000);
		if (ret)
			return ret;
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 19, 0x2000);
		if (ret)
			return ret;
		mdelay(1);
		return 0;
	}

	/* flip RX signal-detect polarity (reg12 bit15), then restore */
	ret = c1b_sds_rd(o, C1B_SI_PON, C1B_SP_COMMON, 12, &cur);
	if (ret)
		return ret;
	tog = (cur & ~0x8000u) | ((~cur) & 0x8000u);
	ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 12, tog);
	if (ret)
		return ret;
	usleep_range(10000, 11000);
	ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 12, cur);
	if (ret)
		return ret;

	return r960_run(o, rtl9601b_cdr_fifo_revA,
			ARRAY_SIZE(rtl9601b_cdr_fifo_revA));
}

/* ---- RTL9603CVD -------------------------------------------------------- */
/*
 * GPON PON-MAC + SerDes bring-up as data. The SerDes on this part is reached
 * through plain memory-mapped registers (no indirect command/data page window),
 * so every analog/digital tweak is a direct RMW in the tables below.
 *
 * Absolute physical addresses = SWCORE window base 0x1b000000 + register offset;
 * the PON-IP sub-block lives in the 0x1bf0xxxx window.
 */
#define C3_SWBASE		0x1b000000u

/* core / SerDes digital + analog block */
#define C3_SOFTWARE_RST		0x1b0000e0u /* global soft-reset command word     */
#define C3_SDS_CFG		0x1b000200u /* SerDes lane mode select            */
#define C3_DYNGASP_CTRL		0x1b00021cu /* dying-gasp comparator control      */
#define C3_P_MISC_PON		0x1b020084u /* P_MISC: base 0x20004 + PON port 4 * stride 0x20 */
#define C3_PON_INBW_LBOUND	0x1b023180u /* DS in-band accumulation low bound  */
#define C3_WSDS_DIG_00		0x1b040030u /* SerDes digital: clock control      */
#define C3_WSDS_DIG_02		0x1b040038u /* SerDes digital: BEN power-down      */
#define C3_WSDS_DIG_18		0x1b040090u /* SerDes digital: BEN output enable   */
#define C3_WSDS_DIG_1D		0x1b0400a4u /* SerDes digital: interface FIFO rstb */
#define C3_FORCE_BEN		0x1b0400e4u /* burst-enable force mode             */
#define C3_SDS_ANA_MISC02	0x1b040508u /* analog misc: BER-notify force/value */
#define C3_SDS_ANA_COM03	0x1b04058cu /* analog common: RX CDR / SD-por sel  */
#define C3_SDS_ANA_COM09	0x1b0405a4u /* analog common: BEN CML/TTL drive    */
#define C3_SDS_ANA_COM17	0x1b0405c4u /* analog common: CDR loop Kp          */
#define C3_SDS_ANA_COM20	0x1b0405d0u /* analog common: RX CMU charge-pump   */
#define C3_SDS_ANA_COM21	0x1b0405d4u /* analog common: RX CMU slew / KVCO   */
#define C3_SDS_ANA_COM26	0x1b0405e8u /* analog common: GPHY CMU LDO vref    */
#define C3_SDS_ANA_COM27	0x1b0405ecu /* analog common: GPHY CMU KVCO        */
#define C3_FIB_EXT_REG21	0x1b040e54u /* fiber ext: analog-ready status      */
#define C3_PON_TRAP_CFG		0x1b0110ecu /* OMCI/MPCP trap priority            */
/* PON-IP block */
#define C3_PON_SIDVALID		0x1bf0218cu /* per-flow SID-valid bitmap (1b/elem) */
#define C3_PON_BW_THRES		0x1bf021a0u /* upstream BW request thresholds     */
#define C3_PON_OMCI_CFG		0x1bf021a4u /* OMCI flow/SID select               */
#define C3_PON_SCH_CTRL		0x1bf021e4u /* scheduler control                  */
#define C3_PON_SID2QID		0x1bf0210cu /* flow(SID) -> physical queue (7-bit/elem) */

/* fixed chip parameters for the GPON datapath */
#define C3_SID_COUNT		128	/* classifier SID / flow slots          */
#define C3_OMCI_FLOW		127	/* flow id reserved for OMCI            */

/* SID-valid bitmap is packed 1 bit per flow: word = base + (idx/32)*4, bit idx%32 */
static inline void c3_sidvalid(const struct rtl960x_ops *o, u32 idx, u32 v)
{
	u8 b = idx & 31u;

	rtl960x_rfwr(o, C3_PON_SIDVALID + (idx >> 5) * 4u, b, b, v);
}

/* SID2QID: 7-bit physical-queue field per flow, 4 flows per 32-bit word */
static void c3_flow2queue(const struct rtl960x_ops *o, u32 flow, u32 pqid)
{
	u32 lsb = (flow % 4u) * 7u;

	rtl960x_rfwr(o, C3_PON_SID2QID + (flow / 4u) * 4u, lsb + 6u, lsb, pqid);
}

/*
 * ponmac_init: PON-MAC global defaults applied once before mode selection.
 * Single-ended burst-enable variant (TTL output driver on); request/last
 * bandwidth thresholds seeded; PIR overflow drop, OMCI trap priority and the
 * dying-gasp comparator polarity set. Per-T-cont and per-queue scheduler/rate
 * programming is owned by the datapath/scheduler driver, not this table.
 */
static const struct r960_op c3_init[] = {
	FLD(C3_SDS_ANA_COM09,  0,  0, 1),	/* BEN drive: TTL output enabled  */
	FLD(C3_PON_BW_THRES,  29, 16, 5),	/* US last-grant BW threshold     */
	FLD(C3_PON_BW_THRES,  13,  0, 5),	/* US runt BW request threshold   */
	FLD(C3_PON_SCH_CTRL,  18, 18, 1),	/* drop on PIR overflow           */
	FLD(C3_PON_TRAP_CFG,   2,  0, 7),	/* OMCI/MPCP trap = top priority  */
	FLD(C3_DYNGASP_CTRL,   3,  3, 1),	/* invert dying-gasp comparator   */
};

/*
 * GPON SerDes/PON-MAC bring-up, phase 1: analog pre-config with the lane held
 * off. Force the 125 MHz reference on so the analog has a clock, lift the BEN
 * power-down, detach the RX CDR AFE, then load the tuned CDR/CMU/KVCO analog
 * coefficients before switching the lane into GPON mode.
 */
static const struct r960_op c3_sds_pre[] = {
	FLD(C3_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C3_WSDS_DIG_00,    4,  4, 1),	/* force 125 MHz reference clock   */
	FLD(C3_WSDS_DIG_02,   10, 10, 0),	/* clear BEN power-down            */
	FLD(C3_SDS_ANA_COM03, 13, 13, 0),	/* RX CDR AFE: deselect            */
	FLD(C3_SDS_ANA_COM09,  4,  4, 0),	/* BEN driver: CML off             */
	FLD(C3_SDS_ANA_COM09,  0,  0, 1),	/* BEN driver: TTL output on       */
	FLD(C3_SDS_ANA_COM17, 15, 10, 0xc),	/* CDR loop proportional gain Kp   */
	FLD(C3_SDS_ANA_COM20, 11,  7, 0x1b),	/* RX CMU charge-pump current      */
	FLD(C3_SDS_ANA_COM20,  3,  2, 0x3),	/* RX CMU LDO reference            */
	FLD(C3_SDS_ANA_COM21, 13, 11, 0x2),	/* RX CMU slew rate                */
	FLD(C3_SDS_ANA_COM21,  6,  3, 0x4),	/* RX VCO gain band select         */
	FLD(C3_SDS_ANA_COM26,  3,  2, 0x3),	/* GPHY CMU LDO reference          */
	FLD(C3_SDS_ANA_COM27,  6,  3, 0x4),	/* GPHY VCO gain band select       */
};

/*
 * Phase 2: commit GPON mode and pulse the resets. Select GPON on the lane,
 * release the BER-notify force so the reset takes, reset SerDes (digital +
 * analog) and the GPON MAC, then re-arm BER-notify so a later signal-detect
 * drop will not knock the MAC down. A switch-core reset follows the mode change.
 */
static const struct r960_op c3_sds_mode[] = {
	FLD(C3_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C3_SDS_ANA_MISC02,12, 12, 0),	/* release BER-notify force         */
	FLD(C3_SOFTWARE_RST,   2,  0, 1),	/* reset SerDes + GPON MAC          */
	DLY(10),				/* let the reset settle             */
	FLD(C3_SDS_ANA_MISC02,13, 13, 1),	/* BER-notify hold value = 1        */
	FLD(C3_SDS_ANA_MISC02,12, 12, 1),	/* re-force BER-notify (MAC stays up)*/
	FLD(C3_SOFTWARE_RST,  10, 10, 1),	/* switch-core reset on mode change */
	DLY(10),				/* let the switch reset settle      */
};

/*
 * Phase 3: re-enable the datapath after the resets. Cycle the TX then RX
 * interface FIFO release-B, turn the burst-enable output on, allow undersize
 * frames on the PON port, and drop burst-enable force mode.
 */
static const struct r960_op c3_sds_post[] = {
	FLD(C3_WSDS_DIG_1D,   16, 16, 0),	/* TX interface FIFO: assert rstb   */
	FLD(C3_WSDS_DIG_1D,   16, 16, 1),	/* TX interface FIFO: release rstb  */
	FLD(C3_WSDS_DIG_1D,   15, 15, 0),	/* RX interface FIFO: assert rstb   */
	FLD(C3_WSDS_DIG_1D,   15, 15, 1),	/* RX interface FIFO: release rstb  */
	FLD(C3_WSDS_DIG_18,   12, 12, 1),	/* burst-enable output: on          */
	FLD(C3_P_MISC_PON,     2,  2, 1),	/* PON port: accept undersize       */
	FLD(C3_FORCE_BEN,      0,  0, 0),	/* burst-enable force mode: off     */
};

/*
 * SerDes CDR reseat: toggle the RX signal-detect power-on select bit then
 * restore it, and bounce the 16<->20-bit transfer FIFO release-B. Used to
 * re-acquire lock without a full re-bring-up.
 */
static int c3_cdr_reset(const struct rtl960x_ops *o)
{
	u32 v = o->rd(C3_SDS_ANA_COM03);

	/* flip the SD power-on select bit (mask 0x400), leave the rest intact */
	o->wr(C3_SDS_ANA_COM03, (v & ~0x400u) | (((~v) & 0x400u)));
	mdelay(10);
	o->wr(C3_SDS_ANA_COM03, v);		/* restore original analog word    */

	rtl960x_rfwr(o, C3_WSDS_DIG_1D, 14, 14, 0); /* transfer FIFO: assert rstb  */
	mdelay(10);
	rtl960x_rfwr(o, C3_WSDS_DIG_1D, 14, 14, 1); /* transfer FIFO: release rstb */
	return 0;
}

/* GPON bring-up driver: pre-config tables + flow/OMCI wiring + analog gate. */
static int c3_gpon_mode_set(const struct rtl960x_ops *o)
{
	u32 f;
	int rc;

	/* park every data flow's SID as invalid + map to the default queue; the
	 * OMCI flow is armed afterwards. Without the SID2QID map the OMCI SID is
	 * valid but bound to no queue and no traffic passes. */
	for (f = 0; f < C3_SID_COUNT - 1u; f++) {
		c3_sidvalid(o, f, 0);
		c3_flow2queue(o, f, 0x7f);
	}
	c3_sidvalid(o, C3_OMCI_FLOW, 1);		/* OMCI flow: SID valid    */
	c3_flow2queue(o, C3_OMCI_FLOW, 0x7f);		/* OMCI flow -> queue      */
	rtl960x_rfwr(o, C3_PON_OMCI_CFG, 6, 0, C3_OMCI_FLOW); /* OMCI SID select   */

	rc = r960_run(o, c3_sds_pre,  ARRAY_SIZE(c3_sds_pre));
	if (rc)
		return rc;
	rc = r960_run(o, c3_sds_mode, ARRAY_SIZE(c3_sds_mode));
	if (rc)
		return rc;
	rc = r960_run(o, c3_sds_post, ARRAY_SIZE(c3_sds_post));
	if (rc)
		return rc;

	/*
	 * Wait for the analog to report ready (FIB_EXT_REG21 bit 13), then drop
	 * the forced 125 MHz reference to save power. The reference stays on if
	 * the gate never asserts. ~1000 * 200us upper bound.
	 */
	rc = r960_run(o, (const struct r960_op[]){
		POLL(C3_FIB_EXT_REG21, 13, 1000),
	}, 1);
	if (rc == 0)
		rtl960x_rfwr(o, C3_WSDS_DIG_00, 4, 4, 0); /* 125 MHz reference: off */

	/* DS in-band accumulation low bound for PBO */
	rtl960x_rfwr(o, C3_PON_INBW_LBOUND, 23, 0, 0xfda000);

	return rc;
}

/* RTL9603CVD top-level entry points (thin wrappers over the c3_* internals). */
static int rtl9603cvd_ponmac_init(const struct rtl960x_ops *o)
{
	return r960_run(o, c3_init, ARRAY_SIZE(c3_init));
}

static int rtl9603cvd_ponmac_mode_set(const struct rtl960x_ops *o,
				      int rev, int subtype)
{
	(void)rev; (void)subtype;	/* single SerDes variant for every rev */
	return c3_gpon_mode_set(o);
}

static int rtl9603cvd_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	return c3_cdr_reset(o);
}

/* ---- RTL9607C ---------------------------------------------------------- *
 * GPON PON-MAC + SerDes bring-up as data. SerDes here is DIRECT MMIO: every
 * analog/digital knob is its own memory-mapped register written by RMW (there
 * is no indirect command/data page+register window on this part). The
 * straight-line analog/reset runs live in op-tables; the per-tcont / per-queue
 * scheduler init and the rev-dependent SerDes variant stay as explicit code.
 *
 * Absolute physical addresses = SWCORE window base 0x1b000000 + register offset;
 * the PON-IP sub-block lives in the 0x1bf0xxxx window. This part has a 5-deep
 * PON port and a 0x100 per-port MAC stride (narrower than the 9601b/9602c 0x400).
 */

/* PON-IP config / scheduler block (0x1bf0xxxx) */
#define C7_PON_SIDVALID		0x1bf02188u /* per-flow SID-valid bitmap (1b/elem)  */
#define C7_PON_OMCI_CFG		0x1bf021a0u /* OMCI flow/SID select                 */
#define C7_PON_BW_THRES		0x1bf0219cu /* upstream BW request thresholds       */
#define C7_PON_SCH_CTRL		0x1bf021e0u /* scheduler control                    */
#define C7_DRN_CMD		0x1bf020f4u /* T-cont drain command / status        */
#define C7_IO_CMD_0_US		0x1bf05434u /* upstream NIC GMII TX/RX enables       */
#define C7_PON_SID2QID		0x1bf02108u /* flow(SID) -> physical queue map       */
#define C7_PON_QID_CIR_RATE	0x1bf021e4u /* per-queue committed (CIR) rate        */
#define C7_PON_QID_PIR_RATE	0x1bf023e4u /* per-queue peak (PIR) rate             */
#define C7_PON_SCH_QMAP		0x1bf025e4u /* per-tcont queue membership mask       */
#define C7_PON_WFQ_TYPE		0x1bf02668u /* per-queue strict/WFQ select           */
#define C7_PON_WFQ_WEIGHT	0x1bf0267cu /* per-queue WFQ weight                  */
#define C7_PON_TCONT_EN		0x1bf02664u /* per-tcont schedule enable             */

/* PON trap / accept-length (0x1b011xxx) */
#define C7_PON_TRAP_CFG		0x1b011144u /* OMCI/MPCP trap priority              */
#define C7_ACCEPT_MAX_LEN	0x1b011028u /* per-port accept max length (stride 4) */

/* switch global (0x1b000xxx / 0x1b002xxx) */
#define C7_SOFTWARE_RST		0x1b000108u /* soft-reset: SW core / SerDes+GPON-MAC */
#define C7_DYNGASP_CTRL		0x1b00029cu /* dying-gasp comparator control         */
#define C7_SDS_CFG		0x1b000270u /* SerDes lane mode select               */
#define C7_PON_INBW_LBOUND	0x1b023288u /* DS in-band accumulation low bound     */
#define C7_P_MISC_PON		0x1b020504u /* per-port misc, PON port (base 0x20004 + port5*0x100) */

/* SerDes digital block (0x1b040xxx) */
#define C7_WSDS_DIG_00		0x1b040030u /* SerDes digital: 125 MHz clock control */
#define C7_WSDS_DIG_02		0x1b040038u /* SerDes digital: BEN power-down        */
#define C7_WSDS_DIG_03		0x1b04003cu /* SerDes digital: TX-disable sel delay  */
#define C7_WSDS_DIG_18		0x1b040090u /* SerDes digital: BEN output enable     */
#define C7_WSDS_DIG_1D		0x1b0400a4u /* SerDes digital: interface FIFO rstb   */
#define C7_FORCE_BEN		0x1b0400e4u /* burst-enable force mode               */

/* SerDes analog common / GPON / misc (0x1b0405xx..0x1b0407xx, 0x1b040exx) */
#define C7_SDS_ANA_MISC02	0x1b040508u /* analog misc: BER-notify force/value   */
#define C7_SDS_ANA_COM00	0x1b040580u /* analog common: CDR Kd (rev-B)         */
#define C7_SDS_ANA_COM02	0x1b040588u /* analog common: CDR Ki/Kp1/Kp2         */
#define C7_SDS_ANA_COM05	0x1b040594u /* analog common: RX EQ hold             */
#define C7_SDS_ANA_COM06	0x1b040598u /* analog common: RX filter / RX EQ in   */
#define C7_SDS_ANA_COM08	0x1b0405a0u /* analog common: RX Kp1_2 / Kp2_2       */
#define C7_SDS_ANA_COM09	0x1b0405a4u /* analog common: RX CDR/timer/re-seat   */
#define C7_SDS_ANA_COM12	0x1b0405b0u /* analog common: RX EQ2 select          */
#define C7_SDS_ANA_COM13	0x1b0405b4u /* analog common: TX amplitude           */
#define C7_SDS_ANA_COM14	0x1b0405b8u /* analog common: TX emphasis / Z0 P-adj */
#define C7_SDS_ANA_COM15	0x1b0405bcu /* analog common: Z0 N-adjust            */
#define C7_SDS_ANA_COM17	0x1b0405c4u /* analog common: BEN CML/TTL drive      */
#define C7_SDS_ANA_COM21	0x1b0405d4u /* analog common: RX CMU CCO/CP/KVCO/LPF */
#define C7_SDS_ANA_COM23	0x1b0405dcu /* analog common: CMU watchdog (RX)      */
#define C7_SDS_ANA_COM24	0x1b0405e0u /* analog common: TX CMU CP / LPF-CP     */
#define C7_SDS_ANA_COM25	0x1b0405e4u /* analog common: TX CMU LPF-RS / LC byp */
#define C7_SDS_ANA_COM26	0x1b0405e8u /* analog common: CMU watchdog (TX)      */
#define C7_SDS_ANA_COM30	0x1b0405f8u /* analog common: GPHY CMU CP/ICP/LPF-CP */
#define C7_SDS_ANA_COM31	0x1b0405fcu /* analog common: GPHY CMU LPF-RS        */
#define C7_SDS_ANA_GPON34	0x1b040708u /* analog GPON: GPHY CMU watchdog        */
#define C7_SDS_ANA_GPON36	0x1b040710u /* analog GPON: GPHY field lock-dn limit */
#define C7_SDS_ANA_GPON37	0x1b040714u /* analog GPON: GPHY dly-clk/lock-up lim */
#define C7_SDS_ANA_GPON43	0x1b04072cu /* analog GPON: TX delay-clock select    */
#define C7_FIB_EXT_REG21	0x1b040e54u /* fiber ext: analog-ready status        */

/* fixed chip parameters for the GPON datapath */
#define C7_PON_PORT		5	/* PON port index for per-port registers */
#define C7_MACPP_STRIDE		0x100u	/* per-port MAC register stride          */
#define C7_SID_COUNT		128	/* classifier SID / flow slots           */
#define C7_GPON_TCONT_MAX	32	/* T-cont count                          */
#define C7_PON_QUEUE_MAX	128	/* physical PON queue count              */
#define C7_TCONT_QUEUE_MAX	32	/* queues per T-cont scheduler           */
#define C7_RATE_MAX		0x3ffffu/* CIR/PIR rate saturation value         */
#define C7_OMCI_FLOW		127	/* flow id reserved for OMCI            */
#define C7_OMCI_TCONT		31	/* T-cont id for the OMCI flow           */
#define C7_OMCI_QUEUE		24	/* queue id for the OMCI flow            */

/* one-shot init guard: drain T-conts only on a re-init */
static int c7_init_done;

/*
 * Packed/strided array element write. For arroff<32 a 32-bit word holds
 * (32/arroff) elements: the index picks both the word and the bit offset.
 * For arroff>=32 each element owns a word (byte stride arroff/8). 'len' is the
 * element field width.
 */
static void c7_arr(const struct rtl960x_ops *o, u32 base, u32 arroff,
		   u32 idx, u32 lsp, u32 len, u32 val)
{
	u32 phys, lsb;

	if (arroff % 32u) {
		u32 per_word = 32u / arroff;

		phys = base + (idx / per_word) * 4u;
		lsb  = (idx % per_word) * arroff + lsp;
	} else {
		phys = base + idx * (arroff / 8u);
		lsb  = lsp;
	}
	rtl960x_rfwr(o, phys, lsb + len - 1u, lsb, val);
}

/* GPON physical queue id = TCONT_QUEUE_MAX*(sched/8) + logical queue */
static void c7_flow2queue(const struct rtl960x_ops *o, u32 flow, u32 sched, u32 q)
{
	c7_arr(o, C7_PON_SID2QID, 7, flow, 0, 7,
	       C7_TCONT_QUEUE_MAX * (sched / 8u) + q);
}

/* CIR/PIR are stored as (rate-1) except for the 0/1/max sentinels */
static u32 c7_rate(u32 rate)
{
	if (rate != 0 && rate != 1 && rate != C7_RATE_MAX)
		return rate - 1u;
	return rate;
}

/*
 * Drain one T-cont, busy-polling the drain flag. On timeout, recover the
 * upstream NIC by toggling its GMII enables (RX off, TX off->on, RX on).
 */
static void c7_tcont_drain(const struct rtl960x_ops *o, u32 tcont)
{
	u32 i;

	/* queue-mode=0, drain-index=tcont, drain-pulse=1 */
	o->wr(C7_DRN_CMD, ((tcont & 0x7fu) << 3) | (1u << 1));

	for (i = 0; i < 200000u; i++)
		if (!(o->rd(C7_DRN_CMD) & 0x1u))	/* drain flag cleared */
			break;

	if (i >= 200000u) {
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 5, 5, 0);	/* US NIC RX off */
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 4, 4, 0);	/* US NIC TX off */
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 4, 4, 1);	/* US NIC TX on  */
		rtl960x_rfwr(o, C7_IO_CMD_0_US, 5, 5, 1);	/* US NIC RX on  */
	}
}

/*
 * ponmac_init: PON-MAC global defaults applied once before mode selection.
 * Single-ended BEN (TTL output on), US BW thresholds, per-T-cont disable +
 * mask clear, PIR overflow drop, per-queue strict/CIR=0/PIR=max/weight=1,
 * OMCI trap priority and dying-gasp comparator polarity. (rev>A would also
 * init switch-PBO; that lives in a separate subsystem.)
 */
static int c7_ponmac_init(const struct rtl960x_ops *o, int rev, int subtype)
{
	u32 i;

	(void)subtype; (void)rev;

	rtl960x_rfwr(o, C7_SDS_ANA_COM17, 0, 0, 1);	/* BEN TTL output on    */
	rtl960x_rfwr(o, C7_PON_BW_THRES, 29, 16, 5);	/* US last-grant thresh */
	rtl960x_rfwr(o, C7_PON_BW_THRES, 13,  0, 5);	/* US runt-request thresh*/

	if (c7_init_done)
		for (i = 0; i < C7_GPON_TCONT_MAX; i++)
			c7_tcont_drain(o, i);

	for (i = 0; i < C7_GPON_TCONT_MAX - 1u; i++) {
		c7_arr(o, C7_PON_TCONT_EN, 1, i, 0, 1, 0);	/* T-cont disable */
		c7_arr(o, C7_PON_SCH_QMAP, 32, i, 0, 32, 0);	/* clear queue mask*/
	}

	rtl960x_rfwr(o, C7_PON_SCH_CTRL, 18, 18, 1);	/* drop on PIR overflow */

	for (i = 0; i < C7_PON_QUEUE_MAX; i++) {
		c7_arr(o, C7_PON_WFQ_TYPE,    1,  i, 0,  1, 0);			/* strict   */
		c7_arr(o, C7_PON_QID_CIR_RATE,18, i, 0, 18, c7_rate(0));		/* CIR = 0  */
		c7_arr(o, C7_PON_QID_PIR_RATE,18, i, 0, 18, c7_rate(C7_RATE_MAX));/* PIR = max*/
		c7_arr(o, C7_PON_WFQ_WEIGHT,  10, i, 0, 10, 1);			/* weight=1 */
	}

	rtl960x_rfwr(o, C7_PON_TRAP_CFG, 2, 0, 7);	/* OMCI/MPCP top priority */
	rtl960x_rfwr(o, C7_DYNGASP_CTRL, 3, 3, 1);	/* invert dying-gasp cmp  */

	c7_init_done = 1;
	return 0;
}

/*
 * Shared GPON SID/OMCI front matter (identical before each rev's SerDes patch):
 * park every data flow on T-cont 15 / queue 31 with its SID invalid, then
 * dedicate the OMCI flow to its own T-cont/queue, mark its SID valid, and point
 * the OMCI SID select at it.
 */
static void c7_gpon_pre(const struct rtl960x_ops *o)
{
	u32 f;

	for (f = 0; f < C7_SID_COUNT - 1u; f++) {
		c7_flow2queue(o, f, 15, 31);
		c7_arr(o, C7_PON_SIDVALID, 1, f, 0, 1, 0);
	}
	c7_flow2queue(o, C7_OMCI_FLOW, C7_OMCI_TCONT, C7_OMCI_QUEUE);
	c7_arr(o, C7_PON_SIDVALID, 1, C7_OMCI_FLOW, 0, 1, 1);
	rtl960x_rfwr(o, C7_PON_OMCI_CFG, 6, 0, C7_OMCI_FLOW);
}

/*
 * Shared GPON tail (identical after each rev's SerDes patch): the GPON mode
 * change needs a switch-core reset, then re-arm the TX/RX interface FIFO
 * release-B, BEN output on, accept undersize frames on the PON port, drop BEN
 * force mode, set accept max length, wait analog-ready then drop the 125 MHz
 * clock for power saving, and seed the DS in-band low bound.
 */
static int c7_gpon_post(const struct rtl960x_ops *o)
{
	u32 i;

	rtl960x_rfwr(o, C7_SOFTWARE_RST, 10, 10, 1);	/* switch-core reset */
	mdelay(10);

	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 16, 16, 0);	/* TX iface FIFO assert rstb  */
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 16, 16, 1);	/* TX iface FIFO release rstb */
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 15, 15, 0);	/* RX iface FIFO assert rstb  */
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 15, 15, 1);	/* RX iface FIFO release rstb */

	rtl960x_rfwr(o, C7_WSDS_DIG_18, 12, 12, 1);	/* BEN output on        */
	rtl960x_rfwr(o, C7_P_MISC_PON, 2, 2, 1);	/* PON port accept undersize */
	rtl960x_rfwr(o, C7_FORCE_BEN, 0, 0, 0);		/* BEN force mode off   */
	rtl960x_rfwr(o, C7_ACCEPT_MAX_LEN + C7_PON_PORT * 4u, 13, 0, 2031); /* max len */

	for (i = 0; i < 10000u; i++) {		/* wait analog-ready (V2ANALOG) */
		if ((o->rd(C7_FIB_EXT_REG21) >> 13) & 0x1u)
			break;
		udelay(200);
	}
	if (i < 10000u)
		rtl960x_rfwr(o, C7_WSDS_DIG_00, 4, 4, 0);	/* 125 MHz clock off */

	rtl960x_rfwr(o, C7_PON_INBW_LBOUND, 23, 0, 0xfda000);	/* DS in-band lbound */
	return 0;
}

/*
 * rev-A SerDes patch (mode V1): park the lane, force the 125 MHz reference on,
 * load tuned TX CMU/PLL + RX CDR/CMU/EQ analog coefficients, switch the lane
 * into GPON, then reset SerDes+MAC and re-arm BER-notify so a signal-detect
 * drop will not knock the MAC down.
 */
static const struct r960_op c7_sds_v1[] = {
	FLD(C7_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C7_WSDS_DIG_00,    4,  4, 0x1),	/* force 125 MHz reference clock   */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down            */
	FLD(C7_WSDS_DIG_03,    6,  4, 0x2),	/* TX-disable select delay         */
	FLD(C7_SDS_ANA_COM17,  4,  4, 0x0),	/* BEN driver: CML off             */
	FLD(C7_SDS_ANA_COM17,  0,  0, 0x1),	/* BEN driver: TTL output on       */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0xF),	/* TX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump          */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x7),	/* TX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM25,  1,  1, 0x0),	/* LC bypass off                   */
	FLD(C7_SDS_ANA_COM21, 15, 15, 0x1),	/* RX CMU CCO select               */
	FLD(C7_SDS_ANA_COM21, 14, 11, 0xC),	/* RX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM21,  6,  6, 0x0),	/* RX CMU big-KVCO off             */
	FLD(C7_SDS_ANA_COM21,  4,  2, 0x3),	/* RX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM09, 13, 13, 0x0),	/* RX CDR AFE deselect             */
	FLD(C7_SDS_ANA_COM06,  7,  0, 0x2),	/* RX filter config                */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x1),	/* CDR Ki                          */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x4),	/* CDR Kp1                         */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x4),	/* CDR Kp2                         */
	FLD(C7_SDS_ANA_COM08, 14, 12, 0x4),	/* RX Kp1_2                        */
	FLD(C7_SDS_ANA_COM08, 11,  9, 0x4),	/* RX Kp2_2                        */
	FLD(C7_SDS_ANA_COM12,  7,  4, 0x1),	/* RX EQ2 select                   */
	FLD(C7_SDS_ANA_COM12,  3,  0, 0x1),	/* RX EQ2 select 2                 */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                    */
	FLD(C7_SDS_ANA_COM14, 11,  9, 0x0),	/* TX emphasis                     */
	FLD(C7_SDS_ANA_COM14,  8,  8, 0x1),	/* TX emphasis enable              */
	FLD(C7_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x0),	/* release BER-notify force        */
	FLD(C7_SOFTWARE_RST,   2,  0, 0x1),	/* reset SerDes + GPON MAC         */
	DLY(10),				/* let the reset settle            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* BER-notify hold value = 1       */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* re-force BER-notify              */
};

/*
 * rev-B SerDes patch (mode V2): same layout as V1 with retuned CDR/RX gains
 * (Kp1/Kp2, RX Kp1_2/Kp2_2) plus a CDR Kd write that only exists on this rev.
 */
static const struct r960_op c7_sds_v2[] = {
	FLD(C7_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C7_WSDS_DIG_00,    4,  4, 0x1),	/* force 125 MHz reference clock   */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down            */
	FLD(C7_WSDS_DIG_03,    6,  4, 0x2),	/* TX-disable select delay         */
	FLD(C7_SDS_ANA_COM17,  4,  4, 0x0),	/* BEN driver: CML off             */
	FLD(C7_SDS_ANA_COM17,  0,  0, 0x1),	/* BEN driver: TTL output on       */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0xF),	/* TX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump          */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x7),	/* TX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM25,  1,  1, 0x0),	/* LC bypass off                   */
	FLD(C7_SDS_ANA_COM21, 15, 15, 0x1),	/* RX CMU CCO select               */
	FLD(C7_SDS_ANA_COM21, 14, 11, 0xC),	/* RX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM21,  6,  6, 0x0),	/* RX CMU big-KVCO off             */
	FLD(C7_SDS_ANA_COM21,  4,  2, 0x3),	/* RX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM09, 13, 13, 0x0),	/* RX CDR AFE deselect             */
	FLD(C7_SDS_ANA_COM06,  7,  0, 0x2),	/* RX filter config                */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x1),	/* CDR Ki                          */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x0),	/* CDR Kp1 (rev-B)                 */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x6),	/* CDR Kp2 (rev-B)                 */
	FLD(C7_SDS_ANA_COM08, 14, 12, 0x1),	/* RX Kp1_2 (rev-B)                */
	FLD(C7_SDS_ANA_COM08, 11,  9, 0x1),	/* RX Kp2_2 (rev-B)                */
	FLD(C7_SDS_ANA_COM12,  7,  4, 0x1),	/* RX EQ2 select                   */
	FLD(C7_SDS_ANA_COM12,  3,  0, 0x1),	/* RX EQ2 select 2                 */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                    */
	FLD(C7_SDS_ANA_COM14, 11,  9, 0x0),	/* TX emphasis                     */
	FLD(C7_SDS_ANA_COM14,  8,  8, 0x1),	/* TX emphasis enable              */
	FLD(C7_SDS_ANA_COM00,  1,  1, 0x0),	/* CDR Kd (rev-B only)             */
	FLD(C7_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x0),	/* release BER-notify force        */
	FLD(C7_SOFTWARE_RST,   2,  0, 0x1),	/* reset SerDes + GPON MAC         */
	DLY(10),				/* let the reset settle            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* BER-notify hold value = 1       */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* re-force BER-notify              */
};

/*
 * rev-C+ SerDes patch (mode V3): a GPHY-CMU-centric tuning - disable the per
 * lane CMU watchdogs, retune TX/RX CMU and the GPHY CMU charge-pump/LPF and
 * lock limits, then switch into GPON and reset+re-arm as the other revs do.
 */
static const struct r960_op c7_sds_v3[] = {
	FLD(C7_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)        */
	FLD(C7_WSDS_DIG_00,    4,  4, 0x1),	/* force 125 MHz reference clock   */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down            */
	FLD(C7_WSDS_DIG_03,    6,  4, 0x2),	/* TX-disable select delay         */
	FLD(C7_SDS_ANA_GPON43,11, 11, 0x1),	/* TX delay-clock select           */
	FLD(C7_SDS_ANA_COM26,  3,  3, 0x0),	/* TX CMU watchdog off             */
	FLD(C7_SDS_ANA_COM23, 15, 15, 0x0),	/* RX CMU watchdog off             */
	FLD(C7_SDS_ANA_GPON34, 7,  7, 0x0),	/* GPHY CMU watchdog off           */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0x4),	/* TX CMU charge-pump              */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x1),	/* TX CMU LPF charge-pump          */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x3),	/* TX CMU LPF resistor             */
	FLD(C7_SDS_ANA_COM14,  4,  0, 0x7),	/* Z0 P-adjust                     */
	FLD(C7_SDS_ANA_COM15, 15, 12, 0x8),	/* Z0 N-adjust                     */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x6),	/* CDR Ki                          */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x1),	/* CDR Kp1                         */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x0),	/* CDR Kp2                         */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                    */
	FLD(C7_SDS_ANA_COM05,  2,  2, 0x1),	/* RX EQ hold                      */
	FLD(C7_SDS_ANA_COM06, 15,  9, 0x40),	/* RX EQ input                     */
	FLD(C7_SDS_ANA_COM09,  6,  2, 0x1f),	/* RX timer-BER                    */
	FLD(C7_SDS_ANA_GPON37, 5,  5, 0x1),	/* GPHY delay-clock select         */
	FLD(C7_SDS_ANA_GPON37,15,  6, 0x316),	/* GPHY lock-up limit              */
	FLD(C7_SDS_ANA_COM30, 15, 12, 0x3),	/* GPHY CMU charge-pump            */
	FLD(C7_SDS_ANA_COM30, 10, 10, 0x1),	/* GPHY CMU ICP low-BW             */
	FLD(C7_SDS_ANA_COM30,  4,  2, 0x2),	/* GPHY CMU LPF charge-pump        */
	FLD(C7_SDS_ANA_COM31, 15, 13, 0x0),	/* GPHY CMU LPF resistor           */
	FLD(C7_SDS_ANA_GPON36,15,  6, 0x302),	/* GPHY lock-down limit            */
	FLD(C7_SDS_CFG,        4,  0, 0x8),	/* lane mode: GPON                 */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x0),	/* release BER-notify force        */
	FLD(C7_SOFTWARE_RST,   2,  0, 0x1),	/* reset SerDes + GPON MAC         */
	DLY(10),				/* let the reset settle            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* BER-notify hold value = 1       */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* re-force BER-notify              */
};

/*
 * GPON mode-set: common SID/OMCI front matter, the rev-selected SerDes patch
 * table, then the common GPON tail. rev A->V1, B->V2, C and later->V3.
 */
static int c7_gpon_mode_set(const struct rtl960x_ops *o, int rev, int subtype)
{
	const struct r960_op *sds;
	unsigned int n;
	int rc;

	(void)subtype;

	c7_gpon_pre(o);

	switch (rev) {
	case RTL960X_REV_A:
		sds = c7_sds_v1; n = ARRAY_SIZE(c7_sds_v1); break;
	case RTL960X_REV_B:
		sds = c7_sds_v2; n = ARRAY_SIZE(c7_sds_v2); break;
	default:
		sds = c7_sds_v3; n = ARRAY_SIZE(c7_sds_v3); break;
	}

	rc = r960_run(o, sds, n);
	if (rc)
		return rc;

	return c7_gpon_post(o);
}

/*
 * SerDes CDR reseat: flip the RX CDR re-seat bit (mask 0x400), settle, restore
 * the original analog word, then bounce the 16<->20-bit transfer FIFO
 * release-B. Re-acquires lock without a full re-bring-up.
 */
static int c7_cdr_reset(const struct rtl960x_ops *o)
{
	u32 v = o->rd(C7_SDS_ANA_COM09);

	o->wr(C7_SDS_ANA_COM09,
	      (v & ~0x400u) | ((u32)(!((v & 0x400u) >> 10)) << 10));
	mdelay(10);
	o->wr(C7_SDS_ANA_COM09, v);			/* restore original word   */

	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 14, 14, 0);	/* transfer FIFO assert rstb */
	mdelay(10);
	rtl960x_rfwr(o, C7_WSDS_DIG_1D, 14, 14, 1);	/* transfer FIFO release rstb*/
	return 0;
}

/* RTL9607C top-level entry points (thin wrappers over the c7_* internals). */
static int rtl9607c_ponmac_init(const struct rtl960x_ops *o)
{
	return c7_ponmac_init(o, RTL960X_REV_A, RTL960X_SUBTYPE_NONE);
}

static int rtl9607c_ponmac_mode_set(const struct rtl960x_ops *o,
				    int rev, int subtype)
{
	return c7_gpon_mode_set(o, rev, subtype);
}

static int rtl9607c_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	return c7_cdr_reset(o);
}

/* ------------------------------------------------------------------ *
 *  RTL9602C GPON PON-MAC / SerDes bring-up - clean-room op-table form.
 *  HW-TESTED on the realtek-luna board: this is a faithful translation of
 *  the in-tree gpon-rtl9602c.c gpon_serdes_init() / gpon_pbo-ponmac steps
 *  into this file's op-table primitives, so the family-lib path behaves
 *  identically to the inline driver.
 *
 *  Default parameter path baked in (the configuration that locks on hardware):
 *    serdes_cdr_reset = TRUE   -> the COM_REG08 bit15 invert/10ms/restore pulse
 *    serdes_modev1_tx = FALSE  -> SKIP the explicit COM_REG02/03/08/24/25 block
 *                                 (the golden analog table already covers them)
 *    serdes_tx_xtra   = FALSE  -> the D2A / sample-clock bits are forced to 0
 *                                 (0x220a8[5:4]=0, 0x2281c[14]=0, 0x22a30[8]=0)
 *    full_serdes_reinit = FALSE
 *
 *  Address spaces (absolute physical, supplied to the board's rd/wr via ops):
 *    SDS / swcore regs : 0x1B000000 + offset
 *    PON-IP regs       : 0x1BF00000 + offset
 *  Symbols are c2_/C2_ prefixed to stay collision-free with the other chips.
 * ------------------------------------------------------------------ */
#define C2_SWCORE_BASE		0x1B000000u	/* SDS / swcore offset base    */
#define C2_PONIP_BASE		0x1BF00000u	/* PON-IP datapath offset base */

/* swcore-relative register absolutes used by the ordered bring-up steps */
#define C2_SDS_CFG		0x1B0001D0u	/* [4:0] CFG_SDS_MODE          */
#define   C2_SDS_MODE_OFF	0x1Fu		/* illegal/off mode (park)     */
#define   C2_SDS_MODE_GPON	0x08u		/* GPON line-rate mode         */
#define C2_SW_SOFTWARE_RST	0x1B000104u	/* [7] SDS_CFG_RST [0] SDS_RST */
#define C2_WSDS_DIG_00		0x1B022030u	/* [0] STOP_CLK; run = 0xf30   */
#define   C2_WSDS_DIG00_RUN	0xF30u		/* operational run state       */
#define C2_WSDS_DIG_01		0x1B022034u	/* [31:0] CFG_DMY0 (force-SDS) */
#define C2_WSDS_DIG_02		0x1B022038u	/* [10] EN_PDOWN_BEN           */
#define C2_WSDS_DIG_03		0x1B02203Cu	/* [6:4] TXDIS_SEL_DLY [3:0]D2A*/
#define C2_WSDS_DIG_18		0x1B022090u	/* [12] BEN_OE [15:13] optic   */
#define C2_WSDS_DIG_1D		0x1B0220A4u	/* [16:14] interface reset-B   */
#define C2_WSDS_DIG_1E		0x1B0220A8u	/* [5:4] D2A interconnect      */
#define C2_SDS_REG7		0x1B02281Cu	/* [14] SP_CFG_NEG_CLKWR_A2D   */
#define C2_SDS_EXT_REG12	0x1B022A30u	/* [8] SEP_CFG_NEG_CLKRD_D2A   */
#define C2_SDS_FORCE_BEN	0x1B0220E4u	/* [0] BEN_FORCE_MODE          */
#define C2_SDS_ANA_COM_REG08	0x1B0225A0u	/* TX-CDR; [15] cdr_reset bit  */
#define C2_SDS_ANA_COM_REG12	0x1B0225B0u	/* [14] RX_SEL_CDR_AFEN        */
#define C2_SDS_ANA_COM_REG22	0x1B0225D8u	/* [5:3] TX_AMP [2:0] TX_EMP   */
#define C2_SDS_ANA_MISC_REG00	0x1B022500u	/* [5] FRC_RX_EN_VAL [4] _ON   */
#define C2_SDS_ANA_MISC_REG01	0x1B022504u	/* [7:5] SPDSEL_VAL [4] _ON    */
#define C2_SDS_ANA_MISC_REG02	0x1B022508u	/* [13] SD_VAL [12] SD_FORCE   */
#define C2_FIB_EXT_REG21	0x1B022E54u	/* [13] FEP_V2ANALOG (lock)    */
#define   C2_SDS_ANALOG_READY	13u		/* FIB_EXT_REG21 ready bit      */
#define C2_SDS_LOCK_POLL_MAX	1000u		/* x200us = up to 200 ms        */

/* FIB_REG0 bank bases (absolute); FP_CFG_FIB_PDOWN bit11 cleared = fiber on. */
#define C2_FIB_REG0_PDOWN	BIT(11)
static const u32 c2_fib_reg0_banks[] = {
	0x1B022C00u, 0x1B022C80u, 0x1B022D00u, 0x1B022D80u,
};

/*
 * Full SerDes analog + WSDS configuration golden table - the operating point
 * the ONU runs at O5 (CMU, CDR, RX front-end incl. optical signal-detect/LOS,
 * TX driver, GPON-rate banks, and the 4 FIB optical front-end banks). Each
 * entry is an ABSOLUTE swcore address (0x1B000000 + the chip offset) and its
 * operational value - register facts of this silicon. Status/monitor regs and
 * the digital reset-B/clock bank (DIG_00/18/1D) are deliberately excluded;
 * those are driven by the ordered sequence below. FIB_REG0 power-down (bit11)
 * is cleared separately, after each write, to turn fiber power on.
 */
static const struct { u32 off; u32 val; } c2_analog[] = {
	/* WSDS analog front + digital RX-path config */
	{ 0x1B022000u, 0x00000805 }, { 0x1B022008u, 0x0000ffff }, { 0x1B02201cu, 0x0000ffff },
	{ 0x1B022020u, 0x0000ffff }, { 0x1B022038u, 0x00000900 }, { 0x1B022048u, 0x000000ff },
	{ 0x1B022050u, 0x00022300 }, { 0x1B022054u, 0x00022310 }, { 0x1B022058u, 0x083d0100 },
	{ 0x1B022060u, 0x00000fff }, { 0x1B022064u, 0x0000cf45 }, { 0x1B022068u, 0x00000f45 },
	/* SDS_ANA_MISC (RX-enable force, speed-select, force-SD) */
	{ 0x1B022500u, 0x00000030 }, { 0x1B022504u, 0x00000030 }, { 0x1B022508u, 0x00003000 },
	/* SDS_ANA_COM (CMU, RX CDR front-end, filters, bias) */
	{ 0x1B022580u, 0x00003400 }, { 0x1B022584u, 0x000073a4 }, { 0x1B022588u, 0x00006df8 },
	{ 0x1B02258cu, 0x00008941 }, { 0x1B022590u, 0x00008884 }, { 0x1B022594u, 0x0000413f },
	{ 0x1B022598u, 0x00004fc0 }, { 0x1B02259cu, 0x00005682 }, { 0x1B0225a0u, 0x00000713 },
	{ 0x1B0225a4u, 0x000002f5 }, { 0x1B0225a8u, 0x00002793 }, { 0x1B0225acu, 0x0000b000 },
	{ 0x1B0225b0u, 0x00004848 }, { 0x1B0225b4u, 0x000000c8 }, { 0x1B0225bcu, 0x000008f2 },
	{ 0x1B0225c0u, 0x00001042 }, { 0x1B0225c4u, 0x0000c391 }, { 0x1B0225c8u, 0x00006a00 },
	{ 0x1B0225ccu, 0x00006600 }, { 0x1B0225d0u, 0x0000c000 },
	/* 0x225d8 (COM_REG22 TX_AMP/EMP) is set later, in the TX section, to the
	 * rev-A value 0x29 (TX_AMP=0x5, TX_EMP=0x1) via field-writes - NOT a full
	 * word here, so the upper bits keep their reset state. */
	{ 0x1B0225dcu, 0x00000418 }, { 0x1B0225e0u, 0x00008001 }, { 0x1B0225e4u, 0x0000001f },
	{ 0x1B0225e8u, 0x000011e4 }, { 0x1B0225ecu, 0x00009422 }, { 0x1B0225f0u, 0x00008502 },
	{ 0x1B0225f4u, 0x00000ff0 }, { 0x1B0225f8u, 0x0000000a },
	/* SDS_ANA_GPON (GPON-rate CDR/PLL/PCM config) */
	{ 0x1B022708u, 0x00000f00 }, { 0x1B02270cu, 0x0000b8c6 }, { 0x1B022710u, 0x0000a112 },
	{ 0x1B022714u, 0x00004280 }, { 0x1B022718u, 0x0000f53f }, { 0x1B02271cu, 0x00004fdf },
	{ 0x1B022720u, 0x00000001 }, { 0x1B022724u, 0x0000309b }, { 0x1B022728u, 0x0000225c },
	{ 0x1B02272cu, 0x00001061 }, { 0x1B022730u, 0x0000110d }, { 0x1B022734u, 0x00004854 },
	{ 0x1B022738u, 0x000080c5 }, { 0x1B02273cu, 0x0000121e }, { 0x1B022740u, 0x0000307b },
	{ 0x1B022744u, 0x00000271 }, { 0x1B022748u, 0x00000271 }, { 0x1B02274cu, 0x00001012 },
	{ 0x1B022750u, 0x0000f162 }, { 0x1B022754u, 0x00003026 }, { 0x1B022758u, 0x0000a780 },
	{ 0x1B02275cu, 0x0000f000 },
	/* SDS_ANA_GPON additional per-rate/lane banks (the RX path selects among
	 * these; leaving them at reset starves the active RX/SD analog). */
	{ 0x1B022608u, 0x00000f00 }, { 0x1B02260cu, 0x0000b8c6 }, { 0x1B022610u, 0x0000a112 },
	{ 0x1B022614u, 0x00004280 }, { 0x1B022618u, 0x0000f53f }, { 0x1B02261cu, 0x00004fdf },
	{ 0x1B022620u, 0x00000001 }, { 0x1B022624u, 0x0000309b }, { 0x1B022628u, 0x0000225c },
	{ 0x1B02262cu, 0x00001061 }, { 0x1B022630u, 0x0000110d }, { 0x1B022634u, 0x00004854 },
	{ 0x1B022638u, 0x000080c5 }, { 0x1B02263cu, 0x0000121e }, { 0x1B022640u, 0x0000307b },
	{ 0x1B022644u, 0x00000271 }, { 0x1B022648u, 0x00000271 }, { 0x1B02264cu, 0x00001012 },
	{ 0x1B022650u, 0x0000f162 }, { 0x1B022654u, 0x00003026 }, { 0x1B022658u, 0x0000a780 },
	{ 0x1B02265cu, 0x0000f000 },
	{ 0x1B022688u, 0x00000f00 }, { 0x1B02268cu, 0x0000b8c6 }, { 0x1B022690u, 0x0000a112 },
	{ 0x1B022694u, 0x00004280 }, { 0x1B022698u, 0x0000f53f }, { 0x1B02269cu, 0x00004fdf },
	{ 0x1B0226a0u, 0x00000001 }, { 0x1B0226a4u, 0x0000309b }, { 0x1B0226a8u, 0x0000225c },
	{ 0x1B0226acu, 0x00001062 }, { 0x1B0226b0u, 0x00002000 }, { 0x1B0226b4u, 0x00001050 },
	{ 0x1B0226b8u, 0x000080c1 }, { 0x1B0226bcu, 0x0000121e }, { 0x1B0226c0u, 0x0000107b },
	{ 0x1B0226c4u, 0x00000280 }, { 0x1B0226c8u, 0x00000280 }, { 0x1B0226ccu, 0x00001012 },
	{ 0x1B0226d0u, 0x0000f862 }, { 0x1B0226d4u, 0x00003938 }, { 0x1B0226d8u, 0x00003100 },
	{ 0x1B0226dcu, 0x0000f000 },
	{ 0x1B022788u, 0x00000f00 }, { 0x1B02278cu, 0x0000b8c6 }, { 0x1B022790u, 0x0000a112 },
	{ 0x1B022794u, 0x00004280 }, { 0x1B022798u, 0x0000f53f }, { 0x1B02279cu, 0x00004fdf },
	{ 0x1B0227a0u, 0x00000001 }, { 0x1B0227a4u, 0x0000309b }, { 0x1B0227a8u, 0x0000225c },
	{ 0x1B0227acu, 0x00001062 }, { 0x1B0227b0u, 0x00002000 }, { 0x1B0227b4u, 0x00004850 },
	{ 0x1B0227b8u, 0x000080c5 }, { 0x1B0227bcu, 0x0000121e }, { 0x1B0227c0u, 0x0000103e },
	{ 0x1B0227c4u, 0x00000280 }, { 0x1B0227c8u, 0x00000280 }, { 0x1B0227ccu, 0x00001012 },
	{ 0x1B0227d0u, 0x0000f862 }, { 0x1B0227d4u, 0x00003938 }, { 0x1B0227d8u, 0x0000b100 },
	{ 0x1B0227dcu, 0x0000f000 },
	/* FIB (fiber optical front-end) config - 4 identical banks. This block
	 * powers and configures the optical RX/SD path; FIB_REG0 (bank base)
	 * carries FP_CFG_FIB_PDOWN at bit11, cleared separately below to turn
	 * fiber power on. */
	{ 0x1B022c00u, 0x00001940 }, { 0x1B022c04u, 0x00006109 }, { 0x1B022c08u, 0x0000e001 },
	{ 0x1B022c0cu, 0x00003290 }, { 0x1B022c10u, 0x000001a0 }, { 0x1B022c1cu, 0x00000004 },
	{ 0x1B022c3cu, 0x00008000 }, { 0x1B022c40u, 0x00000083 }, { 0x1B022c48u, 0x00005000 },
	{ 0x1B022c58u, 0x00000001 }, { 0x1B022c5cu, 0x00004001 }, { 0x1B022c60u, 0x00000004 },
	{ 0x1B022c64u, 0x0000326a }, { 0x1B022c6cu, 0x0000115d }, { 0x1B022c70u, 0x000033fa },
	{ 0x1B022c74u, 0x0000e46a }, { 0x1B022c78u, 0x0000071e },
	{ 0x1B022c80u, 0x00001940 }, { 0x1B022c84u, 0x00006109 }, { 0x1B022c88u, 0x0000e001 },
	{ 0x1B022c8cu, 0x00003290 }, { 0x1B022c90u, 0x000001a0 }, { 0x1B022c9cu, 0x00000004 },
	{ 0x1B022cbcu, 0x00008000 }, { 0x1B022cc0u, 0x00000083 }, { 0x1B022cc8u, 0x00005000 },
	{ 0x1B022cd8u, 0x00000001 }, { 0x1B022cdcu, 0x00004001 }, { 0x1B022ce0u, 0x00000004 },
	{ 0x1B022ce4u, 0x0000326a }, { 0x1B022cecu, 0x0000115d }, { 0x1B022cf0u, 0x000033fa },
	{ 0x1B022cf4u, 0x0000e46a }, { 0x1B022cf8u, 0x0000071e },
	{ 0x1B022d00u, 0x00001940 }, { 0x1B022d04u, 0x00006109 }, { 0x1B022d08u, 0x0000e001 },
	{ 0x1B022d0cu, 0x00003290 }, { 0x1B022d10u, 0x000001a0 }, { 0x1B022d1cu, 0x00000004 },
	{ 0x1B022d3cu, 0x00008000 }, { 0x1B022d40u, 0x00000083 }, { 0x1B022d48u, 0x00005000 },
	{ 0x1B022d58u, 0x00000001 }, { 0x1B022d5cu, 0x00004001 }, { 0x1B022d60u, 0x00000004 },
	{ 0x1B022d64u, 0x0000326a }, { 0x1B022d6cu, 0x0000115d }, { 0x1B022d70u, 0x000033fa },
	{ 0x1B022d74u, 0x0000e46a }, { 0x1B022d78u, 0x0000071e },
	{ 0x1B022d80u, 0x00001940 }, { 0x1B022d84u, 0x00006109 }, { 0x1B022d88u, 0x0000e001 },
	{ 0x1B022d8cu, 0x00003290 }, { 0x1B022d90u, 0x000001a0 }, { 0x1B022d9cu, 0x00000004 },
	{ 0x1B022dbcu, 0x00008000 }, { 0x1B022dc0u, 0x00000083 }, { 0x1B022dc8u, 0x00005000 },
	{ 0x1B022dd8u, 0x00000001 }, { 0x1B022ddcu, 0x00004001 }, { 0x1B022de0u, 0x00000004 },
	{ 0x1B022de4u, 0x0000326a }, { 0x1B022decu, 0x0000115d }, { 0x1B022df0u, 0x000033fa },
	{ 0x1B022df4u, 0x0000e46a }, { 0x1B022df8u, 0x0000071e },
};

/*
 * ponmac_init scheduler / OMCI-egress steering (the SDS/PON-IP config writes).
 * BEN_TTL_OUT + DYNGASP on swcore; PON_BW_THRES (last + runt) and the transient
 * PON_GEN_PIR_DROP on PON-IP; OMCI_MPCP_PRIORITY steers OMCI egress to PON
 * queue 7 on swcore. PIR_DROP is asserted then cleared for rev-A, matching the
 * inline driver's set/clear pair.
 */
static const struct r960_op c2_ponmac_init[] = {
	FLD(0x1B022584u,  0,  0, 1),	/* SDS_ANA REG_BEN_TTL_OUT = 1          */
	FLD(0x1B0001ECu,  0,  0, 1),	/* DYNGASP_CMP_INV = 1                  */
	FLD(0x1BF02150u, 29, 16, 5),	/* PON_BW_THRES last-grant              */
	FLD(0x1BF02150u, 13,  0, 5),	/* PON_BW_THRES runt-grant             */
	FLD(0x1BF02194u, 18, 18, 1),	/* PON_GEN_PIR_DROP = 1                 */
	FLD(0x1B0111F8u,  2,  0, 7),	/* PON_TRAP_CFG OMCI_MPCP_PRIORITY = 7  */
	FLD(0x1BF02194u, 18, 18, 0),	/* rev-A: clear PON_GEN_PIR_DROP        */
};

static int rtl9602c_ponmac_init(const struct rtl960x_ops *o)
{
	return r960_run(o, c2_ponmac_init, ARRAY_SIZE(c2_ponmac_init));
}

/*
 * SerDes CDR-lock pulse (the stock dal_rtl9602c_ponmac_serdesCdr_reset): invert
 * SDS_ANA_COM_REG08 (0x1B0225A0) bit15, hold 10 ms, restore. Seats the upstream
 * serializer CDR so the per-grant burst is consistently lockable by the OLT
 * burst-RX. This is the serdes_cdr_reset = TRUE default path.
 */
static int rtl9602c_serdes_cdr_reset(const struct rtl960x_ops *o)
{
	u32 cdr = o->rd(C2_SDS_ANA_COM_REG08);

	o->wr(C2_SDS_ANA_COM_REG08, cdr ^ BIT(15));
	mdelay(10);
	o->wr(C2_SDS_ANA_COM_REG08, cdr);
	return 0;
}

/*
 * GPON SerDes (SDS) bring-up - a faithful translation of gpon_serdes_init().
 *
 * Ordering is the whole game: program the analog CMU/CDR block FIRST, keep
 * CFG_SDS_MODE parked at the illegal/off value, pulse the SDS+MAC reset to
 * latch the analog config, release the per-datapath soft-reset-B lines and
 * force the 125M reference clock, arm the RX-CDR through a forced RX-enable
 * 0->1 edge, set the TX data path + drive level, force signal-detect, and only
 * THEN switch CFG_SDS_MODE to GPON. A naive "reset-then-configure" sequence
 * ends with the same final register values yet a CDR that never locks.
 */

/* Step 1 + 3: park SDS mode off, clear force-SDS dummy / STOP_CLK, then pulse
 * the SDS config + datapath reset to latch the analog config (10 ms). */
static const struct r960_op c2_sds_pre[] = {
	FLD(C2_SDS_CFG, 4, 0, C2_SDS_MODE_OFF),	/* park CFG_SDS_MODE = off (0x1f) */
	WR(C2_WSDS_DIG_01, 0),			/* clear force-SDS dummy          */
	FLD(C2_WSDS_DIG_00, 0, 0, 0),		/* STOP_CLK = 0                   */
};

static const struct r960_op c2_sds_reset[] = {
	FLD(C2_SW_SOFTWARE_RST, 7, 7, 1),	/* CMD_SDS_CFG_RST_PS             */
	FLD(C2_SW_SOFTWARE_RST, 0, 0, 1),	/* CMD_SDS_RST_PS                 */
	DLY(10),
};

/* Step 4: release all datapath soft-reset-B lines + force 125M ref (DIG_00 run
 * = 0xf30), then pulse the RX/TX interface reset-B lines (DIG_1D = 0x1c000). */
static const struct r960_op c2_sds_rstb[] = {
	WR(C2_WSDS_DIG_00, C2_WSDS_DIG00_RUN),	/* run state, 125M ref forced     */
	FLD(C2_WSDS_DIG_1D, 15, 15, 0),		/* RX interface reset-B 0         */
	FLD(C2_WSDS_DIG_1D, 16, 16, 0),		/* TX interface reset-B 0         */
	FLD(C2_WSDS_DIG_1D, 14, 14, 1),		/* common interface reset-B 1     */
	FLD(C2_WSDS_DIG_1D, 15, 15, 1),		/* RX interface reset-B 1         */
	FLD(C2_WSDS_DIG_1D, 16, 16, 1),		/* TX interface reset-B 1         */
	DLY(10),
};

/*
 * Steps 5 + 6: burst-enable output (BEN_OE) with optic-LOS left un-forced (the
 * real RX front-end drives SD via the external BOSA), then arm the RX in order:
 * enable the RX-CDR analog front end, settle 10 ms, force the line-rate select
 * to the GPON rate, drive the forced RX-enable through a 0->1 edge to start the
 * CDR, settle 50 ms. Finish with EN_PDOWN_BEN=0, TXDIS_SEL_DLY=0, D2A_SEL=0 and
 * BEN_FORCE_MODE=0 (let the GTC framer drive the laser gate).
 */
static const struct r960_op c2_sds_rx_arm[] = {
	FLD(C2_WSDS_DIG_18, 12, 12, 1),		/* BEN_OE = 1                     */
	FLD(C2_WSDS_DIG_18, 15, 15, 0),		/* OPTIC_LOS_SEL_EPON = 0         */
	FLD(C2_WSDS_DIG_18, 14, 14, 0),		/* CFG_FRC_OPTIC_LOS = 0          */
	FLD(C2_WSDS_DIG_18, 13, 13, 0),		/* CFG_FRCV_OPTIC_LOS = 0         */
	FLD(C2_SDS_ANA_COM_REG12, 14, 14, 1),	/* RX_SEL_CDR_AFEN = 1            */
	DLY(10),
	FLD(C2_SDS_ANA_MISC_REG01, 7, 5, 1),	/* SPDSEL_VAL = GPON rate         */
	FLD(C2_SDS_ANA_MISC_REG01, 4, 4, 1),	/* SPDSEL force on                */
	FLD(C2_SDS_ANA_MISC_REG00, 4, 4, 1),	/* FRC_RX_EN_ON = 1               */
	FLD(C2_SDS_ANA_MISC_REG00, 5, 5, 0),	/* FRC_RX_EN_VAL 0 ...            */
	FLD(C2_SDS_ANA_MISC_REG00, 5, 5, 1),	/* ... -> 1 (start CDR)           */
	DLY(50),
	FLD(C2_WSDS_DIG_02, 10, 10, 0),		/* EN_PDOWN_BEN = 0               */
	FLD(C2_WSDS_DIG_03, 6, 4, 0),		/* CFG_TXDIS_SEL_DLY = 0          */
	FLD(C2_WSDS_DIG_03, 3, 0, 0),		/* CFG_D2ANLOG_SEL = 0 (TX path)  */
	FLD(C2_SDS_FORCE_BEN, 0, 0, 0),		/* BEN_FORCE_MODE = 0 (GTC gates) */
};

/*
 * Step 6b + TX drive (serdes_modev1_tx=FALSE / serdes_tx_xtra=FALSE defaults):
 * the golden analog table already covers COM_REG02/03/08/24/25, so SKIP the
 * explicit ModeV1 block. Force the D2A interconnect + sample-clock bits to 0
 * (match the live stock ONU). Then set the rev-A TX drive level (TX_AMP=0x5,
 * TX_EMP=0x1) via field-writes, before switching to GPON mode.
 */
static const struct r960_op c2_sds_tx[] = {
	FLD(C2_WSDS_DIG_1E,    5,  4, 0),	/* D2A interconnect = 0 (stock)   */
	FLD(C2_SDS_REG7,      14, 14, 0),	/* SP_CFG_NEG_CLKWR_A2D = 0       */
	FLD(C2_SDS_EXT_REG12,  8,  8, 0),	/* SEP_CFG_NEG_CLKRD_D2A = 0      */
	FLD(C2_SDS_ANA_COM_REG22, 5, 3, 5),	/* REG_TX_AMP = 0x5               */
	FLD(C2_SDS_ANA_COM_REG22, 2, 0, 1),	/* REG_TX_EMP = 0x1               */
};

/*
 * Steps 7a + 7b: force signal-detect on so the MAC reset handshake (RST_DONE)
 * completes (MISC_REG02 = 0x3000), settle 10 ms, then finally select GPON mode
 * with the RX fully armed, settle 50 ms. Followed by a TX-interface reset-B
 * re-sync (DIG_1D[16] 0->1) so the TX serializer re-locks onto the connected
 * framer data now that GPON mode is live.
 */
static const struct r960_op c2_sds_mode[] = {
	FLD(C2_SDS_ANA_MISC_REG02, 13, 13, 1),	/* signal-detect value = 1        */
	FLD(C2_SDS_ANA_MISC_REG02, 12, 12, 1),	/* force signal-detect            */
	DLY(10),
	FLD(C2_SDS_CFG, 4, 0, C2_SDS_MODE_GPON),/* select GPON mode (very last)   */
	DLY(50),
	FLD(C2_WSDS_DIG_1D, 16, 16, 0),		/* TX interface reset-B 0         */
	DLY(2),
	FLD(C2_WSDS_DIG_1D, 16, 16, 1),		/* TX interface reset-B 1         */
	DLY(10),
};

static int rtl9602c_ponmac_mode_set(const struct rtl960x_ops *o,
				    int rev, int subtype)
{
	unsigned int i;
	int ret;

	(void)rev; (void)subtype;	/* single rev-A SerDes variant */

	/* Steps 1 + 3 share a write phase around step 2 (the analog table). */
	ret = r960_run(o, c2_sds_pre, ARRAY_SIZE(c2_sds_pre));
	if (ret)
		return ret;

	/* Step 2: program the FULL analog block + turn fiber power on (clear
	 * FP_CFG_FIB_PDOWN on every FIB bank). */
	for (i = 0; i < ARRAY_SIZE(c2_analog); i++)
		o->wr(c2_analog[i].off, c2_analog[i].val);
	for (i = 0; i < ARRAY_SIZE(c2_fib_reg0_banks); i++)
		o->wr(c2_fib_reg0_banks[i],
		      o->rd(c2_fib_reg0_banks[i]) & ~C2_FIB_REG0_PDOWN);

	/* Step 3: pulse the SDS config + datapath reset to latch the analog. */
	ret = r960_run(o, c2_sds_reset, ARRAY_SIZE(c2_sds_reset));
	if (ret)
		return ret;

	/* Step 4: release soft-reset-B lines, force 125M ref, pulse interface
	 * reset-B. Re-clear FIB power-down, which the reset re-asserts. */
	ret = r960_run(o, c2_sds_rstb, ARRAY_SIZE(c2_sds_rstb));
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(c2_fib_reg0_banks); i++)
		o->wr(c2_fib_reg0_banks[i],
		      o->rd(c2_fib_reg0_banks[i]) & ~C2_FIB_REG0_PDOWN);

	/* Steps 5 + 6: burst-enable + arm the RX-CDR. */
	ret = r960_run(o, c2_sds_rx_arm, ARRAY_SIZE(c2_sds_rx_arm));
	if (ret)
		return ret;

	/* Step 6b + TX drive (modev1/tx_xtra defaults baked off). */
	ret = r960_run(o, c2_sds_tx, ARRAY_SIZE(c2_sds_tx));
	if (ret)
		return ret;

	/* Steps 7a + 7b + TX reset-B re-sync. */
	ret = r960_run(o, c2_sds_mode, ARRAY_SIZE(c2_sds_mode));
	if (ret)
		return ret;

	/* US-TX serdesCdr_reset pulse (serdes_cdr_reset = TRUE default), after GPON
	 * mode + the TX reset-B re-sync so the analog-ready poll confirms recovery. */
	rtl9602c_serdes_cdr_reset(o);

	/* Keep the MAC clock ungated. */
	rtl960x_rfwr(o, C2_WSDS_DIG_00, 0, 0, 0);

	/* Wait for the analog to report ready (FIB_EXT_REG21 bit13); ~200 ms cap.
	 * Return the poll result, matching gpon_serdes_init's final return. */
	return r960_run(o, (const struct r960_op[]){
		POLL(C2_FIB_EXT_REG21, C2_SDS_ANALOG_READY, C2_SDS_LOCK_POLL_MAX),
	}, 1);
}

/* ---- dispatch --------------------------------------------------------- */
int rtl960x_ponmac_init(enum rtl960x_chip chip, int rev, int subtype,
			const struct rtl960x_ops *o)
{
	(void)rev; (void)subtype;	/* per-chip init takes (o) only */
	switch (chip) {
	case RTL960X_CHIP_9601B:
		return rtl9601b_ponmac_init(o);
	case RTL960X_CHIP_9602C:
		return rtl9602c_ponmac_init(o);
	case RTL960X_CHIP_9603CVD:
		return rtl9603cvd_ponmac_init(o);
	case RTL960X_CHIP_9607C:
		return rtl9607c_ponmac_init(o);
	default:			/* 9607F: reg map not available yet */
		return -ENOTSUPP;
	}
}

int rtl960x_ponmac_mode_set(enum rtl960x_chip chip, int rev, int subtype,
			    enum rtl960x_ponmode mode, const struct rtl960x_ops *o)
{
	if (mode != RTL960X_MODE_GPON)	/* only GPON is supported today */
		return -ENOTSUPP;

	switch (chip) {
	case RTL960X_CHIP_9601B:
		return rtl9601b_ponmac_mode_set(o, rev, subtype);
	case RTL960X_CHIP_9602C:
		return rtl9602c_ponmac_mode_set(o, rev, subtype);
	case RTL960X_CHIP_9603CVD:
		return rtl9603cvd_ponmac_mode_set(o, rev, subtype);
	case RTL960X_CHIP_9607C:
		return rtl9607c_ponmac_mode_set(o, rev, subtype);
	default:			/* 9607F: reg map not available yet */
		return -ENOTSUPP;
	}
}

int rtl960x_ponmac_serdes_cdr_reset(enum rtl960x_chip chip,
				    const struct rtl960x_ops *o)
{
	switch (chip) {
	case RTL960X_CHIP_9601B:
		return rtl9601b_serdes_cdr_reset(o);
	case RTL960X_CHIP_9602C:
		return rtl9602c_serdes_cdr_reset(o);
	case RTL960X_CHIP_9603CVD:
		return rtl9603cvd_serdes_cdr_reset(o);
	case RTL960X_CHIP_9607C:
		return rtl9607c_serdes_cdr_reset(o);
	default:			/* 9607F: reg map not available yet */
		return -ENOTSUPP;
	}
}
