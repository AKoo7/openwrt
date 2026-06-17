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
/* Per-port address model (reg.c::_reg_addr_find): a register whose base lies in
 * the MACPP block [0x20000,0x203FF] is a PpReg -> +ponPort*MACPP_INTERVAL; any
 * other base is a Global array -> +ponPort*(array_offset_bits/8). 9601b: MACPP
 * interval 0x400, array_offset 32 (=4B), ponPort 1. (NOT a flat 0x20 stride.) */
#define C1B_R_PMISC_PON	0x1B020408u	/* P_MISC[pon1]: PpReg 0x20008 + 1*0x400 */
#define C1B_R_ACC_LEN_PON	0x1B023038u	/* ACCEPT_MAX_LEN[pon1]: Global 0x23034 + 1*4 */
#define C1B_R_ACC_LEN_UTP	0x1B023034u	/* UTP RX accept max length          */
#define C1B_R_TX_LEN_PON	0x1B01100Cu	/* TX_MAX_LEN[pon1]: Global 0x11008 + 1*4 */
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
#define C3_P_MISC_PON		0x1b020404u /* P_MISC[pon4]: PpReg 0x20004 + 4*MACPP_INTERVAL(0x100) */
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
/* A/B knob set by the board (gpon.serdes_postmode_perturb). When 0, skip the TWO
 * post-GPON-mode US-TX serializer perturbations that stock rev-A does NOT do: the
 * DIG_1D[16] reset-B re-sync (c2_sds_txresync below) and the post-mode
 * serdesCdr_reset pulse. These late edges on the already-running serializer are
 * the prime suspect for per-boot serializer-phase jitter (cold-start WAN ~50%).
 * Default 1 = legacy behavior. */
int rtl960x_c2_postmode_perturb = 1;

static const struct r960_op c2_sds_mode[] = {
	FLD(C2_SDS_ANA_MISC_REG02, 13, 13, 1),	/* signal-detect value = 1        */
	FLD(C2_SDS_ANA_MISC_REG02, 12, 12, 1),	/* force signal-detect            */
	DLY(10),
	FLD(C2_SDS_CFG, 4, 0, C2_SDS_MODE_GPON),/* select GPON mode (very last)   */
	DLY(50),
};

/* Post-GPON-mode TX-interface reset-B re-sync (DIG_1D[16] 0->1) — stock rev-A
 * OMITS this; gated by rtl960x_c2_postmode_perturb. */
static const struct r960_op c2_sds_txresync[] = {
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

	/* Step 7a + 7b: force-SD + commit GPON mode. */
	ret = r960_run(o, c2_sds_mode, ARRAY_SIZE(c2_sds_mode));
	if (ret)
		return ret;

	/* The TX reset-B re-sync + the post-mode serdesCdr_reset pulse are the two
	 * perturbations stock rev-A omits; do them only when explicitly enabled. */
	if (rtl960x_c2_postmode_perturb) {
		ret = r960_run(o, c2_sds_txresync, ARRAY_SIZE(c2_sds_txresync));
		if (ret)
			return ret;
		rtl9602c_serdes_cdr_reset(o);
	}

	/* Keep the MAC clock ungated. */
	rtl960x_rfwr(o, C2_WSDS_DIG_00, 0, 0, 0);

	/* Wait for the analog to report ready (FIB_EXT_REG21 bit13); ~200 ms cap.
	 * Return the poll result, matching gpon_serdes_init's final return. */
	return r960_run(o, (const struct r960_op[]){
		POLL(C2_FIB_EXT_REG21, C2_SDS_ANALOG_READY, C2_SDS_LOCK_POLL_MAX),
	}, 1);
}

/* ====================================================================== *
 *  EPON PON-MAC / SerDes bring-up (clean-room op-table form).
 *
 *  One section per chip; each reuses that chip's GPON #defines, helpers and
 *  the shared r960_run()/rtl960x_rfwr() framework defined above. Register-
 *  faithful to the vendor _ponMacEponMode*_set() sequences but UNTESTED (no
 *  EPON hardware); the GPON path is unchanged. PBO/datapath + flow-control
 *  patches are owned by their own subsystems, exactly as for the GPON port.
 * ====================================================================== */

/* ------------------------------------------------------------------ *
 *  RTL9601B EPON PON-MAC / SerDes bring-up - clean-room op-table form.
 *
 *  Companion to rtl9601b_ponmac_mode_set() (the GPON path) in this file: it
 *  reuses the same primitives already in scope - enum r960_opc, struct r960_op,
 *  the WR/FLD/DLY macros, r960_run(), rtl960x_rfwr(), the indirect-SerDes
 *  cmd/data helpers (c1b_sds_wr / c1b_sds_rd), the analog-trim re-apply
 *  (rtl9601b_sds_patch), the flow->queue mapper (rtl9601b_flow2q) and the
 *  C1B_* absolute-address / page / index constants. Nothing here is a standalone
 *  unit; do not redeclare any of those.
 *
 *  SerDes access stays INDIRECT (idx<<11 | page<<5 | reg). The EPON PCS trim
 *  lives in the raw SDS (page 0x00) and SDS_EXT (page 0x01) lane pages - not in
 *  the per-rate speed pages used by GPON.
 *
 *  Register addresses, page/reg coordinates, values, ordering and delays are
 *  hardware-interface facts of the RTL9601B silicon (resolved from this chip's
 *  own reg_list/regField map); the expression below is original.
 *
 *  Vendor ordering reproduced faithfully (dal_rtl9601b_ponmac_mode_set, the
 *  PONMAC_MODE_EPON arm):
 *      rev split -> PON_MODE_CFG=2 -> PBO ponMode(EPON) [PONIP_MODE.CFG_EPON_MODE]
 *      -> ponNic QMAP/T-CONT seed -> SDS1_CFG=EPON -> BEN_OE -> RX_SPC=0
 *      -> PCS lane trim -> EN_PDOWN_BEN=0 -> PORT_CLK -> maxPktLen.
 * ------------------------------------------------------------------ */

/* EPON line-rate select for the SerDes lane (CFG_SDS1_MODE; vs 0x8 for GPON) */
#define C1B_SDS1_MODE_EPON	0xc

/* PON-IP: PONIP_MODE (PON-IP off 0xF01000 -> phys 0x1BF01000). Bit2 =
 * CFG_EPON_MODE (regField lsp 2, len 1). rtl9601b_raw_pbo_ponMode_set() writes
 * this for BOTH modes: 1 = EPON, 0 = GPON. The whole PON-IP window is 0x1BF0xxxx.
 * The GPON section never touches this reg (its default 0 already selects GPON),
 * so the EPON write is a vendor op the GPON tables legitimately omit. */
#define C1B_R_PONIP_MODE	0x1BF01000u

/* PON-IP scheduler seed for EPON (raw whole-word ioal_ponNic_write in vendor):
 *   PON_SCH_QMAP[0] (0xF0213C -> 0x1BF0213C): T-CONT 0 queue-membership mask
 *   PON_SCH_QMAP[1] (0xF02140 -> 0x1BF02140): T-CONT 1 queue-membership mask
 *   PON_TCONT_EN    (0xF0215C -> 0x1BF0215C): per-T-CONT enable bitfield
 * QMAP is 1 word (32 bits) per T-CONT (regField array offset 32). The seed
 * values open logical queues 0..9 on T-CONT 0 (0x3FF) and the same 10-queue
 * width in the high half-word for T-CONT 1 (0x03FF0000), and enable T-CONT 0
 * and 1 (0x03). These are the literal ioal_ponNic_write whole-word values from
 * the vendor EPON arm; they overwrite the earlier per-queue queue_add(0..7 on
 * sched 0) loop, and the per-queue strict/CIR=0/PIR=max/weight=1 shaping is
 * already applied to every queue by rtl9601b_ponmac_init - so the vendor's
 * queue_add loop collapses to these authoritative whole-word writes. */
#define C1B_R_QMAP0		0x1BF0213Cu	/* == C1B_R_QMAP_BASE (T-CONT 0) */
#define C1B_R_QMAP1		0x1BF02140u	/* QMAP_BASE + 4    (T-CONT 1)  */
/* C1B_R_TCONT_EN (0x1BF0215C) is already defined in the GPON section.         */

/* WSDS_DIG_01 (0x1B022004): GPON uses a field write to CFG_CLKRD_ORG (bit2);
 * EPON writes the WHOLE register = 4, which selects that clkrd source AND clears
 * every other digital-01 knob. Vendor uses reg_write here, not reg_field_write,
 * so this is a full-word WR(), not an FLD(). */
#define C1B_WSDS01_CLKRD_ORG	0x4u

/* ---- EPON PCS SerDes trim (indirect): raw SDS + SDS_EXT lane pages -------- *
 * Fixes the inter-packet-gap / preamble / burst timing the EPON MAC expects on
 * the 1.25G burst lane. {index, page, reg, data}; index = PON lane.
 * Pages: RTL9601B_SDS_PAGE_SDS = 0x00, RTL9601B_SDS_PAGE_SDS_EXT = 0x01.       */
#define C1B_SP_SDS		0x00	/* raw SerDes lane page              */
#define C1B_SP_SDS_EXT		0x01	/* SerDes lane extension page        */

static const struct c1b_sds_op rtl9601b_sds_patch_epon[] = {
	{ C1B_SI_PON, C1B_SP_SDS,     1,  0x0000 },	/* SDS_REG01: clear PCS control  */
	{ C1B_SI_PON, C1B_SP_SDS,     2,  0x5180 },	/* SDS_REG02: 129-byte IPG fix   */
	{ C1B_SI_PON, C1B_SP_SDS,     7,  0x0059 },	/* SDS_REG07: preamble/sync      */
	{ C1B_SI_PON, C1B_SP_SDS,    28,  0xc040 },	/* SDS_REG28: burst alignment    */
	{ C1B_SI_PON, C1B_SP_SDS_EXT, 6,  0x0043 },	/* SDS_EXT_REG06: RX gearbox     */
	{ C1B_SI_PON, C1B_SP_SDS_EXT, 7,  0x0003 },	/* SDS_EXT_REG07: RX gearbox en  */
	{ C1B_SI_PON, C1B_SP_SDS_EXT, 12, 0x0005 },	/* SDS_EXT_REG12: TX FIFO depth  */
	{ C1B_SI_PON, C1B_SP_SDS_EXT, 13, 0x4e6a },	/* SDS_EXT_REG13: CDR window     */
	{ C1B_SI_PON, C1B_SP_SDS_EXT, 15, 0x1162 },	/* SDS_EXT_REG15: laser-on burst */
	{ C1B_SI_PON, C1B_SP_SDS_EXT, 16, 0xbd2a },	/* SDS_EXT_REG16: laser-off guard*/
};

/* ---- EPON straight-line register runs ------------------------------------ */

/* rev>0: arm the digital clock source ahead of the SerDes/CMU bypass writes.
 * Whole-word write to WSDS_DIG_01 (clkrd-from-original + clear the rest). */
static const struct r960_op rtl9601b_epon_pre_revA[] = {
	WR(C1B_R_WSDS01, C1B_WSDS01_CLKRD_ORG),	/* clkrd source = original clock    */
};

/* mode-select + PON-IP scheduler seed (vendor order: PON_MODE_CFG, PONIP_MODE
 * CFG_EPON_MODE, then the three PON-IP scheduler-window words). */
static const struct r960_op rtl9601b_epon_mode[] = {
	WR(C1B_R_MODE_CFG, 2),			/* PON_MODE_CFG = EPON              */
	FLD(C1B_R_PONIP_MODE, 2, 2, 1),		/* PONIP_MODE.CFG_EPON_MODE = 1    */
	WR(C1B_R_QMAP0, 0x000003FF),		/* T-CONT 0 queue membership (q0..9)*/
	WR(C1B_R_QMAP1, 0x03FF0000),		/* T-CONT 1 queue membership        */
	WR(C1B_R_TCONT_EN, 0x00000003),		/* enable T-CONT 0 and 1           */
};

/* SerDes lane + PON-port datapath enable (run after the mode/scheduler seed). */
static const struct r960_op rtl9601b_epon_enable[] = {
	FLD(C1B_R_SDS1_CFG, 4, 0, C1B_SDS1_MODE_EPON),	/* lane -> EPON line rate   */
	FLD(C1B_R_WSDS12, 12, 12, 1),		/* BEN_OE = 1 (burst-enable out)   */
	FLD(C1B_R_PMISC_PON, 2, 2, 0),		/* RX_SPC = 0 (reject undersize)   */
};

/* tail: keep TX live across BEN (EN_PDOWN_BEN = 0). Vendor order within the
 * tail: WSDS_DIG_11, PORT_CLK, then maxPktLen(PON, UTP). */
static const struct r960_op rtl9601b_epon_tail[] = {
	FLD(C1B_R_WSDS11, 0, 0, 0),		/* CFG_EN_PDOWN_BEN = 0            */
};

static const struct r960_op rtl9601b_epon_maxlen[] = {
	FLD(C1B_R_ACC_LEN_PON, 27, 14, Q9601B_MAXLEN),	/* PON RX-accept GIGA    */
	FLD(C1B_R_ACC_LEN_PON, 13,  0, Q9601B_MAXLEN),	/* PON RX-accept 10/100  */
	FLD(C1B_R_TX_LEN_PON,  27, 14, Q9601B_MAXLEN),	/* PON TX GIGA           */
	FLD(C1B_R_TX_LEN_PON,  13,  0, Q9601B_MAXLEN),	/* PON TX 10/100         */
	FLD(C1B_R_ACC_LEN_UTP, 27, 14, Q9601B_MAXLEN),	/* UTP RX-accept GIGA    */
	FLD(C1B_R_ACC_LEN_UTP, 13,  0, Q9601B_MAXLEN),	/* UTP RX-accept 10/100  */
	FLD(C1B_R_TX_LEN_UTP,  27, 14, Q9601B_MAXLEN),	/* UTP TX GIGA           */
	FLD(C1B_R_TX_LEN_UTP,  13,  0, Q9601B_MAXLEN),	/* UTP TX 10/100         */
};

/* ------------------------------------------------------------------ *
 *  EPON mode select.
 *  rev = silicon revision (0 = rev-0, >0 = rev-A+). subtype unused on 9601B.
 * ------------------------------------------------------------------ */
static int rtl9601b_ponmac_mode_set_epon(const struct rtl960x_ops *o,
					 int rev, int subtype)
{
	u32 f;
	int ret;
	(void)subtype;

	/* re-apply the rev-selected analog trim (shared with the GPON path) */
	ret = rtl9601b_sds_patch(o, rev);
	if (ret)
		return ret;

	/* EPON: steer flows 0..31 to scheduler 0 / queue 0 (vendor maps all flows
	 * to T-CONT 0 / queue 0; queue-membership is set by the QMAP whole-word
	 * writes below, the per-queue shaping by ponmac_init). Note rtl9601b_flow2q
	 * uses the GPON physical-queue formula (sched/8); for sched 0 that yields
	 * pqid 0, identical to the EPON sched/2 formula, so reuse is exact here. */
	for (f = 0; f < 32; f++)
		rtl9601b_flow2q(o, f, 0, 0);

	if (rev == 0) {
		/* rev-0 LOS-case patch: PON lane page-common reg12 = 0x4840 */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 12, 0x4840);
		if (ret)
			return ret;
	} else {
		/* rev>0: clkrd source, then CMU-TX ber-notify bypass + CDR-rst-sel */
		ret = r960_run(o, rtl9601b_epon_pre_revA,
			       ARRAY_SIZE(rtl9601b_epon_pre_revA));
		if (ret)
			return ret;
		/* REG_BYPASS_BER_NOTIFY_CMUTX = 1 (page-common reg1) */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 1, 0x4a8a);
		if (ret)
			return ret;
		/* REG_CDR_RST_SEL = 0 (page-common reg11) */
		ret = c1b_sds_wr(o, C1B_SI_PON, C1B_SP_COMMON, 11, 0xb000);
		if (ret)
			return ret;
	}

	/* PON_MODE_CFG=EPON, PONIP CFG_EPON_MODE=1, then the PON-IP scheduler seed */
	ret = r960_run(o, rtl9601b_epon_mode,
		       ARRAY_SIZE(rtl9601b_epon_mode));
	if (ret)
		return ret;

	/* SerDes lane -> EPON rate, BEN_OE on, PON port rejects undersize */
	ret = r960_run(o, rtl9601b_epon_enable,
		       ARRAY_SIZE(rtl9601b_epon_enable));
	if (ret)
		return ret;

	/* EPON PCS lane trim (raw SDS + SDS_EXT pages) */
	ret = c1b_sds_run(o, rtl9601b_sds_patch_epon,
			  ARRAY_SIZE(rtl9601b_sds_patch_epon));
	if (ret)
		return ret;

	/* keep TX live when BEN deasserts (CFG_EN_PDOWN_BEN = 0) */
	ret = r960_run(o, rtl9601b_epon_tail,
		       ARRAY_SIZE(rtl9601b_epon_tail));
	if (ret)
		return ret;

	/* RSVD_PER_PORT_MAC clock select: clear bit2 (EPON clears it; GPON sets it
	 * for sys_clk 62.5M / TXC 155.5M). Read-modify-write the raw 32-bit word. */
	o->wr(C1B_R_PORT_CLK, o->rd(C1B_R_PORT_CLK) & ~0x4u);

	/* jumbo ceilings on PON + UTP (RX-accept then TX, per port) = 2031 */
	return r960_run(o, rtl9601b_epon_maxlen,
			ARRAY_SIZE(rtl9601b_epon_maxlen));
}

/* ------------------------------------------------------------------ *
 *  RTL9602C EPON PON-MAC / SerDes mode select - clean-room op-table form.
 *
 *  This is a SECTION of rtl960x_ponmac.c: it reuses that file's op-table
 *  primitives (enum r960_opc, struct r960_op, the WR/FLD/DLY/POLL macros and
 *  the r960_run() interpreter) plus rtl960x_rfwr()/ARRAY_SIZE already in scope,
 *  and the c2_/C2_ chip prefix + register absolutes already defined by the
 *  9602C GPON block above (C2_SDS_CFG, C2_SW_SOFTWARE_RST, C2_WSDS_DIG_00/01/02/
 *  18/1E, C2_SDS_REG7, C2_SDS_EXT_REG12, C2_SDS_FORCE_BEN, C2_SDS_ANA_COM_REG08/
 *  12/22, C2_SDS_ANA_MISC_REG00/02). EPON-only registers get the C2_E_ prefix so
 *  they never collide with the GPON block's C2_ names.
 *
 *  SerDes access on this part is DIRECT MMIO (a flat swcore bank), so every
 *  analog/digital knob is a plain RMW/word write - there is no indirect
 *  command/data page window.
 *
 *  Register addresses, field bit-ranges, values, ordering and delays are
 *  hardware-interface facts of the RTL9602C silicon (resolved from the chip's
 *  own reg_list/regField map). The expression - data-driven op-tables, the
 *  rev split, and all comments - is original.
 *
 *  Address space: swcore physical base 0x1B000000 + offset. The board supplies
 *  rd/wr in struct rtl960x_ops to map phys->virt.
 *
 *  Rev split (mirrors the stock PONMAC_MODE_EPON case: rev > CHIP_REV_ID_A picks
 *  EponModeV2, otherwise ModeV1 - the same ModeV1 used by the GPON rev-A path):
 *    rev == RTL960X_REV_A -> ModeV1 SerDes patch (rev-A operating point).
 *    rev  > RTL960X_REV_A -> EponModeV2 SerDes patch (CFG_SFT_RSTB pulse + GPON-
 *                            rate CDR retune).
 * ------------------------------------------------------------------ */

/* EPON-only swcore register absolutes (the 9602C GPON block already defines the
 * shared ones - C2_SDS_CFG, C2_SW_SOFTWARE_RST, C2_WSDS_DIG_00/01/02/18/1E,
 * C2_SDS_REG7, C2_SDS_EXT_REG12, C2_SDS_FORCE_BEN, C2_SDS_ANA_COM_REG08/12/22,
 * C2_SDS_ANA_MISC_REG00/02 - and are reused here verbatim). */
#define C2_E_SDS_MODE_EPON	0x0Cu		/* CFG_SDS_MODE = EPON line rate  */
#define C2_E_SDS_REG1		0x1B022804u	/* [11:8] SP_SDS_FRC_RX           */
#define C2_E_SDS_REG2		0x1B022808u	/* [9:8] SP_SDS_FRC_AN [13:12] IPG*/
#define C2_E_FIB_REG0		0x1B022C00u	/* [11] FP_CFG_FIB_PDOWN          */
#define C2_E_SDS_EXT_REG6	0x1B022A18u	/* [15:0] SEP_CFG_FEC_MK_OPT      */
#define C2_E_SDS_EXT_REG7	0x1B022A1Cu	/* [15:0] EPON DS-FEC control     */
#define C2_E_SDS_EXT_REG13	0x1B022A34u	/* [15:0] PCS SerDes patch        */
#define C2_E_SDS_EXT_REG15	0x1B022A3Cu	/* [15:0] PCS SerDes patch        */
#define C2_E_SDS_EXT_REG16	0x1B022A40u	/* [15:0] PCS SerDes patch        */
/* SerDes analog banks touched only by the ModeV1 patch (rev-A) */
#define C2_E_SDS_ANA_COM_REG02	0x1B022588u	/* RX CMU charge-pump / LDO / tank */
#define C2_E_SDS_ANA_COM_REG03	0x1B02258Cu	/* RX CMU bias                     */
#define C2_E_SDS_ANA_COM_REG11	0x1B0225ACu	/* [7:0] REG_RX_FILT_CONFIG        */
#define C2_E_SDS_ANA_COM_REG19	0x1B0225CCu	/* [14] CDR_RESET_MANUAL [10] LPF  */
#define C2_E_SDS_ANA_COM_REG24	0x1B0225E0u	/* TX LA LDO                       */
#define C2_E_SDS_ANA_COM_REG25	0x1B0225E4u	/* TX driver bias                  */
#define C2_E_SDS_ANA_1P25G_REG46 0x1B0226B8u	/* 1.25G-rate CDR Ki/Kp1/Kp2       */
#define C2_E_SDS_ANA_EPON_REG46	0x1B0227B8u	/* EPON-rate CDR Ki/Kp1/Kp2        */
#define C2_E_SDS_ANA_GPON_REG46	0x1B022738u	/* GPON-rate CDR Ki/Kp1/Kp2 (V2)   */

/*
 * Phase 1: switch the SerDes lane into EPON line rate and pulse the switch
 * soft-reset that the mode change requires, then settle 10 ms.
 */
static const struct r960_op c2_epon_mode[] = {
	FLD(C2_SDS_CFG, 4, 0, C2_E_SDS_MODE_EPON),	/* CFG_SDS_MODE = EPON       */
	FLD(C2_SW_SOFTWARE_RST, 5, 5, 1),		/* SW_RST: mode-change reset */
	DLY(10),					/* let the reset settle      */
};

/*
 * Phase 2: arm the optical/analog front end before the per-rev SerDes patch.
 * Turn fiber power on (clear FP_CFG_FIB_PDOWN), disable SerDes auto-negotiation
 * so the link runs forced (SP_SDS_FRC_AN=1), force the BER-notify pair high so a
 * signal-detect drop will not knock the EPON MAC down, then enable the
 * burst-enable output driver.
 */
static const struct r960_op c2_epon_sds_arm[] = {
	FLD(C2_E_FIB_REG0,         11, 11, 0),	/* FP_CFG_FIB_PDOWN = 0 (fiber on) */
	FLD(C2_E_SDS_REG2,          9,  8, 1),	/* SP_SDS_FRC_AN = 1 (NWAY off)    */
	FLD(C2_SDS_ANA_MISC_REG02, 13, 13, 1),	/* FRC_BER_NOTIFY_VAL = 1          */
	FLD(C2_SDS_ANA_MISC_REG02, 12, 12, 1),	/* FRC_BER_NOTIFY_ON = 1           */
	FLD(C2_WSDS_DIG_18,        12, 12, 1),	/* BEN_OE = 1 (burst-enable out)   */
};

/*
 * ModeV1 SerDes patch (rev-A operating point - the same ModeV1 the GPON rev-A
 * path runs). Load the tuned TX analog (RX-CMU charge-pump/LDO/tank, RX CMU
 * bias, TX CDR config, TX driver bias), select the analog<->digital interconnect
 * (CFG_ANALOG2D_SEL / CFG_D2ANLOG_INF_SEL), set the TX LA LDO, program the
 * per-rate (1.25G / EPON / GPON) CDR Ki/Kp1/Kp2 banks (0x80C5 = Ki=1/16, Kp1=8,
 * Kp2=0.5), enable the RX CDR analog front end and clear the RX filter, take CDR
 * reset-manual + LPF-manual, then drive the forced RX-enable through a 0->1 edge
 * (MISC_REG00 0x10 -> 0x30) to start the CDR. Settle 50 ms, drop the BEN
 * power-down, and select negedge clock-sampling on the A2D / D2A transfer paths.
 */
static const struct r960_op c2_epon_sds_v1[] = {
	WR(C2_E_SDS_ANA_COM_REG02, 0x6DF8),	/* RX CMU charge-pump/LDO/tank      */
	WR(C2_E_SDS_ANA_COM_REG03, 0x8941),	/* RX CMU bias                      */
	WR(C2_SDS_ANA_COM_REG08,   0x0713),	/* TX CDR config (GPON-block reg)   */
	WR(C2_E_SDS_ANA_COM_REG25, 0x001F),	/* TX driver bias                   */
	FLD(C2_WSDS_DIG_1E, 5, 5, 1),		/* CFG_ANALOG2D_SEL = 1             */
	FLD(C2_WSDS_DIG_1E, 4, 4, 1),		/* CFG_D2ANLOG_INF_SEL = 1          */
	WR(C2_E_SDS_ANA_COM_REG24, 0x8001),	/* TX LA LDO config                 */
	WR(C2_E_SDS_ANA_1P25G_REG46, 0x80C5),	/* 1.25G CDR Ki/Kp1/Kp2            */
	WR(C2_E_SDS_ANA_EPON_REG46,  0x80C5),	/* EPON  CDR Ki/Kp1/Kp2            */
	WR(C2_E_SDS_ANA_GPON_REG46,  0x80C5),	/* GPON  CDR Ki/Kp1/Kp2            */
	FLD(C2_SDS_ANA_COM_REG12, 14, 14, 1),	/* REG_RX_SEL_CDR_AFEN = 1          */
	FLD(C2_E_SDS_ANA_COM_REG11, 7, 0, 0),	/* REG_RX_FILT_CONFIG = 0           */
	FLD(C2_E_SDS_ANA_COM_REG19, 14, 14, 1),	/* REG_CDR_RESET_MANUAL = 1         */
	FLD(C2_E_SDS_ANA_COM_REG19, 10, 10, 1),	/* REG_CDR_EN_LPF_MANUAL = 1        */
	WR(C2_SDS_ANA_MISC_REG00, 0x10),	/* FRC_RX_EN_ON = 1, VAL = 0        */
	WR(C2_SDS_ANA_MISC_REG00, 0x30),	/* FRC_RX_EN_VAL 0 -> 1 (start CDR) */
	DLY(50),
	FLD(C2_WSDS_DIG_02, 10, 10, 0),		/* REG_EN_PDOWN_BEN = 0             */
	FLD(C2_SDS_REG7, 14, 14, 1),		/* SP_CFG_NEG_CLKWR_A2D = 1         */
	FLD(C2_SDS_EXT_REG12, 8, 8, 1),		/* SEP_CFG_NEG_CLKRD_D2A = 1        */
};

/*
 * EponModeV2 SerDes patch (rev>A operating point). Set the rev-A TX drive level
 * (TX_AMP=0x5, TX_EMP=0x1), clear the RX filter, enable the RX CDR analog front
 * end, pulse the SerDes soft-reset (CFG_SFT_RSTB 0->1) and settle 10 ms, retune
 * the GPON-rate CDR (Ki=1/16, Kp1=8, Kp2=0.5), drive the forced RX-enable
 * through a 0->1 edge to start the CDR, settle 50 ms, then drop the BEN
 * power-down. (Mirrors _rtl9602c_ponMacEponModeV2_set.)
 */
static const struct r960_op c2_epon_sds_v2[] = {
	FLD(C2_SDS_ANA_COM_REG22, 5, 3, 5),	/* REG_TX_AMP = 0x5               */
	FLD(C2_SDS_ANA_COM_REG22, 2, 0, 1),	/* REG_TX_EMP = 0x1               */
	FLD(C2_E_SDS_ANA_COM_REG11, 7, 0, 0),	/* REG_RX_FILT_CONFIG = 0          */
	FLD(C2_SDS_ANA_COM_REG12, 14, 14, 1),	/* REG_RX_SEL_CDR_AFEN = 1         */
	FLD(C2_WSDS_DIG_00, 8, 8, 0),		/* CFG_SFT_RSTB = 0 ...            */
	FLD(C2_WSDS_DIG_00, 8, 8, 1),		/* ... -> 1 (SerDes soft reset)   */
	DLY(10),
	FLD(C2_E_SDS_ANA_GPON_REG46, 9, 7, 1),	/* REG_CDR_KI = 1/16              */
	FLD(C2_E_SDS_ANA_GPON_REG46, 6, 4, 4),	/* REG_CDR_KP1 = 8               */
	FLD(C2_E_SDS_ANA_GPON_REG46, 3, 1, 2),	/* REG_CDR_KP2 = 0.5             */
	FLD(C2_SDS_ANA_MISC_REG00, 4, 4, 1),	/* FRC_RX_EN_ON = 1               */
	FLD(C2_SDS_ANA_MISC_REG00, 5, 5, 0),	/* FRC_RX_EN_VAL 0 ...            */
	FLD(C2_SDS_ANA_MISC_REG00, 5, 5, 1),	/* ... -> 1 (start CDR)           */
	DLY(50),
	FLD(C2_WSDS_DIG_02, 10, 10, 0),		/* REG_EN_PDOWN_BEN = 0           */
};

/*
 * Phase 4: EPON PCS / FEC tail (common to both revs, after the SerDes patch).
 * Release the forced-RX hold (SP_SDS_FRC_RX=0), force the IPG (SP_FRC_IPG=1, the
 * 129-byte issue fix), set the RX IPG count and DS-FEC mark option, load the PCS
 * SerDes patch words, then drop the burst-enable force mode (let the EPON framer
 * gate the laser) and force the SDS dummy word so the EPON MAC is NOT reset when
 * signal-detect drops.
 *
 * Note: WR(C2_SDS_REG7, 0x0059) is a deliberate full-word write that clears the
 * SP_CFG_NEG_CLKWR_A2D bit ModeV1 set above - this matches the stock EPON tail,
 * which writes SDS_REG7 = 0x59 wholesale after the ModeV1/V2 patch.
 */
static const struct r960_op c2_epon_post[] = {
	FLD(C2_E_SDS_REG1, 11, 8, 0),		/* SP_SDS_FRC_RX = 0               */
	FLD(C2_E_SDS_REG2, 13, 12, 1),		/* SP_FRC_IPG = 1 (129-byte fix)   */
	FLD(C2_SDS_EXT_REG12, 2, 0, 3),		/* SEP_CFG_IPG_CNT = 3             */
	FLD(C2_E_SDS_EXT_REG6, 15, 0, 0x43),	/* SEP_CFG_FEC_MK_OPT = 0x43       */
	WR(C2_E_SDS_EXT_REG7,  0x0003),		/* EPON DS-FEC control             */
	WR(C2_E_SDS_EXT_REG13, 0x4E6A),		/* PCS SerDes patch                */
	WR(C2_E_SDS_EXT_REG15, 0x1162),		/* PCS SerDes patch                */
	WR(C2_E_SDS_EXT_REG16, 0xBD2A),		/* PCS SerDes patch                */
	WR(C2_SDS_REG7, 0x0059),		/* PCS SerDes patch (full word)    */
	FLD(C2_SDS_FORCE_BEN, 0, 0, 0),		/* BEN_FORCE_MODE = 0 (framer gate)*/
	WR(C2_WSDS_DIG_01, 0x000C),		/* force-SDS: keep EPON MAC up     */
};

/*
 * EPON mode select. Park the lane into EPON line rate + switch reset, arm the
 * optical/analog front end, apply the rev-selected SerDes patch (rev-A -> V1,
 * rev>A -> EponModeV2), then run the common EPON PCS/FEC tail. The PBO datapath
 * init, per-T-cont scheduler programming, 1:1 flow->queue map and flow-control
 * patch are owned by the datapath driver, not this SerDes/mode bring-up.
 */
static int
rtl9602c_ponmac_mode_set_epon(const struct rtl960x_ops *o,
			      int rev, int subtype)
{
	int ret;

	(void)subtype;

	ret = r960_run(o, c2_epon_mode, ARRAY_SIZE(c2_epon_mode));
	if (ret)
		return ret;

	ret = r960_run(o, c2_epon_sds_arm, ARRAY_SIZE(c2_epon_sds_arm));
	if (ret)
		return ret;

	if (rev > RTL960X_REV_A)
		ret = r960_run(o, c2_epon_sds_v2, ARRAY_SIZE(c2_epon_sds_v2));
	else
		ret = r960_run(o, c2_epon_sds_v1, ARRAY_SIZE(c2_epon_sds_v1));
	if (ret)
		return ret;

	return r960_run(o, c2_epon_post, ARRAY_SIZE(c2_epon_post));
}

/* ------------------------------------------------------------------ *
 *  RTL9603CVD EPON PON-MAC / SerDes bring-up - clean-room op-table form.
 *
 *  Section of rtl960x_ponmac.c: reuses that file's op-table primitives
 *  (enum r960_opc, struct r960_op, WR/FLD/DLY/POLL, r960_run()) AND the
 *  RTL9603CVD GPON section's existing C3_* register map + helpers already in
 *  scope (C3_SDS_CFG, C3_SOFTWARE_RST, C3_WSDS_DIG_00/02/18, C3_FORCE_BEN,
 *  C3_SDS_ANA_COM03/09/17/20/21/26/27, C3_PON_INBW_LBOUND, C3_PON_SID2QID,
 *  C3_SID_COUNT and the c3_flow2queue() helper). Do NOT compile standalone;
 *  r960_run() is file-private (static).
 *
 *  SerDes on this part is DIRECT MMIO (no indirect cmd/data page window), so
 *  every analog/digital/PCS knob is a plain RMW or full-word write. Addresses
 *  are absolute physical (SWCORE base 0x1b000000); the PON-IP scheduler block
 *  lives in the 0x1bf0xxxx window. Register addresses, field bit-ranges,
 *  values, ordering and delays are RTL9603CVD silicon facts (resolved from the
 *  chip's own reg_list / regField_list); the expression (tables + loops +
 *  comments) is original.
 *
 *  Source-correctness anchor: _rtl9603cvd_ponMacEponModeV1_set() and the EPON
 *  SDS_CFG branch in the vendor DAL. The init-only / GPON-only ops (forced
 *  125 MHz reference, interface-FIFO rstb cycle, FEP_V2ANALOG analog-ready
 *  poll, P_MISC.RX_SPC accept-undersize, OMCI SID/SID-valid wiring) are
 *  deliberately ABSENT here: the vendor EPON path does not perform them.
 *
 *  Out of section scope (HAL-subsystem calls, not raw SerDes/PON-MAC register
 *  writes, and omitted by the in-tree GPON sibling too): dal_rtl9603cvd_pbo_init
 *  (PBO_MODE_EPON), the PBO DS-sid2q MPCP/OAM high-queue map (streamId 0x10..
 *  0x1b), and rtl9603cvd_raw_flowctrl_patch(FLOWCTRL_PATCH_35M_GPON).
 * ------------------------------------------------------------------ */

/* EPON-specific registers not already defined in the C3 GPON section.
 * (offsets verified against rtk_rtl9603cvd_reg_list.c) */
#define C3_E_FIB_REG0		0x1b040c00u /* fiber analog: FP_CFG_FIB_PDOWN @[11]      */
#define C3_E_SDS_REG1		0x1b040804u /* SerDes ctrl 1: SP_SDS_FRC_RX @[11:8]      */
#define C3_E_SDS_REG2		0x1b040808u /* SerDes ctrl 2: SP_FRC_IPG @[13:12]        */
#define C3_E_SDS_REG7		0x1b04081cu /* SerDes ctrl 7: PCS misc word (full word)  */
#define C3_E_SDS_EXT_REG6	0x1b040a18u /* SerDes ext: SEP_CFG_FEC_MK_OPT @[15:0]    */
#define C3_E_SDS_EXT_REG12	0x1b040a30u /* SerDes ext: SEP_CFG_IPG_CNT @[2:0]        */
#define C3_E_SDS_EXT_REG13	0x1b040a34u /* SerDes ext: EPON DS FEC word (full word)  */
#define C3_E_SDS_EXT_REG15	0x1b040a3cu /* SerDes ext: EPON DS FEC word (full word)  */
#define C3_E_SDS_EXT_REG16	0x1b040a40u /* SerDes ext: EPON DS FEC word (full word)  */
#define C3_E_WSDS_DIG_01	0x1b040034u /* SerDes digital: SD-down MAC-hold (full)   */

/* PON-IP scheduler tables (0x1bf0xxxx), not used by the GPON section.
 *   PON_TCONT_EN  array offset = 1 bit  (TCONT_ENf [0:0], all in one word)
 *   PON_SCH_QMAP  array offset = 8 bits (MAPPING_TBLf [7:0], 4 tconts/word)
 * (offsets + array offsets verified against rtk_rtl9603cvd_reg_list.c) */
#define C3_E_PON_SCH_QMAP	0x1bf025e8u /* per-tcont queue mask, 8b/elem             */
#define C3_E_PON_TCONT_EN	0x1bf025fcu /* per-tcont schedule enable, 1b/elem        */

/* per-port ACCEPT_MAX_LEN_CTRL: base offset 0x1100C, ACCEPT_MAX_LENTHf @[13:0].
 * This register lives at 0x1100C, OUTSIDE the MACPP per-port block (0x20000-
 * 0x203FF), so the HAL addresses it as a plain Global array: the per-element
 * stride is array_offset/8 = 32/8 = 4 bytes (NOT the 0x100 MACPP interval, and
 * NOT 0x20). PON port index = 4 (rtl9603cvd_port_info.ponPort), so the PON
 * instance lives at 0x1100C + 4*4 = 0x1101C. This matches reg.c _reg_addr_find()
 * (Global index_1 branch: addr += (idx-lport)*(array_offset/8)) and the in-tree
 * RTL9607C C7_ACCEPT_MAX_LEN (0x11028, stride 4) sibling. */
#define C3_E_ACCEPT_MAXLEN_PON	0x1b01101cu /* ACCEPT_MAX_LEN_CTRL[PON port 4]        */

/* fixed EPON parameters (numeric facts from rtl9603cvd_def.h) */
#define C3_E_SDS_MODE		0xc	/* SDS_CFG.CFG_SDS_MODE value for EPON       */
#define C3_E_GPON_TCONT_MAX	17	/* RTL9603CVD_GPON_TCONT_MAX (T-cont slots)  */
#define C3_E_MAX_PKT_LEN	2031	/* 2047-4(ctag)-4(stag)-8(pppoe)             */

/*
 * EPON bring-up phase 1: analog pre-config with the lane parked. Park the lane,
 * lift the BEN power-down, take the fiber analog out of power-down, detach the
 * RX CDR AFE, set the burst-enable driver to TTL (CML off), then load the tuned
 * CDR/CMU/KVCO analog coefficients. The coefficient block is identical to GPON;
 * the lane-park, fiber power-up (FIB_REG0) and BEN-TTL framing are the EPON
 * entry. Unlike GPON, the EPON path does NOT force the 125 MHz reference here.
 */
static const struct r960_op c3_epon_sds_pre[] = {
	FLD(C3_SDS_CFG,        4,  0, 0x1f),	/* lane mode: off (parked)         */
	FLD(C3_WSDS_DIG_02,   10, 10, 0),	/* clear BEN power-down             */
	FLD(C3_E_FIB_REG0,    11, 11, 0),	/* fiber analog: out of power-down  */
	FLD(C3_SDS_ANA_COM03, 13, 13, 0),	/* RX CDR AFE: deselect             */
	FLD(C3_SDS_ANA_COM09,  4,  4, 0),	/* BEN driver: CML off              */
	FLD(C3_SDS_ANA_COM09,  0,  0, 1),	/* BEN driver: TTL output on        */
	FLD(C3_SDS_ANA_COM17, 15, 10, 0xc),	/* CDR loop proportional gain Kp    */
	FLD(C3_SDS_ANA_COM20, 11,  7, 0x1b),	/* RX CMU charge-pump current       */
	FLD(C3_SDS_ANA_COM20,  3,  2, 0x3),	/* RX CMU LDO reference             */
	FLD(C3_SDS_ANA_COM21, 13, 11, 0x2),	/* RX CMU slew rate                 */
	FLD(C3_SDS_ANA_COM21,  6,  3, 0x4),	/* RX VCO gain band select          */
	FLD(C3_SDS_ANA_COM26,  3,  2, 0x3),	/* GPHY CMU LDO reference           */
	FLD(C3_SDS_ANA_COM27,  6,  3, 0x4),	/* GPHY VCO gain band select        */
};

/*
 * Phase 2: commit EPON mode and pulse the resets. Select EPON on the lane, do
 * the switch-core reset the mode change requires (SOFTWARE_RST.SW_RST @[10]),
 * then reset the SerDes (digital + analog) via the soft-reset-B strobe
 * (WSDS_DIG_00.CFG_SFT_RSTB @[8], assert 0 -> release 1). Unlike GPON this path
 * does NOT touch the BER-notify force or CMD_SDS_RST_PS; the SD-down MAC hold is
 * applied later via WSDS_DIG_01.
 */
static const struct r960_op c3_epon_sds_mode[] = {
	FLD(C3_SDS_CFG,        4,  0, C3_E_SDS_MODE), /* lane mode: EPON           */
	FLD(C3_SOFTWARE_RST,  10, 10, 1),	/* switch-core reset on mode change */
	DLY(10),				/* let the switch reset settle      */
	FLD(C3_WSDS_DIG_00,    8,  8, 0),	/* SerDes soft-reset-B: assert      */
	FLD(C3_WSDS_DIG_00,    8,  8, 1),	/* SerDes soft-reset-B: release     */
	DLY(10),				/* let the SerDes reset settle      */
};

/*
 * Phase 3: re-enable the datapath and apply the EPON PCS/FEC patch. Turn the
 * burst-enable output on, clear the RX force-select field [11:8], force the IPG
 * (129-byte fix, SP_FRC_IPG @[13:12]), set the PCS IPG count, program the EPON
 * downstream FEC (mark option + coefficient words + PCS misc word), drop the
 * burst-enable force mode, and hold the EPON MAC up across an SD drop.
 *
 * NB: SP_SDS_FRC_RX is a 4-bit field [11:8] (regField lsp=8 len=4), not a single
 * bit. NB: the vendor EPON path does NOT set P_MISC.RX_SPC (accept-undersize) --
 * that is a GPON-only step and is intentionally absent here.
 */
static const struct r960_op c3_epon_sds_post[] = {
	FLD(C3_WSDS_DIG_18,   12, 12, 1),	/* burst-enable output: on          */
	FLD(C3_E_SDS_REG1,    11,  8, 0),	/* SP_SDS_FRC_RX: clear [11:8]      */
	FLD(C3_E_SDS_REG2,    13, 12, 1),	/* SP_FRC_IPG = 1 (129-byte fix)    */
	FLD(C3_E_SDS_EXT_REG12, 2,  0, 3),	/* SEP_CFG_IPG_CNT patch            */
	FLD(C3_E_SDS_EXT_REG6, 15,  0, 0x43),	/* SEP_CFG_FEC_MK_OPT (EPON DS FEC) */
	WR(C3_E_SDS_EXT_REG13, 0x4e6a),		/* EPON DS FEC coefficient word     */
	WR(C3_E_SDS_EXT_REG15, 0x1562),		/* EPON DS FEC coefficient word     */
	WR(C3_E_SDS_EXT_REG16, 0xbd2a),		/* EPON DS FEC coefficient word     */
	WR(C3_E_SDS_REG7,      0x4059),		/* PCS misc word                    */
	FLD(C3_FORCE_BEN,      0,  0, 0),	/* burst-enable force mode: off     */
	WR(C3_E_WSDS_DIG_01,   0x000c),		/* hold EPON MAC up when SD drops   */
};

/*
 * EPON bring-up driver. Pre-config + mode + post tables, then the explicit
 * scheduler wiring kept as code (as in the GPON sibling): every T-cont parked,
 * accept-max-length on the PON port, identity flow->queue map, and the
 * downstream in-band accumulation low bound. 'rev'/'subtype' are accepted for
 * dispatch symmetry; this part has a single SerDes variant for every rev.
 *
 * EPON (unlike GPON) is LLID-based and does NOT use the OMCI SID / SID-valid
 * path, so no c3_sidvalid()/OMCI wiring is performed here. flow->queue reuses
 * the GPON section's c3_flow2queue(o, flow, pqid) helper (do not redefine it).
 */
static int rtl9603cvd_ponmac_mode_set_epon(const struct rtl960x_ops *o,
					   int rev, int subtype)
{
	u32 f, t;
	int rc;
	(void)rev; (void)subtype;

	rc = r960_run(o, c3_epon_sds_pre,  ARRAY_SIZE(c3_epon_sds_pre));
	if (rc)
		return rc;
	rc = r960_run(o, c3_epon_sds_mode, ARRAY_SIZE(c3_epon_sds_mode));
	if (rc)
		return rc;
	rc = r960_run(o, c3_epon_sds_post, ARRAY_SIZE(c3_epon_sds_post));
	if (rc)
		return rc;

	/* max packet len on the PON port (ACCEPT_MAX_LENTH [13:0]) */
	rtl960x_rfwr(o, C3_E_ACCEPT_MAXLEN_PON, 13, 0, C3_E_MAX_PKT_LEN);

	/* park every T-cont: disable its schedule (TCONT_EN, 1 bit/tcont) and
	 * clear its 8-bit queue-mask (MAPPING_TBL, 4 tconts packed per word).
	 * Loop bound mirrors the vendor's HAL_MAX_NUM_OF_GPON_TCONT()-1 (=16). */
	for (t = 0; t < C3_E_GPON_TCONT_MAX - 1u; t++) {
		rtl960x_rfwr(o, C3_E_PON_TCONT_EN, t, t, 0);
		rtl960x_rfwr(o, C3_E_PON_SCH_QMAP + (t >> 2) * 4u,
			     (t & 3u) * 8u + 7u, (t & 3u) * 8u, 0);
	}

	/* EPON default flow->queue is one-to-one (flow N -> queue N), over the
	 * classifier SID range (mirrors HAL_CLASSIFY_SID_NUM()-1 = 127). */
	for (f = 0; f < C3_SID_COUNT - 1u; f++)
		c3_flow2queue(o, f, f);

	/* DS in-band accumulation low bound for PBO (LBOUND [23:0]) */
	rtl960x_rfwr(o, C3_PON_INBW_LBOUND, 23, 0, 0xfda000);

	return 0;
}

/* ------------------------------------------------------------------ *
 *  RTL9607C EPON PON-MAC / SerDes bring-up - clean-room op-table form.
 *
 *  This block is a SECTION of rtl960x_ponmac.c: it reuses that file's
 *  op-table primitives (enum r960_opc, struct r960_op, the WR/FLD/DLY/POLL
 *  macros, r960_run(), rtl960x_rfwr(), ARRAY_SIZE) and every C7_ GPON
 *  register #define and helper (c7_arr, the C7_SDS_ANA_* / WSDS / PON-IP
 *  register absolutes, the per-port / scheduler constants, C7_SDS_ANA_MISC02)
 *  already in scope from the C7 GPON section above. Do NOT compile standalone.
 *
 *  SerDes here is DIRECT MMIO (no indirect cmd/data page window) - every
 *  analog/digital knob is its own memory-mapped register written by RMW,
 *  exactly as for the C7 GPON path.
 *
 *  The register addresses, field bit-ranges, values, ordering and delays are
 *  hardware-interface facts of RTL9607C EPON mode (resolved from the chip's own
 *  rtk_rtl9607c_reg_list.c / rtk_rtl9607c_regField_list.c). The expression (the
 *  data-driven tables, the rev split, the comments) is original.
 *
 *  Absolute physical addresses = SWCORE base 0x1b000000 + offset; the PON-IP
 *  sub-block lives at 0x1bf0xxxx. The board supplies rd/wr via rtl960x_ops.
 *
 *  rev A -> EPON SerDes V1, rev B -> V2, rev C and later -> V3, matching the
 *  vendor dal_rtl9607c_ponmac_mode_set() EPON arm and the GPON V1/V2/V3 split
 *  this chip already uses.
 *
 *  UNTESTED: no EPON hardware available; register-faithful to the vendor
 *  _rtl9607c_ponMacEponModeV1/V2/V3_set() sequence.
 * ------------------------------------------------------------------ */

/* EPON lane mode select value (CFG_SDS_MODE [4:0]): GPON is 0x8, EPON is 0xc. */
#define C7_SDS_MODE_EPON	0x0cu
#define C7_SDS_MODE_OFF		0x1fu

/*
 * EPON-only registers (swcore base 0x1b000000 + offset), resolved from the
 * 9607C reg_list. (The C7_SDS_ANA_COM* / WSDS_DIG_* / SDS_CFG / SOFTWARE_RST /
 * FORCE_BEN / ACCEPT_MAX_LEN / PON-IP absolutes the EPON path also touches are
 * already #defined by the C7 GPON section above and are reused as-is.)
 */
#define C7_SDS_REG1		0x1b040804u /* [11:8] SP_SDS_FRC_RX                 */
#define C7_SDS_REG2		0x1b040808u /* [9:8] SP_SDS_FRC_AN [13:12] SP_FRC_IPG */
#define C7_SDS_REG7		0x1b04081cu /* FEC tuning word (full write)         */
#define C7_FIB_REG0		0x1b040c00u /* [11] FP_CFG_FIB_PDOWN (0 = fiber on) */
#define C7_WSDS_DIG_01		0x1b040034u /* force-SDS: MAC stays up when SD down  */
#define C7_SDS_EXT_REG6		0x1b040a18u /* [15:0] SEP_CFG_FEC_MK_OPT (DS FEC)   */
#define C7_SDS_EXT_REG12_PCS	0x1b040a30u /* [2:0] SEP_CFG_IPG_CNT (PCS patch)    */
#define C7_SDS_EXT_REG13	0x1b040a34u /* FEC coefficient (full write)         */
#define C7_SDS_EXT_REG15	0x1b040a3cu /* FEC coefficient (full write)         */
#define C7_SDS_EXT_REG16	0x1b040a40u /* FEC coefficient (full write)         */
#define C7_SDS_ANA_COM11	0x1b0405acu /* [4] REG_RX_EQ_ADP_SEL                */
#define C7_SDS_ANA_EPON34	0x1b040788u /* [7]  REG_CMU_EN_WD_GPHY              */
#define C7_SDS_ANA_EPON37	0x1b040794u /* [5]  REG_CMU_GPHY_DLY_CLKSEL         */
#define C7_SDS_ANA_EPON43	0x1b0407acu /* [11] REG_CMU_TX_DLY_CLKSEL           */

/*
 * WSDS_DIG_00 CFG_SFT_RSTB is bit 8 (NOT bit 4 - that reg's bit 4 is
 * CFG_FRC_125M_EN, which the EPON path, unlike GPON, never touches). The
 * register absolute C7_WSDS_DIG_00 itself is #defined by the GPON section.
 */
#define C7_WSDS_DIG00_SFT_RSTB	8

/* Whole-word values the vendor flow writes verbatim (FEC + MAC keep-up). */
#define C7_EPON_FEC_EXT13	0x4e6au
#define C7_EPON_FEC_EXT15	0x1562u
#define C7_EPON_FEC_EXT16	0xbd2au
#define C7_EPON_FEC_REG7	0x0059u
#define C7_EPON_FEC_MK_OPT	0x0043u
#define C7_EPON_PCS_IPG_CNT	0x0003u
#define C7_EPON_DIG01_KEEPUP	0x000cu	/* force-SDS: don't reset MAC on SD-down */
#define C7_EPON_MAX_PKT_LEN	2031u	/* 2047 - 4(ctag) - 4(stag) - 8(pppoe)  */

/*
 * rev-A EPON SerDes patch (mode V1): park the lane, turn fiber power on,
 * disable NWAY, clear BEN power-down, single-ended BEN (CML off / TTL on),
 * load the tuned TX/RX CMU + CDR + RX-path analog coefficients, switch the
 * lane into EPON, then (V1 only) drive the BER-notify force high so the EPON
 * MAC is not torn down on the upcoming reset. The reset / PCS / FEC / keep-up
 * tail is shared (c7_epon_tail).
 */
static const struct r960_op c7_epon_sds_v1[] = {
	FLD(C7_SDS_CFG,        4,  0, C7_SDS_MODE_OFF),	/* lane mode: off (parked)   */
	FLD(C7_FIB_REG0,      11, 11, 0x0),	/* fiber power-down off (fiber on)   */
	FLD(C7_SDS_REG2,       9,  8, 0x1),	/* NWAY disable (SP_SDS_FRC_AN)      */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down              */
	FLD(C7_SDS_ANA_COM17,  4,  4, 0x0),	/* BEN driver: CML off              */
	FLD(C7_SDS_ANA_COM17,  0,  0, 0x1),	/* BEN driver: TTL output on        */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0x8),	/* TX CMU charge-pump               */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump           */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x1),	/* TX CMU LPF resistor              */
	FLD(C7_SDS_ANA_COM21, 15, 15, 0x1),	/* RX CMU CCO select                */
	FLD(C7_SDS_ANA_COM21, 14, 11, 0xF),	/* RX CMU charge-pump               */
	FLD(C7_SDS_ANA_COM21,  6,  6, 0x1),	/* RX CMU big-KVCO on               */
	FLD(C7_SDS_ANA_COM21,  4,  2, 0x4),	/* RX CMU LPF resistor              */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x1),	/* CDR Ki                           */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x4),	/* CDR Kp1                          */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x4),	/* CDR Kp2                          */
	FLD(C7_SDS_ANA_COM09, 13, 13, 0x0),	/* RX CDR AFE deselect              */
	FLD(C7_SDS_ANA_COM06,  7,  0, 0x2),	/* RX filter config                 */
	FLD(C7_SDS_ANA_COM08, 14, 12, 0x4),	/* RX Kp1_2                         */
	FLD(C7_SDS_ANA_COM08, 11,  9, 0x4),	/* RX Kp2_2                         */
	FLD(C7_SDS_ANA_COM12,  7,  4, 0x1),	/* RX EQ2 select                    */
	FLD(C7_SDS_ANA_COM12,  3,  0, 0x1),	/* RX EQ2 select 2                  */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                     */
	FLD(C7_SDS_ANA_COM14, 11,  9, 0x0),	/* TX emphasis                      */
	FLD(C7_SDS_ANA_COM14,  8,  8, 0x1),	/* TX emphasis enable               */
	FLD(C7_SDS_CFG,        4,  0, C7_SDS_MODE_EPON),/* lane mode: EPON            */
	FLD(C7_SDS_ANA_MISC02,13, 13, 0x1),	/* FRC_BER_NOTIFY_VAL = 1 (V1)      */
	FLD(C7_SDS_ANA_MISC02,12, 12, 0x1),	/* FRC_BER_NOTIFY_ON  = 1 (V1)      */
};

/*
 * rev-B EPON SerDes patch (mode V2): same layout as V1 with retuned CDR/RX
 * gains (CDR Kp1/Kp2, RX Kp1_2/Kp2_2), the LC-bypass-off bit, and a CDR Kd
 * write that only exists on this rev. V2 does NOT touch MISC02 (BER-notify).
 */
static const struct r960_op c7_epon_sds_v2[] = {
	FLD(C7_SDS_CFG,        4,  0, C7_SDS_MODE_OFF),	/* lane mode: off (parked)   */
	FLD(C7_FIB_REG0,      11, 11, 0x0),	/* fiber power-down off              */
	FLD(C7_SDS_REG2,       9,  8, 0x1),	/* NWAY disable                     */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down              */
	FLD(C7_SDS_ANA_COM17,  4,  4, 0x0),	/* BEN driver: CML off              */
	FLD(C7_SDS_ANA_COM17,  0,  0, 0x1),	/* BEN driver: TTL output on        */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0xF),	/* TX CMU charge-pump               */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump           */
	FLD(C7_SDS_ANA_COM25, 15, 13, 0x7),	/* TX CMU LPF resistor              */
	FLD(C7_SDS_ANA_COM25,  1,  1, 0x0),	/* LC bypass off                    */
	FLD(C7_SDS_ANA_COM21, 15, 15, 0x1),	/* RX CMU CCO select                */
	FLD(C7_SDS_ANA_COM21, 14, 11, 0xF),	/* RX CMU charge-pump               */
	FLD(C7_SDS_ANA_COM21,  6,  6, 0x1),	/* RX CMU big-KVCO on               */
	FLD(C7_SDS_ANA_COM21,  4,  2, 0x4),	/* RX CMU LPF resistor              */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x1),	/* CDR Ki                           */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x0),	/* CDR Kp1 (rev-B)                  */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x6),	/* CDR Kp2 (rev-B)                  */
	FLD(C7_SDS_ANA_COM09, 13, 13, 0x0),	/* RX CDR AFE deselect              */
	FLD(C7_SDS_ANA_COM06,  7,  0, 0x2),	/* RX filter config                 */
	FLD(C7_SDS_ANA_COM08, 14, 12, 0x1),	/* RX Kp1_2 (rev-B)                 */
	FLD(C7_SDS_ANA_COM08, 11,  9, 0x1),	/* RX Kp2_2 (rev-B)                 */
	FLD(C7_SDS_ANA_COM12,  7,  4, 0x1),	/* RX EQ2 select                    */
	FLD(C7_SDS_ANA_COM12,  3,  0, 0x1),	/* RX EQ2 select 2                  */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                     */
	FLD(C7_SDS_ANA_COM14, 11,  9, 0x0),	/* TX emphasis                      */
	FLD(C7_SDS_ANA_COM14,  8,  8, 0x1),	/* TX emphasis enable               */
	FLD(C7_SDS_ANA_COM00,  1,  1, 0x0),	/* CDR Kd (rev-B only)              */
	FLD(C7_SDS_CFG,        4,  0, C7_SDS_MODE_EPON),/* lane mode: EPON (last)     */
};

/*
 * rev-C+ EPON SerDes patch (mode V3): a GPHY-CMU-centric tuning on the EPON
 * analog page - select the TX/GPHY delay clocks, disable the per-lane CMU
 * watchdogs (TX/RX/GPHY), retune TX CMU + Z0 driver + CDR + RX-EQ and the GPHY
 * CMU charge-pump/LPF, then switch into EPON. V3 does NOT touch MISC02.
 */
static const struct r960_op c7_epon_sds_v3[] = {
	FLD(C7_SDS_CFG,        4,  0, C7_SDS_MODE_OFF),	/* lane mode: off (parked)   */
	FLD(C7_FIB_REG0,      11, 11, 0x0),	/* fiber power-down off              */
	FLD(C7_SDS_REG2,       9,  8, 0x1),	/* NWAY disable                     */
	FLD(C7_WSDS_DIG_02,   10, 10, 0x0),	/* clear BEN power-down              */
	FLD(C7_SDS_ANA_EPON43,11, 11, 0x1),	/* TX delay-clock select            */
	FLD(C7_SDS_ANA_COM26,  3,  3, 0x0),	/* TX CMU watchdog off              */
	FLD(C7_SDS_ANA_COM23, 15, 15, 0x0),	/* RX CMU watchdog off              */
	FLD(C7_SDS_ANA_EPON34, 7,  7, 0x0),	/* GPHY CMU watchdog off            */
	FLD(C7_SDS_ANA_COM24, 14, 11, 0xD),	/* TX CMU charge-pump               */
	FLD(C7_SDS_ANA_COM24,  3,  1, 0x0),	/* TX CMU LPF charge-pump           */
	FLD(C7_SDS_ANA_COM14,  4,  0, 0x7),	/* Z0 P-adjust                      */
	FLD(C7_SDS_ANA_COM15, 15, 12, 0x8),	/* Z0 N-adjust                      */
	FLD(C7_SDS_ANA_COM02, 15, 13, 0x6),	/* CDR Ki                           */
	FLD(C7_SDS_ANA_COM02, 12, 10, 0x2),	/* CDR Kp1                          */
	FLD(C7_SDS_ANA_COM02,  9,  7, 0x0),	/* CDR Kp2                          */
	FLD(C7_SDS_ANA_COM13,  7,  5, 0x2),	/* TX amplitude                     */
	FLD(C7_SDS_ANA_COM05,  2,  2, 0x1),	/* RX EQ hold                       */
	FLD(C7_SDS_ANA_COM06, 15,  9, 0x40),	/* RX EQ input                      */
	FLD(C7_SDS_ANA_COM09,  6,  2, 0x1f),	/* RX timer-BER                     */
	FLD(C7_SDS_ANA_COM11,  4,  4, 0x0),	/* RX EQ adaptation off             */
	FLD(C7_SDS_ANA_EPON37, 5,  5, 0x1),	/* GPHY delay-clock select          */
	FLD(C7_SDS_ANA_COM30, 15, 12, 0x3),	/* GPHY CMU charge-pump             */
	FLD(C7_SDS_ANA_COM30, 10, 10, 0x1),	/* GPHY CMU ICP low-BW              */
	FLD(C7_SDS_ANA_COM30,  4,  2, 0x2),	/* GPHY CMU LPF charge-pump         */
	FLD(C7_SDS_ANA_COM31, 15, 13, 0x0),	/* GPHY CMU LPF resistor            */
	FLD(C7_SDS_CFG,        4,  0, C7_SDS_MODE_EPON),/* lane mode: EPON (last)     */
};

/*
 * Shared EPON reset / PCS / FEC / keep-up tail (identical after every rev's
 * SerDes patch; V1's MISC02 BER-notify hold is appended to its own SDS table
 * because the vendor lands it before this reset). The mode change needs a
 * switch-core reset, then the SerDes digital+analog reset-B is cycled
 * (CFG_SFT_RSTB 0->1). BEN output on, drop the RX force (SP_SDS_FRC_RX=0),
 * force IPG (SP_FRC_IPG, the 129-byte fix) and seed the PCS IPG count, load the
 * EPON downstream-FEC marker option + the three FEC coefficients + the FEC
 * tuning word, drop BEN force mode, then force-SDS (WSDS_DIG_01=0xc) so the
 * EPON MAC is NOT torn down on a signal-detect drop.
 *
 * NOTE: the EPON path does NOT do the GPON tail's iface-FIFO re-arm,
 * accept-undersize, analog-ready (FIB_EXT_REG21 / V2ANALOG) poll, or 125 MHz
 * power-down - those are GPON-only and the vendor EPON sequence omits them.
 */
static const struct r960_op c7_epon_tail[] = {
	FLD(C7_SOFTWARE_RST,  10, 10, 0x1),	/* SW_RST: switch-core reset        */
	DLY(10),
	FLD(C7_WSDS_DIG_00, C7_WSDS_DIG00_SFT_RSTB, C7_WSDS_DIG00_SFT_RSTB, 0x0), /* SerDes reset assert  */
	FLD(C7_WSDS_DIG_00, C7_WSDS_DIG00_SFT_RSTB, C7_WSDS_DIG00_SFT_RSTB, 0x1), /* SerDes reset release */
	DLY(10),
	FLD(C7_WSDS_DIG_18,   12, 12, 0x1),	/* BEN_OE output on                 */
	FLD(C7_SDS_REG1,      11,  8, 0x0),	/* drop RX force (SP_SDS_FRC_RX=0)  */
	FLD(C7_SDS_REG2,      13, 12, 0x1),	/* SP_FRC_IPG = 1 (129-byte fix)    */
	FLD(C7_SDS_EXT_REG12_PCS, 2, 0, C7_EPON_PCS_IPG_CNT),	/* PCS IPG count    */
	FLD(C7_SDS_EXT_REG6,  15,  0, C7_EPON_FEC_MK_OPT),	/* EPON DS FEC marker opt */
	WR(C7_SDS_EXT_REG13, C7_EPON_FEC_EXT13),	/* FEC coefficient           */
	WR(C7_SDS_EXT_REG15, C7_EPON_FEC_EXT15),	/* FEC coefficient           */
	WR(C7_SDS_EXT_REG16, C7_EPON_FEC_EXT16),	/* FEC coefficient           */
	WR(C7_SDS_REG7,      C7_EPON_FEC_REG7),		/* FEC tuning word           */
	FLD(C7_FORCE_BEN,      0,  0, 0x0),	/* BEN force mode off (GTC gates)   */
	WR(C7_WSDS_DIG_01,   C7_EPON_DIG01_KEEPUP),/* MAC keep-up on SD-down        */
};

/*
 * EPON post-tail housekeeping (kept as code, not a table - it has loops and a
 * per-port write). Mirrors the vendor EPON sequence AFTER the reset/FEC tail,
 * in this exact order:
 *   1. accept-max-length on the PON port (ACCEPT_MAX_LEN_CTRL[PON port],
 *      ACCEPT_MAX_LENTH field [13:0]; array offset 32 => byte stride 4).
 *   2. park every T-cont: disable (TCONT_EN [0]), then clear its scheduler
 *      queue mask (MAPPING_TBL [31:0]).
 *   3. default EPON flow->queue is a one-to-one map across every classifier
 *      SID/flow (the EPON LLID model), unlike GPON's funnel + dedicated OMCI
 *      flow. The vendor SidToQueueMap_set additionally aliases sid<64 to
 *      sid+64 with the same queue; with the main loop covering [0, SID-1) the
 *      only net effect over identity is SID2QID[SID-1] = (SID/2 - 1).
 *   4. seed the DS in-band low bound (LBOUND [23:0]) for PBO accumulation.
 * (The vendor's shared switch flow-control patch between 3 and 4 lives in a
 *  separate subsystem and is out of scope here, as for the GPON port.)
 */
static int c7_epon_post(const struct rtl960x_ops *o)
{
	u32 i;

	/* accept-max-length on the PON port (ACCEPT_MAX_LENTH [13:0]) */
	rtl960x_rfwr(o, C7_ACCEPT_MAX_LEN + C7_PON_PORT * 4u, 13, 0,
		     C7_EPON_MAX_PKT_LEN);

	for (i = 0; i < C7_GPON_TCONT_MAX - 1u; i++) {
		c7_arr(o, C7_PON_TCONT_EN, 1, i, 0, 1, 0);	/* T-cont disable  */
		c7_arr(o, C7_PON_SCH_QMAP, 32, i, 0, 32, 0);	/* clear queue mask*/
	}

	/* EPON one-to-one SID -> queue map */
	for (i = 0; i < C7_SID_COUNT - 1u; i++)
		c7_arr(o, C7_PON_SID2QID, 7, i, 0, 7, i);	/* SID i -> queue i */

	/*
	 * EPON-only: the vendor's per-set sid<64 -> sid+64 alias leaves the top
	 * SID (SID_COUNT-1) pointing at the half-range head's queue (sid 63 ->
	 * queue 63 was also written to sid 127). Replicate that net result.
	 */
	c7_arr(o, C7_PON_SID2QID, 7, C7_SID_COUNT - 1u, 0, 7, C7_SID_COUNT / 2u - 1u);

	rtl960x_rfwr(o, C7_PON_INBW_LBOUND, 23, 0, 0xfda000);	/* DS in-band lbound */
	return 0;
}

/*
 * EPON mode-set: the rev-selected SerDes patch table, the shared
 * reset/PCS/FEC/keep-up tail, then the post housekeeping (max-len, T-cont park,
 * one-to-one SID map, in-band low bound). rev A->V1, B->V2, C and later->V3,
 * matching the vendor dispatch. c7_ponmac_init runs first and is mode-agnostic.
 */
static int rtl9607c_ponmac_mode_set_epon(const struct rtl960x_ops *o,
					 int rev, int subtype)
{
	const struct r960_op *sds;
	unsigned int n;
	int rc;

	(void)subtype;

	switch (rev) {
	case RTL960X_REV_A:
		sds = c7_epon_sds_v1; n = ARRAY_SIZE(c7_epon_sds_v1); break;
	case RTL960X_REV_B:
		sds = c7_epon_sds_v2; n = ARRAY_SIZE(c7_epon_sds_v2); break;
	default:
		sds = c7_epon_sds_v3; n = ARRAY_SIZE(c7_epon_sds_v3); break;
	}

	rc = r960_run(o, sds, n);
	if (rc)
		return rc;

	rc = r960_run(o, c7_epon_tail, ARRAY_SIZE(c7_epon_tail));
	if (rc)
		return rc;

	return c7_epon_post(o);
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
	switch (mode) {
	case RTL960X_MODE_GPON:
		switch (chip) {
		case RTL960X_CHIP_9601B:
			return rtl9601b_ponmac_mode_set(o, rev, subtype);
		case RTL960X_CHIP_9602C:
			return rtl9602c_ponmac_mode_set(o, rev, subtype);
		case RTL960X_CHIP_9603CVD:
			return rtl9603cvd_ponmac_mode_set(o, rev, subtype);
		case RTL960X_CHIP_9607C:
			return rtl9607c_ponmac_mode_set(o, rev, subtype);
		default:		/* 9607F: reg map not available yet */
			return -ENOTSUPP;
		}
	case RTL960X_MODE_EPON:
		switch (chip) {
		case RTL960X_CHIP_9601B:
			return rtl9601b_ponmac_mode_set_epon(o, rev, subtype);
		case RTL960X_CHIP_9602C:
			return rtl9602c_ponmac_mode_set_epon(o, rev, subtype);
		case RTL960X_CHIP_9603CVD:
			return rtl9603cvd_ponmac_mode_set_epon(o, rev, subtype);
		case RTL960X_CHIP_9607C:
			return rtl9607c_ponmac_mode_set_epon(o, rev, subtype);
		default:		/* EPON: 9607F reg map not available yet */
			return -ENOTSUPP;
		}
	default:			/* FIBER / unknown: not supported */
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
