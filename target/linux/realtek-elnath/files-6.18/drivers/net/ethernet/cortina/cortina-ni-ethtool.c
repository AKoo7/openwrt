// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" /
 * Taurus (AOT-5221Zy) - the STANDARD counter + register-snapshot interface:
 *
 *   ethtool -S <dev>   every countable quantity the driver and the NI can
 *                      report, as one flat name->value list
 *   ethtool -d <dev>   the curated NI-window register snapshot, as a flat u32
 *                      array, decodable through debugfs .../regdump_map
 *
 * WHY THIS EXISTS BESIDE /proc, NOT INSTEAD OF IT
 * -----------------------------------------------
 * A test may not depend on a /proc node carrying this driver's name, because
 * the VENDOR firmware has no node of that name - so a case that reads
 * /proc/net/cortina_ni_rx can only ever BLOCK on stock, and the oracle half of
 * the comparison is structurally impossible.  ethtool is served by both
 * firmwares' kernels, so the same case runs on both and the answers are
 * comparable.  /proc/net/cortina_ni_rx, /proc/net/cortina_ni_tx,
 * /proc/cortina_ni_peek, /proc/cortina_ni_gsram and /proc/cortina_l3fe ALL
 * STAY - every live probe in this project depends on them.
 *
 * ============================================================================
 * ★★★ PORT NOTE - THIS FILE IS *NOT* A COPY OF THE UPSTREAM ONE
 * ============================================================================
 * Adapted from upstream commit 7c3c1d4 to THIS tree.  Upstream's file targets
 * ELNATH register offsets.  On Taurus five of the registers it reads are
 * UNMAPPED HOLES and four of those FAULT THE CPU (synchronous external abort ->
 * kernel panic).  Porting it verbatim would have made `ethtool -S eth0` a
 * remote board-kill - strictly worse than the /proc reader it complements.
 * Every deviation below is recorded, with the fix number that established it:
 *
 *  1. THE NI_HV READ-AND-CLEAR BLOCK.  Upstream samples 0xa9bc (l3fe_rx),
 *     0xa9fc (l3qm_rx), 0xaa10 (l3qm_tx), 0xaa3c (mce_rx), 0xaa7c (dma_rx).
 *     On this die:
 *       0xa9bc  - HOLE.  fix#81: the generic map's NI_HV_INTPT_RX_PKT_CNT,
 *                 which Taurus RELOCATED to 0xa92c; the name "L3FE_RX" was
 *                 wrong as well (it is the NI INTERNAL-PORT rx count).  The
 *                 symbol CA_NI_NI_L3FE_RX_PKT_CNT does not exist in our
 *                 regs.h and MUST NOT be re-added.
 *       0xa9fc / 0xaa10 / 0xa9f4 / 0xa9f8 - named in NEITHER register map, and
 *                 a readl of them takes a synchronous external abort:
 *                 "Internal error: 0000000096000010 ... pc: readl+0x0 ...
 *                 Kernel panic".  cortina-ni-rx.c already refuses to read them
 *                 (it prints a literal 0 for l3qm_rx).  They are NOT sampled
 *                 here and must never be.
 *       0xaa3c  - decodes, but is Taurus NI_HV_XRAM_CPUXRAM_BYT_CNT_0, a BYTE
 *                 count.  Publishing it as "mce_rx_packets" would be a named
 *                 lie through the most authoritative interface we have.  DROPPED.
 *       0xaa7c  - decodes, but is Taurus NI_HV_MCE_LAST_IN_HDR2, a header
 *                 LATCH.  Accumulating a latch into a monotonic u64 produces
 *                 pure nonsense.  DROPPED; the real DMA packet count on this
 *                 die is CA_NI_NI_XRAM_DMA_PKT_CNT (0xaa4c) and that is what
 *                 is published.
 *     ⇒ the sampler below reads the five NAMED-on-Taurus internal-port
 *     counters (fix#42/#81), all five of which cortina-ni-rx.c already reads
 *     live on the board without faulting.
 *
 *  2. THE QM COUNTERS moved wholesale (fix#15): QM_RX_CNTR 0x6900 -> 0x67d8,
 *     QM_TX_CNTR 0x690c -> 0x67e4, RMU_NO_BUF_DROP 0x6940 -> 0x6818,
 *     RMU_FE_DROP 0x6944 -> 0x681c, RX_EOP_DROP 0x6948 -> 0x6820,
 *     RX_LEN_ERR 0x694c -> 0x6824, RX_L2TE_DROP 0x6950 -> 0x6828.  The table
 *     below names them BY SYMBOL, so it picks up our offsets automatically.
 *     ⚠ NEVER convert a row here to a literal offset.
 *
 *  3. THE PER-PORT MAC MIB.  Upstream reads it through 0xa168/0xa170 and
 *     OMITS the TX half, on a 2026-07-29 "every cell moved by zero" verdict.
 *     fix#80 RETRACTED that verdict: the sweep was reading PG_SA_CFG1/PC_DA2.
 *     The real Taurus MIB is RX 0xa1a4/0xa1ac and TX 0xa1b0/0xa1b8, and both
 *     are reached here through cortina-ni-rx.c's existing readers, so the
 *     access pair is never open-coded twice.  We therefore publish the per-port
 *     TX MIB that upstream could not.
 *
 *  4. THE `-d` SNAPSHOT IS 728 WORDS HERE, NOT UPSTREAM'S 872.  It is
 *     ARRAY_SIZE-derived from the two tables that already live in
 *     cortina-ni-rx.c beside the /proc reader that prints them - ONE list, so
 *     the dump and the /proc render cannot drift.  Our
 *     cortina_ni_qmdump_offs[] is 662 words rather than 806 because it is the
 *     MECHANICALLY DERIVED, hole-filtered list (a dense ELNATH sweep aborted
 *     the board at 0x6978 = QM_VOQ_STATUS8, which Taurus does not have).
 *     ⚠ Do not "fix" the length toward upstream and do not hard-code 872.
 * ============================================================================
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/stddef.h>
#include <linux/string.h>

#include "cortina-ni.h"
#include "cortina-ni-regs.h"

static inline void __iomem *ni_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_NI];
}

/* ------------------------------------------------------------------ */
/* the NI_HV read-and-clear counters: ONE reader, driver-side totals    */
/* ------------------------------------------------------------------ */

/*
 * The NI_HV internal-port packet counters, in enum cortina_ni_nihv_cnt order.
 * THESE ARE READ-AND-CLEAR (cortina-ni-regs.h: "with access opcode
 * CA_NI_MIB_OP_READ_ONLY it is CUMULATIVE (unlike the NI_HV_INTPT_* per-stage
 * counters, which are read-and-clear)"), which is why they may have exactly
 * one reader - see the contract in cortina-ni.h.
 *
 * Every offset here is a Taurus-NAMED register that cortina-ni-rx.c already
 * reads live on this board (cortina_ni_rx_delivery_dump's "DS-NI:" line and
 * the /proc reader's fwd-chain line).  The ELNATH set upstream samples is
 * deliberately absent: see PORT NOTE 1 at the top of this file.
 */
static const u32 cortina_ni_nihv_off[CA_NI_NIHV_CNT_COUNT] = {
	[CA_NI_NIHV_INTPT_RX]		= CA_NI_NI_INTPT_RX_PKT_CNT,	/* 0xa92c */
	[CA_NI_NIHV_INTPT_MISS_SOP_EOP]	= CA_NI_NI_INTPT_RX_MISS_SOP_EOP, /* 0xa924 */
	[CA_NI_NIHV_INTPT_SHORT_ERR]	= CA_NI_NI_INTPT_RX_SHORT_ERR,	/* 0xa928 */
	[CA_NI_NIHV_INTPT_TX]		= CA_NI_NI_INTPT_TX_PKT_CNT,	/* 0xa940 */
	[CA_NI_NIHV_XRAM_DMA]		= CA_NI_NI_XRAM_DMA_PKT_CNT,	/* 0xaa4c */
};

void cortina_ni_nihv_sample(struct cortina_ni *ni,
			    u64 out[CA_NI_NIHV_CNT_COUNT])
{
	void __iomem *base = ni_base(ni);
	unsigned long flags;
	unsigned int i;

	if (!base) {
		memset(out, 0, sizeof(*out) * CA_NI_NIHV_CNT_COUNT);
		return;
	}

	/*
	 * The readl and the fold are ONE critical section on purpose.  Split
	 * them and two concurrent readers can each take a slice of the same
	 * count and then publish totals that disagree about which slice each
	 * saw - the count is not lost, but the two answers are, which is the
	 * kind of "both numbers look plausible" failure this whole interface
	 * exists to remove.  Five MMIO reads under a spinlock, no sleeping.
	 */
	spin_lock_irqsave(&ni->nihv_lock, flags);
	for (i = 0; i < CA_NI_NIHV_CNT_COUNT; i++) {
		ni->nihv_total[i] += readl(base + cortina_ni_nihv_off[i]);
		out[i] = ni->nihv_total[i];
	}
	spin_unlock_irqrestore(&ni->nihv_lock, flags);
}

/* ------------------------------------------------------------------ */
/* `ethtool -S`: ONE table drives both the names and the values         */
/* ------------------------------------------------------------------ */

enum ca_ni_stat_src {
	CA_ST_RX_U64,		/* u64 at @arg bytes into struct cortina_ni_rx */
	CA_ST_TX_U64,		/* u64 at @arg bytes into struct cortina_ni_tx */
	CA_ST_NI_REG,		/* plain cumulative register at NI + @arg      */
	CA_ST_NIHV,		/* index @arg into the read-and-clear totals   */
	CA_ST_PORT_RXMIB,	/* per-port RX MIB, @arg = counter id, i = port*/
	CA_ST_PORT_TXMIB,	/* per-port TX MIB, @arg = counter id, i = port*/
	CA_ST_DRV_FLAG,		/* a driver state bit, @arg selects which      */
	CA_ST_L3FE,		/* index @arg into the offload snapshot        */
};

/* CA_ST_DRV_FLAG selectors */
#define CA_ST_FLAG_RX_UP	0

/*
 * One ROW may stand for a FAMILY of counters: @n repeats, with the repeat
 * index substituted into @fmt's single %u and added to @arg (scaled by @step
 * for the byte-offset sources).  get_strings() and get_ethtool_stats() walk
 * this one table in the same order, so a name and its value cannot drift apart
 * - which is the failure mode of the two-parallel-lists shape this replaces.
 * @n comes from the driver's own constants, so growing a family (another VoQ,
 * another port) cannot silently leave the extra members unnamed.
 *
 * ⚠ Every name must fit ETH_GSTRING_LEN (32) INCLUDING the substituted index
 * and the NUL; the longest here is 29.
 */
struct ca_ni_stat_grp {
	const char		*fmt;
	u16			n;
	enum ca_ni_stat_src	src;
	u32			arg;
	u32			step;
};

#define S_RX(f)		((u32)offsetof(struct cortina_ni_rx, f))
#define S_TX(f)		((u32)offsetof(struct cortina_ni_tx, f))

static const struct ca_ni_stat_grp cortina_ni_stat_grps[] = {
	/* ---- receive, the driver's own software counters ---------------- */
	{ "rx_datapath_up",		1, CA_ST_DRV_FLAG, CA_ST_FLAG_RX_UP },
	{ "rx_frames",			1, CA_ST_RX_U64, S_RX(frames) },
	{ "rx_bytes",			1, CA_ST_RX_U64, S_RX(bytes) },
	{ "rx_napi_polls",		1, CA_ST_RX_U64, S_RX(polls) },
	{ "rx_headerless_frames",	1, CA_ST_RX_U64, S_RX(swid_frames) },
	{ "rx_pon_omci_frames",		1, CA_ST_RX_U64, S_RX(pon_frames) },
	{ "rx_pon_wan_frames",		1, CA_ST_RX_U64, S_RX(wan_frames) },
	{ "rx_pon_wan_l3_miss_frames",	1, CA_ST_RX_U64, S_RX(wan_l3_frames) },
	{ "rx_deepq_frames",		1, CA_ST_RX_U64, S_RX(dq_frames) },
	{ "rx_drop_no_sop",		1, CA_ST_RX_U64, S_RX(drop_nosop) },
	{ "rx_drop_bad_paddr",		1, CA_ST_RX_U64, S_RX(drop_badpa) },
	{ "rx_drop_bad_len",		1, CA_ST_RX_U64, S_RX(drop_len) },
	{ "rx_drop_runt",		1, CA_ST_RX_U64, S_RX(drop_runt) },
	{ "rx_drop_oversize",		1, CA_ST_RX_U64, S_RX(drop_oversize) },
	{ "rx_drop_no_buffer",		1, CA_ST_RX_U64, S_RX(drop_nobuf) },
	{ "rx_slot_dead",		1, CA_ST_RX_U64, S_RX(slot_dead) },
	{ "rx_stale_buffer",		1, CA_ST_RX_U64, S_RX(stale_buf) },
	{ "rx_pool_push_fail",		1, CA_ST_RX_U64, S_RX(push_fail) },
	{ "rx_gphy_recoveries",		1, CA_ST_RX_U64, S_RX(recoveries) },
	{ "rx_link_rearms",		1, CA_ST_RX_U64, S_RX(rearms) },
	/* one flow must stay on ONE VoQ: two of these climbing during a
	 * unidirectional run means the hardware spread it and the fixed drain
	 * order can reorder the flow */
	{ "rx_voq%u_frames",	CA_NI_RX_VOQ_COUNT, CA_ST_RX_U64,
	  S_RX(voq_frames), sizeof(u64) },
	/* per-CPU-port EPP interrupt index (silicon), NOT the Linux IRQ number
	 * the board happened to allocate */
	{ "rx_epp_irq%u_events", CA_NI_RX_NUM_IRQS, CA_ST_RX_U64,
	  S_RX(irq_hits), sizeof(u64) },
	{ "rx_chain_frames",		1, CA_ST_RX_U64, S_RX(chain_frames) },
	{ "rx_chain_segments",		1, CA_ST_RX_U64, S_RX(chain_segs) },
	/* a HIGH-WATER MARK, not a count - named so nobody differences it */
	{ "rx_chain_max_segments",	1, CA_ST_RX_U64, S_RX(chain_max_segs) },
	{ "rx_chain_abort",		1, CA_ST_RX_U64, S_RX(chain_abort) },
	{ "rx_chain_reopen",		1, CA_ST_RX_U64, S_RX(chain_reopen) },
	{ "rx_chain_orphan",		1, CA_ST_RX_U64, S_RX(chain_orphan) },
	{ "rx_chain_bad_total",		1, CA_ST_RX_U64, S_RX(chain_badtotal) },
	{ "rx_chain_too_long",		1, CA_ST_RX_U64, S_RX(chain_toolong) },
	{ "rx_chain_short",		1, CA_ST_RX_U64, S_RX(chain_short) },
	{ "rx_chain_headerless",	1, CA_ST_RX_U64, S_RX(chain_swid) },
	{ "rx_chain_dlen_mismatch",	1, CA_ST_RX_U64, S_RX(chain_dlen_diff) },

	/* ---- transmit, the driver's own software counters ---------------- */
	{ "tx_drop_no_dma_map",		1, CA_ST_TX_U64, S_TX(drop_nomap) },
	{ "tx_drop_linearize",		1, CA_ST_TX_U64, S_TX(drop_linearize) },
	{ "tx_drop_oversize",		1, CA_ST_TX_U64, S_TX(drop_oversize) },
	{ "tx_ring_busy",		1, CA_ST_TX_U64, S_TX(tx_busy) },
	{ "tx_lan_fdb_hit",		1, CA_ST_TX_U64, S_TX(lan_hit) },
	{ "tx_lan_flood",		1, CA_ST_TX_U64, S_TX(lan_flood) },
	{ "tx_lan_flood_copies",	1, CA_ST_TX_U64, S_TX(lan_dup) },
	{ "tx_lan_fdb_learn",		1, CA_ST_TX_U64, S_TX(lan_learn) },
	{ "tx_lan_fdb_flush",		1, CA_ST_TX_U64, S_TX(lan_flush) },
	{ "tx_pon_omci_frames",		1, CA_ST_TX_U64, S_TX(pon_enq) },
	{ "tx_pon_omci_fail",		1, CA_ST_TX_U64, S_TX(pon_fail) },
	{ "tx_pon_wan_frames",		1, CA_ST_TX_U64, S_TX(pon_data_enq) },
	/* these stride the txq[] ARRAY, not a u64 array - the step is the
	 * struct size, which the compiler computes; never hand-write it */
	{ "tx_vp%u_enqueued",	CA_NI_TX_NUM_VPS, CA_ST_TX_U64,
	  S_TX(txq[0].enq), sizeof(struct cortina_ni_txq) },
	{ "tx_vp%u_reclaimed",	CA_NI_TX_NUM_VPS, CA_ST_TX_U64,
	  S_TX(txq[0].reclaimed), sizeof(struct cortina_ni_txq) },

	/*
	 * ---- per-port MAC RX MIB ----------------------------------------
	 * Read through the indirect ACCESS/DATA pair at the REAL Taurus
	 * addresses (fix#80: CA_NI_HV_RXMIB_ACCESS 0xa1a4 / DATA0 0xa1ac; the
	 * ELNATH 0xa168/0xa170 upstream uses is Taurus PG_SA_CFG1/PG_CFG1, and
	 * reading it returned the constant 0x81008100 on every port - which is
	 * what got this MIB written off as "a phantom").  ~0u means the GO poll
	 * never cleared, so a stuck instrument is VISIBLE rather than reading as
	 * a healthy zero.
	 */
	{ "port%u_mac_rx_unicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_RXMIB,
	  CA_NI_MIB_RX_UC_PKT },
	{ "port%u_mac_rx_multicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_RXMIB,
	  CA_NI_MIB_RX_MC_PKT },
	{ "port%u_mac_rx_broadcast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_RXMIB,
	  CA_NI_MIB_RX_BC_PKT },

	/*
	 * ---- per-port MAC TX MIB ----------------------------------------
	 * ★ OURS, AND DELIBERATELY NOT UPSTREAM'S SHAPE.  Upstream omits the TX
	 * MIB entirely, citing a measurement that fix#80 RETRACTED (that sweep
	 * was reading PG_FXPT_CFG/PC_DA2, not a MIB).  At the real Taurus
	 * addresses (0xa1b0/0xa1b8) this is the ONLY per-PHYSICAL-port egress
	 * packet counter on this silicon, so it is worth publishing.
	 *
	 * ⚠ THE COUNTER IDS 1/2/3 ARE A DERIVATION, NOT A MEASUREMENT (from the
	 * vendor table's size-bin anchor counter_id_TxStatsFrm65to127Oct = 0xf).
	 * The anchor is published as a fourth row precisely so a stock-vs-ours
	 * `ethtool -S` CONFIRMS the mapping instead of us trusting it: if the
	 * anchor row matches stock and the UC/MC/BC rows do not, the ids are
	 * wrong, not the addresses.
	 * ⚠ NOTE cortina-ni-rx.c's /proc loop still passes the RX ids (0/1/2) to
	 * cortina_ni_tx_mib_read() - a copy-paste slip.  Do not replicate it here.
	 */
	{ "port%u_mac_tx_unicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_TXMIB,
	  CA_NI_MIB_TX_UC_PKT },
	{ "port%u_mac_tx_multicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_TXMIB,
	  CA_NI_MIB_TX_MC_PKT },
	{ "port%u_mac_tx_broadcast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_TXMIB,
	  CA_NI_MIB_TX_BC_PKT },
	{ "port%u_mac_tx_frm65_127",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_TXMIB,
	  CA_NI_MIB_TX_FRM65_127 },

	/*
	 * ---- NI internal port, read-and-clear (accumulated) --------------
	 * TOTALS SINCE BOOT, not since the last read: the registers are
	 * read-and-clear and cortina_ni_nihv_sample() is their one reader (see
	 * the contract in cortina-ni.h).  A caller that wants a delta keeps its
	 * own snapshot of the TOTAL and subtracts; differencing the raw register
	 * is a delta of deltas.
	 *
	 * ★ NAMES DIFFER FROM UPSTREAM ON PURPOSE.  Upstream calls the first row
	 * "l3fe_rx_packets" from an ELNATH label that fix#81 proved wrong twice
	 * over - wrong ADDRESS (0xa9bc is a hole here) and wrong NAME (it is the
	 * NI internal-port count, not an L3FE count).  The retracted conclusion
	 * "l3fe_rx=0, so LAN frames never enter the L3FE" came from exactly that
	 * label; it must not be re-derivable from an ethtool string.
	 */
	{ "ni_intpt_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_INTPT_RX },
	{ "ni_intpt_rx_miss_sop_eop",	1, CA_ST_NIHV,
	  CA_NI_NIHV_INTPT_MISS_SOP_EOP },
	{ "ni_intpt_rx_short_err",	1, CA_ST_NIHV,
	  CA_NI_NIHV_INTPT_SHORT_ERR },
	{ "ni_intpt_tx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_INTPT_TX },
	{ "ni_xram_dma_packets",	1, CA_ST_NIHV, CA_NI_NIHV_XRAM_DMA },

	/* ---- engine, cumulative registers -------------------------------
	 * The datapath bisect, in flow order: L2FE ingest -> L2FE drop ->
	 * L2TM buffer manager -> QM/RMU admission -> QM drops.  The first that
	 * stops climbing while the one before it climbs is the death stage.
	 * 32-bit and free-running in hardware, so they wrap; difference two
	 * readings rather than trusting an absolute.
	 *
	 * Every offset comes from OUR regs.h SYMBOL, which is what makes the
	 * fix#15 QM remap (0x6900 -> 0x67d8 and six siblings) automatic here.
	 * ⚠ Never convert a row below to a literal.
	 */
	/* ⛔ THE TWO L2FE ROWS ARE UNPROVEN AND NAMED SO.  RB17 (2026-08-11)
	 * sampled them before and after 20 counted unicast pings + 10 broadcast
	 * ARPs and they did not change by one bit (0x11bc read a constant, 0x1234
	 * a constant 0xFF).  A pegged value that never moves under known-good
	 * traffic is not saturation, it is a register that is not a counter -
	 * the same failure mode as the pre-fix#80 MAC MIB.  Published as raw
	 * register reads so a stock-vs-ours diff can settle them, NOT as
	 * "drops": do not place a break point with these. */
	{ "l2fe_ni_intf_drop_raw",	1, CA_ST_NI_REG,
	  CA_NI_L2FE_NI_INTF_DROP_CNT },
	{ "l2fe_dos_flood_raw",		1, CA_ST_NI_REG,
	  CA_NI_L2FE_DOS_FLOOD_CNT },
	{ "l2tm_bm_rx_packets",		1, CA_ST_NI_REG, CA_NI_L2TM_BM_RX_PCNT },
	{ "l2tm_bm_tx_packets",		1, CA_ST_NI_REG, CA_NI_L2TM_BM_TX_PCNT },
	{ "l2tm_bm_drop_shared_buffer",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_SB_DPCNT },
	{ "l2tm_bm_drop_header",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_HDR_DPCNT },
	{ "l2tm_bm_drop_threshold",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_TE_DPCNT },
	{ "l2tm_bm_drop_error",		1, CA_ST_NI_REG, CA_NI_L2TM_BM_ERR_DPCNT },
	{ "l2tm_bm_drop_enqueue",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_RX_DPCNT },
	{ "l2tm_bm_drop_no_buffer",	1, CA_ST_NI_REG, CA_NI_L2TM_BM_NOBUF_DPCNT },
	{ "qm_rx_packets",		1, CA_ST_NI_REG, CA_NI_QM_RX_CNTR },
	{ "qm_tx_packets",		1, CA_ST_NI_REG, CA_NI_QM_TX_CNTR },
	{ "qm_drop_no_buffer",		1, CA_ST_NI_REG, CA_NI_QM_RMU_NO_BUF_DROP },
	{ "qm_drop_fe",			1, CA_ST_NI_REG, CA_NI_QM_RMU_FE_DROP },
	{ "qm_drop_rx_eop",		1, CA_ST_NI_REG, CA_NI_QM_RX_EOP_DROP_CNTR },
	{ "qm_drop_rx_len_err",		1, CA_ST_NI_REG, CA_NI_QM_RX_LEN_ERR_CNTR },
	{ "qm_drop_rx_l2te",		1, CA_ST_NI_REG, CA_NI_QM_RX_L2TE_DROP_CNTR },

#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	/* ---- L3FE flow offload ------------------------------------------
	 * Absent from the string set when the engine is not built in, which is
	 * a STRUCTURAL absence (there is no engine to count), not a counter
	 * that failed to be measured.
	 * ⚠ CONFIG_CORTINA_NI_FLOWOFFLOAD is OFF in
	 * target/linux/realtek-elnath/config-6.18, so these 19 rows compile out
	 * of the default image and `ethtool -S eth0` reports 117 counters, not 136.
	 */
	{ "l3fe_flows_resident",	1, CA_ST_L3FE, CA_L3FE_FLOWS_RESIDENT },
	{ "l3fe_ds_flows_resident",	1, CA_ST_L3FE, CA_L3FE_DS_FLOWS_RESIDENT },
	{ "l3fe_hw_hits",		1, CA_ST_L3FE, CA_L3FE_HW_HITS },
	{ "l3fe_us_hits",		1, CA_ST_L3FE, CA_L3FE_US_HITS },
	{ "l3fe_ds_hits",		1, CA_ST_L3FE, CA_L3FE_DS_HITS },
	{ "l3fe_hits_unattributed",	1, CA_ST_L3FE, CA_L3FE_HITS_UNATTRIBUTED },
	{ "l3fe_pppoe_us_hits",		1, CA_ST_L3FE, CA_L3FE_PPPOE_US_HITS },
	{ "l3fe_pppoe_ds_hits",		1, CA_ST_L3FE, CA_L3FE_PPPOE_DS_HITS },
	{ "l3fe_flows_refused",		1, CA_ST_L3FE, CA_L3FE_FLOWS_REFUSED },
	{ "l3fe_refused_unsupported",	1, CA_ST_L3FE, CA_L3FE_REFUSED_UNSUPPORTED },
	{ "l3fe_refused_table_full",	1, CA_ST_L3FE, CA_L3FE_REFUSED_TABLE_FULL },
	{ "l3fe_refused_duplicate",	1, CA_ST_L3FE, CA_L3FE_REFUSED_DUPLICATE },
	{ "l3fe_refused_error",		1, CA_ST_L3FE, CA_L3FE_REFUSED_ERROR },
	{ "l3fe_vlan_wan_refused_us",	1, CA_ST_L3FE, CA_L3FE_VLAN_WAN_REFUSED_US },
	{ "l3fe_vlan_wan_refused_ds",	1, CA_ST_L3FE, CA_L3FE_VLAN_WAN_REFUSED_DS },
	{ "l3fe_vlan_pppoe_programmed",	1, CA_ST_L3FE, CA_L3FE_VLAN_PPPOE_PROGRAMMED },
	{ "l3fe_vlan_pppoe_readback_fail", 1, CA_ST_L3FE,
	  CA_L3FE_VLAN_PPPOE_READBACK_FAIL },
	{ "l3fe_vlan_push_legs",	1, CA_ST_L3FE, CA_L3FE_VLAN_PUSH_LEGS },
	{ "l3fe_vlan_strip_legs",	1, CA_ST_L3FE, CA_L3FE_VLAN_STRIP_LEGS },
#endif
};

/*
 * One sample of everything a single `ethtool -S` needs from a shared source,
 * so a read-and-clear register is sampled once per invocation and not once per
 * row that mentions it.
 */
struct ca_ni_stat_ctx {
	u64	nihv[CA_NI_NIHV_CNT_COUNT];
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	u64	l3fe[CA_L3FE_STAT_COUNT];
#endif
};

static u64 ca_ni_stat_value(struct cortina_ni *ni,
			    const struct ca_ni_stat_grp *g, unsigned int i,
			    const struct ca_ni_stat_ctx *ctx)
{
	switch (g->src) {
	case CA_ST_RX_U64:
		if (!ni->rx)
			return 0;	/* rx_datapath_up says why */
		return *(const u64 *)((const u8 *)ni->rx + g->arg +
				      (size_t)i * g->step);
	case CA_ST_TX_U64:
		if (!ni->tx)
			return 0;
		return *(const u64 *)((const u8 *)ni->tx + g->arg +
				      (size_t)i * g->step);
	case CA_ST_NI_REG:
		if (!ni_base(ni))
			return 0;
		return readl(ni_base(ni) + g->arg + (size_t)i * g->step);
	case CA_ST_NIHV:
		return ctx->nihv[g->arg + i];
	case CA_ST_PORT_RXMIB:
		if (!ni_base(ni))
			return 0;
		/* ~0u on a stuck indirect access, so a broken instrument is
		 * visible instead of reading as a silent zero */
		return cortina_ni_rx_mib_read(ni, i, g->arg);
	case CA_ST_PORT_TXMIB:
		if (!ni_base(ni))
			return 0;
		return cortina_ni_tx_mib_read(ni, i, g->arg);
	case CA_ST_DRV_FLAG:
		switch (g->arg) {
		case CA_ST_FLAG_RX_UP:
			return ni->rx ? 1 : 0;
		}
		return 0;
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	case CA_ST_L3FE:
		return ctx->l3fe[g->arg + i];
#endif
	default:
		return 0;
	}
}

/* ------------------------------------------------------------------ */
/* the ethtool_ops bodies                                              */
/* ------------------------------------------------------------------ */

/*
 * cortina_ni_tx_probe() allocates the netdev with
 *   devm_alloc_etherdev(ni->dev, sizeof(struct cortina_ni *))
 * so the private area is exactly ONE pointer, and every callback in
 * cortina-ni-tx.c already reaches the driver state this way.
 *
 * ⚠ NEVER attach cortina_ni_ethtool_ops to the GPON WAN netdev: cortina-gpon.c
 * allocates it with alloc_etherdev(0) - priv size ZERO - and this would read
 * past the end of the allocation.
 */
static struct cortina_ni *cortina_ni_of_netdev(struct net_device *dev)
{
	return *(struct cortina_ni **)netdev_priv(dev);
}

static int cortina_ni_get_sset_count(struct net_device *dev, int sset)
{
	unsigned int i, n = 0;

	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_stat_grps); i++)
		n += cortina_ni_stat_grps[i].n;
	return n;
}

static void cortina_ni_get_strings(struct net_device *dev, u32 sset, u8 *data)
{
	unsigned int g, i;

	if (sset != ETH_SS_STATS)
		return;

	for (g = 0; g < ARRAY_SIZE(cortina_ni_stat_grps); g++) {
		const struct ca_ni_stat_grp *grp = &cortina_ni_stat_grps[g];

		for (i = 0; i < grp->n; i++) {
			/* a %u-less format simply ignores the argument, so the
			 * single-member and the family rows share one path */
			snprintf((char *)data, ETH_GSTRING_LEN, grp->fmt, i);
			data += ETH_GSTRING_LEN;
		}
	}
}

static void cortina_ni_get_ethtool_stats(struct net_device *dev,
					 struct ethtool_stats *stats, u64 *data)
{
	struct cortina_ni *ni = cortina_ni_of_netdev(dev);
	struct ca_ni_stat_ctx ctx;
	unsigned int g, i, n = 0;

	/* ONE sample of every shared source, before the walk */
	cortina_ni_nihv_sample(ni, ctx.nihv);
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	cortina_ni_flowoffload_stats(ctx.l3fe);
#endif

	for (g = 0; g < ARRAY_SIZE(cortina_ni_stat_grps); g++) {
		const struct ca_ni_stat_grp *grp = &cortina_ni_stat_grps[g];

		for (i = 0; i < grp->n; i++)
			data[n++] = ca_ni_stat_value(ni, grp, i, &ctx);
	}
}

/*
 * `ethtool -d`: the curated NI-window register snapshot, as a flat u32 array.
 * The blob is opaque to userspace by design (no vendor decode plugin exists
 * and adding one is a userspace change we do not control), so the name and
 * offset of each word are published separately through debugfs
 * cortina-ni/regdump_map - one index -> one name -> one offset, generated from
 * the SAME table as the dump, so the decode cannot drift from it.
 *
 * ★ VERSION 2, NOT UPSTREAM'S 1.  The word order IS the ABI, and ours is a
 * DIFFERENT snapshot: 66 named words + 662 sweep words = 728 (2912 bytes),
 * against upstream's 66 + 806 = 872.  A decoder pinned to upstream's version 1
 * would mis-decode every word past the first divergence, so the version number
 * says up front that this is another table.  Appending is fine; reordering or
 * removing is not without bumping this again.
 */
#define CA_NI_REGDUMP_VERSION	2

static int cortina_ni_get_regs_len(struct net_device *dev)
{
	return (int)(cortina_ni_regdump_len() * sizeof(u32));
}

static void cortina_ni_get_regs(struct net_device *dev,
				struct ethtool_regs *regs, void *p)
{
	regs->version = CA_NI_REGDUMP_VERSION;
	cortina_ni_regdump_fill(cortina_ni_of_netdev(dev), p);
}

static void cortina_ni_get_drvinfo(struct net_device *dev,
				   struct ethtool_drvinfo *info)
{
	strscpy(info->driver, CA_NI_DRV_NAME, sizeof(info->driver));
	strscpy(info->bus_info, dev_name(cortina_ni_of_netdev(dev)->dev),
		sizeof(info->bus_info));
}

const struct ethtool_ops cortina_ni_ethtool_ops = {
	.get_drvinfo		= cortina_ni_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	/* ndev->phydev is set by phy_connect_direct() in cortina_ni_open and
	 * cleared in cortina_ni_stop, and is never set at all when the driver
	 * is booted with cortina_ni.skip_mdio=1 - phylib then returns
	 * -EOPNOTSUPP here rather than faulting.  A blank `ethtool eth0` on the
	 * bring-up configuration is that, not a broken port. */
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.get_sset_count		= cortina_ni_get_sset_count,
	.get_strings		= cortina_ni_get_strings,
	.get_ethtool_stats	= cortina_ni_get_ethtool_stats,
	.get_regs_len		= cortina_ni_get_regs_len,
	.get_regs		= cortina_ni_get_regs,
};

/* ------------------------------------------------------------------ */
/* debugfs: the decode key for the `ethtool -d` blob                    */
/* ------------------------------------------------------------------ */

/*
 * `ethtool -d` hands userspace a flat u32 array and no tool knows how to name
 * its words, so the map is published beside it: one line per word,
 * index -> name -> NI-window offset, generated by cortina_ni_regdump_entry()
 * from the same tables the dump is taken from.  Read-only, no register access.
 */
static int cortina_ni_dbgfs_regmap_show(struct seq_file *m, void *v)
{
	unsigned int i, n = cortina_ni_regdump_len();

	seq_printf(m, "# ethtool -d words: %u (u32 each, NI window)\n", n);
	seq_printf(m, "# regdump version: %u\n", CA_NI_REGDUMP_VERSION);
	seq_puts(m, "# index name offset\n");
	for (i = 0; i < n; i++) {
		const char *name;
		u32 off;

		cortina_ni_regdump_entry(i, &name, &off);
		seq_printf(m, "%u %s 0x%04x\n", i, name, off);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cortina_ni_dbgfs_regmap);

static void cortina_ni_debugfs_release(void *data)
{
	debugfs_remove_recursive(data);
}

void cortina_ni_debugfs_init(struct cortina_ni *ni)
{
	struct dentry *d;

	/* A stub when debugfs is off: debugfs_create_dir() then returns an
	 * ERR_PTR that every later call swallows, so nothing here has to be
	 * conditional - but an ERR_PTR must not be handed to devm as a pointer
	 * to free. */
	d = debugfs_create_dir(CA_NI_DRV_NAME, NULL);
	if (IS_ERR_OR_NULL(d))
		return;
	ni->dbgfs = d;

	/* teardown is tied to the device: this driver has no .remove, and a
	 * dentry left behind by a module unload would collide with the next
	 * probe's create */
	if (devm_add_action_or_reset(ni->dev, cortina_ni_debugfs_release, d)) {
		ni->dbgfs = NULL;
		return;
	}

	debugfs_create_file("regdump_map", 0444, d, ni,
			    &cortina_ni_dbgfs_regmap_fops);
}
