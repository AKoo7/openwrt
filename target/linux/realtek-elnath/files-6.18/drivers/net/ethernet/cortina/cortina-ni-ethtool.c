// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * the STANDARD statistics + register-snapshot interface.
 *
 *   ethtool -S <dev>	every countable quantity the driver and the engine have
 *   ethtool -d <dev>	the curated NI-window register snapshot
 *
 * WHY THIS FILE EXISTS, and it is not a style preference.  Every counter this
 * port publishes used to be reachable only through /proc nodes named after
 * this driver - cortina_ni_rx, cortina_ni_tx, cortina_l3fe.  The vendor
 * firmware has no node of those names (its own are cntr_rx, cntr_tx,
 * dropcount, ni_debug, l3fe_debug, l3qm_debug, pon_debug), so a test case
 * reading ours could only ever BLOCK when run against stock: the ORACLE half
 * of every stock-vs-ours comparison was structurally impossible, and any
 * "compared against stock" figure taken that way was never compared at all.
 * `ethtool` is served by both firmwares' kernels, so the SAME case runs on
 * both and the comparison exists.
 *
 * The /proc nodes are untouched and stay until this interface is proven on the
 * board; retiring them is a separate step.  While both exist they must not
 * fight over the read-and-clear counters - see cortina_ni_nihv_sample() below,
 * which is the single reader they all go through.
 *
 * ★ AND THE ORACLE HALF IS REAL, not merely hoped for - MEASURED 2026-08-08 in
 * the vendor's own shipped module (tier 2: stock's binary, read with `strings`,
 * no SDK involved).  `ca-ne.ko` exports ca_ni_ethtool_ops and
 * ca_ni_cpu_port_ethtool_ops, backed by ca_ethtool_get_sset_count /
 * _get_strings / _get_ethtool_stats / _get_regs / _get_regs_len - so BOTH
 * `ethtool -S` and `ethtool -d` answer on the vendor firmware.  That is what
 * makes a stock-vs-ours case possible at all; it was never possible through a
 * /proc node of ours.
 *
 * ⚠ The two name sets are NOT the same, and a case must not assume they are.
 * Stock publishes a 44-entry per-port MIB in the vendor's CamelCase
 * (RxUCPktCnt, RxMCFrmCnt, RxBCFrmCnt, RxCrcErrFrmCnt, the RxStatsFrm*oct size
 * bins, and the Tx* equivalents); ours is the broader lower_snake_case set
 * below.  A comparison therefore asks for a SEMANTIC FIELD and a short
 * per-firmware shim maps it to that firmware's string - it must never match on
 * a name and call the absence a device fact.
 *
 * NAMING RULES for the strings, because they become the test suite's field
 * names and a renamed field is a broken case: lower_snake_case, stable, and
 * they name WHAT IS COUNTED - never a board resource number.  The /proc
 * `irq_hits` line labelled its buckets with the Linux IRQ number the board
 * happened to allocate (irq_hits_19), which is not a property of the device at
 * all; here they are rx_epp_irq<N>_events, N being the per-CPU-port EPP
 * interrupt index, which is silicon.
 */

#include <linux/bitfield.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
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
 * The NI_HV per-interface packet counters, in enum cortina_ni_nihv_cnt order.
 * Read-and-clear (stock's ca-ne.ko labels the block "NI counter =====
 * (read-and-clear)"; measured directly on 0xa9bc and 0xa9fc, where a second
 * read inside one show() returned a structural zero).
 *
 * ⚠ The two SIBLING error words at 0xa9f4 (RX_MISSING_SOP_EOP) and 0xa9f8
 * (RX_SHORT_ERR) are deliberately NOT here.  They live in the same block and
 * are very probably the same kind of counter, but each packs TWO quantities
 * into one word (hi16/lo16), so there is no single countable value to publish
 * and naming one would be naming it wrongly.  They remain a /proc-only
 * reading; splitting them needs the hi/lo semantics established on stock
 * first.
 */
static const u32 cortina_ni_nihv_off[CA_NI_NIHV_CNT_COUNT] = {
	[CA_NI_NIHV_L3FE_RX]	= CA_NI_NI_L3FE_RX_PKT_CNT,
	[CA_NI_NIHV_L3QM_RX]	= CA_NI_NI_L3QM_RX_PKT_CNT,
	[CA_NI_NIHV_L3QM_TX]	= CA_NI_NI_L3QM_TX_PKT_CNT,
	[CA_NI_NIHV_MCE_RX]	= CA_NI_NI_MCE_RX_PKT_CNT,
	[CA_NI_NIHV_DMA_RX]	= CA_NI_NI_DMA_RX_PKT_CNT,
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
/* the statistics table                                                 */
/* ------------------------------------------------------------------ */

enum ca_ni_stat_src {
	CA_ST_RX_U64,		/* u64 at @arg bytes into struct cortina_ni_rx */
	CA_ST_TX_U64,		/* u64 at @arg bytes into struct cortina_ni_tx */
	CA_ST_NI_REG,		/* plain cumulative register at NI + @arg      */
	CA_ST_NIHV,		/* index @arg into the read-and-clear totals   */
	CA_ST_PORT_MIB,		/* per-port RX MIB, @arg = counter id, i = port*/
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
	{ "tx_vp%u_enqueued",	CA_NI_TX_NUM_VPS, CA_ST_TX_U64,
	  S_TX(txq[0].enq), sizeof(struct cortina_ni_txq) },
	{ "tx_vp%u_reclaimed",	CA_NI_TX_NUM_VPS, CA_ST_TX_U64,
	  S_TX(txq[0].reclaimed), sizeof(struct cortina_ni_txq) },

	/*
	 * ---- per-port MAC RX MIB ----------------------------------------
	 * ⚠ PUBLISHED AS DATA, NOT AS AN INGRESS WITNESS.  These read ZERO on
	 * fully-working STOCK as well as on ours, so a 0 here says nothing
	 * about whether a socket ingressed - the frame is delivered without
	 * ever incrementing them.  The witnesses that DO move on stock under
	 * load are l3fe_rx_packets and l3qm_rx_packets below.  They are kept
	 * because a raw counter costs nothing to carry and the reading may yet
	 * be explained; they are named for the hardware block they come from
	 * so nobody mistakes them for a datapath verdict.
	 *
	 * ⚠ The per-port TX MIB is deliberately ABSENT.  It was measured on
	 * 2026-07-29 across all 8 ACCESS port values and every plausible
	 * counter id while the driver transmitted 1164 frames out the cabled
	 * port: every cell moved by zero, and some read non-zero and stayed
	 * there.  Publishing a value that looks like a counter and never moves
	 * through an interface as authoritative as `ethtool -S` would invite
	 * exactly the wrong conclusion on a working port.  It comes back when
	 * the counter ids have been re-derived FROM STOCK while stock
	 * transmits - the oracle step that was skipped.
	 *
	 * ★ AND THAT ORACLE STEP IS NOW ONE COMMAND.  Stock's ethtool string
	 * set contains TxUCPktCnt / TxMCFrmCnt / TxBCFrmCnt (see the file
	 * header), i.e. the vendor publishes exactly the per-port TX counters
	 * we could not make move.  So: boot stock, `ethtool -S <lan netdev>`
	 * before and after pushing known traffic OUT that socket, and read
	 * whether the vendor's own Tx cells climb.  They climb => our ACCESS
	 * ids were wrong and the counter is real; they stay flat on WORKING
	 * stock => it is a phantom on this silicon and leaving it out is
	 * correct.  Either answer settles it; neither can be had from our side
	 * alone, which is the whole reason this interface exists.
	 */
	{ "port%u_mac_rx_unicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_MIB,
	  CA_NI_MIB_RX_UC_PKT },
	{ "port%u_mac_rx_multicast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_MIB,
	  CA_NI_MIB_RX_MC_PKT },
	{ "port%u_mac_rx_broadcast",	CA_NI_LAN_PORT_COUNT, CA_ST_PORT_MIB,
	  CA_NI_MIB_RX_BC_PKT },

	/* ---- engine, read-and-clear (accumulated; see the header) -------- */
	{ "l3fe_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_L3FE_RX },
	{ "l3qm_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_L3QM_RX },
	{ "l3qm_tx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_L3QM_TX },
	{ "mce_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_MCE_RX },
	{ "dma_rx_packets",	1, CA_ST_NIHV, CA_NI_NIHV_DMA_RX },

	/* ---- engine, cumulative registers -------------------------------
	 * The datapath bisect, in flow order: L2FE ingest -> L2FE drop ->
	 * L2TM buffer manager -> QM/RMU admission -> QM drops.  The first that
	 * stops climbing while the one before it climbs is the death stage.
	 * 32-bit and free-running in hardware, so they wrap; difference two
	 * readings rather than trusting an absolute.
	 */
	{ "l2fe_ni_ingress_drops",	1, CA_ST_NI_REG,
	  CA_NI_L2FE_NI_INTF_DROP_CNT },
	{ "l2fe_dos_flood_drops",	1, CA_ST_NI_REG,
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

/* One sample of everything a single `ethtool -S` needs from a shared source,
 * so a read-and-clear register is sampled once per invocation and not once per
 * row that mentions it. */
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
	case CA_ST_PORT_MIB:
		if (!ni_base(ni))
			return 0;
		/* ~0u on a stuck indirect access, so a broken instrument is
		 * visible instead of reading as a silent zero */
		return cortina_ni_rx_mib_read(ni, i, g->arg);
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
/* ethtool ops                                                          */
/* ------------------------------------------------------------------ */

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
 * offset of each word are published separately through debugfs .../regdump_map
 * - one index -> one name -> one offset, generated from the SAME table, so the
 * decode cannot drift from the dump.
 */
#define CA_NI_REGDUMP_VERSION	1

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
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.get_sset_count		= cortina_ni_get_sset_count,
	.get_strings		= cortina_ni_get_strings,
	.get_ethtool_stats	= cortina_ni_get_ethtool_stats,
	.get_regs_len		= cortina_ni_get_regs_len,
	.get_regs		= cortina_ni_get_regs,
};
