// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * M2b TX datapath: one netdev ("eth0"), direct-TX (FE-bypass) transmit to
 * LAN port 0 through the DMA-LSO engine.  RX comes in M2c.
 *
 * Register offsets, bit semantics, init order and the descriptor encoding
 * are hardware facts recovered from the shipped RTL9607F firmware
 * (ca-ne.ko: aal_ni_init_tx_dma_lso, rtk_ni_init_tx_dma_lso,
 * aal_ni_set_dma_lso_base_depth_addr, __ca_ni_start_xmit_buf_for_fc_dirTx,
 * aal_ni_eth_port_mac_set, aal_ni_mac_autosync_cfg_set, aal_l2_qm_init,
 * aal_l2_tm_init) and cross-checked against the public CA8277B register
 * bit-field definitions.
 *
 * TX model (the "direct TX to LAN" descriptor mode of this chip generation):
 * the 8-byte ring descriptor itself carries the destination port and CoS
 * (mode=1/direct=0), the buffer is a plain Ethernet frame - no prepended
 * header.  Ring assignment is FIXED per netdev for packet order (see the
 * comment in cortina_ni_start_xmit): eth0 -> txq[CA_NI_TX_ETH_RING] (VP3),
 * PON US OMCI + WAN data -> txq[0] (VP2); TX queue 0 within each VP.
 * Completion is reported through a HW read pointer which we reclaim
 * opportunistically at xmit time plus from a periodic timer (the engine has
 * no TX-done IRQ wired in this minimal bring-up).
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <net/arp.h>
#include <net/net_namespace.h>

#include "cortina-ni.h"

/* Fallback / revert destination for the eth0 LAN TX path, and the port whose
 * speed+duplex adjust_link mirrors.  eth0 TX is FE-bypass direct-TX: the
 * descriptor's DEST field is an LDPID that the L2FE ARB resolves to a physical
 * egress port via the PDPID map (see cortina_ni_arb_lan_map_init).  For LAN NI
 * ports ldpid == physical port (identity), so DEST=3 -> physical port 3.
 *
 * Serving all four RJ45s no longer means a fixed DEST: the port is chosen per
 * frame - see ca_ni_lan_tx_ports() and the lan_tx_mode parameter.  This value
 * remains the single-port fallback (lan_tx_mode=0, or no link information). */
#define CA_NI_TX_PORT		3
#define CA_NI_TX_COS		0
#define CA_NI_TX_TXQ		0

#define CA_NI_RECLAIM_INTERVAL	msecs_to_jiffies(10)

static bool tx_debug;
module_param(tx_debug, bool, 0644);
MODULE_PARM_DESC(tx_debug, "dump the first transmitted frames/descriptors");

/* ★ 2026-07-23 host-free HW-forward test knob: when >= 0, stamp EVERY direct-TX
 * descriptor's DEST = this ldpid instead of CA_NI_TX_PORT.  Set =0 briefly to
 * egress an AF_PACKET-injected forged frame out (uncabled) port 0, which - with
 * that port's MAC in internal loopback (PORT_STATIC_CFG bit12|15) - loops back
 * and physically ingresses the L3FE as a real LAN client frame.  Default -1 =
 * normal (DEST=port 3).  Global, so keep the window short + revert (it also
 * mis-routes the held ssh session's TX while set). */
static int force_dest_ldpid = -1;
module_param(force_dest_ldpid, int, 0644);
MODULE_PARM_DESC(force_dest_ldpid, "override direct-TX DEST ldpid for the HW-forward loopback test (-1=off)");

/* ------------------------------------------------------------------ */
/* CPU -> LAN egress port selection                                    */
/* ------------------------------------------------------------------ */

/*
 * A CPU-originated frame must leave the RJ45 the destination host is behind.
 * The descriptor's DEST field is a LAN ldpid and the ARB map is the identity for
 * LAN NI ports, so DEST *is* the physical port - the only question is which
 * value to stamp.  We answer it the way the shipped firmware does: from an
 * explicit netdev/port binding (its ca_ni_init_dev_port_mapping stamps a dest
 * LDPID per netdev, and its RX demux is port2dev[HEADER_A.lspid]).  It registers
 * one netdev per physical LAN port; we register one, so the binding lives here:
 * a DA -> port table learned from HEADER_A.lspid on RX, with a flood to every
 * LINKED port for broadcast/multicast and for a destination not yet seen.
 *
 * Deliberately NOT done by FE-forwarding the frame (dropping MODE_DIRECT +
 * FEBYPASS): that needs an L2FE flood group holding LAN members and a DFT_FWD
 * entry for the CPU source lspid.  Neither exists here (MCE_INDX[0x19] is
 * written EMPTY, MC_FIB is never written, DFT_FWD covers lspid 0..3 only) and
 * neither exists on stock either - and today's DFT_FWD value would redirect a
 * CPU-sourced flood to mcgid 0x19 = L3_LAN, i.e. straight back to the CPU.
 *
 * Flooding to a LINKED port only: a frame stamped for a dead port sits in that
 * port's egress MAC and consumes the shared L2TM buffer pool.
 */
enum {
	CA_NI_LAN_TX_FIXED	= 0,	/* every frame -> CA_NI_TX_PORT (revert) */
	CA_NI_LAN_TX_FLOOD	= 1,	/* every frame -> every linked port     */
	CA_NI_LAN_TX_LEARN	= 2,	/* learned port, flood fallback         */
};
static int lan_tx_mode = CA_NI_LAN_TX_LEARN;
module_param(lan_tx_mode, int, 0644);
MODULE_PARM_DESC(lan_tx_mode,
	"CPU->LAN egress port: 0=fixed CA_NI_TX_PORT (pre-multi-port behaviour), 1=flood every frame to all linked LAN ports (bring-up/bisect only, 4x TX cost), 2=per-DA learned port with flood fallback (default)");

/* one bucket, published atomically: {mac[47:0], port[50:48], valid[51]} */
#define CA_NI_LAN_FDB_SIZE	ARRAY_SIZE(((struct cortina_ni_tx *)0)->lan_fdb)
#define CA_NI_LAN_FDB_MAC	GENMASK_ULL(47, 0)
#define CA_NI_LAN_FDB_PORT	GENMASK_ULL(50, 48)
#define CA_NI_LAN_FDB_VALID	BIT_ULL(51)

/* explicit byte math: this driver must stay endianness-agnostic */
static u64 ca_ni_mac_key(const u8 *mac)
{
	return ((u64)mac[0] << 40) | ((u64)mac[1] << 32) | ((u64)mac[2] << 24) |
	       ((u64)mac[3] << 16) | ((u64)mac[4] << 8) | mac[5];
}

static unsigned int ca_ni_mac_bucket(const u8 *mac)
{
	return (mac[3] ^ mac[4] ^ mac[5]) & (CA_NI_LAN_FDB_SIZE - 1);
}

/*
 * Bind @sa to the RJ45 it arrived on.  @lspid is HEADER_A.lspid.
 *
 * ★ The link check is the SAFETY GUARD on the one fact this rests on.  If lspid
 * turned out NOT to be the ingress port it would read a constant (typically 0),
 * which would bind every host to one port and kill LAN egress.  Refusing to
 * learn a port whose PHY is down means such a value is never learned at all, so
 * we keep flooding - which works.
 */
void cortina_ni_lan_tx_learn(struct cortina_ni *ni, const u8 *sa, u32 lspid)
{
	struct cortina_ni_tx *tx = ni->tx;
	unsigned int b;
	u64 ent;

	if (!tx || lan_tx_mode != CA_NI_LAN_TX_LEARN)
		return;
	if (lspid >= CA_NI_LAN_PORT_COUNT ||
	    !(READ_ONCE(tx->lan_link) & BIT(lspid)))
		return;
	if (is_multicast_ether_addr(sa) || is_zero_ether_addr(sa))
		return;

	ent = FIELD_PREP(CA_NI_LAN_FDB_MAC, ca_ni_mac_key(sa)) |
	      FIELD_PREP(CA_NI_LAN_FDB_PORT, lspid) | CA_NI_LAN_FDB_VALID;
	b = ca_ni_mac_bucket(sa);
	if (READ_ONCE(tx->lan_fdb[b]) == ent)
		return;			/* unchanged - the common case */
	WRITE_ONCE(tx->lan_fdb[b], ent);
	tx->lan_learn++;
}

/*
 * Publish the set of RJ45s with a PHY link.  A change flushes every binding -
 * which is exactly the operator's cable-move test: unplug LAN1, plug LAN2, and
 * no stale DA->port binding may survive.
 */
void cortina_ni_lan_tx_link_set(struct cortina_ni *ni, u32 link)
{
	struct cortina_ni_tx *tx = ni->tx;
	unsigned int i;

	if (!tx || READ_ONCE(tx->lan_link) == link)
		return;
	dev_info(ni->dev, "lan_tx: LAN link set 0x%x -> 0x%x, flushing DA bindings\n",
		 READ_ONCE(tx->lan_link), link);
	WRITE_ONCE(tx->lan_link, link);
	for (i = 0; i < CA_NI_LAN_FDB_SIZE; i++)
		WRITE_ONCE(tx->lan_fdb[i], 0);
	tx->lan_flush++;
}

static int ca_ni_lan_fdb_lookup(struct cortina_ni_tx *tx, const u8 *da)
{
	u64 ent = READ_ONCE(tx->lan_fdb[ca_ni_mac_bucket(da)]);

	if (!(ent & CA_NI_LAN_FDB_VALID) ||
	    FIELD_GET(CA_NI_LAN_FDB_MAC, ent) != ca_ni_mac_key(da))
		return -1;
	return FIELD_GET(CA_NI_LAN_FDB_PORT, ent);
}

/* The egress port set for one frame, as a port bitmap (never empty). */
static u32 ca_ni_lan_tx_ports(struct cortina_ni_tx *tx, const u8 *da)
{
	u32 link = READ_ONCE(tx->lan_link);
	int port;

	/* Diagnostic knob still wins, but RANGE-CHECKED: the port set is a
	 * bitmap now, and BIT() of an out-of-range value is 0 = "no port",
	 * which would map an skb and attach it to no descriptor at all (a DMA +
	 * skb leak, and the frame silently vanishes).  The DEST field is 4 bits
	 * wide, so anything outside 0..15 could never have been stamped anyway. */
	if (force_dest_ldpid >= 0) {
		if (force_dest_ldpid < CA_NI_TX_DEST_LDPID_COUNT)
			return BIT(force_dest_ldpid);
		WARN_ONCE(1, "force_dest_ldpid=%d out of range 0..%d, ignored\n",
			  force_dest_ldpid, CA_NI_TX_DEST_LDPID_COUNT - 1);
	}
	/* Fail-safe: fall back to the single-port behaviour whenever we have no
	 * link information at all (the 1 Hz poll has not run yet, or MDIO
	 * failed).  Never "no port" - that would silence the only IP management
	 * channel to the board. */
	if (lan_tx_mode == CA_NI_LAN_TX_FIXED || !link)
		return BIT(CA_NI_TX_PORT);

	if (lan_tx_mode == CA_NI_LAN_TX_LEARN &&
	    !is_multicast_ether_addr(da)) {
		port = ca_ni_lan_fdb_lookup(tx, da);
		if (port >= 0 && (link & BIT(port))) {
			tx->lan_hit++;
			return BIT(port);
		}
	}
	tx->lan_flood++;
	return link;
}

/* fallback MAC when the DT carries none (locally administered) */
static const u8 cortina_ni_default_mac[ETH_ALEN] = {
	0x02, 0x96, 0x07, 0xf0, 0x00, 0x01
};

static inline void __iomem *ni_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_NI];
}

static inline void __iomem *dma_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_DMA];
}

static inline void ni_rmw(struct cortina_ni *ni, u32 off, u32 clr, u32 set)
{
	writel((readl(ni_base(ni) + off) & ~clr) | set, ni_base(ni) + off);
}

static inline void dma_rmw(struct cortina_ni *ni, u32 off, u32 clr, u32 set)
{
	writel((readl(dma_base(ni) + off) & ~clr) | set, dma_base(ni) + off);
}

/* ------------------------------------------------------------------ */
/* Mandatory HW init (the stock aal_ni_init/l2_qm/l2_tm subset)        */
/* ------------------------------------------------------------------ */

/*
 * NI block reset handshake (stock aal_ni_reset): wait for the NI self-init
 * done flag, then deassert every interface reset.  U-Boot already did this
 * (it TFTPs through the NI) so both are expected to be settled - soft-warn.
 */
static void cortina_ni_tx_reset_intf(struct cortina_ni *ni)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(ni_base(ni) + CA_NI_HV_INIT_DONE, val,
				 val & CA_NI_HV_INIT_DONE_NI,
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(ni->dev, "NI init-done not set (0x%08x), continuing\n",
			 val);

	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);
	val = readl(ni_base(ni) + CA_NI_HV_INTF_RST);	/* stock reads back */
	if (val)
		dev_warn(ni->dev, "INTF_RST readback 0x%08x != 0\n", val);
}

/* 07f VP->LSPID map (stock rtk_ni_init_tx_dma_lso): VP n sources from CPU
 * logical port 0x10+n (n=1..11), others from CPU port 0x10; all valid. */
static int cortina_ni_tx_lspid_map_init(struct cortina_ni *ni)
{
	int i, ret;
	u32 val, lspid;

	for (i = 0; i < CA_DMA_LSO_LSPID_MAP_ENTRIES; i++) {
		lspid = (i >= 1 && i <= 11) ? CA_DMA_LSO_LSPID_CPU0 + i
					    : CA_DMA_LSO_LSPID_CPU0;

		writel(0, dma_base(ni) + CA_DMA_LSO_LSPID_MAP_DATA1);
		writel(CA_DMA_LSO_LSPID_MAP_VALID |
		       FIELD_PREP(CA_DMA_LSO_LSPID_MAP_LSPID, lspid),
		       dma_base(ni) + CA_DMA_LSO_LSPID_MAP_DATA0);
		writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE |
		       FIELD_PREP(CA_DMA_LSO_LSPID_MAP_IDX, i),
		       dma_base(ni) + CA_DMA_LSO_LSPID_MAP_ACCESS);

		ret = readl_poll_timeout(dma_base(ni) +
					 CA_DMA_LSO_LSPID_MAP_ACCESS, val,
					 !(val & CA_DMA_LSO_BD_ACCESS_GO),
					 CA_NI_TX_POLL_US,
					 CA_NI_TX_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(ni->dev, "lspid map[%d] write timed out\n", i);
			return ret;
		}
	}
	return 0;
}

/*
 * Global TX-engine enable - the "silent stall" block: without these the
 * descriptors are consumed but no frame ever moves (stock
 * aal_ni_init_tx_dma_lso + the 07f-only rtk_ni_init_tx_dma_lso extras).
 */
static int cortina_ni_tx_engine_init(struct cortina_ni *ni)
{
	void __iomem *dma = dma_base(ni);
	void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];
	int i, ret;

	/* non-ACE mode: clear the SRAM-test byte (stock companion of the
	 * DATA1 addr[39:32]=0 ring programming) */
	dma_rmw(ni, CA_DMA_LSO_SRAM_TEST_CTRL1, 0xff, 0);

	/* enable all 8 TX queues of every DMA-LSO VP (stock does all 12) */
	for (i = 0; i < CA_DMA_LSO_VP_COUNT; i++)
		writel(CA_DMA_LSO_VP_TXQ_ALL_EN,
		       dma + CA_DMA_LSO_VP_CONTROL(i));

	/* AXI master: outstanding transactions + cacheline transfers */
	writel(readl(dma + CA_DMA_AXIM2_CONFIG) | CA_DMA_AXIM2_CONFIG_BITS,
	       dma + CA_DMA_AXIM2_CONFIG);

	/* ★ 2026-07-15: NON-coherent read attributes for all VPs.  Stock writes the
	 * coherent/ACE pattern (CA_DMA_LSO_AXI_USER_PAT_VAL) and its fabric snoops;
	 * on our kernel the ACE path is dead (QM 0x611c bit30 proved it for the EPP
	 * writeback) and the ACE descriptor fetch never completed - the TXQ rptr sat
	 * at 0 while the doorbell wptr climbed, so nothing ever transmitted.  The TX
	 * ring/buffers are cache-maintained by the DMA API instead (no dma-coherent
	 * on the NE DT node). */
	writel(CA_DMA_LSO_AXI_USER_SEL0_VAL, dma + CA_DMA_LSO_AXI_USER_SEL0);
	for (i = 0; i < 4; i++)
		writel(CA_DMA_LSO_AXI_USER_PAT_NOCOH,
		       dma + CA_DMA_LSO_AXI_USER_PAT0 + i * 4);

	/* scheduler/shaper global TX enable */
	writel(readl(dma + CA_DMA_SS_CTRL) | CA_DMA_SS_CTRL_TX_EN,
	       dma + CA_DMA_SS_CTRL);

	/* TX DMA enable, burst 64x64bit, HW pad of short frames */
	writel(CA_DMA_LSO_CTRL_VAL, dma + CA_DMA_LSO_CTRL);
	readl(dma + CA_DMA_LSO_CTRL);		/* stock reads back */

	/* 07f-only: AXI reorder slots for the DMA-LSO read path */
	if (reo) {
		for (i = 0; i < CA_AXI_REO_SLOT_COUNT; i++)
			writel(CA_AXI_REO_SLOT_VAL, reo + CA_AXI_REO_SLOT(i));
	} else {
		dev_warn(ni->dev,
			 "axi-reo window unmapped, skipping reorder cfg\n");
	}

	/* 07f-only trio written unconditionally by stock TX init */
	writel(CA_DMA_LSO_MISC_C0_VAL, dma + CA_DMA_LSO_MISC_C0);
	writel(CA_DMA_LSO_VLAN_TAG_TYPE0_VAL, dma + CA_DMA_LSO_VLAN_TAG_TYPE0);
	writel(CA_DMA_LSO_MISC_C4_VAL, dma + CA_DMA_LSO_MISC_C4);

	ret = cortina_ni_tx_lspid_map_init(ni);
	if (ret)
		return ret;

	/* stock's final LSO_CTRL state: keep the source LSPID from the map
	 * table, HW-pad via AFT below instead of lso_padding (0x2d -> 0x1d) */
	dma_rmw(ni, CA_DMA_LSO_CTRL, CA_DMA_LSO_CTRL_PAD_EN,
		CA_DMA_LSO_CTRL_LSPID_KEEP);

	/* HW short-frame pad to 64 bytes */
	dma_rmw(ni, CA_DMA_AFT_CTRL, CA_DMA_AFT_PAD_SIZE,
		CA_DMA_AFT_PAD_EN |
		FIELD_PREP(CA_DMA_AFT_PAD_SIZE, CA_DMA_AFT_PAD_SIZE_VAL));

	/* FE-bypass enable per VP - reset default is 0 = frames routed into
	 * the (uninitialized) forwarding engine and dropped */
	for (i = 0; i < CA_DMA_LSO_VP_COUNT; i++)
		dma_rmw(ni, CA_DMA_LSO_VP_HDRA_CFG(i),
			CA_DMA_LSO_HDRA_LDPID, CA_DMA_LSO_HDRA_FEBYPASS);

	return 0;
}

/* program one VP/TXQ descriptor-ring base+depth via the indirect window
 * (stock aal_ni_set_dma_lso_base_depth_addr) */
static int cortina_ni_tx_ring_program(struct cortina_ni *ni, u8 vp, u8 txq,
				      dma_addr_t base)
{
	void __iomem *dma = dma_base(ni);
	u32 val;
	int ret;

	if (WARN_ON(upper_32_bits(base) || (base & 0xf)))
		return -EINVAL;

	writel((lower_32_bits(base) & ~0xf) |
	       FIELD_PREP(CA_DMA_LSO_BD_DATA0_DEPTH, CA_NI_TX_RING_DEPTH),
	       dma + CA_DMA_LSO_VP_BD_DATA0(vp));
	/* addr[39:32] = 0: ring sits below 4 GB, and stock writes 0 here
	 * (its "2" branch is the disabled dma_lso_ace_test path) */
	writel(0, dma + CA_DMA_LSO_VP_BD_DATA1(vp));
	writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE |
	       FIELD_PREP(CA_DMA_LSO_BD_ACCESS_TXQ, txq),
	       dma + CA_DMA_LSO_VP_BD_ACCESS(vp));

	ret = readl_poll_timeout(dma + CA_DMA_LSO_VP_BD_ACCESS(vp), val,
				 !(val & CA_DMA_LSO_BD_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_err(ni->dev, "VP%u txq%u ring program timed out\n",
			vp, txq);
	return ret;
}

static int cortina_ni_tx_rings_init(struct cortina_ni *ni)
{
	struct cortina_ni_tx *tx = ni->tx;
	int i, ret;

	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];
		u32 wptr, rptr;

		q->vp = CA_NI_TX_VP_BASE + i;
		spin_lock_init(&q->lock);

		q->desc = dmam_alloc_coherent(ni->dev,
					      CA_NI_TX_RING_SIZE *
					      CA_NI_TX_DESC_WORDS * 4,
					      &q->desc_dma, GFP_KERNEL);
		if (!q->desc)
			return -ENOMEM;

		ret = cortina_ni_tx_ring_program(ni, q->vp, CA_NI_TX_TXQ,
						 q->desc_dma);
		if (ret)
			return ret;

		/* adopt whatever pointer state the HW is in (0 after reset) */
		wptr = readl(dma_base(ni) +
			     CA_DMA_LSO_VP_TXQ_WPTR(q->vp, CA_NI_TX_TXQ)) &
			CA_DMA_LSO_PTR_MASK;
		rptr = readl(dma_base(ni) +
			     CA_DMA_LSO_VP_TXQ_RPTR(q->vp, CA_NI_TX_TXQ)) &
			CA_DMA_LSO_PTR_MASK;
		if (wptr >= CA_NI_TX_RING_SIZE || rptr >= CA_NI_TX_RING_SIZE ||
		    wptr != rptr)
			dev_warn(ni->dev,
				 "VP%u txq0 pointers not idle (w=%u r=%u)\n",
				 q->vp, wptr, rptr);
		q->wptr = wptr % CA_NI_TX_RING_SIZE;
		q->finished = rptr % CA_NI_TX_RING_SIZE;

		dev_info(ni->dev, "VP%u txq0 ring @%pad (%u desc)\n",
			 q->vp, &q->desc_dma, CA_NI_TX_RING_SIZE);
	}
	return 0;
}

/* QM buffer manager (stock aal_l2_qm_init values) - without buffers the
 * egress enqueue fails silently */
static void cortina_ni_tx_qm_init(struct cortina_ni *ni)
{
	/* EQ0 disabled/empty; EQ1 enabled, 4K x 64B pool + port private */
	ni_rmw(ni, CA_NI_L2TM_QM_EQ_CFG,
	       CA_NI_L2TM_EQ0_EN | CA_NI_L2TM_EQ0_BUFNUM |
	       CA_NI_L2TM_EQ0_PRVT | CA_NI_L2TM_EQ1_BUFNUM |
	       CA_NI_L2TM_EQ1_PRVT,
	       CA_NI_L2TM_EQ1_EN |
	       FIELD_PREP(CA_NI_L2TM_EQ1_BUFNUM, CA_NI_QM_EQ1_BUFNUM_VAL) |
	       FIELD_PREP(CA_NI_L2TM_EQ1_PRVT, CA_NI_QM_PORT_PRVT_BUFF_NUM));

	/* port-private buffer profile 0 (all ports select it by default) */
	ni_rmw(ni, CA_NI_L2TM_QM_PORT_PRVT_PROF0, 0x7fff,
	       CA_NI_QM_PORT_PRVT_BUFF_NUM);

	/* global buffer thresholds: drop on, no FE back-pressure */
	ni_rmw(ni, CA_NI_L2TM_QM_GLOB_BUF_CFG,
	       CA_NI_L2TM_BUF_NODROP | CA_NI_L2TM_BUF_NONCONG |
	       CA_NI_L2TM_BUF_FE_BP_EN,
	       CA_NI_L2TM_BUF_DROP_EN |
	       FIELD_PREP(CA_NI_L2TM_BUF_NODROP, CA_NI_QM_NODROP_THRESHOLD) |
	       FIELD_PREP(CA_NI_L2TM_BUF_NONCONG,
			  CA_NI_QM_NONCONG_THRESHOLD));
}

/* TM egress scheduler (stock aal_l2_tm_init): global + per-port + per-VOQ
 * enables - the "one-line block enable" whose omission silently stalls TX */
static void cortina_ni_tx_tm_init(struct cortina_ni *ni)
{
	int i;

	ni_rmw(ni, CA_NI_L2TM_ES_CTRL, 0,
	       CA_NI_L2TM_ES_TX_EN | CA_NI_L2TM_ES_PORT_EN_ALL);

	for (i = 0; i < CA_NI_L2TM_ES_SCH_INSTANCES; i++)
		ni_rmw(ni, CA_NI_L2TM_ES_SCH_CFG(i), 0,
		       CA_NI_L2TM_ES_VOQ_EN_ALL);
}

/* LAN port MACs: TX on for EVERY RJ45 (RX is armed by the link path), MAC
 * auto-tracks the PHY.
 *
 * ★ Looped over all CA_NI_LAN_PORT_COUNT ports, not just CA_NI_TX_PORT: with a
 * per-frame egress port the descriptor DEST can now name any LAN port, and the
 * TX side of ports 0..2 was not enabled by anything.
 * cortina_ni_rx_enable_internal_ports() loops p = 1..6, so port 0 had NO TXMAC
 * tx_en at all, and GLB.PWR_DWN_TX was cleared only for CA_NI_TX_PORT - so a
 * frame stamped for port 0/1/2 would have been handed to a powered-down egress
 * MAC.
 */
static void cortina_ni_tx_port_mac_init(struct cortina_ni *ni)
{
	unsigned int p;

	/* connect the port MAC to the internal quad-GPHY over GMII (0xa5c0):
	 * int_cfg=GE_GMII, phy_mode=MAC, MAC-loopback OFF.  NOTE: the upper byte
	 * 0xCB000000 seen on stock is READ-ONLY datapath-active STATUS (a forced
	 * write of it does not stick), not writable config - so it only lights up
	 * once the real GPHY<->MAC datapath gate is satisfied. */
	for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
		ni_rmw(ni, CA_NI_PORT_STATIC_CFG(p),
		       CA_NI_PORT_STATIC_INT_CFG | CA_NI_PORT_STATIC_PHY_MODE |
		       CA_NI_PORT_STATIC_LPBK_MODE, 0);

		ni_rmw(ni, CA_NI_PORT_GLB_CFG(p),
		       CA_NI_PORT_GLB_PWR_DWN_TX, 0);

		ni_rmw(ni, CA_NI_PORT_TXMAC_CFG(p),
		       CA_NI_PORT_TXMAC_TX_DRAIN,
		       CA_NI_PORT_TXMAC_TX_EN | CA_NI_PORT_TXMAC_CRC_CALC_EN);
	}

	/* MAC autosync OFF (=0), matching U-Boot's PROVEN-working datapath
	 * (autosync=0x0 while tftp ran bidirectionally over this port).
	 *
	 * ★ Determinism root cause: we drive phylib (adjust_link writes the GLB
	 * speed/duplex in SW on every link event) AND phylib RESTARTS aneg at
	 * phy_start, bouncing the line link.  If HW autosync (0xf) is ALSO on,
	 * the HW continuously re-derives glb/speed/duplex from the churning PHY
	 * status during that bounce and fights our SW writes - dropping the
	 * internal GMII on the boots where the two collide (== "works some
	 * boots").  Stock tolerates autosync=0xf only because its link is stable
	 * (it never restarts aneg - it inherits U-Boot's link and just monitors
	 * it).  We use phylib, so we adopt U-Boot's consistent model: autosync
	 * OFF, phylib owns speed/duplex via adjust_link.  ONE owner, no fight. */
	/* DIAGNOSTIC: stock uses autosync=0xf (HW MAC-follows-PHY, STS_ALL). Now
	 * that the GPHY firmware matches stock, test stock's autosync model. */
	ni_rmw(ni, CA_NI_HV_MAC_AUTOSYNC,
	       CA_NI_HV_AUTOSYNC_FC_ALL, CA_NI_HV_AUTOSYNC_STS_ALL);
}

/*
 * L2FE ARB ldpid->pdpid map: route the "9th queue" ldpids (0x08..0x0f, the
 * CPU-injected US PON control-frame ports) to the PON-OAM egress.  Without it
 * a CPU-injected OMCI frame (HEADER_A ldpid = PON+8 = 0x0f) has no physical
 * route, never reaches the PUC, and the OLT receives no upstream OMCI.  Only
 * touches ldpid 0x08..0x0f (unused by the Ethernet CPU-RX/LAN paths, which use
 * ldpid 0x19/0x32), so it cannot disturb the working datapath.  32 entries:
 * ldpid 0x08..0x0f x dbuf {0,1} x my_mac {0,1} -> PPORT_OAM.
 */
static int cortina_ni_arb_map_one(struct cortina_ni *ni, u32 idx, u32 pdpid)
{
	void __iomem *ni_r = ni_base(ni);
	u32 val;
	int ret;

	writel(pdpid, ni_r + CA_NI_L2FE_ARB_PDPID_DATA);
	writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE | idx,
	       ni_r + CA_NI_L2FE_ARB_PDPID_ACCESS);
	ret = readl_poll_timeout(ni_r + CA_NI_L2FE_ARB_PDPID_ACCESS, val,
				 !(val & CA_DMA_LSO_BD_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(ni->dev, "ARB map[0x%02x] timed out\n", idx);
	return ret;
}

/* ★ Physical LAN NI ports 0-6: identity ldpid->pdpid so an eth0 direct-TX frame
 * (whose descriptor DEST field is the ldpid) egresses physical port N (vendor
 * aal_port.c global port init).  Left unmapped, the ARB PDPID map reads its reset
 * value 0, so EVERY eth0 CPU-TX frame resolved to physical port 0 (uncabled/dead)
 * regardless of the descriptor DEST - which is why LAN INGRESS worked but the
 * router's ARP/ping/DHCP replies never reached the wired host, and why setting
 * the descriptor DEST or the VP HDRA LDPID alone changed nothing.  dbuf=1 rows ->
 * QM (US-PON data path); ldpid 7 (PON) -> blackhole, per the vendor map. */
static void cortina_ni_arb_lan_map_init(struct cortina_ni *ni)
{
	u32 my_mac, ldpid;

	for (my_mac = 0; my_mac <= 1; my_mac++) {
		for (ldpid = 0; ldpid <= 6; ldpid++) {
			if (cortina_ni_arb_map_one(ni, (my_mac << 7) | ldpid,
						   ldpid))
				return;
			if (cortina_ni_arb_map_one(ni,
						   (my_mac << 7) | BIT(6) | ldpid,
						   CA_NI_PPORT_QM))
				return;
		}
		if (cortina_ni_arb_map_one(ni, (my_mac << 7) | 7,
					   CA_NI_PPORT_BLACKHOLE))
			return;
		if (cortina_ni_arb_map_one(ni, (my_mac << 7) | BIT(6) | 7,
					   CA_NI_PPORT_BLACKHOLE))
			return;
	}
	dev_info(ni->dev,
		 "L2FE ARB: LAN ldpids 0x00-0x06 -> identity pdpid (eth0 egress)\n");
}

static void cortina_ni_arb_oam_map_init(struct cortina_ni *ni)
{
	u32 my_mac, dbuf, ldpid, idx;

	for (my_mac = 0; my_mac <= 1; my_mac++) {
		for (dbuf = 0; dbuf <= 1; dbuf++) {
			/* 9th-queue (control-frame inject) -> OAM engine */
			for (ldpid = CA_NI_LDPID_9QUEUE_LO;
			     ldpid <= CA_NI_LDPID_9QUEUE_HI; ldpid++) {
				idx = (my_mac << 7) | (dbuf << 6) | ldpid;
				if (cortina_ni_arb_map_one(ni, idx,
							   CA_NI_PPORT_OAM))
					return;
			}
			/* CPU_MQ / LLID-GEM-index (US PON DATA inject) -> QM
			 * (vendor aal_port.c global init maps all of
			 * 0x20..0x3f, both dbuf + my_mac, to PPORT_QM) */
			for (ldpid = CA_NI_LDPID_CPU_MQ_LO;
			     ldpid <= CA_NI_LDPID_CPU_MQ_HI; ldpid++) {
				idx = (my_mac << 7) | (dbuf << 6) | ldpid;
				if (cortina_ni_arb_map_one(ni, idx,
							   CA_NI_PPORT_QM))
					return;
			}
		}
	}
	dev_info(ni->dev,
		 "L2FE ARB: ldpids 0x08-0x0f -> PON-OAM, 0x20-0x3f -> QM\n");
}

static int cortina_ni_tx_hw_init(struct cortina_ni *ni)
{
	int ret;

	/* ★ DIAGNOSTIC: deassert the internal digital-PHY resets EARLY (before any
	 * GPHY/MAC init) - stock's dphy_rst (GLB+0xa0) = 0x10000000, ours boots
	 * 0x50302340 (sub-blocks held in reset).  Release-late (link_up) didn't
	 * revive it, so try release-then-init order: release here, before init. */
	if (ni->win[CA_NI_WIN_GLB]) {
		dev_info(ni->dev, "early dphy_rst(glb+0xa0) 0x%08x -> 0x10000000\n",
			 readl(ni->win[CA_NI_WIN_GLB] + 0xa0));
		writel(0x10000000, ni->win[CA_NI_WIN_GLB] + 0xa0);
	}

	/* stock aal_ni_init order: reset -> NI globals -> TX-DMA engine */
	cortina_ni_tx_reset_intf(ni);

	/* unconditional stock globals (unknown names, exact stock values) */
	ni_rmw(ni, CA_NI_HV_CFG_A420, CA_NI_HV_CFG_A420_FIELD,
	       FIELD_PREP(CA_NI_HV_CFG_A420_FIELD, CA_NI_HV_CFG_A420_VAL));
	ni_rmw(ni, CA_NI_HV_CFG_A1B8, CA_NI_HV_CFG_A1B8_FIELD,
	       FIELD_PREP(CA_NI_HV_CFG_A1B8_FIELD, CA_NI_HV_CFG_A1B8_VAL));
	ni_rmw(ni, CA_NI_HV_CFG_AAF0, CA_NI_HV_CFG_AAF0_FIELD,
	       FIELD_PREP(CA_NI_HV_CFG_AAF0_FIELD, CA_NI_HV_CFG_AAF0_VAL));

	/* frame-length limits, stock values */
	ni_rmw(ni, CA_NI_HV_PKT_LEN,
	       CA_NI_HV_PKT_LEN_MIN | CA_NI_HV_PKT_LEN_MAX,
	       FIELD_PREP(CA_NI_HV_PKT_LEN_MIN, CA_NI_HV_PKT_LEN_MIN_VAL) |
	       FIELD_PREP(CA_NI_HV_PKT_LEN_MAX, CA_NI_HV_PKT_LEN_MAX_VAL));
	ni_rmw(ni, CA_NI_HV_PKT_LEN_RX, CA_NI_HV_PKT_LEN_RX_MAX,
	       FIELD_PREP(CA_NI_HV_PKT_LEN_RX_MAX, CA_NI_HV_PKT_LEN_MAX_VAL));

	/* 0xa1bc = INTERNAL_PORT_ID_CFG (the old chipdef mislabeled it
	 * "NIRX_MISC"): keep the aal_ni_init golden mirror bits [13:9] and clear
	 * the stray bit15 (U-Boot left 0xbe80; stock golden 0x3e80).  CRITICAL:
	 * do NOT clear bit20 - that is l3qmrx_to_lan, the NI->QM LAN handoff SET
	 * by the L3QM delivery init; clearing it here (as the old code did) left
	 * NI-RX invisible to the QM. */
	ni_rmw(ni, CA_NI_NI_INTERNAL_PORT_ID_CFG,
	       CA_NI_NI_INTERNAL_BIT15,
	       CA_NI_NI_MRR_CFG);

	/* deferred stock init (not needed for port-0 direct TX): the 0xa01c
	 * port-to-cpu debug bits, the RX demux cfg (0xa180/88/8c), SCH-cfg
	 * field [23:16]=6 on instances 8/10/13, aal_l2_te/l3_tm/l3_te init,
	 * and the streamid/dmaaft/l2fib table clears (reset defaults 0) */

	ret = cortina_ni_tx_engine_init(ni);
	if (ret)
		return ret;

	ret = cortina_ni_tx_rings_init(ni);
	if (ret)
		return ret;

	cortina_ni_tx_qm_init(ni);
	cortina_ni_tx_tm_init(ni);
	cortina_ni_tx_port_mac_init(ni);
	cortina_ni_arb_oam_map_init(ni);	/* US PON control-frame egress route */
	cortina_ni_arb_lan_map_init(ni);	/* LAN NI ldpid->pport egress route */
	return 0;
}

/* ------------------------------------------------------------------ */
/* TX completion                                                       */
/* ------------------------------------------------------------------ */

/* caller holds q->lock */
static unsigned int cortina_ni_tx_reclaim_q(struct cortina_ni *ni,
					    struct cortina_ni_txq *q)
{
	struct net_device *ndev = ni->tx->netdev;
	unsigned int freed = 0;
	u32 rptr;

	rptr = readl(dma_base(ni) +
		     CA_DMA_LSO_VP_TXQ_RPTR(q->vp, CA_NI_TX_TXQ)) &
		CA_DMA_LSO_PTR_MASK;
	rptr %= CA_NI_TX_RING_SIZE;

	while (q->finished != rptr) {
		struct sk_buff *skb = q->slot[q->finished].skb;

		if (!skb) {
			u8 pon = q->slot[q->finished].pon;

			if (q->slot[q->finished].dup) {
				/* extra copy of a flooded eth0 frame: it shares
				 * the mapping owned by the LAST descriptor of
				 * the burst, so there is nothing to unmap or
				 * free here.  The engine consumes the ring in
				 * order, so the owner is always reclaimed after
				 * every copy of its own burst. */
				q->slot[q->finished].dup = 0;
				q->finished = (q->finished + 1) %
					      CA_NI_TX_RING_SIZE;
				q->reclaimed++;
				freed++;
				continue;
			}
			if (!pon) {	/* must not happen: HW advanced past us */
				netdev_err(ndev, "VP%u: hole at %u (rptr %u)\n",
					   q->vp, q->finished, rptr);
				break;
			}
			/* PON control-frame descriptor: coherent scratch,
			 * nothing to unmap/free; the frame (EOF) descriptor
			 * releases its scratch slot (under this q->lock) */
			if (pon >= 2)
				ni->tx->pon_busy &= ~BIT(pon - 2);
			q->slot[q->finished].pon = 0;
			q->finished = (q->finished + 1) % CA_NI_TX_RING_SIZE;
			q->reclaimed++;
			freed++;
			continue;
		}
		dma_unmap_single(ni->dev, q->slot[q->finished].addr,
				 q->slot[q->finished].len, DMA_TO_DEVICE);
		if (!q->slot[q->finished].pon) {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += q->slot[q->finished].len;
		} else {
			/* PON data skb (pon=1 + skb): counted on the WAN
			 * netdev at enqueue, not on eth0 */
			q->slot[q->finished].pon = 0;
		}
		dev_consume_skb_any(skb);
		q->slot[q->finished].skb = NULL;
		q->finished = (q->finished + 1) % CA_NI_TX_RING_SIZE;
		q->reclaimed++;
		freed++;
	}
	return freed;
}

static unsigned int cortina_ni_txq_free_desc(struct cortina_ni_txq *q)
{
	if (q->wptr >= q->finished)
		return CA_NI_TX_RING_SIZE - q->wptr - 1 + q->finished;
	return q->finished - q->wptr - 1;
}

static void cortina_ni_tx_reclaim_timer(struct timer_list *t)
{
	struct cortina_ni_tx *tx = timer_container_of(tx, t, reclaim_timer);
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(tx->netdev);
	bool pending = false;
	unsigned int freed = 0;
	int i;

	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];

		spin_lock_bh(&q->lock);
		freed += cortina_ni_tx_reclaim_q(ni, q);
		if (q->finished != q->wptr)
			pending = true;
		spin_unlock_bh(&q->lock);
	}

	if (freed && netif_queue_stopped(tx->netdev))
		netif_wake_queue(tx->netdev);
	if (pending)
		mod_timer(&tx->reclaim_timer,
			  jiffies + CA_NI_RECLAIM_INTERVAL);
}

/* ------------------------------------------------------------------ */
/* xmit                                                                */
/* ------------------------------------------------------------------ */

static netdev_tx_t cortina_ni_start_xmit(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct cortina_ni_tx *tx = ni->tx;
	struct cortina_ni_txq *q;
	dma_addr_t daddr;
	__le32 *desc;
	u32 word1, w = 0, ports;
	unsigned int len, first, nports, port;

	/* short frames: pad to the wire minimum (also covers the engine's
	 * 34-byte DMA floor); skb freed by the helper on failure */
	if (skb_padto(skb, ETH_ZLEN))
		return NETDEV_TX_OK;
	len = max_t(unsigned int, skb->len, ETH_ZLEN);

	if (unlikely(len > CA_NI_TX_MAX_FRAME)) {
		tx->drop_oversize++;
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* single-descriptor path (stock dirTx is single-descriptor too) */
	if (unlikely(skb_linearize(skb))) {
		tx->drop_linearize++;
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/*
	 * ★ PACKET ORDER (the downstream OOO/TCP-collapse root cause): eth0 is
	 * a SINGLE-queue netdev, so it must feed exactly ONE HW ring.  The old
	 * per-CPU pick here — q = &txq[raw_smp_processor_id() % NUM_VPS], the
	 * vendor scheme (stock __ca_ni_start_xmit @0x1ae230 selects its DMA-LSO
	 * VP via ca_ni_dmalso_vp_sel[cpu % 7] = {8,1,2,3,9,10,11}) — split one
	 * flow across up to 4 VP rings whenever the transmitting CPU changed
	 * (IRQ/NAPI migration across the 8 RX SPIs, qdisc-runner handoff, RPS).
	 * The DMA-LSO engine fetches the VP rings independently, so same-flow
	 * frames overtake each other on the port-0 wire: OOO scaling with rate
	 * (7@100M → 25%@600M), which TCP reads as loss → spurious-retransmit
	 * storms.  Stock gets away with the per-CPU scheme because its bulk
	 * traffic is HW-forwarded (CPU TX is slow-path only) and its netdevs
	 * are 8-queue mq (flow→queue pinned by the stack); we CPU-forward
	 * everything, so wire order must equal qdisc order: one netdev queue →
	 * one ring.  The qdisc already serializes xmit for a single-queue
	 * netdev (one CPU in qdisc_run at a time, in-order dequeue), so a
	 * fixed ring restores strict per-flow order with no new locking;
	 * q->lock still guards against the reclaim timer.  eth0 rides its OWN
	 * ring (VP3), leaving txq[0] (VP2) to the PON OMCI/WAN-data path, so
	 * the two in-order streams neither share a lock nor stall each other.
	 */
	q = &tx->txq[CA_NI_TX_ETH_RING];

	/* Egress port set for this frame: one learned/fixed port, or a flood to
	 * every linked RJ45.  Computed before the ring check because a flood
	 * needs one descriptor per port.  This netdev is single-queue, so the
	 * qdisc serialises us and the plain counter updates need no atomics -
	 * the same assumption the fixed-ring choice above rests on. */
	ports = ca_ni_lan_tx_ports(tx, skb->data);
	/* An empty set must be impossible: it would map the skb below and then
	 * attach it to no descriptor, leaking both and dropping the frame with
	 * no counter moving.  ca_ni_lan_tx_ports() guarantees non-empty; this
	 * keeps a future edit from reintroducing that silently. */
	if (WARN_ONCE(!ports, "lan_tx: empty egress port set\n"))
		ports = BIT(CA_NI_TX_PORT);
	nports = hweight32(ports);

	spin_lock(&q->lock);

	/* opportunistic reclaim, then ring-full check (stock keeps 2 spare) */
	if (cortina_ni_txq_free_desc(q) < CA_NI_TX_RESERVE_DESC + nports) {
		cortina_ni_tx_reclaim_q(ni, q);
		if (cortina_ni_txq_free_desc(q) <
		    CA_NI_TX_RESERVE_DESC + nports) {
			tx->tx_busy++;
			netif_stop_queue(ndev);
			mod_timer(&tx->reclaim_timer,
				  jiffies + CA_NI_RECLAIM_INTERVAL);
			spin_unlock(&q->lock);
			return NETDEV_TX_BUSY;
		}
	}

	daddr = dma_map_single(ni->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ni->dev, daddr))) {
		tx->drop_nomap++;
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		spin_unlock(&q->lock);
		return NETDEV_TX_OK;
	}
	/* the engine takes a 32-bit buffer address (+ the CCI selector);
	 * both DDR pools sit below 4 GB and the DMA mask enforces it */
	WARN_ON_ONCE(upper_32_bits(daddr));

	/*
	 * One direct-TX-to-LAN descriptor per egress port (plain frame, no
	 * header-A, HP=11), ALL pointing at the SAME mapped buffer: the engine
	 * only reads it, and it consumes the ring in order, so the LAST
	 * descriptor of the burst owns the skb + the mapping and every earlier
	 * one is marked `dup`.  A flood therefore costs extra descriptors only -
	 * no copy, no allocation, and TX stats still count the frame once.
	 */
	word1 = CA_NI_TX_DESC1_HP1 | CA_NI_TX_DESC1_HP0 |
		CA_NI_TX_DESC1_MODE_DIRECT |
		FIELD_PREP(CA_NI_TX_DESC1_CHK_SEL, CA_NI_TX_CHK_AUTO) |
		FIELD_PREP(CA_NI_TX_DESC1_LEN, len) |
		FIELD_PREP(CA_NI_TX_DESC1_COS, CA_NI_TX_COS);

	first = q->wptr;
	for (port = 0; ports; port++) {
		bool last;

		if (!(ports & BIT(port)))
			continue;
		ports &= ~BIT(port);
		last = !ports;
		w = word1 | FIELD_PREP(CA_NI_TX_DESC1_DEST, port);

		desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
		desc[0] = cpu_to_le32(lower_32_bits(daddr));
		desc[1] = cpu_to_le32(w);

		q->slot[q->wptr].skb = last ? skb : NULL;
		q->slot[q->wptr].addr = last ? daddr : 0;
		q->slot[q->wptr].len = last ? len : 0;
		q->slot[q->wptr].dup = last ? 0 : 1;
		q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;
		if (!last)
			tx->lan_dup++;
	}
	q->enq++;
	tx->last_word1 = w;

	if (unlikely(tx_debug && q->enq <= 4)) {
		netdev_info(ndev,
			    "TX vp%u idx %u len %u ndesc %u last-desc %08x %08x\n",
			    q->vp, first, len, nports,
			    lower_32_bits(daddr), w);
		print_hex_dump(KERN_INFO, "TX frame: ", DUMP_PREFIX_OFFSET,
			       16, 1, skb->data, min(len, 64u), false);
	}

	/* TX timestamp BEFORE the doorbell: once the doorbell rings, HW may complete
	 * the frame and the reclaim path (timer/other CPU) can free this skb, so
	 * touching it after the unlock below is a use-after-free. */
	skb_tx_timestamp(skb);

	/* descriptor visible before the doorbell (stock: dmb oshst) */
	dma_wmb();
	writel(q->wptr,
	       dma_base(ni) + CA_DMA_LSO_VP_TXQ_WPTR(q->vp, CA_NI_TX_TXQ));

	spin_unlock(&q->lock);

	mod_timer(&tx->reclaim_timer, jiffies + CA_NI_RECLAIM_INTERVAL);
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------ */
/* US PON control-frame (OMCI) TX — see cortina-ni.h / cortina-ni-regs.h */
/* ------------------------------------------------------------------ */

/* set once at TX probe; the GPON driver's responder calls in through it */
static struct cortina_ni *cortina_ni_pon_tx_ni;

/* 16-byte PON control-frame header (stock ca_ni_tx_encap_pon_control_packet,
 * disasm @0xa92ac: fixed DA/SA, link type 0xff 0xf1 = OMCI, byte [14] = 0,
 * byte [15] = (cos > 6) — this board's ko is the G3 build and OMCI goes out
 * at cos 7/8, so [15] = 0x01). */
static const u8 cortina_ni_pon_hdr[CA_NI_PON_HDR_LEN] = {
	0x00, 0x13, 0x25, 0x00, 0x00, 0x00,	/* DA */
	0x00, 0x13, 0x25, 0x00, 0x00, 0x01,	/* SA */
	0xff, 0xf1, 0x00, 0x01,			/* OMCI link type + G3 cos>=7 flag */
};

int cortina_ni_pon_tx(const u8 *pdu, unsigned int len)
{
	struct cortina_ni *ni = READ_ONCE(cortina_ni_pon_tx_ni);
	struct cortina_ni_tx *tx;
	struct cortina_ni_txq *q;
	unsigned int frame_len, slot;
	dma_addr_t blk_dma;
	__le32 *desc, *w;
	u32 lo, hi;
	u8 *blk;

	if (!ni || !ni->tx || !ni->tx->pon_buf)
		return -ENODEV;
	tx = ni->tx;
	if (!len || len > CA_NI_PON_TX_PDU_MAX)
		return -EINVAL;
	frame_len = CA_NI_PON_HDR_LEN + len;	/* 64 for a 48B OMCI PDU */

	/* always txq[0]: pon_busy and the scratch are guarded by its lock;
	 * _bh so the responder may call from NAPI softirq or process ctx */
	q = &tx->txq[0];
	spin_lock_bh(&q->lock);

	/*
	 * Reclaim UNCONDITIONALLY here: the scratch (CA_NI_PON_TX_SLOTS) is much
	 * smaller than the descriptor ring, so under a fast OMCI burst (the OLT's
	 * MIB-Upload-Next walk sends ~1 message every ~25ms) the scratch runs out
	 * long before the ring does.  If we only reclaimed on ring-low, pon_busy
	 * would never get cleared during the burst and we'd -EBUSY-drop replies
	 * even though the HW already drained them.  A dropped reply is fatal to
	 * the stateful MIB-Upload-Next walk (the responder advances its pointer,
	 * so the OLT's retransmit gets the wrong entry -> the upload desyncs and
	 * the OLT aborts).  So free every completed slot on every send.
	 */
	cortina_ni_tx_reclaim_q(ni, q);
	slot = ffz(tx->pon_busy);		/* >= SLOTS when all busy */
	if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC + 2 ||
	    slot >= CA_NI_PON_TX_SLOTS) {
		tx->pon_fail++;
		spin_unlock_bh(&q->lock);
		return -EBUSY;		/* caller drops; the OLT retransmits */
	}
	tx->pon_busy |= BIT(slot);

	blk = (u8 *)tx->pon_buf + slot * CA_NI_PON_TX_SLOT_SZ;
	blk_dma = tx->pon_buf_dma + slot * CA_NI_PON_TX_SLOT_SZ;

	/* frame = 16-byte PON header + the OMCI PDU (coherent, no mapping) */
	memcpy(blk + CA_NI_PON_TX_FRAME_OFF, cortina_ni_pon_hdr,
	       CA_NI_PON_HDR_LEN);
	memcpy(blk + CA_NI_PON_TX_FRAME_OFF + CA_NI_PON_HDR_LEN, pdu, len);

	/* Header block {LSO para0 = 0, LSO para1 = pkt_size, HEADER_A}: plain
	 * little-endian stores, exactly the stock stores (disasm
	 * __ca_ni_send_single_pkt: `stp wzr, w<size>, [x4]` then the 64-bit
	 * HEADER_A at +8).  ★ WORD ORDER (stock-proven): the word at +8 is
	 * the PKT_INFO half (no_drop/pol_id/cpu_flg, spec bits 32..63) and
	 * the word at +12 is the cos/ldpid/lspid/pkt_size half (bits 0..31)
	 * — the pol_id bfi lands in the LOW-address word.  pol_en stays 0
	 * for OMCI (the G3 branch passes pol=INVALID; only the 0xff/0xf1
	 * override sets pol_id = (DA[5]&0x3f)*8+7 = 7, without pol_en). */
	lo = FIELD_PREP(CA_NI_PON_HDRA_LO_COS, CA_NI_PON_COS) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LDPID, CA_NI_PON_LDPID) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LSPID, CA_NI_PON_LSPID) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_PKT_SIZE, frame_len) |
	     CA_NI_PON_HDRA_LO_FE_BYPASS;
	hi = CA_NI_PON_HDRA_HI_NO_DROP |
	     FIELD_PREP(CA_NI_PON_HDRA_HI_POL_ID, CA_NI_PON_POL_ID);
	w = (__le32 *)blk;
	w[0] = 0;
	w[1] = cpu_to_le32(frame_len);
	w[2] = cpu_to_le32(hi);		/* +8:  pkt_info word (stock order) */
	w[3] = cpu_to_le32(lo);		/* +12: cos/ldpid/pkt_size word */

	/* descriptor pair: SOF + HP=01 header block, then the EOF frame */
	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(blk_dma));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_SOF | CA_NI_TX_DESC1_HP0 |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN,
					 CA_NI_PON_HDR_BLK_LEN));
	q->slot[q->wptr].skb = NULL;
	q->slot[q->wptr].addr = 0;
	q->slot[q->wptr].len = 0;
	q->slot[q->wptr].pon = 1;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(blk_dma + CA_NI_PON_TX_FRAME_OFF));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_EOF |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN, frame_len));
	q->slot[q->wptr].skb = NULL;
	q->slot[q->wptr].addr = 0;
	q->slot[q->wptr].len = 0;
	q->slot[q->wptr].pon = 2 + slot;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	q->enq++;
	tx->pon_enq++;

	/* header block + frame + descriptors visible before the doorbell */
	dma_wmb();
	writel(q->wptr,
	       dma_base(ni) + CA_DMA_LSO_VP_TXQ_WPTR(q->vp, CA_NI_TX_TXQ));

	spin_unlock_bh(&q->lock);

	mod_timer(&tx->reclaim_timer, jiffies + CA_NI_RECLAIM_INTERVAL);
	return 0;
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_tx);

/*
 * US PON DATA (WAN) TX — Stage D.  Same 2-descriptor HEADER_A chain as the
 * OMCI path, but the frame is the dma-mapped skb (up to MTU-size, too big
 * for the 128-byte scratch) and the HEADER_A is a plain data header: ldpid =
 * PON (0x07, no 9th-queue offset), cos = the data queue, fe_bypass, no_drop,
 * no policer.  In the PUC 8Q VoQ map the frame lands in T-CONT 7 queue 0
 * (VoQ 56), whose US_PORT_ID the GPON driver bound to the OLT-assigned data
 * GEM and whose T-CONT CAM entry it bound to the OLT's data alloc-id.
 * Descriptor bookkeeping: SOF slot releases the header-block scratch (pon =
 * 2+slot), EOF slot carries the skb (normal unmap+consume reclaim).
 */
netdev_tx_t cortina_ni_pon_data_tx(struct sk_buff *skb,
				   struct net_device *ndev)
{
	struct cortina_ni *ni = READ_ONCE(cortina_ni_pon_tx_ni);
	struct cortina_ni_tx *tx;
	struct cortina_ni_txq *q;
	unsigned int len, slot;
	dma_addr_t blk_dma, daddr;
	__le32 *desc, *w;
	u32 lo, hi;
	u8 *blk;

	if (!ni || !ni->tx || !ni->tx->pon_buf) {
		ndev->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	tx = ni->tx;

	if (skb_padto(skb, ETH_ZLEN))	/* freed by the helper on failure */
		return NETDEV_TX_OK;
	len = max_t(unsigned int, skb->len, ETH_ZLEN);
	if (unlikely(len > CA_NI_TX_MAX_FRAME || skb_linearize(skb))) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* txq[0], shared with the OMCI path: pon_busy + scratch live under
	 * its lock (ndo_start_xmit runs with BH off -> plain spin_lock) */
	q = &tx->txq[0];
	spin_lock(&q->lock);

	cortina_ni_tx_reclaim_q(ni, q);	/* scratch << ring: reclaim every send */
	slot = ffz(tx->pon_busy);
	if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC + 2 ||
	    slot >= CA_NI_PON_TX_SLOTS) {
		tx->pon_fail++;
		ndev->stats.tx_dropped++;
		spin_unlock(&q->lock);
		dev_kfree_skb_any(skb);	/* WAN clients retransmit (DHCP/TCP) */
		return NETDEV_TX_OK;
	}

	daddr = dma_map_single(ni->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ni->dev, daddr))) {
		tx->drop_nomap++;
		ndev->stats.tx_dropped++;
		spin_unlock(&q->lock);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	WARN_ON_ONCE(upper_32_bits(daddr));
	tx->pon_busy |= BIT(slot);

	blk = (u8 *)tx->pon_buf + slot * CA_NI_PON_TX_SLOT_SZ;
	blk_dma = tx->pon_buf_dma + slot * CA_NI_PON_TX_SLOT_SZ;

	/* header block {LSO para0 = 0, LSO para1 = pkt_size, HEADER_A}: the
	 * same stock word order as the OMCI path (+8 = pkt_info half, +12 =
	 * cos/ldpid/pkt_size half) */
	lo = FIELD_PREP(CA_NI_PON_HDRA_LO_COS, CA_NI_PON_DATA_COS) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LDPID, CA_NI_PON_DATA_LDPID) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LSPID, CA_NI_PON_LSPID) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_PKT_SIZE, len) |
	     CA_NI_PON_HDRA_LO_FE_BYPASS;
	hi = CA_NI_PON_HDRA_HI_NO_DROP |
	     FIELD_PREP(CA_NI_PON_HDRA_HI_POL_ID,
			CA_NI_PON_DATA_TCONT * 8 + CA_NI_PON_DATA_COS);
	w = (__le32 *)blk;
	w[0] = 0;
	w[1] = cpu_to_le32(len);
	w[2] = cpu_to_le32(hi);
	w[3] = cpu_to_le32(lo);

	/* SOF + HP=01 header block (releases scratch slot on reclaim) */
	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(blk_dma));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_SOF | CA_NI_TX_DESC1_HP0 |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN,
					 CA_NI_PON_HDR_BLK_LEN));
	q->slot[q->wptr].skb = NULL;
	q->slot[q->wptr].addr = 0;
	q->slot[q->wptr].len = 0;
	q->slot[q->wptr].pon = 2 + slot;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	/* EOF = the frame itself (skb reclaimed by the normal unmap path;
	 * pon=1 with skb set = "WAN data skb", stats counted here not eth0) */
	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(daddr));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_EOF |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN, len));
	q->slot[q->wptr].skb = skb;
	q->slot[q->wptr].addr = daddr;
	q->slot[q->wptr].len = len;
	q->slot[q->wptr].pon = 1;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	q->enq++;
	tx->pon_data_enq++;
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;

	skb_tx_timestamp(skb);	/* before the doorbell (UAF lesson) */

	dma_wmb();
	writel(q->wptr,
	       dma_base(ni) + CA_DMA_LSO_VP_TXQ_WPTR(q->vp, CA_NI_TX_TXQ));

	spin_unlock(&q->lock);

	mod_timer(&tx->reclaim_timer, jiffies + CA_NI_RECLAIM_INTERVAL);
	return NETDEV_TX_OK;
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_data_tx);

/* ------------------------------------------------------------------ */
/* link handling + the M2b on-air proof frame                          */
/* ------------------------------------------------------------------ */

/* one gratuitous ARP so the host tcpdump sees a frame right at link-up,
 * sent through the ordinary xmit path (not a register poke) */
static void cortina_ni_tx_announce(struct work_struct *work)
{
	struct cortina_ni_tx *tx =
		container_of(work, struct cortina_ni_tx, announce_work);
	struct net_device *ndev = tx->netdev;
	struct sk_buff *skb;

	skb = arp_create(ARPOP_REQUEST, ETH_P_ARP, 0, ndev, 0,
			 NULL, ndev->dev_addr, NULL);
	if (!skb)
		return;
	netdev_info(ndev, "sending link-up gratuitous ARP\n");
	dev_queue_xmit(skb);
}

static void cortina_ni_tx_adjust_link(struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;
	u32 clr = 0, set = 0;

	if (phydev->link) {
		/* MAC autosync tracks the PHY in HW; mirror speed/duplex in
		 * the port config bits like stock does (bit0: 1 = 10M) */
		if (phydev->speed == SPEED_10)
			set |= CA_NI_PORT_GLB_SPEED_10M;
		else
			clr |= CA_NI_PORT_GLB_SPEED_10M;
		if (phydev->duplex == DUPLEX_HALF)
			set |= CA_NI_PORT_GLB_HALF_DUPLEX;
		else
			clr |= CA_NI_PORT_GLB_HALF_DUPLEX;
		clr |= CA_NI_PORT_GLB_PWR_DWN_TX;
		ni_rmw(ni, CA_NI_PORT_GLB_CFG(CA_NI_TX_PORT), clr, set);

		/* every link-up (incl. each boot-time bounce): idempotently
		 * re-arm the RX chain + run one GPHY fault-latch check (the
		 * nondeterministic zero-RX wedge latches across a bounce) */
		cortina_ni_rx_link_up(ni);

		netif_wake_queue(ndev);
		if (!ni->tx->announced) {
			ni->tx->announced = true;
			schedule_work(&ni->tx->announce_work);
		}
	}
	phy_print_status(phydev);
	/* eth0 CPU-port carrier is forced up in cortina_ni_rx_link_up (reached via
	 * this path on a real link-up AND via the decoupled bring-up), so nothing to
	 * do here for the carrier - the phy_link_change override just prevents phylib
	 * from clearing it on the tracked PHY's link-down. */
}

/* eth0 is the CPU<->switch port, NOT a single physical link.  phylib's default
 * phy_link_change() netif_carrier_off()s eth0 whenever the one tracked PHY
 * (phy_find_first = port 0, uncabled on this rig) reports link-down, and does
 * NOT re-run adjust_link while the link stays down -- so eth0's carrier is stuck
 * off, the Linux bridge disables the eth0 port, and br-lan drops every LAN frame
 * the switch already delivered.  Override phy_link_change to run adjust_link but
 * never carrier-off the CPU port: adjust_link forces the carrier back up once the
 * datapath is armed, and per-physical-port link/forwarding is the switch's job. */
static void cortina_ni_cpu_link_change(struct phy_device *phydev, bool up)
{
	phydev->adjust_link(phydev->attached_dev);
}

/* ------------------------------------------------------------------ */
/* net_device_ops                                                      */
/* ------------------------------------------------------------------ */

static int cortina_ni_open(struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct phy_device *phydev;
	int ret;

	/* PHY @ addr 1 drives port 0; U-Boot left it linked - no reset */
	phydev = phy_find_first(ni->mii);
	if (!phydev) {
		netdev_err(ndev, "no PHY found on the internal bus\n");
		return -ENODEV;
	}

	ret = phy_connect_direct(ndev, phydev, cortina_ni_tx_adjust_link,
				 PHY_INTERFACE_MODE_INTERNAL);
	if (ret) {
		netdev_err(ndev, "cannot attach PHY %d\n", phydev->mdio.addr);
		return ret;
	}
	phy_set_max_speed(phydev, SPEED_1000);
	/* keep the CPU-port carrier from following the tracked PHY's link-down */
	phydev->phy_link_change = cortina_ni_cpu_link_change;
	netdev_info(ndev, "CPU-port carrier override installed (eth0 stays up once datapath armed)\n");
	ni->tx->phydev = phydev;
	ni->tx->announced = false;

	/* Disable EEE before aneg (stock keeps EEE off on the internal GPHY; this
	 * MAC has no LPI handling). */
	phy_disable_eee(phydev);

	phy_start(phydev);
	cortina_ni_rx_open(ni);	/* M2c: NAPI + RX IRQ + port RXMAC on */
	netif_start_queue(ndev);
	return 0;
}

static int cortina_ni_stop(struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct cortina_ni_tx *tx = ni->tx;
	int i;

	netif_stop_queue(ndev);
	cortina_ni_rx_stop(ni);	/* M2c: RXMAC off, IRQ masked, NAPI off */
	if (tx->phydev) {
		phy_stop(tx->phydev);
		phy_disconnect(tx->phydev);
		tx->phydev = NULL;
	}
	cancel_work_sync(&tx->announce_work);
	timer_delete_sync(&tx->reclaim_timer);

	/* reclaim whatever completed; anything still in flight stays mapped
	 * until the engine drains it (rings are not torn down between
	 * open/stop - the HW keeps its pointers) */
	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];

		spin_lock_bh(&q->lock);
		cortina_ni_tx_reclaim_q(ni, q);
		if (q->finished != q->wptr)
			netdev_warn(ndev, "VP%u: %u frames still in flight\n",
				    q->vp,
				    (q->wptr + CA_NI_TX_RING_SIZE -
				     q->finished) % CA_NI_TX_RING_SIZE);
		spin_unlock_bh(&q->lock);
	}
	return 0;
}

/* ★ Commit a new MAC, then re-key the MAC-keyed HW tables (L2FE FDB, my-MAC
 * comparator, PP FIELD-CAM, offload router-MAC shadow) from it.  netifd
 * applies the per-board factory MAC (05_factory_mac) AFTER the boot RX init
 * and the last link-up re-arm latched dev_addr into those tables, and no
 * further link-up fires on this rig (tracked port-0 PHY uncabled) - so with
 * plain eth_mac_addr the tables stayed keyed on the boot fallback and a LAN
 * transit frame to the factory gateway MAC could never resolve to L3_LAN /
 * enter the L3FE flow engine.  The re-arm is hw_l3_fwd-gated inside
 * cortina_ni_rx_mac_rearm; gate-off = eth_mac_addr behaviour exactly. */
static int cortina_ni_set_mac_address(struct net_device *ndev, void *addr)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, addr);

	if (ret)
		return ret;
	cortina_ni_rx_mac_rearm(ni);
	return 0;
}

static const struct net_device_ops cortina_ni_netdev_ops = {
	.ndo_open		= cortina_ni_open,
	.ndo_stop		= cortina_ni_stop,
	.ndo_start_xmit		= cortina_ni_start_xmit,
	.ndo_set_mac_address	= cortina_ni_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	/* L3FE flow-engine nf_flow_table offload (cortina-ni-flowoffload.c) */
	.ndo_setup_tc		= cortina_ni_setup_tc,
#endif
};

/* ------------------------------------------------------------------ */
/* spy/dump hook: /proc/net/cortina_ni_tx (project rule: probes stay)  */
/* ------------------------------------------------------------------ */

static int cortina_ni_tx_proc_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_tx *tx = ni->tx;
	int i;

	seq_printf(m, "autosync=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC));
	seq_printf(m, "lan_tx: mode=%d (0=fixed port%d 1=flood 2=learn) link=0x%x hit=%llu flood=%llu dup=%llu learn=%llu flush=%llu\n",
		   lan_tx_mode, CA_NI_TX_PORT, READ_ONCE(tx->lan_link),
		   tx->lan_hit, tx->lan_flood, tx->lan_dup, tx->lan_learn,
		   tx->lan_flush);
	/*
	 * ★★ NO per-port TX PACKET counter is printed here, and that is a
	 * MEASURED negative, not an omission.
	 *
	 * NI_HV_GLB_TXMIB (ACCESS 0xa174 / DATA0 0xa17c) looked like the only
	 * per-PHYSICAL-port egress counter, with ids UC/MC/BC = 1/2/3 DERIVED
	 * from the vendor table's TxStatsFrm65to127Oct = 0xf size-bin anchor.
	 * Measured on 2026-07-29 (dev/x400axf/txmib_identify.py): across all 8
	 * ACCESS port values and ids {0,1,2,3,0xf}, every cell moved by ZERO
	 * while the driver transmitted 1164 CPU->LAN frames out the cabled port.
	 * Some cells even read non-zero and STAYED there - a value that looks
	 * like a counter and never moves is exactly the phantom witness this
	 * project keeps losing days to, so it is NOT published.  Publishing it
	 * would let the next session read tx_uc=0 on a WORKING port and chase a
	 * datapath bug that is not there.
	 *
	 * What IS trustworthy, and is printed above: the driver's own software
	 * lan_tx counters (hit/flood/dup/learn/flush) and the per-port MAC
	 * config below.  For "did THIS socket egress", use a far-end capture.
	 * To resurrect a hardware witness, re-derive the ids from stock (read
	 * the same cells on the vendor image WHILE it transmits) - that is the
	 * oracle step that was skipped.
	 */
	for (i = 0; i < CA_NI_LAN_PORT_COUNT; i++)
		seq_printf(m, "port%d glb=0x%08x txmac=0x%08x\n",
			   i,
			   readl(ni_base(ni) + CA_NI_PORT_GLB_CFG(i)),
			   readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(i)));
	seq_printf(m, "lso_ctrl=0x%08x ss_ctrl=0x%08x es_ctrl=0x%08x\n",
		   readl(dma_base(ni) + CA_DMA_LSO_CTRL),
		   readl(dma_base(ni) + CA_DMA_SS_CTRL),
		   readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL));
	seq_printf(m, "last_word1=0x%08x busy=%llu nomap=%llu linearize=%llu oversize=%llu\n",
		   tx->last_word1, tx->tx_busy, tx->drop_nomap,
		   tx->drop_linearize, tx->drop_oversize);
	seq_printf(m, "pon_tx enq=%llu data_enq=%llu fail=%llu busy_slots=0x%02x\n",
		   tx->pon_enq, tx->pon_data_enq, tx->pon_fail, tx->pon_busy);
	/* data_enq above is UPSTREAM-ONLY; print the downstream complement right
	 * next to it so it can never be read as a whole-device CPU-forward rate. */
	cortina_ni_cpu_fwd_show(m, ni);
	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];

		seq_printf(m, "vp%u hw w=%u r=%u sw w=%u f=%u enq=%llu done=%llu\n",
			   q->vp,
			   (u32)(readl(dma_base(ni) +
				       CA_DMA_LSO_VP_TXQ_WPTR(q->vp, 0)) &
				 CA_DMA_LSO_PTR_MASK),
			   (u32)(readl(dma_base(ni) +
				       CA_DMA_LSO_VP_TXQ_RPTR(q->vp, 0)) &
				 CA_DMA_LSO_PTR_MASK),
			   q->wptr, q->finished, q->enq, q->reclaimed);
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* probe                                                               */
/* ------------------------------------------------------------------ */

static void cortina_ni_tx_set_mac(struct cortina_ni *ni,
				  struct net_device *ndev)
{
	struct device_node *child;
	int ret = -ENODEV;

	/* the "ethernet@0" child would carry a DT MAC; on this board neither
	 * U-Boot nor the stock DTB fills it (live stock reads all-zero) */
	child = of_get_child_by_name(ni->dev->of_node, "ethernet");
	if (child) {
		ret = of_get_ethdev_address(child, ndev);
		of_node_put(child);
	}
	if (ret)
		ret = of_get_ethdev_address(ni->dev->of_node, ndev);
	if (ret || !is_valid_ether_addr(ndev->dev_addr)) {
		/* LAA fallback only: the per-board factory MAC (base MAC =
		 * ELAN_MAC_ADDR from the stock ubi_Config/config_hs.xml on
		 * read-only NAND) is applied by the 05_factory_mac
		 * uci-defaults script through netifd before the interface
		 * comes up; .ndo_set_mac_address then re-keys the MAC-keyed
		 * HW tables (cortina_ni_rx_mac_rearm) - the link-up re-arms
		 * alone do NOT follow it, they all fire before netifd (the
		 * tracked port-0 PHY is uncabled on this rig). */
		eth_hw_addr_set(ndev, cortina_ni_default_mac);
		dev_warn(ni->dev, "no MAC in DT, using default %pM\n",
			 ndev->dev_addr);
	} else {
		dev_info(ni->dev, "MAC from DT: %pM\n", ndev->dev_addr);
	}
}

int cortina_ni_tx_probe(struct cortina_ni *ni)
{
	struct net_device *ndev;
	struct cortina_ni_tx *tx;
	struct cortina_ni **priv;
	int ret;

	if (!ni->win[CA_NI_WIN_DMA])
		return dev_err_probe(ni->dev, -ENODEV,
				     "DMA-LSO window not mapped, no TX\n");

	/* the engine hands 32-bit buffer addresses to the DMA */
	ret = dma_set_mask_and_coherent(ni->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(ni->dev, ret, "no 32-bit DMA\n");

	ndev = devm_alloc_etherdev(ni->dev, sizeof(struct cortina_ni *));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, ni->dev);
	priv = netdev_priv(ndev);
	*priv = ni;

	tx = devm_kzalloc(ni->dev, sizeof(*tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;
	tx->netdev = ndev;
	ni->tx = tx;

	timer_setup(&tx->reclaim_timer, cortina_ni_tx_reclaim_timer, 0);
	INIT_WORK(&tx->announce_work, cortina_ni_tx_announce);

	/* US PON control-frame (OMCI) TX scratch — non-fatal when absent,
	 * cortina_ni_pon_tx just reports -ENODEV */
	tx->pon_buf = dmam_alloc_coherent(ni->dev,
					  CA_NI_PON_TX_SLOTS *
					  CA_NI_PON_TX_SLOT_SZ,
					  &tx->pon_buf_dma, GFP_KERNEL);
	if (!tx->pon_buf)
		dev_warn(ni->dev, "no PON TX scratch - US OMCI TX disabled\n");

	ret = cortina_ni_tx_hw_init(ni);
	if (ret)
		return ret;

	ndev->netdev_ops = &cortina_ni_netdev_ops;
	/* the STANDARD counter + register-snapshot interface (cortina-ni-ethtool.c):
	 * `ethtool -S` / `ethtool -d`.  It exists so a test can ask the same
	 * question of the vendor firmware and of ours - the /proc nodes cannot,
	 * because stock has no node of those names. */
	ndev->ethtool_ops = &cortina_ni_ethtool_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_DATA_LEN;	/* len field allows 2047 - keep std */
	cortina_ni_tx_set_mac(ni, ndev);

	ret = devm_register_netdev(ni->dev, ndev);
	if (ret)
		return dev_err_probe(ni->dev, ret, "register_netdev failed\n");

	proc_create_single_data("cortina_ni_tx", 0444, init_net.proc_net,
				cortina_ni_tx_proc_show, ni);

	WRITE_ONCE(cortina_ni_pon_tx_ni, ni);	/* open the PON TX entry */
	dev_info(ni->dev, "TX ready: %s -> LAN ports 0..%u (direct-TX, lan_tx_mode=%d)\n",
		 ndev->name, CA_NI_LAN_PORT_COUNT - 1, lan_tx_mode);
	return 0;
}
