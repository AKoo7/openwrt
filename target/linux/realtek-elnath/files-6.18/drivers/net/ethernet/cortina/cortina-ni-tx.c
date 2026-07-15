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
 * header.  CPU n rings DMA-LSO virtual port n+2, TX queue 0.  Completion is
 * reported through a HW read pointer which we reclaim opportunistically at
 * xmit time plus from a periodic timer (the engine has no TX-done IRQ wired
 * in this minimal bring-up).
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

/* destination physical port for M2b: port 0 = LAN1, the U-Boot TFTP port */
#define CA_NI_TX_PORT		0
#define CA_NI_TX_COS		0
#define CA_NI_TX_TXQ		0

#define CA_NI_RECLAIM_INTERVAL	msecs_to_jiffies(10)

static bool tx_debug;
module_param(tx_debug, bool, 0644);
MODULE_PARM_DESC(tx_debug, "dump the first transmitted frames/descriptors");

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

	/* coherent (CCI/ACE) read attributes for all VPs */
	writel(CA_DMA_LSO_AXI_USER_SEL0_VAL, dma + CA_DMA_LSO_AXI_USER_SEL0);
	for (i = 0; i < 4; i++)
		writel(CA_DMA_LSO_AXI_USER_PAT_VAL,
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

/* port 0 MAC: TX on (RX stays off until M2c), MAC auto-tracks the PHY */
static void cortina_ni_tx_port_mac_init(struct cortina_ni *ni)
{
	/* connect the port MAC to the internal quad-GPHY over GMII (0xa5c0):
	 * int_cfg=GE_GMII, phy_mode=MAC, MAC-loopback OFF.  NOTE: the upper byte
	 * 0xCB000000 seen on stock is READ-ONLY datapath-active STATUS (a forced
	 * write of it does not stick), not writable config - so it only lights up
	 * once the real GPHY<->MAC datapath gate is satisfied. */
	ni_rmw(ni, CA_NI_PORT_STATIC_CFG(CA_NI_TX_PORT),
	       CA_NI_PORT_STATIC_INT_CFG | CA_NI_PORT_STATIC_PHY_MODE |
	       CA_NI_PORT_STATIC_LPBK_MODE, 0);

	ni_rmw(ni, CA_NI_PORT_GLB_CFG(CA_NI_TX_PORT),
	       CA_NI_PORT_GLB_PWR_DWN_TX, 0);

	ni_rmw(ni, CA_NI_PORT_TXMAC_CFG(CA_NI_TX_PORT),
	       CA_NI_PORT_TXMAC_TX_DRAIN,
	       CA_NI_PORT_TXMAC_TX_EN | CA_NI_PORT_TXMAC_CRC_CALC_EN);

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

		if (!skb) {	/* must not happen: HW advanced past us */
			netdev_err(ndev, "VP%u: hole at %u (rptr %u)\n",
				   q->vp, q->finished, rptr);
			break;
		}
		dma_unmap_single(ni->dev, q->slot[q->finished].addr,
				 q->slot[q->finished].len, DMA_TO_DEVICE);
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += q->slot[q->finished].len;
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
	u32 word1;
	unsigned int len;

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

	/* CPU n owns VP n+2; ndo_start_xmit runs with BH off so the CPU id
	 * is stable and each CPU has a private ring (vendor scheme) */
	q = &tx->txq[raw_smp_processor_id() % CA_NI_TX_NUM_VPS];

	spin_lock(&q->lock);

	/* opportunistic reclaim, then ring-full check (stock keeps 2 spare) */
	if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC) {
		cortina_ni_tx_reclaim_q(ni, q);
		if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC) {
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

	/* direct-TX-to-LAN descriptor: plain frame, no header-A (HP=11) */
	word1 = CA_NI_TX_DESC1_HP1 | CA_NI_TX_DESC1_HP0 |
		CA_NI_TX_DESC1_MODE_DIRECT |
		FIELD_PREP(CA_NI_TX_DESC1_CHK_SEL, CA_NI_TX_CHK_AUTO) |
		FIELD_PREP(CA_NI_TX_DESC1_LEN, len) |
		FIELD_PREP(CA_NI_TX_DESC1_COS, CA_NI_TX_COS) |
		FIELD_PREP(CA_NI_TX_DESC1_DEST, CA_NI_TX_PORT);

	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(daddr));
	desc[1] = cpu_to_le32(word1);

	q->slot[q->wptr].skb = skb;
	q->slot[q->wptr].addr = daddr;
	q->slot[q->wptr].len = len;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;
	q->enq++;
	tx->last_word1 = word1;

	if (unlikely(tx_debug && q->enq <= 4)) {
		netdev_info(ndev,
			    "TX vp%u idx %u len %u desc %08x %08x\n",
			    q->vp, (q->wptr + CA_NI_TX_RING_SIZE - 1) %
			    CA_NI_TX_RING_SIZE, len,
			    lower_32_bits(daddr), word1);
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

static const struct net_device_ops cortina_ni_netdev_ops = {
	.ndo_open		= cortina_ni_open,
	.ndo_stop		= cortina_ni_stop,
	.ndo_start_xmit		= cortina_ni_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

/* ------------------------------------------------------------------ */
/* spy/dump hook: /proc/net/cortina_ni_tx (project rule: probes stay)  */
/* ------------------------------------------------------------------ */

static int cortina_ni_tx_proc_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_tx *tx = ni->tx;
	int i;

	seq_printf(m, "port%d glb=0x%08x txmac=0x%08x autosync=0x%08x\n",
		   CA_NI_TX_PORT,
		   readl(ni_base(ni) + CA_NI_PORT_GLB_CFG(CA_NI_TX_PORT)),
		   readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(CA_NI_TX_PORT)),
		   readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC));
	seq_printf(m, "lso_ctrl=0x%08x ss_ctrl=0x%08x es_ctrl=0x%08x\n",
		   readl(dma_base(ni) + CA_DMA_LSO_CTRL),
		   readl(dma_base(ni) + CA_DMA_SS_CTRL),
		   readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL));
	seq_printf(m, "last_word1=0x%08x busy=%llu nomap=%llu linearize=%llu oversize=%llu\n",
		   tx->last_word1, tx->tx_busy, tx->drop_nomap,
		   tx->drop_linearize, tx->drop_oversize);
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

	/* stock keeps the MAC in the "ethernet@0" child (bootarg-patched) */
	child = of_get_child_by_name(ni->dev->of_node, "ethernet");
	if (child) {
		ret = of_get_ethdev_address(child, ndev);
		of_node_put(child);
	}
	if (ret)
		ret = of_get_ethdev_address(ni->dev->of_node, ndev);
	if (ret || !is_valid_ether_addr(ndev->dev_addr)) {
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

	ret = cortina_ni_tx_hw_init(ni);
	if (ret)
		return ret;

	ndev->netdev_ops = &cortina_ni_netdev_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ETH_DATA_LEN;	/* len field allows 2047 - keep std */
	cortina_ni_tx_set_mac(ni, ndev);

	ret = devm_register_netdev(ni->dev, ndev);
	if (ret)
		return dev_err_probe(ni->dev, ret, "register_netdev failed\n");

	proc_create_single_data("cortina_ni_tx", 0444, init_net.proc_net,
				cortina_ni_tx_proc_show, ni);

	dev_info(ni->dev, "M2b TX ready: %s -> port %d (direct-TX)\n",
		 ndev->name, CA_NI_TX_PORT);
	return 0;
}
