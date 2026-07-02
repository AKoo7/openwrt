// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL9607C (Luna, MIPS interAptiv) GMAC0 + on-chip switch — eth0.
 *
 * Clean-room driver for the SoC's CPU-port Gigabit MAC (the "GMAC0" engine at
 * phys 0x18012000) and a minimal open-L2 bring-up of the on-chip switch core
 * (phys 0x1b000000). The CPU reaches the LAN through the switch:
 *
 *	CPU <-> GMAC0 (eth0) <-> switch CPU-port(9) <-> physical LAN port <-> wire
 *
 * The MAC engine is the same family block as the r1/RLX sibling, so the
 * register/descriptor layout is shared; only a few control values and the
 * switch register offsets differ on this chip. This driver carries ONLY the
 * Ethernet datapath — the GPON/OMCI datapath lands in a later milestone.
 *
 * Switch bring-up facts (behavioural):
 *  - CPU port = switch port 9 (this GMAC). LAN copper ports = 0..4 and 8;
 *    SerDes ports = 6,7 (the lab board's real wire egress is the SerDes-6
 *    uplink to an external 2.5G PHY); the fibre/PON port = 5.
 *  - The boot loader leaves the per-ingress-port "source permit" mask cleared,
 *    which blocks ALL ingress forwarding, so received frames never reach the
 *    CPU even though injected TX egresses. Opening that mask is the load-bearing
 *    RX fix.
 *  - The integrated copper PHYs need an analog calibration that is not
 *    reproduced here; the deterministic path is MAC-force-link every port plus
 *    open-L2 flood, NOT PHY autonegotiation.
 *  - The ordered sequence matters: replaying only the final register values is
 *    not sufficient on this hardware.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/timer.h>

/* ---- bring-up knobs (live-tunable; the datapath framing is HW-uncertain on
 * first contact, so expose the few values most likely to need a tweak) ------ */
static int rx_prefix = 2;
module_param(rx_prefix, int, 0644);
MODULE_PARM_DESC(rx_prefix, "bytes the CPU-port prepends ahead of each RX frame (stripped)");

static unsigned int backstop_ms = 10;
module_param(backstop_ms, uint, 0644);
MODULE_PARM_DESC(backstop_ms, "RX/TX drain backstop poll period (catches a missed IRQ)");

static bool sw_cpu_tag;
module_param(sw_cpu_tag, bool, 0644);
MODULE_PARM_DESC(sw_cpu_tag, "enable the switch CPU-port tag engine (default off: plain-L2 forwarding)");

static int rx_dump = 6;
module_param(rx_dump, int, 0644);
MODULE_PARM_DESC(rx_dump, "hex-dump the first N received frames (bring-up framing check)");

static int tx_dump = 6;
module_param(tx_dump, int, 0644);
MODULE_PARM_DESC(tx_dump, "hex-dump the first N transmitted frames + descriptors");

static bool copper_phy = true;
module_param(copper_phy, bool, 0644);
MODULE_PARM_DESC(copper_phy, "power up + auto-neg the internal copper PHYs (ports 0-4)");

static bool rtl8221b_phy = true;
module_param(rtl8221b_phy, bool, 0644);
MODULE_PARM_DESC(rtl8221b_phy, "de-assert the RTL8221B 2.5G PHY reset (SerDes-6 uplink)");

static unsigned int diag_ms = 3000;
module_param(diag_ms, uint, 0644);
MODULE_PARM_DESC(diag_ms, "period of the per-port real-link/rxpkts diagnostic (0 = off)");

static int diag_count = 12;
module_param(diag_count, int, 0644);
MODULE_PARM_DESC(diag_count, "number of periodic link/rxpkts diagnostic dumps");

static bool cpu_no_loopback = true;
module_param(cpu_no_loopback, bool, 0644);
MODULE_PARM_DESC(cpu_no_loopback, "drop the CPU port from its own egress flood (stops self-loopback RX)");

/* ---- GMAC0 register block (offsets from the DT reg base 0x18012000) -------- */
#define R_IDR0		0x00	/* station MAC [0:3], MSB first			*/
#define R_IDR4		0x04	/* station MAC [4:5] in [31:16]			*/
#define R_MAR0		0x08	/* multicast hash [31:0]				*/
#define R_MAR4		0x0C	/* multicast hash [63:32]			*/
#define R_CMD		0x3B	/* 8-bit command: bit0 RST			*/
#define   CMD_RXCHK	0x02	/* RX checksum offload				*/
#define   CMD_RXJUMBO	0x08	/* accept jumbo					*/
#define R_IMR		0x3C	/* 16-bit RX/TX interrupt mask			*/
#define R_ISR		0x3E	/* 16-bit RX/TX interrupt status (W1C)		*/
#define R_TCR		0x40	/* TX control					*/
#define R_RCR		0x44	/* RX control (bit0 = accept-all-physical)	*/
#define R_CPUTAGCR	0x48	/* CPU-tag insert config			*/
#define R_CONFIG	0x4C
#define R_CPUTAG1CR	0x50
#define R_MSR		0x58	/* media/flow status; top byte = force flow ctl	*/
#define R_IMR0		0xD0	/* 32-bit per-ring TX-completion mask		*/
#define R_ISR1		0xD8	/* 32-bit per-ring TX-completion status (W1C)	*/
#define R_TxFDP0	0x1300	/* TX ring0 fetch-descriptor pointer		*/
#define R_TxCDO0	0x1304	/* TX ring0 current-descriptor offset (u16)	*/
#define R_RRING_ROUTE	0x1370	/* RX class -> ring routing			*/
#define R_RxFDP0	0x13F0	/* RX ring0 fetch-descriptor pointer		*/
#define R_RxCDO0	0x13F4	/* RX ring0 current-descriptor offset (u16)	*/
#define R_RxDesNum	0x1430	/* RX ring0 size + flow-control thresholds	*/
#define R_IO_CMD	0x1434	/* DMA enable + per-ring TX kick (bit0 = ring0)	*/
#define R_IO_CMD1	0x1438

/* engine enable values (inherited-config + DMA-enable edge) */
#define IO_CMD_ENABLE	0xc059f130
#define IO_CMD1_ENABLE	0x32000001
#define IMR_RX_BITS	0xf835	/* RX-OK + RX-error + ring descriptor-unavailable */
#define IMR0_TX_BITS	0x3f	/* per-ring TX completion				*/

/* CPU-tag engine config. CTEN_RX (bit31) makes the MAC strip the 8-byte switch
 * tag in hardware on RX and expose the parsed ingress port in the descriptor;
 * the rest selects tag sizes, the 0x04 protocol and the 0x8899 match. Clearing
 * CTEN_RX leaves the raw in-band tag in the delivered frame. */
#define CPUTAGCR_INIT	0x9022FF04
#define CPUTAG1CR_INIT	0x00004000	/* CPU-tag SID base (64 << 8)		*/
#define ABLTY_CPU_FORCE	0xBFFF		/* CPU-port forced-ability mode (keep)	*/

/* descriptor flags (word0 / opts1, both rings) */
#define D_OWN		BIT(31)	/* 1 = owned by the DMA engine			*/
#define D_EOR		BIT(30)	/* end of ring (wrap)				*/
#define D_FS		BIT(29)	/* first segment				*/
#define D_LS		BIT(28)	/* last segment					*/
#define D_TXCRC		BIT(23)	/* TX: append FCS				*/
#define RXD_CRCERR	BIT(27)
#define RXD_DMAERR	BIT(24)
#define RXD_LEN_MASK	0x1fff
#define TXD_LEN_MASK	0x1ffff

#define RX_RING_SIZE	64
#define TX_RING_SIZE	64
#define RX_BUF_SIZE	2048
#define TH_ON_VAL	0x10	/* RX flow-control assert / de-assert thresholds	*/
#define TH_OFF_VAL	0x30

/* ---- switch core (SWCORE), phys 0x1b000000, offsets from the ioremap base -- */
#define SWCORE_PHYS		0x1b000000UL
#define SWCORE_SIZE		0x42000
#define SW_SERDES_LINEMODE	0x00084	/* SerDes egress line mode (0x44 = SGMII)*/
#define SW_FORCE_ABLTY(p)	(0x001CC + (p) * 4)	/* forced ability	*/
#define SW_P_ABLTY(p)		(0x00200 + (p) * 4)	/* live ability (RO)	*/
#define SW_ABLTY_FORCE(p)	(0x00238 + (p) * 4)	/* which abilities forced*/
#define SW_FORCE_ABLTY_X11	0x002F4	/* SerDes/PBO forced ability		*/
#define SW_P_ABLTY_X11		0x002F8	/* SerDes/PBO live ability (RO)		*/
#define SW_ABLTY_FORCE_X11	0x002FC
#define SW_LUT_UNKN_SA		0x1C004	/* unknown-SA action, 2 bits/port	*/
#define SW_LUT_BC_FLOOD		0x1C028	/* broadcast flood, 1 bit/port		*/
#define SW_LUT_UNKN_MC_FLOOD	0x1C02C
#define SW_LUT_UNKN_UC_FLOOD	0x1C030
#define SW_SRC_PORT_PERMIT	0x1C114	/* 1 bit per INGRESS port (the RX gate)	*/
#define SW_CPU_TAG_INSERT	0x230F4	/* CPU-tag insert enable, 1 bit/CPU-GMAC	*/
#define SW_CPU_TAG_AWARE	0x230F8	/* CPU-tag parse enable				*/
/* Per-port spanning-tree / forwarding state. Default instance (CIST) = bits[1:0]
 * of the per-port word; 3 = FORWARDING. The boot loader leaves physical ports in
 * the silicon's non-forwarding reset state -> RX is counted but never L2-forwarded
 * to the CPU port, so this MUST be set on every port. */
#define SW_MSTI_CTRL(p)		(0x1704C + (p) * 4)
#define   STP_STATE_MASK	0x3
#define   STP_FORWARDING	0x3
/* Per-ingress-port egress-allow mask (29-bit; default 0x1FFFFFFF = all). Clearing
 * a port's own bit stops it receiving its own flood (the CPU loopback). */
#define SW_PISO_PORT(p)		(0x27000 + (p) * 4)
#define   PISO_ALL		0x1FFFFFFF

/* VLAN: filtering must not gate CPU<->LAN egress (the boot loader may leave it on
 * with the CPU port outside the member set). */
#define SW_VLAN_CTRL		0x13008
#define   VLAN_FILTERING	BIT(0)
#define SW_VLAN_ACCEPT		0x13000	/* per-port accept-frame-type (0 = accept all) */
#define SW_VLAN_PB_VID		0x1300C	/* per-port default VID (PVID), stride 4	*/

/* Per-port lookup-miss (unknown-DA) action, 2 bits/port; 0 = FORWARD. Needed for
 * the post-ARP unicast / IPv6-ND path (the first broadcast already floods). */
#define SW_UNKN_UC_DA		0x1C00C
#define SW_UNKN_L2_MC		0x1C018
#define SW_UNKN_IP4_MC		0x1C01C
#define SW_UNKN_IP6_MC		0x1C020
#define   DA_ACT_PORTS		0x3FFFFF	/* ports 0..10, 2 bits each		*/

#define ABLTY_1G_FULL_LINK	0x16	/* speed=1000, duplex=full, link=up	*/
#define ABLTY_FORCE_ALL		0xFFF	/* force all basic abilities		*/

#define SW_CPU_PORT		9
#define SW_LAST_PORT		11	/* 0..4,8 copper; 5 PON; 6,7 SerDes; 9 CPU; 11 PBO */
#define SW_PORT_MASK		GENMASK(9, 0)	/* ports 0..9 (LAN+SerDes+PON+CPU)*/

/* switch CPU-port control tag: ethertype 0x8899 followed by 6 control bytes,
 * inserted after the source MAC on frames to/from the CPU. */
#define RTL_CPU_TAG_LEN		8
#define RTL_CPU_TAG_ETYPE	0x8899

/* SoC peripheral control (fixed uncached KSEG1, no ioremap) */
#define SOC_IP_SEL	((void __iomem *)0xb8000600ul)	/* per-engine clock/reset*/
#define   IPSEL_GMAC0	BIT(1)
#define SOC_SW_ENABLE	((void __iomem *)0xb800063cul)	/* switch-core enable	*/
#define   SW_EN_BIT	BIT(5)
#define   SW_PBO_BIT	BIT(25)				/* required on rev > A	*/

/* Internal GPHY indirect-MDIO window (SWCORE 0x000/0x004/0x008). The integrated
 * copper PHYs are addressed by their switch port number; the standard MII page is
 * 0xA40 (so 0xA400 = reg0/BMCR). Busy is RD[16], NOT a CMD bit. */
#define SW_GPHY_WD	0x000
#define SW_GPHY_CMD	0x004
#define SW_GPHY_RD	0x008
#define   GPHY_CMD_EN	BIT(21)		/* start					*/
#define   GPHY_WREN	BIT(22)		/* write strobe					*/
#define   GPHY_RD_BUSY	BIT(16)		/* in the RD register				*/
#define GPHY_MII_PAGE	0xA400		/* page 0xA40, reg 0 (add (reg&7)<<1)		*/
#define SW_SOFTWARE_RST	0x108		/* bit6 = GPHY-macro reset (self-clear)		*/
#define   GPHY_MACRO_RST	BIT(6)
#define SW_GPHY_MISC	0x114		/* bit0 = GPHY patch-done sticky		*/

/* Live link / speed (genuine, independent of the MAC force). Copper genuine link
 * = MDIO BMSR bit2; SerDes genuine link = SDS_FIB_STATUS. */
#define SW_SDS_FIB_STATUS(s)	(0x0028C + (s) * 0x20)	/* SerDes 0..2; port 6 = idx0 */
#define   SDS_LINK_OK		BIT(4)
#define   SDS_SDET		BIT(17)

/* Per-port RX MIB counters (direct reads; block base 0x32600, stride 0x80). */
#define SW_MIB_RX_UCAST(p)	(0x32620 + (p) * 0x80)
#define SW_MIB_RX_MCAST(p)	(0x32628 + (p) * 0x80)
#define SW_MIB_RX_BCAST(p)	(0x3262C + (p) * 0x80)

/* RTL8221B 2.5G PHY reset line: DTS rtl8221b_dev0_reset = <&gpio1 28 1> (active
 * low). gpio1 = bank 1 (pins 32..63); pin 28 -> bit 28 of bank-1 DIR/DAT, plus
 * the GPIO function-enable for pins 32..63. */
#define SW_IO_GPIO_EN_HI	0x03c		/* pinmux function-enable, pins 32..63	*/
#define SOC_GPIO_B1_DIR	((void __iomem *)0xb8003324ul)	/* bank1 direction (1=out)*/
#define SOC_GPIO_B1_DAT	((void __iomem *)0xb8003328ul)	/* bank1 data		*/
#define RTL8221B_RST_BIT	BIT(28)
#define RTL8221B_PHYAD		6

/* MII BMCR/BMSR bits. */
#define MII_PDOWN	0x0800
#define MII_ANENABLE	0x1000
#define MII_ANRESTART	0x0200
#define MII_LSTATUS	0x0004

/* The DMA descriptor address bus window (0 on this SoC). */
#define DMA_BUS_WINDOW	0u

struct rx_desc { u32 opts1, addr, opts2, opts3; };
struct tx_desc { u32 opts1, addr, opts2, opts3, opts4; };

struct rtl9607c_eth {
	struct net_device	*ndev;
	struct device		*dev;
	void __iomem		*base;	/* GMAC0			*/
	void __iomem		*sw;	/* switch core			*/
	int			irq;

	struct napi_struct	napi;
	struct timer_list	backstop;
	struct timer_list	diag;
	int			diag_left;
	spinlock_t		tx_lock;

	/* Plain streaming DMA: the kernel manages the L2 so dma_map/unmap flush +
	 * invalidate it -- no bounce buffers needed. */
	struct rx_desc		*rx_ring;
	dma_addr_t		rx_ring_dma;
	struct sk_buff		*rx_skb[RX_RING_SIZE];
	dma_addr_t		rx_buf_dma[RX_RING_SIZE];
	unsigned int		rx_head;

	struct tx_desc		*tx_ring;
	dma_addr_t		tx_ring_dma;
	struct sk_buff		*tx_skb[TX_RING_SIZE];
	dma_addr_t		tx_buf_dma[TX_RING_SIZE];
	unsigned int		tx_buf_len[TX_RING_SIZE];
	void			*tx_buf[TX_RING_SIZE];	/* per-slot linear copy buffer */
	unsigned int		tx_head, tx_dirty;	/* free-running counters	*/

	int			rx_dumped;
	int			tx_dumped;
};

static inline u32 ep_rd(struct rtl9607c_eth *ep, u32 r) { return ioread32(ep->base + r); }
static inline void ep_wr(struct rtl9607c_eth *ep, u32 r, u32 v) { iowrite32(v, ep->base + r); }
static inline u32 sw_rd(struct rtl9607c_eth *ep, u32 r) { return ioread32(ep->sw + r); }
static inline void sw_wr(struct rtl9607c_eth *ep, u32 r, u32 v) { iowrite32(v, ep->sw + r); }
static inline void sw_or(struct rtl9607c_eth *ep, u32 r, u32 v) { sw_wr(ep, r, sw_rd(ep, r) | v); }

static inline unsigned int tx_slot(unsigned int counter) { return counter % TX_RING_SIZE; }

/* ---- internal GPHY MDIO (indirect window) --------------------------------- */
static int gphy_wait(struct rtl9607c_eth *ep)
{
	int i;

	for (i = 0; i < 10000; i++) {
		if (!(sw_rd(ep, SW_GPHY_RD) & GPHY_RD_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static u16 gphy_read(struct rtl9607c_eth *ep, unsigned int phyad, unsigned int reg)
{
	u32 adr = (phyad << 16) | GPHY_MII_PAGE | ((reg & 7) << 1);

	sw_wr(ep, SW_GPHY_CMD, adr | GPHY_CMD_EN);
	if (gphy_wait(ep))
		return 0xffff;
	return sw_rd(ep, SW_GPHY_RD) & 0xffff;
}

static void gphy_write(struct rtl9607c_eth *ep, unsigned int phyad,
		       unsigned int reg, u16 val)
{
	u32 adr = (phyad << 16) | GPHY_MII_PAGE | ((reg & 7) << 1);

	sw_wr(ep, SW_GPHY_WD, val);
	sw_wr(ep, SW_GPHY_CMD, adr | GPHY_WREN | GPHY_CMD_EN);
	gphy_wait(ep);
}

/* Power up + (re)start auto-negotiation on the integrated copper PHYs so a copper
 * LAN jack actually trains. Each PHY is addressed by its switch port number. */
static void eth_copper_phy_up(struct rtl9607c_eth *ep)
{
	static const u8 copper_ports[] = { 0, 1, 2, 3, 4 };
	unsigned int i;

	/* pulse the GPHY-macro reset, then let it settle. */
	sw_or(ep, SW_SOFTWARE_RST, GPHY_MACRO_RST);
	msleep(50);
	for (i = 0; i < ARRAY_SIZE(copper_ports); i++) {
		unsigned int p = copper_ports[i];
		u16 bmcr = gphy_read(ep, p, 0);

		bmcr &= ~MII_PDOWN;			/* leave power-down	*/
		bmcr |= MII_ANENABLE | MII_ANRESTART;	/* auto-neg + restart	*/
		gphy_write(ep, p, 0, bmcr);
	}
	sw_or(ep, SW_GPHY_MISC, BIT(0));		/* patch-done sticky	*/
}

/* Release the external RTL8221B 2.5G PHY from reset (active-low). This alone does
 * not bring up its SerDes link (HiSGMII mode + analog patch is a separate, larger
 * sequence) but lets the PHY run; combined with the boot-loader-warmed SerDes it
 * gives the host's 2.5G port a chance to stay up. */
static void eth_rtl8221b_reset_release(struct rtl9607c_eth *ep)
{
	/* select GPIO function for the pin (bank-1 / pins 32..63 word). */
	sw_or(ep, SW_IO_GPIO_EN_HI, RTL8221B_RST_BIT);
	/* drive it as an output and pulse reset: assert (low) then release (high). */
	writel(readl(SOC_GPIO_B1_DIR) | RTL8221B_RST_BIT, SOC_GPIO_B1_DIR);
	writel(readl(SOC_GPIO_B1_DAT) & ~RTL8221B_RST_BIT, SOC_GPIO_B1_DAT);
	msleep(10);
	writel(readl(SOC_GPIO_B1_DAT) | RTL8221B_RST_BIT, SOC_GPIO_B1_DAT);
	msleep(10);
}

/* ---- per-port real-link + RX-MIB diagnostic ------------------------------- */
static u32 eth_mib_rx_pkts(struct rtl9607c_eth *ep, unsigned int p)
{
	return sw_rd(ep, SW_MIB_RX_UCAST(p)) + sw_rd(ep, SW_MIB_RX_MCAST(p)) +
	       sw_rd(ep, SW_MIB_RX_BCAST(p));
}

/* Genuine link (independent of the MAC force): copper = MDIO BMSR bit2,
 * SerDes (ports 6,7) = SDS_FIB_STATUS bit4. Returns -1 for ports with no
 * directly-readable PHY/SerDes (5 PON, 8 RGMII, 9 CPU, 10, 11). */
static int eth_port_real_link(struct rtl9607c_eth *ep, unsigned int p)
{
	if (p <= 4)
		return !!(gphy_read(ep, p, 1) & MII_LSTATUS);	/* BMSR */
	if (p == 6 || p == 7)
		return !!(sw_rd(ep, SW_SDS_FIB_STATUS(p - 6)) & SDS_LINK_OK);
	return -1;
}

static void eth_diag_dump(struct rtl9607c_eth *ep)
{
	unsigned int p;

	for (p = 0; p <= SW_LAST_PORT; p++) {
		int link = eth_port_real_link(ep, p);
		u32 ablty = sw_rd(ep, SW_P_ABLTY(p));
		const char *ls = link < 0 ? "n/a  " : (link ? "UP   " : "down ");

		dev_info(ep->dev, "  port %2u: link=%s spdcode=%u stp=%u rxpkts=%u (ablty=%04x)\n",
			 p, ls, (ablty & 3) | (((ablty >> 12) & 3) << 2),
			 sw_rd(ep, SW_MSTI_CTRL(p)) & STP_STATE_MASK,
			 eth_mib_rx_pkts(ep, p), ablty);
	}
	dev_info(ep->dev, "  serdes6 fib=%08x serdes7 fib=%08x\n",
		 sw_rd(ep, SW_SDS_FIB_STATUS(0)), sw_rd(ep, SW_SDS_FIB_STATUS(1)));
}

static void eth_diag_timer(struct timer_list *t)
{
	struct rtl9607c_eth *ep = timer_container_of(ep, t, diag);

	dev_info(ep->dev, "link/rxpkts diag (%d left):\n", ep->diag_left);
	eth_diag_dump(ep);
	if (--ep->diag_left > 0 && diag_ms)
		mod_timer(&ep->diag, jiffies + msecs_to_jiffies(diag_ms));
}

/* ---- station address ------------------------------------------------------ */
static void eth_get_hwaddr(struct rtl9607c_eth *ep, u8 *mac)
{
	u32 lo = ep_rd(ep, R_IDR0), hi = ep_rd(ep, R_IDR4);

	mac[0] = lo >> 24; mac[1] = lo >> 16; mac[2] = lo >> 8; mac[3] = lo;
	mac[4] = hi >> 24; mac[5] = hi >> 16;
}

static void eth_set_hwaddr(struct rtl9607c_eth *ep, const u8 *mac)
{
	ep_wr(ep, R_IDR0, ((u32)mac[0] << 24) | ((u32)mac[1] << 16) |
			  ((u32)mac[2] << 8) | mac[3]);
	ep_wr(ep, R_IDR4, ((u32)mac[4] << 24) | ((u32)mac[5] << 16));
}

/* ---- rings ---------------------------------------------------------------- */
/* Allocate a fresh RX skb, stream-map it, and arm the descriptor on it. */
static int eth_refill(struct rtl9607c_eth *ep, unsigned int idx)
{
	struct sk_buff *skb = netdev_alloc_skb(ep->ndev, RX_BUF_SIZE);
	dma_addr_t da;
	u32 opts1;

	if (!skb)
		return -ENOMEM;
	da = dma_map_single(ep->dev, skb->data, RX_BUF_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}
	ep->rx_skb[idx] = skb;
	ep->rx_buf_dma[idx] = da;
	ep->rx_ring[idx].addr = da | DMA_BUS_WINDOW;
	ep->rx_ring[idx].opts2 = 0;
	ep->rx_ring[idx].opts3 = 0;
	opts1 = D_OWN | RX_BUF_SIZE;
	if (idx == RX_RING_SIZE - 1)
		opts1 |= D_EOR;
	ep->rx_ring[idx].opts1 = opts1;
	return 0;
}

static void eth_free_rings(struct rtl9607c_eth *ep)
{
	unsigned int i;

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (ep->rx_skb[i]) {
			dma_unmap_single(ep->dev, ep->rx_buf_dma[i], RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb_any(ep->rx_skb[i]);
			ep->rx_skb[i] = NULL;
		}
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		kfree(ep->tx_buf[i]);
		ep->tx_buf[i] = NULL;
	}
	/* TX skbs are freed inline at xmit (copied into tx_buf), nothing to free. */
	if (ep->rx_ring)
		dma_free_coherent(ep->dev, RX_RING_SIZE * sizeof(struct rx_desc),
				  ep->rx_ring, ep->rx_ring_dma);
	if (ep->tx_ring)
		dma_free_coherent(ep->dev, TX_RING_SIZE * sizeof(struct tx_desc),
				  ep->tx_ring, ep->tx_ring_dma);
	ep->rx_ring = NULL;
	ep->tx_ring = NULL;
}

static int eth_alloc_rings(struct rtl9607c_eth *ep)
{
	unsigned int i;

	ep->rx_ring = dma_alloc_coherent(ep->dev,
			RX_RING_SIZE * sizeof(struct rx_desc),
			&ep->rx_ring_dma, GFP_KERNEL);
	ep->tx_ring = dma_alloc_coherent(ep->dev,
			TX_RING_SIZE * sizeof(struct tx_desc),
			&ep->tx_ring_dma, GFP_KERNEL);
	if (!ep->rx_ring || !ep->tx_ring)
		return -ENOMEM;

	ep->rx_head = ep->tx_head = ep->tx_dirty = 0;
	for (i = 0; i < TX_RING_SIZE; i++) {
		ep->tx_buf[i] = kmalloc(RX_BUF_SIZE, GFP_KERNEL);
		if (!ep->tx_buf[i])
			return -ENOMEM;
		ep->tx_ring[i].opts1 = (i == TX_RING_SIZE - 1) ? D_EOR : 0;
		ep->tx_skb[i] = NULL;
	}
	for (i = 0; i < RX_RING_SIZE; i++)
		if (eth_refill(ep, i))
			return -ENOMEM;
	return 0;
}

/* ---- switch open-L2 bring-up (ordered; see file header) ------------------- */
static void eth_switch_init(struct rtl9607c_eth *ep)
{
	unsigned int p;

	/* 1. enable the switch IP block (SoC control, not in SWCORE). */
	writel(readl(SOC_SW_ENABLE) | SW_EN_BIT | SW_PBO_BIT, SOC_SW_ENABLE);

	/* 2. re-assert the SerDes egress line mode (boot-ROM leaves it warm; a
	 *    fresh value stops the uplink decaying before the rootfs mounts). */
	sw_wr(ep, SW_SERDES_LINEMODE, 0x44);

	/* 3. bring the physical PHYs up so a host on a jack actually trains a real
	 *    link (MAC-force alone only sets the MAC-side bit). Copper jacks: power
	 *    up + auto-neg the integrated PHYs. SerDes-6 (external RTL8221B 2.5G):
	 *    release its reset so it runs (full HiSGMII SerDes bring-up is a larger
	 *    sequence, added separately). */
	if (copper_phy)
		eth_copper_phy_up(ep);
	if (rtl8221b_phy)
		eth_rtl8221b_reset_release(ep);

	/* 4. open the L2 forwarding plane. */
	for (p = 0; p <= SW_CPU_PORT; p++) {
		u32 reg = SW_LUT_UNKN_SA + (p / 16) * 4;

		/* unknown-source-MAC action 0 = learn + forward */
		sw_wr(ep, reg, sw_rd(ep, reg) & ~(3u << ((p % 16) * 2)));
	}
	sw_or(ep, SW_LUT_BC_FLOOD, SW_PORT_MASK);
	sw_or(ep, SW_LUT_UNKN_MC_FLOOD, SW_PORT_MASK);
	sw_or(ep, SW_LUT_UNKN_UC_FLOOD, SW_PORT_MASK);
	/* SW_SRC_PORT_PERMIT (0x1C114) is a per-source-port EGRESS-FILTER ENABLE
	 * (EN, 1 bit/port), NOT a permit bitmap: EN=1 turns on source-port egress
	 * filtering and DROPS the forwarded frame after lookup/flood selection. The
	 * working firmware leaves it 0 (no filtering = forward). Writing all-ones here
	 * silently dropped every LAN->CPU frame (port RX climbed, CPU RX stayed flat).
	 * Correct forwarding-permissive value is 0. */
	sw_wr(ep, SW_SRC_PORT_PERMIT, 0x00000000);

	/* 4a2. per-port unknown-DA lookup-miss action = FORWARD(0). We only set the
	 *      unknown-SOURCE action above; the unknown-DESTINATION action must also
	 *      forward, else post-ARP unicast / IPv6-ND to a not-yet-learned MAC is
	 *      dropped instead of flooded. Clear the 2-bit field for ports 0..10. */
	sw_wr(ep, SW_UNKN_UC_DA,  sw_rd(ep, SW_UNKN_UC_DA)  & ~DA_ACT_PORTS);
	sw_wr(ep, SW_UNKN_L2_MC,  sw_rd(ep, SW_UNKN_L2_MC)  & ~DA_ACT_PORTS);
	sw_wr(ep, SW_UNKN_IP4_MC, sw_rd(ep, SW_UNKN_IP4_MC) & ~DA_ACT_PORTS);
	sw_wr(ep, SW_UNKN_IP6_MC, sw_rd(ep, SW_UNKN_IP6_MC) & ~DA_ACT_PORTS);

	/* 4a3. VLAN must not gate CPU<->LAN egress. The boot loader can leave VLAN
	 *      filtering ON with the CPU port outside the member set, which silently
	 *      drops the CPU's reply to the host (RX fixed, but ping return blocked).
	 *      Report the live state, then disable filtering for flat-L2 forwarding. */
	{
		u32 vc = sw_rd(ep, SW_VLAN_CTRL);

		sw_wr(ep, SW_VLAN_CTRL, vc & ~VLAN_FILTERING);
		dev_info(ep->dev, "vlan_ctrl %08x (filtering %s) -> %08x\n",
			 vc, (vc & VLAN_FILTERING) ? "ON" : "off",
			 sw_rd(ep, SW_VLAN_CTRL));
	}

	/* 4b. set every port's spanning-tree state to FORWARDING. The boot loader
	 *     leaves physical ports in the non-forwarding reset state, so an ingress
	 *     frame is RX-counted but never L2-forwarded to the CPU port — this is
	 *     what blocks port 3 -> CPU 9 (CPU-injected TX bypasses ingress STP, so
	 *     it looked like only the CPU->all direction worked). */
	for (p = 0; p <= SW_LAST_PORT; p++) {
		u32 v = sw_rd(ep, SW_MSTI_CTRL(p));

		sw_wr(ep, SW_MSTI_CTRL(p), (v & ~STP_STATE_MASK) | STP_FORWARDING);
	}

	/* 4c. (optional) drop the CPU port from its own egress flood so it stops
	 *     receiving the broadcasts it injected (the observed self-loopback). */
	if (cpu_no_loopback)
		sw_wr(ep, SW_PISO_PORT(SW_CPU_PORT), PISO_ALL & ~BIT(SW_CPU_PORT));

	/* 5. force every port's MAC link up (no PHY autoneg). The CPU port (9) is
	 *    the internal MAC<->switch link the boot loader already set up with a
	 *    specific forced-ability mode; preserve that mode and only OR in the
	 *    link bit, so we don't disturb the working internal link (clobbering it
	 *    can stop the switch egressing CPU-injected frames). */
	for (p = 0; p <= SW_LAST_PORT; p++) {
		if (p == SW_CPU_PORT) {
			sw_or(ep, SW_FORCE_ABLTY(p), BIT(4));
			sw_wr(ep, SW_ABLTY_FORCE(p), ABLTY_CPU_FORCE);
		} else {
			sw_wr(ep, SW_FORCE_ABLTY(p), ABLTY_1G_FULL_LINK);
			sw_wr(ep, SW_ABLTY_FORCE(p), ABLTY_FORCE_ALL);
		}
	}
	sw_wr(ep, SW_FORCE_ABLTY_X11, ABLTY_1G_FULL_LINK);
	sw_wr(ep, SW_ABLTY_FORCE_X11, ABLTY_FORCE_ALL);

	/* 6. CPU-tag engine. The switch frames RX to the CPU with the 0x8899 tag
	 *    by default, so the CPU port is tag-aware; keep it on unless asked. */
	if (sw_cpu_tag) {
		sw_or(ep, SW_CPU_TAG_INSERT, BIT(0));
		sw_or(ep, SW_CPU_TAG_AWARE, BIT(0));
	}

	dev_info(ep->dev, "switch: open-L2 up (cpu-port %u, src-permit=%08x, cpu-ablty=%04x)\n",
		 SW_CPU_PORT, sw_rd(ep, SW_SRC_PORT_PERMIT),
		 sw_rd(ep, SW_P_ABLTY(SW_CPU_PORT)));
	/* Baseline real-link snapshot; the periodic diag (armed at open) then shows
	 * which port's genuine link comes up + rxpkts climb under host traffic. */
	eth_diag_dump(ep);
}

/* ---- MAC engine ----------------------------------------------------------- */
static void eth_hw_stop(struct rtl9607c_eth *ep)
{
	ep_wr(ep, R_IO_CMD, 0);
	ep_wr(ep, R_IO_CMD1, 0);
	iowrite16(0, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, 0);
	iowrite16(0xffff, ep->base + R_ISR);
	ep_wr(ep, R_ISR1, 0xffffffff);
	udelay(10);
}

/* GMAC0 IP-block power-cycle: the multi-ring fetch engine only latches its ring
 * state across this reset edge, so it must run before the descriptor program. */
static void eth_ipsel_cycle(void)
{
	writel(readl(SOC_IP_SEL) & ~IPSEL_GMAC0, SOC_IP_SEL);
	msleep(12);
	writel(readl(SOC_IP_SEL) | IPSEL_GMAC0, SOC_IP_SEL);
	msleep(2);
}

static void eth_hw_program(struct rtl9607c_eth *ep)
{
	u32 desnum;

	iowrite8(CMD_RXCHK | CMD_RXJUMBO, ep->base + R_CMD);
	ep_wr(ep, R_TCR, 0x00000C01);		/* IFG=3, TX enabled via IO_CMD	*/
	ep_wr(ep, R_RCR, 0x0000000E);		/* accept broadcast + matching	*/
	ep_wr(ep, R_CONFIG, 0x21000000);
	/* Enable the CPU-tag engine so the MAC strips the in-band 8-byte switch
	 * tag on RX (CTEN_RX); TX stays plain (per-descriptor opts2.cputag=0 =>
	 * the switch forwards by L2 lookup). The IP-block reset above clears this,
	 * so it must be re-asserted here. */
	ep_wr(ep, R_CPUTAGCR, CPUTAGCR_INIT);
	ep_wr(ep, R_CPUTAG1CR, CPUTAG1CR_INIT);

	/* ring pointers (writable only while the engine is stopped). */
	ep_wr(ep, R_TxFDP0, ep->tx_ring_dma | DMA_BUS_WINDOW);
	iowrite16(0, ep->base + R_TxCDO0);
	ep_wr(ep, R_RxFDP0, ep->rx_ring_dma | DMA_BUS_WINDOW);
	desnum = ((RX_RING_SIZE - 1) & 0xff) << 24 | (TH_ON_VAL & 0xff) << 16 |
		 (TH_OFF_VAL & 0xff) << 8 | (((RX_RING_SIZE - 1) >> 8) & 0xf) << 4;
	ep_wr(ep, R_RxDesNum, desnum);
	iowrite16(((RX_RING_SIZE - 1) & 0xff) << 8 |
		  (((RX_RING_SIZE - 1) >> 8) & 0xf) << 4, ep->base + R_RxCDO0);
	/* route every RX class to ring 0. */
	{
		unsigned int k;

		for (k = 0; k < 7; k++)
			ep_wr(ep, R_RRING_ROUTE + k * 4, 0);
	}

	/* force the internal MAC<->switch link flow-control (top byte of MSR). */
	ep_wr(ep, R_MSR, (ep_rd(ep, R_MSR) & 0x00ffffff) | 0xf0000000);
	eth_set_hwaddr(ep, ep->ndev->dev_addr);
	ep_wr(ep, R_MAR0, 0xffffffff);
	ep_wr(ep, R_MAR4, 0xffffffff);

	/* enable edge: IO_CMD1 first, then IO_CMD (latches the fetch engine). */
	ep_wr(ep, R_IO_CMD1, IO_CMD1_ENABLE);
	ep_wr(ep, R_IO_CMD, IO_CMD_ENABLE);

	iowrite16(0xffff, ep->base + R_ISR);
	ep_wr(ep, R_ISR1, 0xffffffff);
	iowrite16(IMR_RX_BITS, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, IMR0_TX_BITS);
}

/* ---- RX / TX datapath ----------------------------------------------------- */
static int eth_rx(struct rtl9607c_eth *ep, int budget)
{
	struct net_device *ndev = ep->ndev;
	int done = 0;

	while (done < budget) {
		unsigned int i = ep->rx_head;
		u32 opts1 = ep->rx_ring[i].opts1;
		struct sk_buff *skb;
		u32 len;

		if (opts1 & D_OWN)		/* still HW-owned */
			break;

		len = opts1 & RXD_LEN_MASK;
		skb = ep->rx_skb[i];
		dma_unmap_single(ep->dev, ep->rx_buf_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		if (ep->rx_dumped < rx_dump && len) {
			ep->rx_dumped++;
			print_hex_dump(KERN_INFO, "rx0: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, min_t(u32, len, 32), false);
		}

		if ((opts1 & (RXD_CRCERR | RXD_DMAERR)) ||
		    len <= (u32)rx_prefix + ETH_HLEN || len > RX_BUF_SIZE) {
			ndev->stats.rx_errors++;
			dev_kfree_skb_any(skb);
		} else {
			/* The CPU port frames a packet as:
			 *   [front prefix][DA][SA][switch tag][ethertype][payload]
			 * Strip the fixed front prefix, then excise the 8-byte 0x8899
			 * switch tag (if present) by sliding DA+SA over it. */
			skb_put(skb, len);
			if (rx_prefix)
				skb_pull(skb, rx_prefix);
			if (skb->len > 2 * ETH_ALEN + RTL_CPU_TAG_LEN &&
			    skb->data[2 * ETH_ALEN] == 0x88 &&
			    skb->data[2 * ETH_ALEN + 1] == 0x99) {
				if (ep->rx_dumped <= rx_dump)
					dev_info(ep->dev, "rx tag: %*ph\n",
						 RTL_CPU_TAG_LEN,
						 skb->data + 2 * ETH_ALEN);
				memmove(skb->data + RTL_CPU_TAG_LEN, skb->data,
					2 * ETH_ALEN);
				skb_pull(skb, RTL_CPU_TAG_LEN);
			}
			/* Drop our own egress that the switch flooded back to the CPU
			 * port (source MAC == ours) so the bridge does not log
			 * "received packet ... with own address as source". */
			if (skb->len >= 2 * ETH_ALEN &&
			    ether_addr_equal(skb->data + ETH_ALEN, ndev->dev_addr)) {
				dev_kfree_skb_any(skb);
			} else {
				/* NAPI poll context: use the receive path, not netif_rx. */
				skb->protocol = eth_type_trans(skb, ndev);
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += len;
				napi_gro_receive(&ep->napi, skb);
			}
		}

		if (eth_refill(ep, i)) {	/* re-arm with a fresh skb */
			ndev->stats.rx_dropped++;
			break;
		}
		ep->rx_head = (i + 1) % RX_RING_SIZE;
		done++;
	}
	return done;
}

static void eth_tx_reclaim(struct rtl9607c_eth *ep)
{
	/* The skb was already freed at xmit (its bytes were copied into tx_buf),
	 * so reclaim just unmaps the copy buffer and releases the ring slot once
	 * the DMA engine has handed the descriptor back (OWN cleared). */
	while (ep->tx_dirty != ep->tx_head) {
		unsigned int i = tx_slot(ep->tx_dirty);

		if (ep->tx_ring[i].opts1 & D_OWN)	/* not transmitted yet */
			break;
		dma_unmap_single(ep->dev, ep->tx_buf_dma[i], ep->tx_buf_len[i],
				 DMA_TO_DEVICE);
		ep->tx_dirty++;
	}
	if (netif_queue_stopped(ep->ndev) &&
	    (ep->tx_head - ep->tx_dirty) < TX_RING_SIZE - 1)
		netif_wake_queue(ep->ndev);
}

static int eth_napi_poll(struct napi_struct *napi, int budget)
{
	struct rtl9607c_eth *ep = container_of(napi, struct rtl9607c_eth, napi);
	unsigned long flags;
	int work;

	work = eth_rx(ep, budget);
	spin_lock_irqsave(&ep->tx_lock, flags);
	eth_tx_reclaim(ep);
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	if (work < budget) {
		napi_complete_done(napi, work);
		/* W1C any status latched while masked, then re-unmask. */
		iowrite16(ioread16(ep->base + R_ISR), ep->base + R_ISR);
		ep_wr(ep, R_ISR1, ep_rd(ep, R_ISR1));
		iowrite16(IMR_RX_BITS, ep->base + R_IMR);
		ep_wr(ep, R_IMR0, IMR0_TX_BITS);
	}
	return work;
}

static irqreturn_t eth_irq(int irq, void *data)
{
	struct rtl9607c_eth *ep = netdev_priv((struct net_device *)data);
	u16 isr = ioread16(ep->base + R_ISR);

	if (!isr && !ep_rd(ep, R_ISR1))
		return IRQ_NONE;
	/* mask and let NAPI drain + re-unmask. */
	iowrite16(0, ep->base + R_IMR);
	ep_wr(ep, R_IMR0, 0);
	napi_schedule(&ep->napi);
	return IRQ_HANDLED;
}

/* Backstop drain: catches a missed/unrouted IRQ so the datapath always makes
 * progress during bring-up (ping does not need IRQ latency). */
static void eth_backstop(struct timer_list *t)
{
	struct rtl9607c_eth *ep = timer_container_of(ep, t, backstop);

	napi_schedule(&ep->napi);
	mod_timer(&ep->backstop, jiffies + msecs_to_jiffies(backstop_ms));
}

static netdev_tx_t eth_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rtl9607c_eth *ep = netdev_priv(ndev);
	unsigned long flags;
	unsigned int i, len = skb->len;
	dma_addr_t da;
	void *buf;
	u32 opts1;

	if (len > RX_BUF_SIZE) {		/* must fit a copy slot */
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
	i = tx_slot(ep->tx_head);
	buf = ep->tx_buf[i];

	/* Copy the WHOLE frame into the linear copy slot. Use skb_copy_bits, not a
	 * flat memcpy(skb->data,...): non-linear/fragmented skbs (e.g. the ICMP echo
	 * reply assembled by ip_append_data in the RX softirq) keep only skb_headlen
	 * bytes at skb->data and the rest in frags — a flat memcpy would copy the
	 * header plus out-of-bounds garbage, which the host then drops. The buffer is
	 * then stream-mapped; the kernel-managed L2 makes that DMA coherent. */
	if (skb_copy_bits(skb, 0, buf, len)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (len < ETH_ZLEN) {		/* zero-pad runt frames (e.g. a 42-byte ARP reply)
					 * in the copy buffer and extend the DMA length;
					 * skb_padto only guarantees tailroom, it does NOT
					 * grow skb->len, so we pad here after the copy. */
		memset((u8 *)buf + len, 0, ETH_ZLEN - len);
		len = ETH_ZLEN;
	}

	if (ep->tx_dumped < tx_dump) {
		ep->tx_dumped++;
		print_hex_dump(KERN_INFO, "tx0: ", DUMP_PREFIX_OFFSET, 16, 1,
			       buf, min_t(unsigned int, len, 48), false);
	}

	da = dma_map_single(ep->dev, buf, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;
	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;
	ep->tx_ring[i].opts2 = 0;	/* plain frame: switch forwards by L2 DA */
	ep->tx_ring[i].opts3 = 0;
	ep->tx_ring[i].opts4 = 0;
	opts1 = D_OWN | D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
	if (i == TX_RING_SIZE - 1)
		opts1 |= D_EOR;
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = opts1;
	wmb();

	ep->tx_head++;
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick ring 0 */
	spin_unlock_irqrestore(&ep->tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	dev_consume_skb_any(skb);	/* bytes copied; release immediately */
	return NETDEV_TX_OK;
}

static void eth_set_rx_mode(struct net_device *ndev)
{
	struct rtl9607c_eth *ep = netdev_priv(ndev);
	u32 rcr = ep_rd(ep, R_RCR);

	/* a bridge enslaving eth0 sets promisc; accept-all-physical (RCR bit0)
	 * is then required to receive LAN-client frames whose DA != our MAC. */
	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))
		rcr |= BIT(0);
	else
		rcr &= ~BIT(0);
	ep_wr(ep, R_RCR, rcr);
}

static int eth_set_mac_address(struct net_device *ndev, void *addr)
{
	struct rtl9607c_eth *ep = netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, addr);

	if (!ret)
		eth_set_hwaddr(ep, ndev->dev_addr);
	return ret;
}

/* ---- open / stop ---------------------------------------------------------- */
static int eth_open(struct net_device *ndev)
{
	struct rtl9607c_eth *ep = netdev_priv(ndev);
	int ret;

	ret = eth_alloc_rings(ep);
	if (ret) {
		eth_free_rings(ep);
		return ret;
	}

	eth_hw_stop(ep);
	eth_ipsel_cycle();
	eth_switch_init(ep);		/* program forwarding before the DMA starts */
	eth_hw_program(ep);

	if (ep->irq > 0) {
		ret = request_irq(ep->irq, eth_irq, 0, ndev->name, ndev);
		if (ret) {
			netdev_warn(ndev, "IRQ %d request failed (%d); poll-only\n",
				    ep->irq, ret);
			ep->irq = -1;
		}
	}
	napi_enable(&ep->napi);
	timer_setup(&ep->backstop, eth_backstop, 0);
	mod_timer(&ep->backstop, jiffies + msecs_to_jiffies(backstop_ms));

	/* periodic real-link/rxpkts diagnostic (bring-up: locate the host's jack). */
	ep->diag_left = diag_count;
	timer_setup(&ep->diag, eth_diag_timer, 0);
	if (diag_ms && diag_count > 0)
		mod_timer(&ep->diag, jiffies + msecs_to_jiffies(diag_ms));

	netif_start_queue(ndev);
	netif_carrier_on(ndev);
	netdev_info(ndev, "up: irq=%d rx_prefix=%d backstop=%ums copper_phy=%d rtl8221b=%d\n",
		    ep->irq, rx_prefix, backstop_ms, copper_phy, rtl8221b_phy);
	return 0;
}

static int eth_stop(struct net_device *ndev)
{
	struct rtl9607c_eth *ep = netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	timer_delete_sync(&ep->diag);
	timer_delete_sync(&ep->backstop);
	napi_disable(&ep->napi);
	if (ep->irq > 0)
		free_irq(ep->irq, ndev);
	eth_hw_stop(ep);
	eth_free_rings(ep);
	return 0;
}

static const struct net_device_ops rtl9607c_eth_netdev_ops = {
	.ndo_open		= eth_open,
	.ndo_stop		= eth_stop,
	.ndo_start_xmit		= eth_xmit,
	.ndo_set_rx_mode	= eth_set_rx_mode,
	.ndo_set_mac_address	= eth_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
};

/* ---- probe ---------------------------------------------------------------- */
static int rtl9607c_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct rtl9607c_eth *ep;
	u8 mac[ETH_ALEN];
	int ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*ep));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);

	ep = netdev_priv(ndev);
	ep->ndev = ndev;
	ep->dev = dev;
	spin_lock_init(&ep->tx_lock);

	ep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ep->base))
		return PTR_ERR(ep->base);
	ep->sw = devm_ioremap(dev, SWCORE_PHYS, SWCORE_SIZE);
	if (!ep->sw)
		return -ENOMEM;

	/* MAC: DT/nvmem, else the value the boot loader programmed into the MAC
	 * engine, else a random locally-administered address. */
	if (of_get_ethdev_address(dev->of_node, ndev)) {
		eth_get_hwaddr(ep, mac);
		if (is_valid_ether_addr(mac))
			eth_hw_addr_set(ndev, mac);
		else
			eth_hw_addr_random(ndev);
	}

	ndev->netdev_ops = &rtl9607c_eth_netdev_ops;
	netif_carrier_off(ndev);
	netif_napi_add(ndev, &ep->napi, eth_napi_poll);

	ep->irq = platform_get_irq_optional(pdev, 0);
	if (ep->irq < 0)
		ep->irq = -1;

	ret = devm_register_netdev(dev, ndev);
	if (ret)
		return ret;

	dev_info(dev, "RTL9607C NIC at %pR, MAC %pM, irq %d\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 ndev->dev_addr, ep->irq);
	return 0;
}

static const struct of_device_id rtl9607c_eth_of_match[] = {
	{ .compatible = "realtek,rtl9607c-nic" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl9607c_eth_of_match);

static struct platform_driver rtl9607c_eth_driver = {
	.probe	= rtl9607c_eth_probe,
	.driver	= {
		.name		= "rtl9607c-eth",
		.of_match_table	= rtl9607c_eth_of_match,
	},
};
module_platform_driver(rtl9607c_eth_driver);

/*
 * GPON OMCI glue stubs — the shared GPON FSM (gpon-rtl9602c.c) references these
 * symbols declared in rtl9602c_gpon_nic.h. On the 9602C the real implementations
 * live in rtl9602c_eth.c; on the 9607C the OMCI datapath is M4 and these are
 * minimal no-ops so the kernel links. They are enough for M3 (reach O5 + DS).
 */
#include "rtl9602c_gpon_nic.h"

void rtl9602c_eth_set_omci_sid(unsigned int sid) { }
EXPORT_SYMBOL(rtl9602c_eth_set_omci_sid);

void rtl9602c_eth_set_omci_identity(const u8 *sn8) { }
EXPORT_SYMBOL(rtl9602c_eth_set_omci_identity);

u32 rtl9602c_eth_omci_rx_count(void) { return 0; }
EXPORT_SYMBOL(rtl9602c_eth_omci_rx_count);

u32 rtl9602c_eth_wan_rx_count(void) { return 0; }
EXPORT_SYMBOL(rtl9602c_eth_wan_rx_count);

u32 rtl9602c_eth_omci_tx_dirty(void) { return 0; }
EXPORT_SYMBOL(rtl9602c_eth_omci_tx_dirty);

void rtl9602c_eth_omci_selftest(void) { }
EXPORT_SYMBOL(rtl9602c_eth_omci_selftest);

void rtl9602c_eth_omci_report_oper_up(void) { }
EXPORT_SYMBOL(rtl9602c_eth_omci_report_oper_up);

MODULE_DESCRIPTION("Realtek RTL9607C GMAC0 + switch Ethernet driver");
MODULE_LICENSE("GPL");
