// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ethernet driver for the Realtek RTL9602C (RLX) GPON SoC.
 *
 * Independent implementation from the SoC's register interface and the
 * G.984/G.988 protocols. Drives the integrated GMAC/DMA NIC that attaches to
 * the SoC switch core. This increment adds the descriptor-ring DMA datapath
 * in POLLED mode (a periodic timer drains RX and reclaims TX) so the rings
 * can be validated before the NIC interrupt input is identified. The switch
 * and the GMAC control registers are left in the state the bootloader
 * configured: the bootloader used this NIC+switch for its TFTP transfer, so
 * re-asserting the established control words (IO_CMD last) brings the datapath
 * up without recomputing every enable bit. NAPI + hardware interrupt replace
 * the poll timer once the INTC input is known.
 *
 * Registers are accessed native (ioread32/iowrite32) — the SoC presents them
 * big-endian, matching the CPU.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "rtl9602c_gpon_nic.h"

/* GMAC register offsets (from the NIC base). */
#define R_IDR0		0x00	/* MAC[0:3] (native word) */
#define R_IDR4		0x04	/* MAC[4:5] in [31:16] */
#define R_TCR		0x40	/* TX control */
#define R_RCR		0x44	/* RX control */
#define R_CPUTAGCR	0x48
#define R_CONFIG	0x4C
#define R_CPUTAG1CR	0x50
#define R_TxFDP1	0x1300	/* TX ring0 fetch-descriptor pointer (phys) */
#define R_TxCDO1	0x1304	/* TX ring0 current-descriptor offset (u16) */
#define R_RxFDP		0x13F0	/* RX ring0 fetch-descriptor pointer (phys) */
#define R_RxCDO		0x13F4	/* RX ring0 current-descriptor offset */
#define R_RxDesNum	0x1430	/* RX ring0 size / flow-control thresholds */
#define R_IO_CMD	0x1434	/* DMA enable + TX kick (rings 0-3 = bits 0-3) */
#define R_IO_CMD1	0x1438
#define R_RRING_ROUTING1 0x1370	/* RX-ring routing by priority: PRI_n_ROUTE = ring# at nibble n (operational default 0x65432100). 0 => all priorities to ring 0. */

/* Descriptor opts1 bits (shared TX/RX where noted). */
#define D_OWN		BIT(31)	/* 1 = HW owns */
#define D_EOR		BIT(30)	/* end of ring (wrap) */
#define D_FS		BIT(29)	/* first segment */
#define D_LS		BIT(28)	/* last segment */
#define D_IPCS		BIT(27)	/* TX: insert IPv4 csum */
#define D_TXCRC		BIT(23)	/* TX: append FCS */
#define RXD_CRCERR	BIT(27)	/* RX: CRC error */
#define RXD_RCDF	BIT(24)	/* RX: DMA error */
#define RXD_LEN_MASK	0x1fff	/* RX length (low bits of opts1) */
#define TXD_LEN_MASK	0x1ffff	/* TX length */

/*
 * TX descriptor CPU-tag fields (the GMAC tx_info layout, selected by the GMAC
 * CPUtagCR = 0x9022FF04). The GMAC reads these descriptor bits and INSERTS the
 * on-wire cpu-tag itself; we send a plain frame. Directed egress requires a
 * NON-ZERO tx_portmask — a zero CPU-netdev mask makes the switch fall back to
 * an empty L2 DA lookup and drop the frame (the "HW emits portmask 0" symptom).
 */
#define TXD2_CPUTAG	BIT(31)		/* opts2: descriptor carries cpu-tag fields */
#define TXD2_PMASK_SHIFT 16		/* opts2: tx_portmask occupies bits 26..16 */
#define TXD3_KEEP	BIT(23)		/* opts3: switch must not modify the frame */
#define TXD3_DISLRN	BIT(21)		/* opts3: do not learn the CPU SA */
#define TXD3_L34_KEEP	BIT(17)		/* opts3: do not L3/L4-filter the injected frame */
/* Egress port bitmask for CPU->LAN. RTL9602C port map: port 0 = FE LAN (100M),
 * port 1 = GE LAN (1000M), port 2 = PON/fiber, port 3 = CPU. Egress to the two
 * LAN jacks (0,1) only — NOT port 2 (PON, would go to the OLT) and NOT port 3
 * (CPU). The earlier 0x2f targeted PON + a nonexistent port 5. */
#define TXD_EGRESS_PMASK 0x3

/* SoC NIC-DMA bus window: the bootloader ORs 0x20000000 into ring/desc addrs
 * (an artifact of its 1:1 map). Observed to be a NO-OP for TX egress and to
 * DEGRADE RX (Linux dma_alloc_coherent already yields correct bus addrs;
 * OR-ing the window corrupts them). Set 0 to disable — kept as a named knob. */
#define DMA_BUS_WINDOW	0x00000000u

#define RX_RING_SIZE	64
#define TX_RING_SIZE	64
#define RX_BUF_SIZE	2048
#define RX_CPU_PREFIX	2	/* switch CPU-port prepends a 2-byte offset word on RX */
/*
 * TX_CPUTAG selects the CPU->switch egress method:
 *   1 = prepend the software 0x8899 cpu-tag and rely on the switch TAG_AWARE
 *       parser to do directed egress per word3 portmask (rtl8_4 model).
 *   0 = send a PLAIN frame (no tag); switch floods/forwards by DA within VID1
 *       (TAG_AWARE off, VLAN filtering on). Does not depend on this SoC's
 *       cpu-tag format. Diagnostic: method 1's frames never reach the host on
 *       any port (directed egress reads portmask 0) — this SoC's cpu-tag layout
 *       differs from mainline rtl8_4.
 */
#define TX_CPUTAG	0
#define TH_ON_VAL	0x10
#define TH_OFF_VAL	0x30
#define POLL_INTERVAL	msecs_to_jiffies(2)

struct rx_desc { u32 opts1, addr, opts2, opts3; };
struct tx_desc { u32 opts1, addr, opts2, opts3, opts4; };

/*
 * SoC switch core (SWCORE), phys 0x1B000000. The switch has 4 ports (0-3); the
 * CPU port (where GMAC0 attaches) is port 3. Flood masks have one bit per port.
 */
#define SWCORE_PHYS	0x1B000000UL
#define SWCORE_SIZE	0x40000	/* must cover MIB @0x32000 + PISO @0x27000 (was 0x24000 = too small, those fell outside the ioremap!) */
#define SW_CPU_PORT		3
/* per-port forced-ability value + force-mode (RTL9602C register map: base 0x180
 * / 0x1B4, stride 4). FORCE_P_ABLTY holds speed/duplex/link; ABLTY_FORCE_MODE
 * = 0xFFF forces all of them. */
#define SW_FORCE_P_ABLTY(p)	(0x180 + ((p) << 2))
#define SW_ABLTY_FORCE_MODE(p)	(0x1B4 + ((p) << 2))
#define SW_LUT_BC_FLOOD		0x1C020	/* bit[port]: flood broadcast to port */
#define SW_LUT_UNKN_MC_FLOOD	0x1C024
#define SW_LUT_UNKN_UC_FLOOD	0x1C028
#define SW_SRC_PORT_PERMIT	0x1C088	/* RTL9602C L2_SRC_PORT_PERMIT, 1 bit/port; 0 drops ingress.
					 * WAS 0x1C114 (= QOS_PB_PRI on this chip — WRONG): the CPU
					 * port's ingress permit was never set, so the fabric dropped
					 * every CPU-injected frame after DMA (TX counter++ / no egress). */
#define SW_SYS_LRN_LIMITNO	0x17018	/* system MAC-learn limit [10:0]; 0 = no learning */
#define SW_LUT_UNKN_UC_DA_CTRL	0x1C008	/* per-port unknown-UC DLF action, 16-bit/port (2-byte stride); ACT[1:0]: 0=fwd 1=drop 2=trap2cpu */
#define SW_DLF_ACT_TRAP2CPU	2
#define SW_PISO_PORT		0x27000	/* per-port egress-forward (isolation) mask, 11b/port bit-packed; set bit = may forward to that port. all-1s = no isolation */
/* Forced ability value: 1000M (speed[1:0]=2) + full duplex (b2) + link-up (b4) */
#define SW_ABLTY_1G_FD_UP	(0x2 | BIT(2) | BIT(4))
#define SW_PORTS_ALL		0xf	/* ports 0-3 (incl CPU port 3) */
/* MAC_CPU_TAG_CTRL: TAG_AWARE[9] makes the switch parse the CPU-tag on CPU-port
 * ingress and STRIP it before physical egress; TRAP_TAGET_INSERT_EN[8] inserts
 * a CPU-tag on frames trapped to the CPU. */
#define SW_MAC_CPU_TAG_CTRL	0x23030
#define SW_TAG_AWARE		BIT(9)
#define SW_TRAP_TAG_INSERT_EN	BIT(8)
/* VLAN filtering: VLAN_CTRL @ 0x13008 bit0 = VLAN_FILTERING; VLAN_INGRESS @
 * 0x13004 = per-port ingress filter. The operational config enables both + a
 * default VLAN; for flat bring-up we DISABLE them so a parsed cpu-tag's directed
 * egress is not dropped by VLAN membership checks (we have no VLAN table set up). */
#define SW_VLAN_INGRESS		0x13004
#define SW_VLAN_CTRL		0x13008
#define SW_VLAN_FILTERING	BIT(0)
#define SW_VLAN_ACCEPT		0x13000	/* per-port accept-frame-type (0 = accept all) */
#define SW_VLAN_PB_VID		0x1300C	/* per-port PVID, stride 4; VID = bits[11:0] */
/* Operational value: VLAN_CTRL=0x19 (filtering + VID0/VID4095 type bits). */
#define SW_VLAN_CTRL_VAL	0x18	/* VLAN filtering OFF: descriptor cpu-tag does directed egress; keeps RX working */
#define SW_DEFAULT_VID		1
/* Indirect VLAN 4k-table access (field positions):
 * TBL_ACCESS_CTRL[31]=start [20:9]=addr/VID [6:4]=method(1) [3]=cmd(1=write)
 * [2:0]=type(1=VLAN); STS bit13=BUSY; WR_DATA holds the entry word. */
#define SW_TBL_CTRL		0x12000
#define SW_TBL_STS		0x12004
#define SW_TBL_WRDATA		0x12008
#define SW_TBL_BUSY		BIT(13)
#define SW_TBL_VLAN_WR(vid)	(BIT(31) | (((vid) & 0xfff) << 9) | (1u << 4) | (1u << 3) | 1u)

/* OMCI (OMCC) trap. The GTC de-encapsulates DS OMCI GEM frames and delivers them
 * to the CPU port tagged with rx-reason OMCI from the PON port; CPUTAG1CR[14:8]
 * selects which DS stream-id the GMAC traps to the CPU. */
#define RTL9602C_OMCI_REASON	246	/* RX cpu-tag reason code = OMCI */
#define RTL9602C_PON_PORT	2	/* PON/fiber switch port */
#define CPUTAG1_OMCI_SID(s)	(((s) & 0x7f) << 8)	/* R_CPUTAG1CR[14:8] */

struct rtl9602c_eth {
	void __iomem	*base;
	void __iomem	*sw;	/* switch core */
	struct net_device *ndev;
	struct device	*dev;

	struct rx_desc	*rx_ring;
	dma_addr_t	rx_ring_dma;
	struct sk_buff	*rx_skb[RX_RING_SIZE];
	dma_addr_t	rx_buf_dma[RX_RING_SIZE];
	unsigned int	rx_head;

	struct tx_desc	*tx_ring;
	dma_addr_t	tx_ring_dma;
	struct sk_buff	*tx_skb[TX_RING_SIZE];
	dma_addr_t	tx_buf_dma[TX_RING_SIZE];
	unsigned int	tx_buf_len[TX_RING_SIZE];
	unsigned int	tx_head, tx_dirty;

	struct timer_list poll_timer;
	/* Host uplink port, learned from the RX descriptor src_port_num. All RX
	 * arrives on the board's single connected LAN port, so this resolves to the
	 * physical switch port the host is on — we then steer CPU->LAN TX there
	 * regardless of the (ambiguous) static port numbering. 0xff = not yet seen. */
	unsigned int	host_port;

	/* Bootloader GMAC0 control snapshot (inherited). */
	u32		ub_tcr, ub_rcr, ub_config, ub_cputagcr, ub_cputag1cr;
	u32		ub_iocmd, ub_iocmd1;

	/* RX datapath debug counters (see /proc/rtl9602c_diag). */
	u32		dbg_poll;	/* poll-timer ticks */
	u32		dbg_filled;	/* RX descriptors HW handed back (D_OWN cleared) */
	u32		dbg_good;	/* frames pushed up the stack */
	u32		dbg_err;	/* RX descriptors with error/oversize */
	u32		dbg_rxlen;	/* raw length of the last captured RX frame */
	u8		dbg_rxbuf[48];	/* first bytes of the last RX frame (pre-pull) */

	/* OMCI (OMCC stream) trap, armed by the GPON driver once it installs the
	 * OMCC GEM datapath (rtl9602c_eth_set_omci_sid). */
	bool		omci_trap_on;
	u32		dbg_omci_rx;	/* DS OMCI frames trapped to the CPU */
	u32		dbg_omci_rxlen;	/* length of the last OMCI frame */
	u8		dbg_omci_rxbuf[48];	/* the last DS OMCI baseline message */
};

static struct rtl9602c_eth *g_ep;	/* single-instance, for /proc diag */

static inline u32 ep_rd(struct rtl9602c_eth *ep, u32 r) { return ioread32(ep->base + r); }
static inline void ep_wr(struct rtl9602c_eth *ep, u32 r, u32 v) { iowrite32(v, ep->base + r); }

/*
 * Arm the GMAC OMCI trap so DS frames on stream-id `sid` (the OMCC) are delivered
 * to the CPU netdev instead of switched. Called by the GPON driver AFTER it has
 * installed the OMCC GEM datapath (NOT at NIC init — arming the trap before the
 * datapath exists was an earlier regression). CPUTAG1CR[14:8] = OMCI SID.
 */
void rtl9602c_eth_set_omci_sid(unsigned int sid)
{
	struct rtl9602c_eth *ep = g_ep;
	u32 v, ct;

	if (!ep)
		return;
	v = ep_rd(ep, R_CPUTAG1CR);
	/* OMCI stream-id in [14:8]; SPA_PON[2:0]=2 tags PON-sourced frames with
	 * source-port 2 (operational CPUTAG1CR = 0x4002). Without SPA_PON the
	 * de-encapsulated PON frame's source-port is mis-set and the OMCI RX
	 * classification misses it. */
	v = (v & ~((0x7fu << 8) | 0x7u)) | CPUTAG1_OMCI_SID(sid) | 2u;
	ep_wr(ep, R_CPUTAG1CR, v);
	/* CT_SWITCH (CPUTAGCR[21:18]) = the GMAC NIC RX CPU-tag SOURCE selector. The
	 * generic NIC init binds it to 8 (the LAN switch fabric); the GPON path must
	 * bind it to 7 (the PON-IP internal source) so the NIC ACCEPTS the PON-IP's
	 * direct OMCI DMA. Without this the NIC ignores the PON-sourced frame, ring 0
	 * stays empty (filled=0), and the frame backs up in PON-IP SRAM -> US stall ->
	 * deactivate. At O5 the controller requires CPUTAGCR = 0x901eff04 (CT_SWITCH=7).
	 * RMW preserves the NIC-init bits (CTEN_RX, CT_RSIZE, CTPM/CTPV). */
	ct = ep_rd(ep, R_CPUTAGCR);
	ct = (ct & ~((0xfu << 18) | (1u << 27))) | (0x7u << 18);	/* CT_SWITCH=7, clear bit27 -> 0x901eff04 */
	ep_wr(ep, R_CPUTAGCR, ct);
	/* CONFIG_REG(0x4c) bits 22,23 = config_rx_sideband: enable the GMAC NIC RX
	 * side-channel that carries the CPU-tag/source metadata the PON-IP attaches when
	 * it DMAs the OMCI frame. The controller requires this on; the base init leaves
	 * 0x4c=0x21000000 (bits 22/23 clear), so a PON-sourced internally-tagged frame is
	 * not accepted. */
	{
		u32 cfg = ep_rd(ep, R_CONFIG);
		cfg |= (3u << 22);
		ep_wr(ep, R_CONFIG, cfg);
	}
	/* Route EVERY RX class to ring 0 (the only ring this driver allocates and
	 * drains). There are SEVEN routing tables (RRING_ROUTING1..7 @ 0x1370..0x1388,
	 * stride 4) selected by source/priority; OMCI's class may use one other than
	 * table 1, and an un-zeroed table sends it to an un-drained ring (the frame
	 * sticks in PON-IP -> US stall -> deactivate). Zero all 7 -> everything to ring 0
	 * (routing value 0 = ring 0, the table the LAN low-priority traffic already uses). */
	{
		unsigned int r;
		for (r = 0; r < 7; r++)
			ep_wr(ep, R_RRING_ROUTING1 + r * 4, 0);
	}
	ep->omci_trap_on = true;
	netdev_info(ep->ndev, "OMCI trap armed: SID %u CPUTAG1CR=0x%08x CPUTAGCR=0x%08x (CT_SWITCH=7)\n",
		    sid, v, ct);
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_sid);

/*
 * Minimal switch bring-up: permit ingress from every port and flood
 * broadcast / unknown unicast+multicast to every port (incl. the CPU port).
 * All-ports masks avoid depending on the exact CPU port number. Idempotent
 * (OR-in). Without this, the bootloader-left switch state only forwarded its
 * own TFTP unicast flow and ingress never reached the CPU GMAC.
 */
static void rtl9602c_sw_min_init(struct rtl9602c_eth *ep)
{
	if (!ep->sw)
		return;
	/* Force CPU port (3) + both LAN ports (0=FE,1=GE) link-up (the switch will
	 * not egress to a port it thinks is link-down), permit all-port ingress at
	 * the CORRECT 9602C address (0x1C088), and open port isolation. */
	/* Force NO port. The bootloader's WORKING config (its TFTP egresses the GE
	 * host port) runs with FORCE_P_ABLTY=0 and ABLTY_FORCE_MODE=0 for EVERY port
	 * including the CPU port (P3), with P_ABLTY status=0x60 (auto-linked) on all.
	 * Force-up of the CPU port to a fixed 1000M overrides that auto-linked state
	 * and kills CPU->LAN egress (MIB: all LAN-port TX=0). Leave every port at its
	 * auto-negotiated reset state, as the bootloader does. */
	/* L2_SRC_PORT_PERMIT (0x1C088), INVERTED polarity: EN=1 PERMITS a frame to
	 * egress its OWN ingress port (reflection); EN=0 (reset default) SUPPRESSES it.
	 * Writing 0xffffffff (permit=1) was the ROOT CAUSE of the broadcast loop — the
	 * board reflected the host's broadcasts back out the GbE ("own address as
	 * source"), disrupting the LAN. Write 0 to suppress source-port egress.
	 * Cross-port forwarding (host port1 <-> CPU port3) is unaffected (different
	 * ports). PISO 0x27000 is an 11-bit positive egress matrix (reset 0x3FFFFF =
	 * forward to all); leave it at reset — do NOT write 0xffffffff (reserved bits). */
	iowrite32(0, ep->sw + SW_SRC_PORT_PERMIT);
	iowrite32(ioread32(ep->sw + SW_LUT_BC_FLOOD) | SW_PORTS_ALL,
		  ep->sw + SW_LUT_BC_FLOOD);
	iowrite32(ioread32(ep->sw + SW_LUT_UNKN_MC_FLOOD) | SW_PORTS_ALL,
		  ep->sw + SW_LUT_UNKN_MC_FLOOD);
	iowrite32(ioread32(ep->sw + SW_LUT_UNKN_UC_FLOOD) | SW_PORTS_ALL,
		  ep->sw + SW_LUT_UNKN_UC_FLOOD);
	/* Unknown-unicast that misses the L2 lookup (e.g. the host's NDP/ARP
	 * reply to the not-yet-learned CPU MAC) must reach the CPU. Flooding via
	 * uc_flood alone proved ineffective for unicast DLF on this switch, so set
	 * the per-port destination-lookup-failure action to trap-to-CPU. Each port
	 * is a 16-bit field at 2-byte stride with ACT in bits[1:0]; one 32-bit
	 * write covers two ports. */
	iowrite32((SW_DLF_ACT_TRAP2CPU << 16) | SW_DLF_ACT_TRAP2CPU,
		  ep->sw + SW_LUT_UNKN_UC_DA_CTRL);		/* ports 0,1 */
	iowrite32((SW_DLF_ACT_TRAP2CPU << 16) | SW_DLF_ACT_TRAP2CPU,
		  ep->sw + SW_LUT_UNKN_UC_DA_CTRL + 4);		/* ports 2,3 */
	/* NOTE: 0x27000 (PISO) is a 5-bit isolation-vector INDEX per port, NOT a
	 * direct portmask — writing 0xffffffff selected index 0x1f and likely blocked
	 * CPU->LAN forwarding. The bootloader leaves it at default (TX works), so we
	 * do too. */
	/* CPU-tag awareness (MAC_CPU_TAG_CTRL @ 0x23030 TAG_AWARE bit 9): when set, a
	 * CPU-tag is MANDATORY for a CPU-port frame to traverse to a LAN port (a clean
	 * frame with cputag=0 does NOT egress -- observed by capture), so the switch
	 * must PARSE it and strip+direct on physical egress. That pairs with the GMAC
	 * CPUtagCR (0x9022FF04, set in open) so the GMAC's on-wire tag encoding matches
	 * this parser. The operational value of this register is 0x300 (TAG_AWARE bit9
	 * + TRAP_TAGET_INSERT_EN bit8); bit8 ("TARGET insert") also gates the
	 * directed/targeted cpu-tag egress, not just RX-trap tagging.
	 *
	 * OPEN ISSUE with TAG_AWARE on: CPU->LAN TX egress does not work — the GMAC
	 * inserts a cpu-tag with a ZERO egress-portmask regardless of opts2.tx_portmask
	 * (observed by capture), so the switch's directed egress drops it; the
	 * portmask-population path is an undocumented GMAC<->switch internal mechanism.
	 * RX + switch->LAN work. The MIB also shows tagged CPU frames register ZERO at
	 * port-3 RX — the cpu-tag parser drops them at ingress.
	 *
	 * So write TAG_AWARE OFF (0x23030 = 0): a PLAIN CPU-port frame is treated
	 * normally and FLOODED to the LAN ports, the same path that already reflects
	 * LAN-ingress frames — the RX-good baseline. */
	iowrite32(0, ep->sw + SW_MAC_CPU_TAG_CTRL);
	/* CPU-tag aux registers: the operational values are 0x230F4=0x00f000ea,
	 * 0x230F8=0x00400034. Part of the cpu-tag forwarding the switch expects. */
	iowrite32(0x00f000ea, ep->sw + 0x230F4);
	iowrite32(0x00400034, ep->sw + 0x230F8);
	/* VLAN forwarding domain. CPU->LAN egress is VLAN-DIRECTED, not flooded: the
	 * operational config runs with VLAN filtering ON (VLAN_CTRL=0x19) + per-port
	 * service VLANs, and every flat / no-VLAN test gave 0 egress. Create a default
	 * VLAN (VID 1) whose member + untag masks are ALL ports (CPU + LAN), point
	 * every port's PVID at it, accept all frame types, enable per-port ingress
	 * filter, then enable the VLAN function. A parsed cpu-tag's directed egress
	 * then lands in a VLAN the target LAN port belongs to instead of being
	 * filtered/dropped. */
	{
		int p, to;
		/* 4k-table entry: untag[7:4]=0xf | mbr[3:0]=0xf (all four ports) */
		iowrite32((0xf << 4) | 0xf, ep->sw + SW_TBL_WRDATA);
		iowrite32(SW_TBL_VLAN_WR(SW_DEFAULT_VID), ep->sw + SW_TBL_CTRL);
		for (to = 0; to < 1000 &&
			    (ioread32(ep->sw + SW_TBL_STS) & SW_TBL_BUSY); to++)
			udelay(1);
		iowrite32(0, ep->sw + SW_VLAN_ACCEPT);	/* accept all frame types */
		for (p = 0; p < 4; p++)
			iowrite32((ioread32(ep->sw + SW_VLAN_PB_VID + p * 4) & ~0xfffu) |
					  SW_DEFAULT_VID,
				  ep->sw + SW_VLAN_PB_VID + p * 4);
		iowrite32(0xf, ep->sw + SW_VLAN_INGRESS);	/* ingress filter, ports 0-3 */
		iowrite32(SW_VLAN_CTRL_VAL, ep->sw + SW_VLAN_CTRL); /* enable VLAN function */
	}
	/*
	 * Force the CPU port (3) link UP. CRITICAL: the bootloader leaves
	 * FORCE_P_ABLTY[3] with the LINK bit (bit4) CLEAR (value 0x186) while
	 * ABLTY_FORCE_MODE[3] forces ALL ability bits (0xfff) — so the switch treats
	 * the CPU port as link-DOWN and refuses to forward any frame to it. Result:
	 * the GMAC RX DMA never receives (CPU RX = 0) and CPU-originated TX never
	 * egresses. Writing FORCE_P_ABLTY[3] = 0x16 (speed 1000M | full-duplex | LINK)
	 * immediately starts RX. The LAN jack ports stay auto-negotiated (real PHYs);
	 * only the internal MAC<->MAC CPU port must be force-linked.
	 */
	iowrite32(SW_ABLTY_1G_FD_UP, ep->sw + SW_FORCE_P_ABLTY(SW_CPU_PORT));
	iowrite32(0xfff, ep->sw + SW_ABLTY_FORCE_MODE(SW_CPU_PORT));

	/*
	 * Force the PON port (2) link UP for the SAME reason as the CPU port: the
	 * PON-IP/PONNIC connects to switch port 2 over an internal MAC<->MAC GMII with
	 * no auto-negotiating PHY, so the bootloader leaves FORCE_P_ABLTY[2] link-DOWN
	 * and the switch refuses to forward de-encapsulated downstream frames (GEM data
	 * + OMCI,
	 * stream-id 64) from the PON port into the fabric — they never reach the CPU
	 * port and the GMAC RX ring stays empty (filled=0) despite the PON-IP DS
	 * datapath being fully up. Forcing 1000M/FD/LINK opens the PON->CPU path.
	 */
	iowrite32(SW_ABLTY_1G_FD_UP, ep->sw + SW_FORCE_P_ABLTY(RTL9602C_PON_PORT));
	iowrite32(0xfff, ep->sw + SW_ABLTY_FORCE_MODE(RTL9602C_PON_PORT));
}

static void rtl9602c_eth_get_hwaddr(struct rtl9602c_eth *ep, u8 *mac)
{
	u32 lo = ep_rd(ep, R_IDR0);
	u32 hi = ep_rd(ep, R_IDR4);

	mac[0] = lo >> 24; mac[1] = lo >> 16; mac[2] = lo >> 8; mac[3] = lo;
	mac[4] = hi >> 24; mac[5] = hi >> 16;
}

static void rtl9602c_eth_set_hwaddr(struct rtl9602c_eth *ep, const u8 *mac)
{
	ep_wr(ep, R_IDR0, ((u32)mac[0] << 24) | ((u32)mac[1] << 16) |
			  ((u32)mac[2] << 8) | mac[3]);
	ep_wr(ep, R_IDR4, ((u32)mac[4] << 24) | ((u32)mac[5] << 16));
}

/*
 * Program the station address into the hardware (IDR) on a MAC change, not just
 * ndev->dev_addr: the MyPhys RX filter matches IDR, so without this, unicast to
 * a freshly-set per-board MAC is dropped by hardware. gpon_provision applies the
 * per-board MAC this way at boot.
 */
static int rtl9602c_eth_set_mac_address(struct net_device *ndev, void *p)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, p);

	if (ret)
		return ret;
	rtl9602c_eth_set_hwaddr(ep, ndev->dev_addr);
	return 0;
}

/* Give RX descriptor @idx a fresh buffer and hand it to HW (own=1). */
static int rtl9602c_eth_refill(struct rtl9602c_eth *ep, unsigned int idx)
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
	ep->rx_ring[idx].addr = da | DMA_BUS_WINDOW;	/* match RxFDP bus window */
	ep->rx_ring[idx].opts2 = 0;
	ep->rx_ring[idx].opts3 = 0;
	opts1 = D_OWN | RX_BUF_SIZE;
	if (idx == RX_RING_SIZE - 1)
		opts1 |= D_EOR;
	ep->rx_ring[idx].opts1 = opts1;
	return 0;
}

static void rtl9602c_eth_rx(struct rtl9602c_eth *ep)
{
	struct net_device *ndev = ep->ndev;

	while (1) {
		unsigned int i = ep->rx_head;
		u32 opts1 = ep->rx_ring[i].opts1;
		struct sk_buff *skb;
		u32 len;

		if (opts1 & D_OWN)		/* still HW-owned: nothing more */
			break;
		ep->dbg_filled++;		/* HW handed this descriptor back */

		len = (opts1 & RXD_LEN_MASK);
		skb = ep->rx_skb[i];
		dma_unmap_single(ep->dev, ep->rx_buf_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		if (ep->omci_trap_on &&
		    ((ep->rx_ring[i].opts2 >> 21) & 0xff) == RTL9602C_OMCI_REASON &&
		    ((ep->rx_ring[i].opts3 >> 16) & 0xf) == RTL9602C_PON_PORT) {
			/* DS OMCI GEM frame trapped to the CPU on the OMCC stream.
			 * Baseline OMCI is 48 bytes (< ETH_ZLEN), so it MUST be caught
			 * here, ahead of the runt filter below. M1: count + capture for
			 * /proc/rtl9602c_diag; M2 will deliver it to the OMCI responder. */
			ep->dbg_omci_rx++;
			ep->dbg_omci_rxlen = len;
			memcpy(ep->dbg_omci_rxbuf, skb->data,
			       min_t(unsigned int, len, sizeof(ep->dbg_omci_rxbuf)));
			dev_kfree_skb_any(skb);
		} else if ((opts1 & (RXD_CRCERR | RXD_RCDF)) ||
		    len < ETH_ZLEN + RX_CPU_PREFIX || len > RX_BUF_SIZE) {
			ndev->stats.rx_errors++;
			ep->dbg_err++;
			dev_kfree_skb_any(skb);
		} else {
			ep->dbg_good++;
			/* Do NOT software-strip a 4-byte FCS here: on this GMAC the
			 * RX descriptor length already excludes most of the FCS, so
			 * subtracting ETH_FCS_LEN on top of the 2-byte CPU prefix
			 * left the IP packet 2 bytes short (truncated -> dropped,
			 * Icmp InEcho=0). The stack uses the L3 length, so any
			 * trailing FCS/pad bytes are ignored harmlessly. */
			skb_put(skb, len);
			/* Capture the raw frame (pre-pull) for /proc diag. */
			ep->dbg_rxlen = len;
			memcpy(ep->dbg_rxbuf, skb->data,
			       min_t(unsigned int, len, sizeof(ep->dbg_rxbuf)));
			{
				/* opts3 src_port_num [19:16] = ingress port. Learn it
				 * as the host uplink so TX egresses to the right port. */
				unsigned int sp = (ep->rx_ring[i].opts3 >> 16) & 0xf;

				ep->host_port = sp;
			}
			/*
			 * The switch CPU port prepends a 2-byte offset word
			 * ahead of the Ethernet header on every frame delivered
			 * to the CPU (the GMAC CPU-port RX framing). Strip it so
			 * eth_type_trans() parses the real dst-MAC / ethertype;
			 * without this every RX frame is malformed and the stack
			 * silently drops it (no ARP reply, no neigh resolution).
			 */
			skb_pull(skb, RX_CPU_PREFIX);
			skb->protocol = eth_type_trans(skb, ndev);
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
			netif_rx(skb);
		}
		/* hand the slot back to HW with a fresh buffer */
		if (rtl9602c_eth_refill(ep, i)) {
			ndev->stats.rx_dropped++;
			/* leave CPU-owned; will retry next poll */
			break;
		}
		ep->rx_head = (i + 1) % RX_RING_SIZE;
	}
}

static void rtl9602c_eth_tx_reclaim(struct rtl9602c_eth *ep)
{
	while (ep->tx_dirty != ep->tx_head) {
		unsigned int i = ep->tx_dirty % TX_RING_SIZE;

		if (ep->tx_ring[i].opts1 & D_OWN)	/* not sent yet */
			break;
		dma_unmap_single(ep->dev, ep->tx_buf_dma[i], ep->tx_buf_len[i],
				 DMA_TO_DEVICE);
		dev_consume_skb_any(ep->tx_skb[i]);
		ep->tx_skb[i] = NULL;
		ep->tx_dirty++;
	}
	if (netif_queue_stopped(ep->ndev) &&
	    (ep->tx_head - ep->tx_dirty) < TX_RING_SIZE - 1)
		netif_wake_queue(ep->ndev);
}

static void rtl9602c_eth_poll(struct timer_list *t)
{
	struct rtl9602c_eth *ep = timer_container_of(ep, t, poll_timer);

	ep->dbg_poll++;
	rtl9602c_eth_rx(ep);
	rtl9602c_eth_tx_reclaim(ep);
	mod_timer(&ep->poll_timer, jiffies + POLL_INTERVAL);
}

static int rtl9602c_eth_alloc_rings(struct rtl9602c_eth *ep)
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
	ep->host_port = 0xff;		/* uplink port unknown until first RX */
	for (i = 0; i < TX_RING_SIZE; i++) {
		ep->tx_ring[i].opts1 = (i == TX_RING_SIZE - 1) ? D_EOR : 0;
		ep->tx_skb[i] = NULL;
	}
	for (i = 0; i < RX_RING_SIZE; i++) {
		if (rtl9602c_eth_refill(ep, i))
			return -ENOMEM;
	}
	return 0;
}

static void rtl9602c_eth_free_rings(struct rtl9602c_eth *ep)
{
	unsigned int i;

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (ep->rx_skb[i]) {
			dma_unmap_single(ep->dev, ep->rx_buf_dma[i],
					 RX_BUF_SIZE, DMA_FROM_DEVICE);
			dev_kfree_skb_any(ep->rx_skb[i]);
			ep->rx_skb[i] = NULL;
		}
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		if (ep->tx_skb[i]) {
			dma_unmap_single(ep->dev, ep->tx_buf_dma[i],
					 ep->tx_buf_len[i], DMA_TO_DEVICE);
			dev_kfree_skb_any(ep->tx_skb[i]);
			ep->tx_skb[i] = NULL;
		}
	}
	if (ep->rx_ring)
		dma_free_coherent(ep->dev, RX_RING_SIZE * sizeof(struct rx_desc),
				  ep->rx_ring, ep->rx_ring_dma);
	if (ep->tx_ring)
		dma_free_coherent(ep->dev, TX_RING_SIZE * sizeof(struct tx_desc),
				  ep->tx_ring, ep->tx_ring_dma);
	ep->rx_ring = NULL;
	ep->tx_ring = NULL;
}

static int rtl9602c_eth_open(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	u32 desnum;
	int ret;

	ret = rtl9602c_eth_alloc_rings(ep);
	if (ret) {
		rtl9602c_eth_free_rings(ep);
		return ret;
	}

	/* Program the ring pointers. The bootloader ORs 0x20000000 into the GMAC DMA
	 * ring base + every TX buffer address (TxFDP/RxFDP |= 0x20000000, desc.addr
	 * |= 0x20000000) — the SoC routes the NIC DMA master to DRAM through this bus
	 * window. RX happened to work with the plain (window-0) address, but TX egress
	 * never did; replicate the bus window on TX (the last unreplicated detail of
	 * the established CPU->LAN TX path). */
	ep_wr(ep, R_TxFDP1, ep->tx_ring_dma | DMA_BUS_WINDOW);
	ep_wr(ep, R_TxCDO1, 0);
	ep_wr(ep, R_RxFDP, ep->rx_ring_dma | DMA_BUS_WINDOW);
	/* RX ring0 size + flow-control thresholds (GMAC field packing). */
	desnum = ((RX_RING_SIZE - 1) & 0xff) << 24 | (TH_ON_VAL & 0xff) << 16 |
		 (TH_OFF_VAL & 0xff) << 8 | (((RX_RING_SIZE - 1) >> 8) & 0xf) << 4;
	ep_wr(ep, R_RxDesNum, desnum);
	ep_wr(ep, R_RxCDO, ((RX_RING_SIZE - 1) & 0xff) << 8 |
			   (((RX_RING_SIZE - 1) >> 8) & 0xf) << 4);

	/*
	 * Program the GMAC control regs. TCR/RCR/CONFIG use known-good bring-up
	 * values (RCR=0x0F accepts bcast/mcast/myphys/allphys, CONFIG RFF-2k +
	 * rx-mring-int-split). IO_CMD is the full CMD_CONFIG that enables RX *and*
	 * TX DMA — the bootloader leaves RX off after its polled TFTP, so re-asserting
	 * the inherited IO_CMD (0x400f3330) never started the RX engine. IO_CMD is
	 * written last (it is the enable/kick).
	 *
	 * Apply the active GMAC init the bootloader uses during a net op (its TFTP
	 * does CPU->LAN TX over the GbE), which forwards CPU<->host both ways: the
	 * alternative recipe (0xd15ff130 / RCR 0x0F / TCR 0x0C01 / CONFIG 0x20c10000)
	 * enabled RX but not TX egress. The control words read at the idle bootloader
	 * prompt are NOT usable (the GMAC is torn down there: CPUtagCR reads reset
	 * 0x015c0000); the established active values are TCR 0x0c00, RCR 0x0e,
	 * CONFIG 0x20000000, IO_CMD 0x400f3330, CPUtagCR 0x981aff04. Write order
	 * matters; IO_CMD last (enable).
	 *
	 * NOTE on TCR: TCR=0x0C01 with auto-padding DISABLED (bit0=1, TX pad OFF) can
	 * corrupt the GMAC-inserted cpu-tag on short frames; the established active
	 * value keeps bit0=0 (TX pad ON), so use 0x0C00.
	 */
	ep_wr(ep, R_RCR, 0x0000000E);
	ep_wr(ep, R_TCR, 0x00000C00);		/* TX pad ON (bit0=0), NOT 0x0C01 */
	ep_wr(ep, R_CONFIG, 0x20000000);	/* CONFIG (= the prior working-RX value) */
	iowrite8(0x0A, ep->base + 0x3B);	/* CMD = RxChkSum|RxJumboSupport (keep working-RX baseline) */
	/* (RX-ring-size bytes 0x1430/0x1432/0x13f6 select a 16-entry ring; our
	 * 64-entry ring is sized by R_RxDesNum/R_RxCDO above — don't clobber.) */
	/* CPU-tag control register. cputag_info = CTEN_RX(1<<31) | 2<<CT_RSIZE_L(16) |
	 * 2<<CT_TSIZE(27) | (8<<18) | CTPM(0xff<<8) | CTPV(0x04) = 0x9022FF04. The
	 * established active value 0x981AFF04 shares the 0xFF04 on-wire tag format;
	 * 0x9022FF04 differs by bit 21 + cpu-tag ring sizes that pair with the switch
	 * TAG_AWARE parser. CPUtag1CR = CT1_SID (64<<8). */
	ep_wr(ep, R_CPUTAGCR, 0x981AFF04);	/* active value (not the idle-snapshot 0x9022FF04) */
	ep_wr(ep, R_CPUTAG1CR, 0x00000000);	/* active value is 0 (not 0x4000) */
	/* MSR (0x58): mask ONLY the top byte to 0x3f, leaving 0x10638000. Do NOT set
	 * FORCE_TX — a prior |BIT(7)|BIT(5) set MSR[31]/[29] (=0xb0638000), NOT
	 * FORCE_TX, corrupting it. */
	iowrite8(ioread8(ep->base + 0x58) & 0x3f, ep->base + 0x58);
	iowrite32(0xffffffff, ep->base + 0x08);	/* MAR0: accept-all-multicast */
	iowrite32(0xffffffff, ep->base + 0x0C);	/* MAR4 */
	ep_wr(ep, R_IO_CMD1, 0x30010000);	/* active value */
	ep_wr(ep, R_IO_CMD, 0x400F3330);	/* full CMD_CONFIG, RX+TX DMA enable (last) */

	rtl9602c_sw_min_init(ep);	/* flood ingress to the CPU port */

	timer_setup(&ep->poll_timer, rtl9602c_eth_poll, 0);
	mod_timer(&ep->poll_timer, jiffies + POLL_INTERVAL);

	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int rtl9602c_eth_stop(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	timer_delete_sync(&ep->poll_timer);
	ep_wr(ep, R_IO_CMD, 0);		/* stop DMA */
	rtl9602c_eth_free_rings(ep);
	return 0;
}

/* CPU-directed TX (GMAC tx_info): opts2 cputag[31] | tx_portmask[26:16];
 * opts3 keep[23] | dislrn[21] | l34_keep[17]. Directs CPU-originated frames out
 * the physical LAN ports (keep/l34_keep stop the switch filtering them). */
#define TXD_CPUTAG	BIT(31)
#define TXD_PORTMASK(m)	(((m) & 0x7ff) << 16)
#define TXD_KEEP	BIT(23)
#define TXD_DISLRN	BIT(21)
#define TXD_L34_KEEP	BIT(17)
/* GMAC tx_portmask: user/LAN ports are 0..6; CPU ports are 7,9,10. 0x2f =
 * ports 0,1,2,3,5 = an "all user ports except CPU" broadcast-fallback mask.
 * Egressing to all user ports is harmless for ports without a linked PHY. */
#define TX_LAN_PORTS	0x2f
/* Software DSA-style cpu-tag (mainline net/dsa/tag_rtl8_4.c "rtl8_4" 0x8899
 * format, 8 bytes). The GMAC's HARDWARE cpu-tag insertion emits a ZERO egress
 * portmask on this 9602C silicon (observed: tag word[3]=0 regardless of
 * opts2.tx_portmask), so we BUILD the tag in software and send a plain frame
 * (opts2.cputag=0). The switch's TAG_AWARE parser then reads OUR portmask.
 * Layout: word0=0x8899 word1=proto(0x04)|reason(0) word2=LEARN_DIS
 * word3=forwarding port mask (RX field, GENMASK 10:0). */
#define RTL8_4_TAG_LEN	8
#define SW_TAG_LAN_MASK	0x7	/* forwarding mask: LAN ports 0,1,2 (CPU port = 3) */

static netdev_tx_t rtl9602c_eth_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	unsigned int i = ep->tx_head % TX_RING_SIZE;
	unsigned int len = skb->len;
	dma_addr_t da;
	u32 opts1;

	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
	if (len < ETH_ZLEN) {
		if (skb_padto(skb, ETH_ZLEN))
			return NETDEV_TX_OK;
		len = ETH_ZLEN;
	}
#if TX_CPUTAG
	/* Prepend the software cpu-tag after DA+SA (the hardware portmask insertion
	 * is broken on this silicon — see RTL8_4 defines). */
	if (skb_cow_head(skb, RTL8_4_TAG_LEN)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	skb_push(skb, RTL8_4_TAG_LEN);
	memmove(skb->data, skb->data + RTL8_4_TAG_LEN, 2 * ETH_ALEN);
	{
		__be16 *t = (__be16 *)(skb->data + 2 * ETH_ALEN);
		t[0] = htons(0x8899);		/* Realtek EtherType */
		t[1] = htons(0x0400);		/* protocol 0x04 (rtl8_4), reason 0 */
		t[2] = htons(0x0020);	/* LEARN_DIS (rtl8_4 word2) */
		t[3] = htons(SW_TAG_LAN_MASK);	/* CPU->switch forwarding port mask */
	}
#endif
	len = skb->len;
	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	ep->tx_skb[i] = skb;
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;

	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;	/* TX desc.addr bus window |= 0x20000000 */
	/* Program the descriptor cpu-tag so the GMAC inserts a cpu-tag carrying a
	 * NON-ZERO egress portmask; the switch (TAG_AWARE) parses it and directs
	 * the frame to the LAN port(s). tx_portmask 0 was the bug: the switch then
	 * does an empty L2 DA lookup and drops the frame ("TX never egresses"). */
#if TX_CPUTAG
	/* The software in-band 0x8899 tag (built above) already carries the egress
	 * portmask; the GMAC must NOT also insert its own (broken-portmask) cpu-tag,
	 * so opts2=0. This software-tag path is the one that put frames on the wire
	 * at the host (observed: clean stripped IPv6 frames received). */
	ep->tx_ring[i].opts2 = 0;
	ep->tx_ring[i].opts3 = 0;
#else
	/* PLAIN frame (cputag/TAG_AWARE trio gave 0 egress even with the correct
	 * GMAC CPUtagCR — reverted). The bootloader's own TX path writes NO cputag
	 * either. */
	ep->tx_ring[i].opts2 = 0;
	ep->tx_ring[i].opts3 = 0;
#endif
	ep->tx_ring[i].opts4 = 0;
	opts1 = D_OWN | D_FS | D_LS | D_TXCRC | (len & TXD_LEN_MASK);
	if (i == TX_RING_SIZE - 1)
		opts1 |= D_EOR;
	wmb();				/* descriptor body before ownership */
	ep->tx_ring[i].opts1 = opts1;
	wmb();

	ep->tx_head++;
	ep_wr(ep, R_IO_CMD, ep_rd(ep, R_IO_CMD) | BIT(0));	/* kick ring 0 */

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	return NETDEV_TX_OK;
}

/* Honour promiscuous/all-multi (the bridge enslaving eth0 requests promisc).
 * Without AcceptAllPhys the GMAC drops unicast frames whose DA != our station
 * MAC — i.e. exactly the LAN-client frames a router/bridge must receive and
 * forward. RCR bit0 = AcceptAllPhys. */
static void rtl9602c_eth_set_rx_mode(struct net_device *ndev)
{
	struct rtl9602c_eth *ep = netdev_priv(ndev);
	u32 rcr = ep_rd(ep, R_RCR);

	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))
		rcr |= BIT(0);
	else
		rcr &= ~BIT(0);
	ep_wr(ep, R_RCR, rcr);
}

static const struct net_device_ops rtl9602c_eth_netdev_ops = {
	.ndo_open		= rtl9602c_eth_open,
	.ndo_stop		= rtl9602c_eth_stop,
	.ndo_start_xmit		= rtl9602c_eth_xmit,
	.ndo_set_rx_mode	= rtl9602c_eth_set_rx_mode,
	.ndo_set_mac_address	= rtl9602c_eth_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
};

/* Live datapath diagnostic. Read /proc/rtl9602c_diag to dump the GMAC DMA/link
 * state, RX-ring ownership, and switch forwarding regs — to localise where a
 * frame is lost on the CPU<->LAN path. */
static int rtl9602c_diag_show(struct seq_file *m, void *v)
{
	struct rtl9602c_eth *ep = g_ep;
	unsigned int i, own = 0, hwfilled = 0;

	if (!ep) { seq_puts(m, "no device\n"); return 0; }

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (ep->rx_ring[i].opts1 & D_OWN)
			own++;
		else
			hwfilled++;
	}
	seq_printf(m, "poll=%u filled=%u good=%u err=%u rx_head=%u\n",
		   ep->dbg_poll, ep->dbg_filled, ep->dbg_good, ep->dbg_err,
		   ep->rx_head);
	seq_printf(m, "last RX frame (pre-pull, len=%u): %*ph\n",
		   ep->dbg_rxlen, (int)sizeof(ep->dbg_rxbuf), ep->dbg_rxbuf);
	seq_printf(m, "omci: trap=%u rx=%u lastlen=%u msg=%*ph\n",
		   ep->omci_trap_on, ep->dbg_omci_rx, ep->dbg_omci_rxlen,
		   (int)sizeof(ep->dbg_omci_rxbuf), ep->dbg_omci_rxbuf);
	seq_printf(m, "rxring: HW-owned(D_OWN=1)=%u  CPU-owned(filled)=%u\n",
		   own, hwfilled);
	seq_printf(m, "GMAC IO_CMD=%08x IO_CMD1=%08x MSR(0x58)=%08x\n",
		   ep_rd(ep, R_IO_CMD), ep_rd(ep, R_IO_CMD1),
		   ioread32(ep->base + 0x58));
	seq_printf(m, "GMAC RCR=%08x TCR=%08x CONFIG=%08x CPUTAGCR=%08x\n",
		   ep_rd(ep, R_RCR), ep_rd(ep, R_TCR), ep_rd(ep, R_CONFIG),
		   ep_rd(ep, R_CPUTAGCR));
	seq_printf(m, "GMAC RxFDP=%08x RxCDO=%08x RxDesNum=%08x ringDMA=%08x\n",
		   ep_rd(ep, R_RxFDP), ep_rd(ep, R_RxCDO), ep_rd(ep, R_RxDesNum),
		   (u32)ep->rx_ring_dma);
	/* NIC interrupt status: per-ring RDU (Receive-Descriptor-Unavailable) bits show
	 * a frame ARRIVED at the NIC on a ring with no posted descriptor. ISR(0x3c):
	 * RDU=bit5(ring0) RDU2=bit11(r1) RDU3=bit12(r2) RDU4=bit13(r3) RDU5=bit14(r4)
	 * RDU6=bit15(r5). If RDU2-6 set => OMCI is reaching the NIC on rings 1-5 that I
	 * don't set up (frame dropped). RxCDO per ring shows HW fetch progress. */
	seq_printf(m, "NIC ISR(0x3c)=%08x ISR1(0xd8)=%08x  perRingRxCDO r0=%04x r1=%04x r2=%04x r3=%04x r4=%04x r5=%04x\n",
		   ep_rd(ep, 0x3c), ep_rd(ep, 0xd8),
		   ep_rd(ep, 0x13f4) & 0xffff, ep_rd(ep, 0x1394) & 0xffff,
		   ep_rd(ep, 0x13a4) & 0xffff, ep_rd(ep, 0x13b4) & 0xffff,
		   ep_rd(ep, 0x13c4) & 0xffff, ep_rd(ep, 0x13d4) & 0xffff);
	if (ep->sw) {
		seq_printf(m, "SW permit(1c088)=%08x flood bc/mc/uc=%08x/%08x/%08x\n",
			   ioread32(ep->sw + SW_SRC_PORT_PERMIT),
			   ioread32(ep->sw + SW_LUT_BC_FLOOD),
			   ioread32(ep->sw + SW_LUT_UNKN_MC_FLOOD),
			   ioread32(ep->sw + SW_LUT_UNKN_UC_FLOOD));
		seq_printf(m, "SW vlan_ctrl(13008)=%08x cputag_ctrl(23030)=%08x\n",
			   ioread32(ep->sw + SW_VLAN_CTRL),
			   ioread32(ep->sw + SW_MAC_CPU_TAG_CTRL));
		seq_printf(m, "SW p0_sts(198)=%08x p1_sts(1b8)=%08x p2_sts(1d8)=%08x cpu_sts(1f8)=%08x\n",
			   ioread32(ep->sw + 0x198), ioread32(ep->sw + 0x1b8),
			   ioread32(ep->sw + 0x1d8), ioread32(ep->sw + 0x1f8));
		/* Per-port MIB packet counters (TX_MIB@0x32000+port*0x80, RX_MIB@0x32400+
		 * port*0x80; dump first 3 counters of each block). Localises the DS drain:
		 * p2(PON) rx>0 => PON-IP frames reach the switch; p3(CPU) tx>0 => switch
		 * forwards them to the CPU. */
		seq_printf(m, "MIB p2(PON) tx=%08x %08x %08x | rx=%08x %08x %08x\n",
			   ioread32(ep->sw + 0x32100), ioread32(ep->sw + 0x32104),
			   ioread32(ep->sw + 0x32108), ioread32(ep->sw + 0x32500),
			   ioread32(ep->sw + 0x32504), ioread32(ep->sw + 0x32508));
		seq_printf(m, "MIB p3(CPU) tx=%08x %08x %08x | rx=%08x %08x %08x\n",
			   ioread32(ep->sw + 0x32180), ioread32(ep->sw + 0x32184),
			   ioread32(ep->sw + 0x32188), ioread32(ep->sw + 0x32600),
			   ioread32(ep->sw + 0x32604), ioread32(ep->sw + 0x32608));
	}
	return 0;
}

static int rtl9602c_eth_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct rtl9602c_eth *ep;
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

	ep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ep->base))
		return PTR_ERR(ep->base);

	/* Switch core (best-effort; minimal L2 flood enabled at open). */
	ep->sw = devm_ioremap(dev, SWCORE_PHYS, SWCORE_SIZE);

	/* Snapshot the bootloader's live GMAC0 control config to re-assert at open. */
	ep->ub_tcr = ep_rd(ep, R_TCR);
	ep->ub_rcr = ep_rd(ep, R_RCR);
	ep->ub_config = ep_rd(ep, R_CONFIG);
	ep->ub_cputagcr = ep_rd(ep, R_CPUTAGCR);
	ep->ub_cputag1cr = ep_rd(ep, R_CPUTAG1CR);
	ep->ub_iocmd = ep_rd(ep, R_IO_CMD);
	ep->ub_iocmd1 = ep_rd(ep, R_IO_CMD1);

	rtl9602c_eth_get_hwaddr(ep, mac);
	if (is_valid_ether_addr(mac))
		eth_hw_addr_set(ndev, mac);
	else
		eth_hw_addr_random(ndev);

	ndev->netdev_ops = &rtl9602c_eth_netdev_ops;
	netif_carrier_off(ndev);

	ret = devm_register_netdev(dev, ndev);
	if (ret)
		return ret;

	dev_info(dev, "RTL9602C NIC at %pR, MAC %pM (inherited IO_CMD %08x)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 ndev->dev_addr, ep->ub_iocmd);

	g_ep = ep;
	proc_create_single("rtl9602c_diag", 0444, NULL, rtl9602c_diag_show);
	return 0;
}

static const struct of_device_id rtl9602c_eth_of_match[] = {
	{ .compatible = "realtek,rtl9602c-nic" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl9602c_eth_of_match);

static struct platform_driver rtl9602c_eth_driver = {
	.probe	= rtl9602c_eth_probe,
	.driver	= {
		.name		= "rtl9602c-eth",
		.of_match_table	= rtl9602c_eth_of_match,
	},
};
module_platform_driver(rtl9602c_eth_driver);

MODULE_DESCRIPTION("Realtek RTL9602C Luna Ethernet driver");
MODULE_LICENSE("GPL");
