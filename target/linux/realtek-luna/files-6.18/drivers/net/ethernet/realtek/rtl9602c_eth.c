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
#include <linux/crc32.h>	/* crc32_le for the optional SW OMCI MIC path */
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
	spinlock_t	tx_lock;	/* serialises tx_head/tx_ring: xmit (process)
					 * vs OMCI inject (poll-timer softirq) */
	/* US OMCI (OMCC) responder state. */
	u8		omci_sn[8];	/* G.984.3 ONU-SN (4 ASCII ID + 4 serial),
					 * for the ONU-G Vendor-ID/Serial GET reply */
	u8		omci_mds;	/* ONU-data (ME 2) MIB-Data-Sync counter */
	u32		dbg_omci_tx;		/* US OMCI responses queued */
	u32		dbg_omci_tx_drop;	/* dropped: ring full / alloc / map */
	u32		dbg_omci_unhandled;	/* requests with no modelled reply */

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
	/* OMCI stream-id in [14:8] + low bits = 2. CONFIRMED against LIVE stock O5:
	 * CPUTAG1CR = 0x00004002 (SID 64 + 2). (The vendor-source enum reading 0x4000
	 * was wrong vs silicon.) */
	v = (v & ~((0x7fu << 8) | 0x7u)) | CPUTAG1_OMCI_SID(sid) | 2u;
	ep_wr(ep, R_CPUTAG1CR, v);
	/* CPUTAGCR[21:18] is CT_APPLO_PRO (the "PON source selector / CT_SWITCH=7"
	 * theory was WRONG). Per the vendor NIC init (re8686_rtl9607c.c CPUtagCR =
	 * CTEN_RX|2<<CT_TSIZE|2<<CT_RSIZE_L|8<<18|CTPM_8370|CTPV_8370 = 0x9022ff04) this
	 * field is 8. My earlier code FORCED it to 7 (0x901eff04), which BROKE the OMCI
	 * trap: DS OMCI de-encapsulated fine (OMCI_RX_PKT_CNT 0x329c0 climbing 6->18) but
	 * never reached the CPU ring (filled=0). Restore the vendor value 8. RMW preserves
	 * CTEN_RX/CT_TSIZE/CT_RSIZE/CTPM/CTPV -> 0x9022ff04. */
	ct = 0x901eff04u;	/* CONFIRMED against LIVE stock O5: CPUTAGCR = 0x901eff04
				 * (CT_APPLO_PRO field = 7, NOT the enum-computed 8/0x9022ff04
				 * — the vendor-source reading was wrong vs silicon). */
	ep_wr(ep, R_CPUTAGCR, ct);
	/* CONFIG_REG(0x4c): leave at the inherited 0x21000000. A live online stock ONU
	 * traps OMCI to the CPU with 0x4c = 0x21000000 (sideband bits 22/23 CLEAR), so the
	 * earlier "the controller requires config_rx_sideband on" guess was wrong — setting
	 * bits 22/23 diverged from the proven-working stock config. Do not touch 0x4c. */
	/* Route EVERY RX class to ring 0 (the only ring this driver allocates and
	 * drains). There are SEVEN routing tables (RRING_ROUTING1..7 @ 0x1370..0x1388,
	 * stride 4) selected by source/priority; OMCI's class may use one other than
	 * table 1, and an un-zeroed table sends it to an un-drained ring (the frame
	 * sticks in PON-IP -> US stall -> deactivate). Zero all 7 -> everything to ring 0
	 * (routing value 0 = ring 0, the table the LAN low-priority traffic already uses). */
	/* RRING routing: stock uses ROUTING1=0x65432100 because it sets up SIX RX rings
	 * and steers traffic by priority across them. My driver allocates only ring 0, so
	 * route EVERY priority to ring 0 (all nibbles 0) — otherwise a GMII-RX OMCI frame
	 * at a nonzero priority is steered to an unallocated ring and dropped. */
	{
		unsigned int r;
		for (r = 0; r < 7; r++)
			ep_wr(ep, R_RRING_ROUTING1 + r * 4, 0);
	}
	ep->omci_trap_on = true;
	netdev_info(ep->ndev, "OMCI trap armed: SID %u CPUTAG1CR=0x%08x CPUTAGCR=0x%08x (vendor 0x9022ff04)\n",
		    sid, v, ct);
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_sid);

/*
 * Provision the ONU identity (G.984.3 ONU-SN: 4 ASCII vendor + 4 serial bytes)
 * so the OMCI ONU-G (ME 256) GET reply reports a Vendor-ID/Serial matching the
 * PLOAM Serial_Number the OLT ranged. Called by the GPON driver once onu_sn is
 * parsed; the responder copies SN[0..3] as Vendor-ID and SN[0..7] as Serial.
 */
void rtl9602c_eth_set_omci_identity(const u8 *sn8)
{
	if (g_ep && sn8)
		memcpy(g_ep->omci_sn, sn8, sizeof(g_ep->omci_sn));
}
EXPORT_SYMBOL(rtl9602c_eth_set_omci_identity);

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
	 * MAC_CPU_TAG_CTRL = 0x300 (TAG_AWARE bit9 + TRAP_TARGET_INSERT_EN bit8):
	 * the switch PARSES the cpu-tag on CPU-port ingress so a CPU TX frame is
	 * steered by the tag (stream-id for the OMCC US OMCI, portmask for LAN)
	 * instead of L2-DA-flooded. Live stock runs 0x300. (An earlier 0x300 test
	 * still flooded, but that predates the OMCI frame carrying a DA+SA: the
	 * GMAC's offset-12 tag landed INSIDE the raw OMCI payload and was unparseable.
	 * With the 12-byte L2 header now prepended (see OMCI_L2_HDR) the tag sits at
	 * offset 12 and the switch parser can read SID 64 from it.) */
	iowrite32(0x300, ep->sw + SW_MAC_CPU_TAG_CTRL);
	/* CPU-tag aux registers (live-stock values): 0x230F0=0x00400034,
	 * 0x230F4=0x00f000ea, 0x230F8=0x00400034. The cpu-tag forwarding format the
	 * switch parser expects. */
	iowrite32(0x00400034, ep->sw + 0x230F0);
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

	/* Accept short (runt) frames on the PON port (2) and CPU port (3). A DS OMCI
	 * frame is the 48-byte G.988 baseline + headers = ~60 bytes, BELOW the 64-byte
	 * Ethernet minimum, so the switch runt-filters it unless RX_SPC (P_MISC bit2) is
	 * set. The vendor sets RX_SPC per-port. P_MISC = 0x020004 + port*0x20; bit2 =
	 * RX_SPC. Without it the de-encapsulated OMCI never reaches the CPU. */
	iowrite32(ioread32(ep->sw + 0x020044) | BIT(2), ep->sw + 0x020044); /* port 2 (PON) */
	iowrite32(ioread32(ep->sw + 0x020064) | BIT(2), ep->sw + 0x020064); /* port 3 (CPU) */

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

/* ===== M2: G.988 OMCI responder + upstream OMCC TX ======================= */

/*
 * US OMCI egresses to the PON port so the GTC GEM-US datapath encapsulates the
 * frame on the OMCC (flow/SID 64). The cpu-tag must name the PON port AND set
 * CPUTAG_PSEL so the switch honours the explicit portmask+SID instead of doing
 * an L2 DA lookup (verified against the vendor gpon_omci_tx tx_info fields:
 * CPUTAG_PSEL=1, DISLRN=1, KEEP=1, CPUTAG=1, TX_PMASK=1<<ponport, DST_SID=64).
 */
#define TXD_OMCI_PMASK		BIT(RTL9602C_PON_PORT)	/* 0x4 -> opts2[26:16] */
#define TXD3_CPUTAG_PSEL	BIT(20)			/* opts3[20]: honour cpu-tag */
#define TXD3_GMAC_ID_PON	(2u << 18)		/* opts3[19:18]=2: route to the PON
							 * GMAC (gmac7/OMCC) TX ring, NOT
							 * GMAC0/LAN. Per re8686_rtl9607c.o
							 * disasm this selects the OMCC US
							 * datapath; 0 => the frame egresses
							 * the LAN ring and ustx stays 0. */
#define TXD3_DST_SID(s)		((s) & 0x7f)		/* opts3[6:0]: tx_dst_stream_id */
#define RTL9602C_OMCC_SID	64			/* OMCC US SID (== GPON flow 64) */

/*
 * The RTL9602C GTC/GEM hardware appends the OMCI MIC on US encapsulation (the
 * DAL registers NO MIC generator for this chip — only the XG-PON Cortina parts
 * do). So leave bytes 44..47 = 0; a CPU-computed MIC would be doubled and the
 * OLT would drop the frame. Set to 0 only if a live capture shows HW does NOT
 * append it (then the SW crc32 path below is used).
 */
#define RTL9602C_OMCI_HW_MIC	1

#if !RTL9602C_OMCI_HW_MIC
/* OMCI MIC = standard CRC-32 (zlib / IEEE-802.3) over bytes 0..43, stored big-
 * endian into 44..47. crc32_le() uses the 0xEDB88320 reflected poly with no
 * internal final XOR, so XOR the ~0 seed out ourselves. */
static void rtl9602c_omci_set_mic(u8 *msg)
{
	u32 c = crc32_le(~0u, msg, 44) ^ ~0u;

	msg[44] = (u8)(c >> 24);
	msg[45] = (u8)(c >> 16);
	msg[46] = (u8)(c >> 8);
	msg[47] = (u8)(c);
}
#endif

/* Stamp the baseline trailer (40..43 = 00 00 00 28) + MIC. Call LAST. */
static void rtl9602c_omci_finalize(u8 *msg)
{
	msg[40] = 0x00;
	msg[41] = 0x00;
	msg[42] = 0x00;
	msg[43] = 0x28;		/* baseline trailer constant 0x0028 */
#if RTL9602C_OMCI_HW_MIC
	msg[44] = msg[45] = msg[46] = msg[47] = 0x00;	/* GTC/GEM HW appends MIC */
#else
	rtl9602c_omci_set_mic(msg);
#endif
}

/* HW cpu-tag insertion does NOT fire on this 9602C clean-room: a capture of the
 * egress frame shows offset 12 holds the OMCI payload, never the 0x8899 tag,
 * despite opts2.cputag=1 + CPUTAGCR armed (the same dead-end the LAN TX path
 * hit). So, exactly like the LAN, we BUILD the rtl8_4 0x8899 cpu-tag in software
 * and send a plain frame (opts2=0). The 8-byte tag goes right after the 12-byte
 * DA+SA (offset 12) where the switch's TAG_AWARE parser reads it. We use the
 * SAME mainline tag_rtl8_4 layout as the proven LAN TX path (portmask in the
 * last 2 tag bytes); for the OMCC US the portmask selects PON port 2, and the
 * PON-IP US applies SID 64 from its armed map. See the byte assignment below. */
#define OMCI_L2_HDR	(2 * ETH_ALEN)		/* 12: DA(6)+SA(6) before the tag */
#define OMCI_CPUTAG_LEN	8			/* software rtl8_4 0x8899 cpu-tag */
/* Benign locally-administered unicast DA (NOT broadcast — the switch floods
 * broadcast); the software cpu-tag steers egress, so the DA is otherwise unused. */
static const u8 omci_tx_da[ETH_ALEN] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

/*
 * Queue a 48-byte US OMCI response on the OMCC (PON, SID 64). Wrapped as
 * [DA(6)][SA(6)][sw rtl8_4 cpu-tag(8)][OMCI] = 68 bytes; opts2/opts3=0 (no HW
 * insert). No ETH_ZLEN padding (GEM payload, no 60-byte minimum). Runs in
 * poll-timer softirq context -> GFP_ATOMIC. Returns 0 on success.
 */
static int rtl9602c_eth_omci_xmit(struct rtl9602c_eth *ep, const u8 *omci,
				  unsigned int len)
{
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int i;
	dma_addr_t da;
	u32 opts1;

	if (len < 8 || len > 1500)
		return -EINVAL;
	skb = netdev_alloc_skb(ep->ndev, OMCI_L2_HDR + OMCI_CPUTAG_LEN + len);
	if (!skb) {
		ep->dbg_omci_tx_drop++;
		return -ENOMEM;
	}
	skb_put(skb, OMCI_L2_HDR + OMCI_CPUTAG_LEN + len);
	ether_addr_copy(skb->data, omci_tx_da);				/* DA (unicast) */
	ether_addr_copy(skb->data + ETH_ALEN, ep->ndev->dev_addr);	/* SA */
	{
		u8 *t = skb->data + OMCI_L2_HDR;		/* sw cpu-tag at offset 12 */

		/* Format B = vendor GMAC tx_info layout: portmask in BYTE3, stream-id in
		 * BYTE7. Format A (portmask in last 2 bytes) delivered the frame to PON
		 * port 2 but the PON-IP US never classified it (ustx=0) — it needs the SID
		 * IN the tag, not just the portmask. So: portmask byte3 = PON port 2 (the
		 * prior format-B try left this 0 = the bug), tx_dst_stream_id byte7 = 64,
		 * and keep=1 so the switch does NOT strip the tag on egress to the PON port
		 * (else the SID never reaches the US-NIC). cputag_psel selects stream-id. */
		t[0] = 0x88; t[1] = 0x99;		/* Realtek cpu-tag EtherType */
		t[2] = 0x04;				/* rtl8_4 protocol */
		t[3] = BIT(RTL9602C_PON_PORT);		/* tx_portmask = PON port 2 (0x04) */
		t[4] = 0x00;				/* aspri / cputag_pri */
		t[5] = (1 << 4) | (1 << 5) | (1 << 7);	/* psel | dislrn | keep = 0xb0 */
		t[6] = 0x00;
		t[7] = RTL9602C_OMCC_SID;		/* tx_dst_stream_id = 64 */
	}
	memcpy(skb->data + OMCI_L2_HDR + OMCI_CPUTAG_LEN, omci, len);
	len += OMCI_L2_HDR + OMCI_CPUTAG_LEN;		/* 12 + 8 + 48 = 68 */
	da = dma_map_single(ep->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ep->dev, da)) {
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -ENOMEM;
	}

	spin_lock_irqsave(&ep->tx_lock, flags);
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		dma_unmap_single(ep->dev, da, len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);
		ep->dbg_omci_tx_drop++;
		return -EBUSY;		/* ring full; OLT retransmits the request */
	}
	i = ep->tx_head % TX_RING_SIZE;
	ep->tx_skb[i] = skb;
	ep->tx_buf_dma[i] = da;
	ep->tx_buf_len[i] = len;
	ep->tx_ring[i].addr = da | DMA_BUS_WINDOW;
	/* The cpu-tag (stream-id 64 + psel) is in the SOFTWARE tag built above, so the
	 * GMAC must NOT also try to insert one (it doesn't on this silicon anyway):
	 * opts2=opts3=0, a plain frame — same as the working LAN TX path. */
	ep->tx_ring[i].opts2 = 0;
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

	ep->dbg_omci_tx++;
	ep->ndev->stats.tx_packets++;
	ep->ndev->stats.tx_bytes += len;
	return 0;
}

/* G.988 baseline message-type action codes (low 5 bits of msg[2]). */
#define OMCI_MT_CREATE		0x04
#define OMCI_MT_DELETE		0x06
#define OMCI_MT_SET		0x08
#define OMCI_MT_GET		0x09
#define OMCI_MT_GET_ALL_ALARMS	0x0b
#define OMCI_MT_GET_ALL_ALRM_NX	0x0c
#define OMCI_MT_MIB_UPLOAD	0x0d
#define OMCI_MT_MIB_UPLOAD_NX	0x0e
#define OMCI_MT_MIB_RESET	0x0f
#define OMCI_MT_GET_NEXT	0x10

/* G.988 result/reason codes (GET / response content byte 8). */
#define OMCI_RC_OK		0x00
#define OMCI_RC_NOT_SUPPORTED	0x02
#define OMCI_RC_UNKNOWN_ME	0x04
#define OMCI_RC_ATTR_FAILED	0x09

/* Managed-Entity classes we answer with real values. */
#define OMCI_ME_ONU_DATA	2
#define OMCI_ME_ONU_G		256
#define OMCI_ME_ONU2_G		257
#define OMCI_ME_CTC_LOID_AUTH	65530	/* 0xFFFA China-Telecom LOID auth */

/* GET attribute-mask bit for attribute number N (N=1 = first attr after the
 * ManagedEntityID): bit (16 - N). (Off-by-one here breaks every discovery GET.) */
#define OMCI_ATTR_BIT(n)	(1u << (16 - (n)))

static inline void omci_put_be16(u8 *p, u16 v)
{
	p[0] = (u8)(v >> 8);
	p[1] = (u8)(v);
}

/*
 * Fill the GET-response attribute region for the discovery MEs. The OLT's
 * requested @mask selects attributes (highest bit = attr#1); we return the
 * subset we model (echoed in @rmask at resp[9..10]) with values, BOUNDED so
 * writes never pass resp[39] (contents end at 39; 40..47 = trailer+MIC).
 * Returns the G.988 result code. Attribute numbers per ITU-T G.988; ONU-G
 * Vendor-ID/Serial come from the provisioned PLOAM SN so the identity matches
 * what the OLT ranged.
 */
static u8 rtl9602c_omci_get_fill(struct rtl9602c_eth *ep, u16 class_id,
				 u16 mask, u8 *resp)
{
	u8 *v = resp + 11;	/* values start after result(8) + attr-mask(9,10) */
	u8 *end = resp + 40;	/* contents hard limit */
	u16 rmask = 0;
	bool over = false;

#define PUT(bit, n, src) do {						\
		if (mask & (bit)) {					\
			if (v + (n) <= end) {				\
				memcpy(v, (src), (n)); v += (n);	\
				rmask |= (bit);				\
			} else { over = true; }				\
		}							\
	} while (0)
#define PUT1(bit, b) do {						\
		if (mask & (bit)) {					\
			if (v + 1 <= end) { *v++ = (b); rmask |= (bit); }\
			else { over = true; }				\
		}							\
	} while (0)
#define PUT2(bit, w) do {						\
		if (mask & (bit)) {					\
			if (v + 2 <= end) {				\
				omci_put_be16(v, (w)); v += 2;		\
				rmask |= (bit);				\
			} else { over = true; }				\
		}							\
	} while (0)

	switch (class_id) {
	case OMCI_ME_ONU_DATA:				/* ME 2 */
		PUT1(OMCI_ATTR_BIT(1), ep->omci_mds);	/* #1 MIB-Data-Sync */
		break;
	case OMCI_ME_ONU_G:				/* ME 256 */
		PUT(OMCI_ATTR_BIT(1), 4, ep->omci_sn);	/* #1 Vendor-ID = SN[0..3] */
		PUT(OMCI_ATTR_BIT(3), 8, ep->omci_sn);	/* #3 Serial-number = SN */
		PUT1(OMCI_ATTR_BIT(4), 0x02);		/* #4 Traffic-mgmt option */
		PUT1(OMCI_ATTR_BIT(8), 0x00);		/* #8 Op-state = enabled */
		break;
	case OMCI_ME_ONU2_G: {				/* ME 257 */
		static const u8 eqid[20] = "RTL9602C";
		PUT(OMCI_ATTR_BIT(1), 20, eqid);	/* #1 Equipment-ID */
		PUT1(OMCI_ATTR_BIT(2), 0x80);		/* #2 OMCC version (live stock) */
		PUT2(OMCI_ATTR_BIT(3), 0x0000);		/* #3 Vendor product code */
		PUT1(OMCI_ATTR_BIT(4), 0x01);		/* #4 Security capability */
		PUT1(OMCI_ATTR_BIT(5), 0x01);		/* #5 Security mode */
		PUT2(OMCI_ATTR_BIT(6), 0x0028);		/* #6 Total priority queues */
		PUT1(OMCI_ATTR_BIT(7), 0x10);		/* #7 Total traffic schedulers */
		PUT2(OMCI_ATTR_BIT(9), 0x0040);		/* #9 Total GEM ports */
		PUT2(OMCI_ATTR_BIT(11), 0x007f);	/* #11 Connectivity capability */
		break;
	}
	case OMCI_ME_CTC_LOID_AUTH:			/* ME 65530 (0xFFFA) */
		PUT1(OMCI_ATTR_BIT(4), 0x01);		/* #4 Auth-status = SUCCESS */
		break;
	default:
		return OMCI_RC_UNKNOWN_ME;
	}

#undef PUT
#undef PUT1
#undef PUT2
	omci_put_be16(resp + 9, rmask);
	return over ? OMCI_RC_ATTR_FAILED : OMCI_RC_OK;
}

/*
 * Downstream OMCI -> upstream OMCI response (M2 G.988 responder). @msg is the raw
 * baseline message (2-byte CPU prefix already stripped). Builds a 48-byte reply
 * (resp MT = (MT & 0x1f) | 0x20: clears AR, sets AK, keeps DB + action), result +
 * body per type, trailer + MIC, then TX on the OMCC. Minimal-but-correct: enough
 * for the OLT to finish discovery + config. Runs in poll-timer softirq context.
 */
static void rtl9602c_eth_omci_input(struct rtl9602c_eth *ep, const u8 *msg,
				    unsigned int len)
{
	u16 class_id, req_mask;
	u8 resp[48];
	u8 mt, devid;

	if (len < 8)
		return;
	devid = msg[3];
	mt = msg[2] & 0x1f;
	class_id = (msg[4] << 8) | msg[5];

	if (net_ratelimit())
		netdev_info(ep->ndev,
			    "OMCI DS: TID=%02x%02x MT=0x%02x class=%u inst=%u len=%u\n",
			    msg[0], msg[1], msg[2], class_id,
			    (msg[6] << 8) | msg[7], len);

	if (devid != 0x0a)		/* only baseline modelled */
		return;

	memset(resp, 0, sizeof(resp));
	resp[0] = msg[0];			/* TID echo */
	resp[1] = msg[1];
	resp[2] = (msg[2] & 0x1f) | 0x20;	/* clear AR, set AK, keep DB+action */
	resp[3] = 0x0a;				/* DevID baseline */
	resp[4] = msg[4];			/* class echo */
	resp[5] = msg[5];
	resp[6] = msg[6];			/* instance echo */
	resp[7] = msg[7];

	switch (mt) {
	case OMCI_MT_MIB_RESET:
		ep->omci_mds = 0;
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_MIB_UPLOAD:
		resp[8] = OMCI_RC_OK;
		omci_put_be16(resp + 9, 0x0000);	/* 0 subsequent Upload-Next */
		break;
	case OMCI_MT_GET:
		req_mask = (msg[8] << 8) | msg[9];
		resp[8] = rtl9602c_omci_get_fill(ep, class_id, req_mask, resp);
		break;
	case OMCI_MT_SET:
	case OMCI_MT_CREATE:
	case OMCI_MT_DELETE:
		if (++ep->omci_mds == 0)		/* G.988: wrap 255 -> 1 */
			ep->omci_mds = 1;
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_GET_ALL_ALARMS:
		omci_put_be16(resp + 9, 0x0000);	/* no active alarms */
		break;
	case OMCI_MT_MIB_UPLOAD_NX:
	case OMCI_MT_GET_ALL_ALRM_NX:
	case OMCI_MT_GET_NEXT:
		/* These response types carry NO result byte (byte 8 is content);
		 * an all-zero baseline reply is well-formed for our empty MIB. */
		break;
	default:
		ep->dbg_omci_unhandled++;
		resp[8] = OMCI_RC_NOT_SUPPORTED;
		break;
	}

	rtl9602c_omci_finalize(resp);		/* trailer + MIC */
	rtl9602c_eth_omci_xmit(ep, resp, sizeof(resp));	/* drop already counted */
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
		    ((((ep->rx_ring[i].opts2 >> 21) & 0xff) == RTL9602C_OMCI_REASON &&
		      ((ep->rx_ring[i].opts3 >> 16) & 0xf) == RTL9602C_PON_PORT) ||
		     /* DS OMCI actually arrives SWITCH-routed (no reason==246): the de-
		      * encapsulated baseline OMCI rides the GMAC CPU-port behind the 2-byte
		      * prefix as raw G.988 -> [TID(2)][MT(1)][DevID(1)=0x0a baseline/0x0b
		      * extended][class(2)][inst(2)]... Match by DevID + MT destination-bit
		      * clear. A LAN frame to the CPU has dst-MAC[3] here (board MAC ..:32:..,
		      * bcast 0xff) never 0x0a, so this does not steal LAN traffic. Verified
		      * live: OLT sent MT 0x49 (GET) DevID 0x0a class 0x0101. */
		     (len >= RX_CPU_PREFIX + 8 &&
		      (skb->data[RX_CPU_PREFIX + 3] == 0x0a ||
		       skb->data[RX_CPU_PREFIX + 3] == 0x0b) &&
		      !(skb->data[RX_CPU_PREFIX + 2] & 0x80)))) {
			/* DS OMCI on the OMCC. Capture for /proc, then hand the raw G.988
			 * message (prefix stripped) to the responder. */
			ep->dbg_omci_rx++;
			ep->dbg_omci_rxlen = len - RX_CPU_PREFIX;
			memcpy(ep->dbg_omci_rxbuf, skb->data + RX_CPU_PREFIX,
			       min_t(unsigned int, len - RX_CPU_PREFIX,
				     sizeof(ep->dbg_omci_rxbuf)));
			rtl9602c_eth_omci_input(ep, skb->data + RX_CPU_PREFIX,
						len - RX_CPU_PREFIX);
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
	/* (Reverted: a prior experiment pointed rings 1-5 at ring 0's buffer on the
	 * theory the OMCI was priority-routed to rings 1-5 — but 6 ring engines sharing
	 * ring 0's descriptors corrupts it. The OMCI arrives via the PON-NIC internal MII
	 * straight to GMAC0 GMII-RX -> ring 0; route everything to ring 0 instead, below.) */

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
	/* If the GPON OMCI trap has already armed CT_SWITCH=7 (PON-IP RX source, the
	 * value a live stock ONU runs permanently = 0x901EFF04), preserve it: this init
	 * re-runs on netdev open, and overwriting with the bootloader/LAN CT_SWITCH=6
	 * (0x981AFF04) after O5 silently un-binds the OMCI RX source so DS OMCI never
	 * reaches the CPU. Pre-trap (probe) keeps the validated bring-up value. */
	ep_wr(ep, R_CPUTAGCR, ep->omci_trap_on ? 0x901EFF04 : 0x981AFF04);
	ep_wr(ep, R_CPUTAG1CR, 0x00000000);	/* active value is 0 (not 0x4000) */
	/* GMAC config regs that a LIVE stock ONU at O5 SETS but my driver left at 0 /
	 * masked wrong — found by full block diff. The earlier masking of MSR(0x58) down
	 * to 0x10638000 was the bug: it CLEARED bits 31/30 that stock keeps set
	 * (0xf0638000 = internal DS-NIC<->GMAC RX link/force bits). With those cleared and
	 * 0xd0 (the per-ring RX-DMA enable mask, stock=0x3f = rings 0-5) left 0, the GMAC
	 * RX engine never DMA'd the DS-NIC-drained OMCI to any ring (filled=0 despite
	 * PKT_OK_CNT_DS climbing). Restore stock values. */
	ep_wr(ep, 0x24, 0x010c0000);		/* stock O5 */
	ep_wr(ep, 0xd0, 0x0000003f);		/* RX-ring DMA enable: rings 0-5 */
	ep_wr(ep, 0x58, (ep_rd(ep, 0x58) & 0x00ffffff) | 0xf0000000);	/* MSR top byte -> 0xf0 (stock 0xf0638000) */
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
	unsigned int i;
	unsigned int len = skb->len;
	unsigned long flags;
	dma_addr_t da;
	u32 opts1;

	/* tx_lock taken first so the ring-full test precedes any skb mutation
	 * (BUSY -> stack retries an UNtouched skb, no double cpu-tag) AND so
	 * tx_head/tx_ring/kick are atomic vs the softirq OMCI inject. The skb
	 * reallocs (padto/cow) and dma_map below all use GFP_ATOMIC / are IRQ-safe,
	 * so holding the irqsave lock across them is sound. */
	spin_lock_irqsave(&ep->tx_lock, flags);
	if ((ep->tx_head - ep->tx_dirty) >= TX_RING_SIZE - 1) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
	if (len < ETH_ZLEN) {
		if (skb_padto(skb, ETH_ZLEN)) {
			spin_unlock_irqrestore(&ep->tx_lock, flags);
			return NETDEV_TX_OK;	/* skb freed by skb_padto */
		}
		len = ETH_ZLEN;
	}
#if TX_CPUTAG
	/* Prepend the software cpu-tag after DA+SA (the hardware portmask insertion
	 * is broken on this silicon — see RTL8_4 defines). */
	if (skb_cow_head(skb, RTL8_4_TAG_LEN)) {
		spin_unlock_irqrestore(&ep->tx_lock, flags);
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
		spin_unlock_irqrestore(&ep->tx_lock, flags);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	i = ep->tx_head % TX_RING_SIZE;
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
	spin_unlock_irqrestore(&ep->tx_lock, flags);

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
/*
 * Self-test: writing /proc/rtl9602c_omci_test injects 5 dummy 48-byte baseline
 * OMCI frames straight through the US OMCC TX path (rtl9602c_eth_omci_xmit),
 * decoupled from the OLT's downstream OMCI. Run at O5 (grants flowing, idle16
 * climbing): if ustx (0x1b0329bc) then climbs, the descriptor steering reaches
 * the OMCC US engine; if it stays 0, the GMAC0-TX -> US-NIC steering is still the
 * gap. This breaks the "OLT won't send OMCI -> can't test US TX" deadlock.
 */
static ssize_t rtl9602c_omci_test_write(struct file *f, const char __user *ubuf,
					size_t cnt, loff_t *off)
{
	struct rtl9602c_eth *ep = g_ep;
	unsigned int k;
	u8 msg[48];

	if (!ep)
		return -ENODEV;
	/* Arm the GMAC OMCI state (CPUTAGCR=0x901eff04, the cpu-tag trap + routing) as
	 * the OLT-driven path would via gpon_install_omcc -> set_omci_sid. Without the
	 * OLT the GMAC sits at the non-OMCI 0x981aff04, so the self-test must arm it to
	 * test the cpu-tag insertion representatively. Idempotent. */
	rtl9602c_eth_set_omci_sid(RTL9602C_OMCC_SID);
	for (k = 0; k < 5; k++) {
		memset(msg, 0, sizeof(msg));
		msg[0] = 0x00; msg[1] = 0x42;	/* TID */
		msg[2] = 0x2f;			/* MIB-Reset response (AK) */
		msg[3] = 0x0a;			/* DevID baseline */
		msg[4] = 0x00; msg[5] = 0x02;	/* ME ONU-data */
		rtl9602c_omci_finalize(msg);	/* trailer + MIC */
		rtl9602c_eth_omci_xmit(ep, msg, sizeof(msg));
	}
	return cnt;
}

static const struct proc_ops rtl9602c_omci_test_pops = {
	.proc_write = rtl9602c_omci_test_write,
};

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
	seq_printf(m, "omci_tx: resp=%u drop=%u unhandled=%u mds=%u sn=%*ph\n",
		   ep->dbg_omci_tx, ep->dbg_omci_tx_drop, ep->dbg_omci_unhandled,
		   ep->omci_mds, 8, ep->omci_sn);
	seq_printf(m, "rxring: HW-owned(D_OWN=1)=%u  CPU-owned(filled)=%u\n",
		   own, hwfilled);
	seq_printf(m, "GMAC IO_CMD=%08x IO_CMD1=%08x MSR(0x58)=%08x\n",
		   ep_rd(ep, R_IO_CMD), ep_rd(ep, R_IO_CMD1),
		   ioread32(ep->base + 0x58));
	seq_printf(m, "GMAC RCR=%08x TCR=%08x CONFIG=%08x CPUTAGCR=%08x\n",
		   ep_rd(ep, R_RCR), ep_rd(ep, R_TCR), ep_rd(ep, R_CONFIG),
		   ep_rd(ep, R_CPUTAGCR));
	/* (GMAC1 0x18014000 / GMAC2 0x18016000 are DEAD MMIO on the 9602C — reading
	 * them bus-aborts the whole diag. The 9602C has only GMAC0; the d1 gmac_id=2
	 * is a 9607C-ism. US OMCI must egress GMAC0.) */
	seq_printf(m, "GMAC RxFDP=%08x RxCDO=%08x RxDesNum=%08x ringDMA=%08x\n",
		   ep_rd(ep, R_RxFDP), ep_rd(ep, R_RxCDO), ep_rd(ep, R_RxDesNum),
		   (u32)ep->rx_ring_dma);
	/* Full GMAC config diff vs LIVE stock O5 (stock golden values in comments).
	 * Hunting the reg that brings up the internal DS-NIC->GMAC RX link (MSR 0x58
	 * stock=0xf0638000 vs mine=0x10638000). */
	seq_printf(m, "GMACcfg 10=%08x[f:04a80457] 20=%08x[034c0003] 24=%08x[010c0000] 38=%08x[0a] 3c=%08x[f8350240]\n",
		   ep_rd(ep, 0x10), ep_rd(ep, 0x20), ep_rd(ep, 0x24),
		   ep_rd(ep, 0x38), ep_rd(ep, 0x3c));
	seq_printf(m, "GMACcfg 44=%08x[0f] 58=%08x[f0638000] 5c=%08x[04000000] d0=%08x[3f] d8=%08x[11110000]\n",
		   ep_rd(ep, 0x44), ep_rd(ep, 0x58), ep_rd(ep, 0x5c),
		   ep_rd(ep, 0xd0), ep_rd(ep, 0xd8));
	/* GMAC0 MAC-level MIB counters (16-bit, BE-packed two per 32-bit word):
	 * 0x10=[TXOK:RXOK] 0x14=[TXERR:RXERR] 0x18=[MISS:..]. DECISIVE for the OMCI MII
	 * delivery: at O5 with no LAN traffic, if rxok climbs the OMCI frame reaches the
	 * GMAC0 MAC; if miss climbs it reached the MAC but couldn't DMA (descriptor gap);
	 * if BOTH stay flat the DS-NIC->GMAC0 internal MII never delivered the frame. */
	seq_printf(m, "GMAC_MIB txok=%u rxok=%u txerr=%u rxerr=%u miss=%u\n",
		   ep_rd(ep, 0x10) >> 16, ep_rd(ep, 0x10) & 0xffff,
		   ep_rd(ep, 0x14) >> 16, ep_rd(ep, 0x14) & 0xffff,
		   ep_rd(ep, 0x18) >> 16);
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
		/* LAN ports p0(FE)/p1(GE): if the injected US OMCI floods here, the cpu-tag
		 * steering failed and the frame went to the L2 switch instead of the US-NIC. */
		seq_printf(m, "MIB p0(LAN) tx=%08x | p1(LAN) tx=%08x\n",
			   ioread32(ep->sw + 0x32000), ioread32(ep->sw + 0x32080));
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
	spin_lock_init(&ep->tx_lock);

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
	proc_create("rtl9602c_omci_test", 0200, NULL, &rtl9602c_omci_test_pops);
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
