// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access GPON MAC driver for the Realtek RTL9607F "Elnath" ONU.
 *
 * The RTL9607F is a Cortina-Access CA8277C ("TAURUS") SoC; its GPON MAC is a
 * Cortina IP block (register set rtl8277c_registers.h), NOT the Realtek "Luna"
 * GTC used on the RTL9602C/9607C.  This driver is a clean-room re-expression of
 * the GPLv2 Cortina ca-network-engine (aal-77c) GPON layer, the same package the
 * sibling cortina-ni Ethernet driver derives from.
 *
 * Key architectural fact (validated on live stock hardware, 2026-07-13):
 *   - The GPON MAC block lives at physical 0x4_F5506000 (the PON register window
 *     0x4_F5500000 + 0x6000).  The vendor-id register (+0x14) reads the ASCII
 *     serial-number prefix "XPON", confirming the base.
 *   - The G.984.3 activation FSM (O1..O5) runs autonomously in the MAC hardware;
 *     software reads the current state from GPON_onu.state (+0xdc) rather than
 *     ticking a software FSM.  So this driver polls/services the MAC, it does not
 *     drive the ranging handshake.
 *
 * Phase 0: probe, map the PON window, expose the ONU state + a register peek via
 * /proc so the register map can be validated from the driver on real hardware.
 * Later phases add the PSDS SerDes optics bring-up, PLOAM servicing, and the
 * OMCI/GEM datapath.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "cortina-gpon-serdes.h"
#include "cortina-gpon-bosa.h"
#include "cortina-ni.h"		/* cortina_ni_pon_rx_hook_set + cortina_ni_pon_tx */
#include "omci_responder.h"	/* Stage C: the G.988 responder + ME model */

#define DRV_NAME		"cortina-gpon"

/* PON register window (from the DT reg entry / SDK): phys 0x4_F5500000, 48 KiB. */
#define CG_PON_WINDOW_PHYS	0x4f5500000ULL
#define CG_PON_WINDOW_SIZE	0xc000

/* The GPON MAC register block sits at window + 0x6000 (aal_pon.h). */
#define CG_GPON_MAC_OFF		0x6000

/*
 * PON-SerDes (PSDS) registers, direct within the PON window.
 *   PSDS_MODE (+0xa02c): SerDes rate/mode.  GPON = 0x408 (sd_s0=1, sds_mode_s0=0x8).
 *   PSDS_RGB8 (+0xa060): SerDes status.  bit10 CKRDY_RX, bit11 CKRDY_TX (TX PLL
 *     locked off the reference clock; asserts without fiber), bit0 RX_LOS.
 */
#define CG_PSDS_MODE		0xa02c
#define CG_PSDS_RGB8		0xa05c	/* DS-lock status; locked = (val & 0x9c01)==0x9c00 (stock 0x19c00) */
#define CG_PSDS_GBOX_CTRL	0xa060	/* rx/tx bit-ordering[7:4]; stock=0x454.  WAS 0xa064 (stock=0) -> our US tx_bit_ordering never took -> OLT saw US LOS (live-diff 2026-07-13) */
#define CG_PON_EPON_SPARE	0x01c8	/* EPON_GLB_SPARE_CFG (PON window); bit31 for GPON los-rst */

/*
 * GLB (global) PON/GPON reset & clock control window: phys 0x4_F4320000, 4 KiB.
 * On our minimal build the GPON MAC reads garbage (block held in reset); the
 * vendor aal_gpon glb-reset clocks it.  Offsets + released values measured on
 * live stock (the block reads "XPON" with these):
 *   EPON_CNTL(+0x078)=0x00030000  GPON_CNTL(+0x080)=0x00000003  PON_CNTL(+0x09c)=0x0000030e
 * GPON_CNTL bits: ani_rst_n[0], gpon_rst_n[1].  PON_CNTL bits: pon_serdes_rst_n[1],
 * psds_reg_rst_n[2], ptp_rst_n[3], puc_reset[8], pdc_reset[9].
 */
#define CG_GLB_WINDOW_PHYS	0x4f4320000ULL
#define CG_GLB_WINDOW_SIZE	0x1000
#define CG_GLB_EPON_CNTL	0x078
#define CG_GLB_GPON_CNTL	0x080
#define CG_GLB_PON_CNTL		0x09c
/*
 * PON interrupt aggregation, level 1 of 2 (GLB window).  The GPON MAC's
 * int_top output feeds GLOBAL_PON_INTERRUPT_0.PON_MACi (bit0); the matching
 * enable is GLOBAL_PON_INTENABLE_0.PON_MACe.  The vendor ISR masks/unmasks
 * THIS bit around servicing ("disable SoC IRQ").
 */
#define CG_GLB_PON_INT0		0x1b0	/* GLOBAL_PON_INTERRUPT_0 */
#define CG_GLB_PON_INTEN0	0x1b4	/* GLOBAL_PON_INTENABLE_0 */
#define CG_PON_INT0_PON_MAC	BIT(0)	/* PON_MACi/e */
/*
 * PON interrupt aggregation, level 2 of 2: the NE global sub-interrupt
 * controller "ne_ictl" @ GLB+0x194 (cortina,per-ictl layout: +0 STATUS,
 * +4 ENABLE), whose output is GIC SPI 1.  The PON aggregate is LINE 5
 * (stock DTB: pon_ictl@0 { interrupt-parent = <&ne_ictl>; interrupts = <5>; }).
 * Our kernel carries no per-ictl irqchip driver, so this driver sets/acks
 * line 5 itself and requests GIC SPI 1 directly (IRQF_SHARED).
 */
#define CG_GLB_NE_ICTL_STS	0x194	/* per-ictl STATUS  (ne_ictl) */
#define CG_GLB_NE_ICTL_EN	0x198	/* per-ictl ENABLE  (ne_ictl) */
#define CG_NE_ICTL_PON_LINE	BIT(5)	/* PON = ne_ictl line 5 */
/*
 * GLOBAL_PSDS_INIT_CNTL: bit5 POW_PCIX powers the PON-SerDes analog+digital
 * logic, which generates the PON APB register-bus clock the GPON MAC lives on.
 * bit4 ben_oen is the laser burst-enable (leave 0 during bring-up).  The CA8277C
 * physical offset is +0x25c (header 0x22c is wrong / shifted); measured live:
 * stock reads 0x30 (POW_PCIX + ben_oen), cold reads 0x00.
 */
#define CG_GLB_PSDS_INIT	0x25c
#define CG_PSDS_POW_PCIX	BIT(5)
#define CG_PSDS_BEN_OEN		BIT(4)

/*
 * Laser TX-disable GPIO.  On this board the GN25L95's hardware TX_DIS input
 * hangs on a net that GPIO pin 34 (group 1, bit 2) only MIRRORS as an input;
 * the net is actually pulled low (= laser enabled) by a GPIO group-0 pin
 * route + drive — see the CG_PERGPIO_CFG0 block below.  The vendor's
 * ca_pon_laser_tx_disable_set (pin = CONFIG_TX_DISABLE_GPIO_PIN = 34)
 * touches only the mirror.  Register map (RMW only, never whole-register
 * writes on the mux):
 *   GLOBAL_GPIO_MUX_1 (GLB  +0x134): SET the bit -> pin is a GPIO
 *   PER_GPIO1_CFG    (PERI +0x324): 1 = INPUT (stock: pin 34 is an input)
 *   PER_GPIO1_IN     (PERI +0x32c): bit2 = live net level, 0 = laser on
 * The PER_GPIO block is a separate MMIO window (0x4_F4329000) from the GLB one.
 *
 * ★ REAL SILICON OFFSETS + POLARITY (three agreeing sources: the stock
 * ca-ne.ko ca_pon_laser_tx_disable_set disasm — MUX = GLB + ((0xf4320130>>2 +
 * group)<<2 & 0xfff), OUT/CFG = PERI + 0x304/0x300 + 0x24*group, MUX bit is
 * ORed in; the stock rootfs /etc/reg.txt — GLOBAL_GPIO_MUX_1 0xf4320134,
 * PER_GPIO1_CFG/OUT 0xf4329324/0xf4329328; and the stock DTB gpio-controller
 * node reg = <0xf4329300 0xb4, 0xf4320130 0x14>).  The older
 * rtl8277c_registers.h values (MUX_1 0x104, CFG1/OUT1 0x2e4/0x2e8, mux
 * cleared) are STALE for this silicon — with them the BOSA status reg 0x6e
 * read 0x80 (bit7 = hardware TX_DIS input pin still ASSERTED) and the OLT saw
 * zero upstream energy while the MAC bursted; stock at Online reads 0x6e=0x00.
 */
#define CG_PERGPIO_PHYS		0x4f4329000ULL
#define CG_PERGPIO_SIZE		0x1000
#define CG_GLB_GPIO_MUX1	0x134
#define CG_PERGPIO_CFG1		0x324
#define CG_PERGPIO_OUT1		0x328
#define CG_PERGPIO_IN1		0x32c
#define CG_LASER_PIN34		BIT(2)
/*
 * ★ The REAL laser-enable path (live golden diff ours-vs-stock-at-Online,
 * txpart-2026-07-16): pin 34 only MIRRORS the TX-disable net; what actually
 * pulls it low is a GPIO **group 0** pin route + drive that our driver never
 * set up (same class as the i2c0 pinmux root cause).  Stock at Online:
 *   GLB +0x42c (pin-route/rstmgr) = 0x01101101 (ours cold: 0x01001101, bit20
 *     clear -> the laser-enable net is never routed);
 *   PER_GPIO0_CFG (+0x300) = 0xFFFFE7BF: pins 6, 11, 12 are OUTPUTS (ours:
 *     0xFFFFF7FF, pin 11 only);
 *   PER_GPIO0_OUT (+0x304) = 0x00000040: pin6=HIGH, pin11=LOW, pin12=LOW
 *     (ours: 0x00000800 = pin11 driven HIGH -> TX_DIS held asserted);
 *   PER_GPIO1_IN (+0x32c) bit2 (pin 34) = 0 -> the net reads LOW =
 *     TX-disable de-asserted; ours read 1 and the GN25L95 statusControl
 *     0x6E kept bit7=1 (hardware TX_DIS input asserted) -> laser dark.
 */
#define CG_PERGPIO_CFG0		0x300
#define CG_PERGPIO_OUT0		0x304
#define CG_GLB_GPIO_MUX0	0x130	/* stock 0x00001FFF: pins 0-12 are GPIO */
#define CG_GLB_GPIO_MUX3	0x13c	/* stock 0x000390FF: pins 96-103,108,111-113 */
#define CG_GLB_GPIO_MUX4	0x140	/* stock 0x00003B00: pins 136,137,139-141 */
#define CG_GLB_PINROUTE		0x42c	/* stock 0x01101101; bit20 = laser net */
#define CG_PINROUTE_LASER	BIT(20)
#define CG_GPIO0_LASER_PINS	(BIT(6) | BIT(11) | BIT(12))
#define CG_GPIO0_PIN6		BIT(6)
/*
 * GPIO groups 3 and 4 (PERI +0x36c/+0x370, +0x390/+0x394): stock ACTIVELY
 * drives these pins at Online (captured live over dssh 2026-07-16) while our
 * cold boot leaves them non-GPIO inputs.  Byte-matching grp0/grp1 + the BOSA
 * program alone left the GN25L95 TX still disabled (0x6e bit7=1, zero TX
 * bias), so the laser-enable / BOSA-control net hangs on one of THESE pins:
 *   grp3 cfg 0xFFFDEF00 (outputs: pins 96-103, 108, 113), out 0x00021010
 *     (pins 100, 108, 113 HIGH; 96-99, 101-103 LOW);
 *   grp4 cfg 0xFFFFC5FF (outputs: pins 137, 139-141), out 0x00003200
 *     (pins 137, 140, 141 HIGH; 139 LOW).
 * Replicate the whole-group state exactly as stock drives it.
 */
#define CG_PERGPIO_CFG3		0x36c
#define CG_PERGPIO_OUT3		0x370
#define CG_PERGPIO_CFG4		0x390
#define CG_PERGPIO_OUT4		0x394
#define CG_GPIO3_CFG_STOCK	0xfffdef00
#define CG_GPIO3_OUT_STOCK	0x00021010
#define CG_GPIO4_CFG_STOCK	0xffffc5ff
#define CG_GPIO4_OUT_STOCK	0x00003200

/*
 * GPON MAC register offsets within the block (rtl8277c_registers.h, aal_gpon.c).
 *
 * ★ SILICON LAYOUT vs THE HEADER (proven live 2026-07-15 + stock ca-ne.ko
 * disasm): the CA8277C inserts an EXTRA 0x20-byte AES-key bank after header
 * offset 0x4c (two 8-word key banks at 0x30-0x6c — stock __gpon_common_init
 * writes both), so EVERY header offset >= 0x50 sits at header+0x20 on silicon.
 * Proof: stock binary reads the onu reg at MAC+0xdc (hdr 0xbc) and services
 * interrupts at MAC+0xa4/0xa8/0xac (hdr 0x84/0x88/0x8c); live 0xdc={id=1,
 * state=4} matches the OLT at Online, 0xfc counts 125us superframes.
 * Offsets < 0x50 are UNshifted.
 */
#define CG_REG_GPON_DS		0x000	/* DS framer thresholds; max_packet_size low */
#define CG_REG_US		0x00c	/* us: frame_var[8:0], eqd_select[16] */
#define CG_REG_SIGNAL		0x010	/* SF/SD BER alarm thresholds */
#define CG_REG_VENDOR		0x014	/* vendor-id (ASCII "XPON") */
#define CG_REG_VENDOR_SPEC	0x018	/* vendor-specific serial number */
#define CG_REG_ALARM		0x09c	/* hdr 0x7c: LOS/LOF alarm bits (live levels) */

/*
 * Interrupt block: header 0x84..0xa8 -> SILICON 0xa4..0xc8 (+0x20 shift).
 * int_top is READ-TO-CLEAR; the four per-group STATUS registers are W1C.
 *   int_top bits[3:0] = {INTERRUPT, INTERRUPT2, INTERRUPT3, INTERRUPT4}.
 * (The header offsets 0x84-0xa8 land on clear-on-read DS MIB counters and the
 * ploamu control reg — writing "enables" there corrupted the US PLOAM engine.)
 */
#define CG_REG_INT_TOP		0x0a4	/* hdr 0x84: interrupt_top (read-clear) */
#define CG_REG_INT_TOP_EN	0x0a8	/* hdr 0x88: int_top_en */
#define CG_REG_INT		0x0ac	/* hdr 0x8c: INTERRUPT  status (W1C) - operational */
#define CG_REG_INT_EN		0x0b0	/* hdr 0x90: INTERRUPT  enable */
#define CG_REG_INT2		0x0b4	/* hdr 0x94: INTERRUPT2 status (W1C) - TC/parse err */
#define CG_REG_INT2_EN		0x0b8	/* hdr 0x98: INTERRUPT2 enable */
#define CG_REG_INT3		0x0bc	/* hdr 0x9c: INTERRUPT3 status (W1C) - negedge/MSB */
#define CG_REG_INT3_EN		0x0c0	/* hdr 0xa0: INTERRUPT3 enable */
#define CG_REG_INT4		0x0c4	/* hdr 0xa4: INTERRUPT4 status (W1C) - FEC MSB */
#define CG_REG_INT4_EN		0x0c8	/* hdr 0xa8: INTERRUPT4 enable */

#define CG_INT_TOP_EN_ALL	0xF		/* vendor GPON_MAC_GPON_INT_TOP_ENA_DEF */
/* vendor GPON_MAC_GPON_INT_ENA_DEF 0xC00AFFFF + bit27; stock at O5 reads
 * 0xC80AFFFF.  bit27 is reserved in the (older) rtl8277c header but is a real
 * source on this silicon: DS-PLOAM message received — the stock __intr_handler
 * keys the Extended_Burst_Length -> us.frame_var recompute off it. */
#define CG_INT_EN_DEFAULT	0xC80AFFFF	/* int2/3/4 enables = 0 */

/* INTERRUPT source bits we service (alarm bits 0..15 are event/diag) */
#define CG_INT_ONU_ST_CHG	BIT(31)	/* ONU activation-FSM state changed */
#define CG_INT_ONU_ID		BIT(30)	/* Assign_ONU-ID accepted -> bind OMCC T-CONT */
#define CG_INT_PLOAMD		BIT(27)	/* DS PLOAM msg received (recheck frame_var) */
#define CG_INT_KSW		BIT(19)	/* Key_Switching_Time (AES rekey; next phase) */
#define CG_INT_PORTID		BIT(17)	/* Configure_Port-ID -> omci_port valid, bind OMCC GEM */
#define CG_INT_DACT		BIT(8)	/* Deactivate_ONU-ID */

#define CG_REG_GPON_ONU		0x0dc	/* hdr 0xbc: ONU id[7:0], state[18:16]; dft id=0xff */
#define CG_ONU_ID(v)		((v) & 0xff)
#define CG_ONU_STATE(v)		(((v) >> 16) & 0x7)
#define CG_ONU_ID_NONE		0xff	/* reset default = unassigned */
/* onu.state encoding (vendor aal_gpon.h): 0=O1 Initial, 1=O2 Standby,
 * 2=O3 SerialNumber, 3=O4 Ranging, 4=O5 Operation, 5=O6 POPUP, 6=O7 EmrgStop.
 * (The 2026-07-13 "state=3 at O5" read was BOGUS: it read hdr offset 0xbc,
 * which on silicon is interrupt3 — LOSi|LOFi latched reads 0x3 at O5 too.
 * The real onu reg at 0xdc reads {id=1, state=4} on stock at Online.) */
#define CG_STATE_RANGING	3
#define CG_STATE_OPERATION	4
#define CG_STATE_POPUP		5
#define CG_STATE_ESTOP		6

#define CG_REG_GPON_MAIN	0x0e0	/* hdr 0xc0: equalization delay (EqD) */
#define CG_REG_OMCI_PORT	0x0e8	/* hdr 0xc8: omci_port id[11:0], en[12]; HW-filled */
#define CG_OMCI_PORT_ID(v)	((v) & 0xfff)
#define CG_OMCI_PORT_EN		BIT(12)
#define CG_REG_T3_PREAMBLE	0x0f8	/* hdr 0xd8: extend[16], ranged[15:8], pre_range[7:0];
					 * HW-latched from the OLT's Extended_Burst_Length PLOAM */

/*
 * Indirect table access pairs, +0x20-shifted like everything >= hdr 0x50.
 * Live-confirmed on stock at Online (TCONT_ACCESS 0x14c=0x40000101,
 * US_PORT_ID_DATA 0x194=0xDF = the OLT-assigned GEM port).
 * Protocol (vendor __GPN_*_DO_INDIRCT_OP): write ACCESS = go(bit31) | rbw(bit30,
 * 1=write) | index/alloc-id, then poll ACCESS bit31 self-clear (<= 10000 reads).
 * Data flows through the DATA register (read entry -> DATA; DATA -> write entry).
 */
#define CG_REG_TCONT_ACCESS	0x14c	/* header 0x12c: alloc_id[11:0], sw_plm_en[16], rbw[30], go[31] */
#define CG_REG_TCONT_DATA	0x150	/* header 0x130: ploam_en[0], omci_en[1], index[6:2] (hw T-CONT 0-31) */
#define CG_REG_DS_GEM_ACCESS	0x154	/* header 0x134: id[11:0] (GEM port-id), sw_aes[16], rbw[30], go[31] */
#define CG_REG_DS_GEM_DATA	0x158	/* header 0x138: vld[0], aes[1], tdm[2], index[10:3] (intern gem) */
#define CG_REG_US_PORT_ACCESS	0x190	/* header 0x170: index[7:0] (us hw gem 0-255), rbw[30], go[31] */
#define CG_REG_US_PORT_DATA	0x194	/* header 0x174: id[11:0] (GEM port-id) */

#define CG_DS_GEM_VLD		BIT(0)
#define CG_DS_GEM_INDEX(x)	(((x) & 0xff) << 3)

#define CG_TBL_GO		BIT(31)
#define CG_TBL_WR		BIT(30)
#define CG_TCONT_PLOAM_EN	BIT(0)
#define CG_TCONT_OMCI_EN	BIT(1)
#define CG_TCONT_INDEX(x)	(((x) & 0x1f) << 2)
#define CG_TCONT_INDEX_MASK	(0x1f << 2)

#define CG_OMCC_US_GEM_IDX_NUM	8	/* vendor AAL_GPON_OMCI_RSV_PORT_MAX: us hw gems 0..7 = OMCC */

/*
 * Stage D — the WAN data path.  ONE data T-CONT + ONE bidirectional data GEM
 * (what this OLT's default lineprofile provisions), plus the DS broadcast GEM.
 *
 * Index scheme (vendor-faithful): hw T-CONT 0 = OMCC, hw T-CONT 1 = data.
 * In the PUC's 8Q VoQ map (VoQID = {HdrA.ldpid[3:0], HdrA.cos[2:0]}) the
 * data T-CONT's queues are VoQ 8..15, and the vendor keeps the internal GEM
 * index == VoQ for US GEMs (OMCC = 0..7, first data GEM = tcont*8+queue), so
 * the US engine stamps US_PORT_ID[VoQ] onto the burst.  The CPU injects data
 * with HEADER_A ldpid = 0x20+tcont (the CPU_MQ/LLID-GEM logical ports, whose
 * ARB map entry routes to the QM physical port) — see cortina-ni-regs.h.
 * The DS broadcast GEM (port-id 4095, carries e.g. the DHCP OFFER on this
 * OLT family) gets the next internal index, DS-only.
 */
#define CG_DATA_TCONT_IDX	1	/* hw T-CONT of the OLT's data alloc-id */
#define CG_DATA_GEM_IDX		(CG_DATA_TCONT_IDX * 8)	/* intern gem idx = VoQ 8 */
#define CG_MCAST_GEM_IDX	(CG_DATA_GEM_IDX + 1)	/* DS-only broadcast GEM */
#define CG_MCAST_GEM_ID		4095	/* G.984 broadcast GEM port-id */

/*
 * PDC (packet-downstream classifier) sub-block: PON window + 0x9000, a
 * SEPARATE block from the GPON MAC (+0x6000) — plain header offsets, no
 * +0x20 silicon shift (that shift is a GPON-MAC-block quirk; confirm with
 * one stock devmem of 0x4f5509014 at Online).  The PDC maps each DS GEM
 * (by internal index) to a logical destination port: without it a
 * de-encapsulated DS frame has nowhere to go and DS OMCI never reaches the
 * CPU.  Vendor __pdc_gpon_family_init (aal_pdc.c): map entries 0..7 (the
 * OMCC-reserved GEMs) -> CPU port 0 with fe_bypass+no_drop+cos 6, entries
 * 8..255 (data GEMs) -> L3_WAN; then PDC_CTRL arms the map memory and the
 * OMCI high-priority override (cos 7 -> CPU_0).
 *
 * PDC_MAP indirect access protocol = the same go/rbw/poll dance as the MAC
 * tables: ACCESS = go(bit31) | rbw(bit30, 1=write) | address[7:0], poll
 * bit31 self-clear (<= 10000 reads); data through DATA0/DATA1.
 */
#define CG_PDC_CTRL		0x9014	/* dft 0x2 (pdc_map_mem_en) */
#define CG_PDC_CTRL_MAP_MEM_EN	BIT(1)
#define CG_PDC_CTRL_HP_COS_SH	16	/* omci_hp_cos[18:16] */
#define CG_PDC_CTRL_HP_LDPID_SH	19	/* omci_hp_ldpid[24:19] */
#define CG_PDC_CTRL_HP_EN	BIT(25)	/* omci_hp_en */
#define CG_PDC_CTRL_HP_MASK	GENMASK(25, 16)
#define CG_PDC_MAP_ACCESS	0x9020	/* address[7:0], rbw[30], go[31] */
#define CG_PDC_MAP_DATA1	0x9024	/* pol_en[3:2], pol_id[12:4], pol_grp_id[15:13], deepq[16] */
#define CG_PDC_MAP_DATA0	0x9028	/* cos[2:0], ldpid[8:3], lspid[14:9], fe_bypass[15], no_drop[31] */
#define CG_PDC_MAP_ENTRIES	256	/* vendor AAL_PDC_MAP_ENTRY_NUM */
#define CG_PDC_D1_POL_ID(x)	(((x) & 0x1ff) << 4)
#define CG_PDC_D0_COS(x)	((x) & 0x7)
#define CG_PDC_D0_LDPID(x)	(((x) & 0x3f) << 3)
#define CG_PDC_D0_LSPID(x)	(((x) & 0x3f) << 9)
#define CG_PDC_D0_FE_BYPASS	BIT(15)
#define CG_PDC_D0_NO_DROP	BIT(31)
/* logical port ids (vendor aal_port.h) */
#define CG_LPORT_CPU_0		0x10	/* CPU port 0 = the NI CPU-RX EPP port we drain */
#define CG_LPORT_L3_WAN		0x18
#define CG_LPORT_PON		0x07

/*
 * PUC (PON Upstream Classifier) sub-block: PON window + 0x8000.  This is the
 * admission stage between the NI DMA-LSO egress and the GPON-MAC GEM-US
 * engine.  A CPU-injected US OMCI frame reaches the PUC (via its HEADER_A
 * ldpid = PON(7)+8 = the "9th queue", 8Q VoQID = {ldpid[3:0]=0xf, cos[2:0]=7}
 * = 127), but with the block at reset defaults the OMCC VoQs are neither
 * mapped, valid, nor GEM/cos-stamped -> the frame is silently dropped: the
 * DMA-LSO ring drains yet nothing ever bursts upstream and the OLT keeps
 * retransmitting its OMCI Get.  This is the vendor aal_puc_init GPON path,
 * run once at __gpon_datapath_init after the PDC.  Offsets are plain PON-
 * window offsets (reg.txt); the +0x20 shift is a GPON-MAC-block quirk and
 * does NOT apply here (PUC is a distinct sub-block, like the PDC at +0x9000).
 *
 * Indirect tables reuse the go/rbw/poll protocol (ACCESS = go[31] | rbw[30]
 * | addr; DATA around it): PVTBL (per-T-CONT VoQ map, 5 data words),
 * VOQBPREMAP (per-VoQ back-pressure remap).
 */
#define CG_PUC_BASE		0x8000	/* PON window + 0x8000 */
#define CG_PUC_PVTBL_ACCESS	(CG_PUC_BASE + 0x000)	/* addr[5:0]=T-CONT, rbw[30], go[31] */
#define CG_PUC_PVTBL_DATA4	(CG_PUC_BASE + 0x004)
#define CG_PUC_PVTBL_DATA3	(CG_PUC_BASE + 0x008)
#define CG_PUC_PVTBL_DATA2	(CG_PUC_BASE + 0x00c)	/* voq7[7:0], schmode[8], entryvld[12], wrr0/1 */
#define CG_PUC_PVTBL_DATA1	(CG_PUC_BASE + 0x010)	/* voq3[3:0],voq4,voq5,voq6,voq7[31] */
#define CG_PUC_PVTBL_DATA0	(CG_PUC_BASE + 0x014)	/* voq0,voq1,voq2,voq3[31:27] */
#define CG_PUC_VOQMAPCFG	(CG_PUC_BASE + 0x04c)	/* voqmapsel[1:0]: 0 = 8Q mode */
#define CG_PUC_BTCCFG		(CG_PUC_BASE + 0x050)
#define CG_PUC_PUCCFG		(CG_PUC_BASE + 0x08c)	/* dft 0x84040001 */
#define CG_PUC_VOQBUFLIMSEL0	(CG_PUC_BASE + 0x090)	/* 16 regs, stride 4 (0x090..0x0cc) */
#define CG_PUC_VOQBUFLIMSEL_N	16
#define CG_PUC_VOQBUFLIMIT_A	(CG_PUC_BASE + 0x0d0)
#define CG_PUC_VOQBUFLIMIT_B	(CG_PUC_BASE + 0x0d4)
#define CG_PUC_VOQBUFLIMIT_C	(CG_PUC_BASE + 0x0d8)
#define CG_PUC_BPCNTL		(CG_PUC_BASE + 0x0e4)	/* bpen[0], dropen[4], bpth[30:16] */
#define CG_PUC_VOQBPREMAP_ACCESS (CG_PUC_BASE + 0x0e8)	/* addr[7:0]=VoQ, rbw[30], go[31] */
#define CG_PUC_VOQBPREMAP_DATA	(CG_PUC_BASE + 0x0ec)	/* tqmvoqid[7:0] */
#define CG_PUC_PONCNTL_INTEN	(CG_PUC_BASE + 0x0f4)
#define CG_PUC_CTRL		(CG_PUC_BASE + 0x13c)	/* dft 0x3300007c; shp_en[30], rl_en[26] */
#define CG_PUC_CTRL1		(CG_PUC_BASE + 0x140)	/* rlovhd[4:0], shpovhd[9:5], agrshpovhd[14:10] */
#define CG_PUC_CTRL2		(CG_PUC_BASE + 0x144)	/* dft 0x03000000; pirovhd[4:0], pir_en[26] */
#define CG_PUC_VOQFLUSH		(CG_PUC_BASE + 0x0dc)	/* voqid[7:0], tcontid[12:8], openpktflushen[16], start[31] */
#define CG_PUC_VALID_VOQ0	(CG_PUC_BASE + 0x1bc)	/* valid_voqN = VALID_VOQ0 - (voq/32)*4 */
#define CG_PUC_Q2PQSRCFG01	(CG_PUC_BASE + 0x230)	/* qm_rpt_lv0[15:0], lv1[31:16] */
#define CG_PUC_Q2PQSRCFG23	(CG_PUC_BASE + 0x234)	/* qm_rpt_lv2[15:0], lv3[31:16] */
#define CG_PUC_BMC_RX_PKT	(CG_PUC_BASE + 0x17c)	/* US frames received by the PUC */
#define CG_PUC_BMC_RX_PKT_ENQ	(CG_PUC_BASE + 0x180)	/* US frames enqueued to a VoQ */
#define CG_PUC_BMC_FORCE_DROP	(CG_PUC_BASE + 0x184)	/* US frames dropped (invalid VoQ) */
#define CG_PUC_US_OMCI_HDR_A	(CG_PUC_BASE + 0x160)	/* gemid[7:0],cos[10:8],tcont[21:16],datapkt[30],en[31] */
#define CG_PUC_US_OMCI_HP_HDR_A	(CG_PUC_BASE + 0x164)	/* gemid[7:0],cos[10:8],tcont[21:16] */
#define CG_PUC_GLOBAL_PLOAM_CFG	(CG_PUC_BASE + 0x168)	/* us_hdr_min_size[21:16], us_ext_omci_en[31] */

/* XGPN_PUCIF_CTRL (PON window + 0x4fe0, dft 0x1040a100): the PUC<->US-scheduler
 * interface control; GPON sets cntr_inccfg=2 (clear-on-read) -> 0x5040a100. */
#define CG_XGPN_PUCIF_CTRL	0x4fe0
#define CG_XGPN_PUCIF_CTRL_VAL	0x5040a100

#define CG_PUC_TCONT_NUM	32	/* AAL_GPON_SYSTEM_MAX_TCONT_NUM */
#define CG_PUC_QUEUE_PER_TCONT	8	/* 8Q mode */
#define CG_PUC_9TH_QUEUE_VOQ	127	/* the CPU high-prio inject VoQ (ldpid 0xf, cos 7) */
/*
 * onu_cfg (hdr 0x118 -> silicon +0x138).  Top byte laser_on_align=0x12 aligns
 * the upstream laser burst to the OLT's grant window; at the reset default
 * (0x00100780, laser_on_align=0) the burst is mis-aligned and the OLT cannot
 * decode the SerialNumber, so ranging stalls at O1.  This is range-critical.
 * laser_pre_bias[11:7]=18 (reset dft 15): stock at Online reads 0x12100900 —
 * the vendor raises the burst pre-bias from a rodata config blob.
 */
#define CG_REG_ONU_CFG_REAL	0x138
#define CG_ONU_CFG_VAL		0x12100900
/*
 * The activation control register: the header calls it onu_ctl at +0x114, but on
 * this CA8277C silicon it is at +0x134 (the +0x20 shift, measured live: stock
 * reads +0x134 = 0x00460262 with the enable bit set at O5, while +0x114 reads 0).
 * Bit1 = en -> the MAC autonomously ranges O1->O5.  We write the full stock value.
 */
#define CG_REG_ONU_CTL		0x134
#define CG_ONU_CTL_VAL		0x00460262	/* stock O5 value: en(bit1) + defaults */
/*
 * GPON_MAC_GPON_CTRL (hdr 0x1c4 -> silicon +0x1e4, dft 0x00430000).
 *   sw_random_en(16): SN random-delay engine.  Default ON = the engine keeps
 *     recomputing the Serial_Number response delay every frame, so the SN burst
 *     lands at a churning offset and the OLT never decodes it ("Laser out").
 *     Vendor aal_pon_mac_enable_set clears it right after enabling the MAC
 *     ("stop random delay calculation when enable GPON MAC"); stock at Online
 *     shows ploamu.sn_rdm_dly frozen.
 *   pti_omci(17): cleared by vendor __gpon_common_init.
 * Stock resting value 0x1F400000 (flush_id residue of the post-O5 drain loop).
 */
#define CG_REG_GPON_MAC_CTRL	0x1e4
#define CG_MAC_CTRL_SW_RANDOM_EN BIT(16)
#define CG_MAC_CTRL_PTI_OMCI	BIT(17)

/* one post-O5 servicing event, snapshotted in hardirq, handled in the work */
struct cg_evt {
	u32 intr;			/* INTERRUPT (0x8c) sources, already enable-masked */
	u8 state;			/* onu.state at IRQ time */
	u8 id;				/* onu.id at IRQ time */
};

#define CG_EVT_RING_SZ		16	/* power of 2 */

struct cortina_gpon {
	struct device *dev;
	void __iomem *pon;		/* ioremap of the whole PON window */
	void __iomem *mac;		/* pon + CG_GPON_MAC_OFF, the GPON MAC block */
	void __iomem *glb;		/* ioremap of the GLB reset/clock window */
	void __iomem *gpio;		/* ioremap of the PER_GPIO window */
	struct proc_dir_entry *proc;

	/* post-O5 servicing (ISR top half -> event ring -> work bottom half) */
	int irq;			/* GIC SPI 1, shared NE global line */
	spinlock_t evt_lock;		/* protects the ring, taken in hardirq */
	struct cg_evt evt[CG_EVT_RING_SZ];
	unsigned int evt_head, evt_tail;
	struct work_struct isr_work;
	u32 irq_count;			/* ISR entries that found PON work */
	u32 evt_drop;			/* events lost to a full ring */
	u8 last_state;			/* FSM tracker (0=O1 .. 6=O7) */
	bool omcc_up;			/* OMCC channel bound + link signalled */
	u16 omcc_alloc;			/* last alloc-id bound to T-CONT[0] */
	u16 omcc_gem;			/* last omci_port.id bound to us-gem 0..7 */

	/* DS OMCI receive (Stage B: count + decode-log; responder = Stage C) */
	u32 omci_rx;			/* DS OMCI PDUs delivered by the NI CPU-RX hook */
	u32 omci_rx_short;		/* runt PDUs (< 8 bytes, not decodable) */
	bool pdc_ready;			/* PDC map + CTRL programmed */
	bool puc_ready;			/* PUC US-VoQ admission programmed */

	/* Stage C: the G.988 OMCI responder + US OMCI TX */
	struct omci_onu *omci;		/* responder context (kzalloc'd at probe) */
	spinlock_t omci_lock;		/* RX hook (softirq) vs isr_work/AVC work */
	bool omci_active;		/* ctx armed (OMCC up) */
	struct delayed_work veip_avc_work;	/* the ~31s post-O5 VEIP oper-up AVC */
	u32 omci_tx;			/* US OMCI responses enqueued to the NI */
	u32 omci_tx_fail;		/* NI TX rejected (ring/scratch busy) */
	u32 omci_ds_crc_ok;		/* DS MIC self-check (first PDUs only) */
	u32 omci_ds_crc_bad;

	/* Stage D: the OLT-provisioned WAN data path.  The shadow (dt_/dg_)
	 * survives an O5 exit so a LOS re-range where the OLT does NOT
	 * re-provision still re-installs (the X111W fiber-pull lesson); an
	 * on-wire MIB-Reset clears it (fresh provisioning follows). */
	u16 dt_alloc;			/* data T-CONT alloc-id (OMCI Set/Create ME 262) */
	u16 dt_inst;			/* ..the ME instance it came on */
	u16 dg_gem;			/* data GEM port-id (OMCI Create ME 268 attr 1) */
	u16 dg_tcont_ptr;		/* ME 268 attr 2 (diagnostic) */
	u8 dg_dir;			/* ME 268 attr 3 direction (diagnostic) */
	bool data_installed;
	u32 omci_cfg_log;		/* config-ME body log budget used */
	struct net_device *wan_ndev;	/* gpon0 */
};

static struct cortina_gpon *cg_singleton;

static bool cg_do_reset = true;
module_param_named(reset, cg_do_reset, bool, 0444);
MODULE_PARM_DESC(reset, "release the GPON MAC from reset/clock-gate at probe (default on)");

static bool cg_do_intr = true;
module_param_named(intr, cg_do_intr, bool, 0444);
MODULE_PARM_DESC(intr, "enable the GPON MAC interrupt servicing path (default on)");

/*
 * Put the whole PON domain into a known reset state so the SerDes can be
 * brought up first.  This is the vendor aal_gpon __gpon_glb_reset sequence
 * ONLY — the aal_gpon_glb_ctrl_init release to the stock state (PON_CNTL=
 * 0x30e, GPON_CNTL=0x3) happens at the END of cg_psds_init(), once the
 * SerDes clock is alive, so the GTC reset edge actually propagates.  The
 * released values match live stock:
 *   EPON_CNTL=0x00030000 (onu mode), PON_CNTL=0x0000030e (pon_serdes/psds/ptp +
 *   puc/pdc), GPON_CNTL=0x00000003 (ani_rst_n + gpon_rst_n).
 * cortina-ni does not touch these registers, so this is safe and independent.
 */
static void cg_glb_reset(struct cortina_gpon *cg)
{
	void __iomem *glb = cg->glb;

	/*
	 * aal_gpon __gpon_glb_reset: SerDes power OFF first, mode select, then
	 * assert every PON-domain reset and release ONLY psds_reg_rst_n (the
	 * SerDes CSR bus), so the analog profile can be loaded.
	 *
	 * ★ The GTC+ANI resets (GPON_CNTL) are HELD asserted through the whole
	 * SerDes bring-up.  Releasing them here — while POW_PCIX=0 and the PON
	 * APB/line clock is dead — means the 0->0x3 sync-reset edge cannot
	 * propagate through the framer's flops: the DS framer powers up in a
	 * nondeterministic state (the same image sometimes frame-syncs,
	 * sometimes sits at O1/LOF forever; warm reboots inherit the wedged
	 * state).  The stock aal_gpon flow does the 0->0x3 edge only AFTER the
	 * SerDes is powered and locked — see the tail of cg_psds_init().
	 */
	writel(0x00000001, glb + CG_GLB_PSDS_INIT);	/* __psds_ad_reset: POW_PCIX=0, SerDes off */
	writel(0x00030000, glb + CG_GLB_EPON_CNTL);	/* select PON/ONU mode */
	writel(0x00000000, glb + CG_GLB_PON_CNTL);	/* assert all PON-domain resets */
	writel(0x00000000, glb + CG_GLB_GPON_CNTL);	/* GTC+ANI stay IN reset until SerDes lock */
	writel(0x00000004, glb + CG_GLB_PON_CNTL);	/* __psds_csr_out_of_reset: psds_reg_rst_n 0->1 only */
	mdelay(1);
}

/*
 * Bring the PON-SerDes CMU/PLL up so it generates the PON APB register-bus clock
 * the GPON MAC lives on.  POW_PCIX alone is not enough: the MAC's clock is
 * derived from the SerDes line/CMU PLL, which must be given its rate (PSDS_MODE)
 * and its analog config (the ~266-row profile) BEFORE it is powered (POW_PCIX).
 * The PLL locks off the reference clock, so this works with no fiber / no DS
 * light — only the DS-RX lock (a later phase) needs actual light.
 * This is aal_psds_out_of_reset minus the RX-lock wait.
 */
static void cg_psds_init(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	u32 v;
	int i;

	/*
	 * aal_psds_init entry (GPON pon_mode, stock ca-ne.ko disasm 2026-07-15):
	 * raise PON_CNTL bit0 and hold it HIGH through the entire SerDes bring-up
	 * (the final PON_CNTL=0x30e release at the tail clears it back to the
	 * stock resting state), then __psds_csr_out_of_reset re-toggles
	 * psds_reg_rst_n (bit2) 1->0->1 with bit0 high.
	 */
	v = readl(cg->glb + CG_GLB_PON_CNTL) | BIT(0);
	writel(v, cg->glb + CG_GLB_PON_CNTL);
	writel(v & ~BIT(2), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);
	writel(v | BIT(2), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);

	/* __psds_mode_init: GPON rate — sd_s0=1, sds_mode_s0=0x8, usx=0 */
	writel(0x00000408, pon + CG_PSDS_MODE);
	udelay(10);

	/* __psds_prof_load: the CMU/PLL/CDR/TX-driver analog profile.  Each row is
	 * a direct write to the PSDS block (applied via its DATAIN/ACCESS pair). */
	for (i = 0; i < ARRAY_SIZE(cg_serdes_gpon); i++) {
		writel(cg_serdes_gpon[i].val, pon + cg_serdes_gpon[i].off);
		udelay(cg_serdes_gpon[i].delay_us ? cg_serdes_gpon[i].delay_us : 10);
	}

	/* __psds_disable_gpon_los_rst: hold EPON in reset + set the spare-cfg bit
	 * around the lock wait (GPON-only quirk). */
	v = readl(cg->glb + CG_GLB_EPON_CNTL);
	writel(v | BIT(0), cg->glb + CG_GLB_EPON_CNTL);		/* epon_rst_n = 1 */
	v = readl(pon + CG_PON_EPON_SPARE);
	writel(v | 0x80000000, pon + CG_PON_EPON_SPARE);

	/* __psds_ad_out_of_reset: power the SerDes -> PON APB clock runs.  Keep the
	 * laser burst-enable (ben_oen) OFF during the SerDes bring-up; it is set
	 * at the END of this function once the SerDes is stable (stock 0x30).
	 * Vendor delay is mdelay(1) — the settle is the poll below, not a fixed sleep. */
	v = readl(cg->glb + CG_GLB_PSDS_INIT);
	writel((v | CG_PSDS_POW_PCIX) & ~CG_PSDS_BEN_OEN, cg->glb + CG_GLB_PSDS_INIT);
	mdelay(1);

	/*
	 * __psds_sync: bounded wait for RX clock lock, continues on timeout
	 * exactly as the vendor does at boot.  Vendor budget is 1001 x 1 ms
	 * (stock ca-ne.ko aal_psds_out_of_reset) — the measured cold-boot lock
	 * latency is ~400-500 ms, so the old 100 ms budget timed out EVERY boot
	 * and the gearbox below got released with the RX still unlocked.  The
	 * vendor order is strict: lock FIRST, then los-rst release, THEN the
	 * gearbox reset release.
	 */
	for (i = 0; i < 1001; i++) {
		if ((readl(pon + CG_PSDS_RGB8) & 0x9c01) == 0x9c00)
			break;
		mdelay(1);
	}
	dev_info(cg->dev, "psds: __psds_sync RX-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(pon + CG_PSDS_RGB8));

	/* release GPON los-reset */
	v = readl(cg->glb + CG_GLB_EPON_CNTL);
	writel(v & ~BIT(0), cg->glb + CG_GLB_EPON_CNTL);	/* epon_rst_n = 0 */

	/*
	 * __psds_gbox_out_of_reset: toggle GLOBAL_PON_CNTL.pon_serdes_rst_n (bit1)
	 * 0->1 HERE, after the SerDes is powered AND its RX clock locked (vendor
	 * order — releasing the gearbox on an unlocked RX lets its elastic FIFO
	 * come up misaligned).  This gearbox connects the SerDes serial stream to
	 * the GPON MAC's parallel DS input -- without it the RX clock locks but
	 * zero downstream frames reach the MAC framer.  Vendor delays: 1 ms.
	 */
	v = readl(cg->glb + CG_GLB_PON_CNTL);
	writel(v & ~BIT(1), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);
	writel(v | BIT(1), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);

	/* __psds_gbox_init: rx/tx bit-ordering = 1 (reset default already 0x454) */
	v = readl(pon + CG_PSDS_GBOX_CTRL);
	v = (v & ~((0x3u << 4) | (0x3u << 6))) | (0x1u << 4) | (0x1u << 6);
	writel(v, pon + CG_PSDS_GBOX_CTRL);
	mdelay(1);

	/*
	 * aal_gpon_glb_ctrl_init (vendor: runs AFTER aal_psds_init, before intr
	 * setup): release the remaining PON-domain resets and give the GTC+ANI
	 * their 0->0x3 sync-reset edge NOW, with the SerDes powered and its
	 * clock LOCKED — the edge propagates and the DS framer starts from a
	 * deterministic state.  Doing this edge before POW_PCIX (the old
	 * cg_glb_reset tail) left the framer in a power-up lottery: same image,
	 * sometimes frame-sync, sometimes stuck O1/LOF.
	 *
	 * The RX lock (RGB8 bit15) arrives ~300 ms after the gearbox release —
	 * later than the 100 ms __psds_sync poll above — so re-wait for the full
	 * lock pattern HERE, bounded, so the edge really fires on a locked clock
	 * (measured cold boot 2026-07-15: edge at RGB8=0x1c00 (unlocked) still
	 * left the framer at O1/LOF; lock showed 0x19c00 ~300 ms later).
	 */
	for (i = 0; i < 2000; i++) {
		if ((readl(pon + CG_PSDS_RGB8) & 0x9c01) == 0x9c00)
			break;
		mdelay(1);
	}
	dev_info(cg->dev, "psds: pre-edge DS-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(pon + CG_PSDS_RGB8));
	writel(0x0000030e, cg->glb + CG_GLB_PON_CNTL);	/* pon_serdes/psds/ptp + puc/pdc */
	writel(0x00000003, cg->glb + CG_GLB_GPON_CNTL);	/* ani_rst_n + gpon_rst_n: the live-clock edge */
	mdelay(100);

	/*
	 * Drive the laser burst-enable output NOW, while the MAC is still
	 * disabled (BEN idles LOW, driven — no grants, no burst): stock runs
	 * with psds_init=0x30 (POW_PCIX + ben_oen) from the in-kernel aal PON
	 * init, long BEFORE userspace rtkbosa programs the GN25L95.  Our old
	 * order (ben_oen only at "the GO", after the BOSA init) left the BEN
	 * pin UNDRIVEN through the GN25L95 bring-up — a floating burst-enable
	 * can read as a stuck-on burst and latch the rogue-ONU TX fault
	 * (TX_CTL 0x6e bit7=1, zero TX bias) that kept the laser dark.
	 */
	v = readl(cg->glb + CG_GLB_PSDS_INIT);
	writel(v | CG_PSDS_BEN_OEN, cg->glb + CG_GLB_PSDS_INIT);
}

static bool cg_activate = true;
module_param_named(activate, cg_activate, bool, 0444);
MODULE_PARM_DESC(activate, "program the SN + start GPON ranging at probe (default on)");

static bool cg_do_bosa_init = true;
module_param_named(bosa_init, cg_do_bosa_init, bool, 0444);
MODULE_PARM_DESC(bosa_init, "program the GN25L95 BOSA laser driver over per_i2c before ranging (default on; off = no upstream burst, DS-side diagnostics only)");

/*
 * Enable the upstream laser.
 *
 * ★ Proven by the ours-vs-stock golden diff at Online (txpart-2026-07-16, see
 * the CG_PERGPIO_CFG0 block comment): the TX-disable net is pulled low by a
 * GPIO **group 0** pin route + drive, not by pin 34 (which only mirrors the
 * net as an input).  Ours held the GN25L95 in hardware TX-disable
 * (statusControl 0x6E bit7=1 constant, stock 0x00) because this setup was
 * missing.  Replicate stock's exact state, RMW-preserving unrelated bits:
 *   1. GLB +0x42c: set bit20 (route the laser-enable net);
 *   2. PER_GPIO0 OUT then CFG: pin6=HIGH, pin11=LOW, pin12=LOW, all three
 *      outputs (OUT first so pins 6/12 drive the correct level the moment
 *      CFG makes them outputs, and the already-output pin11 drops HIGH->LOW);
 *   3. pin 34 (group 1): GPIO input, mirroring the net (unchanged vs stock).
 * On ours' cold values this lands byte-for-byte on stock: cfg 0xFFFFF7FF ->
 * 0xFFFFE7BF, out 0x00000800 -> 0x00000040, 0x42c 0x01001101 -> 0x01101101.
 */
static void cg_laser_on(struct cortina_gpon *cg)
{
	u32 v;

	if (!cg->gpio)
		return;
	/* route the laser-enable net (stock 0x01101101; ours cold lacks bit20) */
	v = readl(cg->glb + CG_GLB_PINROUTE);
	writel(v | CG_PINROUTE_LASER, cg->glb + CG_GLB_PINROUTE);
	/* pin muxes to EXACT stock: group 0 = 0x1fff (ours' cold 0xffff has
	 * pins 13-15 wrongly GPIO), groups 3/4 = the stock-driven pin sets */
	writel(0x00001fff, cg->glb + CG_GLB_GPIO_MUX0);
	writel(0x000390ff, cg->glb + CG_GLB_GPIO_MUX3);
	writel(0x00003b00, cg->glb + CG_GLB_GPIO_MUX4);
	/* group-0 drive: pin6=HIGH, pin11=LOW, pin12=LOW ... */
	v = readl(cg->gpio + CG_PERGPIO_OUT0);
	writel((v & ~CG_GPIO0_LASER_PINS) | CG_GPIO0_PIN6, cg->gpio + CG_PERGPIO_OUT0);
	/* ... then make pins 6/11/12 outputs (cfg bit 0 = output) */
	v = readl(cg->gpio + CG_PERGPIO_CFG0);
	writel(v & ~CG_GPIO0_LASER_PINS, cg->gpio + CG_PERGPIO_CFG0);
	/* groups 3/4: whole-group stock state, OUT before CFG so each pin
	 * drives the correct level the moment it becomes an output */
	writel(CG_GPIO3_OUT_STOCK, cg->gpio + CG_PERGPIO_OUT3);
	writel(CG_GPIO3_CFG_STOCK, cg->gpio + CG_PERGPIO_CFG3);
	writel(CG_GPIO4_OUT_STOCK, cg->gpio + CG_PERGPIO_OUT4);
	writel(CG_GPIO4_CFG_STOCK, cg->gpio + CG_PERGPIO_CFG4);
	/* pin 34: leave the reset state.  Stock at Online has GLOBAL_GPIO_MUX_1
	 * = 0x00000000 (pin 34 NOT muxed to GPIO) and PER_GPIO1_CFG all-inputs
	 * — the live golden refutes the earlier "stock mux1 bit2=1" claim, so
	 * write nothing here (exact-stock).  PER_GPIO1_IN bit2 still serves as
	 * the net-level readback in /proc/gpon. */
}

/* One PDC map-memory entry write: DATA0/DATA1, then kick ACCESS, poll go. */
static int cg_pdc_map_write(struct cortina_gpon *cg, u32 idx, u32 d0, u32 d1)
{
	int i;

	writel(d0, cg->pon + CG_PDC_MAP_DATA0);
	writel(d1, cg->pon + CG_PDC_MAP_DATA1);
	writel(CG_TBL_GO | CG_TBL_WR | (idx & 0xff), cg->pon + CG_PDC_MAP_ACCESS);
	for (i = 0; i < 10000; i++) {
		if (!(readl(cg->pon + CG_PDC_MAP_ACCESS) & CG_TBL_GO))
			return 0;
	}
	dev_warn(cg->dev, "PDC map[%u] write timed out\n", idx);
	return -ETIMEDOUT;
}

/*
 * PDC init (vendor __pdc_gpon_family_init): route the DS GEMs.  Without this
 * the OMCC downstream GEM is de-encapsulated by the MAC (omci_port.en is
 * HW-latched) but the resulting frame has no destination — DS OMCI never
 * reaches the CPU and the OLT parks us Offline/"fail" with Received-OMCI=0.
 * Entries 0..7 (OMCC-reserved internal GEMs) -> CPU port 0, forwarding-engine
 * bypass, no-drop, cos 6, pol_id 0x80+idx (the 128..255 PON-DS policer bank);
 * entries 8..255 (data GEMs) -> L3_WAN, pol_id idx-8 (refined per-GEM at the
 * OMCI Create in Stage D).  Then PDC_CTRL: map-mem enable + the OMCI
 * high-priority override (omci_hp: cos 7, ldpid CPU_0) — expected readback
 * 0x02870002.  Runs once, after the puc/pdc reset release (PON_CNTL=0x30e at
 * the tail of cg_psds_init), before the MAC is enabled (vendor
 * __gpon_datapath_init order: ds_frame_thrsd -> pdc -> puc).
 */
static void cg_pdc_init(struct cortina_gpon *cg)
{
	u32 idx, d0, d1, ctrl;

	for (idx = 0; idx < CG_PDC_MAP_ENTRIES; idx++) {
		if (idx < CG_OMCC_US_GEM_IDX_NUM) {
			d0 = CG_PDC_D0_COS(6) | CG_PDC_D0_LDPID(CG_LPORT_CPU_0) |
			     CG_PDC_D0_LSPID(CG_LPORT_PON) |
			     CG_PDC_D0_FE_BYPASS | CG_PDC_D0_NO_DROP;
			d1 = CG_PDC_D1_POL_ID(idx + 0x80);
		} else {
			d0 = CG_PDC_D0_LDPID(CG_LPORT_L3_WAN) |
			     CG_PDC_D0_LSPID(CG_LPORT_PON);
			d1 = CG_PDC_D1_POL_ID(idx - 8);
		}
		if (cg_pdc_map_write(cg, idx, d0, d1))
			return;
	}

	ctrl = readl(cg->pon + CG_PDC_CTRL);
	ctrl &= ~CG_PDC_CTRL_HP_MASK;
	ctrl |= CG_PDC_CTRL_MAP_MEM_EN | CG_PDC_CTRL_HP_EN |
		(7 << CG_PDC_CTRL_HP_COS_SH) |
		(CG_LPORT_CPU_0 << CG_PDC_CTRL_HP_LDPID_SH);
	writel(ctrl, cg->pon + CG_PDC_CTRL);

	cg->pdc_ready = true;
	dev_info(cg->dev, "PDC: OMCC DS GEMs 0-7 -> CPU_0, ctrl=0x%08x\n",
		 readl(cg->pon + CG_PDC_CTRL));
}

/* PUC indirect-table op: kick ACCESS (go[31] + rbw[30]=write + index), poll go. */
static int cg_puc_ind_write(struct cortina_gpon *cg, u32 access_off, u32 index)
{
	int i;

	writel(CG_TBL_GO | CG_TBL_WR | index, cg->pon + access_off);
	for (i = 0; i < 10000; i++) {
		if (!(readl(cg->pon + access_off) & CG_TBL_GO))
			return 0;
	}
	dev_warn(cg->dev, "PUC indirect +0x%04x[%u] timed out\n", access_off, index);
	return -ETIMEDOUT;
}

/* One PUC per-VoQ valid bit (PUC_valid_voqN, 256-bit mask across 8 regs). */
static void cg_puc_voq_valid(struct cortina_gpon *cg, u32 voq, bool valid)
{
	u32 off = CG_PUC_VALID_VOQ0 - (voq / 32) * 4;
	u32 v = readl(cg->pon + off);

	if (valid)
		v |= BIT(voq % 32);
	else
		v &= ~BIT(voq % 32);
	writel(v, cg->pon + off);
}

/*
 * Program one PUC pvtbl entry (per-T-CONT VoQ map) + its 8 VoQs' back-
 * pressure remap and valid bits.  @ena gates the per-queue enable bit (bit 8
 * of each 9-bit voqN field) and the valid-VoQ mask; the entry itself is
 * always marked entryvld so the scheduler walks it.  voqN 9-bit fields are
 * bit-split across the 5 DATA words exactly as the vendor packs them;
 * schmode = 0 (strict priority), wrr weights 0.
 */
static int cg_puc_pvtbl_program(struct cortina_gpon *cg, u32 tcont, bool ena)
{
	void __iomem *pon = cg->pon;
	u32 voq[CG_PUC_QUEUE_PER_TCONT];
	u32 d0, d1, d2, q;

	for (q = 0; q < CG_PUC_QUEUE_PER_TCONT; q++)
		voq[q] = (q + tcont * CG_PUC_QUEUE_PER_TCONT) | ((u32)ena << 8);

	d0 = voq[0] | (voq[1] << 9) | (voq[2] << 18) |
	     ((voq[3] & 0x1f) << 27);
	d1 = ((voq[3] >> 5) & 0xf) | (voq[4] << 4) | (voq[5] << 13) |
	     (voq[6] << 22) | ((voq[7] & 1) << 31);
	d2 = ((voq[7] >> 1) & 0xff) | BIT(12);	/* schmode=0, entryvld=1 */

	writel(0, pon + CG_PUC_PVTBL_DATA4);
	writel(0, pon + CG_PUC_PVTBL_DATA3);
	writel(d2, pon + CG_PUC_PVTBL_DATA2);
	writel(d1, pon + CG_PUC_PVTBL_DATA1);
	writel(d0, pon + CG_PUC_PVTBL_DATA0);
	if (cg_puc_ind_write(cg, CG_PUC_PVTBL_ACCESS, tcont))
		return -ETIMEDOUT;

	for (q = 0; q < CG_PUC_QUEUE_PER_TCONT; q++) {
		u32 qid = q + tcont * CG_PUC_QUEUE_PER_TCONT;

		if (qid <= 63) {
			writel(qid & 0x7, pon + CG_PUC_VOQBPREMAP_DATA);
			if (cg_puc_ind_write(cg, CG_PUC_VOQBPREMAP_ACCESS, qid))
				return -ETIMEDOUT;
		}
		cg_puc_voq_valid(cg, qid, ena);
	}
	return 0;
}

/*
 * Flush one T-CONT's 8 VoQs (PUC_VOQFLUSH: start + openpktflushen + tcontid +
 * voqid, poll start self-clear) — the vendor aal_gpon_restore_tcont runs this
 * after every CAM re-install ("workaround", aal_puc_voq_flush_by_idx) so a
 * re-range doesn't burst frames queued before the link drop.  The vendor
 * additionally brackets each flush with a VoQ drop-enable + pvtbl disable;
 * ours flushes while gpon0's carrier is off (nothing enqueues), so the plain
 * flush+poll suffices.
 */
static void cg_puc_voq_flush(struct cortina_gpon *cg, u32 tcont)
{
	u32 q, v;
	int i;

	for (q = 0; q < CG_PUC_QUEUE_PER_TCONT; q++) {
		v = BIT(31) | BIT(16) | ((tcont & 0x1f) << 8) |
		    ((tcont * CG_PUC_QUEUE_PER_TCONT + q) & 0xff);
		writel(v, cg->pon + CG_PUC_VOQFLUSH);
		for (i = 0; i < 10000; i++) {
			if (!(readl(cg->pon + CG_PUC_VOQFLUSH) & BIT(31)))
				break;
			udelay(1);
		}
		if (i == 10000)
			dev_warn(cg->dev, "VoQ %u flush timed out\n",
				 tcont * CG_PUC_QUEUE_PER_TCONT + q);
	}
}

/*
 * PUC init (vendor aal_puc_init, GPON path) — the US admission plumbing that
 * connects the CPU-injected DMA-LSO frame to the OMCC T-CONT / GEM-US burst.
 * Without it the DMA-LSO ring drains but the frame lands in an unmapped,
 * invalid VoQ and is dropped -> the OLT never receives our OMCI reply and
 * loops its Get.  Run once, right after the PDC (vendor __gpon_datapath_init
 * order), entirely in the PON+0x8000 sub-block (does not touch the MAC or the
 * Ethernet datapath).  8Q VoQ mode: VoQID = {HdrA.ldpid[3:0], HdrA.cos[2:0]}.
 * Only T-CONT 0 (the OMCC) has its 8 VoQs enabled; the CPU high-priority OMCI
 * inject additionally uses the "9th queue" VoQ 127 (ldpid 0xf, cos 7).
 */
static void cg_puc_init(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	u32 tcont, q, v;

	/* clear the PUC interrupt-enable (vendor: PUC_PONCNTL_INTENABLE = 0) */
	writel(0, pon + CG_PUC_PONCNTL_INTEN);

	/* PUCCFG: inccfg=2 (clear-on-read), crccntl=2 (regenerate US CRC),
	 * invalid_voqdrop_enable=1 (drop frames that hit an invalid VoQ) */
	v = readl(pon + CG_PUC_PUCCFG);
	v = (v & ~(GENMASK(18, 16) | GENMASK(1, 0))) | (2u << 16) | 2u;
	v |= BIT(30);
	writel(v, pon + CG_PUC_PUCCFG);

	/* VoQ buffer limits (GPON scfg VOQBUFLIMIT A/B/C) + per-VoQ limit-select
	 * (below 8 queues use A, 8..16 use B, >16 use C; all 256 -> A) */
	writel(0x7a0, pon + CG_PUC_VOQBUFLIMIT_A);
	writel(0x3b0, pon + CG_PUC_VOQBUFLIMIT_B);
	writel(0x200, pon + CG_PUC_VOQBUFLIMIT_C);
	for (q = 0; q < CG_PUC_VOQBUFLIMSEL_N; q++)
		writel(0x55555555, pon + CG_PUC_VOQBUFLIMSEL0 + q * 4);

	/* VoQ map mode = 8Q (voqmapsel = 0) */
	writel(0, pon + CG_PUC_VOQMAPCFG);

	/*
	 * pvtbl: per-T-CONT VoQ map.  Only T-CONT 0 (OMCC) has queues enabled;
	 * every entry is marked valid (entryvld) so the scheduler walks it.
	 * voqN 9-bit field = queue_id | (enable << 8), bit-split across the 5
	 * DATA words exactly as the vendor packs it.  schmode=0 (strict), SP
	 * weights (wrr=0).  Also program the back-pressure remap (queue_id<=63:
	 * tqmvoqid = queue_id & 7) and the per-VoQ valid bit.
	 */
	for (tcont = 0; tcont < CG_PUC_TCONT_NUM; tcont++)
		if (cg_puc_pvtbl_program(cg, tcont, tcont == 0))
			return;
	/* the CPU high-priority OMCI inject rides the 9th queue (VoQ 127) */
	cg_puc_voq_valid(cg, CG_PUC_9TH_QUEUE_VOQ, true);

	/*
	 * US OMCI header-A replacement: for an OMCI control frame (matched by
	 * the GLOBAL_LNK_TYPE 0xfff1, HW reset default) the PUC stamps the OMCC
	 * GEM index + CoS onto the upstream frame.  Normal: enable_replacement,
	 * gemid=6, cos=6.  High-priority (the 9th-queue inject): gemid=7, cos=7.
	 * us_ext_omci_en + us_hdr_min_size=30 accepts extended (>=14B) OMCI.
	 */
	writel(BIT(31) | (6u << 8) | 6u, pon + CG_PUC_US_OMCI_HDR_A);
	writel((7u << 8) | 7u, pon + CG_PUC_US_OMCI_HP_HDR_A);
	v = readl(pon + CG_PUC_GLOBAL_PLOAM_CFG);
	v = (v & ~GENMASK(21, 16)) | (30u << 16) | BIT(31);
	writel(v, pon + CG_PUC_GLOBAL_PLOAM_CFG);

	/* back-pressure: drop off, bp on, threshold 0x100 */
	v = readl(pon + CG_PUC_BPCNTL);
	v = (v & ~(BIT(4) | GENMASK(30, 16))) | BIT(0) | (0x100u << 16);
	writel(v, pon + CG_PUC_BPCNTL);

	/* BTC (GPON): pfovrhd=5, schmode=FRAGMENT(0), wdaligned=0,
	 * minrmnwindowsz=5, sch2en=1, lrgfrmfragen=1 (segment >4095B frames) */
	v = readl(pon + CG_PUC_BTCCFG);
	v = (v & ~GENMASK(5, 0)) | 5u;
	v &= ~(BIT(8) | BIT(12));
	v |= BIT(16) | BIT(25);
	v = (v & ~GENMASK(31, 27)) | (5u << 27);
	writel(v, pon + CG_PUC_BTCCFG);

	/* QM<->PUC report-adjust levels (GPON) */
	writel(0x00c80000, pon + CG_PUC_Q2PQSRCFG01);	/* lv0=0, lv1=0xc8 */
	writel(0x05c201b8, pon + CG_PUC_Q2PQSRCFG23);	/* lv2=0x1b8, lv3=0x5c2 */

	/* PUC<->US-scheduler interface: cntr clear-on-read */
	writel(CG_XGPN_PUCIF_CTRL_VAL, pon + CG_XGPN_PUCIF_CTRL);

	/* aggregate shaper + PIR (rate limiter off) */
	v = readl(pon + CG_PUC_CTRL);
	v = (v | BIT(30)) & ~BIT(26);	/* shp_en=1, rl_en=0 */
	writel(v, pon + CG_PUC_CTRL);
	writel(20u | (20u << 5) | (20u << 10), pon + CG_PUC_CTRL1);
	v = readl(pon + CG_PUC_CTRL2);
	v = (v & ~GENMASK(4, 0)) | 20u | BIT(26);	/* pirovhd=20, pir_en=1 */
	writel(v, pon + CG_PUC_CTRL2);

	cg->puc_ready = true;
	dev_info(cg->dev,
		 "PUC: OMCC T-CONT0 VoQs + 9th-queue enabled, puccfg=0x%08x pucif=0x%08x\n",
		 readl(pon + CG_PUC_PUCCFG), readl(pon + CG_XGPN_PUCIF_CTRL));
}

/*
 * Program the GPON MAC identity + datapath, then start the activation FSM.  The
 * G.984.3 O1->O5 ranging runs in HARDWARE: once the serial number is programmed
 * and onu_ctl.en is set, the MAC autonomously transmits its Serial_Number PLOAM,
 * answers Assign_ONU-ID / Ranging_Time, and advances onu.state to O5.  Software
 * only configures + polls.  Serial number / config MUST be written while en=0.
 * (aal_gpon __gpon_common_init + aal_pon_mac_enable_set, GPON branch.)
 */
static void cg_mac_activate(struct cortina_gpon *cg)
{
	void __iomem *mac = cg->mac;
	u32 v;
	int i;

	/*
	 * The PON/GPON reset release (PON_CNTL=0x30e, GPON_CNTL=0x3) happened at
	 * the end of cg_psds_init(), after SerDes lock (aal_gpon_glb_ctrl_init
	 * order).  Do NOT re-write it here: a second write is at best a no-op
	 * and at worst a mid-FSM disturb.
	 */

	/*
	 * De-assert the laser TX-disable net BEFORE programming the BOSA: on
	 * stock the GPIO pin-route/drive that pulls the net low is already set
	 * up when rtkbosa runs (its on-wire init trace reads TX_CTL 0x6e with
	 * bit7 CLEAR), so the GN25L95's safe-mode start executes with TX_DIS
	 * de-asserted.  Running the init with TX_DIS still asserted (our old
	 * order: laser_on last) left 0x6e bit7=1 (TX fault / not lasing) and
	 * the OLT saw zero upstream energy despite every GPIO register
	 * byte-matching stock afterwards — the ORDER is part of the sequence.
	 */
	cg_laser_on(cg);
	mdelay(10);	/* let the TX_DIS net settle before the i2c stream */

	/*
	 * Program the external GN25L95 BOSA laser driver over per_i2c (bias/
	 * mod/APD DAC tables, alarm thresholds, TX gate) BEFORE the ranging
	 * FSM starts: out of power-on reset the BOSA is unprogrammed, the
	 * upstream laser never bursts and the OLT reports "Laser out".  This
	 * reproduces the stock boot's rtkbosa init in-kernel.
	 */
	if (cg_do_bosa_init) {
		if (cg_bosa_init(cg->dev))
			dev_warn(cg->dev, "BOSA init failed - upstream laser will not burst\n");
	} else {
		dev_info(cg->dev, "BOSA init SKIPPED (bosa_init=0) - upstream laser will not burst\n");
	}

	/* --- config while en=0 (serial number is range-critical) --- */
	writel(CG_ONU_CFG_VAL, mac + CG_REG_ONU_CFG_REAL);	/* laser_on_align=0x12, pre_bias=18 */
	writel(0x58504f4e, mac + CG_REG_VENDOR);	/* vendor-id "XPON" */
	writel(0x5c6cafcb, mac + CG_REG_VENDOR_SPEC);	/* VSSN 5C6CAFCB */
	/* datapath: gpon_ds.max_packet_size (bits 29:16) = 0x3FFF */
	v = readl(mac + CG_REG_GPON_DS);
	v = (v & ~(0x3fffu << 16)) | (0x3fffu << 16);
	writel(v, mac + CG_REG_GPON_DS);
	/* SF/SD BER-alarm thresholds + BER interval (stock 0x6532 -> +0x10,
	 * 0x13880 -> +0xf0): REMOVED for the O1-stuck bisect 2026-07-15 --
	 * restoring the exact 2026-07-13 working write-set.  Re-add only after
	 * ranging is proven again, one write per boot. */
	/* PDC: route the OMCC DS GEMs to the CPU (Stage B — vendor
	 * __gpon_datapath_init runs it right after the DS max_packet_size,
	 * before the MAC enable).  Safe pre-range: it only writes the PDC
	 * sub-block (+0x9000), not the MAC. */
	cg_pdc_init(cg);
	/* PUC (US-side): the CPU-inject OMCI admission -> OMCC T-CONT/GEM-US
	 * burst.  Vendor __gpon_datapath_init runs aal_puc_init right after the
	 * PDC.  Isolated to the PON+0x8000 sub-block; safe pre-range. */
	cg_puc_init(cg);
	/* password / AES keys: deferred (not needed to range) */

	/* Wait for the downstream to lock (RGB8 bit15 BER_NOTIFY) before enabling
	 * ranging, so the FSM sees a live downstream at the moment en is asserted. */
	for (i = 0; i < 8000; i++) {
		if ((readl(cg->pon + CG_PSDS_RGB8) & 0x9c01) == 0x9c00)
			break;
		mdelay(1);
	}
	dev_info(cg->dev, "activate: DS-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(cg->pon + CG_PSDS_RGB8));

	/* --- the GO --- */
	/* (ben_oen was set at the end of cg_psds_init, stock order; HW gates the
	 * actual burst per grant — onu_cfg.laser_on stays 0 -> burst, not CW) */
	/* onu_ctl.en -> HW starts ranging O1->O5.  onu_ctl/onu_cfg live at the SILICON
	 * offsets +0x134/+0x138 (a +0x20 shift vs the rtl8277c header's +0x114/+0x118,
	 * above offset 0x100 only), proven by live devmem on stock: 0x134=0x00460262
	 * and 0x138=0x12100900 hold the onu_ctl/onu_cfg bit patterns, while 0x114/0x118
	 * are the DS-PLOAM RX FIFO regs (PLOAMD_FF_CTL / PLOAMD_FIFO3).  onu_cfg is
	 * written at +0x138 above; here just assert onu_ctl.en at +0x134.
	 * We used to ALSO poke +0x114/+0x118 to settle the ambiguity -- those writes
	 * corrupted the DS-PLOAM RX FIFO so the MAC never processed the OLT's ranging
	 * PLOAMs and the FSM stalled at O1.  Removed (live-verified 2026-07-13). */
	writel(CG_ONU_CTL_VAL, mac + CG_REG_ONU_CTL);	/* onu_ctl.en @ +0x134 */
	/*
	 * Freeze the SN random-delay engine (vendor aal_pon_mac_enable_set does
	 * this right after aal_gpon_active_set: "stop random delay calculation
	 * when enable GPON MAC").  Left running (the reset default), the delay
	 * of the Serial_Number burst churns every frame and the OLT never
	 * decodes our SN -> "Laser out"/no admit.  Clear pti_omci with it
	 * (vendor __gpon_common_init).
	 */
	v = readl(mac + CG_REG_GPON_MAC_CTRL);
	writel(v & ~(CG_MAC_CTRL_SW_RANDOM_EN | CG_MAC_CTRL_PTI_OMCI),
	       mac + CG_REG_GPON_MAC_CTRL);
	/* (the laser TX-disable net was de-asserted before the BOSA init above) */
}

/*
 * us.frame_var: compensate the US burst position for the OLT's
 * Extended_Burst_Length (DS PLOAM msg 0x14; the MAC latches the type-3
 * preamble lengths into t3_preamble).  The stock __intr_handler recomputes
 * this on every received DS PLOAM (GPON mode):
 *     frame_var = 0x200 - ((pre_range + ranged + 0x20) & 0xff)
 * With this OLT (0x78/0x78) that is 0x1F0 — the live stock value at Online.
 * At the reset value (0) the extended-burst overhead is not accounted for,
 * the SN/US burst is misaligned in the grant window, and the OLT cannot hear
 * the ONU.  eqd_select(16) stays 0 (main EqD in use).
 */
static void cg_frame_var_update(struct cortina_gpon *cg)
{
	u32 t3 = readl(cg->mac + CG_REG_T3_PREAMBLE);
	u32 pre = t3 & 0xff, ranged = (t3 >> 8) & 0xff;
	u32 us, fv;

	if (!pre || !ranged)
		return;		/* Extended_Burst_Length not received yet */
	fv = (0x200 - ((pre + ranged + 0x20) & 0xff)) & 0x1ff;
	us = readl(cg->mac + CG_REG_US);
	if ((us & 0x1ff) == fv)
		return;
	writel((us & ~0x1ffu) | fv, cg->mac + CG_REG_US);
	dev_info(cg->dev, "us.frame_var = 0x%03x (t3_preamble 0x%08x)\n", fv, t3);
}

static inline u32 cg_mac_rd(struct cortina_gpon *cg, u32 off)
{
	return readl(cg->mac + off);
}

/*
 * NOTE: do NOT read the TX-PLOAM MIB (indirect ACCESS/DATA pair at +0x184/
 * +0x188) from the driver.  Kicking that engine (go-bit write + busy-poll)
 * during activation wedges the PON PLOAM engine and pins the FSM at
 * O1-Initial (same class as Board C's pi_rd hanging the FSM softirq; the
 * vendor never touches it during activation).  If a count is ever needed,
 * take a ONE-SHOT devmem read from userspace on a settled O5, never a
 * driver loop.
 */

/* ------------------------------------------------------------------------- */
/* Post-O5 servicing: interrupt handler + FSM tracker + OMCC channel bind.    */
/*                                                                            */
/* The MAC ranges O1->O5 autonomously and auto-ACKs all mandatory PLOAM (no   */
/* SW DS-PLOAM parsing needed); the PLOAM outcomes software must act on       */
/* surface as INTERRUPT (0x8c) sources:                                       */
/*   ONU_ST_CHG(31) FSM moved     -> track state, link up/down, dpath reset   */
/*   ONU_ID(30)     onu-id given  -> bind OMCC T-CONT[0] to alloc = onu-id    */
/*   PORTID(17)     omci port set -> bind us-gem 0..7 to omci_port.id         */
/*   KSW(19)/DACT(8)              -> logged (AES rekey + full dpath reset =   */
/*                                   next phase)                              */
/* (vendor aal_gpon_intr.c __intr_handler / ca_pon.c __pon_isr)               */
/* ------------------------------------------------------------------------- */

static const char *const cg_state_name[8] = {
	"O1-Initial", "O2-Standby", "O3-SerialNumber", "O4-Ranging",
	"O5-Operation", "O6-POPUP", "O7-EmergencyStop", "unknown",
};

/*
 * One indirect table transaction: kick ACCESS with go(bit31) [+ wr(bit30) +
 * index/alloc in cmd], poll go self-clear.  Bounded at 10000 reads like the
 * vendor __CHECK_INDIRCT_OPERATE_STATE (completes in a few bus reads).
 * Runs only in the single-threaded work context, so no lock is needed yet.
 */
static int cg_tbl_op(struct cortina_gpon *cg, u32 access_reg, u32 cmd)
{
	int i;

	writel(CG_TBL_GO | cmd, cg->mac + access_reg);
	for (i = 0; i < 10000; i++) {
		if (!(readl(cg->mac + access_reg) & CG_TBL_GO))
			return 0;
	}
	dev_warn(cg->dev, "indirect access +0x%03x cmd 0x%08x timed out\n",
		 access_reg, cmd);
	return -ETIMEDOUT;
}

/*
 * Bind the OMCC to the T-CONT table: entry[alloc-id = onu-id] -> hw T-CONT 0
 * with omci_en + ploam_en (the G.984.3 default alloc-id == onu-id carries the
 * OMCC).  Read-modify-write of the indirect entry, exactly the vendor
 * aal_gpon_omcc_tcont_enable -> aal_gpon_tcont_set sequence.
 */
static void cg_omcc_tcont_bind(struct cortina_gpon *cg, u32 alloc_id)
{
	u32 d;

	if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, alloc_id & 0xfff))
		return;
	d = readl(cg->mac + CG_REG_TCONT_DATA);
	d &= ~CG_TCONT_INDEX_MASK;			/* index = 0 (OMCC T-CONT) */
	d |= CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN;
	writel(d, cg->mac + CG_REG_TCONT_DATA);
	if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, CG_TBL_WR | (alloc_id & 0xfff)))
		return;

	cg->omcc_alloc = alloc_id & 0xfff;
	dev_info(cg->dev, "OMCC: T-CONT[0] bound to alloc-id %u\n", cg->omcc_alloc);
}

/*
 * Bind the OMCC upstream GEM: us-gem hw indices 0..7 are reserved for the
 * OMCC; point them all at the OLT-assigned omci_port.id (vendor
 * aal_gpon_omcc_gem_enable -> aal_gpon_us_gem_port_set).
 */
static void cg_omcc_gem_bind(struct cortina_gpon *cg, u32 gem_id)
{
	u32 idx, d;

	for (idx = 0; idx < CG_OMCC_US_GEM_IDX_NUM; idx++) {
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, idx))
			return;
		d = readl(cg->mac + CG_REG_US_PORT_DATA);
		d = (d & ~0xfffu) | (gem_id & 0xfff);
		writel(d, cg->mac + CG_REG_US_PORT_DATA);
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, CG_TBL_WR | idx))
			return;
	}

	cg->omcc_gem = gem_id & 0xfff;
	dev_info(cg->dev, "OMCC: us-gem 0..%d bound to GEM port-id %u\n",
		 CG_OMCC_US_GEM_IDX_NUM - 1, cg->omcc_gem);
}

/* DS GEM CAM: entry[GEM port-id] -> {intern gem index, vld} (vendor
 * aal_gpon_ds_gem_port_set; the aes bit is set later on Encrypted_Port-ID) */
static int cg_ds_gem_bind(struct cortina_gpon *cg, u32 gem_id, u32 idx)
{
	if (cg_tbl_op(cg, CG_REG_DS_GEM_ACCESS, gem_id & 0xfff))
		return -ETIMEDOUT;
	writel(CG_DS_GEM_VLD | CG_DS_GEM_INDEX(idx), cg->mac + CG_REG_DS_GEM_DATA);
	return cg_tbl_op(cg, CG_REG_DS_GEM_ACCESS, CG_TBL_WR | (gem_id & 0xfff));
}

/*
 * Stage D — install the OLT-provisioned WAN data path (idempotent; runs in
 * the isr_work context so the indirect-table ops never race the OMCC binds).
 * Needs the OMCC up (O5 + omci_port latched) and both halves of the OLT's
 * provisioning snooped: the data alloc-id (ME 262) and the data GEM port-id
 * (ME 268).  Everything is derived from those two values:
 *   - T-CONT CAM[alloc]  -> hw T-CONT 1 (omci_en + ploam_en, vendor-faithful)
 *   - US_PORT[8..15]     -> data GEM (VoQ==intern-gem-idx, 8Q map; the CPU
 *                           injects on VoQ 8, binding all 8 queues is free)
 *   - DS GEM CAM[gem]    -> intern idx 8;  CAM[4095 broadcast] -> idx 9
 *   - PDC map[8]/map[9]  -> CPU port 0 (fe_bypass, no_drop; lspid = PON so
 *                           the NI CPU-RX delivers to gpon0)
 *   - PUC pvtbl[1] + valid VoQs 8..15, then the vendor voq_flush workaround
 */
static void cg_data_try_install(struct cortina_gpon *cg)
{
	u32 alloc = READ_ONCE(cg->dt_alloc);
	u32 gem = READ_ONCE(cg->dg_gem);
	u32 d, i;

	if (cg->data_installed || !cg->omcc_up || !alloc || !gem)
		return;

	if (alloc == cg->omcc_alloc) {
		/* single-alloc OLT: rebinding the CAM would steal the OMCC's
		 * T-CONT (proven 9602C regression).  Leave the CAM alone and
		 * warn — the data path then needs the ride-the-OMCC variant. */
		dev_warn(cg->dev,
			 "data alloc %u == OMCC alloc: NOT rebinding CAM (single-alloc OLT?)\n",
			 alloc);
	} else {
		if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, alloc & 0xfff))
			return;
		d = readl(cg->mac + CG_REG_TCONT_DATA);
		d &= ~CG_TCONT_INDEX_MASK;
		d |= CG_TCONT_INDEX(CG_DATA_TCONT_IDX) |
		     CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN;
		writel(d, cg->mac + CG_REG_TCONT_DATA);
		if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, CG_TBL_WR | (alloc & 0xfff)))
			return;
	}

	/* US: every VoQ of the data T-CONT stamps the data GEM port-id */
	for (i = 0; i < CG_PUC_QUEUE_PER_TCONT; i++) {
		u32 idx = CG_DATA_GEM_IDX + i;

		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, idx))
			return;
		d = readl(cg->mac + CG_REG_US_PORT_DATA);
		d = (d & ~0xfffu) | (gem & 0xfff);
		writel(d, cg->mac + CG_REG_US_PORT_DATA);
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, CG_TBL_WR | idx))
			return;
	}

	/* DS: unicast data GEM + the broadcast GEM (DHCP OFFER rides it) */
	if (cg_ds_gem_bind(cg, gem, CG_DATA_GEM_IDX) ||
	    cg_ds_gem_bind(cg, CG_MCAST_GEM_ID, CG_MCAST_GEM_IDX))
		return;

	/* PDC: both intern indices -> CPU port 0, forwarding-engine bypass,
	 * lspid = PON (the NI CPU-RX WAN-delivery key) */
	for (i = 0; i < 2; i++) {
		u32 idx = CG_DATA_GEM_IDX + i;

		if (cg_pdc_map_write(cg, idx,
				     CG_PDC_D0_COS(0) |
				     CG_PDC_D0_LDPID(CG_LPORT_CPU_0) |
				     CG_PDC_D0_LSPID(CG_LPORT_PON) |
				     CG_PDC_D0_FE_BYPASS | CG_PDC_D0_NO_DROP,
				     CG_PDC_D1_POL_ID(idx)))
			return;
	}

	/* PUC: enable the data T-CONT's VoQs, then the flush workaround */
	if (cg_puc_pvtbl_program(cg, CG_DATA_TCONT_IDX, true))
		return;
	cg_puc_voq_flush(cg, CG_DATA_TCONT_IDX);

	cg->data_installed = true;
	dev_info(cg->dev,
		 "DATA path UP: alloc %u -> T-CONT %u, gem %u (US VoQ %u, DS idx %u), bcast %u -> idx %u\n",
		 alloc, CG_DATA_TCONT_IDX, gem, CG_DATA_GEM_IDX,
		 CG_DATA_GEM_IDX, CG_MCAST_GEM_ID, CG_MCAST_GEM_IDX);
	if (cg->wan_ndev)
		netif_carrier_on(cg->wan_ndev);
}

/*
 * Datapath reset on O5 exit — STUB for this phase.  The vendor
 * aal_gpon_datapath_reset / drain_out flushes the GEM/T-CONT datapath so a
 * re-range doesn't burst stale frames; ours will do that when the data-plane
 * GEM/T-CONT phase lands.  For now just drop the OMCC-link state so the next
 * O5 re-binds cleanly.
 */
static void cg_datapath_reset_stub(struct cortina_gpon *cg)
{
	cg->omcc_up = false;
	/* disarm the responder: the next O5 re-inits it with a fresh MIB
	 * (the OLT re-provisions after a deact/re-range) */
	spin_lock_bh(&cg->omci_lock);
	cg->omci_active = false;
	spin_unlock_bh(&cg->omci_lock);
	cancel_delayed_work(&cg->veip_avc_work);
	/* data path down; the dt_/dg_ shadow SURVIVES so the O5 re-entry
	 * re-installs even when the OLT does not re-provision (LOS re-range) */
	cg->data_installed = false;
	if (cg->wan_ndev)
		netif_carrier_off(cg->wan_ndev);
	dev_warn(cg->dev, "O5 exit: datapath reset (OMCC + data down, shadow kept)\n");
}

/* Try to bring the OMCC link up: needs O5 + HW-filled omci_port.en. */
static void cg_omcc_try_up(struct cortina_gpon *cg, u8 state)
{
	u32 omci_port;

	if (cg->omcc_up || state != CG_STATE_OPERATION)
		return;
	omci_port = cg_mac_rd(cg, CG_REG_OMCI_PORT);
	if (!(omci_port & CG_OMCI_PORT_EN)) {
		dev_info(cg->dev, "O5 but omci_port not enabled yet (0x%08x)\n",
			 omci_port);
		return;
	}
	cg_omcc_gem_bind(cg, CG_OMCI_PORT_ID(omci_port));
	cg->omcc_up = true;
	dev_info(cg->dev, "OMCC link UP (alloc %u, gem %u) - ready for OMCI\n",
		 cg->omcc_alloc, cg->omcc_gem);

	/* Stage C: arm the G.988 responder on a fresh MIB.  SN = the PLOAM
	 * identity programmed into the MAC ("XPON" + VSSN 5C6CAFCB).  The
	 * MIB-Data-Sync seed 200 is a POISON: it must NOT match the OLT's
	 * stored lsync, so its ME2 audit mismatches and it re-provisions from
	 * MIB-Reset (the X111W warm-readmit lesson; the on-wire MIB-Reset
	 * then zeroes it).  Also start the ~31s VEIP oper-up AVC timer. */
	if (cg->omci) {
		static const u8 sn[8] = {
			'X', 'P', 'O', 'N', 0x5c, 0x6c, 0xaf, 0xcb
		};

		spin_lock_bh(&cg->omci_lock);
		omci_onu_init(cg->omci, sn, 200);
		cg->omci_active = true;
		spin_unlock_bh(&cg->omci_lock);
		schedule_delayed_work(&cg->veip_avc_work, 31 * HZ);
		dev_info(cg->dev, "OMCI responder armed (%u MIB rows, mds seed 200)\n",
			 cg->omci->nrows);
	}
}

/*
 * US OMCI TX (Stage C): the 48-byte PDU (trailer + MIC already stamped by
 * the responder) goes out the NI DMA-LSO ring; the HW GEM-encapsulates it
 * onto the OMCC upstream on the next matching BWmap grant.
 */
static void cg_omci_tx(struct cortina_gpon *cg, const u8 *pdu48)
{
	int ret = -ENODEV;

	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		ret = cortina_ni_pon_tx(pdu48, OMCI_LEN);
	if (ret) {
		cg->omci_tx_fail++;
		dev_warn_ratelimited(cg->dev, "US OMCI TX failed (%d)\n", ret);
	} else {
		cg->omci_tx++;
	}
}

/* The ~31s post-O5 VEIP (ME 329) operational-up AVC: the OLT waits for it
 * before marking the service matched/active (its Match State stays Initial
 * until the ONU reports the WAN egress port up). */
static void cg_veip_avc_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon,
					       veip_avc_work);
	u8 frame[OMCI_LEN];
	bool emit = false;

	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_active && !cg->omci->avc_veip_up_sent) {
		omci_onu_emit_veip_up_avc(cg->omci, frame);
		emit = true;
	}
	spin_unlock_bh(&cg->omci_lock);
	if (emit) {
		cg_omci_tx(cg, frame);
		dev_info(cg->dev, "VEIP oper-up AVC emitted (~31s post-O5)\n");
	}
}

/*
 * DS OMCI receive: the NI CPU-RX hook hands us each OMCI PDU (the 16-byte
 * PON control header already stripped).  Decode-log (Stage B) + answer with
 * the G.988 responder and TX the reply upstream (Stage C).  Runs in NAPI
 * softirq context: no sleeping; the responder context is spinlocked against
 * the isr_work/AVC-work writers.
 *
 * Baseline OMCI PDU layout (G.988, all big-endian byte math):
 *   [0:1] TCI    [2] msg-type {AR=bit6, AK=bit5, MT=bits4:0}
 *   [3]   device-id (0x0A = baseline)
 *   [4:5] ME class    [6:7] ME instance
 *   [8:39] contents   [40:47] trailer (incl. the 4-byte MIC/CRC)
 */
static void cg_rx_omci(const u8 *pdu, unsigned int len)
{
	struct cortina_gpon *cg = READ_ONCE(cg_singleton);
	static const char *const mt_name[32] = {
		[4] = "Create", [5] = "Delete", [8] = "Set", [9] = "Get",
		[11] = "Get-all-alarms", [12] = "Get-all-alarms-next",
		[13] = "MIB-upload", [14] = "MIB-upload-next",
		[15] = "MIB-reset", [16] = "Alarm", [17] = "AVC", [18] = "Test",
		[19] = "Start-SW-dl", [20] = "DL-section", [21] = "End-SW-dl",
		[22] = "Activate-SW", [23] = "Commit-SW", [24] = "Sync-time",
		[25] = "Reboot", [26] = "Get-next", [27] = "Test-result",
		[28] = "Get-current-data", [29] = "Set-table",
	};
	const char *name;
	u8 mt;

	if (!cg)
		return;
	if (len < 8) {
		cg->omci_rx_short++;
		return;
	}
	cg->omci_rx++;

	mt = pdu[2];
	name = mt_name[mt & 0x1f] ? mt_name[mt & 0x1f] : "?";
	/* log the first PDUs + then 1-in-64 (the MIB-upload walk is chatty) */
	if (cg->omci_rx <= 24 || !(cg->omci_rx & 63))
		dev_info(cg->dev,
			 "DS OMCI #%u: len=%u tci=0x%02x%02x mt=%u(%s)%s%s dev=0x%02x me=%u/%u\n",
			 cg->omci_rx, len, pdu[0], pdu[1], mt & 0x1f, name,
			 (mt & BIT(6)) ? " AR" : "", (mt & BIT(5)) ? " AK" : "",
			 pdu[3], (pdu[4] << 8) | pdu[5], (pdu[6] << 8) | pdu[7]);

	/* DS MIC self-check on the first PDUs: decides the CRC-32 convention
	 * against live OLT frames — the same convention our US MIC must use.
	 * be = I.363.5/AAL5 (~crc32_be, the G.984.4 spec form, what the
	 * responder emits); le = reflected zlib (what the 9602C SW path used).
	 * Diagnostic only. */
	if (len >= OMCI_LEN && cg->omci_ds_crc_ok + cg->omci_ds_crc_bad < 16) {
		u32 want = ((u32)pdu[44] << 24) | ((u32)pdu[45] << 16) |
			   ((u32)pdu[46] << 8) | pdu[47];
		u32 be = ~crc32_be(~0u, pdu, 44);
		u32 le = crc32_le(~0u, pdu, 44) ^ ~0u;

		if (be == want)
			cg->omci_ds_crc_ok++;
		else
			cg->omci_ds_crc_bad++;
		if (cg->omci_rx <= 4)
			dev_info(cg->dev, "DS OMCI MIC self-check: %s (want %08x be %08x le %08x)\n",
				 be == want ? "AAL5-BE" :
				 (le == want ? "ZLIB-LE" : "NEITHER"),
				 want, be, le);
	}

	/* ---- Stage D: snoop the data-path-defining MEs (the responder still
	 * answers them; the driver additionally installs the HW tables). ---- */
	{
		u16 class_id = ((u16)pdu[4] << 8) | pdu[5];
		u16 inst = ((u16)pdu[6] << 8) | pdu[7];
		u8 m = mt & 0x1f;
		bool cfg = (m == 4 || m == 8 || m == 6);	/* Create/Set/Delete */

		/* body dump of the datapath/bridging MEs (bounded budget) —
		 * the live source of truth for what THIS OLT provisions */
		if (cfg && cg->omci_cfg_log < 48 && len >= 24) {
			switch (class_id) {
			case 45: case 47: case 84: case 130: case 171:
			case 262: case 266: case 268: case 277: case 280:
			case 281: case 309: case 329:
				cg->omci_cfg_log++;
				dev_info(cg->dev,
					 "OMCI cfg mt=%u me=%u/0x%04x body=%*phN\n",
					 m, class_id, inst, 16, pdu + 8);
				break;
			}
		}

		/* ME 262 T-CONT: the data alloc-id.  Set carries {mask[8:9],
		 * alloc[10:11] when attr-1 bit set}; a Create's SBC body has
		 * alloc first.  The OMCC alloc (= onu-id) never comes here. */
		if (class_id == 262 && len >= 12) {
			u32 alloc = 0;

			if (m == 8 && (((pdu[8] << 8) | pdu[9]) & 0x8000))
				alloc = ((u16)pdu[10] << 8) | pdu[11];
			else if (m == 4)
				alloc = ((u16)pdu[8] << 8) | pdu[9];
			if (alloc && alloc != 0xffff && alloc != cg->dt_alloc) {
				WRITE_ONCE(cg->dt_alloc, alloc);
				cg->dt_inst = inst;
				dev_info(cg->dev,
					 "OMCI: data T-CONT me-inst 0x%04x alloc-id %u\n",
					 inst, alloc);
				schedule_work(&cg->isr_work);
			}
		}

		/* ME 268 GEM-port-network-CTP Create: SBC body = port-id[0:1],
		 * T-CONT ptr[2:3], direction[4] (1=US, 2=DS, 3=bidirectional).
		 * THE data GEM is the BIDIRECTIONAL one (this OLT: gem 223,
		 * tcont-ptr 0x8000, dir 3).  The OLT also creates a DS-only
		 * broadcast CTP FIRST (gem 4095, tcont-ptr 0, dir 2) — that
		 * one is covered by the fixed CG_MCAST_GEM_ID install, so it
		 * must never claim the data-GEM slot (live-proven ordering). */
		if (class_id == 268 && m == 4 && len >= 13) {
			u16 g = ((u16)pdu[8] << 8) | pdu[9];

			if (g && g != cg->omcc_gem) {
				if (pdu[12] == 3 && g != cg->dg_gem) {
					WRITE_ONCE(cg->dg_gem, g);
					cg->dg_tcont_ptr = ((u16)pdu[10] << 8) | pdu[11];
					cg->dg_dir = pdu[12];
					dev_info(cg->dev,
						 "OMCI: data GEM port-id %u (tcont-ptr 0x%04x dir %u)\n",
						 g, cg->dg_tcont_ptr, cg->dg_dir);
					schedule_work(&cg->isr_work);
				} else if (pdu[12] != 3) {
					dev_info(cg->dev,
						 "OMCI: uni-dir GEM CTP %u (dir %u) — not the data GEM\n",
						 g, pdu[12]);
				}
			}
		}

		/* on-wire MIB-Reset: fresh provisioning follows — drop the
		 * shadow so stale ids are never re-installed */
		if (m == 15) {
			WRITE_ONCE(cg->dt_alloc, 0);
			WRITE_ONCE(cg->dg_gem, 0);
			cg->data_installed = false;
			if (cg->wan_ndev)
				netif_carrier_off(cg->wan_ndev);
		}
	}

	/* Stage C: answer with the responder + TX the reply upstream.  The
	 * PDU is 48 bytes; clamp a padded frame so a Create body never
	 * swallows trailing pad bytes. */
	if (len > OMCI_LEN)
		len = OMCI_LEN;
	{
		u8 resp[OMCI_LEN];
		int n = 0;

		spin_lock(&cg->omci_lock);
		if (cg->omci_active)
			n = omci_onu_input(cg->omci, pdu, len, resp);
		spin_unlock(&cg->omci_lock);
		if (n == OMCI_LEN)
			cg_omci_tx(cg, resp);
	}
}

/* Bottom half: drain the event ring and run the FSM tracker + OMCC binds. */
static void cg_isr_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(work, struct cortina_gpon, isr_work);
	struct cg_evt ev;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&cg->evt_lock, flags);
		if (cg->evt_tail == cg->evt_head) {
			spin_unlock_irqrestore(&cg->evt_lock, flags);
			break;
		}
		ev = cg->evt[cg->evt_tail % CG_EVT_RING_SZ];
		cg->evt_tail++;
		spin_unlock_irqrestore(&cg->evt_lock, flags);

		if (ev.intr & CG_INT_ONU_ST_CHG) {
			u8 last = cg->last_state;

			if (last != ev.state)
				dev_info(cg->dev, "FSM %s -> %s (onu-id %u)\n",
					 cg_state_name[last & 7],
					 cg_state_name[ev.state & 7], ev.id);
			/*
			 * O5 exit = link down (vendor condition): leaving
			 * Operation for anything but POPUP, leaving POPUP for
			 * anything but Operation/Ranging (POPUP->Ranging is the
			 * Type-B popup, kept alive), or entering EmergencyStop.
			 */
			if ((last == CG_STATE_OPERATION && ev.state != CG_STATE_OPERATION &&
			     ev.state != CG_STATE_POPUP) ||
			    (last == CG_STATE_POPUP && ev.state != CG_STATE_OPERATION &&
			     ev.state != CG_STATE_RANGING) ||
			    ev.state == CG_STATE_ESTOP)
				cg_datapath_reset_stub(cg);
			cg->last_state = ev.state;
		}

		if ((ev.intr & CG_INT_ONU_ID) && ev.id != CG_ONU_ID_NONE)
			cg_omcc_tcont_bind(cg, ev.id);

		if (ev.intr & CG_INT_DACT)
			dev_warn(cg->dev, "Deactivate_ONU-ID received\n");

		if (ev.intr & CG_INT_KSW)
			dev_info(cg->dev, "Key_Switching_Time (AES rekey = next phase, no AES keys in use)\n");

		/* stock recomputes frame_var on every DS PLOAM (Extended_
		 * Burst_Length may arrive/change any time in O2+) */
		if (ev.intr & (CG_INT_PLOAMD | CG_INT_ONU_ST_CHG))
			cg_frame_var_update(cg);

		/*
		 * OMCC bring-up on PORTID-in-O5 (vendor path), and ALSO on
		 * entering O5 (covers a PORTID interrupt that fired before the
		 * FSM reached Operation — the vendor drops that event and waits
		 * for the OLT to resend; re-checking on O5 entry closes it, and
		 * doubles as the vendor's restore-on-link-up).
		 */
		if (ev.intr & (CG_INT_PORTID | CG_INT_ONU_ST_CHG))
			cg_omcc_try_up(cg, ev.state);
	}

	/* Stage D: (re-)install the data path once the OMCC is up and both
	 * provisioning halves are known (also re-run by cg_rx_omci kicking
	 * this work when the OLT's ME 262/268 arrive). */
	cg_data_try_install(cg);
}

/*
 * Service one interrupt group (vendor __do_intr_isp): read enable + status,
 * zero the enable, W1C the enabled sources, hand back src&ena, re-arm the
 * enable.  The zero/re-arm bracket is MANDATORY re-arm protocol — dropping it
 * stops further interrupts.
 */
static u32 cg_intr_group_service(struct cortina_gpon *cg, u32 sts_off, u32 en_off)
{
	u32 intre, intrs;

	intre = readl(cg->mac + en_off);
	intrs = readl(cg->mac + sts_off);
	writel(0, cg->mac + en_off);
	writel(intrs & intre, cg->mac + sts_off);	/* W1C */
	writel(intre, cg->mac + en_off);
	return intrs & intre;
}

/*
 * Top-level ISR on the shared NE global line (GIC SPI 1).  Vendor __pon_isr
 * shape: mask the PON aggregate, loop { save+zero int_top_en, read int_top
 * (read-clears), dispatch the pending groups, restore int_top_en } bounded at
 * 32 passes (DoS guard), ack + unmask, kick the bottom half.
 */
static irqreturn_t cg_isr(int irq, void *data)
{
	struct cortina_gpon *cg = data;
	bool pending = false, queued = false;
	u32 glb_ie, ie, top, src;
	int pass;

	/* mask the PON aggregate at the GLB level (vendor __pon_top_intr_mask) */
	glb_ie = readl(cg->glb + CG_GLB_PON_INTEN0);
	writel(glb_ie & ~CG_PON_INT0_PON_MAC, cg->glb + CG_GLB_PON_INTEN0);

	for (pass = 0; pass < 32; pass++) {
		ie = readl(cg->mac + CG_REG_INT_TOP_EN);
		writel(0, cg->mac + CG_REG_INT_TOP_EN);
		top = readl(cg->mac + CG_REG_INT_TOP) & ie;	/* read-clears */

		if (top & BIT(0)) {
			src = cg_intr_group_service(cg, CG_REG_INT, CG_REG_INT_EN);
			if (src) {
				u32 onu = cg_mac_rd(cg, CG_REG_GPON_ONU);

				spin_lock(&cg->evt_lock);
				if (cg->evt_head - cg->evt_tail < CG_EVT_RING_SZ) {
					struct cg_evt *ev =
						&cg->evt[cg->evt_head % CG_EVT_RING_SZ];

					ev->intr = src;
					ev->state = CG_ONU_STATE(onu);
					ev->id = CG_ONU_ID(onu);
					cg->evt_head++;
					queued = true;
				} else {
					cg->evt_drop++;
				}
				spin_unlock(&cg->evt_lock);
			}
		}
		/* groups 2/3/4 ship enable=0; still run the W1C/re-arm bracket */
		if (top & BIT(1))
			cg_intr_group_service(cg, CG_REG_INT2, CG_REG_INT2_EN);
		if (top & BIT(2))
			cg_intr_group_service(cg, CG_REG_INT3, CG_REG_INT3_EN);
		if (top & BIT(3))
			cg_intr_group_service(cg, CG_REG_INT4, CG_REG_INT4_EN);

		writel(ie, cg->mac + CG_REG_INT_TOP_EN);
		if (!top)
			break;
		pending = true;
	}

	/* ack the ne_ictl line (harmless if the status is a pure level view;
	 * needed if it latches — the vendor per-ictl irqchip acks it this way) */
	writel(CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_STS);
	/* unmask the PON aggregate (vendor __pon_top_intr_unmask) */
	writel(glb_ie | CG_PON_INT0_PON_MAC, cg->glb + CG_GLB_PON_INTEN0);

	if (queued)
		schedule_work(&cg->isr_work);
	if (!pending)
		return IRQ_NONE;	/* shared line, not ours */
	cg->irq_count++;
	return IRQ_HANDLED;
}

/*
 * Arm the interrupt path (vendor aal_gpon_intr_init order): silence the top,
 * read-clear stale int_top, per group {disable, W1C the default mask, enable},
 * then open int_top_en and the two GLB-level aggregation gates.
 */
static int cg_intr_setup(struct cortina_gpon *cg, struct platform_device *pdev)
{
	static const struct { u32 sts, en, mask; } grp[4] = {
		{ CG_REG_INT,  CG_REG_INT_EN,  CG_INT_EN_DEFAULT },
		{ CG_REG_INT2, CG_REG_INT2_EN, 0 },
		{ CG_REG_INT3, CG_REG_INT3_EN, 0 },
		{ CG_REG_INT4, CG_REG_INT4_EN, 0 },
	};
	u32 v;
	int i, ret;

	spin_lock_init(&cg->evt_lock);
	INIT_WORK(&cg->isr_work, cg_isr_work);

	cg->irq = platform_get_irq(pdev, 0);
	if (cg->irq < 0) {
		dev_warn(cg->dev, "no interrupt in DT (%d) - post-O5 servicing OFF\n",
			 cg->irq);
		return cg->irq;
	}
	/* request BEFORE unmasking the HW gates so no edge is lost */
	ret = devm_request_irq(cg->dev, cg->irq, cg_isr, IRQF_SHARED,
			       DRV_NAME, cg);
	if (ret) {
		dev_warn(cg->dev, "request_irq(%d) failed: %d\n", cg->irq, ret);
		return ret;
	}

	writel(0, cg->mac + CG_REG_INT_TOP_EN);
	(void)readl(cg->mac + CG_REG_INT_TOP);		/* read-clear stale */
	for (i = 0; i < 4; i++) {
		writel(0, cg->mac + grp[i].en);
		writel(grp[i].mask, cg->mac + grp[i].sts);	/* W1C stale */
		writel(grp[i].mask, cg->mac + grp[i].en);
	}
	writel(CG_INT_TOP_EN_ALL, cg->mac + CG_REG_INT_TOP_EN);

	/* GLB aggregation: PON_MACe (level 1) + ne_ictl line 5 (level 2).
	 * RMW set only our bits — other ne_ictl lines belong to the NI. */
	v = readl(cg->glb + CG_GLB_PON_INTEN0);
	writel(v | CG_PON_INT0_PON_MAC, cg->glb + CG_GLB_PON_INTEN0);
	v = readl(cg->glb + CG_GLB_NE_ICTL_EN);
	writel(v | CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_EN);

	dev_info(cg->dev, "interrupts armed: irq %d, int_en=0x%08x int_top_en=0x%x pon_inten0=0x%08x ne_ictl_en=0x%08x\n",
		 cg->irq, readl(cg->mac + CG_REG_INT_EN),
		 readl(cg->mac + CG_REG_INT_TOP_EN),
		 readl(cg->glb + CG_GLB_PON_INTEN0),
		 readl(cg->glb + CG_GLB_NE_ICTL_EN));
	return 0;
}

static void cg_intr_teardown(struct cortina_gpon *cg)
{
	u32 v;

	if (cg->irq < 0)
		return;
	/* close the gates innermost-out, then flush the bottom half */
	writel(0, cg->mac + CG_REG_INT_TOP_EN);
	v = readl(cg->glb + CG_GLB_NE_ICTL_EN);
	writel(v & ~CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_EN);
	cancel_work_sync(&cg->isr_work);
}

/* ------------------------------------------------------------------ */
/* gpon0 — the WAN netdev over the GPON data path (Stage D)            */
/* ------------------------------------------------------------------ */

static int cg_wan_open(struct net_device *ndev)
{
	struct cortina_gpon *cg = cg_singleton;

	if (cg && cg->data_installed)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int cg_wan_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static netdev_tx_t cg_wan_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct cortina_gpon *cg = cg_singleton;

	if (!cg || !cg->data_installed || !IS_REACHABLE(CONFIG_CORTINA_NI)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	return cortina_ni_pon_data_tx(skb, ndev);
}

static const struct net_device_ops cg_wan_ops = {
	.ndo_open		= cg_wan_open,
	.ndo_stop		= cg_wan_stop,
	.ndo_start_xmit		= cg_wan_xmit,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};

/* Register gpon0.  MAC = a locally-administered address one above eth0's
 * (02:96:07:f0:00:01); per-board factory-MAC injection is the fleet TODO
 * shared with eth0.  Carrier tracks the data-path install. */
static void cg_wan_create(struct cortina_gpon *cg)
{
	static const u8 mac[ETH_ALEN] = { 0x02, 0x96, 0x07, 0xf0, 0x00, 0x02 };
	struct net_device *ndev;

	ndev = alloc_etherdev(0);
	if (!ndev)
		return;
	strscpy(ndev->name, "gpon0", sizeof(ndev->name));
	ndev->netdev_ops = &cg_wan_ops;
	eth_hw_addr_set(ndev, mac);
	SET_NETDEV_DEV(ndev, cg->dev);
	netif_carrier_off(ndev);
	if (register_netdev(ndev)) {
		dev_warn(cg->dev, "gpon0 register failed - no WAN netdev\n");
		free_netdev(ndev);
		return;
	}
	cg->wan_ndev = ndev;
	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		cortina_ni_pon_wan_ndev_set(ndev);
	dev_info(cg->dev, "WAN netdev gpon0 registered (%pM)\n", mac);
}

/* Read the 4 ASCII bytes of the vendor-id register in wire order. */
static void cg_read_vendor(struct cortina_gpon *cg, char out[5])
{
	u32 v = cg_mac_rd(cg, CG_REG_VENDOR);

	out[0] = (v >> 24) & 0xff;
	out[1] = (v >> 16) & 0xff;
	out[2] = (v >> 8) & 0xff;
	out[3] = v & 0xff;
	out[4] = '\0';
}

static int cg_proc_show(struct seq_file *m, void *v)
{
	struct cortina_gpon *cg = m->private;
	char vendor[5];
	u32 onu, alarm;

	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	alarm = cg_mac_rd(cg, CG_REG_ALARM);

	seq_printf(m, "gpon-mac @ phys 0x%llx + 0x%x\n",
		   (unsigned long long)CG_PON_WINDOW_PHYS, CG_GPON_MAC_OFF);
	seq_printf(m, "vendor-id      = 0x%08x (\"%s\")\n",
		   cg_mac_rd(cg, CG_REG_VENDOR), vendor);
	seq_printf(m, "vendor-spec    = 0x%08x\n", cg_mac_rd(cg, CG_REG_VENDOR_SPEC));
	seq_printf(m, "gpon_ds        = 0x%08x\n", cg_mac_rd(cg, CG_REG_GPON_DS));
	seq_printf(m, "onu(state+id)  = 0x%08x\n", onu);
	seq_printf(m, "main(eqd)      = 0x%08x\n", cg_mac_rd(cg, CG_REG_GPON_MAIN));
	seq_printf(m, "alarm          = 0x%08x%s\n", alarm,
		   alarm ? " (LOS/LOF!)" : " (no alarm, DS locked)");
	seq_printf(m, "onu_cfg        = 0x%08x\n", cg_mac_rd(cg, CG_REG_ONU_CFG_REAL));
	seq_printf(m, "us(frame_var)  = 0x%08x  t3_preamble = 0x%08x  gpon_ctrl = 0x%08x\n",
		   cg_mac_rd(cg, CG_REG_US), cg_mac_rd(cg, CG_REG_T3_PREAMBLE),
		   cg_mac_rd(cg, CG_REG_GPON_MAC_CTRL));
	/* post-O5 servicing (interrupts / FSM tracker / OMCC bind) */
	seq_puts(m, "-- post-O5 servicing --\n");
	seq_printf(m, "irq            = %d (count=%u, evt_drop=%u)\n",
		   cg->irq, cg->irq_count, cg->evt_drop);
	seq_printf(m, "fsm            = %s (live id 0x%02x), tracked %s\n",
		   cg_state_name[CG_ONU_STATE(onu)], CG_ONU_ID(onu),
		   cg_state_name[cg->last_state & 7]);
	seq_printf(m, "omcc           = %s (alloc=%u gem=%u)\n",
		   cg->omcc_up ? "UP" : "down", cg->omcc_alloc, cg->omcc_gem);
	seq_printf(m, "ds_omci_rx     = %u (short=%u)  pdc_ctrl = 0x%08x (%s, expect 0x02870002)\n",
		   cg->omci_rx, cg->omci_rx_short, readl(cg->pon + CG_PDC_CTRL),
		   cg->pdc_ready ? "programmed" : "NOT programmed");
	seq_printf(m, "puc            = %s  us_rx=%u enq=%u drop=%u  pucif=0x%08x\n",
		   cg->puc_ready ? "programmed" : "NOT programmed",
		   readl(cg->pon + CG_PUC_BMC_RX_PKT),
		   readl(cg->pon + CG_PUC_BMC_RX_PKT_ENQ),
		   readl(cg->pon + CG_PUC_BMC_FORCE_DROP),
		   readl(cg->pon + CG_XGPN_PUCIF_CTRL));
	seq_printf(m, "omci_resp      = %s tx=%u fail=%u ds_crc ok=%u bad=%u",
		   cg->omci_active ? "armed" : "off",
		   cg->omci_tx, cg->omci_tx_fail,
		   cg->omci_ds_crc_ok, cg->omci_ds_crc_bad);
	if (cg->omci)
		seq_printf(m, "  mds=%u store=%u avc=%u unhandled=%u",
			   cg->omci->mds, cg->omci->store_n,
			   cg->omci->avc_count, cg->omci->unhandled);
	seq_putc(m, '\n');
	seq_printf(m, "data           = %s alloc=%u (me 0x%04x) gem=%u (tcont-ptr 0x%04x dir %u) bcast=%u carrier=%d\n",
		   cg->data_installed ? "INSTALLED" : "down",
		   cg->dt_alloc, cg->dt_inst, cg->dg_gem, cg->dg_tcont_ptr,
		   cg->dg_dir, CG_MCAST_GEM_ID,
		   cg->wan_ndev ? netif_carrier_ok(cg->wan_ndev) : -1);
	seq_printf(m, "omci_port      = 0x%08x (en=%d id=%u)\n",
		   cg_mac_rd(cg, CG_REG_OMCI_PORT),
		   !!(cg_mac_rd(cg, CG_REG_OMCI_PORT) & CG_OMCI_PORT_EN),
		   CG_OMCI_PORT_ID(cg_mac_rd(cg, CG_REG_OMCI_PORT)));
	seq_printf(m, "int_en/top_en  = 0x%08x / 0x%x  (int2/3/4_en = 0x%x/0x%x/0x%x)\n",
		   cg_mac_rd(cg, CG_REG_INT_EN), cg_mac_rd(cg, CG_REG_INT_TOP_EN),
		   cg_mac_rd(cg, CG_REG_INT2_EN), cg_mac_rd(cg, CG_REG_INT3_EN),
		   cg_mac_rd(cg, CG_REG_INT4_EN));
	if (cg->glb)
		seq_printf(m, "glb pon_int0   = 0x%08x en=0x%08x  ne_ictl sts=0x%08x en=0x%08x\n",
			   readl(cg->glb + CG_GLB_PON_INT0),
			   readl(cg->glb + CG_GLB_PON_INTEN0),
			   readl(cg->glb + CG_GLB_NE_ICTL_STS),
			   readl(cg->glb + CG_GLB_NE_ICTL_EN));

	/* serdes/gearbox/laser (PON-window raw offsets, for US-LOS diagnosis) */
	seq_puts(m, "-- serdes/gbox/laser --\n");
	seq_printf(m, "rgb8(a05c)     = 0x%08x  (DS-lock: (v&0x9c01)==0x9c00)\n", readl(cg->pon + 0xa05c));
	seq_printf(m, "gbox(a060)     = 0x%08x  (stock 0x454 rx/tx bit-order)\n", readl(cg->pon + 0xa060));
	seq_printf(m, "reg(a064)      = 0x%08x  (stock 0)\n", readl(cg->pon + 0xa064));
	seq_printf(m, "reg(a068)      = 0x%08x  (stock 1)\n", readl(cg->pon + 0xa068));
	seq_printf(m, "reg(a070)      = 0x%08x  (stock 1)\n", readl(cg->pon + 0xa070));
	seq_printf(m, "psds_init(glb) = 0x%08x  (ben_oen bit4, pow_pcix bit5)\n", readl(cg->glb + CG_GLB_PSDS_INIT));
	seq_printf(m, "laser_route    : glb(0x42c)=0x%08x mux0(0x130)=0x%08x gpio0 cfg(0x300)=0x%08x out(0x304)=0x%08x  (stock 0x01101101 / 0x00001fff / 0xffffe7bf / 0x00000040)\n",
		   readl(cg->glb + CG_GLB_PINROUTE), readl(cg->glb + CG_GLB_GPIO_MUX0),
		   readl(cg->gpio + CG_PERGPIO_CFG0), readl(cg->gpio + CG_PERGPIO_OUT0));
	seq_printf(m, "  gpio pin34   : mux(0x134)=0x%08x cfg(0x324)=0x%08x in(0x32c)=0x%08x  (stock 0x00000000 / 0xffffffff / in bit2=0 net-low=TX_DIS de-asserted)\n",
		   readl(cg->glb + CG_GLB_GPIO_MUX1), readl(cg->gpio + CG_PERGPIO_CFG1),
		   readl(cg->gpio + CG_PERGPIO_IN1));
	seq_printf(m, "  gpio grp3/4  : cfg3(0x36c)=0x%08x out3(0x370)=0x%08x cfg4(0x390)=0x%08x out4(0x394)=0x%08x  (stock 0xfffdef00/0x00021010/0xffffc5ff/0x00003200)\n",
		   readl(cg->gpio + CG_PERGPIO_CFG3), readl(cg->gpio + CG_PERGPIO_OUT3),
		   readl(cg->gpio + CG_PERGPIO_CFG4), readl(cg->gpio + CG_PERGPIO_OUT4));
	cg_bosa_proc_show(cg->dev, m);

	/* full GPON MAC block dump (nonzero) for diffing against the stock golden.
	 * SKIP int_top (0xa4, READ-CLEARS: a cat of /proc must never eat a pending
	 * interrupt from under the ISR).  0x80-0x94 (DS MIB) and 0x1a8 (o5 count)
	 * are clear-on-read: dumped, but a read zeroes them. */
	seq_puts(m, "-- MAC block (nonzero; 0xa4 skipped; 0x80-0x94/0x1a8 clear-on-read) --\n");
	{
		u32 off, val;

		for (off = 0; off <= 0x1f4; off += 4) {
			if (off == CG_REG_INT_TOP)
				continue;
			val = cg_mac_rd(cg, off);
			if (val)
				seq_printf(m, "+0x%03x=0x%08x\n", off, val);
		}
	}
	return 0;
}

static int cg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cg_proc_show, cg_singleton);
}

/*
 * ONE-SHOT on-demand PLOAM MIB read: `echo mib <sel-hex> > /proc/gpon`.
 * Result goes to dmesg.  This is the sanctioned replacement for the removed
 * automatic cg_ploam_tx_mib probe: it fires ONLY on an explicit userspace
 * request (the devmem equivalent -- this lean image ships no /dev/mem), never
 * from the activation path or a periodic loop.  The go-poll is bounded and
 * short.  sel bit8 selects the RX bank (0x101 = Upstream_Overhead received),
 * low bits the message/counter index (0x001 = Serial_Number_ONU transmitted).
 */
static ssize_t cg_proc_write(struct file *file, const char __user *ubuf,
			     size_t len, loff_t *ppos)
{
	struct cortina_gpon *cg = cg_singleton;
	char buf[32], *p;
	u32 sel, acc, data;
	int i;

	if (!cg || len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	p = strim(buf);
	/* one-shot full BOSA register dump to dmesg (cold-state diffing) */
	if (strcmp(p, "bosa dump") == 0) {
		cg_bosa_dump(cg->dev);
		return len;
	}
	if (strncmp(p, "mib ", 4) != 0 || kstrtou32(strim(p + 4), 16, &sel))
		return -EINVAL;

	writel(0x80000000u | (sel & 0x3ff), cg->mac + 0x184);
	for (i = 0; i < 1000; i++) {
		acc = readl(cg->mac + 0x184);
		if (!(acc & 0x80000000u))
			break;
		udelay(1);
	}
	data = readl(cg->mac + 0x188);
	dev_info(cg->dev,
		 "one-shot PLM MIB sel=0x%03x: access=0x%08x data=0x%08x (go %s after %d polls)\n",
		 sel, acc, data,
		 (acc & 0x80000000u) ? "STUCK" : "cleared", i);
	return len;
}

static const struct proc_ops cg_proc_ops = {
	.proc_open	= cg_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= cg_proc_write,
};

static int cortina_gpon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_gpon *cg;
	char vendor[5];
	u32 onu;

	cg = devm_kzalloc(dev, sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return -ENOMEM;
	cg->dev = dev;
	cg->irq = -1;		/* until cg_intr_setup succeeds */

	/* Stage C: the G.988 responder context — allocated up front so the
	 * OMCC-up path (which can fire during the probe's ranging poll) only
	 * ever initializes it, never allocates.  ~7 KB. */
	spin_lock_init(&cg->omci_lock);
	INIT_DELAYED_WORK(&cg->veip_avc_work, cg_veip_avc_work);
	cg->omci = devm_kzalloc(dev, sizeof(*cg->omci), GFP_KERNEL);
	if (!cg->omci)
		dev_warn(dev, "no OMCI responder ctx - DS OMCI will not be answered\n");

	/*
	 * Map the whole 48 KiB PON window.  The window is a 40-bit AXI address
	 * (0x4_F5500000); ioremap takes a 64-bit phys_addr_t so this is fine on
	 * arm64.  We ioremap the fixed physical base directly (validated on real
	 * hardware) rather than claiming a DT resource, because the same window is
	 * also listed by the sibling cortina-ni node - a non-exclusive map avoids a
	 * request_mem_region conflict.
	 */
	cg->pon = devm_ioremap(dev, CG_PON_WINDOW_PHYS, CG_PON_WINDOW_SIZE);
	if (!cg->pon) {
		dev_err(dev, "failed to map PON window 0x%llx\n",
			(unsigned long long)CG_PON_WINDOW_PHYS);
		return -ENOMEM;
	}
	cg->mac = cg->pon + CG_GPON_MAC_OFF;

	/*
	 * Map the GLB reset/clock window and dump the PON/GPON reset-control
	 * registers read-only.  On our minimal build the GPON MAC is held in
	 * reset; comparing these against the live-stock released values tells us
	 * the minimal diff to write (done in a later step) without clobbering the
	 * PUC/PDC packet-engine bits the NI datapath shares.
	 */
	cg->glb = devm_ioremap(dev, CG_GLB_WINDOW_PHYS, CG_GLB_WINDOW_SIZE);
	cg->gpio = devm_ioremap(dev, CG_PERGPIO_PHYS, CG_PERGPIO_SIZE);
	if (cg->glb) {
		dev_info(dev, "GLB reset regs (ours): EPON_CNTL=0x%08x GPON_CNTL=0x%08x PON_CNTL=0x%08x PSDS_INIT=0x%08x\n",
			 readl(cg->glb + CG_GLB_EPON_CNTL),
			 readl(cg->glb + CG_GLB_GPON_CNTL),
			 readl(cg->glb + CG_GLB_PON_CNTL),
			 readl(cg->glb + CG_GLB_PSDS_INIT));
		dev_info(dev, "GLB reset regs (stock released): EPON_CNTL=0x00030000 GPON_CNTL=0x00000003 PON_CNTL=0x0000030e\n");

		if (cg_do_reset) {
			cg_glb_reset(cg);
			cg_psds_init(cg);
			dev_info(dev, "GLB after: EPON=0x%08x GPON=0x%08x PON=0x%08x PSDS_INIT=0x%08x\n",
				 readl(cg->glb + CG_GLB_EPON_CNTL),
				 readl(cg->glb + CG_GLB_GPON_CNTL),
				 readl(cg->glb + CG_GLB_PON_CNTL),
				 readl(cg->glb + CG_GLB_PSDS_INIT));
			dev_info(dev, "PSDS after: MODE=0x%08x RGB8=0x%08x (bit11 CKRDY_TX=%d)\n",
				 readl(cg->pon + CG_PSDS_MODE),
				 readl(cg->pon + CG_PSDS_RGB8),
				 !!(readl(cg->pon + CG_PSDS_RGB8) & BIT(11)));

			/* arm the post-O5 servicing BEFORE ranging starts so
			 * the ONU_ID/PORTID/ONU_ST_CHG events of the very
			 * first O1->O5 pass are serviced live */
			if (cg_do_intr)
				cg_intr_setup(cg, pdev);

			if (cg_activate) {
				int i;

				cg_mac_activate(cg);
				dev_info(dev, "activate: onu_cfg=0x%08x onu_ctl=0x%08x gpon_ds=0x%08x\n",
					 cg_mac_rd(cg, CG_REG_ONU_CFG_REAL),
					 cg_mac_rd(cg, CG_REG_ONU_CTL),
					 cg_mac_rd(cg, CG_REG_GPON_DS));
				/* poll the HW ranging FSM: onu.state, RGB8 (bit15 BER_NOTIFY
				 * = DS frame sync), and the superframe counter (advances =
				 * DS frames received; NOT clear-on-read like the DS MIB).
				 * Re-check frame_var each pass: covers cg_do_intr=0 and a
				 * PLOAM event missed while the IRQ path was arming. */
				for (i = 0; i < 30; i++) {
					cg_frame_var_update(cg);
					dev_info(dev, "range t=%ds: onu=0x%08x rgb8=0x%08x superframe=0x%08x alarm=0x%08x us=0x%08x psds_init=0x%08x\n",
						 i, cg_mac_rd(cg, CG_REG_GPON_ONU),
						 readl(cg->pon + CG_PSDS_RGB8),
						 cg_mac_rd(cg, 0xfc),
						 cg_mac_rd(cg, CG_REG_ALARM),
						 cg_mac_rd(cg, CG_REG_US),
						 readl(cg->glb + CG_GLB_PSDS_INIT));
					msleep(200);
				}
			}
		}
	} else {
		dev_warn(dev, "failed to map GLB window 0x%llx\n",
			 (unsigned long long)CG_GLB_WINDOW_PHYS);
	}

	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	dev_info(dev, "GPON MAC vendor-id \"%s\" onu=0x%08x alarm=0x%08x\n",
		 vendor, onu, cg_mac_rd(cg, CG_REG_ALARM));

	if (strcmp(vendor, "XPON") != 0)
		dev_warn(dev, "vendor-id != \"XPON\" - PON window base may be wrong\n");

	cg_singleton = cg;
	/* Stage B: receive the DS OMCI PDUs the NI CPU-RX path classifies out
	 * (ethertype 0xfff1).  Registered after cg_singleton so the handler
	 * never sees a half-initialized context. */
	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		cortina_ni_pon_rx_hook_set(cg_rx_omci);
	cg_wan_create(cg);	/* Stage D: the gpon0 WAN netdev */
	cg->proc = proc_create_data("gpon", 0644, NULL, &cg_proc_ops, cg);
	platform_set_drvdata(pdev, cg);
	dev_info(dev, "cortina-gpon phase-0 probe complete (/proc/gpon)\n");
	return 0;
}

static void cortina_gpon_remove(struct platform_device *pdev)
{
	struct cortina_gpon *cg = platform_get_drvdata(pdev);

	if (IS_REACHABLE(CONFIG_CORTINA_NI)) {
		cortina_ni_pon_wan_ndev_set(NULL);
		cortina_ni_pon_rx_hook_set(NULL);
	}
	cancel_delayed_work_sync(&cg->veip_avc_work);
	cg_intr_teardown(cg);
	if (cg->wan_ndev) {
		unregister_netdev(cg->wan_ndev);
		free_netdev(cg->wan_ndev);
	}
	if (cg->proc)
		proc_remove(cg->proc);
	if (cg_singleton == cg)
		cg_singleton = NULL;
}

static const struct of_device_id cortina_gpon_of_match[] = {
	{ .compatible = "realtek,rtl9607f-gpon" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_gpon_of_match);

static struct platform_driver cortina_gpon_driver = {
	.probe	= cortina_gpon_probe,
	.remove	= cortina_gpon_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= cortina_gpon_of_match,
	},
};
module_platform_driver(cortina_gpon_driver);

MODULE_DESCRIPTION("Cortina-Access GPON MAC driver for Realtek RTL9607F Elnath");
MODULE_LICENSE("GPL");
