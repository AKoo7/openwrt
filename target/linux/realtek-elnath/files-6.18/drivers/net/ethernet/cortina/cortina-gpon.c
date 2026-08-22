// SPDX-License-Identifier: GPL-2.0
/*
 * TIER: CHIP — hardware shell for exactly ONE part: registers, DMA,
 * interrupts, board glue.  It DOES; the core DECIDES.  GPON protocol
 * logic belongs in the core tier (drivers/net/gpon), never here.
 * Role: RTL9607F (Cortina CA8277C) GPON MAC shell.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon-common/files-6.18/drivers/net/gpon/gpon_common.h.
 */
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
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/ratelimit.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "cortina-gpon-serdes.h"
#include "cortina-gpon-bosa.h"
#include "cortina-gpon-ddm.h"	/* SFF-8472 A2h optical decode (functional core) */
#include "cortina-ni.h"		/* cortina_ni_pon_rx_hook_set + cortina_ni_pon_tx */
#include "cortina-omci-bridge.h" /* M2: stock omci_app userspace bridge seam    */

/*
 * ★★ CUT SITE — WHERE THE OMCI RESPONDER WENT (code motion, 2026-08-05).
 *
 * This driver used to carry its own G.988 OMCI responder next to it, in
 * cortina/omci_responder.{c,h} (1112 + 143 lines).  Those two files are GONE
 * from this directory; every line of them now lives, unchanged, in the shared
 * protocol tree:
 *
 *   target/linux/gpon-common/files-6.18/drivers/net/gpon/
 *       gpon_omci_core.{h,c}   the G.988 baseline MESSAGE layer: parse the DS
 *                              PDU, dispatch by message type, build the US
 *                              response, stamp trailer + MIC
 *       gpon_omci_me.{h,c}     the MANAGED-ENTITY model: the descriptor table,
 *                              the static MIB-Upload rows, the dynamic store of
 *                              the instances the OLT created, struct omci_onu
 *
 * WHY THAT LAYER IS COMMON AND NOT OURS.  ITU-T G.988 is a specification, not a
 * property of the Cortina silicon: the same message rules and the same ME model
 * answer the same OLT on the Luna MIPS parts and on the future ARM OLT.  Two
 * private copies of a specification is one copy that silently drifts, so the
 * tree keeps exactly one (operator, 2026-08-05: "la idea es poner en común el
 * código que corresponde para no tener mucho duplicado" and, on the two
 * monoliths that each carried their own, "mal, poner en común").  The prefix is
 * gpon_ and not cortina_/rtl960x_ for the same reason: the layer must outlive
 * this vendor.
 *
 * WHAT STAYED HERE, AND WHY IT HAD TO.  Everything that touches the hardware:
 * the OMCC GEM/T-CONT binding, the DS receive hook and its CRC check, the US
 * transmit ring, the /proc view, the counters, the i2c DDM read that feeds
 * ME 263, and the workqueue that emits the post-O5 VEIP AVC.  The moved layer
 * is a FUNCTIONAL CORE — it decides and never does: no readl/writel, no device
 * pointer, no lock, no allocation, no sleeping, no clock read.  That is what
 * lets one source compile for big-endian MIPS and little-endian ARM64 and be
 * fuzzed on x86 with no board in the loop; it is also why the seam falls
 * exactly where it does.
 *
 * IT WAS CODE MOTION, AND THAT WAS MEASURED, NOT ASSERTED.  Normalised
 * (comment-stripped, whitespace-collapsed) the pre-move and post-move sources
 * differ ONLY by the include-guard rename, nine dropped `static` qualifiers and
 * the prototypes those nine now need in a header — not one statement, constant
 * or expression changed.  Executed differentially over 36 activation/fault
 * vectors before the pre-move file was retired:
 *     make -f gpon_x86_harness.mk gpon-harness-crosscheck   (dev/rtl9607c-test)
 *     -> "CODE MOTION CONFIRMED"
 * The pre-move file is recoverable from git (commit 03e5d15d96 plus the
 * uncommitted ME-65530 upload change, which the shared copy carries).
 */
#include "gpon_omci_core.h"	/* G.988 message layer: omci_onu_input()      */
#include "gpon_omci_me.h"	/* G.988 ME model: struct omci_onu, the store */
#include "gpon_gem_us.h"	/* upstream GEM/T-CONT mapping + bind verdict */

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
#define CG_PSDS_RGB8	0xa060	/* ★AOT fix#12: was 0xa05c (X400AXF map) */	/* DS-lock status; locked = (val & 0x9c01)==0x9c00 (stock 0x19c00) */
#define CG_PSDS_GBOX_CTRL	0xa064	/* ★AOT fix#12: was 0xa060 (X400AXF map) */	/* rx/tx bit-ordering[7:4]; stock=0x454.  WAS 0xa064 (stock=0) -> our US tx_bit_ordering never took -> OLT saw US LOS (live-diff 2026-07-13) */
#define CG_PON_EPON_SPARE	0x01c8	/* EPON_GLB_SPARE_CFG (PON window); bit31 for GPON los-rst */
/*
 * PSDS internal analog-register indirect interface (the ~266-row CMU/PLL/CDR/TX
 * profile is loaded through it).  Command word (+0xa088): bit31 = strobe,
 * bit30 = write (else read), bits[11:0] = internal register index; write-data
 * at +0xa08c, read-data at +0xa090.  The vendor aal_psds_reset CMU/PLL re-lock
 * (cg_psds_relock) strobes internal reg CG_PSDS_CMU_IDX bits[7:4].
 */
#define CG_PSDS_IND_CMD	0xa0cc	/* ★AOT fix#12: was 0xa088 (X400AXF map) */
#define CG_PSDS_IND_WDATA	0xa0d0	/* ★AOT fix#12: was 0xa08c (X400AXF map) */
#define CG_PSDS_IND_RDATA	0xa0d4	/* ★AOT fix#12: was 0xa090 (X400AXF map) */
#define CG_PSDS_IND_READ	0x80000000u	/* command: strobe, read */
#define CG_PSDS_IND_WRITE	0xc0000000u	/* command: strobe, write */
#define CG_PSDS_CMU_IDX		0x400		/* analog CMU reg; [7:4] = re-lock strobe */

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
/*
 * ★ 2026-08-08 AOT5221ZY (fix #11): the PON-domain reset/mode registers are at
 * DIFFERENT offsets on this silicon -- same class of defect as PSDS_INIT (fix #8,
 * 0x25c -> 0x22c).  These three are the disassembly-correct offsets from this
 * board's own shipping driver; the reference's are the header's, which are shifted.
 * Symptom when wrong: the writes land on neighbouring registers, so the GPON MAC
 * is never released from reset and its ENTIRE window reads one constant --
 * vendor-id, onu, alarm, superframe and int_en all returned e.g. 0x18420000 (a
 * different constant each boot), which the driver itself flagged as "PON window
 * base wrong, or the MAC is still gated".  The proof it is the offsets and not
 * the window: writing 0x00030000 to EPON_CNTL at 0x078 read back 0x00000000.
 */
#define CG_GLB_EPON_CNTL	0x074	/* header says 0x078 */
#define CG_GLB_GPON_CNTL	0x07c	/* header says 0x080 */
#define CG_GLB_PON_CNTL		0x094	/* header says 0x09c */
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
/* ★★★ 2026-08-07 AOT5221ZY: 0x22c, NOT 0x25c. The two trees disagree here and THIS BOARD
 * settles it: with 0x25c the probe silently resets the SoC immediately after the
 * "GLB reset regs" prints (reproduced on every boot, no panic text = bus hang). The
 * shipping 6.6 driver for this board records the same finding verbatim: "PSDS_INIT is now
 * the disasm-correct 0x22c (was 0x25c = garbage/bus-hang)", and it reaches O5 on this
 * fibre. Writing absolute 0x1 to the wrong offset lands on an unrelated register. */
#define CG_GLB_PSDS_INIT	0x22c
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
/* ★ 2026-08-08 AOT5221ZY: the 10-byte GPON registration password, wire order,
 * split 2 + 4 + 4 across three registers (PFRAG0 holds the first two bytes in
 * its low half).  Some OLTs -- including the DigiWorld one this board sits on --
 * refuse or drop a ranging ONU that presents no password: our shipping 6.6
 * driver traced repeated LOF/de-ranging to exactly this being unwritten. */
#define CG_REG_PFRAG0		0x01c
#define CG_REG_PFRAG1		0x020
#define CG_REG_PFRAG2		0x024
#define CG_REG_ALARM	0x07c	/* ★AOT fix#12: was 0x09c (X400AXF map) */	/* hdr 0x7c: LOS/LOF alarm bits (live levels) */

/*
 * Interrupt block: header 0x84..0xa8 -> SILICON 0xa4..0xc8 (+0x20 shift).
 * int_top is READ-TO-CLEAR; the four per-group STATUS registers are W1C.
 *   int_top bits[3:0] = {INTERRUPT, INTERRUPT2, INTERRUPT3, INTERRUPT4}.
 * (The header offsets 0x84-0xa8 land on clear-on-read DS MIB counters and the
 * ploamu control reg — writing "enables" there corrupted the US PLOAM engine.)
 */
#define CG_REG_INT_TOP	0x084	/* ★AOT fix#12: was 0x0a4 (X400AXF map) */	/* hdr 0x84: interrupt_top (read-clear) */
#define CG_REG_INT_TOP_EN	0x088	/* ★AOT fix#12: was 0x0a8 (X400AXF map) */	/* hdr 0x88: int_top_en */
#define CG_REG_INT	0x08c	/* ★AOT fix#12: was 0x0ac (X400AXF map) */	/* hdr 0x8c: INTERRUPT  status (W1C) - operational */
#define CG_REG_INT_EN	0x090	/* ★AOT fix#12: was 0x0b0 (X400AXF map) */	/* hdr 0x90: INTERRUPT  enable */
#define CG_REG_INT2	0x094	/* ★AOT fix#12: was 0x0b4 (X400AXF map) */	/* hdr 0x94: INTERRUPT2 status (W1C) - TC/parse err */
#define CG_REG_INT2_EN	0x098	/* ★AOT fix#12: was 0x0b8 (X400AXF map) */	/* hdr 0x98: INTERRUPT2 enable */
#define CG_REG_INT3	0x09c	/* ★AOT fix#12: was 0x0bc (X400AXF map) */	/* hdr 0x9c: INTERRUPT3 status (W1C) - negedge/MSB */
#define CG_REG_INT3_EN	0x0a0	/* ★AOT fix#12: was 0x0c0 (X400AXF map) */	/* hdr 0xa0: INTERRUPT3 enable */
#define CG_REG_INT4	0x0a4	/* ★AOT fix#12: was 0x0c4 (X400AXF map) */	/* hdr 0xa4: INTERRUPT4 status (W1C) - FEC MSB */
#define CG_REG_INT4_EN	0x0a8	/* ★AOT fix#12: was 0x0c8 (X400AXF map) */	/* hdr 0xa8: INTERRUPT4 enable */

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

#define CG_REG_GPON_ONU	0x0bc	/* ★AOT fix#12: was 0x0dc (X400AXF map) */	/* hdr 0xbc: ONU id[7:0], state[18:16]; dft id=0xff */
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

#define CG_REG_GPON_MAIN	0x0c0	/* ★AOT fix#12: was 0x0e0 (X400AXF map) */	/* hdr 0xc0: equalization delay (EqD) */
#define CG_REG_OMCI_PORT	0x0c8	/* ★AOT fix#12: was 0x0e8 (X400AXF map) */	/* hdr 0xc8: omci_port id[11:0], en[12]; HW-filled */
#define CG_OMCI_PORT_ID(v)	((v) & 0xfff)
#define CG_OMCI_PORT_EN		BIT(12)
#define CG_REG_T3_PREAMBLE	0x0d8	/* ★AOT fix#12: was 0x0f8 (X400AXF map) */	/* hdr 0xd8: extend[16], ranged[15:8], pre_range[7:0];
					 * HW-latched from the OLT's Extended_Burst_Length PLOAM */

/*
 * Indirect table access pairs, +0x20-shifted like everything >= hdr 0x50.
 * Live-confirmed on stock at Online (TCONT_ACCESS 0x14c=0x40000101,
 * US_PORT_ID_DATA 0x194=0xDF = the OLT-assigned GEM port).
 * Protocol (vendor __GPN_*_DO_INDIRCT_OP): write ACCESS = go(bit31) | rbw(bit30,
 * 1=write) | index/alloc-id, then poll ACCESS bit31 self-clear (<= 10000 reads).
 * Data flows through the DATA register (read entry -> DATA; DATA -> write entry).
 */
#define CG_REG_TCONT_ACCESS	0x12c	/* ★AOT fix#12: was 0x14c (X400AXF map) */	/* header 0x12c: alloc_id[11:0], sw_plm_en[16], rbw[30], go[31] */
#define CG_REG_TCONT_DATA	0x130	/* ★AOT fix#12: was 0x150 (X400AXF map) */	/* header 0x130: ploam_en[0], omci_en[1], index[6:2] (hw T-CONT 0-31) */
#define CG_REG_DS_GEM_ACCESS	0x134	/* ★AOT fix#12: was 0x154 (X400AXF map) */	/* header 0x134: id[11:0] (GEM port-id), sw_aes[16], rbw[30], go[31] */
#define CG_REG_DS_GEM_DATA	0x138	/* ★AOT fix#12: was 0x158 (X400AXF map) */	/* header 0x138: vld[0], aes[1], tdm[2], index[10:3] (intern gem) */
#define CG_REG_US_PORT_ACCESS	0x170	/* ★AOT fix#12: was 0x190 (X400AXF map) */	/* header 0x170: index[7:0] (us hw gem 0-255), rbw[30], go[31] */
#define CG_REG_US_PORT_DATA	0x174	/* ★AOT fix#12: was 0x194 (X400AXF map) */	/* header 0x174: id[11:0] (GEM port-id) */
/* Last slot the upstream port-map array actually has, read off the index field
 * width above (index[7:0]).  Named rather than left as an 8-bit assumption
 * because a count, a maximum, a stride and an index space are four different
 * quantities, and a write past the end of this array lands in an unrelated
 * register and is accepted without complaint.  Consumed by the compile-time
 * GPON_GEM_US_RANGE_OK() assertions on the two declared slot ranges below. */
#define CG_US_PORT_IDX_MAX	255

/*
 * ★★★ THE UPSTREAM TRANSMIT WITNESS (2026-08-11).
 *
 * This project has never had an oracle for "did an upstream DATA frame actually go out
 * on the fibre".  US OMCI is witnessed by the OLT answering; US data is witnessed by
 * NOTHING - and `data_enq` is an ENQUEUE counter, exactly the same kind of number as
 * `rptr`, which §4 of the 2026-08-10 handoff retracts in as many words ("rptr advancing
 * does NOT mean transmitted").  So "PADI goes out, no PADO comes back" was never a
 * measurement: the PADI may never have left.  Reading the BRAS/VLAN/session angle before
 * settling that is the same mistake in a new costume.
 *
 * The GPON MAC has a per-US-GEM MIB behind an indirect ACCESS/DATA pair, counting what
 * the framer actually transmitted.  Family note: the shipping GP3000 reg.txt is the
 * X400AXF-family map (US MIB at 0x170, TCONT_DATA at 0x150); this board is fix#12's
 * family, uniformly -0x20 from it, which puts the US MIB at 0x150 and leaves 0x170/0x174
 * as the US GEM port-id table above.  rtl8277c_registers.h agrees (0xf5506150), and all
 * five offsets are already read by the fix#75 PONWIN dump without aborting, so they
 * decode.  Raw-dumping them is useless though - it only shows whatever the last indirect
 * access happened to latch.  It has to be DRIVEN.
 */
#define CG_REG_US_MIB_ACCESS	0x150	/* sel[7:0], op_code[29:28], rbw[30], go[31] */
/*
 * ★ op_code[29:28] IS NOT OPTIONAL, and leaving it 0 is what made the RB7 run come back
 * all-zeros (including the OMCC positive control, so the run correctly refused to
 * conclude anything).  The vendor never reads this MIB without an op:
 *   aal_gpon_us_gem_port_mib_get(dev, AAL_GPON_PLOAM_MIB_READ_CLR, gem_idx, &mib)
 * - dal_rt_rtl9607f_gpon.c:4433/4475.  The aal_ layer itself is a closed blob, so the
 * enum's numeric values are not in any source we have; RB8 therefore SWEEPS all four
 * op_codes in one boot and lets the OMCC control identify the live one.
 * ⚠ If the live op turns out to be a READ-CLEAR, each read returns the count SINCE THE
 * LAST READ, not a running total - so a verdict must look for a nonzero SAMPLE, never a
 * last-minus-first delta.  RB8's verdict is written that way.
 */
#define CG_US_MIB_OP_CODE	GENMASK(29, 28)

/*
 * ★ THE DS MIB, used here purely as a POSITIVE CONTROL FOR THE INTERFACE ITSELF.
 *
 * If the US MIB reads zero at every op_code, there are two very different explanations
 * and no way to tell them apart from the US side alone: either nothing is transmitting,
 * or this ACCESS/DATA family is not being driven correctly (wrong sel space, wrong op,
 * needs an init).  The downstream MIB settles it, because downstream traffic is PROVEN
 * to flow on this board - RB6 delivered 286 frames to the WAN netdev - so a correctly
 * driven DS read CANNOT come back zero.
 *   DS nonzero, US zero  -> the interface works; the upstream really is silent.
 *   DS zero too          -> we are driving the family wrong; the US zero means nothing.
 * The block order in rtl8277c_registers.h makes the pairing exact: DS_PORT_ACCESS/
 * DATA3..0 at 0x13c-0x14c sit immediately before US_MIB_ACCESS/DATA3..0 at 0x150-0x160,
 * same shape, same family, same -0x20 fix#12 rebase.
 */
/*
 * ★★★ PER-T-CONT BUFFER OCCUPANCY — the discriminator that splits the last two suspects.
 *
 * Once the US GEM port table and the T-CONT CAM are proven correct and stock-identical
 * (RB10), "the PADI never leaves" has exactly two remaining shapes, and they call for
 * opposite fixes:
 *   BufOcc[data T-CONT] > 0  -> the frames DID reach the PUC and are sitting in the queue
 *                               waiting for a grant.  The wall is the GRANT/DBA path
 *                               (BWmap servicing, status reporting), NOT the injection.
 *   BufOcc[data T-CONT] == 0 -> the frames never arrived at the PUC at all.  The wall is
 *                               UPSTREAM of it - HEADER_A routing / fe_bypass / ARB / QM -
 *                               and no amount of grant work will help.
 * Nothing else measured so far separates these: `data_enq` counts the CPU handing frames
 * over, and the us_mib counts what left the fibre; this is the only view of the middle.
 *
 * ⚠ Read it as an OCCUPANCY, i.e. a level, not a total - it falls back to 0 as the queue
 * drains, so sample it WHILE pppd is dialling, not after.  A 0 read after the dial has
 * finished means nothing either way.
 * Note this table is live regardless of sw_dbru.enble (stock runs with DBRu disabled too,
 * `61b4 00000000`), because bufocc_mode=0 has the HW maintain it.
 */
#define CG_REG_DBRU_BUFOCC_ACCESS	0x1bc	/* Addr[4:0], Select[16], rw[30], access[31] */
#define CG_REG_DBRU_BUFOCC_DATA		0x1c0	/* BufOcc[15:0] */

#define CG_REG_DS_MIB_ACCESS	0x13c
#define CG_REG_DS_MIB_DATA3	0x140
#define CG_REG_DS_MIB_DATA2	0x144
#define CG_REG_DS_MIB_DATA1	0x148
#define CG_REG_DS_MIB_DATA0	0x14c
#define CG_REG_US_MIB_DATA3	0x154	/* bcnt hi */
#define CG_REG_US_MIB_DATA2	0x158	/* bcnt lo */
#define CG_REG_US_MIB_DATA1	0x15c	/* fcnt - GEM frames transmitted */
#define CG_REG_US_MIB_DATA0	0x160	/* pcnt - packets transmitted */

#define CG_DS_GEM_VLD		BIT(0)
#define CG_DS_GEM_INDEX(x)	(((x) & 0xff) << 3)

#define CG_TBL_GO		BIT(31)
#define CG_TBL_WR		BIT(30)
static uint qm_voq_enable = 1;
module_param(qm_voq_enable, uint, 0444);
MODULE_PARM_DESC(qm_voq_enable,
	"fix#102: run stock's queue_add enables for the data VoQs - TE_CB admission profile 3 + PUC DBA report enable (default on; =0 to A/B)");

/* fix#104: independent sub-gates so RC19's async SError (0xbe000011 in cg_isr_work,
 * ~11us after data-GEM install) can be A/B-isolated to admit (step 1) vs report (step 2). */
static uint qm_voq_admit = 1;
module_param(qm_voq_admit, uint, 0444);
MODULE_PARM_DESC(qm_voq_admit,
	"fix#104: step (1) TE_CB profile-3 admission writes (NI 0x95xx); =0 to A/B the async SError");
static uint qm_voq_report;	/* fix#108: default OFF - RC23 proved ANY REPORT_ENABLE0 write
				 * async-SErrors (report engine globally uninit); =1 only to test. */
module_param(qm_voq_report, uint, 0444);
MODULE_PARM_DESC(qm_voq_report,
	"step (2) PUC_QM_REPORT_ENABLE0 write; DEFAULT OFF - known to async-SError until the global "
	"PUC/DBA report-engine init is found (RC23). Set =1 only for report-path experiments");
static uint qm_voq_tbc = 1;
module_param(qm_voq_tbc, uint, 0444);
MODULE_PARM_DESC(qm_voq_tbc,
	"fix#106: init data VoQ PUC token-bucket memory (captured live-stock values); =0 to A/B vs report");
static uint qm_voq_report_mask = 0x0000ff00;
module_param(qm_voq_report_mask, uint, 0444);
MODULE_PARM_DESC(qm_voq_report_mask,
	"fix#107: REPORT_ENABLE0 bitmask (default data VoQ 8..15). =0x80 to probe report on the "
	"fully-set-up OMCI VoQ 7: SError there = report engine itself uninit; clean = per-data-VoQ gap");
static uint qm_puc_qm_enable;	/* fix#109: DEFAULT OFF - RC24 proved the RMW *read* of 0x8274
					 * itself sync-aborts (0x96000010): the whole PUC QM sub-block
					 * (0x8238+) is an UNCLOCKED/held-in-reset domain on our port.
					 * aal_puc_qm_enable_set works on stock only because aal_puc_init
					 * clock/reset-ungated the domain first (RMW bit8 of a sys-control
					 * reg + udelay). The real fix is that ungate, NOT this bit. =1
					 * only after the domain is powered. */
module_param(qm_puc_qm_enable, uint, 0444);
MODULE_PARM_DESC(qm_puc_qm_enable,
	"fix#109: RMW sets cfg_qmplmem_en (bit3) of PLEN_MEM_CTL 0x8274. DEFAULT OFF - the read "
	"sync-aborts until the QM sub-block clock/reset is ungated (see aal_puc_init). =1 only then.");

static uint data_tcont_omci = 1;
module_param(data_tcont_omci, uint, 0444);
MODULE_PARM_DESC(data_tcont_omci,
	"set omci_en|ploam_en on the DATA T-CONT bind (1 = unchanged/current; 0 = clear them, the fix#100 test - a pure data T-CONT carries neither)");

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
/* fix#109: PUC QM packet-length-memory control (PUC_QM_PUC_PLEN_MEM_CTL 0x8274).
 * bit3 cfg_qmplmem_en = the GLOBAL PUC QM enable.  RE'd from ca-ne.ko
 * aal_puc_qm_enable_set@0xb7bb0 (RMW: 0x8274 |= BIT(3)); stock runs it ONCE at
 * ponmac_init BEFORE any queue_add.  With QM disabled the DBA report engine's
 * packet-length memory is off -> reading REPORT_ENABLE0 (0x82d0) sync-aborts
 * (was misread as "write-only" in fix#103) AND enabling report async-SErrors. */
#define CG_PUC_QM_PLEN_MEM_CTL	(CG_PUC_BASE + 0x274)	/* cfg_qmplmem_sel[2:0] en[3] lch[4] */
#define CG_PUC_VOQFLUSH		(CG_PUC_BASE + 0x0dc)	/* voqid[7:0], tcontid[12:8], openpktflushen[16], start[31] */
#define CG_PUC_VALID_VOQ0	(CG_PUC_BASE + 0x1bc)	/* valid_voqN = VALID_VOQ0 - (voq/32)*4 */
/* fix#105: per-VoQ token-bucket indirect memory (go/rbw/poll via cg_puc_ind_write).
 * DATA1: bs[5:0] enb[7:6] mode[8] tbc[29:9] pkt_mode_class_sel[30]; DATA0: rate_k[9:0]
 * rate_m[25:10] bs[31:26].  Stock's queue_add writes both per VoQ before REPORT_ENABLE. */
#define CG_PUC_VOQ_TBC_ACCESS	(CG_PUC_BASE + 0x1c4)	/* CIR bucket: addr[7:0]=VoQ */
#define CG_PUC_VOQ_TBC_DATA1	(CG_PUC_BASE + 0x1c8)
#define CG_PUC_VOQ_TBC_DATA0	(CG_PUC_BASE + 0x1cc)
#define CG_PUC_RL_VOQ_TBC_ACCESS (CG_PUC_BASE + 0x1d0)	/* PIR (rate-limit) bucket */
#define CG_PUC_RL_VOQ_TBC_DATA1	(CG_PUC_BASE + 0x1d4)
#define CG_PUC_RL_VOQ_TBC_DATA0	(CG_PUC_BASE + 0x1d8)
#define CG_PUC_Q2PQSRCFG01	(CG_PUC_BASE + 0x230)	/* qm_rpt_lv0[15:0], lv1[31:16] */
#define CG_PUC_Q2PQSRCFG23	(CG_PUC_BASE + 0x234)	/* qm_rpt_lv2[15:0], lv3[31:16] */
#define CG_PUC_BMC_RX_PKT	(CG_PUC_BASE + 0x17c)	/* US frames received by the PUC */
#define CG_PUC_BMC_RX_PKT_ENQ	(CG_PUC_BASE + 0x180)	/* US frames enqueued to a VoQ */
#define CG_PUC_BMC_FORCE_DROP	(CG_PUC_BASE + 0x184)	/* US frames dropped (invalid VoQ) */
#define CG_PUC_US_OMCI_HDR_A	(CG_PUC_BASE + 0x160)	/* gemid[7:0],cos[10:8],tcont[21:16],datapkt[30],en[31] */
#define CG_PUC_US_OMCI_HP_HDR_A	(CG_PUC_BASE + 0x164)	/* gemid[7:0],cos[10:8],tcont[21:16] */
#define CG_PUC_GLOBAL_PLOAM_CFG	(CG_PUC_BASE + 0x168)	/* us_hdr_min_size[21:16], us_ext_omci_en[31] */

/*
 * The PUC's CONTROL-PACKET classifier and its two dedicated counters — the only
 * upstream witness in this block that does NOT also count upstream user data.
 *
 * Every upstream control frame carries the 16-byte PON control header the NI TX
 * path stamps on it (cortina_ni_pon_hdr): a fixed DA/SA pair, then a 16-bit
 * type.  The PUC holds that same pair in GLOBAL_DA_SA2/1/0 and two types to
 * classify against, and counts each group in its own counter:
 *
 *   GLOBAL_DA_SA2/1/0 = 00:13:25:00:00:00 / 00:13:25:00:00:01  (DA then SA,
 *                       big-endian across the three words — byte-identical to
 *                       the pair cortina_ni_pon_hdr puts on our OMCI PDUs)
 *   GLOBAL_LNK_TYPE   = 0xfff1  = the OMCI type, i.e. the type the NI stamps
 *                                 on every upstream OMCI PDU
 *   GLOBAL_MAC_TYPE   = 0xfff0  = the companion MAC-layer control type
 *   BMC_CONTROL_PKT_CNTR_lnk / _mac = the two matching frame counts
 *   BMC_LENGTH_ERROR                = US frames rejected on length
 *
 * ⇒ _lnk is an OMCI-SPECIFIC upstream frame count, and the vendor treats it as
 * exactly that (its "OMCI packet count" accessor reads this register and
 * nothing else), whereas BMC_RX_PKT is the TOTAL — data and control together.
 * It is the one instrument here that upstream user data cannot inflate.
 *
 * Widths differ inside the group and it matters: BMC_RX_PKT/_ENQ are 32-bit,
 * but FORCE_DROP, LENGTH_ERROR and both CONTROL_PKT counters are cntr:16 with
 * a reserved upper half, which stock masks off on every read.
 *
 * All of them are CLEAR-ON-READ (PUCCFG.inccfg=2) and the block drops them
 * after a short idle window, so exactly one reader in this driver may touch the
 * three control-packet registers — see cg_puc_ctrl_sample().
 *
 * ★ One thing is NOT established: whether the type match is the 16-bit type
 * alone or a 32-bit compare that also covers the two header bytes after it.
 * Our OMCI PDUs carry 0xff 0xf1 0x00 0x01 there (byte 15 = the cos>6 flag)
 * while the register reads 0xfff10000, so under the 32-bit reading our own
 * frames would NOT be classified and _lnk would stay 0 for a reason that has
 * nothing to do with the upstream path.  A vendor OMCI counter that reads 0 on
 * every unit of this generation is implausible, and the two type registers'
 * low halves are 0 while the vendor's OMCI ethertype is a 16-bit 0xfff1 — but
 * "implausible" is not "measured".  So the only thing that settles it is
 * watching us_omci while the responder transmits, and until it has been seen
 * to move on a WORKING board, a zero here must never be read as a defect.
 */
#define CG_PUC_GLOBAL_DA_SA2	(CG_PUC_BASE + 0x14c)	/* DA[0..3] */
#define CG_PUC_GLOBAL_DA_SA1	(CG_PUC_BASE + 0x150)	/* DA[4..5], SA[0..1] */
#define CG_PUC_GLOBAL_DA_SA0	(CG_PUC_BASE + 0x154)	/* SA[2..5] */
#define CG_PUC_GLOBAL_MAC_TYPE	(CG_PUC_BASE + 0x158)	/* type:32, dft 0xfff00000 */
#define CG_PUC_GLOBAL_LNK_TYPE	(CG_PUC_BASE + 0x15c)	/* type:32, dft 0xfff10000 (OMCI) */
#define CG_PUC_BMC_CTRL_PKT_MAC	(CG_PUC_BASE + 0x174)	/* cntr:16, MAC-type control frames */
#define CG_PUC_BMC_CTRL_PKT_LNK	(CG_PUC_BASE + 0x178)	/* cntr:16, OMCI-type control frames */
#define CG_PUC_BMC_LENGTH_ERROR	(CG_PUC_BASE + 0x188)	/* cntr:16, US length-check rejects */
#define CG_PUC_BMC_CNTR_MASK	0xffff	/* the cntr:16 fields' reserved upper half */
/*
 * ★★★★ 2026-08-08 fix#55: the PUC's view of the LAST upstream control frame.  These are
 * the only US-egress witnesses with a real ORACLE: the live-stock golden
 * (session_2026-08-06c/GOLDEN_STOCK_OAM_ROUTE.txt) captured
 *      hdr0 (PUC_BMC_PKT_HDR0, 0xf5508198) = 0x00203108   <- OMCI at the PUC, correct
 *      hdr1 (PUC_BMC_PKT_HDR1, 0xf5508194) = 0x40200000
 * and Track A's wall is precisely that ITS hdr0 reads 0xc0203708 - hdr_type=3, the L2FE
 * DLF re-stamp - instead of the golden.  us_omcc has no oracle (stock never reads it), so
 * judge US egress by THESE, not by us_omcc alone.
 */
#define CG_PUC_BMC_PKT_HDR1	(CG_PUC_BASE + 0x194)	/* golden 0x40200000 */
#define CG_PUC_BMC_PKT_HDR0	(CG_PUC_BASE + 0x198)	/* golden 0x00203108 */
#define  CG_PUC_HDR0_GOLDEN	0x00203108u
#define CG_PUC_LNK_TYPE_OMCI	0xfff1

/*
 * The PUC<->US-scheduler interface control.  ★ On this silicon it is at
 * PON+0x6e00, NOT at the PON+0x4fe0 that a vendor source path names: that
 * window does not decode here (every word from 0x4fc0 to 0x500c reads one and
 * the same constant, unchanged by a write to it) and this board's stock
 * firmware has no register anywhere in PON+0x4xxx.  Read for /proc only — the
 * driver configures nothing here.
 *
 * Field layout: cntr_tconid[4:0], cntr_tconid_en[5], cntr0/1/2_event_sel (3
 * bits each), single_thread, sch_to_threshold[27:16], cntr_inccfg[31:29].  Two
 * facts worth keeping: cntr_inccfg is 0 at reset, which is why the three
 * counters at +0x04/08/0c are free-running rather than clear-on-read (one of
 * them ticks at the 8 kHz GPON upstream frame rate — what each selects is not
 * established); and stock's GPON path sets sch_to_threshold=1000 here, which
 * this driver does NOT (its write went to the 0x4fe0 hole above, so the field
 * has always sat at its reset 64).  That divergence is REPORTED, not silently
 * "fixed": the upstream path works as it is, and re-tuning the US scheduler is
 * not a change to make as a side effect of exposing a counter.
 */
#define CG_GPON_MAC_PUCIF_CTRL	0x6e00	/* dft 0x0040a100 */

/*
 * GPON-MAC statistics counters (MAC-block-relative, i.e. the SILICON offsets —
 * this board's own register table etc/reg.txt is already the silicon view, as
 * its alarm@0x9c / interrupt_top@0xa4 / onu@0xdc entries match the live-proven
 * offsets used above, so no +0x20 header shift applies to these names).
 *
 * ★ SEMANTICS, and why they are read RAW with no accumulator (the opposite of
 * the PUC control counters): these are ACCUMULATING counters that are cleared
 * by SOFTWARE WRITING ZERO, not by being read.  Stock proves it two ways —
 * its aal_gpon_port_stats_clear() writes 0 to exactly this set, and its
 * aal_gpon_current_bip_error_get() reads the BIP pair and then explicitly
 * zeroes it, which would be redundant if a read self-cleared.  So a plain read
 * is idempotent and any number of concurrent readers is safe: DO NOT convert
 * these into clear-on-read deltas, and DO NOT ever write them from here — a
 * write would destroy the history every other reader (and the test suite)
 * depends on.  Clearing is an explicit operator action, never a read side
 * effect.
 *
 * ★ The DS MIB group at silicon 0x084..0x094 belongs to the SAME accumulating,
 * software-cleared family, correcting the "clear-on-read" descriptor in the
 * interrupt-block note above.  Two independent tiers say accumulating: stock's
 * aal_gpon_port_stats_get() reads them plainly as a statistics API (which would
 * be self-destroying if a read cleared) and aal_gpon_port_stats_clear() zeroes
 * them explicitly; and a live stock capture shows large retained values
 * (ds_omci_gem == ds_omci_pkt == 1127, bip_error_frame_count == 0xF9991).  What
 * the note got right, and what actually caused the incident it records, is that
 * WRITING here corrupts the US PLOAM engine — so these are read, never written.
 */
/* ★AOT fix#12 (2026-08-20): the DS-MIB block is the X400AXF map shifted -0x20 on
 * this silicon, like OMCI_PORT/TCONT_DATA/ALARM/T3_PREAMBLE.  GROUND TRUTH: stock
 * ca-ne.ko aal_gpon_port_stats_get (0xe9580) reads this block at pon+0x605c,
 * 0x6060, 0x6064, 0x6068, 0x606c, 0x6070, 0x6074 (= mac 0x05c..0x074) =
 * {BIP_ACCUM, BIP_FRAMES, DS_OMCI_GEM, DS_OMCI_PKT, DS_PKT_CRC, DS_UNDERSIZE,
 * DS_OVERSIZE} in order.  The OLD 0x084/0x088 offsets read an ALIASED CONSTANT
 * (0x088 stuck at 0xf) and FALSELY implied "OLT sent 15 DS OMCI" - the real
 * DS_OMCI_PKT (0x068) reads 0 (Airtel sends no OMCI to the clone).  The shift
 * also frees 0x07c, resolving the old BIP_ERR_ACCUM/ALARM collision. */
#define CG_REG_BIP_ERR		0x058	/* was 0x078; BIP-8 errors of last superframe (inferred -0x20; stock stats_get confirms 0x05c..0x074) */
#define CG_REG_BIP_ERR_ACCUM	0x05c	/* was 0x07c; accumulated BIP-8 errors   */
#define CG_REG_BIP_ERR_FRAMES	0x060	/* was 0x080; frames BIP accumulated over */
#define CG_REG_DS_OMCI_GEM	0x064	/* was 0x084; DS OMCI GEM frames (hw count) */
#define CG_REG_DS_OMCI_PKT	0x068	/* was 0x088; DS OMCI packets (hw count)  */
#define CG_REG_DS_PKT_CRC	0x06c	/* was 0x08c; DS packets failing CRC     */
#define CG_REG_DS_UNDERSIZE	0x070	/* was 0x090; DS undersized packets      */
#define CG_REG_DS_OVERSIZE	0x074	/* was 0x094; DS oversized packets       */
#define CG_REG_SUPERFRAME	0x0fc	/* 125 us superframe counter             */
#define CG_REG_US_OMCC_CNT	0x1e0	/* ★AOT fix#12: was 0x200 (X400AXF map) */	/* upstream OMCC frames, GPON-MAC side.
					 * Present in this board's register map
					 * but stock never reads it, so there is
					 * NO stock oracle: treat as unvalidated
					 * until seen to move.  Worth having —
					 * it is a second, independent angle on
					 * the upstream-OMCI question the PUC
					 * _lnk counter leaves open. */
#define CG_REG_PUCIF_PROTECT	0xe14	/* b0 = PUCIF hang LATCHED, b5:1 = the
					 * T-CONT id that hung.  Stock's periodic
					 * monitor logs "pucif_hang_tcon_id:%d"
					 * and clears by writing 0.  We read it
					 * WITHOUT clearing: a sticky "ever hung"
					 * plus the offender's id costs nothing
					 * and cannot perturb stock-matching
					 * behaviour.  (Clearing would let us
					 * count episodes, but it makes this
					 * driver a mutator of upstream state,
					 * which is not worth it for a witness.) */
#define CG_REG_O5		0x1a8	/* O5-related count (semantics unproven) */
#define CG_REG_GEM_FRAG_DROP	0x1ac	/* DS GEM fragments dropped              */
#define CG_REG_GEM_1BITERR	0x1b0	/* GEM header 1-bit errors (corrected)   */
#define CG_REG_GEM_2BITERR	0x1b4	/* GEM header 2-bit errors               */
#define CG_REG_GEM_UNCORR	0x1b8	/* GEM header uncorrectable errors       */
#define CG_REG_BWMAP_DROP	0x1bc	/* upstream BWmap entries dropped        */
#define CG_REG_OMCI_CRC		0x1c0	/* DS OMCI CRC failures                  */
#define CG_REG_PLEND_ERR	0x1c4	/* PLend field errors                    */
#define CG_REG_PLEND_BITERR	0x1c8	/* PLend bit errors                      */
#define CG_REG_DS_ASMBL_DROP	0x1cc	/* DS reassembly-FIFO drops              */
#define CG_REG_BWMAP_UNCORR	0x1f8	/* BWmap uncorrectable bit errors        */
#define CG_REG_BWMAP_CORR	0x1fc	/* BWmap corrected bit errors            */
/* FEC block.  The five counters have no clear function in stock, so whether
 * they self-clear is NOT established — they are published raw and labelled
 * accordingly rather than presented as cumulative totals. */
#define CG_REG_FEC_CTRL		0x800
#define CG_REG_FEC_MISC_STATUS	0x804
#define CG_REG_FEC_CORR_BLK	0x808	/* correctable FEC blocks   */
#define CG_REG_FEC_UNCORR_BLK	0x80c	/* uncorrectable FEC blocks */
#define CG_REG_FEC_CLEAN_BLK	0x810	/* error-free FEC blocks    */
#define CG_REG_FEC_BLK_TOTAL	0x814	/* total FEC blocks         */
#define CG_REG_FEC_CORR_BYTES	0x818	/* bytes corrected by FEC   */

#define CG_PUC_TCONT_NUM	32	/* AAL_GPON_SYSTEM_MAX_TCONT_NUM */
#define CG_PUC_QUEUE_PER_TCONT	8	/* 8Q mode */
#define CG_PUC_9TH_QUEUE_VOQ	127	/* the CPU high-prio inject VoQ (ldpid 0xf, cos 7) */

/*
 * ★ THE TWO UPSTREAM SLOT RANGES, DECLARED ONCE (shared vocabulary:
 * struct gpon_gem_us_range, drivers/net/gpon/gpon_gem_us.h).
 *
 * An upstream GEM Port-ID is stamped onto the burst by writing it into a run of
 * consecutive slots of the US port-map array, and the two chip families number
 * those slots on INCOMPATIBLE principles.  On this Cortina part the slot number
 * IS the VoQ, so a T-CONT's slots are tcont * CG_PUC_QUEUE_PER_TCONT .. +7; on
 * Luna a slot is a fixed GTC flow/SID per role with no arithmetic relationship
 * to any T-CONT at all (OMCC = {64, 1}, data = {1, 1}).  `index = tcont * 8` is
 * therefore TRUE here and FALSE there, which is exactly why the common layer
 * carries no function that derives a base from a T-CONT: each shell DECLARES
 * its own map, and the shared code only ever walks a declared one.
 *
 * Declaring them here also makes the three loops that stamp these slots read
 * ONE statement of the map instead of re-deriving `base + i` three times, and
 * makes the CG_DATA_GEM_IDX / CG_PUC_QUEUE_PER_TCONT coupling checkable: the
 * static_asserts below cost nothing at run time and fail the BUILD if a range
 * ever runs past the array the index field can address.
 */
static const struct gpon_gem_us_range cg_us_omcc_slots = {
	.base		= 0,				/* us hw gems 0..7 */
	.count		= CG_OMCC_US_GEM_IDX_NUM,
	.index_max	= CG_US_PORT_IDX_MAX,
};
static const struct gpon_gem_us_range cg_us_data_slots = {
	.base		= CG_DATA_GEM_IDX,		/* = VoQ 8..15 */
	.count		= CG_PUC_QUEUE_PER_TCONT,
	.index_max	= CG_US_PORT_IDX_MAX,
};
static_assert(GPON_GEM_US_RANGE_OK(0, CG_OMCC_US_GEM_IDX_NUM,
				   CG_US_PORT_IDX_MAX),
	      "OMCC upstream slot range runs past the US port-map array");
static_assert(GPON_GEM_US_RANGE_OK(CG_DATA_GEM_IDX, CG_PUC_QUEUE_PER_TCONT,
				   CG_US_PORT_IDX_MAX),
	      "data upstream slot range runs past the US port-map array");
/*
 * ★ RECORDED, NOT FIXED (found while wiring this, 2026-08-05): the two halves
 * of the data range are coupled by a LITERAL.  CG_DATA_GEM_IDX is
 * `CG_DATA_TCONT_IDX * 8` while the run length is CG_PUC_QUEUE_PER_TCONT, so
 * changing the queues-per-T-CONT constant alone moves the length without moving
 * the base and the two silently disagree.  Both are 8 today and this refactor is
 * code motion, so it is written down here rather than repaired in the same step
 * — a fix that rides a move is a regression nobody can bisect.
 */
/*
 * onu_cfg (hdr 0x118 -> silicon +0x138).  Top byte laser_on_align=0x12 aligns
 * the upstream laser burst to the OLT's grant window; at the reset default
 * (0x00100780, laser_on_align=0) the burst is mis-aligned and the OLT cannot
 * decode the SerialNumber, so ranging stalls at O1.  This is range-critical.
 * laser_pre_bias[11:7]=18 (reset dft 15): stock at Online reads 0x12100900 —
 * the vendor raises the burst pre-bias from a rodata config blob.
 */
#define CG_REG_ONU_CFG_REAL	0x118	/* ★AOT fix#12: was 0x138 (X400AXF map) */
#define CG_ONU_CFG_VAL		0x12100900
/*
 * The activation control register: the header calls it onu_ctl at +0x114, but on
 * this CA8277C silicon it is at +0x134 (the +0x20 shift, measured live: stock
 * reads +0x134 = 0x00460262 with the enable bit set at O5, while +0x114 reads 0).
 * Bit1 = en -> the MAC autonomously ranges O1->O5.  We write the full stock value.
 */
#define CG_REG_ONU_CTL	0x114	/* ★AOT fix#12: was 0x134 (X400AXF map) */
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
#define CG_REG_GPON_MAC_CTRL	0x1c4	/* ★AOT fix#12: was 0x1e4 (X400AXF map) */
#define CG_MAC_CTRL_SW_RANDOM_EN BIT(16)
#define CG_MAC_CTRL_PTI_OMCI	BIT(17)

/* one post-O5 servicing event, snapshotted in hardirq, handled in the work */
struct cg_evt {
	u32 intr;			/* INTERRUPT (0x8c) sources, already enable-masked */
	u8 state;			/* onu.state at IRQ time */
	u8 id;				/* onu.id at IRQ time */
};

#define CG_EVT_RING_SZ		16	/* power of 2 */

/*
 * Where the ONU's G.984.3 serial number came from, strongest first.  The serial
 * number is the ONU's PON IDENTITY: the OLT keys ranging, authentication and the
 * whole service profile on it, so two units announcing the same serial number
 * collide on one PON.  It must therefore be read FROM THE BOARD and never be a
 * compiled-in literal -- see the cg_sn_* block below for the provisioning path.
 */
enum cg_sn_src {
	CG_SN_NONE = 0,		/* not provisioned yet: ranging is held off */
	CG_SN_PARAM,		/* cortina_gpon.sn= (bring-up / A-B override) */
	CG_SN_BOARD,		/* the board's own factory data, via /proc/gpon */
	CG_SN_FALLBACK,		/* nothing readable: a placeholder, NOT an identity */
};

static const char *const cg_sn_src_name[] = {
	"NONE", "module-param", "board", "FALLBACK",
};

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
	bool omcc_alloc_valid;		/* omcc_alloc actually carries a binding.
					 * G.984.3 ONU-ID 0 is LEGAL, so 0 cannot
					 * double as "never bound": without this
					 * flag an ONU-ID of 0 makes the live-HW
					 * reconcile below think the OMCC T-CONT is
					 * already bound and never replay a lost
					 * Assign_ONU-ID (no US grant -> no OMCI
					 * answer -> OLT Deactivate). */
	u16 omcc_gem;			/* last omci_port.id bound to us-gem 0..7 */

	/* DS OMCI receive (Stage B: count + decode-log; responder = Stage C) */
	u32 omci_rx;			/* DS OMCI PDUs delivered by the NI CPU-RX hook */
	u32 omci_rx_short;		/* runt PDUs (< 8 bytes, not decodable) */
	bool pdc_ready;			/* PDC map + CTRL programmed */
	bool puc_ready;			/* PUC US-VoQ admission programmed */

	/*
	 * The PUC control-packet counters, made CUMULATIVE in software.  The
	 * hardware counters are clear-on-read and hold only a short window, so
	 * a snapshot of them says nothing on an idle device; summing the deltas
	 * does.  Exactly one function reads those registers
	 * (cg_puc_ctrl_sample), which is what lets any number of concurrent
	 * readers of /proc/gpon ADD to these totals instead of stealing from
	 * them.
	 */
	spinlock_t puc_cnt_lock;	/* serializes the read-and-add */
	struct delayed_work puc_cnt_work;	/* samples shortly after a US OMCI TX */
	u32 puc_omci_us;		/* upstream OMCI (link-type 0xfff1) frames */
	u32 puc_ctrl_mac;		/* upstream MAC-type control frames */
	u32 puc_len_err;		/* upstream frames failing the length check */
	u32 puc_cnt_samples;		/* reads folded in (0 = never sampled) */

	/* Stage C: the G.988 OMCI responder + US OMCI TX */
	struct omci_onu *omci;		/* responder context (kzalloc'd at probe) */
	spinlock_t omci_lock;		/* RX hook (softirq) vs isr_work/AVC work */
	bool omci_active;		/* ctx armed (OMCC up) */
	struct delayed_work veip_avc_work;	/* the ~31s post-O5 VEIP oper-up AVC */
	struct work_struct omci_ping_work;	/* on-demand AVC burst: the us_mib control */
	struct delayed_work stats_work;		/* ★AOT: periodic OMCI counter dump */
	unsigned int veip_avc_retry_ms;	/* backoff after a failed AVC TX; 0 = none pending */
	struct delayed_work coldstart_work;	/* stuck-O1 US-lock-miss recovery */
	int coldstart_tries;		/* re-rolls THIS stuck episode (reset on leaving O1) */
	u32 coldstart_rolls;		/* total re-rolls this power-on (/proc visibility) */
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
	/*
	 * HW CAM identity currently ARMED in silicon for the data path, tracked
	 * separately from the dt_/dg_ OLT-provisioned shadow above: what
	 * cg_data_try_install last wrote into the T-CONT / DS-GEM / US-PORT CAMs.
	 * A re-range/reconfig that installs a genuinely DIFFERENT alloc/gem must
	 * invalidate these stale predecessors FIRST (cg_data_teardown, vendor
	 * drain-then-clear order) so a reassigned alloc can never burst into
	 * another ONU's grant slot; a same-{alloc,gem} state is left untouched
	 * (no HW writes -> no re-provision churn, the proven fiber-pull path).
	 * The OMCC's armed alloc is tracked by omcc_alloc above.
	 */
	u16 hw_data_alloc;		/* alloc-id armed -> hw T-CONT 1 (0 = none) */
	u16 hw_data_gem;		/* GEM port-id armed in DS-GEM CAM + US_PORT (0 = none) */
	u32 omci_cfg_log;		/* config-ME body log budget used */
	struct net_device *wan_ndev;	/* gpon0 */

	/*
	 * The per-board PON identity, single source of truth for BOTH the MAC's
	 * vendor-id/vendor-specific registers and the OMCI responder's ME-256
	 * serial number: they can no longer disagree by construction.
	 */
	struct mutex sn_lock;		/* serializes sn/sn_src/activated + activation */
	u8 sn[8];			/* wire order: 4 ASCII vendor-id + 4 VSSN bytes */
	enum cg_sn_src sn_src;
	bool activated;			/* cg_mac_activate() has run at least once */
	struct delayed_work sn_wait_work;	/* bounded wait for the board's serial */
};

static struct cortina_gpon *cg_singleton;

static bool cg_do_reset = true;
module_param_named(reset, cg_do_reset, bool, 0444);

static int cg_stats_s;
module_param_named(stats_s, cg_stats_s, int, 0644);
MODULE_PARM_DESC(stats_s, "seconds between OMCI counter dumps (0 = off)");

MODULE_PARM_DESC(reset, "release the GPON MAC from reset/clock-gate at probe (default on)");
static bool cg_glbdump;	/* fix#110 diag: dump the whole glb block to diff vs stock */
module_param_named(glbdump, cg_glbdump, bool, 0444);
MODULE_PARM_DESC(glbdump, "fix#110 diag: dump glb 0x00-0x2fc at probe (find the QM-domain ungate)");
static uint qm_glbfix;	/* fix#110: write stock's glb clock/reset-region values (captured live)
			 * that our bring-up leaves at bootloader default, then probe QM 0x8274. */
module_param(qm_glbfix, uint, 0444);
MODULE_PARM_DESC(qm_glbfix,
	"fix#110: match glb clock/reset region to live stock + read-test QM 0x8274. =1 all, "
	"=2 only clear 0x0a8 (the reset the port leaves asserted). Oracle: no sync-abort = domain alive.");

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
	/* ★ AOT5221ZY: RMW clearing ONLY POW_PCIX (bit5), matching stock aal_psds_out_of_reset
	 * (`and w0, #0xffffffdf`) and our shipping driver - not an absolute write of 0x1. */
	writel(readl(glb + CG_GLB_PSDS_INIT) & ~CG_PSDS_POW_PCIX, glb + CG_GLB_PSDS_INIT);
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
MODULE_PARM_DESC(activate, "program the SN + start GPON ranging once the serial number is known (default on)");

/*
 * ===========================================================================
 * The per-board GPON serial number (G.984.3 ONU-ID / "VSSN")
 * ===========================================================================
 *
 * The serial number is 8 bytes on the wire: 4 ASCII vendor-id characters
 * followed by 4 binary vendor-specific bytes ("XPON" + 5C 6C AF CB reads as
 * XPONxxxxxxxx).  It is the ONU's identity on the PON, so it MUST come from the
 * board, exactly like the factory MAC -- a compiled-in serial number makes every
 * unit flashed with the same image announce one identity, which collides on a
 * shared PON and breaks OLT provisioning/authentication.
 *
 * Where it lives on this board (tier-1 live read 2026-07-16, corroborated tier-2
 * by the stock userspace):
 *   NAND "ubi_device" -> UBI volume "ubi_Config" -> config_hs.xml:
 *       <Value Name="GPON_SN" Value="XPONxxxxxxxx"/>
 *   the same file and volume that carries ELAN_MAC_ADDR (the factory base MAC).
 * Stock reads it from there in USERSPACE and hands it to its PON stack: rc2
 * mounts ubi0:ubi_Config on /var/config, and runomci.sh does `mib get GPON_SN`
 * (the MIB store is that XML) and passes it as `omci_app -s <SN>`; a second
 * consumer splits the same string into vendor-id (chars 1-4) and VSSN (chars
 * 5-12).  Nothing derives it from the MAC -- the vendor's built-in default is a
 * generic "RTKG11111111" -- so the fact that this unit's VSSN happens to share
 * three bytes with its MAC is factory numbering, not a rule, and is NOT used.
 * Not a source either: the U-Boot env (generic placeholder), the runtime DTB (no
 * bootloader fixup), the OTP (PCIe calibration only) or static_conf (blank).
 *
 * So the kernel cannot read it at probe (the volume is UBIFS, mountable only
 * once userspace runs), and this mirrors the MAC path (05_factory_mac) and stock
 * itself: userspace reads the board and pushes the value in.
 *   /etc/init.d/gpon-identity  ->  echo "sn XPONxxxxxxxx" > /proc/gpon
 * The driver holds ranging off until it has a serial number, then programs it and
 * starts the FSM.  If nothing arrives within CG_SN_WAIT_SECS it shouts and ranges
 * with a deliberately non-identity placeholder so the box never silently sits
 * dark; a real serial number arriving later re-activates with it.
 */
#define CG_SN_WAIT_SECS		60

/*
 * The placeholder used when the board's serial number cannot be read at all.
 * Vendor-id "XPON" is the fleet-wide vendor code (not per-unit) so the OLT still
 * logs a parseable unknown ONU; the all-ones VSSN is the blank-flash value and
 * can never be a factory-programmed unit, so this can never be mistaken for -- or
 * collide with -- a provisioned board.  It is always accompanied by a dev_err and
 * by "sn-source = FALLBACK" in /proc/gpon.
 */
static const u8 cg_sn_unprovisioned[8] = { 'X', 'P', 'O', 'N', 0xff, 0xff, 0xff, 0xff };

static char *cg_sn_param;
module_param_named(sn, cg_sn_param, charp, 0444);
MODULE_PARM_DESC(sn, "GPON serial number override, \"VVVVHHHHHHHH\" (4 ASCII vendor-id chars + 8 hex VSSN digits). Bring-up/A-B use ONLY: the shipping path is the board's own config volume pushed in by /etc/init.d/gpon-identity, so never bake a serial number into an image's bootargs");

/* ★ 2026-08-08 AOT5221ZY: GPON registration password, as three raw register
 * words in wire order (pw0 = first 2 bytes in its low half, pw1/pw2 = 4 each).
 * All three default to 0 = "leave the password registers alone", which is the
 * behaviour this driver had before, so an OLT that does not ask for one is
 * unaffected.  Example (ZYXE123456):
 *   cortina_gpon.pw0=0x5a59 pw1=0x58453132 pw2=0x33343536
 */
static uint cg_pw0, cg_pw1, cg_pw2;
module_param_named(pw0, cg_pw0, uint, 0444);
module_param_named(pw1, cg_pw1, uint, 0444);
module_param_named(pw2, cg_pw2, uint, 0444);
MODULE_PARM_DESC(pw0, "GPON registration password bytes 0-1 (low half of PFRAG0); 0 with pw1/pw2 = do not program a password");
MODULE_PARM_DESC(pw1, "GPON registration password bytes 2-5 (PFRAG1)");
MODULE_PARM_DESC(pw2, "GPON registration password bytes 6-9 (PFRAG2)");

/* One 32-bit register value from 4 wire-order bytes (endianness-agnostic). */
static u32 cg_sn_word(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static int cg_hex_nibble(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

/*
 * "VVVVHHHHHHHH" -> the 8 wire bytes.  Rejects anything that is not exactly 4
 * printable non-space vendor-id characters followed by 8 hex digits, so a
 * truncated/garbled provisioning read can never be programmed as an identity.
 */
static int cg_sn_parse(const char *s, u8 out[8])
{
	int i, hi, lo;

	if (!s || strlen(s) != 12)
		return -EINVAL;
	for (i = 0; i < 4; i++) {
		if (!isprint(s[i]) || isspace(s[i]))
			return -EINVAL;
		out[i] = s[i];
	}
	for (i = 0; i < 4; i++) {
		hi = cg_hex_nibble(s[4 + 2 * i]);
		lo = cg_hex_nibble(s[5 + 2 * i]);
		if (hi < 0 || lo < 0)
			return -EINVAL;
		out[4 + i] = (hi << 4) | lo;
	}
	return 0;
}

/* The 8 wire bytes -> the printable "VVVVHHHHHHHH" form. */
static void cg_sn_format(const u8 sn[8], char out[13])
{
	snprintf(out, 13, "%c%c%c%c%02X%02X%02X%02X",
		 sn[0], sn[1], sn[2], sn[3], sn[4], sn[5], sn[6], sn[7]);
}

static bool cg_do_bosa_init = true;
module_param_named(bosa_init, cg_do_bosa_init, bool, 0444);
MODULE_PARM_DESC(bosa_init, "program the GN25L95 BOSA laser driver over per_i2c before ranging (default on; off = no upstream burst, DS-side diagnostics only)");

static bool cg_coldstart_wd = true;
module_param_named(coldstart_wd, cg_coldstart_wd, bool, 0644);
MODULE_PARM_DESC(coldstart_wd, "stuck-O1 recovery watchdog: re-roll the SerDes/laser bring-up while the FSM sits at O1 (default on; 0 = observe-only A/B baseline — flip live via /sys/module to recover a wedged boot in place)");

/* ★ 2026-07-23 bisection/US-offload-first knob: under hw_l3_fwd, route the DS
 * unicast data GEM into the L3FE (LDPID L3_WAN) so the DS direction can HW-
 * forward.  Default OFF because that DS route BREAKS the wired LAN once the WAN
 * data-path installs (host->ONU ping 100% loss; offload-OFF baseline 0% loss) -
 * a still-open dynamic bug.  With it OFF, DS keeps the proven CPU_0+FE_BYPASS
 * delivery (SW fastpath) while only the US (LAN->WAN) direction attempts HW
 * offload; flip =1 to resume the DS-into-L3FE bring-up. */
/*
 * ★★ 2026-07-25 - THIS GATE IS THE DS-OFFLOAD PRECONDITION, and keeping it off
 * while cortina_ni.hw_ds_offload=1 makes the DS measurement meaningless.
 * With it OFF the data GEM's PDC entry carries FE_BYPASS, so the DS frame is
 * handed straight to CPU port 0 and never visits the L2FE or the L3FE: no DS
 * main-hash entry can be hit, l3fe_rx stays 0, downstream throughput is exactly
 * the CPU-forward baseline, and none of that says anything about the DS hash
 * key or the DS egress action.  (That is precisely the boot measured on
 * 2026-07-24: DS entries installed, ds_flows 2, zero hits.)  The offload
 * backend now reports the route to /proc/cortina_l3fe (ds_pdc=) and warns at
 * arm time so the two gates can no longer be armed half-way by accident.
 *
 * The original reason for OFF is now partly obsolete: it read "DS-into-L3FE is
 * premature - DS CPU-punts (no HW-forward until A2) and starves the shared L3QM
 * CPU pool under sustained load, killing LAN".  A2 has landed (the DS leg
 * carries a real next-hop rewrite), so once a flow is offloaded its DS packets
 * never reach the L3QM CPU pool at all - the starvation premise is what the DS
 * offload removes.  What is NOT yet retested: the PUNT WINDOW (the first
 * packets of every flow, plus any DS traffic that is not an offloadable
 * TCP/UDP 5-tuple) still traverses L3FE -> L3QM -> CPU, which is the path that
 * broke the wired LAN (host->ONU ping 100% loss) when DS could ONLY punt.  So
 * flipping this on is the right next experiment but must be run WITH a LAN
 * health check in the same window, not as a shipping default.
 */
/* Default ON since 2026-07-25: routing the downstream data GEM into the L3FE
 * (instead of CPU_0 + FE_BYPASS) is what lets the DS HW-flow leg be hit at all.
 * Measured with it on: DS 956.2 Mbps at 0.4% ONU CPU (was 642 Mbps with a core
 * pegged), upstream unchanged at 956.3 Mbps.  Set cortina_gpon.hw_l3_ds=0 to
 * fall back to the CPU punt path. */
static bool cg_hw_l3_ds = true;
module_param_named(hw_l3_ds, cg_hw_l3_ds, bool, 0644);
MODULE_PARM_DESC(hw_l3_ds, "route the DS data GEM into the L3FE under hw_l3_fwd (default OFF = CPU_0 + FE_BYPASS). ★ REQUIRED for cortina_ni.hw_ds_offload to do anything: with it off, DS frames bypass both forwarding engines and no DS hash entry is reachable. Watch the wired LAN when enabling (the DS punt window once broke it)");

/*
 * fix#70: the PUC's US-OMCI HEADER_A replacement values.  Normal path (matched OMCI
 * control frame): enable[31] | tcont[21:16] | cos[10:8] | gemid[7:0].  High-priority
 * path (what the ldpid-15 / 9th-queue inject takes) has the same layout minus the
 * enable bit.  Stock's normal value is 0x80000606; the HP one used to be set to
 * 0x0707 here, which stamps the PLOAM code point (cos 7) onto our OMCI reply.
 */
static uint cg_mac_stock = 0xfff;
module_param_named(mac_stock, cg_mac_stock, uint, 0444);
MODULE_PARM_DESC(mac_stock,
	"fix#73: bitmask of GPON-MAC config regs to force to the live-stock capture (0=off)");

static bool cg_puc_stock = true;
module_param_named(puc_stock, cg_puc_stock, bool, 0444);
MODULE_PARM_DESC(puc_stock,
	"fix#72: force the PUC config registers to the values captured from live stock while it was emitting");

static uint cg_us_omci_hdr_a = 0x80000606;
module_param_named(us_omci_hdr_a, cg_us_omci_hdr_a, uint, 0644);
MODULE_PARM_DESC(us_omci_hdr_a, "PUC US_OMCI_HDR_A replacement (en|tcont|cos|gemid)");
static uint cg_us_omci_hp_hdr_a = 0x00000707;	/* fix#72: live stock reads 0x707 here */
module_param_named(us_omci_hp_hdr_a, cg_us_omci_hp_hdr_a, uint, 0644);
MODULE_PARM_DESC(us_omci_hp_hdr_a,
		 "fix#70: PUC US_OMCI_HP_HDR_A replacement for the 9th-queue inject (default 0x606 = gemid 6/cos 6; 0x707 = the old PLOAM code point)");

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
/* ★ 2026-08-07 AOT5221ZY laser enable — THIS BOARD's sequence, replacing the
 * X400AXF one (GLB pinroute 0x42c + GPIO mux groups 0/3/4 + pins 6/11/12).
 * On this board glb+0x42c DOES NOT DECODE: reading it aborts in cg_mac_activate+0x30
 * ("synchronous external abort", reproduced every boot), and this board's GPIO0
 * registers are at 0x2c0/0x2c4/0x2c8, not the X400AXF's 0x300/0x304.
 * This board's stock tx_disable_gpio is bank 0 bit 15; the shipping 6.6 driver drives
 * exactly this and reaches O5 on this fibre. Order matters: drive OUT low FIRST, then
 * clear CFG (0 = output), so the pad goes low the instant it becomes an output and the
 * GN25L95's hardware TX_DIS is de-asserted before the BOSA i2c stream runs. */
#define CG_AOT_LASER_PIN	15
#define CG_AOT_GPIO0_CFG	0x2c0
#define CG_AOT_GPIO0_OUT	0x2c4
#define CG_AOT_GPIO0_IN		0x2c8

static void cg_laser_on(struct cortina_gpon *cg)
{
	u32 cfg, out, in;

	if (!cg->gpio)
		return;
	cfg = readl(cg->gpio + CG_AOT_GPIO0_CFG);
	out = readl(cg->gpio + CG_AOT_GPIO0_OUT);
	in  = readl(cg->gpio + CG_AOT_GPIO0_IN);
	dev_info(cg->dev, "laser: pin%d pre  CFG=%08x OUT=%08x IN=%08x\n",
		 CG_AOT_LASER_PIN, cfg, out, in);
	writel(out & ~BIT(CG_AOT_LASER_PIN), cg->gpio + CG_AOT_GPIO0_OUT);
	writel(cfg & ~BIT(CG_AOT_LASER_PIN), cg->gpio + CG_AOT_GPIO0_CFG);
	cfg = readl(cg->gpio + CG_AOT_GPIO0_CFG);
	out = readl(cg->gpio + CG_AOT_GPIO0_OUT);
	in  = readl(cg->gpio + CG_AOT_GPIO0_IN);
	dev_info(cg->dev, "laser: pin%d post CFG=%08x OUT=%08x IN=%08x (TX_DIS de-asserted)\n",
		 CG_AOT_LASER_PIN, cfg, out, in);
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
	unsigned int dead = 0;		/* upstream 2bb7081: entries this pass could not write */

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
			dead++;		/* upstream 2bb7081: keep going, program PDC_CTRL */
	}

	ctrl = readl(cg->pon + CG_PDC_CTRL);
	ctrl &= ~CG_PDC_CTRL_HP_MASK;
	ctrl |= CG_PDC_CTRL_MAP_MEM_EN | CG_PDC_CTRL_HP_EN |
		(7 << CG_PDC_CTRL_HP_COS_SH) |
		(CG_LPORT_CPU_0 << CG_PDC_CTRL_HP_LDPID_SH);
	writel(ctrl, cg->pon + CG_PDC_CTRL);

	cg->pdc_ready = (dead == 0);	/* upstream 2bb7081: ready = every entry landed */
	if (dead)
		dev_warn(cg->dev, "PDC: %u map entr%s failed - NOT ready, O5 supervisor re-runs\n",
			 dead, dead == 1 ? "y" : "ies");
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

/*
 * ★fix#105 (2026-08-12): initialize a data VoQ's PUC token-bucket memory.
 *
 * Stock's dal_rtl9607f_ponmac_queue_add() runs aal_puc_pir_voq_tbc_mem_set +
 * aal_puc_voq_tbc_mem_set PER VoQ, BEFORE aal_puc_qm_voq_report_enable_set.  This driver
 * skipped both, so the CIR/PIR bucket memory for the data VoQs stayed uninitialized; enabling
 * DBA report (REPORT_ENABLE0) then made the PUC read that garbage -> async SError 0xbe000011
 * (RC19).  The values are CAPTURED from live-serving stock on 2026-08-12 (session_2026-08-12/
 * harnesses/stock_puc_voq.py): the data T-CONT (alloc-id 351, gem 225, unshaped CIR=PIR=0) sets
 * all eight data VoQs 8..15 identically.  Replicated verbatim - measured, not guessed.
 */
static void cg_puc_voq_tbc_init(struct cortina_gpon *cg, u32 voq)
{
	/* CIR bucket (PUC_VOQ_TBC_MEM): live-stock VoQ 8..15 */
	writel(0x200a0080, cg->pon + CG_PUC_VOQ_TBC_DATA1);
	writel(0x14000000, cg->pon + CG_PUC_VOQ_TBC_DATA0);
	cg_puc_ind_write(cg, CG_PUC_VOQ_TBC_ACCESS, voq);
	/* PIR bucket (PUC_RL_VOQ_TBC_MEM): live-stock VoQ 8..15 (same as VoQ 7 / wide open) */
	writel(0x3ffe00bf, cg->pon + CG_PUC_RL_VOQ_TBC_DATA1);
	writel(0xffffffff, cg->pon + CG_PUC_RL_VOQ_TBC_DATA0);
	cg_puc_ind_write(cg, CG_PUC_RL_VOQ_TBC_ACCESS, voq);
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
	unsigned int dead = 0;		/* upstream 2bb7081: per-T-CONT entries this pass missed */

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
	/* ★ upstream 2bb7081: do NOT abandon on one pvtbl failure - that skipped the
	 * HDR-A replacement, BTC config and shapers, leaving upstream admission half
	 * configured with nothing to finish it.  Count it, carry on; the O5 supervisor
	 * re-runs while puc_ready stays false. */
	for (tcont = 0; tcont < CG_PUC_TCONT_NUM; tcont++)
		if (cg_puc_pvtbl_program(cg, tcont, tcont == 0))
			dead++;
	/* the CPU high-priority OMCI inject rides the 9th queue (VoQ 127) */
	cg_puc_voq_valid(cg, CG_PUC_9TH_QUEUE_VOQ, true);

	/*
	 * US OMCI header-A replacement: for an OMCI control frame (matched by
	 * the GLOBAL_LNK_TYPE 0xfff1, HW reset default) the PUC stamps the OMCC
	 * GEM index + CoS onto the upstream frame.  Normal: enable_replacement,
	 * gemid=6, cos=6.  High-priority (the 9th-queue inject): gemid=7, cos=7.
	 * us_ext_omci_en + us_hdr_min_size=30 accepts extended (>=14B) OMCI.
	 */
	/*
	 * ★★★★★ fix#70 (2026-08-10) — THE HIGH-PRIORITY REPLACEMENT WAS STAMPING THE
	 * PLOAM CODE POINT.
	 *
	 * Since fix#69 the US OMCI reply reaches the PUC and is classified and enqueued
	 * (omci_us tracks omci_tx one-for-one, rx/enq tick, drop=0, len_err=0) - and yet
	 * us_omcc stays 0 and the OLT never hears it.  The frame is injected at
	 * HEADER_A ldpid 15 = "PON port 7 + 9th-queue inject", i.e. the HIGH-PRIORITY
	 * queue, so the PUC replaces its header from US_OMCI_HP_HDR_A, not from
	 * US_OMCI_HDR_A.  We were stamping gemid 7 / cos 7 there - and per
	 * PUC_GLOBAL_PLOAM_CFG (live 0x8e1ef760: us_omci_hdr_a_field=6,
	 * us_ploam_hdr_a_field=7) cos 7 IS THE PLOAM CODE POINT.  The reply is therefore
	 * emitted, if at all, as a PLOAM rather than onto the OMCC GEM - which is exactly
	 * "classified, enqueued, never counted as US OMCI".
	 *
	 * The HP replacement now defaults to the same {gemid 6, cos 6, tcont 0} as the
	 * normal one.  Both are module params so the space is sweepable without a rebuild.
	 */
	writel(cg_us_omci_hdr_a, pon + CG_PUC_US_OMCI_HDR_A);
	writel(cg_us_omci_hp_hdr_a, pon + CG_PUC_US_OMCI_HP_HDR_A);
	dev_info(cg->dev,
		 "fix#70: PUC US_OMCI_HDR_A=%08x HP_HDR_A=%08x (readback %08x / %08x)\n",
		 cg_us_omci_hdr_a, cg_us_omci_hp_hdr_a,
		 readl(pon + CG_PUC_US_OMCI_HDR_A),
		 readl(pon + CG_PUC_US_OMCI_HP_HDR_A));
	v = readl(pon + CG_PUC_GLOBAL_PLOAM_CFG);
	v = (v & ~GENMASK(21, 16)) | (30u << 16) | BIT(31);
	writel(v, pon + CG_PUC_GLOBAL_PLOAM_CFG);

	/*
	 * That same link type is what makes the control-packet counter an
	 * OMCI-specific one (see cg_puc_ctrl_sample).  It is a hardware reset
	 * default, so check it rather than write it: were it ever something
	 * else, /proc's us_omci would quietly become a counter of nothing, and a
	 * witness that reads 0 for a reason nobody can see is worse than none.
	 */
	v = readl(pon + CG_PUC_GLOBAL_LNK_TYPE) >> 16;
	if (v != CG_PUC_LNK_TYPE_OMCI)
		dev_warn(cg->dev,
			 "PUC control-packet link type is 0x%04x, expected 0x%04x: the upstream OMCI frame count will not match\n",
			 v, CG_PUC_LNK_TYPE_OMCI);

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

	/* aggregate shaper + PIR (rate limiter off) */
	v = readl(pon + CG_PUC_CTRL);
	v = (v | BIT(30)) & ~BIT(26);	/* shp_en=1, rl_en=0 */
	writel(v, pon + CG_PUC_CTRL);
	writel(20u | (20u << 5) | (20u << 10), pon + CG_PUC_CTRL1);
	v = readl(pon + CG_PUC_CTRL2);
	v = (v & ~GENMASK(4, 0)) | 20u | BIT(26);	/* pirovhd=20, pir_en=1 */
	writel(v, pon + CG_PUC_CTRL2);

	/*
	 * ★★★★★ fix#109 (2026-08-12) — THE MISSING GLOBAL REPORT-ENGINE INIT.
	 *
	 * Disassembled aal_puc_qm_enable_set@0xb7bb0 in ca-ne.ko (the exact fn the
	 * stock DAL calls once at dal_rtl9607f_ponmac_init before any queue_add):
	 *   ldr w0,[iobase+0x8274]; orr w0,w0,#0x8; str w0,[iobase+0x8274]
	 * i.e. RMW-sets bit 3 (cfg_qmplmem_en) of PLEN_MEM_CTL 0x8274.  This enables
	 * the PUC QM packet-length memory that the DBA report engine reads to build
	 * status reports.  The port never did this, which is why (a) reading
	 * REPORT_ENABLE0 (0x82d0) sync-aborted [fix#103 misdiagnosed as write-only]
	 * and (b) any REPORT_ENABLE0 write async-SErrored [RC19..RC23].  This is the
	 * global prerequisite RC23 pointed at (per-VoQ ruled out).  0x8274 is the QM
	 * control gate and is readable regardless of QM state (stock's enable_set
	 * RMW-reads it pre-enable), so match stock exactly with an RMW.
	 */
	if (qm_puc_qm_enable) {
		v = readl(pon + CG_PUC_QM_PLEN_MEM_CTL);
		v |= BIT(3);	/* cfg_qmplmem_en = 1 */
		writel(v, pon + CG_PUC_QM_PLEN_MEM_CTL);
		dev_info(cg->dev,
			 "fix#109: PUC QM enable (0x8274 |= BIT(3)); readback %08x\n",
			 readl(pon + CG_PUC_QM_PLEN_MEM_CTL));
	}

	/*
	 * ★★★★★ fix#72 (2026-08-10) — MAKE THE PUC BYTE-IDENTICAL TO LIVE STOCK.
	 *
	 * Captured from THIS board running STOCK while it was actively answering the OLT
	 * (moscli: rxPackets 511 / txResponses 511 / txErrors 0), with the same 113-offset
	 * window the driver prints: `golden_2026-08-10_STOCK_PONWIN_121.txt`, diffed against
	 * our own capture (`golden_2026-08-10_TRACKB_PONWIN.txt`) - 62 identical, 51 differ.
	 * Stock is the ONLY source on this die that emits an upstream OMCI reply; Track A
	 * reaches the PUC exactly as we now do and is equally stuck, so it is exhausted as a
	 * reference.  These are the PUC *config* rows of that diff (counters and clear-on-read
	 * latches excluded).
	 *
	 * ★ Of these, VALID_VOQ is the one with a mechanism attached: stock validates
	 * VoQ 0..15 and leaves the whole "9th queue" (96..127) INVALID - 81b0/81b4/81b8 all
	 * read 0 - while we validate only 0..7 PLUS VoQ 127, which is the queue our ldpid-15
	 * inject targets.  VoQ 127 belongs to T-CONT 15, which has no alloc-id bound, so a
	 * frame parked there can never be granted and can never be emitted.  That is exactly
	 * "classified, enqueued, drop=0, and us_omcc stays 0".
	 */
	if (cg_puc_stock) {
		static const struct { u16 off; u32 val; } stockcfg[] = {
			{ 0x000, 0x0000003f },	/* PUC control: stock clears bit30, sets one more low bit */
			{ 0x00c, 0x00000000 },
			{ 0x010, 0x00000000 },
			{ 0x014, 0x00000000 },
			{ 0x090, 0xaaa9aaa9 },	/* per-queue 2-bit map */
			{ 0x0d0, 0x00000c00 },
			{ 0x0d4, 0x00000700 },
			{ 0x0e8, 0x400000ff },	/* 8 enables, not 6 */
			{ 0x0ec, 0x000000ff },	/* 8 enables, not 3 */
			{ 0x1bc, 0x0000ffff },	/* VALID_VOQ 0..15, no 9th queue */
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(stockcfg); i++)
			writel(stockcfg[i].val,
			       pon + CG_PUC_BASE + stockcfg[i].off);
		dev_info(cg->dev,
			 "fix#72: PUC forced to live-stock config (%u regs); voq0=%08x e8=%08x ec=%08x ctrl=%08x\n",
			 (unsigned int)ARRAY_SIZE(stockcfg),
			 readl(pon + CG_PUC_BASE + 0x1bc),
			 readl(pon + CG_PUC_BASE + 0x0e8),
			 readl(pon + CG_PUC_BASE + 0x0ec),
			 readl(pon + CG_PUC_BASE + 0x000));
	}

	cg->puc_ready = (dead == 0);	/* upstream 2bb7081: ready = EVERY entry landed */
	if (dead)
		dev_warn(cg->dev, "PUC: %u T-CONT pvtbl entr%s failed - NOT ready, O5 supervisor re-runs\n",
			 dead, dead == 1 ? "y" : "ies");
	dev_info(cg->dev,
		 "PUC: OMCC T-CONT0 VoQs + 9th-queue enabled, puccfg=0x%08x lnk_type=0x%04x\n",
		 readl(pon + CG_PUC_PUCCFG),
		 readl(pon + CG_PUC_GLOBAL_LNK_TYPE) >> 16);
}

/*
 * Fold one read of the PUC control-packet counters into the cumulative totals.
 *
 * ★ These counters are CLEAR-ON-READ (PUCCFG.inccfg=2, which is what stock sets
 * too) and the block also drops them after a short idle window, so a snapshot is
 * only ever "did a control frame arrive in the last instant" — which on a
 * working but idle ONU is always no.  Turning that into a usable witness needs
 * two things:
 *
 *   1. clear-on-read makes every read a DELTA since the previous read, so the
 *      sum of all reads is the exact total, with no double counting.  The mode
 *      that looks like a bug is what makes the accumulation exact;
 *   2. this must be the ONLY reader.  It is: nothing else in the driver touches
 *      +0x174/+0x178/+0x188, and /proc/gpon calls THIS rather than reading them.
 *      So a concurrent, unrelated /proc/gpon poller — a monitor sampling the
 *      node every few seconds, say — CONTRIBUTES a delta instead of destroying
 *      one.  With a plain readl in the show function it would instead silently
 *      steal every count it happened to land on, which is precisely how a
 *      snapshot of the neighbouring us_rx came to be structurally guaranteed to
 *      read 0 on a healthy board.
 *
 * The window counters BMC_RX_PKT/_ENQ/FORCE_DROP are deliberately NOT sampled
 * here: they are read raw by the show function as a burst-delta instrument, and
 * a second reader would be exactly the theft described above.
 *
 * Why accumulate in software rather than ask the hardware to stop clearing:
 * PUCCFG.inccfg is block-global (it governs every PUC counter, not just these
 * three), stock writes 2 into it unconditionally before any PON-mode branch, and
 * no source establishes an encoding that means "accumulate" — the reset default
 * is a third value whose meaning is undocumented.  Diverging from stock inside
 * the upstream admission block, on a guess, to save a few lines of adding is a
 * bad trade.  The 16-bit fields cannot overflow between samples either: a sample
 * follows every OMCI transmit within milliseconds, and 65535 control frames do
 * not fit in one window.
 */
static void cg_puc_ctrl_sample(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;

	if (!cg->puc_ready)
		return;
	spin_lock(&cg->puc_cnt_lock);
	cg->puc_omci_us += readl(pon + CG_PUC_BMC_CTRL_PKT_LNK) &
			   CG_PUC_BMC_CNTR_MASK;
	cg->puc_ctrl_mac += readl(pon + CG_PUC_BMC_CTRL_PKT_MAC) &
			    CG_PUC_BMC_CNTR_MASK;
	cg->puc_len_err += readl(pon + CG_PUC_BMC_LENGTH_ERROR) &
			   CG_PUC_BMC_CNTR_MASK;
	cg->puc_cnt_samples++;
	spin_unlock(&cg->puc_cnt_lock);
}

/*
 * Sample shortly after an upstream OMCI frame was handed to the NI: the PDU
 * reaches the PUC by DMA microseconds later, well inside the counter's window,
 * and this is the one moment at which the OMCI counter is expected to move.  A
 * burst of replies coalesces into one sample (the counter accumulates in
 * hardware meanwhile, so nothing is lost) — that is why a re-arm while already
 * queued is a no-op rather than a reschedule.
 */
#define CG_PUC_CNT_TX_DELAY_MS	20
/* Backoff for a failed VEIP oper-up AVC TX.  Bounded in RATE, not in
 * attempts: the OLT never re-solicits this AVC, so a count cap would end
 * the session's only path back to Match State normal. */
#define CG_VEIP_AVC_RETRY_MIN_MS	500
#define CG_VEIP_AVC_RETRY_MAX_MS	30000

static void cg_puc_cnt_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, puc_cnt_work);

	cg_puc_ctrl_sample(cg);
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
	/* The PON identity, from cg->sn (the board's serial number -- see the
	 * cg_sn_* block).  Both halves come from the SAME 8 bytes the OMCI
	 * responder is armed with, so the PLOAM and OMCI identities cannot drift. */
	writel(cg_sn_word(cg->sn), mac + CG_REG_VENDOR);	/* 4 ASCII vendor-id chars */
	writel(cg_sn_word(cg->sn + 4), mac + CG_REG_VENDOR_SPEC);	/* 4 VSSN bytes */
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

	/*
	 * ★★★★★ fix#73 (2026-08-10) — the GPON-MAC half of the live-stock diff.
	 *
	 * Same capture as fix#72 (stock answering the OLT, 511 responses, 0 errors):
	 * `golden_2026-08-10_STOCK_PONWIN_121.txt` vs `golden_2026-08-10_TRACKB_PONWIN.txt`.
	 * These are the GPON-MAC (0x6xxx) CONFIG rows where we differ.  Deliberately NOT
	 * included, and why:
	 *   6114/6118 onu_ctl/onu_cfg  - live ranging state; writing them mid-FSM disturbs O5
	 *   601c/6020/6024             - the registration ID ("ZYXE123456"); not needed to range
	 *   612c/6130/6134/6138        - indirect ACCESS/DATA latches = residue of stock's last
	 *                                access, not configuration
	 *   60c0, 6810, 6814           - free-running counters/timers
	 *   9020/9024                  - PDC; downstream OMCI receive already works, do not touch
	 * Bitmask so the set can be bisected in one boot each without a rebuild:
	 * cortina_gpon.mac_stock=0 disables, 0xfff = all.
	 */
	if (cg_mac_stock) {
		static const struct { u16 off; u32 val; } stockmac[] = {
			{ 0x00c, 0x00000040 },
			{ 0x078, 0x00000060 },
			{ 0x084, 0x00000000 },
			{ 0x08c, 0x00000000 },
			{ 0x09c, 0x00000103 },
			{ 0x0fc, 0x12409c00 },
			{ 0x170, 0x0000000f },
			{ 0x174, 0x000000e1 },
			{ 0x1c4, 0x1f400000 },
			{ 0xe14, 0x00000002 },
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(stockmac); i++)
			if (cg_mac_stock & BIT(i))
				writel(stockmac[i].val, mac + stockmac[i].off);
		dev_info(cg->dev,
			 "fix#73: GPON MAC forced to live-stock config (mask=0x%x); 170=%08x 174=%08x 09c=%08x 0fc=%08x\n",
			 cg_mac_stock,
			 readl(mac + 0x170), readl(mac + 0x174),
			 readl(mac + 0x09c), readl(mac + 0x0fc));
	}
	/* ★ 2026-08-08 AOT5221ZY: registration password (AES keys still deferred).
	 * Written here, with the serial number and before onu_ctl.en, because the
	 * ONU must already hold it when the OLT's Password_Request arrives during
	 * ranging.  Skipped entirely when unset, preserving the previous behaviour. */
	if (cg_pw0 || cg_pw1 || cg_pw2) {
		writel(cg_pw0, mac + CG_REG_PFRAG0);
		writel(cg_pw1, mac + CG_REG_PFRAG1);
		writel(cg_pw2, mac + CG_REG_PFRAG2);
		dev_info(cg->dev, "registration password programmed (%08x %08x %08x)\n",
			 cg_pw0, cg_pw1, cg_pw2);
	}

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
 * Program the identity + start ranging, and verify the identity actually landed.
 * Caller holds sn_lock and has put a valid serial number in cg->sn.
 *
 * The vendor-id readback doubles as the PON-window sanity check the old
 * compiled-in strcmp(vendor, "XPON") used to provide -- but against what we just
 * wrote rather than a literal, so it catches a wrong window base OR a write that
 * did not stick, on any board.
 */
static void cg_activate_start(struct cortina_gpon *cg)
{
	char sn_str[13];
	u32 vid;

	cg_sn_format(cg->sn, sn_str);
	dev_info(cg->dev, "activating with serial number %s (source: %s)\n",
		 sn_str, cg_sn_src_name[cg->sn_src]);

	cg_mac_activate(cg);
	cg->activated = true;

	vid = readl(cg->mac + CG_REG_VENDOR);
	if (vid != cg_sn_word(cg->sn))
		dev_warn(cg->dev,
			 "vendor-id readback 0x%08x != programmed 0x%08x - PON window base wrong, or the MAC is still gated\n",
			 vid, cg_sn_word(cg->sn));

	/* Post-activation snapshot, on EVERY activation path (the probe's 30-line
	 * ranging poll below only runs when the identity was known at probe).
	 * /proc/gpon carries the full picture on demand. */
	dev_info(cg->dev,
		 "activate: vendor-id=0x%08x vendor-spec=0x%08x onu_cfg=0x%08x onu_ctl=0x%08x gpon_ds=0x%08x onu=0x%08x rgb8=0x%08x\n",
		 vid, readl(cg->mac + CG_REG_VENDOR_SPEC),
		 readl(cg->mac + CG_REG_ONU_CFG_REAL),
		 readl(cg->mac + CG_REG_ONU_CTL),
		 readl(cg->mac + CG_REG_GPON_DS),
		 readl(cg->mac + CG_REG_GPON_ONU),
		 readl(cg->pon + CG_PSDS_RGB8));

	/* Arm the cold-start US-lock recovery watchdog: if the HW ranging FSM is
	 * still stuck at O1 after a grace period (the cold TX-PLL metastability),
	 * re-lock the SerDes CMU and re-arm ranging until it advances -- so every
	 * cold boot reaches O5 (stock does, 100%).  mod_ and not schedule_ for
	 * the same reason as in cg_datapath_reset: a re-activation on a LIVE link
	 * (a serial-number change through /proc) must get the full 15 s grace, not
	 * whatever is left of the pending post-O5 supervisor's deadline. */
	mod_delayed_work(system_wq, &cg->coldstart_work, 15 * HZ);
}

/*
 * Latch a serial number and (re)start ranging with it.  The only entry point for
 * a provisioned identity: the /proc write, the module-param path and the
 * unprovisioned-timeout path all go through here, so the MAC registers and the
 * OMCI responder are always armed from the same 8 bytes.
 *
 * Re-activating an already-ranging MAC is the proven cold-start recovery
 * sequence (cg_coldstart_work does exactly this), so an identity that arrives
 * late is applied by re-running it -- but only when it actually DIFFERS, so a
 * duplicate provisioning write never disturbs a healthy link.
 */
static int cg_sn_set(struct cortina_gpon *cg, const char *s, enum cg_sn_src src)
{
	u8 sn[8];
	char sn_str[13];
	bool changed;
	int ret;

	ret = cg_sn_parse(s, sn);
	if (ret) {
		dev_err(cg->dev, "rejected GPON serial number \"%s\": expected 4 vendor-id characters + 8 hex digits\n",
			s ? s : "");
		return ret;
	}

	mutex_lock(&cg->sn_lock);
	changed = cg->sn_src == CG_SN_NONE || memcmp(cg->sn, sn, sizeof(sn));
	memcpy(cg->sn, sn, sizeof(sn));
	cg->sn_src = src;
	cg_sn_format(cg->sn, sn_str);

	if (!cg_activate)
		dev_info(cg->dev, "serial number %s latched (source: %s); activate=0, not ranging\n",
			 sn_str, cg_sn_src_name[src]);
	else if (cg->activated && !changed)
		dev_info(cg->dev, "serial number %s re-confirmed (source: %s) - link untouched\n",
			 sn_str, cg_sn_src_name[src]);
	else {
		if (cg->activated)
			dev_warn(cg->dev, "serial number CHANGED to %s (source: %s) - re-ranging\n",
				 sn_str, cg_sn_src_name[src]);
		cancel_delayed_work(&cg->sn_wait_work);
		cg_activate_start(cg);
	}
	mutex_unlock(&cg->sn_lock);
	return 0;
}

/*
 * Nothing provisioned a serial number in time.  Never leave the PON side dark
 * and never guess this board's identity: shout, range with the non-identity
 * placeholder so the failure is visible at the OLT too, and stay ready for the
 * real serial number (a later /proc write re-ranges with it).
 */
static void cg_sn_wait_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, sn_wait_work);
	char sn_str[13];

	mutex_lock(&cg->sn_lock);
	if (cg->sn_src != CG_SN_NONE) {		/* raced with a provisioning write */
		mutex_unlock(&cg->sn_lock);
		return;
	}
	memcpy(cg->sn, cg_sn_unprovisioned, sizeof(cg->sn));
	cg->sn_src = CG_SN_FALLBACK;
	cg_sn_format(cg->sn, sn_str);
	dev_err(cg->dev,
		"NO per-board GPON serial number after %ds: is /etc/init.d/gpon-identity running, and is ubi0:ubi_Config mountable? Ranging with the placeholder %s - this is NOT this board's identity, the OLT will not admit it. Push the real one:  echo \"sn <VVVVHHHHHHHH>\" > /proc/gpon\n",
		CG_SN_WAIT_SECS, sn_str);
	cg_activate_start(cg);
	mutex_unlock(&cg->sn_lock);
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

	/* ★★ upstream 2bb7081: RECOMPUTE UNCONDITIONALLY, exactly as stock does on
	 * EVERY downstream PLOAM.  This used to bail out when the Extended_Burst_Length
	 * latch read empty (!pre || !ranged), which leaves the PREVIOUS OLT's
	 * compensation latched: swap the fibre to an OLT that sends no
	 * Extended_Burst_Length (or let the GTC re-roll clear the latch) and
	 * us.frame_var stays at the old value while the new OLT expects the default,
	 * misaligning the serial number in every ranging window (stuck O1, reboot to
	 * recover).  No special case is needed for the empty latch: the formula
	 * already yields the standard-burst default there
	 * (pre=ranged=0 -> (0x200 - 0x20) & 0x1ff = 0x1E0). */
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
 * Re-lock the PON-SerDes CMU/PLL (vendor aal_psds_reset).  At cold power-on the
 * CMU can latch a metastable phase off the reference clock, so the upstream
 * burst never frames: the ONU sits at O1 with the downstream locked and the OLT
 * reports "Laser out".  Strobe the analog CMU field (PSDS internal register
 * CG_PSDS_CMU_IDX bits[7:4]) through the vendor value sequence 0x8 -> 0xd ->
 * 0x7 -> 0x0 with ~1 ms settles, then re-wait the RX/TX clock-ready (a05c bits
 * 15/11 CKRDY_TX/10 CKRDY_RX set, bit0 RX_LOS clear; bit12 frame-lock is not
 * required for the analog re-lock).  This is the dedicated re-lock the vendor
 * runs on every GPON re-range/reconfigure, and the Cortina analog of the 9602C
 * O3-entry TX-PLL relock.  It does NOT power-cycle the SerDes (POW_PCIX
 * untouched), so the PON APB clock and the GPON MAC config are undisturbed.
 */
static void cg_psds_relock(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	static const u8 seq[] = { 0x8, 0xd, 0x7, 0x0 };
	u32 base;
	int i, k;

	/* read the current CMU reg (a088 read strobe -> a090), clear field [7:4] */
	writel(CG_PSDS_IND_READ | CG_PSDS_CMU_IDX, pon + CG_PSDS_IND_CMD);
	udelay(10);
	base = readl(pon + CG_PSDS_IND_RDATA) & ~0xf0u;

	/* strobe [7:4] = 8 -> d -> 7 -> 0, ~1 ms apart (aal_psds_reset) */
	for (k = 0; k < ARRAY_SIZE(seq); k++) {
		writel(base | ((u32)seq[k] << 4), pon + CG_PSDS_IND_WDATA);
		writel(CG_PSDS_IND_WRITE | CG_PSDS_CMU_IDX, pon + CG_PSDS_IND_CMD);
		mdelay(1);
	}

	/* re-wait the CMU/PLL lock (bounded ~1000 ms, as the vendor does) */
	for (i = 0; i < 1001; i++) {
		if ((readl(pon + CG_PSDS_RGB8) & 0x8c01) == 0x8c00)
			break;
		mdelay(1);
	}
	dev_info(cg->dev, "psds re-lock (8/d/7/0): base=0x%08x lock at %dms rgb8=0x%08x\n",
		 base, i, readl(pon + CG_PSDS_RGB8));
}

/* Re-arm the GPON MAC's interrupt enables (the four W1C groups + int_top).  The
 * GLB-level aggregation gates (PON_INTEN0 / NE_ICTL_EN) and the requested IRQ
 * live outside the GTC block and survive a GTC reset, so a cold-start re-roll
 * only needs to restore THIS.  Shared by cg_intr_setup() and the watchdog. */
static void cg_mac_intr_arm(struct cortina_gpon *cg)
{
	static const struct { u32 sts, en, mask; } grp[4] = {
		{ CG_REG_INT,  CG_REG_INT_EN,  CG_INT_EN_DEFAULT },
		{ CG_REG_INT2, CG_REG_INT2_EN, 0 },
		{ CG_REG_INT3, CG_REG_INT3_EN, 0 },
		{ CG_REG_INT4, CG_REG_INT4_EN, 0 },
	};
	int i;

	writel(0, cg->mac + CG_REG_INT_TOP_EN);
	(void)readl(cg->mac + CG_REG_INT_TOP);		/* read-clear stale */
	for (i = 0; i < 4; i++) {
		writel(0, cg->mac + grp[i].en);
		writel(grp[i].mask, cg->mac + grp[i].sts);	/* W1C stale */
		writel(grp[i].mask, cg->mac + grp[i].en);
	}
	writel(CG_INT_TOP_EN_ALL, cg->mac + CG_REG_INT_TOP_EN);
}

/*
 * Cold-start US-lock recovery watchdog.  G.984.3 O1->O5 ranging runs in
 * hardware and post-probe servicing is purely interrupt-driven, so a boot that
 * loses the cold-start analog lottery sits at O1 forever -- the upstream never
 * bursts, the OLT reports "Laser out", and NO state-change event fires, so
 * nothing re-runs.  Stock reaches O5 on 100% of cold boots, so this is ours.
 *
 * The stuck signature (captured live 2026-07-17): state O1, onu-id 0xff,
 * us/t3_preamble = 0, ZERO MAC interrupts -- yet the downstream is LOCKED
 * (rgb8 = 0x19c00, CKRDY_TX set, superframe advancing).  So the RX clock/PLL is
 * fine; the metastable element is deeper (the gearbox/framer comes up such that
 * frames sync but no DS PLOAM decodes, so the ONU never sees the ranging
 * request and never bursts its serial number).  A CMU-only re-strobe does NOT
 * clear it.  The reliable recovery is to re-run the WHOLE proven bring-up --
 * cg_glb_reset() power-cycles the SerDes (POW_PCIX off->on) + re-asserts the
 * GTC/gearbox resets, cg_psds_init() re-rolls the CMU/gearbox/framer from that
 * clean state and re-fires the GTC sync edge -- then re-arm the MAC interrupts
 * and re-assert the SN/ranging.  That is exactly the sequence that reaches O5 on
 * ~71% of cold rolls, so each re-roll is a fresh independent attempt and a few
 * converge to 100%.  A boot that has already left O1 (ranging is progressing) is
 * never disturbed: the watchdog only re-rolls on the exact stuck signature
 * (state O1 AND DS locked) and merely observes otherwise.  It does NOT stop at
 * the first O5 -- cg_datapath_reset() re-arms it on every O5 exit so a LATER
 * relapse into the stuck-O1 class is recovered too, and it NEVER gives up
 * (past a fast-retry budget it just backs off; see below).
 */
/* Fast-retry budget per stuck-O1 EPISODE (resets whenever the FSM leaves O1):
 * past it the watchdog WARNs once and backs off to a 60 s cadence — the RATE is
 * bounded, the COUNT never is.  A re-roll at O1 is OLT-invisible (us=0, no
 * laser before a grant), so retrying forever cannot hang or churn the PON, and
 * the production bar (ship unattended for months, self-recover from ANY event
 * with the OLT untouched) forbids a permanent stop: a transient that outlives
 * any fixed cap — the OLT still in its ~150 s post-power-cycle settle, an
 * admin-Inactive ONT, a churn-locked OLT opening no ranging window — must
 * still recover the moment it clears (relock_rearm_test case [a]). */
#define CG_COLD_FAST_TRIES	12
/*
 * Post-O5 SUPERVISOR cadence.  Once the FSM reaches Operation this same delayed
 * work keeps running at this slow rate and does exactly one thing: re-kick
 * cg_isr_work, whose tail reconciles the soft state against the LIVE FSM
 * register (see the reconcile block there).  30 s is slow enough to be free
 * next to a 1 Gbps datapath and fast enough that a lost edge costs at most one
 * tick of OMCI silence - far inside the OLT's patience before it deactivates
 * the ONU.  The retry RATE is bounded by this value; the COUNT never is.
 */
#define CG_O5_SUPERVISOR_SECS	30
static void cg_coldstart_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, coldstart_work);
	u32 onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	u32 rgb8 = readl(cg->pon + CG_PSDS_RGB8);
	u8 state = CG_ONU_STATE(onu);
	bool ds_locked = (rgb8 & 0x9c01) == 0x9c00;

	if (state != 0) {			/* left O1: ranging is progressing */
		cg->coldstart_tries = 0;	/* fresh episode = fresh fast budget */
		if (state != CG_STATE_OPERATION) {
			schedule_delayed_work(&cg->coldstart_work, 5 * HZ);
			return;
		}
		/*
		 * O5 reached: this work does NOT stop, it becomes the slow
		 * post-O5 SUPERVISOR.  Stopping here left a live link with no
		 * periodic servicing at all, so ONE lost edge - an event
		 * discarded by a full event ring (cg_isr: evt_drop++, counted
		 * and forgotten), or a cg_tbl_op that timed out and bare-
		 * returned out of cg_data_try_install - wedged the soft state
		 * against healthy-O5 hardware with no in-boot recovery, until
		 * the OLT gave up and deactivated the ONU (PON-wide churn, which
		 * the production bar forbids).
		 *
		 * The tick does NOTHING itself: it only re-kicks isr_work, the
		 * single-threaded bottom half that is the ONLY context allowed
		 * to run cg_tbl_op, so that invariant is untouched (re-queueing
		 * a work_struct that is already queued or running is a no-op,
		 * and a work_struct never runs concurrently with itself).  The
		 * stuck-O1 SerDes re-roll is NOT reachable from here - that path
		 * is below, gated on state == O1 - so the supervisor can never
		 * re-roll the analog on a live link.
		 *
		 * A converged link runs a pure soft-state compare: the reconcile
		 * writes nothing once the tracker, the OMCC bind and frame_var
		 * already agree with the live register, and cg_data_try_install
		 * early-outs on data_installed before any HW access - i.e. ZERO
		 * register writes, byte-for-byte the proven no-churn
		 * LOS/fiber-pull keep-path (o5_reconcile_tick_test case [e]).
		 * RATE-bounded at CG_O5_SUPERVISOR_SECS, COUNT never bounded:
		 * the OLT cannot re-solicit, so giving up after N attempts would
		 * strand the session until a power cycle.
		 */
		/* ★★ upstream 2bb7081: THE SUPERVISOR CONSULTS pdc_ready/puc_ready.
		 * cg_pdc_init/cg_puc_init count-and-continue on an indirect-access
		 * timeout and set ready=(dead==0); before this heal, a half-written
		 * PDC map or PUC pvtbl (HDR-A/BTC/shapers/per-VoQ-valid skipped) was
		 * merely printed and then carried for the whole uptime with nothing
		 * to finish it.  Re-run is safe: both inits are idempotent table
		 * writes, this runs at the slow post-O5 rate, and it is skipped once
		 * both are ready (a healthy link does zero extra writes).
		 * ★ NEVER over an ARMED data path: with the data GEM installed,
		 * re-walking the PUC pvtbl would momentarily re-write the very VoQ
		 * admission the live upstream is using. */
		if ((!cg->pdc_ready || !cg->puc_ready) && !cg->data_installed) {
			dev_warn(cg->dev,
				 "O5 supervisor: re-running %s%sinit (idempotent; datapath not yet armed)\n",
				 cg->pdc_ready ? "" : "PDC ",
				 cg->puc_ready ? "" : "PUC ");
			if (!cg->pdc_ready)
				cg_pdc_init(cg);
			if (!cg->puc_ready)
				cg_puc_init(cg);
		}
		schedule_work(&cg->isr_work);
		schedule_delayed_work(&cg->coldstart_work,
				      CG_O5_SUPERVISOR_SECS * HZ);
		return;
	}
	if (!ds_locked) {
		/* No DS frame lock: the RX is still settling (cold boot) OR the
		 * fiber is pulled / dark (LOS).  A re-roll helps NEITHER — there
		 * is no downstream to frame — and would re-init the SerDes +
		 * pulse the laser into a dark fiber and could disturb the proven
		 * LOS/fiber-pull re-range.  The observed cold-start wedge ALWAYS
		 * has DS LOCKED (rgb8=0x19c00), so only wait here: bounded rate,
		 * never give up, until either light returns or DS locks. */
		schedule_delayed_work(&cg->coldstart_work, 3 * HZ);
		return;
	}
	/* state O1 with DS LOCKED = the stuck-O1 signature (no US PLOAM/burst). */
	if (!cg_coldstart_wd) {
		/* A/B baseline (coldstart_wd=0): observe the stuck-O1, never
		 * re-roll — the pre-watchdog wedge.  Flipping the param live
		 * (/sys/module/.../coldstart_wd) lets the SAME wedged boot then
		 * recover, isolating the re-roll as the fix. */
		schedule_delayed_work(&cg->coldstart_work, 16 * HZ);
		return;
	}
	cg->coldstart_tries++;
	cg->coldstart_rolls++;
	if (cg->coldstart_tries == CG_COLD_FAST_TRIES)
		dev_warn(cg->dev,
			 "cold-start recovery: %d fast re-rolls, still O1 - backing off to 60s cadence, never stopping\n",
			 cg->coldstart_tries);
	dev_info(cg->dev,
		 "cold-start stuck O1, DS locked but no PLOAM (onu=0x%08x rgb8=0x%08x us=0x%08x t3=0x%08x) - full SerDes re-roll #%u\n",
		 onu, rgb8, cg_mac_rd(cg, CG_REG_US), cg_mac_rd(cg, CG_REG_T3_PREAMBLE),
		 cg->coldstart_rolls);
	/* re-run the whole proven bring-up = a fresh cold roll of the metastable
	 * gearbox/framer + a clean SN/ranging re-arm.  Each attempt is
	 * internally bounded (SerDes lock poll <=1 s, activate DS-wait <=8 s),
	 * so the cadence below bounds the retry RATE; nothing bounds the count. */
	cg_glb_reset(cg);
	cg_psds_init(cg);
	cg_mac_intr_arm(cg);	/* the GTC reset cleared the MAC int enables */
	/* sn_lock so a serial number arriving from userspace mid-re-roll cannot be
	 * half-applied: cg_mac_activate programs the identity out of cg->sn. */
	mutex_lock(&cg->sn_lock);
	cg_mac_activate(cg);	/* re-config + re-assert onu_ctl.en (SN/ranging) */
	mutex_unlock(&cg->sn_lock);
	schedule_delayed_work(&cg->coldstart_work,
			      cg->coldstart_tries >= CG_COLD_FAST_TRIES ?
			      60 * HZ : 16 * HZ);
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
 * Read one US-GEM MIB slot.  `idx` is the upstream hw-GEM index (the SAME index space
 * as CG_REG_US_PORT_ACCESS, 0..CG_US_PORT_IDX_MAX), not a GEM port-id.
 *
 * Returns 0 and fills the counters, or a negative errno if the indirect access did not
 * complete.  On error the counters are left untouched so a caller printing them cannot
 * silently show a stale latch as if it were a fresh read - the one failure mode that
 * would make this witness lie.
 */
static uint mib_rbw = 1;
module_param(mib_rbw, uint, 0644);
MODULE_PARM_DESC(mib_rbw,
	"US GEM MIB ACCESS rbw bit BIT(30). 1 = the earlier port's measured-working read that indexes per-GEM; 0 = the old form that aliased every index. Default 1.");

static int cg_us_mib_read(struct cortina_gpon *cg, u32 idx, u32 op,
			  u32 *pcnt, u32 *fcnt, u64 *bcnt)
{
	u32 d3, d2, d1, d0;
	int ret;

	if (idx > CG_US_PORT_IDX_MAX)
		return -EINVAL;
	/*
	 * ★fix#97 (2026-08-11) - REVERTS fix#96, which was wrong in three ways.
	 *
	 * Sequence taken from THIS board's own stock blob,
	 * aot_stock/rootfs/lib/modules/5.10.138/ca-ne.ko, aal_gpon_us_gem_port_mib_get
	 * (@0xebd34): ONE fused write go|op_code[29:28]|sel[7:0] -> ACCESS, poll go
	 * self-clear, then read DATA3, DATA2, DATA1, DATA0 in DESCENDING address order.
	 *
	 * WHAT fix#96 GOT WRONG:
	 *  1. It added an op0+op1 "latch".  The vendor issues op0/op1 at ZERO call sites on
	 *     this die.  The latch left DATA holding the residue of operations that do not
	 *     exist, which is what produced the "every sel returns 78" artefact that was then
	 *     read as "the index is ignored".  The index was never ignored.
	 *  2. `idx` is an internal hw-GEM INDEX (0..CG_US_PORT_IDX_MAX), NOT a GEM port-id.
	 *     Probing 92 and 225 (port-ids) addressed unbound slots, and probing 0/8/9 missed
	 *     the only slot that carries traffic: US OMCI rides the index stamped in
	 *     US_OMCI_HP_HDR_A (default 0x707 -> index 7), which was never probed.
	 *     Ground truth on this exact board: US_MIB[sel=7] pkt=49 frag=98 bytes=0x89d
	 *     (= 45.0 B/pkt exactly) with sel 0..6 all zero.
	 *  3. op2 is READ_CLR.  Any double-pump reads the entry it just cleared and reports a
	 *     zero indistinguishable from "nothing was transmitted".
	 *
	 * DATA order matches the vendor deliberately: this register family has a proven
	 * read-side-effect precedent on this silicon (the BWmap table advances its pointer on
	 * the DATA0/DATA1 pair), and the old code read DATA0 FIRST where the vendor reads it
	 * LAST.
	 *
	 * ⛔ Do NOT "restore" op0/op1/op3 - their semantics are unrecoverable and stock never
	 * issues them.
	 */
	/*
	 * ★fix#98: the rbw bit BIT(30) is THE difference, and it was already solved in this
	 * project's own EARLIER AOT port - readable C, not disassembly:
	 *   ~/ak007/.../cortina/gpon/cortina-gpon.c:3838
	 *     writel(BIT(31) | BIT(30) | (k & 0xff), m + 0x150);   go | rbw | sel, op_code 0
	 * with m == cg->mac == pon+0x6000, the identical base pointer this driver uses.
	 *
	 * MEASURED with that exact word on THIS board, three sessions:
	 *   RESUME_2026-08-07d:452   US_MIB[sel=7] pkt=49 frag=98 byteHi=0x89d, all other sel 0
	 *   RESUME_2026-07-30_S3:312 US_MIB[sel=7] pkt=59->72 climbing, sel 0-6 all zero
	 *   RESUME_2026-08-01:111    US_MIB[sel=7] pkt=294->296 frag=588->592, sel 0-6 zero
	 * 2205 bytes / 49 pkt = 45.0 B/pkt exactly, frag = 2 x pkt.  The index has been
	 * honoured since 2026-07-30 - it was THIS driver's access word that was wrong.
	 *
	 * mib_rbw exists so the claim is A/B-able on one boot rather than believed.
	 */
	ret = cg_tbl_op(cg, CG_REG_US_MIB_ACCESS,
			(mib_rbw ? CG_TBL_WR : 0) |
			FIELD_PREP(CG_US_MIB_OP_CODE, op & 3) | (idx & 0xff));
	if (ret)
		return ret;
	d3 = readl(cg->mac + CG_REG_US_MIB_DATA3);
	d2 = readl(cg->mac + CG_REG_US_MIB_DATA2);
	d1 = readl(cg->mac + CG_REG_US_MIB_DATA1);
	d0 = readl(cg->mac + CG_REG_US_MIB_DATA0);
	*pcnt = d0;
	*fcnt = d1;
	*bcnt = ((u64)d3 << 32) | d2;
	return 0;
}

/*
 * Print the upstream transmit witness for the OMCC slot and the data slot side by side.
 *
 * Reading BOTH is the whole point, because it makes the result self-calibrating.  The
 * OMCC column is a positive control: US OMCI is independently PROVEN to reach the OLT
 * (the OLT answers, and fix#78 is exactly the change that made it answer), so if the
 * OMCC counter does not move either, this MIB is not counting on this die and the
 * DATA column means nothing - report that, do not read the data column as a zero.
 * Without that control a dead register file and a dead upstream look identical, which
 * is how `us_omcc(0x1e0)` and `omci_us(0x8188)` got quoted as evidence before §4
 * retracted them for counting nothing.
 */
static void cg_us_mib_show(struct cortina_gpon *cg, struct seq_file *m)
{
	u32 o_idx = gpon_gem_us_index(&cg_us_omcc_slots, 0);
	u32 d_idx = gpon_gem_us_index(&cg_us_data_slots, 0);
	u32 op;

	/* the interface control first - see CG_REG_DS_MIB_ACCESS */
	for (op = 0; op < 4; op++) {
		u32 s_p = 0, s_f = 0;
		u64 s_b = 0;

		if (cg_tbl_op(cg, CG_REG_DS_MIB_ACCESS,
			      FIELD_PREP(CG_US_MIB_OP_CODE, op) | (d_idx & 0xff)))
			continue;
		s_p = readl(cg->mac + CG_REG_DS_MIB_DATA0);
		s_f = readl(cg->mac + CG_REG_DS_MIB_DATA1);
		s_b = ((u64)readl(cg->mac + CG_REG_DS_MIB_DATA3) << 32) |
		      readl(cg->mac + CG_REG_DS_MIB_DATA2);
		if (m)
			seq_printf(m, "ds_mib op%u: data[%u] pkt=%u frm=%u byte=%llu  (CONTROL: DS traffic provably flows, so a correctly-driven read CANNOT be 0)\n",
				   op, d_idx, s_p, s_f, s_b);
		else
			dev_info(cg->dev, "DS-MIB op%u: data[%u] pkt=%u frm=%u byte=%llu (interface control)\n",
				 op, d_idx, s_p, s_f, s_b);
	}

	/*
	 * ★ RB8 RESULT: op2 is the live read (op0/1/3 all read 0; op2 gave pkt=78 frm=156,
	 * and 78 is exactly the US OMCI count at that moment - the positive control worked).
	 * BUT omcc[0] and data[8] returned IDENTICAL values, so sel is NOT selecting a GEM:
	 * this is a global or stuck counter.  That distinction decides what a flat data
	 * column is allowed to mean, so probe the sel space explicitly rather than assume:
	 *   all sel equal        -> global counter; flat while data_enq climbs IS evidence
	 *   some sel differ      -> per-GEM after all, and we had the wrong index
	 * 0/8 are the OMCC and data slot indices, 9 the broadcast slot, 92 and 225 the
	 * OMCC and data GEM PORT-IDs (in case sel is a port-id space, not an index), 200 a
	 * deliberately unused slot as a negative control - if 200 also reads 78, sel is
	 * being ignored outright.
	 */
	if (m) {
		/* ★fix#97: the INDEX space (0..15), not port-ids.  Expect exactly the OMCI
		 * index (US_OMCI_HP_HDR_A & 0xff, default 7) nonzero with fcnt ~ 2*pcnt. */
		static const u16 sel_probe[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
		unsigned int k;

		for (k = 0; k < ARRAY_SIZE(sel_probe); k++) {
			u32 p = 0, f = 0;
			u64 b = 0;

			/* ★fix#99: op_code 0, not 2.  The earlier port's MEASURED-WORKING
			 * word is exactly BIT(31)|BIT(30)|sel (ak007 .../gpon/cortina-gpon.c
			 * :3838) - go|rbw|sel with op_code ZERO.  fix#98 added the rbw bit but
			 * left this probe on op 2, so it sent rbw|op2 and still read zero. */
			if (cg_us_mib_read(cg, sel_probe[k], 0, &p, &f, &b))
				continue;
			seq_printf(m, "us_mib sel-probe op0 sel=%-3u pkt=%u frm=%u byte=%llu\n",
				   sel_probe[k], p, f, b);
		}
	}

	for (op = 0; op < 4; op++) {
		u32 o_p = 0, o_f = 0, d_p = 0, d_f = 0;
		u64 o_b = 0, d_b = 0;
		int o_ret, d_ret;

		o_ret = cg_us_mib_read(cg, o_idx, op, &o_p, &o_f, &o_b);
		d_ret = cg_us_mib_read(cg, d_idx, op, &d_p, &d_f, &d_b);
		if (o_ret || d_ret) {
			if (m)
				seq_printf(m, "us_mib op%u: <access failed omcc=%d data=%d>\n",
					   op, o_ret, d_ret);
			else
				dev_info(cg->dev, "US-MIB op%u: access failed omcc=%d data=%d\n",
					 op, o_ret, d_ret);
			continue;
		}
		if (m)
			seq_printf(m,
				   "us_mib op%u: omcc[%u] pkt=%u frm=%u byte=%llu | data[%u] pkt=%u frm=%u byte=%llu\n",
				   op, o_idx, o_p, o_f, o_b, d_idx, d_p, d_f, d_b);
		else
			dev_info(cg->dev,
				 "US-MIB op%u: omcc[%u] pkt=%u frm=%u byte=%llu | data[%u] pkt=%u frm=%u byte=%llu\n",
				 op, o_idx, o_p, o_f, o_b, d_idx, d_p, d_f, d_b);
	}
}

/*
 * Invalidate a stale T-CONT CAM entry: RMW-clear omci_en + ploam_en and zero
 * the hw-T-CONT index for alloc-id `alloc` (vendor aal_gpon_tcont_set with
 * en=0).  Read-modify-write so unrelated bits are preserved.  Called when a
 * genuinely different alloc/onu-id REPLACES a live binding, so a grant now
 * addressed to a reassigned alloc no longer makes this ONU burst.
 */
static void cg_tcont_unbind(struct cortina_gpon *cg, u32 alloc)
{
	u32 d;

	alloc &= 0xfff;
	if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, alloc))
		return;
	d = readl(cg->mac + CG_REG_TCONT_DATA);
	d &= ~(CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN | CG_TCONT_INDEX_MASK);
	writel(d, cg->mac + CG_REG_TCONT_DATA);
	if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, CG_TBL_WR | alloc))
		return;
	dev_info(cg->dev, "T-CONT CAM[alloc %u] invalidated (stale)\n", alloc);
}

/*
 * Bind the OMCC to the T-CONT table: entry[alloc-id = onu-id] -> hw T-CONT 0
 * with omci_en + ploam_en (the G.984.3 default alloc-id == onu-id carries the
 * OMCC).  Read-modify-write of the indirect entry, exactly the vendor
 * aal_gpon_omcc_tcont_enable -> aal_gpon_tcont_set sequence.
 *
 * Stale-CAM guard: if a genuinely DIFFERENT onu-id/alloc replaces the OMCC
 * binding (an OLT-driven onu-id change), invalidate the stale predecessor
 * FIRST so the old CAM entry can never burst into a reassigned grant slot once
 * we re-enter O5.  This runs during ranging (Assign_ONU-ID is an O4 PLOAM), so
 * the OMCC CAM is corrected BEFORE Operation.  A same-id re-range takes neither
 * branch — the entry already carries this binding, and re-writing the same
 * value is the existing proven idempotent path (no teardown, no churn).
 */
static void cg_omcc_tcont_bind(struct cortina_gpon *cg, u32 alloc_id)
{
	u16 old = cg->omcc_alloc;
	u32 d;

	alloc_id &= 0xfff;
	/* `old != 0` used to stand in for "previously bound", which is wrong for
	 * the legal ONU-ID 0: a real 0 -> N reassignment then left CAM entry 0
	 * live and able to burst into the reassigned grant slot.  The explicit
	 * validity flag says exactly what was meant.  Behaviour is unchanged for
	 * every non-zero id. */
	if (cg->omcc_alloc_valid && old != alloc_id)
		cg_tcont_unbind(cg, old);	/* invalidate the stale OMCC entry */

	if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, alloc_id))
		return;
	d = readl(cg->mac + CG_REG_TCONT_DATA);
	d &= ~CG_TCONT_INDEX_MASK;			/* index = 0 (OMCC T-CONT) */
	d |= CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN;
	writel(d, cg->mac + CG_REG_TCONT_DATA);
	if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, CG_TBL_WR | alloc_id))
		return;

	cg->omcc_alloc = alloc_id;
	cg->omcc_alloc_valid = true;	/* set ONLY here, after both table ops
					 * succeeded: a bind that timed out leaves
					 * the shadow invalid, so the post-O5
					 * supervisor retries it on the next tick */
	dev_info(cg->dev, "OMCC: T-CONT[0] bound to alloc-id %u\n", cg->omcc_alloc);
}

/*
 * Bind the OMCC upstream GEM: us-gem hw indices 0..7 are reserved for the
 * OMCC; point them all at the OLT-assigned omci_port.id (vendor
 * aal_gpon_omcc_gem_enable -> aal_gpon_us_gem_port_set).
 */
static void cg_omcc_gem_bind(struct cortina_gpon *cg, u32 gem_id)
{
	u32 idx, d, n;

	/* walk the DECLARED OMCC slot run rather than re-deriving it here; the
	 * 12-bit Port-ID mask is the G.984.3 field width and lives in the
	 * shared layer (gpon_gem_us_port_id) so the two families state it once */
	for (n = 0; n < cg_us_omcc_slots.count; n++) {
		idx = gpon_gem_us_index(&cg_us_omcc_slots, n);
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, idx))
			return;
		d = readl(cg->mac + CG_REG_US_PORT_DATA);
		d = (d & ~0xfffu) | gpon_gem_us_port_id(gem_id);
		writel(d, cg->mac + CG_REG_US_PORT_DATA);
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, CG_TBL_WR | idx))
			return;
	}

	cg->omcc_gem = gpon_gem_us_port_id(gem_id);
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

/* Invalidate a DS GEM CAM entry: clear the valid bit (and index) for GEM
 * port-id `gem_id` (vendor aal_gpon_ds_gem_port_set with vld=0), so a
 * reassigned DS GEM no longer routes into this ONU's de-encap path. */
static int cg_ds_gem_unbind(struct cortina_gpon *cg, u32 gem_id)
{
	if (cg_tbl_op(cg, CG_REG_DS_GEM_ACCESS, gem_id & 0xfff))
		return -ETIMEDOUT;
	writel(0, cg->mac + CG_REG_DS_GEM_DATA);	/* vld=0, index=0 */
	return cg_tbl_op(cg, CG_REG_DS_GEM_ACCESS, CG_TBL_WR | (gem_id & 0xfff));
}

/*
 * Tear down the currently-armed WAN data path in the vendor drain-then-clear
 * order (aal_gpon_datapath_reset -> aal_puc_voq_flush drain_out): disable the
 * data T-CONT's VoQs and flush them FIRST (so the scheduler stops draining
 * before the CAM changes underneath it), THEN clear the US GEM port stamps, the
 * DS GEM CAM (unicast + broadcast), and finally invalidate the data T-CONT CAM
 * entry.  Keyed on the ARMED identity (hw_data_alloc / hw_data_gem); the CALLER
 * decides WHEN this fires (only a genuine alloc/gem change or an OLT deprovision
 * — never a same-{alloc,gem} re-range).  Runs in the isr_work context.
 */
static void cg_data_teardown(struct cortina_gpon *cg)
{
	u32 i;

	/* 1. drain/disable the data T-CONT's VoQs before touching the CAM */
	cg_puc_pvtbl_program(cg, CG_DATA_TCONT_IDX, false);
	cg_puc_voq_flush(cg, CG_DATA_TCONT_IDX);

	/* 2. clear the US GEM port stamps for the data VoQs (8..15), walking the
	 * declared data slot run.  This is a WHOLE-REGISTER write and not the
	 * read-modify-write the install path uses — US_PORT_DATA is id[11:0] and
	 * nothing else, so the two are equivalent here; the asymmetry is
	 * pre-existing and deliberately left as it was. */
	for (i = 0; i < cg_us_data_slots.count; i++) {
		u32 idx = gpon_gem_us_index(&cg_us_data_slots, i);

		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, idx))
			break;
		writel(GPON_GEM_US_PORT_NONE, cg->mac + CG_REG_US_PORT_DATA);
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, CG_TBL_WR | idx))
			break;
	}

	/* 3. invalidate the DS GEM CAM (unicast data GEM + the broadcast GEM) */
	if (cg->hw_data_gem)
		cg_ds_gem_unbind(cg, cg->hw_data_gem);
	cg_ds_gem_unbind(cg, CG_MCAST_GEM_ID);

	/* 4. finally invalidate the data T-CONT CAM entry — but NEVER the OMCC's:
	 * on a single-alloc OLT the data path rides the OMCC alloc and its
	 * T-CONT (index 0) must stay armed for OMCI/PLOAM. */
	if (cg->hw_data_alloc && cg->hw_data_alloc != cg->omcc_alloc)
		cg_tcont_unbind(cg, cg->hw_data_alloc);

	dev_info(cg->dev, "data path torn down (alloc %u, gem %u): VoQs drained, CAM cleared\n",
		 cg->hw_data_alloc, cg->hw_data_gem);
	cg->hw_data_alloc = 0;
	cg->hw_data_gem = 0;
	/* the L3FE US hit-action must stop targeting the stale GEM */
	cortina_ni_gpon_data_path_set(0, 0);
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
 *
 * Stale-CAM guard: if the OLT-provisioned {alloc, gem} shadow no longer matches
 * what is ARMED (a WAN service reconfig, or a wipe via on-wire MIB-Reset), tear
 * the stale HW path down FIRST so a reassigned alloc/gem can never burst; a
 * same-{alloc,gem} state matches exactly and takes no branch (no HW writes ->
 * no re-provision churn — the proven LOS/fiber-pull keep-path).
 */
static void cg_data_try_install(struct cortina_gpon *cg)
{
	u32 alloc = READ_ONCE(cg->dt_alloc);
	u32 gem = READ_ONCE(cg->dg_gem);
	u32 d, i;

	if (!cg->omcc_up)
		return;

	if (cg->hw_data_alloc &&
	    (cg->hw_data_alloc != (alloc & 0xfff) ||
	     cg->hw_data_gem != (gem & 0xfff))) {
		cg_data_teardown(cg);
		cg->data_installed = false;
	}

	/* "has the OLT provisioned BOTH halves yet" is a provisioning-lifecycle
	 * gate and stays here: the shared verdict below deliberately does not
	 * judge a zero Alloc-ID, because Luna has no such guard and adding one
	 * would change Luna's behaviour. */
	if (cg->data_installed || !alloc || !gem)
		return;

	/*
	 * ★★ THE ALLOC-ID -> T-CONT DECISION IS COMMON, and this is the one call
	 * that makes it so.  gpon_gem_us_tcont_decide() lives in
	 * drivers/net/gpon/gpon_gem_us.c because it is a G.984.3 fact, not a
	 * Cortina one: on ANY GPON ONU, binding the data Alloc-ID to the data
	 * T-CONT when that Alloc-ID is ALSO the OMCC's moves the OMCC off its own
	 * T-CONT, the OLT's grants for the management Alloc-ID stop resolving,
	 * and the ONU goes silent with nothing reporting an error.  That is the
	 * proven 9602C regression, and both families must refuse it identically.
	 *
	 * The two inputs are PASSED, not derived, so this move changes no byte on
	 * either target: @omcc_alloc is our cg->omcc_alloc (the Alloc-ID actually
	 * bound to hw T-CONT 0), where Luna passes its live ONU-ID; @already_bound
	 * is our cg->data_installed ("the whole data path is armed"), where Luna
	 * passes a narrower "this Alloc-ID is bound" flag.  Both are u16 here and
	 * the comparison is the same one as before — cg->dt_alloc and
	 * cg->omcc_alloc are both u16, so the u32 locals above carry no bits the
	 * verdict could lose.
	 *
	 * BIND_DONE cannot be reached from here: the data_installed early-out
	 * above already returned.  It is spelled out rather than folded into the
	 * default so the shared enum stays exhaustively handled at every call
	 * site, and so a later reader who removes that early-out gets a compiler
	 * warning instead of a silent re-bind.
	 */
	switch (gpon_gem_us_tcont_decide((u16)alloc, cg->omcc_alloc,
					 cg->data_installed)) {
	case GPON_GEM_US_BIND_DONE:
		return;
	case GPON_GEM_US_BIND_IS_OMCC:
		/* single-alloc OLT: rebinding the CAM would steal the OMCC's
		 * T-CONT (proven 9602C regression).  Leave the CAM alone and
		 * warn — the data path then needs the ride-the-OMCC variant. */
		dev_warn(cg->dev,
			 "data alloc %u == OMCC alloc: NOT rebinding CAM (single-alloc OLT?)\n",
			 alloc);
		break;
	case GPON_GEM_US_BIND_TCONT:
		if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, alloc & 0xfff))
			return;
		d = readl(cg->mac + CG_REG_TCONT_DATA);
		/*
		 * ★fix#100 (2026-08-11) TEST KNOB - the data T-CONT's OMCI/PLOAM enables.
		 *
		 * alloc 351 is a PURE DATA T-CONT: it carries neither OMCI nor PLOAM (those
		 * ride the OMCC on alloc-id == onu-id, bound separately by
		 * cg_omcc_tcont_bind() to hw T-CONT 0).  Yet this bind sets omci_en|ploam_en
		 * on it, and the live table reads tcont[351] = 0x7 = {ploam_en, omci_en,
		 * index 1}.
		 *
		 * WHY THIS IS WORTH ONE BOOT: fix#99 gave us a real upstream witness, and it
		 * says the PADI is NEVER FRAMED - us_mib[sel=8] (data GEM 225) counts ZERO
		 * transmitted frames across a burst that provably sent PADIs, while
		 * us_mib[sel=7] correctly tracks US OMCI.  Grant starvation is ruled out:
		 * bufocc tcont1 read 0 on all six samples DURING a dial, and a grant-starved
		 * queue accumulates and persists rather than reading zero.  So the frame dies
		 * between enqueue and the framer, and a data T-CONT that claims to be an
		 * OMCI/PLOAM T-CONT is the most specific difference on the table.
		 *
		 * ⚠ NOT a finding - the header names these bits but does not say what the
		 * framer does when a data-only T-CONT asserts them.  Hence a knob, default
		 * UNCHANGED, so the claim is measured rather than believed.
		 *
		 * ORACLE (no PPPoE needed - any upstream data frame will do):
		 *   us_mib[sel=8] pkt CLIMBS  => the frame reached the fibre; these bits were
		 *                                the gate.
		 *   us_mib[sel=7]             => always-on positive control (US OMCI provably
		 *                                transmits), so a dead read is distinguishable
		 *                                from a dead datapath.
		 */
		d &= ~CG_TCONT_INDEX_MASK;
		d |= CG_TCONT_INDEX(CG_DATA_TCONT_IDX);
		if (data_tcont_omci)
			d |= CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN;
		else
			d &= ~(CG_TCONT_OMCI_EN | CG_TCONT_PLOAM_EN);
		writel(d, cg->mac + CG_REG_TCONT_DATA);
		if (cg_tbl_op(cg, CG_REG_TCONT_ACCESS, CG_TBL_WR | (alloc & 0xfff)))
			return;
		break;
	}

	/*
	 * ★fix#102: stock's queue_add ENABLE-TRIPLE, which this driver has never run.
	 * Per provisioned data VoQ stock does [dal_rtl9607f_ponmac.c:1161-1181]:
	 *   (1) TE_CB threshold-profile 3   = ADMISSION      -> cortina_ni_qm_voq_admit()
	 *   (2) PUC_QM_REPORT_ENABLE0        = DBA report     -> below
	 *   (3) aal_l3_tm_es_voq_ena_set     = L3QM ES enable -> NOT LANDED, see below
	 * qm_voq_idx = 128 + puc_voq_idx [ponmac :641]; data VoQs are puc 8..15 -> qm 0x88..0x8F.
	 *
	 * Without (2) the OLT grants this T-CONT NO upstream bandwidth, so even an admitted
	 * frame has no slot - which is why (1) alone measured nothing when poked live.
	 *
	 * ⛔ (3) IS DELIBERATELY NOT LANDED.  Its register is [ASSUMED] - the body lives only in
	 * the stock binary, the header gives no address, and QM_QM_ES_CTRL is one of the
	 * registers taurus_addr_override.h RELOCATES (generic 0xf4306108 vs Taurus
	 * QM_QM_ES_CTRL2 0xf4306968).  This project has retracted six register claims for being
	 * asserted rather than measured; guessing this one would be the seventh.  Resolve it
	 * from live stock first, then add it here.
	 */
	if (qm_voq_enable) {
		unsigned int q;

		/*
		 * fix#104: the two enables are now independently gated (qm_voq_admit /
		 * qm_voq_report) so RC19's async SError can be pinned to one of them.
		 * Boot admit=1/report=0 first: if stable + data frames, step (2) is not
		 * needed; if it still SErrors, the TE_CB admission is the culprit.
		 */
		if (qm_voq_admit)
			for (q = 0; q < 8; q++)
				cortina_ni_qm_voq_admit(128 + CG_DATA_TCONT_IDX * 8 + q);
		/*
		 * ★fix#103 (2026-08-12): step (2) is a BLIND WRITE, never a readl.
		 *
		 * Image 145 (RC18) panicked exactly here:
		 *   Internal error: synchronous external abort 0x96000010, Workqueue cg_isr_work
		 *   pc : readl+0x0/0x20   x0 : pon+0x82d0
		 * i.e. the readl of PUC_QM_REPORT_ENABLE0 in the old RMW took a bus abort.
		 * The address is header-confirmed (0xf55082d0 = PUC_QM_REPORT_ENABLE0,
		 * rtl9607f_registers.h:90327) with _dft=0 and field enable0[31:0].  The PUC block
		 * is otherwise readable (BTCCFG/CTRL/CTRL2 do RMW at boot without faulting), so
		 * ENABLE0..7 (0x82b4..0x82d0) are WRITE-ONLY.  Track B is the sole writer of this
		 * register (grep-verified) and dft=0, so a blind write of bits 8..15 is
		 * byte-identical to the RMW result - without the fatal read.
		 * Bits 8..15 = puc VoQ 8..15 = the data T-CONT's queues.
		 */
		/* fix#106: token-bucket init now has its OWN gate, so the RC21 SError can be
		 * isolated to the TBC writes vs the REPORT_ENABLE write. */
		if (qm_voq_tbc)
			for (q = 0; q < 8; q++)
				cg_puc_voq_tbc_init(cg, CG_DATA_TCONT_IDX * 8 + q);
		if (qm_voq_report)
			writel(qm_voq_report_mask, cg->pon + 0x82d0);
		dev_info(cg->dev,
			 "fix#107: admit=%u tbc=%u report=%u mask=0x%08x (TE_CB qm_voq 0x%02x..0x%02x profile 3; PUC VoQ %u..%u token-buckets; REPORT_ENABLE0(0x82d0))\n",
			 qm_voq_admit, qm_voq_tbc, qm_voq_report, qm_voq_report_mask,
			 128 + CG_DATA_TCONT_IDX * 8, 128 + CG_DATA_TCONT_IDX * 8 + 7,
			 CG_DATA_TCONT_IDX * 8, CG_DATA_TCONT_IDX * 8 + 7);
	}

	/* US: every VoQ of the data T-CONT stamps the data GEM port-id */
	for (i = 0; i < cg_us_data_slots.count; i++) {
		u32 idx = gpon_gem_us_index(&cg_us_data_slots, i);

		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, idx))
			return;
		d = readl(cg->mac + CG_REG_US_PORT_DATA);
		d = (d & ~0xfffu) | gpon_gem_us_port_id(gem);
		writel(d, cg->mac + CG_REG_US_PORT_DATA);
		if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, CG_TBL_WR | idx))
			return;
	}

	/* DS: unicast data GEM + the broadcast GEM (DHCP OFFER rides it) */
	if (cg_ds_gem_bind(cg, gem, CG_DATA_GEM_IDX) ||
	    cg_ds_gem_bind(cg, CG_MCAST_GEM_ID, CG_MCAST_GEM_IDX))
		return;

	/* PDC: both intern indices -> CPU port 0, forwarding-engine bypass,
	 * lspid = PON (the NI CPU-RX WAN-delivery key).
	 * ★ HW-L3-forward staging (gated, default OFF): with
	 * cortina_ni.hw_l3_fwd=1 the UNICAST data GEM instead takes the
	 * vendor-default DS route LDPID = L3_WAN (0x18), no FE bypass - the
	 * PDPID map hands it to the L3FE WAN ingress (l3fe_rx counts it),
	 * STG0 maps 0x18 -> WAN LPB profile -> T1 classifier -> T2 main-hash
	 * consult; a MISS punts to CPU_0 via the internal default action, so
	 * DHCP/ARP/unmatched traffic still reaches Linux.  The broadcast GEM
	 * (i == 1, DHCP OFFER rides it) keeps the proven CPU delivery. */
	for (i = 0; i < 2; i++) {
		u32 idx = CG_DATA_GEM_IDX + i;
		u32 d0 = CG_PDC_D0_COS(0) |
			 CG_PDC_D0_LDPID(CG_LPORT_CPU_0) |
			 CG_PDC_D0_LSPID(CG_LPORT_PON) |
			 CG_PDC_D0_FE_BYPASS | CG_PDC_D0_NO_DROP;

		if (i == 0 && cortina_ni_hw_l3_fwd_active() && cg_hw_l3_ds) {
			d0 = CG_PDC_D0_LDPID(CG_LPORT_L3_WAN) |
			     CG_PDC_D0_LSPID(CG_LPORT_PON);
			dev_info(cg->dev,
				 "PDC: data GEM idx %u -> L3_WAN (HW L3-forward DS armed)\n",
				 idx);
		} else if (i == 0) {
			/* ★ Say it PLAINLY, because this is the DS-offload
			 * precondition and its absence is invisible from the
			 * L3FE side: with FE_BYPASS the DS data GEM goes
			 * straight to CPU port 0 and skips BOTH forwarding
			 * engines, so a DS main-hash entry can NEVER be hit
			 * however correct it is.  Arming cortina_ni.hw_ds_offload
			 * alone is not enough - hw_l3_ds must be on too. */
			dev_info(cg->dev,
				 "PDC: data GEM idx %u -> CPU_0 + FE_BYPASS (hw_l3_fwd=%d hw_l3_ds=%d) - DS frames BYPASS the L3FE, so no DS HW-flow entry can be hit; set cortina_gpon.hw_l3_ds=1 to route DS into the L3FE\n",
				 idx, cortina_ni_hw_l3_fwd_active(),
				 cg_hw_l3_ds);
		}
		if (i == 0)
			cortina_ni_gpon_ds_route_set(!(d0 & CG_PDC_D0_FE_BYPASS));

		if (cg_pdc_map_write(cg, idx, d0, CG_PDC_D1_POL_ID(idx)))
			return;
	}

	/* PUC: enable the data T-CONT's VoQs, then the flush workaround */
	if (cg_puc_pvtbl_program(cg, CG_DATA_TCONT_IDX, true))
		return;
	cg_puc_voq_flush(cg, CG_DATA_TCONT_IDX);

	cg->data_installed = true;
	cg->hw_data_alloc = alloc & 0xfff;	/* record the armed identity so a */
	cg->hw_data_gem = gem & 0xfff;		/* later reconfig can invalidate it */
	/* report the LIVE data-path identity to the L3FE offload backend (the
	 * US hit-action's mcgid/T-CONT source - never a compiled-in constant) */
	cortina_ni_gpon_data_path_set(cg->hw_data_gem, CG_DATA_TCONT_IDX);
	dev_info(cg->dev,
		 "DATA path UP: alloc %u -> T-CONT %u, gem %u (US VoQ %u, DS idx %u), bcast %u -> idx %u\n",
		 alloc, CG_DATA_TCONT_IDX, gem, CG_DATA_GEM_IDX,
		 CG_DATA_GEM_IDX, CG_MCAST_GEM_ID, CG_MCAST_GEM_IDX);
	if (cg->wan_ndev)
		netif_carrier_on(cg->wan_ndev);
}

/*
 * Datapath reset on O5 exit.  Drops the soft link state so the next O5 re-binds
 * cleanly, and — crucially — does NOT tear the HW T-CONT/GEM CAM down here.
 *
 * WHY the CAM teardown is NOT unconditional at O5 exit:  the OLT does not
 * re-send Assign_Alloc-ID on a plain LOS/fiber-pull re-range and keeps the ONU
 * provisioned, so clearing the CAM on every exit would (a) force a needless
 * re-provision = the LOAi churn the alloc-reuse path fixed, and (b) regress the
 * proven 30/30 fiber-pull / 5/5 cold-boot keep-path.  At exit we also don't yet
 * know whether the next O5 carries the SAME or a DIFFERENT alloc/gem/onu-id.
 *
 * So the stale-CAM invalidation is instead deferred to the exact point a
 * genuinely DIFFERENT identity REPLACES the armed one (vendor
 * aal_gpon_datapath_reset semantics, applied surgically):
 *   - OMCC alloc / onu-id change  -> cg_omcc_tcont_bind invalidates the old
 *     entry during ranging (Assign_ONU-ID is an O4 PLOAM, i.e. BEFORE O5);
 *   - data alloc/gem change or an on-wire MIB-Reset wipe -> cg_data_try_install
 *     detects the armed-vs-provisioned mismatch and runs cg_data_teardown
 *     (drain-then-clear) before re-installing.
 * A same-{alloc,gem,onu-id} re-range matches on both paths and writes no HW —
 * byte-for-byte the proven keep-path.
 */
static void cg_datapath_reset(struct cortina_gpon *cg)
{
	cg->omcc_up = false;
	/* disarm the responder: the next O5 re-inits it with a fresh MIB
	 * (the OLT re-provisions after a deact/re-range) */
	spin_lock_bh(&cg->omci_lock);
	cg->omci_active = false;
	spin_unlock_bh(&cg->omci_lock);
	cancel_delayed_work(&cg->veip_avc_work);
	/* data path link down; the dt_/dg_ shadow AND the hw_data_* armed
	 * identity SURVIVE so the O5 re-entry re-installs even when the OLT does
	 * not re-provision (LOS re-range), and a genuine reconfig can still tell
	 * armed-vs-new apart to invalidate only the stale entry. */
	cg->data_installed = false;
	if (cg->wan_ndev)
		netif_carrier_off(cg->wan_ndev);
	/* Re-arm the stuck-O1 recovery watchdog: the analog lock can be LOST
	 * mid-uptime (long-LOS laser cool-down, OLT outage, EMI) and the ensuing
	 * re-range can then wedge at O1 with no further events — the same
	 * cold-start signature, needing the same recovery (relock_rearm_test
	 * case [b]).  Fresh episode = fresh fast-retry budget; 15 s grace so a
	 * healthy re-range (which leaves O1 in seconds) is never disturbed. */
	cg->coldstart_tries = 0;
	/* mod_ and not schedule_: the post-O5 supervisor leaves this delayed work
	 * permanently PENDING, and schedule_delayed_work() on a pending work is a
	 * NO-OP - the watchdog would then inherit whatever remained of the
	 * supervisor's own 30 s deadline and could fire almost immediately after
	 * an O5 exit, re-rolling the SerDes in the middle of a perfectly healthy
	 * Deactivate re-range (a fiber pull is safe either way, DS is unlocked and
	 * the !ds_locked branch only waits, but an OLT-driven Deactivate with the
	 * fiber still lit is not).  mod_delayed_work() re-imposes exactly the 15 s
	 * grace this path had before the supervisor existed. */
	mod_delayed_work(system_wq, &cg->coldstart_work, 15 * HZ);
	dev_warn(cg->dev, "O5 exit: datapath reset (OMCC + data down, CAM shadow kept)\n");
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

	/* Stage C: arm the G.988 responder on a fresh MIB.  The ME-256 serial
	 * number is cg->sn -- the very bytes cg_mac_activate() programmed into
	 * the MAC's vendor-id/vendor-specific registers, so the identity the OLT
	 * ranged cannot differ from the one OMCI reports (one source of truth).
	 * The MIB-Data-Sync seed 200 is a POISON: it must NOT match the OLT's
	 * stored lsync, so its ME2 audit mismatches and it re-provisions from
	 * MIB-Reset (the X111W warm-readmit lesson; the on-wire MIB-Reset
	 * then zeroes it).  Also start the ~31s VEIP oper-up AVC timer. */
	if (cg->omci) {
		char sn_str[13];

		spin_lock_bh(&cg->omci_lock);
		/* CUT SITE: the ME model + MIB reset MOVED to omci_onu_init() in
		 * drivers/net/gpon/gpon_omci_me.c */
		omci_onu_init(cg->omci, cg->sn, 200);
		cg->omci_active = true;
		spin_unlock_bh(&cg->omci_lock);
		cg->veip_avc_retry_ms = 0;
		/* Always schedule; the fire-time guard in cg_veip_avc_work() defers
		 * while the userspace bridge owns the OMCC and emits only if the
		 * daemon is not (or no longer) registered. */
		schedule_delayed_work(&cg->veip_avc_work, 31 * HZ);
		if (cg_stats_s > 0)
			schedule_delayed_work(&cg->stats_work, cg_stats_s * HZ);
		cg_sn_format(cg->sn, sn_str);
		dev_info(cg->dev, "OMCI responder armed (%u MIB rows, mds seed 200, sn %s)\n",
			 cg->omci->nrows, sn_str);
	}
}

/*
 * US OMCI TX (Stage C): the 48-byte PDU (trailer + MIC already stamped by
 * the responder) goes out the NI DMA-LSO ring; the HW GEM-encapsulates it
 * onto the OMCC upstream on the next matching BWmap grant.
 */

/*
 * ★ 2026-08-08 AOT5221ZY: periodic OMCI counter dump.
 * Driving `cat /proc/gpon` over the serial console proved unreliable (the failsafe
 * shell did not echo our writes), so the driver reports the same numbers itself.
 * THE QUESTION THIS ANSWERS: is the OLT sending downstream OMCI at all?
 *   ds_omci_rx > 0  -> OLT IS talking and we are dropping/ignoring frames
 *   ds_omci_rx == 0 -> we have received nothing in software
 *
 * ★★★ 2026-08-08 fix#43 — DO NOT TRUST hw_gem/hw_pkt.  They were introduced as
 * "the MAC's own DS-OMCI hardware counters, which count even if software drops the
 * frame", and the reading "hw_pkt=15 != 0 therefore the OLT DID send us downstream
 * OMCI" became the premise for a long hunt for a MAC->CPU drop.  THAT PREMISE IS
 * WRONG.  Arming this worker at probe instead of at responder-arm shows hw_gem=1 and
 * hw_pkt=15 ALREADY at 4.6 s, while the ONU is still in O1 (onu=0x000100ff, not yet
 * ranged, no ONU-ID, OMCC down) - and they read exactly 1/15 on every boot and never
 * change.  A traffic counter cannot do that.  Whatever these two offsets are, they
 * are not counting DS OMCI, so they are evidence of NOTHING about OLT activity.
 * (By contrast bm_tx(0x2140) starts at 0 and moves, which is what a live counter
 * looks like.)  Until a DS-arrival indicator is validated against a known-good
 * reference, ds_omci_rx is the only trustworthy one here.
 * Enable with cortina_gpon.stats_s=<seconds>.
 */
/*
 * ★★★★ 2026-08-08 fix#57: dump the PON-window registers the driver itself defines, so the
 * PON/PUC path can be diffed against Track A the same way the NI window was (that diff is
 * what found fix#52 after ten hand-picked hypotheses failed).
 * The GPON MAC / PUC / PDC blocks are NOT in rtl9607f_registers.h - they were RE'd
 * separately - so the "named-on-Taurus" filter used for the NI window is unavailable here.
 * Instead dump exactly the offsets this driver already reads or writes: those are known to
 * decode, which sidesteps the synchronous external abort a blind sweep would take.
 *
 * ⛔ 2026-08-11 CORRECTION to the sentence above - "or writes" IS NOT A SAFETY ARGUMENT.
 * MMIO writes are POSTED: they retire into the fabric whether or not anything answers, so a
 * write can never establish that an address decodes.  Only a READ THAT RETURNED can.  This
 * was proven the expensive way on the NI side: cortina_ni_rx_fbm_init() writes the FBM
 * glb/axi/pool windows and never reads them, and the first read of glb+0x00 - from a /proc
 * handler - took a synchronous external abort and panicked the board.  This list happens to
 * be safe because these PON offsets ARE read on the live O5 path, not because they are
 * written.  Anything added here must be justified by a read.
 * Enable with cortina_gpon.dump_pon=1; fires once, late (same reasoning as fix#51 - before
 * the netdev opens, half the state is simply not written yet).
 */
static const u16 cg_pon_dump_offs[] = {
	0x0074, 0x007c, 0x0094, 0x0130, 0x0134, 0x013c, 0x0140, 0x0194, 0x0198, 0x01b0, 0x01b4, 0x022c,
	0x042c, 0x1000, 0x6000, 0x600c, 0x6010, 0x6014, 0x6018, 0x601c, 0x6020, 0x6024, 0x6078, 0x607c,
	0x6080, 0x6084, 0x6088, 0x608c, 0x6090, 0x6094, 0x6098, 0x609c, 0x60a0, 0x60a4, 0x60a8, 0x60bc,
	0x60c0, 0x60c8, 0x60d8, 0x60fc, 0x6114, 0x6118, 0x612c, 0x6130, 0x6134, 0x6138, 0x6170, 0x6174,
	0x61a8, 0x61ac, 0x61b0, 0x61b4, 0x61b8, 0x61bc, 0x61c0, 0x61c4, 0x61c8, 0x61cc, 0x61e0, 0x61f8,
	0x61fc, 0x6800, 0x6804, 0x6808, 0x680c, 0x6810, 0x6814, 0x6818, 0x6e14, 0x8000, 0x8004, 0x8008,
	0x800c, 0x8010, 0x8014, 0x804c, 0x8050, 0x808c, 0x8090, 0x80d0, 0x80d4, 0x80d8, 0x80dc, 0x80e4,
	0x80e8, 0x80ec, 0x80f4, 0x813c, 0x8140, 0x8144, 0x814c, 0x8150, 0x8154, 0x8158, 0x815c, 0x8160,
	0x8164, 0x8168, 0x8174, 0x8178, 0x817c, 0x8180, 0x8184, 0x8188, 0x8194, 0x8198, 0x81bc, 0x8230,
	0x8234, 0x9014, 0x9020, 0x9024, 0x9028,

	/*
	 * ★ fix#75 (2026-08-10): the rest of the NAMED GPON-MAC block (rtl8277c_registers.h,
	 * 0xf5506000-0xf55063ff).  The 113-offset list did not cover the UPSTREAM side at all -
	 * no bwmap_ctl/data (0x178/0x17c/0x180), no bwmap_drop (0x19c), no US_MIB, no
	 * DEBUG_STATUS0/1 - and the frame is enqueued at the PUC and never emitted, so the
	 * scheduler/framer that consumes BWmap grants is exactly what we cannot see.
	 * Filtered to registers NAMED on Taurus (a blind sweep takes a synchronous external
	 * abort - measurement trap #4).
	 */
	0x6004, 0x6008, 0x6028, 0x602c, 0x6030, 0x6034, 0x6038, 0x603c, 0x6040, 0x6044, 0x6048, 0x604c,
	0x6050, 0x6054, 0x6058, 0x605c, 0x6060, 0x6064, 0x6068, 0x606c, 0x6070, 0x6074, 0x60ac, 0x60b0,
	0x60b4, 0x60b8, 0x60c4, 0x60cc, 0x60d0, 0x60d4, 0x60dc, 0x60e0, 0x60e4, 0x60e8, 0x60ec, 0x60f0,
	0x60f4, 0x60f8, 0x6100, 0x6104, 0x6108, 0x610c, 0x6110, 0x611c, 0x6120, 0x6124, 0x6128, 0x613c,
	0x6140, 0x6144, 0x6148, 0x614c, 0x6150, 0x6154, 0x6158, 0x615c, 0x6160, 0x6164, 0x6168, 0x616c,
	0x6178, 0x617c, 0x6180, 0x6184, 0x6188, 0x618c, 0x6190, 0x6194, 0x6198, 0x619c, 0x61a0, 0x61a4,
	0x61d0, 0x61d4, 0x61d8, 0x61dc, 0x61e4, 0x61e8, 0x61ec, 0x61f0, 0x61f4,
};

static bool cg_dump_pon;
module_param_named(dump_pon, cg_dump_pon, bool, 0444);
MODULE_PARM_DESC(dump_pon, "dump the PON/PUC window once after O5 (for the Track A/B diff)");
/*
 * ★★★ fix#74 (2026-08-10): WHICH STATS TICK THE DUMP FIRES ON.
 *
 * fix#57 fired it on the 5th stats tick, which with stats_s=5 lands at t=28s - and the
 * OMCC T-CONT/US-GEM bindings do not run until O5 completes at t~35s.  The 2026-08-10
 * stock-vs-port diff was therefore comparing OUR window BEFORE those tables are programmed
 * against STOCK's long after, which manufactured four "port reads 0" rows (612c/6130/6134/
 * 6138 T-CONT + DS-GEM access latches, and 6170/6174 - the US GEM port-id table, the single
 * most load-bearing pair in the whole diff).  Exactly measurement trap #2 ("the timing of a
 * dump changes its meaning"), which cost this project a wrong conclusion once before.
 *
 * Default 12 ticks = t~60s with stats_s=5: after O5, after the OMCC bindings, and after the
 * US OMCI replies have started.
 */
static uint cg_dump_pon_tick = 12;
module_param_named(dump_pon_tick, cg_dump_pon_tick, uint, 0644);
MODULE_PARM_DESC(dump_pon_tick,
	"fix#74: which OMCI-STATS tick the PON window dump fires on (must be AFTER the OMCC bind)");

static void cg_pon_window_dump(struct cortina_gpon *cg)
{
	unsigned int i;

	if (!cg_dump_pon)
		return;
	dev_info(cg->dev, "PONWIN %zu offsets (pon-relative)\n",
		 ARRAY_SIZE(cg_pon_dump_offs));
	for (i = 0; i < ARRAY_SIZE(cg_pon_dump_offs); i += 4)
		dev_info(cg->dev, "PONWIN %04x=%08x %04x=%08x %04x=%08x %04x=%08x\n",
			 cg_pon_dump_offs[i],
			 readl(cg->pon + cg_pon_dump_offs[i]),
			 cg_pon_dump_offs[min(i + 1, (unsigned int)ARRAY_SIZE(cg_pon_dump_offs) - 1)],
			 readl(cg->pon + cg_pon_dump_offs[min(i + 1, (unsigned int)ARRAY_SIZE(cg_pon_dump_offs) - 1)]),
			 cg_pon_dump_offs[min(i + 2, (unsigned int)ARRAY_SIZE(cg_pon_dump_offs) - 1)],
			 readl(cg->pon + cg_pon_dump_offs[min(i + 2, (unsigned int)ARRAY_SIZE(cg_pon_dump_offs) - 1)]),
			 cg_pon_dump_offs[min(i + 3, (unsigned int)ARRAY_SIZE(cg_pon_dump_offs) - 1)],
			 readl(cg->pon + cg_pon_dump_offs[min(i + 3, (unsigned int)ARRAY_SIZE(cg_pon_dump_offs) - 1)]));
	dev_info(cg->dev, "PONWIN done\n");
}

static void cg_stats_work(struct work_struct *work)
{
	struct cortina_gpon *cg = container_of(to_delayed_work(work),
					       struct cortina_gpon, stats_work);

	{
		static unsigned int calls;

		if (++calls == cg_dump_pon_tick)
			cg_pon_window_dump(cg);		/* ★fix#57/#74, late (see fix#51) */
	}
	cg_puc_ctrl_sample(cg);		/* ★fix#55: refresh the PUC accumulators first */
	dev_info(cg->dev,
		 "OMCI-STATS: ds_omci_rx=%u short=%u hw_gem=%u hw_pkt=%u | omci_tx=%u fail=%u ds_crc ok=%u bad=%u | us_omcc(0x1e0)=%u | PUC omci_us=%u len_err=%u rx=%u enq=%u drop=%u hdr0=%08x%s hdr1=%08x | omcc=%s alloc=%u gem=%u onu=0x%08x\n",
		 cg->omci_rx, cg->omci_rx_short,
		 cg_mac_rd(cg, CG_REG_DS_OMCI_GEM), cg_mac_rd(cg, CG_REG_DS_OMCI_PKT),
		 cg->omci_tx, cg->omci_tx_fail,
		 cg->omci_ds_crc_ok, cg->omci_ds_crc_bad,
		 /* ★ 2026-08-08 fix#53: the GPON MAC's UPSTREAM OMCC frame counter.  The
		  * responder now answers correctly (omci_trace: rmask=mask, rc=0) yet the OLT
		  * retransmits the same TCI 3x and falls back to MIB-reset - so the question is
		  * whether our reply physically leaves the MAC.  us_omcc climbing with omci_tx
		  * = it egressed and the OLT is rejecting it; flat = it never got onto the
		  * fibre.  Stock never reads this register, so it has no oracle - judge it only
		  * by whether it MOVES. */
		 cg_mac_rd(cg, CG_REG_US_OMCC_CNT),
		 /* ★fix#55: the PUC witnesses (sampled just above so they are current). */
		 cg->puc_omci_us, cg->puc_len_err,
		 readl(cg->pon + CG_PUC_BMC_RX_PKT) & CG_PUC_BMC_CNTR_MASK,
		 readl(cg->pon + CG_PUC_BMC_RX_PKT_ENQ) & CG_PUC_BMC_CNTR_MASK,
		 readl(cg->pon + CG_PUC_BMC_FORCE_DROP) & CG_PUC_BMC_CNTR_MASK,
		 readl(cg->pon + CG_PUC_BMC_PKT_HDR0),
		 readl(cg->pon + CG_PUC_BMC_PKT_HDR0) == CG_PUC_HDR0_GOLDEN ? "(GOLDEN)" : "",
		 readl(cg->pon + CG_PUC_BMC_PKT_HDR1),
		 cg->omcc_up ? "UP" : "down", cg->omcc_alloc, cg->omcc_gem,
		 cg_mac_rd(cg, CG_REG_GPON_ONU));
	/* ★ 2026-08-08 fix#36: pair every OMCI-STATS with the NI delivery ledger, so a
	 * single boot separates "the OLT is not sending" from "the frame is lost between
	 * the MAC and the CPU" - and, if it is lost, at which stage. */
	cortina_ni_rx_delivery_dump();
	/* ★ 2026-08-11: and the UPSTREAM half of the same question, which the ledger above
	 * cannot answer - tx->pon_data_enq counts ENQUEUES, so it says nothing about what
	 * left the fibre.  See cg_us_mib_show(). */
	cg_us_mib_show(cg, NULL);
	if (cg_stats_s > 0)
		schedule_delayed_work(&cg->stats_work, cg_stats_s * HZ);
}

int cg_omci_tx(struct cortina_gpon *cg, const u8 *pdu48)
{
	int ret = -ENODEV;

	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		ret = cortina_ni_pon_tx(pdu48, OMCI_LEN);
	if (ret) {
		cg->omci_tx_fail++;
		dev_warn_ratelimited(cg->dev, "US OMCI TX failed (%d)\n", ret);
	} else {
		cg->omci_tx++;
		/* The frame is on its way to the PUC; read the OMCI-specific
		 * control-packet counter once it has arrived, while its
		 * clear-on-read window still holds it (cg_puc_ctrl_sample). */
		schedule_delayed_work(&cg->puc_cnt_work,
				      msecs_to_jiffies(CG_PUC_CNT_TX_DELAY_MS));
	}
	/* Returned, not swallowed: a solicited response can be left to the OLT's
	 * own retry, but an unsolicited AVC has no such backstop -- see
	 * cg_veip_avc_work(). */
	return ret;
}

#ifdef CONFIG_CORTINA_GPON_OMCI_BRIDGE
/*
 * M2 userspace bridge -- Plane B getOnuState.  Return the live ONU state in the
 * stock daemon's convention (O5 == 5), mapped from the MAC onu.state field.
 * MAC encoding is 0-based (0=O1 .. 4=O5 == CG_STATE_OPERATION); the daemon's
 * appinfo[536] gate wants the 1-based O-number, so O5 -> 5.  A simple +1 maps
 * the whole O1..O7 range correctly.
 */
int cg_omci_bridge_onu_state(struct cortina_gpon *cg)
{
	u32 onu = cg_mac_rd(cg, CG_REG_GPON_ONU);

	return CG_ONU_STATE(onu) + 1;
}
#endif /* CONFIG_CORTINA_GPON_OMCI_BRIDGE */

/*
 * What ME 263 ANI-G #10/#14 currently serve the OLT, and whether that is a real
 * DDM measurement or the static fallback.  Printed right under the optical
 * block so a stub can never be mistaken for a live optical level: "FALLBACK"
 * means the OLT is being told a plausible-looking constant.
 */
/*
 * Print an optical power in centi-dBm, or "-inf" when the optic reports no
 * light at all.  A numeric 0 there would read as a perfectly healthy +0 dBm —
 * exactly the kind of fabricated value the whole DDM path exists to avoid — and
 * "-inf" also makes the scrapers' "(-?\d+)" fail to match, so a consumer sees
 * "no reading" instead of a wrong one.
 */
static void cg_seq_cdbm(struct seq_file *m, s32 cdbm)
{
	if (cdbm == CG_DDM_CDBM_NONE)
		seq_puts(m, "-inf");
	else
		seq_printf(m, "%d", cdbm);
}

static void cg_optic_anig_show(struct cortina_gpon *cg, struct seq_file *m)
{
	if (!cg->omci) {
		seq_puts(m, "optic_anig     = (responder not allocated)\n");
		return;
	}
	seq_printf(m, "optic_anig     = %s  me263 #10 rx=0x%04x #14 tx=0x%04x  (G.988 0.002 dB units)\n",
		   cg->omci->anig_live ? "live" : "FALLBACK (static, no DDM sample yet)",
		   cg->omci->anig_rx_level, cg->omci->anig_tx_level);
}

/*
 * Sample the optic's SFF-8472 A2h diagnostics, print them to @m when it is
 * non-NULL, and publish the two optical levels into ME 263 ANI-G #10/#14 so the
 * OLT's optical view of this ONU is a real measurement rather than a constant.
 *
 * PROCESS CONTEXT ONLY — cg_bosa_ddm_read() sleeps.  The read therefore happens
 * OUTSIDE omci_lock and only the two converted u16 are copied in under it.
 *
 * Sampled on demand (here and at ~31s post-O5, which is when the OLT starts its
 * ANI-G audit) rather than from a timer: ten byte-reads at 100 kHz cost ~1 ms,
 * and the project has been bitten badly by self-invented periodic handlers, so
 * a poll timer would have to earn its keep.  A failed read leaves the
 * responder's conformant static fallback in place — the OLT must never get
 * silence — and prints an explicit "unavailable", never a fabricated 0.
 */
static void cg_optic_sample(struct cortina_gpon *cg, struct seq_file *m)
{
	struct cg_bosa_ddm d;
	s32 rx_cdbm, tx_cdbm;

	if (cg_bosa_ddm_read(cg->dev, &d) != CG_DDM_OK) {
		if (m) {
			seq_printf(m, "optic_ddm      = %s\n",
				   cg_ddm_status_str(d.status));
			cg_optic_anig_show(cg, m);
		}
		return;
	}

	rx_cdbm = cg_ddm_uw10_to_cdbm(d.rx_pwr);
	tx_cdbm = cg_ddm_uw10_to_cdbm(d.tx_pwr);

	if (cg->omci) {
		u16 rx = cg_ddm_cdbm_to_omci(rx_cdbm);
		u16 tx = cg_ddm_cdbm_to_omci(tx_cdbm);

		spin_lock_bh(&cg->omci_lock);
		/* CUT SITE: the ME 263 optical attributes MOVED to omci_onu_set_optical() in
		 * drivers/net/gpon/gpon_omci_me.c (the i2c DDM read that feeds it stays here — it is
		 * hardware) */
		omci_onu_set_optical(cg->omci, rx, tx);
		spin_unlock_bh(&cg->omci_lock);
	}

	if (m) {
		unsigned int i;

		seq_printf(m, "optic_ddm      = live (SFF-8472 A2h 0x%02x-0x%02x)\n",
			   CG_DDM_BASE, CG_DDM_BASE + CG_DDM_LEN - 1);
		/* The RAW word sits beside every scaled value on purpose: the
		 * 0.1 uW LSB is the one thing about RX power this module has not
		 * independently confirmed (see cortina-gpon-ddm.h), so a reader must
		 * always be able to re-derive the level without a firmware change. */
		seq_printf(m, "optic_rx_raw: 0x%04x optic_rx_cdbm: ", d.rx_pwr);
		cg_seq_cdbm(m, rx_cdbm);
		seq_printf(m, " optic_tx_raw: 0x%04x\n", d.tx_pwr);
		seq_printf(m, "optic_env:   temp_dc=%d bias_ua=%u tx_cdbm=",
			   cg_ddm_temp_dc(d.temp), cg_ddm_bias_ua(d.bias));
		cg_seq_cdbm(m, tx_cdbm);
		seq_printf(m, " vcc_mv=%u\n", cg_ddm_vcc_mv(d.vcc));
		seq_printf(m, "optic_ddm_raw: %02x..%02x =",
			   CG_DDM_BASE, CG_DDM_BASE + CG_DDM_LEN - 1);
		for (i = 0; i < CG_DDM_LEN; i++)
			seq_printf(m, " %02x", d.raw[i]);
		seq_putc(m, '\n');
		cg_optic_anig_show(cg, m);
	}
}

/*
 * ★★★ OMCI-PING - the ON-DEMAND POSITIVE CONTROL for the us_mib upstream witness.
 *
 * RB9 came back INCONCLUSIVE for the right reason: across a 120 s pppd dial the upstream
 * counter stayed at 77 - but so did omci_tx, because the OLT stops polling once
 * provisioning is done.  With nothing moving in the window, "the counter is frozen" and
 * "the data GEM transmits nothing" predict exactly the same observation, and no amount of
 * staring at a flat number separates them.  The witness only decides anything if something
 * KNOWN-GOOD moves in the SAME window.
 *
 * So make one on demand.  An unsolicited AVC is a spec-legitimate ONU->OLT frame (G.988;
 * the driver already emits one post-O5), it travels the exact US OMCI path that is
 * independently proven to reach the OLT, and re-sending it cannot disturb the session -
 * which is why this uses the existing AVC builder rather than injecting synthetic OMCI.
 *
 * Usage:  echo 20 > /sys/module/cortina_gpon/parameters/omci_ping
 * Then:   us_mib op2 must rise by ~20.  If it does, the MIB is LIVE IN THIS WINDOW, and a
 *         data burst in the same window that adds nothing is CONCLUSIVE.
 *
 * Runs from a workqueue, not from the sysfs store: cg_omci_tx() reaches into the NI TX
 * ring, and the OMCI path everywhere else in this driver enters from workqueue/softirq
 * context.  A store() is process context with different locking assumptions; matching the
 * established context is cheaper than proving a new one is safe.
 */
static unsigned int cg_omci_ping_n;

static void cg_omci_ping_work_fn(struct work_struct *w)
{
	struct cortina_gpon *cg = container_of(w, struct cortina_gpon, omci_ping_work);
	unsigned int i, n = READ_ONCE(cg_omci_ping_n);
	u8 frame[OMCI_LEN];

	for (i = 0; i < n; i++) {
		bool built = false;

		spin_lock_bh(&cg->omci_lock);
		if (cg->omci_active) {
			/* clear the latch so the builder re-emits every time */
			cg->omci->avc_veip_up_sent = false;
			omci_onu_emit_veip_up_avc(cg->omci, frame);
			built = true;
		}
		spin_unlock_bh(&cg->omci_lock);
		if (!built || cg_omci_tx(cg, frame))
			break;
	}
	dev_info(cg->dev,
		 "OMCI-PING: emitted %u/%u AVC frames - us_mib op2 MUST rise by ~%u; if it does not, the MIB is frozen and any data verdict from this window is void\n",
		 i, n, i);
}

static int cg_omci_ping_set(const char *val, const struct kernel_param *kp)
{
	struct cortina_gpon *cg = READ_ONCE(cg_singleton);
	int ret = param_set_uint(val, kp);

	if (ret)
		return ret;
	if (!cg)
		return -ENODEV;
	schedule_work(&cg->omci_ping_work);
	return 0;
}

static const struct kernel_param_ops cg_omci_ping_ops = {
	.set = cg_omci_ping_set,
	.get = param_get_uint,
};
module_param_cb(omci_ping, &cg_omci_ping_ops, &cg_omci_ping_n, 0644);
MODULE_PARM_DESC(omci_ping,
	"emit N unsolicited VEIP AVCs now - the in-window positive control for the us_mib upstream transmit witness");

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

	/* When the userspace stock daemon owns the OMCC (bridge armed), the
	 * kernel must not emit US OMCI -- the daemon sends its own VEIP AVC.
	 * Defer and re-check rather than give up, so the in-kernel fallback
	 * still sends the one-shot AVC if the daemon ever stops owning the OMCC
	 * (e.g. it exits).  cg->omci->avc_veip_up_sent stops the loop once emitted. */
	if (cg_omci_bridge_armed()) {
		schedule_delayed_work(&cg->veip_avc_work, 31 * HZ);
		return;
	}

	/* Publish a live optical reading before the AVC: this fires ~31s after
	 * O5, i.e. just as the OLT begins auditing ANI-G, so its first optical
	 * GET already gets a measurement instead of the static fallback. */
	cg_optic_sample(cg, NULL);

	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_active && !cg->omci->avc_veip_up_sent) {
		/* CUT SITE: building the VEIP oper-state AVC MOVED to omci_onu_emit_veip_up_avc() in
		 * drivers/net/gpon/gpon_omci_core.c (the workqueue that times it stays here) */
		omci_onu_emit_veip_up_avc(cg->omci, frame);
		emit = true;
	}
	spin_unlock_bh(&cg->omci_lock);
	if (!emit)
		return;

	if (!cg_omci_tx(cg, frame)) {
		cg->veip_avc_retry_ms = 0;
		dev_info(cg->dev, "VEIP oper-up AVC emitted (~31s post-O5)\n");
		return;
	}

	/* The TX failed.  The responder latches avc_veip_up_sent at EMIT time,
	 * so without this clear-back the AVC is lost for the whole session: the
	 * OLT never re-solicits an unsolicited AVC, it simply leaves the service
	 * at Match State Initial and the subscriber has no WAN.  Clear the latch
	 * and retry.
	 *
	 * The retry is rate-bounded but NOT count-capped, deliberately.  A failure
	 * here is invisible to the OLT, so nothing else in the system can ever
	 * recover it; giving up after N attempts would strand the session with no
	 * event that could bring it back.  Backoff doubles to a ceiling so a
	 * persistently failing NI ring cannot spin the workqueue.
	 */
	spin_lock_bh(&cg->omci_lock);
	if (cg->omci_active)
		cg->omci->avc_veip_up_sent = false;
	spin_unlock_bh(&cg->omci_lock);

	cg->veip_avc_retry_ms = cg->veip_avc_retry_ms
		? min(cg->veip_avc_retry_ms * 2u,
		      (unsigned int)CG_VEIP_AVC_RETRY_MAX_MS)
		: CG_VEIP_AVC_RETRY_MIN_MS;
	dev_warn(cg->dev, "VEIP oper-up AVC TX failed; retrying in %u ms\n",
		 cg->veip_avc_retry_ms);
	schedule_delayed_work(&cg->veip_avc_work,
			      msecs_to_jiffies(cg->veip_avc_retry_ms));
}

/*
 * Per-message OMCI trace.  DEFAULT OFF.
 *
 * The always-on instruments answer the AGGREGATE questions: /proc/gpon
 * "ds_omci_rx = N (short=M)" and "omci_resp = armed tx= fail= ds_crc ok= bad=
 * mds= store= avc= unhandled= dup_replay= ext= no_ack=", plus the
 * unconditional event lines ("FSM x -> y", "Deactivate_ONU-ID received",
 * "OMCI cfg mt=.. me=..", "OMCI: data T-CONT/GEM ..").
 *
 * What no counter can answer is the PER-MESSAGE one: which attributes did the
 * OLT request in THIS Get, and which of them did we actually answer?  That
 * comparison is the only way to see the ME-model defect class that strands an
 * OLT in a Get audit loop — an attribute the OLT audits that our ME table does
 * not model at all.  So this trace reports, per downstream PDU: message type,
 * ME class/instance, length and, for a Get:
 *   mask   = attributes the OLT requested       (request octets 8..9)
 *   rmask  = attributes we actually emitted     (response octets 9..10)
 *   unsup  = requested but NOT MODELLED by us   (response octets 36..37)
 *   failed = modelled but did not fit the 25-octet value area (octets 38..39)
 *   rc     = the response result code           (response octet 8)
 *
 * unsup != 0 is the real defect signal.  failed != 0 is legitimate G.988
 * behaviour (a Get whose selected attributes overflow the value area; the OLT
 * must then split it), which is exactly why the two are reported separately
 * instead of just "rmask != mask" — the latter cannot tell a missing attribute
 * from a correctly-reported overflow.
 *
 * Off by default because a MIB-upload walk is ~100 PDUs and this is a shipping
 * image; cost when off is one unlikely() test per downstream OMCI PDU, on the
 * control path, not the packet fast path.  Rate-limited when on so a broken or
 * hostile OLT cannot wedge the console, with a burst generous enough that a
 * whole MIB-upload walk still gets through intact.
 *
 * Enable at runtime:  echo 1 > /sys/module/cortina_gpon/parameters/omci_trace
 * or on the kernel command line:  cortina_gpon.omci_trace=1
 */
static bool cg_gem_from_iwtp = true;
module_param_named(gem_from_iwtp, cg_gem_from_iwtp, bool, 0644);
MODULE_PARM_DESC(gem_from_iwtp,
	"derive the data GEM from the ME 266 GEM-IWTP Create (this OLT never creates a bidirectional ME 268)");

static uint cg_omci_log_n = 24;
module_param_named(omci_log_n, cg_omci_log_n, uint, 0644);
MODULE_PARM_DESC(omci_log_n, "how many DS OMCI PDUs to log before falling back to 1-in-64");

static bool cg_omci_trace;
module_param_named(omci_trace, cg_omci_trace, bool, 0644);

/* ★ 2026-08-22: log EVERY GPON interrupt event word (ranging + post-O5) so we can
 * see exactly what the OLT drives after OMCC-up (does it keep sending DS-PLOAM?
 * a Key_Switching (encryption)? a Deactivate? or go silent = OSS non-provision).
 * Read-only of the already-dequeued ev word - does NOT touch the fragile PLOAM
 * RX FIFO.  Enable via cortina_gpon.ploam_trace=1. */
static bool cg_ploam_trace;
module_param_named(ploam_trace, cg_ploam_trace, bool, 0644);
MODULE_PARM_DESC(omci_trace, "log one line per downstream OMCI PDU: message type, ME class/instance and, for a Get, the requested vs answered vs unmodelled attribute masks (default OFF)");


/*
 * Emit one trace line for the PDU just processed.  @resp/@n are the responder's
 * output (@n == OMCI_LEN when a response was built, 0 when none was) and are
 * only decoded when a response exists — resp[] is otherwise uninitialised.
 */
static void cg_omci_trace_one(struct cortina_gpon *cg, const u8 *pdu,
			      unsigned int len, const u8 *resp, int n,
			      const char *name)
{
	static DEFINE_RATELIMIT_STATE(rs, 5 * HZ, 512);
	bool is_get = (pdu[2] & 0x1f) == 9;	/* G.988 Table 11.2.2-1: Get */
	char det[80];

	if (!__ratelimit(&rs))
		return;
	det[0] = '\0';
	if (is_get && len >= 10 && n == OMCI_LEN)
		scnprintf(det, sizeof(det),
			  " mask=0x%04x rmask=0x%04x unsup=0x%04x failed=0x%04x rc=%u",
			  ((u16)pdu[8] << 8) | pdu[9],
			  ((u16)resp[9] << 8) | resp[10],
			  ((u16)resp[36] << 8) | resp[37],
			  ((u16)resp[38] << 8) | resp[39], resp[8]);
	else if (n != OMCI_LEN)
		scnprintf(det, sizeof(det), " noresp");
	else
		/* ★ 2026-08-10: the result byte for Create/Set/Delete too.  The OLT
		 * provisions us fully and then stops short of creating the data GEM
		 * CTP; "did we NAK one of its Creates?" was unanswerable from this
		 * trace because rc was only ever printed for a Get. */
		scnprintf(det, sizeof(det), " rc=%u", resp[8]);
	dev_info(cg->dev, "OMCI DS: MT=0x%02x %s class=%u inst=%u len=%u%s\n",
		 pdu[2], is_get ? "GET" : name,
		 ((u16)pdu[4] << 8) | pdu[5], ((u16)pdu[6] << 8) | pdu[7],
		 len, det);
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
	/* log the first PDUs + then 1-in-64 (the MIB-upload walk is chatty).
	 * ★ 2026-08-10: the budget is a PARAM now.  Counting message types out of
	 * this log with the old fixed cap of 24 made a MIB-upload walk that was
	 * still running look like it stopped at 15 rows - the cap, not the walk.
	 * cortina_gpon.omci_log_n=300 to see a whole provisioning sequence. */
	if (cg->omci_rx <= cg_omci_log_n || !(cg->omci_rx & 63))
		dev_info(cg->dev,
			 "DS OMCI #%u: len=%u tci=0x%02x%02x mt=%u(%s)%s%s dev=0x%02x me=%u/%u\n",
			 cg->omci_rx, len, pdu[0], pdu[1], mt & 0x1f, name,
			 (mt & BIT(6)) ? " AR" : "", (mt & BIT(5)) ? " AK" : "",
			 pdu[3], (pdu[4] << 8) | pdu[5], (pdu[6] << 8) | pdu[7]);

	/*
	 * ★ 2026-08-11: log the ATTRIBUTE PAYLOAD of Create(4) and Set(8), not just the
	 * header.  The header alone says "the OLT created ME 84/0x0101" and stops exactly
	 * where the interesting part starts: ME 84 is the VLAN tagging filter, and its
	 * payload (filter table, forward_operation, num_entries) is what says whether this
	 * line expects upstream frames TAGGED - and with which VID - or untagged.  The
	 * 08-10 handoff proposes answering that by capturing stock's `moscli dump vlantable`
	 * over a ~10-minute stock boot; the OLT already told US the same thing on every one
	 * of our own boots and we were throwing the bytes away.
	 *
	 * Payload starts at pdu[8] (after tci[2] mt[1] devid[1] class[2] inst[2]) and the
	 * message contents run to byte 40; the trailing 8 are OMCI trailer + MIC.
	 */
	if (cg->omci_rx <= cg_omci_log_n && len >= 40 &&
	    ((mt & 0x1f) == 4 || (mt & 0x1f) == 8))
		dev_info(cg->dev, "DS OMCI #%u: me=%u/%u %s payload %*ph\n",
			 cg->omci_rx, (pdu[4] << 8) | pdu[5],
			 (pdu[6] << 8) | pdu[7],
			 (mt & 0x1f) == 4 ? "Create" : "Set", 32, pdu + 8);

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

		/*
		 * ★★★★★ 2026-08-10 — ME 266 GEM Interworking TP Create: on THIS
		 * OLT the data GEM is named HERE, not by a bidirectional ME 268.
		 *
		 * Measured against live stock on this very line (`moscli dump gem`):
		 *     Gem Port Id: 225   Alloc-Id: 351   Tcont instance ID: 32768
		 * and measured on the wire, the OLT's complete provisioning pass gives
		 * us exactly that pair — Set 262/32768 (alloc 351) plus
		 * Create 266/225 whose CTP connectivity pointer is 225 — while the ONLY
		 * ME 268 it creates is the DS-only multicast CTP (port-id 4095, dir 2),
		 * which the fixed CG_MCAST_GEM_ID install already covers.  It never
		 * sends a bidirectional ME 268, so the dir==3 rule below (written for an
		 * OLT that did: "gem 223, tcont-ptr 0x8000, dir 3") never fires and the
		 * data path was never installed.  Nothing is missing from the OLT's
		 * provisioning — we were matching on the wrong ME.
		 */
		if (cg_gem_from_iwtp && class_id == 266 && m == 4 && len >= 10) {
			u16 ctp = ((u16)pdu[8] << 8) | pdu[9];
			u16 g = ctp ? ctp : inst;

			if (g && g != cg->omcc_gem && g != CG_MCAST_GEM_ID &&
			    g != cg->dg_gem) {
				WRITE_ONCE(cg->dg_gem, g);
				cg->dg_tcont_ptr = cg->dt_inst ? cg->dt_inst
							       : 0x8000;
				cg->dg_dir = 3;	/* a bridged data GEM is bidirectional */
				dev_info(cg->dev,
					 "OMCI: data GEM port-id %u from IWTP (inst %u ctp-ptr %u, tcont-ptr 0x%04x)\n",
					 g, inst, ctp, cg->dg_tcont_ptr);
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

		/* on-wire MIB-Reset: the OLT voided our provisioning.  Drop the
		 * shadow so stale ids are never re-installed, and kick isr_work
		 * so cg_data_try_install invalidates the now-stale HW data CAM
		 * (armed hw_data_* != wiped shadow) in process context — closing
		 * the window where a reassigned alloc could burst before fresh
		 * provisioning arrives. */
		if (m == 15) {
			WRITE_ONCE(cg->dt_alloc, 0);
			WRITE_ONCE(cg->dg_gem, 0);
			cg->data_installed = false;
			if (cg->wan_ndev)
				netif_carrier_off(cg->wan_ndev);
			schedule_work(&cg->isr_work);
		}
	}

	/* ★ M2: when the userspace bridge is armed (cortina_gpon.omci_userspace=1)
	 * and the stock omci_app daemon is registered, hand the DS PDU to userspace
	 * over netlink and skip the in-kernel G.988 responder below.  The daemon
	 * builds the real OLT-provisioned MIB; we only own the MAC datapath. */
	if (cg_omci_bridge_ds(pdu, len))
		return;

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
			/* CUT SITE: the whole G.988 responder — parse, dispatch, build the reply, stamp trailer +
			 * MIC — MOVED to omci_onu_input() in drivers/net/gpon/gpon_omci_core.c */
			n = omci_onu_input(cg->omci, pdu, len, resp);
		spin_unlock(&cg->omci_lock);
		if (n == OMCI_LEN)
			cg_omci_tx(cg, resp);
		if (unlikely(cg_omci_trace))
			cg_omci_trace_one(cg, pdu, len, resp, n, name);
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

		if (unlikely(cg_ploam_trace))
			dev_info(cg->dev,
				 "EVT intr=0x%08x [%s%s%s%s%s%s%s] state=%u id=%u\n",
				 ev.intr,
				 (ev.intr & CG_INT_ONU_ST_CHG) ? "STCHG " : "",
				 (ev.intr & CG_INT_ONU_ID)     ? "ONUID " : "",
				 (ev.intr & CG_INT_PLOAMD)     ? "PLOAMD " : "",
				 (ev.intr & CG_INT_PORTID)     ? "PORTID " : "",
				 (ev.intr & CG_INT_KSW)        ? "KSW " : "",
				 (ev.intr & CG_INT_DACT)       ? "DACT " : "",
				 (ev.intr & ~(CG_INT_ONU_ST_CHG|CG_INT_ONU_ID|CG_INT_PLOAMD|
					      CG_INT_PORTID|CG_INT_KSW|CG_INT_DACT)) ? "OTHER " : "",
				 ev.state, ev.id);

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
				cg_datapath_reset(cg);
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

	/*
	 * Reconcile the soft state against the LIVE FSM register before leaving
	 * the bottom half.  The event ring is fixed-size and cg_isr DISCARDS on
	 * overflow (evt_drop++, counted and forgotten), so ev.state/ev.id are a
	 * LOSSY channel: lose the O5-entry ONU_ST_CHG - and/or the Assign_ONU-ID
	 * and Configure_Port-ID that arrive with it - and the per-event gates
	 * above never re-qualify, because a SETTLED O5 generates no further
	 * state-change interrupt.  The OMCC would then stay down against
	 * perfectly healthy hardware until the OLT deactivated us: PON-wide churn
	 * caused by one dropped interrupt.  CG_REG_GPON_ONU is not lossy, so
	 * re-derive from it and replay what was lost.  (The vendor bottom half is
	 * handed the interrupt word directly and re-reads this register in its
	 * FSM tracker, so it has no drop channel to survive; the ring is ours and
	 * closing it is ours.)
	 *
	 * BRING-UP ONLY.  This can only ADD a bind the hardware says should
	 * exist; an O5 EXIT is deliberately NOT inferred from a polled register -
	 * tearing a link down from a poll is the change that could break every
	 * boot, and the event path above already owns the exit.  Worst case here
	 * is one redundant, idempotent CAM write.
	 *
	 * The FSM tracker is resynced TOGETHER with whatever is latched:
	 * cg->last_state is exactly what the O5-exit test above keys on, so
	 * replaying the OMCC while leaving the tracker at Ranging would convert a
	 * loud OMCC-down wedge into a SILENT carrier-UP desync in which no later
	 * O5 exit is ever detected (gpon0 UP on a dead PON, no re-bind and no
	 * fresh MIB with the MDS poison on re-entry).  Only the forward edge
	 * (-> Operation) is taken, so a poll can never push the tracker back.
	 *
	 * Cost on a converged link: three readl of plain status registers and
	 * ZERO writes.  No indirect ACCESS/DATA engine is touched - the FSM/ONU
	 * register is already read from the hardirq (cg_isr), from
	 * cg_coldstart_work and from /proc, so the documented wedge hazard of the
	 * TX-PLOAM MIB pair does not apply.
	 */
	{
		u32 onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
		u8 live = CG_ONU_STATE(onu);
		u8 id = CG_ONU_ID(onu);

		if (live == CG_STATE_OPERATION) {
			if (cg->last_state != CG_STATE_OPERATION) {
				dev_info(cg->dev,
					 "reconcile: live FSM is %s while the tracker says %s (%u events dropped) - replaying from the register\n",
					 cg_state_name[CG_STATE_OPERATION],
					 cg_state_name[cg->last_state & 7],
					 cg->evt_drop);
				cg->last_state = CG_STATE_OPERATION;
			}
			/* A lost Assign_ONU-ID leaves the OMCC T-CONT unbound, so
			 * the ONU gets no US grant and can answer no OMCI - and
			 * cg_omcc_try_up latches omcc_up on state + omci_port.EN
			 * alone, never on this bind, so this must NOT be gated on
			 * !omcc_up or the unbound case can never heal.  Binds only
			 * when the shadow really disagrees, so a converged link and
			 * every same-id re-range write nothing (the proven
			 * keep-path). */
			if (id != CG_ONU_ID_NONE &&
			    (!cg->omcc_alloc_valid || cg->omcc_alloc != id))
				cg_omcc_tcont_bind(cg, id);
			/* A lost DS-PLOAM edge leaves us.frame_var stale = a US
			 * burst misaligned in the grant window.  Idempotent: it
			 * writes only on a genuine change, and stock recomputes it
			 * on every received DS PLOAM. */
			cg_frame_var_update(cg);
			if (!cg->omcc_up)
				cg_omcc_try_up(cg, live);
		}
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
	u32 v;
	int ret;

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

	cg_mac_intr_arm(cg);	/* the four MAC int groups + int_top */

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

	if (cg->irq >= 0) {
		/* close the gates innermost-out first, so nothing can queue
		 * more work behind the flush below */
		writel(0, cg->mac + CG_REG_INT_TOP_EN);
		v = readl(cg->glb + CG_GLB_NE_ICTL_EN);
		writel(v & ~CG_NE_ICTL_PON_LINE, cg->glb + CG_GLB_NE_ICTL_EN);
	}
	/* ALWAYS flush the bottom half, IRQ or not: the DS OMCI RX hook and the
	 * post-O5 supervisor queue isr_work even when the interrupt path was
	 * never armed, so returning early here would leave a work item running
	 * against a context devm is about to free.  cortina_gpon_remove() has
	 * already cancelled coldstart_work at this point, so nothing can re-queue
	 * after this flush. */
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
	/* nf_flow_table HW offload: an nft flowtable with `flags offload`
	 * BINDs a flow block on EVERY hooked device - gpon0 included - so the
	 * WAN netdev must expose the same setup_tc entry as eth0 (cortina-ni).
	 * Without it the flowtable offload setup fails (-EOPNOTSUPP) and fw4
	 * falls back to software offloading.  Flow installs stay gated by
	 * hw_l3_fwd inside the backend; a plain BIND writes no hardware. */
#if IS_REACHABLE(CONFIG_CORTINA_NI)
	.ndo_setup_tc		= cortina_ni_setup_tc,
#endif
};

/* Register gpon0.  MAC = a locally-administered FALLBACK one above eth0's
 * (02:96:07:f0:00:01).  The per-board factory MAC (base+1, mirroring stock
 * nas0_0 = ELAN_MAC_ADDR+1) is applied by userspace before the WAN comes up:
 * the 05_factory_mac uci-defaults script reads ELAN_MAC_ADDR from the stock
 * ubi_Config/config_hs.xml (read-only NAND) and netifd sets it via the
 * `device` macaddr.  Carrier tracks the data-path install. */
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

/*
 * The GPON-MAC hardware error/statistics counters.
 *
 * These are the instruments that let this ONU characterise an *unknown* OLT:
 * BIP-8 and FEC tell you the downstream link quality the far end is actually
 * delivering, the GEM/PLend/BWmap error counters say whether a frame was
 * mangled in the GTC layer rather than never sent, and ds_asmbl_drop /
 * gem_frag_drop distinguish "the OLT never sent it" from "it arrived and this
 * ONU dropped it inside the GEM stage" — which is exactly the attribution a
 * downstream-delivery fault needs, and which no software counter can provide.
 *
 * Read-only and idempotent by design; see the register block comment for why
 * there is deliberately no accumulator here.  `state=` is the runtime support
 * probe: an undecoded or powered-down block reads all-ones, and a caller must
 * treat that as "not available on this hardware", not as a zero measurement.
 */
static void cg_show_gpon_mib(struct seq_file *m, struct cortina_gpon *cg)
{
	u32 bip = cg_mac_rd(cg, CG_REG_BIP_ERR);
	u32 accum = cg_mac_rd(cg, CG_REG_BIP_ERR_ACCUM);
	u32 frames = cg_mac_rd(cg, CG_REG_BIP_ERR_FRAMES);
	u32 fec_total = cg_mac_rd(cg, CG_REG_FEC_BLK_TOTAL);
	u32 v;
	bool live = !(bip == U32_MAX && accum == U32_MAX &&
		      frames == U32_MAX && fec_total == U32_MAX);

	seq_printf(m,
		   "gpon_ds_err    = %s bip=%u bip_accum=%u bip_frames=%u gem_frag_drop=%u gem_1bit=%u gem_2bit=%u gem_uncorr=%u omci_crc=%u ds_asmbl_drop=%u (accumulating, sw-cleared)\n",
		   live ? "live" : "UNAVAILABLE (block reads all-ones)",
		   bip, accum, frames,
		   cg_mac_rd(cg, CG_REG_GEM_FRAG_DROP),
		   cg_mac_rd(cg, CG_REG_GEM_1BITERR),
		   cg_mac_rd(cg, CG_REG_GEM_2BITERR),
		   cg_mac_rd(cg, CG_REG_GEM_UNCORR),
		   cg_mac_rd(cg, CG_REG_OMCI_CRC),
		   cg_mac_rd(cg, CG_REG_DS_ASMBL_DROP));
	seq_printf(m,
		   "gpon_ds_mib    = omci_gem=%u omci_pkt=%u ds_crc=%u undersize=%u oversize=%u superframe=%u (hardware DS counts)\n",
		   cg_mac_rd(cg, CG_REG_DS_OMCI_GEM),
		   cg_mac_rd(cg, CG_REG_DS_OMCI_PKT),
		   cg_mac_rd(cg, CG_REG_DS_PKT_CRC),
		   cg_mac_rd(cg, CG_REG_DS_UNDERSIZE),
		   cg_mac_rd(cg, CG_REG_DS_OVERSIZE),
		   cg_mac_rd(cg, CG_REG_SUPERFRAME));
	seq_printf(m,
		   "gpon_us_grant  = bwmap_drop=%u bwmap_corr=%u bwmap_uncorr=%u plend_err=%u plend_biterr=%u o5=%u us_omcc=%u (us_omcc UNVALIDATED)\n",
		   cg_mac_rd(cg, CG_REG_BWMAP_DROP),
		   cg_mac_rd(cg, CG_REG_BWMAP_CORR),
		   cg_mac_rd(cg, CG_REG_BWMAP_UNCORR),
		   cg_mac_rd(cg, CG_REG_PLEND_ERR),
		   cg_mac_rd(cg, CG_REG_PLEND_BITERR),
		   cg_mac_rd(cg, CG_REG_O5),
		   cg_mac_rd(cg, CG_REG_US_OMCC_CNT));
	/*
	 * The hardware's own upstream-wedge witness.  There is currently NO
	 * witness at all for "the GPON-MAC to PUC interface hung", which is the
	 * failure this latch reports — and it names the offending T-CONT.
	 */
	v = cg_mac_rd(cg, CG_REG_PUCIF_PROTECT);
	seq_printf(m,
		   "gpon_pucif_hang= %s (raw=0x%08x, tcont=%u) (latched, not cleared by this read)\n",
		   (v & BIT(0)) ? "★ HUNG" : "no", v, (v >> 1) & 0x1f);
	seq_printf(m,
		   "gpon_fec       = ctrl=0x%08x status=0x%08x total=%u clean=%u corr=%u uncorr=%u corr_bytes=%u (clear semantics UNPROVEN)\n",
		   cg_mac_rd(cg, CG_REG_FEC_CTRL),
		   cg_mac_rd(cg, CG_REG_FEC_MISC_STATUS),
		   fec_total,
		   cg_mac_rd(cg, CG_REG_FEC_CLEAN_BLK),
		   cg_mac_rd(cg, CG_REG_FEC_CORR_BLK),
		   cg_mac_rd(cg, CG_REG_FEC_UNCORR_BLK),
		   cg_mac_rd(cg, CG_REG_FEC_CORR_BYTES));
}

static int cg_proc_show(struct seq_file *m, void *v)
{
	struct cortina_gpon *cg = m->private;
	char vendor[5], sn_str[13];
	u32 onu, alarm;

	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	alarm = cg_mac_rd(cg, CG_REG_ALARM);

	seq_printf(m, "gpon-mac @ phys 0x%llx + 0x%x\n",
		   (unsigned long long)CG_PON_WINDOW_PHYS, CG_GPON_MAC_OFF);
	seq_printf(m, "vendor-id      = 0x%08x (\"%s\")\n",
		   cg_mac_rd(cg, CG_REG_VENDOR), vendor);
	seq_printf(m, "vendor-spec    = 0x%08x\n", cg_mac_rd(cg, CG_REG_VENDOR_SPEC));
	/* The identity, and WHERE it came from: "board" is the only value that
	 * means "read from this unit"; NONE = ranging is still held off waiting
	 * for it, FALLBACK = a placeholder, not this board's serial number. */
	cg_sn_format(cg->sn, sn_str);
	seq_printf(m, "serial-number  = %s\n",
		   cg->sn_src == CG_SN_NONE ? "(not provisioned)" : sn_str);
	seq_printf(m, "sn-source      = %s%s\n", cg_sn_src_name[cg->sn_src],
		   cg->activated ? "" : " (ranging not started)");
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
	/*
	 * us_rx/enq/drop are the SHORT-WINDOW upstream-admission counters, read
	 * raw on purpose: they are only meaningful as a delta across a burst of
	 * upstream frames the reader generates itself (an idle but perfectly
	 * healthy ONU reads 0 0 0).  FORCE_DROP is a 16-bit field, so the
	 * reserved upper half must not be printed as part of the count.
	 */
	seq_printf(m, "puc            = %s  us_rx=%u enq=%u drop=%u  pucif=0x%08x\n",
		   cg->puc_ready ? "programmed" : "NOT programmed",
		   readl(cg->pon + CG_PUC_BMC_RX_PKT),
		   readl(cg->pon + CG_PUC_BMC_RX_PKT_ENQ),
		   readl(cg->pon + CG_PUC_BMC_FORCE_DROP) & CG_PUC_BMC_CNTR_MASK,
		   readl(cg->pon + CG_GPON_MAC_PUCIF_CTRL));
	/*
	 * ...and the OMCI-SPECIFIC upstream witness, CUMULATIVE: the count of
	 * upstream frames the PUC matched against the OMCI link type, summed
	 * from the clear-on-read deltas (see cg_puc_ctrl_sample).  Reading this
	 * line takes a sample itself, so a poller feeds the totals instead of
	 * stealing from them.  us_omci is the one number here that upstream
	 * user data can never inflate; pair it with omci_resp's tx= below (what
	 * the responder handed to the transmit ring) to tell "the OLT got no
	 * reply because we built none" from "...because it never left the CPU".
	 */
	cg_puc_ctrl_sample(cg);
	spin_lock(&cg->puc_cnt_lock);
	seq_printf(m,
		   "puc_ctrl       = us_omci=%u ctrl_mac=%u len_err=%u samples=%u lnk_type=0x%04x (cumulative)\n",
		   cg->puc_omci_us, cg->puc_ctrl_mac, cg->puc_len_err,
		   cg->puc_cnt_samples,
		   readl(cg->pon + CG_PUC_GLOBAL_LNK_TYPE) >> 16);
	spin_unlock(&cg->puc_cnt_lock);
	cg_show_gpon_mib(m, cg);
	seq_printf(m, "omci_resp      = %s tx=%u fail=%u ds_crc ok=%u bad=%u",
		   cg->omci_active ? "armed" : "off",
		   cg->omci_tx, cg->omci_tx_fail,
		   cg->omci_ds_crc_ok, cg->omci_ds_crc_bad);
	if (cg->omci)
		seq_printf(m, "  mds=%u store=%u avc=%u unhandled=%u dup_replay=%u ext=%u no_ack=%u",
			   cg->omci->mds, cg->omci->store_n,
			   cg->omci->avc_count, cg->omci->unhandled,
			   cg->omci->dup_replay, cg->omci->rx_extended,
			   cg->omci->no_ack);
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
	/*
	 * ★ READ BACK the two tables the upstream data path depends on, instead of trusting
	 * that the install-time writes took.  If the PADI never leaves, these are suspect #1:
	 *   us_port[idx] - every VoQ of the data T-CONT must stamp the data GEM port-id, so
	 *                  slots 8..15 must ALL read 225.  A slot reading 0 or GPON_GEM_US_
	 *                  PORT_NONE means upstream frames on that VoQ carry no GEM port and
	 *                  the OLT has nothing to match them to - they die silently at the
	 *                  far end, which looks exactly like "enqueued but no reply".
	 *   tcont[alloc] - the CAM entry the OLT's grants address.  index must be the data
	 *                  hw T-CONT, and a grant for an unbound alloc makes no burst at all.
	 * Written to be diff-able against the same rows on stock.
	 */
	{
		unsigned int i;

		seq_puts(m, "us_port_tbl   : ");
		for (i = 0; i < 16; i++) {
			u32 v = 0;

			if (cg_tbl_op(cg, CG_REG_US_PORT_ACCESS, i))
				break;
			v = readl(cg->mac + CG_REG_US_PORT_DATA) & 0xfff;
			seq_printf(m, "%u:%u ", i, v);
		}
		seq_printf(m, " (slots %u..%u must ALL read the data GEM %u)\n",
			   CG_DATA_GEM_IDX, CG_DATA_GEM_IDX + CG_PUC_QUEUE_PER_TCONT - 1,
			   cg->hw_data_gem);
		if (cg->hw_data_alloc &&
		    !cg_tbl_op(cg, CG_REG_TCONT_ACCESS, cg->hw_data_alloc & 0xfff)) {
			u32 d = readl(cg->mac + CG_REG_TCONT_DATA);

			seq_printf(m, "tcont[%u]      : 0x%08x (index=%lu omci_en=%u ploam_en=%u; want index=%u)\n",
				   cg->hw_data_alloc, d,
				   (unsigned long)FIELD_GET(CG_TCONT_INDEX_MASK, d),
				   !!(d & CG_TCONT_OMCI_EN), !!(d & CG_TCONT_PLOAM_EN),
				   CG_DATA_TCONT_IDX);
		}
	}
	cg_us_mib_show(cg, m);
	{
		unsigned int t;

		seq_puts(m, "bufocc        : ");
		for (t = 0; t < 4; t++) {
			u32 v = 0;

			if (cg_tbl_op(cg, CG_REG_DBRU_BUFOCC_ACCESS, t & 0x1f))
				break;
			v = readl(cg->mac + CG_REG_DBRU_BUFOCC_DATA) & 0xffff;
			seq_printf(m, "tcont%u=%u ", t, v);
		}
		seq_printf(m, " (LEVEL not total - sample DURING a dial.  tcont%u>0 = frames reached the PUC and await a GRANT; ==0 = they never got there)\n",
			   CG_DATA_TCONT_IDX);
	}
	seq_printf(m, "rgb8(a05c)     = 0x%08x  (DS-lock: (v&0x9c01)==0x9c00)\n", readl(cg->pon + 0xa05c));
	/* PSDS internal CMU reg 0x400 (indirect read strobe -> a090; the re-lock
	 * strobe target).  a08c shown too to disambiguate the read-data register. */
	writel(CG_PSDS_IND_READ | CG_PSDS_CMU_IDX, cg->pon + CG_PSDS_IND_CMD);
	udelay(10);
	seq_printf(m, "cmu[0x400]     = a090=0x%08x a08c=0x%08x  (re-lock strobes [7:4]; coldstart re-rolls=%u episode=%d)\n",
		   readl(cg->pon + CG_PSDS_IND_RDATA), readl(cg->pon + CG_PSDS_IND_WDATA),
		   cg->coldstart_rolls, cg->coldstart_tries);
	seq_printf(m, "gbox(a060)     = 0x%08x  (stock 0x454 rx/tx bit-order)\n", readl(cg->pon + 0xa060));
	seq_printf(m, "reg(a064)      = 0x%08x  (stock 0)\n", readl(cg->pon + 0xa064));
	seq_printf(m, "reg(a068)      = 0x%08x  (stock 1)\n", readl(cg->pon + 0xa068));
	seq_printf(m, "reg(a070)      = 0x%08x  (stock 1)\n", readl(cg->pon + 0xa070));
	seq_printf(m, "psds_init(glb) = 0x%08x  (ben_oen bit4, pow_pcix bit5)\n", readl(cg->glb + CG_GLB_PSDS_INIT));
	/*
	 * ★★★ 2026-08-11: the laser_route / gpio pin34 / gpio grp3-4 lines USED TO BE
	 * HERE, and they are why `cat /proc/gpon` PANICKED THE BOARD.  Decoded from the
	 * aotRA6/RA8 traces: pc = cg_proc_show+0xcd4, x2 = 0xffffffc0810f542c = the glb
	 * window base + 0x42c = CG_GLB_PINROUTE -> synchronous external abort (ESR
	 * 0x96000010: mapped, but nothing on the bus answers).  The glb window is 4 KiB
	 * so the address translates; that is exactly what makes a hole ABORT rather than
	 * fault cleanly.
	 *
	 * Every read in those three lines was proc-only - CG_GLB_PINROUTE, CG_GLB_GPIO_
	 * MUX0/1 and all eight CG_PERGPIO_* appear NOWHERE else in this driver, so not
	 * one of them was ever corroborated by a working access.  The rest of the glb
	 * reads in this function (PON_CNTL, PSDS_INIT, PON_INT0, NE_ICTL_*) are all
	 * <= 0x22c and are read and written on the live O5 path, so they are known good.
	 *
	 * Same rule as fix#57 (cg_pon_dump_offs) and the 2026-08-11 NI qmblock filter:
	 * DUMP ONLY WHAT THE DRIVER ALREADY TOUCHES ON A WORKING PATH.  If the BOSA/laser
	 * routing ever needs inspecting again, corroborate each offset first - the H660
	 * TxPwr-GPIO hunt is the precedent for how much these pins can matter, but that
	 * is not a licence to blind-read them from a /proc handler.
	 */
	cg_bosa_proc_show(cg->dev, m);
	/* live optical diagnostics; also refreshes the ANI-G levels the OLT reads */
	cg_optic_sample(cg, m);

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
	/*
	 * `echo "sn VVVVHHHHHHHH" > /proc/gpon`: hand the driver this board's GPON
	 * serial number.  THE shipping provisioning path -- /etc/init.d/gpon-identity
	 * reads GPON_SN out of the factory config volume and writes it here, the same
	 * way 05_factory_mac feeds the factory MAC in via uci.  Until it arrives the
	 * MAC is configured but ranging is held off, so the ONU never announces an
	 * identity that is not its own.
	 */
	if (strncmp(p, "sn ", 3) == 0) {
		int ret = cg_sn_set(cg, strim(p + 3), CG_SN_BOARD);

		return ret ? ret : len;
	}
	/* one-shot full BOSA register dump to dmesg (cold-state diffing) */
	if (strcmp(p, "bosa dump") == 0) {
		cg_bosa_dump(cg->dev);
		return len;
	}
	/* manual SerDes CMU re-lock (the cold-start recovery primitive) -- for
	 * validating it is non-destructive on a good O5 boot before relying on it */
	if (strcmp(p, "relock") == 0) {
		cg_psds_relock(cg);
		return len;
	}
	/*
	 * ★fix#101: `echo pontx > /proc/gpon` - inject ONE CPU-originated frame straight
	 * into the PON data TX path and report what happened, in one line.
	 *
	 * WHY THIS EXISTS: the open question is whether a CPU-originated upstream frame is
	 * (a) enqueued but never framed, or (b) gated before it ever reaches the framer.
	 * SEVEN attempts to answer it failed because the STIMULUS never ran - the board
	 * initramfs has no python3, no devmem, no tcpdump and no `timeout` applet, and
	 * ping/arping never resolved so the kernel never called ndo_start_xmit at all
	 * (tx_packets AND tx_dropped both flat, which is how we know).  This bypasses
	 * routing, ARP, and userspace entirely: the skb is built here and handed directly
	 * to cortina_ni_pon_data_tx(), so the transmit path is ENTERED by construction.
	 *
	 * The frame is a PPPoE-Discovery PADI (ethertype 0x8863) to broadcast, 60 bytes, so
	 * it is the same shape as the frame that actually matters.
	 *
	 * ORACLE - read all three, they are three distinguishable answers:
	 *   gpon0 tx_packets +1 and us_mib[sel=8] +1 -> CPU TX works AND reaches the fibre
	 *   gpon0 tx_packets +1 and us_mib[sel=8]  0 -> enqueued but NEVER FRAMED  <- the
	 *                                               real question, and the case
	 *                                               Addendum 3 could not separate
	 *   gpon0 tx_dropped +1                      -> cg_wan_xmit gated it (data_installed)
	 * us_mib[sel=7] is the always-on positive control - US OMCI provably transmits, so a
	 * dead MIB read is distinguishable from a dead datapath.
	 */
	if (strcmp(p, "pontx") == 0) {
		static const u8 padi[] = {
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff,	/* dst: broadcast   */
			0x02, 0x96, 0x07, 0xf0, 0x00, 0x02,	/* src: gpon0 MAC   */
			0x88, 0x63,				/* PPPoE Discovery  */
			0x11, 0x09, 0x00, 0x00, 0x00, 0x00,	/* ver/type, PADI   */
		};
		struct net_device *nd = cg->wan_ndev;
		struct sk_buff *skb;
		netdev_tx_t rc;

		if (!nd) {
			dev_err(cg->dev, "pontx: no wan netdev\n");
			return -ENODEV;
		}
		skb = netdev_alloc_skb(nd, 64);
		if (!skb)
			return -ENOMEM;
		skb_put_data(skb, padi, sizeof(padi));
		skb_put_zero(skb, 60 - sizeof(padi));	/* pad to the 60B minimum */
		skb->dev = nd;
		skb->protocol = htons(ETH_P_PPP_DISC);
		dev_info(cg->dev,
			 "pontx: injecting %u B into cortina_ni_pon_data_tx (data_installed=%d) - watch gpon0 tx_packets/tx_dropped and us_mib[sel=8]\n",
			 skb->len, cg->data_installed);
		rc = cortina_ni_pon_data_tx(skb, nd);
		dev_info(cg->dev, "pontx: xmit returned %d (0 = NETDEV_TX_OK)\n", (int)rc);
		return len;
	}
	if (strncmp(p, "mib ", 4) != 0 || kstrtou32(strim(p + 4), 16, &sel))
		return -EINVAL;

	/* Only readable from a settled O5.  This strobes the TX-PLOAM MIB engine,
	 * and doing that during activation wedges the PLOAM FSM at O1 -- a
	 * diagnostic that bricks the link it is diagnosing is worse than no
	 * diagnostic, and the operator cannot tell the wedge from a real ranging
	 * failure.  Refuse rather than "helpfully" running it anyway. */
	if (CG_ONU_STATE(cg_mac_rd(cg, CG_REG_GPON_ONU)) != CG_STATE_OPERATION)
		return -EBUSY;

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
	mutex_init(&cg->sn_lock);
	spin_lock_init(&cg->puc_cnt_lock);
	/*
	 * The event ring and the bottom half are initialised HERE, not in
	 * cg_intr_setup(): that runs only under cg_do_reset && cg_do_intr, while
	 * isr_work is queued from paths that do not depend on either - the DS
	 * OMCI RX hook (registered unconditionally below) and the post-O5
	 * supervisor tick.  Ranging is autonomous in silicon, so with intr=0 or
	 * reset=0 the ONU still reaches O5 and those paths would queue a
	 * work_struct that devm_kzalloc left all-zero (->func == NULL).
	 */
	spin_lock_init(&cg->evt_lock);
	INIT_WORK(&cg->isr_work, cg_isr_work);
	INIT_DELAYED_WORK(&cg->veip_avc_work, cg_veip_avc_work);
	INIT_WORK(&cg->omci_ping_work, cg_omci_ping_work_fn);
	INIT_DELAYED_WORK(&cg->stats_work, cg_stats_work);
	/*
	 * ★★★ 2026-08-08 fix#43: arm the counter dump HERE, at probe, not only when the
	 * OMCI responder arms.  THE MEASUREMENT BUG THIS FIXES: `hw_pkt` freezes at 15
	 * because the OLT sends ONE short DS OMCI burst as we enter O5 and then gives up
	 * (we never answer).  The responder arms *after* O5, so the old arming point put
	 * the first sample AFTER the burst had already happened - every counter reading
	 * we have ever taken was of an idle line, and "counter X did not move" said
	 * nothing about the frames we care about.  Starting at probe gives a
	 * before/after pair spanning the burst.
	 */
	if (cg_stats_s > 0)
		schedule_delayed_work(&cg->stats_work, cg_stats_s * HZ);
	INIT_DELAYED_WORK(&cg->coldstart_work, cg_coldstart_work);
	INIT_DELAYED_WORK(&cg->sn_wait_work, cg_sn_wait_work);
	INIT_DELAYED_WORK(&cg->puc_cnt_work, cg_puc_cnt_work);
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

			/* fix#110 diag: dump the whole glb reset/clock window so we can
			 * diff physical-offset-for-offset against live stock and find the
			 * PUC QM sub-block clock/reset ungate the port is missing. */
			if (cg_glbdump) {
				int gi;

				for (gi = 0; gi < 0x300; gi += 32)
					dev_info(dev, "GLBDUMP 0x%03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
						 gi,
						 readl(cg->glb + gi + 0), readl(cg->glb + gi + 4),
						 readl(cg->glb + gi + 8), readl(cg->glb + gi + 12),
						 readl(cg->glb + gi + 16), readl(cg->glb + gi + 20),
						 readl(cg->glb + gi + 24), readl(cg->glb + gi + 28));
			}

			/*
			 * ★ fix#110: the PUC QM sub-block (venus_pon 0x8238+) reads
			 * bus-error (0x96000010) on our port = a dead clock/reset domain.
			 * The glb reset/clock region differs from live stock at a cluster
			 * of regs our bring-up leaves at bootloader default (captured in
			 * stock_glbcap.log vs port_glbcap.out): 0x088/08c/0a0/0cc/10c/168/
			 * 184/194 stock-set-but-port-zero, and 0x0a8 where the port leaves
			 * 0x198 asserted while stock clears it.  Match stock (safe: stock
			 * serves fine with these), then read-test QM 0x8274 as the oracle.
			 */
			if (qm_glbfix) {
				if (qm_glbfix == 5) {
					/* fix#111b: apply Track A's "pulse reset -> HW self-init"
					 * insight to the PUC's OWN reset.  PUC_soft_reset (0x81c0,
					 * bit0) is in the readable range and the port never touches
					 * it.  U-Boot may leave the PUC QM sub-block (0x8238+)
					 * un-self-inited; pulsing PUC_soft_reset should re-run its
					 * self-init and bring the QM APB slave alive. */
					writel(0x1, cg->pon + 0x81c0);	/* assert */
					mdelay(10);
					writel(0x0, cg->pon + 0x81c0);	/* deassert -> self-init */
					mdelay(50);
					dev_info(dev, "fix#111b: pulsed PUC_soft_reset(0x81c0); reading QM 0x8274...\n");
				} else if (qm_glbfix == 4) {
					/* ★★★★★ fix#111 (Track A hint): the NE datapath
					 * sub-blocks (incl TQM bit5) are left UN-SELF-INITED by
					 * U-Boot.  Track A's aal_glb_ni_ne_rst PULSES their reset
					 * in GLOBAL_BLOCK_RESET (cg->glb+0x098 on us: reads
					 * u-boot 0xD03021C0) so each re-runs HW self-init.  A
					 * static value (modes 1/3) can't do it - it's the
					 * assert->deassert EDGE.  TQM self-init is what the PUC QM
					 * report sub-block (venus_pon 0x8238+) needs to respond. */
					u32 br = readl(cg->glb + 0x098);
					u32 m = (1u<<0)|(1u<<1)|(1u<<2)|(1u<<3)|(1u<<5)|
						(1u<<16)|(1u<<17)|(1u<<27);

					writel(br | m, cg->glb + 0x098);	/* assert */
					mdelay(10);
					writel(br, cg->glb + 0x098);		/* deassert -> self-init */
					mdelay(50);
					dev_info(dev, "fix#111: pulsed GLOBAL_BLOCK_RESET(0x098) 0x%08x -> 0x%08x (datapath self-init)\n",
						 br, readl(cg->glb + 0x098));
				} else if (qm_glbfix == 3) {
					/* GLOBAL_BLOCK_RESET (0x098): the port asserts resets
					 * on bits 6,7,13,20,30 that live stock RELEASES.  Clear
					 * exactly those (release-only, safe) - one may gate the
					 * QM sub-block clock.  Do NOT set stock's pe1 reset. */
					u32 br = readl(cg->glb + 0x098);

					writel(br & ~0x401020c0u, cg->glb + 0x098);
				} else if (qm_glbfix == 2) {
					writel(0x00000000, cg->glb + 0x0a8);
				} else {
					writel(0x00000002, cg->glb + 0x088);
					writel(0x00000301, cg->glb + 0x08c);
					writel(0x0000002c, cg->glb + 0x0a0);
					writel(0x00000000, cg->glb + 0x0a8);
					writel(0x0000006b, cg->glb + 0x0cc);
					writel(0x000000fc, cg->glb + 0x10c);
					writel(0x00000021, cg->glb + 0x168);
					writel(0x00000009, cg->glb + 0x184);
					writel(0x00000003, cg->glb + 0x194);
				}
				dev_info(dev, "fix#110: glb clock-region matched to stock (mode %u); reading QM 0x8274...\n",
					 qm_glbfix);
				/* oracle: sync-aborts if the QM domain is still dead */
				dev_info(dev, "fix#110: QM PLEN_MEM_CTL(0x8274) = 0x%08x -- DOMAIN ALIVE\n",
					 readl(cg->pon + 0x8274));
			}

			/* arm the post-O5 servicing BEFORE ranging starts so
			 * the ONU_ID/PORTID/ONU_ST_CHG events of the very
			 * first O1->O5 pass are serviced live */
			if (cg_do_intr)
				cg_intr_setup(cg, pdev);

			if (cg_activate) {
				/*
				 * The identity gate.  Ranging announces the ONU's
				 * serial number, so it may only start once we know
				 * THIS board's -- which lives in the factory config
				 * volume and is pushed in from userspace (see the
				 * cg_sn_* block).  A bad/absent module-param serial
				 * number defers to that path, bounded by
				 * cg_sn_wait_work so the PON side is never left dark.
				 */
				if (!cg_sn_param ||
				    cg_sn_set(cg, cg_sn_param, CG_SN_PARAM)) {
					dev_warn(dev, "GPON serial number not known yet - MAC configured, ranging DEFERRED up to %ds for /etc/init.d/gpon-identity (echo \"sn <VVVVHHHHHHHH>\" > /proc/gpon)\n",
						 CG_SN_WAIT_SECS);
					schedule_delayed_work(&cg->sn_wait_work,
							      CG_SN_WAIT_SECS * HZ);
				}
			}
			if (cg->activated) {
				int i;

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
	/* Not a correctness check: before the identity is provisioned this reads
	 * the reset value.  cg_activate_start() verifies the vendor-id readback
	 * against what it programmed, which works on any board. */
	dev_info(dev, "GPON MAC vendor-id \"%s\" onu=0x%08x alarm=0x%08x\n",
		 vendor, onu, cg_mac_rd(cg, CG_REG_ALARM));

	cg_singleton = cg;
	/* Stage B: receive the DS OMCI PDUs the NI CPU-RX path classifies out
	 * (ethertype 0xfff1).  Registered after cg_singleton so the handler
	 * never sees a half-initialized context. */
	if (IS_REACHABLE(CONFIG_CORTINA_NI))
		cortina_ni_pon_rx_hook_set(cg_rx_omci);
	cg_wan_create(cg);	/* Stage D: the gpon0 WAN netdev */
	cg->proc = proc_create_data("gpon", 0644, NULL, &cg_proc_ops, cg);
	/* Stage E (M2): stand up the userspace stock-daemon bridge (idle until
	 * armed with cortina_gpon.omci_userspace=1). */
	cg_omci_bridge_init(cg);
	platform_set_drvdata(pdev, cg);
	dev_info(dev, "cortina-gpon phase-0 probe complete (/proc/gpon)\n");
	return 0;
}

static void cortina_gpon_remove(struct platform_device *pdev)
{
	struct cortina_gpon *cg = platform_get_drvdata(pdev);

	/* Unhook the DS OMCI RX path FIRST so cg_rx_omci() (and its bridge
	 * divert) can no longer run, THEN tear the bridge down -- otherwise a DS
	 * OMCI frame in the NI softirq could touch an already-released netlink
	 * socket.  cg_omci_bridge_exit() additionally drains in-flight callers. */
	if (IS_REACHABLE(CONFIG_CORTINA_NI)) {
		cortina_ni_pon_wan_ndev_set(NULL);
		cortina_ni_pon_rx_hook_set(NULL);
	}
	cg_omci_bridge_exit();
	cancel_delayed_work_sync(&cg->veip_avc_work);
	cancel_delayed_work_sync(&cg->coldstart_work);
	cancel_delayed_work_sync(&cg->sn_wait_work);
	cancel_delayed_work_sync(&cg->puc_cnt_work);
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
