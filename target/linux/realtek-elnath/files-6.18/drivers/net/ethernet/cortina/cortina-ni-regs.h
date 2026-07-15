/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cortina-Access NI (network engine) register map for the Realtek
 * RTL9607F "Elnath" GPON SoC.
 *
 * Register locations, bit layouts and access sequences below are hardware
 * facts recovered from the shipped RTL9607F firmware (ca-ne.ko disassembly:
 * ca_ne_reg_init, aal_mdio_read_direct, aal_mdio_write_direct,
 * aal_mdio_speed_set, aal_mdio_global_init, aal_mdio_read/write) and
 * cross-checked against the CA8277B public register bit-field definitions
 * (PER_MDIO_* block: CFG/ADDR/WRDATA/RDDATA/CTRL).  Both sources agree on
 * every bit field; only the block's absolute base differs between chip
 * generations, which is why everything here is relative to the DT window.
 */

#ifndef _CORTINA_NI_REGS_H
#define _CORTINA_NI_REGS_H

#include <linux/bits.h>

/*
 * DT "reg" windows of the ethernet@f4300000 node, by POSITIONAL index.
 * The stock probe (ca_ne_reg_init) maps them with of_address_to_resource()
 * in exactly this index order: 0,1,2,3,4,5,6,7,9,8, <hardcoded peri map>,
 * 10,11,13,15,16,18,19,20,21.  Indices 12/14/17 are null placeholders that
 * only keep the positions of the following entries.
 *
 * Role names follow the per-window aal_*_reg_base_init() callee each mapped
 * base is handed to in ca_ne_reg_init.
 */
enum cortina_ni_win {
	CA_NI_WIN_NI		= 0,	/* 0x4_f4300000 64K  NI/L2/L3/QM core */
	CA_NI_WIN_NE_INTR	= 1,	/* 0x4_f4320194 16   NE global intr   */
	CA_NI_WIN_MDIO		= 2,	/* 0x4_f4329128 0x48 MDIO master      */
	CA_NI_WIN_XRAM		= 3,	/* 0x0_f4500000 8K   XRAM scratch     */
	CA_NI_WIN_DMA		= 4,	/* 0x4_f7001000 4K   DMA/LDMA TX ring */
	CA_NI_WIN_SRAM		= 5,	/* 0x0_80000000 176K descriptor SRAM  */
	CA_NI_WIN_DDR_COHERENT	= 6,	/* reserved DDR, coherent buffer pool */
	CA_NI_WIN_DDR_PKT	= 7,	/* reserved DDR, noncache packet pool */
	CA_NI_WIN_PON		= 8,	/* 0x4_f5500000 48K  PON regs         */
	CA_NI_WIN_GLB		= 9,	/* 0x4_f4320000 4K   GLB chip config  */
	CA_NI_WIN_AXI_REO	= 10,	/* 0x4_f432d000      AXI reorder      */
	CA_NI_WIN_GPHY		= 11,	/* 0x4_f4600000 1M   quad-GPHY+SerDes */
	/* 12 = null placeholder */
	CA_NI_WIN_SGMII_PCS	= 13,	/* 0x4_f4330000 64K  SGMII PCS LP     */
	/* 14 = null placeholder */
	CA_NI_WIN_GPHY_WRAP	= 15,	/* 0x4_f4700000 256  GPHY wrapper     */
	CA_NI_WIN_LDMA_AUX	= 16,	/* 0x0_90301000 8K   LDMA aux         */
	/* 17 = null placeholder */
	CA_NI_WIN_FBM_GLB	= 18,	/* 0x0_90300800      FBM global       */
	CA_NI_WIN_FBM_AXI	= 19,	/* 0x0_90300900      FBM AXI          */
	CA_NI_WIN_FBM_CPU	= 20,	/* 0x0_90300a00      FBM CPU          */
	CA_NI_WIN_FBM_POOL	= 21,	/* 0x0_90300b00      FBM pool         */
	CA_NI_WIN_COUNT		= 22,
};

/*
 * Extra block NOT in the DT: stock ca_ne_reg_init unconditionally
 * __ioremap()s 4K at 0xf4329000 (peripheral/MDIO/SDS area).  Note stock
 * itself uses the 32-bit (low-alias) address here, high cell 0.
 */
#define CA_NI_PERI_PHYS			0xf4329000
#define CA_NI_PERI_SIZE			0x1000

/* GLB window (idx 9): +0x00 is a read-only chip/JTAG ID - probe sanity read */
#define CA_NI_GLB_JTAG_ID		0x00

/*
 * ★ GLOBAL_DPHY_RESET (GLB +0xa0, phys 0xf43200a0; the "cortina,rst-mgr"
 * dphy reset).  Each bit HELDS a digital-PHY / datapath sub-block in reset
 * (1 = asserted).  The internal quad-GPHY <-> NI-MAC datapath sub-blocks are
 * among them.  U-Boot parks several in reset at handoff (live: 0x50302340 -
 * bits 30/21/20/13/9/8/6 asserted); stock's ca_ne RELEASES them, leaving only
 * bit28 (a block we don't use) -> live golden 0x10000000.  Our driver never
 * touched this reg, so the internal GMII sub-blocks stayed held in reset and
 * NO frame crossed GPHY<->MAC on any of the 4 ports (bidirectional dead while
 * the GPHY LINE side still links).  Must be deasserted EARLY (before the GPHY/
 * MAC bring-up) - a late release does not re-init the sub-block.
 */
#define CA_NI_GLB_DPHY_RESET		0xa0
#define CA_NI_GLB_DPHY_RESET_VAL	0x10000000u	/* stock golden (all NE sub-blocks released) */

/*
 * NE block-reset controller (GLB +0x28, phys 0xf4320028; "cortina,rst-mgr"
 * block_rst - per our board's DTS ca8277-soc_fpga.dtsi, whose DDR-reserved
 * bases 0x0bc48000/0x09000000 match this board's boot log).  Each bit HOLDS
 * one NE sub-block in reset (1 = asserted); stock ca_ni_global_reset does a
 * plain RMW set (assert) -> 1ms -> clear (deassert), NOT self-clearing, NOT
 * SMC-gated (ca8277 has no CONFIG_ARCH_CORTINA_SMC; plain MMIO works, same as
 * dphy_rst at GLB+0xa0).  Bit IDs (dt-bindings cortina,rst-mgr.h): NI=0,
 * L2FE=1, L2TM=2, L3FE=3, SDRAM=4, TQM=5.  U-Boot leaves the L3QM un-inited so
 * QM_PHY_PORT_STS.qm_init_done stays 0 and the empty-buffer pools never
 * activate.  (The earlier 0x98 offset was from the wrong DTS variant
 * rtl8277C-soc_asic.dtsi - it pointed at an unmapped/RO word, so every asserted
 * bit read back 0.)  Never pulse SDRAM: that would reset DRAM.
 */
#define CA_NI_GLB_BLOCK_RST		0x28
#define  CA_NI_GLB_RST_NI		BIT(0)	/* NI_RESET  (U-Boot TX path) */
#define  CA_NI_GLB_RST_L2FE		BIT(1)	/* L2FE_RESET */
#define  CA_NI_GLB_RST_L2TM		BIT(2)	/* L2TM_RESET */
#define  CA_NI_GLB_RST_L3FE		BIT(3)	/* L3FE_RESET */
#define  CA_NI_GLB_RST_TQM		BIT(5)	/* TQM_RESET = the L3QM block */
/* ★ Candidate real block-reset registers to disambiguate.  Prior session:
 * reset@0x28 (ca8277-soc_fpga.dtsi) and 0x98 read 0 = "unmapped".  RE of
 * rtl8277C-soc_asic.dtsi says 0x28 = GLOBAL_BIST_CONTROL4 and the REAL
 * GLOBAL_BLOCK_RESET = 0x98 (dft 0xd03021c0), _EXT = 0x9c, GLOBAL_FABRIC_RESET
 * = 0xa4 (has capsram/global_pe bits - a possible MCE gate).  The gate: /proc
 * diagnostic dumps ALL of these via the GLB window (correct 0x4_ addressing) so
 * the DATA settles which is the real reset (structured value vs 0/BIST); 0xa0
 * dphy_rst is the known-mapped reference (reads ~0x10000000). */
#define CA_NI_GLB_BLOCK_RESET_98	0x98
#define CA_NI_GLB_BLOCK_RESET_EXT	0x9c
#define CA_NI_GLB_FABRIC_RESET		0xa4
/* ★ GLOBAL_FABRIC_RESET bit5 = the NI-MCE (multicast-expansion) sub-block reset.
 * "rsrvd1" in the rtl8277c header, but the ours-vs-stock devmem differential
 * proves it: ours 0xa4=0x00079F20 (bit5 SET), stock=0x00079F00 (bit5 CLEAR).
 * With bit5 set the NI-MCE RAM (0xaa6x) is held in reset -> the plain mce_indx
 * DATA write async-SErrors.  DEASSERT-ONLY (clear, no pulse) to match stock. */
#define  CA_NI_GLB_FABRIC_RST_MCE	BIT(5)
/* cortina_ni_qm_reset pulses NI+L2FE+L2TM+L3FE+TQM sequentially (full stock
 * ca_ni_global_reset order, SDRAM=bit4 skipped).  The QM's init-done depends on
 * the NI<->QM handshake, so NI must be reset too; cortina_ni_tx_hw_init (run in
 * tx_probe, right after this) rebuilds the FE-bypass TX path the NI reset
 * drops. */

/*
 * ★ CapSRAM TrustZone segment-secure control (its OWN block, needs its own
 * ioremap - NOT inside the GLB 4KB map).  ★ 40-bit phys 0x4_f4322000: the SoC
 * peripheral region carries the 0x4_ high bits (all NE windows are 0x4_f43xxxxx,
 * e.g. GLB=0x4_f4320000); the register header's 0xf4322000 is the low-32 alias.
 * An earlier test used a BARE 0xf4322000 (missing 0x4_) -> read a wrong/aliased
 * word, so CapSRAM was NOT cleanly disproven; this is the corrected test.
 * The CapSRAM backs the NI MCE index table (0xaa6x); SEGSECURE bits[17:0] (18
 * segments): 1=secure, 0=NS.  BL1 may leave segments SECURE; stock U-Boot clears
 * TZCONTROL=0 before ca_init so the NS kernel can reach the table.  A NS access
 * to a secured segment = async SError (the MCE DATA write).
 */
#define CA_NI_CAPSRAM_PHYS		0x4f4322000ULL	/* 40-bit: 0x4_f4322000 */
#define CA_NI_CAPSRAM_TZCONTROL		0x00	/* SEGSECURE[17:0]; =0 -> all NS */
#define CA_NI_CAPSRAM_INIT		0x04	/* bit0 INIT (SRAM mem-init)      */
#define CA_NI_CAPSRAM_INITRANGE		0x08	/* dft 0x11ff0000                 */

/*
 * MDIO master, relative to the DT window idx 2 base (stock: window base is
 * handed to aal_per_mdio_reg_base_init and used as-is; the direct-access
 * functions address CFG/ADDR/WRDATA/RDDATA/CTRL at +0/+4/+8/+0xc/+0x10).
 *
 * This master drives the external MDIO pins (clause 22 "direct" and
 * clause 45 "indirect").  The four INTERNAL GbE PHYs are NOT behind it -
 * see the GPHY window defines below.
 */
#define CA_NI_MDIO_CFG			0x00
#define  CA_NI_MDIO_CFG_PRESCALE	GENMASK(31, 16)	/* MDC divider */
#define  CA_NI_MDIO_CFG_MANUAL		BIT(0)		/* 1 = manual mode */
/* prescale = 125000 kHz / (2 * mdc_speed_khz)   (from aal_mdio_speed_set) */
#define  CA_NI_MDIO_REFCLK_KHZ		125000

#define CA_NI_MDIO_ADDR			0x04
#define  CA_NI_MDIO_ADDR_PHY		GENMASK(4, 0)	/* PHY address     */
#define  CA_NI_MDIO_ADDR_REG		GENMASK(12, 8)	/* register offset */
#define  CA_NI_MDIO_ADDR_RD_WR		BIT(15)		/* 1 = read        */
#define  CA_NI_MDIO_ADDR_ST		BIT(16)		/* 0=direct 1=indir*/
#define  CA_NI_MDIO_ADDR_OP		GENMASK(19, 18)
#define   CA_NI_MDIO_OP_ADDR_WR		0	/* c45 address write  */
#define   CA_NI_MDIO_OP_WR		1	/* write              */
#define   CA_NI_MDIO_OP_RD_INC		2	/* read + post-incr   */
#define   CA_NI_MDIO_OP_RD		3	/* read               */

#define CA_NI_MDIO_WRDATA		0x08	/* [15:0] write data  */
#define CA_NI_MDIO_RDDATA		0x0c	/* [15:0] read data   */

#define CA_NI_MDIO_CTRL			0x10
#define  CA_NI_MDIO_CTRL_START		BIT(7)	/* kick the frame     */
#define  CA_NI_MDIO_CTRL_DONE		BIT(0)	/* done, write-1-clear*/

/* stock polls done up to 50000 x udelay(1) */
#define CA_NI_MDIO_TIMEOUT_US		50000

/*
 * Internal quad GbE PHYs (MDIO addresses 1..4).  Stock aal_mdio_read/write
 * never touch the MDIO master for these: their registers are memory-mapped
 * in the GPHY window (idx 11), one 256K (1 << 18) bank per PHY:
 *
 *   plain MII reg r        : bank + 0x29000 + 8 * r
 *   paged regs 0x10..0x17  : bank + 0x40 * page + 8 * (r - 0x10)
 *   reg 0x1f (page select) : software shadow only (no HW register); stock
 *                            keeps it per-PHY and delays 1 ms after every
 *                            internal-PHY write, including the page write.
 */
#define CA_NI_GPHY_FIRST		1
#define CA_NI_GPHY_COUNT		4
#define CA_NI_GPHY_BANK_STRIDE		0x40000
#define CA_NI_GPHY_MII_BASE		0x29000
#define CA_NI_GPHY_REG_STRIDE		8
#define CA_NI_GPHY_PAGE_STRIDE		0x40
#define CA_NI_GPHY_PAGED_FIRST		0x10
#define CA_NI_GPHY_PAGED_LAST		0x17

/* bank base of the GPHY wired to LAN port p (PHY MDIO address p + 1) */
#define CA_NI_GPHY_BANK(p)		((p) * CA_NI_GPHY_BANK_STRIDE)
/* memory-mapped extension-page register (pages way beyond the 0x17 shadow
 * scheme are always accessed by direct offset, both here and in stock) */
#define CA_NI_GPHY_EXT(page, reg)	((page) * CA_NI_GPHY_PAGE_STRIDE + \
					 ((reg) - CA_NI_GPHY_PAGED_FIRST) * \
					 CA_NI_GPHY_REG_STRIDE)

/*
 * Internal-GPHY fault latch + port reinit (stock aal_internal_phy_recovery /
 * aal_internal_phy_reinit; RTL9607F ko @0x180d0 adds a wrapper toggle over
 * the SDK C version).  The quad-GPHY can wedge nondeterministically -
 * typically across a link renegotiation: link reads up and every MAC/QM
 * register is normal, but the port never delivers a single ingress frame.
 * Stock polls the latch at 1 Hz on every port ("Incase any of any problem,
 * recover check first") and on a nonzero read re-initializes the port:
 * INTF_RST pulse -> patch re-enable -> wrapper toggle -> INTF_RST pulse ->
 * 200 ms -> calibration reload -> power-up.  All offsets verified in the
 * shipped ko: 0x2e418 / 0x31060 / 0x291a0 / 0x29000 (bank 0).
 */
#define CA_NI_GPHY_FAULT		CA_NI_GPHY_EXT(0xb90, 19)
#define CA_NI_GPHY_PATCH_EN		CA_NI_GPHY_EXT(0xc41, 20)
#define  CA_NI_GPHY_PATCH_EN_BIT	BIT(0)
#define CA_NI_GPHY_HOLD			CA_NI_GPHY_EXT(0xa46, 20)
#define  CA_NI_GPHY_HOLD_BIT		BIT(3)
#define CA_NI_GPHY_BMCR			CA_NI_GPHY_MII_BASE	/* MII reg 0 */
#define  CA_NI_GPHY_BMCR_PDOWN		BIT(11)
/* analog calibration registers reloaded by stock phy_cal_reload after the
 * reinit: rc_cal 0xbcd.22/23, r_cal 0xbcf.18-21, amp_cal 0xbca.22.  We
 * snapshot them at probe (U-Boot-proven-working state) and restore them.
 * The companion DSP-SRAM mirror (MII 27/28, addrs 0x824a/0x824c/0x83a9/
 * 0x83ab) is uC-patch-layout-specific and deliberately NOT rewritten. */
#define CA_NI_RX_GPHY_CAL_REGS		7

/*
 * ---------------------------------------------------------------------------
 * Internal-GPHY firmware (SRAM) patch: the apro_gen2 uC firmware body.
 *
 * The quad-GPHY embeds a microcontroller whose DSP scratch-SRAM (word range
 * 0x8000..0x83f1) must be seeded with the channel-estimation / green-table /
 * RTCT / AFE firmware image before the PHY forwards a single frame across its
 * internal (system-side) GMII to the NI MAC.  Until it is applied the LINE side
 * still auto-negotiates and reports link, but NO frame crosses GPHY<->MAC on
 * any port - the same bidirectional-dead symptom as an un-released dphy reset,
 * one layer up.  The RTL9607F internal quad-GPHY (PHYID 0x001ccac5) is the
 * "apro_gen2" IP and its firmware body is NOT in open source (closed
 * librtk.a/aal blob) - the RTL9607F switch DAL carries no phyPatch, and the
 * RTL9607C sibling's in-source NCTL tables are the rev-B path, not gen2.  Our
 * image is therefore LIVE-DUMPED from stock (per bank, uC held, via OCP
 * 0xa436=addr/read 0xa438=data) - facts read out of the shipped product, not
 * SDK source.
 *
 * All registers are internal-PHY OCP addresses, memory-mapped in the GPHY
 * window: OCP register A of bank b lives at b*0x40000 + A*4 (verified against
 * the plain-MII defines above: BMCR OCP 0xa400 -> +0x29000, HOLD OCP 0xa468 ->
 * +0x291a0, fault OCP 0xb906 -> +0x2e418).
 * ---------------------------------------------------------------------------
 */
#define CA_NI_GPHY_OCP(ocp)		((ocp) * 4)	/* OCP addr -> byte offset */

/* SRAM word access: write the target word address to SRAM_ADDR, then the
 * 16-bit word to SRAM_DATA (reading SRAM_DATA returns the addressed word).
 * These are the same registers as plain-MII page 0xa43 regs 27/28.
 * SRAM_ADDR OCP 0xa436 -> mem +0x290d8, SRAM_DATA OCP 0xa438 -> mem +0x290e0. */
#define CA_NI_GPHY_SRAM_ADDR		0xa436
#define CA_NI_GPHY_SRAM_DATA		0xa438

/*
 * uC patch-lock handshake (RTL9607C Lan_eth_change_default_apro_gen2 +
 * lan_eth_check_pcs_state, tier-3 source; register offsets confirmed against
 * live-stock devmem).  The uC SRAM is only writable while the uC is HELD:
 *   GATE  page 0xa42 reg16 = OCP 0xa420 (mem +0x29080), field [2:0]: value 2
 *         (EXT_INI) = uC not lockable yet -> skip this bank, retry next link-up.
 *   LOCK  page 0xa46 reg21 = OCP 0xa46a (mem +0x291a8), bit1: set = hold uC,
 *         clear = resume uC (runs the freshly-written firmware).
 *   READY page 0xa60 reg16 = OCP 0xa600 (mem +0x29800), field [7:0]: poll until
 *         == 1 (pcs locked) before writing SRAM, bounded 0xff x 100us.
 * (These are byte offsets via CA_NI_GPHY_EXT, used directly like CA_NI_GPHY_HOLD.
 * The earlier single-register 0x291a0-bit1 resume used the wrong ready register
 * - it polled 0xa620 not 0xa600 - giving a "works some boots, dead others" gate;
 * this proper gate/lock/ready handshake replaces it.) */
#define CA_NI_GPHY_LOCK_GATE		CA_NI_GPHY_EXT(0xa42, 16)
#define  CA_NI_GPHY_LOCK_GATE_MASK	GENMASK(2, 0)
#define  CA_NI_GPHY_LOCK_GATE_EXT_INI	0x2	/* skip lock when field == this */
#define CA_NI_GPHY_LOCK			CA_NI_GPHY_EXT(0xa46, 21)
#define  CA_NI_GPHY_LOCK_HOLD		BIT(1)	/* 1 = hold uC, 0 = resume */
#define CA_NI_GPHY_LOCK_READY		CA_NI_GPHY_EXT(0xa60, 16)
#define  CA_NI_GPHY_LOCK_READY_MASK	GENMASK(7, 0)
#define  CA_NI_GPHY_LOCK_READY_VAL	0x1
#define  CA_NI_GPHY_LOCK_READY_TRIES	0xff	/* 0xff x 100us bound */

/*
 * GPHY wrapper (idx 15).  EN0 = WRAP_GPHY_CTRL12, EN1 = WRAP_GPHY_CTRL13:
 *   EN0[31:24] cfg_ocp_timeout = 0xFF  (aal_mdio_global_init)
 *   EN1[0]     mdio_ocp_sel    = 1     (aal_mdio_global_init)
 *   EN1[12]    patch_phy_done  = 1     (aal_internal_phy_init, 07f ko @0x3b0c:
 *              `orr w0,#0x1000` -> str wrap+0x34, the LAST step of integrated-
 *              PHY bring-up).  This is the GPHY->port-MAC datapath RELEASE:
 *              until it is set the PHY line side links + auto-negotiates
 *              (BMSR/aneg OK, gphy_fault=0) but NO ingress frame reaches the
 *              port MAC (mac_rx_p0 = 0).  ★ THE RX determinism gate: our old
 *              init only set EN1[0], so EN1 = 0x1 or 0x1001 depending purely on
 *              U-Boot leftover -> "works some boots, dead others".  Stock live
 *              golden EN0 = 0xFF000000, EN1 = 0x00001001; we now force both.
 * The per-port EN1[4+p] bit is a transient the ko's fault-reinit pulses; stock
 * steady state leaves it CLEAR (golden 0x1001), so establishing 0x1001 clears
 * any stray toggle.
 */
#define CA_NI_GPHY_WRAP_EN0		0x30
#define  CA_NI_GPHY_WRAP_EN0_BITS	GENMASK(31, 24)
#define  CA_NI_GPHY_WRAP_EN0_VAL	0xff000000u	/* stock golden */
#define CA_NI_GPHY_WRAP_EN1		0x34
#define  CA_NI_GPHY_WRAP_EN1_MDIO_OCP	BIT(0)		/* mdio_ocp_sel  */
#define  CA_NI_GPHY_WRAP_EN1_PATCH_DONE	BIT(12)		/* GPHY->MAC datapath */
#define  CA_NI_GPHY_WRAP_EN1_BITS \
	(CA_NI_GPHY_WRAP_EN1_MDIO_OCP | CA_NI_GPHY_WRAP_EN1_PATCH_DONE)
#define  CA_NI_GPHY_WRAP_EN1_VAL	CA_NI_GPHY_WRAP_EN1_BITS	/* 0x1001 */
/* per-GPHY interface enable in EN1; the ko's phy reinit clears then sets it */
#define  CA_NI_GPHY_WRAP_EN1_IF(p)	BIT(4 + (p))
/* other wrapper regs (analog/datapath, U-Boot-inherited) - dumped for verify */
#define CA_NI_GPHY_WRAP_R00		0x00
#define CA_NI_GPHY_WRAP_R04		0x04
#define CA_NI_GPHY_WRAP_R08		0x08
#define CA_NI_GPHY_WRAP_R0C		0x0c

/* Internal GbE PHY identifier (PHYSID1:PHYSID2), low nibble = revision */
#define CA_NI_GPHY_PHY_ID		0x001ccac5
#define CA_NI_GPHY_PHY_ID_MASK		0xfffffff0

/*
 * ---------------------------------------------------------------------------
 * M2b TX datapath registers.
 *
 * Two register files are involved:
 *
 *  1. The NI core window (DT idx 0).  The RTL9607F NI_HV block was
 *     re-generated vs. the public CA8277B map: identical bit layouts but the
 *     block moved from +0x4xxx to +0xaxxx and the per-port stride shrank
 *     from 0xa0 to 0x90.  All offsets below were re-verified per-function in
 *     the shipped RTL9607F firmware (ca-ne.ko disassembly: aal_ni_reset_intf,
 *     aal_ni_eth_port_mac_set, aal_ni_mac_autosync_cfg_set).  The L2TM block
 *     (+0x2xxx) is unchanged from CA8277B (verified: aal_l2_tm_es_ena_set
 *     uses +0x2300, aal_l2_qm_eq_cfg_set +0x2208, aal_l2_qm_buff_cfg_set
 *     +0x2210).
 *
 *  2. The DMA-LSO window (DT idx 4, 4K).  This block matches the public
 *     CA8277B register file; everything here is window-relative (the stock
 *     accessor is base + (phys - 0xf7001000)).
 * ---------------------------------------------------------------------------
 */

/* --- NI core window (idx 0): NI_HV global block, RTL9607F layout --- */
#define CA_NI_HV_INIT_DONE		0xa004
#define  CA_NI_HV_INIT_DONE_NI		BIT(1)	/* ni_init_done */
#define CA_NI_HV_INTF_RST		0xa008	/* write 0 = deassert all */
/* per-GPHY-port interface reset bit (stock NI_HV_GLB_INTF_RST_CONFIG use in
 * aal_internal_phy_reinit: set -> 1 ms -> clear, twice) */
#define  CA_NI_HV_INTF_RST_GPHY(p)	BIT(6 + (p))
#define CA_NI_HV_MAC_AUTOSYNC		0xa010
#define  CA_NI_HV_AUTOSYNC_STS(p)	BIT(p)		/* link/speed/duplex */
#define  CA_NI_HV_AUTOSYNC_FC(p)	BIT(8 + (p))	/* flow control     */
/* stock golden 0xa010 = 0x0000000f: STS-sync enabled for ALL four ports,
 * NO flow-control sync.  Ours was 0x101 (port0 STS + port0 FC). */
#define  CA_NI_HV_AUTOSYNC_STS_ALL	GENMASK(3, 0)
#define  CA_NI_HV_AUTOSYNC_FC_ALL	GENMASK(11, 8)
/* 0xa1b8: unnamed global cfg; stock aal_ni_init sets field [19:14] := 0x21
 * (bfi #14,#6 @0xc5ac).  NOT the packet-length register - that is 0xa034. */
#define CA_NI_HV_CFG_A1B8		0xa1b8
#define  CA_NI_HV_CFG_A1B8_FIELD	GENMASK(19, 14)
#define  CA_NI_HV_CFG_A1B8_VAL		0x21

/* NI_HV_GLB_INTERNAL_PORT_ID_CFG (0xa1bc) = the NI-RX -> L3QM routing register.
 * bit20 l3qmrx_to_lan = present port-0 NI-RX to the L3QM as LAN traffic; stock
 * aal_l3qm_init_empty_buffer sets it =1 (part of the NI->QM handoff).  The old
 * chipdef mislabeled this "NIRX_MISC_CFG" with bit20 = P0_WAN (and CLEARED it)
 * and [13:9] = "rdy_en" - both WRONG: bit20 must be SET, and [13:9] are the
 * mirror mrr_dsel/mrr_ldpid fields left at the aal_ni_init golden (0x3e80). */
/* ★ ELNATH: real INTERNAL_PORT_ID_CFG = 0xa180 (rtl8277c 0xa1bc, which on Elnath
 * is NIRX_MISC_CFG).  Elnath fields: l3qmrx_demux_sel[7:0] (per-port ->
 * 0=L3QM/1=WAN/2=L2FE/3=L2TM - THIS is the NI->QM LAN handoff, no l3qmrx_to_lan
 * bit), mrr_dsel[9:8], mrr_ldpid[15:10], ...  Stock live = 0x00A87F00
 * (demux_sel=0x00 = ALL ports -> L3QM).  We never wrote 0xa180 = the missing
 * handoff (our old 0x3e80 write to 0xa1bc coincidentally set NIRX_MISC right). */
#define CA_NI_NI_INTERNAL_PORT_ID_CFG	0xa180		/* ELNATH (rtl 0xa1bc) */
#define  CA_NI_NI_L3QMRX_DEMUX_SEL_ALL	GENMASK(7, 0)	/* per-port demux, 0 -> L3QM */
#define  CA_NI_NI_MRR_CFG		GENMASK(15, 8)	/* mrr_dsel/mrr_ldpid */
/* ★ ELNATH: real NIRX_MISC_CFG = 0xa1bc (rtl8277c 0xa1f8).  Fields:
 * l2te_ni_mc_rdy_en[9], l3fewan/l3felan/txdma rdy[10:13], l3fe_l3qm_to_l2tm_*[5:7]
 * (dft 0x00008080).  Stock live = 0x00003E80 (this is where our old "intern_pid
 * 0x3e80" write to 0xa1bc coincidentally landed correctly - keep writing it). */
#define CA_NI_NI_NIRX_MISC_CFG		0xa1bc		/* ELNATH (rtl 0xa1f8) */
#define  CA_NI_NI_NIRX_MISC_STOCK_VAL	0x00003e80u
#define  CA_NI_NI_L2TE_NI_MC_RDY_EN	BIT(9)
/* stock leaves bit15 CLEAR (golden 0x3e80); U-Boot left it set on ours
 * (0xbe80).  It gates an RX-side handshake/drop; clear it to match stock. */
#define  CA_NI_NI_INTERNAL_BIT15	BIT(15)
#define  CA_NI_NI_L3QMRX_DEMUX_SEL	GENMASK(7, 0)	/* stock clears to 0 */

/*
 * ★ NI global static cfg (NI_HV_GLB_STATIC_CFG, 0xa01c): port_to_cpu[3:0] =
 * the "management port" whose ingress is delivered to the CPU path
 * (stock aal_ni_set_port_to_cpu, an NI-init step our minimal driver skips).
 * Vendor default 0xf = disabled (it relies on full L2 forwarding/flood
 * instead); we set 0 = LAN port 0 -> CPU, the minimal "port -> CPU" that
 * matches stock's flood-domain-includes-CPU intent without the full L2 stack.
 */
#define CA_NI_NI_GLB_STATIC_CFG		0xa01c
#define  CA_NI_NI_PORT_TO_CPU		GENMASK(3, 0)
#define  CA_NI_NI_MGMT_PT_TO_FE_ALSO	BIT(4)
/* ★ build35: bits[17:16] of 0xa01c are SET by stock (0x1A07002F) but 0 in our
 * boot-ROM leftover (0x1A04002F) - the ONE 0xa000-0xa1fc value that differed from
 * stock after every routing/demux reg was matched (coordinator full-region devmem
 * diff).  Prime NI->L3QM ingress-enable candidate: the frame egresses L2TM but L3QM
 * never accepts it (ni2qm_rx/0xa9fc stays flat) = the exact signature of a disabled
 * NI->QM ingress.  Set via RMW (OR only) so port_to_cpu[3:0] is preserved. */
#define  CA_NI_NI_GLB_STATIC_L3QM_EN	GENMASK(17, 16)	/* 0x00030000 */

/*
 * ★ STOCK-LINUX GOLDEN LIVE WORKING CPU-RX config (STOCK_golden_rx_regs.txt -
 * read via devmem while SSH'd over the LAN port = CPU-RX actively working).
 * This is THE reference for the Linux CPU-EPP path (better than U-Boot, which
 * uses a simpler RX path - two values differ from U-Boot, corrected below).
 * KEY red herrings, ALL 0 on working stock: PHY_PORT_STS, EQM_PA_REQ,
 * SCH0.voq_en, EPP.cmd_mode (stock is 32-bit!), CPU_EPP_CFG, cputag_cfg.
 * Delivery IS the CPU-EPP ring, which lives in DDR-coherent RAM @0x0bc48000. */
#define CA_NI_QM_AXIM2_CONFIG		0x6000
#define  CA_NI_QM_AXIM2_STOCK_VAL	0x000002ffu	/* outstanding 0xff + xbus_len 2 */
/* ★ ELNATH offsets + stock-live values (devmem-captured at the Elnath offsets) */
#define CA_NI_NI_L3QMRX_DEMUX_CFG1	0xa188		/* ELNATH (rtl 0xa1c4) */
#define  CA_NI_NI_DEMUX1_STOCK_VAL	0x00000000u
#define CA_NI_NI_L3QMRX_DEMUX_CFG0	0xa18c		/* ELNATH (rtl 0xa1c8) */
#define  CA_NI_NI_DEMUX0_STOCK_VAL	0xffff7f7fu
/* ★ THE REAL per-ldpid L2FE-vs-L3FE ingress fork (NIRX_L3FE_DEMUX_CFG1/0)
 * lives at 0xa1c4/0xa1c8, NOT the 0xa188/0xa18c above (those are the RM_TBMAX/
 * RM_CNTR rate-meter regs -- our writes there matched stock only by coincidence
 * of reset values).  Without these two the L2FE never forks a host frame into the
 * L3FE classifier -> l3fe_rx(0xa9bc)=0 -> the CPU-trap never fires.  Live-stock
 * golden 2026-07-13: 0xa1c4=0x00CBDA98, 0xa1c8=0x7000DA98. */
#define CA_NI_NIRX_L3FE_DEMUX_CFG1_REAL	0xa1c4
#define  CA_NI_NIRX_L3FE_DEMUX_CFG1_REAL_VAL	0x00CBDA98u
#define CA_NI_NIRX_L3FE_DEMUX_CFG0_REAL	0xa1c8
#define  CA_NI_NIRX_L3FE_DEMUX_CFG0_REAL_VAL	0x7000DA98u
#define CA_NI_NI_PORTORDER_CFG		0xa184		/* ELNATH INTERNAL_PORT_ID_CFG2 (rtl 0xa1c0) */
#define  CA_NI_NI_PORTORDER_STOCK_VAL	0x0024009bu
#define  CA_NI_NI_INTERNAL_STOCK_VAL	0x00a87f00u	/* 0xa180: demux_sel[7:0]=0 -> all ports to L3QM */
#define  CA_NI_NI_AUTOSYNC_STOCK_VAL	0x0000000fu	/* 0xa010 (stock Linux = 0xF, not U-Boot's 0) */
/* CPU-RX dest -> EQ pools.  profile_sel[3:0] indexes EQ_PROFILE.  ★ The DEEP-
 * QUEUE CPU egress ports (dest 8 AND 9) use profile_sel = 0x0C = 12 (tier-1 stock
 * 0x6188/0x618c=0x0C) -> EQ_PROFILE(12) -> EQ12.  Our OLD 0x0D (profile 13 ->
 * EQ13) was the bug: it pointed the deep_q frame at the non-deep CPU pool, which
 * the RMU never admits.  dest port 0 -> 0xf8. */
#define  CA_NI_NI_DESTPORT0_STOCK_VAL	0x000000f8u
#define  CA_NI_NI_DESTPORT8_STOCK_VAL	0x0000000cu	/* ELNATH stock: deep-queue dest 8/9 -> profile 12 -> EQ12 */
/* CPU empty-buffer pool stock CFG0/CFG2 (tier-1 devmem, authoritative):
 * EQ13 = CPU PRIMARY pool (@0x634c), EQ14 = CPU SECONDARY (@0x6360).  CFG0
 * bit31 is the REAL pool-enable (BIT(0) alone does NOT admit); CFG1 (the
 * {bid_start,total_buf} window) is built from the CA_NI_RX_EQ*_BID_START/
 * TOTAL_BUF seed constants so the register window matches the pushed BIDs;
 * CFG3 = CA_NI_QM_CFG3_CPU_POOL_VAL, CFG4 = 0.  (The earlier EQ8
 * CFG0=0x00400100 / CFG1=0x000008ab were misreads: CFG0 bit31 CLEAR = disabled
 * + CFG1 total_buf=0 = zero capacity, so the QM never admitted.) */
/* Legacy stock phys addresses (0x80020080 / 0x09240000) - NO LONGER used: the CPU
 * pools now point at OUR dma_alloc_coherent region (cfg0 built at runtime in
 * cortina_ni_rx_eq_init). Kept only for reference. */
#define  CA_NI_QM_EQ13_CFG0		0x80020081u
/* ★★ EQ13(pool0)/EQ14(pool1) = the CPU empty-buffer pools the RMU0 admits a
 * CPU-dest (PDPID 0x09) frame into, feeding the CPU-EPP ring -> NAPI.
 * CFG2 field layout (QM_QM_CFG2_EQ0): buffer_size[2:0], cpu_eq[3], refill_fbm_eqid
 * [6:4], refill_en[7], refill_ths[15:8], buffer_weight[18:16].
 *
 * cpu_eq (bit3) selects the pool's BUFFER-SOURCE model:
 *   cpu_eq=1 = CPU-push  (driver stages each skb DMA addr via CPU_PUSH_PADDR)
 *   cpu_eq=0 = HW auto-populate: the QM EQM fills the free-list itself by walking
 *              CFG0.phy_addr_start over CFG1.bid_count buffers of CFG2.buffer_size.
 * ca-ne.ko NEVER writes the cpu_eq flag (getters-only; .bss default 0) => stock is
 * unconditionally cpu_eq=0, and the RMU0 admit path allocates ONLY from that HW
 * free-list. A cpu_eq=1 pool's free-list cannot serve the admit -> the frame is
 * never admitted and RMU0_RX_PKT_CNTR (0x6900) stays 0.  That was our wall.
 *
 * FIX: cpu_eq=0 for both, CFG0.phy_addr_start pointed at OUR reserved DRAM region
 * (dma_alloc_coherent, see cortina_ni_rx_eq_init), refill_en=0 (EQM direct-populate,
 * NOT the FBM-refill path of stock EQ8=0x66ec).  NAPI then reads the delivered frame
 * straight out of that DRAM buffer (memcpy into a fresh skb) and the HW auto-recycles
 * the buffer on read-ptr advance - no CPU-push, no seed, no return-register write.
 * Stock live: EQ13 CFG2=0xFF00 (buffer_size idx0 = 128B), EQ14 CFG2=0xFF04 (idx4 =
 * 2048B); refill_ths[15:8]=0xFF is a don't-care with refill_en=0. */
/* Match stock EXACTLY: EQ13 (pool0) = 128B, EQ14 (pool1) = 2048B.  The RMU size-
 * selects eqp0 vs eqp1 and requires eqp0.buffer_size < eqp1.buffer_size (a small +
 * a large pool); making both 2048B (equal) breaks the size-match and RMU0 silently
 * refuses to admit.  Real CPU frames (>~56B once the +0x40 header is added) land in
 * EQ14; EQ13/128B stays idle (as on stock) - so its small buffer is never read. */
/* ★★ build45 ROOT-CAUSE FIX: cpu_eq=1 (bit3).  Tier-1 live diag: frames REACH RMU0 but
 * it drops EVERY one NO_BUFFER (0x6900 RMU0_RX == 0x6940 NO_BUF_DROP, 0x611c bit8 set,
 * eq13/14 pa_req 0x63bc/0x63c0 = 0) - the EQM has no free buffers.  cpu_eq=0 tells the HW
 * "the EQM auto-populates its own free-list", so it NEVER consumes our software
 * CPU_PUSH_PADDR pushes (cortina_ni_rx_populate) - they land nowhere, pa_req stuck 0.  But
 * we DO software-push (the cpu_eq=1 mechanism).  Setting cpu_eq=1 makes the EQM register
 * each pushed buffer -> pa_req>0 -> RMU0 admits.
 * ★★★ build67 (vendor source RE): CPU pools ARE cpu_eq=1 (host-fed via CPU_PUSH_PADDR;
 * aal_l3qm.c:1568,1583).  The REAL bug was ROUTING: our CPU frame resolved to pdpid 0x08 =
 * NI PORT 0 (a PHYSICAL wire egress; NI/phys ports = dest 8-15) with deep_q=1, so it was
 * SWITCHED OUT A WIRE PORT (bm_rx==bm_tx, no drop) and NEVER crossed into L3QM.  CPU ports
 * = dest 0-7; the fix routes the frame to CPU port 0 (pdpid 0x00, deep_q=0).  cpu_eq=0/eqid
 * 8-9 deep-queue pools are for NI egress ports 8-15, never written to the host - build66's
 * cpu_eq=0 was BACKWARDS.  buffer_size idx (aal_l3qm.c:1097): 1=128B..5=2048B, 0=INVALID.
 * EQ13 128B->idx1 cpu_eq=1 (0xff09); EQ14 2048B->idx5 cpu_eq=1 (0xff0d). */
#define  CA_NI_QM_EQ13_CFG2		0x0000ff0du	/* build77: EQ5 pool0 - cpu_eq=1 + buffer_size idx5=2048B (stock CPU pool size) + refill_en=0 */
#define  CA_NI_QM_EQ8_CFG2		0x0000ff00u
#define  CA_NI_QM_EQ14_CFG0		0x09240001u
#define  CA_NI_QM_EQ14_CFG2		0x0000ff0du	/* build76: cpu_eq=1 + buffer_size idx5=2048B (POOL1 host-fed) */
/* EQ_CFG_LOAD commit latch: bit31 = load trigger, low 16 bits = the per-EQ
 * "latch this EQ's CFG" set (EQ_CFG_LOAD_ALL 0xffff = every EQ; U-Boot's live
 * value 0x370 = its own set {EQ4,5,6,8,9} by bit position, NOT a count - 0x370
 * coinciding with the old EQ8 total 880 misled the earlier comment).  We
 * relocated the CPU pools to EQ13+EQ14, so we OR bits 13/14 in: without them the
 * freshly-written EQ13/EQ14 CFG would NOT latch and the QM would never admit
 * (qm_rx=0).  Dominant under either reading: as a bitmask it now includes our
 * pools; as a "count" it stays below the 0xffff the driver committed before. */
#define  CA_NI_QM_EQ_CFG_LOAD_STOCK	(BIT(31) | 0x0370u | BIT(13) | BIT(14))
/* ★★★ build62: CPU-EPP descriptor ring base = the DT `ddr_cache_buffer` @0x0bc48000
 * (0x20000, coherent) = STOCK's value (stock 0x7200 EPP paddr = 0x0BC48000).  Mapped
 * CACHED (MEMREMAP_WB) + coherent CPU_EPP AXI attr (0x12008060) + `dma-coherent` on the NE
 * DT node = the full stock CPU-EPP combo.  (builds 50-56 diverged from stock - moved the
 * ring to 0x09600000, mapped uncached, computed axi_top=0, used the non-coherent attr -
 * ALL reverted: match stock, don't invent.  The genuinely-untested combo is stock-exact +
 * the ni+tqm reset pulse + GO-last + 0x6110=0xFFFF.) */
#define CA_NI_RX_RING_PHYS		0x0bc48000u
/* ★★★ CPU-pool buffer region base: a FIXED phys inside the board's 0x09000000 no-map
 * DDR reserve (DTS reserved-memory 0x09000000, size 0x1628800; stock's EQ14 buffers
 * live at 0x09240000 in the same reserve).  The RMU can only DMA a frame into a
 * buffer whose PA falls in the NE's DDR window - a dma_alloc_coherent buffer lands
 * outside it and is never admitted.  Offset 4MB in to clear the low part; our pool is
 * ~1.02MB so [0x09400000, 0x09504000) stays well within the 22MB reserve. */
#define CA_NI_RX_CPU_POOL_PHYS		0x09400000u

/* ★ FBM EXSTACK (pointer-spill) region: the FBM DMAs buffer-pointer spills HERE
 * (aal_fbm_init __ca_malloc(depth*4), phys -> POOL+0x04 as (pbase>>12)<<4).  It is
 * SEPARATE from the packet buffers.  Leaving POOL+0x04=0 spilled at phys 0 = kernel
 * slab -> the build24 panic.  Place it high in the same 0x09000000 no-map reserve,
 * clear of the CPU pool (0x09400000..~0x09504000) and the ring (0x0bc48000). */
#define CA_NI_RX_FBM_EXSTACK_PHYS	0x0A000000u	/* in the no-map reserve, page-aligned */
#define CA_NI_RX_FBM_EXSTACK_DEPTH	0x800u		/* spill capacity (>= bufs pushed); *4 = 8KB */
#define CA_NI_RX_FBM_POOL0_COUNT	512u		/* pool0 buffer count (mult of 64, <=16384) */
#define CA_NI_RX_FBM_POOL_ID		0		/* CPU pool = FBM pool 0 (POOL+0x40 residue) */
/* FBM_CPU doorbell (window CA_NI_WIN_FBM_CPU), per-pool stride 0x20 (id<<5):
 * +0x00 = cmd (bit31 GO/BUSY, bit30 op:1=push, [2:0] flags), +0x04 = addr high,
 * +0x08 = buffer PA low.  Push = poll outstanding(POOL+0x2c) < depth, poll +0x00
 * bit31 clear, write +0x04/+0x08, then GO cmd (aal_fbm_buf_push/__aal_fbm_buf_access). */
#define CA_NI_QM_FBM_CPU_DOORBELL(id)	((id) << 5)
#define  CA_NI_QM_FBM_CPU_CMD_GO	BIT(31)
#define  CA_NI_QM_FBM_CPU_CMD_PUSH	BIT(30)
#define CA_NI_QM_FBM_POOL(id)		((id) << 7)	/* POOL window: pool desc base = id*0x80 */
#define CA_NI_QM_FBM_POOL_OUTSTND	0x2c		/* +0x2c outstanding-count (gate) */
#define CA_NI_RX_FBM_POOL_BUFSZ		2048u		/* 2048B buffers (stock cpu/dq pool buf_sz) */
/* FBM soft-reset: NI GLB-ctrl (CA_NI_WIN_GLB) +0xa0 bit17, pulse 1->0 (aal_fbm_reset) */
#define CA_NI_GLB_FBM_RESET		0xa0
#define  CA_NI_GLB_FBM_RESET_BIT	BIT(17)

/* ★★ SoC reset-manager @ GLB+0xa0 ("cortina,rst-mgr", stock DTS reset-controller@
 * 0xf43200a0, #reset-cells=1 => cell = bit).  reset-names map (stock rtl9607f.dts):
 * ni=bit0, l2fe=bit1, l2tm=bit2, l3fe=bit3, sdram=bit4, TQM(=the QM/L3QM/RMU block)=
 * bit5, ptp_timer=bit30.  Stock's probe PULSES these assert(set)->1ms->deassert(clear)
 * to make each engine re-run its internal init.  ★ Our clean-room driver never did this
 * for TQM, so the QM/RMU admit engine stays in reset: L3QM init-done (0x6988 bit30)
 * never sets, the deep-queue->RMU0 handoff never runs, and every config reg still reads
 * back correct (block mapped for register access) - the exact "config perfect, engine
 * off" wall.  This is the SoC reset-mgr at 0xa0, NOT the NI GLB_BLOCK_RST at 0x28 (our
 * old GLB+0x28 pulse hit the wrong reg and stormed).  NI/L2FE/L2TM already run (U-Boot
 * TX + our classified frame), so pulse ONLY TQM (bit5) - never NI (breaks U-Boot TX). */
#define CA_NI_GLB_SOC_RST		0xa0
#define  CA_NI_GLB_SOC_RST_TQM		BIT(5)
/* L3QM init-done: 0x6988 bit30 (stock aal_l3qm_check_init_done spins on it; stock live
 * 0x6988=0x65FFFFFF has bit30=1).  This is the REAL init-done, NOT the QM_PHY_PORT_STS
 * "qm_init_done" phantom we used before. */
#define CA_NI_QM_L3QM_STS		0x6988
#define  CA_NI_QM_L3QM_INIT_DONE	BIT(30)

/* ★ QM AXI-attribute indirect table (QM_QM_AXI_ATTRIBUTE_*).  Each EQ / CPU-EPP
 * port has an entry describing the AXI attributes (qos/cache/snoop/domain/prot)
 * the QM's DMA uses when it writes a frame into that pool's DDR buffer or the
 * CPU-EPP ring.  The entries power up INVALID, so the first buffer-DMA STALLS on
 * an unprogrammed entry -> the RMU never completes admission (qm_rx stuck 0).
 * ACCESS is an INDIRECT trigger, NOT a value: writing the stock post-completion
 * readback (0x4000000F: GO clear, rbw=1, ADDR=15) with a stale DATA0 commits a
 * garbage attribute and never polls -> AXI back-pressure hang.  Correct protocol
 * (aal_l3qm_set_epp_axi_attrib): write DATA0, then ACCESS = GO|rbw|ADDR, then
 * poll GO(bit31) clear with a bounded timeout. */
#define CA_NI_QM_AXI_ATTR_ACCESS	0x67cc	/* [31]=access/GO [30]=rbw(1=wr) [5:0]=ADDR */
#define CA_NI_QM_AXI_ATTR_DATA1		0x67d0	/* [1:0]=axi_top_bit (40-bit addr high) */
#define CA_NI_QM_AXI_ATTR_DATA0		0x67d4	/* qos/cache/snoop/bar/domain/prot payload */
#define  CA_NI_QM_AXI_ATTR_GO		BIT(31)
#define  CA_NI_QM_AXI_ATTR_RBW		BIT(30)	/* 1 = write the entry */
#define  CA_NI_QM_AXI_ATTR_ADDR		GENMASK(5, 0)
#define CA_NI_QM_AXI_ATTR_POLL_MAX	300	/* bounded poll, never spin forever */
#define CA_NI_QM_AXI_ATTR_EQ_BASE	0	/* entries 0-15 = EQ0-15 */
#define CA_NI_QM_AXI_ATTR_CPU_BASE	48	/* entries 48-55 = CPU-EPP port 0-7 */
#define CA_NI_QM_CPU_PORT_COUNT		8	/* CPU-EPP ports 0-7 (stock inits all) */
/* Attribute payloads (stock l3qm_axi_attrib_*): a DDR EQ pool = 0x04000010
 * (bufferable, non-coherent - driver dma_syncs); the CPU-EPP ring = 0x12008060
 * (domain-2 / ACE coherent so the CPU sees descriptors without a sync). */
#define CA_NI_QM_AXI_ATTR_DDR_POOL	0x04000010u
#define CA_NI_QM_AXI_ATTR_CPU_EPP	0x12008060u

/* QM_PHY_PORT_STS handshake bit groups (kept as a logged diagnostic only) */
#define  CA_NI_QM_STS_NIRX_PORT_RDY	GENMASK(7, 0)
#define  CA_NI_QM_STS_TE_ES_NI_OK	GENMASK(15, 8)
#define  CA_NI_QM_STS_NIRX_QM_RDY	BIT(26)
#define  CA_NI_QM_STS_AXI_WR_RDY	BIT(28)
#define  CA_NI_QM_STS_AXI_RD_RDY	BIT(29)

/* frame-length limits (stock aal_ni_pkt_len_set): 0xa034 holds
 * {min[25:16], max[13:0]}, 0xa038 the RX-side max [13:0].  Stock init
 * programs min=0x3c, max=0x2ee0 on both. */
#define CA_NI_HV_PKT_LEN		0xa034
#define  CA_NI_HV_PKT_LEN_MIN		GENMASK(25, 16)
#define  CA_NI_HV_PKT_LEN_MAX		GENMASK(13, 0)
#define CA_NI_HV_PKT_LEN_RX		0xa038
#define  CA_NI_HV_PKT_LEN_RX_MAX	GENMASK(13, 0)
#define  CA_NI_HV_PKT_LEN_MIN_VAL	0x3c
#define  CA_NI_HV_PKT_LEN_MAX_VAL	0x2ee0

/* unnamed globals stock aal_ni_init RMWs unconditionally */
#define CA_NI_HV_CFG_A420		0xa420
#define  CA_NI_HV_CFG_A420_FIELD	GENMASK(7, 0)
#define  CA_NI_HV_CFG_A420_VAL		0xc0
#define CA_NI_HV_CFG_AAF0		0xaaf0
#define  CA_NI_HV_CFG_AAF0_FIELD	GENMASK(9, 0)
#define  CA_NI_HV_CFG_AAF0_VAL		0x17

/* NI RX MIB (per-port receive statistics), indirect access.  Global registers
 * with the port selected in the ACCESS word (NOT a per-port stride).  Stock
 * __ni_eth_port_rx_mib_get (07f ko aal_ni_eth_port_mib_get @0xb820): write
 * ACCESS {go, rbw=0=read, op_code, port_id, counter_id}, poll go==0, read
 * DATA0 (cnt_lo, valid for every counter).  Used by the /proc RX spy to see
 * whether ingress frames even reach the MAC (localizes MAC-admit vs L2FE-drop
 * vs ring-write). */
#define CA_NI_HV_RXMIB_ACCESS		0xa168
#define  CA_NI_MIB_ACCESS_GO		BIT(31)		/* self-clears when done */
#define  CA_NI_MIB_ACCESS_RBW		BIT(30)		/* 0 = read              */
#define  CA_NI_MIB_ACCESS_OPCODE	GENMASK(29, 28)
#define  CA_NI_MIB_ACCESS_PORT		GENMASK(7, 5)
#define  CA_NI_MIB_ACCESS_CNTID		GENMASK(4, 0)
#define CA_NI_HV_RXMIB_DATA0		0xa16c		/* counter value [31:0]  */
#define CA_NI_HV_RXMIB_DATA1		0xa170		/* byte-count hi word    */
#define  CA_NI_MIB_OP_READ_ONLY		0		/* read, do not clear    */
/* RX counter ids (subset the spy reads) */
#define  CA_NI_MIB_RX_UC_PKT		0
#define  CA_NI_MIB_RX_MC_PKT		1
#define  CA_NI_MIB_RX_BC_PKT		2
/* bounded poll for the indirect MIB access (stock caps at ~5000 reads) */
#define CA_NI_MIB_POLL_US		1
#define CA_NI_MIB_POLL_TIMEOUT_US	5000

/* per-port block, port 0..3 = the four internal GbE MACs, stride 0x90.
 * ★ The block BASE is 0xa5c0 = NI_HV_PT_PORT_STATIC_CFG (aal_ni_eth_if_set,
 * 07f ko @0x9840): the MAC<->PHY INTERFACE config.  Fields (bit positions
 * from the ko's bfi sequence):
 *   int_cfg   [3:0]  interface mode (aal_ni_if_mode_t; 0 = GE_GMII, the
 *                    internal quad-GPHY runs GMII) - stock aal_port_init sets 0
 *   phy_mode  [4]    0 = this NI port is a MAC connecting to a PHY (correct);
 *                    1 = NI acts as a PHY toward an external MAC (wrong)
 *   lpbk_mode [13:12] MAC loopback: if set, the MAC loops TX->RX internally
 *                    and NEVER reaches the GPHY -> bidirectional dead while the
 *                    line side still links (exactly our symptom)
 * Our driver started the port block at 0xa5c4 and never touched 0xa5c0, so a
 * wrong U-Boot int_cfg/phy_mode/lpbk left the internal GMII disconnected
 * non-deterministically.  Force int_cfg=GE_GMII, phy_mode=MAC, lpbk=off. */
#define CA_NI_PORT_STRIDE		0x90
#define CA_NI_PORT_STATIC_CFG(p)	(0xa5c0 + (p) * CA_NI_PORT_STRIDE)
#define  CA_NI_PORT_STATIC_INT_CFG	GENMASK(3, 0)	/* 0 = GE_GMII */
#define  CA_NI_PORT_STATIC_PHY_MODE	BIT(4)		/* 0 = MAC->PHY */
#define  CA_NI_PORT_STATIC_LPBK_MODE	GENMASK(13, 12)	/* 0 = no MAC loopback */
/* per-port CPU-TAG trap (NI_HV_PT_PORT_STATIC_CFG.cpu_tag_*): enabling rx_en on
 * a port makes NI-RX add a cpu-tag (source port id) and trap the frame to the
 * CPU - the "each eth0.X = port" seam (X111W-style).  NB stock's
 * ca_ni_cpu_tag_init enables this ONLY on RTK_NI_PORT_4 (the external RTL-switch
 * uplink), never the internal GPHY ports - so wiring it on port 0 is a clean-
 * room design extension, not stock behaviour. */
#define  CA_NI_PORT_STATIC_CPU_TAG_RX_EN	BIT(16)
#define  CA_NI_PORT_STATIC_CPU_TAG_TX_EN	BIT(17)
#define  CA_NI_PORT_STATIC_CPU_TAG_CFG_SEL	BIT(18)
#define CA_NI_NI_CPU_TAG_CFG		0xa424	/* ELNATH NI_HV_GLB_CPU_TAG_CFG (rtl 0xa42c); stock 0x00048899 */
/* full stock-measured value (all ports read 0xCB000200 live on stock): low word
 * int_cfg=0/phy_mode=0/lpbk=0 + bit9, and the UPPER byte 0xCB000000 = the per-port
 * MAC<->GPHY datapath enable U-Boot leaves clear.  Write the whole word to match. */
#define  CA_NI_PORT_STATIC_CFG_VAL	0xCB000200u
#define CA_NI_PORT_GLB_CFG(p)		(0xa5c4 + (p) * CA_NI_PORT_STRIDE)
#define  CA_NI_PORT_GLB_SPEED_10M	BIT(0)	/* 1 = 10M, 0 = 100M/1G */
#define  CA_NI_PORT_GLB_HALF_DUPLEX	BIT(1)
#define  CA_NI_PORT_GLB_LINK_STATUS	BIT(2)	/* read-only */
#define  CA_NI_PORT_GLB_PWR_DWN_RX	BIT(11)
#define  CA_NI_PORT_GLB_PWR_DWN_TX	BIT(12)
#define CA_NI_PORT_RXMAC_CFG(p)		(0xa5c8 + (p) * CA_NI_PORT_STRIDE)
#define  CA_NI_PORT_RXMAC_RX_EN		BIT(0)
/* stock aal_ni_eth_port_mac_set programs the RX MAC to {rx_en, bit12, bit13}
 * with bit8 CLEAR (live golden = 0x00003001).  U-Boot leaves 0x1100 (bit8 +
 * bit12) so ours came up 0x1101 - wrong.  bit12/bit13 are the RX MAC
 * crc-check/pad-strip class enables; bit8 is a drop enable stock does NOT
 * want set for the CPU RX path.  Force the stock pattern at bring-up. */
#define  CA_NI_PORT_RXMAC_STOCK_SET	(BIT(12) | BIT(13))
#define  CA_NI_PORT_RXMAC_STOCK_CLR	BIT(8)
#define CA_NI_PORT_TXMAC_CFG(p)		(0xa5d4 + (p) * CA_NI_PORT_STRIDE)
#define  CA_NI_PORT_TXMAC_TX_EN		BIT(0)
#define  CA_NI_PORT_TXMAC_CRC_CALC_EN	BIT(8)	/* HW appends FCS */
#define  CA_NI_PORT_TXMAC_TX_DRAIN	BIT(13)

/* ★★ build36: stock enables ALL 7 port blocks (0xa5c0 + p*0x90); ports 0-4 = physical
 * GMAC, ports 5-6 = INTERNAL (CPU/QM/L3QM-facing).  Our driver init'd ONLY port 0, so
 * the internal ports 5/6 - the ones that accept the L2TM egress INTO L3QM - had
 * rx_en/tx_en=0: a frame egressed L2TM but the internal port never accepted it, so
 * ni2qm_rx (0xa9fc, in that internal-port block) stayed flat 0.  Live-stock golden
 * (aal_port_enable/__ge_port_enable): p1-6 RXMAC=0x3101 (rx_en b0 + 0x3100), TXMAC=
 * 0x04055901 (tx_en b0), RX_CNTRL=0x08000600 (byp_en b28=0).  (Port 0 keeps its own
 * GPHY-link path variant RXMAC=0x3001/TXMAC=0x04054901.) */
#define CA_NI_PORT_COUNT		7		/* 0-4 GMAC, 5-6 internal */
#define CA_NI_PORT_RXMAC_EN_VAL		0x00003101u	/* stock p1-6 RXMAC (rx_en) */
#define CA_NI_PORT_TXMAC_EN_VAL		0x04055901u	/* stock p1-6 TXMAC (tx_en) */
#define CA_NI_PORT_RX_CNTRL_STOCK_VAL	0x08000600u	/* stock all-port RX_CNTRL (byp_en=0) */

/* --- NI core window (idx 0): L2TM block (QM buffer manager + egress
 *     scheduler), offsets identical to CA8277B --- */
#define CA_NI_L2TM_QM_EQ_CFG		0x2208
#define  CA_NI_L2TM_EQ0_BUFNUM		GENMASK(4, 0)
#define  CA_NI_L2TM_EQ0_PRVT		GENMASK(15, 5)
#define  CA_NI_L2TM_EQ1_BUFNUM		GENMASK(19, 16)
#define  CA_NI_L2TM_EQ1_PRVT		GENMASK(29, 20)
#define  CA_NI_L2TM_EQ0_EN		BIT(30)
#define  CA_NI_L2TM_EQ1_EN		BIT(31)
#define CA_NI_L2TM_QM_GLOB_BUF_CFG	0x2210
#define  CA_NI_L2TM_BUF_NODROP		GENMASK(14, 0)
#define  CA_NI_L2TM_BUF_DROP_EN		BIT(15)
#define  CA_NI_L2TM_BUF_NONCONG		GENMASK(30, 16)
#define  CA_NI_L2TM_BUF_FE_BP_EN	BIT(31)
#define CA_NI_L2TM_QM_PORT_PRVT_PROF0	0x2218	/* profiles 0/1 packed 16/16 */
#define CA_NI_L2TM_ES_CTRL		0x2300
#define  CA_NI_L2TM_ES_TX_EN		BIT(31)
/* per-port ES enable = bit(port); valid ports 0..13 and 15 */
#define  CA_NI_L2TM_ES_PORT_EN_ALL	0xbfff
/* per-port VOQ-enable regs: port 0..13 -> 0x2304 + port*12; there is no
 * port 14, and port 15 maps to 0x23ac = instance 14 of this loop (the csel
 * in stock aal_l2_tm_es_voq_ena_set) - so i = 0..14 covers every port.
 * Stock also sets field [23:16] := 6 on instances 8/10/13 (not our egress
 * port 0) - deferred. */
#define CA_NI_L2TM_ES_SCH_CFG(i)	(0x2304 + (i) * 12)  /* i = 0..14 */
#define  CA_NI_L2TM_ES_VOQ_EN_ALL	GENMASK(7, 0)	/* voq0..7 enable */
#define CA_NI_L2TM_ES_SCH_INSTANCES	15
/* ES egress-port indices (stock CA_AAL_ES_PORT_*): port 8 = L3QM = the
 * deep-queue -> QM/CPU drain we must enable for CPU-RX. */
#define CA_NI_L2TM_ES_PORT_L3QM		8

/* ★★ DEEP-QUEUE / CENTRAL-BUFFER SCHEDULER (L2TE_CB + DQSCH).  Tier-1 stock has
 * this whole block populated; our driver never touched it, so a deep_q=1 frame
 * is enqueued into the central buffer but the DQSCH never drains it to the RMU
 * (0x6900 stuck 0).  Direct threshold regs = verbatim stock; the per-VOQ
 * threshold tables are indirect (ACCESS=GO|WR|idx, DATA first). */
#define CA_NI_L2TM_CB_VOQ_THRSH_ACCESS	0x2da0	/* CB_VOQ_THRSH_PROFILE_MEM_ACCESS */
#define CA_NI_L2TM_CB_VOQ_THRSH_DATA1	0x2da4
#define CA_NI_L2TM_CB_VOQ_THRSH_DATA0	0x2da8
#define CA_NI_L2TM_DQSCH_EQ_THRSH	0x2e58	/* DQSCH_EQ_PROFILE_THRSH0 (dft 0x7fff) */
#define CA_NI_L2TM_DQSCH_PORT_THRSH0	0x2e5c	/* DQSCH_PORT_THRSH_PROFILE0..3 */
#define CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS	0x2e70	/* DQSCH_VOQ_THRSH_PROFILE_MEM_ACCESS */
#define CA_NI_L2TM_DQSCH_VOQ_THRSH_DATA		0x2e74
#define CA_NI_L2TM_CB_ABR_CTRL		0x2eec	/* stock 0x88000200 */
#define  CA_NI_L2TM_CB_ABR_CTRL_STOCK	0x88000200u	/* bit31 = ABR deep-q arbiter enable (FINAL) */
#define  CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE 0x7fff7fffu /* permissive VOQ profile threshold */
#define  CA_NI_L2TM_DEEPQ_ACC_POLL	10000u		/* busy-poll cap for the indexed GO bit */
/* ★★ build38: the DQSCH OUTPUT static config (0x2f00/04/08) - the deep-queue-dequeue
 * -> TM-port binding region our driver NEVER wrote (0x2f0c+ are LIVE counters, do NOT
 * write).  Branch-3 (HW-proven): a deep_q frame is dequeued (bm_tx 0x2140 +9) but drains
 * to the WRONG TM-port (not 8=L3QM), so 0xa9fc stays 0 with no drop.  0x2f04=0x33445550
 * is the standout static map (suspected DQSCH-output/src -> TM-port; RE a053902d
 * confirming).  Stock live golden (stock_dqsch_2d00.txt); write verbatim. */
#define CA_NI_L2TM_DQSCH_OUT_CFG0	0x2f00
#define  CA_NI_L2TM_DQSCH_OUT_CFG0_VAL	0x00000020u
#define CA_NI_L2TM_DQSCH_OUT_PORT_MAP	0x2f04		/* suspected deep-queue-output -> TM-port map */
#define  CA_NI_L2TM_DQSCH_OUT_PORT_MAP_VAL 0x33445550u
#define CA_NI_L2TM_DQSCH_OUT_CFG2	0x2f08
#define  CA_NI_L2TM_DQSCH_OUT_CFG2_VAL	0x00000005u
/* ★★★ THE central-buffer SCANNER enable (L2TE_CB_CTRL, from ca-ne.ko disasm).
 * scan_enable = bit31 drives the scanner that SCANS + DRAINS the deep queues out
 * of the central buffer to the QM.  Reset dft = 0x0003ff03 (scan_enable=0); stock
 * = 0x8003FF03 (scan_enable=1).  Our driver NEVER wrote it, so the deep queue was
 * configured but NEVER SCANNED -> a deep_q frame sat in the CB and never reached
 * RMU0 (0x6900=0, no drops).  Enable it LAST, after all thresholds. */
#define CA_NI_L2TM_CB_CTRL		0x2d0c
#define  CA_NI_L2TM_CB_CTRL_STOCK	0x8003ff03u	/* scan_enable=1 + reset scan-cycle */
/* CB occupancy counters (indirect ACCESS/DATA) - the A/B bisect: does a deep_q
 * frame get ENQUEUED into the central buffer at all?  VOQ_BUFCNT climbs = frame
 * in CB (scanner/drain gap); stays 0 while stock climbs = never enqueued. */
#define CA_NI_L2TM_CB_VOQ_BUFCNT_ACCESS	0x2dbc
#define CA_NI_L2TM_CB_VOQ_BUFCNT_DATA	0x2dc0
#define CA_NI_L2TM_CB_PORT_FREECNT_ACCESS 0x2db4
#define CA_NI_L2TM_CB_PORT_FREECNT_DATA	0x2db8
/* ★★★ THE deep-queue POPULATE step (ca-ne.ko: aal_l3qm_init_DQ_pools_pool0 ->
 * aal_l3_te_cb_port_free_buf_cnt_set, looped over ports 0-47).  Initialises the
 * CB's per-port FREE-BUFFER count; without it the CB has 0 free deep-queue
 * buffers and DROPS a deep_q frame at the L2TM->CB enqueue (cb-occupancy=0).
 * DATA fields = cnt0[14:0], cnt0_msb[15], cnt1[30:16], cnt1_msb[31].  Tier-1
 * stock value (read live under ping, UNIFORM on all ports) = 0x8E308000 =
 * {cnt0=0x8000(32768), cnt1=0x8E30(36400)} - the REAL central-buffer free-buffer
 * counts (~32K, the CB SRAM size), not a per-DQ-pool 256.  Our 256 guess was
 * orders of magnitude too small so the CB rejected the enqueue. Write verbatim. */
#define CA_NI_L2TM_CB_PORT_COUNT	48	/* ports 0..47 (stock loop range) */
#define  CA_NI_L2TM_CB_FREECNT_VAL	0x8e308000u	/* tier-1 stock, all ports */
#define CA_NI_L2TM_DEEPQ_VOQ_ENTRIES	8	/* 8 VOQs per deep-queue port */
#define  CA_NI_L2TM_CB_VOQ_THRSH_D1	0x0fffffffu	/* stock CB VOQ thr (last entry) */
#define  CA_NI_L2TM_CB_VOQ_THRSH_D0	0xffffffffu
#define  CA_NI_L2TM_DQSCH_VOQ_THRSH_VAL	0x00700070u	/* stock DQSCH VOQ thr (lo=hi=0x70) */
/* stock QM buffer-manager values (aal_l2_qm_init) */
#define CA_NI_QM_EQ1_BUFNUM_VAL		4	/* 4K x 64-byte buffers */
#define CA_NI_QM_PORT_PRVT_BUFF_NUM	29	/* x64B, one max-size frame */
#define CA_NI_QM_NODROP_THRESHOLD	0x21
#define CA_NI_QM_NONCONG_THRESHOLD	0x800	/* stock l2tm glob_buf (0x2210) */

/* --- DMA-LSO window (idx 4), window-relative (phys - 0xf7001000) --- */
#define CA_DMA_LSO_CTRL			0x000
#define  CA_DMA_LSO_CTRL_VAL		0x2d	/* tx_dma_enable | burst_len=3
						 * (bits 3:2) | lso_padding_en */
#define  CA_DMA_LSO_CTRL_LSPID_KEEP	BIT(4)
#define  CA_DMA_LSO_CTRL_PAD_EN		BIT(5)
/* stock writes 0x2d first (with readback), then flips to lspid_keep=1 /
 * lso_padding_en=0 - final value 0x1d (rtk_ni_init_tx_dma_lso @0x57a4:
 * and ~0x20, orr 0x10).  Without lspid_keep the source LSPID from the
 * lspid-map table is not kept on the frame. */
/* non-ACE companion (ca_ni_init_tx_dma_lso @0xaa314): low byte := 0 */
#define CA_DMA_LSO_SRAM_TEST_CTRL1	0x0bc
#define CA_DMA_LSO_VLAN_TAG_TYPE0	0x00c
#define  CA_DMA_LSO_VLAN_TAG_TYPE0_VAL	0x800088a8	/* en | TPID 0x88a8 */
#define CA_DMA_LSO_AXI_USER_SEL0	0x014
#define  CA_DMA_LSO_AXI_USER_SEL0_VAL	0x00555555	/* vp0..11 rd_pat=1 */
#define CA_DMA_LSO_AXI_USER_PAT0	0x018	/* ..PAT3 at 0x1c/0x20/0x24 */
#define  CA_DMA_LSO_AXI_USER_PAT_VAL	0x02008080	/* ace_cmd=1 (bit25),
						 * cache=2 (17:14),
						 * domain=2 outer-shareable
						 * (7:6) - coherent CCI path */
/* 07f-only pair, written unconditionally by stock TX-DMA init (the CA8277B
 * names at these offsets no longer apply - block regenerated) */
#define CA_DMA_LSO_MISC_C0		0x0c0
#define  CA_DMA_LSO_MISC_C0_VAL		0x1
#define CA_DMA_LSO_MISC_C4		0x0c4
#define  CA_DMA_LSO_MISC_C4_VAL		0xc0007777
/* 07f VP -> LSPID map table (indirect); entry n: lspid 0x10+n, valid */
#define CA_DMA_LSO_LSPID_MAP_ACCESS	0x0d8
#define  CA_DMA_LSO_LSPID_MAP_IDX	GENMASK(3, 0)
#define CA_DMA_LSO_LSPID_MAP_DATA0	0x0dc
#define  CA_DMA_LSO_LSPID_MAP_VALID	BIT(18)
#define  CA_DMA_LSO_LSPID_MAP_LSPID	GENMASK(17, 12)
#define CA_DMA_LSO_LSPID_MAP_DATA1	0x0e0
#define CA_DMA_LSO_LSPID_MAP_ENTRIES	16
#define CA_DMA_LSO_LSPID_CPU0		0x10	/* AAL_LPORT_CPU_0 */

/* per-VP block, stride 0xa0, VP 0..11; CPU n transmits on VP n+2
 * (verified in the 07f __ca_ni_start_xmit_buf_for_fc_dirTx: vp = cpu + 2) */
#define CA_DMA_LSO_VP_STRIDE		0xa0
#define CA_DMA_LSO_VP_COUNT		12
#define CA_DMA_LSO_VP_CONTROL(vp)	(0x100 + (vp) * CA_DMA_LSO_VP_STRIDE)
#define  CA_DMA_LSO_VP_TXQ_ALL_EN	GENMASK(7, 0)	/* txq0..7 enable */
#define CA_DMA_LSO_VP_BD_ACCESS(vp)	(0x104 + (vp) * CA_DMA_LSO_VP_STRIDE)
#define  CA_DMA_LSO_BD_ACCESS_TXQ	GENMASK(2, 0)
#define  CA_DMA_LSO_BD_ACCESS_WRITE	BIT(30)		/* rbw: 1 = write */
#define  CA_DMA_LSO_BD_ACCESS_GO	BIT(31)		/* self-clearing */
/* DATA1 low byte = ring base addr[39:32].  Stock writes 0: the "= 2"
 * branch is the dma_lso_ace_test path (@0x47b8), disabled on the device
 * (scfg CFG_ID_DMA_LSO_ACE_TEST = 0, .bss default 0). */
#define CA_DMA_LSO_VP_BD_DATA1(vp)	(0x108 + (vp) * CA_DMA_LSO_VP_STRIDE)
#define CA_DMA_LSO_VP_BD_DATA0(vp)	(0x10c + (vp) * CA_DMA_LSO_VP_STRIDE)
#define  CA_DMA_LSO_BD_DATA0_DEPTH	GENMASK(3, 0)	/* log2(ring size) */
#define  CA_DMA_LSO_BD_DATA0_BASE	GENMASK(31, 4)	/* paddr, 16B aligned */
#define CA_DMA_LSO_VP_TXQ_WPTR(vp, q)	(0x118 + (vp) * CA_DMA_LSO_VP_STRIDE \
					       + (q) * 8)
#define CA_DMA_LSO_VP_TXQ_RPTR(vp, q)	(0x11c + (vp) * CA_DMA_LSO_VP_STRIDE \
					       + (q) * 8)
#define  CA_DMA_LSO_PTR_MASK		GENMASK(12, 0)
/* per-VP header-A config (stock aal_ni_set_dma_lso_headerA @0x4330):
 * febypass (bit9) is the FE-BYPASS enable for direct TX - reset default 0,
 * stock sets it (ldpid=0, febypass=1) on all 12 VPs. */
#define CA_DMA_LSO_VP_HDRA_CFG(vp)	(0x178 + (vp) * CA_DMA_LSO_VP_STRIDE)
#define  CA_DMA_LSO_HDRA_LDPID		GENMASK(8, 3)
#define  CA_DMA_LSO_HDRA_FEBYPASS	BIT(9)

#define CA_DMA_SS_CTRL			0x900
#define  CA_DMA_SS_CTRL_TX_EN		BIT(31)
#define CA_DMA_AXIM2_CONFIG		0xd80
#define  CA_DMA_AXIM2_CONFIG_BITS	0x7aff	/* out-trans 0xf/0xf,
						 * xbus_len=2, rd/wr
						 * cacheline trans en+size */
/* HW short-frame pad (stock rtk_ni_init_tx_dma_lso @0x57c8) - the real pad
 * once lso_padding_en is off; we SW-pad too, belt and braces */
#define CA_DMA_AFT_CTRL			0xf00
#define  CA_DMA_AFT_PAD_EN		BIT(0)
#define  CA_DMA_AFT_PAD_SIZE		GENMASK(17, 4)
#define  CA_DMA_AFT_PAD_SIZE_VAL	0x40

/* --- AXI-reorder window (idx 10): 07f TX-DMA init writes 0xf to five
 *     stride-0x80 slots (from aal_ni_init_tx_dma_lso, g_ne_axi_reo base) --- */
#define CA_AXI_REO_SLOT(n)		(0x1000 + (n) * 0x80)
#define CA_AXI_REO_SLOT_COUNT		5
#define CA_AXI_REO_SLOT_VAL		0xf
/* ★★ RMU AXI read/write REORDER channels (stock axi_reo_rd_init @0x4fa70 / wr_init
 * @0x4fb40, in ca_ni_init_l3qm after enable_rx).  Without these the RMU dequeue DMA
 * never completes -> a CPU frame reaches the QM but is never admitted (wptr=0,
 * 0x6900=0, no drop).  Same g_ne_axi_reo window (idx 10), low offsets. */
#define CA_NI_AXI_REO_RD0		0x000
#define  CA_NI_AXI_REO_RD0_VAL		0x0000000fu
#define CA_NI_AXI_REO_RD1		0x004
#define  CA_NI_AXI_REO_RD1_VAL		0x8000000cu
#define CA_NI_AXI_REO_RD2		0x008
#define  CA_NI_AXI_REO_RD2_VAL		0x10000000u
#define CA_NI_AXI_REO_RD3		0x00c
#define  CA_NI_AXI_REO_RD3_VAL		0x8000000du
#define CA_NI_AXI_REO_WR0		0x400
#define  CA_NI_AXI_REO_WR0_VAL		0x00000004u
#define CA_NI_AXI_REO_WR1		0x404
#define  CA_NI_AXI_REO_WR1_VAL		0x8000000cu
/* ★★ build97: the L3FE AXI read-reorder channel (vendor aal_l3fe_axi_reo_init, aal_l3fe.c:341,
 * run LAST in aal_l3fe_init).  It is the SIBLING of the main NI AXI-REO channels our driver
 * already programs (win10+0x000 = rd ORIG 0xF->0xC, win10+0x400 = wr ORIG 4); the L3FE channel
 * (AXI ID 2->8) is at win10+0x480 (abs 0xf432d480 - TIER-1 STOCK VALIDATED 2026-07-12; the
 * build96 win10+0x2080 guess was wrong).  Our driver SKIPPED it -> the L3FE's AXI reads (ID 2)
 * aren't reordered -> the L3FE can't fetch the frame from memory -> l3fe_rx=0.  Each channel has
 * 7 regs: ORIG(+0)/NEW(+4)/TOP(+8)/MASK(+c)/NEW0(+10) + two 0xFFFFFFFF at +0x18/+0x24.  Fields:
 * ORIG_ID axi_id[3:0]; NEW_ID axi_id[3:0]+valid[31]; TOP_ADDR/MASK id[31:0].  All 7 values
 * byte-match stock live. */
#define CA_NI_L3FE_AXI_REO_ORIG_ID	0x480
#define  CA_NI_L3FE_AXI_REO_ORIG_ID_VAL	0x00000002u	/* axi_id=2 */
#define CA_NI_L3FE_AXI_REO_NEW_ID	0x484
#define  CA_NI_L3FE_AXI_REO_NEW_ID_VAL	0x80000008u	/* axi_id=8, valid */
#define CA_NI_L3FE_AXI_REO_TOP_ADDR	0x488
#define  CA_NI_L3FE_AXI_REO_TOP_ADDR_VAL	0x10000000u
#define CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK 0x48c
#define  CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK_VAL 0x10000000u
#define CA_NI_L3FE_AXI_REO_NEW_ID0	0x490
#define  CA_NI_L3FE_AXI_REO_NEW_ID0_VAL	0x80000009u	/* axi_id=9, valid */
#define CA_NI_L3FE_AXI_REO_RD18		0x498		/* +0x18 = 0xFFFFFFFF (per-channel, stock live) */
#define  CA_NI_L3FE_AXI_REO_RD18_VAL	0xFFFFFFFFu
#define CA_NI_L3FE_AXI_REO_RD24		0x4a4		/* +0x24 = 0xFFFFFFFF (per-channel, stock live) */
#define  CA_NI_L3FE_AXI_REO_RD24_VAL	0xFFFFFFFFu

/*
 * --- TX descriptor (8 bytes, little-endian, in coherent memory) ---
 *
 * word0 = buffer phys addr [31:0].
 * word1 below is the RTL9607F/RTL8277C-generation "direct TX to LAN"
 * encoding (mode=1, direct=0): the destination port and CoS ride in the
 * DESCRIPTOR and the buffer holds a plain Ethernet frame - no 16-byte
 * header-A prefix, no LSO words (HP[1:0]=11 = "no headers in buffer").
 * Verified against the 07f ko (__ca_ni_start_xmit_buf_for_fc_dirTx builds
 * 0x81400000 | len<<8 | cos<<5 | dest<<1 | chk_sel<<19).
 */
#define CA_NI_TX_DESC_WORDS		2
#define CA_NI_TX_DESC1_DEST		GENMASK(4, 1)	/* LAN port 0..3 */
#define CA_NI_TX_DESC1_COS		GENMASK(7, 5)
#define CA_NI_TX_DESC1_LEN		GENMASK(18, 8)	/* frame len, no FCS */
#define CA_NI_TX_DESC1_CHK_SEL		GENMASK(21, 19)
#define  CA_NI_TX_CHK_DISABLE		0
#define  CA_NI_TX_CHK_AUTO		1
#define CA_NI_TX_DESC1_MODE_DIRECT	BIT(22)		/* direct-TX format */
#define CA_NI_TX_DESC1_TO_WAN		BIT(23)		/* 0 = LAN */
#define CA_NI_TX_DESC1_HP0		BIT(24)
#define CA_NI_TX_DESC1_HP1		BIT(31)

#define CA_NI_TX_RING_SIZE		1024	/* descriptors per queue */
#define CA_NI_TX_RING_DEPTH		10	/* log2(1024), HW encoding */
#define CA_NI_TX_NUM_VPS		4	/* CPU 0..3 -> VP 2..5 */
#define CA_NI_TX_VP_BASE		2
#define CA_NI_TX_MIN_DMA_SIZE		34	/* engine minimum */
#define CA_NI_TX_MAX_FRAME		2047	/* 11-bit len field */
#define CA_NI_TX_RESERVE_DESC		2	/* stock keeps 2 in hand */

#define CA_NI_TX_POLL_US		1
#define CA_NI_TX_POLL_TIMEOUT_US	10000

/*
 * ---------------------------------------------------------------------------
 * M2c RX datapath registers - all in the NI core window (DT idx 0).
 *
 * The RX engine is the L3QM CPU-EPP path: ingress frames land in DMA buffers
 * taken from an empty-buffer pool the driver seeds (QM block, +0x6xxx), and
 * for each filled buffer the HW appends one 8-byte descriptor to a per-
 * (cpu-port,voq) FIFO ring in coherent DDR whose pointers live at +0x7xxx.
 * Offsets re-verified per-function in the shipped RTL9607F firmware
 * (ca-ne.ko: aal_l3qm_init_cpu_epp, aal_l3qm_set_cpu_push_paddr,
 * aal_l3qm_get_rx_write_ptr, aal_l3qm_set_rx_read_ptr,
 * aal_l3qm_get_inactive_bid_cntr, aal_l3qm_enable_cpu_epp_interrupt,
 * ca_ni_rx_napi, ca_ni_rx_napi_get_header_from_64bit_epp,
 * aal_ni_port_rx_ctrl_set, aal_ple_dft_fwd_set); bit layouts cross-checked
 * against the CA8277B public register file (identical fields, some blocks
 * re-based on the 07f).
 * ---------------------------------------------------------------------------
 */

/* --- QM block: CPU-EPP configuration (one-time init) --- */
/* per-cpu-port EPP config; only map_mode used (0 = linear voq map) */
#define CA_NI_QM_CPU_EPP_CFG(p)		(0x6680 + (p) * 4)	/* ELNATH CPU_EPP0_CFG (rtl 0x6558) */
#define  CA_NI_QM_EPP_MAP_MODE		GENMASK(2, 0)
/* descriptor-coalescing timer bases; stock writes 0 */
#define CA_NI_QM_CPU_EPP_CT_CFG		0x66a0	/* ELNATH (rtl 0x6578) */
/* EPP FIFO profiles 0..7; the CPU RX path uses profile 4 */
#define CA_NI_QM_CPU_EPP_FIFO_PROF(n)	(0x66a4 + (n) * 4)	/* ELNATH CPU_EPP_FIFO_CFG_profile0 (rtl 0x657c) */
#define  CA_NI_QM_EPP_PROF_HIGH_THS	GENMASK(3, 0)	/* IRQ at >= N entries */
#define  CA_NI_QM_EPP_PROF_TIMER_THS	GENMASK(7, 4)	/* aging force-IRQ    */
#define  CA_NI_QM_EPP_PROF_SIZE		GENMASK(28, 8)	/* in 4-entry units   */
/* per-(port,voq) profile select */
#define CA_NI_QM_CPU_EPP_FIFO_CFG(p, q)	(0x66cc + (p) * 0x20 + (q) * 4)	/* ELNATH CPU_EPP_FIFO0_0_CFG (rtl 0x65a4), per-port stride 0x20 */
#define  CA_NI_QM_EPP_PROFILE_SEL	GENMASK(2, 0)
#define CA_NI_RX_PROFILE_ID		4	/* stock epp_profile_cpu_id */

/* --- QM block: CPU-EPP interrupt enable (also the mask/ack - the interrupt
 *     is level, driven by FIFO occupancy vs high_ths; no separate W1C).
 *     One byte per cpu_port: port0 -> bits[7:0] of INT_EN0 (voq0..7). --- */
#define CA_NI_QM_EPP64_INT_EN0		0x6110	/* cpu ports 0..3 */
#define CA_NI_QM_EPP64_INT_EN1		0x6114	/* cpu ports 4..7 */
#define  CA_NI_QM_EPP64_INT_PORT0	GENMASK(7, 0)
/* ★★ build61: stock LIVE 0x6110 = 0x0000FFFF (bits 0-15), NOT 0xff - we only ever set
 * 0xff (port0 byte) or 0xffffffff (build49, regressed).  bits[15:8] (RE: the writeback-
 * completion / wptr-update latch enable) were the missing piece; the writeback never
 * fired without them.  Stock 0x6118 = 0x00000100 (per-EQ-pool refill-threshold IRQ en). */
#define  CA_NI_QM_EPP64_INT_EN0_STOCK	0x0000FFFFu
#define CA_NI_QM_EPP64_INT_EN2		0x6118
#define  CA_NI_QM_EPP64_INT_EN2_STOCK	0x00000100u

/* --- QM block: empty-buffer pool --- */
/* push one 128-byte-aligned buffer phys addr into EQ pool <eqid> */
#define CA_NI_QM_CPU_PUSH_PADDR(p)	(0x63cc + (p) * 8)	/* build87: OLD offset RESTORED - live-stock devmem: 0x7328 is a dead/abort zone (SError); 0x63cc is the real CPU_PUSH_PADDR0 */
#define  CA_NI_QM_PUSH_ADDR		GENMASK(31, 7)	/* pa >> 7 */
#define  CA_NI_QM_PUSH_EQID		GENMASK(3, 0)
/* per-cpu-port push status: bit31 = the (shallow) CPU-push stage has a
 * free slot.  MANDATORY gate before every CPU_PUSH_PADDR write: a blind
 * write while the stage is full back-pressures the AXI write and HANGS
 * the CPU.  The stage only drains once the EQ config is COMMITTED (see
 * CA_NI_QM_EQ_CFG_LOAD) and the RMU0 RX master runs; until then it caps
 * at ~4 and the ready bit stays low - so the poll must be bounded. */
#define CA_NI_QM_CPU_PUSH_READY(p)	(0x63c8 + (p) * 8)	/* build87: OLD offset RESTORED - live-stock: 0x63c8=0x80000000 (valid ready-bit); 0x7324 reads blank/abort */
#define  CA_NI_QM_PUSH_READY		BIT(31)
/* generous per-push ready timeout (us); bounds the seed loop, can't hang */
#define CA_NI_RX_PUSH_TIMEOUT_US	1000
/* Per-pool empty-buffer status = the REAL inactive_bid_cntr at 0x6388+eqid*4
 * (stock aal_l3qm_get_inactive_bid_cntr @ca-ne.ko 0x4f1d8): bit31 = count valid,
 * [13:0] = buffer count.  ★ 0x63a8+eqid*4 was a PHANTOM: for the CPU eqids it
 * ALIASES the port-strided CPU-push block (CPU_PUSH_RDY 0x63c8+port*8 /
 * CPU_PUSH_PADDR 0x63cc+port*8), so the earlier "EQ0 pa_req @0x63a8, EQ12 @0x63d8"
 * live reads were actually CPU_PUSH_RDY(port) bits, not pool req - reverted to the
 * real 0x6388 (confirmed by disasm + the coordinator's independent RE). This is
 * the same register as CA_NI_QM_EQM_INACTIVE_BID below. */
#define CA_NI_QM_EQM_PA_REQ(eqid)	(0x6388 + (eqid) * 4)	/* build87: OLD offset RESTORED - live-stock: 0x6388=0 at rest / 0x80000019 during a req; 0x72e4 is a pool-base ADDR table, not EQM_PA_REQ */
#define  CA_NI_QM_PA_REQ_READY		BIT(31)		/* req: the pool WANTS buffers */
#define  CA_NI_QM_PA_INACTIVE_CNT	GENMASK(13, 0)	/* buffer count */
/* Per-packet-engine inactive-bid (free-buffer shortfall) count, read by stock
 * aal_l3qm_get_inactive_bid_cntr: bits[13:0] = # buffers the pool is SHORT (0 when
 * fully populated); bit31 = valid/err.  Indexed by PE, not eqid. */
#define CA_NI_QM_EQM_INACTIVE_BID(pe)	(0x6388 + (pe) * 4)	/* build87: OLD offset RESTORED (= EQM_PA_REQ0 0x6388, same reg) */
#define  CA_NI_QM_INACTIVE_BID_CNT	GENMASK(13, 0)
#define CA_NI_RX_EQ_ID			5	/* build77: CPU_0 pool0 = EQ5 (RE of init_empty_buffer_CPU: EQ_PROFILE[2].eqp0=5; EQ13 was the WRONG deep-queue pool) */

/* --- QM block: RMU + empty-buffer-pool configuration.  NOTE the 07f QM
 *     config block was re-generated vs the public CA8277B map (per-EQ CFG
 *     moved 0x61b8 -> 0x6248, EQ profile 0x612c -> 0x6128, dest-port EQ cfg
 *     0x614c -> 0x6168, pkt-buf cfg 0x6190 -> 0x6228); identical bit
 *     layouts, all offsets below re-verified per-function in the 07f ko
 *     (aal_l3qm_enable_rx, aal_l3qm_dump_empty_buffers,
 *     aal_l3qm_init_empty_buffer, aal_l3qm_init_voq,
 *     aal_l3qm_check_cpu_push_ready, ca_ni_fill_eq_buf_pool). --- */
/* master L3QM RX enable; stock clears it around pool init, sets it last */
#define CA_NI_QM_RMU0_CTRL		0x6104
#define  CA_NI_QM_RMU0_RX_EN		BIT(31)
/* L3QM egress-scheduler control.  This is the drain side of the RX path: the
 * RMU0 above receives frames into pool buffers and enqueues a per-dest VOQ; the
 * egress scheduler here is what actually drains that VOQ and WRITES the 8-byte
 * descriptor into the CPU-EPP FIFO ring.  With it disabled, frames are received
 * (MAC RX MIB climbs) but no descriptor is ever appended -> hw wptr stays 0.
 * Stock enables it in two steps: ca_ni_init_l3qm -> aal_l3qm_enable_tx(1) sets
 * {tx_en, ni_en=0xff, inccfg_for_pkt_cntrs=2, inccfg_for_err_cntrs=6} (07f ko
 * @0x518f0, == aal-77c aal_l3qm_enable_tx), and ca_ni_open ->
 * aal_l3qm_enable_tx_cpu(cpu_port,1) ORs the per-CPU-port cpu_en bit (07f ko
 * @0x519e0).  Both write this same register; our M2c RX init previously did
 * neither, which is why the CPU-EPP ring never advanced in either steer mode. */
#define CA_NI_QM_ES_CTRL		0x6108
/* ★★ build67 (vendor source RE): the REAL L3QM egress-scheduler ctrl = QM_QM_ES_CTRL @
 * 0x7108 (aal_l3qm enable_tx / enable_tx_cpu) - NOT 0x6108 which we mislabeled es_ctrl.
 * enable_tx sets tx_en(b31)+ni_en(0xff)+cpu_en=0; enable_tx_cpu(port) ORs cpu_en|=(1<<port).
 * We never set cpu_en HERE -> the CPU-port ES drain never ran -> epp_wptr frozen.  Same
 * field layout as 0x6108. */
#define CA_NI_QM_ES_CTRL_REAL		0x7108
#define  CA_NI_QM_ES_CPU_EN		GENMASK(7, 0)	/* per-CPU-port egress   */
#define  CA_NI_QM_ES_NI_EN		GENMASK(15, 8)	/* per-NI-port egress    */
#define  CA_NI_QM_ES_INCCFG_PKT		GENMASK(18, 16)	/* stock value 2         */
#define  CA_NI_QM_ES_INCCFG_ERR		GENMASK(22, 20)	/* stock value 6         */
#define  CA_NI_QM_ES_RSVD25		BIT(25)		/* stock clears this     */
#define  CA_NI_QM_ES_TX_EN		BIT(31)		/* master egress enable  */
#define  CA_NI_QM_ES_INCCFG_PKT_VAL	2
#define  CA_NI_QM_ES_INCCFG_ERR_VAL	6
/* Golden live es_ctrl on a stock working-RX boot: tx_en + cpu_en=0xff (ALL
 * CPU ports, via aal_l3_tm_es_cpu_port_ena_set per port in ca_ni_open) +
 * ni_en=0xff + inccfg{pkt=2,err=6} + bit26, bit25 CLEAR = 0x8462FFFF.  Ours
 * came up 0x8662FF01: cpu_en only bit0 (we enabled one CPU port) and bit25
 * stray.  Match stock: enable ALL cpu_en bits + clear bit25. */
#define  CA_NI_QM_ES_CPU_EN_ALL		0xff	/* stock-Linux ES_CTRL=0x8462ffff (cpu_en=0xff) */

/* L3-TM egress-scheduler port enables (the SECOND egress-enable layer, above
 * the RMU es_ctrl).  aal_l3_tm_es_cpu_port_ena_set writes the cpu byte of
 * 0x6108; aal_l3_tm_es_ni_port_ena_set (07f ko @0x68ae0) writes 0x610c.
 * Stock leaves 0x610c = 0xffffffff (all NI egress ports enabled); ours never
 * wrote it (reset). */
#define CA_NI_QM_L3TM_NI_PORT_ENA	0x610c
#define  CA_NI_QM_L3TM_NI_PORT_ENA_ALL	0xffffffffu

/* QM block-init-done latch (stock aal_l3qm_check_init_done): the whole L3QM
 * delivery init must wait for this before touching the EQ/EPP config. */
#define CA_NI_QM_PHY_PORT_STS		0x6988	/* ELNATH PHY_PORT_STS (rtl 0x6850) */
#define  CA_NI_QM_INIT_DONE		BIT(30)	/* qm_init_done */
#define  CA_NI_QM_INIT_DONE_TIMEOUT_US	100000	/* bounded wait, non-fatal */

/* Second egress-scheduler control: ni_qm_hol_pkt_ctrl (bit1) gates the NI->QM
 * head-of-line handoff.  Stock aal_l3qm_init_empty_buffer sets it =1 via RMW; if
 * it stays 0 the NI/L2 side is never presented to the QM at head-of-line, so a
 * deep_q frame drained to ES port 8 never reaches RMU0 (0x6900 stuck 0).
 *
 * ★ THE REAL Elnath ES_CTRL2 IS 0x6a30, NOT 0x6ab0 (register-map shift bug):
 * tier-1 live stock reads 0x6a30=0x0A (ni_qm_hol bit1 = 1) while 0x6ab0=0x300 on
 * BOTH stock and ours (bit1=0) - so 0x6ab0 is a DIFFERENT reg (some fifo-thr /
 * status), and our driver set ES_CTRL2 to the wrong offset this whole time and
 * never set ni_qm_hol.  We keep the (harmless, stock-matching) 0x6ab0=0x300 write
 * and ADD the ni_qm_hol set at the real ES_CTRL2 (0x6a30). */
#define CA_NI_QM_ES_CTRL2		0x6ab0	/* NOT ES_CTRL2 - some fifo/status reg; stock=0x300, keep writing to match */
#define  CA_NI_QM_ES_CTRL2_NI_QM_HOL	BIT(1)	/* ni_qm_hol_pkt_ctrl */
#define  CA_NI_QM_ES_CTRL2_STOCK_VAL	0x00000300u	/* ELNATH stock live */
#define CA_NI_QM_ES_CTRL2_REAL		0x6a30	/* ★ the REAL Elnath ES_CTRL2 (ni_qm_hol@bit1); stock=0x0A */

/* CPU-EPP FIFO command mode: 0 = 32-bit descriptor, 1 = 64-bit.  Our NAPI
 * parses 64-bit descriptors (u64 / __le64 ring), so this MUST be 1 or the HW
 * writes a descriptor shape the poll routine misreads. */
#define CA_NI_QM_EPP			0x6a3c	/* ELNATH EPP (rtl 0x68f4) */
#define  CA_NI_QM_EPP_CMD_MODE_64	BIT(2)	/* cmd_mode: 1 = 64-bit */
/* ★ Egress-scheduler egress-enable pair (Elnath, tier-1 live-stock).  0x6a20 =
 * tx-path egress enable, 0x6a00 = CPU-path egress enable - each an 8-bit port mask
 * (0xff00), NOT a single bit.  Stock live: BOTH read 0x0000FF00.  This is the
 * aal_l3qm enable_tx (0x6a20) / enable_tx_cpu (0x6a00) pair for our chip.  Our
 * match_stock_qm writes 0x6a00=0xFF00 early, but it reads back 0 - the EPP engine
 * arming (0x6a3c GO) clears the CPU-path enable - so we re-assert BOTH last, after
 * the GO.  Without 0x6a00 the ES never services CPU_0 egress -> the CPU-EPP
 * writeback is starved -> the CPU ring keeps its DEADBEEF poison (PA=0, no RX). */
#define CA_NI_QM_EPP_TX_EGR_EN		0x6a20
#define CA_NI_QM_EPP_CPU_EGR_EN		0x6a00
#define  CA_NI_QM_EPP_EGR_EN_ALL	0x0000FF00u
/* EQ-config commit (stock aal_l3qm_load_eq_config, 07f ko @0x4f270): after
 * programming the per-EQ CFG0-4, unlock HDM write-protection, pulse
 * EQ_CFG_LOAD (all EQs), relock.  This LATCHES the bid range into the
 * empty-buffer manager - without it the pushed PAs never leave the shallow
 * push stage (pool caps at ~4, ready bit stuck low). */
/* L3QM RX-side status + moved-frame counters (read-only), recovered from
 * aal_l3qm_get_rx_status0/1 (07f ko @0x4f110/0x4f150) and
 * aal_l3qm_get_tx_rx_cntr (@0x4e5b0).  The RX counter is the most decisive
 * gate probe: on a working boot it climbs as the RMU moves ingress frames
 * into pool buffers; if it stays 0 on a broken boot, frames never reached
 * the RMU (fault is at/before the MAC->RMU handoff, not the EPP drain). */
#define CA_NI_QM_RX_STATUS0		0x66c4
#define CA_NI_QM_RX_STATUS1		0x66c8
#define CA_NI_QM_RX_CNTR		0x6900	/* ELNATH RMU0_RX_PKT_CNTR (real admission) */
/* build69: the admitted-frame header - dest ldpid [7:0] + deep_q flag (bit30) + valid
 * (bit31).  Stock CPU-RX = 0x80000010 (dest 0x10=CPU0, deep_q CLEAR -> CPU-EPP64 ring).
 * Ours pre-build69 = 0xc0000020 (dest 0x20=CPU_MQ_0 + bit30 deep_q -> CPU-EPP256 ring). */
#define CA_NI_QM_RMU0_RX_HDR_INFO0	0x6904
#define  CA_NI_QM_RMU0_RX_DEST_LDPID	GENMASK(7, 0)
#define  CA_NI_QM_RMU0_RX_DEEP_Q	BIT(30)
#define  CA_NI_QM_RMU0_RX_VALID		BIT(31)
#define CA_NI_QM_TX_CNTR		0x690c	/* ELNATH TX_PKT_CNTR (dequeue) */
/* RMU front-end drop counter (stock aal_l3qm_dump_rmu_fe_drop_counter @0x65a4):
 * increments when a frame reaches RMU0 but its size-selected target pool has no
 * free buffer to admit into.  THE witness that partitions "frame never reached
 * RMU" (0x6900=0 AND 0x6944=0) from "reached RMU, pool empty -> dropped"
 * (0x6900=0, 0x6944 climbs under ping). */
#define CA_NI_QM_RMU_FE_DROP		0x6944	/* ELNATH RMU0 FE-drop cntr */
#define CA_NI_QM_RMU_NO_BUF_DROP	0x6940	/* RMU0_NO_BUF_DROP_PKT_CNTR (cumulative, 16-bit) */
/* ★★ build43: RE a053902d's QM/RMU0-side counters - the RIGHT bisect.  0xa9fc may count
 * the WRONG direction (L3QM->NI_HV egress), so it can stay 0 on a WORKING path; and the
 * BM->L3QM accept has NO enable bit - the "credit" is an available EMPTY BUFFER (EQ pool).
 * The operator's recorded M2 wall = EQ13 (128B CPU pool) unseeded -> a small (60B ARP ->
 * 128B class) frame reaches RMU0 but finds no buffer -> silently not admitted, no upstream
 * drop.  These RE offsets differ from our Elnath remaps (0x6900/0x6940) - exposed ALONGSIDE
 * to see which are live.  If no_buf climbs under flood while rmu_rx=0 = EQ13-empty = the fix. */
#define CA_NI_QM_RMU0_RX_PKT_CNTR_RE	0x67d8	/* RE: RMU0_RX_PKT_CNTR (reached QM/RMU0) */
#define CA_NI_QM_RMU0_NO_BUF_DROP_RE	0x6818	/* RE: RMU0_NO_BUF_DROP (pool empty) */
#define CA_NI_QM_EQ13_BUF_USG		0x695c	/* RE: EQ13 buffer-usage (0 = unseeded/empty) */
#define CA_NI_QM_CPU_PUSH_RDY0_RE	0x6368	/* RE: CPU_PUSH_RDY0 (rtl offset) */
#define CA_NI_QM_EQ_STACK_UNFILL	0x63c0	/* RE: EQ_STACK_UNFILL */
/* ★★ PROVEN-cumulative per-stage packet counters (RE a0668fdf, disasm-classified as
 * per-packet running totals, NOT depth/latch): idle-vs-ping each; the first that does
 * not climb under ping = the death stage.  All in the NI-core window (ni_base).
 * stage1 L2TM egress = existing CA_NI_L2TM_BM_TX_PCNT (0x2140, already climbs). */
#define CA_NI_NI_L3QM_RX_PKT_CNT	0xa9fc	/* stage2 QM ingress: NI_HV RX_PKT_CNT_L3QM */
#define CA_NI_NI_L3QM_TX_PKT_CNT	0xaa10	/* stage2 QM ingress: NI_HV TX_PKT_CNT_L3QM (NI->QM) */
/* ★★ build37 TIE-BREAKER (RE a053902d): the NI_HV L3QM interface has sibling error
 * counters just below RX_PKT_CNT.  If RX_PKT_CNT(0xa9fc)=0 but one of these climbs, the
 * frame REACHED the L3QM NI_HV ingress but was rejected at presentation (malformed/short)
 * = egressed-but-lost; if all three are 0, it never arrived (lost upstream at BM egress ->
 * L3QM routing/ES).  hi16/lo16 = the two sub-reasons per reg. */
#define CA_NI_NI_L3QM_RX_MISS_SOP_EOP	0xa9f4	/* RX_MISSING_SOP_EOP (hi16=SOP, lo16=EOP) */
#define CA_NI_NI_L3QM_RX_SHORT_ERR	0xa9f8	/* RX_SHORT_ERR (hi16=short, lo16=err) */
/* ★★ build42 diagnostic: the NI_HV per-interface RX_PKT_CNT block (stride 0x40; RE
 * a053902d: +0x00=L3FE, +0x40=L3QM, +0x80=MCE, +0xc0=DMA).  L3QM=0xa9fc.  Both deep_q
 * paths dequeue (bm_tx +9) but L3QM RX (0xa9fc) stays 0 - so expose the SIBLING interface
 * counters to see WHICH interface the dequeue actually lands on: if L3FE (0xa9bc) climbs
 * instead of L3QM, the frame is presented to the L3FE interface (which our driver does not
 * drain) not L3QM = the mis-route.  If NONE climb, the dequeue never reaches NI_HV at all. */
#define CA_NI_NI_L3FE_RX_PKT_CNT	0xa9bc	/* NI_HV interface +0x00 = L3FE */
#define CA_NI_NI_MCE_RX_PKT_CNT		0xaa3c	/* NI_HV interface +0x80 = MCE */
#define CA_NI_NI_DMA_RX_PKT_CNT		0xaa7c	/* NI_HV interface +0xc0 = DMA */
/* stage3 = 0x6900 (rx, operator-flagged suspect) / 0x690c (sched); stage4 = wptr 0x7000 */

#define CA_NI_QM_HDM_WRITE_PROT		0x67fc	/* ELNATH HDM_WRITE_PROTECTION (rtl 0x66d4) */
#define  CA_NI_QM_HDM_UNLOCK		0x05102013u
#define CA_NI_QM_EQ_CFG_LOAD		0x6408	/* ELNATH EQ_CFG_LOAD (rtl 0x63a8) */
#define  CA_NI_QM_EQ_CFG_LOAD_ALL	0xffffu
/* EQ profile n (0..7): pool0/pool1 EQ ids + fill rule (0 = SW push) */
#define CA_NI_QM_EQ_PROFILE(n)		(0x6128 + (n) * 4)
#define  CA_NI_QM_EQ_PROF_EQP0		GENMASK(3, 0)
#define  CA_NI_QM_EQ_PROF_EQP1		GENMASK(7, 4)
#define  CA_NI_QM_EQ_PROF_RULE		BIT(31)
/* Stock CPU EQ profile = {eqp0 = pool0 EQ id 13, eqp1 = pool1 EQ id 14, rule=0}.
 * ★ ELNATH stock: the CPU dest (9) selects EQ profile 13 (DESTPORT map = 0x0D),
 * so we configure EQ_PROFILE(13).  (The EPP-FIFO profile is a SEPARATE 3-bit
 * space - CA_NI_RX_PROFILE_ID stays 4.)  Both EQ13 + EQ14 must be active pools. */
#define CA_NI_RX_EQ_PROFILE		0x0d	/* ELNATH stock: CPU dest-port profile 13 */
/* Stock EQ_PROFILE(13) = {eqp0=EQ13 (128B small), eqp1=EQ14 (2048B large)} = 0xED.
 * The RMU size-selects: a frame that fits eqp0's 128B goes to EQ13, else EQ14.  Both
 * pools are populated; eqp0.size < eqp1.size is REQUIRED for the size-match (a prior
 * {EQ14,EQ14} workaround made them equal and RMU0 silently refused to admit). */
#define CA_NI_RX_EQ_PROFILE_VAL \
	(FIELD_PREP(CA_NI_QM_EQ_PROF_EQP0, CA_NI_RX_EQ_ID) | \
	 FIELD_PREP(CA_NI_QM_EQ_PROF_EQP1, CA_NI_RX_EQ_ID2))
/* ★★ FIELD-WIDTH FIX: DEST_PORT_EQ_CFG.profile_sel is only 3 bits [2:0] (max 7),
 * and EQ_PROFILE has 8 entries (0-7).  Our DEST_PORT value 0x0d therefore selects
 * profile_sel = 0x0d & 0x7 = 5, NOT 13 - but we configured EQ_PROFILE(13) (out of
 * the DEST_PORT-reachable range).  So a deep_q frame selects the UNCONFIGURED
 * EQ_PROFILE(5) -> wrong/empty EQ -> RMU has no buffer -> no admission (0x6900=0).
 * Fix: ALSO program EQ_PROFILE(5) = {EQ13,EQ14} = the profile the 3-bit field
 * actually selects.  (We keep the EQ_PROFILE(13) write too, harmlessly, in case
 * the Elnath field is wider.)  The vendor sets DEST_PORT.profile_sel == the
 * EQ_PROFILE index it configures - ours diverged by the [3:0]-vs-[2:0] misread. */
#define CA_NI_RX_EQ_PROFILE_SEL		(CA_NI_RX_EQ_PROFILE & 0x7)	/* = 5, the reachable profile */
/* ★ The CPU physical-dest port in the QM DEST_PORT_EQ_CFG table is PHYSICAL
 * DEST 0x09 (AAL_PPORT_CPU), which sits at DEST_PORT index 15 (0x6168+15*4 =
 * 0x61a4 - stock-devmem confirmed: [0x61a4]=0x0D=EQ-profile 13).  Our old
 * "dest-port 8" (0x6188) was physical-dest-2, NOT the CPU = why frames vanished.
 * We point index 15 at our EQ profile (-> EQ13/EQ14 CPU pools). */
#define CA_NI_RX_CPU_DEST_PORT		15
/* ★ deep-queue frames occupy QM DEST_PORT_EQ_CFG indices 8..15 (8 + DeepQ index);
 * we point them all at our CPU EQ profile so admission finds EQ13 whichever the
 * QM VOQ picks (index 15 = the stock CPU slot CA_NI_RX_CPU_DEST_PORT). */
#define CA_NI_RX_DEEPQ_DEST_PORT_LO	8
#define CA_NI_RX_DEEPQ_DEST_PORT_HI	15
/* dest-port (index = ldpid - 0x10; CPU port 0 = 0) -> EQ-profile select */
/* per-dest-port EQ-profile select: authoritative base QM_QM_DEST_PORT0_EQ_CFG
 * = ELNATH 0x6168 (rtl8277c 0x6148), stride 4, 32 entries.  CPU dest port = 8,
 * so the CPU entry is 0x6168 + 8*4 = 0x6188.  (The old 0x6148 base put the CPU
 * entry at 0x6168 = Elnath dest-port-0 - wrong; the full-driver offset audit
 * against arch-elnath/registers.h corrected the whole QM block.) */
#define CA_NI_QM_DEST_PORT_EQ_CFG(p)	(0x6168 + (p) * 4)	/* ELNATH (rtl 0x6148) */
#define  CA_NI_QM_DEST_PORT_PROF_SEL	GENMASK(3, 0)
/* ★ Global-default EQ-profile (3 entries @0x6200/0x6204/0x6208): the EQ profile a
 * dest with no specific DEST_PORT_EQ_CFG entry uses.  Stock = 0x0D (profile 13) each
 * - this is what steers a CPU-dest (physical 0x09) frame to the CPU pool EQ13/EQ14. */
#define CA_NI_QM_EQ_PROFILE_GLOBAL(n)	(0x6200 + (n) * 4)
#define CA_NI_QM_EQ_PROFILE_GLOBAL_COUNT 3
/* CPU EQ-profile selector value = the CPU pool's EQ_PROFILE index (13) */
#define CA_NI_RX_CPU_PROFILE_VAL	CA_NI_RX_EQ_PROFILE	/* 0x0D -> EQ_PROFILE(13) */
/* dest-port head/tail room in 16-byte units; head_room_first = 4 is what
 * puts HEADER_A at buffer+0x40 - it is PROGRAMMED, not innate */
/* authoritative base QM_QM_DEST_PORT0_PKT_BUF_CFG = 0x61c8, stride 4, only 8
 * entries (indexed by CPU-port number, not the 0..31 dest-port number) */
#define CA_NI_QM_DEST_PORT_PKT_BUF_CFG(p) (0x6228 + (p) * 4)	/* ELNATH (rtl 0x61c8) */
#define  CA_NI_QM_PKT_BUF_HEAD_FIRST	GENMASK(5, 0)
#define  CA_NI_QM_PKT_BUF_TAIL_FIRST	GENMASK(13, 8)
#define  CA_NI_QM_PKT_BUF_HEAD_REST	GENMASK(21, 16)
#define  CA_NI_QM_PKT_BUF_TAIL_REST	GENMASK(29, 24)
/* stock CPU-port pkt_buf golden = 0x18041804: head_first=head_rest=4 (16B
 * units -> HEADER_A at +0x40, same as before) AND tail_first=tail_rest=0x18
 * (24 * 16 = 384B reserved tail).  Ours was 0x00040004 (no tail room). */
#define  CA_NI_QM_PKT_BUF_HEAD_UNITS	(CA_NI_RX_BUF_HEADROOM / 16)	/* 4 */
#define  CA_NI_QM_PKT_BUF_TAIL_UNITS	0x18				/* 24 */
/* per-EQ config, 16 EQs, stride 0x14 */
/* per-EQ CFG0..4: ELNATH base QM_QM_CFG0_EQ0 = 0x6248 (rtl8277c 0x61e8), stride
 * 20 (0x14), count 16.  EQ8 CFG0 = 0x6248 + 8*0x14 = 0x62e8.  (A prior session
 * mis-moved this to 0x61e8 believing 0x6248 was a chipdef error - backwards; the
 * offset audit vs arch-elnath/registers.h confirms 0x6248.  With the wrong base
 * EQ8's pool never activated = the CPU-RX-delivery root cause.) */
#define CA_NI_QM_CFG0_EQ(e)		(0x6248 + (e) * 0x14)	/* ELNATH (rtl 0x61e8) */
#define  CA_NI_QM_CFG0_EQ_EN		BIT(0)
#define  CA_NI_QM_CFG0_PHY_ADDR_START	GENMASK(31, 7)	/* 0 for SW-push pools */
#define CA_NI_QM_CFG1_EQ(e)		(0x624c + (e) * 0x14)	/* ELNATH (rtl 0x61ec) */
#define  CA_NI_QM_CFG1_BID_START	GENMASK(13, 0)
#define  CA_NI_QM_CFG1_TOTAL_BUF_NUM	GENMASK(29, 16)
#define CA_NI_QM_CFG2_EQ(e)		(0x6250 + (e) * 0x14)	/* ELNATH (rtl 0x61f0) */
#define  CA_NI_QM_CFG2_BUF_SIZE	GENMASK(2, 0)	/* size index, see below */
#define  CA_NI_QM_CFG2_CPU_EQ		BIT(3)
#define  CA_NI_QM_CFG2_REFILL_EQID	GENMASK(6, 4)
#define  CA_NI_QM_CFG2_REFILL_EN	BIT(7)
#define  CA_NI_QM_CFG2_REFILL_THS	GENMASK(15, 8)
/* CFG3 = per-EQ AXI cache/snoop attributes (cache_eos/domain/snoop/cache/qos).
 * Tier-1 devmem of the LIVE CPU pools (EQ13/EQ14) reads 0x00000010 - use that,
 * not the earlier 0x1a008017 guess (which was the EQ8/EQ9 assumption, never the
 * live CPU-pool value). */
#define CA_NI_QM_CFG3_EQ(e)		(0x6254 + (e) * 0x14)	/* ELNATH (rtl 0x61f4); AXI attrs */
#define  CA_NI_QM_CFG3_CPU_POOL_VAL	0x00000010u	/* stock EQ13/EQ14 CFG3 (tier-1) */
#define CA_NI_QM_CFG4_EQ(e)		(0x6258 + (e) * 0x14)	/* ELNATH (rtl 0x61f8); AXI top bits */
#define CA_NI_QM_EQ_COUNT		16
/* buffer-size index (stock aal_l3qm_get_buffer_size_index):
 * 0x80=0 0x100=1 0x200=2 0x400=3 0x800=4 0x1000=5 0x2000=6 */
#define CA_NI_QM_BUF_SIZE_IDX_2048	4	/* idx 4 -> 0x800 = 2048B stride */
/* Two CPU empty-buffer pools (stock aal_l3qm_init_empty_buffer_CPU), tier-1
 * devmem values for THIS board:
 *  - EQ13 = pool0/PRIMARY (profile eqp0), NAPI keeps it topped one-for-one;
 *  - EQ14 = pool1/SECONDARY (profile eqp1), overflow reserve.
 * Both are CPU-push pools (refill_en=0): WE push DDR buffer PAs via
 * CPU_PUSH_PADDR - there is no FBM auto-refill.  The seed count into a pool
 * MUST be <= its CFG1 total_buf, and the bid windows must not overlap: stock's
 * EQ13 [0x5b0,0x5b0+383) is contiguous with EQ14 [0x72f,0x72f+561).  These bid
 * windows MUST equal what cortina_ni_rx_eq_cfg_pool writes into CFG1 (it builds
 * CFG1 from these same constants), or the pushed BIDs miss the pool window. */
#define CA_NI_RX_EQ_ID2			6	/* build77: CPU_0 pool1 = EQ6 (EQ_PROFILE[2].eqp1=6) */
#define CA_NI_RX_EQ_BID_START		0x1200	/* build77: EQ5 (pool0) bid_start = 4608 (stock CPU pool0) */
#define CA_NI_RX_EQ_TOTAL_BUF		512	/* EQ5 (pool0) buffers pushed (<= stock 1500; plenty for ARP) */
#define CA_NI_RX_EQ2_TOTAL_BUF		512	/* EQ6 (pool1) buffers pushed */
#define CA_NI_RX_EQ2_BID_START		0x17dc	/* build77: EQ6 (pool1) bid_start = 6108 (stock CPU pool1; NOT contiguous - matches stock ranges) */
/* ★★★ CPU-pool DRAM layout (cpu_eq=0, software-populated).  ONE dma_alloc_coherent
 * region holds both CPU pools back-to-back: EQ13 (pool0, 128B buffers) at offset 0,
 * EQ14 (pool1, 2048B buffers) right after.  CFG0.phy_addr_start of each pool points
 * at its sub-region; cortina_ni_rx_populate pushes each buffer PA.  buffer_size
 * MUST match CFG2 and stock (EQ13=idx0=128B, EQ14=idx4=2048B) - unequal sizes are
 * required for the RMU's eqp0-small/eqp1-large size-select. */
#define CA_NI_RX_CPU_POOL0_BUFSZ	2048u	/* build77: EQ5 buffer_size idx5 (2048B, stock CPU pool) */
#define CA_NI_RX_CPU_POOL1_BUFSZ	2048u	/* EQ6 buffer_size idx5 (2048B) */
#define CA_NI_RX_CPU_POOL0_BYTES \
	(CA_NI_RX_CPU_POOL0_BUFSZ * CA_NI_RX_EQ_TOTAL_BUF)	/* 128*128 = 16KB */
#define CA_NI_RX_CPU_POOL1_BYTES \
	(CA_NI_RX_CPU_POOL1_BUFSZ * CA_NI_RX_EQ2_TOTAL_BUF)	/* 512*2048 = 1MB */
#define CA_NI_RX_CPU_DRAM_SIZE \
	(CA_NI_RX_CPU_POOL0_BYTES + CA_NI_RX_CPU_POOL1_BYTES)	/* ~1.02MB */
/* ★ EQ8 = the RMU's empty-buffer ALLOC pool (dest-9->profile-13->EQ13 queue, but
 * the RMU allocates a free buffer from EQ8 to admit a frame).  Brought up as a
 * CPU-PUSH pool - reuse EQ13's CFG0 (enable) + CFG2=0xff00 (refill_en=0, NO FBM,
 * so no hanging AXI alloc, unlike stock's 0x66ec) - bid window after EQ14. */
#define CA_NI_RX_EQ8_ID			8	/* Elnath RMU empty-buffer alloc pool */
#define CA_NI_RX_EQ8_TOTAL_BUF		383	/* seeded free buffers (stock pa_req ~62) */
#define CA_NI_RX_EQ8_BID_START \
	(CA_NI_RX_EQ2_BID_START + CA_NI_RX_EQ2_TOTAL_BUF)	/* 0x960, after EQ14 */
/* ★★ EQ12 = the DEEP-QUEUE CPU pool.  Tier-1 stock: a deep_q=1 frame egresses ES
 * port 8/9 -> EQ_PROFILE(12)=0xEC {eqp0=12,eqp1=14} -> EQ12 buffer -> RMU -> CPU.
 * Stock EQ12: cfg0=0x80000081, cfg1=0x010004b0 (total=256, bid=0x4b0 contiguous
 * just BELOW EQ13's 0x5b0), cfg2=0xFF02 (refill_en=0, FBM-SAFE).  Our driver is
 * CPU-push, so (like EQ13/14) we use cfg2=0xff0d (cpu_eq=1, buffer_size=5=2048B,
 * refill_ths=0xff, refill_en=0) and CPU-push-seed its 256 skbs.  This is the
 * pool the deep_q frame draws from - pointing it at EQ13 (our old profile-13 bug)
 * never admitted; stock uses a SEPARATE deep-queue pool (EQ12). */
#define CA_NI_RX_EQ12_ID		12
#define CA_NI_RX_EQ12_TOTAL_BUF		256	/* stock cfg1 total_buf = 0x100 */
#define CA_NI_RX_EQ12_BID_START		0x4b0	/* stock; 0x4b0+256 = 0x5b0 = EQ13 start */
/* ★★★ EQ12 is now a cpu_eq=0 DRAM-BACKED pool (stock cfg2=0xFF02, cpu_eq=0).  The
 * A/B bisect proved the CB deep-queue enqueue needs a DRAM buffer, NOT our
 * CPU-push skbs.  We dma_alloc_coherent a region; CFG0.phy_addr_start = its phys,
 * cpu_eq=0 => the HW AUTO-POPULATES 256 buffers over the region (no CPU push).
 * cfg0 is computed from the DMA handle at init; cfg2 = stock verbatim. */
#define  CA_NI_QM_EQ12_CFG2		0x0000ff02u	/* stock: cpu_eq=0, buffer_size=2, refill_ths=0xff */
#define CA_NI_RX_DQ_DRAM_SIZE		(256u * 2048u)	/* 256 buf * 2048B (over-reserve) = 512KB */
/* deep-queue EQ profile 12 = {eqp0=EQ12, eqp1=EQ14} = 0xEC (stock 0x6158) */
#define CA_NI_RX_EQ12_PROFILE		12
#define CA_NI_RX_EQ12_PROFILE_VAL \
	(FIELD_PREP(CA_NI_QM_EQ_PROF_EQP0, CA_NI_RX_EQ12_ID) | \
	 FIELD_PREP(CA_NI_QM_EQ_PROF_EQP1, CA_NI_RX_EQ_ID2))
/* per-scheduler VOQ enable = QM_QM_SCH_CFG0.voq_en[7:0]; ELNATH base 0x6424
 * (rtl8277c 0x63c4), stride 4 (stock init_voq ORs 0xff into ALL of them).
 * (A prior session mis-moved this to 0x63c4 calling 0x6424 a chipdef error -
 * backwards; the audit vs arch-elnath confirms 0x6424.) */
#define CA_NI_QM_VOQ_EN(p)		(0x6424 + (p) * 4)	/* ELNATH (rtl 0x63c4) */
#define  CA_NI_QM_VOQ_EN_ALL		GENMASK(7, 0)
#define CA_NI_QM_VOQ_EN_COUNT		32

/* --- QM block: CPU-EPP 64-bit descriptor FIFO ring pointers.
 *     Index = 4 * (8 * cpu_port + voq); pointers are BYTE offsets into the
 *     ring, 24-bit meaningful, wrapping at the ring size (0x400). --- */
#define CA_NI_QM_EPP64_WRPTR(p, q)	(0x7000 + 4 * (8 * (p) + (q)))
#define CA_NI_QM_EPP64_RDPTR(p, q)	(0x7100 + 4 * (8 * (p) + (q)))
#define CA_NI_QM_EPP64_PADDR_START(p, q) (0x7200 + 4 * (8 * (p) + (q)))
/* ★★★ build80: the SECOND per-voq CPU-EPP64 ring buffer (PADDR_HI @0x7220), = PADDR +
 * 0x2000.  Stock live capture: without PADDR_HI the writeback engine writes ZERO
 * descriptors (wptr advances but every slot stays poison/0).  Per-voq, same 0x400
 * stride as PADDR: voq q -> 0x7220+q*4 = ring_base+0x2000+q*0x400. */
#define CA_NI_QM_EPP64_PADDR_HI(p, q)	(0x7220 + 4 * (8 * (p) + (q)))
#define CA_NI_RX_RING_HI_OFFSET		0x2000u
#define  CA_NI_QM_EPP64_PTR		GENMASK(23, 0)

/* ring geometry: 128 descriptors x 8 bytes, byte pointers wrap at 0x400 */
#define CA_NI_RX_EPP_PER_VOQ		128
#define CA_NI_RX_DESC_SIZE		8
#define CA_NI_RX_RING_BYTES		(CA_NI_RX_EPP_PER_VOQ * \
					 CA_NI_RX_DESC_SIZE)	/* 0x400 per voq */
/* ★★ ALL 8 VOQs of the CPU port get their own CPU-EPP FIFO ring (stock
 * aal_l3qm_init_cpu_epp arms every voq).  We armed only voq0, so if the deep_q
 * frame lands on a voq != 0 (by cos/priority) the RMU has no armed ring/FIFO for
 * it and cannot admit (0x6900=0).  8 per-voq rings laid out contiguously in the
 * reserved DDR region; ring[voq*128 + slot].  NAPI drains all 8. */
#define CA_NI_RX_VOQ_COUNT		8
/* build80: 0x4000 = the LOW PADDR ring (0x2000) + the HIGH PADDR_HI ring (0x2000).
 * The memremap + poison must cover BOTH so the PADDR_HI buffers are valid poisoned DDR. */
#define CA_NI_RX_RING_TOTAL_BYTES	(2 * CA_NI_RX_VOQ_COUNT * CA_NI_RX_RING_BYTES) /* 0x4000 */
#define CA_NI_RX_RING_SLOTS_PER_VOQ	CA_NI_RX_EPP_PER_VOQ	/* __le64 slots per voq */
/* RMU0 drop counters (Elnath, from the 0x6900 anchor) - bisect: climb => frame
 * reaches RMU0 but no buffer for its voq; 0 => frame never arrives. */
#define CA_NI_QM_RMU0_NO_BUF_DROP	0x6940
#define CA_NI_QM_RMU0_FE_DROP		0x6944

/* --- 64-bit EPP RX descriptor (little-endian, in the coherent ring).
 *     Stock CPU_EPP_FIFO_CMD 64-bit layout, verified in
 *     ca_ni_rx_napi_get_header_from_64bit_epp: {eqid[3:0], sop bit4,
 *     l4_csum_err bit5, eop bit6, phy_addr[31:7], pkt_size[45:32],
 *     sw_id[51:46]}.  sw_id == 0 is the NORMAL frame (HEADER_A at +0x40);
 *     sw_id != 0 is the header-LESS format (stock WiFi fast-forward):
 *     frame at +0x10, length = pkt_size - 0x10.  Stock aborts on a
 *     descriptor without SOP. --- */
#define CA_NI_RX_DESC_PA		0xffffff80u	/* of the low word */
#define CA_NI_RX_DESC_EQID		GENMASK(3, 0)
#define CA_NI_RX_DESC_SOP		BIT(4)
#define CA_NI_RX_DESC_CSUM_ERR		BIT(5)
#define CA_NI_RX_DESC_EOP		BIT(6)
#define CA_NI_RX_DESC_LEN		GENMASK_ULL(45, 32)
#define CA_NI_RX_DESC_SWID		GENMASK_ULL(51, 46)
#define CA_NI_RX_DESC_HDR_LEN		0x10	/* sw_id != 0: data off/len adj */

/* --- RX buffer layout (buffer base = the pushed, 128B-aligned PA) ---
 *   [0x00..0x3f] 64B SW headroom (QM pkt-buf-cfg head_room_first = 4 x 16B)
 *   [0x40..0x47] HEADER_A - two 32-bit BIG-ENDIAN words (stock byte-swaps
 *                both words before extracting any field)
 *   [0x48......] HEADER_CPU (8B, only if HEADER_A.cpu_flg) then the frame  */
#define CA_NI_RX_BUF_HEADROOM		64
#define CA_NI_RX_HDRA_OFF		0x40
#define CA_NI_RX_FRAME_OFF		0x48
/* word 0 = get_unaligned_be32(buf + 0x40): the pkt_info word */
#define  CA_NI_HDRA_W0_CPU_FLG		BIT(31)	/* HEADER_CPU present */
#define  CA_NI_HDRA_W0_DEEP_Q		BIT(30)
/* word 1 = get_unaligned_be32(buf + 0x44): size/port word */
#define  CA_NI_HDRA_W1_PKT_SIZE		GENMASK(28, 15)
#define  CA_NI_HDRA_W1_LSPID		GENMASK(14, 9)
#define  CA_NI_HDRA_W1_LDPID		GENMASK(8, 3)
#define  CA_NI_HDRA_W1_COS		GENMASK(2, 0)
#define CA_NI_RX_HDR_CPU_LEN		8

/* buffer size pushed to the pool (stock CA_NI_SKB_ALLOC_DATA_SIZE:
 * 2048 frame + 64 headroom + 16 headers) and required PA alignment */
#define CA_NI_RX_BUF_SIZE		2128
#define CA_NI_RX_BUF_ALIGN		128

/* --- NI_HV per-port RX control: drop policy + the FE-bypass RX steer.
 *     rxctrl_byp_en + byp_dpid = "ingress frames of this port bypass the
 *     forwarding engine and go straight to CPU port <dpid>" - the RX twin
 *     of the TX CA_DMA_LSO_VP_HDRA_CFG febypass. Reset value 0x08000400. --- */
#define CA_NI_PORT_RX_CNTRL_CFG(p)	(0xa5f8 + (p) * CA_NI_PORT_STRIDE)
#define  CA_NI_RX_CNTRL_FF_OVR_DROP_DIS	BIT(2)
#define  CA_NI_RX_CNTRL_CRC_DROP_DIS	BIT(4)
#define  CA_NI_RX_CNTRL_OVR_DROP_DIS	BIT(5)
#define  CA_NI_RX_CNTRL_RUNT_DROP_DIS	BIT(6)
#define  CA_NI_RX_CNTRL_FLOW_DROP_DIS	BIT(7)
#define  CA_NI_RX_CNTRL_UKOP_DROP_DIS	BIT(8)
#define  CA_NI_RX_CNTRL_OAM_DROP_DIS	BIT(9)
#define  CA_NI_RX_CNTRL_LNKSTAT_DROP_DIS BIT(10)
#define  CA_NI_RX_CNTRL_PFC_DROP_DIS	BIT(11)
#define  CA_NI_RX_CNTRL_BYP_DPID	GENMASK(21, 16)
#define  CA_NI_RX_CNTRL_BYP_COS		GENMASK(26, 24)
#define  CA_NI_RX_CNTRL_BYP_DPQ		BIT(27)
#define  CA_NI_RX_CNTRL_BYP_EN		BIT(28)
#define  CA_NI_RX_CNTRL_BYP_MIR		BIT(30)
#define  CA_NI_RX_CNTRL_BYP_NDRP	BIT(31)

/* CPU port 0 logical destination port id (AAL_LPORT_CPU_0) */
#define CA_NI_LDPID_CPU0		0x10

/* --- NI-RX L3FE demux routing map (stock aal_ni_init_nirx_l3fe_demux, 07f
 *     ko @0x6030, called from aal_ni_init_ni).  These registers map the
 *     forwarding-engine's per-{ingress, traffic-class} demux output to the
 *     L3QM CPU/NI egress slots.  Stock builds them from the board scfg; the
 *     values below are the LIVE golden read from a stock working-RX boot
 *     (tier-1 devmem, ni_base+off).  Without them the FE forwards a frame to
 *     "CPU0" logically but the L3QM never enqueues it to our EQ8/CPU-EPP ring
 *     -> hw wptr stays 0 (exactly our broken-boot symptom).  0xa1a0 =
 *     0x22AA0000 matches the disasm's bfi-2-at-{16,18,20,22,24,28} exactly,
 *     cross-confirming the capture. --- */
#define CA_NI_NIRX_L3FE_DEMUX0		0xa190
#define  CA_NI_NIRX_L3FE_DEMUX0_VAL	0x040c2040u
#define CA_NI_NIRX_L3FE_DEMUX1		0xa194
#define  CA_NI_NIRX_L3FE_DEMUX1_VAL	0x00007185u
#define CA_NI_NIRX_L3FE_DEMUX2		0xa198
#define  CA_NI_NIRX_L3FE_DEMUX2_VAL	0x00000000u
#define CA_NI_NIRX_L3FE_DEMUX3		0xa19c
#define  CA_NI_NIRX_L3FE_DEMUX3_VAL	0x00000002u
#define CA_NI_NIRX_L3FE_DEMUX4		0xa1a0
#define  CA_NI_NIRX_L3FE_DEMUX4_VAL	0x22aa0000u
#define CA_NI_NIRX_L3FE_DEMUX5		0xa1a4
#define  CA_NI_NIRX_L3FE_DEMUX5_VAL	0x00000000u

/* ★ build34: the DEEP-QUEUE (deep_q=1) per-ldpid L3FE demux table - a parallel copy
 * of the normal per-ldpid table (DEMUX2..5 = 0xa198..0xa1a4) at +0x10.  Each ldpid
 * owns a 2-bit demux_id (0=L3QM, 1=WAN, 2=L2FE, 3=L2TM); reg = 0xa1b4 - 4*(ldpid>>4),
 * field = bits[2*(ldpid&15)+1 : 2*(ldpid&15)].  A deep_q=1 frame is routed by THIS
 * table, NOT the normal one.  Our driver never wrote it -> the CPU frame (deep_q=1,
 * ldpid 0x32 -> 0xa1a8 bits[5:4]) used its reset demux_id and was steered off L3QM
 * (l2tm_tx climbed, ni2qm_rx/0xa9fc flat).  Stock (RE aal_ni_init_nirx_l3fe_demux):
 * 0xa1a8=0 (ldpid 0x32 -> L3QM), 0xa1b0 ldpid 0x19 -> L2FE - both mirror the normal
 * table, so we write the same VALs. */
#define CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63	0xa1a8		/* deep_q, ldpid 48-63 (incl CPU 0x32) */
#define CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47	0xa1ac		/* deep_q, ldpid 32-47 */
#define CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31	0xa1b0		/* deep_q, ldpid 16-31 (incl LAN 0x19) */
#define CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15	0xa1b4		/* deep_q, ldpid 0-15 */
/* ★ build36: EXPLICIT stock live DPQ demux values - the DPQ table is NOT a mirror of
 * the normal table (0xa1b4 stock=0xAAAA0000 vs normal 0xa1a4=0).  build34 mirror-wrote
 * 0xa1b4=0, corrupting ldpid 8-15 (b16-31: 0xAAAA=all L2FE) on the deep-q path (the CPU
 * frame ldpid 0x32 in 0xa1a8=0 was unaffected).  Restore to stock. */
#define  CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63_VAL	0x00000000u	/* ldpid 0x32 -> demux_id 0 = L3QM */
#define  CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47_VAL	0x00000000u	/* ★ stock 0xa1ac=0 (tier-1 live broadcast diff 2026-07-12: the SOLE NIRX-demux divergence). The old 0x2 was copied from the normal-table DEMUX3 (0xa19c=0x2, correct) into the DPQ table where stock leaves it 0; that 0x2 diverted port0 deep-queue frames OFF the L3FE classifier -> l3fe_rx(0xa9bc)=0 -> CLS never sees the frame -> null EPP descriptors. Stock=0 lets the frame reach L3FE->CLS->CPU-trap. */
#define  CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31_VAL	0x22AA0000u	/* ldpid 0x19 -> L2FE (stock live) */
#define  CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15_VAL	0xAAAA0000u	/* stock live */
/* ★★ build40: the REAL aal_ni NIRX-L3FE-demux per-ldpid table (0xa1d4-0xa1f0) - the
 * deep_q ROUTING table our driver NEVER wrote (normal @0xa1d4-e0, DPQ @0xa1e4-f0),
 * DISTINCT from the mislabeled 0xa190-a1a4 / 0xa1a8-b4 tables we wrote before (those are
 * a different table - leave them).  A deep_q=1 CPU frame's dest-select lives HERE; with
 * it at reset (non-L3QM) the frame egressed ES7(physical) not ES8(L3QM), so bm_tx climbed
 * but 0xa9fc stayed 0.  Written VERBATIM from the STOCK LIVE golden (stock_dpq_demux.txt,
 * tier-1) - the per-field L3QM decode was unreliable (RE said 0xa1e4=0, live=0x780C7864);
 * the live values are ground truth.  Same "whole demux table we skip" class as axi_reo/FBM. */
#define CA_NI_NIRX_L3FE_DEMUX_NORM0	0xa1d4
#define  CA_NI_NIRX_L3FE_DEMUX_NORM0_VAL 0x00000000u
#define CA_NI_NIRX_L3FE_DEMUX_NORM1	0xa1d8
#define  CA_NI_NIRX_L3FE_DEMUX_NORM1_VAL 0x00000000u
#define CA_NI_NIRX_L3FE_DEMUX_NORM2	0xa1dc
#define  CA_NI_NIRX_L3FE_DEMUX_NORM2_VAL 0x00000000u
#define CA_NI_NIRX_L3FE_DEMUX_NORM3	0xa1e0
#define  CA_NI_NIRX_L3FE_DEMUX_NORM3_VAL 0x64503C28u
#define CA_NI_NIRX_L3FE_DPQ0		0xa1e4		/* incl ldpid 50=0x32 (our CPU frame) */
#define  CA_NI_NIRX_L3FE_DPQ0_VAL	0x780C7864u
#define CA_NI_NIRX_L3FE_DPQ1		0xa1e8
#define  CA_NI_NIRX_L3FE_DPQ1_VAL	0x78A3E8C8u
#define CA_NI_NIRX_L3FE_DPQ2		0xa1ec
#define  CA_NI_NIRX_L3FE_DPQ2_VAL	0x60506050u
#define CA_NI_NIRX_L3FE_DPQ3		0xa1f0
#define  CA_NI_NIRX_L3FE_DPQ3_VAL	0x00006050u
/* ★ 0xa1c0 = 0x76543210 (stock live) - a per-slot port-index/order map right after the
 * demux tables; the ONE NI_HV word our driver never wrote (all of 0xa180-0xa1bc already
 * match stock).  0x76543210 = identity nibble map (slot n -> port n).  Prime suspect for
 * the L2TM-egress -> L3QM source-select: if unset, the L3QM never accepts the L2TM egress
 * (frame egresses L2TM, 0xa9fc stays 0).  RE-confirm the field, but write the stock word. */
#define CA_NI_NI_L3QMRX_PORT_ORDER	0xa1c0
#define  CA_NI_NI_L3QMRX_PORT_ORDER_VAL	0x76543210u

/* --- L2FE block: PLE default-forward indirect table (the DLF-trap-to-CPU
 *     fallback path).  Entry address = lspid << 2 | type; the ACCESS
 *     register both kicks a read (go) and a write (go|write) of the DATA
 *     register from/to the table; go self-clears. --- */
/* build68: the DFT_FWD control block regs stock sets but we never did (tier-1
 * stock devmem: 0x1560=0x00000007, 0x156c=0x4000003F).  Without them the PLE
 * default-forward engine stays disabled -> the DLF/BC frame is never redirected
 * to its mcgid, so the CPU flood copy is never produced. */
#define CA_NI_PLE_DFT_FWD_CTRL		0x1560
#define  CA_NI_PLE_DFT_FWD_CTRL_VAL	0x00000007u
#define CA_NI_PLE_DFT_FWD_CFG		0x156c
#define  CA_NI_PLE_DFT_FWD_CFG_VAL	0x4000003Fu
#define CA_NI_PLE_DFT_FWD_ACCESS	0x1564
#define  CA_NI_PLE_ACCESS_GO		BIT(31)
#define  CA_NI_PLE_ACCESS_WRITE		BIT(30)
#define CA_NI_PLE_DFT_FWD_DATA		0x1568
#define  CA_NI_PLE_DFT_DENY		BIT(0)
#define  CA_NI_PLE_DFT_MC_GROUP_ID	GENMASK(10, 1)
#define  CA_NI_PLE_DFT_REDIR_EN		BIT(11)
#define  CA_NI_PLE_DFT_MC_GROUP_VALID	BIT(12)
/* non-known traffic types (entry address low 2 bits) */
#define CA_NI_PLE_TYPE_BC		0
#define CA_NI_PLE_TYPE_UUC		1
#define CA_NI_PLE_TYPE_UL2MC		2
#define CA_NI_PLE_TYPE_UL3MC		3

/*
 * ★ L2FE forwarding-to-CPU (stock live-Linux values, STOCK_l2fe_forwarding.txt).
 * The chain for a LAN-port DLF frame: PLE_DFT_FWD redirects it to a redir-LDPID
 * (stock uses 0x18/0x19), and REDIR_LDPID_CONFIG[that] maps it to the real dest
 * LDPID = the CPU port (0x10).  Our driver set DFT_FWD -> 0x10 directly and
 * never populated REDIR_LDPID_CONFIG nor the ARB/PLC forwarding-control regs, so
 * DLF frames resolved to no CPU destination -> dropped before the QM.  aal_ne
 * runs aal_arb_init + aal_arb_redir_ldpid_config_set to set these.
 */
#define CA_NI_L2FE_PLC_DPID_FWD_CTRL	0x1404
#define  CA_NI_L2FE_DPID_FWD_CTRL_VAL	0x00000003u
#define CA_NI_L2FE_PLC_LRN_FWD_CTRL_0	0x1408
#define  CA_NI_L2FE_LRN_FWD_CTRL_0_VAL	0x22000290u
#define CA_NI_L2FE_PLC_LRN_FWD_CTRL_1	0x140c
#define  CA_NI_L2FE_LRN_FWD_CTRL_1_VAL	0x0c100c10u
#define CA_NI_L2FE_FDB_CTRL_0		0x1c00	/* FDB global control (aal_fdb_ctrl_set) - enables per-type fwd actions (unknown-DA/BC -> the LRN_FWD_CTRL dest). Live-stock golden. */
#define  CA_NI_L2FE_FDB_CTRL_0_VAL	0x82BFEFF9u
#define CA_NI_L2FE_FDB_CTRL_1		0x1c04	/* dest_ldpid[21:16]=0x10 (CPU_0) */
#define  CA_NI_L2FE_FDB_CTRL_1_VAL	0x9F90012Cu
#define CA_NI_L2FE_PLE_DEFAULT_REG	0x1504
#define  CA_NI_L2FE_PLE_DEFAULT_VAL	0x001b0000u
#define CA_NI_L2FE_ARB_CTRL		0x1600
#define  CA_NI_L2FE_ARB_CTRL_VAL	0x89c71c82u
#define  CA_NI_L2FE_ARB_MC_IDX_COPY_EN	BIT(3)	/* keep mc_idx on MC replication */
/* ARB reg 0x1608: stock devmem = 0x00000280; we default it to the stock value as a
 * plain stock-match.  NOTE (HW-confirmed 2026-07-11): this is NOT the cpu_flg gate -
 * setting it did NOT flip the header cpu bit.  Kept only to match stock's flag-source
 * block byte-for-byte; purpose being decoded. */
#define CA_NI_L2FE_ARB_REG_1608		0x1608
#define  CA_NI_L2FE_ARB_REG_1608_VAL	0x00000280u
#define CA_NI_L2FE_ARB_CTRL_EXT		0x160c
#define  CA_NI_L2FE_ARB_CTRL_EXT_VAL	0x00000001u
/* ARB regs 0x1614/0x1618/0x161c/0x1620: stock devmem = 0xFFFFFFFF on all four; our
 * driver never wrote them (reset 0).  We match stock (all-ones) as a plain stock-match
 * of the ARB block.  NOTE: NOT the cpu_flg gate - the real gate is the resolved LDPID
 * (must be a CPU port 0x10-0x17), fixed via DFT_FWD/REDIR redir-to-CPU.  Kept only to
 * keep the ARB block byte-identical to stock.  (Elnath: NOT the rtl8277c REDIR table -
 * that moved to 0x1624.) */
#define CA_NI_L2FE_ARB_ALLOW_MASK(n)	(0x1614 + (n) * 4)
#define CA_NI_L2FE_ARB_ALLOW_MASK_COUNT	4
#define  CA_NI_L2FE_ARB_ALLOW_MASK_VAL	0xffffffffu
/* redir-LDPID config table (indirect; aal FIND_INDIRCT_ADDRESS = GO|(rbw<<30)|
 * idx, poll GO clear).  DATA: rdir_cos_vld[0], rdir_cos[3:1], rdir_ldpid[9:4],
 * rdir_en[10], rdir_wan_dst[11]. */
/* ★ ELNATH offsets (our silicon uses the arch-elnath map, NOT rtl8277c): the
 * table-access regs are shifted vs rtl8277c (REDIR 0x1614->0x1624, MC_FIB
 * 0x1634->0x1644, MCE_INDX 0xaa64->0xaaf4).  Writing the rtl8277c offset hit a
 * wrong/unmapped reg -> mce_indx async-SError; REDIR/MC_FIB wrote garbage to a
 * valid-but-wrong reg (no fault, no effect = no delivery). */
#define CA_NI_L2FE_REDIR_LDPID_ACCESS	0x1624
#define CA_NI_L2FE_REDIR_LDPID_DATA	0x1628
#define  CA_NI_L2FE_REDIR_ACCESS_GO	BIT(31)
#define  CA_NI_L2FE_REDIR_ACCESS_WR	BIT(30)
#define  CA_NI_L2FE_REDIR_RDIR_LDPID	GENMASK(9, 4)
#define  CA_NI_L2FE_REDIR_RDIR_EN	BIT(10)
#define CA_NI_RX_BLACKHOLE_LDPID	0x1f	/* AAL_LPORT_BLACKHOLE: the DLF fall-through dest */

/*
 * ★★ L2FE PER-PORT PROFILE TABLES - THE ROOT CAUSE of the constant
 * blackhole resolution (rx_fe ldpid stuck 0x1f on EVERY LAN-ingress frame,
 * NI+0x6900 never climbing, regardless of DFT_FWD/PDPID/my-MAC config).
 *
 * The L2FE pipeline consults three per-logical-port profile tables and one
 * membership table BEFORE/AROUND the FDB + DFT_FWD forwarding decision:
 *   IPPB  (0x11dc/0x11e0)  physical->logical source-port map (1 word)
 *   ILPB  (0x11e4/0x11e8..0x11f8, DATA4..DATA0)  ingress port profile
 *   ELPB  (0x1420/0x1424+0x1428, DATA1+DATA0)    egress  port profile
 *   MMSHP (0x1574/0x1578+0x157c, DATA1+DATA0)    per-port allowed-ldpid
 *                                                64-bit isolation bitmap
 * All use the same indirect protocol as PDPID/DFT_FWD (DATA regs first,
 * then ACCESS = GO|WR|index, poll GO clear).
 *
 * Unprogrammed table contents are FORCED-DROP semantics:
 *   - ILPB.stp_mode[1:0] = 0 = STP DROP state (3 = forward+learn) -> every
 *     ingress frame is discarded at the PLC stage and resolves to the
 *     blackhole ldpid 0x1f BEFORE the FDB/DFT_FWD lookup is even used;
 *   - MMSHP[lspid] = 0x0 = "no egress port allowed" (isolation filter,
 *     active via PLE_CTL igr_fltr_chk_ldpid which is set at reset);
 *   - ELPB.egr_port_stp_status (bit1) = 0 = egress STP blocked.
 * Stock programs ALL of them at init (live leftovers on stock byte-match
 * the composed words below: ILPB DATA0=0xc1000xxx DATA1=xxx001cb/eb
 * DATA2 stp=3, ELPB DATA0=0xb, and PDPID_MAP[..] matches the same init's
 * ARB map).  Ours never touched them => tables at reset state => 100%
 * constant blackhole.  Fix = replicate the stock port-profile init.
 */
#define CA_NI_L2FE_IPPB_ACCESS		0x11dc
#define CA_NI_L2FE_IPPB_DATA		0x11e0	/* lspid[5:0] */
#define CA_NI_L2FE_ILPB_ACCESS		0x11e4
#define CA_NI_L2FE_ILPB_DATA4		0x11e8
#define CA_NI_L2FE_ILPB_DATA3		0x11ec
#define CA_NI_L2FE_ILPB_DATA2		0x11f0
#define CA_NI_L2FE_ILPB_DATA1		0x11f4
#define CA_NI_L2FE_ILPB_DATA0		0x11f8
#define CA_NI_L2FE_ELPB_ACCESS		0x1420
#define CA_NI_L2FE_ELPB_DATA1		0x1424
#define CA_NI_L2FE_ELPB_DATA0		0x1428
#define CA_NI_L2FE_MMSHP_ACCESS		0x1574
#define CA_NI_L2FE_MMSHP_DATA1		0x1578	/* allowed-ldpid bitmap [63:32] */
#define CA_NI_L2FE_MMSHP_DATA0		0x157c	/* allowed-ldpid bitmap [31:0]  */

/* logical-port map facts (lspid/ldpid space, 64 entries) */
#define CA_NI_L2FE_LPORT_COUNT		64
#define CA_NI_LPORT_ETH_NI6		0x06
#define CA_NI_LPORT_CPU_0		0x10
#define CA_NI_LPORT_CPU_7		0x17
#define CA_NI_LPORT_L3_WAN		0x18
#define CA_NI_LPORT_L3_LAN		0x19
#define CA_NI_LPORT_MC			0x1b
#define CA_NI_LPORT_GEM_FIRST		0x20	/* 0x20-0x3f = PON LLID/GEM + CPU-MQ */

/*
 * Composed ILPB profile words (the stock init state; every field named):
 * DATA4: wan_ind(b10) only for the PON-side logical ports.
 * DATA3: vlan_cmd_sel_bm[1:0]=3, sa_miss_fwd(b20)=1 (DLF forwards, no drop).
 * DATA2: stp_mode[1:0]=3 (FWD+LEARN - 0 would be DROP-ALL), tag drops all 0,
 *        c/s_tpid_match=cmp-entry-3 (0x8100), other_tpid_match=entry 3,
 *        top/inner C-TPID compare enabled (LAN ports) or S-TPID (GEM ports),
 *        inner_vid_sel_bm[28:27]=3.  MC port compares all 4 TPID entries.
 * DATA1: 802.1p_mark_bm hi[2:0]=3, change_1p_if_pop(b3)=1,
 *        flowid_sel_bm[8:6]=7, station_move_en(b31) on ETH-NI0-6 + CPU ports.
 * DATA0: 802.1p_mark_bm lo(b31)=1, inner_1p_mode[30:29]=2,
 *        dscp_mode[24:23]=2 (from RX DSCP map), ing_fltr_ena(b3)=0.
 */
#define CA_NI_L2FE_ILPB_D4_WAN		0x00000400u
#define CA_NI_L2FE_ILPB_D3_INIT		0x00100003u
#define CA_NI_L2FE_ILPB_D2_PORT		0x18022163u	/* LAN/CPU/L3/other <0x20 */
#define CA_NI_L2FE_ILPB_D2_MC		0x1803fd7fu	/* MC port 0x1b */
#define CA_NI_L2FE_ILPB_D2_GEM		0x180222a3u	/* GEM/LLID >=0x20 (S-TPID) */
#define CA_NI_L2FE_ILPB_D1_INIT		0x000001cbu
#define CA_NI_L2FE_ILPB_D1_STAMOVE	BIT(31)
#define CA_NI_L2FE_ILPB_D0_INIT		0xc1000000u
#define  CA_NI_L2FE_ILPB_STP_MODE	GENMASK(1, 0)	/* in DATA2; 0=DROP 3=fwd+learn */

/* ELPB DATA0: bit0 egr_vlan_aware, bit1 egr_port_stp FORWARD (0 = blocked!),
 * bit2 egr_ve_srch (off), bit3 egr_dest_wan (PON-side + L3_LAN dests). */
#define CA_NI_L2FE_ELPB_D0_LAN		0x00000003u
#define CA_NI_L2FE_ELPB_D0_WAN		0x0000000bu

/* L2FE direct config regs the stock init also sets (tier-1 live stock values;
 * our board is at the differing hardware defaults noted in comments) */
#define CA_NI_L2FE_GLB_CTRL		0x1004	/* dft 0x14 */
#define  CA_NI_L2FE_GLB_CTRL_STOCK	0x00000054u	/* + change_1p_if_pop_en(b6) */
#define CA_NI_L2FE_PP_TPID_CMP_S	0x1108	/* dft 0x8100 */
#define  CA_NI_L2FE_TPID_S_STOCK	0x000088a8u
#define CA_NI_L2FE_PP_TPID_CMP_O	0x1110	/* dft 0x8100 */
#define  CA_NI_L2FE_TPID_O_STOCK	0x00009100u
#define CA_NI_L2FE_PP_PARSER_CTRL	0x11d0	/* dft 0xa03b0400 */
#define  CA_NI_L2FE_PARSER_CTRL_STOCK	0xa03b2400u	/* + arp_op_filter_dis(b13) */
#define CA_NI_L2FE_PLC_L2_LEARNING	0x1410	/* dft 0x46 */
#define  CA_NI_L2FE_L2_LEARNING_STOCK	0x00000001u	/* unkvlan_learn only */
#define CA_NI_L2FE_PLC_VLAN_MODE	0x1414	/* dft 0x2 */
#define  CA_NI_L2FE_VLAN_MODE_STOCK	0x00000003u	/* + glb_vlan_mode(b0) */
#define CA_NI_L2FE_PLE_CTL		0x1500	/* dft 0x260 */
#define  CA_NI_L2FE_PLE_CTL_STOCK	0x0000027bu	/* + membshp_chk lan/wan(b0/1),
							 * skip_port_lpbk_chk(b3),
							 * pon_mode(b4) */
#define CA_NI_L2FE_PLE_UNKWN_VLAN_DFT1	0x150c	/* dft 0x0 */
#define  CA_NI_L2FE_UNKWN_VLAN_DFT1_STOCK 0x00000001u
/* ★ L2 FDB (MAC table) - hash table, HW computes the bucket on APPEND.  A static
 * entry {our MAC -> DeepQ_0} makes an own-MAC frame a KNOWN unicast forwarded
 * straight to DeepQ_0 -> PDPID=QM -> RMU -> CPU, bypassing the DLF/DFT_FWD/MCE
 * flood (which blackholed) and the L3FE.  Commit: write DATA0-3, ACCESS=GO|op,
 * poll GO clear, CMD_RETURN.status[3:0]==0x5 = HIT (appended OK). */
#define CA_NI_L2FE_FDB_ACCESS		0x1ca0
#define  CA_NI_L2FE_FDB_GO		BIT(31)		/* access / busy */
#define  CA_NI_L2FE_FDB_OP_INIT		0x00		/* one-time hash-table init */
#define  CA_NI_L2FE_FDB_OP_READ		0x04		/* look up (status=HIT on found) */
#define  CA_NI_L2FE_FDB_OP_APPEND	0x45		/* add a static entry */
#define CA_NI_L2FE_FDB_DATA3		0x1ca4
#define CA_NI_L2FE_FDB_DATA2		0x1ca8
#define CA_NI_L2FE_FDB_DATA1		0x1cac
#define CA_NI_L2FE_FDB_DATA0		0x1cb0
#define  CA_NI_L2FE_FDB_LPID		GENMASK(5, 0)	/* action: forward-to ldpid */
#define  CA_NI_L2FE_FDB_VALID		BIT(9)
#define  CA_NI_L2FE_FDB_STATIC		BIT(19)		/* no-age */
#define  CA_NI_L2FE_FDB_DA_PERMIT	BIT(20)		/* mandatory or DA-hit won't fwd */
#define  CA_NI_L2FE_FDB_SA_PERMIT	BIT(21)
#define CA_NI_L2FE_FDB_CMD_RETURN	0x1c2c		/* status[3:0]; 0x5 = HIT (appended) */
#define  CA_NI_L2FE_FDB_STATUS_HIT	0x5
/* ★ MYMAC own-MAC->CPU trap via the L3FE (tier-1: L2FE 0x1150 is a mask, and the
 * rtl8277c STG0-SPB 0x3440 is an UNMAPPED HOLE on Elnath - STG0 ends at LPB_IDX
 * 0x343c then jumps to STG1 0x3480 - so writing 0x3440 async-SErrors).  Stock's
 * ACTIVE my-MAC is comparator A (NI-global 0xa024/a028/a5c0); comparator B
 * (0x3294/98) reads 0 = unused; chk_mymac_for_lan (0x3400 b21) = 0.  The my-MAC-
 * hit route lives in the STG0 LPB profile vector (0x3408-3434), not a separate
 * SPB - so we program comparator A + my_mac_enable and match/decode the LPB. */
/* (A) NI-global my-MAC (the ACTIVE one): CFG0=bytes0-3, CFG1=byte4, PT[31:24]=byte5 */
#define CA_NI_L3FE_NI_MAC_CFG0		0xa024
#define CA_NI_L3FE_NI_MAC_CFG1		0xa028		/* [7:0] = byte4 */
#define  CA_NI_L3FE_NI_MAC_BYTE4	GENMASK(7, 0)
#define CA_NI_L3FE_PT_PORT_STATIC_CFG	0xa5c0		/* [31:24] = MAC byte5 (port 0) */
#define  CA_NI_L3FE_PT_MAC_BYTE5	GENMASK(31, 24)
/* enable (0x3400 bit21 chk_mymac_for_lan deliberately NOT set - stock has it 0) */
#define CA_NI_L3FE_SPCL_PKT_DET_CFG	0x3218
#define  CA_NI_L3FE_MY_MAC_EN		BIT(2)
#define CA_NI_L3FE_STG0_CTRL		0x3400
#define  CA_NI_L3FE_CHK_MYMAC_LAN	BIT(21)
/* STG0 LPB profiles (the my-MAC-hit route + spcl_pkt_en live in this per-profile
 * 96-bit vector LOW/MID/HIGH - the rtl8277c STG0-SPB 0x3440 is unmapped on Elnath
 * so stock's SPB writes drop and the route is in the LPB).  Direct registers.
 * We match stock's tier-1 values (bisect-from-working). */
#define CA_NI_L3FE_STG0_LDPID_MAP	0x3404
#define  CA_NI_L3FE_STG0_LDPID_MAP_VAL	0x03985907u
#define CA_NI_L3FE_STG0_LPB_LOW0	0x3408
#define CA_NI_L3FE_STG0_LPB_MID0	0x340c
#define CA_NI_L3FE_STG0_LPB_HIGH0	0x3410
#define CA_NI_L3FE_STG0_LPB_LOW1	0x3414
#define CA_NI_L3FE_STG0_LPB_MID1	0x3418
#define CA_NI_L3FE_STG0_LPB_HIGH1	0x341c
#define CA_NI_L3FE_STG0_LPB_LOW2	0x3420
#define CA_NI_L3FE_STG0_LPB_MID2	0x3424
#define CA_NI_L3FE_STG0_LPB_HIGH2	0x3428
#define CA_NI_L3FE_STG0_LPB_LOW3	0x342c
#define CA_NI_L3FE_STG0_LPB_MID3	0x3430
#define CA_NI_L3FE_STG0_LPB_HIGH3	0x3434
#define  CA_NI_L3FE_LPB_MID_SEL0	0x000003C0u	/* prof0/2 (spcl_pkt_sel=0) */
#define  CA_NI_L3FE_LPB_MID_SEL1	0x000007C0u	/* prof1/3 (spcl_pkt_sel=1) */
#define  CA_NI_L3FE_LPB_HIGH_P0		0x18100190u	/* WAN prof (spcl_pkt_en=1) */
#define  CA_NI_L3FE_LPB_HIGH_P1		0x19180180u	/* LAN prof */
#define  CA_NI_L3FE_LPB_HIGH_P3		0x1A1BFD90u	/* OAM prof */
#define  CA_NI_L3FE_LPB_SPCL_PKT_EN	BIT(20)
#define  CA_NI_L3FE_LPB_SPCL_PKT_SEL	BIT(19)
#define CA_NI_RX_CPU_LDPID		0x10	/* CPU_0 LDPID (direct-CPU; has NO ES port) */
/* ★ DLF redir target = DeepQ_0 (ldpid 0x00).  The BM-header deep_q (word0 bit30)
 * that steers a frame to L2TM ES port 7 (L3QM -> QM RMU -> CPU) is the L2FE ARB
 * `dbuf` flag, and the ARB resolves PDPID=QM (0x08) ONLY for {dbuf=1, dest-ldpid
 * in DeepQ 0x0..0x6}.  redir to CPU_0(0x10)/CPU_Q(0x1d) resolves to PDPID=CPU
 * (0x09) which has NO ES port (dead-end, qm_rx stayed 0); redir to L3_LAN(0x19)
 * needs the L3FE (we don't run it) to inject dbuf -> went to ES port 8.  So we
 * redir to a DeepQ ldpid and force dbuf=1 ourselves (cortina_ni_rx_arb_deepq_init). */
#define CA_NI_RX_REDIR_LDPID		0x00	/* DeepQ_0: {dbuf=1,ldpid}->PDPID=QM->ES port 7 */
/* ★★ LAN->CPU DLF forwarding = STOCK LITERAL (tier-1 golden stock_l2fe_forwarding.txt,
 * marked SOLVED): DFT_FWD[lspid0, all 4 types] = 0x1832.  redir_en=1, redir_ldpid field
 * = bits[5:0] = 0x32.  The chain: DFT_FWD 0x1832 -> ldpid 0x32 -> PDPID_MAP[0x32]=0x08
 * (QM) == ARB_CTRL.dbuf_dpid -> QM DeepQ -> ES7 -> RMU -> CPU.  Decode proof: stock uses
 * 0x1832 AND reaches CPU, which is only consistent with bits[5:0] (0x32->0x08/QM); a
 * bits[6:1] decode would make 0x1832->0x19->PDPID_MAP[0x19]=0x0D dead-end => stock
 * broken => impossible.  So the redir field is [5:0]; our earlier 0x1820 redirected to
 * ldpid 0x20 (UNMAPPED) -> frame lost -> 0x6900=0.  Copy the stock literal byte-for-
 * byte to sidestep the bit-field ambiguity. */
#define CA_NI_RX_DFT_FWD_CPU_VAL	0x00001832u
#define CA_NI_RX_QM_REDIR_LDPID		0x32	/* legacy redir target (unused after CPU redir) */
/* ★ L3_LAN ldpid (0x19): the L2FE resolves an own-MAC-unicast / ARP / L3-hit
 * frame to ldpid 0x19 via the my-MAC comparator + ARP parser (PARSER_CTRL
 * arp_op_filter_dis), NOT via DFT_FWD.  Stock PDPID_MAP[0x19]=0x0d (L3_LAN ->
 * ES port-8 -> L3FE -> CPU), a SECOND CPU path our RX driver does not drain
 * (we drain only ES7/QM/RMU).  Rather than wire the L3FE ES8 egress, collapse
 * this onto the proven RMU path: PDPID_MAP[0x19]=QM(0x08), same as 0x32. Both
 * classifier outputs (broadcast/DLF->0x32, my-MAC/ARP->0x19) then reach the
 * CPU through QM -> DeepQ -> ES7 -> RMU. */
#define CA_NI_RX_L3LAN_LDPID		0x19
/* ★ PDPID_MAP: the "logical dest LDPID -> physical dest port" table (indirect).
 * Stock: [0x10-0x17]=0x09(CPU), [0x1d]=0x09, [0x19]=0x0d(L3_LAN), [0-6]=0x08(QM).
 * Our driver NEVER programmed it - so a redir dest never resolved to the CPU.
 * We redir to CPU_0 (0x10) and program PDPID_MAP[0x10]=0x09 (CPU physical dest).
 * Indexed by {my_mac<<7|dbuf<<6|ldpid[5:0]}; DATA pdpid[3:0]. */
#define CA_NI_L2FE_PDPID_MAP_ACCESS	0x166c
#define CA_NI_L2FE_PDPID_MAP_DATA	0x1670
#define  CA_NI_L2FE_PDPID_MAP_PDPID	GENMASK(3, 0)
/* PDPID_MAP index = {my_mac[7], dbuf[6], ldpid[5:0]} */
#define  CA_NI_L2FE_PDPID_IDX_DBUF	BIT(6)
#define  CA_NI_L2FE_PDPID_IDX_MYMAC	BIT(7)
#define CA_NI_RX_CPU_PDPID		0x09	/* AAL_PPORT_CPU physical dest (no ES port) */
#define CA_NI_RX_QM_PDPID		0x08	/* AAL PDPID of the QM deep-queue (-> ES port 7) */
/* ★ L2FE ARB deep-buffer (dbuf) table = the L3FE-free source of BM-header deep_q.
 * ARB_CTRL.dbuf_sel picks flow-table(1) vs PORT_DBUF-table(0); we use PORT_DBUF,
 * whose entry matches the resolved dest-ldpid and outputs dbuf_flg. */
#define CA_NI_L2FE_ARB_CTRL		0x1600
#define  CA_NI_L2FE_ARB_DBUF_SEL	BIT(1)		/* 0 = use PORT_DBUF table */
#define  CA_NI_L2FE_ARB_DBUF_DPID	GENMASK(7, 4)	/* resolved PDPID==this => Deep Queue (dft 8=QM) */
#define  CA_NI_L2FE_ARB_USE_HDR_A_DBUF	BIT(8)		/* 1=take dbuf from Header-A (FIB/L3FE) */
#define CA_NI_L2FE_ARB_PORT_DBUF_ACCESS	0x1664
#define CA_NI_L2FE_ARB_PORT_DBUF_DATA	0x1668
/* ★★ build100: the ARB FLOW_DBUF table.  With ARB_CTRL.dbuf_sel=1 (stock) the HW uses THIS
 * table (per-flow deep-buffer flags), NOT PORT_DBUF, as the deep_q source.  Our driver only
 * programmed PORT_DBUF -> ignored -> the flooded broadcast ARP is never marked deep_q/cpu
 * (bm_word0 cpu=0/deep_q=0) -> no buffer attached -> CPU-EPP descriptor PA=0.  Indirect
 * (GO=bit31/WR=bit30, idx=flow_id/4); DATA bits[3:0]=dbuf_flg_0..3 (one per flow_id%4).
 * ★ Elnath offset UNCERTAIN: ca8277b raw=0x1648/0x164c, but Elnath PORT_DBUF is +0x14 at
 * 0x1664, and ca8277b FLOW_DBUF sits 8 bytes before PORT_DBUF -> best-guess 0x165c/0x1660.
 * The before/after readback log confirms which is right (non-zero after write = correct). */
#define CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS	0x165c	/* Elnath best-guess (PORT_DBUF-8); ca8277b raw=0x1648 */
#define CA_NI_L2FE_ARB_FLOW_DBUF_DATA	0x1660	/* ca8277b raw=0x164c */
#define  CA_NI_L2FE_ARB_FLOW_DBUF_FLG_ALL 0x0000000Fu	/* dbuf_flg_0..3 all set = all 4 flows deep_q */
#define  CA_NI_L2FE_ARB_FLOW_DBUF_ENTRIES 16u		/* test: mark flows 0..63 deep_q */
#define  CA_NI_L2FE_ARB_DBUF_FLG	BIT(0)		/* output: deep-buffer => deep_q */
#define  CA_NI_L2FE_ARB_DBUF_LDPID_VLD	BIT(1)
#define  CA_NI_L2FE_ARB_DBUF_LDPID	GENMASK(7, 2)	/* match on resolved dest-ldpid */
/* L2TM BM packet-memory read-back (buffer header): ACCESS selects buffer index,
 * DATA7 = Header-A word0 (bit30=deep_q, bit31=cpu_flag). */
#define CA_NI_L2TM_BM_PKT_MEM_ACCESS	0x218c
#define CA_NI_L2TM_BM_PKT_MEM_DATA7	0x2190
/* ★ BM latches the LAST frame's headers directly (no buffer-index hunt): FE =
 * the L2FE-RESOLVED forwarding header (deep_q b30 / cpu b31 / resolved ldpid),
 * NI = the raw received header, TX-NI = the dequeued frame's header.  These
 * reveal a real frame's actual resolution, unlike an empty seed buffer. */
#define CA_NI_L2TM_BM_RX_FE_HDR_LO	0x2170
#define CA_NI_L2TM_BM_RX_FE_HDR_HI	0x2174
#define CA_NI_L2TM_BM_RX_NI_HDR_LO	0x2178
#define CA_NI_L2TM_BM_RX_NI_HDR_HI	0x217c
#define CA_NI_L2TM_BM_TX_NI_HDR_LO	0x2180
#define CA_NI_L2TM_BM_TX_NI_HDR_HI	0x2184
#define CA_NI_L2TM_BM_STS		0x2188
/* ★ Stock redirs port-0 DLF to L3-LAN (LDPID 0x19 -> PDPID 0x0D), NOT CPU-direct
 * (0x10 -> 0x09).  Tier-1 proven: DFT_FWD[p0]=0x1832 (redir 0x19), and dest 0x09
 * (CPU-direct) does NOT enter the QM (qm_rx stays 0) while dest 0x0D (L3-LAN) DOES
 * (stock qm_rx climbs).  Both dest 0x09 and 0x0D map to EQ profile 13, so the pool
 * is identical; the L3-LAN path is simply the one the QM admits.  dest-port index
 * for physical dest 0x0D = 0x0D + 6 = 0x13 (0x6168 + 0x13*4 = 0x61b4, stock=0x0D). */
#define CA_NI_RX_L3LAN_PDPID		0x0d	/* L3-LAN physical dest (redir 0x19 -> this) */
#define CA_NI_RX_L3LAN_DEST_PORT	19	/* dest-port idx for phys dest 0x0D (0x61b4) */

/*
 * ★ Option A: MC-flood-to-CPU (stock CFG_ID_FLOODING_DOMAIN_1 lists CPU LDPID
 * 16 as a flood member).  A port-0 broadcast/DLF frame -> DFT_FWD -> mcgid G ->
 * ni_mce_indx[G].mc_vec -> the replication set incl the CPU.  We set both the
 * mc_vec bit 16 AND MC_FIB[16].ldpid=16 so it works whether the HW reads mc_vec
 * as a port bitmap (bit16 = CPU port) or a MC_FIB-entry bitmap (bit16 ->
 * MC_FIB[16] -> ldpid 16).  Indirect writes: DATA regs, then ACCESS=GO|WR|idx,
 * poll GO clear (MC_FIB/ni_mce_indx were readable earlier, unlike REDIR_LDPID). */
#define  CA_NI_IND_ACCESS_GO		BIT(31)
#define  CA_NI_IND_ACCESS_WR		BIT(30)		/* rbw = 1 for a store */
/* ★★ build70: the REAL RTL9607F stock MC_FIB offsets (tier-1 devmem golden,
 * dev/x411axf/stock_golden_qm.txt).  Fable's "ELNATH shift" to 0x1644/0x1648-58 was
 * WRONG: 0x1644 is a DEAD register (reads empty) - our whole flood config landed
 * there and never took.  The populated stock flood table lives at 0x1634:
 *   ACCESS 0x1634, DATA2 0x1638 (flood member bitmask), DATA1 0x163c, DATA0 0x1640.
 * Stock [0x10..0x1b] D2: 0F 04 0F 09 0F 05 0F 0A 0F 0B 0F 0C ([0x19]=0x0B=CPU flood). */
#define CA_NI_L2FE_MC_FIB_ACCESS	0x1634		/* real stock (0x1644 = DEAD reg) */
#define CA_NI_L2FE_MC_FIB_DATA2		0x1638		/* flood member bitmask */
#define CA_NI_L2FE_MC_FIB_DATA1		0x163c
#define CA_NI_L2FE_MC_FIB_DATA0		0x1640
#define CA_NI_L2FE_MC_FIB_DATA3		0x1644
#define CA_NI_L2FE_MC_FIB_DATA4		0x1648
#define  CA_NI_L2FE_MC_FIB_LDPID	GENMASK(29, 24)	/* per-copy dest ldpid (unused in flood-bitmask model) */
/* ★ The guaranteed CPU trap (fallback a): DFT_FWD redirects UUC/DLF to a REAL
 * MCE group (mc_group_id=0 was the reserved-NULL group -> blackhole 0x1f).  The
 * group has ONE member: mc_vec bit b selects ARB-FIB[b], whose copy ldpid =
 * DeepQ_0 -> PDPID=QM -> ES port 7 -> RMU -> CPU. */
#define CA_NI_RX_MCGID			0x20	/* our MCE group (non-empty; != reserved group 0) */
#define CA_NI_RX_MC_FIB_BIT		0x10	/* mc_vec bit = ARB-FIB member index (the one copy) */
#define CA_NI_NI_MCE_INDX_ACCESS	0xaaf4		/* ELNATH (rtl8277c 0xaa64) */
#define CA_NI_NI_MCE_INDX_DATA1		0xaaf8		/* mc_vec[63:32] */
#define CA_NI_NI_MCE_INDX_DATA0		0xaafc		/* mc_vec[31:0]  */
#define CA_NI_RX_FLOOD_MCGID		1		/* the mcgid our DFT_FWD floods to */
/* ★★ THE stock LAN->CPU chain (tier-1 golden, RE-confirmed): DFT_FWD 0x1832 target =
 * bits[10:1] = 0x19 is a 10-bit MULTICAST-GROUP INDEX (not a unicast ldpid).  A
 * broadcast/DLF frame REPLICATES via MC group 0x19's member vector; the CPU copy is
 * produced only if that vector includes the CPU deep-buffer ldpid 0x32 (whose
 * PDPID_MAP[0x32]=0x08=QM/dbuf -> TM-port 8 -> deep-buffer -> RMU0 -> CPU; HW then sets
 * cpu=1/deep_q=1).  Our MC group 0x19 was EMPTY -> no CPU copy -> the frame fell back to
 * ldpid 0x10 -> pdpid 0x09 -> missed RMU0.  Member vector is 64-bit, ldpid = bit
 * position: 0x32=50 -> high word (0xaaf8) bit18 = 0x00040000. */
#define CA_NI_RX_DFT_FWD_MCGID		0x19	/* the MC group DFT_FWD 0x1832 replicates to */
#define CA_NI_RX_MC_CPU_LDPID		0x32	/* CPU deep-buffer member ldpid (PDPID_MAP[0x32]=0x08) */

/* --- L3FE special-packet classifier (aal_l3_spcl_pkt / aal_l3_cls) --- build71.
 * ARP-to-CPU is an L3-CLS ethertype TRAP (rtl8277c hdr): a TCAM rule matches
 * ethertype ARP -> action dest CPU_0 (ldpid 0x10) with dpid_pri=1 (wins over the
 * fib/flood dest + becomes direct-CPU-bound).  The pri-7 rule beats the pri-0
 * broadcast->L3_LAN(0x19) rule, so a broadcast ARP is trapped to the CPU.  Two
 * indirect tables: KEY (11 words = match key+mask) and FIB (7 words = action). */
/* build73: L3FE stage-0 control.  aal_l3fe_stg0_set_normal() RMW-sets lpb_idx_mode=0
 * (index the LPB/CLS by HDR_A.ldpid, NOT lspid), use_lspid_ovwr_o_lspid=0,
 * mac_fltr_fail_pkt_mru_chk_en=0.  Our driver never runs stg0_init, so 0x3400 sits at
 * reset default 0x001c7c7e (bits 1,10 extra vs stock 0x001c787c) -> a wrong lpb_idx_mode
 * mis-indexes the STG0 LPB, so the CLS lookup can't match our ingress (build72 rows
 * byte-matched stock yet never fired).  Write stock's exact post-init value. */
#define CA_NI_L3FE_STG0_CTRL		0x3400
#define  CA_NI_L3FE_STG0_CTRL_VAL	0x001c787cu	/* tier-1 stock live (ours reset=0x001c7c7e) */
#define CA_NI_L3FE_MY_MAC_LO		0x3210	/* my-MAC/FIELD_CAM: LO = valid(bit16)|mac[0]<<8|mac[1] (stock=board MAC+valid; ours was 0) */
#define  CA_NI_L3FE_MY_MAC_VALID	BIT(16)	/* 0x3210 bit16 = my-MAC entry valid (tier-1 stock diff 2026-07-12) */
#define CA_NI_L3FE_MY_MAC_HI		0x3214	/* HI = mac[2]<<24|mac[3]<<16|mac[4]<<8|mac[5] */
#define CA_NI_L3FE_SPCL_PKT_DET_CFG	0x3218	/* stock 0x0739DC24 (detection pipeline; matches, leave) */
/* ★★ L3FE ingress-loopback port-validity table (ca8277b/Elnath L3FE_GLB_ILPB_LDPID @0xf43030d8;
 * vendor aal_l3fe_l2lookup_init).  Our driver MISSED it -> a frame forwarded to the L3_LAN
 * loopback ingress (ldpid 0x19) had NO valid L3FE ingress, so it never entered the classifier ->
 * l3fe_rx(0xa9bc)=0 -> cls_hit=0.  Layout: valid0[0],ldpid0[6:1],valid1[8],ldpid1[14:9],
 * valid2[16],ldpid2[22:17],valid3[23].  We mark ldpid0=L3_LAN(0x19), ldpid1=L3_WAN(0x18), and
 * entry3 valid (ldpid3=0=NI port0) as valid L3 ingress (valid2/ETH_WAN left off - confirm vs stock). */
#define CA_NI_L3FE_GLB_ILPB_LDPID	0x30d8	/* stock Elnath=0 (SDK's non-zero value does NOT apply here) - leave 0 */
/* ★★ THE missing L3FE_GLB config block (ca8277b/Elnath).  Our driver NEVER ran the vendor
 * L3FE global init (aal_l3fe_l2lookup_init aal_l3fe.c:269-310 + the glb ELPB setters
 * :215-267, called from aal_l3fe_init :1030) -> the whole 0x30ac-0x30f8 block sat at 0 ->
 * the L3FE stage was uninitialized -> it never ingested frames -> l3fe_rx(0xa9bc)=0 ->
 * cls_hit=0 -> no CPU RX.  Values below = tier-1 live stock (captured under broadcast). */
/* ★ build95: the remaining L3FE_GLB config our driver still left at 0 (tier-1 stock diff).
 * All CONFIG (NOT ring/DMA pointers - ca8277b structs: FWD_CTRL=fwd-control bitfields,
 * LF_CFG=ingress-FIFO thresholds, ILPB=per-port VLAN loopback config).  A zero LF_CFG
 * (thresholds=0) blocks the L3FE ingress FIFO -> l3fe_rx=0 = the prime remaining gate. */
#define CA_NI_L3FE_GLB_FWD_CTRL_1	0x30a4
#define  CA_NI_L3FE_GLB_FWD_CTRL_1_VAL	0x8001B000u
#define CA_NI_L3FE_GLB_FWD_CTRL_2	0x30a8
#define  CA_NI_L3FE_GLB_FWD_CTRL_2_VAL	0x004641F4u
#define CA_NI_L3FE_GLB_LF_CFG		0x30b4	/* ingress-FIFO hi/low/wr_fifo thresholds (dft 0x004641f4) */
#define  CA_NI_L3FE_GLB_LF_CFG_VAL	0x4855CFE4u
#define CA_NI_L3FE_GLB_ILPB_00		0x30bc	/* ingress-loopback VLAN config, entry0 (COUNT=4 stride 8) */
#define  CA_NI_L3FE_GLB_ILPB_00_VAL	0x4856CF7Cu
#define CA_NI_L3FE_GLB_FWD_CTRL_3	0x30ac
#define  CA_NI_L3FE_GLB_FWD_CTRL_3_VAL	0x00000300u
#define CA_NI_L3FE_GLB_CFG_30CC		0x30cc	/* unnamed in ca8277b but stock-mapped (reads 0xE21) - match stock */
#define  CA_NI_L3FE_GLB_CFG_30CC_VAL	0x00000E21u
#define CA_NI_L3FE_GLB_ELPB0		0x30e0	/* egress-loopback entry0 (port map, aal_l3fe_glb_elpb_set) */
#define  CA_NI_L3FE_GLB_ELPB0_VAL	0x00007F03u
#define CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD1	0x30e4	/* deep-queue valid vec hi (aal_l3fe_glb_elpb_deepq_vld_set) */
#define  CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD1_VAL 0x00C00000u
#define CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD0	0x30e8	/* deep-queue valid vec lo */
#define  CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD0_VAL 0x00400000u
#define CA_NI_L3FE_GLB_ELPB_DEEPQ1	0x30ec	/* deep-queue vec hi (aal_l3fe_glb_elpb_deepq_set) */
#define  CA_NI_L3FE_GLB_ELPB_DEEPQ1_VAL	0x00F00000u
#define CA_NI_L3FE_GLB_ELPB_DEEPQ0	0x30f0	/* deep-queue vec lo */
#define  CA_NI_L3FE_GLB_ELPB_DEEPQ0_VAL	0x00F00000u
#define CA_NI_L3FE_GLB_L3FE_L2FE_LDPID	0x30f4	/* the L3FE<->L2FE loopback ldpid binding */
#define  CA_NI_L3FE_GLB_L3FE_L2FE_LDPID_VAL 0x000C0000u
#define CA_NI_L3FE_GLB_VE		0x30f8	/* vlan-edit tpid enc (aal_l3fe_l2lookup_init) */
#define  CA_NI_L3FE_GLB_VE_VAL		0x00040000u
/* build74: L3FE global CLS-stage monitor (read cls_hit_0..3).  aal_l3fe_glb_cls_stg
 * _monitor_get: for i, write CTRL={enable bit0=1, bus_sel=(MONITOR_CLS_RESULT=3)<<5 | i},
 * read RETURN.  cls_hit[i]==0 across all while frames flow = the CLS lookup is NOT
 * invoked for our ingress (an L3-ingress/enable gap, not a key-match issue). */
#define CA_NI_L3FE_CLS_MON_CTRL		0x30b0
#define CA_NI_L3FE_CLS_MON_RETURN	0x30b4
#define CA_NI_L3FE_MON_CLS_RESULT	3	/* l3fe_glb_monitor_vector_e MONITOR_CLS_RESULT */
#define CA_NI_L3FE_CLS_KEY_ACCESS	0x3380	/* GO|WR|idx; poll GO clear (CA_NI_IND_ACCESS_GO/WR) */
#define CA_NI_L3FE_CLS_KEY_DATA_BASE	0x3384	/* 11 words 0x3384..0x33ac */
#define CA_NI_L3FE_CLS_KEY_WORDS	11
#define CA_NI_L3FE_CLS_FIB_ACCESS	0x33b0
#define CA_NI_L3FE_CLS_FIB_DATA_BASE	0x33b4	/* 7 words 0x33b4..0x33cc */
#define CA_NI_L3FE_CLS_FIB_WORDS	7
#define CA_NI_RX_CLS_ENTRIES		16	/* KEY/FIB indices [0..15] */

/*
 * ★ Datapath-bisect counters (REAL, unlike the phantom per-port MAC RX MIB).
 * Read before/after a ping to find the FIRST stage that stays 0 while the
 * prior increments = the death point: L2FE sees the frame? -> L2FE drops it?
 * -> L2FE forwards to TM? -> QM RMU ingest? -> QM drops? -> CPU-EPP wptr.
 */
#define CA_NI_L2FE_NI_INTF_DROP_CNT	0x11bc	/* frames NI->L2FE dropped   */
#define CA_NI_L2FE_NI_INTF_PKT_CNT	0x11c0	/* frames NI->L2FE (ingress) */
#define CA_NI_L2FE_DOS_FLOOD_CNT	0x1234	/* DoS/flood drop            */
#define CA_NI_L2FE_PE_TM_PKT_CNT	0x1760	/* frames L2FE->TM (forwarded)*/
#define CA_NI_QM_RMU0_RX_PKT_CNTR	0x6900	/* ELNATH (rtl 0x67d8); frames into QM RMU */
#define CA_NI_QM_RX_EOP_DROP_CNTR	0x6948	/* ELNATH (rtl 0x6820) */
#define CA_NI_QM_RX_LEN_ERR_CNTR	0x694c	/* ELNATH (rtl 0x6824) */
#define CA_NI_QM_RX_L2TE_DROP_CNTR	0x6950	/* ELNATH (rtl 0x6828) */

/*
 * ★ TM->CPU-VoQ->CPU-EPP final-hop bisect (frames reach L2FE->TM cleanly now;
 * do they get into the CPU VoQ and drain to the CPU-EPP?).  L2TM buffer-manager
 * RX/TX/drop counters (0x21xx): RX = frames into the TM BM, TX = frames out to
 * the QM, and NOBUF = dropped for want of a buffer (the prime suspect given our
 * CPU-push EQ8 differs from stock's DDR empty-buffer pool). */
#define CA_NI_L2TM_BM_RX_PCNT		0x213c	/* frames into L2TM BM (enqueue)  */
#define CA_NI_L2TM_BM_TX_PCNT		0x2140	/* frames out of L2TM BM->QM (egress) */
#define CA_NI_L2TM_BM_SB_DPCNT		0x2144	/* ★ shared-buffer-full drop (RE a053902d) */
#define CA_NI_L2TM_BM_HDR_DPCNT		0x2148	/* header drop */
#define CA_NI_L2TM_BM_TE_DPCNT		0x214c	/* ★ threshold-engine (deep-queue/VOQ/port thr) drop */
#define CA_NI_L2TM_BM_ERR_DPCNT		0x2150	/* error drop */
#define CA_NI_L2TM_BM_RX_DPCNT		0x2164	/* enqueue drop */
#define CA_NI_L2TM_BM_NOBUF_DPCNT	0x216c	/* ★ no-free-buffer drop */
/* QM per-VoQ non-empty status (VOQ_STATUS0..7); a set bit = that VoQ has a
 * queued descriptor.  The CPU dest port (8) VoQ shows here if the frame reached
 * the CPU VoQ (dest routing OK) but wptr=0 (drain broken). */
#define CA_NI_QM_VOQ_STATUS(n)		(0x6958 + (n) * 4)	/* ELNATH (rtl 0x6830) */
/* dest-routing maps: L2TM TM-output -> CPU-queue (stock 0x76543210) and the QM
 * upper-LDPID map - the LDPID(0x10/0x19) -> CPU-port(8) path the TM uses */
#define CA_NI_L2TM_TM_TO_CPUQ_MAP	0x2118
/* ★★ L2TM TM-egress map block (0x2100..0x2120): routes a BM-dequeued frame from its
 * TM-port into the CPU queue that RMU0 services.  U-Boot does NOT set these, so a
 * non-deep CPU frame egresses the L2TM (tm tx++) but never reaches RMU0 (0x6900=0, no
 * drop).  Tier-1 stock (stock_l2tm_deepq): 0x2100=0x2, 0x2110=0xC0000000,
 * 0x2114=0xFEDCBA98, 0x2118=0x76543210, 0x211c=0x76, 0x2120=0x76543210. */
#define CA_NI_L2TM_TM_CFG		0x2100
#define  CA_NI_L2TM_TM_CFG_VAL		0x00000002u
#define CA_NI_L2TM_TM_MAP_A		0x2110
#define  CA_NI_L2TM_TM_MAP_A_VAL	0xc0000000u
#define CA_NI_L2TM_TM_MAP_B		0x2114
#define  CA_NI_L2TM_TM_MAP_B_VAL	0xfedcba98u
#define  CA_NI_L2TM_TM_TO_CPUQ_VAL	0x76543210u	/* 0x2118 */
#define CA_NI_L2TM_TM_MAP_C		0x211c
#define  CA_NI_L2TM_TM_MAP_C_VAL	0x00000076u
#define CA_NI_L2TM_TM_MAP_D		0x2120
#define  CA_NI_L2TM_TM_MAP_D_VAL	0x76543210u
#define CA_NI_QM_UPPER_LDPID_MAP	0x68f8
#define CA_NI_PLE_TYPE_COUNT		4

/* ★ stock __ni_flow_ctrl_init (ca-ne.ko @0xc160) — the L2TM BM dequeue->TM-port
 * map + per-port flow-control thresholds.  Our driver leaves L2FE/L2TM at U-Boot
 * init, but U-Boot does NOT run this flow-ctrl init, so the BM dequeue->TM-port
 * map (0x2124) sits unset: a CPU-dest frame is DEQUEUED (BM_TX_PCNT 0x2140 ++)
 * yet egresses to the wrong TM-port instead of the QM, so qm_rx_cntr(0x690c)
 * stays 0 with no drop counted.  0x88888888 = every BM dequeue -> TM-port 8 (QM). */
#define CA_NI_NI_FLOWCTRL_EN		0x3400		/* RMW |= bit19 (flow-ctrl enable) */
#define  CA_NI_NI_FLOWCTRL_EN_BIT	0x00080000u
#define CA_NI_L2TM_BM_DQ_PORT_MAP	0x2124		/* BM dequeue LANL3FE-queue flag map (->L3QM/ES8) */
#define  CA_NI_L2TM_BM_DQ_PORT_MAP_VAL	0x88888888u	/* all dequeues -> TM-port 8 = QM */
/* ★ build39: the PHYSICAL DQ->TM-port NUMBER map (separate from 0x2124's uplink flag).
 * tm_dpid_sel 4b/entry, stock/default identity 0x76543210 (DQ N -> TM-port N).  Our
 * 0x2124=0x88888888 is confirmed set (real readl) yet the deep-queue still drains to
 * ES7=NI7(physical) not ES8=L3QM - so the 0x2124 uplink override is NOT taking effect
 * on ours.  This map lets us FORCE the physical port: 0x88888888 = every DQ -> TM-port 8
 * = ES8/L3QM.  Swept via the dq_tmport_map module param (rebuild-free A/B). */
#define CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP	0x212c
#define CA_NI_NI_FLOWCTRL_THRESH	0x9798		/* per-port flow-ctrl thresholds  */
#define  CA_NI_NI_FLOWCTRL_THRESH_CNT	7		/* 0x9798..0x97b0, stride 4       */
#define  CA_NI_NI_FLOWCTRL_THRESH_CLR	0x3fff3fffu	/* stock bfxil[0:13]+bfi[16:29]   */
#define  CA_NI_NI_FLOWCTRL_THRESH_VAL	0x01a80178u	/* [0:13]=0x178 [16:29]=0x1a8     */

/* driver-side RX geometry */
#define CA_NI_RX_CPU_PORT		0	/* CPU port 0, voq 0 */
#define CA_NI_RX_VOQ			0
#define CA_NI_RX_PORT			0	/* ingress LAN port */
/* total CPU-PUSH pool = EQ13 primary + EQ14 overflow + EQ8 RMU-alloc.  EQ12 is
 * DRAM-backed (auto-populated), NOT CPU-pushed, so it is NOT in this seed count. */
#define CA_NI_RX_POOL_SIZE		(CA_NI_RX_EQ_TOTAL_BUF + CA_NI_RX_EQ2_TOTAL_BUF + \
					 CA_NI_RX_EQ8_TOTAL_BUF)
#define CA_NI_RX_POOL_MIN		32	/* below this: warn (still usable) */
#define CA_NI_RX_SEED_BUDGET_MS		600	/* total seed wall-clock cap (1327 bufs) */
#define CA_NI_RX_NUM_IRQS		8	/* DT idx 0..7 = SPI 0x54..0x5b */

/* ★★ NI->L3QM ingress handoff / flow-control (stock __ni_flow_ctrl_init @0xc160 +
 * aal_ni_rxmux_fc_thrshld_set + the NI-HV RXFIFO threshold).  The frame egresses the
 * L2TM (tm tx++) but the QM ingress is 0 (qm_rx=0) because the NIRX->L3QM presentation
 * is back-pressured: our flow_ctrl_init did 0x3400 b19 + 0x2124 + 0x9798 but MISSED
 * these.  All values are tier-1 stock live (stock_l2tm_qm_handoff.txt). */
#define CA_NI_NI_FC_2914		0x2914		/* __ni_flow_ctrl_init: RMW |= bit31 */
#define  CA_NI_NI_FC_2914_EN		BIT(31)		/* stock 0x2914=0x800073FF */
/* RXMUX flow-control thresholds (aal_ni_rxmux_fc_thrshld_set, 0xa05c + idx*4, idx 0-11;
 * [9:0]=thr_lo, [19:10]=thr_hi).  Stock: idx0-3 = 0x0002080A, idx4-11 = 0x0000A014. */
#define CA_NI_NI_RXMUX_FC_THR(idx)	(0xa05c + (idx) * 4)
#define  CA_NI_NI_RXMUX_FC_THR_LO_VAL	0x0002080Au	/* idx 0-3 */
#define  CA_NI_NI_RXMUX_FC_THR_HI_VAL	0x0000A014u	/* idx 4-11 */
#define  CA_NI_NI_RXMUX_FC_THR_LO_CNT	4
#define  CA_NI_NI_RXMUX_FC_THR_CNT	12
/* NI-HV RXFIFO threshold block (RXFIFO_THRESHOLD_MISC_CFG, incl. l3qm_rxfifo_hi[6:0]):
 * stock live 0xa1b0=0x22AA0000, 0xa1b4=0xAAAA0000, 0xa1b8=0x00086FFC.  A 0 l3qm_rxfifo_hi
 * back-pressures NIRX->L3QM -> qm_rx=0.  0xa1bc (NIRX_MISC=0x3E80) is written elsewhere;
 * 0xa180-0xa1a0 are L3FE demux ctx we already set - do NOT touch those. */
#define CA_NI_NI_RXFIFO_THR_B0		0xa1b0
#define  CA_NI_NI_RXFIFO_THR_B0_VAL	0x22AA0000u
#define CA_NI_NI_RXFIFO_THR_B4		0xa1b4
#define  CA_NI_NI_RXFIFO_THR_B4_VAL	0xAAAA0000u
#define CA_NI_NI_RXFIFO_THR_B8		0xa1b8
#define  CA_NI_NI_RXFIFO_THR_B8_VAL	0x00086FFCu	/* [6:0]=0x7C = l3qm_rxfifo_hi */

#endif /* _CORTINA_NI_REGS_H */
