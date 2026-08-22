// SPDX-License-Identifier: GPL-2.0-only
/*
 * cortina-l3fe.c - RTL9607F / Cortina CA8277C "Elnath" L3FE main-hash
 * flow-engine bring-up (the one-time init chain that must complete before
 * any flow can be added for a lookup to HIT), plus the mask-table and
 * HS_SWO HW-CRC primitives used by the offload backend and its selftest.
 *
 * Companion of cortina-ni-flowoffload.c (the nf_flow_table flow_block
 * backend).  Called once from the cortina-ni probe via
 * cortina_ni_flowoffload_probe().
 *
 * Facts + citations: dev/x400axf/HW_FLOW_OFFLOAD_L3FE_INIT.md and
 * HW_FLOW_OFFLOAD_DESIGN.md (clean-room RE of the stock ca-ne.ko register
 * sequences, cross-checked against the chip register map).  Every literal
 * below was additionally LIVE-VERIFIED against the stock firmware's armed
 * engine (devmem capture of the 0xf43037xx-0x3cxx block, 2026-07-18) - the
 * capture corrected two RE assumptions:
 *   - HS_HASH_INI = 0x0003007D: hb_size=1 (8-way hash buckets, NOT 32) and
 *     def_reg=1 (default/miss actions come from the internal
 *     HS_DEFAULT_ACTION registers, not a DDR table).
 *   - Only BA_MH0/BA_MA0 are armed; BA_OA0/BA_DA0/BA_CA0 stay 0 (overflow
 *     CAM unused by the stock add path, action cache in on-chip SRAM), so
 *     the DDR carve is key(256K) + FIB(2M) only.
 * The L3FE AXI-REO read-ID remap this engine needs is the channel at the
 * AXI-REO window +0x480 (abs 0xf432d480) - already programmed by
 * cortina-ni-rx.c since build97 and byte-matching stock; the 0xf432f080
 * block reads all-zero on live stock (design divergence D8 resolved).
 *
 * NOTE phase 1 arms the engine only; the ingress classify plumbing that
 * makes traffic actually consult the hash (STG0/LPB profiles, hash-profile
 * tuples + masks, my-MAC CAM, egress L3-IF entries) is phase-2/3 work and
 * deliberately NOT touched here - the working RX/TX datapath depends on the
 * live classifier state.  Stock reference values for those registers are in
 * the 2026-07-18 capture (scratchpad stock_l3fe_hs_dump.txt).
 */

#include <linux/kernel.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>

#include "cortina-l3fe.h"

/* ------------------------------------------------------------------ *
 *  L3FE HS register map, NE-relative (NI window phys 0xf4300000).      *
 * ------------------------------------------------------------------ */
#define L3FE_HS_HASH_INI		0x3834	/* hb[1:0] ht[4:2] ha[7:5] def_reg[16] crc_ntfy[17] */
#define L3FE_HS_BA_MH1			0x3838	/* key table base, phys[39:32] */
#define L3FE_HS_BA_MH0			0x383c	/* key table base, phys[31:7] in place */
#define L3FE_HS_BA_MA1			0x3840	/* action FIB base, phys[39:32] */
#define L3FE_HS_BA_MA0			0x3844	/* action FIB base, phys[31:7] in place */
#define L3FE_HS_DEFAULT_ACTION(i)	(0x3860 + (i) * 4) /* internal default/miss actions (def_reg=1) */
#define L3FE_HS_CACHE_INI		0x38a0
#define L3FE_HS_CACHE_MISC		0x38c4	/* cache replacement policy */
#define L3FE_HS_MASK_ACCESS		0x3910	/* idx | W[30] | GO[31] | upper-128[6] */
#define L3FE_HS_MASK_DATA(n)		(0x3920 - (n) * 4) /* MASK0..3 = 0x3920,191c,1918,1914 */
#define L3FE_HS_AGING_GRANULARITY	0x3924	/* 0 = HW auto-age-countdown OFF */
#define L3FE_HS_MEM_INI			0x393c	/* bit0 req_sts: engine table self-init */
#define L3FE_HS_CHK_FAIL_CTRL		0x3940	/* double-check-fail -> punt */
#define L3FE_HS_RSV0			0x3944	/* HW patch: bit31 crc_offload + bit0 */
#define L3FE_HS_RSV1			0x3948	/* HW patch: bit0 */
#define L3FE_HS_SWO_IDX			0x38d8	/* HW-CRC engine pointer */
#define L3FE_HS_SWO_DAT			0x38dc	/* HW-CRC engine data (auto-inc IDX) */
#define L3FE_HS_SWO_CTRL		0x38e0	/* bit0 = GO / busy */
#define L3FE_AQM_TIMER			0x3aa8	/* AQM flow-stat timer cfg */
#define L3FE_AXIM2_CONFIG		0x3c80	/* AXI outstanding-transaction depth */

/* Internal hash-miss action FIB (HASH_INI.def_reg=1 mode): 6 entries x 3
 * regs = 96 bits each, holding the packed default (miss) action.  The
 * per-profile HS_PROFILEn_INI default_sel picks the entry; a routed miss
 * uses it to punt to CPU_0 (never drop). */
#define L3FE_HS_DEF_REG0_ETY0		0x39dc	/* first of the 6x3-reg internal FIB */
#define L3FE_HS_DEF_REG_COUNT		12	/* entries 0..3 captured live from stock */

/* L3-CLS classifier FIB (indirect): 7 words, DATA0 at 0x33cc down to
 * DATA6 at 0x33b4; ACCESS = GO|WR|idx.  The per-profile routing DEFAULT
 * actions live at idx (max_entry-16)|(profile<<2): 1024/1025 = profile 0
 * (WAN ingress), 1028 = profile 1 (LAN ingress). */
#define L3FE_CLS_FIB_ACCESS		0x33b0
#define L3FE_CLS_FIB_DATA0		0x33cc	/* word0; word i at DATA0 - i*4 */
#define L3FE_CLS_FIB_WORDS		7

/* Main-hash per-profile TUPLE0 maskptr (maskptr[5:0], pri[10:8], type[12]);
 * profile stride 0x2c.  Re-pointed at the 5-tuple mask under hw_l3_fwd so a
 * routed flow's install/lookup CRC uses the 5-tuple-only mask. */
#define L3FE_HS_PROFILE_TUPLE0(p)	(0x3704 + (p) * 0x2c)
#define L3FE_MAIN_HASH_PROFILE_WAN	0
#define L3FE_MAIN_HASH_PROFILE_LAN	1
/* ★ The hash profile the LIVE admission actually runs (P4, 2026-07-19): the
 * transit LAN unicast matches the LAN cls-trap catch-all rows (FIB 256/257/
 * 260/264), and the profile stamped there is 3 - on stock, profile 3 is the
 * flow profile the catch-all class feeds (the RTK asicDriver maps its
 * catch-all flow key-type to HASH_PROFILE_3; tier-1: stock's live routed FIB
 * rows carry t2_ctrl=3).  Ours re-purposes profile 3 with the 5-tuple mask 8
 * (gated), so install + lookup agree end to end. */
#define L3FE_MAIN_HASH_PROFILE_ROUTED	3
/* the engine has 7 hash profiles (0..6), stride 0x2c from HS_PROFILE0_INI */
#define L3FE_MAIN_HASH_PROFILE_MAX	6u
#define L3FE_5TUPLE_MASK_ID		8

/*
 * L3FE PE config (direct MMIO): the GEM-map mode for PON US hit-forwarding.
 * With gemid_map=1, a hit-action carrying the gemMapMode-1 encoding (mc=1,
 * mcgid=gem_id, group-20 {pop_l3_vld=1, t2_ctrl1=tcont}) egresses as
 * hdr_a.ldpid = ldpid_base + {ldpid_offset_msb, t2_ctrl1} - i.e. the PON US
 * logical port 0x20 + tcont, exactly the ldpid the proven CPU US data path
 * injects with (vendor aal_l3pe_pe_gemid_map_set(1) +
 * aal_l3pe_pe_ldpid_base_set(PON_US_0=0x20), rtk_rg_asic_l3qm_init).
 * Register fields per the rtl8277c map: ldpid_base[9:4], gemid_map[10].
 */
#define L3FE_PE_CFG			0x351c
#define L3FE_PE_CFG_LDPID_BASE		GENMASK(9, 4)
#define L3FE_PE_CFG_GEMID_MAP		BIT(10)
#define L3FE_PE_CFG_PAD_CTRL		BIT(12)		/* vendor-init: set by stock */
#define L3FE_PE_CFG_RSVD14		BIT(14)		/* vendor-init: set by stock */
#define L3FE_PE_CFG_MTU_CHK_EN		BIT(31)		/* ★ reset=1; stock CLEARS it.  Left set,
							 * every PE-transiting T2-miss CPU punt is MTU-
							 * length-checked against the (mostly-zero)
							 * L3FE_PE_MTU_SIZE table -> large frames (ssh KEX,
							 * bulk shell) diverted to CPU_REASON_MTU and lost;
							 * small frames (SYN/banner/ICMP) pass = the
							 * terminating-TCP "mangle".  Clear to match stock. */
#define L3FE_LDPID_PON_US_0		0x20	/* AAL_LPORT_PON_US_0 */

/*
 * L3FE PE PPPoE encap globals (direct MMIO, one-time; tier-2 disasm-confirmed
 * on the shipped ca-ne.ko: aal_l3pe_pppoe_cfg_set writes base+0x3500,
 * aal_l3pe_pppoe_ipv4/ipv6_prot_set write base+0x3504).  Consumed by the PE
 * only when a hit-action carries the PPPoE ADD encap (pppoe_set=1 +
 * pppoe_vld=1): CFG supplies the header's code/version/type, PROT_CFG the
 * PPP protocol number per inner IP version.  The 0x8864 ethertype is fixed/
 * implicit and the PPPoE payload length is HW-computed per packet.
 * Facts: scratchpad RE_pppoe_aal77c.md section 3d / PPPoE_DRIVER_SPEC.md 2a.
 */
#define L3FE_PE_PPPOE_CFG		0x3500	/* [15:8]=code 0 (session), [7:4]=ver 1, [3:0]=type 1 */
#define L3FE_PE_PPPOE_CFG_VAL		0x00000011u
#define L3FE_PE_PPPOE_PROT_CFG		0x3504	/* [15:0]=v4 PPP-proto 0x0021, [31:16]=v6 0x0057 */
#define L3FE_PE_PPPOE_PROT_CFG_VAL	0x00570021u

/*
 * Egress L3-IF table (fe_l3e_if_tbl): 32 x 32-bit entries, GLB indirect
 * access (same GO|WRITE protocol as the mask table) - tier-2 binary-confirmed
 * descriptor {access=0xf4303000, 1 data reg, 32 entries}.  Entry, LSB-first:
 * pppoe_set:1(b0) pppoe_vld:1(b1) pppoe_session_id:16(b2-17) mac_sa_vld:1(b18)
 * mac_sa_an_sel:4(b19-22) pppoe_len_control:1(b23).  A hit-action selects an
 * entry via GROUP_20 l3_if_vld1 + egr_l3_if_idx1.
 *
 * Two independent tiers agree on those field positions: the shipping-die
 * register description (L3FE_GLB_EGRESS_L3_IF_TBL_DATA at 0xf4303004:
 * PPPOE_SET/PPPOE_VLD/PPPOE_SESSION_ID/MAC_SA_VLD/MAC_SA_AN_SEL/PAD_CTRL) and
 * the reference HAL's 24-bit table-entry description for the same table (which
 * names b23 pppoe_len_control instead of PAD_CTRL - same bit, two names).  The
 * entry has NO other fields on this die: there is exactly ONE egress L3-IF word
 * per WAN interface, so the SMAC substitution and the PPPoE header ADD must live
 * in the SAME entry - see l3fe_l3if_entry().
 */
#define L3FE_L3IF_ACCESS		0x3000
#define L3FE_L3IF_DATA			0x3004
#define L3FE_L3IF_ENTRIES		32
#define L3FE_L3IF_PPPOE_SET		BIT(0)
#define L3FE_L3IF_PPPOE_VLD		BIT(1)
#define L3FE_L3IF_PPPOE_SESSION		GENMASK(17, 2)
#define L3FE_L3IF_MAC_SA_VLD		BIT(18)
#define L3FE_L3IF_MAC_SA_AN_SEL		GENMASK(22, 19)
#define L3FE_L3IF_PAD_CTRL		BIT(23)	/* = pppoe_len_control; stock sets it */

/* HW ager cadence for the gated experiment: non-zero so the on-chip ager
 * runs and the lookup's HIT age-re-arm is observable (the SECONDARY hit
 * witness - with granularity 0 the ager block is off and the age slot reads
 * stale).  0x08000000 is the stock-magnitude slow value (~20 min class per
 * decay step), so an idle 2-bit IDLE(1) entry outlives any test window and
 * nf-managed lifetimes never race it. */
#define L3FE_AGING_GRAN_SLOW		0x08000000u

#define L3FE_GO				BIT(31)
#define L3FE_WRITE			BIT(30)
#define L3FE_MASK_UPPER128		BIT(6)
#define L3FE_POLL_TRIES			1000

/* L2FE FDB engine (direct regs, NE window; same protocol as the fdb path in
 * cortina-ni-rx.c).  Used here for the terminating DS-WAN delivery entry:
 * the Venus-family design keeps L2 MY-MAC detection off and routes MyMAC
 * frames to the L3FE via a STATIC FDB entry — stock's FDB holds its WAN MAC
 * -> L3_WAN (0x18).  Without it a PON DS unicast to the WAN MAC is a DLF in
 * the L2FE and gets flooded out instead of delivered. */
#define L3FE_FDB_CMD_RETURN		0x1c2c
#define L3FE_FDB_ACCESS			0x1ca0
#define L3FE_FDB_OP_APPEND		0x45
#define L3FE_FDB_DATA3			0x1ca4
#define L3FE_FDB_DATA2			0x1ca8
#define L3FE_FDB_DATA1			0x1cac
#define L3FE_FDB_DATA0			0x1cb0
#define L3FE_FDB_LPID			GENMASK(5, 0)
#define L3FE_FDB_VALID			BIT(9)
#define L3FE_FDB_STATIC			BIT(19)
#define L3FE_FDB_DA_PERMIT		BIT(20)
#define L3FE_FDB_SA_PERMIT		BIT(21)

/* ------------------------------------------------------------------ *
 *  Transit-frame INGRESS ADMISSION registers (Divergence C).           *
 *                                                                      *
 *  How a routed transit frame physically ENTERS the L3FE on stock      *
 *  (RE of ca-ne.ko ca_l3_intf_create/aal_l3fe_stg0_set_normal +        *
 *  cortina-api route.c/port.c, cross-checked tier-1 live):             *
 *   - LAN side: a static L2FE FDB entry {router MAC} forwards the      *
 *     frame to LDPID 0x19 (L3_LAN pseudo-port); the ARB LDPID->PDPID   *
 *     map resolves 0x19 -> physical port 0x0d = the L3FE LAN ingress.  *
 *   - WAN side: the PON PDC stamps DS data-GEM frames with LDPID 0x18  *
 *     (L3_WAN); the map resolves 0x18 -> physical port 0x0a = the      *
 *     L3FE WAN ingress (stock live: PDPID_MAP[0x18]=0xA [0x19]=0xD,    *
 *     dev/x400axf/stock_golden_qm.txt).                                *
 *   - Inside the L3FE, STG0_LDPID_MAP (0x3404 = 0x03985907) selects    *
 *     LPB profile by HDR_A.ldpid: 0x07->prof0, 0x19->prof1,            *
 *     0x18->prof2; the LPB profile rewrites HDR_I.lspid to L3_WAN      *
 *     (0x18) / L3_LAN (0x19) and picks the T1 classifier profile       *
 *     (WAN=0 / LAN=1) - all already stock-programmed by cortina-ni.    *
 * ------------------------------------------------------------------ */

/* L2FE ARB LDPID->PDPID map (indirect, generic GO protocol): index =
 * {my_mac bit7, dbuf bit6, ldpid[5:0]}, DATA = pdpid[3:0]. */
#define L3FE_L2FE_PDPID_MAP_ACCESS	0x166c
#define L3FE_L2FE_PDPID_MAP_DATA	0x1670
#define L3FE_PDPID_IDX_DBUF		BIT(6)
#define L3FE_PDPID_IDX_MYMAC		BIT(7)
#define L3FE_LDPID_L3_WAN		0x18	/* AAL_LPORT_L3_WAN */
#define L3FE_PDPID_L3_WAN		0x0a	/* AAL_PPORT_L3_WAN (stock live 0xA) */

/*
 * L3FE PP FIELD-CAM - the my-MAC / MAC-DA recognition CAM (15 entries).
 * A frame whose DA matches entry i carries mac_da_an_sel = i+1 in HDR_I
 * (the "routing MAC" recognition the T1 classifier and the STG0 lspid
 * rewrite key on).  Protocol (stock cam_hw_entry_set, disasm-verified):
 * data words to DATA0..DATA4 (0x3214 down to 0x3204), then ACCESS =
 * GO | WRITE | (cam_table_sel << 16) | entry_idx, poll GO clear.
 * MAC-DA entry layout: word0 = mac[2..5] (mac[2] in bits31:24), word1 =
 * vld(bit16) | mac[0]<<8 | mac[1], words 2..4 = 0.
 * NOTE our earlier code wrote only 0x3210/0x3214 - that is the DATA
 * STAGING window; without the 0x3200 ACCESS commit the CAM itself is
 * never written (stock live 0x3214 held an 0x86dd ETHERTYPE residue,
 * proving these are shared staging latches, not entry-0 registers).
 */
#define L3FE_PP_FIELD_CAM_ACCESS	0x3200
#define L3FE_PP_FIELD_CAM_DATA(n)	(0x3214 - (n) * 4) /* DATA0..DATA4 */
#define L3FE_CAM_SEL_MAC_DA		3	/* MAC-DA table select (dport=0) */
#define L3FE_CAM_MAC_DA_ENTRIES		15
#define L3FE_CAM_MAC_DA_VLD		BIT(16)	/* in data word1 */

/* The two router-MAC CAM entries this port provisions: entry 0 = the LAN
 * gateway MAC, entry 1 = the WAN MAC (= base + 1).  The PP stamps
 * HDR_I.mac_da_an_sel = entry + 1 on a DA hit (0 = not a router MAC). */
#define L3FE_AN_IDX_LAN			0
#define L3FE_AN_IDX_WAN			1
#define L3FE_AN_SEL(idx)		((idx) + 1)

/*
 * STG0 per-profile LPB HIGH word (direct MMIO, LOW/MID/HIGH stride 0xC from
 * 0x3408; HIGH0 = 0x3410).  mac_da_an_mask = HIGH[17:10]: bit (mac_idx+1)
 * enables the MAC-DA CAM compare-and-stamp for that ingress profile.
 * mac_da_match_en = HIGH[18] stays 0 (promiscuous: stamp, never filter) -
 * tier-1 stock prof3 HIGH=0x1a1bfd90 decodes an_mask=0xff, match_en=0.
 * Profile use (tier-1 STG0_LDPID_MAP 0x3404=0x03985907): ldpid 0x07->prof0,
 * 0x19 (L3_LAN)->prof1, 0x18 (L3_WAN)->prof2; prof3 = OAM.  The an-mask bits
 * go into prof0/1/2 (both router MACs on every routed ingress class - the
 * mask only widens the promiscuous compare, exactly stock prof3's 0xff).
 */
#define L3FE_STG0_LPB_HIGH(p)		(0x3410 + (p) * 0xC)
#define L3FE_LPB_AN_MASK(sel)		BIT(10 + (sel))
#define L3FE_LPB_AN_PROFILES		3	/* prof 0..2 get the an-mask */

/*
 * L3-CLS classifier KEY table (indirect, same GO protocol as the FIB): 11
 * words, word i at ACCESS + (11 - i)*4 (word0 @ 0x33ac .. word10 @ 0x3384).
 * Partitioned: classifier profile 0 (WAN ingress) = KEY[0..63], profile 1
 * (LAN ingress) = KEY[64..127]; rows 0/1/2 + 64/65/66 hold the stock spcl
 * CPU-trap rows (cortina-ni-rx.c cls_key_golden).  FIB idx = (key_row << 2)
 * | sub_slot.
 */
#define L3FE_CLS_KEY_ACCESS		0x3380
#define L3FE_CLS_KEY_WORDS		11

/* The dedicated pri-6 ROUTED rules live in the first free row of each
 * partition; sub-slot 0. */
#define L3FE_CLS_KEY_ROW_WAN		3
#define L3FE_CLS_KEY_ROW_LAN		67
/* The pri-8 WAN non-IP control trap (PPPoE LCP/IPCP/PAP/CHAP etc) - next
 * free row of the WAN partition. */
#define L3FE_CLS_KEY_ROW_WAN_CTL	4
#define L3FE_CLS_FIB_IDX(key_row)	((key_row) << 2)

/*
 * Stock-armed register values, live-captured 2026-07-18 (tier-1) and
 * mirrored verbatim.  HASH_INI decodes as hb_size=1 (8-way bucket),
 * ht_size=7 (64K entries), ha_width=3 (256-bit FIB), def_reg=1,
 * crc_ntfy_en=1.
 */
#define L3FE_HASH_INI_VAL		0x0003007Du
#define L3FE_DEFAULT_ACTION_VAL		0x0E4D0000u	/* words 0..3; miss -> punt */
#define L3FE_CACHE_INI_VAL		0x00050304u
#define L3FE_CACHE_MISC_VAL		0xA0000000u
#define L3FE_CHK_FAIL_CTRL_VAL		0x0000D0D0u
#define L3FE_AXIM2_CONFIG_VAL		0x000002FFu	/* reset value is 0x200 */
#define L3FE_AQM_TIMER_VAL		0x5000A2D0u	/* reset value is 0x9000A2D0 */
#define L3FE_RSV0_PATCH			(BIT(31) | BIT(0))
#define L3FE_RSV1_PATCH			BIT(0)

/* Poll a self-clearing bit, bounded; 0 on clear, -ETIMEDOUT on cap. */
static int l3fe_poll_clear(void __iomem *ne, u32 off, u32 mask)
{
	int i;

	for (i = 0; i < L3FE_POLL_TRIES; i++) {
		if (!(readl(ne + off) & mask))
			return 0;
		cpu_relax();
	}
	return -ETIMEDOUT;
}

/*
 * ★ fix#93 (2026-08-11) INSTRUMENT - do not remove until the L3FE arm boots clean.
 *
 * Image 135 (the first build in this tree with CONFIG_CORTINA_NI_FLOWOFFLOAD=y, i.e. the
 * first time this file has EVER executed on the board) async-SErrors inside this function:
 *
 *   [2.567] cortina-ni: l3fe: pre-arm MH0=00000000 MA0=00000000 INI=00000000 (expect all 0)
 *   [2.577] SError Interrupt on CPU0, code 0x00000000be000011 -- SError
 *   [2.578] Kernel panic - not syncing: Asynchronous SError Interrupt
 *
 * An ASYNCHRONOUS SError names no address - the CPU is already past the offending write by
 * the time it is taken, which is exactly why nine sessions of this project have been unable
 * to attribute one.  So each arm stage prints an EMERG marker and then SETTLES (dsb + isb +
 * a 200 us window) before the next stage runs: the fault therefore lands inside its OWN
 * stage's window and THE LAST MARKER ON THE CONSOLE IS THE STAGE THAT FAULTED.
 *
 * pr_emerg, not dev_info: the panic path truncates lower levels, and `dmesg -n 1` (which
 * every harness sets) would suppress dev_info entirely - a trap already paid for once in
 * this tree (fix#86).
 */
static void l3fe_arm_stage(int n, const char *what)
{
	dsb(sy);
	isb();
	udelay(200);
	pr_emerg("l3fe-arm: stage %d ENTER: %s\n", n, what);
}

int cortina_l3fe_engine_init(void __iomem *ne, const struct cn_l3e_tables *t)
{
	int ret, i;

	/*
	 * 1. Engine SRAM/table self-init - MUST precede the base/size arm.
	 *    Kick req_sts (bit0), poll its self-clear.
	 */
	l3fe_arm_stage(1, "MEM_INI self-init kick (0x393c <- 1)");
	writel(1, ne + L3FE_HS_MEM_INI);
	ret = l3fe_poll_clear(ne, L3FE_HS_MEM_INI, BIT(0));
	if (ret)
		return ret;

	l3fe_arm_stage(2, "SW-zero the DDR carve (no MMIO)");
	/* 2. SW-zero the DDR tables (belt and braces, matches vendor). */
	memset(t->key_virt, 0, CN_L3E_KEY_TBL_BYTES);
	memset(t->fib_virt, 0, CN_L3E_FIB_TBL_BYTES);
	wmb();	/* coherent carve: make the zeroing visible before the arm */

	l3fe_arm_stage(3, "DDR base regs MH0/MH1/MA0/MA1 (0x383c/3838/3844/3840)");
	/* 3. DDR base registers (physical, 128-byte aligned, [31:7] in
	 * place; hi regs hold phys[39:32]).  Overflow/default/cache bases
	 * stay 0 as on stock. */
	writel(lower_32_bits(t->key_pa), ne + L3FE_HS_BA_MH0);
	writel(upper_32_bits(t->key_pa) & 0xff, ne + L3FE_HS_BA_MH1);
	writel(lower_32_bits(t->fib_pa), ne + L3FE_HS_BA_MA0);
	writel(upper_32_bits(t->fib_pa) & 0xff, ne + L3FE_HS_BA_MA1);

	l3fe_arm_stage(4, "geometry HASH_INI/CACHE_INI/CACHE_MISC (0x3834/...)");
	/* 4. Geometry + cache config (stock-verbatim). */
	writel(L3FE_HASH_INI_VAL, ne + L3FE_HS_HASH_INI);
	writel(L3FE_CACHE_INI_VAL, ne + L3FE_HS_CACHE_INI);
	writel(L3FE_CACHE_MISC_VAL, ne + L3FE_HS_CACHE_MISC);

	l3fe_arm_stage(5, "anti-wedge RSV0/RSV1 + AXIM2_CONFIG (0x3944/3948/3c80)");
	/* 5. Anti-wedge HW patch (do NOT skip: the engine can stall under
	 * DDR read load without it) + AXI outstanding depth. */
	writel(readl(ne + L3FE_HS_RSV0) | L3FE_RSV0_PATCH, ne + L3FE_HS_RSV0);
	writel(readl(ne + L3FE_HS_RSV1) | L3FE_RSV1_PATCH, ne + L3FE_HS_RSV1);
	writel(L3FE_AXIM2_CONFIG_VAL, ne + L3FE_AXIM2_CONFIG);

	l3fe_arm_stage(6, "CHK_FAIL_CTRL + HS_DEFAULT_ACTION table (0x3940/3860..)");
	/* 6. Miss/fail never drops: double-check-fail punt + the internal
	 * default (miss) actions, def_reg=1 mode - stock programs words
	 * 0..3, the rest stay 0. */
	writel(L3FE_CHK_FAIL_CTRL_VAL, ne + L3FE_HS_CHK_FAIL_CTRL);
	for (i = 0; i < 4; i++)
		writel(L3FE_DEFAULT_ACTION_VAL, ne + L3FE_HS_DEFAULT_ACTION(i));

	l3fe_arm_stage(7, "AGING_GRANULARITY 0 (0x3924)");
	/* 7. HW auto-age-countdown OFF (stock): hardware must never age a
	 * flow out from under the Linux flowtable.  Liveness = HW hit-rearm
	 * + the SW sweep; lifetime = nf gc + FLOW_CLS_DESTROY. */
	writel(0, ne + L3FE_HS_AGING_GRANULARITY);

	l3fe_arm_stage(8, "AQM_TIMER (0x3aa8)");
	/* 8. AQM flow-stat timer, stock-verbatim. */
	writel(L3FE_AQM_TIMER_VAL, ne + L3FE_AQM_TIMER);
	l3fe_arm_stage(9, "ARM COMPLETE - no stage faulted");

	return 0;
}

/*
 * Profile/tuple + mask-table classify config, tier-1 captured live from the
 * stock-armed engine (RTK_GW 5.10.226, 2026-07-18; the exact register dump is
 * in dev/x400axf/stock_l3fe_dump_full.txt).  Programming this makes the
 * main-hash engine parse/key a routed packet exactly like stock: profiles 0-5
 * (INI/tuple/type-action) select mask-table entries 0-7 (the in-use masks;
 * 8-63 are uninitialised SRAM on stock too).  PF_KEY/TPL stay 0 -
 * hash_key_select is .bss (=0) on stock, so there is no per-profile CRC
 * rotate.  STG0/LDPID (0x3400/0x3404) already match stock via cortina-ni.c.
 *
 * ★ P2 FINDING (2026-07-18) - this is NECESSARY but NOT SUFFICIENT for a HW
 * hit.  On our datapath LAN->WAN packets are SOFTWARE-forwarded (Linux
 * routing / the nf_flowtable SW fast path - conntrack shows [OFFLOAD]); they
 * never enter the NE L3FE HW L3-forwarding path, so the main hash is not
 * consulted.  Proven on hardware: 1.9M matching packets with SWO-correct
 * entries installed in ALL 8 profile/mask buckets -> zero age re-arm.  The
 * remaining work is to steer routed packets into the NE L3FE lookup in HW
 * (HW L3-forwarding with miss->CPU-trap), a datapath piece beyond this table
 * config; until then the offload install stays gated OFF.  Programming the
 * config is runtime-verified to NOT regress the datapath (WAN 0% loss).
 */
/*
 * Masks 0-7 = the stock classify masks (tier-1 captured).  ★ Mask 8 = a
 * dedicated 5-TUPLE-ONLY NAPT mask, added for the HW-L3-forward hit path
 * (P3, 2026-07-19): stock mask 0 keeps far more than the 5-tuple (it also
 * folds mac_sa/mac_da/lspid/ip_dscp/ip_ecn/VLAN/PPPoE - all non-zero on a
 * real routed frame but zero in the driver's synthetic 5-tuple key), so an
 * install-time CRC computed from a sparse key can never equal a parsed
 * packet's lookup-time CRC under mask 0.  Mask 8 EXCLUDES everything except
 * {l4_dp, l4_sp, ip_da/32, ip_sa/32, ip_protocol, ip_ver, ip_vld} (mask bit
 * 1 = EXCLUDE), so a sparse 5-tuple key hashes identically to a parsed
 * packet.  Live on-board probes (swolearn / the driver's own boot-time
 * key-packing liveness test) confirm which HDR_I offsets feed the hash:
 * {233-264 DA, 361-392 SA, 492-499 proto} MOVE the CRC, {116 dscp,
 * 600/700/726 mac/lspid} do NOT; the port pair {74 dport, 90 sport} is
 * tier-2 confirmed from the stock HDR_I packer and re-enters the hash with
 * the exact-port mask fix below.  The routed profiles' TUPLE
 * maskptr is re-pointed at mask 8 by cortina_l3fe_hw_l3_forward_enable()
 * (gated); gate-off leaves the profiles on the stock masks, so programming
 * this spare index changes no datapath behaviour.
 */
#define L3FE_MASK_5TUPLE	8	/* 5-tuple-only NAPT mask index */

/*
 * ★ Mask-entry field geometry, L4 region (needed by the exact-port invariant
 * below).  A mask entry is 224 meaningful bits, one FIELD (not one bit) per
 * hashable key field, LSB-first over the entry:
 *
 *   [0]     l4_chksum_zero      [9:1]   tcp/rdp control flags
 *   [26:10] l4_dp  - 17 bits    [43:27] l4_sp  - 17 bits
 *   [44]    l3_chksum_err       ... [56:55] ip_ttl (2-bit enum)
 *   [66:58] ip_da keep-length   [75:67] ip_sa keep-length
 *   [77]    ip_protocol         [86] ip_ver   [87] ip_vld
 *
 * ★★ Each L4-port field is 17 bits for a 16-bit port, and the extra top bit
 * (field bit 16 = entry bit 26 for dp, 43 for sp) is a MODE SELECT, not a
 * mask bit:
 *   field == 0                  -> EXACT match: the 16-bit port value itself
 *                                  enters the hash tuple (nothing masked).
 *   field == (1<<16) | ~rng_vec -> RANGE match: the tuple takes the parser's
 *                                  port-RANGE-match vector instead of the port
 *                                  value, and the low 16 bits mask which range
 *                                  slots matter.
 * Evidence: tier-3 the vendor classifier's two branches build exactly those
 * two shapes ("one l4 port" -> mask field 0; "l4 port range" -> a port-range
 * CAM entry plus mask field (1<<16)|~(1<<slot)); tier-1 the eight stock masks
 * captured live agree - the two that keep the ports (0 and 7) have BOTH port
 * fields all-zero, and every mask that drops the ports has them all-ones.
 */
#define L3FE_MASK_L4_DP_LSB	10	/* l4_dp field  = entry bits [26:10] */
#define L3FE_MASK_L4_SP_LSB	27	/* l4_sp field  = entry bits [43:27] */
#define L3FE_MASK_L4_PORT_BITS	17	/* 16 value bits + bit16 = range mode */

/*
 * Mask 8 words, named so the exact-port invariant below is a BUILD-time check.
 * Word i covers entry bits [32i+31 : 32i].
 */
#define L3FE_MASK5_W0		0x000003ffu
#define L3FE_MASK5_W1		0x827ff000u
#define L3FE_MASK5_W2		0xff3fd100u
#define L3FE_MASK5_W3		0xffffffffu

/*
 * ★ INVARIANT D (build-time): the 5-tuple mask MUST select EXACT L4-port
 * match, i.e. both 17-bit port fields all-zero.  Setting only the range-mode
 * bit - the 2026-07-19..24 value 0x040003ff/0x827ff800 - is a legal encoding
 * that silently swaps the port VALUE out of the hash tuple for the parser's
 * range-match vector: the boot key-packing test then reports "dport/sport did
 * NOT move the CRC", two NAPT flows differing only in their ports collide on
 * one entry (and the post-hit double-check, which re-derives the hash under
 * this same mask, cannot tell them apart), and matching a real frame starts to
 * depend on the port-range CAM SRAM, which this driver never programs.  Guard
 * it here so an edit fails the build, not a benchmark.
 */
static_assert((L3FE_MASK5_W0 & GENMASK(31, L3FE_MASK_L4_DP_LSB)) == 0,
	      "5-tuple mask word0: l4_dp/l4_sp fields must be 0 = exact-port match");
static_assert((L3FE_MASK5_W1 &
	       GENMASK(L3FE_MASK_L4_SP_LSB + L3FE_MASK_L4_PORT_BITS - 1 - 32, 0)) == 0,
	      "5-tuple mask word1: l4_sp field top bits must be 0 = exact-port match");
static const u32 l3fe_mask_lo[9][4] = {
	{ 0x000003ff, 0x0221f000, 0x15001402, 0xc0f03fe1 },
	{ 0xffffffff, 0x027fffff, 0x1f403000, 0xc0f01fe1 },
	{ 0xffffffff, 0x027fffff, 0xff3ff002, 0xffffffff },
	{ 0xffffffff, 0x027fffff, 0x1f003402, 0xffffffe1 },
	{ 0xffffffff, 0x027fffff, 0x1f003402, 0xffffffe1 },
	{ 0xffffffff, 0x027fffff, 0xfffff000, 0xc0ffffff },
	{ 0xffffffff, 0x027fffff, 0xff7ff000, 0xffffffff },
	{ 0x000003ff, 0x0221f000, 0x15001402, 0xc0f03fe1 },
	/*
	 * ★ mask 8: 5-TUPLE-ONLY NAPT mask, DERIVED FROM THE aal-77c
	 * aal_hash_mask_t field table (the chip's own tree - proven by the
	 * ca-ne.ko symbol aal_hash_add_with_no_crc_calulate; NOT aal-gen2),
	 * decode-validated bit-for-bit against the two stock masks that work
	 * (mask 0 = NAT, mask 1 = bridge).  Polarity: 1 = EXCLUDE a field, 0 =
	 * KEEP; ip_da/ip_sa are a 9-bit KEEP-LENGTH (0x080 = /128, clamps to /32
	 * for IPv4); ip_ttl is a 2-bit enum where 0/1 = exclude, 2/3 = keep.
	 *
	 * KEEP (mask field = 0): l4_dp (all 17 bits = EXACT-port match, see
	 * INVARIANT D above), l4_sp likewise, ip_protocol, ip_ver, ip_vld;
	 * ip_da / ip_sa keep-length = 0x020 (= keep the top 32 bits = the IPv4
	 * address in ip_xa_0).  EXCLUDE everything else (= 1), including ip_ttl
	 * (enum forced to 0).  This exact 224-bit value was derived TWO
	 * independent ways that AGREE bit-for-bit on all meaningful bits
	 * [219:0]: (a) built from the aal-77c aal_hash_mask_t field table,
	 * (b) an independent Ghidra/source extraction of hash_value_calculate.
	 *
	 * ★ FIX 2026-07-25 (word0 0x040003ff -> 0x000003ff, word1 0x827ff800 ->
	 * 0x827ff000): the previous value set field bit 16 of BOTH port fields,
	 * reading it as "exclude the port's range/exact flag".  It is not a mask
	 * bit but the RANGE-MODE SELECT (INVARIANT D), so the hash tuple carried
	 * the parser's 16-bit port-range-match vector INSTEAD of the port value.
	 * Symptoms it produced: the boot key-packing liveness test reporting
	 * "dport/sport did NOT move the CRC (masked-out)", flows differing only
	 * in their L4 ports aliasing onto one entry, and install-vs-lookup CRC
	 * agreement made conditional on port-range CAM SRAM this driver never
	 * programs (so a frame whose ports hit a stale range entry can never
	 * match a sparse install - a direction-dependent silent miss).
	 *
	 * ★ Fixes vs the previous hand-built "mask0|~mask1" value
	 * (0x000003ff,0xffa1f000,0xffbfdfff,0xffffffff), whose decode revealed
	 * three bugs that made the sparse-install CRC diverge from the HW
	 * parsed-frame CRC (board-measured 2026-07-23: install c9b5981e/5fc9 vs
	 * HW crc_ntfy tap 6667e6d3/5a64):
	 *   1. ip_ttl enum = 3 -> KEPT the TTL (the parser sets the real TTL, the
	 *      sparse HDR_I leaves 0) AND armed the TTL>10 -> AAL_E_OUTRANGE
	 *      range-check.  Now 0 (excluded).
	 *   2. ip_da/ip_sa keep-length = 511 (malformed: suffix-flag set + len
	 *      255).  Now 0x020 (prefix, keep top 32 = the v4 addr).
	 *   3. kept ipv6_doh/rh/hbh, ip_fragment_flag, ip_options - all
	 *      parser-set, zero in the sparse build.  Now excluded.  (The L4
	 *      port fields' 17th bit was excluded there too; that part was itself
	 *      wrong - see the FIX note above.)
	 * PPPoE session/type, dscp/ecn, vlan, MAC SA/DA, lspid, l3_chksum: all
	 * excluded, so a real routed frame hashes identically to the driver's
	 * sparse 5-tuple install.  Only reachable under hw_l3_fwd (routed
	 * profiles' maskptr re-pointed here); gate-off leaves the stock masks
	 * untouched.  (The DOUBLE-CHECK is separate and already correct: the
	 * action's chk_msk_ptr1 = this mask id 8 + cache_ctrl1 = 1 - aal-77c
	 * GROUP_20; the chip's flow FIB never fetches GROUP_21/chk_hash_val, so
	 * no xor32 is needed.)
	 */
	{ L3FE_MASK5_W0, L3FE_MASK5_W1, L3FE_MASK5_W2, L3FE_MASK5_W3 },
};
static const u32 l3fe_mask_hi[9][4] = {
	{ 0xffff807f, 0xffffffff, 0xfeffffff, 0xffffffff },
	{ 0xffff807f, 0xffffffff, 0xfeffffff, 0xffffffff },
	{ 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff },
	{ 0xffff807f, 0xffffffff, 0xfeffffff, 0xffffffff },
	{ 0x0000007f, 0x00000000, 0xfeffff80, 0xffffffff },
	{ 0xffff807f, 0xffffffff, 0xffffffff, 0xffffffff },
	{ 0xffffffff, 0xffffffff, 0xfefffeff, 0xffffffff },
	{ 0x007f807f, 0xff800000, 0xfeffffff, 0xffffffff },
	/* mask 8: 5-tuple only - exclude all L2/lspid/dscp/vlan/pppoe */
	{ 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff },
};
/* {NE-relative offset, value} - profile INI/tuple/type-action, stock-verbatim */
static const u32 l3fe_profile_regs[][2] = {
	{ 0x3700, 0x00000001 }, { 0x3724, 0x06140000 }, { 0x3728, 0x06000000 },
	{ 0x372c, 0x00000001 }, { 0x3730, 0x00000001 }, { 0x3750, 0x06140000 },
	{ 0x3754, 0x06000000 },
	{ 0x3758, 0x00004012 }, { 0x375c, 0x00000002 }, { 0x3760, 0x00000103 },
	{ 0x377c, 0x06140000 }, { 0x3780, 0x06000000 },
	{ 0x3784, 0x00084211 }, { 0x3788, 0x00000004 }, { 0x37a8, 0x06140000 },
	{ 0x37ac, 0x06000000 },
	{ 0x37b0, 0x00084211 }, { 0x37b4, 0x00000005 }, { 0x37d4, 0x06140000 },
	{ 0x37d8, 0x06000000 },
	{ 0x37dc, 0x00000002 }, { 0x37e0, 0x00000006 }, { 0x37e4, 0x00000107 },
	{ 0x3800, 0x06140000 }, { 0x3804, 0x06000000 },
};

int cortina_l3fe_classify_setup(void __iomem *ne)
{
	int i, ret;

	/* masks 0-7 = stock; mask 8 = the spare 5-tuple NAPT mask (unused
	 * unless a profile's maskptr is re-pointed at it under hw_l3_fwd) */
	for (i = 0; i < 9; i++) {
		ret = cortina_l3fe_mask_write(ne, i, l3fe_mask_lo[i],
					      l3fe_mask_hi[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_profile_regs); i++)
		writel(l3fe_profile_regs[i][1],
		       ne + l3fe_profile_regs[i][0]);
	return 0;
}

int cortina_l3fe_mask_write(void __iomem *ne, u32 idx,
			    const u32 lo[4], const u32 hi[4])
{
	int i, ret;

	/* lower 128 bits: data words then the GO|W commit */
	for (i = 0; i < 4; i++)
		writel(lo[i], ne + L3FE_HS_MASK_DATA(i));
	writel(L3FE_GO | L3FE_WRITE | (idx & 0x3f), ne + L3FE_HS_MASK_ACCESS);
	ret = l3fe_poll_clear(ne, L3FE_HS_MASK_ACCESS, L3FE_GO);
	if (ret)
		return ret;

	/* upper 128 bits: second beat with bit6 */
	for (i = 0; i < 4; i++)
		writel(hi[i], ne + L3FE_HS_MASK_DATA(i));
	writel(L3FE_GO | L3FE_WRITE | L3FE_MASK_UPPER128 | (idx & 0x3f),
	       ne + L3FE_HS_MASK_ACCESS);
	return l3fe_poll_clear(ne, L3FE_HS_MASK_ACCESS, L3FE_GO);
}

int cortina_l3fe_swo_crc(void __iomem *ne, const u32 *words, int nwords,
			 u32 mask_id, u32 *crc32_out, u16 *crc16_out)
{
	int i, ret;

	if (nwords < 1 || nwords > 32)
		return -EINVAL;

	/* key words at SWO index 0.. (DAT auto-increments IDX) */
	writel(0, ne + L3FE_HS_SWO_IDX);
	for (i = 0; i < nwords; i++)
		writel(words[i], ne + L3FE_HS_SWO_DAT);

	/* mask pointer at SWO index 32 */
	writel(32, ne + L3FE_HS_SWO_IDX);
	writel(mask_id, ne + L3FE_HS_SWO_DAT);

	/* run: bit0 = go/busy (dedicated, not the bit31 protocol) */
	ret = l3fe_poll_clear(ne, L3FE_HS_SWO_CTRL, BIT(0));
	if (ret)
		return ret;
	writel(1, ne + L3FE_HS_SWO_CTRL);
	ret = l3fe_poll_clear(ne, L3FE_HS_SWO_CTRL, BIT(0));
	if (ret)
		return ret;

	/* results: SWO index 33 = CRC32, 34 = CRC16 (read-only slots) */
	writel(33, ne + L3FE_HS_SWO_IDX);
	*crc32_out = readl(ne + L3FE_HS_SWO_DAT);
	writel(34, ne + L3FE_HS_SWO_IDX);
	*crc16_out = readl(ne + L3FE_HS_SWO_DAT) & 0xffff;
	return 0;
}

/* ------------------------------------------------------------------ *
 *  Divergence B: enable HW L3-forwarding (miss -> CPU).               *
 * ------------------------------------------------------------------ */

/*
 * Internal hash-miss action FIB, tier-1 captured from the stock-armed engine
 * (2026-07-18, running HW-NAT: /proc/fc/ctrl/hwnat=1).  With
 * HASH_INI.def_reg=1 the main-hash lookup fetches the MISS action from these
 * registers, NOT a DDR table; our engine_init left them 0, so a miss (which
 * happens for EVERY packet until a flow is installed) had a null action.
 * Entries 1..3 mirror the stock capture (other miss types an empty-bucket
 * zero-flow lookup never selects).
 *
 * ★ Entry 0 = TRAP-TO-CPU_0, our deliberate deviation from the stock capture.
 * The tier-1 stock entry 0 decodes to permit=0 (DROP) with keep_ts set —
 * stock traps its terminating DS-WAN traffic (DHCP/ICMP-to-router/ARP) to the
 * CPU via the L3FE spcl-packet handler + per-interface CLS rules keyed on the
 * router's own IP, which this port does not replicate (it uses the CLS
 * per-profile DEFAULTS only).  Without those, a terminating DS frame that
 * enters the L3FE (PDC ldpid L3_WAN, fe_bypass=0) falls to the hash miss and
 * stock's permit=0 entry 0 DROPS it — verified on the board: frames enter the
 * L3FE (l3fe_rx climbs) but the CPU-RX spy never fires and the DHCP OFFER
 * never reaches gpon0.  For our nf_flow_table model the miss MUST punt to the
 * CPU (the first packet of any flow, and all terminating traffic, is software-
 * handled), so entry 0 is a plain {permit, dpid_vld, dpid_pri, mcgid=CPU_0
 * (0x10), mc=0} trap = the SAME dpid action the CLS per-profile defaults carry
 * (l3fe_cls_default row 1024, decode-verified).  With install gated OFF this
 * makes every DS frame miss -> trap to CPU_0 -> gpon0 = the zero-flow no-
 * regression path.  This table is written only inside the hw_l3_fwd-gated
 * cortina_l3fe_hw_l3_forward_enable(), so gate-off behaviour is unchanged.
 */
static const u32 l3fe_def_reg_stock[L3FE_HS_DEF_REG_COUNT] = {
	/* entry 0: TRAP the hash miss to CPU_0 (permit|dpid_vld|dpid_pri|mcgid=
	 * 0x10, mc=0) — the SAME dpid action the CLS per-profile defaults carry
	 * (l3fe_cls_default row 1024).  ON-BOARD PROOF: with this entry a routed
	 * LAN my-MAC miss reaches the CPU (SSH stays up with admission on), so the
	 * L3FE hash-miss -> CPU_0 -> CPU-EPP path is live and correct.  (An attempt
	 * to re-point it to the deep-queue ldpid 0x32 instead BROKE LAN CPU-RX, so
	 * CPU_0 is the right delivery.)  The earlier "PON-sourced ldpid-0x18 frame
	 * dies before the hash / at the L3QM CPU pool" observation is RESOLVED
	 * (2026-07-19): those DS frames never entered the L3FE at all — the WAN
	 * MAC had no L2FE FDB entry, so they were DLF-FLOODED out (l2fe_ni/bm_tx
	 * climbed, l3fe_rx stayed 0).  Fixed by the static FDB WAN-MAC -> L3_WAN
	 * entry (l3fe_fdb_static_add / cortina_ni_rx_fdb_add_cpu), after which
	 * DS-WAN unicast delivers 0-loss end to end. */
	/* ★ word1 bits 22/23 = keep_orig_pkt_vld|keep_orig_pkt (0x00C00000).  EVERY
	 * vendor to-CPU miss action carries them (aal-gen2/aal_hash.c
	 * WAN/LAN/MC/CLS _HASH_ACT_DEFAULT_TO_CPU) so the punt is delivered VERBATIM;
	 * without them the miss frame is rebuilt by the PE (edited/decapsulated) and
	 * dies at/after CPU RX - which is exactly why enabling the hash-consult
	 * BROKE the routed LAN->WAN path at zero flows (same class as the board-proven
	 * PPPoE-LCP mangling that keep_orig_pkt fixed). */
	0x00000000, 0x00300000, 0x00000000,	/* entry 0: stock hash-miss (NO dpid; the CLS row supplies the CPU port - tier-1 devmem 2026-07-23) */
	/* ★ entry 1 = TRAP -> CPU_0 too (deviation from the stock capture
	 * 0x00009811): profile 3's INI (0x3784 = 0x00084211) points all four
	 * default_sel nibbles at DEFAULT_ACTION entry 1, and the gated routed
	 * admission now stamps t2_ctrl=3 on the LAN catch-all - so a profile-3
	 * T2 MISS (= every ONU-terminating/unoffloaded LAN frame, incl. SSH)
	 * resolves through entry 1.  Stock's 0x9811 is its bridge-flow miss
	 * action; ours MUST punt to CPU_0 or gate-on would black-hole LAN
	 * management on the first miss. */
	0x00009811, 0x00000000, 0x00000000,	/* entry 1: stock bridge/flood miss action 0x9811 (tier-1 devmem 2026-07-23) */
	0x00040001, 0x00300000, 0x00404000,	/* entry 2 */
	0x00009831, 0x00300000, 0x00000000,	/* entry 3 */
};

/*
 * CLS per-profile routing DEFAULT actions - the hash-CONSULT enable.  A routed
 * frame that matches no specific CLS rule falls through to the profile default
 * at idx (max_entry-16)|(profile<<2)|((rslt_type&1)<<1) - ★ that indexing
 * scheme is REFUTED, see the correction note directly above the table.  Per the
 * ca-ne.ko RE
 * (convert_intf_to_cls route.c / cls_type_1_default_set classifier.c) the CLS
 * result's t2_ctrl field is what points HDR_I at the T2 main hash: 0 = WAN
 * hash profile, 1 = LAN hash profile, 0xF = BYPASS.  All rows keep the stock
 * CPU-fallback {permit, dpid_pri, mcgid=CPU_0} so a hash MISS still traps to
 * the CPU.  word0..word6 (word0 = DATA0 at 0x33cc), struct-decode verified.
 *
 * All three rows = EXACT stock bytes.  (An earlier build forced t2_ctrl_vld=1
 * on row 1028 - REFUTED as a lever: the default rows are dead behind the
 * all-wildcard trap row KEY[66]/KEY[2], and T2 admission for routed traffic
 * is the dedicated pri-6 mac_da_an_sel rule (cortina_l3fe_intf_add), exactly
 * stock's ca_l3_intf_add scheme.)
 */
/*
 * ★★ INDICES CORRECTED 2026-07-25 (tier-2) - the rows above are right, the
 * addresses were not.  There is no "default FIB region" above the real entries:
 * the CLS FIB has 512 entries (stock's own table descriptor: max_entry=0x200,
 * entry_size=0x1c, data_reg_num=7, ACCESS=0x33b0 - the last three match this
 * driver's offsets, so the 512 is authoritative), aal_l3_cls_default_set is a
 * stub on this die that logs "not support default fib anymore", and every FIB
 * index is (key_row << 2) | sub_slot.  The old 1024/1025/1028 exceeded the
 * 9-bit HW address field and ALIASED onto FIB[0]/[1]/[4] - and FIB[4] is the WAN
 * partition's routed-unicast action, exactly the row the downstream transit path
 * needs.  It was harmless ONLY because these words are byte-identical to the
 * golden rows cortina-ni-rx.c already programs at 0/1/4; a single edit to either
 * table would have turned it into a silent WAN-only breakage indistinguishable
 * from a hash bug.  Writing the real indices makes the target explicit and
 * keeps the programmed bytes bit-for-bit unchanged (verified against
 * cls_fib_golden[] FIB[0]/[1]/[4]), so this is a no-op on the datapath and no
 * longer depends on cls_trap_enable having run first.
 */
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; } l3fe_cls_default[] = {
	{ 0, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* WAN KEY[0] slot0 (was 1024) */
	{ 1, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* WAN KEY[0] slot1 (was 1025) */
	{ 4, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },	/* WAN KEY[1] slot0 (was 1028) */
};

/*
 * One CLS-FIB indirect write: words then ACCESS=GO|WR|idx, poll GO clear.
 *
 * ★★ RANGE CHECK (added 2026-07-25, tier-2).  The CLS FIB has 512 entries: the
 * stock module's own table descriptor reads max_entry=0x200, entry_size=0x1c,
 * data_reg_num=7, ACCESS=0x33b0 - and the last three cross-check against the
 * offsets this driver already uses, so the 512 is authoritative.  The ACCESS
 * address field decodes only 9 bits, so an index >= 512 does NOT fail: it
 * ALIASES onto idx & 511 and quietly overwrites a live row.  This driver was
 * writing 1024/1025/1028 (see the removed l3fe_cls_default[] note at the
 * caller), which alias onto FIB[0]/[1]/[4] - and FIB[4] is the WAN partition's
 * routed-unicast action, the one the downstream transit path depends on.  It
 * happened to be harmless only because the words written were byte-identical to
 * the golden rows already programmed there; any edit to either table would have
 * turned it into a silent WAN-only breakage that looks exactly like a hash bug.
 * Refuse loudly instead: the old premise (a separate "default FIB" region above
 * the real entries) is false - aal_l3_cls_default_set is a stub on this die that
 * logs "not support default fib anymore" and returns 0, and every FIB index is
 * (key_row << 2) | sub_slot.
 */
#define L3FE_CLS_FIB_ENTRIES	512

static int l3fe_cls_fib_write(void __iomem *ne, u16 idx, const u32 w[L3FE_CLS_FIB_WORDS])
{
	int i;

	if (idx >= L3FE_CLS_FIB_ENTRIES) {
		pr_err("cortina-l3fe: CLS FIB idx %u out of range (max %u) - the HW address field is 9 bits and would ALIAS onto FIB[%u], silently overwriting a live row; refusing\n",
		       idx, L3FE_CLS_FIB_ENTRIES - 1,
		       idx & (L3FE_CLS_FIB_ENTRIES - 1));
		return -ERANGE;
	}
	for (i = 0; i < L3FE_CLS_FIB_WORDS; i++)
		writel(w[i], ne + L3FE_CLS_FIB_DATA0 - i * 4);
	writel(L3FE_GO | L3FE_WRITE | idx, ne + L3FE_CLS_FIB_ACCESS);
	return l3fe_poll_clear(ne, L3FE_CLS_FIB_ACCESS, L3FE_GO);
}

/* Commit one PP FIELD-CAM MAC-DA entry (proper ACCESS commit - see the
 * block comment at the register defines). */
static int l3fe_mac_da_cam_set(void __iomem *ne, u32 idx, const u8 *mac)
{
	if (idx >= L3FE_CAM_MAC_DA_ENTRIES)
		return -EINVAL;

	writel(((u32)mac[2] << 24) | ((u32)mac[3] << 16) |
	       ((u32)mac[4] << 8) | mac[5], ne + L3FE_PP_FIELD_CAM_DATA(0));
	writel(L3FE_CAM_MAC_DA_VLD | ((u32)mac[0] << 8) | mac[1],
	       ne + L3FE_PP_FIELD_CAM_DATA(1));
	writel(0, ne + L3FE_PP_FIELD_CAM_DATA(2));
	writel(0, ne + L3FE_PP_FIELD_CAM_DATA(3));
	writel(0, ne + L3FE_PP_FIELD_CAM_DATA(4));
	writel(L3FE_GO | L3FE_WRITE | (L3FE_CAM_SEL_MAC_DA << 16) | idx,
	       ne + L3FE_PP_FIELD_CAM_ACCESS);
	return l3fe_poll_clear(ne, L3FE_PP_FIELD_CAM_ACCESS, L3FE_GO);
}

/* One CLS-KEY indirect write: 11 words then ACCESS=GO|WR|idx, poll GO clear. */
static int __maybe_unused l3fe_cls_key_write(void __iomem *ne, u16 idx,
					     const u32 w[L3FE_CLS_KEY_WORDS])
{
	int i;

	for (i = 0; i < L3FE_CLS_KEY_WORDS; i++)
		writel(w[i], ne + L3FE_CLS_KEY_ACCESS +
		       (L3FE_CLS_KEY_WORDS - i) * 4);
	writel(L3FE_GO | L3FE_WRITE | (idx & 0x7ff), ne + L3FE_CLS_KEY_ACCESS);
	return l3fe_poll_clear(ne, L3FE_CLS_KEY_ACCESS, L3FE_GO);
}

/*
 * ★ The dedicated pri-6 ROUTED CLS rules - the piece that RUNS T2 (the main-
 * hash lookup) on a routed my-MAC transit frame.  Clean-room re-expression of
 * stock's ca_l3_intf_add / convert_intf_to_cls scheme (RE of ca-ne.ko
 * aal_l3_cls_add@0x821e0 + the aal-gen2 cl_if_id_key_t layout, corroborated
 * tier-1 by the golden trap rows' observed encoding):
 *
 * KEY (cl_if_id_key_t, 4 x 83-bit sub-keys + trailer; don't-care = msk-bit 1
 * with the value bits all-1, exactly the golden rows' convention; only
 * sub-slot 0 valid):
 *   - mac_da_an_sel == AN_SEL(idx) EXACT (msk=0): only frames whose DST-MAC
 *     hit the router-MAC CAM - mutually exclusive with the mac_da_an_sel==0
 *     L2UC catch-all, so a routed frame can no longer fall into the L2FE
 *     bridging disposition;
 *   - lspid == L3_LAN 0x19 (LAN rule) / L3_WAN 0x18 (WAN rule) EXACT;
 *   - ip/L4/VLAN/PPPoE all don't-care (a transit frame's ip.da is the far
 *     end, NOT this box - terminating traffic is resolved by T2-MISS -> the
 *     HS_DEF CPU_0 punt, never dropped);
 *   - trailer: cls_pri = 6 (CL_RUL_PRIO_L3_INTF_BCAST - beats the pri-0/1
 *     catch-alls, below the pri-7..11 ARP/BC/spcl traps a transit unicast
 *     doesn't key), rslt_type 0, key_type IF_ID (0), valid = slot0.
 *
 * ACTION (FIB word6): t2_ctrl_vld=1 (bit11) + t2_ctrl = main-hash profile
 * (bits15:12 - LAN=1 -> 0x1A00, WAN=0 -> 0x0A00, the tier-1 stock routing-
 * default bytes) + word5 bit26 stage2_ctrl_vld with stage2_ctrl=UPDATE(0)
 * (the NAT edit stage).  ★ NO permit / dpid / mcgid / keep_orig_pkt:
 * forwarding is left to the T2 HIT action; a MISS falls to the HS_DEF
 * default action = the CPU_0 trap (l3fe_def_reg_stock entry 0).  A full
 * pre-resolved forwarding disposition here would suppress the T2 lookup
 * (why stamping t2_ctrl on the dispositioned catch-all rows was inert).
 */
/* Refuted (tier-1 2026-07-23): stock leaves CLS rows 3/67 empty; kept
 * __maybe_unused until the golden-row t2_ctrl approach is board-proven. */
static const struct { u16 idx; u32 w[L3FE_CLS_KEY_WORDS]; }
	l3fe_cls_routed_key[] __maybe_unused = {
	/* WAN ingress: an_sel=2 (WAN MAC, CAM idx 1), lspid=0x18, pri 6 */
	{ L3FE_CLS_KEY_ROW_WAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00030FE4, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/* LAN ingress: an_sel=1 (LAN gateway MAC, CAM idx 0), lspid=0x19, pri 6 */
	{ L3FE_CLS_KEY_ROW_LAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00032FE2, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/*
	 * ★ WAN NON-IP CONTROL TRAP (the PPPoE LCP un-mangle fix).  Key:
	 * {ip_vld == 0 EXACT}, everything else don't-care; pri 8 (= stock
	 * CL_RUL_PRIO_L3_TUNNEL_PPPOE - above the pri-6 routed rule, below the
	 * pri-9 IP-multicast traps); slot 0 only.  This row lives in the WAN
	 * partition (KEY[0..63]) so it is searched only by WAN-ingress frames.
	 *
	 * WHY: with hw_l3_fwd on, a DS 0x8864 PPPoE frame whose inner PPP proto
	 * is a CONTROL proto (LCP 0xc021 / IPCP 0x8021 / IPV6CP 0x8057 / PAP
	 * 0xc023 / CHAP 0xc223) rode the pri-6 routed row into T2, missed, and
	 * the HS_DEF CPU punt re-emerged from the PE with the PPP proto
	 * mangled / the frame decapsulated, so pppd never saw the LCP frame and
	 * the session could not establish (BOARD-PROVEN 2026-07-20: DS LCP
	 * Conf-Req/Ack left hades on-wire but reached pppd 0/0; a keep_orig CPU
	 * trap delivered them and LCP+IPCP completed to a 10.99.99.x lease).
	 * All control protos parse ip_vld=0 (no inner IP) while 0x0021/0x0057
	 * session DATA parses ip_vld=1 - so ip_vld==0 is exactly the control-
	 * vs-data split and this row steals NOTHING from the L3FE data path
	 * (IPoE and PPPoE DATA both carry ip_vld=1 and keep the routed pri-6 ->
	 * T2 path).  Mirrors the stock LCP/IPCP scheme (cortina-api
	 * classifier.c cls_rule_add aal_customize CA_CLASSIFIER_AAL_L3_PPP_LCP/
	 * _IPCP/_IP6CP: trap to CPU with keep_orig_pkt=1) WITHOUT the PPP-proto
	 * CAM: ip_vld==0 covers every control proto in one row.  Non-IP non-
	 * PPPoE WAN frames (ARP, 0x8863 Discovery) also land here - they were
	 * already CPU-bound (T2 can never hit a frame with no 5-tuple), now
	 * just delivered with original bytes (no PE edit).  ★ lspid==0x18 was
	 * NOT added to the key: board-proven, these DS control frames do NOT
	 * carry lspid 0x18 at the CLS (adding it dropped every match); ip_vld==0
	 * in the WAN partition is already WAN-scoped and sufficient.  Word
	 * build: word0 0xFFFFFCFF = msk_ip_vld=0/ip_vld=0 (bits 8/9), rest
	 * don't-care (word2 0x0007FFFF like the WAN all-wildcard trap); trailer
	 * pri=8 valid=slot0 (0x08200000).
	 */
	{ L3FE_CLS_KEY_ROW_WAN_CTL, { 0xFFFFFCFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
				      0, 0, 0, 0, 0, 0x08200000 } },
};
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; }
	l3fe_cls_routed_fib[] __maybe_unused = {
	/* ★ CARRY the to-CPU disposition ON THE CLS ROW (w4 = permit|dpid_pri|
	 * dpid_vld 0x1C000000; w5 = mcgid CPU_0 + stage3_ctrl_vld 0x01000004),
	 * NOT on the HS_DEF miss action.  This silicon = the vendor G3 model
	 * (aal-gen2/aal_hash.c *_HASH_ACT_DEFAULT_TO_CPU: "use CLS to decide flow
	 * miss CPU port" - the HS_DEF miss default has NO dpid).  Ours had it
	 * inverted (no disposition on the row, relying on a HS_DEF dpid) so a
	 * routed T2 MISS carried no CPU disposition and was DROPPED - the enable
	 * regression.  Keep stage2_ctrl_vld (0x04000000, the hit-path NAT) +
	 * t2_ctrl (w6); a T2 HIT overrides the disposition with the flow action. */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_WAN),
	  { 0, 0, 0, 0, 0x1C000000, 0x05000004, 0x00000A00 } },
	/* LAN: run T2 profile 1, same to-CPU disposition on the row */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_LAN),
	  { 0, 0, 0, 0, 0x1C000000, 0x05000004, 0x00001A00 } },
	/*
	 * WAN non-IP control trap action: the stock unicast->CPU_0 disposition
	 * bytes (golden FIB[4]: w4 0x1C000000 = dpid_vld|dpid_pri|permit @bits
	 * 154-156, w5 mcgid=0x10 CPU_0 @158-167 + stage3_ctrl_vld @184, w6
	 * 0x200) PLUS keep_orig_pkt_vld|keep_orig_pkt (w5 bits 22/23 = abs bits
	 * 182/183, cls_fib_mod_0/1_t msk_ctrl run - offsets anchored tier-1 by
	 * dpid @154-156, mcgid @162 and stage2_ctrl_vld @186 in the same run):
	 * deliver the ORIGINAL frame bytes, no PE re-encap - so the punted LCP
	 * keeps its real PPP proto.  No t2_ctrl: a dispositioned frame gets no
	 * T2 lookup (control frames need none).
	 */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_WAN_CTL),
	  { 0, 0, 0, 0, 0x1C000000, 0x01C00004, 0x00000200 } },
};

/* WAN MAC = LAN/base MAC + 1 (per-board rule, stock-verified). */
static void l3fe_wan_mac_derive(const u8 *lan_mac, u8 *wan_mac)
{
	u64 v = ((u64)lan_mac[0] << 40) | ((u64)lan_mac[1] << 32) |
		((u64)lan_mac[2] << 24) | ((u64)lan_mac[3] << 16) |
		((u64)lan_mac[4] << 8) | lan_mac[5];

	v++;
	wan_mac[0] = v >> 40;
	wan_mac[1] = v >> 32;
	wan_mac[2] = v >> 24;
	wan_mac[3] = v >> 16;
	wan_mac[4] = v >> 8;
	wan_mac[5] = v;
}

int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac)
{
	u8 wan_mac[6];
	int ret;

	if (!lan_mac)
		return -EINVAL;
	l3fe_wan_mac_derive(lan_mac, wan_mac);

	/* 1. Router MACs into the PP FIELD-CAM MAC-DA table: the PP then
	 * stamps HDR_I.mac_da_an_sel = idx+1 on every frame to that MAC. */
	ret = l3fe_mac_da_cam_set(ne, L3FE_AN_IDX_LAN, lan_mac);
	if (ret)
		return ret;
	ret = l3fe_mac_da_cam_set(ne, L3FE_AN_IDX_WAN, wan_mac);
	if (ret)
		return ret;

	/*
	 * Tier-1 stock-diff (2026-07-23, live devmem on stock NAND): stock's
	 * routed-ingress LPB profiles 0/1/2 carry mac_da_an_mask = 0, and CLS
	 * rows 3/67 (the pri-6 slots) are EMPTY.  Stock drives L3 HW-forward
	 * purely through the ALWAYS-ON golden CLS rows 0/1/2 + 64/65/66, whose
	 * FIB action already carries t2_ctrl + the CPU_0 miss disposition
	 * (cortina-ni-rx.c cls_fib_golden == stock FIB 0/4/8/256/260/264,
	 * byte-matched).  The earlier STG0 an-mask + pri-6 routed-rule scheme is
	 * NOT how stock works and corrupted the routed path on enable - removed.
	 * intf_add now only provisions the PP FIELD-CAM router-MAC entries
	 * (which stock also has: MAC-DA e0 = LAN gw, e1 = WAN = base+1).
	 */
	return 0;
}

/* One ARB LDPID->PDPID map entry (L2FE indirect, generic GO protocol). */
static int l3fe_pdpid_map_set(void __iomem *ne, u32 idx, u32 pdpid)
{
	writel(pdpid & 0xf, ne + L3FE_L2FE_PDPID_MAP_DATA);
	writel(L3FE_GO | L3FE_WRITE | idx, ne + L3FE_L2FE_PDPID_MAP_ACCESS);
	return l3fe_poll_clear(ne, L3FE_L2FE_PDPID_MAP_ACCESS, L3FE_GO);
}

/* APPEND one static L2FE FDB entry {mac} -> {ldpid, valid, static, DA/SA
 * permit}.  Key packing = the vendor __aal_mac_2_fdb_data split (proven by
 * the read-back HIT on the live board); vid/scind/dot1p = 0.  The FDB engine
 * INIT already ran in the RX bring-up (cortina_ni_rx_fdb_add_cpu), which
 * always precedes this probe-time call. */
static int l3fe_fdb_static_add(void __iomem *ne, const u8 *mac, u32 ldpid)
{
	u32 d3 = (mac[0] >> 5) & 0x7;
	u32 d2 = ((u32)(mac[0] & 0x1f) << 27) | ((u32)mac[1] << 19) |
		 ((u32)mac[2] << 11) | ((u32)mac[3] << 3) | ((mac[4] >> 5) & 0x7);
	u32 d1 = (u32)(((mac[4] & 0x1f) << 8) | mac[5]) << 19;
	u32 d0 = FIELD_PREP(L3FE_FDB_LPID, ldpid) | L3FE_FDB_VALID |
		 L3FE_FDB_STATIC | L3FE_FDB_DA_PERMIT | L3FE_FDB_SA_PERMIT;

	writel(0, ne + L3FE_FDB_CMD_RETURN);
	writel(d3, ne + L3FE_FDB_DATA3);
	writel(d2, ne + L3FE_FDB_DATA2);
	writel(d1, ne + L3FE_FDB_DATA1);
	writel(d0, ne + L3FE_FDB_DATA0);
	writel(L3FE_GO | L3FE_FDB_OP_APPEND, ne + L3FE_FDB_ACCESS);
	return l3fe_poll_clear(ne, L3FE_FDB_ACCESS, L3FE_GO);
}

int cortina_l3fe_hw_l3_forward_enable(void __iomem *ne, const u8 *router_mac)
{
	int i, ret;

	/* 1. Internal hash-miss action -> punt to CPU (never drop).  This is
	 * the SAFETY KEYSTONE: with zero flows installed every lookup misses,
	 * so this action is what carries a routed frame to the CPU/software
	 * path unchanged.  Program it BEFORE enabling the hash consult. */
	for (i = 0; i < L3FE_HS_DEF_REG_COUNT; i++)
		writel(l3fe_def_reg_stock[i], ne + L3FE_HS_DEF_REG0_ETY0 + i * 4);

	/* 2. CLS per-profile routing DEFAULT rows, stock bytes (fallthrough
	 * CPU disposition; effectively dead behind the all-wildcard trap
	 * rows).  The REAL T2 admission is the pri-6 mac_da_an_sel routed
	 * rules installed by cortina_l3fe_intf_add() below - stamping
	 * t2_ctrl on a row that also carries a full forwarding disposition
	 * (the catch-alls, these defaults) was PROVEN inert (HS_CACHE_CNT
	 * flat): a pre-dispositioned frame gets no T2 lookup.
	 * ★ CAVEAT (2026-07-24): the OBSERVATION stands, but that EXPLANATION
	 * is doubted.  Stock's own catch-all rows carry dpid_vld|dpid_pri|permit
	 * AND t2_ctrl_vld=1 at the same time, so a disposition evidently does
	 * NOT suppress T2 - dpid_pri looks like the arbitration bit, with
	 * t2_ctrl=0xF as the real bypass encoding.  The inertness was also
	 * measured with HS_CACHE_CNT, since established as a PHANTOM (a real
	 * main-hash hit leaves it flat on stock too), so that experiment proved
	 * less than it appeared to.  Do not build on "a dispositioned frame gets
	 * no T2 lookup" without re-measuring against the age-SRAM re-arm. */
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_cls_default); i++) {
		ret = l3fe_cls_fib_write(ne, l3fe_cls_default[i].idx,
					 l3fe_cls_default[i].w);
		if (ret)
			return ret;
	}

	/* 2b. Re-point the routed profiles' TUPLE0 maskptr at the 5-tuple-only
	 * mask (index 8) so both the SWO install-CRC and the HW lookup-CRC hash
	 * ONLY {l4_dp,l4_sp,ip_da/32,ip_sa/32,ip_protocol} - stock mask 0 also
	 * folds mac/lspid/dscp/vlan into the CRC (non-zero on a real frame, zero
	 * in the driver's sparse key) so a mask-0 install could never HIT.  Keep
	 * pri/type 0.  classify_setup already wrote mask 8's bits; this only runs
	 * under hw_l3_fwd, so gate-off leaves the profiles on the stock masks.
	 * ★ P4: profile 3 is the one the LIVE admission actually consults (the
	 * catch-all rows stamp t2_ctrl=3) - re-point it too; 0/1 kept for the
	 * dedicated per-interface rules once those key correctly. */
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_WAN));
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_LAN));
	writel(L3FE_5TUPLE_MASK_ID,
	       ne + L3FE_HS_PROFILE_TUPLE0(L3FE_MAIN_HASH_PROFILE_ROUTED));

	/* 2c. PON US egress plumbing for a T2 HIT that forwards WAN-ward
	 * (gemMapMode-1): PE gemid_map=1 + ldpid_base=0x20 so a hit-action's
	 * {mc=1, mcgid=gem, t2_ctrl1=tcont} egresses at hdr_a.ldpid 0x20+tcont
	 * = the SAME PON US ldpid the proven CPU data-TX path uses.  RMW: the
	 * reset default already carries ldpid_base=0x20; only gemid_map is
	 * added.  Zero-flow behaviour unchanged (the mode only interprets HIT
	 * actions carrying the gemMapMode encoding). */
	{
		u32 v = readl(ne + L3FE_PE_CFG);

		v &= ~(L3FE_PE_CFG_LDPID_BASE | L3FE_PE_CFG_MTU_CHK_EN);
		v |= FIELD_PREP(L3FE_PE_CFG_LDPID_BASE, L3FE_LDPID_PON_US_0) |
		     L3FE_PE_CFG_GEMID_MAP |
		     L3FE_PE_CFG_PAD_CTRL | L3FE_PE_CFG_RSVD14;
		writel(v, ne + L3FE_PE_CFG);	/* == stock 0x00105602 (tier-1 devmem) */
	}

	/* 2c-bis. PPPoE PE encap globals (one-time): header code/version/type
	 * + the v4/v6 PPP protocol numbers the PE stamps when a hit-action
	 * requests the PPPoE ADD encap.  Inert until an action carries
	 * pppoe_set/pppoe_vld - which only happens when a PPPoE WAN session is
	 * armed (cortina_ni_wan_pppoe_session_set); zero-flow and IPoE
	 * behaviour unchanged. */
	writel(L3FE_PE_PPPOE_CFG_VAL, ne + L3FE_PE_PPPOE_CFG);
	writel(L3FE_PE_PPPOE_PROT_CFG_VAL, ne + L3FE_PE_PPPOE_PROT_CFG);

	/* 2d. Slow HW ager ON (SECONDARY hit witness): with granularity 0 the
	 * ager block never runs and the age slot reads stale, so the age
	 * re-arm could never witness a hit (that broke the last cycle's
	 * witness).  The slow cadence keeps idle entries alive for far longer
	 * than any test window.  PRIMARY witness stays the forwarding change
	 * (CPU-forward counters flat while the far end still receives). */
	writel(L3FE_AGING_GRAN_SLOW, ne + L3FE_HS_AGING_GRANULARITY);

	/* 3. INGRESS ADMISSION, WAN leg: ARB LDPID->PDPID map [0x18] = 0x0a
	 * so a frame the PON PDC stamps LDPID 0x18 (L3_WAN) is physically
	 * handed to the L3FE WAN ingress (stock live [0x18]=0xA; our RX init
	 * leaves 0x18 at reset because nothing on the pre-offload datapath
	 * ever resolves to it).  All 4 {my_mac,dbuf} index combos, like the
	 * always-on map writes in cortina-ni-rx.c. */
	for (i = 0; i < 4; i++) {
		u32 idx = L3FE_LDPID_L3_WAN |
			  ((i & 1) ? L3FE_PDPID_IDX_DBUF : 0) |
			  ((i & 2) ? L3FE_PDPID_IDX_MYMAC : 0);

		ret = l3fe_pdpid_map_set(ne, idx, L3FE_PDPID_L3_WAN);
		if (ret)
			return ret;
	}

	/* 4. INGRESS ADMISSION, my-MAC recognition + the T2 admission rules:
	 * router-MAC CAM entries (0 = LAN gateway MAC, 1 = WAN MAC = base+1)
	 * + the STG0 LPB mac_da_an_mask bits + the dedicated pri-6 routed
	 * CLS rules whose action runs T2 - cortina_l3fe_intf_add(), stock's
	 * ca_l3_intf_add scheme.  Re-applied on every link-up by the
	 * cortina-ni-rx.c cls_init re-run (the my-MAC/STG0 re-init there
	 * rewrites the LPB HIGH words with the stock constants). */
	if (router_mac) {
		u8 wan_mac[6];

		ret = cortina_l3fe_intf_add(ne, router_mac);
		if (ret)
			return ret;
		l3fe_wan_mac_derive(router_mac, wan_mac);

		/* 5. ★ THE terminating DS-WAN delivery: static FDB entry
		 * {WAN MAC -> L3_WAN (0x18)}.  The Venus-family design keeps
		 * L2 MY-MAC detection OFF and "use[s] STATIC FDB to forward
		 * MyMAC packets to L3FE" — without this entry a PON DS unicast
		 * to the WAN MAC (the DHCP OFFER, every ping reply) is a DLF
		 * in the L2FE and gets FLOODED OUT instead of delivered
		 * (proven live 2026-07-19: 0/200 hades pings without it,
		 * 200/200 + DHCP lease .243 + WAN 0% loss with it).  This
		 * probe-time install covers the window before the gate flips
		 * (cn_l3e is set only after this function returns, so the
		 * RX bring-up's fdb_add_cpu skipped it); link-up re-arms
		 * re-install it via cortina_ni_rx_fdb_add_cpu, whose FDB
		 * engine INIT wipes and rebuilds the table.
		 * NOTE the LPB spcl_pkt_en (0x3410/0x3428 bit20) stays at the
		 * stock value 1: the L3 special-packet table behind it does
		 * not exist on this die (stock ca-ne.ko stubs
		 * aal_l3_specpkt_ctrl_set/get; writing its 0x3440/0x3444
		 * access regs SErrors) — the bit is inert, live-A/B-verified. */
		ret = l3fe_fdb_static_add(ne, wan_mac, L3FE_LDPID_L3_WAN);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Re-point ONE main-hash profile's TUPLE0 maskptr at the routed 5-tuple mask
 * (index 8), same write cortina_l3fe_hw_l3_forward_enable() step 2b does for
 * the profiles the US leg needs.
 *
 * WHY a per-profile entry point exists: an installed entry is found only if the
 * LOOKUP hashes the packet under the SAME mask the install used.  The install
 * CRC always uses mask 8, but the lookup uses the mask of whichever profile the
 * ingress classifier stamped into HDR_I.t2_ctrl - so a direction whose CLS row
 * stamps a profile still pointing at a stock mask can never HIT.  Step 2b
 * covers profiles 0/1/3 (the ones the US/LAN admission was proven to stamp);
 * the DS (WAN-ingress) leg calls this for the remaining profiles so its
 * stamped profile cannot be the one that was left out.
 *
 * Harmless for any profile that carries no flows: the main hash holds only our
 * own entries, a mask-8 lookup that finds none falls to the same CPU punt as
 * before, and a "false" match requires an identical 5-tuple - i.e. it IS that
 * flow.  Caller must be inside the hw_l3_fwd gate (this is a datapath-config
 * write); pri/type are kept 0 exactly as step 2b writes them.
 */
int cortina_l3fe_hash_profile_mask_repoint(void __iomem *ne, u32 profile)
{
	if (profile > L3FE_MAIN_HASH_PROFILE_MAX)
		return -EINVAL;
	writel(L3FE_5TUPLE_MASK_ID, ne + L3FE_HS_PROFILE_TUPLE0(profile));
	return 0;
}

/*
 * ★ THE ONE egress L3-IF word, built the way stock builds it.
 *
 * On this die the whole entry is 24 bits wide and holds nothing but the egress
 * SMAC selector and the PPPoE header control - there is no second table an
 * interface could keep its SMAC in.  The reference model therefore builds ONE
 * entry per egress interface: the per-interface pass sets {mac_sa_vld=1,
 * mac_sa_an_sel=cam_idx+1} for every non-loopback interface, and a PPPoE tunnel
 * on top of that interface only ADDS {pppoe_session_id, pppoe_vld=1,
 * pppoe_set=1} to the SAME word.
 *
 * Corroborated tier-1 by the live stock WAN entry, 0x00940001, which decodes as
 * {pppoe_set=1, pppoe_vld=0 (inert), session=0, mac_sa_vld=1, an_sel=2 (the WAN
 * MAC, my-MAC CAM idx 1), pad_ctrl=1} - i.e. stock's idle WAN entry is already
 * pre-shaped for the PPPoE overlay: bringing a session up only sets pppoe_vld
 * and fills the session id.  So:
 *
 *   @session == 0  -> 0x00940001 for an_sel 2: substitute the egress SMAC,
 *                     PPPoE machinery present but INERT (vld=0 = no ADD).  This
 *                     is byte-identical to what the IPoE path has always
 *                     written, so the board-proven US/DS IPoE actions are
 *                     unchanged by this builder.
 *   @session != 0  -> the same word plus {pppoe_vld=1, session}: the PE ADDs the
 *                     8-byte 0x8864 header AND substitutes the WAN SMAC.
 *
 * ★ WHY the SMAC matters (the defect this builder fixes): a PPPoE session is
 * bound to {session_id, peer MAC} negotiated at PADI/PADS, and the peer MAC is
 * the ONU's WAN MAC because pppd runs on the WAN netdev.  An entry that only
 * ADDs the header leaves the ORIGINAL source MAC in place - on a routed transit
 * frame that is the LAN client's MAC - so the access concentrator sees a session
 * frame from an unknown MAC and drops or mis-accounts it.
 *
 * bit23 (PAD_CTRL / pppoe_len_control): stock's live WAN entry has it set, and
 * our IPoE entries have carried it through the board-proven 941/956 Mbps runs,
 * so it is set here too - one entry shape for the WAN, exactly the stock shape.
 * Its consumer is not proven on this die (the header names it PAD_CTRL, the HAL
 * names it pppoe_len_control, and the PE computes the PPPoE payload length per
 * packet); the on-wire PPPoE length field is what settles it, so an on-wire
 * length that disagrees with inner-IP-total-length + 2 is the signal to A/B it.
 */
static u32 l3fe_l3if_entry(u8 an_sel, u16 session)
{
	u32 entry = L3FE_L3IF_MAC_SA_VLD | L3FE_L3IF_PAD_CTRL |
		    L3FE_L3IF_PPPOE_SET |
		    FIELD_PREP(L3FE_L3IF_MAC_SA_AN_SEL, an_sel);

	if (session)
		entry |= L3FE_L3IF_PPPOE_VLD |
			 FIELD_PREP(L3FE_L3IF_PPPOE_SESSION, session);
	return entry;
}

static int l3fe_l3if_write(void __iomem *ne, u32 idx, u32 entry)
{
	if (idx >= L3FE_L3IF_ENTRIES)
		return -EINVAL;
	writel(entry, ne + L3FE_L3IF_DATA);
	writel(L3FE_GO | L3FE_WRITE | (idx & (L3FE_L3IF_ENTRIES - 1)),
	       ne + L3FE_L3IF_ACCESS);
	return l3fe_poll_clear(ne, L3FE_L3IF_ACCESS, L3FE_GO);
}

/*
 * Program egress L3-IF entry @idx for a PPPoE WAN: substitute the egress SMAC
 * named by @an_sel (the WAN MAC) AND ADD the 8-byte 0x8864 session header for
 * @session.  A US hit-action that sets GROUP_20 {pppoe_set1, pppoe_vld1,
 * l3_if_vld1, egr_l3_if_idx1=@idx} then gets both rewrites on egress (the PE
 * globals 0x3500/0x3504 supply code/ver/type + the PPP protocol number).
 *
 * @session == 0 CLEARS the session: the entry reverts to the plain SMAC-only
 * shape (identical to the IPoE entry), so a frame that still referenced this
 * index for one packet during a teardown race leaves as a correct IPoE frame
 * rather than one carrying a stale header or the wrong source MAC.
 *
 * Called only under the hw_l3_fwd gate; an unreferenced entry is inert.
 */
int cortina_l3fe_pppoe_l3if_set(void __iomem *ne, u32 idx, u16 session,
				u8 an_sel)
{
	return l3fe_l3if_write(ne, idx, l3fe_l3if_entry(an_sel, session));
}

/* ---- L3FE HW flow-offload next-hop L2 rewrite (defect A2) -------------- *
 * A routed/NAT flow's hit-action rewrites the next-hop DMAC (mac_da_idx into
 * the HS_LIGHT indexed MAC table) + the egress SMAC (egr_l3_if_idx into the
 * L3-IF table, which selects our WAN MAC from the my-MAC CAM).  Without these
 * an offloaded flow egresses with the ONU's own DMAC (hairpin) - which is why
 * hw_l3_fwd is gated OFF.  Both are plain indirect-access register SRAM; recipe
 * RE'd from cortina-api/flow.c + aal-gen2/aal_hashlite.c + aal-77c/aal_l3_if.c
 * (the same GO/poll protocol as the mask/CLS/CAM tables). */

#define L3FE_MACDA_IDX_ENTRIES		1024

/*
 * ★ CRASH FIX (async SError on the first AUTO flow install) + aal-77c reality.
 *
 * The "HS_LIGHT indexed-interface MAC table" this used to write (ACCESS 0x3dc4,
 * DATA5/DATA4 0x3dc8/0x3dcc, tblsel 8<<12) was RE'd from aal-gen2/aal_hashlite.c
 * - the WRONG chip tree (same class as the FIB layout bug).  On this aal-77c die
 * (rtl8277c) that block DOES NOT EXIST:
 *   - the L3FE register window ends at ~0x3c8c (authoritative rtl8277c_registers.h);
 *     0x3dc4/0x3dc8/0x3dcc are UNMAPPED holes -> a write async-SErrors the CPU
 *     (the t=58.5s panic on the first nf_flow_table install);
 *   - rtl8277c has NO HS_LIGHT register at all (0x3d00-0x3exx undefined).  Even
 *     the *correct* aal-gen2 offset (ca8271 HS_LIGHT_IND_X_ACCESS = 0x3e64) is
 *     absent here.
 *
 * The next-hop DMAC on aal-77c comes from a DIFFERENT table, not an HS_LIGHT
 * indexed MAC: the egress L3-IF entry carries only the SMAC (EGRESS_L3_IF_TBL_DATA
 * = PPPoE + MAC_SA_VLD/AN_SEL only, no DMAC field), and the FIB mac_da_idx indexes
 * the L2 FDB / LUT (fc_mgr programs it via rtk_fc_l2_addr_add -> _rtk_fc_lut_learning).
 * Porting that next-hop path is a follow-up; until then REFUSE, so cn_flow_replace
 * keeps the routed flow on the SW fast path (which forwards+delivers correctly) -
 * no crash, no regression, only no HW offload of that flow yet.
 */
int cortina_l3fe_macda_idx_set(void __iomem *ne, u32 idx, const u8 *mac)
{
	(void)ne; (void)idx; (void)mac;
	return -EOPNOTSUPP;
}

/* IPoE egress L3-IF entry: substitute the egress SMAC from the my-MAC CAM entry
 * named by mac_sa_an_sel (WAN MAC = CAM idx 1 -> an_sel 2), PPPoE inert.  Same
 * builder as the PPPoE entry with session 0 - stock's idle WAN entry shape,
 * 0x00940001 for an_sel 2 (tier-1 live read), byte-identical to what this
 * function has written on every board-proven IPoE run. */
int cortina_l3fe_ipoe_l3if_set(void __iomem *ne, u32 idx, u8 an_sel)
{
	return l3fe_l3if_write(ne, idx, l3fe_l3if_entry(an_sel, 0));
}
