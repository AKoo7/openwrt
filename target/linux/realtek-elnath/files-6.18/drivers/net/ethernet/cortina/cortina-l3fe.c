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
 * Facts + citations: dev/x411axf/HW_FLOW_OFFLOAD_L3FE_INIT.md and
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
#define L3FE_LDPID_PON_US_0		0x20	/* AAL_LPORT_PON_US_0 */

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
 *     dev/x411axf/stock_golden_qm.txt).                                *
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

int cortina_l3fe_engine_init(void __iomem *ne, const struct cn_l3e_tables *t)
{
	int ret, i;

	/*
	 * 1. Engine SRAM/table self-init - MUST precede the base/size arm.
	 *    Kick req_sts (bit0), poll its self-clear.
	 */
	writel(1, ne + L3FE_HS_MEM_INI);
	ret = l3fe_poll_clear(ne, L3FE_HS_MEM_INI, BIT(0));
	if (ret)
		return ret;

	/* 2. SW-zero the DDR tables (belt and braces, matches vendor). */
	memset(t->key_virt, 0, CN_L3E_KEY_TBL_BYTES);
	memset(t->fib_virt, 0, CN_L3E_FIB_TBL_BYTES);
	wmb();	/* coherent carve: make the zeroing visible before the arm */

	/* 3. DDR base registers (physical, 128-byte aligned, [31:7] in
	 * place; hi regs hold phys[39:32]).  Overflow/default/cache bases
	 * stay 0 as on stock. */
	writel(lower_32_bits(t->key_pa), ne + L3FE_HS_BA_MH0);
	writel(upper_32_bits(t->key_pa) & 0xff, ne + L3FE_HS_BA_MH1);
	writel(lower_32_bits(t->fib_pa), ne + L3FE_HS_BA_MA0);
	writel(upper_32_bits(t->fib_pa) & 0xff, ne + L3FE_HS_BA_MA1);

	/* 4. Geometry + cache config (stock-verbatim). */
	writel(L3FE_HASH_INI_VAL, ne + L3FE_HS_HASH_INI);
	writel(L3FE_CACHE_INI_VAL, ne + L3FE_HS_CACHE_INI);
	writel(L3FE_CACHE_MISC_VAL, ne + L3FE_HS_CACHE_MISC);

	/* 5. Anti-wedge HW patch (do NOT skip: the engine can stall under
	 * DDR read load without it) + AXI outstanding depth. */
	writel(readl(ne + L3FE_HS_RSV0) | L3FE_RSV0_PATCH, ne + L3FE_HS_RSV0);
	writel(readl(ne + L3FE_HS_RSV1) | L3FE_RSV1_PATCH, ne + L3FE_HS_RSV1);
	writel(L3FE_AXIM2_CONFIG_VAL, ne + L3FE_AXIM2_CONFIG);

	/* 6. Miss/fail never drops: double-check-fail punt + the internal
	 * default (miss) actions, def_reg=1 mode - stock programs words
	 * 0..3, the rest stay 0. */
	writel(L3FE_CHK_FAIL_CTRL_VAL, ne + L3FE_HS_CHK_FAIL_CTRL);
	for (i = 0; i < 4; i++)
		writel(L3FE_DEFAULT_ACTION_VAL, ne + L3FE_HS_DEFAULT_ACTION(i));

	/* 7. HW auto-age-countdown OFF (stock): hardware must never age a
	 * flow out from under the Linux flowtable.  Liveness = HW hit-rearm
	 * + the SW sweep; lifetime = nf gc + FLOW_CLS_DESTROY. */
	writel(0, ne + L3FE_HS_AGING_GRANULARITY);

	/* 8. AQM flow-stat timer, stock-verbatim. */
	writel(L3FE_AQM_TIMER_VAL, ne + L3FE_AQM_TIMER);

	return 0;
}

/*
 * Profile/tuple + mask-table classify config, tier-1 captured live from the
 * stock-armed engine (RTK_GW 5.10.226, 2026-07-18; the exact register dump is
 * in dev/x411axf/stock_l3fe_dump_full.txt).  Programming this makes the
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
 * {l4_dp, l4_sp, ip_da/32, ip_sa/32, ip_protocol} (mask bit 1 = EXCLUDE);
 * the exact words were verified on-board (swolearn): bits {74 dport, 90 sport,
 * 233-264 DA, 361-392 SA, 492-499 proto} MOVE the CRC, and {505 ip_vld,
 * 116 dscp, 600/700/726 mac/lspid} do NOT.  So a sparse 5-tuple key hashes
 * identically to a parsed packet under mask 8.  The routed profiles' TUPLE
 * maskptr is re-pointed at mask 8 by cortina_l3fe_hw_l3_forward_enable()
 * (gated); gate-off leaves the profiles on the stock masks, so programming
 * this spare index changes no datapath behaviour.
 */
#define L3FE_MASK_5TUPLE	8	/* 5-tuple-only NAPT mask index */
static const u32 l3fe_mask_lo[9][4] = {
	{ 0x000003ff, 0x0221f000, 0x15001402, 0xc0f03fe1 },
	{ 0xffffffff, 0x027fffff, 0x1f403000, 0xc0f01fe1 },
	{ 0xffffffff, 0x027fffff, 0xff3ff002, 0xffffffff },
	{ 0xffffffff, 0x027fffff, 0x1f003402, 0xffffffe1 },
	{ 0xffffffff, 0x027fffff, 0x1f003402, 0xffffffe1 },
	{ 0xffffffff, 0x027fffff, 0xfffff000, 0xc0ffffff },
	{ 0xffffffff, 0x027fffff, 0xff7ff000, 0xffffffff },
	{ 0x000003ff, 0x0221f000, 0x15001402, 0xc0f03fe1 },
	/* mask 8: 5-tuple only (mask0 | ~mask1, swolearn-verified) */
	{ 0x000003ff, 0xffa1f000, 0xf5bfdfff, 0xffffffff },
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
	0x1C000000, 0x00000004, 0x00000000,	/* entry 0: TRAP -> CPU_0 (mcgid 0x10) */
	/* ★ entry 1 = TRAP -> CPU_0 too (deviation from the stock capture
	 * 0x00009811): profile 3's INI (0x3784 = 0x00084211) points all four
	 * default_sel nibbles at DEFAULT_ACTION entry 1, and the gated routed
	 * admission now stamps t2_ctrl=3 on the LAN catch-all - so a profile-3
	 * T2 MISS (= every ONU-terminating/unoffloaded LAN frame, incl. SSH)
	 * resolves through entry 1.  Stock's 0x9811 is its bridge-flow miss
	 * action; ours MUST punt to CPU_0 or gate-on would black-hole LAN
	 * management on the first miss. */
	0x1C000000, 0x00000004, 0x00000000,	/* entry 1: TRAP -> CPU_0 (see above) */
	0x00040001, 0x00300000, 0x00404000,	/* entry 2 */
	0x00009831, 0x00300000, 0x00000000,	/* entry 3 */
};

/*
 * CLS per-profile routing DEFAULT actions - the hash-CONSULT enable.  A routed
 * frame that matches no specific CLS rule falls through to the profile default
 * at idx (max_entry-16)|(profile<<2)|((rslt_type&1)<<1): 1024/1025 = profile 0
 * (WAN ingress), 1028 = profile 1 (LAN ingress).  Per the ca-ne.ko RE
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
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; } l3fe_cls_default[] = {
	{ 1024, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* prof0 WAN (stock) */
	{ 1025, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },	/* prof0 WAN (stock pair) */
	{ 1028, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },	/* prof1 LAN (stock) */
};

/* One CLS-FIB indirect write: words then ACCESS=GO|WR|idx, poll GO clear. */
static int l3fe_cls_fib_write(void __iomem *ne, u16 idx, const u32 w[L3FE_CLS_FIB_WORDS])
{
	int i;

	for (i = 0; i < L3FE_CLS_FIB_WORDS; i++)
		writel(w[i], ne + L3FE_CLS_FIB_DATA0 - i * 4);
	writel(L3FE_GO | L3FE_WRITE | (idx & 0x7ff), ne + L3FE_CLS_FIB_ACCESS);
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
static int l3fe_cls_key_write(void __iomem *ne, u16 idx,
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
static const struct { u16 idx; u32 w[L3FE_CLS_KEY_WORDS]; } l3fe_cls_routed_key[] = {
	/* WAN ingress: an_sel=2 (WAN MAC, CAM idx 1), lspid=0x18, pri 6 */
	{ L3FE_CLS_KEY_ROW_WAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00030FE4, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
	/* LAN ingress: an_sel=1 (LAN gateway MAC, CAM idx 0), lspid=0x19, pri 6 */
	{ L3FE_CLS_KEY_ROW_LAN, { 0xFFFFFFFF, 0xFFFFFFFF, 0x00032FE2, 0, 0,
				  0, 0, 0, 0, 0, 0x08180000 } },
};
static const struct { u16 idx; u32 w[L3FE_CLS_FIB_WORDS]; } l3fe_cls_routed_fib[] = {
	/* WAN: run T2 profile 0, stage2(NAT)=UPDATE+vld, no fwd disposition */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_WAN),
	  { 0, 0, 0, 0, 0, 0x04000000, 0x00000A00 } },
	/* LAN: run T2 profile 1 */
	{ L3FE_CLS_FIB_IDX(L3FE_CLS_KEY_ROW_LAN),
	  { 0, 0, 0, 0, 0, 0x04000000, 0x00001A00 } },
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
	int i, ret;

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

	/* 2. Latch the compare in the ingress STG0 LPB profiles:
	 * mac_da_an_mask |= bit(idx+1), match_en left 0 (promiscuous).
	 * RMW is safe against the link-up re-init: cortina_ni_rx_mymac_trap
	 * rewrites the HIGH words with the stock constants first, then the
	 * cls_init re-run re-applies these bits. */
	for (i = 0; i < L3FE_LPB_AN_PROFILES; i++) {
		u32 v = readl(ne + L3FE_STG0_LPB_HIGH(i));

		v |= L3FE_LPB_AN_MASK(L3FE_AN_SEL(L3FE_AN_IDX_LAN)) |
		     L3FE_LPB_AN_MASK(L3FE_AN_SEL(L3FE_AN_IDX_WAN));
		writel(v, ne + L3FE_STG0_LPB_HIGH(i));
	}

	/* 3. The pri-6 routed CLS rules: ALL FIBs first, then ALL KEYs
	 * (stock aal_l3_cls_add order - a key must never point at a stale
	 * action). */
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_cls_routed_fib); i++) {
		ret = l3fe_cls_fib_write(ne, l3fe_cls_routed_fib[i].idx,
					 l3fe_cls_routed_fib[i].w);
		if (ret)
			return ret;
	}
	for (i = 0; i < (int)ARRAY_SIZE(l3fe_cls_routed_key); i++) {
		ret = l3fe_cls_key_write(ne, l3fe_cls_routed_key[i].idx,
					 l3fe_cls_routed_key[i].w);
		if (ret)
			return ret;
	}
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
	 * flat): a pre-dispositioned frame gets no T2 lookup. */
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

		v &= ~L3FE_PE_CFG_LDPID_BASE;
		v |= FIELD_PREP(L3FE_PE_CFG_LDPID_BASE, L3FE_LDPID_PON_US_0) |
		     L3FE_PE_CFG_GEMID_MAP;
		writel(v, ne + L3FE_PE_CFG);
	}

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
