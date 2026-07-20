/* SPDX-License-Identifier: GPL-2.0 */
/*
 * cortina-l3fe.h - RTL9607F "Elnath" NE L3FE main-hash flow-engine bring-up
 * interface, shared between the init chain (cortina-l3fe.c) and the
 * nf_flow_table offload backend (cortina-ni-flowoffload.c).
 */

#ifndef _CORTINA_L3FE_H
#define _CORTINA_L3FE_H

#include <linux/io.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>

/*
 * DDR tables the engine DMA-reads (one contiguous dma_alloc_coherent carve;
 * the NE fabric is NON-coherent, so a cached carve = stale matches).
 * Live-stock geometry (devmem capture 2026-07-18, this board):
 *   key table  64K x 4 B  = 256 KB   @ carve + 0
 *   action FIB 64K x 32 B = 2 MB     @ carve + 256 KB   (stock: FIB base =
 *                                     key base + 0x40000, same layout)
 * Stock arms ONLY these two DDR bases: BA_OA0/BA_DA0/BA_CA0 stay 0 (overflow
 * CAM unused by the 07f add path; default actions live in the internal
 * DEFAULT_ACTION registers - HASH_INI.def_reg=1; action cache is on-chip
 * SRAM).
 */
#define CN_L3E_KEY_TBL_BYTES	0x40000		/* 64K x u32 CRC32 */
#define CN_L3E_FIB_TBL_BYTES	0x200000	/* 64K x 32 B actions */
#define CN_L3E_CARVE_BYTES	(CN_L3E_KEY_TBL_BYTES + CN_L3E_FIB_TBL_BYTES)

struct cn_l3e_tables {
	void		*key_virt;
	dma_addr_t	key_pa;
	void		*fib_virt;
	dma_addr_t	fib_pa;
};

/*
 * One-time engine arm (ordered init chain, HW_FLOW_OFFLOAD_DESIGN.md section
 * 2): HS_MEM_INI self-zero -> SW-zero the carve -> base regs -> geometry ->
 * anti-wedge HW patch (RSV0/RSV1) -> AXIM2 depth -> punt defaults ->
 * granularity 0 (HW auto-age OFF).  @ne = the NI register window
 * (phys 0xf4300000); HS registers are NE-relative 0x37xx-0x3cxx.
 * Probe-time only (single-threaded, no lock).  Returns 0 or -ETIMEDOUT.
 */
int cortina_l3fe_engine_init(void __iomem *ne, const struct cn_l3e_tables *t);

/*
 * Program the stock profile/tuple + mask-table classify config (tier-1
 * captured, dev/x411axf/stock_l3fe_dump_full.txt) so the main-hash engine
 * parses/keys a routed packet like stock.  ★ Necessary but NOT sufficient for
 * a HW hit: our datapath software-forwards LAN->WAN so the L3FE lookup is not
 * yet consulted (see the comment on the definition).  Returns 0 or -ETIMEDOUT.
 */
int cortina_l3fe_classify_setup(void __iomem *ne);

/*
 * Mask-table entry write (64 x 256-bit, two 128-bit beats, indirect GO
 * protocol).  @lo = bits 127:0 as 4 words MASK0..MASK3 order, @hi = bits
 * 255:128.  Probe/config-time only in phase 1 (caller serializes).
 */
int cortina_l3fe_mask_write(void __iomem *ne, u32 idx,
			    const u32 lo[4], const u32 hi[4]);

/*
 * ★ Divergence B - enable HW L3-forwarding into the main hash with a
 * hash-MISS trap-to-CPU (never drop).  Programs, all tier-1 stock-mirrored:
 *   - the internal hash-miss action FIB (HS_DEF_REGn, def_reg=1 mode) so a
 *     lookup miss punts to CPU instead of the reset-null (=drop) action;
 *   - the CLS per-profile routing DEFAULT actions (FIB idx 1024/1025 prof0
 *     WAN, 1028 prof1 LAN) whose t2_ctrl field points HDR_I at the T2 main
 *     hash (0=WAN profile, 1=LAN profile) - this IS the hash-consult enable
 *     (there is no separate per-port register: t2_ctrl is a HDR_I field the
 *     CLS result carries; ca-ne.ko convert_intf_to_cls / cls_type_1_default).
 * Also programs the transit-frame INGRESS ADMISSION delta (why l3fe_rx was 0:
 * routed frames never physically entered the L3FE):
 *   - ARB LDPID->PDPID map [0x18 L3_WAN] = 0x0a (the L3FE WAN ingress port;
 *     stock live 0xA) so PON PDC frames stamped LDPID 0x18 reach the engine;
 *   - the PP FIELD-CAM MAC-DA entries 0/1 = @router_mac / @router_mac+1 (the
 *     WAN MAC), with the proper 0x3200 ACCESS commit, so the PP recognises
 *     routed frames (mac_da_an_sel) and STG0 rewrites their lspid to
 *     L3_WAN/L3_LAN.  @router_mac may be NULL (CAM step skipped).
 * The LAN leg (FDB {router MAC} -> LDPID 0x19 -> PDPID 0x0d) is already
 * installed by the always-on RX init (cortina-ni-rx.c fdb_add_cpu +
 * pdpid_l3lan=0x0d).
 * Gated OFF by default in the caller (module param): this is the first
 * datapath-touching step, validated by a zero-flow no-regression boot before
 * any flow is installed.  @ne = the NI register window.  Returns 0 or -errno.
 */
int cortina_l3fe_hw_l3_forward_enable(void __iomem *ne, const u8 *router_mac);

/*
 * ★ The per-L3-interface T2 ADMISSION (stock ca_l3_intf_add scheme), all
 * three coordinated pieces for the LAN gateway MAC (@lan_mac, CAM idx 0) and
 * the WAN MAC (base+1, CAM idx 1):
 *   1. router MACs -> the PP FIELD-CAM MAC-DA table, so the PP stamps
 *      HDR_I.mac_da_an_sel = idx+1 (nonzero) on frames to a router MAC;
 *   2. STG0 LPB mac_da_an_mask bits (promiscuous compare-and-stamp) for the
 *      routed ingress profiles;
 *   3. the dedicated pri-6 routed CLS rules keyed {mac_da_an_sel EXACT,
 *      lspid} whose action is ONLY "run T2" (t2_ctrl=profile + NAT stage2,
 *      no forwarding disposition) - T2 HIT = HW forward+NAT, MISS = the
 *      HS_DEF CPU_0 punt (terminating/unoffloaded traffic keeps working).
 * Called at probe from cortina_l3fe_hw_l3_forward_enable() and RE-APPLIED on
 * every link-up (cortina-ni-rx.c cls_init) because the my-MAC/STG0 re-init
 * there rewrites the LPB HIGH words.  Only under hw_l3_fwd; gate-off
 * byte-identical.  Returns 0 or -errno.
 */
int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac);

/*
 * HS_SWO on-chip CRC engine: feed @nwords key words (HDR_I layout, word 0
 * first) + @mask_id, run the engine, return the hardware's {crc32, crc16}.
 * This is the init-time selftest oracle proving the SW CRC convention
 * matches what the lookup hardware computes.  Returns 0 or -ETIMEDOUT.
 */
int cortina_l3fe_swo_crc(void __iomem *ne, const u32 *words, int nwords,
			 u32 mask_id, u32 *crc32_out, u16 *crc16_out);

/*
 * PPPoE WAN egress encap: program (or clear, @session == 0) egress L3-IF
 * table entry @idx as a pure PPPoE ADD-header entry {pppoe_set, pppoe_vld,
 * pppoe_session_id=@session} (SMAC untouched, HW auto-length).  A US
 * hit-action selects it via GROUP_20 l3_if_vld1/egr_l3_if_idx1 to HW-insert
 * the 8-byte 0x8864 session header.  hw_l3_fwd-gated caller only; caller
 * serializes (reg_lock).  Returns 0 or -EINVAL/-ETIMEDOUT.
 */
int cortina_l3fe_pppoe_l3if_set(void __iomem *ne, u32 idx, u16 session);

#endif /* _CORTINA_L3FE_H */
