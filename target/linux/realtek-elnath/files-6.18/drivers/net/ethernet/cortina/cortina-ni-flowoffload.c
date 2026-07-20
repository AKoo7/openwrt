// SPDX-License-Identifier: GPL-2.0-only
/*
 * cortina-ni-flowoffload.c - nf_flow_table HW offload glue for the Cortina
 * NE L3FE "main hash" flow engine (RTL9607F / CA8277C "Elnath").
 *
 * The flow_block / rhashtable / rule-parse layer follows the mainline model
 * established by drivers/net/ethernet/mediatek/mtk_ppe_offload.c; the
 * cn_l3e_* backend implements the engine's programming protocol (register
 * facts recovered from the stock firmware's ca-ne.ko by disassembly/
 * decompilation, verified against the chip register map AND against the
 * live stock-armed engine - devmem capture 2026-07-18).
 *
 * Phase 1 (this build) arms and verifies the engine only: the carve, the
 * ordered init chain (cortina-l3fe.c), the SWO HW-CRC selftest and the
 * ndo_setup_tc hook are live, but no classify profile feeds the hash yet,
 * so no flow is ever offloaded and every request is refused to the normal
 * software path.  Phase 2 adds the first manual flow; phase 3 the full
 * NAPT rule parse.
 *
 * Engine model (differs from mtk PPE in one fundamental way: the hash is
 * SOFTWARE-computed and the entry is SOFTWARE-placed; hardware only looks
 * up):
 *   - hash-key table:   64K x u32 in DDR, entry = CRC32 of the masked
 *                       flow key; bucket = crc16 & ~7 (8-way, stock live)
 *   - action FIB:       64K x 32 B in DDR (48 B in NAPTv6 mode)
 *   - age table:        in-engine, 2 bit/entry via indirect access;
 *                       0 = free, 1..2 = aging (HW re-arms on hit),
 *                       3 = static.  Writing a non-zero age = go-live.
 *   - action cache:     2048-entry on-chip; must be explicitly invalidated
 *                       on delete/update or a stale action keeps matching.
 *   - profiles 0..6:    tuple/mask select, partitioned by ingress CLE
 *                       profile (WAN = 0, LAN = 1); profile miss -> default
 *                       (punt-to-CPU) action.
 *
 * Full sequence + bit-layout documentation:
 *   dev/x411axf/HW_FLOW_OFFLOAD_FLOWBLOCK_MAP.md
 * Synthesized design (init chain, aging sync, >10k-flow scale plan):
 *   dev/x411axf/HW_FLOW_OFFLOAD_DESIGN.md
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/rhashtable.h>
#include <net/flow_offload.h>
#include <net/pkt_cls.h>
#include <net/netfilter/nf_flow_table.h>

#include "cortina-ni.h"
#include "cortina-l3fe.h"

/* ------------------------------------------------------------------ */
/* L3FE main-hash ("HS") engine registers, offsets from the NE iobase  */
/* (NE block phys 0xf4300000; register names match the chip register   */
/* map so anyone can cross-reference).                                  */
/* ------------------------------------------------------------------ */

#define CN_L3E_HS_PROFILE_INI(p)	(0x3700 + (p) * 0x2c)	/* tpl_num[3:0], default_sel_0e[8:4], 0a[13:9], 1e[18:14], 1a[23:19] */
#define CN_L3E_HS_PROFILE_TUPLE(p, t)	(0x3704 + (p) * 0x2c + (t) * 4) /* maskptr[5:0], pri[10:8], type[12] */
#define CN_L3E_HS_PROFILE_T0_ACTION(p)	(0x3724 + (p) * 0x2c)	/* a_mask[24:0], fetch_sz[27:25] */
#define CN_L3E_HS_HASH_INI		0x3834	/* hb_size[1:0], ht_size[4:2], ha_width[7:5], def_reg[16], crc_ntfy_en[17] */
#define CN_L3E_HS_BA_MH0		0x383c	/* hash-key table base, phys bits[31:7] in place */
#define CN_L3E_HS_BA_MA0		0x3844	/* main action FIB base */
#define CN_L3E_HS_OVERFLOW_INI		0x3848	/* oa_width[2:0] */
#define CN_L3E_HS_BA_OA0		0x3850	/* overflow FIB base */
#define CN_L3E_HS_DEFAULT_INI		0x3854	/* da_width[2:0] */
#define CN_L3E_HS_BA_DA0		0x385c	/* default FIB base */
#define CN_L3E_HS_DEFAULT_ACTION(i)	(0x3860 + (i) * 4) /* fib_addr[24:0] | da_width<<25 */
#define CN_L3E_HS_CACHE_INI		0x38a0
#define CN_L3E_HS_BA_CA0		0x38a8	/* cache FIB base */
#define CN_L3E_HS_CACHE_CTRL		0x38ac	/* slot[4:0], crc16[20:5], loc[24], age[27:26], pri[29:28], cmd[31:30] */
#define CN_L3E_HS_CACHE_CTRL_REQ	0x38b0	/* bit0 = GO / busy */
#define CN_L3E_HS_CACHE_CTRL_STS	0x38b4	/* bit1 err_hash, bit2 err_free, bit3 err_nch(benign), bit6 evicted */
#define CN_L3E_HS_CACHE_AGE10		0x38b8	/* 16-bit cache aging units, ages 0/1 */
#define CN_L3E_HS_CACHE_AGE32		0x38bc	/* ages 2/3 */
/* action-cache utilisation count (ut_cnt[11:0]); climbs as the on-chip action
 * cache fills on HW hits.  07f offset = the ca8277b HS_CACHE_CNT (0x3900) minus
 * the live-verified 0x40 cache-block shift on this die = 0x38c0. */
#define CN_L3E_HS_CACHE_CNT		0x38c0
#define CN_L3E_HS_SWO_IDX		0x38d8	/* HW-CRC selftest engine (debug) */
#define CN_L3E_HS_SWO_DAT		0x38dc
#define CN_L3E_HS_SWO_CTRL		0x38e0	/* bit0 = GO / busy */
#define CN_L3E_HS_OVERFLOW_ACCESS	0x3904	/* 64-entry overflow key CAM (unused in phase 1) */
#define CN_L3E_HS_MASK_ACCESS		0x3910	/* mask table: idx | bit30 wr | bit31 GO | bit6 upper-128 beat */
#define CN_L3E_HS_MASK_DATA(n)		(0x3920 - (n) * 4) /* MASK0..3 = 0x3920,191c,1918,1914 */
#define CN_L3E_HS_AGING_GRANULARITY	0x3924	/* 30-bit; = age_time_s * core_clk / 0x2000 */
#define CN_L3E_HS_AGE_ACCESS		0x3928	/* bucket[10:0] | (1<<11 = overflow age) | bit30 wr | bit31 GO */
#define CN_L3E_HS_AGE_DATA_HI		0x3934	/* slots 16..31, 2 bit each */
#define CN_L3E_HS_AGE_DATA_LO		0x3938	/* slots  0..15, 2 bit each */
#define CN_L3E_HS_MEM_INI		0x393c	/* bit0 req_sts: engine SRAM self-init */
#define CN_L3E_HS_PF_KEY(p)		(0x394c + (p) * 0x14) /* sel[5:0]=0 CRC16, crc32_sel[7:6] */
#define CN_L3E_HS_PF_TPL_SP(p)		(0x3950 + (p) * 0x14)
#define CN_L3E_HS_PF_TPL_DP(p)		(0x3954 + (p) * 0x14)
#define CN_L3E_HS_PF_TPL_SIP(p)		(0x3958 + (p) * 0x14)
#define CN_L3E_HS_PF_TPL_DIP(p)		(0x395c + (p) * 0x14)

#define CN_L3E_GO			BIT(31)	/* indirect-access request/busy */
#define CN_L3E_WRITE			BIT(30)	/* indirect-access direction */
#define CN_L3E_POLL_TRIES		1000

/* geometry (live-stock HASH_INI = 0x0003007D, devmem-captured 2026-07-18):
 * 64K entries (ht_size = 7) in 8-WAY hash buckets (hb_size = 1 - NOT the
 * 32-way the static RE first suggested; tier-1 wins), 32-byte FIB entries
 * (ha_width = 3, normal mode).  entry idx = (crc16 & ~7) + way.
 * The AGE SRAM has its own FIXED geometry, independent of the hash bucket
 * width: 2048 rows x 32 slots x 2 bits, row = idx >> 5. */
#define CN_L3E_ENTRIES			65536	/* ht_size = 7 */
#define CN_L3E_HASH_WAYS		8	/* hb_size = 1 (stock live) */
#define CN_L3E_AGE_SLOTS		32	/* slots per age row, fixed */
#define CN_L3E_AGE_ROWS			(CN_L3E_ENTRIES / CN_L3E_AGE_SLOTS)
#define CN_L3E_FIB_BYTES		32	/* ha_width = 3 (256-bit, normal mode) */
#define CN_L3E_KEY_BYTES		92	/* packed key = CRC input */

/* 2-bit age codes (this chip; flow_entry_show on stock prints STATIC at 3) */
#define CN_L3E_AGE_FREE			0
#define CN_L3E_AGE_IDLE			1	/* set by the stats sweep; HW re-arms on hit */
#define CN_L3E_AGE_START		2
#define CN_L3E_AGE_STATIC		3

/* hash profiles, selected by the ingress CLE profile */
#define CN_L3E_PROFILE_WAN		0
#define CN_L3E_PROFILE_LAN		1
/* ★ P4: the profile the LIVE routed admission actually stamps (the LAN
 * catch-all rows carry t2_ctrl=3; cortina-l3fe.c re-points profile 3's tuple
 * at the 5-tuple mask 8 + traps its miss to CPU_0).  An install whose profile
 * != the stamped one can never HIT - so US (LAN->WAN) transit flows install
 * under profile 3. */
#define CN_L3E_PROFILE_ROUTED		3

/* mask-table index per profile (= PROFILE_TUPLE.maskptr the classify config
 * programs; must equal cortina-l3fe.c's mask-table setup so the SWO hash
 * matches the lookup).
 *
 * ★ MASK IDENTITY - RESOLVED on live HW (2026-07-18, divergence-A close).
 * Decoding the 8 stock masks (aal_hash_mask_t layout; mask bit 1 EXCLUDES a
 * field) + a single-bit SWO learn under each shows:
 *   mask[0] (== mask[7]): KEEPS l4 dport+sport, ip proto, full IPv4 SA+DA
 *                         (+ MAC) - the 5-TUPLE / NAPT mask.
 *   mask[1]            : EXCLUDES the whole IP tuple, keeps MAC DA/SA +
 *                         ethertype - an L2 / BRIDGE mask.
 * So a routed IPv4 flow hashes under mask 0, NOT mask 1.  (An earlier note
 * called mask 1 "the 5-tuple mask" - that was the misdiagnosis behind the
 * constant-CRC symptom: our key was fed to the SWO in the wrong layout AND
 * under the bridge mask, so every IP field was masked out.) */
/*
 * ★ P3 (2026-07-19): install + lookup under the dedicated 5-TUPLE-ONLY mask
 * (index 8, programmed by cortina_l3fe_classify_setup, routed profiles
 * re-pointed at it by cortina_l3fe_hw_l3_forward_enable).  Stock mask 0 also
 * keeps mac_sa/mac_da/lspid/ip_dscp/ip_ecn/VLAN (non-zero on a real routed
 * frame, zero in the sparse cn_l3e_key) so a mask-0 install CRC can never
 * equal the parsed packet's lookup CRC.  Mask 8 keeps ONLY the 5-tuple, so a
 * sparse key hashes identically to a matching parsed packet (swolearn-proven).
 */
#define CN_L3E_WAN_MASK_ID		8	/* routed IPv4 5-tuple mask */
#define CN_L3E_LAN_MASK_ID		8	/* routed flow, either direction */
#define CN_L3E_BRIDGE_MASK_ID		1	/* L2 (MAC) key, non-routed */

/* ------------------------------------------------------------------ */
/* ★ HW HDR_I descriptor layout - the key the SWO engine ACTUALLY      */
/* hashes.  The engine does NOT hash our SW cn_l3e_key (the 92-byte    */
/* aal_hash_key_t); it hashes the 128-byte L3FE_HDR_I descriptor the   */
/* classify/parse stage builds for a packet.  So a flow's fields must  */
/* be packed into HDR_I bit positions before feeding the SWO - the     */
/* SW-tuple -> HDR_I conversion below (cn_l3e_build_hdri).             */
/*                                                                     */
/* Bit offsets are LSB-first within the 128-byte little-endian buffer, */
/* recovered TIER-1 from a single-bit SWO learn on the live engine     */
/* under the 5-tuple mask (each field's bits proven to move the CRC),  */
/* cross-checked against the stock ca-ne.ko HDR_I build                */
/* (aal_hash_crc_sw_hw_calc_check).  These are the 9607F "07f" layout, */
/* which differs from the sibling gen2 struct in the IP region (+24 at */
/* the DA, +20 after) - the live learn is authoritative.               */
/* ------------------------------------------------------------------ */
#define CN_L3E_HDRI_BYTES		128
#define CN_L3E_HDRI_WORDS		(CN_L3E_HDRI_BYTES / 4)
/* 5-tuple + IP validity - each proven LIVE (moves the SWO CRC) on the real
 * engine under mask 0; ip_ver/ip_vld = the [504:505] learn window. */
#define CN_HDRI_L4_DP			74	/* dest L4 port, 16b */
#define CN_HDRI_L4_SP			90	/* src  L4 port, 16b */
#define CN_HDRI_IP_DA0			233	/* IPv4 DA / v6 DA LSW; 128b field [233:360] */
#define CN_HDRI_IP_SA0			361	/* IPv4 SA / v6 SA LSW; 128b field [361:488] */
#define CN_HDRI_IP_L4_TYPE		489	/* 3b; masked under mask 0, kept for other masks */
#define CN_HDRI_IP_PROTO		492	/* IP protocol, 8b */
#define CN_HDRI_IP_VER			504	/* 1b: 0 = IPv4 */
#define CN_HDRI_IP_VLD			505	/* 1b: 1 = has an IP header */
/* profile id stamp: HDR_I t2_ctrl (== the SW key's ctrl_set_id).  Position is
 * chip-cut dependent - a_cut(rev'A', ca_soc_data==0x41) [961:964], b_cut
 * [965:968]; BOTH are masked-out under the routed-flow mask (mask 0, board-
 * verified 2026-07-18), so this stamp does NOT affect a 5-tuple flow's CRC and
 * the cut choice is non-load-bearing here.  Placed at the a_cut offset,
 * mirroring stock aal_hash_crc_sw_hw_calc_check (hdr_i.t2_ctrl = ctrl_set_id).
 * (07f HDR_I has NO separate table_id field - table selection is t0/t1/t2_ctrl,
 * and the SW key's table_id is always mask-zeroed before the CRC.) */
#define CN_HDRI_T2_CTRL			961	/* 4b, a_cut */

/* ------------------------------------------------------------------ */
/* Flow key / action - packed to the engine's exact bit layout.        */
/* u64 bitfields, LSB-first on arm64: matches the on-DDR layout the    */
/* stock driver emits.  Only the fields our 5-tuple mask leaves live   */
/* need real values; everything the mask covers is zeroed before CRC.  */
/* ------------------------------------------------------------------ */

struct cn_l3e_key {
	/* L4 */
	u64 l4_chksum_zero	: 1;
	u64 tcp_flags		: 9;
	u64 l4_dport		: 16;
	u64 l4_sport		: 16;
	/* L3 */
	u64 l3_chksum_err	: 1;
	u64 spi			: 32;
	u64 spi_vld		: 3;
	u64 icmp_type		: 8;
	u64 icmp_vld		: 3;
	u64 ipv6_doh		: 1;
	u64 ipv6_rh		: 1;
	u64 ipv6_hbh		: 1;
	u64 ip_fragment		: 1;
	u64 ip_da_sa_equal	: 1;
	u64 ip_options		: 1;
	u64 ip_ttl		: 8;
	u64 ipv6_flow_lbl	: 20;
	u64 ip_da_0		: 32;	/* v4 DA or v6 DA LSW, host order */
	u64 ip_da_1		: 32;
	u64 ip_da_2		: 32;
	u64 ip_da_3		: 32;
	u64 ip_sa_0		: 32;	/* v4 SA or v6 SA LSW, host order */
	u64 ip_sa_1		: 32;
	u64 ip_sa_2		: 32;
	u64 ip_sa_3		: 32;
	u64 ip_l4_type		: 3;
	u64 ip_protocol		: 8;
	u64 ip_ecn		: 2;
	u64 ip_dscp		: 6;
	u64 ip_ver		: 1;
	u64 ip_vld		: 1;
	/* PPPoE */
	u64 ppp_proto_enc	: 4;
	u64 pppoe_session_id	: 16;
	u64 pppoe_code_enc	: 4;
	u64 pppoe_type		: 2;
	/* VLAN */
	u64 inner_dei		: 1;
	u64 inner_pcp		: 3;
	u64 inner_vid		: 12;
	u64 inner_tpid_enc	: 3;
	u64 top_dei		: 1;
	u64 top_pcp		: 3;
	u64 top_vid		: 12;
	u64 top_tpid_enc	: 3;
	u64 vlan_cnt		: 2;
	/* L2 format */
	u64 llc_type_enc	: 2;
	u64 llc_snap		: 2;
	u64 pktlen_rng_vec	: 4;
	u64 len_encoded		: 1;
	/* L2 */
	u64 ethertype_enc	: 6;
	u64 ethertype		: 16;
	u64 mac_sa_0		: 8;
	u64 mac_sa_1		: 8;
	u64 mac_sa_2		: 8;
	u64 mac_sa_3		: 8;
	u64 mac_sa_4		: 8;
	u64 mac_sa_5		: 8;
	u64 mac_da_rsvd		: 1;
	u64 mac_da_rng		: 1;
	u64 mac_da_ip_mc	: 1;
	u64 mac_da_an_sel	: 4;
	u64 mac_da_0		: 8;
	u64 mac_da_1		: 8;
	u64 mac_da_2		: 8;
	u64 mac_da_3		: 8;
	u64 mac_da_4		: 8;
	u64 mac_da_5		: 8;
	/* special packet */
	u64 spcl_pkt_hdr_mtch	: 8;
	u64 spcl_pkt_enc	: 6;
	/* metadata */
	u64 mdata		: 64;
	/* policer / cos */
	u64 qos_premark		: 1;
	u64 pol_grp_id		: 3;
	u64 pol_id		: 9;
	u64 cos			: 3;
	/* dest / source port */
	u64 mcgid		: 10;
	u64 mc			: 1;
	u64 mc_idx_vld		: 1;
	u64 orig_lspid		: 6;
	u64 lspid		: 6;
	/* hash control */
	u64 hkey_id		: 6;	/* mask-table index */
	u64 ctrl_set_id		: 4;	/* profile id (CRC input, then zeroed by its mask bit) */
	u64 table_id		: 4;
	u64 reserved		: 4;
} __packed;

static_assert(sizeof(struct cn_l3e_key) == CN_L3E_KEY_BYTES);

/*
 * Action FIB, "normal" mode = action groups 18 + 20 (a_mask 0x140000),
 * 224 bits packed, fetched as one 256-bit FIB entry.
 */
struct cn_l3e_act {
	/* group 18 - forward/permit (19 bits) */
	u64 mrr_vld		: 1;
	u64 mrr_en		: 1;
	u64 no_drop_vld		: 1;
	u64 no_drop		: 1;
	u64 dpid_vld		: 1;
	u64 dpid_pri		: 1;
	u64 permit		: 1;
	u64 deepq		: 1;
	u64 mcgid		: 10;
	u64 mc			: 1;
	/* group 20 - the NAT/encap rewrite (205 bits) */
	u64 mdata_byte_vld	: 1;
	u64 mdata_byte		: 8;
	u64 l3_if_vld		: 1;
	u64 smac_trans		: 1;
	u64 igr_l3_if_idx	: 6;
	u64 egr_l3_if_idx	: 6;
	u64 l3_if_counter_en	: 1;
	u64 ip_ttl_dec		: 1;
	u64 ip_ttl_zero_drop	: 1;
	u64 ip_addr_vld		: 1;
	u64 ip_type		: 1;	/* which address is rewritten: 0 = SA, 1 = DA */
	u64 ip_addr		: 32;	/* the new IPv4 address */
	u64 ip_addr_napt6	: 1;
	u64 mac_da_idx_vld	: 1;
	u64 mac_da_idx		: 13;	/* next-hop MAC via the MAC-DA table */
	u64 chk_msk_ptr		: 6;
	u64 cache_ctrl		: 2;
	u64 pop_l3_vld		: 1;
	u64 pop_l3_chk_ecn_en	: 1;
	u64 pop_l3_en		: 1;
	u64 t2_ctrl_vld		: 1;
	u64 t2_ctrl		: 4;
	u64 ldpid_offset_msb	: 1;
	u64 ip_dscp_update_en	: 1;
	u64 ip_dscp		: 6;
	u64 cos_update_en	: 1;
	u64 cos			: 3;
	u64 inner_pcp_update_en	: 1;
	u64 inner_pcp		: 3;
	u64 top_pcp_update_en	: 1;
	u64 top_pcp		: 3;
	u64 inner_dei		: 1;
	u64 inner_vid		: 12;
	u64 inner_tpid_enc	: 3;
	u64 top_dei		: 1;
	u64 top_vid		: 12;
	u64 top_tpid_enc	: 3;
	u64 vlan_cnt		: 2;
	u64 vlan_vld		: 1;
	u64 pol_vld		: 1;
	u64 pol_en		: 1;
	u64 pol_id		: 8;
	u64 pol2_id_en		: 1;
	u64 pol2_id		: 6;
	u64 pol3_id_en		: 1;
	u64 pol3_id		: 6;
	u64 pppoe_vld		: 1;
	u64 pppoe_set		: 1;
	u64 l4_port		: 16;	/* the new L4 port */
	u64 ip_mtu_enc_vld	: 1;
	u64 ip_mtu_enc		: 4;
	u64 modify_vlan_only_vld : 1;
	u64 modify_vlan_only	: 1;
	u64 sixrd_fmr_idx_vld	: 1;
	u64 sixrd_fmr_idx	: 2;
	u64 vxlan_sport_msb15	: 6;
	u64 vxlan_sport_update	: 1;
	/* pad to the 32-byte FIB entry */
	u64 pad			: 32;
} __packed;

static_assert(sizeof(struct cn_l3e_act) == CN_L3E_FIB_BYTES);

/* ------------------------------------------------------------------ */
/* backend context (filled by cn_l3e_init() from the cortina-ni probe  */
/* in the build/wiring phase; NULL = offload rejected everywhere)      */
/* ------------------------------------------------------------------ */

struct cn_flow_entry;

struct cn_l3e {
	struct device	*dev;
	void __iomem	*ne_base;	/* NE register window */
	spinlock_t	reg_lock;	/* serializes indirect GO cycles */
	/* DDR carve (one dma_alloc_coherent block, key then FIB - the NE
	 * fabric is NON-coherent, a cached carve = stale matches) */
	void		*carve;
	dma_addr_t	carve_pa;
	u32		*key_tbl;	/* 64K x u32 CRC32 */
	dma_addr_t	key_tbl_pa;
	void		*fib_tbl;	/* 64K x 32 B actions */
	dma_addr_t	fib_tbl_pa;
	/* lean SW shadow (allocated by cn_l3e_init; ~0.9 MB total - never
	 * the vendor's >7 MB per-entry kmalloc model) */
	u32		*shadow_crc32;	/* per entry, 0 = free */
	u16		*shadow_crc16;	/* for cache-invalidate on delete */
	struct cn_flow_entry **entry_by_idx;	/* sweep reverse map */
	u8		*bucket_occ;	/* entries per AGE row: sweep skip mask */
	/* SWO HW-CRC selftest verdict (phase-1 gate instrument) */
	int		selftest_ret;
	u32		selftest_pass;
	u32		selftest_fail;
	/* HDR_I 5-tuple key-packing verdict (divergence-A gate): each 5-tuple
	 * field must move the SWO CRC under the 5-tuple mask */
	u32		hdri_live_pass;
	u32		hdri_live_fail;
	/* router (LAN) MAC for the my-MAC FIELD-CAM commit; WAN = base+1 */
	u8		router_mac[ETH_ALEN];
	bool		router_mac_valid;
	/* LIVE PON data-path identity, reported by the GPON driver at data-GEM
	 * install (cortina_ni_gpon_data_path_set): the US hit-action egresses
	 * WAN-ward via mcgid=data_gem (mc=1) + t2_ctrl=data_tcont.  0 gem = no
	 * data path armed (US forward action is left CPU-only). */
	u16		data_gem;
	u8		data_tcont;
	/* LIVE PPPoE WAN session id (0 = IPoE WAN / no session - the default;
	 * US hit-actions then stay byte-identical to the proven IPoE shape).
	 * Set via cortina_ni_wan_pppoe_session_set (or /proc "pppoe <sess>")
	 * when the WAN negotiates a PPPoE session; a US hit-action then adds
	 * the 8-byte PPPoE header via the dedicated egress L3-IF entry. */
	u16		data_pppoe_session;
};

static struct cn_l3e *cn_l3e;

/* ------------------------------------------------------------------ */
/* CRC over the masked key (SW path; verified against the HS_SWO HW    */
/* CRC engine by a bring-up selftest before first use)                 */
/* ------------------------------------------------------------------ */

/* Steps 3-4 of the HW recipe (host-fuzz reference for the SW CRC path; the
 * runtime hash uses the SWO engine - see cn_l3e_key_hash). */
static __maybe_unused void cn_l3e_bitrev_key(u32 *w, int n_words)
{
	int i;
	u32 t;

	for (i = 0; i < n_words / 2; i++) {
		t = w[i];
		w[i] = bitrev32(w[n_words - 1 - i]);
		w[n_words - 1 - i] = bitrev32(t);
	}
	if (n_words & 1)
		w[i] = bitrev32(w[i]);
}

static __maybe_unused u32 cn_l3e_crc32(const u8 *p, size_t len)
{
	/* reflected CRC-32 (Ethernet poly), seed ~0, reflected output, no
	 * final xor - the engine's convention */
	return bitrev32(crc32_le(~0u, p, len));
}

static __maybe_unused u16 cn_l3e_crc16(const u8 *p, size_t len)
{
	/* reflected CRC-16/CCITT (poly 0x8408), seed 0xffff, reflected
	 * output, no final xor */
	u16 crc = 0xffff;
	int i;

	while (len--) {
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
	}
	return bitrev8(crc & 0xff) << 8 | bitrev8(crc >> 8);
}

/*
 * ★ HW hash recipe - FULLY RECOVERED 2026-07-18 (tier-2 Ghidra decomp of
 * stock hash_value_calculate() @0x11e0 + tier-1 single-bit SWO sweeps on the
 * live engine).  The lookup HW computes, for a 92-byte key and a 256-bit
 * mask-table entry:
 *   1. mask-apply, per FIELD (not per bit): field &= ~mask_field
 *      (mask bit 1 = EXCLUDE the field; 0 = keep it).
 *   2. HW-derived nonlinear FLAG bits (address-equal / zero / prefix-length
 *      checks) are folded into the reduced tuple - so even an all-zero key
 *      does NOT hash to CRC(zeros): the transform is NOT linear in the key
 *      bytes (proven: SWO(0)=0x7fc13ab0 != crc32(92*0x00)=0x3d4ad918).
 *   3. bit-reverse the whole key: 23 u32 words, bitrev32 EACH word AND
 *      reverse the WORD ORDER (cn_l3e_bitrev_key does exactly this).
 *   4. CRC32: reflected poly 0xEDB88320, seed ~0, final = bitrev32 (NO xor).
 *      CRC16: reflected poly 0x8408, seed 0xffff, final = bitrev16 (NO xor).
 *      (cn_l3e_crc32 / cn_l3e_crc16 implement 3+4 exactly - confirmed against
 *      the crctable_32/crctable_ccitt16 tables in the stock .ko.)
 *   5. optional per-profile CRC16 rotate/xor - gated on table_id==1 and
 *      indexed by hash_key_select[ctrl_set_id]; hash_key_select is .bss and
 *      stays 0 in stock (the <0x40 rule keeps CRC32 standard) => rotate is
 *      identity in practice.
 *
 * The nonlinear FLAG derivation (step 2) + the per-field mask-apply (step 1)
 * are ~1100 lines of stock logic; rather than transliterate them (fragile,
 * and the mask VALUES are external to ca-ne.ko anyway), the runtime hash
 * DRIVES THE ON-CHIP SWO CRC ENGINE - the SAME engine the lookup path uses -
 * so the {crc32,crc16} are IDENTICAL to what a parsed packet hashes to, by
 * construction (this is exactly what stock's own runtime add path does; the
 * SW CRC above is stock's host-fuzz-only fallback).  cn_l3e_crc32/16 +
 * cn_l3e_bitrev_key are kept as the host-fuzz reference for step 3/4.
 *
 * ★ KEY LAYOUT (divergence-A fix, 2026-07-18): the SWO engine hashes the
 * 128-byte HW HDR_I descriptor, NOT our 92-byte cn_l3e_key.  cn_l3e_key_hash
 * therefore converts the SW key to HDR_I (cn_l3e_build_hdri) and feeds all 32
 * HDR_I words to the SWO.  The profile id that stock aal_hash_value_cal_part_0
 * stamps into the SW key's ctrl_set_id lands in HDR_I as t2_ctrl (07f has no
 * separate table_id field; its SW-key table_id is always mask-zeroed pre-CRC).
 * (Feeding the raw 92-byte key - the previous bug - put every 5-tuple field in
 * a masked-out HDR_I position, so the CRC was constant; see cn_l3e_build_hdri.)
 */
/* set `width` bits at LSB-first bit offset `off` in a little-endian buffer */
static void cn_l3e_hdri_set(u8 *h, unsigned int off, unsigned int width, u64 val)
{
	unsigned int i;

	for (i = 0; i < width; i++, off++)
		if ((val >> i) & 1)
			h[off >> 3] |= 1u << (off & 7);
		/* buffer is pre-zeroed, so only 1-bits need writing */
}

/*
 * ★ SW-tuple -> HW HDR_I packing (divergence-A fix, 2026-07-18).
 *
 * The SWO/lookup engine hashes the 128-byte L3FE_HDR_I descriptor, NOT our
 * 92-byte cn_l3e_key (the aal_hash_key_t SW shadow).  Build the HDR_I with
 * every flow field at its HW bit position so the SWO CRC equals what the
 * classify/parse stage produces for a matching packet.  Only the fields the
 * 5-tuple mask (mask 0) leaves live matter to the CRC; the rest stay zero.
 *
 * Faithful to the stock HDR_I build (aal_hash_crc_sw_hw_calc_check): the
 * 5-tuple + ip_l4_type + ip_vld/ip_ver + the profile id stamp (t2_ctrl).  DA/SA
 * are 128-bit fields (4 consecutive 32-bit words) so an IPv6 key packs the
 * upper 96 bits too; an IPv4 key leaves them zero (masked out under mask 0).
 */
static void cn_l3e_build_hdri(const struct cn_l3e_key *k, int profile,
			      u32 words[CN_L3E_HDRI_WORDS])
{
	u8 h[CN_L3E_HDRI_BYTES] = { 0 };

	/* L4 */
	cn_l3e_hdri_set(h, CN_HDRI_L4_DP, 16, k->l4_dport);
	cn_l3e_hdri_set(h, CN_HDRI_L4_SP, 16, k->l4_sport);
	/* IP DA/SA - 4x32b each, LSW first (IPv4 = word 0 only) */
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 0,  32, k->ip_da_0);
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 32, 32, k->ip_da_1);
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 64, 32, k->ip_da_2);
	cn_l3e_hdri_set(h, CN_HDRI_IP_DA0 + 96, 32, k->ip_da_3);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 0,  32, k->ip_sa_0);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 32, 32, k->ip_sa_1);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 64, 32, k->ip_sa_2);
	cn_l3e_hdri_set(h, CN_HDRI_IP_SA0 + 96, 32, k->ip_sa_3);
	/* IP proto / L4 type / validity */
	cn_l3e_hdri_set(h, CN_HDRI_IP_L4_TYPE, 3, k->ip_l4_type);
	cn_l3e_hdri_set(h, CN_HDRI_IP_PROTO,   8, k->ip_protocol);
	cn_l3e_hdri_set(h, CN_HDRI_IP_VER,     1, k->ip_ver);
	cn_l3e_hdri_set(h, CN_HDRI_IP_VLD,     1, k->ip_vld);
	/* profile id stamp -> HDR_I t2_ctrl (mirrors stock; masked under mask 0) */
	cn_l3e_hdri_set(h, CN_HDRI_T2_CTRL,    4, profile & 0xf);

	memcpy(words, h, CN_L3E_HDRI_BYTES);
}

static int cn_l3e_key_hash(struct cn_l3e *l3e, const struct cn_l3e_key *key,
			   int profile, u32 mask_id, u32 *crc32_out,
			   u16 *crc16_out)
{
	u32 hdri[CN_L3E_HDRI_WORDS];
	unsigned long flags;
	int ret;

	cn_l3e_build_hdri(key, profile, hdri);

	/* SWO engine == lookup CRC, by construction (the engine hashes the
	 * HDR_I).  Serialized under reg_lock; flow-add is process/workqueue
	 * context, never the packet path. */
	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = cortina_l3fe_swo_crc(l3e->ne_base, hdri, CN_L3E_HDRI_WORDS,
				   mask_id, crc32_out, crc16_out);
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	return ret;
}

/* ------------------------------------------------------------------ */
/* indirect-access primitives                                          */
/* ------------------------------------------------------------------ */

static int cn_l3e_go(struct cn_l3e *l3e, u32 reg, u32 val, u32 busy_bit)
{
	int i;

	writel(val, l3e->ne_base + reg);
	for (i = 0; i < CN_L3E_POLL_TRIES; i++) {
		if (!(readl(l3e->ne_base + reg) & busy_bit))
			return 0;
		cpu_relax();
	}
	return -ETIMEDOUT;
}

/*
 * Age access: ACCESS = bucket | GO (read) -> RMW the 2-bit field in
 * DATA_LO/HI -> ACCESS = bucket | WRITE | GO.  A non-zero age is what
 * makes an entry live; age 0 kills it.
 *
 * ★ AGE WIDTH = 2 BITS on THIS die (tier-2, do NOT change to 4-bit).  The
 * shipping ca-ne.ko aal_hash_age_set (@0x965e0) rejects age>3 (`cmp #3;
 * b.hi`), masks the value with `& 0x3`, and writes a `bfi …,#width=2` field
 * at bit (slot & 0xf)*2 into DATA0=0x3938 (slots 0-15) / DATA1=0x3934 (slots
 * 16-31) - i.e. 2 bits/slot, 16 slots/word, 2 words = 32 slots/bucket.  This
 * is confirmed independently by the live age-decay validation (tier-1).  The
 * aal-gen2 SDK's 4-bit age layout (DATA0..3, START=6/STATIC=7) is a DIFFERENT
 * chip variant and does NOT apply here; writing a 4-bit field would land the
 * age in the wrong bits and leave each entry's real 2-bit slot 0 (INVALID ->
 * never a live hit).  Codes: 0=free, 1..2=aging (HW re-arms on hit), 3=static.
 */
static int cn_l3e_age_set(struct cn_l3e *l3e, u32 idx, u32 age)
{
	u32 bucket = (idx >> 5) & (CN_L3E_AGE_ROWS - 1);
	u32 data_reg = (idx & 0x10) ? CN_L3E_HS_AGE_DATA_HI
				    : CN_L3E_HS_AGE_DATA_LO;
	u32 shift = (idx & 0xf) * 2;
	const char *phase = "latch";
	unsigned long flags;
	u32 w;
	int ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = cn_l3e_go(l3e, CN_L3E_HS_AGE_ACCESS, bucket | CN_L3E_GO,
			CN_L3E_GO);
	if (ret)
		goto out;

	w = readl(l3e->ne_base + data_reg);
	w = (w & ~(3u << shift)) | ((age & 3) << shift);
	writel(w, l3e->ne_base + data_reg);

	phase = "commit";
	ret = cn_l3e_go(l3e, CN_L3E_HS_AGE_ACCESS,
			bucket | CN_L3E_WRITE | CN_L3E_GO, CN_L3E_GO);
out:
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	/* which GO timed out matters: a "commit" timeout means the age write
	 * WAS issued and may land late - the entry can go live after an error
	 * return, so the caller must fully undo (blackhole-safety). */
	if (ret)
		pr_err("cortina-l3fe: age_set idx=%u age=%u: %s GO timeout (%d)\n",
		       idx, age, phase, ret);
	return ret;
}

/* single-entry age read: the P2 bring-up oracle ("did my one flow HIT?" -
 * age 2 = matched since last sweep, 1 = live but idle, 0 = not live).
 * NEVER used on the stats path - that is the batch sweep's job. */
static int __maybe_unused cn_l3e_age_get(struct cn_l3e *l3e, u32 idx, u32 *age)
{
	u32 bucket = (idx >> 5) & (CN_L3E_AGE_ROWS - 1);
	u32 data_reg = (idx & 0x10) ? CN_L3E_HS_AGE_DATA_HI
				    : CN_L3E_HS_AGE_DATA_LO;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = cn_l3e_go(l3e, CN_L3E_HS_AGE_ACCESS, bucket | CN_L3E_GO,
			CN_L3E_GO);
	if (!ret)
		*age = (readl(l3e->ne_base + data_reg) >> ((idx & 0xf) * 2)) & 3;
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	return ret;
}

/*
 * Batch had-traffic read+clear of one 32-slot bucket in a single indirect
 * latch/commit pair - the ONLY stats primitive that scales (a per-flow
 * age_get would be 10k+ indirect reads at the design load).  Returns a
 * bitmap: bit k set == HW re-armed slot k's age since the last sweep
 * (age >= START, i.e. at least one packet matched).  Live slots are
 * rewritten to IDLE(1) so the next sweep sees fresh re-arms; STATIC(3)
 * slots are left untouched.
 */
static int cn_l3e_bucket_sweep(struct cn_l3e *l3e, u32 bucket, u32 *traffic)
{
	unsigned long flags;
	u32 w[2], n[2], trf = 0;
	int r, i, ret;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	ret = cn_l3e_go(l3e, CN_L3E_HS_AGE_ACCESS, bucket | CN_L3E_GO,
			CN_L3E_GO);
	if (ret)
		goto out;

	w[0] = readl(l3e->ne_base + CN_L3E_HS_AGE_DATA_LO);  /* slots 0-15  */
	w[1] = readl(l3e->ne_base + CN_L3E_HS_AGE_DATA_HI);  /* slots 16-31 */
	for (r = 0; r < 2; r++) {
		n[r] = 0;
		for (i = 0; i < 16; i++) {
			u32 age = (w[r] >> (i * 2)) & 3;

			if (age >= CN_L3E_AGE_START)
				trf |= BIT(r * 16 + i);
			if (age == CN_L3E_AGE_START)
				age = CN_L3E_AGE_IDLE;
			n[r] |= age << (i * 2);
		}
	}
	writel(n[0], l3e->ne_base + CN_L3E_HS_AGE_DATA_LO);
	writel(n[1], l3e->ne_base + CN_L3E_HS_AGE_DATA_HI);

	ret = cn_l3e_go(l3e, CN_L3E_HS_AGE_ACCESS,
			bucket | CN_L3E_WRITE | CN_L3E_GO, CN_L3E_GO);
out:
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	*traffic = trf;
	return ret;
}

/*
 * Action-cache invalidate - MANDATORY after every delete/update; a stale
 * cached action otherwise keeps matching {crc16, slot}.  "not cached"
 * (STS bit3) is a benign outcome.
 */
static int cn_l3e_cache_invalidate(struct cn_l3e *l3e, u32 idx, u16 crc16)
{
	unsigned long flags;
	int i, ret = -ETIMEDOUT;

	spin_lock_irqsave(&l3e->reg_lock, flags);
	/* slot = way within the HASH bucket (idx & (bucket_size-1); 8-way) */
	writel((idx & (CN_L3E_HASH_WAYS - 1)) | ((u32)crc16 << 5) | (1u << 30),
	       l3e->ne_base + CN_L3E_HS_CACHE_CTRL);

	for (i = 0; i < CN_L3E_POLL_TRIES; i++) {
		if (!(readl(l3e->ne_base + CN_L3E_HS_CACHE_CTRL_REQ) & 1))
			break;
		cpu_relax();
	}
	if (i < CN_L3E_POLL_TRIES)
		ret = cn_l3e_go(l3e, CN_L3E_HS_CACHE_CTRL_REQ,
				readl(l3e->ne_base + CN_L3E_HS_CACHE_CTRL_REQ) | 1,
				BIT(0));
	spin_unlock_irqrestore(&l3e->reg_lock, flags);
	if (ret)
		pr_err("cortina-l3fe: cache_invalidate idx=%u crc16=%04x FAILED (%d) - a stale cached action may keep matching\n",
		       idx, crc16, ret);
	return ret;
}

/* ------------------------------------------------------------------ */
/* flow add / delete on the engine                                     */
/* ------------------------------------------------------------------ */

static int cn_l3e_flow_add(struct cn_l3e *l3e, const struct cn_l3e_key *key,
			   const struct cn_l3e_act *act, int profile,
			   u32 mask_id, u32 *idx_out, u16 *crc16_out)
{
	u32 crc32, base, idx;
	u16 crc16;
	int way, ret;

	ret = cn_l3e_key_hash(l3e, key, profile, mask_id, &crc32, &crc16);
	if (ret) {
		pr_err("cortina-l3fe: flow_add: SWO key-hash timeout (%d)\n",
		       ret);
		return ret;	/* SWO timeout: refuse, flow stays on the sw path */
	}

	/* SW way-pick inside the 8-way hash bucket (stock hb_size = 1);
	 * guard: keep entry 0 free (its {crc16, slot} cache tag is all-zero
	 * and aliases an empty cache way) */
	base = crc16 & ~(u32)(CN_L3E_HASH_WAYS - 1);
	for (way = 0; way < CN_L3E_HASH_WAYS; way++)
		if (l3e->shadow_crc32[base + way] == crc32) {
			/* normal dup-key (not an error): flow already installed */
			pr_debug("cortina-l3fe: flow_add: EEXIST idx=%u crc32=%08x crc16=%04x\n",
				 base + way, crc32, crc16);
			return -EEXIST;
		}
	way = (base == 0) ? 1 : 0;
	for (; way < CN_L3E_HASH_WAYS; way++)
		if (!l3e->shadow_crc32[base + way])
			break;
	if (way == CN_L3E_HASH_WAYS) {
		/* bucket full: the flow simply stays on the sw path (not an error) */
		pr_debug("cortina-l3fe: flow_add: bucket FULL base=%u crc16=%04x\n",
			 base, crc16);
		return -ENOSPC;
	}
	idx = base + way;

	/* 1. action FIB, 2. key word, 3. age = go-live (order matters) */
	memcpy(l3e->fib_tbl + (size_t)idx * CN_L3E_FIB_BYTES, act,
	       CN_L3E_FIB_BYTES);
	l3e->key_tbl[idx] = crc32;
	/* both tables live in a non-cacheable/coherent carve; make the
	 * stores visible to the engine before arming the age */
	wmb();

	ret = cn_l3e_age_set(l3e, idx, CN_L3E_AGE_START);
	if (ret) {
		/* ★ Blackhole-safety: a "commit" GO timeout means the age
		 * write WAS issued and can land late - the entry may go LIVE
		 * after this error return.  Fully undo: kill the key first
		 * (no new matches), zero the action, then best-effort force
		 * the age back to FREE and invalidate the action cache so a
		 * transient hit can never leave a stale cached action
		 * matching {crc16, slot} with an all-zero (= discard) FIB. */
		l3e->key_tbl[idx] = 0;
		memset(l3e->fib_tbl + (size_t)idx * CN_L3E_FIB_BYTES, 0,
		       CN_L3E_FIB_BYTES);
		wmb();
		cn_l3e_age_set(l3e, idx, CN_L3E_AGE_FREE);
		cn_l3e_cache_invalidate(l3e, idx, crc16);
		return ret;
	}

	l3e->shadow_crc32[idx] = crc32;
	l3e->shadow_crc16[idx] = crc16;
	*idx_out = idx;
	*crc16_out = crc16;
	return 0;
}

static int cn_l3e_flow_del(struct cn_l3e *l3e, u32 idx, u16 crc16)
{
	int age_ret, inv_ret;

	/* ★ Blackhole-safety: run EVERY teardown step even when one fails.
	 * The old early-return on an age timeout left the full entry (key +
	 * action + live age) orphaned and matching forever.  Kill the key
	 * FIRST (no new lookups can match an entry whose CRC is 0), zero the
	 * action, then the age and the action cache; report the first error
	 * but never skip a step because of it. */
	l3e->key_tbl[idx] = 0;
	memset(l3e->fib_tbl + (size_t)idx * CN_L3E_FIB_BYTES, 0,
	       CN_L3E_FIB_BYTES);
	wmb();
	l3e->shadow_crc32[idx] = 0;
	l3e->shadow_crc16[idx] = 0;

	age_ret = cn_l3e_age_set(l3e, idx, CN_L3E_AGE_FREE);
	inv_ret = cn_l3e_cache_invalidate(l3e, idx, crc16);

	return age_ret ? age_ret : inv_ret;
}

/* ------------------------------------------------------------------ */
/* mainline flow_block glue (mtk_ppe_offload-shaped)                   */
/* ------------------------------------------------------------------ */

struct cn_flow_entry {
	struct rhash_head	node;
	unsigned long		cookie;
	u32			hash_idx;
	u16			crc16;
	unsigned long		last_hit;	/* fed by the stats sweep */
};

static struct rhashtable cn_flow_table;
static bool cn_flow_table_ready;
/* Flow installs stay OFF until the P3 key-packing is SWO-validated: an
 * entry installed with the placeholder CRC could never match (harmless)
 * but would waste table slots and report a false "offloaded" state. */
static bool cn_l3e_install_ok;

/*
 * ★ Divergence B gate (default OFF).  When set, cn_l3e_init also programs the
 * HW L3-forwarding enable (hash-miss->CPU internal FIB + CLS routing defaults
 * + per-port hash-consult), so a routed frame consults the L3FE main hash
 * (lookup-then-trap-on-miss) instead of being software-forwarded.  This is
 * the first datapath-touching step: keep it OFF for a clean baseline, flip it
 * via the bootarg `cortina_ni.hw_l3_fwd=1`, and require a zero-flow
 * NO-REGRESSION boot (GPON O5+WAN, LAN NAT, WiFi, SSH all unchanged - every
 * packet misses the empty hash -> CPU) before installing any flow.
 */
static bool hw_l3_fwd;
module_param(hw_l3_fwd, bool, 0644);
MODULE_PARM_DESC(hw_l3_fwd,
	"enable HW L3-forwarding into the L3FE main hash, miss->CPU (default OFF; needs zero-flow no-regression proof)");

/* # of installed nf_flow_table flows (the /proc auto_flows counter); defined
 * here so the PPPoE session-set path below can gate its BUG-B flush on it. */
static atomic_t cn_flow_installed = ATOMIC_INIT(0);

/* Flush every installed nf_flow_table flow (defined after the flow table
 * below); used on a PPPoE sid change - see BUG-B. */
static void cn_l3e_flush_auto_flows(struct cn_l3e *l3e);

/*
 * Cross-module gate probe for the WAN-side ingress admission: the GPON
 * driver (cortina_gpon.ko) consults this at data-GEM install time to decide
 * the PDC route for DS data frames - LDPID L3_WAN (0x18, into the L3FE, the
 * vendor-default route) when the HW L3-forward experiment is armed, or the
 * proven CPU_0 + FE-bypass delivery otherwise.  True only when the operator
 * set hw_l3_fwd=1 AND the engine init actually succeeded (cn_l3e armed), so
 * a failed L3FE bring-up can never leave DS data pointed at a dead engine.
 */
bool cortina_ni_hw_l3_fwd_active(void)
{
	return hw_l3_fwd && cn_l3e;
}
EXPORT_SYMBOL_GPL(cortina_ni_hw_l3_fwd_active);

/*
 * The GPON driver reports the LIVE data-path identity (data GEM port-id + hw
 * T-CONT index) whenever it arms/tears down the WAN data path.  A US
 * (LAN->WAN) hit-action forwards via mcgid=gem (mc=1) + t2_ctrl=tcont, so
 * these must be the OLT-provisioned values, never a constant.  gem_id 0 =
 * torn down (US flows then keep the CPU disposition until re-armed).
 */
void cortina_ni_gpon_data_path_set(u16 gem_id, u8 tcont_idx)
{
	if (!cn_l3e)
		return;
	WRITE_ONCE(cn_l3e->data_gem, gem_id);
	WRITE_ONCE(cn_l3e->data_tcont, tcont_idx);
	pr_info("cortina-l3fe: live PON data-path gem=%u tcont=%u\n",
		gem_id, tcont_idx);
}
EXPORT_SYMBOL_GPL(cortina_ni_gpon_data_path_set);

/* GROUP_18/20 offset within the PON US ldpid map (aal_l3pe ldpid_base=0x20);
 * a T-CONT <= 7 rides the deep queue (vendor flow.c:1116). */
#define CN_L3E_PON_DEEPQ_TCONT_MAX	7

/* The dedicated egress L3-IF entry carrying the PPPoE WAN session header
 * (entry 1; 0 is left free so an all-zero egr_l3_if_idx never aliases it). */
#define CN_L3E_PPPOE_L3IF_IDX		1

/*
 * LIVE PPPoE WAN session push (0 = torn down / IPoE).  Same push model as
 * cortina_ni_gpon_data_path_set above: called when the WAN (re)negotiates a
 * PPPoE session - for the first bring-up via /proc/cortina_l3fe
 * ("pppoe <sess>"), later from the WAN-config plumbing.  Programs the
 * dedicated egress L3-IF entry {pppoe_set, pppoe_vld, session} that US
 * hit-actions select (cn_l3e_set_us_egress); session 0 clears it.  The HW
 * write only happens under the hw_l3_fwd gate (matching every other L3FE
 * datapath write); gate-off is byte-identical.
 */
int cortina_ni_wan_pppoe_session_set(u16 session)
{
	struct cn_l3e *l3e = cn_l3e;
	unsigned long flags;
	int ret = 0;

	if (!l3e)
		return -ENODEV;

	/* ★ BUG-B: a REAL session-id change invalidates every offloaded flow -
	 * they all ride the SINGLE shared L3-IF[1] entry, so once it is
	 * reprogrammed with the new sid the old flows would emit the new (or,
	 * on a clear, an absent) header on the wire.  Flush them so
	 * nf_flow_table reinstalls the still-live conntracks against the new
	 * sid; never leave a live flow carrying a stale sid.  Cheap + rare;
	 * the entry_by_idx scan avoids an rhashtable-walk use-after-free.
	 * ★ Caller MUST hold cn_flow_offload_mutex (both in-tree callers do:
	 * the /proc "pppoe" write and cn_l3e_set_us_egress via cn_flow_replace). */
	if (session != READ_ONCE(l3e->data_pppoe_session) &&
	    atomic_read(&cn_flow_installed))
		cn_l3e_flush_auto_flows(l3e);

	if (hw_l3_fwd) {
		spin_lock_irqsave(&l3e->reg_lock, flags);
		ret = cortina_l3fe_pppoe_l3if_set(l3e->ne_base,
						  CN_L3E_PPPOE_L3IF_IDX,
						  session);
		spin_unlock_irqrestore(&l3e->reg_lock, flags);
	}
	/* ★ BUG-A: commit the shadow ONLY after the HW L3-IF entry is actually
	 * programmed.  On a failed/timed-out write leave data_pppoe_session
	 * unchanged so (a) cn_l3e_set_us_egress sees the error and REFUSES the
	 * offload (no live flow pointing at a stale/zero L3-IF entry), and (b)
	 * the next install retries the reprogram instead of trusting a
	 * never-written entry (the old code advanced the shadow first, so a
	 * later same-sid flow skipped the retry forever). */
	if (!ret)
		WRITE_ONCE(l3e->data_pppoe_session, session);
	pr_info("cortina-l3fe: PPPoE WAN session %#x %s (L3-IF[%u] ret=%d)\n",
		session, session ? "armed" : "cleared",
		CN_L3E_PPPOE_L3IF_IDX, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(cortina_ni_wan_pppoe_session_set);

/*
 * Stamp the US (LAN->WAN) PON egress into a hit-action: gemMapMode-1
 * encoding (vendor flow.c FORWARD_PORT / CN2 mode[1]) -
 *   GROUP_18: mc=1, mcgid=gem_id, deepq=(tcont<=7), dpid_vld/permit/dpid_pri;
 *   GROUP_20: t2_ctrl=tcont (-> hdr_a.ldpid = ldpid_base 0x20 + tcont) and
 *             pop_l3_vld=1 (the sole gemMapMode-1 reuse bit; t2_ctrl_vld=0).
 * With PE_CFG.gemid_map=1 (set in cortina_l3fe_hw_l3_forward_enable) this
 * egresses at the SAME PON US ldpid the proven CPU data-TX path injects with.
 *
 * @live_sid: the LIVE PPPoE session id carried by the flow rule's
 * FLOW_ACTION_PPPOE_PUSH entry (fa->pppoe.sid, u16 host-order - the kernel
 * resolves it from the pppoe socket via pppoe_fill_forward_path ->
 * nft_flow_offload -> nf_flow_rule_route_common, the mtk_ppe precedent), or
 * 0 when the caller has none (manual /proc install) - then the /proc-set
 * data_pppoe_session bring-up fallback applies.  Returns 0, or -ENODEV if
 * no data path is armed yet.
 */
static int cn_l3e_set_us_egress(struct cn_l3e *l3e, struct cn_l3e_act *act,
				u16 live_sid)
{
	u16 gem = READ_ONCE(l3e->data_gem);
	u16 pppoe = live_sid ? live_sid : READ_ONCE(l3e->data_pppoe_session);
	u8 tcont = READ_ONCE(l3e->data_tcont);

	if (!gem)
		return -ENODEV;

	act->mc = 1;
	act->mcgid = gem & 0x3ff;
	act->dpid_vld = 1;
	act->permit = 1;
	act->dpid_pri = 1;
	act->deepq = (tcont <= CN_L3E_PON_DEEPQ_TCONT_MAX);
	act->t2_ctrl = tcont & 0xf;	/* GROUP_20 t2_ctrl1 = T-CONT selector */
	act->pop_l3_vld = 1;		/* gemMapMode-1 marker (bit0) */

	/* PPPoE WAN egress: ADD the 8-byte 0x8864 session header on the hit.
	 * GROUP_20 inline pppoe_set1=1 + pppoe_vld1=1 = ADD/replace; the
	 * session id rides the egress L3-IF entry selected by l3_if_vld1 +
	 * egr_l3_if_idx1 (programmed by cortina_ni_wan_pppoe_session_set;
	 * mac_sa_vld=0 there, SMAC untouched) and the PE globals 0x3500/0x3504
	 * supply code/ver/type + the PPP protocol.  This indexed path IS the
	 * vendor per-flow PPPoE mechanism in flow-normal mode (a_mask G18|G20,
	 * L3FE_NAPT_ACTION_SERIALIZATION.md section 8): the inline GROUP_07
	 * session field is not fetched under this a_mask, and widening the
	 * a_mask would repack the whole FIB layout.  session==0 (IPoE) leaves
	 * all four fields 0 - the action stays byte-identical to the proven
	 * IPoE shape.  Only reachable under hw_l3_fwd.
	 *
	 * A LIVE sid that differs from the armed one (first offloaded flow of
	 * a session, or a re-dial with a new sid) re-programs the L3-IF entry
	 * so the on-wire header always carries the negotiated id - never a
	 * stale or constant one. */
	if (pppoe) {
		if (pppoe != READ_ONCE(l3e->data_pppoe_session)) {
			/* ★ BUG-A: PROPAGATE the L3-IF write result.  If the HW
			 * L3-IF program fails/times out, REFUSE the offload - do
			 * NOT stamp l3_if_vld/egr_l3_if_idx into a live action
			 * that would then point at a stale/zero L3-IF[1] and
			 * blackhole (the header would be absent / sid 0 on the
			 * wire).  The flow stays on the SW path, which forwards
			 * PPPoE correctly. */
			int r = cortina_ni_wan_pppoe_session_set(pppoe);

			if (r)
				return r;
		}
		act->pppoe_set = 1;
		act->pppoe_vld = 1;
		act->l3_if_vld = 1;
		act->egr_l3_if_idx = CN_L3E_PPPOE_L3IF_IDX;
	}
	return 0;
}

static const struct rhashtable_params cn_flow_ht_params = {
	.head_offset	= offsetof(struct cn_flow_entry, node),
	.key_offset	= offsetof(struct cn_flow_entry, cookie),
	.key_len	= sizeof(unsigned long),
	.automatic_shrinking = true,
};

static DEFINE_MUTEX(cn_flow_offload_mutex);

/*
 * Liveness sweep: every CN_L3E_SWEEP_MS walk ONLY the occupied buckets, one
 * batch read+clear each, and stamp last_hit on every flow the hardware saw
 * traffic for.  FLOW_CLS_STATS then answers from last_hit with ZERO MMIO.
 * Worst case (all 2048 buckets occupied) ~= 2048 bounded indirect ops every
 * sweep - a few ms of CPU, ~0.1%.  Keep the period <= a third of the
 * nf_flow_table offload timeout (30 s default) so a HW-refreshed flow can
 * never look stale to nf gc.
 */
#define CN_L3E_SWEEP_MS		5000

static void cn_l3e_sweep_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(cn_l3e_sweep, cn_l3e_sweep_work);

static void cn_l3e_sweep_work(struct work_struct *work)
{
	struct cn_l3e *l3e = cn_l3e;
	unsigned long traffic;
	u32 bucket, trf;
	int slot;

	if (!l3e)
		return;

	mutex_lock(&cn_flow_offload_mutex);
	for (bucket = 0; bucket < CN_L3E_AGE_ROWS; bucket++) {
		if (!l3e->bucket_occ[bucket])
			continue;
		if (cn_l3e_bucket_sweep(l3e, bucket, &trf))
			continue;	/* bounded timeout: retry next sweep */

		traffic = trf;
		for_each_set_bit(slot, &traffic, CN_L3E_AGE_SLOTS) {
			struct cn_flow_entry *e =
				l3e->entry_by_idx[bucket * CN_L3E_AGE_SLOTS +
						  slot];

			if (e)
				e->last_hit = jiffies;
		}
		if (!(bucket & 0x3f))
			cond_resched();
	}
	mutex_unlock(&cn_flow_offload_mutex);

	schedule_delayed_work(&cn_l3e_sweep, msecs_to_jiffies(CN_L3E_SWEEP_MS));
}

/* Names every cn_flow_replace refusal branch so a rejected/silently-erroring
 * REPLACE localises itself.  At pr_debug (dynamic-debug): first-class dump/spy
 * per project policy, but silent at the default loglevel so the shipped tree
 * is not info-spammy under a many-flow load. */
#define cn_rep_dbg(fmt, ...) \
	pr_debug("cn_flow_replace: " fmt, ##__VA_ARGS__)

static int cn_flow_replace(struct flow_cls_offload *f, struct net_device *dev)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct flow_action_entry *fa;
	struct cn_l3e_key key = {};
	struct cn_l3e_act act = {};
	struct cn_flow_entry *entry;
	struct net_device *odev = NULL;
	bool lan_ingress = false, snat_port = false;
	int profile, i, err;
	u16 addr_type = 0;
	u16 pppoe_sid = 0;

	if (!cn_l3e || !cn_l3e_install_ok)
		return -EOPNOTSUPP;
	if (rhashtable_lookup_fast(&cn_flow_table, &f->cookie,
				   cn_flow_ht_params)) {
		cn_rep_dbg("cookie %lx already installed (other direction)\n",
			   f->cookie);
		return -EEXIST;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_CONTROL)) {
		struct flow_match_control m;

		flow_rule_match_control(rule, &m);
		addr_type = m.key->addr_type;
	}
	if (addr_type != FLOW_DISSECTOR_KEY_IPV4_ADDRS)
		return -EOPNOTSUPP;	/* phase 1: IPv4 NAPT only */

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_BASIC)) {
		struct flow_match_basic m;

		flow_rule_match_basic(rule, &m);
		if (m.key->ip_proto != IPPROTO_TCP &&
		    m.key->ip_proto != IPPROTO_UDP)
			return -EOPNOTSUPP;
		key.ip_protocol = m.key->ip_proto;
	} else {
		return -EOPNOTSUPP;
	}

	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_IPV4_ADDRS)) {
		struct flow_match_ipv4_addrs m;

		flow_rule_match_ipv4_addrs(rule, &m);
		key.ip_sa_0 = be32_to_cpu(m.key->src);
		key.ip_da_0 = be32_to_cpu(m.key->dst);
	}
	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_PORTS)) {
		struct flow_match_ports m;

		flow_rule_match_ports(rule, &m);
		key.l4_sport = be16_to_cpu(m.key->src);
		key.l4_dport = be16_to_cpu(m.key->dst);
	} else {
		return -EOPNOTSUPP;
	}
	key.ip_ver = 0;			/* IPv4 */
	key.ip_vld = 1;

	/* Ingress side: the block-cb dev is NOT the flow's ingress (every
	 * registered cb sees every rule); the rule carries the real ingress
	 * ifindex in its META key.  This phase installs ONLY the US (LAN->WAN)
	 * transit direction - ingress on the LAN bridge (or a bridge port once
	 * the kernel resolved the forward path) - under the ROUTED profile the
	 * live admission stamps (t2_ctrl=3).  The WAN-ingress (DS reply) leg
	 * is refused: nf_flow_table accepts a unidirectional HW offload
	 * (flow_offload_rule_add ok_count) and the reply keeps the proven
	 * hash-miss CPU punt path. */
	if (flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_META)) {
		struct flow_match_meta m;
		struct net_device *idev;

		flow_rule_match_meta(rule, &m);
		idev = dev_get_by_index(dev_net(dev), m.key->ingress_ifindex);
		if (idev) {
			lan_ingress = netif_is_any_bridge_port(idev) ||
				      netif_is_bridge_master(idev);
			dev_put(idev);
		}
	}
	if (!lan_ingress) {
		cn_rep_dbg("refuse: not LAN ingress (reply/DS leg keeps the CPU path)\n");
		return -EOPNOTSUPP;
	}
	profile = CN_L3E_PROFILE_ROUTED;

	flow_action_for_each(i, fa, &rule->action) {
		switch (fa->id) {
		case FLOW_ACTION_REDIRECT:
			odev = fa->dev;
			break;
		case FLOW_ACTION_MANGLE:
			switch (fa->mangle.htype) {
			case FLOW_ACT_MANGLE_HDR_TYPE_IP4:
				/* offset 12 = saddr: the SNAT rewrite.  A
				 * daddr rewrite (16, DNAT) is not the US
				 * transit shape - leave it to software. */
				if (fa->mangle.offset !=
				    offsetof(struct iphdr, saddr)) {
					cn_rep_dbg("refuse: IP4 mangle off=%u (not SNAT saddr)\n",
						   fa->mangle.offset);
					return -EOPNOTSUPP;
				}
				act.ip_addr_vld = 1;
				act.ip_type = 0;	/* rewrite SA */
				act.ip_addr = ntohl(fa->mangle.val);
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_TCP:
			case FLOW_ACT_MANGLE_HDR_TYPE_UDP:
				/* nf_flow_table encodes the port rewrite as
				 * one big-endian 32-bit word at offset 0:
				 * source port in the upper half, dest port in
				 * the lower (mask ~0xffff).  Only the source
				 * rewrite belongs to the US SNAT shape. */
				if (fa->mangle.offset != 0 ||
				    fa->mangle.mask == ~htonl(0xffff)) {
					cn_rep_dbg("refuse: L4 mangle off=%u mask=%08x (not sport SNAT)\n",
						   fa->mangle.offset,
						   ntohl(fa->mangle.mask));
					return -EOPNOTSUPP;
				}
				act.l4_port = ntohl(fa->mangle.val) >> 16;
				snat_port = true;
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_ETH:
				/* dmac/smac rewrite: intentionally NOT
				 * applied (see the action comment above) */
				break;
			default:
				return -EOPNOTSUPP;
			}
			break;
		case FLOW_ACTION_CSUM:
			break;		/* implicit in the HW rewrite */
		case FLOW_ACTION_PPPOE_PUSH:
			/* PPPoE WAN: the kernel emits this on the US rule with
			 * the LIVE negotiated session id (host-order u16, from
			 * the reply tuple's encap - nf_flow_table_offload
			 * nf_flow_rule_route_common; mtk_ppe precedent).  Feed
			 * it into the US egress encap below - NEVER the /proc
			 * constant. */
			pppoe_sid = fa->pppoe.sid;
			break;
		default:
			cn_rep_dbg("refuse: unsupported flow action id=%d\n",
				   fa->id);
			return -EOPNOTSUPP;
		}
	}
	/* keep to the proven shape: a full inline SNAT + a WAN redirect */
	if (!odev || !act.ip_addr_vld || !snat_port) {
		cn_rep_dbg("refuse: not the US SNAT shape (odev=%d ip=%d port=%d)\n",
			   !!odev, (int)act.ip_addr_vld, snat_port);
		return -EOPNOTSUPP;
	}

	/* US hit-action - EXACTLY the silicon-proven manual-install shape
	 * (GROUP_18 WAN-forward via the live PON data GEM/T-CONT + GROUP_20
	 * TTL dec + inline SNAT).  No l3_if/smac/mac_da indexed rewrite: those
	 * aux tables are not programmed, and the proven forward does not need
	 * them (the PON US egress needs no neighbour-MAC rewrite).  Runs
	 * AFTER the action loop so a FLOW_ACTION_PPPOE_PUSH sid (collected
	 * above) drives the PPPoE encap; sid 0 = IPoE, byte-identical action.
	 * If no data path is armed yet, refuse - the flow stays on the SW
	 * path. */
	err = cn_l3e_set_us_egress(cn_l3e, &act, pppoe_sid);
	if (err) {
		cn_rep_dbg("refuse: no PON data path armed (set_us_egress %d)\n",
			   err);
		return -EOPNOTSUPP;
	}
	act.ip_ttl_dec = 1;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		pr_err("cn_flow_replace: entry alloc failed\n");
		return -ENOMEM;
	}
	entry->cookie = f->cookie;
	entry->last_hit = jiffies;

	err = cn_l3e_flow_add(cn_l3e, &key, &act, profile, CN_L3E_WAN_MASK_ID,
			      &entry->hash_idx, &entry->crc16);
	if (err) {
		pr_err("cn_flow_replace: install FAILED (%d) %pI4h:%u->%pI4h:%u proto=%u pppoe=%#x\n",
		       err, &(u32){ key.ip_sa_0 }, (u16)key.l4_sport,
		       &(u32){ key.ip_da_0 }, (u16)key.l4_dport,
		       (u8)key.ip_protocol, pppoe_sid);
		goto free;
	}

	err = rhashtable_insert_fast(&cn_flow_table, &entry->node,
				     cn_flow_ht_params);
	if (err) {
		pr_err("cn_flow_replace: rhashtable insert FAILED (%d) - undoing idx=%u\n",
		       err, entry->hash_idx);
		cn_l3e_flow_del(cn_l3e, entry->hash_idx, entry->crc16);
		goto free;
	}

	/* register with the liveness sweep (under cn_flow_offload_mutex) */
	cn_l3e->entry_by_idx[entry->hash_idx] = entry;
	cn_l3e->bucket_occ[entry->hash_idx / CN_L3E_AGE_SLOTS]++;
	atomic_inc(&cn_flow_installed);
	/* per-flow install witness; pr_debug so a 1000-flow soak stays quiet */
	pr_debug("cn_flow_replace: INSTALLED idx=%u crc16=%04x %pI4h:%u->%pI4h:%u proto=%u pppoe=%#x snat=%pI4h:%u\n",
		 entry->hash_idx, entry->crc16,
		 &(u32){ key.ip_sa_0 }, (u16)key.l4_sport,
		 &(u32){ key.ip_da_0 }, (u16)key.l4_dport,
		 (u8)key.ip_protocol, pppoe_sid,
		 &(u32){ act.ip_addr }, (u16)act.l4_port);
	return 0;
free:
	kfree(entry);
	return err;
}

static int cn_flow_destroy(struct flow_cls_offload *f)
{
	struct cn_flow_entry *entry;

	entry = rhashtable_lookup_fast(&cn_flow_table, &f->cookie,
				       cn_flow_ht_params);
	if (!entry)
		return -ENOENT;

	cn_l3e->entry_by_idx[entry->hash_idx] = NULL;
	cn_l3e->bucket_occ[entry->hash_idx / CN_L3E_AGE_SLOTS]--;
	cn_l3e_flow_del(cn_l3e, entry->hash_idx, entry->crc16);
	rhashtable_remove_fast(&cn_flow_table, &entry->node,
			       cn_flow_ht_params);
	kfree(entry);
	atomic_dec(&cn_flow_installed);
	pr_debug("cn_flow_destroy: removed idx (flows=%d)\n",
		 atomic_read(&cn_flow_installed));
	return 0;
}

/*
 * ★ BUG-B: tear down EVERY installed offloaded flow.  Called (under
 * cn_flow_offload_mutex) when the live PPPoE session id CHANGES: all US flows
 * share the single L3-IF[1] entry, so a stale flow would emit the wrong/absent
 * session header once L3-IF[1] is reprogrammed.  nf_flow_table reinstalls the
 * still-live conntracks against the new sid on their next packet.  Iterates the
 * entry_by_idx reverse map (not the rhashtable) to remove-and-free safely
 * without an rhashtable-walk use-after-free.
 */
static void cn_l3e_flush_auto_flows(struct cn_l3e *l3e)
{
	u32 idx;
	int n = 0;

	if (!cn_flow_table_ready)
		return;
	for (idx = 0; idx < CN_L3E_ENTRIES; idx++) {
		struct cn_flow_entry *e = l3e->entry_by_idx[idx];

		if (!e)
			continue;
		l3e->entry_by_idx[idx] = NULL;
		l3e->bucket_occ[idx / CN_L3E_AGE_SLOTS]--;
		cn_l3e_flow_del(l3e, idx, e->crc16);
		rhashtable_remove_fast(&cn_flow_table, &e->node,
				       cn_flow_ht_params);
		kfree(e);
		atomic_dec(&cn_flow_installed);
		n++;
	}
	if (n)
		pr_info("cortina-l3fe: flushed %d offloaded flow(s) on PPPoE sid change\n",
			n);
}

static int cn_flow_stats(struct flow_cls_offload *f)
{
	struct cn_flow_entry *entry;

	entry = rhashtable_lookup_fast(&cn_flow_table, &f->cookie,
				       cn_flow_ht_params);
	if (!entry)
		return -ENOENT;

	/* No per-flow byte/pkt counters in the engine (AQM MIB meters only
	 * 2048 flows); report LIVENESS, fed by the batch bucket sweep -
	 * zero MMIO here, so 10k+ concurrent STATS queries stay free. */
	f->stats.lastused = entry->last_hit;
	return 0;
}

static int cn_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				void *cb_priv)
{
	struct flow_cls_offload *f = type_data;
	int err;

	if (type != TC_SETUP_CLSFLOWER)
		return -EOPNOTSUPP;

	mutex_lock(&cn_flow_offload_mutex);
	switch (f->command) {
	case FLOW_CLS_REPLACE:
		err = cn_flow_replace(f, cb_priv);
		break;
	case FLOW_CLS_DESTROY:
		err = cn_flow_destroy(f);
		break;
	case FLOW_CLS_STATS:
		err = cn_flow_stats(f);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}
	mutex_unlock(&cn_flow_offload_mutex);
	return err;
}

static LIST_HEAD(cn_block_cb_list);

static int cn_setup_tc_block(struct net_device *dev,
			     struct flow_block_offload *f)
{
	struct flow_block_cb *block_cb;
	flow_setup_cb_t *cb = cn_setup_tc_block_cb;

	if (f->binder_type != FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		return -EOPNOTSUPP;

	f->driver_block_list = &cn_block_cb_list;

	switch (f->command) {
	case FLOW_BLOCK_BIND:
		block_cb = flow_block_cb_lookup(f->block, cb, dev);
		if (block_cb) {
			flow_block_cb_incref(block_cb);
			return 0;
		}
		block_cb = flow_block_cb_alloc(cb, dev, dev, NULL);
		if (IS_ERR(block_cb))
			return PTR_ERR(block_cb);
		flow_block_cb_incref(block_cb);
		flow_block_cb_add(block_cb, f);
		list_add_tail(&block_cb->driver_list, &cn_block_cb_list);
		return 0;
	case FLOW_BLOCK_UNBIND:
		block_cb = flow_block_cb_lookup(f->block, cb, dev);
		if (!block_cb)
			return -ENOENT;
		if (!flow_block_cb_decref(block_cb)) {
			flow_block_cb_remove(block_cb, f);
			list_del(&block_cb->driver_list);
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

/* ndo_setup_tc hook for the cortina-ni netdevs (eth0 / gpon0) */
int cortina_ni_setup_tc(struct net_device *dev, enum tc_setup_type type,
			void *type_data)
{
	switch (type) {
	case TC_SETUP_BLOCK:
	case TC_SETUP_FT:
		return cn_setup_tc_block(dev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(cortina_ni_setup_tc);

/* ------------------------------------------------------------------ */
/* engine bring-up - called from the cortina-ni probe (wiring phase);  */
/* implements design doc section 2.6.  Until called, cn_l3e == NULL    */
/* and every offload request is refused (sw fastpath keeps working).   */
/* ------------------------------------------------------------------ */

static void cn_l3e_free_shadow(struct cn_l3e *l3e)
{
	kvfree(l3e->shadow_crc32);
	kvfree(l3e->shadow_crc16);
	kvfree(l3e->entry_by_idx);
	kvfree(l3e->bucket_occ);
	l3e->shadow_crc32 = NULL;
	l3e->shadow_crc16 = NULL;
	l3e->entry_by_idx = NULL;
	l3e->bucket_occ = NULL;
}

static int cn_l3e_init(struct cn_l3e *l3e)
{
	struct cn_l3e_tables t = {
		.key_virt	= l3e->key_tbl,
		.key_pa		= l3e->key_tbl_pa,
		.fib_virt	= l3e->fib_tbl,
		.fib_pa		= l3e->fib_tbl_pa,
	};
	int ret;

	/* lean SW shadow + sweep reverse map (~0.9 MB total) */
	l3e->shadow_crc32 = kvcalloc(CN_L3E_ENTRIES, sizeof(u32), GFP_KERNEL);
	l3e->shadow_crc16 = kvcalloc(CN_L3E_ENTRIES, sizeof(u16), GFP_KERNEL);
	l3e->entry_by_idx = kvcalloc(CN_L3E_ENTRIES,
				     sizeof(struct cn_flow_entry *),
				     GFP_KERNEL);
	l3e->bucket_occ = kvcalloc(CN_L3E_AGE_ROWS, sizeof(u8), GFP_KERNEL);
	if (!l3e->shadow_crc32 || !l3e->shadow_crc16 || !l3e->entry_by_idx ||
	    !l3e->bucket_occ) {
		ret = -ENOMEM;
		goto free;
	}

	/* the ordered engine arm (MEM_INI self-zero -> carve zero -> base
	 * regs -> geometry -> anti-wedge patch -> punt defaults ->
	 * granularity 0), all stock-mirrored - cortina-l3fe.c */
	ret = cortina_l3fe_engine_init(l3e->ne_base, &t);
	if (ret)
		goto free;

	/* stock profile/tuple/mask classify config so the engine parses/keys
	 * like stock (tier-1 captured).  Non-fatal + runtime-verified no
	 * datapath regression; but NOT yet sufficient for a HW hit - routed
	 * packets are still software-forwarded and do not consult the L3FE
	 * (see cortina_l3fe_classify_setup), so install stays gated OFF. */
	ret = cortina_l3fe_classify_setup(l3e->ne_base);
	if (ret)
		dev_warn(l3e->dev,
			 "l3fe: classify_setup timed out (%d) - hash lookup not configured\n",
			 ret);

	/* ★ Divergence B+C (gated OFF by default): steer routed frames into
	 * the HW L3-forwarding lookup with a hash-MISS trap-to-CPU, and open
	 * the transit-frame ingress admission (PDPID[0x18] -> L3FE WAN port +
	 * the my-MAC FIELD-CAM commit).  Non-fatal on timeout - a failed
	 * enable just leaves the software datapath as-is. */
	if (hw_l3_fwd) {
		ret = cortina_l3fe_hw_l3_forward_enable(l3e->ne_base,
							l3e->router_mac_valid ?
							l3e->router_mac : NULL);
		dev_info(l3e->dev,
			 "l3fe: HW L3-forwarding %s (miss->CPU, mac-cam %s, 5-tuple mask %d)\n",
			 ret ? "enable FAILED" : "ENABLED",
			 l3e->router_mac_valid ? "committed" : "SKIPPED (no netdev MAC)",
			 CN_L3E_WAN_MASK_ID);
		/* P3: with the engine armed, the routed profiles pointed at the
		 * 5-tuple mask, and the CLS admission stamping t2_ctrl (on the
		 * link-up cls_init re-run), flows may now be installed for a HW
		 * hit.  Only ungate under the gate + a successful enable. */
		if (!ret)
			cn_l3e_install_ok = true;
	}

	cn_l3e = l3e;
	return 0;
free:
	cn_l3e_free_shadow(l3e);
	return ret;
}

static int cn_flowoffload_init(void)
{
	int ret;

	ret = rhashtable_init(&cn_flow_table, &cn_flow_ht_params);
	cn_flow_table_ready = !ret;
	if (!ret)
		schedule_delayed_work(&cn_l3e_sweep,
				      msecs_to_jiffies(CN_L3E_SWEEP_MS));
	return ret;
}

/* ------------------------------------------------------------------ */
/* HS_SWO HW-CRC selftest - the phase-1 gate proof that the on-chip    */
/* CRC engine works and follows known algebra.                         */
/*                                                                     */
/* ★ Live finding (single-bit SWO probes, 2026-07-18): the engine does */
/* NOT CRC the raw HDR_I bytes.  It first derives the profile-SELECTED */
/* hash tuple (under the phase-1 default-zero profile config + an      */
/* all-ones mask only a 72-bit key window at bits 203-210/233-264/     */
/* 361-392 participates, plus HW-DERIVED flag bits such as zero/equal  */
/* checks - nonlinear in the key), then runs textbook CRC cores over   */
/* it: CRC-32 poly 0x04C11DB7 and CRC-16 poly 0x1021 (both extracted   */
/* from the adjacent-bit delta relation, 31/31 consistent).  A raw-key */
/* SW CRC therefore CANNOT reproduce the values; the real SW hash is   */
/* derived at P3 key-packing time against the STOCK profile/tuple/mask */
/* config, verified against this same SWO oracle.                      */
/*                                                                     */
/* What is asserted here, all from live hardware, no reference values: */
/*   1. determinism  - same key twice -> identical {crc32, crc16}      */
/*   2. window live  - a single key bit changes both CRCs              */
/*   3. linearity    - crc(A^B) == crc(0) ^ dA ^ dB over the window    */
/*   4. CRC algebra  - adjacent-bit deltas step by x mod 0x04C11DB7    */
/*                     (CRC-32) and x mod 0x1021 (CRC-16)              */
/* ------------------------------------------------------------------ */

#define CN_L3E_SWO_POLY32	0x04C11DB7u
#define CN_L3E_SWO_POLY16	0x1021u
#define CN_L3E_SWO_BIT0		240	/* inside the selected key window */
#define CN_L3E_SWO_NBITS	8
/* the selftest needs an all-ones mask; use a spare mask-table index so it
 * never clobbers the real classify masks 0-7 (cortina_l3fe_classify_setup) */
#define CN_L3E_SELFTEST_MASK	63

static u32 cn_l3e_poly32_step(u32 d)
{
	return (d << 1) ^ ((d & BIT(31)) ? CN_L3E_SWO_POLY32 : 0);
}

static u16 cn_l3e_poly16_step(u16 d)
{
	return ((d << 1) ^ ((d & BIT(15)) ? CN_L3E_SWO_POLY16 : 0)) & 0xffff;
}

static int cn_l3e_swo_key(struct cn_l3e *l3e, const u32 *w, u32 *c32, u16 *c16)
{
	return cortina_l3fe_swo_crc(l3e->ne_base, w, CN_L3E_KEY_BYTES / 4,
				    CN_L3E_SELFTEST_MASK, c32, c16);
}

static void cn_l3e_swo_selftest(struct cn_l3e *l3e)
{
	static const u32 ones[4] = { ~0u, ~0u, ~0u, ~0u };
	u32 w[CN_L3E_KEY_BYTES / 4];
	u8 *kb = (u8 *)w;
	u32 z32, r32, ab32, d32[CN_L3E_SWO_NBITS];
	u16 z16, r16, ab16, d16[CN_L3E_SWO_NBITS];
	bool ok = true;
	int i, bit, ret;

	l3e->selftest_ret = cortina_l3fe_mask_write(l3e->ne_base,
						    CN_L3E_SELFTEST_MASK,
						    ones, ones);
	if (l3e->selftest_ret) {
		pr_warn("cortina-l3fe: selftest mask write failed (%d)\n",
			l3e->selftest_ret);
		return;
	}

#define SWO_RUN(c32p, c16p) do {					\
	ret = cn_l3e_swo_key(l3e, w, (c32p), (c16p));			\
	if (ret) {							\
		l3e->selftest_ret = ret;				\
		pr_warn("cortina-l3fe: SWO engine timeout (%d)\n", ret); \
		return;							\
	}								\
} while (0)

	/* 1. determinism on the all-zero key */
	memset(w, 0, sizeof(w));
	SWO_RUN(&z32, &z16);
	memset(w, 0, sizeof(w));
	SWO_RUN(&r32, &r16);
	if (r32 != z32 || r16 != z16) {
		pr_warn("cortina-l3fe: SWO not deterministic: %08x/%04x vs %08x/%04x\n",
			z32, z16, r32, r16);
		ok = false;
	}

	/* single-bit deltas over consecutive window bits */
	for (i = 0; i < CN_L3E_SWO_NBITS; i++) {
		bit = CN_L3E_SWO_BIT0 + i;
		memset(w, 0, sizeof(w));
		kb[bit >> 3] = 1u << (bit & 7);
		SWO_RUN(&r32, &r16);
		d32[i] = r32 ^ z32;
		d16[i] = r16 ^ z16;
		/* 2. window live */
		if (!d32[i] || !d16[i]) {
			pr_warn("cortina-l3fe: SWO key bit %d has no effect\n",
				bit);
			ok = false;
		}
	}

	/* 3. linearity: crc(bit0 + bit1) == z ^ d0 ^ d1 */
	memset(w, 0, sizeof(w));
	kb[CN_L3E_SWO_BIT0 >> 3] = 3u << (CN_L3E_SWO_BIT0 & 7);
	SWO_RUN(&ab32, &ab16);
	if (ab32 != (z32 ^ d32[0] ^ d32[1]) ||
	    ab16 != (z16 ^ d16[0] ^ d16[1])) {
		pr_warn("cortina-l3fe: SWO linearity fail: %08x/%04x want %08x/%04x\n",
			ab32, ab16, z32 ^ d32[0] ^ d32[1],
			z16 ^ d16[0] ^ d16[1]);
		ok = false;
	}

	/* 4. the CRC polynomial algebra across adjacent bits */
	for (i = 0; i + 1 < CN_L3E_SWO_NBITS; i++) {
		if (d32[i + 1] != cn_l3e_poly32_step(d32[i])) {
			pr_warn("cortina-l3fe: SWO crc32 poly fail at bit %d: %08x -> %08x\n",
				CN_L3E_SWO_BIT0 + i, d32[i], d32[i + 1]);
			ok = false;
		}
		if (d16[i + 1] != cn_l3e_poly16_step(d16[i])) {
			pr_warn("cortina-l3fe: SWO crc16 poly fail at bit %d: %04x -> %04x\n",
				CN_L3E_SWO_BIT0 + i, d16[i], d16[i + 1]);
			ok = false;
		}
	}
#undef SWO_RUN

	if (ok)
		l3e->selftest_pass = 1;
	else
		l3e->selftest_fail = 1;
}

/* ------------------------------------------------------------------ */
/* HDR_I 5-tuple key-packing liveness (divergence-A gate proof).       */
/*                                                                     */
/* Builds a real IPv4 5-tuple through cn_l3e_build_hdri() + the SWO    */
/* under the 5-tuple mask (mask 0), then perturbs each field in turn   */
/* and requires the CRC to CHANGE.  Before the HDR_I fix the key went  */
/* to the engine in the 92-byte cn_l3e_key layout, so every IP field   */
/* landed in a masked-out position and the CRC was constant; this      */
/* asserts the fix on the real driver code path, on live HW.           */
/* ------------------------------------------------------------------ */
static void cn_l3e_hdri_live_test(struct cn_l3e *l3e)
{
	struct cn_l3e_key base = {
		.ip_vld = 1, .ip_ver = 0, .ip_protocol = 6,   /* TCP */
		.ip_sa_0 = 0x0a000001, .ip_da_0 = 0x2de14b02,
		.l4_sport = 12345, .l4_dport = 80,
	};
	struct cn_l3e_key k;
	u32 b32, r32;
	u16 b16, r16;
	bool ok = true;
	int ret, i;

	ret = cn_l3e_key_hash(l3e, &base, CN_L3E_PROFILE_WAN,
			      CN_L3E_WAN_MASK_ID, &b32, &b16);
	if (ret) {
		pr_warn("cortina-l3fe: HDR_I liveness: SWO timeout (%d)\n", ret);
		l3e->hdri_live_fail = 1;
		return;
	}

#define HDRI_PERTURB(desc, field, newval) do {				\
	k = base;							\
	k.field = (newval);						\
	ret = cn_l3e_key_hash(l3e, &k, CN_L3E_PROFILE_WAN,		\
			      CN_L3E_WAN_MASK_ID, &r32, &r16);		\
	if (ret) { l3e->hdri_live_fail = 1; return; }			\
	if (r32 == b32 && r16 == b16) {					\
		pr_warn("cortina-l3fe: HDR_I liveness: %s did NOT move the CRC (masked-out)\n", \
			desc);						\
		ok = false;						\
	}								\
} while (0)

	HDRI_PERTURB("dport", l4_dport, 443);
	HDRI_PERTURB("sport", l4_sport, 22);
	HDRI_PERTURB("daddr", ip_da_0, 0x2de14b09);
	HDRI_PERTURB("saddr", ip_sa_0, 0x0a000063);
	HDRI_PERTURB("proto", ip_protocol, 17);
	HDRI_PERTURB("daddr-low-byte", ip_da_0, 0x2de14bff);
	HDRI_PERTURB("saddr-low-byte", ip_sa_0, 0x0a0000ff);
#undef HDRI_PERTURB

	/* determinism: same tuple twice -> identical CRC */
	for (i = 0; i < 2; i++) {
		ret = cn_l3e_key_hash(l3e, &base, CN_L3E_PROFILE_WAN,
				      CN_L3E_WAN_MASK_ID, &r32, &r16);
		if (ret || r32 != b32 || r16 != b16) {
			pr_warn("cortina-l3fe: HDR_I liveness: non-deterministic\n");
			ok = false;
		}
	}

	if (ok)
		l3e->hdri_live_pass = 1;
	else
		l3e->hdri_live_fail = 1;
}

/* ------------------------------------------------------------------ */
/* /proc/cortina_l3fe - manual flow install/read/delete for the P3 HW  */
/* HIT proof.  Installs a 5-tuple entry through the DRIVER's COHERENT   */
/* dma_alloc_coherent mapping (l3e->key_tbl / l3e->fib_tbl) - unlike a  */
/* /dev/mem cached alias, the engine's AXI master reads exactly what    */
/* the CPU wrote (no mismatched-attributes hazard).  Read reports the   */
/* live age (re-arm = HW hit) + HS_CACHE_CNT (climbs on hits).  This is */
/* the deterministic proof vehicle, independent of nf_flow_table.       */
/*   echo 'install <sa> <da> <sport> <dport> <proto> <profile>          */
/*         [mcgid] [new_sa] [new_sport]' > /proc/cortina_l3fe           */
/*   echo 'read'  > ...   (then cat)                                     */
/*   echo 'del'   > ...                                                  */
/* addresses dotted or hex; ports/proto/profile decimal or hex.         */
/* ------------------------------------------------------------------ */
#define CN_L3E_PROC_MAX_MANUAL	8
struct cn_l3e_manual {
	u32	idx;
	u16	crc16;
	bool	valid;
	/* echo of the installed key for the readout */
	u32	sa, da;
	u16	sp, dp;
	u8	proto, profile;
};
static struct cn_l3e_manual cn_l3e_manual[CN_L3E_PROC_MAX_MANUAL];

static u32 cn_l3e_proc_parse_ip(const char *s)
{
	u8 b[4];
	unsigned int v;

	if (sscanf(s, "%hhu.%hhu.%hhu.%hhu", &b[0], &b[1], &b[2], &b[3]) == 4)
		return ((u32)b[0] << 24) | ((u32)b[1] << 16) |
		       ((u32)b[2] << 8) | b[3];
	if (kstrtouint(s, 0, &v) == 0)
		return v;
	return 0;
}

static int cn_l3e_proc_show(struct seq_file *m, void *v)
{
	struct cn_l3e *l3e = cn_l3e;
	u32 cache_cnt;
	int i;

	if (!l3e) {
		seq_puts(m, "l3fe: engine not armed (cn_l3e == NULL)\n");
		return 0;
	}

	mutex_lock(&cn_flow_offload_mutex);
	cache_cnt = readl(l3e->ne_base + CN_L3E_HS_CACHE_CNT);
	seq_printf(m,
		   "install_ok=%d auto_flows=%d HS_CACHE_CNT(0x38c0)=%u live_pon{gem=%u tcont=%u} pppoe_sess=%#x gran(0x3924)=0x%08x\n",
		   cn_l3e_install_ok, atomic_read(&cn_flow_installed),
		   cache_cnt,
		   READ_ONCE(l3e->data_gem), READ_ONCE(l3e->data_tcont),
		   READ_ONCE(l3e->data_pppoe_session),
		   readl(l3e->ne_base + CN_L3E_HS_AGING_GRANULARITY));
	seq_puts(m, "usage: echo 'install <sa> <da> <sp> <dp> <proto> <profile> [mcgid] [new_sa] [new_sp]' > /proc/cortina_l3fe\n");
	seq_puts(m, "       echo 'pppoe <session_id>' (0 = clear/IPoE) > /proc/cortina_l3fe\n");
	for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++) {
		struct cn_l3e_manual *e = &cn_l3e_manual[i];
		u32 age = 0, key = 0, fib0 = 0;

		if (!e->valid)
			continue;
		cn_l3e_age_get(l3e, e->idx, &age);
		key = l3e->key_tbl[e->idx];
		fib0 = *(u32 *)(l3e->fib_tbl + (size_t)e->idx * CN_L3E_FIB_BYTES);
		seq_printf(m,
			   "[%d] idx=%u crc16=%04x prof=%u  %pI4h:%u -> %pI4h:%u proto=%u  key_tbl=%08x fib0=%08x age=%u %s\n",
			   i, e->idx, e->crc16, e->profile,
			   &e->sa, e->sp, &e->da, e->dp, e->proto,
			   key, fib0, age,
			   age >= CN_L3E_AGE_START ? "*** HW HIT (age re-armed 1->2) ***" :
			   age == CN_L3E_AGE_IDLE ? "(live @IDLE, no hit yet)" : "(free)");
	}
	mutex_unlock(&cn_flow_offload_mutex);
	return 0;
}

static int cn_l3e_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cn_l3e_proc_show, NULL);
}

static ssize_t cn_l3e_proc_write(struct file *file, const char __user *ubuf,
				 size_t len, loff_t *ppos)
{
	struct cn_l3e *l3e = cn_l3e;
	char buf[160], cmd[16] = {};
	char sas[40], das[40], nsas[40] = {};
	unsigned int sp, dp, proto, profile, mcgid = 0, nsp = 0;
	int n, i, err;

	if (!l3e)
		return -ENODEV;
	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;

	if (sscanf(buf, "%15s", cmd) != 1)
		return -EINVAL;

	mutex_lock(&cn_flow_offload_mutex);

	if (!strcmp(cmd, "del")) {
		for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++) {
			if (cn_l3e_manual[i].valid) {
				cn_l3e_flow_del(l3e, cn_l3e_manual[i].idx,
						cn_l3e_manual[i].crc16);
				cn_l3e_manual[i].valid = false;
			}
		}
		err = 0;
		goto out;
	}
	if (!strcmp(cmd, "read")) {
		err = 0;	/* the readout is `cat` (show) */
		goto out;
	}
	if (!strcmp(cmd, "pppoe")) {
		/* first-bring-up path for the live session id (dec or 0x hex);
		 * 0 = clear back to IPoE */
		int sess;

		if (sscanf(buf, "%*s %i", &sess) != 1 ||
		    sess < 0 || sess > 0xffff) {
			err = -EINVAL;
			goto out;
		}
		err = cortina_ni_wan_pppoe_session_set(sess);
		goto out;
	}

	if (strcmp(cmd, "install")) {
		err = -EINVAL;
		goto out;
	}

	n = sscanf(buf, "%*s %39s %39s %u %u %u %u %u %39s %u",
		   sas, das, &sp, &dp, &proto, &profile, &mcgid, nsas, &nsp);
	if (n < 6) {
		err = -EINVAL;
		goto out;
	}

	for (i = 0; i < CN_L3E_PROC_MAX_MANUAL; i++)
		if (!cn_l3e_manual[i].valid)
			break;
	if (i == CN_L3E_PROC_MAX_MANUAL) {
		err = -ENOSPC;
		goto out;
	}

	{
		struct cn_l3e_key key = {};
		struct cn_l3e_act act = {};
		u32 mask_id = (profile == CN_L3E_PROFILE_LAN) ?
			      CN_L3E_LAN_MASK_ID : CN_L3E_WAN_MASK_ID;
		struct cn_l3e_manual *e = &cn_l3e_manual[i];

		key.ip_sa_0 = cn_l3e_proc_parse_ip(sas);
		key.ip_da_0 = cn_l3e_proc_parse_ip(das);
		key.l4_sport = sp;
		key.l4_dport = dp;
		key.ip_protocol = proto;
		key.ip_ver = 0;
		key.ip_vld = 1;

		/* Forward action.  With mcgid==0 (default) and a live PON data
		 * path armed, stamp the US WAN-forward egress (mc=1,
		 * mcgid=live gem, t2_ctrl=tcont) so a HIT is OBSERVABLE as a
		 * real forward (the primary witness: CPU/SW-forward counter
		 * goes flat while the far end still receives).  An explicit
		 * mcgid arg overrides (e.g. mcgid=0x10 = CPU_0 for an
		 * age-only, non-forwarding hit probe).  Plus optional inline
		 * SNAT of the SA (shipping normal-mode FIB carries the NAT
		 * address inline, no aux table). */
		if (mcgid == 0 && cn_l3e_set_us_egress(l3e, &act, 0) == 0) {
			act.ip_ttl_dec = 1;
		} else {
			act.permit = 1;
			act.dpid_vld = 1;
			act.dpid_pri = 1;
			act.deepq = 1;
			act.ip_ttl_dec = 1;
			act.mcgid = mcgid & 0x3ff;
		}
		if (n >= 8 && nsas[0]) {
			act.ip_addr_vld = 1;
			act.ip_type = 0;	/* rewrite SA (SNAT) */
			act.ip_addr = cn_l3e_proc_parse_ip(nsas);
		}
		if (n >= 9 && nsp) {
			act.l4_port = nsp;
		}

		err = cn_l3e_flow_add(l3e, &key, &act, profile, mask_id,
				      &e->idx, &e->crc16);
		if (!err) {
			/* ★ HIT WITNESS: cn_l3e_flow_add armed the entry at
			 * START(2); the liveness sweep SKIPS manual entries and
			 * HW auto-age is OFF, so the age would sit at 2 forever
			 * regardless of traffic (ambiguous).  Re-arm it DOWN to
			 * IDLE(1): on a HW T2 HIT the lookup engine re-arms the
			 * slot back UP to START(2), so a subsequent read showing
			 * age >= 2 is an UNAMBIGUOUS proof the entry matched a
			 * frame in silicon. */
			cn_l3e_age_set(l3e, e->idx, CN_L3E_AGE_IDLE);
			e->sa = key.ip_sa_0;
			e->da = key.ip_da_0;
			e->sp = sp;
			e->dp = dp;
			e->proto = proto;
			e->profile = profile;
			e->valid = true;
			pr_info("cortina-l3fe: manual install idx=%u crc16=%04x prof=%u mask=%u age=IDLE(1) (coherent key_tbl write; re-arm to 2 = HW hit)\n",
				e->idx, e->crc16, profile, mask_id);
		}
	}
out:
	mutex_unlock(&cn_flow_offload_mutex);
	return err ? err : len;
}

static const struct proc_ops cn_l3e_proc_ops = {
	.proc_open	= cn_l3e_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= cn_l3e_proc_write,
};

/* ------------------------------------------------------------------ */
/* probe entry (called once from the cortina-ni platform probe).  Any  */
/* failure leaves cn_l3e == NULL: every offload request is refused and */
/* the normal software datapath is untouched.                          */
/* ------------------------------------------------------------------ */

int cortina_ni_flowoffload_probe(struct cortina_ni *ni)
{
	void __iomem *ne = ni->win[CA_NI_WIN_NI];
	struct cn_l3e *l3e;
	int ret;

	if (!ne)
		return -ENODEV;

	l3e = devm_kzalloc(ni->dev, sizeof(*l3e), GFP_KERNEL);
	if (!l3e)
		return -ENOMEM;
	l3e->dev = ni->dev;
	l3e->ne_base = ne;
	spin_lock_init(&l3e->reg_lock);

	/* router MAC for the my-MAC FIELD-CAM commit (same source + fallback
	 * as the RX steer init); WAN MAC is derived as base+1 in the enable */
	if (ni->tx && ni->tx->netdev) {
		ether_addr_copy(l3e->router_mac, ni->tx->netdev->dev_addr);
		l3e->router_mac_valid = true;
	}

	/* one coherent carve: key table then FIB (stock places the FIB at
	 * key base + 0x40000 the same way) */
	l3e->carve = dma_alloc_coherent(ni->dev, CN_L3E_CARVE_BYTES,
					&l3e->carve_pa, GFP_KERNEL);
	if (!l3e->carve)
		return -ENOMEM;
	l3e->key_tbl = l3e->carve;
	l3e->key_tbl_pa = l3e->carve_pa;
	l3e->fib_tbl = l3e->carve + CN_L3E_KEY_TBL_BYTES;
	l3e->fib_tbl_pa = l3e->carve_pa + CN_L3E_KEY_TBL_BYTES;

	/* spy-first: the engine must be un-armed at this point (boot ROM /
	 * U-Boot never touch it; live-verified all-zero pre-init) */
	dev_info(ni->dev,
		 "l3fe: pre-arm MH0=%08x MA0=%08x INI=%08x (expect all 0)\n",
		 readl(ne + CN_L3E_HS_BA_MH0), readl(ne + CN_L3E_HS_BA_MA0),
		 readl(ne + CN_L3E_HS_HASH_INI));

	ret = cn_l3e_init(l3e);
	if (ret)
		goto err_free_carve;

	cn_l3e_swo_selftest(l3e);
	cn_l3e_hdri_live_test(l3e);

	ret = cn_flowoffload_init();
	if (ret) {
		cn_l3e = NULL;
		cn_l3e_free_shadow(l3e);
		goto err_free_carve;
	}

	/* P3 manual-install / HW-HIT proof (coherent carve write) */
	proc_create_data("cortina_l3fe", 0644, NULL, &cn_l3e_proc_ops, NULL);

	/* phase-1 gate evidence: read back everything the arm wrote */
	dev_info(ni->dev,
		 "l3fe: armed carve pa=%pad MH0=%08x MH1=%08x MA0=%08x MA1=%08x INI=%08x MEMINI=%08x RSV0=%08x RSV1=%08x AXIM2=%08x CHKFAIL=%08x GRAN=%08x AQM=%08x\n",
		 &l3e->carve_pa,
		 readl(ne + 0x383c), readl(ne + 0x3838),
		 readl(ne + 0x3844), readl(ne + 0x3840),
		 readl(ne + 0x3834), readl(ne + 0x393c),
		 readl(ne + 0x3944), readl(ne + 0x3948),
		 readl(ne + 0x3c80), readl(ne + 0x3940),
		 readl(ne + 0x3924), readl(ne + 0x3aa8));
	dev_info(ni->dev,
		 "l3fe: SWO CRC selftest %s (pass=%u fail=%u ret=%d)\n",
		 (!l3e->selftest_ret && l3e->selftest_fail == 0 &&
		  l3e->selftest_pass) ? "PASS" : "FAIL",
		 l3e->selftest_pass, l3e->selftest_fail, l3e->selftest_ret);
	dev_info(ni->dev,
		 "l3fe: HDR_I 5-tuple key-packing %s (pass=%u fail=%u)\n",
		 (l3e->hdri_live_pass && !l3e->hdri_live_fail) ? "LIVE" : "FAIL",
		 l3e->hdri_live_pass, l3e->hdri_live_fail);
	return 0;

err_free_carve:
	dma_free_coherent(ni->dev, CN_L3E_CARVE_BYTES, l3e->carve,
			  l3e->carve_pa);
	return ret;
}
