// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * omci_responder.c — clean-room ONU G.988 OMCI baseline responder + ME model
 * (RTL9607F "Elnath", Stage C).  See omci_responder.h for the wire-format
 * provenance: the layout + ME attribute content mirror the responder proven
 * end-to-end (Online/normal + WAN) against the same HSGQ-G008 OLT on the
 * RTL9602C.  The ONU identity is NOT part of this file: the 8-byte serial
 * number is handed to omci_onu_init() by the caller, which reads it from the
 * board (cortina-gpon.c, the cg_sn_* provisioning path).
 *
 * Pure functional core: no HW I/O, no locking (the caller serializes), no
 * allocation — everything lives in the caller-provided struct omci_onu.
 *
 * The ME model is TABLE-DRIVEN: one descriptor row per (class, attribute)
 * carrying {attribute number, wire size, value source} and ONE generic filler
 * walking them.  That is what makes the three cross-vendor invariants
 * structural rather than per-case:
 *   - every reply is bounded by the 25-octet Get value area,
 *   - the set of attributes an ME KNOWS is derivable, so a Get can name the
 *     ones it does not support (optional-attribute mask) and the ones that did
 *     not fit (attribute-execution mask) instead of silently answering
 *     "success" with a short mask — the OLT-re-GETs-forever churn class,
 *   - a class the model does not carry gets ONE policy per class range, not a
 *     hard-coded list of the class IDs one OLT happened to ask for.
 * Byte-for-byte equivalence of the table walk with the hand-written filler it
 * replaced is pinned exhaustively (all 65536 attribute masks x every modelled
 * class/instance) by dev/rtl9607c-test/omci_me_table_test, and the G.988
 * cross-vendor behaviour by dev/rtl9607c-test/omci_conformance_test.
 */
#include <linux/crc32.h>
#include <linux/string.h>

#include "omci_responder.h"

/* Message types (G.988 Table 11.2.2-1). */
#define OMCI_MT_CREATE		0x04
#define OMCI_MT_DELETE		0x06
#define OMCI_MT_SET		0x08
#define OMCI_MT_GET		0x09
#define OMCI_MT_GET_ALL_ALARMS	0x0b
#define OMCI_MT_GET_ALL_ALRM_NX	0x0c
#define OMCI_MT_MIB_UPLOAD	0x0d
#define OMCI_MT_MIB_UPLOAD_NX	0x0e
#define OMCI_MT_MIB_RESET	0x0f
#define OMCI_MT_ALARM		0x10	/* 16 — ONU-autonomous alarm.  NOT Get
					 * Next: an OLT never sends 16, which is
					 * why mislabelling Get Next 0x10 stayed
					 * invisible on this OLT for so long. */
#define OMCI_MT_AVC		0x11	/* 17 — ONU-autonomous notification */
#define OMCI_MT_TEST		0x12
#define OMCI_MT_START_SW_DL	0x13
#define OMCI_MT_DOWNLOAD_SEC	0x14
#define OMCI_MT_END_SW_DL	0x15
#define OMCI_MT_ACTIVATE_SW	0x16
#define OMCI_MT_COMMIT_SW	0x17
#define OMCI_MT_SYNC_TIME	0x18
#define OMCI_MT_REBOOT		0x19
#define OMCI_MT_GET_NEXT	0x1a	/* 26 */

/* Result codes (G.988 Table 11.2.2-2): the assigned set is the dense 0..7
 * plus 9 — 8 and anything above 9 is unassigned and must never be sent. */
#define OMCI_RC_OK		0x00
#define OMCI_RC_NOT_SUPPORTED	0x02	/* "command not supported".  Kept for
					 * the record and deliberately NOT
					 * emitted: stock answers an action a ME
					 * does not implement with 0x00 + empty
					 * contents, and 0x02 has been observed
					 * to abort a foreign OLT's config load
					 * where an empty OK does not. */
#define OMCI_RC_PARAM_ERROR	0x03
#define OMCI_RC_UNKNOWN_ME	0x04
#define OMCI_RC_UNKNOWN_INST	0x05
#define OMCI_RC_INST_EXISTS	0x07
#define OMCI_RC_ATTR_FAILED	0x09

/* Attribute-mask bit: attr #n is bit (16-n), so bit15 = attr 1. */
#define OMCI_ATTR_BIT(n)	(1u << (16 - (n)))

/* ME class IDs presented in the MIB upload (G.988 + the HSGQ OLT's set). */
#define OMCI_ME_ONU_DATA	2
#define OMCI_ME_CARDHOLDER	5
#define OMCI_ME_CIRCUIT_PACK	6
#define OMCI_ME_SW_IMAGE	7
#define OMCI_ME_PPTP_ETH_UNI	11	/* THE HGU gate: the OLT's
					 * gpon_ont_sync_capability counts these */
#define OMCI_ME_OLT_G		131
#define OMCI_ME_ONU_G		256
#define OMCI_ME_ONU2_G		257
#define OMCI_ME_TCONT		262
#define OMCI_ME_ANI_G		263
#define OMCI_ME_UNI_G		264
#define OMCI_ME_PRIORITY_QUEUE	277
#define OMCI_ME_TRAFFIC_SCHED	278
#define OMCI_ME_VEIP		329
#define OMCI_ME_CTC_LOID_AUTH	65530	/* 0xFFFA — CTC extension the OLT audits */

static inline void omci_put_be16(u8 *p, u16 v)
{
	p[0] = (u8)(v >> 8);
	p[1] = (u8)v;
}

/*
 * MIC (bytes 44..47) = the I.363.5 / AAL5 CRC-32 over bytes 0..43 (G.984.4
 * baseline trailer): NON-reflected polynomial 0x04C11DB7 MSB-first, init
 * all-ones, final complement — the kernel's crc32_be — stored big-endian.
 * LIVE-PROVEN on this OLT: the DS OMCI frames' MIC matches ~crc32_be(~0,
 * msg, 44) and NOT the reflected zlib crc32_le (the DS MIC self-check in
 * cortina-gpon.c logs which variant each received frame carries).  Computed
 * in SOFTWARE: the GPON MAC's own OMCI CRC engine stays enabled
 * (onu_cfg.omci_crc_dis = 0, the stock value) — if the HW also inserts, it
 * writes the same correct bytes.  A zero/wrong MIC = the OLT silently drops
 * every response and loops its GET audit (proven failure class).
 */
static void omci_set_mic(u8 *msg)
{
	u32 c = ~crc32_be(~0u, msg, 44);

	msg[44] = (u8)(c >> 24);
	msg[45] = (u8)(c >> 16);
	msg[46] = (u8)(c >> 8);
	msg[47] = (u8)c;
}

/* Stamp the baseline trailer (40..43 = 00 00 00 28) + MIC.  Call LAST. */
static void omci_finalize(u8 *msg)
{
	msg[40] = 0x00;
	msg[41] = 0x00;
	msg[42] = 0x00;
	msg[43] = 0x28;
	omci_set_mic(msg);
}

/* ---- dynamic (OLT-provisioned) ME store ---- */

static struct omci_me_inst *omci_store_find(struct omci_onu *o, u16 class_id,
					    u16 inst)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used && o->store[k].class_id == class_id &&
		    o->store[k].inst == inst)
			return &o->store[k];
	return NULL;
}

/* Does the ONU hold ANY instance of @class_id?  Used to separate "I do not
 * know that class at all" (0x04) from "I know the class, not that instance"
 * (0x05) on a Set the OLT sends for something it never created. */
static bool omci_store_has_class(struct omci_onu *o, u16 class_id)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used && o->store[k].class_id == class_id)
			return true;
	return false;
}

/* idx-th used entry in array order — the MIB-Upload tail rows. */
static struct omci_me_inst *omci_store_nth(struct omci_onu *o, u16 idx)
{
	u16 k, n = 0;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used) {
			if (n == idx)
				return &o->store[k];
			n++;
		}
	return NULL;
}

/* Insert one OLT-created instance.  Returns false when the store is FULL —
 * the caller must then NAK: a dropped Create answered OK keeps the ONU's
 * MIB-Data-Sync in lockstep with the OLT's lsync while the MIB diverged, so
 * the OLT's ME2 audit can never detect it.  NAKing freezes MDS instead, the
 * audit mismatches, and the OLT's own MIB-Reset wipes the store and
 * re-provisions from empty (the MDS-poison self-heal proven on this OLT). */
static bool omci_store_put(struct omci_onu *o, u16 class_id, u16 inst,
			   const u8 *body, int blen)
{
	struct omci_me_inst *e = NULL;
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (!o->store[k].used) {
			e = &o->store[k];
			break;
		}
	if (!e)
		return false;

	e->used = true;
	e->class_id = class_id;
	e->inst = inst;
	e->blen = 0;
	o->store_n++;
	if (body && blen > 0) {
		if (blen > (int)sizeof(e->body))
			blen = sizeof(e->body);
		memcpy(e->body, body, blen);
		e->blen = (u8)blen;
	}
	return true;
}

/*
 * Apply a Set's attribute values to a provisioned instance.  The store holds
 * the instance's attribute bytes as an OPAQUE blob (an OLT-created class has
 * no descriptor table, so there is no attribute -> offset map for it), so a
 * Set writes its values at the head of the blob exactly as a Create does.
 * That is best-effort by construction and it is what an audit GET replays;
 * dropping the Set instead — the previous behaviour — made a
 * Create-then-Set-then-audit OLT (the common provisioning order) read back its
 * own Create defaults and re-Set forever.
 */
static void omci_store_merge(struct omci_me_inst *e, const u8 *val, int vlen)
{
	if (vlen <= 0)
		return;
	if (vlen > (int)sizeof(e->body))
		vlen = sizeof(e->body);
	memcpy(e->body, val, vlen);
	if (e->blen < (u8)vlen)
		e->blen = (u8)vlen;
}

static void omci_store_del(struct omci_onu *o, u16 class_id, u16 inst)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);

	if (e) {
		e->used = false;
		if (o->store_n)
			o->store_n--;
	}
}

/*
 * ---- ME attribute model ----
 *
 * Constant attribute bytes live in one pool so a descriptor row can name them
 * with a 2-byte offset instead of a pointer (no relocation, no per-row
 * padding).  Every offset is verified byte-for-byte by Step 4d's exhaustive
 * GET-equivalence sweep, so a mis-typed offset fails on x86, not on a board.
 */
#define OMCI_B_VENDOR		0	/* "HSGQ"          (4)  */
#define OMCI_B_ONU_G_VER	4	/* "02A5B1"        (14) */
#define OMCI_B_SW_BANK0		18	/* "M225-260525"   (14) */
#define OMCI_B_SW_BANK1		32	/* "M225-260515"   (14) */
#define OMCI_B_EQUIP_ID		46	/* "HSGQ-X411AXF"  (20) */
#define OMCI_B_LOID		66	/* "user"          (24) */
#define OMCI_B_OPER_ID		90	/* "CTC"           (4)  */
#define OMCI_B_ZEROS		94	/* zeros           (16) */

static const u8 omci_blob[] = {
	/* [0] vendor ID — ONU-G #1, Circuit-Pack #5.  The OLT recognizes HSGQ
	 * ONUs; "XPON" was rejected. */
	'H', 'S', 'G', 'Q',
	/* [4] ONU-G #2 version, zero-padded to 14 */
	'0', '2', 'A', '5', 'B', '1', 0, 0, 0, 0, 0, 0, 0, 0,
	/* [18] SW-image bank 0 (active) version — also Circuit-Pack #4 */
	'M', '2', '2', '5', '-', '2', '6', '0', '5', '2', '5', 0, 0, 0,
	/* [32] SW-image bank 1 version */
	'M', '2', '2', '5', '-', '2', '6', '0', '5', '1', '5', 0, 0, 0,
	/* [46] ONU2-G #1 equipment ID, zero-padded to 20 */
	'H', 'S', 'G', 'Q', '-', 'X', '4', '1', '1', 'A', 'X', 'F',
	0, 0, 0, 0, 0, 0, 0, 0,
	/* [66] logical ONU ID — ONU-G #10, CTC LoID #2, zero-padded to 24 */
	'u', 's', 'e', 'r',
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* [90] CTC #1 operation ID, zero-padded to 4 */
	'C', 'T', 'C', 0,
	/* [94] all-zero source: SW-image #5 hash (16), ONU-G #11 logical
	 * password / CTC #3 password (12) */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
_Static_assert(sizeof(omci_blob) == 110, "omci_blob offsets and sizes drifted");

/* Where a descriptor row takes its value bytes from. */
enum omci_attr_src {
	OMCI_SRC_CONST,		/* v = the value, big-endian in `size` bytes */
	OMCI_SRC_BLOB,		/* v = byte offset into omci_blob[] */
	OMCI_SRC_SN,		/* the board serial number (8) */
	OMCI_SRC_MDS,		/* ME 2 #1 = the live MIB-Data-Sync */
	OMCI_SRC_DYN,		/* v = enum omci_attr_dyn */
};

/* The few attributes whose value is derived from the ME INSTANCE. */
enum omci_attr_dyn {
	OMCI_DYN_SW_VER,	/* ME 7 #1: per-bank version blob */
	OMCI_DYN_SW_FLAG,	/* ME 7 #2/#3: bank 0 is the active+committed */
	OMCI_DYN_TCONT_ALLOC,	/* ME 262 #1: alloc-ID of this T-CONT */
	OMCI_DYN_PQ_PORT,	/* ME 277 #6: related port, counts DOWN in the
				 * 8-queue block (queue 0 -> 7, 1 -> 6, ...) */
	OMCI_DYN_TS_TCONT,	/* ME 278 #1: T-CONT pointer == the instance */
	OMCI_DYN_ANIG_RX,	/* ME 263 #10: live RX optical level */
	OMCI_DYN_ANIG_TX,	/* ME 263 #14: live TX optical level */
};

/*
 * One modelled attribute.  Rows of the same class are CONTIGUOUS and in
 * EMISSION order (G.988 packs a Get response in ascending attribute order, and
 * the order is what an OLT decoder walks — a swap silently misaligns the rest
 * of the reply).  A class with no modelled attributes carries one marker row
 * (attr 0), which is how "ME known, nothing to serve" is expressed.
 */
struct omci_attr {
	u16	class_id;
	u16	v;
	u8	attr;
	u8	size;
	u8	src;
};

#define AT(cls, n, sz, s, arg)	{ (cls), (arg), (n), (sz), (s) }
#define A_C(cls, n, sz, val)	AT(cls, n, sz, OMCI_SRC_CONST, val)
#define A_B(cls, n, sz, off)	AT(cls, n, sz, OMCI_SRC_BLOB, off)
#define A_SN(cls, n)		AT(cls, n, 8, OMCI_SRC_SN, 0)
#define A_MDS(cls, n)		AT(cls, n, 1, OMCI_SRC_MDS, 0)
#define A_D(cls, n, sz, dyn)	AT(cls, n, sz, OMCI_SRC_DYN, dyn)
#define A_NO_ATTRS(cls)		AT(cls, 0, 0, OMCI_SRC_CONST, 0)

static const struct omci_attr omci_attrs[] = {
	/* ---- ME 2 ONU-Data (inst 0) ---- */
	A_MDS(2, 1),				/* #1  MIB-Data-Sync */

	/* ---- ME 256 ONU-G (inst 0).  ALL 14 attributes are servable: a
	 * missing one answers a short mask and the OLT re-GETs forever. ---- */
	A_B(256,  1,  4, OMCI_B_VENDOR),	/* #1  Vendor ID */
	A_B(256,  2, 14, OMCI_B_ONU_G_VER),	/* #2  Version */
	A_SN(256, 3),				/* #3  Serial number */
	A_C(256,  4,  1, 0x02),			/* #4  Traffic-mgmt option */
	A_C(256,  5,  1, 0x00),			/* #5  ATM CC option */
	A_C(256,  6,  1, 0x00),			/* #6  Battery backup */
	A_C(256,  7,  1, 0x00),			/* #7  Admin state */
	A_C(256,  8,  1, 0x00),			/* #8  Op state */
	A_C(256,  9,  1, 0x00),			/* #9  Survival time */
	A_B(256, 10, 24, OMCI_B_LOID),		/* #10 Logical ONU ID */
	A_B(256, 11, 12, OMCI_B_ZEROS),		/* #11 Logical password */
	A_C(256, 12,  1, 0x00),			/* #12 Credentials status */
	A_C(256, 13,  2, 0x0000),		/* #13 Ext TC-layer options */
	A_C(256, 14,  1, 0x01),			/* #14 ONT state */

	/* ---- ME 257 ONU2-G (inst 0) ---- */
	A_B(257,  1, 20, OMCI_B_EQUIP_ID),	/* #1  Equipment ID */
	A_C(257,  2,  1, 0x80),			/* #2  OMCC version: G.984.4,
						 * BASELINE only — devid 0x0b is
						 * not served, and the two must
						 * stay consistent */
	A_C(257,  3,  2, 0x0031),		/* #3  Vendor product code */
	A_C(257,  4,  1, 0x01),			/* #4  Security capability */
	A_C(257,  5,  1, 0x01),			/* #5  Security mode */
	A_C(257,  6,  2, 0x0060),		/* #6  Total priority queues */
	A_C(257,  7,  1, 0x0c),			/* #7  Total traffic scheds */
	A_C(257,  8,  1, 0x01),			/* #8  Mode */
	A_C(257,  9,  2, 0x0040),		/* #9  Total GEM ports */
	A_C(257, 10,  4, 3600),			/* #10 SysUpTime — UINT32: two
						 * bytes here misaligns every
						 * later attr (proven bug) */
	A_C(257, 11,  2, 0x007f),		/* #11 Connectivity capability */
	A_C(257, 12,  1, 0x00),			/* #12 Current conn mode */
	A_C(257, 13,  2, 0x003b),		/* #13 QoS config flexibility */
	A_C(257, 14,  2, 0x0001),		/* #14 Priority-queue scale */

	/* ---- ME 5 Cardholder (inst 0x0101) ---- */
	A_C(5, 1, 1, 47),			/* #1  Actual type = Eth UNI */
	A_C(5, 2, 1, 47),			/* #2  Expected type */
	A_C(5, 3, 1, 1),			/* #3  Expected port count */

	/* ---- ME 6 Circuit-Pack (inst 0x0101) ---- */
	A_C(6,  1,  1, 47),			/* #1  Type */
	A_C(6,  2,  1, 1),			/* #2  Number of ports */
	A_SN(6, 3),				/* #3  Serial number */
	A_B(6,  4, 14, OMCI_B_SW_BANK0),	/* #4  Version */
	A_B(6,  5,  4, OMCI_B_VENDOR),		/* #5  Vendor ID */
	A_C(6, 12,  1, 8),			/* #12 Total priority queues */

	/* ---- ME 7 Software-Image, banks 0 (active) + 1 ---- */
	A_D(7, 1, 14, OMCI_DYN_SW_VER),		/* #1  Version */
	A_D(7, 2,  1, OMCI_DYN_SW_FLAG),	/* #2  Is committed */
	A_D(7, 3,  1, OMCI_DYN_SW_FLAG),	/* #3  Is active */
	A_C(7, 4,  1, 1),			/* #4  Is valid */
	A_B(7, 5, 16, OMCI_B_ZEROS),		/* #5  Image hash */

	/* ---- ME 11 PPTP Ethernet UNI (inst 0x0101) — THE HGU gate ---- */
	A_C(11,  1, 1, 47),			/* #1  Expected type */
	A_C(11,  2, 1, 47),			/* #2  Sensed type */
	A_C(11,  3, 1, 0),			/* #3  Auto-detect config */
	A_C(11,  4, 1, 0),			/* #4  Eth loopback config */
	A_C(11,  5, 1, 0),			/* #5  Admin state (unlocked) */
	A_C(11,  6, 1, 1),			/* #6  Op state */
	A_C(11,  7, 1, 0),			/* #7  Config ind */
	A_C(11,  8, 2, 1518),			/* #8  Max frame size */
	A_C(11,  9, 1, 0),			/* #9  DTE/DCE ind */
	A_C(11, 10, 2, 0xffff),			/* #10 Pause time */
	A_C(11, 11, 1, 2),			/* #11 Bridged/IP ind */
	A_C(11, 12, 1, 0),			/* #12 ARC */
	A_C(11, 13, 1, 0),			/* #13 ARC interval */
	A_C(11, 14, 1, 0),			/* #14 PPPoE filter */
	A_C(11, 15, 1, 0),			/* #15 Power control */

	/* ---- ME 131 OLT-G: known, no modelled attributes (the OLT Sets it) */
	A_NO_ATTRS(131),

	/* ---- ME 262 T-CONT (inst 0x8000..0x800b) ---- */
	A_D(262, 1, 2, OMCI_DYN_TCONT_ALLOC),	/* #1  Alloc-ID */
	A_C(262, 2, 1, 1),			/* #2  Mode indicator */
	A_C(262, 3, 1, 0),			/* #3  Policy */

	/* ---- ME 263 ANI-G (inst 0x8001) ---- */
	A_C(263,  1, 1, 1),			/* #1  SR indication */
	A_C(263,  2, 2, 12),			/* #2  Total T-CONTs */
	A_C(263,  3, 2, 48),			/* #3  GEM block length */
	A_C(263,  4, 1, 0),			/* #4  Piggyback DBA */
	A_C(263,  5, 1, 0),			/* #5  (deprecated) */
	A_C(263,  6, 1, 5),			/* #6  SF threshold */
	A_C(263,  7, 1, 9),			/* #7  SD threshold */
	A_C(263,  8, 1, 0),			/* #8  ARC */
	A_C(263,  9, 1, 0),			/* #9  ARC interval */
	/* #10/#14 are the LIVE optical levels, sampled from the optic's
	 * SFF-8472 A2h diagnostics by the shell (see omci_onu_set_optical);
	 * until the first successful read — and after a failed one — they serve
	 * OMCI_ANIG_{RX,TX}_FALLBACK, because the OLT must never get silence. */
	A_D(263, 10, 2, OMCI_DYN_ANIG_RX),	/* #10 RX optical level */
	A_C(263, 11, 1, 0xff),			/* #11 Lower optical thresh */
	A_C(263, 12, 1, 0xff),			/* #12 Upper optical thresh */
	A_C(263, 13, 2, 0x0000),		/* #13 ONU response time */
	A_D(263, 14, 2, OMCI_DYN_ANIG_TX),	/* #14 TX optical level */
	A_C(263, 15, 1, 0x81),			/* #15 Lower TX power thresh */
	A_C(263, 16, 1, 0x81),			/* #16 Upper TX power thresh */

	/* ---- ME 264 UNI-G (inst 0x0101) ---- */
	A_C(264, 1, 2, 0x0000),			/* #1  Config-option status */
	A_C(264, 2, 1, 0),			/* #2  Admin state */
	A_C(264, 3, 1, 1),			/* #3  Management capability */
	A_C(264, 4, 2, 0x0000),			/* #4  Non-OMCI mgmt ID */
	A_C(264, 5, 2, 0x0000),			/* #5  Relay-agent options */

	/* ---- ME 277 Priority-Queue (inst 0..7) ---- */
	A_C(277, 1, 1, 1),			/* #1  Queue config option */
	A_C(277, 2, 2, 3276),			/* #2  Max queue size */
	A_C(277, 3, 2, 3276),			/* #3  Allocated queue size */
	A_C(277, 4, 2, 0),			/* #4  Discard reset interval */
	A_C(277, 5, 2, 0),			/* #5  Threshold value */
	A_D(277, 6, 4, OMCI_DYN_PQ_PORT),	/* #6  Related port */
	A_C(277, 7, 2, 0x0000),			/* #7  Traffic-sched pointer */
	A_C(277, 8, 1, 1),			/* #8  Weight */

	/* ---- ME 278 Traffic-Scheduler (inst 0x8000..0x800b) ---- */
	A_D(278, 1, 2, OMCI_DYN_TS_TCONT),	/* #1  T-CONT pointer */
	A_C(278, 2, 2, 0x0000),			/* #2  Traffic-sched pointer */
	A_C(278, 3, 1, 1),			/* #3  Policy */
	A_C(278, 4, 1, 0),			/* #4  Priority/weight */

	/* ---- ME 329 VEIP (inst 0x0601) — the HGU marker ---- */
	A_C(329, 1, 1, 0),			/* #1  Admin state */
	A_C(329, 2, 1, 0),			/* #2  Op state */
	A_C(329, 3, 2, 0x0000),			/* #3  Interworking-TP ptr */

	/* ---- ME 65530 CTC LoID authentication (inst 0) ---- */
	A_B(65530, 1,  4, OMCI_B_OPER_ID),	/* #1  Operation ID */
	A_B(65530, 2, 24, OMCI_B_LOID),		/* #2  LoID */
	A_B(65530, 3, 12, OMCI_B_ZEROS),	/* #3  Password ("" but MUST be
						 * servable, proven) */
	A_C(65530, 4,  1, 0x01),		/* #4  Auth status = success */

	{ 0, 0, 0, 0, 0 },			/* terminator (class 0 is not a
						 * G.988 class ID) */
};

/*
 * Is @class_id in a G.988 vendor-reserved range (240..255, 350..399,
 * 65280..65535)?  An ONU cannot know WHICH vendor MEs a foreign OLT audits, so
 * the whole reserved space gets ONE policy: a KNOWN ME that models no
 * attributes.  UNKNOWN_ME here aborts an OLT's config load — proven on this
 * HSGQ OLT with classes 0xfff9 and 0xffb1, which used to be hard-coded one by
 * one.  Stock does the same job as DATA (/etc/omci_ignore_mib_tbl.conf lists
 * 255, 247, 65417, 65427, 65505..65509), i.e. a set of classes to answer
 * without modelling; a range policy is the same rule without the list.
 * Vendor MEs are intentionally absent from the MIB upload.
 */
static bool omci_vendor_class(u16 class_id)
{
	return (class_id >= 240 && class_id <= 255) ||
	       (class_id >= 350 && class_id <= 399) ||
	       class_id >= 65280;
}

/* First descriptor row of @class_id, or NULL if the model does not carry it. */
static const struct omci_attr *omci_me_find(u16 class_id)
{
	const struct omci_attr *a;

	for (a = omci_attrs; a->class_id; a++)
		if (a->class_id == class_id)
			return a;
	return NULL;
}

/* Does the ONU model this class at all (either a descriptor or the vendor
 * range policy)? */
static bool omci_class_modelled(u16 class_id)
{
	return omci_me_find(class_id) || omci_vendor_class(class_id);
}

/* The bytes of one attribute.  Integers are big-endian, right-aligned in
 * @size octets; @scratch must hold 4 bytes. */
static const u8 *omci_attr_bytes(const struct omci_onu *o,
				 const struct omci_attr *a, u16 inst,
				 u8 *scratch)
{
	u32 val;

	switch (a->src) {
	case OMCI_SRC_BLOB:
		return omci_blob + a->v;
	case OMCI_SRC_SN:
		return o->sn;
	case OMCI_SRC_MDS:
		val = o->mds;
		break;
	case OMCI_SRC_DYN:
		switch (a->v) {
		case OMCI_DYN_SW_VER:
			return omci_blob + (inst ? OMCI_B_SW_BANK1 :
						   OMCI_B_SW_BANK0);
		case OMCI_DYN_SW_FLAG:
			val = inst ? 0 : 1;
			break;
		case OMCI_DYN_TCONT_ALLOC:
			val = (inst == 0x8000) ? 0x0100 : 0x00ff;
			break;
		case OMCI_DYN_PQ_PORT:
			val = ((u32)0x0101 << 16) | (7u - (inst & 7));
			break;
		case OMCI_DYN_ANIG_RX:
			val = o->anig_rx_level;
			break;
		case OMCI_DYN_ANIG_TX:
			val = o->anig_tx_level;
			break;
		default:	/* OMCI_DYN_TS_TCONT */
			val = inst;
			break;
		}
		break;
	default:		/* OMCI_SRC_CONST */
		val = a->v;
		break;
	}
	scratch[0] = (u8)(val >> 24);
	scratch[1] = (u8)(val >> 16);
	scratch[2] = (u8)(val >> 8);
	scratch[3] = (u8)val;
	return scratch + 4 - a->size;
}

/*
 * ---- the ONE generic attribute filler ----
 * Shared by GET and MIB-Upload-Next so both byte-match.  @mask selects
 * attributes (bit15 = attr #1); the selected ones are emitted into [v..end)
 * in descriptor order, bounded.  An attribute that does not fit is SKIPPED and
 * a later smaller one may still be emitted (G.988 lets the reply carry what
 * fits and name the rest).
 *   *rmask_out = the attributes actually emitted,
 *   *known_out = every attribute this ME models, whether requested or not —
 *                which is what lets the caller distinguish "unsupported" from
 *                "did not fit" instead of answering success with a short mask.
 */
static u8 omci_me_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
		       u8 *v, const u8 *end, u16 *rmask_out, u16 *known_out)
{
	const struct omci_attr *a = omci_me_find(class_id);
	u16 rmask = 0, known = 0;
	bool over = false;

	*rmask_out = 0;
	*known_out = 0;
	if (!a)
		return omci_vendor_class(class_id) ? OMCI_RC_OK :
						     OMCI_RC_UNKNOWN_ME;

	for (; a->class_id == class_id; a++) {
		u16 bit;
		u8 scratch[4];

		if (!a->attr)			/* marker row: no attributes */
			continue;
		bit = (u16)OMCI_ATTR_BIT(a->attr);
		known |= bit;
		if (!(mask & bit))
			continue;
		if (v + a->size > end) {
			over = true;
			continue;
		}
		memcpy(v, omci_attr_bytes(o, a, inst, scratch), a->size);
		v += a->size;
		rmask |= bit;
	}

	*rmask_out = rmask;
	*known_out = known;
	return over ? OMCI_RC_ATTR_FAILED : OMCI_RC_OK;
}

/*
 * Build the static MIB-Upload row table: every auto-instantiated hardware ME
 * the HSGQ-G008 OLT expects to read back, split so each row's attributes fit
 * the 26-byte Upload-Next value area.  The OLT counts the ME 11 instances to
 * classify the ONU as HGU; an empty upload loops its "ONU config load fail".
 * This table is also the ONU's statement of WHICH INSTANCES exist, so a Set of
 * an instance not listed here (and never created) is answered 0x05.
 */
static void omci_build_mib(struct omci_onu *o)
{
	u16 n = 0;
	u16 i;

#define ROW(c, ins, m) do {						\
		if (n < OMCI_MIB_ROWS_MAX) {				\
			o->rows[n].class_id = (c);			\
			o->rows[n].inst = (ins);			\
			o->rows[n].mask = (m);				\
			n++;						\
		}							\
	} while (0)

	ROW(OMCI_ME_ONU_DATA, 0x0000, OMCI_ATTR_BIT(1));

	/* ME 256 ONU-G: 14 attrs split by the 26-byte cap:
	 * A = vid(4)+ver(14)+sn(8) = 26, B = #4..#9 = 6x1, C = LoID(24),
	 * D = #11(12)+#12(1)+#13(2)+#14(1) = 16. */
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
				   OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7) |
				   OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(10));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12) |
				   OMCI_ATTR_BIT(13) | OMCI_ATTR_BIT(14));

	/* ME 257 ONT2-G: A = EquipmentID(20), B = all scalars (21B). */
	ROW(OMCI_ME_ONU2_G, 0x0000, OMCI_ATTR_BIT(1));
	ROW(OMCI_ME_ONU2_G, 0x0000, OMCI_ATTR_BIT(2) | OMCI_ATTR_BIT(3) |
				    OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
				    OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7) |
				    OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9) |
				    OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
				    OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13) |
				    OMCI_ATTR_BIT(14));

	ROW(OMCI_ME_CARDHOLDER, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					OMCI_ATTR_BIT(3));

	/* ME 6 Circuit-Pack: A = #1..#4 = 24B, B = #5(4)+#12(1) = 5B. */
	ROW(OMCI_ME_CIRCUIT_PACK, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_CIRCUIT_PACK, 0x0101, OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(12));

	/* ME 7 Software-Image x2 banks: A = ver+committed+active+valid = 17B,
	 * B = hash(16). */
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				      OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(5));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				      OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(5));

	/* ME 11 PPTP Ethernet UNI: #1..#15 = 17B, one row.  THE HGU GATE. */
	ROW(OMCI_ME_PPTP_ETH_UNI, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
					  OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
					  OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) |
					  OMCI_ATTR_BIT(9) | OMCI_ATTR_BIT(10) |
					  OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12) |
					  OMCI_ATTR_BIT(13) | OMCI_ATTR_BIT(14) |
					  OMCI_ATTR_BIT(15));

	ROW(OMCI_ME_OLT_G, 0x0000, 0x0000);

	/* ME 263 ANI-G: A = #1..#9 = 11B, B = #10..#16 = 10B. */
	ROW(OMCI_ME_ANI_G, 0x8001, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
				   OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
				   OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) |
				   OMCI_ATTR_BIT(9));
	ROW(OMCI_ME_ANI_G, 0x8001, OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
				   OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13) |
				   OMCI_ATTR_BIT(14) | OMCI_ATTR_BIT(15) |
				   OMCI_ATTR_BIT(16));

	/* ME 262 T-CONT (inst 0x8000..0x800b): 4B each. */
	for (i = 0; i < 12; i++)
		ROW(OMCI_ME_TCONT, 0x8000 + i, OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3));

	ROW(OMCI_ME_UNI_G, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
				   OMCI_ATTR_BIT(5));

	/* ME 277 Priority-Queue: only the single UNI's 8 queues.  The full
	 * 96-row stock set made the upload so long the OLT's auth timer
	 * deactivated us mid-config (proven on the 9602C). */
	for (i = 0; i < 8; i++)
		ROW(OMCI_ME_PRIORITY_QUEUE, i, OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3) |
					       OMCI_ATTR_BIT(4) |
					       OMCI_ATTR_BIT(5) |
					       OMCI_ATTR_BIT(6) |
					       OMCI_ATTR_BIT(7) |
					       OMCI_ATTR_BIT(8));

	/* ME 278 Traffic-Scheduler (inst 0x8000..0x800b): 6B each. */
	for (i = 0; i < 12; i++)
		ROW(OMCI_ME_TRAFFIC_SCHED, 0x8000 + i, OMCI_ATTR_BIT(1) |
						       OMCI_ATTR_BIT(2) |
						       OMCI_ATTR_BIT(3) |
						       OMCI_ATTR_BIT(4));

	ROW(OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				  OMCI_ATTR_BIT(3));

	/* ME 65530 CTC LoID auth: 29B total -> A = #1(4)+#4(1), B = #2(24). */
	ROW(OMCI_ME_CTC_LOID_AUTH, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_CTC_LOID_AUTH, 0x0000, OMCI_ATTR_BIT(2));

#undef ROW
	o->nrows = n;
}

void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed)
{
	memset(o, 0, sizeof(*o));
	memcpy(o->sn, sn, 8);
	o->mds = mds_seed;
	/* Seed the ANI-G optical levels with the static fallback: a fresh MIB must
	 * be able to answer an ANI-G GET before the first DDM sample lands (the
	 * OLT audits within seconds of O5).  anig_live stays false until the shell
	 * publishes a real measurement. */
	o->anig_rx_level = OMCI_ANIG_RX_FALLBACK;
	o->anig_tx_level = OMCI_ANIG_TX_FALLBACK;
	omci_build_mib(o);
}

/* Is (class, inst) a MIB instance this ONU holds?  Three sources: the static
 * auto-instantiated set (== the MIB-Upload rows, which is what the OLT learned
 * from us), any vendor-reserved class (we model no attributes but the OLT is
 * entitled to address them), and anything the OLT itself created. */
static bool omci_inst_exists(struct omci_onu *o, u16 class_id, u16 inst)
{
	u16 i;

	if (omci_vendor_class(class_id))
		return true;
	if (omci_store_find(o, class_id, inst))
		return true;
	for (i = 0; i < o->nrows; i++)
		if (o->rows[i].class_id == class_id && o->rows[i].inst == inst)
			return true;
	return false;
}

/*
 * GET-response filler: result(8) + attr-mask(9,10) + values(11..35) + the two
 * masks G.988 RESERVES at 36..39 even on a success reply — the optional-
 * attribute ("unsupported") mask and the attribute-execution ("failed") mask.
 * So the value area is 25 octets, not 29: ONU-G attrs 1|2|3 are 4+14+8 = 26
 * bytes and a conformant OLT decoder would read a serial number short by its
 * last byte plus a bogus non-zero unsupported mask.
 *
 * Three masks decide the answer:
 *   requested (@mask), known (what the ME models), returned (what fit)
 *   unsupported = requested & ~known      -> named at 36..37
 *   failed      = requested & known & ~returned -> named at 38..39
 *   result      = 0x09 when either is set, else 0x00
 * "result 0 with a short attribute mask" is the audit-loop generator: the OLT
 * has no way to learn which attributes to stop asking for, so it re-GETs
 * forever.  Naming them is what ends the loop.
 *
 * Falls back to the dynamic store for OLT-created MEs (a GET of a provisioned
 * ME must not answer UNKNOWN_ME, which aborts the OLT's config load).
 */
static u8 omci_get_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
			u8 *resp)
{
	u16 rmask = 0, known = 0, unsup, failed;
	u8 rc = omci_me_fill(o, class_id, inst, mask, resp + 11, resp + 36,
			     &rmask, &known);

	if (rc == OMCI_RC_UNKNOWN_ME) {
		struct omci_me_inst *e = omci_store_find(o, class_id, inst);

		if (!e) {
			/* Nothing here at all.  If the OLT created OTHER
			 * instances of this class the class IS known and only
			 * the instance is not (0x05); otherwise the class
			 * itself is unknown (0x04). */
			omci_put_be16(resp + 9, 0);
			return omci_store_has_class(o, class_id) ?
					OMCI_RC_UNKNOWN_INST :
					OMCI_RC_UNKNOWN_ME;
		}
		/* Opaque set-by-create/set body: no descriptor table exists for
		 * an OLT-created class, so the bytes are replayed as-is and the
		 * requested mask is echoed (best-effort, bounded by the 25-octet
		 * area).  Naming them unsupported instead would make the OLT
		 * abandon a ME it just provisioned. */
		memcpy(resp + 11, e->body, e->blen > 25 ? 25 : e->blen);
		rmask = mask;
		known = mask;
	}

	omci_put_be16(resp + 9, rmask);
	unsup = (u16)(mask & ~known);
	failed = (u16)(mask & known & ~rmask);
	omci_put_be16(resp + 36, unsup);
	omci_put_be16(resp + 38, failed);
	return (unsup | failed) ? OMCI_RC_ATTR_FAILED : OMCI_RC_OK;
}

/*
 * Create / Set / Delete: APPLY or NAK, and move MIB-Data-Sync ONLY when the
 * MIB actually changed.  An ACK the ONU did not honour is worse than a NAK:
 * the OLT stops retrying AND its lsync still matches our MDS, so the ME 2
 * audit can never discover the divergence.
 *   Create: duplicate instance -> 0x07, full store -> 0x09 (frozen MDS lets
 *           the OLT's own audit self-heal), else store + MDS+1.
 *   Delete: absent instance -> 0x05.
 *   Set:    unknown class -> 0x04, known class + absent instance -> 0x05.
 * Attribute-level Set validation is deliberately NOT done: this OLT Sets
 * ME 131 (OLT-G) attributes the ONU does not model and expects OK, and G.988
 * has no way for the ONU to announce a per-attribute write capability.
 */
static u8 omci_config_apply(struct omci_onu *o, u8 mt, u16 class_id, u16 inst,
			    const u8 *msg, unsigned int len)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);
	u16 mask;

	switch (mt) {
	case OMCI_MT_CREATE:
		/* The dynamic store is the OLT-created space only: an
		 * auto-instantiated ME is not "existing" for Create purposes
		 * (this OLT Creates ME 262/268-shaped instances that the static
		 * model also describes). */
		if (e)
			return OMCI_RC_INST_EXISTS;
		if (!omci_store_put(o, class_id, inst, msg + 8,
				    (len > 8) ? (int)(len - 8) : 0))
			return OMCI_RC_ATTR_FAILED;
		break;
	case OMCI_MT_DELETE:
		if (!e)
			return OMCI_RC_UNKNOWN_INST;
		omci_store_del(o, class_id, inst);
		break;
	default:					/* OMCI_MT_SET */
		if (len < 10)		/* no attribute mask on the wire */
			return OMCI_RC_PARAM_ERROR;
		if (!omci_inst_exists(o, class_id, inst))
			return (omci_class_modelled(class_id) ||
				omci_store_has_class(o, class_id)) ?
					OMCI_RC_UNKNOWN_INST :
					OMCI_RC_UNKNOWN_ME;
		mask = ((u16)msg[8] << 8) | msg[9];
		if (e)
			omci_store_merge(e, msg + 10, (int)(len - 10));
		/* An OLT Set of ME2 attr-1 is an explicit resync write: take
		 * its byte first, then this Set's own +1 still applies. */
		if (class_id == OMCI_ME_ONU_DATA && len >= 11 &&
		    (mask & 0x8000))
			o->mds = msg[10];
		break;
	}

	/* MIB-Data-Sync: +1 per applied config message (not per attribute),
	 * wrap 255 -> 1 (0 = just-reset). */
	if (++o->mds == 0)
		o->mds = 1;
	return OMCI_RC_OK;
}

int omci_onu_input(struct omci_onu *o, const u8 *msg, unsigned int len, u8 *resp)
{
	u16 class_id, inst;
	u8 mt, devid;

	if (len < 8)
		return 0;
	devid = msg[3];
	mt = msg[2] & 0x1f;
	class_id = ((u16)msg[4] << 8) | msg[5];
	inst = ((u16)msg[6] << 8) | msg[7];

	if (devid != 0x0a) {
		/* Only the BASELINE message set is modelled.  An extended-format
		 * request (devid 0x0b) cannot be answered in baseline format —
		 * the response device identifier must match — so it is counted
		 * and dropped rather than answered wrongly.  ONU2-G attribute 2
		 * (OMCC version) therefore advertises 0x80 = G.984.4 BASELINE:
		 * a conformant OLT never sends an extended frame to us, and the
		 * counter says loudly if one ever does. */
		if (devid == 0x0b)
			o->rx_extended++;
		return 0;
	}

	/* G.988 11.2.2.1 retained last response: the OMCC is stop-and-wait, so
	 * a byte-identical repeat of the request we last answered is a
	 * RETRANSMISSION (our US response was lost — cg_omci_tx drops on NI
	 * ring-busy, and a US burst can die on the wire).  Replay the stored
	 * response instead of re-executing: re-execution bumps MDS a second
	 * time for ONE OLT transaction, and ONU mds = OLT lsync + 1 costs a
	 * full MIB-Reset/re-provision churn window at the next ME 2 audit.
	 * Bytes 40..47 (trailer + MIC) are derived, so 0..39 is the identity. */
	if (o->have_last && len >= 40 && !memcmp(msg, o->last_req, 40)) {
		memcpy(resp, o->last_resp, OMCI_LEN);
		o->dup_replay++;
		return OMCI_LEN;
	}

	memset(resp, 0, OMCI_LEN);
	resp[0] = msg[0];			/* TID echo */
	resp[1] = msg[1];
	resp[2] = (msg[2] & 0x1f) | 0x20;	/* clear AR, set AK */
	resp[3] = 0x0a;
	resp[4] = msg[4];			/* class echo */
	resp[5] = msg[5];
	resp[6] = msg[6];			/* instance echo */
	resp[7] = msg[7];

	switch (mt) {
	case OMCI_MT_MIB_RESET:
		/* On-wire MIB-Reset: zero MIB-Data-Sync (the OLT recounts its
		 * lsync from 0; keeping a seed here = permanent mismatch ->
		 * Deactivate loop, proven) + drop the provisioned store. */
		o->mds = 0;
		memset(o->store, 0, sizeof(o->store));
		o->store_n = 0;
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_MIB_UPLOAD:
		/* Row count at contents[8..9], NO result byte (a result byte
		 * here made the OLT read count=0 and never walk, proven). */
		omci_put_be16(resp + 8, o->nrows + o->store_n);
		break;
	case OMCI_MT_GET:
		if (len < 10)	/* mask missing: a shorter GET would read
				 * stale bytes into the reply (info leak) */
			return 0;
		resp[8] = omci_get_fill(o, class_id, inst,
					((u16)msg[8] << 8) | msg[9], resp);
		break;
	case OMCI_MT_SET:
	case OMCI_MT_CREATE:
	case OMCI_MT_DELETE:
		resp[8] = omci_config_apply(o, mt, class_id, inst, msg, len);
		break;
	case OMCI_MT_GET_ALL_ALARMS:
		/* Alarm-entry count at contents[8..9], NO result byte — same
		 * shape as MIB-Upload.  (At 9..10 the count's high byte lands
		 * where the OLT reads a result code: latent while the count is
		 * always 0, wrong the moment an alarm is reported.) */
		omci_put_be16(resp + 8, 0x0000);	/* no active alarms */
		break;
	case OMCI_MT_MIB_UPLOAD_NX: {
		/* Request seq at msg[8..9]; reply = class[8..9] + inst[10..11]
		 * + attr-mask[12..13] + values[14..39], NO result byte. */
		u16 seq;
		u16 wmask = 0, wknown = 0;

		if (len < 10)
			return 0;
		seq = ((u16)msg[8] << 8) | msg[9];
		if (seq < o->nrows) {
			const struct omci_mib_row *r = &o->rows[seq];

			omci_put_be16(resp + 8, r->class_id);
			omci_put_be16(resp + 10, r->inst);
			omci_me_fill(o, r->class_id, r->inst, r->mask,
				     resp + 14, resp + 40, &wmask, &wknown);
			omci_put_be16(resp + 12, wmask);
		} else if (seq < o->nrows + o->store_n) {
			/* provisioned MEs after the static rows: present-only
			 * (mask 0); values are served via GET. */
			const struct omci_me_inst *e =
				omci_store_nth(o, seq - o->nrows);

			if (e) {
				omci_put_be16(resp + 8, e->class_id);
				omci_put_be16(resp + 10, e->inst);
			}
		}
		/* out-of-range seq -> all-zero row, still well-formed */
		break;
	}
	case OMCI_MT_TEST:
	case OMCI_MT_SYNC_TIME:
	case OMCI_MT_REBOOT:		/* ACK, do NOT actually reboot */
	case OMCI_MT_START_SW_DL:
	case OMCI_MT_DOWNLOAD_SEC:
	case OMCI_MT_END_SW_DL:
	case OMCI_MT_ACTIVATE_SW:
	case OMCI_MT_COMMIT_SW:
		/* not performed (no SW image to flash), but must ACK OK so
		 * the OLT's provisioning FSM completes */
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_GET_ALL_ALRM_NX:
	case OMCI_MT_GET_NEXT:
		/* Get Next walks a TABLE attribute; the ME model defines none
		 * (only an OLT-created ME could have one, and its body is
		 * opaque to us), so the honest answer is the "action this ME
		 * does not implement" one below: result 0x00 with empty
		 * contents = nothing to return / end of table.  Get-All-Alarms-
		 * Next likewise: no alarm table to walk.  resp is already zero. */
		break;
	default:
		/* A message type with no ONU-side action.  Answer result 0x00
		 * with EMPTY contents, never 0x02 and never silence: stock
		 * behaves this way, an unanswered OLT request is a documented
		 * deactivation trigger, and 0x02 has been seen to abort a
		 * foreign OLT's config load.  Counted so /proc shows if an OLT
		 * ever sends one (spy-capability rule). */
		o->unhandled++;
		break;
	}

	omci_finalize(resp);

	/*
	 * Refresh the retransmission cache.  It may only ever hold the response
	 * to the request we answered MOST RECENTLY: when this request produced
	 * no response (AR clear) or is too short to be identified by its first
	 * 40 bytes, the previous entry must be DROPPED — the MIB may just have
	 * changed underneath it, and replaying it would answer a later
	 * transaction with a pre-change reply (an AR=0 Set followed by a repeat
	 * of the ME 2 audit GET would report the OLD MIB-Data-Sync).
	 */
	if ((msg[2] & 0x40) && len >= 40) {
		memcpy(o->last_req, msg, 40);
		memcpy(o->last_resp, resp, OMCI_LEN);
		o->have_last = true;
	} else {
		o->have_last = false;
	}

	/* AR clear = the OLT asked for no acknowledgement (G.988): the message
	 * is APPLIED above, but nothing goes upstream.  Counted, because a
	 * silent path still has to be observable — /proc says whether this OLT
	 * ever uses it (spy-capability rule). */
	if (!(msg[2] & 0x40)) {
		o->no_ack++;
		return 0;
	}
	return OMCI_LEN;
}

/*
 * Autonomous AVC (MT 0x11, TID 0): report that (class, inst)'s attributes in
 * @mask changed to @val.  The OLT never GETs the data-plane MEs after
 * creating them — its per-class AVC handlers gate DOWNSTREAM user-data
 * forwarding on the ONU's operational report. */
static void omci_emit_avc(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
			  const u8 *val, unsigned int vlen, u8 *out)
{
	memset(out, 0, OMCI_LEN);
	out[2] = OMCI_MT_AVC;
	out[3] = 0x0a;
	omci_put_be16(out + 4, class_id);
	omci_put_be16(out + 6, inst);
	omci_put_be16(out + 8, mask);
	if (val && vlen) {
		if (vlen > 30)
			vlen = 30;
		memcpy(out + 10, val, vlen);
	}
	omci_finalize(out);
	o->avc_count++;
}

int omci_onu_emit_veip_up_avc(struct omci_onu *o, u8 *out)
{
	/* VEIP inst 0x0601 attr #2 (operational state) mask 0x4000,
	 * value 0 = enabled (G.988). */
	static const u8 up = 0x00;

	omci_emit_avc(o, OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(2), &up, 1, out);
	o->avc_veip_up_sent = true;
	return OMCI_LEN;
}
