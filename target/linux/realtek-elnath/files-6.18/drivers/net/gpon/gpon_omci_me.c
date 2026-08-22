// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.988 managed-entity model and MIB.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 */
/*
 * gpon_omci_me.c — the ITU-T G.988 MANAGED-ENTITY model and MIB store, common
 * to every OpenWrt GPON target in this tree.
 *
 * WHAT THIS IS
 *   The data half of the ONU's OMCI responder, in three parts:
 *     1. the board identity blob + the TABLE-DRIVEN attribute descriptors
 *        (one row per (class, attribute): number, wire size, value source) and
 *        the ONE generic filler that walks them, shared by GET and
 *        MIB-Upload-Next so both byte-match by construction;
 *     2. the STATIC MIB-Upload row table — the ONU's statement of which
 *        instances exist, split so each row's attributes fit the 26-octet
 *        Upload-Next value area;
 *     3. the DYNAMIC store of instances the OLT created, plus the context
 *        initialiser.
 *   It never parses a PDU, dispatches a message type, builds a response
 *   envelope, stamps a trailer or computes a MIC — that is gpon_omci_core.c,
 *   the MESSAGE layer, which calls into here.  Correspondingly this file
 *   includes no crc32 header and touches no octet of a frame it did not fill.
 *
 * WHY IT IS COMMON — and which targets and architectures compile it
 *   Operator, 2026-08-05: "en openwrt debería estar estructurado algo así:
 *   rtl960x* para la familia para tener código común" … "la idea es poner en
 *   común el código que corresponde para no tener mucho duplicado", and on the
 *   two per-target monoliths that each carry a private copy: "mal, poner en
 *   común".
 *   Compiled by:
 *     - realtek-elnath (RTL9607F, Cortina)   aarch64, LITTLE-endian  — today
 *     - realtek-luna   (RTL960xC, Luna)      MIPS32,  BIG-endian     — NOT yet;
 *       Luna's own model in rtl9602c_eth.c emits different bytes, so adopting
 *       this one is a behaviour change with its own board gate (F1/F2/F3), not
 *       code motion.  See gpon_omci_me.h.
 *     - dev/rtl9607c-test on x86-64 through fuzz_shims/, under ASan+UBSan
 *
 * THE CORE/SHELL RULE IT OBEYS
 *   FUNCTIONAL CORE.  It decides; it never does.  No HW I/O, no locking (the
 *   caller serialises), no allocation, no sleeping, no clock read — every byte
 *   of state lives in the caller-provided struct omci_onu.  The imperative
 *   shell publishes the live optical measurement through omci_onu_set_optical()
 *   and reaches into the model no other way.
 *
 *   => THIS FILE MUST NEVER GAIN AN MMIO ACCESS.  No readl/writel, no ioremap,
 *   no msleep/udelay, no jiffies, no spin_lock/mutex, no kmalloc, no dev_ or
 *   netdev_ logging, no schedule_work.  The purity check greps for exactly that
 *   set and the gate goes red if one appears.  That property is what lets the
 *   whole model be swept on x86 — every attribute mask x every modelled class —
 *   instead of on a ~200 s board boot.
 *
 * ENDIANNESS
 *   Attribute integers are emitted big-endian by explicit byte math
 *   (omci_attr_bytes(): shift MSB-first into a 4-byte scratch, take the
 *   right-aligned tail).  No struct or pointer cast over wire bytes, ever.
 *   One source, two byte orders, identical octets.
 *
 * WHAT PINS IT
 *   Byte-for-byte equivalence of the table walk with the hand-written filler it
 *   replaced is pinned exhaustively (all 65536 attribute masks x every modelled
 *   class/instance) by dev/rtl9607c-test/omci_me_table_test, and the G.988
 *   cross-vendor behaviour by dev/rtl9607c-test/omci_conformance_test.  Those
 *   run against THIS file after the move; a green that costs fewer checks than
 *   before is not the same green, so the count is compared, not just the
 *   colour.
 *
 * PROVENANCE
 *   Code motion, 2026-08-05, from realtek-elnath's omci_responder.c
 *   (lines 135-240, 242-474, 476-611, 613-764, 766-782) — the already-pure ME
 *   model of the driver that is at stock parity.  Logic byte-for-byte as it
 *   shipped.  The ONLY textual change is dropping `static` from the nine
 *   functions the message layer now calls across the file boundary; every
 *   contract comment stays at its definition, where it was.
 *
 * FOUND WHILE MOVING, NOT FIXED HERE (fixing during a move makes the
 * regression un-bisectable — the fix comes with its own failing-first case):
 *   - omci_store_nth() walks the store in ARRAY order, so a Delete landing
 *     between two MIB-Upload-Next requests RENUMBERS the remaining rows and the
 *     OLT's walk silently skips one.  Plan follow-up F11.
 *   - ME 257 attribute 1 (equipment ID) is "HSGQ-X411AXF" on a unit certified
 *     as X400AXF.  It is on the wire to the OLT.  Plan follow-up F18 — check it
 *     against the stock dump before anyone "corrects" it.
 *   - the _Static_assert on omci_blob guards the TOTAL size, not the individual
 *     offsets; only the exhaustive x86 sweep pins those.
 *   - omci_me_fill()'s `over` flag is currently UNOBSERVABLE.  It turns the
 *     return into OMCI_RC_ATTR_FAILED, and the only caller that reads the
 *     return tests it solely for OMCI_RC_UNKNOWN_ME; the MIB-Upload-Next caller
 *     discards it.  The truncation ITSELF is still reported — through
 *     @rmask_out, from which the message layer derives its "failed" mask — so
 *     nothing is lost on the wire today.  But a future caller that trusts the
 *     return code will get a distinction nothing pins: inverting `over` leaves
 *     the whole exhaustive differential green.  Not removed, not rewired here.
 */
#include <linux/string.h>

#include "gpon_omci_me.h"

/* ---- dynamic (OLT-provisioned) ME store ---- */

struct omci_me_inst *omci_store_find(struct omci_onu *o, u16 class_id,
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
bool omci_store_has_class(struct omci_onu *o, u16 class_id)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used && o->store[k].class_id == class_id)
			return true;
	return false;
}

/* idx-th used entry in array order — the MIB-Upload tail rows. */
struct omci_me_inst *omci_store_nth(struct omci_onu *o, u16 idx)
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
bool omci_store_put(struct omci_onu *o, u16 class_id, u16 inst,
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
void omci_store_merge(struct omci_me_inst *e, const u8 *val, int vlen)
{
	if (vlen <= 0)
		return;
	if (vlen > (int)sizeof(e->body))
		vlen = sizeof(e->body);
	memcpy(e->body, val, vlen);
	if (e->blen < (u8)vlen)
		e->blen = (u8)vlen;
}

void omci_store_del(struct omci_onu *o, u16 class_id, u16 inst)
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
#define OMCI_B_VENDOR		0	/* "ZYOT"          (4)  */
#define OMCI_B_ONU_G_VER	4	/* "Hardware1.0"   (14) */
#define OMCI_B_SW_BANK0		18	/* "M225-260525"   (14) */
#define OMCI_B_SW_BANK1		32	/* "M225-260515"   (14) */
#define OMCI_B_EQUIP_ID		46	/* "AOT-5221ZY"    (20) */
#define OMCI_B_LOID		66	/* "user"          (24) */
#define OMCI_B_OPER_ID		90	/* "CTC"           (4)  */
#define OMCI_B_ZEROS		94	/* zeros           (16) */

static const u8 omci_blob[] = {
	/* [0] vendor ID — ONU-G #1, Circuit-Pack #5.  The OLT recognizes HSGQ
	 * ONUs; "XPON" was rejected. */
	'Z', 'Y', 'O', 'T',
	/* [4] ONU-G #2 version, zero-padded to 14 */
	'H', 'a', 'r', 'd', 'w', 'a', 'r', 'e', '1', '.', '0', 0, 0, 0,
	/* [18] SW-image bank 0 (active) version — also Circuit-Pack #4 */
	'M', '2', '2', '5', '-', '2', '6', '0', '5', '2', '5', 0, 0, 0,
	/* [32] SW-image bank 1 version */
	'M', '2', '2', '5', '-', '2', '6', '0', '5', '1', '5', 0, 0, 0,
	/* [46] ONU2-G #1 equipment ID, zero-padded to 20 */
	'A', 'O', 'T', '-', '5', '2', '2', '1', 'Z', 'Y',
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
	A_C(257,  3,  2, 0x0000),		/* #3  Vendor product code */
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
	/*
	 * ★★★ 2026-08-10: #6 Op state must be ENABLED (0), not disabled (1).
	 *
	 * G.988: operational state is enabled(0) / disabled(1), and we were reporting
	 * the PPTP Ethernet UNI - the very port the OLT bridges the data GEM to - as
	 * DISABLED.  The live OLT provisions us completely (MIB-reset, the full 51-row
	 * upload, MAC Bridge Service Profile, the GEM IWTP 225 and both bridge ports,
	 * VLAN tagging filter, T-CONT alloc 351) and then stops short of creating the
	 * bidirectional data GEM CTP 225 - which is what an OLT does when the UNI on the
	 * far side of that bridge reports it cannot carry traffic.  The VEIP (ME 329 #2)
	 * already reports enabled; these two disagreed.
	 */
	A_C(11,  6, 1, 0),			/* #6  Op state = enabled */
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
bool omci_class_modelled(u16 class_id)
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
u8 omci_me_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
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

	/*
	 * ★★★★★ 2026-08-10: OLT-G is NOT uploaded.
	 *
	 * It used to be row 15 with an attribute mask of 0x0000, and the live OLT's
	 * MIB-upload walk stopped dead at exactly that row - reproducibly, three runs
	 * running (15, 16, 15 upload-next messages, TCIs incrementing cleanly up to
	 * there and then the OLT falling back to Get-polling).  A MIB-upload-next
	 * reply carrying a row with no attributes is not something this OLT accepts,
	 * and it abandons the walk rather than skipping it.
	 *
	 * OLT-G is the OLT's OWN identity (vendor id, equipment id, version, ToD)
	 * which the OLT SETs into the ONU - there is nothing for us to report in an
	 * upload, and stock does not upload a maskless row here either.
	 */

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

	/*
	 * ME 65530 (CTC LoID authentication) is deliberately NOT uploaded.
	 * Stock models all four of its attributes (#1 Operation ID 4B, #2 LoID
	 * 24B, #3 Password 12B, #4 Auth status 1B) and answers a Get on every
	 * one of them, but keeps the whole CLASS out of the MIB upload: its
	 * table descriptor carries stdType 0x104, and both MIB-Upload walkers
	 * (row count and row packing alike) skip a table whose stdType has bit
	 * 0x10 or 0x100 set.  The exclusion is per class and all-or-nothing --
	 * the four attributes' optionType is 1, so none of them is filtered by
	 * the separate per-attribute optionType & 0x31A rule.  Uploading a
	 * SUBSET (what this used to do: #1|#4 then #2, dropping the 12-byte #3)
	 * matches neither stock nor a complete upload and leaves the OLT with a
	 * MIB copy the ONU can answer beyond.  A Get keeps working with no row:
	 * omci_inst_exists() short-circuits on omci_vendor_class().
	 */

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
bool omci_inst_exists(struct omci_onu *o, u16 class_id, u16 inst)
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
