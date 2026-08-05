/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.988 managed-entity model — interface.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 *
 * gpon_omci_me.h — the ITU-T G.988 MANAGED-ENTITY model and MIB store, common
 * to every OpenWrt GPON target in this tree.
 *
 * WHAT THIS IS
 *   The ONU's answer to "which managed entities do I hold, and what are their
 *   attribute bytes".  Three things, and nothing else:
 *     1. the TABLE-DRIVEN attribute descriptors — one row per (class,
 *        attribute) carrying {attribute number, wire size, value source} — plus
 *        the ONE generic filler that walks them;
 *     2. the STATIC MIB-Upload row table, i.e. the ONU's statement of which
 *        instances exist and how their attributes split across the 26-octet
 *        Upload-Next value area;
 *     3. the DYNAMIC store of the instances the OLT itself created, and the
 *        context struct (struct omci_onu) that holds all of it.
 *   It does NOT parse a PDU, dispatch a message type, build a response
 *   envelope, stamp a trailer or compute a MIC.  That is the MESSAGE layer,
 *   gpon_omci_core.{h,c}, which CALLS this one.  The seam is deliberate: the
 *   G.988 message rules are identical on every ONU ever built, while the set of
 *   MEs a product serves is a property of the PRODUCT.
 *
 *   Table-driven, and that is a footprint decision as much as a design one:
 *   one descriptor row per attribute costs bytes where a hand-written
 *   serialiser per ME costs kilobytes, and these ONUs ship with <=64 MB RAM and
 *   a small NAND.  It is also what makes three cross-vendor invariants
 *   STRUCTURAL instead of per-case — every reply bounded by the Get value area,
 *   the set of attributes an ME knows being derivable (so a Get can NAME the
 *   ones it does not support and the ones that did not fit, instead of
 *   answering success with a short mask and generating the OLT re-GET churn),
 *   and one policy per vendor-reserved class range instead of a hard-coded list
 *   of the class IDs one OLT happened to ask for.
 *
 * WHY IT IS COMMON — and which targets and architectures compile it
 *   Operator, 2026-08-05: "en openwrt debería estar estructurado algo así:
 *   rtl960x* para la familia para tener código común" … "la idea es poner en
 *   común el código que corresponde para no tener mucho duplicado", and on the
 *   two per-target monoliths that each carry a private copy: "mal, poner en
 *   común".
 *   Compiled by:
 *     - realtek-elnath (RTL9607F, Cortina)   aarch64, LITTLE-endian  — today
 *     - realtek-luna   (RTL960xC, Luna)      MIPS32,  BIG-endian     — NOT yet.
 *       Luna carries a third, independently written ME model in
 *       rtl9602c_eth.c, on static file-scope globals rather than a context
 *       struct, with a different identity (HSGQ-X111W) and a different ME 7 /
 *       ME 11 attribute set.  Pointing Luna at this file CHANGES LUNA'S
 *       EMITTED BYTES, so it is a behaviour change with its own board gate —
 *       follow-ups F1/F2/F3 — and was deliberately NOT part of the code-motion
 *       refactor that created this file.  Nothing here was altered to
 *       accommodate it.
 *     - dev/rtl9607c-test on x86-64 through fuzz_shims/, where the whole model
 *       is swept exhaustively (every attribute mask x every modelled class)
 *       under ASan+UBSan
 *   The prefix is gpon_ and not rtl960x_ on purpose: the layer must also serve
 *   the future ARM OLT and other brands, so a Realtek-named prefix would be too
 *   narrow for what it covers.
 *
 * THE CORE/SHELL RULE IT OBEYS
 *   FUNCTIONAL CORE.  It decides; it never does.  It answers questions about
 *   the MIB and fills caller-provided buffers.  No device pointer, no lock (the
 *   caller serialises), no allocation, no sleeping, no clock read — every byte
 *   of state lives in the caller-provided struct omci_onu.  The imperative
 *   shell (cortina-gpon.c on Elnath) owns the OMCC binding, the TX ring, the
 *   i2c DDM read and the /proc view; it publishes a live optical measurement
 *   through omci_onu_set_optical() and never reaches into the model otherwise.
 *
 *   => THIS FILE AND ITS .c MUST NEVER GAIN AN MMIO ACCESS.  No readl/writel,
 *   no ioremap, no msleep/udelay, no jiffies, no spin_lock/mutex, no kmalloc,
 *   no dev_ or netdev_ logging, no schedule_work.  The purity check greps for
 *   exactly that set and the gate goes red if one appears.  It is not hygiene:
 *   that property is what makes the model host-compilable, host-fuzzable and
 *   portable across the two architectures at once.
 *   (Written "dev_ or netdev_" and not with a glob on purpose: the obvious
 *   spelling puts the two characters that CLOSE a block comment in the middle
 *   of one, and the compiler then reads the rest of the banner as code.)
 *
 * ENDIANNESS
 *   Every attribute value is emitted big-endian by explicit byte math (see
 *   omci_attr_bytes() in the .c: an integer is shifted into a 4-byte scratch
 *   MSB-first and taken right-aligned).  Never a struct or pointer cast over
 *   wire bytes, never htons/ntohs on a buffer.  That is not style: it is the
 *   reason ONE source compiles for big-endian MIPS and little-endian ARM64 and
 *   emits the same octets on both.
 *
 * PROVENANCE
 *   Code motion, 2026-08-05, from realtek-elnath's omci_responder.{c,h} — the
 *   already-pure ME model of the driver that is at stock parity.  The logic is
 *   byte-for-byte what shipped; the only textual changes are the file split
 *   itself and dropping `static` from the nine functions the message layer
 *   calls across the new file boundary.  A defect found while moving is written
 *   down, never fixed in the same step, or the regression stops bisecting.
 */
#ifndef GPON_OMCI_ME_H
#define GPON_OMCI_ME_H

#include <linux/types.h>

/* The MESSAGE layer's own facts: OMCI_LEN, the G.988 result codes returned by
 * omci_me_fill(), and OMCI_ATTR_BIT().  It forward-declares struct omci_onu,
 * which this header then defines — so the include runs one way only and the
 * two headers never cycle. */
#include "gpon_omci_core.h"

/* A dynamic ME instance the OLT provisioned (Create).  Stored so GET and the
 * MIB-Upload reflect the actual configured MIB — without it the OLT's
 * post-config audit gets UNKNOWN_ME, re-runs the whole MIB-Reset/Upload/
 * Create sequence every ~50 s and finally Deactivates. */
#define OMCI_STORE_MAX		64
struct omci_me_inst {
	u16	class_id;
	u16	inst;
	u8	body[26];	/* set-by-create attribute bytes (26B cap) */
	u8	blen;
	bool	used;
};

/* One MIB-Upload-Next row: (class, instance, attr-mask) whose selected
 * attributes fit the 26-byte Upload-Next value area. */
struct omci_mib_row {
	u16	class_id;
	u16	inst;
	u16	mask;
};
#define OMCI_MIB_ROWS_MAX	72

struct omci_onu {
	u8	sn[8];			/* PLOAM serial number (vendor+VSSN) */
	u8	mds;			/* ME 2 attr 1: MIB-Data-Sync */
	u16	nrows;			/* static MIB row count */
	u16	store_n;		/* provisioned-ME count */
	struct omci_mib_row	rows[OMCI_MIB_ROWS_MAX];
	struct omci_me_inst	store[OMCI_STORE_MAX];
	/* G.988 11.2.2.1 retained last response: the OMCC is stop-and-wait, so
	 * ONE entry covers every retransmission — the OLT never advances past
	 * an unanswered transaction.  A byte-identical repeat is REPLAYED from
	 * here instead of re-executed, so a lost US response cannot bump
	 * MIB-Data-Sync twice for one OLT transaction. */
	u8	last_req[40];		/* bytes 0..39 (trailer+MIC derived) */
	u8	last_resp[OMCI_LEN];
	bool	have_last;
	/* spy counters (dump/probe capability is first-class, project rule) */
	u32	unhandled;		/* DS message types with no ONU action */
	u32	dup_replay;		/* retransmissions served from the cache */
	u32	rx_extended;		/* devid 0x0b frames seen (not served) */
	u32	no_ack;			/* requests with AR clear: applied, not
					 * answered — a silent path must still be
					 * countable */
	u32	avc_count;		/* autonomous AVC frames emitted */
	bool	avc_veip_up_sent;
	/* ME 263 ANI-G #10 RX / #14 TX optical level, in the G.988 wire form
	 * (2's complement, 0.002 dB increments referred to 1 mW).  Seeded by
	 * omci_onu_init() to the conformant STATIC fallback below and overwritten
	 * by the imperative shell from the live SFF-8472 A2h DDM read — the OLT's
	 * optical view of this ONU then tracks the real fiber instead of a
	 * plausible-looking constant.  The fallback is kept for a failed read
	 * because the OLT must never get silence, and @anig_live says which of the
	 * two a reader is looking at so a stub is never mistaken for a
	 * measurement.  The host oracle never calls the setter, so its GET
	 * responses stay byte-identical to the pre-DDM reference snapshot. */
	u16	anig_rx_level;
	u16	anig_tx_level;
	bool	anig_live;
};

/* The static ANI-G optical levels served until (and after a failed) DDM read.
 * 0xeedc = -8.77 dBm received, 0x04d7 = +2.47 dBm launched — both plausible for
 * this class-B+ optic, which is exactly why they must be labelled: a fabricated
 * value that looks right is the hardest kind to notice. */
#define OMCI_ANIG_RX_FALLBACK	0xeedc
#define OMCI_ANIG_TX_FALLBACK	0x04d7

/* Publish a live optical measurement into ME 263 #10/#14.  The caller does the
 * (sleeping) i2c read OUTSIDE whatever lock guards the responder and passes the
 * two already-converted wire values in. */
static inline void omci_onu_set_optical(struct omci_onu *o, u16 rx_level,
					u16 tx_level)
{
	o->anig_rx_level = rx_level;
	o->anig_tx_level = tx_level;
	o->anig_live = true;
}

/* ME class IDs presented in the MIB upload (G.988 + the HSGQ OLT's set).
 * ONU_DATA and VEIP are also defined identically by gpon_omci_core.h, which
 * reasons about those two itself; a repeated object-like #define with the
 * same replacement list is a benign redefinition, so each header stays
 * readable on its own. */
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

/* Init/reset the responder + rebuild the static MIB rows.  @mds_seed is the
 * MIB-Data-Sync boot value: a POISON that must NOT match the OLT's stored
 * lsync, so its ME2 audit mismatches and it re-provisions from MIB-Reset
 * (the X111W warm-readmit lesson; an on-wire MIB-Reset then zeroes it). */
void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed);

/* ------------------------------------------------------------------------
 * The ME-model API the MESSAGE layer (gpon_omci_core.c) calls.
 *
 * These nine were `static` while the model and the message rules shared one
 * translation unit; the split is the only reason they are declared here.  The
 * contract of each stays written at its DEFINITION in gpon_omci_me.c, where it
 * was, because that is where a reader who is changing the behaviour is
 * standing — a duplicated contract in a header drifts from the code it
 * describes, and this project has already paid for facts that stopped being
 * true while still looking authoritative.
 * ------------------------------------------------------------------------ */

/* dynamic (OLT-provisioned) ME store */
struct omci_me_inst *omci_store_find(struct omci_onu *o, u16 class_id, u16 inst);
bool omci_store_has_class(struct omci_onu *o, u16 class_id);
struct omci_me_inst *omci_store_nth(struct omci_onu *o, u16 idx);
bool omci_store_put(struct omci_onu *o, u16 class_id, u16 inst,
		    const u8 *body, int blen);
void omci_store_merge(struct omci_me_inst *e, const u8 *val, int vlen);
void omci_store_del(struct omci_onu *o, u16 class_id, u16 inst);

/* the descriptor-table engine: emit @mask's attributes into [v..end) in
 * descriptor order, reporting what was emitted and what this ME models */
u8 omci_me_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
		u8 *v, const u8 *end, u16 *rmask_out, u16 *known_out);

/* does the model carry this class at all (descriptor row or vendor range)? */
bool omci_class_modelled(u16 class_id);

/* is (class, inst) a MIB instance this ONU holds? */
bool omci_inst_exists(struct omci_onu *o, u16 class_id, u16 inst);

#endif /* GPON_OMCI_ME_H */
