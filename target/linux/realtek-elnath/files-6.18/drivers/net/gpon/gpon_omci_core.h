/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.988 OMCI message layer — interface.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 *
 * gpon_omci_core.h — the ITU-T G.988 OMCI baseline MESSAGE layer, common to
 * every OpenWrt GPON target in this tree.
 *
 * WHAT THIS IS
 *   The wire half of the ONU's OMCI responder: the baseline PDU facts (message
 *   types, result codes, the 48-octet frame, the attribute-mask bit rule), the
 *   parse of a downstream request, the dispatch by message type, and the
 *   construction of the upstream response — trailer and MIC included.
 *   It holds NO managed-entity storage.  The ME model (descriptor table, board
 *   identity blob, MIB-Upload rows) and the dynamic OLT-created instance store
 *   live in the ME-model layer, which this layer CALLS and never inlines.  The
 *   seam is deliberate: G.988 message rules are the same on every ONU ever
 *   built, while which MEs a product serves is per board.
 *
 * WHY IT IS COMMON — and which targets and architectures compile it
 *   Operator, 2026-08-05: "en openwrt debería estar estructurado algo así:
 *   rtl960x* para la familia para tener código común" … "la idea es poner en
 *   común el código que corresponde para no tener mucho duplicado", and on the
 *   two per-target monoliths carrying a private copy each: "mal, poner en
 *   común".
 *   G.988 is a specification, not a chip fact, so exactly ONE copy of it may
 *   exist in the tree.  Compiled by:
 *     - realtek-elnath (RTL9607F, Cortina)   aarch64, LITTLE-endian  — today
 *     - realtek-luna   (RTL960xC, Luna)      MIPS32,  BIG-endian     — after
 *       follow-ups F1/F2/F3 only.  Luna ships a third, independently written
 *       responder (rtl9602c_eth.c) that diverges on the wire from this one;
 *       switching it over CHANGES LUNA'S EMITTED BYTES, so it is a behaviour
 *       change with its own board gate and was deliberately NOT part of the
 *       code-motion refactor that created this file.  Each divergence is
 *       named at the constant it concerns, below and in the .c.
 *     - dev/rtl9607c-test on x86-64 through fuzz_shims/, where it is fuzzed
 *       under ASan+UBSan thousands of cases per second
 *   The prefix is gpon_ and not rtl960x_ on purpose: the layer must also serve
 *   the future ARM OLT and other brands, so a Realtek-named prefix would be
 *   too narrow for what it covers.
 *
 * THE CORE/SHELL RULE IT OBEYS
 *   FUNCTIONAL CORE.  It decides; it never does.  It parses wire bytes, applies
 *   the message rules and fills a caller-provided buffer.  No device pointer,
 *   no lock (the caller serialises), no allocation, no sleeping, no clock read.
 *   Every byte of state lives in the caller-provided struct omci_onu.  The
 *   imperative shell (cortina-gpon.c on Elnath, rtl9602c_eth.c on Luna) owns
 *   the OMCC/GEM binding, the TX ring, the counters and the /proc view.
 *
 *   ⇒ THIS FILE AND ITS .c MUST NEVER GAIN AN MMIO ACCESS.  No readl/writel,
 *   no ioremap, no msleep/udelay, no jiffies, no spin_lock/mutex, no kmalloc,
 *   no dev_ or netdev_ logging.  The purity check greps for exactly that set
 *   and the gate goes red if one appears.  It is not hygiene: that property is
 *   what makes the layer host-compilable, host-fuzzable, and portable across
 *   the two architectures at once.
 *
 * ENDIANNESS
 *   All wire access is explicit byte math — ((u16)p[0] << 8) | p[1] — never a
 *   struct or pointer cast over wire bytes, and never htons/ntohs on a buffer.
 *   That is not style: it is the reason ONE source compiles for big-endian
 *   MIPS and little-endian ARM64 and emits the same octets on both.
 */
#ifndef GPON_OMCI_CORE_H
#define GPON_OMCI_CORE_H

#include <linux/types.h>

#define OMCI_LEN		48	/* baseline message, incl. trailer+MIC */

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
#define OMCI_MT_GET_NEXT	0x1a	/* 26.  DIVERGENCE, follow-up F1: Luna's
					 * rtl9602c_eth.c:1698 defines this as
					 * 0x10, which is the ALARM opcode above —
					 * so Luna answers a real Get Next through
					 * its default arm.  Unfalsifiable on this
					 * OLT (it never sends one); only a
					 * build-time constant extractor catches
					 * it.  Not changed here: Luna is off the
					 * rig and this file did not move Luna. */

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

/* The two ME class IDs the MESSAGE layer itself reasons about — ME 2 because
 * an OLT Set of its attribute 1 is an explicit MIB-Data-Sync resync write, and
 * ME 329 because the VEIP operational-state AVC is a message this layer emits.
 * The full G.988 class registry belongs to the ME-model layer, which defines
 * these two identically; a repeated #define with the same replacement list is
 * a benign redefinition, so this header stays self-sufficient either way. */
#define OMCI_ME_ONU_DATA	2
#define OMCI_ME_VEIP		329

/* Owned by the ME-model layer; the message layer only ever holds a pointer. */
struct omci_onu;

/* Process one DS baseline PDU -> fill @resp (48 bytes, trailer + MIC done).
 * Returns OMCI_LEN, or 0 when no response must be sent (runt / non-baseline /
 * AR clear, i.e. the OLT asked for no acknowledgement).
 *
 * The three zero-returns are NOT interchangeable and the difference is
 * load-bearing: on AR clear the MIB change has ALREADY BEEN APPLIED and only
 * the acknowledgement is suppressed.  Collapsing "no response" into "did
 * nothing" silently desynchronises MIB-Data-Sync.  o->no_ack / o->rx_extended
 * exist so each case stays countable from /proc. */
int omci_onu_input(struct omci_onu *o, const u8 *req, unsigned int len, u8 *resp);

/* Autonomous VEIP (ME 329) operational-state-up AVC: the OLT never polls the
 * data MEs it created — it gates DOWNSTREAM user-data forwarding on this
 * report.  Fills @out (48 bytes, trailer + MIC done); returns OMCI_LEN. */
int omci_onu_emit_veip_up_avc(struct omci_onu *o, u8 *out);

#endif /* GPON_OMCI_CORE_H */
