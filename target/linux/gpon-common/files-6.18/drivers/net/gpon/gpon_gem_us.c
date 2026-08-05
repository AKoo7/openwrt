// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: Upstream GEM port and T-CONT mapping.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 */
/*
 * gpon_gem_us.c — the UPSTREAM half of GEM provisioning, common to every
 * OpenWrt GPON target in this tree.  See gpon_gem_us.h for what this layer is,
 * why it is common, which targets and architectures compile it, and what is
 * deliberately NOT here (the GEM framer: silicon on both shipping targets).
 *
 * WHY IT IS COMMON — operator, 2026-08-05: "la idea es poner en común el código
 * que corresponde para no tener mucho duplicado", and on the two per-target
 * monoliths each carrying their own copy of this decision: "mal, poner en
 * común".  Compiled for aarch64-LE (realtek-elnath), MIPS32-BE (realtek-luna)
 * and x86-64 (dev/rtl9607c-test, through fuzz_shims/).
 *
 * CORE/SHELL: functional core.  It decides; it never does.  No MMIO, no
 * readl/writel, no ioremap, no device pointer, no allocation, no lock, no
 * sleeping, no clock read — and it MUST NEVER GAIN ONE.  The moment it does,
 * the offline suite can no longer drive this area and an upstream mapping
 * question costs a board boot again.
 *
 * PROVENANCE: this file is code MOTION.  Every rule below is already
 * implemented, independently, in the two drivers; the values each target feeds
 * in are exactly the values it feeds in today, so neither target's registers or
 * emitted bytes change.  Where the two genuinely disagree, the disagreement is
 * an INPUT and is named at the site — never averaged, never quietly resolved.
 */

#include <linux/types.h>

#include "gpon_gem_us.h"

bool gpon_gem_us_range_ok(const struct gpon_gem_us_range *r)
{
	if (!r)
		return false;
	return GPON_GEM_US_RANGE_OK(r->base, r->count, r->index_max);
}

/*
 * Alloc-ID -> T-CONT.
 *
 * ★ WHY THIS PREDICATE HAS A FILE TO ITSELF: both targets wrote it separately,
 * and BOTH paid for it.  It is three comparisons; it has cost this project more
 * than anything else of its size.
 *
 *   Luna, the months-long wall.  gpon-rtl9602c.c:6432-6440 records it in the
 *   code that replaced it: the old Assign_Alloc-ID handler bound the OLT's
 *   Alloc-ID to T-CONT 16 — the OMCC's — "OVERWRITING the ONU-ID so the OLT's
 *   default-alloc(=ONU-ID) grants missed the alloc-CAM -> T-CONT16 unreachable
 *   -> gemus64=0 (the months-long wall)".  Fixed 2026-07-03.
 *
 *   Elnath, refusing the same move.  cortina-gpon.c:2190-2196 declines to
 *   re-bind when the data Alloc-ID equals the OMCC's, and names the other
 *   target's outage as its reason: "single-alloc OLT: rebinding the CAM would
 *   steal the OMCC's T-CONT (proven 9602C regression)".
 *
 * Same rule, learned twice, written twice.  Once here.
 *
 * The failure it prevents is silent in the direction that matters: the OMCC
 * keeps looking configured — the CAM has an entry, the flags are set — while
 * the OLT's grants for the management Alloc-ID no longer resolve to any
 * T-CONT, so the ONU simply stops being heard.  Nothing reports an error.
 *
 * ★★ TWO INPUTS THE TWO TARGETS DISAGREE ON.  Both are passed in rather than
 * derived, so this move changes no byte on either.  Both are follow-ups with a
 * board gate, NOT things to fix while moving code:
 *
 *   @omcc_alloc — WHAT IT IS COMPARED AGAINST.
 *     Elnath passes cg->omcc_alloc (cortina-gpon.c:2190), the Alloc-ID actually
 *     bound to hw T-CONT 0 at :2050.
 *     Luna passes gpon_fsm_onu_id (gpon-rtl9602c.c:6441), the live ONU-ID.
 *     Those agree only while Luna's gpon_omcc_alloc override is 0, because
 *     Luna binds T-CONT 16 to `gpon_omcc_alloc ? gpon_omcc_alloc : onu_id`
 *     (:6264).  0 is the shipped default and is documented as the correct one
 *     (:976-987: the OMCC's upstream Alloc-ID IS the ONU-ID, G.984.3 implicit
 *     default), so on a shipped Luna the two are the same value.  With the
 *     override set they are not, and Luna would then bind the OMCC's real
 *     Alloc-ID to the data T-CONT — precisely the overwrite the 2026-07-03 fix
 *     removed.  Passing Luna's own comparand keeps today's behaviour exactly;
 *     making Luna compare against its real T-CONT-16 alloc is a behaviour
 *     change on a target that is currently off the rig.
 *
 *   @already_bound — WHAT "ALREADY DONE" MEANS.
 *     Luna passes gpon_data_tcont_installed (:975, set at :6443) = "this
 *     Alloc-ID is bound to the data T-CONT".
 *     Elnath passes cg->data_installed (set at :2275) = "the whole data path is
 *     armed" — T-CONT bind AND upstream stamps AND the DS CAM entries AND the
 *     PDC route AND the PUC queues.
 *     They hold the same value today because Elnath arms all of it in one
 *     function, but they answer different questions, and a target that ever
 *     splits the two would need to say which one it means here.
 *
 * ★ WHAT THIS FUNCTION DELIBERATELY DOES NOT JUDGE: whether an Alloc-ID exists
 * at all.  "Has the OLT provisioned both halves yet" (ME 262 alloc and ME 268
 * GEM port) is a provisioning-lifecycle gate, and each target already applies
 * its own before reaching here — Elnath at cortina-gpon.c:2187 (`!alloc ||
 * !gem`), Luna implicitly.  Adding a zero-Alloc arm here would CHANGE Luna:
 * it has no such guard, so an Alloc-ID of 0 against a non-zero ONU-ID takes
 * Luna's bind path today and must keep taking it.  Three arms, not four.
 */
enum gpon_gem_us_bind gpon_gem_us_tcont_decide(u16 alloc, u16 omcc_alloc,
					       bool already_bound)
{
	if (already_bound)
		return GPON_GEM_US_BIND_DONE;
	if (alloc == omcc_alloc)
		return GPON_GEM_US_BIND_IS_OMCC;
	return GPON_GEM_US_BIND_TCONT;
}

const char *gpon_gem_us_bind_name(enum gpon_gem_us_bind v)
{
	switch (v) {
	case GPON_GEM_US_BIND_TCONT:
		return "bind-data-tcont";
	case GPON_GEM_US_BIND_DONE:
		return "already-bound";
	case GPON_GEM_US_BIND_IS_OMCC:
		return "is-omcc-alloc";
	}
	/* An unnamed verdict must READ as unknown, never as a plausible one:
	 * a decision printed under the wrong name is worse than an unprinted
	 * decision. */
	return "unknown";
}

/*
 * ★ TWO UPSTREAM FACTS THAT ARE *NOT* HERE, AND WHY — so a later reader does
 * not "finish the job" by adding them.
 *
 * 1. The physical queue / VoQ a T-CONT drains on.  The two families compute it
 *    with different formulas over different quantities:
 *      Luna    gpon-rtl9602c.c:5807-5809 — qid = 32 * (tcont / 8), overridden
 *              to the fixed OMCC qid for T-CONT 16 and for the alt bind.
 *      Elnath  cortina-gpon.c:321, :1222 — voq = tcont * 8 + queue.
 *    A single "portable" formula would have to be wrong on one of them.  It
 *    stays a per-chip fact in each shell.  (The oracle models Luna's family
 *    form as phys_qid() in chip_config.h and notes it is identical across the
 *    two Luna chips — which is exactly why it belongs at the FAMILY tier, not
 *    at this one.)
 *
 * 2. The upstream teardown order.  cortina-gpon.c:2104-2107 names it as a
 *    hardware requirement — drain the T-CONT's VoQs FIRST "so the scheduler
 *    stops draining before the CAM changes underneath it" — and Luna's sequence
 *    is not the same.  A core may REQUEST a teardown; performing one is the
 *    shell's, in its own order.
 */
