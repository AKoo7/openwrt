/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_common.h — THE CONTRACT between the HW-decoupled GPON protocol core and
 * the per-target imperative shell.
 *
 * WHAT THIS IS
 *   The shared types the common GPON layer is built on: the canonical ONU
 *   state encoding, the G.988/G.984 wire enumerations the core decides on, the
 *   per-chip fact block, and `struct gpon_shell_ops` — the table of function
 *   pointers through which the core asks a shell to touch hardware.
 *
 * WHY IT IS COMMON — operator, 2026-08-05:
 *   "en openwrt debería estar estructurado algo así: rtl960x* para la familia
 *    para tener código común" · "la idea es poner en común el código que
 *    corresponde para no tener mucho duplicado" · on the two monoliths: "mal,
 *    poner en común".
 *   Compiled by BOTH OpenWrt targets, on two architectures and two byte orders:
 *     - realtek-luna   — RTL9602C/9603C/9607C, MIPS32 big-endian (Lexra/RLX)
 *     - realtek-elnath — RTL9607F "Elnath" (Cortina), ARM64 little-endian
 *   and additionally on x86 by the differential suite in dev/rtl9607c-test/,
 *   through the host shims in dev/rtl9607c-test/fuzz_shims/. The prefix is
 *   `gpon_` and deliberately NOT `rtl960x_`: this layer must also serve the
 *   future ARM OLT and other vendors' hardware, so a Realtek-named prefix
 *   would be too narrow.
 *
 * ★★ THE THREE TIERS — WHICH FILE MAY HOLD WHAT.  THIS IS THE CANONICAL COPY.
 *   Every other file the 2026-08-05 refactor touched or created carries a
 *   ONE-LINE "TIER:" header pointing here, and nothing more: the rule is
 *   written out once, so it can be corrected once.  Do not restate it.
 *
 *   The project rule it implements: "name each file by the NARROWEST prefix
 *   that matches everything it targets".  Widening a file's prefix without
 *   widening its content is how chip facts end up in a family file.
 *
 *     TIER    prefix                       holds                    may touch HW
 *     ------  ---------------------------  -----------------------  ------------
 *     core    gpon_                        protocol only: the       NO — never
 *                                          PLOAM FSM, G.988 OMCI
 *                                          + ME model, GEM/T-CONT
 *                                          mapping.  Decides.
 *     family  rtl960x_  and  cortina-      one silicon FAMILY's     yes
 *                                          hardware: PON-MAC and
 *                                          SerDes bring-up, the
 *                                          switch/NE datapath
 *     chip    rtl9602c_  rtl9607c_         exactly ONE part's       yes
 *             rtl9607f_  cortina-gpon.c    registers and quirks
 *
 *   WHICH FILE IS WHICH TODAY (the whole set the refactor touched):
 *     core    drivers/net/gpon/gpon_common.h        this file — the contract
 *             drivers/net/gpon/gpon_ploam.[hc]      G.984.3 PLOAM FSM
 *             drivers/net/gpon/gpon_omci_core.[hc]  G.988 message layer
 *             drivers/net/gpon/gpon_omci_me.[hc]    G.988 ME model
 *             drivers/net/gpon/gpon_gem_us.[hc]     US GEM and T-CONT mapping
 *     family  realtek-luna  ... realtek/rtl960x_ponmac.c   PON-MAC, SerDes
 *     chip    realtek-luna  ... realtek/gpon-rtl9602c.c    RTL9602C GPON shell
 *             realtek-luna  ... realtek/rtl9602c_eth.c     RTL9602C NIC shell
 *             realtek-elnath ... cortina/cortina-gpon.c    RTL9607F GPON shell
 *
 *   ⚠ TWO TIER TRAPS ALREADY PAID FOR, both measured on 2026-08-05:
 *     - rtl960x_ponmac.c LOOKS like the family protocol home because of its
 *       prefix, and is not: it implements ZERO of the ops below (measured —
 *       the file contains no PLOAM, GEM, alloc, ONU-ID, EqD, AES or BOH
 *       identifier at all).  It is family HARDWARE, one tier below the shells.
 *     - a core file with no consumer is worse than duplication.  gpon_proto.c
 *       was a previous attempt at this same refactor, left unwired; it drifted
 *       until its code contradicted its own comment, and it was deleted in this
 *       refactor rather than revived.  A gpon_ file that nothing calls is dead
 *       weight in a 3 MB NAND kernel and a fork waiting to happen.
 *
 * THE CORE / SHELL RULE THIS FILE OBEYS (functional core, imperative shell)
 *   CORE  decides. It parses wire bytes, runs the FSM, builds reply buffers and
 *         holds the ME model. It NEVER performs I/O.
 *   SHELL does. It reads and writes registers, arms DMA, services interrupts,
 *         drives the PON MAC — and calls the core through this table.
 *   Three rules follow, and they are what make the core replayable and fuzzable:
 *     1. A side effect leaves as a RETURN VALUE wherever it can. An op exists
 *        only for what a return value cannot express. (This is why there is no
 *        "send" op invoked from inside a decision: a core entry point returns
 *        what it decided and the shell drains it.)
 *     2. TIME IS AN INPUT, never read. Core entry points take an explicit
 *        millisecond/tick argument. No jiffies, no ktime, no timers, no sleeping.
 *     3. Per-chip facts arrive as `const struct gpon_chip_cfg *`, never #ifdef.
 *
 * ★ THIS FILE, AND EVERY gpon_*.[ch] BESIDE IT, MUST NEVER GAIN AN MMIO ACCESS.
 *   No readl/writel/ioremap, no msleep/udelay, no spin_lock/mutex, no kmalloc,
 *   no dev_* or netdev_* or pr_* logging, no jiffies, no schedule_work, and no
 *   struct device.
 *   A shell detail that reaches this header is a defect in the header, not a
 *   fact about the hardware — express it as an op or as a `gpon_chip_cfg`
 *   member.
 *
 *   ★ THE GUARD, BY NAME — AND IT IS A COMPILER, NOT A GREP:
 *       dev/rtl9607c-test/gpon_layer_hostbuild_test.sh --LABEL="<name>"
 *       (suite step 17; run by run_steps.sh and by `make all`)
 *     It compiles every gpon_ source in this directory on the host against the
 *     kernel-type stubs in dev/rtl9607c-test/fuzz_shims, and those stubs are
 *     the enforcement: they deliberately declare NO register accessor, NO
 *     clock, NO lock, NO allocator, NO work/timer and NO net_device.  A core
 *     file that reaches for hardware therefore FAILS TO COMPILE.  It also
 *     refuses to pass vacuously — zero sources found is a FAIL, so "the layer
 *     is pure" and "there is no layer" can never look the same.
 *     Measured 2026-08-05: 5 pass, 0 fail, over 4 core sources.
 *
 *   ⚠ A TOKEN GREP IS THE WRONG INSTRUMENT HERE, AND THE PARAGRAPH ABOVE IS
 *     THE PROOF.  It NAMES every forbidden token, so a raw scan scores 5 on
 *     this file and 0 on its code (2026-08-05: `grep -cE` = 5; the same
 *     pattern after `gcc -fpreprocessed -dD -E` = 0).  Three independent
 *     authors hit this on the same day.  A guard that fails the file which
 *     DOCUMENTS the rule is the "matching TEXT where STRUCTURE is meant"
 *     defect aimed at itself — and the same scan would MISS a real register
 *     access that a stray comment terminator had exposed, which is exactly
 *     what the first compile of this header caught (a `*` followed by `/`
 *     inside `dev_` + `netdev_` closed the block comment early: prose, not
 *     code, and no grep would have found it).
 *
 *   ⚠ `gpon_core_purity_test` IS NAMED BY TWO BUILD SCRIPTS AND DOES NOT
 *     EXIST.  gpon-common/build_wiring_guard.py:24 and
 *     gpon-common/build_wiring_x86_check.sh:18 both defer the purity half to
 *     "gpon_core_purity_test in dev/rtl9607c-test/"; verified 2026-08-05,
 *     there is no such file, target or suite step, and nothing else in the
 *     tree names it.  Do NOT read those references as a check that ran.  It is
 *     OWED work, not coverage: if it is ever written it MUST strip comments
 *     first, per the paragraph above.  Today the compile guard is the coverage.
 *
 * ENDIANNESS
 *   One source serves MIPS-BE and ARM64-LE. Nothing here overlays a struct on
 *   wire bytes; the wire enumerations below are single octets, and multi-octet
 *   fields are assembled by the cores with explicit byte math ((d[1]<<8)|d[2]).
 *   Never add a packed struct, a pointer cast over a wire buffer, or an
 *   htons/ntohs to this layer.
 *
 * PROVENANCE
 *   Written for the plan at dev/sdk-facts/00-COMMON-LAYER-PLAN.md (2026-08-05).
 *   Every op below was verified against the shell function that implements it
 *   today; the file:line evidence is quoted per member. Where the plan's
 *   proposed shape did not match the measured source, the measurement wins and
 *   the divergence is named in the comment.
 */
#ifndef GPON_COMMON_H
#define GPON_COMMON_H

#include <linux/types.h>

/*
 * `fallthrough` for the HOST builds only.
 *
 * In kernel context this is already defined: compiler_attributes.h is force
 * -included by the kernel build, so the #ifndef below is false and the kernel's
 * own definition stands untouched. The host builds are what need it — neither
 * the offline fuzz shims (dev/rtl9607c-test/fuzz_shims) nor a plain libc
 * provide it, and this layer must compile on all three toolchains.
 *
 * ★ WHY A COMMENT WILL NOT DO, measured 2026-08-05: the kernel builds this tree
 *   with -Werror -Wimplicit-fallthrough=5 (read from the flags the build itself
 *   recorded in scripts/mod/.empty.o.cmd, both targets). At level 5 ONLY the
 *   attribute suppresses the diagnostic; NO comment spelling ever does. A
 *   "fall through" written as a COMMENT therefore builds on the host and is a
 *   hard error on both targets, which is exactly the divergence this macro
 *   closes. Spelled as the kernel spells it, so the two cannot drift.
 *   (That sentence deliberately does not quote the comment delimiters: a stray
 *   terminator inside prose closes the block early and turns the rest of the
 *   text into code, which is a real bug this layer has already had once.)
 */
#ifndef fallthrough
# if defined(__GNUC__) && __GNUC__ >= 7
#  define fallthrough	__attribute__((__fallthrough__))
# else
#  define fallthrough	do {} while (0)
# endif
#endif

/*
 * ---------------------------------------------------------------------------
 * G.984 wire constants the core decides on (protocol facts, not chip facts)
 * ---------------------------------------------------------------------------
 */

/* Broadcast/unassigned ONU-ID. G.984.3: the ONU uses 0xff until the OLT's
 * Assign_ONU-ID lands, and every re-range path restores it. */
#define GPON_ONU_ID_BROADCAST	0xffu

/* The G.984 broadcast GEM port-id. Both targets already carry this number in
 * their own spelling — realtek-luna GPON_MCAST_GEM 0xfff
 * (rtl9602c_gpon_nic.h:67) and realtek-elnath CG_MCAST_GEM_ID 4095
 * (cortina-gpon.c:323) — so unifying it is safe today: it is one G.984 value
 * written twice, not a per-chip difference. */
#define GPON_MCAST_GEM_PORT	4095u

/* Octets of an upstream PLOAM message the core hands to ->ploam_tx().
 * G.984.3 Table 9-3: ONU-ID[0] + Message-ID[1] + 10 message octets = 12; the
 * 13th octet (CRC) is appended by silicon, so it is NOT part of the buffer.
 * Matches the only implementation today, gpon-rtl9602c.c:5041
 * gpon_send_cpu_ploam(u8 queue, const u8 m[12]). */
#define GPON_PLOAM_US_LEN	12u

/*
 * Canonical ONU activation state. G.984.3 clause 10 names, 1-based.
 *
 * ★ THE TWO SILICONS GENUINELY DISAGREE AND NEITHER DRIVER IS WRONG, so this
 *   enum is the core's OWN encoding and each shell maps its hardware field to
 *   it at the boundary (plan D-6):
 *     - realtek-luna is 1-BASED and matches this enum directly
 *       (gpon-rtl9602c.c GPON_ONU_STATE_MASK 0xf, gpon_onu_state_name[1] =
 *       "O1-initial", and the driver's own note "the HW ONU_STATE field uses
 *       the same 1-based encoding");
 *     - realtek-elnath is 0-BASED (cortina-gpon.c CG_STATE_RANGING 3,
 *       CG_STATE_OPERATION 4, cg_state_name[0] = "O1-Initial").
 *   ⚠ Do NOT adopt the host oracle's enum (dev/rtl9607c-oracle/ploam_fsm.h:22).
 *   Its names are SHIFTED — it calls state 1 "GPON_O1_STANDBY" when O1 is
 *   Initial and O2 is Standby per G.984.3. The oracle is wrong here and both
 *   drivers are right; the correction is follow-up F17. The oracle stays an
 *   INDEPENDENT implementation and is never included from the kernel tree.
 *   ⚠ Mapping to this enum must not change either target's LOG STRINGS — those
 *   stay shell-side, byte-for-byte (plan, carve-eln-shell B7).
 */
enum gpon_ostate {
	GPON_O1_INITIAL		= 1,	/* no downstream sync                */
	GPON_O2_STANDBY		= 2,	/* DS sync, awaiting parameters      */
	GPON_O3_SERIAL		= 3,	/* serial-number acquisition         */
	GPON_O4_RANGING		= 4,	/* ranging (EqD assignment)          */
	GPON_O5_OPERATION	= 5,	/* operational                       */
	GPON_O6_POPUP		= 6,	/* intermittent LOS/LOF, popup       */
	GPON_O7_EMERGENCY	= 7,	/* emergency stop (SN disabled)      */
};

/*
 * ME 268 (GEM port network CTP) "direction" attribute, as it arrives on the
 * wire in the Set-by-Create body.
 *
 * ★ CORRECTED AGAINST THE PLAN. 00-COMMON-LAYER-PLAN.md §3 declares
 *   { GPON_GEM_DS = 1, GPON_GEM_US = 2, GPON_GEM_BIDIR = 3 } — US and DS are
 *   SWAPPED there. Two independent sources agree on the values below:
 *     - G.988 clause 9.2.3, "Direction": 1 = upstream, 2 = downstream,
 *       3 = bidirectional;
 *     - the live driver's own comment, cortina-gpon.c:2756: "direction[4]
 *       (1=US, 2=DS, 3=bidirectional)".
 *   No behaviour depends on the 1-vs-2 naming today — cortina-gpon.c:2766/2774
 *   only tests `== 3` / `!= 3` for the bidirectional-only data-GEM election —
 *   so this is a naming defect caught before it could be relied upon, not a
 *   live bug. It is fixed here because a wrong name in the CONTRACT is exactly
 *   the class of defect this project has already paid for twice (the `mcgid`
 *   name trap, cortina-ni-flowoffload.c:2156; mt_name[5]="Delete", F14).
 */
enum gpon_gem_dir {
	GPON_GEM_US	= 1,	/* upstream only   (UNI -> ANI) */
	GPON_GEM_DS	= 2,	/* downstream only (ANI -> UNI) */
	GPON_GEM_BIDIR	= 3,	/* bidirectional — THE data GEM */
};

/*
 * Which upstream PLOAM queue a decision wants its message on.
 *
 * This is an ABSTRACT urgency class, not a register value: the shell maps it to
 * its own silicon. The class must be carried because it is a CORE decision made
 * per message and the three cases are not interchangeable — realtek-luna
 * selects all three at distinct points of the FSM (gpon-rtl9602c.c:5107 SN,
 * :5132 Password, :5154 Acknowledge, :5217 Encryption_Key, :6120/:6853/:7410
 * the idle No_message).
 *
 * ⚠ The plan's §3 op-table declares `ploam_tx(void *sh, const u8 msg[12])` with
 *   NO queue argument. Dropping it would put every upstream PLOAM on one queue
 *   and CHANGE LUNA'S BEHAVIOUR — this is a code-motion refactor, so the
 *   argument stays. The concrete RTL9602C mapping (US_PLOAM_IND[10:8]: SN 0x6,
 *   urgent 0x1, No_message 0x7 — gpon-rtl9602c.c:4995-4997) is a chip fact and
 *   MUST NOT appear in common code.
 */
enum gpon_ploam_q {
	GPON_PLOAM_Q_SN,	/* Serial_Number_ONU: the auto-SN slot        */
	GPON_PLOAM_Q_URGENT,	/* Acknowledge, Password, Encryption_Key      */
	GPON_PLOAM_Q_NOMSG,	/* the idle No_message keepalive slot         */
};

/*
 * Which CRC-32 an OMCI MIC (bytes 44..47) is computed with.
 *
 * ★ THE TWO TARGETS EMIT DIFFERENT BYTES AND THE EVIDENCE DOES NOT SETTLE IT
 *   (plan D-5, follow-up F3). realtek-elnath computes ~crc32_be(~0, msg, 44) —
 *   the non-reflected I.363.5/AAL5 CRC-32, which is what G.984.4 specifies and
 *   which is LIVE-PROVEN against this lab's OLT. realtek-luna computes
 *   crc32_le(~0, msg, 44) ^ ~0, the reflected zlib CRC-32 — and reaches O5 and
 *   provisions on that SAME OLT, which nothing in the tree explains. The host
 *   oracle computes no MIC at all and therefore cannot arbitrate.
 *   ⇒ Neither variant is chosen here. This selector exists so that when Luna
 *   joins the common responder it keeps emitting the bytes it emits today.
 *   F3 names the measurement that settles it: capture the X111W upstream OMCI
 *   and compare bytes 44..47 against both variants.
 */
enum gpon_mic_variant {
	GPON_MIC_CRC32_BE_AAL5,	/* ~crc32_be(~0,m,44)      — G.984.4, elnath */
	GPON_MIC_CRC32_LE_ZLIB,	/* crc32_le(~0,m,44) ^ ~0  — luna today      */
};

/*
 * ---------------------------------------------------------------------------
 * Per-chip facts
 * ---------------------------------------------------------------------------
 *
 * A member belongs here if and only if BOTH hold:
 *   (a) the two silicons are MEASURED to differ on it, and
 *   (b) the CORE must know it — i.e. the shell cannot simply pass it as a call
 *       argument.
 * Test (b) is what keeps this struct from becoming a dumping ground, and it is
 * why several values the plan listed are NOT here:
 *   - `data_gem_port` (luna's compile-time GPON_DATA_GEM 193 vs elnath's
 *     snooped id) is an ARGUMENT to the data install, not a core fact
 *     (carve-eln-gem §5.4);
 *   - `gem_flow_base` / `gem_flow_count` are shell-internal tokens. The
 *     internal GEM index is arithmetically tied to elnath's PUC VoQ map
 *     (CG_DATA_GEM_IDX = CG_DATA_TCONT_IDX * 8, cortina-gpon.c:321) and is a
 *     flat GTC constant on luna (GPON_OMCC_FLOW 64, GPON_DATA_FLOW 1). Common
 *     code that computes a flow is wrong on one of the two targets, so the core
 *     never sees one (carve-eln-gem §5.2);
 *   - `omcc_tcont` / `data_tcont` are silicon T-CONT INDICES the shell writes
 *     into a CAM (luna 16/8; elnath 0/1, cortina-gpon.c:2044/:320). The core
 *     reasons about ALLOC-IDs, which are wire identities, not about indices.
 *
 * ⚠ NOTHING READS THIS STRUCT YET, AND SETTING IT TODAY CHANGES NOTHING.
 *   The common OMCI responder currently hardcodes the conformant values
 *   (gpon_omci_core.c:93 crc32_be; gpon_omci_core.h:90 OMCI_MT_GET_NEXT 0x1a),
 *   because plan D-3 rules that realtek-luna does NOT switch to the common
 *   responder in this refactor — doing so would change its wire bytes on a
 *   board that is off the rig. These members are the hook that lets it switch
 *   later WITHOUT changing those bytes, and each is consumed by the follow-up
 *   named on its line. Until then, a target needing a non-default value must
 *   not use the common responder. Do not add a member here without the code
 *   that reads it: a knob that silently does nothing is this project's most
 *   expensive recurring defect.
 */
struct gpon_chip_cfg {
	/* F3 — the unresolved MIC polynomial. See enum gpon_mic_variant. */
	enum gpon_mic_variant	mic;

	/* F2 — one-past-the-end offset of the GET response value area.
	 * 36 is conformant (25 octets: G.988 reserves 36-37 optional-attribute
	 * mask and 38-39 attribute-execution mask ALWAYS, success included);
	 * the host oracle agrees (omci_msg.h:53 OMCI_GET_VAL_END 36u).
	 * realtek-luna writes to 40 (29 octets, rtl9602c_eth.c:2265), i.e. it
	 * overwrites both masks. */
	u16			get_val_end;

	/* F1 — the Get-Next message type. 0x1a (26) is conformant per G.988
	 * Table 11.2.2-1 and matches the oracle (omci_msg.h:108).
	 * realtek-luna uses 0x10 (rtl9602c_eth.c:1698), which is the ALARM
	 * opcode, so its real Get-Next falls through to NOT_SUPPORTED. The
	 * divergence is unfalsifiable on the wire; only a build-time constant
	 * extractor catches it. */
	u8			mt_get_next;
};

/*
 * ---------------------------------------------------------------------------
 * The op table — everything the core cannot do for itself
 * ---------------------------------------------------------------------------
 *
 * @sh is the shell's opaque handle (realtek-elnath passes its `struct
 * cortina_gpon *`; realtek-luna passes NULL while its state is still
 * file-scope). It is void * so that no target's device struct can reach this
 * header.
 *
 * CALL CONTEXT — two classes, and mixing them is the single largest
 * correctness risk in the whole carve (carve-eln-gem §5.1):
 *   *_decide() core entry points may be called from SOFTIRQ. They must not
 *             sleep and must not take a lock. On realtek-elnath the OMCI snoop
 *             runs in NAPI softirq and publishes to the work context with
 *             WRITE_ONCE/READ_ONCE and no lock at all.
 *   *_apply()  core entry points run in PROCESS context only, and are the ONLY
 *             callers of an op that may block, poll or time out.
 * ⇒ Putting a mutex in common state would break the softirq caller
 *   (might_sleep); a spinlock would be a behaviour change. Neither is done.
 *
 * ORDERING FACTS THE CORE MAY REQUEST BUT MUST NEVER PERFORM:
 *   - teardown ORDER is a hardware fact. realtek-elnath must drain the VoQ
 *     before the CAM moves under the PUC scheduler (cortina-gpon.c:2113
 *     cg_data_teardown, the vendor drain-then-clear order); realtek-luna's
 *     sequence differs. The core says "tear down"; the shell decides how.
 *   - the deferred-work kick fires from INSIDE realtek-elnath's snoop
 *     (cortina-gpon.c:2751/:2773/:2794) BEFORE the responder runs at :2809.
 *     That position is load-bearing and stays shell-side, unchanged.
 *
 * ERROR CONVENTION: an int-returning op returns 0 on success or a negative
 * errno. -ETIMEDOUT is REACHABLE and the core must handle it — every hardware
 * table write on realtek-elnath goes through cg_tbl_op's bounded completion
 * poll (cortina-gpon.c:1977), and realtek-luna's CAM writes poll likewise. A
 * core that assumes an install succeeded leaves silicon carrying a binding its
 * shadow does not record.
 *
 * A NULL op means "this target does not have that hardware step" and the core
 * must skip it, not fail. This is not hypothetical: realtek-luna has no
 * downstream-GEM CAM teardown at all (its stale-CAM story is a re-arm flag),
 * so ->data_teardown is NULL there (carve-eln-gem §5.4).
 */
struct gpon_shell_ops {
	/* --- PLOAM activation ---------------------------------------------
	 * One consumer today (realtek-luna). realtek-elnath has NO software
	 * PLOAM FSM: its MAC runs O1->O5 and auto-acknowledges in silicon
	 * (measured — plan R2). These ops are justified by the roadmap (the
	 * ARM OLT, the offline adversarial fuzzer) and by offline testability,
	 * NOT by de-duplication. Saying so plainly is the point.
	 */

	/* Queue one upstream PLOAM. @msg is GPON_PLOAM_US_LEN octets; silicon
	 * appends the CRC.  <- gpon-rtl9602c.c:5041 gpon_send_cpu_ploam() */
	void (*ploam_tx)(void *sh, enum gpon_ploam_q q,
			 const u8 msg[GPON_PLOAM_US_LEN]);

	/* Publish the activation state into the GTC status field.
	 * <- gpon-rtl9602c.c:6068 gpon_fsm_set_state() / :6137 */
	void (*set_hw_state)(void *sh, enum gpon_ostate st);

	/* Publish the OLT-assigned ONU-ID (GPON_ONU_ID_BROADCAST on every
	 * teardown). Writes BOTH the downstream status copy and the upstream
	 * copy on realtek-luna, which is exactly why it is one op and not two
	 * register writes the core could get half-right.
	 * <- gpon-rtl9602c.c:6235-6236 (assign), :6344-6345, :6573-6574,
	 *    :6662-6663, :6711-6712 (the four teardown paths) */
	void (*set_hw_onu_id)(void *sh, u8 onu_id);

	/* Program the upstream equalization delay, in bits, as the OLT's
	 * Ranging_Time gave it.  <- gpon-rtl9602c.c:6017 gpon_set_eqd()
	 * ⚠ COARSER THAN THE PLAN, deliberately. The plan's §3 table declares a
	 *   `get_min_delay()` op so the core can do the EqD arithmetic itself.
	 *   Measured, that arithmetic is three CHIP facts to one protocol fact:
	 *   it reads the MIN_DELAY1 timing field, scales it by the silicon's
	 *   128-bit unit, and packs the result into [26:24]+[17:0] of one
	 *   register. Moving it into the core would drag all three into common
	 *   code for no protocol gain, and this step must stay bit-identical.
	 *   The read therefore stays inside the shell, and `get_min_delay` is
	 *   NOT declared — an op with no caller cannot be verified. If a later
	 *   step does move the arithmetic, it adds the op together with the
	 *   code that calls it, under its own gate. */
	void (*set_eqd)(void *sh, u32 eqd_bits);

	/* Apply the burst-overhead parameters the core computed (preamble,
	 * delimiter, guard) for the pre-ranged or ranged case.
	 * <- gpon-rtl9602c.c:5947 gpon_apply_boh()
	 * (Absent from the plan's op table; it is a real shell entry the FSM
	 * calls, and the dead gpon_proto.h:73 already declared it.) */
	void (*apply_boh)(void *sh, bool ranged);

	/* Re-lock the transmit analog at O3 entry: the cold-start TX-CMU/PLL
	 * relock without which a cold boot never ranges while a warm reboot
	 * does.  <- gpon-rtl9602c.c:6053 gpon_txpll_relock()
	 *    cf. cortina-gpon.c:1746 cg_psds_relock()
	 * ⚠ void, not int as the plan declares: BOTH implementations return
	 *   void, and an int nobody sets is a return value the core would be
	 *   entitled to branch on. */
	void (*analog_relock)(void *sh);

	/* Load a 128-bit AES key into the STAGED key bank. The core generates
	 * the key (via ->rng) and answers Request_Key with it; the hardware
	 * promotes staged->active at the switch time below.
	 * <- gpon-rtl9602c.c:5180 gpon_aes_stage_key() */
	void (*aes_stage_key)(void *sh, const u8 key[16]);

	/* Arm the superframe counter at which the staged key is promoted, from
	 * the OLT's Key_Switching_Time.  <- gpon-rtl9602c.c:6499
	 * (Absent from the plan's op table; without it the KEY_SWITCH case
	 * cannot leave the core, and an un-armed switch corrupts AES.) */
	void (*aes_set_switch_time)(void *sh, u32 superframe);

	/* Fill @out with @len random octets. INJECTED rather than called
	 * directly so that the core stays deterministic under replay and
	 * fuzzing — the whole reason the key path can be tested offline at all.
	 * <- gpon-rtl9602c.c:5203 get_random_bytes() */
	void (*rng)(void *sh, u8 *out, unsigned int len);

	/* --- GEM / T-CONT provisioning -------------------------------------
	 * Two consumers. THE GRANULARITY IS "INSTALL", NEVER "WRITE THE CAM
	 * WORD" — this is the one place the plan's proposed table had to be
	 * re-cut, and the evidence is unambiguous:
	 *   - realtek-elnath's data install is ONE hardware transaction with
	 *     seven early returns spanning the T-CONT CAM, eight upstream port
	 *     stamps, two downstream CAM entries, two PDC map writes, a PUC
	 *     enable and a VoQ flush (cortina-gpon.c:2171-2287). Decomposing it
	 *     into four core-driven ops WOULD reorder it — a behaviour change,
	 *     and one whose failure mode is a live silicon binding with no
	 *     shadow record (recorded defect F13, not fixed here).
	 *   - a fine-grained op keyed on a `flow` would force the core to know
	 *     an internal index that is computed from the VoQ map on one target
	 *     and is a flat constant on the other (carve-eln-gem §5.2).
	 *   - carve-luna-gem B3 reached the same conclusion from the other
	 *     side: "the op is 'install', never 'write the CAM word'."
	 * The core therefore names WIRE IDENTITIES (alloc-id, GEM port-id) and
	 * the shell owns every silicon index. @gem is a GEM PORT-ID as it
	 * appears on the wire — never an internal GEM index, and never an
	 * `mcgid`, which is a third, unrelated 12-bit egress port-group that has
	 * already cost one board-proven regression.
	 */

	/* Bind the OMCC: the management alloc-id and its GEM port-id.
	 * <- gpon-rtl9602c.c:5420 gpon_install_omcc() + :5765 gpon_install_tcont()
	 *    cortina-gpon.c:2027 cg_omcc_tcont_bind() + :2063 cg_omcc_gem_bind() */
	int (*omcc_install)(void *sh, u16 alloc, u16 gem);

	/* Bind the user-data path: the OLT-provisioned alloc-id and data GEM
	 * port-id, as snooped from ME 262 / ME 268.
	 * <- gpon-rtl9602c.c:5609 gpon_install_data_gem() + :5765
	 *    cortina-gpon.c:2171 cg_data_try_install() */
	int (*data_install)(void *sh, u16 alloc, u16 gem);

	/* Tear the user-data path down, in whatever order that silicon
	 * requires. NULL where a target has no teardown (realtek-luna).
	 * <- cortina-gpon.c:2113 cg_data_teardown() */
	void (*data_teardown)(void *sh);

	/* --- OMCI ----------------------------------------------------------- */

	/* Transmit one OMCI PDU upstream. @len is carried even though baseline
	 * OMCI is always OMCI_LEN: an implicit length is precisely what makes a
	 * buffer unfuzzable, and the extended (devid 0x0b) case is on the
	 * roadmap.  <- cortina-gpon.c:2393 cg_omci_tx() */
	int (*omci_tx)(void *sh, const u8 *pdu, unsigned int len);

	/* --- diagnostics ---------------------------------------------------
	 * NEVER load-bearing. A NULL ->trace must change no decision the core
	 * makes and no byte it emits; it exists because the project requires
	 * spy/dump capability to be first-class rather than bolted on.
	 * ⚠ It is NOT the mechanism for preserving a target's existing log
	 *   lines. Those carry register reads and cross-driver counters inside
	 *   their argument lists (gpon-rtl9602c.c:6327-6333 and three more), so
	 *   they stay in the shell and must be replayed there in the same order
	 *   — otherwise "bit-identical" quietly stops being true
	 *   (carve-luna-ploam B4, carve-eln-shell B2).
	 */
	void (*trace)(void *sh, unsigned int ev, u32 a, u32 b);
};

/*
 * ---------------------------------------------------------------------------
 * How a core holds its state
 * ---------------------------------------------------------------------------
 * Each core owns a CONTEXT STRUCT carrying at least { ops, sh, chip } plus its
 * own state, and takes a pointer to it on every entry point. NEVER file-scope
 * globals.
 *
 * This is not style. The abandoned first attempt at this refactor
 * (realtek-luna gpon_proto.c, dead since 2026-06-18, deleted by plan step M2)
 * kept its FSM state in ~11 externs, and that is what made it impossible to run
 * side by side with the host oracle for a differential comparison — you cannot
 * instantiate two of it. It also reached "hardware freedom" by DELETING the
 * register writes, the udelay and the deferred-work kick from the code it
 * copied, rather than turning them into ops. Both mistakes are why it drifted
 * into a plausible-looking wrong twin that still carries the Assign_Alloc-ID
 * regression this project spent months paying for. Do not repeat either.
 */
struct gpon_core_base {
	const struct gpon_shell_ops	*ops;
	void				*sh;	/* opaque shell handle       */
	const struct gpon_chip_cfg	*chip;
};

#endif /* GPON_COMMON_H */
