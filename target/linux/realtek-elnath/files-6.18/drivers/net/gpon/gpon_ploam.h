/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.984.3 PLOAM activation FSM — interface.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 */
/*
 * gpon_ploam.h — G.984.3 PLOAM activation state machine (O1..O7): the
 *                HARDWARE-DECOUPLED protocol core.
 *
 * WHAT THIS IS
 *   The ONU-side PLOAM engine: downstream message dispatch, the O1->O5
 *   activation transitions, the upstream message BUILDERS (Serial_Number,
 *   Password, Acknowledge, Encryption_Key, No_message), the burst-overhead
 *   (BOH) computation and the equalization-delay (EqD) computation, plus the
 *   periodic-poll DECISIONS (SN re-offer cadence, O5 keepalive cadence, the
 *   downstream-LOS re-range debounce, the O5 provisioning watchdog).
 *
 *   Everything here is pure byte math over the 13-byte wire message and pure
 *   arithmetic over counters. Every SIDE EFFECT leaves through
 *   `struct gpon_ploam_ops`.
 *
 * WHY IT IS COMMON  (operator, 2026-08-05: "en openwrt debería estar
 *   estructurado algo así: rtl960x* para la familia para tener código común"
 *   … "la idea es poner en común el código que corresponde para no tener mucho
 *   duplicado")
 *
 *   Built from `target/linux/gpon-common/files-6.18/`, which BOTH OpenWrt
 *   targets pull in through `FILES_DIR +=`:
 *       realtek-luna    MIPS32 BIG-endian    (RTL9602C / 9603C / 9607C, Luna)
 *       realtek-elnath  ARM64 LITTLE-endian  (RTL9607F, Cortina NE)
 *   and it also compiles for x86-64 in `dev/rtl9607c-test/`, where it is driven
 *   by the ordered differential suite against the INDEPENDENT oracle in
 *   `dev/rtl9607c-oracle/`. One source, three endianness/word-size
 *   combinations: all wire-field extraction is explicit byte math
 *   (`((u32)d[1] << 24) | ...`), never a struct or pointer cast over wire
 *   bytes, never htons()/ntohs().
 *
 *   ★ HONEST SCOPE, MEASURED 2026-08-05, and it is not what the refactor brief
 *   assumed: this file has exactly ONE kernel consumer today. Elnath has NO
 *   software PLOAM FSM — its MAC runs O1->O5 and auto-ACKs in silicon
 *   (`cortina-gpon.c:1955`); a grep for a PLOAM dispatch there returns zero.
 *   So this file is NOT justified by de-duplication. It is justified by
 *     (a) offline adversarial testing — the FSM becomes fuzzable on x86 under
 *         libFuzzer+ASan+UBSan with an adversarial OLT, with no board in the
 *         loop, which is this project's PRIMARY correctness gate;
 *     (b) the roadmap — the future ARM OLT and other ONU brands need this same
 *         engine, and a chip whose MAC does NOT run PLOAM in silicon needs it
 *         verbatim.
 *   Saying so plainly is the point; claiming a saving that does not exist would
 *   be the same reassuring-direction reporting this project keeps paying for.
 *
 * THE CORE / SHELL RULE THIS FILE OBEYS
 *   CORE (here): DECIDES. Parses wire bytes, runs the FSM, builds reply
 *                buffers, computes BOH/EqD words, owns the activation state.
 *   SHELL (per target): DOES. Reads/writes registers, arms DMA, services IRQs,
 *                drives the PON MAC — and calls this core.
 *
 *   ★ THIS FILE MUST NEVER GAIN AN MMIO ACCESS. No readl/writel, no ioremap,
 *   no `struct device`, no jiffies, no timers, no msleep/udelay, no locks, no
 *   allocation, no printk. TIME IS AN EXPLICIT INPUT (`now_ms`) and the tick
 *   counter is advanced by the shell; a hardware read needed mid-decision is an
 *   op (`get_min_delay`), never a register touch. The offline gate G1 greps for
 *   exactly those tokens over every gpon_*.c/h and must score 0 — the guard is
 *   mutation-proven by adding one readl() and confirming the gate goes red.
 *
 *   This is also the specific way the previous attempt at this refactor failed:
 *   the dead `gpon_proto.c` reached "HW freedom" by DELETING the 8 MMIO calls,
 *   the udelay(500) and the schedule_work() from inside the handler, and it
 *   kept file-scope globals. Both mistakes are structurally impossible here:
 *   every removed effect is an op, and all state lives in `struct gpon_ploam`.
 *
 * PROVENANCE
 *   Code motion out of the Luna monolith
 *   `realtek-luna/files-6.18/drivers/net/ethernet/realtek/gpon-rtl9602c.c`,
 *   which is at stock parity. Each block below names the source line range it
 *   came from. Behaviour is intended to be BIT-IDENTICAL: where Elnath does
 *   something different, or where a latent defect was found while moving, the
 *   code is moved UNCHANGED and the divergence is recorded as a follow-up in
 *   the comment at that site (see gpon_ploam.c "FOLLOW-UPS").
 */
#ifndef GPON_PLOAM_H
#define GPON_PLOAM_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/*
 * ★ TEMPORARY COMPATIBILITY SHIM — IT BELONGS IN gpon_common.h, NOT HERE.
 *
 * gpon_common.h includes <linux/types.h> UNCONDITIONALLY and uses u8/u16/u32
 * throughout, so as it stands the common layer is kernel-only. That is not a
 * cosmetic problem: compiling on x86 is the entire justification for this layer
 * (offline adversarial fuzzing is this project's PRIMARY correctness gate, and
 * gates G2 and G3 both require a host build), so a kernel-only "common" layer
 * cannot be tested the way the plan says it must be.
 *
 * ★ AND IT FAILS IN THE REASSURING DIRECTION, which is why it survived: on a
 * glibc host <linux/types.h> RESOLVES — to the UAPI copy — so the include
 * succeeds and the file looks portable. The UAPI header defines __u8/__u32 but
 * NOT the kernel-internal u8/u16/u32, so the build dies far below, on the first
 * struct member, and reads like a typo rather than a missing host branch.
 *
 * Defining the three types here keeps THIS file buildable on all three
 * toolchains without editing a neighbouring file. Delete this block the moment
 * gpon_common.h grows its own `#ifndef __KERNEL__` branch; a duplicate typedef
 * of the identical type is legal C11, so the two can coexist meanwhile.
 */
#ifndef GPON_HOST_TYPES_DEFINED
#define GPON_HOST_TYPES_DEFINED
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif
#endif /* !__KERNEL__ */

#include "gpon_common.h"	/* enum gpon_ostate — the canonical G.984.3 states */

/* ---------------------------------------------------------------------------
 * G.984.3 PLOAM message types and US queue selectors.
 * Moved verbatim from gpon-rtl9602c.c:4977-4997.
 * These are wire FACTS from ITU-T G.984.3 §9.2.3, not vendor expression.
 * ------------------------------------------------------------------------- */
#define PLM_DS_UPSTREAM_OVERHEAD	0x01
#define PLM_DS_ASSIGN_ONU_ID		0x03
#define PLM_DS_RANGING_TIME		0x04
#define PLM_DS_DEACTIVATE_ONU		0x05
#define PLM_DS_DISABLE_SN		0x06
#define PLM_DS_EXT_BURST_LENGTH		0x14
#define PLM_DS_ENCRYPT_PORT		0x08	/* Encrypted_Port-ID (ACK) */
#define PLM_DS_REQUEST_KEY		0x0d	/* Request_key (-> Encryption_Key) */
#define PLM_DS_KEY_SWITCH		0x13	/* Key_Switching_Time (-> arm HW key switch) */
#define PLM_DS_ASSIGN_ALLOC_ID		0x0a	/* Assign_Alloc-ID (ACK) */
#define PLM_DS_REQUEST_PASSWORD		0x09	/* Request_Password (-> US Password 0x02) */
#define PLM_DS_CONFIG_PORT		0x0e	/* Configure_Port-ID (ACK) */
#define PLM_DS_CFG_VPVC			0x07	/* Configure_VP/VC (ATM, unsupported; ACK) */
#define PLM_DS_BER_INTERVAL		0x12	/* BER interval (ACK; arms BER reporting) */
#define PLM_US_ENCRYPT_KEY		0x05	/* US Encryption_Key response */
#define PLM_US_PASSWORD			0x02	/* US Password response (to Request_Password) */
#define PLM_US_SERIAL_NUMBER		0x01
#define PLM_US_ACKNOWLEDGE		0x09	/* US Acknowledge message type */
/* The No_message type was a bare 0x04 literal at gpon-rtl9602c.c:6119 and
 * :6852, commented "GPON_PLOAM_US_NOMESSAGE". Named here; same byte. */
#define PLM_US_NO_MESSAGE		0x04
#define PLM_US_QUEUE_SN			0x6	/* US_PLOAM_IND[10:8] auto-SN queue */
#define PLM_US_QUEUE_URG		0x1	/* US_PLOAM_IND[10:8] urgent queue (ACKs) */
#define PLM_US_QUEUE_NOMSG		0x7	/* US_PLOAM_IND[10:8] HW auto-No_message slot */

/* A DS PLOAM is 13 bytes: [0]=ONU-ID [1]=type [2..11]=data [12]=CRC-8.
 * A US PLOAM is 12 bytes; the hardware appends the CRC.
 *
 * gpon_common.h owns these — it is the contract, and it already spells
 * GPON_PLOAM_US_LEN as 12u. Guarded rather than duplicated so the two headers
 * agree whichever is included first and so a kernel build with -Werror is not
 * broken by a redefinition warning over a suffix. Same value either way. */
#ifndef GPON_PLOAM_DS_LEN
#define GPON_PLOAM_DS_LEN		13u
#endif
#ifndef GPON_PLOAM_US_LEN
#define GPON_PLOAM_US_LEN		12u
#endif

/* Burst-overhead geometry (G.984.3 upstream overhead + the BOH_DATA depth).
 * 12 = TOTAL_OVERHEAD_BITS(96)/8 stored bytes, which the hardware extends out
 * to BOH_LENGTH via the REPEAT pointer; 252 is the BOH_LENGTH field cap.
 * FOLLOW-UP: on a chip whose BOH_DATA array is not 12 deep these become
 * members of a per-chip config; today both values are identical on every
 * RTL960x and no second consumer exists to disagree. */
#define GPON_PLOAM_BOH_LEN		12
#define GPON_PLOAM_BOH_MAX_LEN		252

/* One upstream GTC frame in bits (G.984.3: 19440 octets at 1.24416 Gbit/s).
 * A specification constant, not a register. */
#define GPON_PLOAM_EQD_FRAME_LEN	(19440 * 8)

/* ---------------------------------------------------------------------------
 * Diagnostic events. The core NEVER formats a string; it reports what happened
 * and the shell renders it with whatever counters it wants to read — which is
 * exactly how the seven-counter DEACT line, the three-counter Request_Key line
 * and their siblings keep their MMIO reads OUT of this file.
 *
 * ★ `trace` is NEVER load-bearing: a NULL op changes no decision, and no core
 * branch may ever depend on it. Everything a renderer needs beyond (a, b) it
 * reads from the `struct gpon_ploam *` it was handed at init.
 * ------------------------------------------------------------------------- */
enum gpon_ploam_ev {
	GPON_PLOAM_EV_STATE,		/* a = previous O-state, b = new O-state   */
	GPON_PLOAM_EV_RERANGE_DONE,	/* a = re-range count, b = outage ms       */
	GPON_PLOAM_EV_BOH,		/* a = ranged?, b = BOH_CFG word           */
	GPON_PLOAM_EV_DS,		/* a = DS ONU-ID, b = DS type              */
	GPON_PLOAM_EV_O3_FEED_RESET,	/* -                                       */
	GPON_PLOAM_EV_ONU_ID,		/* a = assigned ONU-ID, b = T-CONT16 alloc */
	GPON_PLOAM_EV_CAM_READBACK,	/* a = T-CONT, b = wanted alloc            */
	GPON_PLOAM_EV_RANGING_TIME,	/* a = EqD                                 */
	GPON_PLOAM_EV_DISABLE_SN,	/* a = disable code d[0]                   */
	GPON_PLOAM_EV_DEACT,		/* a = our ONU-ID at the time              */
	GPON_PLOAM_EV_DEACT_KEEP_LOCK,	/* a = ticks held at O5 (reseat skipped)   */
	GPON_PLOAM_EV_EXT_BURST,	/* a = t3pre, b = t3ranged                 */
	GPON_PLOAM_EV_ACK,		/* a = acknowledged DS type                */
	GPON_PLOAM_EV_ASSIGN_ALLOC,	/* a = alloc, b = op code d[2]             */
	GPON_PLOAM_EV_DATA_TCONT,	/* a = alloc, b = data T-CONT              */
	GPON_PLOAM_EV_REQ_KEY,		/* -                                       */
	GPON_PLOAM_EV_KEY_SENT,		/* a = key index                           */
	GPON_PLOAM_EV_REQ_PW,		/* -                                       */
	GPON_PLOAM_EV_KEY_SWITCH_ARM,	/* a = superframe count armed              */
	GPON_PLOAM_EV_KEY_SWITCH,	/* a = key staged?, b = armed superframe   */
	GPON_PLOAM_EV_UNHANDLED,	/* a = DS type, b = DS ONU-ID              */
	GPON_PLOAM_EV_SN_REPROVISIONED,	/* -                                       */
	GPON_PLOAM_EV_O5_WATCHDOG,	/* a = ticks held at O5 with no WAN RX     */
	GPON_PLOAM_EV_LOS_RERANGE,	/* a = consecutive LOS ticks               */
};

/* ---------------------------------------------------------------------------
 * The imperative shell. Every entry is a SIDE EFFECT the core decided to
 * perform and cannot express as a return value.
 *
 * ★ ORDER IS PART OF THE CONTRACT. These are invoked at exactly the points the
 * Luna driver invoked the register writes they replace; a shell that reorders
 * them changes behaviour. In particular the burst-overhead write happens BEFORE
 * the EqD write on the O3 edge, and the US-PLOAM flush happens AFTER the ranged
 * BOH switch on the O4 edge.
 *
 * ★ NOTE — this is deliberately NOT `struct gpon_shell_ops` from the plan's
 * §3 sketch. Two of that sketch's signatures would have changed behaviour:
 *   - `ploam_tx(sh, msg[12])` drops the US_PLOAM_IND QUEUE selector, and the
 *     queue is load-bearing: the urgent queue (0x1) pre-empts the auto-SN burst
 *     (0x6), which is what gets an Acknowledge to the OLT before it raises LOAi,
 *     and the No_message rides the HW auto slot (0x7). Losing it is a wire
 *     change, so the queue is carried.
 *   - `set_eqd(sh, eqd)` implies the core packs the EqD REGISTER word. The
 *     multiframe/in-frame field layout is a register fact; the core hands over
 *     the two computed quantities and the shell packs them.
 * FOLLOW-UP: when the two op tables are unified, unify them in this direction —
 * the difference is a hole in the sketch, not a divergence of intent.
 * ------------------------------------------------------------------------- */
struct gpon_ploam_ops {
	/* --- upstream PLOAM transmit -------------------------------------- */
	/* Enqueue one 12-byte US PLOAM on `queue` (PLM_US_QUEUE_*). Replaces
	 * gpon_send_cpu_ploam(). May block/poll: it is only ever called from
	 * the poll (softirq) context the driver already called it from. */
	void (*ploam_tx)(void *sh, u8 queue, const u8 m[GPON_PLOAM_US_LEN]);

	/* --- burst overhead / ranging ------------------------------------- */
	/* Write BOH_CFG = cfg_word and the first `size` BOH_DATA bytes.
	 * Replaces gpon-rtl9602c.c:5998-6001. */
	void (*boh_write)(void *sh, u32 cfg_word, const u8 *oh, u8 size);
	/* The ONE hardware read taken mid-decision: US_MIN_DELAY[15:7].
	 * Replaces gpon-rtl9602c.c:6019. Injected so the EqD computation is
	 * reproducible offline. */
	u32  (*get_min_delay)(void *sh);
	/* Write the equalization delay from its two computed halves.
	 * Replaces gpon-rtl9602c.c:6024-6026. */
	void (*set_eqd)(void *sh, u32 multiframe, u32 intraframe);
	/* Flush the single shared CPU US-PLOAM TX buffer (PLM_FLUSH_BUF 0->1)
	 * so no pre-ranged-format burst is latched when the first ranged grant
	 * fires. Replaces gpon-rtl9602c.c:6304-6305. */
	void (*us_ploam_flush)(void *sh);

	/* --- state reflection --------------------------------------------- */
	/* Reflect the canonical O-state into hardware and the PON LED. The
	 * shell MAPS the canonical state to its own silicon encoding: Luna's
	 * ONU_STATE field is 1-based and therefore identical, Elnath's is
	 * 0-based. Replaces gpon-rtl9602c.c:6137 + :6139. */
	void (*set_hw_state)(void *sh, enum gpon_ostate st);
	/* Write the ONU-ID into both hardware fields (US_ONU_ID[15:8] and
	 * DS_ONU_STATUS[15:8]). Replaces the gpon_field() pairs at :6235-6236,
	 * :6344-6345, :6573-6574, :6662-6663, :6711-6712. */
	void (*set_hw_onu_id)(void *sh, u8 onu_id);
	/* The FSM dropped below O5. The shell re-arms whatever it gates on O5
	 * (Luna: the VLAN filter, itself gated on its own shell state).
	 * Replaces gpon-rtl9602c.c:6125-6129. */
	void (*on_below_o5)(void *sh);
	/* The FSM entered O5 and the burst-gate re-arm is enabled: re-apply the
	 * US packed-burst register cluster. The core emits the No_message that
	 * follows it. Replaces gpon-rtl9602c.c:6113-6116. */
	void (*o5_rearm_burst)(void *sh);

	/* --- analog / serializer ------------------------------------------ */
	/* O3-entry TX-CMU/PLL re-lock (the cold-start fix). Replaces
	 * gpon_txpll_relock(), gpon-rtl9602c.c:6215. */
	void (*analog_relock)(void *sh);
	/* The post-relock full GPON-datapath reset-B edge that un-parks the
	 * GEM-US feed. Replaces gpon-rtl9602c.c:6221-6223. */
	void (*o3_feed_reset)(void *sh);
	/* Re-seat the US-TX serializer on a re-range: the interface reset-B
	 * edge plus the deferred CDR-lock pulse. Replaces the identical block
	 * at :6359-6366, :6665-6668 and :6714-6717. */
	void (*cdr_reseat)(void *sh);

	/* --- AES key exchange --------------------------------------------- */
	/* Random bytes. Injected so a differential/fuzz run is reproducible.
	 * Replaces get_random_bytes(), gpon-rtl9602c.c:5203. */
	void (*rng)(void *sh, u8 *out, unsigned int len);
	/* Load the key into the hardware STAGED bank. Replaces
	 * gpon_aes_stage_key(), gpon-rtl9602c.c:5221. */
	void (*aes_stage_key)(void *sh, const u8 key[16]);
	/* Arm the key-switch superframe comparator. Replaces
	 * gpon-rtl9602c.c:6499. */
	void (*aes_arm_switch)(void *sh, u32 superframe);

	/* --- GEM / T-CONT provisioning ------------------------------------
	 * The PLOAM layer decides WHICH identities to bind; the CAM/table work
	 * belongs to the GEM provisioning layer and is reached through these.
	 * Every one may fail (a table op can time out) and the core MUST honour
	 * the failure — a non-zero return leaves the corresponding installed
	 * flag CLEAR so the next provisioning event retries. */
	int  (*install_omcc)(void *sh, u16 gem);
	int  (*install_tcont)(void *sh, u8 tcont, u16 alloc);
	void (*install_data_gem)(void *sh);

	/* --- OMCI --------------------------------------------------------- */
	/* Report the WAN-egress (VEIP) operational state up to the OLT.
	 * Replaces rtl9602c_eth_omci_report_oper_up(), :6611. */
	void (*omci_report_oper_up)(void *sh);

	/* --- diagnostics: NEVER load-bearing, NULL is always legal --------- */
	void (*trace)(void *sh, enum gpon_ploam_ev ev, u32 a, u32 b);
};

/* ---------------------------------------------------------------------------
 * Per-board / per-chip configuration.
 *
 * The Luna driver read ~12 `module_param` globals directly from FSM code. A
 * common core cannot own a module_param, so they become fields here and the
 * shell owns the storage. To keep them RUNTIME-WRITABLE exactly as today, the
 * shell should point module_param_named() at these members rather than mirror
 * them — the core reads through the pointer on every use, so a live write takes
 * effect on the next tick, which is the behaviour being preserved.
 *
 * The core only ever READS this; it is const on the core side.
 * ------------------------------------------------------------------------- */
struct gpon_ploam_cfg {
	/* Park the FSM at O1: refuse every advance past O1 so the GPON never
	 * ranges and the shared switch datapath stops churning. (:6075) */
	bool hold;
	/* Per-PLOAM tracing. Only ever gates a `trace` op call. (:6189) */
	bool trace;
	/* NON-STOCK double-bind of the OMCC alloc to the alternate T-CONT.
	 * Default OFF; see the note at the call site. (:6278) */
	bool omcc_alt_bind;
	/* Re-seat the US-TX serializer on a re-range. (:6353, :6367, :6664, :6713) */
	bool cdr_reseat_on_reactivate;
	/* Install the WAN data GEM once the OLT has solicited it. (:6604) */
	bool data_gem_en;
	/* Re-apply the O5 packed-burst register cluster on every O5 entry, not
	 * only at init. (:6110) */
	bool o5_rearm_burst_gate;
	/* Take the full WSDS datapath reset-B edge after the O3 relock. (:6216) */
	bool o3_feed_reset;

	/* OMCC Alloc-ID override; 0 (default) = bind the LIVE ONU-ID, which is
	 * the G.984.3 default and what stock does. (:6264) */
	u16 omcc_alloc_override;

	/* Consecutive (optic_los && !sds_sdet) poll ticks before an autonomous
	 * downstream-LOS re-range. 0 disables. (:6680) */
	unsigned int los_rerange_ticks;
	/* Re-range if held at O5 this many ticks with zero WAN RX. 0 disables
	 * (the shipped default: proven ineffective). (:6650) */
	unsigned int o5_provision_watchdog_ticks;
	/* Emit a No_message US PLOAM every N ticks at O5. 0 disables (the
	 * shipped default: stock emits no unsolicited US PLOAM at O5). (:6845) */
	unsigned int o5_ploam_keepalive_ticks;

	/* T-CONT indices this silicon uses. */
	u8 omcc_tcont;		/* Luna 16 */
	u8 omcc_tcont_alt;	/* Luna 1, used only when omcc_alt_bind */
	u8 data_tcont;		/* Luna 8  */
};

/* ---------------------------------------------------------------------------
 * The FSM context. ALL state lives here — there is not one file-scope variable
 * in gpon_ploam.c, so two instances can run side by side in one address space
 * (which is what makes a host-side differential against the oracle possible at
 * all, and is precisely what the dead gpon_proto.c could not do).
 *
 * The shell may read every field directly; /proc does. Only the core writes.
 * ------------------------------------------------------------------------- */
struct gpon_ploam {
	const struct gpon_ploam_ops *ops;
	const struct gpon_ploam_cfg *cfg;
	void *sh;			/* opaque shell handle passed to every op */

	/* --- identity ----------------------------------------------------- */
	u8 sn[8];			/* G.984.3 ONU-SN: 4-byte ID + 4-byte serial */
	u8 onu_id;			/* 0xff = unassigned                         */
	bool sn_changed;		/* SN was (re)provisioned -> must re-range   */

	/* --- activation state --------------------------------------------- */
	enum gpon_ostate state;
	u32 ticks;			/* FSM poll ticks (~10 ms each)              */
	u32 o5_entry_tick;		/* tick of the last O5 entry (0 = not at O5) */
	int avc_sent;			/* oper-state AVCs emitted this O5           */
	u32 los_run;			/* consecutive real-LOS ticks                */

	/* --- burst overhead learned from the OLT -------------------------- */
	u8 boh_guard;			/* Upstream_Overhead d[0]   = guard bits        */
	u8 boh_ptn;			/* Upstream_Overhead d[3]   = Type-3 pattern    */
	u8 boh_delim[3];		/* Upstream_Overhead d[4..6]                    */
	u8 boh_t3pre;			/* Ext_Burst_Length d[0] = pre-ranged Type-3 len */
	u8 boh_t3ranged;		/* Ext_Burst_Length d[1] = ranged Type-3 len     */

	/* --- provisioning flags the PLOAM layer owns ----------------------
	 * These are set and cleared by PLOAM events (Assign_ONU-ID,
	 * Configure_Port-ID, Assign_Alloc-ID, Deactivate) and by the poll
	 * teardowns. The CAM/table work itself is behind the install ops.
	 * ★ These same flags are named in the plan's gpon_gem carve. They stay
	 * here because every write to them is a PLOAM decision; the GEM layer
	 * owns the SILICON behind install_omcc/install_tcont/install_data_gem.
	 * FOLLOW-UP for whoever lands both files: agree one owner, do not let
	 * both declare them. */
	bool omcc_installed;		/* OMCC GEM datapath installed (one-shot)    */
	/* ★ MOVED UNCHANGED, AND IT IS DEAD: measured 2026-08-05, nothing in
	 * gpon-rtl9602c.c ever assigns `true` to gpon_tcont_installed. It is
	 * declared (:967), cleared on all four teardown paths (:6339 :6569
	 * :6658 :6695) and read only to pick a log string (:6282) and to print
	 * `tcont_done=` (:6422) — so both readers report a constant false.
	 * Recorded, not fixed: setting it would change which branch the
	 * Assign_ONU-ID log takes, and this is a code-motion step. */
	bool tcont_installed;		/* OMCC T-CONT/alloc-id bound (one-shot)     */
	/* Cleared by the teardowns here and READ by the install gate here, but
	 * SET by the GEM layer from inside its install (gpon-rtl9602c.c:5673),
	 * deliberately before the US-NIC modeset because the modeset's
	 * collision fix reads it (:5373, :5388). Setting it from the core after
	 * the op returned would therefore change behaviour; use
	 * gpon_ploam_set_data_installed() at exactly that point instead. */
	bool data_installed;		/* WAN data GEM datapath installed           */
	bool data_gem_solicited;	/* OLT sent the OMCI ME268 Create -> may install */
	bool data_tcont_installed;	/* the OLT's DATA Alloc-ID bound to the data T-CONT */
	u16  data_alloc;		/* the OLT's data Alloc-ID                   */

	/* --- AES key exchange --------------------------------------------- */
	u8   aes_key[16];
	u8   key_index;
	u32  aes_switch_time;		/* last armed Key_Switching_Time (de-dup)    */
	bool key_staged;		/* a valid key is loaded in the staged bank  */

	/* --- outage / re-range accounting (time in ms, supplied by the shell) */
	u32 rerange_start_ms;		/* 0 = no outage in flight                   */
	u32 rerange_last_log_ms;	/* 0 = never logged (flap damping)           */
	u32 rerange_cnt;
	u32 last_outage_ms;

	/* --- counters the shell/proc read -------------------------------- */
	u32 sn_tx;			/* Serial_Number_ONU messages emitted        */
	u32 ds_rx;			/* DS PLOAMs dispatched (DS-lock liveness)   */
	u32 tx_total;			/* US PLOAMs emitted (all queues)            */
	u8  last_ds_type;		/* last DS PLOAM type seen                   */
};

/* ---------------------------------------------------------------------------
 * Entry points.
 *
 * ★ EVERY ONE TAKES `now_ms`: a free-running millisecond clock the shell
 * supplies. The core never reads a clock. `now_ms` is only ever used through
 * wrap-safe signed differences, so any 32-bit monotonic ms source works.
 *
 * ★ THE POLL ISLANDS ARE SEPARATE CALLS ON PURPOSE. In the Luna driver these
 * decisions are INTERLEAVED with shell-only work inside one poll function, and
 * the interleaving is load-bearing: the feed re-kick and the upstream-interrupt
 * service are gated on `state == O5 && omcc_installed`, which the watchdog and
 * the LOS re-range can clear in the same tick. Merging the islands into one
 * poll() would change which of them run on the tick a re-range fires. So the
 * shell calls them at exactly the points it called the original blocks:
 *
 *     gpon_ploam_tick()          <- :6559   (ticks++)
 *     [shell: LOS LED]                     :6560
 *     gpon_ploam_sn_changed()    <- :6564-6579
 *     [shell: DS drain loop]               :6580-6588
 *        gpon_ploam_ds()  per message   <- :6180-6553
 *     gpon_ploam_poll_provision()<- :6604-6613
 *     [shell: feed_rekick, us_intr_svc]    :6621-6641
 *     gpon_ploam_poll_watchdog() <- :6650-6671
 *     gpon_ploam_poll_los()      <- :6680-6724
 *     [shell: VLAN, trace, SDS re-sync]    :6738-6831
 *     gpon_ploam_poll_sn_reoffer()<- :6834-6836
 *     gpon_ploam_poll_keepalive()<- :6845-6854
 *     [shell: BOSA, CDR recovery, timer]   :6859-6949
 * ------------------------------------------------------------------------- */

/* Initialise the context. `sn` may be NULL (left zero until provisioned).
 * Sets the G.984.3 burst defaults and parks the FSM at O1. Touches no hardware
 * and calls no op. */
void gpon_ploam_init(struct gpon_ploam *o, const struct gpon_ploam_ops *ops,
		     const struct gpon_ploam_cfg *cfg, void *sh, const u8 sn[8]);

/* Parse a provisioned serial number in the G.984.3 CLI form "XPON12345678"
 * (4 ASCII vendor-ID chars + 8 hex digits) into 8 wire bytes.
 * Moved from gpon-rtl9602c.c:5000-5015. Pure; no context needed. */
void gpon_ploam_parse_sn(const char *s, u8 sn[8]);

/* Install a (re)provisioned serial number and ask the FSM to re-range. The
 * re-range itself happens on the next gpon_ploam_sn_changed() call, exactly as
 * the driver's gpon_sn_changed flag did. */
void gpon_ploam_set_sn(struct gpon_ploam *o, const u8 sn[8]);

/* The OMCI layer saw the OLT's ME 268 (GEM-CTP) Create for the WAN data GEM.
 * Until this is set the data GEM is never installed — installing ahead of the
 * OLT's own create is what made a second admit churn-lock. */
void gpon_ploam_set_data_gem_solicited(struct gpon_ploam *o, bool solicited);

/* The GEM layer has installed (or torn down) the WAN data GEM datapath. Called
 * from inside the install, at the point gpon-rtl9602c.c:5673 set the flag —
 * BEFORE the US-NIC modeset, which reads it. */
void gpon_ploam_set_data_installed(struct gpon_ploam *o, bool installed);

/* Advance the FSM tick counter by one and return the new value; the shell uses
 * the return for its own cadence gates so there is a single tick source. */
u32 gpon_ploam_tick(struct gpon_ploam *o);

/* Dispatch one downstream PLOAM. `len` must be at least GPON_PLOAM_DS_LEN; a
 * shorter frame is DROPPED with no state change (the driver could never be
 * handed one — the length exists so the offline fuzzer cannot walk off the
 * buffer). Returns the number of US PLOAMs emitted while handling it. */
int gpon_ploam_ds(struct gpon_ploam *o, const u8 *m, unsigned int len, u32 now_ms);

/* Poll islands, in the order the shell must call them. Each returns the number
 * of US PLOAMs it emitted. */
int gpon_ploam_sn_changed(struct gpon_ploam *o, u32 now_ms);
int gpon_ploam_poll_provision(struct gpon_ploam *o, u32 now_ms);
/* `wan_rx_zero` = the WAN netdev has received nothing since the last re-range;
 * sampled by the shell because it lives in another driver. */
int gpon_ploam_poll_watchdog(struct gpon_ploam *o, bool wan_rx_zero, u32 now_ms);
/* `optic_los` = the GTC optical loss-of-signal bit, `sds_dark` = the SoC SerDes
 * signal-detect is gone. Both sampled by the shell; only the AND is a real
 * fibre pull. */
int gpon_ploam_poll_los(struct gpon_ploam *o, bool optic_los, bool sds_dark, u32 now_ms);
int gpon_ploam_poll_sn_reoffer(struct gpon_ploam *o, u32 now_ms);
int gpon_ploam_poll_keepalive(struct gpon_ploam *o, u32 now_ms);

/* G.984.3 O-state name, for logs and /proc. Canonical spelling (O1 is
 * *Initial*, O2 is Standby) — the oracle's enum has these shifted, which is
 * recorded as a defect of the oracle, not of either driver. */
const char *gpon_ploam_state_name(enum gpon_ostate st);

#endif /* GPON_PLOAM_H */
