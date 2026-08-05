// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TIER: CORE (prefix gpon_) — protocol only.  NEVER touches hardware:
 * no register access, no clock, no lock, no allocator, no device pointer.
 * One source compiles for MIPS big-endian, ARM64 little-endian and x86.
 * Role: G.984.3 PLOAM activation FSM.
 *
 * Canonical tier rule, the file map and the guard name live in ONE place:
 * see "THE THREE TIERS" in gpon_common.h (this directory).
 * Guard: dev/rtl9607c-test/gpon_layer_hostbuild_test.sh (suite step 17) —
 * it COMPILES this tier against stubs that declare no register accessor,
 * no clock, no lock and no allocator, so impurity cannot build.
 */
/*
 * gpon_ploam.c — G.984.3 PLOAM activation state machine (O1..O7):
 *                the HARDWARE-DECOUPLED protocol core.
 *
 * WHAT THIS IS
 *   The decision half of the ONU activation engine: downstream PLOAM dispatch,
 *   the O1->O5 transitions, the upstream message builders (Serial_Number,
 *   Password, Acknowledge, Encryption_Key, No_message), the burst-overhead and
 *   equalization-delay computations, and the periodic-poll decisions (SN
 *   re-offer cadence, O5 keepalive cadence, downstream-LOS re-range debounce,
 *   O5 provisioning watchdog). See gpon_ploam.h for the full contract.
 *
 * WHY IT IS COMMON  (operator, 2026-08-05: "en openwrt debería estar
 *   estructurado algo así: rtl960x* para la familia para tener código común"
 *   … "la idea es poner en común el código que corresponde para no tener mucho
 *   duplicado")
 *   Compiled from the shared tree into BOTH OpenWrt targets — realtek-luna
 *   (MIPS32 big-endian) and realtek-elnath (ARM64 little-endian) — and onto
 *   x86-64 for the offline differential/fuzz suite. Every wire field is read
 *   with explicit byte math, so the same source produces the same decisions on
 *   all three. Measured honestly: only realtek-luna consumes it TODAY (Elnath's
 *   MAC runs PLOAM in silicon); it earns its place through offline adversarial
 *   testing and the roadmap, not through de-duplication — the header says so at
 *   length rather than claiming a saving that does not exist.
 *
 * THE CORE / SHELL RULE THIS FILE OBEYS
 *   This file DECIDES; the shell DOES. ★ IT MUST NEVER GAIN AN MMIO ACCESS —
 *   no readl/writel/ioremap, no jiffies or timers, no sleeping or delays, no
 *   locks, no allocation, no printk, no device pointers. Time enters as an
 *   explicit `now_ms` argument and as the shell-advanced tick counter. The one
 *   hardware value a decision genuinely needs mid-flight (US_MIN_DELAY) arrives
 *   through an op. The offline gate greps this file for those tokens and
 *   requires zero.
 *
 * PROVENANCE AND FIDELITY
 *   Pure code motion out of
 *   realtek-luna/files-6.18/drivers/net/ethernet/realtek/gpon-rtl9602c.c,
 *   which is at stock parity. Every block names the source lines it came from.
 *   Nothing was "improved" on the way: where a defect was found it was moved
 *   unchanged and recorded below, because a fix folded into a move makes the
 *   next regression un-bisectable.
 *
 * FOLLOW-UPS — found while moving, DELIBERATELY NOT FIXED HERE
 *   P1  gpon_tcont_installed is never set true anywhere in the driver
 *       (declared :967, cleared :6339/:6569/:6658/:6695, read only by two log
 *       sites :6282/:6422). Both readers therefore report a constant false.
 *   P2  gpon_data_tcont_installed is cleared by NO teardown path, only by the
 *       OLT's Assign_Alloc-ID deallocate (:6453). After a Deactivate the guard
 *       at :6441 then skips the data T-CONT re-bind for the rest of the boot.
 *       Masked today: this lab's OLT is single-alloc and data rides T-CONT 16.
 *   P3  The four teardowns diverge three ways on data_gem_solicited —
 *       Deactivate clears it (deliberate), LOS keeps it (deliberate and
 *       documented: "internet doesn't come back after I reconnect the fiber"),
 *       the watchdog and the SN-reprovision path neither clear it nor say so.
 *       Written out in full below rather than factored, so the divergence is
 *       visible; converging them is a separate step with its own gate.
 *   P4  In the burst-overhead build, `boh_len` is u8 and is assigned
 *       `rep + t3 + 3`, which is computed as int and can reach 262 before the
 *       truncation — so the `> GPON_PLOAM_BOH_MAX_LEN` clamp on the next line
 *       cannot see a value that already wrapped. Needs t3 >= 249 from the OLT's
 *       Extended_Burst_Length; no real OLT sends that. Moved verbatim.
 *   P5  The Assign_ONU-ID comment block (:6242-6259) describes omcc_alloc being
 *       captured from Assign_Alloc-ID "for subsequent re-ranges". It is not:
 *       the 2026-07-03 root-cause fix stopped writing it (:6440) and grep finds
 *       no writer, so it is the module_param override only. Comment is stale;
 *       the code it describes is correct and is moved as-is.
 *   P6  Password/Acknowledge/Encryption_Key all build m[0] = our ONU-ID, while
 *       the Request_Password, Key_Switching_Time and BER_interval cases accept
 *       a broadcast 0xff — where our ONU-ID may still be 0xff. Harmless on the
 *       wire because the hardware rewrites the field (ONUID_OVRD), but the
 *       BUILT buffer and the TRANSMITTED frame disagree, so a buffer-level
 *       differential against the oracle mismatches unless the oracle models the
 *       override. Decide which side is authoritative before writing that test.
 *   P7  Elnath resolves four of these cases differently and G.988/G.984.3 side
 *       with Elnath: Disable_SN should move the ONU to O7 (here: O1, so we
 *       re-announce the serial number the OLT just disabled), an O5->O7 drop is
 *       not detected, and O6 POPUP tears down where Elnath keeps the session
 *       alive. Not changed: this target is off the rig and each is a wire
 *       behaviour change needing its own board gate.
 */

#include "gpon_ploam.h"

/* Cadences and thresholds that were bare literals in the driver. Values
 * unchanged; named so a reader can see what they gate. None is a module_param
 * upstream, so they stay compile-time here. */
#define GPON_PLOAM_AVC_DELAY_TICKS	2500	/* :6609 hold O5 this long first  */
#define GPON_PLOAM_AVC_PERIOD_TICKS	150	/* :6610 then re-report this often */
#define GPON_PLOAM_AVC_MAX		3	/* :6608 reports per O5 entry     */
#define GPON_PLOAM_SN_REOFFER_TICKS	50	/* :6835 ~twice a second at O3    */
#define GPON_PLOAM_HEALTHY_O5_TICKS	500	/* :6355 ~5 s = a healthy provision */
#define GPON_PLOAM_RERANGE_LOG_MS	2000	/* :6096 flap damping on the summary */

/* Guard/preamble defaults before the OLT has dictated any (:958-:959). */
#define GPON_PLOAM_BOH_PTN_DEFAULT	0xaa
#define GPON_PLOAM_BOH_DELIM_DEFAULT	{ 0xab, 0x59, 0x83 }

/* ---------------------------------------------------------------------------
 * Small internal helpers. No file-scope state: everything hangs off `o`.
 * ------------------------------------------------------------------------- */

/* Wrap-safe "is a strictly after b" on the 32-bit millisecond clock the shell
 * supplies — the same signed-difference idiom the kernel's time_after() uses,
 * written in UNSIGNED arithmetic so it needs neither a kernel header nor a
 * host one, and so it cannot depend on signed overflow. */
static bool ms_after(u32 a, u32 b)
{
	u32 diff = a - b;

	return diff != 0u && diff < 0x80000000u;
}

static void ev(const struct gpon_ploam *o, enum gpon_ploam_ev e, u32 a, u32 b)
{
	if (o->ops->trace)
		o->ops->trace(o->sh, e, a, b);
}

/* The single point every upstream PLOAM leaves through. `queue` selects the
 * US_PLOAM_IND queue and is load-bearing: urgent (0x1) pre-empts the auto-SN
 * burst (0x6), which is what gets an Acknowledge out before the OLT raises
 * LOAi; No_message rides the hardware auto slot (0x7). */
static void ploam_tx(struct gpon_ploam *o, u8 queue,
		     const u8 m[GPON_PLOAM_US_LEN])
{
	o->ops->ploam_tx(o->sh, queue, m);
	o->tx_total++;
}

/* ---------------------------------------------------------------------------
 * Upstream message builders. Pure byte assembly; the hardware appends the CRC.
 * ------------------------------------------------------------------------- */

/* Serial_Number_ONU (US type 0x01). From gpon-rtl9602c.c:5099-5108. */
static void send_sn(struct gpon_ploam *o)
{
	u8 m[GPON_PLOAM_US_LEN];

	m[0] = 0xff;			/* ONU-ID (unassigned)            */
	m[1] = PLM_US_SERIAL_NUMBER;	/* 0x01                           */
	memcpy(&m[2], o->sn, 8);	/* ONU-SN: ID(4) + serial(4)      */
	m[10] = 0x00;			/* random delay (HW may fill)     */
	m[11] = 0x04;			/* G-bit set, power level 0       */

	ploam_tx(o, PLM_US_QUEUE_SN, m);
	o->sn_tx++;
}

/*
 * Password (US type 0x02), from gpon-rtl9602c.c:5125-5132.
 *
 * GROUND TRUTH (OLT poll 2026-06-13): the OLT sits at O5 spamming
 * Request_Password / Encrypted_Port-ID, never advancing to Configure_Port-ID or
 * Assign_Alloc-ID, and deactivates with alarm LOAi (Loss Of Acknowledge) —
 * because this reply was never sent. The OLT is SN-authenticated so the VALUE
 * is ignored, but the activation handshake stalls without the message. Sent
 * three times on the urgent queue: G.984.3 repeats the Password in consecutive
 * upstream slots for reliability over the unacknowledged channel.
 */
static int send_password(struct gpon_ploam *o)
{
	u8 p[GPON_PLOAM_US_LEN] = { 0 };
	int i;

	p[0] = o->onu_id;		/* our assigned ONU-ID — see FOLLOW-UP P6 */
	p[1] = PLM_US_PASSWORD;		/* 0x02 */
	/* p[2..11] = the 10-octet password, all zero = empty */
	for (i = 0; i < 3; i++)
		ploam_tx(o, PLM_US_QUEUE_URG, p);
	return 3;
}

/*
 * Acknowledge (US type 0x09), from gpon-rtl9602c.c:5146-5154.
 *
 * The OLT arms a post-ranging timer waiting for this; with no reply it
 * Deactivates the ONU (~43 s) — this is what stops the ONU staying online after
 * O5. `ds` points at the full downstream message (ds[0]=ONU-ID, ds[1]=type,
 * ds[2..]=payload); the acknowledgement echoes it. Urgent queue, so it
 * pre-empts the SN burst.
 */
static int send_ack(struct gpon_ploam *o, const u8 *ds)
{
	u8 a[GPON_PLOAM_US_LEN] = { 0 };

	a[0] = o->onu_id;		/* our assigned ONU-ID (HW may override) */
	a[1] = PLM_US_ACKNOWLEDGE;	/* 0x09 */
	a[2] = ds[1];			/* acknowledged message type */
	a[3] = ds[0];			/* acknowledged message ONU-ID */
	a[4] = ds[1];			/* acknowledged message type (echo) */
	memcpy(&a[5], &ds[2], 7);	/* first 7 payload octets */
	ploam_tx(o, PLM_US_QUEUE_URG, a);
	return 1;
}

/*
 * Encryption_Key (US type 0x05), from gpon-rtl9602c.c:5198-5224.
 *
 * Generate a 128-bit AES key and send it in two fragments (m[2]=key index,
 * m[3]=row, m[4..11]=8 key bytes; row 0 = key[0..7], row 1 = key[8..15]), the
 * same key three times = 6 PLOAMs, matching stock. The upstream PLOAM channel
 * is lossy and the OLT re-issues Request_key rapidly when it does not receive a
 * complete key, stalling config. THEN load the same key into the hardware
 * staged bank — the OLT waits for that before it will proceed to OMCI.
 *
 * ★ ORDER IS PRESERVED FROM THE DRIVER: all six fragments are transmitted
 * BEFORE the staged-bank load, which is why the transmit is an op called inline
 * here rather than a deferred tx-log drained by the caller.
 */
static int send_key(struct gpon_ploam *o)
{
	u8 m[GPON_PLOAM_US_LEN];
	int row, rep;

	o->ops->rng(o->sh, o->aes_key, sizeof(o->aes_key));
	o->key_index++;
	for (rep = 0; rep < 3; rep++) {
		for (row = 0; row < 2; row++) {
			memset(m, 0, sizeof(m));
			m[0] = o->onu_id;	/* ONU-ID — see FOLLOW-UP P6   */
			m[1] = PLM_US_ENCRYPT_KEY;
			m[2] = o->key_index;
			m[3] = (u8)row;		/* fragment row 0/1            */
			memcpy(&m[4], o->aes_key + row * 8, 8);
			ploam_tx(o, PLM_US_QUEUE_URG, m);
		}
	}
	o->ops->aes_stage_key(o->sh, o->aes_key);
	o->key_staged = true;
	ev(o, GPON_PLOAM_EV_KEY_SENT, o->key_index, 0);
	return 6;
}

/* No_message (US type 0x04) on the hardware auto slot. The 0xaa fill and the
 * broadcast ONU-ID are what the driver built at :6117-6119 and :6850-6852. */
static void build_nomsg(u8 m[GPON_PLOAM_US_LEN])
{
	memset(m, 0xaa, GPON_PLOAM_US_LEN);
	m[0] = 0xff;			/* ONU-ID (HW overrides via ONUID_OVRD) */
	m[1] = PLM_US_NO_MESSAGE;	/* 0x04 */
}

/* ---------------------------------------------------------------------------
 * Burst overhead and equalization delay: the two computations the OLT dictates.
 * ------------------------------------------------------------------------- */

/*
 * Burst-overhead build, from gpon-rtl9602c.c:5949-5999.
 *
 * Extended_Burst_Length (0x14) sets the Type-3 lengths: t3pre for the
 * pre-ranged (SN/ranging) burst, t3ranged for the ranged (operation) burst:
 *   LENGTH = rep + t3{pre,ranged} + 3   (G.984.3 upstream-overhead length).
 * Without 0x14 (t3 == 0) fall back to the 96-bit / 12-byte default.
 *
 * The delimiter goes at the TRUE end of the burst (oh[size-3..size-1]), not at
 * a fixed oh[9..11]: the RANGED burst the OLT dictates is usually SHORTER than
 * 12 bytes, and with BOH_LENGTH < 12 the hardware emits only BOH_LENGTH bytes,
 * so a delimiter parked at oh[9..11] is CUT OFF — the O5 burst goes out with no
 * delimiter, the OLT's burst receiver cannot frame it, and the link dies with
 * LOAi / "Laser out" -> Deactivate.
 *
 * BOH_REPEAT is NOT guard/8: it is the stored-byte index of the LAST preamble
 * byte before the 3-byte delimiter, i.e. (size - 4). That is the pointer the
 * burst builder replicates from when it extends the stored <= 12 bytes out to
 * the full BOH_LENGTH. With size = 12 this is 8, matching the live-stock golden
 * O5 dump BOH_CFG = 0x083f (REPEAT 8, LENGTH 63). Writing guard/8 instead
 * mis-positions the delimiter in the synthesized ranged burst, so the OLT never
 * locks the O5 grant burst. The pre-ranged SN burst tolerates it (size ==
 * LENGTH == 12, no synthesis, wide acquisition window), which is exactly why
 * ranging used to succeed while O5 failed.
 */
static void apply_boh(struct gpon_ploam *o, bool ranged)
{
	u8 oh[GPON_PLOAM_BOH_LEN];
	u8 guard = o->boh_guard;
	u8 t3 = ranged ? o->boh_t3ranged : o->boh_t3pre;
	u8 rep, i, boh_len, size;
	u32 cfg_word;

	if (guard > 32)
		guard = 32;
	rep = guard / 8;			/* boh_repeat = whole guard bytes */

	/* FOLLOW-UP P4: (rep + t3 + 3) is evaluated as int and truncated here,
	 * so the clamp below cannot see a value that already wrapped. Moved
	 * verbatim — unreachable with any t3 a real OLT sends. */
	boh_len = (u8)(t3 ? (rep + t3 + 3) : GPON_PLOAM_BOH_LEN);
	if (boh_len > GPON_PLOAM_BOH_MAX_LEN)
		boh_len = GPON_PLOAM_BOH_MAX_LEN;
	/* stored bytes (<= 12); the hardware synthesizes the rest */
	size = (boh_len > GPON_PLOAM_BOH_LEN) ? GPON_PLOAM_BOH_LEN : boh_len;
	if (size < 4)				/* need >= 1 fill + 3 delimiter */
		size = 4;

	memset(oh, 0xaa, sizeof(oh));
	for (i = 0; i < rep && i < (u8)(size - 3); i++)
		oh[i] = 0xaa;			/* guard bytes */
	for (; i < (u8)(size - 3); i++)
		oh[i] = o->boh_ptn;		/* Type-3 preamble fill */
	oh[size - 3] = o->boh_delim[0];		/* delimiter at the TRUE end */
	oh[size - 2] = o->boh_delim[1];
	oh[size - 1] = o->boh_delim[2];

	cfg_word = (u32)(((size - 4) & 0xf) << 8) | (boh_len & 0xffu);
	o->ops->boh_write(o->sh, cfg_word, oh, size);
	ev(o, GPON_PLOAM_EV_BOH, ranged ? 1 : 0, cfg_word);
}

/*
 * Upstream equalization delay, from gpon-rtl9602c.c:6019-6026.
 *
 * The OLT-visible burst time is `value` plus the local MIN_DELAY1 scaled to
 * bits (x16 x8 = x128), then split across the 19440x8-bit upstream frame into a
 * multiframe count and an in-frame offset. Pre-ranging (value 0) yields
 * 290 * 128 = 37120 (0x9100), the correct burst position before the OLT hands
 * us a ranging EqD.
 *
 * MIN_DELAY1 is the ONE hardware value a decision here needs, so it arrives
 * through an op; the register field packing of the result belongs to the shell.
 */
static void set_eqd(struct gpon_ploam *o, u32 value)
{
	u32 min_delay1 = o->ops->get_min_delay(o->sh);
	u32 eqd1  = value + min_delay1 * 128;
	u32 multi = eqd1 / GPON_PLOAM_EQD_FRAME_LEN;
	u32 intra = eqd1 - multi * GPON_PLOAM_EQD_FRAME_LEN;

	o->ops->set_eqd(o->sh, multi, intra);
}

/* ---------------------------------------------------------------------------
 * State transitions, from gpon-rtl9602c.c:6068-6140.
 * ------------------------------------------------------------------------- */
static void set_state(struct gpon_ploam *o, enum gpon_ostate st, u32 now_ms)
{
	enum gpon_ostate prev = o->state;

	/* hold: keep the FSM parked at O1 — refuse every advance past O1 so the
	 * GPON never ranges or deactivates and the shared switch datapath stops
	 * churning, leaving the LAN bridge and the WiFi AP stable. */
	if (o->cfg->hold && st > GPON_O1_INITIAL)
		return;
	if (o->state != st)
		ev(o, GPON_PLOAM_EV_STATE, o->state, st);
	if (prev != st)
		o->los_run = 0;		/* fresh LOS debounce on every transition */
	o->state = st;

	if (st == GPON_O5_OPERATION && prev != GPON_O5_OPERATION) {
		/* Re-range completion: if we had dropped below O5 (fibre LOS,
		 * Deactivate, watchdog) this O5 re-entry closes the outage.
		 * Count it, record the duration and report ONE flap-damped
		 * summary — the fibre-pull "did it recover" witness. The data
		 * GEM re-installs a moment later from the poll. */
		if (o->rerange_start_ms) {
			o->rerange_cnt++;
			o->last_outage_ms = now_ms - o->rerange_start_ms;
			o->rerange_start_ms = 0;
			if (!o->rerange_last_log_ms ||
			    ms_after(now_ms, o->rerange_last_log_ms +
					     GPON_PLOAM_RERANGE_LOG_MS)) {
				ev(o, GPON_PLOAM_EV_RERANGE_DONE,
				   o->rerange_cnt, o->last_outage_ms);
				/* FOLLOW-UP P8: stored raw, exactly as the
				 * driver stores jiffies at :6099 — so the one
				 * instant where the clock reads 0 is
				 * indistinguishable from "never logged" and
				 * costs one extra summary line. The driver
				 * guards the OTHER sentinel (:6123) and not
				 * this one; the inconsistency is moved as
				 * found rather than tidied, and it is
				 * diagnostic-only either way. */
				o->rerange_last_log_ms = now_ms;
			}
		}
		o->o5_entry_tick = o->ticks ? o->ticks : 1;
		o->avc_sent = 0;	/* re-report oper-up on each online */

		/* Re-apply the O5 packed-burst gate cluster and re-arm the
		 * hardware auto-No_message keepalive on EVERY O5 entry, not
		 * only at init: a re-range performs a GMAC/SerDes reset that
		 * can clear these upstream registers, so a re-ranged O5 must
		 * not run on reset defaults. Upstream-side only, harmless to
		 * the downstream and to ranging. */
		if (o->cfg->o5_rearm_burst_gate) {
			u8 nomsg[GPON_PLOAM_US_LEN];

			o->ops->o5_rearm_burst(o->sh);
			build_nomsg(nomsg);
			ploam_tx(o, PLM_US_QUEUE_NOMSG, nomsg);
		}
	} else if (st < GPON_O5_OPERATION && prev >= GPON_O5_OPERATION) {
		o->rerange_start_ms = now_ms ? now_ms : 1;  /* outage timer starts */
		o->o5_entry_tick = 0;
		/* The shell re-arms whatever it gates on O5 (Luna: the VLAN
		 * filter for the next config pass), keeping its own guard. */
		o->ops->on_below_o5(o->sh);
	}

	/* Reflect the state into the silicon's ONU_STATE field and the PON LED.
	 * The shell maps the canonical G.984.3 state to its own encoding —
	 * Luna's field is 1-based and therefore identical, Elnath's is 0-based. */
	o->ops->set_hw_state(o->sh, st);
}

/* ---------------------------------------------------------------------------
 * Activation teardowns.
 *
 * ★ FOUR PATHS, WRITTEN OUT IN FULL AND DELIBERATELY NOT FACTORED. They have
 * diverged three ways on data_gem_solicited (FOLLOW-UP P3) and one way on the
 * serializer re-seat, and the divergence is the point: two of the four are
 * documented decisions and two are silent. A shared helper would hide that and
 * would make the eventual convergence look like it had already happened.
 * Converging them is a separate step, after the code motion, with its own gate.
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * Downstream PLOAM dispatch, from gpon-rtl9602c.c:6180-6553.
 * ------------------------------------------------------------------------- */
int gpon_ploam_ds(struct gpon_ploam *o, const u8 *m, unsigned int len, u32 now_ms)
{
	u32 tx0 = o->tx_total;
	u8 onu_id, type;
	const u8 *d;

	/* A short frame is dropped with no state change. The driver could never
	 * be handed one (its reader always produces 13 bytes); the bound exists
	 * so the offline fuzzer cannot walk off the buffer. */
	if (!m || len < GPON_PLOAM_DS_LEN)
		return 0;

	onu_id = m[0];
	type   = m[1];
	d      = &m[2];			/* 10 data octets */

	/* Surface any downstream PLOAM that is not the repetitive broadcast
	 * acquisition traffic, so activation progress is visible. */
	o->last_ds_type = type;
	if (o->cfg->trace && type != PLM_DS_UPSTREAM_OVERHEAD &&
	    type != PLM_DS_EXT_BURST_LENGTH)
		ev(o, GPON_PLOAM_EV_DS, onu_id, type);

	switch (type) {
	case PLM_DS_UPSTREAM_OVERHEAD:
		/* The OLT is acquiring ONUs (it broadcasts this continuously).
		 * On the O1/O2 -> O3 edge, program the burst overhead and the
		 * pre-ranging EqD the OLT dictates BEFORE the first Serial
		 * Number, then move to O3; the SN is re-sent, throttled, from
		 * the poll so the upstream PLOAM queue is not flooded.
		 * G.984.3 payload: d[0]=guard bits, d[3]=Type-3 preamble
		 * pattern, d[4..6]=delimiter, d[7] bit5 = pre-EqD present with
		 * value d[8:9] (x32 x8 bits). */
		if (o->state < GPON_O3_SERIAL) {
			u32 pre_eqd = ((d[7] >> 5) & 1) ?
				(((u32)d[8] << 8) | d[9]) * 32 * 8 : 0;

			o->boh_guard    = d[0];
			o->boh_ptn      = d[3];
			o->boh_delim[0] = d[4];
			o->boh_delim[1] = d[5];
			o->boh_delim[2] = d[6];
			apply_boh(o, false);	/* folds in any prior 0x14 t3pre */
			set_eqd(o, pre_eqd);
			set_state(o, GPON_O2_STANDBY, now_ms);
			set_state(o, GPON_O3_SERIAL, now_ms);
			/* Re-lock the TX CMU PLL now the downstream optics are
			 * stable, before the first upstream burst: a fresh
			 * power-on under strong downstream light can assert
			 * signal-detect before the CMU has settled and latch the
			 * TX PLL onto the wrong rate (~50% "Laser out"). */
			o->ops->analog_relock(o->sh);
			if (o->cfg->o3_feed_reset) {
				/* The relock re-parks the GEM-US feed run-state.
				 * Un-park it with the FULL datapath reset-B edge
				 * the O5-light re-arm omits, here at O3 while
				 * the downstream is not yet locked. */
				o->ops->o3_feed_reset(o->sh);
				ev(o, GPON_PLOAM_EV_O3_FEED_RESET, 0, 0);
			}
			send_sn(o);		/* first SN immediately */
		}
		break;

	case PLM_DS_ASSIGN_ONU_ID:
		/* d[0] = assigned ONU-ID, d[1..8] = serial number to match. */
		if (!memcmp(&d[1], o->sn, 8)) {
			u16 tcont16_alloc;

			o->onu_id = d[0];
			o->ops->set_hw_onu_id(o->sh, o->onu_id);
			/* Bind the OMCC's T-CONT to its management Alloc-ID.
			 * The OMCC upstream alloc IS the live ONU-ID (G.984.3
			 * default, and what stock does); the override exists
			 * only for A/B on an OLT that grants T-CONT 1 not 0.
			 * See FOLLOW-UP P5: the long comment upstream describes
			 * a capture-from-Assign_Alloc-ID that no longer exists. */
			tcont16_alloc = o->cfg->omcc_alloc_override ?
					o->cfg->omcc_alloc_override : o->onu_id;
			(void)o->ops->install_tcont(o->sh, o->cfg->omcc_tcont,
						    tcont16_alloc);
			ev(o, GPON_PLOAM_EV_CAM_READBACK, o->cfg->omcc_tcont,
			   tcont16_alloc);
			/* NON-STOCK double-bind (default off): binding the same
			 * alloc ALSO to the alternate T-CONT makes the GTC
			 * alloc-CAM resolve a BWMAP grant to the EMPTY T-CONT,
			 * so the DBRu reports zero occupancy for it while the
			 * real T-CONT holds the pages — the OLT grants once then
			 * stops. Stock binds the OMCC T-CONT only. */
			if (o->cfg->omcc_alt_bind && tcont16_alloc != o->onu_id)
				(void)o->ops->install_tcont(o->sh,
						o->cfg->omcc_tcont_alt,
						tcont16_alloc);
			ev(o, GPON_PLOAM_EV_ONU_ID, o->onu_id, tcont16_alloc);
			set_state(o, GPON_O4_RANGING, now_ms);
		}
		break;

	case PLM_DS_RANGING_TIME:
		/* Accept only the main-path EqD (d[0] bit0 == 0); the
		 * protect-path EqD is not configurable. EqD is d[1..4]
		 * big-endian and is folded with MIN_DELAY1 by set_eqd(). */
		if (onu_id == o->onu_id && !(d[0] & 0x01)) {
			u32 eqd = ((u32)d[1] << 24) | ((u32)d[2] << 16) |
				  ((u32)d[3] << 8) | d[4];

			set_eqd(o, eqd);
			apply_boh(o, true);	/* switch to the ranged burst */
			/* Flush any pre-ranged-format upstream PLOAM still
			 * latched in the single shared CPU TX buffer before the
			 * first ranged grant fires, so the OLT's NARROW ranged
			 * burst-receive window never has to frame a stale
			 * pre-ranged burst. Stock flushes at exactly this
			 * O4-EqD edge. */
			o->ops->us_ploam_flush(o->sh);
			ev(o, GPON_PLOAM_EV_RANGING_TIME, eqd, 0);
			set_state(o, GPON_O5_OPERATION, now_ms);
		}
		break;

	case PLM_DS_DISABLE_SN:
		/* Disable_Serial_Number (G.984.3): d[0] is the disable/enable
		 * code, d[1..8] the target SN; the OLT broadcasts it. ONLY a
		 * real DISABLE resets us: 0xff for OUR serial number, or 0x0f
		 * (disable all). An ENABLE (0x00 for our SN = the OLT
		 * RE-ALLOWING a previously disabled SN) must NOT reset — the
		 * old code reset on every 0x06 and fought the OLT's re-enable,
		 * trapping the ONU in a re-range loop.
		 * FOLLOW-UP P7: G.984.3 puts a disabled ONU in O7; falling
		 * through to O1 here re-announces the serial number the OLT
		 * just disabled. Unchanged: a wire behaviour change. */
		if (!((d[0] == 0xff && !memcmp(&d[1], o->sn, 8)) || d[0] == 0x0f))
			break;		/* enable / not our SN -> keep activating */
		ev(o, GPON_PLOAM_EV_DISABLE_SN, d[0], 0);
		/* A real Disable_SN tears down exactly like an OLT Deactivate, so
		 * it drops into that case rather than duplicating the teardown.
		 * The attribute (not a comment) is what the kernel's
		 * -Wimplicit-fallthrough=5 accepts; see gpon_common.h. */
		fallthrough;

	case PLM_DS_DEACTIVATE_ONU:
		if (onu_id == o->onu_id || onu_id == 0xff) {
			ev(o, GPON_PLOAM_EV_DEACT, o->onu_id, 0);
			/* --- TEARDOWN 1 of 4: OLT Deactivate / Disable_SN.
			 * FULL reset to O1. Clearing only the software ONU-ID
			 * and key used to leave the one-shot install guards
			 * TRUE and the hardware ONU-ID registers stale, so
			 * under OLT deactivate-churn the second and later
			 * re-ranges SKIPPED the OMCC/T-CONT install and the ONU
			 * never rebuilt its OMCI datapath.
			 * This is the ONLY path that clears data_gem_solicited
			 * (a real deprovision: the OLT will re-send its ME 268
			 * on re-admit). See FOLLOW-UP P3, and P2 for the
			 * data T-CONT flag that no path clears. */
			o->onu_id = 0xff;
			o->omcc_installed = false;
			o->tcont_installed = false;
			o->data_installed = false;
			o->data_gem_solicited = false;
			o->aes_switch_time = 0xffffffff;
			o->key_staged = false;
			o->ops->set_hw_onu_id(o->sh, 0xff);
			/* Re-seat the serializer ONLY when the prior O5 was
			 * short/marginal (a real upstream burst-quality fault).
			 * A healthy, long-provisioned O5 that the OLT
			 * deactivated is NOT a serializer fault — re-rolling it
			 * there only manufactures a fresh re-range the OLT must
			 * re-admit, feeding the OLT's churn-lock. Stock
			 * re-acquires gently and never re-rolls the CDR on a
			 * deactivate. ~500 ticks (~5 s) of held O5 marks a
			 * healthy provision. */
			if (o->cfg->cdr_reseat_on_reactivate &&
			    !(o->state == GPON_O5_OPERATION && o->o5_entry_tick &&
			      (o->ticks - o->o5_entry_tick) > GPON_PLOAM_HEALTHY_O5_TICKS)) {
				o->ops->cdr_reseat(o->sh);
			} else if (o->cfg->cdr_reseat_on_reactivate) {
				ev(o, GPON_PLOAM_EV_DEACT_KEEP_LOCK,
				   o->o5_entry_tick ? o->ticks - o->o5_entry_tick : 0, 0);
			}
			set_state(o, GPON_O1_INITIAL, now_ms);
		}
		break;

	case PLM_DS_EXT_BURST_LENGTH:
		/* Extended_Burst_Length: d[0] = Type-3 preamble length for the
		 * PRE-RANGED (SN/ranging) burst, d[1] = for the ranged
		 * (operation) burst. The OLT broadcasts this during
		 * acquisition; honouring d[0] lengthens our SN-burst preamble
		 * so the OLT's burst receiver can lock and range us. Re-armed
		 * only while still broadcast-addressed / pre-ranging. */
		o->boh_t3ranged = d[1];		/* applied at the O5 transition */
		if (o->onu_id == 0xff && o->boh_t3pre != d[0]) {
			o->boh_t3pre = d[0];
			apply_boh(o, false);
			ev(o, GPON_PLOAM_EV_EXT_BURST, o->boh_t3pre,
			   o->boh_t3ranged);
		}
		break;

	case PLM_DS_CONFIG_PORT:
		/* Configure_Port-ID: the OLT assigns the OMCC GEM port for OMCI
		 * (d[0] bit0 = enable, gem = (d[1]<<4)|(d[2]>>4)). Install the
		 * OMCC GEM datapath (one-shot) so downstream OMCI reaches the
		 * CPU, THEN Acknowledge — the ONU is receive-ready before the
		 * OLT proceeds.
		 * The WAN data GEM is NOT installed here: doing it at PLOAM
		 * config time, before the OLT had created its own GEM, made the
		 * OLT unable to reconcile ours on a second admit and
		 * churn-lock. It is driven from the poll, gated on the OLT's
		 * own ME 268 Create. */
		if (onu_id == o->onu_id) {
			u16 gem = ((u16)d[1] << 4) | (d[2] >> 4);

			if ((d[0] & 0x1) && !o->omcc_installed) {
				if (!o->ops->install_omcc(o->sh, gem))
					o->omcc_installed = true;
			}
			send_ack(o, m);
			if (o->cfg->trace)
				ev(o, GPON_PLOAM_EV_ACK, type, 0);
		}
		break;

	case PLM_DS_ASSIGN_ALLOC_ID:
		/* Assign_Alloc-ID: alloc = (d[0]<<4)|(d[1]>>4);
		 * d[2] 0x01 = allocate, 0xff = deallocate. Then Acknowledge.
		 *
		 * ★ ROOT-CAUSE FIX (2026-07-03), do not regress it: this
		 * carries a DATA Alloc-ID (this OLT sends 0x100; >= 255 is a
		 * data alloc per G.984.3, below that is OMCC-implicit). The
		 * OMCC T-CONT rides the LIVE ONU-ID, bound at Assign_ONU-ID,
		 * and is NEVER reassigned here. The old code bound this alloc
		 * to the OMCC T-CONT, overwriting the ONU-ID, so the OLT's
		 * default-alloc grants missed the alloc-CAM and the OMCC
		 * T-CONT became unreachable — the months-long upstream wall.
		 * Bind it to the DATA T-CONT; the OMCC T-CONT stays = ONU-ID.
		 * (The dead gpon_proto.c fork still carries the pre-fix
		 * binding; that is one reason it is deleted rather than
		 * revived.) */
		if (onu_id == o->onu_id) {
			u16 alloc = ((u16)d[0] << 4) | (d[1] >> 4);

			ev(o, GPON_PLOAM_EV_ASSIGN_ALLOC, alloc, d[2]);
			if (d[2] == 0x01) {
				if (alloc != o->onu_id && !o->data_tcont_installed) {
					if (!o->ops->install_tcont(o->sh,
							o->cfg->data_tcont, alloc)) {
						o->data_tcont_installed = true;
						o->data_alloc = alloc;
						ev(o, GPON_PLOAM_EV_DATA_TCONT,
						   alloc, o->cfg->data_tcont);
					}
				}
			} else if (d[2] == 0xff && o->data_tcont_installed &&
				   alloc == o->data_alloc) {
				/* OLT deallocated the data Alloc-ID: allow a
				 * clean re-bind on the next allocate. The CAM
				 * binding is idempotent; no teardown needed. */
				o->data_tcont_installed = false;
			}
			send_ack(o, m);
			if (o->cfg->trace)
				ev(o, GPON_PLOAM_EV_ACK, type, 0);
		}
		break;

	case PLM_DS_REQUEST_KEY:
		/* The OLT requests a downstream AES key; reply with
		 * Encryption_Key and load the same key into the staged bank. */
		if (onu_id == o->onu_id) {
			send_key(o);
			ev(o, GPON_PLOAM_EV_REQ_KEY, 0, 0);
		}
		break;

	case PLM_DS_REQUEST_PASSWORD:
		/* GROUND TRUTH: without the reply the OLT stalls at O5 spamming
		 * this and deactivates us with LOAi. The OLT is SN-authenticated
		 * so the value is ignored, but the message is required to
		 * advance activation. Broadcast is accepted, like stock. */
		if (onu_id == o->onu_id || onu_id == 0xff) {
			send_password(o);
			ev(o, GPON_PLOAM_EV_REQ_PW, 0, 0);
		}
		break;

	case PLM_DS_KEY_SWITCH:
		/* Key_Switching_Time: the OLT supplies the 30-bit superframe
		 * count at which the hardware promotes the staged AES key to
		 * active. Arm the comparator and Acknowledge. The OLT will not
		 * advance to OMCI until this handshake completes, so a missing
		 * handler leaves it re-cycling Request_Key / Configure_Port-ID
		 * forever. De-duplicated per superframe (the OLT re-sends every
		 * cycle). */
		if (onu_id == o->onu_id || onu_id == 0xff) {
			u32 fc = ((u32)(d[0] & 0x3f) << 24) | ((u32)d[1] << 16) |
				 ((u32)d[2] << 8) | d[3];

			/* Arm only once a key has actually been loaded into the
			 * staged bank: arming a switch to an empty or stale bank
			 * would promote a garbage key and corrupt AES.
			 * Acknowledge either way so the OLT sees it handled. */
			if (o->key_staged && fc != o->aes_switch_time) {
				o->aes_switch_time = fc;
				o->ops->aes_arm_switch(o->sh, fc);
				ev(o, GPON_PLOAM_EV_KEY_SWITCH_ARM, fc, 0);
			}
			send_ack(o, m);
			ev(o, GPON_PLOAM_EV_KEY_SWITCH, o->key_staged ? 1 : 0,
			   o->aes_switch_time);
		}
		break;

	case PLM_DS_ENCRYPT_PORT:
		/* Encrypted_Port-ID: G.984.3 requires an Acknowledge; the OLT
		 * arms a ~43 s timer and Deactivates us if none arrives. */
		if (onu_id == o->onu_id) {
			send_ack(o, m);
			if (o->cfg->trace)
				ev(o, GPON_PLOAM_EV_ACK, type, 0);
		}
		break;

	case PLM_DS_CFG_VPVC:
		/* Configure_VP/VC: legacy ATM connection setup, unsupported on
		 * a GEM ONU — but stock still Acknowledges, and a missing ACK
		 * to any acknowledge-required downstream PLOAM raises LOAi. */
		if (onu_id == o->onu_id) {
			send_ack(o, m);
			ev(o, GPON_PLOAM_EV_ACK, type, 0);
		}
		break;

	case PLM_DS_BER_INTERVAL:
		/* BER_interval: the OLT configures the upstream BER reporting
		 * interval and REQUIRES an Acknowledge. Stock also arms a timer
		 * that periodically emits Remote_Error_Indication; the ACK is
		 * the part that prevents LOAi, so send it. Broadcast accepted,
		 * like stock. */
		if (onu_id == o->onu_id || onu_id == 0xff) {
			send_ack(o, m);
			ev(o, GPON_PLOAM_EV_ACK, type, 0);
		}
		break;

	default:
		/* Anything addressed to us that we do not model: report it so a
		 * missing acknowledge-required type is visible instead of a
		 * silent drop into LOAi. PEE / Power_Level / PST /
		 * Ranging_Adjustment do NOT require an ACK in G.984.3; if a
		 * reported type turns out to need one, add an explicit case. */
		if (onu_id == o->onu_id || onu_id == 0xff)
			ev(o, GPON_PLOAM_EV_UNHANDLED, type, onu_id);
		break;
	}

	o->ds_rx++;			/* downstream-lock liveness */
	return (int)(o->tx_total - tx0);
}

/* ---------------------------------------------------------------------------
 * Poll islands. Called by the shell at exactly the points the original blocks
 * sat at — see the ordering note in gpon_ploam.h. Merging them would change
 * which shell blocks run on the tick a re-range fires.
 * ------------------------------------------------------------------------- */

/* :6559 — the single tick source. */
u32 gpon_ploam_tick(struct gpon_ploam *o)
{
	o->ticks++;
	return o->ticks;
}

/*
 * :6564-6579 — the serial number was (re)provisioned after ranging had begun
 * (the driver starts with a placeholder SN, which the OLT auto-ranges as a
 * phantom that never matches the provisioned ONU). Drop to O1 and re-offer.
 *
 * TEARDOWN 2 of 4. Does NOT touch data_gem_solicited and does NOT re-seat the
 * serializer — see FOLLOW-UP P3.
 */
int gpon_ploam_sn_changed(struct gpon_ploam *o, u32 now_ms)
{
	u32 tx0 = o->tx_total;

	if (o->sn_changed) {
		o->sn_changed = false;
		if (o->state > GPON_O1_INITIAL) {
			o->onu_id = 0xff;
			o->omcc_installed = false;
			o->tcont_installed = false;
			o->data_installed = false;
			o->aes_switch_time = 0xffffffff;
			o->key_staged = false;
			o->ops->set_hw_onu_id(o->sh, 0xff);
			set_state(o, GPON_O1_INITIAL, now_ms);
			ev(o, GPON_PLOAM_EV_SN_REPROVISIONED, 0, 0);
		}
	}
	return (int)(o->tx_total - tx0);
}

/*
 * :6604-6613 — two O5 gates that must stay adjacent and in this order.
 *
 * 1. Install the WAN data GEM once the OLT has issued its own ME 268 (GEM-CTP)
 *    Create — idempotently over the OLT's GEM, never proactively ahead of it
 *    (that was the second-admit churn cause). Driven from the poll so the
 *    upstream NIC modeset stays off the OMCI receive softirq.
 * 2. Report the WAN-egress (VEIP) operational up to the OLT. Must fire AFTER
 *    config-apply finishes AND after the OLT has created its ME 329, ~2500
 *    ticks (~31 s) after O5; firing earlier, during config, DISRUPTED config.
 *    The OLT never polls the data MEs it creates — it un-gates downstream user
 *    data only when the ONU reports the port up. Re-armed on each O5 entry.
 */
int gpon_ploam_poll_provision(struct gpon_ploam *o, u32 now_ms)
{
	(void)now_ms;

	if (o->state == GPON_O5_OPERATION && o->cfg->data_gem_en &&
	    o->omcc_installed && o->data_gem_solicited && !o->data_installed)
		o->ops->install_data_gem(o->sh);

	if (o->state == GPON_O5_OPERATION && o->omcc_installed &&
	    o->avc_sent < GPON_PLOAM_AVC_MAX && o->o5_entry_tick &&
	    (o->ticks - o->o5_entry_tick) > GPON_PLOAM_AVC_DELAY_TICKS &&
	    ((o->ticks - o->o5_entry_tick) % GPON_PLOAM_AVC_PERIOD_TICKS) == 0) {
		o->ops->omci_report_oper_up(o->sh);
		o->avc_sent++;
	}
	return 0;		/* emits no upstream PLOAM */
}

/*
 * :6650-6671 — O5 provisioning watchdog. A boot that reached O5 locally but
 * that the OLT never provisioned (WAN receive still zero well past the slow
 * lease window) is stuck on a non-frameable upstream serializer phase with no
 * OLT Deactivate to recover it. Self-re-range to RE-ROLL the phase, mirroring
 * the Deactivate path including the re-seat that actually changes the lock.
 * WAN receive is above zero on any working or slow-leasing link, so this fires
 * only on a genuinely dead link.
 *
 * TEARDOWN 3 of 4. Does NOT touch data_gem_solicited, and unlike the LOS path
 * does not say why — FOLLOW-UP P3.
 */
int gpon_ploam_poll_watchdog(struct gpon_ploam *o, bool wan_rx_zero, u32 now_ms)
{
	u32 tx0 = o->tx_total;

	if (o->cfg->o5_provision_watchdog_ticks &&
	    o->state == GPON_O5_OPERATION && o->onu_id != 0xff &&
	    o->o5_entry_tick &&
	    (o->ticks - o->o5_entry_tick) > o->cfg->o5_provision_watchdog_ticks &&
	    wan_rx_zero) {
		ev(o, GPON_PLOAM_EV_O5_WATCHDOG, o->ticks - o->o5_entry_tick, 0);
		o->onu_id = 0xff;
		o->omcc_installed = false;
		o->tcont_installed = false;
		o->data_installed = false;
		o->aes_switch_time = 0xffffffff;
		o->key_staged = false;
		o->ops->set_hw_onu_id(o->sh, 0xff);
		if (o->cfg->cdr_reseat_on_reactivate)
			o->ops->cdr_reseat(o->sh);
		set_state(o, GPON_O1_INITIAL, now_ms);
	}
	return (int)(o->tx_total - tx0);
}

/*
 * :6680-6724 — autonomous downstream-LOS recovery (fibre pull / loss of
 * downstream light). The OLT cannot send a Deactivate when downstream light is
 * gone, so the ONU must notice the sustained optical LOS itself, tear down to
 * O1 and re-acquire when light returns. Without this the FSM sits stale at O5
 * after a fibre pull and never re-ranges on reconnect.
 *
 * Real light loss = the GTC optical LOS AND the SoC SerDes signal-detect both
 * gone. An I2C pad steal perturbs the optical LOS alone (signal-detect stays
 * up); an internal SerDes re-seat can blip signal-detect alone (optical LOS
 * stays clear); only a true fibre pull drops BOTH. Requiring the AND lets the
 * debounce be short without false-tripping on either transient. Once at O1 the
 * state gate stops counting until the FSM climbs back past O1 on relight.
 *
 * TEARDOWN 4 of 4.
 */
int gpon_ploam_poll_los(struct gpon_ploam *o, bool optic_los, bool sds_dark,
			u32 now_ms)
{
	u32 tx0 = o->tx_total;

	if (o->cfg->los_rerange_ticks && o->state >= GPON_O2_STANDBY) {
		if (optic_los && sds_dark) {
			if (++o->los_run == o->cfg->los_rerange_ticks) {
				ev(o, GPON_PLOAM_EV_LOS_RERANGE, o->los_run, 0);
				o->onu_id = 0xff;
				o->omcc_installed = false;
				o->tcont_installed = false;
				o->data_installed = false;
				/* ★2026-07-05: do NOT clear data_gem_solicited
				 * on a fibre-LOS re-range. This re-range is
				 * ONU-initiated: the OLT never Deactivated us,
				 * so it KEEPS our OMCI/GEM provisioning across
				 * the brief outage and does NOT re-send the
				 * ME 268 on re-admit. Clearing it made the data
				 * GEM wait forever for a create that never
				 * arrives — "internet doesn't come back after I
				 * reconnect the fiber". Keeping it re-installs
				 * the GEM the OLT still holds as soon as O5 is
				 * re-reached. This does NOT re-introduce the
				 * second-admit churn: that was a fresh admit
				 * where the OLT had not yet created the GEM.
				 * A true deprovision (OLT Deactivate) still
				 * clears it in that handler. */
				o->aes_switch_time = 0xffffffff;
				o->key_staged = false;
				o->ops->set_hw_onu_id(o->sh, 0xff);
				if (o->cfg->cdr_reseat_on_reactivate)
					o->ops->cdr_reseat(o->sh);
				set_state(o, GPON_O1_INITIAL, now_ms);
			}
		} else {
			o->los_run = 0;
		}
	}
	return (int)(o->tx_total - tx0);
}

/* :6834-6836 — while unregistered at O3, re-offer the Serial_Number_ONU about
 * twice a second (the OLT grants SN windows intermittently). */
int gpon_ploam_poll_sn_reoffer(struct gpon_ploam *o, u32 now_ms)
{
	u32 tx0 = o->tx_total;

	(void)now_ms;
	if (o->state >= GPON_O3_SERIAL && o->onu_id == 0xff &&
	    (o->ticks % GPON_PLOAM_SN_REOFFER_TICKS) == 0)
		send_sn(o);
	return (int)(o->tx_total - tx0);
}

/*
 * :6845-6854 — periodic O5 upstream-PLOAM keepalive. Once ranged the FSM
 * otherwise emits ZERO upstream PLOAM, and the shared buffer's auto-No_message
 * template can be stale-clobbered by intervening ACK/SN sends. Emitting a fresh
 * No_message keeps a valid PLOAM in the OLT's granted slots each window,
 * defeating a PLOAM/ack-liveness timeout that fires Deactivate ~25-35 s after
 * provision. Mirrors the unranged SN cadence; upstream PLOAM only, does not
 * touch the downstream or OMCI. Default off, matching stock.
 */
int gpon_ploam_poll_keepalive(struct gpon_ploam *o, u32 now_ms)
{
	u32 tx0 = o->tx_total;

	(void)now_ms;
	if (o->state == GPON_O5_OPERATION && o->onu_id != 0xff &&
	    o->cfg->o5_ploam_keepalive_ticks &&
	    (o->ticks % o->cfg->o5_ploam_keepalive_ticks) == 0) {
		u8 nomsg[GPON_PLOAM_US_LEN];

		build_nomsg(nomsg);
		ploam_tx(o, PLM_US_QUEUE_NOMSG, nomsg);
	}
	return (int)(o->tx_total - tx0);
}

/* ---------------------------------------------------------------------------
 * Identity and lifecycle.
 * ------------------------------------------------------------------------- */

/* Parse "XPON12345678" -> {'X','P','O','N',0x12,0x34,0x56,0x78}.
 * From gpon-rtl9602c.c:5000-5015, with the kernel's hex_to_bin() re-expressed
 * locally so the core compiles on the host too (the driver's dependency on
 * hex_to_bin was one of the two things stopping this file being fuzzable).
 *
 * Behaviour preserved exactly, INCLUDING that an invalid hex digit contributes
 * its hex_to_bin() result: the kernel returns -1 there, so a bad character
 * yields 0xf in that nibble rather than being rejected. Elnath validates its
 * serial number instead; converging is FOLLOW-UP P7 and a behaviour change. */
static u8 sn_hex_nibble(char c)
{
	if (c >= '0' && c <= '9')
		return (u8)(c - '0');
	if (c >= 'a' && c <= 'f')
		return (u8)(c - 'a' + 10);
	if (c >= 'A' && c <= 'F')
		return (u8)(c - 'A' + 10);
	return 0xf;			/* kernel hex_to_bin() returns -1 here */
}

void gpon_ploam_parse_sn(const char *s, u8 sn[8])
{
	int i;

	if (!s || !sn)
		return;
	for (i = 0; i < 4 && s[i]; i++)
		sn[i] = (u8)s[i];
	for (i = 0; i < 4; i++) {
		u8 hi = 0, lo = 0;

		if (s[4 + 2 * i])
			hi = sn_hex_nibble(s[4 + 2 * i]);
		if (s[4 + 2 * i + 1])
			lo = sn_hex_nibble(s[4 + 2 * i + 1]);
		sn[4 + i] = (u8)((hi << 4) | lo);
	}
}

void gpon_ploam_set_sn(struct gpon_ploam *o, const u8 sn[8])
{
	memcpy(o->sn, sn, sizeof(o->sn));
	o->sn_changed = true;
}

void gpon_ploam_set_data_gem_solicited(struct gpon_ploam *o, bool solicited)
{
	o->data_gem_solicited = solicited;
}

void gpon_ploam_set_data_installed(struct gpon_ploam *o, bool installed)
{
	o->data_installed = installed;
}

void gpon_ploam_init(struct gpon_ploam *o, const struct gpon_ploam_ops *ops,
		     const struct gpon_ploam_cfg *cfg, void *sh, const u8 sn[8])
{
	static const u8 delim_default[3] = GPON_PLOAM_BOH_DELIM_DEFAULT;

	memset(o, 0, sizeof(*o));
	o->ops = ops;
	o->cfg = cfg;
	o->sh  = sh;

	if (sn)
		memcpy(o->sn, sn, sizeof(o->sn));
	o->onu_id = 0xff;
	o->state  = GPON_O1_INITIAL;
	o->boh_ptn = GPON_PLOAM_BOH_PTN_DEFAULT;
	memcpy(o->boh_delim, delim_default, sizeof(o->boh_delim));
	o->aes_switch_time = 0xffffffff;
}

const char *gpon_ploam_state_name(enum gpon_ostate st)
{
	switch (st) {
	case GPON_O1_INITIAL:	return "O1-Initial";
	case GPON_O2_STANDBY:	return "O2-Standby";
	case GPON_O3_SERIAL:	return "O3-Serial-Number";
	case GPON_O4_RANGING:	return "O4-Ranging";
	case GPON_O5_OPERATION:	return "O5-Operation";
	case GPON_O6_POPUP:	return "O6-POPUP";
	case GPON_O7_EMERGENCY:	return "O7-Emergency-Stop";
	}
	return "O?-unknown";
}
