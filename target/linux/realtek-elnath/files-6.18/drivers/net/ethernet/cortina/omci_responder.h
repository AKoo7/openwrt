/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * omci_responder.h — clean-room ONU G.988 OMCI baseline responder + ME model
 * for the RTL9607F "Elnath" GPON driver (Stage C of the GPON-WAN bring-up).
 *
 * Kernel port of the host oracle's responder (dev/rtl9607c-oracle), with the
 * on-wire facts aligned to the responder PROVEN against the same HSGQ-G008
 * OLT on the RTL9602C (realtek-luna rtl9602c_eth.c, reached Online/normal +
 * WAN end-to-end).  The ONE rule the whole layout follows: message contents
 * start at octet 8, and only a response that carries a RESULT code spends that
 * octet on the result.
 *   - GET request attribute mask   = msg[8..9]  (a request has no result byte)
 *   - GET response  = result[8] + attr-mask[9..10] + values[11..35]
 *     (25 octets) + optional-attribute mask[36..37] + attribute-execution
 *     mask[38..39].  The last four octets are RESERVED ALWAYS, success
 *     included, so the value area is 25 octets and NOT 29.
 *   - SET request = mask[8..9] + values from msg[10]; CREATE request = the
 *     set-by-create values from msg[8] (no mask)
 *   - MIB-Upload reply row count   = resp[8..9] (NO result byte)
 *   - Get-All-Alarms reply count   = resp[8..9] (NO result byte)
 *   - MIB-Upload-Next request seq  = msg[8..9]; reply = class[8..9] +
 *     inst[10..11] + mask[12..13] + values[14..39] (26 value bytes, no result)
 *   - MIB-Data-Sync +1 per APPLIED Create/Set/Delete, wrap 255 -> 1; an
 *     on-wire MIB-Reset zeroes it
 *   - MIC (bytes 44..47) = the NON-reflected AAL5/I.363.5 CRC-32 over bytes
 *     0..43 (the kernel's crc32_be, init all-ones, final complement), stored
 *     big-endian and computed in SOFTWARE.  It is NOT the reflected zlib
 *     crc32_le — LIVE-PROVEN on this OLT (see the MIC comment in the .c).  A
 *     zero/wrong MIC = the OLT silently drops the response and loops its GET
 *     audit.
 *   - autonomous AVC: TID=0, MT=0x11, changed-attr mask at [8..9], value
 *     from [10]
 * (The host oracle's old mask-at-9/seq-from-inst layout was a host-sim-only
 * convention — self-consistent with its OLT sim but wrong on the real wire.)
 *
 * Endianness-agnostic: all wire access is explicit byte math, never a
 * struct/pointer cast, so the same code runs on LE ARM64 and BE MIPS and can
 * be lifted back to the x86 host suite for fuzzing.
 */
#ifndef OMCI_RESPONDER_H
#define OMCI_RESPONDER_H

#include <linux/types.h>

#define OMCI_LEN		48	/* baseline message, incl. trailer+MIC */

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

/* Init/reset the responder + rebuild the static MIB rows.  @mds_seed is the
 * MIB-Data-Sync boot value: a POISON that must NOT match the OLT's stored
 * lsync, so its ME2 audit mismatches and it re-provisions from MIB-Reset
 * (the X111W warm-readmit lesson; an on-wire MIB-Reset then zeroes it). */
void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed);

/* Process one DS baseline PDU -> fill @resp (48 bytes, trailer + MIC done).
 * Returns OMCI_LEN, or 0 when no response must be sent (runt / non-baseline /
 * AR clear, i.e. the OLT asked for no acknowledgement). */
int omci_onu_input(struct omci_onu *o, const u8 *req, unsigned int len, u8 *resp);

/* Autonomous VEIP (ME 329) operational-state-up AVC: the OLT never polls the
 * data MEs it created — it gates DOWNSTREAM user-data forwarding on this
 * report.  Fills @out (48 bytes, trailer + MIC done); returns OMCI_LEN. */
int omci_onu_emit_veip_up_avc(struct omci_onu *o, u8 *out);

#endif /* OMCI_RESPONDER_H */
