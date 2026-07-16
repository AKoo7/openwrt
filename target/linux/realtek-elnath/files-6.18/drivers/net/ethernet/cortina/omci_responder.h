/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * omci_responder.h — clean-room ONU G.988 OMCI baseline responder + ME model
 * for the RTL9607F "Elnath" GPON driver (Stage C of the GPON-WAN bring-up).
 *
 * Kernel port of the host oracle's responder (dev/rtl9607c-oracle), with the
 * on-wire facts aligned to the responder PROVEN against the same HSGQ-G008
 * OLT on the RTL9602C (realtek-luna rtl9602c_eth.c, reached Online/normal +
 * WAN end-to-end):
 *   - GET request attribute mask   = msg[8..9]  (contents start at octet 8)
 *   - MIB-Upload  reply row count  = resp[8..9] (NO result byte)
 *   - MIB-Upload-Next request seq  = msg[8..9]; reply = class[8..9] +
 *     inst[10..11] + mask[12..13] + values[14..39] (26 value bytes, no result)
 *   - Create body = msg+8; MIB-Data-Sync +1 per Create/Set/Delete, wrap
 *     255 -> 1; on-wire MIB-Reset zeroes it
 *   - MIC (bytes 44..47) = zlib CRC-32 over bytes 0..43, stored big-endian,
 *     computed in SOFTWARE (a zero/wrong MIC = the OLT silently drops the
 *     response and loops its GET audit)
 *   - autonomous AVC: TID=0, MT=0x11, changed-attr mask at [8..9], value
 *     from [10]
 * (The host oracle's mask-at-9/seq-from-inst layout is a host-sim-only
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
	/* spy counters (dump/probe capability is first-class, project rule) */
	u32	unhandled;		/* DS message types answered NOT_SUPPORTED */
	u32	avc_count;		/* autonomous AVC frames emitted */
	bool	avc_veip_up_sent;
};

/* Init/reset the responder + rebuild the static MIB rows.  @mds_seed is the
 * MIB-Data-Sync boot value: a POISON that must NOT match the OLT's stored
 * lsync, so its ME2 audit mismatches and it re-provisions from MIB-Reset
 * (the X111W warm-readmit lesson; an on-wire MIB-Reset then zeroes it). */
void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed);

/* Process one DS baseline PDU -> fill @resp (48 bytes, trailer + MIC done).
 * Returns OMCI_LEN, or 0 when no response must be sent (runt / non-baseline). */
int omci_onu_input(struct omci_onu *o, const u8 *req, unsigned int len, u8 *resp);

/* Autonomous VEIP (ME 329) operational-state-up AVC: the OLT never polls the
 * data MEs it created — it gates DOWNSTREAM user-data forwarding on this
 * report.  Fills @out (48 bytes, trailer + MIC done); returns OMCI_LEN. */
int omci_onu_emit_veip_up_avc(struct omci_onu *o, u8 *out);

#endif /* OMCI_RESPONDER_H */
