/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GPON ONU management ops — the ONU listing / bandwidth / block-unblock layer.
 * Struct layouts + operation IDs recovered from trmux.ko DWARF (pahole) — see
 * docs/onu-management-data-contract.md. Builds on maple_bal_call().
 */
#ifndef MAPLE_ONU_H
#define MAPLE_ONU_H

#include <linux/types.h>

#include "maple_regs.h"	/* obj/group/op IDs (shared with oracle test) */

struct maple_dev;

/* BAL instance = bcmolt_gpon_onu_key {pon_ni:8, pad:8, onu_id:16} packed LE. */
static inline u8 maple_onu_instance(u8 pon_ni, u16 onu_id)
{
	(void)onu_id;
	/* The vendor packs (pon_ni, onu_id) into the 1-byte instance field of
	 * bcmtr_hdr by hashing; for full fidelity the instance is carried in the
	 * key object body. We use pon_ni here and put the full key in the body. */
	return pon_ni;
}

/* bcmolt_gpon_onu_key (4 B). */
struct maple_onu_key {
	u8	pon_ni;
	u8	_pad;
	u16	onu_id;
} __packed;

/* bcmolt_gpon_onu_set_onu_state_data (4 B) — block/unblock body. */
struct maple_onu_set_state {
	u32	onu_operation;
} __packed;

/* bcmolt_gpon_onu_stat_data (160 B) — bandwidth + error counters.
 * Host-order in memory (unpacked from the BE wire by maple_codec.h). */
struct maple_onu_stat {
	u64 fec_codewords, fec_bytes_corrected, fec_codewords_corrected;
	u64 fec_codewords_uncorrected, bip8_bytes, bip8_errors;
	u64 rx_ploams_crc_error, rx_ploams_non_idle, positive_drift;
	u64 negative_drift, rx_omci, rx_omci_crc_error, ber_reported;
	u64 unreceived_burst, lcdg_errors, rdi_errors;
	u64 rx_bytes;	/* downstream bytes  */
	u64 rx_packets;	/* downstream packets */
	u64 tx_bytes;	/* upstream bytes    */
	u64 tx_packets;	/* upstream packets  */
} __packed;

/* bcmolt_gpon_onu_cfg_data (140 B) — the ONT Basic-Info record, host-order.
 * Nested sub-structs (alarm_state 48 B, the two list headers, etc.) are kept
 * as raw fields so the layout matches the DWARF byte offsets exactly. */
struct maple_onu_cfg {
	u32	onu_state;		/*   0 */
	u32	onu_old_state;		/*   4 */
	u8	serial_number[8];	/*   8 vendor_id[4]+vendor_specific[4] */
	u8	password[10];		/*  16 */
	u8	auto_password_learning;	/*  26 */
	u8	us_fec;			/*  27 */
	u16	omci_port_id;		/*  28 */
	u8	_pad30[2];		/*  30 */
	u32	ds_ber_reporting_interval; /* 32 */
	u8	aes_encryption_key[16];	/*  36 */
	u32	alarm_state[12];		/*  52  LOSi/LOFi/LOAMi/DGi/TIWi/DOWi/
					 *       SUFi/SFi/SDi/DFi/LOAi/LOKi */
	u32	ranging_time;		/* 100  -> Distance(m) */
	u32	disabled_after_discovery;/* 104 */
	u32	deactivation_reason;	/* 108  -> Last down cause */
	u8	all_gem_ports[8];	/* 112  list header */
	u8	all_allocs[8];		/* 120  list header */
	u8	onu_ps_type_c;		/* 128 */
	u8	_pad129[3];		/* 129 */
	u8	extended_guard_time[8];/* 132 */
} __packed;

int maple_onu_set_state(struct maple_dev *mdev, u8 pon_ni, u16 onu_id, u32 op);
int maple_onu_get_stat(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
		       struct maple_onu_stat *stat);
int maple_onu_get_cfg(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
		      struct maple_onu_cfg *cfg);

/* Raw BE wire bytes from the MAC (endian-independent ABI: caller unpacks via
 * maple_codec.h). Returns the byte length or negative errno. */
int maple_onu_get_cfg_raw(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
			  void *buf, size_t buflen);
int maple_onu_get_stat_raw(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
			   void *buf, size_t buflen);

static inline int maple_onu_block(struct maple_dev *m, u8 p, u16 id)
{
	return maple_onu_set_state(m, p, id, MAPLE_ONU_OP_DISABLE);
}
static inline int maple_onu_unblock(struct maple_dev *m, u8 p, u16 id)
{
	return maple_onu_set_state(m, p, id, MAPLE_ONU_OP_ENABLE);
}

#endif /* MAPLE_ONU_H */
