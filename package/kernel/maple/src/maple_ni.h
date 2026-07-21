/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GPON-NI (PON port) management ops — port-level status, counters, and config.
 * Struct layouts recovered from trmux.ko DWARF (pahole):
 *   bcmolt_gpon_ni_key        = {pon_ni}            (1 B)
 *   bcmolt_gpon_ni_cfg_data   = 340 B (status, active ONUs, distance, FEC, ...)
 *   bcmolt_gpon_ni_stat_data  = 272 B (rx/tx gem, ploam, omci, bip8, fec, ...)
 * The full cfg is large; we expose only the most useful management fields.
 * See docs/onu-management-data-contract.md + docs/decomp/gpcapi-handlers-reference.md.
 */
#ifndef MAPLE_NI_H
#define MAPLE_NI_H

#include <linux/types.h>
#include "maple_regs.h"

struct maple_dev;

/* bcmolt_gpon_ni_key (1 B). */
struct maple_ni_key {
	u8	pon_ni;
} __packed;

/* bcmolt_gpon_ni_cfg_data — host-order view of the management-useful subset.
 * Full BAL struct is 340 B; we keep the first ~64 B which cover the
 * status/ONU-count/distance/FEC fields an operator needs. Remaining fields
 * (dba, rogue-onu, upgrade, etc.) are left as raw tail bytes. */
struct maple_ni_cfg {
	u64	pon_status;			/*   0 — operation state */
	u8	available_bandwidth[12];	/*   8 */
	u16	number_of_active_onus;		/*  20 */
	u16	number_of_active_standby_onus;	/*  22 */
	u8	prbs_status[8];			/*  24 */
	u64	pon_distance;			/*  32 — reach (cm) */
	u32	ranging_window_size;		/*  40 */
	u32	preassigned_equalization_delay;	/*  44 */
	u32	eqd_cycles_number;		/*  48 */
	u8	power_level[8];			/*  52 */
	u32	ds_fec_mode;			/*  60 — control state */
} __packed;

/* bcmolt_gpon_ni_stat_data (272 B) — aggregate PON counters, host-order. */
struct maple_ni_stat {
	u64 fec_codewords, fec_codewords_uncorrected;
	u64 bip8_bytes, bip8_errors;
	u64 rx_gem_packets, rx_gem_dropped, rx_gem_idle, rx_gem_corrected;
	u64 rx_gem_illegal, rx_allocations_valid, rx_allocations_invalid;
	u64 rx_allocations_disabled, rx_ploams, rx_ploams_non_idle;
	u64 rx_ploams_error, rx_ploams_dropped, rx_cpu, rx_omci;
	u64 rx_omci_packets_crc_error, rx_dropped_too_short, rx_dropped_too_long;
	u64 rx_crc_errors, rx_key_errors, rx_fragments_errors;
	u64 rx_packets_dropped, tx_gem, tx_ploams, tx_gem_fragments;
	u64 tx_cpu, tx_omci;
	u8  tx_cpu_omci_packets_dropped;
	u8  _pad[7];
	u64 tx_dropped_illegal_length, tx_dropped_tpid_miss, tx_dropped_vid_miss;
} __packed;

int maple_ni_get_cfg(struct maple_dev *mdev, u8 pon_ni, struct maple_ni_cfg *cfg);
int maple_ni_get_stat(struct maple_dev *mdev, u8 pon_ni, struct maple_ni_stat *stat);

/* Raw BE wire bytes from the MAC. */
int maple_ni_get_cfg_raw(struct maple_dev *mdev, u8 pon_ni,
			 void *buf, size_t buflen);
int maple_ni_get_stat_raw(struct maple_dev *mdev, u8 pon_ni,
			  void *buf, size_t buflen);

#endif /* MAPLE_NI_H */
