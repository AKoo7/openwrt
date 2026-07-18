/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2026  CatchChallenger project
 *
 * RTL8192XB: 2.4 GHz-only 2x2 802.11ax (Wi-Fi 6) PCIe chip, HW chip id 0x53.
 * MAC-domain twin of the RTL8852C (same "g6" generation, V1/HAXI register
 * set); single band, single CMAC, 20/40 MHz only.
 *
 * The logical efuse map is byte-identical to the RTL8852C's in every field
 * this struct names; the 8852C's 6 GHz blocks are unprogrammed padding here
 * (2.4 GHz-only chip), kept as reserved bytes so the offsets line up:
 * TSSI path A/B blocks @0x210/0x23a, board fields @0x2b8..0x2d6,
 * PCIe MAC address @0x400.
 */

#ifndef __RTW89_8192XB_H__
#define __RTW89_8192XB_H__

#include "core.h"

#define RF_PATH_NUM_8192XB 2
#define BB_PATH_NUM_8192XB 2

struct rtw8192xb_u_efuse {
	u8 rsvd[0x38];
	u8 mac_addr[ETH_ALEN];
};

struct rtw8192xb_e_efuse {
	u8 mac_addr[ETH_ALEN];
};

struct rtw8192xb_tssi_offset {
	u8 cck_tssi[TSSI_CCK_CH_GROUP_NUM];
	u8 bw40_tssi[TSSI_MCS_2G_CH_GROUP_NUM];
	u8 rsvd[7];
	u8 bw40_1s_tssi_5g[TSSI_MCS_5G_CH_GROUP_NUM];
} __packed;

struct rtw8192xb_efuse {
	u8 rsvd[0x210];
	struct rtw8192xb_tssi_offset path_a_tssi;	/* 0x210 */
	u8 rsvd1[10];
	struct rtw8192xb_tssi_offset path_b_tssi;	/* 0x23a */
	u8 rsvd2[94];
	u8 channel_plan;				/* 0x2b8 */
	u8 xtal_k;
	u8 rsvd3;
	u8 iqk_lck;
	u8 rsvd4[5];
	u8 reg_setting:2;
	u8 tx_diversity:1;
	u8 rx_diversity:2;
	u8 ac_mode:1;
	u8 module_type:2;
	u8 rsvd5;
	u8 shared_ant:1;
	u8 coex_type:3;
	u8 ant_iso:1;
	u8 radio_on_off:1;
	u8 rsvd6:2;
	u8 eeprom_version;
	u8 customer_id;
	u8 tx_bb_swing_2g;
	u8 tx_bb_swing_5g;
	u8 tx_cali_pwr_trk_mode;
	u8 trx_path_selection;
	u8 rfe_type;					/* 0x2ca */
	u8 country_code[2];
	u8 rsvd7[3];
	u8 path_a_therm;				/* 0x2d0 */
	u8 path_b_therm;
	u8 rsvd8[2];
	u8 rx_gain_2g_ofdm;				/* 0x2d4 */
	u8 rsvd9;
	u8 rx_gain_2g_cck;				/* 0x2d6 */
	u8 rsvd10;
	u8 rx_gain_5g_low;
	u8 rsvd11;
	u8 rx_gain_5g_mid;
	u8 rsvd12;
	u8 rx_gain_5g_high;
	u8 rsvd13[35];
	/* 0x300-0x3ff: the 8852C's 6 GHz TSSI/rx-gain blocks; unused padding
	 * on this 2.4 GHz-only chip
	 */
	u8 rsvd14[0x100];
	union {						/* 0x400 */
		struct rtw8192xb_u_efuse u;
		struct rtw8192xb_e_efuse e;
	};
} __packed;

extern const struct rtw89_chip_info rtw8192xb_chip_info;

#endif
