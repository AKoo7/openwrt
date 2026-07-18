/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2019-2022  Realtek Corporation
 */

#ifndef __RTW89_8192XB_RFK_H__
#define __RTW89_8192XB_RFK_H__

#include "core.h"

void rtw8192xb_mcc_get_ch_info(struct rtw89_dev *rtwdev, enum rtw89_phy_idx phy);
void rtw8192xb_rck(struct rtw89_dev *rtwdev);
void rtw8192xb_dack(struct rtw89_dev *rtwdev, enum rtw89_chanctx_idx chanctx_idx);
void rtw8192xb_iqk(struct rtw89_dev *rtwdev, enum rtw89_phy_idx phy_idx,
		  enum rtw89_chanctx_idx chanctx_idx);
void rtw8192xb_rx_dck(struct rtw89_dev *rtwdev, enum rtw89_phy_idx phy_idx, bool is_afe);
void rtw8192xb_rx_dck_track(struct rtw89_dev *rtwdev);
void rtw8192xb_dpk_init(struct rtw89_dev *rtwdev);
void rtw8192xb_dpk(struct rtw89_dev *rtwdev, enum rtw89_phy_idx phy_idx,
		  enum rtw89_chanctx_idx chanctx_idx);
void rtw8192xb_dpk_track(struct rtw89_dev *rtwdev);
void rtw8192xb_tssi(struct rtw89_dev *rtwdev, enum rtw89_phy_idx phy,
		   enum rtw89_chanctx_idx chanctx_idx);
void rtw8192xb_tssi_scan(struct rtw89_dev *rtwdev, enum rtw89_phy_idx phy,
			const struct rtw89_chan *chan);
void rtw8192xb_tssi_cont_en_phyidx(struct rtw89_dev *rtwdev, bool en, u8 phy_idx,
				  const struct rtw89_chan *chan);
void rtw8192xb_wifi_scan_notify(struct rtw89_dev *rtwdev, bool scan_start,
			       enum rtw89_phy_idx phy_idx);
void rtw8192xb_set_channel_rf(struct rtw89_dev *rtwdev,
			     const struct rtw89_chan *chan,
			     enum rtw89_phy_idx phy_idx);
void rtw8192xb_lck_init(struct rtw89_dev *rtwdev);
void rtw8192xb_lck_track(struct rtw89_dev *rtwdev);
void rtw8192xb_rfk_chanctx_cb(struct rtw89_dev *rtwdev,
			     enum rtw89_chanctx_state state);

#endif
