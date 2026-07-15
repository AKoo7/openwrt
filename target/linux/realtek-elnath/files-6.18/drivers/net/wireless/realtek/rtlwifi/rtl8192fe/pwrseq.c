// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2026  Realtek RTL8192FE clean-room driver authors */

#include "pwrseq.h"

/*
 * Each flow below is a concatenation of one or more transition macros from
 * pwrseq.h followed by the terminator.  rtl_hal_pwrseqcmdparsing() in the
 * rtlwifi core consumes these tables; hw.c selects the right flow for each
 * power-state change (power on, radio off, card disable/enable, suspend,
 * resume, hardware power-down, and the firmware-driven LPS enter/leave).
 *
 * The arrays are dimensioned by the *_STEPS upper bounds.  A transition
 * macro that emits fewer entries than its bound leaves the tail zeroed; the
 * PWR_CMD_END entry from RTL8192F_TRANS_END stops the parser before it
 * reaches any unused slot.
 */

/* Power-on: CARDEMU -> ACT. */
struct wlan_pwr_cfg rtl8192f_power_on_flow
		[RTL8192F_TRANS_CARDEMU_TO_ACT_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_CARDEMU_TO_ACT
	RTL8192F_TRANS_END
};

/* Radio off (GPIO / RF disable): ACT -> CARDEMU. */
struct wlan_pwr_cfg rtl8192f_radio_off_flow
		[RTL8192F_TRANS_ACT_TO_CARDEMU_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_ACT_TO_CARDEMU
	RTL8192F_TRANS_END
};

/* Card disable: ACT -> CARDEMU -> CARDDIS. */
struct wlan_pwr_cfg rtl8192f_card_disable_flow
		[RTL8192F_TRANS_ACT_TO_CARDEMU_STEPS +
		 RTL8192F_TRANS_CARDEMU_TO_PDN_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_ACT_TO_CARDEMU
	RTL8192F_TRANS_CARDEMU_TO_CARDDIS
	RTL8192F_TRANS_END
};

/* Card enable: CARDDIS -> CARDEMU -> ACT. */
struct wlan_pwr_cfg rtl8192f_card_enable_flow
		[RTL8192F_TRANS_ACT_TO_CARDEMU_STEPS +
		 RTL8192F_TRANS_CARDEMU_TO_PDN_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_CARDDIS_TO_CARDEMU
	RTL8192F_TRANS_CARDEMU_TO_ACT
	RTL8192F_TRANS_END
};

/* Suspend: ACT -> CARDEMU -> SUS. */
struct wlan_pwr_cfg rtl8192f_suspend_flow
		[RTL8192F_TRANS_ACT_TO_CARDEMU_STEPS +
		 RTL8192F_TRANS_CARDEMU_TO_SUS_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_ACT_TO_CARDEMU
	RTL8192F_TRANS_CARDEMU_TO_SUS
	RTL8192F_TRANS_END
};

/* Resume: SUS -> CARDEMU -> ACT. */
struct wlan_pwr_cfg rtl8192f_resume_flow
		[RTL8192F_TRANS_ACT_TO_CARDEMU_STEPS +
		 RTL8192F_TRANS_CARDEMU_TO_SUS_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_SUS_TO_CARDEMU
	RTL8192F_TRANS_CARDEMU_TO_ACT
	RTL8192F_TRANS_END
};

/* Hardware power-down: ACT -> CARDEMU -> PDN. */
struct wlan_pwr_cfg rtl8192f_hwpdn_flow
		[RTL8192F_TRANS_ACT_TO_CARDEMU_STEPS +
		 RTL8192F_TRANS_CARDEMU_TO_PDN_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_ACT_TO_CARDEMU
	RTL8192F_TRANS_CARDEMU_TO_PDN
	RTL8192F_TRANS_END
};

/* Enter LPS (firmware behaviour): ACT -> LPS. */
struct wlan_pwr_cfg rtl8192f_enter_lps_flow
		[RTL8192F_TRANS_ACT_TO_LPS_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_ACT_TO_LPS
	RTL8192F_TRANS_END
};

/* Leave LPS (firmware behaviour): LPS -> ACT. */
struct wlan_pwr_cfg rtl8192f_leave_lps_flow
		[RTL8192F_TRANS_LPS_TO_ACT_STEPS +
		 RTL8192F_TRANS_END_STEPS] = {
	RTL8192F_TRANS_LPS_TO_ACT
	RTL8192F_TRANS_END
};
