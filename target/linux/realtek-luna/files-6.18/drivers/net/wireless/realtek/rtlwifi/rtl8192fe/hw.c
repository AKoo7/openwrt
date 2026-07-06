// SPDX-License-Identifier: GPL-2.0
/* Clean-room rtl8192fe sub-driver for the Realtek RTL8192F (2T2R 802.11n). */

#include "../wifi.h"
#include "../efuse.h"
#include "../base.h"
#include "../regd.h"
#include "../cam.h"
#include "../ps.h"
#include "../pci.h"
#include "reg.h"
#include "def.h"
#include "phy.h"
#include "dm.h"
#include "fw.h"
#include "led.h"
#include "hw.h"
#include "../pwrseqcmd.h"
#include "pwrseq.h"

#define LLT_CONFIG	5

/* Crystal load-cap trim. This board's WiFi EFUSE is blank (crystalcap reads
 * 0x00), so the per-chip XCAP comes from the board factory cal instead: the
 * flash apmib HW_WLAN0_11N_XCAP value (0x47) is injected into
 * efuse.crystalcap by _rtl92fe_apply_board_cal() and applied by the AFE trim
 * in rtl92fe_hw_init(). Default xtal_cap = -1 means "use the (now cal-filled)
 * EFUSE value"; set xtal_cap >= 0 to override for per-board tuning. */
static int xtal_cap = -1;
module_param(xtal_cap, int, 0644);
MODULE_PARM_DESC(xtal_cap, "RTL8192F crystal load-cap trim 0..0x3f (<0 = use EFUSE/board-cal value)");

static void _rtl92fe_set_bcn_ctrl_reg(struct ieee80211_hw *hw,
				      u8 set_bits, u8 clear_bits)
{
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	rtlpci->reg_bcn_ctrl_val |= set_bits;
	rtlpci->reg_bcn_ctrl_val &= ~clear_bits;

	rtl_write_byte(rtlpriv, REG_BCN_CTRL, (u8)rtlpci->reg_bcn_ctrl_val);
}

static void _rtl92fe_stop_tx_beacon(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 tmp;

	tmp = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2);
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2, tmp & (~BIT(6)));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT + 1, 0x64);
	tmp = rtl_read_byte(rtlpriv, REG_TBTT_PROHIBIT + 2);
	tmp &= ~(BIT(0));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT + 2, tmp);
}

static void _rtl92fe_resume_tx_beacon(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 tmp;

	tmp = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2);
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2, tmp | BIT(6));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT + 1, 0xff);
	tmp = rtl_read_byte(rtlpriv, REG_TBTT_PROHIBIT + 2);
	tmp |= BIT(0);
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT + 2, tmp);
}

static void _rtl92fe_enable_bcn_sub_func(struct ieee80211_hw *hw)
{
	_rtl92fe_set_bcn_ctrl_reg(hw, 0, BIT(1));
}

static void _rtl92fe_disable_bcn_sub_func(struct ieee80211_hw *hw)
{
	_rtl92fe_set_bcn_ctrl_reg(hw, BIT(1), 0);
}

static void _rtl92fe_set_fw_clock_on(struct ieee80211_hw *hw,
				     u8 rpwm_val, bool b_need_turn_off_ckk)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	bool b_support_remote_wake_up;
	u32 count = 0, isr_regaddr, content;
	bool b_schedule_timer = b_need_turn_off_ckk;

	rtlpriv->cfg->ops->get_hw_reg(hw, HAL_DEF_WOWLAN,
				      (u8 *)(&b_support_remote_wake_up));

	if (!rtlhal->fw_ready)
		return;
	if (!rtlpriv->psc.fw_current_inpsmode)
		return;

	while (1) {
		spin_lock_bh(&rtlpriv->locks.fw_ps_lock);
		if (rtlhal->fw_clk_change_in_progress) {
			while (rtlhal->fw_clk_change_in_progress) {
				spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
				count++;
				udelay(100);
				if (count > 1000)
					return;
				spin_lock_bh(&rtlpriv->locks.fw_ps_lock);
			}
			spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
		} else {
			rtlhal->fw_clk_change_in_progress = false;
			spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
			break;
		}
	}

	if (IS_IN_LOW_POWER_STATE_92F(rtlhal->fw_ps_state)) {
		rtlpriv->cfg->ops->get_hw_reg(hw, HW_VAR_SET_RPWM,
					      (u8 *)(&rpwm_val));
		if (FW_PS_IS_ACK(rpwm_val)) {
			isr_regaddr = REG_HISR;
			content = rtl_read_dword(rtlpriv, isr_regaddr);
			while (!(content & IMR_CPWM) && (count < 500)) {
				udelay(50);
				count++;
				content = rtl_read_dword(rtlpriv, isr_regaddr);
			}

			if (content & IMR_CPWM) {
				rtl_write_word(rtlpriv, isr_regaddr, 0x0100);
				rtlhal->fw_ps_state = FW_PS_STATE_RF_ON_92F;
				rtl_dbg(rtlpriv, COMP_POWER, DBG_LOUD,
					"Receive CPWM INT!!! PSState = %X\n",
					rtlhal->fw_ps_state);
			}
		}

		spin_lock_bh(&rtlpriv->locks.fw_ps_lock);
		rtlhal->fw_clk_change_in_progress = false;
		spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
		if (b_schedule_timer) {
			mod_timer(&rtlpriv->works.fw_clockoff_timer,
				  jiffies + MSECS(10));
		}
	} else  {
		spin_lock_bh(&rtlpriv->locks.fw_ps_lock);
		rtlhal->fw_clk_change_in_progress = false;
		spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
	}
}

static void _rtl92fe_set_fw_clock_off(struct ieee80211_hw *hw, u8 rpwm_val)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	struct rtl8192_tx_ring *ring;
	enum rf_pwrstate rtstate;
	bool b_schedule_timer = false;
	u8 queue;

	if (!rtlhal->fw_ready)
		return;
	if (!rtlpriv->psc.fw_current_inpsmode)
		return;
	if (!rtlhal->allow_sw_to_change_hwclc)
		return;

	rtlpriv->cfg->ops->get_hw_reg(hw, HW_VAR_RF_STATE, (u8 *)(&rtstate));
	if (rtstate == ERFOFF || rtlpriv->psc.inactive_pwrstate == ERFOFF)
		return;

	for (queue = 0; queue < RTL_PCI_MAX_TX_QUEUE_COUNT; queue++) {
		ring = &rtlpci->tx_ring[queue];
		if (skb_queue_len(&ring->queue)) {
			b_schedule_timer = true;
			break;
		}
	}

	if (b_schedule_timer) {
		mod_timer(&rtlpriv->works.fw_clockoff_timer,
			  jiffies + MSECS(10));
		return;
	}

	if (FW_PS_STATE(rtlhal->fw_ps_state) != FW_PS_STATE_RF_OFF_LOW_PWR) {
		spin_lock_bh(&rtlpriv->locks.fw_ps_lock);
		if (!rtlhal->fw_clk_change_in_progress) {
			rtlhal->fw_clk_change_in_progress = true;
			spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
			rtlhal->fw_ps_state = FW_PS_STATE(rpwm_val);
			rtl_write_word(rtlpriv, REG_HISR, 0x0100);
			rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_SET_RPWM,
						      (u8 *)(&rpwm_val));
			spin_lock_bh(&rtlpriv->locks.fw_ps_lock);
			rtlhal->fw_clk_change_in_progress = false;
			spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
		} else {
			spin_unlock_bh(&rtlpriv->locks.fw_ps_lock);
			mod_timer(&rtlpriv->works.fw_clockoff_timer,
				  jiffies + MSECS(10));
		}
	}
}

static void _rtl92fe_set_fw_ps_rf_on(struct ieee80211_hw *hw)
{
	u8 rpwm_val = 0;

	rpwm_val |= (FW_PS_STATE_RF_OFF_92F | FW_PS_ACK);
	_rtl92fe_set_fw_clock_on(hw, rpwm_val, true);
}

static void _rtl92fe_set_fw_ps_rf_off_low_power(struct ieee80211_hw *hw)
{
	u8 rpwm_val = 0;

	rpwm_val |= FW_PS_STATE_RF_OFF_LOW_PWR;
	_rtl92fe_set_fw_clock_off(hw, rpwm_val);
}

void rtl92fe_fw_clk_off_timer_callback(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;

	_rtl92fe_set_fw_ps_rf_off_low_power(hw);
}

static void _rtl92fe_fwlps_leave(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	bool fw_current_inps = false;
	u8 rpwm_val = 0, fw_pwrmode = FW_PS_ACTIVE_MODE;

	if (ppsc->low_power_enable) {
		rpwm_val = (FW_PS_STATE_ALL_ON_92F | FW_PS_ACK);/* RF on */
		_rtl92fe_set_fw_clock_on(hw, rpwm_val, false);
		rtlhal->allow_sw_to_change_hwclc = false;
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_H2C_FW_PWRMODE,
					      (u8 *)(&fw_pwrmode));
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_FW_PSMODE_STATUS,
					      (u8 *)(&fw_current_inps));
	} else {
		rpwm_val = FW_PS_STATE_ALL_ON_92F;	/* RF on */
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_SET_RPWM,
					      (u8 *)(&rpwm_val));
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_H2C_FW_PWRMODE,
					      (u8 *)(&fw_pwrmode));
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_FW_PSMODE_STATUS,
					      (u8 *)(&fw_current_inps));
	}
}

static void _rtl92fe_fwlps_enter(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	bool fw_current_inps = true;
	u8 rpwm_val;

	if (ppsc->low_power_enable) {
		rpwm_val = FW_PS_STATE_RF_OFF_LOW_PWR;	/* RF off */
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_FW_PSMODE_STATUS,
					      (u8 *)(&fw_current_inps));
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_H2C_FW_PWRMODE,
					      (u8 *)(&ppsc->fwctrl_psmode));
		rtlhal->allow_sw_to_change_hwclc = true;
		_rtl92fe_set_fw_clock_off(hw, rpwm_val);
	} else {
		rpwm_val = FW_PS_STATE_RF_OFF_92F;	/* RF off */
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_FW_PSMODE_STATUS,
					      (u8 *)(&fw_current_inps));
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_H2C_FW_PWRMODE,
					      (u8 *)(&ppsc->fwctrl_psmode));
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_SET_RPWM,
					      (u8 *)(&rpwm_val));
	}
}

void rtl92fe_get_hw_reg(struct ieee80211_hw *hw, u8 variable, u8 *val)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));

	switch (variable) {
	case HW_VAR_RCR:
		*((u32 *)(val)) = rtlpci->receive_config;
		break;
	case HW_VAR_RF_STATE:
		*((enum rf_pwrstate *)(val)) = ppsc->rfpwr_state;
		break;
	case HW_VAR_FWLPS_RF_ON:{
			enum rf_pwrstate rfstate;
			u32 val_rcr;

			rtlpriv->cfg->ops->get_hw_reg(hw, HW_VAR_RF_STATE,
						      (u8 *)(&rfstate));
			if (rfstate == ERFOFF) {
				*((bool *)(val)) = true;
			} else {
				val_rcr = rtl_read_dword(rtlpriv, REG_RCR);
				val_rcr &= 0x00070000;
				if (val_rcr)
					*((bool *)(val)) = false;
				else
					*((bool *)(val)) = true;
			}
		}
		break;
	case HW_VAR_FW_PSMODE_STATUS:
		*((bool *)(val)) = ppsc->fw_current_inpsmode;
		break;
	case HW_VAR_CORRECT_TSF:{
		u64 tsf;
		u32 *ptsf_low = (u32 *)&tsf;
		u32 *ptsf_high = ((u32 *)&tsf) + 1;

		*ptsf_high = rtl_read_dword(rtlpriv, (REG_TSFTR + 4));
		*ptsf_low = rtl_read_dword(rtlpriv, REG_TSFTR);

		*((u64 *)(val)) = tsf;
		}
		break;
	case HAL_DEF_WOWLAN:
		break;
	default:
		rtl_dbg(rtlpriv, COMP_ERR, DBG_DMESG,
			"switch case %#x not processed\n", variable);
		break;
	}
}

static void _rtl92fe_download_rsvd_page(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 tmp_regcr, tmp_reg422;
	u8 bcnvalid_reg, txbc_reg;
	u8 count = 0, dlbcn_count = 0;
	bool b_recover = false;

	/* Set REG_CR bit 8. DMA beacon by SW. */
	tmp_regcr = rtl_read_byte(rtlpriv, REG_CR + 1);
	rtl_write_byte(rtlpriv, REG_CR + 1, tmp_regcr | BIT(0));

	/* Disable Hw protection for the window reserved for Hw beacon TX so
	 * that the reserved-page download does not collide with it.
	 */
	_rtl92fe_set_bcn_ctrl_reg(hw, 0, BIT(3));
	_rtl92fe_set_bcn_ctrl_reg(hw, BIT(4), 0);

	/* FWHW_TXQ_CTRL 0x422[6]=0: tell HW this is not a real beacon. */
	tmp_reg422 = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2);
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2, tmp_reg422 & (~BIT(6)));

	if (tmp_reg422 & BIT(6))
		b_recover = true;

	do {
		/* Clear beacon-valid check bit */
		bcnvalid_reg = rtl_read_byte(rtlpriv, REG_DWBCN0_CTRL + 2);
		rtl_write_byte(rtlpriv, REG_DWBCN0_CTRL + 2,
			       bcnvalid_reg | BIT(0));

		/* download rsvd page */
		rtl92fe_set_fw_rsvdpagepkt(hw, false);

		txbc_reg = rtl_read_byte(rtlpriv, REG_MGQ_TXBD_NUM + 3);
		count = 0;
		while ((txbc_reg & BIT(4)) && count < 20) {
			count++;
			udelay(10);
			txbc_reg = rtl_read_byte(rtlpriv, REG_MGQ_TXBD_NUM + 3);
		}
		rtl_write_byte(rtlpriv, REG_MGQ_TXBD_NUM + 3,
			       txbc_reg | BIT(4));

		/* check rsvd page download OK. */
		bcnvalid_reg = rtl_read_byte(rtlpriv, REG_DWBCN0_CTRL + 2);
		count = 0;
		while (!(bcnvalid_reg & BIT(0)) && count < 20) {
			count++;
			udelay(50);
			bcnvalid_reg = rtl_read_byte(rtlpriv,
						     REG_DWBCN0_CTRL + 2);
		}

		if (bcnvalid_reg & BIT(0))
			rtl_write_byte(rtlpriv, REG_DWBCN0_CTRL + 2, BIT(0));

		dlbcn_count++;
	} while (!(bcnvalid_reg & BIT(0)) && dlbcn_count < 5);

	if (!(bcnvalid_reg & BIT(0)))
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
			"Download RSVD page failed!\n");

	/* Enable Bcn */
	_rtl92fe_set_bcn_ctrl_reg(hw, BIT(3), 0);
	_rtl92fe_set_bcn_ctrl_reg(hw, 0, BIT(4));

	if (b_recover)
		rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 2, tmp_reg422);

	tmp_regcr = rtl_read_byte(rtlpriv, REG_CR + 1);
	rtl_write_byte(rtlpriv, REG_CR + 1, tmp_regcr & (~BIT(0)));
}

void rtl92fe_set_hw_reg(struct ieee80211_hw *hw, u8 variable, u8 *val)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	struct rtl_efuse *efuse = rtl_efuse(rtl_priv(hw));
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	u8 idx;

	switch (variable) {
	case HW_VAR_ETHER_ADDR:
		for (idx = 0; idx < ETH_ALEN; idx++)
			rtl_write_byte(rtlpriv, (REG_MACID + idx), val[idx]);
		break;
	case HW_VAR_BASIC_RATE:{
		u16 b_rate_cfg = ((u16 *)val)[0];

		b_rate_cfg = b_rate_cfg & 0x15f;
		b_rate_cfg |= 0x01;
		b_rate_cfg = (b_rate_cfg | 0xd) & (~BIT(1));
		rtl_write_byte(rtlpriv, REG_RRSR, b_rate_cfg & 0xff);
		rtl_write_byte(rtlpriv, REG_RRSR + 1, (b_rate_cfg >> 8) & 0xff);
		break; }
	case HW_VAR_BSSID:
		for (idx = 0; idx < ETH_ALEN; idx++)
			rtl_write_byte(rtlpriv, (REG_BSSID + idx), val[idx]);
		break;
	case HW_VAR_SIFS:
		rtl_write_byte(rtlpriv, REG_SIFS_CTX + 1, val[0]);
		rtl_write_byte(rtlpriv, REG_SIFS_TRX + 1, val[1]);

		rtl_write_byte(rtlpriv, REG_SPEC_SIFS + 1, val[0]);
		rtl_write_byte(rtlpriv, REG_MAC_SPEC_SIFS + 1, val[0]);

		if (!mac->ht_enable)
			rtl_write_word(rtlpriv, REG_RESP_SIFS_OFDM, 0x0e0e);
		else
			rtl_write_word(rtlpriv, REG_RESP_SIFS_OFDM,
				       *((u16 *)val));
		break;
	case HW_VAR_SLOT_TIME:{
		u8 e_aci;

		rtl_dbg(rtlpriv, COMP_MLME, DBG_TRACE,
			"HW_VAR_SLOT_TIME %x\n", val[0]);

		rtl_write_byte(rtlpriv, REG_SLOT, val[0]);

		for (e_aci = 0; e_aci < AC_MAX; e_aci++) {
			rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_AC_PARAM,
						      (u8 *)(&e_aci));
		}
		break; }
	case HW_VAR_ACK_PREAMBLE:{
		u8 reg_tmp;
		u8 short_preamble = (bool)(*(u8 *)val);

		reg_tmp = (rtlpriv->mac80211.cur_40_prime_sc) << 5;
		if (short_preamble)
			reg_tmp |= 0x80;
		rtl_write_byte(rtlpriv, REG_RRSR + 2, reg_tmp);
		rtlpriv->mac80211.short_preamble = short_preamble;
		}
		break;
	case HW_VAR_WPA_CONFIG:
		rtl_write_byte(rtlpriv, REG_SECCFG, *((u8 *)val));
		break;
	case HW_VAR_AMPDU_FACTOR:{
		u8 regtoset_normal[4] = { 0x41, 0xa8, 0x72, 0xb9 };
		u8 fac;
		u8 *reg = NULL;
		u8 i = 0;

		reg = regtoset_normal;

		fac = *((u8 *)val);
		if (fac <= 3) {
			fac = (1 << (fac + 2));
			if (fac > 0xf)
				fac = 0xf;
			for (i = 0; i < 4; i++) {
				if ((reg[i] & 0xf0) > (fac << 4))
					reg[i] = (reg[i] & 0x0f) |
						(fac << 4);
				if ((reg[i] & 0x0f) > fac)
					reg[i] = (reg[i] & 0xf0) | fac;
				rtl_write_byte(rtlpriv,
					       (REG_AGGLEN_LMT + i),
					       reg[i]);
			}
			rtl_dbg(rtlpriv, COMP_MLME, DBG_LOUD,
				"Set HW_VAR_AMPDU_FACTOR:%#x\n", fac);
		}
		}
		break;
	case HW_VAR_AC_PARAM:{
		u8 e_aci = *((u8 *)val);

		if (rtlpci->acm_method != EACMWAY2_SW)
			rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_ACM_CTRL,
						      (u8 *)(&e_aci));
		}
		break;
	case HW_VAR_ACM_CTRL:{
		u8 e_aci = *((u8 *)val);
		union aci_aifsn *aifs = (union aci_aifsn *)(&mac->ac[0].aifs);

		u8 acm = aifs->f.acm;
		u8 acm_ctrl = rtl_read_byte(rtlpriv, REG_ACMHWCTRL);

		acm_ctrl = acm_ctrl | ((rtlpci->acm_method == 2) ? 0x0 : 0x1);

		if (acm) {
			switch (e_aci) {
			case AC0_BE:
				acm_ctrl |= ACMHW_BEQEN;
				break;
			case AC2_VI:
				acm_ctrl |= ACMHW_VIQEN;
				break;
			case AC3_VO:
				acm_ctrl |= ACMHW_VOQEN;
				break;
			default:
				rtl_dbg(rtlpriv, COMP_ERR, DBG_WARNING,
					"HW_VAR_ACM_CTRL acm set failed: eACI is %d\n",
					acm);
				break;
			}
		} else {
			switch (e_aci) {
			case AC0_BE:
				acm_ctrl &= (~ACMHW_BEQEN);
				break;
			case AC2_VI:
				acm_ctrl &= (~ACMHW_VIQEN);
				break;
			case AC3_VO:
				acm_ctrl &= (~ACMHW_VOQEN);
				break;
			default:
				rtl_dbg(rtlpriv, COMP_ERR, DBG_DMESG,
					"switch case %#x not processed\n",
					e_aci);
				break;
			}
		}

		rtl_dbg(rtlpriv, COMP_QOS, DBG_TRACE,
			"SetHwReg8192pci(): [HW_VAR_ACM_CTRL] Write 0x%X\n",
			acm_ctrl);
		rtl_write_byte(rtlpriv, REG_ACMHWCTRL, acm_ctrl);
		}
		break;
	case HW_VAR_RCR:{
		rtl_write_dword(rtlpriv, REG_RCR, ((u32 *)(val))[0]);
		rtlpci->receive_config = ((u32 *)(val))[0];
		}
		break;
	case HW_VAR_RETRY_LIMIT:{
		u8 retry_limit = ((u8 *)(val))[0];

		rtl_write_word(rtlpriv, REG_RETRY_LIMIT,
			       retry_limit << RETRY_LIMIT_SHORT_SHIFT |
			       retry_limit << RETRY_LIMIT_LONG_SHIFT);
		}
		break;
	case HW_VAR_DUAL_TSF_RST:
		rtl_write_byte(rtlpriv, REG_DUAL_TSF_RST, (BIT(0) | BIT(1)));
		break;
	case HW_VAR_EFUSE_BYTES:
		efuse->efuse_usedbytes = *((u16 *)val);
		break;
	case HW_VAR_EFUSE_USAGE:
		efuse->efuse_usedpercentage = *((u8 *)val);
		break;
	case HW_VAR_IO_CMD:
		rtl92fe_phy_set_io_cmd(hw, (*(enum io_type *)val));
		break;
	case HW_VAR_SET_RPWM:{
		u8 rpwm_val;

		rpwm_val = rtl_read_byte(rtlpriv, REG_PCIE_HRPWM);
		udelay(1);

		if (rpwm_val & BIT(7)) {
			rtl_write_byte(rtlpriv, REG_PCIE_HRPWM, (*(u8 *)val));
		} else {
			rtl_write_byte(rtlpriv, REG_PCIE_HRPWM,
				       ((*(u8 *)val) | BIT(7)));
		}
		}
		break;
	case HW_VAR_H2C_FW_PWRMODE:
		rtl92fe_set_fw_pwrmode_cmd(hw, (*(u8 *)val));
		break;
	case HW_VAR_FW_PSMODE_STATUS:
		ppsc->fw_current_inpsmode = *((bool *)val);
		break;
	case HW_VAR_RESUME_CLK_ON:
		_rtl92fe_set_fw_ps_rf_on(hw);
		break;
	case HW_VAR_FW_LPS_ACTION:{
		bool b_enter_fwlps = *((bool *)val);

		if (b_enter_fwlps)
			_rtl92fe_fwlps_enter(hw);
		else
			_rtl92fe_fwlps_leave(hw);
		}
		break;
	case HW_VAR_H2C_FW_JOINBSSRPT:{
		u8 mstatus = (*(u8 *)val);

		if (mstatus == RT_MEDIA_CONNECT) {
			rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_AID, NULL);
			_rtl92fe_download_rsvd_page(hw);
		}
		rtl92fe_set_fw_media_status_rpt_cmd(hw, mstatus, 0);
		}
		break;
	case HW_VAR_H2C_FW_P2P_PS_OFFLOAD:
		rtl92fe_set_p2p_ps_offload_cmd(hw, (*(u8 *)val));
		break;
	case HW_VAR_AID:{
		u16 u2btmp;

		u2btmp = rtl_read_word(rtlpriv, REG_BCN_PSR_RPT);
		u2btmp &= 0xC000;
		rtl_write_word(rtlpriv, REG_BCN_PSR_RPT,
			       (u2btmp | mac->assoc_id));
		}
		break;
	case HW_VAR_CORRECT_TSF:{
		u8 btype_ibss = ((u8 *)(val))[0];

		if (btype_ibss)
			_rtl92fe_stop_tx_beacon(hw);

		_rtl92fe_set_bcn_ctrl_reg(hw, 0, BIT(3));

		rtl_write_dword(rtlpriv, REG_TSFTR,
				(u32)(mac->tsf & 0xffffffff));
		rtl_write_dword(rtlpriv, REG_TSFTR + 4,
				(u32)((mac->tsf >> 32) & 0xffffffff));

		_rtl92fe_set_bcn_ctrl_reg(hw, BIT(3), 0);

		if (btype_ibss)
			_rtl92fe_resume_tx_beacon(hw);
		}
		break;
	case HW_VAR_KEEP_ALIVE: {
		u8 array[2];

		array[0] = 0xff;
		array[1] = *((u8 *)val);
		rtl92fe_fill_h2c_cmd(hw, H2C_92F_KEEP_ALIVE_CTRL, 2, array);
		}
		break;
	default:
		rtl_dbg(rtlpriv, COMP_ERR, DBG_DMESG,
			"switch case %#x not processed\n", variable);
		break;
	}
}

static bool _rtl92fe_llt_table_init(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 txpktbuf_bndy;
	u8 u8tmp, testcnt = 0;

	/* The RTL8192F packet buffer holds 0xF7 TX pages; the boundary at
	 * 0xF8 reserves the top of the buffer for the beacon/MGNT queue.
	 * The 4-EP RQPN split allocates HIQ/LOQ/NORMQ = 8 pages each and
	 * the remainder (0xDE) to the public queue, with the auto-LLT bit
	 * driving the on-chip linked-list initialisation.
	 */
	txpktbuf_bndy = TX_PAGE_BOUNDARY;

	rtl_write_dword(rtlpriv, REG_RQPN, RQPN_INIT_VALUE);

	rtl_write_byte(rtlpriv, REG_TRXFF_BNDY, txpktbuf_bndy);
	rtl_write_word(rtlpriv, REG_TRXFF_BNDY + 2, 0x3f00 - 1);

	rtl_write_byte(rtlpriv, REG_DWBCN0_CTRL + 1, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_DWBCN1_CTRL + 1, txpktbuf_bndy);

	rtl_write_byte(rtlpriv, REG_BCNQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_BCNQ1_BDNY, txpktbuf_bndy);

	rtl_write_byte(rtlpriv, REG_MGQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_WMAC_LBK_BF_HD, txpktbuf_bndy);

	/* 256-byte page boundary, 8 bytes of RX driver-info. */
	rtl_write_byte(rtlpriv, REG_PBP, 0x31);
	rtl_write_byte(rtlpriv, REG_RX_DRVINFO_SZ, 0x4);

	u8tmp = rtl_read_byte(rtlpriv, REG_AUTO_LLT + 2);
	rtl_write_byte(rtlpriv, REG_AUTO_LLT + 2, u8tmp | BIT(0));

	while (u8tmp & BIT(0)) {
		u8tmp = rtl_read_byte(rtlpriv, REG_AUTO_LLT + 2);
		udelay(10);
		testcnt++;
		if (testcnt > 10)
			break;
	}

	return true;
}

static void _rtl92fe_gen_refresh_led_state(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	enum rtl_led_pin pin0 = rtlpriv->ledctl.sw_led0;

	if (rtlpriv->rtlhal.up_first_time)
		return;

	if (ppsc->rfoff_reason == RF_CHANGE_BY_IPS)
		rtl92fe_sw_led_on(hw, pin0);
	else if (ppsc->rfoff_reason == RF_CHANGE_BY_INIT)
		rtl92fe_sw_led_on(hw, pin0);
	else
		rtl92fe_sw_led_off(hw, pin0);
}

static bool _rtl92fe_init_mac(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));

	u8 bytetmp;
	u32 dwordtmp;

	rtl_write_byte(rtlpriv, REG_RSV_CTRL, 0x0);

	dwordtmp = rtl_read_dword(rtlpriv, REG_SYS_CFG1);
	if (dwordtmp & BIT(24)) {
		rtl_write_byte(rtlpriv, REG_LDO_SW_CTRL, 0xc3);
	} else {
		bytetmp = rtl_read_byte(rtlpriv, REG_AFE_PLL_CTRL + 2);
		rtl_write_byte(rtlpriv, REG_AFE_PLL_CTRL + 2,
			       bytetmp | BIT(4) | BIT(6));
		rtl_write_byte(rtlpriv, REG_LDO_SW_CTRL, 0x83);
	}

	/* 1. 40 MHz crystal source */
	bytetmp = rtl_read_byte(rtlpriv, REG_AFE_XTAL_CTRL);
	bytetmp &= 0xfb;
	rtl_write_byte(rtlpriv, REG_AFE_XTAL_CTRL, bytetmp);

	dwordtmp = rtl_read_dword(rtlpriv, REG_AFE_PLL_CTRL);
	dwordtmp &= 0xfffffc7f;
	rtl_write_dword(rtlpriv, REG_AFE_PLL_CTRL, dwordtmp);

	/* 2. AFE parameter trim */
	bytetmp = rtl_read_byte(rtlpriv, REG_AFE_XTAL_CTRL);
	bytetmp &= 0xbf;
	rtl_write_byte(rtlpriv, REG_AFE_XTAL_CTRL, bytetmp);

	dwordtmp = rtl_read_dword(rtlpriv, REG_AFE_PLL_CTRL);
	dwordtmp &= 0xffdfffff;
	rtl_write_dword(rtlpriv, REG_AFE_PLL_CTRL, dwordtmp);

	/* HW power-on sequence (parsed pwrseq command list). */
	if (!rtl_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK,
				      PWR_INTF_PCI_MSK,
				      RTL8192F_NIC_ENABLE_FLOW)) {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
			"init MAC Fail as rtl_hal_pwrseqcmdparsing\n");
		return false;
	}

	/* Release MAC IO register reset */
	bytetmp = rtl_read_byte(rtlpriv, REG_CR);
	bytetmp = 0xff;
	rtl_write_byte(rtlpriv, REG_CR, bytetmp);
	mdelay(2);
	bytetmp = 0x7f;
	rtl_write_byte(rtlpriv, REG_HWSEQ_CTRL, bytetmp);
	mdelay(2);

	/* Add for wakeup online */
	bytetmp = rtl_read_byte(rtlpriv, REG_SYS_CLKR);
	rtl_write_byte(rtlpriv, REG_SYS_CLKR, bytetmp | BIT(3));
	bytetmp = rtl_read_byte(rtlpriv, REG_GPIO_MUXCFG + 1);
	rtl_write_byte(rtlpriv, REG_GPIO_MUXCFG + 1, bytetmp & (~BIT(4)));
	/* Release MAC IO register reset */
	rtl_write_word(rtlpriv, REG_CR, 0x2ff);

	if (!rtlhal->mac_func_enable) {
		if (!_rtl92fe_llt_table_init(hw)) {
			rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
				"LLT table init fail\n");
			return false;
		}
	}

	rtl_write_dword(rtlpriv, REG_HISR, 0xffffffff);
	rtl_write_dword(rtlpriv, REG_HISRE, 0xffffffff);

	/* TRXDMA_CTRL (REG_TXDMA_PQ_MAP) is a 16-bit, 2-bit-field-per-queue
	 * priority map on the RTL8192F (vendor HAL == 8192ee): VOQ=4, VIQ=6,
	 * BEQ=8, BKQ=10, MGQ=12, HIQ=14, with LOW=1/NORMAL=2/HIGH=3. Program all
	 * queues while preserving the RX-DMA nibble in bits[3:0]. MUST be a WORD
	 * access -- a 32-bit write spills the high half into 0x010E.
	 */
	dwordtmp = rtl_read_word(rtlpriv, REG_TRXDMA_CTRL);
	dwordtmp &= 0xf;
	dwordtmp |= TRXDMA_CTRL_QMAP_VALUE;
	rtl_write_word(rtlpriv, REG_TRXDMA_CTRL, dwordtmp);
	/* Reported Tx status from HW for rate adaptive. */
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL + 1, 0x1F);

	/* Set RCR register */
	rtl_write_dword(rtlpriv, REG_RCR, rtlpci->receive_config);
	/* Per-frame-type RX subtype filter maps. Only RXFLTMAP2 (management) was
	 * programmed before; RXFLTMAP0 (data) + RXFLTMAP1 (control) were left at the
	 * chip reset default, which drops the client's DATA frames. ★2026-07-05: this
	 * is the WiFi ASSOCIATION bug: the WPA 4-way-handshake EAPOL messages (M2/M4)
	 * are DATA frames — with RXFLTMAP0 unset the AP admits the auth/assoc (mgmt via
	 * RXFLTMAP2) but never receives the client's EAPOL replies, so the 4-way handshake
	 * times out and hostapd deauthenticates the STA ("local deauth request"). Program
	 * the full triple to the values this driver's DESIGN.md specifies: data=accept-all,
	 * control=PS-Poll only (so AP power-save clients work), management=accept-all. */
	rtl_write_word(rtlpriv, REG_RXFLTMAP0, 0xffff);	/* data: accept all subtypes (incl. EAPOL) */
	rtl_write_word(rtlpriv, REG_RXFLTMAP1, 0x0400);	/* control: admit PS-Poll (subtype 10) */
	rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xffff);	/* management: accept all subtypes */

	/* Set TCR register */
	rtl_write_dword(rtlpriv, REG_TCR, rtlpci->transmit_config);

	/* Set TX/RX descriptor physical address -- HI part */
	if (!rtlpriv->cfg->mod_params->dma64)
		goto dma64_end;

	rtl_write_dword(rtlpriv, REG_BCNQ_DESA + 4,
			((u64)rtlpci->tx_ring[BEACON_QUEUE].buffer_desc_dma) >>
				32);
	rtl_write_dword(rtlpriv, REG_MGQ_DESA + 4,
			(u64)rtlpci->tx_ring[MGNT_QUEUE].buffer_desc_dma >> 32);
	rtl_write_dword(rtlpriv, REG_VOQ_DESA + 4,
			(u64)rtlpci->tx_ring[VO_QUEUE].buffer_desc_dma >> 32);
	rtl_write_dword(rtlpriv, REG_VIQ_DESA + 4,
			(u64)rtlpci->tx_ring[VI_QUEUE].buffer_desc_dma >> 32);
	rtl_write_dword(rtlpriv, REG_BEQ_DESA + 4,
			(u64)rtlpci->tx_ring[BE_QUEUE].buffer_desc_dma >> 32);
	rtl_write_dword(rtlpriv, REG_BKQ_DESA + 4,
			(u64)rtlpci->tx_ring[BK_QUEUE].buffer_desc_dma >> 32);
	rtl_write_dword(rtlpriv, REG_HQ0_DESA + 4,
			(u64)rtlpci->tx_ring[HIGH_QUEUE].buffer_desc_dma >> 32);

	rtl_write_dword(rtlpriv, REG_RX_DESA + 4,
			(u64)rtlpci->rx_ring[RX_MPDU_QUEUE].dma >> 32);

dma64_end:

	/* Set TX/RX descriptor physical address (lo part). The ring-base
	 * DMA value split is host-endianness-independent (BE-MIPS safe):
	 * rtl_write_dword normalises the register write, and the DMA
	 * address is masked to 32 bits before being handed to the chip.
	 */
	rtl_write_dword(rtlpriv, REG_BCNQ_DESA,
			((u64)rtlpci->tx_ring[BEACON_QUEUE].buffer_desc_dma) &
			DMA_BIT_MASK(32));
	rtl_write_dword(rtlpriv, REG_MGQ_DESA,
			(u64)rtlpci->tx_ring[MGNT_QUEUE].buffer_desc_dma &
			DMA_BIT_MASK(32));
	rtl_write_dword(rtlpriv, REG_VOQ_DESA,
			(u64)rtlpci->tx_ring[VO_QUEUE].buffer_desc_dma &
			DMA_BIT_MASK(32));
	rtl_write_dword(rtlpriv, REG_VIQ_DESA,
			(u64)rtlpci->tx_ring[VI_QUEUE].buffer_desc_dma &
			DMA_BIT_MASK(32));

	rtl_write_dword(rtlpriv, REG_BEQ_DESA,
			(u64)rtlpci->tx_ring[BE_QUEUE].buffer_desc_dma &
			DMA_BIT_MASK(32));

	dwordtmp = rtl_read_dword(rtlpriv, REG_BEQ_DESA);

	rtl_write_dword(rtlpriv, REG_BKQ_DESA,
			(u64)rtlpci->tx_ring[BK_QUEUE].buffer_desc_dma &
			DMA_BIT_MASK(32));
	rtl_write_dword(rtlpriv, REG_HQ0_DESA,
			(u64)rtlpci->tx_ring[HIGH_QUEUE].buffer_desc_dma &
			DMA_BIT_MASK(32));

	rtl_write_dword(rtlpriv, REG_RX_DESA,
			(u64)rtlpci->rx_ring[RX_MPDU_QUEUE].dma &
			DMA_BIT_MASK(32));

	rtl_write_dword(rtlpriv, REG_TSFTIMER_HCI, 0x3fffffff);

	bytetmp = rtl_read_byte(rtlpriv, REG_PCIE_CTRL_REG + 3);
	rtl_write_byte(rtlpriv, REG_PCIE_CTRL_REG + 3, bytetmp | 0xF7);

	rtl_write_dword(rtlpriv, REG_INT_MIG, 0);

	rtl_write_dword(rtlpriv, REG_MCUTST_1, 0x0);

	/* Ring SW/HW-depth invariants (build-time, chip-agnostic).
	 *
	 * The depth we program into the HW BD-ring registers below MUST equal
	 * the SW ring depth the shared rtlwifi PCI core actually allocates for
	 * this chip; if they diverge, the HW read pointer wraps at a different
	 * modulus than the SW write pointer (cur_tx_wp), TX-reclaim's
	 * is_tx_desc_closed() latches false, and every AC stop-queues forever
	 * (the 512-vs-128 association/DHCP/hang regression).
	 *
	 * TX: PCI id 0x818c is unknown to _rtl_pci_find_adapter(), so hw_type
	 * falls back to the non-8192EE default and _rtl_pci_init_trx_var()
	 * sizes every TX ring to RT_TXDESC_NUM -- NOT TX_DESC_NUM_92E. So the
	 * per-queue HW TXBD depth (TX_DESC_NUM_92F) must equal RT_TXDESC_NUM.
	 * (A sibling chip that _is_ mapped to HARDWARE_TYPE_RTL8192EE would
	 * instead assert its own TX_DESC_NUM_xx == TX_DESC_NUM_92E: same rule,
	 * the SW sizer for that chip's resolved hw_type.)
	 * RX: rxringcount is always RTL_PCI_MAX_RX_COUNT, independent of
	 * hw_type, so the HW RXBD depth (RX_DESC_NUM_92F) must equal it.
	 *
	 * Derived from the driver's own constants (never a hand-copied literal)
	 * so each family member is checked against ITS own ring size at build.
	 * Inert while correct (128==128 / 512==512); fires only on a real drift
	 * -- a copy-paste to a new model that keeps a stale depth, or a core
	 * change to the SW sizer.
	 */
	BUILD_BUG_ON(TX_DESC_NUM_92F != RT_TXDESC_NUM);
	BUILD_BUG_ON(RX_DESC_NUM_92F != RTL_PCI_MAX_RX_COUNT);
	/* ...and each depth must fit its register's depth field without
	 * corrupting the adjacent segment-count/enable bits: TXBD_NUM packs
	 * depth in [11:0] | seg in [13:12]; RXBD_NUM packs depth in [12:0] |
	 * seg in [14:13] | enable BIT(15).  (Both true today; guards a future
	 * model that bumps a ring past the field width.)
	 */
	BUILD_BUG_ON(TX_DESC_NUM_92F & ~0xFFFU);
	BUILD_BUG_ON(RX_DESC_NUM_92F & ~0x1FFFU);

	/* Program the per-queue TXBD ring depth + segment count. */
	rtl_write_word(rtlpriv, REG_MGQ_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_VOQ_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_VIQ_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_BEQ_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_BKQ_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI0Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI1Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI2Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI3Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI4Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI5Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI6Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	rtl_write_word(rtlpriv, REG_HI7Q_TXBD_NUM,
		       TX_DESC_NUM_92F | ((RTL8192FE_SEG_NUM << 12) & 0x3000));
	/* RX ring depth + 0x8000 = enable RX DMA. */
	rtl_write_word(rtlpriv, REG_RX_RXBD_NUM,
		       RX_DESC_NUM_92F |
		       ((RTL8192FE_SEG_NUM << 13) & 0x6000) | 0x8000);

	rtl_write_dword(rtlpriv, REG_TSFTIMER_HCI, 0xFFFFFFFF);

	_rtl92fe_gen_refresh_led_state(hw);
	return true;
}

static void _rtl92fe_hw_configure(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	u32 reg_rrsr;

	reg_rrsr = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
	/* Init value for RRSR. */
	rtl_write_dword(rtlpriv, REG_RRSR, reg_rrsr);

	/* ARFB table 8 for 11n 2SS */
	rtl_write_dword(rtlpriv, REG_ARFR0, 0x00000010);
	rtl_write_dword(rtlpriv, REG_ARFR0 + 4, 0x3e0ff000);

	/* ARFB table 9 for 11n 1SS */
	rtl_write_dword(rtlpriv, REG_ARFR1, 0x00000010);
	rtl_write_dword(rtlpriv, REG_ARFR1 + 4, 0x000ff000);

	/* Set SLOT time */
	rtl_write_byte(rtlpriv, REG_SLOT, 0x09);

	/* CF-End setting. */
	rtl_write_word(rtlpriv, REG_FWHW_TXQ_CTRL, 0x1F80);

	/* Set retry limit */
	rtl_write_word(rtlpriv, REG_RETRY_LIMIT, 0x0707);

	/* BAR settings */
	rtl_write_dword(rtlpriv, REG_BAR_MODE_CTRL, 0x0201ffff);

	/* Set Data / Response auto rate fallback retry count */
	rtl_write_dword(rtlpriv, REG_DARFRC, 0x01000000);
	rtl_write_dword(rtlpriv, REG_DARFRC + 4, 0x07060504);
	rtl_write_dword(rtlpriv, REG_RARFRC, 0x01000000);
	rtl_write_dword(rtlpriv, REG_RARFRC + 4, 0x07060504);

	/* Beacon related, for rate adaptive */
	rtl_write_byte(rtlpriv, REG_ATIMWND, 0x2);
	rtl_write_byte(rtlpriv, REG_BCN_MAX_ERR, 0xff);

	rtlpci->reg_bcn_ctrl_val = 0x1d;
	rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtlpci->reg_bcn_ctrl_val);

	/* Second-beacon (multi-BSSID) control register; disabled for the
	 * single-BSSID case.
	 */
	rtl_write_byte(rtlpriv, REG_BCN_CTRL_1, 0);

	/* TBTT prohibit hold time. */
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT + 1, 0xff); /* 8 ms */

	rtl_write_byte(rtlpriv, REG_PIFS, 0);
	rtl_write_byte(rtlpriv, REG_AGGR_BREAK_TIME, 0x16);

	rtl_write_word(rtlpriv, REG_NAV_PROT_LEN, 0x0040);
	rtl_write_word(rtlpriv, REG_PROT_MODE_CTRL, 0x08ff);

	/* For Rx TP. */
	rtl_write_dword(rtlpriv, REG_FAST_EDCA_CTRL, 0x03086666);

	/* ACKTO for IOT issue. */
	rtl_write_byte(rtlpriv, REG_ACKTO, 0x40);

	/* Set Spec SIFS (used in NAV) */
	rtl_write_word(rtlpriv, REG_SPEC_SIFS, 0x100a);
	rtl_write_word(rtlpriv, REG_MAC_SPEC_SIFS, 0x100a);

	/* Set SIFS for CCK */
	rtl_write_word(rtlpriv, REG_SIFS_CTX, 0x100a);

	/* Set SIFS for OFDM */
	rtl_write_word(rtlpriv, REG_SIFS_TRX, 0x100a);

	rtl_write_byte(rtlpriv, REG_RX_PKT_LIMIT, 0x20);

	rtl_write_word(rtlpriv, REG_MAX_AGGR_NUM, 0x1f1f);

	/* Set Multicast Address. */
	rtl_write_dword(rtlpriv, REG_MAR, 0xffffffff);
	rtl_write_dword(rtlpriv, REG_MAR + 4, 0xffffffff);
}

static void _rtl92fe_enable_aspm_back_door(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	u32 tmp32 = 0, count = 0;
	u8 tmp8 = 0;

	rtl_write_word(rtlpriv, REG_BACKDOOR_DBI_DATA, 0x78);
	rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2, 0x2);
	tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
	count = 0;
	while (tmp8 && count < 20) {
		udelay(10);
		tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
		count++;
	}

	if (tmp8 == 0) {
		tmp32 = rtl_read_dword(rtlpriv, REG_BACKDOOR_DBI_RDATA);
		if ((tmp32 & 0xff00) != 0x2000) {
			tmp32 &= 0xffff00ff;
			rtl_write_dword(rtlpriv, REG_BACKDOOR_DBI_WDATA,
					tmp32 | BIT(13));
			rtl_write_word(rtlpriv, REG_BACKDOOR_DBI_DATA, 0xf078);
			rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2, 0x1);

			tmp8 = rtl_read_byte(rtlpriv,
					     REG_BACKDOOR_DBI_DATA + 2);
			count = 0;
			while (tmp8 && count < 20) {
				udelay(10);
				tmp8 = rtl_read_byte(rtlpriv,
						     REG_BACKDOOR_DBI_DATA + 2);
				count++;
			}
		}
	}

	rtl_write_word(rtlpriv, REG_BACKDOOR_DBI_DATA, 0x70c);
	rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2, 0x2);
	tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
	count = 0;
	while (tmp8 && count < 20) {
		udelay(10);
		tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
		count++;
	}
	if (tmp8 == 0) {
		tmp32 = rtl_read_dword(rtlpriv, REG_BACKDOOR_DBI_RDATA);
		rtl_write_dword(rtlpriv, REG_BACKDOOR_DBI_WDATA,
				tmp32 | BIT(31));
		rtl_write_word(rtlpriv, REG_BACKDOOR_DBI_DATA, 0xf70c);
		rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2, 0x1);
	}

	tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
	count = 0;
	while (tmp8 && count < 20) {
		udelay(10);
		tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
		count++;
	}

	rtl_write_word(rtlpriv, REG_BACKDOOR_DBI_DATA, 0x718);
	rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2, 0x2);
	tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
	count = 0;
	while (tmp8 && count < 20) {
		udelay(10);
		tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
		count++;
	}
	if (ppsc->support_backdoor || (tmp8 == 0)) {
		tmp32 = rtl_read_dword(rtlpriv, REG_BACKDOOR_DBI_RDATA);
		rtl_write_dword(rtlpriv, REG_BACKDOOR_DBI_WDATA,
				tmp32 | BIT(11) | BIT(12));
		rtl_write_word(rtlpriv, REG_BACKDOOR_DBI_DATA, 0xf718);
		rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2, 0x1);
	}
	tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
	count = 0;
	while (tmp8 && count < 20) {
		udelay(10);
		tmp8 = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 2);
		count++;
	}
}

void rtl92fe_enable_hw_security_config(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 sec_reg_value;
	u8 tmp;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_DMESG,
		"PairwiseEncAlgorithm = %d GroupEncAlgorithm = %d\n",
		rtlpriv->sec.pairwise_enc_algorithm,
		rtlpriv->sec.group_enc_algorithm);

	if (rtlpriv->cfg->mod_params->sw_crypto || rtlpriv->sec.use_sw_sec) {
		rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG,
			"not open hw encryption\n");
		return;
	}

	sec_reg_value = SCR_TXENCENABLE | SCR_RXDECENABLE;

	if (rtlpriv->sec.use_defaultkey) {
		sec_reg_value |= SCR_TXUSEDK;
		sec_reg_value |= SCR_RXUSEDK;
	}

	sec_reg_value |= (SCR_RXBCUSEDK | SCR_TXBCUSEDK);

	tmp = rtl_read_byte(rtlpriv, REG_CR + 1);
	rtl_write_byte(rtlpriv, REG_CR + 1, tmp | BIT(1));

	rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG,
		"The SECR-value %x\n", sec_reg_value);

	rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_WPA_CONFIG, &sec_reg_value);
}

static bool _rtl92fe_check_pcie_dma_hang(struct rtl_priv *rtlpriv)
{
	u8 tmp;

	/* Enable the PCIe debug port (reg 0x350 bit[26]). */
	tmp = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 3);
	if (!(tmp & BIT(2))) {
		rtl_write_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 3,
			       tmp | BIT(2));
		mdelay(100);
	}

	/* reg 0x350 bit[25] = RX hang, bit[24] = TX hang. */
	tmp = rtl_read_byte(rtlpriv, REG_BACKDOOR_DBI_DATA + 3);
	if ((tmp & BIT(0)) || (tmp & BIT(1))) {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
			"CheckPcieDMAHang8192FE(): true!!\n");
		return true;
	}
	return false;
}

static void _rtl92fe_reset_pcie_interface_dma(struct rtl_priv *rtlpriv,
					      bool mac_power_on)
{
	u8 tmp;
	bool release_mac_rx_pause;
	u8 backup_pcie_dma_pause;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
		"ResetPcieInterfaceDMA8192FE()\n");

	/* PCIe RX-DMA-hang reset flow. */

	/* 1. disable register write lock
	 *	write 0x1C bit[1:0] = 2'h0
	 *	write 0xCC bit[2] = 1'b1
	 */
	tmp = rtl_read_byte(rtlpriv, REG_RSV_CTRL);
	tmp &= ~(BIT(1) | BIT(0));
	rtl_write_byte(rtlpriv, REG_RSV_CTRL, tmp);
	tmp = rtl_read_byte(rtlpriv, REG_PMC_DBG_CTRL2);
	tmp |= BIT(2);
	rtl_write_byte(rtlpriv, REG_PMC_DBG_CTRL2, tmp);

	/* 2. Check and pause TRX DMA
	 *	write 0x284 bit[18] = 1'b1
	 *	write 0x301 = 0xFF
	 */
	tmp = rtl_read_byte(rtlpriv, REG_RXDMA_CONTROL);
	if (tmp & BIT(2)) {
		/* Already paused for another reason. */
		release_mac_rx_pause = false;
	} else {
		rtl_write_byte(rtlpriv, REG_RXDMA_CONTROL, (tmp | BIT(2)));
		release_mac_rx_pause = true;
	}

	backup_pcie_dma_pause = rtl_read_byte(rtlpriv, REG_PCIE_CTRL_REG + 1);
	if (backup_pcie_dma_pause != 0xFF)
		rtl_write_byte(rtlpriv, REG_PCIE_CTRL_REG + 1, 0xFF);

	if (mac_power_on) {
		/* 3. reset TRX function: write 0x100 = 0x00 */
		rtl_write_byte(rtlpriv, REG_CR, 0);
	}

	/* 4. Reset PCIe DMA: write 0x003 bit[0] = 0 */
	tmp = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN + 1);
	tmp &= ~(BIT(0));
	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN + 1, tmp);

	/* 5. Enable PCIe DMA: write 0x003 bit[0] = 1 */
	tmp = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN + 1);
	tmp |= BIT(0);
	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN + 1, tmp);

	if (mac_power_on) {
		/* 6. enable TRX function: write 0x100 = 0xFF.
		 * LLT/RQPN and ring base addresses are re-initialised later
		 * because the MAC function was reset.
		 */
		rtl_write_byte(rtlpriv, REG_CR, 0xFF);
	}

	/* 7. Restore PCIe autoload-down bit: write 0xF8 bit[17] = 1'b1 */
	tmp = rtl_read_byte(rtlpriv, REG_MAC_PHY_CTRL_NORMAL + 2);
	tmp |= BIT(1);
	rtl_write_byte(rtlpriv, REG_MAC_PHY_CTRL_NORMAL + 2, tmp);

	/* In MAC-power-on state BB/RF may be ON; releasing TRX DMA here
	 * would start traffic, so defer it until after re-init.
	 */
	if (!mac_power_on) {
		/* 8. release TRX DMA */
		if (release_mac_rx_pause) {
			tmp = rtl_read_byte(rtlpriv, REG_RXDMA_CONTROL);
			rtl_write_byte(rtlpriv, REG_RXDMA_CONTROL,
				       (tmp & (~BIT(2))));
		}
		rtl_write_byte(rtlpriv, REG_PCIE_CTRL_REG + 1,
			       backup_pcie_dma_pause);
	}

	/* 9. lock system register: write 0xCC bit[2] = 1'b0 */
	tmp = rtl_read_byte(rtlpriv, REG_PMC_DBG_CTRL2);
	tmp &= ~(BIT(2));
	rtl_write_byte(rtlpriv, REG_PMC_DBG_CTRL2, tmp);
}

/* Fixed-argument TX/RX-path ("TRX mode") init for the configuration this
 * board runs:
 *   tx_path_en      = BB_PATH_AB  (both TX paths)
 *   rx_path         = BB_PATH_AB  (both RX paths)
 *   tx_path_sel_1ss = BB_PATH_A   (nominal; with path-diversity off the 1ss
 *                                  selection collapses to tx_path_en = AB)
 *   tx_path_sel_cck = BB_PATH_A   (likewise collapses to AB)
 *   rfe_type        = 3           (no handled case -> rfe sub-step is a no-op)
 *
 * Without this TRX-mode init after BB config the path registers
 * (0x804/0x808-equiv c04/0x90c/...) are left mis-configured. Path-diversity,
 * spur calibration, dynamic energy-TH and MP-mode branches are intentionally
 * omitted (not used for a fixed-mode AP).
 */
/* RFE (RF front-end) pinmux for the external-PA/FEM board variant.
 *
 * The WiFi efuse on this board is blank, so rfe_type can't be auto-read. GPON
 * ONUs use an external PA/FEM (SKY85201-class = the vendor's DSL-PON rfe_type 7).
 * Without this the T/R-switch + PA-enable + RX-LNA control pins stay at their BB
 * table defaults, so TX radiates ~40dB down AND the RX is deaf (a close peer is
 * heard at ~-100dBm). These are the vendor phydm 8192F rfe_type-7 RFE-pinmux
 * writes (0x940 control word, 0x930/0x938 T/R-switch invert/sel, 0x944, 0x934/
 * 0x93c, 0x92c/0x920 PAPE, 0x968, plus the 0x103c/0x04c/0x064/0x1038 seeds). */
static void _rtl92fe_config_rfe(struct ieee80211_hw *hw)
{
	rtl_set_bbreg(hw, 0x103c, 0x70000, 0x7);
	rtl_set_bbreg(hw, 0x04c, 0x6c00000, 0x0);
	rtl_set_bbreg(hw, 0x064, BIT(29) | BIT(28), 0x3);
	rtl_set_bbreg(hw, 0x1038, 0x600010, 0x0);
	rtl_set_bbreg(hw, 0x944, 0xfff, 0x081f);
	rtl_set_bbreg(hw, 0x930, 0xfffff, 0x23200);
	rtl_set_bbreg(hw, 0x938, 0xfffff, 0x23200);
	rtl_set_bbreg(hw, 0x934, 0xf000, 0x3);
	rtl_set_bbreg(hw, 0x93c, 0xf000, 0x3);
	rtl_set_bbreg(hw, 0x968, BIT(2), 0x0);
	rtl_set_bbreg(hw, 0x920, 0xffffffff, 0x03000003);
	rtl_set_bbreg(hw, 0x940, 0xffffffff, 0x004007ae);
}

static void _rtl92fe_config_trx_mode_ab(struct ieee80211_hw *hw)
{
	/* ==== [RF Mode Table] (tx_path_en == BB_PATH_AB) ==== */
	rtl_set_bbreg(hw, 0x824, 0xe, 2);
	rtl_set_bbreg(hw, 0x82c, 0xe, 2);

	rtl_set_bbreg(hw, 0x804, 0xf, 0x3);
	/* CCK TX path control by REG */
	rtl_set_bbreg(hw, 0x80c, BIT(31), 0x0);

	/* ==== [RX Path] configure RX paths for BB_PATH_AB ==== */
	/* OFDM Rx path (val = 3 for AB) */
	rtl_set_bbreg(hw, 0xc04, 0xff, 0x33);
	rtl_set_bbreg(hw, 0xd04, 0xf, 3);
	/* CCK Rx path: generic 2R block (num_enable_path > 1) */
	rtl_set_bbreg(hw, 0xa04, (BIT(27) | BIT(26)), 0);
	rtl_set_bbreg(hw, 0xa04, (BIT(25) | BIT(24)), 1);
	rtl_set_bbreg(hw, 0xa74, BIT(8), 1);
	rtl_set_bbreg(hw, 0xa2c, BIT(18), 1);
	rtl_set_bbreg(hw, 0xa2c, BIT(22), 1);
	/* CCK Rx path: 8192F-specific AB override */
	rtl_set_bbreg(hw, 0xa04, (BIT(27) | BIT(26)), 0);
	rtl_set_bbreg(hw, 0xa04, (BIT(25) | BIT(24)), 1);
	rtl_set_bbreg(hw, 0xa74, BIT(8), 1);
	rtl_set_bbreg(hw, 0xa2c, (BIT(18) | BIT(17)), 1);
	rtl_set_bbreg(hw, 0xa2c, (BIT(22) | BIT(21)), 1);

	/* ==== [TX Path] configure TX paths for AB, AB, AB ==== */
	/* CCK TX antenna mapping (BB_PATH_AB) */
	rtl_set_bbreg(hw, 0xa04, 0xf0000000, 0xc);
	/* OFDM TX path (tx_path_en == AB -> ofdm_tx_path(BB_PATH_AB)) */
	rtl_set_bbreg(hw, 0x90c, 0xffffffff, 0x83321333);

	/* External-PA/FEM front-end pinmux (T/R switch + PA-enable + RX-LNA). */
	_rtl92fe_config_rfe(hw);
}

int rtl92fe_hw_init(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	struct rtl_phy *rtlphy = &rtlpriv->phy;
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	bool rtstatus = true;
	int err = 0;
	u8 tmp_u1b, u1byte;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, " Rtl8192FE hw init\n");
	rtlpriv->rtlhal.being_init_adapter = true;
	rtlpriv->intf_ops->disable_aspm(hw);

	/* Decide whether the MAC sub-system is already enabled (warm boot). */
	tmp_u1b = rtl_read_byte(rtlpriv, REG_SYS_CLKR + 1);
	u1byte = rtl_read_byte(rtlpriv, REG_CR);
	if ((tmp_u1b & BIT(3)) && (u1byte != 0 && u1byte != 0xEA)) {
		rtlhal->mac_func_enable = true;
	} else {
		rtlhal->mac_func_enable = false;
		rtlhal->fw_ps_state = FW_PS_STATE_ALL_ON_92F;
	}

	if (_rtl92fe_check_pcie_dma_hang(rtlpriv)) {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_DMESG, "92fe dma hang!\n");
		_rtl92fe_reset_pcie_interface_dma(rtlpriv,
						  rtlhal->mac_func_enable);
		rtlhal->mac_func_enable = false;
	}

	/* RTL8192F crystal-frequency select: 25 MHz crystal -> REG_AFE_CTRL5
	 * (0x0094) BIT_REF_SEL [28:25] = 1 (0 selects 40 MHz). Set before pwron. */
	rtl_write_dword(rtlpriv, 0x0094,
			(rtl_read_dword(rtlpriv, 0x0094) & ~(0xfu << 25)) |
			(1u << 25));

	/* Init order: pwron -> MAC init/LLT/queue-page alloc. */
	rtstatus = _rtl92fe_init_mac(hw);
	{
		struct rtl_pci *rp = rtl_pcidev(rtl_pcipriv(hw));
		u16 pw;
		int round = 0;

		pcie_capability_clear_word(rp->pdev, PCI_EXP_LNKCTL,
					   PCI_EXP_LNKCTL_ASPMC);
		if (rp->pdev->bus->self)
			pcie_capability_clear_word(rp->pdev->bus->self,
						   PCI_EXP_LNKCTL,
						   PCI_EXP_LNKCTL_ASPMC);

		/* Power-on tail the pwrseq-only init_mac does not cover (stock
		 * brings it up in this order): AFE power-on, SYS_PW_CTRL(0x04)
		 * power-ready handshake, WLAN auto-enable, hold the 8051, then
		 * LDO/SPS regulator(0x7c). Without the regulator + power-ready
		 * the high-offset RAM region stays marginally powered. */
		rtl_write_byte(rtlpriv, 0x24, rtl_read_byte(rtlpriv, 0x24) | BIT(0));
		pw = (rtl_read_word(rtlpriv, 0x04) & 0xe7ff) | 0x0800;
		rtl_write_word(rtlpriv, 0x04, pw);
		while (!(rtl_read_dword(rtlpriv, 0x04) & 0x00020000) &&
		       ++round < 10000)
			;
		rtl_write_word(rtlpriv, 0x04, rtl_read_word(rtlpriv, 0x04) & 0x7fff);
		rtl_write_word(rtlpriv, 0x04, rtl_read_word(rtlpriv, 0x04) & 0xe7ff);
		mdelay(1);
		rtl_write_byte(rtlpriv, 0x05, rtl_read_byte(rtlpriv, 0x05) | BIT(0));
		round = 0;
		while ((rtl_read_byte(rtlpriv, 0x05) & BIT(0)) && ++round < 1000)
			udelay(100);
		rtl_write_byte(rtlpriv, 0x1d, rtl_read_byte(rtlpriv, 0x1d) & ~BIT(0));
		rtl_write_word(rtlpriv, 0x02, rtl_read_word(rtlpriv, 0x02) & ~BIT(10));
		udelay(2);
		if (rtl_read_dword(rtlpriv, 0xf0) & BIT(24))
			rtl_write_byte(rtlpriv, 0x7c, 0xc3);
		else
			rtl_write_byte(rtlpriv, 0x7c, 0x83);
	}

	/* AFE PLL/XTAL are set by the crystal-select (AFE_CTRL5 BIT_REF_SEL=25M)
	 * in the power-on tail above + the pwrseq MAC table; the old hardcoded 40 MHz
	 * block here re-mistuned the PLL on this 25 MHz board, re-breaking the
	 * config-core clock and hanging high-offset writes in phy_mac_config. */

	if (!rtstatus) {
		pr_err("Init MAC failed\n");
		err = 1;
		return err;
	}

	rtl_write_word(rtlpriv, REG_PCIE_CTRL_REG, 0x8000);

	/* Download the 8051 firmware. With the 25 MHz crystal-select + the
	 * power-on tail in place, the high-offset FW-FIFO writes (0x4000) complete,
	 * so the real download path runs. Non-fatal: even on failure the radio
	 * still brings up for scan/monitor. */
	err = rtl92fe_download_fw(hw, false);
	rtlhal->fw_ready = !err;
	if (err)
		pr_warn("rtl8192fe: FW download failed (err=%d)\n", err);
	err = 0;
	/* FW-related variable init. */
	ppsc->fw_current_inpsmode = false;
	rtlhal->fw_ps_state = FW_PS_STATE_ALL_ON_92F;
	rtlhal->fw_clk_change_in_progress = false;
	rtlhal->allow_sw_to_change_hwclc = false;
	rtlhal->last_hmeboxnum = 0;

	/* BB/RF bring-up via phy.c (MAC table, BB, RF, then channel). */
	rtl92fe_phy_mac_config(hw);
	rtl92fe_phy_bb_config(hw);
	rtl92fe_phy_rf_config(hw);

	rtlphy->rfreg_chnlval[0] = rtl_get_rfreg(hw, RF90_PATH_A,
						 RF_CHNLBW, RFREG_OFFSET_MASK);
	rtlphy->rfreg_chnlval[1] = rtl_get_rfreg(hw, RF90_PATH_B,
						 RF_CHNLBW, RFREG_OFFSET_MASK);
	rtlphy->backup_rf_0x1a = (u32)rtl_get_rfreg(hw, RF90_PATH_A, RF_RX_G1,
						    RFREG_OFFSET_MASK);
	rtlphy->rfreg_chnlval[0] = (rtlphy->rfreg_chnlval[0] & 0xfffff3ff) |
				   BIT(10) | BIT(11);
	/* Seed path B's tracked channel word from path A's known-good value
	 * (band + BW20 bits) so sw_chnl never re-keys RF_B[0x18] from a stale
	 * power-on read.  Belt-and-suspenders with the sw_chnl full-word fix. */
	rtlphy->rfreg_chnlval[1] = rtlphy->rfreg_chnlval[0];

	rtl_set_rfreg(hw, RF90_PATH_A, RF_CHNLBW, RFREG_OFFSET_MASK,
		      rtlphy->rfreg_chnlval[0]);
	rtl_set_rfreg(hw, RF90_PATH_B, RF_CHNLBW, RFREG_OFFSET_MASK,
		      rtlphy->rfreg_chnlval[0]);

	/* ---- Set CCK and OFDM block "ON" ---- */
	rtl_set_bbreg(hw, RFPGA0_RFMOD, BCCKEN, 0x1);
	rtl_set_bbreg(hw, RFPGA0_RFMOD, BOFDMEN, 0x1);

	/* RX-sensitivity LNA bias trim required by the RTL8192F radio. */
	rtl_set_rfreg(hw, RF90_PATH_A, 0xB1, RFREG_OFFSET_MASK, 0x33B8F);

	/* Set hardware (MAC default setting). */
	_rtl92fe_hw_configure(hw);

	rtlhal->mac_func_enable = true;

	rtl_cam_reset_all_entry(hw);
	rtl92fe_enable_hw_security_config(hw);

	ppsc->rfpwr_state = ERFON;

	rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_ETHER_ADDR, mac->mac_addr);
	_rtl92fe_enable_aspm_back_door(hw);
	rtlpriv->intf_ops->enable_aspm(hw);

	rtl92fe_bt_hw_init(hw);

	rtlpriv->rtlhal.being_init_adapter = false;

	/* Trim the crystal load cap into the AFE registers (8192F: XTAL1 =
	 * REG_AFE_PLL_CTRL[6:1], XTAL0 = REG_AFE_XTAL_CTRL[30:25]) before
	 * calibration. Without this the 25 MHz crystal is untrimmed and RX
	 * cannot demodulate. (The old 8192EE put this in REG_MAC_PHY_CTRL.)
	 * This board's WiFi EFUSE crystalcap reads 0x00 (invalid), so a sane
	 * default is used via the xtal_cap module parameter. */
	{
		u8 cc = (xtal_cap >= 0) ? (xtal_cap & 0x3f)
				       : (rtlpriv->efuse.crystalcap & 0x3f);
		u32 x1 = rtl_read_dword(rtlpriv, REG_AFE_PLL_CTRL);
		u32 x0 = rtl_read_dword(rtlpriv, REG_AFE_XTAL_CTRL);

		x1 = (x1 & ~0x7eu) | (cc << 1);
		x0 = (x0 & ~0x7e000000u) | (cc << 25);
		rtl_write_dword(rtlpriv, REG_AFE_PLL_CTRL, x1);
		rtl_write_dword(rtlpriv, REG_AFE_XTAL_CTRL, x0);
	}

	/* Run the 8192F TX/RX-path ("TRX mode") init that must follow BB config
	 * (AB/AB paths, rfe_type 3). Skipping it leaves the path registers
	 * mis-configured (0x804[3:0], 0xc04, 0x90c, ...).
	 */
	_rtl92fe_config_trx_mode_ab(hw);

	/* RX/TX engine is configured (RCR/CR) via the MAC init + hw_configure
	 * above; finish with calibration.
	 */
	if (ppsc->rfpwr_state == ERFON) {
		/* The RTL8192F runs LCK then a 3-run IQK (TX-LOK+IQK then
		 * RX-IQK, path A then B); the heavy lifting lives in phy.c.
		 */
		rtl92fe_phy_lc_calibrate(hw);
		if (rtlphy->iqk_initialized) {
			rtl92fe_phy_iq_calibrate(hw, true);
		} else {
			rtl92fe_phy_iq_calibrate(hw, false);
			rtlphy->iqk_initialized = true;
		}
	}

	rtlphy->rfpath_rx_enable[0] = true;
	if (rtlphy->rf_type == RF_2T2R)
		rtlphy->rfpath_rx_enable[1] = true;

	/* PA-bias compensation per efuse flag at raw byte 0x1FA. */
	efuse_one_byte_read(hw, 0x1FA, &tmp_u1b);
	if (!(tmp_u1b & BIT(0))) {
		rtl_set_rfreg(hw, RF90_PATH_A, 0x15, 0x0F, 0x05);
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, "PA BIAS path A\n");
	}

	if ((!(tmp_u1b & BIT(1))) && (rtlphy->rf_type == RF_2T2R)) {
		rtl_set_rfreg(hw, RF90_PATH_B, 0x15, 0x0F, 0x05);
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, "PA BIAS path B\n");
	}

	rtl_write_byte(rtlpriv, REG_NAV_UPPER, ((30000 + 127) / 128));

	rtl92fe_dm_init(hw);

	/* One-line RF bring-up summary (info level so it shows in dmesg without
	 * raising the debug mask). Confirms which chip version/RFE the driver
	 * latched and that the RF-serial path is alive: a sane non-zero RF
	 * read of A:0x18 (RF_CHNLBW) / A:0x00 (RF mode) means the radio keyed
	 * up; rfe_type picks the front-end/IQK path. */
	pr_info("rtl8192fe: RF up: chipver=0x%x rf_type=%s rfe_type=0x%x xtal_cap=0x%x RF_A[0x00]=0x%05x RF_A[0x18]=0x%05x RF_B[0x18]=0x%05x\n",
		rtlhal->version,
		(rtlphy->rf_type == RF_2T2R) ? "2T2R" : "1T1R",
		rtlhal->rfe_type,
		(xtal_cap >= 0) ? (xtal_cap & 0x3f)
				: (rtlpriv->efuse.crystalcap & 0x3f),
		rtl_get_rfreg(hw, RF90_PATH_A, RF_AC, RFREG_OFFSET_MASK),
		rtl_get_rfreg(hw, RF90_PATH_A, RF_CHNLBW, RFREG_OFFSET_MASK),
		rtl_get_rfreg(hw, RF90_PATH_B, RF_CHNLBW, RFREG_OFFSET_MASK));

	/* Post-IQK operating dump: TX-IQ correction result (0xc80/0xc94 path A,
	 * 0xc88/0xc9c path B), TX path-enable (0x90c), TX-AGC rate18-06 (0xe00)
	 * and RF mode 0x00 for both paths. Diff path-A 0xc80/0xc94/0xe00/RF0x00
	 * against the working stock twin: a 0xc80 stuck at the table default
	 * 0x40000100 with reg_e94==0x100/reg_e9c==0 means IQK produced no TX-IQ
	 * correction (ran but failed -> identity), while a non-default 0xc80 with
	 * a sane gain (~0x100 in [31:22]) and small phase means IQK took. */
	pr_info("rtl8192fe: PHY op: 0xc80=0x%08x 0xc94=0x%08x 0xc88=0x%08x 0xc9c=0x%08x 0x90c=0x%08x 0xe00=0x%08x RF_A[0x00]=0x%05x RF_B[0x00]=0x%05x iqk{e94=0x%x e9c=0x%x eb4=0x%x ebc=0x%x}\n",
		rtl_get_bbreg(hw, ROFDM0_XATXIQIMBALANCE, MASKDWORD),
		rtl_get_bbreg(hw, ROFDM0_XCTXAFE, MASKDWORD),
		rtl_get_bbreg(hw, ROFDM0_XBTXIQIMBALANCE, MASKDWORD),
		rtl_get_bbreg(hw, ROFDM0_XDTXAFE, MASKDWORD),
		rtl_get_bbreg(hw, 0x90c, MASKDWORD),
		rtl_get_bbreg(hw, RTXAGC_A_RATE18_06, MASKDWORD),
		rtl_get_rfreg(hw, RF90_PATH_A, RF_AC, RFREG_OFFSET_MASK),
		rtl_get_rfreg(hw, RF90_PATH_B, RF_AC, RFREG_OFFSET_MASK),
		(u32)rtlphy->reg_e94, (u32)rtlphy->reg_e9c,
		(u32)rtlphy->reg_eb4, (u32)rtlphy->reg_ebc);

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
		"end of Rtl8192FE hw init %x\n", err);
	return 0;
}

static enum version_8192f _rtl92fe_read_chip_version(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_phy *rtlphy = &rtlpriv->phy;
	enum version_8192f version;
	u32 value32;

	/* The RTL8192F silicon is a 2-chain (2T2R) RECEIVER, and rf_type gates the RX
	 * chain enable, path-B PA-bias and dual-path IQK/AGC (hw.c:1552/1562, dm.c) --
	 * so it MUST track the silicon or the AP goes deaf and never hears the client's
	 * 4-way M2 (setting RF_1T1R here disabled path-B RX and stalled association).
	 * The MCS15 seen in the TX spy is only the PRE-override negotiated rate; AP
	 * unicast DATA is already force-pinned to legacy DESC_RATE54M in tx_fill_desc,
	 * so the on-air data rate is single-stream regardless. The single-stream cap
	 * belongs in the rate path, NOT in a lie about the receiver's chain count. */
	rtlphy->rf_type = RF_2T2R;

	value32 = rtl_read_dword(rtlpriv, REG_SYS_CFG1);
	if (value32 & TRP_VAUX_EN)
		version = (enum version_8192f)VERSION_TEST_CHIP_2T2R_8192F;
	else
		version = (enum version_8192f)VERSION_NORMAL_CHIP_2T2R_8192F;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
		"Chip RF Type: %s\n", (rtlphy->rf_type == RF_2T2R) ?
		"RF_2T2R" : "RF_1T1R");

	return version;
}

static int _rtl92fe_set_media_status(struct ieee80211_hw *hw,
				     enum nl80211_iftype type)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 bt_msr = rtl_read_byte(rtlpriv, MSR) & 0xfc;
	enum led_ctl_mode ledaction = LED_CTL_NO_LINK;
	u8 mode = MSR_NOLINK;

	switch (type) {
	case NL80211_IFTYPE_UNSPECIFIED:
		mode = MSR_NOLINK;
		rtl_dbg(rtlpriv, COMP_INIT, DBG_TRACE,
			"Set Network type to NO LINK!\n");
		break;
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_MESH_POINT:
		mode = MSR_ADHOC;
		rtl_dbg(rtlpriv, COMP_INIT, DBG_TRACE,
			"Set Network type to Ad Hoc!\n");
		break;
	case NL80211_IFTYPE_STATION:
		mode = MSR_INFRA;
		ledaction = LED_CTL_LINK;
		rtl_dbg(rtlpriv, COMP_INIT, DBG_TRACE,
			"Set Network type to STA!\n");
		break;
	case NL80211_IFTYPE_AP:
		mode = MSR_AP;
		ledaction = LED_CTL_LINK;
		rtl_dbg(rtlpriv, COMP_INIT, DBG_TRACE,
			"Set Network type to AP!\n");
		break;
	default:
		pr_err("Network type %d not support!\n", type);
		return 1;
	}

	if (mode != MSR_AP && rtlpriv->mac80211.link_state < MAC80211_LINKED) {
		mode = MSR_NOLINK;
		ledaction = LED_CTL_NO_LINK;
	}

	if (mode == MSR_NOLINK || mode == MSR_INFRA) {
		_rtl92fe_stop_tx_beacon(hw);
		_rtl92fe_enable_bcn_sub_func(hw);
	} else if (mode == MSR_ADHOC || mode == MSR_AP) {
		_rtl92fe_resume_tx_beacon(hw);
		_rtl92fe_disable_bcn_sub_func(hw);
	} else {
		rtl_dbg(rtlpriv, COMP_ERR, DBG_WARNING,
			"Set HW_VAR_MEDIA_STATUS: No such media status(%x).\n",
			mode);
	}

	rtl_write_byte(rtlpriv, MSR, bt_msr | mode);
	rtlpriv->cfg->ops->led_control(hw, ledaction);
	if (mode == MSR_AP)
		rtl_write_byte(rtlpriv, REG_BCNTCFG + 1, 0x00);
	else
		rtl_write_byte(rtlpriv, REG_BCNTCFG + 1, 0x66);
	return 0;
}

void rtl92fe_set_check_bssid(struct ieee80211_hw *hw, bool check_bssid)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	u32 reg_rcr = rtlpci->receive_config;

	if (rtlpriv->psc.rfpwr_state != ERFON)
		return;

	if (check_bssid) {
		reg_rcr |= (RCR_CBSSID_DATA | RCR_CBSSID_BCN);
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_RCR,
					      (u8 *)(&reg_rcr));
		_rtl92fe_set_bcn_ctrl_reg(hw, 0, BIT(4));
	} else {
		reg_rcr &= (~(RCR_CBSSID_DATA | RCR_CBSSID_BCN));
		_rtl92fe_set_bcn_ctrl_reg(hw, BIT(4), 0);
		rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_RCR,
					      (u8 *)(&reg_rcr));
	}
}

int rtl92fe_set_network_type(struct ieee80211_hw *hw, enum nl80211_iftype type)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	if (_rtl92fe_set_media_status(hw, type))
		return -EOPNOTSUPP;

	/* check-BSSID (drop RX not matching our BSSID) is wanted ONLY for an
	 * infrastructure STA that is LINKED. AP/mesh must NEVER check-BSSID: it has
	 * to receive frames from unassociated peers -- crucially a phone's wildcard
	 * PROBE REQUEST during a scan. The old code left AP-while-LINKED falling
	 * through BOTH branches, so stale CBSSID bits from a prior state survived
	 * and the HW silently dropped probe requests -> hostapd never answered ->
	 * the phone never listed the AP (worked only from a fresh, non-LINKED
	 * bring-up). Force it explicitly for every case. */
	if (rtlpriv->mac80211.link_state == MAC80211_LINKED &&
	    type != NL80211_IFTYPE_AP &&
	    type != NL80211_IFTYPE_MESH_POINT)
		rtl92fe_set_check_bssid(hw, true);
	else
		rtl92fe_set_check_bssid(hw, false);

	return 0;
}

/* Don't set REG_EDCA_BE_PARAM here because mac80211 sends pkt when scanning. */
void rtl92fe_set_qos(struct ieee80211_hw *hw, int aci)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	rtl92fe_dm_init_edca_turbo(hw);
	switch (aci) {
	case AC1_BK:
		rtl_write_dword(rtlpriv, REG_EDCA_BK_PARAM, 0xa44f);
		break;
	case AC0_BE:
		/* handled by EDCA-turbo */
		break;
	case AC2_VI:
		rtl_write_dword(rtlpriv, REG_EDCA_VI_PARAM, 0x5ea324);
		break;
	case AC3_VO:
		rtl_write_dword(rtlpriv, REG_EDCA_VO_PARAM, 0x2fa226);
		break;
	default:
		WARN_ONCE(true, "rtl8192fe: invalid aci: %d !\n", aci);
		break;
	}
}

void rtl92fe_enable_interrupt(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));

	rtl_write_dword(rtlpriv, REG_HIMR, rtlpci->irq_mask[0] & 0xFFFFFFFF);
	rtl_write_dword(rtlpriv, REG_HIMRE, rtlpci->irq_mask[1] & 0xFFFFFFFF);
	rtlpci->irq_enabled = true;
}

void rtl92fe_disable_interrupt(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));

	rtl_write_dword(rtlpriv, REG_HIMR, IMR_DISABLED);
	rtl_write_dword(rtlpriv, REG_HIMRE, IMR_DISABLED);
	rtlpci->irq_enabled = false;
}

static void _rtl92fe_poweroff_adapter(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	u8 u1b_tmp;

	rtlhal->mac_func_enable = false;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, "POWER OFF adapter\n");

	/* Run LPS WL RFOFF flow */
	rtl_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK,
				 PWR_INTF_PCI_MSK, RTL8192F_NIC_LPS_ENTER_FLOW);
	/* turn off RF */
	rtl_write_byte(rtlpriv, REG_RF_CTRL, 0x00);

	/* ==== Reset digital sequence ==== */
	if ((rtl_read_byte(rtlpriv, REG_MCUFWDL) & BIT(7)) && rtlhal->fw_ready)
		rtl92fe_firmware_selfreset(hw);

	/* Reset MCU */
	u1b_tmp = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN + 1);
	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN + 1, (u1b_tmp & (~BIT(2))));

	/* reset MCU ready status */
	rtl_write_byte(rtlpriv, REG_MCUFWDL, 0x00);

	/* HW card disable configuration. */
	rtl_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK,
				 PWR_INTF_PCI_MSK, RTL8192F_NIC_DISABLE_FLOW);

	/* Reset MCU IO Wrapper */
	u1b_tmp = rtl_read_byte(rtlpriv, REG_RSV_CTRL + 1);
	rtl_write_byte(rtlpriv, REG_RSV_CTRL + 1, (u1b_tmp & (~BIT(0))));
	u1b_tmp = rtl_read_byte(rtlpriv, REG_RSV_CTRL + 1);
	rtl_write_byte(rtlpriv, REG_RSV_CTRL + 1, (u1b_tmp | BIT(0)));

	/* lock ISO/CLK/Power control register */
	rtl_write_byte(rtlpriv, REG_RSV_CTRL, 0x0E);
}

void rtl92fe_card_disable(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	enum nl80211_iftype opmode;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, "RTL8192fe card disable\n");

	RT_SET_PS_LEVEL(ppsc, RT_RF_OFF_LEVL_HALT_NIC);

	mac->link_state = MAC80211_NOLINK;
	opmode = NL80211_IFTYPE_UNSPECIFIED;

	_rtl92fe_set_media_status(hw, opmode);

	if (rtlpriv->rtlhal.driver_is_goingto_unload ||
	    ppsc->rfoff_reason > RF_CHANGE_BY_PS)
		rtlpriv->cfg->ops->led_control(hw, LED_CTL_POWER_OFF);

	_rtl92fe_poweroff_adapter(hw);

	/* P3: after power off we must redo IQK. Clear iqk_initialized
	 * UNCONDITIONALLY -- get_btc_status() returns true for this chip, so the
	 * old btc gate skipped the clear, and every post-reload hw_init then took
	 * the "restore from backup" IQK path, re-imposing the FIRST boot's result
	 * (which can settle to identity) on a freshly reset baseband. Only a reboot
	 * re-ran a real IQK. Clearing here makes a reload recalibrate for real. */
	rtlpriv->phy.iqk_initialized = false;
}

void rtl92fe_interrupt_recognized(struct ieee80211_hw *hw,
				  struct rtl_int *intvec)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));

	intvec->inta = rtl_read_dword(rtlpriv, ISR) & rtlpci->irq_mask[0];
	rtl_write_dword(rtlpriv, ISR, intvec->inta);

	intvec->intb = rtl_read_dword(rtlpriv, REG_HISRE) & rtlpci->irq_mask[1];
	rtl_write_dword(rtlpriv, REG_HISRE, intvec->intb);
}

void rtl92fe_set_beacon_related_registers(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));
	u16 bcn_interval, atim_window;

	bcn_interval = mac->beacon_interval;
	atim_window = 2;
	rtl92fe_disable_interrupt(hw);
	rtl_write_word(rtlpriv, REG_ATIMWND, atim_window);
	rtl_write_word(rtlpriv, REG_BCN_INTERVAL, bcn_interval);
	rtl_write_word(rtlpriv, REG_BCNTCFG, 0x660f);
	rtl_write_byte(rtlpriv, REG_RXTSF_OFFSET_CCK, 0x18);
	rtl_write_byte(rtlpriv, REG_RXTSF_OFFSET_OFDM, 0x18);
	rtl_write_byte(rtlpriv, REG_RXTSF_OFFSET_OFDM - 2, 0x30);
	rtlpci->reg_bcn_ctrl_val |= BIT(3);
	rtl_write_byte(rtlpriv, REG_BCN_CTRL, (u8)rtlpci->reg_bcn_ctrl_val);
	/* NB: do NOT set ENSWBCN (REG_CR bit8) here to force a live-TIM SW beacon --
	 * the rtl8192fe SW-beacon path (BEACON_QUEUE) has no DWBCN commit/beacon-valid
	 * handshake, so the tasklet-filled beacon misses TBTT on the slow MIPS core
	 * and the AP goes intermittently/fully SILENT (clients can't even scan it).
	 * Keep the reliable FW-auto reserved-page beacon. Delivering buffered
	 * downstream to a legacy-PS STA needs the reserved-page beacon rebuilt with a
	 * live TIM -- a separate, carefully-soaked project. */
	/* P4: this function disabled interrupts above (to program beacon regs
	 * atomically) but the stock code never re-enabled them, so on the FIRST
	 * AP bring-up beacon/TX/RX IRQs stay masked until some later path happens
	 * to call update_interrupt_mask -- until then beacon DMA is never serviced
	 * and the AP is enabled but silent (the "not on-air until wifi reload"
	 * quirk). Re-enable here so the beacon starts on the first bring-up. */
	rtl92fe_enable_interrupt(hw);
}

void rtl92fe_set_beacon_interval(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	u16 bcn_interval = mac->beacon_interval;

	rtl_dbg(rtlpriv, COMP_BEACON, DBG_DMESG,
		"beacon_interval:%d\n", bcn_interval);
	rtl_write_word(rtlpriv, REG_BCN_INTERVAL, bcn_interval);
}

void rtl92fe_update_interrupt_mask(struct ieee80211_hw *hw,
				   u32 add_msr, u32 rm_msr)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));

	rtl_dbg(rtlpriv, COMP_INTR, DBG_LOUD,
		"add_msr:%x, rm_msr:%x\n", add_msr, rm_msr);

	if (add_msr)
		rtlpci->irq_mask[0] |= add_msr;
	if (rm_msr)
		rtlpci->irq_mask[0] &= (~rm_msr);
	rtl92fe_disable_interrupt(hw);
	rtl92fe_enable_interrupt(hw);
}

static __always_inline u8 _rtl92fe_get_chnl_group(u8 chnl)
{
	u8 group = 0;

	/* The RTL8192F is a 2.4 GHz-only part: only the 14-channel
	 * 5-group mapping applies.
	 */
	if (chnl >= 1 && chnl <= 2)
		group = 0;
	else if (chnl >= 3 && chnl <= 5)
		group = 1;
	else if (chnl >= 6 && chnl <= 8)
		group = 2;
	else if (chnl >= 9 && chnl <= 11)
		group = 3;
	else if (chnl >= 12 && chnl <= 14)
		group = 4;

	return group;
}

static void _rtl92fe_read_power_value_fromprom(struct ieee80211_hw *hw,
					       struct txpower_info_2g *pwr2g,
					       bool autoload_fail, u8 *hwinfo)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u32 rf, addr = EEPROM_TX_PWR_INX, group, i = 0;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
		"hal_ReadPowerValueFromPROM92F(): PROMContent[0x%x]=0x%x\n",
		(addr + 1), hwinfo[addr + 1]);
	if (hwinfo[addr + 1] == 0xFF)	/* signature byte unprogrammed */
		autoload_fail = true;

	if (autoload_fail) {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
			"auto load fail : Use Default value!\n");
		for (rf = 0; rf < MAX_RF_PATH; rf++) {
			/* 2.4 GHz default value */
			for (group = 0; group < MAX_CHNL_GROUP_24G; group++) {
				pwr2g->index_cck_base[rf][group] = 0x2D;
				pwr2g->index_bw40_base[rf][group] = 0x2D;
			}
			for (i = 0; i < MAX_TX_COUNT; i++) {
				if (i == 0) {
					pwr2g->bw20_diff[rf][0] = 0x02;
					pwr2g->ofdm_diff[rf][0] = 0x04;
				} else {
					pwr2g->bw20_diff[rf][i] = 0xFE;
					pwr2g->bw40_diff[rf][i] = 0xFE;
					pwr2g->cck_diff[rf][i] = 0xFE;
					pwr2g->ofdm_diff[rf][i] = 0xFE;
				}
			}
		}
		return;
	}

	rtl_priv(hw)->efuse.txpwr_fromeprom = true;

	for (rf = 0; rf < MAX_RF_PATH; rf++) {
		/* 2.4 GHz CCK base */
		for (group = 0; group < MAX_CHNL_GROUP_24G; group++) {
			pwr2g->index_cck_base[rf][group] = hwinfo[addr++];
			if (pwr2g->index_cck_base[rf][group] == 0xFF)
				pwr2g->index_cck_base[rf][group] = 0x2D;
		}
		/* 2.4 GHz BW40 base (5 group bytes; last reused for ch14) */
		for (group = 0; group < MAX_CHNL_GROUP_24G - 1; group++) {
			pwr2g->index_bw40_base[rf][group] = hwinfo[addr++];
			if (pwr2g->index_bw40_base[rf][group] == 0xFF)
				pwr2g->index_bw40_base[rf][group] = 0x2D;
		}
		for (i = 0; i < MAX_TX_COUNT; i++) {
			if (i == 0) {
				pwr2g->bw40_diff[rf][i] = 0;
				if (hwinfo[addr] == 0xFF) {
					pwr2g->bw20_diff[rf][i] = 0x02;
				} else {
					pwr2g->bw20_diff[rf][i] = (hwinfo[addr]
								   & 0xf0) >> 4;
					if (pwr2g->bw20_diff[rf][i] & BIT(3))
						pwr2g->bw20_diff[rf][i] |= 0xF0;
				}

				if (hwinfo[addr] == 0xFF) {
					pwr2g->ofdm_diff[rf][i] = 0x04;
				} else {
					pwr2g->ofdm_diff[rf][i] = (hwinfo[addr]
								   & 0x0f);
					if (pwr2g->ofdm_diff[rf][i] & BIT(3))
						pwr2g->ofdm_diff[rf][i] |= 0xF0;
				}
				pwr2g->cck_diff[rf][i] = 0;
				addr++;
			} else {
				if (hwinfo[addr] == 0xFF) {
					pwr2g->bw40_diff[rf][i] = 0xFE;
				} else {
					pwr2g->bw40_diff[rf][i] = (hwinfo[addr]
								   & 0xf0) >> 4;
					if (pwr2g->bw40_diff[rf][i] & BIT(3))
						pwr2g->bw40_diff[rf][i] |= 0xF0;
				}

				if (hwinfo[addr] == 0xFF) {
					pwr2g->bw20_diff[rf][i] = 0xFE;
				} else {
					pwr2g->bw20_diff[rf][i] = (hwinfo[addr]
								   & 0x0f);
					if (pwr2g->bw20_diff[rf][i] & BIT(3))
						pwr2g->bw20_diff[rf][i] |= 0xF0;
				}
				addr++;

				if (hwinfo[addr] == 0xFF) {
					pwr2g->ofdm_diff[rf][i] = 0xFE;
				} else {
					pwr2g->ofdm_diff[rf][i] = (hwinfo[addr]
								   & 0xf0) >> 4;
					if (pwr2g->ofdm_diff[rf][i] & BIT(3))
						pwr2g->ofdm_diff[rf][i] |= 0xF0;
				}

				if (hwinfo[addr] == 0xFF) {
					pwr2g->cck_diff[rf][i] = 0xFE;
				} else {
					pwr2g->cck_diff[rf][i] = (hwinfo[addr]
								  & 0x0f);
					if (pwr2g->cck_diff[rf][i] & BIT(3))
						pwr2g->cck_diff[rf][i] |= 0xF0;
				}
				addr++;
			}
		}

		/* The 2.4 GHz-only RTL8192F has no 5 GHz tx-power block, but
		 * the path-B sub-table still begins at the 0x3A offset; the
		 * EEPROM_TX_PWR_INX base + per-rf stride lands path B there.
		 */
	}
}

static noinline_for_stack void
_rtl92fe_read_txpower_info_from_hwpg(struct ieee80211_hw *hw,
				     bool autoload_fail, u8 *hwinfo)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_efuse *efu = rtl_efuse(rtl_priv(hw));
	struct txpower_info_2g pwr2g;
	u8 rf, idx;
	u8 i;

	_rtl92fe_read_power_value_fromprom(hw, &pwr2g, autoload_fail, hwinfo);

	for (rf = 0; rf < MAX_RF_PATH; rf++) {
		for (i = 0; i < 14; i++) {
			idx = _rtl92fe_get_chnl_group(i + 1);

			if (i == CHANNEL_MAX_NUMBER_2G - 1) {
				efu->txpwrlevel_cck[rf][i] =
						pwr2g.index_cck_base[rf][5];
				efu->txpwrlevel_ht40_1s[rf][i] =
						pwr2g.index_bw40_base[rf][idx];
			} else {
				efu->txpwrlevel_cck[rf][i] =
						pwr2g.index_cck_base[rf][idx];
				efu->txpwrlevel_ht40_1s[rf][i] =
						pwr2g.index_bw40_base[rf][idx];
			}
		}
		for (i = 0; i < MAX_TX_COUNT; i++) {
			efu->txpwr_cckdiff[rf][i] = pwr2g.cck_diff[rf][i];
			efu->txpwr_legacyhtdiff[rf][i] = pwr2g.ofdm_diff[rf][i];
			efu->txpwr_ht20diff[rf][i] = pwr2g.bw20_diff[rf][i];
			efu->txpwr_ht40diff[rf][i] = pwr2g.bw40_diff[rf][i];
		}
	}

	/* thermal meter @ 0xBA */
	if (!autoload_fail)
		efu->eeprom_thermalmeter = hwinfo[EEPROM_THERMAL_METER_92F];
	else
		efu->eeprom_thermalmeter = EEPROM_DEFAULT_THERMALMETER;

	if (efu->eeprom_thermalmeter == 0xff || autoload_fail) {
		efu->apk_thermalmeterignore = true;
		efu->eeprom_thermalmeter = EEPROM_DEFAULT_THERMALMETER;
	}

	efu->thermalmeter[0] = efu->eeprom_thermalmeter;
	RTPRINT(rtlpriv, FINIT, INIT_TXPOWER,
		"thermalmeter = 0x%x\n", efu->eeprom_thermalmeter);

	/* RFE/board option @ 0xCA on the RTL8192F (& 0x1F). */
	if (!autoload_fail) {
		efu->eeprom_regulatory = hwinfo[EEPROM_RFE_OPTION_92F] & 0x07;
		if (hwinfo[EEPROM_RFE_OPTION_92F] == 0xFF)
			efu->eeprom_regulatory = 0;
	} else {
		efu->eeprom_regulatory = 0;
	}
	RTPRINT(rtlpriv, FINIT, INIT_TXPOWER,
		"eeprom_regulatory = 0x%x\n", efu->eeprom_regulatory);

	/* The RFE type (efuse RFE option @0xCA, bits[4:0]) selects the RF
	 * front-end variant: it drives the IQK PAD_TXG branch and the
	 * external-PA/LNA RFE antenna-switch overrides (rfe 7/8/9/12). It was
	 * previously left at the zero-init default, so an external-PA board
	 * was silently calibrated as internal-PA -> the front-end never keyed
	 * up. Mirror the mainline behaviour and latch it from efuse here.
	 * 0xFF (unprogrammed) falls back to type 0 (internal PA/LNA). */
	if (!autoload_fail && hwinfo[EEPROM_RFE_OPTION_92F] != 0xFF)
		rtl_hal(rtlpriv)->rfe_type = hwinfo[EEPROM_RFE_OPTION_92F] & 0x1f;
	else
		rtl_hal(rtlpriv)->rfe_type = 0;
	RTPRINT(rtlpriv, FINIT, INIT_TXPOWER,
		"rfe_type = 0x%x\n", rtl_hal(rtlpriv)->rfe_type);
}

static void _rtl92fe_read_adapter_info(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_efuse *rtlefuse = rtl_efuse(rtl_priv(hw));
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	/* The RTL8192F efuse signature (0x8129) lives at offset 0x00 and
	 * the per-board MAC at 0x107. The MAC pulled here is ONLY an
	 * identity hint / fallback: the operational MAC is provisioned
	 * from the board (DT/nvmem via of_get_mac_address(), else
	 * SoC-derived) and must NOT be baked from efuse.
	 */
	int params[] = {RTL8192F_EEPROM_ID, EEPROM_VID_92F, EEPROM_DID_92F,
			EEPROM_SVID_92F, EEPROM_SMID_92F, EEPROM_MAC_ADDR_92F,
			EEPROM_CHANNELPLAN_92F, EEPROM_VERSION_92F,
			EEPROM_CUSTOMER_ID_92F, COUNTRY_CODE_WORLD_WIDE_13};
	u8 *hwinfo;

	hwinfo = kzalloc(HWSET_MAX_SIZE, GFP_KERNEL);
	if (!hwinfo)
		return;

	if (rtl_get_hwinfo(hw, rtlpriv, HWSET_MAX_SIZE, hwinfo, params))
		goto exit;

	if (rtlefuse->eeprom_oemid == 0xFF)
		rtlefuse->eeprom_oemid = 0;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
		"EEPROM Customer ID: 0x%2x\n", rtlefuse->eeprom_oemid);
	/* set channel plan from efuse (@ 0xB8) */
	rtlefuse->channel_plan = rtlefuse->eeprom_channelplan;
	/* tx power */
	_rtl92fe_read_txpower_info_from_hwpg(hw, rtlefuse->autoload_failflag,
					     hwinfo);

	rtl92fe_read_bt_coexist_info_from_hwpg(hw, rtlefuse->autoload_failflag,
					       hwinfo);

	/* board type (RFE option @ 0xCA, board-type nibble in bits[7:5]) */
	rtlefuse->board_type = (((*(u8 *)&hwinfo[EEPROM_RFE_OPTION_92F])
				& 0xE0) >> 5);
	if ((*(u8 *)&hwinfo[EEPROM_RFE_OPTION_92F]) == 0xFF)
		rtlefuse->board_type = 0;

	if (rtlpriv->btcoexist.btc_info.btcoexist == 1)
		rtlefuse->board_type |= BIT(2); /* ODM_BOARD_BT */

	rtlhal->board_type = rtlefuse->board_type;
	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD,
		"board_type = 0x%x\n", rtlefuse->board_type);
	/* parse xtal (@ 0xB9, & 0x3F) */
	rtlefuse->crystalcap = hwinfo[EEPROM_XTAL_92F] & 0x3F;
	if (hwinfo[EEPROM_XTAL_92F] == 0xFF)
		rtlefuse->crystalcap = 0x20;

	/* antenna diversity */
	rtlefuse->antenna_div_type = NO_ANTDIV;
	rtlefuse->antenna_div_cfg = 0;

	if (rtlhal->oem_id == RT_CID_DEFAULT) {
		switch (rtlefuse->eeprom_oemid) {
		case EEPROM_CID_DEFAULT:
			if (rtlefuse->eeprom_did == 0x818C) {
				if ((rtlefuse->eeprom_svid == 0x10EC) &&
				    (rtlefuse->eeprom_smid == 0x001B))
					rtlhal->oem_id = RT_CID_819X_LENOVO;
			} else {
				rtlhal->oem_id = RT_CID_DEFAULT;
			}
			break;
		default:
			rtlhal->oem_id = RT_CID_DEFAULT;
			break;
		}
	}
exit:
	kfree(hwinfo);
}

static void _rtl92fe_hal_customized_behavior(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));

	rtlpriv->ledctl.led_opendrain = true;

	rtl_dbg(rtlpriv, COMP_INIT, DBG_DMESG,
		"RT Customized ID: 0x%02X\n", rtlhal->oem_id);
}

/* ------------------------------------------------------------------------- *
 * Board factory WiFi calibration (flash apmib HW_WLAN0_*)
 *
 * The RTL8192FE on this HSGQ X111W ships with a BLANK PCIe efuse (signature
 * byte != 0x8129).  The rtlwifi core therefore sets autoload_failflag and the
 * normal _rtl92fe_read_adapter_info() path reads nothing: tx-power, MAC,
 * crystalcap and thermal meter are all left zero, so the radio comes up with a
 * random 00:e0:4c MAC, generic default tx-power and an untrimmed crystal -> the
 * AP beacons off-frequency / at the wrong power and is invisible to clients.
 *
 * The real per-chip cal lives in the board's NOR flash apmib (HW_WLAN0_*); the
 * stock WiFi driver reads it from there, not from efuse.  Until we wire
 * a flash/apmib reader (or a DT/nvmem cell) into the clean-room driver, the
 * values for THIS board are baked here so the radio calibrates like stock.
 *
 * Single 2.4 GHz 1T1R RTL8192FE, 14 channels (ch1..ch14).  All tx-power tables
 * below are PER-CHANNEL (one byte per channel), matching the apmib layout, so
 * they are copied straight into the driver's per-channel txpwrlevel_* arrays
 * (no efuse group-byte -> channel expansion is needed).
 *
 * TODO: source these from flash apmib HW_WLAN0_* (or a DT/nvmem cell) so the
 * driver works across units instead of carrying one board's cal.
 * ------------------------------------------------------------------------- */
struct rtl92fe_board_cal {
	u8 mac[ETH_ALEN];
	/* per-channel (ch1..ch14) CCK base power, path A / path B */
	u8 cck_a[CHANNEL_MAX_NUMBER_2G];
	u8 cck_b[CHANNEL_MAX_NUMBER_2G];
	/* per-channel (ch1..ch14) HT40 1S base power, path A / path B */
	u8 ht40_1s_a[CHANNEL_MAX_NUMBER_2G];
	u8 ht40_1s_b[CHANNEL_MAX_NUMBER_2G];
	u8 thermalmeter;	/* HW_WLAN0_11N_THER  */
	u8 crystalcap;		/* HW_WLAN0_11N_XCAP  (xtal load cap) */
	u8 pa_type;		/* HW_WLAN0_11N_PA_TYPE (0 = internal PA) */
	u8 reg_domain;		/* HW_WLAN0_REG_DOMAIN */
};

static const struct rtl92fe_board_cal rtl92fe_x111w_cal = {
	.mac = { 0x98, 0xc7, 0xa4, 0x32, 0x82, 0xae },
	.cck_a = { 0x27, 0x27, 0x27, 0x28, 0x28, 0x28, 0x28,
		   0x28, 0x28, 0x29, 0x29, 0x29, 0x29, 0x29 },
	.cck_b = { 0x26, 0x26, 0x26, 0x28, 0x28, 0x28, 0x28,
		   0x28, 0x28, 0x27, 0x27, 0x27, 0x27, 0x27 },
	.ht40_1s_a = { 0x2e, 0x2e, 0x2e, 0x2e, 0x2e, 0x2e, 0x2e,
		       0x2e, 0x2e, 0x2e, 0x2e, 0x2e, 0x2e, 0x2e },
	.ht40_1s_b = { 0x2c, 0x2c, 0x2c, 0x2c, 0x2c, 0x2c, 0x2c,
		       0x2c, 0x2c, 0x2c, 0x2c, 0x2c, 0x2c, 0x2c },
	.thermalmeter = 0x36,
	.crystalcap = 0x47,
	.pa_type = 0,
	.reg_domain = 1,
};

/* Populate rtlefuse from the board cal table when the efuse is blank, then
 * clear autoload_failflag so the regular RF / tx-power init runs with these
 * values instead of the generic defaults.
 */
static void _rtl92fe_apply_board_cal(struct ieee80211_hw *hw,
				     const struct rtl92fe_board_cal *cal)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_efuse *efu = rtl_efuse(rtl_priv(hw));
	u8 rf, ch;

	/* MAC: identity hint / fallback only.  The operational MAC is still
	 * provisioned from the board (DT/nvmem/uci); but filling dev_addr with
	 * a valid per-chip address stops base.c from assigning a random
	 * 00:e0:4c MAC when no board MAC is present.
	 */
	memcpy(efu->dev_addr, cal->mac, ETH_ALEN);

	/* Per-channel tx-power.  Paths A/B are the only populated RF paths on
	 * this 1T1R part; fill A and B (and mirror onto any extra MAX_RF_PATH
	 * slots from path A so nothing is left at 0).
	 */
	for (rf = 0; rf < MAX_RF_PATH; rf++) {
		const u8 *cck = (rf == RF90_PATH_B) ? cal->cck_b : cal->cck_a;
		const u8 *ht40 = (rf == RF90_PATH_B) ? cal->ht40_1s_b
						     : cal->ht40_1s_a;

		for (ch = 0; ch < CHANNEL_MAX_NUMBER_2G; ch++) {
			efu->txpwrlevel_cck[rf][ch] = cck[ch];
			efu->txpwrlevel_ht40_1s[rf][ch] = ht40[ch];
		}

		/* All power diffs are zero on this board (DIFF_OFDM /
		 * DIFF_HT20 / DIFF_HT40_2S == 0): HT20 == HT40, OFDM == CCK
		 * base, no 2S path.
		 */
		for (ch = 0; ch < MAX_TX_COUNT; ch++) {
			efu->txpwr_cckdiff[rf][ch] = 0;
			efu->txpwr_legacyhtdiff[rf][ch] = 0;
			efu->txpwr_ht20diff[rf][ch] = 0;
			efu->txpwr_ht40diff[rf][ch] = 0;
		}
	}
	efu->txpwr_fromeprom = true;

	/* Thermal meter: drives the thermal-tracking tx-power compensation.
	 * A real (non-0xff) value means we do NOT ignore thermal tracking.
	 */
	efu->eeprom_thermalmeter = cal->thermalmeter;
	efu->thermalmeter[0] = cal->thermalmeter;
	efu->thermalmeter[1] = cal->thermalmeter;
	efu->apk_thermalmeterignore = false;

	/* Crystal load cap (XCAP).  Critical for an on-frequency beacon; the
	 * AFE trim in rtl92fe_hw_init() picks this up via efuse.crystalcap
	 * when xtal_cap < 0 (the default).
	 */
	efu->crystalcap = cal->crystalcap & 0x3f;
	efu->eeprom_crystalcap = cal->crystalcap & 0x3f;

	/* Front-end / regulatory.  The WiFi efuse is blank, so rfe_type cannot be
	 * auto-read.  The board WiFi chip is an RTL8192FR (integrated PA/LNA -- no
	 * external FEM on the PCB), so the PA is physically internal.  Two settings
	 * are decoupled here, both tuned empirically on this board:
	 *  - RFE control pins (T/R switch + RX-LNA routing) MUST be programmed: the
	 *    no-op rfe_type 3 left them at BB-table defaults -> deaf RX (peer heard
	 *    at -100dBm).  _rtl92fe_config_rfe() (from _rtl92fe_config_trx_mode_ab)
	 *    writes the rfe_type-7 RFE-pinmux pattern -- the only PCIe 8192F RFE init
	 *    that enables the T/R switch here; its ext-PA-enable half is inert with
	 *    no external PA fitted.  RX recovers ~68dB.
	 *  - external_pa: despite the integrated (internal) PA, external_pa=1 is
	 *    EMPIRICALLY better -- it selects the ext-PA IQK TX-gain (PAD_TXG 0x30),
	 *    which drives this PA cleanly (~96% link).  external_pa=0 (internal
	 *    PAD_TXG 0xe9) REGRESSED to ~40% with assoc timeouts (the DM under-drives
	 *    the TX), so keep external_pa=1.
	 */
	rtl_hal(rtlpriv)->rfe_type = 7;
	efu->board_type = 0;
	rtl_hal(rtlpriv)->board_type = 0;
	efu->external_pa = 1;
	efu->eeprom_regulatory = cal->reg_domain & 0x07;

	/* Channel plan: 2.4 GHz world-wide 13 (+ ch14 handled per-channel). */
	efu->channel_plan = COUNTRY_CODE_WORLD_WIDE_13;

	/* The cal is now in place: let the normal init treat the efuse as
	 * loaded so RF / tx-power bring-up uses these values.
	 */
	efu->autoload_failflag = false;

	pr_info("rtl8192fe: applied board WiFi cal (X111W): MAC=%pM xtal=0x%02x thermal=0x%02x cck[A1]=0x%02x ht40[A1]=0x%02x reg=%u\n",
		efu->dev_addr, efu->crystalcap, efu->eeprom_thermalmeter,
		efu->txpwrlevel_cck[RF90_PATH_A][0],
		efu->txpwrlevel_ht40_1s[RF90_PATH_A][0],
		efu->eeprom_regulatory);
}

void rtl92fe_read_eeprom_info(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_efuse *rtlefuse = rtl_efuse(rtl_priv(hw));
	struct rtl_phy *rtlphy = &rtlpriv->phy;
	struct rtl_hal *rtlhal = rtl_hal(rtl_priv(hw));
	u8 tmp_u1b;

	rtlhal->version = _rtl92fe_read_chip_version(hw);
	if (get_rf_type(rtlphy) == RF_1T1R) {
		rtlpriv->dm.rfpath_rxenable[0] = true;
	} else {
		rtlpriv->dm.rfpath_rxenable[0] = true;
		rtlpriv->dm.rfpath_rxenable[1] = true;
	}
	rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, "VersionID = 0x%4x\n",
		rtlhal->version);
	tmp_u1b = rtl_read_byte(rtlpriv, REG_9346CR);
	if (tmp_u1b & BIT(4)) {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_DMESG, "Boot from EEPROM\n");
		rtlefuse->epromtype = EEPROM_93C46;
	} else {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_DMESG, "Boot from EFUSE\n");
		rtlefuse->epromtype = EEPROM_BOOT_EFUSE;
	}
	if (tmp_u1b & BIT(5)) {
		rtl_dbg(rtlpriv, COMP_INIT, DBG_LOUD, "Autoload OK\n");
		rtlefuse->autoload_failflag = false;
		_rtl92fe_read_adapter_info(hw);
	} else {
		pr_err("Autoload ERR!!\n");
		rtlefuse->autoload_failflag = true;
	}

	/* Blank efuse (no autoload, or bad signature so rtl_get_hwinfo() left
	 * autoload_failflag set): the per-chip cal is not in efuse on this
	 * board, it is in flash apmib.  Inject the board factory cal and clear
	 * autoload_failflag so the radio calibrates instead of falling back to
	 * a random MAC + generic tx-power + untrimmed crystal.
	 */
	if (rtlefuse->autoload_failflag)
		_rtl92fe_apply_board_cal(hw, &rtl92fe_x111w_cal);

	_rtl92fe_hal_customized_behavior(hw);

	rtlphy->rfpath_rx_enable[0] = true;
	if (rtlphy->rf_type == RF_2T2R)
		rtlphy->rfpath_rx_enable[1] = true;
}

static u8 _rtl92fe_mrate_idx_to_arfr_id(struct ieee80211_hw *hw, u8 rate_index)
{
	u8 ret = 0;

	switch (rate_index) {
	case RATR_INX_WIRELESS_NGB:
		ret = 0;
		break;
	case RATR_INX_WIRELESS_N:
	case RATR_INX_WIRELESS_NG:
		ret = 4;
		break;
	case RATR_INX_WIRELESS_NB:
		ret = 2;
		break;
	case RATR_INX_WIRELESS_GB:
		ret = 6;
		break;
	case RATR_INX_WIRELESS_G:
		ret = 7;
		break;
	case RATR_INX_WIRELESS_B:
		ret = 8;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static void rtl92fe_update_hal_rate_mask(struct ieee80211_hw *hw,
					 struct ieee80211_sta *sta,
					 u8 rssi_level, bool update_bw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_phy *rtlphy = &rtlpriv->phy;
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	struct rtl_sta_info *sta_entry = NULL;
	u32 ratr_bitmap;
	u8 ratr_index;
	u8 curtxbw_40mhz = (sta->deflink.ht_cap.cap &
			    IEEE80211_HT_CAP_SUP_WIDTH_20_40) ? 1 : 0;
	u8 b_curshortgi_40mhz = (sta->deflink.ht_cap.cap &
				 IEEE80211_HT_CAP_SGI_40) ? 1 : 0;
	u8 b_curshortgi_20mhz = (sta->deflink.ht_cap.cap &
				 IEEE80211_HT_CAP_SGI_20) ? 1 : 0;
	enum wireless_mode wirelessmode = 0;
	bool b_shortgi = false;
	u8 rate_mask[7] = {0};
	u8 macid = 0;

	sta_entry = (struct rtl_sta_info *)sta->drv_priv;
	wirelessmode = sta_entry->wireless_mode;
	if (mac->opmode == NL80211_IFTYPE_STATION ||
	    mac->opmode == NL80211_IFTYPE_MESH_POINT)
		curtxbw_40mhz = mac->bw_40;
	else if (mac->opmode == NL80211_IFTYPE_AP ||
		 mac->opmode == NL80211_IFTYPE_ADHOC) {
		macid = sta->aid + 1;
		/* This board's 8192FR has a single usable TX chain, so 2-spatial-
		 * stream rates (MCS8-15) are un-transmittable: an AP unicast DATA
		 * frame emitted at MCS15 reaches the air as a broken 2SS PPDU no
		 * client can decode -- the real "Obtaining IP" root cause (the frame
		 * IS transmitted, the ring drains, but the client rx-decodes nothing).
		 * Cap the peer to single-stream AT THE SOURCE: with rx_mask[1]=0,
		 * _rtl_get_highest_n_rate() returns MCS7 and the ratr_bitmap below
		 * excludes MCS8-15, so every data rate the driver/FW can pick is
		 * deliverable -- WITHOUT touching rf_type (the silicon is a 2-chain
		 * RECEIVER; leaving RF_2T2R keeps RX diversity, which is what lets the
		 * WPA2 4-way complete). Idempotent across rate refreshes. */
		sta->deflink.ht_cap.mcs.rx_mask[1] = 0;
	}

	ratr_bitmap = sta->deflink.supp_rates[0];
	if (mac->opmode == NL80211_IFTYPE_ADHOC)
		ratr_bitmap = 0xfff;

	ratr_bitmap |= (sta->deflink.ht_cap.mcs.rx_mask[1] << 20 |
			sta->deflink.ht_cap.mcs.rx_mask[0] << 12);

	switch (wirelessmode) {
	case WIRELESS_MODE_B:
		ratr_index = RATR_INX_WIRELESS_B;
		if (ratr_bitmap & 0x0000000c)
			ratr_bitmap &= 0x0000000d;
		else
			ratr_bitmap &= 0x0000000f;
		break;
	case WIRELESS_MODE_G:
		ratr_index = RATR_INX_WIRELESS_GB;

		if (rssi_level == 1)
			ratr_bitmap &= 0x00000f00;
		else if (rssi_level == 2)
			ratr_bitmap &= 0x00000ff0;
		else
			ratr_bitmap &= 0x00000ff5;
		break;
	case WIRELESS_MODE_N_24G:
		if (curtxbw_40mhz)
			ratr_index = RATR_INX_WIRELESS_NGB;
		else
			ratr_index = RATR_INX_WIRELESS_NB;

		if (rtlphy->rf_type == RF_1T1R) {
			if (curtxbw_40mhz) {
				if (rssi_level == 1)
					ratr_bitmap &= 0x000f0000;
				else if (rssi_level == 2)
					ratr_bitmap &= 0x000ff000;
				else
					ratr_bitmap &= 0x000ff015;
			} else {
				if (rssi_level == 1)
					ratr_bitmap &= 0x000f0000;
				else if (rssi_level == 2)
					ratr_bitmap &= 0x000ff000;
				else
					ratr_bitmap &= 0x000ff005;
			}
		} else {
			if (curtxbw_40mhz) {
				if (rssi_level == 1)
					ratr_bitmap &= 0x0f8f0000;
				else if (rssi_level == 2)
					ratr_bitmap &= 0x0ffff000;
				else
					ratr_bitmap &= 0x0ffff015;
			} else {
				if (rssi_level == 1)
					ratr_bitmap &= 0x0f8f0000;
				else if (rssi_level == 2)
					ratr_bitmap &= 0x0ffff000;
				else
					ratr_bitmap &= 0x0ffff005;
			}
		}

		if ((curtxbw_40mhz && b_curshortgi_40mhz) ||
		    (!curtxbw_40mhz && b_curshortgi_20mhz)) {
			if (macid == 0)
				b_shortgi = true;
			else if (macid == 1)
				b_shortgi = false;
		}
		break;
	default:
		ratr_index = RATR_INX_WIRELESS_NGB;

		if (rtlphy->rf_type == RF_1T1R)
			ratr_bitmap &= 0x000ff0ff;
		else
			ratr_bitmap &= 0x0f8ff0ff;
		break;
	}
	ratr_index = _rtl92fe_mrate_idx_to_arfr_id(hw, ratr_index);
	sta_entry->ratr_index = ratr_index;

	rtl_dbg(rtlpriv, COMP_RATR, DBG_DMESG,
		"ratr_bitmap :%x\n", ratr_bitmap);
	*(u32 *)&rate_mask = (ratr_bitmap & 0x0fffffff) |
				       (ratr_index << 28);
	rate_mask[0] = macid;
	rate_mask[1] = ratr_index | (b_shortgi ? 0x80 : 0x00);
	rate_mask[2] = curtxbw_40mhz | ((!update_bw) << 3);
	rate_mask[3] = (u8)(ratr_bitmap & 0x000000ff);
	rate_mask[4] = (u8)((ratr_bitmap & 0x0000ff00) >> 8);
	rate_mask[5] = (u8)((ratr_bitmap & 0x00ff0000) >> 16);
	rate_mask[6] = (u8)((ratr_bitmap & 0xff000000) >> 24);
	rtl_dbg(rtlpriv, COMP_RATR, DBG_DMESG,
		"Rate_index:%x, ratr_val:%x, %x:%x:%x:%x:%x:%x:%x\n",
		ratr_index, ratr_bitmap, rate_mask[0], rate_mask[1],
		rate_mask[2], rate_mask[3], rate_mask[4],
		rate_mask[5], rate_mask[6]);
	/* AP/ADHOC peer: the FW rate/security context for this macid (aid+1) is only
	 * usable after the macid is reported CONNECTED. The infra-STA path
	 * (JOINBSSRPT, hw.c:631) only ever registers macid 0, so without this a
	 * use_rate=0 unicast DATA frame addressed to an aid+1 macid is HELD by the FW
	 * (no DOK -> the BE ring fills -> stop-queue latch) -- which is why the DHCP
	 * OFFER and all post-association unicast data never reach the client. Register
	 * the peer macid here, right before its RA-mask, so CONNECT and the rate table
	 * land together (order matters: allocate the FW slot, then populate it). */
	if (macid)		/* != 0 => an AP/ADHOC peer, not the STA-self (macid 0) */
		rtl92fe_set_fw_media_status_rpt_cmd(hw, RT_MEDIA_CONNECT, macid);

	rtl92fe_fill_h2c_cmd(hw, H2C_92F_RA_MASK, 7, rate_mask);
	_rtl92fe_set_bcn_ctrl_reg(hw, BIT(3), 0);
}

void rtl92fe_update_hal_rate_tbl(struct ieee80211_hw *hw,
				 struct ieee80211_sta *sta, u8 rssi_level,
				 bool update_bw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	if (rtlpriv->dm.useramask)
		rtl92fe_update_hal_rate_mask(hw, sta, rssi_level, update_bw);
}

void rtl92fe_update_channel_access_setting(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	u16 sifs_timer;

	rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_SLOT_TIME,
				      (u8 *)&mac->slot_time);
	if (!mac->ht_enable)
		sifs_timer = 0x0a0a;
	else
		sifs_timer = 0x0e0e;
	rtlpriv->cfg->ops->set_hw_reg(hw, HW_VAR_SIFS, (u8 *)&sifs_timer);
}

bool rtl92fe_gpio_radio_on_off_checking(struct ieee80211_hw *hw, u8 *valid)
{
	/* The RTL8192F has no hardware RF-kill GPIO wired on this board;
	 * report the radio as always present and let software RFKILL drive
	 * the on/off state.
	 * TODO(8192f): validate on hardware if a GPIO RF-kill input exists.
	 */
	*valid = 1;
	return true;
}

void rtl92fe_set_key(struct ieee80211_hw *hw, u32 key_index,
		     u8 *p_macaddr, bool is_group, u8 enc_algo,
		     bool is_wepkey, bool clear_all)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_mac *mac = rtl_mac(rtl_priv(hw));
	struct rtl_efuse *rtlefuse = rtl_efuse(rtl_priv(hw));
	u8 *macaddr = p_macaddr;
	u32 entry_id = 0;
	bool is_pairwise = false;

	static u8 cam_const_addr[4][6] = {
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x02},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x03}
	};
	static u8 cam_const_broad[] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};

	if (clear_all) {
		u8 idx = 0;
		u8 cam_offset = 0;
		u8 clear_number = 5;

		rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG, "clear_all\n");

		for (idx = 0; idx < clear_number; idx++) {
			rtl_cam_mark_invalid(hw, cam_offset + idx);
			rtl_cam_empty_entry(hw, cam_offset + idx);

			if (idx < 5) {
				memset(rtlpriv->sec.key_buf[idx], 0,
				       MAX_KEY_LEN);
				rtlpriv->sec.key_len[idx] = 0;
			}
		}

	} else {
		switch (enc_algo) {
		case WEP40_ENCRYPTION:
			enc_algo = CAM_WEP40;
			break;
		case WEP104_ENCRYPTION:
			enc_algo = CAM_WEP104;
			break;
		case TKIP_ENCRYPTION:
			enc_algo = CAM_TKIP;
			break;
		case AESCCMP_ENCRYPTION:
			enc_algo = CAM_AES;
			break;
		default:
			rtl_dbg(rtlpriv, COMP_ERR, DBG_DMESG,
				"switch case %#x not processed\n", enc_algo);
			enc_algo = CAM_TKIP;
			break;
		}

		if (is_wepkey || rtlpriv->sec.use_defaultkey) {
			macaddr = cam_const_addr[key_index];
			entry_id = key_index;
		} else {
			if (is_group) {
				macaddr = cam_const_broad;
				entry_id = key_index;
			} else {
				if (mac->opmode == NL80211_IFTYPE_AP ||
				    mac->opmode == NL80211_IFTYPE_MESH_POINT) {
					entry_id = rtl_cam_get_free_entry(hw,
								     p_macaddr);
					if (entry_id >= TOTAL_CAM_ENTRY) {
						pr_err("Can not find free hw security cam entry\n");
						return;
					}
				} else {
					entry_id = CAM_PAIRWISE_KEY_POSITION;
				}

				key_index = PAIRWISE_KEYIDX;
				is_pairwise = true;
			}
		}

		if (rtlpriv->sec.key_len[key_index] == 0) {
			rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG,
				"delete one entry, entry_id is %d\n",
				entry_id);
			if (mac->opmode == NL80211_IFTYPE_AP ||
			    mac->opmode == NL80211_IFTYPE_MESH_POINT)
				rtl_cam_del_entry(hw, p_macaddr);
			rtl_cam_delete_one_entry(hw, p_macaddr, entry_id);
		} else {
			rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG,
				"add one entry\n");
			/* KEY spy: prove the PTK/GTK actually reach the HW CAM.
			 * A missing/mis-slotted GROUP key = downstream broadcast
			 * (e.g. a DHCP OFFER) can't be encrypted -> a client that
			 * finished the 4-way still never gets an IP. alg: 1=WEP40
			 * 2=TKIP 4=AES 5=WEP104. */
			pr_info("92f-spy KEY add entry=%u kidx=%u %s %pM alg=%u\n",
				entry_id, key_index,
				is_pairwise ? "PAIRWISE" : (is_group ? "GROUP" : "def"),
				macaddr, enc_algo);
			if (is_pairwise) {
				rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG,
					"set Pairwise key\n");

				rtl_cam_add_one_entry(hw, macaddr, key_index,
					       entry_id, enc_algo,
					       CAM_CONFIG_NO_USEDK,
					       rtlpriv->sec.key_buf[key_index]);
			} else {
				rtl_dbg(rtlpriv, COMP_SEC, DBG_DMESG,
					"set group key\n");

				if (mac->opmode == NL80211_IFTYPE_ADHOC) {
					rtl_cam_add_one_entry(hw,
						rtlefuse->dev_addr,
						PAIRWISE_KEYIDX,
						CAM_PAIRWISE_KEY_POSITION,
						enc_algo, CAM_CONFIG_NO_USEDK,
						rtlpriv->sec.key_buf[entry_id]);
				}

				rtl_cam_add_one_entry(hw, macaddr, key_index,
						entry_id, enc_algo,
						CAM_CONFIG_NO_USEDK,
						rtlpriv->sec.key_buf[entry_id]);
			}
		}
	}
}

void rtl92fe_read_bt_coexist_info_from_hwpg(struct ieee80211_hw *hw,
					    bool auto_load_fail, u8 *hwinfo)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	u8 value;

	if (!auto_load_fail) {
		value = hwinfo[EEPROM_RFE_OPTION_92F];
		if (((value & 0xe0) >> 5) == 0x1)
			rtlpriv->btcoexist.btc_info.btcoexist = 1;
		else
			rtlpriv->btcoexist.btc_info.btcoexist = 0;

		rtlpriv->btcoexist.btc_info.bt_type = BT_RTL8192E;
		rtlpriv->btcoexist.btc_info.ant_num = ANT_X2;
	} else {
		rtlpriv->btcoexist.btc_info.btcoexist = 1;
		rtlpriv->btcoexist.btc_info.bt_type = BT_RTL8192E;
		rtlpriv->btcoexist.btc_info.ant_num = ANT_X1;
	}
}

void rtl92fe_bt_reg_init(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	/* 0:Low, 1:High, 2:From Efuse. */
	rtlpriv->btcoexist.reg_bt_iso = 2;
	/* 0:Idle, 1:None-SCO, 2:SCO, 3:From Counter. */
	rtlpriv->btcoexist.reg_bt_sco = 3;
	/* 0:Disable BT control A-MPDU, 1:Enable BT control A-MPDU. */
	rtlpriv->btcoexist.reg_bt_sco = 0;
}

void rtl92fe_bt_hw_init(struct ieee80211_hw *hw)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);

	if (rtlpriv->cfg->ops->get_btc_status())
		rtlpriv->btcoexist.btc_ops->btc_init_hw_config(rtlpriv);
}

void rtl92fe_suspend(struct ieee80211_hw *hw)
{
}

void rtl92fe_resume(struct ieee80211_hw *hw)
{
}

/* Turn on AAP (RCR:bit 0) for promiscuous mode. */
void rtl92fe_allow_all_destaddr(struct ieee80211_hw *hw,
				bool allow_all_da, bool write_into_reg)
{
	struct rtl_priv *rtlpriv = rtl_priv(hw);
	struct rtl_pci *rtlpci = rtl_pcidev(rtl_pcipriv(hw));

	if (allow_all_da)	/* Set BIT0 */
		rtlpci->receive_config |= RCR_AAP;
	else			/* Clear BIT0 */
		rtlpci->receive_config &= ~RCR_AAP;

	if (write_into_reg)
		rtl_write_dword(rtlpriv, REG_RCR, rtlpci->receive_config);

	rtl_dbg(rtlpriv, COMP_TURBO | COMP_INIT, DBG_LOUD,
		"receive_config=0x%08X, write_into_reg=%d\n",
		rtlpci->receive_config, write_into_reg);
}
