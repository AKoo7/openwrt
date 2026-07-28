/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2026  Realtek Corporation
 */

#ifndef __RTW89_LED_H__
#define __RTW89_LED_H__

#ifdef CONFIG_RTW89_LEDS

void rtw89_led_init(struct rtw89_dev *rtwdev);
void rtw89_led_deinit(struct rtw89_dev *rtwdev);

/* Tell the LED the radio just came up or went down.  Called from the core
 * start/stop pair, which is what actually owns this LED -- see led.c.
 */
void rtw89_led_radio_state(struct rtw89_dev *rtwdev, bool on);

/* Shared LED-pad driver for every RTW89 part whose LED hangs off MAC GPIO 8.
 * Chips point struct rtw89_chip_ops::led_set at this; see led.c for why one
 * implementation covers both 8852C and 8192XB.
 */
void rtw89_led_set_gpio8(struct led_classdev *led,
			 enum led_brightness brightness);

#else

static inline void rtw89_led_init(struct rtw89_dev *rtwdev)
{
}

static inline void rtw89_led_deinit(struct rtw89_dev *rtwdev)
{
}

static inline void rtw89_led_radio_state(struct rtw89_dev *rtwdev, bool on)
{
}

#endif

#endif
