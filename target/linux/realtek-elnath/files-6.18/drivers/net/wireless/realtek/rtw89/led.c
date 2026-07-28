// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2026  Realtek Corporation
 */

/* LED support for RTW89 parts whose front-panel LED is wired to the WiFi MAC's
 * own LED pad.
 *
 * How the pad is driven
 * ---------------------
 * The MAC has two ways to light its LED pad:
 *
 *  a) a hardware blink engine behind R_AX_LED_CFG (0x004C): a 3-bit control-mode
 *     field per LED block (values 2 = TRX, 4 = TX, 6 = RX) that makes the pad
 *     blink on traffic without software help.  It also needs the pad muxed to
 *     the dedicated "LED" pad function.  Nothing here uses it -- see below.
 *
 *  b) plain software GPIO: the LED pad doubles as MAC GPIO 8, so muxing that pad
 *     to its software-GPIO function and driving the GPIO output bit gives direct
 *     on/off control.
 *
 * We use (b), for two reasons.  It is the path the shipping firmware actually
 * takes -- it selects the software-GPIO control mode at init and never programs
 * 0x004C at all -- and it is the only one that maps onto what a Linux
 * led_classdev asks of a driver, namely "set this exact brightness now".
 *
 * What decides when it is lit is the driver itself, from the core start/stop
 * pair: steady on while the radio is up, dark when it is down, which is what the
 * shipping firmware's panel shows.  Not a mac80211 LED trigger -- see the long
 * note on rtw89_led_radio_state() for why that was tried and does not hold.  The
 * throughput trigger is still registered for anyone who prefers activity blink,
 * it just is not the default.
 *
 * The register facts, and how they are known
 * -----------------------------------------
 * Recovered from the shipping vendor WiFi module for this SoC by disassembly
 * (its LED entry points reduce to: LED id 0 -> MAC GPIO 8 -> a masked
 * read-modify-write of the GPIO byte registers), and independently corroborated
 * by mainline rtw89's own field names for the very same registers, which agree
 * on the byte layout of R_AX_GPIO_EXT_CTRL:
 *
 *   R_AX_GPIO8_15_FUNC_SEL (0x02D4)  one 4-bit pad-function nibble per GPIO,
 *                                    GPIO 8 in bits 3:0; 0xF selects software
 *                                    GPIO.  (reg.h already documents GPIO 9's
 *                                    nibble at bits 7:4, and rtw89's rfkill
 *                                    tables already use 0xF for that pad.)
 *   R_AX_GPIO_EXT_CTRL (0x0060)      four byte-wide fields covering GPIO 8..15,
 *                                    which reg.h names:
 *                                      bits 31:24 GPIO_MOD_15_TO_8
 *                                      bits 23:16 GPIO_IO_SEL_15_TO_8
 *                                      bits 15:8  GPIO_OUT_15_TO_8
 *                                      bits  7:0  GPIO_IN_15_TO_8  (read-only)
 *                                    GPIO 8 is bit 0 of each field.  Setting
 *                                    both MOD and IO_SEL makes the pad a
 *                                    push-pull output; clearing both makes it an
 *                                    input (which is exactly how the existing
 *                                    rfkill tables configure GPIO 9).
 *
 * Polarity is ACTIVE LOW: the pad is pulled low to light the LED.  That is not
 * an assumption -- the vendor module inverts the userspace "on" request into a
 * "drive the pad low" action, lights the LED on netdev-open, releases it high on
 * netdev-close, and leaves it high (dark) at init.
 *
 * All of the above was then confirmed on the running board against the shipping
 * firmware, which is why it is stated as fact: with a radio up, that firmware
 * rests with the pad nibble at 0xF, MOD and IO_SEL set, and the output bit clear;
 * forcing the output bit to 1 puts the panel LED out and clearing it lights it,
 * and the register's read-only input byte follows the pin, so the pad really is
 * switching rather than just latching.  Each radio lights its own indicator with
 * no cross-mapping: the 2.4 GHz-only part drives the 2.4G lamp and the 5/6 GHz
 * part drives the 5G one, which is what rtw89_led_label() below encodes.
 *
 * Both radios share this one implementation.  In the vendor module every LED
 * entry point is a single chip-generic function and the per-GPIO pad-function
 * tables for the 5 GHz and 2.4 GHz parts are byte-identical, so there is no
 * per-chip divergence to model.  The chip_ops indirection is kept anyway, so
 * that only parts whose LED wiring has actually been established opt in.
 *
 * Note there is no separate "set the control mode" step to forget: for the
 * software-GPIO path the mode *is* the pad mux plus the output-enable, and both
 * are re-applied on every brightness change.  That also makes the sequence
 * idempotent, so the LED survives a chip power cycle re-muxing the pad.
 */

#include "core.h"
#include "debug.h"
#include "led.h"
#include "reg.h"

/* GPIO 8's slice of the registers described above -- it is bit 0 of each of the
 * four byte fields.  reg.h carries the whole-field masks and GPIO 9's individual
 * bits; these are GPIO 8's counterparts.
 */
#define B_AX_PINMUX_GPIO8_FUNC_SEL_MASK	GENMASK(3, 0)
#define PINMUX_GPIO_FUNC_SEL_SW_GPIO	0xf
#define B_AX_GPIO_MOD_8			BIT(24)
#define B_AX_GPIO_IO_SEL_8		BIT(16)
#define B_AX_GPIO_OUT_8			BIT(8)

void rtw89_led_set_gpio8(struct led_classdev *led,
			 enum led_brightness brightness)
{
	struct rtw89_dev *rtwdev = container_of(led, struct rtw89_dev, led_cdev);

	/* Route the pad away from whatever function it booted with. */
	rtw89_write8_mask(rtwdev, R_AX_GPIO8_15_FUNC_SEL,
			  B_AX_PINMUX_GPIO8_FUNC_SEL_MASK,
			  PINMUX_GPIO_FUNC_SEL_SW_GPIO);

	/* Make it a push-pull output.  MOD and IO_SEL live in the upper half of
	 * the dword, addressed here as a 16-bit window the same way the rfkill
	 * tables address them.
	 */
	rtw89_write16_set(rtwdev, R_AX_GPIO_EXT_CTRL + 2,
			  (B_AX_GPIO_MOD_8 | B_AX_GPIO_IO_SEL_8) >> 16);

	/* Drive the level.  Active low: pad low lights the LED. */
	if (brightness == LED_OFF)
		rtw89_write8_set(rtwdev, R_AX_GPIO_EXT_CTRL + 1,
				 B_AX_GPIO_OUT_8 >> 8);
	else
		rtw89_write8_clr(rtwdev, R_AX_GPIO_EXT_CTRL + 1,
				 B_AX_GPIO_OUT_8 >> 8);
}

/* The one place brightness changes enter the driver, whatever asked for them --
 * a trigger, sysfs, or our own deinit.
 *
 * ★ This is deliberately the SYNCHRONOUS setter (brightness_set, never
 * brightness_set_blocking).  The radio-down write is issued from rtw89_core_stop()
 * while the chip is still powered; deferring it to a workqueue would let it run
 * after the power-down, where the guard below drops it and the LED stays lit for
 * as long as the radio is down.  Being synchronous also means this can be reached
 * from atomic context (led_trigger_event() holds the RCU read lock, for whoever
 * binds a trigger by hand), so it must not sleep -- hence no locking here, only
 * plain MMIO, which is all the register accessors below do.
 */
static void rtw89_led_set(struct led_classdev *led,
			  enum led_brightness brightness)
{
	struct rtw89_dev *rtwdev = container_of(led, struct rtw89_dev, led_cdev);

	/* With the chip powered down the pad is not driven by the MAC at all, so
	 * the LED is dark no matter what was asked for.  Say so, instead of
	 * leaving brightness claiming a value the pin does not have: the caller
	 * has already stored the requested value, so correct it here rather than
	 * just returning.
	 */
	if (!test_bit(RTW89_FLAG_POWERON, rtwdev->flags)) {
		led->brightness = LED_OFF;
		return;
	}

	rtwdev->chip->ops->led_set(led, brightness);
}

/* The radio came up or went down.  This driver, not a trigger, is the authority
 * for this LED.
 *
 * ★ WHY NOT mac80211's "<phy>radio" trigger, which looks purpose-built for this:
 * that trigger only emits on a radio on/off TRANSITION, and binding it does not
 * sample the current state (its activate callback just counts users).  A LED on
 * it therefore has no defined value -- miss or duplicate one event, or bind after
 * the radio is already up, and it latches at whatever it last held with no path
 * back.  Observed on this board: after a client associated and roamed away and
 * the AP interfaces bounced, both LEDs sat dark with both radios up and passing
 * traffic, and re-binding the trigger did not resync them.  So this is called
 * instead from rtw89_core_start() and rtw89_core_stop(), which every path that
 * brings the radio up or down goes through, and which is also what the vendor
 * firmware does (it drives the pad in its netdev open/close paths, no trigger).
 *
 * Goes through the LED class rather than straight at the pad so that the
 * brightness attribute always reports what is really on the pin.
 */
void rtw89_led_radio_state(struct rtw89_dev *rtwdev, bool on)
{
	if (!rtwdev->led_registered)
		return;

	led_set_brightness(&rtwdev->led_cdev, on ? LED_ON : LED_OFF);
}

/* Which panel indicator this radio owns.  The 2.4 GHz-only part drives the
 * "2.4G" LED; a part that also reaches 5/6 GHz is the board's 5 GHz radio and
 * drives the "5G" one.
 */
static const char *rtw89_led_label(struct rtw89_dev *rtwdev)
{
	u8 bands = rtwdev->chip->support_bands;

	if (bands & (BIT(NL80211_BAND_5GHZ) | BIT(NL80211_BAND_6GHZ)))
		return "green:5g";

	return "green:2.4g";
}

void rtw89_led_init(struct rtw89_dev *rtwdev)
{
	/* Blink faster the busier the radio is.  Ordered by throughput, in
	 * kilobits per second, as the trigger requires.  Not the default -- see
	 * below.
	 */
	static const struct ieee80211_tpt_blink rtw89_tpt_blink[] = {
		{ .throughput =   0 * 1024, .blink_time = 334 },
		{ .throughput =   1 * 1024, .blink_time = 260 },
		{ .throughput =   5 * 1024, .blink_time = 220 },
		{ .throughput =  20 * 1024, .blink_time = 170 },
		{ .throughput =  70 * 1024, .blink_time = 130 },
		{ .throughput = 200 * 1024, .blink_time =  80 },
		{ .throughput = 300 * 1024, .blink_time =  50 },
	};
	struct led_classdev *led = &rtwdev->led_cdev;
	struct led_init_data init_data = {};
	int err;

	if (!rtwdev->chip->ops->led_set)
		return;

	led->brightness_set = rtw89_led_set;
	led->max_brightness = LED_ON;

	/* Offer the throughput trigger as an option, so "blink on traffic" can be
	 * selected at runtime by writing <phy>tpt to the LED's trigger file.  Merely
	 * creating it here is enough to register it; mac80211 does that during
	 * ieee80211_register_hw(), which is why this must run before that call.
	 *
	 * ★ Deliberately NOT installed as default_trigger, and neither is the radio
	 * trigger -- the LED ships with NO trigger so that rtw89_led_radio_state()
	 * above is the single authority and the LED always has a defined state.
	 * Binding any trigger by hand hands that authority over, which is a choice
	 * the operator can make but not the shipped default.
	 */
	ieee80211_create_tpt_led_trigger(rtwdev->hw,
					 IEEE80211_TPT_LEDTRIG_FL_RADIO,
					 rtw89_tpt_blink,
					 ARRAY_SIZE(rtw89_tpt_blink));

	/* Name it "<prefix>:<colour>:<function>" so it matches the rest of the
	 * board's panel LEDs.  Deliberately no fwnode: these are PCI devices
	 * whose firmware node, where one exists at all, describes the radio and
	 * not an LED, and the LED core would otherwise harvest max-brightness,
	 * color and linux,default-trigger out of it.
	 */
	init_data.devicename = CONFIG_RTW89_LED_NAME_PREFIX;
	init_data.default_label = rtw89_led_label(rtwdev);

	err = led_classdev_register_ext(rtwdev->dev, led, &init_data);
	if (err) {
		rtw89_warn(rtwdev, "failed to register LED, error %d\n", err);
		return;
	}

	rtwdev->led_registered = true;
}

void rtw89_led_deinit(struct rtw89_dev *rtwdev)
{
	struct led_classdev *led = &rtwdev->led_cdev;

	if (!rtwdev->led_registered)
		return;

	/* Leave the panel dark on the way out, and go through the same setter as
	 * everything else so the chip-down guard applies here too: on the remove
	 * path the chip may already be powered off, and this must not poke it.
	 */
	rtw89_led_set(led, LED_OFF);
	led_classdev_unregister(led);

	rtwdev->led_registered = false;
}
