// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * the per-RJ45 front-panel link lamps.
 *
 * WHY THIS IS NOT THE netdev TRIGGER
 * ----------------------------------
 * Each RJ45 on the front panel has a green link lamp.  The obvious way to drive
 * one is the in-kernel `netdev` trigger, and that is exactly what the vendor
 * firmware does - but it CANNOT express what is wanted here.  That trigger
 * reports the state of a net_device, and this clean-room driver registers ONE
 * net_device (eth0) for the whole switch rather than a subdevice per port.
 * Bound to eth0 all four lamps would show the same thing - the CPU port - and
 * would light together the instant any single port came up.  (The vendor
 * creates per-port HW-VLAN netdevs eth0.2/.3/.4/.5, so it has something per
 * RJ45 to bind to; we do not.  If per-port netdevs ever land, this file becomes
 * unnecessary: use the netdev trigger with mode "link tx rx" and delete it.)
 *
 * So the per-port state has to come from the driver, which already knows it:
 * the 1 Hz recovery poll in cortina-ni-rx.c reads every port's PHY link out of
 * BMSR to choose the CPU->LAN egress port set, and hands the resulting bitmap
 * here.  There is no second timer, no second MDIO read and no new state - this
 * file is that bitmap, published as LEDs.
 *
 * Stated plainly, because it is a difference from the vendor: these show LINK
 * only.  Stock's netdev trigger did link+tx+rx, so its lamps also blinked on
 * traffic.
 *
 * WHERE THE PANEL MAP LIVES, AND WHY IT IS NOT IN THIS FILE
 * --------------------------------------------------------
 * WHICH lamp a given switch port lights is a property of the BOARD, not of the
 * silicon.  This board's silkscreen is MIRRORED - printed LAN1 is switch port 3
 * - and the next board on this SoC will have its own answer.  A per-board table
 * compiled in here would have to be edited for every such board, and would put
 * a board fact in family code where nobody looks for it.
 *
 * So this driver publishes one LED TRIGGER PER SWITCH PORT, named by the port
 * INDEX and by nothing else, and the DEVICE TREE says which lamp binds to
 * which trigger:
 *
 *	led-lan1 {
 *		label = "x400axf:green:lan1";
 *		gpios = <&gpio3 0x07 0x00>;
 *		linux,default-trigger = "cortina-ni-port3-link";
 *	};
 *
 * A differently-wired board changes those four strings and not one line of
 * code.  The mapping is DATA, and it sits next to the label it belongs with,
 * where a reviewer sees both at once.
 *
 * WHY A TRIGGER AND NOT of_led_get() ON THE LED NODES
 * --------------------------------------------------
 * Resolving an LED by phandle from the consumer is the other way to do this,
 * and it was deliberately not used: of_led_get() returns -EPROBE_DEFER until
 * the LED provider has probed, and THIS driver's probe brings up the whole
 * datapath - the Ethernet switch, and the GPON stack that sits behind it.
 * Making that probe's ordering depend on leds-gpio, in order to light a lamp,
 * is a real risk taken for no benefit.
 *
 * A trigger has no ordering requirement in EITHER direction:
 * led_trigger_register() walks the existing LEDs and binds any whose
 * linux,default-trigger names it, and led_classdev_register() binds to any
 * trigger already registered.  So whichever of the two drivers probes first,
 * the binding happens.  If this driver never loads the lamps simply stay at
 * their device-tree default-state; if leds-gpio never loads the trigger events
 * fall on an empty list.  Neither outcome can disturb the datapath.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/types.h>

#include "cortina-ni.h"
#include "cortina-ni-regs.h"

#if IS_ENABLED(CONFIG_LEDS_TRIGGERS)

/*
 * One trigger per switch port.  The name carries the port INDEX and no panel
 * label: which socket port N lights is the device tree's business (above).
 *
 * ★ The array is deliberately declared WITHOUT a dimension, so ARRAY_SIZE() is
 * the number of names actually written here and the assert below is a REAL
 * check - raise CA_NI_LAN_PORT_COUNT without adding a name and the BUILD fails.
 * Declared as [CA_NI_LAN_PORT_COUNT] it would silently pad with NULL instead,
 * and led_trigger_register() strcmp()s the name of every trigger it is handed.
 */
static const char *const ca_ni_link_trig_name[] = {
	"cortina-ni-port0-link",
	"cortina-ni-port1-link",
	"cortina-ni-port2-link",
	"cortina-ni-port3-link",
};

static_assert(ARRAY_SIZE(ca_ni_link_trig_name) == CA_NI_LAN_PORT_COUNT,
	      "one link trigger name per LAN port");

static struct led_trigger ca_ni_link_trig[CA_NI_LAN_PORT_COUNT];

/*
 * Bit p set = trigger p is REGISTERED and may be fired.
 *
 * ★ Load-bearing, not defensive clutter.  led_trigger_event() walks
 * trig->led_cdevs with list_for_each_entry_rcu(), and that list head is
 * initialised by led_trigger_register().  devm_led_trigger_register() can fail
 * BEFORE reaching it - its devres_alloc() runs first - so a trigger whose
 * registration failed still has a zeroed list head, and walking one
 * dereferences NULL.  Fire only what actually registered.
 */
static u32 ca_ni_link_trig_live;

/*
 * Publish the per-port link triggers.  Called from probe; devm-managed, so
 * there is nothing to undo and this driver needs no .remove.
 *
 * A trigger that fails to register is NOT fatal to anything - no part of the
 * datapath depends on a lamp - so it is warned about and its port is left out
 * of the live mask.
 */
void cortina_ni_leds_probe(struct cortina_ni *ni)
{
	unsigned int p;

	/*
	 * These triggers are module-global because there is exactly one NI
	 * switch per SoC (a single "cortina,ni-interface" node), so they belong
	 * to one device.  That also makes a re-probe after an unbind a genuine
	 * re-registration: devm has already unregistered the previous set by the
	 * time we are called again, and the mask has to follow.
	 */
	ca_ni_link_trig_live = 0;

	for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
		int ret;

		ca_ni_link_trig[p].name = ca_ni_link_trig_name[p];
		ret = devm_led_trigger_register(ni->dev, &ca_ni_link_trig[p]);
		if (ret) {
			dev_warn(ni->dev,
				 "LAN link LED trigger %s not registered (%d) - that port's lamp will not track link\n",
				 ca_ni_link_trig_name[p], ret);
			continue;
		}
		ca_ni_link_trig_live |= BIT(p);
	}
}

/*
 * One tick of the 1 Hz poll: @link is the per-port PHY-link bitmap that
 * cortina-ni-rx.c has just read out of BMSR, bit p = port p is up.
 *
 * Fired EVERY tick rather than only on a change, for two reasons.  An LED that
 * binds to its trigger after this driver probed - leds-gpio is a separate
 * driver and may come up later - has missed every event so far and would sit
 * dark until the cable next moved.  And these lamps share GPIO group 3 with the
 * BOSA/laser nets the GPON driver owns, which re-applies stock's whole-group
 * snapshot on each (re-)activation and momentarily forces their levels; an
 * unconditional re-assert puts them back within one second.  The cost is four
 * MMIO writes per second.
 *
 * ★ IT CANNOT SLEEP, BLOCK OR FAIL, which is what makes it safe to call from
 * that poll at all.  led_trigger_event() returns void and tolerates a NULL or
 * empty trigger; for these GPIO lamps it reaches gpiod_set_value() through the
 * non-sleeping brightness_set op, which leds-gpio installs when
 * gpiod_cansleep() is false - true for this memory-mapped GPIO block.  It takes
 * no lock this driver holds, in particular not the MDIO mutex, so it cannot
 * deadlock against the PHY reads that produced @link.
 */
void cortina_ni_leds_link_set(u32 link)
{
	unsigned int p;

	for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
		if (!(ca_ni_link_trig_live & BIT(p)))
			continue;
		/*
		 * LED_ON is 1, which is exactly gpio-leds' max_brightness, so
		 * /sys/class/leds/<lamp>/brightness reads back a plain 1 or 0.
		 */
		led_trigger_event(&ca_ni_link_trig[p],
				  (link & BIT(p)) ? LED_ON : LED_OFF);
	}
}

#else	/* !CONFIG_LEDS_TRIGGERS */

/*
 * ★ A LAMP MAY NOT BREAK THE DATAPATH AT BUILD TIME EITHER, which is the same
 * rule that keeps _link_set() out of harm's way at run time.
 *
 * With the LED triggers configured out, <linux/leds.h> reduces struct
 * led_trigger to an EMPTY struct and provides no devm_led_trigger_register()
 * at all - so the code above would not compile, and an Ethernet driver would
 * stop building because of an indicator.  This target ships
 * CONFIG_LEDS_TRIGGERS=y, but this tree deliberately trims a lean kernel per
 * model, so that config is a thing someone will one day switch off.
 *
 * Degrade to "the lamps are not driven", say so once, and leave every other
 * part of the driver untouched.
 */
void cortina_ni_leds_probe(struct cortina_ni *ni)
{
	dev_info(ni->dev,
		 "built without CONFIG_LEDS_TRIGGERS - the per-RJ45 link lamps will not be driven\n");
}

void cortina_ni_leds_link_set(u32 link)
{
}

#endif	/* CONFIG_LEDS_TRIGGERS */
