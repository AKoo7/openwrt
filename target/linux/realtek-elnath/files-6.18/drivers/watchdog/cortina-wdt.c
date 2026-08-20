// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cortina-Access peripheral watchdog (PER_WDT).
 *
 * Found on the Cortina-Access "Venus" family and on the Realtek parts derived
 * from it -- this driver is used on the RTL9607F "Elnath" GPON ONU, where the
 * block sits at 0xf432902c and the SoC reset glue it drives lives outside the
 * block, in GLOBAL_GLOBAL_CONFIG at 0xf43200c8.
 *
 * The hardware is a down-counter with a programmable prescaler:
 *
 *	PER_WDT_PS   scales the input clock to a 1 ms tick
 *	PER_WDT_DIV  divides that tick further (1000 -> one count per second)
 *	PER_WDT_LD   is the reload value, i.e. the timeout in counter units
 *	PER_WDT_LOADE reloads the counter from LD -- this is the "kick"
 *	PER_WDT_CTRL enables the counter and, with RSTEN, the SoC reset
 *
 * Reaching zero raises PER_WDT_INT_0 and, when RSTEN is set, asserts the SoC
 * reset DELAY_RESET/16 kHz clock cycles later.  The reset itself is gated by
 * bits in GLOBAL_GLOBAL_CONFIG which are NOT part of this block: without them
 * the counter expires and nothing happens.  That word also carries unrelated
 * power-down bits for the GPON/L3FE/crypto blocks, so it is only ever
 * read-modify-written.
 *
 * Register offsets, field semantics, the prescaler arithmetic and the reset
 * enables are hardware facts, cross-checked between the SoC's own register
 * name table and a live read of a running device.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

/* --- PER_WDT block, offsets from the block base (0xf432902c on RTL9607F) - */
#define PER_WDT_CTRL			0x00
#define  PER_WDT_CTRL_WDTEN		BIT(0)	/* counter runs		*/
#define  PER_WDT_CTRL_RSTEN		BIT(1)	/* expiry asserts reset	*/
#define  PER_WDT_CTRL_CLKSEL		BIT(2)	/* 0: DIV output, 1: 1 ms tick */
#define  PER_WDT_CTRL_DELAY		GENMASK(31, 12)
#define  PER_WDT_CTRL_DELAY_SHIFT	12
#define PER_WDT_PS			0x04	/* prescaler reload	*/
#define PER_WDT_DIV			0x08	/* prescaler divider	*/
#define PER_WDT_LD			0x0c	/* counter reload	*/
#define PER_WDT_LOADE			0x10
#define  PER_WDT_LOADE_WDT		BIT(0)	/* reload the counter	*/
#define  PER_WDT_LOADE_PRE		BIT(1)	/* reload the prescaler	*/
#define PER_WDT_CNT			0x14	/* live count		*/
#define PER_WDT_IE_0			0x18
#define  PER_WDT_IE_0_WDTE		BIT(0)
#define PER_WDT_INT_0			0x1c
#define  PER_WDT_INT_0_WDTI		BIT(0)
#define PER_WDT_STAT_0			0x20
#define PER_WDT_BLOCK_SIZE		0x30	/* CTRL..STAT_1, 12 registers */

/*
 * --- GLOBAL_GLOBAL_CONFIG, the SoC reset glue (a single 32-bit word) -------
 *
 * The five enables below are what turn an expired counter into an actual chip
 * reset.  Everything else in the word belongs to other subsystems -- including
 * power-down bits for the GPON, EPON, L3FE, crypto and core blocks -- which is
 * why this word is only ever READ-MODIFY-WRITTEN.  A blind store here would
 * enable the reset and power down half the device on its way past.
 *
 * ★ bit 8 and bit 9 are TWO DIFFERENT BITS -- settled 2026-08-10 against this
 * silicon's own register table, cross-checked on a sibling part.  An earlier
 * version of this comment said "EXT_RESET (bit 9) ... is deliberately left
 * alone" and read as if it contradicted the mask below, which includes BIT(8):
 *
 *	bit 8  wd_reset_ext_reset -- the WATCHDOG's external-reset ENABLE, one
 *	       of the wd_reset_* group, and correctly part of the mask;
 *	bit 9  ext_reset          -- the standalone external reset, which this
 *	       driver never touches, then or now.
 *
 * So the mask was right and the WORDING was wrong.  Both statements were true
 * of different bits, which is the most durable kind of wrong comment: nothing
 * ever fails to force the fix.
 *
 * MEASURED the same day, at the U-Boot prompt on a COLD boot, before this
 * driver has ever run: the word already reads 0x076455f0, which is the
 * documented reset default 0x07645420 plus exactly bits 4, 6, 7 and 8.  An
 * earlier boot stage therefore already enables the watchdog reset path, and
 * this driver's read-modify-write changes nothing at all on this board -- it
 * exists for a board whose loader does not set them.
 *
 * Note bit 5: it is already set at power-on here (it is part of that default),
 * and the vendor's own driver ORs only 4|6|7|8 on this chip.  Keeping it in the
 * mask is a no-op on this board either way; it is recorded rather than removed
 * because neither spelling can be told from the other by measurement here.
 */
#define GLOBAL_WD_RESET_SUBSYS_ENABLE	BIT(4)
#define GLOBAL_WD_RESET_PCIE		BIT(5)
#define GLOBAL_WD_RESET_ALL_BLOCKS	BIT(6)
#define GLOBAL_WD_RESET_REMAP		BIT(7)
#define GLOBAL_WD_RESET_EXT_RESET	BIT(8)
#define GLOBAL_WD_RESET_ENABLES		(GLOBAL_WD_RESET_SUBSYS_ENABLE | \
					 GLOBAL_WD_RESET_PCIE | \
					 GLOBAL_WD_RESET_ALL_BLOCKS | \
					 GLOBAL_WD_RESET_REMAP | \
					 GLOBAL_WD_RESET_EXT_RESET)

/*
 * The prescaler is programmed for a 1 ms tick, and PER_WDT_DIV then turns that
 * into one count per second.  Both are hardware constants of the block, not
 * tunables.
 */
#define PER_WDT_TICK_HZ			1000		/* the prescaled tick */
#define PER_WDT_DIV_PER_SECOND		1000		/* ticks per second   */

/* DELAY_RESET is expressed in units of a 16 kHz clock derived from the APB. */
#define PER_WDT_DELAY_HZ		16000

#define CORTINA_WDT_DEFAULT_TIMEOUT	60
#define CORTINA_WDT_MIN_TIMEOUT		1

/*
 * The counter is 32 bits wide and, in seconds mode, counts one per second, so
 * the register is not the limit -- the watchdog core is, because it does its
 * arithmetic in milliseconds.
 */
#define CORTINA_WDT_MAX_TIMEOUT		(UINT_MAX / 1000)

static unsigned int timeout;
module_param(timeout, uint, 0444);
MODULE_PARM_DESC(timeout,
		 "Watchdog timeout in seconds (default: device tree, else "
		 __MODULE_STRING(CORTINA_WDT_DEFAULT_TIMEOUT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default: "
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/**
 * struct cortina_wdt - one PER_WDT instance
 * @wdd:	the watchdog the core sees
 * @base:	PER_WDT register block
 * @rstcfg:	GLOBAL_GLOBAL_CONFIG, the reset glue outside the block
 * @clk:	APB clock feeding the prescaler
 * @prescaler:	PER_WDT_PS value giving a 1 ms tick from @clk
 * @delay:	PER_WDT_CTRL delay field, in 16 kHz cycles
 * @reset:	true when expiry must reset the SoC (reset-on-timeout)
 * @irq_armed:	true when we own the expiry interrupt
 * @lock:	serialises the read-modify-write of PER_WDT_CTRL
 */
struct cortina_wdt {
	struct watchdog_device	wdd;
	void __iomem		*base;
	void __iomem		*rstcfg;
	struct clk		*clk;
	u32			prescaler;
	u32			delay;
	bool			reset;
	bool			irq_armed;
	spinlock_t		lock;		/* see kernel-doc above */
};

static inline struct cortina_wdt *to_cortina_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct cortina_wdt, wdd);
}

/*
 * Load the counter and (re)start it.
 *
 * @count is in whole seconds unless @millisecond is set, in which case the
 * DIV stage is bypassed and the counter runs on the raw 1 ms tick -- that mode
 * exists for the restart handler, which wants the shortest reachable timeout.
 *
 * The closing PER_WDT_LOADE strobe is what makes a *running* watchdog adopt a
 * new PER_WDT_LD immediately; without it a shortened timeout would only take
 * effect at the next kick, which is precisely the window an unattended device
 * must not have.
 */
static void cortina_wdt_program(struct cortina_wdt *wdt, u32 count,
				bool millisecond, bool with_delay)
{
	u32 ctrl = PER_WDT_CTRL_WDTEN;

	if (wdt->reset)
		ctrl |= PER_WDT_CTRL_RSTEN;
	if (millisecond)
		ctrl |= PER_WDT_CTRL_CLKSEL;
	if (with_delay)
		ctrl |= (wdt->delay << PER_WDT_CTRL_DELAY_SHIFT) &
			PER_WDT_CTRL_DELAY;

	writel(wdt->prescaler, wdt->base + PER_WDT_PS);
	writel(millisecond ? 0 : PER_WDT_DIV_PER_SECOND, wdt->base + PER_WDT_DIV);
	writel(count, wdt->base + PER_WDT_LD);
	/*
	 * The expiry event is enabled even on a board that takes no interrupt.
	 * Whether PER_WDT_IE_0 also gates the path from "counter reached zero"
	 * to "RSTEN asserts the reset" is not documented anywhere we can check,
	 * and a watchdog that quietly fails to bite is the one outcome this
	 * driver may not have.  A firmware known to reset this SoC leaves the
	 * bit set, so we do too; with no handler registered the line is simply
	 * never unmasked at the interrupt controller.
	 */
	writel(PER_WDT_IE_0_WDTE, wdt->base + PER_WDT_IE_0);
	writel(ctrl, wdt->base + PER_WDT_CTRL);
	writel(PER_WDT_LOADE_WDT | PER_WDT_LOADE_PRE, wdt->base + PER_WDT_LOADE);
}

static int cortina_wdt_start(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	cortina_wdt_program(wdt, wdd->timeout, false, true);
	spin_unlock_irqrestore(&wdt->lock, flags);

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
}

static int cortina_wdt_stop(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);
	unsigned long flags;
	u32 ctrl;

	spin_lock_irqsave(&wdt->lock, flags);
	ctrl = readl(wdt->base + PER_WDT_CTRL);
	writel(ctrl & ~PER_WDT_CTRL_WDTEN, wdt->base + PER_WDT_CTRL);
	spin_unlock_irqrestore(&wdt->lock, flags);

	clear_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
}

static int cortina_wdt_ping(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);

	writel(PER_WDT_LOADE_WDT | PER_WDT_LOADE_PRE, wdt->base + PER_WDT_LOADE);

	return 0;
}

static int cortina_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int new_timeout)
{
	wdd->timeout = new_timeout;

	/*
	 * Reprogram only when the counter is already running.  Programming a
	 * stopped watchdog here would arm it behind the caller's back: the
	 * core allows WDIOC_SETTIMEOUT before WDIOC_SETOPTIONS/start.
	 */
	if (watchdog_hw_running(wdd))
		return cortina_wdt_start(wdd);

	return 0;
}

static unsigned int cortina_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);

	return readl(wdt->base + PER_WDT_CNT);
}

/*
 * Last-resort machine restart.
 *
 * Registered at the default (lowest) priority, so a firmware reset -- PSCI on
 * this SoC -- is always preferred.  This path only runs when everything above
 * it declined, which is exactly the situation an unattended device needs a way
 * out of.  It uses the millisecond tick and no reset delay to fire as fast as
 * the block can.
 */
static int cortina_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	struct cortina_wdt *wdt = to_cortina_wdt(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	cortina_wdt_program(wdt, 1, true, false);
	spin_unlock_irqrestore(&wdt->lock, flags);

	mdelay(1000);

	return 0;
}

/*
 * Only reached on a board that wires the expiry interrupt AND does not ask for
 * reset-on-timeout: with RSTEN set the SoC is already on its way down.  Ack the
 * event and mask it, so a board without a reset path reports the timeout once
 * instead of live-locking on a level-triggered line.
 */
static irqreturn_t cortina_wdt_isr(int irq, void *dev_id)
{
	struct cortina_wdt *wdt = dev_id;

	writel(PER_WDT_INT_0_WDTI, wdt->base + PER_WDT_INT_0);
	writel(0, wdt->base + PER_WDT_IE_0);

	dev_crit(wdt->wdd.parent,
		 "watchdog expired and this board has no reset-on-timeout\n");

	return IRQ_HANDLED;
}

static const struct watchdog_info cortina_wdt_info = {
	.identity	= "Cortina PER_WDT",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops cortina_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= cortina_wdt_start,
	.stop		= cortina_wdt_stop,
	.ping		= cortina_wdt_ping,
	.set_timeout	= cortina_wdt_set_timeout,
	.get_timeleft	= cortina_wdt_get_timeleft,
	.restart	= cortina_wdt_restart,
};

/*
 * Turn an expired counter into a chip reset.
 *
 * These bits are outside the watchdog's own register window, which is the
 * whole reason this is a separate step: a fully and correctly programmed
 * PER_WDT on a SoC whose glue is untouched counts down and does nothing.
 *
 * ⚠ OPEN, MEASURED 2026-08-10 -- OUR RESET DOES NOT LEAVE A RESET *REASON*.
 *
 * The vendor firmware exposes /proc/realtek/reboot_reason.  Established as a
 * real witness on STOCK before believing anything from it, which is the only
 * way to tell a finding from a phantom:
 *
 *	stock's OWN watchdog fired (its feeder SIGSTOPped)  -> reads 3
 *	stock rebooted cleanly                              -> reads 0
 *	OUR watchdog fired                                  -> reads 0  (twice)
 *
 * So the node DOES discriminate, and a reset we cause is not recorded as a
 * watchdog reset.  That matters in the field: it is how an operator tells "the
 * watchdog recovered a hang" from "the power went out", and without it the
 * recovery this driver provides is invisible exactly when it is needed.
 *
 * IT IS NOT A RESET-PATH DIFFERENCE, and both candidates are ruled out by
 * measurement so nobody re-chases them.  The reset ENABLES are identical: live
 * on stock GLOBAL_GLOBAL_CONFIG reads 0x076445f0 and at the U-Boot prompt
 * 0x076455f0 -- bits 4..8 set in both, before this driver runs, and the only
 * write below is an OR, which cannot clear them.  Nor is the PER_WDT
 * programming: stock live reads CTRL=0xff918003, PS=0x1e847, DIV=1000, i.e.
 * WDTEN|RSTEN with a delay field of 0xff918, and this driver computes that same
 * delay from the same device-tree value.  The one difference is LD (stock 60 s,
 * ours whatever userspace asks), which is a timeout, not a reset path.
 *
 * ★ THE MECHANISM, established from the vendor's own shipped code: the value is
 * a SOFTWARE BREADCRUMB, not a hardware reset-cause latch.  It lives in
 * GLOBAL_SOFTWARE (0xf43201c4) bits [3:0] -- named in the vendor's own
 * register table -- and the vendor's kernel writes it:
 *
 *	1  from a reboot notifier, i.e. an orderly reboot
 *	2  from a panic notifier
 *	3  when the watchdog is STARTED
 *
 * The node is sampled ONCE at kernel init and cached, and nothing in the
 * bootloader or userspace ever writes it, so a late read is not a timing
 * artefact.  Our kernel writes none of them, which is exactly why every reset
 * we cause reads 0.  That accounts for all four of our readings without any
 * appeal to the reset path.
 *
 * ⚠⚠ AND THE OBVIOUS "FIX" WOULD INSTALL A PHANTOM.  `3` means THE WATCHDOG WAS
 * ARMED, not "the watchdog fired" -- stock arms it on every boot, so 3 is its
 * steady state and it reads 3 after a power cut just the same.  Writing 3 from
 * .start and calling the result WDIOF_CARDRESET would publish a witness that is
 * true whenever the previous boot merely ENABLED a watchdog.  That is the
 * phantom-witness family this port has already paid for twice.
 *
 * ⚠ A PARTIAL adoption is WORSE THAN NONE, which is why this is left as a
 * scoped decision rather than a two-line patch: the vendor's own userspace
 * counts an "abnormal reboot" whenever the value exceeds 1, so a driver that
 * wrote 3 on arm WITHOUT also writing 1 on an orderly reboot would make every
 * clean reboot of our firmware look abnormal to the vendor's tooling.  A
 * correct adoption needs all three writers, and the reboot/panic ones are
 * SoC-level concerns that do not belong to a watchdog driver.
 *
 * ⇒ Recorded as a DECLARED difference: our resets leave no vendor-readable
 * reason.  Also measured: wdd.bootstatus is never set here, so
 * /sys/class/watchdog/watchdog0/bootstatus reads 0 on our image -- but stock
 * cannot report it either (its own watchdog advertises no WDIOF_CARDRESET and
 * never populates bootstatus), so this is a gap in BOTH, not a regression.  And
 * it could not be observed on the dev rig anyway: our image is TFTP->RAM, so a
 * watchdog reset lands the board on the NAND vendor image and our own
 * bootstatus is never read after a bite.  That needs a NAND install.
 */
static void cortina_wdt_enable_soc_reset(struct cortina_wdt *wdt)
{
	u32 val = readl(wdt->rstcfg);

	writel(val | GLOBAL_WD_RESET_ENABLES, wdt->rstcfg);
}

static int cortina_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_wdt *wdt;
	struct resource *res;
	resource_size_t phys;
	unsigned long rate;
	u32 ctrl, adopted_s = 0;
	bool adopted;
	int irq, ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	spin_lock_init(&wdt->lock);

	/*
	 * devm_ioremap(), not devm_ioremap_resource(): both windows live inside
	 * peripheral blocks that other drivers on this SoC also map, and
	 * claiming them exclusively would make whichever probes second fail.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing the PER_WDT register window\n");
	if (resource_size(res) < PER_WDT_BLOCK_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "PER_WDT window is %pa, need at least %#x\n",
				     &res->start, PER_WDT_BLOCK_SIZE);
	phys = res->start;
	wdt->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!wdt->base)
		return -ENOMEM;

	wdt->reset = of_property_read_bool(dev->of_node, "reset-on-timeout");
	if (wdt->reset) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res)
			return dev_err_probe(dev, -EINVAL,
					     "reset-on-timeout needs the SoC reset-config window\n");
		wdt->rstcfg = devm_ioremap(dev, res->start, resource_size(res));
		if (!wdt->rstcfg)
			return -ENOMEM;
	}

	wdt->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(wdt->clk))
		return dev_err_probe(dev, PTR_ERR(wdt->clk),
				     "no input clock\n");

	rate = clk_get_rate(wdt->clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "input clock reports 0 Hz\n");
	wdt->prescaler = rate / PER_WDT_TICK_HZ - 1;

	/*
	 * Optional: how long the block waits between raising the expiry event
	 * and asserting the reset.  Expressed in the device tree in 16 kHz
	 * cycles, which is also how the vendor firmware programs it.
	 */
	if (!of_property_read_u32(dev->of_node, "delay-reset", &wdt->delay)) {
		u64 cycles = (u64)wdt->delay * (rate / PER_WDT_DELAY_HZ);

		wdt->delay = min_t(u64, cycles,
				   PER_WDT_CTRL_DELAY >> PER_WDT_CTRL_DELAY_SHIFT);
	}

	/*
	 * The interrupt is optional and only matters without reset-on-timeout.
	 * A board whose interrupt controller is not described simply does not
	 * get one; the reset is asserted by the hardware either way.
	 */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, cortina_wdt_isr, 0,
				       dev_name(dev), wdt);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot take the expiry interrupt\n");
		wdt->irq_armed = true;
	}

	wdt->wdd.info = &cortina_wdt_info;
	wdt->wdd.ops = &cortina_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = CORTINA_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = CORTINA_WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = CORTINA_WDT_DEFAULT_TIMEOUT;

	/*
	 * ★ Was it already counting when Linux took over?
	 *
	 * A bootloader that arms the watchdog and hands over to a kernel that
	 * ignores it produces a board which reboots mid-boot, forever, with
	 * nothing to see.  So the state is READ rather than assumed and
	 * reported on every boot.
	 */
	ctrl = readl(wdt->base + PER_WDT_CTRL);
	adopted = ctrl & PER_WDT_CTRL_WDTEN;
	if (adopted) {
		u32 load = readl(wdt->base + PER_WDT_LD);

		adopted_s = (ctrl & PER_WDT_CTRL_CLKSEL) ? load / PER_WDT_TICK_HZ
							 : load;
	}

	/*
	 * watchdog_init_timeout() reports failure ONLY when a value it was
	 * handed is out of range; with no module parameter and no timeout-sec
	 * property it succeeds and leaves our default standing.  It therefore
	 * says nothing about an inherited window, and must not be used to
	 * decide anything about one -- an earlier version of this driver keyed
	 * the adopt path on this return value, so that path could only ever
	 * have run on a board that was already misconfigured.
	 */
	ret = watchdog_init_timeout(&wdt->wdd, timeout, dev);
	if (ret)
		dev_warn(dev, "timeout out of range, keeping %u s\n",
			 wdt->wdd.timeout);

	/*
	 * Deliberately NO watchdog_stop_on_reboot().
	 *
	 * The watchdog stays armed for the whole of an orderly reboot, which
	 * covers the one case nobody is in the building for: a device that
	 * wedges while shutting down.  Once userspace stops being scheduled
	 * nothing kicks the counter, so a shutdown that never completes is
	 * reset within one window instead of hanging until a human cuts power.
	 *
	 * ★ MEASURED on this board (2026-08-10), because the alternative
	 * -- a counter that keeps running into the next boot -- would be far
	 * worse than no watchdog at all: it would reboot-loop the unit.
	 *
	 * Taken with the watchdog ARMED and being fed (a 30 s window), then a
	 * plain `reboot` == PSCI SYSTEM_RESET, with NO power event anywhere in
	 * the sequence.  Read at the U-Boot prompt ~8 s later, over three warm
	 * reboots plus a cold control:
	 *
	 *   warm (PSCI reset) CTRL=0x00000000  DIV=1000  LD=23  CNT static
	 *   cold (power cut)  CTRL=0x00000000  DIV=1000  LD=23  CNT static
	 *
	 * CTRL=0 is WDTEN clear, and the count does not move between two reads
	 * 4.3 s apart, so nothing is counting when the bootloader hands over.
	 * This driver's probe then reported "stopped at kernel entry" on all
	 * four boots.
	 *
	 * ★ AND THE MECHANISM IS NOT "the SoC reset cleared our block", which
	 * is what the raw reading first looks like and what an earlier draft of
	 * this comment claimed.  LD reads 23 and OUR window was 30, so between
	 * the reset and the prompt an earlier boot stage REPROGRAMMED the block
	 * with a 23 s window of its own -- on the warm path exactly as on the
	 * cold one.  The vendor's own loader then stops it, gated on
	 * recognising precisely that signature (DIV == 1000 and LD == 23) late
	 * in its init, before the kernel image is loaded.  Our countdown does
	 * not reach the next boot because it is OVERWRITTEN, which is a
	 * different and weaker claim than the reset clearing the block -- and
	 * the one the measurement actually supports.
	 *
	 * ⚠ So the loader's stop is CONDITIONAL on its own signature and is NOT
	 * a general safety net for an arbitrary window left running by us; it
	 * is the earlier stage's unconditional re-arming that makes ours
	 * harmless here.  Re-measure this if the boot chain is ever replaced.
	 *
	 * Coverage is therefore: probe -> the reset instant, on every boot,
	 * including the reboot path itself, which is the point.
	 *
	 * The alternative designs, and why not: watchdog_stop_on_reboot() would
	 * give up the shutdown-hang coverage that is the entire reason to stay
	 * armed, to remove a hazard measured not to exist; re-arming long in a
	 * reboot notifier would keep both but adds a moving part for the same
	 * non-existent hazard, and slows real recovery to that longer window.
	 * The vendor's driver makes the same call as this one -- its reboot
	 * notifier and .shutdown are both compiled out.
	 */
	watchdog_set_nowayout(&wdt->wdd, nowayout);

	if (wdt->reset)
		cortina_wdt_enable_soc_reset(wdt);

	if (adopted) {
		/*
		 * Take the window OVER rather than inherit it.  Two reasons,
		 * and neither is cosmetic:
		 *
		 *  - the counter handed to us is part-way through a countdown
		 *    nobody recorded.  It may have a fraction of its window
		 *    left, and running with that would bite during our own
		 *    boot for no reason;
		 *  - the watchdog core derives its keepalive interval from
		 *    wdd->timeout, NOT from what the hardware is carrying, so
		 *    an inherited window shorter than wdd->timeout would be
		 *    fed too slowly to survive.
		 *
		 * Programming our own timeout and reloading settles both, and
		 * makes WDOG_HW_RUNNING an honest description of the hardware.
		 * The inherited window is still reported below as evidence.
		 */
		cortina_wdt_start(&wdt->wdd);
	}

	platform_set_drvdata(pdev, wdt);

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;

	if (adopted)
		dev_info(dev,
			 "PER_WDT at %pa: %u s timeout, reset-on-timeout %s, expiry irq %s, ALREADY RUNNING at kernel entry (inherited %u s window, reprogrammed to %u s)%s\n",
			 &phys, wdt->wdd.timeout, wdt->reset ? "on" : "off",
			 wdt->irq_armed ? "taken" : "not wired",
			 adopted_s, wdt->wdd.timeout,
			 nowayout ? ", nowayout" : "");
	else
		dev_info(dev,
			 "PER_WDT at %pa: %u s timeout, reset-on-timeout %s, expiry irq %s, stopped at kernel entry%s\n",
			 &phys, wdt->wdd.timeout, wdt->reset ? "on" : "off",
			 wdt->irq_armed ? "taken" : "not wired",
			 nowayout ? ", nowayout" : "");

	return 0;
}

static const struct of_device_id cortina_wdt_of_match[] = {
	{ .compatible = "cortina,wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_wdt_of_match);

static struct platform_driver cortina_wdt_driver = {
	.probe	= cortina_wdt_probe,
	.driver	= {
		.name		= "cortina-wdt",
		.of_match_table	= cortina_wdt_of_match,
	},
};
module_platform_driver(cortina_wdt_driver);

MODULE_DESCRIPTION("Cortina-Access peripheral watchdog");
MODULE_LICENSE("GPL");
