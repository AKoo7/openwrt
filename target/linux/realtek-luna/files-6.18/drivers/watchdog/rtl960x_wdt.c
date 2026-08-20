// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek "Luna" SoC watchdog timer (RTL960x family).
 *
 * The block is three 32-bit registers sitting at the top of the SoC timer/
 * counter (TC) block, at TC_BASE + 0x60 -- physically 0x18003260 on every Luna
 * part looked at so far, including the RTL9602C "Europa" this target ships on.
 * It is a TWO-PHASE watchdog:
 *
 *	phase 1 expires  -> the block raises its own timeout interrupt, so a
 *			    vendor kernel can dump state while still running;
 *	phase 2 expires  -> the SoC is RESET, in the mode selected by
 *			    WDT_RESET_MODE.
 *
 * Both phases restart together when the counter is kicked, so the time from the
 * last kick to the reset is (PH1 + 1 + PH2 + 1) ticks, and one tick is
 * 2^(25 + WDT_CLK_SC) cycles of the LX peripheral bus clock.  At the 200 MHz LX
 * clock this SoC runs, the coarsest scale gives a 1.342 s tick and a 42 s
 * ceiling; the finest gives 168 ms.
 *
 * ---------------------------------------------------------------------------
 * WHERE THESE FACTS COME FROM, and how far each one is actually proven.  The
 * question "does this silicon even HAVE a watchdog" was open when this driver
 * was written, so the answer is recorded rather than assumed.
 *
 *  [1] LIVE, ON THIS SoC.  arch/mips/realtek-luna/setup.c has driven exactly
 *      these registers since bring-up: luna_machine_restart() writes WDT_CTRL
 *      at 0x18003268 with enable | scale 0 | PH1 = 1 and does NOT kick, and
 *      that is how this port reboots.  It works, which proves the block exists
 *      at that address, that WDT_E arms it, that the timeout really resets the
 *      chip, and that RESET_MODE 0 re-enters the boot ROM.
 *  [2] THIS BOARD'S OWN STOCK FIRMWARE.  The X111W vendor kernel image contains
 *      the string "arch/mips/bsp_rtl8686/luna_watchdog.c" together with
 *      "WDT_E=%d", "WDT_CLK_SC=%d", "PH1_TO=%d", "PH2_TO=%d",
 *      "WDT_RESET_MODE=%d" and "[BSP_WDT_PH1TO_IRQ #%d]", and its rootfs arms
 *      the thing on every boot -- /etc/init.d/rc34 does
 *      `echo 1 > /proc/luna_watchdog/watchdog_flag`.  The shipped product runs
 *      with this watchdog enabled.
 *  [3] THE FAMILY'S OWN BSP HEADERS.  TC_BASE 0xB8003200, the +0x60/+0x64/+0x68
 *      register triple, WDT_E and the kick at bit 31, and the 22/15/29 field
 *      shifts are declared once for the whole Luna family and are NOT
 *      chip-conditional, in a tree whose build variants include this chip.  The
 *      scale encoding (0 = 2^25 ... 3 = 2^28) and the reset modes (0 = full
 *      chip, 1 = CPU + IPSec, 2 = software) are named there too.
 *  [4] A LIVE READ ON A SIBLING PART.  A boot log from the RTL9607C reference
 *      board prints the three registers by address -- 0xb8003260, 0xb8003264,
 *      0xb8003268 -- with WDT_CTRL holding 0xe7c00000, which decodes under the
 *      layout below as scale 3, PH1 31, PH2 0, reset mode 0.  Every field lands
 *      where this file says it does.
 *
 * The one thing NOT established is the meaning of the middle register
 * (+0x04, the vendor calls it WDTINTRR).  Nothing here reads or writes it.
 *
 * ---------------------------------------------------------------------------
 * ARMED IS NOT FIRED, AND THIS DRIVER REFUSES TO PRETEND OTHERWISE.
 *
 * There is no established latch on this SoC that says "the last reset was mine".
 * WDT_E reads 1 whenever the watchdog is ENABLED, which on a healthy unit is all
 * the time -- so a witness built on it would report a bite after every power cut
 * and every clean reboot.  That is the exact phantom this project already
 * installed once on its other board, where a vendor breadcrumb reading 3 meant
 * "the watchdog was started" and was nearly published as "the watchdog fired".
 *
 * The X111W stock firmware has no reset-reason node at all (searched: no
 * reboot_reason / reset_reason / reset_cause anywhere in its kernel or rootfs),
 * so there is nothing to adopt either.
 *
 * => this driver advertises NO WDIOF_CARDRESET and never populates
 *    wdd.bootstatus.  The witness that the watchdog BIT is the one the test
 *    suite already mandates for every case: /proc/uptime went BACKWARDS while
 *    nothing asked for a reboot.  If a real latch is ever established here, it
 *    belongs in bootstatus and this paragraph should be deleted, not softened.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/watchdog.h>

/* --- the register block, offsets from its base (0x18003260 on RTL9602C) --- */
#define RTL960X_WDT_CNT			0x00
#define RTL960X_WDT_CNT_KICK		BIT(31)
#define RTL960X_WDT_INTR		0x04
#define RTL960X_WDT_CTRL		0x08
#define RTL960X_WDT_CTRL_EN		BIT(31)
#define RTL960X_WDT_BLOCK_SIZE		0x0c

/*
 * WDT_CTRL fields.  Named for what the silicon's own headers call them, so a
 * reader can carry a name straight from a register dump to this file.
 */
#define RTL960X_WDT_CLK_SC_SHIFT	29
#define RTL960X_WDT_CLK_SC_MSK		0x3u
#define RTL960X_WDT_CLK_SC_MAX		3u
#define RTL960X_WDT_PH1_TO_SHIFT	22
#define RTL960X_WDT_PH2_TO_SHIFT	15
#define RTL960X_WDT_PH_TO_MSK		0x1fu
#define RTL960X_WDT_RESET_MODE_MSK	0x3u

/*
 * RESET_MODE 0 is the H/W FULL CHIP reset: it takes the CPU down together with
 * the PLL, analog and SerDes domains, i.e. as close to a power-on as this SoC
 * can do to itself, and it re-enters the boot ROM.  That is the only mode worth
 * having in an unattended device -- mode 1 leaves the peripherals in whatever
 * state wedged them, and mode 2 is a software reset, which is precisely what is
 * not running when a watchdog is needed.  Proven by this target's own restart
 * path, which has used mode 0 since bring-up.
 */
#define RTL960X_WDT_RESET_MODE_FULL_CHIP 0u

/* One tick is 2^(SCALE_SHIFT + WDT_CLK_SC) cycles of the LX bus clock. */
#define RTL960X_WDT_SCALE_SHIFT		25

/* Each phase field is 5 bits and counts from ONE, so 0 means a single tick. */
#define RTL960X_WDT_TICKS		32u

/*
 * Phase 2 is kept at its shortest.  It exists so a vendor kernel can dump state
 * from the phase-1 interrupt before the reset lands; we take no interrupt (see
 * the probe), so every phase-2 tick is dead time between "the deadline passed"
 * and "the device comes back".
 */
#define RTL960X_WDT_PH2_TICKS		1u

#define RTL960X_WDT_MIN_TIMEOUT		1u
#define RTL960X_WDT_DEFAULT_TIMEOUT	30u

static unsigned int timeout;
module_param(timeout, uint, 0444);
MODULE_PARM_DESC(timeout,
		 "Watchdog timeout in seconds (default: device tree, else "
		 __MODULE_STRING(RTL960X_WDT_DEFAULT_TIMEOUT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default: "
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/**
 * struct rtl960x_wdt_timing - the WDT_CTRL timing fields for one timeout
 * @clk_sc:	WDT_CLK_SC, the tick scale, 0..RTL960X_WDT_CLK_SC_MAX
 * @ph1:	WDT_PH1_TO field value; the phase lasts @ph1 + 1 ticks
 * @ph2:	WDT_PH2_TO field value; the phase lasts @ph2 + 1 ticks
 */
struct rtl960x_wdt_timing {
	u32 clk_sc;
	u32 ph1;
	u32 ph2;
};

/*
 * Pure: the longest timeout this block can express on a @rate Hz input clock.
 *
 * Rounded DOWN, deliberately: the number this returns is published to userspace
 * as max_timeout, so it has to be a window the hardware can actually hold.
 */
static unsigned int rtl960x_wdt_max_timeout(unsigned long rate)
{
	u64 cycles = (u64)RTL960X_WDT_TICKS <<
		     (RTL960X_WDT_SCALE_SHIFT + RTL960X_WDT_CLK_SC_MAX);

	return rate ? (unsigned int)div_u64(cycles, rate) : 0;
}

/*
 * Pure: turn a timeout in whole seconds into the WDT_CTRL timing fields.
 *
 * The scale is chosen as the FINEST one that still reaches @secs, which matters
 * because the field is only 5 bits: at the coarsest scale a 2 s request would be
 * rounded up to a 2.7 s window, while at the finest it lands within 168 ms.
 * Rounding is always UP, so the device never resets EARLIER than the timeout
 * userspace was promised -- the direction that matters, since the watchdog core
 * feeds at timeout/2 and an early bite would reboot a healthy unit.
 *
 * The arithmetic is deliberately shifts-only.  A 64-bit divide by a runtime
 * value is not something a 32-bit kernel should be doing in a helper that also
 * has to run in a restart path, and the divisor here is always a power of two.
 *
 * Returns 0, or -ERANGE when @secs cannot be expressed at any scale.
 */
static int rtl960x_wdt_calc_timing(unsigned long rate, unsigned int secs,
				   struct rtl960x_wdt_timing *t)
{
	unsigned int sc;

	if (!rate || !secs)
		return -ERANGE;

	for (sc = 0; sc <= RTL960X_WDT_CLK_SC_MAX; sc++) {
		unsigned int shift = RTL960X_WDT_SCALE_SHIFT + sc;
		u64 cycles = (u64)secs * rate;
		u64 ticks = (cycles + (1ULL << shift) - 1) >> shift;

		if (ticks > RTL960X_WDT_TICKS)
			continue;
		if (!ticks)
			ticks = 1;

		t->clk_sc = sc;
		t->ph1 = (u32)ticks - 1;
		t->ph2 = RTL960X_WDT_PH2_TICKS - 1;
		return 0;
	}

	return -ERANGE;
}

/* Pure: those fields packed into the WDT_CTRL bit positions, enable excluded. */
static u32 rtl960x_wdt_ctrl_timing(const struct rtl960x_wdt_timing *t,
				   u32 reset_mode)
{
	return ((t->clk_sc & RTL960X_WDT_CLK_SC_MSK) << RTL960X_WDT_CLK_SC_SHIFT) |
	       ((t->ph1 & RTL960X_WDT_PH_TO_MSK) << RTL960X_WDT_PH1_TO_SHIFT) |
	       ((t->ph2 & RTL960X_WDT_PH_TO_MSK) << RTL960X_WDT_PH2_TO_SHIFT) |
	       (reset_mode & RTL960X_WDT_RESET_MODE_MSK);
}

/**
 * struct rtl960x_wdt - one Luna watchdog instance
 * @wdd:	the watchdog the core sees
 * @base:	the three-register block
 * @clk:	LX peripheral bus clock, which the counter is scaled from
 * @rate:	@clk's frequency, cached because every timeout change needs it
 * @lock:	serialises the read-modify-write of WDT_CTRL against the kick
 */
struct rtl960x_wdt {
	struct watchdog_device	wdd;
	void __iomem		*base;
	struct clk		*clk;
	unsigned long		rate;
	spinlock_t		lock;		/* see kernel-doc above */
};

static inline struct rtl960x_wdt *to_rtl960x_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct rtl960x_wdt, wdd);
}

/*
 * Reload both phases.
 *
 * ★ READ-MODIFY-WRITE, and it is not a style preference.  Bit 31 of WDT_CNT is
 * the kick; what the other 31 bits of that register are is NOT established
 * anywhere -- the vendor prints the word and never decodes it, and it reads back
 * as zero on a running sibling, which tells you nothing about a chip that has
 * something latched there.  A blind store would be a write of 31 unknown bits on
 * the hot path of the one mechanism that has to keep working when everything
 * else has stopped.  This project has already paid for exactly that shape on its
 * other board, where the reset-enable word turned out to also hold the power-down
 * bits for the GPON and the forwarding engine.
 */
static void rtl960x_wdt_kick(struct rtl960x_wdt *wdt)
{
	u32 cnt = readl(wdt->base + RTL960X_WDT_CNT);

	writel(cnt | RTL960X_WDT_CNT_KICK, wdt->base + RTL960X_WDT_CNT);
}

static int rtl960x_wdt_ping(struct watchdog_device *wdd)
{
	rtl960x_wdt_kick(to_rtl960x_wdt(wdd));

	return 0;
}

/*
 * Program the window and arm.
 *
 * The kick comes LAST and is not optional: WDT_CTRL is written while the counter
 * may already be part-way through an older, possibly shorter window, and without
 * the reload the new timeout would only take effect at the next feed.
 */
static int rtl960x_wdt_start(struct watchdog_device *wdd)
{
	struct rtl960x_wdt *wdt = to_rtl960x_wdt(wdd);
	struct rtl960x_wdt_timing t;
	unsigned long flags;
	u32 ctrl;
	int ret;

	ret = rtl960x_wdt_calc_timing(wdt->rate, wdd->timeout, &t);
	if (ret)
		return ret;

	spin_lock_irqsave(&wdt->lock, flags);
	ctrl = readl(wdt->base + RTL960X_WDT_CTRL);
	ctrl &= ~((RTL960X_WDT_CLK_SC_MSK << RTL960X_WDT_CLK_SC_SHIFT) |
		  (RTL960X_WDT_PH_TO_MSK << RTL960X_WDT_PH1_TO_SHIFT) |
		  (RTL960X_WDT_PH_TO_MSK << RTL960X_WDT_PH2_TO_SHIFT) |
		  RTL960X_WDT_RESET_MODE_MSK);
	ctrl |= rtl960x_wdt_ctrl_timing(&t, RTL960X_WDT_RESET_MODE_FULL_CHIP);
	writel(ctrl | RTL960X_WDT_CTRL_EN, wdt->base + RTL960X_WDT_CTRL);
	rtl960x_wdt_kick(wdt);
	spin_unlock_irqrestore(&wdt->lock, flags);

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
}

static int rtl960x_wdt_stop(struct watchdog_device *wdd)
{
	struct rtl960x_wdt *wdt = to_rtl960x_wdt(wdd);
	unsigned long flags;
	u32 ctrl;

	spin_lock_irqsave(&wdt->lock, flags);
	ctrl = readl(wdt->base + RTL960X_WDT_CTRL);
	writel(ctrl & ~RTL960X_WDT_CTRL_EN, wdt->base + RTL960X_WDT_CTRL);
	spin_unlock_irqrestore(&wdt->lock, flags);

	clear_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
}

static int rtl960x_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int new_timeout)
{
	struct rtl960x_wdt *wdt = to_rtl960x_wdt(wdd);
	struct rtl960x_wdt_timing t;
	int ret;

	/* Refuse before recording it: a timeout the block cannot hold must not
	 * end up in wdd->timeout, where the core would derive a feed interval
	 * from a window the hardware was never given. */
	ret = rtl960x_wdt_calc_timing(wdt->rate, new_timeout, &t);
	if (ret)
		return ret;

	wdd->timeout = new_timeout;

	/*
	 * Reprogram only a watchdog that is already counting.  Programming a
	 * stopped one here would arm it behind the caller's back: the core
	 * allows WDIOC_SETTIMEOUT before the start.
	 */
	if (watchdog_hw_running(wdd))
		return rtl960x_wdt_start(wdd);

	return 0;
}

/*
 * No .get_timeleft.
 *
 * WDT_CNT is the kick register; whether its lower bits are a live down-counter
 * is not established, and the one live reading available reads 0 on a running,
 * healthy sibling -- which is either "not a counter" or "a counter we are
 * decoding wrong", and there is no way to tell them apart from here.  A
 * get_timeleft that returned that number would publish it through
 * /sys/class/watchdog/watchdogN/timeleft as a measurement.
 *
 * No .restart either, and that one is not caution but duplication: this target
 * already restarts through this same block, from luna_machine_restart() in
 * arch/mips/realtek-luna/setup.c, which MIPS reaches via _machine_restart.  It
 * is the proven path -- it is how the port has rebooted since bring-up -- and a
 * second handler at watchdog-core priority would silently take it over.  The
 * field definitions there and here describe one piece of hardware twice, so the
 * offline case luna_wdt_test asserts that the two agree.
 */
static const struct watchdog_info rtl960x_wdt_info = {
	.identity	= "Realtek Luna WDT",
	/*
	 * No WDIOF_CARDRESET: see the file header.  Nothing on this SoC has been
	 * shown to distinguish "the watchdog reset the chip" from "the chip was
	 * reset", and bootstatus is a claim, not a placeholder.
	 */
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops rtl960x_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= rtl960x_wdt_start,
	.stop		= rtl960x_wdt_stop,
	.ping		= rtl960x_wdt_ping,
	.set_timeout	= rtl960x_wdt_set_timeout,
};

static int rtl960x_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtl960x_wdt *wdt;
	struct resource *res;
	resource_size_t phys;
	unsigned int max_timeout;
	bool adopted;
	u32 ctrl;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	spin_lock_init(&wdt->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing the watchdog register window\n");
	if (resource_size(res) < RTL960X_WDT_BLOCK_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "window at %pa is too small, need %#x\n",
				     &res->start, RTL960X_WDT_BLOCK_SIZE);
	phys = res->start;

	/*
	 * devm_ioremap(), not devm_ioremap_resource(): this block is the tail of
	 * the SoC TC block that the system timer also lives in, and the arch
	 * restart path reaches the same registers through a fixed KSEG1 address.
	 * Claiming the window exclusively would make whichever driver probes
	 * second fail, for no benefit.
	 */
	wdt->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!wdt->base)
		return -ENOMEM;

	wdt->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(wdt->clk))
		return dev_err_probe(dev, PTR_ERR(wdt->clk), "no input clock\n");

	wdt->rate = clk_get_rate(wdt->clk);
	if (!wdt->rate)
		return dev_err_probe(dev, -EINVAL, "input clock reports 0 Hz\n");

	max_timeout = rtl960x_wdt_max_timeout(wdt->rate);
	if (max_timeout < RTL960X_WDT_MIN_TIMEOUT)
		return dev_err_probe(dev, -EINVAL,
				     "a %lu Hz input clock cannot reach a 1 s window\n",
				     wdt->rate);

	/*
	 * The phase-1 timeout has its own interrupt line on this SoC and it is
	 * deliberately NOT requested here.  It exists so a vendor kernel can
	 * collect debug state in the gap before the reset; taking it would buy
	 * this driver nothing, and the input number is the one fact in this file
	 * with a single source behind it.  The reset does not depend on it: the
	 * block asserts it from phase 2 in hardware.
	 */

	wdt->wdd.info = &rtl960x_wdt_info;
	wdt->wdd.ops = &rtl960x_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = RTL960X_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = max_timeout;
	wdt->wdd.timeout = min(RTL960X_WDT_DEFAULT_TIMEOUT, max_timeout);

	/*
	 * ★ Was it already counting when Linux took over?
	 *
	 * A boot stage that arms the watchdog and hands over to a kernel that
	 * ignores it gives a unit that reboots part-way through every boot,
	 * forever, with nothing in any log to see.  So the state is READ and
	 * reported, never assumed.
	 */
	ctrl = readl(wdt->base + RTL960X_WDT_CTRL);
	adopted = !!(ctrl & RTL960X_WDT_CTRL_EN);

	ret = watchdog_init_timeout(&wdt->wdd, timeout, dev);
	if (ret)
		dev_warn(dev, "timeout out of range, keeping %u s\n",
			 wdt->wdd.timeout);

	watchdog_set_nowayout(&wdt->wdd, nowayout);

	/*
	 * Stop it across an orderly reboot.
	 *
	 * The alternative -- staying armed so that a shutdown which itself hangs
	 * is also recovered -- is the better behaviour, and the other board in
	 * this project takes it.  It is NOT taken here, because there it rests on
	 * a measurement: warm reset and cold control read at the boot prompt,
	 * showing the block does not carry its countdown into the next boot.  No
	 * such reading exists for this SoC, and getting it wrong is not a missed
	 * improvement, it is a unit that reboot-loops with an armed counter and
	 * no way in.  Take the measurement, then delete this call and say so.
	 */
	watchdog_stop_on_reboot(&wdt->wdd);

	if (adopted) {
		/*
		 * Take the window OVER rather than inherit it.  The counter
		 * handed to us is part-way through a countdown nobody recorded,
		 * and the watchdog core derives its feed interval from
		 * wdd->timeout and not from what the hardware is carrying -- so
		 * an inherited window shorter than ours would be fed too slowly
		 * and bite during our own boot.
		 */
		ret = rtl960x_wdt_start(&wdt->wdd);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot reprogram the inherited window\n");
	}

	platform_set_drvdata(pdev, wdt);

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;

	dev_info(dev,
		 "Luna WDT at %pa on a %lu Hz clock: %u s timeout (max %u s), full-chip reset, %s at kernel entry%s\n",
		 &phys, wdt->rate, wdt->wdd.timeout, wdt->wdd.max_timeout,
		 adopted ? "ALREADY RUNNING (reprogrammed to our window)"
			 : "stopped",
		 nowayout ? ", nowayout" : "");

	return 0;
}

static const struct of_device_id rtl960x_wdt_of_match[] = {
	{ .compatible = "realtek,rtl960x-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl960x_wdt_of_match);

static struct platform_driver rtl960x_wdt_driver = {
	.probe	= rtl960x_wdt_probe,
	.driver	= {
		.name		= "rtl960x-wdt",
		.of_match_table	= rtl960x_wdt_of_match,
	},
};
module_platform_driver(rtl960x_wdt_driver);

MODULE_DESCRIPTION("Realtek Luna (RTL960x) SoC watchdog");
MODULE_LICENSE("GPL");
