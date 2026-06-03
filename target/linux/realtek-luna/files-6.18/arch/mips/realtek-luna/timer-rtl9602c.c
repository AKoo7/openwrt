// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL960xC "Luna" SoC timer/counter (TC) clockevent.
 *
 * The RLX/Taroko core's CP0 Count is unreliable, so the SoC TC block drives
 * the system tick. Independent implementation from the SoC's register
 * interface, derived from hardware facts observed on the silicon.
 *
 * Per-channel registers (channels are 0x10 apart):
 *   TC_DATA  +0x00  reload/target value (counts DOWN to 0)
 *   TC_CNT   +0x04  current count (read-only)
 *   TC_CTRL  +0x08  BIT(28) enable, BIT(24) auto-reload (periodic),
 *                   low bits = input clock divisor (DIVF)
 *   TC_INT   +0x0c  BIT(20) interrupt enable, BIT(16) pending (write-1-clear)
 *
 * We run channel 0 at a fixed 10 MHz tick base (divisor = src_clk / 10 MHz)
 * and let the kernel fall back to the jiffies clocksource for timekeeping.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define TC_DATA		0x00
#define TC_CNT		0x04
#define TC_CTRL		0x08
#define TC_INT		0x0c

#define TC_CTRL_EN	BIT(28)
#define TC_CTRL_RELOAD	BIT(24)		/* auto-reload => periodic */
#define TC_INT_IE	BIT(20)
#define TC_INT_IP	BIT(16)		/* write 1 to clear */

#define TC_TICK_RATE	10000000U	/* 10 MHz tick base */

struct luna_timer {
	void __iomem		*base;
	u32			divisor;
	struct clock_event_device evt;
};

#define to_luna_timer(d)	container_of(d, struct luna_timer, evt)

static void luna_timer_stop(struct luna_timer *t)
{
	writel(0, t->base + TC_CTRL);
	writel(readl(t->base + TC_INT) & ~TC_INT_IE, t->base + TC_INT);
}

static int luna_timer_shutdown(struct clock_event_device *evt)
{
	luna_timer_stop(to_luna_timer(evt));
	return 0;
}

static int luna_timer_set_periodic(struct clock_event_device *evt)
{
	struct luna_timer *t = to_luna_timer(evt);

	luna_timer_stop(t);
	writel(DIV_ROUND_UP(TC_TICK_RATE, HZ), t->base + TC_DATA);
	writel(TC_INT_IE, t->base + TC_INT);
	writel(TC_CTRL_EN | TC_CTRL_RELOAD | t->divisor, t->base + TC_CTRL);
	return 0;
}

static int luna_timer_set_next(unsigned long delta,
			       struct clock_event_device *evt)
{
	struct luna_timer *t = to_luna_timer(evt);

	luna_timer_stop(t);
	writel(delta, t->base + TC_DATA);
	writel(TC_INT_IE, t->base + TC_INT);
	writel(TC_CTRL_EN | t->divisor, t->base + TC_CTRL);
	return 0;
}

static irqreturn_t luna_timer_isr(int irq, void *dev_id)
{
	struct luna_timer *t = dev_id;

	/* Acknowledge: write back the pending bit (write-1-clear). */
	writel(readl(t->base + TC_INT) | TC_INT_IP, t->base + TC_INT);

	t->evt.event_handler(&t->evt);
	return IRQ_HANDLED;
}

static int __init luna_timer_of_init(struct device_node *node)
{
	struct luna_timer *t;
	struct clk *clk;
	unsigned long rate;
	int irq, ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->base = of_iomap(node, 0);
	if (!t->base) {
		ret = -ENXIO;
		goto err_free;
	}

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_unmap;
	}
	ret = clk_prepare_enable(clk);
	if (ret)
		goto err_unmap;

	rate = clk_get_rate(clk);
	t->divisor = rate / TC_TICK_RATE;
	if (t->divisor < 2 || t->divisor > 0xffff) {
		pr_err("rtl9602c-timer: bad divisor %u for src %lu Hz\n",
		       t->divisor, rate);
		ret = -EINVAL;
		goto err_unmap;
	}

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		ret = -EINVAL;
		goto err_unmap;
	}

	luna_timer_stop(t);

	t->evt.name		= "rtl9602c-tick";
	t->evt.rating		= 300;
	t->evt.features		= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	t->evt.cpumask		= cpumask_of(0);
	t->evt.irq		= irq;
	t->evt.set_state_shutdown = luna_timer_shutdown;
	t->evt.set_state_periodic = luna_timer_set_periodic;
	t->evt.set_state_oneshot  = luna_timer_shutdown;
	t->evt.tick_resume	= luna_timer_shutdown;
	t->evt.set_next_event	= luna_timer_set_next;

	ret = request_irq(irq, luna_timer_isr, IRQF_TIMER | IRQF_IRQPOLL,
			  "rtl9602c-tick", t);
	if (ret)
		goto err_unmap;

	clockevents_config_and_register(&t->evt, TC_TICK_RATE, 2, 0x0fffffff);
	pr_info("rtl9602c-timer: tick %u Hz (src %lu Hz, div %u), irq %d\n",
		TC_TICK_RATE, rate, t->divisor, irq);
	return 0;

err_unmap:
	iounmap(t->base);
err_free:
	kfree(t);
	return ret;
}

TIMER_OF_DECLARE(rtl9602c_timer, "realtek,rtl9602c-timer", luna_timer_of_init);
