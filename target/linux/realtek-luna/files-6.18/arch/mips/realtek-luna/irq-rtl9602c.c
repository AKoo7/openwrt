// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL960xC "Luna" SoC interrupt controller (RLX/Taroko core).
 *
 * Clean-room driver written from observed hardware facts only (register
 * bases/offsets, bit semantics and routing scheme). No vendor source was
 * copied. The block is a 64-input aggregator that funnels SoC interrupts
 * onto the CPU's CP0 HW interrupt lines:
 *
 *   GIMR0  0x00  mask,   inputs  0..31  (1 = enabled)
 *   GIMR1  0x04  mask,   inputs 32..63
 *   GISR0  0x08  status, inputs  0..31  (raw, AND with mask for pending)
 *   GISR1  0x0c  status, inputs 32..63
 *   IRR0   0x10  routing, 4 bits per input, 8 inputs per word
 *   ...                   a routing nibble of 0 disconnects the input;
 *   IRR7   0x2c           1..15 selects CPU output line 0..14.
 *
 * Realtek numbers the routing nibbles in inverted word order: input 0 lives
 * in the *last* IRR word's lowest nibble. The aggregator's output line 0 is
 * wired to CP0 HW IRQ 2 on this family.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#define LUNA_INTC_INPUTS	64
#define LUNA_INTC_GIMR(w)	(0x00 + (w) * 4)	/* w = 0,1 */
#define LUNA_INTC_GISR(w)	(0x08 + (w) * 4)	/* w = 0,1 */
#define LUNA_INTC_IRR_BASE	0x10
#define LUNA_INTC_IRR_WORDS	(LUNA_INTC_INPUTS / 8)	/* 8 inputs/word */

/* Routing nibble -> output line that the DT declares as our parent. */
#define LUNA_INTC_ROUTE_ON	2	/* (routing now pre-loaded with vendor GIRR values) */
#define LUNA_INTC_PERIPH_EN	12	/* GIMR0 bit12 = master peripheral-IRQ enable */
#define LUNA_INTC_ROUTE_OFF	0

struct luna_intc {
	void __iomem	*base;
	raw_spinlock_t	lock;
	struct irq_domain *domain;
};

static void luna_intc_mask(struct irq_data *d)
{
	struct luna_intc *ic = irq_data_get_irq_chip_data(d);
	unsigned int word = d->hwirq / 32;
	u32 val;

	raw_spin_lock(&ic->lock);
	val = readl(ic->base + LUNA_INTC_GIMR(word));
	val &= ~BIT(d->hwirq % 32);
	writel(val, ic->base + LUNA_INTC_GIMR(word));
	raw_spin_unlock(&ic->lock);
}

static void luna_intc_unmask(struct irq_data *d)
{
	struct luna_intc *ic = irq_data_get_irq_chip_data(d);
	unsigned int word = d->hwirq / 32;
	u32 val;

	raw_spin_lock(&ic->lock);
	val = readl(ic->base + LUNA_INTC_GIMR(word));
	val |= BIT(d->hwirq % 32);
	writel(val, ic->base + LUNA_INTC_GIMR(word));
	raw_spin_unlock(&ic->lock);
}

static struct irq_chip luna_intc_chip = {
	.name		= "rtl9602c-intc",
	.irq_mask	= luna_intc_mask,
	.irq_unmask	= luna_intc_unmask,
};

static int luna_intc_map(struct irq_domain *d, unsigned int irq,
			 irq_hw_number_t hw)
{
	struct luna_intc *ic = d->host_data;

	irq_set_chip_and_handler(irq, &luna_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, ic);
	/* routing (GIRR) is pre-loaded with vendor values in of_init */

	return 0;
}

static const struct irq_domain_ops luna_intc_domain_ops = {
	.map	= luna_intc_map,
	.xlate	= irq_domain_xlate_onecell,
};

static void luna_intc_dispatch(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct luna_intc *ic = irq_desc_get_handler_data(desc);
	int word;

	chained_irq_enter(chip, desc);

	for (word = 0; word < 2; word++) {
		unsigned long pending = readl(ic->base + LUNA_INTC_GISR(word)) &
					readl(ic->base + LUNA_INTC_GIMR(word));
		unsigned int bit;

		if (word == 0)
			pending &= ~BIT(LUNA_INTC_PERIPH_EN);	/* aggregate, not a real irq */

		for_each_set_bit(bit, &pending, 32)
			generic_handle_domain_irq(ic->domain, word * 32 + bit);
	}

	chained_irq_exit(chip, desc);
}

static int __init luna_intc_of_init(struct device_node *node,
				    struct device_node *parent)
{
	struct luna_intc *ic;
	int parent_irq, n = 0, ret;

	ic = kzalloc(sizeof(*ic), GFP_KERNEL);
	if (!ic)
		return -ENOMEM;

	raw_spin_lock_init(&ic->lock);

	ic->base = of_iomap(node, 0);
	if (!ic->base) {
		ret = -ENXIO;
		goto err_free;
	}

	/*
	 * Mask all sources except the master peripheral-IRQ enable
	 * (GIMR0 bit IRQ_PERIPHERAL=12 — "must be set to enable peripheral
	 * irq"), then load the vendor's known-good per-irq routing (GIRR/IRR).
	 * Word0 sources are delivered on CP0 IP3, word1 on IP4.
	 */
	writel(BIT(LUNA_INTC_PERIPH_EN), ic->base + LUNA_INTC_GIMR(0));
	writel(0, ic->base + LUNA_INTC_GIMR(1));
	writel(0x03333330, ic->base + LUNA_INTC_IRR_BASE + 0x00);
	writel(0x30302222, ic->base + LUNA_INTC_IRR_BASE + 0x04);
	/*
	 * Word at IRR_BASE+0x08 holds the routing nibbles for inputs 26..33
	 * (ascending tiling: input 42 is nibble1 of the +0x10 word per the
	 * UART/timer fix below). GMAC0 ethernet is input 26 = bits[3:0] here.
	 * The vendor value 2 targets a CP0 IP line our cascade does not pick
	 * up (same failure mode as the original UART input), so route it to
	 * the proven-delivering value 6 (as timer input 43 / uart input 49
	 * use). [3:0]: 2 -> 6. Once GIMR0 bit26 is unmasked (request_irq ->
	 * luna_intc_unmask) input 26 is then delivered through the cascade.
	 */
	writel(0x00020226, ic->base + LUNA_INTC_IRR_BASE + 0x08);
	writel(0x22020333, ic->base + LUNA_INTC_IRR_BASE + 0x0c);
	/*
	 * IRR4 (base+0x20) holds the routing nibbles for inputs 42..49: the
	 * SoC timer TC0 (input 43) at bits[7:4] and the 16550 UART0 (input 49)
	 * at bits[31:28]. The timer is proven to deliver with routing value 6,
	 * so route the UART to 6 as well (the vendor value 3 targets an IP line
	 * that is not picked up by our cascade, leaving userspace TX/RX dead
	 * while polled kernel printk still works). [31:28]: 3 -> 6.
	 */
	writel(0x63333063, ic->base + LUNA_INTC_IRR_BASE + 0x10);
	writel(0x32322022, ic->base + LUNA_INTC_IRR_BASE + 0x14);
	writel(0x00333000, ic->base + LUNA_INTC_IRR_BASE + 0x18);

	ic->domain = irq_domain_create_linear(of_fwnode_handle(node),
					      LUNA_INTC_INPUTS,
					      &luna_intc_domain_ops, ic);
	if (!ic->domain) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	/*
	 * Cascade our dispatch on every CP0 IP line the routing may target
	 * (GIRR value = IP); the dispatch scans both GISR words regardless.
	 */
	for (n = 0; (parent_irq = irq_of_parse_and_map(node, n)) > 0; n++)
		irq_set_chained_handler_and_data(parent_irq,
						 luna_intc_dispatch, ic);
	if (!n) {
		ret = -ENODEV;
		goto err_unmap;
	}

	return 0;

err_unmap:
	iounmap(ic->base);
err_free:
	kfree(ic);
	return ret;
}

IRQCHIP_DECLARE(rtl9602c_intc, "realtek,rtl9602c-intc", luna_intc_of_init);
