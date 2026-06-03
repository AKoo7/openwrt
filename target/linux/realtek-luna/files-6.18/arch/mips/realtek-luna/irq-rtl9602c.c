// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL960xC "Luna" SoC interrupt controller (RLX/Taroko core).
 *
 * Independent implementation from the SoC's register interface (register
 * bases/offsets, bit semantics and routing scheme observed on the hardware).
 * The block is a 64-input aggregator that funnels SoC interrupts
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
 * The routing nibbles are laid out in inverted word order: input 0 lives
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
#define LUNA_INTC_ROUTE_ON	1
#define LUNA_INTC_ROUTE_OFF	0

struct luna_intc {
	void __iomem	*base;
	raw_spinlock_t	lock;
	struct irq_domain *domain;
};

/* Inverted word order: input 0 is in the highest IRR word, lowest nibble. */
static u32 luna_irr_reg(unsigned int hwirq)
{
	unsigned int word = LUNA_INTC_IRR_WORDS - 1 - (hwirq / 8);

	return LUNA_INTC_IRR_BASE + word * 4;
}

static unsigned int luna_irr_shift(unsigned int hwirq)
{
	return (hwirq % 8) * 4;
}

static void luna_set_route(struct luna_intc *ic, unsigned int hwirq, u32 route)
{
	u32 reg = luna_irr_reg(hwirq);
	unsigned int shift = luna_irr_shift(hwirq);
	u32 val;

	val = readl(ic->base + reg);
	val &= ~(0xf << shift);
	val |= (route & 0xf) << shift;
	writel(val, ic->base + reg);
}

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

	raw_spin_lock(&ic->lock);
	luna_set_route(ic, hw, LUNA_INTC_ROUTE_ON);
	raw_spin_unlock(&ic->lock);

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

		for_each_set_bit(bit, &pending, 32)
			generic_handle_domain_irq(ic->domain, word * 32 + bit);
	}

	chained_irq_exit(chip, desc);
}

static int __init luna_intc_of_init(struct device_node *node,
				    struct device_node *parent)
{
	struct luna_intc *ic;
	int parent_irq, i, ret;

	ic = kzalloc(sizeof(*ic), GFP_KERNEL);
	if (!ic)
		return -ENOMEM;

	raw_spin_lock_init(&ic->lock);

	ic->base = of_iomap(node, 0);
	if (!ic->base) {
		ret = -ENXIO;
		goto err_free;
	}

	/* Mask everything and clear all routing before wiring the cascade. */
	writel(0, ic->base + LUNA_INTC_GIMR(0));
	writel(0, ic->base + LUNA_INTC_GIMR(1));
	for (i = 0; i < LUNA_INTC_IRR_WORDS; i++)
		writel(0, ic->base + LUNA_INTC_IRR_BASE + i * 4);

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq) {
		ret = -ENODEV;
		goto err_unmap;
	}

	ic->domain = irq_domain_create_linear(of_fwnode_handle(node),
					      LUNA_INTC_INPUTS,
					      &luna_intc_domain_ops, ic);
	if (!ic->domain) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	irq_set_chained_handler_and_data(parent_irq, luna_intc_dispatch, ic);

	return 0;

err_unmap:
	iounmap(ic->base);
err_free:
	kfree(ic);
	return ret;
}

IRQCHIP_DECLARE(rtl9602c_intc, "realtek,rtl9602c-intc", luna_intc_of_init);
