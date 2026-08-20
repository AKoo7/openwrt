// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL9603CVD SoC interrupt controller ("GISR" aggregator).
 *
 * Clean-room driver written from observed hardware facts only -- register
 * bases/offsets, bit semantics, routing scheme and the vendor's own init
 * values. No proprietary source text or structure was copied.
 *
 * ---------------------------------------------------------------------
 * WHY THIS CHIP DOES NOT USE A MIPS GIC
 *
 * The RTL9603CVD has NO MIPS GIC and no Coherence Manager. Its own vendor
 * kernel says so on this exact board (tier 1, LANLY G24W boot console,
 * captured 2026-08-18):
 *
 *	MIPS CPS SMP unable to proceed without a CM
 *	GIC isn't present!
 *	rtl9603cvd_gisr_init: gisr_translate_d2c = ...
 *
 * so a "mti,gic" node here describes silicon that is not present: the UART
 * interrupt would never arrive, early printk would work and the console
 * would never become interactive. Describe this aggregator instead.
 * ---------------------------------------------------------------------
 *
 * REGISTER MAP (block base 0x18003000; one block PER CPU, stride 0x40):
 *
 *	GIMR0	+0x00	mask,   inputs  0..31  (1 = enabled)
 *	GIMR1	+0x04	mask,   inputs 32..63
 *	GISR0	+0x08	status, inputs  0..31  (raw; AND with mask for pending)
 *	GISR1	+0x0c	status, inputs 32..63
 *	IRR0..6	+0x10..	routing, 4 bits per input, 8 inputs per word
 *
 * GIMR0 bit 12 is not an input: it is the master peripheral-IRQ enable and
 * must be set for any peripheral interrupt to be delivered at all. The
 * dispatch below therefore excludes it from the pending scan.
 *
 * ROUTING NIBBLE -> CP0 LINE. A nibble of 0 disconnects the input; a value
 * V selects aggregator output line V-1, and output line 0 is wired to CP0
 * HW IRQ 2, so the delivered line is CP0 HW IRQ (V + 1). Cross-checked two
 * ways, and the two agree on every case:
 *
 *   - the vendor's own dispatchers on this chip read GISR word 0 from CP0
 *     IP3 and GISR word 1 from CP0 IP4, and take the SoC timer on IP7;
 *   - the vendor's own routing values (loaded below) give nibble 2 to every
 *     input in word 0 that it uses (switch core, GMAC0, PON NIC), nibble 3
 *     to every input in word 1 that it uses (UART0, GPIO bank ABCD), and
 *     nibble 6 to the per-CPU system timer.
 *
 *   2 -> CP0 IRQ 3	3 -> CP0 IRQ 4		6 -> CP0 IRQ 7
 *
 * The strongest single check is the per-CPU one: the vendor loads a
 * DIFFERENT word 4 on each CPU, and the only nibbles that differ are the
 * two system-timer inputs -- TC0 enabled on CPU0 and disconnected on CPU1,
 * TC1 the other way round. That is exactly what a per-CPU clockevent needs
 * and nothing else in the map would explain it.
 *
 * INPUT NUMBERING IS THE NATIVE GISR BIT, and it is shared with the
 * RTL9602C, whose device tree in this same target independently uses 49
 * for UART0 and 43 for the system timer. Named inputs (a reserved input is
 * omitted):
 *
 *	 4 ECC		 5 NAND		 8 switch core	14 USB host p2
 *	16 PCIe		17 PCM0		18 PCM1		24 PON NIC
 *	25 PON NIC DS	26 GMAC0 int0	27 GMAC0 int1	28 VoIP XSI
 *	29 VoIP SPI	31 LX bus debug	32 GPIO JKMN	39 WDT phase-1
 *	40 WDT phase-2	41 GPIO EFGH	42 GPIO ABCD	43..48 TC0..TC5
 *	49..52 UART0..3	60..62 TC6..TC8
 *
 * DELIVERY INVARIANT: every GISR interrupt is delivered to CPU 0. Only
 * CPU 0's block carries routing and mask bits; CPU 1's block is masked
 * shut, so a secondary CPU can never take an interrupt this driver did not
 * arrange. Per-CPU affinity is OWED work and needs a running SMP kernel to
 * verify, which this chip cannot have until the CPS/CM question is settled.
 *
 * OWED: this aggregator is the same block the RTL9602C carries, and
 * irq-rtl9602c.c in this directory drives it with the same offsets. They
 * are kept apart deliberately for now -- that driver carries two routing
 * overrides established empirically on the X111W, which are facts about
 * THAT board's cascade and must not be inherited here. Merge them into one
 * rtl960x_* driver once this chip has booted and both routings are proven.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define GISR_INPUTS		64
#define GISR_WORDS		(GISR_INPUTS / 32)
#define GISR_CPU_STRIDE		0x40		/* one register block per CPU */
#define GISR_GIMR(w)		(0x00 + (w) * 4)
#define GISR_GISR(w)		(0x08 + (w) * 4)
#define GISR_IRR(i)		(0x10 + (i) * 4)
#define GISR_IRR_WORDS		7
#define GISR_PERIPH_EN		12		/* GIMR0 bit: master enable */

/*
 * Routing words as the vendor kernel loads them on this chip, unmodified.
 * Index is the IRR word; the values are hardware configuration, not code.
 * Word 4 is the one that differs per CPU: on CPU 0 it enables system timer
 * input 43 and disconnects 44, which is what the delivery invariant above
 * requires.
 */
static const u32 gisr_routing_cpu0[GISR_IRR_WORDS] = {
	0x03333330, 0x30302222, 0x00020222, 0x22020333,
	0x33333063, 0x32322022, 0x00333000,
};

struct rtl9603cvd_intc {
	void __iomem		*base;		/* CPU 0's block */
	raw_spinlock_t		lock;
	struct irq_domain	*domain;
};

static void rtl9603cvd_intc_mask(struct irq_data *d)
{
	struct rtl9603cvd_intc *ic = irq_data_get_irq_chip_data(d);
	unsigned int word = d->hwirq / 32;
	u32 val;

	raw_spin_lock(&ic->lock);
	val = readl(ic->base + GISR_GIMR(word));
	val &= ~BIT(d->hwirq % 32);
	writel(val, ic->base + GISR_GIMR(word));
	raw_spin_unlock(&ic->lock);
}

static void rtl9603cvd_intc_unmask(struct irq_data *d)
{
	struct rtl9603cvd_intc *ic = irq_data_get_irq_chip_data(d);
	unsigned int word = d->hwirq / 32;
	u32 val;

	raw_spin_lock(&ic->lock);
	val = readl(ic->base + GISR_GIMR(word));
	val |= BIT(d->hwirq % 32);
	writel(val, ic->base + GISR_GIMR(word));
	raw_spin_unlock(&ic->lock);
}

static struct irq_chip rtl9603cvd_intc_chip = {
	.name		= "rtl9603cvd-intc",
	.irq_mask	= rtl9603cvd_intc_mask,
	.irq_unmask	= rtl9603cvd_intc_unmask,
};

static int rtl9603cvd_intc_map(struct irq_domain *d, unsigned int irq,
			       irq_hw_number_t hw)
{
	struct rtl9603cvd_intc *ic = d->host_data;

	irq_set_chip_and_handler(irq, &rtl9603cvd_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, ic);

	return 0;
}

static const struct irq_domain_ops rtl9603cvd_intc_domain_ops = {
	.map	= rtl9603cvd_intc_map,
	.xlate	= irq_domain_xlate_onecell,
};

/*
 * Both status words are scanned whichever CP0 line woke us: the routing
 * spreads inputs over three lines, and a scan that trusted the line it
 * arrived on would drop every interrupt whose nibble we had mis-read.
 * A spurious wake simply finds nothing pending.
 */
static void rtl9603cvd_intc_dispatch(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct rtl9603cvd_intc *ic = irq_desc_get_handler_data(desc);
	int word;

	chained_irq_enter(chip, desc);

	for (word = 0; word < GISR_WORDS; word++) {
		unsigned long pending = readl(ic->base + GISR_GISR(word)) &
					readl(ic->base + GISR_GIMR(word));
		unsigned int bit;

		if (word == 0)
			pending &= ~BIT(GISR_PERIPH_EN);

		for_each_set_bit(bit, &pending, 32)
			generic_handle_domain_irq(ic->domain, word * 32 + bit);
	}

	chained_irq_exit(chip, desc);
}

static int __init rtl9603cvd_intc_of_init(struct device_node *node,
					  struct device_node *parent)
{
	struct rtl9603cvd_intc *ic;
	void __iomem *cpu1;
	int parent_irq, n = 0, i, ret;

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
	 * Mask every input on CPU 0's block except the master peripheral
	 * enable, then load the routing. Order matters: routing an input
	 * that is still unmasked from whatever the bootloader left behind
	 * would let it fire before its handler exists.
	 */
	writel(BIT(GISR_PERIPH_EN), ic->base + GISR_GIMR(0));
	writel(0, ic->base + GISR_GIMR(1));
	for (i = 0; i < GISR_IRR_WORDS; i++)
		writel(gisr_routing_cpu0[i], ic->base + GISR_IRR(i));

	/* Enforce the delivery invariant: CPU 1 takes nothing. */
	cpu1 = ic->base + GISR_CPU_STRIDE;
	writel(0, cpu1 + GISR_GIMR(0));
	writel(0, cpu1 + GISR_GIMR(1));

	ic->domain = irq_domain_create_linear(of_fwnode_handle(node),
					      GISR_INPUTS,
					      &rtl9603cvd_intc_domain_ops, ic);
	if (!ic->domain) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	for (n = 0; (parent_irq = irq_of_parse_and_map(node, n)) > 0; n++)
		irq_set_chained_handler_and_data(parent_irq,
						 rtl9603cvd_intc_dispatch, ic);
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

IRQCHIP_DECLARE(rtl9603cvd_intc, "realtek,rtl9603cvd-intc",
		rtl9603cvd_intc_of_init);
