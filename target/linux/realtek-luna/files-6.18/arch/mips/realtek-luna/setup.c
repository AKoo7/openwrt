// SPDX-License-Identifier: GPL-2.0-only
/*
 * "Luna" GPON ONU (RTL960xC, RLX/Taroko core) — platform setup.
 *
 * Independent implementation from the SoC's register interface (register bases,
 * reset and watchdog programming) and mainline MIPS DT-platform conventions.
 * The generic arch/mips device_tree_init() (unflatten) is used as-is.
 * Interrupts are handled by the SoC INTC (irqchip driver) via the standard
 * irqchip_init() entry; the system tick comes from the SoC TC timer (clocksource
 * driver) because the Taroko core's CP0 Count is unreliable.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/of_clk.h>
#include <linux/of_fdt.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/prom.h>
#include <asm/reboot.h>
#include <asm/time.h>

/* SoC watchdog control (KSEG1). bit31 = enable. */
#define LUNA_WDT_CTRL		((void __iomem *)CKSEG1ADDR(0x18003268))
/* Software-reset register (KSEG1). bit7 = system reset. */
#define LUNA_SOFT_RESET		((void __iomem *)CKSEG1ADDR(0x1b0000e0))

extern char __dtb_start[];
void prom_putchar(char c);	/* bring-up bisect markers (remove later) */

static void luna_machine_restart(char *command)
{
	local_irq_disable();
	pr_emerg("Restarting via SoC soft-reset...\n");
	while (1)
		__raw_writel(__raw_readl(LUNA_SOFT_RESET) | BIT(7),
			     LUNA_SOFT_RESET);
}

static void luna_machine_halt(void)
{
	local_irq_disable();
	pr_emerg("System halted.\n");
	while (1)
		cpu_relax();
}

void __init plat_mem_setup(void)
{
	prom_putchar('['); prom_putchar('M'); prom_putchar(']');
	/*
	 * The preloader may arm the SoC hardware watchdog; a minimal kernel
	 * has no kicker, so disable it before any driver runs. (Replace with
	 * a real watchdog driver once one is integrated.)
	 */
	__raw_writel(0, LUNA_WDT_CTRL);

	/* MMIO/peripheral window: SPI-NOR + SoC registers. */
	ioport_resource.start = 0x14000000;
	ioport_resource.end   = 0x1fffffff;
	iomem_resource.start  = 0x14000000;
	iomem_resource.end    = 0x1fffffff;

	_machine_restart = luna_machine_restart;
	_machine_halt    = luna_machine_halt;
	pm_power_off     = luna_machine_halt;

	/*
	 * The board's bootloader passes no device tree, so the image carries
	 * an appended DTB (CONFIG_MIPS_RAW_APPENDED_DTB). get_fdt() returns the
	 * appended/fw-passed/builtin blob, whichever applies.
	 */
	__dt_setup_arch(get_fdt());
}

/* device_tree_init() intentionally omitted: the generic arch/mips weak
 * implementation (unflatten_and_copy_device_tree) is exactly what this platform
 * needs, so it is used unchanged rather than re-stated here. */

void __init plat_time_init(void)
{
	prom_putchar('['); prom_putchar('T'); prom_putchar(']');
	of_clk_init(NULL);
	timer_probe();
}

void __init arch_init_irq(void)
{
	prom_putchar('['); prom_putchar('I'); prom_putchar(']');
	irqchip_init();
}
