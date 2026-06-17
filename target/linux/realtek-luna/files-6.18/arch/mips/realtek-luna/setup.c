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

/*
 * SoC watchdog timer (KSEG1), TC block. Forcing a WDT timeout is the only reset
 * that actually re-enters the boot ROM: the control register's RESET_MODE=0 is a
 * H/W full-chip reset (resets the CPU AND the PLL/analog/SerDes domains, ~= a
 * power-on reset). Poking the swcore software-reset register (0x1b0000e0 bit7)
 * only resets the switch core, leaving the CPU spinning -> the board hangs and
 * never reboots, which also defeats the GPON WAN cold-start auto-recovery
 * (a clean reboot is what re-rolls the non-deterministic upstream analog lock).
 *
 * BSP_WDTCTRLR @0x18003268: [31] enable, [30:29] clk-scale (0=2^25 .. 3=2^28
 * LX clocks/unit), [26:22] phase-1 timeout (5b), [19:15] phase-2 timeout (5b),
 * [1:0] reset-mode (0=full chip, 1=CPU+IPSec, 2=S/W). Kick reg @0x18003260.
 */
#define LUNA_WDT_CTRL		((void __iomem *)CKSEG1ADDR(0x18003268))
#define LUNA_WDT_E		BIT(31)		/* watchdog enable          */
#define LUNA_WDT_CLK_SC_SHIFT	29		/* overflow scale           */
#define LUNA_WDT_PH1_TO_SHIFT	22		/* phase-1 timeout          */
#define LUNA_WDT_PH2_TO_SHIFT	15		/* phase-2 timeout          */
#define LUNA_WDT_RST_FULLCHIP	0u		/* RESET_MODE = full chip   */

extern char __dtb_start[];
void prom_putchar(char c);	/* bring-up bisect markers (remove later) */

static void luna_machine_restart(char *command)
{
	local_irq_disable();
	pr_emerg("Restarting via SoC watchdog full-chip reset...\n");
	/*
	 * Full-chip reset, fastest scale (2^25 LX clocks ~ 0.17s), phase-1=1,
	 * phase-2=0, enabled; do NOT kick it -> it times out almost immediately
	 * and the boot ROM takes over. Spin until the reset lands.
	 */
	__raw_writel(LUNA_WDT_E |
		     (0u << LUNA_WDT_CLK_SC_SHIFT) |
		     (1u << LUNA_WDT_PH1_TO_SHIFT) |
		     (0u << LUNA_WDT_PH2_TO_SHIFT) |
		     LUNA_WDT_RST_FULLCHIP,
		     LUNA_WDT_CTRL);
	while (1)
		cpu_relax();
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
