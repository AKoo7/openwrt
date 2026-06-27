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

#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/of_clk.h>
#include <linux/of_fdt.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/cpu-features.h>
#include <asm/cpu-info.h>
#include <asm/mips-cps.h>
#include <asm/mipsregs.h>
#include <asm/prom.h>
#include <asm/reboot.h>
#include <asm/smp-ops.h>
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

#ifdef CONFIG_MIPS_CM
/*
 * The on-chip L2 (256 KB, 8-way, 32-byte lines) comes out of the boot loader
 * with UNINITIALIZED tags. This SoC reports the L2 line size as 0 in Config2
 * (the geometry is fixed in silicon, not described), so the generic MIPS cache
 * code never sizes or initializes the L2; the first cached write-back to a DRAM
 * page that maps onto a garbage-tagged set/way -- the very top of usable DRAM,
 * touched first by setup_zero_pages() -- then wedges. Invalidate every L2 line
 * with Index_Store_Tag (writing a zeroed, invalid tag; NO write-back, so no
 * garbage is flushed to memory). Run this very early, while the boot loader's
 * CCA override still keeps DRAM out of the L2, so no live kernel data is lost.
 */
#define LUNA_L2_SIZE	(256 << 10)	/* 256 KB */
#define LUNA_L2_LINE	32		/* bytes (Config2 SL reads 0 on this SoC) */

static void __init luna_l2_invalidate_tags(void)
{
	unsigned long addr;

	/* Zero the L2 tag/data shadow registers -> Index_Store_Tag writes an
	 * invalid tag. (CP0 $28 sel 4/5, $29 sel 5 = L2 TagLo/DataLo/DataHi.) */
	__asm__ __volatile__(
		"	.set	push		\n"
		"	.set	noreorder	\n"
		"	mtc0	$0, $28, 4	\n"
		"	mtc0	$0, $28, 5	\n"
		"	mtc0	$0, $29, 5	\n"
		"	.set	pop		\n"
		::: "memory");

	for (addr = CKSEG0; addr < CKSEG0 + LUNA_L2_SIZE; addr += LUNA_L2_LINE)
		__asm__ __volatile__(
			"	.set	push		\n"
			"	.set	noreorder	\n"
			"	cache	0x0b, 0(%0)	\n"	/* Index_Store_Tag_S */
			"	.set	pop		\n"
			:: "r" (addr) : "memory");

	__asm__ __volatile__("sync" ::: "memory");
}

/* Synchronous hex via the early UART (bring-up probe; remove later). */
static void __init prom_puthex(unsigned int v)
{
	int i;

	for (i = 28; i >= 0; i -= 4) {
		int d = (v >> i) & 0xf;

		prom_putchar(d < 10 ? '0' + d : 'a' + (d - 10));
	}
}

/*
 * Direct probe of the top usable-DRAM page (phys 0x11fff000, just under the
 * 0x12000000 / 288 MB cap), uncached then cached, with synchronous markers, to
 * confirm the capped top now responds: full "P1 2 3 <deadbeef> 4 5 6 <cafe>"
 * means the memory + cache are good there and the boot can proceed; output
 * stopping at "P1"/"P12" would mean the real top is still lower.
 * (bring-up diagnostic; remove later.)
 */
static void __init luna_probe_top_page(void)
{
	local_irq_disable();
	prom_putchar('\n');
	prom_putchar('P'); prom_putchar('1');
	*(volatile unsigned int *)0xb1fff000 = 0xdeadbeef;	/* KSEG1 uncached */
	prom_putchar('2');
	prom_putchar('3'); prom_puthex(*(volatile unsigned int *)0xb1fff000);
	prom_putchar('4');
	*(volatile unsigned int *)0x91fff000 = 0x0000cafe;	/* KSEG0 cached */
	prom_putchar('5');
	prom_putchar('6'); prom_puthex(*(volatile unsigned int *)0x91fff000);
	prom_putchar('\n');
}

/*
 * UserLocal / thread-pointer (TLS) enable.
 *
 * The C library reads the per-thread pointer with `rdhwr $29` (hardware
 * register 29 = UserLocal). That instruction runs in user mode only when
 * CP0 HWREna[29] is set, and the generic trap init sets it only for a CPU
 * whose feature set records UserLocal (Config3.ULRI). If the per-CPU probe
 * did not record it, HWREna[29] stays clear, the very first TLS read in the
 * dynamic loader traps as a Reserved Instruction, and PID 1 dies with SIGILL.
 *
 * This core implements UserLocal, so trust the architectural Config3.ULRI bit
 * directly and record the option before per_cpu_trap_init() programs HWREna.
 * If the register really were absent (ULRI == 0) we leave the feature off and
 * the kernel's rdhwr emulation handles the trap instead. Runs after cpu_probe()
 * and well before trap_init(), in setup_arch()'s device_tree_init().
 */
static void __init luna_enable_userlocal(void)
{
	unsigned int cfg3 = read_c0_config3();

	/* M1 bring-up diagnostic: show how the CPU was identified (remove later). */
	pr_emerg("LUNA-DIAG: prid=%08x cputype=%d config3=%08x ULRI=%d userlocal=%d mmips=%d\n",
		 read_c0_prid(), current_cpu_type(), cfg3,
		 !!(cfg3 & MIPS_CONF3_ULRI),
		 cpu_has_userlocal ? 1 : 0, cpu_has_mmips ? 1 : 0);

	if (cfg3 & MIPS_CONF3_ULRI)
		current_cpu_data.options |= MIPS_CPU_ULRI;
}
#endif /* CONFIG_MIPS_CM */

void __init plat_mem_setup(void)
{
	prom_putchar('['); prom_putchar('M'); prom_putchar(']');
#ifdef CONFIG_MIPS_CM
	luna_l2_invalidate_tags();	/* clean the boot-time garbage L2 tags */
	luna_probe_top_page();		/* bring-up: probe phys 0x1bfff000 */
#endif
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

/*
 * Unflatten the DT and, on the interAptiv (Coherent Processing System) parts,
 * register the SMP operations before the generic setup_arch() reaches
 * plat_smp_setup() -- which unconditionally dereferences the smp-ops vector, so
 * a platform that leaves it NULL hangs there (right after the reserved-memory
 * scan, before paging_init). Probe the Coherency Manager and Cluster Power
 * Controller, then register the CPS ops; fall back to uniprocessor ops if no CM
 * is present. On the single-threaded RLX parts CONFIG_SMP is off, these probes
 * compile out to no-ops and this is a plain DT unflatten.
 */
void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();

	mips_cm_probe();
	mips_cpc_probe();

#ifdef CONFIG_MIPS_CM
	/* Record the UserLocal (TLS rdhwr $29) feature before trap_init() so
	 * HWREna[29] gets programmed and the C library's TLS read does not SIGILL. */
	luna_enable_userlocal();

	/*
	 * Let Linux MANAGE the on-chip L2 so streaming DMA stays coherent without
	 * bounce buffers. With CONFIG_MIPS_CPU_SCACHE selected, cpu_cache_init() ->
	 * mips_sc_probe() sizes the L2 from Config2 -- which on this SoC reads
	 * SS=4 / SL=4 / SA=7 => 1024 sets x 32-byte line x 8 ways = 256 KB, with the
	 * bypass bit (Config2[12]) clear -- and wires the L2 into the DMA cache ops
	 * (r4k_dma_cache_wback_inv). The boot-time garbage L2 tags are invalidated
	 * first in plat_mem_setup() (luna_l2_invalidate_tags) before the kernel
	 * touches the L2. (Earlier bring-up forced MIPS_CACHE_NOT_PRESENT to dodge a
	 * setup_scache() panic; that panic was really MIPS_CPU_SCACHE being
	 * unselected for this platform, not a zero L2 line size.)
	 */

	/*
	 * The boot loader programs a CCA-default-override in the CM GCR_BASE
	 * (low byte) so accesses use a fixed cache attribute before the L2 is
	 * set up. mips_cm_probe() only fixes the default target (-> MEM); clear
	 * the override too (as the vendor L2-init does) so coherent-domain
	 * accesses fall back to the per-page cache attribute. Left set, the
	 * overridden CCA routes top-of-DRAM cached accesses through the CM
	 * coherent path beyond its range, wedging the first such access -- the
	 * memblock alloc in setup_zero_pages(), early in mm_core_init().
	 */
	if (mips_cm_present()) {
		unsigned long l2cfg = read_gcr_l2_config();

		/* M1 bring-up diagnostic: dump the CM/L2 state (remove later). */
		pr_emerg("LUNA-DIAG: cm_rev=%x config=%x config2=%x gcr_base=%llx l2cfg=%lx l2bypass=%d\n",
			 mips_cm_revision(), read_c0_config(), read_c0_config2(),
			 (unsigned long long)read_gcr_base(), l2cfg,
			 !!(l2cfg & CM_GCR_L2_CONFIG_BYPASS));

		write_gcr_base(read_gcr_base() & ~0xffULL);
	}
#endif

	if (!register_cps_smp_ops())
		return;
	register_up_smp_ops();
}

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
	prom_putchar('['); prom_putchar('i'); prom_putchar(']');	/* irqchip/GIC probe returned */
}
