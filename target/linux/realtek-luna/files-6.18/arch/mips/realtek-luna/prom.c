// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek "Luna" GPON ONU (RTL960xC, RLX/Taroko core) — early boot / PROM.
 *
 * Independent implementation from the documented MIPS boot contract and the
 * SoC's bootloader handoff: the bootloader enters Linux with a legacy
 * argc/argv pair in fw_arg0/fw_arg1.
 *
 * Copyright (C) 2026 Confiared <contact@confiared.com>
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>

#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/prom.h>

/* 16550 UART0 (KSEG1), reg-shift 2: THR @ +0x00, LSR @ +0x14, LSR.THRE = 0x20. */
#define LUNA_UART0	((void __iomem *)CKSEG1ADDR(0x18002000))
#define UART_THR	0x00
#define UART_LSR	(5 << 2)
#define UART_LSR_THRE	0x20

const char *get_system_type(void)
{
	return "Realtek RTL960xC (Luna/Taroko)";
}

void prom_putchar(char c)
{
	while (!(__raw_readb(LUNA_UART0 + UART_LSR) & UART_LSR_THRE))
		;
	__raw_writeb(c, LUNA_UART0 + UART_THR);
}

/*
 * The bootloader enters Linux with a legacy MIPS argument vector:
 *   a0 (fw_arg0) = argc, a1 (fw_arg1) = argv (KSEG-mapped char **).
 * Stitch argv back into arcs_cmdline; the arch then merges this with the
 * device-tree /chosen bootargs. ethaddr=... arrives this way too and is
 * consumed by the NIC driver (per the per-board MAC policy).
 */
static void __init prom_init_cmdline(void)
{
	int argc = (int)fw_arg0;
	char **argv = (char **)CKSEG1ADDR(fw_arg1);
	int i;

	if (!argc || !argv)
		return;

	for (i = 0; i < argc; i++) {
		char *arg = (char *)CKSEG1ADDR((unsigned long)argv[i]);

		if (!arg || !*arg)
			continue;
		if (arcs_cmdline[0])
			strlcat(arcs_cmdline, " ", sizeof(arcs_cmdline));
		strlcat(arcs_cmdline, arg, sizeof(arcs_cmdline));
	}
}

void __init prom_init(void)
{
	/* Bring-up bisect markers (earliest reliable output; remove later). */
	prom_putchar('\n');
	prom_putchar('[');
	prom_putchar('P');
	prom_putchar(']');
	prom_init_cmdline();
}

void __init prom_free_prom_memory(void)
{
}
