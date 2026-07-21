/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BAR0/BAR2 hardware definitions for the Broadcom Maple BCM68620 GPON MAC.
 * Pure register/protocol #defines live in maple_regs.h (shared with the oracle
 * test); this header adds the SRAM (BAR2) boot mailbox, CPU-state bits and the
 * MMIO helpers used only by the kernel driver. See docs/decomp/bar0-register-map.md.
 */
#ifndef MAPLE_HW_H
#define MAPLE_HW_H

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/types.h>

#include "maple_regs.h"		/* PCI IDs, BAR0 offsets, IRQ bits, ring consts */

/* ---- SRAM (BAR2) mailbox — host/device boot-record + comm area -----------
 * Lives at the top of the Maple's SRAM window (soc_sram_base + offset).
 * Recovered from fld.ko (see docs/decomp/fld.md). */
#define MAPLE_SRAM_EXC_STATE_CPU0	0xff24	/* RO exception state */
#define MAPLE_SRAM_EXC_STATE_CPU1	0xff28
#define MAPLE_SRAM_DEV_BOOTREC_0	0xff3c	/* RO device rx_tu_ring_offset */
#define MAPLE_SRAM_DEV_BOOTREC_1	0xff40	/* RO device tx_tu_ring_offset */
#define MAPLE_SRAM_HOST_EVENT		0xdf1c	/* WO host event */
#define MAPLE_SRAM_LOG_AREA_LEN		0xccc4	/* postmortem log length */
#define MAPLE_SRAM_MTU_SIZE		0xff44	/* WO mtu */
#define MAPLE_SRAM_HOST_TX_SIZE		0xff48	/* WO ring TX depth */
#define MAPLE_SRAM_HOST_RX_SIZE		0xff4c	/* WO ring RX depth */
#define MAPLE_SRAM_RINGS_READY		0xff50	/* RW bit0: host sets after sizes */
#define MAPLE_SRAM_OS_ENTRY		0xff54	/* WO OS entry address */
#define MAPLE_SRAM_PROTO_VERSION	0xff58	/* RO protocol version */
#define MAPLE_SRAM_REASON		0xffb4	/* RO failure reason */
#define MAPLE_SRAM_MODEL_ID		0xffc4	/* WO fw envelope */
#define MAPLE_SRAM_REL_REV		0xffc8
#define MAPLE_SRAM_REL_MINOR		0xffcc
#define MAPLE_SRAM_REL_MAJOR		0xffd0
#define MAPLE_SRAM_IMAGE_CHECKSUM	0xffd4
#define MAPLE_SRAM_IMAGE_LENGTH		0xffd8
#define MAPLE_SRAM_HOST_FLAG		0xffdc	/* RW bit0: host→dev handshake */
#define MAPLE_SRAM_DEV_FLAG		0xffe0	/* RW bit0: dev→host handshake */
#define MAPLE_SRAM_TEST_DDR		0xffec	/* WO test_ddr / finish-ddr flag */
#define MAPLE_SRAM_CPU_STATE		0xfff0	/* RO one-hot, see below */
#define MAPLE_SRAM_RAS_MODE_B		0xfff8
#define MAPLE_SRAM_RAS_MODE_A		0xfffc
#define MAPLE_BOOT_IMAGE_MAX		0xff20	/* bootloader fits SRAM[0..] */

/* ---- CPU state (MAPLE_SRAM_CPU_STATE, one-hot) --------------------------- */
#define MAPLE_CPU_FINISH_BOOTLOADER	BIT(0)
#define MAPLE_CPU_VERIFY_DDR		BIT(1)
#define MAPLE_CPU_RUNNING_FROM_DDR	BIT(2)
#define MAPLE_CPU_READY			BIT(3)	/* target: firmware fully running */
#define MAPLE_CPU_DDR_IMAGE_ERROR	BIT(4)

/* ---- Convenience MMIO helpers (vendor wraps each in a DSB) ---------------- */
static inline u32 maple_rd(void __iomem *bar0, u32 off)
{
	return readl(bar0 + off);
}

static inline void maple_wr(void __iomem *bar0, u32 off, u32 val)
{
	writel(val, bar0 + off);
	/* The vendor issues DataSynchronizationBarrier(SY) after every MMIO
	 * store (ARM dsb(sy)). wmb() resolves to dsb(sy) on ARM. */
	wmb();
}

static inline void maple_irq_enable(void __iomem *bar0, u32 bits)
{
	maple_wr(bar0, MAPLE_BAR0_ISR_MASK_CLEAR, bits);	/* unmask */
}

static inline void maple_irq_disable(void __iomem *bar0, u32 bits)
{
	maple_wr(bar0, MAPLE_BAR0_ISR_MASK_SET, bits);		/* mask */
}

static inline void maple_irq_clear(void __iomem *bar0, u32 bits)
{
	maple_wr(bar0, MAPLE_BAR0_ISR_CLEAR, bits);		/* W1C */
}

#endif /* MAPLE_HW_H */
