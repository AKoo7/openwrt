/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Single source of truth for register/protocol constants — pure numbers, no
 * kernel deps, so both the driver (maple_hw.h/maple_onu.h) and the userspace
 * oracle test can include it.
 */
#ifndef MAPLE_REGS_H
#define MAPLE_REGS_H

/* PCI device IDs */
#define MAPLE_VENDOR_BROADCOM	0x14e4
#define MAPLE_DEV_6862		0x6862
#define MAPLE_DEV_6863		0x6863
#define MAPLE_DEV_5554		0x5554

/* BAL object / group IDs (decoded from trmux.ko DWARF — see docs/decomp/) */
#define MAPLE_OBJ_GPON_ONU	25
#define MAPLE_OBJ_GPON_NI	24
#define MAPLE_OBJ_GPON_TRX	26
#define MAPLE_GRP_GPON_ONU_KEY		480
#define MAPLE_GRP_GPON_ONU_CFG		481
#define MAPLE_GRP_GPON_ONU_STAT		482
#define MAPLE_GRP_GPON_ONU_SET_ONU_STATE	544
#define MAPLE_GRP_GPON_NI_KEY		411
#define MAPLE_GRP_GPON_NI_CFG		412
#define MAPLE_GRP_GPON_NI_STAT		413

/* ONU operation (onu_operation enum) for set_onu_state */
#define MAPLE_ONU_OP_ACTIVE		0
#define MAPLE_ONU_OP_INACTIVE		1
#define MAPLE_ONU_OP_DISABLE		2	/* block */
#define MAPLE_ONU_OP_ENABLE		3	/* unblock */
#define MAPLE_ONU_OP_ACTIVE_STANDBY	4

/* BAL transport header */
#define MAPLE_BAL_HDR_SIZE	0x10
#define MAPLE_BAL_DIR_REQUEST	0
#define MAPLE_BAL_DIR_RESPONSE	1

/* BAR0 register offsets (see docs/decomp/bar0-register-map.md) */
#define MAPLE_BAR0_CAPABILITY		0x64068
#define MAPLE_BAR0_DEVICE_ID		0x6406c
#define MAPLE_BAR0_HOST_RESET		0x64204
#define MAPLE_BAR0_ISR_STATUS		0x64318
#define MAPLE_BAR0_ISR_CLEAR		0x64320
#define MAPLE_BAR0_ISR_MASK		0x64324
#define MAPLE_BAR0_ISR_MASK_SET		0x64328
#define MAPLE_BAR0_ISR_MASK_CLEAR	0x6432c
#define MAPLE_BAR0_CTRL			0x64410
#define MAPLE_BAR0_TX_DOORBELL		0x64414
#define MAPLE_BAR0_CTRL_SETUP_A		0x69308
#define MAPLE_BAR0_CTRL_SETUP_B		0x6930c
#define MAPLE_BAR0_BOOT_CTRL		0x4800c0
#define MAPLE_BAR0_CPU_START		0x4800c4

/* IRQ status bits (MAPLE_BAR0_ISR_STATUS) */
#define MAPLE_IRQ_TX_DONE	0x01
#define MAPLE_IRQ_TX_ERR	0x02
#define MAPLE_IRQ_RX		0x04
#define MAPLE_IRQ_RX_ERR	0x08
#define MAPLE_IRQ_TX_MASK	(MAPLE_IRQ_TX_DONE | MAPLE_IRQ_TX_ERR)	/* = 0x3 */
#define MAPLE_IRQ_RX_MASK	(MAPLE_IRQ_RX | MAPLE_IRQ_RX_ERR)	/* = 0xc */
#define MAPLE_IRQ_ALL		(MAPLE_IRQ_TX_MASK | MAPLE_IRQ_RX_MASK)

/* CPU boot magic (fld start_bootloader) */
#define MAPLE_CPU_START_VAL	0x01000000
#define MAPLE_BOOT_CTRL_VAL	0x0c401080

/* PCIe DMA ring */
#define MAPLE_TU_SIZE		0x40
#define MAPLE_TX_DOORBELL_VAL	1
/* DDR alias: vendor converts kernel-virt → Maple bus addr by subtracting
 * 0x7f000000. On ARM (PHYS_OFFSET=0x60000000), dma_addr = virt - 0x60000000,
 * so pkt_addr = dma_addr - (0x7f000000 - 0x60000000) = dma_addr - 0x1f000000. */
#define MAPLE_DDR_ALIAS		0x7f000000
#define MAPLE_HOST_PHYS_OFFSET	0x60000000
#define MAPLE_PCIE_WINDOW_OFF	(MAPLE_DDR_ALIAS - MAPLE_HOST_PHYS_OFFSET) /* = 0x1f000000 */
#define MAPLE_TU_LAST		0x80000000U
#define MAPLE_TU_NEXT		0x00000004U
#define MAPLE_RX_OWNED		0x01000000U

#endif /* MAPLE_REGS_H */
