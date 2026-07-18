// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2026  CatchChallenger project
 *
 * PCIe bus glue for the RTL8192XB (PCI [10ec:0192]).  The 8192XB uses the
 * same AX-generation HAXI/V1 PCIe access path as the RTL8852C, so the
 * rtw89_pci_info here mirrors the 8852C's (register facts, shared core
 * helpers).
 */

#include <linux/module.h>
#include <linux/pci.h>

#include "pci.h"
#include "reg.h"
#include "rtw8192xb.h"

static const struct rtw89_pci_bd_idx_addr rtw8192xb_bd_idx_addr_low_power = {
	.tx_bd_addrs = {R_AX_DRV_FW_HSK_0, R_AX_DRV_FW_HSK_1, R_AX_DRV_FW_HSK_2,
			R_AX_DRV_FW_HSK_3, 0, 0,
			0, 0, R_AX_DRV_FW_HSK_4,
			0, 0, 0,
			R_AX_DRV_FW_HSK_5},
	.rx_bd_addrs = {R_AX_DRV_FW_HSK_6, R_AX_DRV_FW_HSK_7},
};

static const struct rtw89_pci_info rtw8192xb_pci_info = {
	.gen_def		= &rtw89_pci_gen_ax,
	.isr_def		= &rtw89_pci_isr_ax,
	.txbd_trunc_mode	= MAC_AX_BD_TRUNC,
	.rxbd_trunc_mode	= MAC_AX_BD_TRUNC,
	.rxbd_mode		= MAC_AX_RXBD_PKT,
	.tag_mode		= MAC_AX_TAG_MULTI,
	.tx_burst		= MAC_AX_TX_BURST_V1_256B,
	.rx_burst		= MAC_AX_RX_BURST_V1_128B,
	.wd_dma_idle_intvl	= MAC_AX_WD_DMA_INTVL_256NS,
	.wd_dma_act_intvl	= MAC_AX_WD_DMA_INTVL_256NS,
	.multi_tag_num		= MAC_AX_TAG_NUM_8,
	.lbc_en			= MAC_AX_PCIE_ENABLE,
	.lbc_tmr		= MAC_AX_LBC_TMR_2MS,
	.autok_en		= MAC_AX_PCIE_DISABLE,
	.io_rcy_en		= MAC_AX_PCIE_ENABLE,
	.io_rcy_tmr		= MAC_AX_IO_RCY_ANA_TMR_6MS,
	.rx_ring_eq_is_full	= false,
	.check_rx_tag		= false,
	.no_rxbd_fs		= false,
	.group_bd_addr		= false,
	.rpp_fmt_size		= sizeof(struct rtw89_pci_rpp_fmt),

	.init_cfg_reg		= R_AX_HAXI_INIT_CFG1,
	.txhci_en_bit		= B_AX_TXHCI_EN_V1,
	.rxhci_en_bit		= B_AX_RXHCI_EN_V1,
	.rxbd_mode_bit		= B_AX_RXBD_MODE_V1,
	.exp_ctrl_reg		= R_AX_HAXI_EXP_CTRL,
	.max_tag_num_mask	= B_AX_MAX_TAG_NUM_V1_MASK,
	.rxbd_rwptr_clr_reg	= R_AX_RXBD_RWPTR_CLR_V1,
	.txbd_rwptr_clr2_reg	= R_AX_TXBD_RWPTR_CLR2_V1,
	.dma_io_stop		= {R_AX_HAXI_INIT_CFG1, B_AX_STOP_AXI_MST},
	.dma_stop1		= {R_AX_HAXI_DMA_STOP1, B_AX_TX_STOP1_MASK},
	.dma_stop2		= {R_AX_HAXI_DMA_STOP2, B_AX_TX_STOP2_ALL},
	.dma_busy1		= {R_AX_HAXI_DMA_BUSY1, DMA_BUSY1_CHECK},
	.dma_busy2_reg		= R_AX_HAXI_DMA_BUSY2,
	.dma_busy3_reg		= R_AX_HAXI_DMA_BUSY3,

	.rpwm_addr		= R_AX_PCIE_HRPWM_V1,
	.cpwm_addr		= R_AX_PCIE_CRPWM,
	.mit_addr		= R_AX_INT_MIT_RX_V1,
	.wp_sel_addr		= R_AX_WP_ADDR_H_SEL0_3,
	.tx_dma_ch_mask		= 0,
	.bd_idx_addr_low_power	= &rtw8192xb_bd_idx_addr_low_power,
	.dma_addr_set		= &rtw89_pci_ch_dma_addr_set_v1,
	.bd_ram_table		= &rtw89_bd_ram_table_dual,

	.ltr_set		= rtw89_pci_ltr_set_v1,
	.fill_txaddr_info	= rtw89_pci_fill_txaddr_info_v1,
	.parse_rpp		= rtw89_pci_parse_rpp,
	.config_intr_mask	= rtw89_pci_config_intr_mask_v1,
	.enable_intr		= rtw89_pci_enable_intr_v1,
	.disable_intr		= rtw89_pci_disable_intr_v1,
	.recognize_intrs	= rtw89_pci_recognize_intrs_v1,

	.ssid_quirks		= NULL,
};

static const struct rtw89_driver_info rtw89_8192xbe_info = {
	.chip = &rtw8192xb_chip_info,
	.variant = NULL,
	.quirks = NULL,
	.bus = {
		.pci = &rtw8192xb_pci_info,
	},
};

static const struct pci_device_id rtw89_8192xbe_id_table[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x0192),
		.driver_data = (kernel_ulong_t)&rtw89_8192xbe_info,
	},
	{},
};
MODULE_DEVICE_TABLE(pci, rtw89_8192xbe_id_table);

static struct pci_driver rtw89_8192xbe_driver = {
	.name		= "rtw89_8192xbe",
	.id_table	= rtw89_8192xbe_id_table,
	.probe		= rtw89_pci_probe,
	.remove		= rtw89_pci_remove,
	.driver.pm	= &rtw89_pm_ops,
	.err_handler    = &rtw89_pci_err_handler,
};
module_pci_driver(rtw89_8192xbe_driver);

MODULE_AUTHOR("CatchChallenger project");
MODULE_DESCRIPTION("Realtek 802.11ax wireless 8192XBE driver");
MODULE_LICENSE("Dual BSD/GPL");
