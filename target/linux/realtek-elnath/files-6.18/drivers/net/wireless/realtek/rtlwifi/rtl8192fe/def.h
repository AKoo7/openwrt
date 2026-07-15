/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2026  rtl8192fe clean-room contributors */

#ifndef __RTL92F_DEF_H__
#define __RTL92F_DEF_H__

/* RX DMA ring depth (descriptors). The TX ring depth (TX_DESC_NUM_92E = 512)
 * and the buffer-descriptor segment count (BUFDESC_SEG_NUM) are provided by
 * the shared rtlwifi PCI core (pci.h / wifi.h); this sub-driver reuses them.
 */
#define RX_DESC_NUM_92F					512

/* TX DMA ring depth (descriptors).  MUST equal the SW ring size the rtlwifi
 * PCI core actually allocates for this chip, i.e. rtlpci->txringcount[] ==
 * RT_TXDESC_NUM == 128 (pci.h).  It is NOT 512: the 512-deep TX_DESC_NUM_92E
 * ring is only taken when hw_type == HARDWARE_TYPE_RTL8192EE, but this chip's
 * PCI id (0x818c) is unknown to _rtl_pci_find_adapter(), so hw_type falls back
 * to the RTL8192CE default and _rtl_pci_init_trx_var() sizes every TX ring to
 * RT_TXDESC_NUM (128).  (use_new_trx_flow is true here, so the BE_QUEUE 256
 * override is skipped and every data/mgmt ring is 128; the beacon queue is a
 * separate 2-desc ring with cur_tx_wp pinned to 0.)  This constant is
 * programmed into the HW TXBD depth registers (REG_*Q_TXBD_NUM, hw.c) and used
 * by rtl92fe_get_available_desc() (trx.c); set_desc wraps cur_tx_wp at
 * ring->entries (128).  If HW depth (this) > SW depth (128), cur_tx_wp wraps
 * at 128 while the HW read pointer wraps at 512 -> at the 128th frame the
 * pointers desync, the HW DMA-fetches phantom BDs past the allocation,
 * is_tx_desc_closed() goes permanently false, TX-reclaim stops, and
 * _rtl_pci_tx() stop-queues every AC forever (beacon exempt).  Keep == 128.
 */
#define TX_DESC_NUM_92F					128

/* Buffer-descriptor segment count for this chip's PCIe BD ring.
 * 0: 2 seg, 1: 4 seg, 2: 8 seg.  The RTL8192F uses the 4-segment layout
 * (seg0 = TX descriptor, seg1 = payload), matching the core default.
 */
#define RTL8192FE_SEG_NUM				BUFDESC_SEG_NUM

#define HAL_PRIME_CHNL_OFFSET_DONT_CARE			0
#define HAL_PRIME_CHNL_OFFSET_LOWER			1
#define HAL_PRIME_CHNL_OFFSET_UPPER			2

#define RX_MPDU_QUEUE					0

#define IS_HT_RATE(_rate)	\
	((_rate) >= DESC_RATEMCS0)
#define IS_CCK_RATE(_rate)	\
	((_rate) >= DESC_RATE1M && (_rate) <= DESC_RATE11M)
#define IS_OFDM_RATE(_rate)	\
	((_rate) >= DESC_RATE6M && (_rate) <= DESC_RATE54M)

/* Chip-version magic for the RTL8192F (2T2R 802.11n).
 * The part is read in hw.c: the test-chip variant is distinguished from the
 * mass-production "normal" chip via the SYS_CFG1 TRP_VAUX_EN strap, exactly
 * as on the rtl8192ee.  RF type is always RF_2T2R for this device.  Only the
 * two values below are used by hw.c to tag rtlhal->version; downstream code
 * branches on RF type (RF_2T2R) rather than on the raw composite, so the exact
 * encoding only needs to round-trip consistently within this driver.
 * TODO(8192f): validate on hardware -- confirm the SYS_CFG1/SYS_CFG2 readout
 * matches these composite ids on a real RTL8192F endpoint.
 */
enum version_8192f {
	VERSION_TEST_CHIP_2T2R_8192F = 0x0020,
	VERSION_NORMAL_CHIP_2T2R_8192F = 0x1028,
	VERSION_UNKNOWN_8192F = 0xFF,
};

/* HW queue-selector values written into the TX descriptor QSEL field. */
enum rtl_desc_qsel {
	QSLT_BK = 0x2,
	QSLT_BE = 0x0,
	QSLT_VI = 0x5,
	QSLT_VO = 0x7,
	QSLT_BEACON = 0x10,
	QSLT_HIGH = 0x11,
	QSLT_MGNT = 0x12,
	QSLT_CMD = 0x13,
};

/* TX/RX descriptor rate codes (DESC_RATE1M..DESC_RATEMCS15) are provided by
 * enum rtl_desc_rate in the shared rtlwifi core (wifi.h) with the identical
 * ordering this chip expects; the IS_HT/CCK/OFDM_RATE macros above resolve
 * against that enum.  No per-chip rate enum is defined here to avoid a
 * redefinition collision with wifi.h.
 */

#endif
