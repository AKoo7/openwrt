/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe DMA ring transport — open reimplementation of the vendor bcmtr_pcie.c.
 * See docs/decomp/trmux.md + docs/decomp/bar0-register-map.md for derivation.
 *
 * Wire protocol (decoded from bcmtr_pcie_send/receive/tx_collect/connect):
 *
 *  - TX/RX descriptor ("transfer unit", TU) rings live in the Maple MAC's own
 *    DDR, windowed to the host through BAR4 (ddr_win). TU stride = 0x40 bytes.
 *  - Immediately after each ring, the device expects two companion u32 arrays
 *    (also in DDR): a buffer-descriptor-info array and an ownership array.
 *  - TX: host sets tx_owner[idx]=1 when posting; the device clears it on
 *    completion. tx_collect reclaims slots where tx_owner==0 && tx_skb!=NULL.
 *  - RX: the device writes rx_bd_info[idx] = len | (chan<<16); bit24 (0x01000000)
 *    is the "owned by device / empty" flag. Host polls until bit24 clears,
 *    reads len+chan, then reposts the buffer and sets bit24 again.
 *  - Host→Maple-DDR address translation: TU.pkt_addr = host_dma - 0x7f000000.
 *  - TX doorbell: writel(1, BAR0+0x64414) after posting.
 */
#ifndef MAPLE_RING_H
#define MAPLE_RING_H

#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct maple_dev;

#define MAPLE_TU_SIZE		0x40	/* TU descriptor stride               */
#define MAPLE_TX_DOORBELL	1
#define MAPLE_DDR_ALIAS		0x7f000000
#define MAPLE_TU_LAST		0x80000000U	/* +0x30 bit31: last-in-frame */
#define MAPLE_TU_NEXT		0x00000004U	/* +0x30 bit2:  has-next       */
#define MAPLE_RX_OWNED		0x01000000U	/* rx_bd_info bit24: device-owned/empty */

/* Ring geometry defaults — MUST match the values written to the SRAM mailbox
 * in maple_fw.c (bcm_fld_set_rings_size / bcm_fld_set_mtu_size). */
#define MAPLE_DFLT_RXQ		256
#define MAPLE_DFLT_TXQ		256
#define MAPLE_DFLT_MTU		2048

struct maple_ring {
	struct device	*dev;
	void __iomem	*bar0;		/* TX doorbell                  */
	void __iomem	*bar2;		/* mailbox (ring offsets)       */
	void __iomem	*ddr_win;	/* BAR4 = Maple DDR window      */

	/* Rings + companion arrays in Maple DDR (all u32-stride except TU). */
	void __iomem	*rx_tu;		/* RX TU descriptors (0x40 each) */
	void __iomem	*rx_bd_info;	/* RX bd_info array (u32 each)   */
	void __iomem	*rx_owner;	/* RX ownership array (u32 each) */
	void __iomem	*tx_tu;		/* TX TU descriptors (0x40 each) */
	void __iomem	*tx_hdr;	/* TX header array (u32 each)    */
	void __iomem	*tx_owner;	/* TX ownership array (u32 each) */

	u32		rxq_length, txq_length, max_mtu;

	/* Host-side indices (vmalloc/RAM). */
	u32		current_tx;	/* producer                      */
	u32		tx_collect;	/* reclaim cursor                */
	u32		last_tx;	/* prev posted (descriptor chain) */
	u32		current_rx;	/* RX consumer                   */

	/* Host shadow arrays. */
	struct sk_buff	**rx_skb;
	dma_addr_t	*rx_dma;
	struct sk_buff	**tx_skb;

	/* RX upcall: hand a completed packet (fresh skb) to the BAL layer. */
	void		*rx_ctx;
	void		(*rx_upcall)(void *ctx, struct sk_buff *skb);

	spinlock_t	lock;		/* protects indices + shadow arrays */

	u64		tx_pcie_full;	/* stat: TX slots busy */
	u64		rx_empty;	/* stat: RX polled empty */
};

int  maple_ring_alloc(struct maple_ring *r, struct maple_dev *mdev);
void maple_ring_free(struct maple_ring *r);
int  maple_ring_connect(struct maple_ring *r);
void maple_ring_disconnect(struct maple_ring *r);

void maple_ring_rx(struct maple_ring *r);		/* bcmtr_pcie_receive   */
void maple_ring_tx_done(struct maple_ring *r);		/* bcmtr_pcie_tx_collect */
int  maple_ring_tx(struct maple_ring *r, struct sk_buff *skb);

#endif /* MAPLE_RING_H */
