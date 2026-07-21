// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe DMA ring transport — open reimplementation of bcmtr_pcie.c.
 * See maple_ring.h + docs/decomp/trmux.md for the decoded protocol.
 *
 * RX/TX bookkeeping runs in hard-IRQ context (called from the ISR), hence the
 * spinlock and GFP_ATOMIC allocations. A production version would defer packet
 * processing to NAPI/a tasklet (TODO); the ownership protocol here matches the
 * vendor byte-for-byte.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/printk.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "maple_hw.h"
#include "maple_pci.h"

/* Ring geometry defaults are in maple_ring.h (shared with maple_fw.c). */

/* Transfer-unit descriptor (0x40 bytes), in the Maple DDR ring. */
struct maple_tu {
	u32	pad0;		/* +0x00 */
	u32	pkt_addr;	/* +0x04: host_buf - MAPLE_DDR_ALIAS */
	u32	pad8;		/* +0x08 */
	u32	length;		/* +0x0c */
	u32	pad10[8];	/* +0x10..0x2c */
	u32	last_next;	/* +0x30: MAPLE_TU_LAST / MAPLE_TU_NEXT */
	u32	pad34[3];	/* +0x34..0x3c */
};

static inline struct maple_tu *__tu(void __iomem *ring, u32 idx)
{
	return (struct maple_tu *)(ring + idx * MAPLE_TU_SIZE);
}

static inline u32 rd_arr(void __iomem *arr, u32 idx)
{
	return readl(arr + idx * 4);
}
static inline void wr_arr(void __iomem *arr, u32 idx, u32 val)
{
	writel(val, arr + idx * 4);
}

int maple_ring_alloc(struct maple_ring *r, struct maple_dev *mdev)
{
	r->dev     = &mdev->pdev->dev;
	r->bar0    = mdev->bar0;
	r->bar2    = mdev->bar2;
	r->ddr_win = mdev->bar4;
	r->rxq_length = MAPLE_DFLT_RXQ;
	r->txq_length = MAPLE_DFLT_TXQ;
	r->max_mtu    = MAPLE_DFLT_MTU;
	r->current_tx = r->tx_collect = r->last_tx = r->current_rx = 0;
	spin_lock_init(&r->lock);

	r->rx_skb = kcalloc(r->rxq_length, sizeof(*r->rx_skb), GFP_KERNEL);
	r->rx_dma = kcalloc(r->rxq_length, sizeof(*r->rx_dma), GFP_KERNEL);
	r->tx_skb = kcalloc(r->txq_length, sizeof(*r->tx_skb), GFP_KERNEL);
	if (!r->rx_skb || !r->rx_dma || !r->tx_skb)
		return -ENOMEM;
	return 0;
}

static dma_addr_t post_rx_buf(struct maple_ring *r, u32 idx)
{
	struct sk_buff *skb;
	dma_addr_t dma;

	skb = alloc_skb(r->max_mtu + 0xff, GFP_ATOMIC);
	if (!skb)
		return 0;
	if ((uintptr_t)skb->data & 0x7f)
		skb_reserve(skb, 0x80 - ((uintptr_t)skb->data & 0x7f));
	dma = dma_map_single(r->dev, skb->data, r->max_mtu + 0xff, DMA_FROM_DEVICE);
	if (dma_mapping_error(r->dev, dma)) {
		dev_kfree_skb_any(skb);
		return 0;
	}
	r->rx_skb[idx] = skb;
	r->rx_dma[idx] = dma;
	/* Write the buffer addr into the RX TU, mark bd_info device-owned. */
	writel(dma - MAPLE_PCIE_WINDOW_OFF, &__tu(r->rx_tu, idx)->pkt_addr);
	writel(r->max_mtu,            &__tu(r->rx_tu, idx)->length);
	writel(MAPLE_TU_LAST,         &__tu(r->rx_tu, idx)->last_next);
	wr_arr(r->rx_bd_info, idx, MAPLE_RX_OWNED);
	wr_arr(r->rx_owner,   idx, 0);
	wmb();
	return dma;
}

static void free_rx_bufs(struct maple_ring *r)
{
	u32 i;

	for (i = 0; i < r->rxq_length; i++) {
		if (r->rx_skb[i]) {
			dma_unmap_single(r->dev, r->rx_dma[i],
					 r->max_mtu + 0xff, DMA_FROM_DEVICE);
			dev_kfree_skb_any(r->rx_skb[i]);
			r->rx_skb[i] = NULL;
		}
	}
}

void maple_ring_free(struct maple_ring *r)
{
	free_rx_bufs(r);
	for (u32 i = 0; i < r->txq_length; i++)
		if (r->tx_skb[i]) {
			dev_kfree_skb_any(r->tx_skb[i]);
			r->tx_skb[i] = NULL;
		}
	kfree(r->rx_skb);
	kfree(r->rx_dma);
	kfree(r->tx_skb);
	r->rx_skb = NULL;
	r->rx_dma = NULL;
	r->tx_skb = NULL;
}

int maple_ring_connect(struct maple_ring *r)
{
	u32 rx_off, tx_off, i;

	/* Negotiated ring offsets from the boot mailbox
	 * (bcm_fld_get_device_bootrecord: SRAM+0xff3c / +0xff40). */
	rx_off = maple_rd(r->bar2, MAPLE_SRAM_DEV_BOOTREC_0);
	tx_off = maple_rd(r->bar2, MAPLE_SRAM_DEV_BOOTREC_1);
	if (!rx_off || !tx_off)
		return -ENOSYS;

	r->rx_tu      = r->ddr_win + rx_off;
	r->rx_bd_info = r->rx_tu + r->rxq_length * MAPLE_TU_SIZE;
	r->rx_owner   = r->rx_bd_info + r->rxq_length * 4;
	r->tx_tu      = r->ddr_win + tx_off;
	r->tx_hdr     = r->tx_tu + r->txq_length * MAPLE_TU_SIZE;
	r->tx_owner   = r->tx_hdr + r->txq_length * 4;

	for (i = 0; i < r->rxq_length; i++)
		if (!post_rx_buf(r, i))
			return -ENOMEM;

	dev_info(r->dev, "rings up: rx@%#x tx@%#x depth rx=%d tx=%d mtu=%d\n",
		 rx_off, tx_off, r->rxq_length, r->txq_length, r->max_mtu);
	return 0;
}

void maple_ring_disconnect(struct maple_ring *r)
{
	free_rx_bufs(r);
	r->rx_tu = r->rx_bd_info = r->rx_owner = NULL;
	r->tx_tu = r->tx_hdr = r->tx_owner = NULL;
}

/* bcmtr_pcie_receive: dequeue RX from the device. Returns nothing; hands the
 * packet skb to the upper layer via maple_rx_handler (TODO: BAL dispatch). */
void maple_ring_rx(struct maple_ring *r)
{
	u32 idx, bd, len;
	struct sk_buff *skb;
	dma_addr_t dma;

	if (!r->rx_bd_info)
		return;

	spin_lock(&r->lock);
	idx = r->current_rx;
	bd = rd_arr(r->rx_bd_info, idx);
	if (bd & MAPLE_RX_OWNED)	/* device still owns it → empty */
		goto out;

	len = bd & 0xffff;
	if (len == 0 || len > r->max_mtu)	/* bad frame: just repost */
		goto repost;

	dma = r->rx_dma[idx];
	skb = r->rx_skb[idx];
	dma_sync_single_for_cpu(r->dev, dma, len, DMA_FROM_DEVICE);

	/* Copy the packet into a fresh skb and hand it to the BAL layer; the
	 * posted buffer is then reposted to the device (swap pattern). */
	if (r->rx_upcall) {
		struct sk_buff *up = alloc_skb(len, GFP_ATOMIC);

		if (up) {
			skb_put_data(up, skb->data, len);
			r->rx_upcall(r->rx_ctx, up);
		}
	}
	skb_trim(skb, 0);

repost:
	/* Swap in a fresh buffer (the vendor allocs a new skb per RX). */
	dma_unmap_single(r->dev, r->rx_dma[idx], r->max_mtu + 0xff, DMA_FROM_DEVICE);
	r->rx_skb[idx] = NULL;
	post_rx_buf(r, idx);
	r->current_rx = (idx + 1) % r->rxq_length;
out:
	spin_unlock(&r->lock);
}

/* bcmtr_pcie_tx_collect: reclaim TX buffers the device has released. */
void maple_ring_tx_done(struct maple_ring *r)
{
	if (!r->tx_owner)
		return;

	spin_lock(&r->lock);
	while (r->tx_skb[r->tx_collect] &&
	       (rd_arr(r->tx_owner, r->tx_collect) == 0)) {
		u32 idx = r->tx_collect;

		/* the skb data was mapped TO device in maple_ring_tx; there is no
		 * separate dma handle stored — it is reconstructed below in TX. For
		 * correctness we keep the skb until reclaim and free it here. */
		dev_kfree_skb_any(r->tx_skb[idx]);
		r->tx_skb[idx] = NULL;
		r->tx_collect = (idx + 1) % r->txq_length;
	}
	spin_unlock(&r->lock);
}

/* bcmtr_pcie_send: post one packet to the Maple. */
int maple_ring_tx(struct maple_ring *r, struct sk_buff *skb)
{
	u32 idx, next, len, bound;
	dma_addr_t dma;
	int ret = 0;

	len = skb->len;
	if (len == 0 || len > r->max_mtu)
		return -EINVAL;
	if (!r->tx_owner)
		return -ENOSYS;

	dma = dma_map_single(r->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(r->dev, dma))
		return -ENOMEM;

	spin_lock(&r->lock);
	idx   = r->current_tx;
	bound = r->txq_length - 1;

	if (rd_arr(r->tx_owner, idx) != 0) {		/* slot busy */
		r->tx_pcie_full++;
		ret = -EBUSY;
		goto unlock;
	}

	wr_arr(r->tx_owner, idx, 1);			/* claim for device */
	wr_arr(r->tx_hdr,   idx, len | (0u << 16));	/* len | chan<<16 (chan=0 for BAL) */

	next = idx + 1;
	if (bound < next)
		next = 0;
	if (r->tx_skb[idx]) {				/* ring full: bump reclaim */
		r->tx_collect = next;
	}
	r->tx_skb[idx] = skb;

	writel(dma - MAPLE_PCIE_WINDOW_OFF, &__tu(r->tx_tu, idx)->pkt_addr);
	writel(0,                      &__tu(r->tx_tu, idx)->pad8);
	writel(len,                    &__tu(r->tx_tu, idx)->length);
	writel((idx == bound) ? MAPLE_TU_LAST : (MAPLE_TU_LAST | MAPLE_TU_NEXT),
	       &__tu(r->tx_tu, idx)->last_next);
	wmb();

	/* Clear the "last" bit on the previously-posted descriptor (chain). */
	if (r->last_tx != idx) {
		u32 prev = r->last_tx;
		writel((prev == bound) ? 0 : MAPLE_TU_NEXT,
		       &__tu(r->tx_tu, prev)->last_next);
		wmb();
	}

	writel(MAPLE_TX_DOORBELL, r->bar0 + MAPLE_BAR0_TX_DOORBELL);	/* kick */
	wmb();

	r->last_tx = idx;
	r->current_tx = next;
unlock:
	spin_unlock(&r->lock);
	if (ret)
		dma_unmap_single(r->dev, dma, len, DMA_TO_DEVICE);
	return ret;
}
