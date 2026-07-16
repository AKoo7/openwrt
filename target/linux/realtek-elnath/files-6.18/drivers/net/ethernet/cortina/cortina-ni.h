/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * shared declarations between the core (probe/MDIO) and the TX datapath.
 */

#ifndef _CORTINA_NI_H
#define _CORTINA_NI_H

#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "cortina-ni-regs.h"

/* peek "window" selector for the peri block (not a DT window index) */
#define CA_NI_PEEK_PERI		0xff
#define CA_NI_PEEK_MAX		64	/* max 32-bit words per peek */
#define CA_NI_GSRAM_MAX		1024	/* max SRAM words per /proc/gsram dump */

struct mii_bus;

/* One DMA-LSO virtual port (VP); M2b uses TXQ 0 of each CPU VP only. */
struct cortina_ni_txq {
	u8		vp;		/* DMA-LSO VP index (CPU n -> VP n+2) */
	__le32		*desc;		/* coherent descriptor ring, 2 words/desc */
	dma_addr_t	desc_dma;
	u16		wptr;		/* next descriptor to fill (SW) */
	u16		finished;	/* oldest un-reclaimed descriptor */
	spinlock_t	lock;		/* xmit vs. reclaim-timer (both BH) */
	struct {
		struct sk_buff	*skb;
		dma_addr_t	addr;
		unsigned int	len;
		/* skb == NULL descriptor kinds: 0 = unused/hole, 1 = PON
		 * header-block (coherent scratch, nothing to free), >= 2 =
		 * PON frame using scratch slot (pon - 2), released by the
		 * reclaim.  With skb set: 1 = PON WAN data skb (TX stats
		 * counted on the WAN netdev at enqueue, not on eth0). */
		u8		pon;
	} slot[CA_NI_TX_RING_SIZE];
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64		enq;
	u64		reclaimed;
};

struct cortina_ni_tx {
	struct net_device	*netdev;
	struct phy_device	*phydev;
	struct cortina_ni_txq	txq[CA_NI_TX_NUM_VPS];
	struct timer_list	reclaim_timer;
	struct work_struct	announce_work;	/* gratuitous ARP on link-up */
	bool			announced;
	u64			drop_nomap;
	u64			drop_linearize;
	u64			drop_oversize;
	u64			tx_busy;
	u32			last_word1;	/* last descriptor word1 (spy) */

	/* US PON control-frame (OMCI) TX: coherent scratch of
	 * CA_NI_PON_TX_SLOTS slots, each {16B DMA-LSO header block @0,
	 * frame @+32}, sent as a 2-descriptor HEADER_A chain on txq[0].
	 * pon_busy (slot bitmap) is guarded by txq[0].lock. */
	void			*pon_buf;
	dma_addr_t		pon_buf_dma;
	u32			pon_busy;	/* CA_NI_PON_TX_SLOTS-bit in-use bitmap */
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64			pon_enq;
	u64			pon_fail;
	u64			pon_data_enq;	/* US WAN data frames enqueued */
};

struct cortina_ni;

/* one RX pool buffer: the skb whose ->data was pushed to the HW pool */
struct cortina_ni_rx_buf {
	struct sk_buff	*skb;
	dma_addr_t	addr;		/* mapped PA of skb->data (128B aligned) */
	s16		hnext;		/* hash chain, -1 = end */
	u8		eqid;		/* which CPU pool (EQ13/EQ14) this slot feeds */
};

struct cortina_ni_rx_irqctx {
	struct cortina_ni	*ni;
	u8			idx;	/* DT interrupt index 0..7 */
};

#define CA_NI_RX_HASH_BITS	11	/* 2048 heads for 880 pool slots */
#define CA_NI_RX_HASH_SIZE	BIT(CA_NI_RX_HASH_BITS)

struct cortina_ni_rx {
	struct cortina_ni	*ni;
	struct net_device	*netdev;
	struct napi_struct	napi;
	__le64			*ring;		/* coherent EPP descriptor ring (8 voqs) */
	dma_addr_t		ring_dma;
	u32			rptr[CA_NI_RX_VOQ_COUNT];	/* SW read ptr per voq, byte offset */
	/* CPU-pool DRAM region (EQ5+EQ6, cpu_eq=0, HW self-populating): the RMU0
	 * admits a CPU-dest frame into a buffer here; NAPI reads it via the phys
	 * offset (bufPA - cpu_dram_dma) and the HW recycles the bid on the EPP
	 * read-pointer advance.  Mapped WC, so no per-frame map/sync. */
	void			*cpu_dram;
	dma_addr_t		cpu_dram_dma;
	/* legacy CPU-push bookkeeping (unused now the CPU pools are DRAM auto-
	 * populated; kept so the /proc spy + struct layout stay stable) */
	struct cortina_ni_rx_buf buf[CA_NI_RX_POOL_SIZE];
	s16			hash[CA_NI_RX_HASH_SIZE];
	unsigned int		nbufs;		/* buffers live in the HW pool */
	u16			pool_target;	/* buffers to keep in the pool */
	bool			qm_up;		/* QM_PHY_PORT_STS.qm_init_done seen */
	struct cortina_ni_rx_irqctx irqctx[CA_NI_RX_NUM_IRQS];
	int			irq[CA_NI_RX_NUM_IRQS];	/* <0 = not mapped */
	/* GPHY fault poll + reinit (stock aal_internal_phy_recovery, 1 Hz) */
	struct delayed_work	recovery_work;
	u16			gphy_cal[CA_NI_RX_GPHY_CAL_REGS]; /* probe snapshot */
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64			rearms;		/* link-up RX re-arms */
	u64			recoveries;	/* GPHY reinits fired */
	u32			last_fault;	/* last GPHY fault-latch read */
	u64			irq_hits[CA_NI_RX_NUM_IRQS];
	u64			polls;
	u64			frames;
	u64			bytes;
	u64			swid_frames;	/* headerless (sw_id != 0) frames */
	u64			pon_frames;	/* DS PON control frames (0xfff1) handed to the GPON hook */
	u64			wan_frames;	/* DS PON data frames (lspid=PON) delivered to the WAN netdev */
	u64			drop_nosop;	/* descriptor without SOP */
	u64			drop_badpa;	/* PA not in our map */
	u64			drop_len;	/* bad frame length */
	u64			drop_nobuf;	/* refill alloc failed */
	u64			slot_dead;	/* buffer lost (remap failed) */
	u64			last_desc;	/* last non-empty descriptor */
	u64			last_hdra;	/* last HEADER_A (host order) */
};

/* /proc/cortina_ni_peek query state (single-user debug tool) */
struct cortina_ni_peek {
	u8	win;		/* CA_NI_WIN_* index, or CA_NI_PEEK_PERI */
	u32	off;		/* byte offset within that window */
	u32	count;		/* number of 32-bit words, 1..CA_NI_PEEK_MAX */
};

/* /proc/cortina_ni_gsram query state (ours-vs-stock internal-GPHY SRAM diff) */
struct cortina_ni_gsram {
	u8	bank;		/* internal-PHY bank 0..CA_NI_GPHY_COUNT-1 */
	u16	start;		/* first SRAM word address */
	u16	count;		/* words to dump, 1..CA_NI_GSRAM_MAX */
};

struct cortina_ni {
	struct device		*dev;
	void __iomem		*win[CA_NI_WIN_COUNT];
	size_t			winsz[CA_NI_WIN_COUNT];	/* mapped size, 0 = absent */
	void __iomem		*peri;	/* hardcoded 4K block @0xf4329000 */
	struct cortina_ni_peek	peek;
	struct cortina_ni_gsram	gsram;
	struct mii_bus		*mii;
	/* per-internal-PHY page-select shadow (reg 0x1f is not a HW reg) */
	u16			gphy_page[CA_NI_GPHY_COUNT];
	/* internal-GPHY SRAM firmware applied, per bank (one-shot per boot) */
	bool			gphy_patched[CA_NI_GPHY_COUNT];
	struct cortina_ni_tx	*tx;
	struct cortina_ni_rx	*rx;
};

/*
 * DS PON control-frame hand-off: the NI CPU-RX path recognizes frames whose
 * 16-byte PON header carries ethertype 0xff,0xf1 (GPON OMCI; vendor
 * CA_PUC_GLOBAL_LNK_TYPE) and hands the OMCI PDU (header already stripped,
 * PDU = frame + 16) to the registered consumer — the cortina-gpon driver —
 * instead of the network stack.  Called from NAPI (softirq) context.
 */
typedef void (*cortina_ni_pon_rx_fn)(const u8 *pdu, unsigned int len);
void cortina_ni_pon_rx_hook_set(cortina_ni_pon_rx_fn fn);

/*
 * US PON control-frame TX (the cortina-gpon responder calls this): wrap the
 * OMCI PDU in the 16-byte PON header and enqueue it on the DMA-LSO ring with
 * the OMCC HEADER_A (see cortina-ni-regs.h).  The HW GEM-encapsulates it
 * onto the OMCC upstream on the next matching BWmap grant.  Safe from
 * process and softirq context.  Returns 0, -ENODEV (TX not up), -EINVAL
 * (bad length) or -EBUSY (ring/scratch full — caller drops, OLT retransmits).
 */
int cortina_ni_pon_tx(const u8 *pdu, unsigned int len);

/*
 * DS PON DATA (WAN) delivery: frames whose RX HEADER_A.lspid = PON are
 * delivered to this netdev (the GPON driver's WAN port) instead of eth0.
 * NULL (default) = fall through to eth0.
 */
void cortina_ni_pon_wan_ndev_set(struct net_device *ndev);

/*
 * US PON DATA (WAN) TX (the GPON WAN netdev's ndo_start_xmit calls this):
 * enqueue one Ethernet frame toward the PON with the data HEADER_A (ldpid =
 * PON, data CoS, fe_bypass) on the DMA-LSO ring; the HW GEM-encapsulates it
 * with the US_PORT_ID of the VoQ it lands in and bursts it on the data
 * T-CONT's grants.  Consumes the skb.  Returns NETDEV_TX_OK always (errors
 * are counted in @ndev->stats).
 */
netdev_tx_t cortina_ni_pon_data_tx(struct sk_buff *skb,
				   struct net_device *ndev);

int cortina_ni_tx_probe(struct cortina_ni *ni);
int cortina_ni_rx_probe(struct cortina_ni *ni);
void cortina_ni_rx_open(struct cortina_ni *ni);
void cortina_ni_rx_stop(struct cortina_ni *ni);
void cortina_ni_rx_link_up(struct cortina_ni *ni);	/* phylib link-up hook */
/* internal-GPHY SRAM firmware patch + uC resume; called at link-up (the uC is
 * only held/writable then, not at probe) */
void cortina_ni_gphy_patch_and_resume(struct cortina_ni *ni);

#endif /* _CORTINA_NI_H */
