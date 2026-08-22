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

/* platform driver name; also what `ethtool -i` reports and what names the
 * driver's debugfs directory */
#define CA_NI_DRV_NAME		"cortina-ni"

/* peek "window" selector for the peri block (not a DT window index) */
#define CA_NI_PEEK_PERI		0xff
#define CA_NI_PEEK_MAX		64	/* max 32-bit words per peek */
#define CA_NI_GSRAM_MAX		1024	/* max SRAM words per /proc/gsram dump */

struct mii_bus;
struct dentry;
/* forward-declared HERE, before the first prototype that takes one: the full
 * definition is further down, and a struct first named inside a parameter list
 * is a DIFFERENT type scoped to that declaration */
struct cortina_ni;

/*
 * ★★ THE NI_HV INTERNAL-PORT COUNTER BLOCK IS READ-AND-CLEAR: IT HAS EXACTLY
 * ONE READER.
 *
 * cortina-ni-regs.h records the fact ("...unlike the NI_HV_INTPT_* per-stage
 * counters, which are read-and-clear"), and it is measured: reading one of
 * these twice inside a single /proc show returned the count and then a
 * structural 0.  A read-and-clear register cannot have two consumers - whoever
 * reads it first TAKES the count and everyone after sees zero, which is a
 * phantom that reads as a healthy "nothing arrived".  This tree has two such
 * consumers TODAY (cortina_ni_rx_delivery_dump's "DS-NI:" line and the /proc
 * reader's fwd-chain line), which is what edits R4/R5 fix.
 *
 * So the registers are read in exactly ONE place - cortina_ni_nihv_sample() -
 * which folds each sample into a 64-bit driver-side total and hands the TOTAL
 * to every consumer.  /proc/net/cortina_ni_rx, /proc/cortina_l3fe and
 * `ethtool -S` all call it; none of them may readl() these offsets again.
 *
 * ⚠ A caller that wants a DELTA keeps its own snapshot of the TOTAL and
 * subtracts.  Differencing the raw register is wrong twice over: the raw read
 * is already a delta, so `raw - prev_raw` is a delta of deltas.
 *
 * ⚠ The read-and-clear behaviour of this block is itself configurable (NI
 * 0x2010 bit31 is the control, and its setting differs between the stock image
 * and ours), so accumulation is the FAIL-CLOSED choice: if the block were ever
 * left cumulative instead, the totals would OVER-count rather than lose counts,
 * and an over-count is visible (the total races ahead of the raw register)
 * where a lost count is not.  Anything that changes 0x2010 must revisit this.
 * 0x2010 IS in the `ethtool -d` snapshot, so a stock-vs-ours dump diff shows
 * which mode each firmware is in.
 *
 * ★ THESE ARE *OUR* OFFSETS, NOT UPSTREAM'S.  Upstream 7c3c1d4 samples the
 * ELNATH set 0xa9bc/0xa9fc/0xaa10/0xaa3c/0xaa7c.  On Taurus 0xa9bc is a HOLE
 * (fix#81), 0xa9fc/0xaa10 take a synchronous external abort that PANICS the
 * board, 0xaa3c is a BYTE count and 0xaa7c is a header latch.  The five below
 * are the Taurus-NAMED internal-port counters (fix#42/#81) that
 * cortina-ni-rx.c already reads live without faulting.
 */
enum cortina_ni_nihv_cnt {
	CA_NI_NIHV_INTPT_RX,		/* 0xa92c NI_HV_INTPT_RX_PKT_CNT         */
	CA_NI_NIHV_INTPT_MISS_SOP_EOP,	/* 0xa924 INTPT_RX_MISSING_SOP_EOP_CNT   */
	CA_NI_NIHV_INTPT_SHORT_ERR,	/* 0xa928 INTPT_RX_SHORT_ERR_CNT         */
	CA_NI_NIHV_INTPT_TX,		/* 0xa940 NI_HV_INTPT_TX_PKT_CNT         */
	CA_NI_NIHV_XRAM_DMA,		/* 0xaa4c NI_HV_XRAM_DMA_PKT_CNT         */
	CA_NI_NIHV_CNT_COUNT,
};

/*
 * Sample every NI_HV read-and-clear counter ONCE, fold it into the driver's
 * running totals and return those totals in @out.  The ONLY reader of these
 * registers.  Process or softirq context; takes a spinlock, never sleeps.
 * (cortina-ni-ethtool.c)
 */
void cortina_ni_nihv_sample(struct cortina_ni *ni,
			    u64 out[CA_NI_NIHV_CNT_COUNT]);

/*
 * The curated NI-window register snapshot, published through `ethtool -d`.
 * The two tables ARE the snapshot and they stay in cortina-ni-rx.c beside the
 * /proc reader that also prints them, so there is ONE list and it cannot
 * drift; these three accessors are all the ethtool side needs.  _len() is in
 * u32 words - 728 on this board (66 named + 662 swept), NOT upstream's 872.
 */
unsigned int cortina_ni_regdump_len(void);
void cortina_ni_regdump_fill(struct cortina_ni *ni, u32 *buf);
/* name + NI-window offset of dump word @i, so the opaque `ethtool -d` blob can
 * be decoded (surfaced as debugfs cortina-ni/regdump_map). */
void cortina_ni_regdump_entry(unsigned int i, const char **name, u32 *off);

/*
 * Read one per-port NI MAC MIB counter through the indirect ACCESS/DATA pair,
 * at the REAL Taurus addresses (fix#80: RX 0xa1a4/0xa1ac, TX 0xa1b0/0xa1b8).
 * Returns ~0u if the GO poll never cleared, so a stuck access is visible
 * instead of reading as a silent zero.  May sleep (bounded poll): process
 * context only - never call these inside the nihv spinlock.
 * (cortina-ni-rx.c)
 */
u32 cortina_ni_rx_mib_read(struct cortina_ni *ni, u32 port, u32 cnt_id);
u32 cortina_ni_tx_mib_read(struct cortina_ni *ni, u32 port, u32 cnt_id);

/* One DMA-LSO virtual port (VP); M2b uses TXQ 0 of each CPU VP only. */
struct cortina_ni_txq {
	u8		vp;		/* DMA-LSO VP index (CPU n -> VP n+2) */
	u8		txq;		/* fix#68: DMA-LSO queue index within the VP.
					 * Each (vp, txq) is an independent ring with its
					 * own BASE_DEPTH, WPTR/RPTR and scheduler
					 * priority.  Stock puts upstream OMCI on txq 7
					 * ("highest priority"); everything else on 0. */
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
		/* 1 = an extra copy of a FLOODED eth0 frame: it points at the
		 * mapping owned by the LAST descriptor of the same burst, so
		 * there is nothing here to unmap or free.  Fits in the struct's
		 * existing tail padding - no extra memory. */
		u8		dup;
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
	struct delayed_work	dma_dump_work;	/* ★fix#59 periodic DMA-LSO engine sampler */
	bool			announced;
	u64			drop_nomap;
	u64			drop_linearize;
	u64			drop_oversize;
	u64			tx_busy;
	u32			last_word1;	/* last descriptor word1 (spy) */

	/* CPU->LAN egress port binding (cortina-ni-tx.c): DA -> RJ45, learned
	 * from the ingress port of received frames.  One bucket = one atomically
	 * published u64 {mac[47:0], port[50:48], valid[51]}, so the RX learn path
	 * and the TX lookup need no lock.  512 bytes total. */
	u64			lan_fdb[64];
	u32			lan_link;	/* RJ45s with a PHY link, bit = port */
	u64			lan_hit;	/* frames sent to a learned port */
	u64			lan_flood;	/* frames flooded (BC/MC/unknown) */
	u64			lan_dup;	/* extra descriptors a flood cost */
	u64			lan_learn;	/* DA bindings installed/changed */
	u64			lan_flush;	/* table flushes (link set changed) */

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
	/* fix#69: NI CPUXRAM management-FIFO TX path (US OMCI) */
	bool			xram_inited;
	u32			xram_enq;
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

/*
 * ★★★ A pool buffer's USABLE PAYLOAD WINDOW is not its size.
 *
 * These belong beside the other buffer-geometry defines in cortina-ni-regs.h and
 * are kept here only to avoid colliding with concurrent edits to that header -
 * please move them when convenient.
 *
 * QM_DEST_PORTn_PKT_BUF_CFG, which this driver programs with stock's golden
 * 0x18041804, reserves head_room = CA_NI_QM_PKT_BUF_HEAD_UNITS (4 x 16B = 64B,
 * which is what puts HEADER_A at +0x40) at the front of a buffer and tail_room =
 * CA_NI_QM_PKT_BUF_TAIL_UNITS (0x18 x 16B = 384B) at the back.  A 2048-byte pool
 * buffer therefore holds 1600 bytes of frame, in [0x40, 0x680) - which is exactly
 * the window the shipped firmware invalidates and clamps every segment to.
 *
 * The bounds check in cortina_ni_rx_frame() used the BUFFER SIZE (2048) as its
 * ceiling, which is 384 bytes too generous.  A frame whose HEADER_A.pkt_size
 * landed in the gap passed the check and was copied out of memory the hardware
 * never wrote - stale bytes from whatever frame used that buffer before,
 * silently, with no counter moving.  Only the chain path can carry such a frame;
 * until it is enabled the correct outcome is a counted drop.
 */
#define CA_NI_RX_BUF_TAILROOM		(CA_NI_QM_PKT_BUF_TAIL_UNITS * 16)
#define CA_NI_RX_BUF_USABLE_END(bufsz)	((bufsz) - CA_NI_RX_BUF_TAILROOM)

/*
 * Chain bounds.  A malformed chain must cost a counter, never a buffer, a spin or
 * an unbounded skb - this device runs unattended for months, and the shipped
 * firmware is no model here: its chain loop has no segment cap, no timeout and no
 * availability check, so a corrupt pkt_size walks it off the end of the
 * descriptor ring.
 *
 * MAX_LEN is a policy cap, not a hardware limit: it bounds the ONE allocation a
 * chain makes (the skb is sized from the SOP's pkt_size and never grown), and is
 * set for a 1500-byte MTU plus every encapsulation this port carries - QinQ,
 * PPPoE, the 16-byte PON header - with room to spare.  Raise it deliberately if
 * jumbo frames are ever wanted; it is not a guess about the silicon.
 */
/* ★fix#130: 2048 was too small - the QM coalesces a CPU-bound flow's back-to-back
 * frames into ONE chain, so a single HTTP REQUEST larger than this (a browser's
 * full header block) overflows CHAIN_BADTOTAL and is dropped -> the request never
 * completes -> uhttpd hangs ~20s (LuCI unusable in a browser).  16 KiB covers any
 * realistic request; MAX_SEGS below is raised to carry it. */
#define CA_NI_RX_CHAIN_MAX_LEN		16384u
/* Segment cap.  8 is generous for the pools we configure (a 2048-byte buffer
 * offers 1600 usable bytes, so MAX_LEN needs two), and the static_assert in
 * cortina-ni-rx.c is what stops it silently under-covering if a pool is ever
 * shrunk - giving the deep-queue pool stock's 512-byte geometry is exactly why
 * this path exists. */
#define CA_NI_RX_CHAIN_MAX_SEGS		32u	/* ★fix#130: was 8; carry the 16 KiB CHAIN_MAX_LEN (12*1592=19104 >= 16384) */

/*
 * Multi-buffer receive: the arithmetic of one in-flight SOP..EOP chain.
 *
 * Deliberately free of skb and of any hardware reference - it holds only the
 * running length accounting - so ca_ni_chain_step() in cortina-ni-rx.c is a
 * pure function the x86 adversarial suite can drive directly
 * (rtl9607c-test/gen_rx_chain_impl.sh extracts both).  The skb lives beside it
 * in struct cortina_ni_rx_chain, which the imperative shell owns.
 */
struct ca_ni_chain_state {
	u32	total;		/* payload bytes the SOP HEADER_A promised */
	u32	got;		/* payload bytes accounted so far */
	u16	segs;		/* descriptors consumed by this chain */
	bool	open;		/* a chain is in flight on this voq */
};

/* One in-flight chain, PER CPU-port VOQ.  Per-voq and not global: a frame's
 * descriptors are appended to one voq's FIFO in order, while the NAPI budget
 * can cut a chain in half - so the state must survive into the next poll
 * without the frames of the next voq we drain appending into it. */
struct cortina_ni_rx_chain {
	struct sk_buff			*skb;	/* NULL = nothing held */
	/* HEADER_A word 1 of the SOP buffer.  The delivery decision (lspid ->
	 * WAN netdev or eth0) is made when the chain COMPLETES, on a descriptor
	 * that carries no header of its own, and a chain can span NAPI polls -
	 * so the deciding word is kept here rather than read from rx->last_hdra,
	 * which any other frame overwrites. */
	u32				hdra_lo;
	struct ca_ni_chain_state	st;
};

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
	u16			gphy_cal[CA_NI_GPHY_COUNT][CA_NI_RX_GPHY_CAL_REGS]; /* per-bank probe snapshot */
	bool			intf_done;	/* per-port GPHY->MAC interface established (once) */
	/* spy counters (project rule: dump/probe capability is first-class) */
	u64			rearms;		/* link-up RX re-arms */
	u64			recoveries;	/* GPHY reinits fired */
	u32			last_fault;	/* last GPHY fault-latch read */
	u64			irq_hits[CA_NI_RX_NUM_IRQS];
	u64			polls;
	/* per-voq delivered frames: a single flow must land on ONE voq; two
	 * or more climbing during a unidirectional bench = the HW spreads the
	 * flow across voqs and the fixed 0..7 drain order can reorder it (the
	 * packet-order live check on the board) */
	u64			voq_frames[CA_NI_RX_VOQ_COUNT];
	u64			frames;
	u64			bytes;
	u64			swid_frames;	/* headerless (sw_id != 0) frames */
	u64			lspid_cnt[16];	/* M0 de-risk: per-source-jack (HEADER_A.lspid) RX histogram */
	u64			pon_frames;	/* DS PON control frames (0xfff1) handed to the GPON hook */
	u64			wan_frames;	/* DS PON data frames (lspid=PON) delivered to the WAN netdev */
	u64			wan_l3_frames;	/* HW-L3 miss-punt DS frames (lspid=L3_WAN) delivered to the WAN netdev */
	u64			drop_nosop;	/* descriptor without SOP */
	u64			drop_badpa;	/* PA not in our map */
	u64			drop_len;	/* bad frame length */
	/* drop_len's two causes, split so the buffer-window fix is measurable:
	 * a runt is a real bad frame, an oversize one is a frame that does not
	 * fit the buffer's usable window and needs the chain path (rx_chain). */
	u64			drop_runt;	/* len < ETH_HLEN */
	u64			drop_oversize;	/* off + len past the usable window */
	u64			drop_nobuf;	/* refill alloc failed */
	u64			slot_dead;	/* buffer lost (remap failed) */
	/* ---- multi-buffer receive (rx_chain).  One in-flight chain per voq,
	 * plus the malformation ledger: the hardware is not trusted to
	 * terminate a chain, so every way one can go wrong is counted and
	 * logged rather than assumed impossible. */
	struct cortina_ni_rx_chain chain[CA_NI_RX_VOQ_COUNT];
	u32			chain_rest_off;	/* payload offset in a non-first buffer */
	u64			chain_frames;	/* chains assembled and delivered */
	u64			chain_segs;	/* segments those frames consumed */
	u64			chain_max_segs;	/* deepest chain seen */
	u64			chain_abort;	/* partial frames freed (total) */
	u64			chain_reopen;	/* SOP arrived with a chain still open */
	u64			chain_orphan;	/* non-SOP segment, no chain open */
	u64			chain_badtotal;	/* SOP pkt_size out of range */
	u64			chain_toolong;	/* segment cap hit / segments overran total */
	u64			chain_short;	/* EOP with fewer bytes than promised */
	u64			chain_swid;	/* headerless format: geometry unknown */
	/* Per-segment descriptor pkt_size as REPORTED by the hardware, beside the
	 * byte count we DERIVED, for the last chain assembled.  Recorded rather
	 * than judged: the shipped firmware discards this field on a continuation
	 * descriptor, so there is no evidence of what it holds and a mismatch
	 * counter alone would be a phantom.  The first chained frame settles it. */
	u32			chain_dlen_seen[CA_NI_RX_CHAIN_MAX_SEGS];
	u32			chain_dlen_calc[CA_NI_RX_CHAIN_MAX_SEGS];
	u64			chain_dlen_diff;	/* how often the two disagreed */
	/* ★ RX-buffer ownership witnesses (see cpu_pool_push in cortina-ni-rx.c).
	 * stale_buf = the descriptor's pktlen and the buffer's HEADER_A.pkt_size
	 * disagree, i.e. the buffer no longer holds the frame this descriptor was
	 * written for.  It read non-zero on every fragmented datagram with the
	 * hardware-managed pool and MUST stay 0 with the software-owned one.
	 * push_fail = a recycle doorbell timed out; the pool shrinks by one. */
	u64			stale_buf;
	u64			push_fail;
	u64			settle_spins;	/* ★fix#134: total DMA-settle re-read spins */
	/* ★fix#132: deferred recycle FIFO.  The vendor ca-ne never re-posts a
	 * consumed CPU-pool buffer (it refills the free-list with a FRESH buffer and
	 * lets the read one live out its life as an skb).  We keep the monolithic
	 * pool but HOLD each consumed buffer defer_keep frames before returning it,
	 * so a just-copied buffer is never re-DMA'd at reuse-distance 1 (the QM
	 * free-list is a LIFO stack, so an immediate re-post is popped for the very
	 * next frame - the overwrite that corrupts a multi-buffer chain's tail). */
#define CA_NI_RX_DEFER_DEPTH	1024u	/* power of 2 */
	u32			defer_ring[CA_NI_RX_DEFER_DEPTH];
	u32			defer_head;	/* index of the oldest held PA */
	u32			defer_count;	/* PAs currently held out of the pool */
	/* frames delivered out of the hardware-managed DEEP-QUEUE pool (EQ12).
	 * Non-zero is the witness that that pool is populated and the deep-queue
	 * admission path is alive; the GPON downstream punt rides it. */
	u64			dq_frames;
	u64			last_desc;	/* last non-empty descriptor */
	u64			last_hdra;	/* last HEADER_A (host order) */
	/* ★ TEMPORARY DIAGNOSTIC (P3 crc_ntfy tap, rx_crc_tap gate - REVERT
	 * once the T2 hash divergence is pinned): the HW lookup CRC read from
	 * the last matching punted frame's HEADER_CPU meta (+0x48/+0x4C). */
	u64			tap_hits;	/* matching punted frames seen */
	u32			tap_crc32;	/* HEADER_CPU +0x48 (BE) last match */
	u16			tap_crc16;	/* HEADER_CPU +0x4c (BE16) last match */
	u8			tap_cpuflg;	/* HEADER_A cpu_flg of last match */
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
	/* NI_HV read-and-clear counter totals - see cortina_ni_nihv_sample().
	 * The lock is what makes "one reader" true when two files are cat'ed at
	 * the same moment: the sample and the fold are one critical section, so
	 * a count can be taken once and only once. */
	spinlock_t		nihv_lock;
	u64			nihv_total[CA_NI_NIHV_CNT_COUNT];
	/* debugfs root (the `ethtool -d` decode map); NULL when debugfs is not
	 * built in */
	struct dentry		*dbgfs;

#if IS_ENABLED(CONFIG_CORTINA_NI_DSA)
	/* ---- DSA (lan1..lan4) — see cortina-ni-dsa.c ----
	 * ds: the registered switch (its priv points back here).  lan_ndev: the
	 * DSA user netdevs (lan1..lan4), cached at register time so the RX hot
	 * path can deliver a LAN frame straight to its jack's netdev by
	 * HEADER_A.lspid (dsa_to_port() is a list-walk, too costly per frame).
	 * Direct delivery (rather than a METADATA_HW_PORT_MUX md_dst) keeps
	 * skb->offload_fwd_mark = 0, so the Linux bridge software-forwards
	 * LAN<->LAN (this switch forwards through the CPU, not autonomously). */
	struct dsa_switch	*ds;
	struct net_device	*lan_ndev[CA_NI_DSA_USER_PORTS];
#endif
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
 * ★ 2026-08-08 fix#36: dump the MAC->CPU delivery ledger (QM admission, buffer
 * drops, CPU-EPP ring pointers, NAPI progress).  The GPON stats work calls this
 * alongside OMCI-STATS so one boot shows both "did the OLT send" and "how far did
 * the frame get".  Safe to call with the NI not yet probed (no-op).
 */
void cortina_ni_rx_delivery_dump(void);

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
 * Print BOTH directions' CPU-forward counters (US pon_data_enq + DS wan_l3 /
 * wan_pon) as one unambiguous line.  Emitted into /proc/net/cortina_ni_rx AND
 * /proc/net/cortina_ni_tx so a single read can never be mistaken for a
 * whole-device statement when it only covers one direction.
 */
struct seq_file;
void cortina_ni_cpu_fwd_show(struct seq_file *m, struct cortina_ni *ni);

/*
 * US PON DATA (WAN) TX (the GPON WAN netdev's ndo_start_xmit calls this):
 * enqueue one Ethernet frame toward the PON with the data HEADER_A (ldpid =
 * PON, data CoS, fe_bypass) on the DMA-LSO ring; the HW GEM-encapsulates it
 * with the US_PORT_ID of the VoQ it lands in and bursts it on the data
 * T-CONT's grants.  Consumes the skb.  Returns NETDEV_TX_OK always (errors
 * are counted in @ndev->stats).
 */
int cortina_ni_qm_voq_admit(unsigned int qm_voq_idx);
netdev_tx_t cortina_ni_pon_data_tx(struct sk_buff *skb,
				   struct net_device *ndev);

int cortina_ni_tx_probe(struct cortina_ni *ni);
int cortina_ni_rx_probe(struct cortina_ni *ni);

/*
 * CPU->LAN egress port binding (cortina-ni-tx.c), driven from the RX side:
 *  - _learn(): one call per delivered LAN frame - bind @sa to the RJ45 it came
 *    in on.  @lspid is HEADER_A.lspid, which for a LAN frame is the ingress NI
 *    port (the same field the vendor RX demux uses to pick its per-port netdev).
 *  - _link_set(): publish the set of RJ45s that currently have a PHY link, from
 *    the 1 Hz poll (the only context that may take the MDIO mutex).  A change of
 *    the set flushes every binding.
 */
void cortina_ni_lan_tx_learn(struct cortina_ni *ni, const u8 *sa, u32 lspid);
void cortina_ni_lan_tx_link_set(struct cortina_ni *ni, u32 link);

/*
 * Program a static L2FE FDB entry {mac -> ldpid} and return its 13-bit entry
 * index (= the L3FE forward action mac_da_idx / aal-77c egr_lutidx), or -1.
 * Used by the flow-offload next-hop path to resolve the egress DMAC by
 * reference (cortina-ni-rx.c).  `base` = the NI/NE window (cn_l3e->ne_base).
 */
int cortina_ni_l2fe_fdb_add_idx(void __iomem *base, const u8 *mac, u32 ldpid);

/*
 * LOOK UP {mac} in the L2FE FDB - no table write - and report its 13-bit entry
 * index (= mac_da_idx / egr_lutidx) plus the entry's forward-to LDPID (for a
 * LAN NI port: the physical port number).  Used by the DS (WAN->LAN) flow
 * offload leg to resolve the LAN client's next-hop DMAC and egress port from
 * the entry the switch already learned.  Returns -1 when absent.
 */
int cortina_ni_l2fe_fdb_lookup_idx(void __iomem *base, const u8 *mac,
				   u32 *ldpid_out);

/*
 * L3FE main-hash flow engine (nf_flow_table HW offload backend,
 * cortina-ni-flowoffload.c + cortina-l3fe.c).  The probe arms + verifies
 * the engine; any failure is non-fatal - the offload stays disabled and
 * every request falls back to the software path.
 */
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
int cortina_ni_flowoffload_probe(struct cortina_ni *ni);
int cortina_ni_setup_tc(struct net_device *dev, enum tc_setup_type type,
			void *type_data);
/* true only when the hw_l3_fwd experiment is armed AND the L3FE engine init
 * succeeded; the GPON driver keys the DS data-GEM PDC route on it (LDPID
 * L3_WAN into the L3FE vs the proven CPU_0 + FE-bypass delivery). */
bool cortina_ni_hw_l3_fwd_active(void);
/* per-L3-interface T2 admission (CAM + LPB an-mask + pri-6 routed CLS rules,
 * cortina-l3fe.c); re-applied from the link-up cls_init re-run under the
 * hw_l3_fwd gate because the my-MAC/STG0 re-init rewrites the LPB words. */
int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac);
/*
 * LIVE PON data-path identity push (GPON -> offload backend): the GPON driver
 * reports the OLT-provisioned data GEM port-id + the hw T-CONT index whenever
 * it arms the WAN data path (cg_data_try_install) and clears them on teardown
 * (gem_id 0 = no data path).  The L3FE US hit-action needs the LIVE values
 * (GROUP_18 mcgid = gem_id with mc=1; the T-CONT rides the action's t2_ctrl
 * ldpid offset) - never a compiled-in constant.
 */
void cortina_ni_gpon_data_path_set(u16 gem_id, u8 tcont_idx);
/*
 * ★ LIVE DS (PON->host) PDC ROUTE push (GPON -> offload backend).  The DS data
 * GEM's PDC entry is written either as {LDPID L3_WAN, LSPID PON} - into the
 * L3FE, so a DS hash entry can be reached - or as {LDPID CPU_0, FE_BYPASS,
 * NO_DROP}, which delivers straight to the CPU and BYPASSES BOTH forwarding
 * engines.  In the bypass case no DS main-hash entry can ever be hit, no
 * matter how correct it is.  That is a precondition the offload backend cannot
 * observe (the PDC lives in the GPON register window, another module), and
 * without it /proc's DS stage verdict would blame the hash for a route that was
 * switched off - the exact misdetection this project keeps paying for.  So the
 * GPON driver reports it whenever it (re)writes the data-GEM PDC entry:
 * into_l3fe = true for the L3_WAN route, false for CPU_0 + FE-bypass.
 */
void cortina_ni_gpon_ds_route_set(bool into_l3fe);
/*
 * LIVE PPPoE WAN session push (offload backend): report the negotiated PPPoE
 * session id when the WAN runs PPPoE (0 = torn down / IPoE WAN).  US
 * hit-actions then HW-insert the 8-byte PPPoE header via the dedicated egress
 * L3-IF entry; session 0 keeps the proven IPoE action shape byte-identical.
 * First bring-up feeds it via /proc/cortina_l3fe ("pppoe <sess>").
 */
int cortina_ni_wan_pppoe_session_set(u16 session);
/*
 * ★ GAP-2 instrument: inspect a CPU-punted PPPoE session frame (@f = the start
 * of the received Ethernet frame, @len its length) for SELF-CONSISTENCY - PPPoE
 * length vs inner IPv4 total length, inner TCP data-offset plausibility, session
 * id.  Answers "does the DS mangling regression still reproduce" from the frame
 * the CPU actually got, which no register or hit counter can see.  Results in
 * /proc/cortina_l3fe (`pppoe_punt:`).  Off unless
 * cortina_ni.pppoe_punt_check=1, so the RX path pays one predicted branch:
 * ALWAYS test cortina_ni_pppoe_punt_armed() at the call site.
 */
extern bool cortina_ni_pppoe_punt_check;
#define cortina_ni_pppoe_punt_armed()	READ_ONCE(cortina_ni_pppoe_punt_check)
void cortina_ni_pppoe_punt_inspect(const u8 *f, unsigned int len);
/* refresh the backend's probe-time router-MAC shadow when the netdev MAC
 * changes (the HW consumers - FDB/comparator/FIELD-CAM - are re-programmed
 * by cortina_ni_rx_mac_rearm, which is the only caller) */
void cortina_ni_flowoffload_router_mac_set(const u8 *mac);
/*
 * The offload engine's countable quantities, for `ethtool -S`.  A snapshot of
 * the driver's OWN counters and nothing else.
 *
 * ⚠ It deliberately does NOT run the per-bucket age sweep that
 * /proc/cortina_l3fe does: that sweep CONSUMES the engine's age re-arms, so a
 * second consumer of it would steal hits from the 5 s sweep exactly the way a
 * second reader of a read-and-clear register steals counts.  l3fe_hw_hits here
 * is the total the sweep has accumulated, never a fresh consumption of its own.
 */
enum cortina_ni_l3fe_stat {
	CA_L3FE_FLOWS_RESIDENT,		/* GAUGE: entries currently in silicon */
	CA_L3FE_DS_FLOWS_RESIDENT,	/* GAUGE: the DS subset of the above   */
	CA_L3FE_HW_HITS,
	CA_L3FE_US_HITS,
	CA_L3FE_DS_HITS,
	CA_L3FE_HITS_UNATTRIBUTED,
	CA_L3FE_PPPOE_US_HITS,
	CA_L3FE_PPPOE_DS_HITS,
	CA_L3FE_FLOWS_REFUSED,
	CA_L3FE_REFUSED_UNSUPPORTED,
	CA_L3FE_REFUSED_TABLE_FULL,
	CA_L3FE_REFUSED_DUPLICATE,
	CA_L3FE_REFUSED_ERROR,
	CA_L3FE_VLAN_WAN_REFUSED_US,
	CA_L3FE_VLAN_WAN_REFUSED_DS,
	CA_L3FE_VLAN_PPPOE_PROGRAMMED,
	CA_L3FE_VLAN_PPPOE_READBACK_FAIL,
	CA_L3FE_VLAN_PUSH_LEGS,
	CA_L3FE_VLAN_STRIP_LEGS,
	CA_L3FE_STAT_COUNT,
};
void cortina_ni_flowoffload_stats(u64 out[CA_L3FE_STAT_COUNT]);
#else
static inline int cortina_ni_flowoffload_probe(struct cortina_ni *ni)
{
	return 0;
}
static inline int cortina_ni_setup_tc(struct net_device *dev,
				      enum tc_setup_type type, void *type_data)
{
	return -EOPNOTSUPP;
}
static inline bool cortina_ni_hw_l3_fwd_active(void)
{
	return false;
}
static inline int cortina_l3fe_intf_add(void __iomem *ne, const u8 *lan_mac)
{
	return 0;
}
static inline void cortina_ni_gpon_data_path_set(u16 gem_id, u8 tcont_idx)
{
}
static inline void cortina_ni_gpon_ds_route_set(bool into_l3fe)
{
}
static inline int cortina_ni_wan_pppoe_session_set(u16 session)
{
	return -EOPNOTSUPP;
}
#define cortina_ni_pppoe_punt_armed()	false
static inline void cortina_ni_pppoe_punt_inspect(const u8 *f, unsigned int len)
{
}
static inline void cortina_ni_flowoffload_router_mac_set(const u8 *mac)
{
}
#endif
void cortina_ni_rx_open(struct cortina_ni *ni);
void cortina_ni_rx_stop(struct cortina_ni *ni);
void cortina_ni_rx_link_up(struct cortina_ni *ni);	/* phylib link-up hook */
/* re-key the MAC-keyed admission/offload tables (L2FE FDB, my-MAC comparator,
 * PP FIELD-CAM, offload router-MAC shadow) from the current dev_addr; called
 * from .ndo_set_mac_address (netifd applies the factory MAC after the last
 * link-up re-arm).  hw_l3_fwd-gated no-op otherwise. */
void cortina_ni_rx_mac_rearm(struct cortina_ni *ni);
/* internal-GPHY SRAM firmware patch + uC resume; called at link-up (the uC is
 * only held/writable then, not at probe) */

/*
 * The STANDARD counter + register-snapshot interface (cortina-ni-ethtool.c):
 * `ethtool -S <dev>` for every countable quantity and `ethtool -d <dev>` for
 * the curated register snapshot.
 *
 * WHY it exists NEXT TO the /proc nodes rather than instead of them: a test may
 * not depend on a /proc node carrying this driver's name, because the vendor
 * firmware has no such node - so a case reading /proc/net/cortina_ni_rx can
 * only ever BLOCK on stock and the oracle half of the comparison is
 * structurally impossible.  ethtool is served by both firmwares' kernels, so
 * the same case runs on both.  The /proc nodes STAY.
 *
 * ⚠ eth0 ONLY.  The GPON WAN netdev is allocated with alloc_etherdev(0) - priv
 * size ZERO - so attaching these ops to it would read past the allocation.
 */
extern const struct ethtool_ops cortina_ni_ethtool_ops;
/* publish the `ethtool -d` decode key at debugfs cortina-ni/regdump_map; safe
 * (and a no-op) when debugfs is not built in */
void cortina_ni_debugfs_init(struct cortina_ni *ni);

/*
 * DSA (lan1..lan4) — cortina-ni-dsa.c.  _register() is called from the probe
 * AFTER the conduit netdev ("eth0") and the MDIO bus are up; it allocates the
 * per-port metadata dsts and calls dsa_register_switch().  When
 * CONFIG_CORTINA_NI_DSA is off they compile to no-ops so the probe/remove call
 * sites need no #ifdef.
 */
#if IS_ENABLED(CONFIG_CORTINA_NI_DSA)
int cortina_ni_dsa_register(struct cortina_ni *ni);
void cortina_ni_dsa_unregister(struct cortina_ni *ni);
#else
static inline int cortina_ni_dsa_register(struct cortina_ni *ni) { return 0; }
static inline void cortina_ni_dsa_unregister(struct cortina_ni *ni) { }
#endif

#endif /* _CORTINA_NI_H */
