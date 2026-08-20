// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * M2c RX datapath: L3QM CPU-EPP descriptor ring + empty-buffer pool + NAPI,
 * making port-0 ingress reach eth0 so ping is bidirectional.
 *
 * Register offsets, bit semantics, descriptor format and init order are
 * hardware facts recovered from the shipped RTL9607F firmware (ca-ne.ko:
 * aal_l3qm_init_cpu_epp, aal_l3qm_init_empty_buffer, aal_l3qm_init_voq,
 * aal_l3qm_enable_rx, aal_l3qm_check_cpu_push_ready,
 * aal_l3qm_set_cpu_push_paddr, ca_ni_fill_eq_buf_pool,
 * ca_ni_fill_empty_buffer_by_rule_zero, ca_ni_rx_napi,
 * ca_ni_rx_napi_get_header_from_64bit_epp, ca_ni_rx_interrupt,
 * aal_l3qm_enable_cpu_epp_interrupt, aal_ni_port_rx_ctrl_set,
 * aal_ple_dft_fwd_set) and cross-checked against the CA8277B public register
 * bit-field definitions (identical fields, some blocks re-based on the 07f,
 * e.g. the per-EQ config 0x61b8 -> 0x6248).
 *
 * RX model: the driver seeds the CPU empty-buffer pools (EQ id 13 primary +
 * EQ id 14 secondary) with DMA-mapped skb data buffers.  For every ingress
 * frame steered to CPU port 0 the HW
 * picks a pool buffer, writes [64B headroom | 8B HEADER_A | (8B HEADER_CPU) |
 * frame] into it and appends one 8-byte descriptor to a 128-entry FIFO ring
 * in coherent DDR, advancing a byte-granular write pointer.  The level
 * interrupt (GIC SPI 0x54 for CPU port 0) fires on FIFO occupancy; NAPI
 * drains descriptors, recovers the skb via a driver-side PA->skb map (the
 * vendor stashes a backpointer inside the buffer; a map avoids the
 * cache-maintenance trap of writing into a FROM_DEVICE-mapped buffer),
 * refills the pool one-for-one and publishes the new read pointer.
 *
 * Frame steering: the FULL FORWARDING-ENGINE path, matching stock's golden
 * working-RX config (devmem-verified).  An ingress frame is looked up by the
 * L2FE and, being an FDB miss, default-forwarded (DLF) to CPU port 0; the
 * L3FE demux map routes that FE output into the CPU-EPP ring.  (The earlier
 * FE-bypass RX_CNTRL byp_dpid shortcut was abandoned: it left hw wptr stuck
 * at 0 on ~half the boots - non-deterministic.  See cortina_ni_rx_steer_init.)
 */

#include <linux/bitfield.h>
#include <linux/crc32.h>	/* ★ TEMP DIAG rx_frag_tap - revert with it */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>	/* ★ TEMP DIAG rx_stack_tap - revert with it */
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <net/net_namespace.h>

#include "cortina-ni.h"

/* DIAGNOSTIC (temporary): when set, link_up skips ALL port-MAC/GPHY reconfig
 * (wrap/static_cfg/autosync/GLB/RXMAC) and does ONLY the CPU delivery chain -
 * leaving the GPHY<->MAC datapath exactly as U-Boot (working) left it.  Tests
 * whether our reconfig is the clobber.  Set via bootargs cortina_ni_rx.rx_skip_portcfg=1 */
static bool rx_skip_portcfg;
module_param(rx_skip_portcfg, bool, 0644);

static bool rx_debug;
module_param(rx_debug, bool, 0644);
MODULE_PARM_DESC(rx_debug, "dump the first received descriptors/frames");

/* ★ TEMPORARY DIAGNOSTIC (rx_frag_tap, 2026-07-27 - REVERT once the IPv4
 * fragment-reassembly defect is pinned).  The board reassembles no fragmented
 * datagram while the vendor image on the same silicon reassembles every one:
 * fragments all arrive (Ip.ReasmReqds exact), yet 2-fragment datagrams never
 * complete and 3-fragment ones complete with a payload that fails the ICMP
 * checksum.  The suspicion is that the frame buffer is reused before the CPU
 * has copied out of it, so two descriptors resolve to the same buffer and the
 * second frame overwrites the first.
 *
 * For the first N IPv4 FRAGMENTS this logs the descriptor's buffer address and
 * pool id, the descriptor length against HEADER_A's, the fragment's IP identity
 * - and then checksums the frame twice: once from the skb we just copied, and
 * again from the pool buffer after a delay.  If both fragments of a datagram
 * report the same pa, the buffer is being handed out under us; if the second
 * checksum differs, the DMA landed after our copy.  Those are different bugs
 * with different fixes, which is why the tap reports which one it is instead of
 * one flag.
 *
 * rx_frag_tap_us=0 is the control: the tap's own delay runs inside NAPI, so a
 * zero-delay run proves the delay is not itself creating what it reports. */
static int rx_frag_tap;
module_param(rx_frag_tap, int, 0644);
MODULE_PARM_DESC(rx_frag_tap,
	"TEMP DIAG: log buffer identity + re-read checksum for the first N IPv4 fragments (0 = off)");

static int rx_frag_tap_us = 20;
module_param(rx_frag_tap_us, int, 0644);
MODULE_PARM_DESC(rx_frag_tap_us,
	"TEMP DIAG: delay before the rx_frag_tap re-read, microseconds (0 = control run)");

static void cortina_ni_rx_frag_tap(struct net_device *ndev, const u8 *buf,
				   u32 off, int len, u64 desc, u32 pa, u32 dlen,
				   u32 hdra_lo, const u8 *copied);

/*
 * ★ TEMPORARY DIAGNOSTIC (rx_ds_tap, 2026-07-28 - REVERT with rx_frag_tap).
 * The open question is whether a GPON DOWNSTREAM frame reaches this driver at
 * all under software pool ownership, and if it does, which pool its buffer
 * came from.  Independent of rx_frag_tap so the fragment ladder stays
 * measurable on the same build.
 *
 * ★ It MUST be sampled before the PON-control and WAN-netdev branches of
 * cortina_ni_rx_frame(), which both return early: a tap placed at the eth0
 * delivery site (where rx_frag_tap sits) can NEVER see a DS frame, so its
 * silence there would mean nothing.  Placed where it is, silence is itself the
 * answer - if LAN frames still print and DS frames do not, the DS frame died
 * before the CPU and the pool is exonerated.
 *
 * Matches on HEADER_A alone (lspid = PON, or ldpid = L3_WAN under the HW-L3
 * route), so it catches DS OMCI control frames, DS data frames, and anything
 * the L3FE punts, whichever branch they would go on to take.
 */
static int rx_ds_tap;
module_param(rx_ds_tap, int, 0644);
MODULE_PARM_DESC(rx_ds_tap,
	"TEMP DIAG: log descriptor+pool identity for the first N GPON downstream frames (HEADER_A lspid=PON or ldpid=L3_WAN), sampled before the PON/WAN delivery branches (0 = off)");

static void cortina_ni_rx_ds_tap(struct net_device *ndev, const u8 *buf,
				 u32 off, int len, u64 desc, u32 pa, u32 dlen,
				 u32 hdra_hi, u32 hdra_lo);

/* ★ TEMPORARY DIAGNOSTIC (P3 crc_ntfy tap, 2026-07-23 - REVERT once the T2
 * hash divergence is pinned).  HASH_INI.crc_ntfy_en=1 makes the T2 lookup
 * write its computed {crc32, crc16} into every punted frame's HEADER_CPU meta
 * (frame buffer +0x48/+0x4C, big-endian).  With this gate on, each punted
 * IPv4/UDP frame to the offload-test sink port (19555) has that HW lookup CRC
 * logged (ratelimited) and exported via /proc/net/cortina_ni_rx, so it can be
 * diffed directly against the driver's install-side SWO CRC (the
 * "manual install ... crc16=" log line) - the diff IS the executed-no-match
 * divergence.  Runs on the coherent CPU-pool mapping (a userspace /dev/mem
 * read of the ring SIGBUSed).  Runtime-flippable, default OFF, off = zero
 * cost beyond one predicted-untaken branch. */
static bool rx_crc_tap;
module_param(rx_crc_tap, bool, 0644);
MODULE_PARM_DESC(rx_crc_tap,
	"TEMP DIAG: log the HW lookup CRC (HEADER_CPU crc_ntfy) of punted UDP:19555 frames");

/* ★ TEMPORARY DIAGNOSTIC (P3 packet-STACK tap, 2026-07-23 - REVERT together
 * with rx_crc_tap once the T2 hash divergence is pinned).  For each punted
 * transit probe frame (innermost IPv4/UDP dport 19555 - matched through any
 * VLAN/QinQ/PPPoE/IP-in-IP layering, unlike the fixed-offset crc tap, so a
 * tag-shifted probe still fires), log side by side:
 *   (a) the SW-decoded protocol stack of the actual frame bytes (explicit
 *       byte math), flagging any deviation from the expected plain
 *       {Ethernet -> IPv4 -> UDP}: a per-port VLAN tag the L2FE did NOT
 *       untag, a QinQ double-tag, PPPoE, IP-over-IP;
 *   (b) the HW's OWN parse of the frame: the 128-byte L3FE HDR_I descriptor
 *       read from the L3FE debug snapshot mux (stock aal_l3fe_glb_dbg_get;
 *       tap 2 = "HDR_I between STG1 ~ T2 (Hash)" = the exact T2 lookup-key
 *       input), decoded at the a_cut bit offsets recovered from the stock
 *       ca-ne.ko l3fe_debug_dump_hw_hdr_i_a_cut field extractions.
 * A divergence between (a) and (b) - or an unexpected tag in (a) - IS a
 * hash-divergence mechanism: the HW parsed/hashed different bytes than the
 * driver's install-side HDR_I build assumed.  The debug mux is a LIVE
 * snapshot of the LAST frame through STG1->T2 (not a per-frame meta), so the
 * dump prints a same-frame check (HW l4_dp vs 19555); a STALE verdict means
 * another frame raced the mux between the punt and this NAPI read - re-read
 * under quieter traffic.  Ratelimited; raw HDR_I words hexdumped on the
 * first hits for offline decode.  Runtime-flippable, default OFF, off = one
 * predicted-untaken branch. */
static bool rx_stack_tap;
module_param(rx_stack_tap, bool, 0644);
MODULE_PARM_DESC(rx_stack_tap,
	"TEMP DIAG: dump SW frame-stack vs HW HDR_I parse for punted UDP:19555 frames");

/* ★ Decoupled datapath bring-up (default ON, 2026-07-22).  The full datapath
 * bring-up (dphy_rst release glb+0xa0, all-bank GPHY line<->system SRAM patch,
 * GPHY<->MAC wrapper, FE path) normally runs ONLY from the connected PHY's
 * link-up hook.  This driver connects ONE PHY (phy_find_first) to eth0; on a rig
 * where that PHY's port has no cable, link-up never fires, so the internal
 * GPHY/datapath stays HELD IN RESET and NO physical LAN port (eth0.2..5)
 * ingresses (host ARP flood -> rx=0) even though the L2FE forwarding tables
 * byte-match stock.  Fix: also drive the bring-up from the 1 Hz recovery_work
 * (which runs regardless of eth0's link) until every GPHY bank is patched, so
 * the LAN ports come up independent of eth0's PHY.  Set =0 to restore the old
 * link-up-only behaviour for an A/B. */
static bool rx_decoupled_bringup = true;
module_param(rx_decoupled_bringup, bool, 0644);
MODULE_PARM_DESC(rx_decoupled_bringup,
		 "drive the datapath bring-up from the 1Hz poll so LAN ports come up without an eth0 cable (default on)");

/* ★ build88: A/B which CPU-EPP ring NAPI reads descriptors from - 0 = the LOW ring at
 * PADDR(0x7200)=0x0bc48000 (default/current); 1 = the HIGH ring at PADDR_HI(0x7220)=
 * 0x0bc4a000.  The engine advances wptr but the LOW ring reads poison, so the engine
 * may write descriptors into the HIGH ring; set rx_ring_hi=1 to test reading it. */
static bool rx_ring_hi;
module_param(rx_ring_hi, bool, 0644);
MODULE_PARM_DESC(rx_ring_hi, "NAPI reads the HIGH (PADDR_HI 0x0bc4a000) CPU-EPP ring instead of the LOW one");

/* ★ FBM pool ENABLE/FILL/PRELOAD gate - DEFAULT OFF because it CRASHED (build24):
 * enabling+preloading the FBM pool makes it reference/DMA our CPU-pool region, which
 * is NOT a truly reserved coherent window (no reserved-memory DT node excludes it), so
 * the kernel slab that lives there gets trashed -> paging-fault panic in the next
 * kmalloc.  Also the raw-phys POOL+0x40 push did not register (outstanding stayed 0),
 * so it is the wrong feed interface anyway.  Keep the FBM MAPPED + config-only by
 * default (bootable); flip this on only once the reserved region + correct buffer-feed
 * are confirmed.  Set via bootargs cortina_ni_rx.fbm_enable=1 */
static bool fbm_enable;
module_param(fbm_enable, bool, 0644);
MODULE_PARM_DESC(fbm_enable, "enable+fill+preload the FBM pool (DANGER: crashes until reserved-mem+feed fixed)");

/* (build27's qm_reset dropped: 0x6988 bit30=1 proved the QM is NOT held in reset - the
 * wall is the NI->L3QM handoff flow-control, not a reset.  The GLB+0xa0/0x6988 defines
 * are kept as the correct L3QM init-done readback.) */

/* ★★ build33 - route the CPU frame to ldpid 0x32 the STOCK way.  Stock devmem
 * ground-truth: PDPID_MAP[0x19]=0x0D (L3-LAN dead-end), [0x32]=0x08 (QM); stock's
 * WORKING CPU frame resolves to ldpid 0x32, never 0x19.  Ours had an MCE[0x19]
 * member (build14/17) that resolved the DFT_FWD=0x1832 frame to the mcgid ldpid 0x19,
 * and a build15 bodge [0x19]->0x08 to force it to QM - but the NI per-LDPID L3QMRX
 * demux (0xa180-0xa1c0) and the deep-queue demux are indexed by the RAW ldpid, so
 * entry[0x19] != entry[0x32]: the frame at ldpid 0x19 hit a demux slot that never
 * crosses into L3QM (l2tm_tx climbed while ni2qm_rx/0xa9fc stayed 0), while stock's
 * frame on ldpid 0x32 rides the demux slot that routes to L3QM.  (ES egress port =
 * PDPID from the ARB map at 0x166c/0x1670, NOT the raw ldpid - so the divergence is
 * the per-ldpid demux, not the ES port.)  Fix (build34, HW-PROVEN):
 * REDIR_LDPID[0x19]->X forces the frame to unicast ldpid X (mcgid 0x19 intercepted
 * before MC replication).  On Elnath an empty MCE[0x19] alone does NOT fall through
 * to the raw ldpid 0x32 - it still resolves to the mcgid ldpid 0x19 and, with
 * [0x19]=0x0D (L3-LAN), floods/loops (l2tm_tx storm 124M->190M).  The baked redirect
 * (redir_cpu_ldpid != 0) fires before PDPID[0x19] so there is no stray-0x19 L3-LAN
 * flood regardless of PDPID[0x19]'s value.
 *
 * ★★ build69 (THE last-ring fix): redirect target 0x32 -> 0x10 (CPU_0).  Stock's
 * admitted-frame header RMU0_RX_HDR_INFO0(0x6904)=0x80000010 = dest LDPID 0x10 (CPU0)
 * with bit30(deep_q) CLEAR -> the CPU-EPP64 ring @0x7000 that NAPI polls.  Our old
 * ->0x32 gave 0xc0000020: PDPID[0x32]=0x08==dbuf_dpid(8) => deep_q=1 => dest 0x20
 * (CPU_MQ_0) => the DIFFERENT CPU-EPP256 ring, so the 0x7000 wptr never advanced.
 * ->0x10 makes PDPID[0x10]=0x09 (CPU, != dbuf_dpid) => deep_q=0 => dest 0x10 =>
 * CPU-EPP64, matching stock.  (build14's "->0x10 never reached RMU0" was the
 * 0x212c=0x88888888 L3QM dead-end, FIXED in build68 (0x212c=identity); a dest-0x10
 * frame now reaches RMU0/CPU-EPP64 exactly like stock.)  redir_cpu_ldpid=0 A/B-tries
 * the pure MC_FIB[0x19]=0x0b flood path instead (no redirect).
 *
 * ★★★ build70 (stock-exact, tier-1 golden): DEFAULT 0 = NO REDIR.  Stock's
 * REDIR_LDPID[0x19]=0 (empty) - it has no REDIR_LDPID redirect at all.
 * ★ Corrected 2026-07-15: there is NO MC_FIB flood either (the real MC_FIB is at
 * ACCESS 0x1644 and stock keeps it EMPTY; "MC_FIB@0x1634" was a different table).
 * Stock's CPU delivery is: DFT_FWD 0x1832 (redirect lookup-miss to mc_group_id
 * 0x19 = L3_LAN) -> RMU -> L3FE -> L3-CLS ethertype trap -> CPU_0.  Set
 * redir_cpu_ldpid=0x10 only to A/B fall back to the unicast-redir workaround. */
static u8 redir_cpu_ldpid;	/* 0 = stock (no redir; DFT_FWD->L3_LAN->L3FE->CLS trap delivers the CPU copy) */
module_param(redir_cpu_ldpid, byte, 0644);
MODULE_PARM_DESC(redir_cpu_ldpid, "REDIR_LDPID[0x19]->this ldpid (0=stock: no redir, L3FE/CLS trap path; 0x10=A/B unicast-redir workaround)");

/* ★★ build41: ARB_CTRL.dbuf_dpid = the deep_q TRIGGER (bits[7:4]; a resolved PDPID ==
 * this => deep_q=1 => deep-queue/DQSCH path).  Stock = 8 (== the QM pdpid our CPU frame
 * resolves to).  Our deep_q=1 path NEVER crossed into L3QM (0xa9fc=0) despite EVERY
 * register matching stock across ~25 builds -> our clean-room DQSCH has a residual flaw
 * no register-diff finds.  Default 0xf = disable the trigger (no pdpid == 0xf) so the
 * pdpid-0x08 CPU frame is deep_q=0 and takes the NORMAL BM-dequeue path (0x2124=0x88888888
 * all->L3QM + 0x212c, both proven stock-match) -> TM-port 8 -> L3QM.  Set arb_dbuf_dpid=8
 * to restore stock's deep_q=1 for A/B. */
static u8 arb_dbuf_dpid = 0x08;	/* build68 STOCK: ARB_CTRL 0x89c71c82 (bits[7:4]=8); 0xf gave 0x89c71cf2 */
module_param(arb_dbuf_dpid, byte, 0644);
MODULE_PARM_DESC(arb_dbuf_dpid, "ARB_CTRL deep_q trigger pdpid (0xf=disable deep-queue/use normal path, 8=stock deep_q)");

/* ★ build101 EXPERIMENT (prior-session never-tested fix): route L3_LAN (ldpid 0x19, where
 * the flooded LAN broadcast resolves) DIRECTLY to a CPU port via PDPID_MAP[0x19], bypassing
 * the L3FE/CLS trap.  Default 0x00 = the prior-session "CPU port 0" value (DIVERGENT from
 * stock's 0x0d; note this session's vendor RE reads pdpid 0x00 as NI-wire-port0 and CPU as
 * 0x09 - so if 0x00 egresses a wire, A/B 0x09/0x0d at runtime).  0x0d = stock L3_LAN. */
static u8 pdpid_l3lan = 0x0d;	/* ★ STOCK L3_LAN->L3FE (entry fix); paired with the unconditional pool seed (drain fix) so frames enter L3FE AND RMU0 has buffers to admit them. */
module_param(pdpid_l3lan, byte, 0644);
MODULE_PARM_DESC(pdpid_l3lan, "PDPID_MAP[0x19] L3_LAN route: 0x00=CPU-port0 (prior-session test), 0x09=CPU(L2FE), 0x0d=stock L3_LAN");

/* ★★ build52 single-variable test: the CPU-EPP descriptor-ring AXI attr.  The coherent
 * CPU_EPP attr (0x12008060, ACE) got the writeback REJECTED (0x611c bit22, wptr stuck 0)
 * even with `dma-coherent` on the NE DT node - the SoC interconnect isn't routing the
 * coherent write.  Default 1 = write the ring entries with the POOL's NON-coherent attr
 * (0x04000010) instead, exactly like the working RMU0 frame-pool writes; judge by
 * bit22-clears + wptr-advances (the cached ring dump reads stale on a non-coherent write).
 * =0 restores the coherent attr (A/B control). */
static bool ring_noncoh = true;
module_param(ring_noncoh, bool, 0644);
MODULE_PARM_DESC(ring_noncoh, "CPU-EPP ring AXI attr (1=non-coherent DDR_POOL 0x04000010; 0=coherent CPU_EPP 0x12008060)");

/*
 * ★★★ RX-buffer OWNERSHIP.  CFG2.cpu_eq (bit3) selects which side owns a CPU
 * pool buffer:
 *   0 = hardware-managed - the QM populates the pool itself from
 *       CFG0.phy_addr_start and frees the bid at the EPP descriptor writeback;
 *   1 = software-owned  - the buffer belongs to the CPU until the driver
 *       stages it back through the CPU_PUSH_PADDR doorbell (the vendor model:
 *       its CPU pools are cpu_eq=1 and it re-pushes one batch per NAPI poll).
 *
 * We shipped cpu_eq=0 on the belief that the QM reclaims a bid only when NAPI
 * advances the EPP read pointer.  BOARD-MEASURED 2026-07-27, that belief is
 * false.  Two frames admitted ~1 us apart - the two fragments of one IPv4
 * datagram, which is the only traffic on this device that puts two frames
 * back-to-back into the CPU punt path - were both DMA'd into the SAME buffer
 * (pa=0x09400800) with no read-pointer advance in between: the descriptor for
 * frame 1 resolved to a buffer already overwritten by frame 2 (descriptor
 * pktlen 1530 against HEADER_A.pkt_size 570).  The stack then received two
 * byte-identical copies of fragment 2 - an exact duplicate, which
 * ip_frag_queue() drops on its IPFRAG_DUP path WITHOUT incrementing any
 * counter - so the datagram never completed and expired 30 s later.  When the
 * overwrite lands mid-copy instead, the frame keeps a valid header and gains a
 * corrupt tail, which completes reassembly and fails the L4 checksum.
 *
 * A deeper pool cannot fix this: the free list behaves as a stack, so a bid
 * freed at writeback is handed straight back out - measured, the two frames
 * collided on one buffer even though 512 are configured per pool.  Only
 * transferring ownership to the CPU closes the window, by construction.
 *
 * =0 restores the previous hardware-managed pool for an A/B (and as the
 * fallback if a board ever fails to seed).  Read-only: the pool model is
 * chosen once, at probe.
 */
/* ★ 2026-07-28 DEFAULT OFF pending one open regression.  Software ownership
 * demonstrably fixes IPv4 fragment reassembly - every rung 5/5, matching the
 * vendor image, where the hardware-managed pool gave 0/5, with stale_buf and
 * push_fail both 0.  But on the same boot the WAN downstream punt stopped
 * delivering entirely: gpon0 TX climbs (DHCP DISCOVERs go out) while its RX
 * stays at exactly 0 packets, so no lease is ever obtained, even though the LAN
 * path and O5/OMCI are healthy.  A defect traded for a defect is not a fix, so
 * this ships off until the downstream punt path is understood - most likely a
 * pool or return path the recycle does not cover (the WAN netdev branch of
 * cortina_ni_rx_frame() is a separate delivery path from the eth0 one, and the
 * deep-queue DS punt may draw from a pool other than EQ5/EQ6). */
/* ★★ DEFAULT ON since 2026-07-28 - this is the buffer-ownership model the
 * hardware actually requires, and it is now proven end to end on the board.
 *
 * With cpu_eq=0 the QM frees a buffer at descriptor writeback, so the CPU never
 * owns it: a second frame arriving before NAPI drains lands on the SAME buffer.
 * Measured directly - every frame at pa=09400800, fragment 1's descriptor
 * (dlen=1530) resolving to a buffer already holding fragment 2
 * (hdra_pkt_size=570), two byte-identical copies of fragment 2 under one IP id
 * => IPFRAG_DUP => the datagram never completes.  IPv4 fragment reassembly was
 * therefore broken outright (ping -s 2000/3000 = 0/5) while the vendor image on
 * the same silicon reassembled every one.
 *
 * cpu_eq=1 + CPU_PUSH_PADDR recycle removes the window rather than narrowing
 * it: the QM cannot reclaim a buffer until we re-push it, which happens after
 * skb_put_data.  Result: 150/150 on every rung of the suite's fragment ladder,
 * local and transit, matching the vendor numbers, with stale_buf and push_fail
 * both 0.
 *
 * ★ It only works together with the dest-port 16..47 gate below: the per-GEM
 * PDC map stamps DATA GEMs with ldpid 0x18 = L3_WAN = 24, which lands in that
 * range, and a forwarding-engine consumer cannot draw from a cpu_eq=1 pool.
 * Leaving that range on the software-owned pools killed the WAN downstream punt
 * completely (gpon0 RX at exactly 0 from boot) while the LAN punt and DS OMCI -
 * which enter at dest port 0 - stayed healthy.  Set to 0 to restore the old
 * hardware-managed model, fragment defect included. */
static bool cpu_pool_push = true;
module_param(cpu_pool_push, bool, 0444);
MODULE_PARM_DESC(cpu_pool_push,
	"CPU RX pool ownership: 1 = software-owned (cpu_eq=1 + CPU_PUSH_PADDR recycle, vendor model; fixes IPv4 fragment reassembly but currently breaks the WAN downstream punt), 0 = hardware-managed self-populating pool (default; collides a back-to-back frame pair onto one buffer)");

/*
 * ★★ MULTI-BUFFER RECEIVE (the SOP..EOP descriptor chain).  Default OFF until a
 * chained frame has been assembled on hardware.
 *
 * A pool buffer's usable payload window is NOT its buffer size: the QM reserves
 * head_room at the front and tail_room at the back (see
 * CA_NI_RX_BUF_USABLE_END), leaving 1600 bytes of a 2048-byte buffer.  A frame
 * longer than that window is split across buffers - first descriptor SOP, last
 * EOP, the ones between neither - and the vendor firmware assembles it.  We
 * never did: cortina_ni_rx_frame() reads SOP only, so a chain's first segment
 * was delivered short and its continuations counted as drop_nosop.
 *
 * Why it matters beyond long frames: it is what makes a SMALL-buffer pool
 * viable.  The deep-queue pool has to stay hardware-managed on this silicon
 * (CA_NI_RX_DQ_CFG2), and the only reason it also has to be 2048B is that we
 * cannot chain; with chaining, stock's own 512B geometry becomes reachable.
 *
 * With this off the receive path is the one that shipped: the test below
 * short-circuits on the parameter before reading EOP, so a descriptor's
 * treatment is decided exactly as before.
 */
static bool rx_chain;
module_param(rx_chain, bool, 0444);
MODULE_PARM_DESC(rx_chain,
	"assemble multi-buffer frames from the SOP..EOP descriptor chain (1) or treat a non-self-contained descriptor as today - deliver a SOP short, drop a continuation as nosop (0, default)");

/*
 * Whether a CONTINUATION buffer repeats HEADER_A at +0x40, or starts its payload
 * there.  ESTABLISHED: it starts its payload there - a continuation carries no
 * header of any kind, so all 1600 bytes of its window are frame bytes.
 *
 * Three independent readings of the shipped firmware agree, recovered by two
 * separate passes over ca-ne.ko:
 *   - both header parsers gate the HEADER_A read on the descriptor's SOP bit
 *     (ca_ni_rx_napi_get_header +0xc8, ca_ni_rx_napi_get_header_from_64bit_epp
 *     +0xa0), so no SOP means no header;
 *   - the continuation reader (ca_ni_rx_napi_read_epp_64bit_mode) contains no
 *     header parse at all - it invalidates buf+0x40 for 1600 bytes and returns;
 *   - the pointer arithmetic differs decisively between the two cases: the first
 *     buffer's payload is buf + 0x40 + K with K the 8- or 16-byte header block,
 *     while a continuation does skb_push(64) with NO +K adjustment and then puts
 *     min(remaining, 1600) bytes (ca_ni_rx_napi +0x640/+0x64c and the same shape
 *     in ca_ni_rx_napi_fbm).
 * The same 1600-byte window is independently confirmed by our own register
 * config: QM_DEST_PORTn_PKT_BUF_CFG reserves 64 bytes of head and 384 of tail in
 * a 2048-byte buffer, which is 1600 - see CA_NI_RX_BUF_USABLE_END.
 *
 * The parameter survives only as a one-bootarg A/B if a future chip's QM turns
 * out to prepend a header per buffer; it is not a placeholder for a guess.
 */
static bool rx_chain_rest_hdra;
module_param(rx_chain_rest_hdra, bool, 0444);
MODULE_PARM_DESC(rx_chain_rest_hdra,
	"a continuation buffer of a chain repeats HEADER_A at +0x40, so its payload starts at +0x48 (1), or its payload starts at +0x40 (0, default, and what the shipped firmware does)");

/*
 * ★★ The seed count is deliberately NOT a constant of its own: each pool must
 * be handed exactly the CFG1.total_buf it was configured with, so the two are
 * derived from one source and cannot drift.
 *
 * BOARD-MEASURED 2026-07-27, why this matters: a first cut seeded a separate
 * 256 per pool while total_buf stayed 512, and the QM then sat permanently in
 * "want buffers" - inactive_bid_cntr (0x6388+eqid*4, the vendor's
 * aal_l3qm_get_inactive_bid_cntr, documented as the number of buffers the pool
 * is SHORT) read 256 on BOTH pools under software ownership and 0 under
 * hardware ownership.  512 - 256, exactly, and a difference the fix itself
 * introduced.  Only the floor below stays a constant: it is the failure
 * threshold, not a capacity.
 */
#define CA_NI_RX_PUSH_SEED_MIN		64
/* A full seed must fit inside the sub-region the PA->VA math maps, whatever a
 * future total_buf or buffer size is set to. */
static_assert(CA_NI_RX_EQ_TOTAL_BUF * CA_NI_RX_CPU_POOL0_BUFSZ <=
	      CA_NI_RX_CPU_POOL0_BYTES,
	      "CPU pool0: total_buf x buffer_size overflows its sub-region");
static_assert(CA_NI_RX_CPU_POOL0_BYTES +
	      CA_NI_RX_EQ2_TOTAL_BUF * CA_NI_RX_CPU_POOL1_BUFSZ <=
	      CA_NI_RX_CPU_DRAM_SIZE,
	      "CPU pool1: total_buf x buffer_size overflows the reserved region");
/* Per-poll recycle batch.  The NAPI budget is clamped to this so a poll can
 * never consume more buffers than we can hand back - an overflow would leak
 * buffers out of the pool until RX starves. */
#define CA_NI_RX_RECYCLE_MAX		64
/*
 * ★★★ DEEP-QUEUE pool (EQ12), required by the software-owned CPU pools.
 *
 * These belong beside the other pool addresses in cortina-ni-regs.h and are
 * kept here only to avoid colliding with concurrent edits to that header -
 * please move them when convenient; nothing else references them.
 *
 * WHY a second pool exists at all.  Two consumers reach the CPU through the QM:
 * the DIRECT punt and the DEEP-QUEUE punt (dest ports 8..15, deep_q=1).  Our
 * config collapsed both onto EQ5/EQ6.
 *
 * ★ The mechanism of that collapse, stated correctly: DEST_PORT_EQ_CFG.prof_sel
 * is CA_NI_QM_DEST_PORT_PROF_SEL = GENMASK(3, 0), a FOUR-bit field at bit 0
 * that indexes EQ_PROFILE DIRECTLY (0..15) - it is NOT a 3-bit field, and there
 * is no masking.  So the deep-queue dest ports, programmed 0x0D, select
 * EQ_PROFILE(13), and profile 13 is written with {EQ5, EQ6}.  Evidence, three
 * ways that agree: the field definition; the live readback prof13(0x615c) =
 * 0x00000065 = {eqp0=5, eqp1=6} alongside destp8 = destp15 = 0x0D; and stock's
 * own pairing destp8 = 0x0C with prof12 = 0xEC = {EQ12, EQ14}, which is the
 * documented deep-queue pool pair and only lines up under direct indexing.
 * (A "3-bit, so 0x0D & 7 = profile 5" reading elsewhere in this file reaches
 * the same pool by accident, because the 0..7 loop happens to fill profile 5
 * with {EQ5, EQ6} too.  It is wrong, and it made a coincidence look like a
 * derivation.)  Stock keeps the two consumers SEPARATE: prof12 = {EQ12, EQ14},
 * prof13 = {EQ13, EQ14}.
 *
 * That collapse is fine while both pools are hardware-managed, and fatal once
 * they are not: this chip's deep-queue enqueue CANNOT consume a CPU-pushed
 * buffer - it needs a self-populated DRAM one (A/B-proven earlier on EQ12, see
 * CA_NI_QM_EQ12_CFG2).  BOARD-MEASURED 2026-07-28: with EQ5/EQ6 flipped to
 * cpu_eq=1, NO downstream frame of any kind reached the driver (not data, not
 * OMCI control; 30-frame DS tap silent, zero OMCI cfg messages) while the LAN
 * path and the fragment ladder were perfect - and 40 samples of rmu0_rx_hdr
 * taken on the WORKING image with no LAN traffic in flight showed DS producing
 * deep_q=1 admissions in 17 of 40.  So the deep queue lost its only DRAM-backed
 * pool.  Giving it its own hardware-managed pool is what lets the direct punt
 * be software-owned.
 */
#define CA_NI_RX_DQ_POOL_OFF		CA_NI_RX_CPU_DRAM_SIZE
#define CA_NI_RX_DQ_POOL_PHYS \
	(CA_NI_RX_CPU_POOL_PHYS + CA_NI_RX_DQ_POOL_OFF)
/* One mapping covers both CPU pools AND the deep-queue pool, so the PA->VA
 * math and the bounds check in cortina_ni_rx_frame() need no special case. */
#define CA_NI_RX_MAP_SIZE \
	(CA_NI_RX_CPU_DRAM_SIZE + CA_NI_RX_DQ_DRAM_SIZE)
/* ★ NOT stock's CFG2 0x0000ff02.  Buffer-size index 2 is 512B on this die, and
 * a 512B buffer cannot hold a full frame plus the 64B head and 384B tail the QM
 * reserves - stock copes because it has a multi-buffer receive path
 * (aal_l3qm_*_jumbo_buf, SOP..EOP chain) and we do not; our copy-break assumes
 * one buffer per frame.  So index 4 = 2048B, matching the CPU pools, and
 * cpu_eq=0 + refill_en=0 kept from stock. */
#define CA_NI_RX_DQ_CFG2		0x0000ff04u
/* The EQ profile the deep queue gets.  prof_sel is a 4-bit DIRECT index, so any
 * of 0..15 is reachable; use 12 - stock's own deep-queue profile index, and the
 * only choice that collides with nothing this driver writes.  Profiles 0..7 are
 * filled wholesale by a loop in eq_init and 13 is the direct punt's, so a low
 * index would need write-ordering care and would also change what profile means
 * for dest ports 1..7, which we never program.  12 needs neither. */
#define CA_NI_RX_DQ_PROFILE_SEL		CA_NI_RX_EQ12_PROFILE	/* = 12 */
/* the deep-queue pool is hardware-managed, so BOTH halves of the profile point
 * at it - there is no software-owned overflow reserve for this consumer */
#define CA_NI_RX_DQ_PROFILE_VAL \
	(FIELD_PREP(CA_NI_QM_EQ_PROF_EQP0, CA_NI_RX_EQ12_ID) | \
	 FIELD_PREP(CA_NI_QM_EQ_PROF_EQP1, CA_NI_RX_EQ12_ID))
static_assert(CA_NI_RX_EQ12_TOTAL_BUF * 2048u <= CA_NI_RX_DQ_DRAM_SIZE,
	      "deep-queue pool: total_buf x buffer_size overflows its region");

/* Ready-poll bound for the NAPI recycle path.  Deliberately far tighter than
 * CA_NI_RX_PUSH_TIMEOUT_US: that one bounds the bring-up seed in process
 * context, whereas this runs in softirq once per recycled buffer, so a whole
 * batch at the generous bound would spin ~64 ms inside one poll.  A stuck gate
 * must degrade (push_fail, pool shrinks, logged) rather than hang the CPU. */
#define CA_NI_RX_PUSH_TIMEOUT_NAPI_US	20

/* head_room_rest == head_room_first and tail_room_rest == tail_room_first in the
 * value we program, so one window serves both the first and the continuation
 * buffers (see CA_NI_RX_BUF_USABLE_END in cortina-ni.h).  Should they ever be
 * programmed apart, the chain path must compute the first and continuation
 * windows separately - this assert is the tripwire. */
static_assert(CA_NI_QM_PKT_BUF_HEAD_UNITS * 16 == CA_NI_RX_BUF_HEADROOM,
	      "pkt-buf head_room does not put HEADER_A at CA_NI_RX_HDRA_OFF");
static_assert(CA_NI_RX_CHAIN_MAX_SEGS *
	      (CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ) -
	       CA_NI_RX_FRAME_OFF) >= CA_NI_RX_CHAIN_MAX_LEN,
	      "chain: MAX_SEGS cannot carry MAX_LEN out of a CPU-pool buffer");
/* The shipped firmware hardcodes this same window as the literal 1600 in both of
 * its receive polls; our derivation from the pkt-buf config must agree with it on
 * the 2048-byte pools, and unlike the literal it stays correct if a pool is ever
 * given a different buffer size - which is the point of the chain path. */
static_assert(CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ) -
	      CA_NI_RX_BUF_HEADROOM == 1600,
	      "CPU-pool usable window is not the firmware's 1600 bytes");

static inline void __iomem *ni_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_NI];
}

static inline void ni_rmw(struct cortina_ni *ni, u32 off, u32 clr, u32 set)
{
	writel((readl(ni_base(ni) + off) & ~clr) | set, ni_base(ni) + off);
}

/* Which CPU pool a buffer belongs to: pool0 (EQ5) occupies the first
 * POOL0_BYTES of the reserved region, pool1 (EQ6) the rest. */
static inline u32 cortina_ni_rx_pool_eqid(struct cortina_ni *ni, u32 pa)
{
	u32 off = pa - lower_32_bits(ni->rx->cpu_dram_dma);

	return off < CA_NI_RX_CPU_POOL0_BYTES ? CA_NI_RX_EQ_ID :
					        CA_NI_RX_EQ_ID2;
}

/*
 * Stage one 128-byte-aligned buffer PA into EQ pool <eqid> through the CPU
 * push doorbell.  CA_NI_QM_CPU_PUSH_READY bit31 is a MANDATORY gate: a blind
 * write while the shallow push stage is full back-pressures the AXI write and
 * hangs the CPU, so the ready poll is bounded and a timeout is reported rather
 * than spun on.  In the steady state the stage is empty and this costs one
 * readl plus one writel.  Returns 0, or -EBUSY if no slot ever freed.
 */
/* @timeout_us bounds the mandatory ready poll.  It is a parameter, not the
 * shared constant, because the two callers have very different budgets: the
 * bring-up seed runs in process context and can afford the generous
 * CA_NI_RX_PUSH_TIMEOUT_US, while the NAPI recycle runs in softirq and repeats
 * this once per buffer - a whole batch at the generous bound would spin for
 * tens of milliseconds inside the poll and trip the watchdog. */
static int cortina_ni_rx_push_buf(struct cortina_ni *ni, u32 eqid, u32 pa,
				  unsigned int timeout_us)
{
	void __iomem *rdy = ni_base(ni) +
			    CA_NI_QM_CPU_PUSH_READY(CA_NI_RX_CPU_PORT);
	unsigned int i;

	for (i = 0; i < timeout_us; i++) {
		if (readl(rdy) & CA_NI_QM_PUSH_READY) {
			writel((pa & CA_NI_QM_PUSH_ADDR) |
			       FIELD_PREP(CA_NI_QM_PUSH_EQID, eqid),
			       ni_base(ni) +
			       CA_NI_QM_CPU_PUSH_PADDR(CA_NI_RX_CPU_PORT));
			return 0;
		}
		udelay(1);
	}
	return -EBUSY;
}

/* ------------------------------------------------------------------ */
/* CPU-pool buffers (HW self-populating, copy-break)                   */
/* ------------------------------------------------------------------ */
/* ★★ 2026-07-15 STOCK GOLDEN (STOCK_cpurx_dynamic_golden.txt): the CPU pools are
 * cpu_eq=0 SELF-POPULATING - the QM hardware maps bid n -> CFG0.phy_addr_start +
 * n * CFG2.buffer_size and recycles the bid when NAPI advances the CPU-EPP read
 * pointer.  Stock NEVER software-pushes an RX buffer (CPU_PUSH_PADDR0 0x63cc = 0
 * since boot); the former cpu_eq=1 host-fed push machinery here (cortina_ni_rx_
 * push/seed_pool/populate) was UNVALIDATED HW territory on this silicon and left
 * every admitted frame DMA'd to a bid whose PA never matched our pushed skbs
 * (inactive climbed +1/frame, NAPI read 0xdeadbeef poison).  We back both pools
 * with the reserved DRAM region (rx->cpu_dram) and run copy-break: NAPI copies
 * each delivered frame out of its pool buffer into a fresh skb; the buffer
 * returns to the pool by the EPP-rdptr advance alone - no push path exists.
 *
 * ★★ 2026-07-27 THE PARAGRAPH ABOVE IS WRONG AND IS KEPT ONLY AS A WARNING.
 * "Stock NEVER software-pushes" was concluded from reading CPU_PUSH_PADDR - a
 * WRITE-ONLY doorbell - and finding zero.  A write-only register reads zero on
 * a working system too, so that observation carried no information at all.  The
 * vendor does push: aal_l3qm_set_cpu_push_paddr writes {pa[31:7]<<7 | eqid} to
 * 0x63cc + cpu_port*8, batched once per NAPI poll.
 *
 * The cost of believing it was the IPv4 fragment defect.  With cpu_eq=0 the QM
 * frees the bid at descriptor writeback, so the CPU never owns the buffer: a
 * second frame arriving before NAPI drains lands on the SAME bid.  Measured
 * directly - every frame at pa=09400800, fragment 1's descriptor (dlen=1530)
 * resolving to a buffer already holding fragment 2 (hdra_pkt_size=570), two
 * byte-identical copies of fragment 2 with one IP id => IPFRAG_DUP => the
 * datagram never completes.  Longer bursts instead show the buffer changing
 * under the copy, which completes reassembly with a payload that fails its
 * checksum.  The vendor image reassembles every one on this same silicon.
 *
 * cpu_eq=1 removes the window rather than narrowing it: the QM cannot reclaim a
 * buffer until we re-push it, which happens after skb_put_data.  The earlier
 * attempt failed because it pushed dynamically allocated skb addresses, which
 * lie outside the NE's DDR window; we push PAs from the same reserved region
 * the RMU is provably writing to now.  Selectable: cpu_pool_push=0 restores the
 * self-populating behaviour described above, defect included. */

/* ------------------------------------------------------------------ */
/* interrupt mask / ack                                                */
/* ------------------------------------------------------------------ */

/*
 * CPU port 0 owns byte 0 of INT_EN0; a set bit enables the level interrupt
 * of that voq's EPP FIFO.  We only ever use port0/voq0-7, so plain writes
 * (not RMW) keep the ISR-vs-NAPI enable/disable race-free.
 *
 * ★★ MASKING MUST LEAVE bits[15:8] SET - they are the descriptor WRITEBACK
 * ENGINE enable, not part of the mask (fixed 2026-08-08; see the register
 * header).  This used to write 0, i.e. it turned the writeback engine OFF on
 * every interrupt until the poll completed.  Under a line-rate flood the
 * interrupt rate is enormous, so the engine spent a large share of the time
 * disabled, and a frame the QM admitted inside that window had its buffer
 * consumed with NO descriptor ever written: NAPI could not see it, so it was
 * never recycled into the pool.  That is a buffer leak proportional to the
 * interrupt rate, and it is INVISIBLE to every drop counter in this file
 * (push_fail, badpa, nosop, len, stale_buf all require a descriptor to have
 * been read), which is why the pool gauges kept reading healthy.
 *
 * Measured consequences, both now attributed to this one write:
 *   - transient: CPU delivery starves for a window under load and recovers
 *     (management blackout, pool still healthy, nobuf still 0);
 *   - permanent: the leak accumulates until the supply is empty, after which
 *     the BM no-free-buffer-drops EVERY arriving frame (0x216c climbing 1:1
 *     with 0x213c, RMU0 0x6900/0x6940 both 0 because nothing reaches them),
 *     killing all wired AND PON ingress with the kernel still alive and WiFi
 *     (separate PCIe DMA) unaffected.  Only a cold boot cleared it, because
 *     the refill is reachable only from the NAPI poll that no longer runs.
 */
static void cortina_ni_rx_irq_set(struct cortina_ni *ni, bool enable)
{
	writel(enable ? CA_NI_QM_EPP64_INT_EN0_STOCK
		      : CA_NI_QM_EPP64_INT_EN0_MASKED,
	       ni_base(ni) + CA_NI_QM_EPP64_INT_EN0);
}

static irqreturn_t cortina_ni_rx_isr(int irq, void *dev_id)
{
	struct cortina_ni_rx_irqctx *ctx = dev_id;
	struct cortina_ni *ni = ctx->ni;

	ni->rx->irq_hits[ctx->idx]++;

	/* mask (this is also the only ack - level by FIFO occupancy) */
	cortina_ni_rx_irq_set(ni, false);
	napi_schedule(&ni->rx->napi);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* NAPI poll                                                           */
/* ------------------------------------------------------------------ */

static u32 cortina_ni_rx_wptr_voq(struct cortina_ni *ni, unsigned int voq)
{
	u32 w = readl(ni_base(ni) +
		      CA_NI_QM_EPP64_WRPTR(CA_NI_RX_CPU_PORT, voq));

	/* byte offset, wraps at the per-voq ring size (stock count formula) */
	return (w & CA_NI_QM_EPP64_PTR) & (CA_NI_RX_RING_BYTES - 1);
}

static u32 cortina_ni_rx_wptr(struct cortina_ni *ni)
{
	return cortina_ni_rx_wptr_voq(ni, CA_NI_RX_VOQ);
}

/*
 * DS PON control-frame consumer (the cortina-gpon driver registers here).
 * A plain pointer store/load: the hook is set once at GPON probe and only
 * cleared at GPON remove, and the RX path tolerates either value.
 */
static cortina_ni_pon_rx_fn cortina_ni_pon_rx_cb;

void cortina_ni_pon_rx_hook_set(cortina_ni_pon_rx_fn fn)
{
	WRITE_ONCE(cortina_ni_pon_rx_cb, fn);
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_rx_hook_set);

/*
 * DS PON DATA (WAN) consumer: de-encapsulated data-GEM frames the PDC steers
 * to CPU port 0 arrive on this same EPP ring as plain Ethernet frames whose
 * HEADER_A.lspid = PON (7); the GPON driver registers its WAN netdev here so
 * they are delivered there instead of eth0 (LAN lspids are the NI ports
 * 0..6, so the test cannot steal a LAN frame).  Same store/load discipline
 * as the control-frame hook above.
 */
static struct net_device *cortina_ni_pon_wan_ndev;

void cortina_ni_pon_wan_ndev_set(struct net_device *ndev)
{
	WRITE_ONCE(cortina_ni_pon_wan_ndev, ndev);
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_wan_ndev_set);

/* ------------------------------------------------------------------ */
/* ★ TEMP DIAG rx_stack_tap implementation (REVERT with rx_crc_tap)    */
/* ------------------------------------------------------------------ */

/* L3FE debug snapshot mux (stock ca-ne.ko aal_l3fe_glb_dbg_get, tier-2
 * disasm): write (tap_idx << 5) | word_idx to NE+0x30b8, read the 32-bit
 * word at NE+0x30bc; 32 words per tap point = the 128-byte HDR_I.  Plain
 * address/data mux - no GO bit, no poll.  Tap idx 2 = "HDR_I between STG1 ~
 * T2 (Hash)" (the stock help text), i.e. the exact descriptor the T2 lookup
 * hashes. */
#define CA_NI_L3FE_DBG_ADDR		0x30b8
#define CA_NI_L3FE_DBG_DATA		0x30bc
#define CA_NI_L3FE_DBG_TAP_T2IN		2
#define CA_NI_HDRI_WORDS		32

/* a_cut HDR_I bit offsets (LSB-first over the 128-byte little-endian
 * descriptor: bit n = word[n >> 5] bit (n & 31)) - recovered from the stock
 * ca-ne.ko l3fe_debug_dump_hw_hdr_i_a_cut pretty-printer field extractions.
 * The l4_dp/l4_sp/ip_da/ip_sa/ip_l4_type/ip_proto/ip_ver/ip_vld offsets
 * independently agree with the tier-1 single-bit SWO learn (the same values
 * cortina-ni-flowoffload.c CN_HDRI_* uses), which validates the whole map. */
#define HDRI_L4_DP		74	/* 16b dest L4 port */
#define HDRI_L4_SP		90	/* 16b src L4 port */
#define HDRI_L3_TOTLEN		122	/* 14b IP total length */
#define HDRI_L3_CK_ERR		136	/*  1b l3_chksum_err */
#define HDRI_L3_CKSUM		137	/* 16b IP header checksum */
#define HDRI_IP_DA0		233	/* 32b v4 DA (LSW of the 128b field) */
#define HDRI_IP_SA0		361	/* 32b v4 SA (LSW of the 128b field) */
#define HDRI_IP_L4_TYPE		489	/*  3b 0=UDP 1=TCP .. */
#define HDRI_IP_PROTO		492	/*  8b IP protocol */
#define HDRI_IP_IHL		500	/*  4b IHL (words) */
#define HDRI_IP_VER		504	/*  1b 0=IPv4 1=IPv6 */
#define HDRI_IP_VLD		505	/*  1b parsed an IP header */
#define HDRI_PPP_PROTO_ENC	506	/*  4b ppp_protocol_enc */
#define HDRI_PPPOE_SESS		510	/* 16b pppoe_session_id */
#define HDRI_PPPOE_TYPE		526	/*  2b pppoe_type */
#define HDRI_INNER_1P		528	/*  3b inner 802.1p */
#define HDRI_TOP_1P		531	/*  3b top 802.1p */
#define HDRI_INNER_DEI		534	/*  1b */
#define HDRI_INNER_VID		535	/* 12b */
#define HDRI_INNER_TPID_ENC	547	/*  3b */
#define HDRI_TOP_DEI		550	/*  1b */
#define HDRI_TOP_VID		551	/* 12b */
#define HDRI_TOP_TPID_ENC	563	/*  3b */
#define HDRI_VLAN_CNT		566	/*  2b tags still ON the frame at T2 */
#define HDRI_ETYPE_ENC		585	/*  4b ethertype_enc */
#define HDRI_ETYPE		589	/* 16b ethertype (post-tag) */
#define HDRI_O_LSPID		816	/*  6b original lspid */
#define HDRI_LSPID		822	/*  6b lspid (post-LPB rewrite) */
#define HDRI_L4_OFFSET		897	/*  8b PE: L4 offset in the frame */
#define HDRI_L3_OFFSET		913	/*  8b PE: L3 offset in the frame */
#define HDRI_PKT_LEN		929	/* 14b PE: orig_packet_len */

/* extract an LSB-first bitfield (width <= 32) from the 32 LE HDR_I words */
static u32 rx_hdri_get(const u32 *w, unsigned int bit, unsigned int width)
{
	u64 v = ((u64)w[(bit >> 5) + 1] << 32) | w[bit >> 5];

	v >>= bit & 31;
	return v & (width < 32 ? (1u << width) - 1 : 0xffffffffu);
}

/* SW-decoded frame layering (explicit byte math, endianness-agnostic) */
struct rx_stack_sw {
	u8	tags;			/* VLAN tags found (2 recorded) */
	u16	tpid[2], tci[2];	/* outermost first */
	u16	ethertype;		/* after the last tag */
	bool	pppoe;
	u16	pppoe_sess, ppp_proto;
	bool	ip, inner;		/* outer IPv4 seen / IP-in-IP seen */
	u8	ipver, ihl;		/* outer version, outer IHL bytes */
	u8	proto;			/* INNERMOST IPv4 protocol */
	u32	sa, da;			/* INNERMOST IPv4, host-order value */
	bool	l4;
	u16	sp, dp;			/* innermost L4 ports */
};

static bool rx_stack_sw_parse(const u8 *p, int len, struct rx_stack_sw *s)
{
	int pos = 12, ip_pos, i;
	u16 et = 0;

	memset(s, 0, sizeof(*s));
	if (len < 14)
		return false;
	for (i = 0; i < 3; i++) {	/* walk up to 3 stacked VLAN tags */
		if (pos + 4 > len)
			return false;
		et = ((u16)p[pos] << 8) | p[pos + 1];
		if (et != 0x8100 && et != 0x88a8 && et != 0x9100)
			break;
		if (s->tags < 2) {
			s->tpid[s->tags] = et;
			s->tci[s->tags] = ((u16)p[pos + 2] << 8) | p[pos + 3];
		}
		s->tags++;
		pos += 4;
	}
	s->ethertype = et;
	pos += 2;			/* now at the payload */

	if (et == 0x8863) {		/* PPPoE discovery: no IP inside */
		s->pppoe = true;
		return true;
	}
	if (et == 0x8864) {		/* PPPoE session */
		if (pos + 8 > len)
			return false;
		s->pppoe = true;
		s->pppoe_sess = ((u16)p[pos + 2] << 8) | p[pos + 3];
		s->ppp_proto = ((u16)p[pos + 6] << 8) | p[pos + 7];
		if (s->ppp_proto != 0x0021)	/* descend only into PPP-IPv4 */
			return true;
		pos += 8;
		et = 0x0800;
	}
	if (et != 0x0800)		/* v6/ARP/...: stack recorded, no v4 */
		return true;

	ip_pos = pos;
	if (ip_pos + 20 > len)
		return false;
	s->ip = true;
	s->ipver = p[ip_pos] >> 4;
	s->ihl = (p[ip_pos] & 0xf) * 4;
	s->proto = p[ip_pos + 9];
	s->sa = get_unaligned_be32(p + ip_pos + 12);
	s->da = get_unaligned_be32(p + ip_pos + 16);
	if (s->proto == 4) {		/* IPv4-in-IPv4 */
		ip_pos += s->ihl;
		if (ip_pos + 20 > len)
			return false;
		s->inner = true;
		s->proto = p[ip_pos + 9];
		s->sa = get_unaligned_be32(p + ip_pos + 12);
		s->da = get_unaligned_be32(p + ip_pos + 16);
	}
	/* L4 of the innermost IPv4 (proto 41 = 6in4: flagged, not descended) */
	pos = ip_pos + (p[ip_pos] & 0xf) * 4;
	if ((s->proto == 17 || s->proto == 6) && pos + 4 <= len) {
		s->l4 = true;
		s->sp = ((u16)p[pos] << 8) | p[pos + 1];
		s->dp = ((u16)p[pos + 2] << 8) | p[pos + 3];
	}
	return true;
}

static noinline void cortina_ni_rx_stack_tap(struct cortina_ni *ni,
					     const u8 *p, int len)
{
	static DEFINE_RATELIMIT_STATE(rs, 2 * HZ, 2);
	static unsigned int hits;
	struct net_device *ndev = ni->rx->netdev;
	u32 w[CA_NI_HDRI_WORDS];
	struct rx_stack_sw s;
	u32 hw_dp, hw_sa, hw_da, hw_vcnt, hw_vld;
	char fl[128];
	unsigned int i;
	int n = 0;

	if (!rx_stack_sw_parse(p, len, &s))
		return;
	/* the offload probe: innermost IPv4/UDP dport 19555, any layering */
	if (!s.ip || s.proto != 17 || !s.l4 || s.dp != 19555)
		return;
	if (!__ratelimit(&rs))
		return;
	hits++;

	/* (a) actual frame stack + anomaly flags vs plain {Eth->IPv4->UDP} */
	fl[0] = '\0';
	if (s.tags)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85VLAN %04x/vid=0x%03x NOT untagged%s",
			       s.tpid[0], s.tci[0] & 0xfff,
			       s.tags > 1 ? " +QinQ" : "");
	if (s.tags > 1)
		n += scnprintf(fl + n, sizeof(fl) - n, " inner %04x/vid=0x%03x",
			       s.tpid[1], s.tci[1] & 0xfff);
	if (s.pppoe)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85PPPoE sess=0x%04x", s.pppoe_sess);
	if (s.inner)
		n += scnprintf(fl + n, sizeof(fl) - n, " \xe2\x98\x85IP-over-IP");
	if (!n)
		scnprintf(fl, sizeof(fl), " plain Eth/IPv4/UDP (as expected)");
	netdev_info(ndev,
		    "stack_tap#%u SW : eth{da=%pM sa=%pM} tags=%u et=%04x ip{v%u ihl=%u proto=%u sa=%08x da=%08x} l4{sp=%u dp=%u} |%s\n",
		    hits, p, p + 6, s.tags, s.ethertype, s.ipver, s.ihl,
		    s.proto, s.sa, s.da, s.sp, s.dp, fl);

	/* (b) the HW's parse: HDR_I at the T2 (hash) input, via the debug mux */
	for (i = 0; i < CA_NI_HDRI_WORDS; i++) {
		writel((CA_NI_L3FE_DBG_TAP_T2IN << 5) | i,
		       ni_base(ni) + CA_NI_L3FE_DBG_ADDR);
		w[i] = readl(ni_base(ni) + CA_NI_L3FE_DBG_DATA);
	}
	hw_dp   = rx_hdri_get(w, HDRI_L4_DP, 16);
	hw_sa   = rx_hdri_get(w, HDRI_IP_SA0, 32);
	hw_da   = rx_hdri_get(w, HDRI_IP_DA0, 32);
	hw_vcnt = rx_hdri_get(w, HDRI_VLAN_CNT, 2);
	hw_vld  = rx_hdri_get(w, HDRI_IP_VLD, 1);
	netdev_info(ndev,
		    "stack_tap#%u HW : HDR_I@T2 lspid=%02lx(o=%02lx) et=%04lx(enc%lx) vlan_cnt=%u top{tpid%lx vid=0x%03lx p%lu d%lu} inner{tpid%lx vid=0x%03lx p%lu d%lu} pppoe{t%lu sess=0x%04lx enc%lx} ip{vld%u v%lu ihl=%lu proto=%lu l4t=%lu} sa=%08x da=%08x sp=%lu dp=%u l3{ck=%04lx err%lu len=%lu} pe_off{l3=%lu l4=%lu plen=%lu}\n",
		    hits,
		    (unsigned long)rx_hdri_get(w, HDRI_LSPID, 6),
		    (unsigned long)rx_hdri_get(w, HDRI_O_LSPID, 6),
		    (unsigned long)rx_hdri_get(w, HDRI_ETYPE, 16),
		    (unsigned long)rx_hdri_get(w, HDRI_ETYPE_ENC, 4),
		    hw_vcnt,
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_TPID_ENC, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_VID, 12),
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_1P, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_TOP_DEI, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_TPID_ENC, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_VID, 12),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_1P, 3),
		    (unsigned long)rx_hdri_get(w, HDRI_INNER_DEI, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_PPPOE_TYPE, 2),
		    (unsigned long)rx_hdri_get(w, HDRI_PPPOE_SESS, 16),
		    (unsigned long)rx_hdri_get(w, HDRI_PPP_PROTO_ENC, 4),
		    hw_vld,
		    (unsigned long)rx_hdri_get(w, HDRI_IP_VER, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_IP_IHL, 4),
		    (unsigned long)rx_hdri_get(w, HDRI_IP_PROTO, 8),
		    (unsigned long)rx_hdri_get(w, HDRI_IP_L4_TYPE, 3),
		    hw_sa, hw_da,
		    (unsigned long)rx_hdri_get(w, HDRI_L4_SP, 16),
		    hw_dp,
		    (unsigned long)rx_hdri_get(w, HDRI_L3_CKSUM, 16),
		    (unsigned long)rx_hdri_get(w, HDRI_L3_CK_ERR, 1),
		    (unsigned long)rx_hdri_get(w, HDRI_L3_TOTLEN, 14),
		    (unsigned long)rx_hdri_get(w, HDRI_L3_OFFSET, 8),
		    (unsigned long)rx_hdri_get(w, HDRI_L4_OFFSET, 8),
		    (unsigned long)rx_hdri_get(w, HDRI_PKT_LEN, 14));

	/* verdict: same frame?  (mux = LAST frame through STG1->T2) + diffs */
	n = 0;
	fl[0] = '\0';
	if (hw_vcnt != s.tags)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85vlan_cnt HW=%u vs frame=%u",
			       hw_vcnt, s.tags);
	if (hw_vcnt)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85tag still present at T2");
	if (!hw_vld)
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85HW parsed NO IP header");
	if (!((hw_sa == s.sa || hw_sa == swab32(s.sa)) &&
	      (hw_da == s.da || hw_da == swab32(s.da))))
		n += scnprintf(fl + n, sizeof(fl) - n,
			       " \xe2\x98\x85HW sa/da != frame sa/da");
	if (!n)
		scnprintf(fl, sizeof(fl), " HW parse == SW stack");
	netdev_info(ndev, "stack_tap#%u -->: %s;%s\n", hits,
		    (hw_dp == 19555 || hw_dp == swab16(19555)) ?
		    "same-frame" :
		    "\xe2\x98\x85STALE snapshot (another frame raced the mux)",
		    fl);

	/* raw words for offline decode of anything not printed above */
	if (hits <= 4)
		print_hex_dump(KERN_INFO, "stack_tap HDR_I: ",
			       DUMP_PREFIX_OFFSET, 16, 4, w, sizeof(w), false);
}

/* ------------------------------------------------------------------ */
/* multi-buffer receive: the SOP..EOP descriptor chain                  */
/* ------------------------------------------------------------------ */

/*
 * Where a frame with this HEADER_A is delivered: the GPON WAN netdev, or NULL
 * for the eth0 one.  A de-encapsulated data-GEM frame carries lspid = PON; once
 * the DS data GEM is routed through the L3FE, a terminating or not-yet-offloaded
 * frame reaches the CPU as lspid = L3_WAN instead, because STG0's LPB profile has
 * already rewritten it.  Both are WAN-only lspids, so neither can steal a LAN
 * frame.  A NULL return with *is_l3 untouched means "no WAN lspid"; a NULL return
 * after a WAN lspid means no WAN netdev is registered yet, and both fall back to
 * eth0 - which is what the single-buffer path did inline before this became the
 * ONE copy of the decision shared with the chain path.
 */
static inline struct net_device *cortina_ni_rx_wan_dest(u32 hdra_lo, bool *is_l3)
{
	u32 lspid = FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo);
	bool pon = (lspid == CA_NI_LSPID_PON);
	bool l3 = (lspid == CA_NI_LSPID_L3_WAN) &&
		  cortina_ni_hw_l3_fwd_active();

	if (!pon && !l3)
		return NULL;
	*is_l3 = l3;
	return READ_ONCE(cortina_ni_pon_wan_ndev);
}

/* what one descriptor of a chain contributes */
struct ca_ni_chain_seg {
	u32	off;		/* payload offset inside this buffer */
	u32	len;		/* payload bytes to take from it */
	u32	dlen_expect;	/* what this descriptor's own pkt_size should read */
};

enum ca_ni_chain_act {
	CA_NI_CHAIN_OPEN,	/* SOP accepted, more segments expected */
	CA_NI_CHAIN_APPEND,	/* continuation accepted, more expected */
	CA_NI_CHAIN_DONE,	/* EOP accepted, the frame is complete */
	/* ---- malformations; the caller drops the partial frame for each ---- */
	CA_NI_CHAIN_ORPHAN,	/* continuation with no chain open on this voq */
	CA_NI_CHAIN_BADTOTAL,	/* SOP pkt_size out of range, or no room in the window */
	CA_NI_CHAIN_TOOLONG,	/* segment cap hit, or the segments overran total */
	CA_NI_CHAIN_SHORT,	/* EOP with fewer bytes than pkt_size promised */
};

/*
 * The chain state machine, as pure arithmetic: no skb, no MMIO, no netdev.  Given
 * this descriptor's SOP/EOP bits, the SOP buffer's HEADER_A words, the usable
 * window of the buffer and where a continuation's payload starts, decide what
 * this segment contributes and whether the chain is still well formed.
 *
 * The caller must have dropped any previously held partial frame when @st->open
 * is true and @sop is set - the SOP branch below re-initialises the accounting
 * unconditionally, which is what bounds an unterminated chain: it cannot outlive
 * the next frame that arrives on the same voq.
 *
 * Every malformation return leaves @st dirty on purpose; the caller resets it.
 *
 * ★ The arithmetic is the shipped firmware's, arrived at independently: it takes
 * min(bytes still owed, 1600) per segment, with the first segment's window
 * shortened by the 8- or 16-byte header block, and treats HEADER_A.pkt_size on
 * the SOP buffer as the sole authority for the total.  Expressed here in terms of
 * the buffer's usable window rather than a literal 1600, so it stays correct for
 * a pool with a different buffer size - which is the reason this path exists.
 *
 * @out->dlen_expect is the byte count we derive for this segment, reported so the
 * caller can record it beside what the descriptor actually said.  It is NOT used
 * to decide anything: the firmware discards the per-segment descriptor length
 * outright, so there is no evidence of what the hardware writes there.
 */
static enum ca_ni_chain_act
ca_ni_chain_step(struct ca_ni_chain_state *st, bool sop, bool eop,
		 u32 hdra_hi, u32 hdra_lo, u32 buf_end, u32 rest_off,
		 struct ca_ni_chain_seg *out)
{
	u32 off, want, avail;

	out->off = out->len = out->dlen_expect = 0;

	if (sop) {
		int total = FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo);

		off = CA_NI_RX_FRAME_OFF;
		if (hdra_hi & CA_NI_HDRA_W0_CPU_FLG) {
			off += CA_NI_RX_HDR_CPU_LEN;
			total -= CA_NI_RX_HDR_CPU_LEN;
		}
		if (total < (int)ETH_HLEN ||
		    total > (int)CA_NI_RX_CHAIN_MAX_LEN || off >= buf_end)
			return CA_NI_CHAIN_BADTOTAL;
		st->total = total;
		st->got = 0;
		st->segs = 0;
		st->open = true;
	} else {
		if (!st->open)
			return CA_NI_CHAIN_ORPHAN;
		off = rest_off;
		if (off >= buf_end)
			return CA_NI_CHAIN_BADTOTAL;
	}

	if (++st->segs > CA_NI_RX_CHAIN_MAX_SEGS)
		return CA_NI_CHAIN_TOOLONG;

	avail = buf_end - off;
	want = st->total - st->got;
	out->off = off;
	out->len = min(want, avail);
	out->dlen_expect = (off - CA_NI_RX_HDRA_OFF) + out->len;
	st->got += out->len;

	if (eop) {
		if (st->got != st->total)
			return CA_NI_CHAIN_SHORT;
		/* the frame is complete, so this voq holds no chain any more.
		 * Cleared HERE and not only in the caller: leaving it set would
		 * make a delivered chain indistinguishable from an open one, so
		 * the next continuation to arrive would be appended to a frame
		 * that has already gone up the stack instead of being counted as
		 * the orphan it is. */
		st->open = false;
		return CA_NI_CHAIN_DONE;
	}
	/*
	 * Not the last segment, so it must have FILLED its window: if the frame
	 * owes fewer bytes than this buffer can hold and yet more descriptors
	 * are coming, pkt_size and the hardware have already disagreed.  This is
	 * also the test that makes overrun impossible - st->got can never pass
	 * st->total, so the skb allocated for st->total is never outgrown.
	 */
	if (out->len != avail)
		return CA_NI_CHAIN_TOOLONG;
	return sop ? CA_NI_CHAIN_OPEN : CA_NI_CHAIN_APPEND;
}

/* free a held partial frame WITHOUT touching the accounting (the SOP path has
 * already re-initialised it for the new chain) */
static void cortina_ni_rx_chain_free(struct cortina_ni_rx *rx,
				     struct cortina_ni_rx_chain *ch)
{
	if (!ch->skb)
		return;
	dev_kfree_skb_any(ch->skb);
	ch->skb = NULL;
	rx->chain_abort++;
}

/* abandon a chain entirely: free the partial frame and clear the accounting */
static void cortina_ni_rx_chain_reset(struct cortina_ni_rx *rx,
				      struct cortina_ni_rx_chain *ch)
{
	cortina_ni_rx_chain_free(rx, ch);
	memset(&ch->st, 0, sizeof(ch->st));
}

/*
 * Consume one descriptor of a multi-buffer frame.  Reached only with rx_chain=1
 * and only for a descriptor that is not a self-contained frame, so the
 * single-buffer path is left alone.
 *
 * Returns the PA to recycle, ALWAYS - on delivery, on a malformation, on an
 * allocation failure.  Every buffer of a chain is therefore handed back exactly
 * once, because the caller collects one PA per descriptor and each segment is
 * copied out before the batch push at the end of the poll: a buffer is free the
 * moment its own segment has been copied, and nothing in a chain defers that.
 */
static noinline u32 cortina_ni_rx_chain_seg(struct cortina_ni *ni,
					    unsigned int voq, u64 desc,
					    const u8 *buf, u32 buf_end, u32 rpa)
{
	struct cortina_ni_rx *rx = ni->rx;
	struct cortina_ni_rx_chain *ch = &rx->chain[voq];
	struct net_device *ndev = rx->netdev;
	u32 lo = lower_32_bits(desc);
	bool sop = !!(lo & CA_NI_RX_DESC_SOP);
	bool eop = !!(lo & CA_NI_RX_DESC_EOP);
	u32 dlen = FIELD_GET(CA_NI_RX_DESC_LEN, desc);
	u32 hdra_hi = 0, hdra_lo = 0;
	struct ca_ni_chain_seg seg;
	enum ca_ni_chain_act act;
	struct net_device *wan;
	bool is_l3 = false;

	/* The headerless (sw_id != 0) format's chain geometry is unknown - its
	 * frame starts at +0x10, inside the head_room the chain arithmetic is
	 * built on, and no such descriptor has ever been observed on this board.
	 * Refuse it rather than guess; a non-zero counter here is a finding. */
	if (unlikely(FIELD_GET(CA_NI_RX_DESC_SWID, desc))) {
		rx->chain_swid++;
		ndev->stats.rx_errors++;
		net_warn_ratelimited("%s: RX chain: headerless (sw_id) segment, geometry unknown - dropped\n",
				     netdev_name(ndev));
		return rpa;
	}

	if (sop) {
		hdra_hi = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF);
		hdra_lo = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 4);
		rx->last_hdra = ((u64)hdra_hi << 32) | hdra_lo;
		/* a chain still open here never got its EOP: drop the partial
		 * frame and let this SOP start over */
		if (unlikely(ch->st.open)) {
			cortina_ni_rx_chain_free(rx, ch);
			rx->chain_reopen++;
			ndev->stats.rx_errors++;
		}
		ch->hdra_lo = hdra_lo;
	}

	act = ca_ni_chain_step(&ch->st, sop, eop, hdra_hi, hdra_lo, buf_end,
			       rx->chain_rest_off, &seg);

	switch (act) {
	case CA_NI_CHAIN_OPEN:
	case CA_NI_CHAIN_APPEND:
	case CA_NI_CHAIN_DONE:
		break;
	case CA_NI_CHAIN_ORPHAN:
		rx->chain_orphan++;
		ndev->stats.rx_errors++;
		net_warn_ratelimited("%s: RX chain: continuation with no chain open on voq %u (desc %016llx)\n",
				     netdev_name(ndev), voq, desc);
		return rpa;
	case CA_NI_CHAIN_BADTOTAL:
		rx->chain_badtotal++;
		goto drop;
	case CA_NI_CHAIN_TOOLONG:
		rx->chain_toolong++;
		goto drop;
	case CA_NI_CHAIN_SHORT:
		rx->chain_short++;
		goto drop;
	}

	/*
	 * ★ The per-segment descriptor pkt_size is RECORDED, not judged.
	 *
	 * The shipped firmware's chain reader discards it outright - it returns
	 * only the low 32 bits of the continuation descriptor and never looks at
	 * bits [45:32] - so there is no evidence of what the hardware puts there
	 * for a segment, and a mismatch against our derived byte count would be
	 * a claim, not a measurement.  Treating it as an error counter would
	 * manufacture exactly the kind of witness that reads like a fault while
	 * measuring nothing.
	 *
	 * So: store what each segment's descriptor actually reported, expose it
	 * beside the value we derived, and let the FIRST chained frame on
	 * hardware establish the convention.  chain_dlen_diff counts the
	 * disagreements only so the /proc line can say whether there were any.
	 */
	if (ch->st.segs <= CA_NI_RX_CHAIN_MAX_SEGS) {
		rx->chain_dlen_seen[ch->st.segs - 1] = dlen;
		rx->chain_dlen_calc[ch->st.segs - 1] = seg.dlen_expect;
	}
	if (dlen != seg.dlen_expect)
		rx->chain_dlen_diff++;

	if (sop) {
		/* ONE allocation per frame, sized from the SOP's pkt_size and
		 * never grown - the arithmetic above cannot exceed it */
		ch->skb = napi_alloc_skb(&rx->napi, ch->st.total);
		if (unlikely(!ch->skb)) {
			rx->drop_nobuf++;
			ndev->stats.rx_dropped++;
			memset(&ch->st, 0, sizeof(ch->st));
			return rpa;
		}
	} else if (unlikely(!ch->skb)) {
		/* accounting says a chain is open but the allocation failed
		 * earlier: swallow the rest of it quietly */
		if (eop)
			memset(&ch->st, 0, sizeof(ch->st));
		return rpa;
	}

	skb_put_data(ch->skb, buf + seg.off, seg.len);

	if (act != CA_NI_CHAIN_DONE)
		return rpa;

	/* complete frame: same delivery decision as the single-buffer path.
	 * A PON control frame is never chained (an OMCI PDU is tens of bytes),
	 * so this path deliberately does not re-test the 0xfff1 link type. */
	{
		struct sk_buff *skb = ch->skb;
		u32 segs = ch->st.segs;
		/* the frame length must be taken BEFORE the skb is handed over:
		 * eth_type_trans() pulls the Ethernet header off it and
		 * napi_gro_receive() consumes it outright */
		u32 flen = ch->st.total;

		ch->skb = NULL;
		memset(&ch->st, 0, sizeof(ch->st));
		rx->chain_frames++;
		rx->chain_segs += segs;
		if (segs > rx->chain_max_segs)
			rx->chain_max_segs = segs;
		wan = cortina_ni_rx_wan_dest(ch->hdra_lo, &is_l3);
		if (wan) {
			if (is_l3)
				rx->wan_l3_frames++;
			else
				rx->wan_frames++;
			wan->stats.rx_packets++;
			wan->stats.rx_bytes += flen;
			skb->protocol = eth_type_trans(skb, wan);
			napi_gro_receive(&rx->napi, skb);
			return rpa;
		}
		rx->frames++;
		rx->bytes += flen;
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += flen;
		skb->protocol = eth_type_trans(skb, ndev);
		napi_gro_receive(&rx->napi, skb);
	}
	return rpa;

drop:
	ndev->stats.rx_errors++;
	net_warn_ratelimited("%s: RX chain: malformed (act %d, segs %u got %u of %u) - partial frame dropped\n",
			     netdev_name(ndev), act, ch->st.segs, ch->st.got,
			     ch->st.total);
	cortina_ni_rx_chain_reset(rx, ch);
	return rpa;
}

/* consume one CPU-EPP descriptor.  The frame sits in a software-populated DRAM buffer
 * inside our coherent CPU-pool region; copy it into a fresh skb and deliver, then
 * RE-PUSH that buffer's PA back to its EQ free-list (copy-break recycle).  cpu_eq=0
 * pools are NOT hardware-recycled - the vendor refills them by software push (stock
 * ca_ni_refill_eq_buf_pool), so every buffer we consume must be pushed back or the
 * free-list drains and RMU0 stops admitting. */
static u32 cortina_ni_rx_frame(struct cortina_ni *ni, unsigned int voq, u64 desc)
{
	struct cortina_ni_rx *rx = ni->rx;
	struct net_device *ndev = rx->netdev;
	u32 pa = lower_32_bits(desc) & CA_NI_RX_DESC_PA;
	u32 dlen = FIELD_GET(CA_NI_RX_DESC_LEN, desc);
	u32 swid = FIELD_GET(CA_NI_RX_DESC_SWID, desc);
	u32 base = lower_32_bits(rx->cpu_dram_dma);
	u32 hdra_hi = 0, hdra_lo = 0, off_in_region, buf_max;
	u32 rpa;	/* the PA to hand back, or 0 if this buffer is not ours */
	struct sk_buff *skb;
	const u8 *buf;
	unsigned int off;
	int len;

	rx->last_desc = desc;

	/* bufPA (128B-aligned, low 7 bits are flags) must land inside the mapped
	 * window: the two CPU pools, then the deep-queue pool after them */
	if (unlikely(pa < base || pa >= base + CA_NI_RX_MAP_SIZE)) {
		rx->drop_badpa++;
		ndev->stats.rx_errors++;
		net_err_ratelimited("%s: RX desc %016llx: PA outside CPU pool\n",
				    netdev_name(ndev), desc);
		return 0;	/* unknown PA: cannot safely recycle it */
	}
	off_in_region = pa - base;
	buf = (const u8 *)rx->cpu_dram + off_in_region;
	/*
	 * One buffer's span, and who owns it.  EQ5 (pool0) below POOL0_BYTES,
	 * EQ6 (pool1) above it, and the DEEP-QUEUE pool (EQ12) above both.
	 *
	 * Under cpu_pool_push the caller returns a CPU-pool PA to its free list
	 * once the frame has been copied out - that ownership interval is what
	 * stops the next frame landing on top of this one.  ★ The deep-queue
	 * pool is deliberately hardware-managed (this chip's deep-queue enqueue
	 * cannot consume a pushed buffer), so its buffers are NOT ours to hand
	 * back: pushing one into a CPU pool's free list would put the same
	 * buffer in two allocators at once and corrupt both.  rpa = 0 for those.
	 */
	/* ★ buf_max is the end of the buffer's USABLE PAYLOAD WINDOW, not the
	 * buffer size: the QM reserves CA_NI_RX_BUF_TAILROOM at the back and the
	 * frame DMA never writes there, so the 384 bytes this used to allow were
	 * memory the hardware had not written.  See CA_NI_RX_BUF_USABLE_END. */
	if (off_in_region < CA_NI_RX_CPU_POOL0_BYTES) {
		buf_max = CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ);
		rpa = pa;
	} else if (off_in_region < CA_NI_RX_DQ_POOL_OFF) {
		buf_max = CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL1_BUFSZ);
		rpa = pa;
	} else {
		/* DQ pool, same 2048B buffers */
		buf_max = CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ);
		rpa = 0;				/* hardware-managed */
		rx->dq_frames++;
	}

	/* ★ multi-buffer receive (rx_chain, default off).  A descriptor that is
	 * not a self-contained frame - SOP without EOP, or neither - belongs to a
	 * chain.  The test short-circuits on the parameter, so with the feature
	 * disabled EOP is not even read and the descriptor is treated exactly as
	 * it was before: a SOP is delivered short, a continuation counted nosop. */
	if (unlikely(rx_chain &&
		     (lower_32_bits(desc) & (CA_NI_RX_DESC_SOP |
					     CA_NI_RX_DESC_EOP)) !=
		     (CA_NI_RX_DESC_SOP | CA_NI_RX_DESC_EOP)))
		return cortina_ni_rx_chain_seg(ni, voq, desc, buf, buf_max, rpa);

	if (unlikely(!(lower_32_bits(desc) & CA_NI_RX_DESC_SOP))) {
		/* SOP-less descriptor = jumbo continuation or desync; drop */
		rx->drop_nosop++;
		ndev->stats.rx_errors++;
		return rpa;
	}

	if (likely(!swid)) {
		/* normal frame: HEADER_A at +0x40 = two BIG-ENDIAN 32-bit
		 * words (stock byte-swaps both before extracting fields);
		 * authoritative frame length = HEADER_A.pkt_size */
		hdra_hi = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF);
		hdra_lo = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 4);
		rx->last_hdra = ((u64)hdra_hi << 32) | hdra_lo;

		len = FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo);

		/* ★ Staleness witness, free: the descriptor's own pktlen and
		 * HEADER_A.pkt_size describe the SAME frame from two
		 * INDEPENDENT places - the ring, written by the EPP writeback
		 * engine, and the buffer, written by the RMU frame DMA - and on
		 * every good frame differ by exactly the 8-byte HEADER_A
		 * (pktlen counts from buf+0x40, pkt_size from buf+0x48;
		 * board-measured 414/406, 1530/1522, 578/570).  A mismatch
		 * means the buffer no longer holds the frame this descriptor
		 * was written for - the exact condition the ownership fix
		 * removes, so this counter must read 0.  Both values are
		 * already in registers here, so the check costs one compare.
		 * Counted and logged, NOT dropped: dropping would turn a
		 * duplicate into a hole, the datagram fails either way, and a
		 * silent behaviour change would muddy the A/B. */
		if (unlikely(dlen != len + (CA_NI_RX_FRAME_OFF -
					    CA_NI_RX_HDRA_OFF))) {
			rx->stale_buf++;
			net_warn_ratelimited("%s: RX stale buffer: desc pktlen=%u vs HEADER_A.pkt_size=%d at pa=%08x (buffer reused before the copy)\n",
					     netdev_name(ndev), dlen, len, pa);
		}

		off = CA_NI_RX_FRAME_OFF;
		if (hdra_hi & CA_NI_HDRA_W0_CPU_FLG) {
			off += CA_NI_RX_HDR_CPU_LEN;
			len -= CA_NI_RX_HDR_CPU_LEN;
		}
	} else {
		/* headerless format (stock: nonzero sw_id, WiFi-FF style):
		 * frame at +0x10, length from the descriptor */
		rx->swid_frames++;
		off = CA_NI_RX_DESC_HDR_LEN;
		len = (int)dlen - CA_NI_RX_DESC_HDR_LEN;
	}

	if (unlikely(rx_debug && rx->frames < 4)) {
		netdev_info(ndev,
			    "RX desc=%016llx hdra=%08x:%08x (lspid=%u pkt_size=%u cpu=%u swid=%u dlen=%u) pa=%08x len=%d off=%u\n",
			    desc, hdra_hi, hdra_lo,
			    (u32)FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo),
			    (u32)FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo),
			    !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG), swid, dlen,
			    pa, len, off);
		print_hex_dump(KERN_INFO, "RX buf+0x40: ", DUMP_PREFIX_OFFSET,
			       16, 1, buf + CA_NI_RX_HDRA_OFF, 96, false);
	}

	if (unlikely(len < (int)ETH_HLEN || off + len > buf_max)) {
		rx->drop_len++;
		/* ★ split, because the two causes mean opposite things and the
		 * aggregate cannot tell them apart: a runt is a bad frame, while
		 * an oversize one is a good frame that does not fit a buffer's
		 * usable window and needs rx_chain.  Distinguishing them is what
		 * makes the buffer-window fix measurable rather than a claim. */
		if (len < (int)ETH_HLEN)
			rx->drop_runt++;
		else
			rx->drop_oversize++;
		ndev->stats.rx_length_errors++;
		ndev->stats.rx_errors++;
		return rpa;
	}

	/* ★ TEMPORARY DIAGNOSTIC (rx_ds_tap) - revert with rx_frag_tap.  Sampled
	 * HERE, ahead of the PON-control and WAN-netdev branches below, both of
	 * which return early: a downstream frame must be visible to the tap
	 * whichever branch it goes on to take. */
	if (unlikely(rx_ds_tap > 0 && !swid))
		cortina_ni_rx_ds_tap(ndev, buf, off, len, desc, pa, dlen,
				     hdra_hi, hdra_lo);

	/*
	 * DS-WAN delivery spy (rx_debug): a DHCP frame (UDP src/dst 67/68) is
	 * the one DS-terminating packet we must trace when the HW-L3 miss-punt
	 * path is under test.  Log its lspid + swid so we can see exactly what
	 * the L3FE miss-punt hands the CPU (PON 7 vs L3_WAN 0x18 vs a LAN lspid)
	 * and which delivery branch it will take.  Ratelimited; off by default.
	 */
	if (unlikely(rx_debug && !swid && off + 38 <= buf_max &&
		     buf[off + 12] == 0x08 && buf[off + 13] == 0x00 &&
		     buf[off + 23] == 0x11)) {
		u16 dport = ((u16)buf[off + 36] << 8) | buf[off + 37];
		u16 sport = ((u16)buf[off + 34] << 8) | buf[off + 35];

		if (dport == 67 || dport == 68 || sport == 67 || sport == 68)
			net_info_ratelimited(
			  "%s: DHCP DS frame lspid=%lu swid=%u sport=%u dport=%u DA=%02x:%02x:%02x:%02x:%02x:%02x len=%d\n",
			  netdev_name(ndev),
			  FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo), swid,
			  sport, dport, buf[off], buf[off+1], buf[off+2],
			  buf[off+3], buf[off+4], buf[off+5], len);
	}

	/*
	 * ★ TEMPORARY DIAGNOSTIC crc_ntfy tap (rx_crc_tap gate; see the module
	 * param above - REVERT once the divergence is pinned).  Match a punted
	 * IPv4/UDP frame to the offload-test sink port (dport 19555, any
	 * src - covers the transit probe 192.168.1.99:41099 and sport bumps)
	 * and read the T2-computed lookup CRC the hardware wrote into THIS
	 * frame's HEADER_CPU meta: crc32 = BE32 @buf+0x48, crc16 = BE16
	 * @buf+0x4C (meaningful when HEADER_A.cpu_flg=1 - logged so a missing
	 * HEADER_CPU is itself a finding).  Compare against the install-side
	 * SWO crc for the same 5-tuple; install the read value verbatim via
	 * /proc/cortina_l3fe "rawinst <crc32> <crc16>" for the guaranteed-hit
	 * proof (age 1->2 + HS_CACHE_CNT climb).
	 */
	if (unlikely(rx_crc_tap && !swid && off + 38 <= buf_max &&
		     buf[off + 12] == 0x08 && buf[off + 13] == 0x00 &&
		     buf[off + 23] == 0x11 &&
		     ((((u16)buf[off + 36] << 8) | buf[off + 37]) == 19555))) {
		u32 c32 = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 8);
		u16 c16 = get_unaligned_be16(buf + CA_NI_RX_HDRA_OFF + 12);
		bool cpuf = !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG);

		rx->tap_hits++;
		rx->tap_crc32 = c32;
		rx->tap_crc16 = c16;
		rx->tap_cpuflg = cpuf;
		net_info_ratelimited(
			"%s: crc_tap %pI4:%u -> %pI4:19555 UDP lspid=%lu cpu_flg=%u HW crc32=%08x crc16=%04x (vs the install crc = the divergence)\n",
			netdev_name(ndev), buf + off + 26,
			(u16)(((u16)buf[off + 34] << 8) | buf[off + 35]),
			buf + off + 30,
			FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo),
			cpuf, c32, c16);
	}

	/*
	 * ★ TEMP DIAG packet-stack tap (rx_stack_tap gate - REVERT with
	 * rx_crc_tap): SW-decoded frame layering vs the HW's HDR_I parse,
	 * side by side.  Matches the probe through ANY VLAN/QinQ/PPPoE/
	 * IP-in-IP layering (the fixed-offset crc tap above misses a
	 * tag-shifted probe entirely - itself a symptom this tap exposes).
	 */
	if (unlikely(rx_stack_tap && !swid))
		cortina_ni_rx_stack_tap(ni, buf + off, len);

	/*
	 * ★ PPPoE punt-integrity witness (GAP-2).  A DS PPPoE session frame that
	 * reaches the CPU must be self-consistent (PPPoE length == inner IPv4
	 * total length + 2); the 2026-07-20 regression made those frames arrive
	 * with the TCP header shifted by the 8-byte encap, which is invisible to
	 * every register and hit counter but obvious in the frame.  Default OFF -
	 * one predicted-not-taken branch per received frame when disarmed.
	 */
	if (unlikely(cortina_ni_pppoe_punt_armed()))
		cortina_ni_pppoe_punt_inspect(buf + off, len);

	/*
	 * DS PON control frame (GPON OMCI): the PDC steers the OMCC DS GEM to
	 * CPU port 0 with a HW-prepended 16-byte PON header — DA 00:13:25:00:
	 * 00:00, SA 00:13:25:00:00:01, ethertype bytes [12:13] = 0xff,0xf1
	 * (vendor CA_PUC_GLOBAL_LNK_TYPE; 0xfff0 = PLOAM/MPCP, which on this
	 * silicon never reaches the CPU — the GPON MAC handles PLOAM in HW).
	 * Strip the header and hand the OMCI PDU to the GPON driver; these
	 * frames never enter the network stack (vendor ca_ni_rx_fill_pkt
	 * does the same data += 16 / len -= 16).  Ethernet frames cannot
	 * false-match: 0xfff1 is not a real ethertype on this LAN.
	 */
	if (unlikely(buf[off + 12] == 0xff && buf[off + 13] == 0xf1)) {
		cortina_ni_pon_rx_fn fn = READ_ONCE(cortina_ni_pon_rx_cb);

		rx->pon_frames++;
		if (fn && len > 16)
			fn(buf + off + 16, len - 16);
		return rpa;
	}

	/*
	 * DS PON DATA (WAN): a de-encapsulated data-GEM frame carries
	 * HEADER_A.lspid = PON (7) — the lspid our PDC map entry stamps —
	 * while LAN frames carry an NI-port lspid (0..6) and the headerless
	 * swid format parses lspid 0.  Deliver to the GPON WAN netdev.
	 *
	 * ★ HW-L3-forward miss-punt (gated, cortina_ni.hw_l3_fwd=1): once the
	 * DS data GEM is routed through the L3FE (ldpid L3_WAN), a terminating
	 * / not-yet-offloaded frame MISSES the T2 hash and the CLS default
	 * action punts it to CPU_0 — but STG0's LPB profile has already
	 * rewritten HDR_I.lspid PON -> L3_WAN, so it reaches the CPU as
	 * lspid = L3_WAN, not PON.  Deliver that to the same WAN netdev so
	 * DHCP/ICMP-to-router terminating traffic still reaches gpon0 (the
	 * zero-flow no-regression path).  L3_WAN is a WAN-only lspid, so this
	 * cannot steal a LAN frame; it is additionally gated on the armed
	 * engine so gate-off behaviour is byte-identical.
	 */
	if (unlikely(!swid)) {
		/* ONE copy of this decision, shared with the chain path's
		 * completion handler - see cortina_ni_rx_wan_dest() */
		bool is_l3wan = false;
		struct net_device *wan = cortina_ni_rx_wan_dest(hdra_lo,
								&is_l3wan);

		if (wan) {
			if (is_l3wan)
				rx->wan_l3_frames++;
			else
				rx->wan_frames++;
			skb = napi_alloc_skb(&rx->napi, len);
			if (unlikely(!skb)) {
				rx->drop_nobuf++;
				wan->stats.rx_dropped++;
				return rpa;
			}
			skb_put_data(skb, buf + off, len);
			skb->protocol = eth_type_trans(skb, wan);
			napi_gro_receive(&rx->napi, skb);
			wan->stats.rx_packets++;
			wan->stats.rx_bytes += len;
			return rpa;
		}
		/* no WAN netdev registered: fall through to eth0 */
	}

	/*
	 * ★ CPU->LAN egress binding: remember which RJ45 this source MAC is
	 * behind, so a CPU-originated reply to it is stamped for that port
	 * instead of a fixed one (cortina-ni-tx.c).  HEADER_A.lspid is the
	 * ingress NI port for a LAN frame - the same field the shipped
	 * firmware's RX demux uses to select its per-port netdev.  Only for a
	 * header-A frame (swid == 0); the guard inside also refuses a port with
	 * no PHY link, so a wrong lspid can never bind anything.
	 */
	if (likely(!swid && len >= ETH_HLEN))
		cortina_ni_lan_tx_learn(ni, buf + off + ETH_ALEN,
					FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo));

	skb = napi_alloc_skb(&rx->napi, len);
	if (unlikely(!skb)) {
		rx->drop_nobuf++;
		ndev->stats.rx_dropped++;
		return rpa;
	}
	skb_put_data(skb, buf + off, len);
	/* ★ TEMPORARY DIAGNOSTIC (rx_frag_tap) - revert with the params above.
	 * Must run before eth_type_trans(), which pulls the Ethernet header off
	 * the skb; skb->data still points at the copied frame here. */
	if (unlikely(rx_frag_tap > 0))
		cortina_ni_rx_frag_tap(ndev, buf, off, len, desc, pa, dlen,
				       hdra_lo, skb->data);
	skb->protocol = eth_type_trans(skb, ndev);
	napi_gro_receive(&rx->napi, skb);

	rx->frames++;
	rx->bytes += len;
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;
	return rpa;
}

/* ★ TEMPORARY DIAGNOSTIC (rx_frag_tap) - revert with the module params above.
 * noinline and called under an unlikely() guard so the disabled cost is one
 * global load and a predicted-not-taken branch on the 1 Gbps hot path.
 * Wire bytes are read with explicit byte math (the stack must stay
 * endianness-agnostic), never a struct cast. */
static noinline void cortina_ni_rx_frag_tap(struct net_device *ndev,
					    const u8 *buf, u32 off, int len,
					    u64 desc, u32 pa, u32 dlen,
					    u32 hdra_lo, const u8 *copied)
{
	static atomic_t seen = ATOMIC_INIT(0);
	const u8 *f = buf + off;
	u32 ip_id, frag, tot_len, crc_skb, crc_reread, hdra_lo2;
	const u8 *ip;
	int n;

	if (len < ETH_HLEN + 20)
		return;
	/* plain Ethernet/IPv4 only: the ladder is a ping, no tag expected */
	if (f[12] != 0x08 || f[13] != 0x00)
		return;
	ip = f + ETH_HLEN;
	if ((ip[0] >> 4) != 4)
		return;
	frag = ((u32)ip[6] << 8) | ip[7];
	/* a fragment carries a non-zero offset or the more-fragments bit */
	if (!(frag & 0x3fff) && !(frag & 0x2000))
		return;

	n = atomic_inc_return(&seen);
	if (n > rx_frag_tap)
		return;

	ip_id = ((u32)ip[4] << 8) | ip[5];
	tot_len = ((u32)ip[2] << 8) | ip[3];

	/* the bytes as we handed them to the stack, then the same range read
	 * back out of the pool buffer after a delay */
	crc_skb = crc32(0, copied, len);
	if (rx_frag_tap_us > 0)
		udelay(rx_frag_tap_us);
	crc_reread = crc32(0, buf + off, len);
	hdra_lo2 = get_unaligned_be32(buf + CA_NI_RX_HDRA_OFF + 4);

	netdev_info(ndev,
		    "frag_tap#%d pa=%08x eqid=%u sop=%u eop=%u csum_err=%u dlen=%u hdra_pkt_size=%u len=%d off=%u ip{id=%04x frag=%04x tot_len=%u} crc=%08x/%08x %s%s\n",
		    n, pa, (u32)(desc & CA_NI_RX_DESC_EQID),
		    !!(desc & CA_NI_RX_DESC_SOP), !!(desc & CA_NI_RX_DESC_EOP),
		    !!(desc & CA_NI_RX_DESC_CSUM_ERR), dlen,
		    (u32)FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo),
		    len, off, ip_id, frag, tot_len,
		    crc_skb, crc_reread,
		    crc_skb == crc_reread ? "STABLE" : "CHANGED-AFTER-COPY",
		    hdra_lo2 != hdra_lo ? " HDRA-ALSO-CHANGED" : "");
}

/*
 * ★ TEMPORARY DIAGNOSTIC (rx_ds_tap) - revert with rx_frag_tap.  Identity only:
 * which pool the buffer came from (the descriptor's OWN eqid, not one derived
 * from the address), the two independent length sources, and the HEADER_A
 * routing fields that decide which delivery branch this frame is about to take.
 * No re-read and no delay - the question here is arrival, not stability.
 */
static noinline void cortina_ni_rx_ds_tap(struct net_device *ndev,
					  const u8 *buf, u32 off, int len,
					  u64 desc, u32 pa, u32 dlen,
					  u32 hdra_hi, u32 hdra_lo)
{
	static atomic_t seen = ATOMIC_INIT(0);
	u32 lspid = FIELD_GET(CA_NI_HDRA_W1_LSPID, hdra_lo);
	u32 ldpid = FIELD_GET(CA_NI_HDRA_W1_LDPID, hdra_lo);
	u32 etype;
	int n;

	/* downstream = the PON source lspid, or the L3_WAN ldpid the HW-L3 DS
	 * route stamps.  Everything else (the LAN lspids) is not our question. */
	if (lspid != CA_NI_LSPID_PON && ldpid != CA_NI_LSPID_L3_WAN)
		return;

	n = atomic_inc_return(&seen);
	if (n > rx_ds_tap)
		return;

	/* 0xfff1 = the vendor PON link-type marker on a DS OMCI control frame;
	 * anything else here is a de-encapsulated data-GEM frame */
	etype = ((u32)buf[off + 12] << 8) | buf[off + 13];

	netdev_info(ndev,
		    "ds_tap#%d pa=%08x eqid=%u sop=%u eop=%u csum_err=%u dlen=%u hdra_pkt_size=%u len=%d off=%u lspid=%u ldpid=0x%02x cpu_flg=%u deep_q=%u etype=%04x %s\n",
		    n, pa, (u32)(desc & CA_NI_RX_DESC_EQID),
		    !!(desc & CA_NI_RX_DESC_SOP), !!(desc & CA_NI_RX_DESC_EOP),
		    !!(desc & CA_NI_RX_DESC_CSUM_ERR), dlen,
		    (u32)FIELD_GET(CA_NI_HDRA_W1_PKT_SIZE, hdra_lo),
		    len, off, lspid, ldpid,
		    !!(hdra_hi & CA_NI_HDRA_W0_CPU_FLG),
		    !!(hdra_hi & CA_NI_HDRA_W0_DEEP_Q), etype,
		    etype == 0xfff1 ? "OMCI-control" : "data/punt");
}

/* drain one voq's CPU-EPP ring; returns work done, updates rx->rptr[voq] */
static int cortina_ni_rx_poll_voq(struct cortina_ni *ni, unsigned int voq,
				  int budget)
{
	struct cortina_ni_rx *rx = ni->rx;
	__le64 *vring = rx->ring +
		(rx_ring_hi ? CA_NI_RX_RING_HI_OFFSET / sizeof(__le64) : 0) +
		voq * CA_NI_RX_RING_SLOTS_PER_VOQ;
	u32 recycle[CA_NI_RX_RECYCLE_MAX];
	unsigned int nrecycle = 0, i;
	u32 rptr = rx->rptr[voq];
	u32 wptr;
	int work = 0;

	/* never consume more buffers in one poll than we can hand back: an
	 * overflow would leak them out of the pool until RX starves.  This clamp
	 * is also why a chain must be able to span two polls: it can cut one in
	 * half, so the per-voq chain state persists rather than the poll spinning
	 * on for the rest of the segments.  (The shipped firmware takes the other
	 * route - it keeps chain state on the stack and walks the ring for the
	 * continuations without re-reading the write pointer or checking
	 * availability, so it can read descriptors the hardware has not written
	 * yet.  Persisting the state is both bounded and cheaper.) */
	if (budget > CA_NI_RX_RECYCLE_MAX)
		budget = CA_NI_RX_RECYCLE_MAX;

	wptr = cortina_ni_rx_wptr_voq(ni, voq);
	dma_rmb();	/* descriptor reads after the pointer read */

	while (work < budget) {
		u64 desc;
		u32 pa;

		if (rptr == wptr) {
			wptr = cortina_ni_rx_wptr_voq(ni, voq);
			dma_rmb();
			if (rptr == wptr)
				break;
		}

		desc = le64_to_cpu(READ_ONCE(vring[rptr / CA_NI_RX_DESC_SIZE]));
		WRITE_ONCE(vring[rptr / CA_NI_RX_DESC_SIZE], 0);
		rptr = (rptr + CA_NI_RX_DESC_SIZE) & (CA_NI_RX_RING_BYTES - 1);
		work++;

		pa = cortina_ni_rx_frame(ni, voq, desc);
		if (cpu_pool_push && pa)
			/* Return the buffer to the pool the HARDWARE says it
			 * came from - desc[3:0] - not one derived from its
			 * address.  Deriving it assumes a pool layout instead
			 * of reading the one fact the descriptor already
			 * carries, and every descriptor observed on this board
			 * reports eqid 0xF while the driver only configures
			 * EQ5/EQ6, so the two disagree.  Packed exactly as the
			 * doorbell encodes it: the PA is 128-byte aligned, so
			 * the low nibble is free for the eqid. */
			recycle[nrecycle++] = (pa & CA_NI_QM_PUSH_ADDR) |
					      (u32)(desc & CA_NI_RX_DESC_EQID);
	}

	dma_wmb();	/* stock: dmb oshst before the rdptr store */
	writel(rptr, ni_base(ni) +
	       CA_NI_QM_EPP64_RDPTR(CA_NI_RX_CPU_PORT, voq));
	rx->rptr[voq] = rptr;
	rx->voq_frames[voq] += work;	/* spy: flow→voq spread (order check) */

	/*
	 * Return the buffers we have finished with, one batch per poll (the
	 * vendor's ca_ni_alloc_scatter_refill mode-3 batch, after the read
	 * pointer).  ORDER IS THE WHOLE POINT: every frame above has already
	 * been copied into its skb, so from admission until this loop the
	 * buffer belongs to the CPU and the QM cannot hand it to an incoming
	 * frame.  That interval is what the hardware-managed pool never had.
	 */
	for (i = 0; i < nrecycle; i++) {
		if (unlikely(cortina_ni_rx_push_buf(ni,
				recycle[i] & CA_NI_QM_PUSH_EQID,
				recycle[i],
				CA_NI_RX_PUSH_TIMEOUT_NAPI_US))) {
			rx->push_fail++;
			net_err_ratelimited("%s: RX pool push timeout for pa=%08x - buffer lost, pool will shrink\n",
					    netdev_name(rx->netdev),
					    recycle[i]);
		}
	}
	return work;
}

static int cortina_ni_rx_poll(struct napi_struct *napi, int budget)
{
	struct cortina_ni_rx *rx =
		container_of(napi, struct cortina_ni_rx, napi);
	struct cortina_ni *ni = rx->ni;
	unsigned int voq;
	int work = 0, more;

	rx->polls++;

	/* ★ Drain ALL 8 CPU-port VOQs: the deep_q frame may land on any voq (by
	 * cos/priority), and stock arms every voq's CPU-EPP FIFO.  Round-robin
	 * within the shared NAPI budget. */
	for (voq = 0; voq < CA_NI_RX_VOQ_COUNT && work < budget; voq++)
		work += cortina_ni_rx_poll_voq(ni, voq, budget - work);

	if (work < budget && napi_complete_done(napi, work)) {
		cortina_ni_rx_irq_set(ni, true);
		/* close the enable-vs-new-frame race across all voqs */
		more = 0;
		for (voq = 0; voq < CA_NI_RX_VOQ_COUNT; voq++)
			if (cortina_ni_rx_wptr_voq(ni, voq) != rx->rptr[voq])
				more = 1;
		if (more && napi_schedule(&rx->napi))
			cortina_ni_rx_irq_set(ni, false);
	}
	return work;
}

/* ------------------------------------------------------------------ */
/* RX steer: the full forwarding-engine path (stock golden config)      */
/* ------------------------------------------------------------------ */

/* program one PLE default-forward table entry: redirect a lookup-miss
 * traffic type of <lspid> to CPU port 0 (indirect read-modify-write) */
static int cortina_ni_rx_ple_dft_fwd(struct cortina_ni *ni, u32 lspid,
				     u32 type)
{
	void __iomem *acc = ni_base(ni) + CA_NI_PLE_DFT_FWD_ACCESS;
	u32 addr = lspid << 2 | type;
	u32 val;
	int ret;

	/* (0x1560/0x156c are NOT a DFT_FWD control block - the old writes here
	 * corrupted the VLAN check-id map; see cortina-ni-regs.h.  The default-forward
	 * table itself is programmed via the ACCESS/DATA indirect protocol below.) */
	ret = readl_poll_timeout(acc, val, !(val & CA_NI_PLE_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	/* latch the entry into DATA */
	writel(CA_NI_PLE_ACCESS_GO | addr, acc);
	ret = readl_poll_timeout(acc, val, !(val & CA_NI_PLE_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	val = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);
	/* ★ Stock LAN->CPU base forwarding, VERBATIM: 0x1832 decode (Elnath field
	 * layout, corrected 2026-07-15): valid=b12 + redir_en=b11 +
	 * mc_group_id[10:1]=0x19 (= AAL_LPORT_L3_LAN) + deny=b0=0, i.e. redirect the
	 * lookup-miss to L3_LAN -> RMU -> L3FE -> L3-CLS ethertype trap -> CPU_0.
	 * (The earlier "redir_ldpid[5:0]=0x32" decode here was the 8277B-only layout
	 * and WRONG on Elnath - there is no ldpid-0x32 target in this word.  The
	 * MC_GROUP_ID[10:1] field decode was the correct one all along.) */
	val = CA_NI_RX_DFT_FWD_CPU_VAL;
	writel(val, ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);

	/* write it back */
	writel(CA_NI_PLE_ACCESS_GO | CA_NI_PLE_ACCESS_WRITE | addr, acc);
	ret = readl_poll_timeout(acc, val, !(val & CA_NI_PLE_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	/* ★ READ IT BACK (2026-08-04).  The indirect protocol ACKs by clearing GO
	 * whether or not `addr` names a real entry, so an index past the end of the
	 * table completes "successfully" and changes nothing -- the silent-wrong-
	 * offset failure this driver has already paid for.  The caller writes a
	 * LOGICAL port index here (the PON one), so this is exactly where it must
	 * be proven rather than assumed. */
	writel(CA_NI_PLE_ACCESS_GO | addr, acc);
	ret = readl_poll_timeout(acc, val, !(val & CA_NI_PLE_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		return ret;
	val = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);
	if (val != CA_NI_RX_DFT_FWD_CPU_VAL) {
		dev_err(ni->dev,
			"PLE dft-fwd entry %#x (lspid %u type %u) read back %#010x, wrote %#010x -- the entry did not take\n",
			addr, lspid, type, val, CA_NI_RX_DFT_FWD_CPU_VAL);
		return -EIO;
	}
	return 0;
}

/* Async-SError fault-attribution helper: full barrier so the suspect MMIO
 * write has posted, then a short delay so a delayed AXI external-abort (async
 * SError) is taken HERE - before the next marker prints.  So the LAST marker
 * line in the log before a panic pins the exact faulting access. */
static void cortina_ni_rx_settle(void)
{
	mb();
	mdelay(2);
}

/* Generic indirect-table store: write ACCESS = GO|WR|idx, poll GO clear
 * (stock DO_INDIRCT_OP write path).  Bounded + non-fatal. */
static void cortina_ni_rx_ind_store(struct cortina_ni *ni, u32 access_reg, unsigned int idx)
{
	void __iomem *acc = ni_base(ni) + access_reg;
	u32 v;

	writel(CA_NI_IND_ACCESS_GO | CA_NI_IND_ACCESS_WR | idx, acc);
	if (readl_poll_timeout(acc, v, !(v & CA_NI_IND_ACCESS_GO),
			       CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US))
		dev_warn(ni->dev, "indirect store @0x%x[%u] GO stuck (0x%08x)\n",
			 access_reg, idx, v);
}

/* Generic indirect-table read: write ACCESS = GO|idx (rbw=0), poll GO clear;
 * the entry is then latched in the DATA registers for the caller to read.
 * Bounded + non-fatal. */
static void cortina_ni_rx_ind_read(struct cortina_ni *ni, u32 access_reg, unsigned int idx)
{
	void __iomem *acc = ni_base(ni) + access_reg;
	u32 v;

	writel(CA_NI_IND_ACCESS_GO | idx, acc);
	if (readl_poll_timeout(acc, v, !(v & CA_NI_IND_ACCESS_GO),
			       CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US))
		/* RATELIMITED: this runs from /proc show paths, which a soak or a
		 * benchmark harness polls in a loop - an un-ratelimited warn there
		 * floods the serial console the witnesses are read over. */
		dev_warn_ratelimited(ni->dev,
				     "indirect read @0x%x[%u] GO stuck (0x%08x)\n",
				     access_reg, idx, v);
}

/*
 * ★★ ROOT-CAUSE FIX for the constant blackhole (rx_fe ldpid stuck 0x1f):
 * program the L2FE PER-PORT PROFILE tables the stock init sets and ours never
 * touched.  With them unprogrammed the pipeline FORCES every ingress frame to
 * the blackhole ldpid BEFORE the FDB/DFT_FWD forwarding decision:
 *   - ILPB[lspid].stp_mode = 0 = STP DROP state (want 3 = forward+learn);
 *   - MMSHP[lspid] = 0 = port-isolation bitmap allows NO destination port;
 *   - ELPB[ldpid].egr_stp = 0 = egress STP blocked.
 * That is why every table we wrote (DFT_FWD 0x1832, PDPID, my-MAC, redirect)
 * read back correct yet had zero effect - the frame died upstream, constantly.
 *
 * Values are the stock init state (validated against live-stock leftovers of
 * the last-touched entries).  Table protocol = DATA regs, then ACCESS=GO|WR|i.
 * Idempotent (absolute values), so safe from the link-up re-arm path.
 */
/* Stock L2FE VLAN membership check-id map (aal_l2_vlan.c __g_l2_vlan_port_map):
 * lport -> membership check-id.  NI0-7 -> 0-7, CPU_0 -> 8, CPU_1 -> 9,
 * L3_WAN/L3_LAN (0x18/0x19) -> 15, GEM (0x20+) -> 7.  Programmed into the
 * MMSHP_CHK_ID_MAP so the membership check qualifies LAN<->CPU forwarding. */
static const u8 ca_ni_vlan_chkid_map[CA_NI_L2FE_LPORT_COUNT] = {
	 0, 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0,
	 8, 9, 0,10,11,12,13,14,15,15, 0, 0, 0, 0, 0, 0,
	 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
};

static void cortina_ni_rx_port_profiles_init(struct cortina_ni *ni)
{
	u32 i, d0, d1, d2;

	dev_info(ni->dev, "port-prof: programming IPPB/MMSHP/ILPB/ELPB (64 lports)\n");

	for (i = 0; i < CA_NI_L2FE_LPORT_COUNT; i++) {
		/* IPPB: physical -> logical source-port map, identity */
		writel(i, ni_base(ni) + CA_NI_L2FE_IPPB_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_IPPB_ACCESS, i);

		/* MMSHP: allowed-ldpid bitmap = all-but-self (isolation off) */
		writel(i >= 32 ? ~BIT(i - 32) : ~0u,
		       ni_base(ni) + CA_NI_L2FE_MMSHP_DATA1);
		writel(i < 32 ? ~BIT(i) : ~0u,
		       ni_base(ni) + CA_NI_L2FE_MMSHP_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_MMSHP_ACCESS, i);

		/* MMSHP check-id map: lport -> VLAN membership check-id (stock
		 * __g_l2_vlan_port_map).  Unprogrammed (our old bug wrote garbage
		 * into entry 63 only), the membership check qualifies nothing and
		 * LAN frames never reach the CPU port. */
		writel(ca_ni_vlan_chkid_map[i],
		       ni_base(ni) + CA_NI_L2FE_CHKID_MAP_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_CHKID_MAP_ACCESS, i);

		/* ILPB ingress profile: stp_mode=FWD+LEARN + the stock defaults */
		if (i == CA_NI_LPORT_MC)
			d2 = CA_NI_L2FE_ILPB_D2_MC;
		else if (i < CA_NI_LPORT_GEM_FIRST)
			d2 = CA_NI_L2FE_ILPB_D2_PORT;
		else
			d2 = CA_NI_L2FE_ILPB_D2_GEM;
		d1 = CA_NI_L2FE_ILPB_D1_INIT;
		if (i <= CA_NI_LPORT_ETH_NI6 ||
		    (i >= CA_NI_LPORT_CPU_0 && i <= CA_NI_LPORT_CPU_7))
			d1 |= CA_NI_L2FE_ILPB_D1_STAMOVE;
		writel(i >= CA_NI_LPORT_GEM_FIRST ? CA_NI_L2FE_ILPB_D4_WAN : 0,
		       ni_base(ni) + CA_NI_L2FE_ILPB_DATA4);
		writel(CA_NI_L2FE_ILPB_D3_INIT, ni_base(ni) + CA_NI_L2FE_ILPB_DATA3);
		writel(d2, ni_base(ni) + CA_NI_L2FE_ILPB_DATA2);
		writel(d1, ni_base(ni) + CA_NI_L2FE_ILPB_DATA1);
		writel(CA_NI_L2FE_ILPB_D0_INIT, ni_base(ni) + CA_NI_L2FE_ILPB_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ILPB_ACCESS, i);

		/* ELPB egress profile: egr STP forward + vlan-aware (+dest_wan
		 * on the PON-side dest ports and the L3_LAN dest) */
		d0 = (i >= CA_NI_LPORT_GEM_FIRST || i == CA_NI_LPORT_L3_LAN) ?
		     CA_NI_L2FE_ELPB_D0_WAN : CA_NI_L2FE_ELPB_D0_LAN;
		writel(0, ni_base(ni) + CA_NI_L2FE_ELPB_DATA1);
		writel(d0, ni_base(ni) + CA_NI_L2FE_ELPB_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ELPB_ACCESS, i);
	}

	/* the L2FE direct config regs stock also moves off hardware default
	 * (tier-1 live-stock values; incl. PLE_CTL skip_port_lpbk_chk+pon_mode
	 * and PARSER_CTRL arp_op_filter_dis) */
	writel(CA_NI_L2FE_GLB_CTRL_STOCK, ni_base(ni) + CA_NI_L2FE_GLB_CTRL);
	writel(CA_NI_L2FE_TPID_S_STOCK, ni_base(ni) + CA_NI_L2FE_PP_TPID_CMP_S);
	writel(CA_NI_L2FE_TPID_O_STOCK, ni_base(ni) + CA_NI_L2FE_PP_TPID_CMP_O);
	writel(CA_NI_L2FE_PARSER_CTRL_STOCK,
	       ni_base(ni) + CA_NI_L2FE_PP_PARSER_CTRL);
	writel(CA_NI_L2FE_L2_LEARNING_STOCK,
	       ni_base(ni) + CA_NI_L2FE_PLC_L2_LEARNING);
	writel(CA_NI_L2FE_VLAN_MODE_STOCK, ni_base(ni) + CA_NI_L2FE_PLC_VLAN_MODE);
	writel(CA_NI_L2FE_PLE_CTL_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_CTL);
	writel(CA_NI_L2FE_UNKWN_VLAN_DFT1_STOCK,
	       ni_base(ni) + CA_NI_L2FE_PLE_UNKWN_VLAN_DFT1);
	/* PLE regs stock programs that ours skipped (golden 2026-07-15) */
	writel(CA_NI_L2FE_PLE_TRUNK_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_TRUNK0);
	writel(CA_NI_L2FE_PLE_TRUNK_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_TRUNK1);
	writel(CA_NI_L2FE_PLE_HD_FF_STOCK, ni_base(ni) + CA_NI_L2FE_PLE_HD_FF_CTL);

	/* readback proof (genuine indirect reads) for the boot log */
	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ILPB_ACCESS, CA_NI_RX_PORT);
	d2 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA2);
	d1 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA1);
	d0 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA0);
	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MMSHP_ACCESS, CA_NI_RX_PORT);
	dev_info(ni->dev,
		 "port-prof: ilpb[0] d2=0x%08x d1=0x%08x d0=0x%08x (stp=%lu want 3) mmshp[0]=%08x_%08x ple_ctl=0x%08x\n",
		 d2, d1, d0, FIELD_GET(CA_NI_L2FE_ILPB_STP_MODE, d2),
		 readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA1),
		 readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA0),
		 readl(ni_base(ni) + CA_NI_L2FE_PLE_CTL));
}

/*
 * ★ Program the L2FE PDPID_MAP so our redir dest (CPU_0, LDPID 0x10) resolves to
 * the CPU physical dest port (0x09).  The vendor programs this table in
 * aal_port init; our driver never did, so a redir dest never reached the CPU.
 * Indirect: write DATA=pdpid, then ACCESS=GO|WR|ldpid, poll GO clear.
 */
static void cortina_ni_rx_redir_ldpid_set(struct cortina_ni *ni, u8 idx,
					  u8 dest_ldpid);

/* Stock golden PDPID_MAP (tier-1 stock_l2fe_forwarding.txt): ldpid -> physical dest */
static const struct { u8 ldpid, pdpid; } cortina_ni_rx_pdpid_map[] = {
	{ 0x08, 0x0c }, { 0x09, 0x0c }, { 0x0d, 0x0c }, { 0x10, 0x09 },
	{ 0x19, 0x0d }, { 0x1d, 0x09 }, { 0x1f, 0x0f }, { 0x32, 0x08 },
	/* ★★★ build68: REVERT build67's [0x19]/[0x32]->0x00.  Stock ground truth (working
	 * CPU-RX boot, devmem): the CPU frame does NOT reach the CPU via a PDPID unicast
	 * dest at all - it rides the MC_FIB FLOOD: DFT_FWD 0x1832 -> mcgid 0x19 -> MC_FIB
	 * [0x19] D2=0x0B (flood bitmask incl a CPU member) -> CPU copy.  So the PDPID map is
	 * the plain STOCK literal ([0x19]=0x0d L3_LAN unicast, [0x32]=0x08 QM); the CPU copy
	 * comes from the flood, not this table.  build67's unicast-to-CPU-port theory was
	 * wrong (rmu_rx still 0). */
};

/*
 * ★★ 2026-07-15 THE CPU-RX FIX: zero the ARB FLOW_DBUF table to match STOCK.
 * With ARB_CTRL.dbuf_sel=1 (stock) this table is THE deep_q source; build100
 * marked all 16 entries 0x0F (every flow deep_q), which diverted every LAN
 * frame into the (broken for us) deep-queue path BEFORE it could enter L3FE -
 * stock's l3fe_rx(0xa9bc) climbs, ours read 0.  Stock keeps the whole table 0:
 * its CPU delivery is DFT_FWD 0x1832 -> mcgid 0x19 (L3_LAN) -> RMU -> L3FE ->
 * L3-CLS ethertype trap -> CPU_0, with NO deep-queue marking.  Keep the
 * explicit zero-write + before/after readback so /proc + the boot log still
 * show the table (and catch anything re-marking it).
 */
static void cortina_ni_rx_flow_dbuf_init(struct cortina_ni *ni)
{
	u32 b[8], a[8];
	unsigned int i;

	for (i = 0; i < 8; i++) {
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS, i);
		b[i] = readl(ni_base(ni) + CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
	}
	for (i = 0; i < CA_NI_L2FE_ARB_FLOW_DBUF_ENTRIES; i++) {
		writel(0, ni_base(ni) + CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS, i);
	}
	for (i = 0; i < 8; i++) {
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS, i);
		a[i] = readl(ni_base(ni) + CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
	}
	dev_info(ni->dev,
		 "flow-dbuf(0x165c/0x1660) before[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		 b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
	dev_info(ni->dev,
		 "flow-dbuf AFTER[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x (want all 0 = stock, no deep_q marks)\n",
		 a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
}

static void cortina_ni_rx_arb_deepq_init(struct cortina_ni *ni)
{
	u32 d0, d1, i, e;

	/*
	 * ★★ THE CPU-RX handoff (rev. 2026-07-11): resolve every CPU-bound frame to
	 * physical dest PDPID = CPU (0x09), NOT QM (0x08).  Tier-1 golden
	 * (STOCK_cpu_rx_golden): physical dest 0x09 = CPU; the RMU0 admits ONLY
	 * CPU-dest frames into the CPU pool (EQ13/EQ14) -> CPU-EPP -> NAPI.  Our old
	 * PDPID=QM(0x08) routing sent frames down the QM/deep-queue (ES port 7) path,
	 * which the coordinator proved is a phantom for CPU-RX (cb-occupancy flat 0 on
	 * stock while the RMU admits) -> the RMU never saw them (0x6900=0).  So all
	 * three CPU-reaching classifier outputs below map to PDPID=CPU(0x09):
	 *   - REDIR_LDPID (blackhole/DLF fall-through, ldpid 0x00)
	 *   - 0x32 (DFT_FWD broadcast/DLF redirect target)
	 *   - 0x19 (my-MAC / L3-hit unicast)
	 * The dbuf/PORT_DBUF/deep-queue belts below are now inert for CPU-RX (dbuf_dpid
	 * =8 does not grab a PDPID-9 frame) - kept only as harmless no-ops on the
	 * L3FE-free fall-through path.
	 */
	/* NOTE: dbuf_sel (bit1) is set to match stock (0x89c71c82) in
	 * cortina_ni_rx_deepq_sched_init - do NOT clear it here. */

	/* ★ 2026-07-15: PORT_DBUF[0] = 0 = STOCK.  The old DBUF_FLG|LDPID_VLD mark
	 * was the second half of the deep-queue regression (with the FLOW_DBUF 0x0F
	 * marks): stock never deep-buffer-marks the LAN->CPU path.  Explicit zero
	 * write so a stale mark from a previous boot cannot survive. */
	writel(0, ni_base(ni) + CA_NI_L2FE_ARB_PORT_DBUF_DATA);
	cortina_ni_rx_ind_store(ni, CA_NI_L2FE_ARB_PORT_DBUF_ACCESS, 0);

	/* ★★ FULL PDPID_MAP = STOCK GOLDEN literals (tier-1 stock_l2fe_forwarding.txt,
	 * "SOLVED").  The map is the plain stock literal.  ★ 2026-07-15 decode fix:
	 *   DLF/broadcast: DFT_FWD 0x1832 = redirect to mc_group_id[10:1]=0x19 (L3_LAN)
	 *                  -> [0x19]=0x0D -> RMU -> L3FE -> L3-CLS trap -> CPU_0.  (The
	 *                  old "-> ldpid 0x32 -> QM DeepQ" chain here was the wrong
	 *                  8277B ldpid[5:0] decode of 0x1832.)
	 *   my-MAC/L3-hit: -> CPU LPORT 0x10/0x1d -> [0x10]/[0x1d]=0x09 (CPU).
	 * [0x1f]=0x0F (blackhole/drop).  Each written for all 4 {my_mac,dbuf} index
	 * combos so the frame resolves identically whatever its flags. */
	for (e = 0; e < ARRAY_SIZE(cortina_ni_rx_pdpid_map); e++) {
		u8 pdpid = cortina_ni_rx_pdpid_map[e].pdpid;

		/* ★ build101: override L3_LAN (ldpid 0x19) -> pdpid_l3lan (default 0x00 = CPU
		 * port 0, the prior-session never-tested fix).  Routes the flooded LAN broadcast
		 * to a CPU port directly, bypassing the L3FE/CLS trap, to test whether a
		 * CPU-destined frame gets a buffer attached (descriptor PA != 0). */
		if (cortina_ni_rx_pdpid_map[e].ldpid == CA_NI_RX_L3LAN_LDPID)
			pdpid = pdpid_l3lan;

		/* build33: write the STOCK PDPID for every ldpid - NO CPU override.  The CPU
		 * frame reaches QM by resolving to ldpid 0x32 ([0x32]=0x08), the stock path,
		 * NOT by bodging [0x19]->0x08 (which took the wrong ES-port route). */
		for (i = 0; i < 4; i++) {
			u32 idx = cortina_ni_rx_pdpid_map[e].ldpid |
				  ((i & 1) ? CA_NI_L2FE_PDPID_IDX_DBUF : 0) |
				  ((i & 2) ? CA_NI_L2FE_PDPID_IDX_MYMAC : 0);

			writel(FIELD_PREP(CA_NI_L2FE_PDPID_MAP_PDPID, pdpid),
			       ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA);
			cortina_ni_rx_ind_store(ni, CA_NI_L2FE_PDPID_MAP_ACCESS, idx);
		}
	}
	dev_info(ni->dev, "pdpid: map written ([0x19]=0x%02x (build101 override, stock=0x0d), [0x32]=0x08 QM); L3_LAN routed to pdpid 0x%02x\n",
		 pdpid_l3lan, pdpid_l3lan);

	/*
	 * ★★ REMOVED the LDPID->CPU redirect (was REDIR_LDPID[0x19]/[0x1f]->0x10).
	 * a00f891's ca-ne.ko disasm proved the old "cpu_flg latches from a CPU ldpid
	 * 0x10-0x17" premise is FALSE on Elnath: aal_l3qm_init_upper_ldpid_map is an
	 * empty stub and RMU0 admit is gated on deep_q (frame PDPID==dbuf_dpid=8), NOT
	 * cpu_flg.  Worse, that redirect was the ACTIVE BUG (confirmed on HW build14):
	 * a DFT_FWD=0x1832 frame carries mcgid 0x19, but REDIR_LDPID[0x19]->0x10
	 * intercepted it BEFORE MC replication, so it resolved to ldpid 0x10 ->
	 * PDPID_MAP[0x10]=9 (CPU-direct, misses the deep-buffer) -> never reached RMU0
	 * (member 0x32 took, yet bm-hdr stayed ldpid 0x10, rmu0_rx=0).  With the
	 * redirect gone the frame replicates via MCE[0x19] member 0x32 ->
	 * PDPID_MAP[0x32]=0x08 -> deep_q=1 -> deep-buffer -> RMU0.  Readback ONLY (no
	 * write) so the boot confirms REDIR_LDPID[0x19] is no longer forced to CPU.
	 */
	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_REDIR_LDPID_ACCESS, CA_NI_RX_L3LAN_LDPID);
	dev_info(ni->dev,
		 "arb-deepq: REDIR_LDPID[0x19] data=0x%08x (redirect REMOVED; want NOT rdir_en|0x%x)\n",
		 readl(ni_base(ni) + CA_NI_L2FE_REDIR_LDPID_DATA), CA_NI_RX_CPU_LDPID);

	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS, CA_NI_RX_QM_REDIR_LDPID);
	d0 = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
	     CA_NI_L2FE_PDPID_MAP_PDPID;
	cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
			       CA_NI_L2FE_PDPID_IDX_DBUF | CA_NI_RX_QM_REDIR_LDPID);
	d1 = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
	     CA_NI_L2FE_PDPID_MAP_PDPID;
	dev_info(ni->dev,
		 "arb-deepq: base LAN->CPU: PDPID_MAP[0x32]{dbuf0}=0x%x {dbuf1}=0x%x arb_ctrl=0x%08x (want both 0x%x=CPU); DFT_FWD set to 0x%04x\n",
		 d0, d1, readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL),
		 CA_NI_RX_CPU_PDPID, CA_NI_RX_DFT_FWD_CPU_VAL);
}

/*
 * ★ De-secure the shared CapSRAM (TrustZone).  BL1 leaves some CapSRAM segments
 * SECURE; stock U-Boot clears CAPSRAM_TZCONTROL=0 (all NS) before ca_init so the
 * non-secure kernel can reach the CapSRAM that backs the NI MCE index table and
 * the L2FE ARB indirect tables (REDIR_LDPID/MC_FIB).  Our minimal U-Boot skips
 * it, so a NS access to a secured segment SErrors (MCE DATA write) or stalls the
 * indirect GO (ARB REDIR_LDPID).  This one write is the shared fix for both.
 * Markers + settle: if 0xf4322000 is itself EL3-write-protected the access
 * faults here (last CAPSRAM marker pins it); the readback tells us whether the
 * write TOOK (0) or was ignored (need the U-Boot/SMC route instead).
 */
/*
 * ★ Deassert GLOBAL_FABRIC_RESET bit5 - the NI-MCE (multicast-expansion) sub-
 * block reset.  The stock differential (devmem, 40-bit) pinned it: ours
 * 0xa4=0x00079F20 (bit5 SET) vs stock 0x00079F00 (bit5 CLEAR).  With bit5 set
 * the NI-MCE RAM (0xaa6x) is held in fabric-reset, so the plain mce_indx DATA
 * write async-SErrors.  DEASSERT-ONLY: read-clear-bit5-write, NO assert/pulse
 * (so no storm), FIRST in steer_init before any MCE access, to match stock.
 */
static void __maybe_unused cortina_ni_rx_fabric_mce_ungate(struct cortina_ni *ni)
{
	void __iomem *glb = ni->win[CA_NI_WIN_GLB];
	u32 before, after;

	if (!glb) {
		dev_warn(ni->dev, "fabric-ungate: no GLB window\n");
		return;
	}
	before = readl(glb + CA_NI_GLB_BLOCK_RESET_EXT);
	dev_emerg(ni->dev, "FABRIC 1: reset(a4)=0x%08x, clearing bit5 (NI-MCE)\n",
		  before);
	writel(before & ~CA_NI_GLB_BLOCK_RESET_EXT_MCE, glb + CA_NI_GLB_BLOCK_RESET_EXT);
	cortina_ni_rx_settle();
	after = readl(glb + CA_NI_GLB_BLOCK_RESET_EXT);
	dev_emerg(ni->dev, "FABRIC 2: reset(a4) now 0x%08x (stock=0x00079f00)\n",
		  after);
}

static void __maybe_unused cortina_ni_rx_capsram_desecure(struct cortina_ni *ni)
{
	void __iomem *cap;
	u32 before, after;

	dev_emerg(ni->dev, "CAPSRAM 1: ioremap TZCONTROL @0x%llx\n",
		  (unsigned long long)CA_NI_CAPSRAM_PHYS);
	cap = ioremap(CA_NI_CAPSRAM_PHYS, 0x10);
	if (!cap) {
		dev_err(ni->dev, "capsram: ioremap failed\n");
		return;
	}

	dev_emerg(ni->dev, "CAPSRAM 2: read TZCONTROL\n");
	before = readl(cap + CA_NI_CAPSRAM_TZCONTROL);
	cortina_ni_rx_settle();

	dev_emerg(ni->dev, "CAPSRAM 3: TZCONTROL=0x%08x, writing 0 (all NS)\n",
		  before);
	writel(0, cap + CA_NI_CAPSRAM_TZCONTROL);
	cortina_ni_rx_settle();

	after = readl(cap + CA_NI_CAPSRAM_TZCONTROL);
	dev_emerg(ni->dev, "CAPSRAM 4: TZCONTROL now 0x%08x (write %s)\n",
		  after, after ? "IGNORED - EL3-protected?" : "TOOK");
	iounmap(cap);
}

/*
 * ★ MC-flood-to-CPU (needs the CapSRAM de-secure above to not SError).  Build
 * one mcgid whose replication set includes the CPU: MC_FIB[16].ldpid=16 (per-
 * copy CPU action) + ni_mce_indx[G].mc_vec bit 16 (mc_vec bit i selects
 * MC_FIB[i]).  DFT_FWD points port-0 BC/DLF at mcgid G (redir_en=0).  MCFLOOD
 * markers + settle pin any residual faulting access.
 */
static void __maybe_unused cortina_ni_rx_mc_flood_init(struct cortina_ni *ni)
{
	/* MC_FIB[16]: per-copy action -> ldpid 16 (CPU); no VLAN edit */
	dev_emerg(ni->dev, "MCFLOOD 1: MC_FIB[%u] DATA write\n", CA_NI_RX_CPU_LDPID);
	writel(0, ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA0);
	writel(FIELD_PREP(CA_NI_L2FE_MC_FIB_LDPID, CA_NI_RX_CPU_LDPID),
	       ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA1);
	writel(0, ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA2);
	cortina_ni_rx_settle();
	dev_emerg(ni->dev, "MCFLOOD 2: MC_FIB[%u] ACCESS store\n", CA_NI_RX_CPU_LDPID);
	cortina_ni_rx_ind_store(ni, CA_NI_L2FE_MC_FIB_ACCESS, CA_NI_RX_CPU_LDPID);
	cortina_ni_rx_settle();

	/* ni_mce_indx[G].mc_vec = bit 16 (CPU) - the write that SErrored pre-fix */
	dev_emerg(ni->dev, "MCFLOOD 3: mce_indx[%u] DATA write (was the SError)\n",
		  CA_NI_RX_FLOOD_MCGID);
	writel(BIT(CA_NI_RX_CPU_LDPID), ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0);
	writel(0, ni_base(ni) + CA_NI_NI_MCE_INDX_DATA1);
	cortina_ni_rx_settle();
	dev_emerg(ni->dev, "MCFLOOD 4: mce_indx[%u] ACCESS store\n", CA_NI_RX_FLOOD_MCGID);
	cortina_ni_rx_ind_store(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_FLOOD_MCGID);
	cortina_ni_rx_settle();

	/* pollable read-back for the boot log */
	cortina_ni_rx_ind_read(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_FLOOD_MCGID);
	dev_emerg(ni->dev, "MCFLOOD 5 (survived): mcgid=%u mc_vec.lo=0x%08x\n",
		  CA_NI_RX_FLOOD_MCGID,
		  readl(ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0));
}

/*
 * ★ THE guaranteed L3FE-free CPU trap.  Our DFT_FWD redirects UUC/DLF frames to
 * MCE group CA_NI_RX_MCGID; that group must be a REAL, non-empty group (group 0
 * is the reserved all-zero group whose empty replication set produced the
 * AAL_LPORT_BLACKHOLE(0x1f) drop we saw).  Build a ONE-member group:
 *   ARB-FIB[b] = one copy, dest ldpid = DeepQ_0 (no VLAN/MAC edits)
 *   MCE[G].mc_vec = bit b   (mc_vec bit i selects ARB-FIB[i])
 * A DLF frame then replicates to exactly one copy destined for DeepQ_0 ->
 * PDPID=QM(0x08) -> dbuf_dpid deep-queue -> ES port 7 -> RMU -> CPU.  All L2FE.
 *
 * The mce_indx write (0xaaf4) once async-SError'd - but at the WRONG rtl8277c
 * offset (0xaa64); 0xaaf4 is the correct Elnath offset (same map family as the
 * MC_FIB/PDPID indirect tables that already work).  dev_emerg markers + settle
 * barriers bracket the write so a residual fault pins the exact line; this runs
 * at init off the volatile TFTP image, so a cold boot always recovers.
 */
static void cortina_ni_rx_mc_group_init(struct cortina_ni *ni)
{
	/* ★★ build33: MCE[0x19] must be EMPTY (stock devmem ground-truth) so a
	 * DFT_FWD=0x1832 frame (mc=1, mcgid[10:1]=0x19, raw ldpid[10:0]=0x32) falls
	 * THROUGH the empty MC group to unicast ldpid 0x32 -> PDPID_MAP[0x32]=0x08
	 * (QM) -> the exact stock ES-port -> L3QM -> RMU0 -> CPU, matching stock's
	 * working CPU frame.  build14/17 wrongly added member 0x32 here, which
	 * resolved the frame to the mcgid ldpid 0x19 instead (PDPID_MAP[0x19]=0x0D
	 * L3-LAN; even our build15 0x19->0x08 override kept the RAW ldpid at 0x19, so
	 * the per-LDPID L3QMRX/deep-queue demux still used entry[0x19] not entry[0x32]
	 * -> l2tm_tx climbed but ni2qm_rx/0xa9fc stayed 0).  The L2FE MC-FIB
	 * member writes are also dropped so both MC tables stay at their empty reset. */
	writel(0, ni_base(ni) + CA_NI_NI_MCE_INDX_DATA1);
	writel(0, ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0);
	cortina_ni_rx_settle();
	cortina_ni_rx_ind_store(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_DFT_FWD_MCGID);
	cortina_ni_rx_settle();

	/* ★ 2026-07-15 relabeled (build70 misread): this table @0x1634 is NOT the
	 * MC_FIB (real MC_FIB ACCESS = 0x1644, EMPTY on stock - there is NO flood-to-
	 * CPU; the CPU copy comes via DFT_FWD 0x1832 -> L3_LAN -> L3FE -> CLS trap).
	 * 0x1634 is a different table (likely NON_KNOWN_POL_MAP, rtl8277c 0x1624 +
	 * 0x10).  The values below ARE stock's own content of THAT table (tier-1
	 * devmem, dev/x400axf/stock_golden_qm.txt), so writing them is a plain
	 * stock-match of it - kept byte-identical to the proven boots.  The two zero
	 * latch-writes to 0x1640/0x163c (DSCP_TE block, no GO) are inert; also kept. */
	{
		static const u8 nkpol_d[] = {
			/* 0x10 */ 0x0F, 0x04, 0x0F, 0x09, 0x0F, 0x05,
			/* 0x16 */ 0x0F, 0x0A, 0x0F, 0x0B, 0x0F, 0x0C,
		};
		unsigned int k;

		for (k = 0; k < ARRAY_SIZE(nkpol_d); k++) {
			writel(0, ni_base(ni) + CA_NI_L2FE_DSCP_TE_DATA);
			writel(0, ni_base(ni) + CA_NI_L2FE_DSCP_TE_ACCESS);
			writel(nkpol_d[k], ni_base(ni) + CA_NI_L2FE_NKPOL_MAP_DATA);
			cortina_ni_rx_settle();
			cortina_ni_rx_ind_store(ni, CA_NI_L2FE_NKPOL_MAP_ACCESS,
						CA_NI_RX_CPU_LDPID + k);	/* 0x10 + k */
			cortina_ni_rx_settle();
		}
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_NKPOL_MAP_ACCESS, CA_NI_RX_L3LAN_LDPID);
		dev_info(ni->dev,
			 "tbl@0x1634 (ex-\"MC_FIB\", stock-match): [0x19]=0x%08x (want 0x0b); [0x10..0x1b] set\n",
			 readl(ni_base(ni) + CA_NI_L2FE_NKPOL_MAP_DATA));
	}

	/* readback via the INDIRECT read protocol (ACCESS=idx|GO, poll, then the entry
	 * latches into DATA) - want 0/0 = empty so the DFT_FWD 0x1832 frame falls
	 * through to raw ldpid 0x32. */
	cortina_ni_rx_ind_read(ni, CA_NI_NI_MCE_INDX_ACCESS, CA_NI_RX_DFT_FWD_MCGID);
	dev_info(ni->dev,
		 "mc-group[0x%x]: EMPTIED mc_vec hi(0xaaf8)=0x%08x lo(0xaafc)=0x%08x (want 0/0 -> DFT_FWD 0x1832 falls through to raw ldpid 0x%x)\n",
		 CA_NI_RX_DFT_FWD_MCGID,
		 readl(ni_base(ni) + CA_NI_NI_MCE_INDX_DATA1),
		 readl(ni_base(ni) + CA_NI_NI_MCE_INDX_DATA0),
		 CA_NI_RX_MC_CPU_LDPID);

	/* build33 rebuild-free fallback: if the empty-group fallthrough still lands on
	 * ldpid 0x19 on this silicon, redir_cpu_ldpid=0x32 forces REDIR_LDPID[0x19]->
	 * 0x32 (the mcgid 0x19 is intercepted BEFORE MC replication, per build14). */
	if (redir_cpu_ldpid) {
		cortina_ni_rx_redir_ldpid_set(ni, CA_NI_RX_DFT_FWD_MCGID,
					      redir_cpu_ldpid);
		dev_info(ni->dev,
			 "mc-group: REDIR_LDPID[0x%x]->0x%x forced (fallback)\n",
			 CA_NI_RX_DFT_FWD_MCGID, redir_cpu_ldpid);
	}
}

/*
 * ★ THE own-MAC CPU trap: install a static L2 FDB entry {our MAC -> L3_LAN}.
 * A frame to our own MAC is then a KNOWN unicast, forwarded to the L3_LAN
 * lport (0x19) -> PDPID 0x0d -> L3FE -> my-MAC comparator/CLS trap -> CPU -
 * exactly stock's my-MAC route (STOCK_cpurx_dynamic_golden.txt sec. 3), and
 * it avoids the DLF/UUC blackhole (AAL_LPORT_BLACKHOLE 0x1f).
 * ★ 2026-07-15: the action used to be DeepQ_0 (ldpid 0x00, from the era when
 * the my-MAC registers were unprogrammed) - but the deep-queue dest ports map
 * to EQ profile 13 = {EQ13,EQ14} which are DISABLED pools, so every unicast
 * to our MAC died in the L2TM with a bm hdr-drop (0x2148 climbing +1/frame)
 * while broadcast ARP (flood path) sailed through.  Now that the my-MAC
 * comparator + L3FE trap ARE programmed (MYMAC 1-4), send known unicast down
 * the same proven L3FE route.
 * The L2FE hashes {DA,fid} into a bucket on APPEND; we supply the full key+action.
 */

/* Stage one static FDB entry {mac, vid/scind/dot1p=0} -> {ldpid, valid,
 * static, DA/SA permit}, APPEND it, then READ-verify (status 0x5 = HIT). */
/*
 * Program a STATIC L2FE FDB entry {mac -> ldpid} and RETURN its hash-table entry
 * INDEX = CMD_RETURN.ext_status[16:4] (13-bit), or -1 on failure.
 *
 * ★ That index IS the L3FE forward action's mac_da_idx == the aal-77c "egr_lutidx"
 * (hw_dump/l2 lutidx): on a HW-offloaded routed frame the engine fetches the
 * next-hop DMAC from L2 FDB[idx] BY REFERENCE.  This is exactly how stock resolves
 * the egress DMAC - no raw MAC write to any L3FE table, so no repeat of the
 * aal-gen2 HS_LIGHT 0x3dc4 unmapped-register SError.
 *
 * `base` = the NI/NE register window (ni_base(ni) == cn_l3e->ne_base, the single
 * 0xf4300000 window).  Exported for the flow-offload next-hop path.
 */
/*
 * Pack a MAC into the FDB key words (aal __aal_mac_2_fdb_data; vid/scind/dot1p
 * = 0) - shared by the append and the lookup-only path below so both hash to
 * the same bucket.
 */
static void cortina_ni_l2fe_fdb_key(const u8 *mac, u32 *d3, u32 *d2, u32 *d1)
{
	*d3 = (mac[0] >> 5) & 0x7;
	*d2 = ((u32)(mac[0] & 0x1f) << 27) | ((u32)mac[1] << 19) |
	      ((u32)mac[2] << 11) | ((u32)mac[3] << 3) | ((mac[4] >> 5) & 0x7);
	*d1 = (u32)(((mac[4] & 0x1f) << 8) | mac[5]) << 19;
}

/*
 * Issue one OP_READ (look-up) for the packed key and return the 13-bit entry
 * index, or -1 when the key is not present / the engine times out.  DATA0 is
 * deliberately NOT written here: on a HIT the engine returns the matched
 * entry's ACTION word in it (that is how the append path can validate its own
 * write without re-supplying the action), so @act_out - when non-NULL - yields
 * the entry's stored forward-to LDPID + valid/static/permit bits.
 */
static int cortina_ni_l2fe_fdb_read_idx(void __iomem *base, u32 d3, u32 d2,
					u32 d1, u32 *act_out)
{
	u32 acc, cr;

	writel(0, base + CA_NI_L2FE_FDB_CMD_RETURN);
	writel(d3, base + CA_NI_L2FE_FDB_DATA3);
	writel(d2, base + CA_NI_L2FE_FDB_DATA2);
	writel(d1, base + CA_NI_L2FE_FDB_DATA1);
	writel(CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_READ,
	       base + CA_NI_L2FE_FDB_ACCESS);
	if (readl_poll_timeout(base + CA_NI_L2FE_FDB_ACCESS, acc,
			       !(acc & CA_NI_L2FE_FDB_GO), CA_NI_TX_POLL_US,
			       CA_NI_TX_POLL_TIMEOUT_US))
		return -1;
	cr = readl(base + CA_NI_L2FE_FDB_CMD_RETURN);
	if ((cr & 0xf) != CA_NI_L2FE_FDB_STATUS_HIT)
		return -1;			/* not present */
	if (act_out)
		*act_out = readl(base + CA_NI_L2FE_FDB_DATA0);
	return (int)((cr >> 4) & 0x1fff);	/* ext_status[16:4] = entry index */
}

int cortina_ni_l2fe_fdb_add_idx(void __iomem *base, const u8 *mac, u32 ldpid)
{
	u32 d0, d1, d2, d3, acc;

	cortina_ni_l2fe_fdb_key(mac, &d3, &d2, &d1);
	d0 = FIELD_PREP(CA_NI_L2FE_FDB_LPID, ldpid) |
	     CA_NI_L2FE_FDB_VALID | CA_NI_L2FE_FDB_STATIC |
	     CA_NI_L2FE_FDB_DA_PERMIT | CA_NI_L2FE_FDB_SA_PERMIT;

	writel(0, base + CA_NI_L2FE_FDB_CMD_RETURN);
	writel(d3, base + CA_NI_L2FE_FDB_DATA3);
	writel(d2, base + CA_NI_L2FE_FDB_DATA2);
	writel(d1, base + CA_NI_L2FE_FDB_DATA1);
	writel(d0, base + CA_NI_L2FE_FDB_DATA0);
	writel(CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_APPEND,
	       base + CA_NI_L2FE_FDB_ACCESS);
	if (readl_poll_timeout(base + CA_NI_L2FE_FDB_ACCESS, acc,
			       !(acc & CA_NI_L2FE_FDB_GO), CA_NI_TX_POLL_US,
			       CA_NI_TX_POLL_TIMEOUT_US))
		return -1;

	/* READ back the key: CMD_RETURN.status[3:0]=0x5 HIT, ext_status[16:4]=idx */
	return cortina_ni_l2fe_fdb_read_idx(base, d3, d2, d1, NULL);
}

/*
 * LOOK UP @mac in the L2FE FDB without touching the table, and report BOTH the
 * entry index (= the L3FE forward action's mac_da_idx / aal-77c egr_lutidx) and
 * the entry's stored forward-to LDPID.  Used by the DS (WAN->LAN) flow-offload
 * leg: the LAN client's MAC is already in the FDB (it was learned from the
 * client's own upstream traffic - the very traffic that created the conntrack),
 * so the DS next-hop DMAC *and* the LAN egress port both come from the one L2
 * entry the switch already resolved.  Deliberately lookup-ONLY: appending a
 * static entry for a dynamically-learned client MAC would pin it to a guessed
 * port and hijack normal bridging for that host.
 *
 * Returns the index, or -1 if the MAC is not present / the engine timed out.
 * @ldpid_out (optional) = the entry action's forward-to LDPID; for a LAN NI
 * port that equals the physical port number (cortina-ni-tx.c ARB identity map).
 */
int cortina_ni_l2fe_fdb_lookup_idx(void __iomem *base, const u8 *mac,
				   u32 *ldpid_out)
{
	u32 d1, d2, d3, act = 0;
	int idx;

	cortina_ni_l2fe_fdb_key(mac, &d3, &d2, &d1);
	idx = cortina_ni_l2fe_fdb_read_idx(base, d3, d2, d1, &act);
	if (idx < 0)
		return -1;
	if (!(act & CA_NI_L2FE_FDB_VALID) || !(act & CA_NI_L2FE_FDB_DA_PERMIT))
		return -1;		/* present but not forwardable as a DA */
	if (ldpid_out)
		*ldpid_out = FIELD_GET(CA_NI_L2FE_FDB_LPID, act);
	return idx;
}

static void cortina_ni_rx_fdb_append(struct cortina_ni *ni, const u8 *mac,
				     u32 ldpid)
{
	int idx = cortina_ni_l2fe_fdb_add_idx(ni_base(ni), mac, ldpid);

	dev_info(ni->dev, "fdb-add: %pM -> ldpid 0x%02x : entry_idx=%d %s\n",
		 mac, ldpid, idx, idx < 0 ? "(FAILED)" : "(HIT)");
}

static void cortina_ni_rx_fdb_add_cpu(struct cortina_ni *ni)
{
	static const u8 def_mac[ETH_ALEN] = { 0x02, 0x96, 0x07, 0xf0, 0x00, 0x01 };
	const u8 *mac = (ni->tx && ni->tx->netdev) ?
			ni->tx->netdev->dev_addr : def_mac;
	u32 acc;
	int ret;

	/* (0) ★ one-time FDB engine INIT (opcode 0) - the hash table must be built
	 * before the first APPEND, else APPEND silently no-ops.  No DATA; longer poll
	 * (it clears the whole table).  (APPEND itself has "nothing feedback" - the
	 * vendor never reads cmd_return for it - so the earlier cmd_return=0 was NOT
	 * the failure; the missing INIT was.) */
	writel(0, ni_base(ni) + CA_NI_L2FE_FDB_CMD_RETURN);
	writel(CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_INIT,
	       ni_base(ni) + CA_NI_L2FE_FDB_ACCESS);
	ret = readl_poll_timeout(ni_base(ni) + CA_NI_L2FE_FDB_ACCESS, acc,
				 !(acc & CA_NI_L2FE_FDB_GO), 10, 20000);
	if (ret)
		dev_warn(ni->dev, "fdb-add: engine INIT GO stuck\n");

	/* (1) router (LAN) MAC -> L3_LAN (0x19): stock's my-MAC route */
	cortina_ni_rx_fdb_append(ni, mac, CA_NI_RX_L3LAN_LDPID);

	/* (2) ★ WAN MAC (= base+1, the stock-confirmed per-board derivation) ->
	 * L3_WAN (0x18), gated on HW L3-forwarding.  THE terminating DS-WAN
	 * delivery: the Venus-family design keeps L2 MY-MAC detection OFF and
	 * "use[s] STATIC FDB to forward MyMAC packets to L3FE" (vendor
	 * special-packet layer), i.e. stock's FDB holds its WAN MAC -> L3_WAN.
	 * Without this entry a PON DS unicast to the WAN MAC (PDC ldpid stamp
	 * notwithstanding) is a DLF in the L2FE and gets FLOODED OUT instead of
	 * delivered (proven live 2026-07-19: 0/200 hades pings, l2fe_ni +200
	 * with bm_tx +200, l3fe_rx 0; with the entry: 200/200 + DHCP lease +
	 * WAN 0% loss).  Gate-off = byte-identical behavior (DS data then rides
	 * the PDC CPU_0+FE_BYPASS route and never consults the FDB).
	 * cortina_l3fe_hw_l3_forward_enable() installs the same entry at
	 * probe-time (this path runs before the engine arms; re-arms on
	 * link-up re-run it here with the gate true). */
	if (cortina_ni_hw_l3_fwd_active()) {
		u8 wan_mac[ETH_ALEN];
		u64 v = ((u64)mac[0] << 40) | ((u64)mac[1] << 32) |
			((u64)mac[2] << 24) | ((u64)mac[3] << 16) |
			((u64)mac[4] << 8) | mac[5];

		v++;
		wan_mac[0] = v >> 40;
		wan_mac[1] = v >> 32;
		wan_mac[2] = v >> 24;
		wan_mac[3] = v >> 16;
		wan_mac[4] = v >> 8;
		wan_mac[5] = v;
		cortina_ni_rx_fdb_append(ni, wan_mac, CA_NI_RX_L3WAN_LDPID);
	}
}

/*
 * ★ THE own-MAC CPU trap (architectural, via the L3FE).  Tier-1 stock: comparator
 * A (NI-global 0xa024/a028/a5c0) is the ACTIVE my-MAC; comparator B (0x3294/98) is
 * unused; chk_mymac_for_lan (0x3400 b21) = 0.  The rtl8277c STG0-SPB (0x3440) is
 * an UNMAPPED HOLE on Elnath and writing it async-SErrors (that was the panic) -
 * so we DROP it.  Gate-off (no HW L3-forwarding) programs comparator A +
 * my_mac_enable = today's own-MAC CPU trap; under hw_l3_fwd the comparator A
 * MAC is actively CLEARED (vendor Venus MYMAC=0 design - see the block comment
 * below) so a my-MAC frame rides the static FDB into the L3FE instead, while
 * my_mac_enable (0x3218 bit2) stays SET like stock - clearing it broke
 * GPON/OMCI (OLT Deactivate churn).  The my-MAC-hit
 * route lives in the STG0 LPB profile vector, dumped here to decode/match stock.
 * dev_emerg bisect markers bracket each write group so any residual fault pins
 * the exact register.
 */
static void cortina_ni_rx_mymac_trap(struct cortina_ni *ni)
{
	static const u8 def_mac[ETH_ALEN] = { 0x02, 0x96, 0x07, 0xf0, 0x00, 0x01 };
	const u8 *mac = (ni->tx && ni->tx->netdev) ?
			ni->tx->netdev->dev_addr : def_mac;
	bool l2_trap = !cortina_ni_hw_l3_fwd_active();
	u32 det, ctrl, hi_p0;

	/* (A) NI-global my-MAC: CFG0=bytes0-3, CFG1[7:0]=byte4, PT[31:24]=byte5.
	 *
	 * ★ Under HW L3-forwarding (hw_l3_fwd) this own-MAC comparator must stay
	 * UNARMED: a LAN->WAN transit frame's DST-MAC IS the gateway MAC, and the
	 * armed comparator resolved it to the CPU BEFORE the DA-FDB, so it never
	 * rode the static FDB {gwMAC -> L3_LAN 0x19} route into the L3FE and the
	 * T2 main-hash never executed - no installed flow could ever HIT
	 * (board-proven 2026-07-23: 47kpps matching transit flow, L3FE DBG
	 * pkt-count flat at idle rate, 5-tuple absent from every L3FE stage
	 * monitor, installed entry age never re-armed).  This is the Venus-family
	 * design (vendor cortina-api special_packet.c: "we do NOT use L2 MY-MAC
	 * but use STATIC FDB to forward MyMAC packets to L3FE" - MYMAC is set to
	 * 00:00:00:00:00:00 to disable L2 my-MAC detection).  Comparator A = 0
	 * IS that MYMAC=0 disable; terminating/mgmt frames then reach the CPU
	 * via the L3FE T2-miss -> HS_DEF entry-0 CPU_0 punt.
	 * ★ ONLY the comparator MAC is cleared - SPCL_PKT_DET.my_mac_enable
	 * (0x3218 bit2) MUST STAY SET like stock (0x0739DC24): a first fix also
	 * cleared bit2 and BROKE GPON (board 2026-07-23: O5 reached, then OLT
	 * Deactivate_ONU-ID ~27s later, endless re-range churn, no WAN - bit2
	 * gates the special-packet delivery of the GPON/OMCI control frames to
	 * the CPU, exactly the vendor detect pipeline stock leaves enabled while
	 * zeroing only the MYMAC compare value).
	 * The clear is ACTIVE (not skipped) so a link-up/MAC re-arm undoes the
	 * probe-time armed state (the first RX-init run precedes the l3fe engine
	 * arm, so its gate reads inactive and still arms the comparator).
	 * Gate-off keeps today's trap writes byte-identical. */
	dev_emerg(ni->dev, "MYMAC 1: comparator A (0xa024/a028/a5c0)\n");
	if (l2_trap) {
		writel(((u32)mac[0] << 24) | ((u32)mac[1] << 16) |
		       ((u32)mac[2] << 8) | mac[3],
		       ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG0);
		ni_rmw(ni, CA_NI_L3FE_NI_MAC_CFG1, CA_NI_L3FE_NI_MAC_BYTE4,
		       mac[4]);
		ni_rmw(ni, CA_NI_L3FE_PT_PORT_STATIC_CFG,
		       CA_NI_L3FE_PT_MAC_BYTE5,
		       FIELD_PREP(CA_NI_L3FE_PT_MAC_BYTE5, mac[5]));
	} else {
		writel(0, ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG0);
		ni_rmw(ni, CA_NI_L3FE_NI_MAC_CFG1, CA_NI_L3FE_NI_MAC_BYTE4, 0);
		ni_rmw(ni, CA_NI_L3FE_PT_PORT_STATIC_CFG,
		       CA_NI_L3FE_PT_MAC_BYTE5, 0);
	}

	/* enable my-MAC detection (0x3400 b21 chk_mymac_for_lan left 0 = stock).
	 * Unconditional - kept SET under hw_l3_fwd too (stock keeps it set;
	 * clearing it broke GPON/OMCI - see the block comment above). */
	dev_emerg(ni->dev, "MYMAC 2: my_mac_enable (0x3218 bit2)\n");
	ni_rmw(ni, CA_NI_L3FE_SPCL_PKT_DET_CFG, 0, CA_NI_L3FE_MY_MAC_EN);

	if (!l2_trap)
		dev_info(ni->dev,
			 "mymac-trap: comparator A cleared under hw_l3_fwd (0x3218 my_mac_enable kept stock/set) -> transit rides FDB into L3FE\n");

	/* ★ match stock STG0 LPB profiles + ldpid_map (tier-1; ours had spcl_pkt_en=0,
	 * MID=0, prof2/3 empty, wrong ldpid_map).  bisect-from-working: stock with
	 * these EXACT values routes own-MAC -> CPU, and the rtl8277c SPB (0x3440) is
	 * unmapped on Elnath so stock's SPB writes drop -> the route lives here in the
	 * LPB vector.  Direct registers (0x3408-3434). */
	dev_emerg(ni->dev, "MYMAC 3: STG0 LPB match (0x3404-3434)\n");
	/* ★ WAN LPB profiles (prof0 @HIGH0, prof2 @HIGH2) = stock 0x18100190
	 * verbatim, spcl_pkt_en (bit20) INCLUDED.  An earlier build cleared
	 * bit20 under hw_l3_fwd on the theory the L3FE special-packet handler
	 * diverted terminating DS-WAN frames — REFUTED 2026-07-19: the L3
	 * special-packet behavior table behind that bit does not exist on this
	 * die (stock ca-ne.ko stubs aal_l3_specpkt_ctrl_set/get to `mov w0,#0;
	 * ret`, matching the 0x3440 SError), the bit is inert, and a live A/B
	 * (bit20 0 vs 1 under hw_l3_fwd) showed identical 0%-loss DS delivery.
	 * The real DS-WAN delivery is the static FDB WAN-MAC -> L3_WAN entry
	 * (cortina_ni_rx_fdb_add_cpu).  Byte-match stock. */
	hi_p0 = CA_NI_L3FE_LPB_HIGH_P0;
	writel(CA_NI_L3FE_STG0_LDPID_MAP_VAL,
	       ni_base(ni) + CA_NI_L3FE_STG0_LDPID_MAP);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW0);
	writel(CA_NI_L3FE_LPB_MID_SEL0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID0);
	writel(hi_p0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH0);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW1);
	writel(CA_NI_L3FE_LPB_MID_SEL1, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID1);
	writel(CA_NI_L3FE_LPB_HIGH_P1, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH1);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW2);
	writel(CA_NI_L3FE_LPB_MID_SEL0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID2);
	writel(hi_p0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH2);
	writel(0, ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW3);
	writel(CA_NI_L3FE_LPB_MID_SEL1, ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID3);
	writel(CA_NI_L3FE_LPB_HIGH_P3, ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH3);

	/* ★ Program the L3FE STG0 my-MAC (0x3210/0x3214) from our per-board MAC + the
	 * valid bit (0x3210 bit16).  Tier-1 stock diff (2026-07-12): stock sets these to
	 * its board MAC WITH the valid bit; ours were 0 (no MAC, valid clear).  Encoding:
	 * LO = valid | mac[0]<<8 | mac[1];  HI = mac[2..5]. */
	writel(CA_NI_L3FE_MY_MAC_VALID | ((u32)mac[0] << 8) | mac[1],
	       ni_base(ni) + CA_NI_L3FE_MY_MAC_LO);
	writel(((u32)mac[2] << 24) | ((u32)mac[3] << 16) | ((u32)mac[4] << 8) | mac[5],
	       ni_base(ni) + CA_NI_L3FE_MY_MAC_HI);

	/* ★ ILPB_LDPID (0x30d8) write DROPPED: tier-1 live shows stock 0x30d8=0 and stock's
	 * L3FE works (l3fe_rx>0) WITH it 0 - so it is NOT the enter-L3 gate (the vendor
	 * aal_l3fe_l2lookup_init value does not apply to Elnath).  Leave 0x30d8=0 to match
	 * stock.  The real enter-L3 gate is elsewhere in the L3FE_GLB ELPB/DEEPQ_VLD block
	 * (0x30e0/0x30e4/0x30e8 stock-nonzero, ours=0) - pinned by the live diff. */
	dev_info(ni->dev,
		 "l3fe-loopback: ilpb(0x30d8)=0x%08x(stock 0) elpb0(0x30e0)=0x%08x dqvld1(0x30e4)=0x%08x dqvld0(0x30e8)=0x%08x my_mac lo(0x3210)=0x%08x hi=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ILPB_LDPID),
		 readl(ni_base(ni) + 0x30e0), readl(ni_base(ni) + 0x30e4),
		 readl(ni_base(ni) + 0x30e8),
		 readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_LO),
		 readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_HI));

	dev_emerg(ni->dev, "MYMAC 4 (survived - no SError)\n");
	det = readl(ni_base(ni) + CA_NI_L3FE_SPCL_PKT_DET_CFG);
	ctrl = readl(ni_base(ni) + CA_NI_L3FE_STG0_CTRL);
	dev_info(ni->dev,
		 "mymac-trap: %pM niA cfg0=0x%08x cfg1=0x%08x pt(a5c0)=0x%08x | det=0x%08x(myen=%u) stg0=0x%08x(chklan=%u)\n",
		 mac, readl(ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG0),
		 readl(ni_base(ni) + CA_NI_L3FE_NI_MAC_CFG1),
		 readl(ni_base(ni) + CA_NI_L3FE_PT_PORT_STATIC_CFG),
		 det, !!(det & CA_NI_L3FE_MY_MAC_EN),
		 ctrl, !!(ctrl & CA_NI_L3FE_CHK_MYMAC_LAN));
	dev_info(ni->dev,
		 "mymac-trap: STG0 hi[0-3]=0x%08x 0x%08x 0x%08x 0x%08x lo0=0x%08x mid0=0x%08x ldpid_map=0x%08x (stock hi0=0x18100190, spcl_pkt_en=bit20)\n",
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH0),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH1),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH2),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_HIGH3),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_LOW0),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LPB_MID0),
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_LDPID_MAP));
}

/*
 * ★ Re-key every MAC-keyed admission/offload table from the CURRENT netdev
 * MAC (called from .ndo_set_mac_address after eth_mac_addr commits dev_addr).
 * The boot RX init and the link-up re-arms latch dev_addr into the tables,
 * but netifd applies the per-board factory MAC (05_factory_mac) AFTER the
 * LAST re-arm - on this rig the tracked port-0 PHY is uncabled, so once
 * intf_done no further link-up fires - leaving everything keyed on the probe
 * fallback 02:96:07:f0:00:01 (BOARD-MEASURED 2026-07-23: fdb-add/mymac-trap
 * logged the fallback at t=4.9s AND t=17.5s while br-lan answered LAN ARP
 * with the factory MAC).  A LAN transit frame to the factory gateway MAC then
 * misses the FDB -> DLF flood -> CPU software forward, and never resolves to
 * L3_LAN, so it cannot enter the L3FE flow engine; an offloaded US flow would
 * also egress the stale fallback+1 SMAC (PP FIELD-CAM idx 1 via
 * L3-IF[2].mac_sa_an_sel).  Four latches, re-armed in order:
 *   1. offload backend router-MAC shadow (cortina-ni-flowoffload.c);
 *   2. L2FE static FDB - fdb_add_cpu's OP_INIT wipes the stale fallback
 *      entries, then re-appends LAN -> L3_LAN + (gated) WAN -> L3_WAN;
 *   3. my-MAC comparator A + L3FE STG0 my-MAC (mymac_trap, idempotent);
 *   4. PP FIELD-CAM router-MAC entries (intf_add: LAN idx 0, WAN idx 1) -
 *      the ingress mac_da_an_sel stamp AND the IPoE egress SMAC source.
 * hw_l3_fwd-gated: gate-off is byte-identical to today (own-MAC frames are
 * then delivered by the DLF flood-to-CPU path regardless of the FDB key).
 */
void cortina_ni_rx_mac_rearm(struct cortina_ni *ni)
{
	if (!ni->rx || !ni->tx || !ni->tx->netdev ||
	    !cortina_ni_hw_l3_fwd_active())
		return;

	cortina_ni_flowoffload_router_mac_set(ni->tx->netdev->dev_addr);
	cortina_ni_rx_fdb_add_cpu(ni);
	cortina_ni_rx_mymac_trap(ni);
	cortina_l3fe_intf_add(ni_base(ni), ni->tx->netdev->dev_addr);
	dev_info(ni->dev,
		 "MAC-keyed admission re-armed for %pM (FDB + my-MAC + router-CAM)\n",
		 ni->tx->netdev->dev_addr);
}

/* Program one REDIR_LDPID_CONFIG entry: idx (a redir-LDPID) -> real dest LDPID.
 * Indirect table: write DATA, then ACCESS = GO|WR|idx, poll GO clear (stock aal
 * FIND_INDIRCT_ADDRESS / CHECK_INDIRCT_OPERATE_STATE).  Bounded + non-fatal. */
static void __maybe_unused
cortina_ni_rx_redir_ldpid_set(struct cortina_ni *ni, u8 idx,
			      u8 dest_ldpid)
{
	void __iomem *acc = ni_base(ni) + CA_NI_L2FE_REDIR_LDPID_ACCESS;
	u32 val;
	int ret;

	writel(FIELD_PREP(CA_NI_L2FE_REDIR_RDIR_LDPID, dest_ldpid) |
	       CA_NI_L2FE_REDIR_RDIR_EN,
	       ni_base(ni) + CA_NI_L2FE_REDIR_LDPID_DATA);
	writel(CA_NI_L2FE_REDIR_ACCESS_GO | CA_NI_L2FE_REDIR_ACCESS_WR | idx, acc);
	ret = readl_poll_timeout(acc, val, !(val & CA_NI_L2FE_REDIR_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(ni->dev,
			 "redir_ldpid[0x%x]->0x%x: ACCESS GO stuck (0x%08x)\n",
			 idx, dest_ldpid, val);
}

/*
 * L2FE forwarding-control regs = stock's live values byte-for-byte (tier-1
 * devmem: 0x1404=0x3, 0x1408=0x22000290, 0x140c=0x0c100c10, 0x1504=0x001b0000,
 * 0x1600=0x89c71c82, 0x160c=0x1).  The port-0 DLF->CPU decision is the DFT_FWD
 * redir (0x1832) in cortina_ni_rx_ple_dft_fwd; the REDIR_LDPID *table* is NOT
 * used (stock leaves 0x18/0x19 = 0 - mc_group_id 0x19 with redir_en=1 is a
 * DIRECT CPU dest LDPID the QM resolves to CPU port 8).  Idempotent.
 */
static void cortina_ni_rx_l2fe_forwarding(struct cortina_ni *ni)
{
	unsigned int i;

	writel(CA_NI_L2FE_DPID_FWD_CTRL_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLC_DPID_FWD_CTRL);
	writel(CA_NI_L2FE_LRN_FWD_CTRL_0_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLC_LRN_FWD_CTRL_0);
	writel(CA_NI_L2FE_LRN_FWD_CTRL_1_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLC_LRN_FWD_CTRL_1);
	writel(CA_NI_L2FE_PLE_DEFAULT_VAL,
	       ni_base(ni) + CA_NI_L2FE_PLE_DEFAULT_REG);
	/* arb_ctrl: dbuf_dpid[7:4]=8 (deep_q comparator) AND cpu_dpid[27:24]=9 (the CPU
	 * comparator - a resolved PDPID==9 is what should set the header cpu_flg).  Ours
	 * matches stock 0x89c71c82, so the comparators are correct. */
	writel(CA_NI_L2FE_ARB_CTRL_VAL, ni_base(ni) + CA_NI_L2FE_ARB_CTRL);
	/* ★★ build41: override the deep_q trigger dbuf_dpid[7:4] (default 0xf = disable, so
	 * the pdpid-0x08 CPU frame is deep_q=0 and takes the normal BM path to L3QM, bypassing
	 * our DQSCH).  arb_dbuf_dpid=8 restores stock deep_q=1. */
	ni_rmw(ni, CA_NI_L2FE_ARB_CTRL, CA_NI_L2FE_ARB_DBUF_DPID,
	       FIELD_PREP(CA_NI_L2FE_ARB_DBUF_DPID, arb_dbuf_dpid));
	dev_info(ni->dev, "arb: dbuf_dpid(deep_q trigger)=0x%x -> arb_ctrl=0x%08x (0xf=deep_q OFF/normal path)\n",
		 arb_dbuf_dpid, readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL));
	writel(CA_NI_L2FE_ARB_REG_1608_VAL,
	       ni_base(ni) + CA_NI_L2FE_ARB_REG_1608);
	writel(CA_NI_L2FE_ARB_CTRL_EXT_VAL,
	       ni_base(ni) + CA_NI_L2FE_ARB_CTRL_EXT);
	/* ★★ ARB per-port allow/classify masks (0x1614-0x1620) = stock 0xFFFFFFFF each.
	 * We never wrote them (reset 0 = deny), the top suspect for cpu_flg staying 0
	 * despite a resolved PDPID==cpu_dpid(9): a 0 mask masks CPU classification off. */
	for (i = 0; i < CA_NI_L2FE_ARB_ALLOW_MASK_COUNT; i++)
		writel(CA_NI_L2FE_ARB_ALLOW_MASK_VAL,
		       ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(i));

	dev_info(ni->dev,
		 "l2fe-fwd: arb_ctrl=0x%08x reg1608=0x%08x allow[0..3]=0x%08x/0x%08x/0x%08x/0x%08x ple_dflt=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_REG_1608),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(0)),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(1)),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(2)),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_ALLOW_MASK(3)),
		 readl(ni_base(ni) + CA_NI_L2FE_PLE_DEFAULT_REG));
}

/*
 * RX steer = the FULL FORWARDING-ENGINE path, matching stock's golden
 * working-RX config (devmem-verified on a stock boot).  Stock does NOT use
 * the FE-bypass shortcut we relied on (which left hw wptr stuck at 0 ~half
 * the boots): instead an ingress port-0 frame is looked up by the L2FE, and
 * because we keep no FDB every frame is a lookup miss (DLF) that the PLE
 * default-forwards to CPU port 0, while the L3FE demux map routes that FE
 * output into our EQ13 CPU-EPP ring.  This is deterministic across boots.
 *
 * Writes (all NI window):
 *  - rx_cntrl 0xa5f8 = 0x08000600 (reset 0x08000400 + OAM pass, NO byp_dpid)
 *  - L3FE demux 0xa190..0xa1a4 = the golden routing map
 *  - L3TM ni-port-ena 0x610c = 0xffffffff (second egress-enable layer)
 *  - PLE default-forward: port-0 BC/UUC/UL2MC/UL3MC -> CPU0
 * Idempotent - safe to re-apply from the link-up re-arm.
 */
/*
 * Replicate stock __ni_flow_ctrl_init (ca-ne.ko @0xc160): the L2TM BM
 * dequeue->TM-port map + per-port flow-control thresholds.  We leave L2FE/L2TM
 * running at U-Boot init, but U-Boot does NOT run this flow-ctrl init, so the BM
 * dequeue->TM-port map (0x2124) is unset and a CPU-dest frame is dequeued
 * (BM_TX_PCNT 0x2140 ++) yet routed to the wrong TM-port instead of the QM -
 * exactly the observed qm_rx_cntr(0x690c)=0 while tm tx=31, no drop counted.
 * Idempotent: if U-Boot happened to set these, we re-write the same values.
 */
/* ★ build39: the PHYSICAL DQ->TM-port map (0x212c).  Our 0x2124 uplink-flag override
 * is confirmed set (0x88888888) yet the deep-queue drains to ES7(physical) not ES8/L3QM
 * -> the override is not taking effect on ours.  Default 0x88888888 = force EVERY DQ to
 * TM-port 8 = ES8/L3QM (the direct steer-to-L3QM test); set =0x76543210 to restore the
 * stock/identity map for A/B. */
static unsigned int dq_tmport_map = 0x76543210u;	/* build68 STOCK identity: DQ N -> TM port N.  0x88888888 (build39) forced ALL DQs to ES port 8 = L3LAN dead-end -> frame never reached ES7/L3QM -> rmu_rx=0.  THE routing bug. */
module_param(dq_tmport_map, uint, 0644);
MODULE_PARM_DESC(dq_tmport_map, "physical DQ->TM-port map @0x212c (0x88888888=all->ES8/L3QM, 0x76543210=stock identity)");

static void cortina_ni_rx_flow_ctrl_init(struct cortina_ni *ni)
{
	int i;

	/* flow-control enable (RMW OR bit19) */
	ni_rmw(ni, CA_NI_NI_FLOWCTRL_EN, 0, CA_NI_NI_FLOWCTRL_EN_BIT);

	/* BM dequeue -> TM-port uplink-flag map: every TM-port -> L3QM/ES8 (= QM/CPU) */
	writel(CA_NI_L2TM_BM_DQ_PORT_MAP_VAL,
	       ni_base(ni) + CA_NI_L2TM_BM_DQ_PORT_MAP);

	/* ★★ build39: the PHYSICAL DQ->TM-port NUMBER map (0x212c) - force every DQ to
	 * TM-port 8 = ES8/L3QM (dq_tmport_map, default 0x88888888).  Then read back the
	 * REAL 0x2124 + 0x212c so dmesg proves what the HW actually holds (settles the
	 * "is 0x2124 a /proc shadow?" question). */
	writel(dq_tmport_map, ni_base(ni) + CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP);
	dev_info(ni->dev,
		 "bm-dq-map: REAL 0x2124=0x%08x (uplink-flag, want 0x88888888) 0x212c=0x%08x (phys DQ->TMport, set 0x%08x)\n",
		 readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_PORT_MAP),
		 readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP),
		 dq_tmport_map);

	/* per-port flow-control thresholds 0x9798..0x97b0 (stock bfxil+bfi) */
	for (i = 0; i < CA_NI_NI_FLOWCTRL_THRESH_CNT; i++)
		ni_rmw(ni, CA_NI_NI_FLOWCTRL_THRESH + i * 4,
		       CA_NI_NI_FLOWCTRL_THRESH_CLR, CA_NI_NI_FLOWCTRL_THRESH_VAL);

	/* ★★ L2TM TM-egress -> CPU-queue map block (stock values; U-Boot leaves these,
	 * esp. 0x2100, unset).  Without them a non-deep CPU frame is BM-dequeued (tm tx++)
	 * but routed to the wrong TM output and never reaches RMU0's CPU queue (0x6900=0,
	 * no drop) - the current wall.  Match stock byte-for-byte. */
	writel(CA_NI_L2TM_TM_CFG_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_CFG);
	writel(CA_NI_L2TM_TM_MAP_A_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_A);
	writel(CA_NI_L2TM_TM_MAP_B_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_B);
	writel(CA_NI_L2TM_TM_TO_CPUQ_VAL, ni_base(ni) + CA_NI_L2TM_TM_TO_CPUQ_MAP);
	writel(CA_NI_L2TM_TM_MAP_C_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_C);
	writel(CA_NI_L2TM_TM_MAP_D_VAL,	  ni_base(ni) + CA_NI_L2TM_TM_MAP_D);
	dev_info(ni->dev,
		 "l2tm tm-map: 0x2100=0x%08x 0x2114=0x%08x 0x2118=0x%08x 0x2120=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L2TM_TM_CFG),
		 readl(ni_base(ni) + CA_NI_L2TM_TM_MAP_B),
		 readl(ni_base(ni) + CA_NI_L2TM_TM_TO_CPUQ_MAP),
		 readl(ni_base(ni) + CA_NI_L2TM_TM_MAP_D));

	/* ★★ NI->L3QM ingress-handoff flow-control the frame needs to PRESENT to the QM
	 * (stock __ni_flow_ctrl_init + aal_ni_rxmux_fc_thrshld_set + the NI-HV RXFIFO
	 * threshold).  Missing, the NIRX->L3QM FIFO back-pressures and the QM ingress stays
	 * 0 (qm_rx=0) even though the L2TM egresses (tm tx++).  Tier-1 stock live values. */
	ni_rmw(ni, CA_NI_NI_FC_2914, 0, CA_NI_NI_FC_2914_EN);	/* 0x2914 |= bit31 */
	for (i = 0; i < CA_NI_NI_RXMUX_FC_THR_CNT; i++)
		writel(i < CA_NI_NI_RXMUX_FC_THR_LO_CNT ?
			       CA_NI_NI_RXMUX_FC_THR_LO_VAL :
			       CA_NI_NI_RXMUX_FC_THR_HI_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_NI_RXMUX_FC_THR, i,
						CA_NI_NI_RXMUX_FC_THR_COUNT));
	writel(CA_NI_NI_RXFIFO_THR_B0_VAL, ni_base(ni) + CA_NI_NI_RXFIFO_THR_B0);
	writel(CA_NI_NI_RXFIFO_THR_B4_VAL, ni_base(ni) + CA_NI_NI_RXFIFO_THR_B4);
	writel(CA_NI_NI_RXFIFO_THR_B8_VAL, ni_base(ni) + CA_NI_NI_RXFIFO_THR_B8);
	dev_info(ni->dev,
		 "ni->qm handoff: 0x2914=0x%08x rxmux_fc[0]=0x%08x rxfifo_thr(0xa1b8)=0x%08x (l3qm_rxfifo_hi[6:0]=0x%02x)\n",
		 readl(ni_base(ni) + CA_NI_NI_FC_2914),
		 readl(ni_base(ni) + CA_NI_NI_RXMUX_FC_THR(0)),
		 readl(ni_base(ni) + CA_NI_NI_RXFIFO_THR_B8),
		 readl(ni_base(ni) + CA_NI_NI_RXFIFO_THR_B8) & 0x7f);
}

/*
 * ★★ Enable the L2TM egress scheduler (stock aal_l2_tm_init).  At reset
 * ES_CTRL (0x2300) = 0x24000000 = tx_en(bit31)=0 and NO per-port enable bits, so
 * the L2TM never DRAINS a buffered frame out to any egress port.  A LAN-ingress
 * frame that resolves to the deep queue is enqueued into the L2TM (tm rx climbs)
 * but, with ES port 8 (= L3QM, the deep-queue -> QM handoff) disabled, is never
 * scheduled out to the QM, so RMU0 never admits it (RMU0_RX_PKT_CNTR 0x6900=0)
 * and the CPU-EPP ring never advances.  U-Boot's minimal bring-up uses the
 * FE-bypass TX path and never runs this init, so ES_CTRL stays at reset.
 *
 * Stock enables tx_en + every real ES port (0-13 and 15; bit14 is reserved =
 * 0xbfff) and all 8 VOQs per scheduler.  ES port 8 = CA_AAL_ES_PORT_L3QM is the
 * one that drains the deep queue to the QM -> RMU -> CPU.  Idempotent (RMW-OR),
 * so safe from the link-up re-arm.
 */
static void cortina_ni_rx_l2tm_es_init(struct cortina_ni *ni)
{
	int i;

	/* tx_en + per-port enable (ports 0-13,15) - preserve the reset deglitch
	 * bits (26,29) via RMW-OR */
	ni_rmw(ni, CA_NI_L2TM_ES_CTRL, 0,
	       CA_NI_L2TM_ES_TX_EN | CA_NI_L2TM_ES_PORT_EN_ALL);

	/* per-scheduler VOQ enable (bits[7:0]); reset default is already 0xff but
	 * re-assert so instance 8 (L3QM) definitely drains all VOQs */
	for (i = 0; i < CA_NI_L2TM_ES_SCH_INSTANCES; i++)
		ni_rmw(ni, CA_NI_L2TM_ES_SCH_CFG(i), 0, CA_NI_L2TM_ES_VOQ_EN_ALL);

	dev_info(ni->dev,
		 "l2tm-es: es_ctrl=0x%08x sch0=0x%08x sch8(L3QM)=0x%08x (want es_ctrl bit31+bit8 set, sch voq_en=0xff)\n",
		 readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL),
		 readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(0)),
		 readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(CA_NI_L2TM_ES_PORT_L3QM)));
}

/*
 * ★★ Deep-queue / central-buffer SCHEDULER init (L2TE_CB + DQSCH).  Tier-1 stock
 * (stock_l2tm_deepq_2000_2fff.txt) populates this whole block; our driver never
 * touched it, so a deep_q=1 frame is enqueued into the central buffer but the
 * deep-queue scheduler (DQSCH) never drains it to RMU0 (0x6900 stuck 0).  We
 * replicate stock's DIRECT threshold regs verbatim + the per-VOQ threshold
 * indirect tables.  (Counters/status and the policer block are NOT touched.)
 */
static const struct { u16 off; u32 val; } cortina_ni_deepq_cb_cfg[] = {
	{ 0x2364, 0x000600ffu },
	{ 0x237c, 0x000600ffu },
	{ 0x23a0, 0x000600ffu },
	{ 0x23b8, 0x01010101u },
	{ 0x23bc, 0x01010101u },
	{ 0x23c0, 0x01010101u },
	{ 0x23c4, 0x01000101u },
	{ 0x23c8, 0x00000014u },
	{ 0x23cc, 0x00000740u },
	{ 0x23d0, 0x00003fffu },
	{ 0x23e4, 0xffffffffu },
	{ 0x23e8, 0xffffffffu },
	{ 0x23ec, 0xffffffffu },
	{ 0x23f0, 0xffffffffu },
	{ 0x23f4, 0xffffffffu },
	{ 0x2404, 0x0700000fu },
	{ 0x2410, 0x8700f000u },
	{ 0x2414, 0x000007ffu },
	{ 0x241c, 0x20000f00u },
	{ 0x2500, 0x00000500u },
	{ 0x2504, 0x00000502u },
	{ 0x2508, 0x00000502u },
	{ 0x250c, 0x00000502u },
	{ 0x2510, 0x00000502u },
	{ 0x2514, 0x00000502u },
	{ 0x2518, 0x00000502u },
	{ 0x251c, 0x00000502u },
	{ 0x2520, 0x00000502u },
	{ 0x2524, 0x00000502u },
	{ 0x2528, 0x00000502u },
	{ 0x252c, 0x00000502u },
	{ 0x2530, 0x00000502u },
	{ 0x2534, 0x00000502u },
	{ 0x2538, 0x00000502u },
	{ 0x253c, 0x00000502u },
	{ 0x2540, 0x4000000au },
	{ 0x2544, 0x00000064u },
	{ 0x2548, 0x00017c01u },
	{ 0x254c, 0x900005f0u },
	{ 0x2550, 0x00000502u },
	{ 0x2554, 0x00000502u },
	{ 0x2558, 0x00000502u },
	{ 0x255c, 0x00000502u },
	{ 0x2560, 0x00000502u },
	{ 0x2564, 0x00000502u },
	{ 0x2568, 0x00000502u },
	{ 0x256c, 0x00000502u },
	{ 0x2570, 0x00000502u },
	{ 0x2574, 0x00000502u },
	{ 0x2578, 0x00000502u },
	{ 0x257c, 0x00000502u },
	{ 0x2580, 0x00000502u },
	{ 0x2584, 0x00000502u },
	{ 0x2588, 0x00000502u },
	{ 0x258c, 0x00000502u },
	{ 0x2590, 0x00000502u },
	{ 0x2594, 0x00000502u },
	{ 0x2598, 0x00000502u },
	{ 0x259c, 0x40000006u },
	{ 0x25a0, 0x00000040u },
	{ 0x25a4, 0x7ffff9ffu },
	{ 0x25a8, 0xfdffffe7u },
	{ 0x25ac, 0x00000502u },
	{ 0x25b0, 0x00000502u },
	{ 0x25b4, 0x00000502u },
	{ 0x25b8, 0x00000502u },
	{ 0x25bc, 0x00000502u },
	{ 0x25c0, 0x00000502u },
	{ 0x25c4, 0x00000502u },
	{ 0x25c8, 0x00000502u },
	{ 0x25cc, 0x00000502u },
	{ 0x25d0, 0x00000502u },
	{ 0x25d4, 0x00000502u },
	{ 0x25f4, 0x2ff3e723u },
	{ 0x25f8, 0x000001f3u },
	{ 0x25fc, 0x001fffffu },
	{ 0x2600, 0x001fffffu },
	{ 0x2604, 0x001fffffu },
	{ 0x2608, 0x001fffffu },
	{ 0x260c, 0xdc0087c0u },
	{ 0x2700, 0x14141414u },
	{ 0x2714, 0x0000007cu },
	{ 0x2718, 0x40000006u },
	{ 0x271c, 0x3ffffe01u },
	{ 0x2720, 0x01ffffe7u },
	{ 0x2748, 0x001fffffu },
	{ 0x274c, 0x2ff3e723u },
	{ 0x2750, 0x000001f3u },
	{ 0x2d7c, 0x02000010u },
	{ 0x2d80, 0x7fff7fffu },
	{ 0x2d84, 0x03000010u },
	{ 0x2d88, 0x7fff7fffu },
	{ 0x2d8c, 0x7fff3fffu },
	{ 0x2d90, 0x7fff7fffu },
	{ 0x2d94, 0x7fff3fffu },
	{ 0x2d98, 0x7fff7fffu },
	{ 0x2db0, 0x00000002u },
	{ 0x2db4, 0x4000000fu },
	{ 0x2db8, 0x8e308000u },
	{ 0x2dcc, 0x0e200020u },
	{ 0x2dd0, 0x00200010u },
	{ 0x2dd4, 0x00200010u },
	{ 0x2dd8, 0x00200010u },
	{ 0x2ddc, 0x00200010u },
	{ 0x2de0, 0x00200010u },
	{ 0x2de4, 0x00200010u },
	{ 0x2de8, 0x00200010u },
	{ 0x2dec, 0x00200010u },
	{ 0x2df4, 0x0c000100u },
	{ 0x2df8, 0x0c000100u },
	{ 0x2dfc, 0x0c000100u },
	{ 0x2e00, 0x0c000100u },
	{ 0x2e08, 0x02800110u },
	{ 0x2e0c, 0x01800110u },
	{ 0x2e10, 0x01800110u },
	{ 0x2e14, 0x01800110u },
	{ 0x2e18, 0x01800110u },
	{ 0x2e1c, 0x01800110u },
	{ 0x2e20, 0x01800110u },
	{ 0x2e24, 0x01800110u },
	{ 0x2e28, 0x02800110u },
	{ 0x2e2c, 0x01800060u },
	{ 0x2e30, 0x01800060u },
	{ 0x2e34, 0x01800060u },
	{ 0x2e38, 0x01800060u },
	{ 0x2e3c, 0x01800060u },
	{ 0x2e40, 0x01800060u },
	{ 0x2e44, 0x01800060u },
	{ 0x2e58, 0x00007fffu },
	{ 0x2e5c, 0x00200020u },
	{ 0x2e60, 0x05200020u },
	{ 0x2e64, 0x3fff3fffu },
	{ 0x2e68, 0x3fff3fffu },
	{ 0x2ec8, 0x7fff7fffu },
	{ 0x2ecc, 0x7fff7fffu },
	{ 0x2ed0, 0x7fff7fffu },
	{ 0x2ed4, 0x7fff7fffu },
	{ 0x2ed8, 0x7fff7fffu },
	{ 0x2edc, 0x7fff7fffu },
	{ 0x2ee0, 0x7fff7fffu },
	{ 0x2ee4, 0x7fff7fffu },
	{ 0x2ee8, 0x7fff7fffu },
};

/*
 * ★★ UPSTREAM QUEUE DEPTH - the L2TM deep-queue per-VoQ admission thresholds.
 *
 * There are TWO different things at 0x2d.. / 0x2e.. that both hold 0x7fff7fff, and
 * confusing them costs a wrong "we diverge from stock" reading:
 *
 *  (1) the DIRECT registers in cortina_ni_deepq_cb_cfg[] - 0x2d80/0x2d88/0x2d8c/
 *      0x2d90/0x2d94/0x2d98 and 0x2ec8..0x2ee8.  These are tier-1 stock live golden
 *      (dev/x400axf/stock/stock_dqsch_2d00.txt, devmem on stock 2026-07-12): stock
 *      itself reads 0x7FFF7FFF there.  We MATCH stock at every one of them - they
 *      are NOT a divergence and are deliberately NOT touched by the knob below.
 *
 *  (2) the two INDEXED per-VoQ threshold profile tables, written through the
 *      two-phase ACCESS/DATA protocol:
 *        DQSCH VOQ  ACCESS 0x2e70 / DATA  0x2e74   - stock residual 0x00700070
 *                                                    (lo = hi = 0x70 = 112)
 *        CB    VOQ  ACCESS 0x2da0 / DATA1 0x2da4   - stock residual 0x0FFFFFFF
 *                            DATA0 0x2da8         - stock residual 0xFFFFFFFF
 *      Here we DO diverge: we write the permissive 0x7fff7fff into all 8 entries of
 *      both tables.  On the DQSCH table that is 32767 vs stock's 112 = ~292x DEEPER,
 *      the prime bufferbloat suspect on the upstream path (the central buffer is the
 *      reservoir that fills when the GPON upstream grant is the bottleneck).  On the
 *      CB table it is the other way round: stock's {0x0FFFFFFF, 0xFFFFFFFF} is far
 *      MORE permissive than our 0x7fff7fff, so "match stock" there means going
 *      DEEPER, not shallower.
 *      ⚠ The stock values are the DATA-register RESIDUAL after stock's last indexed
 *      write (ACCESS read back as 0x40000007 = last index 7), i.e. proven for entry
 *      7 and assumed uniform across the 8 entries - the same assumption our own
 *      uniform write makes.
 *
 * ★ SCOPE - WHICH traffic this actually governs (know it before reading a result):
 * only frames that take the DEEP-QUEUE path go through these thresholds.
 *   - HW-offloaded UPSTREAM (LAN->WAN) flows: YES, but CONDITIONALLY - the L3FE US
 *     egress action sets deepq only when the live data T-CONT <= 7
 *     (CN_L3E_PON_DEEPQ_TCONT_MAX in cortina-ni-flowoffload.c).  Read the live
 *     T-CONT off the box first: /proc/cortina_l3fe prints `live_pon{gem= tcont=}`.
 *     With a T-CONT above 7 the upstream offloaded path does NOT use the deep
 *     queue and this knob cannot move an upstream number at all.
 *   - CPU-forwarded UPSTREAM frames (the pon_data_enq path in cortina-ni-tx.c):
 *     NO - that path never sets deep_q in its HEADER_A.
 *   - HW-offloaded DOWNSTREAM flows: only with hw_ds_deepq=1, which is default OFF.
 *   - CPU-RX (frames trapped to the CPU): YES - this is the path the deep queue was
 *     brought up for in the first place.
 *
 * DEFAULT = the CURRENT permissive behaviour, byte-identical to what shipped: this
 * knob exists to MEASURE the trade (depth vs latency vs throughput), and picking a
 * different default is the operator's call, from numbers, never a silent change.
 *
 * RUNTIME-settable (not boot-time only): the tables are plain indexed RAM behind the
 * ACCESS/DATA protocol, so a write re-walks all 8 entries of both tables in place.
 *   echo 0x00700070 > /sys/module/cortina_ni/parameters/deepq_voq_thrsh
 *   echo 1          > /sys/module/cortina_ni/parameters/deepq_cb_stock
 * A bootarg works too and is the cleaner A/B (no in-flight transition):
 *   cortina_ni.deepq_voq_thrsh=0x00700070 cortina_ni.deepq_cb_stock=1
 * Read the ACTIVE value back off the running device with
 *   grep deepq-thrsh /proc/net/cortina_ni_rx
 * which prints BOTH the parameters AND a live indirect read of the HW table, so a
 * benchmark's configuration is provable after the fact.
 */
static unsigned int deepq_voq_thrsh = CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE;
static bool deepq_cb_stock;
/* set once the NI is probed, so a sysfs write can re-walk the tables */
static struct cortina_ni *cortina_ni_deepq_ni;
static DEFINE_MUTEX(cortina_ni_deepq_lock);

/* Write the CURRENT parameter values into all 8 entries of BOTH per-VoQ threshold
 * profile tables (DQSCH @0x2e70/0x2e74 and CB @0x2da0/0x2da4/0x2da8).  Every entry
 * of every table, always - a partial walk would leave some VoQs deep and make any
 * measurement meaningless. */
static void cortina_ni_rx_deepq_thrsh_program(struct cortina_ni *ni)
{
	u32 dq = READ_ONCE(deepq_voq_thrsh);
	bool cbs = READ_ONCE(deepq_cb_stock);
	u32 cb1 = cbs ? CA_NI_L2TM_CB_VOQ_THRSH_D1
		      : CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE;
	u32 cb0 = cbs ? CA_NI_L2TM_CB_VOQ_THRSH_D0
		      : CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE;
	unsigned int i;

	mutex_lock(&cortina_ni_deepq_lock);
	for (i = 0; i < CA_NI_L2TM_DEEPQ_VOQ_ENTRIES; i++) {
		writel(dq, ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS, i);

		writel(cb1, ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA1);
		writel(cb0, ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA0);
		cortina_ni_rx_ind_store(ni, CA_NI_L2TM_CB_VOQ_THRSH_ACCESS, i);
	}
	mutex_unlock(&cortina_ni_deepq_lock);

	dev_info(ni->dev,
		 "deepq-thrsh: %u entries x {dqsch(0x2e74)=0x%08x, cb(0x2da4/0x2da8)={0x%08x,0x%08x}} [dqsch stock=0x%08x permissive=0x%08x; cb stock={0x%08x,0x%08x}]\n",
		 CA_NI_L2TM_DEEPQ_VOQ_ENTRIES, dq, cb1, cb0,
		 CA_NI_L2TM_DQSCH_VOQ_THRSH_VAL,
		 CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE,
		 CA_NI_L2TM_CB_VOQ_THRSH_D1, CA_NI_L2TM_CB_VOQ_THRSH_D0);
}

/* Live readback of one entry of each table (indirect read: ACCESS = GO|idx, then
 * the entry is latched in DATA).  Bounded + non-fatal; the ACCESS word is printed
 * alongside so a GO that never cleared is visible instead of silently believed. */
static void cortina_ni_rx_deepq_thrsh_read(struct cortina_ni *ni, unsigned int idx,
					   u32 *dq, u32 *cb1, u32 *cb0)
{
	mutex_lock(&cortina_ni_deepq_lock);
	cortina_ni_rx_ind_read(ni, CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS, idx);
	*dq = readl(ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_DATA);
	cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_VOQ_THRSH_ACCESS, idx);
	*cb1 = readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA1);
	*cb0 = readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_DATA0);
	mutex_unlock(&cortina_ni_deepq_lock);
}

static int cortina_ni_rx_deepq_thrsh_reprogram(void)
{
	struct cortina_ni *ni = READ_ONCE(cortina_ni_deepq_ni);

	/* set before probe (bootarg / insmod): the init walk picks it up */
	if (ni)
		cortina_ni_rx_deepq_thrsh_program(ni);
	return 0;
}

static int deepq_voq_thrsh_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	return ret ? ret : cortina_ni_rx_deepq_thrsh_reprogram();
}

static int deepq_cb_stock_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	return ret ? ret : cortina_ni_rx_deepq_thrsh_reprogram();
}

static const struct kernel_param_ops deepq_voq_thrsh_ops = {
	.set	= deepq_voq_thrsh_set,
	.get	= param_get_uint,
};
static const struct kernel_param_ops deepq_cb_stock_ops = {
	.flags	= KERNEL_PARAM_OPS_FL_NOARG,	/* bare name = 1, like a bool param */
	.set	= deepq_cb_stock_set,
	.get	= param_get_bool,
};

module_param_cb(deepq_voq_thrsh, &deepq_voq_thrsh_ops, &deepq_voq_thrsh, 0644);
MODULE_PARM_DESC(deepq_voq_thrsh,
	"L2TM DQSCH per-VoQ admission threshold, all 8 profile entries (0x2e70/0x2e74). 0x7fff7fff = permissive/DEFAULT (32767 per half, ~292x stock - the shipping behaviour, unchanged); 0x00700070 = the tier-1 stock value (112 per half); any other value is written verbatim. RUNTIME-settable (re-walks the table); read the active value back with 'grep deepq-thrsh /proc/net/cortina_ni_rx'");

module_param_cb(deepq_cb_stock, &deepq_cb_stock_ops, &deepq_cb_stock, 0644);
MODULE_PARM_DESC(deepq_cb_stock,
	"L2TM central-buffer per-VoQ threshold, all 8 profile entries (0x2da0/0x2da4/0x2da8). 0 = DEFAULT 0x7fff7fff in both words (the shipping behaviour); 1 = the tier-1 stock pair {hi=0x0fffffff, lo=0xffffffff}, which is DEEPER than ours, not shallower. RUNTIME-settable");

/*
 * ★★ THE DEEP-QUEUE / DQSCH init (stock aal_l2_tm_cb_init) - the block our driver
 * never ran, so a deep_q=1 frame is enqueued into the central buffer but the DQSCH +
 * arbiter never dequeue it to ES port 7 -> RMU0 (0x6900 stuck 0, NO drop).  Vendor
 * ORDER (RE-confirmed): all DIRECT thresholds/watermarks/credits/per-queue cfg ->
 * the INDEXED per-VOQ profile tables (two-phase, a flat write of the 0x40000007
 * resting value is a GO=0 no-op that never loads the RAM) -> ARB dbuf_sel -> then the
 * TWO MASTER ENABLES LAST and IN ORDER: 0x2d0c bit31 (cb scheduler run), then 0x2eec
 * bit31 (ABR arbiter, FINAL op).  Arming before the config is loaded runs an
 * unconfigured scheduler that still drops - the order is mandatory.
 */
static void cortina_ni_rx_deepq_sched_init(struct cortina_ni *ni)
{
	unsigned int i;

	/* (1) DIRECT threshold/watermark/credit/per-queue config (stock resting values) */
	for (i = 0; i < ARRAY_SIZE(cortina_ni_deepq_cb_cfg); i++)
		writel(cortina_ni_deepq_cb_cfg[i].val,
		       ni_base(ni) + cortina_ni_deepq_cb_cfg[i].off);

	/* (2) CB per-port free-buffer count (indexed, all 48 ports) - without it the CB
	 * has 0 free deep-queue buffers and drops the frame at the L2TM->CB enqueue. */
	for (i = 0; i < CA_NI_L2TM_CB_PORT_COUNT; i++) {
		writel(CA_NI_L2TM_CB_FREECNT_VAL,
		       ni_base(ni) + CA_NI_L2TM_CB_PORT_FREECNT_DATA);
		cortina_ni_rx_ind_store(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, i);
	}

	/* (3) the two INDEXED per-VOQ profile tables (8 entries each) via the two-phase
	 * ACCESS-command protocol (DATA then ACCESS=idx|GO|WR, poll GO clear).  A 0
	 * profile = zero threshold = drop-all, so the value must be non-zero for a frame
	 * to be admitted at all.  The VALUE is the runtime-selectable queue depth
	 * (deepq_voq_thrsh / deepq_cb_stock, default = the permissive shipping value);
	 * table A = DQSCH VOQ (0x2e74/0x2e70), table B = CB VOQ (0x2da4+0x2da8/0x2da0). */
	WRITE_ONCE(cortina_ni_deepq_ni, ni);
	cortina_ni_rx_deepq_thrsh_program(ni);

	/* (4) ARB_CTRL.dbuf_sel (bit1)=1 so a PDPID-8 frame takes the deep-buffer path */
	ni_rmw(ni, CA_NI_L2FE_ARB_CTRL, 0, CA_NI_L2FE_ARB_DBUF_SEL);

	/* ★★ build38 (4b) THE DQSCH-OUTPUT -> TM-port binding (0x2f00/04/08) - the static
	 * config region our driver skipped entirely.  Without it the deep-queue dequeue
	 * drains to the WRONG TM-port (branch-3: bm_tx 0x2140 +9 but 0xa9fc/L3QM=0, no drop);
	 * 0x2f04=0x33445550 is the port map that should route the drain to TM-port 8 = L3QM.
	 * Stock live golden, written verbatim (0x2f0c+ are live counters - never write). */
	writel(CA_NI_L2TM_DQSCH_OUT_CFG0_VAL,	  ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_CFG0);
	writel(CA_NI_L2TM_DQSCH_OUT_PORT_MAP_VAL, ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_PORT_MAP);
	writel(CA_NI_L2TM_DQSCH_OUT_CFG2_VAL,	  ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_CFG2);

	/* (5) THE TWO MASTER ENABLES - LAST, IN ORDER (cb_ctrl then abr_ctrl) */
	writel(CA_NI_L2TM_CB_CTRL_STOCK, ni_base(ni) + CA_NI_L2TM_CB_CTRL);
	writel(CA_NI_L2TM_CB_ABR_CTRL_STOCK, ni_base(ni) + CA_NI_L2TM_CB_ABR_CTRL);

	dev_info(ni->dev,
		 "deepq-sched: cb_ctrl(0x2d0c)=0x%08x abr(0x2eec)=0x%08x out_map(0x2f04)=0x%08x dqschA(0x2e70)=0x%08x cbB(0x2da0)=0x%08x arb=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L2TM_CB_CTRL),
		 readl(ni_base(ni) + CA_NI_L2TM_CB_ABR_CTRL),
		 readl(ni_base(ni) + CA_NI_L2TM_DQSCH_OUT_PORT_MAP),
		 readl(ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS),
		 readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_ACCESS),
		 readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL));
}

/*
 * ★★ build36: enable port MAC blocks 1-6 (stock enables all 7; ports 0-4 = physical
 * GMAC, ports 5-6 = INTERNAL CPU/QM/L3QM-facing).  Our driver only ever init'd port 0,
 * so the internal ports 5/6 had rx_en/tx_en=0 - a frame egressed L2TM but the internal
 * port that hands it to L3QM never accepted it -> ni2qm_rx (0xa9fc, itself a counter in
 * that internal-port block) stayed flat 0.  Stock live golden: p1-6 RXMAC=0x3101,
 * TXMAC=0x04055901, RX_CNTRL=0x08000600.  Port 0 is skipped (its RXMAC/TXMAC are the
 * port-0 GPHY-link variant 0x3001/0x04054901, programmed by the link path).  Idempotent.
 */
static void cortina_ni_rx_enable_internal_ports(struct cortina_ni *ni)
{
	int p;

	for (p = 1; p < CA_NI_PORT_COUNT; p++) {
		writel(CA_NI_PORT_RXMAC_EN_VAL,
		       ni_base(ni) + CA_NI_PORT_RXMAC_CFG(p));
		writel(CA_NI_PORT_TXMAC_EN_VAL,
		       ni_base(ni) + CA_NI_PORT_TXMAC_CFG(p));
		writel(CA_NI_PORT_RX_CNTRL_STOCK_VAL,
		       ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(p));
	}
	dev_info(ni->dev,
		 "ports 1-6 enabled (internal 5/6=CPU/QM/L3QM): p5 rxmac=0x%08x txmac=0x%08x; p6 rxmac=0x%08x txmac=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(5)),
		 readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(5)),
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(6)),
		 readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(6)));
}

/*
 * ★★★ build72: the L3-CLS special-packet TRAP - VERBATIM replication of the live-stock
 * CPU-trap rule set (tier-1 golden, dev/x400axf/stock_golden_qm.txt).  Stock's
 * broadcast ARP reaches the CPU (30/30 replies) via KEY-based rules (the ethertype
 * FIELD_CAM @0x3200 is EMPTY -> NOT ethertype-based).  build71 HAND-ENCODED a rule that
 * was written but never fired; build72 instead writes stock's EXACT KEY+FIB words (table
 * values are facts) to the same rows on our identical silicon.
 *
 * The CPU-trap rows (key_type=0 IF_ID; FIB index = (key_row<<2)|sub_slot):
 *   KEY[0] (pri 9, slots 0+1) -> FIB[0],FIB[1]  = MAC-DA IP-multicast / an_hit -> CPU_0
 *   KEY[1] (pri 1, slot 0)    -> FIB[4]         = non-broadcast unicast     -> CPU_0
 *   KEY[2] (pri 0, slot 0)    -> FIB[8]         = ALL-WILDCARD (broadcast)  -> CPU_0
 * All FIBs: DATA4=0x1C000000 (dpid_vld/dpid_pri/permit=1), DATA5=0x01000004 (mcgid=0x10
 * CPU_0).  A broadcast ARP entering the CLS falls to KEY[2] (all-wildcard, pri 0) ->
 * FIB[8] -> dest CPU_0 (0x10), dpid_pri wins -> CPU-EPP64.
 *
 * Word arrays below are struct word[0..N-1] (word0 = low bits).  Golden was dumped
 * word10..word0 (reg 0x3384=word10 trailer .. 0x33ac=word0); reversed here to word0-up.
 * Write protocol (generic aal_table): struct word[i] -> ACCESS + (N-i)*4; then kick
 * ACCESS = GO|WR|idx, poll GO clear.  Commit ALL FIBs, then ALL KEYs.
 */
static bool cls_trap_enable = true;
module_param(cls_trap_enable, bool, 0644);
MODULE_PARM_DESC(cls_trap_enable, "install the stock L3-CLS ->CPU_0 trap rows (ARP-to-CPU)");

/* stock KEY rows, struct word0..word10 (verbatim, reversed from the word10..word0 dump).
 * ★★ build75: the CLS key table is PARTITIONED - profile 0 = KEY[0..63] (WAN ingress),
 * profile 1 = KEY[64..127] (LAN ingress; STG0 LPB t1_ctrl selects the profile).  A LAN
 * broadcast ARP searches ONLY profile 1, so the profile-0 rows (0/1/2) were INVISIBLE to
 * it (cls_hit=0).  Duplicate the same 3 CPU-trap rows into profile 1 at rows 64/65/66. */
static const struct { u16 idx; u32 w[CA_NI_L3FE_CLS_KEY_WORDS]; } cls_key_golden[] = {
	/* profile 0 (WAN) */
	{ 0, { 0xFFFFFEFF, 0xFEFFFFFF, 0xF77FFFFF, 0xFFFFFFFF, 0xFFFDFFFF,
	       0x0000003F, 0, 0, 0, 0, 0x18240000 } },
	{ 1, { 0xFFFFFECF, 0xCFFFFFFF, 0x0007FFFF, 0, 0,
	       0, 0, 0, 0, 0, 0x08040000 } },
	{ 2, { 0xFFFFFFFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
	       0, 0, 0, 0, 0, 0x08000000 } },
	/* profile 1 (LAN) - identical rows at +64 */
	{ 64, { 0xFFFFFEFF, 0xFEFFFFFF, 0xF77FFFFF, 0xFFFFFFFF, 0xFFFDFFFF,
		0x0000003F, 0, 0, 0, 0, 0x18240000 } },
	{ 65, { 0xFFFFFECF, 0xCFFFFFFF, 0x0007FFFF, 0, 0,
		0, 0, 0, 0, 0, 0x08040000 } },
	{ 66, { 0xFFFFFFFF, 0xFFFFFFFF, 0x0007FFFF, 0, 0,
		0, 0, 0, 0, 0, 0x08000000 } },
};

/* stock FIB rows, struct word0..word6 (verbatim; word0=DATA0=0x33cc low .. word6=DATA6).
 * FIB idx = (key_row<<2)|slot: profile0 rows 0/1/2 -> 0,1,4,8; profile1 rows 64/65/66 ->
 * 256,257,260,264 (KEY64 slots0/1, KEY65 slot0, KEY66 slot0 = the LAN bcast catch-all). */
static const struct { u16 idx; u32 w[CA_NI_L3FE_CLS_FIB_WORDS]; } cls_fib_golden[] = {
	/* profile 0 (WAN) */
	{ 0,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 1,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 4,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },
	{ 8,   { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000600 } },
	/* profile 1 (LAN) */
	{ 256, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 257, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000A00 } },
	{ 260, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000200 } },
	{ 264, { 0, 0, 0, 0, 0x1C000000, 0x01000004, 0x00000600 } },
};

/*
 * ★★ Replicate the vendor L3FE GLOBAL init that our driver never ran (the whole missing
 * enter-L3 stage).  Vendor: aal_l3fe_l2lookup_init (aal_l3fe.c:269-310) + the glb ELPB/
 * deep-queue setters (aal_l3fe_glb_elpb_set :215, _elpb_deepq_vld_set :238,
 * _elpb_deepq_set :261), all reached from aal_l3fe_init (:1030).  Without this the
 * L3FE_GLB block (0x30ac-0x30f8) sat at 0, so the L3FE stage was uninitialized and never
 * ingested frames -> l3fe_rx(0xa9bc)=0 -> cls_hit=0 -> null CPU-EPP descriptors.  The
 * values are tier-1 LIVE STOCK (captured under broadcast) because the SDK's derived values
 * do NOT match Elnath (e.g. ILPB_LDPID 0x30d8 is 0 on stock but non-zero in the SDK) - so
 * we replicate STOCK, not the SDK arithmetic.  ILPB_LDPID (0x30d8) is deliberately left 0.
 */
static void cortina_ni_rx_l3fe_glb_init(struct cortina_ni *ni)
{
	/* forwarding control 1/2/3 + ingress-FIFO thresholds (LF_CFG) + ingress-loopback
	 * VLAN config (ILPB entry0) + the (unnamed-in-SDK but stock-mapped) 0x30cc slot.
	 * LF_CFG at 0 leaves the L3FE ingress FIFO thresholds zero -> it never accepts a
	 * frame -> l3fe_rx=0; these were the last L3FE_GLB regs our driver left unset. */
	writel(CA_NI_L3FE_GLB_FWD_CTRL_1_VAL, ni_base(ni) + CA_NI_L3FE_GLB_FWD_CTRL_1);
	writel(CA_NI_L3FE_GLB_FWD_CTRL_2_VAL, ni_base(ni) + CA_NI_L3FE_GLB_FWD_CTRL_2);
	writel(CA_NI_L3FE_GLB_FWD_CTRL_3_VAL, ni_base(ni) + CA_NI_L3FE_GLB_FWD_CTRL_3);
	writel(CA_NI_L3FE_GLB_LF_CFG_VAL, ni_base(ni) + CA_NI_L3FE_GLB_LF_CFG);
	writel(CA_NI_L3FE_GLB_ILPB_00_VAL, ni_base(ni) + CA_NI_L3FE_GLB_ILPB_00);
	writel(CA_NI_L3FE_GLB_CFG_30CC_VAL, ni_base(ni) + CA_NI_L3FE_GLB_CFG_30CC);

	/* egress-loopback entry + deep-queue valid-vec + deep-queue vec (the ELPB block
	 * that lets an L2FE frame loop into the L3FE ingress) */
	writel(CA_NI_L3FE_GLB_ELPB0_VAL, ni_base(ni) + CA_NI_L3FE_GLB_ELPB0);
	writel(CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD1_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD1);
	writel(CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD0_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD0);
	writel(CA_NI_L3FE_GLB_ELPB_DEEPQ1_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ1);
	writel(CA_NI_L3FE_GLB_ELPB_DEEPQ0_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ0);

	/* the L3FE<->L2FE loopback ldpid binding + the VLAN-edit tpid config */
	writel(CA_NI_L3FE_GLB_L3FE_L2FE_LDPID_VAL,
	       ni_base(ni) + CA_NI_L3FE_GLB_L3FE_L2FE_LDPID);
	writel(CA_NI_L3FE_GLB_VE_VAL, ni_base(ni) + CA_NI_L3FE_GLB_VE);

	dev_info(ni->dev,
		 "l3fe-glb-init: fwd1(0x30a4)=0x%08x fwd2(0x30a8)=0x%08x lf_cfg(0x30b4)=0x%08x ilpb00(0x30bc)=0x%08x fwd3(0x30ac)=0x%08x 30cc=0x%08x elpb0(0x30e0)=0x%08x dqvld=0x%08x/0x%08x dq=0x%08x/0x%08x l2fe_ldpid(0x30f4)=0x%08x ve(0x30f8)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_FWD_CTRL_1),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_FWD_CTRL_2),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_LF_CFG),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ILPB_00),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_FWD_CTRL_3),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_CFG_30CC),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ELPB0),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD1),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ_VLD0),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ1),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_ELPB_DEEPQ0),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_L3FE_L2FE_LDPID),
		 readl(ni_base(ni) + CA_NI_L3FE_GLB_VE));
}

/*
 * ★★ build96: the L3FE AXI read-reorder channel init (vendor aal_l3fe_axi_reo_init,
 * aal_l3fe.c:341, run LAST in aal_l3fe_init).  Our driver programs the main NI AXI-REO
 * (cortina_ni_rx_axi_reo_init, low offsets) but SKIPS the L3FE channel at win10+0x2080.
 * Without it the L3FE cannot AXI-fetch/reorder a frame from memory -> never ingests ->
 * l3fe_rx(0xa9bc)=0.  Uses the same AXI-REO window (idx10) as the main reorder.  ★ the
 * values are SDK-derived (aal_l3fe_axi_reo_init) - flag for stock validation.
 */
static void cortina_ni_rx_l3fe_axi_reo_init(struct cortina_ni *ni)
{
	void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];

	if (!reo) {
		dev_warn(ni->dev, "RX: L3FE AXI-REO window (idx %d) not mapped\n",
			 CA_NI_WIN_AXI_REO);
		return;
	}
	writel(CA_NI_L3FE_AXI_REO_ORIG_ID_VAL, reo + CA_NI_L3FE_AXI_REO_ORIG_ID);
	writel(CA_NI_L3FE_AXI_REO_NEW_ID_VAL, reo + CA_NI_L3FE_AXI_REO_NEW_ID);
	writel(CA_NI_L3FE_AXI_REO_TOP_ADDR_VAL, reo + CA_NI_L3FE_AXI_REO_TOP_ADDR);
	writel(CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK_VAL,
	       reo + CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK);
	writel(CA_NI_L3FE_AXI_REO_NEW_ID0_VAL, reo + CA_NI_L3FE_AXI_REO_NEW_ID0);
	writel(CA_NI_L3FE_AXI_REO_RD18_VAL, reo + CA_NI_L3FE_AXI_REO_RD18);
	writel(CA_NI_L3FE_AXI_REO_RD24_VAL, reo + CA_NI_L3FE_AXI_REO_RD24);

	dev_info(ni->dev,
		 "l3fe-axi-reo (win10+0x480, abs 0xf432d480): orig=0x%08x new=0x%08x top=0x%08x mask=0x%08x new0=0x%08x +18=0x%08x +24=0x%08x (want 2/8|/1e8/1e8-mask/9|/FFFFFFFF x2)\n",
		 readl(reo + CA_NI_L3FE_AXI_REO_ORIG_ID),
		 readl(reo + CA_NI_L3FE_AXI_REO_NEW_ID),
		 readl(reo + CA_NI_L3FE_AXI_REO_TOP_ADDR),
		 readl(reo + CA_NI_L3FE_AXI_REO_TOP_ADDR_MASK),
		 readl(reo + CA_NI_L3FE_AXI_REO_NEW_ID0),
		 readl(reo + CA_NI_L3FE_AXI_REO_RD18),
		 readl(reo + CA_NI_L3FE_AXI_REO_RD24));
}

static void cortina_ni_rx_cls_init(struct cortina_ni *ni)
{
	unsigned int e, i;

	if (!cls_trap_enable)
		return;

	/* ★★ build73: fix L3FE STG0_CTRL first - our reset default 0x001c7c7e has 2 bits
	 * (1,10) stock's stg0_set_normal clears (lpb_idx_mode=0 etc).  A wrong lpb_idx_mode
	 * mis-indexes the STG0 LPB so the CLS lookup can't match our LAN ingress; build72's
	 * byte-exact rows never fired.  Match stock's 0x001c787c before installing the rules. */
	writel(CA_NI_L3FE_STG0_CTRL_VAL, ni_base(ni) + CA_NI_L3FE_STG0_CTRL);

	/* commit ALL FIBs first, then ALL KEYs (stock aal_l3_cls_add order) */
	for (e = 0; e < ARRAY_SIZE(cls_fib_golden); e++) {
		for (i = 0; i < CA_NI_L3FE_CLS_FIB_WORDS; i++)
			writel(cls_fib_golden[e].w[i],
			       ni_base(ni) + CA_NI_L3FE_CLS_FIB_ACCESS +
			       (CA_NI_L3FE_CLS_FIB_WORDS - i) * 4);
		cortina_ni_rx_ind_store(ni, CA_NI_L3FE_CLS_FIB_ACCESS,
					cls_fib_golden[e].idx);
	}
	for (e = 0; e < ARRAY_SIZE(cls_key_golden); e++) {
		for (i = 0; i < CA_NI_L3FE_CLS_KEY_WORDS; i++)
			writel(cls_key_golden[e].w[i],
			       ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS +
			       (CA_NI_L3FE_CLS_KEY_WORDS - i) * 4);
		cortina_ni_rx_ind_store(ni, CA_NI_L3FE_CLS_KEY_ACCESS,
					cls_key_golden[e].idx);
	}

	/*
	 * ★ Tier-1 stock-diff (2026-07-23, live devmem on stock NAND): under
	 * hw_l3_fwd, re-provision ONLY the PP FIELD-CAM router-MAC entries
	 * (intf_add).  The golden CLS rows written above (0/1/2 + 64/65/66)
	 * already carry stock's t2_ctrl + CPU_0 miss disposition byte-for-byte
	 * (cls_fib_golden == stock FIB 0/4/8/256/260/264), so they need NO
	 * per-link-up rewrite.  The earlier build stamped t2_ctrl=3 onto the LAN
	 * golden FIBs and added an STG0 an-mask + pri-6 routed rows - NONE of
	 * which stock does (stock LPB an-mask=0, CLS rows 3/67 empty); that
	 * corrupted the routed path on enable and is removed.  A routed frame
	 * hits its golden CLS row -> runs T2 (t2_ctrl) -> HIT forwards, MISS
	 * punts to CPU_0 (the row's own dpid) - exactly stock.
	 */
	if (cortina_ni_hw_l3_fwd_active() && ni->tx && ni->tx->netdev) {
		int ret = cortina_l3fe_intf_add(ni_base(ni),
						ni->tx->netdev->dev_addr);

		dev_info(ni->dev, "cls: PP MAC-DA router-CAM re-applied %s\n",
			 ret ? "FAILED" : "ok");
	}

	/* read back the broadcast row (KEY[2]) for the boot log */
	cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_KEY_ACCESS, 2);
	dev_info(ni->dev,
		 "cls-trap: stg0_ctrl(0x3400)=0x%08x (want 0x001c787c); rows 0/1/2 + fib 0/1/4/8 written; KEY[2] w0(0x33ac)=0x%08x trailer(0x3384)=0x%08x det_cfg(0x3218)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_L3FE_STG0_CTRL),
		 readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 11 * 4),
		 readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 1 * 4),
		 readl(ni_base(ni) + CA_NI_L3FE_SPCL_PKT_DET_CFG));
}

static int cortina_ni_rx_steer_init(struct cortina_ni *ni)
{
	int type, ret;

	/* ★ L2TM BM dequeue->TM-port map (stock __ni_flow_ctrl_init) - MUST run so
	 * a dequeued CPU-dest frame reaches the QM (else qm_rx_cntr stays 0). */
	cortina_ni_rx_flow_ctrl_init(ni);

	/* ★★ Deep-queue/central-buffer SCHEDULER (DQSCH) - drains the deep queue a
	 * deep_q=1 frame lands in, to the RMU.  Absent = frame enqueued but never
	 * drained (0x6900=0).  Must run alongside the L2TM ES enable. */
	cortina_ni_rx_deepq_sched_init(ni);

	/* ★★ Enable the L2TM egress scheduler (ES port 8 = L3QM) so the deep-queue
	 * frame is actually drained OUT of the L2TM to the QM -> RMU -> CPU.  At
	 * reset the scheduler is OFF (ES_CTRL=0x24000000), so without this the frame
	 * sits in the L2TM (tm rx climbs) and RMU0 never admits it (0x6900=0). */
	cortina_ni_rx_l2tm_es_init(ni);

	/* ★★ THE blackhole root-cause fix: L2FE per-port profiles (ILPB stp,
	 * MMSHP isolation, ELPB egr-stp, IPPB map).  Unprogrammed = force-DROP
	 * upstream of every forwarding table, resolving all frames to 0x1f. */
	cortina_ni_rx_port_profiles_init(ni);

	/* one-shot L2E hash (FDB) init: with stp=fwd+LEARN the engine now
	 * learns SAs and looks up DAs - an uninitialized hash SRAM could
	 * false-hit and mis-steer unicast.  Once only: re-running on a link
	 * bounce would flush learned entries. */
	{
		static bool fdb_hash_ready;

		if (!fdb_hash_ready) {
			u32 acc;

			writel(CA_NI_L2FE_FDB_GO | CA_NI_L2FE_FDB_OP_INIT,
			       ni_base(ni) + CA_NI_L2FE_FDB_ACCESS);
			if (readl_poll_timeout(ni_base(ni) + CA_NI_L2FE_FDB_ACCESS,
					       acc, !(acc & CA_NI_L2FE_FDB_GO),
					       10, 20000))
				dev_warn(ni->dev, "fdb hash init timeout (0x%08x)\n",
					 acc);
			else
				fdb_hash_ready = true;
		}
	}

	/* L2FE forwarding-control regs (ARB/PLC/PLE_DEFAULT) = stock byte-for-byte;
	 * the DFT_FWD loop below sets the port-0 DLF -> CPU redir (0x1832).  No
	 * mc-flood (mce_indx/MC_FIB): stock uses plain REDIR, and the MCE path
	 * SErrored - it was never stock's mechanism. */
	cortina_ni_rx_l2fe_forwarding(ni);

	/* ★★ 2026-07-13 (ca-ne.ko RE aaeb153d + live-stock): the FDB GLOBAL control
	 * (aal_fdb_ctrl_set 0x1c00/0x1c04) enables the per-type FDB forwarding actions
	 * so unknown-DA / broadcast frames take the LRN_FWD_CTRL dest 0x10 (CPU_0).  We
	 * wrote LRN_FWD_CTRL (0x1408/0x140c) but LEFT 0x1c00/0x1c04 at RESET, so the FDB
	 * action never activated -> DLF/BC fell through to the DFT_FWD flood -> forwarded
	 * OUT (bm_rx==bm_tx, l3fe_rx=0).  Golden: 0x1c00=0x82BFEFF9, 0x1c04=0x9F90012C. */
	writel(CA_NI_L2FE_FDB_CTRL_0_VAL, ni_base(ni) + CA_NI_L2FE_FDB_CTRL_0);
	writel(CA_NI_L2FE_FDB_CTRL_1_VAL, ni_base(ni) + CA_NI_L2FE_FDB_CTRL_1);

	/* ★★ 2026-07-13: initialise the FDB engine (OP_INIT builds the hash table) +
	 * add the own-MAC entry.  Without OP_INIT the FDB is dead, so the FDB
	 * forwarding-control (0x1408/0x140c/0x1c00/0x1c04, unknown-DA/BC -> CPU_0)
	 * never takes effect and DLF/broadcast frames fall through to DFT_FWD ->
	 * forwarded out (l3fe_rx=0).  fdb_add_cpu was defined but never called. */
	cortina_ni_rx_fdb_add_cpu(ni);

	/* ★ ARB deep-queue: DeepQ_0 -> PDPID=QM -> ES port 7 -> RMU -> CPU (PDPID map
	 * + dbuf_dpid + REDIR[0x1f] belt).  Must run before the DFT_FWD redir. */
	cortina_ni_rx_arb_deepq_init(ni);
	cortina_ni_rx_flow_dbuf_init(ni);	/* zero the deep_q source (dbuf_sel=1) to stock - build100's 0x0f marks were the CPU-RX regression */

	/* ★ Build the REAL one-member MC group (member ldpid = DeepQ_0) that DFT_FWD
	 * replicates DLF frames to.  Without it, DFT_FWD -> reserved null group 0 ->
	 * blackhole(0x1f) drop.  Must run before the DFT_FWD loop points at the group. */
	cortina_ni_rx_mc_group_init(ni);

	/* ★ THE own-MAC CPU trap: an own-MAC frame is classified MYMAC and resolved
	 * BEFORE the DA-FDB (so a FDB entry is never consulted for it).  Program our
	 * MAC into PP_MY_MAC + set the MYMAC SPB action -> force-forward to DeepQ_0 ->
	 * PDPID=QM -> RMU -> CPU.  (The FDB path cortina_ni_rx_fdb_add_cpu is kept for
	 * a future learned/foreign unicast but is not the own-MAC path.) */
	cortina_ni_rx_mymac_trap(ni);

	/* ★★ THE missing L3FE global init (0x30ac-0x30f8) - runs BEFORE the CLS rows,
	 * mirroring the vendor aal_l3fe_init order (l2lookup_init/glb-setters before the
	 * classifier rules).  This is what actually lets a frame INGRESS the L3FE stage
	 * (l3fe_rx) so the CLS trap below can even see it. */
	cortina_ni_rx_l3fe_glb_init(ni);

	/* ★★ build96: the L3FE AXI read-reorder channel - vendor runs aal_l3fe_axi_reo_init
	 * LAST in aal_l3fe_init (after l2lookup).  This is the L3FE's own DMA read-reorder,
	 * distinct from the main NI AXI-REO we already program - lets the L3FE fetch the
	 * frame from memory so it can ingest (l3fe_rx). */
	cortina_ni_rx_l3fe_axi_reo_init(ni);

	/* ★★★ build71: the L3-CLS special-packet trap - broadcast (ARP) -> CPU_0 via a
	 * TCAM rule with dpid_pri=1 (dest 0x10 wins, cpu-bound -> CPU-EPP64).  This is
	 * stock's ARP-to-CPU mechanism (ethertype trap), matched here on broadcast MAC-DA
	 * to avoid the ethertype-CAM.  Runs after the L2FE/MC setup so it overrides the
	 * broadcast->L3_LAN(0x19) resolution. */
	cortina_ni_rx_cls_init(ni);

	/* full FE path: byp OFF, pass OAM, drop unknown opcodes (stock policy).
	 * ★ Apply to EVERY GPHY LAN port, not just CA_NI_RX_PORT, so whichever port
	 * has the host cable runs the FE lookup (and hits the DLF trap below). */
	{
		unsigned int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++)
			ni_rmw(ni, CA_NI_PORT_RX_CNTRL_CFG(p),
			       CA_NI_RX_CNTRL_BYP_EN | CA_NI_RX_CNTRL_BYP_DPID |
			       CA_NI_RX_CNTRL_BYP_COS | CA_NI_RX_CNTRL_UKOP_DROP_DIS,
			       CA_NI_RX_CNTRL_OAM_DROP_DIS);
	}

	/* L3FE demux golden routing map (FE output -> L3QM CPU-EPP) */
	writel(CA_NI_NIRX_L3FE_DEMUX0_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX0);
	writel(CA_NI_NIRX_L3FE_DEMUX1_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX1);
	writel(CA_NI_NIRX_L3FE_DEMUX2_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX2);
	writel(CA_NI_NIRX_L3FE_DEMUX3_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX3);
	writel(CA_NI_NIRX_L3FE_DEMUX4_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX4);
	writel(CA_NI_NIRX_L3FE_DEMUX5_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX5);

	/* ★★ build34/36: the DEEP-QUEUE (deep_q=1) parallel per-ldpid demux table
	 * (0xa1a8-0xa1b4).  Our driver wrote only the normal table (above) and never
	 * this one, so a deep_q CPU frame (ldpid 0x32) used the DPQ table's RESET
	 * demux_id and was steered off L3QM (build34 fixed the 0x32 entry).  build36:
	 * write the EXPLICIT stock live values - the DPQ table is NOT a mirror of the
	 * normal table (0xa1b4 stock=0xAAAA0000, not 0), so build34's mirror corrupted
	 * ldpid 8-15.  Now byte-matches stock: 0xa1a8=0 (0x32->L3QM), 0xa1b0=0x22AA0000
	 * (0x19->L2FE), 0xa1b4=0xAAAA0000. */
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63);
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47);
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31);
	writel(CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15_VAL,  ni_base(ni) + CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15);

	/* ★★ build40 THE FIX: the REAL aal_ni NIRX-L3FE-demux table (0xa1d4-0xa1f0) - the
	 * deep_q ROUTING table our driver never wrote (the 0xa1a8-b4 block above is a
	 * DIFFERENT table).  Without it a deep_q=1 CPU frame's dest-select sat at reset
	 * (non-L3QM) so it egressed ES7(physical) not ES8(L3QM): bm_tx climbed but 0xa9fc=0.
	 * Written VERBATIM from the STOCK LIVE golden (tier-1; the per-field decode was
	 * unreliable - live 0xa1e4=0x780C7864, not the RE's 0).  Normal @0xa1d4-e0, DPQ
	 * @0xa1e4-f0 (0xa1e4 covers our ldpid 0x32). */
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM0_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM0);
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM1_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM1);
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM2_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM2);
	writel(CA_NI_NIRX_L3FE_DEMUX_NORM3_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM3);
	writel(CA_NI_NIRX_L3FE_DPQ0_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ0);
	writel(CA_NI_NIRX_L3FE_DPQ1_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ1);
	writel(CA_NI_NIRX_L3FE_DPQ2_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ2);
	writel(CA_NI_NIRX_L3FE_DPQ3_VAL, ni_base(ni) + CA_NI_NIRX_L3FE_DPQ3);
	dev_info(ni->dev,
		 "l3fe-demux (REAL 0xa1d4-f0): norm3(0xa1e0)=0x%08x dpq0(0xa1e4)=0x%08x dpq3(0xa1f0)=0x%08x (want stock 0x64503C28/0x780C7864/0x00006050)\n",
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_NORM3),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DPQ0),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DPQ3));

	/* ★★ build36 THE FIX: enable the internal (CPU/QM/L3QM-facing) port MAC blocks.
	 * Stock enables ALL 7 port blocks; our driver init'd ONLY port 0, leaving the
	 * internal ports 5/6 (rx_en/tx_en=0) - the ones that accept the L2TM egress INTO
	 * L3QM.  So the frame egressed L2TM but no internal port accepted it -> ni2qm_rx
	 * (0xa9fc) never incremented.  Enable ports 1-6 to stock live values (port 0 is
	 * kept as-is: its RXMAC/TXMAC are programmed by the GPHY-link path). */
	cortina_ni_rx_enable_internal_ports(ni);

	/* ★ 0xa1c0 = 0x76543210 - the ONE NI_HV word our driver never wrote (all others
	 * 0xa180-0xa1bc match stock).  Prime L2TM-egress -> L3QM source-select suspect. */
	writel(CA_NI_NI_L3QMRX_PORT_ORDER_VAL,
	       ni_base(ni) + CA_NI_NI_L3QMRX_PORT_ORDER);

	/* second egress-enable layer: all NI egress ports (stock 0x610c) */
	writel(CA_NI_QM_L3TM_NI_PORT_ENA_ALL,
	       ni_base(ni) + CA_NI_QM_L3TM_NI_PORT_ENA);

	/* DLF trap: every FE lookup miss (BC/UUC/UL2MC/UL3MC) -> CPU0.  This is how
	 * an ingress frame reaches the CPU WITHOUT the bypass.  ★ Set it for EVERY
	 * GPHY LAN port (lspid 0..3), not just CA_NI_RX_PORT: the host cable can be
	 * on any port (this rig: port 3), and a port whose lspid has no DLF trap
	 * drops its lookup-miss frames instead of trapping them to the CPU.
	 *
	 * ★★ AND THE WAN SOURCE PORT NEEDS IT TOO (2026-08-04).  The loop above
	 * covered the LAN ports only, which was complete while the downstream data
	 * GEM still ran with FE_BYPASS: a bypassed frame is handed straight to
	 * CPU_0 and never does a lookup, so it cannot be a lookup MISS.  Under the
	 * HW-L3 downstream route (cortina_gpon.hw_l3_ds=1, the default) the DS data
	 * GEM instead enters the forwarding engines, and there a DOWNSTREAM
	 * BROADCAST -- every ARP request for this ONU's WAN address, and a
	 * broadcast DHCP OFFER/ACK -- resolves to nothing in the L2FE and becomes a
	 * lookup miss on the PON source port.  With no DFT_FWD entry for that lspid
	 * the miss is DROPPED rather than trapped, so the CPU never sees it.
	 *
	 * MEASURED, both firmwares, same instrument (ONU-test-case/wan_bcast_probe.py:
	 * 10 broadcast frames counted onto the OLT-facing NIC, then the ONU's own WAN
	 * netdev rx read): stock delivered them (nas0_0 rx +77), ours delivered ZERO.
	 * Downstream UNICAST was unaffected throughout -- it resolves via the static
	 * FDB entry for our WAN MAC -- which is why the fault hid: the ONU keeps its
	 * lease, its default route and full outbound connectivity while no far end
	 * can ever ARP it, so nothing can initiate traffic towards the ONU and a
	 * DHCP server that has aged its own ARP entry out cannot deliver a lease at
	 * all.  Only a reboot cleared it, because a booting ONU transmits and thereby
	 * re-teaches every far end its MAC.
	 *
	 * Kept as a SEPARATE loop rather than widening the bound above: the LAN ports
	 * are a contiguous range of PHYSICAL ports, the PON is a logical one, and a
	 * loop bound that silently walks past its table is precisely the class of bug
	 * this driver already paid for once.  A failure here is logged and does not
	 * abort the datapath bring-up -- a WAN without broadcast is degraded, a board
	 * whose steer_init returned early has no datapath at all.
	 */
	{
		unsigned int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++)
			for (type = 0; type < CA_NI_PLE_TYPE_COUNT; type++) {
				ret = cortina_ni_rx_ple_dft_fwd(ni, p, type);
				if (ret) {
					dev_err(ni->dev,
						"PLE dft-fwd (lspid %u type %d) timed out\n",
						p, type);
					return ret;
				}
			}

		for (type = 0; type < CA_NI_PLE_TYPE_COUNT; type++) {
			ret = cortina_ni_rx_ple_dft_fwd(ni, CA_NI_LSPID_PON, type);
			if (ret)
				dev_err(ni->dev,
					"PLE dft-fwd (PON lspid %u type %d) failed (%d): downstream broadcast (an ARP request for our WAN address, a broadcast DHCP OFFER) will be DROPPED instead of trapped to the CPU\n",
					CA_NI_LSPID_PON, type, ret);
		}
		ret = 0;
	}

	/* ★ build99 ORDER TEST: re-arm the L2TE->L3FE ready handshake (NIRX_MISC rdy-bits
	 * 0xa1bc=0x3e80, already stock-matching) HERE, at the END of steer_init - AFTER the
	 * whole L3FE pipeline (l3fe_glb_init, l3fe_axi_reo_init, mymac_trap/STG0, cls_init)
	 * is configured.  Our stock_routing arms it EARLY (in eq_init, before that config),
	 * but the vendor sets it in aal_ni_init_ni which runs AFTER aal_l3fe_init.  If the
	 * rdy-enable samples the L3FE pipeline-ready state at write time, arming it before
	 * L3FE config is complete would leave the LAN->L3FE handoff disabled -> l3fe_rx=0.
	 * Re-writing it last tests that order dependency (a candidate for the dynamic wall
	 * where every static value already matches stock). */
	writel(CA_NI_NI_NIRX_MISC_STOCK_VAL, ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG);
	dev_info(ni->dev,
		 "l3fe-handoff re-arm (post-L3FE-config): nirx_misc(0xa1bc)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG));

	return 0;
}

/* ------------------------------------------------------------------ */
/* L3QM empty-buffer pool + delivery-chain init (stock                 */
/* aal_l3qm_init_empty_buffer / init_voq / enable_rx, CPU port 0)      */
/* ------------------------------------------------------------------ */

/* commit the per-EQ CFG0-4 into the empty-buffer manager (stock
 * aal_l3qm_load_eq_config, 07f ko @0x4f270): unlock HDM write-protection,
 * pulse EQ_CFG_LOAD for all EQs, relock.  MANDATORY after programming an
 * EQ: until this runs the bid range does not latch, so pushed PAs pile up
 * in the shallow CPU-push stage (pool caps at ~4, ready bit stuck low).
 * writel carries the barrier stock spells as dmb-oshst between writes. */
static void cortina_ni_rx_eq_commit(struct cortina_ni *ni)
{
	/* ★ Vendor-EXACT commit-latch (aal_l3qm_load_eq_config @ca-ne.ko 0x4f270):
	 * 0x67fc=0x05102013, 0x6408=0x0000ffff (EQ_CFG_LOAD = LOAD ALL EQs), 0x6408=0,
	 * 0x67fc=0.  The prior 0x80006370 partial mask (spurious bit31 + a subset of
	 * EQ bits) did NOT latch our relocated CPU pools into the QM active pipeline,
	 * so EQ13's free-list stayed empty and the QM had no BID to admit into
	 * (qm_rx_cntr=0, no drop).  This register auto-clears, so a static diff can't
	 * see the wrong value.  writel carries the dmb-oshst the vendor spells out. */
	writel(CA_NI_QM_HDM_UNLOCK, ni_base(ni) + CA_NI_QM_HDM_WRITE_PROT);
	writel(CA_NI_QM_EQ_CFG_LOAD_ALL, ni_base(ni) + CA_NI_QM_EQ_CFG_LOAD);
	writel(0, ni_base(ni) + CA_NI_QM_EQ_CFG_LOAD);
	writel(0, ni_base(ni) + CA_NI_QM_HDM_WRITE_PROT);
}

/*
 * Program ONE QM AXI-attribute table entry via the indirect protocol
 * (aal_l3qm_set_epp_axi_attrib): write DATA0, trigger ACCESS = GO|write|ADDR,
 * then poll GO(bit31) clear with a BOUNDED cap.  This replaces the fatal blind
 * write of the stock post-completion readback (0x4000000F) that hung the AXI.
 * A timeout is logged and skipped - never an unbounded spin.
 */
static void cortina_ni_rx_set_axi_attrib(struct cortina_ni *ni, u32 entry,
					 u32 data0)
{
	u32 acc;
	int i;

	writel(data0, ni_base(ni) + CA_NI_QM_AXI_ATTR_DATA0);
	writel(CA_NI_QM_AXI_ATTR_GO | CA_NI_QM_AXI_ATTR_RBW |
	       FIELD_PREP(CA_NI_QM_AXI_ATTR_ADDR, entry),
	       ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
	for (i = 0; i < CA_NI_QM_AXI_ATTR_POLL_MAX; i++) {
		acc = readl(ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
		if (!(acc & CA_NI_QM_AXI_ATTR_GO))
			return;
		udelay(1);
	}
	dev_warn(ni->dev, "AXI-attr entry %u commit timeout (acc=0x%08x)\n",
		 entry, acc);
}

/* ★★ build55 DIAGNOSTIC: read ONE AXI-attr entry back via the indirect GET protocol
 * (ACCESS = GO|entry with RBW=0 => read; poll GO clear; then DATA0 = the stored attr).
 * Proves whether the set_axi_attrib writes actually LATCH (bit22 is invariant across 7
 * attr/address/axi_top builds - either the writes never latch, or bit22 isn't the attr). */
static u32 cortina_ni_rx_get_axi_attrib(struct cortina_ni *ni, u32 entry)
{
	u32 acc = 0;
	int i;

	writel(CA_NI_QM_AXI_ATTR_GO | FIELD_PREP(CA_NI_QM_AXI_ATTR_ADDR, entry),
	       ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
	for (i = 0; i < CA_NI_QM_AXI_ATTR_POLL_MAX; i++) {
		acc = readl(ni_base(ni) + CA_NI_QM_AXI_ATTR_ACCESS);
		if (!(acc & CA_NI_QM_AXI_ATTR_GO))
			break;
		udelay(1);
	}
	return readl(ni_base(ni) + CA_NI_QM_AXI_ATTR_DATA0);
}

/*
 * Initialise the QM AXI-attribute table BEFORE any frame can be admitted.  The
 * entries are invalid at power-on; the QM's buffer-DMA stalls on an unprogrammed
 * entry, so admission (qm_rx) never advances until every EQ pool + CPU-EPP port
 * we use has a valid attribute.  Our DDR cpu-push pools (EQ8/13/14) get the DDR
 * bufferable attribute; every CPU-EPP port gets the coherent/ACE attribute so
 * the descriptor ring is CPU-visible.  Unused EQs are left at the reset default.
 */
static void cortina_ni_rx_axi_attrib_init(struct cortina_ni *ni)
{
	static const u32 ddr_eqs[] = {
		CA_NI_RX_EQ_ID, CA_NI_RX_EQ_ID2, CA_NI_RX_EQ8_ID,
	};
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ddr_eqs); i++)
		cortina_ni_rx_set_axi_attrib(ni,
			CA_NI_QM_AXI_ATTR_EQ_BASE + ddr_eqs[i],
			CA_NI_QM_AXI_ATTR_DDR_POOL);

	/* ★★ build62: stock writes the coherent CPU_EPP attr 0x12008060 here (raw, as
	 * aal_l3qm_set_epp_axi_attrib writes it) and its coherent interconnect path works.
	 * ★★★ 2026-07-15 ROOT CAUSE of "wptr advances but the ring keeps its DEADBEEF":
	 * on OUR kernel the coherent (ACE) write does NOT route - QM_INT_SRC 0x611c bit30
	 * (axim_cpuepp_resp_error, rtl8277c_registers.h) latches per frame and the 8-byte
	 * descriptor never reaches DRAM, while the engine still advances wptr.  Our ring
	 * is WC/uncached anyway, so coherency is unnecessary: honor the (previously
	 * unwired) ring_noncoh param and write the descriptor ring with the SAME
	 * non-coherent DDR attr the frame-buffer pool DMA provably lands with. */
	for (i = 0; i < CA_NI_QM_CPU_PORT_COUNT; i++)
		cortina_ni_rx_set_axi_attrib(ni,
			CA_NI_QM_AXI_ATTR_CPU_BASE + i,
			ring_noncoh ? CA_NI_QM_AXI_ATTR_DDR_POOL :
				      CA_NI_QM_AXI_ATTR_CPU_EPP);
	dev_info(ni->dev, "axi-attr: CPU-EPP entries = %s\n",
		 ring_noncoh ? "non-coherent DDR 0x04000010 (ring_noncoh=1)" :
			       "stock 0x12008060 (coherent)");

	/* ★★ build55 DIAGNOSTIC: read back the 8 CPU-EPP entries + the 2 pool EQs to prove
	 * the writes latched (does idx48 read 0x00000010?), plus the EPP cmd/ctrl block
	 * (0x6a30-0x6a3c; ours 0x6a3c=0x04 cmd_mode bit2) - stock may set an EPP-writeback
	 * MASTER-RUN bit here that we miss (would make the writeback never fire). */
	for (i = 0; i < CA_NI_QM_CPU_PORT_COUNT; i++)
		dev_info(ni->dev, "axi-attr-readback: cpu_epp[%u] idx%u = 0x%08x\n",
			 i, CA_NI_QM_AXI_ATTR_CPU_BASE + i,
			 cortina_ni_rx_get_axi_attrib(ni,
				CA_NI_QM_AXI_ATTR_CPU_BASE + i));
	dev_info(ni->dev, "axi-attr-readback: eq13 idx%u=0x%08x  eq14 idx%u=0x%08x\n",
		 CA_NI_QM_AXI_ATTR_EQ_BASE + CA_NI_RX_EQ_ID,
		 cortina_ni_rx_get_axi_attrib(ni,
			CA_NI_QM_AXI_ATTR_EQ_BASE + CA_NI_RX_EQ_ID),
		 CA_NI_QM_AXI_ATTR_EQ_BASE + CA_NI_RX_EQ_ID2,
		 cortina_ni_rx_get_axi_attrib(ni,
			CA_NI_QM_AXI_ATTR_EQ_BASE + CA_NI_RX_EQ_ID2));
	dev_info(ni->dev, "epp-ctrl: 0x6a30=0x%08x 0x6a34=0x%08x 0x6a38=0x%08x 0x6a3c=0x%08x\n",
		 readl(ni_base(ni) + 0x6a30), readl(ni_base(ni) + 0x6a34),
		 readl(ni_base(ni) + 0x6a38), readl(ni_base(ni) + 0x6a3c));
}

/*
 * Program the CPU-RX DELIVERY CHAIN (stock aal_l3qm_init_empty_buffer_CPU):
 *   ingress -> FE DLF -> CPU dest 9 -> EQ profile 13 -> EQ13/EQ14 -> CPU-EPP
 *
 * The routing pieces (all NI window, tier-1 devmem):
 *   CPU dest 9 eq_cfg  (0x61a4)  profile_sel = 13   select EQ profile 13
 *   EQ profile 13      (0x615c)  = {eqp0=EQ13, eqp1=EQ14, rule=0}
 *   EQ13 pool cfg0..4  (0x634c..) = 0x80020081 (bit31 en), bid 0x5b0/383,
 *                                   0x0000ff00, 0x10, 0
 *   EQ14 pool cfg0..4  (0x6360..) = 0x09240001 (bit31 en), bid 0x72f/561,
 *                                   0x0000ff04, 0x10, 0
 *   dest-port 0 pkt_buf (0x6228)  = 0x18041804     head 0x40 (+ 384B tail)
 * The EQ pool-enable is CFG0 bit31, NOT bit0: an earlier remake pointed the CPU
 * profile at EQ8 configured with CFG0 bit31 clear + CFG1 total_buf 0, so the
 * pool was disabled + zero-capacity and the QM stalled WITHOUT admitting (no
 * drop counted) - EQM_PA_REQ=0, qm_rx_cntr=0, hw wptr=0.
 */
/* Program one self-populating empty-buffer pool from EXACT (tier-1 devmem)
 * values: CFG0 = pool-base PA | enable, the {bid_start,total_buf} CFG1 window
 * (built here so it stays in lockstep with the region constants), and CFG2
 * (cpu_eq=0, refill_en=0, buffer_size).  CFG3 = the shared CPU-pool AXI attrs,
 * CFG4 = 0.  Caller commits (EQ_CFG_LOAD) ONCE after every pool is programmed;
 * the commit is what makes the QM build its own free-list over the region. */
static void cortina_ni_rx_eq_cfg_pool(struct cortina_ni *ni, unsigned int eqid,
				      u32 cfg0, u32 bid_start, u32 total_buf,
				      u32 cfg2)
{
	writel(cfg0, ni_base(ni) + CA_NI_QM_CFG0_EQ(eqid));
	writel(FIELD_PREP(CA_NI_QM_CFG1_BID_START, bid_start) |
	       FIELD_PREP(CA_NI_QM_CFG1_TOTAL_BUF_NUM, total_buf),
	       ni_base(ni) + CA_NI_QM_CFG1_EQ(eqid));
	writel(cfg2, ni_base(ni) + CA_NI_QM_CFG2_EQ(eqid));
	/* AXI cache/snoop attrs, same for both CPU pools (tier-1 = 0x10) */
	writel(CA_NI_QM_CFG3_CPU_POOL_VAL, ni_base(ni) + CA_NI_QM_CFG3_EQ(eqid));
	writel(0, ni_base(ni) + CA_NI_QM_CFG4_EQ(eqid));
}

/*
 * Hand the software-owned CPU pools their buffers (cpu_pool_push only).
 *
 * MUST run AFTER the EQ commit AND after the RMU0 RX master is enabled: the
 * CPU push stage is only a few entries deep and does not drain until then, so
 * seeding any earlier stalls on the ready gate (see CA_NI_QM_CPU_PUSH_READY).
 *
 * The PAs are the same 2048-byte-strided addresses inside the reserved region
 * that the hardware-managed pool used, so cortina_ni_rx_frame()'s PA->VA math
 * and its bounds check need no change.  This is also the one difference from
 * the earlier cpu_eq=1 attempt, which pushed dynamically allocated skb
 * addresses: those land outside the NE's DDR window, so the RMU never DMA'd
 * into them (every frame arrived on a bid whose PA matched nothing and NAPI
 * read poison).  These PAs are the window the RMU is provably writing to right
 * now - drop_badpa is 0 and the live descriptors name this region.
 */
static int cortina_ni_rx_push_seed(struct cortina_ni *ni)
{
	/* nbufs MUST equal the total_buf each pool was configured with (the same
	 * constants cortina_ni_rx_eq_init passes to CFG1) - see the note at
	 * CA_NI_RX_PUSH_SEED_MIN. */
	static const struct {
		u32 eqid, base_off, bufsz, nbufs;
	} pools[] = {
		{ CA_NI_RX_EQ_ID,  0,
		  CA_NI_RX_CPU_POOL0_BUFSZ, CA_NI_RX_EQ_TOTAL_BUF },
		{ CA_NI_RX_EQ_ID2, CA_NI_RX_CPU_POOL0_BYTES,
		  CA_NI_RX_CPU_POOL1_BUFSZ, CA_NI_RX_EQ2_TOTAL_BUF },
	};
	unsigned int p, i, want = 0, done = 0;
	int ret = 0;

	for (p = 0; p < ARRAY_SIZE(pools); p++)
		want += pools[p].nbufs;

	for (p = 0; p < ARRAY_SIZE(pools) && !ret; p++) {
		for (i = 0; i < pools[p].nbufs; i++) {
			u32 pa = CA_NI_RX_CPU_POOL_PHYS + pools[p].base_off +
				 i * pools[p].bufsz;

			ret = cortina_ni_rx_push_buf(ni, pools[p].eqid, pa,
						     CA_NI_RX_PUSH_TIMEOUT_US);
			if (ret)
				break;
			done++;
		}
	}

	if (done < CA_NI_RX_PUSH_SEED_MIN) {
		dev_err(ni->dev,
			"RX: CPU pool seed FAILED - staged only %u of %u buffers (push stage never freed a slot).  RX would starve; boot with cortina_ni_rx.cpu_pool_push=0 to fall back to the hardware-managed pool.\n",
			done, want);
		return -EBUSY;
	}
	if (ret)
		dev_warn(ni->dev,
			 "RX: CPU pool seed SHORT - staged %u of %u buffers; the pools will report an inactive-bid shortfall\n",
			 done, want);
	else
		dev_info(ni->dev,
			 "RX: CPU pool seeded %u software-owned buffers (EQ%u=%u + EQ%u=%u, = CFG1.total_buf; expect inactive=0 ready=0)\n",
			 done, CA_NI_RX_EQ_ID, CA_NI_RX_EQ_TOTAL_BUF,
			 CA_NI_RX_EQ_ID2, CA_NI_RX_EQ2_TOTAL_BUF);
	return 0;
}

/* Log QM_PHY_PORT_STS with the handshake bits decoded (all should climb from
 * 0 toward the 0xa5ffffff default as the NI/TE/ES/AXI blocks come up). */
static void cortina_ni_rx_log_qm_sts(struct cortina_ni *ni, const char *stage)
{
	u32 s = readl(ni_base(ni) + CA_NI_QM_PHY_PORT_STS);

	dev_info(ni->dev,
		 "QM-hs[%s]: phy_sts=0x%08x nirx_port_rdy=0x%02lx te_es_ni_ok=0x%02lx nirx_qm_rdy=%u axi_wr=%u axi_rd=%u\n",
		 stage, s,
		 (unsigned long)FIELD_GET(CA_NI_QM_STS_NIRX_PORT_RDY, s),
		 (unsigned long)FIELD_GET(CA_NI_QM_STS_TE_ES_NI_OK, s),
		 !!(s & CA_NI_QM_STS_NIRX_QM_RDY),
		 !!(s & CA_NI_QM_STS_AXI_WR_RDY),
		 !!(s & CA_NI_QM_STS_AXI_RD_RDY));
}

/*
 * Match STOCK-LINUX's LIVE working NI-RX->CPU routing (STOCK_golden_rx_regs.txt,
 * read via devmem while CPU-RX was actively working over the LAN port).  These
 * are the exact NI/QM routing values the vendor aal_ne driver leaves.  Two
 * differ from U-Boot (stock Linux uses a different, real CPU-EPP path): autosync
 * = 0xF (not 0) and intern_pid = 0x3E80 (not 0x8080).  cpu-tag is NOT used for
 * LAN ports (cputag_cfg=0 on stock), so no cpu_tag_rx_en here.  The whole-word
 * writes overwrite whatever tx_hw_init left at 0xa1bc.
 */
static void cortina_ni_rx_stock_routing(struct cortina_ni *ni)
{
	cortina_ni_rx_log_qm_sts(ni, "before");

	writel(CA_NI_QM_AXIM2_STOCK_VAL, ni_base(ni) + CA_NI_QM_AXIM2_CONFIG);
	writel(CA_NI_NI_DEMUX1_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_L3QMRX_DEMUX_CFG1);
	writel(CA_NI_NI_DEMUX0_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_L3QMRX_DEMUX_CFG0);
	/* ★★ THE l3fe_rx=0 FIX (live-stock 2026-07-13): the REAL per-ldpid
	 * L2FE-vs-L3FE ingress fork is NIRX_L3FE_DEMUX_CFG1/0 at 0xa1c4/0xa1c8, not
	 * the 0xa188/0xa18c above (rate-meter regs).  Without these, host frames never
	 * enter the L3FE classifier so the CPU-trap can't fire.  Stock 0x00CBDA98/0x7000DA98. */
	writel(CA_NI_NIRX_L3FE_DEMUX_CFG1_REAL_VAL,
	       ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_CFG1_REAL);
	writel(CA_NI_NIRX_L3FE_DEMUX_CFG0_REAL_VAL,
	       ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX_CFG0_REAL);
	writel(CA_NI_NI_PORTORDER_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_PORTORDER_CFG);
	/* ★ INTERNAL_PORT_ID_CFG (0xa180) = 0x00A87F00: l3qmrx_demux_sel[7:0]=0 =
	 * present ALL ports' NI-RX to the L3QM = THE NI->QM LAN handoff.  We never
	 * wrote 0xa180 before (wrote 0x3e80 to 0xa1bc, which is NIRX_MISC). */
	writel(CA_NI_NI_INTERNAL_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG);
	/* NIRX_MISC_CFG (0xa1bc) = 0x3E80 (stock live) - the value our old code
	 * coincidentally wrote here thinking it was intern_pid; keep it. */
	writel(CA_NI_NI_NIRX_MISC_STOCK_VAL,
	       ni_base(ni) + CA_NI_NI_NIRX_MISC_CFG);
	writel(CA_NI_NI_AUTOSYNC_STOCK_VAL,
	       ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC);

	/* ★★ build35: OR NI_HV_GLB_STATIC_CFG (0xa01c) bits[17:16] - the ONLY
	 * 0xa000-0xa1fc reg that differed from stock (stock 0x1A07002F vs our boot-ROM
	 * 0x1A04002F) after every routing/demux reg matched.  Prime NI->L3QM
	 * ingress-enable candidate (frame egresses L2TM but 0xa9fc/L3QM never accepts).
	 * RMW (OR only): the port_to_cpu[3:0] field is PRESERVED, so this is NOT the
	 * old "write 0" that clobbered port_to_cpu and froze the L2FE path - it only
	 * adds the two missing high bits. */
	writel(readl(ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG) | CA_NI_NI_GLB_STATIC_L3QM_EN,
	       ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG);

	dev_info(ni->dev,
		 "stock-routing: intern_pid(a1bc)=0x%08x portorder(a1c0)=0x%08x autosync(a010)=0x%08x glb_static(a01c)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG),
		 readl(ni_base(ni) + CA_NI_NI_PORTORDER_CFG),
		 readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC),
		 readl(ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG));

	cortina_ni_rx_log_qm_sts(ni, "stock-routing");
}

/*
 * ★ Bisect-from-working: replicate stock's FULL QM config (tier-1 devmem golden)
 * for the config divergences our minimal driver leaves at 0 - the RMU-RX=0
 * admission wall.  EQ8/EQ12 = the shared empty-buffer pools the RMU allocates
 * FROM (dest-9 -> profile-13 -> EQ13 queue, buffers from EQ8); DWRR weights
 * (0x656c-0x6660 = 0x01010101, aal_l3qm_config_DWRR) give the CPU VOQ scheduler
 * credit; the per-queue CPU-EPP-FIFO profile-sel (0x66d0-0x67c8) + AXI-attr
 * access + misc QM cfg complete the delivery.  Written before EQ_CFG_LOAD so the
 * EQ-pool words latch.  (Minimize AFTER CPU-RX is proven working.) */
static const struct { u16 off; u32 val; } cortina_ni_stock_qm_cfg[] = {
	/* ★ EQ8/EQ12 pool-enable INTENTIONALLY OMITTED (they HUNG the CPU): stock's
	 * C2 (EQ8=0x66ec / EQ12=0xff02) has refill_en=1 (FBM auto-refill from EQ6),
	 * but we have no FBM/EQ6, so enabling them + EQ_CFG_LOAD triggers a hanging
	 * AXI buffer-alloc that wedges the CPU.  This table now holds ONLY the SAFE
	 * non-alloc config (DWRR VOQ weights / per-queue CPU-EPP-FIFO / misc).  EQ8 is
	 * instead brought up in eq_init as a CPU-PUSH pool with C2=CA_NI_QM_EQ8_CFG2
	 * (cpu_eq=1, refill_en=0, non-overlapping bid window) + CPU-push-seeded so
	 * pa_req>0 - that is the RMU's empty-buffer source for admission.  NB: C2 must
	 * be 0x0D (cpu_eq=1), NOT 0xff00 - 0xff00 has cpu_eq=0 so the EQM never issues
	 * EQM_PA_REQ and every pushed buffer lands nowhere (pa_req stuck at 0). */
	{ 0x656c, 0x01010101 }, { 0x6570, 0x01010101 }, { 0x6574, 0x01010101 },
	{ 0x6578, 0x01010101 }, { 0x657c, 0x01010101 }, { 0x6580, 0x01010101 },
	{ 0x6584, 0x01010101 }, { 0x6588, 0x01010101 }, { 0x658c, 0x01010101 },
	{ 0x6590, 0x01010101 }, { 0x6594, 0x01010101 }, { 0x6598, 0x01010101 },
	{ 0x659c, 0x01010101 }, { 0x65a0, 0x01010101 }, { 0x65a4, 0x01010101 },
	{ 0x65a8, 0x01010101 }, { 0x65ac, 0x01010101 }, { 0x65b0, 0x01010101 },
	{ 0x65b4, 0x01010101 }, { 0x65b8, 0x01010101 }, { 0x65bc, 0x01010101 },
	{ 0x65c0, 0x01010101 }, { 0x65c4, 0x01010101 }, { 0x65c8, 0x01010101 },
	{ 0x65cc, 0x01010101 }, { 0x65d0, 0x01010101 }, { 0x65d4, 0x01010101 },
	{ 0x65d8, 0x01010101 }, { 0x65dc, 0x01010101 }, { 0x65e0, 0x01010101 },
	{ 0x65e4, 0x01010101 }, { 0x65e8, 0x01010101 }, { 0x65ec, 0x01010101 },
	{ 0x65f0, 0x01010101 }, { 0x65f4, 0x01010101 }, { 0x65f8, 0x01010101 },
	{ 0x65fc, 0x01010101 }, { 0x6600, 0x01010101 }, { 0x6604, 0x01010101 },
	{ 0x6608, 0x01010101 }, { 0x660c, 0x01010101 }, { 0x6610, 0x01010101 },
	{ 0x6614, 0x01010101 }, { 0x6618, 0x01010101 }, { 0x661c, 0x01010101 },
	{ 0x6620, 0x01010101 }, { 0x6624, 0x01010101 }, { 0x6628, 0x01010101 },
	{ 0x662c, 0x01010101 }, { 0x6630, 0x01010101 }, { 0x6634, 0x01010101 },
	{ 0x6638, 0x01010101 }, { 0x663c, 0x01010101 }, { 0x6640, 0x01010101 },
	{ 0x6644, 0x01010101 }, { 0x6648, 0x01010101 }, { 0x664c, 0x01010101 },
	{ 0x6650, 0x01010101 }, { 0x6654, 0x01010101 }, { 0x6658, 0x01010101 },
	{ 0x665c, 0x01010101 }, { 0x6660, 0x01010101 },
	/* ★ build17 THE DRAIN-DELIVERY block (0x6664-0x66cc) stock programs but our
	 * table SKIPPED (it jumped 0x6660 -> 0x66d0), leaving it 0.  Stock ground-truth
	 * (stock_qm_epp_rmu.txt + live devmem): with these 0, the QM never pushes a
	 * drained deep-queue frame into the CPU-EPP ring, so wptr 0x7000 stays 0 - our
	 * exact symptom (routing/TM fine, EPP dead).  0x66a4-0x66c0 =
	 * CA_NI_QM_CPU_EPP_FIFO_PROF(n): the per-FIFO descriptor telling the QM HOW to
	 * deliver a drained frame into the CPU-EPP FIFO/ring (distinct from the DWRR
	 * credit 0x65fc-0x6660 and the FIFO_CFG 0x66d0+ we already set).  All values
	 * tier-1 from the stock golden.  (0x6680-0x66a0 read 0 on stock too - skip.) */
	{ 0x6664, 0x11111111 }, { 0x6668, 0x11111111 }, { 0x666c, 0x11111111 },
	{ 0x6670, 0x11111111 }, { 0x6674, 0x11111111 }, { 0x6678, 0x11111111 },
	{ 0x667c, 0x000C0114 },
	{ 0x66a4, 0xE0008001 }, { 0x66a8, 0xE0008001 }, { 0x66ac, 0xE0008001 },
	{ 0x66b0, 0xE0008001 }, { 0x66b4, 0xE00040F1 }, { 0x66b8, 0x20006801 },
	{ 0x66bc, 0x2000C801 }, { 0x66c0, 0xE0080001 }, { 0x66cc, 0x00000004 },
	{ 0x66d0, 0x00000004 }, { 0x66d4, 0x00000004 }, { 0x66d8, 0x00000004 },
	{ 0x66dc, 0x00000004 }, { 0x66e0, 0x00000004 }, { 0x66e4, 0x00000004 },
	{ 0x66e8, 0x00000004 }, { 0x66ec, 0x00000004 }, { 0x66f0, 0x00000004 },
	{ 0x66f4, 0x00000004 }, { 0x66f8, 0x00000004 }, { 0x66fc, 0x00000004 },
	{ 0x6700, 0x00000004 }, { 0x6704, 0x00000004 }, { 0x6708, 0x00000004 },
	{ 0x670c, 0x00000006 }, { 0x6710, 0x00000005 }, { 0x6714, 0x00000005 },
	{ 0x6718, 0x00000005 }, { 0x671c, 0x00000006 }, { 0x6720, 0x00000005 },
	{ 0x6724, 0x00000005 }, { 0x6728, 0x00000005 }, { 0x672c, 0x00000007 },
	{ 0x6730, 0x00000007 }, { 0x6734, 0x00000007 }, { 0x6738, 0x00000007 },
	{ 0x673c, 0x00000007 }, { 0x6740, 0x00000007 }, { 0x6744, 0x00000007 },
	{ 0x6748, 0x00000007 }, { 0x674c, 0x00000007 }, { 0x6750, 0x00000007 },
	{ 0x6754, 0x00000007 }, { 0x6758, 0x00000007 }, { 0x675c, 0x00000007 },
	{ 0x6760, 0x00000007 }, { 0x6764, 0x00000007 }, { 0x6768, 0x00000007 },
	{ 0x676c, 0x00000007 }, { 0x6770, 0x00000007 }, { 0x6774, 0x00000007 },
	{ 0x6778, 0x00000007 }, { 0x677c, 0x00000007 }, { 0x6780, 0x00000007 },
	{ 0x6784, 0x00000007 }, { 0x6788, 0x00000007 }, { 0x678c, 0x00000007 },
	{ 0x6790, 0x00000007 }, { 0x6794, 0x00000007 }, { 0x6798, 0x00000007 },
	{ 0x679c, 0x00000007 }, { 0x67a0, 0x00000007 }, { 0x67a4, 0x00000007 },
	{ 0x67a8, 0x00000007 }, { 0x67ac, 0x00000007 }, { 0x67b0, 0x00000007 },
	{ 0x67b4, 0x00000007 }, { 0x67b8, 0x00000007 }, { 0x67bc, 0x00000007 },
	{ 0x67c0, 0x00000007 }, { 0x67c4, 0x00000007 }, { 0x67c8, 0x00000007 },
	/* ★ 0x67cc CONFIRMED TOXIC + REQUIRED: per-queue CPU-EPP-FIFO indirect COMMIT
	 * (bit30) - hangs the CPU (AXI back-pressure) even with EQ13/14 populated.
	 * Needed to bind queue->pool for RMU admission but can't be written cold.
	 * EXCLUDED pending the safe precondition/sequence (Fable RE). */
	/* SAFE QM config (added back): int-en / maps / misc - plain config, needed for
	 * admission/delivery, no HW-trigger. */
	{ 0x69b4, 0x80080000 }, { 0x69bc, 0x06061616 },
	/* ★ 0x69bc corrected 0x06006666 -> stock 0x06061616 (fixes OUR own past
	 * divergence - this word IS in our table).  NOTE (RE a4ee42, ca-ne.ko): the
	 * rest of the RMU 0x6900-0x69fc block (0x6934/0x6988/0x698c/0x6994...) is NOT
	 * driver-programmed on stock - HW power-on defaults, and 0x6988 is the init-done
	 * STATUS reg ca_ni_init_l3qm POLLS (bit30).  So we deliberately do NOT blind-write
	 * them (a status/trigger write risks a regression that would confound the
	 * drain-map fix); if ours truly differs there, hunt the clobber, don't poke. */
	{ 0x69f8, 0x000000FF }, { 0x6a00, 0x0000FF00 },
	{ 0x6110, 0x0000FFFF }, { 0x6118, 0x00000100 }, { 0x611c, 0x10000000 },
	{ 0x6120, 0xE6D54F85 },
	/* ★ EXCLUDED (hang triggers / board-specific DMA state): 0x67cc=0x4000000F
	 * (bit30 per-queue indirect COMMIT) + the 0x69c4-0x69e0 block (0x0863A000 etc.
	 * = STOCK's CPU-EPP ring/DMA addresses; ours live at 0x0bc48000 - replicating
	 * stock's would point the QM at wrong memory = hang).  Do NOT replicate these. */
};

/* ★★ build45 verify: the EQM buffer-availability + RMU0-admit ledger.  The cpu_eq=1 fix
 * should make pa_req(EQ13 0x63bc / EQ14 0x63c0) climb >0 (buffers now register with the
 * EQM) so RMU0 admits: 0x6940 (NO_BUF_DROP) STOPS climbing, 0x6900 (RMU0_RX) climbs,
 * epp_wptr(0x7000) advances.  (0x69xx read-back dropped: RE-confirmed READ-ONLY RMU0
 * status, not config.) */
static void cortina_ni_rx_eqm_readback(struct cortina_ni *ni, const char *label)
{
	dev_info(ni->dev,
		 "eqm-readback(%s): pa_req eq13(0x63bc)=0x%08x eq14(0x63c0)=0x%08x | eq_prof5(0x613c)=0x%08x | fifo_prof4(0x66b4)=0x%08x (want 0xE00040F1) | int_en0(0x6110)=0x%08x en1(0x6114)=0x%08x refill_en(0x611c)=0x%08x REAL_int_src(0x6120)=0x%08x [b22=eqm_cfg_err b21=buf_size b20=cpuepp_fifo] | no_buf(0x6940)=%u rmu_rx(0x6900)=%u tx_cntr(0x690c)=%u epp_wptr(0x7000)=0x%06x\n",
		 label,
		 readl(ni_base(ni) + 0x63bc),
		 readl(ni_base(ni) + 0x63c0),
		 readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE_SEL)),
		 readl(ni_base(ni) + CA_NI_QM_CPU_EPP_FIFO_PROF(CA_NI_RX_PROFILE_ID)),
		 readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN0),
		 readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN1),
		 readl(ni_base(ni) + 0x611c),
		 readl(ni_base(ni) + 0x6120),
		 readl(ni_base(ni) + CA_NI_QM_RMU_NO_BUF_DROP),
		 readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		 readl(ni_base(ni) + CA_NI_QM_TX_CNTR),
		 cortina_ni_rx_wptr(ni));
}

static void __maybe_unused cortina_ni_rx_match_stock_qm(struct cortina_ni *ni)
{
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(cortina_ni_stock_qm_cfg); k++)
		writel(cortina_ni_stock_qm_cfg[k].val,
		       ni_base(ni) + cortina_ni_stock_qm_cfg[k].off);
	dev_info(ni->dev, "match-stock-qm: wrote %zu QM cfg regs (EQ8/12 pools, DWRR, per-queue, misc)\n",
		 ARRAY_SIZE(cortina_ni_stock_qm_cfg));
}

static int cortina_ni_rx_eq_init(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;
	u32 cfg0_p0, cfg0_p1, cfg2_p0, cfg2_p1;
	u32 sts;
	int i, ret;

	/* (0) NI-RX->QM routing = U-Boot's live working values (bisect-from-
	 * working): the real fix for "frames never reach the QM". */
	cortina_ni_rx_stock_routing(ni);

	/* (1) wait for the QM block's own init to finish before touching it
	 * (stock aal_l3qm_check_init_done); bounded + non-fatal.  We no longer
	 * GATE on qm_init_done (it is a phantom 0 even in the 0xa5ffffff default)
	 * - qm_up is kept as a logged diagnostic only. */
	ret = readl_poll_timeout(ni_base(ni) + CA_NI_QM_PHY_PORT_STS, sts,
				 sts & CA_NI_QM_INIT_DONE, 10,
				 CA_NI_QM_INIT_DONE_TIMEOUT_US);
	rx->qm_up = !ret;
	if (ret)
		dev_warn(ni->dev, "RX: QM init-done not seen (sts=0x%x) - is the TQM reset firing?\n",
			 sts);
	else
		dev_info(ni->dev, "RX: QM init-done OK (sts=0x%x)\n", sts);

	/* (2) master L3QM RX off while (re)programming (stock order) */
	ni_rmw(ni, CA_NI_QM_RMU0_CTRL, CA_NI_QM_RMU0_RX_EN, 0);

	/* spy: report every EQ already holding a bid range (overlap with ours
	 * would corrupt the pool accounting) */
	for (i = 0; i < CA_NI_QM_EQ_COUNT; i++) {
		u32 c1 = readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(i));

		if (FIELD_GET(CA_NI_QM_CFG1_TOTAL_BUF_NUM, c1))
			dev_info(ni->dev,
				 "RX: EQ%d pre-set: bid_start=%lu num=%lu\n", i,
				 FIELD_GET(CA_NI_QM_CFG1_BID_START, c1),
				 FIELD_GET(CA_NI_QM_CFG1_TOTAL_BUF_NUM, c1));
	}

	/* (3) CPU empty-buffer pools EQ5(pool0)/EQ6(pool1) = cpu_eq=0 SELF-POPULATING,
	 * exactly the stock model (STOCK_cpurx_dynamic_golden.txt: stock EQ12/13/14, e.g.
	 * EQ14 CFG0=0x09240001 CFG2=0xFF04).  CFG0.phy_addr_start[31:7] points at our
	 * reserved DRAM region (EQ5 at offset 0, EQ6 right after EQ5's sub-region); the
	 * QM maps bid n -> base + n*2048 itself and recycles the bid on the CPU-EPP
	 * read-pointer advance - no software push, ever.  The EQ_CFG_LOAD pulse in
	 * cortina_ni_rx_eq_commit (below, before RMU0 RX enable) is what makes the QM
	 * self-populate the latched range.  eq_cfg_pool also writes CFG3=0x10, CFG4=0
	 * (32-bit pool PA, axi_top_bit 0).
	 * Target values: EQ5 CFG0@0x62ac=0x09400001 CFG1=0x02001200 CFG2=0x0000ff04;
	 *                EQ6 CFG0@0x62c0=0x09500001 CFG1=0x020017dc CFG2=0x0000ff04. */
	/*
	 * ★★★ 2026-07-27 buffer-ownership fix (cpu_pool_push, currently OFF).
	 * cpu_eq=1 makes each buffer software-owned: the QM hands it out once
	 * and cannot take it back until the driver re-pushes it, which is the
	 * interval the copy-break needs and the hardware-managed pool never
	 * provided.  ONLY CFG2 differs between the two models.
	 *
	 * ★ CFG0.phy_addr_start is now kept at the real region base in BOTH
	 * models.  A first cut zeroed it under cpu_pool_push, on the reasoning
	 * that a non-zero base would leave the QM self-populating the very bids
	 * we push and hand the same buffer out twice.  That reasoning rested on
	 * an UNTIERED comment ("0 for SW-push pools" at CA_NI_QM_CFG0_PHY_ADDR_
	 * START) plus an inference of mine - neither is a measurement, and if
	 * the QM also uses that base to map or validate a PUSHED PA then zeroing
	 * it breaks buffer delivery for some consumers and not others.  That is
	 * the shape of the open DS-punt regression, so restoring the base is the
	 * discriminator: if the WAN downstream punt returns, the zeroing was the
	 * cause; if stale_buf starts climbing again, the double-population risk
	 * was real and the zeroing was right for the wrong reason.
	 */
	if (cpu_pool_push) {
		/* ★ A software-owned pool carries NO base: the QM maps bid n only
		 * for a pool it populates itself, and a pushed buffer arrives with
		 * its own full address.  Leaving a base here would have the QM
		 * self-populate the very bids we push.  This is what stock does -
		 * its CPU pools write a literal 1 (eq_en alone, no address field)
		 * while every hardware-managed pool writes a real base, and live
		 * stock reads cfg0=0x00000001 on all three of its cpu_eq=1 pools.
		 * An earlier revision here kept the real base on an untiered
		 * comment plus an inference; the binary settles it. */
		/* ★ REVERTED 2026-07-28: keep the REAL base here too.  Stock does
		 * write a bare eq_en (cfg0=0x00000001) on its cpu_eq=1 pools, and
		 * that is well evidenced - but shipping it here cost the 2.4 GHz
		 * radio: the case that proves the WiFi LED pad is routed went
		 * PASS -> FAIL across exactly this change, phy1 came up with
		 * txpower 0.00 dBm and never beaconed, and only the 5 GHz AP
		 * reached the air.  A zero base points the QM's mapping at
		 * physical address 0, so anything that still self-populates writes
		 * over low memory.  We have no defect that the alignment fixes, so
		 * it is not worth a regression: the real base is what the working
		 * configuration was verified with (fragments 150/150, WAN up,
		 * both radios on air). */
		cfg0_p0 = (CA_NI_RX_CPU_POOL_PHYS &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg0_p1 = ((CA_NI_RX_CPU_POOL_PHYS + CA_NI_RX_CPU_POOL0_BYTES) &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg2_p0 = CA_NI_QM_EQ13_CFG2 | CA_NI_QM_CFG2_CPU_EQ;
		cfg2_p1 = CA_NI_QM_EQ14_CFG2 | CA_NI_QM_CFG2_CPU_EQ;
	} else {
		cfg0_p0 = (CA_NI_RX_CPU_POOL_PHYS &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg0_p1 = ((CA_NI_RX_CPU_POOL_PHYS + CA_NI_RX_CPU_POOL0_BYTES) &
			   CA_NI_QM_CFG0_PHY_ADDR_START) | CA_NI_QM_CFG0_EQ_EN;
		cfg2_p0 = CA_NI_QM_EQ13_CFG2;
		cfg2_p1 = CA_NI_QM_EQ14_CFG2;
	}
	cortina_ni_rx_eq_cfg_pool(ni, CA_NI_RX_EQ_ID, cfg0_p0,
				  CA_NI_RX_EQ_BID_START, CA_NI_RX_EQ_TOTAL_BUF,
				  cfg2_p0);
	cortina_ni_rx_eq_cfg_pool(ni, CA_NI_RX_EQ_ID2, cfg0_p1,
				  CA_NI_RX_EQ2_BID_START, CA_NI_RX_EQ2_TOTAL_BUF,
				  cfg2_p1);
	dev_info(ni->dev,
		 "RX: CPU pools EQ%u/EQ%u %s (cfg0=0x%08x/0x%08x cfg2=0x%08x/0x%08x)\n",
		 CA_NI_RX_EQ_ID, CA_NI_RX_EQ_ID2,
		 cpu_pool_push ? "SOFTWARE-OWNED (cpu_eq=1, push recycle)" :
				 "hardware-managed (cpu_eq=0, self-populating)",
		 cfg0_p0, cfg0_p1, cfg2_p0, cfg2_p1);

	/*
	 * ★★★ (3a) The DEEP-QUEUE pool, EQ12, hardware-managed - only needed when
	 * the CPU pools are software-owned.  See the CA_NI_RX_DQ_POOL_OFF block for
	 * the full account; in one line: the deep-queue enqueue on this chip cannot
	 * consume a CPU-pushed buffer, and the GPON downstream punt is deep-queued
	 * (17 of 40 rmu0_rx_hdr samples on the working image, no LAN traffic in
	 * flight), so the deep queue needs a self-populating pool of its own once
	 * EQ5/EQ6 stop being one.
	 *
	 * cpu_eq=0 + a CFG0.phy_addr_start inside the same proven DDR reserve means
	 * the QM builds this free list itself at the EQ_CFG_LOAD commit below - no
	 * push, no seed, and nothing for the NAPI recycle to hand back (which is
	 * why cortina_ni_rx_frame() returns 0 for a PA in this region).
	 *
	 * With cpu_pool_push off, none of this is written and the routing below
	 * stays exactly as it was, so gate-off behaviour is byte-identical.
	 */
	if (cpu_pool_push) {
		cortina_ni_rx_eq_cfg_pool(ni, CA_NI_RX_EQ12_ID,
					  (CA_NI_RX_DQ_POOL_PHYS &
					   CA_NI_QM_CFG0_PHY_ADDR_START) |
					  CA_NI_QM_CFG0_EQ_EN,
					  CA_NI_RX_EQ12_BID_START,
					  CA_NI_RX_EQ12_TOTAL_BUF,
					  CA_NI_RX_DQ_CFG2);
		/* BOTH halves of the profile point at EQ12, deliberately: this
		 * consumer has no software-owned overflow reserve, and pointing
		 * eqp1 at EQ6 (as the pre-existing CA_NI_RX_EQ12_PROFILE_VAL
		 * does) would hand the deep queue a pushed buffer - the exact
		 * thing it cannot use.  Safe to write here: profile 12 is
		 * outside the 0..7 range the loop further down fills, and
		 * nothing else in this driver writes EQ_PROFILE(12). */
		writel(CA_NI_RX_DQ_PROFILE_VAL,
		       ni_base(ni) +
		       CA_NI_QM_EQ_PROFILE(CA_NI_RX_DQ_PROFILE_SEL));
		dev_info(ni->dev,
			 "RX: deep-queue pool EQ%u hardware-managed @0x%08x (%u bufs, bid 0x%x, cfg2=0x%08x) -> EQ_PROFILE(%u)=0x%02x\n",
			 CA_NI_RX_EQ12_ID, (u32)CA_NI_RX_DQ_POOL_PHYS,
			 CA_NI_RX_EQ12_TOTAL_BUF,
			 (u32)CA_NI_RX_EQ12_BID_START,
			 (u32)CA_NI_RX_DQ_CFG2, CA_NI_RX_DQ_PROFILE_SEL,
			 (u32)CA_NI_RX_DQ_PROFILE_VAL);
	}

	/* (3b) Steer the CPU-bound frame to our pool via EQ_PROFILE(13) = {eqp0=EQ13,
	 * eqp1=EQ14} = 0xED (stock 0x615c=0xED).  DEST_PORT_EQ_CFG.prof_sel is a 4-bit
	 * field (GENMASK(3,0), 0-15) that indexes EQ_PROFILE DIRECTLY - stock destp8=0x0C
	 * -> EQ_PROFILE(12) (NOT a 3-bit field), so value 0x0D -> EQ_PROFILE(13).
	 *
	 * ★ build18 FIX (the wptr=0 bug): our live routing sends the frame into the
	 * DEEP-QUEUE (HW-confirmed bm-hdr deep_q=1, PDPID 0x08 -> QM dest-port 8), but
	 * this block previously programmed ONLY the CPU-direct slot (dest-port 15) on a
	 * stale "CPU-RX routes to PDPID 0x09" assumption.  So dest-port 8's prof_sel
	 * stayed 0 (unconfigured) -> the deep_q frame found NO EQ profile -> no pool ->
	 * the QM never drained it to the CPU-EPP ring (wptr 0x7000=0).  Stock steers
	 * dest-port 8 to EQ12/profile-12 (FBM refill_en=1 - would hang our FBM-less
	 * driver), so we point the whole deep-queue dest-port range 8..15 at OUR
	 * EQ_PROFILE(13)={EQ13,EQ14} instead. */
	writel(CA_NI_RX_EQ_PROFILE_VAL,
	       ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE));
	/* ★★ build47 THE FIX (final root cause): DEST_PORT_EQ_CFG[8]=0x0d, but profile_sel
	 * is a 3-BIT field, so 0x0d & 0x7 = profile 5 -> EQ_PROFILE(5), NOT 13.  We wrote
	 * ONLY EQ_PROFILE(13); profiles 0..7 stayed 0 = {eqp0=EQ0, eqp1=EQ0} (empty) -> RMU0
	 * size-selects an EMPTY EQ -> NO_BUFFER (frame reached RMU0, pool auto-filled, but the
	 * SELECTED profile pointed at EQ0).  Program ALL 8 3-bit-reachable profiles (0..7) =
	 * 0xED={EQ13,EQ14} so whichever profile the CPU frame picks maps to our live pools.
	 * (Documented at CA_NI_RX_EQ_PROFILE_SEL=5 but the write was missing.)
	 *
	 * ★ CORRECTION 2026-07-28: the "3-BIT field, 0x0d & 0x7 = profile 5" premise
	 * above is WRONG, and it contradicts the (correct) 4-bit note a few lines up.
	 * CA_NI_QM_DEST_PORT_PROF_SEL is GENMASK(3, 0) - four bits at bit 0, a DIRECT
	 * index into EQ_PROFILE(0..15), no masking.  So destp8=0x0D selects profile 13,
	 * not 5.  Live readback agrees: prof13(0x615c)=0x65={EQ5,EQ6} with
	 * destp8=destp15=0x0D; and stock's destp8=0x0C with prof12=0xEC={EQ12,EQ14},
	 * its documented deep-queue pair, only lines up under direct indexing.
	 * THE WRITE BELOW IS STILL RIGHT AND STILL NEEDED - dest port 0 selects
	 * profile 2, which lives in this 0..7 range and would otherwise be empty - so
	 * build47's fix worked, but for a different reason than it recorded.  Kept
	 * because a filled 0..7 is harmless and covers the dest ports 1..7 we never
	 * program; corrected because the next reader should not re-derive the wrong
	 * field width from it. */
	for (i = 0; i < 8; i++)
		writel(CA_NI_RX_EQ_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_EQ_PROFILE, i,
						CA_NI_QM_EQ_PROFILE_COUNT));
	for (i = 0; i < CA_NI_QM_EQ_PROFILE_GLOBAL_COUNT; i++)
		writel(CA_NI_RX_CPU_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_EQ_PROFILE_GLOBAL, i,
						CA_NI_QM_EQ_PROFILE_GLOBAL_COUNT));
	/* build77: cpu_port 0 (CPU_0) -> profile_sel=2 -> EQ_PROFILE[2]={EQ5,EQ6} (the
	 * stock CPU pools, now configured+seeded).  Our 0..7 loop above set EQ_PROFILE[2]
	 * to {EQ5,EQ6}; point destport0 at profile 2 (stock value; was 0xf8=profile 8). */
	writel((CA_NI_NI_DESTPORT0_STOCK_VAL & ~CA_NI_QM_DEST_PORT_PROF_SEL) |
	       FIELD_PREP(CA_NI_QM_DEST_PORT_PROF_SEL, 2),
	       ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(0));
	/*
	 * The deep-queue dest-ports (8..15, incl. the CPU slot 15).
	 *
	 * Historically these carried CA_NI_RX_CPU_PROFILE_VAL (0x0D).  prof_sel is
	 * a 4-bit DIRECT index (GENMASK(3,0), no masking), so 0x0D selects
	 * EQ_PROFILE(13) - which holds {EQ5, EQ6}, the same pools as the direct CPU
	 * punt (live: prof13(0x615c)=0x65 with destp8=destp15=0x0D).
	 *
	 * ★ Under cpu_pool_push that sharing is fatal: EQ5/EQ6 are then
	 * software-owned and the deep-queue enqueue cannot consume a pushed buffer,
	 * so point this range at the hardware-managed deep-queue profile instead.
	 * Stock likewise keeps the two consumers apart (prof12 = {EQ12, EQ14},
	 * prof13 = {EQ13, EQ14}); the collapse onto one profile was ours.
	 * FIELD_PREP rather than a bare value so the field placement is explicit
	 * instead of relying on prof_sel sitting at bit 0.
	 * Gate off => the historic value, byte-identical.
	 */
	for (i = CA_NI_RX_DEEPQ_DEST_PORT_LO; i <= CA_NI_RX_DEEPQ_DEST_PORT_HI; i++)
		writel(cpu_pool_push ?
		       FIELD_PREP(CA_NI_QM_DEST_PORT_PROF_SEL,
				  CA_NI_RX_DQ_PROFILE_SEL) :
		       CA_NI_RX_CPU_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_DEST_PORT_EQ_CFG, i,
						CA_NI_QM_DEST_PORT_ENTRIES));
	/* ★ 2026-07-23 (Fable RE): stock configures DEST_PORT_EQ_CFG for the whole
	 * CPU/PON dest-port range up to 0x2f; ours left 16..0x2f at profile 0 -> EQ0
	 * (empty).  A DS-into-L3FE frame (hw_l3_fwd + cg_hw_l3_ds) that resolves to a
	 * CPU_0 dest port >= 16 then hit NO_BUFFER and head-of-line-blocked the shared
	 * RMU -> the LAN CPU-delivery went 100% loss the instant the DS route installed.
	 * Point the whole range at our configured CPU pool (EQ13/14) so nothing wedges. */
	/* ★★ This range carries the DOWNSTREAM DATA punt, and leaving it on the
	 * software-owned pools is what killed the WAN under cpu_pool_push=1: gpon0
	 * RX sat at exactly 0 packets from boot while the LAN punt and DS OMCI were
	 * both fine.  The per-GEM PDC map stamps data GEMs (8..255) with
	 * ldpid 0x18 = L3_WAN = 24, which lands here, while OMCI GEMs (0..7) are
	 * stamped CPU_0 with fe_bypass=1 and go to dest port 0 alongside the LAN
	 * punt - which is exactly why control frames kept arriving while data
	 * frames never did.  A forwarding-engine consumer cannot draw from a
	 * cpu_eq=1 pool, so under software ownership this range must point at the
	 * hardware-managed pool instead.  Stock likewise gives 16..47 a different
	 * pool pair from its CPU ports. */
	for (i = CA_NI_RX_DEEPQ_DEST_PORT_HI + 1; i <= CA_NI_QM_DEST_PORT_LAST; i++)
		writel(cpu_pool_push ?
		       FIELD_PREP(CA_NI_QM_DEST_PORT_PROF_SEL,
				  CA_NI_RX_DQ_PROFILE_SEL) :
		       CA_NI_RX_CPU_PROFILE_VAL,
		       ni_base(ni) + CA_NI_IDX(CA_NI_QM_DEST_PORT_EQ_CFG, i,
						CA_NI_QM_DEST_PORT_ENTRIES));

	/* 0x6ab0 = 0x300 (stock-matching; this is NOT the real ES_CTRL2 - kept as a
	 * harmless match). */
	writel(CA_NI_QM_ES_CTRL2_STOCK_VAL, ni_base(ni) + CA_NI_QM_ES_CTRL2);

	/* ★ REMOVED the 0x6a30 (ni_qm_hol bit1) write: a full ca-ne.ko .text scan (RE
	 * ae64b034 + coordinator) finds NO stock writer of 0x6a30 on the datapath - it
	 * was a sibling-chip guess, not the Elnath gate, and setting it did not help.
	 * The only 0x6a3x stock write is 0x6a3c bit2 (CMD_MODE_64), which we already do.
	 * The real RMU0-admit gate is the per-cpu-port ES dequeue enable 0x6108[7:0]
	 * (aal_l3qm_enable_tx_cpu, armed at open - see cortina_ni_rx_es_cpu). */

	/* dest-port pkt_buf (CPU-port-indexed table): head 4 (HEADER_A @+0x40)
	 * + tail 0x18, first and rest - stock 0x18041804 */
	ni_rmw(ni, CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT),
	       CA_NI_QM_PKT_BUF_HEAD_FIRST | CA_NI_QM_PKT_BUF_TAIL_FIRST |
	       CA_NI_QM_PKT_BUF_HEAD_REST | CA_NI_QM_PKT_BUF_TAIL_REST,
	       FIELD_PREP(CA_NI_QM_PKT_BUF_HEAD_FIRST, CA_NI_QM_PKT_BUF_HEAD_UNITS) |
	       FIELD_PREP(CA_NI_QM_PKT_BUF_TAIL_FIRST, CA_NI_QM_PKT_BUF_TAIL_UNITS) |
	       FIELD_PREP(CA_NI_QM_PKT_BUF_HEAD_REST, CA_NI_QM_PKT_BUF_HEAD_UNITS) |
	       FIELD_PREP(CA_NI_QM_PKT_BUF_TAIL_REST, CA_NI_QM_PKT_BUF_TAIL_UNITS));

	/* ★ Initialise the QM AXI-attribute table (0x67cc indirect) for our EQ pools
	 * + CPU-EPP ports.  This is the piece the raw 0x67cc=0x4000000F write botched:
	 * unprogrammed entries stall the QM's buffer-DMA so admission (qm_rx) never
	 * advances.  Uses the bounded DATA0->ACCESS(GO)->poll protocol (no hang). */
	cortina_ni_rx_axi_attrib_init(ni);

	/* ★ config block RE-ENABLED with the suspect HW-triggers (0x67cc + 0x69xx)
	 * excluded from the table (see cortina_ni_stock_qm_cfg) - bisecting the hang.
	 * DWRR weights + per-queue profile-sel only. */
	cortina_ni_rx_match_stock_qm(ni);

	/* (4) COMMIT: latch both bid ranges into the empty-buffer manager
	 * (stock aal_l3qm_load_eq_config).  Until this fires the pushed PAs
	 * never leave the shallow push stage and EQM_PA_REQ stays 0. */
	cortina_ni_rx_eq_commit(ni);
	dev_info(ni->dev,
		 "RX: committed EQ%d(%u)+EQ%d(%u), bid 0x%x/0x%x\n",
		 CA_NI_RX_EQ_ID, CA_NI_RX_EQ_TOTAL_BUF,
		 CA_NI_RX_EQ_ID2, CA_NI_RX_EQ2_TOTAL_BUF,
		 CA_NI_RX_EQ_BID_START, CA_NI_RX_EQ2_BID_START);

	/* (5) NO l3qmrx_to_lan / ni_qm_hol handoff here: the WORKING U-Boot ref
	 * has intern_pid(0xa1bc) bit20 = 0 and does not set ES_CTRL2 - and
	 * cortina_ni_rx_stock_routing already wrote 0xa1bc = 0x8080 wholesale.
	 * (This overrides the earlier "l3qmrx_to_lan must be set" reasoning, which
	 * the live working reference contradicts.) */

	/* (6) ★★ QM VOQ enable (stock aal_l3qm_init_voq, ca-ne.ko @0x53450):
	 * OR 0xff into voq_en[7:0] of EVERY QM SCH_CFG (Elnath 0x6424..0x64e0 step
	 * 4).  The RMU admits a frame INTO a QM VOQ, so if the target VOQ is disabled
	 * the admit fails (0x6900=0, no drop).  Our earlier "skip" read the WRONG
	 * offset (rtl 0x63c4, not the Elnath 0x6424) and wrongly concluded stock=0. */
	for (i = 0; i < CA_NI_QM_VOQ_EN_COUNT; i++)
		ni_rmw(ni, CA_NI_QM_VOQ_EN(i), 0, CA_NI_QM_VOQ_EN_ALL);
	dev_info(ni->dev, "qm-voq: 0x6424=0x%08x 0x6428=0x%08x (want voq_en[7:0]=0xff)\n",
		 readl(ni_base(ni) + CA_NI_QM_VOQ_EN(0)),
		 readl(ni_base(ni) + CA_NI_QM_VOQ_EN(1)));

	return 0;
}

/* Enable the L3QM egress scheduler master (stock aal_l3qm_enable_tx, 07f ko
 * @0x518f0).  This is the DRAIN side that moves an enqueued VOQ descriptor into
 * the CPU-EPP FIFO ring; without it the ring wptr never advances even though
 * the RMU0 RX master and the pool are live.  Program the master (tx_en) + the
 * NI-egress byte + the stock counter-increment config, and CLEAR cpu_en - the
 * per-CPU-port drain is armed at open (cortina_ni_rx_es_cpu), matching stock's
 * enable_tx-at-init / enable_tx_cpu-at-open split, so RX stays fully quiescent
 * until ndo_open. */
static void cortina_ni_rx_es_enable(struct cortina_ni *ni)
{
	/* match stock 0x8462FFFF: set tx_en/ni_en/inccfg, CLEAR the stray
	 * bit25 (ours came up 0x8662...); cpu_en is armed by es_cpu at open */
	ni_rmw(ni, CA_NI_QM_ES_CTRL,
	       CA_NI_QM_ES_CPU_EN | CA_NI_QM_ES_NI_EN |
	       CA_NI_QM_ES_INCCFG_PKT | CA_NI_QM_ES_INCCFG_ERR |
	       CA_NI_QM_ES_RSVD25,
	       CA_NI_QM_ES_TX_EN |
	       FIELD_PREP(CA_NI_QM_ES_NI_EN, 0xff) |
	       FIELD_PREP(CA_NI_QM_ES_INCCFG_PKT, CA_NI_QM_ES_INCCFG_PKT_VAL) |
	       FIELD_PREP(CA_NI_QM_ES_INCCFG_ERR, CA_NI_QM_ES_INCCFG_ERR_VAL));
	/* (A second copy of this write used to go to 0x7108 as a supposed "real"
	 * ES_CTRL.  0x7108 is EPP64_RDPTR(cpu_port 0, voq 2) - a ring read pointer -
	 * so that wrote a control word into hardware ring state.  It was harmless only
	 * by accident: cortina_ni_rx_poll_voq stores the rdptr unconditionally, so the
	 * next NAPI poll overwrote it.  Removed; 0x6108 above is the real register.) */
}

/* arm/disarm the CPU-port drain (stock aal_l3qm_enable_tx_cpu / l3_tm
 * es_cpu_port_ena_set).  Stock enables ALL cpu ports (cpu_en byte = 0xff) in
 * ca_ni_open's per-port loop; match that so the golden es_ctrl = 0x8462FFFF.
 * (build19's bit0-only narrowing was WRONG - tier-1 stock 0x6108=0x8462FFFF with
 * [7:0]=0xff, and our build18 0xff already matched; the RMU-admit bug was the
 * separate axi_reo DMA-reorder window, not this byte.) */
static void cortina_ni_rx_es_cpu(struct cortina_ni *ni, bool enable)
{
	u32 mask = FIELD_PREP(CA_NI_QM_ES_CPU_EN, CA_NI_QM_ES_CPU_EN_ALL);

	if (enable)
		ni_rmw(ni, CA_NI_QM_ES_CTRL, CA_NI_QM_ES_RSVD25, mask);
	else
		ni_rmw(ni, CA_NI_QM_ES_CTRL, mask, 0);
}

/* FBM pools the RMU allocates CPU-RX buffers from.  RE (aca2223c): the pool is chosen
 * by the frame's queue, and a deep_q=1 / dest-port-8 / voqid=8 frame draws from the
 * DEEP-QUEUE pool = stock l3qm_eq_profile_dq fbm_pool_id 7 - a DIFFERENT pool than
 * cpu_pool0.  We filled only pool 0, so the deep-queue pop found it empty -> no_buffer
 * drop (0x611c b8, 0x693c voqid=8, eqid=0=unresolved).  Fill BOTH pool 0 and pool 7 so
 * whichever the RMU pops from is seeded.  Each pool needs its OWN exstack (pointer-spill)
 * + buffer region in the 0x09000000 no-map reserve; all 2048B (>= max frame). */
static const struct { u8 id; u32 exstack; u32 buf_base; } cortina_ni_fbm_pools[] = {
	{ 0, 0x0A000000u, 0x09404000u },	/* cpu_pool0 (non-deep) - our EQ14 2048B region */
	{ 7, 0x0A010000u, 0x0A100000u },	/* deep-queue pool (voqid=8, stock fbm_pool_id 7) */
};

/* ★★ FBM (Free Buffer Manager) init - the HW buffer-allocator the RMU pops a buffer
 * from to DMA each admitted RX frame into.  Lives on a SEPARATE window group (FBM_GLB/
 * AXI/CPU/POOL @0x90300800/900/a00/b00) our driver never mapped or wrote.  RE (ca-ne.ko
 * aal_fbm_reset / aal_fbm_init / aal_fbm_pool_init) corrected the whole approach:
 *  - the pool is ENABLED by the GLB pool bit (GLB+0x00 low byte 0xFF), NOT by POOL+0x30
 *    (that read/write-enable+preload is a DEBUG DMA-dump interface - setting it triggered
 *    a real exstack DMA and CRASHED us; never touch it for init);
 *  - ★ POOL+0x04 is the EXSTACK pointer-spill base (not a buffer base): the FBM DMAs
 *    buffer-pointer spills there.  We left it 0 -> spill at phys 0 = kernel slab ->
 *    build24 panic.  It must point at a real reserved DRAM region (SEPARATE from the
 *    packet buffers), encoded (pbase>>12)<<4;
 *  - the free-list is FILLED by pushing buffer PAs through the FBM_CPU gated doorbell
 *    (cortina_ni_rx_fbm_fill), NOT the POOL+0x40 raw store (which never registered).
 * Sequence: reset -> GLB/AXI config -> pool geometry+exstack -> (later) doorbell fill. */
static void cortina_ni_rx_fbm_init(struct cortina_ni *ni)
{
	void __iomem *glb    = ni->win[CA_NI_WIN_FBM_GLB];
	void __iomem *axi    = ni->win[CA_NI_WIN_FBM_AXI];
	void __iomem *pool   = ni->win[CA_NI_WIN_FBM_POOL];
	void __iomem *ni_glb = ni->win[CA_NI_WIN_GLB];
	unsigned int k;

	if (!glb || !axi || !pool) {
		dev_warn(ni->dev,
			 "fbm: window(s) unmapped (glb=%d axi=%d pool=%d) - RMU cannot alloc\n",
			 !!glb, !!axi, !!pool);
		return;
	}

	/* (1) aal_fbm_reset: soft-reset the FBM via NI GLB-ctrl +0xa0 bit17, pulse 1->0. */
	if (ni_glb) {
		u32 v = readl(ni_glb + CA_NI_GLB_FBM_RESET);

		writel(v | CA_NI_GLB_FBM_RESET_BIT, ni_glb + CA_NI_GLB_FBM_RESET);
		cortina_ni_rx_settle();
		writel(v & ~CA_NI_GLB_FBM_RESET_BIT, ni_glb + CA_NI_GLB_FBM_RESET);
		cortina_ni_rx_settle();
	}

	/* (2) aal_fbm_init GLB config: +0x04 mode, +0x70 ECC, +0x00 low byte 0xFF =
	 * enable pools 0-7 (★ THE pool enable is this GLB bit, NOT POOL+0x30). */
	writel(0x00060100, glb + 0x04);
	writel(0xE0C04025, glb + 0x70);
	writel(0x010109FF, glb + 0x00);

	writel(0x00000200, axi + 0x00);

	/* (3) aal_fbm_pool_init per pool (id*0x80): geometry + the EXSTACK pointer-spill
	 * region.  +0x04 = exstack_phys>>12 in [31:4]; +0x08 = spill depth; +0x0c =
	 * ((count/64)-1)<<6.  NO +0x30 (debug DMA), NO +0x40 here. */
	for (k = 0; k < ARRAY_SIZE(cortina_ni_fbm_pools); k++) {
		void __iomem *p = pool + CA_NI_QM_FBM_POOL(cortina_ni_fbm_pools[k].id);
		u32 exstack = (cortina_ni_fbm_pools[k].exstack >> 12) << 4;

		writel(0xC0400300, p + 0x00);
		writel(exstack, p + 0x04);
		writel(CA_NI_RX_FBM_EXSTACK_DEPTH, p + 0x08);
		writel(((CA_NI_RX_FBM_POOL0_COUNT / 64) - 1) << 6, p + 0x0c);

		dev_info(ni->dev,
			 "fbm cfg pool%u: cfg0=0x%08x exstack(0x04)=0x%08x depth=0x%08x cnt=0x%08x\n",
			 cortina_ni_fbm_pools[k].id,
			 readl(p + 0x00), readl(p + 0x04),
			 readl(p + 0x08), readl(p + 0x0c));
	}
	dev_info(ni->dev, "fbm cfg: glb0=0x%08x (%zu pools configured)\n",
		 readl(glb + 0x00), ARRAY_SIZE(cortina_ni_fbm_pools));
}

/* ★★ Fill the FBM pool free-list via the FBM_CPU GATED doorbell (stock aal_fbm_buf_push
 * / __aal_fbm_buf_access) - the interface the RMU actually pops from on this board
 * (use_fbm=1, eq_rule=0 => software MUST push).  Per buffer: (gate 1) wait until the
 * outstanding count POOL+0x2c < the exstack depth; (gate 2) poll the FBM_CPU cmd word
 * bit31 (BUSY) clear; then write addr-high (+0x04), addr-low (+0x08), and a GO|push cmd
 * (+0x00).  Buffer = a raw reserved-DRAM PA.  ★ NOT the POOL+0x40 raw store (which never
 * registered - outstanding stayed 0) and NOT the POOL+0x30 preload (a debug DMA-dump
 * interface that DMAs the exstack and crashed us).  Bounded polls - can't hang. */
static void cortina_ni_rx_fbm_fill(struct cortina_ni *ni)
{
	void __iomem *pool = ni->win[CA_NI_WIN_FBM_POOL];
	void __iomem *cpu  = ni->win[CA_NI_WIN_FBM_CPU];
	unsigned int k, i, s;

	if (!pool || !cpu) {
		dev_warn(ni->dev, "fbm fill: pool/cpu window unmapped - cannot fill\n");
		return;
	}

	for (k = 0; k < ARRAY_SIZE(cortina_ni_fbm_pools); k++) {
		u8 id = cortina_ni_fbm_pools[k].id;
		void __iomem *db = cpu + CA_NI_QM_FBM_CPU_DOORBELL(id);
		void __iomem *p  = pool + CA_NI_QM_FBM_POOL(id);
		u32 base = cortina_ni_fbm_pools[k].buf_base;

		for (i = 0; i < CA_NI_RX_FBM_POOL0_COUNT; i++) {
			u32 buf = base + i * CA_NI_RX_FBM_POOL_BUFSZ;

			/* gate 1: outstanding < depth (else the push is rejected -1) */
			for (s = 0; s < 4096; s++) {
				if (readl(p + CA_NI_QM_FBM_POOL_OUTSTND) <
				    CA_NI_RX_FBM_EXSTACK_DEPTH)
					break;
				cpu_relax();
			}
			/* gate 2: FBM_CPU cmd not BUSY (bit31 clear) before issuing */
			for (s = 0; s < 4096; s++) {
				if (!(readl(db + 0x00) & CA_NI_QM_FBM_CPU_CMD_GO))
					break;
				cpu_relax();
			}
			writel(0, db + 0x04);		/* addr high (32-bit PA -> 0) */
			writel(buf, db + 0x08);		/* addr low */
			writel(CA_NI_QM_FBM_CPU_CMD_GO | CA_NI_QM_FBM_CPU_CMD_PUSH,
			       db + 0x00);		/* GO | op=push (pool = offset) */
		}

		dev_info(ni->dev,
			 "fbm fill pool%u: pushed %u bufs @0x%08x+; outstanding(0x2c)=0x%08x\n",
			 id, CA_NI_RX_FBM_POOL0_COUNT, base,
			 readl(p + CA_NI_QM_FBM_POOL_OUTSTND));
	}
}

/* ★★ Program the RMU AXI read/write REORDER engine (stock axi_reo_rd_init/wr_init,
 * run by ca_ni_init_l3qm after enable_rx).  This lives in a SEPARATE MMIO block - DT
 * window idx 10 (g_ne_axi_reo, 0xf432d000), NOT the NI core window - which our driver
 * never wrote.  Without it the RMU's dequeue-side AXI transactions never complete, so
 * a CPU-bound frame reaches the QM but is never admitted (wptr=0, 0x6900=0, no drop). */
/* Full stock axi_reo golden (tier-1 live devmem, g_ne_axi_reo base 0xf432d000):
 * THREE channel blocks - READ (0x000), WRITE (0x400), WRITE2 (0x480) - each with 7
 * non-zero words at block-relative 0x00/0x04/0x08/0x0c/0x10/0x18/0x24 (the two
 * 0xFFFFFFFF at +0x18/+0x24 are mask/valid words); every other offset resets to 0.
 * Blocks 1&2 are identical bar word[0] (0x0F read vs 0x04 write); block 3 differs
 * (word[0]=0x02, word[1]=0x80000008, word[4]=0x80000009).  Our OLD init wrote only
 * 6 words with rd3@0x0c wrong (0x8000000d belongs at +0x10) and skipped block 3
 * entirely -> the RMU's dequeue-side AXI DMA never completed, so a CPU-bound frame
 * reached the QM but was NEVER admitted (0x6900=0, 0x6944=0, wptr 0x7000=0).  This
 * engine is on the SEPARATE g_ne_axi_reo window (DT idx10), invisible to every
 * NI-core-window diff - which is why all NI regs matched stock yet RX was dead. */
static const struct { u16 off; u32 val; } cortina_ni_axi_reo_cfg[] = {
	{ 0x000, 0x0000000F }, { 0x004, 0x8000000C }, { 0x008, 0x10000000 },
	{ 0x00c, 0x10000000 }, { 0x010, 0x8000000D }, { 0x018, 0xFFFFFFFF },
	{ 0x024, 0xFFFFFFFF },
	{ 0x400, 0x00000004 }, { 0x404, 0x8000000C }, { 0x408, 0x10000000 },
	{ 0x40c, 0x10000000 }, { 0x410, 0x8000000D }, { 0x418, 0xFFFFFFFF },
	{ 0x424, 0xFFFFFFFF },
	{ 0x480, 0x00000002 }, { 0x484, 0x80000008 }, { 0x488, 0x10000000 },
	{ 0x48c, 0x10000000 }, { 0x490, 0x80000009 }, { 0x498, 0xFFFFFFFF },
	{ 0x4a4, 0xFFFFFFFF },
};

static void cortina_ni_rx_axi_reo_init(struct cortina_ni *ni)
{
	void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];
	unsigned int i;

	if (!reo) {
		dev_warn(ni->dev,
			 "RX: AXI-reorder window (idx %d) not mapped - RMU DMA stalls\n",
			 CA_NI_WIN_AXI_REO);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(cortina_ni_axi_reo_cfg); i++)
		writel(cortina_ni_axi_reo_cfg[i].val,
		       reo + cortina_ni_axi_reo_cfg[i].off);

	dev_info(ni->dev,
		 "RX: AXI-reorder init (%zu regs): rd[0x00/0x0c/0x10]=0x%08x/0x%08x/0x%08x wr[0x400]=0x%08x wr2[0x480]=0x%08x\n",
		 ARRAY_SIZE(cortina_ni_axi_reo_cfg),
		 readl(reo + 0x000), readl(reo + 0x00c), readl(reo + 0x010),
		 readl(reo + 0x400), readl(reo + 0x480));
}

/* ------------------------------------------------------------------ */
/* L3QM CPU-EPP ring init (stock aal_l3qm_init_cpu_epp, port0/voq0)    */
/* ------------------------------------------------------------------ */

static void cortina_ni_rx_epp_init(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;
	u32 wptr;
	unsigned int i;

	/* ★★ build61: set the STOCK CPU-EPP interrupt-enables now (match_stock_qm wrote them
	 * but was clobbered).  0x6110=0x0000FFFF - bits[15:8] are the writeback-completion/wptr
	 * latch enable, the piece the writeback engine needed; 0x6118=0x00000100 (refill-thr
	 * IRQ en); 0x6114 (EN1, cpu ports 4-7) stays 0 = stock.  (build49's 0xffffffff to BOTH
	 * regressed L3QM egress; the exact 0xFFFF/0x100 stock values do not.) */
	writel(CA_NI_QM_EPP64_INT_EN0_STOCK, ni_base(ni) + CA_NI_QM_EPP64_INT_EN0);
	writel(0, ni_base(ni) + CA_NI_QM_EPP64_INT_EN1);
	writel(CA_NI_QM_EPP64_INT_EN2_STOCK, ni_base(ni) + CA_NI_QM_EPP64_INT_EN2);

	/* ★★ build57: the 0x6a3c bit2 cmd_mode/GO set is MOVED to AFTER the per-voq FIFO
	 * paddr loop below (stock init_cpu_epp order: 0x7200 paddr -> 0x6a3c GO LAST).  It is
	 * the run bit for the EPP writeback engine; arming it BEFORE the FIFO paddr latched the
	 * engine in a bad state so it NEVER wrote back (all 64 wptr slots 0x7000-0x70fc stuck
	 * 0, 0x611c bit22 backpressure, QM wedge). */

	/* linear port->voq map */
	ni_rmw(ni, CA_NI_QM_CPU_EPP_CFG(CA_NI_RX_CPU_PORT),
	       CA_NI_QM_EPP_MAP_MODE, 0);

	/* no descriptor coalescing timer */
	writel(0, ni_base(ni) + CA_NI_QM_CPU_EPP_CT_CFG);

	/* ★★ build48 FIX: do NOT re-write CPU_EPP_FIFO_PROF(4)=0x66b4 here.  match_stock_qm
	 * (cortina_ni_stock_qm_cfg) already set it to the correct stock init_cpu_epp value
	 * 0xE00040F1; this RMW recomputed SIZE (EPP_PER_VOQ/4) and CLOBBERED it to 0xe00020f1
	 * (bit14->bit13), which gated the CPU-EPP ring delivery (wptr stuck 0 even though the
	 * QM VOQ drained, tx_cntr 0x690c climbing).  RE-confirmed 0xE00040F1 is correct and
	 * this write is spurious - removed so the stock value survives. */

	/* ★ Arm ALL 8 VOQs of the CPU port: each gets its FIFO profile + its own
	 * contiguous ring region (ring_dma + voq*RING_BYTES).  Stock arms every voq;
	 * we armed only voq0, so a deep_q frame on voq!=0 had no ring to push to. */
	WARN_ON_ONCE(upper_32_bits(rx->ring_dma));
	for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++) {
		u32 vphys = lower_32_bits(rx->ring_dma) + i * CA_NI_RX_RING_BYTES;

		ni_rmw(ni, CA_NI_QM_CPU_EPP_FIFO_CFG(CA_NI_RX_CPU_PORT, i),
		       CA_NI_QM_EPP_PROFILE_SEL,
		       FIELD_PREP(CA_NI_QM_EPP_PROFILE_SEL, CA_NI_RX_PROFILE_ID));

		writel(vphys, ni_base(ni) +
		       CA_NI_QM_EPP64_PADDR_START(CA_NI_RX_CPU_PORT, i));
		/* ★★★ build80: the SECOND per-voq ring buffer (PADDR_HI = PADDR + 0x2000).
		 * Stock sets it; ours was 0 -> the writeback engine wrote ZERO descriptors. */
		writel(vphys + CA_NI_RX_RING_HI_OFFSET, ni_base(ni) +
		       CA_NI_QM_EPP64_PADDR_HI(CA_NI_RX_CPU_PORT, i));

		/* adopt the HW write pointer (0 after reset) as this voq's start */
		wptr = cortina_ni_rx_wptr_voq(ni, i);
		if (wptr)
			dev_warn(ni->dev, "RX voq%u wptr not idle at init (0x%x)\n",
				 i, wptr);
		rx->rptr[i] = wptr;
		writel(wptr, ni_base(ni) +
		       CA_NI_QM_EPP64_RDPTR(CA_NI_RX_CPU_PORT, i));
	}

	/* ★★ build57 THE FIX: set 0x6a3c bit2 (cmd_mode/GO = the EPP writeback-engine run
	 * bit) LAST, AFTER every voq's 0x7200 FIFO paddr + profile + rdptr are latched -
	 * exactly stock init_cpu_epp's order.  Arming GO before the paddr wedged the engine
	 * (no writeback, wptr stuck 0).  Our NAPI parses the 64-bit __le64 ring, so bit2
	 * (64-bit descriptor mode) is also the mode our ring parse needs. */
	ni_rmw(ni, CA_NI_QM_EPP, 0, CA_NI_QM_EPP_CMD_MODE_64);
	dev_info(ni->dev, "epp-init: cmd_mode/GO(0x6a3c)=0x%08x set LAST (after per-voq paddr)\n",
		 readl(ni_base(ni) + CA_NI_QM_EPP));

	/* ★★ THE CPU-egress ES enable (enable_tx_cpu equivalent).  Re-assert the
	 * egress-scheduler egress-enable PAIR here, LAST, after the 0x6a3c GO - the
	 * only spot nothing clobbers.  The tx-path enable 0x6a20 already reads 0xFF00,
	 * but its CPU-path sibling 0x6a00, though set by match_stock_qm, reads back 0
	 * (the EPP engine arming clears it).  Stock live: both = 0x0000FF00.  Without
	 * the CPU-path enable the ES never services CPU_0 egress -> the CPU-EPP
	 * writeback is starved -> the LOW ring keeps its DEADBEEF poison (PA=0). */
	writel(CA_NI_QM_EPP_EGR_EN_ALL, ni_base(ni) + CA_NI_QM_EPP_TX_EGR_EN);
	writel(CA_NI_QM_EPP_EGR_EN_ALL, ni_base(ni) + CA_NI_QM_EPP_CPU_EGR_EN);
	dev_info(ni->dev, "epp-egr: tx(0x6a20)=0x%08x cpu(0x6a00)=0x%08x (want both 0x0000ff00)\n",
		 readl(ni_base(ni) + CA_NI_QM_EPP_TX_EGR_EN),
		 readl(ni_base(ni) + CA_NI_QM_EPP_CPU_EGR_EN));
}

/* ------------------------------------------------------------------ */
/* GPHY fault poll + port reinit (stock aal_internal_phy_recovery)     */
/* ------------------------------------------------------------------ */

/*
 * WHY RX is nondeterministic across boots: the internal quad-GPHY can wedge
 * its RX path (typically across one of the boot-time link renegotiations)
 * while the link still reads Up and every MAC/QM register stays byte-
 * identical to a good boot - the classic "config matches, behaviour
 * doesn't".  Realtek knows: stock polls a GPHY fault latch (page 0xb90
 * reg 19) at 1 Hz on every port and, whenever it reads nonzero, performs a
 * full port interface reinit.  Our driver used to do neither, so a boot
 * that latched the fault stayed RX-dead forever (an rx_en/es_cpu/ndo_open
 * toggle does not clear this state - only the reinit below does).
 */

static inline void __iomem *cortina_ni_rx_gphy(struct cortina_ni *ni)
{
	/* the gphy window is optional in the DT: NULL = feature unavailable */
	if (!ni->win[CA_NI_WIN_GPHY])
		return NULL;
	return ni->win[CA_NI_WIN_GPHY] + CA_NI_GPHY_BANK(CA_NI_RX_PORT);
}

static u32 cortina_ni_rx_gphy_fault(struct cortina_ni *ni)
{
	void __iomem *gphy = cortina_ni_rx_gphy(ni);

	return gphy ? readl(gphy + CA_NI_GPHY_FAULT) & 0xffff : 0;
}

/* analog calibration registers stock reloads after the reinit */
static const u32 cortina_ni_rx_gphy_cal_off[CA_NI_RX_GPHY_CAL_REGS] = {
	CA_NI_GPHY_EXT(0xbcd, 22),	/* rc_cal_len_l */
	CA_NI_GPHY_EXT(0xbcd, 23),
	CA_NI_GPHY_EXT(0xbcf, 18),	/* r_cal (tapbin A-D) */
	CA_NI_GPHY_EXT(0xbcf, 19),
	CA_NI_GPHY_EXT(0xbcf, 20),
	CA_NI_GPHY_EXT(0xbcf, 21),
	CA_NI_GPHY_EXT(0xbca, 22),	/* amp_cal (ibadj) */
};

static void cortina_ni_rx_gphy_cal_save(struct cortina_ni *ni)
{
	void __iomem *gphy = ni->win[CA_NI_WIN_GPHY];
	unsigned int b;
	int i;

	if (!gphy)
		return;

	/* taken at probe: the U-Boot-initialized state that just TFTP'd the
	 * kernel over the cabled port, i.e. a proven-working calibration.
	 * Snapshot EVERY bank so the per-port interface reinit can restore the
	 * cabled port's cal (any of 0..3), not just port 0's. */
	for (b = 0; b < CA_NI_GPHY_COUNT; b++)
		for (i = 0; i < CA_NI_RX_GPHY_CAL_REGS; i++)
			ni->rx->gphy_cal[b][i] =
				readl(gphy + CA_NI_GPHY_BANK(b) +
				      cortina_ni_rx_gphy_cal_off[i]);
}

static void cortina_ni_rx_gphy_intf_rst_pulse(struct cortina_ni *ni)
{
	writel(CA_NI_HV_INTF_RST_GPHY(CA_NI_RX_PORT),
	       ni_base(ni) + CA_NI_HV_INTF_RST);
	usleep_range(1000, 1500);	/* stock: 1 ms */
	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);
}

/*
 * Force the GPHY-wrapper enables to the stock golden steady state (EN0 =
 * 0xFF000000, EN1 = 0x1001).  EN1 bit12 (patch_phy_done) is the GPHY->port-MAC
 * datapath release - the RX determinism gate.  Idempotent; also clears any
 * stray per-port EN1[4+p] toggle so we land on 0x1001 after a fault reinit.
 * Re-run on every link-up in case phylib's PHY handling disturbed the wrapper.
 */
static void cortina_ni_rx_wrap_establish(struct cortina_ni *ni)
{
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];

	if (!wrap)
		return;
	writel(CA_NI_GPHY_WRAP_EN0_VAL, wrap + CA_NI_GPHY_WRAP_EN0);
	writel(CA_NI_GPHY_WRAP_EN1_VAL, wrap + CA_NI_GPHY_WRAP_EN1);
}

/* the stock reinit sequence, port 0 only (order verified in the shipped ko;
 * every step is a plain register write - nothing here can hang) */
static void cortina_ni_rx_gphy_reinit(struct cortina_ni *ni)
{
	void __iomem *gphy = cortina_ni_rx_gphy(ni);
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];
	u32 val;
	int i;

	if (!gphy)
		return;

	/* serialize against phylib's MDIO polling of the same GPHY */
	mutex_lock(&ni->mii->mdio_lock);

	cortina_ni_rx_gphy_intf_rst_pulse(ni);

	/* re-enable the GPHY uC patch/self-check */
	val = readl(gphy + CA_NI_GPHY_PATCH_EN);
	writel(val | CA_NI_GPHY_PATCH_EN_BIT, gphy + CA_NI_GPHY_PATCH_EN);

	/* wrapper interface toggle (ko-only step, absent in the SDK C) */
	if (wrap) {
		val = readl(wrap + CA_NI_GPHY_WRAP_EN1);
		writel(val & ~CA_NI_GPHY_WRAP_EN1_IF(CA_NI_RX_PORT),
		       wrap + CA_NI_GPHY_WRAP_EN1);
		writel(val | CA_NI_GPHY_WRAP_EN1_IF(CA_NI_RX_PORT),
		       wrap + CA_NI_GPHY_WRAP_EN1);
	}

	cortina_ni_rx_gphy_intf_rst_pulse(ni);
	msleep(200);			/* stock: 200 ms settle */

	/* restore the probe-time calibration snapshot (register file only;
	 * the DSP-SRAM mirror is uC-patch-specific, see cortina-ni-regs.h) */
	for (i = 0; i < CA_NI_RX_GPHY_CAL_REGS; i++)
		writel(ni->rx->gphy_cal[CA_NI_RX_PORT][i],
		       gphy + cortina_ni_rx_gphy_cal_off[i]);

	/* power the PHY back up + release the page-0xa46 hold bit */
	val = readl(gphy + CA_NI_GPHY_BMCR);
	writel(val & ~CA_NI_GPHY_BMCR_PDOWN, gphy + CA_NI_GPHY_BMCR);
	val = readl(gphy + CA_NI_GPHY_HOLD);
	writel(val & ~CA_NI_GPHY_HOLD_BIT, gphy + CA_NI_GPHY_HOLD);

	/* land on the stock steady wrapper state (EN1 = 0x1001, per-port toggle
	 * cleared, patch_phy_done re-set) so RX resumes after the reinit */
	cortina_ni_rx_wrap_establish(ni);

	mutex_unlock(&ni->mii->mdio_lock);
}

/* ★ Per-port GPHY<->MAC interface establishment (Fable RE 2026-07-22): the
 * vendor per-port INTF_RST + EN1_IF-edge + 200ms-settle sequence that connects
 * GPHY <port> to its MAC AFTER the port's SRAM bank is patched.  gphy_reinit
 * above does this for CA_NI_RX_PORT=0 ONLY - so a cabled port other than 0 (this
 * rig: port 3) LINKS but never delivers a frame into the L2FE, because its
 * MAC-side GMII sync is left at the pre-patch state.  EN1_IF(port) is an
 * edge/strobe (not a resting level), pulsed here between the two INTF_RSTs. */
static void cortina_ni_rx_gphy_intf_establish(struct cortina_ni *ni,
					      unsigned int port)
{
	void __iomem *gphy = ni->win[CA_NI_WIN_GPHY];
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];
	void __iomem *bank;
	u32 val;
	int i;

	if (!gphy || !ni->mii || port >= CA_NI_GPHY_COUNT)
		return;
	bank = gphy + CA_NI_GPHY_BANK(port);

	mutex_lock(&ni->mii->mdio_lock);

	/* 1st INTF_RST pulse for THIS port */
	writel(CA_NI_HV_INTF_RST_GPHY(port), ni_base(ni) + CA_NI_HV_INTF_RST);
	usleep_range(1000, 1500);
	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);

	/* re-enable the uC patch/self-check on THIS bank */
	val = readl(bank + CA_NI_GPHY_PATCH_EN);
	writel(val | CA_NI_GPHY_PATCH_EN_BIT, bank + CA_NI_GPHY_PATCH_EN);

	/* wrapper EN1_IF(port) 0->1 EDGE (connect THIS GPHY to its MAC) */
	if (wrap) {
		val = readl(wrap + CA_NI_GPHY_WRAP_EN1);
		writel(val & ~CA_NI_GPHY_WRAP_EN1_IF(port),
		       wrap + CA_NI_GPHY_WRAP_EN1);
		writel(val | CA_NI_GPHY_WRAP_EN1_IF(port),
		       wrap + CA_NI_GPHY_WRAP_EN1);
	}

	/* 2nd INTF_RST pulse + 200 ms settle (stock) */
	writel(CA_NI_HV_INTF_RST_GPHY(port), ni_base(ni) + CA_NI_HV_INTF_RST);
	usleep_range(1000, 1500);
	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);
	msleep(200);

	/* restore THIS bank's probe-time analog cal */
	for (i = 0; i < CA_NI_RX_GPHY_CAL_REGS; i++)
		writel(ni->rx->gphy_cal[port][i],
		       bank + cortina_ni_rx_gphy_cal_off[i]);

	/* power up + release hold on THIS bank */
	val = readl(bank + CA_NI_GPHY_BMCR);
	writel(val & ~CA_NI_GPHY_BMCR_PDOWN, bank + CA_NI_GPHY_BMCR);
	val = readl(bank + CA_NI_GPHY_HOLD);
	writel(val & ~CA_NI_GPHY_HOLD_BIT, bank + CA_NI_GPHY_HOLD);

	/* land the wrapper on stock steady EN1=0x1001 */
	cortina_ni_rx_wrap_establish(ni);

	mutex_unlock(&ni->mii->mdio_lock);
	dev_info(ni->dev, "gphy port %u: MAC<->GPHY interface established\n",
		 port);
}

/* 1 Hz self-rearming poll, stock cadence ("recover check first").  Runs
 * between open and stop; each pass is one register read unless faulted. */
static void cortina_ni_rx_recovery_work(struct work_struct *work)
{
	struct cortina_ni_rx *rx = container_of(to_delayed_work(work),
						struct cortina_ni_rx,
						recovery_work);
	struct cortina_ni *ni = rx->ni;
	u32 fault = cortina_ni_rx_gphy_fault(ni);

	rx->last_fault = fault;
	if (unlikely(fault)) {
		dev_warn(ni->dev,
			 "GPHY port %d fault latch 0x%04x - reinit (#%llu)\n",
			 CA_NI_RX_PORT, fault, rx->recoveries + 1);
		cortina_ni_rx_gphy_reinit(ni);
		rx->recoveries++;
		dev_info(ni->dev, "GPHY port %d reinit done (latch now 0x%04x)\n",
			 CA_NI_RX_PORT, cortina_ni_rx_gphy_fault(ni));
	}

	/* ★ Decoupled datapath bring-up: until every GPHY bank is patched, run the
	 * full link-up bring-up here (1 Hz) so the LAN ports ingress even when
	 * eth0's connected PHY has no cable and never fires link-up.  Idempotent;
	 * the GPHY SRAM patch is one-shot per bank (ni->gphy_patched[]); capped via
	 * rearms so a never-lockable bank cannot spin the full reconfig forever. */
	if (rx_decoupled_bringup && !rx->intf_done) {
		unsigned int b;
		bool all_patched;

		/* keep driving the bring-up (patches the GPHY banks) until done */
		if (rx->rearms < 30)
			cortina_ni_rx_link_up(ni);

		all_patched = true;
		for (b = 0; b < CA_NI_GPHY_COUNT; b++)
			if (!ni->gphy_patched[b]) {
				all_patched = false;
				break;
			}

		/* ★ once EVERY bank is patched, establish the MAC<->GPHY interface
		 * for every port IN THIS SAME TICK (Fable RE #1) - do NOT defer to
		 * a later recovery tick, which may never come if the netdev/poll
		 * stops after port-0's PHY stays link-down.  The cabled port (any
		 * of 0..3, this rig port 3) needs the per-port INTF_RST+EN1_IF edge
		 * after its bank patch or it links but delivers no frame into the
		 * L2FE.  Once. */
		if (all_patched) {
			unsigned int p;

			rx->intf_done = true;
			dev_info(ni->dev,
				 "all GPHY banks patched -> establishing MAC<->GPHY interface on all ports\n");
			for (p = 0; p < CA_NI_GPHY_COUNT; p++)
				cortina_ni_rx_gphy_intf_establish(ni, p);
		}
	}

	/* ★ Hold the CPU-port (eth0) carrier UP every tick once the datapath has been
	 * armed (rx->rearms>0).  eth0 is the CPU<->switch port; a carrier-off bridge
	 * member is disabled and br-lan drops all LAN frames the switch delivered.  Do
	 * it here, not only in link_up (which stops once intf_done), so any later
	 * netif_carrier_off (netifd re-ifup, phy_stop, a port-0 bounce) is undone
	 * within 1 s.  The phy_link_change override stops phylib's own carrier-off. */
	if (rx->rearms && rx->netdev && !netif_carrier_ok(rx->netdev)) {
		netif_carrier_on(rx->netdev);
		netdev_info(rx->netdev,
			    "CPU-port carrier re-asserted (switch datapath up)\n");
	}

	/*
	 * ★ Publish which RJ45s have a PHY link, for the CPU->LAN egress port
	 * choice (cortina-ni-tx.c): we may only stamp or flood to a port whose
	 * PHY is up, because a frame handed to a dead egress MAC sits there and
	 * consumes the shared L2TM buffer pool.  This poll is the only place
	 * that may do it - mdiobus_read takes the MDIO mutex, so neither the
	 * xmit path nor the reclaim timer could.  First read clears the
	 * latched-low bit, second is the live state (as the /proc reader does).
	 */
	if (ni->mii) {
		u32 link = 0;
		unsigned int p;

		for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
			int a = CA_NI_GPHY_FIRST + p, bmsr;

			mdiobus_read(ni->mii, a, MII_BMSR);	/* clear latch */
			bmsr = mdiobus_read(ni->mii, a, MII_BMSR);
			if (bmsr >= 0 && (bmsr & BMSR_LSTATUS))
				link |= BIT(p);
		}
		cortina_ni_lan_tx_link_set(ni, link);

		/*
		 * ★ Same bitmap, one more consumer: the front-panel link lamps.
		 * Deliberately the LAST thing in the tick, and by construction
		 * unable to sleep, block or fail - it is a void call that ends
		 * in a GPIO store and takes no lock this function holds - so it
		 * can neither delay nor break the GPHY bring-up above.
		 * cortina-ni-leds.c documents why that holds, and why the
		 * netdev trigger cannot do this job on a one-netdev switch.
		 */
		cortina_ni_leds_link_set(link);
	}

	schedule_delayed_work(&rx->recovery_work, HZ);
}

/*
 * phylib link-up hook (called from cortina_ni_tx_adjust_link): idempotently
 * re-arm the whole RX delivery chain, so anything a boot-time link bounce
 * may have clobbered is re-applied, and immediately run one GPHY fault
 * check - the wedge, when it happens, latches across exactly such a bounce.
 */
void cortina_ni_rx_link_up(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;

	if (!rx)
		return;		/* TX-only mode */

	/* ★ DIAGNOSTIC: dphy_rst reset-manager (GLB+0xa0) = internal digital-PHY
	 * reset.  Ours boots 0x50302340 (many reset bits set), STOCK(working)=
	 * 0x10000000 -> ours holds internal-GPHY/datapath sub-blocks in reset.
	 * Write stock's value to release them and see if the datapath comes alive. */
	if (ni->win[CA_NI_WIN_GLB]) {
		void __iomem *glb = ni->win[CA_NI_WIN_GLB];

		dev_info(ni->dev, "dphy_rst(glb+0xa0) was 0x%08x -> writing 0x10000000\n",
			 readl(glb + 0xa0));
		writel(0x10000000, glb + 0xa0);
	}

	/* ★ Load the internal-GPHY SRAM firmware + resume the uC HERE (at link-up),
	 * not at probe: at probe 0x291a0 reads 0x0 (uC running, SRAM not writable);
	 * it becomes 0x3 (uC HELD, SRAM writable) only after the MDIO/GPHY init.
	 * Per held bank: write the firmware image, then clear HOLD bit1 -> 0x1 so
	 * the uC runs the PATCHED code (a bare resume to 0x1 matches stock's state
	 * but its ROM firmware does NOT forward).  Idempotent/one-shot. */
	cortina_ni_gphy_patch_and_resume(ni);

	/* ★ Once every GPHY bank is patched, establish the per-port MAC<->GPHY
	 * interface (Fable RE #1) HERE - link_up is fired by adjust_link and DOES
	 * run; recovery_work (the other candidate) is canceled when eth0/port-0 goes
	 * carrier-down, so it never gets a tick.  The cabled port (any of 0..3, this
	 * rig port 3) needs the per-port INTF_RST+EN1_IF edge after its bank patch or
	 * it links but delivers no frame into the L2FE.  Once (ni->rx->intf_done). */
	if (ni->rx && !ni->rx->intf_done) {
		unsigned int b, p;
		bool all_patched = true;

		for (b = 0; b < CA_NI_GPHY_COUNT; b++)
			if (!ni->gphy_patched[b]) {
				all_patched = false;
				break;
			}
		if (all_patched) {
			ni->rx->intf_done = true;
			dev_info(ni->dev,
				 "all GPHY banks patched -> establishing MAC<->GPHY interface on all ports (in link_up)\n");
			for (p = 0; p < CA_NI_GPHY_COUNT; p++)
				cortina_ni_rx_gphy_intf_establish(ni, p);
		}
	}

	if (!rx_skip_portcfg) {
		unsigned int p;

		/* ★ re-establish the GPHY->port-MAC datapath (wrapper EN1 bit12) FIRST:
		 * this is the ingress determinism gate - without it the PHY links but no
		 * frame reaches the MAC.  Idempotent; also heals any phylib disturbance. */
		cortina_ni_rx_wrap_establish(ni);

		/* ★ re-assert the MAC<->GPHY internal GMII interface (int_cfg=GE_GMII,
		 * phy_mode=MAC, MAC-loopback OFF) for EVERY GPHY LAN port, not just
		 * CA_NI_RX_PORT.  The host cable can be on ANY physical port (this rig
		 * links on port 3, phy4), and a port forwards line->MAC only once its
		 * GMII interface is established - configuring only port 0 leaves the
		 * cabled port dark (stock sets STATIC=0xcb000200 on all ports).  Upper
		 * byte 0xCB is RO datapath-active status; writable fields only,
		 * idempotent. */
		for (p = 0; p < CA_NI_GPHY_COUNT; p++)
			ni_rmw(ni, CA_NI_PORT_STATIC_CFG(p),
			       CA_NI_PORT_STATIC_INT_CFG | CA_NI_PORT_STATIC_PHY_MODE |
			       CA_NI_PORT_STATIC_LPBK_MODE, 0);

		/* autosync = 0xF (match STOCK live-Linux; U-Boot's 0 was the
		 * wrong reference for the CPU-EPP RX path) */
		writel(CA_NI_NI_AUTOSYNC_STOCK_VAL,
		       ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC);
	}

	/* re-apply the FULL golden FE-path config: steer/demux/DLF + ES master
	 * (+RSVD25 clear) + cpu_en=0xff + RMU RX + port MAC RX (stock 0x3001) */
	cortina_ni_rx_steer_init(ni);
	cortina_ni_rx_es_enable(ni);
	cortina_ni_rx_es_cpu(ni, true);
	ni_rmw(ni, CA_NI_QM_RMU0_CTRL, 0, CA_NI_QM_RMU0_RX_EN);
	/* ★ power-up RX + enable RXMAC on EVERY GPHY LAN port (not just
	 * CA_NI_RX_PORT) - same reason as the GMII loop above, so the cabled port
	 * (any of 0..3) ingresses to the CPU. */
	{
		unsigned int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++) {
			ni_rmw(ni, CA_NI_PORT_GLB_CFG(p),
			       CA_NI_PORT_GLB_PWR_DWN_RX, 0);
			if (!rx_skip_portcfg)
				ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(p),
				       CA_NI_PORT_RXMAC_STOCK_CLR,
				       CA_NI_PORT_RXMAC_RX_EN |
				       CA_NI_PORT_RXMAC_STOCK_SET);
			else	/* leave U-Boot's rxmac bits, just ensure RX_EN on */
				ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(p), 0,
				       CA_NI_PORT_RXMAC_RX_EN);
		}
	}

	rx->rearms++;
	dev_info(ni->dev,
		 "RX (re)armed on link-up (#%llu): wrap_en1=0x%08x rxmac=0x%08x es_ctrl=0x%08x gphy_fault=0x%04x\n",
		 rx->rearms,
		 ni->win[CA_NI_WIN_GPHY_WRAP] ?
			readl(ni->win[CA_NI_WIN_GPHY_WRAP] + CA_NI_GPHY_WRAP_EN1) : 0,
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_QM_ES_CTRL),
		 cortina_ni_rx_gphy_fault(ni));

	/* ★ eth0 is the CPU<->switch port, NOT a single physical link.  Once the
	 * switch datapath is armed here - via adjust_link on a real link-up AND via
	 * the 1 Hz decoupled bring-up when the phylib-tracked PHY (port 0) is uncabled
	 * and never fires link-up - force eth0's carrier UP.  The Linux bridge disables
	 * (stops forwarding) a member port whose carrier is off, so leaving eth0's
	 * carrier tied to port 0's link makes br-lan drop every LAN frame the switch
	 * delivered (eth0 rx climbs, br-lan rx stays 0, host cannot reach the ONU).
	 * The phy_link_change override keeps phylib from re-clearing it; per-physical-
	 * port link/forwarding is handled by the switch HW itself. */
	if (rx->netdev && !netif_carrier_ok(rx->netdev))
		netif_carrier_on(rx->netdev);

	/* fault check now instead of waiting for the next 1 Hz tick */
	if (cortina_ni_rx_gphy(ni))
		mod_delayed_work(system_wq, &rx->recovery_work, 0);
}

/* ------------------------------------------------------------------ */
/* open/stop hooks (called from the netdev ops in cortina-ni-tx.c)     */
/* ------------------------------------------------------------------ */

void cortina_ni_rx_open(struct cortina_ni *ni)
{
	if (!ni->rx)
		return;		/* TX-only mode (RX probe failed/absent) */

	napi_enable(&ni->rx->napi);

	/* arm the CPU-port drain (stock enable_tx_cpu at ca_ni_open) so the ES
	 * scheduler starts writing descriptors for our CPU port */
	cortina_ni_rx_es_cpu(ni, true);

	/* port-0 MAC RX on (M2b left it off), stock pattern 0x3001:
	 * rx_en + bit12/bit13 set, bit8 CLEAR.  (DIAGNOSTIC: rx_skip_portcfg leaves
	 * U-Boot's working rxmac 0x1101 untouched to test the clobber hypothesis) */
	if (!rx_skip_portcfg)
		ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
		       CA_NI_PORT_RXMAC_STOCK_CLR,
		       CA_NI_PORT_RXMAC_RX_EN | CA_NI_PORT_RXMAC_STOCK_SET);
	else	/* leave U-Boot's rxmac bits, just ensure RX_EN on (-> 0x1101) */
		ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT), 0,
		       CA_NI_PORT_RXMAC_RX_EN);
	ni_rmw(ni, CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT),
	       CA_NI_PORT_GLB_PWR_DWN_RX, 0);

	cortina_ni_rx_irq_set(ni, true);

	/* start the 1 Hz GPHY fault poll (stock runs it for the device's
	 * whole lifetime; ours runs while the netdev is up) */
	if (cortina_ni_rx_gphy(ni))
		schedule_delayed_work(&ni->rx->recovery_work, HZ);
}

void cortina_ni_rx_stop(struct cortina_ni *ni)
{
	unsigned int i;

	if (!ni->rx)
		return;

	cancel_delayed_work_sync(&ni->rx->recovery_work);

	/* stop new ingress first, then quiesce the drain side.  Buffers
	 * already pushed to the HW pool CANNOT be popped back (no CPU pop
	 * primitive on this path) - they stay allocated for the device's
	 * lifetime and simply get reused on the next open. */
	ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
	       CA_NI_PORT_RXMAC_RX_EN, 0);
	cortina_ni_rx_es_cpu(ni, false);	/* disarm the CPU-port drain */
	cortina_ni_rx_irq_set(ni, false);
	napi_disable(&ni->rx->napi);

	/* NAPI is quiesced, so nothing can be mid-chain any more: release any
	 * partially assembled frame instead of carrying it across a close/open
	 * (the accounting would be reset by the next SOP, but the skb would not
	 * be - one per voq, held until the interface came back up). */
	for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++)
		cortina_ni_rx_chain_reset(ni->rx, &ni->rx->chain[i]);
}

/* ------------------------------------------------------------------ */
/* spy/dump hook: /proc/net/cortina_ni_rx (project rule: probes stay)  */
/* ------------------------------------------------------------------ */

/* Read one NI RX MIB counter for <port> via the indirect ACCESS/DATA0 pair
 * (stock __ni_eth_port_rx_mib_get).  Bounded poll; on timeout returns ~0 so a
 * stuck access is visible rather than silently zero.  Called from the /proc
 * reader and from `ethtool -S`, both process context - readl_poll_timeout may
 * sleep, so it must stay out of NAPI and out of any spinlock. */
u32 cortina_ni_rx_mib_read(struct cortina_ni *ni, u32 port, u32 cnt_id)
{
	u32 val;

	writel(CA_NI_MIB_ACCESS_GO |
	       FIELD_PREP(CA_NI_MIB_ACCESS_OPCODE, CA_NI_MIB_OP_READ_ONLY) |
	       FIELD_PREP(CA_NI_MIB_ACCESS_PORT, port) |
	       FIELD_PREP(CA_NI_MIB_ACCESS_CNTID, cnt_id),
	       ni_base(ni) + CA_NI_HV_RXMIB_ACCESS);
	if (readl_poll_timeout(ni_base(ni) + CA_NI_HV_RXMIB_ACCESS, val,
			       !(val & CA_NI_MIB_ACCESS_GO),
			       CA_NI_MIB_POLL_US, CA_NI_MIB_POLL_TIMEOUT_US))
		return ~0u;
	return readl(ni_base(ni) + CA_NI_HV_RXMIB_DATA0);
}

/* ------------------------------------------------------------------ */
/* values `ethtool -d` structurally cannot carry, published to -S       */
/* ------------------------------------------------------------------ */

/*
 * `ethtool -d` is a flat sweep of PLAIN readl()s - that is its stated
 * contract, and it is what makes a dump safe to take at any moment.  Three
 * kinds of value cannot ride it and therefore get an `ethtool -S` row of their
 * own, computed here where the access already lives:
 *
 *   INDIRECT   the central-buffer occupancy and per-port free-word counts are
 *              an ACCESS-write + DATA-read pair, and 128 of them for the VOQ
 *              scan.  Two readers racing on the ACCESS register get each
 *              other's index - a confident wrong number, not an error - so
 *              there must be exactly one path, and it is this one.
 *   DERIVED    the EPP write pointer is masked to a ring byte offset before it
 *              means anything; the raw word is a different quantity.
 *   MDIO       per-port PHY link is a bus transaction, not a register.
 *
 * All are called from `ethtool -S` (process context, may sleep).
 */

/* The EPP write pointer as a RING BYTE OFFSET - the same two masks the RX poll
 * applies, so the published value is the one the datapath acts on and not the
 * raw register word. */
u32 cortina_ni_rx_epp_wrptr(struct cortina_ni *ni, unsigned int voq)
{
	if (!ni_base(ni) || voq >= CA_NI_RX_VOQ_COUNT)
		return 0;
	return cortina_ni_rx_wptr_voq(ni, voq);
}

/*
 * Central-buffer occupancy, AGGREGATED here rather than published as 128 raw
 * rows: nobody differences a per-VOQ gauge, and three aggregates answer the
 * question the wedge signature is built from (pages held while the free pool
 * reads 0).  The pages are the register word's HIGH half.
 */
void cortina_ni_rx_cb_occupancy(struct cortina_ni *ni, u64 *total, u64 *max,
				u64 *nonzero)
{
	unsigned int q;

	*total = 0;
	*max = 0;
	*nonzero = 0;
	if (!ni_base(ni))
		return;

	for (q = 0; q < CA_NI_RX_CB_VOQ_ENTRIES; q++) {
		u32 c, pages;

		cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_VOQ_BUFCNT_ACCESS, q);
		c = readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_BUFCNT_DATA);
		if (!c)
			continue;
		pages = c >> CA_NI_RX_CB_VOQ_PAGES_SHIFT;
		*total += pages;
		if (pages > *max)
			*max = pages;
		(*nonzero)++;
	}
}

/* The per-port free-buffer count register WORD, undecoded and named as a word:
 * the packing is not proven on this silicon, so a row called "free pages"
 * would be naming it wrongly. */
u32 cortina_ni_rx_cb_port_free_word(struct cortina_ni *ni, unsigned int port)
{
	if (!ni_base(ni))
		return 0;
	cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, port);
	return readl(ni_base(ni) + CA_NI_L2TM_CB_PORT_FREECNT_DATA);
}

/*
 * Per-GPHY-port PHY link.  ★ THIS IS THE ONE ANSWER TO "WHICH PRINTED SOCKET
 * HAS THE CABLE", and this port has already paid for guessing it: the host
 * links on port 3, not port 0, and the panel order is MIRRORED.  The driver
 * registers ONE netdev bound to ONE phylib PHY, so /sys/class/net/<if>/carrier
 * and ethtool's own get_link can only ever speak for that one - every other
 * socket would be invisible.  Hence a per-port row.
 *
 * -1 when the bus read failed, so "the MDIO transaction did not complete" and
 * "the port has no link" stay different answers; a u64 stat carries it as
 * ~0ULL.
 */
int cortina_ni_rx_phy_link(struct cortina_ni *ni, unsigned int port)
{
	int addr, bmsr;

	if (!ni->mii || port >= CA_NI_GPHY_COUNT)
		return -1;
	addr = CA_NI_GPHY_FIRST + port;
	mdiobus_read(ni->mii, addr, MII_BMSR);		/* clear the latch */
	bmsr = mdiobus_read(ni->mii, addr, MII_BMSR);
	if (bmsr < 0)
		return -1;
	return !!(bmsr & BMSR_LSTATUS);
}

/*
 * Full curated NI-window register snapshot for a good-vs-bad-boot diff.
 * Every offset is < 0x10000 (inside the 64K NI window) and read with a
 * plain readl - no indirect/latching access, no side effects.  A single
 * `cat /proc/net/cortina_ni_rx` captures all of it; diff a working boot
 * (ping 5/5) against a broken one (wptr=0) and the register(s) that differ
 * are the gate.  For anything OUTSIDE this list or this window, use
 * /proc/cortina_ni_peek.
 */
static const struct {
	const char	*name;
	u32		off;
} cortina_ni_rx_regs[] = {
	/* NI_HV globals */
	{ "hv_init_done",	CA_NI_HV_INIT_DONE },
	{ "hv_intf_rst",	CA_NI_HV_INTF_RST },
	{ "hv_mac_autosync",	CA_NI_HV_MAC_AUTOSYNC },
	{ "hv_pkt_len",		CA_NI_HV_PKT_LEN },
	{ "hv_pkt_len_rx",	CA_NI_HV_PKT_LEN_RX },
	{ "hv_cfg_a1b8",	CA_NI_HV_CFG_A1B8 },
	{ "ni_intern_portid",	CA_NI_NI_INTERNAL_PORT_ID_CFG },
	{ "hv_cfg_a420",	CA_NI_HV_CFG_A420 },
	{ "hv_cfg_aaf0",	CA_NI_HV_CFG_AAF0 },
	/* L3FE demux golden routing map (FE output -> CPU-EPP) */
	{ "l3fe_demux0_a190",	CA_NI_NIRX_L3FE_DEMUX0 },
	{ "l3fe_demux1_a194",	CA_NI_NIRX_L3FE_DEMUX1 },
	{ "l3fe_demux2_a198",	CA_NI_NIRX_L3FE_DEMUX2 },
	{ "l3fe_demux3_a19c",	CA_NI_NIRX_L3FE_DEMUX3 },
	{ "l3fe_demux4_a1a0",	CA_NI_NIRX_L3FE_DEMUX4 },
	{ "l3fe_demux5_a1a4",	CA_NI_NIRX_L3FE_DEMUX5 },
	/* deep_q=1 demux (build34): ldpid 0x32 routing = dpq_48_63(0xa1a8) bits[5:4], want 0=L3QM */
	{ "dpq_demux_a1a8",	CA_NI_NIRX_L3FE_DPQ_DEMUX_48_63 },
	{ "dpq_demux_a1ac",	CA_NI_NIRX_L3FE_DPQ_DEMUX_32_47 },
	{ "dpq_demux_a1b0",	CA_NI_NIRX_L3FE_DPQ_DEMUX_16_31 },
	{ "dpq_demux_a1b4",	CA_NI_NIRX_L3FE_DPQ_DEMUX_0_15 },
	/* L2FE forwarding-control (stock 0x140c=0x0c100c10, 0x160c=0x1) */
	{ "l2fe_lrn_fwd1_140c",	CA_NI_L2FE_PLC_LRN_FWD_CTRL_1 },
	{ "l2fe_arb_ext_160c",	CA_NI_L2FE_ARB_CTRL_EXT },
	/* ★ __ni_flow_ctrl_init map (stock: 0x2124=0x88888888 BM dq->TM-port8=QM,
	 * 0x3400=0x001c787c, 0x9798=0x81a80178). If 0x2124!=0x88888888 the BM
	 * dequeues CPU frames to the wrong port -> qm_rx=0. */
	{ "bm_dq_port_map_2124", CA_NI_L2TM_BM_DQ_PORT_MAP },
	{ "ni_flowctrl_en_3400", CA_NI_NI_FLOWCTRL_EN },
	{ "ni_flowthr0_9798",	CA_NI_NI_FLOWCTRL_THRESH },
	/* port-0 MAC block */
	{ "p0_glb",		CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT) },
	{ "p0_rxmac",		CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT) },
	{ "p0_txmac",		CA_NI_PORT_TXMAC_CFG(CA_NI_RX_PORT) },
	{ "p0_rx_cntrl",	CA_NI_PORT_RX_CNTRL_CFG(CA_NI_RX_PORT) },
	/* L2TM (TX-side scheduler/QM, dumped for completeness) */
	{ "l2tm_qm_eq_cfg",	CA_NI_L2TM_QM_EQ_CFG },
	{ "l2tm_qm_glob_buf",	CA_NI_L2TM_QM_GLOB_BUF_CFG },
	{ "l2tm_qm_prvt_prof0",	CA_NI_L2TM_QM_PORT_PRVT_PROF0 },
	{ "l2tm_es_ctrl",	CA_NI_L2TM_ES_CTRL },
	{ "l2tm_es_sch0",	CA_NI_L2TM_ES_SCH_CFG(0) },
	/* L3QM / RMU (RX drain path) */
	{ "qm_rmu0_ctrl",	CA_NI_QM_RMU0_CTRL },
	{ "qm_es_ctrl_6108",	CA_NI_QM_ES_CTRL },
	{ "qm_l3tm_ni_ena",	CA_NI_QM_L3TM_NI_PORT_ENA },	/* 0x610c */
	{ "qm_int_en0",		CA_NI_QM_EPP64_INT_EN0 },
	{ "qm_int_en1",		CA_NI_QM_EPP64_INT_EN1 },
	{ "qm_eq_profile13",	CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE) },  /* {eqp0=13,eqp1=14} */
	{ "qm_eq14_cfg1",	CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID2) },
	{ "qm_eq14_pa_req",	CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2) },
	{ "qm_es_ctrl2",	CA_NI_QM_ES_CTRL2 },
	{ "qm_epp_cmd",		CA_NI_QM_EPP },
	{ "qm_destp0_eq_cfg",	CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT) },
	{ "qm_destp0_pkt_buf",	CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT) },
	{ "qm_eq13_cfg0",	CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_cfg1",	CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_cfg2",	CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_cfg3",	CA_NI_QM_CFG3_EQ(CA_NI_RX_EQ_ID) },
	{ "qm_eq13_pa_req",	CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID) },
	{ "qm_push_ready0",	CA_NI_QM_CPU_PUSH_READY(CA_NI_RX_CPU_PORT) },
	{ "qm_voq_en0",		CA_NI_QM_VOQ_EN(CA_NI_RX_CPU_PORT) },
	{ "qm_eq_cfg_load",	CA_NI_QM_EQ_CFG_LOAD },
	{ "qm_hdm_wr_prot",	CA_NI_QM_HDM_WRITE_PROT },
	{ "qm_rx_status0",	CA_NI_QM_RX_STATUS0 },
	{ "qm_rx_status1",	CA_NI_QM_RX_STATUS1 },
	{ "qm_tx_cntr",		CA_NI_QM_TX_CNTR },
	{ "qm_rx_cntr",		CA_NI_QM_RX_CNTR },
	/* CPU-EPP FIFO config + ring pointers (port0/voq0) */
	{ "qm_cpu_epp_cfg0",	CA_NI_QM_CPU_EPP_CFG(CA_NI_RX_CPU_PORT) },
	{ "qm_cpu_epp_ct",	CA_NI_QM_CPU_EPP_CT_CFG },
	{ "qm_epp_fifo_prof0_66a4",	CA_NI_QM_CPU_EPP_FIFO_PROF(0) },	/* stock 0xE0008001 */
	{ "qm_epp_fifo_prof4",	CA_NI_QM_CPU_EPP_FIFO_PROF(CA_NI_RX_PROFILE_ID) },
	{ "qm_epp_fifo_cfg0",	CA_NI_QM_CPU_EPP_FIFO_CFG(CA_NI_RX_CPU_PORT,
							 CA_NI_RX_VOQ) },
	{ "qm_epp_wrptr0",	CA_NI_QM_EPP64_WRPTR(CA_NI_RX_CPU_PORT,
						     CA_NI_RX_VOQ) },
	{ "qm_epp_rdptr0",	CA_NI_QM_EPP64_RDPTR(CA_NI_RX_CPU_PORT,
						     CA_NI_RX_VOQ) },
	{ "qm_epp_paddr0",	CA_NI_QM_EPP64_PADDR_START(CA_NI_RX_CPU_PORT,
							   CA_NI_RX_VOQ) },
};

static void cortina_ni_rx_dump_regs(struct seq_file *m, struct cortina_ni *ni)
{
	int i;

	seq_puts(m, "regs:\n");
	for (i = 0; i < ARRAY_SIZE(cortina_ni_rx_regs); i++)
		seq_printf(m, "  %-20s ni+0x%04x = 0x%08x\n",
			   cortina_ni_rx_regs[i].name,
			   cortina_ni_rx_regs[i].off,
			   readl(ni_base(ni) + cortina_ni_rx_regs[i].off));

	/* Per-port-0 MAC block (0xa5c4..0xa630): the GMAC-level mac_rx=0 gate is a
	 * per-port config we set only 4 of - dump the mapped ones so a good-vs-bad
	 * diff finds the interface/media/enable we skip.  NOTE: 0xa634..0xa64c is an
	 * unmapped hole on this SoC (readl faults -> synchronous external abort), so
	 * stop at 0xa630 - never widen this past 0x70. */
	seq_puts(m, "port0 block (0xa5c0..0xa630):\n");
	/* ★ 0xa5c0 = STATIC_CFG (MAC<->PHY interface: int_cfg[3:0]/phy_mode[4]/
	 * lpbk[13:12]) - the true port-block base, one word below GLB_CFG, and
	 * the bidirectional datapath gate our driver used to skip */
	seq_printf(m, "  p0-0x04   ni+0x%04x = 0x%08x  <- STATIC_CFG (int_cfg/phy_mode/lpbk)\n",
		   CA_NI_PORT_STATIC_CFG(CA_NI_RX_PORT),
		   readl(ni_base(ni) + CA_NI_PORT_STATIC_CFG(CA_NI_RX_PORT)));
	for (i = 0; i < 0x70; i += 4) {
		u32 off = CA_NI_PORT_GLB_CFG(CA_NI_RX_PORT) + i;

		seq_printf(m, "  p0+0x%02x   ni+0x%04x = 0x%08x\n",
			   i, off, readl(ni_base(ni) + off));
	}

	/* the port-0 GPHY block (bank 0): fault latch + BMCR/BMSR + link */
	if (cortina_ni_rx_gphy(ni)) {
		void __iomem *gphy = cortina_ni_rx_gphy(ni);

		seq_printf(m, "  %-20s gphy+0x%05x = 0x%08x\n", "gphy_fault",
			   CA_NI_GPHY_FAULT, readl(gphy + CA_NI_GPHY_FAULT));
		seq_printf(m, "  %-20s gphy+0x%05x = 0x%08x\n", "gphy_bmcr",
			   CA_NI_GPHY_BMCR, readl(gphy + CA_NI_GPHY_BMCR));
		seq_printf(m, "  %-20s gphy+0x%05x = 0x%08x\n", "gphy_bmsr",
			   CA_NI_GPHY_BMCR + CA_NI_GPHY_REG_STRIDE,
			   readl(gphy + CA_NI_GPHY_BMCR + CA_NI_GPHY_REG_STRIDE));
	}

	/* ★ GPHY wrapper: EN1 bit12 (patch_phy_done) is the ingress determinism
	 * gate; also the analog/datapath regs w+00..0c (golden 0x3110/0x10ff0/
	 * 0x10bc06/0x0800a400) for a good-vs-bad-boot diff */
	if (ni->win[CA_NI_WIN_GPHY_WRAP]) {
		void __iomem *w = ni->win[CA_NI_WIN_GPHY_WRAP];
		static const struct { const char *n; u32 off; } wr[] = {
			{ "wrap_r00", CA_NI_GPHY_WRAP_R00 },
			{ "wrap_r04", CA_NI_GPHY_WRAP_R04 },
			{ "wrap_r08", CA_NI_GPHY_WRAP_R08 },
			{ "wrap_r0c", CA_NI_GPHY_WRAP_R0C },
			{ "wrap_en0", CA_NI_GPHY_WRAP_EN0 },
			{ "wrap_en1", CA_NI_GPHY_WRAP_EN1 },
		};

		for (i = 0; i < ARRAY_SIZE(wr); i++)
			seq_printf(m, "  %-20s wrap+0x%02x = 0x%08x\n",
				   wr[i].n, wr[i].off, readl(w + wr[i].off));
	}
}

/* ★ QM+L2TM full-block offset sweep (ours-vs-stock diff hunt for the L2TM->QM
 * admit gate).  These are the ONLY mapped offsets in the NI window's QM/L2TM
 * region (proven by a stock devmem sweep - readl is SAFE); an offset OUTSIDE
 * this list is an unmapped hole that would fault.  Read-only. */
static const u16 cortina_ni_qmdump_offs[] = {
	0x2000, 0x2004, 0x2008, 0x200c, 0x2010, 0x2014, 0x2018, 0x201c, 0x2020, 0x2024, 0x2100, 0x2104,
	0x2108, 0x210c, 0x2110, 0x2114, 0x2118, 0x211c, 0x2120, 0x2124, 0x2128, 0x212c, 0x2130, 0x2134,
	0x2138, 0x213c, 0x2140, 0x2144, 0x2148, 0x214c, 0x2150, 0x2154, 0x2158, 0x215c, 0x2160, 0x2164,
	0x2168, 0x216c, 0x2170, 0x2174, 0x2178, 0x217c, 0x2180, 0x2184, 0x2188, 0x218c, 0x2190, 0x2194,
	0x2198, 0x219c, 0x21a0, 0x21a4, 0x21a8, 0x21ac, 0x21b0, 0x21b4, 0x2200, 0x2204, 0x2208, 0x220c,
	0x2210, 0x2214, 0x2218, 0x221c, 0x2220, 0x2224, 0x2228, 0x222c, 0x2230, 0x2234, 0x2238, 0x223c,
	0x2240, 0x2244, 0x2248, 0x224c, 0x2250, 0x2254, 0x2258, 0x225c, 0x2260, 0x2264, 0x2268, 0x226c,
	0x2270, 0x2274, 0x2278, 0x227c, 0x2280, 0x2284, 0x2288, 0x228c, 0x2290, 0x2294, 0x2298, 0x229c,
	0x22a0, 0x22a4, 0x22a8, 0x2300, 0x2304, 0x2308, 0x230c, 0x2310, 0x2314, 0x2318, 0x231c, 0x2320,
	0x2324, 0x2328, 0x232c, 0x2330, 0x2334, 0x2338, 0x233c, 0x2340, 0x2344, 0x2348, 0x234c, 0x2350,
	0x2354, 0x2358, 0x235c, 0x2360, 0x2364, 0x2368, 0x236c, 0x2370, 0x2374, 0x2378, 0x237c, 0x2380,
	0x2384, 0x2388, 0x238c, 0x2390, 0x2394, 0x2398, 0x239c, 0x23a0, 0x23a4, 0x23a8, 0x23ac, 0x23b0,
	0x23b4, 0x23b8, 0x23bc, 0x23c0, 0x23c4, 0x23c8, 0x23cc, 0x23d0, 0x23d4, 0x23d8, 0x23dc, 0x23e0,
	0x23e4, 0x23e8, 0x23ec, 0x23f0, 0x23f4, 0x23f8, 0x23fc, 0x6000, 0x6004, 0x6008, 0x600c, 0x6100,
	0x6104, 0x6108, 0x610c, 0x6110, 0x6114, 0x6118, 0x611c, 0x6120, 0x6124, 0x6128, 0x612c, 0x6130,
	0x6134, 0x6138, 0x613c, 0x6140, 0x6144, 0x6148, 0x614c, 0x6150, 0x6154, 0x6158, 0x615c, 0x6160,
	0x6164, 0x6168, 0x616c, 0x6170, 0x6174, 0x6178, 0x617c, 0x6180, 0x6184, 0x6188, 0x618c, 0x6190,
	0x6194, 0x6198, 0x619c, 0x61a0, 0x61a4, 0x61a8, 0x61ac, 0x61b0, 0x61b4, 0x61b8, 0x61bc, 0x61c0,
	0x61c4, 0x61c8, 0x61cc, 0x61d0, 0x61d4, 0x61d8, 0x61dc, 0x61e0, 0x61e4, 0x61e8, 0x61ec, 0x61f0,
	0x61f4, 0x61f8, 0x61fc, 0x6200, 0x6204, 0x6208, 0x620c, 0x6210, 0x6214, 0x6218, 0x621c, 0x6220,
	0x6224, 0x6228, 0x622c, 0x6230, 0x6234, 0x6238, 0x623c, 0x6240, 0x6244, 0x6248, 0x624c, 0x6250,
	0x6254, 0x6258, 0x625c, 0x6260, 0x6264, 0x6268, 0x626c, 0x6270, 0x6274, 0x6278, 0x627c, 0x6280,
	0x6284, 0x6288, 0x628c, 0x6290, 0x6294, 0x6298, 0x629c, 0x62a0, 0x62a4, 0x62a8, 0x62ac, 0x62b0,
	0x62b4, 0x62b8, 0x62bc, 0x62c0, 0x62c4, 0x62c8, 0x62cc, 0x62d0, 0x62d4, 0x62d8, 0x62dc, 0x62e0,
	0x62e4, 0x62e8, 0x62ec, 0x62f0, 0x62f4, 0x62f8, 0x62fc, 0x6300, 0x6304, 0x6308, 0x630c, 0x6310,
	0x6314, 0x6318, 0x631c, 0x6320, 0x6324, 0x6328, 0x632c, 0x6330, 0x6334, 0x6338, 0x633c, 0x6340,
	0x6344, 0x6348, 0x634c, 0x6350, 0x6354, 0x6358, 0x635c, 0x6360, 0x6364, 0x6368, 0x636c, 0x6370,
	0x6374, 0x6378, 0x637c, 0x6380, 0x6384, 0x6388, 0x638c, 0x6390, 0x6394, 0x6398, 0x639c, 0x63a0,
	0x63a4, 0x63a8, 0x63ac, 0x63b0, 0x63b4, 0x63b8, 0x63bc, 0x63c0, 0x63c4, 0x63c8, 0x63cc, 0x63d0,
	0x63d4, 0x63d8, 0x63dc, 0x63e0, 0x63e4, 0x63e8, 0x63ec, 0x63f0, 0x63f4, 0x63f8, 0x63fc, 0x6400,
	0x6404, 0x6408, 0x640c, 0x6410, 0x6414, 0x6418, 0x641c, 0x6420, 0x6424, 0x6428, 0x642c, 0x6430,
	0x6434, 0x6438, 0x643c, 0x6440, 0x6444, 0x6448, 0x644c, 0x6450, 0x6454, 0x6458, 0x645c, 0x6460,
	0x6464, 0x6468, 0x646c, 0x6470, 0x6474, 0x6478, 0x647c, 0x6480, 0x6484, 0x6488, 0x648c, 0x6490,
	0x6494, 0x6498, 0x649c, 0x64a0, 0x64a4, 0x64a8, 0x64ac, 0x64b0, 0x64b4, 0x64b8, 0x64bc, 0x64c0,
	0x64c4, 0x64c8, 0x64cc, 0x64d0, 0x64d4, 0x64d8, 0x64dc, 0x64e0, 0x64e4, 0x64e8, 0x64ec, 0x64f0,
	0x64f4, 0x64f8, 0x64fc, 0x6500, 0x6504, 0x6508, 0x650c, 0x6510, 0x6514, 0x6518, 0x651c, 0x6520,
	0x6524, 0x6528, 0x652c, 0x6530, 0x6534, 0x6538, 0x653c, 0x6540, 0x6544, 0x6548, 0x654c, 0x6550,
	0x6554, 0x6558, 0x655c, 0x6560, 0x6564, 0x6568, 0x656c, 0x6570, 0x6574, 0x6578, 0x657c, 0x6580,
	0x6584, 0x6588, 0x658c, 0x6590, 0x6594, 0x6598, 0x659c, 0x65a0, 0x65a4, 0x65a8, 0x65ac, 0x65b0,
	0x65b4, 0x65b8, 0x65bc, 0x65c0, 0x65c4, 0x65c8, 0x65cc, 0x65d0, 0x65d4, 0x65d8, 0x65dc, 0x65e0,
	0x65e4, 0x65e8, 0x65ec, 0x65f0, 0x65f4, 0x65f8, 0x65fc, 0x6600, 0x6604, 0x6608, 0x660c, 0x6610,
	0x6614, 0x6618, 0x661c, 0x6620, 0x6624, 0x6628, 0x662c, 0x6630, 0x6634, 0x6638, 0x663c, 0x6640,
	0x6644, 0x6648, 0x664c, 0x6650, 0x6654, 0x6658, 0x665c, 0x6660, 0x6664, 0x6668, 0x666c, 0x6670,
	0x6674, 0x6678, 0x667c, 0x6680, 0x6684, 0x6688, 0x668c, 0x6690, 0x6694, 0x6698, 0x669c, 0x66a0,
	0x66a4, 0x66a8, 0x66ac, 0x66b0, 0x66b4, 0x66b8, 0x66bc, 0x66c0, 0x66c4, 0x66c8, 0x66cc, 0x66d0,
	0x66d4, 0x66d8, 0x66dc, 0x66e0, 0x66e4, 0x66e8, 0x66ec, 0x66f0, 0x66f4, 0x66f8, 0x66fc, 0x6700,
	0x6704, 0x6708, 0x670c, 0x6710, 0x6714, 0x6718, 0x671c, 0x6720, 0x6724, 0x6728, 0x672c, 0x6730,
	0x6734, 0x6738, 0x673c, 0x6740, 0x6744, 0x6748, 0x674c, 0x6750, 0x6754, 0x6758, 0x675c, 0x6760,
	0x6764, 0x6768, 0x676c, 0x6770, 0x6774, 0x6778, 0x677c, 0x6780, 0x6784, 0x6788, 0x678c, 0x6790,
	0x6794, 0x6798, 0x679c, 0x67a0, 0x67a4, 0x67a8, 0x67ac, 0x67b0, 0x67b4, 0x67b8, 0x67bc, 0x67c0,
	0x67c4, 0x67c8, 0x67cc, 0x67d0, 0x67d4, 0x67d8, 0x67dc, 0x67e0, 0x67e4, 0x67e8, 0x67ec, 0x67f0,
	0x67f4, 0x67f8, 0x67fc, 0x6800, 0x6804, 0x6808, 0x680c, 0x6810, 0x6814, 0x6818, 0x681c, 0x6820,
	0x6824, 0x6828, 0x682c, 0x6830, 0x6834, 0x6838, 0x683c, 0x6840, 0x6844, 0x6848, 0x684c, 0x6850,
	0x6854, 0x6858, 0x685c, 0x6860, 0x6864, 0x6868, 0x686c, 0x6870, 0x6874, 0x6878, 0x687c, 0x6880,
	0x6884, 0x6888, 0x688c, 0x6890, 0x6894, 0x6898, 0x689c, 0x68a0, 0x68a4, 0x68a8, 0x68ac, 0x68b0,
	0x68b4, 0x68b8, 0x68bc, 0x68c0, 0x68c4, 0x68c8, 0x68cc, 0x68d0, 0x68d4, 0x68d8, 0x68dc, 0x68e0,
	0x68e4, 0x68e8, 0x68ec, 0x68f0, 0x68f4, 0x68f8, 0x68fc, 0x6900, 0x6904, 0x6908, 0x690c, 0x6910,
	0x6914, 0x6918, 0x691c, 0x6920, 0x6924, 0x6928, 0x692c, 0x6930, 0x6934, 0x6938, 0x693c, 0x6940,
	0x6944, 0x6948, 0x694c, 0x6950, 0x6954, 0x6958, 0x695c, 0x6960, 0x6964, 0x6968, 0x696c, 0x6970,
	0x6974, 0x6978, 0x697c, 0x6980, 0x6984, 0x6988, 0x698c, 0x6990, 0x6994, 0x6998, 0x699c, 0x69a0,
	0x69a4, 0x69a8, 0x69ac, 0x69b0, 0x69b4, 0x69b8, 0x69bc, 0x69c0, 0x69c4, 0x69c8, 0x69cc, 0x69d0,
	0x69d4, 0x69d8, 0x69dc, 0x69e0, 0x69e4, 0x69e8, 0x69ec, 0x69f0, 0x69f4, 0x69f8, 0x69fc, 0x6a00,
	0x6a04, 0x6a08, 0x6a0c, 0x6a10, 0x6a14, 0x6a18, 0x6a1c, 0x6a20, 0x6a24, 0x6a28, 0x6a2c, 0x6a30,
	0x6a34, 0x6a38, 0x6a3c, 0x6a40, 0x6a44, 0x6a48, 0x6a4c, 0x6a50, 0x6a54, 0x6a58, 0x6a5c, 0x6a60,
	0x6a64, 0x6a68, 0x6a6c, 0x6a70, 0x6a74, 0x6a78, 0x6a7c, 0x6a80, 0x6a84, 0x6a88, 0x6a8c, 0x6a90,
	0x6a94, 0x6a98, 0x6a9c, 0x6aa0, 0x6aa4, 0x6aa8, 0x6aac, 0x6ab0, 0x6ab4, 0x6ab8, 0x6abc, 0x6ac0,
	0x6ac4, 0x6ac8,
	/* ★ CPU-EPP ring pointer block (wptr 0x7000 = THE drain-alive discriminator:
	 * stock advances, ours stuck 0) + per-voq ring config, so the qmblock dump
	 * shows the EPP delivery stage next to the QM drain-map. */
	0x7000, 0x7004, 0x7008, 0x700c, 0x7010, 0x7014, 0x7018, 0x701c,
	0x7020, 0x7024, 0x7028, 0x702c,
};

/* ------------------------------------------------------------------ */
/* `ethtool -d`: the same curated snapshot, through a standard interface */
/* ------------------------------------------------------------------ */

/*
 * The two tables above ARE the register snapshot, and they stay here - beside
 * the /proc reader that also prints them - so there is exactly ONE list.  A
 * copy in the ethtool file would be a second thing to keep in sync, and this
 * tree has already paid for a duplicated table that drifted.
 *
 * Word order: the curated named registers first, then the QM/L2TM block
 * sweep.  Both are plain readl of NI-window offsets that a stock devmem sweep
 * proved mapped - no indirect access, no latch, no side effect - so a dump is
 * safe to take at any time and taking one cannot perturb what it measures.
 *
 * The order is the ABI of `ethtool -d`: appending is fine, reordering or
 * removing is not without bumping CA_NI_REGDUMP_VERSION.
 */
unsigned int cortina_ni_regdump_len(void)
{
	return ARRAY_SIZE(cortina_ni_rx_regs) +
	       ARRAY_SIZE(cortina_ni_qmdump_offs);
}

void cortina_ni_regdump_fill(struct cortina_ni *ni, u32 *buf)
{
	unsigned int i, n = 0;

	if (!ni_base(ni)) {
		memset(buf, 0, cortina_ni_regdump_len() * sizeof(*buf));
		return;
	}
	for (i = 0; i < ARRAY_SIZE(cortina_ni_rx_regs); i++)
		buf[n++] = readl(ni_base(ni) + cortina_ni_rx_regs[i].off);
	for (i = 0; i < ARRAY_SIZE(cortina_ni_qmdump_offs); i++)
		buf[n++] = readl(ni_base(ni) + cortina_ni_qmdump_offs[i]);
}

/* Decode key for word @i of the dump above: its name and its NI-window offset.
 * Generated from the SAME tables as the dump, so the map cannot describe a
 * different snapshot than the one taken.  The sweep half has no vendor name,
 * so it is identified by its offset alone and says so. */
void cortina_ni_regdump_entry(unsigned int i, const char **name, u32 *off)
{
	if (i < ARRAY_SIZE(cortina_ni_rx_regs)) {
		*name = cortina_ni_rx_regs[i].name;
		*off = cortina_ni_rx_regs[i].off;
		return;
	}
	i -= ARRAY_SIZE(cortina_ni_rx_regs);
	if (i < ARRAY_SIZE(cortina_ni_qmdump_offs)) {
		*name = "qm_l2tm_sweep";
		*off = cortina_ni_qmdump_offs[i];
		return;
	}
	*name = "<out of range>";
	*off = 0;
}

/*
 * ★ BOTH DIRECTIONS' CPU-forward witness, in ONE read.
 *
 * `data_enq` (tx->pon_data_enq) counts ONLY the UPSTREAM leg: LAN->WAN frames the
 * CPU enqueued to the PON TX.  Reading it alone and concluding "the CPU-forward
 * counter stayed flat, so the flow was HW-offloaded" is wrong by construction - it
 * says nothing about the DOWNSTREAM direction, which is CPU-punted through an
 * entirely different counter.  The downstream complement is on the RX side:
 *   wan_l3  (rx->wan_l3_frames) - DS frames the HW-L3 engine MISSED and punted to
 *                                 the CPU (lspid = L3_WAN), delivered to the WAN
 *                                 netdev.  This is the DS "software forwarded"
 *                                 counter, the true mirror of us data_enq.
 *   wan_pon (rx->wan_frames)    - DS frames arriving with lspid = PON: terminating
 *                                 traffic (DHCP/ICMP to the router itself) plus
 *                                 every DS frame when HW-L3 forwarding is off.  Not
 *                                 all of it is transit, so it is reported SEPARATELY
 *                                 rather than folded into one number.
 * Printed identically into /proc/net/cortina_ni_rx AND /proc/net/cortina_ni_tx so
 * either file alone answers "was EITHER direction on the CPU during this run".
 */
void cortina_ni_cpu_fwd_show(struct seq_file *m, struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx = ni->rx;
	struct cortina_ni_tx *tx = ni->tx;

	seq_printf(m,
		   "cpu_fwd: us_data_enq=%llu [UPSTREAM only: LAN->WAN frames the CPU enqueued to PON TX] ds_wan_l3=%llu [DOWNSTREAM: HW-L3 miss punted to CPU] ds_wan_pon=%llu [DOWNSTREAM: lspid=PON, incl. terminating traffic] -- BOTH directions must stay FLAT to claim HW offload; us_data_enq alone proves nothing about DS\n",
		   tx ? tx->pon_data_enq : 0ULL,
		   rx ? rx->wan_l3_frames : 0ULL,
		   rx ? rx->wan_frames : 0ULL);
}

/*
 * The RX-side narrative dump.  ★ IT IS DEBUGFS NOW, NOT /proc (2026-08-08).
 *
 * It used to be /proc/net/cortina_ni_rx -- a node named after ONE driver on ONE
 * model, which the vendor firmware has under no name.  A test reading it could
 * therefore only ever BLOCK when run against stock, so the ORACLE half of every
 * "compared against stock" datapath claim taken through it was structurally
 * impossible.  Every countable VALUE moved to `ethtool -S` and the register
 * snapshot to `ethtool -d`, both of which stock's kernel serves too.
 *
 * What is left here is what a HUMAN reads while debugging and no standard
 * interface can carry: the annotated bisect narrative with its RE'd
 * stock-expected `want ...` values, the L2FE/PDPID/MC-FIB table read-backs, the
 * axi_reo and fbm window dumps.  Those are facts recorded nowhere else in the
 * tree, which is exactly why this function is MOVED and not deleted - the
 * project rule is that the spy/dump capability is never stripped; only where it
 * is exposed changed.
 *
 * ⚠ AND NO TEST MAY READ IT.  That is the whole safety of allowing a
 * driver-named debugfs directory: debugfs is root-only, absent when
 * CONFIG_DEBUG_FS is off, frequently unmounted, and explicitly not a stable ABI.
 * A case reading it would re-create the same defect one directory over.
 */
int cortina_ni_rx_debug_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_rx *rx = ni->rx;
	u32 pa_req;
	int i;
	/* ★★ THE CLEAR-ON-READ NI_HV COUNTERS ARE NOT READ HERE ANY MORE.
	 * They are read-and-clear (stock's own `ca-ne.ko` labels the block "NI counter
	 * ===== (read-and-clear)"), and this ONE show() used to read each of them TWICE:
	 * l3qm at the fwd-chain line and again at ni_hv-rx, l3fe likewise.  The first read
	 * took the count and every later one printed a structural ~0 - a value that was
	 * then quoted as evidence in several places in this tree.
	 * (Fixed 2026-08-05.  The l3qm half was found by an offline verification pass; the
	 * l3fe half was the same defect one line up, which that pass had not looked at.)
	 * ⇒ THE SAME DEFECT, ONE FILE WIDER: /proc/cortina_l3fe reads the same two
	 * registers, and `ethtool -S` now publishes them too, so "read it once in this
	 * function" was never enough - two files cat'ed a second apart steal from each
	 * other exactly as two lines of one file did.  The ONE reader is now
	 * cortina_ni_nihv_sample(); it folds each sample into a cumulative driver-side
	 * total and every consumer prints THAT.  Never readl() 0xa9bc/0xa9fc/0xaa10/
	 * 0xaa3c/0xaa7c from anywhere again.
	 * The values printed below are therefore TOTALS SINCE BOOT, not the raw
	 * since-last-read deltas the old lines showed - which is what a counter quoted
	 * as evidence should have been all along. */
	u64 nihv[CA_NI_NIHV_CNT_COUNT];
	u64 l3fe_rx, l3qm_rx;

	cortina_ni_nihv_sample(ni, nihv);
	l3fe_rx = nihv[CA_NI_NIHV_L3FE_RX];
	l3qm_rx = nihv[CA_NI_NIHV_L3QM_RX];

	/* RX fell back to TX-only (pool never came up): rx is gone, but dump the
	 * QM/pool registers so the failure is debuggable live */
	if (!rx) {
		u32 pr8 = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID));
		u32 pr9 = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2));
		u32 sts = readl(ni_base(ni) + CA_NI_QM_PHY_PORT_STS);

		seq_printf(m, "mode=tx-only (RX pool never came up)\n");
		seq_printf(m, "qm_phy_sts=0x%08x qm_init_done(phantom)=%u | l3qm_sts(0x6988)=0x%08x init_done(b30)=%u [REAL, want 1]\n",
			   sts, !!(sts & CA_NI_QM_INIT_DONE),
			   readl(ni_base(ni) + CA_NI_QM_L3QM_STS),
			   !!(readl(ni_base(ni) + CA_NI_QM_L3QM_STS) & CA_NI_QM_L3QM_INIT_DONE));
		seq_printf(m, "es_ctrl=0x%08x es_ctrl2=0x%08x rmu0=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_QM_ES_CTRL),
			   readl(ni_base(ni) + CA_NI_QM_ES_CTRL2),
			   readl(ni_base(ni) + CA_NI_QM_RMU0_CTRL));
		seq_printf(m, "eq13 cfg0/1/2=0x%08x/0x%08x/0x%08x pa_req=0x%08x (req=%u inact=%lu)\n",
			   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID)),
			   pr8, !!(pr8 & CA_NI_QM_PA_REQ_READY),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pr8));
		seq_printf(m, "eq14 cfg1=0x%08x pa_req=0x%08x (req=%u inact=%lu)\n",
			   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID2)),
			   pr9, !!(pr9 & CA_NI_QM_PA_REQ_READY),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pr9));
		seq_printf(m, "push_rdy0=0x%08x profile4=0x%08x destp8_eq_cfg=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_QM_CPU_PUSH_READY(CA_NI_RX_CPU_PORT)),
			   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
			   readl(ni_base(ni) +
				 CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)));
		seq_printf(m, "epp wptr=0x%06x rdptr=0x%06x cmd_mode=%s demux_sel=0x%x\n",
			   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_WRPTR(0, 0)) &
				 CA_NI_QM_EPP64_PTR),
			   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_RDPTR(0, 0)) &
				 CA_NI_QM_EPP64_PTR),
			   (readl(ni_base(ni) + CA_NI_QM_EPP) & CA_NI_QM_EPP_CMD_MODE_64) ?
			   "64b" : "32b",
			   (unsigned int)(readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG) &
					  CA_NI_NI_L3QMRX_DEMUX_SEL_ALL));
		return 0;
	}

	seq_printf(m, "mode=fe-path ring @%pad rptr[0]=0x%03x (8 voqs)\n",
		   &rx->ring_dma, rx->rptr[0]);
	seq_printf(m, "hw wptr=0x%06x rdptr=0x%06x paddr_start=0x%08x\n",
		   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_WRPTR(0, 0)) &
			 CA_NI_QM_EPP64_PTR),
		   (u32)(readl(ni_base(ni) + CA_NI_QM_EPP64_RDPTR(0, 0)) &
			 CA_NI_QM_EPP64_PTR),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_PADDR_START(0, 0)));
	seq_printf(m, "rx_cntrl=0x%08x rxmac=0x%08x int_en=0x%08x/0x%08x\n",
		   readl(ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(CA_NI_RX_PORT)),
		   readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT)),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN0),
		   readl(ni_base(ni) + CA_NI_QM_EPP64_INT_EN1));
	/* QM egress-scheduler drain gate + MAC RX MIB: if es_ctrl has tx_en +
	 * our cpu_en set and mac_rx_{uc,mc,bc} climb while hw wptr stays 0, the
	 * fault is downstream of the MAC (ES/steer/ring); if the MIB stays 0,
	 * frames never reach the port MAC (link/steer/wire). */
	seq_printf(m, "es_ctrl=0x%08x\n", readl(ni_base(ni) + CA_NI_QM_ES_CTRL));
	/* DIAGNOSTIC: read the RX MIB for ALL 4 ports - if the host's frames land
	 * on a port != CA_NI_RX_PORT, our port<->GPHY mapping assumption is wrong */
	{
		int p;
		for (p = 0; p < 4; p++)
			seq_printf(m, "mac_rx_p%d: uc=%u mc=%u bc=%u\n", p,
				   cortina_ni_rx_mib_read(ni, p, CA_NI_MIB_RX_UC_PKT),
				   cortina_ni_rx_mib_read(ni, p, CA_NI_MIB_RX_MC_PKT),
				   cortina_ni_rx_mib_read(ni, p, CA_NI_MIB_RX_BC_PKT));
	}
	/* ★ DATAPATH BISECT (real counters, unlike the phantom MAC MIB): the
	 * FIRST stage that stays 0 after a ping while the prior increments = the
	 * death point.  L2FE ingest -> L2FE drop -> L2FE->TM forward -> QM RMU
	 * ingest -> QM drops -> (epp wptr, below). */
	{
		/* l2fe_ni_pkt = {sop[31:16], eop[15:0]}: sop==eop on stock (clean
		 * frames); ours diverges = looping/fragmented frames.  l2fe_tm_fwd
		 * is the same {sop,eop} form.  (qm_rmu_rx 0x67d8 dropped - phantom,
		 * reads 0 even on working stock.) */
		u32 nipkt = readl(ni_base(ni) + CA_NI_L2FE_NI_INTF_PKT_CNT);
		u32 tmfwd = readl(ni_base(ni) + CA_NI_L2FE_PE_TM_PKT_CNT);

		seq_printf(m,
			   "bisect: l2fe_ni sop=%u eop=%u l2fe_ni_drop=%u l2fe_dos=%u l2fe_tm sop=%u eop=%u qm_eop_drop=%u qm_len_err=%u qm_l2te_drop=%u\n",
			   nipkt >> 16, nipkt & 0xffff,
			   readl(ni_base(ni) + CA_NI_L2FE_NI_INTF_DROP_CNT),
			   readl(ni_base(ni) + CA_NI_L2FE_DOS_FLOOD_CNT),
			   tmfwd >> 16, tmfwd & 0xffff,
			   readl(ni_base(ni) + CA_NI_QM_RX_EOP_DROP_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_RX_LEN_ERR_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_RX_L2TE_DROP_CNTR));
	}
	/* ★ TM->CPU final hop: does the frame get into the TM BM, drain OUT to the
	 * QM (tx), or get dropped (esp. NOBUF = no CPU buffer)?  Then the QM per-VoQ
	 * non-empty status: if a VoQ bit sets but wptr stays 0, the frame reached
	 * the CPU VoQ (dest OK) and the drain to CPU-EPP is the break; if no VoQ
	 * bit sets, dest routing to the CPU VoQ is the break. */
	seq_printf(m,
		   "tm: rx=%u tx=%u drop{nobuf=%u rx=%u te=%u err=%u hdr=%u}\n",
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_PCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_PCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_NOBUF_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TE_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_ERR_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_HDR_DPCNT));
	seq_printf(m,
		   "voq_status: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(0)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(1)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(2)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(3)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(4)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(5)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(6)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_STATUS(7)));
	seq_printf(m,
		   "dest-maps: tm_to_cpuq(2118)=0x%08x upper_ldpid(68f8)=0x%08x destp8_eq(6168)=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_L2TM_TM_TO_CPUQ_MAP),
		   readl(ni_base(ni) + CA_NI_QM_UPPER_LDPID_MAP),
		   readl(ni_base(ni) +
			 CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)));
	/* ★ ingress-baseline check: glb_static(a01c) port_to_cpu nibble MUST be the
	 * boot-ROM value (NOT 0) - a stale port_to_cpu=0 latch diverted port-0 off
	 * L2FE (l2fe_ni froze) and PERSISTED across warm reboots (NE reset off);
	 * only a cold boot clears it. */
	{
		u32 glb = readl(ni_base(ni) + CA_NI_NI_GLB_STATIC_CFG);

		seq_printf(m, "fwd: glb_static(a01c)=0x%08x port_to_cpu_nib=0x%x\n",
			   glb, (unsigned int)(glb & CA_NI_NI_PORT_TO_CPU));
	}
	/* ★ CPU-forwarding chain: the DFT_FWD[port-0] redir entry (indirect read,
	 * stop trusting the hardcoded string) + PDPID_MAP[CPU_0] resolution.
	 * Expect dft_fwd = 0x1820 (redir_en=1, mc_group_id=0x10=CPU_0) and pdpid=0x9
	 * (CPU).  If the frame reaches TM but qm_rx_cntr stays 0, the death is
	 * between the redir resolution and the QM. */
	{
		void __iomem *acc = ni_base(ni) + CA_NI_PLE_DFT_FWD_ACCESS;
		u32 addr = CA_NI_RX_PORT << 2;	/* port-0, DLF type 0 */
		u32 dft = 0, pdpid = 0, p19 = 0, v;

		writel(CA_NI_PLE_ACCESS_GO | addr, acc);
		if (!readl_poll_timeout(acc, v, !(v & CA_NI_PLE_ACCESS_GO),
					CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US))
			dft = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);

		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				       CA_NI_RX_CPU_LDPID);
		pdpid = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
			CA_NI_L2FE_PDPID_MAP_PDPID;

		/* PDPID_MAP[0x19] (L3_LAN classifier output): must read QM(0x08)
		 * after our remap so my-MAC/ARP frames reach the RMU, not ES8. */
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				       CA_NI_RX_L3LAN_LDPID);
		p19 = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
		      CA_NI_L2FE_PDPID_MAP_PDPID;

		/* PDPID_MAP[0x18] (L3_WAN): the HW-L3 DS ingress admission - a PON
		 * PDC frame stamped ldpid L3_WAN must resolve to pdpid 0x0a (the
		 * L3FE WAN physical ingress).  0 here = the DS data GEM's L3_WAN
		 * frames never enter the L3FE (stock live [0x18]=0xA). */
		{
			u32 p18;

			cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
					       CA_NI_RX_L3WAN_LDPID);
			p18 = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
			      CA_NI_L2FE_PDPID_MAP_PDPID;
			seq_printf(m, "fwd-chain: pdpid[0x18]=0x%x (L3_WAN; stock 0x0a = L3FE WAN ingress; 0 = DS never enters L3FE)\n",
				   p18);
		}

		/* ★ pdpid[0x19]=0x0d (L3_LAN) is STOCK-CORRECT (vendor aal_port.h: 0x0d=L3_LAN,
		 * 0x08=QM, 0x09=CPU).  The old "(want 0x8)" was WRONG - 0x08=QM egresses a wire
		 * via L2TM, NOT the CPU.  On stock the CLS trap overrides L2 fwd -> dest CPU_0
		 * (0x10) -> pdpid[0x10]=0x09 -> CPU-EPP; pdpid[0x19] is never on the CPU path.
		 * l3fe_rx(0xa9bc) vs l3qm_rx(0xa9fc) UNDER BROADCAST = the decisive bisect:
		 * l3fe_rx=0 -> frame never reaches the L3 classifier (routing/demux); l3fe_rx>0 &
		 * cls_hit=0 -> STG0/CLS not matching; cls_hit>0 & qm_rx=0 -> trap not admitted. */
		seq_printf(m,
			   "fwd-chain: dft_fwd[p0]=0x%08x (redir_en=%u mcgid=0x%lx) pdpid[0x10]=0x%x pdpid[0x19]=0x%x(stock 0x0d L3_LAN; NOT 0x8=QM->wire) qm_rx=%u qm_tx=%u l3fe_rx(0xa9bc)=%llu l3qm_rx(0xa9fc)=%llu [totals since boot]\n",
			   dft, !!(dft & CA_NI_PLE_DFT_REDIR_EN),
			   FIELD_GET(CA_NI_PLE_DFT_MC_GROUP_ID, dft), pdpid, p19,
			   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_TX_CNTR),
			   l3fe_rx, l3qm_rx);	/* the sampled TOTAL; see the note at the top */
	}
	/* ★ build68: full DFT_FWD[0..15] + MC_FIB[0x10..0x1b] dump so the coordinator can
	 * VERIFY the routing tables from /proc (our image has no devmem).  DFT_FWD read =
	 * addr(lspid<<2|type=0); MC_FIB read = indirect ACCESS[idx] then DATA0..2. */
	{
		void __iomem *acc = ni_base(ni) + CA_NI_PLE_DFT_FWD_ACCESS;
		unsigned int i;
		u32 v;

		seq_puts(m, "build68 dft_fwd[0..15]:");
		for (i = 0; i < 16; i++) {
			u32 d = 0;

			writel(CA_NI_PLE_ACCESS_GO | (i << 2), acc);
			if (!readl_poll_timeout(acc, v, !(v & CA_NI_PLE_ACCESS_GO),
						CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US))
				d = readl(ni_base(ni) + CA_NI_PLE_DFT_FWD_DATA);
			seq_printf(m, " [%u]=0x%08x", i, d);
		}
		/* verify the VLAN check-id map is programmed (the CPU-RX-dead fix) */
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_CHKID_MAP_ACCESS, 0x10);
		v = readl(ni_base(ni) + CA_NI_L2FE_CHKID_MAP_DATA);
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_CHKID_MAP_ACCESS, 0x19);
		seq_printf(m, "  chkid[CPU_0]=%u(want 8) chkid[L3_LAN]=%u(want 15)\n",
			   v, readl(ni_base(ni) + CA_NI_L2FE_CHKID_MAP_DATA));

		/* ★ 2026-07-15: real MC_FIB is @0x1644 and STOCK KEEPS IT EMPTY (no
		 * flood-to-CPU).  Dump it (want all 0) plus the 0x1634 table build70
		 * misread as MC_FIB (want stock's 0F 04 0F 09 .. values). */
		seq_puts(m, "mc_fib@0x1644 [0x10..0x1b] D2:");
		for (i = 0x10; i <= 0x1b; i++) {
			cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MC_FIB_ACCESS, i);
			seq_printf(m, " [0x%x]=0x%08x", i,
				   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA2));
		}
		seq_puts(m, "  (want all 0 = stock EMPTY)\ntbl@0x1634 [0x10..0x1b]:");
		for (i = 0x10; i <= 0x1b; i++) {
			cortina_ni_rx_ind_read(ni, CA_NI_L2FE_NKPOL_MAP_ACCESS, i);
			seq_printf(m, " [0x%x]=0x%08x", i,
				   readl(ni_base(ni) + CA_NI_L2FE_NKPOL_MAP_DATA));
		}
		seq_printf(m, "  (want stock 0f 04 0f 09 0f 05 0f 0a 0f 0b 0f 0c; arb_ctrl0x1600=0x%08x want 0x89c71c82; dq_tmport0x212c=0x%08x want 0x76543210)\n",
			   readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_TO_TM_PORT_MAP));

		/* the full mc_fib[0x19] entry - all 5 Elnath data words (want all 0) */
		{
			cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MC_FIB_ACCESS, 0x19);
			seq_printf(m,
				   "mc_fib[0x19] full D4..D0(0x1648..0x1658): %08x %08x %08x %08x %08x (want all 0 = stock)\n",
				   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA4),
				   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA3),
				   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA2),
				   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA1),
				   readl(ni_base(ni) + CA_NI_L2FE_MC_FIB_DATA0));
		}

		/* ★ build69: the admitted-frame header - THE last-ring witness.  Want
		 * 0x80000010 (dest 0x10=CPU0, deep_q CLEAR) like stock, NOT 0xc0000020
		 * (dest 0x20=CPU_MQ + deep_q -> wrong CPU-EPP256 ring). */
		v = readl(ni_base(ni) + CA_NI_QM_RMU0_RX_HDR_INFO0);
		seq_printf(m,
			   "build69 rmu0_rx_hdr(0x6904)=0x%08x dest_ldpid=0x%lx deep_q=%u (want 0x80000010 dest 0x10 deep_q 0); rmu_rx(0x6900)=%u epp_wptr(0x7000)=0x%08x\n",
			   v, FIELD_GET(CA_NI_QM_RMU0_RX_DEST_LDPID, v),
			   !!(v & CA_NI_QM_RMU0_RX_DEEP_Q),
			   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_EPP64_WRPTR(0, 0)));
	}
	/* ★ build71: L3-CLS KEY[0..15] + FIB[0..15] readback (the ARP-trap tables) so the
	 * coordinator can diff our install vs the stock golden.  Read-only indirect. */
	{
		unsigned int e, w;

		for (e = 0; e < CA_NI_RX_CLS_ENTRIES; e++) {
			cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_KEY_ACCESS, e);
			seq_printf(m, "build71 cls_key[%2u]:", e);
			for (w = 0; w < CA_NI_L3FE_CLS_KEY_WORDS; w++)
				seq_printf(m, " %08x",
					   readl(ni_base(ni) +
						 CA_NI_L3FE_CLS_KEY_DATA_BASE + 4 * w));
			cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_FIB_ACCESS, e);
			seq_puts(m, "  fib:");
			for (w = 0; w < CA_NI_L3FE_CLS_FIB_WORDS; w++)
				seq_printf(m, " %08x",
					   readl(ni_base(ni) +
						 CA_NI_L3FE_CLS_FIB_DATA_BASE + 4 * w));
			seq_puts(m, "\n");
		}
		seq_printf(m, "build73 stg0_ctrl(0x3400)=0x%08x (want 0x001c787c) spcl_pkt_det(0x3218)=0x%08x my_mac lo(0x3210)=0x%08x hi(0x3214)=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_L3FE_STG0_CTRL),
			   readl(ni_base(ni) + CA_NI_L3FE_SPCL_PKT_DET_CFG),
			   readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_LO),
			   readl(ni_base(ni) + CA_NI_L3FE_MY_MAC_HI));

		/*
		 * ★★ WITHDRAWN 2026-07-25 - the "build74 cls_hit[0..3]" probe was
		 * MISDETECTION, of exactly the class this project keeps paying
		 * for, and every conclusion ever drawn from it ("cls_hit all 0
		 * => the CLS is never consulted") is VOID.  Two defects: it put
		 * the monitor ENABLE at bit0 when it is BIT(8) (tier-2 stock
		 * aal_l3fe_glb_cls_stg_monitor_get), so the monitor was never
		 * enabled and 0x30b4 returned whatever was already there; and it
		 * read only 4 words of a read-out port that carries up to 32.
		 * Left as a read-only register dump here: the CORRECT monitor,
		 * the DBG per-stage packet counters (0x30b8/0x30bc vector 15 =
		 * L3FE_IN/OUT/T1_T2/STG3_PE) and the one-shot descriptor latch
		 * (0x30c0/c4/c8) are implemented in cortina-ni-flowoffload.c and
		 * surfaced through /proc/cortina_l3fe, which is where the
		 * flow-offload stage discrimination belongs - duplicating them
		 * here would just create a second thing to keep in sync.
		 * ★ Note the naming conflict recorded in cortina-ni-regs.h:
		 * 0x30b4/0x30bc are ALSO named GLB_LF_CFG / GLB_ILPB_00 and are
		 * written by cortina_ni_rx_l3fe_glb_init(); per the tier-2
		 * accessors they are read-data ports, so those writes are inert
		 * and did NOT unblock the L3FE ingress FIFO.
		 */
		seq_printf(m,
			   "l3fe_glb: cls_mon_ctrl(0x30b0)=0x%08x cls_mon_data(0x30b4)=0x%08x (also written as GLB_LF_CFG=0x%08x - see the conflict note in cortina-ni-regs.h; monitor enable is BIT(8), and the real stage counters live in /proc/cortina_l3fe)\n",
			   readl(ni_base(ni) + CA_NI_L3FE_CLS_MON_CTRL),
			   readl(ni_base(ni) + CA_NI_L3FE_GLB_LF_CFG),
			   CA_NI_L3FE_GLB_LF_CFG_VAL);

		/* ★ build75: profile-1 (LAN) CPU-trap rows - the LAN classifier searches
		 * KEY[64..127]; KEY[66] (wildcard) is the LAN bcast/DLF catch-all -> FIB[264]
		 * -> CPU_0.  Confirm they landed. */
		{
			static const u16 kr[] = { 64, 65, 66 };
			static const u16 fr[] = { 256, 257, 260, 264 };
			unsigned int k;

			seq_puts(m, "build75 profile1:");
			for (k = 0; k < ARRAY_SIZE(kr); k++) {
				cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_KEY_ACCESS, kr[k]);
				seq_printf(m, " key[%u]{w0=0x%08x tr=0x%08x}", kr[k],
					   readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 11 * 4),
					   readl(ni_base(ni) + CA_NI_L3FE_CLS_KEY_ACCESS + 1 * 4));
			}
			for (k = 0; k < ARRAY_SIZE(fr); k++) {
				cortina_ni_rx_ind_read(ni, CA_NI_L3FE_CLS_FIB_ACCESS, fr[k]);
				seq_printf(m, " fib[%u]{d4=0x%08x d6=0x%08x}", fr[k],
					   readl(ni_base(ni) + CA_NI_L3FE_CLS_FIB_ACCESS + 3 * 4),
					   readl(ni_base(ni) + CA_NI_L3FE_CLS_FIB_ACCESS + 1 * 4));
			}
			seq_puts(m, " (want fib[264] d4=0x1c000000 d6=0x600)\n");
		}
	}
	/* ★ per-port profile readback (the blackhole root-cause tables): expect
	 * ilpb[0] d2=0x18022163 (stp=3) d1=0x800001cb d0=0xc1000000
	 * d3=0x00100003, mmshp[0]=ffffffff_fffffffe, elpb[0]=0x3,
	 * ple_ctl=0x27b.  stp=0 or mmshp=0 = the force-drop state is back. */
	{
		u32 pd0, pd1, pd2, pd3, mh, ml, el;

		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ILPB_ACCESS, CA_NI_RX_PORT);
		pd3 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA3);
		pd2 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA2);
		pd1 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA1);
		pd0 = readl(ni_base(ni) + CA_NI_L2FE_ILPB_DATA0);
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_MMSHP_ACCESS, CA_NI_RX_PORT);
		mh = readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA1);
		ml = readl(ni_base(ni) + CA_NI_L2FE_MMSHP_DATA0);
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ELPB_ACCESS, CA_NI_RX_PORT);
		el = readl(ni_base(ni) + CA_NI_L2FE_ELPB_DATA0);

		seq_printf(m,
			   "port-prof: ilpb[0]={d3=%08x d2=%08x d1=%08x d0=%08x} stp=%lu mmshp[0]=%08x_%08x elpb[0]=0x%02x ple_ctl=0x%08x\n",
			   pd3, pd2, pd1, pd0,
			   FIELD_GET(CA_NI_L2FE_ILPB_STP_MODE, pd2),
			   mh, ml, el,
			   readl(ni_base(ni) + CA_NI_L2FE_PLE_CTL));
	}
	/* ★ ARB deep-queue diagnostics.  2026-07-15: PORT_DBUF[0] want 0 = stock
	 * (a dbuf_flg mark here = the deep-queue regression is back); BM word0
	 * bit30 = the deep_q on the last frame in buffer 0 (want 0). */
	{
		u32 arb = readl(ni_base(ni) + CA_NI_L2FE_ARB_CTRL);
		u32 portdbuf, pd0, pd1, bmhdr;

		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_ARB_PORT_DBUF_ACCESS, 0);
		portdbuf = readl(ni_base(ni) + CA_NI_L2FE_ARB_PORT_DBUF_DATA);
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				       CA_NI_RX_REDIR_LDPID);
		pd0 = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
		      CA_NI_L2FE_PDPID_MAP_PDPID;
		cortina_ni_rx_ind_read(ni, CA_NI_L2FE_PDPID_MAP_ACCESS,
				       CA_NI_L2FE_PDPID_IDX_DBUF | CA_NI_RX_REDIR_LDPID);
		pd1 = readl(ni_base(ni) + CA_NI_L2FE_PDPID_MAP_DATA) &
		      CA_NI_L2FE_PDPID_MAP_PDPID;
		cortina_ni_rx_ind_read(ni, CA_NI_L2TM_BM_PKT_MEM_ACCESS, 0);
		bmhdr = readl(ni_base(ni) + CA_NI_L2TM_BM_PKT_MEM_DATA7);

		seq_printf(m,
			   "arb-deepq: arb_ctrl=0x%08x (dbuf_sel=%u dbuf_dpid=%lu use_hdr_a=%u) port_dbuf[0]=0x%08x pdpid{DeepQ0,dbuf0/1}=0x%x/0x%x bm_word0=0x%08x (deep_q=%u cpu=%u)\n",
			   arb, !!(arb & CA_NI_L2FE_ARB_DBUF_SEL),
			   FIELD_GET(CA_NI_L2FE_ARB_DBUF_DPID, arb),
			   !!(arb & CA_NI_L2FE_ARB_USE_HDR_A_DBUF),
			   portdbuf, pd0, pd1, bmhdr,
			   !!(bmhdr & BIT(30)), !!(bmhdr & BIT(31)));

		/* ★ FLOW_DBUF (the deep_q source when dbuf_sel=1) at traffic time -
		 * want all 0 = stock (0x0f = the build100 deep_q regression is back). */
		{
			u32 fd[4];
			unsigned int k;

			for (k = 0; k < 4; k++) {
				cortina_ni_rx_ind_read(ni,
					CA_NI_L2FE_ARB_FLOW_DBUF_ACCESS, k);
				fd[k] = readl(ni_base(ni) +
					      CA_NI_L2FE_ARB_FLOW_DBUF_DATA);
			}
			seq_printf(m,
				   "flow-dbuf[0..3]@0x165c=0x%08x 0x%08x 0x%08x 0x%08x (want all 0 = stock; 0x0f = deep_q regression)\n",
				   fd[0], fd[1], fd[2], fd[3]);
		}
	}
	/* ★ LAST-FRAME resolution: the BM latches the last RX FE (L2FE-resolved)
	 * header + the raw RX-NI + the dequeued TX-NI header.  This shows what a REAL
	 * ingress frame actually resolved to (deep_q b30 / cpu b31, and the resolved
	 * ldpid), distinguishing "redir didn't fire / resolved elsewhere" from
	 * "resolved to DeepQ0/PDPID8 but the deep-queue enqueue is gated". */
	{
		u32 fe_lo = readl(ni_base(ni) + CA_NI_L2TM_BM_RX_FE_HDR_LO);
		u32 fe_hi = readl(ni_base(ni) + CA_NI_L2TM_BM_RX_FE_HDR_HI);

		/* HEADER_A: low word (0x2170) = {cos[2:0], ldpid[8:3], lspid[14:9],
		 * pkt_size[28:15], ...}; high word (0x2174) = pkt_info, deep_q=bit30,
		 * cpu_flg=bit31.  A resolved ldpid in 0x0..0x6 = DeepQ (-> QM). */
		seq_printf(m,
			   "bm-hdr: rx_fe=%08x_%08x (ldpid=0x%lx lspid=0x%lx deep_q=%u cpu=%u) rx_ni=%08x_%08x tx_ni=%08x_%08x bm_sts=0x%08x\n",
			   fe_hi, fe_lo,
			   FIELD_GET(GENMASK(8, 3), fe_lo),
			   FIELD_GET(GENMASK(14, 9), fe_lo),
			   !!(fe_hi & BIT(30)), !!(fe_hi & BIT(31)),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_NI_HDR_HI),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_NI_HDR_LO),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_NI_HDR_HI),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_NI_HDR_LO),
			   readl(ni_base(ni) + CA_NI_L2TM_BM_STS));
	}
	/* ★ 2026-07-15: L2FE post-parse HEADER_A of the LAST parsed frame (vendor
	 * aal_l2_fe_pp_heada_get) - the L2FE's OWN resolution witness.  Under a host
	 * ping expect ldpid=0x19 (the DFT_FWD 0x1832 redirect to L3_LAN) and deep_q=0
	 * now that FLOW_DBUF/PORT_DBUF are stock-zero; deep_q=1 = the regression is
	 * back; ldpid!=0x19 = the redirect never resolved. */
	{
		u32 hi = readl(ni_base(ni) + CA_NI_L2FE_PP_HEADER_A_HI);
		u32 mid = readl(ni_base(ni) + CA_NI_L2FE_PP_HEADER_A_MID);
		u32 low = readl(ni_base(ni) + CA_NI_L2FE_PP_HEADER_A_LOW);

		seq_printf(m,
			   "header_a(pp 0x11c4-cc): hi=%08x mid=%08x low=%08x | ldpid=0x%02x(want 0x19) lspid=0x%02x cpu_flag=%u deep_q=%u(want 0) mcgid=0x%02x drop_code=%u fe_bypass=%u pkt_size=%u\n",
			   hi, mid, low,
			   (mid >> 3) & 0x3f, (mid >> 9) & 0x3f,
			   hi >> 31, (hi >> 30) & 1, hi & 0xff,
			   (hi >> 8) & 7, (mid >> 29) & 1, (mid >> 15) & 0x3fff);
	}
	/* ★ GATE DIAGNOSTIC (all SAFE reads, GLB window = correct 0x4_ addressing).
	 * Only the NI-MCE region (0xaa6x) SErrors; REDIR_LDPID + MC_FIB survive - so
	 * it is a NI-MCE-specific gate, not a shared one.  We dump EVERY candidate
	 * block-reset reg to settle the 0x28-vs-0x98 conflict by DATA: the real
	 * GLOBAL_BLOCK_RESET shows a structured value (~0xd03021c0) with bits
	 * NI=0,L2FE=1,L2TM=2,L3FE=3,TQM=5; a BIST/unmapped reg reads 0.  0xa0
	 * dphy_rst is the known-mapped reference (~0x10000000).  GLOBAL_FABRIC_RESET
	 * (0xa4) has capsram/global_pe bits - a candidate NI-MCE gate. */
	{
		u32 initd = readl(ni_base(ni) + CA_NI_HV_INIT_DONE);
		void __iomem *glb = ni->win[CA_NI_WIN_GLB];

		/* ★ build98: NIRX_MISC_CFG offset is DISPUTED - our driver treats 0xa1bc as
		 * NIRX_MISC (writes 0x3e80 = rdy_en bits9-13 SET, believing a -0x3c shift vs
		 * rtl8277c), but the raw rtl8277c map puts NIRX_MISC (l2te_ni_*_rdy_en[13:9],
		 * bit11=l3felan_port_rdy_en = the LAN->L3FE handoff gate) at 0xa1f8.  Read BOTH:
		 * whichever holds ~0x3e80 (bits9-13) on stock is the real NIRX_MISC; if OURS
		 * differs there, that unset rdy-enable is the l3fe_rx=0 gate. */
		seq_printf(m,
			   "gate: ni_init_done(a004)=0x%08x (ni_done=%u) nirx_misc@0xa1bc=0x%08x nirx_misc@0xa1f8=0x%08x (real one holds rdy_en bits9-13 ~0x3e80; bit11=l3felan_rdy)\n",
			   initd, !!(initd & CA_NI_HV_INIT_DONE_NI),
			   readl(ni_base(ni) + 0xa1bc),
			   readl(ni_base(ni) + 0xa1f8));
		if (glb)
			seq_printf(m,
				   /* ★ LABELS ANCHORED IN STOCK'S OWN REGISTER TABLE (tier 2),
				    * 2026-08-05.  They previously named five registers wrongly,
				    * because our GLB reset constants carried the sibling
				    * rtl8277C offsets (uniformly -8) - so the wrong NAME and the
				    * wrong ADDRESS cancelled and nothing ever failed to force the
				    * fix.  0x28 is a BIST control, 0x98 is the OPTICAL MODULE
				    * status and 0x9c is PON control: none of the three is a reset
				    * register.  The block reset really lives at 0xa0. */
				   "gate glb: bist_ctrl4(28)=0x%08x opt_module_status(98)=0x%08x pon_cntl(9c)=0x%08x block_reset(a0)=0x%08x block_reset_ext(a4)=0x%08x\n",
				   readl(glb + CA_NI_GLB_BIST_CONTROL4),
				   readl(glb + CA_NI_GLB_OPT_MODULE_STATUS),
				   readl(glb + CA_NI_GLB_PON_CNTL),
				   readl(glb + CA_NI_GLB_BLOCK_RESET),
				   readl(glb + CA_NI_GLB_BLOCK_RESET_EXT));
	}
	/* ★ PORT CHECK: which physical GPHY carries the host's link.  Stock's
	 * only-carrier port may not be our configured port 0 - if link=1 shows on
	 * a port != 0 (and mac/l2fe counters move only for that port), our
	 * port<->GPHY mapping is the bug (stock configures all 4 LAN ports). */
	if (ni->mii) {
		int p;

		for (p = 0; p < CA_NI_GPHY_COUNT; p++) {
			int a = CA_NI_GPHY_FIRST + p;
			int bmsr;

			mdiobus_read(ni->mii, a, MII_BMSR);	/* clear latch */
			bmsr = mdiobus_read(ni->mii, a, MII_BMSR);
			seq_printf(m, "link port%d (phy%d): bmsr=0x%04x link=%d\n",
				   p, a, bmsr,
				   bmsr >= 0 ? !!(bmsr & BMSR_LSTATUS) : -1);
		}
	}

	/* self-populating pool health: inactive MUST be 0 and stay 0 (a bid that
	 * goes inactive without a refill source = a leaked buffer) */
	pa_req = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID));
	/* "self-pop" was hardcoded and became a lie once the pool model was made
	 * selectable; print the live mode instead.  inactive = buffers the pool is
	 * SHORT, so 0 is the healthy reading in both models. */
	seq_printf(m, "pool: %s eq%d ready=%lu inactive=%lu (want 0)\n",
		   cpu_pool_push ? "sw-owned" : "self-pop",
		   CA_NI_RX_EQ_ID,
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_REQ_READY, pa_req),
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pa_req));
	seq_printf(m, "eq%d cfg0/1/2=0x%08x/0x%08x/0x%08x rmu0=0x%08x rmu_fe_drop(0x6944)=0x%08x rx_cntr(0x6900)=0x%08x\n",
		   CA_NI_RX_EQ_ID,
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID)),
		   readl(ni_base(ni) + CA_NI_QM_RMU0_CTRL),
		   readl(ni_base(ni) + CA_NI_QM_RMU_FE_DROP),
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR));
	/* ★ EQ-profile routing: the deep_q frame's dest-port (0x0d) selects
	 * profile_sel[2:0]=5, so EQ_PROFILE(5) MUST point at {eqp0=13,eqp1=14}
	 * (0x000000ed).  If prof5=0 the frame hits an unconfigured profile -> no
	 * buffer -> no RMU admission. */
	/* ★ deep-queue chain: dest8/9 profile_sel=0x0C=12 -> EQ_PROFILE(12)=0xEC ->
	 * EQ12 pool.  eq12 cfg0 bit0=eq_en, cfg1[29:16]=total(want 0x100). */
	seq_printf(m, "dq-chain: destp8(0x6188)=0x%08x destp15=0x%08x prof13(0x615c)=0x%08x(want 0xed) prof12=0x%08x eq12 cfg0/1/2=0x%08x/0x%08x/0x%08x\n",
		   readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(8)),
		   readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ12_PROFILE)),
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ12_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ12_ID)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ12_ID)));
	/* ★ ALL-16-EQ active-pool map: for each EQ, eq_en (cfg0 bit0) + total_buf
	 * (cfg1[29:16]).  CPU pool = EQ13/14.  The DEEP-QUEUE pool (for a deep_q=1
	 * frame) is at whichever OTHER EQs are active - NOT EQ0/1/2 (those read reset
	 * on stock).  Diff ours-vs-stock to locate the real Elnath DQ pool EQ ids. */
	{
		int e;

		seq_printf(m, "eq-map (en:total):");
		for (e = 0; e < 16; e++) {
			u32 c0 = readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(e));
			u32 c1 = readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(e));

			seq_printf(m, " eq%d=%u:%lu", e, c0 & 1,
				   (unsigned long)FIELD_GET(CA_NI_QM_CFG1_TOTAL_BUF_NUM, c1));
		}
		seq_printf(m, "\n");
	}
	seq_printf(m, "eq-prof0-7: %08x %08x %08x %08x %08x %08x %08x %08x (prof13=%08x)\n",
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(0)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(1)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(2)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(3)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(4)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(5)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(6)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(7)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)));
	/* pool1 (EQ14) PA-request + the CPU-EPP ring pointers: if eq13 ready=1
	 * and the ring wptr advances past rdptr, the delivery chain is live */
	pa_req = readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2));
	/* ★ The old annotation here recommended "cfg2 want 0xff0d: bufsz=5".  That
	 * is WRONG on this die - buffer_size index 5 is 4096B, not 2048 (the live
	 * decode at CA_NI_QM_EQ13_CFG2 records idx5 as a fixed stride bug), and
	 * the PA->VA math shears if the index does not match the pool stride.  The
	 * correct software-owned value is 0xff0c (cpu_eq=1, bufsz idx4 = 2048).
	 * ready/inactive are a per-pool GAUGE, not a cumulative counter: inactive
	 * = how many buffers the pool is SHORT, so it must read 0 once the seed
	 * matches CFG1.total_buf.  If you need to know whether it is clear-on-read,
	 * cat this file twice with no traffic between - a gauge repeats. */
	seq_printf(m, "eq%d ready=%lu inactive=%lu cfg1=0x%08x cfg2=0x%08x (sw-owned wants 0x0000ff0c: cpu_eq=1 bufsz idx4=2048)\n",
		   CA_NI_RX_EQ_ID2,
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_REQ_READY, pa_req),
		   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT, pa_req),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID2)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID2)));
	seq_printf(m, "l2tm-es: es_ctrl=0x%08x (tx_en=%u p8_L3QM=%u) sch8=0x%08x (voq_en=0x%02lx) bm_dq_map=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL),
		   !!(readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL) & CA_NI_L2TM_ES_TX_EN),
		   !!(readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL) & BIT(CA_NI_L2TM_ES_PORT_L3QM)),
		   readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(CA_NI_L2TM_ES_PORT_L3QM)),
		   (unsigned long)(readl(ni_base(ni) + CA_NI_L2TM_ES_SCH_CFG(CA_NI_L2TM_ES_PORT_L3QM)) & CA_NI_L2TM_ES_VOQ_EN_ALL),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_DQ_PORT_MAP));
	seq_printf(m, "ni_qm_hol: es_ctrl2_real(0x6a30)=0x%08x (bit1=%u want 1) rmu0_ctrl=0x%08x qm_es_ctrl=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_QM_ES_CTRL2_REAL),
		   !!(readl(ni_base(ni) + CA_NI_QM_ES_CTRL2_REAL) & CA_NI_QM_ES_CTRL2_NI_QM_HOL),
		   readl(ni_base(ni) + CA_NI_QM_RMU0_CTRL),
		   readl(ni_base(ni) + CA_NI_QM_ES_CTRL));
	seq_printf(m, "epp ring: wptr=0x%06x rdptr_sw=0x%x cmd_mode=%s es_ctrl2=0x%08x demux_sel=0x%x\n",
		   cortina_ni_rx_wptr(ni), rx->rptr[0],
		   (readl(ni_base(ni) + CA_NI_QM_EPP) & CA_NI_QM_EPP_CMD_MODE_64) ?
		   "64b" : "32b",
		   readl(ni_base(ni) + CA_NI_QM_ES_CTRL2),
		   (unsigned int)(readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG) &
				  CA_NI_NI_L3QMRX_DEMUX_SEL_ALL));
	/* ★★ build49 THE DECISIVE TEST: dump the CPU-virtual EPP ring DRAM (0x0bc48000).
	 * After arping, non-zero descriptor slots => the HW writeback LANDS (the gap is
	 * wptr/detection); all-zero => the writeback FAILS (0x611c bit22 error is real). */
	if (rx->ring) {
		u64 s0 = le64_to_cpu(rx->ring[0]), s1 = le64_to_cpu(rx->ring[1]);
		u64 s2 = le64_to_cpu(rx->ring[2]), s3 = le64_to_cpu(rx->ring[3]);
		unsigned int v;

		seq_printf(m, "epp-ring voq0: slot0=%08x_%08x slot1=%08x_%08x slot2=%08x_%08x slot3=%08x_%08x\n",
			   upper_32_bits(s0), lower_32_bits(s0), upper_32_bits(s1), lower_32_bits(s1),
			   upper_32_bits(s2), lower_32_bits(s2), upper_32_bits(s3), lower_32_bits(s3));
		seq_puts(m, "epp-ring voq0..7 slot0:");
		for (v = 0; v < CA_NI_RX_VOQ_COUNT; v++) {
			u64 sv = le64_to_cpu(rx->ring[v * CA_NI_RX_RING_SLOTS_PER_VOQ]);

			seq_printf(m, " v%u=%08x_%08x", v, upper_32_bits(sv), lower_32_bits(sv));
		}
		seq_puts(m, "\n");

		/* ★ build88 DECISIVE: dump BOTH per-voq rings' voq0 first 16 entries -
		 * LOW = PADDR(0x7200)=0x0bc48000 (where NAPI reads) vs HIGH = PADDR_HI(0x7220)=
		 * 0x0bc4a000 (=PADDR+0x2000).  wptr advanced 38 but NAPI's LOW reads poison, so
		 * the engine's real {PA,len} descriptors are in whichever ring is NOT deadbeef. */
		{
			unsigned int hi = CA_NI_RX_RING_HI_OFFSET / sizeof(__le64);
			unsigned int k;

			seq_puts(m, "build88 LOW(0x0bc48000) voq0:");
			for (k = 0; k < 16; k++) {
				u64 d = le64_to_cpu(rx->ring[k]);

				seq_printf(m, " %08x_%08x", upper_32_bits(d), lower_32_bits(d));
			}
			seq_puts(m, "\nbuild88 HIGH(0x0bc4a000) voq0:");
			for (k = 0; k < 16; k++) {
				u64 d = le64_to_cpu(rx->ring[hi + k]);

				seq_printf(m, " %08x_%08x", upper_32_bits(d), lower_32_bits(d));
			}
			seq_puts(m, "  (deadbeef=poison; real desc = a 0x094xxxxx PA + small len; rx_ring_hi param = which ring NAPI reads)\n");
		}
	}
	/* ★★ THE NI_HV VALUES BELOW COME FROM THE ONE SAMPLE TAKEN AT THE TOP.
	 * 0xa9fc is CLEAR-ON-READ, and this one show() used to read it TWICE - here and
	 * again in the ni_hv-rx line below.  The first read took the count and the second
	 * was therefore a structural ~0, printed as if it were a measurement and cited as
	 * evidence in several places (found 2026-08-05, verified against stock's
	 * `ca-ne.ko`, whose own text says "NI counter ===== (read-and-clear)").
	 * ⚠ ni2qm_tx (0xaa10), mce (0xaa3c) and dma (0xaa7c) are the SAME kind of counter,
	 * so they are taken from the same sample too - a plain readl here would now find
	 * them already cleared by cortina_ni_nihv_sample() and print a confident zero,
	 * which is the identical defect one register along.
	 * miss_sop_eop (0xa9f4) and short_err (0xa9f8) are still read directly: each packs
	 * TWO quantities in one word (hi16/lo16), so they are not accumulated and are
	 * printed raw - read them as "since the last read of this file", never as a total. */
	/* ★★ PROVEN-cumulative per-stage counters (RE a0668fdf) - idle-vs-ping each; the
	 * FIRST that does not climb under ping = the death stage.  l2tm_tx already climbs
	 * (frame leaves L2TM); ni2qm_* = frames accepted into L3QM (stage2); rmu_* = RMU0
	 * (0x6900 operator-suspect, 0x690c=scheduled); epp_wptr = descriptors written. */
	seq_printf(m, "datapath: bm_rx(0x213c)=%u bm_tx(0x2140)=%u | ni2qm_rx(0xa9fc)=%llu miss_sop_eop(0xa9f4)=0x%08x short_err(0xa9f8)=0x%08x ni2qm_tx(0xaa10)=%llu | rmu_rx(0x6900)=%u rmu_sched(0x690c)=%u | epp_wptr(0x7000)=0x%06x | drop no_buf(0x6940)=%u fe(0x6944)=%u\n",
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_PCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TX_PCNT),
		   l3qm_rx,
		   readl(ni_base(ni) + CA_NI_NI_L3QM_RX_MISS_SOP_EOP),
		   readl(ni_base(ni) + CA_NI_NI_L3QM_RX_SHORT_ERR),
		   nihv[CA_NI_NIHV_L3QM_TX],
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		   readl(ni_base(ni) + CA_NI_QM_TX_CNTR),
		   cortina_ni_rx_wptr(ni),
		   readl(ni_base(ni) + CA_NI_QM_RMU_NO_BUF_DROP),
		   readl(ni_base(ni) + CA_NI_QM_RMU_FE_DROP));
	/* ★ build37 BM-drop ledger (RE a053902d): the FIRST drop that climbs +N under an
	 * N-ARP flood tells WHERE/why a deep_q frame dies before L3QM.  te=threshold-engine
	 * (deep-queue/VOQ/port threshold), sb=shared-buffer full, nobuf=no free buffer,
	 * hdr=header, err=error, rx=enqueue drop.  All 0 + bm_tx climbing = egressed-but-lost
	 * downstream (read the L3QM miss_sop_eop/short_err siblings above). */
	seq_printf(m, "bm-drops: te(0x214c)=%u sb(0x2144)=%u nobuf(0x216c)=%u hdr(0x2148)=%u err(0x2150)=%u rx(0x2164)=%u\n",
		   readl(ni_base(ni) + CA_NI_L2TM_BM_TE_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_SB_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_NOBUF_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_HDR_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_ERR_DPCNT),
		   readl(ni_base(ni) + CA_NI_L2TM_BM_RX_DPCNT));
	/* ★ build42: WHICH NI_HV interface does the dequeue land on? (all 4 RX_PKT_CNT,
	 * stride 0x40).  bm_tx +9 but l3qm(0xa9fc)=0 -> if l3fe(0xa9bc) climbs, the frame
	 * goes to the L3FE interface we don't drain, not L3QM; if ALL 4 stay 0, the dequeue
	 * never reaches NI_HV = the shared BM->NI_HV handoff is the gate. */
	/* all four come from the ONE sample taken at the top of this function */
	seq_printf(m, "ni_hv-rx: l3fe(0xa9bc)=%llu l3qm(0xa9fc)=%llu mce(0xaa3c)=%llu dma(0xaa7c)=%llu [read-and-clear registers, accumulated: totals since boot]\n",
		   l3fe_rx,
		   l3qm_rx,
		   nihv[CA_NI_NIHV_MCE_RX],
		   nihv[CA_NI_NIHV_DMA_RX]);
	/* ★★ build43: the QM/RMU0-side bisect (RE a053902d - the RIGHT counters; 0xa9fc may
	 * be egress-direction).  no_buf climbing while rmu_rx=0 = frame reached RMU0 but EQ13
	 * (128B CPU pool) empty = the operator's recorded wall = seed EQ13.  RE offsets shown
	 * next to our Elnath offsets (0x6900/0x6940) to see which are live. */
	seq_printf(m, "qm-rmu: rmu_rx[RE0x67d8]=%u [ours0x6900]=%u | no_buf[RE0x6818]=%u [ours0x6940]=%u | eq13_usg(0x695c)=0x%08x cpu_push_rdy(0x6368)=0x%08x eq_unfill(0x63c0)=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_QM_RMU0_RX_PKT_CNTR_RE),
		   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
		   readl(ni_base(ni) + CA_NI_QM_RMU0_NO_BUF_DROP_RE),
		   readl(ni_base(ni) + CA_NI_QM_RMU_NO_BUF_DROP),
		   readl(ni_base(ni) + CA_NI_QM_EQ13_BUF_USG),
		   readl(ni_base(ni) + CA_NI_QM_CPU_PUSH_RDY0_RE),
		   readl(ni_base(ni) + CA_NI_QM_EQ_STACK_UNFILL));
	/* ★ NI_HV interconnect region (L2TM-egress -> L3QM-ingress demux/source-select) -
	 * the proven death stage.  Dump 0xa180-0xa1c4 vs the stock golden to catch any
	 * ours-vs-stock mismatch (0xa1c0=0x76543210 was the one we never wrote). */
	{
		static const u32 nihv_want[] = {
			0x00a87f00, 0x0024009b, 0x00000000, 0xffff7f7f,	/* a180 a184 a188 a18c */
			0x040c2040, 0x00007185, 0x00000000, 0x00000002,	/* a190 a194 a198 a19c */
			0x22aa0000, 0x00000000, 0x00000000, 0x00000000,	/* a1a0 a1a4 a1a8 a1ac */
			0x22aa0000, 0xaaaa0000, 0x00086ffc, 0x00003e80,	/* a1b0 a1b4 a1b8 a1bc */
			0x76543210,					/* a1c0 */
		};
		unsigned int j;

		seq_puts(m, "ni_hv (0xa180-0xa1c0, L2TM->L3QM demux; ! = differs from stock):\n");
		for (j = 0; j < ARRAY_SIZE(nihv_want); j++) {
			u32 off = 0xa180 + j * 4;
			u32 v = readl(ni_base(ni) + off);

			seq_printf(m, "  0x%04x=0x%08x want 0x%08x %s\n",
				   off, v, nihv_want[j],
				   v == nihv_want[j] ? "" : "  !MISMATCH");
		}
	}
	/* ★ per-voq CPU-EPP wptrs + RMU0 admission/drop bisect: if a voq's wptr
	 * moves the RMU pushed to it; if 0x6900 stays 0 but no_buf/fe_drop climb the
	 * frame reaches RMU0 but has no buffer; both 0 = frame never arrives. */
	{
		unsigned int q;

		seq_printf(m, "epp voq-wptr:");
		for (q = 0; q < CA_NI_RX_VOQ_COUNT; q++)
			seq_printf(m, " q%u=0x%03x", q,
				   cortina_ni_rx_wptr_voq(ni, q));
		seq_printf(m, " | rmu0_rx=%u no_buf_drop(0x6940)=%u fe_drop(0x6944)=%u\n",
			   readl(ni_base(ni) + CA_NI_QM_RX_CNTR),
			   readl(ni_base(ni) + CA_NI_QM_RMU0_NO_BUF_DROP),
			   readl(ni_base(ni) + CA_NI_QM_RMU0_FE_DROP));
	}
	/* ★ CB occupancy A/B bisect: scan VOQ buf-cnt 0..63, print non-zero (frame
	 * IN the central buffer = scanner/drain gap; all 0 while stock climbs =
	 * deep_q never enqueued into the CB). */
	{
		unsigned int q, nz = 0;

		seq_printf(m, "cb-occupancy voq_bufcnt(nonzero, idx 0..127):");
		for (q = 0; q < 128; q++) {
			u32 c;

			cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_VOQ_BUFCNT_ACCESS, q);
			c = readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_BUFCNT_DATA);
			if (c) {
				seq_printf(m, " q%u=%u", q, c);
				nz++;
			}
		}
		if (!nz)
			seq_printf(m, " NONE(all 0)");
		seq_printf(m, "\n");
	}
	/* ★ deep-queue populate check: EQ12 pa_req (bit31=req active, like stock's
	 * 0x80000000) + the CB per-port free-buf-cnt we seeded (ports 0/8). */
	{
		u32 f0, f8;

		cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, 0);
		f0 = readl(ni_base(ni) + CA_NI_L2TM_CB_PORT_FREECNT_DATA);
		cortina_ni_rx_ind_read(ni, CA_NI_L2TM_CB_PORT_FREECNT_ACCESS, 8);
		f8 = readl(ni_base(ni) + CA_NI_L2TM_CB_PORT_FREECNT_DATA);
		seq_printf(m, "dq-populate: eq12_pa_req(0x63d8)=0x%08x (req=%u want 1) cb_freebuf[p0]=0x%08x [p8]=0x%08x\n",
			   readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ12_ID)),
			   !!(readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ12_ID)) & CA_NI_QM_PA_REQ_READY),
			   f0, f8);
	}
	/* ★ QUEUE-DEPTH READBACK - which setting was actually in force.  Prints the
	 * parameters AND a live indirect read of the first and last profile entry of
	 * both per-VoQ threshold tables, so a benchmark's queue depth is provable after
	 * the run instead of remembered.  A benchmark whose configuration cannot be read
	 * back afterwards is not evidence. */
	{
		u32 dq0, cb0_1, cb0_0, dq7, cb7_1, cb7_0;

		cortina_ni_rx_deepq_thrsh_read(ni, 0, &dq0, &cb0_1, &cb0_0);
		cortina_ni_rx_deepq_thrsh_read(ni,
					       CA_NI_L2TM_DEEPQ_VOQ_ENTRIES - 1,
					       &dq7, &cb7_1, &cb7_0);
		seq_printf(m,
			   "deepq-thrsh: param{deepq_voq_thrsh=0x%08x deepq_cb_stock=%d} hw_dqsch[0]=0x%08x hw_dqsch[%u]=0x%08x hw_cb[0]={0x%08x,0x%08x} hw_cb[%u]={0x%08x,0x%08x} acc{dqsch(0x2e70)=0x%08x cb(0x2da0)=0x%08x}\n",
			   READ_ONCE(deepq_voq_thrsh), READ_ONCE(deepq_cb_stock),
			   dq0, CA_NI_L2TM_DEEPQ_VOQ_ENTRIES - 1, dq7,
			   cb0_1, cb0_0, CA_NI_L2TM_DEEPQ_VOQ_ENTRIES - 1,
			   cb7_1, cb7_0,
			   readl(ni_base(ni) + CA_NI_L2TM_DQSCH_VOQ_THRSH_ACCESS),
			   readl(ni_base(ni) + CA_NI_L2TM_CB_VOQ_THRSH_ACCESS));
		seq_puts(m,
			 "deepq-thrsh: SCOPE = the deep-queue path only: CPU-RX always; HW-offloaded US only when the live data T-CONT <= 7 (see live_pon{tcont=} in /proc/cortina_l3fe); HW-offloaded DS only with hw_ds_deepq=1 (default off); CPU-forwarded US (pon_data_enq) never\n");
		seq_printf(m,
			   "deepq-thrsh: dqsch stock=0x%08x permissive=0x%08x (ours ~292x deeper); cb stock={0x%08x,0x%08x} permissive=0x%08x (stock is DEEPER here); direct regs 0x2d80/88/8c/90/94/98 + 0x2ec8..0x2ee8 are tier-1 stock golden and are NOT part of this knob\n",
			   CA_NI_L2TM_DQSCH_VOQ_THRSH_VAL,
			   CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE,
			   CA_NI_L2TM_CB_VOQ_THRSH_D1, CA_NI_L2TM_CB_VOQ_THRSH_D0,
			   CA_NI_L2TM_DEEPQ_PROFILE_PERMISSIVE);
	}
	cortina_ni_cpu_fwd_show(m, ni);
	seq_printf(m, "destport0: eq_cfg=0x%08x pkt_buf=0x%08x profile%d=0x%08x voq_en=0x%08x\n",
		   readl(ni_base(ni) +
			 CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)),
		   readl(ni_base(ni) +
			 CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT)),
		   CA_NI_RX_EQ_PROFILE,
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
		   readl(ni_base(ni) + CA_NI_QM_VOQ_EN(CA_NI_RX_CPU_PORT)));
	/* ★ build77: the CPU_0 pools = EQ5(pool0)/EQ6(pool1) (RE of init_empty_buffer_CPU).
	 * Confirm cfg2 has cpu_eq=1+bufsz=5, ibid (inactive_bid) dropping as buffers push,
	 * EQ_PROFILE[2]={eqp0=5,eqp1=6}=0x65, destport0 profile_sel=2. */
	seq_printf(m,
		   "build81 EQ5{cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x pa_req(0x72f8)=0x%08x} EQ6{cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x pa_req(0x72fc)=0x%08x} prof[2]=0x%08x(want 0x65) destp0=0x%08x(psel 2) [want pa_req req(bit31)=0]\n",
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(5)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(5)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(5)),
		   readl(ni_base(ni) + CA_NI_QM_EQM_INACTIVE_BID(5)),
		   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(6)),
		   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(6)),
		   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(6)),
		   readl(ni_base(ni) + CA_NI_QM_EQM_INACTIVE_BID(6)),
		   readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(2)),
		   readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(0)));
	/* GPHY wedge spy: fault != 0 on a zero-RX boot = the root cause the
	 * 1 Hz recovery is there to heal; recoveries counts reinits fired */
	seq_printf(m, "gphy: fault=0x%04x last=0x%04x recoveries=%llu rearms=%llu\n",
		   cortina_ni_rx_gphy_fault(ni), rx->last_fault,
		   rx->recoveries, rx->rearms);
	seq_printf(m, "frames=%llu bytes=%llu polls=%llu swid=%llu pon=%llu wan=%llu wan_l3=%llu\n",
		   rx->frames, rx->bytes, rx->polls, rx->swid_frames,
		   rx->pon_frames, rx->wan_frames, rx->wan_l3_frames);
	/* packet-order spy: one flow must stay on ONE voq (>=2 climbing under a
	 * unidirectional bench = HW spreads the flow, drain order can reorder) */
	seq_puts(m, "voq_frames:");
	for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++)
		seq_printf(m, " %d:%llu", i, rx->voq_frames[i]);
	seq_puts(m, "\n");
	seq_printf(m, "drops: nosop=%llu badpa=%llu len=%llu (runt=%llu oversize=%llu) nobuf=%llu dead=%llu\n",
		   rx->drop_nosop, rx->drop_badpa, rx->drop_len,
		   rx->drop_runt, rx->drop_oversize,
		   rx->drop_nobuf, rx->slot_dead);
	/*
	 * Multi-buffer receive.  What each number means for a verdict:
	 *   frames>0        a chain was assembled and delivered - the witness
	 *                   that this path works at all;
	 *   nosop / oversize above should STOP climbing once it is on, because
	 *                   both are what an unassembled chain looked like;
	 *   abort/reopen/orphan/badtotal/toolong/short  MUST stay 0 - each is a
	 *                   distinct malformation, named so a non-zero one says
	 *                   which;
	 *   dlen seen vs calc  an OBSERVATION, not a verdict: the shipped
	 *                   firmware discards the per-segment descriptor length,
	 *                   so what the hardware puts there is unestablished and
	 *                   the first chained frame is what settles it.
	 */
	{
		unsigned int open = 0;

		for (i = 0; i < CA_NI_RX_VOQ_COUNT; i++)
			if (rx->chain[i].st.open)
				open++;
		seq_printf(m, "chain: mode=%s rest_off=+0x%02x frames=%llu segs=%llu max_segs=%llu open_now=%u\n",
			   rx_chain ? "on" : "OFF", rx->chain_rest_off,
			   rx->chain_frames, rx->chain_segs,
			   rx->chain_max_segs, open);
		seq_printf(m, "chain-bad: abort=%llu reopen=%llu orphan=%llu badtotal=%llu toolong=%llu short=%llu swid=%llu (all want 0)\n",
			   rx->chain_abort, rx->chain_reopen, rx->chain_orphan,
			   rx->chain_badtotal, rx->chain_toolong,
			   rx->chain_short, rx->chain_swid);
		if (rx->chain_frames) {
			seq_printf(m, "chain-dlen: diff=%llu seen/calc", rx->chain_dlen_diff);
			for (i = 0; i < CA_NI_RX_CHAIN_MAX_SEGS &&
				    rx->chain_dlen_calc[i]; i++)
				seq_printf(m, " %u:%u/%u", i,
					   rx->chain_dlen_seen[i],
					   rx->chain_dlen_calc[i]);
			seq_puts(m, " (observation, not a fault)\n");
		}
	}
	/* pool ownership: stale_buf MUST be 0 - non-zero means a buffer was
	 * reused before NAPI copied it out (the fragmented-datagram defect) */
	seq_printf(m, "pool-own: mode=%s stale_buf=%llu (want 0) push_fail=%llu (want 0)\n",
		   cpu_pool_push ? "sw-owned(cpu_eq=1)" : "hw-managed(cpu_eq=0)",
		   rx->stale_buf, rx->push_fail);
	/* deep-queue pool witness: dq_frames > 0 proves EQ12 is populated and the
	 * deep-queue admission path is delivering (the GPON DS punt rides it);
	 * inactive is the usual shortfall gauge, 0 = the QM self-populated it. */
	{
		u32 dq_req = readl(ni_base(ni) +
				   CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ12_ID));

		seq_printf(m, "pool-dq: eq%u frames=%llu cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x prof%u=0x%08x ready=%lu inactive=%lu (want inactive 0; frames>0 once DS flows)\n",
			   CA_NI_RX_EQ12_ID, rx->dq_frames,
			   readl(ni_base(ni) + CA_NI_QM_CFG0_EQ(CA_NI_RX_EQ12_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ12_ID)),
			   readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ12_ID)),
			   CA_NI_RX_DQ_PROFILE_SEL,
			   readl(ni_base(ni) +
				 CA_NI_QM_EQ_PROFILE(CA_NI_RX_DQ_PROFILE_SEL)),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_REQ_READY, dq_req),
			   (unsigned long)FIELD_GET(CA_NI_QM_PA_INACTIVE_CNT,
						    dq_req));
	}
	seq_printf(m, "last_desc=%016llx last_hdra=%016llx\n",
		   rx->last_desc, rx->last_hdra);
	/* ★ TEMP DIAG (rx_crc_tap): machine-readable HW lookup-CRC witness */
	if (rx_crc_tap)
		seq_printf(m,
			   "crc_tap: hits=%llu hw_crc32=%08x hw_crc16=%04x cpu_flg=%u (TEMP DIAG - diff vs install crc)\n",
			   rx->tap_hits, rx->tap_crc32, rx->tap_crc16,
			   rx->tap_cpuflg);
	seq_puts(m, "irq_hits:");
	for (i = 0; i < CA_NI_RX_NUM_IRQS; i++)
		seq_printf(m, " %d:%llu", rx->irq[i], rx->irq_hits[i]);
	seq_puts(m, "\n");

	cortina_ni_rx_dump_regs(m, ni);

	/* ★ QM+L2TM full-block sweep (grep/diff-friendly, one per line) - the
	 * ours-vs-stock hunt for the L2TM->QM admit gate (qm_rx_cntr=0). */
	{
		unsigned int k;

		seq_puts(m, "qmblock:\n");
		for (k = 0; k < ARRAY_SIZE(cortina_ni_qmdump_offs); k++)
			seq_printf(m, "  0x%04x=0x%08x\n",
				   cortina_ni_qmdump_offs[k],
				   readl(ni_base(ni) + cortina_ni_qmdump_offs[k]));
	}

	/* ★ axi_reo (RMU DMA-reorder) window dump - a SEPARATE MMIO window (idx 10),
	 * NOT covered by the NI-core qmblock sweep.  Dump the 3 channel blocks so the
	 * 21-reg golden (READ 0x000 / WRITE 0x400 / WRITE2 0x480) is diff-able vs stock. */
	{
		void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];
		unsigned int k;

		seq_puts(m, "axi_reo (window idx10, g_ne_axi_reo):\n");
		if (!reo) {
			seq_puts(m, "  <window not mapped>\n");
		} else {
			for (k = 0; k < ARRAY_SIZE(cortina_ni_axi_reo_cfg); k++)
				seq_printf(m, "  0x%04x=0x%08x (want 0x%08x)\n",
					   cortina_ni_axi_reo_cfg[k].off,
					   readl(reo + cortina_ni_axi_reo_cfg[k].off),
					   cortina_ni_axi_reo_cfg[k].val);
		}
	}

	/* ★ FBM window dump (GLB/AXI/POOL) - the RMU buffer-allocator, separate windows
	 * (idx 18/19/21).  glb0 low byte = pool-enable (want 0xFF), pool0+0x10 = refill
	 * (want 0 = OFF).  Not covered by the NI-core qmblock sweep. */
	{
		void __iomem *glb  = ni->win[CA_NI_WIN_FBM_GLB];
		void __iomem *axi  = ni->win[CA_NI_WIN_FBM_AXI];
		void __iomem *pool = ni->win[CA_NI_WIN_FBM_POOL];

		seq_puts(m, "fbm (windows idx18/19/21):\n");
		if (!glb || !axi || !pool) {
			seq_printf(m, "  <unmapped glb=%d axi=%d pool=%d>\n",
				   !!glb, !!axi, !!pool);
		} else {
			seq_printf(m, "  glb 0x00=0x%08x 0x04=0x%08x 0x0c=0x%08x 0x10=0x%08x 0x70=0x%08x\n",
				   readl(glb + 0x00), readl(glb + 0x04),
				   readl(glb + 0x0c), readl(glb + 0x10),
				   readl(glb + 0x70));
			seq_printf(m, "  axi 0x00=0x%08x (want 0x200)\n",
				   readl(axi + 0x00));
			seq_printf(m, "  pool0 cfg0x00=0x%08x exstack0x04=0x%08x depth0x08=0x%08x cnt0x0c=0x%08x outstanding0x2c=0x%08x\n",
				   readl(pool + 0x00), readl(pool + 0x04),
				   readl(pool + 0x08), readl(pool + 0x0c),
				   readl(pool + 0x2c));
		}
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* probe                                                               */
/* ------------------------------------------------------------------ */

static int cortina_ni_rx_irqs_init(struct cortina_ni *ni)
{
	struct platform_device *pdev = to_platform_device(ni->dev);
	struct cortina_ni_rx *rx = ni->rx;
	int i, irq, ret, got = 0;

	/*
	 * DT lists the 8 per-cpu-port EPP interrupts first (SPI 0x54..0x5b);
	 * SPI 0x54 should be cpu port 0.  Request all 8 so a different
	 * ordering shows up as a nonzero irq_hits[i!=0] instead of silence
	 * (HW-verify #3); every handler drains the same port0/voq0 ring.
	 */
	for (i = 0; i < CA_NI_RX_NUM_IRQS; i++) {
		rx->irq[i] = -1;
		rx->irqctx[i].ni = ni;
		rx->irqctx[i].idx = i;

		irq = platform_get_irq_optional(pdev, i);
		if (irq < 0)
			continue;

		ret = devm_request_irq(ni->dev, irq, cortina_ni_rx_isr, 0,
				       devm_kasprintf(ni->dev, GFP_KERNEL,
						      "%s-rx%d",
						      dev_name(ni->dev), i),
				       &rx->irqctx[i]);
		if (ret) {
			dev_warn(ni->dev, "cannot request RX irq %d (#%d)\n",
				 irq, i);
			continue;
		}
		rx->irq[i] = irq;
		got++;
	}

	if (rx->irq[0] < 0) {
		dev_err(ni->dev, "RX interrupt 0 (SPI 0x54) unavailable\n");
		return -ENXIO;
	}
	dev_info(ni->dev, "RX: %d EPP interrupts requested (irq0=%d)\n",
		 got, rx->irq[0]);
	return 0;
}

int cortina_ni_rx_probe(struct cortina_ni *ni)
{
	struct cortina_ni_rx *rx;
	int ret;

	if (!ni->tx || !ni->tx->netdev)
		return dev_err_probe(ni->dev, -ENODEV,
				     "RX needs the TX netdev first\n");

	rx = devm_kzalloc(ni->dev, sizeof(*rx), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;
	rx->ni = ni;
	rx->netdev = ni->tx->netdev;
	INIT_DELAYED_WORK(&rx->recovery_work, cortina_ni_rx_recovery_work);
	/* where a chain continuation's payload starts.  Resolved once, at probe,
	 * so the receive path reads a field instead of a module parameter. */
	rx->chain_rest_off = CA_NI_RX_HDRA_OFF +
			     (rx_chain_rest_hdra ? CA_NI_RX_HDR_CPU_LEN : 0);
	ni->rx = rx;
	if (rx_chain)
		dev_info(ni->dev,
			 "RX: multi-buffer receive ON (chain payload window %u B/buffer, continuation at +0x%02x, max %u segs, max %u B)\n",
			 CA_NI_RX_BUF_USABLE_END(CA_NI_RX_CPU_POOL0_BUFSZ) -
			 CA_NI_RX_BUF_HEADROOM, rx->chain_rest_off,
			 CA_NI_RX_CHAIN_MAX_SEGS, CA_NI_RX_CHAIN_MAX_LEN);

	/* snapshot the GPHY calibration while it is in the U-Boot-proven
	 * state, and log the fault latch so a wedge already present at
	 * probe is visible in the boot log */
	cortina_ni_rx_gphy_cal_save(ni);
	dev_info(ni->dev, "RX: GPHY port %d fault latch 0x%04x at probe\n",
		 CA_NI_RX_PORT, cortina_ni_rx_gphy_fault(ni));

	/*
	 * CPU-EPP descriptor ring: STOCK puts it at a FIXED phys in the
	 * DDR-coherent reserved region (paddr reg 0x7200 = 0x0bc48000, VoQ stride
	 * 0x400) - NOT a dynamically-allocated DMA buffer (ours landed at
	 * 0x00f49000, which the QM never targets).  Map that exact region cached
	 * (the region is HW-cache-coherent, so no explicit cache ops needed).
	 */
	rx->ring_dma = CA_NI_RX_RING_PHYS;
	/* ★★★ build78: map the CPU-EPP ring UNCACHED (MEMREMAP_WC = Normal-Non-Cacheable
	 * on ARM64).  The NE DMA is NON-coherent (proven on HW: NAPI read the ring's stale
	 * CACHED poison 0xdeadbeef instead of the HW-DMA'd descriptor -> rx_errs, "PA outside
	 * pool").  build62's MEMREMAP_WB + `dma-coherent` assumption was WRONG.  WC makes CPU
	 * reads bypass the cache and see the HW writeback directly. */
	rx->ring = devm_memremap(ni->dev, CA_NI_RX_RING_PHYS,
				 CA_NI_RX_RING_TOTAL_BYTES, MEMREMAP_WC);
	if (IS_ERR_OR_NULL(rx->ring)) {
		dev_err(ni->dev, "RX ring memremap(%pa) failed\n",
			&rx->ring_dma);
		ni->rx = NULL;
		return -ENOMEM;
	}
	/* ★★ build56: SEED 0xDEADBEEF into the whole ring BEFORE arming EPP/RMU0 (stock
	 * aal_l3qm_insert_magic_number).  The HW writeback engine may refuse to write a slot
	 * that doesn't already hold the sentinel; our ring was un-seeded poison.  Fill every
	 * u32 word (a superset of stock's tail guard-band); dma_wmb() flushes it to DRAM. */
	{
		u32 *r = (u32 *)rx->ring;
		unsigned int n;

		for (n = 0; n < CA_NI_RX_RING_TOTAL_BYTES / sizeof(u32); n++)
			r[n] = 0xDEADBEEFu;
		dma_wmb();
		dev_info(ni->dev, "RX ring: seeded 0x%x u32 = 0xDEADBEEF (uncached WC) @0x%08x\n",
			 (unsigned int)(CA_NI_RX_RING_TOTAL_BYTES / sizeof(u32)),
			 (u32)CA_NI_RX_RING_PHYS);
	}

	/* ★★★ CPU-pool buffer region: like the ring, it MUST live at a FIXED phys in the
	 * NE's reserved DDR window - a dynamically dma_alloc_coherent'd buffer lands at an
	 * address the QM/RMU never targets (the exact bug that kept the ring dead), so the
	 * RMU allocates but its frame-DMA misses and it never admits (0x6900=0, no drop).
	 * Map our reserved sub-region (inside the board's 0x09000000 no-map DDR reserve,
	 * where stock's EQ14 buffers at 0x09240000 also live) at a fixed 32-bit phys so
	 * CFG0.phy_addr + every pushed buffer PA fall inside the window (CFG4.axi_top=0). */
	/* ★★★ build78: map the RX frame-buffer pool UNCACHED (MEMREMAP_WC) too - the HW
	 * DMA-writes each frame's data here and the CPU reads it in NAPI; cached (WB) would
	 * return stale data on the non-coherent NE DMA.  WC (Normal-NC) is memcpy/unaligned-
	 * safe (unlike Device ioremap), so the NAPI skb build reads the real frame bytes. */
	/* ★ The mapping spans BOTH CPU pools and the deep-queue pool that follows
	 * them, so cortina_ni_rx_frame()'s PA->VA math and bounds check cover a
	 * deep-queue buffer with no special case.  Still one fixed-phys region
	 * inside the same proven DDR reserve. */
	rx->cpu_dram_dma = CA_NI_RX_CPU_POOL_PHYS;
	rx->cpu_dram = devm_memremap(ni->dev, CA_NI_RX_CPU_POOL_PHYS,
				     CA_NI_RX_MAP_SIZE, MEMREMAP_WC);
	if (IS_ERR_OR_NULL(rx->cpu_dram)) {
		dev_err(ni->dev, "RX: CPU-pool memremap(%pa) failed\n",
			&rx->cpu_dram_dma);
		ni->rx = NULL;
		return -ENOMEM;
	}
	dev_info(ni->dev,
		 "RX: CPU-pool DRAM @%pad size %u (reserved-window; %u CPU pools + %u deep-queue @0x%08x)\n",
		 &rx->cpu_dram_dma, CA_NI_RX_MAP_SIZE, CA_NI_RX_CPU_DRAM_SIZE,
		 CA_NI_RX_DQ_DRAM_SIZE, (u32)CA_NI_RX_DQ_POOL_PHYS);

	/* U-Boot TFTP'd through port 0 and may have left the MAC RX on;
	 * force it off so nothing feeds the ring before ndo_open */
	ni_rmw(ni, CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT),
	       CA_NI_PORT_RXMAC_RX_EN, 0);

	ret = cortina_ni_rx_eq_init(ni);
	if (ret) {
		ni->rx = NULL;
		return ret;
	}
	cortina_ni_rx_epp_init(ni);
	dev_info(ni->dev, "RX: EQ pool + EPP ring init done\n");

	/* ★ FBM bring-up (the RMU buffer-allocator, must be up BEFORE RMU RX): the
	 * strict order is config+ENABLE -> FILL the pool free-list -> PRELOAD.  The pool
	 * only accepts +0x40 doorbell pushes once write-enable(bit15) is set, so enable
	 * (in fbm_init) MUST precede fill; the fire-once preload MUST follow fill.  Our
	 * fill uses our own reserved CPU-pool buffers, so it needs no L3QM pool-fill. */
	/* ★ FBM bring-up GATED default-OFF (the DMA path; flip via cortina_ni_rx.fbm_enable=1).
	 * RE-corrected: reset+config with a VALID exstack base (POOL+0x04, was 0 = phys-0
	 * DMA crash) then fill via the FBM_CPU gated doorbell (not POOL+0x40/preload). */
	if (fbm_enable) {
		cortina_ni_rx_fbm_init(ni);	/* reset + GLB/AXI/POOL config + exstack base */
		cortina_ni_rx_fbm_fill(ni);	/* FBM_CPU gated doorbell push */
	} else {
		dev_info(ni->dev, "fbm: DISABLED (safe baseline) - set cortina_ni_rx.fbm_enable=1 to bring up\n");
	}

	/* Enable the L3QM egress-scheduler master BEFORE the RX master, exactly
	 * as stock ca_ni_init_l3qm orders aal_l3qm_enable_tx(1) then
	 * aal_l3qm_enable_rx(1).  This is the drain gate that lets an enqueued
	 * descriptor reach the CPU-EPP ring; the per-CPU-port cpu_en stays off
	 * until open, so nothing is delivered yet. */
	cortina_ni_rx_es_enable(ni);

	/* master L3QM RX on - stock ca_ni_init_l3qm ends with aal_l3qm_enable_rx(1).
	 * The engine must be live before pushing buffers so the pushed PAs leave the
	 * shallow push stage into the committed bid pool.  Ingress cannot flow yet:
	 * the port-0 MAC RX stays off until open. */
	ni_rmw(ni, CA_NI_QM_RMU0_CTRL, 0, CA_NI_QM_RMU0_RX_EN);
	cortina_ni_rx_eqm_readback(ni, "after RMU0 enable, pre-populate");

	/* ★ Software-owned pools: stage their buffers now - the push stage
	 * only drains once the EQ config is committed and RMU0 runs, both of
	 * which have just happened, and the port-0 MAC RX is still off so
	 * nothing can consume a buffer before the pools are full.  A failure
	 * here is fatal to RX, so it is reported and propagated, never
	 * swallowed. */
	if (cpu_pool_push) {
		ret = cortina_ni_rx_push_seed(ni);
		if (ret) {
			ni->rx = NULL;
			return ret;
		}
	}

	/* ★★ RMU AXI reorder engine (stock runs this right after enable_rx) - the
	 * separate g_ne_axi_reo MMIO block our driver never touched; without it the RMU
	 * dequeue DMA never completes and no CPU frame is admitted. */
	cortina_ni_rx_axi_reo_init(ni);

	/* Create the RX /proc NOW so the QM/pool registers are readable live even if
	 * the pool never activates (proc_show is NULL-safe). */
	/* the narrative dump is published from cortina_ni_debugfs_init(), which
	 * runs at the end of probe - after this - so nothing is registered here */

	/* ★ NO software populate: the cpu_eq=0 pools self-populated at the EQ_CFG_LOAD
	 * commit (stock model - the QM built its own free-list over CFG0.phy_addr_start).
	 * The per-pool pa_req/inactive counters (0x6388+eqid*4) must read 0 and STAY 0;
	 * a climbing inactive counter = the pool is leaking bids again. */
	dev_info(ni->dev,
		 "RX pool self-populated (%u+%u DRAM bufs @0x%08x): eq5_pa_req=0x%08x eq6_pa_req=0x%08x wptr=0x%06x (want pa_req 0)\n",
		 CA_NI_RX_EQ_TOTAL_BUF, CA_NI_RX_EQ2_TOTAL_BUF,
		 (u32)CA_NI_RX_CPU_POOL_PHYS,
		 readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID)),
		 readl(ni_base(ni) + CA_NI_QM_EQM_PA_REQ(CA_NI_RX_EQ_ID2)),
		 cortina_ni_rx_wptr(ni));
	cortina_ni_rx_eqm_readback(ni, "self-populating pools (pa_req should stay 0)");

	/* ★★ build66: W1C-clear the LATCHED eqm_cfg_error at QM_INT_SRC 0x611c ONLY (write
	 * bit22).  Do NOT write 0x6120 - that is the INT_SRCE ENABLE MASK (0xe6d54f85), not a
	 * status latch; build65 clobbered it to 0x00400000 (disabling most int sources) - fixed
	 * by leaving it alone.  With cpu_eq=0 the eqm_cfg_error is no longer re-raised. */
	writel(BIT(22), ni_base(ni) + 0x611c);
	dev_info(ni->dev, "eqm_cfg_error W1C: int_src 0x611c=0x%08x en_mask 0x6120=0x%08x (want 0x611c bit22 CLEAR, 0x6120=0xe6d54f85)\n",
		 readl(ni_base(ni) + 0x611c), readl(ni_base(ni) + 0x6120));

	/* (FBM pool config+enable+fill+preload already done above, before RMU RX enable -
	 * the pool must accept pushes (write-enable) before fill, and it uses our own
	 * reserved buffers, not this L3QM cpu_eq=0 push.) */

	ret = cortina_ni_rx_steer_init(ni);
	if (ret) {
		/* steer failed: RX can't deliver, but don't take TX down */
		dev_err(ni->dev, "RX steer init failed (%d) - staying TX-only\n",
			ret);
		ni_rmw(ni, CA_NI_QM_RMU0_CTRL, CA_NI_QM_RMU0_RX_EN, 0);
		ni->rx = NULL;
		return 0;
	}
	dev_info(ni->dev, "RX: steer done\n");

	netif_napi_add(rx->netdev, &rx->napi, cortina_ni_rx_poll);

	ret = cortina_ni_rx_irqs_init(ni);
	if (ret) {
		/* no RX IRQ: keep TX alive rather than fail the whole device */
		dev_err(ni->dev, "RX irq init failed (%d) - staying TX-only\n",
			ret);
		netif_napi_del(&rx->napi);
		ni_rmw(ni, CA_NI_QM_RMU0_CTRL, CA_NI_QM_RMU0_RX_EN, 0);
		ni->rx = NULL;
		return 0;
	}

	/* /proc already created before the seed (NULL-safe) */

	/* devmem-verify: final golden FE-path config after probe (rxmac rx_en
	 * is added at open; the stock bit12/13/es cpu_en=0xff show here too) */
	dev_info(ni->dev,
		 "M2c RX ready (FE path): port %d -> cpu0/voq0, %u DRAM buffers\n",
		 CA_NI_RX_PORT, CA_NI_RX_EQ_TOTAL_BUF + CA_NI_RX_EQ2_TOTAL_BUF);
	dev_info(ni->dev,
		 "RX golden: static_cfg(a5c0)=0x%08x rx_cntrl=0x%08x rxmac=0x%08x es_ctrl=0x%08x l3tm=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_PORT_STATIC_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_PORT_RX_CNTRL_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_PORT_RXMAC_CFG(CA_NI_RX_PORT)),
		 readl(ni_base(ni) + CA_NI_QM_ES_CTRL),
		 readl(ni_base(ni) + CA_NI_QM_L3TM_NI_PORT_ENA));
	dev_info(ni->dev,
		 "RX golden demux: a190=0x%08x a194=0x%08x a19c=0x%08x a1a0=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX0),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX1),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX3),
		 readl(ni_base(ni) + CA_NI_NIRX_L3FE_DEMUX4));
	dev_info(ni->dev,
		 "RX delivery chain: destp9(61a4)=0x%08x prof13(615c)=0x%08x eq13_cfg1(6350)=0x%08x eq13_cfg2(6354)=0x%08x pkt_buf(6228)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_QM_DEST_PORT_EQ_CFG(CA_NI_RX_CPU_DEST_PORT)),
		 readl(ni_base(ni) + CA_NI_QM_EQ_PROFILE(CA_NI_RX_EQ_PROFILE)),
		 readl(ni_base(ni) + CA_NI_QM_CFG1_EQ(CA_NI_RX_EQ_ID)),
		 readl(ni_base(ni) + CA_NI_QM_CFG2_EQ(CA_NI_RX_EQ_ID)),
		 readl(ni_base(ni) + CA_NI_QM_DEST_PORT_PKT_BUF_CFG(CA_NI_RX_CPU_PORT)));
	dev_info(ni->dev,
		 "RX misc match: intern_portid(a1bc)=0x%08x autosync(a010)=0x%08x l2tm_glob(2210)=0x%08x\n",
		 readl(ni_base(ni) + CA_NI_NI_INTERNAL_PORT_ID_CFG),
		 readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC),
		 readl(ni_base(ni) + CA_NI_L2TM_QM_GLOB_BUF_CFG));
	return 0;
}
