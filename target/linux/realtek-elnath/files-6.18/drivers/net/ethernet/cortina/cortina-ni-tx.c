// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI Ethernet driver for the Realtek RTL9607F "Elnath" -
 * M2b TX datapath: one netdev ("eth0"), direct-TX (FE-bypass) transmit to
 * LAN port 0 through the DMA-LSO engine.  RX comes in M2c.
 *
 * Register offsets, bit semantics, init order and the descriptor encoding
 * are hardware facts recovered from the shipped RTL9607F firmware
 * (ca-ne.ko: aal_ni_init_tx_dma_lso, rtk_ni_init_tx_dma_lso,
 * aal_ni_set_dma_lso_base_depth_addr, __ca_ni_start_xmit_buf_for_fc_dirTx,
 * aal_ni_eth_port_mac_set, aal_ni_mac_autosync_cfg_set, aal_l2_qm_init,
 * aal_l2_tm_init) and cross-checked against the public CA8277B register
 * bit-field definitions.
 *
 * TX model (the "direct TX to LAN" descriptor mode of this chip generation):
 * the 8-byte ring descriptor itself carries the destination port and CoS
 * (mode=1/direct=0), the buffer is a plain Ethernet frame - no prepended
 * header.  Ring assignment is FIXED per netdev for packet order (see the
 * comment in cortina_ni_start_xmit): eth0 -> txq[CA_NI_TX_ETH_RING] (VP3),
 * PON US OMCI + WAN data -> txq[0] (VP2); TX queue 0 within each VP.
 * Completion is reported through a HW read pointer which we reclaim
 * opportunistically at xmit time plus from a periodic timer (the engine has
 * no TX-done IRQ wired in this minimal bring-up).
 */

#include <linux/crc32.h>
#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <net/arp.h>
#include <net/dsa.h>
#include <net/net_namespace.h>

#include "cortina-ni.h"

/* Fallback / revert destination for the eth0 LAN TX path, and the port whose
 * speed+duplex adjust_link mirrors.  eth0 TX is FE-bypass direct-TX: the
 * descriptor's DEST field is an LDPID that the L2FE ARB resolves to a physical
 * egress port via the PDPID map (see cortina_ni_arb_lan_map_init).  For LAN NI
 * ports ldpid == physical port (identity), so DEST=3 -> physical port 3.
 *
 * Serving all four RJ45s no longer means a fixed DEST: the port is chosen per
 * frame - see ca_ni_lan_tx_ports() and the lan_tx_mode parameter.  This value
 * remains the single-port fallback (lan_tx_mode=0, or no link information). */
#define CA_NI_TX_PORT		3
#define CA_NI_TX_COS		0
#define CA_NI_TX_TXQ		0

#define CA_NI_RECLAIM_INTERVAL	msecs_to_jiffies(10)

static bool tx_debug;
module_param(tx_debug, bool, 0644);
MODULE_PARM_DESC(tx_debug, "dump the first transmitted frames/descriptors");

/* ★ 2026-07-23 host-free HW-forward test knob: when >= 0, stamp EVERY direct-TX
 * descriptor's DEST = this ldpid instead of CA_NI_TX_PORT.  Set =0 briefly to
 * egress an AF_PACKET-injected forged frame out (uncabled) port 0, which - with
 * that port's MAC in internal loopback (PORT_STATIC_CFG bit12|15) - loops back
 * and physically ingresses the L3FE as a real LAN client frame.  Default -1 =
 * normal (DEST=port 3).  Global, so keep the window short + revert (it also
 * mis-routes the held ssh session's TX while set). */
static int force_dest_ldpid = -1;
module_param(force_dest_ldpid, int, 0644);
MODULE_PARM_DESC(force_dest_ldpid, "override direct-TX DEST ldpid for the HW-forward loopback test (-1=off)");

/* ------------------------------------------------------------------ */
/* CPU -> LAN egress port selection                                    */
/* ------------------------------------------------------------------ */

/*
 * A CPU-originated frame must leave the RJ45 the destination host is behind.
 * The descriptor's DEST field is a LAN ldpid and the ARB map is the identity for
 * LAN NI ports, so DEST *is* the physical port - the only question is which
 * value to stamp.  We answer it the way the shipped firmware does: from an
 * explicit netdev/port binding (its ca_ni_init_dev_port_mapping stamps a dest
 * LDPID per netdev, and its RX demux is port2dev[HEADER_A.lspid]).  It registers
 * one netdev per physical LAN port; we register one, so the binding lives here:
 * a DA -> port table learned from HEADER_A.lspid on RX, with a flood to every
 * LINKED port for broadcast/multicast and for a destination not yet seen.
 *
 * Deliberately NOT done by FE-forwarding the frame (dropping MODE_DIRECT +
 * FEBYPASS): that needs an L2FE flood group holding LAN members and a DFT_FWD
 * entry for the CPU source lspid.  Neither exists here (MCE_INDX[0x19] is
 * written EMPTY, MC_FIB is never written, DFT_FWD covers lspid 0..3 only) and
 * neither exists on stock either - and today's DFT_FWD value would redirect a
 * CPU-sourced flood to mcgid 0x19 = L3_LAN, i.e. straight back to the CPU.
 *
 * Flooding to a LINKED port only: a frame stamped for a dead port sits in that
 * port's egress MAC and consumes the shared L2TM buffer pool.
 */
enum {
	CA_NI_LAN_TX_FIXED	= 0,	/* every frame -> CA_NI_TX_PORT (revert) */
	CA_NI_LAN_TX_FLOOD	= 1,	/* every frame -> every linked port     */
	CA_NI_LAN_TX_LEARN	= 2,	/* learned port, flood fallback         */
};
/* ★ 2026-08-08 AOT5221ZY: SError bisect gate, see cortina_ni_tx_hw_init(). */
static int tx_stage = 3;
module_param(tx_stage, int, 0444);
MODULE_PARM_DESC(tx_stage,
		 "AOT5221ZY bisect: 0=skip tx_hw_init, 1=reset_intf only, 2=+NI global RMWs, 3=+TX-DMA engine (default)");

/* ★ 2026-08-08 AOT5221ZY: see fix #29 in cortina_ni_tx_engine_init(). */
static bool lspid_map;
module_param(lspid_map, bool, 0444);
MODULE_PARM_DESC(lspid_map, "program the VP->LSPID map table (SErrors on this SKU; default off)");

/* ★fix#60 (2026-08-09): per-VP HEADER_A for the PON (OMCI) VP - see the block at the end of
 * cortina_ni_tx_engine_init().  -1 = leave the field alone (upstream behaviour). */
static int pon_hdra_ldpid = -1;
module_param(pon_hdra_ldpid, int, 0444);
MODULE_PARM_DESC(pon_hdra_ldpid, "PON VP HDRA_CFG.hdra_ldpid (-1=leave at 0/CPU0, 15=PON)");
static int pon_hdra_cos = -1;
module_param(pon_hdra_cos, int, 0444);
MODULE_PARM_DESC(pon_hdra_cos, "PON VP HDRA_CFG.hdra_cos (-1=leave at 0, 6=US OMCI class)");
static int pon_hdra_nodrop;
module_param(pon_hdra_nodrop, int, 0444);
MODULE_PARM_DESC(pon_hdra_nodrop, "PON VP HDRA_CFG.hdra_nodrop");
static int pon_hdra_deepq;
module_param(pon_hdra_deepq, int, 0444);
MODULE_PARM_DESC(pon_hdra_deepq, "PON VP HDRA_CFG.hdra_deepq");

/* ★fix#61 (2026-08-09): send US OMCI as a single direct-TX (HP=11) descriptor instead of the
 * HP=01 header-mode SOF+EOF chain - the discriminator for "is the header-mode encoding wrong?".
 * See the block in cortina_ni_pon_tx().  Oracle: VP2's RPTR advancing. */
static int pon_direct_tx;
module_param(pon_direct_tx, int, 0444);
MODULE_PARM_DESC(pon_direct_tx, "send US OMCI as one direct-TX HP=11 descriptor (diagnostic)");
static int pon_direct_wan = 1;
module_param(pon_direct_wan, int, 0444);
MODULE_PARM_DESC(pon_direct_wan, "fix#61: set DESC1_TO_WAN on the direct-TX OMCI descriptor");
static int pon_direct_dest;
module_param(pon_direct_dest, int, 0444);
MODULE_PARM_DESC(pon_direct_dest, "fix#61: DESC1_DEST for the direct-TX OMCI descriptor");

/* ★fix#63 (2026-08-09): send US OMCI from a dma_map_single'd skb instead of the coherent
 * scratch - the "is it the buffer?" test.  See the block in cortina_ni_pon_tx(). */
static int pon_skb_tx;
module_param(pon_skb_tx, int, 0444);
MODULE_PARM_DESC(pon_skb_tx, "send US OMCI from an skb-mapped buffer, not the coherent scratch");

/* ★fix#62 (2026-08-09): which txq eth0 transmits on.  Default CA_NI_TX_ETH_RING (=1, VP3).
 * Set to 0 to put eth0 on the PON ring (VP2) - see the block in cortina_ni_start_xmit(). */
static int eth_ring = CA_NI_TX_ETH_RING;
module_param(eth_ring, int, 0444);
MODULE_PARM_DESC(eth_ring, "txq index eth0 transmits on (0=PON ring VP2, 1=default VP3)");

/*
 * ★★★★★ fix#64 (2026-08-09) — THE LAST CLEAN BISECTION: FRAME CONTENT vs ENQUEUE PATH.
 *
 * State going in (RESUME_2026-08-09 §0f): the OMCI frame wedges the DMA-LSO output stage
 * (dbg0=0x01021145, txfout_cs=2, no error latched anywhere, rptr never advances) and the
 * following are ALL exonerated on the rptr oracle - do not re-test them:
 *   HDRA_CFG dest fields (fix#60) · HP=01 header-mode encoding (fix#61) · VP2/ring/SS
 *   weights (fix#62: eth0's OWN frames retire on VP2, post-O5) · the coherent scratch
 *   (fix#63: wedges from a dma_map_single'd skb too) · egress direction (R72: wedges to a
 *   LAN port with TO_WAN cleared).
 *
 * Exactly two degrees of freedom survive between eth0's working frame and the OMCI frame:
 *   (a) THE BYTES - 16B PON header (DA/SA/etype 0xfff1) + 48B OMCI PDU, len exactly 64;
 *   (b) THE ENQUEUE PATH - cortina_ni_pon_tx() from cg_rx_omci() in NAPI softirq, versus
 *       cortina_ni_start_xmit() behind the qdisc.
 * These params cut both, plus the one interaction that spans them (the descriptor's
 * checksum-engine selector parses the frame it is pointed at).
 */
/* (a) content: 0 = the real OMCI frame; 1 = a plain broadcast Ethernet frame (etype 0x0800,
 * zero payload); 2 = the OMCI frame with ONLY the 2 etype bytes changed to 0x0800; 3 = 2 plus
 * a broadcast DA.  Mode 2 is the sharpest single-variable test in the whole bisection: two
 * bytes separate a frame that wedges from one that (may) fly. */
static int pon_fm;
module_param(pon_fm, int, 0444);
MODULE_PARM_DESC(pon_fm,
	"fix#64 US OMCI frame content: 0=real, 1=plain bcast Ethernet, 2=etype 0x0800 only, 3=etype+bcast DA");
/* Apply pon_fm only to the first N sends, then revert to the real OMCI frame.  0 =
 * apply to every send.  N>0 gives an A/B INSIDE ONE BOOT: if the first N retire and the frame
 * at N+1 wedges, the bytes are the fault with everything else held constant. */
static int pon_fn;
module_param(pon_fn, int, 0444);
MODULE_PARM_DESC(pon_fn, "fix#64: apply pon_fm to the first N sends only (0=all)");
/* The checksum-engine selector rides the direct-TX descriptor and is the one field whose
 * behaviour DEPENDS on the frame's bytes: it parses the ethertype to find L3/L4.  eth0's
 * frames (0x0800/0x0806) and the OMCI frame (0xfff1) have always been sent with the same
 * CA_NI_TX_CHK_AUTO, so "AUTO on an unparseable etype" has never been eliminated - and it
 * would stall the output stage with no error latched, which is exactly the symptom.
 * -1 = leave at CA_NI_TX_CHK_AUTO; 0 = CA_NI_TX_CHK_DISABLE. */
static int pon_chk_sel = -1;
module_param(pon_chk_sel, int, 0444);
MODULE_PARM_DESC(pon_chk_sel, "fix#64: DESC1_CHK_SEL for the OMCI descriptor (-1=AUTO, 0=disable)");
/* (b) path: hand the identical bytes to dev_queue_xmit() so they ride cortina_ni_start_xmit()
 * behind the qdisc instead of cortina_ni_pon_tx().  Pair with eth_ring=0 to hold the RING
 * constant too, leaving the code path as the ONLY difference. */
static int omci_via_xmit;
module_param(omci_via_xmit, int, 0444);
MODULE_PARM_DESC(omci_via_xmit,
	"fix#64: send US OMCI through cortina_ni_start_xmit()/qdisc instead of cortina_ni_pon_tx()");

/*
 * ★★★★★ fix#65 (2026-08-10) — ATTRIBUTION, AND THE DESCRIPTOR-FIELD BISECTION.
 *
 * R74 (fix#64, pon_fm=1 pon_fn=3) produced the result that breaks the previous model:
 *   t=19.92 send nth=0 (PLAIN broadcast Ethernet)  -> retired
 *   t=19.95 send nth=1 (PLAIN broadcast Ethernet)  -> retired      (vp2 wptr=2 rptr=2)
 *   t=22.91 send nth=2 (PLAIN broadcast Ethernet)  -> WEDGED       (vp2 wptr=4 rptr=2 forever)
 * A plain Ethernet frame - byte-identical in class to the ones that just flew - wedged the
 * engine.  ⇒ "THE BYTES ARE THE FAULT" is REFUTED, and so is the framing of fix#62: its
 * "the first OMCI frame sticks" was an INFERENCE about which descriptor stuck, never a
 * measurement (eth0 was transmitting on VP3 in the same window).
 *
 * What was never isolated is the one thing that still differs between the two descriptors:
 *   eth0 : COS = CA_NI_TX_COS (0),  DEST = a LINKED LAN port (ca_ni_lan_tx_ports), TO_WAN=0
 *   OMCI : COS = CA_NI_PON_COS (7), DEST = pon_direct_dest (0),                    TO_WAN=1
 * Both were only ever judged on PUC counters, which - per RESUME_2026-08-09 §0b - could
 * never have moved, because nothing was ever emitted for them to count.
 *
 * A destination queue that is not drained (a dead LAN port, or a fabric queue whose
 * buffer/credit is unprogrammed) accepts a frame or two into its buffer and then
 * back-pressures the shared LSO output stage FOREVER, with no drop and no latched error.
 * That is EXACTLY the observed shape - including why the first sends retire.
 */
/* Log the first N enqueues on EVERY ring, with the live engine state at enqueue time, so a
 * stuck descriptor can be attributed to its source with no inference.  This is the tool that
 * both fix#62 and fix#64 lacked. */
static int txlog;
module_param(txlog, int, 0644);
MODULE_PARM_DESC(txlog, "fix#65: log the first N TX enqueues on any ring with engine state");
/* Mute eth0 TX entirely, so the PON path is the ONLY user of the DMA-LSO engine and no
 * stall can be blamed on (or hidden by) LAN traffic.  Frames are freed and counted. */
static bool eth_off;
module_param(eth_off, bool, 0644);
MODULE_PARM_DESC(eth_off, "fix#65: drop all eth0 TX so only the PON path uses the engine");
/* ★ THE DECISIVE ONE: make the US OMCI descriptor BYTE-IDENTICAL to an eth0 descriptor -
 * direct-TX, skb-mapped, TO_WAN clear, COS = CA_NI_TX_COS, DEST = the same linked LAN port
 * ca_ni_lan_tx_ports() would pick.  Only the frame's own bytes then differ.
 *   rptr keeps tracking wptr -> the wedge is the DESTINATION/COS the frame is offered to,
 *                               i.e. a fabric queue that never drains  (then bisect with
 *                               pon_cos / pon_direct_dest, one field at a time)
 *   still wedges after 1-2    -> destination is exonerated too; the fault is in what
 *                               cortina_ni_pon_tx() does around the enqueue */
static bool pon_like_eth;
module_param(pon_like_eth, bool, 0444);
MODULE_PARM_DESC(pon_like_eth,
	"fix#65: stamp the OMCI descriptor exactly like an eth0 one (cos/dest/to_wan)");

/*
 * ★★★★★ fix#66 (2026-08-10) — PIPELINING vs QUEUE-DEPTH, settled in one send.
 *
 * R75 (pon_like_eth=1, eth0 muted, so the PON path was the engine's ONLY user) transmitted
 * SIXTEEN OMCI frames successfully and then wedged.  The per-enqueue log shows the rule
 * exactly: every frame that flew was enqueued with hw_rptr == hw_wptr (ring EMPTY), and the
 * one that wedged - idx=16 - was the first enqueued while a descriptor was still in flight
 * (pre: hw_wptr=16 hw_rptr=15, dbg0 already 0x01011140 = busy).  Two models fit:
 *   (A) PIPELINING: the engine wedges whenever a descriptor is added while it is working;
 *       the 16 successes were simply spaced far enough apart (OMCI replies are ~1/s).
 *   (B) QUEUE DEPTH: the destination queue holds ~16 frames and never drains, so the 17th
 *       offer back-pressures.  (With DEST=0 - LAN port 0, unlinked - it took only 2.)
 * They are indistinguishable in a paced stream, so PACE IT DIFFERENTLY: enqueue N copies of
 * the frame back-to-back under one doorbell, from an EMPTY ring, on the very first send.
 *   (A) predicts an immediate wedge at the 2nd descriptor (rptr stops at 1).
 *   (B) predicts all N retire, and the wedge still arrives around a cumulative 16.
 * Copies share one mapping, exactly like the eth0 flood path: only the LAST descriptor owns
 * the skb, every earlier one is marked `dup`.
 */
static int pon_burst = 1;
module_param(pon_burst, int, 0444);
MODULE_PARM_DESC(pon_burst,
	"fix#66: enqueue N back-to-back copies of each US OMCI frame under one doorbell");

/*
 * ★★★★★ fix#67 (2026-08-10) — DOES A CPU FRAME REACH THE WIRE AT ALL?
 *
 * R79 answered a question nobody had ever asked, and the answer invalidates the oracle this
 * whole investigation has been standing on.  With the OMCI frames stamped exactly like eth0's
 * (direct-TX, COS=0, DEST=3) their descriptors RETIRED - and a raw AF_PACKET sniffer on the
 * machine at the other end of the LAN cable saw ZERO of them (0 frames of ethertype 0xfff1),
 * and ZERO frames from the board's MAC 02:96:07:f0:00:01 of any kind.  The board's own
 * `ping` got 0 replies and its ARP never resolved.
 * ⇒ `rptr` advancing means only "the fabric ACCEPTED the descriptor", NOT "the frame was
 *   transmitted".  Frames are accepted, and then they die inside the chip.
 * ⇒ And the ~15-doorbell budget after which the engine wedges is the size of whatever queue
 *   they are piling up in: with eth0 muted the PON path got 15 sends (R75: 15 frames;
 *   R78: 15 sends x 4 = 60 frames - so the cap counts DOORBELLS, not frames); with eth0
 *   transmitting, eth0 consumed the budget and the PON path got only 2 (R74, R79).
 *
 * The first thing to rule out is the dumbest one: DEST is a LAN port number, the driver has
 * no MDIO (skip_mdio=1 is required on this board) and therefore no link bitmap, so it falls
 * back to CA_NI_TX_PORT=3 for every frame - and the cable may simply be on another socket.
 * This knob floods every direct-TX frame to ALL FOUR LAN ports (one descriptor each, one
 * doorbell, sharing one mapping - exactly the existing flood path), so a single ping cannot
 * miss the cabled port.
 *   sniffer sees the frame -> LAN egress works; the earlier silence was the wrong port, and
 *                             the PON destination is a separate question
 *   sniffer sees nothing    -> CPU TX egress is globally dead inside the chip, which is a
 *                             far more fundamental (and more tractable) statement than
 *                             "the OMCI frame wedges the DMA-LSO"
 */
/* ★fix#128: default ON for this target.  skip_mdio=1 is mandatory here (no usable
 * MDIO), so tx->lan_link is always 0 -> ca_ni_lan_tx_ports() hits the !link path and
 * egresses the SINGLE fallback port CA_NI_TX_PORT(3).  A client on any other RJ45
 * (e.g. port 0) then never receives the CPU's replies (ARP stays unresolved, LuCI
 * unreachable).  Flooding all 4 LAN sockets guarantees the cabled one is hit. */
static bool lan_all = true;
module_param(lan_all, bool, 0644);
MODULE_PARM_DESC(lan_all,
	"fix#67: flood every direct-TX frame to all four LAN ports (cable-port finder)");

/*
 * ★★★★★ fix#68 (2026-08-10) — TWO THINGS STOCK DOES ON EVERY CPU TRANSMIT THAT TRACK B
 * HAS NEVER DONE.  Both come from disassembling ca-ne.ko (the only software that has ever
 * moved a CPU frame on this silicon), which is the ground truth this port is supposed to
 * replicate.
 *
 * R80 established the fact that reframes everything: with frames flooded to ALL FOUR LAN
 * ports, an AF_PACKET sniffer at the other end of the cable saw NOTHING from the board -
 * not one frame from 02:96:07:f0:00:01, and the board's own ping/ARP never arrived.  So
 * `rptr` advancing means only "the fabric took the descriptor"; frames are dying inside the
 * chip, and the "wedge" is simply the queue they pile up in filling.
 *
 * (1) misc_w1c - THE txq_empty INTERRUPT BIT.  Stock W1C-clears the per-VP
 *     MISC_INTERRUPT txq_empty bit between the descriptor stores and the doorbell, EVERY
 *     time (aal_ni_clear_dma_lso_misc_interrupts @0x6530 <- __ca_ni_send_single_pkt
 *     @0xa8d5c).  Track B never writes that register, and the sampler shows the bit LATCHED
 *     at 1 on precisely the VP that goes on to wedge (R75/R78 vp2 miscint=0x1, R80 vp3
 *     miscint=0x1).  If the scheduler treats that sticky flag as "this queue is empty,
 *     don't service it", the shape is exactly what we see: frames are accepted until the
 *     queue fills, and nothing ever drains.
 *
 * (2) pon_txq - THE QUEUE INDEX.  Stock sends upstream OMCI on DMA-LSO **txq 7**, not
 *     txq 0: the tx-parameter getter's device-type-4 branch (the device whose 16-byte PON
 *     header carries ethertype 0xfff1) sets txq_index = 7 unconditionally @0xa0764, and the
 *     vendor header spells out why - CA_NI_DMA_LSO_CA_TX_TXQ_IDX 7 "TX queue ID 7 has
 *     highest priority".  Track B hardcodes CA_NI_TX_TXQ = 0 for every ring including the
 *     PON one.  Each (vp, txq) is an independent ring with its own BASE_DEPTH, WPTR/RPTR
 *     and scheduler priority, so this is not cosmetic.
 */
/*
 * ★★★ fix#78 (2026-08-10): the LSPID stock stamps in the OMCI HEADER_A.
 *
 * The vendor's own GPON driver source builds the upstream OMCI HEADER_A as
 *   cos=7, ldpid=0xf, lspid=0x11, pkt_size=msgLen+16, fe_bypass=1, no_drop=1
 * (GPL: .../rtl86900/sdk/src/dal/rtl9607f/dal_rt_rtl9607f_gpon.c:3345-3355), and live stock's
 * PUC header latch carries an lspid that is NOT our 0x10 either.  Track B has always stamped
 * CA_NI_PON_LSPID = 0x10 ("CPU0 logical port").  Now that fix#77 makes the DMA-LSO actually
 * move data, this is the last field of the OMCI header that still differs from the vendor's.
 */
static int pon_lspid = 0x11;
module_param(pon_lspid, int, 0644);
MODULE_PARM_DESC(pon_lspid,
	"fix#78: HEADER_A lspid for the DMA-LSO OMCI path (vendor GPL stamps 0x11; was 0x10)");

/*
 * ★★★ 2026-08-11: the DATA path's HEADER_A ldpid, as a knob.
 *
 * fix#78 broke the OMCI wall by changing ONE HEADER_A field to the value the vendor's own
 * driver stamps.  The upstream DATA path has never had the same treatment, and it is now
 * the prime suspect: the OMCI path routes on ldpid=0xf (the OAM logical port -> ARB
 * PPORT_OAM -> the PUC's 9th queue) and demonstrably reaches the OLT, while data routes on
 * CA_NI_PON_DATA_LDPID = 0x20+T-CONT = 0x21 and has never been witnessed leaving.
 *
 * Making it a parameter rather than editing the constant is deliberate: each boot+range to
 * O5 costs ~3 minutes, so a rebuild per candidate value is the expensive way to ask the
 * question.  With this knob one boot can A/B several encodings over a live PPPoE dial,
 * judged against the us_mib op2 upstream transmit counter.
 *
 * ⚠ Note the Track A tree (~/ak007 ca_ni_tx.c:208) records "gpon156 REFUTED (inject
 * ldpid=0x21): the L2FE IGNORES the Header-A ldpid and DLF-blackholes the CPU-source frame
 * regardless (fe_bypass not honored on Taurus)".  That was measured for OMCI INJECTION on a
 * different tree, so it does not transfer unexamined - but if it holds for data too, then
 * no ldpid value will help and the fix has to make the L2FE resolve a CPU-source frame to
 * the PON destination (static FDB / MC-FIB), not relabel the header.  -1 = leave the
 * compiled-in CA_NI_PON_DATA_LDPID.
 */
static int pon_data_ldpid = -1;
module_param(pon_data_ldpid, int, 0644);
MODULE_PARM_DESC(pon_data_ldpid,
	"HEADER_A ldpid for the UPSTREAM DATA path (-1 = compiled default 0x20+tcont; try 0xf = the proven OMCI/OAM route)");
/* ★fix#121: the DMA-LSO STREAMID entry that stamps ldpid 0x21 onto the host-injected US frame ALSO
 * carries deep_q in bit0 - which cortina_ni_rx_streamid_init omitted, so the BM header deep_q=0.
 * RMU0 admits ONLY deep_q frames (rx.c ~2711: "RMU0 admit is gated on deep_q, NOT cpu_flg"), so a
 * deep_q=0 frame is SILENTLY never a QM candidate - no drop fires (matches every hard fact: bm_tx
 * climbs, RMU0_RX 0x67d8=0, TE_CB_VOQ_BUFCNT=0, all INT_SRC/no_buf/taildrop=0). Stamp deep_q here. */
static int sid_deepq = 1;
module_param(sid_deepq, int, 0644);
MODULE_PARM_DESC(sid_deepq,
	"fix#121: set DMA-LSO STREAMID deep_q bit (BIT0) on the ldpid-0x21 US entries so RMU0 admits them (default 1)");

/*
 * ★★★ THE DOCUMENTED DIVERGENCE FROM THE VENDOR, as a knob.
 *
 * cortina-ni-regs.h:1393-1398 says it against us in our own tree: "Vendor data TX uses
 * fe_bypass=0 + pol_en=1/pol_id=gem-idx with a fully-programmed FE; ours keeps the proven
 * OMCI-inject convention: fe_bypass=1, no policer."  That convention was inherited from
 * the OMCI path because it WORKED THERE - but OMCI reaches the fibre via ldpid 0xf ->
 * PPORT_OAM, a route with no DA lookup, so it never had to care whether the bypass bit is
 * real.  Data goes ldpid 0x21 -> PPORT_QM, which does.
 *
 * And Track A measured that it is NOT real: "the L2FE IGNORES the Header-A ldpid and
 * DLF-blackholes the CPU-source frame regardless (fe_bypass not honored on Taurus)"
 * (~/ak007 ca_ni_tx.c:208-210).  If a CPU-source frame takes a normal L2 lookup instead of
 * the bypass it asked for, a BROADCAST PADI floods the LAN bridge domain and never egresses
 * upstream - "enqueued, retired, never on the fibre", which is exactly what the us_mib
 * witness reports.
 *
 * 1 = keep the current (OMCI-inherited) convention; 0 = the vendor's data convention.
 * Judge the change ONLY against the us_mib op2 delta with an omci_ping control on each
 * side - not against "no PADO", which cannot distinguish this from ten other faults.
 */
static int pon_data_fe_bypass = 1;
module_param(pon_data_fe_bypass, int, 0644);
MODULE_PARM_DESC(pon_data_fe_bypass,
	"HEADER_A fe_bypass for the UPSTREAM DATA path (1 = current OMCI-inherited convention, 0 = the vendor's data convention)");

/*
 * ★fix#113 rank-3 ride-along: HEADER_A pol_en + pol_id.  Stock's get_pon_paras
 * DATA branch sets pol_en=1 with pol_id = the US hw-GEM index, and CLEARS pol_en
 * for OMCI (which is why OMCI needs no GEM selector - the PUC's US-OMCI header-A
 * replacement supplies it).  The port already stamps pol_id (=data VoQ 8) but
 * never sets pol_en, so the PUC has no validated US-GEM to encapsulate the burst
 * in.  0644 so it A/B's live with zero reboots; default off = unchanged.
 */
static bool pon_data_pol_en = true;	/* ★2026-08-18m: default ON - PROVEN (pontx x3 -> us_mib data[8]=3) that
					 * pon_data_pol_en=1 stamps HEADER_A pol_id=8 -> the DATA GEM (225/T-CONT 1).
					 * The data-US-TX path (cortina_ni_pon_data_tx=gpon0 xmit) egresses on the
					 * data GEM only with this on; default off left it on the OMCC T-CONT. */
module_param(pon_data_pol_en, bool, 0644);
MODULE_PARM_DESC(pon_data_pol_en,
	"fix#113 r3: set HEADER_A pol_en (US-GEM selector valid) on the UPSTREAM DATA path (default off)");
static uint pon_data_polid = CA_NI_PON_DATA_TCONT * 8 + CA_NI_PON_DATA_COS;
module_param(pon_data_polid, uint, 0644);
MODULE_PARM_DESC(pon_data_polid,
	"fix#113 r3: HEADER_A pol_id (US hw-GEM index) for the DATA path when pon_data_pol_en=1 (default = data VoQ 8)");

static bool lso_stock = true;
module_param(lso_stock, bool, 0444);
MODULE_PARM_DESC(lso_stock,
	"fix#77: set the DMA-LSO SRAM-test/light-sleep/CCI_MAP globals to the live-stock capture (0 = old behaviour)");

static bool misc_w1c = true;
module_param(misc_w1c, bool, 0644);
MODULE_PARM_DESC(misc_w1c,
	"fix#68: clear the per-VP MISC_INTERRUPT txq_empty bit before every doorbell (stock)");

static int pon_txq = CA_NI_TX_TXQ;
module_param(pon_txq, int, 0444);
MODULE_PARM_DESC(pon_txq,
	"fix#68: DMA-LSO txq index for the PON/OMCI ring (stock uses 7; default 0 = legacy)");

static int lan_tx_mode = CA_NI_LAN_TX_LEARN;
module_param(lan_tx_mode, int, 0644);
MODULE_PARM_DESC(lan_tx_mode,
	"CPU->LAN egress port: 0=fixed CA_NI_TX_PORT (pre-multi-port behaviour), 1=flood every frame to all linked LAN ports (bring-up/bisect only, 4x TX cost), 2=per-DA learned port with flood fallback (default)");

/* one bucket, published atomically: {mac[47:0], port[50:48], valid[51]} */
#define CA_NI_LAN_FDB_SIZE	ARRAY_SIZE(((struct cortina_ni_tx *)0)->lan_fdb)
#define CA_NI_LAN_FDB_MAC	GENMASK_ULL(47, 0)
#define CA_NI_LAN_FDB_PORT	GENMASK_ULL(50, 48)
#define CA_NI_LAN_FDB_VALID	BIT_ULL(51)

/* explicit byte math: this driver must stay endianness-agnostic */
static u64 ca_ni_mac_key(const u8 *mac)
{
	return ((u64)mac[0] << 40) | ((u64)mac[1] << 32) | ((u64)mac[2] << 24) |
	       ((u64)mac[3] << 16) | ((u64)mac[4] << 8) | mac[5];
}

static unsigned int ca_ni_mac_bucket(const u8 *mac)
{
	return (mac[3] ^ mac[4] ^ mac[5]) & (CA_NI_LAN_FDB_SIZE - 1);
}

/*
 * Bind @sa to the RJ45 it arrived on.  @lspid is HEADER_A.lspid.
 *
 * ★ The link check is the SAFETY GUARD on the one fact this rests on.  If lspid
 * turned out NOT to be the ingress port it would read a constant (typically 0),
 * which would bind every host to one port and kill LAN egress.  Refusing to
 * learn a port whose PHY is down means such a value is never learned at all, so
 * we keep flooding - which works.
 */
void cortina_ni_lan_tx_learn(struct cortina_ni *ni, const u8 *sa, u32 lspid)
{
	struct cortina_ni_tx *tx = ni->tx;
	unsigned int b;
	u64 ent;

	if (!tx || lan_tx_mode != CA_NI_LAN_TX_LEARN)
		return;
	if (lspid >= CA_NI_LAN_PORT_COUNT ||
	    !(READ_ONCE(tx->lan_link) & BIT(lspid)))
		return;
	if (is_multicast_ether_addr(sa) || is_zero_ether_addr(sa))
		return;

	ent = FIELD_PREP(CA_NI_LAN_FDB_MAC, ca_ni_mac_key(sa)) |
	      FIELD_PREP(CA_NI_LAN_FDB_PORT, lspid) | CA_NI_LAN_FDB_VALID;
	b = ca_ni_mac_bucket(sa);
	if (READ_ONCE(tx->lan_fdb[b]) == ent)
		return;			/* unchanged - the common case */
	WRITE_ONCE(tx->lan_fdb[b], ent);
	tx->lan_learn++;
}

/*
 * Publish the set of RJ45s with a PHY link.  A change flushes every binding -
 * which is exactly the operator's cable-move test: unplug LAN1, plug LAN2, and
 * no stale DA->port binding may survive.
 */
void cortina_ni_lan_tx_link_set(struct cortina_ni *ni, u32 link)
{
	struct cortina_ni_tx *tx = ni->tx;
	unsigned int i;

	if (!tx || READ_ONCE(tx->lan_link) == link)
		return;
	dev_info(ni->dev, "lan_tx: LAN link set 0x%x -> 0x%x, flushing DA bindings\n",
		 READ_ONCE(tx->lan_link), link);
	WRITE_ONCE(tx->lan_link, link);
	for (i = 0; i < CA_NI_LAN_FDB_SIZE; i++)
		WRITE_ONCE(tx->lan_fdb[i], 0);
	tx->lan_flush++;
}

static int ca_ni_lan_fdb_lookup(struct cortina_ni_tx *tx, const u8 *da)
{
	u64 ent = READ_ONCE(tx->lan_fdb[ca_ni_mac_bucket(da)]);

	if (!(ent & CA_NI_LAN_FDB_VALID) ||
	    FIELD_GET(CA_NI_LAN_FDB_MAC, ent) != ca_ni_mac_key(da))
		return -1;
	return FIELD_GET(CA_NI_LAN_FDB_PORT, ent);
}

/* The egress port set for one frame, as a port bitmap (never empty). */
static u32 ca_ni_lan_tx_ports(struct cortina_ni_tx *tx, const u8 *da)
{
	u32 link = READ_ONCE(tx->lan_link);
	int port;

	/* Diagnostic knob still wins, but RANGE-CHECKED: the port set is a
	 * bitmap now, and BIT() of an out-of-range value is 0 = "no port",
	 * which would map an skb and attach it to no descriptor at all (a DMA +
	 * skb leak, and the frame silently vanishes).  The DEST field is 4 bits
	 * wide, so anything outside 0..15 could never have been stamped anyway. */
	/* fix#67: flood all four LAN sockets, so a frame cannot miss the cabled one
	 * (there is no MDIO on this board, hence no link bitmap to pick from). */
	if (lan_all)
		return GENMASK(CA_NI_LAN_PORT_COUNT - 1, 0);

	if (force_dest_ldpid >= 0) {
		if (force_dest_ldpid < CA_NI_TX_DEST_LDPID_COUNT)
			return BIT(force_dest_ldpid);
		WARN_ONCE(1, "force_dest_ldpid=%d out of range 0..%d, ignored\n",
			  force_dest_ldpid, CA_NI_TX_DEST_LDPID_COUNT - 1);
	}
	/* Fail-safe: fall back to the single-port behaviour whenever we have no
	 * link information at all (the 1 Hz poll has not run yet, or MDIO
	 * failed).  Never "no port" - that would silence the only IP management
	 * channel to the board. */
	if (lan_tx_mode == CA_NI_LAN_TX_FIXED || !link)
		return BIT(CA_NI_TX_PORT);

	if (lan_tx_mode == CA_NI_LAN_TX_LEARN &&
	    !is_multicast_ether_addr(da)) {
		port = ca_ni_lan_fdb_lookup(tx, da);
		if (port >= 0 && (link & BIT(port))) {
			tx->lan_hit++;
			return BIT(port);
		}
	}
	tx->lan_flood++;
	return link;
}

/* fallback MAC when the DT carries none (locally administered) */
static const u8 cortina_ni_default_mac[ETH_ALEN] = {
	0x02, 0x96, 0x07, 0xf0, 0x00, 0x01
};

static inline void __iomem *ni_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_NI];
}

/* ★fix#54: program the DMA-LSO STREAMID table with stock's shape (see cortina-ni-regs.h). */
static bool streamid_init = true;
module_param(streamid_init, bool, 0444);
MODULE_PARM_DESC(streamid_init,
		 "write the DMA-LSO STREAMID table with stock's shape (8 live entries, rest disabled)");

/* ★fix#54b: the COS stamped into the US OMCI HEADER_A.  This port uses CA_NI_PON_COS=7;
 * the live-stock golden hdr0 (0x00203108) carries cos=0, and with stock's STREAMID table a
 * cos=7 frame indexes sid[7] while stock's own OMCI indexes sid[0].  Track A's RE flagged
 * "the frame's own cos 7 -> 0" as the next single variable to try after the table shape. */
static int pon_cos = -1;
module_param(pon_cos, int, 0444);
MODULE_PARM_DESC(pon_cos,
		 "COS for the US OMCI HEADER_A (-1 = CA_NI_PON_COS default 7; stock golden hdr0 has cos=0)");

static inline void __iomem *dma_base(struct cortina_ni *ni)
{
	return ni->win[CA_NI_WIN_DMA];
}

static inline void ni_rmw(struct cortina_ni *ni, u32 off, u32 clr, u32 set)
{
	writel((readl(ni_base(ni) + off) & ~clr) | set, ni_base(ni) + off);
}

static inline void dma_rmw(struct cortina_ni *ni, u32 off, u32 clr, u32 set)
{
	writel((readl(dma_base(ni) + off) & ~clr) | set, dma_base(ni) + off);
}

/* ------------------------------------------------------------------ */
/* Mandatory HW init (the stock aal_ni_init/l2_qm/l2_tm subset)        */
/* ------------------------------------------------------------------ */

/*
 * NI block reset handshake (stock aal_ni_reset): wait for the NI self-init
 * done flag, then deassert every interface reset.  U-Boot already did this
 * (it TFTPs through the NI) so both are expected to be settled - soft-warn.
 */
static void cortina_ni_tx_reset_intf(struct cortina_ni *ni)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(ni_base(ni) + CA_NI_HV_INIT_DONE, val,
				 val & CA_NI_HV_INIT_DONE_NI,
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(ni->dev, "NI init-done not set (0x%08x), continuing\n",
			 val);

	writel(0, ni_base(ni) + CA_NI_HV_INTF_RST);
	val = readl(ni_base(ni) + CA_NI_HV_INTF_RST);	/* stock reads back */
	if (val)
		dev_warn(ni->dev, "INTF_RST readback 0x%08x != 0\n", val);
}

/*
 * ★★★★ 2026-08-08 fix#54: program the DMA-LSO STREAMID table with STOCK'S SHAPE.
 * See the CA_DMA_LSO_STREAMID_ACCESS comment for the register identity (0x0e0/0x0e4 are
 * STREAMID on Taurus, not LSPID_MAP) and the bitfield.  This driver has never written this
 * table at all, so it sits at whatever reset/U-Boot left - a different baseline from Track
 * A, whose port wrote all 256 entries live with a bogus ldpid.
 *   stock: sid[0..7] = en_flag 1, ldpid 0x21, cos v, pol_id 8+v ; sid[8..255] = 0
 * ⚠ Never read this table back to check it - the read path returns garbage on this die.
 * The GO bit is polled on ACCESS, which is a real register here.
 */
static void cortina_ni_tx_streamid_init(struct cortina_ni *ni)
{
	unsigned int i;
	u32 val;

	if (!streamid_init)
		return;

	for (i = 0; i < CA_DMA_LSO_STREAMID_ENTRIES; i++) {
		u32 e = 0;

		if (i < CA_DMA_LSO_SID_STOCK_LIVE)
			e = CA_DMA_LSO_SID_EN |
			    (sid_deepq ? CA_DMA_LSO_SID_DEEP_Q : 0) |	/* ★fix#121 */
			    FIELD_PREP(CA_DMA_LSO_SID_LDPID,
				       CA_DMA_LSO_SID_STOCK_LDPID) |
			    FIELD_PREP(CA_DMA_LSO_SID_COS, i) |
			    FIELD_PREP(CA_DMA_LSO_SID_POL_ID, 8 + i);

		writel(e, dma_base(ni) + CA_DMA_LSO_STREAMID_DATA);
		writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE | i,
		       dma_base(ni) + CA_DMA_LSO_STREAMID_ACCESS);
		if (readl_poll_timeout(dma_base(ni) + CA_DMA_LSO_STREAMID_ACCESS,
				       val, !(val & CA_DMA_LSO_BD_ACCESS_GO),
				       CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US)) {
			dev_warn(ni->dev,
				 "streamid: entry %u did not complete (access=0x%08x) - stopping\n",
				 i, readl(dma_base(ni) + CA_DMA_LSO_STREAMID_ACCESS));
			return;
		}
	}
	dev_info(ni->dev,
		 "fix#54: STREAMID table written, stock shape (sid[0..7] en=1 ldpid=0x%02x cos=v pol_id=8+v, sid[8..255]=0; sid0 word=0x%08x)\n",
		 CA_DMA_LSO_SID_STOCK_LDPID,
		 (u32)(CA_DMA_LSO_SID_EN |
		       FIELD_PREP(CA_DMA_LSO_SID_LDPID, CA_DMA_LSO_SID_STOCK_LDPID) |
		       FIELD_PREP(CA_DMA_LSO_SID_POL_ID, 8)));
}

/* 07f VP->LSPID map (stock rtk_ni_init_tx_dma_lso): VP n sources from CPU
 * logical port 0x10+n (n=1..11), others from CPU port 0x10; all valid. */
static int cortina_ni_tx_lspid_map_init(struct cortina_ni *ni)
{
	int i, ret;
	u32 val, lspid;

	for (i = 0; i < CA_DMA_LSO_LSPID_MAP_ENTRIES; i++) {
		lspid = (i >= 1 && i <= 11) ? CA_DMA_LSO_LSPID_CPU0 + i
					    : CA_DMA_LSO_LSPID_CPU0;

		writel(0, dma_base(ni) + CA_DMA_LSO_LSPID_MAP_DATA1);
		writel(CA_DMA_LSO_LSPID_MAP_VALID |
		       FIELD_PREP(CA_DMA_LSO_LSPID_MAP_LSPID, lspid),
		       dma_base(ni) + CA_DMA_LSO_LSPID_MAP_DATA0);
		writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE |
		       FIELD_PREP(CA_DMA_LSO_LSPID_MAP_IDX, i),
		       dma_base(ni) + CA_DMA_LSO_LSPID_MAP_ACCESS);

		ret = readl_poll_timeout(dma_base(ni) +
					 CA_DMA_LSO_LSPID_MAP_ACCESS, val,
					 !(val & CA_DMA_LSO_BD_ACCESS_GO),
					 CA_NI_TX_POLL_US,
					 CA_NI_TX_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(ni->dev, "lspid map[%d] write timed out\n", i);
			return ret;
		}
	}
	return 0;
}

/*
 * Global TX-engine enable - the "silent stall" block: without these the
 * descriptors are consumed but no frame ever moves (stock
 * aal_ni_init_tx_dma_lso + the 07f-only rtk_ni_init_tx_dma_lso extras).
 */

/* ★ 2026-08-08 AOT5221ZY: async-SError localiser (same idea as ni_bar() in
 * cortina-ni-rx.c).  dsb+isb force a pending async abort to be TAKEN here, so the
 * LAST "tx-bar:" printed names the step that actually faulted. */
static void tx_bar(struct cortina_ni *ni, const char *tag)
{
	dsb(sy);
	isb();
	dev_info(ni->dev, "tx-bar: %s\n", tag);
}

static int cortina_ni_tx_engine_init(struct cortina_ni *ni)
{
	void __iomem *dma = dma_base(ni);
	void __iomem *reo = ni->win[CA_NI_WIN_AXI_REO];
	int i, ret;

	/*
	 * ★★★★★ fix#77 (2026-08-10) — THREE WRITES THAT LAND ON THE DMA-LSO's SRAM
	 * TEST / LIGHT-SLEEP CONTROLS, AND ONE THAT IS MISSING.
	 *
	 * The DMA-LSO window has now been captured from LIVE STOCK while stock was
	 * emitting OMCI (golden_2026-08-10_STOCK_DMALSO_187.txt - the golden this
	 * project never had; the 2026-08-09 workflow called for it explicitly and it
	 * was never taken).  Stock reads:
	 *     0x0b8/0x0bc/0x0c0 SRAM_TEST_CONTROL_0/1/2 = 0x02020202 each
	 *     0x0c4             SRAM_LS_CONTROL         = 0x00000000
	 *     0x0c8             CCI_MAP                 = 0x00000002
	 *     0x0cc             CFG_LENFIX_EN           = 0x00000001
	 * We were doing, on the same silicon:
	 *     rmw(0x0bc, 0xff, 0)      -> clears a byte of SRAM_TEST_CONTROL_1
	 *     writel(0x1,        0x0c0) -> overwrites SRAM_TEST_CONTROL_2 (3 of 4 bytes)
	 *     writel(0xc0007777, 0x0c4) -> junk into SRAM_LIGHT_SLEEP control
	 *     CCI_MAP never written at all
	 * Those macros are the ELNATH/generic addresses ("MISC_C0/C4", "lenfix_en"),
	 * carried over before the Taurus relocation was understood - the same defect
	 * class as fix#33/#35/#50.  Putting the block's SRAMs into a test/retention
	 * mode while leaving the coherency map unset is a mechanism that fetches
	 * descriptors and then moves no data, which is exactly what we measure: the
	 * engine consumes descriptors and NOTHING reaches the wire, on any port
	 * (R79/R80's sniffer).
	 *
	 * cortina_ni.lso_stock=0 restores the old behaviour for an A/B.
	 */
	if (lso_stock) {
		writel(0x02020202, dma + CA_DMA_LSO_SRAM_TEST_CTRL0);
		writel(0x02020202, dma + CA_DMA_LSO_SRAM_TEST_CTRL1);
		writel(0x02020202, dma + CA_DMA_LSO_SRAM_TEST_CTRL2);
		writel(0x00000000, dma + CA_DMA_LSO_SRAM_LS_CTRL);
		writel(0x00000002, dma + CA_DMA_LSO_CCI_MAP);
		writel(0x00000001, dma + CA_DMA_LSO_CFG_LENFIX_EN);
		writel(0x0000003f, dma + CA_DMA_LSO_INTENABLE);
		dev_info(ni->dev,
			 "fix#77: DMA-LSO globals set to live stock: sram_test=%08x/%08x/%08x ls=%08x cci_map=%08x lenfix=%08x inten=%08x\n",
			 readl(dma + CA_DMA_LSO_SRAM_TEST_CTRL0),
			 readl(dma + CA_DMA_LSO_SRAM_TEST_CTRL1),
			 readl(dma + CA_DMA_LSO_SRAM_TEST_CTRL2),
			 readl(dma + CA_DMA_LSO_SRAM_LS_CTRL),
			 readl(dma + CA_DMA_LSO_CCI_MAP),
			 readl(dma + CA_DMA_LSO_CFG_LENFIX_EN),
			 readl(dma + CA_DMA_LSO_INTENABLE));
	} else {
		dma_rmw(ni, CA_DMA_LSO_SRAM_TEST_CTRL1, 0xff, 0);
	}
	tx_bar(ni, "sram_test");

	/* enable all 8 TX queues of every DMA-LSO VP (stock does all 12) */
	for (i = 0; i < CA_DMA_LSO_VP_COUNT; i++)
		writel(CA_DMA_LSO_VP_TXQ_ALL_EN,
		       dma + CA_DMA_LSO_VP_CONTROL(i));
	tx_bar(ni, "vp_txq");

	/* AXI master: outstanding transactions + cacheline transfers */
	writel(readl(dma + CA_DMA_AXIM2_CONFIG) | CA_DMA_AXIM2_CONFIG_BITS,
	       dma + CA_DMA_AXIM2_CONFIG);
	tx_bar(ni, "axim2");

	/* ★ 2026-07-15: NON-coherent read attributes for all VPs.  Stock writes the
	 * coherent/ACE pattern (CA_DMA_LSO_AXI_USER_PAT_VAL) and its fabric snoops;
	 * on our kernel the ACE path is dead (QM 0x611c bit30 proved it for the EPP
	 * writeback) and the ACE descriptor fetch never completed - the TXQ rptr sat
	 * at 0 while the doorbell wptr climbed, so nothing ever transmitted.  The TX
	 * ring/buffers are cache-maintained by the DMA API instead (no dma-coherent
	 * on the NE DT node). */
	writel(CA_DMA_LSO_AXI_USER_SEL0_VAL, dma + CA_DMA_LSO_AXI_USER_SEL0);
	for (i = 0; i < 4; i++)
		writel(CA_DMA_LSO_AXI_USER_PAT_NOCOH,
		       dma + CA_DMA_LSO_AXI_USER_PAT0 + i * 4);
	tx_bar(ni, "user_sel");

	/* scheduler/shaper global TX enable */
	/* ★ 2026-08-08 AOT5221ZY (fix #27): TX-enable MOVED OUT -- see
	 * cortina_ni_tx_engine_enable(), called AFTER cortina_ni_tx_rings_init(). */

	/* TX DMA enable, burst 64x64bit, HW pad of short frames */
	/* ★ fix #27: LSO_CTRL (TX DMA enable) also moved out -- see below. */

	/* 07f-only: AXI reorder slots for the DMA-LSO read path */
	if (reo) {
		for (i = 0; i < CA_AXI_REO_SLOT_COUNT; i++)
			writel(CA_AXI_REO_SLOT_VAL, reo + CA_AXI_REO_SLOT(i));
	tx_bar(ni, "axi_reo_loop");
	} else {
		dev_warn(ni->dev,
			 "axi-reo window unmapped, skipping reorder cfg\n");
	}

	/* 07f-only: VLAN tag type (stock reads 0x800088a8 here - matches) */
	writel(CA_DMA_LSO_VLAN_TAG_TYPE0_VAL, dma + CA_DMA_LSO_VLAN_TAG_TYPE0);
	tx_bar(ni, "vlan_tag0");
	/* ★ fix#77: the old "MISC_C0/MISC_C4" writes went here.  On Taurus those
	 * offsets are SRAM_TEST_CONTROL_2 and SRAM_LS_CONTROL - see the block above,
	 * where the whole group is now set to stock's values instead. */
	if (!lso_stock) {
		writel(CA_DMA_LSO_MISC_C0_VAL, dma + CA_DMA_LSO_MISC_C0);
		writel(CA_DMA_LSO_MISC_C4_VAL, dma + CA_DMA_LSO_MISC_C4);
	}
	tx_bar(ni, "misc_c4");

	tx_bar(ni, "pre-lspid_map");
	/*
	 * ★ 2026-08-08 AOT5221ZY (fix #29): SKIPPABLE.  tx_bar() proved the
	 * `Asynchronous SError 0xbe000011` happens INSIDE this call (pre-lspid_map
	 * prints, post-lspid_map never does), and it is NOT the DATA0/DATA1 swap
	 * (fix #28 corrected those and the fault did not move).  This table only maps
	 * VP -> LSPID for LAN TX; the PON US-OMCI path (cortina_ni_pon_tx) may not need
	 * it.  Skip it to get a working TX engine, then come back and fix the table.
	 * Re-enable with cortina_ni.lspid_map=1.
	 */
	if (lspid_map) {
		ret = cortina_ni_tx_lspid_map_init(ni);
		tx_bar(ni, "post-lspid_map");
		if (ret)
			return ret;
	} else {
		dev_info(ni->dev, "lspid_map=0: VP->LSPID table init SKIPPED\n");
	}

	/* ★fix#54: the table that DOES exist at 0x0e0/0x0e4 on this die is STREAMID, not
	 * LSPID_MAP - program it with stock's shape.  Must run AFTER the lspid_map block,
	 * because with lspid_map=1 that loop writes STREAMID ACCESS as a side effect and
	 * would otherwise leave stray entries behind it. */
	cortina_ni_tx_streamid_init(ni);
	tx_bar(ni, "post-streamid");

	/* stock's final LSO_CTRL state: keep the source LSPID from the map
	 * table, HW-pad via AFT below instead of lso_padding (0x2d -> 0x1d) */
	dma_rmw(ni, CA_DMA_LSO_CTRL, CA_DMA_LSO_CTRL_PAD_EN,
		CA_DMA_LSO_CTRL_LSPID_KEEP);
	tx_bar(ni, "lso_ctrl_rmw");

	/* HW short-frame pad to 64 bytes */
	dma_rmw(ni, CA_DMA_AFT_CTRL, CA_DMA_AFT_PAD_SIZE,
		CA_DMA_AFT_PAD_EN |
		FIELD_PREP(CA_DMA_AFT_PAD_SIZE, CA_DMA_AFT_PAD_SIZE_VAL));
	tx_bar(ni, "aft_ctrl_rmw");

	/* FE-bypass enable per VP - reset default is 0 = frames routed into
	 * the (uninitialized) forwarding engine and dropped */
	for (i = 0; i < CA_DMA_LSO_VP_COUNT; i++)
		dma_rmw(ni, CA_DMA_LSO_VP_HDRA_CFG(i),
			CA_DMA_LSO_HDRA_LDPID, CA_DMA_LSO_HDRA_FEBYPASS);
	tx_bar(ni, "hdra_cfg_loop");

	/*
	 * ★★★★★ fix#60 (2026-08-09) — GIVE THE PON VP A REAL DESTINATION.
	 *
	 * The loop above is a NO-OP on this silicon: HDRA_CFG's reset default is already
	 * 0x00000200 (febypass=1, everything else 0), so it clears an already-zero LDPID and
	 * sets an already-set FEBYPASS.  fix#59 confirmed it live — every VP reads exactly
	 * 0x00000200 from probe to t=148 s.
	 *
	 * That is a real defect, because **with hdra_febypass=1 the forwarding engine is
	 * skipped and the fabric lifts the egress destination from hdra_ldpid** — which is 0,
	 * i.e. CPU port 0, not the PON.  Track A's driver says the same thing in its own words:
	 * "for a DMA-LSO frame the QM lifts the egress destination from HDRA_CFG[VP].hdra_ldpid
	 * (the FE is bypassed), so OMCI needs its OWN VP whose HDRA_CFG points at the PON", and
	 * "the fe_bypassed frame's dest resolves to hdra_ldpid=0=CPU0 (dpid16) not the PON".
	 *
	 * fix#59 measured the consequence: the engine FETCHES our descriptor
	 * (fdes==cdes==VP2 ring base, desw/para latched) and then stalls in its output stage
	 * with dbg0=0x01021145 (txfout_cs=2, the LSO->fabric handoff) and NO error latched
	 * anywhere — a clean back-pressure wedge, not a drop.  A frame offered to a destination
	 * the fabric will not accept is exactly what that looks like, and it also explains why
	 * every PUC/ARB/L2FE counter stayed at 0: nothing was ever emitted to count.
	 *
	 * ★ ORACLE FOR THIS CHANGE: **VP2's RPTR advancing**, not any PUC counter.  Several
	 * earlier dest-resolution experiments were judged on PUC counters that could not have
	 * moved regardless, so their refutations do not carry over.  Corroborator:
	 * dbg0.txfout_cs returning to idle (see cortina_ni_tx_dma_dump / dma_dump_s).
	 *
	 * Applied to the PON VP ONLY; the LAN VPs keep the existing behaviour (eth0 TX works).
	 * All four fields are module params so the space can be swept in ONE build:
	 *   pon_hdra_ldpid  -1 = leave at 0 (upstream behaviour); 15 = the PON ldpid
	 *   pon_hdra_cos    -1 = leave;      6 = what the PUC classifies as US OMCI
	 *   pon_hdra_nodrop / pon_hdra_deepq  0/1
	 */
	if (pon_hdra_ldpid >= 0 || pon_hdra_cos >= 0 ||
	    pon_hdra_nodrop || pon_hdra_deepq) {
		u8 pon_vp = CA_NI_TX_VP_BASE;	/* txq[0] = the OMCI ring */
		u32 clr = 0, set = CA_DMA_LSO_HDRA_FEBYPASS;

		if (pon_hdra_ldpid >= 0) {
			clr |= CA_DMA_LSO_HDRA_LDPID;
			set |= FIELD_PREP(CA_DMA_LSO_HDRA_LDPID, pon_hdra_ldpid);
		}
		if (pon_hdra_cos >= 0) {
			clr |= CA_DMA_LSO_HDRA_COS;
			set |= FIELD_PREP(CA_DMA_LSO_HDRA_COS, pon_hdra_cos);
		}
		if (pon_hdra_nodrop)
			set |= CA_DMA_LSO_HDRA_NODROP;
		if (pon_hdra_deepq)
			set |= CA_DMA_LSO_HDRA_DEEPQ;

		dma_rmw(ni, CA_DMA_LSO_VP_HDRA_CFG(pon_vp), clr, set);
		dev_info(ni->dev,
			 "fix#60: PON VP%u HDRA_CFG = %08x (ldpid=%d cos=%d nodrop=%d deepq=%d)\n",
			 pon_vp,
			 readl(dma_base(ni) + CA_DMA_LSO_VP_HDRA_CFG(pon_vp)),
			 pon_hdra_ldpid, pon_hdra_cos, pon_hdra_nodrop,
			 pon_hdra_deepq);
	}

	return 0;
}

/* program one VP/TXQ descriptor-ring base+depth via the indirect window
 * (stock aal_ni_set_dma_lso_base_depth_addr) */
static int cortina_ni_tx_ring_program(struct cortina_ni *ni, u8 vp, u8 txq,
				      dma_addr_t base)
{
	void __iomem *dma = dma_base(ni);
	u32 val;
	int ret;

	dev_info(ni->dev, "ring-program vp%u txq%u base=%pad\n", vp, txq, &base);
	if (WARN_ON(upper_32_bits(base) || (base & 0xf)))
		return -EINVAL;

	writel((lower_32_bits(base) & ~0xf) |
	       FIELD_PREP(CA_DMA_LSO_BD_DATA0_DEPTH, CA_NI_TX_RING_DEPTH),
	       dma + CA_DMA_LSO_VP_BD_DATA0(vp));
	/* addr[39:32] = 0: ring sits below 4 GB, and stock writes 0 here
	 * (its "2" branch is the disabled dma_lso_ace_test path) */
	writel(0, dma + CA_DMA_LSO_VP_BD_DATA1(vp));
	writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE |
	       FIELD_PREP(CA_DMA_LSO_BD_ACCESS_TXQ, txq),
	       dma + CA_DMA_LSO_VP_BD_ACCESS(vp));

	ret = readl_poll_timeout(dma + CA_DMA_LSO_VP_BD_ACCESS(vp), val,
				 !(val & CA_DMA_LSO_BD_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_err(ni->dev, "VP%u txq%u ring program timed out\n",
			vp, txq);
	return ret;
}

static int cortina_ni_tx_rings_init(struct cortina_ni *ni)
{
	struct cortina_ni_tx *tx = ni->tx;
	int i, ret;

	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];
		u32 wptr, rptr;

		q->vp = CA_NI_TX_VP_BASE + i;
		/* fix#68: txq[0] is the PON/OMCI ring - stock runs it on txq 7
		 * (highest priority); every other ring keeps txq 0. */
		q->txq = (i == 0) ? (u8)clamp(pon_txq, 0, 7) : CA_NI_TX_TXQ;
		spin_lock_init(&q->lock);

		q->desc = dmam_alloc_coherent(ni->dev,
					      CA_NI_TX_RING_SIZE *
					      CA_NI_TX_DESC_WORDS * 4,
					      &q->desc_dma, GFP_KERNEL);
		if (!q->desc)
			return -ENOMEM;

		ret = cortina_ni_tx_ring_program(ni, q->vp, q->txq,
						 q->desc_dma);
		if (ret)
			return ret;

		/* adopt whatever pointer state the HW is in (0 after reset) */
		wptr = readl(dma_base(ni) +
			     CA_DMA_LSO_VP_TXQ_WPTR(q->vp, q->txq)) &
			CA_DMA_LSO_PTR_MASK;
		rptr = readl(dma_base(ni) +
			     CA_DMA_LSO_VP_TXQ_RPTR(q->vp, q->txq)) &
			CA_DMA_LSO_PTR_MASK;
		if (wptr >= CA_NI_TX_RING_SIZE || rptr >= CA_NI_TX_RING_SIZE ||
		    wptr != rptr)
			dev_warn(ni->dev,
				 "VP%u txq0 pointers not idle (w=%u r=%u)\n",
				 q->vp, wptr, rptr);
		q->wptr = wptr % CA_NI_TX_RING_SIZE;
		q->finished = rptr % CA_NI_TX_RING_SIZE;

		dev_info(ni->dev, "VP%u txq0 ring @%pad (%u desc)\n",
			 q->vp, &q->desc_dma, CA_NI_TX_RING_SIZE);
	}
	return 0;
}

/* QM buffer manager (stock aal_l2_qm_init values) - without buffers the
 * egress enqueue fails silently */
static void cortina_ni_tx_qm_init(struct cortina_ni *ni)
{
	/* EQ0 disabled/empty; EQ1 enabled, 4K x 64B pool + port private */
	ni_rmw(ni, CA_NI_L2TM_QM_EQ_CFG,
	       CA_NI_L2TM_EQ0_EN | CA_NI_L2TM_EQ0_BUFNUM |
	       CA_NI_L2TM_EQ0_PRVT | CA_NI_L2TM_EQ1_BUFNUM |
	       CA_NI_L2TM_EQ1_PRVT,
	       CA_NI_L2TM_EQ1_EN |
	       FIELD_PREP(CA_NI_L2TM_EQ1_BUFNUM, CA_NI_QM_EQ1_BUFNUM_VAL) |
	       FIELD_PREP(CA_NI_L2TM_EQ1_PRVT, CA_NI_QM_PORT_PRVT_BUFF_NUM));

	/* port-private buffer profile 0 (all ports select it by default) */
	ni_rmw(ni, CA_NI_L2TM_QM_PORT_PRVT_PROF0, 0x7fff,
	       CA_NI_QM_PORT_PRVT_BUFF_NUM);

	/* global buffer thresholds: drop on, no FE back-pressure */
	ni_rmw(ni, CA_NI_L2TM_QM_GLOB_BUF_CFG,
	       CA_NI_L2TM_BUF_NODROP | CA_NI_L2TM_BUF_NONCONG |
	       CA_NI_L2TM_BUF_FE_BP_EN,
	       CA_NI_L2TM_BUF_DROP_EN |
	       FIELD_PREP(CA_NI_L2TM_BUF_NODROP, CA_NI_QM_NODROP_THRESHOLD) |
	       FIELD_PREP(CA_NI_L2TM_BUF_NONCONG,
			  CA_NI_QM_NONCONG_THRESHOLD));
}

/* TM egress scheduler (stock aal_l2_tm_init): global + per-port + per-VOQ
 * enables - the "one-line block enable" whose omission silently stalls TX */
static void cortina_ni_tx_tm_init(struct cortina_ni *ni)
{
	int i;

	ni_rmw(ni, CA_NI_L2TM_ES_CTRL, 0,
	       CA_NI_L2TM_ES_TX_EN | CA_NI_L2TM_ES_PORT_EN_ALL);

	for (i = 0; i < CA_NI_L2TM_ES_SCH_INSTANCES; i++)
		ni_rmw(ni, CA_NI_L2TM_ES_SCH_CFG(i), 0,
		       CA_NI_L2TM_ES_VOQ_EN_ALL);
}

/* LAN port MACs: TX on for EVERY RJ45 (RX is armed by the link path), MAC
 * auto-tracks the PHY.
 *
 * ★ Looped over all CA_NI_LAN_PORT_COUNT ports, not just CA_NI_TX_PORT: with a
 * per-frame egress port the descriptor DEST can now name any LAN port, and the
 * TX side of ports 0..2 was not enabled by anything.
 * cortina_ni_rx_enable_internal_ports() loops p = 1..6, so port 0 had NO TXMAC
 * tx_en at all, and GLB.PWR_DWN_TX was cleared only for CA_NI_TX_PORT - so a
 * frame stamped for port 0/1/2 would have been handed to a powered-down egress
 * MAC.
 */
static void cortina_ni_tx_port_mac_init(struct cortina_ni *ni)
{
	unsigned int p;

	/* connect the port MAC to the internal quad-GPHY over GMII (0xa5c0):
	 * int_cfg=GE_GMII, phy_mode=MAC, MAC-loopback OFF.  NOTE: the upper byte
	 * 0xCB000000 seen on stock is READ-ONLY datapath-active STATUS (a forced
	 * write of it does not stick), not writable config - so it only lights up
	 * once the real GPHY<->MAC datapath gate is satisfied. */
	for (p = 0; p < CA_NI_LAN_PORT_COUNT; p++) {
		ni_rmw(ni, CA_NI_PORT_STATIC_CFG(p),
		       CA_NI_PORT_STATIC_INT_CFG | CA_NI_PORT_STATIC_PHY_MODE |
		       CA_NI_PORT_STATIC_LPBK_MODE, 0);

		ni_rmw(ni, CA_NI_PORT_GLB_CFG(p),
		       CA_NI_PORT_GLB_PWR_DWN_TX, 0);

		ni_rmw(ni, CA_NI_PORT_TXMAC_CFG(p),
		       CA_NI_PORT_TXMAC_TX_DRAIN,
		       CA_NI_PORT_TXMAC_TX_EN | CA_NI_PORT_TXMAC_CRC_CALC_EN);
	}

	/* MAC autosync OFF (=0), matching U-Boot's PROVEN-working datapath
	 * (autosync=0x0 while tftp ran bidirectionally over this port).
	 *
	 * ★ Determinism root cause: we drive phylib (adjust_link writes the GLB
	 * speed/duplex in SW on every link event) AND phylib RESTARTS aneg at
	 * phy_start, bouncing the line link.  If HW autosync (0xf) is ALSO on,
	 * the HW continuously re-derives glb/speed/duplex from the churning PHY
	 * status during that bounce and fights our SW writes - dropping the
	 * internal GMII on the boots where the two collide (== "works some
	 * boots").  Stock tolerates autosync=0xf only because its link is stable
	 * (it never restarts aneg - it inherits U-Boot's link and just monitors
	 * it).  We use phylib, so we adopt U-Boot's consistent model: autosync
	 * OFF, phylib owns speed/duplex via adjust_link.  ONE owner, no fight. */
	/* DIAGNOSTIC: stock uses autosync=0xf (HW MAC-follows-PHY, STS_ALL). Now
	 * that the GPHY firmware matches stock, test stock's autosync model. */
	ni_rmw(ni, CA_NI_HV_MAC_AUTOSYNC,
	       CA_NI_HV_AUTOSYNC_FC_ALL, CA_NI_HV_AUTOSYNC_STS_ALL);
}

/*
 * L2FE ARB ldpid->pdpid map: route the "9th queue" ldpids (0x08..0x0f, the
 * CPU-injected US PON control-frame ports) to the PON-OAM egress.  Without it
 * a CPU-injected OMCI frame (HEADER_A ldpid = PON+8 = 0x0f) has no physical
 * route, never reaches the PUC, and the OLT receives no upstream OMCI.  Only
 * touches ldpid 0x08..0x0f (unused by the Ethernet CPU-RX/LAN paths, which use
 * ldpid 0x19/0x32), so it cannot disturb the working datapath.  32 entries:
 * ldpid 0x08..0x0f x dbuf {0,1} x my_mac {0,1} -> PPORT_OAM.
 */
static int cortina_ni_arb_map_one(struct cortina_ni *ni, u32 idx, u32 pdpid)
{
	void __iomem *ni_r = ni_base(ni);
	u32 val;
	int ret;

	writel(pdpid, ni_r + CA_NI_L2FE_ARB_PDPID_DATA);
	writel(CA_DMA_LSO_BD_ACCESS_GO | CA_DMA_LSO_BD_ACCESS_WRITE | idx,
	       ni_r + CA_NI_L2FE_ARB_PDPID_ACCESS);
	ret = readl_poll_timeout(ni_r + CA_NI_L2FE_ARB_PDPID_ACCESS, val,
				 !(val & CA_DMA_LSO_BD_ACCESS_GO),
				 CA_NI_TX_POLL_US, CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(ni->dev, "ARB map[0x%02x] timed out\n", idx);
	return ret;
}

/* ★ Physical LAN NI ports 0-6: identity ldpid->pdpid so an eth0 direct-TX frame
 * (whose descriptor DEST field is the ldpid) egresses physical port N (vendor
 * aal_port.c global port init).  Left unmapped, the ARB PDPID map reads its reset
 * value 0, so EVERY eth0 CPU-TX frame resolved to physical port 0 (uncabled/dead)
 * regardless of the descriptor DEST - which is why LAN INGRESS worked but the
 * router's ARP/ping/DHCP replies never reached the wired host, and why setting
 * the descriptor DEST or the VP HDRA LDPID alone changed nothing.  dbuf=1 rows ->
 * QM (US-PON data path); ldpid 7 (PON) -> blackhole, per the vendor map. */
static void cortina_ni_arb_lan_map_init(struct cortina_ni *ni)
{
	u32 my_mac, ldpid;

	for (my_mac = 0; my_mac <= 1; my_mac++) {
		for (ldpid = 0; ldpid <= 6; ldpid++) {
			if (cortina_ni_arb_map_one(ni, (my_mac << 7) | ldpid,
						   ldpid))
				return;
			if (cortina_ni_arb_map_one(ni,
						   (my_mac << 7) | BIT(6) | ldpid,
						   CA_NI_PPORT_QM))
				return;
		}
		if (cortina_ni_arb_map_one(ni, (my_mac << 7) | 7,
					   CA_NI_PPORT_BLACKHOLE))
			return;
		if (cortina_ni_arb_map_one(ni, (my_mac << 7) | BIT(6) | 7,
					   CA_NI_PPORT_BLACKHOLE))
			return;
	}
	dev_info(ni->dev,
		 "L2FE ARB: LAN ldpids 0x00-0x06 -> identity pdpid (eth0 egress)\n");
}

static void cortina_ni_arb_oam_map_init(struct cortina_ni *ni)
{
	u32 my_mac, dbuf, ldpid, idx;

	for (my_mac = 0; my_mac <= 1; my_mac++) {
		for (dbuf = 0; dbuf <= 1; dbuf++) {
			/* 9th-queue (control-frame inject) -> OAM engine */
			for (ldpid = CA_NI_LDPID_9QUEUE_LO;
			     ldpid <= CA_NI_LDPID_9QUEUE_HI; ldpid++) {
				idx = (my_mac << 7) | (dbuf << 6) | ldpid;
				if (cortina_ni_arb_map_one(ni, idx,
							   CA_NI_PPORT_OAM))
					return;
			}
			/* CPU_MQ / LLID-GEM-index (US PON DATA inject) -> QM
			 * (vendor aal_port.c global init maps all of
			 * 0x20..0x3f, both dbuf + my_mac, to PPORT_QM) */
			for (ldpid = CA_NI_LDPID_CPU_MQ_LO;
			     ldpid <= CA_NI_LDPID_CPU_MQ_HI; ldpid++) {
				idx = (my_mac << 7) | (dbuf << 6) | ldpid;
				if (cortina_ni_arb_map_one(ni, idx,
							   CA_NI_PPORT_QM))
					return;
			}
		}
	}
	dev_info(ni->dev,
		 "L2FE ARB: ldpids 0x08-0x0f -> PON-OAM, 0x20-0x3f -> QM\n");
}


/*
 * ★ 2026-08-08 AOT5221ZY (fix #27): arm the TX DMA/LSO engine.
 * MUST run AFTER cortina_ni_tx_rings_init() has programmed each VP/TXQ descriptor
 * ring base+depth.  Upstream enables the engine inside cortina_ni_tx_engine_init(),
 * i.e. BEFORE the ring bases exist -- the engine goes live pointing at an
 * unprogrammed base, fetches a descriptor from a garbage address and raises
 * `Asynchronous SError 0xbe000011` ~5 ms later.  Proven with tx_bar() barriers: all
 * 10 inside engine_init print (no write faults), the abort arrives after it returns.
 */
static void cortina_ni_tx_engine_enable(struct cortina_ni *ni)
{
	void __iomem *dma = dma_base(ni);

	writel(readl(dma + CA_DMA_SS_CTRL) | CA_DMA_SS_CTRL_TX_EN,
	       dma + CA_DMA_SS_CTRL);
	tx_bar(ni, "ss_ctrl_tx_en(post-rings)");
	writel(CA_DMA_LSO_CTRL_VAL, dma + CA_DMA_LSO_CTRL);
	readl(dma + CA_DMA_LSO_CTRL);		/* stock reads back */
	tx_bar(ni, "lso_ctrl(post-rings)");
}

static int cortina_ni_tx_hw_init(struct cortina_ni *ni)
{
	int ret;

	/* ★ DIAGNOSTIC: deassert the internal digital-PHY resets EARLY (before any
	 * GPHY/MAC init) - stock's dphy_rst (GLB+0xa0) = 0x10000000, ours boots
	 * 0x50302340 (sub-blocks held in reset).  Release-late (link_up) didn't
	 * revive it, so try release-then-init order: release here, before init. */
	/* ★ 2026-08-07 AOT5221ZY bring-up: SKIPPED on this board.
	 * This block is labelled DIAGNOSTIC upstream and its premise does not hold here:
	 * it expects glb+0xa0 to read 0x50302340 ("sub-blocks held in reset"), but this
	 * board reads 0x00000000 (already released). Forcing 0x10000000 makes the posted
	 * write fault, surfacing as "Kernel panic - Asynchronous SError Interrupt" right
	 * after this very log line (reproduced across boots). The shipping 6.6 driver for
	 * this board never touches glb+0xa0 and its NI/GPHY come up fine. */
	/* ★ 2026-08-07 AOT5221ZY: the whole glb+0xa0 access is removed, READ INCLUDED.
	 * Skipping only the write still panicked with "Asynchronous SError Interrupt"
	 * (code 0xbe000011, on CPU1) immediately after the READ's log line - on this SoC
	 * an undecoded read returns 0 and raises the abort asynchronously, so the read is
	 * itself the faulting access. Upstream marks this block DIAGNOSTIC; the shipping
	 * 6.6 driver for this board never touches glb+0xa0. */

	/*
	 * ★ 2026-08-08 AOT5221ZY SError bisect (cortina_ni.tx_stage=N):
	 *   0 = nothing here at all   1 = reset_intf only
	 *   2 = + the NI global RMWs  3/default = + the TX-DMA engine init
	 * skip_mdio=1 proved the `Asynchronous SError 0xbe000011` is NOT the MDIO/PHY
	 * path: removing it moved the fault 64 ms EARLIER (1.2656 s vs 1.3300 s) rather
	 * than removing it, so the trigger is inside this probe.  This gate says which
	 * part.  (2026-08-07's eight fixes never moved the fault time >12 ms because
	 * they were all downstream of the real trigger.)
	 */
	if (tx_stage < 1) {
		dev_info(ni->dev, "tx_stage=%d: tx_hw_init SKIPPED entirely\n", tx_stage);
		return 0;
	}

	/* stock aal_ni_init order: reset -> NI globals -> TX-DMA engine */
	cortina_ni_tx_reset_intf(ni);
	if (tx_stage < 2) {
		dev_info(ni->dev, "tx_stage=%d: stopping after reset_intf\n", tx_stage);
		return 0;
	}

	/* unconditional stock globals (unknown names, exact stock values) */
	ni_rmw(ni, CA_NI_HV_CFG_A420, CA_NI_HV_CFG_A420_FIELD,
	       FIELD_PREP(CA_NI_HV_CFG_A420_FIELD, CA_NI_HV_CFG_A420_VAL));
	ni_rmw(ni, CA_NI_HV_CFG_A1B8, CA_NI_HV_CFG_A1B8_FIELD,
	       FIELD_PREP(CA_NI_HV_CFG_A1B8_FIELD, CA_NI_HV_CFG_A1B8_VAL));
	/* ★ 2026-08-07 AOT5221ZY bring-up: SKIPPED on this board.
	 * CA_NI_HV_CFG_AAF0 (0xaaf0 = NI_HV_MCE_CTL_REG in the vendor map) does NOT
	 * decode on this SKU (GLB chip id 0x6706d8f3): the read half of this RMW takes a
	 * synchronous external abort and panics the kernel at cortina_ni_tx_probe+0x23c,
	 * reproduced on two independent boots. The two RMWs immediately above (0xa420,
	 * 0xa1b8) succeed on the same window, so it is this register specifically, not
	 * the mapping or the reset. The shipping 6.6 driver for this board never touches
	 * 0xaaf0 at all and its NI works. Re-enable behind a chip-id check if the X400AXF
	 * needs it. */
	if (0)
		ni_rmw(ni, CA_NI_HV_CFG_AAF0, CA_NI_HV_CFG_AAF0_FIELD,
		       FIELD_PREP(CA_NI_HV_CFG_AAF0_FIELD, CA_NI_HV_CFG_AAF0_VAL));

	/* frame-length limits, stock values */
	ni_rmw(ni, CA_NI_HV_PKT_LEN,
	       CA_NI_HV_PKT_LEN_MIN | CA_NI_HV_PKT_LEN_MAX,
	       FIELD_PREP(CA_NI_HV_PKT_LEN_MIN, CA_NI_HV_PKT_LEN_MIN_VAL) |
	       FIELD_PREP(CA_NI_HV_PKT_LEN_MAX, CA_NI_HV_PKT_LEN_MAX_VAL));
	ni_rmw(ni, CA_NI_HV_PKT_LEN_RX, CA_NI_HV_PKT_LEN_RX_MAX,
	       FIELD_PREP(CA_NI_HV_PKT_LEN_RX_MAX, CA_NI_HV_PKT_LEN_MAX_VAL));

	/* 0xa1bc = INTERNAL_PORT_ID_CFG (the old chipdef mislabeled it
	 * "NIRX_MISC"): keep the aal_ni_init golden mirror bits [13:9] and clear
	 * the stray bit15 (U-Boot left 0xbe80; stock golden 0x3e80).  CRITICAL:
	 * do NOT clear bit20 - that is l3qmrx_to_lan, the NI->QM LAN handoff SET
	 * by the L3QM delivery init; clearing it here (as the old code did) left
	 * NI-RX invisible to the QM. */
	ni_rmw(ni, CA_NI_NI_INTERNAL_PORT_ID_CFG,
	       CA_NI_NI_INTERNAL_BIT15,
	       CA_NI_NI_MRR_CFG);

	/* deferred stock init (not needed for port-0 direct TX): the 0xa01c
	 * port-to-cpu debug bits, the RX demux cfg (0xa180/88/8c), SCH-cfg
	 * field [23:16]=6 on instances 8/10/13, aal_l2_te/l3_tm/l3_te init,
	 * and the streamid/dmaaft/l2fib table clears (reset defaults 0) */

	if (tx_stage < 3) {
		dev_info(ni->dev, "tx_stage=%d: stopping before tx_engine_init\n", tx_stage);
		return 0;
	}

	ret = cortina_ni_tx_engine_init(ni);
	if (ret)
		return ret;

	tx_bar(ni, "engine_init-returned");
	ret = cortina_ni_tx_rings_init(ni);
	tx_bar(ni, "rings_init-returned");
	if (ret)
		return ret;

	/* ★ fix #27: only NOW is it safe to arm the engine (rings are programmed). */
	cortina_ni_tx_engine_enable(ni);

	cortina_ni_tx_qm_init(ni);
	cortina_ni_tx_tm_init(ni);
	cortina_ni_tx_port_mac_init(ni);
	cortina_ni_arb_oam_map_init(ni);	/* US PON control-frame egress route */
	cortina_ni_arb_lan_map_init(ni);	/* LAN NI ldpid->pport egress route */
	return 0;
}

/* ------------------------------------------------------------------ */
/* TX completion                                                       */
/* ------------------------------------------------------------------ */

/* caller holds q->lock */
static unsigned int cortina_ni_tx_reclaim_q(struct cortina_ni *ni,
					    struct cortina_ni_txq *q)
{
	struct net_device *ndev = ni->tx->netdev;
	unsigned int freed = 0;
	u32 rptr;

	rptr = readl(dma_base(ni) +
		     CA_DMA_LSO_VP_TXQ_RPTR(q->vp, q->txq)) &
		CA_DMA_LSO_PTR_MASK;
	rptr %= CA_NI_TX_RING_SIZE;

	while (q->finished != rptr) {
		struct sk_buff *skb = q->slot[q->finished].skb;

		if (!skb) {
			u8 pon = q->slot[q->finished].pon;

			if (q->slot[q->finished].dup) {
				/* extra copy of a flooded eth0 frame: it shares
				 * the mapping owned by the LAST descriptor of
				 * the burst, so there is nothing to unmap or
				 * free here.  The engine consumes the ring in
				 * order, so the owner is always reclaimed after
				 * every copy of its own burst. */
				q->slot[q->finished].dup = 0;
				q->finished = (q->finished + 1) %
					      CA_NI_TX_RING_SIZE;
				q->reclaimed++;
				freed++;
				continue;
			}
			if (!pon) {	/* must not happen: HW advanced past us */
				netdev_err(ndev, "VP%u: hole at %u (rptr %u)\n",
					   q->vp, q->finished, rptr);
				break;
			}
			/* PON control-frame descriptor: coherent scratch,
			 * nothing to unmap/free; the frame (EOF) descriptor
			 * releases its scratch slot (under this q->lock) */
			if (pon >= 2)
				ni->tx->pon_busy &= ~BIT(pon - 2);
			q->slot[q->finished].pon = 0;
			q->finished = (q->finished + 1) % CA_NI_TX_RING_SIZE;
			q->reclaimed++;
			freed++;
			continue;
		}
		dma_unmap_single(ni->dev, q->slot[q->finished].addr,
				 q->slot[q->finished].len, DMA_TO_DEVICE);
		if (!q->slot[q->finished].pon) {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += q->slot[q->finished].len;
		} else {
			/* PON data skb (pon=1 + skb): counted on the WAN
			 * netdev at enqueue, not on eth0 */
			q->slot[q->finished].pon = 0;
		}
		dev_consume_skb_any(skb);
		q->slot[q->finished].skb = NULL;
		q->finished = (q->finished + 1) % CA_NI_TX_RING_SIZE;
		q->reclaimed++;
		freed++;
	}
	return freed;
}

static unsigned int cortina_ni_txq_free_desc(struct cortina_ni_txq *q)
{
	if (q->wptr >= q->finished)
		return CA_NI_TX_RING_SIZE - q->wptr - 1 + q->finished;
	return q->finished - q->wptr - 1;
}

/*
 * ★★★★★ fix#59 (2026-08-09) — PERIODIC DMA-LSO CPU-TX ENGINE SAMPLER.
 *
 * WHY: /proc/net/cortina_ni_tx measured, live, that the fetch engine NEVER retires a VP2
 * descriptor (`vp2 hw w=60 r=0 done=0`) and that VP3/eth0 completes 11 frames and then stalls
 * too (`vp3 hw w=18 r=11 done=11`, and a post-O5 ping enqueues 4 more that are never fetched).
 * That RETRACTS the standing premise "the hardware does consume the TX descriptors, so the
 * frame is lost between the ring and the PUC" — the frame never leaves the ring, and every
 * PUC/ARB/L2FE/STREAMID hypothesis was therefore aimed downstream of a frame never emitted.
 * eth0 DOES transmit earlier in the boot, so this is a TEARDOWN, not a never-worked, and it
 * looks to happen around O5/OMCC-up (19.1 s).
 *
 * WHAT THIS ANSWERS: sampling across the whole boot pins the exact moment the engine stops and
 * shows which state changed at that moment. In particular it reads the ring BASE+DEPTH back
 * through the indirect handshake — if the base reads back zeroed or changed after O5, that
 * confirms outright the "O5 re-range tears down the per-VP binding and ring base" candidate
 * (Track A hit exactly this and fixed it with cortina_ni_pon_vp_rearm(), which Track B has no
 * equivalent of: Track B programs the rings ONCE, at probe).
 *
 * ⚠ Indirect tables MUST be read through ACCESS -> poll GO -> DATA. A raw readl of the DATA
 *   word returns staging residue, not table content.
 * ⚠ Offsets 0x0d0/0x0d4/0x0d8/0x0dc/0x0e8/0x0ec/0x0f0/0x0f4 DO NOT DECODE on this die (a bare
 *   write to 0xf70010d8 raises Asynchronous SError 0xbe000011) — none are touched here.
 * ⚠ Compare every value against the TAURUS defaults, not the generic ones: HDRA_CFG _dft
 *   0x200, SS_CTRL 0x30000fff, DEBUG0 0x01000000 (not 0), LSO_CTRL 0x20. A read equal to _dft
 *   means UNWRITTEN, not "captured".
 *
 * cortina_ni.dma_dump_s=<seconds> (0 = off).
 */
static unsigned int dma_dump_s;
module_param(dma_dump_s, uint, 0644);
MODULE_PARM_DESC(dma_dump_s, "period in seconds for the DMA-LSO CPU-TX engine sampler (0=off)");


/* read one VP/TXQ ring base+depth back through the indirect window */
static int cortina_ni_tx_ring_readback(struct cortina_ni *ni, u8 vp, u8 txq,
				       u32 *d0, u32 *d1)
{
	void __iomem *dma = dma_base(ni);
	u32 val;
	int ret;

	/* rbw = 0 -> READ (CA_DMA_LSO_BD_ACCESS_WRITE deliberately not set) */
	writel(CA_DMA_LSO_BD_ACCESS_GO |
	       FIELD_PREP(CA_DMA_LSO_BD_ACCESS_TXQ, txq),
	       dma + CA_DMA_LSO_VP_BD_ACCESS(vp));
	ret = readl_poll_timeout_atomic(dma + CA_DMA_LSO_VP_BD_ACCESS(vp), val,
					!(val & CA_DMA_LSO_BD_ACCESS_GO),
					CA_NI_TX_POLL_US,
					CA_NI_TX_POLL_TIMEOUT_US);
	if (ret)
		return ret;
	*d0 = readl(dma + CA_DMA_LSO_VP_BD_DATA0(vp));
	*d1 = readl(dma + CA_DMA_LSO_VP_BD_DATA1(vp));
	return 0;
}

static void cortina_ni_tx_dma_dump(struct work_struct *work)
{
	struct cortina_ni_tx *tx =
		container_of(to_delayed_work(work), struct cortina_ni_tx, dma_dump_work);
	/* ⚠ the netdev private area holds a POINTER to struct cortina_ni, not the
	 * struct itself (devm_alloc_etherdev(dev, sizeof(struct cortina_ni *))), so it
	 * must be double-dereferenced exactly as every other site in this file does.
	 * Getting this wrong panics at ~2.0 s with a garbage ni. */
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(tx->netdev);
	void __iomem *dma = dma_base(ni);
	int i;

	/* GLB engine state.  DEBUG0 decodes dmatx_cs[2:0] desc_cs[6:4] txfer_cs[10:8]
	 * dat_cs[13:12] txfout_cs[19:16] base_cs[22:20] pktcnt_cs[26:24]; txfout_cs is
	 * the LSO->fabric handoff, the last stage before the frame leaves this block. */
	dev_info(ni->dev,
		 "DMALSO glb: lso_ctrl=%08x lso_int=%08x dbg0=%08x dbg1=%08x dbg2=%08x | "
		 "fdes=%08x_%08x cdes=%08x_%08x desw=%08x_%08x para=%08x_%08x\n",
		 readl(dma + CA_DMA_LSO_CTRL), readl(dma + 0x004),
		 readl(dma + 0x050), readl(dma + 0x054), readl(dma + 0x058),
		 readl(dma + 0x034), readl(dma + 0x030),
		 readl(dma + 0x03c), readl(dma + 0x038),
		 readl(dma + 0x044), readl(dma + 0x040),
		 readl(dma + 0x04c), readl(dma + 0x048));
	dev_info(ni->dev,
		 "DMALSO ss: ss_ctrl=%08x ss_sts=%08x drr=%08x port3_0=%08x | "
		 "aximn2_int=%08x aft_ctrl=%08x aft_sts=%08x aft_sts1=%08x\n",
		 readl(dma + CA_DMA_SS_CTRL), readl(dma + 0x904),
		 readl(dma + 0x908), readl(dma + 0x90c),
		 readl(dma + 0xd88), readl(dma + 0xf00),
		 readl(dma + 0xf30), readl(dma + 0xf34));

	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];
		u32 d0 = 0xdeadbeef, d1 = 0xdeadbeef;
		u8 vp = q->vp;
		int ret;

		/* the handshake writes ACCESS, so serialise against pon_tx/xmit on
		 * this queue rather than racing the doorbell path */
		spin_lock_bh(&q->lock);
		ret = cortina_ni_tx_ring_readback(ni, vp, CA_NI_TX_TXQ, &d0, &d1);
		spin_unlock_bh(&q->lock);

		dev_info(ni->dev,
			 "DMALSO vp%u: ctrl=%08x hdra=%08x wptr=%08x rptr=%08x | "
			 "base_rb=%08x_%08x%s (programmed %pad) | vpint=%08x miscint=%08x | "
			 "ssw30=%08x ssw74=%08x\n",
			 vp,
			 readl(dma + CA_DMA_LSO_VP_CONTROL(vp)),
			 readl(dma + CA_DMA_LSO_VP_HDRA_CFG(vp)),
			 readl(dma + CA_DMA_LSO_VP_TXQ_WPTR(vp, CA_NI_TX_TXQ)),
			 readl(dma + CA_DMA_LSO_VP_TXQ_RPTR(vp, CA_NI_TX_TXQ)),
			 d1, d0, ret ? " [READBACK TIMED OUT]" : "",
			 &q->desc_dma,
			 readl(dma + 0x160 + vp * CA_DMA_LSO_VP_STRIDE),
			 readl(dma + 0x168 + vp * CA_DMA_LSO_VP_STRIDE),
			 readl(dma + 0x918 + vp * 8),
			 readl(dma + 0x91c + vp * 8));
	}

	if (dma_dump_s)
		schedule_delayed_work(&tx->dma_dump_work, dma_dump_s * HZ);
}

static void cortina_ni_tx_reclaim_timer(struct timer_list *t)
{
	struct cortina_ni_tx *tx = timer_container_of(tx, t, reclaim_timer);
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(tx->netdev);
	bool pending = false;
	unsigned int freed = 0;
	int i;

	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];

		spin_lock_bh(&q->lock);
		freed += cortina_ni_tx_reclaim_q(ni, q);
		if (q->finished != q->wptr)
			pending = true;
		spin_unlock_bh(&q->lock);
	}

	if (freed && netif_queue_stopped(tx->netdev))
		netif_wake_queue(tx->netdev);
	if (pending)
		mod_timer(&tx->reclaim_timer,
			  jiffies + CA_NI_RECLAIM_INTERVAL);
}

/* ------------------------------------------------------------------ */
/* xmit                                                                */
/* ------------------------------------------------------------------ */

/*
 * fix#65: ONE LINE PER ENQUEUE, on every ring, carrying the engine state as it was
 * immediately BEFORE this frame was offered (the doorbell has not been rung yet).
 * `rptr` on that line therefore says whether everything enqueued so far has retired,
 * so the descriptor that wedges - and which path enqueued it - is read off directly
 * instead of being inferred from a 2-second sampler.  Bounded to `txlog` lines total,
 * across both rings, in enqueue order.
 */
/*
 * fix#68: ring the doorbell exactly as stock does - W1C the per-VP MISC_INTERRUPT
 * txq_empty bit for THIS queue between the descriptor stores and the write-pointer
 * publish (aal_ni_clear_dma_lso_misc_interrupts @0x6530 <- __ca_ni_send_single_pkt
 * @0xa8d5c, doorbell @0xa8d98).  The barrier stays where it was: descriptors visible
 * before either write.
 */
static void cortina_ni_tx_doorbell(struct cortina_ni *ni,
				   struct cortina_ni_txq *q)
{
	void __iomem *dma = dma_base(ni);

	dma_wmb();
	if (misc_w1c)
		writel(ca_dma_lso_txq_empty_bit(q->txq),
		       dma + CA_DMA_LSO_VP_MISC_INT(q->vp));
	writel(q->wptr, dma + CA_DMA_LSO_VP_TXQ_WPTR(q->vp, q->txq));
}

static atomic_t txlog_seq = ATOMIC_INIT(0);

static void cortina_ni_txlog(struct cortina_ni *ni, struct cortina_ni_txq *q,
			     const char *src, unsigned int idx, u32 desc1,
			     dma_addr_t daddr, unsigned int len)
{
	void __iomem *dma = dma_base(ni);

	if (!txlog || atomic_inc_return(&txlog_seq) > txlog)
		return;

	dev_info(ni->dev,
		 "TXLOG %s vp%u.q%u idx=%u desc1=%08x dma=%pad len=%u | pre: hw_wptr=%u hw_rptr=%u dbg0=%08x miscint=%08x\n",
		 src, q->vp, q->txq, idx, desc1, &daddr, len,
		 readl(dma + CA_DMA_LSO_VP_TXQ_WPTR(q->vp, q->txq)),
		 readl(dma + CA_DMA_LSO_VP_TXQ_RPTR(q->vp, q->txq)),
		 readl(dma + 0x050),
		 readl(dma + CA_DMA_LSO_VP_MISC_INT(q->vp)));
}

static netdev_tx_t cortina_ni_start_xmit(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct cortina_ni_tx *tx = ni->tx;
	struct cortina_ni_txq *q;
	dma_addr_t daddr;
	__le32 *desc;
	u32 word1, w = 0, ports;
	unsigned int len, first, nports, port;
	bool is_dsa = netdev_uses_dsa(ndev);
	u8 dsa_dest = 0;

	/* fix#65: mute eth0 TX so the PON path is the engine's only user and no stall
	 * can be blamed on - or hidden behind - LAN traffic. */
	if (eth_off) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* ★DSA: net/dsa/tag_cortina.c front-pushed a CA_NI_DSA_TAG_LEN shim
	 * (byte[0] = MAGIC | egress jack).  Consume it HERE — before the padto
	 * below, so padding lands on the clean frame — and fold it into the single
	 * DESC1_DEST for this port (replacing the FDB/flood selection).  The shim
	 * never reaches the wire.  A frame WITHOUT the magic is one the conduit
	 * itself originated untagged (e.g. a kernel IPv6 DAD once DSA brings the
	 * conduit UP); it has no valid jack, so drop it rather than mis-strip and
	 * misroute it out a pseudo-random port. */
	if (is_dsa) {
		if (unlikely(skb->len < CA_NI_DSA_TAG_LEN ||
			     (skb->data[CA_NI_DSA_TAG_PORT_OFF] &
			      CA_NI_DSA_TAG_MAGIC_MASK) != CA_NI_DSA_TAG_MAGIC)) {
			ndev->stats.tx_dropped++;
			dev_kfree_skb_any(skb);
			return NETDEV_TX_OK;
		}
		dsa_dest = skb->data[CA_NI_DSA_TAG_PORT_OFF] &
			   CA_NI_DSA_TAG_PORT_MASK;
		skb_pull(skb, CA_NI_DSA_TAG_LEN);
	}

	/* short frames: pad to the wire minimum (also covers the engine's
	 * 34-byte DMA floor); skb freed by the helper on failure */
	if (skb_padto(skb, ETH_ZLEN))
		return NETDEV_TX_OK;
	len = max_t(unsigned int, skb->len, ETH_ZLEN);

	if (unlikely(len > CA_NI_TX_MAX_FRAME)) {
		tx->drop_oversize++;
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* single-descriptor path (stock dirTx is single-descriptor too) */
	if (unlikely(skb_linearize(skb))) {
		tx->drop_linearize++;
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/*
	 * ★ PACKET ORDER (the downstream OOO/TCP-collapse root cause): eth0 is
	 * a SINGLE-queue netdev, so it must feed exactly ONE HW ring.  The old
	 * per-CPU pick here — q = &txq[raw_smp_processor_id() % NUM_VPS], the
	 * vendor scheme (stock __ca_ni_start_xmit @0x1ae230 selects its DMA-LSO
	 * VP via ca_ni_dmalso_vp_sel[cpu % 7] = {8,1,2,3,9,10,11}) — split one
	 * flow across up to 4 VP rings whenever the transmitting CPU changed
	 * (IRQ/NAPI migration across the 8 RX SPIs, qdisc-runner handoff, RPS).
	 * The DMA-LSO engine fetches the VP rings independently, so same-flow
	 * frames overtake each other on the port-0 wire: OOO scaling with rate
	 * (7@100M → 25%@600M), which TCP reads as loss → spurious-retransmit
	 * storms.  Stock gets away with the per-CPU scheme because its bulk
	 * traffic is HW-forwarded (CPU TX is slow-path only) and its netdevs
	 * are 8-queue mq (flow→queue pinned by the stack); we CPU-forward
	 * everything, so wire order must equal qdisc order: one netdev queue →
	 * one ring.  The qdisc already serializes xmit for a single-queue
	 * netdev (one CPU in qdisc_run at a time, in-order dequeue), so a
	 * fixed ring restores strict per-flow order with no new locking;
	 * q->lock still guards against the reclaim timer.  eth0 rides its OWN
	 * ring (VP3), leaving txq[0] (VP2) to the PON OMCI/WAN-data path, so
	 * the two in-order streams neither share a lock nor stall each other.
	 */
	/*
	 * ★★★★★ fix#62 (2026-08-09) — THE VP-vs-FRAME DISCRIMINATOR.
	 *
	 * fix#61 exonerated the descriptor FORMAT: the OMCI frame wedges on VP2 even when
	 * sent as a plain direct-TX (HP=11) descriptor - the identical format eth0 pushes
	 * successfully on VP3 through the same engine.  So the difference is the VP (or
	 * something bound to it), not the encoding.
	 *
	 * This param moves eth0's OWN traffic onto the PON ring so the two are compared with
	 * everything else held constant.  eth0's boot traffic goes out at ~13-19 s, well
	 * BEFORE the first OMCI TX (~21-30 s), so the early samples are uncontaminated by the
	 * OMCI wedge:
	 *   eth_ring=0, eth0 frames COMPLETE on VP2 -> VP2 is healthy; the fault is the OMCI
	 *                                              frame itself (its destination/content)
	 *   eth_ring=0, eth0 frames WEDGE on VP2    -> VP2 is broken; the frame is exonerated
	 *
	 * ⚠ This also settles a confound in every previous run: VP3 was seen to stall too,
	 * but ALWAYS after the first OMCI TX had already wedged the shared output stage - so
	 * "VP3 stalls" was never independent evidence about VP3.
	 */
	q = &tx->txq[eth_ring];

	/* Egress port set for this frame: one learned/fixed port, or a flood to
	 * every linked RJ45.  Computed before the ring check because a flood
	 * needs one descriptor per port.  This netdev is single-queue, so the
	 * qdisc serialises us and the plain counter updates need no atomics -
	 * the same assumption the fixed-ring choice above rests on. */
	/* ★DSA sends to exactly ONE jack (dp->index): a single descriptor, no
	 * FDB/flood.  The Linux bridge does BC/MC/unknown-unicast replication by
	 * queueing one skb per bridge port, so each still arrives here as a
	 * single-jack frame.  Non-DSA keeps the legacy per-DA flood selection. */
	if (is_dsa)
		ports = BIT(dsa_dest);
	else
		ports = ca_ni_lan_tx_ports(tx, skb->data);
	/* An empty set must be impossible: it would map the skb below and then
	 * attach it to no descriptor, leaking both and dropping the frame with
	 * no counter moving.  ca_ni_lan_tx_ports() guarantees non-empty; this
	 * keeps a future edit from reintroducing that silently. */
	if (WARN_ONCE(!ports, "lan_tx: empty egress port set\n"))
		ports = BIT(CA_NI_TX_PORT);
	nports = hweight32(ports);

	spin_lock(&q->lock);

	/* opportunistic reclaim, then ring-full check (stock keeps 2 spare) */
	if (cortina_ni_txq_free_desc(q) < CA_NI_TX_RESERVE_DESC + nports) {
		cortina_ni_tx_reclaim_q(ni, q);
		if (cortina_ni_txq_free_desc(q) <
		    CA_NI_TX_RESERVE_DESC + nports) {
			tx->tx_busy++;
			netif_stop_queue(ndev);
			mod_timer(&tx->reclaim_timer,
				  jiffies + CA_NI_RECLAIM_INTERVAL);
			spin_unlock(&q->lock);
			return NETDEV_TX_BUSY;
		}
	}

	daddr = dma_map_single(ni->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ni->dev, daddr))) {
		tx->drop_nomap++;
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		spin_unlock(&q->lock);
		return NETDEV_TX_OK;
	}
	/* the engine takes a 32-bit buffer address (+ the CCI selector);
	 * both DDR pools sit below 4 GB and the DMA mask enforces it */
	WARN_ON_ONCE(upper_32_bits(daddr));

	/*
	 * One direct-TX-to-LAN descriptor per egress port (plain frame, no
	 * header-A, HP=11), ALL pointing at the SAME mapped buffer: the engine
	 * only reads it, and it consumes the ring in order, so the LAST
	 * descriptor of the burst owns the skb + the mapping and every earlier
	 * one is marked `dup`.  A flood therefore costs extra descriptors only -
	 * no copy, no allocation, and TX stats still count the frame once.
	 */
	word1 = CA_NI_TX_DESC1_HP1 | CA_NI_TX_DESC1_HP0 |
		CA_NI_TX_DESC1_MODE_DIRECT |
		FIELD_PREP(CA_NI_TX_DESC1_CHK_SEL, CA_NI_TX_CHK_AUTO) |
		FIELD_PREP(CA_NI_TX_DESC1_LEN, len) |
		FIELD_PREP(CA_NI_TX_DESC1_COS, CA_NI_TX_COS);

	first = q->wptr;
	for (port = 0; ports; port++) {
		bool last;

		if (!(ports & BIT(port)))
			continue;
		ports &= ~BIT(port);
		last = !ports;
		w = word1 | FIELD_PREP(CA_NI_TX_DESC1_DEST, port);

		desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
		desc[0] = cpu_to_le32(lower_32_bits(daddr));
		desc[1] = cpu_to_le32(w);

		q->slot[q->wptr].skb = last ? skb : NULL;
		q->slot[q->wptr].addr = last ? daddr : 0;
		q->slot[q->wptr].len = last ? len : 0;
		q->slot[q->wptr].dup = last ? 0 : 1;
		q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;
		if (!last)
			tx->lan_dup++;
	}
	q->enq++;
	tx->last_word1 = w;
	cortina_ni_txlog(ni, q, "eth0", first, w, daddr, len);

	if (unlikely(tx_debug && q->enq <= 4)) {
		netdev_info(ndev,
			    "TX vp%u idx %u len %u ndesc %u last-desc %08x %08x\n",
			    q->vp, first, len, nports,
			    lower_32_bits(daddr), w);
		print_hex_dump(KERN_INFO, "TX frame: ", DUMP_PREFIX_OFFSET,
			       16, 1, skb->data, min(len, 64u), false);
	}

	/* TX timestamp BEFORE the doorbell: once the doorbell rings, HW may complete
	 * the frame and the reclaim path (timer/other CPU) can free this skb, so
	 * touching it after the unlock below is a use-after-free. */
	skb_tx_timestamp(skb);

	cortina_ni_tx_doorbell(ni, q);

	spin_unlock(&q->lock);

	mod_timer(&tx->reclaim_timer, jiffies + CA_NI_RECLAIM_INTERVAL);
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------ */
/* US PON control-frame (OMCI) TX — see cortina-ni.h / cortina-ni-regs.h */
/* ------------------------------------------------------------------ */

/* set once at TX probe; the GPON driver's responder calls in through it */
static struct cortina_ni *cortina_ni_pon_tx_ni;

/* 16-byte PON control-frame header (stock ca_ni_tx_encap_pon_control_packet,
 * disasm @0xa92ac: fixed DA/SA, link type 0xff 0xf1 = OMCI, byte [14] = 0,
 * byte [15] = (cos > 6) — this board's ko is the G3 build and OMCI goes out
 * at cos 7/8, so [15] = 0x01). */
static const u8 cortina_ni_pon_hdr[CA_NI_PON_HDR_LEN] = {
	0x00, 0x13, 0x25, 0x00, 0x00, 0x00,	/* DA */
	0x00, 0x13, 0x25, 0x00, 0x00, 0x01,	/* SA */
	0xff, 0xf1, 0x00, 0x01,			/* OMCI link type + G3 cos>=7 flag */
};

/*
 * fix#64 (a): build the US control frame into `frame` and apply the content knob.
 * `nth` is this send's 0-based index, so pon_fn can flip the content mid-boot.
 * Everything the caller does around this - ring, descriptor, buffer, context - is
 * untouched, so a behaviour change here is attributable to the BYTES alone.
 */
static void cortina_ni_pon_build_frame(struct cortina_ni_tx *tx, u8 *frame,
				       const u8 *pdu, unsigned int len,
				       u64 nth)
{
	unsigned int frame_len = CA_NI_PON_HDR_LEN + len;
	int mode = pon_fm;

	memcpy(frame, cortina_ni_pon_hdr, CA_NI_PON_HDR_LEN);
	memcpy(frame + CA_NI_PON_HDR_LEN, pdu, len);

	if (pon_fn && nth >= (u64)pon_fn)
		mode = 0;			/* A/B: back to the real frame */

	switch (mode) {
	case 1:		/* a plain Ethernet frame, same length, same everything else */
		eth_broadcast_addr(frame);
		ether_addr_copy(frame + ETH_ALEN, tx->netdev->dev_addr);
		frame[12] = 0x08;
		frame[13] = 0x00;
		memset(frame + 14, 0, frame_len - 14);
		break;
	case 3:		/* etype + broadcast DA */
		eth_broadcast_addr(frame);
		fallthrough;
	case 2:		/* ONLY the two ethertype bytes differ from the real frame */
		frame[12] = 0x08;
		frame[13] = 0x00;
		break;
	}
}

/* ------------------------------------------------------------------ */
/* fix#69: US OMCI over the NI CPUXRAM management FIFO                  */
/* ------------------------------------------------------------------ */
/*
 * ★★★★★ fix#69 (2026-08-10) — STOP USING THE DMA-LSO FOR OMCI.
 *
 * WHY (the measurement that forced this, R79/R80): with the OMCI descriptor stamped
 * byte-identically to an eth0 one, and again with every frame FLOODED to all four LAN
 * sockets, a raw AF_PACKET sniffer on the machine at the other end of the cable saw
 * ZERO frames from this board - not one, of any ethertype, from 02:96:07:f0:00:01 - and
 * Linux's own ping/ARP never arrived either.  Sixty seconds earlier, on the same board,
 * cable and socket, U-BOOT pinged that same host and TFTP'd a 15 MB image.
 * ⇒ No CPU-originated frame from this driver has EVER reached the wire, on any port.
 *   `rptr` advancing only means the fabric latched the descriptor; the ~11-15 doorbell
 *   allowance we kept measuring is just whatever queue they pile into filling up.
 *
 * AND THE ASYMMETRY THAT POINTS AT THE FIX: U-Boot's working TX does not touch the
 * DMA-LSO at all.  cortina_ni_send() (u-boot-2020.01 .../common/cortina_ni.c:955-1153)
 * writes the frame into the NI CPUXRAM management FIFO and rings CPU_CFG_TX_0.  Track A's
 * cortina_ni_pon_xram_send() (ca-ne/net/ca_ni.c:615) does the same and is the ONE path
 * that has ever delivered an OMCI reply to the PUC on this die (us_rx=1 enq=1,
 * hdr0=0x0016a207).  Two independent implementations, same silicon, both work.
 *
 * ★ AND IT CARRIES A SECOND, INDEPENDENT BUG FIX.  Track A's ground truth (its own
 * live PUC captures, ca_ni.c:640-690): PUC_GLOBAL_PLOAM_CFG carries us_omci_hdr_a_field
 * and us_ploam_hdr_a_field; live 0x8e1ef760 => OMCI = cos 6, PLOAM = cos 7.  This port
 * injects cos 7 on every path (CA_NI_PON_COS = 7), which is the PLOAM code point, so the
 * PUC classifies our OMCI reply as PLOAM, PUC_US_OMCI_HDR_A_CFG (0x80000606: enable=1
 * gemid=6 cos=6) never fires, the reply is never stamped onto the OMCC GEM, and the OLT
 * never hears it - exactly the Get-loop-then-Deactivate we see, and exactly why
 * us_omcc_cnt stays 0.  Stock's own header reads cos=6 pol_id=6; ours reads cos=7
 * pol_id=7.  So this path is sent with cos=6 / pol_id=6 by default.
 *
 * Ring: TX occupies XRAM slots 0x300..0x3FF (2 KiB, 8 bytes/slot), RX 0x000..0x2FF.
 * Per frame: [slot: word0 unused | word1 = Header-XT][8B HEADER_A][frame][FCS], then
 * publish next_link to CPU_CFG_TX_0.  The FCS is REQUIRED - Track A measured that
 * omitting it drops the frame before the PUC (us_rx=0).
 */
#define CA_NI_XRAM_TX_BASE	0x300	/* slot index, 8 bytes per slot */
#define CA_NI_XRAM_TX_TOP	0x3ff
#define CA_NI_XRAM_SIZE		0x2000
#define CA_NI_XRAM_WRAP		0x1800	/* byte offset of slot 0x300 */

static DEFINE_SPINLOCK(cortina_ni_xram_lock);

static int xram_tx;
module_param(xram_tx, int, 0644);
MODULE_PARM_DESC(xram_tx,
	"fix#69: send US OMCI through the NI CPUXRAM management FIFO instead of the DMA-LSO");
static int xram_cos = 6;	/* PUC's US-OMCI code point; 7 = PLOAM (the old bug) */
module_param(xram_cos, int, 0644);
MODULE_PARM_DESC(xram_cos, "fix#69: HEADER_A cos for the CPUXRAM OMCI path (6 = US OMCI)");
static int xram_ldpid = CA_NI_PON_LDPID;
module_param(xram_ldpid, int, 0644);
MODULE_PARM_DESC(xram_ldpid, "fix#69: HEADER_A ldpid for the CPUXRAM OMCI path");
static int xram_polid = 6;
module_param(xram_polid, int, 0644);
MODULE_PARM_DESC(xram_polid, "fix#69: HEADER_A pol_id for the CPUXRAM OMCI path");
/*
 * ★★★ HEADER_A pkt_size.  Track A's live PUC-reaching capture stamps
 *   lo=0x2020227e -> pkt_size = 64 = the PRE-pad, PRE-FCS L2 length
 *   (16B PON control header + 48B OMCI PDU),
 * while the buffer it hands the engine is the padded 72-byte one.  The GEM framer
 * uses pkt_size to size the upstream OMCC SDU, and an OMCI SDU must be exactly 48
 * bytes; stamping the padded length (72) makes the framer emit a malformed SDU.
 * Default = stamp the raw frame length, exactly as Track A does with
 * ca_ni_pon_nofcs=1.  Set 1 to stamp the padded length for an A/B.
 */
static int xram_pktsz_full;
module_param(xram_pktsz_full, int, 0644);
MODULE_PARM_DESC(xram_pktsz_full,
	"fix#69 HEADER_A pkt_size: 0 = raw L2 length (64), 1 = padded+FCS, >1 = that literal value (stock's live capture reads 48 = the OMCI SDU alone)");

/*
 * ★★★ fix#71: the 8-byte alignment pad.  U-Boot pads "packet size + CRC" to an 8-byte
 * multiple (a 2015 workaround in cortina_ni_send); stock's OMCI frame is NOT padded - it
 * is exactly 64 bytes (16B control header + 48B OMCI PDU) plus a 4-byte FCS.  The PUC
 * derives the length it stamps on the OMCC SDU from the bytes it actually receives, and
 * with the pad in place it reads 44 where stock reads 48 (PUC hdr0 = 0x00162206 vs stock's
 * 0x00182206 - one field, four bytes).  Default = no pad, i.e. stock's shape.
 */
/*
 * ★★★ fix#76 (2026-08-10): the LSPID the PUC actually observes.
 *
 * Live stock, captured while it was answering the OLT, latches PUC hdr0 = 0x00183006 for its
 * US OMCI frame (and 0x00203108 for the other class).  Decoded with the HEADER_A layout
 * cos[2:0] ldpid[8:3] lspid[14:9] pkt_size[28:15] fe_bypass[29], BOTH of stock's headers
 * carry lspid = 0x18, where ours carries 0x11 - the value Track A's notes recommended and
 * which no live stock capture ever supported.  (Same capture: stock's pkt_size reads 48, the
 * OMCI SDU, where ours reads 45.)
 * Capture: golden_2026-08-10_STOCK_PONWIN_197.txt / the [PUC live] line of stock_ponwin.log.
 */
static int xram_lspid = 0x18;
module_param(xram_lspid, int, 0644);
MODULE_PARM_DESC(xram_lspid,
	"fix#76: HEADER_A lspid for the CPUXRAM OMCI path (live stock shows 0x18; Track A used 0x11)");

static bool xram_nopad = true;
module_param(xram_nopad, bool, 0644);
MODULE_PARM_DESC(xram_nopad,
	"fix#71: skip the 8-byte alignment pad so the frame is stock-shaped (64 + FCS)");

static bool xram_det = true;
module_param(xram_det, bool, 0644);
MODULE_PARM_DESC(xram_det,
	"fix#69: re-init the CPUXRAM ring before every frame (deterministic, never wraps)");

static int cortina_ni_pon_xram_tx(struct cortina_ni *ni, const u8 *frame,
				  unsigned int flen)
{
	void __iomem *nib = ni->win[CA_NI_WIN_NI];
	void __iomem *xram = ni->win[CA_NI_WIN_XRAM];
	static u8 xb[2048];
	unsigned long flags;
	u32 rd, wr, hdrxt, next_link, off, i, words, crc, lo, hi;
	unsigned int nlen, pktsz;
	int poll, ret = 0;

	if (!xram || !nib || !flen || flen > 1536)
		return -EINVAL;

	spin_lock_irqsave(&cortina_ni_xram_lock, flags);

	if (xram_det || !ni->tx->xram_inited) {
		/* TX window 0x300..0x3ff, then toggle tx_0_cpu_pkt_dis (bit9) to
		 * re-arm the engine after an address-config change - U-Boot does
		 * exactly this (cortina_ni.c:688-698). RMW bit 9 ONLY: bit 0, the
		 * promisc field [15:14] and the two soft-resets share this word. */
		writel(FIELD_PREP(CA_NI_XRAM_ADRCFG_TOP, CA_NI_XRAM_TX_TOP) |
		       FIELD_PREP(CA_NI_XRAM_ADRCFG_BASE, CA_NI_XRAM_TX_BASE),
		       nib + CA_NI_XRAM_ADRCFG_TX0);
		writel(readl(nib + CA_NI_XRAM_CFG) | CA_NI_XRAM_CFG_TX0_DIS,
		       nib + CA_NI_XRAM_CFG);
		writel(readl(nib + CA_NI_XRAM_CFG) & ~CA_NI_XRAM_CFG_TX0_DIS,
		       nib + CA_NI_XRAM_CFG);
		ni->tx->xram_inited = true;
	}

	if (xram_det) {
		/* fresh ring every frame: no wrap, so pkt_size and the byte layout
		 * are identical on every send (Track A's ca_ne_xram_det) */
		wr = CA_NI_XRAM_TX_BASE;
		writel(wr, nib + CA_NI_XRAM_CPU_CFG_TX0);
	} else {
		wr = readl(nib + CA_NI_XRAM_CPU_CFG_TX0) & CA_NI_XRAM_PTR_MASK;
		if (wr < CA_NI_XRAM_TX_BASE || wr > CA_NI_XRAM_TX_TOP) {
			wr = CA_NI_XRAM_TX_BASE;
			writel(wr, nib + CA_NI_XRAM_CPU_CFG_TX0);
		}
		/* the FIFO is available only when HW has consumed everything */
		for (poll = 0; poll < 200; poll++) {
			rd = readl(nib + CA_NI_XRAM_CPU_STAT_TX0) &
			     CA_NI_XRAM_PTR_MASK;
			if (rd == wr)
				break;
			udelay(2);
		}
		if (poll == 200) {
			ret = -EBUSY;	/* caller drops; the OLT retransmits */
			goto out;
		}
	}

	/* [8B HEADER_A][frame][pad to 60][FCS] */
	memcpy(xb + CA_NI_XRAM_HDRA_LEN, frame, flen);
	nlen = flen;
	if (!xram_nopad && ((nlen + ETH_FCS_LEN) % 8) != 0) {
		unsigned int pad = 8 - ((nlen + ETH_FCS_LEN) % 8);

		memset(xb + CA_NI_XRAM_HDRA_LEN + nlen, 0, pad);
		nlen += pad;
	}
	if (nlen < ETH_ZLEN) {
		memset(xb + CA_NI_XRAM_HDRA_LEN + nlen, 0, ETH_ZLEN - nlen);
		nlen = ETH_ZLEN;
	}
	/* The FCS is REQUIRED for L2 delivery to the PUC - Track A measured that
	 * without it the frame is dropped before the PUC (us_rx stays 0). */
	crc = ~crc32_le(~0u, xb + CA_NI_XRAM_HDRA_LEN, nlen);
	memcpy(xb + CA_NI_XRAM_HDRA_LEN + nlen, &crc, ETH_FCS_LEN);
	nlen += ETH_FCS_LEN;
	/* 0 = the raw L2 length (Track A's proven value, 64); 1 = the padded/FCS
	 * length; anything larger is stamped verbatim - stock's own live capture
	 * reads pkt_size = 48, i.e. the OMCI SDU without the 16-byte PON control
	 * header, and stock is the only source that ever got a reply emitted. */
	if (xram_pktsz_full > 1)
		pktsz = (u32)xram_pktsz_full;
	else
		pktsz = xram_pktsz_full ? nlen : flen;

	/* HEADER_A: word0 = {no_drop, pol_id}, word1 = {cos, ldpid, lspid,
	 * pkt_size, fe_bypass} - stock's word order (low address = pkt_info) */
	lo = FIELD_PREP(CA_NI_PON_HDRA_LO_COS, xram_cos & 7) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LDPID, xram_ldpid & 0x3f) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LSPID, xram_lspid & 0x3f) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_PKT_SIZE, pktsz) |
	     CA_NI_PON_HDRA_LO_FE_BYPASS;
	hi = CA_NI_PON_HDRA_HI_NO_DROP |
	     FIELD_PREP(CA_NI_PON_HDRA_HI_POL_ID, xram_polid & 0x1ff);
	((u32 *)xb)[0] = hi;
	((u32 *)xb)[1] = lo;

	next_link = wr + (nlen + 7 + CA_NI_XRAM_HDRA_LEN) / 8 + 1;
	if (next_link > CA_NI_XRAM_TX_TOP)
		next_link = CA_NI_XRAM_TX_BASE +
			    (next_link - (CA_NI_XRAM_TX_TOP + 1));
	hdrxt = CA_NI_XRAM_XT_OWN | CA_NI_XRAM_XT_HDRA |
		FIELD_PREP(CA_NI_XRAM_XT_BYTES_VALID, nlen % 8) |
		FIELD_PREP(CA_NI_XRAM_XT_NEXT_LINK, next_link);

	/* Header-XT goes in the SECOND word of the slot; data follows. Every
	 * offset is bounds-checked - an unclamped one walks off the ioremap. */
	off = wr * 8 + 4;
	if (WARN_ON_ONCE(off >= CA_NI_XRAM_SIZE))
		off = CA_NI_XRAM_WRAP;
	writel(hdrxt, xram + off);
	off += 4;
	if (off >= CA_NI_XRAM_SIZE)
		off = CA_NI_XRAM_WRAP;
	words = (nlen + CA_NI_XRAM_HDRA_LEN + 3) / 4;
	for (i = 0; i < words; i++) {
		writel(((u32 *)xb)[i], xram + off);
		off += 4;
		if (off >= CA_NI_XRAM_SIZE)
			off = CA_NI_XRAM_WRAP;
	}
	wmb();
	writel(next_link, nib + CA_NI_XRAM_CPU_CFG_TX0);

	ni->tx->xram_enq++;
	if (ni->tx->xram_enq <= 4)
		dev_info(ni->dev,
			 "fix#69: XRAM OMCI #%u hdrxt=%08x hdr_a=%08x_%08x [cos=%d ldpid=%d lspid=0x%02x pkt_size=%u pol_id=%d] len=%u->%u next_link=0x%x rd=%u\n",
			 ni->tx->xram_enq, hdrxt, hi, lo, xram_cos, xram_ldpid,
			 (unsigned int)(xram_lspid & 0x3f), pktsz, xram_polid,
			 flen, nlen,
			 next_link,
			 (unsigned int)(readl(nib + CA_NI_XRAM_CPU_STAT_TX0) &
					CA_NI_XRAM_PTR_MASK));
out:
	spin_unlock_irqrestore(&cortina_ni_xram_lock, flags);
	return ret;
}

int cortina_ni_pon_tx(const u8 *pdu, unsigned int len)
{
	struct cortina_ni *ni = READ_ONCE(cortina_ni_pon_tx_ni);
	struct cortina_ni_tx *tx;
	struct cortina_ni_txq *q;
	unsigned int frame_len, slot;
	unsigned long free_slots;
	dma_addr_t blk_dma;
	__le32 *desc, *w;
	u32 lo, hi;
	u8 *blk;

	if (!ni || !ni->tx || !ni->tx->pon_buf)
		return -ENODEV;
	tx = ni->tx;
	if (!len || len > CA_NI_PON_TX_PDU_MAX)
		return -EINVAL;
	frame_len = CA_NI_PON_HDR_LEN + len;	/* 64 for a 48B OMCI PDU */

	/*
	 * ★★★★★ fix#69: leave through the NI CPUXRAM management FIFO instead of the
	 * DMA-LSO.  The DMA-LSO has never put a CPU frame on this board's wire (R79/R80),
	 * while both U-Boot and Track A transmit successfully through CPUXRAM on this
	 * exact die.  The PON control header is built the same way; only the egress
	 * engine and the HEADER_A cos/pol_id change.
	 */
	if (xram_tx) {
		u8 frame[CA_NI_PON_HDR_LEN + CA_NI_PON_TX_PDU_MAX];

		memcpy(frame, cortina_ni_pon_hdr, CA_NI_PON_HDR_LEN);
		memcpy(frame + CA_NI_PON_HDR_LEN, pdu, len);
		tx->pon_enq++;
		return cortina_ni_pon_xram_tx(ni, frame, frame_len);
	}

	if (omci_via_xmit) {
		/*
		 * ★★★★★ fix#64 (b) — THE PATH HALF OF THE BISECTION.
		 *
		 * Hand the identical 64 bytes to the stack so they leave through
		 * cortina_ni_start_xmit() behind the qdisc - the exact path eth0's
		 * frames take.  Boot this with cortina_ni.eth_ring=0 and the RING is
		 * held constant too (both on VP2), leaving the enqueue path as the
		 * only remaining difference:
		 *   rptr ADVANCES -> the bytes are fine; cortina_ni_pon_tx() (its
		 *                    descriptor bookkeeping, its NAPI-softirq context,
		 *                    or its scratch handling) is the fault;
		 *   rptr STILL 0  -> the path is fine and it is the BYTES - which
		 *                    pon_fm then pins down to the field.
		 * netif_running() guards the window before OpenWrt brings eth0 up;
		 * the OLT simply retransmits, exactly as for any -EBUSY drop.
		 */
		struct sk_buff *skb;
		u8 *frame;

		if (!tx->netdev || !netif_running(tx->netdev))
			return -EBUSY;
		skb = netdev_alloc_skb(tx->netdev, frame_len + NET_IP_ALIGN);
		if (!skb)
			return -ENOMEM;
		skb_reserve(skb, NET_IP_ALIGN);
		frame = skb_put(skb, frame_len);
		cortina_ni_pon_build_frame(tx, frame, pdu, len, tx->pon_enq);
		skb->dev = tx->netdev;
		skb->protocol = cpu_to_be16(frame[12] << 8 | frame[13]);
		skb_reset_mac_header(skb);
		tx->pon_enq++;
		if (tx->pon_enq <= 3)
			dev_info(ni->dev,
				 "fix#64: OMCI #%llu via start_xmit (eth_ring=%d) len=%u\n",
				 tx->pon_enq, eth_ring, frame_len);
		return dev_queue_xmit(skb) == NET_XMIT_SUCCESS ? 0 : -EBUSY;
	}

	/* always txq[0]: pon_busy and the scratch are guarded by its lock;
	 * _bh so the responder may call from NAPI softirq or process ctx */
	q = &tx->txq[0];
	spin_lock_bh(&q->lock);

	/*
	 * Reclaim UNCONDITIONALLY here: the scratch (CA_NI_PON_TX_SLOTS) is much
	 * smaller than the descriptor ring, so under a fast OMCI burst (the OLT's
	 * MIB-Upload-Next walk sends ~1 message every ~25ms) the scratch runs out
	 * long before the ring does.  If we only reclaimed on ring-low, pon_busy
	 * would never get cleared during the burst and we'd -EBUSY-drop replies
	 * even though the HW already drained them.  A dropped reply is fatal to
	 * the stateful MIB-Upload-Next walk (the responder advances its pointer,
	 * so the OLT's retransmit gets the wrong entry -> the upload desyncs and
	 * the OLT aborts).  So free every completed slot on every send.
	 */
	cortina_ni_tx_reclaim_q(ni, q);
	/*
	 * ⚠ fix#64: allocate the scratch slot from an explicitly-masked unsigned
	 * long.  The old `ffz(tx->pon_busy)` evaluated `~x` in unsigned int, so a
	 * FULL bitmap (pon_busy == 0xffffffff, which happens on every wedged run
	 * once nothing is ever reclaimed) fed __ffs(0) - undefined - and the
	 * `slot >= CA_NI_PON_TX_SLOTS` guard did not hold.  The out-of-range slot
	 * then indexed past the one-page coherent scratch and the memcpy below
	 * took a level-3 translation fault (Oops 0x96000047 at ~162 s, seen on
	 * every scratch-path run this session), truncating the measurement.  A
	 * full scratch must degrade to -EBUSY, never to a data abort.
	 */
	free_slots = ~(unsigned long)tx->pon_busy &
		     GENMASK(CA_NI_PON_TX_SLOTS - 1, 0);
	if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC + 2 ||
	    !free_slots) {
		tx->pon_fail++;
		spin_unlock_bh(&q->lock);
		return -EBUSY;		/* caller drops; the OLT retransmits */
	}
	slot = __ffs(free_slots);
	tx->pon_busy |= BIT(slot);

	blk = (u8 *)tx->pon_buf + slot * CA_NI_PON_TX_SLOT_SZ;
	blk_dma = tx->pon_buf_dma + slot * CA_NI_PON_TX_SLOT_SZ;

	/* frame = 16-byte PON header + the OMCI PDU (coherent, no mapping),
	 * with fix#64's content knob applied */
	cortina_ni_pon_build_frame(tx, blk + CA_NI_PON_TX_FRAME_OFF, pdu, len,
				   tx->pon_enq);

	/* Header block {LSO para0 = 0, LSO para1 = pkt_size, HEADER_A}: plain
	 * little-endian stores, exactly the stock stores (disasm
	 * __ca_ni_send_single_pkt: `stp wzr, w<size>, [x4]` then the 64-bit
	 * HEADER_A at +8).  ★ WORD ORDER (stock-proven): the word at +8 is
	 * the PKT_INFO half (no_drop/pol_id/cpu_flg, spec bits 32..63) and
	 * the word at +12 is the cos/ldpid/lspid/pkt_size half (bits 0..31)
	 * — the pol_id bfi lands in the LOW-address word.  pol_en stays 0
	 * for OMCI (the G3 branch passes pol=INVALID; only the 0xff/0xf1
	 * override sets pol_id = (DA[5]&0x3f)*8+7 = 7, without pol_en). */
	lo = FIELD_PREP(CA_NI_PON_HDRA_LO_COS,
			pon_cos >= 0 ? (u32)pon_cos : CA_NI_PON_COS) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LDPID, CA_NI_PON_LDPID) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LSPID, pon_lspid & 0x3f) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_PKT_SIZE, frame_len) |
	     CA_NI_PON_HDRA_LO_FE_BYPASS;
	hi = CA_NI_PON_HDRA_HI_NO_DROP |
	     FIELD_PREP(CA_NI_PON_HDRA_HI_POL_ID, CA_NI_PON_POL_ID);
	w = (__le32 *)blk;
	w[0] = 0;
	w[1] = cpu_to_le32(frame_len);
	w[2] = cpu_to_le32(hi);		/* +8:  pkt_info word (stock order) */
	w[3] = cpu_to_le32(lo);		/* +12: cos/ldpid/pkt_size word */

	if (pon_skb_tx || pon_like_eth) {
		/*
		 * ★★★★★ fix#63 (2026-08-09) — IS IT THE BUFFER?
		 *
		 * fix#62 proved the fault is the OMCI FRAME, not VP2: with eth_ring=0,
		 * eth0's own frames retire on VP2 (rptr tracks wptr through 11 descriptors)
		 * and only the first OMCI frame sticks.  fix#61 had already exonerated the
		 * descriptor format.  Three differences remain between an eth0 frame and the
		 * OMCI frame on the same ring in the same format, and this tests the first:
		 *
		 *   BUFFER: OMCI is sent from the COHERENT dmam_alloc_coherent scratch
		 *   (tx->pon_buf, at blk_dma + CA_NI_PON_TX_FRAME_OFF = offset 32 inside a
		 *   128-byte slot), whereas eth0 sends from a dma_map_single'd skb.
		 *
		 * That is a live suspicion on this SoC: the RX side needed Track A's
		 * 0x2_00000000 alias (cfg4.axi_top_bit) because the NE DMA is NOT coherent
		 * with the CPU, and the TX engine is programmed with
		 * CA_DMA_LSO_AXI_USER_PAT_NOCOH.  A buffer the engine cannot read would stall
		 * it exactly where it stalls (txfout_cs=2, no error latched).
		 *
		 * So: copy the identical 64 bytes into an skb, map it exactly the way
		 * cortina_ni_start_xmit does, and send it with the same direct-TX descriptor.
		 * Everything except the buffer's provenance is held constant.
		 *   rptr ADVANCES -> the coherent scratch / its DMA attributes are the bug
		 *   rptr STILL 0  -> the buffer is exonerated; it is the frame CONTENT
		 */
		struct sk_buff *skb;
		dma_addr_t daddr;
		u32 w1;
		int nburst, i;

		skb = netdev_alloc_skb(tx->netdev, frame_len);	/* GFP_ATOMIC */
		if (!skb) {
			tx->pon_busy &= ~BIT(slot);
			tx->pon_fail++;
			spin_unlock_bh(&q->lock);
			return -ENOMEM;
		}
		skb_put_data(skb, blk + CA_NI_PON_TX_FRAME_OFF, frame_len);
		daddr = dma_map_single(ni->dev, skb->data, frame_len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(ni->dev, daddr)) {
			dev_kfree_skb_any(skb);
			tx->pon_busy &= ~BIT(slot);
			tx->pon_fail++;
			spin_unlock_bh(&q->lock);
			return -ENOMEM;
		}

		/* the scratch slot is not used by this path - hand it straight back */
		tx->pon_busy &= ~BIT(slot);

		w1 = CA_NI_TX_DESC1_HP1 | CA_NI_TX_DESC1_HP0 |
		     CA_NI_TX_DESC1_MODE_DIRECT |
		     FIELD_PREP(CA_NI_TX_DESC1_CHK_SEL,
				pon_chk_sel >= 0 ? (u32)pon_chk_sel
						 : CA_NI_TX_CHK_AUTO) |
		     FIELD_PREP(CA_NI_TX_DESC1_LEN, frame_len) |
		     FIELD_PREP(CA_NI_TX_DESC1_COS,
				pon_cos >= 0 ? (u32)pon_cos : CA_NI_PON_COS) |
		     FIELD_PREP(CA_NI_TX_DESC1_DEST, pon_direct_dest);
		if (pon_direct_wan)
			w1 |= CA_NI_TX_DESC1_TO_WAN;

		/*
		 * ★ fix#65: stamp this descriptor exactly as cortina_ni_start_xmit stamps
		 * eth0's - same COS, same linked-port DEST, TO_WAN clear.  eth0's frames
		 * retire on this very ring (fix#62), so if the OMCI frame now retires too,
		 * the wedge is the destination/COS the frame is offered to, not the frame,
		 * the buffer, the format, the ring or the path.
		 */
		if (pon_like_eth) {
			u32 ports = ca_ni_lan_tx_ports(tx,
						       blk + CA_NI_PON_TX_FRAME_OFF);

			w1 = CA_NI_TX_DESC1_HP1 | CA_NI_TX_DESC1_HP0 |
			     CA_NI_TX_DESC1_MODE_DIRECT |
			     FIELD_PREP(CA_NI_TX_DESC1_CHK_SEL,
					pon_chk_sel >= 0 ? (u32)pon_chk_sel
							 : CA_NI_TX_CHK_AUTO) |
			     FIELD_PREP(CA_NI_TX_DESC1_LEN, frame_len) |
			     FIELD_PREP(CA_NI_TX_DESC1_COS, CA_NI_TX_COS) |
			     FIELD_PREP(CA_NI_TX_DESC1_DEST,
					ports ? __ffs(ports) : CA_NI_TX_PORT);
		}

		/* fix#66: N back-to-back copies under one doorbell (N=1 = normal).
		 * Bounded by the free-descriptor check above (which reserves 2). */
		nburst = clamp(pon_burst, 1, 8);
		if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC + nburst)
			nburst = 1;

		for (i = 0; i < nburst; i++) {
			bool last = (i == nburst - 1);

			desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
			desc[0] = cpu_to_le32(lower_32_bits(daddr));
			desc[1] = cpu_to_le32(w1);
			cortina_ni_txlog(ni, q, last ? "omci" : "omci-dup",
					 q->wptr, w1, daddr, frame_len);
			/* pon=1 WITH an skb: reclaim unmaps + frees it and does NOT
			 * count it on eth0's stats (see cortina_ni_tx_reclaim_q).
			 * Only the LAST copy owns the mapping; the engine consumes
			 * the ring in order, so the owner retires last. */
			q->slot[q->wptr].skb = last ? skb : NULL;
			q->slot[q->wptr].addr = last ? daddr : 0;
			q->slot[q->wptr].len = last ? frame_len : 0;
			q->slot[q->wptr].pon = 1;
			q->slot[q->wptr].dup = last ? 0 : 1;
			q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;
		}
		if (tx->pon_enq < 3)
			dev_info(ni->dev,
				 "fix#63: skb-mapped OMCI desc1=%08x dma=%pad len=%u\n",
				 w1, &daddr, frame_len);
	} else if (pon_direct_tx) {
		/*
		 * ★★★★★ fix#61 (2026-08-09) — DISCRIMINATOR: send the OMCI frame as a
		 * SINGLE direct-TX (HP=11) descriptor instead of the HP=01 header-mode chain.
		 *
		 * WHY: fix#59 showed the engine choked on the FIRST header-mode SOF
		 * descriptor and never advanced (cdes == fdes == ring base while wptr climbed
		 * to 0x40), and fix#60 proved the whole HDRA_CFG destination field space
		 * (ldpid/cos/nodrop/deepq) is inert - so the stall is not about where the
		 * frame is going.  Meanwhile eth0 transmits happily through the SAME engine
		 * using the OTHER word1 layout.  Note the header-mode (HP=01) path has NEVER
		 * successfully transmitted anything on this port: cortina_ni_pon_data_tx()
		 * uses it too and its data_enq counter has never left 0.
		 *
		 * This isolates the two possibilities in one boot:
		 *   rptr ADVANCES  -> the HP=01 header-mode encoding (or the 16-byte header
		 *                     block it points at) is wrong for this silicon;
		 *   rptr STILL 0   -> the PON destination genuinely cannot be reached, and
		 *                     the descriptor format is exonerated.
		 *
		 * There is no HEADER_A in this format, so the destination must come from
		 * HDRA_CFG(VP2) (fix#60 points it at ldpid 15 / cos 6) and/or the TO_WAN bit
		 * - which is DEFINED IN THE HEADER BUT USED NOWHERE in the driver, so this is
		 * also its first exercise.  Both are module params so the space is sweepable
		 * without a rebuild.
		 */
		u32 w1 = CA_NI_TX_DESC1_HP1 | CA_NI_TX_DESC1_HP0 |
			 CA_NI_TX_DESC1_MODE_DIRECT |
			 FIELD_PREP(CA_NI_TX_DESC1_CHK_SEL,
				    pon_chk_sel >= 0 ? (u32)pon_chk_sel
						     : CA_NI_TX_CHK_AUTO) |
			 FIELD_PREP(CA_NI_TX_DESC1_LEN, frame_len) |
			 FIELD_PREP(CA_NI_TX_DESC1_COS,
				    pon_cos >= 0 ? (u32)pon_cos : CA_NI_PON_COS) |
			 FIELD_PREP(CA_NI_TX_DESC1_DEST, pon_direct_dest);

		if (pon_direct_wan)
			w1 |= CA_NI_TX_DESC1_TO_WAN;

		desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
		desc[0] = cpu_to_le32(lower_32_bits(blk_dma +
						    CA_NI_PON_TX_FRAME_OFF));
		desc[1] = cpu_to_le32(w1);
		cortina_ni_txlog(ni, q, "omci", q->wptr, w1,
				 blk_dma + CA_NI_PON_TX_FRAME_OFF, frame_len);
		q->slot[q->wptr].skb = NULL;
		q->slot[q->wptr].addr = 0;
		q->slot[q->wptr].len = 0;
		q->slot[q->wptr].pon = 2 + slot;	/* releases the scratch slot */
		q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;
		if (tx->pon_enq < 3)
			dev_info(ni->dev,
				 "fix#61: direct-TX OMCI desc1=%08x len=%u (wan=%d dest=%d)\n",
				 w1, frame_len, pon_direct_wan, pon_direct_dest);
	} else {
	/* descriptor pair: SOF + HP=01 header block, then the EOF frame */
	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(blk_dma));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_SOF | CA_NI_TX_DESC1_HP0 |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN,
					 CA_NI_PON_HDR_BLK_LEN));
	q->slot[q->wptr].skb = NULL;
	q->slot[q->wptr].addr = 0;
	q->slot[q->wptr].len = 0;
	q->slot[q->wptr].pon = 1;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(blk_dma + CA_NI_PON_TX_FRAME_OFF));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_EOF |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN, frame_len));
	q->slot[q->wptr].skb = NULL;
	q->slot[q->wptr].addr = 0;
	q->slot[q->wptr].len = 0;
	q->slot[q->wptr].pon = 2 + slot;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;
	}

	q->enq++;
	tx->pon_enq++;

	cortina_ni_tx_doorbell(ni, q);

	spin_unlock_bh(&q->lock);

	mod_timer(&tx->reclaim_timer, jiffies + CA_NI_RECLAIM_INTERVAL);
	return 0;
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_tx);

/*
 * US PON DATA (WAN) TX — Stage D.  Same 2-descriptor HEADER_A chain as the
 * OMCI path, but the frame is the dma-mapped skb (up to MTU-size, too big
 * for the 128-byte scratch) and the HEADER_A is a plain data header: ldpid =
 * PON (0x07, no 9th-queue offset), cos = the data queue, fe_bypass, no_drop,
 * no policer.  In the PUC 8Q VoQ map the frame lands in T-CONT 7 queue 0
 * (VoQ 56), whose US_PORT_ID the GPON driver bound to the OLT-assigned data
 * GEM and whose T-CONT CAM entry it bound to the OLT's data alloc-id.
 * Descriptor bookkeeping: SOF slot releases the header-block scratch (pon =
 * 2+slot), EOF slot carries the skb (normal unmap+consume reclaim).
 */
/*
 * ★fix#102 (2026-08-11) - step (1) of stock's queue_add enable-triple: VoQ ADMISSION.
 *
 * Stock's dal_rtl9607f_ponmac_queue_add() points every provisioned data VoQ at TE_CB
 * threshold profile 3 (PON_QM_EN_ENQUEUE_PROFILE_NUM) [dal_rtl9607f_ponmac.c:1161-1181].
 * THIS DRIVER HAS NEVER WRITTEN ANY ni 0x9xxx REGISTER - grep-verified empty across the whole
 * cortina/ tree.  That is why ~25 builds of "every register matches stock" never found it:
 * no golden ever sampled this block.  An admission gate also produces exactly the symptom we
 * measure - ZERO ENQUEUE (us_mib[sel=8]=0) with tx_dropped=0 - rather than drops.
 *
 * Addresses are the TAURUS overrides, not the generic map
 * [src/cane_full/include/reg/taurus_addr_override.h:2749-2760]:
 *   SELECT_MEM  ACCESS 0x95c0 / DATA 0x95c4      indirect: DATA then ACCESS = GO|WR|idx
 *   PROFILE_MEM ACCESS 0x95b0 / DATA0..2 0x95bc,0x95b8,0x95b4
 * (The generic rtl9607f map differs - this file has been bitten three times by that.)
 */
#define CA_NI_TECB_SEL_ACCESS	0x95c0
#define CA_NI_TECB_SEL_DATA	0x95c4
#define CA_NI_TECB_PROF_ACCESS	0x95b0
#define CA_NI_TECB_PROF_DATA0	0x95bc
#define CA_NI_TECB_PROF_DATA1	0x95b8
#define CA_NI_TECB_PROF_DATA2	0x95b4
#define CA_NI_TECB_ADMIT_PROFILE 3

int cortina_ni_qm_voq_admit(unsigned int qm_voq_idx)
{
	struct cortina_ni *ni = READ_ONCE(cortina_ni_pon_tx_ni);
	void __iomem *b;

	if (!ni)
		return -ENODEV;
	b = ni_base(ni);
	/* seed profile 3 wide open (stock's memset(0xff) default) - idempotent */
	writel(0xffffffff, b + CA_NI_TECB_PROF_DATA0);
	writel(0xffffffff, b + CA_NI_TECB_PROF_DATA1);
	writel(0xffffffff, b + CA_NI_TECB_PROF_DATA2);
	writel(0xc0000000 | CA_NI_TECB_ADMIT_PROFILE, b + CA_NI_TECB_PROF_ACCESS);
	/* point this VoQ at it */
	writel(CA_NI_TECB_ADMIT_PROFILE, b + CA_NI_TECB_SEL_DATA);
	writel(0xc0000000 | (qm_voq_idx & 0xff), b + CA_NI_TECB_SEL_ACCESS);
	return 0;
}
EXPORT_SYMBOL_GPL(cortina_ni_qm_voq_admit);

netdev_tx_t cortina_ni_pon_data_tx(struct sk_buff *skb,
				   struct net_device *ndev)
{
	struct cortina_ni *ni = READ_ONCE(cortina_ni_pon_tx_ni);
	struct cortina_ni_tx *tx;
	struct cortina_ni_txq *q;
	unsigned int len, slot;
	unsigned long free_slots;
	dma_addr_t blk_dma, daddr;
	__le32 *desc, *w;
	u32 lo, hi;
	u8 *blk;

	if (!ni || !ni->tx || !ni->tx->pon_buf) {
		ndev->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	tx = ni->tx;

	if (skb_padto(skb, ETH_ZLEN))	/* freed by the helper on failure */
		return NETDEV_TX_OK;
	len = max_t(unsigned int, skb->len, ETH_ZLEN);
	if (unlikely(len > CA_NI_TX_MAX_FRAME || skb_linearize(skb))) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* txq[0], shared with the OMCI path: pon_busy + scratch live under
	 * its lock (ndo_start_xmit runs with BH off -> plain spin_lock) */
	q = &tx->txq[0];
	spin_lock(&q->lock);

	cortina_ni_tx_reclaim_q(ni, q);	/* scratch << ring: reclaim every send */
	/* same masked allocation as the OMCI path - see the fix#64 note there:
	 * plain ffz() on a full u32 bitmap is undefined and hands back an
	 * out-of-range slot that indexes past the one-page coherent scratch */
	free_slots = ~(unsigned long)tx->pon_busy &
		     GENMASK(CA_NI_PON_TX_SLOTS - 1, 0);
	if (cortina_ni_txq_free_desc(q) <= CA_NI_TX_RESERVE_DESC + 2 ||
	    !free_slots) {
		tx->pon_fail++;
		ndev->stats.tx_dropped++;
		spin_unlock(&q->lock);
		dev_kfree_skb_any(skb);	/* WAN clients retransmit (DHCP/TCP) */
		return NETDEV_TX_OK;
	}
	slot = __ffs(free_slots);

	daddr = dma_map_single(ni->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ni->dev, daddr))) {
		tx->drop_nomap++;
		ndev->stats.tx_dropped++;
		spin_unlock(&q->lock);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	WARN_ON_ONCE(upper_32_bits(daddr));
	tx->pon_busy |= BIT(slot);

	blk = (u8 *)tx->pon_buf + slot * CA_NI_PON_TX_SLOT_SZ;
	blk_dma = tx->pon_buf_dma + slot * CA_NI_PON_TX_SLOT_SZ;

	/* header block {LSO para0 = 0, LSO para1 = pkt_size, HEADER_A}: the
	 * same stock word order as the OMCI path (+8 = pkt_info half, +12 =
	 * cos/ldpid/pkt_size half) */
	lo = FIELD_PREP(CA_NI_PON_HDRA_LO_COS, CA_NI_PON_DATA_COS) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LDPID,
			pon_data_ldpid < 0 ? CA_NI_PON_DATA_LDPID
					   : (u32)pon_data_ldpid & 0x3f) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_LSPID, pon_lspid & 0x3f) |
	     FIELD_PREP(CA_NI_PON_HDRA_LO_PKT_SIZE, len) |
	     (pon_data_fe_bypass ? CA_NI_PON_HDRA_LO_FE_BYPASS : 0);
	hi = CA_NI_PON_HDRA_HI_NO_DROP |
	     (pon_data_pol_en ? FIELD_PREP(CA_NI_PON_HDRA_HI_POL_EN, 1) : 0) |
	     FIELD_PREP(CA_NI_PON_HDRA_HI_POL_ID, pon_data_polid & 0x1ff);
	w = (__le32 *)blk;
	w[0] = 0;
	w[1] = cpu_to_le32(len);
	w[2] = cpu_to_le32(hi);
	w[3] = cpu_to_le32(lo);

	/* SOF + HP=01 header block (releases scratch slot on reclaim) */
	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(blk_dma));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_SOF | CA_NI_TX_DESC1_HP0 |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN,
					 CA_NI_PON_HDR_BLK_LEN));
	q->slot[q->wptr].skb = NULL;
	q->slot[q->wptr].addr = 0;
	q->slot[q->wptr].len = 0;
	q->slot[q->wptr].pon = 2 + slot;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	/* EOF = the frame itself (skb reclaimed by the normal unmap path;
	 * pon=1 with skb set = "WAN data skb", stats counted here not eth0) */
	desc = q->desc + q->wptr * CA_NI_TX_DESC_WORDS;
	desc[0] = cpu_to_le32(lower_32_bits(daddr));
	desc[1] = cpu_to_le32(CA_NI_TX_DESC1_EOF |
			      FIELD_PREP(CA_NI_TX_DESC1_HDR_LEN, len));
	q->slot[q->wptr].skb = skb;
	q->slot[q->wptr].addr = daddr;
	q->slot[q->wptr].len = len;
	q->slot[q->wptr].pon = 1;
	q->wptr = (q->wptr + 1) % CA_NI_TX_RING_SIZE;

	q->enq++;
	tx->pon_data_enq++;
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;

	skb_tx_timestamp(skb);	/* before the doorbell (UAF lesson) */

	cortina_ni_tx_doorbell(ni, q);

	spin_unlock(&q->lock);

	mod_timer(&tx->reclaim_timer, jiffies + CA_NI_RECLAIM_INTERVAL);
	return NETDEV_TX_OK;
}
EXPORT_SYMBOL_GPL(cortina_ni_pon_data_tx);

/* ------------------------------------------------------------------ */
/* link handling + the M2b on-air proof frame                          */
/* ------------------------------------------------------------------ */

/* one gratuitous ARP so the host tcpdump sees a frame right at link-up,
 * sent through the ordinary xmit path (not a register poke) */
static void cortina_ni_tx_announce(struct work_struct *work)
{
	struct cortina_ni_tx *tx =
		container_of(work, struct cortina_ni_tx, announce_work);
	struct net_device *ndev = tx->netdev;
	struct sk_buff *skb;

	/* ★DSA: eth0 is the conduit — no IP of its own (the LAN address lives on
	 * br-lan over lan1..lan4), and a frame dev_queue_xmit'd directly on the
	 * conduit reaches cortina_ni_start_xmit UNTAGGED, so its first two bytes
	 * would be mis-read as the DSA shim and stripped, corrupting it and
	 * egressing a wrong jack.  The gratuitous ARP is also pointless here, so
	 * skip it when the switch owns the ports. */
	if (netdev_uses_dsa(ndev))
		return;

	skb = arp_create(ARPOP_REQUEST, ETH_P_ARP, 0, ndev, 0,
			 NULL, ndev->dev_addr, NULL);
	if (!skb)
		return;
	netdev_info(ndev, "sending link-up gratuitous ARP\n");
	dev_queue_xmit(skb);
}

static void cortina_ni_tx_adjust_link(struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;
	u32 clr = 0, set = 0;

	if (phydev->link) {
		/* MAC autosync tracks the PHY in HW; mirror speed/duplex in
		 * the port config bits like stock does (bit0: 1 = 10M) */
		if (phydev->speed == SPEED_10)
			set |= CA_NI_PORT_GLB_SPEED_10M;
		else
			clr |= CA_NI_PORT_GLB_SPEED_10M;
		if (phydev->duplex == DUPLEX_HALF)
			set |= CA_NI_PORT_GLB_HALF_DUPLEX;
		else
			clr |= CA_NI_PORT_GLB_HALF_DUPLEX;
		clr |= CA_NI_PORT_GLB_PWR_DWN_TX;
		ni_rmw(ni, CA_NI_PORT_GLB_CFG(CA_NI_TX_PORT), clr, set);

		/* every link-up (incl. each boot-time bounce): idempotently
		 * re-arm the RX chain + run one GPHY fault-latch check (the
		 * nondeterministic zero-RX wedge latches across a bounce) */
		cortina_ni_rx_link_up(ni);

		netif_wake_queue(ndev);
		if (!ni->tx->announced) {
			ni->tx->announced = true;
			schedule_work(&ni->tx->announce_work);
		}
	}
	phy_print_status(phydev);
	/* eth0 CPU-port carrier is forced up in cortina_ni_rx_link_up (reached via
	 * this path on a real link-up AND via the decoupled bring-up), so nothing to
	 * do here for the carrier - the phy_link_change override just prevents phylib
	 * from clearing it on the tracked PHY's link-down. */
}

/* eth0 is the CPU<->switch port, NOT a single physical link.  phylib's default
 * phy_link_change() netif_carrier_off()s eth0 whenever the one tracked PHY
 * (phy_find_first = port 0, uncabled on this rig) reports link-down, and does
 * NOT re-run adjust_link while the link stays down -- so eth0's carrier is stuck
 * off, the Linux bridge disables the eth0 port, and br-lan drops every LAN frame
 * the switch already delivered.  Override phy_link_change to run adjust_link but
 * never carrier-off the CPU port: adjust_link forces the carrier back up once the
 * datapath is armed, and per-physical-port link/forwarding is the switch's job. */
static void cortina_ni_cpu_link_change(struct phy_device *phydev, bool up)
{
	phydev->adjust_link(phydev->attached_dev);
}

/* ------------------------------------------------------------------ */
/* net_device_ops                                                      */
/* ------------------------------------------------------------------ */

static int cortina_ni_open(struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct phy_device *phydev;
	int ret;

	/*
	 * ★ 2026-08-08 AOT5221ZY (fix #30): tolerate a NULL MDIO bus.
	 * With cortina_ni.skip_mdio=1 (needed on this board) cortina_ni_mdio_init() is
	 * never run, so ni->mii is NULL and phy_find_first()/mdiobus_get_phy() NULL-derefs
	 * -> `Oops: Fatal exception` in cortina_ni_open+0x20 the moment userspace does
	 * `ip link set eth0 up` (~13.5 s).  The GPON/PON US-OMCI datapath does not need a
	 * LAN PHY, so bring the netdev up without one.
	 */
	if (!ni->mii) {
		netdev_info(ndev, "no MDIO bus (skip_mdio) - opening without a PHY\n");
		netif_carrier_on(ndev);
		/*
		 * ★★★★★ 2026-08-08 fix#52: THE RX PATH MUST STILL BE OPENED HERE.
		 * fix#30 added this no-PHY branch to stop a NULL-deref Oops, but it returned
		 * BEFORE the cortina_ni_rx_open(ni) below - and since skip_mdio=1 is required
		 * on this board, that branch is ALWAYS taken.  So on every boot since fix#30
		 * the entire RX open path silently never ran: no NAPI enable, no RX IRQ, no
		 * port RXMAC enable, and - the one that hides everything else -
		 * cortina_ni_rx_es_cpu(ni, true) was never called, leaving
		 * ES_CTRL.cpu_en = 0, i.e. THE CPU EGRESS SCHEDULER DISABLED.
		 * Live evidence: ES_CTRL reads 0x8462ff00 on every sample out to 28 s+, while
		 * the working Track A driver reads 0x8462ffff (cpu_en = 0xff).  With the CPU
		 * egress scheduler off, nothing is ever scheduled into the CPU-EPP ring, which
		 * is exactly the wptr=0 / polls=0 / frames=0 symptom.
		 * ⚠ This is fix#30's own recorded lesson repeating: "a workaround in one layer
		 * became a defect two stages later - audit every gate you add for downstream
		 * assumptions."  The workaround fixed the Oops and disabled the receiver.
		 */
		cortina_ni_rx_open(ni);
		netif_start_queue(ndev);
		return 0;
	}

	/* PHY @ addr 1 drives port 0; U-Boot left it linked - no reset */
	phydev = phy_find_first(ni->mii);
	if (!phydev) {
		netdev_err(ndev, "no PHY found on the internal bus\n");
		return -ENODEV;
	}

	ret = phy_connect_direct(ndev, phydev, cortina_ni_tx_adjust_link,
				 PHY_INTERFACE_MODE_INTERNAL);
	if (ret) {
		netdev_err(ndev, "cannot attach PHY %d\n", phydev->mdio.addr);
		return ret;
	}
	phy_set_max_speed(phydev, SPEED_1000);
	/* keep the CPU-port carrier from following the tracked PHY's link-down */
	phydev->phy_link_change = cortina_ni_cpu_link_change;
	netdev_info(ndev, "CPU-port carrier override installed (eth0 stays up once datapath armed)\n");
	ni->tx->phydev = phydev;
	ni->tx->announced = false;

	/* Disable EEE before aneg (stock keeps EEE off on the internal GPHY; this
	 * MAC has no LPI handling). */
	phy_disable_eee(phydev);

	phy_start(phydev);
	cortina_ni_rx_open(ni);	/* M2c: NAPI + RX IRQ + port RXMAC on */
	netif_start_queue(ndev);
	return 0;
}

static int cortina_ni_stop(struct net_device *ndev)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	struct cortina_ni_tx *tx = ni->tx;
	int i;

	netif_stop_queue(ndev);
	cortina_ni_rx_stop(ni);	/* M2c: RXMAC off, IRQ masked, NAPI off */
	if (tx->phydev) {
		phy_stop(tx->phydev);
		phy_disconnect(tx->phydev);
		tx->phydev = NULL;
	}
	cancel_work_sync(&tx->announce_work);
	cancel_delayed_work_sync(&tx->dma_dump_work);	/* ★fix#59 */
	timer_delete_sync(&tx->reclaim_timer);

	/* reclaim whatever completed; anything still in flight stays mapped
	 * until the engine drains it (rings are not torn down between
	 * open/stop - the HW keeps its pointers) */
	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];

		spin_lock_bh(&q->lock);
		cortina_ni_tx_reclaim_q(ni, q);
		if (q->finished != q->wptr)
			netdev_warn(ndev, "VP%u: %u frames still in flight\n",
				    q->vp,
				    (q->wptr + CA_NI_TX_RING_SIZE -
				     q->finished) % CA_NI_TX_RING_SIZE);
		spin_unlock_bh(&q->lock);
	}
	return 0;
}

/* ★ Commit a new MAC, then re-key the MAC-keyed HW tables (L2FE FDB, my-MAC
 * comparator, PP FIELD-CAM, offload router-MAC shadow) from it.  netifd
 * applies the per-board factory MAC (05_factory_mac) AFTER the boot RX init
 * and the last link-up re-arm latched dev_addr into those tables, and no
 * further link-up fires on this rig (tracked port-0 PHY uncabled) - so with
 * plain eth_mac_addr the tables stayed keyed on the boot fallback and a LAN
 * transit frame to the factory gateway MAC could never resolve to L3_LAN /
 * enter the L3FE flow engine.  The re-arm is hw_l3_fwd-gated inside
 * cortina_ni_rx_mac_rearm; gate-off = eth_mac_addr behaviour exactly. */
static int cortina_ni_set_mac_address(struct net_device *ndev, void *addr)
{
	struct cortina_ni *ni = *(struct cortina_ni **)netdev_priv(ndev);
	int ret = eth_mac_addr(ndev, addr);

	if (ret)
		return ret;
	cortina_ni_rx_mac_rearm(ni);
	return 0;
}

static const struct net_device_ops cortina_ni_netdev_ops = {
	.ndo_open		= cortina_ni_open,
	.ndo_stop		= cortina_ni_stop,
	.ndo_start_xmit		= cortina_ni_start_xmit,
	.ndo_set_mac_address	= cortina_ni_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
#if IS_ENABLED(CONFIG_CORTINA_NI_FLOWOFFLOAD)
	/* L3FE flow-engine nf_flow_table offload (cortina-ni-flowoffload.c) */
	.ndo_setup_tc		= cortina_ni_setup_tc,
#endif
};

/* ------------------------------------------------------------------ */
/* spy/dump hook: /proc/net/cortina_ni_tx (project rule: probes stay)  */
/* ------------------------------------------------------------------ */

static int cortina_ni_tx_proc_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_tx *tx = ni->tx;
	int i;

	seq_printf(m, "autosync=0x%08x\n",
		   readl(ni_base(ni) + CA_NI_HV_MAC_AUTOSYNC));
	seq_printf(m, "lan_tx: mode=%d (0=fixed port%d 1=flood 2=learn) link=0x%x hit=%llu flood=%llu dup=%llu learn=%llu flush=%llu\n",
		   lan_tx_mode, CA_NI_TX_PORT, READ_ONCE(tx->lan_link),
		   tx->lan_hit, tx->lan_flood, tx->lan_dup, tx->lan_learn,
		   tx->lan_flush);
	/*
	 * ⛔⛔ 2026-08-11 fix#80 — THE "MEASURED NEGATIVE" BELOW IS RETRACTED.
	 *
	 * It was measured at the ELNATH addresses.  On Taurus 0xa174 is
	 * NI_HV_GLB_PG_FXPT_CFG and 0xa17c is NI_HV_GLB_PC_DA2 - CONFIG registers, not the
	 * TX MIB (which lives at 0xa1b0/0xa1b4/0xa1b8; taurus_addr_override.h:1721-1726).
	 * A config register does not move when you transmit 1164 frames, and some cells
	 * "reading non-zero and staying there" is exactly what a config register does.  So
	 * the 2026-07-29 probe swept 8 ports x 5 ids across the wrong registers - and WROTE
	 * to PG_FXPT_CFG 40 times doing it.
	 *
	 * ⇒ The TX MIB was NEVER TESTED.  "It is a phantom, do not publish it" is withdrawn;
	 * re-test it at the corrected addresses before either publishing or condemning it.
	 * The rule the old note ends with is still exactly right, and now applies to the
	 * RE-test: a value that looks like a counter and never moves must not be published.
	 *
	 * ---- original 2026-07-29 note, kept for the record ----
	 * ★★ NO per-port TX PACKET counter is printed here, and that is a
	 * MEASURED negative, not an omission.
	 *
	 * NI_HV_GLB_TXMIB (ACCESS 0xa174 / DATA0 0xa17c) looked like the only
	 * per-PHYSICAL-port egress counter, with ids UC/MC/BC = 1/2/3 DERIVED
	 * from the vendor table's TxStatsFrm65to127Oct = 0xf size-bin anchor.
	 * Measured on 2026-07-29 (dev/x400axf/txmib_identify.py): across all 8
	 * ACCESS port values and ids {0,1,2,3,0xf}, every cell moved by ZERO
	 * while the driver transmitted 1164 CPU->LAN frames out the cabled port.
	 * Some cells even read non-zero and STAYED there - a value that looks
	 * like a counter and never moves is exactly the phantom witness this
	 * project keeps losing days to, so it is NOT published.  Publishing it
	 * would let the next session read tx_uc=0 on a WORKING port and chase a
	 * datapath bug that is not there.
	 *
	 * What IS trustworthy, and is printed above: the driver's own software
	 * lan_tx counters (hit/flood/dup/learn/flush) and the per-port MAC
	 * config below.  For "did THIS socket egress", use a far-end capture.
	 * To resurrect a hardware witness, re-derive the ids from stock (read
	 * the same cells on the vendor image WHILE it transmits) - that is the
	 * oracle step that was skipped.
	 */
	for (i = 0; i < CA_NI_LAN_PORT_COUNT; i++)
		seq_printf(m, "port%d glb=0x%08x txmac=0x%08x\n",
			   i,
			   readl(ni_base(ni) + CA_NI_PORT_GLB_CFG(i)),
			   readl(ni_base(ni) + CA_NI_PORT_TXMAC_CFG(i)));
	seq_printf(m, "lso_ctrl=0x%08x ss_ctrl=0x%08x es_ctrl=0x%08x\n",
		   readl(dma_base(ni) + CA_DMA_LSO_CTRL),
		   readl(dma_base(ni) + CA_DMA_SS_CTRL),
		   readl(ni_base(ni) + CA_NI_L2TM_ES_CTRL));
	seq_printf(m, "last_word1=0x%08x busy=%llu nomap=%llu linearize=%llu oversize=%llu\n",
		   tx->last_word1, tx->tx_busy, tx->drop_nomap,
		   tx->drop_linearize, tx->drop_oversize);
	seq_printf(m, "pon_tx enq=%llu data_enq=%llu fail=%llu busy_slots=0x%02x\n",
		   tx->pon_enq, tx->pon_data_enq, tx->pon_fail, tx->pon_busy);
	/* data_enq above is UPSTREAM-ONLY; print the downstream complement right
	 * next to it so it can never be read as a whole-device CPU-forward rate. */
	cortina_ni_cpu_fwd_show(m, ni);
	for (i = 0; i < CA_NI_TX_NUM_VPS; i++) {
		struct cortina_ni_txq *q = &tx->txq[i];

		seq_printf(m, "vp%u hw w=%u r=%u sw w=%u f=%u enq=%llu done=%llu\n",
			   q->vp,
			   (u32)(readl(dma_base(ni) +
				       CA_DMA_LSO_VP_TXQ_WPTR(q->vp, 0)) &
				 CA_DMA_LSO_PTR_MASK),
			   (u32)(readl(dma_base(ni) +
				       CA_DMA_LSO_VP_TXQ_RPTR(q->vp, 0)) &
				 CA_DMA_LSO_PTR_MASK),
			   q->wptr, q->finished, q->enq, q->reclaimed);
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* probe                                                               */
/* ------------------------------------------------------------------ */

static void cortina_ni_tx_set_mac(struct cortina_ni *ni,
				  struct net_device *ndev)
{
	struct device_node *child;
	int ret = -ENODEV;

	/* the "ethernet@0" child would carry a DT MAC; on this board neither
	 * U-Boot nor the stock DTB fills it (live stock reads all-zero) */
	child = of_get_child_by_name(ni->dev->of_node, "ethernet");
	if (child) {
		ret = of_get_ethdev_address(child, ndev);
		of_node_put(child);
	}
	if (ret)
		ret = of_get_ethdev_address(ni->dev->of_node, ndev);
	if (ret || !is_valid_ether_addr(ndev->dev_addr)) {
		/* LAA fallback only: the per-board factory MAC (base MAC =
		 * ELAN_MAC_ADDR from the stock ubi_Config/config_hs.xml on
		 * read-only NAND) is applied by the 05_factory_mac
		 * uci-defaults script through netifd before the interface
		 * comes up; .ndo_set_mac_address then re-keys the MAC-keyed
		 * HW tables (cortina_ni_rx_mac_rearm) - the link-up re-arms
		 * alone do NOT follow it, they all fire before netifd (the
		 * tracked port-0 PHY is uncabled on this rig). */
		eth_hw_addr_set(ndev, cortina_ni_default_mac);
		dev_warn(ni->dev, "no MAC in DT, using default %pM\n",
			 ndev->dev_addr);
	} else {
		dev_info(ni->dev, "MAC from DT: %pM\n", ndev->dev_addr);
	}
}

int cortina_ni_tx_probe(struct cortina_ni *ni)
{
	struct net_device *ndev;
	struct cortina_ni_tx *tx;
	struct cortina_ni **priv;
	int ret;

	if (!ni->win[CA_NI_WIN_DMA])
		return dev_err_probe(ni->dev, -ENODEV,
				     "DMA-LSO window not mapped, no TX\n");

	/* the engine hands 32-bit buffer addresses to the DMA */
	ret = dma_set_mask_and_coherent(ni->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(ni->dev, ret, "no 32-bit DMA\n");

	ndev = devm_alloc_etherdev(ni->dev, sizeof(struct cortina_ni *));
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, ni->dev);
	priv = netdev_priv(ndev);
	*priv = ni;

	tx = devm_kzalloc(ni->dev, sizeof(*tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;
	tx->netdev = ndev;
	ni->tx = tx;

	timer_setup(&tx->reclaim_timer, cortina_ni_tx_reclaim_timer, 0);
	INIT_WORK(&tx->announce_work, cortina_ni_tx_announce);
	INIT_DELAYED_WORK(&tx->dma_dump_work, cortina_ni_tx_dma_dump);	/* ★fix#59 */

	/* US PON control-frame (OMCI) TX scratch — non-fatal when absent,
	 * cortina_ni_pon_tx just reports -ENODEV */
	tx->pon_buf = dmam_alloc_coherent(ni->dev,
					  CA_NI_PON_TX_SLOTS *
					  CA_NI_PON_TX_SLOT_SZ,
					  &tx->pon_buf_dma, GFP_KERNEL);
	if (!tx->pon_buf)
		dev_warn(ni->dev, "no PON TX scratch - US OMCI TX disabled\n");

	ret = cortina_ni_tx_hw_init(ni);
	if (ret)
		return ret;

	/* ★fix#59: arm the sampler HERE, immediately after the rings are programmed, so
	 * the very first sample is the known-good post-probe baseline.  Everything later
	 * is compared against it — that is what turns "the engine is stalled" into "the
	 * engine stopped at T, and THIS is the register that changed".  Sampling that
	 * starts late cannot distinguish the two (the same trap that made fix#51 move the
	 * NI window dump). */
	if (dma_dump_s)
		schedule_delayed_work(&tx->dma_dump_work, 0);

	ndev->netdev_ops = &cortina_ni_netdev_ops;
	/* the STANDARD counter + register-snapshot interface
	 * (cortina-ni-ethtool.c): `ethtool -S` / `ethtool -d`.  It exists so a
	 * test can ask the same question of the vendor firmware and of ours -
	 * the /proc nodes cannot, because stock has no node of those names. */
	ndev->ethtool_ops = &cortina_ni_ethtool_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	/* ★DSA: the conduit must allow user_mtu(1500) + the tag_cortina overhead
	 * (CA_NI_DSA_TAG_LEN) or dsa_conduit_setup() fails with -EINVAL ("error
	 * setting MTU to 1502 to include DSA overhead").  The HW len field allows
	 * 2047, so headroom for DSA + a VLAN is free. */
	ndev->max_mtu = ETH_DATA_LEN + 32;
	cortina_ni_tx_set_mac(ni, ndev);

	/* ★DSA: bind this netdev's of_node to the "ethernet@0" child so the DSA
	 * CPU port's `ethernet = <&conduit>` phandle resolves to eth0 via
	 * of_find_net_device_by_node().  Must be set before register_netdev, and
	 * before cortina_ni_dsa_register() runs (else dsa_register_switch would
	 * -EPROBE_DEFER).  NULL (no such child) leaves the pre-DSA behaviour. */
	ndev->dev.of_node = of_get_child_by_name(ni->dev->of_node, "ethernet");

	ret = devm_register_netdev(ni->dev, ndev);
	if (ret)
		return dev_err_probe(ni->dev, ret, "register_netdev failed\n");

	proc_create_single_data("cortina_ni_tx", 0444, init_net.proc_net,
				cortina_ni_tx_proc_show, ni);

	WRITE_ONCE(cortina_ni_pon_tx_ni, ni);	/* open the PON TX entry */
	dev_info(ni->dev, "TX ready: %s -> LAN ports 0..%u (direct-TX, lan_tx_mode=%d)\n",
		 ndev->name, CA_NI_LAN_PORT_COUNT - 1, lan_tx_mode);
	return 0;
}
