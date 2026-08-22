/* SPDX-License-Identifier: GPL-2.0 */
/*
 * cortina-omci-bridge.h -- kernel seam for the stock Zyxel/EcoNet omci_app daemon
 *
 * The M2 bridge presents the two planes the *unmodified* stock OMCI daemon
 * (omci_app / mosapp + libpr.so + libomci_mib.so) talks to the kernel over, so
 * it can run in a chroot and build the REAL provisioned MIB while our clean-room
 * Cortina GPON driver owns the optics/PLOAM/GEM datapath.  Contract RE'd from
 * the stock libraries and verified end-to-end by the M1 dry run (kshim.so).
 *
 * Everything here is internal to cortina_gpon.ko: the bridge (this file's .c)
 * and cortina-gpon.c are linked into the same module, so the two functions the
 * bridge reaches back into the driver for are declared here rather than exported.
 */
#ifndef CORTINA_OMCI_BRIDGE_H
#define CORTINA_OMCI_BRIDGE_H

#include <linux/types.h>

struct cortina_gpon;

/*
 * Implemented in cortina-gpon.c, called by the bridge:
 *  cg_omci_tx()               - inject a 48-byte US OMCI PDU onto the OMCC T-CONT
 *                               (the daemon's upstream replies).
 *  cg_omci_bridge_onu_state() - the live ONU state in the daemon's convention
 *                               (O5 == 5), so Plane-B getOnuState unlocks the
 *                               daemon's TX gate (appinfo[536] == 5).
 */
int cg_omci_tx(struct cortina_gpon *cg, const u8 *pdu48);

#ifdef CONFIG_CORTINA_GPON_OMCI_BRIDGE
int cg_omci_bridge_onu_state(struct cortina_gpon *cg);

int  cg_omci_bridge_init(struct cortina_gpon *cg);
void cg_omci_bridge_exit(void);

/*
 * Hand a downstream OMCI PDU to the userspace daemon.  Returns true if the PDU
 * was diverted (bridge armed via cortina_gpon.omci_userspace=1 AND a daemon is
 * registered) -- the caller then skips the in-kernel G.988 responder.  Returns
 * false when the in-kernel responder should run as before.
 */
bool cg_omci_bridge_ds(const u8 *pdu, unsigned int len);

/*
 * True when the bridge is armed AND a daemon is registered.  cortina-gpon.c
 * gates its autonomous US-OMCI emitters (the post-O5 VEIP oper-up AVC) on this
 * so exactly one OMCI stack transmits on the OMCC.
 */
bool cg_omci_bridge_armed(void);
#else
static inline int  cg_omci_bridge_init(struct cortina_gpon *cg) { return 0; }
static inline void cg_omci_bridge_exit(void) { }
static inline bool cg_omci_bridge_ds(const u8 *pdu, unsigned int len) { return false; }
static inline bool cg_omci_bridge_armed(void) { return false; }
#endif /* CONFIG_CORTINA_GPON_OMCI_BRIDGE */

#endif /* CORTINA_OMCI_BRIDGE_H */
