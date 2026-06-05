/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared interface between the RTL9602C GPON MAC driver
 * (gpon-rtl9602c.c) and the NIC/switch driver
 * (rtl9602c_eth.c). Independent implementation from the SoC's register interface
 * and the G.984/G.988 protocols. Once the OLT assigns the OMCC GEM port and the
 * GPON driver installs the OMCC GEM datapath, it arms the NIC OMCI trap so that
 * downstream OMCI frames on the OMCC stream-id are delivered to the CPU netdev.
 */
#ifndef _RTL9602C_GPON_NIC_H
#define _RTL9602C_GPON_NIC_H

/* Arm the GMAC OMCI trap for downstream stream-id @sid (the OMCC). Provided by
 * rtl9602c_eth.c; called from the GPON Configure_Port-ID handler. */
void rtl9602c_eth_set_omci_sid(unsigned int sid);

/* Provision the ONU identity (G.984.3 ONU-SN: 4 ASCII vendor + 4 serial bytes)
 * so the eth driver's OMCI responder reports an ONU-G Vendor-ID/Serial matching
 * the PLOAM Serial_Number the OLT ranged. Provided by rtl9602c_eth.c. */
void rtl9602c_eth_set_omci_identity(const u8 *sn8);

#endif /* _RTL9602C_GPON_NIC_H */
