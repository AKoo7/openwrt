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

/* DS OMCI frames that reached the CPU NIC ring (defined in rtl9602c_eth.c). */
u32 rtl9602c_eth_omci_rx_count(void);

/* US-OMCI TX-ring reclaim cursor ("dirty"): count of OMCC descriptors the HW has
 * consumed (OWN cleared). Non-zero => the OMCC TX ring is being fetched. Defined
 * in rtl9602c_eth.c; surfaced for the periodic O5 serial diagnostic. */
u32 rtl9602c_eth_omci_tx_dirty(void);

/* OLT-INDEPENDENT US-OMCI datapath self-test: inject a synthetic OMCI frame
 * through the US-OMCI TX path so RX_SID_GOOD_CNT_US[4] can be checked at O5
 * without the OLT sending DS OMCI. Defined in rtl9602c_eth.c. */
void rtl9602c_eth_omci_selftest(void);

/* Full PON US/DS-NIC + PBO bring-up (defined in gpon-rtl9602c.c). Non-__init: also
 * re-run from rtl9602c_eth_open() after the GMAC IP-block reset so the US-NIC RX
 * engine re-latches against the freshly-reset GMAC (stock order: GMAC reset -> NIC). */
void gpon_pbo_init(void);

/* Faithful port of the stock SDK rtk_all_module_init() GPON datapath bring-up,
 * run on the quiescent switch in the eth reset path (after the GMAC reset + swcore
 * resync, before the GMAC is programmed/armed). Defined in gpon-rtl9602c.c. */
void rtl9602c_full_sdk_datapath_init(void);

/* WAN data-GEM datapath. GPON_DATA_FLOW = the internal SID/flow the gpon0 WAN netdev
 * steers US frames to (tx_dst_stream_id); GPON_DATA_GEM = the OLT-assigned wire gem-port-id.
 * Shared so the eth driver's gpon0 TX descriptor uses the same SID the GPON install programs. */
#define GPON_DATA_FLOW	1
#define GPON_DATA_GEM	193u

/* The OLT also provisions a MULTICAST/broadcast GEM (OMCI ME268 inst=1 Port-ID=0x0fff,
 * paired with ME281 Multicast GEM IWTP). Broadcast DS (e.g. the DHCP OFFER the server
 * can only broadcast to an IP-less client) may ride this GEM, not the unicast data GEM.
 * Wire it to its own internal DS flow so broadcast DS de-encapsulates to the CPU/gpon0. */
#define GPON_MCAST_FLOW	2
#define GPON_MCAST_GEM	0xfffu

/* Install the WAN data GEM datapath (bridged, gem-id 193 on flow 1, riding the OMCC
 * T-CONT). Idempotent/one-shot; called from the eth OMCI GEM-Port-CTP (ME268) create
 * handler once the OMCC is up. Defined in gpon-rtl9602c.c. Returns 0 on success,
 * -EAGAIN if the OMCC isn't installed yet. */
int gpon_install_data_gem(void);

/* Emit an OMCI Attribute-Value-Change reporting the HGU WAN-egress (VEIP ME329) operational,
 * so the OLT un-gates downstream user-data forwarding. The OLT never polls the data MEs after
 * creating them; it waits for this AVC. Defined in rtl9602c_eth.c; called from the GPON FSM a
 * few seconds after O5 (config-apply done). */
void rtl9602c_eth_omci_report_oper_up(void);

#endif /* _RTL9602C_GPON_NIC_H */
