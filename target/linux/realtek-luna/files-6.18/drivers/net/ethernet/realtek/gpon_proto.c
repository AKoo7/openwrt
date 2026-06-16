// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon_proto.c — portable, endianness-agnostic, HW-free GPON PLOAM protocol
 * core. Extracted verbatim from gpon-rtl9602c.c so the SAME logic the board
 * runs is compiled + fuzzed on the x86 host (libFuzzer+ASan+UBSan), where
 * KASAN is unavailable on MIPS. All wire parsing is explicit byte math, so it
 * behaves identically on big-endian MIPS and little-endian ARM. The HW side
 * effects are the gpon_* ops in gpon_proto.h (driver implements; fuzzer stubs).
 */
#include "gpon_proto.h"

void gpon_fsm_handle(const u8 *m)
{
	u8 onu_id = m[0], type = m[1];
	const u8 *d = &m[2];		/* 10 data octets */

	/* Surface any DS PLOAM that is not the repetitive broadcast acquisition
	 * traffic (Upstream_Overhead 0x01 / profile 0x14) — e.g. Assign_ONU-ID or
	 * anything addressed to us — so activation progress is visible. */
	if (type != PLM_DS_UPSTREAM_OVERHEAD && type != PLM_DS_EXT_BURST_LENGTH)
		pr_info_ratelimited("rtl9602c-gpon: DS PLOAM onu_id=0x%02x type=0x%02x d=%*phN\n",
				    onu_id, type, 8, d);

	switch (type) {
	case PLM_DS_UPSTREAM_OVERHEAD:
		/* OLT is acquiring ONUs (it broadcasts this continuously). On the
		 * O1/O2 -> O3 edge, program the burst overhead + pre-ranging EqD the
		 * OLT dictates BEFORE the first SN, then move to O3; the SN is (re)sent,
		 * throttled, from the poll loop so we don't flood the US PLOAM queue.
		 * G.984.3 Upstream_Overhead payload: d[0]=guard bits, d[3]=preamble
		 * (type3) pattern, d[4..6]=delimiter, d[7] bit5=pre-EqD present with
		 * value d[8:9] (x32x8 bits). */
		if (gpon_fsm_state < 3) {
			u32 pre_eqd = ((d[7] >> 5) & 1) ?
				(((u32)d[8] << 8) | d[9]) * 32 * 8 : 0;

			gpon_boh_guard    = d[0];
			gpon_boh_ptn      = d[3];
			gpon_boh_delim[0] = d[4];
			gpon_boh_delim[1] = d[5];
			gpon_boh_delim[2] = d[6];
			gpon_apply_boh(false);	/* folds in any prior 0x14 t3pre */
			gpon_set_eqd(pre_eqd);
			gpon_fsm_set_state(2);
			gpon_fsm_set_state(3);
			gpon_send_sn();		/* first SN immediately */
		}
		break;
	case PLM_DS_ASSIGN_ONU_ID:
		/* d[0] = assigned ONU-ID, d[1..8] = serial number to match */
		if (!memcmp(&d[1], gpon_sn_bytes, 8)) {
			gpon_fsm_onu_id = d[0];
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, gpon_fsm_onu_id);
			gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, gpon_fsm_onu_id);
			/* Bind the DEFAULT/management Alloc-ID (= ONU-ID, per G.984.3) to the
			 * OMCC's T-CONT 16 (NOT T-CONT 0). Pre-config the OLT grants only this
			 * default alloc for PLOAM/OMCI, and per the vendor it owns the OMCC
			 * T-CONT, so its grants must serve the OMCC US queue (phys qid 64). With
			 * it on an empty T-CONT 0 the OMCC upstream is never drained, the OLT
			 * sees the OMCC half-dead and WITHHOLDS DS OMCI. (The separate data
			 * Alloc-ID 0x400 is bound to T-CONT 8 on Assign_Alloc-ID.) */
			gpon_install_tcont(GPON_OMCC_TCONT, gpon_fsm_onu_id);
			pr_info("rtl9602c-gpon: OLT assigned ONU-ID %u\n",
				gpon_fsm_onu_id);
			gpon_fsm_set_state(4);
		}
		break;
	case PLM_DS_RANGING_TIME:
		/* Accept only the main-path EqD (d[0] bit0 == 0); protect-path EqD is
		 * not configurable. EqD is d[1..4] big-endian and is folded with
		 * MIN_DELAY1 by gpon_set_eqd (same as the pre-ranging path). */
		if (onu_id == gpon_fsm_onu_id && !(d[0] & 0x01)) {
			u32 eqd = ((u32)d[1] << 24) | ((u32)d[2] << 16) |
				  ((u32)d[3] << 8) | d[4];

			gpon_set_eqd(eqd);
			gpon_apply_boh(true);	/* switch to the ranged operation burst */
			pr_info("rtl9602c-gpon: Ranging_Time EqD=0x%x -> O5\n", eqd);
			gpon_fsm_set_state(5);
		}
		break;
	case PLM_DS_DISABLE_SN:
		/* Disable_serial_number (0x06): d[0]=disable/ENABLE code. Only a real DISABLE
		 * (0xFF for our SN, or 0x0F disable-all) resets; ENABLE (0x00 for our SN) must
		 * NOT reset or we fight the OLT's re-enable. Matches stock gpon_ploam.c:561. */
		if (!((d[0] == 0xff && !memcmp(&d[1], gpon_sn_bytes, 8)) || d[0] == 0x0f))
			break;
		fallthrough;
	case PLM_DS_DEACTIVATE_ONU:
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			gpon_fsm_onu_id = 0xff;
			/* FULL reset to O1 — mirror the SN-reprovision path (≈line 3068).
			 * Previously only the SW onu-id/key were cleared, leaving the
			 * one-shot OMCC/T-CONT install guards TRUE and the HW ONU-ID regs
			 * stale. Consequence under OLT deactivate-churn: the 2nd+ re-range
			 * SKIPS gpon_install_omcc()/gpon_install_tcont() (guard still set),
			 * so the ONU never rebuilds its OMCI datapath, and the freshly
			 * rebooted OLT keeps directed-deactivating the stale HW ONU-ID. */
			gpon_omcc_installed = false;
			gpon_tcont_installed = false;
			gpon_aes_switch_time = 0xffffffff;	/* re-arm 0x13 on next activation */
			gpon_key_staged = false;
			gpon_field(GPON_GTC_DS_ONU_STATUS, 15, 8, 0xff);
			gpon_field(GPON_GTC_US_ONU_ID, 15, 8, 0xff);
			gpon_fsm_set_state(1);
		}
		break;
	case PLM_DS_EXT_BURST_LENGTH:
		/* Extended_Burst_Length (G.984.3): d[0] = Type-3 preamble length
		 * for the PRE-RANGED (SN/ranging) burst, d[1] = for the ranged
		 * (operation) burst. The OLT broadcasts this during acquisition;
		 * honoring d[0] lengthens our SN-burst preamble (BOH_LENGTH) so
		 * the OLT's burst receiver can lock and range us. Re-arm while
		 * still broadcast-addressed/pre-ranging. The Extended_Burst_Length
		 * PLOAM is acted on at O3. */
		gpon_boh_t3ranged = d[1];	/* applied at the O5 transition */
		if (gpon_fsm_onu_id == 0xff && gpon_boh_t3pre != d[0]) {
			gpon_boh_t3pre = d[0];
			gpon_apply_boh(false);
			pr_info("rtl9602c-gpon: Extended_Burst_Length type3_preranged=%u ranged=%u\n",
				gpon_boh_t3pre, gpon_boh_t3ranged);
		}
		break;
	case PLM_DS_CONFIG_PORT:
		/* Configure_Port-ID (0x0e): the OLT assigns the OMCC GEM port for OMCI
		 * (d[0] bit0 = enable, gem = (d[1]<<4)|(d[2]>>4)). Install the OMCC GEM
		 * datapath (one-shot) so DS OMCI reaches the CPU, THEN Acknowledge (so
		 * the ONU is RX-ready before the OLT proceeds). */
		if (onu_id == gpon_fsm_onu_id) {
			u16 gem = ((u16)d[1] << 4) | (d[2] >> 4);

			if ((d[0] & 0x1) && !gpon_omcc_installed) {
				if (!gpon_install_omcc(gem))
					gpon_omcc_installed = true;
			}
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
					    type, 8, d);
		}
		break;
	case PLM_DS_ASSIGN_ALLOC_ID:
		/* Assign_Alloc-ID (0x0a): bind the OLT's separate DATA Alloc-ID
		 * (alloc=(d[0]<<4)|(d[1]>>4)) to a DATA T-CONT (8), NOT the OMCC T-CONT 16
		 * (which now belongs to the management Alloc-ID = ONU-ID, set at Assign_ONU-ID,
		 * so two allocs do not collide on T-CONT 16). (d[2]: 0x01=allocate,
		 * 0xff=deallocate.) Then Acknowledge. */
		if (onu_id == gpon_fsm_onu_id) {
			u16 alloc = ((u16)d[0] << 4) | (d[1] >> 4);

			/* THE alloc the OLT assigns here (e.g. 0x400) is the OMCC's upstream
			 * Alloc-ID, NOT a data Alloc-ID: bind it to the OMCC T-CONT 16 (overwriting
			 * the placeholder ONU-ID bind from Assign_ONU-ID). The OLT grants ONLY this
			 * Alloc-ID pre-OMCI; binding it to a separate T-CONT 8 left the OMCC T-CONT 16
			 * (on the ungranted ONU-ID alloc) SILENT — TCONT_IDLE[16]=0 — so the OLT never
			 * saw the OMCC upstream operate, kept re-Configure_Port-ID and withheld OMCI.
			 * On stock this same alloc is bound to T-CONT 16 and its OMCC emits (~10M). */
			if (d[2] == 0x01 && !gpon_tcont_installed) {
				if (!gpon_install_tcont(GPON_OMCC_TCONT, alloc))
					gpon_tcont_installed = true;
			}
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
					    type, 8, d);
		}
		break;
	case PLM_DS_REQUEST_PASSWORD:
		/* Request_Password (0x09): reply with the (empty) US Password (0x02). Ground
		 * truth: the OLT stalls at O5 spamming 0x09 and deactivates with LOAi without
		 * it. OLT is SN-auth so the value is ignored, but the message is required. */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff)
			gpon_send_password();
		break;
	case PLM_DS_REQUEST_KEY:
		/* OLT requests a downstream AES key; reply with Encryption_Key (US 0x05). */
		if (onu_id == gpon_fsm_onu_id)
			gpon_send_key();
		break;
	case PLM_DS_KEY_SWITCH:
		/* Key_Switching_Time (0x13): the OLT supplies the 30-bit superframe count at
		 * which the HW promotes the staged AES key (loaded by gpon_send_key) to active.
		 * Arm the HW comparator (write SWITCH_SUPERFRAME) and Acknowledge. The OLT will
		 * not advance to OMCI until this key handshake completes, so a missing 0x13
		 * handler leaves it re-cycling Request_Key/Configure_Port-ID forever. De-dup the
		 * register write per superframe (the OLT re-sends 0x13 every cycle). */
		if (onu_id == gpon_fsm_onu_id || onu_id == 0xff) {
			u32 fc = ((u32)(d[0] & 0x3f) << 24) | ((u32)d[1] << 16) |
				 ((u32)d[2] << 8) | d[3];

			/* Only arm the HW key-switch once we have actually loaded a key into
			 * the staged bank (via Request_Key); arming a switch to an empty/stale
			 * staged bank would promote a garbage key and corrupt AES. ACK either way
			 * so the OLT sees the message handled. */
			if (gpon_key_staged && fc != gpon_aes_switch_time) {
				gpon_aes_switch_time = fc;
				gpon_wr(0x3014, fc);	/* AES_KEY_SWITCH_TIME[29:0] */
				pr_info("rtl9602c-gpon: Key_Switching_Time -> arm switch @superframe %u\n",
					fc);
			}
			gpon_send_ack(m);
		}
		break;
	case PLM_DS_ENCRYPT_PORT:
		/* Encrypted_Port-ID (0x08): G.984.3 requires a US Acknowledge; the OLT
		 * arms a ~43s timer and Deactivates us if none arrives. */
		if (onu_id == gpon_fsm_onu_id) {
			gpon_send_ack(m);
			pr_info_ratelimited("rtl9602c-gpon: ACK type=0x%02x d=%*phN\n",
					    type, 8, d);
		}
		break;
	}
}

/* --- US PLOAM response builders: read ds/state, build the 12-byte TX buffer.
 * The actual HW send is the stubbable gpon_send_cpu_ploam() op. --- */
void gpon_send_sn(void)
{
	u8 m[12];

	m[0] = 0xff;			/* ONU-ID (unassigned)            */
	m[1] = PLM_US_SERIAL_NUMBER;	/* 0x01                           */
	memcpy(&m[2], gpon_sn_bytes, 8);/* ONU-SN: ID(4) + serial(4)      */
	m[10] = 0x00;			/* random delay (HW may fill)     */
	m[11] = 0x04;			/* G-bit set, power level 0       */

	gpon_send_cpu_ploam(PLM_US_QUEUE_SN, m);
	gpon_fsm_sn_tx++;
}

/* US Password (0x02) reply to Request_Password (0x09): empty (zero) password 3x on
 * the urgent queue. OLT is SN-auth so the value is ignored, but the message is
 * required or the OLT stalls at O5 and deactivates with LOAi (Loss Of Acknowledge). */
void gpon_send_password(void)
{
	u8 p[12] = { 0 };
	int i;

	p[0] = gpon_fsm_onu_id;
	p[1] = PLM_US_PASSWORD;		/* 0x02 */
	for (i = 0; i < 3; i++)
		gpon_send_cpu_ploam(PLM_US_QUEUE_URG, p);
}

void gpon_send_ack(const u8 *ds)
{
	u8 a[12] = { 0 };

	a[0] = gpon_fsm_onu_id;		/* our assigned ONU-ID (HW may override) */
	a[1] = PLM_US_ACKNOWLEDGE;	/* 0x09 */
	a[2] = ds[1];			/* acknowledged message type */
	a[3] = ds[0];			/* acknowledged message ONU-ID */
	a[4] = ds[1];			/* acknowledged message type (echo) */
	memcpy(&a[5], &ds[2], 7);	/* first 7 payload octets */
	gpon_send_cpu_ploam(PLM_US_QUEUE_URG, a);
}
