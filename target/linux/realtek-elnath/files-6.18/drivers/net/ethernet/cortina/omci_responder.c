// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * omci_responder.c — clean-room ONU G.988 OMCI baseline responder + ME model
 * (RTL9607F "Elnath", Stage C).  See omci_responder.h for the wire-format
 * provenance: the layout + ME attribute content mirror the responder proven
 * end-to-end (Online/normal + WAN) against the same HSGQ-G008 OLT on the
 * RTL9602C, with this board's identity (SN "XPON"+5C6CAFCB, X411AXF).
 *
 * Pure functional core: no HW I/O, no locking (the caller serializes), no
 * allocation — everything lives in the caller-provided struct omci_onu.
 */
#include <linux/crc32.h>
#include <linux/string.h>

#include "omci_responder.h"

/* Message types (G.988 Table 11.2.2-1). */
#define OMCI_MT_CREATE		0x04
#define OMCI_MT_DELETE		0x06
#define OMCI_MT_SET		0x08
#define OMCI_MT_GET		0x09
#define OMCI_MT_GET_ALL_ALARMS	0x0b
#define OMCI_MT_GET_ALL_ALRM_NX	0x0c
#define OMCI_MT_MIB_UPLOAD	0x0d
#define OMCI_MT_MIB_UPLOAD_NX	0x0e
#define OMCI_MT_MIB_RESET	0x0f
#define OMCI_MT_GET_NEXT	0x10
#define OMCI_MT_AVC		0x11	/* 17 — ONU-autonomous notification */
#define OMCI_MT_TEST		0x12
#define OMCI_MT_START_SW_DL	0x13
#define OMCI_MT_DOWNLOAD_SEC	0x14
#define OMCI_MT_END_SW_DL	0x15
#define OMCI_MT_ACTIVATE_SW	0x16
#define OMCI_MT_COMMIT_SW	0x17
#define OMCI_MT_SYNC_TIME	0x18
#define OMCI_MT_REBOOT		0x19

/* Result codes (G.988 Table 11.2.2-2). */
#define OMCI_RC_OK		0x00
#define OMCI_RC_NOT_SUPPORTED	0x02
#define OMCI_RC_UNKNOWN_ME	0x04
#define OMCI_RC_ATTR_FAILED	0x09

/* Attribute-mask bit: attr #n is bit (16-n), so bit15 = attr 1. */
#define OMCI_ATTR_BIT(n)	(1u << (16 - (n)))

/* ME class IDs presented in the MIB upload (G.988 + the HSGQ OLT's set). */
#define OMCI_ME_ONU_DATA	2
#define OMCI_ME_CARDHOLDER	5
#define OMCI_ME_CIRCUIT_PACK	6
#define OMCI_ME_SW_IMAGE	7
#define OMCI_ME_PPTP_ETH_UNI	11	/* THE HGU gate: the OLT's
					 * gpon_ont_sync_capability counts these */
#define OMCI_ME_OLT_G		131
#define OMCI_ME_ONU_G		256
#define OMCI_ME_ONU2_G		257
#define OMCI_ME_TCONT		262
#define OMCI_ME_ANI_G		263
#define OMCI_ME_UNI_G		264
#define OMCI_ME_PRIORITY_QUEUE	277
#define OMCI_ME_TRAFFIC_SCHED	278
#define OMCI_ME_VEIP		329
#define OMCI_ME_CTC_LOID_AUTH	65530	/* 0xFFFA — CTC extension the OLT audits */

static inline void omci_put_be16(u8 *p, u16 v)
{
	p[0] = (u8)(v >> 8);
	p[1] = (u8)v;
}

/*
 * MIC (bytes 44..47) = the I.363.5 / AAL5 CRC-32 over bytes 0..43 (G.984.4
 * baseline trailer): NON-reflected polynomial 0x04C11DB7 MSB-first, init
 * all-ones, final complement — the kernel's crc32_be — stored big-endian.
 * LIVE-PROVEN on this OLT: the DS OMCI frames' MIC matches ~crc32_be(~0,
 * msg, 44) and NOT the reflected zlib crc32_le (the DS MIC self-check in
 * cortina-gpon.c logs which variant each received frame carries).  Computed
 * in SOFTWARE: the GPON MAC's own OMCI CRC engine stays enabled
 * (onu_cfg.omci_crc_dis = 0, the stock value) — if the HW also inserts, it
 * writes the same correct bytes.  A zero/wrong MIC = the OLT silently drops
 * every response and loops its GET audit (proven failure class).
 */
static void omci_set_mic(u8 *msg)
{
	u32 c = ~crc32_be(~0u, msg, 44);

	msg[44] = (u8)(c >> 24);
	msg[45] = (u8)(c >> 16);
	msg[46] = (u8)(c >> 8);
	msg[47] = (u8)c;
}

/* Stamp the baseline trailer (40..43 = 00 00 00 28) + MIC.  Call LAST. */
static void omci_finalize(u8 *msg)
{
	msg[40] = 0x00;
	msg[41] = 0x00;
	msg[42] = 0x00;
	msg[43] = 0x28;
	omci_set_mic(msg);
}

/* ---- dynamic (OLT-provisioned) ME store ---- */

static struct omci_me_inst *omci_store_find(struct omci_onu *o, u16 class_id,
					    u16 inst)
{
	u16 k;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used && o->store[k].class_id == class_id &&
		    o->store[k].inst == inst)
			return &o->store[k];
	return NULL;
}

/* idx-th used entry in array order — the MIB-Upload tail rows. */
static struct omci_me_inst *omci_store_nth(struct omci_onu *o, u16 idx)
{
	u16 k, n = 0;

	for (k = 0; k < OMCI_STORE_MAX; k++)
		if (o->store[k].used) {
			if (n == idx)
				return &o->store[k];
			n++;
		}
	return NULL;
}

static void omci_store_put(struct omci_onu *o, u16 class_id, u16 inst,
			   const u8 *body, int blen)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);
	u16 k;

	if (!e)
		for (k = 0; k < OMCI_STORE_MAX; k++)
			if (!o->store[k].used) {
				e = &o->store[k];
				e->used = true;
				e->class_id = class_id;
				e->inst = inst;
				e->blen = 0;
				o->store_n++;
				break;
			}
	if (!e)		/* store full -> bounded drop (the OLT re-creates) */
		return;
	if (body && blen > 0) {
		if (blen > (int)sizeof(e->body))
			blen = sizeof(e->body);
		memcpy(e->body, body, blen);
		e->blen = (u8)blen;
	}
}

static void omci_store_del(struct omci_onu *o, u16 class_id, u16 inst)
{
	struct omci_me_inst *e = omci_store_find(o, class_id, inst);

	if (e) {
		e->used = false;
		if (o->store_n)
			o->store_n--;
	}
}

/*
 * ---- ME attribute filler ----
 * The SINGLE source of truth shared by GET and MIB-Upload-Next so both
 * byte-match.  @mask selects attributes (bit15 = attr #1); the modelled
 * subset is emitted into [v..end) (bounded), *rmask_out reports the emitted
 * bits.  EVERY attribute an ME defines must be emitted when requested
 * (rmask == mask) or the OLT re-GETs forever — the primary churn-lock class.
 * Attribute values = the live HSGQ stock set, identity = this board.
 */
static u8 omci_me_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
		       u8 *v, const u8 *end, u16 *rmask_out)
{
	u16 rmask = 0;
	bool over = false;

#define PUT(bit, n, src) do {						\
		if (mask & (bit)) {					\
			if (v + (n) <= end) {				\
				memcpy(v, (src), (n)); v += (n);	\
				rmask |= (bit);				\
			} else { over = true; }				\
		}							\
	} while (0)
#define PUT1(bit, b) do {						\
		if (mask & (bit)) {					\
			if (v + 1 <= end) { *v++ = (u8)(b); rmask |= (bit); } \
			else { over = true; }				\
		}							\
	} while (0)
#define PUT2(bit, w) do {						\
		if (mask & (bit)) {					\
			if (v + 2 <= end) {				\
				omci_put_be16(v, (w)); v += 2;		\
				rmask |= (bit);				\
			} else { over = true; }				\
		}							\
	} while (0)
#define PUT4(bit, dw) do {						\
		if (mask & (bit)) {					\
			if (v + 4 <= end) {				\
				v[0] = (u8)((dw) >> 24);		\
				v[1] = (u8)((dw) >> 16);		\
				v[2] = (u8)((dw) >> 8);			\
				v[3] = (u8)(dw); v += 4;		\
				rmask |= (bit);				\
			} else { over = true; }				\
		}							\
	} while (0)

	switch (class_id) {
	case OMCI_ME_ONU_DATA:				/* ME 2 */
		PUT1(OMCI_ATTR_BIT(1), o->mds);		/* #1 MIB-Data-Sync */
		break;
	case OMCI_ME_ONU_G: {				/* ME 256 */
		static const u8 vid[4]   = "HSGQ";	/* #1 Vendor-ID: the OLT
							 * recognizes HSGQ ONUs;
							 * "XPON" was rejected */
		static const u8 ver[14]  = "02A5B1";
		static const u8 loid[24] = "user";
		static const u8 empty12[12] = { 0 };

		/* ALL 14 attrs must be servable: a missing one -> rmask !=
		 * mask -> the OLT re-GETs forever (proven churn-lock). */
		PUT(OMCI_ATTR_BIT(1), 4, vid);
		PUT(OMCI_ATTR_BIT(2), 14, ver);
		PUT(OMCI_ATTR_BIT(3), 8, o->sn);	/* #3 Serial number */
		PUT1(OMCI_ATTR_BIT(4), 0x02);		/* #4 Traffic-mgmt opt */
		PUT1(OMCI_ATTR_BIT(5), 0x00);		/* #5 ATM CC option */
		PUT1(OMCI_ATTR_BIT(6), 0x00);		/* #6 Battery backup */
		PUT1(OMCI_ATTR_BIT(7), 0x00);		/* #7 Admin state */
		PUT1(OMCI_ATTR_BIT(8), 0x00);		/* #8 Op state */
		PUT1(OMCI_ATTR_BIT(9), 0x00);		/* #9 Survival time */
		PUT(OMCI_ATTR_BIT(10), 24, loid);	/* #10 Logical ONU ID */
		PUT(OMCI_ATTR_BIT(11), 12, empty12);	/* #11 Logical password */
		PUT1(OMCI_ATTR_BIT(12), 0x00);		/* #12 Credentials status */
		PUT2(OMCI_ATTR_BIT(13), 0x0000);	/* #13 Ext TC-layer opts */
		PUT1(OMCI_ATTR_BIT(14), 0x01);		/* #14 ONT state */
		break;
	}
	case OMCI_ME_ONU2_G: {				/* ME 257 */
		static const u8 eqid[20] = "HSGQ-X411AXF";

		PUT(OMCI_ATTR_BIT(1), 20, eqid);	/* #1 Equipment ID */
		PUT1(OMCI_ATTR_BIT(2), 0x80);		/* #2 OMCC version (G.984.4) */
		PUT2(OMCI_ATTR_BIT(3), 0x0031);		/* #3 Vendor product code */
		PUT1(OMCI_ATTR_BIT(4), 0x01);		/* #4 Security capability */
		PUT1(OMCI_ATTR_BIT(5), 0x01);		/* #5 Security mode */
		PUT2(OMCI_ATTR_BIT(6), 0x0060);		/* #6 Total priority queues */
		PUT1(OMCI_ATTR_BIT(7), 0x0c);		/* #7 Total traffic scheds */
		PUT1(OMCI_ATTR_BIT(8), 0x01);		/* #8 Mode */
		PUT2(OMCI_ATTR_BIT(9), 0x0040);		/* #9 Total GEM ports */
		PUT4(OMCI_ATTR_BIT(10), 3600);		/* #10 SysUpTime — UINT32:
							 * 2 bytes here misaligns every
							 * later attr (proven bug) */
		PUT2(OMCI_ATTR_BIT(11), 0x007f);	/* #11 Connectivity capability */
		PUT1(OMCI_ATTR_BIT(12), 0x00);		/* #12 Current conn mode */
		PUT2(OMCI_ATTR_BIT(13), 0x003b);	/* #13 QoS config flexibility */
		PUT2(OMCI_ATTR_BIT(14), 0x0001);	/* #14 Priority-queue scale */
		break;
	}
	case OMCI_ME_CARDHOLDER:			/* ME 5 (inst 0x0101) */
		PUT1(OMCI_ATTR_BIT(1), 47);		/* #1 Actual type = Eth UNI */
		PUT1(OMCI_ATTR_BIT(2), 47);		/* #2 Expected type */
		PUT1(OMCI_ATTR_BIT(3), 1);		/* #3 Expected port count */
		break;
	case OMCI_ME_CIRCUIT_PACK: {			/* ME 6 (inst 0x0101) */
		static const u8 ver[14] = "M225-260525";
		static const u8 vid[4]  = "HSGQ";

		PUT1(OMCI_ATTR_BIT(1), 47);		/* #1 Type */
		PUT1(OMCI_ATTR_BIT(2), 1);		/* #2 Number of ports */
		PUT(OMCI_ATTR_BIT(3), 8, o->sn);	/* #3 Serial number */
		PUT(OMCI_ATTR_BIT(4), 14, ver);		/* #4 Version */
		PUT(OMCI_ATTR_BIT(5), 4, vid);		/* #5 Vendor ID */
		PUT1(OMCI_ATTR_BIT(12), 8);		/* #12 Total priority queues */
		break;
	}
	case OMCI_ME_SW_IMAGE: {			/* ME 7, banks 0 (active) + 1 */
		static const u8 v0[14] = "M225-260525";
		static const u8 v1[14] = "M225-260515";
		static const u8 hash[16] = { 0 };

		PUT(OMCI_ATTR_BIT(1), 14, inst ? v1 : v0);	/* #1 Version */
		PUT1(OMCI_ATTR_BIT(2), inst ? 0 : 1);	/* #2 Is committed */
		PUT1(OMCI_ATTR_BIT(3), inst ? 0 : 1);	/* #3 Is active */
		PUT1(OMCI_ATTR_BIT(4), 1);		/* #4 Is valid */
		PUT(OMCI_ATTR_BIT(5), 16, hash);	/* #5 Image hash */
		break;
	}
	case OMCI_ME_PPTP_ETH_UNI:			/* ME 11 (inst 0x0101) — the HGU gate */
		PUT1(OMCI_ATTR_BIT(1), 47);		/* #1 Expected type */
		PUT1(OMCI_ATTR_BIT(2), 47);		/* #2 Sensed type */
		PUT1(OMCI_ATTR_BIT(3), 0);		/* #3 Auto-detect config */
		PUT1(OMCI_ATTR_BIT(4), 0);		/* #4 Eth loopback config */
		PUT1(OMCI_ATTR_BIT(5), 0);		/* #5 Admin state (unlocked) */
		PUT1(OMCI_ATTR_BIT(6), 1);		/* #6 Op state */
		PUT1(OMCI_ATTR_BIT(7), 0);		/* #7 Config ind */
		PUT2(OMCI_ATTR_BIT(8), 1518);		/* #8 Max frame size */
		PUT1(OMCI_ATTR_BIT(9), 0);		/* #9 DTE/DCE ind */
		PUT2(OMCI_ATTR_BIT(10), 0xffff);	/* #10 Pause time */
		PUT1(OMCI_ATTR_BIT(11), 2);		/* #11 Bridged/IP ind */
		PUT1(OMCI_ATTR_BIT(12), 0);		/* #12 ARC */
		PUT1(OMCI_ATTR_BIT(13), 0);		/* #13 ARC interval */
		PUT1(OMCI_ATTR_BIT(14), 0);		/* #14 PPPoE filter */
		PUT1(OMCI_ATTR_BIT(15), 0);		/* #15 Power control */
		break;
	case OMCI_ME_OLT_G:				/* ME 131 — empty; the OLT Sets it */
		break;
	case OMCI_ME_TCONT:				/* ME 262 (inst 0x8000..0x800b) */
		PUT2(OMCI_ATTR_BIT(1), inst == 0x8000 ? 0x0100 : 0x00ff); /* #1 Alloc-ID */
		PUT1(OMCI_ATTR_BIT(2), 1);		/* #2 Mode indicator */
		PUT1(OMCI_ATTR_BIT(3), 0);		/* #3 Policy */
		break;
	case OMCI_ME_ANI_G:				/* ME 263 (inst 0x8001) */
		PUT1(OMCI_ATTR_BIT(1), 1);		/* #1 SR indication */
		PUT2(OMCI_ATTR_BIT(2), 12);		/* #2 Total T-CONTs */
		PUT2(OMCI_ATTR_BIT(3), 48);		/* #3 GEM block length */
		PUT1(OMCI_ATTR_BIT(4), 0);		/* #4 Piggyback DBA */
		PUT1(OMCI_ATTR_BIT(5), 0);		/* #5 (deprecated) */
		PUT1(OMCI_ATTR_BIT(6), 5);		/* #6 SF threshold */
		PUT1(OMCI_ATTR_BIT(7), 9);		/* #7 SD threshold */
		PUT1(OMCI_ATTR_BIT(8), 0);		/* #8 ARC */
		PUT1(OMCI_ATTR_BIT(9), 0);		/* #9 ARC interval */
		PUT2(OMCI_ATTR_BIT(10), 0xeedc);	/* #10 RX optical level
							 * (static; live DDM hookup
							 * is a follow-up) */
		PUT1(OMCI_ATTR_BIT(11), 0xff);		/* #11 Lower optical thresh */
		PUT1(OMCI_ATTR_BIT(12), 0xff);		/* #12 Upper optical thresh */
		PUT2(OMCI_ATTR_BIT(13), 0);		/* #13 ONU response time */
		PUT2(OMCI_ATTR_BIT(14), 0x04d7);	/* #14 TX optical level */
		PUT1(OMCI_ATTR_BIT(15), 0x81);		/* #15 Lower TX power thresh */
		PUT1(OMCI_ATTR_BIT(16), 0x81);		/* #16 Upper TX power thresh */
		break;
	case OMCI_ME_UNI_G:				/* ME 264 (inst 0x0101) */
		PUT2(OMCI_ATTR_BIT(1), 0x0000);		/* #1 Config-option status */
		PUT1(OMCI_ATTR_BIT(2), 0);		/* #2 Admin state */
		PUT1(OMCI_ATTR_BIT(3), 1);		/* #3 Management capability */
		PUT2(OMCI_ATTR_BIT(4), 0x0000);		/* #4 Non-OMCI mgmt ID */
		PUT2(OMCI_ATTR_BIT(5), 0x0000);		/* #5 Relay-agent options */
		break;
	case OMCI_ME_PRIORITY_QUEUE: {			/* ME 277 (inst 0..7) */
		/* #6 Related port: upper 16b = port 0x0101, lower counts DOWN
		 * within the 8-queue block (queue 0 -> 7, 1 -> 6, ...). */
		u32 related = (0x0101u << 16) | (7 - (inst & 7));

		PUT1(OMCI_ATTR_BIT(1), 1);		/* #1 Queue config option */
		PUT2(OMCI_ATTR_BIT(2), 3276);		/* #2 Max queue size */
		PUT2(OMCI_ATTR_BIT(3), 3276);		/* #3 Allocated queue size */
		PUT2(OMCI_ATTR_BIT(4), 0);		/* #4 Discard reset interval */
		PUT2(OMCI_ATTR_BIT(5), 0);		/* #5 Threshold value */
		PUT4(OMCI_ATTR_BIT(6), related);	/* #6 Related port */
		PUT2(OMCI_ATTR_BIT(7), 0x0000);		/* #7 Traffic-sched pointer */
		PUT1(OMCI_ATTR_BIT(8), 1);		/* #8 Weight */
		break;
	}
	case OMCI_ME_TRAFFIC_SCHED:			/* ME 278 (inst 0x8000..0x800b) */
		PUT2(OMCI_ATTR_BIT(1), inst);		/* #1 T-CONT pointer */
		PUT2(OMCI_ATTR_BIT(2), 0x0000);		/* #2 Traffic-sched pointer */
		PUT1(OMCI_ATTR_BIT(3), 1);		/* #3 Policy */
		PUT1(OMCI_ATTR_BIT(4), 0);		/* #4 Priority/weight */
		break;
	case OMCI_ME_VEIP:				/* ME 329 (inst 0x0601) — HGU marker */
		PUT1(OMCI_ATTR_BIT(1), 0);		/* #1 Admin state */
		PUT1(OMCI_ATTR_BIT(2), 0);		/* #2 Op state */
		PUT2(OMCI_ATTR_BIT(3), 0x0000);		/* #3 Interworking-TP pointer */
		break;
	case OMCI_ME_CTC_LOID_AUTH: {			/* ME 65530 (0xFFFA) */
		static const u8 opid[4]  = "CTC";
		static const u8 loid[24] = "user";
		static const u8 empty12[12] = { 0 };

		PUT(OMCI_ATTR_BIT(1), 4, opid);		/* #1 Operation ID */
		PUT(OMCI_ATTR_BIT(2), 24, loid);	/* #2 LoID */
		PUT(OMCI_ATTR_BIT(3), 12, empty12);	/* #3 Password ("" but MUST
							 * be servable, proven) */
		PUT1(OMCI_ATTR_BIT(4), 0x01);		/* #4 Auth status = success */
		break;
	}
	case 0xfff9:	/* ME 65529 OnuCapability (vendor) — the OLT GETs both; */
	case 0xffb1:	/* ME 65457 (vendor).  UNKNOWN_ME aborts its config load,
			 * so ACK OK with no modelled attrs (stock does the same;
			 * intentionally NOT in the MIB-Upload rows). */
		break;
	default:
		*rmask_out = 0;
		return OMCI_RC_UNKNOWN_ME;
	}

#undef PUT
#undef PUT1
#undef PUT2
#undef PUT4
	*rmask_out = rmask;
	return over ? OMCI_RC_ATTR_FAILED : OMCI_RC_OK;
}

/*
 * Build the static MIB-Upload row table: every auto-instantiated hardware ME
 * the HSGQ-G008 OLT expects to read back, split so each row's attributes fit
 * the 26-byte Upload-Next value area.  The OLT counts the ME 11 instances to
 * classify the ONU as HGU; an empty upload loops its "ONU config load fail".
 */
static void omci_build_mib(struct omci_onu *o)
{
	u16 n = 0;
	u16 i;

#define ROW(c, ins, m) do {						\
		if (n < OMCI_MIB_ROWS_MAX) {				\
			o->rows[n].class_id = (c);			\
			o->rows[n].inst = (ins);			\
			o->rows[n].mask = (m);				\
			n++;						\
		}							\
	} while (0)

	ROW(OMCI_ME_ONU_DATA, 0x0000, OMCI_ATTR_BIT(1));

	/* ME 256 ONU-G: 14 attrs split by the 26-byte cap:
	 * A = vid(4)+ver(14)+sn(8) = 26, B = #4..#9 = 6x1, C = LoID(24),
	 * D = #11(12)+#12(1)+#13(2)+#14(1) = 16. */
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
				   OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7) |
				   OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(10));
	ROW(OMCI_ME_ONU_G, 0x0000, OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12) |
				   OMCI_ATTR_BIT(13) | OMCI_ATTR_BIT(14));

	/* ME 257 ONT2-G: A = EquipmentID(20), B = all scalars (21B). */
	ROW(OMCI_ME_ONU2_G, 0x0000, OMCI_ATTR_BIT(1));
	ROW(OMCI_ME_ONU2_G, 0x0000, OMCI_ATTR_BIT(2) | OMCI_ATTR_BIT(3) |
				    OMCI_ATTR_BIT(4) | OMCI_ATTR_BIT(5) |
				    OMCI_ATTR_BIT(6) | OMCI_ATTR_BIT(7) |
				    OMCI_ATTR_BIT(8) | OMCI_ATTR_BIT(9) |
				    OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
				    OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13) |
				    OMCI_ATTR_BIT(14));

	ROW(OMCI_ME_CARDHOLDER, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					OMCI_ATTR_BIT(3));

	/* ME 6 Circuit-Pack: A = #1..#4 = 24B, B = #5(4)+#12(1) = 5B. */
	ROW(OMCI_ME_CIRCUIT_PACK, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_CIRCUIT_PACK, 0x0101, OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(12));

	/* ME 7 Software-Image x2 banks: A = ver+committed+active+valid = 17B,
	 * B = hash(16). */
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				      OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_SW_IMAGE, 0x0000, OMCI_ATTR_BIT(5));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				      OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_SW_IMAGE, 0x0001, OMCI_ATTR_BIT(5));

	/* ME 11 PPTP Ethernet UNI: #1..#15 = 17B, one row.  THE HGU GATE. */
	ROW(OMCI_ME_PPTP_ETH_UNI, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
					  OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
					  OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
					  OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) |
					  OMCI_ATTR_BIT(9) | OMCI_ATTR_BIT(10) |
					  OMCI_ATTR_BIT(11) | OMCI_ATTR_BIT(12) |
					  OMCI_ATTR_BIT(13) | OMCI_ATTR_BIT(14) |
					  OMCI_ATTR_BIT(15));

	ROW(OMCI_ME_OLT_G, 0x0000, 0x0000);

	/* ME 263 ANI-G: A = #1..#9 = 11B, B = #10..#16 = 10B. */
	ROW(OMCI_ME_ANI_G, 0x8001, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
				   OMCI_ATTR_BIT(5) | OMCI_ATTR_BIT(6) |
				   OMCI_ATTR_BIT(7) | OMCI_ATTR_BIT(8) |
				   OMCI_ATTR_BIT(9));
	ROW(OMCI_ME_ANI_G, 0x8001, OMCI_ATTR_BIT(10) | OMCI_ATTR_BIT(11) |
				   OMCI_ATTR_BIT(12) | OMCI_ATTR_BIT(13) |
				   OMCI_ATTR_BIT(14) | OMCI_ATTR_BIT(15) |
				   OMCI_ATTR_BIT(16));

	/* ME 262 T-CONT (inst 0x8000..0x800b): 4B each. */
	for (i = 0; i < 12; i++)
		ROW(OMCI_ME_TCONT, 0x8000 + i, OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3));

	ROW(OMCI_ME_UNI_G, 0x0101, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				   OMCI_ATTR_BIT(3) | OMCI_ATTR_BIT(4) |
				   OMCI_ATTR_BIT(5));

	/* ME 277 Priority-Queue: only the single UNI's 8 queues.  The full
	 * 96-row stock set made the upload so long the OLT's auth timer
	 * deactivated us mid-config (proven on the 9602C). */
	for (i = 0; i < 8; i++)
		ROW(OMCI_ME_PRIORITY_QUEUE, i, OMCI_ATTR_BIT(1) |
					       OMCI_ATTR_BIT(2) |
					       OMCI_ATTR_BIT(3) |
					       OMCI_ATTR_BIT(4) |
					       OMCI_ATTR_BIT(5) |
					       OMCI_ATTR_BIT(6) |
					       OMCI_ATTR_BIT(7) |
					       OMCI_ATTR_BIT(8));

	/* ME 278 Traffic-Scheduler (inst 0x8000..0x800b): 6B each. */
	for (i = 0; i < 12; i++)
		ROW(OMCI_ME_TRAFFIC_SCHED, 0x8000 + i, OMCI_ATTR_BIT(1) |
						       OMCI_ATTR_BIT(2) |
						       OMCI_ATTR_BIT(3) |
						       OMCI_ATTR_BIT(4));

	ROW(OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(2) |
				  OMCI_ATTR_BIT(3));

	/* ME 65530 CTC LoID auth: 29B total -> A = #1(4)+#4(1), B = #2(24). */
	ROW(OMCI_ME_CTC_LOID_AUTH, 0x0000, OMCI_ATTR_BIT(1) | OMCI_ATTR_BIT(4));
	ROW(OMCI_ME_CTC_LOID_AUTH, 0x0000, OMCI_ATTR_BIT(2));

#undef ROW
	o->nrows = n;
}

void omci_onu_init(struct omci_onu *o, const u8 sn[8], u8 mds_seed)
{
	memset(o, 0, sizeof(*o));
	memcpy(o->sn, sn, 8);
	o->mds = mds_seed;
	omci_build_mib(o);
}

/*
 * GET-response filler: result(8) + attr-mask(9,10) + values(11..39).  Falls
 * back to the dynamic store for OLT-created MEs (a GET of a provisioned ME
 * must return OK, not UNKNOWN_ME which aborts the OLT's config). */
static u8 omci_get_fill(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
			u8 *resp)
{
	u16 rmask = 0;
	u8 rc = omci_me_fill(o, class_id, inst, mask, resp + 11, resp + 40,
			     &rmask);

	if (rc == OMCI_RC_UNKNOWN_ME) {
		struct omci_me_inst *e = omci_store_find(o, class_id, inst);

		if (e) {
			int n = e->blen > 26 ? 26 : e->blen;

			memcpy(resp + 11, e->body, n);
			rmask = mask;	/* echo requested mask; best-effort */
			rc = OMCI_RC_OK;
		}
	}
	omci_put_be16(resp + 9, rmask);
	return rc;
}

int omci_onu_input(struct omci_onu *o, const u8 *msg, unsigned int len, u8 *resp)
{
	u16 class_id, inst;
	u8 mt, devid;

	if (len < 8)
		return 0;
	devid = msg[3];
	mt = msg[2] & 0x1f;
	class_id = ((u16)msg[4] << 8) | msg[5];
	inst = ((u16)msg[6] << 8) | msg[7];

	if (devid != 0x0a)	/* only baseline modelled */
		return 0;

	memset(resp, 0, OMCI_LEN);
	resp[0] = msg[0];			/* TID echo */
	resp[1] = msg[1];
	resp[2] = (msg[2] & 0x1f) | 0x20;	/* clear AR, set AK */
	resp[3] = 0x0a;
	resp[4] = msg[4];			/* class echo */
	resp[5] = msg[5];
	resp[6] = msg[6];			/* instance echo */
	resp[7] = msg[7];

	switch (mt) {
	case OMCI_MT_MIB_RESET:
		/* On-wire MIB-Reset: zero MIB-Data-Sync (the OLT recounts its
		 * lsync from 0; keeping a seed here = permanent mismatch ->
		 * Deactivate loop, proven) + drop the provisioned store. */
		o->mds = 0;
		memset(o->store, 0, sizeof(o->store));
		o->store_n = 0;
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_MIB_UPLOAD:
		/* Row count at contents[8..9], NO result byte (a result byte
		 * here made the OLT read count=0 and never walk, proven). */
		omci_put_be16(resp + 8, o->nrows + o->store_n);
		break;
	case OMCI_MT_GET:
		if (len < 10)	/* mask missing: a shorter GET would read
				 * stale bytes into the reply (info leak) */
			return 0;
		resp[8] = omci_get_fill(o, class_id, inst,
					((u16)msg[8] << 8) | msg[9], resp);
		break;
	case OMCI_MT_SET:
	case OMCI_MT_CREATE:
	case OMCI_MT_DELETE:
		if (mt == OMCI_MT_CREATE)
			omci_store_put(o, class_id, inst, msg + 8,
				       (len > 8) ? (int)(len - 8) : 0);
		else if (mt == OMCI_MT_DELETE)
			omci_store_del(o, class_id, inst);
		/* MIB-Data-Sync: +1 per applied config message (not per
		 * attribute), wrap 255 -> 1 (0 = just-reset).  An OLT Set of
		 * ME2 attr-1 is an explicit resync write: take its byte
		 * first, then this Set's own +1 still applies. */
		if (mt == OMCI_MT_SET && class_id == OMCI_ME_ONU_DATA &&
		    len >= 11 && ((((u16)msg[8] << 8) | msg[9]) & 0x8000))
			o->mds = msg[10];
		if (++o->mds == 0)
			o->mds = 1;
		resp[8] = OMCI_RC_OK;
		break;
	case OMCI_MT_GET_ALL_ALARMS:
		omci_put_be16(resp + 9, 0x0000);	/* no active alarms */
		break;
	case OMCI_MT_MIB_UPLOAD_NX: {
		/* Request seq at msg[8..9]; reply = class[8..9] + inst[10..11]
		 * + attr-mask[12..13] + values[14..39], NO result byte. */
		u16 seq;
		u16 wmask = 0;

		if (len < 10)
			return 0;
		seq = ((u16)msg[8] << 8) | msg[9];
		if (seq < o->nrows) {
			const struct omci_mib_row *r = &o->rows[seq];

			omci_put_be16(resp + 8, r->class_id);
			omci_put_be16(resp + 10, r->inst);
			omci_me_fill(o, r->class_id, r->inst, r->mask,
				     resp + 14, resp + 40, &wmask);
			omci_put_be16(resp + 12, wmask);
		} else if (seq < o->nrows + o->store_n) {
			/* provisioned MEs after the static rows: present-only
			 * (mask 0); values are served via GET. */
			const struct omci_me_inst *e =
				omci_store_nth(o, seq - o->nrows);

			if (e) {
				omci_put_be16(resp + 8, e->class_id);
				omci_put_be16(resp + 10, e->inst);
			}
		}
		/* out-of-range seq -> all-zero row, still well-formed */
		break;
	}
	case OMCI_MT_GET_ALL_ALRM_NX:
	case OMCI_MT_GET_NEXT:
		/* no result byte; all-zero contents = empty, well-formed */
		break;
	case OMCI_MT_TEST:
	case OMCI_MT_SYNC_TIME:
	case OMCI_MT_REBOOT:		/* ACK, do NOT actually reboot */
	case OMCI_MT_START_SW_DL:
	case OMCI_MT_DOWNLOAD_SEC:
	case OMCI_MT_END_SW_DL:
	case OMCI_MT_ACTIVATE_SW:
	case OMCI_MT_COMMIT_SW:
		/* not performed (no SW image to flash), but must ACK OK so
		 * the OLT's provisioning FSM completes */
		resp[8] = OMCI_RC_OK;
		break;
	default:
		o->unhandled++;
		resp[8] = OMCI_RC_NOT_SUPPORTED;
		break;
	}

	omci_finalize(resp);
	return OMCI_LEN;
}

/*
 * Autonomous AVC (MT 0x11, TID 0): report that (class, inst)'s attributes in
 * @mask changed to @val.  The OLT never GETs the data-plane MEs after
 * creating them — its per-class AVC handlers gate DOWNSTREAM user-data
 * forwarding on the ONU's operational report. */
static void omci_emit_avc(struct omci_onu *o, u16 class_id, u16 inst, u16 mask,
			  const u8 *val, unsigned int vlen, u8 *out)
{
	memset(out, 0, OMCI_LEN);
	out[2] = OMCI_MT_AVC;
	out[3] = 0x0a;
	omci_put_be16(out + 4, class_id);
	omci_put_be16(out + 6, inst);
	omci_put_be16(out + 8, mask);
	if (val && vlen) {
		if (vlen > 30)
			vlen = 30;
		memcpy(out + 10, val, vlen);
	}
	omci_finalize(out);
	o->avc_count++;
}

int omci_onu_emit_veip_up_avc(struct omci_onu *o, u8 *out)
{
	/* VEIP inst 0x0601 attr #2 (operational state) mask 0x4000,
	 * value 0 = enabled (G.988). */
	static const u8 up = 0x00;

	omci_emit_avc(o, OMCI_ME_VEIP, 0x0601, OMCI_ATTR_BIT(2), &up, 1, out);
	o->avc_veip_up_sent = true;
	return OMCI_LEN;
}
