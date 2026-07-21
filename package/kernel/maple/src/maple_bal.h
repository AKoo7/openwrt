/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BAL messaging layer — open reimplementation of bcmolt_msg + bcmtrmux_msg_pack.
 * See docs/decomp/bal-msg-format.md for the wire format.
 *
 * Frames a BAL request (16-byte bcmtr_hdr + packed object body) and matches the
 * reply by corr_tag. The per-object body pack/unpack (the 683 bcmolt_*_pack/
 * _unpack serializers) is the remaining codec work — until those exist the body
 * is passed through opaquely.
 */
#ifndef MAPLE_BAL_H
#define MAPLE_BAL_H

#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct maple_dev;
struct sk_buff;

#define MAPLE_BAL_HDR_SIZE	0x10
#define MAPLE_BAL_DIR_REQUEST	0
#define MAPLE_BAL_DIR_RESPONSE	1
#define MAPLE_BAL_PENDING	8	/* in-flight corr_tag slots */

/* 16-byte transport header (bcmtr_hdr), BIG-ENDIAN wire form (decoded from the
 * RX unpack in trmux bcmtrmux_rx_from_line: the u16 fields are network-order).
 * In-memory the vendor struct is 24 B with holes; on the wire it is 16 B. */
struct maple_bal_hdr {
	u8	flags;		/* +0: bit7=dir, bit2=auto_proxy_reg,
				 *     bit1=auto_proxy_unreg, bit0=more_fragments */
	u8	instance;	/* +1: object-key low byte (pon_ni for GPON_ONU) */
	__be16	msg_id;		/* +2: BCMOLT_GROUP_ID_* (BE) */
	__be16	corr_tag;	/* +4: reply-match tag (BE) */
	__be16	frag_number;	/* +6 (BE) */
	__be16	msg_len;	/* +8: body_len + 16 (BE) */
	__be16	subch;		/* +10 (BE) */
	u8	reserved[3];	/* +13..14 */
	u8	device;		/* +15: bcmolt_devid */
} __packed;

/* Pending request slot (matched by corr_tag on the reply). */
struct maple_bal_pending {
	struct completion	done;
	int			err;
	void			*reply;
	size_t			reply_len;
};

/* Per-device BAL state. */
struct maple_bal {
	spinlock_t			lock;
	u16				next_corr;
	struct maple_bal_pending	slots[MAPLE_BAL_PENDING];
};

int  maple_bal_init(struct maple_dev *mdev);
void maple_bal_exit(struct maple_dev *mdev);

/* Build a BAL request and send it; block for the reply (or -EIO on timeout).
 *  msg_id   = BCMOLT_GROUP_ID_* (table-resolved; see docs/decomp/bal-msg-format.md)
 *  instance = packed BAL instance (e.g. onu_key {pon_ni, onu_id})
 *  obj_type = bcmolt_obj_id (encoded in the 18-byte bcmolt_msg body header)
 *  mgt_group= bcmolt_mgt_group (KEY/CFG/STAT/...)
 *  subgroup = which sub-field (0 for the whole group)
 *  type     = bcmolt_msg_type (GET/SET/CLEAR/...)
 *  body/len = already-packed object payload (key for GET, key+data for SET)
 * NOTE: group_id is table-driven in the vendor (bcmolt_group_id_combine); the
 * BCMOLT_GROUP_ID_* enum values are used directly here. */
int maple_bal_call(struct maple_dev *mdev, u16 msg_id, u8 instance,
		   u8 obj_type, u8 mgt_group, u16 subgroup, u8 type,
		   const void *body, size_t len,
		   void *reply, size_t *reply_len);

/* RX dispatch — called from the ring layer when a RESPONSE arrives. Parses the
 * bcmtr_hdr, matches corr_tag, wakes the caller. The 18-byte bcmolt_msg body
 * header is stripped before reply is filled. */
void maple_bal_rx(struct maple_dev *mdev, struct sk_buff *skb);

#endif /* MAPLE_BAL_H */
