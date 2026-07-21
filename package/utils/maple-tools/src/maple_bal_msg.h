/* SPDX-License-Identifier: GPL-2.0 */
/*
 * bcmolt_msg wire header (18 bytes) — decoded from bcmolt_msg_unpack/pack
 * in trmux.ko. The body of every BAL request/reply starts with this header,
 * followed by the key + data fields.
 *
 *   bcmolt_msg_get_packed_length(): base = 0x12 (18 bytes) when err==OK
 *   bcmolt_msg_unpack() reads: obj_type(1) group(1) subgroup(2 BE) type(1)
 *   dir(1) err(2 BE) presence_mask(8 BE) err_field_idx(2 BE) = 18 bytes
 *
 * Then the object key + data follow (per the per-object codec).
 */
#ifndef MAPLE_BAL_MSG_H
#define MAPLE_BAL_MSG_H

#include <linux/types.h>

/* bcmolt_msg_type */
#define MAPLE_MSG_TYPE_GET		1
#define MAPLE_MSG_TYPE_SET		2
#define MAPLE_MSG_TYPE_CLEAR		4
#define MAPLE_MSG_TYPE_MULTI		8
#define MAPLE_MSG_TYPE_GET_MULTI	9

/* bcmolt_msg_dir */
#define MAPLE_MSG_DIR_REQUEST		0
#define MAPLE_MSG_DIR_RESPONSE		1

/* bcmolt_mgt_group */
#define MAPLE_MGT_GROUP_KEY		0
#define MAPLE_MGT_GROUP_CFG		1
#define MAPLE_MGT_GROUP_STAT		2
#define MAPLE_MGT_GROUP_STAT_CFG	3
#define MAPLE_MGT_GROUP_AUTO		4
#define MAPLE_MGT_GROUP_AUTO_CFG	5
#define MAPLE_MGT_GROUP_OPER		6

/* 18-byte bcmolt_msg wire header (all fields BIG-ENDIAN on the wire). */
struct maple_bal_msg_hdr {
	u8	obj_type;	/* +0: bcmolt_obj_id */
	u8	group;		/* +1: bcmolt_mgt_group */
	u8	subgroup[2];	/* +2: u16 BE (which sub-field) */
	u8	type;		/* +4: bcmolt_msg_type (GET/SET/...) */
	u8	dir;		/* +5: bcmolt_msg_dir (REQUEST/RESPONSE) */
	u8	err[2];		/* +6: s16 BE (bcmos_errno) */
	u8	presence_mask[8];/* +8: u64 BE (which fields are present) */
	u8	err_field_idx[2];/* +16: u16 BE */
} __packed;

#define MAPLE_BAL_MSG_HDR_SIZE	18

/* Build the 18-byte bcmolt_msg header. presence_mask=0xffff...=all fields. */
static inline void maple_bal_msg_hdr_pack(struct maple_bal_msg_hdr *h,
					  u8 obj, u8 group, u16 subgroup,
					  u8 type, u8 dir)
{
	u64 pm = ~0ULL;		/* all fields present */
	int i;

	h->obj_type = obj;
	h->group = group;
	h->subgroup[0] = subgroup >> 8;
	h->subgroup[1] = subgroup & 0xff;
	h->type = type;
	h->dir = dir;
	h->err[0] = h->err[1] = 0;
	for (i = 0; i < 8; i++)
		h->presence_mask[i] = (pm >> (8 * (7 - i))) & 0xff;
	h->err_field_idx[0] = h->err_field_idx[1] = 0;
}

#endif /* MAPLE_BAL_MSG_H */
