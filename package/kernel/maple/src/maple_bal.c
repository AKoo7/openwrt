// SPDX-License-Identifier: GPL-2.0
/*
 * BAL messaging — build/send a bcmolt_msg request, match the reply by corr_tag.
 * See maple_bal.h + docs/decomp/bal-msg-format.md.
 *
 * NOTE: this frames/serializes the 16-byte transport header and routes replies;
 * the per-object body pack/unpack (the 683 bcmolt_*_pack/_unpack codec) is the
 * remaining work — until then bodies are carried opaquely.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "maple_bal.h"
#include "maple_bal_msg.h"
#include "maple_pci.h"
#include "maple_ring.h"

static u16 maple_bal_alloc_corr(struct maple_bal *bal, struct maple_bal_pending **out)
{
	u16 tag;

	spin_lock(&bal->lock);
	tag = bal->next_corr;
	bal->next_corr = (bal->next_corr + 1) % MAPLE_BAL_PENDING;
	spin_unlock(&bal->lock);

	*out = &bal->slots[tag % MAPLE_BAL_PENDING];
	reinit_completion(&(*out)->done);
	(*out)->err = 0;
	(*out)->reply = NULL;
	(*out)->reply_len = 0;
	return tag;
}

int maple_bal_init(struct maple_dev *mdev)
{
	int i;

	spin_lock_init(&mdev->bal.lock);
	mdev->bal.next_corr = 0;
	for (i = 0; i < MAPLE_BAL_PENDING; i++)
		init_completion(&mdev->bal.slots[i].done);
	return 0;
}

void maple_bal_exit(struct maple_dev *mdev)
{
}

/* BCMOLT_GROUP_ID_* values are table-resolved in the vendor
 * (bcmolt_group_id_combine); callers pass them directly. */

int maple_bal_call(struct maple_dev *mdev, u16 msg_id, u8 instance,
		   u8 obj_type, u8 mgt_group, u16 subgroup, u8 type,
		   const void *body, size_t len,
		   void *reply, size_t *reply_len)
{
	struct maple_bal_pending *p;
	struct maple_bal_hdr *hdr;
	struct maple_bal_msg_hdr *mhdr;
	struct maple_bal_msg_hdr reply_mhdr;
	struct sk_buff *skb;
	u16 corr;
	size_t total = MAPLE_BAL_HDR_SIZE + MAPLE_BAL_MSG_HDR_SIZE + len;
	size_t strip = MAPLE_BAL_MSG_HDR_SIZE;

	if (len > 0x4000)
		return -EMSGSIZE;
	skb = alloc_skb(total, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hdr = skb_put(skb, MAPLE_BAL_HDR_SIZE);
	memset(hdr, 0, sizeof(*hdr));
	/* Wire header is BIG-ENDIAN (decoded from bcmtrmux_rx_from_line).
	 * flags byte: bit7=dir (0=request,1=response), bit0=more_fragments. */
	hdr->flags    = MAPLE_BAL_DIR_REQUEST << 7;
	hdr->instance = instance;
	hdr->msg_id   = cpu_to_be16(msg_id);
	hdr->msg_len  = cpu_to_be16(total);
	hdr->device   = 0;

	/* 18-byte bcmolt_msg body header (decoded from bcmolt_msg_pack/_unpack in
	 * trmux.ko): obj_type(1) group(1) subgroup(2 BE) type(1) dir(1) err(2 BE)
	 * presence_mask(8 BE) err_field_idx(2 BE). */
	mhdr = skb_put(skb, MAPLE_BAL_MSG_HDR_SIZE);
	maple_bal_msg_hdr_pack(mhdr, obj_type, mgt_group, subgroup,
			       type, MAPLE_MSG_DIR_REQUEST);

	corr = maple_bal_alloc_corr(&mdev->bal, &p);
	hdr->corr_tag = cpu_to_be16(corr);
	if (body && len)
		skb_put_data(skb, body, len);

	/* Stash a place to receive the reply msg header so we can strip it. */
	p->reply = reply;
	p->reply_len = reply_len ? *reply_len : 0;
	(void)reply_mhdr; (void)strip;

	/* TODO: stamp channel routing into hdr->channel_routing (bcmtrmux_send_to_line). */
	if (maple_ring_tx(&mdev->ring, skb)) {
		dev_kfree_skb_any(skb);
		return -EIO;
	}

	if (!wait_for_completion_timeout(&p->done, msecs_to_jiffies(2000)))
		return -ETIMEDOUT;
	if (reply_len)
		*reply_len = p->reply_len;
	return p->err;
}

void maple_bal_rx(struct maple_dev *mdev, struct sk_buff *skb)
{
	struct maple_bal_hdr *hdr;
	struct maple_bal_pending *p;
	u16 corr;
	size_t body, payload;

	if (!skb || skb->len < MAPLE_BAL_HDR_SIZE + MAPLE_BAL_MSG_HDR_SIZE)
		return;
	hdr = (struct maple_bal_hdr *)skb->data;
	if (!(hdr->flags & 0x80))	/* bit7 = dir (1=response) */
		return;
	corr = be16_to_cpu(hdr->corr_tag);
	p = &mdev->bal.slots[corr % MAPLE_BAL_PENDING];

	body = skb->len - MAPLE_BAL_HDR_SIZE;
	/* Strip the 18-byte bcmolt_msg reply header; deliver just the object data. */
	payload = (body >= MAPLE_BAL_MSG_HDR_SIZE) ? body - MAPLE_BAL_MSG_HDR_SIZE : 0;
	if (p->reply && payload <= p->reply_len) {
		memcpy(p->reply, skb->data + MAPLE_BAL_HDR_SIZE + MAPLE_BAL_MSG_HDR_SIZE,
		       payload);
		if (p->reply_len)
			p->reply_len = payload;
		p->err = 0;
	} else {
		p->err = -EPROTO;
	}
	complete(&p->done);
}
