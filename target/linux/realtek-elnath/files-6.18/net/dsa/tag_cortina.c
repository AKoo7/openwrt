// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * net/dsa/tag_cortina.c - soft-shim DSA tagger for the RTL9607F Cortina NI
 * (drivers/net/ethernet/cortina/cortina-ni*).
 *
 * The NI carries the per-jack selector OUT OF BAND (RX in the DMA HEADER_A
 * metadata, TX in the DMA descriptor DESC1_DEST), never on the wire.  So this
 * tagger is asymmetric:
 *
 *   RX: nothing here.  The cortina RX path attaches a METADATA_HW_PORT_MUX
 *       metadata_dst carrying the source jack, and net/dsa/tag.c:dsa_switch_rcv()
 *       consumes it and demuxes to lanN before ever calling .rcv.  .rcv below is
 *       only a defensive drop for the (unexpected) no-metadata frame.
 *
 *   TX: push a 2-byte prefix (FRONT, before the DMAC) carrying dp->index.  This
 *       is the ONLY channel that survives dsa_user_xmit()'s memset(skb->cb) and
 *       the conduit's qdisc (which owns skb->cb via qdisc_skb_cb).  The cortina
 *       conduit xmit (cortina_ni_start_xmit) reads the prefix, folds it into
 *       DESC1_DEST, and skb_pull()s it off before the DMA, so it never hits the
 *       wire.
 *
 * ★ CORTINA_TAG_* MUST stay byte-identical to CA_NI_DSA_TAG_* in
 *   drivers/net/ethernet/cortina/cortina-ni-regs.h (that header cannot be
 *   included from net/dsa/).
 */

#include "tag.h"

#define CORTINA_NAME		"cortina"

#define CORTINA_TAG_LEN		2
#define CORTINA_TAG_PORT_OFF	0	/* byte[0] = MAGIC | dp->index */
#define CORTINA_TAG_PORT_MASK	0x0f
#define CORTINA_TAG_MAGIC	0xd0	/* high nibble marks a tagged frame */

static struct sk_buff *cortina_tag_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct dsa_port *dp = dsa_user_to_port(dev);
	u8 *tag;

	/* Front prefix, before the DMAC.  No dsa_alloc_etype_header(): the shim
	 * is not an EtherType header, and the cortina conduit xmit strips it in
	 * software (pull) before DMA rather than the hardware parsing it.  The
	 * MAGIC high nibble lets the conduit xmit distinguish this from an
	 * untagged frame the conduit might originate. */
	skb_push(skb, CORTINA_TAG_LEN);
	tag = skb->data;
	tag[CORTINA_TAG_PORT_OFF] = CORTINA_TAG_MAGIC | (dp->index & CORTINA_TAG_PORT_MASK);
	tag[1] = 0;

	return skb;
}

static struct sk_buff *cortina_tag_rcv(struct sk_buff *skb,
				       struct net_device *dev)
{
	/* Real LAN frames carry a METADATA_HW_PORT_MUX metadata_dst that
	 * dsa_switch_rcv() consumes before reaching here.  A frame with none has
	 * no recoverable source jack, so drop it rather than risk a misroute. */
	net_warn_ratelimited("cortina tag: XDSA frame without port metadata on %s\n",
			     netdev_name(dev));
	return NULL;
}

static const struct dsa_device_ops cortina_netdev_ops = {
	.name			= CORTINA_NAME,
	.proto			= DSA_TAG_PROTO_CORTINA,
	.xmit			= cortina_tag_xmit,
	.rcv			= cortina_tag_rcv,
	.needed_headroom	= CORTINA_TAG_LEN,
};

module_dsa_tag_driver(cortina_netdev_ops);
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_CORTINA, CORTINA_NAME);
MODULE_DESCRIPTION("DSA tag driver for the Cortina/Realtek RTL9607F NI switch");
MODULE_LICENSE("GPL");
