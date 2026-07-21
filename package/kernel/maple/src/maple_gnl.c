// SPDX-License-Identifier: GPL-2.0
/*
 * Genetlink userspace ABI. See maple_gnl.h.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <net/genetlink.h>
#include <net/netlink.h>

#include "maple_gnl.h"
#include "maple_onu.h"
#include "maple_ni.h"
#include "maple_pci.h"

extern struct maple_dev *maple_first_dev;

static struct genl_family maple_family;

static const struct nla_policy maple_policy[MAPLE_A_MAX + 1] = {
	[MAPLE_A_PON_NI] = { .type = NLA_U8 },
	[MAPLE_A_ONU_ID] = { .type = NLA_U16 },
	[MAPLE_A_OP]     = { .type = NLA_U32 },
};

static int maple_gnl_need(struct genl_info *info, struct maple_dev **out,
			  u8 *pon, u16 *onu)
{
	if (!maple_first_dev)
		return -ENODEV;
	if (!info->attrs[MAPLE_A_PON_NI] || !info->attrs[MAPLE_A_ONU_ID])
		return -EINVAL;
	*out = maple_first_dev;
	*pon = nla_get_u8(info->attrs[MAPLE_A_PON_NI]);
	*onu = nla_get_u16(info->attrs[MAPLE_A_ONU_ID]);
	return 0;
}

static int maple_gnl_onu_cfg(struct sk_buff *skb, struct genl_info *info)
{
	struct maple_dev *mdev;
	u8 wire[256];
	struct sk_buff *rep;
	void *hdr;
	u8 pon; u16 onu;
	int err, len;

	err = maple_gnl_need(info, &mdev, &pon, &onu);
	if (err)
		return err;
	/* raw BE wire bytes from the MAC — endian-independent ABI */
	len = maple_onu_get_cfg_raw(mdev, pon, onu, wire, sizeof(wire));
	if (len < 0)
		return len;

	rep = genlmsg_new(nla_total_size(len), GFP_KERNEL);
	if (!rep)
		return -ENOMEM;
	hdr = genlmsg_put_reply(rep, info, &maple_family, 0, MAPLE_C_ONU_GET_CFG);
	if (!hdr)
		goto nla_put_failure;
	if (nla_put(rep, MAPLE_A_CFG, len, wire))
		goto nla_put_failure;
	genlmsg_end(rep, hdr);
	return genlmsg_reply(rep, info);

nla_put_failure:
	nlmsg_free(rep);
	return -EMSGSIZE;
}

static int maple_gnl_onu_stat(struct sk_buff *skb, struct genl_info *info)
{
	struct maple_dev *mdev;
	u8 wire[192];
	struct sk_buff *rep;
	void *hdr;
	u8 pon; u16 onu;
	int err, len;

	err = maple_gnl_need(info, &mdev, &pon, &onu);
	if (err)
		return err;
	len = maple_onu_get_stat_raw(mdev, pon, onu, wire, sizeof(wire));
	if (len < 0)
		return len;

	rep = genlmsg_new(nla_total_size(len), GFP_KERNEL);
	if (!rep)
		return -ENOMEM;
	hdr = genlmsg_put_reply(rep, info, &maple_family, 0, MAPLE_C_ONU_GET_STAT);
	if (!hdr)
		goto nla_put_failure;
	if (nla_put(rep, MAPLE_A_STAT, len, wire))
		goto nla_put_failure;
	genlmsg_end(rep, hdr);
	return genlmsg_reply(rep, info);

nla_put_failure:
	nlmsg_free(rep);
	return -EMSGSIZE;
}

static int maple_gnl_need_pon(struct genl_info *info, struct maple_dev **out, u8 *pon)
{
	if (!maple_first_dev)
		return -ENODEV;
	if (!info->attrs[MAPLE_A_PON_NI])
		return -EINVAL;
	*out = maple_first_dev;
	*pon = nla_get_u8(info->attrs[MAPLE_A_PON_NI]);
	return 0;
}

static int maple_gnl_onu_set_state(struct sk_buff *skb, struct genl_info *info)
{
	struct maple_dev *mdev;
	u8 pon; u16 onu; u32 op;
	int err;

	err = maple_gnl_need(info, &mdev, &pon, &onu);
	if (err)
		return err;
	if (!info->attrs[MAPLE_A_OP])
		return -EINVAL;
	op = nla_get_u32(info->attrs[MAPLE_A_OP]);
	return maple_onu_set_state(mdev, pon, onu, op);
}

static int maple_gnl_ni_cfg(struct sk_buff *skb, struct genl_info *info)
{
	struct maple_dev *mdev;
	u8 wire[384];
	struct sk_buff *rep;
	void *hdr;
	u8 pon;
	int err, len;

	err = maple_gnl_need_pon(info, &mdev, &pon);
	if (err)
		return err;
	len = maple_ni_get_cfg_raw(mdev, pon, wire, sizeof(wire));
	if (len < 0)
		return len;

	rep = genlmsg_new(nla_total_size(len), GFP_KERNEL);
	if (!rep)
		return -ENOMEM;
	hdr = genlmsg_put_reply(rep, info, &maple_family, 0, MAPLE_C_NI_GET_CFG);
	if (!hdr)
		goto nla_put_failure;
	if (nla_put(rep, MAPLE_A_CFG, len, wire))
		goto nla_put_failure;
	genlmsg_end(rep, hdr);
	return genlmsg_reply(rep, info);

nla_put_failure:
	nlmsg_free(rep);
	return -EMSGSIZE;
}

static int maple_gnl_ni_stat(struct sk_buff *skb, struct genl_info *info)
{
	struct maple_dev *mdev;
	u8 wire[288];
	struct sk_buff *rep;
	void *hdr;
	u8 pon;
	int err, len;

	err = maple_gnl_need_pon(info, &mdev, &pon);
	if (err)
		return err;
	len = maple_ni_get_stat_raw(mdev, pon, wire, sizeof(wire));
	if (len < 0)
		return len;

	rep = genlmsg_new(nla_total_size(len), GFP_KERNEL);
	if (!rep)
		return -ENOMEM;
	hdr = genlmsg_put_reply(rep, info, &maple_family, 0, MAPLE_C_NI_GET_STAT);
	if (!hdr)
		goto nla_put_failure;
	if (nla_put(rep, MAPLE_A_STAT, len, wire))
		goto nla_put_failure;
	genlmsg_end(rep, hdr);
	return genlmsg_reply(rep, info);

nla_put_failure:
	nlmsg_free(rep);
	return -EMSGSIZE;
}

static const struct genl_ops maple_ops[] = {
	{ .cmd = MAPLE_C_ONU_GET_CFG,    .doit = maple_gnl_onu_cfg,	      },
	{ .cmd = MAPLE_C_ONU_GET_STAT,   .doit = maple_gnl_onu_stat,      },
	{ .cmd = MAPLE_C_ONU_SET_STATE,  .doit = maple_gnl_onu_set_state, },
	{ .cmd = MAPLE_C_NI_GET_CFG,     .doit = maple_gnl_ni_cfg,	      },
	{ .cmd = MAPLE_C_NI_GET_STAT,    .doit = maple_gnl_ni_stat,	      },
};

static struct genl_family maple_family __ro_after_init = {
	.name	  = MAPLE_GENL_NAME,
	.version  = MAPLE_GENL_VERSION,
	.maxattr  = MAPLE_A_MAX,
	.policy	  = maple_policy,
	.ops	  = maple_ops,
	.n_ops	  = ARRAY_SIZE(maple_ops),
	.module	  = THIS_MODULE,
};

int maple_gnl_init(void)
{
	return genl_register_family(&maple_family);
}

void maple_gnl_exit(void)
{
	genl_unregister_family(&maple_family);
}
