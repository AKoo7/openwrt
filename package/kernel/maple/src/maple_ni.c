// SPDX-License-Identifier: GPL-2.0
/*
 * GPON-NI (PON port) management ops. Thin BAL-call wrapper; the wire
 * pack/unpack lives in maple_codec.h. See docs/onu-management-data-contract.md.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <asm/byteorder.h>

#include "maple_ni.h"
#include "maple_codec.h"
#include "maple_bal.h"
#include "maple_bal_msg.h"
#include "maple_pci.h"

int maple_ni_get_cfg_raw(struct maple_dev *mdev, u8 pon_ni,
			 void *buf, size_t buflen)
{
	u8 key[1];
	struct maple_cur kc = { .p = key, .len = sizeof(key) };
	size_t rlen = buflen;
	int err = maple_ni_key_pack(&kc, pon_ni);

	if (err)
		return -EINVAL;
	err = maple_bal_call(mdev, MAPLE_GRP_GPON_NI_CFG, pon_ni,
			     MAPLE_OBJ_GPON_NI, MAPLE_MGT_GROUP_CFG, 0,
			     MAPLE_MSG_TYPE_GET,
			     key, kc.off, buf, &rlen);
	if (err)
		return err;
	return (int)rlen;
}

int maple_ni_get_stat_raw(struct maple_dev *mdev, u8 pon_ni,
			  void *buf, size_t buflen)
{
	u8 key[1];
	struct maple_cur kc = { .p = key, .len = sizeof(key) };
	size_t rlen = buflen;
	int err = maple_ni_key_pack(&kc, pon_ni);

	if (err)
		return -EINVAL;
	err = maple_bal_call(mdev, MAPLE_GRP_GPON_NI_STAT, pon_ni,
			     MAPLE_OBJ_GPON_NI, MAPLE_MGT_GROUP_STAT, 0,
			     MAPLE_MSG_TYPE_GET,
			     key, kc.off, buf, &rlen);
	if (err)
		return err;
	return (int)rlen;
}

int maple_ni_get_cfg(struct maple_dev *mdev, u8 pon_ni, struct maple_ni_cfg *cfg)
{
	u8 buf[384];
	int n = maple_ni_get_cfg_raw(mdev, pon_ni, buf, sizeof(buf));

	if (n < 0)
		return n;
	return maple_ni_cfg_unpack(cfg, buf, n) ? -EBADMSG : 0;
}

int maple_ni_get_stat(struct maple_dev *mdev, u8 pon_ni,
		      struct maple_ni_stat *stat)
{
	u8 buf[288];
	int n = maple_ni_get_stat_raw(mdev, pon_ni, buf, sizeof(buf));

	if (n < 0)
		return n;
	return maple_ni_stat_unpack(stat, buf, n) ? -EBADMSG : 0;
}
