// SPDX-License-Identifier: GPL-2.0
/*
 * GPON ONU management ops. The wire pack/unpack lives in maple_codec.h (shared
 * with the oracle test); this file is the thin BAL-call wrapper. See
 * docs/onu-management-data-contract.md + docs/decomp/trmux.md.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <asm/byteorder.h>

#include "maple_onu.h"
#include "maple_codec.h"
#include "maple_bal.h"
#include "maple_bal_msg.h"
#include "maple_pci.h"

int maple_onu_set_state(struct maple_dev *mdev, u8 pon_ni, u16 onu_id, u32 op)
{
	u8 body[1];
	struct maple_cur c = { .p = body, .len = sizeof(body) };
	int r = maple_set_state_pack(&c, op);

	if (r)
		return -EINVAL;
	return maple_bal_call(mdev, MAPLE_GRP_GPON_ONU_SET_ONU_STATE, pon_ni,
			      MAPLE_OBJ_GPON_ONU, MAPLE_MGT_GROUP_OPER, 0,
			      MAPLE_MSG_TYPE_SET,
			      body, c.off, NULL, NULL);
}

int maple_onu_get_cfg(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
		      struct maple_onu_cfg *cfg)
{
	u8 buf[256];
	int n = maple_onu_get_cfg_raw(mdev, pon_ni, onu_id, buf, sizeof(buf));

	if (n < 0)
		return n;
	return maple_onu_cfg_unpack(cfg, buf, n) ? -EBADMSG : 0;
}

int maple_onu_get_stat(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
		       struct maple_onu_stat *stat)
{
	u8 buf[192];
	int n = maple_onu_get_stat_raw(mdev, pon_ni, onu_id, buf, sizeof(buf));

	if (n < 0)
		return n;
	return maple_onu_stat_unpack(stat, buf, n) ? -EBADMSG : 0;
}

int maple_onu_get_cfg_raw(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
			  void *buf, size_t buflen)
{
	u8 key[4];
	struct maple_cur kc = { .p = key, .len = sizeof(key) };
	size_t rlen = buflen;
	int err = maple_onu_key_pack(&kc, pon_ni, onu_id);

	if (err)
		return -EINVAL;
	err = maple_bal_call(mdev, MAPLE_GRP_GPON_ONU_CFG, pon_ni,
			     MAPLE_OBJ_GPON_ONU, MAPLE_MGT_GROUP_CFG, 0,
			     MAPLE_MSG_TYPE_GET,
			     key, kc.off, buf, &rlen);
	if (err)
		return err;
	return (int)rlen;
}

int maple_onu_get_stat_raw(struct maple_dev *mdev, u8 pon_ni, u16 onu_id,
			   void *buf, size_t buflen)
{
	u8 key[4];
	struct maple_cur kc = { .p = key, .len = sizeof(key) };
	size_t rlen = buflen;
	int err = maple_onu_key_pack(&kc, pon_ni, onu_id);

	if (err)
		return -EINVAL;
	err = maple_bal_call(mdev, MAPLE_GRP_GPON_ONU_STAT, pon_ni,
			     MAPLE_OBJ_GPON_ONU, MAPLE_MGT_GROUP_STAT, 0,
			     MAPLE_MSG_TYPE_GET,
			     key, kc.off, buf, &rlen);
	if (err)
		return err;
	return (int)rlen;
}
