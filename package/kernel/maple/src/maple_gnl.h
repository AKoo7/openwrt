/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Userspace ABI for the open Maple driver — a genetlink family "maple" exposing
 * the ONU management ops (get-cfg = listing fields, get-stat = bandwidth,
 * set-state = block/unblock). Driven by the `maplectl` userspace tool.
 */
#ifndef MAPLE_GNL_H
#define MAPLE_GNL_H

#define MAPLE_GENL_NAME		"maple"
#define MAPLE_GENL_VERSION	1

/* netlink attributes */
enum {
	MAPLE_A_UNSPEC,
	MAPLE_A_PON_NI,		/* u8  */
	MAPLE_A_ONU_ID,		/* u16 */
	MAPLE_A_OP,		/* u32 (onu_operation, for set_state) */
	MAPLE_A_CFG,		/* binary struct maple_onu_cfg / maple_ni_cfg */
	MAPLE_A_STAT,		/* binary struct maple_onu_stat / maple_ni_stat */
	__MAPLE_A_MAX,
};
#define MAPLE_A_MAX (__MAPLE_A_MAX - 1)

/* commands */
enum {
	MAPLE_C_UNSPEC,
	MAPLE_C_ONU_GET_CFG,
	MAPLE_C_ONU_GET_STAT,
	MAPLE_C_ONU_SET_STATE,
	MAPLE_C_NI_GET_CFG,
	MAPLE_C_NI_GET_STAT,
	__MAPLE_C_MAX,
};
#define MAPLE_C_MAX (__MAPLE_C_MAX - 1)

int maple_gnl_init(void);
void maple_gnl_exit(void);

#endif /* MAPLE_GNL_H */
