/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Userspace genetlink client for the open Maple "maple" family. Shared by
 * `maplectl` and `maple_snmp`. Defines the kernel fixed-width types as their
 * userspace equivalents, then pulls in the shared maple_onu.h/maple_regs.h so
 * there is ONE definition of the ONU structs + op-IDs (no duplication, and the
 * in-memory structs are host-order — the BE wire conversion happens only in
 * maple_codec.h).
 */
#ifndef MAPLE_LIB_H
#define MAPLE_LIB_H

#include <stdint.h>
#include <stddef.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#ifndef __packed
#define __packed __attribute__((packed))
#endif
#define _LINUX_TYPES_H		/* let maple_onu.h's <linux/types.h> be a no-op */

#include "maple_regs.h"		/* op-IDs, op constants */
#include "maple_onu.h"		/* struct maple_onu_cfg / maple_onu_stat */
#include "maple_ni.h"		/* struct maple_ni_cfg / maple_ni_stat */

/* onu_operation aliases for the CLI */
#define MAPLE_OP_ACTIVE		MAPLE_ONU_OP_ACTIVE
#define MAPLE_OP_INACTIVE	MAPLE_ONU_OP_INACTIVE
#define MAPLE_OP_DISABLE	MAPLE_ONU_OP_DISABLE
#define MAPLE_OP_ENABLE	MAPLE_ONU_OP_ENABLE

/* API. <0 = -errno. */
int  maple_gnl_open(void);
int  maple_gnl_onu_get_cfg(uint8_t pon, uint16_t onu, struct maple_onu_cfg *out);
int  maple_gnl_onu_get_stat(uint8_t pon, uint16_t onu, struct maple_onu_stat *out);
int  maple_gnl_onu_set_state(uint8_t pon, uint16_t onu, uint32_t op);
int  maple_gnl_ni_get_cfg(uint8_t pon, struct maple_ni_cfg *out);
int  maple_gnl_ni_get_stat(uint8_t pon, struct maple_ni_stat *out);

const char *maple_state_name(uint32_t s);
const char *maple_deact_reason_name(uint32_t r);

#endif /* MAPLE_LIB_H */
