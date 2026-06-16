/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gpon_proto.h — interface to the portable, endianness-agnostic, HW-free GPON
 * PLOAM protocol core. The driver (imperative shell) implements the side-effect
 * ops below against real hardware; the host fuzzer / unit tests stub them. The
 * core itself (gpon_fsm_handle) does ONLY explicit byte math on the wire bytes,
 * so it is identical on big-endian MIPS and little-endian ARM and compiles +
 * fuzzes on the x86 host (libFuzzer+ASan+UBSan), where KASAN is unavailable on
 * MIPS.  See CLAUDE.md "Endianness-agnostic, host-fuzzable code".
 */
#ifndef GPON_PROTO_H
#define GPON_PROTO_H

#ifndef __KERNEL__
/* Host build (fuzzer / unit test): provide the kernel-ish types + no-op logging
 * so the core compiles standalone. In the kernel these come from <linux/*>. */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>			/* memcmp */
#ifndef fallthrough			/* kernel provides this; host fuzzer build does not */
#define fallthrough
#endif
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
#ifndef pr_info
#define pr_info(...)			do { } while (0)
#endif
#ifndef pr_info_ratelimited
#define pr_info_ratelimited(...)	do { } while (0)
#endif
#endif /* !__KERNEL__ */

/* G.984.3 downstream PLOAM message types. */
#define PLM_DS_UPSTREAM_OVERHEAD	0x01
#define PLM_DS_ASSIGN_ONU_ID		0x03
#define PLM_DS_RANGING_TIME		0x04
#define PLM_DS_DEACTIVATE_ONU		0x05
#define PLM_DS_DISABLE_SN		0x06
#define PLM_DS_ENCRYPT_PORT		0x08
#define PLM_DS_ASSIGN_ALLOC_ID		0x0a
#define PLM_DS_REQUEST_PASSWORD		0x09
#define PLM_DS_REQUEST_KEY		0x0d
#define PLM_DS_CONFIG_PORT		0x0e
#define PLM_DS_KEY_SWITCH		0x13
#define PLM_DS_EXT_BURST_LENGTH		0x14

/* G.984.3 upstream PLOAM message types + US_PLOAM_IND queue selectors. */
#define PLM_US_SERIAL_NUMBER		0x01
#define PLM_US_PASSWORD			0x02
#define PLM_US_ACKNOWLEDGE		0x09
#define PLM_US_QUEUE_SN			0x6
#define PLM_US_QUEUE_URG		0x1

#define GPON_GTC_DS_ONU_STATUS		0x1010
#define GPON_GTC_US_ONU_ID		0x5010
#define GPON_OMCC_TCONT			16

/* --- FSM state the core reads/writes (owned by the shell) --- */
extern u8   gpon_fsm_state;
extern u8   gpon_fsm_onu_id;
extern u8   gpon_sn_bytes[8];
extern u8   gpon_boh_guard, gpon_boh_ptn, gpon_boh_delim[3];
extern u8   gpon_boh_t3pre, gpon_boh_t3ranged;
extern bool gpon_omcc_installed, gpon_tcont_installed, gpon_key_staged;
extern u32  gpon_aes_switch_time;
extern u32  gpon_fsm_sn_tx;

/* --- imperative shell: HW register I/O + side effects the core invokes.
 *     Driver implements vs real HW; fuzzer/tests stub. --- */
void gpon_field(u32 off, unsigned int msb, unsigned int lsb, u32 val);
void gpon_wr(u32 off, u32 val);
void gpon_apply_boh(bool ranged);
void gpon_set_eqd(u32 value);
void gpon_fsm_set_state(u8 st);
void gpon_send_cpu_ploam(u8 queue, const u8 m[12]);	/* HW US-PLOAM TX (shell) */
void gpon_send_sn(void);	/* core (gpon_proto.c): builds SN buffer */
void gpon_send_ack(const u8 *ds);	/* core (gpon_proto.c): builds ACK from ds */
void gpon_send_password(void);	/* core (gpon_proto.c): builds empty Password (US 0x02) */
void gpon_send_key(void);
int  gpon_install_omcc(u16 gem);
int  gpon_install_tcont(u8 tcont, u16 alloc);

/* --- the portable protocol core --- */
void gpon_fsm_handle(const u8 *m);

#endif /* GPON_PROTO_H */
