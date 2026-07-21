/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BAL wire codec — pack/unpack the GPON ONU objects to/from the vendor's
 * field-by-field BIG-ENDIAN wire format (enums=1B, u16/u32/u64 BE, byte-arrays
 * raw, no padding). Pure static-inline, shared by the driver (maple_onu.c) and
 * the userspace oracle test. Returns 0 on success, -1 on overflow/underflow.
 *
 * Wire widths verified against bcmolt_*_pack in trmux.ko (see
 * docs/decomp/trmux.md + onu-management-data-contract.md).
 */
#ifndef MAPLE_CODEC_H
#define MAPLE_CODEC_H

#ifdef __KERNEL__
#include <linux/string.h>
#else
#include <stddef.h>
#include <string.h>
#endif
#include "maple_onu.h"	/* structs (also provides size_t in-kernel) */
#include "maple_ni.h"

#ifndef cpu_to_be16
#include <endian.h>
#define cpu_to_be16 htobe16
#define cpu_to_be32 htobe32
#define cpu_to_be64 htobe64
#define be16_to_cpu be16toh
#define be32_to_cpu be32toh
#define be64_to_cpu be64toh
#endif

struct maple_cur {
	unsigned char *p;
	size_t len, off;
};

static inline int mcur_put(struct maple_cur *c, const void *src, size_t n)
{
	if (c->off + n > c->len)
		return -1;
	memcpy(c->p + c->off, src, n);
	c->off += n;
	return 0;
}
static inline int mcur_get(struct maple_cur *c, void *dst, size_t n)
{
	if (c->off + n > c->len)
		return -1;
	memcpy(dst, c->p + c->off, n);
	c->off += n;
	return 0;
}
static inline int mput_u8(struct maple_cur *c, unsigned char v) { return mcur_put(c, &v, 1); }
static inline int mput_be16(struct maple_cur *c, unsigned short v) { unsigned short b = cpu_to_be16(v); return mcur_put(c, &b, 2); }
static inline int mput_be32(struct maple_cur *c, unsigned int v) { unsigned int b = cpu_to_be32(v); return mcur_put(c, &b, 4); }
static inline int mput_be64(struct maple_cur *c, unsigned long long v) { unsigned long long b = cpu_to_be64(v); return mcur_put(c, &b, 8); }
static inline int mget_u8(struct maple_cur *c, unsigned char *v) { return mcur_get(c, v, 1); }
static inline int mget_be16(struct maple_cur *c, unsigned short *v) { unsigned short b; int r = mcur_get(c, &b, 2); if (!r) *v = be16_to_cpu(b); return r; }
static inline int mget_be32(struct maple_cur *c, unsigned int *v) { unsigned int b; int r = mcur_get(c, &b, 4); if (!r) *v = be32_to_cpu(b); return r; }
static inline int mget_be64(struct maple_cur *c, unsigned long long *v) { unsigned long long b; int r = mcur_get(c, &b, 8); if (!r) *v = be64_to_cpu(b); return r; }

/* bcmolt_gpon_onu_key pack (3 wire bytes: pon_ni:1 + onu_id:2 BE). */
static inline int maple_onu_key_pack(struct maple_cur *c, u8 pon_ni, u16 onu_id)
{
	int r = mput_u8(c, pon_ni);
	if (!r) r = mput_be16(c, onu_id);
	return r;
}

/* bcmolt_gpon_onu_set_onu_state_data pack (1 wire byte: onu_operation). */
static inline int maple_set_state_pack(struct maple_cur *c, u32 op)
{
	return mput_u8(c, (unsigned char)op);
}

/* bcmolt_gpon_onu_stat_data unpack (20 × u64 BE) -> struct maple_onu_stat. */
static inline int maple_onu_stat_unpack(struct maple_onu_stat *o, const void *buf, size_t len)
{
	struct maple_cur c = { (unsigned char *)buf, len, 0 };
	unsigned long long *f = (unsigned long long *)o;
	int i, r;

	for (i = 0; i < 20; i++) {
		r = mget_be64(&c, &f[i]);
		if (r)
			return r;
	}
	return 0;
}

/* bcmolt_gpon_onu_cfg_data unpack -> struct maple_onu_cfg (field widths match
 * the vendor pack; serial=4B wire, alarm_state=12×1B). */
static inline int maple_onu_cfg_unpack(struct maple_onu_cfg *o, const void *buf, size_t len)
{
	struct maple_cur c = { (unsigned char *)buf, len, 0 };
	unsigned char v;
	unsigned short s16;
	unsigned int s32;
	int i, r;

	memset(o, 0, sizeof(*o));
	r = mget_u8(&c, &v);   if (r) return r;  o->onu_state = v;
	r = mget_u8(&c, &v);   if (r) return r;  o->onu_old_state = v;
	r = mcur_get(&c, o->serial_number, 4);            if (r) return r;
	r = mcur_get(&c, o->password, 10);                if (r) return r;
	r = mget_u8(&c, &o->auto_password_learning);      if (r) return r;
	r = mget_u8(&c, &o->us_fec);                      if (r) return r;
	r = mget_be16(&c, &s16); o->omci_port_id = s16;   if (r) return r;
	r = mget_be32(&c, &s32); o->ds_ber_reporting_interval = s32; if (r) return r;
	r = mcur_get(&c, o->aes_encryption_key, 16);      if (r) return r;
	for (i = 0; i < 12; i++) { r = mget_u8(&c, &v); if (r) return r; o->alarm_state[i] = v; }
	r = mget_be32(&c, &s32); o->ranging_time = s32;   if (r) return r;
	r = mget_u8(&c, &v); o->disabled_after_discovery = v; if (r) return r;
	r = mget_u8(&c, &v); o->deactivation_reason = v;  if (r) return r;
	return 0;
}

/* bcmolt_gpon_ni_key pack (1 wire byte: pon_ni). */
static inline int maple_ni_key_pack(struct maple_cur *c, u8 pon_ni)
{
	return mput_u8(c, pon_ni);
}

/* bcmolt_gpon_ni_cfg_data unpack -> struct maple_ni_cfg (subset: first 64 B).
 * Field layout from pahole; the BAL wire is field-by-field BE with NO padding,
 * so we unpack in DWARF order, skipping the nested sub-struct bytes raw. */
static inline int maple_ni_cfg_unpack(struct maple_ni_cfg *o, const void *buf, size_t len)
{
	struct maple_cur c = { (unsigned char *)buf, len, 0 };
	unsigned long long v64;
	unsigned int v32;
	unsigned short v16;
	int r;

	memset(o, 0, sizeof(*o));
	r = mget_be64(&c, &v64); o->pon_status = v64;            if (r) return r;
	r = mcur_get(&c, o->available_bandwidth, 12);            if (r) return r;
	r = mget_be16(&c, &v16); o->number_of_active_onus = v16; if (r) return r;
	r = mget_be16(&c, &v16); o->number_of_active_standby_onus = v16; if (r) return r;
	r = mcur_get(&c, o->prbs_status, 8);                     if (r) return r;
	r = mget_be64(&c, &v64); o->pon_distance = v64;          if (r) return r;
	r = mget_be32(&c, &v32); o->ranging_window_size = v32;   if (r) return r;
	r = mget_be32(&c, &v32); o->preassigned_equalization_delay = v32; if (r) return r;
	r = mget_be32(&c, &v32); o->eqd_cycles_number = v32;     if (r) return r;
	r = mcur_get(&c, o->power_level, 8);                     if (r) return r;
	r = mget_be32(&c, &v32); o->ds_fec_mode = v32;           if (r) return r;
	return 0;
}

/* bcmolt_gpon_ni_stat_data unpack -> struct maple_ni_stat (272 B, 34 × u64 BE
 * with one u8 at offset 240, then 3 × u64 trailing). */
static inline int maple_ni_stat_unpack(struct maple_ni_stat *o, const void *buf, size_t len)
{
	struct maple_cur c = { (unsigned char *)buf, len, 0 };
	unsigned long long *f = (unsigned long long *)o;
	unsigned char v8;
	int i, r;

	/* 30 × u64 (offsets 0..232) */
	for (i = 0; i < 30; i++) {
		r = mget_be64(&c, &f[i]);
		if (r)
			return r;
	}
	/* 1 × u8 (offset 240) */
	r = mget_u8(&c, &v8);
	if (r)
		return r;
	o->tx_cpu_omci_packets_dropped = v8;
	/* skip 7-byte hole */
	r = mcur_get(&c, o->_pad, 7);
	if (r)
		return r;
	/* 3 × u64 (offsets 248..264) */
	for (i = 0; i < 3; i++) {
		r = mget_be64(&c, &f[30 + i]);
		if (r)
			return r;
	}
	return 0;
}

#endif /* MAPLE_CODEC_H */
