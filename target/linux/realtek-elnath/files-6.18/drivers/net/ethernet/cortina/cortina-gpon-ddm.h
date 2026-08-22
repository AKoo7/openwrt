/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SFF-8472 A2h digital-diagnostic-monitoring (DDM) decode + integer scaling for
 * the GPON BOSA optical monitor (Realtek RTL9607F "Elnath").
 *
 * FUNCTIONAL CORE: pure functions only.  No MMIO, no i2c, no kernel API beyond
 * the integer types, no floating point.  The imperative shell that actually
 * drives the bus lives in cortina-gpon-bosa.c; keeping the decode separate is
 * what lets the scaling + the "unavailable" policy be unit-tested on the x86
 * host instead of on a board (project rule: prove it offline first).
 *
 * ---------------------------------------------------------------------------
 * The register map, and the evidence for it
 * ---------------------------------------------------------------------------
 * The BOSA answers at i2c slave 0x51, i.e. the SFF-8472 "A2h" diagnostic
 * address (0xA2 >> 1).  Three independent facts from this board say the device
 * really does implement the standard A2h layout, not just the address:
 *
 *   - the stock bring-up trace (tier 2: this board's own captured on-wire
 *     rtkbosa sequence, replayed verbatim in cortina-gpon-bosa-seq.h) writes
 *     un-paged registers 0x00-0x4F and then ONLY 0x6E, 0x6F, 0x72, 0x78, 0x79,
 *     0x7B-0x7E and 0x7F.  0x00-0x37 is exactly the A2h alarm/warning
 *     THRESHOLD block (bytes 0-55) and 0x38-0x4F the calibration-constant
 *     block (bytes 56-79) -- precisely what a host programs on an A2h device;
 *   - register 0x6E bit6 is the laser soft-disable the existing code already
 *     uses (BOSA_REG_TX_CTL / BOSA_TX_SOFT_DIS), which is SFF-8472 byte 110
 *     "Status/Control" bit 6 "Soft TX Disable" -- an exact field match;
 *   - register 0x7F is the table/page select for the 0x80-0xFF window, which is
 *     SFF-8472 byte 127 "page select" -- a second exact field match.
 *
 * Regs 0x60-0x69 (bytes 96-105) are the real-time diagnostics.  They are LIVE,
 * not part of the programmed image: the stock write trace contains ZERO writes
 * to them, and their values drift between reads seconds apart.  They sit in the
 * UN-PAGED 0x00-0x7F window, so reading them needs no page dance and must never
 * touch the table-select register 0x7F.
 *
 * The optic is factory-calibrated internally (the stock rtkbosa carries
 * gn28l95_ddmi_k_* / gn28l95_is_calibrated helpers), so the words below are
 * already-calibrated readings: do NOT apply a host-side calibration polynomial
 * and do NOT add a per-board constant.
 *
 * Scaling (SFF-8472 table 9-11; the four non-RX fields cross-checked against
 * decoded stock dumps of this very module, tier 2):
 *
 *   0x60-61  temperature   s16, 1/256 degC     raw 0x265b -> 38.36 degC
 *   0x62-63  Vcc           u16, 100 uV         raw 0x7e24 -> 3.229 V
 *   0x64-65  TX bias       u16, 2 uA           raw 0x1d7e -> 15.10 mA
 *   0x66-67  TX power      u16, 0.1 uW         raw 0x432d -> +2.35 dBm
 *   0x68-69  RX power      u16, 0.1 uW         raw 0x1652 -> -2.43 dBm
 *
 * On the 0.1 uW LSB: SFF-8472 specifies the same unit for TX and RX power, and
 * TX is decisive -- at a 0.01 uW LSB the stock TX reading would be -7.65 dBm,
 * impossible for a GPON class-B+ transmitter (roughly +0.5..+5 dBm), while
 * 0.1 uW gives +2.35 dBm.  So 0.1 uW is used for both, as the standard says.
 * The resulting RX of -2.43 dBm is above the nominal class-B+ overload point
 * (~-8 dBm); that is consistent with this lab rig being a short patch cord with
 * no attenuator, i.e. the link is hot rather than the scaling being wrong.  The
 * RAW word is exported unconditionally either way, so a reader can re-scale
 * without a firmware change.  The direct oracle for RX specifically is the
 * ONU-reported RX field in the OLT's optical view, or a stock NAND boot reading
 * these same registers with the vendor's own scaling -- NOT the OLT's
 * "show ont-info" received power, which validates our TX.
 */

#ifndef _CORTINA_GPON_DDM_H_
#define _CORTINA_GPON_DDM_H_

#include <linux/types.h>

/* SFF-8472 A2h real-time diagnostics: byte 96, ten bytes, read-only. */
#define CG_DDM_BASE		0x60
#define CG_DDM_LEN		10

/* Why a sample is not usable.  Anything but CG_DDM_OK means NO number may be
 * printed or reported -- a fabricated 0 for a failed read is the one thing this
 * whole file exists to prevent. */
enum cg_ddm_status {
	CG_DDM_OK = 0,
	CG_DDM_ERR_IO,		/* an i2c transfer failed */
	CG_DDM_ERR_ALL_ZERO,	/* every byte 0x00: unrouted pinmux fake-ACK */
	CG_DDM_ERR_ALL_ONES,	/* every byte 0xFF: floating bus / no device */
};

/* One decoded sample.  The RAW 16-bit words are kept verbatim (spy/dump is a
 * first-class feature, and it makes the scaling re-checkable after the fact). */
struct cg_bosa_ddm {
	u16	temp;		/* 0x60-61 */
	u16	vcc;		/* 0x62-63 */
	u16	bias;		/* 0x64-65 */
	u16	tx_pwr;		/* 0x66-67 */
	u16	rx_pwr;		/* 0x68-69 */
	u8	raw[CG_DDM_LEN];
	u8	status;		/* enum cg_ddm_status */
};

/* An optical power that cannot come from any real reading (the weakest real
 * value, raw = 1, is -40.00 dBm), used for "the optic reports zero light". */
#define CG_DDM_CDBM_NONE	(-100000)

/* Clamp for the OMCI conversion: +-60 dBm covers every physical reading and
 * keeps the G.988 0.002 dB result inside s16. */
#define CG_DDM_CDBM_MIN		(-6000)
#define CG_DDM_CDBM_MAX		(6000)

/*
 * Decode ten raw A2h bytes.  @io_err is nonzero if any i2c transfer failed, in
 * which case @raw is not looked at.  Also rejects the two dead-bus signatures:
 * an unrouted i2c0 pinmux fake-ACKs every transfer and reads back all-0x00,
 * and an absent/floating device reads all-0xFF.  Both are indistinguishable
 * from "a real sample of exactly zero / exactly full-scale" field by field,
 * which is why the check is over the whole block.
 *
 * Returns an enum cg_ddm_status; @d->status carries the same value.  On any
 * failure the value fields are left zero AND status says so -- a caller that
 * ignores the status and prints d->rx_pwr gets an obvious 0, but every caller
 * in tree keys off the status.
 */
static inline int cg_ddm_decode(const u8 *raw, int io_err, struct cg_bosa_ddm *d)
{
	unsigned int i;
	u8 and_all = 0xff, or_all = 0x00;

	d->temp = 0;
	d->vcc = 0;
	d->bias = 0;
	d->tx_pwr = 0;
	d->rx_pwr = 0;
	for (i = 0; i < CG_DDM_LEN; i++)
		d->raw[i] = 0;

	if (io_err) {
		d->status = CG_DDM_ERR_IO;
		return d->status;
	}

	for (i = 0; i < CG_DDM_LEN; i++) {
		d->raw[i] = raw[i];
		and_all &= raw[i];
		or_all |= raw[i];
	}
	if (or_all == 0x00) {
		d->status = CG_DDM_ERR_ALL_ZERO;
		return d->status;
	}
	if (and_all == 0xff) {
		d->status = CG_DDM_ERR_ALL_ONES;
		return d->status;
	}

	/* explicit byte math, never a cast over wire bytes: the same code has to
	 * run on LE ARM64 and BE MIPS (project endianness rule) */
	d->temp   = ((u16)raw[0] << 8) | raw[1];
	d->vcc    = ((u16)raw[2] << 8) | raw[3];
	d->bias   = ((u16)raw[4] << 8) | raw[5];
	d->tx_pwr = ((u16)raw[6] << 8) | raw[7];
	d->rx_pwr = ((u16)raw[8] << 8) | raw[9];
	d->status = CG_DDM_OK;
	return d->status;
}

/* Temperature in milli-degrees C (s16, 1/256 degC).  Truncates toward zero. */
static inline s32 cg_ddm_temp_mdegc(u16 raw)
{
	return ((s32)(s16)raw * 1000) / 256;
}

/* Temperature in deci-degrees C -- the unit the existing rpcd/LuCI and test
 * scrapers already parse ("temp_dc="); full precision stays available through
 * the raw word (raw / 256 degC). */
static inline s32 cg_ddm_temp_dc(u16 raw)
{
	return ((s32)(s16)raw * 10) / 256;
}

/* Supply voltage in mV (100 uV units). */
static inline u32 cg_ddm_vcc_mv(u16 raw)
{
	return (u32)raw / 10;
}

/* Laser bias current in uA (2 uA units). */
static inline u32 cg_ddm_bias_ua(u16 raw)
{
	return (u32)raw * 2;
}

/*
 * 1000 * log10(x) for 1 <= x <= 65535, integer only.
 *
 * x = 2^msb * (1 + f), f in [0,1).  log10(x) = msb*log10(2) + log10(1+f); the
 * first term is msb * 0.30103 and the second comes from a 17-entry table of
 * 1000*log10(1 + k/16) with linear interpolation on 4 extra fractional bits.
 *
 * Cost: 34 bytes of .rodata plus a few dozen bytes of code.  MEASURED against
 * libm over the whole 1..65535 domain (host case [A] of
 * optic_ddm_decode_test): worst error 3 milli-decades = 0.03 dB, at x=1039.
 * That is two orders of magnitude tighter than the +-3 dB accuracy SFF-8472
 * itself allows for RX power.  A direct lookup table would cost 128 KB and the
 * kernel has no floating point, so this is the cheap honest middle.
 */
static inline s32 cg_ddm_log10_milli(u32 x)
{
	static const u16 mant[17] = {
		0,  26,  51,  75,  97, 118, 138, 158, 176,
		194, 211, 227, 243, 258, 273, 287, 301,
	};
	u32 r, frac, k, rem;
	int msb = 0;

	if (!x)
		return 0;		/* caller must special-case zero */
	while (x >> (msb + 1))
		msb++;

	r = x - ((u32)1 << msb);		/* 0 <= r < 2^msb */
	frac = msb ? ((r << 8) >> msb) : 0;	/* mantissa fraction, /256 */
	k = frac >> 4;				/* table index 0..15 */
	rem = frac & 0xf;			/* interpolation weight /16 */

	return ((s32)msb * 30103 + 50) / 100 + mant[k] +
	       (s32)((mant[k + 1] - mant[k]) * rem) / 16;
}

/*
 * Optical power in centi-dBm from an SFF-8472 0.1 uW word.
 *
 * P_mW = raw / 10000, so dBm = 10*log10(raw) - 40 and
 * centi-dBm = 1000*log10(raw) - 4000.
 *
 * raw == 0 means the optic reports no light at all (a dark fiber, or TX gated);
 * that is a VALID reading of an invalid logarithm, so it returns the
 * CG_DDM_CDBM_NONE sentinel rather than a made-up floor.
 */
static inline s32 cg_ddm_uw10_to_cdbm(u16 raw)
{
	if (!raw)
		return CG_DDM_CDBM_NONE;
	return cg_ddm_log10_milli(raw) - 4000;
}

/*
 * centi-dBm -> the G.988 ANI-G (ME 263) #10/#14 wire form: 2's complement, in
 * 0.002 dB increments referred to 1 mW, so dBm * 500 = centi-dBm * 5.  Clamped
 * to +-60 dBm, which also turns the "no light" sentinel into the floor -- the
 * OLT gets a conformant worst-case level instead of a nonsense number.
 */
static inline u16 cg_ddm_cdbm_to_omci(s32 cdbm)
{
	if (cdbm < CG_DDM_CDBM_MIN)
		cdbm = CG_DDM_CDBM_MIN;
	else if (cdbm > CG_DDM_CDBM_MAX)
		cdbm = CG_DDM_CDBM_MAX;
	return (u16)(s16)(cdbm * 5);
}

/* Human-readable reason a sample is unusable.  Never returns NULL. */
static inline const char *cg_ddm_status_str(u8 status)
{
	switch (status) {
	case CG_DDM_OK:
		return "live";
	case CG_DDM_ERR_IO:
		return "unavailable (i2c read error)";
	case CG_DDM_ERR_ALL_ZERO:
		return "unavailable (DEAD BUS: all ten bytes 0x00 - i2c0 pinmux unrouted?)";
	default:
		return "unavailable (DEAD BUS: all ten bytes 0xff - BOSA absent?)";
	}
}

#endif /* _CORTINA_GPON_DDM_H_ */
