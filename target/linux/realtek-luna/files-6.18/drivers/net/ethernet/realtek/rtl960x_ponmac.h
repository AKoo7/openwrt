/* SPDX-License-Identifier: GPL-2.0 */
/*
 * rtl960x_ponmac.h - board-agnostic bring-up of the Realtek RTL960x family
 * GPON PON-MAC / SerDes, matching the stock device's register behavior.
 *
 * Purpose: have a stock-faithful ponmac/SerDes bring-up for the WHOLE
 * RTL960x family in-tree, so any future 960x board can be brought online by
 * wiring its register accessor + chip id, without re-deriving the sequence
 * from scratch.
 *
 * Reconstructed per-chip from the stock register behavior, with every register
 * resolved to its ABSOLUTE physical address from that chip's register map and
 * field bit-ranges from its field map. The 9602C path is HW-tested (the
 * realtek-luna board); the other family members are behavior-faithful
 * transcriptions, untested for lack of hardware, ready for when a board is
 * connected.
 *
 * The bring-up uses ABSOLUTE physical register addresses; the board driver
 * supplies rd/wr that map phys->virt (KSEG1 0xa0000000|phys, or ioremap).
 * On the 9602C "Luna" SoC: swcore phys 0x1b000000, PON-IP 0x1bf00000,
 * GTC 0x1b700000 (all in the KSEG1-reachable 0x1bxxxxxx window).
 */
#ifndef _RTL960X_PONMAC_H
#define _RTL960X_PONMAC_H

#include <linux/types.h>

enum rtl960x_chip {
	RTL960X_CHIP_9601B = 0,
	RTL960X_CHIP_9602C,	/* + 9601C / 9601C_VB subtypes */
	RTL960X_CHIP_9603CVD,
	RTL960X_CHIP_9607C,
	RTL960X_CHIP_9607F,	/* no register map observed yet - bring-up TBD */
};

/* Chip revision id as read from HW: A=0x1, B=0x2, C=0x3, ... rev>A => B+ */
#define RTL960X_REV_A		0x1
#define RTL960X_REV_B		0x2
#define RTL960X_REV_C		0x3

/* Subtype (9602C family): pick GponModeV3 (9601C) vs V2 on rev>A. */
#define RTL960X_SUBTYPE_NONE		0x00
#define RTL960X_SUBTYPE_9601C_VB	0x01
#define RTL960X_SUBTYPE_9601C		0x03

enum rtl960x_ponmode {
	RTL960X_MODE_GPON = 0,
	RTL960X_MODE_EPON = 1,
	RTL960X_MODE_FIBER = 2,
};

/*
 * Board-agnostic register accessor. phys is the ABSOLUTE physical address from
 * the chip's register map. The board driver maps it (KSEG1 or ioremap).
 */
struct rtl960x_ops {
	u32  (*rd)(u32 phys);
	void (*wr)(u32 phys, u32 val);
};

/* read-modify-write bits [msb:lsb] at absolute phys address */
static inline void rtl960x_rfwr(const struct rtl960x_ops *o, u32 phys,
				u8 msb, u8 lsb, u32 val)
{
	u32 mask = (msb == 31 && lsb == 0) ? 0xffffffffu
					   : ((((1u << (msb - lsb + 1)) - 1)) << lsb);

	o->wr(phys, (o->rd(phys) & ~mask) | ((val << lsb) & mask));
}

/*
 * Bring-up entry points. rev = RTL960X_REV_A or the HW chip-revision id; subtype
 * as above. Return 0 on success, -ETIMEDOUT if the SerDes analog-ready gate never
 * asserts (non-fatal; the board driver may proceed + diagnose). chip == 9607F
 * returns -ENOTSUPP until its register map is available.
 */
int rtl960x_ponmac_init(enum rtl960x_chip chip, int rev, int subtype,
			const struct rtl960x_ops *o);
int rtl960x_ponmac_mode_set(enum rtl960x_chip chip, int rev, int subtype,
			    enum rtl960x_ponmode mode, const struct rtl960x_ops *o);
int rtl960x_ponmac_serdes_cdr_reset(enum rtl960x_chip chip,
				    const struct rtl960x_ops *o);

struct seq_file;
void rtl960x_c7_diag(const struct rtl960x_ops *o, struct seq_file *s);

extern int rtl960x_c2_postmode_perturb;
extern int rtl960x_c2_sds_cfgrst;
extern int rtl960x_c2_stock_analog;
extern int rtl960x_c2_analog_postreset;
extern int rtl960x_c2_cmu_settle_ms;
extern int rtl960x_c2_clkgate_rstb;
extern int rtl960x_c2_skip_rstb_dance;
extern int rtl960x_c2_minimal_analog;
#endif /* _RTL960X_PONMAC_H */
