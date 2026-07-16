// SPDX-License-Identifier: GPL-2.0
/*
 * Semtech GN25L95 BOSA (burst-mode laser driver + APD/TIA) boot initialization
 * for the Realtek RTL9607F "Elnath" GPON ONU.
 *
 * The external BOSA sits on the SoC per_i2c bus at slave 0x51 (the SFF-8472
 * "A2h" diagnostic address).  Out of power-on reset the laser driver is
 * unprogrammed: the bias/modulation/APD DAC tables are empty and TX is gated,
 * so the ONU never bursts upstream and the OLT reports "Laser out".  The
 * vendor programs it on every boot from userspace ("rtkbosa", fed by the
 * per-board /var/config/rtkbosa_k.bin calibration file); this file reproduces
 * that init in-kernel so the laser is ready BEFORE the ranging FSM starts.
 *
 * Register model (verified on-wire against the live stock init, ftrace i2c
 * capture dev/x411axf/stock/bosa/rtkbosa_i2c_trace.txt — our sequence below
 * byte-matches all 638 init writes):
 *   - all writes are plain 2-byte {reg, val}; reads = reg pointer + 1 byte
 *   - regs 0x00-0x7F are un-paged; 0x80-0xFF are paged via table-select 0x7F
 *   - no inter-write delay is required (stock runs back-to-back at 100 kHz)
 *
 * The essential laser gate at the end of the sequence: TX_CTL(0x6E) bit6 = 0
 * (soft TX-disable cleared, after a 1->0 reset pulse) and PON_CTL(0x6F) = 0.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>

#include "cortina-i2c.h"
#include "cortina-gpon-bosa.h"
#include "cortina-gpon-bosa-cal.h"
#include "cortina-gpon-bosa-seq.h"

#define BOSA_I2C_ADDR		0x51	/* SFF-8472 A2h address */

/* GN25L95 registers (un-paged, 0x00-0x7F window) */
#define BOSA_REG_TX_CTL		0x6e	/* bit6 = soft TX-disable */
#define BOSA_REG_PON_CTL	0x6f	/* 0x00 = burst TX enabled */
#define BOSA_REG_PON_CTL2	0x72
#define BOSA_REG_PON_CTL3	0x78
#define BOSA_REG_PON_CTL4	0x79
#define BOSA_REG_PASSWD		0x7b	/* 0x7b-0x7e, unlock = all 0xFF */
#define BOSA_REG_PAGE		0x7f	/* table select for 0x80-0xFF */

#define BOSA_TX_SOFT_DIS	0x40	/* TX_CTL bit6 */

/* paged registers (table.reg) */
#define BOSA_T1_ALARM_EN	0xf8	/* alarm enables (0xf8/0xf9) */
#define BOSA_T1_WARN_EN		0xfc	/* warning enables (0xfc/0xfd) */
#define BOSA_T2_SAFE_MODE	0xa0	/* safe-mode start-up */
#define BOSA_T2_PASSWD_LVL	0xbb	/* bit0 = password level */
#define BOSA_SAFE_MODE_START	0x6a

/* pages (tables) of the 0x80-0xFF window */
#define BOSA_PAGE_ALARM		0x01	/* table 1: alarm/warning enables */
#define BOSA_PAGE_DEVICE	0x02	/* table 2: device settings, slopes/offsets */
#define BOSA_PAGE_BIAS_LUT	0x04	/* table 4: bias-DAC LUT */
#define BOSA_PAGE_MOD_LUT	0x05	/* table 5: modulation-DAC LUT */
#define BOSA_PAGE_APD_LUT	0x06	/* table 6: APD LUT */

/*
 * Calibration-image addressing: cg_bosa_cal[reg + delta].
 *   un-paged reg r (0x00-0xFF)          -> cal[r + 0x100]
 *   table 2 reg r (0x80-0xFF)           -> cal[r + 0x180]  (0x200-0x27f)
 *   table 4/5/6 reg r (0x80-0xFF)       -> cal[r + 0x200/0x280/0x300]
 */
#define BOSA_CAL_A2		0x100
#define BOSA_CAL_T2		0x180
#define BOSA_CAL_T4		0x200
#define BOSA_CAL_T5		0x280
#define BOSA_CAL_T6		0x300

static int bosa_wr(struct device *dev, u8 reg, u8 val)
{
	int ret = cg_i2c_write_byte(BOSA_I2C_ADDR, reg, val);

	if (ret)
		dev_err(dev, "BOSA: write reg 0x%02x=0x%02x failed (%d)\n",
			reg, val, ret);
	return ret;
}

static int bosa_rd(struct device *dev, u8 reg, u8 *val)
{
	int ret = cg_i2c_read_byte(BOSA_I2C_ADDR, reg, val);

	if (ret)
		dev_err(dev, "BOSA: read reg 0x%02x failed (%d)\n", reg, ret);
	return ret;
}

/*
 * Program the BOSA exactly as the stock boot does: a VERBATIM replay of the
 * stock rtkbosa on-wire i2c write sequence (684 writes, captured with
 * timestamps; continuous, no delays needed — see cortina-gpon-bosa-seq.h).
 *
 * The earlier step-wise re-expression (LUT pages 4/5/6 + device page 2 +
 * alarm page 1 + unpaged thresholds) MISSED whole sections of the stock
 * sequence — pages 0x00/0x03/0x86/0x87/0xff never got programmed — and the
 * laser never armed (TX_CTL 0x6e stuck at 0x80, zero TX bias, OLT saw zero
 * upstream) even with every GPIO/pin-route register byte-matching stock.
 * Replay first, re-express into named steps only after ranging is proven.
 *
 * Idempotent; call before the GPON ranging FSM is enabled.
 */
int cg_bosa_init(struct device *dev)
{
	unsigned int i;
	u8 v;
	int ret;

	ret = cg_i2c_init(dev);
	if (ret)
		return ret;

	/* presence check: the GN25L95 must ACK before we stream 600+ writes */
	ret = cg_i2c_read_byte(BOSA_I2C_ADDR, BOSA_REG_TX_CTL, &v);
	if (ret) {
		dev_err(dev, "BOSA: GN25L95 not responding at 0x%02x (%d) - laser stays unprogrammed\n",
			BOSA_I2C_ADDR, ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(cg_bosa_stock_seq); i++) {
		ret = bosa_wr(dev, cg_bosa_stock_seq[i][0],
			      cg_bosa_stock_seq[i][1]);
		if (ret) {
			dev_err(dev, "BOSA: stock-seq write %u/%zu (reg 0x%02x) failed (%d)\n",
				i, ARRAY_SIZE(cg_bosa_stock_seq),
				cg_bosa_stock_seq[i][0], ret);
			return ret;
		}
	}

	/*
	 * Read-back verify: the sequence ends with page 2 selected and 0xbb
	 * written 0x1c.  An UNROUTED i2c0 pinmux fake-ACKs every write and
	 * reads back 0x00 — this catches the dead bus loud on every boot
	 * instead of silently leaving the BOSA unprogrammed.
	 */
	ret = bosa_rd(dev, BOSA_T2_PASSWD_LVL, &v);
	if (ret)
		return ret;
	if (v != 0x1c) {
		dev_err(dev, "BOSA: post-replay verify failed (0xbb=0x%02x, want 0x1c) - I2C bus dead / not programmed (pinmux?)\n",
			v);
		return -EIO;
	}

	/* report the laser gate state: TX_CTL bit6 == 0, PON_CTL == 0 */
	{
		u8 tx = 0xff, pon = 0xff;

		bosa_rd(dev, BOSA_REG_TX_CTL, &tx);
		bosa_rd(dev, BOSA_REG_PON_CTL, &pon);
		dev_info(dev, "BOSA: GN25L95 programmed (stock replay, %zu writes), tx_ctl=0x%02x pon_ctl=0x%02x\n",
			 ARRAY_SIZE(cg_bosa_stock_seq), tx, pon);
		if ((tx & BOSA_TX_SOFT_DIS) || pon)
			dev_warn(dev, "BOSA: laser gate NOT open after init\n");
	}
	return 0;
}

/*
 * One-shot full GN25L95 register dump to dmesg (`echo 'bosa dump' >
 * /proc/gpon`), for diffing the cold power-on BOSA state against the stock
 * golden (the cold-vs-warm DS-RX/LOF investigation: config the stock daemon
 * programs survives a warm reboot inside the BOSA but is lost on a
 * power-cycle).  Read-only apart from the table-select register 0x7F, which
 * is saved and restored.  Reads 0x00-0xFF of every table the stock daemon
 * touches (0-6 plus the high tables 0x80/0x81/0x86/0x87/0xFF seen in the
 * rtkbosa write trace); 0x00-0x7F is un-paged so it repeats per page — a
 * free consistency check against bus noise.  ~3k reads at 100 kHz ≈ 1.5 s,
 * one shot, never touches the PON engine.
 */
int cg_bosa_dump(struct device *dev)
{
	static const u8 pages[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
				    0x80, 0x81, 0x86, 0x87, 0xff };
	char line[3 * 16 + 1];
	u8 curpage = 0xee, v;
	unsigned int p, r, i, rd_fail = 0;
	int ret;

	ret = cg_i2c_init(dev);
	if (ret)
		return ret;

	ret = bosa_rd(dev, BOSA_REG_PAGE, &curpage);
	if (ret) {
		dev_err(dev, "BOSA dump: no ACK from 0x%02x (%d)\n",
			BOSA_I2C_ADDR, ret);
		return ret;
	}
	dev_info(dev, "BOSADUMP start, curpage=0x%02x\n", curpage);

	for (p = 0; p < ARRAY_SIZE(pages); p++) {
		ret = bosa_wr(dev, BOSA_REG_PAGE, pages[p]);
		if (ret)
			break;
		for (r = 0; r < 0x100; r += 16) {
			for (i = 0; i < 16; i++) {
				if (cg_i2c_read_byte(BOSA_I2C_ADDR, r + i, &v)) {
					rd_fail++;
					sprintf(line + 3 * i, " ??");
				} else {
					sprintf(line + 3 * i, " %02x", v);
				}
			}
			dev_info(dev, "BOSADUMP p%02x %02x:%s\n",
				 pages[p], r, line);
		}
	}

	bosa_wr(dev, BOSA_REG_PAGE, curpage);
	dev_info(dev, "BOSADUMP %s, rd_fail=%u (page restored to 0x%02x)\n",
		 ret ? "FAILED" : "done", rd_fail, curpage);
	return ret;
}

/*
 * Always-on spy hook for /proc/gpon: read back the laser-gate registers live
 * (un-paged only — never flips the table-select out from under nothing, but
 * stay conservative).  Laser bursts when tx_ctl bit6 == 0 and pon_ctl == 0.
 */
void cg_bosa_proc_show(struct device *dev, struct seq_file *m)
{
	u8 tx = 0xff, pon = 0xff;

	if (cg_i2c_read_byte(BOSA_I2C_ADDR, BOSA_REG_TX_CTL, &tx) ||
	    cg_i2c_read_byte(BOSA_I2C_ADDR, BOSA_REG_PON_CTL, &pon)) {
		seq_puts(m, "bosa           = i2c read FAILED\n");
		return;
	}
	seq_printf(m, "bosa           = tx_ctl(6e)=0x%02x pon_ctl(6f)=0x%02x  (laser gate open = bit6:0 / 0x00)\n",
		   tx, pon);
}
