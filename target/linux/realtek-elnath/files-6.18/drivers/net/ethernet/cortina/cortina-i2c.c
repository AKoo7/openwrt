// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal poll-mode MMIO I2C master for the Cortina-Access ca77xx "BIW"
 * (bus-interface-wrapper) I2C controller of the Realtek RTL9607F "Elnath"
 * GPON SoC — per_i2c bus 0 @ phys 0xf4329170 (stock DT: i2c@f4329170,
 * compatible "cortina,ca77xx-i2c"; a second instance sits at 0xf4329198).
 *
 * The controller is an OpenCores-style single-byte engine: one data byte per
 * TXR/RXR command, a CTRL register carrying the start/stop/read/write/ack-in
 * strobes plus a transfer-done flag, and an ACK register with arbitration-
 * lost / bus-busy / RxACK status.  SCL prescaler: prer = pclk/(5*SCL) - 1
 * with pclk = 125 MHz APB ("g3_apb_pclk") -> 249 for the 100 kHz standard
 * mode the stock BOSA init runs at.  No FIFO, no quirks; clock stretching
 * disabled (matches the stock configuration).
 *
 * Poll-mode only, NOT wired to the Linux I2C subsystem, on purpose:
 *   - the only in-kernel user is the GN25L95 BOSA laser-driver programming,
 *     which must run deterministically BEFORE the GPON ranging FSM starts,
 *     inside the cortina-gpon probe.  drivers/net links ahead of drivers/i2c,
 *     so an i2c-subsystem client could not guarantee that ordering without
 *     probe-deferral gymnastics;
 *   - the lean per-model kernel carries no other I2C user, so the subsystem
 *     would cost image size for a single slave;
 *   - a byte at 100 kHz completes in ~90 us, so polling is cheap and it only
 *     runs at bring-up + on /proc/gpon reads.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>

#include "cortina-i2c.h"

/* per_i2c bus 0 (the BOSA bus, /dev/i2c-0 on stock) */
#define CGI2C_PHYS		0xf4329170ULL
#define CGI2C_SIZE		0x28

/*
 * GLB pinmux: the i2c0 SCL/SDA pads must be routed to the BIW engine or every
 * transfer silently no-ops (unrouted pins float, the ACK line reads low = fake
 * ACK, reads return 0x00 — the exact cold-boot "BOSA all-zero / DS LOF / FSM
 * stuck O1" failure).  Live-verified: cold power-on reset value 0x00100000,
 * stock-running value 0x00110000 — bit16 = the i2c0 pin group.  Stock routes
 * it from userspace before rtkbosa runs; we must do it in-kernel before the
 * first BIW transfer.
 */
#define CGI2C_PINMUX_PHYS	0xf4320430ULL	/* GLB 0xf4320000 + 0x430 */
#define CGI2C_PINMUX_I2C0_EN	BIT(16)		/* route the i2c0 pin group */

#define CGI2C_PCLK_HZ		125000000	/* g3_apb_pclk */
#define CGI2C_BUS_HZ		100000		/* standard mode, as stock */

/* BIW register block (all 32-bit) */
#define BIW_CFG			0x00
#define   BIW_CFG_CORE_EN	BIT(0)
#define   BIW_CFG_SOFT_RESET	BIT(1)
#define   BIW_CFG_PRER_SHIFT	16	/* prer[31:16] = pclk/(5*SCL) - 1 */
#define BIW_CTRL		0x04
#define   BIW_CTRL_DONE		BIT(0)	/* transfer done, write-back to clear */
#define   BIW_CTRL_ACK_IN	BIT(3)	/* master drives NAK on the read byte */
#define   BIW_CTRL_WRITE	BIT(4)
#define   BIW_CTRL_READ		BIT(5)
#define   BIW_CTRL_STOP		BIT(6)
#define   BIW_CTRL_START	BIT(7)
#define BIW_TXR			0x08
#define BIW_RXR			0x0c
#define BIW_ACK			0x10
#define   BIW_ACK_AL		BIT(0)	/* arbitration lost */
#define   BIW_ACK_BUSY		BIT(1)	/* bus busy */
#define   BIW_ACK_NAK		BIT(2)	/* ack_out: 1 = slave NAKed */
#define BIW_IE0			0x14	/* left 0: poll mode, no IRQ routed */
#define BIW_INT0		0x18
#define BIW_IE1			0x1c
#define BIW_INT1		0x20
#define BIW_STAT		0x24

#define CGI2C_BYTE_TIMEOUT_US	10000	/* one byte at 100 kHz is ~90 us */
#define CGI2C_BUSY_TIMEOUT_US	100000

static void __iomem *cgi2c_base;
static DEFINE_MUTEX(cgi2c_lock);

/* Issue one command strobe and wait for the engine's done flag. */
static int cgi2c_cmd(u32 ctrl)
{
	u32 v;
	int ret;

	writel(ctrl, cgi2c_base + BIW_CTRL);
	ret = readl_poll_timeout(cgi2c_base + BIW_CTRL, v, v & BIW_CTRL_DONE,
				 10, CGI2C_BYTE_TIMEOUT_US);
	if (ret)
		return ret;
	writel(v, cgi2c_base + BIW_CTRL);	/* write-back clears done */

	v = readl(cgi2c_base + BIW_ACK);
	if (v & BIW_ACK_AL)
		return -EAGAIN;
	/* the slave ACK only matters on address/write phases */
	if (!(ctrl & BIW_CTRL_READ) && (v & BIW_ACK_NAK))
		return -ENXIO;
	return 0;
}

/* Best-effort bus release after a failed phase, so the next xfer can start. */
static void cgi2c_stop(void)
{
	cgi2c_cmd(BIW_CTRL_STOP);
}

static int cgi2c_wait_idle(void)
{
	u32 v;

	return readl_poll_timeout(cgi2c_base + BIW_ACK, v, !(v & BIW_ACK_BUSY),
				  10, CGI2C_BUSY_TIMEOUT_US);
}

/* Address phase: START (or repeated START) + slave address byte. */
static int cgi2c_start(u8 addr, bool read)
{
	writel((addr << 1) | (read ? 1 : 0), cgi2c_base + BIW_TXR);
	return cgi2c_cmd(BIW_CTRL_START | BIW_CTRL_WRITE);
}

/* Plain 2-byte {reg, val} write — the only write shape the BOSA uses. */
int cg_i2c_write_byte(u8 addr, u8 reg, u8 val)
{
	int ret;

	if (!cgi2c_base)
		return -ENODEV;

	mutex_lock(&cgi2c_lock);
	ret = cgi2c_wait_idle();
	if (ret)
		goto out;
	ret = cgi2c_start(addr, false);
	if (ret)
		goto out_stop;
	writel(reg, cgi2c_base + BIW_TXR);
	ret = cgi2c_cmd(BIW_CTRL_WRITE);
	if (ret)
		goto out_stop;
	writel(val, cgi2c_base + BIW_TXR);
	ret = cgi2c_cmd(BIW_CTRL_WRITE | BIW_CTRL_STOP);
	goto out;

out_stop:
	cgi2c_stop();
out:
	mutex_unlock(&cgi2c_lock);
	return ret;
}

/*
 * Register read: reg-pointer write, then a repeated-START 1-byte read with
 * NAK+STOP on the data byte (the exact shape of the stock i2c-0 trace).
 */
int cg_i2c_read_byte(u8 addr, u8 reg, u8 *val)
{
	int ret;

	if (!cgi2c_base)
		return -ENODEV;

	mutex_lock(&cgi2c_lock);
	ret = cgi2c_wait_idle();
	if (ret)
		goto out;
	ret = cgi2c_start(addr, false);
	if (ret)
		goto out_stop;
	writel(reg, cgi2c_base + BIW_TXR);
	ret = cgi2c_cmd(BIW_CTRL_WRITE);
	if (ret)
		goto out_stop;
	ret = cgi2c_start(addr, true);		/* repeated START, read */
	if (ret)
		goto out_stop;
	ret = cgi2c_cmd(BIW_CTRL_READ | BIW_CTRL_ACK_IN | BIW_CTRL_STOP);
	if (ret)
		goto out;
	*val = readl(cgi2c_base + BIW_RXR) & 0xff;
	goto out;

out_stop:
	cgi2c_stop();
out:
	mutex_unlock(&cgi2c_lock);
	return ret;
}

/*
 * Map + initialize the controller: soft-reset if it was left enabled (warm
 * boot / bootloader use), program the SCL prescaler, enable the core.
 * Idempotent; interrupts stay off (poll mode).
 */
int cg_i2c_init(struct device *dev)
{
	u32 prer = CGI2C_PCLK_HZ / (5 * CGI2C_BUS_HZ) - 1;
	void __iomem *pinmux;
	u32 v;

	if (cgi2c_base)
		return 0;

	/* route the i2c0 pads BEFORE any BIW transfer (see CGI2C_PINMUX_PHYS) */
	pinmux = devm_ioremap(dev, CGI2C_PINMUX_PHYS, sizeof(u32));
	if (!pinmux) {
		dev_err(dev, "cortina-i2c: cannot map pinmux @ 0x%llx\n",
			CGI2C_PINMUX_PHYS);
		return -ENOMEM;
	}
	v = readl(pinmux);
	if (!(v & CGI2C_PINMUX_I2C0_EN)) {
		writel(v | CGI2C_PINMUX_I2C0_EN, pinmux);
		dev_info(dev, "cortina-i2c: routed i2c0 pinmux (0x%08x -> 0x%08x)\n",
			 v, readl(pinmux));
	}
	devm_iounmap(dev, pinmux);

	cgi2c_base = devm_ioremap(dev, CGI2C_PHYS, CGI2C_SIZE);
	if (!cgi2c_base) {
		dev_err(dev, "cortina-i2c: cannot map BIW @ 0x%llx\n", CGI2C_PHYS);
		return -ENOMEM;
	}

	v = readl(cgi2c_base + BIW_CFG);
	if (v & BIW_CFG_CORE_EN) {
		writel(v | BIW_CFG_SOFT_RESET, cgi2c_base + BIW_CFG);
		msleep(50);
		writel(v & ~BIW_CFG_SOFT_RESET, cgi2c_base + BIW_CFG);
	}
	writel((prer << BIW_CFG_PRER_SHIFT) | BIW_CFG_CORE_EN,
	       cgi2c_base + BIW_CFG);
	msleep(50);
	writel(0, cgi2c_base + BIW_IE0);
	writel(0, cgi2c_base + BIW_IE1);

	dev_info(dev, "cortina-i2c: BIW @ 0x%llx up, %d kHz (prer=%u)\n",
		 CGI2C_PHYS, CGI2C_BUS_HZ / 1000, prer);
	return 0;
}
