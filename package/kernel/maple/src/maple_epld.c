// SPDX-License-Identifier: GPL-2.0
/*
 * maple-epld — board EPLD control via I2C (open replacement for the EPLD
 * parts of vendor i2c_devs.ko). Loads early, releases the Maple GPON MAC
 * from reset, then triggers a PCIe bus rescan so the maple.ko PCI driver
 * can bind to the now-visible BCM6862x device.
 *
 * EPLD topology (from docs/decomp/i2c_devs.md):
 *   i2c-0 → PCA9548A mux @0x70 → channel 1 → EPLD/FPGA @0x40
 * The EPLD is a 32-bit register space reached via a 4-byte BE I2C address.
 * Register offset 2 = system RESET register (bit-level per-device reset control).
 *
 * The vendor's reset_maple() sequence:
 *   1. write 0x01 to mux @0x70 (select channel 1)
 *   2. read 4 bytes BE from EPLD @0x40, reg 2 (reset register)
 *   3. clear the Maple MAC reset bit
 *   4. write 4 bytes BE back
 *
 * HW-unverified — the I2C bus controller (iProc I2C @0x18047200) DTS node
 * must be correct for this driver to probe.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#define EPLD_MUX_ADDR		0x70
#define EPLD_MUX_CHANNEL	0x01	/* PCA9548A channel 1 */
#define EPLD_ADDR		0x40
#define EPLD_RESET_REG		2	/* 32-bit register offset 2 = system reset */
#define MAPLE_RESET_BIT		0	/* bit 0 in the reset register = Maple MAC */

/* Default I2C bus number (the iProc I2C controller). Override via module param. */
static int i2c_bus = 0;
module_param(i2c_bus, int, 0444);
MODULE_PARM_DESC(i2c_bus, "I2C bus number for the EPLD (default 0)");

/* Select the PCA9548A mux channel that gates the EPLD. */
static int epld_select_mux(struct i2c_adapter *adap)
{
	struct i2c_msg msg;
	u8 ch = EPLD_MUX_CHANNEL;

	msg.addr = EPLD_MUX_ADDR;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &ch;
	return i2c_transfer(adap, &msg, 1) == 1 ? 0 : -EIO;
}

/* Read a 32-bit EPLD register (4-byte BE address → 4-byte BE data). */
static int epld_read32(struct i2c_adapter *adap, u32 reg, u32 *val)
{
	struct i2c_msg msg[2];
	u8 addr_be[4];
	u8 data_be[4];
	int ret;

	addr_be[0] = (reg >> 24) & 0xff;
	addr_be[1] = (reg >> 16) & 0xff;
	addr_be[2] = (reg >> 8)  & 0xff;
	addr_be[3] =  reg        & 0xff;

	msg[0].addr  = EPLD_ADDR;
	msg[0].flags = 0;
	msg[0].len   = 4;
	msg[0].buf   = addr_be;

	msg[1].addr  = EPLD_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 4;
	msg[1].buf   = data_be;

	ret = i2c_transfer(adap, msg, 2);
	if (ret != 2)
		return -EIO;
	*val = (data_be[0] << 24) | (data_be[1] << 16) |
	       (data_be[2] << 8)  | data_be[3];
	return 0;
}

/* Write a 32-bit EPLD register (4-byte BE addr + 4-byte BE data = 8 bytes). */
static int epld_write32(struct i2c_adapter *adap, u32 reg, u32 val)
{
	struct i2c_msg msg;
	u8 buf[8];

	buf[0] = (reg >> 24) & 0xff;
	buf[1] = (reg >> 16) & 0xff;
	buf[2] = (reg >> 8)  & 0xff;
	buf[3] =  reg        & 0xff;
	buf[4] = (val >> 24) & 0xff;
	buf[5] = (val >> 16) & 0xff;
	buf[6] = (val >> 8)  & 0xff;
	buf[7] =  val        & 0xff;

	msg.addr  = EPLD_ADDR;
	msg.flags = 0;
	msg.len   = 8;
	msg.buf   = buf;
	return i2c_transfer(adap, &msg, 1) == 1 ? 0 : -EIO;
}

/* Release the Maple GPON MAC from reset (the critical bring-up step). */
static int maple_epld_release_maple(struct i2c_adapter *adap)
{
	u32 reset_reg;
	int ret;

	ret = epld_select_mux(adap);
	if (ret) {
		pr_err("failed to select EPLD mux: %d\n", ret);
		return ret;
	}

	ret = epld_read32(adap, EPLD_RESET_REG, &reset_reg);
	if (ret) {
		pr_err("failed to read EPLD reset register: %d\n", ret);
		return ret;
	}
	pr_info("EPLD reset register = %#010x\n", reset_reg);

	/* Deassert the Maple MAC reset bit (set bit → reset, clear bit → run).
	 * The vendor's reset_maple(bit=0, param=1) SETS the bit to assert reset;
	 * param=0 CLEARS the bit to release. We want to RELEASE. */
	reset_reg &= ~(1u << MAPLE_RESET_BIT);

	ret = epld_write32(adap, EPLD_RESET_REG, reset_reg);
	if (ret) {
		pr_err("failed to write EPLD reset register: %d\n", ret);
		return ret;
	}
	pr_info("Maple MAC released from reset\n");
	return 0;
}

static int __init maple_epld_init(void)
{
	struct i2c_adapter *adap;
	int ret;

	adap = i2c_get_adapter(i2c_bus);
	if (!adap) {
		pr_err("I2C bus %d not found — is i2c-bcm-iproc loaded?\n", i2c_bus);
		return -ENODEV;
	}

	ret = maple_epld_release_maple(adap);
	i2c_put_adapter(adap);
	if (ret)
		return ret;

	/* Wait for the Maple MAC's PCIe link to stabilize after reset release. */
	msleep(200);

	/* Trigger a PCIe bus rescan so the Maple MAC is enumerated and maple.ko
	 * can bind to it. */
	pci_rescan_bus(0);

	pr_info("PCIe rescan triggered — waiting for Maple MAC enumeration\n");
	return 0;
}

static void __exit maple_epld_exit(void)
{
	/* Nothing to do — the MAC stays out of reset. */
}

module_init(maple_epld_init);
module_exit(maple_epld_exit);

MODULE_AUTHOR("hg08-olt-re");
MODULE_DESCRIPTION("HSGQ HG08 board EPLD control + Maple MAC reset release");
MODULE_LICENSE("GPL");
