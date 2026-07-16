// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access QSPI flash controller ("cortina,ca-qspi") — spi-mem driver
 * for the Realtek RTL9607F "Elnath" GPON ONU SoC (Cortina CA8277C "TAURUS").
 *
 * Small PIO engine, one 0x50-byte register window ("qspi-base", on the
 * X411AXF at 0x4_F4324000), no interrupts, no clock gate to manage (the
 * block is always clocked; the boot ROM/U-Boot load from it).  Serves the
 * board's SPI-NAND (Fudan FM25S01A on this board — supported by the
 * mainline SPI-NAND core) through the generic "spi-nand" child node.
 *
 * On this platform the flash holds the STOCK firmware plus the per-board
 * factory-provisioning UBI volume (ubi_Config: config_hs.xml with
 * ELAN_MAC_ADDR / GPON_SN).  Our OpenWrt runs from RAM and only ever needs
 * to READ that data, and the DTS marks every partition read-only — but the
 * controller itself implements the full op set (write kicks are also what
 * carry SET-FEATURE etc. to the chip's volatile config registers).
 *
 * Programming model (register/bit facts confirmed against the live board:
 * this exact sequence is what the stock 5.10 kernel drives; stock dmesg
 * probes it as "ca-qspi ... mode_bits=0x0000" and boots from it):
 *
 *   A flash command is described by an "access code" (a hardware sequence
 *   selector, ACCESS[11:8]) plus the flash opcode.  Three shapes cover the
 *   whole SPI-NAND op set:
 *     0x0  opcode only                      (RESET, WRITE ENABLE, ...)
 *     0x5  opcode + address, no data        (PAGE READ, PROGRAM EXEC, ERASE)
 *     0xF  "extended": opcode + optional address/dummy + data phase, the
 *          geometry given in EXT_ACCESS {opcode, dummy-1, addr-1, count-1}
 *   Data moves 32 bits at a time through DATA (little-endian byte order):
 *   each word is clocked by writing the start bit (plus the write-access
 *   bit for output) to BUSY and polling the busy bits clear.
 *
 * TAURUS (CA8277C) specifics baked in (this driver targets the RTL9607F):
 *   - BUSY completion polls bits {16,1} clear (older chips: bit 1 only);
 *   - EXT_ACCESS data count is programmed with the FULL transfer length
 *     (older chips clamp the field to the 4-byte burst window);
 *   - the TIMING register is left at reset (older chips program it).
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/* register window */
#define CA_QSPI_TYPE		0x0c	/* transfer geometry */
#define CA_QSPI_BUSY		0x10	/* kick + completion status */
#define CA_QSPI_ACCESS		0x30	/* opcode / access-code / IO width */
#define CA_QSPI_EXT_ACCESS	0x34	/* extended-op geometry */
#define CA_QSPI_ADDR		0x38	/* flash address */
#define CA_QSPI_DATA		0x3c	/* 32-bit PIO data */

/* TYPE */
#define CA_QSPI_TYPE_SIZE_MASK	GENMASK(10, 9)	/* address size code: 0 = 3 bytes, 2 = 4 bytes */
#define CA_QSPI_TYPE_SIZE_4B	(2 << 9)

/* BUSY */
#define CA_QSPI_BUSY_START	BIT(1)
#define CA_QSPI_BUSY_WR		BIT(9)	/* this kick moves data OUT */
/* TAURUS: completion = both bits clear */
#define CA_QSPI_BUSY_READY_MASK	(BIT(16) | CA_QSPI_BUSY_START)

/* ACCESS */
#define CA_QSPI_ACC_OPCODE(op)	((op) & GENMASK(7, 0))
#define CA_QSPI_ACC_CODE(ac)	(((ac) << 8) & GENMASK(11, 8))
#define CA_QSPI_ACC_FORCE_TERM	BIT(12)
#define CA_QSPI_ACC_FORCE_BURST	BIT(13)
#define CA_QSPI_ACC_MIO_DATA	BIT(24)	/* multi-IO applies to the data phase */
#define CA_QSPI_ACC_MIO_ADDR	BIT(25)	/* ... to the address phase */
#define CA_QSPI_ACC_MIO_CMD	BIT(26)	/* ... to the command phase */
#define CA_QSPI_ACC_MIO_WIDTH(w)	(((w) << 30) & GENMASK(31, 30))	/* 0/1/2 = single/dual/quad */

/* access codes (hardware sequence selectors) */
#define CA_QSPI_AC_OP		0x0	/* bare opcode */
#define CA_QSPI_AC_OP_ADDR	0x5	/* opcode + address */
#define CA_QSPI_AC_EXTENDED	0xf	/* geometry from EXT_ACCESS */

/*
 * EXT_ACCESS: all three count fields are programmed as COUNT-1 and simply
 * wrap into their mask when the phase is absent (a 0-dummy op programs the
 * dummy field as 0x3f, a no-address op programs the address field as 7).
 * That wrap is the vendor-programmed, silicon-proven encoding for "phase
 * absent" on this hardware — do not "fix" it to 0.
 */
#define CA_QSPI_EXT_OPCODE(op)	((op) & GENMASK(7, 0))
#define CA_QSPI_EXT_DATA(n)	(((n) << 8) & GENMASK(20, 8))
#define CA_QSPI_EXT_ADDR(n)	(((n) << 21) & GENMASK(23, 21))
#define CA_QSPI_EXT_DUMMY(n)	(((n) << 24) & GENMASK(29, 24))
#define CA_QSPI_EXT_ENABLE	BIT(31)

/* one chunk of the PIO data engine (also the EXT_ACCESS data-field ceiling
 * we advertise via adjust_op_size, so spi-mem splits anything bigger) */
#define CA_QSPI_MAX_DATA	2048

/* generous per-word completion cap; a word normally completes in < 1 us */
#define CA_QSPI_POLL_TIMEOUT_US	200000

struct ca_qspi {
	struct device *dev;
	void __iomem *base;
};

/* kick one controller transaction and wait for it to retire */
static int ca_qspi_kick(struct ca_qspi *qspi, bool data_out)
{
	u32 stat;

	writel(data_out ? CA_QSPI_BUSY_WR | CA_QSPI_BUSY_START :
			  CA_QSPI_BUSY_START,
	       qspi->base + CA_QSPI_BUSY);
	return readl_poll_timeout_atomic(qspi->base + CA_QSPI_BUSY, stat,
					 !(stat & CA_QSPI_BUSY_READY_MASK),
					 0, CA_QSPI_POLL_TIMEOUT_US);
}

static int ca_qspi_read_words(struct ca_qspi *qspi, u8 *buf, unsigned int len)
{
	while (len) {
		unsigned int n = min_t(unsigned int, len, 4);
		int ret = ca_qspi_kick(qspi, false);
		u32 data;

		if (ret)
			return ret;
		data = readl(qspi->base + CA_QSPI_DATA);
		len -= n;
		while (n--) {
			*buf++ = data & 0xff;
			data >>= 8;
		}
	}
	return 0;
}

static int ca_qspi_write_words(struct ca_qspi *qspi, const u8 *buf,
			       unsigned int len)
{
	while (len) {
		unsigned int n = min_t(unsigned int, len, 4);
		u32 data = 0;
		int i, ret;

		for (i = 0; i < n; i++)
			data |= (u32)buf[i] << (8 * i);
		buf += n;
		len -= n;
		writel(data, qspi->base + CA_QSPI_DATA);
		ret = ca_qspi_kick(qspi, true);
		if (ret)
			return ret;
	}
	return 0;
}

/* extended op: opcode + optional address/dummy + a data phase */
static int ca_qspi_exec_extended(struct ca_qspi *qspi,
				 const struct spi_mem_op *op)
{
	unsigned int dummy_clks = op->dummy.buswidth ?
		(op->dummy.nbytes * 8) / op->dummy.buswidth :
		op->dummy.nbytes * 8;
	bool burst = op->data.nbytes > 4;
	u32 val;
	int ret;

	writel(CA_QSPI_ACC_CODE(CA_QSPI_AC_EXTENDED),
	       qspi->base + CA_QSPI_ACCESS);

	/* multi-IO phase widths (vendor quirk kept: only re-programmed for
	 * transfers > 6 bytes; shorter ones run on the just-cleared = single
	 * setting, which is also all this board uses) */
	if (op->data.nbytes > 6) {
		u32 width;

		switch (op->data.buswidth) {
		case 1:
			width = CA_QSPI_ACC_MIO_WIDTH(0);
			break;
		case 2:
			width = CA_QSPI_ACC_MIO_WIDTH(1) | CA_QSPI_ACC_MIO_DATA;
			break;
		case 4:
			width = CA_QSPI_ACC_MIO_WIDTH(2) | CA_QSPI_ACC_MIO_DATA;
			break;
		default:
			return -EOPNOTSUPP;
		}
		if (op->addr.buswidth > 1)
			width |= CA_QSPI_ACC_MIO_ADDR;
		if (op->cmd.buswidth > 1)
			width |= CA_QSPI_ACC_MIO_CMD;
		val = readl(qspi->base + CA_QSPI_ACCESS);
		val &= ~(CA_QSPI_ACC_MIO_WIDTH(3) | CA_QSPI_ACC_MIO_DATA |
			 CA_QSPI_ACC_MIO_ADDR | CA_QSPI_ACC_MIO_CMD);
		writel(val | width, qspi->base + CA_QSPI_ACCESS);
	}

	/* hold CS across the whole multi-word data phase */
	if (burst) {
		val = readl(qspi->base + CA_QSPI_ACCESS);
		val &= ~CA_QSPI_ACC_FORCE_TERM;
		writel(val | CA_QSPI_ACC_FORCE_BURST,
		       qspi->base + CA_QSPI_ACCESS);
	}

	writel(0, qspi->base + CA_QSPI_EXT_ACCESS);
	writel(CA_QSPI_EXT_OPCODE(op->cmd.opcode) |
	       CA_QSPI_EXT_DUMMY(dummy_clks - 1) |
	       CA_QSPI_EXT_ADDR(op->addr.nbytes - 1) |
	       CA_QSPI_EXT_DATA(op->data.nbytes - 1) |	/* TAURUS: full length */
	       CA_QSPI_EXT_ENABLE,
	       qspi->base + CA_QSPI_EXT_ACCESS);

	if (op->addr.nbytes)
		writel((u32)op->addr.val, qspi->base + CA_QSPI_ADDR);

	if (op->data.dir == SPI_MEM_DATA_IN)
		ret = ca_qspi_read_words(qspi, op->data.buf.in,
					 op->data.nbytes);
	else
		ret = ca_qspi_write_words(qspi, op->data.buf.out,
					  op->data.nbytes);

	if (burst) {
		val = readl(qspi->base + CA_QSPI_ACCESS);
		val &= ~CA_QSPI_ACC_FORCE_BURST;
		writel(val | CA_QSPI_ACC_FORCE_TERM,
		       qspi->base + CA_QSPI_ACCESS);
	}
	writel(0, qspi->base + CA_QSPI_ACCESS);
	return ret;
}

/* dataless op, with or without an address phase */
static int ca_qspi_exec_simple(struct ca_qspi *qspi,
			       const struct spi_mem_op *op)
{
	u8 ac = op->addr.nbytes ? CA_QSPI_AC_OP_ADDR : CA_QSPI_AC_OP;
	int ret;

	writel(CA_QSPI_ACC_CODE(ac) | CA_QSPI_ACC_OPCODE(op->cmd.opcode),
	       qspi->base + CA_QSPI_ACCESS);

	if (op->addr.nbytes) {
		u32 val = readl(qspi->base + CA_QSPI_TYPE);

		val &= ~CA_QSPI_TYPE_SIZE_MASK;
		if (op->addr.nbytes > 3)
			val |= CA_QSPI_TYPE_SIZE_4B;
		writel(val, qspi->base + CA_QSPI_TYPE);
		writel((u32)op->addr.val, qspi->base + CA_QSPI_ADDR);
	}

	/* dataless ops kick as "write" (no data to clock in) */
	ret = ca_qspi_kick(qspi, op->data.dir != SPI_MEM_DATA_IN);

	/* small settle after an address-bearing command (vendor-proven) */
	if (op->addr.nbytes)
		udelay(10);
	return ret;
}

static int ca_qspi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct ca_qspi *qspi =
		spi_controller_get_devdata(mem->spi->controller);

	if (op->data.nbytes)
		return ca_qspi_exec_extended(qspi, op);
	return ca_qspi_exec_simple(qspi, op);
}

static int ca_qspi_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	op->data.nbytes = min_t(unsigned int, op->data.nbytes,
				CA_QSPI_MAX_DATA);
	return 0;
}

static const struct spi_controller_mem_ops ca_qspi_mem_ops = {
	.adjust_op_size	= ca_qspi_adjust_op_size,
	.exec_op	= ca_qspi_exec_op,
};

static int ca_qspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct ca_qspi *qspi;
	u32 width;

	ctlr = devm_spi_alloc_host(dev, sizeof(*qspi));
	if (!ctlr)
		return -ENOMEM;

	qspi = spi_controller_get_devdata(ctlr);
	qspi->dev = dev;
	qspi->base = devm_platform_ioremap_resource_byname(pdev, "qspi-base");
	if (IS_ERR(qspi->base))
		return PTR_ERR(qspi->base);

	if (!of_property_read_u32(dev->of_node, "spi-tx-bus-width", &width)) {
		if (width == 2)
			ctlr->mode_bits |= SPI_TX_DUAL;
		else if (width == 4)
			ctlr->mode_bits |= SPI_TX_QUAD;
	}
	if (!of_property_read_u32(dev->of_node, "spi-rx-bus-width", &width)) {
		if (width == 2)
			ctlr->mode_bits |= SPI_RX_DUAL;
		else if (width == 4)
			ctlr->mode_bits |= SPI_RX_QUAD;
	}

	ctlr->bus_num = -1;
	ctlr->mem_ops = &ca_qspi_mem_ops;
	ctlr->dev.of_node = dev->of_node;

	/* hw init: default to 4-byte address geometry (TAURUS: TIMING keeps
	 * its reset/bootloader value) */
	writel(CA_QSPI_TYPE_SIZE_4B, qspi->base + CA_QSPI_TYPE);

	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id ca_qspi_of_match[] = {
	{ .compatible = "cortina,ca-qspi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ca_qspi_of_match);

static struct platform_driver ca_qspi_driver = {
	.probe	= ca_qspi_probe,
	.driver	= {
		.name		= "ca-qspi",
		.of_match_table	= ca_qspi_of_match,
	},
};
module_platform_driver(ca_qspi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cortina-Access QSPI flash controller (RTL9607F Elnath)");
