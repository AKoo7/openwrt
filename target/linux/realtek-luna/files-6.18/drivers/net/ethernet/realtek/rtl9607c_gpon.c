// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RTL9607C GPON driver — chip-specific hardware bring-up for the Luna
 * interAptiv SoC.  Provides the register-access ops (rtl960x_ops) that the
 * shared family library (rtl960x_ponmac.c) and the shared protocol core
 * (gpon_proto.c / gpon-rtl9602c.c) use.
 *
 * Key difference from the 9602C: the SWCORE has a decode hole at offsets
 * 0xB0-0xD8 (the I2C indirect registers).  Those are accessed through a
 * PHY-10 register proxy over the GMAC MDIO controller (MIIAR at GMAC+0x5C),
 * matching the switch's MDIO indirect memory-read path.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/io.h>
#include <asm/addrspace.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include "rtl960x_ponmac.h"

#define SWCORE_PHYS		0x1b000000u
#define SWCORE_SIZE		0x00041000u

/* SMI indirect access registers (accessible via direct I/O, 0x23000 range) */
#define SW_IO_MODE_EN		0x23014u
#define   SW_MDX_M_EN		BIT(10)
#define SMI_CTRL_0		0x230B8u
#define   SMI_CMD_EN		BIT(0)
#define SMI_CTRL_2		0x230C0u	/* port mask for SMI access */
#define SMI_CTRL_3		0x230C4u
#define SMI_BC_PHYID		0x230C8u
#define SWCORE_PROXY_PHY	10

static void __iomem *swcore_base;

/* ---- SMI PHY access (switch-internal MDIO) ----------------------------- */
/*
 * The switch core's internal MDIO bus reaches PHY 0-31, including
 * PHY 10 (the SWCORE register proxy).  Use SMI_INDRT_ACCESS_CTRL which
 * IS directly memory-mapped (unlike the I2C indirect registers at 0xB0).
 */
static u16 smi_phy_read(u8 phy, u8 reg)
{
	u32 ctrl, data;
	int i;

	iowrite32(phy << 5, swcore_base + SMI_BC_PHYID);
	iowrite32(BIT(phy), swcore_base + SMI_CTRL_2);	/* target port mask */
	/* MAIN_PAGE=0x1FFF (broadcast), REG=phyreg, CMD=1 (trigger read) */
	iowrite32((0x1FFFu << 11) | ((u32)reg << 6) | SMI_CMD_EN,
		  swcore_base + SMI_CTRL_0);
	for (i = 0; i < 1000; i++) {
		ctrl = ioread32(swcore_base + SMI_CTRL_0);
		if (!(ctrl & SMI_CMD_EN))
			break;
		udelay(10);
	}
	data = ioread32(swcore_base + SMI_CTRL_3);
	return (u16)(data >> 16);
}

static void smi_phy_write(u8 phy, u8 reg, u16 val)
{
	u32 ctrl;
	int i;

	iowrite32(phy << 5, swcore_base + SMI_BC_PHYID);
	iowrite32(BIT(phy), swcore_base + SMI_CTRL_2);
	iowrite32(val, swcore_base + SMI_CTRL_3);	/* data in bits[15:0] */
	/* MAIN_PAGE=0x1FFF, REG=phyreg, RWOP=1 (write), CMD=1 */
	iowrite32((0x1FFFu << 11) | ((u32)reg << 6) | BIT(4) | SMI_CMD_EN,
		  swcore_base + SMI_CTRL_0);
	for (i = 0; i < 1000; i++) {
		ctrl = ioread32(swcore_base + SMI_CTRL_0);
		if (!(ctrl & SMI_CMD_EN))
			break;
		udelay(10);
	}
}

/* ---- SWCORE register access via PHY-10 proxy --------------------------- */
static u32 sw_proxy_rd(u32 swc_off)
{
	u16 lo, hi;

	smi_phy_write(SWCORE_PROXY_PHY, 0, (u16)(swc_off & 0xffff));
	smi_phy_write(SWCORE_PROXY_PHY, 1, (u16)((swc_off >> 16) & 0xffff));
	smi_phy_write(SWCORE_PROXY_PHY, 6, 0x800b);	/* read trigger */
	lo = smi_phy_read(SWCORE_PROXY_PHY, 4);
	hi = smi_phy_read(SWCORE_PROXY_PHY, 5);
	return ((u32)hi << 16) | lo;
}

static void sw_proxy_wr(u32 swc_off, u32 val)
{
	smi_phy_write(SWCORE_PROXY_PHY, 0, (u16)(swc_off & 0xffff));
	smi_phy_write(SWCORE_PROXY_PHY, 1, (u16)((swc_off >> 16) & 0xffff));
	smi_phy_write(SWCORE_PROXY_PHY, 2, (u16)(val & 0xffff));
	smi_phy_write(SWCORE_PROXY_PHY, 3, (u16)((val >> 16) & 0xffff));
	smi_phy_write(SWCORE_PROXY_PHY, 6, 0x804b);	/* write trigger */
}

/* ---- rtl960x_ops: direct I/O with MDIO-proxy fallback ------------------ */
/*
 * Most SWCORE registers are directly memory-mapped.  The I2C indirect
 * registers (0xB0-0xD8) are a decode hole and need the PHY-10 proxy.
 */
static u32 rtl9607c_sw_rd(u32 phys)
{
	u32 off = phys - SWCORE_PHYS;

	if (off >= 0xB0 && off <= 0xD8)
		return sw_proxy_rd(off);
	return ioread32(swcore_base + off);
}

static void rtl9607c_sw_wr(u32 phys, u32 val)
{
	u32 off = phys - SWCORE_PHYS;

	if (off >= 0xB0 && off <= 0xD8)
		sw_proxy_wr(off, val);
	else
		iowrite32(val, swcore_base + off);
}

static const struct rtl960x_ops rtl9607c_ops = {
	.rd = rtl9607c_sw_rd,
	.wr = rtl9607c_sw_wr,
};

/* ---- /proc/gpon diagnostic --------------------------------------------- */
void rtl960x_c7_diag(const struct rtl960x_ops *o, struct seq_file *s);

static int rtl9607c_gpon_proc_show(struct seq_file *s, void *v)
{
	rtl960x_c7_diag(&rtl9607c_ops, s);
	return 0;
}

/* ---- module init ------------------------------------------------------- */
static int __init rtl9607c_gpon_init(void)
{
	int rc;

	if (!of_machine_is_compatible("realtek,rtl9607c"))
		return -ENODEV;

	swcore_base = ioremap(SWCORE_PHYS, SWCORE_SIZE);
	if (!swcore_base)
		return -ENOMEM;

	/* Enable the switch-internal MDIO master + toggle I2C_EN to release
	 * the I2C indirect register aperture from clock-gating. The I2C
	 * sub-block registers at 0xB0-0xD8 stay decode-gated until I2C_EN
	 * transitions 0→1. */
	{
		u32 iomode = ioread32(swcore_base + SW_IO_MODE_EN);
		pr_info("rtl9607c-gpon: IO_MODE_EN before: 0x%08x (I2C_EN=%d)\n",
			iomode, (iomode >> 13) & 1);
		/* Clear I2C_EN, then set it — forces a 0→1 transition */
		iomode &= ~BIT(13);
		iowrite32(iomode, swcore_base + SW_IO_MODE_EN);
		(void)ioread32(swcore_base + SW_IO_MODE_EN);
		udelay(100);
		iomode |= BIT(13) | SW_MDX_M_EN;
		iowrite32(iomode, swcore_base + SW_IO_MODE_EN);
		(void)ioread32(swcore_base + SW_IO_MODE_EN);
		udelay(1000);	/* let the I2C block settle */
		pr_info("rtl9607c-gpon: IO_MODE_EN after: 0x%08x\n",
			ioread32(swcore_base + SW_IO_MODE_EN));
	/* Broad scan: find non-zero SWCORE registers to locate the I2C
	 * indirect registers. */
	{
		/* GMAC IP power-cycle (matches the working firmware's NIC reset):
		 * clear BSP_IP_SEL bit1, wait, set bit1. This power-cycles
		 * the GMAC which may bring the switch-core sub-blocks
		 * (including I2C indirect regs) out of reset. */
		void __iomem *ipsel = ioremap(0x18000600u, 4);
		if (ipsel) {
			u32 v = ioread32(ipsel);
			pr_info("rtl9607c-gpon: GMAC power-cycle: BSP_IP_SEL 0x%x\n", v);
			iowrite32(v & ~BIT(1), ipsel);	/* clear EN_GMAC */
			mdelay(10);
			iowrite32(v | BIT(1), ipsel);	/* set EN_GMAC */
			mdelay(10);
			(void)ioread32(ipsel);
			iounmap(ipsel);
		}

		/* Re-enable I2C_EN after the power-cycle */
		{
			u32 iomode = ioread32(swcore_base + SW_IO_MODE_EN);
			iomode |= BIT(13) | SW_MDX_M_EN;
			iowrite32(iomode, swcore_base + SW_IO_MODE_EN);
			udelay(1000);
		}

		/* Scan for non-zero registers */
		u32 off;
		pr_info("rtl9607c-gpon: SWCORE scan post-reset:\n");
		for (off = 0x80; off < 0x100; off += 4) {
			u32 v = ioread32(swcore_base + off);
			if (v) pr_info("  sw[0x%05x]=0x%08x\n", off, v);
		}
		for (off = 0x400; off < 0x520; off += 4) {
			u32 v = ioread32(swcore_base + off);
			if (v) pr_info("  sw[0x%05x]=0x%08x\n", off, v);
		}
	}
	}

	/* Enable PON PBO IP.
	 * SoC register 0x1800063C bit5 = PONPBO enable, bit25 = rev>B enable. */
	{
		void __iomem *ipen = ioremap(0x1800063Cu, 4);
		if (ipen) {
			u32 v = ioread32(ipen);
			v |= BIT(5);		/* PONPBO IP enable */
			v |= BIT(25);		/* PONPBO (rev > A) */
			iowrite32(v, ipen);
			(void)ioread32(ipen);
			iounmap(ipen);
		}
	}

	/* Enable switch-core IP (BSP_IP_SEL bit1 = GMAC).
	 * The switch core sub-blocks (incl. I2C indirect regs at 0xB0)
	 * may not decode until the GMAC IP is enabled in SoC register
	 * 0x18000600 (BSP_IP_SEL). */
	{
		void __iomem *ipsel = ioremap(0x18000600u, 4);
		if (ipsel) {
			u32 v = ioread32(ipsel);
			pr_info("rtl9607c-gpon: BSP_IP_SEL(0x18000600) before=0x%08x (EN_GMAC=%d)\n",
				v, (v >> 1) & 1);
			v |= BIT(1);		/* EN_GMAC */
			iowrite32(v, ipsel);
			(void)ioread32(ipsel);
			mdelay(10);		/* let the switch core come up */
			pr_info("rtl9607c-gpon: BSP_IP_SEL after=0x%08x\n",
				ioread32(ipsel));
			iounmap(ipsel);
		}
	}

	/* Verify the MDIO proxy: read SDS_CFG (0x270) both ways */
	{
		u32 direct = ioread32(swcore_base + 0x270);
		u32 proxy  = sw_proxy_rd(0x270);

		pr_info("rtl9607c-gpon: MDIO verify SDS_CFG: direct=0x%x proxy=0x%x %s\n",
			direct, proxy, (direct == proxy) ? "OK" : "MISMATCH");
	}

	/*
	 * Bring up the SerDes + PON-MAC via the shared family library.
	 *
	 * Revision selects the SerDes analog operating point: rev A -> ModeV1,
	 * B -> V2, C -> V3 (the tables differ substantially on the RX/FIB block).
	 * This board is rev C: the datapath already forces the "rev > A" PON-PBO
	 * enable (0x1800063C bit25), and the GPON bring-up requires the FIB analog
	 * power-on (FIB_REG0=0x1140) that only the ModeV3 sequence performs. Running
	 * the rev-A table (V1) leaves the RX front-end in the wrong operating point,
	 * which is consistent with SDS_SDET never asserting. Select V3.
	 * TODO: read the real revision from the SWCORE chip-id register instead of
	 * pinning it, to stay correct across the family.
	 */
	rc = rtl960x_ponmac_init(RTL960X_CHIP_9607C, RTL960X_REV_C,
				 RTL960X_SUBTYPE_NONE, &rtl9607c_ops);
	if (rc)
		pr_warn("rtl9607c-gpon: ponmac_init failed: %d\n", rc);

	rc = rtl960x_ponmac_mode_set(RTL960X_CHIP_9607C, RTL960X_REV_C,
				     RTL960X_SUBTYPE_NONE,
				     RTL960X_MODE_GPON, &rtl9607c_ops);
	if (rc)
		pr_warn("rtl9607c-gpon: mode_set failed: %d\n", rc);

	pr_info("rtl9607c-gpon: SerDes + PON-MAC initialized\n");

	/* Expose the SerDes/FIB signal-detect state (sds_sdet, gtc_los, fib_reg16)
	 * for M3 DS-lock bring-up. */
	proc_create_single("gpon", 0444, NULL, rtl9607c_gpon_proc_show);

	/* TODO: I2C BOSA init, PON-MAC datapath, PLOAM FSM */

	return 0;
}

module_init(rtl9607c_gpon_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek RTL9607C GPON PON-MAC driver");
