// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access GPON MAC driver for the Realtek RTL9607F "Elnath" ONU.
 *
 * The RTL9607F is a Cortina-Access CA8277C ("TAURUS") SoC; its GPON MAC is a
 * Cortina IP block (register set rtl8277c_registers.h), NOT the Realtek "Luna"
 * GTC used on the RTL9602C/9607C.  This driver is a clean-room re-expression of
 * the GPLv2 Cortina ca-network-engine (aal-77c) GPON layer, the same package the
 * sibling cortina-ni Ethernet driver derives from.
 *
 * Key architectural fact (validated on live stock hardware, 2026-07-13):
 *   - The GPON MAC block lives at physical 0x4_F5506000 (the PON register window
 *     0x4_F5500000 + 0x6000).  The vendor-id register (+0x14) reads the ASCII
 *     serial-number prefix "XPON", confirming the base.
 *   - The G.984.3 activation FSM (O1..O5) runs autonomously in the MAC hardware;
 *     software reads the current state from GPON_onu.state (+0xbc) rather than
 *     ticking a software FSM.  So this driver polls/services the MAC, it does not
 *     drive the ranging handshake.
 *
 * Phase 0: probe, map the PON window, expose the ONU state + a register peek via
 * /proc so the register map can be validated from the driver on real hardware.
 * Later phases add the PSDS SerDes optics bring-up, PLOAM servicing, and the
 * OMCI/GEM datapath.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "cortina-gpon-serdes.h"

#define DRV_NAME		"cortina-gpon"

/* PON register window (from the DT reg entry / SDK): phys 0x4_F5500000, 48 KiB. */
#define CG_PON_WINDOW_PHYS	0x4f5500000ULL
#define CG_PON_WINDOW_SIZE	0xc000

/* The GPON MAC register block sits at window + 0x6000 (aal_pon.h). */
#define CG_GPON_MAC_OFF		0x6000

/*
 * PON-SerDes (PSDS) registers, direct within the PON window.
 *   PSDS_MODE (+0xa02c): SerDes rate/mode.  GPON = 0x408 (sd_s0=1, sds_mode_s0=0x8).
 *   PSDS_RGB8 (+0xa060): SerDes status.  bit10 CKRDY_RX, bit11 CKRDY_TX (TX PLL
 *     locked off the reference clock; asserts without fiber), bit0 RX_LOS.
 */
#define CG_PSDS_MODE		0xa02c
#define CG_PSDS_RGB8		0xa05c	/* DS-lock status; locked = (val & 0x9c01)==0x9c00 (stock 0x19c00) */
#define CG_PSDS_GBOX_CTRL	0xa060	/* rx/tx bit-ordering[7:4]; stock=0x454.  WAS 0xa064 (stock=0) -> our US tx_bit_ordering never took -> OLT saw US LOS (live-diff 2026-07-13) */
#define CG_PON_EPON_SPARE	0x01c8	/* EPON_GLB_SPARE_CFG (PON window); bit31 for GPON los-rst */

/*
 * GLB (global) PON/GPON reset & clock control window: phys 0x4_F4320000, 4 KiB.
 * On our minimal build the GPON MAC reads garbage (block held in reset); the
 * vendor aal_gpon glb-reset clocks it.  Offsets + released values measured on
 * live stock (the block reads "XPON" with these):
 *   EPON_CNTL(+0x078)=0x00030000  GPON_CNTL(+0x080)=0x00000003  PON_CNTL(+0x09c)=0x0000030e
 * GPON_CNTL bits: ani_rst_n[0], gpon_rst_n[1].  PON_CNTL bits: pon_serdes_rst_n[1],
 * psds_reg_rst_n[2], ptp_rst_n[3], puc_reset[8], pdc_reset[9].
 */
#define CG_GLB_WINDOW_PHYS	0x4f4320000ULL
#define CG_GLB_WINDOW_SIZE	0x1000
#define CG_GLB_EPON_CNTL	0x078
#define CG_GLB_GPON_CNTL	0x080
#define CG_GLB_PON_CNTL		0x09c
/*
 * GLOBAL_PSDS_INIT_CNTL: bit5 POW_PCIX powers the PON-SerDes analog+digital
 * logic, which generates the PON APB register-bus clock the GPON MAC lives on.
 * bit4 ben_oen is the laser burst-enable (leave 0 during bring-up).  The CA8277C
 * physical offset is +0x25c (header 0x22c is wrong / shifted); measured live:
 * stock reads 0x30 (POW_PCIX + ben_oen), cold reads 0x00.
 */
#define CG_GLB_PSDS_INIT	0x25c
#define CG_PSDS_POW_PCIX	BIT(5)
#define CG_PSDS_BEN_OEN		BIT(4)

/*
 * Laser TX-disable GPIO.  On this board the laser is gated by GPIO pin 34
 * (group 1, bit 2): tx-disable is active-high, so driving the pin LOW enables
 * the laser (allows the upstream burst).  The vendor asserts it (laser off) at
 * init and de-asserts it (laser on) at activation.  Three registers, all RMW on
 * bit 2 only (never whole-register writes):
 *   GLOBAL_GPIO_MUX_1 (GLB +0x104): pin->GPIO
 *   PER_GPIO1_CFG (0x4_F43292E4):   0 = OUTPUT
 *   PER_GPIO1_OUT (0x4_F43292E8):   0 = tx-disable de-asserted = LASER ON
 * The PER_GPIO block is a separate MMIO window (0x4_F4329000) from the GLB one.
 */
#define CG_PERGPIO_PHYS		0x4f4329000ULL
#define CG_PERGPIO_SIZE		0x1000
#define CG_GLB_GPIO_MUX1	0x104
#define CG_PERGPIO_CFG1		0x2e4
#define CG_PERGPIO_OUT1		0x2e8
#define CG_LASER_PIN34		BIT(2)

/* GPON MAC register offsets within the block (rtl8277c_registers.h, aal_gpon.c). */
#define CG_REG_GPON_DS		0x000	/* DS framer thresholds; max_packet_size low */
#define CG_REG_VENDOR		0x014	/* vendor-id (ASCII "XPON") */
#define CG_REG_VENDOR_SPEC	0x018	/* vendor-specific serial number */
#define CG_REG_ALARM		0x07c	/* LOS/LOF alarm bits */
#define CG_REG_GPON_ONU		0x0bc	/* ONU activation state + assigned ONU-ID */
#define CG_REG_GPON_MAIN	0x0c0	/* equalization delay (EqD) */
#define CG_REG_ONU_CFG		0x118	/* (header onu_cfg offset; the real one is +0x138) */
/*
 * onu_cfg (real, +0x138 on this silicon).  Top byte laser_on_align=0x12 aligns
 * the upstream laser burst to the OLT's grant window; at the reset default
 * (0x00100780, laser_on_align=0) the burst is mis-aligned and the OLT cannot
 * decode the SerialNumber, so ranging stalls at O1.  This is range-critical.
 */
#define CG_REG_ONU_CFG_REAL	0x138
#define CG_ONU_CFG_VAL		0x12100780
/*
 * The activation control register: the header calls it onu_ctl at +0x114, but on
 * this CA8277C silicon it is at +0x134 (a +0x20 shift measured live: stock reads
 * +0x134 = 0x00460262 with the enable bit set at O5, while +0x114 reads 0).  Bit1
 * = en -> the MAC autonomously ranges O1->O5.  We write the full stock value.
 */
#define CG_REG_ONU_CTL		0x134
#define CG_ONU_CTL_VAL		0x00460262	/* stock O5 value: en(bit1) + defaults */
#define CG_REG_GPON_MAC_CTRL	0x1c4	/* GPON_MAC_GPON_CTRL (sw_random_en) */

struct cortina_gpon {
	struct device *dev;
	void __iomem *pon;		/* ioremap of the whole PON window */
	void __iomem *mac;		/* pon + CG_GPON_MAC_OFF, the GPON MAC block */
	void __iomem *glb;		/* ioremap of the GLB reset/clock window */
	void __iomem *gpio;		/* ioremap of the PER_GPIO window */
	struct proc_dir_entry *proc;
};

static struct cortina_gpon *cg_singleton;

static bool cg_do_reset = true;
module_param_named(reset, cg_do_reset, bool, 0444);
MODULE_PARM_DESC(reset, "release the GPON MAC from reset/clock-gate at probe (default on)");

/*
 * Release the GPON MAC from reset and clock-gate so its registers read valid.
 * This is the vendor aal_gpon __gpon_glb_reset + aal_gpon_glb_ctrl_init sequence
 * MINUS the aal_psds_init SerDes/optics block (which is only needed for actual
 * downstream light, not register readability).  The literal values match the
 * live-stock released state measured on this board:
 *   EPON_CNTL=0x00030000 (onu mode), PON_CNTL=0x0000030e (pon_serdes/psds/ptp +
 *   puc/pdc), GPON_CNTL=0x00000003 (ani_rst_n + gpon_rst_n).
 * cortina-ni does not touch these registers, so this is safe and independent.
 */
static void cg_glb_reset(struct cortina_gpon *cg)
{
	void __iomem *glb = cg->glb;
	u32 v;

	/* aal_gpon __gpon_glb_reset: mode select + assert-then-release resets */
	writel(0x00030000, glb + CG_GLB_EPON_CNTL);	/* select PON/ONU mode */
	writel(0x00000000, glb + CG_GLB_PON_CNTL);	/* assert all PON-domain resets */
	writel(0x00000000, glb + CG_GLB_GPON_CNTL);	/* assert ani/gpon reset */
	writel(0x00000004, glb + CG_GLB_PON_CNTL);	/* __psds_csr_out_of_reset: psds_reg_rst_n 0->1 edge */
	mdelay(1);
	/* aal_gpon_glb_ctrl_init: release PON/GPON resets to the stock state */
	writel(0x0000030e, glb + CG_GLB_PON_CNTL);	/* pon_serdes/psds/ptp + puc/pdc */
	writel(0x00000003, glb + CG_GLB_GPON_CNTL);	/* ani_rst_n + gpon_rst_n */
	mdelay(1);

	/* __psds_ad_reset: hold the SerDes analog/digital powered-down (POW_PCIX=0)
	 * until PSDS_MODE + the analog profile are loaded (done in cg_psds_init). */
	v = readl(glb + CG_GLB_PSDS_INIT);
	writel(v & ~(CG_PSDS_POW_PCIX | CG_PSDS_BEN_OEN), glb + CG_GLB_PSDS_INIT);
	udelay(1);
}

/*
 * Bring the PON-SerDes CMU/PLL up so it generates the PON APB register-bus clock
 * the GPON MAC lives on.  POW_PCIX alone is not enough: the MAC's clock is
 * derived from the SerDes line/CMU PLL, which must be given its rate (PSDS_MODE)
 * and its analog config (the ~266-row profile) BEFORE it is powered (POW_PCIX).
 * The PLL locks off the reference clock, so this works with no fiber / no DS
 * light — only the DS-RX lock (a later phase) needs actual light.
 * This is aal_psds_out_of_reset minus the RX-lock wait.
 */
static void cg_psds_init(struct cortina_gpon *cg)
{
	void __iomem *pon = cg->pon;
	u32 v;
	int i;

	/* __psds_mode_init: GPON rate — sd_s0=1, sds_mode_s0=0x8, usx=0 */
	writel(0x00000408, pon + CG_PSDS_MODE);
	udelay(10);

	/* __psds_prof_load: the CMU/PLL/CDR/TX-driver analog profile.  Each row is
	 * a direct write to the PSDS block (applied via its DATAIN/ACCESS pair). */
	for (i = 0; i < ARRAY_SIZE(cg_serdes_gpon); i++) {
		writel(cg_serdes_gpon[i].val, pon + cg_serdes_gpon[i].off);
		udelay(cg_serdes_gpon[i].delay_us ? cg_serdes_gpon[i].delay_us : 10);
	}

	/* __psds_disable_gpon_los_rst: hold EPON in reset + set the spare-cfg bit
	 * around the lock wait (GPON-only quirk). */
	v = readl(cg->glb + CG_GLB_EPON_CNTL);
	writel(v | BIT(0), cg->glb + CG_GLB_EPON_CNTL);		/* epon_rst_n = 1 */
	v = readl(pon + CG_PON_EPON_SPARE);
	writel(v | 0x80000000, pon + CG_PON_EPON_SPARE);

	/* __psds_ad_out_of_reset: power the SerDes -> PON APB clock runs.  Leave the
	 * laser burst-enable (ben_oen) OFF for now (set at activation). */
	v = readl(cg->glb + CG_GLB_PSDS_INIT);
	writel((v | CG_PSDS_POW_PCIX) & ~CG_PSDS_BEN_OEN, cg->glb + CG_GLB_PSDS_INIT);
	mdelay(100);

	/* __psds_sync: bounded best-effort wait for RX clock lock (continues on
	 * timeout, exactly as the vendor does at boot). */
	for (i = 0; i < 100; i++) {
		if ((readl(pon + CG_PSDS_RGB8) & 0x9c01) == 0x9c00)
			break;
		mdelay(1);
	}

	/* release GPON los-reset */
	v = readl(cg->glb + CG_GLB_EPON_CNTL);
	writel(v & ~BIT(0), cg->glb + CG_GLB_EPON_CNTL);	/* epon_rst_n = 0 */

	/*
	 * __psds_gbox_out_of_reset: toggle GLOBAL_PON_CNTL.pon_serdes_rst_n (bit1)
	 * 0->1 HERE, after the SerDes is powered.  This gearbox connects the SerDes
	 * serial stream to the GPON MAC's parallel DS input -- without it the RX
	 * clock locks but zero downstream frames reach the MAC framer.
	 */
	v = readl(cg->glb + CG_GLB_PON_CNTL);
	writel(v & ~BIT(1), cg->glb + CG_GLB_PON_CNTL);
	mdelay(1);
	writel(v | BIT(1), cg->glb + CG_GLB_PON_CNTL);
	mdelay(100);

	/* __psds_gbox_init: rx/tx bit-ordering = 1 (reset default already 0x454) */
	v = readl(pon + CG_PSDS_GBOX_CTRL);
	v = (v & ~((0x3u << 4) | (0x3u << 6))) | (0x1u << 4) | (0x1u << 6);
	writel(v, pon + CG_PSDS_GBOX_CTRL);
	mdelay(1);
}

static bool cg_activate = true;
module_param_named(activate, cg_activate, bool, 0444);
MODULE_PARM_DESC(activate, "program the SN + start GPON ranging at probe (default on)");

/* De-assert the laser TX-disable (pin 34) so the ONU can burst upstream.  RMW
 * bit 2 only, matching the live-stock O5 state (mux=0, cfg=output, out=0). */
static void cg_laser_on(struct cortina_gpon *cg)
{
	u32 v;

	if (!cg->gpio)
		return;
	v = readl(cg->glb + CG_GLB_GPIO_MUX1);
	writel(v & ~CG_LASER_PIN34, cg->glb + CG_GLB_GPIO_MUX1);
	v = readl(cg->gpio + CG_PERGPIO_CFG1);
	writel(v & ~CG_LASER_PIN34, cg->gpio + CG_PERGPIO_CFG1);	/* output */
	v = readl(cg->gpio + CG_PERGPIO_OUT1);
	writel(v & ~CG_LASER_PIN34, cg->gpio + CG_PERGPIO_OUT1);	/* drive 0 = laser on */
}

/*
 * Program the GPON MAC identity + datapath, then start the activation FSM.  The
 * G.984.3 O1->O5 ranging runs in HARDWARE: once the serial number is programmed
 * and onu_ctl.en is set, the MAC autonomously transmits its Serial_Number PLOAM,
 * answers Assign_ONU-ID / Ranging_Time, and advances onu.state to O5.  Software
 * only configures + polls.  Serial number / config MUST be written while en=0.
 * (aal_gpon __gpon_common_init + aal_pon_mac_enable_set, GPON branch.)
 */
static void cg_mac_activate(struct cortina_gpon *cg)
{
	void __iomem *mac = cg->mac;
	u32 v;
	int i;

	/*
	 * aal_gpon_glb_ctrl_init runs AFTER aal_psds_init in the vendor flow: (re-)
	 * assert the PON/GPON resets + GPON mode now that the SerDes is up, so the
	 * GTC downstream framer latches GPON (not nxgs/us10g 10G) framing with a live
	 * SerDes.  GLOBAL_GPON_CNTL=0x03 clears nxgs_mode[9]/us10g_mode[8] (default
	 * 0x300 = XGS-PON 10G) and sets ani_rst_n/gpon_rst_n.
	 */
	writel(0x0000030e, cg->glb + CG_GLB_PON_CNTL);
	writel(0x00000003, cg->glb + CG_GLB_GPON_CNTL);
	mdelay(10);

	/* --- config while en=0 (serial number is range-critical) --- */
	writel(CG_ONU_CFG_VAL, mac + CG_REG_ONU_CFG_REAL);	/* laser_on_align=0x12 (align US burst) */
	writel(0x58504f4e, mac + CG_REG_VENDOR);	/* vendor-id "XPON" */
	writel(0x5c6cafcb, mac + CG_REG_VENDOR_SPEC);	/* VSSN 5C6CAFCB */
	/* datapath: gpon_ds.max_packet_size (bits 29:16) = 0x3FFF */
	v = readl(mac + CG_REG_GPON_DS);
	v = (v & ~(0x3fffu << 16)) | (0x3fffu << 16);
	writel(v, mac + CG_REG_GPON_DS);
	/* password / AES keys / gemport / PDC-PUC: deferred (not needed to range) */

	/* Wait for the downstream to lock (RGB8 bit15 BER_NOTIFY) before enabling
	 * ranging, so the FSM sees a live downstream at the moment en is asserted. */
	for (i = 0; i < 8000; i++) {
		if ((readl(cg->pon + CG_PSDS_RGB8) & 0x9c01) == 0x9c00)
			break;
		mdelay(1);
	}
	dev_info(cg->dev, "activate: DS-lock wait done at %dms, rgb8=0x%08x\n",
		 i, readl(cg->pon + CG_PSDS_RGB8));

	/* --- the GO --- */
	/* enable the laser burst output (ben_oen); HW gates the actual burst per
	 * grant (onu_cfg.laser_on stays 0 -> burst, not CW). */
	v = readl(cg->glb + CG_GLB_PSDS_INIT);
	writel(v | CG_PSDS_BEN_OEN, cg->glb + CG_GLB_PSDS_INIT);
	/* onu_ctl.en -> HW starts ranging O1->O5.  onu_ctl/onu_cfg live at the SILICON
	 * offsets +0x134/+0x138 (a +0x20 shift vs the rtl8277c header's +0x114/+0x118,
	 * above offset 0x100 only), proven by live devmem on stock: 0x134=0x00460262
	 * and 0x138=0x12100900 hold the onu_ctl/onu_cfg bit patterns, while 0x114/0x118
	 * are the DS-PLOAM RX FIFO regs (PLOAMD_FF_CTL / PLOAMD_FIFO3).  onu_cfg is
	 * written at +0x138 above; here just assert onu_ctl.en at +0x134.
	 * We used to ALSO poke +0x114/+0x118 to settle the ambiguity -- those writes
	 * corrupted the DS-PLOAM RX FIFO so the MAC never processed the OLT's ranging
	 * PLOAMs and the FSM stalled at O1.  Removed (live-verified 2026-07-13). */
	writel(CG_ONU_CTL_VAL, mac + CG_REG_ONU_CTL);	/* onu_ctl.en @ +0x134 */
	/* de-assert the laser TX-disable so the ONU can burst its US Serial_Number */
	mdelay(1);
	cg_laser_on(cg);
}

static inline u32 cg_mac_rd(struct cortina_gpon *cg, u32 off)
{
	return readl(cg->mac + off);
}

/* Read the 4 ASCII bytes of the vendor-id register in wire order. */
static void cg_read_vendor(struct cortina_gpon *cg, char out[5])
{
	u32 v = cg_mac_rd(cg, CG_REG_VENDOR);

	out[0] = (v >> 24) & 0xff;
	out[1] = (v >> 16) & 0xff;
	out[2] = (v >> 8) & 0xff;
	out[3] = v & 0xff;
	out[4] = '\0';
}

static int cg_proc_show(struct seq_file *m, void *v)
{
	struct cortina_gpon *cg = m->private;
	char vendor[5];
	u32 onu, alarm;

	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	alarm = cg_mac_rd(cg, CG_REG_ALARM);

	seq_printf(m, "gpon-mac @ phys 0x%llx + 0x%x\n",
		   (unsigned long long)CG_PON_WINDOW_PHYS, CG_GPON_MAC_OFF);
	seq_printf(m, "vendor-id      = 0x%08x (\"%s\")\n",
		   cg_mac_rd(cg, CG_REG_VENDOR), vendor);
	seq_printf(m, "vendor-spec    = 0x%08x\n", cg_mac_rd(cg, CG_REG_VENDOR_SPEC));
	seq_printf(m, "gpon_ds        = 0x%08x\n", cg_mac_rd(cg, CG_REG_GPON_DS));
	seq_printf(m, "onu(state+id)  = 0x%08x\n", onu);
	seq_printf(m, "main(eqd)      = 0x%08x\n", cg_mac_rd(cg, CG_REG_GPON_MAIN));
	seq_printf(m, "alarm          = 0x%08x%s\n", alarm,
		   alarm ? " (LOS/LOF!)" : " (no alarm, DS locked)");
	seq_printf(m, "onu_cfg        = 0x%08x\n", cg_mac_rd(cg, CG_REG_ONU_CFG));

	/* serdes/gearbox/laser (PON-window raw offsets, for US-LOS diagnosis) */
	seq_puts(m, "-- serdes/gbox/laser --\n");
	seq_printf(m, "rgb8(a05c)     = 0x%08x  (DS-lock: (v&0x9c01)==0x9c00)\n", readl(cg->pon + 0xa05c));
	seq_printf(m, "gbox(a060)     = 0x%08x  (stock 0x454 rx/tx bit-order)\n", readl(cg->pon + 0xa060));
	seq_printf(m, "reg(a064)      = 0x%08x  (stock 0)\n", readl(cg->pon + 0xa064));
	seq_printf(m, "reg(a068)      = 0x%08x  (stock 1)\n", readl(cg->pon + 0xa068));
	seq_printf(m, "reg(a070)      = 0x%08x  (stock 1)\n", readl(cg->pon + 0xa070));
	seq_printf(m, "psds_init(glb) = 0x%08x  (ben_oen bit4, pow_pcix bit5)\n", readl(cg->glb + CG_GLB_PSDS_INIT));
	seq_printf(m, "laser_gpio1    = 0x%08x  (pin34=bit2, 0=laser-on)\n", readl(cg->gpio + CG_PERGPIO_OUT1));

	/* full GPON MAC block dump (nonzero) for diffing against the stock golden */
	seq_puts(m, "-- MAC block (nonzero) --\n");
	{
		u32 off, val;

		for (off = 0; off <= 0x1c4; off += 4) {
			val = cg_mac_rd(cg, off);
			if (val)
				seq_printf(m, "+0x%03x=0x%08x\n", off, val);
		}
	}
	return 0;
}

static int cg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cg_proc_show, cg_singleton);
}

static const struct proc_ops cg_proc_ops = {
	.proc_open	= cg_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int cortina_gpon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_gpon *cg;
	char vendor[5];
	u32 onu;

	cg = devm_kzalloc(dev, sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return -ENOMEM;
	cg->dev = dev;

	/*
	 * Map the whole 48 KiB PON window.  The window is a 40-bit AXI address
	 * (0x4_F5500000); ioremap takes a 64-bit phys_addr_t so this is fine on
	 * arm64.  We ioremap the fixed physical base directly (validated on real
	 * hardware) rather than claiming a DT resource, because the same window is
	 * also listed by the sibling cortina-ni node - a non-exclusive map avoids a
	 * request_mem_region conflict.
	 */
	cg->pon = devm_ioremap(dev, CG_PON_WINDOW_PHYS, CG_PON_WINDOW_SIZE);
	if (!cg->pon) {
		dev_err(dev, "failed to map PON window 0x%llx\n",
			(unsigned long long)CG_PON_WINDOW_PHYS);
		return -ENOMEM;
	}
	cg->mac = cg->pon + CG_GPON_MAC_OFF;

	/*
	 * Map the GLB reset/clock window and dump the PON/GPON reset-control
	 * registers read-only.  On our minimal build the GPON MAC is held in
	 * reset; comparing these against the live-stock released values tells us
	 * the minimal diff to write (done in a later step) without clobbering the
	 * PUC/PDC packet-engine bits the NI datapath shares.
	 */
	cg->glb = devm_ioremap(dev, CG_GLB_WINDOW_PHYS, CG_GLB_WINDOW_SIZE);
	cg->gpio = devm_ioremap(dev, CG_PERGPIO_PHYS, CG_PERGPIO_SIZE);
	if (cg->glb) {
		dev_info(dev, "GLB reset regs (ours): EPON_CNTL=0x%08x GPON_CNTL=0x%08x PON_CNTL=0x%08x PSDS_INIT=0x%08x\n",
			 readl(cg->glb + CG_GLB_EPON_CNTL),
			 readl(cg->glb + CG_GLB_GPON_CNTL),
			 readl(cg->glb + CG_GLB_PON_CNTL),
			 readl(cg->glb + CG_GLB_PSDS_INIT));
		dev_info(dev, "GLB reset regs (stock released): EPON_CNTL=0x00030000 GPON_CNTL=0x00000003 PON_CNTL=0x0000030e\n");

		if (cg_do_reset) {
			cg_glb_reset(cg);
			cg_psds_init(cg);
			dev_info(dev, "GLB after: EPON=0x%08x GPON=0x%08x PON=0x%08x PSDS_INIT=0x%08x\n",
				 readl(cg->glb + CG_GLB_EPON_CNTL),
				 readl(cg->glb + CG_GLB_GPON_CNTL),
				 readl(cg->glb + CG_GLB_PON_CNTL),
				 readl(cg->glb + CG_GLB_PSDS_INIT));
			dev_info(dev, "PSDS after: MODE=0x%08x RGB8=0x%08x (bit11 CKRDY_TX=%d)\n",
				 readl(cg->pon + CG_PSDS_MODE),
				 readl(cg->pon + CG_PSDS_RGB8),
				 !!(readl(cg->pon + CG_PSDS_RGB8) & BIT(11)));

			if (cg_activate) {
				int i;

				cg_mac_activate(cg);
				dev_info(dev, "activate: onu_cfg=0x%08x onu_ctl=0x%08x gpon_ds=0x%08x\n",
					 cg_mac_rd(cg, CG_REG_ONU_CFG),
					 cg_mac_rd(cg, CG_REG_ONU_CTL),
					 cg_mac_rd(cg, CG_REG_GPON_DS));
				/* poll the HW ranging FSM: onu.state, RGB8 (bit15 BER_NOTIFY
				 * = DS frame sync), and a DS packet counter (nonzero = DS
				 * frames received). */
				for (i = 0; i < 30; i++) {
					dev_info(dev, "range t=%ds: onu=0x%08x rgb8=0x%08x ds_cnt=0x%08x alarm=0x%08x\n",
						 i, cg_mac_rd(cg, CG_REG_GPON_ONU),
						 readl(cg->pon + CG_PSDS_RGB8),
						 cg_mac_rd(cg, 0x64),
						 cg_mac_rd(cg, CG_REG_ALARM));
					msleep(200);
				}
			}
		}
	} else {
		dev_warn(dev, "failed to map GLB window 0x%llx\n",
			 (unsigned long long)CG_GLB_WINDOW_PHYS);
	}

	cg_read_vendor(cg, vendor);
	onu = cg_mac_rd(cg, CG_REG_GPON_ONU);
	dev_info(dev, "GPON MAC vendor-id \"%s\" onu=0x%08x alarm=0x%08x\n",
		 vendor, onu, cg_mac_rd(cg, CG_REG_ALARM));

	if (strcmp(vendor, "XPON") != 0)
		dev_warn(dev, "vendor-id != \"XPON\" - PON window base may be wrong\n");

	cg_singleton = cg;
	cg->proc = proc_create_data("gpon", 0444, NULL, &cg_proc_ops, cg);
	platform_set_drvdata(pdev, cg);
	dev_info(dev, "cortina-gpon phase-0 probe complete (/proc/gpon)\n");
	return 0;
}

static void cortina_gpon_remove(struct platform_device *pdev)
{
	struct cortina_gpon *cg = platform_get_drvdata(pdev);

	if (cg->proc)
		proc_remove(cg->proc);
	if (cg_singleton == cg)
		cg_singleton = NULL;
}

static const struct of_device_id cortina_gpon_of_match[] = {
	{ .compatible = "realtek,rtl9607f-gpon" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_gpon_of_match);

static struct platform_driver cortina_gpon_driver = {
	.probe	= cortina_gpon_probe,
	.remove	= cortina_gpon_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= cortina_gpon_of_match,
	},
};
module_platform_driver(cortina_gpon_driver);

MODULE_DESCRIPTION("Cortina-Access GPON MAC driver for Realtek RTL9607F Elnath");
MODULE_LICENSE("GPL");
