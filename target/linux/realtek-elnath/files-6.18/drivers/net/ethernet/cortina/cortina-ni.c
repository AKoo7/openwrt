// SPDX-License-Identifier: GPL-2.0
/*
 * Cortina-Access NI (network engine) Ethernet driver for the Realtek
 * RTL9607F "Elnath" GPON SoC.
 *
 * Bring-up stage M2a: platform probe, register-window mapping,
 * reserved-memory resolution, internal MDIO bus and GbE PHY discovery.
 * Stage M2b adds the TX datapath (cortina-ni-tx.c).
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_reserved_mem.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "cortina-ni.h"

/* CA_NI_DRV_NAME now lives in cortina-ni.h: `ethtool -i` reports it too, so
 * the driver name had to stop being a literal private to this file. */

struct cortina_ni_window_desc {
	u8		idx;
	const char	*name;
	bool		required;
};

/* Only the windows this stage touches; the rest are mapped in M2c. */
static const struct cortina_ni_window_desc cortina_ni_windows[] = {
	{ CA_NI_WIN_NI,		"ni-core",	true  },
	{ CA_NI_WIN_NE_INTR,	"ne-intr",	true  },
	{ CA_NI_WIN_MDIO,	"mdio",		true  },
	{ CA_NI_WIN_XRAM,	"xram",		false },
	{ CA_NI_WIN_DMA,	"dma-ldma",	true  },	/* TX rings */
	{ CA_NI_WIN_SRAM,	"bd-sram",	false },
	{ CA_NI_WIN_GLB,	"glb",		true  },
	{ CA_NI_WIN_AXI_REO,	"axi-reo",	true  },	/* RMU AXI rd/wr reorder - RX dequeue DMA */
	/* ★ FBM (Free Buffer Manager) + LDMA-aux windows: the RMU allocates a buffer to
	 * DMA each admitted RX frame into, and that alloc routes through the FBM.  Left
	 * unmapped/uninitialised, the RMU can't allocate -> never admits (0x6900=0,
	 * wptr=0) - the same whole-window-missing class as axi-reo.  DTS reg idx 16 +
	 * 18-21 are already present; map non-required so a DTS gap can't kill probe. */
	{ CA_NI_WIN_LDMA_AUX,	"ldma-aux",	false },
	{ CA_NI_WIN_FBM_GLB,	"fbm-glb",	false },
	{ CA_NI_WIN_FBM_AXI,	"fbm-axi",	false },
	{ CA_NI_WIN_FBM_CPU,	"fbm-cpu",	false },
	{ CA_NI_WIN_FBM_POOL,	"fbm-pool",	false },
	{ CA_NI_WIN_GPHY,	"gphy",		false },
	{ CA_NI_WIN_GPHY_WRAP,	"gphy-wrap",	false },
};

static bool cortina_ni_phy_is_internal(int addr)
{
	return addr >= CA_NI_GPHY_FIRST &&
	       addr < CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT;
}

/*
 * External MDIO master, clause 22 (stock aal_mdio_read/write_direct):
 * write the command word, pulse CTRL.start, poll CTRL.done, then - for a
 * read - fetch RDDATA *before* acking done (W1C), matching stock order.
 */
static int cortina_ni_mdio_c22_cmd(struct cortina_ni *ni, u32 cmd, u32 *data)
{
	void __iomem *base = ni->win[CA_NI_WIN_MDIO];
	u32 ctrl;
	int ret;

	writel(cmd, base + CA_NI_MDIO_ADDR);
	/*
	 * Kick the frame.  Writing DONE too clears any stale done latch
	 * (stock ORs START into the current CTRL value, which has the same
	 * effect when done was left set).
	 */
	writel(CA_NI_MDIO_CTRL_START | CA_NI_MDIO_CTRL_DONE,
	       base + CA_NI_MDIO_CTRL);

	ret = readl_poll_timeout(base + CA_NI_MDIO_CTRL, ctrl,
				 ctrl & CA_NI_MDIO_CTRL_DONE,
				 1, CA_NI_MDIO_TIMEOUT_US);
	if (ret)
		return ret;

	if (data)
		*data = readl(base + CA_NI_MDIO_RDDATA) & 0xffff;

	/* ack: done is write-1-clear */
	writel(ctrl | CA_NI_MDIO_CTRL_DONE, base + CA_NI_MDIO_CTRL);
	return 0;
}

static int cortina_ni_mdio_master_read(struct cortina_ni *ni, int addr,
				       int regnum)
{
	u32 cmd, data;
	int ret;

	cmd = FIELD_PREP(CA_NI_MDIO_ADDR_PHY, addr) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_REG, regnum) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_OP, CA_NI_MDIO_OP_RD) |
	      CA_NI_MDIO_ADDR_RD_WR;

	ret = cortina_ni_mdio_c22_cmd(ni, cmd, &data);
	if (ret)
		return ret;

	return data;
}

static int cortina_ni_mdio_master_write(struct cortina_ni *ni, int addr,
					int regnum, u16 val)
{
	void __iomem *base = ni->win[CA_NI_WIN_MDIO];
	u32 cmd;

	cmd = FIELD_PREP(CA_NI_MDIO_ADDR_PHY, addr) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_REG, regnum) |
	      FIELD_PREP(CA_NI_MDIO_ADDR_OP, CA_NI_MDIO_OP_WR);

	writel(val, base + CA_NI_MDIO_WRDATA);
	return cortina_ni_mdio_c22_cmd(ni, cmd, NULL);
}

/*
 * Internal quad GbE PHYs: registers are memory-mapped in the GPHY window,
 * one 256K bank per PHY.  Regs 0x10..0x17 are banked by the (software)
 * page-select shadow; all other MII regs live at a fixed offset.
 */
static u32 cortina_ni_gphy_reg(struct cortina_ni *ni, int addr, int regnum)
{
	u32 off = (addr - CA_NI_GPHY_FIRST) * CA_NI_GPHY_BANK_STRIDE;

	if (regnum >= CA_NI_GPHY_PAGED_FIRST &&
	    regnum <= CA_NI_GPHY_PAGED_LAST)
		off += ni->gphy_page[addr - CA_NI_GPHY_FIRST] *
			       CA_NI_GPHY_PAGE_STRIDE +
		       (regnum - CA_NI_GPHY_PAGED_FIRST) *
			       CA_NI_GPHY_REG_STRIDE;
	else
		off += CA_NI_GPHY_MII_BASE + regnum * CA_NI_GPHY_REG_STRIDE;

	return off;
}

static int cortina_ni_gphy_read(struct cortina_ni *ni, int addr, int regnum)
{
	if (regnum == 0x1f)	/* page select: software shadow only */
		return ni->gphy_page[addr - CA_NI_GPHY_FIRST];

	return readl(ni->win[CA_NI_WIN_GPHY] +
		     cortina_ni_gphy_reg(ni, addr, regnum)) & 0xffff;
}

static int cortina_ni_gphy_write(struct cortina_ni *ni, int addr, int regnum,
				 u16 val)
{
	if (regnum == 0x1f)
		ni->gphy_page[addr - CA_NI_GPHY_FIRST] = val;
	else
		writel(val, ni->win[CA_NI_WIN_GPHY] +
			    cortina_ni_gphy_reg(ni, addr, regnum));

	/* stock delays 1 ms after every internal-PHY write */
	usleep_range(1000, 1500);
	return 0;
}


static u16 cortina_ni_gphy_ocp_read(void __iomem *bank, u16 ocp)
{
	return readl(bank + CA_NI_GPHY_OCP(ocp)) & 0xffff;
}

static void cortina_ni_gphy_ocp_write(void __iomem *bank, u16 ocp, u16 val)
{
	writel(val, bank + CA_NI_GPHY_OCP(ocp));
}

/* ★ 2026-08-08 AOT5221ZY: see the bisect note at the call site in the probe. */
static bool skip_mdio;
module_param(skip_mdio, bool, 0444);
MODULE_PARM_DESC(skip_mdio,
		 "skip the MDIO bus + internal-PHY scan (AOT5221ZY SError bisect; the GPON WAN path does not need the LAN PHYs)");


static int cortina_ni_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct cortina_ni *ni = bus->priv;

	regnum &= 0x1f;
	if (cortina_ni_phy_is_internal(addr) && ni->win[CA_NI_WIN_GPHY])
		return cortina_ni_gphy_read(ni, addr, regnum);

	return cortina_ni_mdio_master_read(ni, addr, regnum);
}

static int cortina_ni_mdio_write(struct mii_bus *bus, int addr, int regnum,
				 u16 val)
{
	struct cortina_ni *ni = bus->priv;

	regnum &= 0x1f;
	if (cortina_ni_phy_is_internal(addr) && ni->win[CA_NI_WIN_GPHY])
		return cortina_ni_gphy_write(ni, addr, regnum, val);

	return cortina_ni_mdio_master_write(ni, addr, regnum, val);
}

/*
 * GPHY-wrapper enable (stock aal_mdio_global_init + the patch_phy_done bit
 * from aal_internal_phy_init).  We FORCE the stock golden EN0/EN1 rather than
 * OR-onto-U-Boot: EN1 bit12 (patch_phy_done) is the GPHY->port-MAC datapath
 * release, and U-Boot leaves it non-deterministically set -> the whole
 * "RX works some boots, dead others" bug.  Writing the exact golden values
 * (EN0 = 0xFF000000, EN1 = 0x1001) makes ingress deterministic every boot.
 * The internal PHYs are memory-mapped (win GPHY), so forcing these MDIO-OCP
 * bits does not disturb our register reads.
 */
static void cortina_ni_mdio_hw_enable(struct cortina_ni *ni)
{
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];

	if (!wrap)
		return;

	writel(CA_NI_GPHY_WRAP_EN0_VAL, wrap + CA_NI_GPHY_WRAP_EN0);
	writel(CA_NI_GPHY_WRAP_EN1_VAL, wrap + CA_NI_GPHY_WRAP_EN1);
	usleep_range(1000, 1500);
	dev_info(ni->dev, "GPHY wrapper: EN0=0x%08x EN1=0x%08x (patch_phy_done set)\n",
		 readl(wrap + CA_NI_GPHY_WRAP_EN0),
		 readl(wrap + CA_NI_GPHY_WRAP_EN1));
}

/*
 * ★fix#138: faithful port of the vendor aal_internal_phy_init (Track A
 * ca-ne/el/aal_phy.c) - the internal quad-GPHY bring-up - done entirely via the
 * MEMORY-MAPPED GPHY path (cortina_ni_gphy_*), which NEVER touches the MDIO bus
 * whose auto-scan SErrors on this AOT-5221Zy revision.  Vendor order:
 *   1. aal_mdio_global_init + patch_phy_done: enable the GPHY wrapper (EN0/EN1),
 *      which turns on the GPHY->port-MAC datapath for ALL four ports.
 *   2. aal_internal_phy_patch: stream the SRAM firmware patch to all 4 banks.
 *   3. POWER UP each PHY: the vendor does `data = MDIO_READ(phy,0); data &= 0xf7ff;
 *      MDIO_WRITE(phy,0,data)` for phy 1..4 - clearing BMCR(reg0) bit11 (power-down)
 *      so EVERY LAN jack's PHY comes up, not just the one U-Boot left enabled.
 *      (0xf7ff keeps bit9 clear, so no auto-neg restart / no link bounce.)
 * The vendor's optional rtct/green/force-100m/uC sub-patches (EEE + cable-diag)
 * are NOT ported here - they are enhancements, not required for link.
 */
static void cortina_ni_internal_phy_init(struct cortina_ni *ni)
{
	void __iomem *wrap = ni->win[CA_NI_WIN_GPHY_WRAP];
	int p;

	if (wrap) {
		u32 c1 = readl(wrap + CA_NI_GPHY_WRAP_CTRL1);

		writel((c1 & ~CA_NI_GPHY_WRAP_CTRL1_PWRUP) |
		       CA_NI_GPHY_WRAP_CTRL1_PWRUP, wrap + CA_NI_GPHY_WRAP_CTRL1);
		usleep_range(1000, 1500);
	}

	cortina_ni_mdio_hw_enable(ni);	/* EN0/EN1: ocp_sel + patch_done + rst_gphy=0xF */

	/*
	 * ★fix#141b (2026-08-19): the CTRL1 PWRUP(0xF) + EN1 rst_gphy[7:4]=0xF
	 * above are the whole NON-DESTRUCTIVE job here — they un-poison the
	 * DIGITAL cores of banks 1..3 (regs.h:361) so cal_save/ID reads succeed
	 * for all four; the decoupled bring-up (cortina_ni_rx_link_up ->
	 * gphy_patch_and_resume + gphy_intf_establish, at link-up) then patches
	 * and links every port exactly as image 210 does.
	 *
	 * ★ REMOVED (was the physical-LINK regression): the per-port
	 * CA_NI_HV_INTF_RST_GPHY(p) teardown loop.  Pulsing INTF_RST on port 0 —
	 * the port U-Boot had already CALIBRATED and TFTP'd over — tore down its
	 * live ANALOG/PCS page (0xbxx) into 0xbad0 poison.  Two probe-order facts
	 * then made it permanent: cortina_ni_rx_gphy_cal_save() (run right after
	 * this, in rx_probe) snapshotted the POISONED cal, and the probe-time
	 * patch_and_resume below consumed the gphy_patched[] one-shot so link-up's
	 * clean patch never re-ran — so gphy_intf_establish restored 0xbad0 back
	 * into port 0's analog cal and the line side never linked (peer carrier=0).
	 * The digital ID read on OCP page 0xa4x still ACK'd (=VALID), which masked
	 * it.  Not resetting the analog domain here keeps port 0's U-Boot cal (fault
	 * 0x0000), which is what makes all four jacks link.
	 */

	/*
	 * DIAGNOSTIC (fix#141): the reset-release effect BEFORE the firmware
	 * patch and BEFORE any link-up wrap re-establish.  Demoted to dev_info
	 * now that the 4-GPHY bring-up is proven (all cores read VALID).
	 */
	if (wrap) {
		dev_info(ni->dev, "FIX141 after reset-release: CTRL13=0x%08x\n",
			readl(wrap + CA_NI_GPHY_WRAP_EN1));
		for (p = CA_NI_GPHY_FIRST;
		     p < CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT; p++) {
			int id1 = cortina_ni_gphy_read(ni, p, 2);
			int id2 = cortina_ni_gphy_read(ni, p, 3);

			dev_info(ni->dev, "FIX141 PHY%d ID=0x%04x:0x%04x %s\n",
				p, id1, id2,
				(id1 == 0x001c) ? "VALID" : "POISON");
		}
	}

	/* ★fix#141b: do NOT patch_and_resume() here — that consumes the
	 * gphy_patched[] one-shot so the link-up decoupled path (which snapshots
	 * cal, holds the uC, streams SRAM fresh and establishes the interface)
	 * would no-op.  Likewise do NOT force BMCR power-up here.  Both now run at
	 * link-up (cortina_ni_rx_link_up), AFTER cortina_ni_rx_gphy_cal_save() has
	 * captured port 0's intact U-Boot analog cal — so the analog domain relocks
	 * and the line side links. */
	dev_info(ni->dev, "FIX141b internal-PHY cores powered (patch/BMCR/link deferred to link-up)\n");
}

static int cortina_ni_map_windows(struct cortina_ni *ni)
{
	struct device *dev = ni->dev;
	struct device_node *np = dev->of_node;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_windows); i++) {
		const struct cortina_ni_window_desc *w = &cortina_ni_windows[i];
		struct resource res;

		ret = of_address_to_resource(np, w->idx, &res);
		if (ret || !resource_size(&res)) {
			if (w->required)
				return dev_err_probe(dev, ret ? ret : -EINVAL,
						     "no reg entry %u (%s)\n",
						     w->idx, w->name);
			dev_warn(dev, "window %u (%s) absent, skipping\n",
				 w->idx, w->name);
			continue;
		}

		ni->win[w->idx] = devm_ioremap(dev, res.start,
					       resource_size(&res));
		if (!ni->win[w->idx]) {
			if (w->required)
				return dev_err_probe(dev, -ENOMEM,
						     "cannot map window %u (%s) %pR\n",
						     w->idx, w->name, &res);
			dev_warn(dev, "cannot map window %u (%s) %pR\n",
				 w->idx, w->name, &res);
			continue;
		}
		ni->winsz[w->idx] = resource_size(&res);	/* peek bounds */

		dev_info(dev, "window %u (%s): %pR mapped\n",
			 w->idx, w->name, &res);
	}

	/*
	 * Stock ca_ne_reg_init also unconditionally maps a 4K peripheral
	 * block at (32-bit) 0xf4329000, not described in the DT.
	 */
	ni->peri = devm_ioremap(ni->dev, CA_NI_PERI_PHYS, CA_NI_PERI_SIZE);
	if (!ni->peri)
		return dev_err_probe(dev, -ENOMEM,
				     "cannot map peri block @%#x\n",
				     CA_NI_PERI_PHYS);
	dev_info(dev, "peri block @%#x (4K) mapped\n", CA_NI_PERI_PHYS);

	/* window-map liveness proof: GLB+0 is a read-only chip/JTAG ID */
	dev_info(dev, "GLB chip id: 0x%08x\n",
		 readl(ni->win[CA_NI_WIN_GLB] + CA_NI_GLB_JTAG_ID));

	return 0;
}

static void cortina_ni_log_reserved_mem(struct cortina_ni *ni)
{
	struct device_node *np = ni->dev->of_node;
	int i;

	/* two DDR pools: coherent buffer + noncache packet-buffer pool */
	for (i = 0; i < 2; i++) {
		struct device_node *rn;
		struct reserved_mem *rmem;

		rn = of_parse_phandle(np, "memory-region", i);
		if (!rn) {
			dev_warn(ni->dev,
				 "memory-region %d absent (needed from M2b)\n",
				 i);
			continue;
		}

		rmem = of_reserved_mem_lookup(rn);
		if (rmem)
			dev_info(ni->dev,
				 "reserved pool %d (%s): base %pa size %pa\n",
				 i, rmem->name, &rmem->base, &rmem->size);
		else
			dev_warn(ni->dev,
				 "memory-region %d (%pOF) not in reserved-memory\n",
				 i, rn);
		of_node_put(rn);
	}
}

static void cortina_ni_scan_phys(struct cortina_ni *ni)
{
	int addr;

	for (addr = CA_NI_GPHY_FIRST;
	     addr < CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT; addr++) {
		int id1, id2;
		u32 id;

		id1 = mdiobus_read(ni->mii, addr, MII_PHYSID1);
		id2 = mdiobus_read(ni->mii, addr, MII_PHYSID2);
		if (id1 < 0 || id2 < 0) {
			dev_warn(ni->dev, "PHY %d: read failed (%d/%d)\n",
				 addr, id1, id2);
			continue;
		}

		id = (u32)id1 << 16 | id2;
		dev_info(ni->dev, "PHY %d: id 0x%08x (%s, expected 0x%08x)\n",
			 addr, id,
			 (id & CA_NI_GPHY_PHY_ID_MASK) ==
			 (CA_NI_GPHY_PHY_ID & CA_NI_GPHY_PHY_ID_MASK) ?
			 "match" : "MISMATCH",
			 CA_NI_GPHY_PHY_ID);

		/*
		 * Diagnostic A/B: if the memory-mapped GPHY path returned
		 * nothing sane, also try the external MDIO master once so
		 * the boot log settles which path reaches these PHYs.
		 */
		if ((id == 0 || id == 0xffffffff) && ni->win[CA_NI_WIN_GPHY]) {
			id1 = cortina_ni_mdio_master_read(ni, addr,
							  MII_PHYSID1);
			id2 = cortina_ni_mdio_master_read(ni, addr,
							  MII_PHYSID2);
			dev_info(ni->dev,
				 "PHY %d via MDIO master: id1=%d id2=%d\n",
				 addr, id1, id2);
		}
	}
}

static int cortina_ni_mdio_init(struct cortina_ni *ni)
{
	struct device *dev = ni->dev;
	struct device_node *mdio_np;
	struct mii_bus *bus;
	int ret;

	bus = devm_mdiobus_alloc(dev);
	if (!bus)
		return -ENOMEM;

	bus->name = CA_NI_DRV_NAME "-mdio";
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));
	bus->parent = dev;
	bus->priv = ni;
	bus->read = cortina_ni_mdio_read;
	bus->write = cortina_ni_mdio_write;
	/* limit auto-scan to the internal PHYs at addresses 1..4 */
	bus->phy_mask = ~(u32)GENMASK(CA_NI_GPHY_FIRST + CA_NI_GPHY_COUNT - 1,
				      CA_NI_GPHY_FIRST);

	cortina_ni_mdio_hw_enable(ni);

	/*
	 * Do NOT reset the PHYs here: U-Boot already brought the TFTP port's
	 * link up and we want to keep it provable at this stage.
	 */
	mdio_np = of_get_child_by_name(dev->of_node, "mdio");
	ret = devm_of_mdiobus_register(dev, bus, mdio_np);
	of_node_put(mdio_np);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register MDIO bus\n");

	ni->mii = bus;
	return 0;
}

/* ------------------------------------------------------------------ */
/* /proc/cortina_ni_peek: dump ARBITRARY window registers, read-only.  */
/* Decisive good-vs-bad-boot diff tool - every RX-subset register reads */
/* identical on a working and a broken boot, so the deciding bit lives  */
/* in a register the fixed spy does not print.  Bounded, plain readl,   */
/* no side effects.  (project rule: dump/spy stays first-class.)        */
/* ------------------------------------------------------------------ */

/* window name -> selector.  "peri" is the non-DT 4K @0xf4329000 block. */
static const struct {
	const char	*name;
	u8		win;
} cortina_ni_peek_wins[] = {
	{ "ni",		CA_NI_WIN_NI		},
	{ "dma",	CA_NI_WIN_DMA		},
	{ "glb",	CA_NI_WIN_GLB		},
	{ "gphy",	CA_NI_WIN_GPHY		},
	{ "wrap",	CA_NI_WIN_GPHY_WRAP	},
	{ "reo",	CA_NI_WIN_AXI_REO	},
	{ "mdio",	CA_NI_WIN_MDIO		},
	{ "intr",	CA_NI_WIN_NE_INTR	},
	{ "sgmii",	CA_NI_WIN_SGMII_PCS	},
	{ "peri",	CA_NI_PEEK_PERI		},
};

/* resolve a peek window selector to {base, size}; NULL base = unavailable */
static void __iomem *cortina_ni_peek_base(struct cortina_ni *ni, u8 win,
					  size_t *size)
{
	if (win == CA_NI_PEEK_PERI) {
		*size = CA_NI_PERI_SIZE;
		return ni->peri;
	}
	if (win >= CA_NI_WIN_COUNT)
		return NULL;
	*size = ni->winsz[win];
	return ni->win[win];
}

static int cortina_ni_peek_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_peek q = ni->peek;	/* snapshot */
	void __iomem *base;
	size_t size;
	const char *name = "?";
	u32 i;

	for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_wins); i++)
		if (cortina_ni_peek_wins[i].win == q.win)
			name = cortina_ni_peek_wins[i].name;

	base = cortina_ni_peek_base(ni, q.win, &size);
	if (!base) {
		seq_printf(m, "%s: window unavailable\n", name);
		return 0;
	}
	if (!q.count) {
		seq_puts(m,
			 "usage: echo '[win] <hex_off> [count]' > /proc/cortina_ni_peek\n"
			 "  win = ni|dma|glb|gphy|wrap|reo|mdio|intr|sgmii|peri (default ni)\n"
			 "  gphy off is raw in the 1M window (port p bank = p*0x40000)\n"
			 "  poke: echo 'poke [win] <hex_off> <hex_val>' (SPB/SPKTP holes refused)\n");
		return 0;
	}

	for (i = 0; i < q.count; i++) {
		u32 off = q.off + i * 4;

		if (off + 4 > size) {
			seq_printf(m, "%s+0x%05x = <out of range, size=0x%zx>\n",
				   name, off, size);
			break;
		}
		seq_printf(m, "%s+0x%05x = 0x%08x\n", name, off,
			   readl(base + off));
	}
	return 0;
}

static int cortina_ni_peek_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_peek_show, pde_data(inode));
}

static ssize_t cortina_ni_peek_write(struct file *file, const char __user *ubuf,
				     size_t len, loff_t *ppos)
{
	struct cortina_ni *ni = pde_data(file_inode(file));
	char buf[64], *p, *tok;
	u8 win = CA_NI_WIN_NI;
	u32 off, count = 1;
	int i;

	if (len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	p = strim(buf);

	tok = strsep(&p, " \t");
	if (!tok || !*tok)
		return -EINVAL;

	/* 'poke' verb: WRITE a register for fast live RE iteration (a boot is
	 * ~200s; a poke is instant).  usage:
	 *   echo 'poke [win] <hex_off> <hex_val>' > /proc/cortina_ni_peek
	 * The read-back is armed to the poked reg, so a follow-up cat shows it.
	 * (project rule: dump/spy/poke stays a first-class, always-on feature.) */
	if (!strcmp(tok, "poke")) {
		void __iomem *base;
		size_t size;
		u32 val;

		tok = p ? strsep(&p, " \t") : NULL;
		if (!tok || !*tok)
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_wins); i++)
			if (!strcmp(tok, cortina_ni_peek_wins[i].name)) {
				win = cortina_ni_peek_wins[i].win;
				tok = p ? strsep(&p, " \t") : NULL;
				break;
			}
		if (!tok || !*tok || kstrtou32(tok, 16, &off))
			return -EINVAL;
		tok = p ? strsep(&p, " \t") : NULL;
		if (!tok || !*tok || kstrtou32(tok, 16, &val))
			return -EINVAL;
		if (off & 3)
			return -EINVAL;			/* 32-bit aligned only */
		/* crash-hole guard: the L3 special-packet block (SPKTP 0x333c/40,
		 * SPB 0x3440/44) is UNMAPPED on RTL9607F silicon - a write there
		 * hangs the CPU and async-SErrors the board (proven on live stock).
		 * Refuse those four; every other 0x32xx-0x33xx reg is real.  */
		if (win == CA_NI_WIN_NI &&
		    (off == 0x333c || off == 0x3340 ||
		     off == 0x3440 || off == 0x3444))
			return -EPERM;
		base = cortina_ni_peek_base(ni, win, &size);
		if (!base)
			return -ENODEV;
		if (off + 4 > size)
			return -ERANGE;
		writel(val, base + off);
		ni->peek.win = win;
		ni->peek.off = off;
		ni->peek.count = 1;
		return len;
	}

	/* optional leading window name (non-hex first token) */
	for (i = 0; i < ARRAY_SIZE(cortina_ni_peek_wins); i++)
		if (!strcmp(tok, cortina_ni_peek_wins[i].name)) {
			win = cortina_ni_peek_wins[i].win;
			tok = p ? strsep(&p, " \t") : NULL;
			break;
		}
	if (!tok || !*tok)
		return -EINVAL;

	if (kstrtou32(tok, 16, &off))
		return -EINVAL;
	tok = p ? strsep(&p, " \t") : NULL;
	if (tok && *tok && kstrtou32(tok, 0, &count))
		return -EINVAL;

	if (off & 3)
		return -EINVAL;			/* 32-bit aligned only */
	count = clamp_t(u32, count, 1, CA_NI_PEEK_MAX);

	ni->peek.win = win;
	ni->peek.off = off;
	ni->peek.count = count;
	return len;
}

static const struct proc_ops cortina_ni_peek_pops = {
	.proc_open	= cortina_ni_peek_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= cortina_ni_peek_write,
};

/* ------------------------------------------------------------------ */
/* /proc/cortina_ni_gsram: read the internal-GPHY uC SRAM (0x8xxx words) */
/* for the ours-vs-stock firmware diff, read-only.  Same access the      */
/* operator used on stock: OCP 0xa436=addr, read OCP 0xa438=data.  The uC */
/* is best-effort held around the dump (stock reads fine while held).     */
/* ------------------------------------------------------------------ */
static int cortina_ni_gsram_show(struct seq_file *m, void *v)
{
	struct cortina_ni *ni = m->private;
	struct cortina_ni_gsram q = ni->gsram;	/* snapshot */
	void __iomem *win = ni->win[CA_NI_WIN_GPHY];
	void __iomem *bank;
	u32 i;

	if (!win) {
		seq_puts(m, "gphy window unavailable\n");
		return 0;
	}
	if (!q.count) {
		seq_puts(m,
			 "usage: echo '<bank> <hex_start> <count>' > /proc/cortina_ni_gsram\n"
			 "  bank 0..3, start = SRAM word addr (hex, e.g. 8000), count 1..1024\n");
		return 0;
	}
	if (q.bank >= CA_NI_GPHY_COUNT) {
		seq_printf(m, "bank %u out of range\n", q.bank);
		return 0;
	}

	bank = win + CA_NI_GPHY_BANK(q.bank);

	/* best-effort hold around the whole dump; no ready-poll (read-only) */
	writel(readl(bank + CA_NI_GPHY_LOCK) | CA_NI_GPHY_LOCK_HOLD,
	       bank + CA_NI_GPHY_LOCK);

	for (i = 0; i < q.count; i++) {
		u16 addr = q.start + i;

		cortina_ni_gphy_ocp_write(bank, CA_NI_GPHY_SRAM_ADDR, addr);
		seq_printf(m, "0x%04x=0x%04x\n", addr,
			   cortina_ni_gphy_ocp_read(bank, CA_NI_GPHY_SRAM_DATA));
	}

	writel(readl(bank + CA_NI_GPHY_LOCK) & ~CA_NI_GPHY_LOCK_HOLD,
	       bank + CA_NI_GPHY_LOCK);
	return 0;
}

static int cortina_ni_gsram_open(struct inode *inode, struct file *file)
{
	return single_open(file, cortina_ni_gsram_show, pde_data(inode));
}

static ssize_t cortina_ni_gsram_write(struct file *file, const char __user *ubuf,
				      size_t len, loff_t *ppos)
{
	struct cortina_ni *ni = pde_data(file_inode(file));
	char buf[64], *p, *tok;
	u32 bank, start, count;

	if (len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	p = strim(buf);

	tok = strsep(&p, " \t");
	if (!tok || !*tok || kstrtou32(tok, 0, &bank))
		return -EINVAL;
	tok = p ? strsep(&p, " \t") : NULL;
	if (!tok || !*tok || kstrtou32(tok, 16, &start))
		return -EINVAL;
	tok = p ? strsep(&p, " \t") : NULL;
	if (!tok || !*tok || kstrtou32(tok, 0, &count))
		return -EINVAL;

	if (bank >= CA_NI_GPHY_COUNT || start > 0xffff)
		return -EINVAL;
	count = clamp_t(u32, count, 1, CA_NI_GSRAM_MAX);

	ni->gsram.bank = bank;
	ni->gsram.start = start;
	ni->gsram.count = count;
	return len;
}

static const struct proc_ops cortina_ni_gsram_pops = {
	.proc_open	= cortina_ni_gsram_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= cortina_ni_gsram_write,
};

/* Pulse one NE block reset: assert (set bit) -> 1ms -> deassert (clear bit),
 * stock ca_ni_global_reset order.  Returns the register value read back WHILE
 * asserted so a write that does not stick (e.g. a secure-only register) shows
 * up in the log. */
static u32 cortina_ni_block_reset_pulse(void __iomem *rst, u32 bit)
{
	u32 held;

	writel(readl(rst) | bit, rst);		/* assert  */
	held = readl(rst);			/* read-back while held */
	usleep_range(1000, 2000);		/* stock mdelay(1) */
	writel(readl(rst) & ~bit, rst);		/* deassert */
	return held;
}

/*
 * Bring up the L3QM block.  Stock ca_ni_global_reset pulses the NE block
 * resets (assert -> 1ms -> deassert, per block) so each re-runs its internal
 * init and asserts its *_init_done.  U-Boot brings up only the TX/NI path for
 * TFTP and leaves the QM un-inited, so QM_PHY_PORT_STS.qm_init_done reads 0,
 * the empty-buffer pools never activate and the CPU-push FIFO never drains.
 * A TQM-only pulse was NOT enough; nor was L2FE+L2TM+L3FE+TQM (the reset fired
 * - held bits read back correctly - but qm_init_done stayed 0).  QM_PHY_PORT_STS
 * is dominated by NI<->QM handshake bits (nirx_qm_rdy, qm_nitx_rdy, nitx_qm_vld,
 * te_qm_es_ni_ok), so the QM only completes init once the NI is reset alongside
 * it.  Match stock ca_ni_global_reset EXACTLY: pulse NI+L2FE+L2TM+L3FE+TQM in
 * that order (SDRAM is skipped - it is DRAM).  Resetting NI drops U-Boot's
 * FE-bypass direct-TX, but cortina_ni_tx_hw_init() (run right after, in
 * tx_probe) fully rebuilds the NI/TX path - same reset->full-init order stock's
 * ca_init uses.  Direct MMIO via the GLB window (same as the dphy_rst release).
 */
static void __maybe_unused cortina_ni_qm_reset(struct cortina_ni *ni)
{
	void __iomem *rst;
	u32 h_ni, h_tqm;

	if (!ni->win[CA_NI_WIN_GLB])
		return;
	rst = ni->win[CA_NI_WIN_GLB] + CA_NI_GLB_BIST_CONTROL4;

	/* ★ Reset ONLY NI + TQM, NOT L2FE/L2TM/L3FE.  Resetting the forwarding
	 * blocks and NOT re-initialising them (aal_l2fe_init/aal_arb_init/
	 * aal_l2_tm_init - a huge, table-heavy init we do not port) leaves them
	 * reset-but-dead: their indirect-access engines hang (REDIR_LDPID/FIB
	 * writes never complete, GO stuck) and they make no forwarding decision,
	 * so LAN ingress never reaches the CPU.  The boot ROM / U-Boot already
	 * brings L2FE/L2TM/L3FE up (U-Boot RX works), so leave them running and
	 * only reset the two blocks we DO fully reconfigure: NI (rebuilt by
	 * cortina_ni_tx_hw_init) and the TQM/QM (our EQ/EPP/ring config). */
	h_ni  = cortina_ni_block_reset_pulse(rst, CA_NI_GLB_RST_NI);
	h_tqm = cortina_ni_block_reset_pulse(rst, CA_NI_GLB_RST_TQM);
	usleep_range(100000, 110000);		/* stock trailing mdelay(100) */

	dev_info(ni->dev,
		 "NE reset pulsed (held ni=0x%08x tqm=0x%08x; L2FE/L2TM/L3FE left at boot init; now=0x%08x)\n",
		 h_ni, h_tqm, readl(rst));
}


static int cortina_ni_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cortina_ni *ni;
	int ret;

	ni = devm_kzalloc(dev, sizeof(*ni), GFP_KERNEL);
	if (!ni)
		return -ENOMEM;
	ni->dev = dev;
	/* guards the ONE reader of the read-and-clear NI_HV counters; must be
	 * live before anything can sample them (the rx and l3fe /proc nodes and
	 * `ethtool -S` all go through it) */
	spin_lock_init(&ni->nihv_lock);
	platform_set_drvdata(pdev, ni);

	ret = cortina_ni_map_windows(ni);
	if (ret)
		return ret;

	/*
	 * ★ Release the internal-GPHY <-> NI datapath sub-blocks from reset
	 * FIRST, before any GPHY/MAC bring-up (release-then-init order).
	 * U-Boot leaves GLOBAL_DPHY_RESET (GLB+0xa0) with datapath sub-blocks
	 * held in reset (0x50302340); stock releases them to 0x10000000.  With
	 * them held, NO frame crosses the internal GMII on any port even though
	 * the GPHY line side links - the bidirectional-dead gate.  A late
	 * release (at link-up) sticks but does not re-init the sub-block, so we
	 * must do it here at probe entry, ahead of MDIO/PHY/MAC init.
	 */
	if (ni->win[CA_NI_WIN_GLB]) {
		void __iomem *glb = ni->win[CA_NI_WIN_GLB];
		/* ★★ build64: the EXACT stock ca_ni_global_reset sequence (vendor source): the NE
		 * core resets done SEQUENTIAL, ONE-AT-A-TIME, IN ORDER - ni, l2fe, l2tm, l3fe, tqm -
		 * each a 1ms assert->deassert; tqm(bit5) LAST, after its deps ni/l2fe/l2tm/l3fe are
		 * back up.  SKIP sdram(bit4, resetting it kills DRAM) and ptp.  cortina,rst-mgr @
		 * glb+0xa0 is a plain bit set/clear (no clock/delay in the reg), so a direct poke is
		 * faithful; the ORDER + one-at-a-time + delays are what matter (build60's combined
		 * ni+tqm booted but the engine stayed dead - wrong SHAPE).  This runs right after the
		 * glb window is mapped, before any NI-core access.  First release the U-Boot-held DPHY
		 * datapath sub-blocks (0x10000000 resting = the original bidirectional-dead fix), then
		 * run the sequence (read fresh each iter to preserve the other bits). */
		static const int order[] = { 0, 1, 2, 3, 5 };	/* ni,l2fe,l2tm,l3fe,tqm (SKIP 4=sdram) */
		u32 was = readl(glb + CA_NI_GLB_BLOCK_RESET);
		int j;

		/* ★ fix #14: the DPHY sub-block release goes to DPHY_RESET (0x0a0),
		 * the per-block sequence below to BLOCK_RESET (0x098) -- two
		 * different registers on this silicon (see cortina-ni-regs.h). */
		writel(CA_NI_GLB_BLOCK_RESET_VAL, glb + CA_NI_GLB_DPHY_RESET);
		for (j = 0; j < ARRAY_SIZE(order); j++) {
			u32 v = readl(glb + CA_NI_GLB_BLOCK_RESET);

			writel(v | BIT(order[j]), glb + CA_NI_GLB_BLOCK_RESET);	/* ASSERT one */
			mdelay(1);
			writel(v & ~BIT(order[j]), glb + CA_NI_GLB_BLOCK_RESET);	/* DEASSERT it */
		}
		mdelay(100);					/* final settle (stock) */
		dev_info(dev, "ne-reset: sequential ni,l2fe,l2tm,l3fe,tqm at probe start (0x%08x -> 0x%08x)\n",
			 was, readl(glb + CA_NI_GLB_BLOCK_RESET));
	}

	/* ★ NE block reset DISABLED (storm bisect test): our L2FE showed a
	 * ~100k/s internal packet storm (sop/eop diverging = looping frames) with
	 * wptr=0.  qm_init_done proved a phantom, so the block reset never had a
	 * real justification - and re-resetting NI/TQM after the boot ROM already
	 * initialised the whole NE datapath can leave the L2FE->TM->QM handoff in
	 * a self-feeding loop.  Run on the pure boot-ROM datapath + only our
	 * QM/EQ/CPU-EPP + forwarding config on top, and see if the storm clears.
	 * (Re-enable cortina_ni_qm_reset(ni) here to A/B the reset.) */

	/*
	 * NOTE: the internal-GPHY firmware patch + uC resume is NOT done here -
	 * at probe the uC is still running (HOLD == 0) and its SRAM is not
	 * writable.  It is deferred to link-up (cortina_ni_rx_link_up ->
	 * cortina_ni_gphy_patch_and_resume), where the uC has entered the held
	 * state after MDIO/GPHY init.
	 */

	cortina_ni_log_reserved_mem(ni);

	/*
	 * ★ 2026-08-08 AOT5221ZY bisect switch: skip the MDIO bus + internal-PHY
	 * scan entirely.  The `Asynchronous SError 0xbe000011` on idle CPU1 lands
	 * ~9 ms after the LAST MDIO PHY-id read, boot after boot, and does not move
	 * when unrelated register maps or the block-reset target change (fixes #13
	 * and #14 both left it at 1.3298/1.3301 s).  This board's GPON WAN datapath
	 * (the US OMCI TX ring) does not need the LAN PHYs at all, so this is both
	 * the isolation test and, if it holds, a usable bring-up path.
	 * Boot with cortina_ni.skip_mdio=1.
	 */
	if (skip_mdio) {
		dev_info(dev, "skip_mdio=1: MDIO bus + internal-PHY scan SKIPPED\n");
		/*
		 * ★fix#137: the MDIO-BUS SCAN (devm_of_mdiobus_register probing
		 * phy@1..4) is what SErrors on this AOT-5221Zy revision - NOT the
		 * memory-mapped GPHY path.  So we STILL bring up the internal quad-
		 * GPHY for all 4 LAN jacks via the mmio path the vendor uses
		 * (aal_mdio_global_init + aal_internal_phy_init), which never
		 * touches the MDIO bus: force the GPHY wrapper EN0/EN1 (EN1 bit12
		 * patch_phy_done = the GPHY->port-MAC datapath for every port), then
		 * stream the SRAM firmware patch to all banks.  Without this, only
		 * the port U-Boot happened to leave enabled passes traffic; with it
		 * all four jacks come up with hardware auto-neg/link.  Both calls are
		 * mmio-only and one-shot-guarded, so they are safe here at probe.
		 */
		cortina_ni_internal_phy_init(ni);
	} else {
		ret = cortina_ni_mdio_init(ni);
		if (ret)
			return ret;

		cortina_ni_scan_phys(ni);
	}

	ret = cortina_ni_tx_probe(ni);
	if (ret)
		return ret;

	ret = cortina_ni_rx_probe(ni);
	if (ret)
		return ret;

	/* ★DSA (lan1..lan4): register the switch now that the conduit netdev
	 * (eth0) and the MDIO bus are up.  Non-fatal — on any non-defer error we
	 * keep the working single-netdev datapath so the board still comes up. */
	ret = cortina_ni_dsa_register(ni);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_warn(dev, "DSA lan1..lan4 not registered (%d); eth0 only\n",
			 ret);

	/* L3FE flow-engine arm + verify (nf_flow_table HW offload, phase 1).
	 * Non-fatal: on any error the offload stays disabled and the normal
	 * datapath is untouched. */
	ret = cortina_ni_flowoffload_probe(ni);
	if (ret)
		dev_warn(dev, "L3FE flow offload disabled (%d)\n", ret);

	/* arbitrary-register peek (good-vs-bad-boot diff tool) */
	proc_create_data("cortina_ni_peek", 0644, NULL, &cortina_ni_peek_pops,
			 ni);
	/* internal-GPHY uC SRAM reader (ours-vs-stock firmware diff) */
	proc_create_data("cortina_ni_gsram", 0644, NULL, &cortina_ni_gsram_pops,
			 ni);
	/* the `ethtool -d` decode map (cortina-ni/regdump_map) */
	cortina_ni_debugfs_init(ni);

	dev_info(dev, "M2c probe complete\n");
	return 0;
}

static void cortina_ni_remove(struct platform_device *pdev)
{
	struct cortina_ni *ni = platform_get_drvdata(pdev);

	cortina_ni_dsa_unregister(ni);
}

static const struct of_device_id cortina_ni_of_match[] = {
	{ .compatible = "cortina,ni-interface" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_ni_of_match);

static struct platform_driver cortina_ni_driver = {
	.probe = cortina_ni_probe,
	.remove = cortina_ni_remove,
	.driver = {
		.name = CA_NI_DRV_NAME,
		.of_match_table = cortina_ni_of_match,
	},
};

/* ------------------------------------------------------------------ */
/* internal quad-GPHY phy_driver: inherit U-Boot's link, NEVER re-aneg  */
/* ------------------------------------------------------------------ */

/*
 * ★ THE determinism gate.  U-Boot brings the internal GPHY up to a stable
 * 1G/full link (it TFTPs our kernel over it) and hands it to Linux working
 * bidirectionally.  Generic phylib, at phy_start, RESTARTS auto-negotiation
 * (genphy_config_aneg -> BMCR restart), which bounces the GPHY LINE link
 * Up->Down->Down->Up over ~5s.  The internal GPHY<->NI-MAC datapath dies
 * across that bounce and never re-establishes - RX+TX both dead, and
 * non-deterministic ("works the boots the bounce timing spares it").  Stock
 * never restarts aneg: it inherits U-Boot's link and only MONITORS it.
 *
 * A no-op config_aneg makes phylib do exactly that: phy_start still runs the
 * state machine and calls read_status (so adjust_link fires once and we adopt
 * the live link + set up the RX datapath), but it writes NO BMCR - so the
 * line link is never bounced and the U-Boot datapath survives into Linux.
 * No .soft_reset / .config_init either (phy_init_hw then never resets the
 * PHY, preserving its U-Boot patch/calibration).  Features fall through to
 * genphy_read_abilities automatically.
 */
static int cortina_ni_gphy_config_aneg(struct phy_device *phydev)
{
	return 0;	/* never touch aneg - keep U-Boot's stable link */
}

static struct phy_driver cortina_ni_gphy_driver[] = { {
	.phy_id		= CA_NI_GPHY_PHY_ID,
	.phy_id_mask	= CA_NI_GPHY_PHY_ID_MASK,
	.name		= "Cortina RTL960x internal GPHY",
	.config_aneg	= cortina_ni_gphy_config_aneg,
	.read_status	= genphy_read_status,
} };

/* ★fix#127: stop the Cortina per-CPU hardware watchdog (block base 0xf432901c, CTRL@+0x00).
 * U-Boot's cortina_wdt arms this HW watchdog (EN|RSTEN) before booting the OS.  Stock's 5.10
 * kernel wires a wdt driver that pets it; this clean-room 6.18 kernel wires none, so nothing
 * kicks it and the SoC SILENTLY hard-resets a few minutes in -- no kernel panic on the console,
 * then U-Boot boots NAND stock.  Writing 0 to CTRL is exactly cortina_wdt_stop() (writel(0,base):
 * clears EN|RSTEN|CLK_SEL).  Done at module_init, which for this built-in driver runs a few
 * seconds into boot -- far ahead of the multi-minute timeout.  Petting (a real wdt driver +
 * procd /dev/watchdog) is the proper follow-up; stopping keeps this bring-up image alive. */
#define CA_WDT_CTRL_PA	0xf432901cUL
static bool wdt_disable = true;
module_param(wdt_disable, bool, 0444);
MODULE_PARM_DESC(wdt_disable, "fix#127: stop the U-Boot-armed Cortina HW watchdog (write 0 to CTRL 0xf432901c) so this wdt-driver-less 6.18 kernel is not silently reset ~minutes in (=0 to A/B)");

static void cortina_ni_wdt_stop(void)
{
	void __iomem *ctrl;

	if (!wdt_disable)
		return;
	ctrl = ioremap(CA_WDT_CTRL_PA, 4);
	if (!ctrl) {
		pr_warn("cortina-ni: fix#127 wdt ioremap(0x%lx) failed; HW watchdog left armed\n",
			CA_WDT_CTRL_PA);
		return;
	}
	writel(0, ctrl);
	iounmap(ctrl);
	pr_info("cortina-ni: fix#127 HW watchdog stopped (CTRL 0x%lx <- 0)\n", CA_WDT_CTRL_PA);
}

static int __init cortina_ni_module_init(void)
{
	int ret;

	cortina_ni_wdt_stop();

	/* register the GPHY driver FIRST so it binds the internal PHYs at MDIO
	 * scan time (during the platform probe), overriding genphy + its aneg */
	ret = phy_drivers_register(cortina_ni_gphy_driver,
				   ARRAY_SIZE(cortina_ni_gphy_driver),
				   THIS_MODULE);
	if (ret)
		return ret;

	ret = platform_driver_register(&cortina_ni_driver);
	if (ret)
		phy_drivers_unregister(cortina_ni_gphy_driver,
				       ARRAY_SIZE(cortina_ni_gphy_driver));
	return ret;
}
#ifdef MODULE
module_init(cortina_ni_module_init);
#else
/* ★ late_initcall (NOT device_initcall) when built-in: the NI driver registers a
 * DSA switch (cortina-ni-dsa.c) whose tagger, net/dsa/tag_cortina, registers at
 * device_initcall.  drivers/ links before net/, so a device_initcall probe runs
 * BEFORE the tagger -> dsa_register_switch() returns -EPROBE_DEFER (dsa.c:1194,
 * -ENOPROTOOPT) -> the deferred_probe core RE-RUNS this probe, which is NOT
 * idempotent (re-proc_create() WARNs, a stale recovery_work UAF-panics, the GPHY
 * reset-release re-runs and poisons a core).  Registering at late_initcall runs
 * this after ALL device_initcalls, so the tagger is present and DSA registers
 * cleanly on the FIRST probe.  The fix#127 wdt-stop still runs seconds into boot,
 * far ahead of its multi-minute timeout. */
late_initcall(cortina_ni_module_init);
#endif

static void __exit cortina_ni_module_exit(void)
{
	platform_driver_unregister(&cortina_ni_driver);
	phy_drivers_unregister(cortina_ni_gphy_driver,
			       ARRAY_SIZE(cortina_ni_gphy_driver));
}
module_exit(cortina_ni_module_exit);

MODULE_DESCRIPTION("Cortina-Access NI Ethernet driver (RTL9607F)");
MODULE_LICENSE("GPL");
