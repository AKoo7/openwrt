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

#define CA_NI_DRV_NAME	"cortina-ni"

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

/*
 * Internal-GPHY firmware (SRAM) patch (see cortina-ni-regs.h).  The on-PHY
 * microcontroller's DSP-SRAM must be seeded with the apro_gen2 firmware body
 * before the PHY forwards a frame across its system-side GMII to the NI MAC.
 * We keep U-Boot's live link (never reset the PHY), so the uC is running - the
 * gate/lock handshake holds it, the firmware image is written word-by-word,
 * then the uC is resumed.  Applied per bank.  The raw OCP accessors are plain
 * memory-mapped readl/writel (no page shadow, no 1 ms MDIO delay - the image
 * is streamed back-to-back).
 */
struct cortina_ni_gphy_patch_word {
	u16	addr;	/* SRAM word address (via OCP 0xa436) */
	u16	data;	/* 16-bit word        (via OCP 0xa438) */
};

/*
 * apro_gen2 internal-GPHY uC firmware body, LIVE-DUMPED from RTL9607F stock
 * (per bank, uC held, via OCP 0xa436=addr / read 0xa438=data).  This is the
 * closed apro_gen2 image (not in open source; see cortina-ni-regs.h) and is
 * applied verbatim, in ascending-address order, to all four internal-PHY banks.
 * The known analog tweaks (0x8011=0xff77, 0x809b=0x7c70, 0x80ad=0x0c23) are
 * already baked into these words - do NOT layer any separate RMW on top.
 * 560 words, SRAM 0x8000..0x83f1.
 */
static const struct cortina_ni_gphy_patch_word cortina_ni_gphy_patch_words[] = {
	{ 0x8000, 0x0430 }, { 0x8001, 0x3051 }, { 0x8002, 0x5144 }, { 0x8003, 0x4451 },
	{ 0x8004, 0x5144 }, { 0x8005, 0x4404 }, { 0x8006, 0x0400 }, { 0x8007, 0x0051 },
	{ 0x8008, 0x5144 }, { 0x8009, 0x4451 }, { 0x800a, 0x5144 }, { 0x800b, 0x4402 },
	{ 0x800c, 0x02e8 }, { 0x800d, 0xe851 }, { 0x800e, 0x5144 }, { 0x800f, 0x44ff },
	{ 0x8010, 0xffff }, { 0x8011, 0xff77 }, { 0x8012, 0x77ff }, { 0x8013, 0xff00 },
	{ 0x8014, 0x0078 }, { 0x8015, 0x7805 }, { 0x8016, 0x0500 }, { 0x8029, 0x0001 },
	{ 0x802a, 0x0100 }, { 0x803e, 0x0080 }, { 0x803f, 0x8048 }, { 0x8040, 0x4806 },
	{ 0x8041, 0x061f }, { 0x8042, 0x1f08 }, { 0x8043, 0x081f }, { 0x8044, 0x1f32 },
	{ 0x8045, 0x321e }, { 0x8046, 0x1e6e }, { 0x8047, 0x6e24 }, { 0x8048, 0x2423 },
	{ 0x8049, 0x23e7 }, { 0x804a, 0xe730 }, { 0x804b, 0x3000 }, { 0x804d, 0x0024 },
	{ 0x804e, 0x2423 }, { 0x804f, 0x2327 }, { 0x8050, 0x2731 }, { 0x8051, 0x3100 },
	{ 0x8053, 0x0024 }, { 0x8054, 0x2423 }, { 0x8055, 0x23e7 }, { 0x8056, 0xe730 },
	{ 0x8057, 0x3000 }, { 0x8059, 0x0024 }, { 0x805a, 0x2423 }, { 0x805b, 0x23e7 },
	{ 0x805c, 0xe730 }, { 0x805d, 0x3000 }, { 0x805f, 0x0019 }, { 0x8060, 0x1923 },
	{ 0x8061, 0x23e7 }, { 0x8062, 0xe730 }, { 0x8063, 0x3000 }, { 0x8065, 0x0019 },
	{ 0x8066, 0x1923 }, { 0x8067, 0x23e7 }, { 0x8068, 0xe730 }, { 0x8069, 0x3000 },
	{ 0x806b, 0x0019 }, { 0x806c, 0x1923 }, { 0x806d, 0x23e7 }, { 0x806e, 0xe730 },
	{ 0x806f, 0x3000 }, { 0x8072, 0x0049 }, { 0x8073, 0x4900 }, { 0x8075, 0x007e },
	{ 0x8076, 0x7e01 }, { 0x8077, 0x0100 }, { 0x8078, 0x007e }, { 0x8079, 0x7e00 },
	{ 0x807b, 0x007f }, { 0x807c, 0x7f00 }, { 0x807e, 0x009e }, { 0x807f, 0x9e00 },
	{ 0x8083, 0x0080 }, { 0x8084, 0x808b }, { 0x8085, 0x8b16 }, { 0x8086, 0x1610 },
	{ 0x8087, 0x1049 }, { 0x8088, 0x4910 }, { 0x8089, 0x1097 }, { 0x808a, 0x9743 },
	{ 0x808b, 0x4300 }, { 0x808c, 0x00d4 }, { 0x808d, 0xd4c4 }, { 0x808e, 0xc409 },
	{ 0x808f, 0x09ea }, { 0x8090, 0xea07 }, { 0x8091, 0x0709 }, { 0x8092, 0x09ae },
	{ 0x8093, 0xaeca }, { 0x8094, 0xca1f }, { 0x8095, 0x1ffd }, { 0x8096, 0xfd1e },
	{ 0x8097, 0x1e0a }, { 0x8098, 0x0add }, { 0x8099, 0xdd0e }, { 0x809a, 0x0e7c },
	{ 0x809b, 0x7c70 }, { 0x809c, 0x7073 }, { 0x809d, 0x7340 }, { 0x809e, 0x4000 },
	{ 0x80a0, 0x002e }, { 0x80a1, 0x2efb }, { 0x80a2, 0xfb73 }, { 0x80a3, 0x7308 },
	{ 0x80a4, 0x0808 }, { 0x80a5, 0x08fc }, { 0x80a6, 0xfc05 }, { 0x80a7, 0x05f0 },
	{ 0x80a8, 0xf087 }, { 0x80a9, 0x8737 }, { 0x80aa, 0x3719 }, { 0x80ab, 0x19af },
	{ 0x80ac, 0xaf0c }, { 0x80ad, 0x0c23 }, { 0x80ae, 0x23dd }, { 0x80af, 0xdd1e },
	{ 0x80b0, 0x1e78 }, { 0x80b1, 0x7870 }, { 0x80b2, 0x7073 }, { 0x80b3, 0x7340 },
	{ 0x80b4, 0x4000 }, { 0x80b6, 0x002e }, { 0x80b7, 0x2e06 }, { 0x80b8, 0x067c },
	{ 0x80b9, 0x7c68 }, { 0x80ba, 0x6806 }, { 0x80bb, 0x0688 }, { 0x80bc, 0x8805 },
	{ 0x80bd, 0x05f1 }, { 0x80be, 0xf17a }, { 0x80bf, 0x7a31 }, { 0x80c0, 0x3119 },
	{ 0x80c1, 0x19c1 }, { 0x80c2, 0xc10c }, { 0x80c3, 0x0c23 }, { 0x80c4, 0x23dd },
	{ 0x80c5, 0xdd0e }, { 0x80c6, 0x0e78 }, { 0x80c7, 0x7870 }, { 0x80c8, 0x7073 },
	{ 0x80c9, 0x7340 }, { 0x80ca, 0x4000 }, { 0x80d4, 0x000a }, { 0x80d5, 0x0a04 },
	{ 0x80d6, 0x0400 }, { 0x80ea, 0x0020 }, { 0x80eb, 0x2001 }, { 0x80ec, 0x0120 },
	{ 0x80ed, 0x2001 }, { 0x80ee, 0x0120 }, { 0x80ef, 0x2001 }, { 0x80f0, 0x0120 },
	{ 0x80f1, 0x2001 }, { 0x80f2, 0x0120 }, { 0x80f3, 0x2001 }, { 0x80f4, 0x0120 },
	{ 0x80f5, 0x2001 }, { 0x80f6, 0x0120 }, { 0x80f7, 0x2001 }, { 0x80f8, 0x0120 },
	{ 0x80f9, 0x2001 }, { 0x80fa, 0x0120 }, { 0x80fb, 0x2001 }, { 0x80fc, 0x0120 },
	{ 0x80fd, 0x2001 }, { 0x80fe, 0x0157 }, { 0x80ff, 0x5724 }, { 0x8100, 0x2400 },
	{ 0x8101, 0x0011 }, { 0x8102, 0x1111 }, { 0x8103, 0x1111 }, { 0x8104, 0x1122 },
	{ 0x8105, 0x2222 }, { 0x8106, 0x2222 }, { 0x8107, 0x2222 }, { 0x8108, 0x2243 },
	{ 0x8109, 0x4365 }, { 0x810a, 0x6577 }, { 0x810b, 0x7707 }, { 0x810c, 0x0700 },
	{ 0x8118, 0x0006 }, { 0x8119, 0x0676 }, { 0x811a, 0x7600 }, { 0x811b, 0x000a },
	{ 0x811c, 0x0a00 }, { 0x811d, 0x0064 }, { 0x811e, 0x6403 }, { 0x811f, 0x03e8 },
	{ 0x8120, 0xe804 }, { 0x8121, 0x0401 }, { 0x8122, 0x0144 }, { 0x8123, 0x440b },
	{ 0x8124, 0x0be0 }, { 0x8125, 0xe007 }, { 0x8126, 0x0700 }, { 0x8127, 0x00d6 },
	{ 0x8128, 0xd6dc }, { 0x8129, 0xdce4 }, { 0x812a, 0xe4ea }, { 0x812b, 0xeaf0 },
	{ 0x812c, 0xf0f6 }, { 0x812d, 0xf6fc }, { 0x812e, 0xfc04 }, { 0x812f, 0x040a },
	{ 0x8130, 0x0a10 }, { 0x8131, 0x1016 }, { 0x8132, 0x161c }, { 0x8133, 0x1c24 },
	{ 0x8134, 0x242a }, { 0x8135, 0x2a30 }, { 0x8136, 0x3036 }, { 0x8137, 0x363c },
	{ 0x8138, 0x3c00 }, { 0x813c, 0x0008 }, { 0x813d, 0x0803 }, { 0x813e, 0x0300 },
	{ 0x8142, 0x0001 }, { 0x8143, 0x012c }, { 0x8144, 0x2c00 }, { 0x8149, 0x009f },
	{ 0x814a, 0x9f00 }, { 0x814b, 0x009f }, { 0x814c, 0x9f00 }, { 0x814d, 0x002e },
	{ 0x814e, 0x2e00 }, { 0x814f, 0x00ae }, { 0x8150, 0xae02 }, { 0x8151, 0x0216 },
	{ 0x8152, 0x1604 }, { 0x8153, 0x0404 }, { 0x8154, 0x0400 }, { 0x8155, 0x0055 },
	{ 0x8156, 0x5500 }, { 0x8158, 0x0055 }, { 0x8159, 0x5500 }, { 0x815b, 0x0055 },
	{ 0x815c, 0x5500 }, { 0x815e, 0x0055 }, { 0x815f, 0x5500 }, { 0x8166, 0x0004 },
	{ 0x8167, 0x0410 }, { 0x8168, 0x1000 }, { 0x8169, 0x0004 }, { 0x816a, 0x0403 },
	{ 0x816b, 0x030f }, { 0x816c, 0x0f10 }, { 0x816d, 0x1001 }, { 0x816e, 0x0100 },
	{ 0x816f, 0x0043 }, { 0x8170, 0x4384 }, { 0x8171, 0x84ed }, { 0x8172, 0xed6a },
	{ 0x8173, 0x6a01 }, { 0x8174, 0x0100 }, { 0x8175, 0x00f8 }, { 0x8176, 0xf8ce },
	{ 0x8177, 0xce00 }, { 0x8178, 0x0090 }, { 0x8179, 0x9030 }, { 0x817a, 0x3070 },
	{ 0x817b, 0x7010 }, { 0x817c, 0x1000 }, { 0x817d, 0x0018 }, { 0x817e, 0x1806 },
	{ 0x817f, 0x06f4 }, { 0x8180, 0xf408 }, { 0x8181, 0x08ef }, { 0x8182, 0xef08 },
	{ 0x8183, 0x08f3 }, { 0x8184, 0xf30c }, { 0x8185, 0x0ca6 }, { 0x8186, 0xa60d },
	{ 0x8187, 0x0dc8 }, { 0x8188, 0xc806 }, { 0x8189, 0x0410 }, { 0x818a, 0x1008 },
	{ 0x818b, 0x0800 }, { 0x818c, 0x00ff }, { 0x818d, 0xffff }, { 0x818e, 0xff00 },
	{ 0x818f, 0x00ff }, { 0x8190, 0xffff }, { 0x8191, 0xff00 }, { 0x8192, 0x00ff },
	{ 0x8193, 0xffff }, { 0x8194, 0xff00 }, { 0x8195, 0x00ff }, { 0x8196, 0xffff },
	{ 0x8197, 0xff00 }, { 0x8198, 0x00ff }, { 0x8199, 0xffff }, { 0x819a, 0xff00 },
	{ 0x819b, 0x00ff }, { 0x819c, 0xffff }, { 0x819d, 0xff00 }, { 0x819e, 0x00ff },
	{ 0x819f, 0xffff }, { 0x81a0, 0xff00 }, { 0x81a1, 0x00ff }, { 0x81a2, 0xffff },
	{ 0x81a3, 0xff00 }, { 0x81a4, 0x00ff }, { 0x81a5, 0xffff }, { 0x81a6, 0xff00 },
	{ 0x81a7, 0x00ff }, { 0x81a8, 0xffff }, { 0x81a9, 0xff00 }, { 0x81aa, 0x00ff },
	{ 0x81ab, 0xffff }, { 0x81ac, 0xff00 }, { 0x81ad, 0x00ff }, { 0x81ae, 0xffff },
	{ 0x81af, 0xff00 }, { 0x81b0, 0x00ff }, { 0x81b1, 0xffff }, { 0x81b2, 0xff00 },
	{ 0x81b3, 0x00ff }, { 0x81b4, 0xffff }, { 0x81b5, 0xff00 }, { 0x81b6, 0x00ff },
	{ 0x81b7, 0xffff }, { 0x81b8, 0xff00 }, { 0x81b9, 0x00ff }, { 0x81ba, 0xffff },
	{ 0x81bb, 0xff00 }, { 0x81be, 0x0082 }, { 0x81bf, 0x8249 }, { 0x81c0, 0x4982 },
	{ 0x81c1, 0x8249 }, { 0x81c2, 0x4909 }, { 0x81c3, 0x0906 }, { 0x81c4, 0x0609 },
	{ 0x81c5, 0x0906 }, { 0x81c6, 0x0601 }, { 0x81c7, 0x01f0 }, { 0x81c8, 0xf003 },
	{ 0x81c9, 0x03e4 }, { 0x81ca, 0xe401 }, { 0x81cb, 0x0100 }, { 0x81cc, 0x0004 },
	{ 0x81cd, 0x041a }, { 0x81ce, 0x1a43 }, { 0x81cf, 0x4303 }, { 0x81d0, 0x0344 },
	{ 0x81d1, 0x4440 }, { 0x81d2, 0x4009 }, { 0x81d3, 0x09cb }, { 0x81d4, 0xcb00 },
	{ 0x81d5, 0x00f0 }, { 0x81d6, 0xf0f6 }, { 0x81d7, 0xf632 }, { 0x81d8, 0x3201 },
	{ 0x81d9, 0x0117 }, { 0x81da, 0x1704 }, { 0x81db, 0x0410 }, { 0x81dc, 0x10bc },
	{ 0x81dd, 0xbc02 }, { 0x81de, 0x023f }, { 0x81df, 0x3f11 }, { 0x81e0, 0x1105 },
	{ 0x81e1, 0x05d3 }, { 0x81e2, 0xd300 }, { 0x81e3, 0x00f4 }, { 0x81e4, 0xf43a },
	{ 0x81e5, 0x3a01 }, { 0x81e6, 0x0106 }, { 0x81e7, 0x0604 }, { 0x81e8, 0x0410 },
	{ 0x81e9, 0x10cb }, { 0x81ea, 0xcb02 }, { 0x81eb, 0x023c }, { 0x81ec, 0x3c3d },
	{ 0x81ed, 0x3d05 }, { 0x81ee, 0x05df }, { 0x81ef, 0xdf00 }, { 0x81f0, 0x0008 },
	{ 0x81f1, 0x08f0 }, { 0x81f2, 0xf0f6 }, { 0x81f3, 0xf632 }, { 0x81f4, 0x3201 },
	{ 0x81f5, 0x0117 }, { 0x81f6, 0x1704 }, { 0x81f7, 0x0410 }, { 0x81f8, 0x10bc },
	{ 0x81f9, 0xbc02 }, { 0x81fa, 0x023f }, { 0x81fb, 0x3f11 }, { 0x81fc, 0x1105 },
	{ 0x81fd, 0x05d3 }, { 0x81fe, 0xd300 }, { 0x81ff, 0x00f4 }, { 0x8200, 0xf43a },
	{ 0x8201, 0x3a01 }, { 0x8202, 0x0106 }, { 0x8203, 0x0604 }, { 0x8204, 0x0410 },
	{ 0x8205, 0x10cb }, { 0x8206, 0xcb02 }, { 0x8207, 0x023c }, { 0x8208, 0x3c3d },
	{ 0x8209, 0x3d05 }, { 0x820a, 0x05df }, { 0x820b, 0xdf00 }, { 0x820d, 0x00f0 },
	{ 0x820e, 0xf0f6 }, { 0x820f, 0xf632 }, { 0x8210, 0x3201 }, { 0x8211, 0x0117 },
	{ 0x8212, 0x1704 }, { 0x8213, 0x0410 }, { 0x8214, 0x10bc }, { 0x8215, 0xbc02 },
	{ 0x8216, 0x023f }, { 0x8217, 0x3f11 }, { 0x8218, 0x1105 }, { 0x8219, 0x05d3 },
	{ 0x821a, 0xd300 }, { 0x821b, 0x00f4 }, { 0x821c, 0xf43a }, { 0x821d, 0x3a01 },
	{ 0x821e, 0x0106 }, { 0x821f, 0x0604 }, { 0x8220, 0x0410 }, { 0x8221, 0x10cb },
	{ 0x8222, 0xcb02 }, { 0x8223, 0x023c }, { 0x8224, 0x3c3d }, { 0x8225, 0x3d05 },
	{ 0x8226, 0x05df }, { 0x8227, 0xdf00 }, { 0x8229, 0x00f0 }, { 0x822a, 0xf002 },
	{ 0x822b, 0x021e }, { 0x822c, 0x1e0a }, { 0x822d, 0x0a01 }, { 0x822e, 0x0100 },
	{ 0x8242, 0x0004 }, { 0x8243, 0x0400 }, { 0x8244, 0x000a }, { 0x8245, 0x0a07 },
	{ 0x8246, 0x0727 }, { 0x8247, 0x27fe }, { 0x8248, 0xfe26 }, { 0x8249, 0x2688 },
	{ 0x824a, 0x8888 }, { 0x824b, 0x8888 }, { 0x824c, 0x8888 }, { 0x824d, 0x8800 },
	{ 0x824e, 0x0002 }, { 0x824f, 0x0201 }, { 0x8250, 0x0100 }, { 0x827a, 0x0001 },
	{ 0x827b, 0x0100 }, { 0x832b, 0x0008 }, { 0x832c, 0x0808 }, { 0x832d, 0x0808 },
	{ 0x832e, 0x0808 }, { 0x832f, 0x0806 }, { 0x8330, 0x0606 }, { 0x8331, 0x0606 },
	{ 0x8332, 0x0606 }, { 0x8333, 0x0600 }, { 0x8334, 0x0002 }, { 0x8335, 0x0201 },
	{ 0x8336, 0x0110 }, { 0x8337, 0x1010 }, { 0x8338, 0x1010 }, { 0x8339, 0x1010 },
	{ 0x833a, 0x1000 }, { 0x833e, 0x0010 }, { 0x833f, 0x1000 }, { 0x8358, 0x0088 },
	{ 0x8359, 0x8888 }, { 0x835a, 0x8800 }, { 0x836f, 0x0099 }, { 0x8370, 0x9938 },
	{ 0x8371, 0x3806 }, { 0x8372, 0x0638 }, { 0x8373, 0x3800 }, { 0x8375, 0x0003 },
	{ 0x8376, 0x0304 }, { 0x8377, 0x0405 }, { 0x8378, 0x0506 }, { 0x8379, 0x06f4 },
	{ 0x837a, 0xf44d }, { 0x837b, 0x4d06 }, { 0x837c, 0x0600 }, { 0x837e, 0x0001 },
	{ 0x837f, 0x0100 }, { 0x8380, 0x0080 }, { 0x8381, 0x8000 }, { 0x83a7, 0x0007 },
	{ 0x83a8, 0x0700 }, { 0x83af, 0x0001 }, { 0x83b0, 0x0106 }, { 0x83b1, 0x0600 },
	{ 0x83b6, 0x00c8 }, { 0x83b7, 0xc800 }, { 0x83b8, 0x00c8 }, { 0x83b9, 0xc842 },
	{ 0x83ba, 0x42fd }, { 0x83bc, 0x7800 }, { 0x83bd, 0x0050 }, { 0x83be, 0x5080 },
	{ 0x83bf, 0x8000 }, { 0x83c2, 0x4a29 }, { 0x83c3, 0x004a }, { 0x83c4, 0x4aa9 },
	{ 0x83c5, 0xa94e }, { 0x83c6, 0x4eb3 }, { 0x83c7, 0xfd00 }, { 0x83c8, 0x3fde },
	{ 0x83c9, 0xc84a }, { 0x83ca, 0x4ac4 }, { 0x83cb, 0xc444 }, { 0x83cc, 0x44f8 },
	{ 0x83cd, 0xf84a }, { 0x83ce, 0x4aa9 }, { 0x83cf, 0xa900 }, { 0x83d0, 0x00c8 },
	{ 0x83d1, 0xc84e }, { 0x83d2, 0x4ebc }, { 0x83d3, 0xbc4a }, { 0x83d4, 0x4ad6 },
	{ 0x83d5, 0xd64a }, { 0x83d6, 0x4a93 }, { 0x83d7, 0x9300 }, { 0x83d8, 0x0045 },
	{ 0x83d9, 0x454b }, { 0x83da, 0x4b75 }, { 0x83db, 0x751f }, { 0x83dc, 0x1f41 },
	{ 0x83dd, 0x4182 }, { 0x83de, 0x8249 }, { 0x83df, 0x4907 }, { 0x83e0, 0x077a },
	{ 0x83e1, 0x7a07 }, { 0x83e2, 0x0771 }, { 0x83e3, 0x7107 }, { 0x83e4, 0x077a },
	{ 0x83e5, 0x7a05 }, { 0x83e6, 0x0526 }, { 0x83e7, 0x261e }, { 0x83e8, 0x1ee1 },
	{ 0x83e9, 0xe100 }, { 0x83ea, 0x0029 }, { 0x83eb, 0x2900 }, { 0x83ed, 0x0007 },
	{ 0x83ee, 0x077a }, { 0x83ef, 0x7a4b }, { 0x83f0, 0x4b69 }, { 0x83f1, 0x6900 },
};

static u16 cortina_ni_gphy_ocp_read(void __iomem *bank, u16 ocp)
{
	return readl(bank + CA_NI_GPHY_OCP(ocp)) & 0xffff;
}

static void cortina_ni_gphy_ocp_write(void __iomem *bank, u16 ocp, u16 val)
{
	writel(val, bank + CA_NI_GPHY_OCP(ocp));
}

/* write one firmware word: address register, then data register */
static void cortina_ni_gphy_sram_write(void __iomem *bank, u16 addr, u16 data)
{
	cortina_ni_gphy_ocp_write(bank, CA_NI_GPHY_SRAM_ADDR, addr);
	cortina_ni_gphy_ocp_write(bank, CA_NI_GPHY_SRAM_DATA, data);
}

/*
 * Load the internal-GPHY SRAM firmware image + resume the uC, per bank.
 *
 * TIMING: the uC SRAM is only writable while the uC is HELD.  The uC only
 * becomes lockable AFTER the MDIO/GPHY init - i.e. around LINK-UP, NOT at probe
 * (where SRAM writes are ignored).  So this runs from the link-up hook
 * (cortina_ni_rx_link_up), not probe.  For each bank we take the gate/lock/
 * ready-poll handshake (cortina-ni-regs.h), stream the firmware image, then
 * clear the hold bit so the uC runs the freshly-written image (its default ROM
 * firmware does NOT forward - the SRAM image is what enables line<->system
 * forwarding).
 *
 * One-shot per bank per boot (ni->gphy_patched[]); a bank whose gate reads
 * EXT_INI (not lockable yet) is left for the next link-up.  If the ready-poll
 * times out we log it and write the words anyway (the hold bit still took).
 */
static bool gphy_patch = true;
module_param(gphy_patch, bool, 0444);
MODULE_PARM_DESC(gphy_patch,
		 "load the internal-GPHY SRAM firmware image before resuming the uC (default on)");

void cortina_ni_gphy_patch_and_resume(struct cortina_ni *ni)
{
	void __iomem *win = ni->win[CA_NI_WIN_GPHY];
	unsigned int b, i, t;

	if (!win)
		return;		/* GPHY window not mapped */

	for (b = 0; b < CA_NI_GPHY_COUNT; b++) {
		void __iomem *bank = win + CA_NI_GPHY_BANK(b);
		u16 gate, ready = 0;
		bool ok = false;

		if (ni->gphy_patched[b])
			continue;	/* one-shot per bank per boot */

		/* gate: EXT_INI = uC not lockable yet, retry next link-up */
		gate = readl(bank + CA_NI_GPHY_LOCK_GATE) & 0xffff;
		if ((gate & CA_NI_GPHY_LOCK_GATE_MASK) ==
		    CA_NI_GPHY_LOCK_GATE_EXT_INI)
			continue;

		/* hold the uC (SRAM only writable while held) */
		writel(readl(bank + CA_NI_GPHY_LOCK) | CA_NI_GPHY_LOCK_HOLD,
		       bank + CA_NI_GPHY_LOCK);

		/* wait for pcs-locked, bounded; on timeout patch anyway */
		for (t = 0; t < CA_NI_GPHY_LOCK_READY_TRIES; t++) {
			ready = readl(bank + CA_NI_GPHY_LOCK_READY) & 0xffff;
			if ((ready & CA_NI_GPHY_LOCK_READY_MASK) ==
			    CA_NI_GPHY_LOCK_READY_VAL) {
				ok = true;
				break;
			}
			udelay(100);
		}

		/* stream the live-dumped firmware, one word at a time, in order */
		if (gphy_patch)
			for (i = 0;
			     i < ARRAY_SIZE(cortina_ni_gphy_patch_words); i++)
				cortina_ni_gphy_sram_write(bank,
					cortina_ni_gphy_patch_words[i].addr,
					cortina_ni_gphy_patch_words[i].data);

		/* resume: clear the hold bit so the uC runs the fresh firmware */
		writel(readl(bank + CA_NI_GPHY_LOCK) & ~CA_NI_GPHY_LOCK_HOLD,
		       bank + CA_NI_GPHY_LOCK);

		ni->gphy_patched[b] = true;

		dev_info(ni->dev,
			 "gphy patch bank %u: gate=0x%04x ready=%s wrote %u words\n",
			 b, gate, ok ? "ok" : "timeout",
			 gphy_patch ?
			 (unsigned int)ARRAY_SIZE(cortina_ni_gphy_patch_words)
			 : 0);
	}
}

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
	rst = ni->win[CA_NI_WIN_GLB] + CA_NI_GLB_BLOCK_RST;

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
		u32 was = readl(glb + CA_NI_GLB_DPHY_RESET);
		int j;

		writel(CA_NI_GLB_DPHY_RESET_VAL, glb + CA_NI_GLB_DPHY_RESET);
		for (j = 0; j < ARRAY_SIZE(order); j++) {
			u32 v = readl(glb + CA_NI_GLB_DPHY_RESET);

			writel(v | BIT(order[j]), glb + CA_NI_GLB_DPHY_RESET);	/* ASSERT one */
			mdelay(1);
			writel(v & ~BIT(order[j]), glb + CA_NI_GLB_DPHY_RESET);	/* DEASSERT it */
		}
		mdelay(100);					/* final settle (stock) */
		dev_info(dev, "ne-reset: sequential ni,l2fe,l2tm,l3fe,tqm at probe start (0x%08x -> 0x%08x)\n",
			 was, readl(glb + CA_NI_GLB_DPHY_RESET));
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

	ret = cortina_ni_mdio_init(ni);
	if (ret)
		return ret;

	cortina_ni_scan_phys(ni);

	ret = cortina_ni_tx_probe(ni);
	if (ret)
		return ret;

	ret = cortina_ni_rx_probe(ni);
	if (ret)
		return ret;

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

	dev_info(dev, "M2c probe complete\n");
	return 0;
}

static const struct of_device_id cortina_ni_of_match[] = {
	{ .compatible = "cortina,ni-interface" },
	{ }
};
MODULE_DEVICE_TABLE(of, cortina_ni_of_match);

static struct platform_driver cortina_ni_driver = {
	.probe = cortina_ni_probe,
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

static int __init cortina_ni_module_init(void)
{
	int ret;

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
module_init(cortina_ni_module_init);

static void __exit cortina_ni_module_exit(void)
{
	platform_driver_unregister(&cortina_ni_driver);
	phy_drivers_unregister(cortina_ni_gphy_driver,
			       ARRAY_SIZE(cortina_ni_gphy_driver));
}
module_exit(cortina_ni_module_exit);

MODULE_DESCRIPTION("Cortina-Access NI Ethernet driver (RTL9607F)");
MODULE_LICENSE("GPL");
