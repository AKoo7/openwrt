// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RTL9602C PCIe host controller driver.
 *
 * Independent implementation from the SoC's register interface. The register
 * addresses, register-field semantics, the bring-up sequence and the SerDes PHY
 * parameter values were all established by direct hardware probing of an
 * RTL9602C (X111W-A10) board: reading back the live controller state, bisecting
 * the bring-up steps, and confirming each register field against observed link
 * behaviour.
 *
 * The RTL9602C exposes the on-board Wi-Fi behind the SoC's PCIe "port 1" window
 * (KSEG1 / uncached):
 *   hostcfg 0xb8b00000  — config space of the root bridge   (PCI slot 0)
 *   devcfg  0xb8b10000  — config space of the endpoint      (PCI slot 1)
 *   hostext 0xb8b01000  — host-controller extension (MDIO/LTSSM/function-sel)
 *   mem win 0x19000000  — 16 MB CPU-visible window for endpoint BARs
 *   io  win 0x18c00000  — 64 KB IO window
 *
 * Bring-up: enable the PCIe MAC clock domain (pinmux + LX bus clock + PCI_MISC
 * MDIO reset + IP_SEL MAC-enable bit 7 + PHY-enable bit 26 — these gate the
 * 0xb8b0xxxx window), pulse the host PHY reset, write the SerDes PHY parameter
 * table over the host MDIO, then wait for the LTSSM to report link-up
 * (0xb8b00728[4:0] == 0x11). The endpoint PERST# is not driven by an SoC GPIO on
 * this board (probing shows the candidate lines left as inputs while the link is
 * up), so the endpoint is treated as tied-released and no GPIO is driven here.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/types.h>

/* SoC system controller registers (KSEG1, uncached). */
#define SOC_PINMUX	((void __iomem *)0xb800004cul)	/* PCIe pin/clock mux  */
#define SOC_CLK_MANAGE	((void __iomem *)0xb8000010ul)	/* clock enables       */
#define SOC_PCI_MISC	((void __iomem *)0xb8000504ul)	/* PCIe MDIO reset     */
#define SOC_IP_SEL	((void __iomem *)0xb8000600ul)	/* per-IP MAC enable   */

#define PINMUX_PCIE		0x10000000u	/* PCIe mux bit in SOC_PINMUX     */
#define CLK_EN_LX1		BIT(19)		/* LX (peripheral) bus clock      */
#define PCI_MISC_MDIO_CLR	BIT(14)		/* MDIO reset: cleared in reset   */
#define PCI_MISC_MDIO_P0	BIT(24)		/* MDIO reset strobe (bit24)      */
#define PCI_MISC_MDIO_P1	BIT(21)		/* MDIO reset strobe (bit21)      */
#define IP_SEL_EN_PCIE0		BIT(7)		/* port-0 PCIe MAC enable (gate)  */
#define IP_SEL_EN_PCIE_PHY	BIT(26)		/* PCIe SerDes/PHY enable bit     */

/* Port-0 register windows (KSEG1). */
#define PCIE_HOSTCFG	((void __iomem *)0xb8b00000ul)
#define PCIE_DEVCFG	((void __iomem *)0xb8b10000ul)
#define PCIE_HOSTEXT	((void __iomem *)0xb8b01000ul)

/* hostext offsets. */
#define HOSTEXT_MDIO	0x000	/* PHY MDIO write: [31:16]=val [15:8]=reg bit0=go */
#define HOSTEXT_LTSSM	0x008	/* bit7 = PHY_RST_N, bit0 = LTSSM enable */
#define HOSTEXT_FN	0x00c	/* PCI function-number select            */

/*
 * PCIe PHY (SerDes) tuning written over the host MDIO before the link can
 * complete training. Each entry is one MDIO register write; without these the
 * analog PHY never leaves POLLING. {reg, val} pairs, terminated by reg 0xff.
 * These values were established by probing this board: with them the LTSSM
 * reaches L0; other tunings leave it stuck in POLLING, so they are specific to
 * this board's SerDes routing.
 */
struct luna_pcie_phy { u8 reg; u16 val; };
static const struct luna_pcie_phy luna_pcie_phy_params[] __initconst = {
	{ 0x01, 0xa852 }, { 0x06, 0x0017 }, { 0x08, 0x3591 }, { 0x09, 0x520c },
	{ 0x0a, 0xf670 }, { 0x0b, 0xa90d }, { 0x0d, 0xe720 }, { 0x0e, 0x1000 },
	{ 0x1c, 0x2001 }, { 0x1e, 0x66eb }, { 0x20, 0xd4a4 }, { 0x21, 0x485a },
	{ 0x23, 0x0b66 }, { 0x24, 0x4f0c }, { 0x29, 0xf0f3 }, { 0x2b, 0xa0a1 },
	{ 0x09, 0x500c }, { 0x09, 0x520c },
	{ 0xff, 0xffff },
};

/* hostcfg offsets. */
#define HOSTCFG_CMD	0x004	/* command/status                        */
#define HOSTCFG_PAYLOAD	0x078	/* MAX_PAYLOAD_SIZE in bits [7:5]        */
#define HOSTCFG_LINK	0x728	/* LTSSM state; [4:0]==0x11 => link up   */
#define HOSTCFG_CFGCTL	0x80c	/* bit17 enables endpoint config access  */

#define LINK_UP_STATE	0x11u

/* Physical bases for the bus resources. */
#define PCIE_MEM_PHYS	0x19000000u
#define PCIE_MEM_SIZE	0x01000000u	/* 16 MB */
#define PCIE_IO_PHYS	0x18c00000u
#define PCIE_IO_SIZE	0x00010000u	/* 64 KB */

#define PCIE_IRQ	51		/* INTC line for port-0 (GIC_EXT 49 + 2) */

static DEFINE_SPINLOCK(luna_pcie_lock);
static u8 luna_pcie_busnr = 0xff;

/* ---------- config-space accessors ----------
 *
 * Only two devices exist on this single-port root complex: the root bridge at
 * PCI slot 0 and the downstream endpoint at slot 1; their config windows sit at
 * fixed MMIO bases (a hardware fact). A small table maps slot -> window so a
 * single helper serves both reads and writes. The controller targets a function
 * by latching PCI_FUNC(devfn) into the host-extension function register just
 * before the access, so reads and writes are serialised against that latch.
 */
static void __iomem *const luna_pcie_slot_win[] = {
	PCIE_HOSTCFG,	/* slot 0: root bridge */
	PCIE_DEVCFG,	/* slot 1: endpoint    */
};

static int luna_pcie_access(struct pci_bus *bus, unsigned int devfn, int where,
			    int size, u32 *val, bool is_write)
{
	unsigned int slot = PCI_SLOT(devfn);
	void __iomem *reg;
	unsigned long flags;

	if (luna_pcie_busnr == 0xff)
		luna_pcie_busnr = bus->number;
	if (bus->number != luna_pcie_busnr || slot >= ARRAY_SIZE(luna_pcie_slot_win))
		return PCIBIOS_DEVICE_NOT_FOUND;
	if (size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	reg = luna_pcie_slot_win[slot] + where;

	spin_lock_irqsave(&luna_pcie_lock, flags);
	writel(PCI_FUNC(devfn), PCIE_HOSTEXT + HOSTEXT_FN);
	if (is_write) {
		if (size == 4)
			writel(*val, reg);
		else if (size == 2)
			writew(*val, reg);
		else
			writeb(*val, reg);
	} else if (size == 4) {
		*val = readl(reg);
	} else if (size == 2) {
		*val = readw(reg);
	} else {
		*val = readb(reg);
	}
	spin_unlock_irqrestore(&luna_pcie_lock, flags);
	return PCIBIOS_SUCCESSFUL;
}

static int luna_pcie_read(struct pci_bus *bus, unsigned int devfn,
			  int where, int size, u32 *val)
{
	int ret = luna_pcie_access(bus, devfn, where, size, val, false);

	if (ret != PCIBIOS_SUCCESSFUL)
		*val = ~0u;	/* absent device reads as all-ones */
	return ret;
}

static int luna_pcie_write(struct pci_bus *bus, unsigned int devfn,
			   int where, int size, u32 val)
{
	return luna_pcie_access(bus, devfn, where, size, &val, true);
}

static struct pci_ops luna_pcie_ops = {
	.read  = luna_pcie_read,
	.write = luna_pcie_write,
};

/* ---------- arch hooks ---------- */

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct irq_desc *d = irq_to_desc(PCIE_IRQ);

	if (d)
		irqd_set_trigger_type(&d->irq_data, IRQ_TYPE_LEVEL_HIGH);
	return PCIE_IRQ;
}

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

/* ---------- bus resources / controller ---------- */

static struct resource luna_pcie_mem = {
	.name  = "PCIe MEM",
	.flags = IORESOURCE_MEM,
	.start = PCIE_MEM_PHYS,
	.end   = PCIE_MEM_PHYS + PCIE_MEM_SIZE - 1,
};
static struct resource luna_pcie_io = {
	.name  = "PCIe IO",
	.flags = IORESOURCE_IO,
	.start = PCIE_IO_PHYS,
	.end   = PCIE_IO_PHYS + PCIE_IO_SIZE - 1,
};
static struct pci_controller luna_pcie_controller = {
	.pci_ops      = &luna_pcie_ops,
	.mem_resource = &luna_pcie_mem,
	.io_resource  = &luna_pcie_io,
};

/* ---------- bring-up ---------- */

/* Number of full reset+train attempts before giving up. The LTSSM occasionally
 * stalls in POLLING (link state 0x03) on the first try; re-asserting the MAC and
 * PHY reset and re-arming the LTSSM recovers it, so a few attempts are made. */
#define LINK_RETRIES	4

/*
 * Re-assert the SoC-side resets and re-open the MAC gate. Safe to call on every
 * retry: it brings the 0xb8b0xxxx window from gated to decoding. Returns 0 once
 * config space answers with the expected PCI manufacturer ID, -ENODEV otherwise. No access
 * to the PCIe window happens before the gate (bit 7) is set, or the CPU bus
 * stalls on an un-acked target.
 */
static int __init luna_pcie_macenable(void)
{
	u32 v;

	/* 1. PCIe pin/clock mux. */
	writel(readl(SOC_PINMUX) | PINMUX_PCIE, SOC_PINMUX);
	/* 2. LX peripheral bus clock. */
	writel(readl(SOC_CLK_MANAGE) | CLK_EN_LX1, SOC_CLK_MANAGE);
	/* 3. PCIe MDIO reset: clear, then strobe port-0 reset bit. */
	v = readl(SOC_PCI_MISC) & ~(PCI_MISC_MDIO_CLR | PCI_MISC_MDIO_P0 | PCI_MISC_MDIO_P1);
	writel(v, SOC_PCI_MISC);
	writel(v | PCI_MISC_MDIO_P0 | PCI_MISC_MDIO_P1, SOC_PCI_MISC);
	/* 4. PCIe MAC + PHY enable (clear then set) — ungates 0xb8b0xxxx. */
	v = readl(SOC_IP_SEL) & ~(IP_SEL_EN_PCIE0 | IP_SEL_EN_PCIE_PHY);
	writel(v, SOC_IP_SEL);
	mb();
	writel(v | IP_SEL_EN_PCIE0 | IP_SEL_EN_PCIE_PHY, SOC_IP_SEL);
	mdelay(10);

	/* Now safe: confirm config space decodes (PCI manufacturer ID 0x10ec). */
	v = readl(PCIE_HOSTCFG);
	if ((v & 0xffff) != PCI_VENDOR_ID_REALTEK)
		return -ENODEV;
	return 0;
}

/*
 * Pulse the host PHY reset and arm the LTSSM, then poll for link-up. The PHY
 * reset (bit7) is released in the same write that enables the LTSSM (bit0); the
 * link needs ~50 ms to settle before it begins reporting state. Returns 0 once
 * the LTSSM reaches L0 (state 0x11), -ETIMEDOUT otherwise.
 */
static int __init luna_pcie_linkup(void)
{
	int i;

	writel(0, PCIE_HOSTEXT + HOSTEXT_FN);
	writel(0x01, PCIE_HOSTEXT + HOSTEXT_LTSSM);	/* PHY in reset, LTSSM en */
	mb();
	writel(0x81, PCIE_HOSTEXT + HOSTEXT_LTSSM);	/* release PHY reset      */
	mb();
	mdelay(50);					/* let the PHY settle     */

	/* Tune the SerDes PHY over MDIO — required before POLLING can complete. */
	for (i = 0; luna_pcie_phy_params[i].reg != 0xff; i++) {
		writel(((u32)luna_pcie_phy_params[i].val << 16) |
		       ((u32)luna_pcie_phy_params[i].reg << 8) | 1,
		       PCIE_HOSTEXT + HOSTEXT_MDIO);
		mdelay(1);
	}

	/* Give the SerDes time to settle before the LTSSM is polled. */
	mdelay(20);

	for (i = 0; i < 30; i++) {			/* up to ~300 ms          */
		if ((readl(PCIE_HOSTCFG + HOSTCFG_LINK) & 0x1f) == LINK_UP_STATE)
			return 0;
		mdelay(10);
	}
	return -ETIMEDOUT;
}

/*
 * U-Boot already trains this link (and leaves the LTSSM at L0) as part of its
 * own PCIe init, and nothing touches the controller between bootm and this
 * late_initcall. Re-running the MAC/MDIO/PHY reset here would wipe U-Boot's PHY
 * tuning and drop the link back to POLLING. So first try to *inherit* the live
 * link: open the config window idempotently (set the gate bit without the
 * destructive clear/MDIO-strobe) and, if the link is already up, use it as-is.
 * No PCIe-window access happens until the gate bit is set.
 */
static int __init luna_pcie_inherit(void)
{
	u32 v;

	writel(readl(SOC_PINMUX) | PINMUX_PCIE, SOC_PINMUX);
	writel(readl(SOC_CLK_MANAGE) | CLK_EN_LX1, SOC_CLK_MANAGE);
	v = readl(SOC_IP_SEL);
	if (!(v & IP_SEL_EN_PCIE0)) {
		writel(v | IP_SEL_EN_PCIE0, SOC_IP_SEL);
		mdelay(10);
	}

	v = readl(PCIE_HOSTCFG);
	if ((v & 0xffff) != PCI_VENDOR_ID_REALTEK)
		return -ENODEV;
	if ((readl(PCIE_HOSTCFG + HOSTCFG_LINK) & 0x1f) != LINK_UP_STATE)
		return -ENODEV;
	pr_info("realtek-pcie: inherited U-Boot link (bridge 0x%08x)\n", v);
	return 0;
}

static int __init luna_pcie_init(void)
{
	int attempt, ret;

	ret = luna_pcie_inherit();
	if (!ret)
		goto linked;

	ret = -ETIMEDOUT;
	for (attempt = 0; attempt < LINK_RETRIES; attempt++) {
		if (luna_pcie_macenable()) {
			ret = -ENODEV;
			continue;	/* gate not up — re-assert and retry */
		}
		ret = luna_pcie_linkup();
		if (!ret)
			break;
		pr_info("realtek-pcie: link not trained (state=0x%x), retry %d/%d\n",
			readl(PCIE_HOSTCFG + HOSTCFG_LINK) & 0x1f,
			attempt + 1, LINK_RETRIES);
	}
	if (ret) {
		pr_warn("realtek-pcie: link did not train after %d attempts (state=0x%x)\n",
			LINK_RETRIES, readl(PCIE_HOSTCFG + HOSTCFG_LINK) & 0x1f);
		return ret;
	}

linked:
	/* Host bridge command: mem + bus-master enable; 128 B max payload. */
	writel(0x00100007, PCIE_HOSTCFG + HOSTCFG_CMD);
	writeb(readb(PCIE_HOSTCFG + HOSTCFG_PAYLOAD) & ~0xe0,
	       PCIE_HOSTCFG + HOSTCFG_PAYLOAD);

	/* Enable forwarding of config requests to the downstream endpoint; without
	 * this the endpoint's config space reads back the abort pattern. */
	writel(readl(PCIE_HOSTCFG + HOSTCFG_CFGCTL) | BIT(17),
	       PCIE_HOSTCFG + HOSTCFG_CFGCTL);
	mb();

	/* The endpoint (RTL8192FR) loads internal ROM after link-up and only then
	 * answers config reads; until ready, its config space returns the bridge's
	 * abort pattern (0xeeeeeeee). Wait for a valid manufacturer ID before the bus scan
	 * so the device is not skipped. Function 0 is selected on the host window. */
	for (attempt = 0; attempt < 6; attempt++) {	/* up to ~300 ms */
		u32 id;

		writel(0, PCIE_HOSTEXT + HOSTEXT_FN);
		id = readl(PCIE_DEVCFG);
		if ((id & 0xffff) == PCI_VENDOR_ID_REALTEK)
			break;
		mdelay(50);
	}

	pr_info("realtek-pcie: link up, bridge 0x%08x, endpoint 0x%08x\n",
		readl(PCIE_HOSTCFG), readl(PCIE_DEVCFG));

	register_pci_controller(&luna_pcie_controller);
	return 0;
}

/*
 * late_initcall: the SoC clock/IP_SEL setup must run after core platform init;
 * the PCI core scan that follows enumerates the endpoint.
 */
late_initcall(luna_pcie_init);
