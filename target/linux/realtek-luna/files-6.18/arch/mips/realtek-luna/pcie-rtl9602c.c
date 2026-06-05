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
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>
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
/* Extra IP_SEL gates the operational config sets alongside the PCIe MAC/PHY
 * (IP_SEL = 0x04001887). Without these the downstream endpoint's config space
 * never decodes (reads the 0xeeeeeeee abort pattern) even though the link trains. */
#define IP_SEL_EN_EXTRA		(BIT(0) | BIT(2) | BIT(11) | BIT(12))

/* Port-0 register windows (KSEG1). */
#define PCIE_HOSTCFG	((void __iomem *)0xb8b00000ul)
#define PCIE_DEVCFG	((void __iomem *)0xb8b10000ul)
#define PCIE_HOSTEXT	((void __iomem *)0xb8b01000ul)

/* hostext offsets. */
#define HOSTEXT_MDIO	0x000	/* PHY MDIO write: [31:16]=val [15:8]=reg bit0=go */
#define HOSTEXT_LTSSM	0x008	/* bit7 = PHY_RST_N, bit0 = LTSSM enable */
#define HOSTEXT_FN	0x00c	/* PCI function-number select            */

/*
 * PCIe SerDes PHY tuning, written over the host MDIO before link training. These
 * are the revB analog values that match this RTL9602C (rev A) SerDes. The match
 * is load-bearing beyond just training: regs 0x20/0x21 set the SerDes PLL and
 * clock divider that also clock the downstream endpoint's config core. A sibling
 * part's revC table (0x20=0xd4a4/0x21=0x485a) still bit-locks the lane to L0, but
 * leaves the endpoint's config-core clock mistuned so every config TLP aborts
 * (reads back the 0xeeeeeeee pattern). {reg, val} pairs, terminated by reg 0xff.
 */
struct luna_pcie_phy { u8 reg; u16 val; };
static const struct luna_pcie_phy luna_pcie_phy_params[] __initconst = {
	{ 0x01, 0xa852 }, { 0x06, 0x0017 }, { 0x08, 0x3591 }, { 0x09, 0x520c },
	{ 0x0a, 0xf670 }, { 0x0b, 0xa90d }, { 0x0d, 0xe720 }, { 0x0e, 0x1000 },
	{ 0x1c, 0x2001 }, { 0x1e, 0x66eb }, { 0x20, 0xd4a4 }, { 0x21, 0x485a },
	{ 0x23, 0x0b66 }, { 0x24, 0x4f0c }, { 0x29, 0xf0f3 }, { 0x2b, 0xa0a1 },
	{ 0x09, 0x500c }, { 0x09, 0x520c },
	/* 25 MHz reference-clock SerDes values (the board has a 25 MHz crystal at
	 * the WiFi PCIe PHY). This is the vendor "9602C 25M clk" ePHY table; reg
	 * 0x03 is the refclk PLL multiplier (0x3031 for 25 MHz, vs 0x7b31 for
	 * 40 MHz) and reg 0x06 = 0xe0b8 (vs 0xe2b8). A 40 MHz PLL on a 25 MHz
	 * refclk mistunes the config-core clock -> marginal high-offset access. */
	{ 0x03, 0x3031 }, { 0x06, 0xe0b8 }, { 0x0e, 0x98c5 },
	{ 0x0f, 0x400f }, { 0x19, 0xfc70 },
	{ 0xff, 0xffff },
};

/* hostcfg offsets. */
#define HOSTCFG_CMD	0x004	/* command/status                        */
#define HOSTCFG_PAYLOAD	0x078	/* MAX_PAYLOAD_SIZE in bits [7:5]        */
#define HOSTCFG_LINK	0x728	/* LTSSM state; [4:0]==0x11 => link up   */
#define HOSTCFG_CFGCTL	0x80c	/* bit17 enables endpoint config access  */
#define CFGCTL_FWD_EN	BIT(17)	/* write-enable; reads back as 0x100 once active */

#define LINK_UP_STATE	0x11u

/* Physical bases for the bus resources. */
#define PCIE_MEM_PHYS	0x19000000u
#define PCIE_MEM_SIZE	0x01000000u	/* 16 MB */
#define PCIE_IO_PHYS	0x18c00000u
#define PCIE_IO_SIZE	0x00010000u	/* 64 KB */

/*
 * PCIe host-controller input on the SoC INTC aggregator. Established by probing
 * the live INTC GISR while the endpoint asserted INTx: the host-bridge interrupt
 * status (HOSTEXT+0x04) reads 0x1 exactly when INTC GISR0 bit 15 latches, so the
 * aggregated PCIe INTx lands on INTC input 15. (Input 51 is UART2 — the earlier
 * value never fired.) Input 15's IRR routing nibble is preset to 3 -> CP0 IRQ4,
 * which the INTC cascade picks up.
 */
#define PCIE_HWIRQ	15

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
	mb();			/* order the function-select latch before the access */
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
	static int pcie_virq;

	/* The endpoint's INTx is aggregated by the SoC INTC onto a single input
	 * line; map that hwirq through the INTC's (linear) irq_domain to obtain
	 * the Linux virq the PCI core hands to the endpoint driver. Returning the
	 * raw hwirq would yield an unmapped virq (no_irq_chip) so request_irq()
	 * fails with -ENOSYS. The INTC drives handle_level_irq, so the line is
	 * already level-triggered. */
	if (!pcie_virq) {
		struct device_node *np;

		np = of_find_compatible_node(NULL, NULL,
					     "realtek,rtl9602c-intc");
		if (np) {
			struct irq_domain *domain = irq_find_host(np);

			of_node_put(np);
			if (domain)
				pcie_virq = irq_create_mapping(domain,
							       PCIE_HWIRQ);
		}
	}
	return pcie_virq;
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

/* Number of full reset+train attempts before giving up — the LTSSM occasionally
 * stalls in POLLING on the first try and a fresh cold reset recovers it. */
#define LINK_RETRIES	3

/*
 * Full PCIe host bring-up, in the controller's documented reset order/timing:
 * MDIO reset, MAC-enable pulse, PHY reset + SerDes tuning, then link training.
 * Returns 0 once the LTSSM reaches L0 (state 0x11), -ETIMEDOUT otherwise. No
 * access to the 0xb8b0xxxx window happens before the MAC gate is set in step 2,
 * or the CPU bus stalls on an un-acked target.
 */
static int __init luna_pcie_reset(void)
{
	u32 v;
	int i;

	/* 0. Reset settle (the per-board device-reset strap is not software-driven
	 *    on this board, so this is the bare timing budget). */
	mdelay(10);

	/* 1. PCIe pin mux, then MDIO reset: clear the reset-hold bit and the port
	 *    reset bits, then strobe the port reset bits. This board trains only
	 *    with both port reset bits strobed. */
	writel(readl(SOC_PINMUX) | PINMUX_PCIE, SOC_PINMUX);
	v = readl(SOC_PCI_MISC) & ~(PCI_MISC_MDIO_CLR | PCI_MISC_MDIO_P0 | PCI_MISC_MDIO_P1);
	writel(v, SOC_PCI_MISC);
	mb();
	writel(v | PCI_MISC_MDIO_P0 | PCI_MISC_MDIO_P1, SOC_PCI_MISC);
	mdelay(1);

	/* 2. Ensure the PHY + operational gates are enabled, then pulse ONLY the MAC
	 *    enable bit (clear then set) as the MAC reset. The PHY-enable bit is left
	 *    set throughout — clearing it mid-bring-up resets the SerDes. */
	writel(readl(SOC_IP_SEL) | IP_SEL_EN_PCIE_PHY | IP_SEL_EN_EXTRA, SOC_IP_SEL);
	v = readl(SOC_IP_SEL) & ~IP_SEL_EN_PCIE0;
	writel(v, SOC_IP_SEL);
	mb();
	writel(v | IP_SEL_EN_PCIE0, SOC_IP_SEL);
	mdelay(100);

	/* 3. Arm the LTSSM with the PHY held in reset, then release the PHY reset. */
	writel(0, PCIE_HOSTEXT + HOSTEXT_FN);
	writel(0x01, PCIE_HOSTEXT + HOSTEXT_LTSSM);	/* PHY in reset, LTSSM en */
	mb();
	writel(0x81, PCIE_HOSTEXT + HOSTEXT_LTSSM);	/* release PHY reset      */
	mdelay(50);

	/* 4. SerDes PHY tuning over MDIO — required before POLLING can complete. */
	for (i = 0; luna_pcie_phy_params[i].reg != 0xff; i++) {
		writel(((u32)luna_pcie_phy_params[i].val << 16) |
		       ((u32)luna_pcie_phy_params[i].reg << 8) | 1,
		       PCIE_HOSTEXT + HOSTEXT_MDIO);
		mdelay(1);
	}
	mdelay(20);

	/* 5. Poll for link-up (L0). */
	for (i = 0; i < 10; i++) {
		if ((readl(PCIE_HOSTCFG + HOSTCFG_LINK) & 0x1f) == LINK_UP_STATE)
			return 0;
		mdelay(10);
	}
	return -ETIMEDOUT;
}

static int __init luna_pcie_init(void)
{
	int attempt, ret = -ETIMEDOUT;

	/* Train the link: the pcie1-revC SerDes table + bit7 MAC enable bring the
	 * lane to L0, retried a few times to clear an occasional POLLING stall. */
	for (attempt = 0; attempt < LINK_RETRIES; attempt++) {
		ret = luna_pcie_reset();
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

	/* Configuration-retry settle before any config/BAR access. */
	mdelay(100);

	/* Program the downstream endpoint's BARs + command register, then enable the
	 * host bridge (written twice), set 128 B max payload, and enable config
	 * forwarding. The forwarding-enable is the bit17 write-strobe — the register
	 * then reads back the operational value 0x100, but writing 0x100 does not
	 * enable it. */
	writel(0x18c00001, PCIE_DEVCFG + 0x10);
	writel(0x19000004, PCIE_DEVCFG + 0x18);
	writel(0x00180007, PCIE_DEVCFG + 0x04);
	writel(0x00100007, PCIE_HOSTCFG + HOSTCFG_CMD);
	writel(0x00100007, PCIE_HOSTCFG + HOSTCFG_CMD);
	writeb(readb(PCIE_HOSTCFG + HOSTCFG_PAYLOAD) & ~0xe0,
	       PCIE_HOSTCFG + HOSTCFG_PAYLOAD);
	writel(readl(PCIE_HOSTCFG + HOSTCFG_CFGCTL) | CFGCTL_FWD_EN,
	       PCIE_HOSTCFG + HOSTCFG_CFGCTL);
	mb();

	/* Wait for the endpoint's config space to answer before the bus scan so the
	 * device is not skipped. Function 0 is selected on the host window. */
	for (attempt = 0; attempt < 20; attempt++) {	/* up to ~1 s */
		u32 id;

		writel(0, PCIE_HOSTEXT + HOSTEXT_FN);
		mb();
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
