// SPDX-License-Identifier: GPL-2.0
/*
 * Open Maple GPON MAC driver — PCIe glue + interrupt layer.
 *
 * Target: Broadcom "Maple" BCM68620 combo GPON/XGS-PON/EPON OLT MAC,
 *         PCI 14e4:6862 / 14e4:6863 / 14e4:5554 ("maple line card").
 *
 * This is the open reimplementation of the vendor ll_pcie.ko + bcmtr_pcie
 * transport + fld firmware-load + bcm_dev_ctrl_linux ioctl layers, written from
 * their DWARF decompiles (see docs/decomp/). The PCI glue, BAR mapping and the
 * interrupt model here are COMPLETE and match the hardware contract in
 * docs/decomp/bar0-register-map.md. The firmware bring-up and the PCIe ring
 * transport are stubbed with exact offsets + TODOs (next implementation phase).
 *
 * The companion firmware blobs are:
 *   /lib/firmware/bcm68620/bcm68620_boot.bin   (43 KB bootloader)
 *   /lib/firmware/bcm68620/bcm68620_appl.bin   (11.78 MB application)
 * recovered from the stock kernel's embedded initramfs (work/initramfs/broadcom).
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include "maple_hw.h"
#include "maple_pci.h"
#include "maple_gnl.h"

/* Per-device state lives in struct maple_dev (maple_pci.h): the open
 * equivalent of the vendor's 0x3c-byte maple_device_info + the 0x98-byte
 * bcm_pcied_comm_data. */

/* The first probed Maple MAC, for the genetlink userspace ABI (maple_gnl.c). */
struct maple_dev *maple_first_dev;
EXPORT_SYMBOL_GPL(maple_first_dev);

static bool nomsi;
module_param(nomsi, bool, 0444);
MODULE_PARM_DESC(nomsi, "fall back to legacy INTx instead of MSI");

/* RX upcall shim: ring → BAL dispatch. */
static void maple_rx_upcall(void *ctx, struct sk_buff *skb)
{
	struct maple_dev *mdev = ctx;

	maple_bal_rx(mdev, skb);
	dev_kfree_skb_any(skb);
}

/* ------------------------------------------------------------------ IRQ -- */
/* Exact decode of the Maple BAR0 interrupt block (see bar0-register-map.md):
 *   status@0x64318, mask@0x64324  ->  pending = status & ~mask
 *   bits: 0=TX-done 1=TX-err 2=RX 3=RX-err ; clear via write-1-to-@0x64320 */
static irqreturn_t maple_isr(int irq, void *data)
{
	struct maple_dev *mdev = data;
	void __iomem *bar0 = mdev->bar0;
	u32 status, mask, pending;
	irqreturn_t ret = IRQ_NONE;

	status = maple_rd(bar0, MAPLE_BAR0_ISR_STATUS);
	mask = maple_rd(bar0, MAPLE_BAR0_ISR_MASK);
	pending = status & ~mask;

	if (pending & MAPLE_IRQ_RX) {
		maple_ring_rx(&mdev->ring);		/* TODO: full bcmtr_pcie_receive */
		maple_irq_clear(bar0, MAPLE_IRQ_RX_MASK);
		ret = IRQ_HANDLED;
	}
	if (pending & MAPLE_IRQ_TX_DONE) {
		maple_ring_tx_done(&mdev->ring);	/* TODO: bcmtr_pcie_tx_collect  */
		maple_irq_clear(bar0, MAPLE_IRQ_TX_MASK);
		ret = IRQ_HANDLED;
	}
	if (status & (MAPLE_IRQ_TX_ERR | MAPLE_IRQ_RX_ERR)) {
		dev_warn(&mdev->pdev->dev, "irq error status %#x\n", status);
		maple_irq_clear(bar0, MAPLE_IRQ_TX_ERR | MAPLE_IRQ_RX_ERR);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static int maple_irq_setup(struct maple_dev *mdev)
{
	int ret;

	if (!nomsi && pci_enable_msi(mdev->pdev) == 0)
		mdev->irq = mdev->pdev->irq;
	else
		mdev->irq = mdev->pdev->irq;

	/* Mask everything before requesting, mirror the vendor enable sequence. */
	maple_irq_disable(mdev->bar0, MAPLE_IRQ_ALL);
	maple_irq_clear(mdev->bar0, MAPLE_IRQ_ALL);

	ret = request_irq(mdev->irq, maple_isr, 0, DRV_NAME, mdev);
	if (ret) {
		dev_err(&mdev->pdev->dev, "request_irq(%d) failed: %d\n",
			mdev->irq, ret);
		return ret;
	}
	return 0;
}

static void maple_irq_teardown(struct maple_dev *mdev)
{
	maple_irq_disable(mdev->bar0, MAPLE_IRQ_ALL);
	free_irq(mdev->irq, mdev);
	pci_disable_msi(mdev->pdev);
}

/* --------------------------------------------------------------- probe -- */
static int maple_map_bars(struct pci_dev *pdev, struct maple_dev *mdev)
{
	int ret;

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "pci_request_regions: %d\n", ret);
		return ret;
	}
	mdev->bar0 = pci_ioremap_bar(pdev, 0);
	mdev->bar2 = pci_ioremap_bar(pdev, 2);
	mdev->bar4 = pci_ioremap_bar(pdev, 4);
	if (!mdev->bar0 || !mdev->bar2 || !mdev->bar4) {
		dev_err(&pdev->dev, "failed to ioremap BAR0/2/4\n");
		return -ENOMEM;
	}
	mdev->bar2_sz = pci_resource_len(pdev, 2);
	return 0;
}

static void maple_unmap_bars(struct pci_dev *pdev, struct maple_dev *mdev)
{
	if (mdev->bar0) iounmap(mdev->bar0);
	if (mdev->bar2) iounmap(mdev->bar2);
	if (mdev->bar4) iounmap(mdev->bar4);
	pci_release_regions(pdev);
}

static int maple_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct maple_dev *mdev;
	u32 devid;
	int ret;

	dev_info(&pdev->dev, "probe bus=%02x devfn=%02x vendor=%04x device=%04x rev=%02x\n",
		 pdev->bus->number, pdev->devfn, pdev->vendor, pdev->device,
		 pdev->revision);

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;
	mdev->pdev = pdev;
	pci_set_drvdata(pdev, mdev);

	ret = pci_enable_device_mem(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pci_enable_device_mem: %d\n", ret);
		return ret;
	}
	pci_set_master(pdev);
	pci_save_state(pdev);

	ret = maple_map_bars(pdev, mdev);
	if (ret)
		goto err_disable;

	/* Confirm the Maple MAC: vendor DEVICE_ID register = (id<<8)|rev. */
	devid = maple_rd(mdev->bar0, MAPLE_BAR0_DEVICE_ID);
	dev_info(&pdev->dev, "BAR0 device_id=%02x rev=%02x capability=%#x\n",
		 (devid >> 8) & 0xff, devid & 0xff,
		 maple_rd(mdev->bar0, MAPLE_BAR0_CAPABILITY));

	ret = maple_ring_alloc(&mdev->ring, mdev);
	if (ret)
		goto err_unmap;
	mdev->ring.rx_ctx = mdev;
	mdev->ring.rx_upcall = maple_rx_upcall;

	maple_bal_init(mdev);

	ret = maple_fw_load(&mdev->fw, mdev);	/* fld boot handshake */
	if (ret) {
		dev_err(&pdev->dev, "firmware bring-up failed: %d\n", ret);
		goto err_ring;
	}

	ret = maple_ring_connect(&mdev->ring);	/* bcmtr_pcie_connect — TODO */
	if (ret)
		goto err_fw;

	ret = maple_irq_setup(mdev);
	if (ret)
		goto err_disc;

	/* Enable RX/TX interrupts (vendor: unmask 0xc / 0x3 at 0x6432c). */
	maple_irq_enable(mdev->bar0, MAPLE_IRQ_RX_MASK | MAPLE_IRQ_TX_MASK);
	mdev->running = true;
	if (!maple_first_dev)
		maple_first_dev = mdev;

	dev_info(&pdev->dev, "Maple MAC up (bar0=%p bar2=%p/%zx bar4=%p irq=%d)\n",
		 mdev->bar0, mdev->bar2, mdev->bar2_sz, mdev->bar4, mdev->irq);
	return 0;

err_disc:
	maple_ring_disconnect(&mdev->ring);
err_fw:
	maple_fw_unload(&mdev->fw);
err_ring:
	maple_ring_free(&mdev->ring);
err_unmap:
	maple_unmap_bars(pdev, mdev);
err_disable:
	pci_disable_device(pdev);
	return ret;
}

static void maple_remove(struct pci_dev *pdev)
{
	struct maple_dev *mdev = pci_get_drvdata(pdev);

	if (!mdev)
		return;
	mdev->running = false;
	if (maple_first_dev == mdev)
		maple_first_dev = NULL;

	maple_irq_teardown(mdev);

	/* Vendor disconnect: CTRL bit0 off, teardown writes. */
	maple_wr(mdev->bar0, MAPLE_BAR0_CTRL, maple_rd(mdev->bar0, MAPLE_BAR0_CTRL) & ~BIT(0));
	maple_wr(mdev->bar0, MAPLE_BAR0_CTRL_SETUP_A, 0x20);

	maple_ring_disconnect(&mdev->ring);
	maple_fw_unload(&mdev->fw);
	maple_ring_free(&mdev->ring);
	maple_bal_exit(mdev);
	maple_unmap_bars(pdev, mdev);
	pci_disable_device(pdev);
	dev_info(&pdev->dev, "removed\n");
}

static const struct pci_device_id maple_ids[] = {
	{ PCI_DEVICE(MAPLE_VENDOR_BROADCOM, MAPLE_DEV_6862) },
	{ PCI_DEVICE(MAPLE_VENDOR_BROADCOM, MAPLE_DEV_6863) },
	{ PCI_DEVICE(MAPLE_VENDOR_BROADCOM, MAPLE_DEV_5554) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, maple_ids);

static struct pci_driver maple_driver = {
	.name		= DRV_NAME,
	.id_table	= maple_ids,
	.probe		= maple_probe,
	.remove		= maple_remove,
};

static int __init maple_init(void)
{
	int ret = pci_register_driver(&maple_driver);

	if (ret)
		return ret;
	ret = maple_gnl_init();
	if (ret)
		pci_unregister_driver(&maple_driver);
	return ret;
}

static void __exit maple_exit(void)
{
	maple_gnl_exit();
	pci_unregister_driver(&maple_driver);
}

module_init(maple_init);
module_exit(maple_exit);

MODULE_AUTHOR("open-maple RE project");
MODULE_DESCRIPTION("Broadcom Maple BCM68620 GPON MAC open driver (PCIe glue)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("bcm68620/bcm68620_boot.bin");
MODULE_FIRMWARE("bcm68620/bcm68620_appl.bin");
