// SPDX-License-Identifier: GPL-2.0
/*
 * Firmware bring-up — open reimplementation of the vendor fld.ko BCM FLD layer.
 * See docs/decomp/fld.md + docs/decomp/bar0-register-map.md for the derivation.
 *
 * Boot flow (recovered from fld DWARF):
 *   1. host-reset the MAC
 *   2. stream bcm68620_boot.bin into SRAM (BAR2) [0..0xff20]
 *   3. set mailbox: app image envelope + test_ddr flag
 *   4. start the Maple CPU (BAR0 boot-control regs)
 *   5. poll SRAM CPU_STATE until bootloader done
 *   6. stream bcm68620_appl.bin into DDR (BAR4) [0..ddr_len]
 *   7. host_finish_write_ddr (os_entry + finish flag)
 *   8. poll SRAM CPU_STATE until CPU_READY (bit3)
 *
 * NOTE: the exact interleaving of host_flag/dev_flag mailbox handshakes is
 * orchestrated by the BAL core (bcm.user). This implements the register/mailbox
 * primitives faithfully; the top-level sequence matches fld's documented state
 * machine. Unverified on hardware (no device yet) — see TODOs.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/delay.h>

#include "maple_hw.h"
#include "maple_pci.h"

#define MAPLE_BOOT_TIMEOUT_MS	30000	/* bootloader + DDR test */
#define MAPLE_APPL_TIMEOUT_MS	60000	/* application boot */

/* Write a contiguous image into an MMIO window as 32-bit words with a barrier
 * per store, matching bcm_fld_write (the vendor does DSB after every word). */
static void maple_fw_memcpy32(void __iomem *dst, const void *src, size_t len)
{
	const u32 *p = src;
	size_t n = (len + 3) / 4;

	while (n--) {
		writel(*p++, dst);
		dst += 4;
		wmb();
	}
}

static int maple_fw_wait_state(struct maple_dev *mdev, u32 want, u32 err,
			       unsigned int timeout_ms)
{
	void __iomem *sram = mdev->bar2;
	u32 st;
	unsigned int t;

	for (t = 0; t < timeout_ms; t += 10) {
		st = maple_rd(sram, MAPLE_SRAM_CPU_STATE);
		if (st & err)
			return -EIO;
		if (st & want)
			return 0;
		msleep(10);
	}
	dev_err(&mdev->pdev->dev, "firmware state timeout: want=%#x got=%#x\n",
		want, st);
	return -ETIMEDOUT;
}

/* ---- mailbox primitives (mirror of fld bcm_fld_*) ------------------------ */

/* Ring/MTU parameters must be set BEFORE starting the CPU (bcm_fld_set_rings_size
 * + bcm_fld_set_mtu_size in the vendor). The firmware reads these from the SRAM
 * mailbox during init to size its DMA rings. These MUST match the host-side
 * ring geometry in maple_ring.c (MAPLE_DFLT_{RXQ,TXQ,MTU}). */
#define MAPLE_RING_TX_DEPTH	MAPLE_DFLT_TXQ
#define MAPLE_RING_RX_DEPTH	MAPLE_DFLT_RXQ
#define MAPLE_RING_MTU		MAPLE_DFLT_MTU

/* The application firmware starts with a 180-byte bcmolt_firmware_envelope
 * header (BE). The vendor parses it in bcm_fld_set_app_image_params and writes
 * the fields to the SRAM mailbox, then streams only the PAYLOAD (file+180) to
 * DDR. The envelope layout (from fld.ko DWARF):
 *   +0   u8  envelope_revision
 *   +4   u8  release_major_id      → SRAM 0xffd0
 *   +5   u8  release_minor_id      → SRAM 0xffcc
 *   +6   u8  release_revision_id   → SRAM 0xffc8
 *   +8   u32 model_id (BE)         → SRAM 0xffc4
 *   +20  u32 build_time (BE)
 *   +159 u8[16] checksum           → SRAM 0xffd4 (first 4 bytes)
 *   +176 u32 image_len (BE)        → SRAM 0xffd8 (payload length, excludes header)
 */
#define MAPLE_FW_ENVELOPE_SIZE	180

static void maple_fw_set_rings_size(struct maple_dev *mdev, u32 tx_depth, u32 rx_depth)
{
	void __iomem *sram = mdev->bar2;
	maple_wr(sram, MAPLE_SRAM_HOST_TX_SIZE, tx_depth);
	maple_wr(sram, MAPLE_SRAM_HOST_RX_SIZE, rx_depth);
}

static void maple_fw_set_mtu_size(struct maple_dev *mdev, u32 mtu)
{
	void __iomem *sram = mdev->bar2;
	maple_wr(sram, MAPLE_SRAM_MTU_SIZE, mtu);
}

/* Parse the 180-byte firmware envelope from the appl.bin header and write the
 * fields to the SRAM mailbox (bcm_fld_set_app_image_params equivalent). */
static void maple_fw_set_app_params(struct maple_dev *mdev, const u8 *appl, size_t appl_len)
{
	void __iomem *sram = mdev->bar2;
	const u8 *env = appl;

	if (appl_len < MAPLE_FW_ENVELOPE_SIZE) {
		maple_wr(sram, MAPLE_SRAM_IMAGE_LENGTH, appl_len);
		return;
	}

	/* Single-byte fields (no byte-swap needed) */
	maple_wr(sram, MAPLE_SRAM_REL_MAJOR,     env[4]);  /* release_major_id */
	maple_wr(sram, MAPLE_SRAM_REL_MINOR,     env[5]);  /* release_minor_id */
	maple_wr(sram, MAPLE_SRAM_REL_REV,       env[6]);  /* release_revision_id */

	/* BE u32 fields — read as BE, store as native (LE) to the mailbox */
	maple_wr(sram, MAPLE_SRAM_MODEL_ID,
		 (env[8]  << 24) | (env[9]  << 16) | (env[10] << 8) | env[11]);
	maple_wr(sram, MAPLE_SRAM_IMAGE_CHECKSUM,
		 (env[159] << 24) | (env[160] << 16) | (env[161] << 8) | env[162]);

	/* image_len from the envelope (BE u32) = payload length (excludes header) */
	maple_wr(sram, MAPLE_SRAM_IMAGE_LENGTH,
		 (env[176] << 24) | (env[177] << 16) | (env[178] << 8) | env[179]);
}

static void maple_fw_start_cpu(struct maple_dev *mdev, u32 test_ddr)
{
	void __iomem *bar0 = mdev->bar0;
	void __iomem *sram = mdev->bar2;

	maple_wr(sram, MAPLE_SRAM_TEST_DDR, test_ddr);
	/* bcm_fld_start_bootloader: CPU-start then boot-control magic. */
	maple_wr(bar0, MAPLE_BAR0_CPU_START,  0x01000000);
	maple_wr(bar0, MAPLE_BAR0_BOOT_CTRL,  0x0c401080);
}

static void maple_fw_finish_ddr(struct maple_dev *mdev, u32 os_entry)
{
	void __iomem *sram = mdev->bar2;

	maple_wr(sram, MAPLE_SRAM_OS_ENTRY, os_entry);
	maple_wr(sram, MAPLE_SRAM_TEST_DDR, 1);	/* finish flag */
}

/* Bring-up sequence. */
int maple_fw_load(struct maple_fw *fw, struct maple_dev *mdev)
{
	void __iomem *bar0 = mdev->bar0;
	void __iomem *sram = mdev->bar2;
	void __iomem *ddr  = mdev->bar4;
	u32 proto;
	int ret;

	ret = request_firmware(&fw->boot, MAPLE_FW_BOOT, &mdev->pdev->dev);
	if (ret) {
		dev_err(&mdev->pdev->dev, "%s not found: %d\n", MAPLE_FW_BOOT, ret);
		return ret;
	}
	ret = request_firmware(&fw->appl, MAPLE_FW_APPL, &mdev->pdev->dev);
	if (ret) {
		dev_err(&mdev->pdev->dev, "%s not found: %d\n", MAPLE_FW_APPL, ret);
		goto err_boot;
	}
	if (fw->boot->size > MAPLE_BOOT_IMAGE_MAX) {
		dev_err(&mdev->pdev->dev, "boot image too large (%zu > %x)\n",
			fw->boot->size, MAPLE_BOOT_IMAGE_MAX);
		ret = -EINVAL;
		goto err_appl;
	}

	dev_info(&mdev->pdev->dev,
		 "firmware loaded: boot=%zu appl=%zu\n",
		 fw->boot->size, fw->appl->size);

	/* 1. host reset (ll_pcie: HOST_RESET = enabled ^ 1). */
	maple_wr(bar0, MAPLE_BAR0_HOST_RESET, 1);
	fsleep(1000);
	maple_wr(bar0, MAPLE_BAR0_HOST_RESET, 0);

	/* 2. protocol-version sanity read. */
	proto = maple_rd(sram, MAPLE_SRAM_PROTO_VERSION);
	dev_info(&mdev->pdev->dev, "mailbox protocol=%#x state=%#x\n", proto,
		 maple_rd(sram, MAPLE_SRAM_CPU_STATE));

	/* 3. stream bootloader into SRAM. */
	maple_fw_memcpy32(sram, fw->boot->data, fw->boot->size);

	/* 4. set ring geometry + MTU + app-image envelope, then start the CPU.
	 *    The vendor (bcm_fld_set_rings_size + bcm_fld_set_mtu_size) writes
	 *    these BEFORE the CPU starts so the firmware can size its DMA rings.
	 *    The app-image envelope (model_id/checksum/revision) is parsed from
	 *    the appl.bin's 180-byte BE header. */
	maple_fw_set_rings_size(mdev, MAPLE_RING_TX_DEPTH, MAPLE_RING_RX_DEPTH);
	maple_fw_set_mtu_size(mdev, MAPLE_RING_MTU);
	maple_fw_set_app_params(mdev, fw->appl->data, fw->appl->size);
	maple_fw_start_cpu(mdev, 1 /* test_ddr */);

	/* 5. wait for bootloader to finish + DDR self-test. */
	ret = maple_fw_wait_state(mdev, MAPLE_CPU_VERIFY_DDR | MAPLE_CPU_READY,
				  MAPLE_CPU_DDR_IMAGE_ERROR, MAPLE_BOOT_TIMEOUT_MS);
	if (ret) {
		dev_err(&mdev->pdev->dev, "bootloader failed: %d reason=%#x\n",
			ret, maple_rd(sram, MAPLE_SRAM_REASON));
		goto err_appl;
	}

	/* 6. stream application PAYLOAD into DDR (skip the 180-byte envelope header).
	 *    The firmware code starts at appl.bin + MAPLE_FW_ENVELOPE_SIZE. */
	if (fw->appl->size > MAPLE_FW_ENVELOPE_SIZE) {
		maple_fw_memcpy32(ddr,
				  fw->appl->data + MAPLE_FW_ENVELOPE_SIZE,
				  fw->appl->size - MAPLE_FW_ENVELOPE_SIZE);
	}

	/* 7. signal DDR image complete (os_entry = 0 for BAL). */
	maple_fw_finish_ddr(mdev, 0);

	/* 8. wait for CPU_READY. */
	ret = maple_fw_wait_state(mdev, MAPLE_CPU_READY,
				  MAPLE_CPU_DDR_IMAGE_ERROR, MAPLE_APPL_TIMEOUT_MS);
	if (ret) {
		dev_err(&mdev->pdev->dev, "application boot failed: %d reason=%#x\n",
			ret, maple_rd(sram, MAPLE_SRAM_REASON));
		goto err_appl;
	}

	fw->cpu_ready = true;
	dev_info(&mdev->pdev->dev, "Maple CPU ready (protocol=%#x)\n", proto);
	return 0;

err_appl:
	release_firmware(fw->appl);
	fw->appl = NULL;
err_boot:
	release_firmware(fw->boot);
	fw->boot = NULL;
	return ret;
}

void maple_fw_unload(struct maple_fw *fw)
{
	release_firmware(fw->boot);
	release_firmware(fw->appl);
	fw->boot = NULL;
	fw->appl = NULL;
	fw->cpu_ready = false;
}
