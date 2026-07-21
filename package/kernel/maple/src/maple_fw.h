/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Firmware bring-up stub — open reimplementation of the vendor fld.ko BCM FLD
 * layer (see docs/decomp/fld.md). Pushes the Maple bootloader + application
 * images into the chip's SRAM/DDR and runs the host/device boot-record
 * handshake until CPU_READY, then sets ring/MTU sizes.
 *
 * Status: STUB. Loads the two firmware blobs via request_firmware() and pins
 * them in the per-device state; the boot-record handshake register sequence is
 * TODO (recoverable from fld.ko DWARF + the comm-area layout in BAR2/4).
 */
#ifndef MAPLE_FW_H
#define MAPLE_FW_H

#include <linux/types.h>
#include <linux/firmware.h>

struct maple_dev;

#define MAPLE_FW_BOOT	"bcm68620/bcm68620_boot.bin"
#define MAPLE_FW_APPL	"bcm68620/bcm68620_appl.bin"

struct maple_fw {
	const struct firmware	*boot;	/* bootloader image (43 KB) */
	const struct firmware	*appl;	/* application image (11.78 MB) */
	bool			cpu_ready;
};

int  maple_fw_load(struct maple_fw *fw, struct maple_dev *mdev);
void maple_fw_unload(struct maple_fw *fw);

#endif /* MAPLE_FW_H */
