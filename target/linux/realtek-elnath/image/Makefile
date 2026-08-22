# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

# Elnath U-Boot (2022.10, "Elnath-SoC") boots a FIT image carrying an
# lzma-compressed aarch64 kernel + the board DTB (as the stock ubi_k0 does:
# FIT with kernel(lzma)+fdt). Bring-up is run-from-RAM: the initramfs FIT is
# TFTP'd into RAM and bootm'd at the U-Boot prompt (stop key Ctrl+A), so NAND
# is never written during bring-up.
#
# The aarch64 kernel Image is position-independent and relocates itself, so the
# FIT load/entry can sit at any 2 MiB-aligned DRAM offset; 0x02000000 keeps it
# clear of the low memory U-Boot/ATF use and of the TFTP landing area.
# ★ 2026-08-07 AOT5221ZY bring-up: 0x02000000 is inside this board's production-DTB
# reserved region voip_buffer@2000000, and a kernel loaded there dies silently right
# after "Starting kernel ..." (no earlycon output, board resets). The known-good
# load address on this exact board is 0x0 (our working rtl9607f FIT uses it).
KERNEL_LOADADDR := 0x00000000

define Device/Default
  PROFILES := Default
  KERNEL_NAME := Image
  # Deferred '=' (not ':='): DEVICE_DTS is only set in the per-device stanza,
  # so the dtb path must expand at device-build time, not at Device/Default parse.
  # ★ The vendor Taurus U-Boot enforces CONFIG_SYS_BOOTM_LEN ~16 MiB (0x1000000)
  # on the loaded kernel for BOTH compressed AND uncompressed FITs ("Image too
  # large: increase CONFIG_SYS_BOOTM_LEN" -> silent reset -> NAND autoboot), so
  # the Image-initramfs MUST stay < 16 MiB (see memory rtl9607f-uboot-boot-verb).
  # DSA pushed it to 16.16 MiB; tcpdump-mini/libpcap were dropped to get back
  # under.  If it grows past 16 MiB again, trim more or split the initramfs into
  # a separate FIT ramdisk node (kernel-only Image is ~14 MiB).
  KERNEL_INITRAMFS = kernel-bin | lzma | fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb with-initrd
  KERNEL = kernel-bin | lzma | fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb
  IMAGE_NAME = $$(IMAGE_PREFIX)-$$(2)
  IMAGES :=
endef

include $(SUBTARGET).mk

$(eval $(call BuildImage))
