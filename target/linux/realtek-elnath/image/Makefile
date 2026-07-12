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
KERNEL_LOADADDR := 0x02000000

define Device/Default
  PROFILES := Default
  KERNEL_NAME := Image
  # Deferred '=' (not ':='): DEVICE_DTS is only set in the per-device stanza,
  # so the dtb path must expand at device-build time, not at Device/Default parse.
  KERNEL_INITRAMFS = kernel-bin | lzma | fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb
  KERNEL = kernel-bin | lzma | fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb
  IMAGE_NAME = $$(IMAGE_PREFIX)-$$(2)
  IMAGES :=
endef

include $(SUBTARGET).mk

$(eval $(call BuildImage))
