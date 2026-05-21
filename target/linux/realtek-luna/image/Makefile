# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

# Realtek "Luna" U-Boot expects a legacy uImage: MIPS/Linux/kernel, LZMA
# compressed, load+entry at 0x80000000 (per vendor fwu.sh / on-board uImage).
# Bring-up is run-from-RAM: a uImage carrying an embedded initramfs is
# TFTP'd into RAM and bootm'd; no flash write during bring-up.
KERNEL_LOADADDR:=0x80000000

define Device/Default
  PROFILES := Default
  KERNEL_NAME := vmlinux
  KERNEL_INITRAMFS := kernel-bin | lzma | uImage lzma
  KERNEL := kernel-bin | lzma | uImage lzma
  IMAGE_NAME = $$(IMAGE_PREFIX)-$$(2)
  IMAGES :=
endef

include $(SUBTARGET).mk

$(eval $(call BuildImage))
