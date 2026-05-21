# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl960x subtarget.
# Boards are brought up run-from-RAM first: the initramfs uImage is TFTP'd
# into RAM and bootm'd by the vendor U-Boot (no flash write during bring-up).

define Device/hsgq_x111w
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X111W
  DEVICE_DTS := rtl9602c_x111w
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9602c
  DEVICE_PACKAGES :=
endef
TARGET_DEVICES += hsgq_x111w
