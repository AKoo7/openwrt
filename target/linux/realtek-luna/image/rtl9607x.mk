# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl9607x subtarget.
# Bring-up is run-from-RAM: the initramfs uImage is TFTP'd into RAM and
# bootm'd by the vendor U-Boot ("9607C#"), no flash write during bring-up.

define Device/realtek_rtl9607c
  DEVICE_VENDOR := Realtek
  DEVICE_MODEL := RTL9607C
  DEVICE_DTS := rtl9607c_engboard
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9607c
  # M1 brings the SoC up headless to a serial console + initramfs shell; the
  # full router/GPON package set is added once the 9607C datapath drivers land.
endef
TARGET_DEVICES += realtek_rtl9607c
