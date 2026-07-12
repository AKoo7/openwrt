# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl9607f subtarget.
# M1 brings the SoC up headless to a serial console + initramfs shell; the
# router/GPON package set is added once the Elnath datapath drivers land.

define Device/realtek_rtl9607f_x411axf
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X411AXF
  DEVICE_DTS := rtl9607f_x411axf
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-elnath
  SOC := rtl9607f
endef
TARGET_DEVICES += realtek_rtl9607f_x411axf
