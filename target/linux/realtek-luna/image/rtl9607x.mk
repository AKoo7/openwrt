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

# LANLY G24W (RTL9603CVD). Same interAptiv MIPS32 R2 core as the RTL9607C
# above -- MEASURED from the board's own /proc/cpuinfo ("MIPS interAptiv
# V2.0", isa mips32r2, tlb_entries 64) -- so it belongs in THIS subtarget
# and shares its 24kc toolchain. It is emphatically NOT an rtl960x/RLX part
# despite the "9603" in the name; see rtl9603cvd.dtsi for that argument.
define Device/lanly_g24w
  DEVICE_VENDOR := LANLY
  DEVICE_MODEL := G24W
  DEVICE_DTS := rtl9603cvd_g24w
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-luna
  SOC := rtl9603cvd
  # Flash partition sizes MEASURED from the live device /proc/mtd:
  #   k0 = 0x334000 (3280k) kernel, r0 = 0xaa0000 (10880k) rootfs.
  # KERNEL_SIZE auto-fires check-size on the kernel (image.mk), so a kernel
  # that would not fit k0 fails the BUILD instead of being discovered on the
  # device. IMAGE_SIZE records the measured rootfs bound for the same reason.
  # NOTE these duplicate the DTS partition table by hand -- nothing in the
  # build checks the two agree, so they are changed together or not at all.
  KERNEL_SIZE := 3280k
  IMAGE_SIZE := 10880k
  # M1 is a headless run-from-RAM bring-up: serial console + initramfs shell,
  # nothing else is proven on this board yet (no Ethernet driver match, no
  # PCIe/WiFi bus identified, no GPON). Device/Default leaves IMAGES empty, so
  # this builds the initramfs uImage only -- no flashable artifact is produced
  # for a board whose flash controller base is still unverified. The router /
  # GPON / WiFi package set is added once each of those lands.
endef
TARGET_DEVICES += lanly_g24w
