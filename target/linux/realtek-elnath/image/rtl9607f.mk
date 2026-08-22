# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl9607f subtarget.

# Zyxel AOT5221ZY — RTL9607F "Elnath" GPON ONT.  Cold-boots a split kernel+ramdisk
# FIT (separate_ramdisk + `fit ... with-initrd`, see image/Makefile) to fit the
# vendor U-Boot's ~16 MiB bootm-decompress cap.
define Device/realtek_rtl9607f_aot5221zy
  DEVICE_VENDOR := Zyxel
  DEVICE_MODEL := AOT5221ZY
  DEVICE_DTS := rtl9607f_aot5221zy
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-elnath
  SOC := rtl9607f
  # GPON PPPoE router stack: fw4 (nftables NAT), PPPoE (ppp/ppp-mod-pppoe ->
  # kmod-ppp/pppoe/pppox), dnsmasq (LAN DHCP/DNS), odhcpd (IPv6).  8021q builtin
  # (config-6.18).
  DEVICE_PACKAGES := -firewall firewall4 ppp ppp-mod-pppoe dnsmasq odhcpd-ipv6only
endef
TARGET_DEVICES += realtek_rtl9607f_aot5221zy
