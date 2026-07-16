# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl9607f subtarget.

define Device/realtek_rtl9607f_x411axf
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X411AXF
  DEVICE_DTS := rtl9607f_x411axf
  DEVICE_DTS_DIR := $(DTS_DIR)/realtek-elnath
  SOC := rtl9607f
  # IPv4 HGU router layer (same pattern as realtek-luna/hsgq_x111w):
  #   dnsmasq   - LAN DHCPv4 pool + DNS forwarder (default variant = no DHCPv6;
  #               prod is IPv4-only). Ships the standard /etc/config/dhcp
  #               (lan pool .100-.249 12h, wan ignored).
  #   firewall4 - lan/wan zones, lan->wan forward + wan masquerade (NAT) +
  #               mtu_fix, wan input REJECT (closes WAN-side SSH/mgmt). Pulls
  #               nftables-json + kmod-nft-{core,fib,offload,nat} (and through
  #               them nf_conntrack/nf_nat) so the netfilter kernel side comes
  #               from the package KCONFIG, not hand-edited target config.
  #               Ships the standard /etc/config/firewall.
  DEVICE_PACKAGES := dnsmasq firewall4
endef
TARGET_DEVICES += realtek_rtl9607f_x411axf
