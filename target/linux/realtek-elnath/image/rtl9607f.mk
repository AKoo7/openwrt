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
  #   LuCI web UI (plain http on the LAN, same lean set proven on
  #   realtek-luna/hsgq_x111w -- NOT the `luci` collection, which would drag in
  #   luci-app-package-manager; no ssl/nginx variants on this lab HGU):
  #     uhttpd uhttpd-mod-ubus - the web server on :80 + the ubus JSON-RPC
  #                              bridge LuCI's client-side JS talks to
  #     rpcd rpcd-mod-file     - the ubus RPC daemon backing session auth,
  #                              uci access and file reads for the UI
  #     luci-base luci-mod-admin-full luci-theme-bootstrap - the UI core,
  #       the standard admin pages (status/system/network) and the theme
  DEVICE_PACKAGES := dnsmasq firewall4 \
	luci-base luci-mod-admin-full luci-theme-bootstrap \
	uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file
endef
TARGET_DEVICES += realtek_rtl9607f_x411axf
