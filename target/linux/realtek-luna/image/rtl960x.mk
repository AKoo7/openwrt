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
  # luci-app-gpon builds (validated) but its luci-base dep needs cgi-io from the
  # 'packages' feed; add "luci-app-gpon luci" here once that feed is installed.
  # Router stack so the ONU actually routes/NATs LAN<->WAN and serves DHCP:
  #   dnsmasq           - LAN DHCPv4 + DNS server
  #   firewall4(+nft)   - lan/wan zones, lan->wan forward + masquerade (NAT)
  #   odhcpd-ipv6only   - LAN IPv6 RA/DHCPv6 server (distributes the WAN PD)
  #   odhcp6c           - WAN DHCPv6 client (PD request toward the OLT)
  DEVICE_PACKAGES := gpon-provision luci-app-gpon luci-base luci-mod-admin-full luci-theme-bootstrap uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file \
	dnsmasq firewall4 odhcpd-ipv6only odhcp6c
endef
TARGET_DEVICES += hsgq_x111w
