# SPDX-License-Identifier: GPL-2.0-only
#
# Per-board image definitions for the rtl9607f subtarget.

define Device/realtek_rtl9607f_x400axf
  DEVICE_VENDOR := HSGQ
  DEVICE_MODEL := X400AXF
  DEVICE_DTS := rtl9607f_x400axf
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
  #   WiFi AP userspace (M3d). mac80211/cfg80211 + both drivers are BUILT-IN
  #   kernel (config-6.18), NOT kmod-mac80211, so nothing pulls the userspace
  #   wifi plumbing implicitly -- list it all here:
  #     wpad-basic-mbedtls - hostapd+wpa_supplicant, WPA2-PSK(psk2)+SAE, the
  #                          same lean variant proven on realtek-luna/hsgq_x111w
  #     wifi-scripts       - /sbin/wifi + the netifd wireless handlers +
  #                          /etc/config/wireless generation (ucode variant)
  #     wireless-regdb     - /lib/firmware/regulatory.db; without it cfg80211
  #                          stays on the world regdomain and every 5 GHz
  #                          channel is no-IR = the rtw8852ce AP cannot beacon
  #                          (the 5G efuse has no RF-frontend type, rfe_type
  #                          255, so the country comes from config, not HW)
  #     iw                 - on-device verification (iw dev / iw reg get)
  #   PPPoE WAN client (runtime-selectable; the default WAN stays DHCP/IPoE -
  #   flip with `uci set network.wan.proto='pppoe'` + username/password):
  #     ppp ppp-mod-pppoe - pppd + the rp-pppoe plugin (PADI/PADO/PADR/session)
  #       and the netifd ppp/pppoe proto handlers
  #     kmod-ppp kmod-pppox kmod-pppoe - PPP core + PPPoE socket/session kernel
  #       modules (their package KCONFIG brings CONFIG_PPP/PPPOE=m into the
  #       kernel build; the lean per-model config-6.18 needs no hand edit)
  #   UPnP IGD + NAT-PMP/PCP (miniupnpd), so an application on the LAN can open
  #   its own inbound port - games, consoles, conferencing - which is what a home
  #   gateway is expected to do and what the suite's four upnp_igd_* cases probe.
  #
  #     miniupnpd-nftables - THE VARIANT IS NOT A PREFERENCE, IT MUST MATCH THE
  #       FIREWALL. This image is fw4/nftables (CONFIG_PACKAGE_firewall4=y and
  #       `# CONFIG_PACKAGE_firewall is not set`; nftables-json + kmod-nft-* are
  #       in, no iptables anywhere). The two variants differ in the daemon's
  #       COMPILED backend (--firewall=nftables vs =iptables), so the iptables
  #       build would drive a ruleset this image does not have. It is also the
  #       package's own DEFAULT_VARIANT and CONFLICTS with -iptables; naming it
  #       explicitly keeps that a decision rather than an inherited default.
  #       libmnl/libnftnl are already in the image; it adds libuuid + libcap-ng.
  #
  #     ★ THE WAN SURFACE AT REST IS UNCHANGED, and that is checkable rather
  #       than asserted: the package installs three nft chains into fw4
  #       (upnp_prerouting / upnp_forward / upnp_postrouting, in
  #       /usr/share/nftables.d/) and they ship EMPTY, with only a jump into
  #       each. The daemon listens on the LAN side (internal_iface) and adds no
  #       WAN listener, so an unsolicited-inbound scan still finds nothing.
  #       What changes is that the WAN becomes LAN-CONTROLLABLE: a LAN host can
  #       ask for a hole. base-files/etc/uci-defaults/30-upnp-secure is what
  #       bounds that (secure_mode, high ports only, terminal deny) and
  #       upnp_igd_wan_exposure is what proves the daemon stays LAN-only.
  #   IPv6 to the LAN (odhcpd-ipv6only): the RA and DHCPv6 server. WITHOUT IT
  #   THE DEVICE CANNOT HAND IPv6 TO A CLIENT AT ALL, whatever address the LAN
  #   bridge carries -- MEASURED 2026-08-13, and it is the same shape as the
  #   UPnP defect found the same day: the image shipped the CONFIGURATION for a
  #   daemon it did not ship. Applying the lab's static /64 wrote
  #   dhcp.lan.ra='server' / dhcpv6='server' happily, then died on
  #   `/etc/init.d/odhcpd: not found`. Unlike UPnP this was NOT a silent build
  #   drop -- the config says `# CONFIG_PACKAGE_odhcpd is not set`, an explicit
  #   omission -- so the new selected-package guard would (correctly) never have
  #   flagged it.
  #
  #     odhcpd-ipv6only, NOT plain odhcpd: dnsmasq already serves DHCPv4 here,
  #       and the full variant would take that over. The -ipv6only build does
  #       RA + DHCPv6 + prefix delegation and leaves v4 alone, which is exactly
  #       the split this image already has and is upstream's own default.
  #
  #     ★ SCOPE THE CLAIM IT SUPPORTS. This makes the device able to SERVE a
  #       prefix; it does not obtain one -- the upstream here delegates none
  #       (a /128, no IA_PD), which is why the lab uses a declared static /64.
  #       So a green v6 case says "routes and forwards IPv6 when given a
  #       prefix", never "IPv6 works in production". Keep DHCPv6-PD correct so
  #       it works the day a real delegation arrives.
  DEVICE_PACKAGES := dnsmasq firewall4 \
	luci-base luci-mod-admin-full luci-theme-bootstrap \
	uhttpd uhttpd-mod-ubus rpcd rpcd-mod-file \
	wpad-basic-mbedtls wifi-scripts wireless-regdb iw \
	ppp ppp-mod-pppoe kmod-ppp kmod-pppox kmod-pppoe \
	miniupnpd-nftables \
	odhcpd-ipv6only
endef
TARGET_DEVICES += realtek_rtl9607f_x400axf
