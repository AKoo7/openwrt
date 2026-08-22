# OpenWrt for the Zyxel AOT-5221Zy (RTL9607F "Elnath" GPON ONU)

OpenWrt board support for the **Zyxel AOT-5221Zy**, a GPON fibre ONU built on the
**Realtek RTL9607F "Elnath"** SoC (a Cortina-Access CA8277C / "Taurus" part).

## Features
- GPON MAC bring-up: ranging, PLOAM, OMCI, GEM (reaches OMCI state O5)
- Clean OMCI / PLOAM / GEM software stack (`drivers/net/gpon`)
- 4x Gigabit LAN via DSA (`cortina-ni-dsa` + `tag_cortina`)
- GPON PPPoE WAN
- Optional: a kernel seam to run the **stock vendor OMCI daemon** in a chroot
  (`cortina-omci-bridge` + the `omci_stock` service) - **disabled by default**

## Attribution / provenance

This target is a **GPL-2.0 derivative** of the `realtek-elnath` target from
**[alphaonex86/openwrt](https://github.com/alphaonex86/openwrt)** by
**BRULE Herman** (`alpha_one_x86@first-world.info`), which is itself built on
**Cortina-Access GPL-2.0** datapath sources (`(c) Cortina-Access Limited`).

The Cortina NI/GPON/L3FE MAC driver core, the SoC peripheral drivers (PCIe, QSPI,
UART, GPIO), and the OpenWrt target scaffolding originate with alphaonex86 and
Cortina-Access. Added for this AOT-5221Zy port by **AK Sharma** (`@AKoo7`):

- the `drivers/net/gpon` OMCI / PLOAM / GEM software stack;
- `cortina-omci-bridge` (stock-daemon seam) and the `omci_stock` service;
- DSA support (`cortina-ni-dsa`, `net/dsa/tag_cortina`) and ethtool ops;
- the AOT-5221Zy board DTS and the base-files.

See **`PROVENANCE.md`** for the file-by-file breakdown.

## License

GPL-2.0. Every source file retains its SPDX identifier and upstream copyright
notices.

## Build

See **`BUILDING.md`** for full, reproducible build steps (host prerequisites,
feeds, target/subtarget/profile selection, and how to RAM-boot the result).
In short: `make menuconfig` -> Target `Realtek Elnath GPON ONU` / Subtarget
`Realtek RTL9607F Cortex-A55` / Profile `Zyxel AOT5221ZY`, then `make -j$(nproc)`.

## Configuration notes

- **PPPoE:** set your ISP credentials in `/etc/config/network`
  (`option username` / `option password`, both shipped as `CHANGEME`) and the
  WAN VLAN on `gpon0`.
- **Serial number:** read at runtime from the board's NAND `ubi_Config`
  (`GPON_SN`); no serial is baked into the image.
- **`omci_stock`:** disabled by default. It runs a **stock vendor OMCI daemon
  that is NOT distributed here** - you must supply it from your own device.

## Caveats

- `cortina-gpon-bosa-cal.h` holds a **per-board** laser-calibration image for a
  single bring-up board. It MUST be replaced with your board's own calibration
  (from `/var/config/rtkbosa_k.bin`) before flashing any other unit.
- Reverse-engineered community port, no warranty. Flashing GPON firmware can
  brick the device and/or violate your ISP's terms - proceed at your own risk.
