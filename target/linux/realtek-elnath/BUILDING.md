# Building OpenWrt for the Zyxel AOT-5221Zy (RTL9607F "Elnath")

⚠️ **Read `README.md` first.** This is a reverse-engineered community port of a
GPON ONU. Flashing/booting it can brick the device (recovery is UART-only) and,
on a shared PON, a misbehaving ONU can disrupt other subscribers. The shipped
`cortina-gpon-bosa-cal.h` is **one board's** laser calibration — do not run a
persistent image built with it on a different unit. Use a spare unit with serial
access.

This branch is a complete OpenWrt tree with the `realtek-elnath` target added,
so you build it like any OpenWrt image.

Target facts (from the target Makefiles):

| | value |
|---|---|
| Target (BOARD)   | `realtek-elnath` — "Realtek Elnath GPON ONU" |
| Subtarget        | `rtl9607f` — "Realtek RTL9607F Cortex-A55" (aarch64) |
| Device profile   | `realtek_rtl9607f_aot5221zy` — Zyxel AOT5221ZY |
| Kernel           | 6.18 |
| Boot model       | run-from-RAM: initramfs FIT TFTP'd into RAM by the vendor U-Boot |

## 1. Build host

Any OpenWrt-capable Linux host. Install the standard build prerequisites — see
https://openwrt.org/docs/guide-developer/toolchain/install-buildsystem
(Debian/Ubuntu: `build-essential`, `clang`, `flex`, `bison`, `g++`, `gawk`,
`gcc-multilib`, `git`, `libncurses-dev`, `libssl-dev`, `python3`, `rsync`,
`unzip`, `zlib1g-dev`, `file`, `wget`). Do **not** build as root.

## 2. Get the source

```
git clone https://github.com/AKoo7/openwrt.git aot5221zy
cd aot5221zy
git checkout aot5221zy
```

## 3. Feeds

```
./scripts/feeds update -a
./scripts/feeds install -a
```

## 4. Configure

Interactive (source of truth):

```
make menuconfig
```
- **Target System**  → `Realtek Elnath GPON ONU`
- **Subtarget**      → `Realtek RTL9607F Cortex-A55`
- **Target Profile** → `Zyxel AOT5221ZY`

…then exit and save. Equivalent non-interactive seed:

```
cat > .config <<'CFG'
CONFIG_TARGET_realtek_elnath=y
CONFIG_TARGET_realtek_elnath_rtl9607f=y
CONFIG_TARGET_realtek_elnath_rtl9607f_DEVICE_realtek_rtl9607f_aot5221zy=y
CFG
make defconfig
```

## 5. Build

```
make -j$(nproc)          # add V=s for verbose, download first with: make download
```

First build compiles the whole toolchain and can take a while.

## 6. Output

```
bin/targets/realtek-elnath/rtl9607f/
```
The primary artifact is the **initramfs FIT** (RAM-boot):
`openwrt-realtek-elnath-rtl9607f-realtek_rtl9607f_aot5221zy-initramfs-kernel.bin`.

## 7. Boot it (RAM, non-destructive)

The safe way to try this port is to run it from RAM without touching NAND — a
power-cycle returns the unit to stock:

1. Attach UART (3V3) and open the serial console; power on.
2. Stop autoboot at the vendor U-Boot prompt (stop key **Ctrl+A** on the
   "Elnath-SoC" U-Boot 2022.10; some units use a different key).
3. Serve the initramfs FIT over TFTP, load it into RAM, and `bootm` it.

⚠️ Exact load addresses and boot verbs depend on your unit's bootloader
(there is a ~16 MiB `bootm` decompress cap — the image is split kernel+ramdisk
to stay under it). **Before any NAND write, dump and keep a full stock backup;
have a UART recovery path ready.** NAND installation is advanced and out of
scope here.

## Provenance / license

GPL-2.0; derived from `alphaonex86/openwrt realtek-elnath` (BRULE Herman) on
Cortina-Access GPL-2.0 sources. See `README.md` and `PROVENANCE.md`.
