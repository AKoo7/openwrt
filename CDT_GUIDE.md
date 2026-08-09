# Changing the CDT on an IPQ9574 board

**How to safely edit Qualcomm's Configuration Data Table — the static board-description blob that
tells SBL1 how much DRAM to decode.**

Written against the Arcadyan JIDU6411/JIDU6111 (`JIO_JIDU6J11_R2.0.9`, IPQ9574, 256 MiB NAND).
Everything marked ✅ was re-verified against the on-disk artifacts on 2026-08-09; anything inferred
is labelled as such.

---

## 1. Why the CDT exists

**The IPQ9574 does not auto-detect DRAM.** A soldered-down DDR4 part has no SPD EEPROM, so nothing
on the boot path can interrogate the chip. Instead:

```
PBL → SBL1 ──reads──> 0:CDT ──> programs the DDR controller
                        │
                        └────> publishes a RAM ptable via SMEM
                                        │
                          U-Boot ───────┤  (bdinfo, and it patches the kernel DT memory node)
                          Linux DT ─────┘
```

Consequence: **U-Boot and Linux are both downstream consumers.** If the box reports the wrong RAM
size, editing the DTS or bootargs cannot fix it — the upper half of the address space is never
decoded by the controller at all. The decisive test is a write in U-Boot, *before Linux*:

| Probe (512 MB-declared board with a 1 GB chip) | Result |
|---|---|
| `mw.l 0x42000000` | fine |
| `mw.l 0x62000000` | **data abort → CPU reset** |

A fault, not an alias. Aliasing would mean "mis-sized on a real part"; a fault means **undecoded**.

The CDT also carries platform ID / DDR timings / mode registers, so this guide applies to any CDT
edit — density is just the common case.

---

## 2. Where it lives

| Partition | mtd | Contents |
|---|---|---|
| `0:CDT` | **mtd15** | the live CDT |
| `0:CDT_1` | **mtd16** | fallback, tried when the primary fails CRC |
| `0:TRAINING` | mtd22 | 2 KB of `0x0F` filler — no cached training, SBL re-trains every boot |

Partition size is 512 KiB but only the first **0x223 bytes** are meaningful; the rest is blank.

> ⚠ Address partitions as `/dev/mtdN`, never by name — the names contain a colon (`0:CDT`).

> ⚠ **On this board `0:CDT_1` shipped completely blank (all `0xFF`)** — there was no fallback at all
> from the factory. Section 6 fixes that before anything else.

SBL1 strings that prove the fallback path (`boot_images/QcomPkg/XBLLoader/boot_ddr_info.c`):

```
Warning: CRC missmatch trying alternate CDT Partition...
CDT Partition Loading Failed. Trying Alternate CDT Partition...
Error: CDT is not programmed
```

---

## 3. Container format ✅

Decoded from `get_cdt_section` @`0x8c4c42c` in SBL1, and re-verified by parsing the live dump.

```
0x00  u32   magic "CDT\0"            (LE 0x00544443)
0x04  u16   version = 2
0x06  u16   CRC low half   ─┐ compared as (hi<<16)|lo
0x08  u16   CRC high half  ─┘
0x0e  u16   section-table length (26) — and doubles as entry[0].offset
0x0e + type*4 :  section table, entries { u16 offset, u16 length }
```

Live section table from `CDT_mtd15.bin`:

| type | offset | len | meaning |
|---|---|---|---|
| 0 | `0x001a` | 5 | platform: `02 08 05 00 01` = Platform ID 8, Major 5, Minor 0, Subtype 1 |
| 1 | `0x001f` | **512** | ★ **DDR section** — what `sbl1_ddr_set_params` fetches with `r1=1` |
| 2 | `0x021f` | 4 | `4e 1f 00 00` — board-specific, see §9 |

The type-0 bytes match the SBL boot log line
`CDT Version:2, Platform ID:8, Major ID:5, Minor ID:0, Subtype:1` exactly — that's what confirmed
the format decode.

### DDR section (base = file `0x1f`, 512 bytes)

Consumer function @`0x21ab48` (SRAM segment VA `0x218000`), reached via
`memcpy(0x08600190, section, size)` from `sbl1_ddr_set_params` @`0x8c417fc`.

```
+0x00  = 3      struct version (must be >= 2, else rejected)
+0x05           "RDD" tag  ('DDR' byte-reversed)      → file 0x24
+0x0c  = 1
+0x34  = 10     ★ DDR device type, validated 2..10, drives a jump table @0x21ab94  → file 0x53
+0x30           profile A, 232 bytes (0xE8)           → file 0x04f
+0x118          profile B, 232 bytes (0xE8)           → file 0x137
                0x30 + 0xE8 + 0xE8 = 0x200 — exact fit
```

**Profiles A and B are byte-identical** (rank/channel pair) ✅. Every field therefore exists twice,
at a stride of **`0xE8`**. *Edit both or the board will not boot reliably.*

---

## 4. Verified field offsets ✅

Derived by generating `256M16_DDR4` and `512M16_DDR4` from QSDK's own XML sources and diffing the
outputs, then confirming each value in the live board dump. All fields are **u32 little-endian**.

| Field | profile A | profile B | Stock value | Units |
|---|---|---|---|---|
| `num_rows_cs0` | `0x057` | `0x13f` | 16 | rows |
| `num_cols_cs0` | `0x05b` | `0x143` | 10 | cols |
| **`device_size_cs0`** | **`0x07b`** | **`0x163`** | 512 | **megabytes** |
| `tCCD_L` | `0x0eb` | `0x1d3` | 8 | |
| `mr0` | `0x10b` | `0x1f3` | 3412 (`0x0D54`) | mode reg |
| `mr2` | `0x113` | `0x1fb` | 40 | mode reg |
| `wrlat_adj_f0` | `0x11b` | `0x203` | 12 | |
| `rdlat_adj_f0` | `0x11f` | `0x207` | 20 | |
| `mr6` | `0x12b` | `0x213` | 4096 | mode reg |

**Density is encoded in exactly one field.** QSDK naming is *words* × *width*:

| QSDK config | `device_size_cs0` | rows | cols | banks | width | ndev | = |
|---|---|---|---|---|---|---|---|
| `256M16_DDR4` | 512 | 16 | 10 | 8 | ×16 | 1 | 512 MB |
| `512M16_DDR4` | 1024 | 16 | 10 | 8 | ×16 | 1 | **1 GB** |
| `1024M32_DDR4` | 4096 | 17 | 10 | 8 | ×32 | 2 | 4 GB |

Geometry is **identical** between the 512 MB and 1 GB configs — only `device_size_cs0` moves.

---

## 5. The CRC ✅

Recovered by disassembling `SBL1_mtd0.bin`. Tool: **`~/jidu6411/cdt_crc.py`** (round-trips against
both the stock and patched blobs).

```
algorithm : table-driven CRC-32, MSB-first (NOT reflected)
poly      : 0x04C11DB7           literal @0x8c578b4
init      : 0                    mov r2,#0 @0x8c4c8c0
xorout    : 0                    accumulator returned directly
length    : passed in BITS       lsl r1,r1,#3 @0x8c4c8d8
range     : file 0x0e .. 0x223   = 533 bytes / 4264 bits
stored    : 0x06 = lo u16, 0x08 = hi u16, compared as (hi<<16)|lo
```

Brute-forcing this originally failed only because **the length is in bits** and `init=0` *together
with* `xorout=0` was never tried.

```bash
python3 ~/jidu6411/cdt_crc.py <cdt.bin>              # verify
python3 ~/jidu6411/cdt_crc.py <cdt.bin> --fix out.bin # recompute + write
```

Independent confirmation: QSDK's own `meta-tools-oss/scripts/cdt_generator.py` carries a CRC table
starting `0x00000000, 0x04c11db7` — the same polynomial, arrived at from the other direction.

> **Bypass worth knowing:** if the header version at `0x04` is **< 2**, SBL **skips the CRC check
> entirely** (`blo` @`0x8c4c90c`). Useful as an escape hatch; not needed if you compute the CRC.

---

## 6. Procedure

### Step 0 — establish the safety net (do this first, always)

```bash
dd if=/dev/mtd15 of=/tmp/cdt_good.bin bs=64k
md5sum /dev/mtd15 /tmp/cdt_good.bin      # must match
mtd write /tmp/cdt_good.bin /dev/mtd16   # populate the fallback
cmp /dev/mtd15 /dev/mtd16                # identical
```

Also pull a copy off the box (`nc` to a host `socat`/listener) — a CDT that lives only on the device
is not a backup.

### Step 1 — patch on the host

Prefer a **surgical byte edit of your own blob** over generating a fresh CDT from QSDK XML. See §9
for why. Script:

```python
#!/usr/bin/env python3
# cdt_patch.py <in.bin> <out.bin> <field=value> ...   e.g. device_size_cs0=1024
import sys, struct
sys.path.insert(0, '/home/a/jidu6411')
from cdt_crc import qcrc, set_crc, CRC_START, CRC_END

FIELDS = {                    # name: (profile A offset, profile B offset)
    'num_rows_cs0':    (0x057, 0x13f),
    'num_cols_cs0':    (0x05b, 0x143),
    'device_size_cs0': (0x07b, 0x163),
}
buf = bytearray(open(sys.argv[1], 'rb').read())
assert bytes(buf[0:4]) == b'CDT\x00' and struct.unpack_from('<H', buf, 4)[0] == 2
assert qcrc(buf[CRC_START:CRC_END]) == (struct.unpack_from('<H', buf, 8)[0] << 16 |
                                        struct.unpack_from('<H', buf, 6)[0]), 'input CRC bad'
for arg in sys.argv[3:]:
    name, val = arg.split('='); val = int(val)
    for off in FIELDS[name]:
        print(f"{name} @0x{off:03x}: {struct.unpack_from('<I', buf, off)[0]} -> {val}")
        struct.pack_into('<I', buf, off, val)
set_crc(buf, qcrc(buf[CRC_START:CRC_END]))
open(sys.argv[2], 'wb').write(buf)
print("new CRC 0x%08x" % qcrc(buf[CRC_START:CRC_END]))
```

Then **always** re-verify the output and eyeball the diff:

```bash
python3 cdt_crc.py CDT_new.bin           # must print "OK - match"
cmp -l CDT_stock.bin CDT_new.bin         # expect ONLY your fields + the 4 CRC bytes
```

### Step 2 — transfer and verify

```bash
# host:  python3 -m http.server 8099 --bind <host-ip> --directory .
wget -q http://<host-ip>:8099/CDT_new.bin -O /tmp/cdt_new.bin
md5sum /tmp/cdt_new.bin                  # compare against the host md5
```

### Step 3 — flash

```bash
mtd write /tmp/cdt_new.bin /dev/mtd15    # erases + writes
md5sum /dev/mtd15 /dev/mtd16             # primary = new, fallback = stock
reboot
```

### Step 4 — verify the change took

| Check | Command | 1 GB expectation |
|---|---|---|
| U-Boot | `bdinfo` | `size 0x40000000` |
| U-Boot | `mw.l 0x62000000` | no abort |
| iomem | `cat /proc/iomem \| grep "System RAM"` | tops at `0x7fffffff` |
| DT | `hexdump -C /proc/device-tree/memory*/reg` | size `0x40000000` |
| kernel | `dmesg \| grep "^\[.*Memory:"` | `.../1048576K` |
| total | `grep MemTotal /proc/meminfo` | ~960932 kB |

**The DT `memory` node updates itself** — U-Boot patches it from the SMEM RAM ptable, so no DTS edit
is needed. This held under both the stock 5.4 kernel and mainline OpenWrt 6.18.36 ✅.

### Step 5 — prove the memory is real

Reported size is not proof. Cross-check with an address-uniqueness test:

```bash
STAGING_DIR=~/jidu6x11/openwrt/staging_dir \
  ~/jidu6x11/openwrt/staging_dir/toolchain-aarch64_cortex-a53_gcc-14.3.0_musl/bin/aarch64-openwrt-linux-gcc \
  -O2 -static -o ramtest ramtest.c
./ramtest 640 3        # MB, passes
```

`~/jidu6411/ramtest.c` runs zeros / ones / 0x55AA / 0xAA55 / walking-ones (64 patterns) / xorshift
random / **`addr_own_addr`**.

**`addr_own_addr` is the one that matters for a density change**: each word stores its own address
and reads it back. If the controller is configured for more memory than the chip holds, aliased
words read back a *different* address. Plain pattern tests cannot detect this — two aliased
locations holding the same pattern still "verify". Result on this board: 640 MB × 3 passes,
`mlock`'d, swap off → **0 errors**, no EDAC errors, 40 °C → 42 °C, ~3.7 GB/s ✅.

### Rollback

```bash
mtd write /tmp/CDT_stock.bin /dev/mtd15   # md5 7523b644fa5d785bd857a77f4f8b8ad6 on this board
```

---

## 7. Worked example: 512 MB → 1 GB ✅

Board chip is a Samsung **`K4A8G165WC-BCWE`** = 8 Gb (1 GByte) ×16 DDR4-3200. `-BCWE` = 3200 MT/s =
1600 MHz clock, matching the SBL log's `DDR Frequency, 1600 MHz`. That is exactly QSDK's
`512M16_DDR4`.

`~/jidu6411/CDT_1GB.bin` differs from stock in **exactly 6 bytes** (re-verified with `cmp -l`):

```
0x006  c9 -> 2a  ┐
0x007  e0 -> 43  │ CRC 0x3ca9e0c9 -> 0x79aa432a
0x008  a9 -> aa  │
0x009  3c -> 79  ┘
0x07c  02 -> 04    device_size_cs0 profile A: 512 -> 1024
0x164  02 -> 04    device_size_cs0 profile B: 512 -> 1024
```

md5: stock `7523b644fa5d785bd857a77f4f8b8ad6` → patched `fa18110996ed88e797957ebee6cdca54`.

**Result:**

| | before | after |
|---|---|---|
| `MemTotal` | 441 636 kB | **960 932 kB** |
| kernel `Memory:` | 440612K/456704K | **959908K/1048576K** (exactly 1 GiB) |
| `/proc/iomem` top | `0x5fffffff` | **`0x7fffffff`** |
| DT `memory/reg` size | `0x20000000` | **`0x40000000`** |
| `/tmp` tmpfs | 215 MB | 469 MB |

### The tell that this was safe

Diffing the QSDK `256M16_DDR4` and `512M16_DDR4` reference blobs gives **17 non-CRC delta bytes**.
This board's stock CDT already matched the **1 GB** value at **14 of them** ✅ — `tCCD_L`=8,
`mr0`=3412, `mr2`=40, `wrlat_adj_f0`=12, `rdlat_adj_f0`=20, `mr6`=4096, across both profiles. Every
timing and mode register was already provisioned for an 8 Gb part; **only the capacity declaration
was left at 512**, and one board-specific byte at `0x220` differs from both references.

That is a strong pre-flight signal: if your board's timings already match the target config, the
density edit is a declaration fix rather than a retune. **Check this before flashing anything.**

---

## 8. Next ceiling: 1 GB → 2 GB

| Limit | Value |
|---|---|
| QSDK-validated max for IPQ9574 | 4 GB (`1024M32_DDR4`: ×32, 2 devices, 17 rows) — needs two chips |
| **This board (single ×16 footprint)** | **2 GB** — 16 Gb ×16 is the largest monolithic ×16 DDR4 die |
| Stock firmware | 32-bit `armv7l` userspace/kernel → ~3 GB practical cap regardless |
| OpenWrt port | aarch64, no such limit |

Fit a 16 Gb ×16 part, then edit **two** fields in **both** profiles ✅ (derived by generating a
modified `512M16_DDR4` and diffing — 4 non-CRC bytes):

```
0x057 / 0x13f   num_rows_cs0     16 -> 17     (0x10 -> 0x11)
0x07c / 0x164   device_size_cs0  1024 -> 2048 (0x04 -> 0x08)
+ CRC
```

Or with the script: `cdt_patch.py CDT_1GB.bin CDT_2GB.bin num_rows_cs0=17 device_size_cs0=2048`.

**A16 is not a routing concern** — DDR4 multiplexes A16/A15/A14 with `RAS_n`/`CAS_n`/`WE_n`, so those
lines are wired on every DDR4 board regardless of density. **CS1 is different**: it is a dedicated
pin with no dual purpose, so a *second rank* would need real routing and a second footprint.
At 2 GB, DRAM spans `0x40000000–0xBFFFFFFF` — still under 4 GB, fine for both kernels.

---

## 9. Risk, and what the fallback does *not* cover

**Prefer editing your own blob. Do not flash a CDT generated fresh from QSDK XML.** Comparing this
board's CDT against a generated `512M16_DDR4` reference shows **16 differing non-CRC bytes** outside
the density field (`0x18`, `0x1c`–`0x1d`, `0x93`–`0x95`, `0x110`, `0x123`, `0x17b`–`0x17d`, `0x1f8`,
`0x20b`, `0x220`). Those are board-specific calibration/identity values; a generated blob would
silently overwrite them. Use QSDK XML as a **reference to diff against**, not as a source image.

| Failure | Covered by `0:CDT_1`? |
|---|---|
| Corrupt write / bad CRC | ✅ yes — SBL falls back |
| Truncated or erased primary | ✅ yes |
| **Structurally valid CDT declaring the wrong size** | ❌ **no** |

That last row is the real hazard: a well-formed CDT with a wrong density passes CRC, is accepted
without error, and DDR init then fails **before there is any console**. There is no U-Boot to fall
back to, and **EDL recovery is gated by the blown secure-boot fuse**, leaving NAND chip-off as the
only route back.

**So: confirm the physical part number on the chip before changing density.** Read the marking; do
not infer capacity from what you hope is there.

`sbl1_ddr_set_default_params` does run before the CDT parse, so some default likely exists as a last
resort — but that is inferred from the symbol name and has **not** been tested. Do not rely on it.

### Secure boot

The fuse is blown (`is_sec_boot_enabled` → *"secure boot fuse is enabled"*), and SBL1/QSEE/APPSBL
are MBN+RSA authenticated. **The CDT is a data table and is evidently not authenticated** — the
patched CDT boots. Only the CRC gates it.

---

## 10. Provenance

| Artifact | Path |
|---|---|
| Stock CDT (512 MB) | `~/jidu6411/mtd_bak/CDT_mtd15.bin` |
| Patched CDT (1 GB) | `~/jidu6411/CDT_1GB.bin` |
| CRC tool | `~/jidu6411/cdt_crc.py` |
| RAM validator | `~/jidu6411/ramtest.c`, prebuilt `~/jidu6411/ramtest` |
| SBL1 (for RE) | `~/jidu6411/mtd_bak/SBL1_mtd0.bin` |
| QSDK CDT sources | `~/qsdk13/meta-tools-oss/ipq9574/cdt/*.xml` |
| QSDK CDT generator | `~/qsdk13/meta-tools-oss/scripts/cdt_generator.py` |
| Narrative writeup | `~/jidu6411/docs/RAM_UPGRADE.md` |

### Re-doing the RE

SBL1 partition begins with the Qualcomm boot-header magic `0x844BDCD1`; the ELF is carved at
offset **`0x2800`**, ARM32, code segment file `0x3000` → VA `0x08c39000`. Key addresses:

| What | Address |
|---|---|
| CRC function | `0x8c57878` (poly literal @`0x8c578b4`) |
| CDT validator | `0x8c4c6a4` – `0x8c4c928` |
| CRC call / compare | `0x8c4c8e0` / `0x8c4c8fc` |
| version < 2 skip | `0x8c4c90c` |
| `get_cdt_section` | `0x8c4c42c` |
| `sbl1_ddr_set_params` | string refs near `0x8c408c0`, section call @`0x8c417fc` |
| DDR section consumer | `0x21ab48` (SRAM seg VA `0x218000`), jump table @`0x21ab94` |

Disassemble with capstone ARM and **`skipdata=True`** — it halts at the first literal pool otherwise.

Running the QSDK generator needs two shims on modern Python:
`from inspect import getfullargspec as getargspec`, and do not run it from a directory containing a
`dis.py` (it shadows the stdlib `dis`).

### Corrections to earlier notes

- Profiles are at file **`0x4f` / `0x137`** (not `0x50` / `0x138`).
- The `mov r2,#0xf0` / `add r0,r0,#0x120` path @`0x21abb8` is a **different handler branch** — a
  240-byte stride does not fit (it overruns `0x200`, and the blocks differ in 118 bytes).
- Populated region is `0x000`–`0x222` (545 bytes non-blank), not ~197 bytes.
