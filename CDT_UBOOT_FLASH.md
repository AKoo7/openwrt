# Flashing the CDT from U-Boot

**Companion to [`CDT_GUIDE.md`](CDT_GUIDE.md).** That guide covers the format, the CRC, the field
offsets and the Linux `mtd write` route. This one covers the case where **Linux cannot write the
partition at all** and you have to do it from U-Boot instead.

Written against the JIDU6811 unit (`~/jidu6411/6811/`), IPQ9574, running OpenWrt, 2026-08-10.

---

## 1. When you need this

On a board running our OpenWrt build, `0:CDT` may be exposed read-only, so every Linux-side attempt
fails at `open()`:

```
root@OpenWrt:~# mtd write /tmp/cdt_1gb.bin /dev/mtd16
Could not open mtd device: /dev/mtd16
Can't open device for writing!
```

Check which of the two causes you have:

```sh
cat /sys/class/mtd/mtd16/flags
```

| flags | meaning |
|---|---|
| `0xc00` | `MTD_WRITEABLE` set — writable; the failure is something else |
| `0x800` | `MTD_WRITEABLE` cleared — **read-only partition**, use U-Boot |

A `read-only;` property in the DTS partition node clears that bit. Either rebuild the image without
it, or take the U-Boot route below — the U-Boot route needs no rebuild and no reflash.

---

## 2. Two traps that will bite before you get there

### The colon trap (Linux only)

Partition names contain a colon. Linux's `mtd` tool splits on it and tries to open a device called
`0`:

```
root@OpenWrt:~# mtd write /tmp/cdt_1gb.bin 0:CDT
Could not open mtd device: 0
```

**Under Linux, always address `/dev/mtdN`.** In U-Boot the opposite is true — `flash 0:CDT` works
fine, because U-Boot looks the name up in the SMEM partition table verbatim.

### The index trap — *mtd numbering is not portable between boards* ★

The CDT pair is ordered differently on different units:

| board | `0:CDT` (primary) | `0:CDT_1` (fallback) |
|---|---|---|
| JIDU6411 / 6111 (stock fw) | **mtd15** | mtd16 |
| JIDU6811 (OpenWrt) | **mtd16** | mtd15 |

They are **swapped**. Several other pairs are reversed on the 6811 too (`DEVCFG_1` before `DEVCFG`,
`RPM_1` before `RPM`, `APPSBL_1` before `APPSBL`) — the SMEM ptable order simply differs, and mtd
indices follow it.

**Never carry an index over from another board or another doc.** Read `/proc/mtd` (Linux) or
`smeminfo` (U-Boot) on the unit in front of you, and then confirm by content:

```sh
md5sum /dev/mtd<CDT>     # a live 512 MB CDT on this platform = 7523b644fa5d785bd857a77f4f8b8ad6
```

If that hash matches, the partition is positively identified. mtd0 is `0:SBL1` — a mis-aimed write
in this region is unrecoverable, see §7.

### JIDU6811 partition map (ground truth, 2026-08-10)

27 entries, `0x20000` erasesize throughout. Differs from the 6411's 29-entry map (this one has
`0:TME`/`0:ETHPHYFW`, and a single `ubi` instead of `rootfs`/`rootfs_1`/`customfs`):

```
mtd0  0:SBL1        mtd7  0:DEVCFG_1   mtd14 0:RPM        mtd21 0:ETHPHYFW
mtd1  0:SBL1_1      mtd8  0:DEVCFG     mtd15 0:CDT_1  ★   mtd22 0:TRAINING
mtd2  0:MIBIB       mtd9  0:APDP_1     mtd16 0:CDT    ★   mtd23 ubi  (0xe100000)
mtd3  0:BOOTCONFIG  mtd10 0:APDP       mtd17 0:APPSBLENV  mtd24 Jio-Reserved
mtd4  0:BOOTCONFIG1 mtd11 0:TME        mtd18 0:APPSBL_1   mtd25 MFG
mtd5  0:QSEE        mtd12 0:TME_1      mtd19 0:APPSBL     mtd26 0:BDF
mtd6  0:QSEE_1      mtd13 0:RPM_1      mtd20 0:ART
```

Both CDT partitions are `0x80000` (512 KiB), and the blobs are exactly that size — so this is a
whole-partition erase-and-write, not a partial one.

---

## 3. Blobs and checksums

| file | size | md5 | crc32 | goes to |
|---|---|---|---|---|
| `~/jidu6411/6811/cdt_good.bin` | 524288 | `7523b644fa5d785bd857a77f4f8b8ad6` | `86441502` | `0:CDT_1` (fallback) |
| `~/jidu6411/6811/cdt_1gb.bin` | 524288 | `fa18110996ed88e797957ebee6cdca54` | `89e0295b` | `0:CDT` (primary) |

`crc32` is listed because U-Boot's `md5sum` is not compiled into every build; `crc32` always is.

The 6811's stock CDT is **byte-identical** to the 6411's, so the same 6-byte patch applies
(`device_size_cs0` 512→1024 at `0x07c`/`0x164`, plus the 4 CRC bytes). Regenerate at any time:

```sh
python3 ~/jidu6411/6811/cdt_patch.py cdt_good.bin cdt_1gb.bin device_size_cs0=1024
```

---

## 4. Host side

```sh
cp ~/jidu6411/6811/cdt_good.bin ~/jidu6411/6811/cdt_1gb.bin /srv/tftp/
sudo dnsmasq --port=0 --enable-tftp --tftp-root=/srv/tftp \
     --interface=<iface> --bind-interfaces --user=root
sudo ip addr add 192.168.10.1/24 dev <iface>
```

U-Boot may be password-gated (on the 6411: user = last 8 chars of the serial, password from `MFG`
at offset `0x75`). Both are readable from Linux beforehand: `dd if=/dev/mtd25 bs=1 count=256 | xxd`.

---

## 5. Procedure

### Step 0 — recon, before writing anything

```
smeminfo                      # note the byte offset + size of 0:CDT and 0:CDT_1
flash                         # bare, to print this build's usage string
printenv ipaddr serverip
```

Confirm `flash` is `flash <part_name> [addr size]` on this build before relying on it.

### Step 1 — network, then back up both partitions off-box

```
setenv serverip 192.168.10.1
setenv ipaddr 192.168.10.10
ping $serverip

nand read 0x50000000 <cdt_off> 0x80000
tftpput 0x50000000 0x80000 192.168.10.1:cdt_6811_primary.bin
nand read 0x50000000 <cdt1_off> 0x80000
tftpput 0x50000000 0x80000 192.168.10.1:cdt_6811_fallback.bin
```

On the host, `cdt_6811_primary.bin` must hash to `7523b644…`. **That is the offset check** — it is
the U-Boot equivalent of the `md5sum /dev/mtdN` identification in §2. Do not continue without it.

### Step 2 — fallback first, with the STOCK blob

```
tftpboot 0x50000000 cdt_good.bin
crc32 0x50000000 0x80000               -> 86441502
flash 0:CDT_1
nand read 0x52000000 <cdt1_off> 0x80000
cmp.b 0x50000000 0x52000000 0x80000
```

`cmp.b` must report the ranges identical. **Do not proceed until the fallback holds a valid stock
CDT** — on the 6411 it shipped blank (all `0xFF`), i.e. no factory fallback at all.

> The fallback gets the **stock** blob, never the patched one. If both partitions hold the 1 GB CDT
> there is no recovery path from a wrong density.

### Step 3 — primary, with the 1 GB blob

```
tftpboot 0x50000000 cdt_1gb.bin
crc32 0x50000000 0x80000               -> 89e0295b
flash 0:CDT
nand read 0x52000000 <cdt_off> 0x80000
cmp.b 0x50000000 0x52000000 0x80000
reset
```

RAM addresses: DRAM starts at `0x40000000`; with 512 MB still declared, everything up to
`0x5fffffff` is valid, so `0x50000000` and `0x52000000` are both safe and non-overlapping
(`0x80000` apart is not enough — they are `0x2000000` apart here).

### Step 4 — verify in U-Boot, before booting Linux

```
bdinfo                  -> DRAM size 0x40000000
mw.l 0x62000000 0       -> must NOT data-abort
md.l 0x62000000 1
```

**The upper-half write is the real proof.** `bdinfo` only reports what SBL published; a successful
write at `0x62000000` proves the controller actually decodes the second 512 MB. Under 512 MB that
same command data-aborts and resets the CPU.

Then boot Linux and confirm as per `CDT_GUIDE.md` §4: `MemTotal` ≈ 960932 kB, `/proc/iomem` System
RAM topping at `0x7fffffff`, DT `memory/reg` size `0x40000000` — the DT patches itself from SMEM.
Finish with `ramtest 640 3` (`addr_own_addr` is the test that catches aliasing).

---

## 6. If `flash` is absent or refuses

Raw route, using the offset from `smeminfo`:

```
nand erase <off> 0x80000
nand write 0x50000000 <off> 0x80000
```

⚠ `nand erase` takes a **raw offset**, with no name check and no confirmation. A typo lands on
`0:APPSBLENV` or `0:APPSBL` and you lose U-Boot. Take the offset from `smeminfo` output, never
derive it from an mtd index.

## 7. Rollback and risk

```
tftpboot 0x50000000 cdt_good.bin
flash 0:CDT
```

| failure | recoverable? |
|---|---|
| Corrupt write / bad CRC on primary | ✅ SBL falls back to `0:CDT_1` |
| Truncated or erased primary | ✅ same |
| Mis-aimed write over `0:APPSBL` / `0:SBL1` | ❌ no console, no U-Boot |
| **Valid CDT declaring a density the chip lacks** | ❌ **DDR init fails before any console** |

The last row is why this is gated on reading the physical part marking, not on inference. The board
must carry an 8 Gb ×16 DDR4 part (`K4A8G165WC-BCWE` or equivalent) before `device_size_cs0` goes to
1024. **EDL recovery is blocked by the blown secure-boot fuse**, so the only route back from a dead
DDR config is NAND chip-off.

Secure boot does not otherwise interfere: the CDT is a data table and is not signature-authenticated
— only the CRC gates it (`CDT_GUIDE.md` §9).
