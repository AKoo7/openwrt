# RTL9602C (HSGQ X111W) — LAN / WAN / WiFi status & handoff (2026-06-15)

DUT: HSGQ X111W, SN XPON39013867, WAN MAC 98:c7:a4:32:82:b1, on OLT PON1/ONT0.
HARD CONSTRAINT: fixes are ONU-side only — do NOT change the OLT.

## Summary
| Feature | State |
|---|---|
| **LAN** (eth0/br-lan, 192.168.1.1) | ✅ reliable every boot — http/ssh/ping |
| **WAN** (gpon0 DHCP) | ◐ works **end-to-end on ~50% of boots** (leases 192.168.158.x + pings 8.8.8.8). Fully traced; remaining 50% is a deep DUT-side cold-start marginality (below). |
| **WiFi** (rtl8192fe, ONU-3282AE) | ✗ dark — AP comes up (hostapd AP-ENABLED, ch7, MAC ae) but beacon does not radiate (0/scan). Deep RF; open. |

## Verify loops (no guesswork — both verifiable here)
- **WAN**: `/tmp/bootdut.py` (RAM-boot), wait O5, `udhcpc -i gpon0`, check `ifconfig gpon0` + `ping 8.8.8.8`.
- **WiFi**: host `iw dev wlan0 scan | grep -i ONU-3282AE`. Sanity: the adjacent STOCK twin **HS-3282A2** (oracle) is ALWAYS visible — proves the scanner + RF env. `iw list` is NOT a scan (capabilities only).
- Rig: ttyUSB0=power, ttyUSB1=DUT console, ttyUSB2=OLT console, ttyUSB3=oracle (stock). Helpers in /tmp: con.py, bootdut.py, rebuild_gpon.sh, oltcli.py (OLT CLI root/admin), orsh.py (oracle shell).

## WAN — fully traced (datapath CORRECT; ~50% = cold-start handoff)
Proven via the OLT (creds **root/admin**, `/tmp/oltcli.py`): on a good boot the OLT shows Run State **Online** (stable 23 min), `show service-port ont 1 0` **UP**, `show mac-address ont 1 0` **learned WAN MAC 82:b1** on GEM 1, `show gpon statistic 1` **valid allocations, ~1e-8 BER** (US burst clean), and `show gemport statistic 1 0 1` shows the OLT **transmitting DS** to the ONU. So OLT + US burst + data-plane are all good.
Remaining 50% (DUT-side, cold-start): on a bad boot the DS **de-encaps + drains** (`/proc/gpon` `D_rxok` climbs to match the OLT's DS count) **but never reaches the eth driver** — `gpon0 RX=0`, no `DHCP-DS` log fires → the OFFER never reaches gpon0 → no lease. i.e. the **PON-IP-NIC-drain → GMAC-RX handoff is intermittent at cold start** (same DS path that works on the leasing boots). Also a transient OLT `Deactivate(0x05)` on some boots (its reason is NOT in the OLT CLI; would need the OLT root-shell = rebooting the OLT, too disruptive).
Fixes applied (gpon-rtl9602c.c, all confirmed in-tree, A/B-revertible params):
- flow-1 US classify GMII-edge commit in gpon_install_data_gem (`relatch_us`).
- `cdr_reseat_on_reactivate=1` (default ON) — corrected CDR reset addr 0x225a0 (was poking 0x22560/RX).
- US-PLOAM flush (`PLM_FLUSH_BUF` 0→1) at Ranging→O5; O5-entry burst-gate re-apply + keepalive (`o5_rearm_burst_gate`/`o5_keepalive`).
- `full_serdes_reinit=0` (default OFF) — **do NOT enable**: re-running the full gpon_serdes_init on re-range BREAKS the GPON (races FSM re-acquisition).
5 fixes did NOT move the ~50%. NEXT lever (open): the DS NIC-drain→GMAC-RX delivery (DMA/ring handoff) — why drained DS doesn't reach rtl9602c_eth_rx on bad boots. (DS-OFFER diag log is in rtl9602c_eth.c: prints `DHCP-DS sp/opts3/dst -> gpon0|host`.)

## WiFi — open (beacon configured but doesn't radiate)
Fixes applied (rtl8192fe/, all in-tree): factory cal injection (hw.c `_rtl92fe_apply_board_cal`: MAC ae, per-channel CCK/HT40 power, xtal, thermal, rfe_type=3); RF radio table (table.c) corrected to the **rfe==3** vendor branch (was the rfe!=3 branch); path-B RF 0x18 full-word re-key (phy.c — was leaving RF_B[0x18]=0); TRX-mode init. Live: RF_A on + tuned ch7, TX-AGC=0x2e, XTAL matches stock — yet 0/scan. The beacon is 1-stream on path A (tuned) so path-B isn't the cause. NEXT lever (open): add an RF-register dump to the driver (rtl_get_rfreg/bbreg via a /proc or debugfs) to diff path-A's full RF/BB state vs the working stock chip (oracle iwpriv `read_rf '<pathDec> <regDec>'`), and check IQK/PA-LDO/TX power-on. Needs that instrumentation or a spectrum analyzer to converge.

## ★UPDATE 2026-06-15 (parallel WAN+WiFi round)
- **WiFi: ✅ FIXED — ONU-3282AE now radiates (7/7 scans across 4 boots).** Root cause: the clean-room power-on sequence `RTL8192F_TRANS_CARDEMU_TO_ACT` (rtl8192fe/pwrseq.h) was TRUNCATED (18 of the vendor's 30 steps) — it enabled RF path S1 (MAC 0x1F=0x07) but never released RF path S0 (MAC 0x7B=0x00 then 0x07) and never powered the analog TX front-end (AFE 0x97[5]=1, 0xDC=0xC4). Path S0 + the analog block stayed in reset → digital configured but on-air TX dark (and that is why RF_B[0x18]=0 in all the earlier RE). Fix = the 4 missing pwrseq entries + array-size bump + a phy.c 0x7B mirror. Confirmed live via the new `dump_rf` instrument (rtl8192fe/dm.c, `echo 1 > /sys/module/rtl8192fe/parameters/dump_rf`): post-fix **MAC 7b=07** + **RF_B alive (00/18=0x31e16)** + RF_A tuned ch7. WiFi is done.
- **WAN: improved but still ~25-50%.** The missing DS-NIC FORCELINK was added (`PI_MEDIA_STS_DS=0x106e8400`) and CONFIRMED applied — `/proc/gpon ds_fwd` now reads `media_sts=0x106e8400 (bit18 link=1) gmii_en=1`. But on bad boots gpon0 RX still=0, so the FORCELINK wasn't the (only) gate. Checked + likely-not-it: rxfdp_ds=0 (the US twin rxfdp_us=0 too and US works = probably normal); the switch-side GMAC0<->NIC force (gpon-rtl9602c.c:2430-2437) is US-only but its comment claims the DS direction's link is already up. The DS NIC-drain->GMAC-RX delivery on bad boots remains the open gate (deep, multi-mechanism). New diag in place: `/proc/gpon ds_fwd` (DS link bit18) + the DHCP-DS log in rtl9602c_eth.c. NEXT lever to try: a GMAC0-side DS MII force mirroring the US 2430-2437 for the DS direction; and capture ds_fwd/D_rxok/rxfdp on a bad boot to see which sub-block rejects.

See the per-area memory notes for full RE detail.
