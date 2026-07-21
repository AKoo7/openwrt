# SPDX-License-Identifier: GPL-2.0
# Kbuild file for the open Maple GPON MAC driver + EPLD reset controller.

# Main driver: PCIe glue + firmware boot + ring transport + BAL messaging + ONU/NI ops
maple-y := maple_pci.o maple_ring.o maple_fw.o maple_bal.o maple_onu.o maple_ni.o maple_gnl.o
obj-m += maple.o

# EPLD reset controller: releases the Maple MAC from reset before PCIe probe.
# Load BEFORE maple.ko (earlier /etc/modules.d/ number).
maple-epld-y := maple_epld.o
obj-m += maple-epld.o
