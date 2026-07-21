// SPDX-License-Identifier: GPL-2.0
/* maplectl — CLI for the open Maple GPON driver. Uses maple_lib (genetlink). */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "maple_lib.h"

static void usage(void) {
	fprintf(stderr,
	"usage: maplectl <command> [args]\n"
	"  list <pon>                  - list active ONUs on a PON port\n"
	"  info <pon> <onu>            - show ONU details (state/SN/distance/alarms)\n"
	"  bandwidth <pon> <onu>       - show per-ONU up/down bandwidth (1s sample)\n"
	"  block <pon> <onu>           - block (disable) an ONU\n"
	"  unblock <pon> <onu>         - unblock (enable) an ONU\n"
	"  ni-info <pon>               - show PON port info (status/ONU count/distance/FEC)\n"
	"  ni-stat <pon>               - show PON port aggregate counters\n");
}

static int do_ni_info(uint8_t pon) {
	struct maple_ni_cfg c;
	if (maple_gnl_ni_get_cfg(pon, &c)) { perror("ni_get_cfg"); return 1; }
	printf("PON %u\n", pon);
	printf("  Status:             0x%llx\n", (unsigned long long)c.pon_status);
	printf("  Active ONUs:        %u (+ %u standby)\n",
		c.number_of_active_onus, c.number_of_active_standby_onus);
	printf("  Distance:           %llu cm\n", (unsigned long long)c.pon_distance);
	printf("  Ranging window:     %u\n", c.ranging_window_size);
	printf("  DS FEC mode:        %u\n", c.ds_fec_mode);
	printf("  EQD cycles:         %u\n", c.eqd_cycles_number);
	return 0;
}

static int do_ni_stat(uint8_t pon) {
	struct maple_ni_stat s;
	if (maple_gnl_ni_get_stat(pon, &s)) { perror("ni_get_stat"); return 1; }
	printf("PON %u statistics\n", pon);
	printf("  RX GEM:    packets=%llu dropped=%llu idle=%llu corrected=%llu illegal=%llu\n",
		(unsigned long long)s.rx_gem_packets, (unsigned long long)s.rx_gem_dropped,
		(unsigned long long)s.rx_gem_idle, (unsigned long long)s.rx_gem_corrected,
		(unsigned long long)s.rx_gem_illegal);
	printf("  TX GEM:    %llu   TX PLOAMs: %llu   TX OMCI: %llu\n",
		(unsigned long long)s.tx_gem, (unsigned long long)s.tx_ploams,
		(unsigned long long)s.tx_omci);
	printf("  RX OMCI:   %llu (CRC err %llu)   RX CPU: %llu\n",
		(unsigned long long)s.rx_omci, (unsigned long long)s.rx_omci_packets_crc_error,
		(unsigned long long)s.rx_cpu);
	printf("  FEC:       codewords=%llu uncorrected=%llu\n",
		(unsigned long long)s.fec_codewords, (unsigned long long)s.fec_codewords_uncorrected);
	printf("  BIP8:      bytes=%llu errors=%llu\n",
		(unsigned long long)s.bip8_bytes, (unsigned long long)s.bip8_errors);
	printf("  PLOAMs:    rx=%llu (non-idle %llu, err %llu, dropped %llu)\n",
		(unsigned long long)s.rx_ploams, (unsigned long long)s.rx_ploams_non_idle,
		(unsigned long long)s.rx_ploams_error, (unsigned long long)s.rx_ploams_dropped);
	printf("  Drops:     too_short=%llu too_long=%llu crc=%llu key=%llu\n",
		(unsigned long long)s.rx_dropped_too_short, (unsigned long long)s.rx_dropped_too_long,
		(unsigned long long)s.rx_crc_errors, (unsigned long long)s.rx_key_errors);
	return 0;
}

static int do_info(uint8_t pon, uint16_t onu) {
	struct maple_onu_cfg c;
	if (maple_gnl_onu_get_cfg(pon, onu, &c)) { perror("get_cfg"); return 1; }
	printf("PON %u ONU %u\n", pon, onu);
	printf("  State:           %s\n", maple_state_name(c.onu_state));
	printf("  Serial Number:   %c%c%c%c%02x%02x%02x%02x\n",
		c.serial_number[0], c.serial_number[1], c.serial_number[2],
		c.serial_number[3], c.serial_number[4], c.serial_number[5],
		c.serial_number[6], c.serial_number[7]);
	printf("  Distance(m):     %u\n", c.ranging_time);
	printf("  Last down cause: %s\n", maple_deact_reason_name(c.deactivation_reason));
	printf("  Alarms LOSi/LOFi/DGi/SDi/SFi: %u/%u/%u/%u/%u\n",
		c.alarm_state[0], c.alarm_state[1], c.alarm_state[3],
		c.alarm_state[9], c.alarm_state[7]);
	return 0;
}

static int do_bandwidth(uint8_t pon, uint16_t onu) {
	struct maple_onu_stat s1, s2;
	struct timeval tv = { .tv_sec = 1 };
	if (maple_gnl_onu_get_stat(pon, onu, &s1)) { perror("get_stat"); return 1; }
	printf("rx_bytes=%llu tx_bytes=%llu  ",
		(unsigned long long)s1.rx_bytes, (unsigned long long)s1.tx_bytes);
	fflush(stdout);
	select(0, NULL, NULL, NULL, &tv);
	if (maple_gnl_onu_get_stat(pon, onu, &s2)) { perror("get_stat"); return 1; }
	printf("down=%llu B/s  up=%llu B/s\n",
		(unsigned long long)(s2.rx_bytes - s1.rx_bytes),
		(unsigned long long)(s2.tx_bytes - s1.tx_bytes));
	return 0;
}

static int do_list(uint8_t pon) {
	for (uint16_t onu = 0; onu < 128; onu++) {
		struct maple_onu_cfg c;
		if (maple_gnl_onu_get_cfg(pon, onu, &c)) continue;
		if (!c.onu_state) continue;
		printf("ONU %3u  SN %c%c%c%c%02x%02x%02x%02x  %-14s  dist=%u\n",
			onu, c.serial_number[0], c.serial_number[1], c.serial_number[2],
			c.serial_number[3], c.serial_number[4], c.serial_number[5],
			c.serial_number[6], c.serial_number[7],
			maple_state_name(c.onu_state), c.ranging_time);
	}
	return 0;
}

int main(int argc, char **argv) {
	if (argc < 3) { usage(); return 1; }
	if (maple_gnl_open()) { fprintf(stderr, "maple family not found (module loaded?)\n"); return 1; }

	uint8_t pon = atoi(argv[2]);
	uint16_t onu = argc > 3 ? atoi(argv[3]) : 0;

	if (!strcmp(argv[1], "info"))	   return do_info(pon, onu);
	if (!strcmp(argv[1], "bandwidth")) return do_bandwidth(pon, onu);
	if (!strcmp(argv[1], "list"))	   return do_list(pon);
	if (!strcmp(argv[1], "block"))	   return maple_gnl_onu_set_state(pon, onu, MAPLE_OP_DISABLE) ? 1 : (printf("OK\n"),0);
	if (!strcmp(argv[1], "unblock"))   return maple_gnl_onu_set_state(pon, onu, MAPLE_OP_ENABLE)  ? 1 : (printf("OK\n"),0);
	if (!strcmp(argv[1], "ni-info"))   return do_ni_info(pon);
	if (!strcmp(argv[1], "ni-stat"))   return do_ni_stat(pon);
	usage();
	return 1;
}
