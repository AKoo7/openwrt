// SPDX-License-Identifier: GPL-2.0-only
/*
 * caregt - Cortina Access register read / write / poll-trace via /dev/mem.
 *
 * One static aarch64 binary that runs IDENTICALLY on the RTL9607F "Elnath"
 * stock firmware AND on our clean-room OpenWrt image, so the same tool reads
 * the same registers on both for a clean stock-vs-ours differential.  Stock
 * has busybox `devmem` (read/write only); our image does not ship the devmem
 * CLI - and neither can catch a *dynamic* register sequence.  This adds the
 * missing capability: a fast poll-trace (`watch`) that samples a set of regs
 * and reports which one moved, when, and in what order - the process capture
 * a static snapshot is blind to (CLAUDE.md "config matches stock but behaves
 * wrong -> capture the PROCESS not the snapshot").
 *
 * Physical map fact (tier-1, stock devmem cross-check): the CA8277C NE
 * register file is at phys 0x4f430_0000 (SDK 0xf43xxxxx + 0x4_0000_0000);
 * AXI-REO at 0x4f432d000/0x4f432f000.  Pass FULL physical addresses.
 *
 * Modes:
 *   caregt r  <phys> [count]                 read count (def 1) 32-bit words
 *   caregt w  <phys> <val>                   write one 32-bit word
 *   caregt ind <acc> <accval> <data> [n] [stride]
 *                                            indirect table: write accval to
 *                                            acc, then read n (def 1) words at
 *                                            data, data+stride,... (stride def 4,
 *                                            may be negative e.g. -4 for MC_FIB)
 *   caregt snap <phys> [<phys>...]           one-shot: "phys=val" per reg
 *   caregt watch <secs> <us> <phys>...       poll-trace: sample every <us> us
 *                                            for <secs>s; print a row only when
 *                                            any watched reg CHANGES (--all env
 *                                            CAREGT_ALL=1 prints every sample).
 *
 * Build: aarch64-openwrt-linux-musl-gcc -O2 -static -o caregt caregt.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>

#define PAGE 0x1000UL
#define MAXW 64			/* max regs in snap/watch */

static int g_fd = -1;

/* Map the page containing phys and return a volatile pointer to the word.
 * Small mapping cache so watch mode maps each distinct page once. */
struct pagemap { uint64_t base; volatile uint8_t *va; };
static struct pagemap g_cache[MAXW * 2];
static int g_ncache;

static volatile uint32_t *reg_ptr(uint64_t phys)
{
	uint64_t base = phys & ~(PAGE - 1);
	uint64_t off = phys & (PAGE - 1);
	int i;

	for (i = 0; i < g_ncache; i++)
		if (g_cache[i].base == base)
			return (volatile uint32_t *)(g_cache[i].va + off);

	if (g_ncache >= (int)(sizeof(g_cache) / sizeof(g_cache[0]))) {
		fprintf(stderr, "caregt: page cache full\n");
		exit(1);
	}
	void *va = mmap(NULL, PAGE, PROT_READ | PROT_WRITE, MAP_SHARED,
			g_fd, (off_t)base);
	if (va == MAP_FAILED) {
		fprintf(stderr, "caregt: mmap 0x%llx: %s\n",
			(unsigned long long)base, strerror(errno));
		exit(1);
	}
	g_cache[g_ncache].base = base;
	g_cache[g_ncache].va = (volatile uint8_t *)va;
	g_ncache++;
	return (volatile uint32_t *)((volatile uint8_t *)va + off);
}

static uint32_t reg_rd(uint64_t phys) { return *reg_ptr(phys); }
static void reg_wr(uint64_t phys, uint32_t v) { *reg_ptr(phys) = v; }

static uint64_t xstr(const char *s) { return strtoull(s, NULL, 0); }

static int64_t now_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static int usage(void)
{
	fprintf(stderr,
	 "caregt r <phys> [count] | w <phys> <val> |\n"
	 "       ind <acc> <accval> <data> [n] [stride] |\n"
	 "       snap <phys>... | watch <secs> <us> <phys>...\n");
	return 2;
}

int main(int argc, char **argv)
{
	if (argc < 3)
		return usage();

	g_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (g_fd < 0) {
		fprintf(stderr, "caregt: open /dev/mem: %s\n", strerror(errno));
		return 1;
	}

	const char *cmd = argv[1];

	if (!strcmp(cmd, "r")) {
		uint64_t p = xstr(argv[2]);
		int n = argc > 3 ? (int)xstr(argv[3]) : 1;
		for (int i = 0; i < n; i++)
			printf("0x%09llx = 0x%08x\n",
			       (unsigned long long)(p + 4 * i), reg_rd(p + 4 * i));
		return 0;
	}

	if (!strcmp(cmd, "w")) {
		if (argc < 4) return usage();
		uint64_t p = xstr(argv[2]);
		uint32_t v = (uint32_t)xstr(argv[3]);
		reg_wr(p, v);
		printf("0x%09llx <= 0x%08x (readback 0x%08x)\n",
		       (unsigned long long)p, v, reg_rd(p));
		return 0;
	}

	if (!strcmp(cmd, "ind")) {
		if (argc < 5) return usage();
		uint64_t acc = xstr(argv[2]);
		uint32_t accv = (uint32_t)xstr(argv[3]);
		uint64_t data = xstr(argv[4]);
		int n = argc > 5 ? (int)xstr(argv[5]) : 1;
		int stride = argc > 6 ? (int)strtol(argv[6], NULL, 0) : 4;
		/* optional 7th arg: poll_mask - after writing accv, wait until
		 * (reg[acc] & poll_mask) == 0 (a self-clearing GO/BUSY bit, e.g.
		 * the NI RX-MIB access GO 0x80000000) before reading data. */
		uint32_t poll_mask = argc > 7 ? (uint32_t)xstr(argv[7]) : 0;
		reg_wr(acc, accv);
		if (poll_mask) {
			int t;
			for (t = 0; t < 200000; t++)
				if (!(reg_rd(acc) & poll_mask))
					break;
		}
		for (int i = 0; i < n; i++) {
			uint64_t d = data + (int64_t)stride * i;
			printf("0x%09llx = 0x%08x\n",
			       (unsigned long long)d, reg_rd(d));
		}
		return 0;
	}

	if (!strcmp(cmd, "snap")) {
		for (int i = 2; i < argc; i++) {
			uint64_t p = xstr(argv[i]);
			printf("0x%09llx=0x%08x\n",
			       (unsigned long long)p, reg_rd(p));
		}
		return 0;
	}

	if (!strcmp(cmd, "watch")) {
		if (argc < 5) return usage();
		int secs = (int)xstr(argv[2]);
		int us = (int)xstr(argv[3]);
		uint64_t ph[MAXW];
		uint32_t prev[MAXW], cur[MAXW];
		int nw = 0;
		for (int i = 4; i < argc && nw < MAXW; i++)
			ph[nw++] = xstr(argv[i]);
		int all = getenv("CAREGT_ALL") && atoi(getenv("CAREGT_ALL"));

		/* header */
		printf("# t_us");
		for (int i = 0; i < nw; i++)
			printf(",0x%llx", (unsigned long long)ph[i]);
		printf("\n");

		int64_t t0 = now_us(), tend = t0 + (int64_t)secs * 1000000;
		long lines = 0, samples = 0;
		for (int i = 0; i < nw; i++) prev[i] = reg_rd(ph[i]) + 1; /* force first print */
		while (now_us() < tend) {
			int changed = 0;
			for (int i = 0; i < nw; i++) {
				cur[i] = reg_rd(ph[i]);
				if (cur[i] != prev[i]) changed = 1;
			}
			samples++;
			if (changed || all) {
				printf("%lld", (long long)(now_us() - t0));
				for (int i = 0; i < nw; i++)
					printf(",0x%08x", cur[i]);
				printf("\n");
				memcpy(prev, cur, sizeof(uint32_t) * nw);
				if (++lines >= 200000) { /* runaway cap */
					printf("# line cap hit\n");
					break;
				}
			}
			if (us > 0) {
				struct timespec ts = { us / 1000000,
						       (long)(us % 1000000) * 1000 };
				nanosleep(&ts, NULL);
			}
		}
		/* final summary: first/last per reg */
		printf("# samples=%ld changed_rows=%ld\n", samples, lines);
		for (int i = 0; i < nw; i++)
			printf("# 0x%llx last=0x%08x\n",
			       (unsigned long long)ph[i], reg_rd(ph[i]));
		return 0;
	}

	return usage();
}
