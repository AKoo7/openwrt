# SPDX-License-Identifier: GPL-2.0-only
CFLAGS ?= -O2 -Wall -Wextra
# Static: these run on the vendor firmware too, whose libc is not ours.  The
# whole point is that one binary takes the same measurement on both images.
LDFLAGS += -static

PROGS = caregt forge_inject

all: $(PROGS)

%: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -f $(PROGS)

.PHONY: all clean
