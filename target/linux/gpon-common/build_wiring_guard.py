#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""build_wiring_guard.py -- offline proof that the SHARED GPON files- tree is
actually wired into BOTH OpenWrt targets, and wired the only way that works.

WHAT THIS IS
  A no-device, no-build guard over the build wiring of the common GPON layer:
  target/linux/gpon-common/files-<ver>/drivers/net/gpon/.  It reads Makefiles
  and Kconfigs and asserts the handful of invariants whose violation is SILENT
  -- the ones a green build would not reveal.

WHY IT IS COMMON -- and which targets it covers
  Operator, 2026-08-05: "la idea es poner en comun el codigo que corresponde
  para no tener mucho duplicado", and on the two per-target monoliths carrying
  a private protocol copy each: "mal, poner en comun".  The shared tree is
  compiled by realtek-luna (MIPS32, big-endian) and realtek-elnath (aarch64,
  little-endian) from ONE authored copy; this guard is what proves the "one
  copy, two targets" claim instead of asserting it.

THE CORE/SHELL RULE IT OBEYS
  It is a build-wiring guard, not a driver: it touches no device, opens no
  register and never runs on the board.  Nothing it checks may ever require
  hardware, and no file it guards may ever gain an MMIO access -- that second
  half is proven separately by gpon_core_purity_test in dev/rtl9607c-test/.

  Run it with --self-check to see every assertion FAIL on a mutated scratch
  copy.  A guard whose removal leaves the gate green is not a guard.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

VER = "6.18"
SHARED = "target/linux/gpon-common"
SHARED_FILES = SHARED + "/files-" + VER
GPON_DIR = SHARED_FILES + "/drivers/net/gpon"
TARGETS = ("realtek-luna", "realtek-elnath")

# The one line that makes the shared tree reachable.  `+=` and nothing else:
# include/kernel.mk:44 sets FILES_DIR with `?=`.
FILES_DIR_RE = re.compile(
    r'^FILES_DIR\s*\+=\s*"\$\(TOPDIR\)/target/linux/gpon-common/files-\$\(KERNEL_PATCHVER\)"\s*$')


class Guard:
    def __init__(self, root):
        self.root = root
        self.fails = []
        self.checks = 0

    def p(self, *a):
        return os.path.join(self.root, *a)

    def check(self, ok, name, detail):
        self.checks += 1
        if not ok:
            self.fails.append("%-28s %s" % (name, detail))
        return ok

    def read(self, rel):
        try:
            with open(self.p(rel), "r", encoding="utf-8", errors="replace") as f:
                return f.read()
        except OSError:
            return None

    # -- W1 ---------------------------------------------------------------
    def w1_files_dir_line(self):
        """Each target appends the shared tree, AFTER the target.mk include.

        Placed before the include, kernel.mk's `?=` never fires and the
        target's OWN files-<ver> disappears -- the whole vendor driver with it.
        """
        for t in TARGETS:
            rel = "target/linux/%s/Makefile" % t
            txt = self.read(rel)
            if not self.check(txt is not None, "W1.exists", rel + " missing"):
                continue
            lines = txt.splitlines()
            idx = [i for i, l in enumerate(lines) if FILES_DIR_RE.match(l)]
            if not self.check(len(idx) == 1, "W1.append/" + t,
                              "expected exactly 1 `FILES_DIR +=` shared-tree line, got %d"
                              % len(idx)):
                continue
            inc = [i for i, l in enumerate(lines)
                   if l.strip() == "include $(INCLUDE_DIR)/target.mk"]
            bt = [i for i, l in enumerate(lines)
                  if "call BuildTarget" in l]
            self.check(len(inc) == 1 and len(bt) == 1, "W1.anchors/" + t,
                       "need one target.mk include and one BuildTarget")
            if len(inc) == 1 and len(bt) == 1:
                self.check(inc[0] < idx[0] < bt[0], "W1.order/" + t,
                           "FILES_DIR line at %d must sit between include (%d) and "
                           "BuildTarget (%d)" % (idx[0] + 1, inc[0] + 1, bt[0] + 1))
            self.check(not re.search(r'^FILES_DIR\s*:?=[^+]', txt, re.M),
                       "W1.noclobber/" + t,
                       "a plain FILES_DIR assignment would defeat kernel.mk's `?=`")

    # -- W2 ---------------------------------------------------------------
    def w2_trees_disjoint(self):
        """No path may exist in both the shared tree and a target's tree.

        A collision is resolved SILENTLY by $(CP) order (rules.mk:314
        CP:=cp -fpR, later wins), so one target would build a different file
        from the other with nothing to see.
        """
        def rels(base):
            out = set()
            b = self.p(base)
            for dirpath, dirnames, filenames in os.walk(b, followlinks=False):
                for fn in filenames:
                    out.add(os.path.relpath(os.path.join(dirpath, fn), b))
            return out

        shared = rels(SHARED_FILES)
        self.check(bool(shared), "W2.nonempty", "shared tree has no files")
        for t in TARGETS:
            clash = sorted(shared & rels("target/linux/%s/files-%s" % (t, VER)))
            self.check(not clash, "W2.disjoint/" + t,
                       "path in BOTH trees (later $(CP) silently wins): " + ", ".join(clash))

    # -- W3 ---------------------------------------------------------------
    def w3_o2(self):
        """The shared directory must carry its own -O2.

        realtek-luna forces -Os kernel-wide (patches-6.18/100-cc-optimize-for-
        size.patch) and exempts its datapath objects one by one.  Code moved
        out of gpon-rtl9602c.o into drivers/net/gpon/ silently drops to -Os,
        which shows up only as a throughput number -- on a driver at parity.
        """
        mk = self.read(GPON_DIR + "/Makefile")
        if not self.check(mk is not None, "W3.exists", GPON_DIR + "/Makefile missing"):
            return
        body = "\n".join(l for l in mk.splitlines() if not l.lstrip().startswith("#"))
        self.check(re.search(r'^\s*ccflags-y\s*\+=.*\B-O2\b', body, re.M) is not None,
                   "W3.o2", "no `ccflags-y += -O2`: luna would build this "
                            "directory at -Os")

    # -- W4 ---------------------------------------------------------------
    def w4_objects_accounted(self):
        """Every .c must be BUILT or explicitly PENDING-with-a-reason.

        Three states, never two.  Declared-with-no-source fails the build
        loudly, which is fine.  The dangerous direction is the other one: a .c
        that no line names is dead source which silently never compiles, while
        every test of it passes.  A staged carve legitimately has sources in
        the tree before their consumer moves -- so that state gets a NAME and a
        REASON (`# gpon-pending: <obj> -- why`) instead of being indistinguish-
        able from an oversight.
        """
        mk = self.read(GPON_DIR + "/Makefile")
        if mk is None:
            return
        body = "\n".join(l for l in mk.splitlines() if not l.lstrip().startswith("#"))
        built = set()
        for m in re.finditer(r'^\s*(?:obj-\S+|\S+-\S+)\s*\+?=\s*(.+)$', body, re.M):
            for tok in m.group(1).split():
                if tok.endswith(".o"):
                    built.add(tok[:-2])
        pending = {}
        for m in re.finditer(r'^#\s*gpon-pending:\s*(\S+)\.o\s*--\s*(.+)$', mk, re.M):
            pending[m.group(1)] = m.group(2).strip()
        d = self.p(GPON_DIR)
        present = {f[:-2] for f in os.listdir(d) if f.endswith(".c")} if os.path.isdir(d) else set()

        self.check(not (built - present), "W4.declared_no_src",
                   "declared but no .c: " + ", ".join(sorted(built - present)))
        self.check(not (set(pending) - present), "W4.pending_no_src",
                   "pending but no .c: " + ", ".join(sorted(set(pending) - present)))
        self.check(not (built & set(pending)), "W4.both",
                   "both built and pending: " + ", ".join(sorted(built & set(pending))))
        unaccounted = present - built - set(pending)
        self.check(not unaccounted, "W4.src_never_built",
                   "source present, neither built nor `# gpon-pending:` (silently "
                   "dead): " + ", ".join(sorted(unaccounted)))
        thin = sorted(k for k, v in pending.items() if len(v) < 20)
        self.check(not thin, "W4.pending_reason",
                   "pending with no real reason given: " + ", ".join(thin))

    # -- W5 ---------------------------------------------------------------
    def w5_no_patch_collides(self):
        """No patch may target a file one of our overlays provides.

        files- overlays are copied in BEFORE patches (include/quilt.mk:93-107),
        so a patch against an overlaid file applies to OUR copy.  This is the
        B-BUILD-1 collision: patches-6.18/103-realtek-luna-gpon.patch carries
        the very Kconfig/Makefile now overlaid, and must be deleted.
        """
        provided = set()
        for base in [SHARED_FILES] + ["target/linux/%s/files-%s" % (t, VER) for t in TARGETS]:
            b = self.p(base)
            for dirpath, _, filenames in os.walk(b, followlinks=False):
                for fn in filenames:
                    provided.add(os.path.relpath(os.path.join(dirpath, fn), b))
        hunk = re.compile(r'^(?:\+\+\+|---)\s+[ab]/(\S+)', re.M)
        for t in TARGETS:
            pdir = self.p("target/linux/%s/patches-%s" % (t, VER))
            if not os.path.isdir(pdir):
                continue
            for fn in sorted(os.listdir(pdir)):
                if not fn.endswith(".patch"):
                    continue
                with open(os.path.join(pdir, fn), "r", encoding="utf-8",
                          errors="replace") as f:
                    hit = sorted({m for m in hunk.findall(f.read()) if m in provided})
                self.check(not hit, "W5.collide/" + t,
                           "%s patches a file an overlay provides: %s -- delete the "
                           "patch (the overlay is the sanctioned mechanism)"
                           % (fn, ", ".join(hit)))

    # -- W6 ---------------------------------------------------------------
    def w6_no_scanned_makefile(self):
        """No Makefile in the shared tree's top 3 levels.

        include/scan.mk:85 greps `find -L target/linux -maxdepth 3 -name Makefile`
        for `call BuildTarget`.  gpon-common is a files- tree, not a target, and
        must stay invisible to target enumeration.
        """
        for depth, sub in ((1, SHARED), (2, SHARED_FILES),
                           (3, SHARED_FILES + "/drivers")):
            p = self.p(sub, "Makefile")
            self.check(not os.path.exists(p), "W6.depth%d" % depth,
                       "%s is at scan.mk depth<=3" % os.path.join(sub, "Makefile"))

    # -- W7 ---------------------------------------------------------------
    def w7_no_symlinks(self):
        """No symlink anywhere in the shared tree.

        scripts/timestamp.pl:21 does `next if -l $file`, so an edit behind a
        symlink NEVER re-triggers the prepare step: the kernel builds a stale
        core, silently.  rules.mk:314 `cp -fpR` also copies the link itself,
        whose relative path cannot resolve inside build_dir.  And the toplevel
        scan uses `find -L`, which would follow it off the tree.
        """
        b = self.p(SHARED)
        bad = []
        for dirpath, dirnames, filenames in os.walk(b, followlinks=False):
            for n in list(dirnames) + list(filenames):
                if os.path.islink(os.path.join(dirpath, n)):
                    bad.append(os.path.relpath(os.path.join(dirpath, n), b))
        self.check(not bad, "W7.symlink", "symlink in shared tree: " + ", ".join(sorted(bad)))

    # -- W8 ---------------------------------------------------------------
    def w8_parent_wiring(self):
        """The overlaid drivers/net/{Kconfig,Makefile} really reach gpon/."""
        kc = self.read(SHARED_FILES + "/drivers/net/Kconfig")
        mk = self.read(SHARED_FILES + "/drivers/net/Makefile")
        self.check(kc is not None and 'source "drivers/net/gpon/Kconfig"' in kc,
                   "W8.kconfig", "drivers/net/Kconfig does not source gpon/Kconfig")
        self.check(mk is not None
                   and re.search(r'^obj-\$\(CONFIG_GPON_CORE\)\s*\+=\s*gpon/\s*$',
                                 mk or "", re.M) is not None,
                   "W8.makefile", "drivers/net/Makefile does not descend into gpon/")

    # -- W9 ---------------------------------------------------------------
    def w9_someone_selects(self):
        """At least one vendor driver must `select GPON_CORE` in EACH target.

        Nobody selecting it is the quietest possible failure: the parent never
        descends, the whole shared core is never compiled, and every build is
        green.  GPON_CORE deliberately has no `default`, so a select is the
        only way in -- and no config-<ver> needs editing.
        """
        want = {
            "realtek-luna": "target/linux/realtek-luna/files-%s/drivers/net/ethernet/"
                            "realtek/Kconfig" % VER,
            "realtek-elnath": "target/linux/realtek-elnath/files-%s/drivers/net/ethernet/"
                              "cortina/Kconfig" % VER,
        }
        for t, rel in want.items():
            txt = self.read(rel)
            self.check(txt is not None and re.search(r'^\s*select\s+GPON_CORE\b', txt, re.M),
                       "W9.select/" + t,
                       "no `select GPON_CORE` in %s -- the shared core would never "
                       "be compiled for this target, silently" % rel)
        kc = self.read(GPON_DIR + "/Kconfig")
        self.check(kc is not None and re.search(r'^config GPON_CORE\s*$', kc or "", re.M),
                   "W9.declared", "GPON_CORE is not declared in " + GPON_DIR + "/Kconfig")

    def run(self):
        for fn in (self.w1_files_dir_line, self.w2_trees_disjoint, self.w3_o2,
                   self.w4_objects_accounted, self.w5_no_patch_collides,
                   self.w6_no_scanned_makefile, self.w7_no_symlinks,
                   self.w8_parent_wiring, self.w9_someone_selects):
            fn()
        return self.fails


def find_root(start):
    d = os.path.abspath(start)
    while d != "/":
        if os.path.isdir(os.path.join(d, "target/linux")) and \
           os.path.isfile(os.path.join(d, "include/kernel.mk")):
            return d
        d = os.path.dirname(d)
    sys.exit("could not find the openwrt tree root above " + start)


# --------------------------------------------------------------------------
# --self-check: MUTATE the subject in a scratch copy and require each
# assertion to go RED.  Reading the guard proves nothing about it.
# --------------------------------------------------------------------------
MUTATIONS = [
    ("W1.append", "target/linux/realtek-luna/Makefile",
     lambda s: re.sub(r'^FILES_DIR \+=.*$', '', s, flags=re.M)),
    ("W1.order", "target/linux/realtek-luna/Makefile",
     lambda s: re.sub(r'^(FILES_DIR \+=.*)$', '', s, flags=re.M).replace(
         "include $(INCLUDE_DIR)/target.mk",
         'FILES_DIR += "$(TOPDIR)/target/linux/gpon-common/files-$(KERNEL_PATCHVER)"\n'
         "include $(INCLUDE_DIR)/target.mk", 1)),
    ("W3.o2", SHARED_FILES + "/drivers/net/gpon/Makefile",
     lambda s: re.sub(r'^ccflags-y \+= -O2$', '', s, flags=re.M)),
    # the "silently dead source" arm: drop one pending declaration and the file
    # it names becomes a .c nothing accounts for.
    ("W4.src_never_built", SHARED_FILES + "/drivers/net/gpon/Makefile",
     lambda s: re.sub(r'^# gpon-pending: gpon_ploam\.o.*(?:\n#   .*)*$', '', s, flags=re.M)),
    # the "reason is a placeholder" arm.
    ("W4.pending_reason", SHARED_FILES + "/drivers/net/gpon/Makefile",
     lambda s: re.sub(r'^(# gpon-pending: gpon_ploam\.o --).*(?:\n#   .*)*$',
                      r'\1 later', s, flags=re.M)),
    ("W8.kconfig", SHARED_FILES + "/drivers/net/Kconfig",
     lambda s: s.replace('source "drivers/net/gpon/Kconfig"', '')),
    ("W8.makefile", SHARED_FILES + "/drivers/net/Makefile",
     lambda s: re.sub(r'^obj-\$\(CONFIG_GPON_CORE\) \+= gpon/$', '', s, flags=re.M)),
    ("W9.select", "target/linux/realtek-elnath/files-%s/drivers/net/ethernet/"
                  "cortina/Kconfig" % VER,
     lambda s: re.sub(r'^\tselect GPON_CORE.*$', '', s, flags=re.M)),
]


def self_check(root):
    """Copy only what the guard reads, mutate one thing, require a FAIL."""
    subtrees = [SHARED, "include", "target/linux/realtek-luna",
                "target/linux/realtek-elnath"]
    rc = 0
    for name, rel, mutate in MUTATIONS:
        tmp = tempfile.mkdtemp(prefix="gpon-wiring-")
        try:
            for s in subtrees:
                src, dst = os.path.join(root, s), os.path.join(tmp, s)
                if os.path.isdir(src):
                    shutil.copytree(src, dst, symlinks=True,
                                    ignore=shutil.ignore_patterns("*.o", "*.ko"))
            p = os.path.join(tmp, rel)
            with open(p, "r", encoding="utf-8") as f:
                before = f.read()
            after = mutate(before)
            if before == after:
                print("  %-14s MUTATION IS A NO-OP -- the guard is untested" % name)
                rc = 1
                continue
            with open(p, "w", encoding="utf-8") as f:
                f.write(after)
            fails = Guard(tmp).run()
            hit = any(f.split()[0].startswith(name) for f in fails)
            print("  %-14s mutate %-58s -> %s" %
                  (name, os.path.basename(rel), "RED (guard works)" if hit
                   else "STILL GREEN -- VACUOUS GUARD"))
            if not hit:
                rc = 1
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    return rc


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=None, help="openwrt tree root (autodetected)")
    ap.add_argument("--self-check", action="store_true",
                    help="mutate each subject in a scratch copy; every check must go RED")
    ap.add_argument("--LABEL", default=None, help="run label, recorded in the output")
    a = ap.parse_args()

    root = a.root or find_root(os.path.dirname(os.path.abspath(__file__)))
    print("gpon build-wiring guard   root=%s%s" %
          (root, "   LABEL=%s" % a.LABEL if a.LABEL else ""))

    g = Guard(root)
    fails = g.run()
    for f in fails:
        print("FAIL  " + f)
    print("%d check(s), %d failed" % (g.checks, len(fails)))

    rc = 1 if fails else 0
    if a.self_check:
        print("\n--self-check: each mutation MUST turn the gate red")
        rc |= self_check(root)
    return rc


if __name__ == "__main__":
    sys.exit(main())
