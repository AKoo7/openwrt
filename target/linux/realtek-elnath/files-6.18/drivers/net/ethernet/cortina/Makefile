# SPDX-License-Identifier: GPL-2.0
# Makefile for the Cortina Gemini network device drivers.

# The shared, hardware-decoupled GPON protocol core lives in its own tree,
# target/linux/gpon-common/files-6.18/drivers/net/gpon/, and reaches this build
# through FILES_DIR (see this target's Makefile).  cortina-gpon.c includes its
# headers -- gpon_omci_core.h, gpon_omci_me.h, gpon_gem_us.h -- so this
# directory needs to be able to find them.
#
# WHY AN -I AND NOT A RELATIVE INCLUDE: the shared tree is a SEPARATE files-
# overlay that is merged into the kernel source at the same place upstream's own
# directories land, so its path relative to this file is an artefact of the
# overlay mechanism, not a property of the source.  Naming the directory from
# $(srctree) says what is actually meant -- "the in-tree GPON core" -- and keeps
# working if either overlay moves.
#
# The OBJECTS behind these headers are built by drivers/net/gpon/Makefile, which
# the parent drivers/net/Makefile descends into when CONFIG_GPON_CORE is set.
# CONFIG_CORTINA_GPON selects GPON_CORE (see Kconfig in this directory), so a
# kernel that builds cortina_gpon.ko always has the core available to link
# against; there is deliberately no way to configure one without the other.
ccflags-y += -I$(srctree)/drivers/net/gpon

obj-$(CONFIG_GEMINI_ETHERNET) += gemini.o
obj-$(CONFIG_CORTINA_NI) += cortina_ni.o
cortina_ni-y := cortina-ni.o cortina-ni-tx.o cortina-ni-rx.o cortina-ni-ethtool.o
# DSA (lan1..lan4): the switch driver half; the tagger is net/dsa/tag_cortina.o
cortina_ni-$(CONFIG_CORTINA_NI_DSA) += cortina-ni-dsa.o
# L3FE main-hash flow engine (nf_flow_table HW offload backend)
cortina_ni-$(CONFIG_CORTINA_NI_FLOWOFFLOAD) += cortina-ni-flowoffload.o \
	cortina-l3fe.o cortina-l3fe-aging.o
obj-$(CONFIG_CORTINA_GPON) += cortina_gpon.o
# ★ omci_responder.o is GONE from this list (code motion, 2026-08-05).  The
# G.988 OMCI responder it built -- omci_responder.{c,h}, 1112 + 143 lines -- was
# moved UNCHANGED into the shared protocol core as gpon_omci_core.{h,c} (the
# baseline message layer) and gpon_omci_me.{h,c} (the managed-entity model), and
# is now built by drivers/net/gpon/Makefile instead.  G.988 is a specification,
# not a Cortina fact: two private copies of it is one copy that silently drifts,
# which is why the tree keeps exactly one.  The cut site, the evidence that the
# move changed no byte, and what stayed behind are documented at the include in
# cortina-gpon.c.
cortina_gpon-y := cortina-gpon.o cortina-gpon-bosa.o cortina-i2c.o
# M2 userspace-daemon bridge: netlink OMCI plane + nf_sockopt control plane +
# /proc/omci/devMode, so the stock Zyxel omci_app can run in a chroot and drive
# the real provisioned MIB.  Selects nothing at runtime unless
# cortina_gpon.omci_userspace=1.
cortina_gpon-$(CONFIG_CORTINA_GPON_OMCI_BRIDGE) += cortina-omci-bridge.o
