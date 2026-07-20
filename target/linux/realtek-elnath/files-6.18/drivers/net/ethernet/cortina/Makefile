# SPDX-License-Identifier: GPL-2.0
# Makefile for the Cortina Gemini network device drivers.

obj-$(CONFIG_GEMINI_ETHERNET) += gemini.o
obj-$(CONFIG_CORTINA_NI) += cortina_ni.o
cortina_ni-y := cortina-ni.o cortina-ni-tx.o cortina-ni-rx.o
# L3FE main-hash flow engine (nf_flow_table HW offload backend)
cortina_ni-$(CONFIG_CORTINA_NI_FLOWOFFLOAD) += cortina-ni-flowoffload.o \
	cortina-l3fe.o cortina-l3fe-aging.o
obj-$(CONFIG_CORTINA_GPON) += cortina_gpon.o
cortina_gpon-y := cortina-gpon.o cortina-gpon-bosa.o cortina-i2c.o omci_responder.o
