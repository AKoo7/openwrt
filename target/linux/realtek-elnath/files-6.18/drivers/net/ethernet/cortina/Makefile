# SPDX-License-Identifier: GPL-2.0
# Makefile for the Cortina Gemini network device drivers.

obj-$(CONFIG_GEMINI_ETHERNET) += gemini.o
obj-$(CONFIG_CORTINA_NI) += cortina_ni.o
cortina_ni-y := cortina-ni.o cortina-ni-tx.o cortina-ni-rx.o
obj-$(CONFIG_CORTINA_GPON) += cortina_gpon.o
cortina_gpon-y := cortina-gpon.o cortina-gpon-bosa.o cortina-i2c.o omci_responder.o
