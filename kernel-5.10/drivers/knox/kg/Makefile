# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/crypto/asymmetric_keys
ifdef CONFIG_TZDEV
  ifeq ($(CONFIG_TZDEV),m)
    ccflags-y += -I$(srctree)/drivers/misc/tzdev/include/tzdev
  endif
endif

kg_drv-y := kg_main.o kg_key_info.o kg_resource_control.o kg_resource_memory.o kg_ta_call.o kg_dr.o kg_x509_cert.o kg_kernel_api.o kg_netlink.o
kg_drv-$(CONFIG_KG_LOG)	+= kg_log.o
kg_drv-$(CONFIG_QSEECOM_PROXY) += kg_ta_qsee.o
kg_drv-$(CONFIG_TZDEV) += kg_ta_tzdev.o
kg_drv-$(CONFIG_KG_RESOURCE_CPU) += kg_resource_cpu.o

obj-$(CONFIG_KG_DRV)	+= kg_drv.o
