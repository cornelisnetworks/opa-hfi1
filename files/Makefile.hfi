# SPDX-License-Identifier: GPL-2.0
#
# hfi1 module
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

NOSTDINC_FLAGS += -I${M}/include -I${M}/include/uapi -I${M}/compat

obj-$(CONFIG_INFINIBAND_HFI1) += hfi1.o

hfi1-y := affinity.o aspm.o  chip.o device.o driver.o efivar.o eprom.o \
	file_ops.o firmware.o init.o intr.o mad.o mmu_rb.o pcie.o pio.o \
	pio_copy.o platform.o qp.o qsfp.o rc.o ruc.o sdma.o sysfs.o trace.o \
	uc.o ud.o user_exp_rcv.o exp_rcv.o user_pages.o user_sdma.o verbs.o \
	verbs_txreq.o diag.o tid_rdma.o opfn.o iowait.o \
	ipoib_main.o ipoib_tx.o rcva.o msix.o ipoib_rx.o netdev_rx.o
hfi1-$(CONFIG_DEBUG_FS) += debugfs.o

CFLAGS_trace.o = -I$(src)
ifdef MVERSION
CFLAGS_driver.o = -DHFI1_DRIVER_VERSION_BASE=\"$(MVERSION)\"
endif

else
#normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD NOSTDINC_FLAGS=-I$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KDIR) M=$$PWD modules_install

endif
