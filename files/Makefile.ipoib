# SPDX-License-Identifier: GPL-2.0

ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS += -D IPOIB_DRIVER

NOSTDINC_FLAGS := -I${M}/include -I${M}/compat

obj-$(CONFIG_INFINIBAND_IPOIB)			+= ib_ipoib.o

ib_ipoib-y					:= ipoib_main.o \
						   ipoib_ib.o \
						   ipoib_multicast.o \
						   ipoib_verbs.o \
						   ipoib_vlan.o \
						   ipoib_ethtool.o \
						   ipoib_netlink.o
ib_ipoib-$(CONFIG_INFINIBAND_IPOIB_CM)		+= ipoib_cm.o
ib_ipoib-$(CONFIG_INFINIBAND_IPOIB_DEBUG)	+= ipoib_fs.o

else

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD NOSTDINC_FLAGS=-I$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KDIR) M=$$PWD modules_install


endif
