#
# rdmavt module
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

NOSTDINC_FLAGS += -I${M}/include -I${M}/compat

obj-$(CONFIG_INFINIBAND_RDMAVT) += rdmavt.o

rdmavt-y := vt.o ah.o cq.o mad.o mcast.o mmap.o mr.o pd.o qp.o srq.o \
	 trace.o rc.o compat_common.o compat.o

CFLAGS_trace.o = -I$(src)

else
#normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD CONFIG_INFINIBAND_RDMAVT=m NOSTDINC_FLAGS=-I$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KDIR) M=$$PWD modules_install

endif
