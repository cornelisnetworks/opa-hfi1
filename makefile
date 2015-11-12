# Copyright (c) 2013-2015. Intel Corporation. All rights reserved.
# Copyright (c) 2007, 2008, 2009. QLogic Corp. All rights reserved.
# Copyright (c) 2003, 2004, 2005. PathScale, Inc. All rights reserved.
#
# This software is available to you under a choice of one of two
# licenses.  You may choose to be licensed under the terms of the GNU
# General Public License (GPL) Version 2, available from the file
# COPYING in the main directory of this source tree, or the
# OpenIB.org BSD license below:
#
#     Redistribution and use in source and binary forms, with or
#     without modification, are permitted provided that the following
#     conditions are met:
#
#      - Redistributions of source code must retain the above
#        copyright notice, this list of conditions and the following
#        disclaimer.
#
#      - Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials
#        provided with the distribution.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Patent licenses, if any, provided herein do not apply to
# combinations of this program with other software, or any other
# product whatsoever.
#
# The desired version number comes from the most recent tag starting with "v"
BASEVERSION=0.2
VERSION = $(shell if [ -e .git ] ; then  git describe --tags --abbrev=0 --match='v*' | sed -e 's/^v//' -e 's/-/_/'; else echo "version" ; fi)
# The desired release number comes the git describe following the version which
# is the number of commits since the version tag was planted suffixed by the g<commitid>
RELEASE = $(shell if [ -e .git ] ; then git describe --tags --long --match='v*' | sed -e 's/v[0-9.]*-\([0-9]*\)/\1/' | sed 's/-g.*$$//'; else echo "release" ; fi)

EXCLUDES = --exclude-vcs --exclude-backups --exclude='*.patch' --exclude='*.swp' --exclude='series' --exclude='*.orig' --exclude=hfi.spec.in

NAME = hfi1

KVER=$(shell uname -r)
KBUILD  ?= /lib/modules/$(KVER)/build

# Enable extra SDMA debugging
#INTERNAL_NOSTDINC_FLAGS += -DCONFIG_SDMA_VERBOSITY
# Disable compatibility module parameters
#INTERNAL_NOSTDINC_FLAGS += -DHFI_COMPAT_MODPARAMS=0
# Debug sdma ordering
#INTERNAL_NOSTDINC_FLAGS += -DCONFIG_HFI1_DEBUG_SDMA_ORDER
INTERNAL_NOSTDINC_FLAGS += -DCONFIG_PRESCAN_RXQ
INTERNAL_NOSTDINC_FLAGS += -DCONFIG_HFI1_VERBS_31BIT_PSN
INTERNAL_NOSTDINC_FLAGS += -I$(KBUILD)/include-ifs-kernel -I$(PWD)
override NOSTDINC_FLAGS += $(INTERNAL_NOSTDINC_FLAGS)
KBUILD_EXTRA_SYMBOLS := $(KBUILD)/include-ifs-kernel/Module.symvers
HFI_HEADER_DIR := $(dir $(shell find . -name "hfi1_user.h" | sed -e 's%^\./%%'))
HFI_HEADER_INSTALL_DIR := /usr/include/$(HFI_HEADER_DIR)

PWD:=$(shell pwd)

driver:
	[ -e $(KBUILD)/include-ifs-kernel/Module.symvers ] && cp $(KBUILD)/include-ifs-kernel/Module.symvers . || :
	make RELEASE=$(VERSION)-$(RELEASE)$(KVER) -C $(KBUILD) M=$(PWD) MVERSION=$(VERSION)-$(RELEASE) \
		CONFIG_INFINIBAND_HFI1=m \
		NOSTDINC_FLAGS="$(NOSTDINC_FLAGS)" KBUILD_EXTRA_SYMBOLS="$(KBUILD_EXTRA_SYMBOLS)"
clean:
	make -C $(KBUILD) M=$(PWD) CONFIG_INFINIBAND_HFI1=m clean

distclean: clean
	rm -f hfi.spec
	rm -f *.tgz

specfile: hfi.spec.in
	sed \
		-e 's/@VERSION@/'${VERSION}'/g' \
		-e 's/@RELEASE@/'${RELEASE}'/g' \
		-e 's/@NAME@/'${NAME}'/g' \
		-e 's%@USER_INC_DIR@%'$(HFI_HEADER_INSTALL_DIR)'%g' \
		hfi.spec.in > hfi.spec
	@if [ -e .git ]; then \
		echo '%changelog' >> hfi.spec; \
		for x in $(shell git log v$(BASEVERSION)..HEAD --no-merges --format="%at %H" |sort -rg | cut -d' ' -f2); do \
			git log -1 --format="* %ad <%ae>%n- %s%n" $$x \
			| sed -e 's/-[0-9][0-9][0-9][0-9] //' \
				-e 's/ [0-9][0-9]:[0-9][0-9]:[0-9][0-9]//' >> hfi.spec;\
		done \
	fi


dist: distclean specfile
	rm -rf /tmp/hfi1-$(VERSION)
	mkdir -p /tmp/hfi1-$(VERSION)
	cp -r . /tmp/hfi1-$(VERSION)
	tar $(EXCLUDES) -C /tmp -zcvf $(PWD)/hfi1-$(VERSION).tgz hfi1-$(VERSION)
	rm -rf /tmp/hfi1-$(VERSION)

install:
	mkdir -p $(RPM_BUILD_ROOT)/lib/modules/$(KVER)/updates
	install hfi1.ko $(RPM_BUILD_ROOT)/lib/modules/$(KVER)/updates
	mkdir -p $(RPM_BUILD_ROOT)$(HFI_HEADER_INSTALL_DIR) && \
		(cd $(KBUILD) && sh scripts/headers_install.sh \
			$(RPM_BUILD_ROOT)$(HFI_HEADER_INSTALL_DIR) \
			$(PWD) $(HFI_HEADER_DIR)hfi1_user.h)
version:
	@[ -e .git ] && echo -n $(VERSION)-$(RELEASE)

print_vers:
	@echo $(VERSION)

print_rel:
	@echo $(RELEASE)
