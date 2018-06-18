#!/bin/bash

DEFAULT_KERNEL_VERSION=""
DEFAULT_USER=$USER
DEFAULT_URL_PREFIX="ssh://"
DEFAULT_URL_SUFFIX="@git-amr-2.devtools.intel.com:29418/wfr-linux-devel"
DEFAULT_BRANCH="upstream-for-wfr-ifs-sles12.2"
wfrconfig="arch/x86/configs/wfr-config"
kerneldir="./"

# Add each module separately
modules[0]="rdmavt"
files_to_copy[0]="
	drivers/infiniband/sw/rdmavt/ah.c
	drivers/infiniband/sw/rdmavt/ah.h
	drivers/infiniband/sw/rdmavt/cq.c
	drivers/infiniband/sw/rdmavt/cq.h
	drivers/infiniband/sw/rdmavt/mad.c
	drivers/infiniband/sw/rdmavt/mad.h
	drivers/infiniband/sw/rdmavt/mcast.c
	drivers/infiniband/sw/rdmavt/mcast.h
	drivers/infiniband/sw/rdmavt/mmap.c
	drivers/infiniband/sw/rdmavt/mmap.h
	drivers/infiniband/sw/rdmavt/mr.c
	drivers/infiniband/sw/rdmavt/mr.h
	drivers/infiniband/sw/rdmavt/pd.c
	drivers/infiniband/sw/rdmavt/pd.h
	drivers/infiniband/sw/rdmavt/qp.c
	drivers/infiniband/sw/rdmavt/qp.h
	drivers/infiniband/sw/rdmavt/srq.c
	drivers/infiniband/sw/rdmavt/srq.h
	drivers/infiniband/sw/rdmavt/trace.c
	drivers/infiniband/sw/rdmavt/trace.h
	drivers/infiniband/sw/rdmavt/trace_rvt.h
	drivers/infiniband/sw/rdmavt/trace_qp.h
	drivers/infiniband/sw/rdmavt/trace_tx.h
	drivers/infiniband/sw/rdmavt/trace_mr.h
	drivers/infiniband/sw/rdmavt/trace_rc.h
	drivers/infiniband/sw/rdmavt/trace_cq.h
	drivers/infiniband/sw/rdmavt/vt.c
	drivers/infiniband/sw/rdmavt/vt.h
	drivers/infiniband/sw/rdmavt/rc.c
	compat/SLES12SP2/compat.c
	compat/common/compat_common.c
"

modules[1]="hfi1"
files_to_copy[1]="
	drivers/infiniband/hw/hfi1/affinity.c
	drivers/infiniband/hw/hfi1/affinity.h
	drivers/infiniband/hw/hfi1/aspm.h
	drivers/infiniband/hw/hfi1/chip.c
	drivers/infiniband/hw/hfi1/chip.h
	drivers/infiniband/hw/hfi1/chip_registers.h
	drivers/infiniband/hw/hfi1/common.h
	drivers/infiniband/hw/hfi1/debugfs.c
	drivers/infiniband/hw/hfi1/debugfs.h
	drivers/infiniband/hw/hfi1/device.c
	drivers/infiniband/hw/hfi1/device.h
	drivers/infiniband/hw/hfi1/driver.c
	drivers/infiniband/hw/hfi1/efivar.c
	drivers/infiniband/hw/hfi1/efivar.h
	drivers/infiniband/hw/hfi1/eprom.c
	drivers/infiniband/hw/hfi1/eprom.h
	drivers/infiniband/hw/hfi1/file_ops.c
	drivers/infiniband/hw/hfi1/gdr_ops.c
	drivers/infiniband/hw/hfi1/gdr_ops.h
	drivers/infiniband/hw/hfi1/firmware.c
	drivers/infiniband/hw/hfi1/hfi.h
	drivers/infiniband/hw/hfi1/init.c
	drivers/infiniband/hw/hfi1/intr.c
	drivers/infiniband/hw/hfi1/iowait.h
	drivers/infiniband/hw/hfi1/mad.c
	drivers/infiniband/hw/hfi1/mad.h
	drivers/infiniband/hw/hfi1/mmu_rb.c
	drivers/infiniband/hw/hfi1/mmu_rb.h
	drivers/infiniband/hw/hfi1/opa_compat.h
	drivers/infiniband/hw/hfi1/pcie.c
	drivers/infiniband/hw/hfi1/pio.c
	drivers/infiniband/hw/hfi1/pio_copy.c
	drivers/infiniband/hw/hfi1/pio.h
	drivers/infiniband/hw/hfi1/platform.c
	drivers/infiniband/hw/hfi1/platform.h
	drivers/infiniband/hw/hfi1/qp.c
	drivers/infiniband/hw/hfi1/qp.h
	drivers/infiniband/hw/hfi1/qsfp.c
	drivers/infiniband/hw/hfi1/qsfp.h
	drivers/infiniband/hw/hfi1/rc.c
	drivers/infiniband/hw/hfi1/ruc.c
	drivers/infiniband/hw/hfi1/diag.c
	drivers/infiniband/hw/hfi1/sdma.c
	drivers/infiniband/hw/hfi1/sdma.h
	drivers/infiniband/hw/hfi1/sdma_txreq.h
	drivers/infiniband/hw/hfi1/sysfs.c
	drivers/infiniband/hw/hfi1/trace.c
	drivers/infiniband/hw/hfi1/trace.h
	drivers/infiniband/hw/hfi1/trace_ctxts.h
	drivers/infiniband/hw/hfi1/trace_dbg.h
	drivers/infiniband/hw/hfi1/trace_ibhdrs.h
	drivers/infiniband/hw/hfi1/trace_misc.h
	drivers/infiniband/hw/hfi1/trace_rc.h
	drivers/infiniband/hw/hfi1/trace_rx.h
	drivers/infiniband/hw/hfi1/trace_tx.h
	drivers/infiniband/hw/hfi1/trace_mmu.h
	drivers/infiniband/hw/hfi1/trace_gpu.h
	drivers/infiniband/hw/hfi1/gpu.c
	drivers/infiniband/hw/hfi1/gpu.h
	drivers/infiniband/hw/hfi1/uc.c
	drivers/infiniband/hw/hfi1/ud.c
	drivers/infiniband/hw/hfi1/user_exp_rcv.c
	drivers/infiniband/hw/hfi1/user_exp_rcv.h
	drivers/infiniband/hw/hfi1/user_exp_rcv_gpu.c
	drivers/infiniband/hw/hfi1/user_exp_rcv_gpu.h
	drivers/infiniband/hw/hfi1/user_pages.c
	drivers/infiniband/hw/hfi1/user_sdma.c
	drivers/infiniband/hw/hfi1/user_sdma.h
	drivers/infiniband/hw/hfi1/user_sdma_gpu.c
	drivers/infiniband/hw/hfi1/user_sdma_gpu.h
	drivers/infiniband/hw/hfi1/verbs.c
	drivers/infiniband/hw/hfi1/verbs.h
	drivers/infiniband/hw/hfi1/verbs_txreq.c
	drivers/infiniband/hw/hfi1/verbs_txreq.h
	drivers/infiniband/hw/hfi1/vnic.h
	drivers/infiniband/hw/hfi1/exp_rcv.c
	drivers/infiniband/hw/hfi1/exp_rcv.h
	drivers/infiniband/hw/hfi1/iowait.c
        drivers/infiniband/hw/hfi1/opfn.c
        drivers/infiniband/hw/hfi1/opfn.h
        drivers/infiniband/hw/hfi1/rc.h
        drivers/infiniband/hw/hfi1/tid_rdma.c
        drivers/infiniband/hw/hfi1/tid_rdma.h
        drivers/infiniband/hw/hfi1/trace_iowait.h
        drivers/infiniband/hw/hfi1/trace_tid.h
"

modules[2]="ib_qib"
files_to_copy[2]="
	drivers/infiniband/hw/qib/qib_6120_regs.h
	drivers/infiniband/hw/qib/qib_7220.h
	drivers/infiniband/hw/qib/qib_7220_regs.h
	drivers/infiniband/hw/qib/qib_7322_regs.h
	drivers/infiniband/hw/qib/qib_debugfs.h
	drivers/infiniband/hw/qib/qib_qsfp.h
	drivers/infiniband/hw/qib/qib_user_sdma.h
	drivers/infiniband/hw/qib/qib_wc_ppc64.c
	drivers/infiniband/hw/qib/qib_pio_copy.c
	drivers/infiniband/hw/qib/qib_diag.c
	drivers/infiniband/hw/qib/qib_eeprom.c
	drivers/infiniband/hw/qib/qib_mad.h
	drivers/infiniband/hw/qib/qib_sd7220.c
	drivers/infiniband/hw/qib/qib_twsi.c
	drivers/infiniband/hw/qib/qib_tx.c
	drivers/infiniband/hw/qib/qib_user_sdma.c
	drivers/infiniband/hw/qib/qib_debugfs.c
	drivers/infiniband/hw/qib/qib_pcie.c
	drivers/infiniband/hw/qib/qib.h
	drivers/infiniband/hw/qib/qib_common.h
	drivers/infiniband/hw/qib/qib_driver.c
	drivers/infiniband/hw/qib/qib_file_ops.c
	drivers/infiniband/hw/qib/qib_fs.c
	drivers/infiniband/hw/qib/qib_iba6120.c
	drivers/infiniband/hw/qib/qib_iba7220.c
	drivers/infiniband/hw/qib/qib_iba7322.c
	drivers/infiniband/hw/qib/qib_init.c
	drivers/infiniband/hw/qib/qib_intr.c
	drivers/infiniband/hw/qib/qib_keys.c
	drivers/infiniband/hw/qib/qib_mad.c
	drivers/infiniband/hw/qib/qib_qp.c
	drivers/infiniband/hw/qib/qib_qsfp.c
	drivers/infiniband/hw/qib/qib_rc.c
	drivers/infiniband/hw/qib/qib_ruc.c
	drivers/infiniband/hw/qib/qib_sdma.c
	drivers/infiniband/hw/qib/qib_sysfs.c
	drivers/infiniband/hw/qib/qib_uc.c
	drivers/infiniband/hw/qib/qib_ud.c
	drivers/infiniband/hw/qib/qib_user_pages.c
	drivers/infiniband/hw/qib/qib_verbs.c
	drivers/infiniband/hw/qib/qib_verbs.h
	drivers/infiniband/hw/qib/qib_wc_x86_64.c
"
modules[3]="ib_mad"
files_to_copy[3]="
	distro/SLES12SP2/core/agent.c
	distro/SLES12SP2/core/agent.h
	distro/SLES12SP2/core/mad.c
	distro/SLES12SP2/core/mad_priv.h
	distro/SLES12SP2/core/mad_rmpp.c
	distro/SLES12SP2/core/mad_rmpp.h
	distro/SLES12SP2/core/opa_smi.h
	distro/SLES12SP2/core/smi.c
	distro/SLES12SP2/core/smi.h
"
modules_cnt=${#modules[@]}

include_dirs[0]="include/rdma"
include_dirs[1]="include/rdma/hfi"
include_dirs[2]="compat/"
include_files_to_copy[0]="
	include/rdma/rdmavt_qp.h
	include/rdma/rdmavt_mr.h
	include/rdma/rdma_vt.h
	include/rdma/ib_hdrs.h
	include/rdma/opa_vnic.h
	include/rdma/opa_addr.h
	include/rdma/opa_port_info.h
	include/rdma/tid_rdma_defs.h
	include/uapi/rdma/rdma_user_ioctl.h
"
include_files_to_copy[1]="
	include/uapi/rdma/hfi/hfi1_user.h
	include/uapi/rdma/hfi/hfi1_ioctl.h
"

include_files_to_copy[2]="
        compat/SLES12SP2/compat.h
        compat/common/compat_common.h
"
include_dirs_cnt=${#include_dirs[@]}

# ridiculously long to encourage good names later
rpmname="ifs-kernel-updates"

set -e

function usage
{
	cat <<EOL
usage:
	${0##*/} -h
	${0##*/} [-G] [-b branch] [-w dirname] [-u user] [-U URL]
	${0##*/} -S srcdir [-w dirname]

Options:

-G         - fetch source from a git repository [DEFAULT]
-S srcdir  - fetch source directly from a specified directory

-b branch  - branch of URL to check out [$DEFAULT_BRANCH]
-w dirname - work directory, defaults to a mktemp directory
-h         - this help text
-u user    - user for the defaut URL [$DEFAULT_USER]
-U URL	   - source checkout url [$DEFAULT_URL_PREFIX$USER$DEFAULT_URL_SUFFIX]
EOL
}

gitfetch="maybe"
srcdir=""
workdir=""
url=""
branch=$DEFAULT_BRANCH
user=$DEFAULT_USER
while getopts "GS:U:b:hu:w:" opt; do
    	case "$opt" in
	G)	[ "$gitfetch" = "false" ] && usage && exit 1
		gitfetch="true"
		;;
	S)	[ "$gitfetch" = "true" ] && usage && exit 1
		srcdir="$OPTARG"
		gitfetch="false"
		[ ! -e "$srcdir" ] && echo "srcdir $srcdir not found" && exit 1
		srcdir=$(readlink -f "$srcdir")
		;;
	U)	[ "$gitfetch" = "false" ] && usage && exit 1
		url="$OPTARG"
		gitfetch="true"
		;;
	b)	[ "$gitfetch" = "false" ] && usage && exit 1
		branch="$OPTARG"
		gitfetch="true"
		;;
	h)	usage
		exit 0
		;;
	u)	[ "$gitfetch" = "false" ] && usage && exit 1
		user="$OPTARG"
		gitfetch="true"
		;;
	w)	workdir="$OPTARG"
		;;

    	esac
done

# create final version of the variables
if [ -n "$workdir" ]; then
	mkdir -p "$workdir" || exit 1
else
	workdir=$(mktemp -d --tmpdir=$(pwd) build.XXXX)
	[ ! $? ] && exit 1
fi
[ -z "$url" ] && url="$DEFAULT_URL_PREFIX$user$DEFAULT_URL_SUFFIX"

# after cd, where are we *really*
cd -P "$workdir"; workdir=$(pwd)
tardir=$workdir/stage
rm -rf $tardir
for (( i = 0 ; i < modules_cnt ; i++ ))
do
	mkdir -p $tardir/${modules[$i]}
done

echo "Working in $workdir"

# check out the sources
if [ "$gitfetch" != "false" ]; then
	rm -rf ksrc
	echo "Checking out source"
	time git clone --single-branch --branch $branch $url ksrc
	srcdir="ksrc"
fi


# create the Makefiles
echo "Creating Makefile ($tardir/Makefile)"
cat > $tardir/Makefile <<'EOF'
#
# Top level
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

CFLAGS_MODULE += -DIFS_SLES12SP2
obj-y := rdmavt/ hfi1/ ib_qib/ ib_mad/

else
#normal makefile
KDIR ?= /lib/modules/`uname -r`/build
ifneq ($(NVIDIA_GPU_DIRECT),)
export KBUILD_EXTRA_SYMBOLS=${NVIDIA_GPU_DIRECT}/Module.symvers
endif

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KDIR) M=$$PWD modules_install

endif

EOF

echo "Creating Makefile ($tardir/rdmavt/Makefile)"
cat > $tardir/rdmavt/Makefile <<'EOF'
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

EOF

echo "Creating Makefile ($tardir/hfi1/Makefile)"
cat > $tardir/hfi1/Makefile <<'EOF'
#
# hfi1 module
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

NOSTDINC_FLAGS += -I${M}/include -I${M}/compat
ifneq ($(NVIDIA_GPU_DIRECT),)
NOSTDINC_FLAGS += -DNVIDIA_GPU_DIRECT
endif

obj-$(CONFIG_INFINIBAND_HFI1) += hfi1.o

hfi1-y := affinity.o chip.o device.o driver.o efivar.o eprom.o \
	file_ops.o firmware.o init.o intr.o mad.o mmu_rb.o pcie.o pio.o \
	pio_copy.o platform.o qp.o qsfp.o rc.o ruc.o sdma.o sysfs.o trace.o \
	uc.o ud.o user_exp_rcv.o diag.o exp_rcv.o user_pages.o user_sdma.o verbs.o \
	verbs_txreq.o user_exp_rcv_gpu.o user_sdma_gpu.o gpu.o \
	gdr_ops.o tid_rdma.o opfn.o iowait.o
hfi1-$(CONFIG_DEBUG_FS) += debugfs.o

CFLAGS_trace.o = -I$(src)

ifneq ($(NVIDIA_GPU_DIRECT),)
CFLAGS_driver.o = -DHFI1_IDSTR=\"gpu-direct\"
CFLAGS_gpu.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
CFLAGS_gdr_ops.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
CFLAGS_file_ops.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
CFLAGS_user_sdma.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
CFLAGS_user_sdma_gpu.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
CFLAGS_user_exp_rcv.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
CFLAGS_user_exp_rcv_gpu.o = -I$(NVIDIA_GPU_DIRECT) -I$(NVIDIA_GPU_DIRECT)/nvidia
endif

ifdef MVERSION
CFLAGS_driver.o += -DHFI1_DRIVER_VERSION_BASE=\"$(MVERSION)\"
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

EOF

echo "Creating Makefile ($tardir/ib_qib/Makefile)"
cat > $tardir/ib_qib/Makefile <<'EOF'
#
# ib_qib module
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

EXTRA_CFLAGS +=-D QIB_DRIVER

NOSTDINC_FLAGS := -I${M}/include -I${M}/compat

obj-$(CONFIG_INFINIBAND_QIB) += ib_qib.o

ib_qib-y := qib_diag.o qib_driver.o qib_eeprom.o \
        qib_file_ops.o qib_fs.o qib_init.o qib_intr.o \
        qib_mad.o qib_pcie.o qib_pio_copy.o \
        qib_qp.o qib_qsfp.o qib_rc.o qib_ruc.o qib_sdma.o \
        qib_sysfs.o qib_twsi.o qib_tx.o qib_uc.o qib_ud.o \
        qib_user_pages.o qib_user_sdma.o qib_iba7220.o \
        qib_sd7220.o qib_iba7322.o qib_verbs.o

# 6120 has no fallback if no MSI interrupts, others can do INTx
ib_qib-$(CONFIG_PCI_MSI) += qib_iba6120.o

ib_qib-$(CONFIG_X86_64) += qib_wc_x86_64.o
ib_qib-$(CONFIG_PPC64) += qib_wc_ppc64.o
ib_qib-$(CONFIG_DEBUG_FS) += qib_debugfs.o

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

EOF

echo "Creating Makefile ($tardir/ib_mad/Makefile)"
cat > $tardir/ib_mad/Makefile <<'EOF'
#
# Ib_mad module
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

obj-$(CONFIG_INFINIBAND) += ib_mad.o

ib_mad-y := mad.o smi.o agent.o mad_rmpp.o

else
#normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KDIR) M=$$PWD modules_install

endif

EOF

DEFAULT_KERNEL_VERSION=$(uname -r)

if [ "$DEFAULT_KERNEL_VERSION" == "" ]; then
	echo "Unable to generate the kernel version"
	exit 1
fi

rpmrelease=1514
rpmrelease+="cuda"
echo "rpmrelease = $rpmrelease"

echo "Setting up RPM build area"
mkdir -p rpmbuild/{BUILD,RPMS,SOURCES,SPECS,SRPMS}

# make sure rpm component strings are clean, should be no-ops
rpmname=$(echo "$rpmname" | sed -e 's/[.]/_/g')
rpmversion=$(echo "$DEFAULT_KERNEL_VERSION" | sed -e 's/-/_/g')
rpmrequires=$(echo "$DEFAULT_KERNEL_VERSION" | sed -e 's/.[^.]*$//')

# get kernel(-devel) rpm version and release values
kernel_rpmver=$(rpm -q --qf %{VERSION} kernel-default)

# create a new $rpmname.conf and $rpmname.files
src_path=$workdir/rpmbuild/SOURCES/

# prepare files list and depmod config for every module built
echo "%defattr(644,root,root,755)" > $src_path/$rpmname.files
kmod_subdir=updates
modlist=""
for (( i = 0 ; i < modules_cnt ; i++ ))
do
	echo "override ${modules[$i]} $kernel_rpmver-* weak-updates/${modules[$i]}" >> $src_path/$rpmname.conf
        echo "/lib/modules/%{kver}/$kmod_subdir/$rpmname/${modules[$i]}.ko" >> $src_path/$rpmname.files
	modlist+=" ${modules[$i]}"
done
echo "/etc/depmod.d/$rpmname.conf" >> $src_path/$rpmname.files

# build the tarball
echo "Copy the working files from $srcdir/$kerneldir"
echo "Copy the working files to $tardir"
pushd $srcdir/$kerneldir
for (( i = 0 ; i < modules_cnt ; i++ ))
do
	cp ${files_to_copy[$i]} $tardir/${modules[$i]}/
done
echo "Copying header files"
for (( i = 0 ; i < include_dirs_cnt ; i++ ))
do
        mkdir -p $tardir/${include_dirs[$i]}
        cp ${include_files_to_copy[$i]} $tardir/${include_dirs[$i]}/
done
cp $srcdir/$kerneldir/LICENSE $tardir/.
popd
echo "Building tar file"
(cd $tardir; tar cfz - --transform="s,^,${rpmname}-${rpmversion}/," *) > \
	rpmbuild/SOURCES/$rpmname-$rpmversion.tgz
cd $workdir

# create the spec file
echo "Creating spec file"
cat > rpmbuild/SPECS/$rpmname.spec <<EOF
%{!?kver: %global kver %(uname -r)}
%define kdir /lib/modules/%{kver}/source

Name:           $rpmname
Group:		System Environment/Kernel
Summary:        Extra kernel modules for IFS
Version:        %(echo %{kver}|sed -e 's/-/_/g')
Release:        $rpmrelease
License:        GPLv2
Source0:        %{name}-$rpmversion.tgz
Source1:        %{name}.files
Source2:        %{name}.conf
BuildRoot:      %{_tmppath}/%{name}-%{version}-%{release}-root
BuildRequires:  %kernel_module_package_buildreqs

Requires:       kernel = %(echo %{kver} | sed -e 's/\(.*\).default/\1/g')

%kernel_module_package -f %{SOURCE1} default

%global kernel_source %(
        echo "/lib/modules/%{kver}/build"
)

%global kernelrpm_ver %(
	echo "%(rpm -q --qf %{VERSION} %(rpm -qf /lib/modules/%{kver}/kernel/))"
)

# find our target version
%global kbuild %(
if [ -z "\$kbuild" ]; then
	echo "/lib/modules/%{kver}/build"
else
	echo "\$kbuild"
fi
)

%global kver %(
if [ -f "%{kbuild}/include/config/kernel.release" ]; then
	cat %{kbuild}/include/config/kernel.release
else
	echo "fail"
fi
)

%define modlist $modlist
%define kmod_moddir %kernel_module_package_moddir

%description
Updated kernel modules for OPA IFS

%package devel
Summary: Development headers for Intel HFI1 driver interface
Group: System Environment/Development

%description devel
Development header files for Intel HFI1 driver interface

%prep
%setup -qn %{name}-$rpmversion
rm -rf %{SOURCE2}
for flavor in %flavors_to_build; do
        for mod in %modlist; do
                rm -rf "\$mod"_\$flavor
                cp -r \$mod  "\$mod"_\$flavor
		echo "override \$mod %{kernelrpm_ver}-* weak-updates/\$mod" >> %{SOURCE2}
        done
done

%build
if [ "%kver" = "fail" ]; then
        if [ -z "%kbuild" ]; then
                echo "The default target kernel, %kver, is not installed" >&2
                echo "To build, set \\\$kbuild to your target kernel build directory" >&2
        else
                echo "Cannot find kernel version in %kbuild" >&2
        fi
        exit 1
fi
echo "Kernel version is %kver"
echo "Kernel source directory is \"%kbuild\""

# Build
#make CONFIG_INFINIBAND_RDMAVT=m
for flavor in %flavors_to_build; do
        for mod in %modlist; do
                rm -rf \$mod
                cp -r "\$mod"_\$flavor \$mod
                done
        echo rpm kernel_source %{kernel_source \$flavor}
        make CONFIG_INFINIBAND_RDMAVT=m CONFIG_INFINIBAND_HFI1=m CONFIG_INFINIBAND_QIB=m KDIR=%{kernel_source \$flavor}
        for mod in %modlist; do
                rm -rf "\$mod"_\$flavor
                mv -f \$mod "\$mod"_\$flavor
        done
done

%install
install -m 644 -D %{SOURCE2} \$RPM_BUILD_ROOT/etc/depmod.d/%{name}.conf
for flavor in %flavors_to_build ; do
        mkdir -p \$RPM_BUILD_ROOT/lib/modules/%kver/%kmod_moddir/%{name}
        for mod in %modlist; do
#		make -C /usr/src/linux-obj/%_target_cpu/\$flavor modules_install M=$PWD/"\$mod"_\$flavor
		if [[ "$KERNEL_MOD_SIGNING_ENABLED" == "1" ]]; then
			RPM_KMOD_DIR=\$RPM_BUILD_ROOT/../../KMODS
			if [ -d \$RPM_KMOD_DIR ]; then
				install -m 644 -t \$RPM_BUILD_ROOT/lib/modules/%kver/%kmod_moddir/%{name} \$RPM_BUILD_ROOT/../../KMODS/"\$mod".ko
			else
				echo "WARNING: Installing unsigned kernel module: \$mod.ko"
				install -m 644 -t \$RPM_BUILD_ROOT/lib/modules/%kver/%kmod_moddir/%{name} "\$mod"_\$flavor/"\$mod".ko
			fi
		else
			install -m 644 -t \$RPM_BUILD_ROOT/lib/modules/%kver/%kmod_moddir/%{name} "\$mod"_\$flavor/"\$mod".ko
		fi
        done
done
targetdir=\$RPM_BUILD_ROOT%{_includedir}/uapi/rdma/hfi/
rtargetdir=\$RPM_BUILD_ROOT%{_includedir}/uapi/rdma/
mkdir -p \$targetdir
(srcdir=\$(pwd)
cd %kdir
# build 'unifdef' if binary does not exist
if [[ ! -f %kdir/scripts/unifdef ]]; then
	if [[ \$(id -u) = 0 ]]; then
		cd scripts
		/usr/bin/gcc unifdef.c -o unifdef
		cd ../
	fi
fi
sh ./scripts/headers_install.sh \$targetdir \$srcdir include/rdma/hfi/hfi1_user.h
sh ./scripts/headers_install.sh \$targetdir \$srcdir include/rdma/hfi/hfi1_ioctl.h
sh ./scripts/headers_install.sh \$rtargetdir \$srcdir include/rdma/rdma_user_ioctl.h
)

%files devel
%defattr(-,root,root,-)
%dir %{_includedir}/uapi/rdma
%dir %{_includedir}/uapi/rdma/hfi
%{_includedir}/uapi/rdma/hfi/hfi1_user.h
%{_includedir}/uapi/rdma/hfi/hfi1_ioctl.h
%{_includedir}/uapi/rdma/rdma_user_ioctl.h

%changelog
* Thu Nov 09 2017 Dennis Dalessandro <dennis.dalessandro@intel.com>
- Support latest upstream uAPI header scheme
* Fri Jun 9 2017 Harish Chegondi <harish.chegondi@intel.com>
- Change the path of the hfi1_user.h header so it doesn't conflict with the distro supplied one.
* Sat Sep 24 2016 Alex Estrin <alex.estrin@intel.com>
- Add KMP format spec. Build fix. Minor fix for License string.

EOF
# moment of truth, run rpmbuild
rm -rf ksrc
echo "Building SRPM"
cd rpmbuild
rpmbuild -bs --define "_topdir $(pwd)" SPECS/${rpmname}.spec
ret=$?

exit $ret
