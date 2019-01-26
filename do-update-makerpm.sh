#!/bin/bash

DEFAULT_KERNEL_VERSION=""
kerneldir="./"

declare -A distros=( ["7.4"]="IFS_RH74" ["7.5"]="IFS_RH75" ["7.6"]="IFS_RH76" ["12.2"]="IFS_SLES12SP2" ["12.3"]="IFS_SLES12SP3" ["12.4"]="IFS_SLES12SP4" ["15"]="IFS_SLES15" )
declare -A distro_directory=(["7.4"]="RH74" ["7.5"]="RH75" ["7.6"]="RH76" ["12.2"]="SLES12SP2" ["12.3"]="SLES12SP3" ["12.4"]="SLES12SP4" ["15"]="SLES15" )

modules_cnt=0

# Add each module separately
modules[$modules_cnt]="rdmavt"
files_to_copy[$modules_cnt]="
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
"

((modules_cnt++))
modules[$modules_cnt]="hfi1"
files_to_copy[$modules_cnt]="
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
	drivers/infiniband/hw/hfi1/firmware.c
	drivers/infiniband/hw/hfi1/hfi.h
	drivers/infiniband/hw/hfi1/ipoib.h
	drivers/infiniband/hw/hfi1/ipoib_main.c
	drivers/infiniband/hw/hfi1/ipoib_tx.c
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
	drivers/infiniband/hw/hfi1/uc.c
	drivers/infiniband/hw/hfi1/ud.c
	drivers/infiniband/hw/hfi1/user_exp_rcv.c
	drivers/infiniband/hw/hfi1/user_exp_rcv.h
	drivers/infiniband/hw/hfi1/user_pages.c
	drivers/infiniband/hw/hfi1/user_sdma.c
	drivers/infiniband/hw/hfi1/user_sdma.h
	drivers/infiniband/hw/hfi1/verbs.c
	drivers/infiniband/hw/hfi1/verbs.h
	drivers/infiniband/hw/hfi1/verbs_txreq.c
	drivers/infiniband/hw/hfi1/verbs_txreq.h
	drivers/infiniband/hw/hfi1/vnic.h
	drivers/infiniband/hw/hfi1/exp_rcv.c
	drivers/infiniband/hw/hfi1/exp_rcv.h
	drivers/infiniband/hw/hfi1/diag.c
	drivers/infiniband/hw/hfi1/iowait.c
	drivers/infiniband/hw/hfi1/opfn.c
	drivers/infiniband/hw/hfi1/opfn.h
	drivers/infiniband/hw/hfi1/rc.h
	drivers/infiniband/hw/hfi1/tid_rdma.c
	drivers/infiniband/hw/hfi1/tid_rdma.h
	drivers/infiniband/hw/hfi1/trace_iowait.h
	drivers/infiniband/hw/hfi1/trace_tid.h
	drivers/infiniband/hw/hfi1/fault.c
	drivers/infiniband/hw/hfi1/fault.h
	drivers/infiniband/hw/hfi1/rcva.c
	drivers/infiniband/hw/hfi1/rcva.h
	drivers/infiniband/hw/hfi1/msix.c
	drivers/infiniband/hw/hfi1/msix.h
	drivers/infiniband/hw/hfi1/netdev_rx.c
	drivers/infiniband/hw/hfi1/ipoib_rx.c
	drivers/infiniband/hw/hfi1/netdev.h
"

((modules_cnt++))
modules[$modules_cnt]="ib_qib"
files_to_copy[$modules_cnt]="
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

include_dirs[0]="include/rdma"
include_dirs[1]="include/rdma/hfi"
include_dirs[2]="compat/"
include_files_to_copy[0]="
	include/rdma/rdmavt_qp.h
	include/rdma/rdmavt_mr.h
	include/rdma/rdmavt_cq.h
	include/rdma/rdma_vt.h
	include/rdma/ib_hdrs.h
	include/rdma/opa_vnic.h
	include/rdma/opa_addr.h
	include/rdma/tid_rdma_defs.h
	include/uapi/rdma/rdma_user_ioctl.h
"
include_files_to_copy[1]="
	include/uapi/rdma/hfi/hfi1_user.h
	include/uapi/rdma/hfi/hfi1_ioctl.h
"
include_files_to_copy[2]=""

include_dirs_cnt=${#include_dirs[@]}

# ridiculously long to encourage good names later
rpmname="ifs-kernel-updates"

set -e

if [[ -e /etc/os-release ]]; then
	. /etc/os-release
else
	echo "File /etc/os-release is missing."
	exit 1
fi

echo "VERSION_ID = $VERSION_ID"
echo "PRETTY_NAME = $PRETTY_NAME"

function usage
{
	cat <<EOL
usage:
	${0##*/} -h
	${0##*/} [-V] [-G] [-w dirname]
	${0##*/} -S srcdir [-w dirname]

Options:

-V         - Enable building VNIC in the package
-G         - Enable building a GPU Direct package
-A	   - Force enable building Accelerated IPoIB package
-S srcdir  - fetch source directly from a specified directory

-w dirname - work directory, defaults to a mktemp directory
-h         - this help text
EOL
}

vnicbuild="false"
gpubuild="false"
aipbuild="false"
srcdir=""
workdir=""
filedir=""
ifs_distro=""
distro=""
distro_dir=""
while getopts ":GVAS:hu:w:" opt; do
    	case "$opt" in
	G)	gpubuild="true"
		;;
	V)      vnicbuild="true"
		;;
	S)	srcdir="$OPTARG"
		[ ! -e "$srcdir" ] && echo "srcdir $srcdir not found" && exit 1
		srcdir=$(readlink -f "$srcdir")
		;;
	A)	aipbuild="true"
		;;
	h)	usage
		exit 0
		;;
	w)	workdir="$OPTARG"
		;;

    	esac
done

rm -f drivers/infiniband/ulp/ipoib

if [[ $ID == "rhel" ]]; then
	if [[ $VERSION_ID == "7.6" ]]; then
		ln -s $PWD/distro/RHEL76/ipoib drivers/infiniband/ulp/ipoib
		aipbuild="true"
	fi
	if [[ $VERSION_ID == "7.5" ]]; then
		ln -s $PWD/distro/RHEL76/ipoib drivers/infiniband/ulp/ipoib
		aipbuild="true"
	fi
elif [[ $ID == "sles" ]]; then
	if [[ $VERSION_ID == "12.4" ]]; then
		ln -s $PWD/distro/SLES12SP4/ipoib drivers/infiniband/ulp/ipoib
		aipbuild="true"
	fi

	if [[ $VERSION_ID == "15" ]]; then
		ln -s $PWD/distro/SLES12SP4/ipoib drivers/infiniband/ulp/ipoib
		aipbuild="true"
	fi
fi


if [ $gpubuild = 'true' ]; then
	echo "GPU Direct enabled build"
fi

# create final version of the variables
if [ -n "$workdir" ]; then
	mkdir -p "$workdir" || exit 1
else
	workdir=$(mktemp -d --tmpdir=$(pwd) build.XXXX)
	[ ! $? ] && exit 1
fi

ifs_distro="${distros[$VERSION_ID]}"
distro_dir="${distro_directory[$VERSION_ID]}"
distro=$ID

echo "ifs_distro = $ifs_distro"
echo "distro_dir = $distro_dir"
echo "distro = $distro"

files_to_copy[0]+="
	compat/$distro_dir/compat.c
	compat/common/compat_common.c
	"

include_files_to_copy[2]="
	compat/$distro_dir/compat.h
	compat/common/compat_common.h
	"

if [ $VERSION_ID = '12.2' ]
then

((modules_cnt++))
modules[$modules_cnt]="ib_mad"
files_to_copy[$modules_cnt]="
	distro/$distro_dir/core/agent.c
	distro/$distro_dir/core/agent.h
	distro/$distro_dir/core/mad.c
	distro/$distro_dir/core/mad_priv.h
	distro/$distro_dir/core/mad_rmpp.c
	distro/$distro_dir/core/mad_rmpp.h
	distro/$distro_dir/core/opa_smi.h
	distro/$distro_dir/core/smi.c
	distro/$distro_dir/core/smi.h
	"
fi

if [ $gpubuild = 'true' ]; then
        files_to_copy[1]+="
	drivers/infiniband/hw/hfi1/gdr_ops.c
	drivers/infiniband/hw/hfi1/gdr_ops.h
	drivers/infiniband/hw/hfi1/trace_gpu.h
	drivers/infiniband/hw/hfi1/gpu.c
	drivers/infiniband/hw/hfi1/gpu.h
	drivers/infiniband/hw/hfi1/user_exp_rcv_gpu.c
	drivers/infiniband/hw/hfi1/user_exp_rcv_gpu.h
	drivers/infiniband/hw/hfi1/user_sdma_gpu.c
	drivers/infiniband/hw/hfi1/user_sdma_gpu.h
        "
fi

if [ $vnicbuild = 'true' ]; then
	files_to_copy[1]+="
	drivers/infiniband/hw/hfi1/vnic_main.c
	drivers/infiniband/hw/hfi1/vnic_sdma.c
	"

	((modules_cnt++))
	modules[$modules_cnt]="opa_vnic"
	files_to_copy[$modules_cnt]="
		drivers/infiniband/ulp/opa_vnic/opa_vnic_debugfs.c
		drivers/infiniband/ulp/opa_vnic/opa_vnic_debugfs.h
		drivers/infiniband/ulp/opa_vnic/opa_vnic_encap.c
		drivers/infiniband/ulp/opa_vnic/opa_vnic_encap.h
		drivers/infiniband/ulp/opa_vnic/opa_vnic_ethtool.c
		drivers/infiniband/ulp/opa_vnic/opa_vnic_internal.h
		drivers/infiniband/ulp/opa_vnic/opa_vnic_netdev.c
		drivers/infiniband/ulp/opa_vnic/opa_vnic_vema.c
		drivers/infiniband/ulp/opa_vnic/opa_vnic_vema_iface.c
		"

	patches_dir="distro/RHEL7"

fi

if [ $aipbuild = 'true' ]; then
	((modules_cnt++))
	modules[$modules_cnt]="ib_ipoib"
	files_to_copy[$modules_cnt]="
		drivers/infiniband/ulp/ipoib/ipoib_cm.c
		drivers/infiniband/ulp/ipoib/ipoib_ethtool.c
		drivers/infiniband/ulp/ipoib/ipoib_fs.c
		drivers/infiniband/ulp/ipoib/ipoib.h
		drivers/infiniband/ulp/ipoib/ipoib_ib.c
		drivers/infiniband/ulp/ipoib/ipoib_main.c
		drivers/infiniband/ulp/ipoib/ipoib_multicast.c
		drivers/infiniband/ulp/ipoib/ipoib_netlink.c
		drivers/infiniband/ulp/ipoib/ipoib_verbs.c
		drivers/infiniband/ulp/ipoib/ipoib_vlan.c
		"
fi
# configure the file dir
filedir=$srcdir/files

# after cd, where are we *really*
cd -P "$workdir"; workdir=$(pwd)
tardir=$workdir/stage
rm -rf $tardir
for (( i = 0 ; i <= modules_cnt ; i++ ))
do
	mkdir -p $tardir/${modules[$i]}
done

echo "Working in $workdir"

# create the Makefiles
echo "Creating Makefile ($tardir/Makefile)"

if [ $gpubuild = 'true' ]; then
cp $filedir/Makefile.top.gpu $tardir/Makefile
else
cp $filedir/Makefile.top $tardir/Makefile
fi

sed -i "s/IFS_DISTRO/$ifs_distro/g" $tardir/Makefile

echo "Creating Makefile ($tardir/rdmavt/Makefile)"
cp $filedir/Makefile.rdmavt $tardir/rdmavt/Makefile

if [ $VERSION_ID = '15' -o $VERSION_ID = '12.4' ]
then
	sed -i "s/compat.o//g" $tardir/rdmavt/Makefile
fi

echo "Creating Makefile ($tardir/hfi1/Makefile)"
if [ $gpubuild = 'true' ]; then
cp $filedir/Makefile.hfi.gpu $tardir/hfi1/Makefile
else
cp $filedir/Makefile.hfi $tardir/hfi1/Makefile
fi

echo "Creating Makefile ($tardir/ib_qib/Makefile)"
cp $filedir/Makefile.qib $tardir/ib_qib/Makefile

if [ $aipbuild = 'true' ]; then
	echo "Creating Makefile ($tardir/ipoib/Makefile)"
	cp $filedir/Makefile.ipoib $tardir/ib_ipoib/Makefile

	sed -i "s/ib_qib/ib_qib\/ ib_ipoib/g"  $tardir/Makefile
fi

if [ $VERSION_ID = '12.2' ]
then
	echo "Creating Makefile ($tardir/ib_mad/Makefile)"
	cp $filedir/Makefile.mad $tardir/ib_mad/Makefile

	sed -i "s/ib_qib/ib_qib\/ ib_mad/g"  $tardir/Makefile
fi

if [ $vnicbuild = 'true' ]; then
	echo "Creating Makefile ($tardir/Makefile)"
	cp $filedir/Makefile.vnic $tardir/opa_vnic/Makefile

	sed -i "s/ib_qib/ib_qib\/ opa_vnic/g"  $tardir/Makefile

	sed -i "s/iowait.o/iowait.o vnic_main.o vnic_sdma.o/g"  $tardir/hfi1/Makefile
fi

DEFAULT_KERNEL_VERSION=$(uname -r)

if [ "$DEFAULT_KERNEL_VERSION" == "" ]; then
	echo "Unable to generate the kernel version"
	exit 1
fi

rpmrelease=$(cd "$srcdir"; git rev-list "origin..HEAD" | wc -l)
rpmrelease=$((rpmrelease + 1754))
if [ $gpubuild = 'true' ]; then
	rpmrelease+="cuda"
fi
echo "rpmrelease = $rpmrelease"

echo "Setting up RPM build area"
mkdir -p rpmbuild/{BUILD,RPMS,SOURCES,SPECS,SRPMS}

# make sure rpm component strings are clean, should be no-ops
rpmname=$(echo "$rpmname" | sed -e 's/[.]/_/g')
rpmversion=$(echo "$DEFAULT_KERNEL_VERSION" | sed -e 's/-/_/g')
rpmrequires=$(echo "$DEFAULT_KERNEL_VERSION" | sed -e 's/.[^.]*$//')

# get kernel(-devel) rpm version and release values
if [ $distro = 'rhel' ]
then
	kernel_rpmver=$(rpm -q --qf %{VERSION} kernel-$(uname -r))
	kmod_subdir=extra
else
	kernel_rpmver=$(rpm -q --qf %{VERSION} kernel-default)
	kmod_subdir=updates
fi
# create a new $rpmname.conf and $rpmname.files
src_path=$workdir/rpmbuild/SOURCES/

# prepare files list and depmod config for every module built
echo "%defattr(644,root,root,755)" > $src_path/$rpmname.files

modlist=""
for (( i = 0 ; i <= modules_cnt ; i++ ))
do
        echo "override ${modules[$i]} $kernel_rpmver-* weak-updates/${modules[$i]}" >> $src_path/$rpmname.conf
        echo "/lib/modules/%2-%1/$kmod_subdir/$rpmname/${modules[$i]}.ko" >> $src_path/$rpmname.files
	modlist+=" ${modules[$i]}"
done
echo "/etc/depmod.d/$rpmname.conf" >> $src_path/$rpmname.files

# build the tarball
echo "Copy the working files from $srcdir/$kerneldir"
echo "Copy the working files to $tardir"
pushd $srcdir/$kerneldir
for (( i = 0 ; i <= modules_cnt ; i++ ))
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

if [ $distro = 'rhel' ]
then
	cp $filedir/$rpmname.spec.rhel $workdir/rpmbuild/SPECS/$rpmname.spec
else
	cp $filedir/$rpmname.spec.sles $workdir/rpmbuild/SPECS/$rpmname.spec
fi

sed -i "s/RPMNAME/$rpmname/g" $workdir/rpmbuild/SPECS/$rpmname.spec
sed -i "s/RPMRELEASE/$rpmrelease/g" $workdir/rpmbuild/SPECS/$rpmname.spec
sed -i "s/RPMVERSION/$rpmversion/g" $workdir/rpmbuild/SPECS/$rpmname.spec
sed -i "s/MODLIST/$modlist/g" $workdir/rpmbuild/SPECS/$rpmname.spec

# moment of truth, run rpmbuild
rm -rf ksrc
echo "Building SRPM"
cd rpmbuild
rpmbuild -bs --define "_topdir $(pwd)" SPECS/${rpmname}.spec
ret=$?

rm -f drivers/infiniband/ulp/ipoib

exit $ret
