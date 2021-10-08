#!/bin/sh
build_cmd="cd $PWD && ./build_it.sh "
#rhel7_5="phkpstl093"
#rhel7_6="awfm-01"

rhel7_7="bld-node-02"
rhel7_7_gpu="/usr/src/nvidia-460.27.04"

rhel7_8="bld-node-11"
rhel7_8_gpu="/usr/src/nvidia-460.27.04"

#rhel8_0="bld-node-38"
rhel8_1="bld-node-12"
rhel8_1_gpu="/usr/src/nvidia-460.27.04"

rhel8_2="bld-node-30"
rhel8_2_gpu="/usr/src/nvidia-460.27.04"

rhel8_3="bld-node-26"
rhel8_3_gpu="/usr/src/nvidia-460.27.04"

#sles12_3="ph-bld-node-22"

sles12_4="bld-node-37"
sles12_4_gpu="/usr/src/kernel-modules/nvidia-440.33.01-default"

sles12_5="bld-node-01"
sles12_5_gpu="/usr/src/kernel-modules/nvidia-440.64.00-default"

#sles15_0="aw-bld-node-28"

sles15_1="bld-node-40"
sles15_1_gpu="/usr/src/kernel-modules/nvidia-440.33.01-default"

sles15_2="bld-node-20"
sles15_2_gpu="/usr/src/kernel-modules/nvidia-450.36.06-default"

run_build()
# $1 - type
# $2 - server
# $3 - GPU dir
# $4 - log name
{
	local type=$1
	local server=$2
	local nvdir=$3
	local logname=$4

	if [[ $type == "gpu" ]]; then
		ssh $server "export NVIDIA_GPU_DIRECT=$nvdir;$build_cmd" > ${logname}_gpu.build 2>&1 &
	else
		ssh $server "$build_cmd" > ${logname}.build 2>&1 &
	fi
}

if [[ $1 == "test" ]]; then
	echo "Testing each host to make sure we can connect and NFS is mounted"
	#echo "Checking RHEL 7.5" &&
	#ssh $rhel7_5 "ls $PWD" &&
	#echo "Checking RHEL 7.6" &&
	#ssh $rhel7_6 "ls $PWD" &&
	echo "Checking RHEL 7.7" &&
	ssh $rhel7_7 "ls $PWD" &&
	echo "Checking RHEL 7.8" &&
	ssh $rhel7_8 "ls $PWD" &&
	#echo "Checking RHEL 8.0" &&
	#ssh $rhel8_0 "ls $PWD" &&
	echo "Checking RHEL 8.1" &&
	ssh $rhel8_1 "ls $PWD" &&
	echo "Checking RHEL 8.2" &&
	ssh $rhel8_2 "ls $PWD" &&
	echo "Checking RHEL 8.3" &&
	ssh $rhel8_3 "ls $PWD" &&
	#echo "Checking SLES 12.3" &&
	#ssh $sles12_3 "ls $PWD" &&
	echo "Checking SLES 12.4" &&
	ssh $sles12_4 "ls $PWD" &&
	echo "Checking SLES 12.5" &&
	ssh $sles12_5 "ls $PWD" &&
	#echo "Checking SLES 15.0" &&
	#ssh $sles15_0 "ls $PWD" &&
	echo "Checking SLES 15.1" &&
	ssh $sles15_1 "ls $PWD" &&
	echo "Checking SLES 15.2" &&
	ssh $sles15_2 "ls $PWD" 
	exit 0
fi

echo "Clearing build files"
rm *.build

#echo "Launching RHEL 7.5 build..."
#ssh $rhel7_5 "$build_cmd" > rhel75.build 2>&1 &

#echo "Launching RHEL 7.6 build..."
#ssh $rhel7_6 "$build_cmd" > rhel76.build 2>&1 &

echo "Launching RHEL 7.7 build..."
run_build "$1" $rhel7_7 $rhel7_7_gpu rhel77

echo "Launching RHEL 7.8 build..."
run_build "$1" $rhel7_8 $rhel7_8_gpu rhel78

#echo "Launching RHEL 8.0 build..."
#ssh $rhel8_0 "$build_cmd" > rhel80.build 2>&1 &

echo "Launching RHEL 8.1 build..."
run_build "$1" $rhel8_1 $rhel8_1_gpu rhel81

echo "Launching RHEL 8.2 build..."
run_build "$1" $rhel8_2 $rhel8_2_gpu rhel82

echo "Launching RHEL 8.3 build..."
run_build "$1" $rhel8_3 $rhel8_3_gpu rhel83

#echo "Launching SLES 12.3 build..."
#ssh $sles12_3 "$build_cmd" > sles123.build 2>&1 &

echo "Launching SLES 12.4 build..."
run_build "$1" $sles12_4 $sles12_4_gpu sles124

echo "Launching SLES 12.5 build..."
run_build "$1" $sles12_5 $sles12_5_gpu sles125

#echo "Launching SLES 15.0 build..."
#ssh $sles15_0 "$build_cmd" > sles150.build 2>&1 &

echo "Launching SLES 15.1 build..."
run_build "$1" $sles15_1 $sles15_1_gpu sles151

echo "Launching SLES 15.2 build..."
run_build "$1" $sles15_2 $sles15_2_gpu sles152

echo "Waiting for builds to finish"
wait

echo "--------------------"
echo "Checking for Errors:"
echo "--------------------"
grep -n "RPM build errors:" *.build
grep "WARNING:" *.build
echo ""
echo "-------------"
echo "Checking RPMS"
echo "-------------"
echo "Created the following RPMS successfully:"
#RHEL
grep kmod-ifs-kernel-updates *.build | grep Wrote | grep -v debuginfo
#SLES
grep Wrote *build | grep ifs-kernel-updates-kmp

