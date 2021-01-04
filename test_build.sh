build_cmd="cd $PWD && ./build_it.sh "

rhel7_5="ph-bld-node-26"
rhel7_6="awfm-01"
rhel7_7="aw-bld-node-02"
rhel7_8="aw-bld-node-11"
rhel8_0="aw-bld-node-38"
rhel8_1="aw-bld-node-12"
rhel8_2="aw-bld-node-30"
sles12_3="ph-bld-node-22"
sles12_4="aw-bld-node-36"
sles12_5="aw-bld-node-01"
sles15_0="aw-bld-node-28"
sles15_1="aw-bld-node-40"
sles15_2="aw-bld-node-20"

echo "Clearing build files"
rm *.build

echo "Launching RHEL 7.5 build..."
ssh $rhel7_5 "$build_cmd" > rhel75.build 2>&1 &

echo "Launching RHEL 7.6 build..."
ssh $rhel7_6 "$build_cmd" > rhel76.build 2>&1 &

echo "Launching RHEL 7.7 build..."
ssh $rhel7_7 "$build_cmd" > rhel77.build 2>&1 &

echo "Launching RHEL 7.8 build..."
ssh $rhel7_8 "$build_cmd" > rhel78.build 2>&1 &

echo "Launching RHEL 8.0 build..."
ssh $rhel8_0 "$build_cmd" > rhel80.build 2>&1 &

echo "Launching RHEL 8.1 build..."
ssh $rhel8_1 "$build_cmd" > rhel81.build 2>&1 &

echo "Launching RHEL 8.2 build..."
ssh $rhel8_2 "$build_cmd" > rhel82.build 2>&1 &

echo "Launching SLES 12.3 build..."
ssh $sles12_3 "$build_cmd" > sles123.build 2>&1 &

echo "Launching SLES 12.4 build..."
ssh $sles12_4 "$build_cmd" > sles124.build 2>&1 &

echo "Launching SLES 12.5 build..."
ssh $sles12_5 "$build_cmd" > sles125.build 2>&1 &

echo "Launching SLES 15.0 build..."
ssh $sles15_0 "$build_cmd" > sles150.build 2>&1 &

echo "Launching SLES 15.1 build..."
ssh $sles15_1 "$build_cmd" > sles151.build 2>&1 &

echo "Launching SLES 15.2 build..."
ssh $sles15_2 "$build_cmd" > sles152.build 2>&1 &

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

