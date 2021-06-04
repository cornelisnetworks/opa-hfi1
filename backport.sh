#!/bin/bash

commitish=$1
buildhost=$2

echo "Backporting $commitish for IFS"

rm -rf tmp

if ls *.patch 1> /dev/null 2>&1; then
	echo "Patch files exist please remove."
	exit 1
else
	git format-patch $commitish
fi

echo ""
for i in `ls *.patch`; do
	echo "Backporting: $i"

	# Apply just the commit message
	echo "Creating commit message"
	git am --exclude=* $i

	cp $i ${i}.orig
	cp $i ${i}.rhel
	cp $i ${i}.sles

	echo "Patching non-distro specific stuff first"
	git apply -3 --exclude=drivers/infiniband/ulp/ipoib/* ${i}.orig
	if [[ $? -ne 0 ]]; then
		echo "Patch did not apply cleanly"
		echo "Resolve conflicts and press ENTER to continue"
		echo "Do not commit, just git add the file"
		read blah
	fi
	echo ""

	# Now look for any drivers/infiniband/ulp and patch only the RHEL stuff
	sed -i 's/drivers\/infiniband\/ulp\/ipoib/distro\/RHEL76\/ipoib/g' ${i}.rhel
	echo "Patching RHEL76"
	git apply -3 --include=distro/RHEL76/ipoib/* --exclude=* ${i}.rhel
	if [[ $? -ne 0 ]]; then
		echo "Patch did not apply cleanly"
		echo "Resolve conflicts and press ENTER to continue"
		echo "Do not commit, just git add the file"
		read blah
	fi
	echo ""

	# Now look for any drivers/infiniband/ulp and patch only the SLES stuff
	sed -i 's/drivers\/infiniband\/ulp\/ipoib/distro\/SLES12SP4\/ipoib/g' ${i}.sles
	echo "Patching SLES12SP4"
	git apply -3 --include=distro/SLES12SP4/ipoib/* --exclude=* ${i}.sles
	if [[ $? -ne 0 ]]; then
		echo "Patch did not apply cleanly"
		echo "Resolve conflicts and press ENTER to continue"
		echo "Do not commit, just git add the file"
		read blah
	fi

	echo "Applied full patch checking for new files"

	git status | grep "new file:"
	if [[ $? -eq 0 ]]; then
		echo "New files found. Go add them to the build script."
		echo "Pres ENTER when ready."
		read blah
	fi

	echo ""

	if [[ ! -z $buildhost ]]; then
		echo "Do a test build on $buildhost:"
		ssh $buildhost "cd $PWD && ./do-update-makerpm.sh -S ${PWD} -w ${PWD}/tmp && cd tmp/rpmbuild && rpmbuild --rebuild --define \"_topdir $PWD/tmp/rpmbuild\" --nodeps SRPMS/*.src.rpm 2>&1" | tee build.log
		if [[ $? -ne 0 ]]; then
			echo "Build failed!"
			echo "Fix up and run \"git commit\" to commit changes."
			echo "Press ENTER to continue once you have tested the build."
			read blah
			rm -rf tmp
		else
			#compiled now check for build warnings
			echo "Looking for build warnings that are not fatal"
			
			# ignore the existing qib one
			grep -v "qib_fs.c" build.log | grep -i "warning:"
			if [[ $? -eq 0 ]]; then
				echo "Build warnings found!"
				echo "Fix up and run \"git commit\" to commit changes."
				echo "Press ENTER to continue once you have tested the build."
				read blah
				rm -rf tmp
			else
				echo "Build completed. Press ENTER to remove tmp dir/symlink and continue"
				read blah
				rm -rf tmp
			fi
		fi
	else
		echo "No build host specified, skipping incremental build"
	fi

	echo "Removing patch files associated with $i"
	rm $i ${i}.orig ${i}.rhel ${i}.sles

	# Commit it and sign it
	git commit --amend -s

done

