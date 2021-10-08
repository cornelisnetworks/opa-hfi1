#!/bin/bash

if [ -n "$NVIDIA_GPU_DIRECT" ]; then
	bname=gpu
else
	bname=oot
fi

rm -rf /tmp/$bname-$USER

mkdir /tmp/$bname-$USER

cp -r compat /tmp/$bname-$USER
cp -r distro /tmp/$bname-$USER
cp -r drivers /tmp/$bname-$USER
cp -r files /tmp/$bname-$USER
cp -r include *.sh LICENSE modules.* Module.symvers System.map /tmp/$bname-$USER

cd /tmp/$bname-$USER

ls

if [ -n "$NVIDIA_GPU_DIRECT" ]; then
	./do-update-makerpm.sh -G -S ${PWD} -w ${PWD}/tmp && cd tmp/rpmbuild  && rpmbuild --rebuild --define "_topdir $(pwd)" --nodeps SRPMS/*.src.rpm
else
	./do-update-makerpm.sh -S ${PWD} -w ${PWD}/tmp && cd tmp/rpmbuild  && rpmbuild --rebuild --define "_topdir $(pwd)" --nodeps SRPMS/*.src.rpm
fi
