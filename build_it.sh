#!/bin/bash

rm -rf /tmp/oot-$USER

mkdir /tmp/oot-$USER

cp -r compat /tmp/oot-$USER
cp -r distro /tmp/oot-$USER
cp -r drivers /tmp/oot-$USER
cp -r files /tmp/oot-$USER
cp -r include *.sh LICENSE modules.* Module.symvers System.map /tmp/oot-$USER

cd /tmp/oot-$USER

ls

./do-update-makerpm.sh -S ${PWD} -w ${PWD}/tmp && cd tmp/rpmbuild  && rpmbuild --rebuild --define "_topdir $(pwd)" --nodeps SRPMS/*.src.rpm
