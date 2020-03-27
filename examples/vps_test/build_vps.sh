#!/bin/bash
##Current Directory : topdir/examples/vps_test
cd ../../
topdir=`pwd`

cd ${topdir}/examples/vps_test
mkdir -p build
cd build
cmake ..
make install

cd "$topdir/bin"
ln -sf ../src/vps/data_vps
