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
hostname |grep ccsmm && ln -sf ~/Naverlabs/query_etri_cart/191115_ETRI.avi data/
