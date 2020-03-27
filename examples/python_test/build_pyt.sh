#!/bin/bash
##Current Directory : topdir/examples/python_test
cd ../../
topdir=`pwd`

cd ${topdir}/examples/python_test
mkdir -p build
cd build
cmake ..
make install
