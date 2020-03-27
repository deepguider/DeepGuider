#!/bin/bash
##Current Directory : topdir/examples/map_manager_test
cd ../../
topdir=`pwd`

cd ${topdir}/examples/map_manager_test
mkdir -p build
cd build
cmake ..
make install

if [ $? -ne 0 ];then # If it met error in previous step
	#echo $?
	exit 0
fi
