#!/bin/bash
##Current Directory : topdir/examples/guidance_test
cd ../../
topdir=`pwd`
echo "####### Compile ..... #######"
cd ${topdir}/examples/guidance_test
mkdir -p build
cd build
cmake ..
make install

if [ $? -ne 0 ];then # If it met error in previous step
	#echo $?
	exit 0
fi

echo "####### Run     ..... #######"
cd ${topdir}/bin
./guidance_test

