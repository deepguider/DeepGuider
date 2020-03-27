#!/bin/bash
##Current Directory : topdir/examples/vps_test
cd ../../
topdir=`pwd`

echo "####### Compile ..... #######"
cd ${topdir}/examples/map_manager_test
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
./map_manager_test
#cd ${topdir}/examples/python_test
#source ~/.virtualenvs/dg_venv3.6/bin/activate
#gdb ./python_test
#./python_test
