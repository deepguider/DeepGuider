#!/bin/bash
##Current Directory : topdir/examples/vps_test
cd ../../
topdir=`pwd`

echo "####### Compile ..... #######"
cd ${topdir}/examples/vps_test
mkdir -p build
cd build
cmake ..
make install

if [ $? -ne 0 ];then # If it met error in previous step
	#echo $?
	exit 0
fi

echo "####### Run  ..... #######"
cd "$topdir/bin"
ln -sf ../src/vps/data_vps
hostname |grep ccsmm && ln -sf ~/Naverlabs/query_etri_cart/191115_ETRI.avi data/
hostname |grep ccsmm && source ~/.virtualenvs/dg_venv3.6/bin/activate || echo "You need to enter VirtualEnv"

#source ~/.virtualenvs/5env3.5/bin/activate
#gdb ./vps_test
#kdbg ./vps_test
./vps_test
