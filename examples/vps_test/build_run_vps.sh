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

echo "####### Run vps_test ..... #######"
cd "$topdir/bin"
ln -sf ../src/vps/data_vps
#ln -fs ../../src/vps/netvlad/ccsmmutils/img_utils.py
#ln -fs ../../src/vps/netvlad
#ln -fs ../../src/vps/netvlad_etri_datasets
source ~/.virtualenvs/dg_venv3.6/bin/activate
#source ~/.virtualenvs/5env3.5/bin/activate
#gdb ./vps_test
#kdbg ./vps_test
./vps_test
