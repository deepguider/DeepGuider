#!/bin/bash
##Current Directory : topdir/examples/vps_test
cd ../../
topdir=`pwd`

echo "####### Compile ..... #######"
cd ${topdir}/examples/python_test
mkdir -p build
cd build
cmake ..
make install

echo "####### Run     ..... #######"
cd ${topdir}/examples/python_test
source ~/.virtualenvs/dg_venv3.6/bin/activate
#gdb ./python_test
./python_test
