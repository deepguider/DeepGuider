#!/bin/bash

topdir=`pwd`

echo "####### Compile ..... #######"
cd examples/python_test
mkdir -p build
cd build
cmake ..
make install
cd "$topdir/examples/python_test"

echo "####### Run vps_test ..... #######"
source ~/.virtualenvs/dg_venv3.6/bin/activate
./python_test
