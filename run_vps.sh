#!/bin/bash

topdir=`pwd`

echo "####### Compile ..... #######"
cd examples/vps_test
mkdir -p build
cd build
cmake ..
make install
cd "$topdir/bin"
ln -fs ../src/vps/netvlad/ccsmmutils/img_utils.py
ln -fs ../src/vps/netvlad
ln -fs ../src/vps/netvlad_etri_datasets

echo "####### Run vps_test ..... #######"
source ~/.virtualenvs/dg_venv3.6/bin/activate
./vps_test
