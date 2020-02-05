#!/bin/bash

topdir=`pwd`

echo "####### Compile ..... #######"
cd examples/vps_test
mkdir -p build
cd build
cmake ..
make install

echo "####### Run vps_test ..... #######"
cd "$topdir/bin"
ln -fs ../src/vps/netvlad/ccsmmutils/img_utils.py
ln -fs ../src/vps/netvlad
ln -fs ../src/vps/netvlad_etri_datasets
source ~/.virtualenvs/dg_venv3.6/bin/activate
./vps_test
