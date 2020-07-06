#!/bin/bash
##Current Directory : topdir/examples/dg_simple
mkdir -p build
cd build
cmake ..
make install
if [ $? -ne 0 ];then # If it met error in previous step
    #echo $?
    cd ..
    exit 0
fi

cd ../../../bin
ln -sf ../src/vps/data_vps
ln -sf ../src/poi_recog/model
cd data
ln -sf ../../src/poi_recog/data/test
ln -sf ../../src/poi_recog/data/preprocessed
ln -sf ../../src/poi_recog/data/litw_annotation.py
