#!/bin/bash
##Current Directory : topdir/examples/exploration_test
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
ln -sf ../src/exploration/data_exp

