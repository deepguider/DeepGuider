#!/bin/bash
##Current Directory : topdir/examples/logo_test
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
ln -sf ../src/logo_recog/logo_data
ln -sf ../src/logo_recog/model

cd ../examples/logo_test   # return to initial directory