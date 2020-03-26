#!/bin/bash
##Current Directory : topdir/examples/dg_test
echo "####### Compile ..... #######"
mkdir -p build
cd build
cmake ..
make install
if [ $? -ne 0 ];then # If it met error in previous step
    #echo $?
    cd ..
    exit 0
fi

echo "####### Run ..... #######"
cd ../../../bin
ln -sf ../src/vps/data_vps
ln -sf ../src/poi_recog/model
cd data
ln -sf ../../src/poi_recog/data/test
ln -sf ../../src/poi_recog/data/preprocessed
ln -sf ../../src/poi_recog/data/litw_annotation.py
cd ..

#gdb ./dg_test #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
#kdbg ./dg_test #for debugging in graphic mode
./dg_test # for running without debugging

cd ../examples/dg_test   # return to initial directory