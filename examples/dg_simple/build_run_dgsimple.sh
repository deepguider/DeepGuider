#!/bin/bash

##Current Directory : topdir/examples/dg_simple
echo "####### Compile ..... #######"
mkdir -p build
cd build
cmake ..
make -j4 install
if [ $? -ne 0 ];then # If it met error in previous step
    #echo $?
    cd ..
    exit 0
fi

echo "####### Run ..... #######"
cd ../../../bin

#gdb ./dg_simple #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
#kdbg ./dg_simple #for debugging in graphic mode
#./dg_simple # for running without debugging

cd ../examples/dg_simple   # return to initial directory
