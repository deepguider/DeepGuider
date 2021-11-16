#!/bin/bash
##Current Directory : topdir/examples/roadlr_test
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
PWD=`pwd`
cd ../../../bin

#gdb ./roadlr_test #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
#kdbg ./roadlr_test #for debugging in graphic mode
./roadlr_test # for running without debugging

cd $PWD   # return to initial directory
