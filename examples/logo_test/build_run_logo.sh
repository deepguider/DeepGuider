#!/bin/bash
##Current Directory : topdir/examples/logo_test
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
ln -sf ../src/logo_recog/logo_data
ln -sf ../src/logo_recog/model
#gdb ./logo_test #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
#kdbg ./logo_test #for debugging in graphic mode
./logo_test # for running without debugging

cd ../examples/logo_test   # return to initial directory