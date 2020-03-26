#!/bin/bash
##Current Directory : topdir/examples/poi_recog_test
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
ln -sf ../src/poi_recog/model
cd data
ln -sf ../../src/poi_recog/data/test
ln -sf ../../src/poi_recog/data/preprocessed
ln -sf ../../src/poi_recog/data/litw_annotation.py
cd ..

#gdb ./poi_recog_test #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
#kdbg ./poi_recog_test #for debugging in graphic mode
./poi_recog_test # for running without debugging

cd ../examples/poi_recog_test   # return to initial directory