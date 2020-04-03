#!/bin/bash
##Current Directory : dgdir/examples/dg_test_ros

catkin_ws="catkin_ws_deepguider"
CWD=`pwd`

## Create catkin workspace
if [ ! -z ~/$catkin_ws ];then
    source /opt/ros/kinetic/setup.bash
    mkdir -p ~/$catkin_ws/src
    cd ~/$catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
	cd $CWD
fi

## Set rosdir
cd ~/$catkin_ws;rosdir=`pwd`;cd $CWD
#rosdir="/work/deepguider"  # set ros workspace directory here.
cp .dg_run.sh dg_run.sh
grep setup dg_run.sh || echo "source $rosdir/devel/setup.bash" | cat - dg_run.sh > .tmp && mv .tmp dg_run.sh
chmod +x dg_run.sh

## 
cd $CWD
cd ../../
dgdir=`pwd`

if [ ! -d $rosdir ]; then
    echo "You have to set rosdir first.";
    exit 0
fi

ln -sf $dgdir/examples/dg_test_ros $rosdir/src/
ln -sf $dgdir/src/vps/data_vps $rosdir/
ln -sf $dgdir/src/poi_recog/model $rosdir/
ln -sf $dgdir/bin/data $rosdir/

cp $dgdir/examples/dg_test_ros/dg_run.sh $rosdir/
cp $dgdir/examples/dg_test_ros/dg_databag.sh $rosdir/

## change path of python source in dg_test.cpp
cd $CWD
sed -e "s|\/work\/deepguider|$dgdir|g" .CMakeLists.txt > CMakeLists.txt
sed -e "s|\/work\/deepguider|$dgdir|g" src/.dg_test.cpp > src/dg_test.cpp
