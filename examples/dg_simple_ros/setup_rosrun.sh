#!/bin/bash

## Set path of deepguider and ros workspace
#DGDIR="/home/ccsmm/tmp/DeepGuider"        # path of DeepGuider source root
CWD=`pwd`
cd ../../
DGDIR=`pwd`
## Check if specified DeepGuider directory exists.
if [ ! -d $DGDIR ]; then
    echo "Error: The specified DeepGuider directory '$DGDIR' doesn't exists." 
    echo "Modify it to valid path."
    exit
fi
cd $DGDIR

ROS_WS_DIR_CONFIG_FILE="$CWD/.ROS_WS_DIR"
if [ -e "${ROS_WS_DIR_CONFIG_FILE}" ]; then
	ROS_WS_DIR=`cat ${ROS_WS_DIR_CONFIG_FILE}`
else
	ROS_WS_DIR="catkin_ws"
fi

ROSDIR="/home/${USER}/${ROS_WS_DIR}"         # path of ROS workspace

## Check if specified ROS workspace exists.
if [ ! -d $ROSDIR ]; then
    echo "Warning: The specified ROS workspace '$ROSDIR' doesn't exists." 
    echo "Create it first or specify valid ROS workspace path."
	mkdir -p $ROSDIR
fi

## Make symbolic links to ROS workspace
mkdir -p $ROSDIR/src
ln -sf $DGDIR/examples/dg_simple_ros $ROSDIR/src/
ln -sf $DGDIR/bin/font $ROSDIR/
ln -sf $DGDIR/bin/recordings $ROSDIR/
ln -sf $DGDIR/bin/data $ROSDIR/
#ln -sf $DGDIR/bin/data_localizer $ROSDIR/
ln -sf $DGDIR/src/intersection_cls/data_intersection_cls $ROSDIR/
ln -sf $DGDIR/src/ocr_recog/data_ocr $ROSDIR/
ln -sf $DGDIR/src/vps/data_vps $ROSDIR/
ln -sf $DGDIR/src/roadlr/data_roadlr $ROSDIR/
ln -sf $DGDIR/src/exploration/data_exp $ROSDIR/
ln -sf $DGDIR/src/logo_recog/logo_data $ROSDIR/
ln -sf $DGDIR/src/logo_recog/model $ROSDIR/
#ln -sf $DGDIR/src/poi_recog/data_poi $ROSDIR/
#ln -sf $DGDIR/src/poi_recog/model_poi $ROSDIR/

## Copy machine-specific files from template to local
cp $DGDIR/examples/dg_simple_ros/CMakeListsTemplate.txt $DGDIR/examples/dg_simple_ros/CMakeLists.txt
cp $DGDIR/examples/dg_simple_ros/dg_ros.yml $ROSDIR/
cp $DGDIR/src/vps/init_vps_ramdisk.sh $ROSDIR/
sed -i "s|/home/dgtest/deepguider|$DGDIR|g" $DGDIR/examples/dg_simple_ros/CMakeLists.txt
sed -i "s|/home/dgtest/deepguider|$DGDIR|g" $ROSDIR/dg_ros.yml

## Copy example shell scripts to ROS workspace
cp $DGDIR/examples/dg_simple_ros/dg_run.sh $ROSDIR/
cp $DGDIR/examples/dg_simple_ros/dg_run_saved_bag.sh $ROSDIR/
cp $DGDIR/examples/dg_simple_ros/dg_cart_sensor_install.sh $ROSDIR/
cp $DGDIR/examples/dg_simple_ros/dg_cart_sensor_run.sh $ROSDIR/
chmod a+x $ROSDIR/dg_run.sh
chmod a+x $ROSDIR/dg_run_saved_bag.sh
chmod a+x $ROSDIR/dg_cart_sensor_install.sh
chmod a+x $ROSDIR/dg_cart_sensor_run.sh

cp $DGDIR/examples/dg_simple_ros/odo_cal.py $ROSDIR/

echo "Run:"
echo "     cd $ROSDIR;./dg_run.sh"
