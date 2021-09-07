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

#ROSDIR="/home/ccsmm/catkin_ws"           # path of ROS workspace
ROSDIR="/home/${USER}/catkin_ws"         # path of ROS workspace

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
ln -sf $DGDIR/bin/data $ROSDIR/
ln -sf $DGDIR/bin/data_localizer $ROSDIR/
ln -sf $DGDIR/src/vps/data_vps $ROSDIR/
ln -sf $DGDIR/src/lrpose_recog/data_lrpose $ROSDIR/
ln -sf $DGDIR/src/intersection_cls/data_intersection_cls $ROSDIR/
ln -sf $DGDIR/src/logo_recog/logo_data $ROSDIR/
ln -sf $DGDIR/src/logo_recog/model $ROSDIR/
ln -sf $DGDIR/src/ocr_recog/data_ocr $ROSDIR/
ln -sf $DGDIR/src/poi_recog/data_poi $ROSDIR/
ln -sf $DGDIR/src/poi_recog/model_poi $ROSDIR/

## Copy machine-specific files from template to local
cp $DGDIR/examples/dg_simple_ros/CMakeListsTemplate.txt $DGDIR/examples/dg_simple_ros/CMakeLists.txt
cp $DGDIR/examples/dg_simple_ros/dg_ros.yml $ROSDIR/
sed -i "s|/work/deepguider|$DGDIR|g" $DGDIR/examples/dg_simple_ros/CMakeLists.txt
sed -i "s|/home/dgtest/deepguider|$DGDIR|g" $ROSDIR/dg_ros.yml

## Copy example shell scripts to ROS workspace
cp $DGDIR/examples/dg_simple_ros/dg_run.sh $ROSDIR/
cp $DGDIR/examples/dg_simple_ros/dg_databag.sh $ROSDIR/
chmod a+x $ROSDIR/dg_run.sh
chmod a+x $ROSDIR/dg_databag.sh

echo "Run:"
echo "     cd $ROSDIR;./dg_run.sh"
