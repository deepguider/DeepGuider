#!/bin/bash

## Set path of deepguider and ros workspace
DGDIR="/work/deepguider"        # path of DeepGuider source root
ROSDIR="/work/dg_ros"           # path of ROS workspace

## Check if specified DeepGuider directory exists.
if [ ! -d $DGDIR ]; then
    echo "Error: The specified DeepGuider directory '$DGDIR' doesn't exists." 
    echo "Modify it to valid path."
    exit
fi

## Check if specified ROS workspace exists.
if [ ! -d $ROSDIR ]; then
    echo "Error: The specified ROS workspace '$ROSDIR' doesn't exists." 
    echo "Create it first or specify valid ROS workspace path."
    exit
fi

## Make symbolic links to ROS workspace
mkdir -p ~/$ROSDIR/src
ln -sf $DGDIR/examples/dg_simple_ros $ROSDIR/src/
ln -sf $DGDIR/src/vps/data_vps $ROSDIR/
ln -sf $DGDIR/src/poi_recog/model $ROSDIR/
ln -sf $DGDIR/src/intersection_cls/data_intersection_cls $ROSDIR/
ln -sf $DGDIR/bin/data_localizer $ROSDIR/
ln -sf $DGDIR/bin/data $ROSDIR/

## Copy machine-specific files from template to local
cp $DGDIR/examples/dg_simple_ros/CMakeListsTemplate.txt $DGDIR/examples/dg_simple_ros/CMakeLists.txt
cp $DGDIR/examples/dg_simple_ros/dg_ros.yml $ROSDIR/

## Copy example shell scripts to ROS workspace
cp $DGDIR/examples/dg_simple_ros/dg_run.sh $ROSDIR/
cp $DGDIR/examples/dg_simple_ros/dg_databag.sh $ROSDIR/
chmod a+x $ROSDIR/dg_run.sh
chmod a+x $ROSDIR/dg_databag.sh
