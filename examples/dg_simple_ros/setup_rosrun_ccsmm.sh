#!/bin/bash

## Set path of deepguider and ros workspace

# ROSDIR is the only one you have to change to your own.
#ROSDIR="/work/dg_ros"   # path of ROS workspace
ROSDIR="/tmp/catkin_ws"  # path of ROS workspace



############### Do not modify from here

CWD0=`pwd`
CWD1=`dirname $CWD0`
CWD2=`dirname $CWD1`

#DGDIR="/work/deepguider"       # path of DeepGuider source root
DGDIR=$CWD2        				# path of DeepGuider source root


function func_caktin_ws_create(){
	CWD=`pwd`
	local catkin_ws=$1

	if [ ! -z $catkin_ws ];then
		source /opt/ros/melodic/setup.bash
		mkdir -p $catkin_ws/src
		cd $catkin_ws
		catkin_make
		echo "Run :"
		echo "      source $catkin_ws/devel/setup.bash"
		cd $CWD
	else
		echo "Usage :"
		echo "       $0 [new_catkin_ws_dirname]"
	fi
}


## Copy and make symbolic links to ROS workspace from DGdir
function func_copy_src_to_catkin_ws(){
	local DGDIR=$1
	local ROSDIR=$2

	if [ -d ~/$ROSDIR/src ]; then
		mv ~/$ROSDIR/src ~/$ROSDIR/src.bak
	fi

	mkdir -p ~/$ROSDIR/src

	## Link related source codes to catkin workspace
	#cp $DGDIR/examples/dg_simple_ros $ROSDIR/src/. -rf
	#SRCDST=$ROSDIR/src/dg_simple_ros/src
	#cp -f $DGDIR/examples/dg_simple/dg_simple.cpp $SRCDST
	#cp -f $DGDIR/src/*.hpp $SRCDST 
	#cp -rf $DGDIR/src/core $SRCDST 

	ln -sf $DGDIR/examples/dg_simple_ros $ROSDIR/src/
	
	ln -sf $DGDIR/bin/data $ROSDIR/
	ln -sf $DGDIR/bin/data_localizer $ROSDIR/
	ln -sf $DGDIR/src/vps/data_vps $ROSDIR/
	ln -sf $DGDIR/src/intersection_cls/data_intersection_cls $ROSDIR/
	ln -sf $DGDIR/src/logo_recog/logo_data $ROSDIR/
	ln -sf $DGDIR/src/logo_recog/model $ROSDIR/
	ln -sf $DGDIR/src/ocr_recog/data_ocr $ROSDIR/
	ln -sf $DGDIR/src/poi_recog/data_poi $ROSDIR/
	ln -sf $DGDIR/src/poi_recog/model_poi $ROSDIR/

	## Copy machine-specific files from template to local
	#cp $DGDIR/examples/dg_simple_ros/CMakeListsTemplate.txt $DGDIR/examples/dg_simple_ros/CMakeLists.txt
	sed -e "s|\$ENV{HOME}/dg/DeepGuider|$DGDIR|g" $DGDIR/examples/dg_simple_ros/CMakeListsTemplate.txt > $DGDIR/examples/dg_simple_ros/CMakeLists.txt
	cp $DGDIR/examples/dg_simple_ros/dg_ros.yml $ROSDIR/

	## Copy example shell scripts to ROS workspace
	cp $DGDIR/examples/dg_simple_ros/dg_run.sh $ROSDIR/
	cp $DGDIR/examples/dg_simple_ros/dg_databag.sh $ROSDIR/
	chmod a+x $ROSDIR/dg_run.sh
	chmod a+x $ROSDIR/dg_databag.sh

	mkdir -p $ROSDIR/recordings
}


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
	echo "$ROSDIR will be created automatically."
    #exit
	func_caktin_ws_create $ROSDIR
fi

func_copy_src_to_catkin_ws $DGDIR $ROSDIR

echo "Make link of ./recordings/2020-02-19-15-01-53.bag to $ROSDIR"
echo "Then run followings :"
echo "    source $ROSDIR/devel/setup.bash;cd $ROSDIR;catkin_make;./dg_run.sh"
