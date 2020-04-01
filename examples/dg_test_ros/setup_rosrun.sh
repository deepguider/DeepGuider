#!/bin/bash
##Current Directory : dgdir/examples/dg_test_ros
cd ../../
dgdir=`pwd`
rosdir="/work/dg_ros"  # set ros workspace directory here.

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

