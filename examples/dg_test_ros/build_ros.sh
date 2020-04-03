#!/bin/bash
##Current Directory : dgdir/examples/dg_test_ros

catkin_ws="catkin_ws_deepguider"
CWD=`pwd`

cd ~/$catkin_ws
catkin_make

echo "######################################"
echo "To Run :"
echo "        - roscore # in a new terminal"
echo "        - # Enter python3.6's virtualenv"
echo "        - cd ~/$catkin_ws;./dg_run.sh"
echo "        - rosbag play [your_rosbag_filepath] # in a new terminal"
