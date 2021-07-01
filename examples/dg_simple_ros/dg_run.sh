source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
gnome-terminal -- roscore
sleep 2s    # wait until roscore is ready
gnome-terminal -- rosbag play --pause -s 500 -r 10 ./recordings/2020-02-19-15-01-53.bag
echo "Press [Space] to start rosbag play on the gnome terminal"
gnome-terminal -- rosrun dg_simple_ros dg_ocr # run ocr_ros first in another console
rosrun dg_simple_ros dg_simple_ros            # run dg_simple_ros
