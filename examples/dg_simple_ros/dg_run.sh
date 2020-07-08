gnome-terminal -e roscore
sleep 2s    # wait until roscore is ready
gnome-terminal -e "rosbag play --pause -s 500 -r 10 ./recordings/2020-02-19-15-01-53.bag"
echo "Press [Space] to start rosbag play on the gnome terminal"
rosrun dg_simple_ros dg_simple_ros
