source devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

## Start roscore
pid=`pgrep roscore`
if [ ! -n "${pid}" ];then  # If process is not running.
    gnome-terminal --tab -- roscore
    sleep 2s    # wait until roscore is ready
fi

## Start dg_simple_ros package (working directory: devel/lib/dg_simple_ros/)
pid=`pgrep -f dg_odometry`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi

gnome-terminal --tab -- rosrun dg_simple_ros dg_odometry
#gnome-terminal --tab -- rostopic echo /dg_odometry/pose

