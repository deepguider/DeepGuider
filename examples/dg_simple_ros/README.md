## DeepGuider Simple Test in ROS
Simplified deepguider system that integrates its sub-modules and is implemented as a ROS package.

### Dependencies
Refere to each sub-module's README.md

### How to Build and Run Codes
#### Do manually
1. Set the value of **DGDIR** and **ROSDIR** properly in setup_rosrun.sh
2. `$./setup_rosrun.sh`
3. Open CMakeLists.txt and set the value of **DGDIR** and Python paths properly in CMakeLists.txt
4. Open **ROSDIR/dg_ros.yml** and edit **dg_srcdir** properly
5. Run the following shell commands in order:

```
$ cd $ROSDIR # change working directory to ROS workspace
$ source /opt/ros/kinetic/setup.bash
$ catkin_make
$ source $ROSDIR/devel/setup.bash
$ roscore   # in a new terminal
$ rosrun dg_ros dg_ros_node
$ rosbag play [your_rosbag_file_path] # in a new terminal
```

#### Do automatically
1. `$./rosbuild.sh /home/your_ros_ws_path`  ==> It will create or overwrite your path of ros catkin workspace.
2. Run final message of above script.
3. Cycles of Compile and Run :
```
$ # Modify your code in examples/dg_simple_ros/dg_simple_ros.cpp
$ cd /home/your_ros_ws_path
$ ./build_and_run.sh # for rosrun
$ ./dg_run.sh # for rosrun
```

### Tips
You can add _source /opt/ros/kinetic/setup.bash_ and _source $ROSDIR/devel/setup.bash_ to .bashrc for your convenience.
You can also use the provided example script `$ ./dg_run.sh` to run dg_ros in one step.
For ubuntu 18.04, use /opt/ros/melodic/setup.bash instead of /opt/ros/kinetic/setup.bash
