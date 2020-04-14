## DeepGuider Test in ROS

Simplified deepguider system that integrates its sub-modules and is implemented as a ROS package.

### Dependencies

Refere to each sub-module's README.md

### How to Build and Run Codes

1. Set the value of **DGDIR** and Python paths properly in CMakeLists.txt
2. Set **DGDIR** and **ROSDIR** properly in setup_rosrun.sh
3. Modify **m_srcdir** properly in the code src/dg_ros.cpp:176-186_.
4. Run the following shell commands in order:
```
$ ./setup_rosrun.sh
$ cd $ROSDIR # change working directory to ROS workspace
$ source /opt/ros/kinetic/setup.bash
$ catkin_make
$ roscore
$ ./dg_run.sh
$ rosbag play [your_rosbag_filepath]
```