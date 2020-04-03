## DeepGuider Test in ROS

Simplified deepguider system that integrates its sub-modules and tests their integrated execution on a test dataset.

### Dependencies

Refere to each sub-module's README.md

### How to Build and Run Codes

1. Modify CMakeLists.txt (set variable `DGDIR` to be root directory of deepguider at `set(DGDIR   "/work/deepguider")`)
2. Modify setup_rosrun.sh (set variable `rosdir` to be ROS workspace directory)
3. Run the following shell script:
```
$ ./setup_rosrun.sh
$ cd rosdir # change working directory to ros workspace
$ catkin_make
$ roscore
$ ./dg_run.sh
$ rosbag play [your_rosbag_filepath]
```