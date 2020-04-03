## DeepGuider Test in ROS

Simplified deepguider system that integrates its sub-modules and is implemented as a ROS package.

### Dependencies

Refere to each sub-module's README.md

### How to Build and Run Codes

1. Modify CMakeLists.txt (set `DGDIR` in `set(DGDIR   "/work/deepguider")` to be the path of local deepguider repository)
2. Modify setup_rosrun.sh (set `rosdir` in `rosdir="/work/dg_ros"` to be ROS workspace directory)
3. Modify path of python modules in the source code (src/dg_test.cpp:176-186)
```
    // initialize VPS
    if (enable_vps && !m_vps.initialize("vps", "/work/deepguider/src/vps")) return false;
    if (enable_vps) printf("\tVPS initialized!\n");

    // initialize POI
    if (enable_poi && !m_poi.initialize("poi_recognizer", "/work/deepguider/src/poi_recog")) return false;
    if (enable_poi) printf("\tPOI initialized!\n");

    // initialize roadTheta
    if (enable_roadTheta && !m_roadTheta.initialize("road_direction_recognizer", "/work/deepguider/src/road_recog")) return false;
    if (enable_roadTheta) printf("\tRoadTheta initialized!\n");

```
4. Run the following shell commads:
```
$ ./setup_rosrun.sh
$ cd rosdir # change working directory to ROS workspace
$ catkin_make
$ roscore
$ ./dg_run.sh
$ rosbag play [your_rosbag_filepath]
```