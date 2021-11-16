The RoadLR detects and outputs the relative positions of the automobile road and pedestrian road.



```
# Definition of relative position classes.
'||'  means road where cars are moving.

Left (class 0)  :   Robot ||       # The robot is on the left side of the road.

Right (class 2) :         || Robot # the robot is on the right side of the road.

Uncertain (class 1) : uncertain case, ex) cross road, intersection, no way, etc. 
```

For python unit tests, copy the pre-trained weights under src/roadlr/data_roadlr.
The weight can be downloaded from: (not yet)

## Python unit test
 In your virtual environment,

```
# Before run, You need to modify input directory including *.jpg files in it as follow.
236: fnlist = mod_pose_recog.read_testdata('/home/ccsmm/workdir/DB_Repo/ETRI_CartRobot/extracted/200626')

$ cd src/roadlr
$ python3 roadlr.py
```

## CPP unit test
 In your virtual environment,

```
$ cd examples/roadlr_test
$ ./setup_roadlr_test.sh   # Once at first
$ ./build_roadlr_test.sh  # build and run
```
