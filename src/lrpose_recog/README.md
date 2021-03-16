
The lrpose_recognizer detects and outputs the relative positions of the automobile road and pedestrian road.
For python unit tests, copy the pre-trained weights under src/lrpose/data_lrpose.
The weight can be downloaded from: (not yet)

## Python unit test
 In your virtual environment,
	$ cd src/lrpose_recog
	$ python3 lrpose_recognizer.py  # you need to modify input directory including *.jpg files in it as follow.
		236: fnlist = mod_pose_recog.read_testdata('/home/ccsmm/workdir/DB_Repo/ETRI_CartRobot/extracted/200626')

## CPP unit test
 In your virtual environment,
 	$ cd examples/lrpose_test
	$ ./setup_pose_recog.sh   # Once at first
	$ ./build_lrpose_test.sh  # build and run
