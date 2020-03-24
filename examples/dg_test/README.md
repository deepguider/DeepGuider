## DeepGuider Test

Simplified deepguider system that integrates its sub-modules and tests their integrated execution on a test dataset.

### Dependencies

Refere to each sub-module's README.md

### How to Build and Run Codes

1. Place a sample video clip into _deepguider/bin/data/_ folder to be used as test camera input and name it "19115_ETR.avi" (you can change the input video name in the source code: main.cpp:25).
2. Run the following shell script:
```
$ ./build_run_dgtest.sh
```