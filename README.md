## DeepGuider

### Dependency
* [OpenCV](http://opencv.org/) (> 3.0.0, [3-clause BSD License](https://opencv.org/license/))
  * _OpenCV_ is a base of all example codes for basic computer vision algorithms, linear algebra, image/video manipulation, and GUI.
* [QGC Ground Control Station](https://github.com/mavlink/qgroundcontrol) ([Apache 2.0 License](https://github.com/mavlink/qgroundcontrol/blob/master/COPYING.md))
  * UTM-related routines in _QGC Ground Control Station_ is utilized for conversion between geodesic notation (latitude and longitude) and UTM.
  * It is included in `EXTERNAL` directory.

### How to Run Codes
* Microsoft Windows with Microsoft Visual Studio
  * Prerequisite: [Visual Studio](https://visualstudio.microsoft.com/) (>= 2015; for [binary compatibility](https://docs.microsoft.com/ko-kr/cpp/porting/binary-compat-2015-2017)), [DeepGuider codes](https://github.com/deepguider/RoadGPS/archive/master.zip), [OpenCV binaries](https://github.com/sunglok/3dv_tutorial/releases/download/misc/OpenCV_v4.1.1_MSVS2017_x64.zip) (v4.1.1, x64)
  * Unzip DeepGuider codes and OpenCV binaries at `your_folder`
  * Run your Visual Studio and open the solution file, `your_folder\msvs\examples.sln`
* Linux with GCC
  * Install OpenCV if you don't have yet

### License
Please refer [DeepGuider Project LSA](LICENSE.md).

### Authors
* [Sunglok Choi](http://sites.google.com/site/sunglok/) (sunglok AT hanmail DOT net)

* [Jae-Yeong Lee](http://sites.google.com/site/roricljy/) (roricljy AT gmail DOT com)

* Seungmin Choi (ccsmm78 AT gmail DOT com)

* [Seohyun Jeon] (seohyunatwork AT gmail DOT com)

* [Jaeho Lim] (zeozeo7142 AT gmail DOT com)

  

### Acknowledgement
The authors thank the following contributors and projects.

* This work was supported by the ICT R&D program of [MSIT](https://msit.go.kr/)/[IITP](https://www.iitp.kr/), *Development of AI Technology for Guidance of a Mobile Robot to its Goal with Uncertain Maps in Indoor/Outdoor Environments* (2019-0-01309).
