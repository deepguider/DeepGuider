#include "dg_core.hpp"
#include "dg_road_recog.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>

using namespace dg;
using namespace std;

int main()
{
	RoadDirectionRecognizer road_recog;

	cv::Mat image = cv::imread("road_sample.jpg");
	Timestamp t = 10;
	if (road_recog.apply(image, t) < 0) {
		return -1;
	}

	double ang, p;
	road_recog.get(ang, p);
	printf("ang = %lf, prob = %lf\n", ang, p);

	return 0;
}
