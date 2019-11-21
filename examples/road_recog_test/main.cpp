#include "dg_core.hpp"
#include "dg_road_recog.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>

using namespace dg;
using namespace std;

int main()
{
	// Initialize the Python interpreter
	wchar_t* program = Py_DecodeLocale("road_recog_test", NULL);
	if (program == NULL) {
		fprintf(stderr, "Fatal error: cannot decode road_recog_test\n");
		exit(-1);
	}
	Py_SetProgramName(program);
	Py_Initialize();

	// Initialize Python module
	RoadDirectionRecognizer road_recog;
	if (!road_recog.initialize())
	{
		return -1;
	}

	// Run the Python module
	cv::Mat image = cv::imread("road_sample.jpg");
	int nIter = 5;
	for (int i = 0; i < nIter; i++)
	{
		Timestamp t = i;
		if (road_recog.apply(image, t) < 0) {
			return -1;
		}

		double ang, p;
		road_recog.get(ang, p, t);
		printf("angle = %lf, prob = %lf, timestamp = %lf\n", ang, p, t);
	}

	// Clear the Python module
	road_recog.clear();

	// Close the Python Interpreter
	if (Py_FinalizeEx() < 0) {
		return -1;
	}

	return 0;
}
