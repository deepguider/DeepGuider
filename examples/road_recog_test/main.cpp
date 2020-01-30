#include "dg_core.hpp"
#include "dg_road_recog.hpp"
#include <chrono>

#define PY_SSIZE_T_CLEAN
#include <Python.h>

using namespace dg;
using namespace std;

int main()
{
	// Initialize the Python interpreter
	wchar_t* program = Py_DecodeLocale("python3", NULL);
	if (program == NULL) {
		fprintf(stderr, "Fatal error: cannot decode locale.\n");
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
		Timestamp t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

		if (!road_recog.apply(image, t)) {
			return -1;
		}

		double ang, p;
		road_recog.get(ang, p);
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
