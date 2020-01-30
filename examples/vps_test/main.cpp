#include "dg_core.hpp"
#include "dg_vps.hpp"

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
	VPS vps;
	if (!vps.initialize())
	{
		return -1;
	}

	// Run the Python module
	cv::Mat image = cv::imread("vps_sample.jpg");
    double gps_lat = 0;
    double gps_lon = 0;
    double gps_accuracy = 0;        // error boundary (meter)

	int nIter = 5;
	for (int i = 0; i < nIter; i++)
	{
		Timestamp t = i;
		if (!vps.apply(image, gps_lat, gps_lon, gps_accuracy, t)) {
			return -1;
		}

        double lat, lon, prob;
		vps.get(lat, lon, prob);
		printf("lat = %lf, lon = %lf, prob = %lf, timestamp = %lf\n", lat, lon, prob, t);
	}

	// Clear the Python module
	vps.clear();

	// Close the Python Interpreter
	if (Py_FinalizeEx() < 0) {
		return -1;
	}

	return 0;
}
