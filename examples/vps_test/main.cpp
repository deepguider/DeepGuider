#include "dg_core.hpp"
#include "dg_vps.hpp"
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
	VPS vps;
	if (!vps.initialize())
	{
		return -1;
	}

	// Run the Python module
	cv::Mat image = cv::imread("vps_sample.jpg");
    double gps_lat = 0;
    double gps_lon = 0;
    double gps_accuracy = 10;        // error boundary (meter)

	int nIter = 5;
	for (int i = 0; i < nIter; i++)
	{
        Timestamp t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        int N = 3;  // top-3
		if (!vps.apply(image, N, gps_lat, gps_lon, gps_accuracy, t)) {
			return -1;
		}

        std::vector<VPSResult> streetviews;
        vps.get(streetviews);
        Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t);
        for (int k = 0; k < streetviews.size(); k++)
        {
            printf("\ttop%d: id=%ld, confidence=%lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t);
        }
	}

	// Clear the Python module
	vps.clear();

	// Close the Python Interpreter
	if (Py_FinalizeEx() < 0) {
		return -1;
	}

	return 0;
}
