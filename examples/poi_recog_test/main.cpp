#include "dg_core.hpp"
#include "dg_poi_recog.hpp"
#include <chrono>

#define PY_SSIZE_T_CLEAN
#include <Python.h>

using namespace dg;
using namespace std;

int main()
{
	// Initialize the Python interpreter
	wchar_t* program = Py_DecodeLocale("poi_recog_test", NULL);
	if (program == NULL) {
		fprintf(stderr, "Fatal error: cannot decode poi_recog_test\n");
		exit(-1);
	}
	Py_SetProgramName(program);
	Py_Initialize();

	// Initialize Python module
	POIRecognizer poi_recog;
	if (!poi_recog.initialize())
	{
		return -1;
	}

	// Run the Python module
	cv::Mat image = cv::imread("poi_sample.jpg");
	int nIter = 5;
	for (int i = 1; i <= nIter; i++)
	{
        Timestamp t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

		if (poi_recog.apply(image, t) < 0) {
			return -1;
		}

        std::vector<POIResult> pois;
		poi_recog.get(pois);
        Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t);
        for (int k = 0; k < pois.size(); k++)
        {
            printf("\tpoi%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, t=%lf\n", k, pois[k].xmin, pois[k].ymin, pois[k].xmax, pois[k].ymax, pois[k].label.c_str(), pois[k].confidence, t);
        }
	}

	// Clear the Python module
	poi_recog.clear();

	// Close the Python Interpreter
	if (Py_FinalizeEx() < 0) {
		return -1;
	}

	return 0;
}
