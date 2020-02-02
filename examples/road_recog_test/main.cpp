#include "dg_core.hpp"
#include "dg_road_recog.hpp"
#include "python_embedding.hpp"
#include <chrono>

using namespace dg;
using namespace std;

int main()
{
	// Initialize the Python interpreter
    init_python_environment();

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
    close_python_environment();

	return 0;
}
