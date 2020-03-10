#include "dg_core.hpp"
#include "dg_vps.hpp"
#include "python_embedding.hpp"
#include <chrono>

using namespace dg;
using namespace std;

int main()
{
    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    VPS vps;
    if (!vps.initialize())
    {
        return -1;
    }

    // Run the Python module
    cv::Mat image = cv::imread("./data_vps/vps_query.jpg");
    double gps_lat = 0;
    double gps_lon = 0;
    double gps_accuracy = 10;        // error boundary (meter)

	//cv::imshow("query",image);
	//cv::waitKey(1);

    int nIter = 2;
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
			//printf("\ttop%d: id=%lld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t);
            printf("\ttop%d: id=%ld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t);
        }
    }

    // Clear the Python module
    vps.clear();

    // Close the Python Interpreter
    close_python_environment();

    return 0;
}
