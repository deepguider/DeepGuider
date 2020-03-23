#include "dg_core.hpp"
#include "dg_vps.hpp"
#include "python_embedding.hpp"
#include <chrono>
#include <iostream>
#include <experimental/filesystem>
#include <string>

using namespace dg;
using namespace std;
using namespace std::experimental::filesystem;



int main()
{
    VPS vps;
	cv::Mat inImg;
	cv::Mat image;
    double gps_lat, gps_lat_d;
    double gps_lon;
    double gps_accuracy; //(0~1), 
	int frame;
	Timestamp t, t2;

    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    if (!vps.initialize())
    {
        return -1;
    }

    int N = 3;  // top-3

	// Read query images
    // cv::Mat image = cv::imread("./data_vps/vps_query.jpg");
	// images from ~/Naverlabs/query_etri_cart/images_2019_11_15_12_45_11/*.jpg";
    string qlist = "./data_vps/query_list.txt";
	string line;
	ifstream infile(qlist);
	frame = 0;

	gps_lat = 36.381438;
	gps_lat_d = 0.00001;
    gps_lon = 127.378867;
    gps_accuracy = 1.0;    //(0~1), 

	while(infile >> line){
    	std::vector<VPSResult> streetviews; //Since it is manipulated in python, this declaration must be placed in a local loop.
		frame ++;
		if (frame % 1 != 0)
		{
			continue;
		}

		string img = line;
		image = cv::imread(img);
		// inImg = cv::imread(img);
		// cv::resize(inImg, image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
		// imshow("new",image);
		// cv::waitKey(1);
		cout << "[" << frame << "-th frame(" << image.rows << "x" << image.cols << ")] : " << img << endl;

    // Run the Python module
	    t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
		gps_lat += gps_lat_d;
        if (!vps.apply(image, N, gps_lat, gps_lon, gps_accuracy, t)) {
            return -1;
        }
        vps.get(streetviews);
        t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("It took %lf seconds\n", t2 - t);
        for (int k = 0; k < streetviews.size(); k++)
        {
            printf("\ttop%d: id=%ld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t);
        }
    }

	printf("Finished\n");

    // Clear the Python module
    vps.clear();

    // Close the Python Interpreter
    close_python_environment();

    return 0;
}
