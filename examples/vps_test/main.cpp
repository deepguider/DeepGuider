#include "dg_core.hpp"
#include "dg_vps.hpp"
#include "python_embedding.hpp"
#include <chrono>
#include <iostream>
#include <experimental/filesystem>
#include <string>
#include <thread>
#include "vvs.h"
#include "opencx.hpp"

using namespace dg;
using namespace std;
using namespace std::experimental::filesystem;

//std::string map_server_ip = "129.254.87.96"; // You must pass this to vps.apply() as "const char*" using map_server_ip.c_str()
std::string map_server_ip = "localhost"; // You must pass this to vps.apply() as "const char*" using map_server_ip.c_str()

void drawVPSResult(cv::Mat image, const std::vector<VPSResult>& streetviews)
{
    for(size_t i=0; i<streetviews.size(); i++)
    {
        // curerntly, do nothing (reserved for future use)
    }
}

void test_image_run(VPS& vps, bool recording = false, const char* image_file = "./data_vps/vps_query.jpg", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());
    cv::namedWindow(image_file);

    int N = 3;  // top-3
	double gps_lat = 36.381438;
	double gps_lat_d = 0.00001;
    double gps_lon = 127.378867;
    double gps_accuracy = 1.0;    //(0~1), 

    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        gps_lat += gps_lat_d;
        VVS_CHECK_TRUE(vps.apply(image, N, gps_lat, gps_lon, gps_accuracy, t1, map_server_ip.c_str()));

        std::vector<VPSResult> streetviews;
        vps.get(streetviews);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < streetviews.size(); k++)
        {
            printf("\ttop%d: id=%ld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t1);
        }

        cv::Mat image_result = image.clone();
        drawVPSResult(image_result, streetviews);
        cv::imshow(image_file, image_result);
        cv::waitKey(1);

        if(recording)
        {
            cv::imwrite("vps_result_image.jpg", image_result);
        }
    }
    printf("press any key...\n");
    cv::waitKey();
    cv::destroyWindow(image_file);
}

void test_video_run(VPS& vps, bool recording = false, const char* video_file = "data/191115_ETRI.avi")
{
    bool save_latest_frame = false; // for debugging purpose

    cx::VideoWriter video;
    if (recording)
    {
        time_t start_t;
        time(&start_t);
        tm _tm = *localtime(&start_t);
        char szfilename[255];
        strftime(szfilename, 255, "vps_result_%y%m%d_%H%M%S.avi", &_tm);
        std::string filename = szfilename;
        video.open(filename, 30);
    }
 
    printf("#### Test Video Run ####################\n");
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(video_file));
    cv::namedWindow(video_file);

    int N = 3;  // top-3
	double gps_lat = 36.381438;
	double gps_lat_d = 0.00001;
    double gps_lon = 127.378867;
    double gps_accuracy = 1.0;    //(0~1), 
 
    int i = 1;
    while (1)
    {
        int frame_i = video_data.get(cv::CAP_PROP_POS_FRAMES);

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;
        if(save_latest_frame) cv::imwrite("vps_latest_input.png", image);

        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
		if (int(i/300)*300 == i)
		{
			gps_lat_d *= -1.0;
		}
        gps_lat += gps_lat_d;
        //VVS_CHECK_TRUE(vps.apply(image, N, gps_lat, gps_lon, gps_accuracy, t1, map_server_ip.c_str()));
        VVS_CHECK_TRUE(vps.thread_apply(image, N, gps_lat, gps_lon, gps_accuracy, t1, map_server_ip.c_str()));

        std::vector<VPSResult> streetviews;
        vps.get(streetviews);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < streetviews.size(); k++)
        {
            printf("\ttop%d: id=%ld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t1);
        }
        i++;

        drawVPSResult(image, streetviews);
        std::string fn = cv::format("#%d", frame_i);
        cv::putText(image, fn.c_str(), cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
        cv::putText(image, fn.c_str(), cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);
        if(recording) video << image;

        cv::imshow(video_file, image);
        int key = cv::waitKey(1);
        if (key == cx::KEY_SPACE) key = cv::waitKey(0);
        if (key == cx::KEY_ESC) break;
    }
    cv::destroyWindow(video_file); 
}

void test_query_run(VPS& vps, bool recording = false, const char* qlist = "./data_vps/query_list.txt")
{
    printf("#### Test Query Run ####################\n");
 
	cv::Mat inImg;
	cv::Mat image;
    double gps_lat, gps_lat_d;
    double gps_lon;
    double gps_accuracy; //(0~1), 
	int frame;
	Timestamp t, t2;

    int N = 3;  // top-3

	// Read query images
    // cv::Mat image = cv::imread("./data_vps/vps_query.jpg");
	// images from ~/Naverlabs/query_etri_cart/images_2019_11_15_12_45_11/*.jpg";
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
        VVS_CHECK_TRUE(vps.apply(image, N, gps_lat, gps_lon, gps_accuracy, t, map_server_ip.c_str()));

        vps.get(streetviews);
        t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("It took %lf seconds\n", t2 - t);
        for (int k = 0; k < streetviews.size(); k++)
        {
            printf("\ttop%d: id=%ld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, t);
        }
    }
	printf("Finished\n");
}


void threadfunc_vps(VPS* vps, bool* is_running)
{
    printf("#### Test Thread Run ####################\n");

    *is_running = true;
    test_video_run(*vps, false);
    *is_running = false;

    printf("Finished\n");
}


int main()
{
    bool test_image = false; // OK
    bool test_video = false; // OK
    bool test_query = false; // OK
    bool test_thread_run = true; // OK after make thread_apply()
    bool enable_recording = false;

    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    VPS vps;
    Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    if (!vps.initialize())
    {
        return -1;
    }
    Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    printf("Initialization: it took %lf seconds\n", t2 - t1);

    // Run the Python module
    if(test_image) test_image_run(vps, enable_recording);
    if(test_video) test_video_run(vps, enable_recording);
    if(test_query) test_query_run(vps, enable_recording);

    if(test_thread_run)
    {
        bool is_running_vps = true;
        std::thread* vps_thread = new std::thread(threadfunc_vps, &vps, &is_running_vps);
        while(is_running_vps)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Clear the Python module
    vps.clear();

    // Close the Python Interpreter
    close_python_environment();

    return 0;
}
