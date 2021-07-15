#include "dg_vps.hpp"
#include "dg_utils.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#ifdef VPSSERVER
	#include <jsoncpp/json/json.h>
	#include <curl/curl.h>
#endif	// #ifdef VPSSERVER

using namespace dg;
using namespace std;

//std::string map_server_ip = "129.254.87.96"; // You must pass this to vps.apply() as "const char*" using map_server_ip.c_str()
std::string map_server_ip = "localhost"; // You must pass this to vps.apply() as "const char*" using map_server_ip.c_str()


void test_image_run(VPS& recognizer, bool recording = false, const char* image_file = "./data_vps/vps_query.jpg", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());

    int N = 3;  // top-3
	double gps_lat = 36.381438;
	double gps_lat_d = 0.00001;
    double gps_lon = 127.378867;
    double gps_accuracy = 1.0;    //(0~1), 

    cv::namedWindow(image_file);
    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        bool ok = recognizer.apply(image, N, gps_lat, gps_lon, gps_accuracy, ts, map_server_ip.c_str());
        gps_lat += gps_lat_d;

		printf("iteration: %d (it took %lf seconds)\n", i, recognizer.procTime());
		recognizer.print();

        cv::Mat image_result = image.clone();
		recognizer.draw(image_result);
        cv::imshow(image_file, image_result);
        cv::waitKey(1000);

		if (recording)
		{
			string fname = cv::format("%s_result.png", recognizer.name());
			cv::imwrite(fname, image_result);
		}
	}
    cv::destroyWindow(image_file);
}

void test_video_run(VPS& recognizer, bool recording = false, int fps = 10, const char* video_file = "data/191115_ETRI.avi")
{
	printf("#### Test Video Run ####################\n");
	cv::VideoCapture video_data;
	VVS_CHECK_TRUE(video_data.open(video_file));

	cx::VideoWriter video;
	std::ofstream log;
	if (recording)
	{
		char sztime[255];
		time_t start_t;
		time(&start_t);
		tm _tm = *localtime(&start_t);
		strftime(sztime, 255, "%y%m%d_%H%M%S", &_tm);
		video.open(cv::format("%s_%s.avi", recognizer.name(), sztime), fps);
		log.open(cv::format("%s_%s.txt", recognizer.name(), sztime), ios::out);
	}

    int N = 3;  // top-3
#if 1	// Daejeon
	double gps_lat = 36.381438;
    double gps_lon = 127.378867;
#else // Seoul
	double gps_lat = 37.514852;
    double gps_lon = 127.0551879;
#endif
	double gps_lat_d = 0.00001;
    double gps_accuracy = 1.0;    //(0~1), 

	cv::namedWindow(video_file);
	int i = 1;
    while (1)
    {
		int frame_i = (int)video_data.get(cv::CAP_PROP_POS_FRAMES);

		cv::Mat image;
		video_data >> image;
		if (image.empty()) break;

		dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
		bool ok = recognizer.apply(image, N, gps_lat, gps_lon, gps_accuracy, ts, map_server_ip.c_str());
		if (int(i/300)*300 == i)
		{
			gps_lat_d *= -1.0;
		}
        gps_lat += gps_lat_d;

		printf("iteration: %d (it took %lf seconds)\n", i++, recognizer.procTime());
		recognizer.print();

		// draw frame number & fps
		std::string fn = cv::format("#%d (FPS: %.1lf)", frame_i, 1.0 / recognizer.procTime());
		cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
		cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

		if (recording)
		{
			video << image;
			recognizer.write(log);
		}

		cv::imshow(video_file, image);
		int key = cv::waitKey(1);
		if (key == cx::KEY_SPACE) key = cv::waitKey(0);
		if (key == 83)    // Right Key
		{
			int fi = (int)video_data.get(cv::CAP_PROP_POS_FRAMES);
			video_data.set(cv::CAP_PROP_POS_FRAMES, fi + 30);
		}
		if (key == cx::KEY_ESC) break;
	}
	cv::destroyWindow(video_file);
}


#ifdef VPSSERVER
std::size_t curl_callback(
        const char* in,
        std::size_t size,
        std::size_t num,
        std::string* out)
{
    const std::size_t totalBytes(size * num);
    out->append(in, totalBytes);
    return totalBytes;
}

bool curl_request(const std::string url, const char * CMD, const Json::Value * post_json, Json::Value * get_json)
{
	CURLcode res;
	CURL *hnd;
	struct curl_slist *slist1;
	slist1 = NULL;
	slist1 = curl_slist_append(slist1, "Content-Type: application/json"); //Do not edit
	int ret = 0;
	std::string jsonstr;
	long httpCode(0);
	std::unique_ptr<std::string> httpData(new std::string());

	hnd = curl_easy_init();
	// url = "http://localhost:7729/Apply/";
	curl_easy_setopt(hnd, CURLOPT_URL, url.c_str());
	curl_easy_setopt(hnd, CURLOPT_NOPROGRESS, 1L);

	/** Post messate to server **/
	if(strcmp("POST", CMD) == 0)
	{
		// std::string jsonstr = "{\"username\":\"bob\",\"password\":\"12345\"}";
		jsonstr = post_json->toStyledString();
		// cout << jsonstr << endl;
		curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, jsonstr.c_str());
		curl_easy_setopt(hnd, CURLOPT_USERAGENT, "curl/7.38.0");
		curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist1);
		curl_easy_setopt(hnd, CURLOPT_MAXREDIRS, 50L);
		curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "POST");
		curl_easy_setopt(hnd, CURLOPT_TCP_KEEPALIVE, 1L);
	}

	/** Get response from server **/
 	// POST needs GET afere it, so we don't need break; here.
	if(strcmp("POST", CMD) == 0 || strcmp("GET", CMD) == 0)
	{
		curl_easy_setopt(hnd, CURLOPT_WRITEFUNCTION, curl_callback);
		curl_easy_setopt(hnd, CURLOPT_WRITEDATA, httpData.get());
		curl_easy_setopt(hnd, CURLOPT_PROXY, "");
		
		res = curl_easy_perform(hnd);
		curl_easy_getinfo(hnd, CURLINFO_RESPONSE_CODE, &httpCode);
		curl_easy_cleanup(hnd);
		curl_slist_free_all(slist1);

		hnd = NULL;
		slist1 = NULL;
		
		// Run our HTTP GET command, capture the HTTP response code, and clean up.
		if (httpCode == 200)
		{
		    //Json::Value *get_json;
		    Json::Reader jsonReader;
		    if (jsonReader.parse(*httpData.get(), *get_json))
		    {
		        //std::cout << get_json->toStyledString() << std::endl;
				ret = 0;
		    }
		    else
		    {
		        std::cout << "Could not parse HTTP data as JSON" << std::endl;
		        std::cout << "HTTP data was:\n" << *httpData.get() << std::endl;
		        ret = 1;
		    }
		}
		else
		{
		    std::cout << "Couldn't GET from " << url << " - exiting" << std::endl;
		    ret = 1;
		}
	}
	return ret;
}

void test_video_server_run(bool recording = false, const char* video_file = "data/191115_ETRI.avi")
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

    int N = 5;  // top-3
#if 1	// Daejeon
	double gps_lat = 36.381438;
    double gps_lon = 127.378867;
#else // Seoul
	double gps_lat = 37.514852;
    double gps_lon = 127.0551879;
#endif
	double gps_lat_d = 0.00001;
    double gps_accuracy = 1.0;    //(0~1), 

	const std::string vps_server_addr = "http://localhost:7729";
	const std::string streetview_server_addr = "localhost";
	std::string vps_url = vps_server_addr + "/Apply/";
	Json::Value post_json;
	Json::Value ret_json;
	std::string post_str;
	const char * post_data;
	list<int> image_size;
	std::vector<uchar> image_data; 

    int i = 0;
    while (1)
    {
        int frame_i = video_data.get(cv::CAP_PROP_POS_FRAMES);

		i++;

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;

		if ((i % 1) != 0)
		{
			continue;
		}

        if(save_latest_frame) cv::imwrite("vps_latest_input.png", image);

        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
		if (int(i/300)*300 == i)
		{
			gps_lat_d *= -1.0;
		}
        gps_lat += gps_lat_d;
		

		post_json["K"] = N;
		post_json["gps_lat"] = gps_lat;
		post_json["gps_lon"] = gps_lon;
		post_json["gps_accuracy"] = gps_accuracy;
		post_json["timestamp"] = t1;
		post_json["streetview_server_ipaddr"] = streetview_server_addr;
		
		post_json["image_size"].clear();
		post_json["image_size"].append(image.rows); // h
		post_json["image_size"].append(image.cols); // w
		post_json["image_size"].append(image.channels()); // c

		post_json["image_data"].clear();
		image.reshape( image.cols * image.rows * image.channels()); // Serialize
		uchar * image_data = image.data;
		for(int idx = 0; idx < image.rows*image.cols*image.channels(); idx++)
		{
			post_json["image_data"].append(image_data[idx]);
		}

	 	curl_request(vps_url, "POST", &post_json, &ret_json);
		curl_request(vps_url, "GET" , 0, &ret_json); // [Optional]

        std::vector<VPSResult> streetviews;
        VPSResult IDandConf;
		//cout << ret_json["vps_IDandConf"].size() << endl;

		for(int idx = 0; idx < ret_json["vps_IDandConf"][0].size(); idx++) // 5 : K
		{
			IDandConf.id = ret_json["vps_IDandConf"][0][idx].asDouble(); 
			IDandConf.confidence = ret_json["vps_IDandConf"][1][idx].asDouble(); 
			streetviews.push_back(IDandConf);
		}

        // vps.get(streetviews);
        // VVS_CHECK_TRUE(vps.apply(image, N, gps_lat, gps_lon, gps_accuracy, t1, map_server_ip.c_str()));

        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < streetviews.size(); k++)
        {
            printf("\ttop%d: id=%ld, confidence=%.2lf, t=%lf\n", k, streetviews[k].id, streetviews[k].confidence, ret_json["timestamp"].asDouble());
        }

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
#endif	// #ifdef VPSSERVER


void test_query_run(VPS& recognizer, bool recording = false, const char* qlist = "./data_vps/query_list.txt")
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

	while(infile >> line)
	{
		frame ++;
		if (frame % 1 != 0)
		{
			continue;
		}

		string img = line;
		image = cv::imread(img);
		cout << "[" << frame << "-th frame(" << image.rows << "x" << image.cols << ")] : " << img << endl;

	    t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
		gps_lat += gps_lat_d;
        VVS_CHECK_TRUE(recognizer.apply(image, N, gps_lat, gps_lon, gps_accuracy, t, map_server_ip.c_str()));

		recognizer.print();
    }
	printf("Finished\n");
}


void threadfunc_vps(VPS* recognizer, bool* is_running)
{
    printf("#### Test Thread Run ####################\n");

    *is_running = true;
    test_video_run(*recognizer, false);
    *is_running = false;

    printf("Finished\n");
}


void threadfunc_vps_server(bool* is_running)
{
    printf("#### Test Thread Run ####################\n");

    *is_running = true;
#ifdef VPSSERVER
	test_video_server_run(false);
#endif	// #ifdef VPSSERVER
    *is_running = false;

    printf("Finished\n");
}


int main()
{
	// Uses Python embedded
    bool test_image = false; // OK
    bool test_video = true; // OK
    bool test_query = false; // OK
    bool test_thread_run = true; // OK

	// Uses server call to external Python flask server
    bool test_thread_run_server = false; // OK
    bool test_video_server = false; // OK

    bool enable_recording = false;

    int video_sel = 0;
    const char* video_path[] = {
        "video/191115_ETRI.avi",
        "video/etri_cart_200219_15h01m_2fps.avi",
        "video/201007_taeheran1.avi",
        "video/street-GOTOMall.mp4",
        "video/street-Itaewon.mp4",
        "video/street-MyeongDong.mp4",
        "video/street-MyeongDongShoppingAlley.mp4",
        "video/street-Shibuya.mp4"
    };


	if (test_image || test_video || test_query || test_thread_run) /** Python Embedded Version **/
	{

	    // Initialize the Python interpreter
	    init_python_environment("python3", "", test_thread_run);
	
	    
	    // Initialize Python module
	    VPS recognizer;
		if (!recognizer.initialize())
	    {
	        return -1;
	    }
		printf("Initialization: it took %.3lf seconds\n", recognizer.procTime());

	    // Run the Python module
	    if(test_image) test_image_run(recognizer, enable_recording);
	    if(test_video) test_video_run(recognizer, enable_recording, 10, video_path[video_sel]);
		if(test_query) test_query_run(recognizer, enable_recording);
	
	    if(test_thread_run)
	    {
	        bool is_running_vps = true;
	        std::thread* vps_thread = new std::thread(threadfunc_vps, &recognizer, &is_running_vps);
	        while(is_running_vps)
	        {
	            std::this_thread::sleep_for(std::chrono::milliseconds(100));
	        }
	    }
	
	    // Clear the Python module
	    recognizer.clear();
	
	    // Close the Python Interpreter
	    close_python_environment();
	}
	else if (test_thread_run_server || test_video_server) /** Python Server Call version **/
	{
	#ifdef VPSSERVER
		// We don't need Python interpreter, instead of it, use server call
	    if(test_video_server) test_video_server_run(enable_recording);
	
	    if(test_thread_run_server)
	    {
	        bool is_running_vps = true;
	        std::thread* vps_thread = new std::thread(threadfunc_vps_server, &is_running_vps);
	        while(is_running_vps)
	        {
	            std::this_thread::sleep_for(std::chrono::milliseconds(100));
	        }
	    }
	#endif	// #ifdef VPSSERVER
	}

    return 0;
}
