#include "dg_core.hpp"
#include "dg_poi_recog.hpp"
#include "dg_utils.hpp"
#include <chrono>

using namespace dg;
using namespace std;

void drawPOIResult(cv::Mat image, const std::vector<POIResult>& pois)
{
    for(size_t i=0; i<pois.size(); i++)
    {
        cv::Rect rc(pois[i].xmin, pois[i].ymin, pois[i].xmax-pois[i].xmin+1, pois[i].ymax-pois[i].ymin+1);
        cv::rectangle(image, rc, cv::Scalar(0, 255, 0), 2);
        cv::Point pt(pois[i].xmin + 5, pois[i].ymin + 35);
        std::string msg = cv::format("%s %.2lf", pois[i].label.c_str(), pois[i].confidence);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 6);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2);
    }
}

void test_image_run(POIRecognizer& poi_recog, bool recording = false, const char* image_file = "poi_sample.jpg", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());
    cv::namedWindow(image_file);
    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(poi_recog.apply(image, t1));

        std::vector<POIResult> pois;
        poi_recog.get(pois);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < pois.size(); k++)
        {
            printf("\tpoi%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, t=%lf\n", k, pois[k].xmin, pois[k].ymin, pois[k].xmax, pois[k].ymax, pois[k].label.c_str(), pois[k].confidence, t1);
        }

        cv::Mat image_result = image.clone();
        drawPOIResult(image_result, pois);
        cv::imshow(image_file, image_result);
        cv::waitKey(1);

        if(recording)
        {
            cv::imwrite("poi_result_image.jpg", image_result);
        }
    }
    printf("press any key...\n");
    cv::waitKey();
    cv::destroyWindow(image_file);
}

void test_video_run(POIRecognizer& poi_recog, bool recording = false, const char* video_file = "data/191115_ETRI.avi")
{
    bool save_latest_frame = true;

    cx::VideoWriter video;
    if (recording)
    {
        time_t start_t;
        time(&start_t);
        tm _tm = *localtime(&start_t);
        char szfilename[255];
        strftime(szfilename, 255, "poi_result_%y%m%d_%H%M%S.avi", &_tm);
        std::string filename = szfilename;
        video.open(filename, 30);
    }
 
    printf("#### Test Video Run ####################\n");
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(video_file));

    cv::namedWindow(video_file);
    int i = 1;
    while (1)
    {
        int frame_i = video_data.get(cv::CAP_PROP_POS_FRAMES);

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;

        if(save_latest_frame) cv::imwrite("poi_latest_input.png", image);

        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(poi_recog.apply(image, t1));

        std::vector<POIResult> pois;
        poi_recog.get(pois);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < pois.size(); k++)
        {
            printf("\tpoi%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, t=%lf\n", k, pois[k].xmin, pois[k].ymin, pois[k].xmax, pois[k].ymax, pois[k].label.c_str(), pois[k].confidence, t1);
        }
        i++;

        drawPOIResult(image, pois);
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

int main()
{
    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    POIRecognizer poi_recog;
    Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    if (!poi_recog.initialize())
    {
        return -1;
    }
    Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    printf("Initialization: it took %lf seconds\n", t2 - t1);

    // Run the Python module
    test_image_run(poi_recog, false);
    test_video_run(poi_recog, false);

    // Clear the Python module
    poi_recog.clear();

    // Close the Python Interpreter
    close_python_environment();

    return 0;
}
