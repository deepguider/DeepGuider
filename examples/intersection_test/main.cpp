#include "dg_intersection.hpp"
#include "dg_utils.hpp"
#include <chrono>

using namespace dg;
using namespace std;

void drawIntersectionResult(cv::Mat image, IntersectionResult& intersect)
{
    cv::Point pt(100, 50);
    std::string msg = cv::format("Intersect: %d (%.2lf)", intersect.cls, intersect.confidence);
    cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 6);
    cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2);
}

void test_video_run(IntersectionClassifier& classifier, bool recording = false, const char* video_file = "data/191115_ETRI.avi")
{
    bool save_latest_frame = false;

    cx::VideoWriter video;
    if (recording)
    {
        time_t start_t;
        time(&start_t);
        tm _tm = *localtime(&start_t);
        char szfilename[255];
        strftime(szfilename, 255, "intersection_result_%y%m%d_%H%M%S.avi", &_tm);
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
        int frame_i = (int)video_data.get(cv::CAP_PROP_POS_FRAMES);

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;

        if(save_latest_frame) cv::imwrite("intersection_latest_input.png", image);

        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(classifier.apply(image, t1));

        IntersectionResult intersect;
        classifier.get(intersect);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        printf("\tIntersect: %d (%.2lf)", intersect.cls, intersect.confidence);
        i++;

        drawIntersectionResult(image, intersect);
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
    IntersectionClassifier classifier;
    Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    if (!classifier.initialize())
    {
        return -1;
    }
    Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    printf("Initialization: it took %lf seconds\n", t2 - t1);

    // Run the Python module
    test_video_run(classifier, false);

    // Clear the Python module
    classifier.clear();

    // Close the Python Interpreter
    close_python_environment();

    printf("done..\n");

    return 0;
}
