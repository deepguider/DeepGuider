#include "dg_intersection.hpp"
#include "dg_utils.hpp"
#include <chrono>

using namespace dg;
using namespace std;

#define RECOGNIZER IntersectionClassifier


void test_image_run(RECOGNIZER& recognizer, bool recording = false, const char* image_file = "sample.png", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());

    cv::namedWindow(image_file);
    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(recognizer.apply(image, ts));

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


void test_video_run(RECOGNIZER& recognizer, bool recording = false, int fps = 10, const char* video_file = "data/191115_ETRI.avi")
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

    cv::namedWindow(video_file);
    int i = 1;
    while (1)
    {
        int frame_i = (int)video_data.get(cv::CAP_PROP_POS_FRAMES);

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;

        dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(recognizer.apply(image, ts));

        printf("iteration: %d (it took %lf seconds)\n", i++, recognizer.procTime());
        recognizer.print();
        
        // draw frame number & fps
        std::string fn = cv::format("#%d (FPS: %.1lf)", frame_i, 1.0/ recognizer.procTime());
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

        recognizer.draw(image);
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

int main()
{
    bool recording = false;
    int rec_fps = 5;

    const char* video_path = "data/191115_ETRI.avi";
    //const char* video_path = "data/etri_cart_200219_15h01m_2fps.avi";
    //const char* video_path = "data/etri_cart_191115_11h40m_10fps.avi";

    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    RECOGNIZER recognizer;
    if (!recognizer.initialize()) return -1;
    printf("Initialization: it took %.3lf seconds\n", recognizer.procTime());

    // Run the Python module
    test_image_run(recognizer, false, cv::format("%s_sample.png", recognizer.name()).c_str());
    test_video_run(recognizer, recording, rec_fps, video_path);

    // Clear the Python module
    recognizer.clear();

    // Close the Python Interpreter
    close_python_environment();

    printf("done..\n");

    return 0;
}
