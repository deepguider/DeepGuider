#include "dg_roadtheta.hpp"
#include "utils/vvs.h"
#include "utils/opencx.hpp"
#include <chrono>
#include <thread>

using namespace dg;
using namespace std;

#define RECOGNIZER RoadTheta

void test_image_run(RECOGNIZER& recognizer, bool recording = false, const char* image_file = "roadtheta_sample.png", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());

    cv::namedWindow(image_file);
    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        bool ok = recognizer.apply(image, ts);

        printf("iteration: %d (it took %lf seconds)\n", i, recognizer.procTime());
        recognizer.print();

        cv::Mat image_result = image.clone();
        recognizer.draw(image_result);
        cv::imshow(image_file, image_result);
        int key = cv::waitKey(1000);
        if (key == cx::KEY_SPACE) key = cv::waitKey(0);
        if (key == cx::KEY_ESC) break;        

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
    dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    while (1)
    {
        int frame_i = (int)video_data.get(cv::CAP_PROP_POS_FRAMES);

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;

        dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        bool ok = recognizer.apply(image, ts);

        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i++, t2 - t1);
        t1 = t2;
        //recognizer.print();

        // draw frame number & fps
        std::string fn = cv::format("#%d (FPS: %.1lf)", frame_i, 1.0 / recognizer.procTime());
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

        recognizer.draw(image, 2);
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


void procfunc(bool recording, int rec_fps, const char* video_path)
{
    // Initialize module
    RECOGNIZER recognizer;
    if (!recognizer.initialize()) return;
    printf("Initialization: it took %.3lf seconds\n\n\n", recognizer.procTime());

    // Run the module
    //test_image_run(recognizer, false, cv::format("%s_sample.png", recognizer.name()).c_str());
    test_video_run(recognizer, recording, rec_fps, video_path);

    // Clear the module
    recognizer.clear();
}


int main()
{
    bool recording = true;
    int rec_fps = 15;
    bool threaded_run = false;

    int video_sel = 1;
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

    if(threaded_run)
    {
		std::thread* test_thread = new std::thread(procfunc, recording, rec_fps, video_path[video_sel]);
        test_thread->join();
    }
    else
    {
        procfunc(recording, rec_fps, video_path[video_sel]);
    }

    printf("done..\n");

    return 0;
}
