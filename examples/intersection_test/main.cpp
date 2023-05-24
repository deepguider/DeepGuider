#include "dg_intersection.hpp"
#include "utils/python_embedding.hpp"
#include "utils/vvs.h"
#include "utils/opencx.hpp"
#include <chrono>
#include <thread>

using namespace dg;
using namespace std;

#define RECOGNIZER IntersectionClassifier

bool rotate_epr_image_180degree = false;

void test_image_run(RECOGNIZER& recognizer, bool recording = false, const char* image_file = "sample.png", int nItr = 5)
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

void erp2rect(cv::Mat src, cv::Mat& dst, double theta, double hfov, int img_w = 640, int img_h = 480)
{
    int dst_rows = img_h;
    int dst_cols = img_w;
    if (dst.empty() || dst.rows != dst_rows || dst.cols != dst_cols)
    {
        dst = cv::Mat(dst_rows, dst_cols, src.type());
    }

    double dst_cx = dst_cols / 2;
    double dst_cy = dst_rows / 2;
    double f = dst_cx / tan(hfov / 2);
    for (int x = 0; x < dst_cols; x++)
    {
        double xth = atan((x - dst_cx) / f);
        int src_x = (int)((xth + theta) * src.rows / CV_PI + 0.5);
        src_x = (src_x + src.cols) % src.cols;

        double yf = f / cos(xth);
        for (int y = 0; y < dst_rows; y++)
        {
            double yth = atan((y - dst_cy) / yf);
            int src_y = (int)(yth * src.rows / CV_PI + src.rows / 2 + 0.5);
            dst.at<cv::Vec3b>(y, x) = src.at<cv::Vec3b>(src_y, src_x);
        }
    }
}

cv::Mat convert_to_3camera(cv::Mat erp_image, bool rotate_180 = false)
{
    if (rotate_180)
    {
        int img_w = erp_image.cols;
        int img_h = erp_image.rows;
        cv::Mat first_half = erp_image(cv::Rect(0, 0, img_w/2, img_h));
        cv::Mat second_half = erp_image(cv::Rect(img_w/2, 0, img_w/2, img_h));
        cv::Mat rotated;
        cv::hconcat(second_half, first_half, rotated);
        erp_image = rotated;
    }

    cv::Mat rect_f, rect_l, rect_r;
    double hfov = cx::cvtDeg2Rad(90);
    erp2rect(erp_image, rect_f, cx::cvtDeg2Rad(180), hfov);
    erp2rect(erp_image, rect_l, cx::cvtDeg2Rad(90), hfov);
    erp2rect(erp_image, rect_r, cx::cvtDeg2Rad(270), hfov);

    cv::Mat stiched;
    cv::Mat src[3] = { rect_l, rect_f, rect_r };
    cv::hconcat(src, 3, stiched);
    return stiched;
}

void test_video_run(RECOGNIZER& recognizer, bool recording = false, int fps = 10, const char* video_file = "data/191115_ETRI.avi", bool use_3camera = false)
{
    printf("#### Test Video Run ####################\n");
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(video_file));
    video_data.set(cv::CAP_PROP_POS_FRAMES, 910);

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
    dg::Timestamp ts;
    int detected_cnt = 0;
    int detect_cnt_thr = 2;
    while (1)
    {
        int frame_i = (int)video_data.get(cv::CAP_PROP_POS_FRAMES);

        cv::Mat image;
        video_data >> image;
        if (image.empty()) break;
        if (use_3camera) image = convert_to_3camera(image, rotate_epr_image_180degree);

        dg::Timestamp ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        bool ok = recognizer.apply(image, ts);
        dg::IntersectionResult r = recognizer.get(ts);
        if (r.cls == 1) detected_cnt++;
        else detected_cnt = 0;

        printf("iteration: %d (it took %lf seconds)\n", i++, recognizer.procTime());
        recognizer.print();

        // draw image boundary
        if (use_3camera)
        {
            cv::Scalar color(255, 255, 255);
            int line_width = 5;
            int img_w = image.cols/3;
            int img_h = image.rows;
            cv::rectangle(image, cv::Rect(0, 0, img_w, img_h), color, line_width);
            cv::rectangle(image, cv::Rect(img_w, 0, img_w, img_h), color, line_width);
            cv::rectangle(image, cv::Rect(img_w*2, 0, img_w, img_h), color, line_width);
        }

        // draw frame number & fps
        std::string fn = cv::format("#%d (FPS: %.1lf)", frame_i, 1.0 / recognizer.procTime());
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

        if (detected_cnt >= detect_cnt_thr) recognizer.draw(image);
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


void procfunc(bool recording, int rec_fps, const char* video_path, bool use_3camera)
{
    // Initialize Python module
    RECOGNIZER recognizer;
    if (!recognizer.initialize("dg_simple.yml")) return;
    printf("Initialization: it took %.3lf seconds\n\n\n", recognizer.procTime());

    // Run the Python module
    //test_image_run(recognizer, false, cv::format("%s_sample.png", recognizer.name()).c_str());
    test_video_run(recognizer, recording, rec_fps, video_path, use_3camera);

    // Clear the Python module
    recognizer.clear();
}


int main()
{
    bool recording = false;
    int rec_fps = 15;
    bool threaded_run = false;
    bool use_3camera = true;
    rotate_epr_image_180degree = false;

    int video_sel = 2;
    const char* video_path[] = {
        "video/191115_ETRI.avi",
        "video/etri_cart_200219_15h01m_2fps.avi",
        "video/ricohtheta_bucheon_erp.avi",
        "video/ricohtheta_etri_erp_2022-06-13-16-56-54.avi",
        "video/201007_taeheran1.avi",
        "video/street-GOTOMall.mp4",
        "video/street-Itaewon.mp4",
        "video/street-MyeongDong.mp4",
        "video/street-MyeongDongShoppingAlley.mp4",
        "video/street-Shibuya.mp4"
    };

    // Initialize the Python interpreter
    init_python_environment("python3", "", threaded_run);

    if(threaded_run)
    {
		std::thread* test_thread = new std::thread(procfunc, recording, rec_fps, video_path[video_sel], use_3camera);
        test_thread->join();
    }
    else
    {
        procfunc(recording, rec_fps, video_path[video_sel], use_3camera);
    }

    // Close the Python Interpreter
    close_python_environment();

    printf("done..\n");

    return 0;
}
