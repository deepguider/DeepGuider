#include "dg_core.hpp"
#include "dg_ocr.hpp"
#include "dg_utils.hpp"
#include <chrono>

using namespace dg;
using namespace std;

void drawOCRResult(cv::Mat image, const std::vector<OCRResult>& ocrs)
{
    for(size_t i=0; i<ocrs.size(); i++)
    {
        cv::Rect rc(ocrs[i].xmin, ocrs[i].ymin, ocrs[i].xmax-ocrs[i].xmin+1, ocrs[i].ymax-ocrs[i].ymin+1);
        cv::rectangle(image, rc, cv::Scalar(0, 255, 0), 2);
        cv::Point pt(ocrs[i].xmin + 5, ocrs[i].ymin + 35);
        std::string msg = cv::format("%s %.2lf", ocrs[i].label.c_str(), ocrs[i].confidence);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 6);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2);
    }
}

void test_image_run(OCRRecognizer& ocr_recog, bool recording = false, const char* image_file = "ocr_sample.jpg", int nItr = 5)
{
    printf("#### Test Image Run ####################\n");
    cv::Mat image = cv::imread(image_file);
    VVS_CHECK_TRUE(!image.empty());
    cv::namedWindow(image_file);
    for (int i = 1; i <= nItr; i++)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(ocr_recog.apply(image, t1));

        std::vector<OCRResult> ocrs;
        ocr_recog.get(ocrs);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < ocrs.size(); k++)
        {
            printf("\tocr%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, t=%lf\n", k, ocrs[k].xmin, ocrs[k].ymin, ocrs[k].xmax, ocrs[k].ymax, ocrs[k].label.c_str(), ocrs[k].confidence, t1);
        }

        cv::Mat image_result = image.clone();
        drawOCRResult(image_result, ocrs);
        cv::imshow(image_file, image_result);
        cv::waitKey(1);

        if(recording)
        {
            cv::imwrite("ocr_result_image.jpg", image_result);
        }
    }
    cv::destroyWindow(image_file);
}

void test_video_run(OCRRecognizer& ocr_recog, bool recording = false, int fps = 10, const char* video_file = "data/191115_ETRI.avi")
{
    bool save_latest_frame = false;

    cx::VideoWriter video;
    if (recording)
    {
        time_t start_t;
        time(&start_t);
        tm _tm = *localtime(&start_t);
        char szfilename[255];
        strftime(szfilename, 255, "ocr_result_%y%m%d_%H%M%S.avi", &_tm);
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

        if(save_latest_frame) cv::imwrite("ocr_latest_input.png", image);

        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        VVS_CHECK_TRUE(ocr_recog.apply(image, t1));

        std::vector<OCRResult> ocrs;
        ocr_recog.get(ocrs);
        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("iteration: %d (it took %lf seconds)\n", i, t2 - t1);
        for (int k = 0; k < (int)ocrs.size(); k++)
        {
            printf("\tocr%d: x1=%d, y1=%d, x2=%d, y2=%d, label=%s, confidence=%lf, t=%lf\n", k, ocrs[k].xmin, ocrs[k].ymin, ocrs[k].xmax, ocrs[k].ymax, ocrs[k].label.c_str(), ocrs[k].confidence, t1);
        }
        i++;

        // draw frame number & fps
        std::string fn = cv::format("#%d (FPS: %.1lf)", frame_i, 1.0 / (t2 - t1));
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
        cv::putText(image, fn.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

        drawOCRResult(image, ocrs);
        if(recording) video << image;

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
    const char* video_path = "data/etri_cart_200219_15h01m_2fps.avi";
    //const char* video_path = "data/etri_cart_191115_11h40m_10fps.avi";

    // Initialize the Python interpreter
    init_python_environment("python3", "");

    // Initialize Python module
    OCRRecognizer ocr_recog;
    Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    if (!ocr_recog.initialize())
    {
        return -1;
    }
    Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    printf("Initialization: it took %lf seconds\n", t2 - t1);

    // Run the Python module
    test_image_run(ocr_recog, false);
    test_video_run(ocr_recog, recording, rec_fps, video_path);

    // Clear the Python module
    ocr_recog.clear();

    // Close the Python Interpreter
    close_python_environment();

    printf("done..\n");

    return 0;
}
