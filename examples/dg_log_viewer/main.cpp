#define VVS_NO_ASSERT
#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_road_recog.hpp"
#include "dg_logo.hpp"
#include "dg_ocr.hpp"
#include "dg_intersection.hpp"
#include "dg_vps.hpp"
#include "dg_guidance.hpp"
#include "dg_exploration.hpp"
#include "dg_utils.hpp"
#include <chrono>
#include <fstream>

using namespace dg;
using namespace std;

#define MAX_DATA_LEN 1024

struct LogFrameData
{
    std::vector<std::string> vps;
    std::vector<std::string> ocr;
    std::vector<std::string> logo;
    std::vector<std::string> intersection;
};

class LogViewer
{
public:
    void run(std::string fname_log, std::string fname_cam, int sel)
    {
        // open log file
        ifstream stream;
        stream.open(fname_log, ios::in);
        if (!stream.is_open())
        {
            printf("Error: can't open %s\n", fname_log.c_str());
            return;
        }

        // open cam data
        cv::VideoCapture vc(fname_cam);
        if (!vc.isOpened())
        {
            printf("Error: can't open %s\n", fname_log.c_str());
            return;
        }

        // build data
        build(stream, vc);

        // modules
        VPS vps;
        OCRRecognizer ocr;
        LogoRecognizer logo;
        IntersectionClassifier intersection;

        MapManager map_manager;
        //std::string server_ip = "127.0.0.1";
        std::string server_ip = "129.254.87.96";
        map_manager.setIP(server_ip);
        map_manager.initialize();
        cv::Mat sv_black(720, 720, CV_8UC3);
        sv_black = 0;

        cv::Mat image;
        cv::Mat result;
        cv::Mat display;
        int fn = -1;
        while (1)
        {
            vc >> image;
            if (image.empty()) break;
            fn++;
            
            // drawing
            double fps = -1;
            if (sel == 0) result = sv_black;
            else result = image.clone();

            if (sel == 0 && !m_data[fn].vps.empty())
            {
                vps.read(m_data[fn].vps);
                std::vector<VPSResult> svs;
                vps.get(svs);
                cv::Mat sv_image;
                if (!svs.empty() && map_manager.getStreetViewImage(svs[0].id, sv_image, "f") && !sv_image.empty())
                {
                    int h = image.rows;
                    int w = sv_image.cols * image.rows / sv_image.rows;
                    cv::resize(sv_image, sv_image, cv::Size(w, h));
                    result = sv_image;
                }

                if (!svs.empty() && svs[0].id > 0)
                {
                    vps.draw(result);
                    fps = 1.0 / vps.procTime();
                }
            }
            else if (sel == 1 && !m_data[fn].ocr.empty())
            {
                ocr.read(m_data[fn].ocr);
                ocr.draw(result);
                if (!m_data[fn].ocr.empty()) fps = 1.0 / ocr.procTime();
            }
            else if (sel == 2 && !m_data[fn].logo.empty())
            {
                logo.read(m_data[fn].logo);
                logo.draw(result);
                if (!m_data[fn].logo.empty()) fps = 1.0 / logo.procTime();
            }
            else if (sel == 3 && !m_data[fn].intersection.empty())
            {
                intersection.read(m_data[fn].intersection);
                intersection.draw(result);
            }

            // fn & fps
            std::string str;
            if (fps > 0)
                str = cv::format("#%d (FPS: %.1lf)", fn, fps);
            else
                str = cv::format("#%d (FPS: N/A)", fn, fps);
            cv::putText(image, str.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
            cv::putText(image, str.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

            // display
            cv::hconcat(image, result, display);
            cv::imshow("result", display);
            int key = cv::waitKey(100);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == 83)    // Right Key
            {
                fn += 30;
                if (fn > m_total_frames) fn = m_total_frames - 1;
                vc.set(cv::CAP_PROP_POS_FRAMES, fn);
                fn--;
            }
            if (key == cx::KEY_ESC) break;
        }
    }

protected:
    int m_total_frames = 0;
    std::vector<LogFrameData> m_data;

    bool build(ifstream& stream, cv::VideoCapture& vc)
    {
        m_total_frames = (int)(vc.get(cv::CAP_PROP_FRAME_COUNT));
        if (m_total_frames <= 0) return false;
        m_data.resize(m_total_frames);

        // parse log data
        char buf[MAX_DATA_LEN];
        char buf_tmp[MAX_DATA_LEN];
        int buf_len = MAX_DATA_LEN;
        while (!stream.eof())
        {
            // read one line (skip blank lines)
            buf[0] = '\0';
            stream.getline(buf, buf_len);
            while (!stream.eof() && buf[0] == '\0') stream.getline(buf, buf_len);
            if (buf[0] == '\0' && stream.eof()) break;

            // parse data
            dg::Timestamp ts;
            int fn = -1;
            sscanf(buf, "%lf,%d,%s", &ts, &fn, buf_tmp);
            int idx = (int)string(buf_tmp).find_first_of(',');
            std::string name = string(buf_tmp).substr(0, idx);
            //printf("ts = %.3lf, fn = %d, name = %s\n", ts, fn, name.c_str());
            if (fn >= 0 && fn < m_total_frames)
            {
                if (name == "vps")
                    m_data[fn].vps.push_back(std::string(buf));
                else if (name == "ocr")
                    m_data[fn].ocr.push_back(std::string(buf));
                else if (name == "logo")
                    m_data[fn].logo.push_back(std::string(buf));
                else if (name == "intersection")
                    m_data[fn].intersection.push_back(std::string(buf));
            }
        }

        return true;
    }
};


int main(int argc, char* argv[])
{
    int module_selection = 0;     // 0: vps, 1: ocr, 2: logo, 3: intersection

    std::string dataname = "dg_simple_200804_102346";
    if (argc > 1)dataname = argv[1];

    std::string fname_log = dataname + ".txt";
    std::string fname_cam = dataname + "_cam.avi";

    LogViewer viewer;
    viewer.run(fname_log, fname_cam, module_selection);
    return 0;
}

