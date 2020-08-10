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

        // winname
        std::string winname = "dg_result";
        if (sel == 0) winname = "VPS";
        if (sel == 1) winname = "POI-OCR";
        if (sel == 2) winname = "POI-Logo";
        if (sel == 3) winname = "Intersection Classifier";

        cv::Mat image;
        cv::namedWindow(winname, cv::WINDOW_NORMAL);
        cv::setWindowProperty(winname, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        int fn = -1;
        while (1)
        {
            vc >> image;
            if (image.empty()) break;
            fn++;
            if (fn >= m_total_frames) break;
            cv::Mat image_disp = image;
            
            // drawing
            int disp_delay = 1;
            double fps = -1;
            if (sel == 0 && !m_data[fn].vps.empty())
            {
                vps.read(m_data[fn].vps);
                std::vector<VPSResult> svs;
                vps.get(svs);
                cv::Mat sv_image;
                cv::Mat sv_image_resized;
                if (!svs.empty() && svs[0].id>0 && map_manager.getStreetViewImage(svs[0].id, sv_image, "f") && !sv_image.empty())
                {
                    int h = image.rows;
                    int w = sv_image.cols * image.rows / sv_image.rows;
                    cv::resize(sv_image, sv_image_resized, cv::Size(w, h));
                    disp_delay = 2000;
                }
                else
                {
                    sv_image_resized = sv_black;
                }                

                if (!svs.empty() && svs[0].id > 0)
                {
                    vps.draw(sv_image_resized);
                    fps = 1.0 / vps.procTime();
                }
                cv::hconcat(image, sv_image_resized, image_disp);
            }
            else if(sel == 0)
            {
                cv::hconcat(image, sv_black, image_disp);
            }
            else if (sel == 1 && !m_data[fn].ocr.empty())
            {
                image_disp = image;
                ocr.read(m_data[fn].ocr);
                ocr.draw(image_disp);
                if (!m_data[fn].ocr.empty()) fps = 1.0 / ocr.procTime();
                disp_delay = 2000;
            }
            else if (sel == 2 && !m_data[fn].logo.empty())
            {
                image_disp = image;
                logo.read(m_data[fn].logo);
                logo.draw(image_disp);
                if (!m_data[fn].logo.empty()) fps = 1.0 / logo.procTime();
                disp_delay = 2000;
            }
            else if (sel == 3 && !m_data[fn].intersection.empty())
            {
                image_disp = image;
                intersection.read(m_data[fn].intersection);
                intersection.draw(image_disp);
                if (!m_data[fn].intersection.empty()) fps = 1.0 / intersection.procTime();
                IntersectionResult intersect;
                intersection.get(intersect);
                if(intersect.cls>0) disp_delay = 300;
                else disp_delay = 50;
            }

            // fn & fps
            std::string str;
            if (fps > 0)
                str = cv::format("#%d (FPS: %.1lf)", fn, fps);
            else
                str = cv::format("#%d (FPS: N/A)", fn);
            cv::putText(image_disp, str.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 0, 0), 4);
            cv::putText(image_disp, str.c_str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 255), 2);

            // display
            cv::imshow(winname, image_disp);
            int key = cv::waitKey(disp_delay);
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
        // in case of broken video, total_frames is zero.
        m_data.clear();
        m_total_frames = (int)(vc.get(cv::CAP_PROP_FRAME_COUNT));
        if(m_total_frames>0) m_data.resize(m_total_frames);

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
            while (fn >= 0 && m_total_frames <= fn)
            {
                m_data.push_back(LogFrameData());
                m_total_frames++;
            }
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
    if (argc > 1) dataname = argv[1];
    if (argc > 2) 
    {
        std::string module_name = argv[2];
        if(module_name == "vps") module_selection = 0;
        if(module_name == "ocr") module_selection = 1;
        if(module_name == "logo") module_selection = 2;
        if(module_name == "intersection" || module_name == "intersect") module_selection = 3;
    }

    std::string fname_log = dataname + ".txt";
    std::string fname_cam = dataname + "_cam.avi";

    LogViewer viewer;
    viewer.run(fname_log, fname_cam, module_selection);
    printf("done...\n");
    return 0;
}

