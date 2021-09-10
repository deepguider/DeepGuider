#include "dg_ocr.hpp"
#include "utils/python_embedding.hpp"
#include "utils/vvs.h"
#include "utils/opencx.hpp"
#include <chrono>
#include <thread>
#include "localizer/data_loader.hpp"

#include "../dg_module_test/module_runner.hpp"

using namespace dg;
using namespace std;

#define RECOGNIZER OCRRecognizer
#define LOCALIZER OCRLocalizer

struct MapGUIProp
{
public:
    std::string image_file;
    std::string map_file;
    dg::LatLon  origin_latlon;      // origin of UTM
    cv::Point2d origin_px;          // pixel coordinte of UTM origin at map image
    cv::Point2d image_scale;
    double      image_rotation = 0; // radian
    cv::Point   map_view_offset = cv::Point(0, 0);
    cv::Size    map_view_size = cv::Size(1920, 1080);
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    double      video_resize = 0;
    cv::Point   video_offset;
    double      result_resize = 0;
};

/**
 * @brief DataLoader4Rec
 *
 * This implement interfaces for loading and providing timestamped sensor data for recording
 */
class DataLoader4Rec : public dg::DataLoader
{
public:
    int getFrameNumber(const Timestamp time)
    {
        int frame_i;
        if (m_camera_data.isOpened())
        {
            frame_i = (int)((time - m_first_data_time) * m_video_fps / m_video_scale + 0.5);
        }
        return frame_i;
    }
}; // End of 'DataLoader4Rec'


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
        bool ok = recognizer.apply(image, ts);

        printf("iteration: %d (it took %lf seconds)\n", i++, recognizer.procTime());
        recognizer.print();

        // draw frame number & fps
        std::string fn = cv::format("#%d (FPS: %.1lf)", frame_i, 1.0 / recognizer.procTime());
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


void procfunc(bool recording, int rec_fps, const char* video_path)
{
    // Initialize Python module
    RECOGNIZER recognizer;
    if (!recognizer.initialize()) return;
    printf("Initialization: it took %.3lf seconds\n\n\n", recognizer.procTime());

    // Run the Python module
    //test_image_run(recognizer, false, cv::format("%s_sample.png", recognizer.name()).c_str());
    test_video_run(recognizer, recording, rec_fps, video_path);

    // Clear the Python module
    recognizer.clear();
}


int runOCRLocalizer()
{
    bool recording = false;
    int rec_fps = 5;
    bool threaded_run = false;

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

    // Initialize the Python interpreter
    init_python_environment("python3", "", threaded_run);

    if(threaded_run)
    {
		std::thread* test_thread = new std::thread(procfunc, recording, rec_fps, video_path[video_sel]);
        test_thread->join();
    }
    else
    {
        procfunc(recording, rec_fps, video_path[video_sel]);
    }

    // Close the Python Interpreter
    close_python_environment();

    printf("done..\n");

    return 0;
}


int runOCRloc(LOCALIZER& localizer, DataLoader& data_loader, const std::string& rec_traj_file = "")
{
    // Prepare the result trajectory
    FILE* out_traj = nullptr;
    
    if (!rec_traj_file.empty())
    {
        out_traj = fopen(rec_traj_file.c_str(), "wt");//, ccs=UTF-8");
        if (out_traj == nullptr) return -1;
        fprintf(out_traj, "fnumber,timestamp,poi_id,poi_x,poi_y,distance,angle,confidence\n");
        //fprintf(out_traj, "fnumber,timestamp,dname,x,y,w,h,poi_id,poi_name,poi_x,poi_y,poi_floor,distance,angle,confidence,cam_lat,cam_lon\n");
    }
    
    int type;
    std::vector<std::string> data;
    dg::Timestamp data_time;
    //std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    
    while (1)
    {
        if (data_loader.getNext(type, data, data_time) == false) break;

        int fnumber = std::stoi(data[0]);
        double timestamp = std::stod(data[1]); 
        std::string dname = data[2];
        //double confidence = data[3];
        int xmin = std::stoi(data[4]);
        int ymin = std::stoi(data[5]);
        int xmax = std::stoi(data[6]);
        int ymax = std::stoi(data[7]);      
        double cam_lat = std::stod(data[8]);
        double cam_lon = std::stod(data[9]);     
        std::vector<POI> pois;
        Polar2 relatives;
        double poi_confidences;

        // poi 
        double x = (xmin + xmax) / 2.0;
        double y = (ymin + ymax) / 2.0;
        double w = xmax - xmin;
        double h = ymax - ymin;

        bool ok = localizer.applyLoc(data, pois, relatives, poi_confidences);        
        
        // Record the current state on the CSV file
        if (out_traj != nullptr)
        {
            if(!pois.empty())
            {
                printf("%d,%.3lf,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,(int)pois[0].id,pois[0].x,pois[0].y,relatives.lin,relatives.ang,poi_confidences);
                fprintf(out_traj, "%d,%.3lf,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,(int)pois[0].id,pois[0].x,pois[0].y,relatives.lin,relatives.ang,poi_confidences);       
                // printf("%d,%.3lf,%s,%.2lf,%.2lf,%.2lf,%.2lf,%d,%s,%.2lf,%.2lf,%d,%.2lf,%.2lf,%.2lf,%.7lf,%.7lf\n", fnumber,timestamp,dname.c_str(),x,y,w,h,(int)poi->id,(converter.to_bytes(poi->name)).c_str(),poi->x,poi->y,poi->floor,relatives.lin,relatives.ang,poi_confidences,cam_lat,cam_lon);
                // fprintf(out_traj, "%d,%.3lf,%s,%.2lf,%.2lf,%.2lf,%.2lf,%d,%s,%.2lf,%.2lf,%d,%.2lf,%.2lf,%.2lf,%.7lf,%.7lf\n", fnumber,timestamp,dname.c_str(),x,y,w,h,(int)poi->id,(converter.to_bytes(poi->name)).c_str(),poi->x,poi->y,poi->floor,relatives.lin,relatives.ang,poi_confidences,cam_lat,cam_lon);
            }
            // else
            // {
            //     printf("%d,%.3lf,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,0,0.00,0.00,0.00,0.00,0.00);
            //     fprintf(out_traj, "%d,%.3lf,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,0,0.00,0.00,0.00,0.00,0.00);
            // }
        }
    }
    
    if (out_traj != nullptr) fclose(out_traj);

    return 0;
}


int runOCRLocalizerReal(const std::string& site, DataLoader& data_loader, const std::string& rec_traj_file = "")
{
    // Define GUI properties for ETRI and COEX sites
    MapGUIProp ETRI;
    ETRI.image_file = "data/ETRI/NaverMap_ETRI(Satellite).png";
    ETRI.map_file = "data/ETRI/TopoMap_ETRI_210803.csv";
    ETRI.origin_latlon = dg::LatLon(36.379208, 127.364585);
    ETRI.origin_px = cv::Point2d(1344, 1371);
    ETRI.image_scale = cv::Point2d(1.2474, 1.2474);
    ETRI.image_rotation = cx::cvtDeg2Rad(0.95);
    ETRI.map_view_offset = cv::Point(1289, 371);
    ETRI.map_radius = 1500; // meter
    ETRI.grid_unit_pos = cv::Point(-215, -6);
    ETRI.video_resize = 0.2;
    ETRI.video_offset = cv::Point(350, 658);
    ETRI.result_resize = 0.5;

    MapGUIProp COEX;
    COEX.image_file = "data/NaverMap_COEX(Satellite)_200929.png";
    COEX.image_scale = cv::Point2d(1.055, 1.055);
    COEX.image_rotation = cx::cvtDeg2Rad(1.2);
    COEX.origin_latlon = dg::LatLon(37.506207, 127.05482);
    COEX.origin_px = cv::Point2d(1090, 1018);
    COEX.map_radius = 1500; // meter
    COEX.grid_unit_pos = cv::Point(-230, -16);
    COEX.map_file = "data/COEX/TopoMap_COEX_210803.csv";
    COEX.video_resize = 0.4;
    COEX.video_offset = cv::Point(10, 50);
    COEX.result_resize = 0.6;

    MapGUIProp Bucheon;
    Bucheon.image_file = "data/NaverMap_Bucheon(Satellite).png";
    Bucheon.image_scale = cv::Point2d(1.056, 1.056);
    Bucheon.image_rotation = cx::cvtDeg2Rad(0);
    Bucheon.origin_latlon = dg::LatLon(37.510928, 126.764344);
    Bucheon.origin_px = cv::Point2d(1535, 1157);
    Bucheon.map_radius = 1500; // meter
    Bucheon.grid_unit_pos = cv::Point(-215, -6);
    Bucheon.map_file = "data/Bucheon/TopoMap_Bucheon_210803.csv";
    Bucheon.video_resize = 0.25;
    Bucheon.video_offset = cv::Point(270, 638);
    Bucheon.result_resize = 0.4;

    MapGUIProp guiprop = (cx::toLowerCase(site) == "coex") ? COEX : (cx::toLowerCase(site) == "bucheon") ? Bucheon : ETRI;
    
    // Prepare a map if given
    Map map;
    map.setReference(guiprop.origin_latlon);
    if (!guiprop.map_file.empty())
    {
        VVS_CHECK_TRUE(map.load(guiprop.map_file.c_str()));
    }

    SharedInterface* shared = nullptr;
    shared = new ModuleRunner();
    shared->setMap(map);
    
    // Initialize Python module
    LOCALIZER localizer;
    if (!localizer.initialize_without_python(shared)) return -1;
    printf("Initialization: it took %.3lf seconds\n\n\n", localizer.procTime());
    
    // Run the Python module
    runOCRloc(localizer, data_loader, rec_traj_file);

    // Clear the Python module
    localizer.clear();

    delete shared;
    
    return 0;
}


int testOCRLocalizer()
{
    std::string gps_file, imu_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file;

    //bool enable_gps = false;
    //bool use_novatel = false;

    int data_sel = 5;
    double start_time = 0;     // time skip (seconds)
 
    std::vector<std::string> data_head[] = {
        {"data/ETRI/191115_151140", "1.75"},    // 0, 11296 frames, 1976 sec, video_scale = 1.75
        {"data/ETRI/200219_150153", "1.6244"},  // 1, 23911 frames, 3884 sec, video_scale = 1.6244
        {"data/ETRI/200326_132938", "1.6694"},  // 2, 18366 frames, 3066 sec, video_scale = 1.6694
        {"data/ETRI/200429_131714", "1.6828"},  // 3, 13953 frames, 2348 sec, video_scale = 1.6828
        {"data/ETRI/200429_140025", "1.6571"},  // 4, 28369 frames, 4701 sec, video_scale = 1.6571
        {"data/COEX/201007_142326", "2.8918"},  // 5, 12435 frames, 1240 sec, video_scale = 2.8918
        {"data/COEX/201007_145022", "2.869"},   // 6, 18730 frames, 1853 sec, video_scale = 2.869
        {"data/COEX/201007_152840", "2.8902"}   // 7, 20931 frames, 2086 sec, video_scale = 2.8902
    };
    const int coex_idx = 5;
    const std::string site = (data_sel < coex_idx) ? "ETRI" : "COEX";
    std::string video_file;// = (data_sel < coex_idx) ? data_head[data_sel][0] + "_images.avi" : data_head[data_sel][0] + "_images.mkv";
    // if (enable_gps && !use_novatel) gps_file = data_head[data_sel][0] + "_ascen_fix.csv";
    // if (enable_gps && use_novatel) gps_file = data_head[data_sel][0] + "_novatel_fix.csv";
    poi_file = data_head[data_sel][0] + "_ocr.csv";

    dg::DataLoader data_loader;
    if (!data_loader.load(video_file, gps_file, imu_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file))
    {
        printf("Failed to load data file\n");
        return -1;
    }
    data_loader.setStartSkipTime(start_time);

    poi_file = data_head[data_sel][0] + "_poi.csv";

    return runOCRLocalizerReal(site, data_loader, poi_file);
}


int runOCRrec(RECOGNIZER recognizer, DataLoader4Rec& data_loader, const std::string& rec_traj_file = "")
{
    // Prepare the result trajectory
    FILE* out_traj = nullptr;
    
    if (!rec_traj_file.empty())
    {
        out_traj = fopen(rec_traj_file.c_str(), "wt");//, ccs=UTF-8");
        if (out_traj == nullptr) return -1;
        fprintf(out_traj, "fnumber,timestamp,dname,confidence,xmin,ymin,xmax,ymax,lat,lon\n");
    }
    
    int type;
    std::vector<double> data;
    dg::Timestamp data_time;
    cv::Mat video_image;
    //double timestart = data_loader.getStartTime();
    while (1)
    {
        video_image = data_loader.getNextFrame(data_time);
        if (video_image.empty() == true) break;        
        int fnumber = data_loader.getFrameNumber(data_time);

        data_loader.getNextUntil(data_time, type, data, data_time);
        dg::LatLon gps_datum(data[1], data[2]);        

        bool ok = recognizer.apply(video_image, data_time);//ts);
        std::vector<OCRResult> result;
        result.clear();
        recognizer.get(result);
        
        // Record the current state on the CSV file
        if (out_traj != nullptr)
        {
            if(result.size() != 0)
            {                
                for (int k = 0; k < result.size(); k++)
                {
                    printf("%d,%.3lf,%s,%.2lf,%d,%d,%d,%d,%.7lf,%.7lf\n", fnumber, data_time, result[k].label.c_str(), result[k].confidence, result[k].xmin, result[k].ymin, result[k].xmax, result[k].ymax, (double)gps_datum.lat, (double)gps_datum.lon);
                    fprintf(out_traj, "%d,%.3lf,%s,%.2lf,%d,%d,%d,%d,%.7lf,%.7lf\n", fnumber, data_time, result[k].label.c_str(), result[k].confidence, result[k].xmin, result[k].ymin, result[k].xmax, result[k].ymax, (double)gps_datum.lat, (double)gps_datum.lon);
                }
            }
            // else
            // {   
            //     printf("%d,%.3lf,%s,%.2lf,%d,%d,%d,%d,%.7lf,%.7lf\n", fnumber, data_time, "", 0.00, 0, 0, 0, 0, (double)gps_datum.lat, (double)gps_datum.lon);
            //     fprintf(out_traj, "%d,%.3lf,%s,%.2lf,%d,%d,%d,%d,%.7lf,%.7lf\n", fnumber, data_time, "", 0.00, 0, 0, 0, 0, (double)gps_datum.lat, (double)gps_datum.lon);
            // }
        }
    }
    
    if (out_traj != nullptr) fclose(out_traj);

    return 0;
}


int runOCRRecognizerReal(DataLoader4Rec& data_loader, const std::string& rec_traj_file = "")
{
    // initialize python environment
    dg::init_python_environment("python3", "", false);

    // Initialize Python module
    RECOGNIZER recognizer;
    if (!recognizer.initialize()) return -1;
    printf("Initialization: it took %.3lf seconds\n\n\n", recognizer.procTime());

    // Run the Python module
    runOCRrec(recognizer, data_loader, rec_traj_file);

    // Clear the Python module
    recognizer.clear();

    // Close the Python Interpreter
    dg::close_python_environment();

    return 0;
}


int testOCRRecognizer()
{
    std::string gps_file, imu_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file;

    bool enable_gps = true;
    bool use_novatel = false;

    int data_sel = 0;
    double start_time = 0;     // time skip (seconds)
 
    std::vector<std::string> data_head[] = {
        {"data/ETRI/191115_151140", "1.75"},    // 0, 11296 frames, 1976 sec, video_scale = 1.75
        {"data/ETRI/200219_150153", "1.6244"},  // 1, 23911 frames, 3884 sec, video_scale = 1.6244
        {"data/ETRI/200326_132938", "1.6694"},  // 2, 18366 frames, 3066 sec, video_scale = 1.6694
        {"data/ETRI/200429_131714", "1.6828"},  // 3, 13953 frames, 2348 sec, video_scale = 1.6828
        {"data/ETRI/200429_140025", "1.6571"},  // 4, 28369 frames, 4701 sec, video_scale = 1.6571
        {"data/COEX/201007_142326", "2.8918"},  // 5, 12435 frames, 1240 sec, video_scale = 2.8918
        {"data/COEX/201007_145022", "2.869"},   // 6, 18730 frames, 1853 sec, video_scale = 2.869
        {"data/COEX/201007_152840", "2.8902"}   // 7, 20931 frames, 2086 sec, video_scale = 2.8902
    };
    const int coex_idx = 5;
    std::string video_file = (data_sel < coex_idx) ? data_head[data_sel][0] + "_images.avi" : data_head[data_sel][0] + "_images.mkv";
    if (enable_gps && !use_novatel) gps_file = data_head[data_sel][0] + "_ascen_fix.csv";
    if (enable_gps && use_novatel) gps_file = data_head[data_sel][0] + "_novatel_fix.csv";

    //dg::DataLoader data_loader;
    DataLoader4Rec data_loader;
    if (!data_loader.load(video_file, gps_file, imu_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file))
    {
        printf("Failed to load data file\n");
        return -1;
    }
    data_loader.setStartSkipTime(start_time);

    poi_file = data_head[data_sel][0] + "_ocr.csv"; //"_poi.csv";

    return runOCRRecognizerReal(data_loader, poi_file);
}


int main()
{
    return testOCRLocalizer();
    //return testOCRRecognizer();
    //return runUnitTest();    
    //return runOCRLocalizer();
}
