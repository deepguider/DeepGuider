#ifndef __DEEPGUIDER_SIMPLE__
#define __DEEPGUIDER_SIMPLE__

#define VVS_NO_ASSERT
#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_roadtheta.hpp"
#include "dg_logo.hpp"
#include "dg_ocr.hpp"
#include "dg_intersection.hpp"
#include "dg_vps.hpp"
#include "dg_guidance.hpp"
#include "dg_exploration.hpp"
#include "dg_utils.hpp"
#include "lrpose_recog/lrpose_recognizer.hpp"
#include "localizer/data_loader.hpp"
#include <chrono>

using namespace dg;
using namespace std;

class DeepGuider : public SharedInterface, public cx::Algorithm
{
protected:
    // configuable parameters
    bool m_enable_intersection = false;
    bool m_enable_vps = false;
    bool m_enable_vps_lr = false;
    bool m_enable_logo = false;
    bool m_enable_ocr = false;
    bool m_enable_roadtheta = false;
    bool m_enable_exploration = false;
    bool m_enable_imu = true;
    bool m_enable_mapserver = true;

    std::string m_server_ip = "127.0.0.1";  // default: 127.0.0.1 (localhost)
    std::string m_srcdir = "./../src";      // path of deepguider/src (required for python embedding)
    bool m_enable_tts = false;
    bool m_threaded_run_python = true;
    bool m_use_high_precision_gps = false;  // use high-precision gps (novatel)

    bool m_data_logging = false;
    bool m_recording = false;
    int m_recording_fps = 15;
    std::string m_recording_header_name = "dg_simple_";

    std::string m_map_image_path = "data/NaverMap_ETRI(Satellite)_191127.png";
    std::string m_map_data_path = "data/ETRI/TopoMap_ETRI_210803.csv";
    dg::LatLon m_map_ref_point = dg::LatLon(36.383837659737, 127.367880828442);
    double m_map_pixel_per_meter = 1.039;
    double m_map_image_rotation = cx::cvtDeg2Rad(1.0);
    dg::Point2 m_map_canvas_offset = dg::Point2(347, 297);
    std::string m_gps_input_path = "data/191115_ETRI_asen_fix.csv";
    std::string m_video_input_path = "video/191115_ETRI.avi";

public:
    DeepGuider() {}
    virtual ~DeepGuider();

    bool initialize(std::string config_file);
    int run();

    virtual Pose2 getPose(Timestamp* timestamp = nullptr) const;
    virtual LatLon getPoseGPS(Timestamp* timestamp = nullptr) const;
    virtual TopometricPose getPoseTopometric(Timestamp* timestamp = nullptr) const;
    virtual double getPoseConfidence(Timestamp* timestamp = nullptr) const;
    virtual bool procOutOfPath(const Point2& curr_pose);

    void procMouseEvent(int evt, int x, int y, int flags);
    void procTTS();

protected:
    virtual int readParam(const cv::FileNode& fn);

    // internal api's
    bool initializeDefaultMap();
    bool setDeepGuiderDestination(dg::LatLon gps_dest);
    bool updateDeepGuiderPath(dg::LatLon gps_start, dg::LatLon gps_dest);
    void drawGuiDisplay(cv::Mat& gui_image);
    void drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect);
    void drawLogo(cv::Mat target_image, std::vector<LogoResult> pois, cv::Size original_image_size);
    void drawOcr(cv::Mat target_image, std::vector<OCRResult> pois, cv::Size original_image_size);
    void drawIntersection(cv::Mat image, IntersectionResult r, cv::Size original_image_size);
    void procGpsData(dg::LatLon gps_datum, dg::Timestamp ts);
    void procImuData(double ori_w, double ori_x, double ori_y, double ori_z, dg::Timestamp ts);
    void procGuidance(dg::Timestamp ts);
    bool procIntersectionClassifier();
    bool procLogo();
    bool procOcr();
    bool procVps();
    bool procRoadTheta();

    // sub modules
    dg::MapManager m_map_manager;
    dg::DGLocalizer m_localizer;
    dg::VPSLocalizer m_vps;
    dg::LRPoseRecognizer m_vps_lr;
    dg::LogoRecognizer m_logo;
    dg::OCRLocalizer m_ocr;
    dg::IntersectionLocalizer m_intersection;
    dg::RoadThetaLocalizer m_roadtheta;
    dg::GuidanceManager m_guider;
    dg::ActiveNavigation m_active_nav;

    // global variables
    dg::LatLon m_gps_start;
    dg::LatLon m_gps_dest;
    bool m_dest_defined = false;
    bool m_pose_initialized = false;
    bool m_path_initialized = false;

    // local variables
    std::string m_winname = "DeepGuider";           // title of gui window
    cx::VideoWriter m_video_gui;
    cx::VideoWriter m_video_cam;
    std::ofstream m_log;
    cv::Mutex m_log_mutex;
    cv::Mat m_map_image;
    cv::Mat m_map_image_original;
    dg::MapPainter m_painter;
    dg::GuidanceManager::Motion m_guidance_cmd = dg::GuidanceManager::Motion::STOP;
    dg::GuidanceManager::GuideStatus m_guidance_status = dg::GuidanceManager::GuideStatus::GUIDE_INITIAL;
    std::list<dg::LatLon> m_gps_history_asen;
    std::list<dg::LatLon> m_gps_history_novatel;

    // tts
    cv::Mutex m_tts_mutex;
    std::vector<std::string> m_tts_msg;
    std::thread* tts_thread = nullptr;
    static void threadfunc_tts(DeepGuider* guider);
    bool is_tts_running = false;
    void putTTS(const char* msg);

    // Thread routines
    std::thread* vps_thread = nullptr;
    std::thread* ocr_thread = nullptr;
    std::thread* logo_thread = nullptr;
    std::thread* intersection_thread = nullptr;
    std::thread* roadtheta_thread = nullptr;
    static void threadfunc_vps(DeepGuider* guider);
    static void threadfunc_ocr(DeepGuider* guider);
    static void threadfunc_logo(DeepGuider* guider);
    static void threadfunc_intersection(DeepGuider* guider);
    static void threadfunc_roadtheta(DeepGuider* guider);
    bool is_vps_running = false;
    bool is_ocr_running = false;
    bool is_logo_running = false;
    bool is_intersection_running = false;
    bool is_roadtheta_running = false;
    void terminateThreadFunctions();

#ifdef VPSSERVER
	// curl api's
	static std::size_t curl_callback(const char* in, std::size_t size, std::size_t num, std::string* out);
	bool curl_request(const std::string url, const char * CMD, const Json::Value * post_json, Json::Value * get_json);
#endif

    // shared variables for multi-threading
    cv::Mutex m_cam_mutex;
    cv::Mat m_cam_image;
    dg::Timestamp m_cam_capture_time;
    dg::LatLon m_cam_gps;
    int m_cam_fnumber;              // frame number

    cv::Mutex m_vps_mutex;
    cv::Mat m_vps_image;            // top-1 matched streetview image
    dg::ID m_vps_id;                // top-1 matched streetview id
    double m_vps_confidence;        // top-1 matched confidence(similarity)

    cv::Mutex m_logo_mutex;
    cv::Mat m_logo_image;
    std::vector<LogoResult> m_logos;

    cv::Mutex m_ocr_mutex;
    cv::Mat m_ocr_image;
    std::vector<OCRResult> m_ocrs;

    cv::Mutex m_intersection_mutex;
    cv::Mat m_intersection_image;
    IntersectionResult m_intersection_result;

    cv::Mutex m_localizer_mutex;
    cv::Mutex m_map_mutex;
    cv::Mutex m_guider_mutex;
    int m_gps_update_cnt = 0;

    // guidance icons
    cv::Mat m_icon_forward;
    cv::Mat m_mask_forward;
    cv::Mat m_icon_turn_left;
    cv::Mat m_mask_turn_left;
    cv::Mat m_icon_turn_right;
    cv::Mat m_mask_turn_right;
    cv::Mat m_icon_turn_back;
    cv::Mat m_mask_turn_back;

    dg::ID id_invalid = 0;
    Polar2 rel_pose_defualt = Polar2(-1, CV_PI);     // default relative pose (invalid)
    double confidence_default = -1.0;
};

void onMouseEvent(int event, int x, int y, int flags, void* param)
{
    DeepGuider* dg = (DeepGuider *)param;
    dg->procMouseEvent(event, x, y, flags);
}

DeepGuider::~DeepGuider()
{
    if (m_enable_intersection) m_intersection.clear();
    if (m_enable_vps) m_vps.clear();
    if(m_enable_logo) m_logo.clear();
    if (m_enable_ocr) m_ocr.clear();
    if (m_enable_roadtheta) m_roadtheta.clear();

    bool enable_python = m_enable_vps || m_enable_logo || m_enable_ocr || m_enable_intersection || m_enable_exploration;
    if(enable_python) close_python_environment();
}

int DeepGuider::readParam(const cv::FileNode& fn)
{
    int n_read = cx::Algorithm::readParam(fn);

    // Read Activate/deactivate Options
    CX_LOAD_PARAM_COUNT(fn, "enable_intersection", m_enable_intersection, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_vps", m_enable_vps, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_vps_lr", m_enable_vps_lr, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_poi_logo", m_enable_logo, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_poi_ocr", m_enable_ocr, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_roadtheta", m_enable_roadtheta, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_exploration", m_enable_exploration, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_imu", m_enable_imu, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_mapserver", m_enable_mapserver, n_read);

    // Read Main Options
    int server_ip_index = -1;
    std::vector<cv::String> server_ip_list;
    CX_LOAD_PARAM_COUNT(fn, "server_ip_list", server_ip_list, n_read);
    CX_LOAD_PARAM_COUNT(fn, "server_ip_index", server_ip_index, n_read);
    if (server_ip_index >= 0 && server_ip_index < server_ip_list.size()) m_server_ip = server_ip_list[server_ip_index];

    int site_index = -1;
    std::string site_tagname;
    std::vector<cv::String> site_names;
    CX_LOAD_PARAM_COUNT(fn, "site_names", site_names, n_read);
    CX_LOAD_PARAM_COUNT(fn, "site_index", site_index, n_read);
    if (site_index >= 0 && site_index < site_names.size()) site_tagname = site_names[site_index];

    CX_LOAD_PARAM_COUNT(fn, "dg_srcdir", m_srcdir, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_tts", m_enable_tts, n_read);
    CX_LOAD_PARAM_COUNT(fn, "threaded_run_python", m_threaded_run_python, n_read);
    CX_LOAD_PARAM_COUNT(fn, "use_high_precision_gps", m_use_high_precision_gps, n_read);

    // Read Other Options
    CX_LOAD_PARAM_COUNT(fn, "enable_data_logging", m_data_logging, n_read);
    CX_LOAD_PARAM_COUNT(fn, "video_recording", m_recording, n_read);
    CX_LOAD_PARAM_COUNT(fn, "video_recording_fps", m_recording_fps, n_read);
    CX_LOAD_PARAM_COUNT(fn, "recording_header_name", m_recording_header_name, n_read);

    // Read Site-specific Setting
    CX_LOAD_PARAM_COUNT(fn, "map_image_path", m_map_image_path, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_data_path", m_map_data_path, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_lat", m_map_ref_point.lat, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_lon", m_map_ref_point.lon, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_pixel_per_meter", m_map_pixel_per_meter, n_read);
    double map_image_rotation = m_map_image_rotation;
    CX_LOAD_PARAM_COUNT(fn, "map_image_rotation", map_image_rotation, n_read);
    if(map_image_rotation != m_map_image_rotation) m_map_image_rotation = cx::cvtDeg2Rad(map_image_rotation);
    CX_LOAD_PARAM_COUNT(fn, "map_canvas_offset", m_map_canvas_offset, n_read);
    CX_LOAD_PARAM_COUNT(fn, "gps_input_path", m_gps_input_path, n_read);
    CX_LOAD_PARAM_COUNT(fn, "video_input_path", m_video_input_path, n_read);

    // Read Place Setting
    if (!site_tagname.empty())
    {
        cv::FileNode fn_site = fn[site_tagname];
        if (!fn_site.empty())
        {
            n_read += DeepGuider::readParam(fn_site);
        }
    }
    return n_read;
}

bool DeepGuider::initialize(std::string config_file)
{
    printf("Initialize deepguider system...\n");

    // load config
    bool ok = loadParam(config_file);
    if(ok) printf("\tConfiguration %s loaded!\n", config_file.c_str());

    // initialize python
    bool enable_python = m_enable_vps || m_enable_ocr || m_enable_logo || m_enable_intersection || m_enable_exploration;
    if (enable_python && !init_python_environment("python3", "", m_threaded_run_python)) return false;
    if(enable_python) printf("\tPython environment initialized!\n");

    // initialize map manager
    m_map_manager.setReference(m_map_ref_point);
    if (m_enable_mapserver && !m_map_manager.initialize(m_server_ip)) return false;
    if (m_enable_mapserver) printf("\tMapManager initialized!\n");

    // initialize VPS
    std::string py_module_path = m_srcdir + "/vps";
    if (m_enable_vps && !m_vps.initialize(this, m_server_ip, py_module_path)) return false;
    if (m_enable_vps) printf("\tVPS initialized in %.3lf seconds!\n", m_vps.procTime());

    // initialize OCR
    py_module_path = m_srcdir + "/ocr_recog";
    if (m_enable_ocr && !m_ocr.initialize(this, py_module_path)) return false;
    if (m_enable_ocr) printf("\tOCR initialized in %.3lf seconds!\n", m_ocr.procTime());

    // initialize Intersection
    py_module_path = m_srcdir + "/intersection_cls";
    if (m_enable_intersection && !m_intersection.initialize(this, py_module_path)) return false;
    if (m_enable_intersection) printf("\tIntersection initialized in %.3lf seconds!\n", m_intersection.procTime());

    // initialize Logo
    py_module_path = m_srcdir + "/logo_recog";
    if (m_enable_logo && !m_logo.initialize("logo_recognizer", py_module_path.c_str())) return false;
    if (m_enable_logo) printf("\tLogo initialized in %.3lf seconds!\n", m_logo.procTime());

    // initialize RoadTheta
    if (m_enable_roadtheta && !m_roadtheta.initialize(this)) return false;
    if (m_enable_roadtheta) printf("\tRoadTheta initialized in %.3lf seconds!\n", m_roadtheta.procTime());

    //initialize exploation 
    if (m_enable_exploration && !m_active_nav.initialize()) return false;
    if (m_enable_exploration) printf("\tExploation initialized!\n");

    // initialize default map
    if (m_enable_mapserver) initializeDefaultMap();
    else
    {
        Map map;
        map.setReference(m_map_ref_point);
        bool ok = map.load(m_map_data_path.c_str());
        if (ok) setMap(map);
    }

    // initialize localizer
    if (!m_localizer.initialize(this, "EKFLocalizer")) return false;
    if (!m_localizer.setParamMotionNoise(1, 10)) return false;      // linear_velocity(m), angular_velocity(deg)
    if (!m_localizer.setParamGPSNoise(4)) return false;             // position error(m)
    if (!m_localizer.setParamGPSOffset(1, 0)) return false;         // displacement(lin,ang) from robot origin
    if (!m_localizer.setParamIMUCompassNoise(1, 0)) return false;   // angle arror(deg), angle offset(deg)
    if (!m_localizer.setParamPOINoise(2, 10, 1)) return false;      // rel. distance error(m), rel. orientation error(deg), position error of poi info (m)
    if (!m_localizer.setParamVPSNoise(2, 10, 1)) return false;      // rel. distance error(m), rel. orientation error(deg), position error of poi info (m)
    if (!m_localizer.setParamIntersectClsNoise(0.1)) return false;  // position error(m)
    if (!m_localizer.setParamRoadThetaNoise(10, 0)) return false;   // angle arror(deg), angle offset(deg)
    if (!m_localizer.setParamCameraOffset(1, 0)) return false;      // displacement(lin,ang) from robot origin
    m_localizer.setParamValue("gps_reverse_vel", -1);
    m_localizer.setParamValue("search_turn_weight", 100);
    m_localizer.setParamValue("track_near_radius", 20);
    m_localizer.setParamValue("enable_path_projection", true);
    m_localizer.setParamValue("enable_map_projection", false);
    m_localizer.setParamValue("enable_backtracking_ekf", true);
    m_localizer.setParamValue("enable_gps_smoothing)", true);
    printf("\tLocalizer initialized!\n");

    // initialize guidance
    if (!m_guider.initialize(this)) return false;
    printf("\tGuidance initialized!\n");

    // load background GUI image
    m_map_image = cv::imread(m_map_image_path);
    VVS_CHECK_TRUE(!m_map_image.empty());
    m_map_image_original = m_map_image.clone();

    // prepare GUI map
    m_painter.configCanvas(m_map_canvas_offset, cv::Point2d(m_map_pixel_per_meter, m_map_pixel_per_meter), m_map_image.size(), 0, 0);
    m_painter.setImageRotation(m_map_image_rotation);
    m_painter.drawGrid(m_map_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, cv::Point(-215, -6));
    m_painter.drawOrigin(m_map_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    m_painter.setParamValue("node_radius", 4);
    m_painter.setParamValue("node_font_scale", 0);
    m_painter.setParamValue("node_color", { 255, 50, 255 });
    m_painter.setParamValue("edge_color", { 200, 100, 100 });
    m_painter.setParamValue("crosswalk_color", { 0, 150, 50 });
    m_painter.setParamValue("edge_thickness", 2);
    VVS_CHECK_TRUE(m_painter.drawMap(m_map_image, m_map));

    // load icon images
    m_icon_forward = cv::imread("data/forward.png");
    m_icon_turn_left = cv::imread("data/turn_left.png");
    m_icon_turn_right = cv::imread("data/turn_right.png");
    m_icon_turn_back = cv::imread("data/turn_back.png");
    cv::threshold(m_icon_forward, m_mask_forward, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(m_icon_turn_left, m_mask_turn_left, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(m_icon_turn_right, m_mask_turn_right, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(m_icon_turn_back, m_mask_turn_back, 250, 1, cv::THRESH_BINARY_INV);

    // show GUI window
    cv::namedWindow(m_winname, cv::WINDOW_NORMAL);
    cv::setMouseCallback(m_winname, onMouseEvent, this);
    cv::resizeWindow(m_winname, m_map_image.cols, m_map_image.rows);
    cv::imshow(m_winname, m_map_image);
    cv::waitKey(1);

    // init video recording
    time_t start_t;
    time(&start_t);
    tm _tm = *localtime(&start_t);
    char sztime[255];
    strftime(sztime, 255, "%y%m%d_%H%M%S", &_tm);
    if (m_recording)
    {
        std::string filename = m_recording_header_name + sztime + "_gui.avi";
        m_video_gui.open(filename, m_recording_fps);
    }

    // init data logging
    if (m_data_logging)
    {
        std::string filename = m_recording_header_name + sztime + ".txt";
        m_log.open(filename, ios::out);

        std::string filename_cam = m_recording_header_name + sztime + "_cam.avi";
        m_video_cam.open(filename_cam, m_recording_fps);
    }

    // reset interval variables
    m_dest_defined = false;
    m_pose_initialized = false;
    m_path_initialized = false;
    m_gps_update_cnt = 0;
    m_cam_image.release();
    m_cam_capture_time = -1;
    m_cam_fnumber = -1;
    m_vps_image.release();
    m_vps_id = 0;
    m_vps_confidence = 0;
    m_logo_image.release();
    m_logos.clear();
    m_ocr_image.release();
    m_ocrs.clear();
    m_intersection_image.release();
    m_gps_history_asen.clear();
    m_gps_history_novatel.clear();
    m_guidance_cmd = dg::GuidanceManager::Motion::STOP;
    m_guidance_status = dg::GuidanceManager::GuideStatus::GUIDE_INITIAL;

    // tts
    if (m_enable_tts)
    {
        tts_thread = new std::thread(threadfunc_tts, this);
        putTTS("System is initialized!");
    } 

    printf("\tInitialization is done!\n\n");

    return true;
}

bool DeepGuider::initializeDefaultMap()
{
    double radius = 2000;   // meter
    Map map;
    map.reserveMemory();
    VVS_CHECK_TRUE(m_map_manager.getMapAll(m_map_ref_point.lat, m_map_ref_point.lon, radius, map));
    setMap(map);
    printf("\tDefault map is downloaded: nodes=%d, edges=%d, pois=%d, views=%d\n", m_map->countNodes(), m_map->countEdges(), m_map->countPOIs(), m_map->countViews());

    return true;
}

std::vector<std::pair<double, dg::LatLon>> loadExampleGPSData(std::string csv_file)
{
    const string ANDRO_POSTFIX = "AndroSensor.csv";
    cx::CSVReader csv;
    std::vector<std::pair<double, dg::LatLon>> data;
    const string postfix = csv_file.substr(csv_file.length() - ANDRO_POSTFIX.length(), ANDRO_POSTFIX.length());
    if (postfix.compare(ANDRO_POSTFIX) == 0)
    {
        VVS_CHECK_TRUE(csv.open(csv_file, ';'));
        cx::CSVReader::Double2D csv_ext = csv.extDouble2D(2, { 31, 22, 23, 28 }); // Skip the header

        for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
        {
            double timestamp = 1e-3 * row->at(0);
            dg::LatLon ll(row->at(1), row->at(2));
            data.push_back(std::make_pair(timestamp, ll));
        }
    }
    else
    {
        VVS_CHECK_TRUE(csv.open(csv_file));
        cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 2, 3, 7, 8 }); // Skip the header

        for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
        {
            double timestamp = row->at(0) + 1e-9 * row->at(1);
            dg::LatLon ll(row->at(2), row->at(3));
            data.push_back(std::make_pair(timestamp, ll));
        }
    }
    return data;
}


int DeepGuider::run()
{
    // load test dataset
    dg::DataLoader data_loader;
    if (!data_loader.load(m_video_input_path, m_gps_input_path))
    {
        printf("DeepGuider::run() - Fail to load test data. Exit program...\n");
        return -1;
    }
    printf("Run deepguider system...\n");

    // set initial destination
    //dg::LatLon gps_dest = gps_data.back().second;
    //VVS_CHECK_TRUE(setDeepGuiderDestination(gps_dest));

    cv::Mat video_image;
    Timestamp capture_time;
    int itr = 0;
    while (1)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // get next data
        int type;
        std::vector<double> data;
        dg::Timestamp data_time;
        if (data_loader.getNext(type, data, data_time) == false) break;

        // process data
        bool update_gui = false;
        if (type == dg::DATA_GPS)
        {
            const dg::LatLon gps_datum(data[1], data[2]);
            procGpsData(gps_datum, data_time);
            m_painter.drawPoint(m_map_image, toMetric(gps_datum), 2, cv::Vec3b(0, 255, 0));
            m_gps_history_asen.push_back(gps_datum);
            printf("[GPS] lat=%lf, lon=%lf, ts=%lf\n", gps_datum.lat, gps_datum.lon, data_time);

            video_image = data_loader.getFrame(data_time, &capture_time);
            update_gui = true;
        }
        else if (type == dg::DATA_IMU)
        {
            auto euler = cx::cvtQuat2EulerAng(data[1], data[2], data[3], data[4]);
            procImuData(data[1], data[2], data[3], data[4], data_time);
        }
        else if (type == dg::DATA_POI)
        {
            dg::Point2 clue_xy(data[1], data[2]);
            dg::Polar2 relative(data[3], data[4]);
            double confidence = data[5];
            //bool success = m_localizer.applyPOI(clue_xy, relative, data_time, confidence);
            //if (!success) fprintf(stderr, "applyPOI() was failed.\n");
        }
        else if (type == dg::DATA_VPS)
        {
            dg::Point2 clue_xy(data[1], data[2]);
            dg::Polar2 relative(data[3], data[4]);
            double confidence = data[5];
            //bool success = m_localizer.applyVPS(clue_xy, relative, data_time, confidence);
            //if (!success) fprintf(stderr, "applyVPS() was failed.\n");
        }
        else if (type == dg::DATA_IntersectCls)
        {
            dg::Point2 clue_xy(data[1], data[2]);
            dg::Polar2 relative(data[3], data[4]);
            double confidence = data[5];
            //bool success = m_localizer.applyIntersectCls(clue_xy, relative, data_time, confidence);
            //if (!success) fprintf(stderr, "applyIntersectCls() was failed.\n");
        }
        else if (type == dg::DATA_LR)
        {
            double lr_result = data[1];
            double confidence = data[2];
            //bool success = m_localizer.applyVPS_LR(lr_result, data_time, confidence);
            //if (!success) fprintf(stderr, "applyVPS_LR() was failed.\n");
        }
        else if (type == dg::DATA_RoadTheta)
        {
            double theta = data[1];
            double confidence = data[2];
            //bool success = m_localizer.applyRoadTheta(theta, data_time, confidence);
            //if (!success) fprintf(stderr, "applyRoadTheta() was failed.\n");
        }

        // update
        if (update_gui)
        {
            // draw robot trajectory
            Pose2 pose_m = getPose();
            m_painter.drawPoint(m_map_image, pose_m, 1, cv::Vec3b(0, 0, 255));

            m_cam_mutex.lock();
            m_cam_image = video_image;
            m_cam_capture_time = capture_time;
            m_cam_gps = getPoseGPS();
            m_cam_fnumber++;
            m_cam_mutex.unlock();

            // process vision modules
            if (m_enable_roadtheta) procRoadTheta();
            if (m_enable_vps) procVps();
            if (m_enable_logo) procLogo();
            if (m_enable_ocr) procOcr();
            if (m_enable_intersection) procIntersectionClassifier();

            // process Guidance
            procGuidance(data_time);

            // draw GUI display
            cv::Mat gui_image = m_map_image.clone();
            drawGuiDisplay(gui_image);

            // recording
            if (m_recording) m_video_gui << gui_image;
            if (m_data_logging) m_video_cam << m_cam_image;

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            printf("Iteration: %d (it took %lf seconds)\n", itr, t2 - t1);

            // gui display
            cv::imshow(m_winname, gui_image);
            int key = cv::waitKey(1);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == cx::KEY_ESC) break;
            if (key == 83) itr += 30;   // Right Key

            // update iteration
            itr++;

            // flush out logging data
            if (m_data_logging) m_log.flush();
        }
    }

    // end system
    printf("End deepguider system...\n");
    terminateThreadFunctions();
    printf("\tthread terminated\n");
    if(m_recording) m_video_gui.release();
    if(m_data_logging) m_video_cam.release();
    printf("\tclose recording\n");
    cv::destroyWindow(m_winname);
    printf("\tgui window destroyed\n");
    printf("all done!\n");    

    return 0;
}

Pose2 DeepGuider::getPose(Timestamp* timestamp) const
{
    return m_localizer.getPose(timestamp);
}

LatLon DeepGuider::getPoseGPS(Timestamp* timestamp) const
{
    return m_localizer.getPoseGPS(timestamp);
}

TopometricPose DeepGuider::getPoseTopometric(Timestamp* timestamp) const
{
    return m_localizer.getPoseTopometric(timestamp);
}

double DeepGuider::getPoseConfidence(Timestamp* timestamp) const
{
    return m_localizer.getPoseConfidence(timestamp);
}

bool DeepGuider::procOutOfPath(const Point2& curr_pose)
{
    if (!m_dest_defined) return false;
    dg::LatLon pose_gps = toLatLon(curr_pose);
    return updateDeepGuiderPath(pose_gps, m_gps_dest);
}

void DeepGuider::procGpsData(dg::LatLon gps_datum, dg::Timestamp ts)
{    
    // apply gps to localizer
    m_localizer_mutex.lock();
    VVS_CHECK_TRUE(m_localizer.applyGPS(gps_datum, ts));
    double pose_confidence = m_localizer.getPoseConfidence();
    m_localizer_mutex.unlock();    

    // check pose initialization
    m_gps_update_cnt++;
    if (!m_pose_initialized && pose_confidence > 0.2 && m_gps_update_cnt > 10)
    {
        m_pose_initialized = true;
        if(m_dest_defined)
        {
            setDeepGuiderDestination(m_gps_dest);
        }
        printf("[Localizer] initial pose is estimated!\n");
    }
}

void DeepGuider::procImuData(double ori_w, double ori_x, double ori_y, double ori_z, dg::Timestamp ts)
{
    auto euler = cx::cvtQuat2EulerAng(ori_w, ori_x, ori_y, ori_z);
    m_localizer_mutex.lock();
    VVS_CHECK_TRUE(m_localizer.applyIMUCompass(euler.z, ts));
    m_localizer_mutex.unlock();
}

void DeepGuider::procMouseEvent(int evt, int x, int y, int flags)
{
    if (evt == cv::EVENT_MOUSEMOVE)
    {
    }
    else if (evt == cv::EVENT_LBUTTONDOWN)
    {
    }
    else if (evt == cv::EVENT_LBUTTONUP)
    {
    }
    else if (evt == cv::EVENT_LBUTTONDBLCLK)
    {
        dg::LatLon ll = toLatLon(m_painter.cvtPixel2Value(cv::Point(x, y)));
        setDeepGuiderDestination(ll);
    }
    else if (evt == cv::EVENT_RBUTTONDOWN)
    {
    }
    else if (evt == cv::EVENT_RBUTTONUP)
    {
    }
    else if (evt == cv::EVENT_MOUSEWHEEL)
    {
    }
}


bool DeepGuider::setDeepGuiderDestination(dg::LatLon gps_dest)
{
    // check self-localized
    if(!m_pose_initialized)
    {
        m_gps_dest = gps_dest;
        m_dest_defined = true;
        return true;
    }

    // get current pose
    dg::LatLon pose_gps = toLatLon(getPose());

    // generate & apply new path
    bool ok = updateDeepGuiderPath(pose_gps, gps_dest);
    if(!ok) return false;

    // update system status
    m_gps_dest = gps_dest;
    m_dest_defined = true;
    m_path_initialized = true;
    return true;
}

bool DeepGuider::updateDeepGuiderPath(dg::LatLon gps_start, dg::LatLon gps_dest)
{
    if (m_enable_mapserver)
    {
        int start_floor = 0;
        int dest_floor = 0;
        Path path;
        setMapLock();
        bool ok = m_map_manager.getPath_mapExpansion(gps_start.lat, gps_start.lon, start_floor, gps_dest.lat, gps_dest.lon, dest_floor, path, *m_map);
        releaseMapLock();
        if (!ok)
        {
            printf("[MapManager] fail to find path to (lat=%lf, lon=%lf)\n", gps_dest.lat, gps_dest.lon);
            return false;
        }
        setPath(path);
        printf("[MapManager] New path generated to (lat=%lf, lon=%lf)\n", gps_dest.lat, gps_dest.lon);
    }
    else
    {
        Point2 p_start = toMetric(gps_start);
        Point2 p_dest = toMetric(gps_dest);
        Path path;
        setMapLock();
        bool ok = m_map && m_map->getPath(p_start, p_dest, path);
        releaseMapLock();
        if (!ok) return false;
        setPath(path);
    }

    // guidance: init map and path for guidance
    m_guider_mutex.lock();
    VVS_CHECK_TRUE(m_guider.initiateNewGuidance());
    m_guider_mutex.unlock();
    printf("\tGuidance is updated with new map and path!\n");

    // draw map
    m_map_image_original.copyTo(m_map_image);
    setMapLock();
    m_painter.drawMap(m_map_image, m_map);
    releaseMapLock();
    setPathLock();
    m_painter.drawPath(m_map_image, m_map, m_path);
    releasePathLock();
    for(auto itr = m_gps_history_novatel.begin(); itr != m_gps_history_novatel.end(); itr++)
    {
        m_painter.drawPoint(m_map_image, toMetric(*itr), 2, cv::Vec3b(0, 0, 255));
    }
    for(auto itr = m_gps_history_asen.begin(); itr != m_gps_history_asen.end(); itr++)
    {
        m_painter.drawPoint(m_map_image, toMetric(*itr), 2, cv::Vec3b(0, 255, 0));
    }

    printf("\tGUI map is updated with new map and path!\n");

    return true;    
}


void DeepGuider::drawGuiDisplay(cv::Mat& image)
{
    double video_resize_scale = 0.4;
    int win_delta = 10;

    // cam image
    m_cam_mutex.lock();
    cv::Mat cam_image = m_cam_image.clone();
    m_cam_mutex.unlock();

    // draw cam image as subwindow on the GUI map image
    cv::Rect win_rect;
    cv::Mat video_image;
    cv::Point video_offset(20, image.rows - 322);
    if (!cam_image.empty())
    {
        // adjust video scale not to outfit gui_image
        if (video_offset.y + cam_image.rows * video_resize_scale > image.rows - win_delta)
            video_resize_scale = (double)(image.rows - win_delta - video_offset.y) / cam_image.rows;
        if (video_offset.x + cam_image.cols * video_resize_scale > image.cols - win_delta)
            video_resize_scale = (double)(image.cols - win_delta - video_offset.x) / cam_image.cols;

        cv::resize(cam_image, video_image, cv::Size(), video_resize_scale*0.8, video_resize_scale);
        win_rect = cv::Rect(video_offset, video_offset + cv::Point(video_image.cols, video_image.rows));
        if (win_rect.br().x < image.cols && win_rect.br().y < image.rows) image(win_rect) = video_image * 1;
    }
    else
    {
        int win_w = (int)(1280 * video_resize_scale * 0.8);
        int win_h = (int)(720 * video_resize_scale);
        win_rect = cv::Rect(video_offset, video_offset + cv::Point(win_w, win_h));
    }
    cv::Rect video_rect = win_rect;

    // draw intersection result
    if (m_enable_intersection)
    {
        cv::Mat intersection_image;
        IntersectionResult intersection_result;
        cv::Size original_image_size;
        m_intersection_mutex.lock();
        if(!m_intersection_image.empty())
        {
            original_image_size.width = m_intersection_image.cols;
            original_image_size.height = m_intersection_image.rows;
            double fy = (double)win_rect.height / m_intersection_image.rows;
            cv::resize(m_intersection_image, intersection_image, cv::Size(win_rect.height, win_rect.height));
            intersection_result = m_intersection_result;
        }
        m_intersection_mutex.unlock();

        if (!intersection_image.empty())
        {
            drawIntersection(intersection_image, intersection_result, original_image_size);
            cv::Point intersection_offset = video_offset;
            intersection_offset.x = win_rect.x + win_rect.width + win_delta;
            cv::Rect rect(intersection_offset, intersection_offset + cv::Point(intersection_image.cols, intersection_image.rows));
            if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = intersection_image * 1;
            win_rect = rect;
        }
    }


    // draw vps result
    if (m_enable_vps)
    {
        // top-1 matched streetview image
        cv::Mat sv_image;
        dg::ID sv_id = 0;
        double sv_confidence = 0;
        m_vps_mutex.lock();
        if(!m_vps_image.empty())
        {
            double fy = (double)win_rect.height / m_vps_image.rows;
            cv::resize(m_vps_image, sv_image, cv::Size(), fy, fy);
            sv_id = m_vps_id;
            sv_confidence = m_vps_confidence;
        }
        m_vps_mutex.unlock();

        if (!sv_image.empty())
        {
            cv::Point sv_offset = video_offset;
            sv_offset.x = win_rect.x + win_rect.width + win_delta;
            cv::Rect rect(sv_offset, sv_offset + cv::Point(sv_image.cols, sv_image.rows));
            if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows)
            {
                image(rect) = sv_image * 1;

                cv::Point msg_offset = sv_offset + cv::Point(10, 30);
                double font_scale = 0.8;
                std::string str_confidence = cv::format("Confidence: %.2lf", sv_confidence);
                cv::putText(image, str_confidence.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), 5);
                cv::putText(image, str_confidence.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), 2);
                std::string str_id = cv::format("ID: %zu", sv_id);
                msg_offset.y += 30;
                cv::putText(image, str_id.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), 5);
                cv::putText(image, str_id.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), 2);

                win_rect = rect;
            }
        }

        // show gps position of top-1 matched image on the map
        if (sv_id > 0)
        {
            dg::StreetView* sv = m_map->getView(sv_id);
            if(sv)
            {
                m_painter.drawPoint(image, *sv, 6, cv::Vec3b(255, 255, 0));
            }
        }
    }

    // draw logo result
    if (m_enable_logo)
    {
        cv::Mat logo_image;
        std::vector<LogoResult> logos;
        cv::Size original_image_size;
        m_logo_mutex.lock();
        if(!m_logo_image.empty())
        {
            original_image_size.width = m_logo_image.cols;
            original_image_size.height = m_logo_image.rows;
            double fy = (double)win_rect.height / m_logo_image.rows;
            cv::resize(m_logo_image, logo_image, cv::Size(), fy*0.9, fy);
            logos = m_logos;
        }
        m_logo_mutex.unlock();

        if (!logo_image.empty())
        {
            drawLogo(logo_image, logos, original_image_size);
            cv::Point logo_offset = video_offset;
            logo_offset.x = win_rect.x + win_rect.width + win_delta;
            cv::Rect rect(logo_offset, logo_offset + cv::Point(logo_image.cols, logo_image.rows));
            if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = logo_image * 1;
            win_rect = rect;
        }
    }

    // draw ocr result
    if (m_enable_ocr)
    {
        m_ocr_mutex.lock();
        cv::Mat ocr_image = m_ocr_image.clone();
        std::vector<OCRResult> ocrs = m_ocrs;
        m_ocr_mutex.unlock();

        cv::Size original_image_size;
        if (!ocr_image.empty())
        {
            OCRRecognizer recognizer;
            recognizer.set(ocrs, 0, 0);
            recognizer.draw(ocr_image, 40);

            original_image_size.width = m_ocr_image.cols;
            original_image_size.height = m_ocr_image.rows;
            double fy = (double)win_rect.height / m_ocr_image.rows;
            cv::resize(ocr_image, ocr_image, cv::Size(), fy*0.9, fy);
            ocrs = m_ocrs;
        }
        //m_ocr_mutex.unlock();

        if (!ocr_image.empty())
        {
            //drawOcr(ocr_image, ocrs, original_image_size);
            cv::Point ocr_offset = video_offset;
            ocr_offset.x = win_rect.x + win_rect.width + win_delta;
            cv::Rect rect(ocr_offset, ocr_offset + cv::Point(ocr_image.cols, ocr_image.rows));
            if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = ocr_image * 1;
            win_rect = rect;
        }
    }

    // current localization
    m_localizer_mutex.lock();
    dg::Pose2 pose_metric = getPose();
    dg::TopometricPose pose_topo = getPoseTopometric();
    dg::LatLon pose_gps = toLatLon(pose_metric);
    double pose_confidence = m_localizer.getPoseConfidence();
    m_localizer_mutex.unlock();

    // draw robot on the map
    m_painter.drawPoint(image, pose_metric, 10, cx::COLOR_YELLOW);
    m_painter.drawPoint(image, pose_metric, 8, cx::COLOR_BLUE);
    dg::Point2 pose_pixel = m_painter.cvtValue2Pixel(pose_metric);
    cv::line(image, pose_pixel, pose_pixel + 10 * dg::Point2(cos(pose_metric.theta), -sin(pose_metric.theta)), cx::COLOR_YELLOW, 2);

    // draw status message (localization)
    cv::String info_topo = cv::format("Node: %zu, Edge: %d, D: %.3fm", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist);
    cv::putText(image, info_topo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
    cv::putText(image, info_topo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    std::string info_confidence = cv::format("Confidence: %.2lf", pose_confidence);
    cv::putText(image, info_confidence, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
    cv::putText(image, info_confidence, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

    // print status message (localization)
    printf("[Localizer]\n");
    printf("\ttopo: node=%zu, edge=%d, dist=%lf\n", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist);
    printf("\tmetr: x=%lf, y=%lf, theta=%lf\n", pose_metric.x, pose_metric.y, pose_metric.theta);
    printf("\tgps : lat=%lf, lon=%lf\n", pose_gps.lat, pose_gps.lon);
    printf("\tconfidence: %lf\n", pose_confidence);

    // draw guidance info
    dg::GuidanceManager::GuideStatus cur_status;
    dg::GuidanceManager::Guidance cur_guide;
    m_guider_mutex.lock();
    cur_status = m_guider.getGuidanceStatus();
    cur_guide = m_guider.getGuidance();
    m_guider_mutex.unlock();
    if (!video_image.empty())
    {
        drawGuidance(image, cur_guide, video_rect);
    }

    // check and draw arrival
    if (cur_status == GuidanceManager::GuideStatus::GUIDE_ARRIVED)
    {
        std::string msg = "ARRIVED!";
        cv::Point pt(600, 500);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0, 255, 0), 8);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0, 0, 0), 4);
    }
}


void DeepGuider::drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect)
{
    if(!m_path_initialized || !m_dest_defined) return;
    if(guide.actions.empty()) return;

    int guide_cx = rect.x + rect.width / 2;
    int guide_cy = rect.y + m_icon_forward.rows / 2 + 40;
    cv::Point center_pos(guide_cx, guide_cy);

    std::string dir_msg;
    dg::GuidanceManager::Motion cmd = guide.actions[0].cmd;
    if (cmd == dg::GuidanceManager::Motion::GO_FORWARD || cmd == dg::GuidanceManager::Motion::CROSS_FORWARD || cmd == dg::GuidanceManager::Motion::ENTER_FORWARD || cmd == dg::GuidanceManager::Motion::EXIT_FORWARD)
    {
        cv::Mat& icon = m_icon_forward;
        cv::Mat& mask = m_mask_forward;
        int x1 = center_pos.x - icon.cols / 2;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        if (cmd == dg::GuidanceManager::Motion::GO_FORWARD) dir_msg = "[Guide] GO_FORWARD";
        if (cmd == dg::GuidanceManager::Motion::CROSS_FORWARD) dir_msg = "[Guide] CROSS_FORWARD";
        if (cmd == dg::GuidanceManager::Motion::ENTER_FORWARD) dir_msg = "[Guide] ENTER_FORWARD";
        if (cmd == dg::GuidanceManager::Motion::EXIT_FORWARD) dir_msg = "[Guide] EXIT_FORWARD";
    }
    else if (cmd == dg::GuidanceManager::Motion::TURN_LEFT || cmd == dg::GuidanceManager::Motion::CROSS_LEFT || cmd == dg::GuidanceManager::Motion::ENTER_LEFT || cmd == dg::GuidanceManager::Motion::EXIT_LEFT)
    {
        cv::Mat& icon = m_icon_turn_left;
        cv::Mat& mask = m_mask_turn_left;
        int x1 = center_pos.x - icon.cols + icon.cols / 6;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        if (cmd == dg::GuidanceManager::Motion::TURN_LEFT) dir_msg = "[Guide] TURN_LEFT";
        if (cmd == dg::GuidanceManager::Motion::CROSS_LEFT) dir_msg = "[Guide] CROSS_LEFT";
        if (cmd == dg::GuidanceManager::Motion::ENTER_LEFT) dir_msg = "[Guide] ENTER_LEFT";
        if (cmd == dg::GuidanceManager::Motion::EXIT_LEFT) dir_msg = "[Guide] EXIT_LEFT";
    }
    else if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT || cmd == dg::GuidanceManager::Motion::CROSS_RIGHT || cmd == dg::GuidanceManager::Motion::ENTER_RIGHT || cmd == dg::GuidanceManager::Motion::EXIT_RIGHT)
    {
        cv::Mat& icon = m_icon_turn_right;
        cv::Mat& mask = m_mask_turn_right;
        int x1 = center_pos.x - icon.cols / 6;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT) dir_msg = "[Guide] TURN_RIGHT";
        if (cmd == dg::GuidanceManager::Motion::CROSS_RIGHT) dir_msg = "[Guide] CROSS_RIGHT";
        if (cmd == dg::GuidanceManager::Motion::ENTER_RIGHT) dir_msg = "[Guide] ENTER_RIGHT";
        if (cmd == dg::GuidanceManager::Motion::EXIT_RIGHT) dir_msg = "[Guide] EXIT_RIGHT";
    }
    else if (cmd == dg::GuidanceManager::Motion::TURN_BACK)
    {
        cv::Mat& icon = m_icon_turn_back;
        cv::Mat& mask = m_mask_turn_back;
        int x1 = center_pos.x - icon.cols / 2;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        dir_msg = "[Guide] TURN_BACK";
    }
    else
    {
        dir_msg = "[Guide] N/A";
    }    

    // show direction message
    cv::Point msg_offset = rect.tl() + cv::Point(10, 30);
    cv::putText(image, dir_msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
    cv::putText(image, dir_msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

    // show distance message
    msg_offset = center_pos + cv::Point(50, 10);
    std::string distance = cv::format("D=%.2lfm", guide.distance_to_remain);
    cv::putText(image, distance.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
    cv::putText(image, distance.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

    // show guidance message
    msg_offset = rect.tl() + cv::Point(0, rect.height + 25);
    cv::putText(image, guide.msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 4);
    cv::putText(image, guide.msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
}


void DeepGuider::drawLogo(cv::Mat image, std::vector<LogoResult> logos, cv::Size original_image_size)
{
    double xscale = (double)image.cols / original_image_size.width;
    double yscale = (double)image.rows / original_image_size.height;

    for(size_t i=0; i<logos.size(); i++)
    {
        logos[i].xmin = (int)(xscale * logos[i].xmin + 0.5);
        logos[i].ymin = (int)(yscale * logos[i].ymin + 0.5);
        logos[i].xmax = (int)(xscale * logos[i].xmax + 0.5);
        logos[i].ymax = (int)(yscale * logos[i].ymax + 0.5);

        cv::Rect rc(logos[i].xmin, logos[i].ymin, logos[i].xmax-logos[i].xmin+1, logos[i].ymax-logos[i].ymin+1);
        cv::rectangle(image, rc, cv::Scalar(0, 255, 0), 2);
        cv::Point pt(logos[i].xmin + 5, logos[i].ymin + 35);
        std::string msg = cv::format("%s %.2lf", logos[i].label.c_str(), logos[i].confidence);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 4);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    }
}


void DeepGuider::drawOcr(cv::Mat image, std::vector<OCRResult> ocrs, cv::Size original_image_size)
{
    double xscale = (double)image.cols / original_image_size.width;
    double yscale = (double)image.rows / original_image_size.height;

    for (size_t i = 0; i < ocrs.size(); i++)
    {
        ocrs[i].xmin = (int)(xscale * ocrs[i].xmin + 0.5);
        ocrs[i].ymin = (int)(yscale * ocrs[i].ymin + 0.5);
        ocrs[i].xmax = (int)(xscale * ocrs[i].xmax + 0.5);
        ocrs[i].ymax = (int)(yscale * ocrs[i].ymax + 0.5);

        cv::Rect rc(ocrs[i].xmin, ocrs[i].ymin, ocrs[i].xmax - ocrs[i].xmin + 1, ocrs[i].ymax - ocrs[i].ymin + 1);
        cv::rectangle(image, rc, cv::Scalar(0, 255, 0), 2);
        cv::Point pt(ocrs[i].xmin + 5, ocrs[i].ymin + 35);
        std::string msg = cv::format("%s %.2lf", ocrs[i].label.c_str(), ocrs[i].confidence);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 4);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    }
}


void DeepGuider::drawIntersection(cv::Mat image, IntersectionResult r, cv::Size original_image_size)
{
    cv::Point pt(image.cols/2-120, 50);
    std::string msg = cv::format("Intersect: %d (%.2lf)", r.cls, r.confidence);
    cv::putText(image, msg, pt, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 5);
    cv::putText(image, msg, pt, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
}


void DeepGuider::procGuidance(dg::Timestamp ts)
{
    if(!m_path_initialized || !m_dest_defined) return;

    // get updated pose & localization confidence
    m_localizer_mutex.lock();
    dg::TopometricPose pose_topo = getPoseTopometric();
    dg::Pose2 pose_metric = getPose();
    dg::LatLon pose_gps = toLatLon(pose_metric);
    double pose_confidence = m_localizer.getPoseConfidence();
    m_localizer_mutex.unlock();

    // Guidance: generate navigation guidance
    dg::GuidanceManager::GuideStatus cur_status;
    dg::GuidanceManager::Guidance cur_guide;
    m_map_mutex.lock();
    dg::Node* node = m_map->getNode(pose_topo.node_id);
    m_map_mutex.unlock();
    if(node==nullptr)
    {
        printf("[Guidance] Error - Undefined localization node: %zu!\n", pose_topo.node_id);
        return;
    }
    m_guider_mutex.lock();
    m_guider.update(pose_topo, pose_confidence);
    cur_status = m_guider.getGuidanceStatus();
    cur_guide = m_guider.getGuidance();
    if (node != nullptr)
    {
        m_guider.applyPoseGPS(toLatLon(*node));
    }
    m_guider_mutex.unlock();

    // print guidance message
    printf("%s\n", cur_guide.msg.c_str());
    
    // tts guidance message
    if (m_enable_tts && cur_guide.announce && !cur_guide.actions.empty())
    {
        dg::GuidanceManager::Motion cmd = cur_guide.actions[0].cmd;
        if (cmd != m_guidance_cmd)
        {
            std::string tts_msg;
            if (cmd == dg::GuidanceManager::Motion::GO_FORWARD) tts_msg = "Go forward";
            else if (cmd == dg::GuidanceManager::Motion::CROSS_FORWARD)  tts_msg = "Cross forward";
            else if (cmd == dg::GuidanceManager::Motion::ENTER_FORWARD) tts_msg = "Enter forward";
            else if (cmd == dg::GuidanceManager::Motion::EXIT_FORWARD) tts_msg = "Exit forward";
            else if (cmd == dg::GuidanceManager::Motion::TURN_LEFT) tts_msg = "Turn left";
            else if (cmd == dg::GuidanceManager::Motion::CROSS_LEFT) tts_msg = "Cross left";
            else if (cmd == dg::GuidanceManager::Motion::ENTER_LEFT) tts_msg = "Enter left";
            else if (cmd == dg::GuidanceManager::Motion::EXIT_LEFT) tts_msg = "Exit left";
            else if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT) tts_msg = "Turn right";
            else if (cmd == dg::GuidanceManager::Motion::CROSS_RIGHT) tts_msg = "Cross right";
            else if (cmd == dg::GuidanceManager::Motion::ENTER_RIGHT) tts_msg = "Enter right";
            else if (cmd == dg::GuidanceManager::Motion::EXIT_RIGHT) tts_msg = "Exit right";
            else if (cmd == dg::GuidanceManager::Motion::TURN_BACK) tts_msg = "Turn back";

            if(!tts_msg.empty()) putTTS(tts_msg.c_str());
            m_guidance_cmd = cmd;
        }
    }

    if (cur_status != m_guidance_status)
    {
        // check arrival
        if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_ARRIVED && cur_guide.announce)
        {
            printf("Arrived to destination!\n");
            if (m_enable_tts) putTTS("Arrived to destination!");
        }

        // check out of path
        if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_OOP || cur_status == dg::GuidanceManager::GuideStatus::GUIDE_LOST)
        {
            printf("GUIDANCE: out of path detected!\n");
            if (m_enable_tts) putTTS("Regenerate path!");
            VVS_CHECK_TRUE(updateDeepGuiderPath(pose_gps, m_gps_dest));
        }

        // check lost
        if (m_enable_exploration)
        {
            m_guider.makeLostValue(m_guider.m_prevconf, pose_confidence);
            m_active_nav.apply(m_cam_image, cur_guide, ts);
            if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_LOST)
            {
                std::vector<ExplorationGuidance> actions;
                dg::GuidanceManager::GuideStatus status;
                m_active_nav.get(actions, status);
                for (int k = 0; k < actions.size(); k++)
                {
                    printf("\t action %d: [%lf, %lf, %lf]\n", k, actions[k].theta1, actions[k].d, actions[k].theta2);
                }
            }
        }
    }
    m_guidance_status = cur_status;
}


bool DeepGuider::procIntersectionClassifier()
{
    dg::Timestamp ts_old = m_intersection.timestamp();
    m_cam_mutex.lock(); 
    dg::Timestamp capture_time = m_cam_capture_time;
    if(capture_time <= ts_old)
    {
        m_cam_mutex.unlock();
        return true;
    }
    cv::Mat cam_image = m_cam_image.clone();
    dg::LatLon capture_pos = m_cam_gps;
    int cam_fnumber = m_cam_fnumber;
    m_cam_mutex.unlock();

    dg::Point2 xy;
    double confidence;
    bool valid_xy = false;
    if (!cam_image.empty() && m_intersection.apply(cam_image, capture_time, xy, confidence, valid_xy))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_intersection.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        } 
        m_intersection.print();

        m_intersection_mutex.lock();
        m_intersection.get(m_intersection_result);
        m_intersection_image = cam_image;
        m_intersection_mutex.unlock();

        // apply the result to localizer
        if(valid_xy) m_localizer.applyIntersectCls(xy, capture_time, confidence);
    }
    else
    {
        m_intersection_mutex.lock();
        m_intersection_image = cv::Mat();
        m_intersection_mutex.unlock();
    }

    return true;
}


bool DeepGuider::procLogo()
{
    dg::Timestamp ts_old = m_logo.timestamp();
    m_cam_mutex.lock(); 
    dg::Timestamp capture_time = m_cam_capture_time;
    if(capture_time <= ts_old)
    {
        m_cam_mutex.unlock();
        return true;
    }
    cv::Mat cam_image = m_cam_image.clone();
    dg::LatLon capture_pos = m_cam_gps;
    int cam_fnumber = m_cam_fnumber;
    m_cam_mutex.unlock();

    if (!cam_image.empty() && m_logo.apply(cam_image, capture_time))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_logo.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        }
        m_logo.print();

        std::vector<dg::ID> ids;
        std::vector<Polar2> obs;
        std::vector<double> confs;
        std::vector<LogoResult> logos;
        m_logo.get(logos);
        for (int k = 0; k < (int)logos.size(); k++)
        {
            dg::ID logo_id = 0;
            ids.push_back(logo_id);
            obs.push_back(rel_pose_defualt);
            confs.push_back(logos[k].confidence);
        }
        //VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, capture_time, confs));

        if(!logos.empty())
        {
            m_logo_mutex.lock();
            m_logo_image = cam_image;
            m_logos = logos;
            m_logo_mutex.unlock();
        }
        else
        {
            m_logo_mutex.lock();
            m_logo_image = cv::Mat();
            m_logos.clear();
            m_logo_mutex.unlock();
        }          
    }

    return true;
}

bool DeepGuider::procOcr()
{
    dg::Timestamp ts_old = m_ocr.timestamp();
    m_cam_mutex.lock(); 
    dg::Timestamp capture_time = m_cam_capture_time;
    if(capture_time <= ts_old)
    {
        m_cam_mutex.unlock();
        return true;
    }
    cv::Mat cam_image = m_cam_image.clone();
    dg::LatLon capture_pos = m_cam_gps;
    int cam_fnumber = m_cam_fnumber;
    m_cam_mutex.unlock();

    std::vector<dg::Point2> poi_xys;
    std::vector<dg::Polar2> relatives;
    std::vector<double> poi_confidences;
    if (!cam_image.empty() && m_ocr.apply(cam_image, capture_time, poi_xys, relatives, poi_confidences))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_ocr.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        }
        m_ocr.print();

        for (int k = 0; k < (int)poi_xys.size(); k++)
        {
            m_localizer.applyPOI(poi_xys[k], relatives[k], capture_time, poi_confidences[k]);
        }

        std::vector<OCRResult> ocrs;
        m_ocr.get(ocrs);
        if (!ocrs.empty())
        {
            m_ocr_mutex.lock();
            m_ocr_image = cam_image;
            m_ocrs = ocrs;
            m_ocr_mutex.unlock();
        }
        else
        {
            m_ocr_mutex.lock();
            m_ocr_image = cv::Mat();
            m_ocrs.clear();
            m_ocr_mutex.unlock();
        }
    }

    return true;
}

bool DeepGuider::procRoadTheta()
{
    dg::Timestamp ts_old = m_roadtheta.timestamp();
    m_cam_mutex.lock(); 
    dg::Timestamp capture_time = m_cam_capture_time;
    if(capture_time <= ts_old)
    {
        m_cam_mutex.unlock();
        return true;
    }
    cv::Mat cam_image = m_cam_image.clone();
    dg::LatLon capture_pos = m_cam_gps;
    int cam_fnumber = m_cam_fnumber;
    m_cam_mutex.unlock();
    
    double theta, confidence;
    if (!cam_image.empty() && m_roadtheta.apply(cam_image, capture_time, theta, confidence))
    {
        m_localizer.applyRoadTheta(theta, capture_time, confidence);
    }

    return true;
}


#ifdef VPSSERVER
std::size_t DeepGuider::curl_callback(const char* in, std::size_t size, std::size_t num, std::string* out)
{
    const std::size_t totalBytes(size * num);
    out->append(in, totalBytes);
    return totalBytes;
}

bool DeepGuider::curl_request(const std::string url, const char* CMD, const Json::Value* post_json, Json::Value* get_json)
{
    CURLcode res;
    CURL* hnd;
    struct curl_slist* slist1;
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
    if (strcmp("POST", CMD) == 0)
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
    if (strcmp("POST", CMD) == 0 || strcmp("GET", CMD) == 0)
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

bool DeepGuider::procVps() // This sends query image and parameters to server using curl_request() and it receives its results (Id,conf.)
{
    dg::Timestamp ts_old = m_vps.timestamp();
    m_cam_mutex.lock(); 
    dg::Timestamp capture_time = m_cam_capture_time;
    if(capture_time <= ts_old)
    {
        m_cam_mutex.unlock();
        return true;
    }
    cv::Mat cam_image = m_cam_image.clone();
    dg::LatLon capture_pos = m_cam_gps;
    int cam_fnumber = m_cam_fnumber;
    m_cam_mutex.unlock();

    int N = 3;  // top-3
    double gps_accuracy = 1;   // 0: search radius = 230m ~ 1: search radius = 30m

	const std::string vps_server_addr = "http://localhost:7729";
	const std::string streetview_server_addr = m_server_ip.c_str(); // "localhost";
	std::string vps_url = vps_server_addr + "/Apply/";
	Json::Value post_json;
	Json::Value ret_json;
	std::string post_str;
	const char * post_data;
	list<int> image_size;
	std::vector<uchar> image_data; 

    if (!cam_image.empty())
	//	&& m_vps.apply(cam_image, N, capture_pos.lat, capture_pos.lon, gps_accuracy, capture_time, m_server_ip.c_str()))
    {
        std::vector<dg::ID> ids;
        std::vector<dg::Polar2> obs;
        std::vector<double> confs;
        std::vector<VPSResult> streetviews;
	    VPSResult IDandConf;

		post_json["K"] = N;
		post_json["gps_lat"] = capture_pos.lat;
		post_json["gps_lon"] = capture_pos.lon;
		post_json["gps_accuracy"] = gps_accuracy;
		post_json["timestamp"] = capture_time;
		post_json["streetview_server_ipaddr"] = streetview_server_addr;
		
		post_json["image_size"].clear();
		post_json["image_size"].append(cam_image.rows); // h
		post_json["image_size"].append(cam_image.cols); // w
		post_json["image_size"].append(cam_image.channels()); // c
	
		post_json["image_data"].clear();
		cam_image.reshape(cam_image.cols * cam_image.rows * cam_image.channels()); // Serialize
		uchar * image_data = cam_image.data;
		for(int idx = 0; idx < cam_image.rows*cam_image.cols*cam_image.channels(); idx++)
		{
			post_json["image_data"].append(image_data[idx]);
		}
	
	 	curl_request(vps_url, "POST", &post_json, &ret_json);
		curl_request(vps_url, "GET" , 0, &ret_json); // [Optional]
		//cout << ret_json["vps_IDandConf"].size() << endl;

		for(int idx = 0; idx < ret_json["vps_IDandConf"][0].size(); idx++) // 5 : K
		{
			IDandConf.id = ret_json["vps_IDandConf"][0][idx].asDouble(); 
			IDandConf.confidence = ret_json["vps_IDandConf"][1][idx].asDouble(); 
			streetviews.push_back(IDandConf);
		}
	
        //m_vps.get(streetviews);

        printf("[VPS]\n");
        for (int k = 0; k < (int)streetviews.size(); k++)
        {
            if (streetviews[k].id <= 0 || streetviews[k].confidence <= 0) continue;        // invalid data
            ids.push_back(streetviews[k].id);
            obs.push_back(rel_pose_defualt);
            confs.push_back(streetviews[k].confidence);
            printf("\ttop%d: id=%zu, confidence=%lf, ts=%lf\n", k, streetviews[k].id, streetviews[k].confidence, capture_time);
        }

        if(ids.size() > 0)
        {
            m_localizer_mutex.lock();
            VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, capture_time, confs));
            m_localizer_mutex.unlock();

            cv::Mat sv_image;
            if(m_map_manager.getStreetViewImage(ids[0], sv_image, "f") && !sv_image.empty())
            {
                m_vps_mutex.lock();
                m_vps_image = sv_image;
                m_vps_id = ids[0];
                m_vps_confidence = confs[0];
                m_vps_mutex.unlock();
            }
            else
            {
                m_vps_mutex.lock();
                m_vps_image = cv::Mat();
                m_vps_id = ids[0];
                m_vps_confidence = confs[0];
                m_vps_mutex.unlock();
            }
        }
        else
        {
            m_vps_mutex.lock();
            m_vps_image = cv::Mat();
            m_vps_id = 0;
            m_vps_confidence = 0;
            m_vps_mutex.unlock();
        }
    }

    return true;
}

#else // VPSSERVER
bool DeepGuider::procVps() // This will call apply() in vps.py embedded by C++ 
{
    dg::Timestamp ts_old = m_vps.timestamp();
    m_cam_mutex.lock(); 
    dg::Timestamp capture_time = m_cam_capture_time;
    if(capture_time <= ts_old)
    {
        m_cam_mutex.unlock();
        return true;
    }
    cv::Mat cam_image = m_cam_image.clone();
    dg::LatLon capture_pos = m_cam_gps;
    int cam_fnumber = m_cam_fnumber;
    m_cam_mutex.unlock();

    dg::Point2 sv_xy;
    dg::Polar2 relative;
    ID sv_id;
    cv::Mat sv_image;
    double sv_confidence;
    if (!cam_image.empty() && m_vps.apply(cam_image, capture_time, sv_xy, relative, sv_confidence, sv_id, sv_image))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_vps.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        } 
        m_vps.print();


        m_localizer_mutex.lock();
        VVS_CHECK_TRUE(m_localizer.applyVPS(sv_xy, relative, capture_time, sv_confidence));
        m_localizer_mutex.unlock();

        if(!sv_image.empty())
        {
            m_vps_mutex.lock();
            m_vps_image = sv_image;
            m_vps_id = sv_id;
            m_vps_confidence = sv_confidence;
            m_vps_mutex.unlock();
        }
        else
        {
            m_vps_mutex.lock();
            m_vps_image = cv::Mat();
            m_vps_id = sv_id;
            m_vps_confidence = sv_confidence;
            m_vps_mutex.unlock();
        }
    }
    else
    {
        m_vps_mutex.lock();
        m_vps_image = cv::Mat();
        m_vps_id = 0;
        m_vps_confidence = 0;
        m_vps_mutex.unlock();
    }

    return true;
}
#endif // VPSSERVER


// Thread fnuction for VPS
void DeepGuider::threadfunc_vps(DeepGuider* guider)
{
    guider->is_vps_running = true;
    printf("\tvps thread starts\n");
    while (guider->m_enable_vps)
    {
        guider->procVps();
    }
    guider->is_vps_running = false;
    printf("\tvps thread ends\n");
}

// Thread fnuction for POI OCR
void DeepGuider::threadfunc_ocr(DeepGuider* guider)
{
    guider->is_ocr_running = true;
    printf("\tocr thread starts\n");
    while (guider->m_enable_ocr)
    {
        guider->procOcr();
    }
    guider->is_ocr_running = false;
    printf("\tocr thread ends\n");
}

// Thread fnuction for POI Logo
void DeepGuider::threadfunc_logo(DeepGuider* guider)
{
    guider->is_logo_running = true;
    printf("\tlogo thread starts\n");
    while (guider->m_enable_logo)
    {
        guider->procLogo();
    }
    guider->is_logo_running = false;
    printf("\tlogo thread ends\n");
}

// Thread fnuction for IntersectionClassifier
void DeepGuider::threadfunc_intersection(DeepGuider* guider)
{
    guider->is_intersection_running = true;
    printf("\tintersection thread starts\n");
    while (guider->m_enable_intersection)
    {
        guider->procIntersectionClassifier();
    }
    guider->is_intersection_running = false;
    printf("\tintersection thread ends\n");
}

// Thread fnuction for RoadTheta
void DeepGuider::threadfunc_roadtheta(DeepGuider* guider)
{
    guider->is_roadtheta_running = true;
    printf("\troadtheta thread starts\n");
    while (guider->m_enable_roadtheta)
    {
        guider->procRoadTheta();
    }
    guider->is_roadtheta_running = false;
    printf("\troadtheta thread ends\n");
}

// Thread fnuction for tts
void DeepGuider::threadfunc_tts(DeepGuider* guider)
{
    guider->is_tts_running = true;
    printf("\ttts thread starts\n");
    while (guider->m_enable_tts)
    {
        guider->procTTS();
    }
    guider->is_tts_running = false;
    printf("\ttts thread ends\n");
}

void DeepGuider::putTTS(const char* msg)
{
    m_tts_mutex.lock();
    m_tts_msg.push_back(msg);
    m_tts_mutex.unlock();
}

void DeepGuider::procTTS()
{
    m_tts_mutex.lock();
    std::vector<std::string> tts_msg = m_tts_msg;
    m_tts_msg.clear();
    m_tts_mutex.unlock();

    for(int i=0; i<(int)tts_msg.size(); i++)
    {
        tts(tts_msg[i]);
    }
}

void DeepGuider::terminateThreadFunctions()
{
    if (vps_thread == nullptr && ocr_thread == nullptr && logo_thread == nullptr && intersection_thread == nullptr && roadtheta_thread == nullptr && tts_thread == nullptr) return;

    // disable all thread running
    m_enable_vps = false;
    m_enable_ocr = false;
    m_enable_logo = false;
    m_enable_intersection = false;
    m_enable_roadtheta = false;
    m_enable_tts = false;

    // wait child thread to terminate
    if (vps_thread && is_vps_running) vps_thread->join();
    if (ocr_thread && is_ocr_running) ocr_thread->join();
    if (logo_thread && is_logo_running) logo_thread->join();
    if (intersection_thread && is_intersection_running) intersection_thread->join();
    if (roadtheta_thread && is_roadtheta_running) roadtheta_thread->join();
    if (tts_thread && is_tts_running) tts_thread->join();

    // clear threads
    vps_thread = nullptr;
    ocr_thread = nullptr;
    logo_thread = nullptr;
    intersection_thread = nullptr;
    roadtheta_thread = nullptr;
    tts_thread = nullptr;
}

#endif      // #ifndef __DEEPGUIDER_SIMPLE__


#ifdef DG_TEST_SIMPLE
int main()
{
    DeepGuider deepguider;
    if (!deepguider.initialize("dg_simple.yml")) return -1;
    deepguider.run();

    return 0;
}
#endif      // #ifdef DG_TEST_SIMPLE
