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
#include "dg_roadlr.hpp"
#include "dg_guidance.hpp"
#include "dg_exploration.hpp"
#include "dg_utils.hpp"
#include "localizer/data_loader.hpp"
#include "utils/viewport.hpp"
#include <chrono>

using namespace dg;
using namespace std;

class DeepGuider : public SharedInterface, public cx::Algorithm
{
protected:
    // configuable parameters
    int m_enable_360cam = 0;
    int m_enable_360cam_crop = 0;
    int m_enable_intersection = 0;
    int m_enable_ocr = 0;
    int m_enable_vps = 0;
    int m_enable_roadlr = 0;
    int m_enable_roadtheta = 0;
    int m_enable_exploration = 0;
    int m_enable_logo = 0;
    int m_enable_imu = 0;
    int m_enable_odometry = 0;
    int m_enable_mapserver = 1;

    bool m_apply_intersection = true;
    bool m_apply_imu = true;
    bool m_apply_ocr = true;
    bool m_apply_vps = true;
    bool m_apply_roadlr = true;
    bool m_apply_roadtheta = true;
    bool m_apply_gps = true;
    bool m_apply_odometry = true;
    bool m_show_ekf_pose = false;

    std::string m_server_ip = "127.0.0.1";  // default: 127.0.0.1 (localhost)
    std::string m_image_server_port = "10000";  // etri: 10000, coex: 10001, bucheon: 10002, etri_indoor: 10003
    std::string m_srcdir = "./../src";      // path of deepguider/src (required for python embedding)
    bool m_enable_tts = false;
    bool m_threaded_run_modules = true;
    bool m_use_high_precision_gps = false;  // use high-precision gps (novatel)

    bool m_video_recording = false;
    int m_video_recording_fps = 15;
    std::string m_recording_header_name = "dg_simple_";

    std::string m_map_image_path = "data/NaverMap_ETRI(Satellite)_191127.png";
    std::string m_map_data_path = "data/ETRI/TopoMap_ETRI.csv";
    dg::LatLon m_map_ref_point = dg::LatLon(36.383837659737, 127.367880828442);
    dg::Point2 m_map_ref_point_pixel = dg::Point2(347, 297);
    double m_map_pixel_per_meter = 1.039;
    double m_map_image_rotation = cx::cvtDeg2Rad(1.0);
    std::string m_gps_input_path = "data/191115_ETRI_asen_fix.csv";
    std::string m_video_input_path = "data/ETRI/191115_151140_images.avi";
    cv::Vec3b m_gui_vps_color = cv::Vec3b(229, 204, 255); // bright pink
    cv::Vec3b m_gui_vps_rpose_color = cv::Vec3b(127, 0, 255);  // pink
    bool m_gui_auto_scroll = true;

    // gps drawing
    cv::Vec3b m_gui_gps_color = cv::Vec3b(0, 255, 0);
    cv::Vec3b m_gui_gps_novatel_color = cv::Vec3b(255, 0, 0);
    int m_gui_gps_trj_radius = 3;

    // robot trajectory drawing
    cv::Vec3b m_gui_robot_color = cv::Vec3b(0, 0, 255);  // B,G,R
    int m_gui_robot_trj_radius = 2;
    cv::Vec3b m_gui_path_color = cv::Vec3b(255, 0, 0);

    // VPS parameters
	// 0.0 means "Not using", 1.0 means "Using"
    double m_vps_max_error_distance = 25;
	int m_vps_load_dbfeat = 0;
	int m_vps_save_dbfeat = 0;
	double m_vps_gps_accuracy = 0.9;  // Constant gps accuracy related to search range. In streetview image server, download_radius = int(10 + 190*(1-vps_gps_accuracy)) , 1:10m, 0.95:20m, 0.9:29m, 0.79:50, 0.0:200 meters
	int m_vps_use_custom_image_server = 0;  // use custom image dataset instead of naver roadview to avoid wide baseline problem

    int m_exploration_state_count = 0;
    const int m_exploration_state_count_max = 20;

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
	cv::Mat crop_image(cv::Mat cam_image, int num, int idx); // num : number of h-stacked images, idx, 0: left, 1: front, 2:right
	bool get_cam_image(cv::Mat& cam_image, cv::Mat& cam_image_for_draw, int idx_for_draw, double& txt_scale, dg::Timestamp& capture_time, dg::Timestamp capture_time_prev);

    void procMouseEvent(int evt, int x, int y, int flags);
    void procTTS();

protected:
    virtual int readParam(const cv::FileNode& fn);

    // internal api's
    bool initializeDefaultMap();
    bool setDeepGuiderDestination(dg::Point2F dest);
    bool updateDeepGuiderPath(dg::Point2F start, dg::Point2F dest);
    void drawGuiDisplay(cv::Mat& gui_image, const cv::Point2d& view_offset, double view_zoom);
    void drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect);
    void procGpsData(dg::LatLon gps_datum, dg::Timestamp ts);
    void procImuData(double ori_w, double ori_x, double ori_y, double ori_z, dg::Timestamp ts);
    void procOdometryData(double x, double y, double theta, dg::Timestamp ts);
    void procGuidance(dg::Timestamp ts);
    bool procIntersectionClassifier();
    bool procExploration();
    bool procLogo();
    bool procOcr();
    bool procVps();
    bool procRoadLR();
    bool procRoadTheta();

    // sub modules
    dg::GuidanceManager m_guider;
    dg::MapManager m_map_manager;
    dg::DGLocalizer m_localizer;
    dg::IntersectionLocalizer m_intersection;
    dg::OCRLocalizer m_ocr;
    dg::VPSLocalizer m_vps;
    dg::RoadLRLocalizer m_roadlr;
    dg::RoadThetaLocalizer m_roadtheta;
    dg::ActiveNavigation m_active_nav;
    dg::LogoLocalizer m_logo;

    // global variables
    dg::Point2F m_dest;
    bool m_dest_defined = false;
    bool m_path_generation_pended = false;

    // local variables
    std::string m_winname = "DeepGuider";           // title of gui window
    cx::VideoWriter m_video_gui;
    cx::VideoWriter m_video_cam;
    std::ofstream m_log;
    cv::Mutex m_log_mutex;
    cv::Mat m_map_image;
    dg::MapPainter m_painter;
    dg::GuidanceManager::Motion m_guidance_cmd = dg::GuidanceManager::Motion::STOP;
    dg::GuidanceManager::GuideStatus m_guidance_status = dg::GuidanceManager::GuideStatus::GUIDE_INITIAL;

    // GUI display
    dg::Viewport m_viewport;
    cv::Point m_view_offset = cv::Point(0, 0);
    cv::Size m_view_size = cv::Size(1800, 1012);
    int m_view_bottom_padding = 0;
    int m_video_win_height = 288;   // pixels
    int m_video_win_margin = 10;    // pixels
    int m_video_win_gap = 10;       // pixels

    // TTS
    cv::Mutex m_tts_mutex;
    std::vector<std::string> m_tts_msg;
    std::thread* tts_thread = nullptr;
    static void threadfunc_tts(DeepGuider* guider);
    bool is_tts_running = false;
    void putTTS(const char* msg);

    // Thread routines
    std::thread* intersection_thread = nullptr;
    std::thread* ocr_thread = nullptr;
    std::thread* vps_thread = nullptr;
    std::thread* roadlr_thread = nullptr;
    std::thread* roadtheta_thread = nullptr;
    std::thread* exploration_thread = nullptr;
    std::thread* logo_thread = nullptr;
    static void threadfunc_intersection(DeepGuider* guider);
    static void threadfunc_ocr(DeepGuider* guider);
    static void threadfunc_vps(DeepGuider* guider);
    static void threadfunc_roadlr(DeepGuider* guider);
    static void threadfunc_roadtheta(DeepGuider* guider);
    static void threadfunc_exploration(DeepGuider* guider);
    static void threadfunc_logo(DeepGuider* guider);
    bool is_intersection_running = false;
    bool is_ocr_running = false;
    bool is_vps_running = false;
    bool is_roadlr_running = false;
    bool is_roadtheta_running = false;
    bool is_exploration_running = false;
    bool is_logo_running = false;
    void terminateThreadFunctions();

    // shared variables for multi-threading
    cv::Mutex m_cam_mutex;
    cv::Mat m_cam_image;
    dg::Timestamp m_cam_capture_time;

    cv::Mutex m_360cam_mutex;
    cv::Mutex m_360cam_crop_mutex;
    cv::Mat m_360cam_image;
    cv::Mat m_360cam_crop_image;
    dg::Timestamp m_360cam_capture_time;

    cv::Mutex m_vps_mutex;
    cv::Mat m_vps_image;            // top-1 matched streetview image
	dg::ID m_vps_id;
    dg::Point2 m_vps_xy;            // top-1 matched streetview's position (x,y)
    dg::Polar2 m_vps_relative;      // top-1 matched streetview's relative position (pan, tz) or (theta_z, delta_z)

    cv::Mutex m_roadlr_mutex;
    cv::Mat m_roadlr_image;

    cv::Mutex m_logo_mutex;
    cv::Mat m_logo_image;

    cv::Mutex m_ocr_mutex;
    cv::Mat m_ocr_image;

    cv::Mutex m_intersection_mutex;
    cv::Mat m_intersection_image;

    cv::Mutex m_roadtheta_mutex;
    cv::Mat m_roadtheta_image;

    cv::Mutex m_exploration_mutex;
    cv::Mat m_exploration_image;

    cv::Mutex m_map_mutex;
    cv::Mutex m_guider_mutex;

    cv::Mutex m_robotmap_mutex;
    cv::Mat m_robotmap_image;
    dg::Timestamp m_robotmap_capture_time;

    // guidance icons
    cv::Mat m_icon_forward;
    cv::Mat m_mask_forward;
    cv::Mat m_icon_turn_left;
    cv::Mat m_mask_turn_left;
    cv::Mat m_icon_turn_right;
    cv::Mat m_mask_turn_right;
    cv::Mat m_icon_turn_back;
    cv::Mat m_mask_turn_back;
};

void onMouseEvent(int event, int x, int y, int flags, void* param)
{
    DeepGuider* dg = (DeepGuider *)param;
    dg->procMouseEvent(event, x, y, flags);
}

DeepGuider::~DeepGuider()
{
    m_intersection.clear();
    m_ocr.clear();
    m_vps.clear();
    m_roadlr.clear();
    m_roadtheta.clear();
    m_active_nav.clear();
    m_logo.clear();

    close_python_environment();
}

int DeepGuider::readParam(const cv::FileNode& fn)
{
    int n_read = cx::Algorithm::readParam(fn);

    // Read Activate/deactivate Options
    CX_LOAD_PARAM_COUNT(fn, "enable_360cam", m_enable_360cam, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_360cam_crop", m_enable_360cam_crop, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_intersection", m_enable_intersection, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_ocr", m_enable_ocr, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_vps", m_enable_vps, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_roadlr", m_enable_roadlr, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_roadtheta", m_enable_roadtheta, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_exploration", m_enable_exploration, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_logo", m_enable_logo, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_imu", m_enable_imu, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_odometry", m_enable_odometry, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_mapserver", m_enable_mapserver, n_read);

    // Read Main Options
    int server_ip_index = -1;
    std::vector<cv::String> server_ip_list;
    CX_LOAD_PARAM_COUNT(fn, "server_ip_list", server_ip_list, n_read);
    CX_LOAD_PARAM_COUNT(fn, "server_ip_index", server_ip_index, n_read);
    if (server_ip_index >= 0 && server_ip_index < server_ip_list.size()) m_server_ip = server_ip_list[server_ip_index];
    CX_LOAD_PARAM_COUNT(fn, "image_server_port", m_image_server_port, n_read);

    int site_index = -1;
    std::string site_tagname;
    std::vector<cv::String> site_names;
    CX_LOAD_PARAM_COUNT(fn, "site_names", site_names, n_read);
    CX_LOAD_PARAM_COUNT(fn, "site_index", site_index, n_read);
    if (site_index >= 0 && site_index < site_names.size()) site_tagname = site_names[site_index];
    m_guider.setSiteName(site_tagname);
	printf("site_tagname: %s\n", site_tagname.c_str());

    CX_LOAD_PARAM_COUNT(fn, "dg_srcdir", m_srcdir, n_read);
    CX_LOAD_PARAM_COUNT(fn, "enable_tts", m_enable_tts, n_read);
    CX_LOAD_PARAM_COUNT(fn, "threaded_run_modules", m_threaded_run_modules, n_read);
    CX_LOAD_PARAM_COUNT(fn, "use_high_precision_gps", m_use_high_precision_gps, n_read);

    // Read Other Options
    CX_LOAD_PARAM_COUNT(fn, "video_recording", m_video_recording, n_read);
    CX_LOAD_PARAM_COUNT(fn, "video_recording_fps", m_video_recording_fps, n_read);
    CX_LOAD_PARAM_COUNT(fn, "recording_header_name", m_recording_header_name, n_read);

    // Read Site Setting
    CX_LOAD_PARAM_COUNT(fn, "map_image_path", m_map_image_path, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_data_path", m_map_data_path, n_read);
    cv::Vec2d ref_point = cv::Vec2d(m_map_ref_point.lat, m_map_ref_point.lon);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_latlon", ref_point, n_read);
    m_map_ref_point = dg::LatLon(ref_point[0], ref_point[1]);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_pixel", m_map_ref_point_pixel, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_pixel_per_meter", m_map_pixel_per_meter, n_read);
    double map_image_rotation = cx::cvtRad2Deg(m_map_image_rotation);
    CX_LOAD_PARAM_COUNT(fn, "map_image_rotation", map_image_rotation, n_read);
    m_map_image_rotation = cx::cvtDeg2Rad(map_image_rotation);
    CX_LOAD_PARAM_COUNT(fn, "map_view_offset", m_view_offset, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_view_size", m_view_size, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_view_bottom_padding", m_view_bottom_padding, n_read);
    CX_LOAD_PARAM_COUNT(fn, "gps_input_path", m_gps_input_path, n_read);
    CX_LOAD_PARAM_COUNT(fn, "video_input_path", m_video_input_path, n_read);

	// Read VPS specific Options
    CX_LOAD_PARAM_COUNT(fn, "vps_load_dbfeat", m_vps_load_dbfeat, n_read);
    CX_LOAD_PARAM_COUNT(fn, "vps_save_dbfeat", m_vps_save_dbfeat, n_read);
    CX_LOAD_PARAM_COUNT(fn, "vps_gps_accuracy", m_vps_gps_accuracy, n_read);
    CX_LOAD_PARAM_COUNT(fn, "vps_use_custom_image_server", m_vps_use_custom_image_server, n_read);

    // Read Site-specific Setting
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
    dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

    printf("Initialize deepguider system...\n");

    // load config
    bool ok = loadParam(config_file);
    if(ok) printf("\tConfiguration %s loaded!\n", config_file.c_str());

    // initialize python
    bool enable_python = (m_enable_intersection==1) || (m_enable_ocr==1) || (m_enable_vps==1) || (m_enable_roadlr==1) || (m_enable_exploration==1) || (m_enable_logo==1);
    if (enable_python && !init_python_environment("python3", "", m_threaded_run_modules)) return false;
    if (enable_python) printf("\tPython environment initialized!\n");

    // initialize map manager
    m_map_manager.setReference(m_map_ref_point);
    if (!m_map_manager.initialize(m_server_ip, m_image_server_port)) return false;
    printf("\tMapManager initialized!\n");

    // initialize Intersection
    std::string py_module_path = m_srcdir + "/intersection_cls";
    if (m_enable_intersection==1 && !m_intersection.initialize(this, py_module_path)) return false;
    if (m_enable_intersection==1) printf("\tIntersection initialized in %.3lf seconds!\n", m_intersection.procTime());

    // initialize OCR
    py_module_path = m_srcdir + "/ocr_recog";
    if (m_enable_ocr==1 && !m_ocr.initialize(this, py_module_path)) return false;
    if (m_enable_ocr==1) printf("\tOCR initialized in %.3lf seconds!\n", m_ocr.procTime());

    // initialize VPS
    py_module_path = m_srcdir + "/vps";
    if (m_enable_vps==1 && !m_vps.initialize(this, py_module_path, m_server_ip, m_image_server_port, m_vps_use_custom_image_server)) return false;
    if (m_enable_vps==1) printf("\tVPS initialized in %.3lf seconds!\n", m_vps.procTime());

    // initialize RoadLR
    py_module_path = m_srcdir + "/roadlr";
    if (m_enable_roadlr==1 && !m_roadlr.initialize(this, py_module_path)) return false;
    if (m_enable_roadlr==1) printf("\tRoadLR initialized in %.3lf seconds!\n", m_roadlr.procTime());

    // initialize RoadTheta
    if (m_enable_roadtheta==1 && !m_roadtheta.initialize(this)) return false;
    if (m_enable_roadtheta==1) printf("\tRoadTheta initialized in %.3lf seconds!\n", m_roadtheta.procTime());

    // initialize exploation 
    py_module_path = m_srcdir + "/exploration";
    if (m_enable_exploration==1 && !m_active_nav.initialize(py_module_path)) return false;
    if (m_enable_exploration==1) printf("\tExploation initialized!\n");

    // initialize Logo
    py_module_path = m_srcdir + "/logo_recog";
    if (m_enable_logo==1 && !m_logo.initialize(this, py_module_path)) return false;
    if (m_enable_logo==1) printf("\tLogo initialized in %.3lf seconds!\n", m_logo.procTime());

    // initialize default map
    if (m_enable_mapserver) initializeDefaultMap();
    else
    {
        m_map.setReference(m_map_ref_point);
        m_map.load(m_map_data_path.c_str());
        m_map.updateEdgeLR();
    }

    // initialize localizer
    if (!m_localizer.initialize(this, "EKFLocalizerHyperTan")) return false;
    if (!m_localizer.setParamMotionNoise(1, 10)) return false;      // linear_velocity(m/sec), angular_velocity(deg/sec)
    if (!m_localizer.setParamMotionBounds(1, 10)) return false;     // max_linear_velocity(m/sec), max_angular_velocity(deg/sec)
    if (!m_localizer.setParamGPSNoise(10)) return false;            // position error(m)
    if (!m_localizer.setParamGPSOffset(1, 0)) return false;         // displacement(lin,ang) from robot origin
    if (!m_localizer.setParamOdometryNoise(0.01, 1)) return false;  // position error(m), orientation error(deg)
    if (!m_localizer.setParamIMUCompassNoise(1, 0)) return false;   // angle arror(deg), angle offset(deg)
    if (!m_localizer.setParamPOINoise(5, 20, 25)) return false;    // position error(m), orientation error(deg), max error (m)
    if (!m_localizer.setParamVPSNoise(5, 20, m_vps_max_error_distance)) return false;    // position error(m), orientation error(deg), max error (m)
    if (!m_localizer.setParamIntersectClsNoise(0.1)) return false;  // position error(m)
    if (!m_localizer.setParamRoadThetaNoise(50)) return false;      // angle arror(deg), angle offset(deg)
    if (!m_localizer.setParamCameraOffset(1, 0)) return false;      // displacement(lin,ang) from robot origin
    m_localizer.setParamValue("gps_reverse_vel", -0.5);
    m_localizer.setParamValue("search_turn_weight", 100);
    m_localizer.setParamValue("track_near_radius", 20);
    m_localizer.setParamValue("enable_path_projection", true);
    m_localizer.setParamValue("enable_map_projection", false);
    m_localizer.setParamValue("enable_backtracking_ekf", true); // default : true, for demo : false
    m_localizer.setParamValue("enable_gps_smoothing", false);
    m_localizer.setParamValue("enable_debugging_display", false);
    m_localizer.setParamValue("lr_mismatch_cost", 50);
    m_localizer.setParamValue("enable_lr_reject", false);
    m_localizer.setParamValue("lr_reject_cost", 20);             // 20
    m_localizer.setParamValue("enable_discontinuity_cost", true);
    m_localizer.setParamValue("discontinuity_weight", 0.5);      // 0.5
    printf("\tLocalizer initialized!\n");

    // initialize guidance
    if (!m_guider.initialize(this)) return false;
    printf("m_dxrobot_usage: %d\n", m_guider.m_dxrobot_usage);
    printf("\tGuidance initialized!\n");

    // load background GUI image
    m_map_image = cv::imread(m_map_image_path);
    VVS_CHECK_TRUE(!m_map_image.empty());

    // prepare GUI map
    m_viewport.initialize(m_map_image, m_view_size, m_view_offset);
    m_viewport.setBottomPadding(m_view_bottom_padding);
    m_painter.configCanvas(m_map_ref_point_pixel, cv::Point2d(m_map_pixel_per_meter, m_map_pixel_per_meter), m_map_image.size(), 0, 0);
    m_painter.setImageRotation(m_map_image_rotation);
    m_painter.drawGrid(m_map_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, cv::Point(-215, -6));
    m_painter.drawOrigin(m_map_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    m_painter.setParamValue("node_radius", 3);
    m_painter.setParamValue("node_font_scale", 0);
    m_painter.setParamValue("node_color", { 255, 50, 255 });
    m_painter.setParamValue("junction_color", { 255, 250, 0 });
    m_painter.setParamValue("edge_color", { 0, 255, 255 });
    m_painter.setParamValue("sidewalk_color", { 200, 100, 100 });
    m_painter.setParamValue("crosswalk_color", { 0, 150, 50 });
    m_painter.setParamValue("mixedroad_color", { 200, 100, 100 });
    m_painter.setParamValue("edge_thickness", 2);
    VVS_CHECK_TRUE(m_painter.drawMap(m_map_image, &m_map));

    // load icon images
    double icon_scale = 3.0;
    m_icon_forward = cv::imread("data/forward.png");
    m_icon_turn_left = cv::imread("data/turn_left.png");
    m_icon_turn_right = cv::imread("data/turn_right.png");
    m_icon_turn_back = cv::imread("data/turn_back.png");
    cv::resize(m_icon_forward, m_icon_forward, cv::Size(), icon_scale, icon_scale);
    cv::resize(m_icon_turn_left, m_icon_turn_left, cv::Size(), icon_scale, icon_scale);
    cv::resize(m_icon_turn_right, m_icon_turn_right, cv::Size(), icon_scale, icon_scale);
    cv::resize(m_icon_turn_back, m_icon_turn_back, cv::Size(), icon_scale, icon_scale);
    cv::threshold(m_icon_forward, m_mask_forward, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(m_icon_turn_left, m_mask_turn_left, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(m_icon_turn_right, m_mask_turn_right, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(m_icon_turn_back, m_mask_turn_back, 250, 1, cv::THRESH_BINARY_INV);

    // create GUI window
    cv::namedWindow(m_winname, cv::WINDOW_NORMAL);
    cv::resizeWindow(m_winname, m_viewport.size().width, m_viewport.size().height + m_view_bottom_padding);
    cv::setMouseCallback(m_winname, onMouseEvent, this);

    // init video recording
    std::string sztime = getTimeString();
    if (m_video_recording)
    {
        std::string filename = m_recording_header_name + sztime + "_gui.avi";
        m_video_gui.open(filename, m_video_recording_fps);
    }

    // reset interval variables
    m_dest_defined = false;
    m_path_generation_pended = false;
    m_cam_image.release();
    m_cam_capture_time = -1;
    m_vps_image.release();
    m_roadlr_image.release();
    m_logo_image.release();
    m_ocr_image.release();
    m_intersection_image.release();
    m_exploration_image.release();
    m_roadtheta_image.release();
    m_guidance_cmd = dg::GuidanceManager::Motion::STOP;
    m_guidance_status = dg::GuidanceManager::GuideStatus::GUIDE_INITIAL;

    // tts
    if (m_enable_tts)
    {
        tts_thread = new std::thread(threadfunc_tts, this);
        putTTS("System is initialized!");
    } 

    dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

    printf("\tInitialization is done in %lf seconds!\n\n", t2 - t1);

    return true;
}

bool DeepGuider::initializeDefaultMap()
{
    double radius = 2000;   // meter
    Map map;
    map.reserveMemory();
    VVS_CHECK_TRUE(m_map_manager.getMapAll(m_map_ref_point.lat, m_map_ref_point.lon, radius, map));
    setMap(map);
    printf("\tDefault map is downloaded: nodes=%d, edges=%d, pois=%d, views=%d\n", m_map.countNodes(), m_map.countEdges(), m_map.countPOIs(), m_map.countViews());

    return true;
}

int DeepGuider::run()
{
    // load test dataset    
    dg::DataLoader data_loader;
    std::string ahrs_file, ocr_file, poi_file, vps_file, intersection_file, roadlr_file, roadtheta_file, exploration_file;
    std::string data_header = "data/ETRI/191115_151140";
    //ahrs_file = data_header + "_imu_data.csv";
    //ocr_file = data_header + "_ocr.csv";
    //poi_file = data_header + "_poi.csv";
    //vps_file = data_header + "_vps.csv";
    //intersection_file = data_header + "_intersect.csv";
    //lr_file = data_header + "_roadlr.csv";
    //roadtheta_file = data_header + "_roadtheta.csv";
    if (!data_loader.load(m_video_input_path, m_gps_input_path, ahrs_file, ocr_file, poi_file, vps_file, intersection_file, roadlr_file, roadtheta_file))
    {
        printf("DeepGuider::run() - Fail to load test data. Exit program...\n");
        return -1;
    }
    printf("Run deepguider system...\n");

    // start module threads
    if(m_threaded_run_modules)
    {
        if (m_enable_intersection==1) intersection_thread = new std::thread(threadfunc_intersection, this);
        if (m_enable_ocr==1) ocr_thread = new std::thread(threadfunc_ocr, this);    
        if (m_enable_vps==1) vps_thread = new std::thread(threadfunc_vps, this);
        if (m_enable_roadlr==1) roadlr_thread = new std::thread(threadfunc_roadlr, this);
        if (m_enable_roadtheta==1) roadtheta_thread = new std::thread(threadfunc_roadtheta, this);
        if (m_enable_exploration==1) exploration_thread = new std::thread(threadfunc_exploration, this);    
        if (m_enable_logo==1) logo_thread = new std::thread(threadfunc_logo, this);
    }

    cv::Mat video_image;
    cv::Mat gui_image;
    int wait_msec = 200;
    int itr = 0;
    double start_time = 500;
    data_loader.setStartSkipTime(start_time);
    while (1)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // get next data
        int type;
        std::vector<double> vdata;
        std::vector<std::string> sdata;
        dg::Timestamp data_time;
        if (data_loader.getNext(type, vdata, sdata, data_time) == false) break;

        // process data
        bool update_gui = false;
        if (type == dg::DATA_GPS)
        {
            const dg::LatLon gps_datum(vdata[1], vdata[2]);
            procGpsData(gps_datum, data_time);
			m_painter.drawPoint(m_map_image, toMetric(gps_datum), m_gui_gps_trj_radius, m_gui_gps_color);
            printf("[GPS] lat=%lf, lon=%lf, ts=%lf\n", gps_datum.lat, gps_datum.lon, data_time);

            video_image = data_loader.getFrame(data_time);
            update_gui = true;
        }
        else if (type == dg::DATA_IMU)
        {
            procImuData(vdata[1], vdata[2], vdata[3], vdata[4], data_time);
        }
        else if (type == dg::DATA_POI)
        {
            dg::Point2 clue_xy(vdata[2], vdata[3]);
            dg::Polar2 relative(vdata[4], vdata[5]);
            double confidence = vdata[6];
            bool success = m_localizer.applyPOI(clue_xy, relative, data_time, confidence);
            if (!success) fprintf(stderr, "applyPOI() was failed.\n");
        }
        else if (type == dg::DATA_OCR)
        {
            std::string name = sdata[0];
            double conf = vdata[1];
            double xmin = vdata[2];
            double ymin = vdata[3];
            double xmax = vdata[4];
            double ymax = vdata[5];
            dg::POI* poi;
            dg::Polar2 relative;
            double confidence;
            if (m_ocr.applyPreprocessed(name, xmin, ymin, xmax, ymax, conf, data_time, poi, relative, confidence))
            {
                bool success = m_localizer.applyPOI(*poi, relative, data_time, confidence);
                if (!success) fprintf(stderr, "applyOCR() was failed.\n");
            }
        }
        else if (type == dg::DATA_VPS)
        {
            dg::Point2 clue_xy = toMetric(dg::LatLon(vdata[3], vdata[4]));
            dg::Polar2 relative(vdata[5], vdata[6]);
            double confidence = vdata[7];
            bool success = m_localizer.applyVPS(clue_xy, relative, data_time, confidence);
            if (!success) fprintf(stderr, "applyVPS() was failed.\n");
        }
        else if (type == dg::DATA_IntersectCls)
        {
            double cls = vdata[1];
            double cls_conf = vdata[2];
            dg::Point2 xy;
            double xy_confidence;
            bool xy_valid = false;
            if (m_intersection.applyPreprocessed(cls, cls_conf, data_time, xy, xy_confidence, xy_valid) && xy_valid)
            {
                bool success = m_localizer.applyIntersectCls(xy, data_time, xy_confidence);
                if (!success) fprintf(stderr, "applyIntersectCls() was failed.\n");
            }
        }
        else if (type == dg::DATA_RoadLR)
        {
            double cls = vdata[1];
            double cls_conf = vdata[2];
            int lr_cls;
            double lr_confidence;
            if (m_roadlr.applyPreprocessed(cls, cls_conf, data_time, lr_cls, lr_confidence))
            {
                bool success = m_localizer.applyRoadLR(lr_cls, data_time, lr_confidence);
                if (!success) fprintf(stderr, "applyRoadLR() was failed.\n");
            }
        }
        else if (type == dg::DATA_RoadTheta)
        {
            double theta = vdata[3];
            double confidence = vdata[4];
            bool success = m_localizer.applyRoadTheta(theta, data_time, confidence);
            if (!success) fprintf(stderr, "applyRoadTheta() was failed.\n");
        }

        // process path generation
        if(m_dest_defined && m_path_generation_pended && m_localizer.isPoseInitialized())
        {
            printf("[run]]\n");
            if(updateDeepGuiderPath(getPose(), m_dest)) m_path_generation_pended = false;
        }

        // update
        if (update_gui)
        {
            m_cam_mutex.lock();
            m_cam_image = video_image;
            m_cam_capture_time = data_time;
            m_cam_mutex.unlock();

            // process vision modules
            if(!m_threaded_run_modules)
            {
                if (m_enable_intersection==1 && intersection_file.empty()) procIntersectionClassifier();
                if (m_enable_ocr==1 && ocr_file.empty()) procOcr();
                if (m_enable_vps==1 && vps_file.empty()) procVps();
                if (m_enable_roadlr==1 && roadlr_file.empty()) procRoadLR();
                if (m_enable_roadtheta==1 && roadtheta_file.empty()) procRoadTheta();
                if (m_enable_exploration==1 && exploration_file.empty()) procExploration();         
                if (m_enable_logo==1) procLogo();
            }

            // process Guidance
            procGuidance(data_time);

            // draw GUI display
            dg::Pose2 pose_m = getPose();
            dg::Pose2 pose_px = m_painter.cvtValue2Pixel(pose_m);
            if (m_gui_auto_scroll && m_localizer.isPoseInitialized()) m_viewport.centerizeViewportTo(pose_px);
            m_viewport.getViewportImage(gui_image);
            drawGuiDisplay(gui_image, m_viewport.offset(), m_viewport.zoom());

            // recording
            if (m_video_recording) m_video_gui << gui_image;

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            printf("Iteration: %d (it took %lf seconds)\n", itr, t2 - t1);

            // gui display
            cv::imshow(m_winname, gui_image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == cx::KEY_ESC) break;
            if (key == '1') m_viewport.setZoom(1);
            if (key == '2') m_viewport.setZoom(2);
            if (key == '3') m_viewport.setZoom(3);
            if (key == '4') m_viewport.setZoom(4);
            if (key == '0') m_viewport.setZoom(0.1);
            if (key == 'a') m_gui_auto_scroll = !m_gui_auto_scroll;  // toggle auto scroll of the map view
            if (key == 'g' || key == 'G') m_apply_gps = !m_apply_gps;
            if (key == 'm' || key == 'M') m_apply_imu = !m_apply_imu;
            if (key == 'o' || key == 'O')
            {
                m_apply_odometry = !m_apply_odometry;
                if(m_enable_odometry && m_apply_odometry) m_localizer.resetOdometry();
            }
            if (key == 'v' || key == 'V') m_apply_vps = !m_apply_vps;
            if (key == 'p' || key == 'P') m_apply_ocr = !m_apply_ocr;
            if (key == 'i' || key == 'I') m_apply_intersection = !m_apply_intersection;
            if (key == 'l' || key == 'L') m_apply_roadlr = !m_apply_roadlr;
            if (key == 't' || key == 'T') m_apply_roadtheta = !m_apply_roadtheta;
            if (key == 'k') m_show_ekf_pose = !m_show_ekf_pose;
            if (key == 'j') m_localizer.toggleEnablePathProjection();
            if (key == 83) itr += 30;   // Right Key

            // update iteration
            itr++;
        }
    }

    // end system
    printf("Shutdown deepguider system...\n");
    terminateThreadFunctions();
    if(m_video_recording) m_video_gui.release();
    if(m_video_recording) printf("\trecording closed\n");
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
    return updateDeepGuiderPath(curr_pose, m_dest);
}

void DeepGuider::procGpsData(dg::LatLon gps_datum, dg::Timestamp ts)
{
    if (!m_apply_gps) return;

    VVS_CHECK_TRUE(m_localizer.applyGPS(gps_datum, ts));
}

void DeepGuider::procImuData(double ori_w, double ori_x, double ori_y, double ori_z, dg::Timestamp ts)
{
    if (!m_apply_imu) return;

    auto euler = cx::cvtQuat2EulerAng(ori_w, ori_x, ori_y, ori_z);
    VVS_CHECK_TRUE(m_localizer.applyIMUCompass(euler.z, ts));
}

void DeepGuider::procOdometryData(double x, double y, double theta, dg::Timestamp ts)
{
    if (!m_apply_odometry) return;

    VVS_CHECK_TRUE(m_localizer.applyOdometry(Pose2(x,y,theta), ts));
}

void DeepGuider::procMouseEvent(int evt, int x, int y, int flags)
{
    m_viewport.procMouseEvent(evt, x, y, flags);

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
        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        dg::Point2 dest = m_painter.cvtPixel2Value(px);
        setDeepGuiderDestination(dest);
    }
    else if (evt == cv::EVENT_RBUTTONDOWN)
    {
        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d val = m_painter.cvtPixel2Value(px);
        dg::Timestamp time = -1;
        dg::Pose2 pose = getPose(&time);
        pose.x = val.x;
        pose.y = val.y;
        if(time<0) time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        m_localizer.setPose(pose, time);
        printf("[Localizer] set user pose: x = %lf, y = %lf\n", val.x, val.y);
    }
    else if (evt == cv::EVENT_RBUTTONUP)
    {
    }
    else if (evt == cv::EVENT_MOUSEWHEEL)
    {
    }
}

bool DeepGuider::setDeepGuiderDestination(dg::Point2F dest)
{
    if(!m_localizer.isPoseInitialized())
    {
        m_dest = dest;
        m_dest_defined = true;
        m_path_generation_pended = true;
        return true;
    }
    
    if (!updateDeepGuiderPath(getPose(), dest)) return false;
    m_dest = dest;
    m_dest_defined = true;
    m_path_generation_pended = false;
    
    return true;
}

bool DeepGuider::updateDeepGuiderPath(dg::Point2F start, dg::Point2F dest)
{
    // stop other works
    m_exploration_state_count = 0;

    if (m_enable_tts) putTTS("Regenerate path!");
    if (m_enable_mapserver)
    {
        Path path;
        dg::LatLon gps_start = toLatLon(start);
        dg::LatLon gps_dest = toLatLon(dest);
        bool ok = m_map_manager.getPath(gps_start.lat, gps_start.lon, start.floor, gps_dest.lat, gps_dest.lon, dest.floor, path);
        if (!ok)
        {
            printf("[MapManager] fail to find path to (lat=%lf, lon=%lf)\n", gps_dest.lat, gps_dest.lon);
            return false;
        }
        dg::Pose2 pose = m_localizer.getPose();
        m_localizer.setPose(pose);
        setPath(path);
        printf("[MapManager] New path generated to (lat=%lf, lon=%lf)\n", gps_dest.lat, gps_dest.lon);
    }
    else
    {
        Path path;
        bool ok = m_map.getPath(start, dest, path);
        if (!ok) return false;
        setPath(path);
        dg::LatLon gps_dest = toLatLon(dest);
        printf("[OfflineMap] New path generated to (lat=%lf, lon=%lf)\n", gps_dest.lat, gps_dest.lon);
    }

    // guidance: init map and path for guidance
    m_guider_mutex.lock();
    //VVS_CHECK_TRUE(m_guider.initiateNewGuidance());
    //VVS_CHECK_TRUE(m_guider.initiateNewGuidance(start, dest));
    dg::TopometricPose pose_topo = getPoseTopometric();
    VVS_CHECK_TRUE(m_guider.initiateNewGuidance(pose_topo, dest));
    m_guider_mutex.unlock();
    printf("\tGuidance is updated with new map and path!\n");

    return true;    
}

void DeepGuider::drawGuiDisplay(cv::Mat& image, const cv::Point2d& view_offset, double view_zoom)
{
    // draw robot trajectory
    dg::Pose2 pose_m = getPose();
    m_painter.drawPoint(m_map_image, pose_m, m_gui_robot_trj_radius, m_gui_robot_color);

    // draw path
    dg::Path path = getPath();
    if (!path.empty())
    {
        m_painter.drawPath(image, &m_map, &path, view_offset, view_zoom, m_gui_path_color);
    }

    // draw cam image on the GUI map (image is a map Mat)
    if (m_cam_image.empty()) return;
    cv::Rect image_rc(0, 0, image.cols, image.rows);

    cv::Mat video_image;
    cv::Point video_offset(m_video_win_margin, image.rows - m_video_win_margin - m_video_win_height);
    double video_resize_scale = (double)m_video_win_height / m_cam_image.rows;
    m_cam_mutex.lock();
    cv::resize(m_cam_image, video_image, cv::Size(), video_resize_scale * 0.76, video_resize_scale);
    m_cam_mutex.unlock();
    cv::Rect win_rect = cv::Rect(video_offset, video_image.size()) & image_rc;
    video_image.copyTo(image(win_rect));
    cv::Rect video_rect = win_rect;

    // draw 360cam image
    if (m_enable_360cam)
    {
        cv::Mat result_image;
        m_360cam_mutex.lock();
        if(!m_360cam_image.empty())
        {
            cv::resize(m_360cam_image, result_image, cv::Size((int)(win_rect.height * 0.9), win_rect.height));
        }
        m_360cam_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // draw 360cam_crop image
    if (m_enable_360cam_crop)
    {
        cv::Mat result_image;
        m_360cam_crop_mutex.lock();
        if(!m_360cam_crop_image.empty())
        {
            cv::resize(m_360cam_crop_image, result_image, cv::Size((int)(win_rect.height * 0.9), win_rect.height));
        }
        m_360cam_crop_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }


    // draw intersection result
    if (m_enable_intersection)
    {
        cv::Mat result_image;
        m_intersection_mutex.lock();
        if(!m_intersection_image.empty())
        {
            cv::resize(m_intersection_image, result_image, cv::Size((int)(win_rect.height * 0.9), win_rect.height));
        }
        m_intersection_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // draw roadlr result
    if (m_enable_roadlr)
    {
        cv::Mat result_image;
        m_roadlr_mutex.lock();
        if (!m_roadlr_image.empty())
        {
            cv::resize(m_roadlr_image, result_image, cv::Size((int)(win_rect.height * 0.9), win_rect.height));
        }
        m_roadlr_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // draw roadtheta result
    if (m_enable_roadtheta)
    {
        cv::Mat result_image;
        m_roadtheta_mutex.lock();
        if (!m_roadtheta_image.empty())
        {
            cv::resize(m_roadtheta_image, result_image, cv::Size(win_rect.height*0.9, win_rect.height));
        }
        m_roadtheta_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // draw vps result
    if (m_enable_vps)
    {
        cv::Mat result_image;
        m_vps_mutex.lock();
        dg::Point2 sv_xy = m_vps_xy;
        dg::Polar2 sv_relative = m_vps_relative;          
        if (!m_vps_image.empty())
        {
            double fy = (double)win_rect.height / m_vps_image.rows;
            cv::resize(m_vps_image, result_image, cv::Size(), fy * 0.7, fy);
        }
        m_vps_mutex.unlock();

        if (!result_image.empty())
        {
            // Draw matched streetview position
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));

            double sv_distance = norm(pose_m - sv_xy);
            if (sv_distance <= m_vps_max_error_distance)
            {
                m_painter.drawPoint(image, sv_xy, 20, m_gui_vps_color, view_offset, view_zoom);  // sky color for streetview position
            }
            else
            {
                m_painter.drawPoint(image, sv_xy, 20, cv::Vec3b(128, 128, 128), view_offset, view_zoom);  // sky color for streetview position
            }

            // Draw virtual robot position computed from relative pose
            dg::Pose2 pose = getPose();
            double poi_theta = pose.theta + sv_relative.ang;
            double rx = sv_xy.x - sv_relative.lin * cos(poi_theta);
            double ry = sv_xy.y - sv_relative.lin * sin(poi_theta);
            m_painter.drawPoint(image, Point2(rx,ry), 20, m_gui_vps_rpose_color, view_offset, view_zoom); // light black for  streetview position with relative pose
        }
    }

    // draw ocr result
    if (m_enable_ocr)
    {
        cv::Mat result_image;
        m_ocr_mutex.lock();
        if (!m_ocr_image.empty())
        {
            double fy = (double)win_rect.height / m_ocr_image.rows;
            cv::resize(m_ocr_image, result_image, cv::Size(), fy * 0.7, fy);
        }
        m_ocr_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // draw exploration result
    if (m_enable_exploration)
    {
        cv::Mat result_image;
        m_exploration_mutex.lock();
        if (!m_exploration_image.empty())
        {
            cv::resize(m_exploration_image, result_image, cv::Size(win_rect.height, win_rect.height));
        }
        m_exploration_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // draw logo result
    if (m_enable_logo)
    {
        cv::Mat result_image;
        m_logo_mutex.lock();
        if(!m_logo_image.empty())
        {
            double fy = (double)win_rect.height / m_logo_image.rows;
            cv::resize(m_logo_image, result_image, cv::Size(), fy * 0.7, fy);
        }
        m_logo_mutex.unlock();

        if (!result_image.empty())
        {
            win_rect = cv::Rect(win_rect.x + win_rect.width + m_video_win_gap, win_rect.y, result_image.cols, result_image.rows);
            if ((win_rect & image_rc) == win_rect) result_image.copyTo(image(win_rect));
        }
    }

    // current localization
    dg::Pose2 pose_metric = getPose();
    dg::TopometricPose pose_topo = getPoseTopometric();
    dg::LatLon pose_gps = toLatLon(pose_metric);
    double pose_confidence = m_localizer.getPoseConfidence();

    // draw robot on the map
    if (m_show_ekf_pose)
    {
        dg::Pose2 pose_ekf = m_localizer.getEkfPose();
        m_painter.drawPoint(image, pose_ekf, 10 * 2, cx::COLOR_WHITE, view_offset, view_zoom);
        m_painter.drawPoint(image, pose_ekf, 8 * 2, cx::COLOR_BLUE, view_offset, view_zoom);
        cv::Point2d px = (m_painter.cvtValue2Pixel(pose_ekf) - view_offset) * view_zoom;
        cv::line(image, px, px + 10 * 2 * view_zoom * dg::Point2(cos(pose_ekf.theta), -sin(pose_ekf.theta)) + cv::Point2d(0.5, 0.5), cx::COLOR_YELLOW, (int)(2 * 2 * view_zoom + 0.5));
    }
    else
    {
        m_painter.drawPoint(image, pose_metric, 10 * 2, cx::COLOR_YELLOW, view_offset, view_zoom);
        m_painter.drawPoint(image, pose_metric, 8 * 2, cx::COLOR_BLUE, view_offset, view_zoom);
        cv::Point2d px = (m_painter.cvtValue2Pixel(pose_metric) - view_offset) * view_zoom;
        cv::line(image, px, px + 10 * 2 * view_zoom * dg::Point2(cos(pose_metric.theta), -sin(pose_metric.theta)) + cv::Point2d(0.5, 0.5), cx::COLOR_YELLOW, (int)(2 * 2 * view_zoom + 0.5));
    }

    // draw status message (localization)
    cv::String info_topo = cv::format("Node: %zu, Edge: %d, D: %.3fm", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist);
    cv::putText(image, info_topo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
    cv::putText(image, info_topo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    std::string info_confidence = cv::format("Confidence: %.2lf", pose_confidence);
    cv::putText(image, info_confidence, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
    cv::putText(image, info_confidence, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

    // draw status message (Auto Scroll & Sensor Connections)
    cv::Point gui_xy(10, 100);
    cv::Scalar gui_bg(255, 255, 255);
    cv::Scalar gui_fg(200, 0, 0);
    double gui_fscale = 0.8;
    std::string gui_msg  = cv::format("Zoom(0~4): %.1lfx", m_viewport.zoom());
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_fg, 2);
    gui_xy.y += 40;

    gui_msg = (m_gui_auto_scroll) ? "AutoScroll(A): On" : "AutoScroll(A): Off";
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (m_gui_auto_scroll) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_fg, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, cv::Scalar(128, 128, 128), 2);
    gui_xy.y += 40;

    gui_msg = "GPS Traj: ----";
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, cx::COLOR_BLACK, 5);
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, m_gui_gps_color, 2);
    gui_xy.y += 40;

    gui_msg = "Robot Traj: ---";
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, m_gui_robot_color, 2);
    gui_xy.y += 40;
 
    // sensor status
    cv::Scalar gui_active(255, 0, 0);
    cv::Scalar gui_deactive(128, 128, 128);
    gui_msg = "GPS(G)";
    bool active = m_apply_gps;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if(active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "IMU(M)";
    active = m_enable_imu && m_apply_imu;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "ODO(O)";
    active = m_enable_odometry && m_apply_odometry;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "VPS(V)";
    active = m_enable_vps && m_apply_vps;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "POI(P)";
    active = m_enable_ocr && m_apply_ocr;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "Intersect(I)";
    active = m_enable_intersection && m_apply_intersection;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "RoadLR(L)";
    active = m_enable_roadlr && m_apply_roadlr;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

    gui_msg = "RoadTheta(T)";
    active = m_enable_roadtheta && m_apply_roadtheta;
    cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_bg, 5);
    if (active) cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_active, 2);
    else cv::putText(image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, gui_deactive, 2);
    gui_xy.y += 40;

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
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 8, cv::Scalar(0, 255, 0), 16);
        cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 8, cv::Scalar(0, 0, 0), 8);
    }
}

void DeepGuider::drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect)
{
    if(!m_dest_defined || m_path_generation_pended) return;
    if(guide.actions.empty()) return;

    int guide_cx = rect.x + rect.width / 2;
    int guide_cy = rect.y + m_icon_forward.rows / 2 + 40;
    cv::Point center_pos(guide_cx, guide_cy);

    std::string dir_msg;
    dg::GuidanceManager::Motion cmd; 
    cv::Point msg_offset;
    float fontsize;

    for (size_t i = 0; i < guide.actions.size(); i++)
    {
        cmd = guide.actions[i].cmd;   
    
    
        if (cmd == dg::GuidanceManager::Motion::GO_FORWARD || cmd == dg::GuidanceManager::Motion::CROSS_FORWARD || cmd == dg::GuidanceManager::Motion::ENTER_FORWARD || cmd == dg::GuidanceManager::Motion::EXIT_FORWARD)
        {
            cv::Mat& icon = m_icon_forward;
            cv::Mat& mask = m_mask_forward;
            int x1 = center_pos.x - icon.cols / 2;
            int y1 = center_pos.y - icon.rows / 2;
            cv::Rect rect(x1, y1, icon.cols, icon.rows);
            if (i == guide.actions.size()-1)
                if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
            if (cmd == dg::GuidanceManager::Motion::GO_FORWARD) dir_msg = "GO_FORWARD";
            if (cmd == dg::GuidanceManager::Motion::CROSS_FORWARD) dir_msg = "CROSS_FORWARD";
            if (cmd == dg::GuidanceManager::Motion::ENTER_FORWARD) dir_msg = "ENTER_FORWARD";
            if (cmd == dg::GuidanceManager::Motion::EXIT_FORWARD) dir_msg = "EXIT_FORWARD";
        }
        else if (cmd == dg::GuidanceManager::Motion::TURN_LEFT || cmd == dg::GuidanceManager::Motion::CROSS_LEFT || cmd == dg::GuidanceManager::Motion::ENTER_LEFT || cmd == dg::GuidanceManager::Motion::EXIT_LEFT)
        {
            cv::Mat& icon = m_icon_turn_left;
            cv::Mat& mask = m_mask_turn_left;
            int x1 = center_pos.x - icon.cols + icon.cols / 6;
            int y1 = center_pos.y - icon.rows / 2;
            cv::Rect rect(x1, y1, icon.cols, icon.rows);
            if (i == guide.actions.size()-1)
                if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
            if (cmd == dg::GuidanceManager::Motion::TURN_LEFT) dir_msg = "TURN_LEFT";
            if (cmd == dg::GuidanceManager::Motion::CROSS_LEFT) dir_msg = "CROSS_LEFT";
            if (cmd == dg::GuidanceManager::Motion::ENTER_LEFT) dir_msg = "ENTER_LEFT";
            if (cmd == dg::GuidanceManager::Motion::EXIT_LEFT) dir_msg = "EXIT_LEFT";
        }
        else if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT || cmd == dg::GuidanceManager::Motion::CROSS_RIGHT || cmd == dg::GuidanceManager::Motion::ENTER_RIGHT || cmd == dg::GuidanceManager::Motion::EXIT_RIGHT)
        {
            cv::Mat& icon = m_icon_turn_right;
            cv::Mat& mask = m_mask_turn_right;
            int x1 = center_pos.x - icon.cols / 6;
            int y1 = center_pos.y - icon.rows / 2;
            cv::Rect rect(x1, y1, icon.cols, icon.rows);
            if (i == guide.actions.size()-1)
                if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
            if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT) dir_msg = "TURN_RIGHT";
            if (cmd == dg::GuidanceManager::Motion::CROSS_RIGHT) dir_msg = "CROSS_RIGHT";
            if (cmd == dg::GuidanceManager::Motion::ENTER_RIGHT) dir_msg = "ENTER_RIGHT";
            if (cmd == dg::GuidanceManager::Motion::EXIT_RIGHT) dir_msg = "EXIT_RIGHT";
        }
        else if (cmd == dg::GuidanceManager::Motion::TURN_BACK)
        {
            cv::Mat& icon = m_icon_turn_back;
            cv::Mat& mask = m_mask_turn_back;
            int x1 = center_pos.x - icon.cols / 2;
            int y1 = center_pos.y - icon.rows / 2;
            cv::Rect rect(x1, y1, icon.cols, icon.rows);
            if (i == guide.actions.size()-1)
                if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
            dir_msg = "TURN_BACK";
        }
        else if (cmd == dg::GuidanceManager::Motion::STOP)
        {            cv::Mat& icon = m_icon_forward;
            cv::Mat& mask = m_mask_forward;
            int x1 = center_pos.x - icon.cols / 2;
            int y1 = center_pos.y - icon.rows / 2;
            cv::Rect rect(x1, y1, icon.cols, icon.rows);
            if (i == guide.actions.size()-1)
                if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
            dir_msg = "Arrival"; 
        }
        else
        {
            dir_msg = "[Guide] N/A";
        }   

        // show direction message
        if (i == 0)
        {
            msg_offset = rect.tl() + cv::Point(10, 40);
            fontsize = 2.0;
        }
        else   
        {
            msg_offset = center_pos + cv::Point(50, 80);
            fontsize = 1.0;
        }
        
        cv::putText(image, dir_msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 255, 255), 12);
        cv::putText(image, dir_msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(255, 0, 0), 4);

    }
 
    // if (guide.heading_node_id == 0) //if next goal is final
    // {
    //     dir_msg = "Arrival"; 
    //     msg_offset = center_pos + cv::Point(50, 80);
    //     fontsize = 1.0;
    //     cv::putText(image, dir_msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 255, 255), 12);
    //     cv::putText(image, dir_msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(255, 0, 0), 4);
    // }

    // show distance message
    msg_offset = center_pos + cv::Point(50, 20);
    std::string distance = cv::format("After = %.1lfm", guide.distance_to_remain);
    cv::putText(image, distance.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 14);
    cv::putText(image, distance.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 4);

    // show guidance message
    msg_offset = rect.tl() + cv::Point(0, rect.height + 25);
    cv::putText(image, guide.msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 4);
    cv::putText(image, guide.msg.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
}

void DeepGuider::procGuidance(dg::Timestamp ts)
{
    if(!m_dest_defined || m_path_generation_pended) return;

    // get updated pose & localization confidence
    dg::TopometricPose pose_topo = getPoseTopometric();
    dg::Pose2 pose_metric = getPose();
    double pose_confidence = m_localizer.getPoseConfidence();

    // Guidance: generate navigation guidance
    dg::GuidanceManager::GuideStatus cur_status;
    dg::GuidanceManager::Guidance cur_guide;
    m_map_mutex.lock();
    dg::Node* node = m_map.getNode(pose_topo.node_id);
    m_map_mutex.unlock();
    if(node==nullptr)
    {
        printf("[Guidance] Error - Undefined localization node: %zu!\n", pose_topo.node_id);
        return;
    }
    m_guider_mutex.lock();
    if(m_guider.m_dxrobot_usage)    m_guider.updateWithRobot(pose_topo, pose_metric);
    else m_guider.update(pose_topo, pose_metric);
    cur_status = m_guider.getGuidanceStatus();
    cur_guide = m_guider.getGuidance();
    
    m_guider_mutex.unlock();
 
    // tts guidance message
    if (m_enable_tts && cur_guide.announce && !cur_guide.actions.empty())
    {
        dg::GuidanceManager::Motion cmd = cur_guide.actions[0].cmd;
        //if (cmd != m_guidance_cmd)
        {
            std::string tts_msg;
            /*
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
            if (cur_guide.distance_to_remain > 25)
                tts_msg = "Go forward";
            */
           tts_msg = cur_guide.msg;
            if(!tts_msg.empty()) 
            {
                putTTS(tts_msg.c_str());
                printf("[tts] %s\n", cur_guide.msg.c_str());
            }
            m_guidance_cmd = cmd;
        }
    }

    if (cur_status != m_guidance_status)
    {
        // check near arrival
        if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_OPTIMAL_VIEW && cur_guide.announce)
        {
            printf("Near arrival!\n");
            if (m_enable_tts) putTTS("Near arrival!");
            m_exploration_state_count = m_exploration_state_count_max;  //Enter exploration mode until state_count becomes 0 from count_max
        }

        // check arrival
        if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_ARRIVED && cur_guide.announce)
        {
            printf("Arrived to destination!\n");
            m_dest_defined = false;
            m_path_generation_pended = false;
            clearPath();
            if (m_enable_tts) putTTS("Arrived to destination!");
            m_exploration_state_count = m_exploration_state_count_max;  //Enter exploration mode until state_count becomes 0 from count_max
        }

        // check out of path
        if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_OOP || cur_status == dg::GuidanceManager::GuideStatus::GUIDE_LOST)
        {
            printf("GUIDANCE: out of path detected!\n");
            VVS_CHECK_TRUE(updateDeepGuiderPath(pose_metric, m_dest));
        }

        // check lost
        /*
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
        */
    }
    m_guidance_status = cur_status;
}

cv::Mat DeepGuider::crop_image(cv::Mat cam_image, int num, int idx) // idx, 0: left, 1: front, 2:right
{
	int w,h;
	w = (int)(cam_image.cols/num);
	h = cam_image.rows;
	cv::Rect rect(idx*w, 0, w, h);  // (X,Y,W,H), crop front image (Left, Front, Right)
	return cam_image(rect);
}

bool DeepGuider::procIntersectionClassifier()
{
	cv::Mat cam_image;
	cv::Mat cam_image_for_draw;
	dg::Timestamp capture_time;
	double txt_scale;

	if(!get_cam_image(cam_image, cam_image_for_draw, 1, txt_scale, capture_time, m_intersection.timestamp()))
	{
	    return false;
	}

    dg::Point2 xy;
    double confidence;
    bool valid_xy = false;
    if (m_intersection.apply(cam_image, capture_time, xy, confidence, valid_xy))
    {
        if(m_apply_intersection && valid_xy) m_localizer.applyIntersectCls(xy, capture_time, confidence);
        m_intersection.print();

        m_intersection.draw(cam_image_for_draw, txt_scale);  // 2.0 when w is 1280(webcam), 1.0 when w is 640(360cam_crop)
        m_intersection_mutex.lock();
        m_intersection_image = cam_image_for_draw;
        m_intersection_mutex.unlock();
        return true;
    }
    else
    {
        m_intersection.print();
    }
    return false;
}

bool DeepGuider::procLogo()
{
    m_cam_mutex.lock();
    dg::Timestamp capture_time = m_cam_capture_time;
    if (m_cam_image.empty() || capture_time <= m_logo.timestamp())
    {
        m_cam_mutex.unlock();
        return false;
    }
    cv::Mat cam_image = m_cam_image.clone();
    m_cam_mutex.unlock();

    std::vector<dg::Point2> poi_xys;
    std::vector<dg::Polar2> relatives;
    std::vector<double> poi_confidences;
    if (m_logo.apply(cam_image, capture_time, poi_xys, relatives, poi_confidences))
    {
        for (int k = 0; k < (int)poi_xys.size(); k++)
        {
            m_localizer.applyPOI(poi_xys[k], relatives[k], capture_time, poi_confidences[k]);
        }
        m_logo.print();
        return true;
    }
    else
    {
        m_logo.print();
    }

    return false;
}

bool DeepGuider::procOcr()
{
    m_cam_mutex.lock();
    dg::Timestamp capture_time = m_cam_capture_time;
    if (m_cam_image.empty() || capture_time <= m_ocr.timestamp())
    {
        m_cam_mutex.unlock();
        return false;
    }
    cv::Mat cam_image = m_cam_image.clone();
    m_cam_mutex.unlock();

    std::vector<dg::POI*> pois;
    std::vector<dg::Polar2> relatives;
    std::vector<double> poi_confidences;
    if (m_ocr.apply(cam_image, capture_time, pois, relatives, poi_confidences))
    {
        if (m_apply_ocr)
        {
            for (int k = 0; k < (int)pois.size(); k++)
            {
                m_localizer.applyPOI(*(pois[k]), relatives[k], capture_time, poi_confidences[k]);
            }
        }
    }

    std::vector<OCRResult> ocrs = m_ocr.get();
    if(!ocrs.empty())
    {
        m_ocr.draw(cam_image, 1.5);
        m_ocr_mutex.lock();
        m_ocr_image = cam_image;
        m_ocr_mutex.unlock();
        return true;
    }
    else
    {
        m_ocr.print();
        return false;
    }
}

bool DeepGuider::procRoadTheta()
{
    m_cam_mutex.lock();
    dg::Timestamp capture_time = m_cam_capture_time;
    if (m_cam_image.empty() || capture_time <= m_roadtheta.timestamp())
    {
        m_cam_mutex.unlock();
        return false;
    }
    cv::Mat cam_image = m_cam_image.clone();
    m_cam_mutex.unlock();

    double theta, confidence;
    if (m_roadtheta.apply(cam_image, capture_time, theta, confidence))
    {
        if (m_apply_roadtheta) m_localizer.applyRoadTheta(theta, capture_time, confidence);
        m_roadtheta.print();

        m_roadtheta.draw(cam_image, 4);
        m_roadtheta_mutex.lock();
        m_roadtheta_image = cam_image;
        m_roadtheta_mutex.unlock();
        return true;
    }
    else
    {
        m_roadtheta.print();
    }

    return false;
}

bool DeepGuider::procVps()
{
    m_cam_mutex.lock();
    dg::Timestamp capture_time = m_cam_capture_time;
    if (m_cam_image.empty() || capture_time <= m_vps.timestamp())
    {
        m_cam_mutex.unlock();
        return false;
    }
    cv::Mat cam_image = m_cam_image.clone();
    m_cam_mutex.unlock();

    dg::Point2 sv_xy;
    dg::Polar2 relative;
    double sv_confidence;
    if (m_vps.apply(cam_image, capture_time, sv_xy, relative, sv_confidence, m_vps_gps_accuracy, m_vps_load_dbfeat, m_vps_save_dbfeat))
    {
        if (m_apply_vps) m_localizer.applyVPS(sv_xy, relative, capture_time, sv_confidence);
        m_vps.print();

	    cv::Mat sv_image = m_vps.getViewImage();  // I will return naver or custom db image according to m_vps_use_custom_image_server
        if(!sv_image.empty())
        {
            m_vps.draw(sv_image, 2.0);
            m_vps_mutex.lock();
            m_vps_image = sv_image;
            m_vps_xy = sv_xy;
			m_vps_id = m_vps.getViewID();
            m_vps_relative = relative;
            m_vps_mutex.unlock();
        }
		else
		{
			printf("[vps] empty sv_image : m_vps_id %ld\n", m_vps.getViewID());
		}
        return true;
    }
    else
    {
        m_vps.print();
    }
    return false;
}

bool DeepGuider::get_cam_image(cv::Mat& cam_image, cv::Mat& cam_image_for_draw, int idx_for_draw, double& txt_scale, dg::Timestamp& capture_time, dg::Timestamp capture_time_prev)
{
	int num_of_hstack_image = 3;
   	m_cam_mutex.lock();
	capture_time = m_cam_capture_time;
   	m_cam_mutex.unlock();

	if (m_enable_360cam_crop)  // use 360cam
	{
    	m_360cam_crop_mutex.lock();
	    if(m_360cam_crop_image.empty() || capture_time <= capture_time_prev)
	    {
	        m_360cam_crop_mutex.unlock();
	        return false;
	    }
	    cam_image = m_360cam_crop_image.clone();
    	m_360cam_crop_mutex.unlock();
		cam_image_for_draw = crop_image(cam_image, num_of_hstack_image, idx_for_draw); // num=3(l,f,r),  idx=0 (0:left, 1:front, 2:right)
	}
	else  // use webcam
	{
    	m_cam_mutex.lock();
	    if(m_cam_image.empty() || capture_time <= capture_time_prev)
	    {
	        m_cam_mutex.unlock();
	        return false;
	    }
	    cam_image = m_cam_image.clone();
	    m_cam_mutex.unlock();
		cam_image_for_draw = cam_image;
	}

	if(cam_image_for_draw.empty() || cam_image.empty())
	{
		return false;
	}

	txt_scale = 2.0*(((double)cam_image_for_draw.cols)/1280.0);  // 2.0 when w is 1280(webcam), 1.0 when w is 640(360cam_crop)

	return true;
}


bool DeepGuider::procRoadLR()
{
	cv::Mat cam_image;
	cv::Mat cam_image_for_draw;
	dg::Timestamp capture_time;
	double txt_scale;

	if(!get_cam_image(cam_image, cam_image_for_draw, 1, txt_scale, capture_time, m_intersection.timestamp()))
	{
	    return false;
	}

    int lr_pose;
    double lr_confidence;

    if (m_roadlr.apply(cam_image, capture_time, lr_pose, lr_confidence))
    {
        if (m_apply_roadlr) m_localizer.applyRoadLR(lr_pose, capture_time, lr_confidence);
        m_roadlr.print();

        m_roadlr.draw(cam_image_for_draw, txt_scale);  // 2.0 when w is 1280(webcam), 1.0 when w is 640(360cam_crop)
        m_roadlr_mutex.lock();
        m_roadlr_image = cam_image_for_draw;
        m_roadlr_mutex.unlock();
        return true;
    }
    else
    {
        m_roadlr.print();
    }

    return false;
}

bool DeepGuider::procExploration()
{
    // printf("################m_exploration_state_count = %d\n", m_exploration_state_count);
    if ( m_exploration_state_count <= 0)
    {
        m_exploration_state_count = 0;
        return false;
    }    
    m_exploration_state_count--;

    m_cam_mutex.lock();
    dg::Timestamp capture_time = m_cam_capture_time;
    if (m_cam_image.empty() || capture_time <= m_active_nav.timestamp())
    {
        m_cam_mutex.unlock();
        return false;
    }
    cv::Mat cam_image = m_cam_image.clone();
    m_cam_mutex.unlock();
	
	std::vector<ExplorationGuidance> actions;
	GuidanceManager::GuideStatus status;
    
    GuidanceManager::Guidance guidance;  
    std::vector<GuidanceManager::Action> actionss;
    GuidanceManager::Action action;
    actionss.push_back(action);
    guidance.actions = actionss;
    guidance.guide_status = GuidanceManager::GuideStatus::GUIDE_OPTIMAL_VIEW;

	if (m_active_nav.apply(cam_image, guidance, capture_time))
	{
		m_active_nav.get(actions, status);

		for (int k = 0; k < (int)actions.size(); k++)
		{
			printf("\t exploration action %d: [%lf, %lf, %lf]\n", k, actions[k].theta1, actions[k].d, actions[k].theta2);
		}        
        m_active_nav.print();
        m_active_nav.draw(cam_image, 2);
        m_exploration_mutex.lock();
        m_exploration_image = cam_image;
        m_exploration_mutex.unlock();

        double z = (actions[0].theta1 * actions[0].theta1) + (actions[0].d * actions[0].d) + (actions[0].theta2 * actions[0].theta2);
        if ( z < 0.0001 )
        {
            putTTS("Arrived at destination point");
            m_exploration_state_count = 0;
            return true;
        }
        if (!m_enable_exploration || m_exploration_state_count <= 0) return false;
        std::string msg1, msg2, msg3;

        if (actions[0].theta1 > 0){
            msg1 = cv::format("Turn right %d degree.", (int)(actions[0].theta1 + 0.5));
            putTTS((const char*)msg1.c_str());            
        }
        else if (actions[0].theta1 < 0)
        {
            msg1 = cv::format("Turn left %d degree.", (int)(-1*actions[0].theta1 + 0.5));
            putTTS((const char*)msg1.c_str());                        
        }
    
        if (!m_enable_exploration || m_exploration_state_count <= 0) return false;
        // std::this_thread::sleep_for(std::chrono::seconds(5));
        // if (!m_enable_exploration || m_exploration_state_count <= 0) return false;

        if (actions[0].d > 0){
            msg2 = cv::format("Move %.2f meters.", (actions[0].d + 0.5));
            putTTS((const char*)msg2.c_str());            
        }
        else if (actions[0].d < 0)
        {
            msg2 = cv::format("Move back %.2f meters.", (-1*actions[0].d + 0.5));
            putTTS((const char*)msg2.c_str());                        
        }
            
        if (!m_enable_exploration || m_exploration_state_count <= 0) return false;

        // std::this_thread::sleep_for(std::chrono::seconds(5));
        // if (!m_enable_exploration || m_exploration_state_count <= 0) return false;

        if (actions[0].theta2 > 0){
            msg3 = cv::format("Turn right %d degree.", (int)(actions[0].theta2 + 0.5));
            putTTS((const char*)msg3.c_str());            
        }
        else if (actions[0].theta2 < 0){
            msg3 = cv::format("Turn left %d degree.", (int)(-1*actions[0].theta2 + 0.5));
            putTTS((const char*)msg3.c_str());            
        }

        if (!m_enable_exploration || m_exploration_state_count <= 0) return false;
        
        std::this_thread::sleep_for(std::chrono::seconds(5));

        if (!m_enable_exploration || m_exploration_state_count <= 0) return false;

        return true;        
	}
	else
	{
		m_active_nav.print();
	}

    return false;
}

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

// Thread fnuction for RoadLR
void DeepGuider::threadfunc_roadlr(DeepGuider* guider)
{
    guider->is_roadlr_running = true;
    printf("\troadlr thread starts\n");
    while (guider->m_enable_roadlr)
    {
        guider->procRoadLR();
    }
    guider->is_roadlr_running = false;
    printf("\troadlr thread ends\n");
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

// Thread fnuction for Exploration
void DeepGuider::threadfunc_exploration(DeepGuider* guider)
{
    guider->is_exploration_running = true;
    printf("\texploration thread starts\n");
    while (guider->m_enable_exploration)
    {
        guider->procExploration();
        if (guider->m_exploration_state_count <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    guider->is_exploration_running = false;
    printf("\texploration thread ends\n");
}


// Thread fnuction for tts
void DeepGuider::threadfunc_tts(DeepGuider* guider)
{
    guider->is_tts_running = true;
    printf("\ttts thread starts\n");
    while (guider->m_enable_tts)
    {
        guider->procTTS();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    guider->is_tts_running = false;
    printf("\ttts thread ends\n");
}

void DeepGuider::putTTS(const char* msg)
{
    bool discard = false;
    int max_tts_message_cnt = 3;
    std::string msgstr = msg;
    m_tts_mutex.lock();
    if (!m_tts_msg.empty() && m_tts_msg.back() == msgstr) discard = true;
    if (!discard) m_tts_msg.push_back(msg);
    if(m_tts_msg.size() > max_tts_message_cnt)
    {
        int n_erase = (int)m_tts_msg.size() - max_tts_message_cnt;
        m_tts_msg.erase(m_tts_msg.begin(), m_tts_msg.begin() + n_erase);
    }
    m_tts_mutex.unlock();
}

void DeepGuider::procTTS()
{
    std::string tts_msg;
    m_tts_mutex.lock();
    if(!m_tts_msg.empty())
    {
        tts_msg = m_tts_msg[0];
        m_tts_msg.erase(m_tts_msg.begin());
    }
    m_tts_mutex.unlock();

    if(m_enable_tts && !tts_msg.empty()) tts(tts_msg);
}

void DeepGuider::terminateThreadFunctions()
{
    if (vps_thread == nullptr && roadlr_thread == nullptr && ocr_thread == nullptr && logo_thread == nullptr && intersection_thread == nullptr && roadtheta_thread == nullptr && tts_thread == nullptr && exploration_thread == nullptr) return;

    // disable all thread running
    m_enable_intersection = 0;
    m_enable_ocr = 0;
    m_enable_vps = 0;
    m_enable_roadlr = 0;
    m_enable_roadtheta = 0;
    m_enable_exploration = 0;
    m_enable_logo = 0;
    m_exploration_state_count = 0;
    m_enable_tts = false;

    // wait child thread to terminate
    if (intersection_thread && is_intersection_running) intersection_thread->join();
    if (ocr_thread && is_ocr_running) ocr_thread->join();
    if (vps_thread && is_vps_running) vps_thread->join();
    if (roadlr_thread && is_roadlr_running) roadlr_thread->join();
    if (roadtheta_thread && is_roadtheta_running) roadtheta_thread->join();
    if (exploration_thread && is_exploration_running) exploration_thread->join();
    if (logo_thread && is_logo_running) logo_thread->join();
    if (tts_thread && is_tts_running) tts_thread->join();

    // clear threads
    intersection_thread = nullptr;
    ocr_thread = nullptr;
    vps_thread = nullptr;
    roadlr_thread = nullptr;
    roadtheta_thread = nullptr;
    exploration_thread = nullptr;
    logo_thread = nullptr;
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
