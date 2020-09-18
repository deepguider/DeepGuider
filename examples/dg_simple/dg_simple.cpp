#ifndef __DEEPGUIDER_SIMPLE__
#define __DEEPGUIDER_SIMPLE__

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
#ifdef VPSSERVER
    #include <jsoncpp/json/json.h>
    #include <curl/curl.h>
#endif

using namespace dg;
using namespace std;

#define LOAD_PARAM_VALUE(fn, name_cfg, name_var) \
    if (!(fn)[name_cfg].empty()) (fn)[name_cfg] >> name_var;


class DeepGuider
{
public:
    DeepGuider() {}
    ~DeepGuider();

    bool initialize(std::string config_file);
    int run();

    void procMouseEvent(int evt, int x, int y, int flags);
    void procTTS();

protected:
    bool loadConfig(std::string config_file);

    // configuable parameters
    bool m_enable_roadtheta = false;
    bool m_enable_vps = false;
    bool m_enable_logo = false;
    bool m_enable_ocr = false;
    bool m_enable_intersection = false;
    bool m_enable_exploration = false;

    //std::string m_server_ip = "127.0.0.1";        // default: 127.0.0.1 (localhost)
    std::string m_server_ip = "129.254.87.96";      // default: 127.0.0.1 (localhost)
    bool m_threaded_run_python = false;
    std::string m_srcdir = "./../src";              // path of deepguider/src (required for python embedding)

    bool m_use_high_gps = false;             // use high-precision gps (novatel)

    bool m_data_logging = false;
    bool m_enable_tts = false;
    bool m_recording = false;
    int m_recording_fps = 30;
    std::string m_map_image_path = "data/NaverMap_ETRI(Satellite)_191127.png";
    std::string m_gps_input = "data/191115_ETRI_asen_fix.csv";
    std::string m_video_input = "data/191115_ETRI.avi";
    std::string m_recording_header_name = "dg_simple_";

    // local variables
    cx::VideoWriter m_video_gui;
    cx::VideoWriter m_video_cam;
    std::ofstream m_log;
    cv::Mutex m_log_mutex;
    cv::Mat m_map_image;
    cv::Mat m_map_image_original;
    dg::MapPainter m_painter;
    dg::MapCanvasInfo m_map_info;    
    dg::GuidanceManager::Motion m_guidance_cmd = dg::GuidanceManager::Motion::STOP;
    std::list<dg::LatLon> m_gps_history_asen;
    std::list<dg::LatLon> m_gps_history_novatel;

    // global variables
    dg::LatLon m_gps_start;
    dg::LatLon m_gps_dest;
    bool m_dest_defined = false;
    bool m_pose_initialized = false;
    bool m_path_initialized = false;
    std::string m_winname = "DeepGuider";           // title of gui window

    // internal api's
    bool initializeDefaultMap();
    bool setDeepGuiderDestination(dg::LatLon gps_dest);
    bool updateDeepGuiderPath(dg::TopometricPose pose_topo, dg::LatLon gps_start, dg::LatLon gps_dest);
    void drawGuiDisplay(cv::Mat& gui_image);
    void drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect);
    void drawLogo(cv::Mat target_image, std::vector<LogoResult> pois, cv::Size original_image_size);
    void drawOcr(cv::Mat target_image, std::vector<OCRResult> pois, cv::Size original_image_size);
    void drawIntersection(cv::Mat image, IntersectionResult r, cv::Size original_image_size);
    void procGpsData(dg::LatLon gps_datum, dg::Timestamp ts);
    void procGuidance(dg::Timestamp ts);
    bool procIntersectionClassifier();
    bool procLogo();
    bool procOcr();
    bool procVps();
    bool procRoadTheta();

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

    // sub modules
    dg::MapManager m_map_manager;
    dg::EKFLocalizerSinTrack m_localizer;
    dg::VPS m_vps;
    dg::LogoRecognizer m_logo;
    dg::OCRRecognizer m_ocr;
    dg::IntersectionClassifier m_intersection_classifier;
    dg::RoadDirectionRecognizer m_roadtheta;
    dg::GuidanceManager m_guider;
    dg::ActiveNavigation m_active_nav;

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
    if (m_enable_vps) m_vps.clear();
    if(m_enable_logo) m_logo.clear();
    if (m_enable_ocr) m_ocr.clear();
    if(m_enable_intersection) m_intersection_classifier.clear();
    if (m_enable_roadtheta) m_roadtheta.clear();

    bool enable_python = m_enable_roadtheta || m_enable_vps || m_enable_logo || m_enable_ocr || m_enable_intersection || m_enable_exploration;
    if(enable_python) close_python_environment();
}


bool DeepGuider::loadConfig(std::string config_file)
{
    if (config_file.empty())
    {
        return false;
    }

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    cv::FileNode fn = fs.root();
    LOAD_PARAM_VALUE(fn, "enable_intersection", m_enable_intersection);
    LOAD_PARAM_VALUE(fn, "enable_vps", m_enable_vps);
    LOAD_PARAM_VALUE(fn, "enable_poi_logo", m_enable_logo);
    LOAD_PARAM_VALUE(fn, "enable_poi_ocr", m_enable_ocr);
    LOAD_PARAM_VALUE(fn, "enable_roadtheta", m_enable_roadtheta);
    LOAD_PARAM_VALUE(fn, "enable_exploration", m_enable_exploration);

    LOAD_PARAM_VALUE(fn, "server_ip", m_server_ip);
    LOAD_PARAM_VALUE(fn, "threaded_run_python", m_threaded_run_python);
    LOAD_PARAM_VALUE(fn, "dg_srcdir", m_srcdir);

    LOAD_PARAM_VALUE(fn, "use_high_gps", m_use_high_gps);

    LOAD_PARAM_VALUE(fn, "enable_data_logging", m_data_logging);
    LOAD_PARAM_VALUE(fn, "enable_tts", m_enable_tts);
    LOAD_PARAM_VALUE(fn, "video_recording", m_recording);
    LOAD_PARAM_VALUE(fn, "video_recording_fps", m_recording_fps);
    LOAD_PARAM_VALUE(fn, "map_image_path", m_map_image_path);
    LOAD_PARAM_VALUE(fn, "gps_input", m_gps_input);
    LOAD_PARAM_VALUE(fn, "video_input", m_video_input);
    LOAD_PARAM_VALUE(fn, "recording_header_name", m_recording_header_name);

    return true;
}


bool DeepGuider::initialize(std::string config_file)
{
    printf("Initialize deepguider system...\n");

    // load config
    bool ok = loadConfig(config_file);
    if(ok) printf("\tConfiguration %s loaded!\n", config_file.c_str());

    // initialize python
    bool enable_python = m_enable_roadtheta || m_enable_vps || m_enable_ocr || m_enable_logo || m_enable_intersection || m_enable_exploration;
    if (enable_python && !init_python_environment("python3", "", m_threaded_run_python)) return false;
    if(enable_python) printf("\tPython environment initialized!\n");

    // initialize map manager
    m_map_manager.setIP(m_server_ip);
    if (!m_map_manager.initialize()) return false;
    printf("\tMapManager initialized!\n");

    // initialize VPS
    std::string module_path = m_srcdir + "/vps";
    if (m_enable_vps && !m_vps.initialize("vps", module_path.c_str())) return false;
    if (m_enable_vps) printf("\tVPS initialized in %.3lf seconds!\n", m_vps.procTime());

    // initialize OCR
    module_path = m_srcdir + "/ocr_recog";
    if (m_enable_ocr && !m_ocr.initialize("ocr_recognizer", module_path.c_str())) return false;
    if (m_enable_ocr) printf("\tOCR initialized in %.3lf seconds!\n", m_ocr.procTime());

    // initialize Intersection
    module_path = m_srcdir + "/intersection_cls";
    if (m_enable_intersection && !m_intersection_classifier.initialize("intersection_cls", module_path.c_str())) return false;
    if (m_enable_intersection) printf("\tIntersection initialized in %.3lf seconds!\n", m_intersection_classifier.procTime());

    // initialize Logo
    module_path = m_srcdir + "/logo_recog";
    if (m_enable_logo && !m_logo.initialize("logo_recognizer", module_path.c_str())) return false;
    if (m_enable_logo) printf("\tLogo initialized in %.3lf seconds!\n", m_logo.procTime());

    //initialize exploation 
    if (m_enable_exploration && !m_active_nav.initialize()) return false;
    if (m_enable_exploration) printf("\tExploation initialized!\n");

    // initialize default map
    VVS_CHECK_TRUE(initializeDefaultMap());

    // load background GUI image
    m_map_image = cv::imread(m_map_image_path);
    VVS_CHECK_TRUE(!m_map_image.empty());
    m_map_image_original = m_map_image.clone();

    // prepare GUI map
    dg::LatLon ref_node(36.383837659737, 127.367880828442);
    m_painter.setReference(ref_node);
    m_painter.setParamValue("pixel_per_meter", 1.045);
    m_painter.setParamValue("canvas_margin", 0);
    m_painter.setParamValue("canvas_offset", { 344, 293 });
    m_painter.setParamValue("grid_step", 100);
    m_painter.setParamValue("grid_unit_pos", { 120, 10 });
    m_painter.setParamValue("node_radius", 4);
    m_painter.setParamValue("node_font_scale", 0);
    m_painter.setParamValue("node_color", { 255, 50, 255 });
    m_painter.setParamValue("edge_color", { 200, 100, 100 });
    //m_painter.setParamValue("edge_thickness", 2);
    m_map_info = m_painter.getCanvasInfo(m_map_image);

    // draw topology of default map
    dg::Map& map = m_map_manager.getMap();
    VVS_CHECK_TRUE(m_painter.drawMap(m_map_image, m_map_info, map));

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
    // load map
    dg::Map map;
    double lat_center = 36.382517;      // center of deepgudier background map
    double lon_center = 127.372893;     // center of deepguider background map
    double radius = 1000;               // radius of deepguider background map
    m_map_mutex.lock();
    VVS_CHECK_TRUE(m_map_manager.getMap(lat_center, lon_center, radius, map));
    m_map_mutex.unlock();
    printf("\tDefault map is downloaded, n_nodes=%d\n", (int)map.nodes.size());

    // download streetview map
    std::vector<StreetView> sv_list;
    m_map_mutex.lock();
    m_map_manager.getStreetView(lat_center, lon_center, radius, sv_list);
    m_map_mutex.unlock();
    printf("\tStreetviews are downloaded! nViews = %d\n", (int)map.views.size());

    // localizer: set default map to localizer
    dg::LatLon ref_node(36.383837659737, 127.367880828442);
    m_localizer_mutex.lock();
    m_localizer.setParamMotionNoise(0.1, 0.1);
    m_localizer.setParamGPSNoise(0.5);
    m_localizer.setParamValue("offset_gps", {1., 0.});
    VVS_CHECK_TRUE(m_localizer.setReference(ref_node));
    VVS_CHECK_TRUE(m_localizer.loadMap(map));
    m_localizer_mutex.unlock();
    printf("\tDefault map is appyed to Localizer!\n");

    return true;
}


std::vector<std::pair<double, dg::LatLon>> loadExampleGPSData(std::string csv_file = "data/191115_ETRI_asen_fix.csv")
{
    cx::CSVReader csv;
    VVS_CHECK_TRUE(csv.open(csv_file));
    cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 2, 3, 7, 8 }); // Skip the header

    std::vector<std::pair<double, dg::LatLon>> data;
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        double timestamp = row->at(0) + 1e-9 * row->at(1);
        dg::LatLon ll(row->at(2), row->at(3));
        data.push_back(std::make_pair(timestamp, ll));
    }
    return data;
}


int DeepGuider::run()
{
    printf("Run deepguider system...\n");

    // load gps sensor data (ETRI dataset)
    auto gps_data = loadExampleGPSData(m_gps_input);
    VVS_CHECK_TRUE(!gps_data.empty());
    printf("\tSample gps data loaded!\n");

    // load image sensor data (ETRI dataset)
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(m_video_input));
    double video_time_offset = gps_data.front().first - 0.5, video_time_scale = 1.75; // Calculated from 'bag' files
    double video_resize_scale = 0.4;
    cv::Point video_offset(32, 542);
    double video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
    printf("\tSample video data loaded!\n");

    // set initial destination
    //dg::LatLon gps_dest = gps_data.back().second;
    //VVS_CHECK_TRUE(setDeepGuiderDestination(gps_dest));

    // run iteration
    int maxItr = (int)gps_data.size();
    int itr = 300;
    while (itr < maxItr)
    {
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        // gps update
        const dg::LatLon gps_datum = gps_data[itr].second;
        const dg::Timestamp gps_time = gps_data[itr].first;
        procGpsData(gps_datum, gps_time);
        m_painter.drawNode(m_map_image, m_map_info, gps_datum, 2, 0, cv::Vec3b(0, 255, 0));
        m_gps_history_asen.push_back(gps_datum);
        printf("[GPS] lat=%lf, lon=%lf, ts=%lf\n", gps_datum.lat, gps_datum.lon, gps_time);

        // video capture
        cv::Mat video_image;
        while (video_time <= gps_time)
        {
            video_data >> video_image;
            if (video_image.empty()) break;
            video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
        }
        m_cam_mutex.lock();
        m_cam_image = video_image;
        m_cam_capture_time = video_time;
        m_cam_gps = gps_datum;
        m_cam_fnumber++;
        m_cam_mutex.unlock();

        // process vision modules
        if(m_enable_roadtheta) procRoadTheta();
        if(m_enable_vps) procVps();
        if(m_enable_logo) procLogo();
        if (m_enable_ocr) procOcr();
        if(m_enable_intersection) procIntersectionClassifier();

        // process Guidance
        procGuidance(gps_time);

        // draw GUI display
        cv::Mat gui_image = m_map_image.clone();
        drawGuiDisplay(gui_image);

        // recording
        if (m_recording) m_video_gui << gui_image;
        if (m_data_logging) m_video_cam << m_cam_image;

        cv::imshow(m_winname, gui_image);
        int key = cv::waitKey(1);
        if (key == cx::KEY_SPACE) key = cv::waitKey(0);
        if (key == cx::KEY_ESC) break;
        if (key == 83) itr += 30;   // Right Key

        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        printf("Iteration: %d (it took %lf seconds)\n", itr, t2 - t1);

        // update iteration
        itr++;

        // flush out logging data
        if (m_data_logging) m_log.flush();
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
        dg::LatLon ll = m_painter.cvtPixel2LatLon(cv::Point(x, y), m_map_info);
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
    m_localizer_mutex.lock();
    dg::LatLon pose_gps = m_localizer.getPoseGPS();
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    m_localizer_mutex.unlock();

    // generate & apply new path
    bool ok = updateDeepGuiderPath(pose_topo, pose_gps, gps_dest);
    if(!ok) return false;

    // update system status
    m_gps_dest = gps_dest;
    m_dest_defined = true;
    m_path_initialized = true;
    return true;
}


bool DeepGuider::updateDeepGuiderPath(dg::TopometricPose pose_topo, dg::LatLon gps_start, dg::LatLon gps_dest)
{
    // set start position to nearest node position
    m_map_mutex.lock();
    dg::Map& tmpmap = m_map_manager.getMap();    
    dg::LatLon pose_gps = gps_start;
    dg::Node* node = tmpmap.findNode(pose_topo.node_id);
    if(node)
    {
        pose_gps.lat = node->lat;
        pose_gps.lon = node->lon;
    }
    m_map_mutex.unlock();

    // generate path to destination
    dg::Path path;
    m_map_mutex.lock();
    bool ok = m_map_manager.getPath_expansion(pose_gps.lat, pose_gps.lon, gps_dest.lat, gps_dest.lon, path);
    m_map_mutex.unlock();
    path.start_pos = gps_start;
    path.dest_pos = gps_dest;
    if(!ok)
    {
        printf("[MapManager] fail to find path to (lat=%lf, lon=%lf)\n", gps_dest.lat, gps_dest.lon);
        return false;
    }
    dg::ID nid_start = path.pts.front().node_id;
    dg::ID nid_dest = path.pts.back().node_id;
    printf("[MapManager] New path generated! start=%zu, dest=%zu\n", nid_start, nid_dest);    

    // check if the generated path is valid on the map
    m_map_mutex.lock();
    dg::Map& map = m_map_manager.getMap();    
    dg::Node* node_start = map.findNode(nid_start);
    dg::Node* node_dest = map.findNode(nid_dest);
    m_map_mutex.unlock();
    VVS_CHECK_TRUE(node_start != nullptr);
    VVS_CHECK_TRUE(node_dest != nullptr);

    // localizer: set map to localizer
    m_localizer_mutex.lock();
    VVS_CHECK_TRUE(m_localizer.loadMap(map));
    m_localizer_mutex.unlock();
    printf("\tLocalizer is updated with new map!\n");

    // guidance: init map and path for guidance
    m_guider_mutex.lock();
    VVS_CHECK_TRUE(m_guider.initiateNewGuidance(path, map));
    m_guider_mutex.unlock();
    printf("\tGuidance is updated with new map and path!\n");

    // draw map
    m_map_image_original.copyTo(m_map_image);
    m_painter.drawMap(m_map_image, m_map_info, map);
    m_painter.drawPath(m_map_image, m_map_info, map, path);
    for(auto itr = m_gps_history_novatel.begin(); itr != m_gps_history_novatel.end(); itr++)
    {
        m_painter.drawNode(m_map_image, m_map_info, *itr, 2, 0, cv::Vec3b(0, 0, 255));
    }
    for(auto itr = m_gps_history_asen.begin(); itr != m_gps_history_asen.end(); itr++)
    {
        m_painter.drawNode(m_map_image, m_map_info, *itr, 2, 0, cv::Vec3b(0, 255, 0));
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
    cv::Point video_offset(20, 542);
    if (!cam_image.empty())
    {
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
            vector<dg::StreetView> sv = m_map_manager.getStreetView(sv_id);
            if(!sv.empty() && sv.front().id == sv_id)
            {
                m_painter.drawNode(image, m_map_info, dg::LatLon(sv.front().lat, sv.front().lon), 6, 0, cv::Vec3b(255, 255, 0));
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
    dg::Pose2 pose_metric = m_localizer.getPose();
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    dg::LatLon pose_gps = m_localizer.getPoseGPS();
    double pose_confidence = m_localizer.getPoseConfidence();
    m_localizer_mutex.unlock();

    // draw robot on the map
    m_painter.drawNode(image, m_map_info, pose_gps, 10, 0, cx::COLOR_YELLOW);
    m_painter.drawNode(image, m_map_info, pose_gps, 8, 0, cx::COLOR_BLUE);
    dg::Point2 pose_pixel = m_painter.cvtMeter2Pixel(pose_metric, m_map_info);
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
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    dg::Pose2 pose_metric = m_localizer.getPose();
    dg::LatLon pose_gps = m_localizer.getPoseGPS();
    double pose_confidence = m_localizer.getPoseConfidence();
    m_localizer_mutex.unlock();

    // Guidance: generate navigation guidance
    dg::GuidanceManager::GuideStatus cur_status;
    dg::GuidanceManager::Guidance cur_guide;
    m_map_mutex.lock();
    dg::Node* node = m_map_manager.getMap().findNode(pose_topo.node_id);
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
        m_guider.applyPoseGPS(dg::LatLon(node->lat, node->lon));
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

    // check arrival
    if (cur_status == GuidanceManager::GuideStatus::GUIDE_ARRIVED)
    {
        printf("Arrived to destination!\n");
        if(m_enable_tts) putTTS("Arrived to destination!");
    }

    // check out of path
    //if (cur_status == GuidanceManager::GuideStatus::GUIDE_OOP_DETECT || cur_status == GuidanceManager::GuideStatus::GUIDE_OOP || cur_status == GuidanceManager::GuideStatus::GUIDE_LOST)
    if (cur_status == GuidanceManager::GuideStatus::GUIDE_OOP || cur_status == GuidanceManager::GuideStatus::GUIDE_LOST)
    {
        printf("GUIDANCE: out of path detected!\n");
        if(m_enable_tts) putTTS("Regenerate path!");
        VVS_CHECK_TRUE(updateDeepGuiderPath(pose_topo, pose_gps, m_gps_dest));
    }

    // check lost
    if (m_enable_exploration)
    {
        m_guider.makeLostValue(m_guider.m_prevconf, pose_confidence);
        m_active_nav.apply(m_cam_image, cur_guide, ts);
        if (cur_status == dg::GuidanceManager::GuideStatus::GUIDE_LOST)
        {
            std::vector<ExplorationGuidance> actions;
            GuidanceManager::GuideStatus status;
            m_active_nav.get(actions, status);
            for (int k = 0; k < actions.size(); k++)
            {
                printf("\t action %d: [%lf, %lf, %lf]\n", k, actions[k].theta1, actions[k].d, actions[k].theta2);
            }
        }
    }
}


bool DeepGuider::procIntersectionClassifier()
{
    dg::Timestamp ts_old = m_intersection_classifier.timestamp();
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

    if (!cam_image.empty() && m_intersection_classifier.apply(cam_image, capture_time))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_intersection_classifier.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        } 
        m_intersection_classifier.print();

        m_intersection_mutex.lock();
        m_intersection_classifier.get(m_intersection_result);
        m_intersection_image = cam_image;
        m_intersection_mutex.unlock();

        // apply the result to localizer
        // TBD...
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
        VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, capture_time, confs));

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

    if (!cam_image.empty() && m_ocr.apply(cam_image, capture_time))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_ocr.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        }
        m_ocr.print();

        std::vector<dg::ID> ids;
        std::vector<Polar2> obs;
        std::vector<double> confs;
        std::vector<OCRResult> ocrs;
        m_ocr.get(ocrs);
        for (int k = 0; k < (int)ocrs.size(); k++)
        {
            //dg::ID ocr_id = m_map_manager.get_poi(ocrs[k].label);
            dg::ID ocr_id = 0;
            ids.push_back(ocr_id);
            obs.push_back(rel_pose_defualt);
            confs.push_back(ocrs[k].confidence);
        }
        VVS_CHECK_TRUE(m_localizer.applyLocClue(ids, obs, capture_time, confs));

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
    
    if (!cam_image.empty() && m_roadtheta.apply(cam_image, capture_time))
    {
        double angle, confidence;
        m_roadtheta.get(angle, confidence);
        VVS_CHECK_TRUE(m_localizer.applyLocClue(id_invalid, Polar2(-1, angle), capture_time, confidence));
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

    int N = 3;  // top-3
    double gps_accuracy = 1;   // 0: search radius = 230m ~ 1: search radius = 30m
    if (!cam_image.empty() && m_vps.apply(cam_image, N, capture_pos.lat, capture_pos.lon, gps_accuracy, capture_time, m_server_ip.c_str()))
    {
        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_vps.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        } 
        m_vps.print();

        std::vector<dg::ID> ids;
        std::vector<dg::Polar2> obs;
        std::vector<double> confs;
        std::vector<VPSResult> streetviews;
        m_vps.get(streetviews);
        for (int k = 0; k < (int)streetviews.size(); k++)
        {
            if (streetviews[k].id <= 0 || streetviews[k].confidence <= 0) continue;        // invalid data
            ids.push_back(streetviews[k].id);
            obs.push_back(rel_pose_defualt);
            confs.push_back(streetviews[k].confidence);
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
