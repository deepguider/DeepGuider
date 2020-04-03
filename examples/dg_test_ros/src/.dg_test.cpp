#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#define VVS_NO_ASSERT

#include "dg_core.hpp"
#include "dg_map_manager.hpp"
#include "dg_localizer.hpp"
#include "dg_road_recog.hpp"
#include "dg_poi_recog.hpp"
#include "dg_vps.hpp"
#include "dg_guidance.hpp"
#include "dg_utils.hpp"
#include <chrono>

using namespace dg;
using namespace std;

class DeepGuider
{
public:
    DeepGuider(ros::NodeHandle& nh);
    virtual ~DeepGuider();

    bool initialize();
    int run();
    bool runOnce(double timestamp);

protected:
    // Parameters
    double m_wait_sec;

    // Topic subscribers
    ros::Subscriber sub_image_webcam;
    ros::Subscriber sub_image_realsense_image;
    ros::Subscriber sub_image_realsense_depth;
    ros::Subscriber sub_gps_asen;
    ros::Subscriber sub_gps_novatel;
    ros::Subscriber sub_imu_xsense;

    // Subscriber callbacks
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg);

    // Topic publishers
    ros::Publisher pub_image_gui;

    // A node handler
    ros::NodeHandle& nh_dg;

    // DeepGuider modudles
    dg::RoadDirectionRecognizer m_roadTheta;
    dg::POIRecognizer m_poi;
    dg::VPS m_vps;
    dg::SimpleLocalizer m_localizer;
    dg::MapManager m_map_manager;
    dg::GuidanceManager m_guider;

    cv::Mutex webcam_mutex;
    cv::Mat webcam_image;
    dg::Timestamp webcam_time;
    dg::LatLon webcam_gps;

    cv::Mutex vps_mutex;
    cv::Mat vps_sv_image;
    dg::ID vps_sv_id;
    double vps_sv_confidence;

    cv::Mutex localizer_mutex;

    std::thread* vps_thread = nullptr;
    std::thread* poi_thread = nullptr;
    std::thread* roadTheta_thread = nullptr;
    static void threadfunc_vps(DeepGuider* guider);
    static void threadfunc_poi(DeepGuider* guider);
    static void threadfunc_roadTheta(DeepGuider* guider);    
    bool enable_vps = true;
    bool enable_poi = false;
    bool enable_roadTheta = false;
    bool is_vps_running = false;
    bool is_poi_running = false;
    bool is_roadTheta_running = false;
    void terminateThreadFunctions();

    // System variables
    int gps_update_cnt = 0;
    bool pose_initialized = false;
    bool path_initialized = false;

    // GUI support
    bool recording = false;
    cx::VideoWriter video;
    dg::SimpleRoadPainter painter;
    cv::Mat map_image;
    dg::RoadMap road_map;
    dg::CanvasInfo map_info;

    static std::string map_server_ip;

    // guidance icons
    cv::Mat icon_forward;
    cv::Mat mask_forward;
    cv::Mat icon_turn_left;
    cv::Mat mask_turn_left;
    cv::Mat icon_turn_right;
    cv::Mat mask_turn_right;
    cv::Mat icon_turn_back;
    cv::Mat mask_turn_back;
    void drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect);
};

std::string DeepGuider::map_server_ip = "129.254.87.96";    // default: 127.0.0.1 (localhost)


DeepGuider::DeepGuider(ros::NodeHandle& nh) : nh_dg(nh), m_wait_sec(0.1)
{
    // Read parameters
    nh_dg.param<double>("wait_sec", m_wait_sec, m_wait_sec);

    // Initialize subscribers
    sub_image_webcam = nh_dg.subscribe("/uvc_image_raw/compressed", 1, &DeepGuider::callbackImageCompressed, this);
    sub_gps_asen = nh_dg.subscribe("/asen_fix", 1, &DeepGuider::callbackGPSAsen, this);
    sub_gps_novatel = nh_dg.subscribe("/novatel_fix", 1, &DeepGuider::callbackGPSNovatel, this);
    sub_imu_xsense = nh_dg.subscribe("/imu/data", 1, &DeepGuider::callbackIMU, this);
    sub_image_realsense_image = nh_dg.subscribe("/camera/color/image_raw/compressed", 1, &DeepGuider::callbackRealsenseImage, this);
    sub_image_realsense_depth = nh_dg.subscribe("/camera/depth/image_rect_raw/compressed", 1, &DeepGuider::callbackRealsenseDepth, this);

    // Initialize publishers
    pub_image_gui = nh_dg.advertise<sensor_msgs::CompressedImage>("dg_image_gui", 1, true);
}

DeepGuider::~DeepGuider()
{
    if(enable_vps) m_vps.clear();
    if(enable_poi) m_poi.clear();
    if(enable_roadTheta) m_roadTheta.clear();

    close_python_environment();
}

bool DeepGuider::initialize()
{
    printf("Initialize deepguider system...\n");

    // initialize python
    bool threaded_run_python = true;
    if (!init_python_environment("python3", "", threaded_run_python)) return false;
    printf("\tPython environment initialized!\n");

    // initialize map manager
    m_map_manager.setIP(map_server_ip);
    if (!m_map_manager.initialize()) return false;
    printf("\tMapManager initialized!\n");

    // initialize localizer
    //if (!m_localizer.initialize()) return false;
    printf("\tLocalizer initialized!\n");

    // initialize guidance
    //if (!m_guidance.initialize()) return false;
    printf("\tGuidance initialized!\n");

    // initialize VPS
    if (enable_vps && !m_vps.initialize("vps", "/work/deepguider/src/vps")) return false;
    if (enable_vps) printf("\tVPS initialized!\n");

    // initialize POI
    if (enable_poi && !m_poi.initialize("poi_recognizer", "/work/deepguider/src/poi_recog")) return false;
    if (enable_poi) printf("\tPOI initialized!\n");

    // initialize roadTheta
    if (enable_roadTheta && !m_roadTheta.initialize("road_direction_recognizer", "/work/deepguider/src/road_recog")) return false;
    if (enable_roadTheta) printf("\tRoadTheta initialized!\n");

    // load icon images
    icon_forward = cv::imread("data/forward.png");
    icon_turn_left = cv::imread("data/turn_left.png");
    icon_turn_right = cv::imread("data/turn_right.png");
    icon_turn_back = cv::imread("data/turn_back.png");
    cv::threshold(icon_forward, mask_forward, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(icon_turn_left, mask_turn_left, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(icon_turn_right, mask_turn_right, 250, 1, cv::THRESH_BINARY_INV);
    cv::threshold(icon_turn_back, mask_turn_back, 250, 1, cv::THRESH_BINARY_INV);

    return true;
}


void DeepGuider::drawGuidance(cv::Mat image, dg::GuidanceManager::Guidance guide, cv::Rect rect)
{
    int guide_cx = rect.x + rect.width / 2;
    int guide_cy = rect.y + icon_forward.rows / 2 + 40;
    cv::Point center_pos(guide_cx, guide_cy);

    std::string dir_msg;
    dg::GuidanceManager::Motion cmd = guide.actions[0].cmd;
    if (cmd == dg::GuidanceManager::Motion::GO_FORWARD || cmd == dg::GuidanceManager::Motion::CROSS_FORWARD || cmd == dg::GuidanceManager::Motion::ENTER_FORWARD || cmd == dg::GuidanceManager::Motion::EXIT_FORWARD)
    {
        cv::Mat& icon = icon_forward;
        cv::Mat& mask = mask_forward;
        int x1 = center_pos.x - icon.cols / 2;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        if (cmd == dg::GuidanceManager::Motion::GO_FORWARD) dir_msg = "[Guide] GO_FORWARD";
        if (cmd == dg::GuidanceManager::Motion::CROSS_FORWARD) dir_msg = "[Guide] CROSS_FORWARD";
        if (cmd == dg::GuidanceManager::Motion::ENTER_FORWARD) dir_msg = "[Guide] ENTER_FORWARD";
        if (cmd == dg::GuidanceManager::Motion::EXIT_FORWARD) dir_msg = "[Guide] EXIT_FORWARD";
    }
    if (cmd == dg::GuidanceManager::Motion::TURN_LEFT || cmd == dg::GuidanceManager::Motion::CROSS_LEFT || cmd == dg::GuidanceManager::Motion::ENTER_LEFT || cmd == dg::GuidanceManager::Motion::EXIT_LEFT)
    {
        cv::Mat& icon = icon_turn_left;
        cv::Mat& mask = mask_turn_left;
        int x1 = center_pos.x - icon.cols + icon.cols / 6;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        if (cmd == dg::GuidanceManager::Motion::TURN_LEFT) dir_msg = "[Guide] TURN_LEFT";
        if (cmd == dg::GuidanceManager::Motion::CROSS_LEFT) dir_msg = "[Guide] CROSS_LEFT";
        if (cmd == dg::GuidanceManager::Motion::ENTER_LEFT) dir_msg = "[Guide] ENTER_LEFT";
        if (cmd == dg::GuidanceManager::Motion::EXIT_LEFT) dir_msg = "[Guide] EXIT_LEFT";
    }
    if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT || cmd == dg::GuidanceManager::Motion::CROSS_RIGHT || cmd == dg::GuidanceManager::Motion::ENTER_RIGHT || cmd == dg::GuidanceManager::Motion::EXIT_RIGHT)
    {
        cv::Mat& icon = icon_turn_right;
        cv::Mat& mask = mask_turn_right;
        int x1 = center_pos.x - icon.cols / 6;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT) dir_msg = "[Guide] TURN_RIGHT";
        if (cmd == dg::GuidanceManager::Motion::CROSS_RIGHT) dir_msg = "[Guide] CROSS_RIGHT";
        if (cmd == dg::GuidanceManager::Motion::ENTER_RIGHT) dir_msg = "[Guide] ENTER_RIGHT";
        if (cmd == dg::GuidanceManager::Motion::EXIT_RIGHT) dir_msg = "[Guide] EXIT_RIGHT";
    }
    if (cmd == dg::GuidanceManager::Motion::TURN_BACK)
    {
        cv::Mat& icon = icon_turn_back;
        cv::Mat& mask = mask_turn_back;
        int x1 = center_pos.x - icon.cols / 2;
        int y1 = center_pos.y - icon.rows / 2;
        cv::Rect rect(x1, y1, icon.cols, icon.rows);
        if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) icon.copyTo(image(rect), mask);
        dir_msg = "[Guide] TURN_BACK";
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


int DeepGuider::run()
{
    printf("Run deepguider system...\n");

    //cx::VideoWriter video;
    if (recording)
    {
        time_t start_t;
        time(&start_t);
        tm _tm = *localtime(&start_t);
        char szfilename[255];
        strftime(szfilename, 255, "dg_ros_%y%m%d_%H%M%S.avi", &_tm);
        std::string filename = szfilename;
        video.open(filename, 30);
    }

    // start & goal position
    dg::LatLon gps_start(36.38205717, 127.3676462);
    dg::LatLon gps_dest(36.37944417, 127.3788568);
    printf("\tgps_start: lat=%lf, lon=%lf\n", gps_start.lat, gps_start.lon);
    printf("\tgps_dest: lat=%lf, lon=%lf\n", gps_dest.lat, gps_dest.lon);

    // generate path to the destination
    dg::Path path;
    VVS_CHECK_TRUE(m_map_manager.getPath(gps_start.lat, gps_start.lon, gps_dest.lat, gps_dest.lon, path));
    dg::ID nid_start = path.pts.front().node->id;
    dg::ID nid_dest = path.pts.back().node->id;
    printf("\tPath generated! start=%zu, dest=%zu\n", nid_start, nid_dest);

    // fix lat,lon error in the download map (temporal fix -> it should be corrected at the map server)
    dg::Map& map = m_map_manager.getMap();
    for (auto itr = map.nodes.begin(); itr != map.nodes.end(); itr++)
    {
        if (itr->lat > itr->lon) { double l = itr->lat; itr->lat = itr->lon; itr->lon = l; }
    }

    // check consistency between map and path
    for (int idx = 0; idx < (int)path.pts.size(); idx++)
    {
        if (path.pts[idx].node)
        {
            dg::ID node_id = path.pts[idx].node->id;
            dg::Node* node = map.findNode(node_id);
            printf("\tpath[%d]: node_id=%zu, id=%zu, lat=%lf, lon=%lf, type=%d\n", idx, node->id, node_id, node->lat, node->lon, node->type);
        }
        if (path.pts[idx].edge)
        {
            dg::ID edge_id = path.pts[idx].edge->id;
            dg::ID from = path.pts[idx].edge->node1->id;
            dg::ID to = path.pts[idx].edge->node2->id;
            dg::Edge* edge = map.findEdge(from, to);
            printf("\tpath[%d]: edge_id=%zu, id=%zu, length=%lf, type=%d\n", idx, edge->id, edge_id, edge->length, edge->type);
        }
    }

    // download streetview map
    double lat_center = (path.pts.front().node->lat + path.pts.back().node->lat) / 2;
    double lon_center = (path.pts.front().node->lon + path.pts.back().node->lon) / 2;
    double radius = 1000;
    std::list<StreetView> sv_list;
    m_map_manager.getStreetView(lat_center, lon_center, radius, sv_list);
    printf("\tStreetView images are downloaded! nviews = %d\n", (int)map.views.size());

    // set map to localizer
    dg::LatLon ref_node(36.383837659737, 127.367880828442);
    VVS_CHECK_TRUE(m_localizer.setReference(ref_node));
    VVS_CHECK_TRUE(m_localizer.loadMap(map));

    // prepare GUI map
    //dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1.045));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 4));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 0));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 50, 255 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 200, 100, 100 }));
    //VVS_CHECK_TRUE(painter.setParamValue("edge_thickness", 2));

    const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png";
    map_image = cv::imread(background_file);
    VVS_CHECK_TRUE(!map_image.empty());
    road_map = m_localizer.getMap();
    VVS_CHECK_TRUE(painter.drawMap(map_image, road_map));
    map_info = painter.getCanvasInfo(road_map, map_image.size());

    dg::DirectedGraph<dg::Point2ID, double>::Node* node_prev = nullptr;
    for (int idx = 0; idx < (int)path.pts.size(); idx++)
    {
        dg::ID node_id = path.pts[idx].node->id;
        dg::DirectedGraph<dg::Point2ID, double>::Node* node = road_map.findNode(node_id);
        if (node){
            if (node_prev) painter.drawEdge(map_image, map_info, node_prev->data, node->data, 0, cv::Vec3b(200, 0, 0), 2);
            if (node_prev) painter.drawNode(map_image, map_info, node_prev->data, 5, 0, cv::Vec3b(50, 0, 255));
            painter.drawNode(map_image, map_info, node->data, 5, 0, cv::Vec3b(50, 0, 255));
            node_prev = node;
        }
    }
    printf("\tGUI map initialized!\n");
    cv::namedWindow("deep_guider");

    // localizer: set initial pose of localizer
    dg::ID id_invalid = 0;
    Polar2 rel_pose_defualt(-1, CV_PI);     // default relative pose (invalid)
    double confidence_default = 1.0;
    double timestamp = ros::Time::now().toSec();
    m_localizer.applyLocClue(nid_start, rel_pose_defualt, timestamp, confidence_default);

    // initial pose of the system
    dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
    dg::Pose2 pose_metric = m_localizer.getPose();
    dg::LatLon pose_gps = m_localizer.getPoseGPS();
    double pose_confidence = m_localizer.getPoseConfidence();

    // guidance: init map and path for guidance
    VVS_CHECK_TRUE(m_guider.setPathNMap(path, map));
    VVS_CHECK_TRUE(m_guider.initializeGuides());

    dg::GuidanceManager::MoveStatus cur_status;
    dg::GuidanceManager::Guidance cur_guide;
    cur_status = m_guider.applyPose(pose_topo);

    // run recognizer threads
    if (enable_vps) vps_thread = new std::thread(threadfunc_vps, this);
    if (enable_poi) poi_thread = new std::thread(threadfunc_poi, this);
    if (enable_roadTheta) roadTheta_thread = new std::thread(threadfunc_roadTheta, this);

    ros::Rate loop(1 / m_wait_sec);
    while (ros::ok())
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec())) break;
        ros::spinOnce();
        loop.sleep();
    }
    if(recording) video.release();

    terminateThreadFunctions();

    printf("End deepguider system...\n");

    return 0;
}

bool DeepGuider::runOnce(double timestamp)
{
    //cout << "Time: " << timestamp << endl;

    // draw video image as subwindow on the map
    cv::Mat image = map_image.clone();
    cv::Mat video_image;
    cv::Rect video_rect;
    double video_resize_scale = 0.4;
    cv::Point video_offset(32, 542);
    if (!webcam_image.empty())
    {
        cv::resize(webcam_image, video_image, cv::Size(), video_resize_scale, video_resize_scale);
        video_rect = cv::Rect(video_offset, video_offset + cv::Point(video_image.cols, video_image.rows));
        if (video_rect.br().x < image.cols && video_rect.br().y < image.rows) image(video_rect) = video_image * 1;
    }

    // draw vps result
    if (enable_vps)
    {
        // draw top-1 matching image
        double fy = (double)video_rect.height / vps_sv_image.rows;
        cv::Mat sv_image;
        dg::ID sv_id = 0;
        double sv_confidence = 0;
        vps_mutex.lock();
        if(!vps_sv_image.empty())
        {
            cv::resize(vps_sv_image, sv_image, cv::Size(), fy, fy);
            sv_id = vps_sv_id;
            sv_confidence = vps_sv_confidence;
        }
        vps_mutex.unlock();

        if (!sv_image.empty())
        {
            double fy = (double)video_rect.height / sv_image.rows;
            cv::resize(sv_image, sv_image, cv::Size(), fy, fy);
            cv::Point sv_offset = video_offset;
            sv_offset.x = video_rect.x + video_rect.width + 20;
            cv::Rect rect(sv_offset, sv_offset + cv::Point(sv_image.cols, sv_image.rows));
            if (rect.x >= 0 && rect.y >= 0 && rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = sv_image * 1;

            cv::Point msg_offset = sv_offset + cv::Point(10, 30);
            double font_scale = 0.8;
            std::string str_confidence = cv::format("Confidence: %.2lf", sv_confidence);
            cv::putText(image, str_confidence.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), 5);
            cv::putText(image, str_confidence.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), 2);
            std::string str_id = cv::format("ID: %zu", sv_id);
            msg_offset.y += 30;
            cv::putText(image, str_id.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), 5);
            cv::putText(image, str_id.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), 2);
        }

        // show gps position of top-1 matched image on the map
        dg::StreetView sv;
        if (sv_id > 0 && m_map_manager.getStreetView(sv_id, sv))
        {
            dg::Point2 sv_pos = m_localizer.toMetric(dg::LatLon(sv.lat, sv.lon));
            painter.drawNode(image, map_info, dg::Point2ID(0, sv_pos.x, sv_pos.y), 6, 0, cv::Vec3b(255, 255, 0));
        }
    }

    // draw localization & guidance info
    if (pose_initialized)
    {
        // draw robot on the map
        localizer_mutex.lock();
        dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
        dg::LatLon pose_gps = m_localizer.getPoseGPS();
        dg::Pose2 pose_metric = m_localizer.toTopmetric2Metric(pose_topo);
        double pose_confidence = m_localizer.getPoseConfidence();
        localizer_mutex.unlock();

        painter.drawNode(image, map_info, dg::Point2ID(0, pose_metric.x, pose_metric.y), 10, 0, cx::COLOR_YELLOW);
        painter.drawNode(image, map_info, dg::Point2ID(0, pose_metric.x, pose_metric.y), 8, 0, cx::COLOR_BLUE);

        // draw status message (localization)
        cv::String info_topo = cv::format("Node: %zu, Edge: %d, D: %.3fm", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist);
        cv::putText(image, info_topo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
        cv::putText(image, info_topo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
        std::string info_confidence = cv::format("Confidence: %.2lf", pose_confidence);
        cv::putText(image, info_confidence, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 5);
        cv::putText(image, info_confidence, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

        printf("[Localizer]\n");
        printf("\ttopo: node=%zu, edge=%d, dist=%lf\n", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist);
        printf("\tmetr: x=%lf, y=%lf, theta=%lf\n", pose_metric.x, pose_metric.y, pose_metric.theta);
        printf("\tgps : lat=%lf, lon=%lf\n", pose_gps.lat, pose_gps.lon);
        printf("\tconfidence: %lf\n", pose_confidence);

        // Guidance: generate navigation guidance
        dg::GuidanceManager::MoveStatus cur_status;
        dg::GuidanceManager::Guidance cur_guide;
        cur_status = m_guider.applyPose(pose_topo);
        cur_guide = m_guider.getGuidance(cur_status);
        printf("%s\n", cur_guide.msg.c_str());

        // draw guidance output on the video image
        if (!video_image.empty())
        {
            drawGuidance(image, cur_guide, video_rect);
        }
    }

    // recording
    if (recording) video << image;

    cv::imshow("deep_guider", image);
    int key = cv::waitKey(1);
    //if (key == cx::KEY_SPACE) key = cv::waitKey(0);
    if (key == cx::KEY_ESC) return false;

    return true;
}

// Thread fnuction for VPS
void DeepGuider::threadfunc_vps(DeepGuider* guider)
{
    guider->is_vps_running = true;
    while (guider->enable_vps)
    {
        guider->webcam_mutex.lock();
        cv::Mat webcam_image = guider->webcam_image.clone();
        dg::Timestamp webcam_time = guider->webcam_time;
        dg::LatLon webcam_gps = guider->webcam_gps;
        guider->webcam_mutex.unlock();

        int N = 3;  // top-3
        double gps_accuracy = 1;   // 0: search radius = 230m ~ 1: search radius = 30m
        dg::Polar2 rel_pose_defualt(-1, CV_PI);     // default relative pose (invalid)
        if (!webcam_image.empty() && guider->m_vps.apply(webcam_image, N, webcam_gps.lat, webcam_gps.lon, gps_accuracy, webcam_time, map_server_ip.c_str()))
        {
            std::vector<dg::ID> ids;
            std::vector<dg::Polar2> obs;
            std::vector<double> confs;
            std::vector<VPSResult> streetviews;
            guider->m_vps.get(streetviews);
            printf("[VPS]\n");
            for (int k = 0; k < (int)streetviews.size(); k++)
            {
                if (streetviews[k].id <= 0 || streetviews[k].confidence <= 0) continue;        // invalid data
                ids.push_back(streetviews[k].id);
                obs.push_back(rel_pose_defualt);
                confs.push_back(streetviews[k].confidence);
                printf("\ttop%d: id=%zu, confidence=%lf, ts=%lf\n", k, streetviews[k].id, streetviews[k].confidence, webcam_time);
            }

            if(ids.size() > 0)
            {
                guider->localizer_mutex.lock();
                VVS_CHECK_TRUE(guider->m_localizer.applyLocClue(ids, obs, webcam_time, confs));
                guider->localizer_mutex.unlock();

                cv::Mat sv_image;
                if(guider->m_map_manager.getStreetViewImage(ids[0], sv_image, "f") && !sv_image.empty())
                {
                    guider->vps_mutex.lock();
                    guider->vps_sv_image = sv_image;
                    guider->vps_sv_id = ids[0];
                    guider->vps_sv_confidence = confs[0];
                    guider->vps_mutex.unlock();
                }
                else
                {
                    guider->vps_mutex.lock();
                    guider->vps_sv_image = cv::Mat();
                    guider->vps_sv_id = ids[0];
                    guider->vps_sv_confidence = confs[0];
                    guider->vps_mutex.unlock();
                }
            }
            else
            {
                guider->vps_mutex.lock();
                guider->vps_sv_image = cv::Mat();
                guider->vps_sv_id = 0;
                guider->vps_sv_confidence = 0;
                guider->vps_mutex.unlock();
            }
        }
    }
    guider->is_vps_running = false;
}

// Thread fnuction for POI
void DeepGuider::threadfunc_poi(DeepGuider* guider)
{
    guider->is_poi_running = true;
    while (guider->enable_poi)
    {
        guider->webcam_mutex.lock();
        cv::Mat webcam_image = guider->webcam_image.clone();
        dg::Timestamp webcam_time = guider->webcam_time;
        guider->webcam_mutex.unlock();

        if (!webcam_image.empty() && guider->m_poi.apply(webcam_image, webcam_time))
        {
            //TODO
        }
    }
    guider->is_poi_running = false;
}

// Thread fnuction for RoadTheta
void DeepGuider::threadfunc_roadTheta(DeepGuider* guider)
{
    guider->is_roadTheta_running = true;
    while (guider->enable_roadTheta)
    {
        guider->webcam_mutex.lock();
        cv::Mat webcam_image = guider->webcam_image.clone();
        dg::Timestamp webcam_time = guider->webcam_time;
        dg::LatLon webcam_gps = guider->webcam_gps;
        guider->webcam_mutex.unlock();

        if (!webcam_image.empty() && guider->m_roadTheta.apply(webcam_image, webcam_time))
        {
            //TODO
        }
    }
    guider->is_roadTheta_running = false;
}


void DeepGuider::terminateThreadFunctions()
{
    if (vps_thread == nullptr && poi_thread == nullptr && roadTheta_thread == nullptr) return;

    // disable all thread running
    enable_vps = false;
    enable_poi = false;
    enable_roadTheta = false;

    // wait up to 4000 ms at maximum
    for (int i = 0; i<40; i++)
    {
        if (!is_vps_running && !is_poi_running && !is_roadTheta_running) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // wait child thread to terminate
    if (is_vps_running) vps_thread->join();
    if (is_poi_running) poi_thread->join();
    if (is_roadTheta_running) roadTheta_thread->join();

    // clear threads
    if (vps_thread) delete vps_thread;
    if (poi_thread) delete poi_thread;
    if (roadTheta_thread) delete roadTheta_thread;
    vps_thread = nullptr;
    poi_thread = nullptr;
    roadTheta_thread = nullptr;
}

// A callback function for subscribing a RGB image
void DeepGuider::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "RGB image (timestamp: %f [sec]).", msg->header.stamp.toSec());
    cv_bridge::CvImagePtr image_ptr;
    cv::Mat image;
    try
    {
        image_ptr = cv_bridge::toCvCopy(msg);
        cv::cvtColor(image_ptr->image, image, cv::COLOR_RGB2BGR);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception @ callbackImage(): %s", e.what());
        return;
    }
}

// A callback function for subscribing a compressed RGB image
void DeepGuider::callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "Compressed RGB(timestamp: %f [sec]).", msg->header.stamp.toSec());
    cv_bridge::CvImagePtr image_ptr;
    try
    {
        webcam_mutex.lock();
        webcam_image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
        webcam_time = msg->header.stamp.toSec();
        webcam_gps = m_localizer.getPoseGPS();
        webcam_mutex.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ callbackImageCompressed(): %s", e.what());
        return;
    }
}

// A callback function for Realsense compressed RGB
void DeepGuider::callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "Realsense: RGB (timestamp=%f)", msg->header.stamp.toSec());
    cv::Mat image;
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ callbackRealsenseImage(): %s", e.what());
        return;
    }
}

// A callback function for Realsense compressed Depth
void DeepGuider::callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "Realsense: Depth (timestamp=%f)", msg->header.stamp.toSec());
    cv::Mat image;
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ callbackRealsenseDepth(): %s", e.what());
        return;
    }
}

// A callback function for subscribing GPS Asen
void DeepGuider::callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix)
{
    //ROS_INFO_THROTTLE(1.0, "GPS Asen is subscribed (timestamp: %f [sec]).", fix->header.stamp.toSec());

    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_DEBUG_THROTTLE(60, "Asen: No fix.");
        return;
    }

    if (fix->header.stamp == ros::Time(0)) {
        return;
    }

    double lat = fix->latitude;
    double lon = fix->longitude;
    ROS_INFO_THROTTLE(1.0, "GPS Asen: lat=%f, lon=%f", lat, lon);

    // apply gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    m_localizer.applyGPS(gps_datum, gps_time);

    // draw gps history on the map
    dg::Point2 gps_pt = m_localizer.toMetric(gps_datum);
    painter.drawNode(map_image, map_info, dg::Point2ID(0, gps_pt), 2, 0, cv::Vec3b(0, 255, 0));

    // check pose initialization
    if (!pose_initialized)
    {
        gps_update_cnt++;
        if (gps_update_cnt > 10)
        {
            pose_initialized = true;
        }
    }
}

// A callback function for subscribing GPS Novatel
void DeepGuider::callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix)
{
    //ROS_INFO_THROTTLE(1.0, "GPS Novatel is subscribed (timestamp: %f [sec]).", fix->header.stamp.toSec());

    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_DEBUG_THROTTLE(60, "Novatel: No fix.");
        return;
    }

    if (fix->header.stamp == ros::Time(0)) {
        return;
    }

    double lat = fix->latitude;
    double lon = fix->longitude;
    ROS_INFO_THROTTLE(1.0, "GPS Novatel: lat=%f, lon=%f", lat, lon);

    // draw gps history on the map
    const dg::LatLon gps_datum(lat, lon);
    dg::Point2 gps_pt = m_localizer.toMetric(gps_datum);
    painter.drawNode(map_image, map_info, dg::Point2ID(0, gps_pt), 2, 0, cv::Vec3b(0, 0, 255));
}

// A callback function for subscribing IMU
void DeepGuider::callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
    int seq = msg->header.seq;
    double ori_x = msg->orientation.x;
    double ori_y = msg->orientation.y;
    double ori_z = msg->orientation.z;
    double angvel_x = msg->angular_velocity.x;
    double angvel_y = msg->angular_velocity.y;
    double angvel_z = msg->angular_velocity.z;
    double linacc_x = msg->linear_acceleration.x;
    double linacc_y = msg->linear_acceleration.y;
    double linacc_z = msg->linear_acceleration.z;

    ROS_INFO_THROTTLE(1.0, "IMU: seq=%d, orientation=(%f,%f,%f), angular_veloctiy=(%f,%f,%f), linear_acceleration=(%f,%f,%f)", seq, ori_x, ori_y, ori_z, angvel_x, angvel_y, angvel_z, linacc_x, linacc_y, linacc_z);
}

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_test");
    ros::NodeHandle nh("~");
    DeepGuider dg_node(nh);
    if (!dg_node.initialize()) return -1;
    dg_node.run();
    return 0;
}
