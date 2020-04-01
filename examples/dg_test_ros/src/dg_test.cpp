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
    dg::Guidance m_guider;

    cv::Mutex webcam_mutex;
    cv::Mat webcam_image;
    dg::Timestamp webcam_time;
    dg::LatLon webcam_gps;
    std::thread* vps_thread = nullptr;
    std::thread* poi_thread = nullptr;
    static void threadfunc_vps(DeepGuider* guider);
    static void threadfunc_poi(DeepGuider* guider);
    bool enable_vps = false;
    bool enable_poi = false;
    bool is_vps_running = false;
    bool is_poi_running = false;
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
};

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
    m_roadTheta.clear();
    m_poi.clear();
    m_vps.clear();

    close_python_environment();
}

bool DeepGuider::initialize()
{
    printf("Initialize deepguider system...\n");

    // initialize python
    if (!init_python_environment("python3", "")) return false;
    printf("\tPython environment initialized!\n");

    // initialize map manager
    if (!m_map_manager.initialize()) return false;
    printf("\tMapManager initialized!\n");

    // initialize localizer
    //if (!m_localizer.initialize()) return false;
    printf("\tLocalizer initialized!\n");

    // initialize guidance
    //if (!m_guidance.initialize()) return false;
    printf("\tGuidance initialized!\n");

    // initialize roadTheta
    if (!m_roadTheta.initialize("road_direction_recognizer", "/work/deepguider/src/road_recog")) return false;
    printf("\tRoadTheta initialized!\n");

    // initialize POI
    //if (enable_poi && !m_poi.initialize()) return false;
    printf("\tPOI initialized!\n");

    // initialize VPS
    if (enable_vps && !m_vps.initialize("vps", "/work/deepguider/src/vps")) return false;
    printf("\tVPS initialized!\n");

    return true;
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
        strftime(szfilename, 255, "dg_test_%y%m%d_%H%M%S.avi", &_tm);
        std::string filename = szfilename;
        video.open(filename, 30);
    }

    // start & goal position
    dg::LatLon gps_start(36.38205717, 127.3676462);
    dg::LatLon gps_dest(36.37944417, 127.3788568);
    printf("\tgps_start: lat=%lf, lon=%lf\n", gps_start.lat, gps_start.lon);
    printf("\tgps_dest: lat=%lf, lon=%lf\n", gps_dest.lat, gps_dest.lon);

    // generate path to the destination
    dg::Path path = m_map_manager.getPath(gps_start.lat, gps_start.lon, gps_dest.lat, gps_dest.lon);
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

    const char* background_file = "/work/deepguider/bin/data/NaverMap_ETRI(Satellite)_191127.png";
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

    // guidance: load files for guidance test (ask JSH)
    VVS_CHECK_TRUE(m_guider.setPathNMap(path, map));
    VVS_CHECK_TRUE(m_guider.initializeGuides());

    // guidance: Initial move (ask JSH)
    dg::Guidance::MoveStatus cur_status;
    std::vector<dg::Guidance::RobotGuide> cur_guide;
    cur_guide = m_guider.getInitGuide();

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

    // run recognizer threads
    if (enable_vps) vps_thread = new std::thread(threadfunc_vps, this);
    if (enable_poi) poi_thread = new std::thread(threadfunc_poi, this);

    ros::Rate loop(1 / m_wait_sec);
    while (ros::ok())
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec())) break;
        ros::spinOnce();
        loop.sleep();
    }

    // terminate recoginizer threads
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
    if (!webcam_image.empty())
    {
        double video_resize_scale = 0.4;
        cv::Point video_offset(32, 542);
        cv::resize(webcam_image, video_image, cv::Size(), video_resize_scale, video_resize_scale);
        cv::Rect rect(video_offset, video_offset + cv::Point(video_image.cols, video_image.rows));
        if (rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = video_image * 1;
    }

    // draw localization info
    if (pose_initialized)
    {
        // draw robot on the map        
        dg::TopometricPose pose_topo = m_localizer.getPoseTopometric();
        dg::LatLon pose_gps = m_localizer.getPoseGPS();
        dg::Pose2 pose_m = m_localizer.toTopmetric2Metric(pose_topo);

        painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 10, 0, cx::COLOR_YELLOW);
        painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 8, 0, cx::COLOR_BLUE);

        // draw status message (localization)
        cv::String info_topo = cv::format("Node: %zu, Edge: %d, D: %.3f (Lat: %.6f, Lon: %.6f)", pose_topo.node_id, pose_topo.edge_idx, pose_topo.dist, pose_gps.lat, pose_gps.lon);
        cv::putText(image, info_topo, cv::Point(5, 30), cv::FONT_HERSHEY_PLAIN, 1.9, cv::Scalar(0, 200, 0), 2);
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
        if (!webcam_image.empty() && guider->m_vps.apply(webcam_image, N, webcam_gps.lat, webcam_gps.lon, gps_accuracy, webcam_time))
        {
            std::vector<dg::ID> ids;
            std::vector<dg::Polar2> obs;
            std::vector<double> confs;
            std::vector<VPSResult> streetviews;
            guider->m_vps.get(streetviews);
            printf("[VPS]\n");
            for (int k = 0; k < (int)streetviews.size(); k++)
            {
                ids.push_back(streetviews[k].id);
                obs.push_back(rel_pose_defualt);
                confs.push_back(streetviews[k].confidence);
                printf("\ttop%d: id=%zu, confidence=%lf, ts=%lf\n", k, streetviews[k].id, streetviews[k].confidence, webcam_time);
            }
            VVS_CHECK_TRUE(guider->m_localizer.applyLocClue(ids, obs, webcam_time, confs));
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
        dg::LatLon webcam_gps = guider->webcam_gps;
        guider->webcam_mutex.unlock();

        if (!webcam_image.empty() && guider->m_poi.apply(webcam_image, webcam_time))
        {
            //TODO
        }
    }
    guider->is_poi_running = false;
}

void DeepGuider::terminateThreadFunctions()
{
    if (vps_thread == nullptr && poi_thread == nullptr) return;

    // disable all thread running
    enable_vps = false;
    enable_poi = false;

    // wait up to 4000 ms at maximum
    for (int i = 0; i<40; i++)
    {
        if (!is_vps_running && !is_poi_running) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // wait child thread to terminate
    if (is_vps_running) vps_thread->join();
    if (is_poi_running) poi_thread->join();

    // clear threads
    if (vps_thread) delete vps_thread;
    if (poi_thread) delete poi_thread;
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
