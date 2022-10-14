#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include "dg_simple.cpp"
#include "dg_simple_ros/ocr_info.h"
#include "dg_simple_ros/vps.h"
#include "dg_simple_ros/guidance.h"
#include "dg_simple_ros/action.h"
#include "dg_simple_ros/dg_status.h"
#include "utils/utm_converter.hpp"


class DeepGuiderROS : public DeepGuider
{
public:
    DeepGuiderROS(ros::NodeHandle& nh);
    virtual ~DeepGuiderROS();

    bool initialize(std::string config_file);
    int run();
    bool runOnce(double timestamp);

protected:
    virtual int readParam(const cv::FileNode& fn);
    int readRosParam(const cv::FileNode& fn);

    // Topic names
    std::string m_topic_360cam;
    std::string m_topic_360cam_crop;
    std::string m_topic_cam;
    std::string m_topic_gps;
    std::string m_topic_dgps;
    std::string m_topic_imu;
    std::string m_topic_odo;
    std::string m_topic_rgbd_image;
    std::string m_topic_rgbd_depth;

    // Topic subscribers (sensor data)
    ros::Subscriber sub_image_360cam;
    ros::Subscriber sub_image_360cam_crop;
    ros::Subscriber sub_image_webcam;
    ros::Subscriber sub_image_realsense_image;
    ros::Subscriber sub_image_realsense_depth;
    ros::Subscriber sub_gps_asen;
    ros::Subscriber sub_gps_novatel;
    ros::Subscriber sub_imu_xsense;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_robot_status;
    ros::Subscriber sub_robot_pose;
    ros::Subscriber sub_robot_heading;
    ros::Subscriber sub_robot_map;
    void callbackThetaZ1360Image(const sensor_msgs::Image::ConstPtr& msg);
    void callbackThetaZ1360Crop(const sensor_msgs::Image::ConstPtr& msg);
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg);
    void callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
    void callbackRobotStatus(const std_msgs::String::ConstPtr& msg);
    void callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callbackRobotHeading(const std_msgs::String::ConstPtr& msg);
    void callbackRobotMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // Topic publishers (sensor data)
    ros::Publisher pub_guide;
    ros::Publisher pub_path;
    ros::Publisher pub_pose;
    ros::Publisher pub_subgoal;
    ros::Publisher pub_status;
    void publishGuidance();
    void publishPath();
    void publishDGPose();
    void publishSubGoal();
    void publishDGStatus(bool system_shutdown = false);

    // Topic subscribers (sub modules)
    ros::Subscriber sub_ocr;
    ros::Subscriber sub_ocr_image;
    void callbackOCR(const dg_simple_ros::ocr_info::ConstPtr& msg);
    void callbackOCRImage(const sensor_msgs::Image::ConstPtr& msg);
    ros::Subscriber sub_vps;
    void callbackVPS(const dg_simple_ros::vps::ConstPtr& msg);

    // A node handler
    ros::NodeHandle& nh_dg;
    double m_update_hz;
    double m_timestamp_offset = -1;

    //Robot parameters
    std::string m_robotmap_path = "data/Bucheon/bucheon_220922/occumap_bucheon.png";
    dg::LatLon m_dx_map_ref_latlon = dg::LatLon(37.5177542, 126.7651744);
    dg::Point2 m_dx_map_ref_pixel = dg::Point2(715, 650);
    double m_dx_map_meter_per_pixel = 0.1;
    double m_dx_map_rotation_radian = -0.8;
    double m_robotmap_scale = 10.0;
    cv::Point2d m_dx_map_origin_pixel = cv::Point2d(345, 1110);
    bool m_pub_flag = false;
    GuidanceManager::RobotStatus m_prev_state = GuidanceManager::RobotStatus::READY;
    ros::Time m_begin_time = ros::Time::now();

    // DX 로봇 부천 지도 origin 계산
    dg::LatLon m_dx_map_origin_latlon;
    dg::LatLon m_dg_map_origin_latlon; // 딥가이더 부천 원점 (dg_simple.yml 참조)
    dg::UTMConverter m_dx_converter;
    dg::UTMConverter m_dg_converter;

    //robot related functions
    std::vector<Point2> m_undrivable_points;
    void initialize_DG_DX_conversion();
    Point2 makeSubgoal();
    Point2 makeImagePixels(cv::Mat& image, Point2& robot_dx_pixel, Point2& node_dx_pixel);
    geometry_msgs::PoseStamped makeRosPubPoseMsg(ID nid, Point2 xy);
    Point2 findDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px);
    Point2 findDrivableNearPoint(cv::Mat &image, Point2 robot_px, Point2 node_px);
    Point2 findDrivableNearPixel(cv::Mat &image, Point2 robot_px, Point2 node_px);
    Point2 reselectDrivablePoint();

};

DeepGuiderROS::DeepGuiderROS(ros::NodeHandle& nh) : nh_dg(nh)
{
    // set ros-specific defaults
    m_srcdir = "/home/dgtest/deepguider/src";      // absolute system path of deepguider/src (required for python embedding)
    m_threaded_run_modules = true;
    m_recording_header_name = "dg_ros_";
    m_update_hz = 10;
}

DeepGuiderROS::~DeepGuiderROS()
{    
}

int DeepGuiderROS::readParam(const cv::FileNode& fn)
{
    int n_read = DeepGuider::readParam(fn);
    n_read += readRosParam(fn);
    return n_read;
}

int DeepGuiderROS::readRosParam(const cv::FileNode& fn)
{
    int n_read = 0;

    // force to set threaded run
    m_threaded_run_modules = true;
    CX_LOAD_PARAM_COUNT(fn, "ros_update_hz", m_update_hz, n_read);

    // topic names configuration
    CX_LOAD_PARAM_COUNT(fn, "topic_360cam", m_topic_360cam, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_360cam_crop", m_topic_360cam_crop, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_cam", m_topic_cam, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_gps", m_topic_gps, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_odo", m_topic_odo, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_dgps", m_topic_dgps, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_imu", m_topic_imu, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_rgbd_image", m_topic_rgbd_image, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_rgbd_depth", m_topic_rgbd_depth, n_read);

    int topic_name_index = -1;
    std::string topicset_tagname;
    std::vector<cv::String> topic_names_set;
    CX_LOAD_PARAM_COUNT(fn, "topic_names_set", topic_names_set, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_name_index", topic_name_index, n_read);
    if (topic_name_index >= 0 && topic_name_index < topic_names_set.size()) topicset_tagname = topic_names_set[topic_name_index];

    // Read Topic Setting
    if (!topicset_tagname.empty())
    {
        cv::FileNode fn_topic = fn[topicset_tagname];
        if (!fn_topic.empty())
        {
            n_read += readRosParam(fn_topic);
        }
    }


    //read robot configuration
    CX_LOAD_PARAM_COUNT(fn, "robotmap_path", m_robotmap_path, n_read);
    cv::Vec2d dx_ref_point = cv::Vec2d(m_dx_map_ref_latlon.lat, m_dx_map_ref_latlon.lon);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_ref_point_latlon", dx_ref_point, n_read);
    m_dx_map_ref_latlon = dg::LatLon(dx_ref_point[0], dx_ref_point[1]);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_ref_point_pixel", m_dx_map_ref_pixel, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_ref_point_pixel_per_meter", m_dx_map_meter_per_pixel, n_read);
    double robotmap_rotation;
    CX_LOAD_PARAM_COUNT(fn, "robotmap_rotation", robotmap_rotation, n_read);
    m_dx_map_rotation_radian = cx::cvtDeg2Rad(robotmap_rotation); // 로봇 지도 rotation
    CX_LOAD_PARAM_COUNT(fn, "robotmap_scale", m_robotmap_scale, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_origin_pixel", m_dx_map_origin_pixel, n_read);
    cv::Vec2d dg_ref_point = cv::Vec2d(m_map_ref_point.lat, m_map_ref_point.lon);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_latlon", dg_ref_point, n_read);
    m_dg_map_origin_latlon = dg::LatLon(dg_ref_point[0], dg_ref_point[1]);

    int robot_map_index = -1;
    std::string robot_site_tagname;
    std::vector<cv::String> robot_map_set;
    CX_LOAD_PARAM_COUNT(fn, "robot_map_set", robot_map_set, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robot_map_index", robot_map_index, n_read);
    if (robot_map_index >= 0 && robot_map_index < robot_map_set.size())
        robot_site_tagname = robot_map_set[robot_map_index];

    // Read Robot Setting
    if (!robot_site_tagname.empty())
    {
        cv::FileNode fn_robot = fn[robot_site_tagname];
        if (!fn_robot.empty())
        {
            n_read += readRosParam(fn_robot);
        }
    }
    m_guider.setRobotMap(robot_site_tagname);
    m_guider.setRobotUsage(topicset_tagname);
    printf("topicset_tagname: %s\n", topicset_tagname.c_str());
    printf("m_dx_map_ref_pixel.x: %f, m_dx_map_ref_pixel.y: %f\n", m_dx_map_ref_pixel.x, m_dx_map_ref_pixel.y);
    printf("m_dx_map_origin_pixel: %f, %f\n", m_dx_map_origin_pixel.x, m_dx_map_origin_pixel.y);
    printf("m_dg_map_origin_latlon: %f, %f\n", m_dg_map_origin_latlon.lat, m_dg_map_origin_latlon.lon);
    printf("robotmap_rotation: %f\n", robotmap_rotation);

    return n_read;
}

bool DeepGuiderROS::initialize(std::string config_file)
{
    // Initialize main system
    bool ok = DeepGuider::initialize(config_file);
    if(!ok) return false;

    // Initialize sensor subscribers
    if(!m_topic_360cam.empty()) sub_image_360cam = nh_dg.subscribe(m_topic_360cam, 1, &DeepGuiderROS::callbackThetaZ1360Image, this);
    if(!m_topic_360cam_crop.empty()) sub_image_360cam_crop = nh_dg.subscribe(m_topic_360cam_crop, 1, &DeepGuiderROS::callbackThetaZ1360Crop, this);
    if(!m_topic_cam.empty()) sub_image_webcam = nh_dg.subscribe(m_topic_cam, 1, &DeepGuiderROS::callbackImageCompressed, this);
    if(!m_topic_gps.empty()) sub_gps_asen = nh_dg.subscribe(m_topic_gps, 1, &DeepGuiderROS::callbackGPSAsen, this);
    if(!m_topic_dgps.empty()) sub_gps_novatel = nh_dg.subscribe(m_topic_dgps, 1, &DeepGuiderROS::callbackGPSNovatel, this);
    if(!m_topic_imu.empty()) sub_imu_xsense = nh_dg.subscribe(m_topic_imu, 1, &DeepGuiderROS::callbackIMU, this);
    if(!m_topic_odo.empty()) sub_odometry = nh_dg.subscribe(m_topic_odo, 1, &DeepGuiderROS::callbackOdometry, this);
    if(!m_topic_rgbd_image.empty()) sub_image_realsense_image = nh_dg.subscribe(m_topic_rgbd_image, 1, &DeepGuiderROS::callbackRealsenseImage, this);
    if(!m_topic_rgbd_depth.empty()) sub_image_realsense_depth = nh_dg.subscribe(m_topic_rgbd_depth, 1, &DeepGuiderROS::callbackRealsenseDepth, this);

    // Initialize deepguider subscribers
    // sub_robot_status = nh_dg.subscribe("/keti_robot_state", 1, &DeepGuiderROS::callbackRobotStatus, this);
    sub_robot_status = nh_dg.subscribe("/keti_robot/state", 1, &DeepGuiderROS::callbackRobotStatus, this);
    sub_robot_heading = nh_dg.subscribe("/keti_robot/heading_node", 1, &DeepGuiderROS::callbackRobotHeading, this);
    sub_robot_pose = nh_dg.subscribe("/mcl3d/current/pose", 1, &DeepGuiderROS::callbackRobotPose, this);
    sub_robot_map = nh_dg.subscribe("/deepmerge/map/occu", 1, &DeepGuiderROS::callbackRobotMap, this);
    sub_ocr = nh_dg.subscribe("/dg_ocr/output", 1, &DeepGuiderROS::callbackOCR, this);
    sub_ocr_image = nh_dg.subscribe("/dg_ocr/image", 1, &DeepGuiderROS::callbackOCRImage, this);
	if(m_enable_vps == 2)sub_vps = nh_dg.subscribe("/dg_vps/output", 1, &DeepGuiderROS::callbackVPS, this);

    // Initialize deepguider publishers
    pub_guide = nh_dg.advertise<dg_simple_ros::guidance>("dg_guide", 1, true);
    pub_path = nh_dg.advertise<nav_msgs::Path>("dg_path", 1, true);
    pub_pose = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_pose", 1, true);
    pub_subgoal = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_subgoal", 1, true);
    pub_status = nh_dg.advertise<dg_simple_ros::dg_status>("dg_status", 1, true);

    // Initialize robot parameters
    initialize_DG_DX_conversion();

    return true;
}

int DeepGuiderROS::run()
{
    printf("Run deepguider system...\n");

    // start internal recognizer threads
    if (m_enable_intersection==1) intersection_thread = new std::thread(threadfunc_intersection, this);
    if (m_enable_ocr==1) ocr_thread = new std::thread(threadfunc_ocr, this);
    if (m_enable_vps==1) vps_thread = new std::thread(threadfunc_vps, this);
    if (m_enable_roadlr==1) roadlr_thread = new std::thread(threadfunc_roadlr, this);
    if (m_enable_roadtheta==1) roadtheta_thread = new std::thread(threadfunc_roadtheta, this);
    if (m_enable_exploration==1) exploration_thread = new std::thread(threadfunc_exploration, this);    
    if (m_enable_logo==1) logo_thread = new std::thread(threadfunc_logo, this);

    // run main loop
    ros::Rate loop(m_update_hz);
    while (ros::ok())
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec())) break;
        ros::spinOnce();
        loop.sleep();
    }

    // broadcast shutdown message
    publishDGStatus(true);
    ros::spinOnce();

    // shutdown system
    printf("Shutdown deepguider system...\n");
    terminateThreadFunctions();
    printf("\tthread terminated\n");
    if(m_video_recording) m_video_gui.release();
    if(m_video_recording) printf("\trecording closed\n");
    cv::destroyWindow(m_winname);
    printf("\tgui window destroyed\n");
    nh_dg.shutdown();
    printf("\tros shutdowned\n");
    printf("all done!\n");

    return 0;
}

bool DeepGuiderROS::runOnce(double timestamp)
{
    // process Guidance
    procGuidance(timestamp);
    
    // get guidance messages
    publishGuidance();
    publishPath();
    publishDGPose();
    publishSubGoal();
    publishDGStatus();

    // process path generation
    if(m_dest_defined && m_path_generation_pended && m_localizer->isPoseInitialized())
    {
        if(updateDeepGuiderPath(getPose(), m_dest)) m_path_generation_pended = false;        
    }
    
    // draw GUI display
    cv::Mat gui_image;
    dg::Pose2 px = m_painter.cvtValue2Pixel(getPose());
    if (m_gui_auto_scroll && m_localizer->isPoseInitialized()) m_viewport.centerizeViewportTo(px);

    m_viewport.getViewportImage(gui_image);
    drawGuiDisplay(gui_image, m_viewport.offset(), m_viewport.zoom());

    // recording
    if (m_video_recording) m_video_gui << gui_image;

    cv::imshow(m_winname, gui_image);
    int key = cv::waitKey(1);
    if (key == cx::KEY_SPACE) key = cv::waitKey(0);
    if (key == '1') m_viewport.setZoom(1);
    if (key == '2') m_viewport.setZoom(2);
    if (key == '3') m_viewport.setZoom(3);
    if (key == '4') m_viewport.setZoom(4);
    if (key == '0') m_viewport.setZoom(0.1);
    if (key == 'g' || key == 'G') m_apply_gps = !m_apply_gps;
    if (key == 'm' || key == 'M') m_apply_imu = !m_apply_imu;
    if (key == 'o' || key == 'O')
    {
        m_apply_odometry = !m_apply_odometry;
        if(m_enable_odometry && m_apply_odometry) m_localizer->resetOdometry();
        if(m_enable_odometry && !m_apply_odometry) m_localizer->resetOdometryActivated();
    }
    if (key == 'v' || key == 'V') m_apply_vps = !m_apply_vps;
    if (key == 'p' || key == 'P') m_apply_ocr = !m_apply_ocr;
    if (key == 'i' || key == 'I') m_apply_intersection = !m_apply_intersection;
    if (key == 'l' || key == 'L') m_apply_roadlr = !m_apply_roadlr;
    if (key == 't' || key == 'T') m_apply_roadtheta = !m_apply_roadtheta;    
    if (key == 'a') m_gui_auto_scroll = !m_gui_auto_scroll;  // toggle auto scroll of the map view
    if (key == 'k') m_show_ekf_pose = !m_show_ekf_pose;
    if (key == 'j') m_localizer->toggleEnablePathProjection();

    if (key == cx::KEY_ESC) return false;

    return true;
}

// A callback function for subscribing a RGB image
void DeepGuiderROS::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
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

// A callback function for subscribing a RicohThetaZ1 360 image
void DeepGuiderROS::callbackThetaZ1360Image(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "ThetaZ1 360 RGB image (timestamp: %f [sec]).", msg->header.stamp.toSec());
    try
    {
        m_360cam_mutex.lock();
    	cv_bridge::CvImagePtr image_ptr;
        image_ptr = cv_bridge::toCvCopy(msg);
		m_360cam_image = image_ptr->image.clone();
		//cv::cvtColor(image_ptr->image, m_360cam_image, cv::COLOR_RGB2BGR);
        m_360cam_capture_time = msg->header.stamp.toSec();
        m_360cam_mutex.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception @ callbackThetaZ1360Image(): %s", e.what());
        return;
    }
}

// A callback function for subscribing a RicohThetaZ1 360 image
void DeepGuiderROS::callbackThetaZ1360Crop(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "ThetaZ1 360 Crop image (timestamp: %f [sec]).", msg->header.stamp.toSec());
    try
    {
        m_360cam_crop_mutex.lock();
    	cv_bridge::CvImagePtr image_ptr;
        image_ptr = cv_bridge::toCvCopy(msg);
		m_360cam_crop_image = image_ptr->image.clone();
		//cv::cvtColor(image_ptr->image, m_360cam_crop_image, cv::COLOR_RGB2BGR);
        m_360cam_crop_mutex.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception @ callbackThetaZ1360Crop(): %s", e.what());
        return;
    }
}

// A callback function for subscribing a compressed RGB image
void DeepGuiderROS::callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "Compressed RGB");
    cv_bridge::CvImagePtr image_ptr; 
    try
    {
        cv::Mat logging_image;
        m_cam_mutex.lock();
        m_cam_image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
        m_cam_capture_time = msg->header.stamp.toSec();
        m_cam_mutex.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ callbackImageCompressed(): %s", e.what());
        return;
    }
}

// A callback function for Realsense compressed RGB
void DeepGuiderROS::callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg)
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
void DeepGuiderROS::callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg)
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
void DeepGuiderROS::callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix)
{
    double timestamp = ros::Time::now().toSec();
    if (fix->header.stamp == ros::Time(0)) return;

    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_INFO_THROTTLE(1.0, "GPS: no fix");
        return;
    }
    ROS_INFO_THROTTLE(1.0, "GPS: lat=%f, lon=%f", fix->latitude, fix->longitude);

    double lat = fix->latitude;
    double lon = fix->longitude;

    // apply & draw gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    if (!m_use_high_precision_gps) procGpsData(gps_datum, gps_time);
    m_painter.drawPoint(m_map_image, toMetric(gps_datum), m_gui_gps_trj_radius, m_gui_gps_color);
}

// A callback function for subscribing GPS Novatel
void DeepGuiderROS::callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix)
{
    if (fix->header.stamp == ros::Time(0)) return;

    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_INFO_THROTTLE(1.0, "GPS Novatel: no fix");
        return;
    }
    ROS_INFO_THROTTLE(1.0, "GPS Novatel: lat=%f, lon=%f", fix->latitude, fix->longitude);

    double lat = fix->latitude;
    double lon = fix->longitude;

    // apply & draw gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    if (m_use_high_precision_gps) procGpsData(gps_datum, gps_time);
    m_painter.drawPoint(m_map_image, toMetric(gps_datum), m_gui_gps_trj_radius, m_gui_gps_novatel_color);
}

// A callback function for subscribing Odometry
void DeepGuiderROS::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.stamp == ros::Time(0)) {
        return;
    }

    nav_msgs::Odometry odo;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = msg->pose.pose.orientation.z;
    ROS_INFO_THROTTLE(1.0, "ODO: x=%.2lf, y=%.2lf, theta=%.1lf", x, y, theta);

    dg::Timestamp odo_time = msg->header.stamp.toSec();
    if (m_enable_odometry)
    {
        procOdometryData(x, y, theta, odo_time);
    }
}

// A callback function for subscribing IMU
void DeepGuiderROS::callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (msg->header.stamp == ros::Time(0)) {
        return;
    }

    int seq = msg->header.seq;
    double ori_x = msg->orientation.x;
    double ori_y = msg->orientation.y;
    double ori_z = msg->orientation.z;
    double ori_w = msg->orientation.w;
    double angvel_x = msg->angular_velocity.x;
    double angvel_y = msg->angular_velocity.y;
    double angvel_z = msg->angular_velocity.z;
    double linacc_x = msg->linear_acceleration.x;
    double linacc_y = msg->linear_acceleration.y;
    double linacc_z = msg->linear_acceleration.z;
    ROS_INFO_THROTTLE(1.0, "IMU: seq=%d, orientation=(%f,%f,%f), angular_veloctiy=(%f,%f,%f), linear_acceleration=(%f,%f,%f)", seq, ori_x, ori_y, ori_z, angvel_x, angvel_y, angvel_z, linacc_x, linacc_y, linacc_z);

    const dg::Timestamp imu_time = msg->header.stamp.toSec();
    if (m_enable_imu)
    {
        procImuData(ori_w, ori_x, ori_y, ori_z, imu_time);
    }
}

// A callback function for subscribing OCR output
void DeepGuiderROS::callbackOCR(const dg_simple_ros::ocr_info::ConstPtr& msg)
{
    dg::Timestamp capture_time = msg->timestamp;
    double proc_time = msg->processingtime;
    
    dg::Point2 poi_xy;
    dg::Polar2 relative;
    double poi_confidence;
    for(int i = 0; i<(int)msg->ocrs.size(); i++)
    {
        poi_xy.x = msg->ocrs[i].x;
        poi_xy.y = msg->ocrs[i].y;
        relative.lin = msg->ocrs[i].rel_r;
        relative.ang = msg->ocrs[i].rel_pi;
        poi_confidence = msg->ocrs[i].confidence;
        m_localizer->applyPOI(poi_xy, relative, capture_time, poi_confidence);
    }
}

// A callback function for subscribing a RGB image
void DeepGuiderROS::callbackOCRImage(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1.0, "OCR image (timestamp: %f [sec]).", msg->header.stamp.toSec());
    cv_bridge::CvImagePtr image_ptr;
    cv::Mat image;
    try
    {
        image_ptr = cv_bridge::toCvCopy(msg);
        m_ocr_mutex.lock();
        m_ocr_image = image_ptr->image;
        m_ocr_mutex.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception @ callbackImage(): %s", e.what());
        return;
    }
}

// A callback function for subscribing VPS output topic
void DeepGuiderROS::callbackVPS(const dg_simple_ros::vps::ConstPtr& msg)
{
    dg::Point2 sv_xy(msg->x, msg->y);
    dg::Polar2 relative(msg->rel_r, msg->rel_pi);
    double sv_confidence = msg->confidence;
    dg::Timestamp capture_time = msg->timestamp;
    double proc_time = msg->processingtime;
    m_localizer->applyVPS(sv_xy, relative, capture_time, sv_confidence);

    dg::ID sv_id = msg->id;
    cv::Mat sv_image;
	if (m_vps_use_custom_image_server == 1)
	{
		sv_image = m_vps.getViewImage();
	}
	else
	{
    	MapManager::getStreetViewImage(sv_id, sv_image, m_server_ip, m_image_server_port, "f");
	}
	printf("#####################Debug : VPS Ros Sub.\n");
    //if (MapManager::getStreetViewImage(sv_id, sv_image, m_server_ip, m_image_server_port, "f") && !sv_image.empty())
    if (!sv_image.empty())
    {
        m_vps.set(sv_id, sv_confidence, capture_time, proc_time);
        m_vps.draw(sv_image, 3.0);
        m_vps_mutex.lock();
        m_vps_id = sv_id;
        m_vps_image = sv_image;
        m_vps_mutex.unlock();
    }
}

void DeepGuiderROS::callbackRobotStatus(const std_msgs::String::ConstPtr& msg)
{
    const char* str = msg->data.c_str();
    ROS_INFO_THROTTLE(1.0, "%s", str);
    if (!strcmp(str, "ready"))
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::READY);
    }
    else if (!strcmp(str, "run_manual"))   
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::RUN_MANUAL);
    } 
    else if (!strcmp(str, "run_auto"))   
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::RUN_AUTO);
    } 
    else if (!strcmp(str, "arrived_point"))   
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::ARRIVED_NODE);
    } 
    else if (!strcmp(str, "arrived_goal"))   
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::ARRIVED_GOAL);
    } 
    else if (!strcmp(str, "no_path"))
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::NO_PATH);
    }
}


void DeepGuiderROS::callbackRobotHeading(const std_msgs::String::ConstPtr& msg)
{
    const char* str = msg->data.c_str();
    ROS_INFO_THROTTLE(1.0, "%s", str);

    m_guider.m_robot_heading_node_id = stoi(str);
}

void DeepGuiderROS::callbackRobotMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    ROS_INFO_THROTTLE(1.0, "callbackRobotMap: (timestamp=%f)", map->header.stamp.toSec());

    int size_x = map->info.width;
    int size_y = map->info.height;

    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

    m_robotmap_mutex.lock();

    cv::Mat image(size_y, size_x, CV_8UC1);
    for (int i = 0; i < size_y; i++)
    {
        for (int j = 0; j < size_x; j++)
        {
            int count = i*size_x + j;
            if (map->data[count] <= 0)
            {
                image.at<uchar>(i,j) = 255;
            }
            else
            {
                image.at<uchar>(i,j) = map->data[count];
            }
        }
    }

    m_robotmap_image = image;  
    m_robotmap_mutex.unlock();     
}

void DeepGuiderROS::callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    ROS_INFO_THROTTLE(1.0, "Robot: %f,%f", x, y);

    double ori_x = msg->pose.orientation.x;
    double ori_y = msg->pose.orientation.y;
    double ori_z = msg->pose.orientation.z;
    double ori_w = msg->pose.orientation.w;
    cv::Point3d euler = cx::cvtQuat2EulerAng(ori_w, ori_x, ori_y, ori_z);
    double theta = euler.z;

    dg::Timestamp timestamp = msg->header.stamp.toSec();
    if (m_enable_odometry)
    {
        procOdometryData(x, y, theta, timestamp);
    }

    dg::Timestamp capture_time = msg->header.stamp.toSec();

    //Align robot pose rostopic to display image point
    // 4. DX 로봇 좌표 --> 딥가이더 좌표
    double x_rotation_corrected = x * cos(m_dx_map_rotation_radian) - y * sin(m_dx_map_rotation_radian);
    double y_rotation_corrected = x * sin(m_dx_map_rotation_radian) + y * cos(m_dx_map_rotation_radian);
    dg::LatLon latlon = m_dx_converter.toLatLon(dg::Point2(x_rotation_corrected, y_rotation_corrected));
    dg::Point2 dg_metric2 = m_dg_converter.toMetric(latlon);
    dg::Point2 robot_display = m_painter.cvtValue2Pixel(dg_metric2);
    // ROS_INFO_THROTTLE(1.0, "Robot_img: %f,%f", robot_display.x, robot_display.y);
    cv::circle(m_map_image, robot_display, 3, cv::Vec3b(0, 255, 255));

    m_guider_mutex.lock();
    m_guider.m_robot_pose = cv::Point2d(x, y);
    m_guider.m_robot_on_image = robot_display;
    m_guider_mutex.unlock();
}


void DeepGuiderROS::publishGuidance()
{  
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();

    // make action messages
    dg_simple_ros::action msg_action;
    std::vector<dg_simple_ros::action> msg_actions;
    for (int i = 0; i < cur_guide.actions.size(); i++) 
    {
        msg_action.motion = (int)cur_guide.actions[i].cmd;
        msg_action.move_distance = cur_guide.actions[i].distance;
        msg_action.node_type = (int)cur_guide.actions[i].node_type;
        msg_action.edge_type = (int)cur_guide.actions[i].edge_type;
        msg_action.degree = cur_guide.actions[i].degree;
        msg_action.mode = (int)cur_guide.actions[i].mode;

        msg_actions.push_back(msg_action);
    }    

    // make guidance messages
    dg_simple_ros::guidance msg_guide;
    msg_guide.guide_status = (int)cur_guide.guide_status;
    msg_guide.actions = msg_actions;
    msg_guide.heading_node_id = cur_guide.heading_node_id;
    msg_guide.relative_angle = cur_guide.relative_angle;
    msg_guide.distance_to_remain = cur_guide.distance_to_remain;
    msg_guide.msg = cur_guide.msg;    
    
    pub_guide.publish(msg_guide);
}

void DeepGuiderROS::publishDGPose()
{
    geometry_msgs::PoseStamped rosps;
    dg::Timestamp  timestamp;
    dg::Point2UTM cur_pose = m_localizer->getPoseUTM(&timestamp);
    if (timestamp < 0) return;

    rosps.header.stamp.fromSec(timestamp);
    rosps.pose.position.x = cur_pose.x;
    rosps.pose.position.y = cur_pose.y;    
    pub_pose.publish(rosps);    
}

void DeepGuiderROS::publishSubGoal()
{
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    ROS_INFO_THROTTLE(1.0, "Will publish guide of node: %zu", nid);

    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return;
    
    Pose2 pub_pose = m_guider.m_subgoal_pose;
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
    if (!m_guider.m_dxrobot_usage)
    {
        Point2 pt = cvtLatLon2UTM(toLatLon(node_metric));
        geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pt);
        m_guider.m_subgoal_pose = pt;
        pub_subgoal.publish(rosps);
    }
    else
    {        
        GuidanceManager::RobotStatus cur_state = m_guider.getRobotStatus();
        if (!m_pub_flag && (cur_state == GuidanceManager::RobotStatus::ARRIVED_NODE || cur_state == GuidanceManager::RobotStatus::ARRIVED_GOAL 
        || cur_state == GuidanceManager::RobotStatus::READY || cur_state == GuidanceManager::RobotStatus::NO_PATH))
        {            
            pub_pose = makeSubgoal();
            if (pub_pose.x == 0 || pub_pose.y == 0)
            {
                ROS_INFO_THROTTLE(1.0, "Cannot reselect point: %f, %f", pub_pose.x, pub_pose.y);
                return;
            }
            geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pub_pose);
            pub_subgoal.publish(rosps);
            m_guider.m_subgoal_pose = pub_pose;
            ROS_INFO_THROTTLE(1.0, "SubGoal published!: %f, %f<=====================", pub_pose.x, pub_pose.y);
            m_begin_time = ros::Time::now();
            m_pub_flag = true;
            m_prev_state = cur_state;
        }
        else if (m_pub_flag && m_prev_state == cur_state)
        {
            ros::Time cur_time = ros::Time::now();
            ros::Duration duration = cur_time - m_begin_time;
            if (duration > ros::Duration(5.0))
            {
                ROS_INFO_THROTTLE(1.0, "Duration seconds: %d", duration.sec);
                if (cur_state == GuidanceManager::RobotStatus::NO_PATH) //if robot cannot generate path with the subgoal
                {
                    pub_pose = reselectDrivablePoint();
                    if (pub_pose.x == 0 || pub_pose.y == 0)
                    {
                        ROS_INFO_THROTTLE(1.0, "Cannot reselect point: %f, %f", pub_pose.x, pub_pose.y);
                        return;
                    }
                    
                    ROS_INFO_THROTTLE(1.0, "reselectDrivablePoint!: %f, %f", pub_pose.x, pub_pose.y);
                }
                geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pub_pose);
                pub_subgoal.publish(rosps);
                ROS_INFO_THROTTLE(1.0, "SubGoal published, again!: %f, %f<=====================", pub_pose.x, pub_pose.y);
                m_begin_time = ros::Time::now();
            }
        }
        else if (cur_state == GuidanceManager::RobotStatus::RUN_MANUAL || cur_state == GuidanceManager::RobotStatus::RUN_AUTO)
        {
            m_prev_state = cur_state;
            m_pub_flag = false;
            m_begin_time = ros::Time::now();
        }    
    }
}

Point2 DeepGuiderROS::makeSubgoal()
{
    cv::Mat image;
    Point2 robot_dx_pixel, node_dx_pixel;
    Point2 pub_pose = makeImagePixels(image, robot_dx_pixel, node_dx_pixel);

    if (image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x) < 200) //if node is in black area
    {
        //find drivable area on the line from robot_pt
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-current node is black. find drivable area. value:%d",image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x));
        pub_pose = findDrivablePoint(image, robot_dx_pixel, node_dx_pixel);
    }

    return pub_pose;
}

Point2 DeepGuiderROS::reselectDrivablePoint()
{
    cv::Mat image;
    Point2 robot_dx_pixel, node_dx_pixel, dx_metric, dx_pixel;
    Point2 pub_pose = makeImagePixels(image, robot_dx_pixel, node_dx_pixel);

    Point2 undrivable_pose = m_guider.m_subgoal_pose;
    m_undrivable_points.push_back(undrivable_pose);

    for (int i = 0; i < m_undrivable_points.size(); i++)
    {
        dx_metric = m_undrivable_points[i];
        // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
        dx_pixel.x = m_dx_map_origin_pixel.x + dx_metric.x / m_dx_map_meter_per_pixel;
        dx_pixel.y = m_dx_map_origin_pixel.y - dx_metric.y / m_dx_map_meter_per_pixel;
        image.at<uchar>((int)dx_pixel.y, (int)dx_pixel.x) = 0;
    }

    pub_pose = findDrivablePoint(image, robot_dx_pixel, node_dx_pixel);

    return pub_pose;
}

Point2 DeepGuiderROS::makeImagePixels(cv::Mat& image, Point2& robot_dx_pixel, Point2& node_dx_pixel)
{
    Point2 pub_pose;
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return pub_pose;
    
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
    LatLon latlon = m_map.toLatLon(node_metric);

    cv::Mat onlinemap, offlinemap;
    Point2 robot_dx_metric;
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    // dg::LatLon latlon = m_dg_converter.toLatLon(node_metric);
    dg::Point2 dx_metric_rotated = m_dx_converter.toMetric(latlon);
    double dx_x = dx_metric_rotated.x * cos(m_dx_map_rotation_radian) + dx_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double dx_y = -dx_metric_rotated.x * sin(m_dx_map_rotation_radian) + dx_metric_rotated.y * cos(m_dx_map_rotation_radian);

    dg::Point2 node_dx_metric(dx_x, dx_y);
    ROS_INFO_THROTTLE(1.0, "Subgoal for robot: %f,%f", node_dx_metric.x, node_dx_metric.y);

    //Projecting node point to drivable area
    m_robotmap_mutex.lock();
    onlinemap = m_robotmap_image;
    robot_dx_metric = m_guider.m_robot_pose;
    m_guider.m_robot_heading_node_id = nid;
    m_guider.m_robot_heading_node_pose = node_dx_metric;
    m_robotmap_mutex.unlock();

    // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
    node_dx_pixel.x = m_dx_map_origin_pixel.x + node_dx_metric.x / m_dx_map_meter_per_pixel;
    node_dx_pixel.y = m_dx_map_origin_pixel.y - node_dx_metric.y / m_dx_map_meter_per_pixel;

    robot_dx_pixel.x = m_dx_map_origin_pixel.x + robot_dx_metric.x / m_dx_map_meter_per_pixel;
    robot_dx_pixel.y = m_dx_map_origin_pixel.y - robot_dx_metric.y / m_dx_map_meter_per_pixel;

    ROS_INFO_THROTTLE(1.0, "publishSubGoal-node_on_robotmap: %f,%f", node_dx_pixel.x, node_dx_pixel.y);
    ROS_INFO_THROTTLE(1.0, "publishSubGoal-robot_on_robotmap: %f,%f", robot_dx_pixel.x, robot_dx_pixel.y);

    offlinemap = cv::imread(m_robotmap_path);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO_THROTTLE(1.0, "No occumap");
        image = offlinemap;
    }
    else
    {
        ROS_INFO_THROTTLE(1.0, "Receiving robotmap");
        cv::flip(onlinemap,image,0);
        //robot map is expanded
        if (onlinemap.cols > offlinemap.cols || onlinemap.rows > offlinemap.rows)
        {
            ROS_INFO_THROTTLE(1.0, "publishSubGoal-the robot map is expanded");
            // return false;
        }
    }

    if (image.channels() != 1)
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);


    if (node_dx_pixel.x < 1 || node_dx_pixel.x > image.cols || node_dx_pixel.y < 1 || node_dx_pixel.y > image.rows 
    || robot_dx_pixel.x < 1 || robot_dx_pixel.x > image.cols || robot_dx_pixel.y < 1 || robot_dx_pixel.y > image.rows)
    {
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-the pixel exceeds image size");
        // return false;
    }     

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    pub_pose.x = (node_dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    pub_pose.y = (m_dx_map_origin_pixel.y - node_dx_pixel.y) * m_dx_map_meter_per_pixel;

    return pub_pose;   
}

geometry_msgs::PoseStamped DeepGuiderROS::makeRosPubPoseMsg(ID nid, dg::Point2 xy)
{
    //check publish time
    geometry_msgs::PoseStamped rosps;
    rosps.header.stamp = ros::Time::now();
    rosps.header.frame_id = to_string(nid);
    rosps.pose.position.x = xy.x;
    rosps.pose.position.y = xy.y;
    rosps.pose.position.z = 0;

    return rosps;
}

Point2 DeepGuiderROS::findDrivableNearPoint(cv::Mat &image, Point2 robot_dx, Point2 node_dx)
{
    Point2 dx_pixel, metric;
    dx_pixel = findDrivableNearPixel(image, robot_dx, node_dx);

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    metric.x = (dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    metric.y = (m_dx_map_origin_pixel.y - dx_pixel.y) * m_dx_map_meter_per_pixel;

    return metric;
}

Point2 DeepGuiderROS::findDrivableNearPixel(cv::Mat &image, Point2 robot_px, Point2 node_px)
{
    Point2 px = node_px;
    if (!image.empty())
    {       
        int jump_step = 50;
        int center_x, center_y, x_min, x_max, y_min, y_max;

        cv::Mat cropped_img; 
        double jump_deg = atan2(robot_px.y - node_px.y, robot_px.x - node_px.x);
        int jump_max = (int) norm(robot_px - node_px)/jump_step;

        for (int h = 0; h < jump_max; h++)
        {
            center_x = node_px.x + h * jump_step * cos(jump_deg);
            center_y = node_px.y + h * jump_step * sin(jump_deg);

            x_min = (center_x-jump_step < 1) ? 1 :center_x-jump_step;
            x_max = (center_x+jump_step > image.cols) ? image.cols : center_x+jump_step;
            y_min = (center_y-jump_step < 1) ? 1 : center_y-jump_step;
            y_max = (center_y+jump_step > image.rows) ? image.rows : center_y+jump_step;

            cv::Rect rect(x_min, y_min, y_max-y_min, x_max-x_min);
            cropped_img = image(rect); // cropped image
            if ((cropped_img.cols < 3) || (cropped_img.rows < 3) ){
                ROS_INFO_THROTTLE(1.0,"Cannot provide goal. The goal is on the margin");
                continue;
            }

            double min, max;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(cropped_img, &min, &max, &min_loc, &max_loc);
            
            if (max < 200)
                continue;            

            px.x = max_loc.x + x_min;
            px.y = max_loc.y + y_min;

            //Save image
            cv::Mat img_color;
            cropped_img.copyTo(img_color);
            if (img_color.channels() == 1)
                cv::cvtColor(cropped_img, img_color, cv::COLOR_GRAY2BGR);
            cv::circle(img_color, max_loc, 10, cv::Vec3b(0, 255, 255), 10);
            cv::imwrite("occumap_cropped.png", img_color);   

            return px;
        }
        
        ROS_INFO_THROTTLE(1.0,"Cannot provide goal. All undrivable area!");
        return px;        
    }
}

Point2 DeepGuiderROS::findDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px)
{
    Point2 jump_px, dx_metric;
    cv::Mat img_erode;
    if (!image.empty())
    {
        //find drivable area on the line from robot_pt
        double jump_deg = atan2(robot_px.y - node_px.y, robot_px.x - node_px.x);
        double jump_dist = norm(robot_px - node_px);
        int jump_step = (int)m_robotmap_scale;
        int max_count = (int)jump_dist / jump_step;
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-jump_deg: %f\n", cx::cvtRad2Deg(jump_deg));
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-max_count: %d", max_count);

        //for display map
        erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

        int i;
        for (i = 0; i < max_count; i++)
        {
            jump_px.x = node_px.x + i * jump_step * cos(jump_deg);
            jump_px.y = node_px.y + i * jump_step * sin(jump_deg);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) > 200)
            {
                printf("publishSubGoal-img_erode-%d <%.1f, %.1f>: %d\n", i, jump_px.x, jump_px.y, img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x));
                break;
            }
        }
        if (i >= max_count - 1) //no drivble area on the line
        {
            jump_px = findDrivableNearPixel(img_erode, robot_px, node_px);
            ROS_INFO_THROTTLE(1.0, "publishSubGoal-Find drivable point near goal: %f, %f", jump_px.x, jump_px.y);
        }
    }

    //save to image
    cv::Mat img_color;
    // img_erode.copyTo(img_color);
    cv::cvtColor(img_erode, img_color, cv::COLOR_GRAY2BGR);
    cv::line(img_color, robot_px, node_px, cv::Vec3b(50, 50, 50), 5);
    cv::circle(img_color, node_px, 10, cv::Vec3b(0, 0, 255), 10);
    cv::circle(img_color, robot_px, 10, cv::Vec3b(0, 255, 0), 10);
    cv::circle(img_color, jump_px, 10, cv::Vec3b(0, 255, 255), 10);
    cv::imwrite("occumap_origin.png", img_erode);
    cv::imwrite("occumap_erode.png", img_color);

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    dx_metric.x = (jump_px.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    dx_metric.y = (m_dx_map_origin_pixel.y - jump_px.y) * m_dx_map_meter_per_pixel;

    return dx_metric;
}

void DeepGuiderROS::initialize_DG_DX_conversion()
{
    // DX 로봇 부천 지도 origin 계산
    double ref_x_rotated = (m_dx_map_ref_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    double ref_y_rotated = (m_dx_map_origin_pixel.y - m_dx_map_ref_pixel.y) * m_dx_map_meter_per_pixel;
    double ref_x = ref_x_rotated * cos(m_dx_map_rotation_radian) - ref_y_rotated * sin(m_dx_map_rotation_radian);
    double ref_y = ref_x_rotated * sin(m_dx_map_rotation_radian) + ref_y_rotated * cos(m_dx_map_rotation_radian);

    dg::UTMConverter converter;
    dg::Point2UTM dx_map_ref_utm = converter.cvtLatLon2UTM(m_dx_map_ref_latlon);
    dg::Point2UTM dx_map_origin_utm;
    dx_map_origin_utm.x = dx_map_ref_utm.x - ref_x;
    dx_map_origin_utm.y = dx_map_ref_utm.y - ref_y;
    dx_map_origin_utm.zone = dx_map_ref_utm.zone;
    m_dx_map_origin_latlon = converter.cvtUTM2LatLon(dx_map_origin_utm);
    printf("Robot map origin: lat = %lf, lon = %lf\n", m_dx_map_origin_latlon.lat, m_dx_map_origin_latlon.lon);

    // 딥가이더 부천 좌표 --> DX 부천 로봇 좌표 --> DX 부천 로봇맵 픽셀좌표
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    m_dx_converter.setReference(m_dx_map_origin_latlon);
    m_dg_converter.setReference(m_dg_map_origin_latlon);
}

void DeepGuiderROS::publishPath()
{    
    dg::Path path = getPath();

    // make path points messages
    nav_msgs::Path rospath;
    geometry_msgs::PoseStamped rosps;
    dg::Point2UTM pose_utm;
    for (int i = 0; i < path.pts.size(); i++) 
    {
        pose_utm = cvtLatLon2UTM(toLatLon(path.pts[i]));
        rosps.pose.position.x = pose_utm.x;
        rosps.pose.position.y = pose_utm.y;
        rospath.poses.push_back(rosps);
    }

    // printf("start_lat: %f, start_lon: %f, dest_lat: %f, dest_lon: %f", path.start_pos.lat, path.start_pos.lon, path.dest_pos.lat, path.dest_pos.lon);

    pub_path.publish(rospath);
}

void DeepGuiderROS::publishDGStatus(bool system_shutdown)
{
    dg_simple_ros::dg_status msg;

    dg::Timestamp  timestamp;
    dg::Point2UTM cur_pose = m_localizer->getPoseUTM(&timestamp);
    if (timestamp < 0) return;

    dg::LatLon ll = m_localizer->getPoseGPS();
    msg.dg_shutdown = system_shutdown;
    msg.x = cur_pose.x;
    msg.y = cur_pose.y;
    msg.lat = ll.lat;
    msg.lon = ll.lon;
    msg.confidence = m_localizer->getPoseConfidence();
    msg.timestamp = timestamp;

    pub_status.publish(msg);
}

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_simple_ros");
    ros::NodeHandle nh("~");
    DeepGuiderROS dg_node(nh);
    if (!dg_node.initialize("dg_ros.yml")) return -1;
    dg_node.run();
    return 0;
}
