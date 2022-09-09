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
    bool robot_map_onoff;
    dg::LatLon m_robotmap_ref_point_latlon  = dg::LatLon(37.515838, 126.764309);
    dg::Point2 m_robotmap_ref_point_pixel = dg::Point2(2420, 5142);
    double m_robotmap_pixel_per_meter = 10.0;
    double m_robotmap_image_rotation = 1.4;
    double m_robot_map_rotation = -90.0;
    cv::Point2d m_robot_map_origin = cv::Point2d(-311, -257.6);
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
    CX_LOAD_PARAM_COUNT(fn, "robot_map_onoff", robot_map_onoff, n_read);
    m_guider.setRobotMapOnOff(robot_map_onoff);
    cv::Vec2d ref_point = cv::Vec2d(m_robotmap_ref_point_latlon.lat, m_robotmap_ref_point_latlon.lon);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_latlon", ref_point, n_read);
    m_robotmap_ref_point_latlon = dg::LatLon(ref_point[0], ref_point[1]);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_pixel", m_robotmap_ref_point_pixel, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_pixel_per_meter", m_robotmap_pixel_per_meter, n_read);
    CX_LOAD_PARAM_COUNT(fn, "map_image_rotation", m_robotmap_image_rotation, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robot_map_rotation", m_robot_map_rotation, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robot_map_origin", m_robot_map_origin, n_read);

    int robot_site_index = -1;
    std::string robot_site_tagname;
    std::vector<cv::String> robot_site_set;
    CX_LOAD_PARAM_COUNT(fn, "robot_site_set", robot_site_set, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robot_site_index", robot_site_index, n_read);
    if (robot_site_index >= 0 && robot_site_index < robot_site_set.size()) robot_site_tagname = robot_site_set[robot_site_index];

    // Read Robot Setting
    if (!robot_site_tagname.empty())
    {
        cv::FileNode fn_robot = fn[robot_site_tagname];
        if (!fn_robot.empty())
        {
            n_read += readRosParam(fn_robot);
        }
    }
    m_guider.setRobotUsage(topicset_tagname);
	printf("topicset_tagname: %s\n", topicset_tagname.c_str());
    m_guider.setRobotMapOnOff(false);


	printf("m_robotmap_ref_point_pixel.x: %f, m_robotmap_ref_point_pixel.y: %f\n", m_robotmap_ref_point_pixel.x, m_robotmap_ref_point_pixel.y);
	printf("m_robotmap_pixel_per_meter: %f\n", m_robotmap_pixel_per_meter);
	printf("map_image_rotation: %f\n", m_robotmap_image_rotation);
    
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
    if(m_dest_defined && m_path_generation_pended && m_localizer.isPoseInitialized())
    {
        if(updateDeepGuiderPath(getPose(), m_dest)) m_path_generation_pended = false;        
    }
    
    // draw GUI display
    cv::Mat gui_image;
    dg::Pose2 px = m_painter.cvtValue2Pixel(getPose());
    if (m_gui_auto_scroll && m_localizer.isPoseInitialized()) m_viewport.centerizeViewportTo(px);

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
        if(m_enable_odometry && m_apply_odometry) m_localizer.resetOdometry();
    }
    if (key == 'v' || key == 'V') m_apply_vps = !m_apply_vps;
    if (key == 'p' || key == 'P') m_apply_ocr = !m_apply_ocr;
    if (key == 'i' || key == 'I') m_apply_intersection = !m_apply_intersection;
    if (key == 'l' || key == 'L') m_apply_roadlr = !m_apply_roadlr;
    if (key == 't' || key == 'T') m_apply_roadtheta = !m_apply_roadtheta;    
    if (key == 'a') m_gui_auto_scroll = !m_gui_auto_scroll;  // toggle auto scroll of the map view
    if (key == 'k') m_show_ekf_pose = !m_show_ekf_pose;
    if (key == 'j') m_localizer.toggleEnablePathProjection();

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
    ROS_INFO_THROTTLE(1.0, "Compressed RGB(timestamp: %f [sec]).", msg->header.stamp.toSec());
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
        ROS_INFO_THROTTLE(1.0, "GPS Asen: no fix (timestamp: %f [sec])", fix->header.stamp.toSec());
        return;
    }
    ROS_INFO_THROTTLE(1.0, "GPS Asen: lat=%f, lon=%f (timestamp: %f [sec])", fix->latitude, fix->longitude, fix->header.stamp.toSec());

    double lat = fix->latitude;
    double lon = fix->longitude;

    // apply & draw gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    double timestamp_offset = timestamp - gps_time;
    if (timestamp_offset > 60) m_timestamp_offset = timestamp_offset;
    if (!m_use_high_precision_gps) procGpsData(gps_datum, gps_time);
    m_painter.drawPoint(m_map_image, toMetric(gps_datum), m_gui_gps_trj_radius, m_gui_gps_color);
}

// A callback function for subscribing GPS Novatel
void DeepGuiderROS::callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix)
{
    if (fix->header.stamp == ros::Time(0)) return;

    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_INFO_THROTTLE(1.0, "GPS Novatel: no fix (timestamp: %f [sec])", fix->header.stamp.toSec());
        return;
    }
    ROS_INFO_THROTTLE(1.0, "GPS Novatel: lat=%f, lon=%f (timestamp: %f [sec])", fix->latitude, fix->longitude, fix->header.stamp.toSec());

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
    if (m_timestamp_offset>0) odo_time = odo_time - m_timestamp_offset;
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
        m_localizer.applyPOI(poi_xy, relative, capture_time, poi_confidence);
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
    m_localizer.applyVPS(sv_xy, relative, capture_time, sv_confidence);

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
}


void DeepGuiderROS::callbackRobotHeading(const std_msgs::String::ConstPtr& msg)
{
    const char* str = msg->data.c_str();
    ROS_INFO_THROTTLE(1.0, "%s", str);

    m_guider.m_robot_heading = stoi(str);

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

    // if (image.empty() == false)
    // {
    //     cv::imshow("occumap_kaist",image);
    //     cv::waitKey();
    //     cv::imwrite("occumap_kaist.png", image);
    // }
    // else
    // {
    //     ROS_INFO_THROTTLE(1.0, "No occumap");
    // }

    m_robotmap_image = image;  
    m_robotmap_mutex.unlock();
     


/**
    // try
    {
        m_robotmap_mutex.lock();
        std_msgs::Header header = map->header;
        nav_msgs::MapMetaData info = map->info;
        ROS_INFO_THROTTLE(1.0, "Got robot map (timestamp: %f [sec], W:%d, H:%d).", map->header.stamp.toSec(), info.width, info.height);
        
        cv::Mat image(size_y, size_x, CV_8UC1);
        image = cv::imdecode(cv::Mat(map->data), 1);
        if (image.empty() == false)
        {
            cv::imshow("occumap_kaist",image);
            cv::waitKey();
            cv::imwrite("occumap_kaist2.png", image);
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "No occumap");
        }

        m_robotmap_image = image;  
        m_robotmap_capture_time = map->header.stamp.toSec();
        m_robotmap_mutex.unlock();

    }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("exception @ callbackRobotMap(): %s", e.what());
    //     return;
    // }

    */
}


void DeepGuiderROS::callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    dg::Pose2 robot_pose;
    robot_pose.x = msg->pose.position.x;
    robot_pose.y = msg->pose.position.y;
    ROS_INFO_THROTTLE(1.0, "Robot: %f,%f", robot_pose.x, robot_pose.y);

    dg::Timestamp capture_time = msg->header.stamp.toSec();        
    cv::Point2d img_pt_dx;

    //Align robot pose to image point
	ROS_INFO_THROTTLE(1.0, "m_guider.m_site_name: %d", m_guider.m_site_name);
	if(m_guider.m_site_name >= 1) // "KETI_ROBOT"
    {    //Aligned to "bucheon_robotmap(flipudlr).png"

        // double img_deg_dx1 = -90;
        // cv::Point2d img_scale_dx1 = cv::Point2d(10.0, 10.0);
        // cv::Point2d img_offset_dx1 = cv::Point2d(2576, 3110);
        // cv::Point2d img_pt_dx1 = m_guider.cvtValue2Pixel4Guidance(robot_pose, img_deg_dx1, img_scale_dx1, img_offset_dx1);
        // ROS_INFO_THROTTLE(1.0, "[before]callbackRobotPose-robot2image: %f,%f", img_pt_dx1.x, img_pt_dx1.y);

        double img_deg_dx = m_robot_map_rotation;
        cv::Point2d img_scale_dx = cv::Point2d(m_robotmap_pixel_per_meter, m_robotmap_pixel_per_meter);
        cv::Point2d img_offset_dx = m_guider.cvtValue2Pixel4Guidance(m_robot_map_origin, -img_deg_dx, img_scale_dx, cv::Point2d(0.0,0.0));
        img_pt_dx = m_guider.cvtValue2Pixel4Guidance(robot_pose, img_deg_dx, img_scale_dx, img_offset_dx);
        ROS_INFO_THROTTLE(1.0, "callbackRobotPose-robot2image: %f,%f", img_pt_dx.x, img_pt_dx.y);
   
        m_guider.m_robot_pose = img_pt_dx;
        cv::circle(m_map_image, img_pt_dx, 10, cv::Vec3b(0, 255, 0));
    }
        
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
    dg::Point2UTM cur_pose = m_localizer.getPoseUTM(&timestamp);
    if (timestamp < 0) return;

    rosps.header.stamp.fromSec(timestamp);
    rosps.pose.position.x = cur_pose.x;
    rosps.pose.position.y = cur_pose.y;    
    pub_pose.publish(rosps);    
}

void DeepGuiderROS::publishSubGoal()
{
    geometry_msgs::PoseStamped rosps;
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    double angle = cur_guide.relative_angle;
    double dist = cur_guide.distance_to_remain;
     
    Node* hNode = m_map.getNode(nid);
    if(hNode == nullptr) return;
    cv::Point2d pt;

    //Align KAIST's Bucheon map 2022-03-10 to DG map
	if(m_guider.m_site_name >= 1) // 1:"Bucheon_KETI"
    {    //Aligned to "bucheon_robotmap(flipudlr).png"

        Pose2 node_pt = Pose2(hNode->x, hNode->y);
        //ROS_INFO_THROTTLE(1.0, "heading_node_id: %zd<==============", nid);

        // double img_deg1 = 1.4;
        // cv::Point2d img_scale1 = cv::Point2d(10.2, 10.2);
        // cv::Point2d img_offset1 = cv::Point2d(2400, 5142);
        // cv::Point2d img_pt1 = m_guider.cvtValue2Pixel4Guidance(node_pt, img_deg1, img_scale1, img_offset1);;
        // ROS_INFO_THROTTLE(1.0, "[before] publishSubGoal-node2img: %f,%f", img_pt1.x, img_pt1.y);


        double img_deg = m_robotmap_image_rotation;
        cv::Point2d img_scale = cv::Point2d(m_robotmap_pixel_per_meter, m_robotmap_pixel_per_meter);
        cv::Point2d img_offset = m_robotmap_ref_point_pixel;
        cv::Point2d img_pt = m_guider.cvtValue2Pixel4Guidance(node_pt, img_deg, img_scale, img_offset);
        // ROS_INFO_THROTTLE(1.0, "[after] publishSubGoal-node2img: %f,%f", img_pt.x, img_pt.y);

        //translation to image to robot's coordintate
        // double r_deg1 = -90;
        // cv::Point2d r_scale1 = cv::Point2d(0.1, 0.1);
        // cv::Point2d r_offset1 = cv::Point2d(-311, -257.6);
        // cv::Point2d pt1 = m_guider.cvtValue2Pixel4Guidance(img_pt1, r_deg1, r_scale1, r_offset1);
        // ROS_INFO_THROTTLE(1.0, "[before] publishSubGoal-img2robot: %f,%f", pt1.x, pt1.y);

        double r_deg = m_robot_map_rotation;
        cv::Point2d r_scale = cv::Point2d(1/m_robotmap_pixel_per_meter, 1/m_robotmap_pixel_per_meter);
        cv::Point2d r_offset = cv::Point2d(m_robot_map_origin.x, m_robot_map_origin.y);
        pt = m_guider.cvtValue2Pixel4Guidance(img_pt, r_deg, r_scale, r_offset);

        // if (m_guider.m_use_online_map)
        // {
        //     //translated robot's pose
        //     //translated heading node's pose
        //     pt;
        // } 
  
    }
    else
    {
        Pose2 metric = Pose2(hNode->x, hNode->y);
        pt = cvtLatLon2UTM(toLatLon(metric));
    }

    //m_guider.getGuidancePoint();

    dg::Timestamp  timestamp;
    dg::Point2UTM cur_pose = m_localizer.getPoseUTM(&timestamp);
    if (timestamp < 0) return;
    rosps.header.stamp.fromSec(timestamp);
    rosps.header.frame_id = to_string(cur_guide.heading_node_id);

    rosps.pose.position.x = pt.x;
    rosps.pose.position.y = pt.y;
    rosps.pose.position.z = 0;

    ROS_INFO_THROTTLE(1.0, "publishSubGoal-img2robot: %f,%f", pt.x, pt.y);
    
    pub_subgoal.publish(rosps);  
    m_guider.m_goal_pose = pt;
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
    dg::Point2UTM cur_pose = m_localizer.getPoseUTM(&timestamp);
    if (timestamp < 0) return;

    dg::LatLon ll = m_localizer.getPoseGPS();
    msg.dg_shutdown = system_shutdown;
    msg.x = cur_pose.x;
    msg.y = cur_pose.y;
    msg.lat = ll.lat;
    msg.lon = ll.lon;
    msg.confidence = m_localizer.getPoseConfidence();
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
