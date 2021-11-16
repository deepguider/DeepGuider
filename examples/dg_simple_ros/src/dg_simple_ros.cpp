#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
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
    std::string m_topic_cam;
    std::string m_topic_gps;
    std::string m_topic_dgps;
    std::string m_topic_imu;
    std::string m_topic_rgbd_image;
    std::string m_topic_rgbd_depth;

    // Topic subscribers (sensor data)
    ros::Subscriber sub_image_webcam;
    ros::Subscriber sub_image_realsense_image;
    ros::Subscriber sub_image_realsense_depth;
    ros::Subscriber sub_gps_asen;
    ros::Subscriber sub_gps_novatel;
    ros::Subscriber sub_imu_xsense;
    ros::Subscriber sub_robot_status;
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg);
    void callbackRobotStatus(const std_msgs::String::ConstPtr& msg);

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
    CX_LOAD_PARAM_COUNT(fn, "topic_cam", m_topic_cam, n_read);
    CX_LOAD_PARAM_COUNT(fn, "topic_gps", m_topic_gps, n_read);
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
    return n_read;
}

bool DeepGuiderROS::initialize(std::string config_file)
{
    // Initialize main system
    bool ok = DeepGuider::initialize(config_file);
    if(!ok) return false;

    // Initialize sensor subscribers
    if(!m_topic_cam.empty()) sub_image_webcam = nh_dg.subscribe(m_topic_cam, 1, &DeepGuiderROS::callbackImageCompressed, this);
    if(!m_topic_gps.empty()) sub_gps_asen = nh_dg.subscribe(m_topic_gps, 1, &DeepGuiderROS::callbackGPSAsen, this);
    if(!m_topic_dgps.empty()) sub_gps_novatel = nh_dg.subscribe(m_topic_dgps, 1, &DeepGuiderROS::callbackGPSNovatel, this);
    if(!m_topic_imu.empty()) sub_imu_xsense = nh_dg.subscribe(m_topic_imu, 1, &DeepGuiderROS::callbackIMU, this);
    if(!m_topic_rgbd_image.empty()) sub_image_realsense_image = nh_dg.subscribe(m_topic_rgbd_image, 1, &DeepGuiderROS::callbackRealsenseImage, this);
    if(!m_topic_rgbd_depth.empty()) sub_image_realsense_depth = nh_dg.subscribe(m_topic_rgbd_depth, 1, &DeepGuiderROS::callbackRealsenseDepth, this);

    // Initialize deepguider subscribers
    sub_robot_status = nh_dg.subscribe("/keti_robot/status", 1, &DeepGuiderROS::callbackRobotStatus, this);
    sub_ocr = nh_dg.subscribe("/dg_ocr/output", 1, &DeepGuiderROS::callbackOCR, this);
    sub_ocr_image = nh_dg.subscribe("/dg_ocr/image", 1, &DeepGuiderROS::callbackOCRImage, this);
    sub_vps = nh_dg.subscribe("/dg_vps/output", 1, &DeepGuiderROS::callbackVPS, this);

    // Initialize deepguider publishers
    pub_guide = nh_dg.advertise<dg_simple_ros::guidance>("dg_guide", 1, true);
    pub_path = nh_dg.advertise<nav_msgs::Path>("dg_path", 1, true);
    pub_pose = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_", 1, true);
    pub_subgoal = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_goal_utm", 1, true);
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
    if (m_enable_lrpose==1) lrpose_thread = new std::thread(threadfunc_lrpose, this);
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

    // shutdown system
    printf("Shutdown deepguider system...\n");
    publishDGStatus(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    
    // draw GUI display
    cv::Mat gui_image;
    dg::Pose2 px = m_painter.cvtValue2Pixel(getPose());
    if (m_localizer.isPoseStabilized()) m_viewport.centerizeViewportTo(px);

    m_viewport.getViewportImage(gui_image);
    drawGuiDisplay(gui_image, m_viewport.offset(), m_viewport.zoom());

    // recording
    if (m_video_recording) m_video_gui << gui_image;

    cv::imshow(m_winname, gui_image);
    int key = cv::waitKey(1);
    if (key == cx::KEY_SPACE) key = cv::waitKey(0);
    if (key == '1') m_viewport.setZoom(0.5);
    if (key == '2') m_viewport.setZoom(1);
    if (key == '3') m_viewport.setZoom(2);
    if (key == '4') m_viewport.setZoom(3);
    if (key == '5') m_viewport.setZoom(4);    
    if (key == '0') m_exploration_state_count = 0;  // terminate active view
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

    // apply & draw gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    if (!m_use_high_precision_gps) procGpsData(gps_datum, gps_time);
    m_painter.drawPoint(m_map_image, toMetric(gps_datum), m_gui_gps_trj_radius, m_gui_gps_color);
}

// A callback function for subscribing GPS Novatel
void DeepGuiderROS::callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix)
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

    // apply & draw gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    if (m_use_high_precision_gps) procGpsData(gps_datum, gps_time);
    m_painter.drawPoint(m_map_image, toMetric(gps_datum), m_gui_gps_trj_radius, m_gui_gps_novatel_color);
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

// A callback function for subscribing VPS output
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
    if (MapManager::getStreetViewImage(sv_id, sv_image, "f") && !sv_image.empty())
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
    /*
    const char* str = msg->data.c_str();
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
    else if (!strcmp(str, "arrived_node"))   
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::ARRIVED_NODE);
    } 
    else if (!strcmp(str, "arrived_goal"))   
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::ARRIVED_GOAL);
    } 
    */
}

void DeepGuiderROS::publishGuidance()
{  
    /*  
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
    msg_guide.moving_status = (int)cur_guide.moving_status;
    msg_guide.actions = msg_actions;
    msg_guide.heading_node_id = cur_guide.heading_node_id;
    msg_guide.relative_angle = cur_guide.relative_angle;
    msg_guide.distance_to_remain = cur_guide.distance_to_remain;
    msg_guide.msg = cur_guide.msg;    
    
    pub_guide.publish(msg_guide);
    */
}

void DeepGuiderROS::publishDGPose()
{
    geometry_msgs::PoseStamped rosps;
    dg::Timestamp  timestamp;
    dg::Point2UTM cur_pose = m_localizer.getPoseUTM(&timestamp);
    if(timestamp<0) return;

    rosps.header.stamp.fromSec(timestamp);
    rosps.pose.position.x = cur_pose.x;
    rosps.pose.position.y = cur_pose.y;    
    pub_pose.publish(rosps);    
}

void DeepGuiderROS::publishSubGoal()
{
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node* hNode = m_map.getNode(nid);
    if(hNode == nullptr) return;
    Pose2 metric = Pose2(hNode->x, hNode->y);

    dg::Point2UTM node_utm = cvtLatLon2UTM(toLatLon(metric));
    //printf("node_utm.x: %f, node_utm.y: %f\n", node_utm.x, node_utm.y); 

    geometry_msgs::PoseStamped rosps;
    rosps.pose.position.x = node_utm.x;
    rosps.pose.position.y = node_utm.y;
    rosps.pose.position.z = nid;
    
    pub_subgoal.publish(rosps);  
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
    if(timestamp<0) return;

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
