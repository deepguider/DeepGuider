#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include "dg_simple_ros/ocr_info.h"
#include "dg_simple.cpp"
#include <dg_simple_ros/guidance.h>
#include <dg_simple_ros/action.h>
#include <nav_msgs/Path.h>

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
    double m_wait_sec = 0.01;

    // Topic names
    std::string m_topic_cam;
    std::string m_topic_gps;
    std::string m_topic_dgps;
    std::string m_topic_imu;
    std::string m_topic_rgbd_image;
    std::string m_topic_rgbd_depth;

    // Topic subscribers (sub modules)
    ros::Subscriber sub_ocr;
    void callbackOCR(const dg_simple_ros::ocr_info::ConstPtr& msg);

    // Topic subscribers (sensor data)
    ros::Subscriber sub_image_webcam;
    ros::Subscriber sub_image_realsense_image;
    ros::Subscriber sub_image_realsense_depth;
    ros::Subscriber sub_gps_asen;
    ros::Subscriber sub_gps_novatel;
    ros::Subscriber sub_imu_xsense;

    // Subscriber callbacks (sensor data)    
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg);
    void callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix);
    void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg);

    // Topic publishers
    ros::Publisher pub_guide;
    ros::Publisher pub_path;
    ros::Publisher pub_pose;
    void publishGuidance();
    void publishPath();
    void publishDGPose();

    // A node handler
    ros::NodeHandle& nh_dg;

    // timestamp to framenumber converter (utility function)
    int t2f_n = 0;
    double t2f_offset_fn = 0;
    double t2f_scale = 0;
    double t2f_offset_ts = 0;
    dg::Timestamp t2f_ts;
    int t2f_fn;
    void updateTimestamp2Framenumber(dg::Timestamp ts, int fn);
    int timestamp2Framenumber(dg::Timestamp ts);
};   

DeepGuiderROS::DeepGuiderROS(ros::NodeHandle& nh) : nh_dg(nh)
{
    // overwrite configuable parameters of base class
    m_enable_intersection = false;
    m_enable_vps = false;
    m_enable_lrpose = false;
    m_enable_logo = false;
    m_enable_ocr = false;
    m_enable_roadtheta = false;
    m_enable_exploration = false;
    m_enable_mapserver = true;

    m_server_ip = "127.0.0.1";  // default: 127.0.0.1 (localhost)
    m_srcdir = "/home/dgtest/deepguider/src";      // system path of deepguider/src (required for python embedding)
    m_enable_tts = true;
    m_threaded_run_python = true;
    m_use_high_precision_gps = false;

    m_data_logging = false;
    m_video_recording = false;
    m_video_recording_fps = 15;
    m_recording_header_name = "dg_ros_";

    m_map_image_path = "data/NaverMap_ETRI(Satellite)_191127.png";
    m_map_data_path = "data/ETRI/TopoMap_ETRI_210803.csv";
    m_map_ref_point = dg::LatLon(36.383837659737, 127.367880828442);
    m_map_ref_point_pixel = dg::Point2(347, 297);
    m_map_pixel_per_meter = 1.039;
    m_map_image_rotation = cx::cvtDeg2Rad(1.0);

    // ros-specific parameters
    m_wait_sec = 0.1;
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

    // Initialize module subscribers
    sub_ocr = nh_dg.subscribe("/dg_ocr/output", 1, &DeepGuiderROS::callbackOCR, this);

    // Initialize sensor subscribers
    if(!m_topic_cam.empty()) sub_image_webcam = nh_dg.subscribe(m_topic_cam, 1, &DeepGuiderROS::callbackImageCompressed, this);
    if(!m_topic_gps.empty()) sub_gps_asen = nh_dg.subscribe(m_topic_gps, 1, &DeepGuiderROS::callbackGPSAsen, this);
    if(!m_topic_dgps.empty()) sub_gps_novatel = nh_dg.subscribe(m_topic_dgps, 1, &DeepGuiderROS::callbackGPSNovatel, this);
    if(!m_topic_imu.empty()) sub_imu_xsense = nh_dg.subscribe(m_topic_imu, 1, &DeepGuiderROS::callbackIMU, this);
    if(!m_topic_rgbd_image.empty()) sub_image_realsense_image = nh_dg.subscribe(m_topic_rgbd_image, 1, &DeepGuiderROS::callbackRealsenseImage, this);
    if(!m_topic_rgbd_depth.empty()) sub_image_realsense_depth = nh_dg.subscribe(m_topic_rgbd_depth, 1, &DeepGuiderROS::callbackRealsenseDepth, this);

    // Initialize publishers
    pub_guide = nh_dg.advertise<dg_simple_ros::guidance>("dg_guide", 1, true);
    pub_path = nh_dg.advertise<nav_msgs::Path>("dg_path", 1, true);
    pub_pose = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_pose_utm", 1, true);

    return true;
}

int DeepGuiderROS::run()
{
    printf("Run deepguider system...\n");

    // start recognizer threads
    if (m_enable_vps) vps_thread = new std::thread(threadfunc_vps, this);
    if (m_enable_ocr) ocr_thread = new std::thread(threadfunc_ocr, this);    
    if (m_enable_logo) logo_thread = new std::thread(threadfunc_logo, this);
    if (m_enable_intersection) intersection_thread = new std::thread(threadfunc_intersection, this);
    if (m_enable_roadtheta) roadtheta_thread = new std::thread(threadfunc_roadtheta, this);

    // run main loop
    ros::Rate loop(1 / m_wait_sec);
    while (ros::ok())
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec())) break;
        ros::spinOnce();
        loop.sleep();
    }

    // end system
    printf("End deepguider system...\n");
    terminateThreadFunctions();
    printf("\tthread terminated\n");
    if(m_video_recording) m_video_gui.release();
    if(m_data_logging) m_video_cam.release();
    printf("\tclose recording\n");
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
    
    // draw GUI display
    cv::Mat gui_image;
    m_viewport.getViewportImage(gui_image);
    drawGuiDisplay(gui_image, m_viewport.offset(), m_viewport.zoom());

    // recording
    if (m_video_recording) m_video_gui << gui_image;

    cv::imshow(m_winname, gui_image);
    int key = cv::waitKey(1);
    if (key == cx::KEY_SPACE) key = cv::waitKey(0);
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
    std::vector<OCRResult> ocrs;
    for(int i = 0; i<(int)msg->ocrs.size(); i++)
    {
        OCRResult ocr;
        ocr.label = msg->ocrs[i].label;
        ocr.xmin = msg->ocrs[i].xmin;
        ocr.ymin = msg->ocrs[i].ymin;
        ocr.xmax = msg->ocrs[i].xmax;
        ocr.ymax = msg->ocrs[i].ymax;
        ocr.confidence = msg->ocrs[i].confidence;
        ocrs.push_back(ocr);
    }
    dg::Timestamp capture_time = msg->timestamp;
    double proc_time = msg->processingtime;
    int cam_fnumber = timestamp2Framenumber(capture_time);

    if (!ocrs.empty() && cam_fnumber>=0)
    {
        m_ocr.set(ocrs, capture_time, proc_time);

        if (m_data_logging)
        {
            m_log_mutex.lock();
            m_ocr.write(m_log, cam_fnumber);
            m_log_mutex.unlock();
        }
        m_ocr.print();

        std::vector<dg::Point2> poi_xys;
        std::vector<dg::Polar2> relatives;
        std::vector<double> poi_confidences;
        if (m_ocr.getLocClue(getPose(), poi_xys, relatives, poi_confidences))
        {
            for (int k = 0; k < (int)poi_xys.size(); k++)
            {
                m_localizer.applyPOI(poi_xys[k], relatives[k], capture_time, poi_confidences[k]);
            }
        }
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
    msg_guide.moving_status = (int)cur_guide.moving_status;
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
    dg::Point2UTM cur_pose = m_localizer.getPoseUTM();
    rosps.pose.position.x = cur_pose.x;
    rosps.pose.position.y = cur_pose.y;
    
    //printf("cur_pose.x: %f, cur_pose.y: %f\n", cur_pose.x, cur_pose.y);

    pub_pose.publish(rosps);    
}

void DeepGuiderROS::publishPath()
{    
    Path* path = getPathLocked();
    if(path == nullptr || path->empty())
    {
        releasePathLock();
        return;
    }

    // make path points messages
    nav_msgs::Path rospath;
    geometry_msgs::PoseStamped rosps;
    dg::Point2UTM pose_utm;
    for (int i = 0; i < path->pts.size(); i++) 
    {
        pose_utm = cvtLatLon2UTM(toLatLon(path->pts[i]));
        rosps.pose.position.x = pose_utm.x;
        rosps.pose.position.y = pose_utm.y;
        rospath.poses.push_back(rosps);
    }
    releasePathLock();

    // printf("start_lat: %f, start_lon: %f, dest_lat: %f, dest_lon: %f", path.start_pos.lat, path.start_pos.lon, path.dest_pos.lat, path.dest_pos.lon);

    pub_path.publish(rospath);
}

void DeepGuiderROS::updateTimestamp2Framenumber(dg::Timestamp ts, int fn)
{
    if(t2f_n>1)
    {
        double scale = (fn - t2f_fn) / (ts - t2f_ts);
        t2f_scale = t2f_scale * 0.9 + scale * 0.1;

        double fn_est = (ts - t2f_offset_ts)*t2f_scale + t2f_offset_fn;
        double est_err = fn - fn_est;
        t2f_offset_fn = t2f_offset_fn + est_err;

        t2f_ts = ts;
        t2f_fn = fn;
        t2f_n++;
        //int fn_est2 = timestamp2Framenumber(ts);
        //printf("[timestamp=%d] err=%.1lf, fn=%d, fn_est=%d", t2f_n, est_err, fn, fn_est2);
        return;
    }
    if(t2f_n == 1)
    {
        t2f_scale = (fn - t2f_fn) / (ts - t2f_ts);
        t2f_ts = ts;
        t2f_fn = fn;
        t2f_n = 2;        
        return;
    }
    if(t2f_n<=0)
    {
        t2f_ts = ts;
        t2f_fn = fn;        
        t2f_offset_ts = ts;
        t2f_offset_fn = fn;
        t2f_scale = 1;
        t2f_n = 1;
        return;
    }
}

int DeepGuiderROS::timestamp2Framenumber(dg::Timestamp ts)
{
    if(t2f_n<=0) return -1;

    int fn = (int)((ts - t2f_offset_ts)*t2f_scale + t2f_offset_fn + 0.5);
    return fn;
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
