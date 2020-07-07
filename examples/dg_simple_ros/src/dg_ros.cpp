#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include "dg_simple.cpp"

class DeepGuiderROS : public DeepGuider
{
public:
    DeepGuiderROS(ros::NodeHandle& nh);
    virtual ~DeepGuiderROS();

    bool initialize();
    int run();
    bool runOnce(double timestamp);

protected:
    double m_wait_sec = 0.1;

    // Thread routines
    std::thread* vps_thread = nullptr;
    std::thread* poi_thread = nullptr;
    std::thread* intersection_thread = nullptr;
    std::thread* roadtheta_thread = nullptr;
    static void threadfunc_vps(DeepGuiderROS* guider);
    static void threadfunc_poi(DeepGuiderROS* guider);
    static void threadfunc_intersection(DeepGuiderROS* guider);
    static void threadfunc_roadtheta(DeepGuiderROS* guider);    
    bool is_vps_running = false;
    bool is_poi_running = false;
    bool is_intersection_running = false;
    bool is_roadtheta_running = false;
    void terminateThreadFunctions();

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
};


DeepGuiderROS::DeepGuiderROS(ros::NodeHandle& nh) : nh_dg(nh)
{
    // overwrite configuable parameters of base class
    m_enable_roadtheta = false;
    m_enable_vps = true;
    m_enable_poi = true;
    m_enable_logo = false;
    m_enable_intersection = false;
    m_enable_exploration = false;

    //m_server_ip = "127.0.0.1";        // default: 127.0.0.1 (localhost)
    m_server_ip = "129.254.87.96";      // default: 127.0.0.1 (localhost)
    m_threaded_run_python = true;
    m_srcdir = "/work/deepguider/src";   // system path of deepguider/src (required for python embedding)

    m_recording = false;
    m_map_image_path = "data/NaverMap_ETRI(Satellite)_191127.png";
    m_recording_header_name = "dg_ros_";

    // Read ros-specific parameters
    m_wait_sec = 0.1;
    nh_dg.param<double>("wait_sec", m_wait_sec, m_wait_sec);

    // Initialize subscribers
    sub_image_webcam = nh_dg.subscribe("/uvc_image_raw/compressed", 1, &DeepGuiderROS::callbackImageCompressed, this);
    sub_gps_asen = nh_dg.subscribe("/asen_fix", 1, &DeepGuiderROS::callbackGPSAsen, this);
    sub_gps_novatel = nh_dg.subscribe("/novatel_fix", 1, &DeepGuiderROS::callbackGPSNovatel, this);
    sub_imu_xsense = nh_dg.subscribe("/imu/data", 1, &DeepGuiderROS::callbackIMU, this);
    sub_image_realsense_image = nh_dg.subscribe("/camera/color/image_raw/compressed", 1, &DeepGuiderROS::callbackRealsenseImage, this);
    sub_image_realsense_depth = nh_dg.subscribe("/camera/depth/image_rect_raw/compressed", 1, &DeepGuiderROS::callbackRealsenseDepth, this);

    // Initialize publishers
    pub_image_gui = nh_dg.advertise<sensor_msgs::CompressedImage>("dg_image_gui", 1, true);
}

DeepGuiderROS::~DeepGuiderROS()
{    
}

bool DeepGuiderROS::initialize()
{
    bool ok = DeepGuider::initialize("dg_ros.yml");
    return ok;
}

int DeepGuiderROS::run()
{
    printf("Run deepguider system...\n");

    // start & goal position
    dg::LatLon gps_start(36.38205717, 127.3676462);
    dg::LatLon gps_dest(36.37944417, 127.3788568);
    VVS_CHECK_TRUE(initializeMapAndPath(gps_start, gps_dest));
    printf("\tgps_start: lat=%lf, lon=%lf\n", gps_start.lat, gps_start.lon);
    printf("\tgps_dest: lat=%lf, lon=%lf\n", gps_dest.lat, gps_dest.lon);
    m_gps_start = gps_start;
    m_gps_dest = gps_dest;

    cv::namedWindow("deep_guider", cv::WINDOW_AUTOSIZE);

    // start recognizer threads
    if (m_enable_vps) vps_thread = new std::thread(threadfunc_vps, this);
    if (m_enable_poi) poi_thread = new std::thread(threadfunc_poi, this);
    if (m_enable_intersection) intersection_thread = new std::thread(threadfunc_intersection, this);
    if (m_enable_roadtheta) roadtheta_thread = new std::thread(threadfunc_roadtheta, this);

    ros::Rate loop(1 / m_wait_sec);
    while (ros::ok())
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec())) break;
        ros::spinOnce();
        loop.sleep();
    }
    if(m_recording) m_video.release();
    terminateThreadFunctions();
    cv::destroyWindow("deep_guider");
    printf("End deepguider system...\n");

    return 0;
}

bool DeepGuiderROS::runOnce(double timestamp)
{
    // process Guidance
    procGuidance(timestamp);

    // draw GUI display
    cv::Mat gui_image = m_map_image.clone();
    drawGuiDisplay(gui_image);

    // recording
    if (m_recording) m_video << gui_image;

    cv::imshow("deep_guider", gui_image);
    int key = cv::waitKey(1);
    //if (key == cx::KEY_SPACE) key = cv::waitKey(0);
    if (key == cx::KEY_ESC) return false;

    return true;
}

// Thread fnuction for VPS
void DeepGuiderROS::threadfunc_vps(DeepGuiderROS* guider)
{
    guider->is_vps_running = true;
    while (guider->m_enable_vps)
    {
        guider->procVps();
    }
    guider->is_vps_running = false;
}

// Thread fnuction for POI
void DeepGuiderROS::threadfunc_poi(DeepGuiderROS* guider)
{
    guider->is_poi_running = true;
    while (guider->m_enable_poi)
    {
        guider->procPoi();
    }
    guider->is_poi_running = false;
}

// Thread fnuction for IntersectionClassifier
void DeepGuiderROS::threadfunc_intersection(DeepGuiderROS* guider)
{
    guider->is_intersection_running = true;
    while (guider->m_enable_intersection)
    {
        guider->procIntersectionClassifier();
    }
    guider->is_intersection_running = false;
}

// Thread fnuction for RoadTheta
void DeepGuiderROS::threadfunc_roadtheta(DeepGuiderROS* guider)
{
    guider->is_roadtheta_running = true;
    while (guider->m_enable_roadtheta)
    {
        guider->procRoadTheta();
    }
    guider->is_roadtheta_running = false;
}


void DeepGuiderROS::terminateThreadFunctions()
{
    if (vps_thread == nullptr && poi_thread == nullptr && intersection_thread == nullptr && roadtheta_thread == nullptr) return;

    // disable all thread running
    m_enable_vps = false;
    m_enable_poi = false;
    m_enable_intersection = false;
    m_enable_roadtheta = false;

    // wait up to 4000 ms at maximum
    for (int i = 0; i<40; i++)
    {
        if (!is_vps_running && !is_poi_running && !is_intersection_running && !is_roadtheta_running) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // wait child thread to terminate
    if (is_vps_running) vps_thread->join();
    if (is_poi_running) poi_thread->join();
    if (is_intersection_running) intersection_thread->join();
    if (is_roadtheta_running) roadtheta_thread->join();

    // clear threads
    if (vps_thread) delete vps_thread;
    if (poi_thread) delete poi_thread;
    if (intersection_thread) delete intersection_thread;
    if (roadtheta_thread) delete roadtheta_thread;
    vps_thread = nullptr;
    poi_thread = nullptr;
    intersection_thread = nullptr;
    roadtheta_thread = nullptr;
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
        m_cam_mutex.lock();
        m_cam_image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
        m_cam_capture_time = msg->header.stamp.toSec();
        m_cam_capture_pos = m_localizer.getPoseGPS();
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

    // apply gps
    const dg::LatLon gps_datum(lat, lon);
    const dg::Timestamp gps_time = fix->header.stamp.toSec();
    applyGpsData(gps_datum, gps_time);
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

    // draw gps history on the map
    const dg::LatLon gps_datum(lat, lon);
    dg::Point2 gps_pt = m_localizer.toMetric(gps_datum);
    m_painter.drawNode(m_map_image, m_map_info, dg::Point2ID(0, gps_pt), 2, 0, cv::Vec3b(0, 0, 255));
}

// A callback function for subscribing IMU
void DeepGuiderROS::callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
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
    DeepGuiderROS dg_node(nh);
    if (!dg_node.initialize()) return -1;
    dg_node.run();
    return 0;
}
