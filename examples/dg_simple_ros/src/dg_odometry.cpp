#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>

#define LOAD_PARAM_VALUE(fn, name_cfg, name_var) \
    if (!(fn)[name_cfg].empty()) (fn)[name_cfg] >> name_var;

struct Pose2
{
    double x,y,theta;
    Pose2(): x(0), y(0), theta(0) {}
};

inline double trimRad(double radian)
{
    radian -= static_cast<int>(radian / (2 * CV_PI)) * (2 * CV_PI);
    if (radian >= CV_PI) radian -= 2 * CV_PI;
    if (radian < -CV_PI) radian += 2 * CV_PI;
    return radian;
}

class DGNodeOdometry
{
public:
    DGNodeOdometry(ros::NodeHandle& nh);
    virtual ~DGNodeOdometry();

    bool initialize(std::string config_file);
    int run();
    bool runOnce(double timestamp);

protected:
    bool loadConfig(std::string config_file);
    bool m_enable_module = true;

    double m_pulse_left = 0;
    double m_pulse_right = 0;
    double m_prev_pulse_left = 0;
    double m_prev_pulse_right = 0;
    double m_prev_timestamp = 0;
    Pose2 m_pose;
    Pose2 m_pose_prev;
    cv::Mutex m_mutex_left;
    cv::Mutex m_mutex_right;
    
    // Topic subscribers
    ros::Subscriber sub_encoder_left;
    ros::Subscriber sub_encoder_right;
    void callbackEncoderLeft(const std_msgs::Int32& msg);
    void callbackEncoderRight(const std_msgs::Int32& msg);

    // Topic publishers
    ros::Publisher m_publisher_odometry;

    // A node handler
    ros::NodeHandle& nh_dg;
    double m_update_hz = 10;
};

DGNodeOdometry::DGNodeOdometry(ros::NodeHandle& nh) : nh_dg(nh)
{
}

DGNodeOdometry::~DGNodeOdometry()
{
}

bool DGNodeOdometry::loadConfig(std::string config_file)
{
    if (config_file.empty()) return false;

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    cv::FileNode fn = fs.root();
    LOAD_PARAM_VALUE(fn, "enable_odometry", m_enable_module);
    LOAD_PARAM_VALUE(fn, "odometry_update_hz", m_update_hz);

    return true;
}

bool DGNodeOdometry::initialize(std::string config_file)
{
    printf("Initialize dg_odometry..\n");
    loadConfig(config_file);

    // check if module enabled in the configuration file
    if(!m_enable_module)
    {
        printf("\tdg_odometry disabled in %s\n", config_file.c_str());
        return false;
    }

    // Initialize subscribers
    sub_encoder_left = nh_dg.subscribe("/dg_encoder/left_tick_data", 1, &DGNodeOdometry::callbackEncoderLeft, this);
    sub_encoder_right = nh_dg.subscribe("/dg_encoder/right_tick_data", 1, &DGNodeOdometry::callbackEncoderRight, this);

    // Initialize publishers
    m_publisher_odometry = nh_dg.advertise<nav_msgs::Odometry>("pose", 1, true);

    return true;
}

int DGNodeOdometry::run()
{
    printf("Run dg_odometry...\n");

    ros::Rate loop(m_update_hz);
    while (ros::ok())
    {
        double timestamp = ros::Time::now().toSec();
        if (!runOnce(timestamp)) break;
        ros::spinOnce();
        loop.sleep();
    }

    printf("End dg_odometry...\n");
    nh_dg.shutdown();

    return 0;
}

bool DGNodeOdometry::runOnce(double timestamp)
{
    double wheelbase = 0.588;       // distance between left and right wheel
    double wL = 1;                  // compensation factor for left wheel
    double wR = 1;                  // compensation factor for right wheel

    double pulse_left, pulse_right;    
    m_mutex_left.lock();
    pulse_left = m_pulse_left;
    m_mutex_left.unlock();
    m_mutex_right.lock();
    pulse_right = m_pulse_right;
    m_mutex_right.unlock();

    double dt = timestamp - m_prev_timestamp;
    double dpL = pulse_left - m_prev_pulse_left;
    double dpR = pulse_right - m_prev_pulse_right;

    double dL = wL * dpL * 0.99 / 3800;
    double dR = wR * dpR * 0.99 / 3809;

    double D = (dL + dR) / 2;
    double dtheta = (dR - dL) / wheelbase;

    m_pose.x = m_pose_prev.x + D * cos(m_pose_prev.theta + dtheta/2);
    m_pose.y = m_pose_prev.y + D * sin(m_pose_prev.theta + dtheta/2);
    m_pose.theta = trimRad(m_pose_prev.theta + dtheta);

    nav_msgs::Odometry odo;
    odo.pose.pose.position.x = m_pose.x;
    odo.pose.pose.position.y = m_pose.y;
    odo.pose.pose.orientation.z = m_pose.theta;
    odo.twist.twist.linear.x = D/dt;
    odo.twist.twist.angular.z = dtheta/dt;

    m_publisher_odometry.publish(odo);

    m_pose_prev = m_pose;
    m_prev_timestamp = timestamp;
    m_prev_pulse_left = pulse_left;
    m_prev_pulse_right = pulse_right;

    return true;
}

void DGNodeOdometry::callbackEncoderLeft(const std_msgs::Int32& msg)
{
    try
    {
        m_mutex_left.lock();
        m_pulse_left = msg.data;
        m_mutex_left.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ DGNodeOdometry::callbackEncoderLeft(): %s", e.what());
        return;
    }
}

void DGNodeOdometry::callbackEncoderRight(const std_msgs::Int32& msg)
{
    try
    {
        m_mutex_right.lock();
        m_pulse_right = msg.data;
        m_mutex_right.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ DGNodeOdometry::callbackEncoderRight(): %s", e.what());
        return;
    }
}

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_odometry");
    ros::NodeHandle nh("~");
    DGNodeOdometry dg_node(nh);
    if (dg_node.initialize("dg_ros.yml"))
    {
        dg_node.run();
    }
    return 0;
}
