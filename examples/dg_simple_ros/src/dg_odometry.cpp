#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include "opencv2/highgui.hpp"

#define LOAD_PARAM_VALUE(fn, name_cfg, name_var) \
    if (!(fn)[name_cfg].empty()) (fn)[name_cfg] >> name_var;

#define RAD2DEG(x)         ((x) * 180.0 / CV_PI)
#define DEG2RAD(x)         ((x) * CV_PI / 180.0)

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
    bool m_print_trajectory = false;
    bool m_simulated_encoder = false;
    bool m_display_trajectory = false;
    cv::Mat m_traj_map;

    double m_pulse_left = 0;
    double m_pulse_right = 0;
    double m_prev_pulse_left = 0;
    double m_prev_pulse_right = 0;
    double m_prev_timestamp = -1;
    bool m_left_initialized = false;
    bool m_right_initialized = false;
    Pose2 m_pose;
    Pose2 m_pose_prev;
    cv::Mutex m_mutex_left;
    cv::Mutex m_mutex_right;
    
    // Topic subscribers
    ros::Subscriber sub_encoder_left;
    ros::Subscriber sub_encoder_right;
    void callbackEncoderLeft(const std_msgs::Float32& msg);
    void callbackEncoderRight(const std_msgs::Float32& msg);

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
    LOAD_PARAM_VALUE(fn, "odometry_debug_print", m_print_trajectory);
    LOAD_PARAM_VALUE(fn, "odometry_debug_window", m_display_trajectory);
    LOAD_PARAM_VALUE(fn, "enable_simulated_encoder", m_simulated_encoder);

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
    if(m_simulated_encoder)
    {
        sub_encoder_left = nh_dg.subscribe("/dg_encoder/left_tick_data", 10, &DGNodeOdometry::callbackEncoderLeft, this);
        sub_encoder_right = nh_dg.subscribe("/dg_encoder/right_tick_data", 10, &DGNodeOdometry::callbackEncoderRight, this);
    }
    else
    {
        sub_encoder_left = nh_dg.subscribe("/left_tick_data", 10, &DGNodeOdometry::callbackEncoderLeft, this);
        sub_encoder_right = nh_dg.subscribe("/right_tick_data", 10, &DGNodeOdometry::callbackEncoderRight, this);
    }

    // Initialize publishers
    m_publisher_odometry = nh_dg.advertise<nav_msgs::Odometry>("pose", 1, true);

    // Initialize trajectory map
    if(m_display_trajectory)
    {
        int delta = 12;  // meter
        int scale = 50;
        int sz = (int)(delta*scale*2 + 1);
        m_traj_map = cv::Mat::zeros(sz, sz, CV_8UC3);        
    }

    return true;
}

int DGNodeOdometry::run()
{
    printf("Run dg_odometry...\n");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    if(m_display_trajectory)
    {
        cv::namedWindow("odometry", 0);
        cv::resizeWindow("odometry", m_traj_map.cols, m_traj_map.rows);
    }

    ros::Rate loop(m_update_hz);
    m_prev_timestamp = ros::Time::now().toSec();
    while (ros::ok())
    {
        if (!runOnce(ros::Time::now().toSec())) break;
        loop.sleep();
    }

    if(m_display_trajectory)
    {
        cv::destroyAllWindows();
    }

    printf("End dg_odometry...\n");
    nh_dg.shutdown();

    return 0;
}

bool DGNodeOdometry::runOnce(double timestamp)
{
    double wheelbase = 0.588;       // distance between left and right wheel
    //double wL = 0.9799845;                // compensation factor for left wheel
    //double wR = 0.9913012;                // compensation factor for right wheel

    double wL = 0.9757583432906607;                // compensation factor for left wheel
    double wR = 0.9785017433324273;      // compensation factor for right wheel

    double pulse_left, pulse_right;
    bool left_initialized, right_initialized;
    m_mutex_left.lock();
    pulse_left = m_pulse_left;
    left_initialized = m_left_initialized;
    m_mutex_left.unlock();
    m_mutex_right.lock();
    pulse_right = m_pulse_right;
    right_initialized = m_right_initialized;
    m_mutex_right.unlock();

    double dpL = (left_initialized) ? pulse_left - m_prev_pulse_left : 0;
    double dpR = (right_initialized) ? pulse_right - m_prev_pulse_right : 0;

    double dL = wL * dpL * 0.99 / 3485;
    double dR = wR * dpR * 0.99 / 3567;

    double D = (dL + dR) / 2;
    double dtheta = (dR - dL) / wheelbase;

    m_pose.x = m_pose_prev.x + D * cos(m_pose_prev.theta + dtheta/2);
    m_pose.y = m_pose_prev.y + D * sin(m_pose_prev.theta + dtheta/2);
    m_pose.theta = trimRad(m_pose_prev.theta + dtheta);

    nav_msgs::Odometry odo;
    odo.header.stamp.fromSec(timestamp);
    odo.pose.pose.position.x = m_pose.x;
    odo.pose.pose.position.y = m_pose.y;
    odo.pose.pose.orientation.z = m_pose.theta;

    double dt = timestamp - m_prev_timestamp;
    if(dt>0) odo.twist.twist.linear.x = D/dt;
    if(dt>0) odo.twist.twist.angular.z = dtheta/dt;

    m_publisher_odometry.publish(odo);

    m_pose_prev = m_pose;
    m_prev_timestamp = timestamp;
    m_prev_pulse_left = pulse_left;
    m_prev_pulse_right = pulse_right;

    if(m_print_trajectory)
    {
        printf("odo:x=%.3lf, y=%.3lf, theta=%.2lf\n", m_pose.x, m_pose.y, RAD2DEG(m_pose.theta));
        printf("    v=%.3lf, w=%.2lf\n", D/dt, RAD2DEG(dtheta/dt));
    }

    if(m_display_trajectory)
    {
        int delta = 12;  // meter
        int scale = 50;
        int sz = (int)(delta*scale*2 + 1);
        cv::Scalar grid_color(50, 50, 50);
        if(m_traj_map.empty()) m_traj_map = cv::Mat::zeros(sz, sz, CV_8UC3);
        else m_traj_map = 0;

        int w = m_traj_map.cols;
        int h = m_traj_map.rows;
        int cx = m_traj_map.cols/2;
        int cy = m_traj_map.rows/2;
        for(int i=-delta; i<=delta; i++)
        {
            int x = cx + (int)(i*scale);
            line(m_traj_map, cv::Point(x,0), cv::Point(x, h), grid_color);
            int y = cy + (int)(i*scale);
            line(m_traj_map, cv::Point(0,y), cv::Point(w, y), grid_color);
        }    double wL = 0.9799845;                // compensation factor for left wheel
    double wR = 0.9913012;                // compensation factor for right wheel

        line(m_traj_map, cv::Point(cx,0), cv::Point(cx, h), cv::Scalar(100,100,100));
        line(m_traj_map, cv::Point(0,cy), cv::Point(w, cy), cv::Scalar(100,100,100));

        double ix = cy - scale*m_pose.y;      // odometry to image point
        double iy = cx - scale*m_pose.x;      // odometry to image point
        cv::Point center = cv::Point2d(ix, iy) + cv::Point2d(0.5, 0.5);
        int hr = 15;               // heading arrow length
        int heading_x = (int)(ix - hr*sin(m_pose.theta) + 0.5);
        int heading_y = (int)(iy - hr*cos(m_pose.theta) + 0.5);
        int robot_radius = 10;
        cv::Scalar robot_color(0,255,0);

        cv::circle(m_traj_map, center, robot_radius, robot_color, 1);
        cv::line(m_traj_map, center, cv::Point(heading_x, heading_y), robot_color, 1);
        cv::imshow("odometry", m_traj_map);
        int key = cv::waitKey(1);
        if(key == 27) return false;
    }

    return true;
}

void DGNodeOdometry::callbackEncoderLeft(const std_msgs::Float32& msg)
{
    try
    {
        m_mutex_left.lock();
        m_pulse_left += msg.data;
        if(!m_left_initialized)
        {
            m_prev_pulse_left = m_pulse_left;
            m_left_initialized = true;
        }
        m_mutex_left.unlock();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("exception @ DGNodeOdometry::callbackEncoderLeft(): %s", e.what());
        return;
    }
}

void DGNodeOdometry::callbackEncoderRight(const std_msgs::Float32& msg)
{
    try
    {
        m_mutex_right.lock();
        m_pulse_right += msg.data;
        if(!m_right_initialized)
        {
            m_prev_pulse_right = m_pulse_right;
            m_right_initialized = true;
        }
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
