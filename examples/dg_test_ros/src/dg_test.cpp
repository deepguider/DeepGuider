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

using namespace std;

// Your classs
class YourRunner
{
public:
    YourRunner() : m_wait_sec(0.1) { }

    virtual bool runOnce(double timestamp)
    {
        //cout << "Time: " << timestamp << endl;
        return true;
    }

    virtual bool runForever()
    {
        double timestamp = 0;
        while (true)
        {
            if (!runOnce(timestamp)) break;
            this_thread::sleep_for(chrono::milliseconds(int(m_wait_sec * 1000)));
            timestamp += m_wait_sec;
        }
        return true;
    }

protected:
    double m_wait_sec;
};

// A ROS wrapper class for 'YourRunner'
class YourRunnerNode : public YourRunner
{
public:
    // A constructor
    YourRunnerNode(ros::NodeHandle& nh) : nh_(nh)
    {
        // Read parameters
        nh_.param<double>("wait_sec", m_wait_sec, m_wait_sec);

        // Initialize subscribers
        sub_image_webcam = nh_.subscribe("/uvc_image_raw/compressed", 1, &YourRunnerNode::callbackImageCompressed, this);
        sub_gps_asen = nh_.subscribe("/asen_fix", 1, &YourRunnerNode::callbackGPSAsen, this);
        sub_gps_novatel = nh_.subscribe("/novatel_fix", 1, &YourRunnerNode::callbackGPSNovatel, this);
        sub_imu_xsense = nh_.subscribe("/imu/data", 1, &YourRunnerNode::callbackIMU, this);
        sub_image_realsense_image = nh_.subscribe("/camera/color/image_raw/compressed", 1, &YourRunnerNode::callbackRealsenseImage, this);
        sub_image_realsense_depth = nh_.subscribe("/camera/depth/image_rect_raw/compressed", 1, &YourRunnerNode::callbackRealsenseDepth, this);        

        // Initialize publishers
        pub_image_ = nh_.advertise<sensor_msgs::CompressedImage>("image_out", 1, true);
    }

    // A callback function for subscribing a RGB image
    void callbackImage(const sensor_msgs::Image::ConstPtr& msg)
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
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        ROS_INFO_THROTTLE(1.0, "Compressed RGB(timestamp: %f [sec]).", msg->header.stamp.toSec());
        cv_bridge::CvImagePtr image_ptr;
        cv::Mat image;
        try
        {
            cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("exception @ callbackImageCompressed(): %s", e.what());
            return;
        }
    }    
    
    // A callback function for Realsense compressed RGB
    void callbackRealsenseImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        ROS_INFO_THROTTLE(1.0, "Realsense: RGB (timestamp=%f)", msg->header.stamp.toSec());
        cv::Mat image;
        try
        {
            cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("exception @ callbackRealsenseImage(): %s", e.what());
            return;
        }
    }    
    
    // A callback function for Realsense compressed Depth
    void callbackRealsenseDepth(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        ROS_INFO_THROTTLE(1.0, "Realsense: Depth (timestamp=%f)", msg->header.stamp.toSec());
        cv::Mat image;
        try
        {
            cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("exception @ callbackRealsenseDepth(): %s", e.what());
            return;
        }
    }
            
    // A callback function for subscribing GPS Asen
    void callbackGPSAsen(const sensor_msgs::NavSatFixConstPtr& fix)
    {
        //ROS_INFO_THROTTLE(1.0, "GPS Asen is subscribed (timestamp: %f [sec]).", fix->header.stamp.toSec());
        
        if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
            ROS_DEBUG_THROTTLE(60,"Asen: No fix.");
            return;
        }

        if (fix->header.stamp == ros::Time(0)) {
            return;
        }
        
        double lat = fix->latitude;
        double lon = fix->longitude;
        ROS_INFO_THROTTLE(1.0, "GPS Asen: lat=%f, lon=%f", lat, lon);
    }
    
    
    // A callback function for subscribing GPS Novatel
    void callbackGPSNovatel(const sensor_msgs::NavSatFixConstPtr& fix)
    {
        //ROS_INFO_THROTTLE(1.0, "GPS Novatel is subscribed (timestamp: %f [sec]).", fix->header.stamp.toSec());
        
        if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
            ROS_DEBUG_THROTTLE(60,"Novatel: No fix.");
            return;
        }

        if (fix->header.stamp == ros::Time(0)) {
            return;
        }
        
        double lat = fix->latitude;
        double lon = fix->longitude;
        ROS_INFO_THROTTLE(1.0, "GPS Novatel: lat=%f, lon=%f", lat, lon);
    }    
    
    // A callback function for subscribing IMU
    void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
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
        
        ROS_INFO_THROTTLE(1.0, "IMU: seq=%d, orientation=(%f,%f,%f), angular_veloctiy=(%f,%f,%f), linear_acceleration=(%f,%f,%f)", seq,ori_x,ori_y,ori_z,angvel_x,angvel_y,angvel_z,linacc_x,linacc_y,linacc_z);
    }    

    virtual bool runAndSpin()
    {
        ros::Rate loop(1 / m_wait_sec);
        while (ros::ok())
        {
            ros::Time timestamp = ros::Time::now();
            if (!runOnce(timestamp.toSec())) break;
            // publish topics if necessary
            ros::spinOnce();
            loop.sleep();
        }
        return true;
    }

private:
    // Topic subscribers
    ros::Subscriber sub_image_webcam;
    ros::Subscriber sub_image_realsense_image;
    ros::Subscriber sub_image_realsense_depth;
    ros::Subscriber sub_gps_asen;
    ros::Subscriber sub_gps_novatel;
    ros::Subscriber sub_imu_xsense;

    // Topic publishers
    ros::Publisher pub_image_;

    // A node handler
    ros::NodeHandle& nh_;
};

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_test");
    ros::NodeHandle nh("~");
    YourRunnerNode node(nh);
    node.runAndSpin();
    return 0;
}
