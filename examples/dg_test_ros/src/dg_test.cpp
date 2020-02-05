#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

// Your classs
class YourRunner
{
public:
    YourRunner() : m_wait_sec(0.1) { }

    virtual bool runOnce(double timestamp)
    {
        cout << "Time: " << timestamp << endl;
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
        sub_image_ = nh_.subscribe("/uvc_image_raw/compressed", 1, &YourRunnerNode::callbackImage, this);

        // Initialize publishers
        pub_image_ = nh_.advertise<sensor_msgs::CompressedImage>("image_out", 1, true);
    }

    // A callback function for subscribing a RGB image
    void callbackImage(const sensor_msgs::CompressedImage::ConstPtr& msg)
    {
       cout << "image callback: " << msg->header.stamp.toSec() << endl;
 
        ROS_INFO_THROTTLE(1.0, "A RGB image is subscribed (timestamp: %f [sec]).", msg->header.stamp.toSec());
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
    ros::Subscriber sub_image_;

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
