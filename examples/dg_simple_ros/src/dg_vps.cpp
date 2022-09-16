#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "dg_simple_ros/vps.h"
#include "dg_simple_ros/dg_status.h"
#include "core/basic_type.hpp"
#include "vps/vps_localizer.hpp"
#include "utils/python_embedding.hpp"

#define LOAD_PARAM_VALUE(fn, name_cfg, name_var) \
    if (!(fn)[name_cfg].empty()) (fn)[name_cfg] >> name_var;

using namespace dg;

class DGNodeVPS : public SharedInterface
{
public:
    DGNodeVPS(ros::NodeHandle& nh);
    virtual ~DGNodeVPS();

    bool initialize(std::string config_file);
    int run();
    bool runOnce(double timestamp);
    void close();

protected:
    bool loadConfig(std::string config_file);

    dg::VPSLocalizer m_recognizer;
    bool m_enable_module = true;
    std::string m_srcdir = "/home/dgtest/deepguider/src";
    std::string m_server_ip = "127.0.0.1";  // default: 127.0.0.1 (localhost)
    std::string m_image_server_port = "10000";  // etri: 10000, coex: 10001, bucheon: 10002, etri_indoor: 10003
    std::string m_map_data_path = "data/ETRI/TopoMap_ETRI.csv";
    dg::LatLon m_map_ref_point = dg::LatLon(36.383837659737, 127.367880828442);
    virtual Pose2 getPose(Timestamp* timestamp = nullptr) const;

	double m_vps_load_dbfeat = 0.0;
	double m_vps_save_dbfeat = 0.0;
	double m_vps_gps_accuracy = 0.9;  // Constant gps accuracy related to search range. In streetview image server, download_radius = int(10 + 190*(1-vps_gps_accuracy)) , 1:10m, 0.95:20m, 0.9:29m, 0.79:50, 0.0:200 meters

    cv::Mutex m_cam_mutex;
    cv::Mat m_cam_image;
    dg::Timestamp m_cam_capture_time;

    // Topic subscribers
    std::string m_topic_cam;
    ros::Subscriber sub_image_webcam;
    void callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg);

    dg::LatLon m_dg_latlon;
    double m_dg_latlon_confidence;
    mutable cv::Mutex m_dg_status_mutex;
    ros::Subscriber sub_dg_status;
    void callbackDGStatus(const dg_simple_ros::dg_status::ConstPtr& msg);

    // Topic publishers
    ros::Publisher m_publisher;

    // A node handler
    ros::NodeHandle& nh_dg;
    double m_update_hz = 10;
    bool m_stop_running = false;
};

DGNodeVPS::DGNodeVPS(ros::NodeHandle& nh) : nh_dg(nh)
{
}

DGNodeVPS::~DGNodeVPS()
{
}

bool DGNodeVPS::loadConfig(std::string config_file)
{
    if (config_file.empty())
    {
        return false;
    }

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    // Read Main Options
    cv::FileNode fn = fs.root();
    int enable_mode = -1;
    LOAD_PARAM_VALUE(fn, "enable_vps", enable_mode);
    if(enable_mode == 2) m_enable_module = true;
    else m_enable_module = false;
    LOAD_PARAM_VALUE(fn, "dg_srcdir", m_srcdir);
    LOAD_PARAM_VALUE(fn, "topic_cam", m_topic_cam);
    LOAD_PARAM_VALUE(fn, "ros_update_hz", m_update_hz);

    // Read Topic Setting
    int topic_name_index = -1;
    std::string topicset_tagname;
    std::vector<cv::String> topic_names_set;
    LOAD_PARAM_VALUE(fn, "topic_names_set", topic_names_set);
    LOAD_PARAM_VALUE(fn, "topic_name_index", topic_name_index);
    if (topic_name_index >= 0 && topic_name_index < topic_names_set.size()) topicset_tagname = topic_names_set[topic_name_index];
    if (!topicset_tagname.empty())
    {
        cv::FileNode fn_topic = fn[topicset_tagname];
        if (!fn_topic.empty())
        {
            LOAD_PARAM_VALUE(fn_topic, "topic_cam", m_topic_cam);
        }
    }

    // Read image server settings
    int server_ip_index = -1;
    std::vector<cv::String> server_ip_list;
    LOAD_PARAM_VALUE(fn, "server_ip_list", server_ip_list);
    LOAD_PARAM_VALUE(fn, "server_ip_index", server_ip_index);
    if (server_ip_index >= 0 && server_ip_index < server_ip_list.size()) m_server_ip = server_ip_list[server_ip_index];
    LOAD_PARAM_VALUE(fn, "image_server_port", m_image_server_port);
    LOAD_PARAM_VALUE(fn, "map_data_path", m_map_data_path);
    cv::Vec2d ref_point = cv::Vec2d(m_map_ref_point.lat, m_map_ref_point.lon);
    LOAD_PARAM_VALUE(fn, "map_ref_point_latlon", ref_point);
    m_map_ref_point = dg::LatLon(ref_point[0], ref_point[1]);

	// Read VPS specific Options
    LOAD_PARAM_VALUE(fn, "vps_load_dbfeat", m_vps_load_dbfeat);
    LOAD_PARAM_VALUE(fn, "vps_save_dbfeat", m_vps_save_dbfeat);
    LOAD_PARAM_VALUE(fn, "vps_gps_accuracy", m_vps_gps_accuracy);

    // Read site-specific settings
    int site_index = -1;
    std::string site_tagname;
    std::vector<cv::String> site_names;
    LOAD_PARAM_VALUE(fn, "site_names", site_names);
    LOAD_PARAM_VALUE(fn, "site_index", site_index);
    if (site_index >= 0 && site_index < site_names.size()) site_tagname = site_names[site_index];
    if (!site_tagname.empty())
    {
        cv::FileNode fn_site = fn[site_tagname];
        if (!fn_site.empty())
        {
            LOAD_PARAM_VALUE(fn_site, "image_server_port", m_image_server_port);
            LOAD_PARAM_VALUE(fn_site, "vps_load_dbfeat", m_vps_load_dbfeat);
            LOAD_PARAM_VALUE(fn_site, "vps_save_dbfeat", m_vps_save_dbfeat);
            LOAD_PARAM_VALUE(fn_site, "vps_gps_accuracy", m_vps_gps_accuracy);
            LOAD_PARAM_VALUE(fn_site, "map_data_path", m_map_data_path);
            cv::Vec2d ref_point = cv::Vec2d(m_map_ref_point.lat, m_map_ref_point.lon);
            LOAD_PARAM_VALUE(fn_site, "map_ref_point_latlon", ref_point);
            m_map_ref_point = dg::LatLon(ref_point[0], ref_point[1]);
        }
    }

    return true;
}

void DGNodeVPS::close()
{ 
    m_recognizer.clear();
    close_python_environment();
}

bool DGNodeVPS::initialize(std::string config_file)
{
    printf("Initialize %s...\n", m_recognizer.name());

    // load config
    if(!loadConfig(config_file))
    {
        printf("\tfail to load %s\n", config_file.c_str());
        return false;
    } 
    printf("\tConfiguration %s loaded!\n", config_file.c_str());

    // check if module enabled in the configuration file
    if(!m_enable_module)
    {
        if(!m_enable_module) printf("\t%s disabled in %s\n", m_recognizer.name(), config_file.c_str());
        return false;
    }

    // Initialize subscribers
    if(m_topic_cam.empty())
    {
        printf("\tTopic name for camera input is not specified!\n");
        return false;
    }
    sub_image_webcam = nh_dg.subscribe(m_topic_cam, 1, &DGNodeVPS::callbackImageCompressed, this);
    sub_dg_status = nh_dg.subscribe("/dg_simple_ros/dg_status", 1, &DGNodeVPS::callbackDGStatus, this);

    // Initialize publishers
    m_publisher = nh_dg.advertise<dg_simple_ros::vps>("output", 1, true);

    // initialize python
    bool threaded_run = false;
    if (!init_python_environment("python3", "", threaded_run)) return false;
    printf("\tPython environment initialized!\n");

    // initialize recognizer
    std::string module_path = m_srcdir + "/vps";
    if (!m_recognizer.initialize(this, module_path, m_server_ip, m_image_server_port)) return false;
    printf("\t%s initialized in %.3lf seconds!\n", m_recognizer.name(), m_recognizer.procTime());

    // initialize offline map
    m_map.setReference(m_map_ref_point);
    m_map.load(m_map_data_path.c_str());

    // reset interval variables
    m_cam_image.release();
    m_cam_capture_time = -1;

    return true;
}

int DGNodeVPS::run()
{
    printf("Run %s...\n", m_recognizer.name());

    ros::Rate loop(m_update_hz);
    while (ros::ok() && !m_stop_running)
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec())) break;
        ros::spinOnce();
        loop.sleep();
    }

    // end system
    printf("End %s...\n", m_recognizer.name());
    nh_dg.shutdown();

    return 0;
}

bool DGNodeVPS::runOnce(double timestamp)
{
    dg::Timestamp ts_old = m_recognizer.timestamp();
    cv::Mat cam_image;
    dg::Timestamp capture_time;
    m_cam_mutex.lock();
    if(!m_cam_image.empty() && m_cam_capture_time > ts_old)
    {
        cam_image = m_cam_image.clone();
        capture_time = m_cam_capture_time;
    }    
    m_cam_mutex.unlock();

    dg::Point2 sv_xy;
    dg::Polar2 relative;
    double sv_confidence;
    //if (!cam_image.empty() && m_recognizer.apply(cam_image, capture_time, sv_xy, relative, sv_confidence, m_vps_gps_accuracy, m_vps_load_dbfeat, m_vps_save_dbfeat))
    if (!cam_image.empty() && m_recognizer.apply(cam_image, capture_time, sv_xy, relative, sv_confidence, m_vps_gps_accuracy))
    {
        dg_simple_ros::vps msg;
        msg.id = m_recognizer.getViewID();
        msg.x = sv_xy.x;
        msg.y = sv_xy.y;
        msg.rel_r = relative.lin;
        msg.rel_pi = relative.ang;
        msg.confidence = sv_confidence;
        msg.processingtime = m_recognizer.procTime();
        msg.timestamp = capture_time;
        m_publisher.publish(msg);

        m_recognizer.print();
    }

    return true;
}

void DGNodeVPS::callbackImageCompressed(const sensor_msgs::CompressedImageConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1.0, "Compressed RGB(timestamp: %f [sec]).", msg->header.stamp.toSec());
    try
    {
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

void DGNodeVPS::callbackDGStatus(const dg_simple_ros::dg_status::ConstPtr& msg)
{
    cv::AutoLock lock(m_dg_status_mutex);
    m_stop_running = msg->dg_shutdown;
    m_dg_latlon.lat = msg->lat;
    m_dg_latlon.lon = msg->lon;
    m_dg_latlon_confidence = msg->confidence;
}

Pose2 DGNodeVPS::getPose(Timestamp* timestamp) const
{
    cv::AutoLock lock(m_dg_status_mutex);
    return m_map.toMetric(m_dg_latlon);
}

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_vps");
    ros::NodeHandle nh("~");
    DGNodeVPS dg_node(nh);
    if (dg_node.initialize("dg_ros.yml"))
    {
        dg_node.run();
        dg_node.close();
    }
    return 0;
}
