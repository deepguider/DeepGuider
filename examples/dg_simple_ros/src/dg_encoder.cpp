#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>

#define LOAD_PARAM_VALUE(fn, name_cfg, name_var) \
    if (!(fn)[name_cfg].empty()) (fn)[name_cfg] >> name_var;


class dg_encoder
{
public:
    dg_encoder(ros::NodeHandle& nh);
    virtual ~dg_encoder();

    bool initialize(std::string config_file);
    int run();
    bool runOnce(double timestamp);

protected:
    bool loadConfig(std::string config_file);
    bool m_enable_module = true;

    std_msgs::Int32 right_tick_msg;
    std_msgs::Int32 left_tick_msg;
    double m_left_count = 0;
    double m_right_count = 0;
    double m_initial_time = 0;
    double m_max_timeout = 100;
    void generateSimulationData(double dt);

    // Topic publishers
    ros::Publisher m_publisher_left;
    ros::Publisher m_publisher_right;

    // A node handler
    ros::NodeHandle& nh_dg;
    double m_update_hz = 10;
};

dg_encoder::dg_encoder(ros::NodeHandle& nh) : nh_dg(nh)
{
}

dg_encoder::~dg_encoder()
{
}

void dg_encoder::generateSimulationData(double elapsed_total)
{
    double baseline = 0.588;    // wheel baseline distance
    double interval = 10;       // seconds

    int max_idx = 7;
    double lin_velocity[8] = {1, -1, 0, 0, 1, 1, 1, 1};
    double ang_velocity[8] = {0, 0, CV_PI/4, -CV_PI/2, 0, CV_PI/5, -CV_PI/5, 0};

    int idx = (int)(elapsed_total/10);
    if (idx>max_idx) idx = max_idx;

    m_left_count = 0;
    m_right_count = 0;
    for(int i=0; i<=idx; i++)
    {
        double dt = (i<idx) ? interval : (elapsed_total - interval*i);
        double D = lin_velocity[i] * dt;
        double th = ang_velocity[i] * dt;
        double L = (2*D - baseline * th) / 2;
        double R = (2*D + baseline * th) / 2;

        m_left_count += L * 3800 / 0.99;
        m_right_count += R * 3809 / 0.99;
    }
}

bool dg_encoder::loadConfig(std::string config_file)
{
    if (config_file.empty()) return false;

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    cv::FileNode fn = fs.root();
    LOAD_PARAM_VALUE(fn, "enable_encoder", m_enable_module);
    LOAD_PARAM_VALUE(fn, "encoder_update_hz", m_update_hz);

    return true;
}

bool dg_encoder::initialize(std::string config_file)
{
    printf("Initialize dg_encoder..\n");
    // load config
    loadConfig(config_file);

    // check if module enabled in the configuration file
    if(!m_enable_module)
    {
        printf("\tdg_encoder disabled in %s\n", config_file.c_str());
        return false;
    }

    // Initialize publishers
    m_publisher_left = nh_dg.advertise<std_msgs::Int32>("left_tick_data", 1, true);
    m_publisher_right = nh_dg.advertise<std_msgs::Int32>("right_tick_data", 1, true);

    return true;
}

int dg_encoder::run()
{
    printf("Run dg_encoder...\n");

    ros::Rate loop(m_update_hz);
    m_initial_time = ros::Time::now().toSec();
    while (ros::ok())
    {
        double timestamp = ros::Time::now().toSec();
        if (timestamp - m_initial_time > m_max_timeout) break;
        if (!runOnce(timestamp)) break;
        ros::spinOnce();
        loop.sleep();
    }

    printf("End dg_encoder...\n");
    nh_dg.shutdown();

    return 0;
}

bool dg_encoder::runOnce(double timestamp)
{
    double elapsed_total = timestamp - m_initial_time;
    generateSimulationData(elapsed_total);

    left_tick_msg.data = (int)(m_left_count + 0.5);
    right_tick_msg.data = (int)(m_right_count + 0.5);

    m_publisher_left.publish(left_tick_msg);
    m_publisher_right.publish(right_tick_msg); 

    return true;
}

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_encoder");
    ros::NodeHandle nh("~");
    dg_encoder dg_node(nh);
    if (dg_node.initialize("dg_ros.yml"))
    {
        dg_node.run();
    }
    return 0;
}
