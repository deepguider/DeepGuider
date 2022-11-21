#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include "dg_simple_ros.cpp"
#include "dg_simple_ros/guidance.h"
#include "dg_simple_ros/action.h"
#include "dg_simple_ros/dg_status.h"
#include "utils/utm_converter.hpp"

class DGRobot : public DeepGuiderROS
{
public:
    DGRobot(ros::NodeHandle& nh);
    virtual ~DGRobot();

    bool initialize(std::string config_file);
    int run();
    bool runOnce(double timestamp);

protected:
    virtual int readParam(const cv::FileNode& fn);
    int readRobotParam(const cv::FileNode& fn);

    // Topic subscribers 
    ros::Subscriber sub_draw_robot;    
    ros::Subscriber sub_dg_status; 
    ros::Subscriber sub_robot_status;
    ros::Subscriber sub_robot_pose;
    ros::Subscriber sub_robot_heading;
    ros::Subscriber sub_robot_map;   
    void callbackDrawRobot(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callbackDGStatus(const dg_simple_ros::dg_status::ConstPtr& msg);
    void callbackRobotStatus(const std_msgs::String::ConstPtr& msg);
    void callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callbackRobotMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // Topic publishers (sensor data)
    ros::Publisher pub_subgoal;
    void publishSubGoal();

    bool m_stop_running = false;
    double m_min_goal_dist = 3.0;
    Pose2 m_prev_node_metric;
    Pose2 m_cur_node_metric;
    Pose2 m_prev_robot_metric;
    ID m_prev_node_id = 0; 
    ID m_cur_head_node_id = 0; 
    
    //Robot parameters
    bool m_first_robot_pose = true;
    cv::Mat m_robotmap_image;
    std::string m_robotmap_path = "data/Bucheon/bucheon_220922/occumap_bucheon.png";
    dg::LatLon m_dx_map_ref_latlon = dg::LatLon(37.5177542, 126.7651744);
    dg::Point2 m_dx_map_ref_pixel = dg::Point2(715, 650);
    double m_dx_map_meter_per_pixel = 0.1;
    double m_dx_map_rotation_radian = -0.8;
    double m_robotmap_scale = 10.0;
    dg::Point2 m_dx_map_origin_pixel = dg::Point2(345, 1110);
    bool m_pub_flag = false;
    GuidanceManager::RobotStatus m_prev_state = GuidanceManager::RobotStatus::READY;
    ros::Time m_begin_time = ros::Time::now();

    // DX 로봇 부천 지도 origin 계산
    dg::LatLon m_dx_map_origin_latlon;
    dg::LatLon m_dg_map_origin_latlon; // 딥가이더 부천 원점 (dg_simple.yml 참조)
    dg::UTMConverter m_dx_converter;
    dg::UTMConverter m_dg_converter;
    Point2 cvtDg2Dx(const Point2& dg_metric) const;
    Point2 cvtDx2Dg(const Point2& dx_metric) const;
    Point2 cvtMetric2Pixel(const Point2& val) const;
    Point2 cvtPixel2Metric(const Point2& px) const;

    //robot related functions
    std::vector<Point2> m_undrivable_points;
    void initialize_DG_DX_conversion();
    bool makeSubgoal(Pose2& pub_pose);
    bool makeSubgoal2(Point2& pub_pose, Point2& goal_px);
    bool makeSubgoal3(Pose2& pub_pose);
    bool makeSubgoal4(Point2& pub_pose, Point2& node_dx_pixel);

    geometry_msgs::PoseStamped makeRosPubPoseMsg(ID nid, Point2 xy);
    bool findExtendedDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2& result_metric);
    bool findDrivableinLine(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2& result_metric);
    bool reselectDrivablePoint(Point2& pub_pose);
    bool drawSubgoal(Point2& pub_pose);

    int m_video_recording_fps = 15;
    cv::Size m_framesize = cv::Size(3500, 2500);
    cv::Size m_framesize_crop = cv::Size(800, 800);
    int m_fourcc = cv::VideoWriter::fourcc('A', 'V', 'C', '1');   
    cv::VideoWriter m_video_gui;
    cv::VideoWriter m_video_crop;
};

DGRobot::DGRobot(ros::NodeHandle& nh) : DeepGuiderROS(nh)
{
}

DGRobot::~DGRobot()
{    
}

int DGRobot::readParam(const cv::FileNode& fn)
{
    int n_read = DeepGuiderROS::readParam(fn);
    n_read += readRobotParam(fn);
    return n_read;
}

int DGRobot::readRobotParam(const cv::FileNode& fn)
{
    int n_read = 0;

    //read robot configuration
    CX_LOAD_PARAM_COUNT(fn, "robotmap_path", m_robotmap_path, n_read);
    cv::Vec2d dx_ref_point = cv::Vec2d(m_dx_map_ref_latlon.lat, m_dx_map_ref_latlon.lon);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_ref_point_latlon", dx_ref_point, n_read);
    m_dx_map_ref_latlon = dg::LatLon(dx_ref_point[0], dx_ref_point[1]);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_ref_point_pixel", m_dx_map_ref_pixel, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_ref_point_pixel_per_meter", m_dx_map_meter_per_pixel, n_read);
    double robotmap_rotation;
    CX_LOAD_PARAM_COUNT(fn, "robotmap_rotation", robotmap_rotation, n_read);
    m_dx_map_rotation_radian = cx::cvtDeg2Rad(robotmap_rotation); // 로봇 지도 rotation
    CX_LOAD_PARAM_COUNT(fn, "robotmap_scale", m_robotmap_scale, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robotmap_origin_pixel", m_dx_map_origin_pixel, n_read);
    cv::Vec2d dg_ref_point = cv::Vec2d(m_map_ref_point.lat, m_map_ref_point.lon);
    CX_LOAD_PARAM_COUNT(fn, "map_ref_point_latlon", dg_ref_point, n_read);
    m_dg_map_origin_latlon = dg::LatLon(dg_ref_point[0], dg_ref_point[1]);

    int robot_map_index = -1;
    std::string robot_site_tagname;
    std::vector<cv::String> robot_map_set;
    CX_LOAD_PARAM_COUNT(fn, "robot_map_set", robot_map_set, n_read);
    CX_LOAD_PARAM_COUNT(fn, "robot_map_index", robot_map_index, n_read);
    if (robot_map_index >= 0 && robot_map_index < robot_map_set.size())
        robot_site_tagname = robot_map_set[robot_map_index];

    // Read Robot Setting
    if (!robot_site_tagname.empty())
    {
        cv::FileNode fn_robot = fn[robot_site_tagname];
        if (!fn_robot.empty())
        {
            n_read += readRobotParam(fn_robot);
        }
    }
    m_guider.setRobotMap(robot_site_tagname);
    m_guider.setRobotUsage(m_topicset_tagname);
    printf("topicset_tagname: %s\n", m_topicset_tagname.c_str());
    printf("m_dx_map_ref_pixel.x: %f, m_dx_map_ref_pixel.y: %f\n", m_dx_map_ref_pixel.x, m_dx_map_ref_pixel.y);
    printf("m_dx_map_origin_pixel: %f, %f\n", m_dx_map_origin_pixel.x, m_dx_map_origin_pixel.y);
    printf("m_dg_map_origin_latlon: %f, %f\n", m_dg_map_origin_latlon.lat, m_dg_map_origin_latlon.lon);
    printf("robotmap_rotation: %f\n", robotmap_rotation);

    return n_read;
}

bool DGRobot::initialize(std::string config_file)
{
    printf("Initialize dg_robot..\n");

    // Initialize DeeGuiderRos system
    bool ok = DeepGuiderROS::initialize(config_file);
    if(!ok) return false;

    // Initialize subscribers
    sub_dg_status = nh_dg.subscribe("/dg_simple_ros/dg_status", 1, &DGRobot::callbackDGStatus, this);
    sub_robot_pose = nh_dg.subscribe("/mcl3d/current/pose", 1, &DGRobot::callbackRobotPose, this);
    sub_draw_robot = nh_dg.subscribe("/mcl3d/current/pose", 1, &DGRobot::callbackDrawRobot, this);
    // sub_robot_status = nh_dg.subscribe("/keti_robot_state", 1, &DGRobot::callbackRobotStatus, this); //run_manual
    sub_robot_status = nh_dg.subscribe("/keti_robot/state", 1, &DGRobot::callbackRobotStatus, this);  //run_auto
    sub_robot_map = nh_dg.subscribe("/deepmerge/map/occu", 1, &DGRobot::callbackRobotMap, this);

    // Initialize deepguider publishers
    pub_subgoal = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_subgoal", 1, true);
    
    // Initialize robot parameters
    initialize_DG_DX_conversion();

    m_video_gui.open("../../../online_map.avi", m_fourcc, m_video_recording_fps, m_framesize);
    m_video_crop.open("../../../online_crop.avi", m_fourcc, m_video_recording_fps, m_framesize_crop);

    return true;
}

int DGRobot::run()
{
    printf("Run Deepguider with Robot...\n");

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
    m_video_gui.release();
    m_video_crop.release();
    printf("\trecording closed\n");
    cv::destroyWindow(m_winname);
    printf("\tgui window destroyed\n");
    nh_dg.shutdown();
    printf("\tros shutdowned\n");
    printf("all done!\n");

    return 0;
}

bool DGRobot::runOnce(double timestamp)
{    
    bool ok = DeepGuiderROS::runOnce(timestamp);
    if(!ok) return false;

    publishSubGoal();

    return true;
}

void DGRobot::callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    dg::Timestamp timestamp = msg->header.stamp.toSec();
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    double ori_x = msg->pose.orientation.x;
    double ori_y = msg->pose.orientation.y;
    double ori_z = msg->pose.orientation.z;
    double ori_w = msg->pose.orientation.w;
    cv::Point3d euler = cx::cvtQuat2EulerAng(ori_w, ori_x, ori_y, ori_z);
    double theta = euler.z;
    ROS_INFO_THROTTLE(1.0, "Robot: %f,%f, deg:%f", x, y, cx::cvtRad2Deg(theta));

    // Reset deepguider pose by first arrived robot pose
    if(m_first_robot_pose)
    {
        m_localizer->setPose(dg::Pose2(x, y, theta), timestamp);
        m_first_robot_pose = false;
    }
    // Use robot pose as odometry data
    else if (m_enable_odometry)
    {
        procOdometryData(x, y, theta, timestamp);
    }

    m_guider_mutex.lock();
    m_guider.m_robot_pose = dg::Pose2(x, y, theta);
    m_guider_mutex.unlock();

}

void DGRobot::callbackDrawRobot(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    dg::Point2 robot_dg = cvtDx2Dg(cv::Point2d(x,y));
    cv::circle(m_map_image, robot_dg, 3, cv::Vec3b(0, 100, 255));
}

void DGRobot::callbackRobotStatus(const std_msgs::String::ConstPtr& msg)
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
    else if (!strcmp(str, "no_path"))
    {
        m_guider.setRobotStatus(GuidanceManager::RobotStatus::NO_PATH);
    }
}

void DGRobot::callbackRobotMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    int size_x = map->info.width;
    int size_y = map->info.height;

    ROS_INFO_THROTTLE(1.0, "callbackRobotMap: Robot map size x: %d, y: %d", size_x, size_y);
    
    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

    m_robotmap_mutex.lock();

    Point2 new_origin_pixel;
    new_origin_pixel.x = -map->info.origin.position.x * m_robotmap_scale;
    new_origin_pixel.y = size_y + map->info.origin.position.y * m_robotmap_scale;
    if (abs(m_dx_map_origin_pixel.x - new_origin_pixel.x) >= 1 || abs(m_dx_map_origin_pixel.y - new_origin_pixel.y) >= 1)
    {
        m_dx_map_origin_pixel = new_origin_pixel;
    }
    
    cv::Mat image(size_y, size_x, CV_8UC1);
    for (int i = 0; i < size_y; i++)
    {
        for (int j = 0; j < size_x; j++)
        {
            int index = i*size_x + j;
            if (map->data[index] <= 15)
            {
                image.at<uchar>(i,j) = 255 - map->data[index]*5;
            }
            else
            {
                image.at<uchar>(i,j) = 0;
            }            
        }
    }

    m_robotmap_image = image;  
    m_robotmap_mutex.unlock();     
}

void DGRobot::publishSubGoal()
{
    if (!m_guider.isGuidanceInitialized())
        return;

    Pose2 pub_pose;
    GuidanceManager::RobotStatus cur_state = m_guider.getRobotStatus();
    if (cur_state == GuidanceManager::RobotStatus::ARRIVED_NODE || cur_state == GuidanceManager::RobotStatus::ARRIVED_GOAL 
    || cur_state == GuidanceManager::RobotStatus::READY || cur_state == GuidanceManager::RobotStatus::NO_PATH)
    {            
        if(!m_pub_flag)
        {
            // if (makeSubgoal(pub_pose))
            if (makeSubgoal3(pub_pose))
            {
                geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(m_cur_head_node_id, pub_pose);
                pub_subgoal.publish(rosps);
                m_guider.m_subgoal_pose = pub_pose;
                ROS_INFO("==============================================================\n");
                ROS_INFO("SubGoal published!: %f, %f<=====================", pub_pose.x, pub_pose.y);
                m_begin_time = ros::Time::now();
                m_pub_flag = true;
                return;
            }
        }
        else
        {
            ros::Time cur_time = ros::Time::now();
            ros::Duration duration = cur_time - m_begin_time;
            if (duration > ros::Duration(5.0))
            {
                ROS_INFO("Duration seconds: %d", duration.sec);
                if (cur_state == GuidanceManager::RobotStatus::NO_PATH) //if robot cannot generate path with the subgoal
                {
                    if (reselectDrivablePoint(pub_pose))
                    {
                        ROS_INFO("reselectDrivablePoint!: %f, %f", pub_pose.x, pub_pose.y);
                        m_guider.m_subgoal_pose = pub_pose;
                    }                        
                }
                pub_pose = m_guider.m_subgoal_pose;
                geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(m_cur_head_node_id, pub_pose);
                pub_subgoal.publish(rosps);
                ROS_INFO("==============================================================\n");
                ROS_INFO("SubGoal published, again!: %f, %f<=====================", pub_pose.x, pub_pose.y);
                m_begin_time = ros::Time::now();
            }
        }            
    }
    else //cur_state == GuidanceManager::RobotStatus::RUN_MANUAL || cur_state == GuidanceManager::RobotStatus::RUN_AUTO)
    {
        m_pub_flag = false;
    }    
}

bool DGRobot::makeSubgoal(Pose2& pub_pose)
{
    GuidanceManager::ExtendedPathElement cur_guide = m_guider.getNextExtendedPath();
    GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();

    Pose2 cur_node_metric = Point2(cur_guide);
    Pose2 next_node_metric = Point2(next_guide);
    ROS_INFO("[makeSubgoal] Heading %zd, node_metric.x: %f, y:%f",next_guide.cur_node_id, next_node_metric.x, next_node_metric.y);
    pub_pose = next_node_metric;

    //load robot info 
    m_robotmap_mutex.lock();
    cv::Mat onlinemap = m_robotmap_image;
    Pose2 robot_dx_metric = m_guider.m_robot_pose;
    m_robotmap_mutex.unlock();

    cv::Mat robotmap;
    cv::flip(onlinemap, robotmap,0);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO("No occumap");
        robotmap = cv::imread(m_robotmap_path);
    }

    if (robotmap.empty())
        return false;

    Pose2 dg_pose = m_localizer->getPose();
    Pose2 dg_px = cvtMetric2Pixel(dg_pose);
    Point2 robot_px = cvtMetric2Pixel(robot_dx_metric);
    Point2 node_px_dx = cvtMetric2Pixel(next_node_metric);

    //save image
    cv::Mat colormap;
    if (robotmap.channels() == 1)
        cv::cvtColor(robotmap, colormap, cv::COLOR_GRAY2BGR); 

    Point2 node_px_new, node_px_moved;
    ROS_INFO("[makeSubgoal] %d", m_guider.getCurGuideIdx());
    //here, select new goal
    if (m_cur_head_node_id != next_guide.cur_node_id)
    {
        //check wheter pub_pose is undrivable find drivable point  
        cv::Mat img_erode;
        erode(robotmap, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);
        if (img_erode.at<uchar>((int)node_px_dx.y, (int)node_px_dx.x) < 200) //if node is in black area
        {
            ROS_INFO("Current node is in dark area");
            if(findDrivableinLine(img_erode, robot_px, node_px_dx, pub_pose))
            {
                node_px_moved = cvtMetric2Pixel(pub_pose);
            }
            else          
            {
                if(findExtendedDrivablePoint(img_erode, robot_px, node_px_dx, pub_pose))
                {
                    node_px_moved = cvtMetric2Pixel(pub_pose);
                }
                else
                {
                    ROS_INFO("Tried all. Failed to provide drivable goal.");
                    return false;
                }                
            } 
        }

        ROS_INFO("Found subggoal: <%f, %f>", pub_pose.x, pub_pose.y);
        m_prev_node_metric = m_cur_node_metric;
        m_cur_node_metric = next_node_metric;
        m_prev_node_id = m_cur_head_node_id;
        m_cur_head_node_id = next_guide.cur_node_id;
        m_prev_robot_metric = robot_dx_metric;
                
        ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        ///save image 
        cv::circle(colormap, robot_px, 10, cv::Vec3b(0, 255, 0), 2);
        cv::line(colormap, robot_px, node_px_dx, cv::Vec3b(0, 0, 255), 2);
        cv::circle(colormap, dg_px, 10, cv::Vec3b(255, 0, 0), 2);
        cv::circle(colormap, node_px_dx, 10, cv::Vec3b(0, 0, 255), 2);
        cv::circle(colormap, node_px_moved, 10, cv::Vec3b(0, 255, 255), 2);
        Point2 heading;
        heading.x = robot_px.x + 20 * cos(-robot_dx_metric.theta);
        heading.y = robot_px.y + 20 * sin(-robot_dx_metric.theta);
        cv::line(colormap, robot_px, heading, cv::Vec3b(0, 255, 0), 2);
        imwrite("../../../test_image.png", colormap);
    }

    return true;
}

bool DGRobot::makeSubgoal3(Pose2& pub_pose)
{ 
    GuidanceManager::ExtendedPathElement cur_guide = m_guider.getNextExtendedPath();
    GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();

    Pose2 cur_node_metric = Point2(cur_guide);
    Pose2 next_node_metric = Point2(next_guide);
    ROS_INFO("[makeSubgoal3] Heading %zd, node_metric.x: %f, y:%f",next_guide.cur_node_id, next_node_metric.x, next_node_metric.y);
    pub_pose = next_node_metric;

    //load robot info 
    m_robotmap_mutex.lock();
    cv::Mat onlinemap = m_robotmap_image;
    Pose2 robot_dx_metric = m_guider.m_robot_pose;
    m_robotmap_mutex.unlock();

    cv::Mat robotmap;
    cv::flip(onlinemap, robotmap,0);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO("No occumap");
        robotmap = cv::imread(m_robotmap_path);
    }

    if (robotmap.empty())
        return false;

    Pose2 dg_pose = m_localizer->getPose();
    Pose2 dg_px = cvtMetric2Pixel(dg_pose);
    Point2 robot_px = cvtMetric2Pixel(robot_dx_metric);
    Point2 node_px_dx = cvtMetric2Pixel(next_node_metric);

    //save image
    cv::Mat colormap;
    if (robotmap.channels() == 1)
        cv::cvtColor(robotmap, colormap, cv::COLOR_GRAY2BGR); 

    Point2 node_px_new, node_px_moved, node_px_dg;
    ROS_INFO("[makeSubgoal3] %d", m_guider.getCurGuideIdx());
    //here, select new goal
    if (m_cur_head_node_id != next_guide.cur_node_id)
    {
        if(m_prev_node_id != 0) //initial start
        {            
            ROS_INFO("[makeSubgoal3] prev_id: %zd, cur_id: %zd, next_id: %zd", m_prev_node_id, m_cur_head_node_id, next_guide.cur_node_id);
            int diff_deg = m_guider.getDegree(m_prev_node_metric, m_cur_node_metric, next_node_metric);
            double diff_dist = norm(next_node_metric-m_cur_node_metric);

            //give calculated r, theta point based on robot pose, and find near subgoal
            ROS_INFO("[makeSubgoal3] robot_dx_metric: <%f, %f>, m_prev_robot_metric <%f, %f>", 
            robot_dx_metric.x, robot_dx_metric.y, m_prev_robot_metric.x, m_prev_robot_metric.y);
            double base_theta1 = robot_dx_metric.theta;
            Pose2 robot_diff = robot_dx_metric - m_prev_robot_metric;
            double base_theta2 = atan2(robot_diff.y, robot_diff.x);
            ROS_INFO("[makeSubgoal3] robot_diff theta: %f, robot.theta: %f, +theta:: %d", cx::cvtRad2Deg(base_theta1), cx::cvtRad2Deg(base_theta2), diff_deg);
            
            pub_pose.theta = base_theta2 + cx::cvtDeg2Rad(diff_deg);
            pub_pose.x = robot_dx_metric.x + diff_dist * cos(pub_pose.theta);
            pub_pose.y = robot_dx_metric.y + diff_dist * sin(pub_pose.theta);

        }

        //check wheter pub_pose is undrivable find drivable point  
        node_px_dg = cvtMetric2Pixel(pub_pose);
        cv::Mat img_erode;
        erode(robotmap, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);
        if (img_erode.at<uchar>((int)node_px_dg.y, (int)node_px_dg.x) < 200) //if node is in black area
        {
            ROS_INFO("Current node is in dark area");
            if(findDrivableinLine(img_erode, robot_px, node_px_dg, pub_pose))
            {
                node_px_moved = cvtMetric2Pixel(pub_pose);
            }
            else          
            {
                if(findExtendedDrivablePoint(img_erode, robot_px, node_px_dg, pub_pose))
                {
                    node_px_moved = cvtMetric2Pixel(pub_pose);
                }
                else
                {
                    ROS_INFO("Tried all. Failed to provide drivable goal.");
                    return false;
                }                
            } 
        }

        ROS_INFO("Found subggoal: <%f, %f>", pub_pose.x, pub_pose.y);
        m_prev_node_metric = m_cur_node_metric;
        m_cur_node_metric = next_node_metric;
        m_prev_node_id = m_cur_head_node_id;
        m_cur_head_node_id = next_guide.cur_node_id;
        m_prev_robot_metric = robot_dx_metric;
                
        ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        ///save image 
        cv::circle(colormap, robot_px, 10, cv::Vec3b(0, 255, 0), 2);
        cv::line(colormap, robot_px, node_px_dx, cv::Vec3b(0, 0, 255), 2);
        cv::circle(colormap, dg_px, 10, cv::Vec3b(255, 0, 0), 2);
        cv::circle(colormap, node_px_dx, 10, cv::Vec3b(0, 0, 255), 2);
        cv::circle(colormap, node_px_dg, 10, cv::Vec3b(0, 100, 255), 2);
        cv::line(colormap, robot_px, node_px_dg, cv::Vec3b(0, 100, 255), 2);
        cv::circle(colormap, node_px_moved, 10, cv::Vec3b(0, 255, 255), 2);
        Point2 heading;
        heading.x = robot_px.x + 20 * cos(-robot_dx_metric.theta);
        heading.y = robot_px.y + 20 * sin(-robot_dx_metric.theta);
        cv::line(colormap, robot_px, heading, cv::Vec3b(0, 255, 0), 2);
        imwrite("../../../test_image.png", colormap);
    }


    //record image   
    // cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);  
    // cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
    // colormap.copyTo(roi);
    // m_video_gui << videoFrame;
    // cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
    // int x = robot_px.x-400; 
    // int y = robot_px.y-400;
    // if (x+800 >= colormap.cols) x = colormap.cols - 800 - 1;
    // if (x<=1) x = 1;
    // if (y+800 >= colormap.rows) y = colormap.rows - 800 - 1;
    // if (y<=1) y = 1;
    
    // cv::Mat roicrop(colormap, cv::Rect(x, y, 800, 800));
    // roicrop.copyTo(videoFrameCrop);
    // m_video_crop << videoFrameCrop;
    // imwrite("../../../online_crop.png", videoFrameCrop);
    return true;
}

bool DGRobot::findDrivableinLine(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2& result_metric)
{
    if (image.empty())
        return false;

    Point2 ro2no = robot_px - node_px;
    double jump_deg = atan2(ro2no.y, ro2no.x);
    double jump_dist = norm(ro2no);
    int jump_step = m_robotmap_scale;
    int max_count = jump_dist / jump_step;
    
    Point2 jump_px, dx_metric;
    for (int i = 0; i < max_count; i++)
    {
        jump_px.x = node_px.x + i * jump_step * cos(jump_deg);
        jump_px.y = node_px.y + i * jump_step * sin(jump_deg);
        if (image.at<uchar>((int)jump_px.y, (int)jump_px.x) > 200)
        {
            ROS_INFO("Found drivable point in Line-img_erode-%d <%d, %d>: %d\n", i, (int) jump_px.x, (int) jump_px.y, image.at<uchar>((int)jump_px.y, (int)jump_px.x));
            break;
        }
    }

    double dist = norm(jump_px - robot_px)/m_robotmap_scale;
    if (dist < m_min_goal_dist) //no drivble area on the line
    {        
        ROS_INFO("subgoal is to near: %f, %f", jump_px.x, jump_px.y);
        return false;
    }

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    result_metric = cvtPixel2Metric(jump_px);
    return true;
}

bool DGRobot::findExtendedDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2& result_metric)
{
    if (image.empty())
        return false;

    //save to image
    cv::Mat img_color;
    image.copyTo(img_color);
    cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2BGR);

    Point2 px_diff = node_px - robot_px;
    double base_rad = atan2(px_diff.y, px_diff.x);
    double max_dist = norm(px_diff);
    int jump_step = (int)m_robotmap_scale;
    int max_count = (int)max_dist / jump_step;
    ROS_INFO("max_count: %d, max_dist: %f", max_count, max_dist);
        
    //search -30~30 drivable point
    double jump_rad, dist;
    Point2 jump_px, prev_px;
    std::map<int, Point2, greater<double>> candidate1;
    std::map<double, int, greater<double>> candidate2;
    std::map<int, int> candidate3;
    for (int deg = -30; deg < 30; deg++)
    {
        jump_rad = base_rad + cx::cvtDeg2Rad(deg);  
        ROS_INFO("Searching %f deg, base_deg: %f", cx::cvtRad2Deg(jump_rad), cx::cvtRad2Deg(base_rad));
        //find drivable area on the based on robot_px
        for (int i = 0; i < max_count; i++)
        {
            jump_px.x = robot_px.x + i * jump_step * cos(jump_rad);
            jump_px.y = robot_px.y + i * jump_step * sin(jump_rad);
            if (image.at<uchar>((int)jump_px.y, (int)jump_px.x) < 200)
            {
                dist = norm(prev_px - robot_px)/m_robotmap_scale;
                if (dist > 0)
                {
                    cv::circle(img_color, prev_px, 2, cv::Vec3b(0, 255, 255), 2);
                    cv::line(img_color, robot_px, prev_px, cv::Vec3b(0, 255, 255), 2);

                    candidate1.insert(make_pair(deg, prev_px));
                    candidate2.insert(make_pair(dist, deg));
                    ROS_INFO("Find drivable point dist: %d, %f, <%f, %f>", deg, dist, prev_px.x, prev_px.y);
                }
                i = max_count;
            }        
            prev_px = jump_px;        
        }
    }

    if (candidate1.size() == 0)
    {
        ROS_INFO("Cannot find goal. all dark. Provide drivable map.");
        cv::imwrite("../../../finddrivable.png", img_color);
        return false;
    }

    for (auto itr = candidate2.begin(); itr != candidate2.end(); itr++)
    {
        dist = itr->first;
        int deg = itr->second;
        if(dist > m_min_goal_dist)
        {
            candidate3.insert(make_pair(abs(deg), deg));
            ROS_INFO("Find drivable point abs(deg)%d, deg: %d, dist: %f",abs(deg), deg, dist);
        }        
    }   

    int deg;
    //if there is no over 3m, select longest  
    if (candidate3.empty())
    {
        auto itr2 = candidate2.begin();
        deg = itr2->second;        
        ROS_INFO("Select among longest dist: %f, deg:%d", itr2->first, itr2->second);
    }
    else
    {
        auto itr3 = candidate3.begin();
        deg = itr3->second;
        ROS_INFO("Select smallest deg: %d, deg:%d", itr3->first, itr3->second);
    }
       
    //find corresponding deg-pixel;
    for (auto itr1 = candidate1.begin(); itr1 !=  candidate1.end(); itr1++)
    {
        if (itr1->first == deg)
        {
            jump_px = itr1->second;
            ROS_INFO("Find drivable point: <%f, %f>", jump_px.x, jump_px.y);            

            cv::line(img_color, robot_px, node_px, cv::Vec3b(0, 0, 255), 2);
            cv::circle(img_color, node_px, 10, cv::Vec3b(0, 0, 255), 2);
            cv::circle(img_color, robot_px, 10, cv::Vec3b(0, 255, 0), 2);

            // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
            result_metric = cvtPixel2Metric(jump_px);

            cv::circle(img_color, jump_px, 10, cv::Vec3b(0, 255, 255), 2);
            cv::imwrite("../../../finddrivable.png", img_color);

            return true;
        } 
    }

    return false;

}

bool DGRobot::reselectDrivablePoint(Point2& pub_pose)
{
    m_robotmap_mutex.lock();
    cv::Mat onlinemap = m_robotmap_image;
    Pose2 robot_dx_metric = m_guider.m_robot_pose;
    Point2 undrivable_pose = m_guider.m_subgoal_pose;
    m_robotmap_mutex.unlock();

    cv::Mat robotmap;
    cv::flip(onlinemap, robotmap,0);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO("No occumap");
        robotmap = cv::imread(m_robotmap_path);
    }

    if (robotmap.empty())
        return false;

    Pose2 node_metric, node_pixel;
    Point2 robot_px = cvtMetric2Pixel(robot_dx_metric);
    Point2 node_px_dg = cvtMetric2Pixel(undrivable_pose);

    m_undrivable_points.push_back(undrivable_pose);
    for (int i = 0; i < m_undrivable_points.size(); i++)
    {
        node_metric = m_undrivable_points[i];
        node_pixel = cvtMetric2Pixel(node_metric);
        robotmap.at<uchar>((int)node_pixel.y, (int)node_pixel.x) = 0;
    }

    if(findDrivableinLine(robotmap, robot_px, node_px_dg, pub_pose))
    {
        ROS_INFO("[reselectDrivablePoint] findDrivableinLine <%f, %f>", pub_pose.x, pub_pose.y);
        return true;
    }
    else          
    {
        if(findExtendedDrivablePoint(robotmap, robot_px, node_px_dg, pub_pose))
        {
            ROS_INFO("[reselectDrivablePoint] findExtendedDrivablePoint <%f, %f>", pub_pose.x, pub_pose.y);
            return true;
        }
        else
        {
            ROS_INFO("Tried all. Failed to provide drivable goal.");
            return false;
        }        
    } 

    return false;
}

geometry_msgs::PoseStamped DGRobot::makeRosPubPoseMsg(ID nid, dg::Point2 xy)
{
    //check publish time
    geometry_msgs::PoseStamped rosps;
    rosps.header.stamp = ros::Time::now();
    rosps.header.frame_id = to_string(nid);
    rosps.pose.position.x = xy.x;
    rosps.pose.position.y = xy.y;
    rosps.pose.position.z = 0;

    return rosps;
}


void DGRobot::initialize_DG_DX_conversion()
{
    // DX 로봇 부천 지도 origin 계산
    double ref_x_rotated = (m_dx_map_ref_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    double ref_y_rotated = (m_dx_map_origin_pixel.y - m_dx_map_ref_pixel.y) * m_dx_map_meter_per_pixel;
    double ref_x = ref_x_rotated * cos(m_dx_map_rotation_radian) - ref_y_rotated * sin(m_dx_map_rotation_radian);
    double ref_y = ref_x_rotated * sin(m_dx_map_rotation_radian) + ref_y_rotated * cos(m_dx_map_rotation_radian);

    dg::UTMConverter converter;
    dg::Point2UTM dx_map_ref_utm = converter.cvtLatLon2UTM(m_dx_map_ref_latlon);
    dg::Point2UTM dx_map_origin_utm;
    dx_map_origin_utm.x = dx_map_ref_utm.x - ref_x;
    dx_map_origin_utm.y = dx_map_ref_utm.y - ref_y;
    dx_map_origin_utm.zone = dx_map_ref_utm.zone;
    m_dx_map_origin_latlon = converter.cvtUTM2LatLon(dx_map_origin_utm);
    printf("Robot map origin: x = %lf, y = %lf\n", m_dx_map_origin_pixel.x, m_dx_map_origin_pixel.y);
    printf("Robot map origin: lat = %lf, lon = %lf\n", m_dx_map_origin_latlon.lat, m_dx_map_origin_latlon.lon);

    // 딥가이더 부천 좌표 --> DX 부천 로봇 좌표 --> DX 부천 로봇맵 픽셀좌표
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    m_dx_converter.setReference(m_dx_map_origin_latlon);
    m_dg_converter.setReference(m_dg_map_origin_latlon);
}

Point2 DGRobot::cvtDg2Dx(const Point2& dg_metric) const
{
    double dx_x = dg_metric.x * cos(m_dx_map_rotation_radian) + dg_metric.y * sin(m_dx_map_rotation_radian);
    double dx_y = -dg_metric.x * sin(m_dx_map_rotation_radian) + dg_metric.y * cos(m_dx_map_rotation_radian);
    return Point2(dx_x, dx_y);
}

Point2 DGRobot::cvtDx2Dg(const Point2& dx_metric) const
{
    double x_rotation_corrected = dx_metric.x * cos(m_dx_map_rotation_radian) - dx_metric.y * sin(m_dx_map_rotation_radian);
    double y_rotation_corrected = dx_metric.x * sin(m_dx_map_rotation_radian) + dx_metric.y * cos(m_dx_map_rotation_radian);
    return Point2(x_rotation_corrected, y_rotation_corrected);
}

Point2 DGRobot::cvtMetric2Pixel(const Point2& val) const
{
    Point2 px;
    px.x = m_dx_map_origin_pixel.x + val.x / m_dx_map_meter_per_pixel;
    px.y = m_dx_map_origin_pixel.y - val.y / m_dx_map_meter_per_pixel;
    
    return px;
}

Point2 DGRobot::cvtPixel2Metric(const Point2& px) const
{
    CV_DbgAssert(m_px_per_val.x > 0 && m_px_per_val.y > 0);

    Point2 val;
    val.x = (px.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    val.y = (m_dx_map_origin_pixel.y - px.y) * m_dx_map_meter_per_pixel;

    return val;
}

void DGRobot::callbackDGStatus(const dg_simple_ros::dg_status::ConstPtr& msg)
{
    if(msg->dg_shutdown) m_stop_running = true;
}

// The main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dg_robot");
    ros::NodeHandle nh("~");
    DGRobot dg_node(nh);
    if (dg_node.initialize("dg_ros.yml"))
    {
        dg_node.run();
    }
    return 0;
}
  