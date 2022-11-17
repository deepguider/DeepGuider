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
    double m_min_search_dist = 3.0;
    
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

    //robot related functions
    std::vector<Point2> m_undrivable_points;
    void initialize_DG_DX_conversion();
    bool makeSubgoal(Point2& pub_pose, Point2& goal_px);
    bool makeSubgoal2(Point2& pub_pose, Point2& goal_px);
    bool makeSubgoal3(Point2& pub_pose, Point2& node_dx_pixel);
    bool makeSubgoal4(Point2& pub_pose, Point2& node_dx_pixel);

    bool makeImagePixelsWithRobot(Point2& pub_pose, cv::Mat& image, Pose2& robot_dx_pixel, Point2& node_dx_pixel);
    bool makeImagePixelsWithDG(Point2& pub_pose, cv::Mat& image, Pose2& robot_dx_pixel, Point2& node_dx_pixel);
    geometry_msgs::PoseStamped makeRosPubPoseMsg(ID nid, Point2 xy);
    Point2 findDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px);
    bool findDrivablePointDirection(Point2& pub_pose, Point2& goal_px);
    Point2 findDrivableNearPoint(cv::Mat &image, Point2 robot_px, Point2 node_px);
    bool reselectDrivablePoint(Point2& pub_pose);
    bool findRelativeGoal(double start_rad, Point2& result);
    bool drawSubgoal(Point2& pub_pose);
    bool findRelativeGoalLeft(Point2& result_px);
    bool findRelativeGoalRight(Point2& result_px);
    bool findRelativeGoalForward(Point2& result_px);

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

    //saving current DG pose in robot map
    dg::Pose2 dg_pose = m_localizer->getPose();
    dg::LatLon dg_latlon = toLatLon(dg_pose);
    dg::Point2 dg_metric_rotated = m_dx_converter.toMetric(dg_latlon);
    double dg_dx_x = dg_metric_rotated.x * cos(m_dx_map_rotation_radian) + dg_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double dg_dx_y = -dg_metric_rotated.x * sin(m_dx_map_rotation_radian) + dg_metric_rotated.y * cos(m_dx_map_rotation_radian);
    dg::Point2 dg_cur_metric(dg_dx_x, dg_dx_y);
    m_guider.m_dg_pose = dg_cur_metric;
}

void DGRobot::callbackDrawRobot(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    //Align robot pose rostopic to display image point
    // 4. DX 로봇 좌표 --> 딥가이더 좌표
    double x_rotation_corrected = x * cos(m_dx_map_rotation_radian) - y * sin(m_dx_map_rotation_radian);
    double y_rotation_corrected = x * sin(m_dx_map_rotation_radian) + y * cos(m_dx_map_rotation_radian);
    dg::LatLon latlon = m_dx_converter.toLatLon(dg::Point2(x_rotation_corrected, y_rotation_corrected));
    dg::Point2 dg_metric2 = m_dg_converter.toMetric(latlon);
    dg::Point2 robot_display = m_painter.cvtValue2Pixel(dg_metric2);
    cv::circle(m_map_image, robot_display, 3, cv::Vec3b(0, 255, 255));

    m_guider.m_robot_on_image = robot_display;
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

    //record image    
    Pose2 ro = m_guider.m_robot_pose;
    Pose2 px, px_end, pub_pose;
    Point2 goal_px1, goal_px2, goal_px3, goal_px4, px_node;
    px.x = m_dx_map_origin_pixel.x + ro.x / m_dx_map_meter_per_pixel;
    px.y = m_dx_map_origin_pixel.y - ro.y / m_dx_map_meter_per_pixel;
    double rad = -ro.theta;
    px_end.x = px.x + 20*cos(rad);
    px_end.y = px.y + 20*sin(rad);
    Pose2 node = m_guider.m_robot_heading_node_pose;
    px_node.x = m_dx_map_origin_pixel.x + node.x / m_dx_map_meter_per_pixel;
    px_node.y = m_dx_map_origin_pixel.y - node.y / m_dx_map_meter_per_pixel;

    cv::Mat colormap;
    if (image.channels() == 1)
        cv::cvtColor(image,colormap,cv::COLOR_GRAY2BGR);    
    cv::flip(colormap,colormap,0);
    cv::circle(colormap, px, 10, cv::Vec3b(0, 255, 0), 2);
    cv::line(colormap, px, px_end, cv::Vec3b(0, 0, 255), 3);
    
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    if (nid != 0)
    {
        makeSubgoal(pub_pose, goal_px1);
        makeSubgoal2(pub_pose, goal_px2);
        makeSubgoal3(pub_pose, goal_px3);
        makeSubgoal4(pub_pose, goal_px4);
        cv::circle(colormap, goal_px1, 5, cv::Vec3b(255, 255, 0), 2);
        cv::circle(colormap, goal_px2, 5, cv::Vec3b(255, 0, 255), 2);
        cv::circle(colormap, goal_px3, 5, cv::Vec3b(0, 153, 255), 2);
        cv::circle(colormap, goal_px4, 5, cv::Vec3b(0, 0, 255), 2);
        cv::circle(colormap, px_node, 5, cv::Vec3b(0, 255, 255), 1);
        cv::line(colormap, px, px_node, cv::Vec3b(0, 255, 255), 2);
    }    

    cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);  
    cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
    int x = px.x-400; 
    int y = px.y-400;
    if (x+800 >= colormap.cols) x = colormap.cols - 800 - 1;
    if (x<=1) x = 1;
    if (y+800 >= colormap.rows) y = colormap.rows - 800 - 1;
    if (y<=1) y = 1;
    
    cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
    cv::Mat roicrop(colormap, cv::Rect(x, y, 800, 800));
    colormap.copyTo(roi);
    roicrop.copyTo(videoFrameCrop);

    // recording
    m_video_gui << videoFrame;
    m_video_crop << videoFrameCrop;
    imwrite("../../../online_map.png", videoFrame);
    imwrite("../../../online_crop.png", videoFrameCrop);

    m_robotmap_mutex.unlock();     
}

void DGRobot::publishSubGoal()
{
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    Pose2 node_metric;

    if (hNode == nullptr)
    {
        if (m_guider.isExpendedPathGenerated())
        {
            node_metric = Point2(m_guider.getCurExtendedPath());
            //last guide
            if (nid == 0 && node_metric.x != 0)
            {
                ROS_INFO("Last node_metric.x: %f, y:%f", node_metric.x, node_metric.y);
            }
        }
        else
            return;
    }
    else
    {
        node_metric = Pose2(hNode->x, hNode->y);
    }
    
    Pose2 pub_pose = m_guider.m_subgoal_pose;
    Point2 goal_px;

    //current DG pose
    dg::Pose2 px = m_painter.cvtValue2Pixel(getPose());

    if (!m_guider.m_dxrobot_usage)
    {
        Point2 pt = cvtLatLon2UTM(toLatLon(node_metric));
        geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pt);
        m_guider.m_subgoal_pose = pt;
        pub_subgoal.publish(rosps);
    }
    else
    {        
        GuidanceManager::RobotStatus cur_state = m_guider.getRobotStatus();
        if (cur_state == GuidanceManager::RobotStatus::ARRIVED_NODE || cur_state == GuidanceManager::RobotStatus::ARRIVED_GOAL 
        || cur_state == GuidanceManager::RobotStatus::READY || cur_state == GuidanceManager::RobotStatus::NO_PATH)
        {            
            if(!m_pub_flag)
            {
                // bool ok = makeSubgoal(pub_pose, goal_px);
                // bool ok = makeSubgoal2(pub_pose, goal_px);
                bool ok = makeSubgoal3(pub_pose, goal_px);
                if (pub_pose.x == 0 || pub_pose.y == 0)
                {
                    ROS_INFO_THROTTLE(1.0, "Cannot reselect point: %f, %f", pub_pose.x, pub_pose.y);
                    return;
                }
                geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pub_pose);
                pub_subgoal.publish(rosps);
                m_guider.m_subgoal_pose = pub_pose;

                ROS_INFO("==============================================================\n");
                ROS_INFO("SubGoal published!: %f, %f<=====================", pub_pose.x, pub_pose.y);
                m_begin_time = ros::Time::now();
                m_pub_flag = true;
            }
            else
            {
                ros::Time cur_time = ros::Time::now();
                ros::Duration duration = cur_time - m_begin_time;
                if (duration > ros::Duration(5.0))
                {
                    ROS_INFO_THROTTLE(1.0, "Duration seconds: %d", duration.sec);
                    if (cur_state == GuidanceManager::RobotStatus::NO_PATH) //if robot cannot generate path with the subgoal
                    {
                        bool ok = reselectDrivablePoint(pub_pose);
                        if (pub_pose.x == 0 || pub_pose.y == 0)
                        {
                            ROS_INFO_THROTTLE(1.0, "Cannot reselect point: %f, %f", pub_pose.x, pub_pose.y);
                            return;
                        }                        
                        ROS_INFO_THROTTLE(1.0, "reselectDrivablePoint!: %f, %f", pub_pose.x, pub_pose.y);
                    }
                    geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pub_pose);
                    pub_subgoal.publish(rosps);
                    m_guider.m_subgoal_pose = pub_pose;
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
}

bool DGRobot::makeSubgoal4(Point2& pub_pose, Point2& node_dx_pixel)
{
    cv::Mat image;
    Pose2 robot_dx_pixel;
    bool ok = makeImagePixelsWithDG(pub_pose, image, robot_dx_pixel, node_dx_pixel);
    if (!ok)
        return false;
    ROS_INFO("makeSubgoal4-with DG<%f,%f>",node_dx_pixel.x, node_dx_pixel.y);
    
    //from "makeImagePixelsWithDG" line the original node pixel on robot map image is decided in "node_dx_pixel"
    //customize your node pixel from here.



    ///////////////////////////
        
    

    //for example, if the node pixel is in black area,
    //I draw a line and found drivable area along the line in "findDrivablePoint".
    if (image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x) < 200) //if node is in black area
    {
        //find drivable area on the line from robot_pt
        ROS_INFO("makeSubgoal-current node <%d,%d> is black. find drivable area. value:%d",(int)node_dx_pixel.x, (int)node_dx_pixel.y, image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x));
        pub_pose = findDrivablePoint(image, robot_dx_pixel, node_dx_pixel);
    }



    ROS_INFO("makeSubgoal4 node pixel<%f,%f> ", node_dx_pixel.x, node_dx_pixel.y);
    return true;
}


bool DGRobot::makeSubgoal2(Point2& pub_pose, Point2& goal_px)
{
    ROS_INFO("makeSubgoal2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    if(cur_guide.actions.empty()) return false;

    //robot coordinaate -> robot map coordinate
    bool ok;
    GuidanceManager::Motion cmd = cur_guide.actions[0].cmd;   
    ROS_INFO_THROTTLE(1.0, "makeSubgoal2!: %d", (int) cmd);
    if (cmd == dg::GuidanceManager::Motion::GO_FORWARD || cmd == dg::GuidanceManager::Motion::CROSS_FORWARD || cmd == dg::GuidanceManager::Motion::ENTER_FORWARD || cmd == dg::GuidanceManager::Motion::EXIT_FORWARD)
    {
        double start_rad = 0;
        // ok = findRelativeGoal(start_rad, goal_px);
        ok = findRelativeGoalForward(goal_px);
    }
    else if (cmd == dg::GuidanceManager::Motion::TURN_LEFT || cmd == dg::GuidanceManager::Motion::CROSS_LEFT || cmd == dg::GuidanceManager::Motion::ENTER_LEFT || cmd == dg::GuidanceManager::Motion::EXIT_LEFT)
    {
        double start_rad = cx::cvtDeg2Rad(-40); //image coordinate left
        // ok = findRelativeGoal(start_rad, goal_px);
        ok = findRelativeGoalLeft(goal_px);
    }
    else if (cmd == dg::GuidanceManager::Motion::TURN_RIGHT || cmd == dg::GuidanceManager::Motion::CROSS_RIGHT || cmd == dg::GuidanceManager::Motion::ENTER_RIGHT || cmd == dg::GuidanceManager::Motion::EXIT_RIGHT)
    {
        double start_rad = cx::cvtDeg2Rad(+40); //image coordinate right
        ok = findRelativeGoalRight(goal_px);
    }        

        ROS_INFO("makeSubgoal2 node pixel<%f,%f> ", goal_px.x, goal_px.y);
    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    pub_pose.x = (goal_px.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    pub_pose.y = (m_dx_map_origin_pixel.y - goal_px.y) * m_dx_map_meter_per_pixel;

    return true;
}

bool DGRobot::findRelativeGoalForward(Point2& result_px)
{
    cv::Mat image, img_erode;
    Pose2 dx_pixel, pub_pose;
    Point2 node_dx_pixel;
    bool ok = makeImagePixelsWithDG(pub_pose, image, dx_pixel, node_dx_pixel);
    if (!ok)
    {
        ROS_INFO_THROTTLE(1.0, "findRelativeGoalForward! makeImagePixels did not complete");
        return false;
    }

    //Erode image
    erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    double remain_dist = cur_guide.distance_to_remain;

    double jump_rad, r, max_rad;
    cv::Point jump_px, prev_jump;
    int jump_step = (int)m_robotmap_scale;
    double max_dist = m_min_search_dist*m_robotmap_scale;
    double max_cnt = (int)remain_dist/jump_step;

    //Save image
    cv::Mat img_color;
    if (img_color.channels() == 1)
        cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);
    cv::circle(img_color, dx_pixel, 2, cv::Vec3b(0, 255, 0), 2);   

    for (int n = -30; n < 30; n++)
    {           
        jump_rad = -dx_pixel.theta + cx::cvtDeg2Rad(n);      

        for (size_t i = 0; i <= max_cnt; i++)
        {        
            r = jump_step * i;
            jump_px.x = dx_pixel.x + r * cos(jump_rad);
            jump_px.y = dx_pixel.y + r * sin(jump_rad);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) < 200)
            {
                cv::circle(img_color, prev_jump, 2, cv::Vec3b(0, 255, 255), 2);                    
                if (r > max_dist)
                {
                    max_dist = r;
                    max_rad = jump_rad;
                    result_px = prev_jump;
                    ROS_INFO("max_dist r:%f, deg: %d", max_dist/m_robotmap_scale,(int)n);  
                    break;     
                }
                i = max_cnt;
            }
            prev_jump = jump_px;

        }
    }

    cv::circle(img_color, result_px, 4, cv::Vec3b(0, 0, 255), 4); 
    int x = dx_pixel.x-400; 
    int y = dx_pixel.y-400;
    if (x+800 >= img_color.cols) x = img_color.cols - 800 - 1;
    if (x<=1) x = 1;
    if (y+800 >= img_color.rows) y = img_color.rows - 800 - 1;
    if (y<=1) y = 1;
    
    cv::Mat img_forward(img_color, cv::Rect(x, y, 800, 800));
    cv::imwrite("../../../img_forward.png", img_forward);  

    ROS_INFO("result.x:%f, result.y:%f",result_px.x, result_px.y);
                    
    return true;
}

bool DGRobot::findRelativeGoalLeft(Point2& result_px)
{
    cv::Mat image, img_erode;
    Pose2 dx_pixel, pub_pose;
    Point2 node_dx_pixel;
    bool ok = makeImagePixelsWithDG(pub_pose, image, dx_pixel, node_dx_pixel);
    if (!ok)
    {
        ROS_INFO_THROTTLE(1.0, "findRelativeGoal! makeImagePixels did not complete");
        return false;
    }

    //Erode image
    erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    double remain_dist = cur_guide.distance_to_remain;

    double jump_rad, r, max_rad;
    cv::Point jump_px, prev_jump;
    int jump_step = (int)m_robotmap_scale;
    double max_dist = m_min_search_dist*m_robotmap_scale;
    double max_cnt = remain_dist*jump_step;

    //Save image
    cv::Mat img_color;
    if (img_color.channels() == 1)
        cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);

    for (int n = 90; n > 10; n--)
    {           
        jump_rad = -dx_pixel.theta + cx::cvtDeg2Rad(n);      

        for (size_t i = 0; i <= max_cnt; i++)
        {        
            r = jump_step * i;
            jump_px.x = dx_pixel.x + r * cos(jump_rad);
            jump_px.y = dx_pixel.y + r * sin(jump_rad);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) < 200)
            {
                cv::circle(img_color, prev_jump, 2, cv::Vec3b(0, 255, 255), 2);                    
                if (r > max_dist)
                {
                    max_dist = r;
                    max_rad = jump_rad;
                    result_px = prev_jump;      
                    ROS_INFO("findRelativeGoalLeft max_dist r:%f, deg: %d", max_dist/m_robotmap_scale, n); 
                    break;
                }
                i = max_cnt;
            }
            prev_jump = jump_px;
        }
    }

    cv::circle(img_color, result_px, 4, cv::Vec3b(0, 0, 255), 4); 
    int x = dx_pixel.x-400; 
    int y = dx_pixel.y-400;
    if (x+800 >= img_color.cols) x = img_color.cols - 800 - 1;
    if (x<=1) x = 1;
    if (y+800 >= img_color.rows) y = img_color.rows - 800 - 1;
    if (y<=1) y = 1;
    
    cv::Mat img_left(img_color, cv::Rect(x, y, 800, 800));
    cv::imwrite("../../../img_left.png", img_left);  

    ROS_INFO("findRelativeGoalLeft result.x:%f, result.y:%f",result_px.x, result_px.y);
                    
    return true;
}

bool DGRobot::findRelativeGoalRight(Point2& result_px)
{
    cv::Mat image, img_erode;
    Pose2 dx_pixel, pub_pose;
    Point2 node_dx_pixel;
    bool ok = makeImagePixelsWithRobot(pub_pose, image, dx_pixel, node_dx_pixel);
    if (!ok)
    {
        ROS_INFO("findRelativeGoalRight! makeImagePixels did not complete");
        return false;
    }

    //Erode image
    erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    double remain_dist = cur_guide.distance_to_remain;

    double jump_rad, r, max_rad;
    cv::Point jump_px, prev_jump;
    int jump_step = (int)m_robotmap_scale;
    double max_dist = m_min_search_dist*m_robotmap_scale;
    double max_cnt = remain_dist*jump_step;

    //Save image
    cv::Mat img_color;
    if (img_color.channels() == 1)
        cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);

    for (int n = -90; n < -10; n++)
    {           
        jump_rad = -dx_pixel.theta + cx::cvtDeg2Rad(n);      

        for (size_t i = 0; i <= max_cnt; i++)
        {        
            r = jump_step * i;
            jump_px.x = dx_pixel.x + r * cos(jump_rad);
            jump_px.y = dx_pixel.y + r * sin(jump_rad);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) < 200)
            {
                cv::circle(img_color, prev_jump, 2, cv::Vec3b(0, 255, 255), 2);                    
                if (r > max_dist)
                {
                    max_dist = r;
                    max_rad = jump_rad;
                    result_px = prev_jump;     
                    ROS_INFO("found! max_dist:%f, deg: %d", max_dist/m_robotmap_scale, n);  
                    break;
                }
                i = max_cnt;
            }
            prev_jump = jump_px;
        }
    }

    cv::circle(img_color, result_px, 4, cv::Vec3b(0, 0, 255), 4); 
    int x = dx_pixel.x-400; 
    int y = dx_pixel.y-400;
    if (x+800 >= img_color.cols) x = img_color.cols - 800 - 1;
    if (x<=1) x = 1;
    if (y+800 >= img_color.rows) y = img_color.rows - 800 - 1;
    if (y<=1) y = 1;
    
    cv::Mat img_left(img_color, cv::Rect(x, y, 800, 800));
    cv::imwrite("../../../img_right.png", img_left);  

    ROS_INFO("findRelativeGoalRight result.x:%f, findRelativeGoalRight result.y:%f",result_px.x, result_px.y);
                    
    return true;
}

bool DGRobot::findRelativeGoal(double search_rad, Point2& result)
{
    cv::Mat image;
    Pose2 dx_pixel, pub_pose;
    Point2 node_dx_pixel;
    bool ok = makeImagePixelsWithDG(pub_pose, image, dx_pixel, node_dx_pixel);
    if (!ok)
    {
        ROS_INFO_THROTTLE(1.0, "findRelativeGoal! makeImagePixels did not complete");
        return false;
    }
    
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    double remain_dist = cur_guide.distance_to_remain;

    double jump_rad_plus, jump_rad_minus, r, max_rad;
    cv::Point start_px, jump_px_plus, prev_jump_plus, jump_px_minus, prev_jump_minus;
    int jump_step = (int)m_robotmap_scale;
    double max_dist = m_min_search_dist*m_robotmap_scale;
    double max_cnt = remain_dist*jump_step;
    double start_rad = -dx_pixel.theta + search_rad;

    for (size_t n = 0; n < 30; n++)
    {                 
        jump_rad_plus = start_rad + cx::cvtDeg2Rad(n);
        jump_rad_minus = start_rad - cx::cvtDeg2Rad(n);
        for (size_t i = 0; i <= max_cnt; i++)
        {        
            r = jump_step * i;
            jump_px_plus.x = dx_pixel.x + r * cos(jump_rad_plus);
            jump_px_plus.y = dx_pixel.y + r * sin(jump_rad_plus);
            if (image.at<uchar>((int)jump_px_plus.y, (int)jump_px_plus.x) < 200)
            {
                // ROS_INFO("findRelativeGoal+%d,! r:%f, start_deg: %d+%d", (int) i, r,(int)cx::cvtRad2Deg(search_rad), (int) n);
                    
                if (r > max_dist)
                {
                    max_dist = r;
                    max_rad = jump_rad_plus;
                    result = prev_jump_plus;
                    ROS_INFO("findRelativeGoal found! r:%f, start_deg: %d+%d", max_dist, (int)cx::cvtRad2Deg(search_rad), (int) n);
                    break;
                }
                i = max_cnt;
            }
            prev_jump_plus = jump_px_plus;
            jump_px_minus.x = dx_pixel.x + r * cos(jump_rad_minus);
            jump_px_minus.y = dx_pixel.y + r * sin(jump_rad_minus);
            if (image.at<uchar>((int)jump_px_minus.y, (int)jump_px_minus.x) < 200)
            {
                // ROS_INFO("findRelativeGoal-%d,! r:%f, start_deg: %d+%d", (int) i, r,(int)cx::cvtRad2Deg(search_rad), (int) n);
             
                if (r > max_dist)
                {
                    max_dist = r;
                    max_rad = jump_rad_minus;
                    result = prev_jump_minus;
                    ROS_INFO("found-%d,! r:%f, deg:%d", (int) i, max_dist,(int) n);
                    break;
                }
                i = max_cnt;
            }
            prev_jump_minus = jump_px_minus;
        }
    }

    ROS_INFO("result.x:%f, result.y:%f",result.x, result.y);
                    
    return true;
}

bool DGRobot::makeSubgoal(Point2& pub_pose, Point2& node_dx_pixel)
{
    ROS_INFO("makeSubgoal+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    cv::Mat image;
    Pose2 robot_dx_pixel;
    bool ok = makeImagePixelsWithRobot(pub_pose, image, robot_dx_pixel, node_dx_pixel);
    // ROS_INFO("makeSubgoal node pixel with robot<%f,%f> ", node_dx_pixel.x, node_dx_pixel.y);
    if (!ok)
        return false;

    if (image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x) < 200) //if node is in black area
    {
        //find drivable area on the line from robot_pt
        ROS_INFO("makeSubgoal-current node <%d,%d> is black. find drivable area. value:%d",(int)node_dx_pixel.x, (int)node_dx_pixel.y, image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x));
        pub_pose = findDrivablePoint(image, robot_dx_pixel, node_dx_pixel);

        // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
        dg::Point2 dx_pixel;
        dx_pixel.x = m_dx_map_origin_pixel.x + pub_pose.x / m_dx_map_meter_per_pixel;  
        dx_pixel.y = m_dx_map_origin_pixel.y - pub_pose.y / m_dx_map_meter_per_pixel;

        // ROS_INFO("makeSubgoal node pixel<%f,%f> ", dx_pixel.x, dx_pixel.y);
        node_dx_pixel = dx_pixel;
    }

    ROS_INFO("makeSubgoal node pixel<%f,%f> ", node_dx_pixel.x, node_dx_pixel.y);
    return true;
}

bool DGRobot::makeImagePixelsWithRobot(Point2& pub_pose, cv::Mat& image, Pose2& robot_dx_pixel, Point2& node_dx_pixel)
{
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return false;
    
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
    LatLon node_latlon = m_map.toLatLon(node_metric);
    
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    // dg::LatLon latlon = m_dg_converter.toLatLon(node_metric);
    dg::Point2 node_metric_rotated = m_dx_converter.toMetric(node_latlon);
    double node_dx_x = node_metric_rotated.x * cos(m_dx_map_rotation_radian) + node_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double node_dx_y = -node_metric_rotated.x * sin(m_dx_map_rotation_radian) + node_metric_rotated.y * cos(m_dx_map_rotation_radian);
    dg::Point2 node_dx_metric(node_dx_x, node_dx_y);
    ROS_INFO_THROTTLE(1.0, "Subgoal for robot: %f,%f", node_dx_metric.x, node_dx_metric.y);
    pub_pose = node_dx_metric;    

    //save to 
    cv::Mat onlinemap, offlinemap;
    Pose2 robot_dx_metric;
    //Projecting node point to drivable area
    m_robotmap_mutex.lock();
    onlinemap = m_robotmap_image;
    robot_dx_metric = m_guider.m_robot_pose;
    m_guider.m_robot_heading_node_id = nid;
    m_guider.m_robot_heading_node_pose = node_dx_metric;
    m_robotmap_mutex.unlock();

    offlinemap = cv::imread(m_robotmap_path);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO_THROTTLE(1.0, "No occumap");
        image = offlinemap;
    }
    else
    {
        ROS_INFO_THROTTLE(1.0, "Receiving robotmap");
        cv::flip(onlinemap,image,0);
    }

    if (image.channels() != 1)
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
    node_dx_pixel.x = m_dx_map_origin_pixel.x + node_dx_metric.x / m_dx_map_meter_per_pixel;
    node_dx_pixel.y = m_dx_map_origin_pixel.y - node_dx_metric.y / m_dx_map_meter_per_pixel;

    robot_dx_pixel.x = m_dx_map_origin_pixel.x + robot_dx_metric.x / m_dx_map_meter_per_pixel;
    robot_dx_pixel.y = m_dx_map_origin_pixel.y - robot_dx_metric.y / m_dx_map_meter_per_pixel;
    robot_dx_pixel.theta = robot_dx_metric.theta;

    if (node_dx_pixel.x < 1 ) {node_dx_pixel.x = 10; ROS_INFO("Node pixel smaller than x=0");}
    if (node_dx_pixel.x > image.cols) {node_dx_pixel.x = image.cols-10; ROS_INFO("Node pixel exceeds x=cols");}
    if (node_dx_pixel.y < 1) {node_dx_pixel.y = 10; ROS_INFO("Node pixel smaller than y=0");}
    if (node_dx_pixel.y > image.rows) {node_dx_pixel.y = image.rows-10; ROS_INFO("Node pixel exceeds y=rows");}
    if (robot_dx_pixel.x < 1 ) {robot_dx_pixel.x = 10; ROS_INFO("Robot pixel smaller than x=0");}
    if (robot_dx_pixel.x > image.cols) {robot_dx_pixel.x = image.cols-10; ROS_INFO("Robot pixel exceeds x=cols");}
    if (robot_dx_pixel.y < 1) {robot_dx_pixel.y = 10; ROS_INFO("Robot pixel smaller than y=0");}
    if (robot_dx_pixel.y > image.rows) {robot_dx_pixel.y = image.rows-10; ROS_INFO("Robot pixel exceeds y=rows");}
    
    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    pub_pose.x = (node_dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    pub_pose.y = (m_dx_map_origin_pixel.y - node_dx_pixel.y) * m_dx_map_meter_per_pixel;

    return true;   
}

bool DGRobot::makeImagePixelsWithDG(Point2& pub_pose, cv::Mat& image, Pose2& robot_dx_pixel, Point2& node_dx_pixel)
{
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return false;
    
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
    LatLon node_latlon = m_map.toLatLon(node_metric);
    
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    // dg::LatLon latlon = m_dg_converter.toLatLon(node_metric);
    dg::Point2 node_metric_rotated = m_dx_converter.toMetric(node_latlon);
    double node_dx_x = node_metric_rotated.x * cos(m_dx_map_rotation_radian) + node_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double node_dx_y = -node_metric_rotated.x * sin(m_dx_map_rotation_radian) + node_metric_rotated.y * cos(m_dx_map_rotation_radian);
    dg::Point2 node_dx_metric(node_dx_x, node_dx_y);
    ROS_INFO("Subgoal for robot-m_robot_heading_node_pose: %f,%f", node_dx_metric.x, node_dx_metric.y);
    pub_pose = node_dx_metric;

    //read images and parameters
    cv::Mat onlinemap, offlinemap;
    Pose2 robot_dx_metric, dg_prev, dx_prev;

    m_robotmap_mutex.lock();

    onlinemap = m_robotmap_image;
    robot_dx_metric = m_guider.m_robot_pose;
    dg_prev = m_guider.m_prev_dg_pose;
    dx_prev = m_guider.m_prev_dx_pose;
    m_guider.m_robot_heading_node_id = nid;
    m_guider.m_robot_heading_node_pose = node_dx_metric;

    m_robotmap_mutex.unlock();

    offlinemap = cv::imread(m_robotmap_path);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO_THROTTLE(1.0, "No occumap");
        image = offlinemap;
    }
    else
    {
        ROS_INFO_THROTTLE(1.0, "Receiving robotmap");
        cv::flip(onlinemap,image,0);
    }

    if (image.channels() != 1)
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    //current DG pose in robot map
    dg::Pose2 dg_pose = m_localizer->getPose();
    dg::LatLon dg_latlon = toLatLon(dg_pose);
    dg::Point2 dg_metric_rotated = m_dx_converter.toMetric(dg_latlon);
    double dg_dx_x = dg_metric_rotated.x * cos(m_dx_map_rotation_radian) + dg_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double dg_dx_y = -dg_metric_rotated.x * sin(m_dx_map_rotation_radian) + dg_metric_rotated.y * cos(m_dx_map_rotation_radian);
    dg::Point2 dg_cur_metric(dg_dx_x, dg_dx_y);
    m_guider.m_dg_pose = dg_cur_metric;

    //transport dg based node to dx based subgoal
    double dg_prev_dx_x = dg_prev.x * cos(m_dx_map_rotation_radian) + dg_prev.y * sin(m_dx_map_rotation_radian);
    double dg_prev_dx_y = -dg_prev.x * sin(m_dx_map_rotation_radian) + dg_prev.y * cos(m_dx_map_rotation_radian);
    dg::Point2 dg_prev_metric(dg_prev_dx_x, dg_prev_dx_y);
    dg::Pose2 dg_next_metric = node_dx_metric;
    dg::Pose2 robot_prev_metric = dx_prev;
    dg::Pose2 robot_cur_metric = robot_dx_metric;
    
    dg::Pose2 dg_pose_diff = dg_next_metric - dg_cur_metric; 
    dg::Pose2 dx_pose_diff = robot_cur_metric - robot_prev_metric; 
    ROS_INFO("makeImagePixelsWithDG-dg_prev_metric: %f,%f", dg_prev_metric.x, dg_prev_metric.y);
    ROS_INFO("makeImagePixelsWithDG-dg_cur_metric: %f,%f", dg_cur_metric.x, dg_cur_metric.y);
    ROS_INFO("makeImagePixelsWithDG-dg_next_metric: %f,%f", dg_next_metric.x, dg_next_metric.y);
    
    double angle_rad, angle_deg, base_angle;
        
    if (dg_prev_metric.x != 0)
    {   
        angle_deg = m_guider.getDegree(dg_prev_metric, dg_cur_metric, dg_next_metric);
        angle_rad = cx::cvtDeg2Rad(angle_deg);
        base_angle = atan2(dx_pose_diff.y, dx_pose_diff.x);
    }
    else
    {  
        double angle_diff = atan2(dg_pose_diff.y, dg_pose_diff.x);
        angle_rad = angle_diff - robot_dx_metric.theta;  //in intial pose follow robot's theta
        base_angle = robot_dx_metric.theta;
    }

    double dist_dg = norm(dg_pose_diff); 

    Pose2 target_pose;

    target_pose.x = robot_dx_metric.x + dist_dg*cos(angle_rad + base_angle);
    target_pose.y = robot_dx_metric.y + dist_dg*sin(angle_rad + base_angle);

    ROS_INFO("makeImagePixelsWithDG-base_angle: %f, angle:%f", cx::cvtRad2Deg(base_angle), cx::cvtRad2Deg(angle_rad));
    ROS_INFO("makeImagePixelsWithDG-target origin: %f,%f", node_dx_metric.x, node_dx_metric.y);
    ROS_INFO("makeImagePixelsWithDG-target modified: %f,%f", target_pose.x, target_pose.y);
    m_guider.m_robot_heading_node_pose = target_pose;

    //Projecting node point to drivable area
    // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
    node_dx_pixel.x = m_dx_map_origin_pixel.x + target_pose.x / m_dx_map_meter_per_pixel;
    node_dx_pixel.y = m_dx_map_origin_pixel.y - target_pose.y / m_dx_map_meter_per_pixel;

    robot_dx_pixel.x = m_dx_map_origin_pixel.x + robot_dx_metric.x / m_dx_map_meter_per_pixel;
    robot_dx_pixel.y = m_dx_map_origin_pixel.y - robot_dx_metric.y / m_dx_map_meter_per_pixel;
    robot_dx_pixel.theta = robot_dx_metric.theta;

    ROS_INFO_THROTTLE(1.0, "makeImagePixels-node px_on_robotmap: %f,%f", node_dx_pixel.x, node_dx_pixel.y);
    ROS_INFO_THROTTLE(1.0, "makeImagePixels-robot px_on_robotmap: %f,%f", robot_dx_pixel.x, robot_dx_pixel.y);

    if (node_dx_pixel.x < 1 ) {node_dx_pixel.x = 0; ROS_INFO("Node pixel exceeds image size");}
    if (node_dx_pixel.x > image.cols) {node_dx_pixel.x = image.cols; ROS_INFO("Node pixel exceeds image size");}
    if (node_dx_pixel.y < 1) {node_dx_pixel.y = 0; ROS_INFO("Node pixel exceeds image size");}
    if (node_dx_pixel.y > image.rows) {node_dx_pixel.y = image.rows; ROS_INFO("Node pixel exceeds image size");}
    if (robot_dx_pixel.x < 1 ) {robot_dx_pixel.x = 0; ROS_INFO("Robot pixel exceeds image size");}
    if (robot_dx_pixel.x > image.cols) {robot_dx_pixel.x = image.cols; ROS_INFO("Robot pixel exceeds image size");}
    if (robot_dx_pixel.y < 1) {robot_dx_pixel.y = 0; ROS_INFO("Robot pixel exceeds image size");}
    if (robot_dx_pixel.y > image.rows) {robot_dx_pixel.y = image.rows; ROS_INFO("Robot pixel exceeds image size");}
    
    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    pub_pose.x = (node_dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    pub_pose.y = (m_dx_map_origin_pixel.y - node_dx_pixel.y) * m_dx_map_meter_per_pixel;

    return true;   
}

bool DGRobot::makeSubgoal3(Point2& pub_pose, Point2& node_dx_pixel)
{
    ROS_INFO("makeSubgoal3+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    cv::Mat image; 
    Pose2 robot_dx_pixel;
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return false;
    
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
    LatLon node_latlon = m_map.toLatLon(node_metric);
    
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    // dg::LatLon latlon = m_dg_converter.toLatLon(node_metric);
    dg::Point2 node_metric_rotated = m_dx_converter.toMetric(node_latlon);
    double node_dx_x = node_metric_rotated.x * cos(m_dx_map_rotation_radian) + node_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double node_dx_y = -node_metric_rotated.x * sin(m_dx_map_rotation_radian) + node_metric_rotated.y * cos(m_dx_map_rotation_radian);
    dg::Point2 node_dx_metric(node_dx_x, node_dx_y);
    ROS_INFO("Subgoal for robot-m_robot_heading_node_pose: %f,%f", node_dx_metric.x, node_dx_metric.y);
    pub_pose = node_dx_metric;

    //read images and parameters
    cv::Mat onlinemap, offlinemap;
    Pose2 robot_dx_metric, dg_prev, dx_prev;

    m_robotmap_mutex.lock();

    onlinemap = m_robotmap_image;
    robot_dx_metric = m_guider.m_robot_pose;
    dg_prev = m_guider.m_prev_dg_pose;
    dx_prev = m_guider.m_prev_dx_pose;
    m_guider.m_robot_heading_node_id = nid;
    m_guider.m_robot_heading_node_pose = node_dx_metric;

    m_robotmap_mutex.unlock();

    offlinemap = cv::imread(m_robotmap_path);
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO_THROTTLE(1.0, "No occumap");
        image = offlinemap;
    }
    else
    {
        ROS_INFO_THROTTLE(1.0, "Receiving robotmap");
        cv::flip(onlinemap,image,0);
    }

    if (image.channels() != 1)
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    //current DG pose in robot map
    dg::Pose2 dg_pose = m_localizer->getPose();
    dg::LatLon dg_latlon = toLatLon(dg_pose);
    dg::Point2 dg_metric_rotated = m_dx_converter.toMetric(dg_latlon);
    double dg_dx_x = dg_metric_rotated.x * cos(m_dx_map_rotation_radian) + dg_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double dg_dx_y = -dg_metric_rotated.x * sin(m_dx_map_rotation_radian) + dg_metric_rotated.y * cos(m_dx_map_rotation_radian);
    dg::Point2 dg_cur_metric(dg_dx_x, dg_dx_y);
    m_guider.m_dg_pose = dg_cur_metric;

    //transport dg based node to dx based subgoal
    double dg_prev_dx_x = dg_prev.x * cos(m_dx_map_rotation_radian) + dg_prev.y * sin(m_dx_map_rotation_radian);
    double dg_prev_dx_y = -dg_prev.x * sin(m_dx_map_rotation_radian) + dg_prev.y * cos(m_dx_map_rotation_radian);
    dg::Point2 dg_prev_metric(dg_prev_dx_x, dg_prev_dx_y);
    dg::Pose2 dg_next_metric = node_dx_metric;
    dg::Pose2 robot_prev_metric = dx_prev;
    dg::Pose2 robot_cur_metric = robot_dx_metric;
    
    dg::Pose2 dg_pose_diff = dg_next_metric - dg_cur_metric; 
    dg::Pose2 dx_pose_diff = robot_cur_metric - robot_prev_metric; 
    ROS_INFO("makeSubgoal3-dg_prev_metric: %f,%f", dg_prev_metric.x, dg_prev_metric.y);
    ROS_INFO("makeSubgoal3-dg_cur_metric: %f,%f", dg_cur_metric.x, dg_cur_metric.y);
    ROS_INFO("makeSubgoal3-dg_next_metric: %f,%f", dg_next_metric.x, dg_next_metric.y);
    
    double angle_rad, angle_deg, base_angle;
        
    if (dg_prev_metric.x != 0)
    {   
        angle_deg = m_guider.getDegree(dg_prev_metric, dg_cur_metric, dg_next_metric);
        angle_rad = cx::cvtDeg2Rad(angle_deg);
        base_angle = atan2(dx_pose_diff.y, dx_pose_diff.x);
    }
    else
    {  
        double angle_diff = atan2(dg_pose_diff.y, dg_pose_diff.x);
        angle_rad = angle_diff - robot_dx_metric.theta;  //in intial pose follow robot's theta
        angle_deg = cx::cvtRad2Deg(angle_rad);
        base_angle = robot_dx_metric.theta;
    }

    double dist_dg = norm(dg_pose_diff); 

    Pose2 target_pose;

    target_pose.x = robot_dx_metric.x + dist_dg*cos(angle_rad + base_angle);
    target_pose.y = robot_dx_metric.y + dist_dg*sin(angle_rad + base_angle);

    ROS_INFO("makeSubgoal3-base_angle: %f, angle:%f", cx::cvtRad2Deg(base_angle), cx::cvtRad2Deg(angle_rad));
    ROS_INFO("makeSubgoal3-target origin: %f,%f", node_dx_metric.x, node_dx_metric.y);
    ROS_INFO("makeSubgoal3-target modified: %f,%f", target_pose.x, target_pose.y);
    m_guider.m_robot_heading_node_pose = target_pose;

    //Projecting node point to drivable area
    // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
    node_dx_pixel.x = m_dx_map_origin_pixel.x + target_pose.x / m_dx_map_meter_per_pixel;
    node_dx_pixel.y = m_dx_map_origin_pixel.y - target_pose.y / m_dx_map_meter_per_pixel;

    robot_dx_pixel.x = m_dx_map_origin_pixel.x + robot_dx_metric.x / m_dx_map_meter_per_pixel;
    robot_dx_pixel.y = m_dx_map_origin_pixel.y - robot_dx_metric.y / m_dx_map_meter_per_pixel;
    robot_dx_pixel.theta = robot_dx_metric.theta;

    ROS_INFO("makeImagePixels-node px_on_robotmap: %f,%f", node_dx_pixel.x, node_dx_pixel.y);
    ROS_INFO("makeImagePixels-robot px_on_robotmap: %f,%f", robot_dx_pixel.x, robot_dx_pixel.y);

    if (node_dx_pixel.x < 1 ) {node_dx_pixel.x = 0; ROS_INFO("Node pixel exceeds image size");}
    if (node_dx_pixel.x > image.cols) {node_dx_pixel.x = image.cols; ROS_INFO("Node pixel exceeds image size");}
    if (node_dx_pixel.y < 1) {node_dx_pixel.y = 0; ROS_INFO("Node pixel exceeds image size");}
    if (node_dx_pixel.y > image.rows) {node_dx_pixel.y = image.rows; ROS_INFO("Node pixel exceeds image size");}
    if (robot_dx_pixel.x < 1 ) {robot_dx_pixel.x = 0; ROS_INFO("Robot pixel exceeds image size");}
    if (robot_dx_pixel.x > image.cols) {robot_dx_pixel.x = image.cols; ROS_INFO("Robot pixel exceeds image size");}
    if (robot_dx_pixel.y < 1) {robot_dx_pixel.y = 0; ROS_INFO("Robot pixel exceeds image size");}
    if (robot_dx_pixel.y > image.rows) {robot_dx_pixel.y = image.rows; ROS_INFO("Robot pixel exceeds image size");}

    //if target is in black area.
    Point2 jump_px, goal_px;

    //for display map
    cv::Mat img_erode;
    erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

    if (img_erode.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x) < 200) //if node is in black area
    {
        //find drivable area on the line from robot_pt
        ROS_INFO("makeSubgoal-current node <%d,%d> is black. find drivable area. value:%d",(int)node_dx_pixel.x, (int)node_dx_pixel.y, image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x));
        
        //find drivable area on the line from robot_pt
        double jump_deg = atan2(robot_dx_pixel.y - node_dx_pixel.y, robot_dx_pixel.x - node_dx_pixel.x);
        double jump_dist = norm(robot_dx_pixel - node_dx_pixel);
        int jump_step = (int)m_robotmap_scale;
        int max_count = (int)jump_dist / jump_step;
        ROS_INFO("findDrivablePoint-jump_deg: %f", cx::cvtRad2Deg(jump_deg));
        ROS_INFO("findDrivablePoint-max_count: %d", max_count);

        
        int i;
        for (i = 0; i < max_count; i++)
        {
            jump_px.x = node_dx_pixel.x + i * jump_step * cos(jump_deg);
            jump_px.y = node_dx_pixel.y + i * jump_step * sin(jump_deg);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) > 200)
            {
                ROS_INFO("Found DrivablePoint-img_erode-%d <%d, %d>: %d\n", i, (int) jump_px.x, (int) jump_px.y, img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x));
                break;
            }
        }
        node_dx_pixel = jump_px;
    }

    double dist = norm(node_dx_pixel - robot_dx_pixel)/m_robotmap_scale; 
    ROS_INFO("Distance to node is %f", dist);

    bool ok;
    if(dist < m_min_search_dist)
    {
        if (abs(angle_deg)<15) //GO_FORWARD
        {
            ROS_INFO("GO_FORWARD: angle %f", angle_deg);
            ok = findRelativeGoal(angle_deg, node_dx_pixel);
        }
        else if (base_angle >= 15)//TURN_LEFT
        {
            ROS_INFO("TURN_LEFT: angle %f", angle_deg);
            ok = findRelativeGoalLeft(node_dx_pixel);
        }
        else
        {
            ROS_INFO("TURN_RIGHT: angle %f", angle_deg);
            ok = findRelativeGoalRight(node_dx_pixel);
        }            
    }

    ROS_INFO("makeSubgoal3 node pixel<%f,%f> ", node_dx_pixel.x, node_dx_pixel.y);
    
    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    pub_pose.x = (node_dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    pub_pose.y = (m_dx_map_origin_pixel.y - node_dx_pixel.y) * m_dx_map_meter_per_pixel;

    return true;   
}

Point2 DGRobot::findDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px)
{
    Point2 jump_px, dx_metric;
    cv::Mat img_erode;
    if (!image.empty())
    {
        //find drivable area on the line from robot_pt
        double jump_deg = atan2(robot_px.y - node_px.y, robot_px.x - node_px.x);
        double jump_dist = norm(robot_px - node_px);
        int jump_step = (int)m_robotmap_scale;
        int max_count = (int)jump_dist / jump_step;
        ROS_INFO("findDrivablePoint-jump_deg: %f", cx::cvtRad2Deg(jump_deg));
        ROS_INFO("findDrivablePoint-max_count: %d", max_count);

        //for display map
        erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

        int i;
        for (i = 0; i < max_count; i++)
        {
            jump_px.x = node_px.x + i * jump_step * cos(jump_deg);
            jump_px.y = node_px.y + i * jump_step * sin(jump_deg);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) > 200)
            {
                ROS_INFO("Found DrivablePoint-img_erode-%d <%d, %d>: %d\n", i, (int) jump_px.x, (int) jump_px.y, img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x));
                break;
            }
        }

        double dist = norm(jump_px - robot_px);
        ROS_INFO("node dist is %f",dist);
        if (i >= max_count - 1 || dist < 3) //no drivble area on the line
        {        
            // ROS_INFO("cannot provide goal");
            ROS_INFO("Find drivable point near goal: %f, %f", jump_px.x, jump_px.y);
        }
    }

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    dx_metric.x = (jump_px.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    dx_metric.y = (m_dx_map_origin_pixel.y - jump_px.y) * m_dx_map_meter_per_pixel;
    ROS_INFO("Find drivable point completed<%f, %f>", jump_px.x, jump_px.y);

    //save to image
    cv::Mat img_color;
    // img_erode.copyTo(img_color);
    cv::cvtColor(img_erode, img_color, cv::COLOR_GRAY2BGR);
    cv::line(img_color, robot_px, node_px, cv::Vec3b(50, 50, 50), 5);
    cv::circle(img_color, node_px, 10, cv::Vec3b(0, 0, 255), 10);
    cv::circle(img_color, robot_px, 10, cv::Vec3b(0, 255, 0), 10);
    cv::circle(img_color, jump_px, 10, cv::Vec3b(0, 255, 255), 10);
    cv::imwrite("../../../occumap_origin.png", img_erode);
    cv::imwrite("../../../occumap_erode.png", img_color);

    return dx_metric;
}

bool DGRobot::reselectDrivablePoint(Point2& pub_pose)
{
    cv::Mat image;
    Point2 node_dx_pixel, dx_metric, dx_pixel;
    Pose2 robot_dx_pixel;
    bool ok = makeImagePixelsWithRobot(pub_pose, image, robot_dx_pixel, node_dx_pixel);
    if (!ok)
        return false;

    Point2 undrivable_pose = m_guider.m_subgoal_pose;
    m_undrivable_points.push_back(undrivable_pose);

    for (int i = 0; i < m_undrivable_points.size(); i++)
    {
        dx_metric = m_undrivable_points[i];
        // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
        dx_pixel.x = m_dx_map_origin_pixel.x + dx_metric.x / m_dx_map_meter_per_pixel;
        dx_pixel.y = m_dx_map_origin_pixel.y - dx_metric.y / m_dx_map_meter_per_pixel;
        image.at<uchar>((int)dx_pixel.y, (int)dx_pixel.x) = 0;
    }

    pub_pose = findDrivablePoint(image, robot_dx_pixel, node_dx_pixel);

    return true;
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
  