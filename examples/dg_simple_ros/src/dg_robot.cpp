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
    Pose2 m_prev_node_dg;  //in DG coordinate
    Pose2 m_cur_node_dg;  //in DG coordinate
    Pose2 m_prev_robot_pose;  //in robot coordinate
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
    double m_robotmap_scale = 10.0;  // TODO: no need for m_robotmap_scale. Just use 1/m_dx_map_meter_per_pixel
    dg::Point2 m_dx_map_origin_pixel = dg::Point2(345, 1110);
    dg::Point2UTM m_dx_map_origin_utm;
    dg::Point2UTM m_dg_map_origin_utm;
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
    bool isSubPathDrivable(cv::Mat robotmap, Pose2 pointA, Pose2 pointB);
    bool isSubPathDrivablev2(cv::Mat robotmap, Pose2 dest_point);
    bool isSubPathDrivablev3(cv::Mat robotmap, Pose2 dest_point, Pose2 source_point);
    bool findAlternativePath(Pose2& alternate_point, Pose2 robot_pose, Pose2 next_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_nextnode, int num_alternatives, cv::Mat& colormap, double& min_dist_to_next_node);
    bool findAlternativePathv2(Pose2& alternate_point, Pose2 robot_pose, Pose2 next_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_nextnode, int num_alternatives, cv::Mat& colormap, double angle_range, double& min_dist_to_next_node);
    void rotatePose(Pose2& P, double theta_rot);
    Pose2 cvtDGtoDXcoordinate(Pose2 P, Pose2 P_DG, Pose2 P_DX);
    Pose2 cvtMaptoRobotcoordinate(Pose2 P);
    Pose2 cvtRobottoMapcoordinate(Pose2 P);
    Pose2 m_robot_origin;
    int m_drivable_threshold = 240;
    bool m_robotarrived_but_nodenotyetupdated = false;
    bool m_save_video = true;

    geometry_msgs::PoseStamped makeRosPubPoseMsg(ID nid, Point2 xy);
    bool findExtendedDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2& result_metric);
    bool findDrivableinLine(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2& result_metric);
    bool reselectDrivablePoint(Point2& pub_pose);
    bool drawSubgoal(Point2& pub_pose);

    int m_video_recording_fps = 15;
    cv::Size m_framesize = cv::Size(4000, 4000); 
    cv::Size m_framesize_crop = cv::Size(800, 800);
    int m_fourcc = cv::VideoWriter::fourcc('A', 'V', 'C', '1');   
    cv::VideoWriter m_video_gui;
    cv::VideoWriter m_video_crop;
    cv::VideoWriter m_mapvideo_crop;
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
    m_mapvideo_crop.open("../../../map_online_crop.avi", m_fourcc, m_video_recording_fps, m_framesize_crop);

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
    m_robot_origin = dg::Point2(map->info.origin.position.x, map->info.origin.position.y);

    ROS_INFO_THROTTLE(1.0, "callbackRobotMap: Robot map size x: %d, y: %d", size_x, size_y);
    
    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

    m_robotmap_mutex.lock();

    Point2 new_origin_pixel;
    // // convert new origin from robot to map coordinate
    // new_origin_pixel.x = -map->info.origin.position.x / m_dx_map_meter_per_pixel;
    // new_origin_pixel.y = -map->info.origin.position.y / m_dx_map_meter_per_pixel;
    // alternatively
    Point2 origin = dg::Point2(0, 0);
    new_origin_pixel = cvtRobottoMapcoordinate(origin);
    
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
                image.at<uchar>(i,j) = 255 - map->data[index];
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
    if (!m_guider.isGuidanceInitialized()){
        return;
    }

    Pose2 pub_pose;
    GuidanceManager::RobotStatus cur_state = m_guider.getRobotStatus();
    if (cur_state == GuidanceManager::RobotStatus::ARRIVED_NODE || cur_state == GuidanceManager::RobotStatus::ARRIVED_GOAL 
    || cur_state == GuidanceManager::RobotStatus::READY || cur_state == GuidanceManager::RobotStatus::NO_PATH)
    {          
        if(!m_pub_flag)
        {
            // if (makeSubgoal(pub_pose))
            if (makeSubgoal3(pub_pose))  // Robot's coordinate  
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
                makeSubgoal3(pub_pose); // TODO: delete this afterwards in final version. Uncomment below
            //     ROS_INFO("Duration seconds: %d", duration.sec);
            //     if (cur_state == GuidanceManager::RobotStatus::NO_PATH) //if robot cannot generate path with the subgoal
            //     {
            //         if (reselectDrivablePoint(pub_pose))  // make undrivable black, then makesubgoal
            //         {
            //             ROS_INFO("reselectDrivablePoint!: %f, %f", pub_pose.x, pub_pose.y);
            //             m_guider.m_subgoal_pose = pub_pose;
            //         }                        
            //     }
            //     pub_pose = m_guider.m_subgoal_pose;
            //     geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(m_cur_head_node_id, pub_pose);
            //     pub_subgoal.publish(rosps);
            //     ROS_INFO("==============================================================\n");
            //     ROS_INFO("SubGoal published, again!: %f, %f<=====================", pub_pose.x, pub_pose.y);
            //     m_begin_time = ros::Time::now();
            // 
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
    // GuidanceManager::ExtendedPathElement cur_guide = m_guider.getCurExtendedPath();
    // GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();

    // Pose2 cur_node_metric = Point2(cur_guide);
    // Pose2 next_node_metric = Point2(next_guide);
    // ROS_INFO("[makeSubgoal] Heading %zd, node_metric.x: %f, y:%f",next_guide.cur_node_id, next_node_metric.x, next_node_metric.y);
    // pub_pose = next_node_metric;

    // //load robot info 
    // m_robotmap_mutex.lock();
    // cv::Mat onlinemap = m_robotmap_image;
    // Pose2 robot_dx_metric = m_guider.m_robot_pose;
    // m_robotmap_mutex.unlock();

    // cv::Mat robotmap;
    // cv::flip(onlinemap, robotmap,0);
    // if (onlinemap.empty()) //on robot occumap
    // {
    //     ROS_INFO("No occumap");
    //     robotmap = cv::imread(m_robotmap_path);
    // }

    // if (robotmap.empty())
    //     return false;

    // Pose2 dg_pose = m_localizer->getPose();
    // Pose2 dg_px = cvtMetric2Pixel(dg_pose);
    // Point2 robot_px = cvtMetric2Pixel(robot_dx_metric);
    // Point2 node_px_dx = cvtMetric2Pixel(next_node_metric);

    // //save image
    // cv::Mat colormap;
    // if (robotmap.channels() == 1)
    //     cv::cvtColor(robotmap, colormap, cv::COLOR_GRAY2BGR); 

    // Point2 node_px_new, node_px_moved;
    // ROS_INFO("[makeSubgoal] %d", m_guider.getCurGuideIdx());
    // //here, select new goal
    // if (m_cur_head_node_id != next_guide.cur_node_id)
    // {
    //     //check wheter pub_pose is undrivable find drivable point  
    //     cv::Mat img_erode;
    //     erode(robotmap, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);
    //     if (img_erode.at<uchar>((int)node_px_dx.y, (int)node_px_dx.x) < 210) //if node is in black area
    //     {
    //         ROS_INFO("Current node is in dark area");
    //         if(findDrivableinLine(img_erode, robot_px, node_px_dx, pub_pose))
    //         {
    //             node_px_moved = cvtMetric2Pixel(pub_pose);
    //         }
    //         else          
    //         {
    //             if(findExtendedDrivablePoint(img_erode, robot_px, node_px_dx, pub_pose))
    //             {
    //                 node_px_moved = cvtMetric2Pixel(pub_pose);
    //             }
    //             else
    //             {
    //                 ROS_INFO("Tried all. Failed to provide drivable goal.");
    //                 return false;
    //             }                
    //         } 
    //     }

    //     ROS_INFO("Found subggoal: <%f, %f>", pub_pose.x, pub_pose.y);
    //     m_prev_node_metric = m_cur_node_metric;
    //     m_cur_node_metric = next_node_metric;
    //     m_prev_node_id = m_cur_head_node_id;
    //     m_cur_head_node_id = next_guide.cur_node_id;
    //     m_prev_robot_metric = robot_dx_metric;
                
    //     ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    //     ///save image 
    //     cv::circle(colormap, robot_px, 10, cv::Vec3b(0, 255, 0), 2);
    //     cv::line(colormap, robot_px, node_px_dx, cv::Vec3b(0, 0, 255), 2);
    //     cv::circle(colormap, dg_px, 10, cv::Vec3b(255, 0, 0), 2);
    //     cv::circle(colormap, node_px_dx, 10, cv::Vec3b(0, 0, 255), 2);
    //     cv::circle(colormap, node_px_moved, 10, cv::Vec3b(0, 255, 255), 2);
    //     Point2 heading;
    //     heading.x = robot_px.x + 20 * cos(-robot_dx_metric.theta);
    //     heading.y = robot_px.y + 20 * sin(-robot_dx_metric.theta);
    //     cv::line(colormap, robot_px, heading, cv::Vec3b(0, 255, 0), 2);
    //     imwrite("../../../test_image.png", colormap);
    // }

    return true;
}

bool DGRobot::findAlternativePath(Pose2& alternative_point, Pose2 robot_pose, Pose2 next_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_nextnode, int num_alternatives, cv::Mat& colormap, double& min_dist_to_next_node){
    /*
    Alternative candidates sampled uniformly on a circle. Angle of each sample is based on robot origin.  
    */
    
    // find alternative that is drivable and the closest to the goal
    min_dist_to_next_node=1000000;  // 100 km. 
    Pose2 valid_point_with_min_dist;
    double new_theta = 0;

    // iterate over all alternatives
    for (int i=0; i < num_alternatives; i++){
        Pose2 alternative_pub_pose;

        // a new alternative
        alternative_pub_pose.x = robot_pose.x +  sub_goal_distance * cos(cx::cvtDeg2Rad(new_theta));
        alternative_pub_pose.y = robot_pose.y +  sub_goal_distance * sin(cx::cvtDeg2Rad(new_theta));
        // ROS_INFO("alternative_pub_pose x %f, y %f", alternative_pub_pose.x, alternative_pub_pose.y);
        cv::drawMarker(colormap, cvtRobottoMapcoordinate(alternative_pub_pose), cv::Vec3b(255, 255, 0), 1, 10, 2);  // cyan small cross

        // distance from alternative_pub_pose to next_node_robot
        double dist_to_next_node = norm(next_node_robot-alternative_pub_pose);
        
        // if alternative is drivable and distance to next_node is less than the min distance
        // if (isSubPathDrivable(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node){  
        // if (isSubPathDrivablev2(robotmap_erode, alternative_pub_pose) && dist_to_next_node < min_dist_to_next_node){ 
        if (isSubPathDrivablev3(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node){  
            min_dist_to_next_node=dist_to_next_node;
            valid_point_with_min_dist=alternative_pub_pose;
            ROS_INFO("Success finding better sub goal");
        }

        // go to the next alternative
        new_theta += (360/num_alternatives);

    }
    if (min_dist_to_next_node == 1000000){
        return false; // can't find alternative :(
    }
    else{  // can find alternative :)
        alternative_point.x = valid_point_with_min_dist.x;
        alternative_point.y = valid_point_with_min_dist.y;
        return true;
    }
}

bool DGRobot::findAlternativePathv2(Pose2& alternative_point, Pose2 robot_pose, Pose2 next_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_nextnode, int num_alternatives, cv::Mat& colormap, double angle_range, double& min_dist_to_next_node){
    /*
    Alternative candidates sampled from -angle_range to angle_range. Angle_range 180 is similar (but not same) to findAlternativePath (difference: angle of sample) 
    */

    // Find destination angle based on robot coordinate
    Pose2 dummy;
    dummy.x = robot_pose.x - 10.0;
    dummy.y = robot_pose.y;
    int next_node_theta = m_guider.getDegree(dummy, robot_pose, next_node_robot);
    
    // find alternative that is drivable and the closest to the goal
    min_dist_to_next_node=1000000;  // 100 km. 
    Pose2 valid_point_with_min_dist;
    double new_theta = -angle_range + (double)next_node_theta;

    double final_excluded_theta;
    if (angle_range == 180){ //if circle, angle_range*2: don't include the final 360 degree
        final_excluded_theta = angle_range*2;
    }
    else{  // if not circle, include the final angle_range*2, so (angle_range*2 + (angle_range*2/num_alternatives))
        final_excluded_theta = angle_range*2 + (angle_range*2/num_alternatives);
    }

    // bool isdistlessthanmindist=false;
    // bool isanypathdrivable=false;
    // iterate over all alternatives
    for (int i=0; i < num_alternatives; i++){
        Pose2 alternative_pub_pose;

        // a new alternative
        alternative_pub_pose.x = robot_pose.x +  sub_goal_distance * cos(cx::cvtDeg2Rad(new_theta));
        alternative_pub_pose.y = robot_pose.y +  sub_goal_distance * sin(cx::cvtDeg2Rad(new_theta));
        // ROS_INFO("alternative_pub_pose x %f, y %f", alternative_pub_pose.x, alternative_pub_pose.y);
        cv::drawMarker(colormap, cvtRobottoMapcoordinate(alternative_pub_pose), cv::Vec3b(255, 255, 0), 1, 10, 2);  // cyan small cross

        // distance from alternative_pub_pose to next_node_robot
        double dist_to_next_node = norm(next_node_robot-alternative_pub_pose);
        
        // if alternative is drivable and distance to next_node is less than the min distance
        // if (isSubPathDrivable(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node){  
        // if (isSubPathDrivablev2(robotmap_erode, alternative_pub_pose) && dist_to_next_node < min_dist_to_next_node){ 
        if (isSubPathDrivablev3(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node){  
            min_dist_to_next_node=dist_to_next_node;
            valid_point_with_min_dist=alternative_pub_pose;
            ROS_INFO("Success finding better sub goal");
        }
        // else{
        //     if (isSubPathDrivablev3(robotmap_erode, alternative_pub_pose, robot_pose)){
        //         isanypathdrivable = true;
        //     }
        //     if (dist_to_next_node < min_dist_to_next_node){
        //         isdistlessthanmindist=true;
        //     }
        // }

        // go to the next alternative
        new_theta += (final_excluded_theta/num_alternatives);

    }
    if (min_dist_to_next_node == 1000000){
        // if (!isanypathdrivable) ROS_INFO("Can't find alternative because of no path drivable");
        // if (!min_dist_to_next_node) ROS_INFO("Can't find alternative because of min dist is still same.. HAVE TO CHANGE THE INITIAL VALUE");
        return false; // can't find alternative :(
    }
    else{  // can find alternative :)
        alternative_point.x = valid_point_with_min_dist.x;
        alternative_point.y = valid_point_with_min_dist.y;
        return true;
    }
}

void DGRobot::rotatePose(Pose2& P, double theta_rot){
    Pose2 P_ = P;
    P.x = P_.x * cos(theta_rot) - P_.y * sin(theta_rot);
    P.y = P_.x * sin(theta_rot) + P_.y * cos(theta_rot);
    P.theta = P_.theta + P.theta;  // TODO: don't know if this is correct
}

Pose2 DGRobot::cvtDGtoDXcoordinate(Pose2 P, Pose2 P_DG, Pose2 P_DX)
{
    Pose2 newP;

    // P-P_DG
    newP.x = P.x - P_DG.x;
    newP.y = P.y - P_DG.y;

    // R(P_DG.theta-P_DX.theta) (P-P_DG)
    double theta_rot = P_DG.theta - P_DX.theta;
    rotatePose(newP, theta_rot);

    // R(P_DG.theta-P_DX.theta) (P-P_DG) + P_DX
    newP.x = newP.x + P_DX.x;
    newP.y = newP.y + P_DX.y;

    // transformed theta
    newP.theta = P.theta - P_DG.theta + P_DX.theta;

    return newP;
}

Pose2 DGRobot::cvtMaptoRobotcoordinate(Pose2 P)
{
    // convert map coordinate (in pixel) to robot coordinate (in meter)

    Pose2 newP;

    newP.x = P.x * m_dx_map_meter_per_pixel + m_robot_origin.x; 
    newP.y = P.y * m_dx_map_meter_per_pixel + m_robot_origin.y; 

    return newP;
}

Pose2 DGRobot::cvtRobottoMapcoordinate(Pose2 P)
{
    // convert robot coordinate (in meter) to map coordinate (in pixel)
    Pose2 newP;

    newP.x = P.x / m_dx_map_meter_per_pixel - m_robot_origin.x / m_dx_map_meter_per_pixel;
    newP.y = P.y / m_dx_map_meter_per_pixel - m_robot_origin.y / m_dx_map_meter_per_pixel;

    return newP;
}

bool DGRobot::isSubPathDrivable(cv::Mat robotmap, Pose2 pointA, Pose2 pointB){
    // if ANY point between pointA and pointB is below drivable threshold, then not drivable

    // pointA and pointB in robot coordinate
    Pose2 pointA_px = cvtRobottoMapcoordinate(pointA);
    Pose2 pointB_px = cvtRobottoMapcoordinate(pointB);
    cv::LineIterator line_it(robotmap, cv::Point(pointA_px.x, pointA_px.y), cv::Point(pointB_px.x, pointB_px.y), 8);
    // ROS_INFO("from pointA <%f, %f> to pointB <%f, %f> Num of line iterator %d", pointA_px.x, pointA_px.y, pointB_px.x, pointB_px.y, line_it.count);
    for(int i = 0; i < line_it.count; i++, ++line_it)
    {
        cv::Point point_px = line_it.pos();
        int value = robotmap.at<uchar>(point_px.y, point_px.x);
        if (value < m_drivable_threshold)  // any non drivable area
        {
            // ROS_INFO("non drivable value <%d>", value);
            return false;
        }
    }
    return true;
}


bool DGRobot::isSubPathDrivablev2(cv::Mat robotmap, Pose2 dest_point){
    // as long as the dest_point is in drivable area, then drivable

    Pose2 point_px = cvtRobottoMapcoordinate(dest_point);

    int value = robotmap.at<uchar>(point_px.y, point_px.x); 
    if (value < m_drivable_threshold){
        return false;
    }
    return true;
}

bool DGRobot::isSubPathDrivablev3(cv::Mat robotmap, Pose2 dest_point, Pose2 source_point){
    // if current position is drivable, isSubPathDrivable
    // else, ignore the black continuous to the source_point first, then after getting to drivable area (new_source_point), isSubPathDrivable

    // pointA and pointB in robot coordinate
    Pose2 dest_point_px = cvtRobottoMapcoordinate(dest_point);
    Pose2 source_point_px = cvtRobottoMapcoordinate(source_point);

    if (robotmap.at<uchar>(source_point_px.y, source_point_px.x) < m_drivable_threshold){  // current position is NOT drivable
        cv::LineIterator line_it(robotmap, cv::Point(dest_point_px.x, dest_point_px.y), cv::Point(source_point_px.x, source_point_px.y), 8);
            
        for(int i = 0; i < line_it.count; i++, ++line_it)
        {
            cv::Point point_px = line_it.pos();
            int value = robotmap.at<uchar>(point_px.y, point_px.x);
            if (value >= m_drivable_threshold)  // first encounter of drivable area
            {
                Pose2 point_px_pose;
                point_px_pose.x = point_px.x;
                point_px_pose.y = point_px.y;
                Pose2 new_source_point = cvtMaptoRobotcoordinate(point_px_pose);
                return isSubPathDrivable(robotmap, dest_point, new_source_point);
            }
        }

        ROS_INFO("No encounter of the first drivable area");
        return false;  // all non-drivable until the destination
    }
    else{  // current position is drivable
        return isSubPathDrivable(robotmap, dest_point, source_point);
    }
}

bool DGRobot::makeSubgoal3(Pose2& pub_pose)  
{ 

    /////////////////////////////////////////////////
    ////////Code to play around with getDegree
    // Pose2 dummytest;
    // Pose2 dummydummy;
    // Pose2 dummyrobot;
    // dummyrobot.x = 0.0;
    // dummyrobot.y = 0.0;
    // dummytest.x = 1.0;
    // dummytest.y = -1.0;
    // dummydummy.x = 1.0;
    // dummydummy.y = 0.0;
    // ROS_INFO("[DUMMYYYYYYYYYYY] getdegree 1 %d", m_guider.getDegree(dummytest, dummyrobot, dummydummy));
    // ROS_INFO("[DUMMYYYYYYYYYYY] getdegree 2 %d", m_guider.getDegree(dummydummy, dummyrobot, dummytest));

    /////////////////////////////////////////////////
    ////////Code to check rotatePose()
    // Pose2 dummypose;
    // dummypose.x = 1;
    // dummypose.y = 0;
    // dummypose.theta = 0;
    // ROS_INFO("[DUMMYYYYYYYYYYY] BEFORE node_metric.x: %f, y:%f", dummypose.x, dummypose.y);
    // rotatePose(dummypose, cx::cvtDeg2Rad(-90));
    // ROS_INFO("[DUMMYYYYYYYYYYY] AFTER node_metric.x: %f, y:%f", dummypose.x, dummypose.y);
    
    // in DeepGuider coordinate
    GuidanceManager::ExtendedPathElement cur_guide = m_guider.getCurExtendedPath();
    GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();
    Pose2 dg_pose = m_localizer->getPose();
    
    ROS_INFO("[makeSubgoal3] DG Pose node_robot.x: %f, y:%f",dg_pose.x, dg_pose.y);
    
    Pose2 cur_node_dg = Point2(cur_guide);
    Pose2 next_node_dg = Point2(next_guide);

    //load robot info
    m_robotmap_mutex.lock();
    cv::Mat onlinemap = m_robotmap_image;  // <map image>
    Pose2 robot_pose = m_guider.m_robot_pose;  // robot pose in robot's coordinate
    m_robotmap_mutex.unlock();

    ROS_INFO("[makeSubgoal3] robot_pose node_robot.x: %f, y:%f",robot_pose.x, robot_pose.y);

    Pose2 dg_pose_robot = cvtDGtoDXcoordinate(dg_pose, dg_pose, robot_pose);  // dg_pose in robot's coordinate
    
    cv::Mat robotmap = onlinemap;
    if (onlinemap.empty()) //on robot occumap
    {
        ROS_INFO("No occumap");
        robotmap = cv::imread(m_robotmap_path, cv::IMREAD_GRAYSCALE);  // if no online map, read offline map
    }
    cv::Size robotmap_size = robotmap.size();
    m_robot_origin.x = -m_dx_map_origin_pixel.x * m_dx_map_meter_per_pixel;
    m_robot_origin.y = -m_dx_map_origin_pixel.y * m_dx_map_meter_per_pixel;
    
    // imwrite("../../../robotmap.png", robotmap);  

    if (robotmap.empty())
        return false;

    // smooth robotmap
    cv::Mat robotmap_smooth=robotmap.clone();
    ///////////////////////////////////////////////////////////////////////////////////
    // // Plan A-3: comment below (except the last line)
    // // Plan B-3: uncomment below (except the last line)
    // int smoothing_filter = 3; // filter size. Must be odd
    // cv::medianBlur(robotmap, robotmap_smooth, smoothing_filter);

    // // erode robotmap
    // int erode_value = 10;
    int erode_value = 6;  // Plan A-3: uncomment. Plan B-3: comment
    ///////////////////////////////////////////////////////////////////////////////
    cv::Mat robotmap_erode=robotmap_smooth.clone();
    erode(robotmap_smooth, robotmap_erode, cv::Mat::ones(cv::Size(erode_value, erode_value), CV_8UC1), cv::Point(-1, -1), 1);

    //save image
    cv::Mat colormap=robotmap_erode;
    cv::Mat clean_colormap=robotmap;
    cv::cvtColor(robotmap_erode, colormap, cv::COLOR_GRAY2BGR); 
    cv::cvtColor(robotmap, clean_colormap, cv::COLOR_GRAY2BGR); 

    imwrite("../../../smootheroderobotmap.png", robotmap_erode);  

    ROS_INFO("[makeSubgoal3] CurGuideIdx %d", m_guider.getCurGuideIdx());

    int diff_deg_robot;
    double diff_dist_robot, new_theta;
    new_theta = robot_pose.theta;

    //convert variables with DG coordinate to DX coordinate
    Pose2 prev_node_robot = cvtDGtoDXcoordinate(m_prev_node_dg, dg_pose, robot_pose);
    Pose2 cur_node_robot = cvtDGtoDXcoordinate(cur_node_dg, dg_pose, robot_pose);  // Note: cur_node_dg is same with m_cur_node_dg except when m_cur_node_dg hasn't been assigned for the first time
    Pose2 next_node_robot = cvtDGtoDXcoordinate(next_node_dg, dg_pose, robot_pose);
    // Pose2 prev_node_robotreal = cvtDg2Dx(m_prev_node_dg);
    // Pose2 cur_node_robotreal = cvtDg2Dx(cur_node_dg);  
    // Pose2 next_node_robotreal = cvtDg2Dx(next_node_dg);

    // drawing in colormap
    Point2 cur_node_robot_px = cvtRobottoMapcoordinate(cur_node_robot); //green star
    Point2 next_node_robot_px = cvtRobottoMapcoordinate(next_node_robot); //blue star
    // Point2 prev_node_robotreal_px = cvtRobottoMapcoordinate(prev_node_robotreal); //red diamond
    // Point2 cur_node_robotreal_px = cvtRobottoMapcoordinate(cur_node_robotreal); //green diamond
    // Point2 next_node_robotreal_px = cvtRobottoMapcoordinate(next_node_robotreal); //blue diamond
    cv::drawMarker(colormap, cur_node_robot_px, cv::Vec3b(0, 255, 0), 2, 20, 5);
    cv::drawMarker(colormap, next_node_robot_px, cv::Vec3b(255, 0, 0), 2, 40, 5);
    // cv::drawMarker(colormap, prev_node_robotreal_px, cv::Vec3b(0, 0, 255), 3, 40, 5);
    // cv::drawMarker(colormap, cur_node_robotreal_px, cv::Vec3b(0, 255, 0), 3, 20, 5);
    // cv::drawMarker(colormap, next_node_robotreal_px, cv::Vec3b(255, 0, 0), 3, 40, 5);

    
    //here, select new node goal
    if (m_cur_head_node_id != next_guide.cur_node_id)  // new dg_next_node_robot
    {
        if(m_prev_node_id != 0) //NOT initial start (when m_prev_node_robot is not 0 anymore)
        {            
            ROS_INFO("[makeSubgoal3] prev_id: %zd, cur_id: %zd, next_id: %zd", m_prev_node_id, m_cur_head_node_id, next_guide.cur_node_id);
            
            // find theta (Note: doesn't matter using robot/dg. Value same)
            diff_deg_robot = m_guider.getDegree(prev_node_robot, cur_node_robot, next_node_robot); // Note: probably unnecessary variable
            // display theta and 
            // string disp = "degree = " + std::to_string(diff_deg_robot);
            // cv::putText(colormap, disp, cur_node_robot_px, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 255, 0),5);
            // disp = "   rp = " + std::to_string(cx::cvtRad2Deg(robot_pose.theta));
            // cv::putText(colormap, disp, cv::Point(500, 50) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 255, 0),5);
            // disp = "   dp = " + std::to_string(cx::cvtRad2Deg(dg_pose.theta));
            // cv::putText(colormap, disp, cv::Point(500, 120) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 255, 0),5);
            // imwrite("../../../test_image_withtext.png", colormap);

            // visualize prev node here only when update the new node
            Point2 prev_node_robot_px = cvtRobottoMapcoordinate(prev_node_robot); //red star
            cv::drawMarker(colormap, prev_node_robot_px, cv::Vec3b(0, 0, 255), 2, 40, 5);

        }
        else  
        {
            diff_deg_robot = 0;  // go straight. Note: probably unnecessary variable
        }

        // update global variable
        m_prev_node_dg = m_cur_node_dg;
        m_cur_node_dg = next_node_dg;
        m_prev_node_id = m_cur_head_node_id;
        m_cur_head_node_id = next_guide.cur_node_id;
        m_prev_robot_pose = robot_pose;   
        m_robotarrived_but_nodenotyetupdated = false; // node updated 
    }
    else{  // if not new node
        diff_deg_robot = m_guider.getDegree(cur_node_robot, robot_pose, next_node_robot);  // Here, cur_node_robot = prev_node_robot
    }

    //distance from robot to the next node
    double dist_robot_to_nextnode = norm(next_node_robot-robot_pose);
    // // new_theta for robot pose (facing the next node). Note: probably not used as for subgoal, we only need coordinate.
    // new_theta = robot_pose.theta + cx::cvtDeg2Rad(diff_deg_robot);  // current pose + how much to turn based on diff_deg_robot
    ROS_INFO("[makeSubgoal3] dist_robot_to_nextnode: %f", dist_robot_to_nextnode);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // // Plan A-2: comment below
    // // Plan B-2: uncomment below (still have problem: what if the pub pose is not in drivable area? Try to go to next next node?)
    // if (dist_robot_to_nextnode < 1 || m_robotarrived_but_nodenotyetupdated){  // if too close to the next node but DG hasn't update to the next node OR already done the solution before but node is still not yet updated
    //     m_robotarrived_but_nodenotyetupdated = true;

    //     double dist_cur_to_nextnode = norm(next_node_robot-cur_node_robot);
    //     double p = min(1.0 / dist_cur_to_nextnode, 1.0); // what is (e.g.) 1 meter ratio with the distance between cur and next node  
        
    //     pub_pose.x = cur_node_robot.x + (1.0+p) * (next_node_robot.x - cur_node_robot.x);
    //     pub_pose.y = cur_node_robot.y + (1.0+p) * (next_node_robot.y - cur_node_robot.y);

    //     ROS_INFO("Found subggoal: <%f, %f>", pub_pose.x, pub_pose.y);  // OUTPUT.. care about pub_pose in robot's coordinate    
    //     Pose2 pub_pose_px = cvtRobottoMapcoordinate(pub_pose);
    //     cv::circle(colormap, pub_pose_px, 20, cv::Vec3b(255, 0, 255), 5);  // small purple circle
    //     cv::circle(colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2);  // with robot real size
    //     cv::circle(clean_colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2);  // with robot real size
    
    //     bool isdrivable = isSubPathDrivablev3(robotmap_erode, pub_pose, robot_pose);
    //     if (!isdrivable){
    //         Pose2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose);
    //         cv::putText(colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255),5);
    //         cv::putText(clean_colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255),5);
    //     }
    //     ///////////////////////////////////////////////////
    //     if (m_save_video){
    //         //record image   
    //         // ///save image 
    //         Point2 dg_pose_robot_px = cvtRobottoMapcoordinate(dg_pose_robot);
    //         // Point2 dx_pose_robot_px = cvtRobottoMapcoordinate(robot_pose);  // dx_pose_robot_px = dg_pose_robot_px
            
    //         cv::circle(colormap, dg_pose_robot_px, 20, cv::Vec3b(0, 255, 0), 5);
    //         cv::circle(colormap, dg_pose_robot_px, 5, cv::Vec3b(0, 255, 0), 2);  // with robot real size
    //         cv::circle(clean_colormap, dg_pose_robot_px, 5, cv::Vec3b(0, 255, 0), 2);  // with robot real size
    //         // cv::circle(colormap, dx_pose_robot_px, 20, cv::Vec3b(0, 0, 255), 5);

    //         Point2 robot_heading;
    //         robot_heading.x = dg_pose_robot_px.x + 20 * cos(robot_pose.theta);
    //         robot_heading.y = dg_pose_robot_px.y + 20 * sin(robot_pose.theta);
    //         cv::line(colormap, dg_pose_robot_px, robot_heading, cv::Vec3b(0, 255, 0), 5);
    //         ROS_INFO("robot_pose theta %f", robot_pose.theta);
            
    //         cv::drawMarker(colormap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), 0, 50, 10);
    //         imwrite("../../../test_image.png", colormap);
            
    //         // record video
    //         cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);  
    //         cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
    //         colormap.copyTo(roi);
    //         m_video_gui << videoFrame;
            
    //         cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
    //         int x = dg_pose_robot_px.x-400; 
    //         int y = dg_pose_robot_px.y-400;
    //         if (x+800 >= colormap.cols) x = colormap.cols - 800 - 1;
    //         if (x<=1) x = 1;
    //         if (y+800 >= colormap.rows) y = colormap.rows - 800 - 1;
    //         if (y<=1) y = 1;
    //         cv::Mat roicrop(colormap, cv::Rect(x, y, videoFrameCrop.cols,  videoFrameCrop.rows));
    //         roicrop.copyTo(videoFrameCrop);
    //         m_video_crop << videoFrameCrop;
    //         // imwrite("../../../online_crop.png", videoFrameCrop);

    //         videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
    //         cv::Mat maproicrop(clean_colormap, cv::Rect(x, y, videoFrameCrop.cols,  videoFrameCrop.rows));
    //         maproicrop.copyTo(videoFrameCrop);
    //         m_mapvideo_crop << videoFrameCrop;
    //     }

    //     if (isdrivable) {return true;}
    //     else {return false;}
    // }
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // if not too close to the next node
    // get a point between robot pose and next node. 1 meter from the robot position to the direction of next_node_robot
    double sub_goal_distance = 4.0;  // in meter. Distance from robot to sub goal. // 3.0 for visualization
    double p = min(sub_goal_distance / dist_robot_to_nextnode, 1.0); // what is (e.g.) 1 meter ratio with the distance to the next node  
    ROS_INFO("[makeSubgoal3] p: %f", p);
    pub_pose.x = robot_pose.x + p * (next_node_robot.x - robot_pose.x);
    pub_pose.y = robot_pose.y + p * (next_node_robot.y - robot_pose.y);

    // draw the pub_pose from the first step (regardless drivable or not)
    cv::drawMarker(colormap, cvtRobottoMapcoordinate(pub_pose), cv::Vec3b(255, 0, 255), 1, 10, 2);  // purple small cross

    // is there non drivable area from current position to next node?
    // if (!isSubPathDrivable(robotmap_erode, pub_pose, robot_pose)){  // if not drivable
    // if (!isSubPathDrivablev2(robotmap_erode, pub_pose)){  // if not drivable
    if (!isSubPathDrivablev3(robotmap_erode, pub_pose, robot_pose)){  // if not drivable
        std::vector<double> alternative_sub_goal_distances = {4.0, 3.0, 2.0, 1.0, 0.5, 0.1}; // {3.0} for visualization
        std::vector<int> num_alternatives = {36, 24, 16, 12, 8, 8};  // {4} for visualization
        double angle_range = 180;  // max 180 (whole circle)
        bool isAlternativeFound;
        double min_dist_temp=1000000;  // 100 km
        
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Plan A-1
        for (int i; i < num_alternatives.size(); i++){
            isAlternativeFound = findAlternativePathv2(pub_pose, robot_pose, next_node_robot, robotmap_erode, alternative_sub_goal_distances.at(i), dist_robot_to_nextnode, num_alternatives.at(i), colormap, angle_range, min_dist_temp);
            if (isAlternativeFound){
                break;
            }
        }
        // /////////////////////////////////////////////////////////////////////////////////////////////
        // // Plan B-1
        // Pose2 pub_pose_temp;
        // double min_dist_best = 1000000;  // 100 km

        // for (int i; i < num_alternatives.size(); i++){   
        //     double min_dist_temp;     
        //     isAlternativeFound = findAlternativePathv2(pub_pose_temp, robot_pose, next_node_robot, robotmap_erode, alternative_sub_goal_distances.at(i), dist_robot_to_nextnode, num_alternatives.at(i), colormap, angle_range, min_dist_temp);
        //     if (isAlternativeFound && min_dist_temp < min_dist_best){
        //         pub_pose = pub_pose_temp;
        //         min_dist_best = min_dist_temp;
        //     }
        // }
        // if (min_dist_best == 1000000){
        //     isAlternativeFound = false;
        // }
        // else{
        //     isAlternativeFound = true;
        // }
        // /////////////////////////////////////////////////////////////////////////////////////////////

        if (!isAlternativeFound){
            ROS_INFO("can't find sub goal :(");
            if (m_save_video){
                Pose2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose);
                cv::putText(colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255),5);
                cv::putText(clean_colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255),5);

                //record image   
                // ///save image 
                Point2 dg_pose_robot_px = cvtRobottoMapcoordinate(dg_pose_robot);
                // Point2 dx_pose_robot_px = cvtRobottoMapcoordinate(robot_pose);  // dx_pose_robot_px = dg_pose_robot_px
                
                cv::circle(colormap, dg_pose_robot_px, 20, cv::Vec3b(0, 255, 0), 5);
                cv::circle(colormap, dg_pose_robot_px, 5, cv::Vec3b(0, 255, 0), 2);  // with robot real size
                cv::circle(clean_colormap, dg_pose_robot_px, 5, cv::Vec3b(0, 255, 0), 2);  // with robot real size
                // cv::circle(colormap, dx_pose_robot_px, 20, cv::Vec3b(0, 0, 255), 5);

                Point2 robot_heading;
                robot_heading.x = dg_pose_robot_px.x + 20 * cos(robot_pose.theta);
                robot_heading.y = dg_pose_robot_px.y + 20 * sin(robot_pose.theta);
                cv::line(colormap, dg_pose_robot_px, robot_heading, cv::Vec3b(0, 255, 0), 5);
                ROS_INFO("robot_pose theta %f", robot_pose.theta);
                
                cv::drawMarker(colormap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), 0, 50, 10);
                imwrite("../../../test_image.png", colormap);
                
                // record video
                cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);  
                cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
                colormap.copyTo(roi);
                m_video_gui << videoFrame;

                cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
                int x = dg_pose_robot_px.x-400; 
                int y = dg_pose_robot_px.y-400;
                if (x+800 >= colormap.cols) x = colormap.cols - 800 - 1;
                if (x<=1) x = 1;
                if (y+800 >= colormap.rows) y = colormap.rows - 800 - 1;
                if (y<=1) y = 1;
                cv::Mat roicrop(colormap, cv::Rect(x, y, videoFrameCrop.cols,  videoFrameCrop.rows));
                roicrop.copyTo(videoFrameCrop);
                m_video_crop << videoFrameCrop;

                videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
                cv::Mat maproicrop(clean_colormap, cv::Rect(x, y, videoFrameCrop.cols,  videoFrameCrop.rows));
                maproicrop.copyTo(videoFrameCrop);
                m_mapvideo_crop << videoFrameCrop;
            }

            return false; // can't find alternative :(
        }
    }
    

    ROS_INFO("Found subggoal: <%f, %f>", pub_pose.x, pub_pose.y);  // OUTPUT.. care about pub_pose in robot's coordinate
    Pose2 pub_pose_px = cvtRobottoMapcoordinate(pub_pose);
    cv::circle(colormap, pub_pose_px, 20, cv::Vec3b(255, 0, 255), 5);  // small purple circle
    cv::circle(colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2);  // with robot real size
    cv::circle(clean_colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2);  // with robot real size
        
    ///////////////////////////////////////////////////
    if (m_save_video){
        //record image   
        // ///save image 
        Point2 dg_pose_robot_px = cvtRobottoMapcoordinate(dg_pose_robot);
        // Point2 dx_pose_robot_px = cvtRobottoMapcoordinate(robot_pose);  // dx_pose_robot_px = dg_pose_robot_px
        
        cv::circle(colormap, dg_pose_robot_px, 20, cv::Vec3b(0, 255, 0), 5);
        cv::circle(colormap, dg_pose_robot_px, 5, cv::Vec3b(0, 255, 0), 2);  // with robot real size
        cv::circle(clean_colormap, dg_pose_robot_px, 5, cv::Vec3b(0, 255, 0), 2);  // with robot real size
        // cv::circle(colormap, dx_pose_robot_px, 20, cv::Vec3b(0, 0, 255), 5);

        Point2 robot_heading;
        robot_heading.x = dg_pose_robot_px.x + 20 * cos(robot_pose.theta);
        robot_heading.y = dg_pose_robot_px.y + 20 * sin(robot_pose.theta);
        cv::line(colormap, dg_pose_robot_px, robot_heading, cv::Vec3b(0, 255, 0), 5);
        ROS_INFO("robot_pose theta %f", robot_pose.theta);
        
        cv::drawMarker(colormap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), 0, 50, 10);
        imwrite("../../../test_image.png", colormap);
        
        // record video
        cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);  
        cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
        colormap.copyTo(roi);
        m_video_gui << videoFrame;
        
        cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
        int x = dg_pose_robot_px.x-400; 
        int y = dg_pose_robot_px.y-400;
        if (x+800 >= colormap.cols) x = colormap.cols - 800 - 1;
        if (x<=1) x = 1;
        if (y+800 >= colormap.rows) y = colormap.rows - 800 - 1;
        if (y<=1) y = 1;
        cv::Mat roicrop(colormap, cv::Rect(x, y, videoFrameCrop.cols,  videoFrameCrop.rows));
        roicrop.copyTo(videoFrameCrop);
        m_video_crop << videoFrameCrop;
        // imwrite("../../../online_crop.png", videoFrameCrop);

        videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);  
        cv::Mat maproicrop(clean_colormap, cv::Rect(x, y, videoFrameCrop.cols,  videoFrameCrop.rows));
        maproicrop.copyTo(videoFrameCrop);
        m_mapvideo_crop << videoFrameCrop;
    }
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
        if (image.at<uchar>((int)jump_px.y, (int)jump_px.x) > m_drivable_threshold)
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
            if (image.at<uchar>((int)jump_px.y, (int)jump_px.x) < 210)
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
   double ori_x_rotated = (m_dx_map_origin_pixel.x - m_dx_map_ref_pixel.x) * m_dx_map_meter_per_pixel;
   double ori_y_rotated = (m_dx_map_origin_pixel.y - m_dx_map_ref_pixel.y) * m_dx_map_meter_per_pixel;
 
   //rotate ori_rotated to robot's map
   printf("Robot map ori_rotated: x = %f, y = %f\n", ori_x_rotated, ori_y_rotated);
   double ori_x = ori_x_rotated * cos(m_dx_map_rotation_radian) - ori_y_rotated * sin(m_dx_map_rotation_radian);
   double ori_y = ori_x_rotated * sin(m_dx_map_rotation_radian) + ori_y_rotated * cos(m_dx_map_rotation_radian);
   printf("Robot map rotation Deg = %f\n", cx::cvtRad2Deg(m_dx_map_rotation_radian));
   printf("Robot map origin-ref: x = %lf, y = %lf\n", ori_x, ori_y);
 
   //calculate m_dx_map_origin_utm
   dg::UTMConverter converter;
   dg::Point2UTM dx_map_ref_utm = converter.cvtLatLon2UTM(m_dx_map_ref_latlon);
   printf("Robot map ref lat = %lf, lon = %lf\n", m_dx_map_ref_latlon.lat, m_dx_map_ref_latlon.lon);
   printf("Robot map ref utm: x = %lf, y = %lf\n", dx_map_ref_utm.x, dx_map_ref_utm.y);
   m_dx_map_origin_utm.x = dx_map_ref_utm.x + ori_x;
   m_dx_map_origin_utm.y = dx_map_ref_utm.y + ori_y;
   m_dx_map_origin_utm.zone = dx_map_ref_utm.zone;
   m_dx_map_origin_latlon = converter.cvtUTM2LatLon(m_dx_map_origin_utm);
   printf("Robot map origin pixel: x = %lf, y = %lf\n", m_dx_map_origin_pixel.x, m_dx_map_origin_pixel.y);
   printf("Robot map origin utm: x = %lf, y = %lf\n", m_dx_map_origin_utm.x, m_dx_map_origin_utm.y);
   printf("Robot map origin lat = %lf, lon = %lf\n", m_dx_map_origin_latlon.lat, m_dx_map_origin_latlon.lon);
 
   m_dx_converter.setReference(m_dx_map_origin_latlon);
   m_dg_converter.setReference(m_dg_map_origin_latlon);
   m_dg_map_origin_utm = converter.cvtLatLon2UTM(m_dg_map_origin_latlon);
   printf("dg map origin lat = %lf, lon = %lf\n", m_dg_map_origin_latlon.lat, m_dg_map_origin_latlon.lon);
   printf("dg map origin utm: x = %f, y = %f\n", m_dg_map_origin_utm.x, m_dg_map_origin_utm.y);
 
   //display origin in gui
   Pose2 robot_origin_metric = m_map.toMetric(m_dx_map_origin_latlon);
   Point2 robot_origin = m_painter.cvtValue2Pixel(robot_origin_metric);
   cv::circle(m_map_image, robot_origin, 10, cv::Vec3b(0, 255, 255),10);
 
   Pose2 robot_ref_metric = m_map.toMetric(m_dx_map_ref_latlon);
   Point2 robot_ref = m_painter.cvtValue2Pixel(robot_ref_metric);
   cv::circle(m_map_image, robot_ref, 10, cv::Vec3b(255, 0, 0),10);
}


Point2 DGRobot::cvtDg2Dx(const Point2& dg_metric) const
{
   //from dg metric to utm
   dg::Point2UTM dg_utm;
   dg_utm.x = m_dg_map_origin_utm.x + dg_metric.x;
   dg_utm.y = m_dg_map_origin_utm.y + dg_metric.y;
   printf("[cvtDg2Dx]000000000 m_dg_map_origin_utm: <%f, %f>\n", m_dg_map_origin_utm.x, m_dg_map_origin_utm.y);
   printf("[cvtDg2Dx]000000000 dg_utm: <%f, %f>\n", dg_utm.x, dg_utm.y);
 
   //from dg_utm to dx map. but it's rotated.
   Point2 dg_dx_rotated;
   dg_dx_rotated.x = dg_utm.x - m_dx_map_origin_utm.x;
   dg_dx_rotated.y = dg_utm.y - m_dx_map_origin_utm.y;
   printf("[cvtDg2Dx]000000000 m_dx_map_origin_utm: <%f, %f>\n", m_dx_map_origin_utm.x, m_dx_map_origin_utm.y);
   printf("[cvtDg2Dx]000000000 dg_dx_rotated: <%f, %f>\n", dg_dx_rotated.x, dg_dx_rotated.y);
 
   //rotate utm to robot's coordinate
   Point2 dg_dx;
   double theta = -m_dx_map_rotation_radian;
   dg_dx.x = dg_dx_rotated.x * cos(theta) - dg_dx_rotated.y * sin(theta);
   dg_dx.y = dg_dx_rotated.x * sin(theta) + dg_dx_rotated.y * cos(theta);
   printf("[cvtDg2Dx]000000000 dg_dx: <%f, %f>\n", dg_dx.x, dg_dx.y);
 
   return dg_dx;  
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
  