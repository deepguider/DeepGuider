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
    void callbackDrawRobot(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::Subscriber sub_dg_status;    
    void callbackDGStatus(const dg_simple_ros::dg_status::ConstPtr& msg);

    // Topic publishers (sensor data)
    ros::Publisher pub_subgoal;
    void publishSubGoal();

    bool m_stop_running = false;
    
    //Robot parameters
    std::string m_robotmap_path = "data/Bucheon/bucheon_220922/occumap_bucheon.png";
    dg::LatLon m_dx_map_ref_latlon = dg::LatLon(37.5177542, 126.7651744);
    dg::Point2 m_dx_map_ref_pixel = dg::Point2(715, 650);
    double m_dx_map_meter_per_pixel = 0.1;
    double m_dx_map_rotation_radian = -0.8;
    double m_robotmap_scale = 10.0;
    cv::Point2d m_dx_map_origin_pixel = cv::Point2d(345, 1110);
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
    Point2 makeSubgoal();
    Point2 makeImagePixels(cv::Mat& image, Point2& robot_dx_pixel, Point2& node_dx_pixel);
    geometry_msgs::PoseStamped makeRosPubPoseMsg(ID nid, Point2 xy);
    Point2 findDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px);
    Point2 findDrivableNearPoint(cv::Mat &image, Point2 robot_px, Point2 node_px);
    Point2 findDrivableNearPixel(cv::Mat &image, Point2 robot_px, Point2 node_px);
    Point2 reselectDrivablePoint();
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
    // cv::Vec2d dg_ref_point = cv::Vec2d(m_map_ref_point.lat, m_map_ref_point.lon);
    // CX_LOAD_PARAM_COUNT(fn, "map_ref_point_latlon", dg_ref_point, n_read);
    // m_dg_map_origin_latlon = dg::LatLon(dg_ref_point[0], dg_ref_point[1]);

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
    sub_draw_robot = nh_dg.subscribe("/mcl3d/current/pose", 1, &DGRobot::callbackDrawRobot, this);

    // Initialize deepguider publishers
    pub_subgoal = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_subgoal", 1, true);
    
    // Initialize robot parameters
    initialize_DG_DX_conversion();

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
    if(m_video_recording) m_video_gui.release();
    if(m_video_recording) printf("\trecording closed\n");
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
    ROS_INFO_THROTTLE(1.0, "DrawRobot: %f,%f", robot_display.x, robot_display.y);
    cv::circle(m_map_image, robot_display, 3, cv::Vec3b(0, 255, 255));

    m_guider.m_robot_on_image = robot_display;

}

void DGRobot::publishSubGoal()
{
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return;
    
    Pose2 pub_pose = m_guider.m_subgoal_pose;
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
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
                pub_pose = makeSubgoal();
                if (pub_pose.x == 0 || pub_pose.y == 0)
                {
                    ROS_INFO_THROTTLE(1.0, "Cannot reselect point: %f, %f", pub_pose.x, pub_pose.y);
                    return;
                }
                geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pub_pose);
                pub_subgoal.publish(rosps);
                m_guider.m_subgoal_pose = pub_pose;
                // ROS_INFO_THROTTLE(1.0, "SubGoal published!:%d, %f, %f<=====================", cur_state, pub_pose.x, pub_pose.y);
                m_begin_time = ros::Time::now();
                m_pub_flag = true;
                // m_prev_state = cur_state;
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
                        pub_pose = reselectDrivablePoint();
                        if (pub_pose.x == 0 || pub_pose.y == 0)
                        {
                            ROS_INFO_THROTTLE(1.0, "Cannot reselect point: %f, %f", pub_pose.x, pub_pose.y);
                            return;
                        }
                        
                        ROS_INFO_THROTTLE(1.0, "reselectDrivablePoint!: %f, %f", pub_pose.x, pub_pose.y);
                    }
                    geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(nid, pub_pose);
                    pub_subgoal.publish(rosps);
                    ROS_INFO_THROTTLE(1.0, "SubGoal published, again!: %f, %f<=====================", pub_pose.x, pub_pose.y);
                    m_begin_time = ros::Time::now();
                }
            }
            
        }
        else //cur_state == GuidanceManager::RobotStatus::RUN_MANUAL || cur_state == GuidanceManager::RobotStatus::RUN_AUTO)
        {
            // m_prev_state = cur_state;
            m_pub_flag = false;
            // m_begin_time = ros::Time::now();
        }    
    }
}

Point2 DGRobot::makeSubgoal()
{
    cv::Mat image;
    Point2 robot_dx_pixel, node_dx_pixel;
    Point2 pub_pose = makeImagePixels(image, robot_dx_pixel, node_dx_pixel);

    if (image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x) < 200) //if node is in black area
    {
        //find drivable area on the line from robot_pt
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-current node is black. find drivable area. value:%d",image.at<uchar>((int)node_dx_pixel.y, (int)node_dx_pixel.x));
        pub_pose = findDrivablePoint(image, robot_dx_pixel, node_dx_pixel);
        m_painter.drawPoint(m_map_image, robot_dx_pixel, 10,  cv::Vec3b(200, 100, 230));
    }

    return pub_pose;
}

Point2 DGRobot::reselectDrivablePoint()
{
    cv::Mat image;
    Point2 robot_dx_pixel, node_dx_pixel, dx_metric, dx_pixel;
    Point2 pub_pose = makeImagePixels(image, robot_dx_pixel, node_dx_pixel);

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
    m_painter.drawPoint(m_map_image, robot_dx_pixel, 10,  cv::Vec3b(200, 100, 230));

    return pub_pose;
}

Point2 DGRobot::makeImagePixels(cv::Mat& image, Point2& robot_dx_pixel, Point2& node_dx_pixel)
{
    Point2 pub_pose;
    GuidanceManager::Guidance cur_guide = m_guider.getGuidance();
    ID nid = cur_guide.heading_node_id;
    Node *hNode = m_map.getNode(nid);
    if (hNode == nullptr)
        return pub_pose;
    
    Pose2 node_metric = Pose2(hNode->x, hNode->y);
    LatLon latlon = m_map.toLatLon(node_metric);

    cv::Mat onlinemap, offlinemap;
    Point2 robot_dx_metric;
    // 1. 딥가이더 좌표 --> DX 로봇 좌표
    // dg::LatLon latlon = m_dg_converter.toLatLon(node_metric);
    dg::Point2 dx_metric_rotated = m_dx_converter.toMetric(latlon);
    double dx_x = dx_metric_rotated.x * cos(m_dx_map_rotation_radian) + dx_metric_rotated.y * sin(m_dx_map_rotation_radian);
    double dx_y = -dx_metric_rotated.x * sin(m_dx_map_rotation_radian) + dx_metric_rotated.y * cos(m_dx_map_rotation_radian);

    dg::Point2 node_dx_metric(dx_x, dx_y);
    ROS_INFO_THROTTLE(1.0, "Subgoal for robot: %f,%f", node_dx_metric.x, node_dx_metric.y);

    //Projecting node point to drivable area
    m_robotmap_mutex.lock();
    onlinemap = m_robotmap_image;
    robot_dx_metric = m_guider.m_robot_pose;
    m_guider.m_robot_heading_node_id = nid;
    m_guider.m_robot_heading_node_pose = node_dx_metric;
    m_robotmap_mutex.unlock();

    // 2. DX 로봇 좌표 --> DX 로봇맵 픽셀좌표
    node_dx_pixel.x = m_dx_map_origin_pixel.x + node_dx_metric.x / m_dx_map_meter_per_pixel;
    node_dx_pixel.y = m_dx_map_origin_pixel.y - node_dx_metric.y / m_dx_map_meter_per_pixel;

    robot_dx_pixel.x = m_dx_map_origin_pixel.x + robot_dx_metric.x / m_dx_map_meter_per_pixel;
    robot_dx_pixel.y = m_dx_map_origin_pixel.y - robot_dx_metric.y / m_dx_map_meter_per_pixel;

    ROS_INFO_THROTTLE(1.0, "publishSubGoal-node_on_robotmap: %f,%f", node_dx_pixel.x, node_dx_pixel.y);
    ROS_INFO_THROTTLE(1.0, "publishSubGoal-robot_on_robotmap: %f,%f", robot_dx_pixel.x, robot_dx_pixel.y);

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
        //robot map is expanded
        if (onlinemap.cols > offlinemap.cols || onlinemap.rows > offlinemap.rows)
        {
            ROS_INFO_THROTTLE(1.0, "publishSubGoal-the robot map is expanded");
            // return false;
        }
    }

    if (image.channels() != 1)
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);


    if (node_dx_pixel.x < 1 || node_dx_pixel.x > image.cols || node_dx_pixel.y < 1 || node_dx_pixel.y > image.rows 
    || robot_dx_pixel.x < 1 || robot_dx_pixel.x > image.cols || robot_dx_pixel.y < 1 || robot_dx_pixel.y > image.rows)
    {
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-the pixel exceeds image size");
        // return false;
    }     

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    pub_pose.x = (node_dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    pub_pose.y = (m_dx_map_origin_pixel.y - node_dx_pixel.y) * m_dx_map_meter_per_pixel;

    return pub_pose;   
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

Point2 DGRobot::findDrivableNearPoint(cv::Mat &image, Point2 robot_dx, Point2 node_dx)
{
    Point2 dx_pixel, metric;
    dx_pixel = findDrivableNearPixel(image, robot_dx, node_dx);

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    metric.x = (dx_pixel.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    metric.y = (m_dx_map_origin_pixel.y - dx_pixel.y) * m_dx_map_meter_per_pixel;

    return metric;
}

Point2 DGRobot::findDrivableNearPixel(cv::Mat &image, Point2 robot_px, Point2 node_px)
{
    Point2 px = node_px;
    if (!image.empty())
    {       
        int jump_step = 50;
        int center_x, center_y, x_min, x_max, y_min, y_max;

        cv::Mat cropped_img; 
        double jump_deg = atan2(robot_px.y - node_px.y, robot_px.x - node_px.x);
        int jump_max = (int) norm(robot_px - node_px)/jump_step;

        for (int h = 0; h < jump_max; h++)
        {
            center_x = node_px.x + h * jump_step * cos(jump_deg);
            center_y = node_px.y + h * jump_step * sin(jump_deg);

            x_min = (center_x-jump_step < 1) ? 1 :center_x-jump_step;
            x_max = (center_x+jump_step > image.cols) ? image.cols : center_x+jump_step;
            y_min = (center_y-jump_step < 1) ? 1 : center_y-jump_step;
            y_max = (center_y+jump_step > image.rows) ? image.rows : center_y+jump_step;

            cv::Rect rect(x_min, y_min, y_max-y_min, x_max-x_min);
            cropped_img = image(rect); // cropped image
            if ((cropped_img.cols < 3) || (cropped_img.rows < 3) ){
                ROS_INFO_THROTTLE(1.0,"Cannot provide goal. The goal is on the margin");
                continue;
            }

            double min, max;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(cropped_img, &min, &max, &min_loc, &max_loc);
            
            if (max < 200)
                continue;            

            px.x = max_loc.x + x_min;
            px.y = max_loc.y + y_min;

            //Save image
            cv::Mat img_color;
            cropped_img.copyTo(img_color);
            if (img_color.channels() == 1)
                cv::cvtColor(cropped_img, img_color, cv::COLOR_GRAY2BGR);
            cv::circle(img_color, max_loc, 10, cv::Vec3b(0, 255, 255), 10);
            cv::imwrite("../../../occumap_cropped.png", img_color);   

            return px;
        }
        
        ROS_INFO_THROTTLE(1.0,"Cannot provide goal. All undrivable area!");
        return px;        
    }
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
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-jump_deg: %f", cx::cvtRad2Deg(jump_deg));
        ROS_INFO_THROTTLE(1.0, "publishSubGoal-max_count: %d", max_count);

        //for display map
        erode(image, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);

        int i;
        for (i = 0; i < max_count; i++)
        {
            jump_px.x = node_px.x + i * jump_step * cos(jump_deg);
            jump_px.y = node_px.y + i * jump_step * sin(jump_deg);
            if (img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x) > 200)
            {
                printf("publishSubGoal-img_erode-%d <%.1f, %.1f>: %d\n", i, jump_px.x, jump_px.y, img_erode.at<uchar>((int)jump_px.y, (int)jump_px.x));
                break;
            }
        }
        if (i >= max_count - 1) //no drivble area on the line
        {
            jump_px = findDrivableNearPixel(img_erode, robot_px, node_px);
            ROS_INFO_THROTTLE(1.0, "publishSubGoal-Find drivable point near goal: %f, %f", jump_px.x, jump_px.y);
        }
    }

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

    // 3. DX 로봇맵 픽셀좌표 --> DX 로봇 좌표
    dx_metric.x = (jump_px.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    dx_metric.y = (m_dx_map_origin_pixel.y - jump_px.y) * m_dx_map_meter_per_pixel;

    return dx_metric;
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
  