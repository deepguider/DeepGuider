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
    DGRobot(ros::NodeHandle &nh);
    virtual ~DGRobot();

    bool initialize(std::string config_file);
    int run();
    bool runOnce(double timestamp);

protected:
    virtual int readParam(const cv::FileNode &fn);
    int readRobotParam(const cv::FileNode &fn);

    // Topic subscribers
    ros::Subscriber sub_draw_robot;
    ros::Subscriber sub_dg_status;
    ros::Subscriber sub_robot_status;
    ros::Subscriber sub_robot_pose;
    ros::Subscriber sub_robot_heading;
    ros::Subscriber sub_robot_map;
    void callbackDrawRobot(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callbackRobotStatus(const std_msgs::String::ConstPtr &msg);
    void callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callbackRobotMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    // Topic publishers (sensor data)
    ros::Publisher pub_subgoal;
    void publishSubGoal3();

    double m_min_goal_dist = 3.0;
    Pose2 m_prev_node_dg;      // in DG coordinate
    Pose2 m_cur_node_dg;       // in DG coordinate
    Pose2 m_prev_robot_pose;   // in robot coordinate
    Pose2 m_cur_robot_pose;    // in robot coordinate
    Pose2 m_prev_node_dx;      // in robot coordinate. The shifted node
    Pose2 m_cur_node_dx;       // in robot coordinate. The shifted node
    Pose2 m_next_node_dx;      // in robot coordinate. The shifted node
    Pose2 m_next_next_node_dx; // in robot coordinate. The shifted node
    ID m_prev_node_id = 0;
    ID m_cur_head_node_id = 0;

    // Robot parameters
    bool m_first_robot_pose = true;
    cv::Mat m_robotmap_image;
    std::string m_robotmap_path = "data/Bucheon/bucheon_220922/occumap_bucheon.png";
    dg::LatLon m_dx_map_ref_latlon = dg::LatLon(37.5177542, 126.7651744);
    dg::Point2 m_dx_map_ref_pixel = dg::Point2(715, 650);
    double m_dx_map_meter_per_pixel = 0.1;
    double m_dx_map_rotation_radian = -0.8;
    double m_robotmap_scale = 10.0; // TODO: no need for m_robotmap_scale. Just use 1/m_dx_map_meter_per_pixel
    dg::Point2 m_dx_map_origin_pixel = dg::Point2(345, 1110);
    dg::Point2UTM m_dx_map_origin_utm;
    dg::Point2UTM m_dg_map_origin_utm;
    bool m_pub_flag = false;
    GuidanceManager::RobotStatus m_prev_state = GuidanceManager::RobotStatus::READY;
    ros::Time m_begin_time = ros::Time::now();
    bool m_nopath_flag = false;

    // DX 로봇 부천 지도 origin 계산
    dg::LatLon m_dx_map_origin_latlon;
    dg::LatLon m_dg_map_origin_latlon; // 딥가이더 부천 원점 (dg_simple.yml 참조)
    dg::UTMConverter m_dx_converter;
    dg::UTMConverter m_dg_converter;
    Point2 cvtDg2Dx(const Point2 &dg_metric) const;
    Point2 cvtDx2Dg(const Point2 &dx_metric) const;
    Point2 cvtMetric2Pixel(const Point2 &val) const;
    Point2 cvtPixel2Metric(const Point2 &px) const;

    // robot related functions
    std::deque<Point2> m_undrivable_points;
    int m_undrivable_points_queue_size = 5;
    void initialize_DG_DX_conversion();
    void initializeSensorSpecificParameters();
    bool makeSubgoal1(Pose2 &pub_pose);
    bool makeSubgoal12(Pose2 &pub_pose);
    bool makeSubgoal13(Pose2 &pub_pose);
    bool isSubPathDrivable(cv::Mat robotmap, Pose2 pointA, Pose2 pointB);
    bool isSubPathDrivablev2(cv::Mat robotmap, Pose2 dest_point);
    bool isSubPathDrivablev3(cv::Mat robotmap, Pose2 dest_point, Pose2 source_point);
    Point2 getFarthestPoint(cv::Mat robotmap, Point2 dest_point, Point2 source_point);
    Point2 getFarthestPointSafe(cv::Mat robotmap, Point2 dest_point, Point2 source_point, double min_space_to_obstacle);
    bool findAlternativePath(Pose2 &alternate_point, Pose2 robot_pose, Pose2 target_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_targetnode, int num_alternatives, cv::Mat &colormap, double &min_dist_to_next_node);
    bool findAlternativePathv2(Pose2 &alternate_point, Pose2 robot_pose, Pose2 target_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_targetnode, int num_alternatives, cv::Mat &colormap, double angle_range, double &min_dist_to_next_node);
    void rotatePose(Pose2 &P, double theta_rot);
    Pose2 cvtDGtoDXcoordinate(Pose2 P, Pose2 P_DG, Pose2 P_DX);
    Pose2 cvtMaptoRobotcoordinate(Pose2 P);
    Pose2 cvtRobottoMapcoordinate(Pose2 P);
    int findRobotCoordAngleofVectorSource2Dest(Pose2 source, Pose2 dest);
    void record2Video(cv::Mat colormap, cv::Mat clean_colormap, Pose2 robot_pose);
    void record2VideoCropped(cv::Mat crop);
    Pose2 m_robot_origin;
    Pose2 m_errorvec_dgnode_curpose;
    int m_drivable_threshold = 220;
    bool m_nodeupdated_but_problematic = false;
    bool m_align_dg_nodes = false;

    geometry_msgs::PoseStamped makeRosPubPoseMsg(ID nid, Pose2 pose);
    bool findExtendedDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2 &result_px);
    bool findDrivableinLine(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2 &result_px);
    bool drawSubgoal(Point2 &pub_pose);

    bool m_save_guidance_video = false;
    bool m_save_guidance_image = false;
    bool m_auto_jump_tooclose_target = true;
    double m_min_subgoalspace_to_obstacle = 1.2;
    bool m_show_dg_pose = false;
    bool m_test_continuous_subgoal = false;
    int m_crop_radius = 400;
    cv::Mat m_offline_robotmap;
    cv::Size m_framesize = cv::Size(8000, 8000);
    cv::Size m_framesize_crop = cv::Size(802, 802);
    cv::Mat m_frame_crop;
    int m_fourcc = cv::VideoWriter::fourcc('A', 'V', 'C', '1');
    bool m_record_robotmap = false;
    cv::VideoWriter m_video_robotmap;
    bool m_record_robotmap_crop = false;
    cv::VideoWriter m_video_robotmap_crop;
    cv::VideoWriter m_video_guidance_crop;
};

DGRobot::DGRobot(ros::NodeHandle &nh) : DeepGuiderROS(nh)
{
}

DGRobot::~DGRobot()
{
}

int DGRobot::readParam(const cv::FileNode &fn)
{
    int n_read = DeepGuiderROS::readParam(fn);
    n_read += readRobotParam(fn);
    return n_read;
}

int DGRobot::readRobotParam(const cv::FileNode &fn)
{
    int n_read = 0;

    // read robot configuration
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
    m_dg_map_origin_latlon = m_map_ref_point;

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
    printf("m_dx_map_rotation_radian: %f\n", cx::cvtRad2Deg(m_dx_map_rotation_radian));

    return n_read;
}

bool DGRobot::initialize(std::string config_file)
{
    printf("Initialize dg_robot..\n");

    // Initialize DeeGuiderRos system
    bool ok = DeepGuiderROS::initialize(config_file);
    if (!ok)
        return false;

    // Initialize subscribers
    sub_robot_pose = nh_dg.subscribe("/mcl3d/current/pose", 1, &DGRobot::callbackRobotPose, this);
    sub_draw_robot = nh_dg.subscribe("/mcl3d/current/pose", 1, &DGRobot::callbackDrawRobot, this);
    // sub_robot_status = nh_dg.subscribe("/keti_robot_state", 1, &DGRobot::callbackRobotStatus, this); //run_manual
    sub_robot_status = nh_dg.subscribe("/keti_robot/state", 1, &DGRobot::callbackRobotStatus, this); // run_auto
    sub_robot_map = nh_dg.subscribe("/deepmerge/map/occu", 1, &DGRobot::callbackRobotMap, this);

    // Initialize deepguider publishers
    pub_subgoal = nh_dg.advertise<geometry_msgs::PoseStamped>("dg_subgoal", 1, true);

    // Initialize robot parameters
    initialize_DG_DX_conversion();

    // Initialize Sensor-specific parameters for DeepGuider Recoginition Modules
    initializeSensorSpecificParameters();

    if (m_save_guidance_video)
    {
        if (m_record_robotmap) m_video_robotmap.open("../../../online_robotmap.avi", m_fourcc, m_video_recording_fps, m_framesize);
        if (m_record_robotmap_crop) m_video_robotmap_crop.open("../../../online_crop.avi", m_fourcc, m_video_recording_fps, m_framesize_crop);
        m_video_guidance_crop.open("../../../guidance_crop.avi", m_fourcc, m_video_recording_fps, m_framesize_crop);
        m_frame_crop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);
    }

    // load offline robotmap
    m_offline_robotmap = cv::imread(m_robotmap_path, cv::IMREAD_GRAYSCALE);

    return true;
}

void DGRobot::initializeSensorSpecificParameters()
{
    // road theta
    RoadThetaParam param = m_roadtheta.param();
    param.camera_vanishing_y = 0.612; // ratio w.r.t. image height
    param.vy_range[0] = 0.5;
    param.vy_range[1] = 0.8;
    param.apply_validation = true;
    param.valid_vy_range[0] = 0.55;
    param.valid_vy_range[1] = 0.67;
    param.valid_peak_score = 600;
    m_roadtheta.set_param(param);

    // OCR
    m_ocr.setParam(82.1, 0.83, 0.612);
}

int DGRobot::run()
{
    printf("Run Deepguider with Robot...\n");

    // start internal recognizer threads
    if (m_enable_intersection == 1)
        intersection_thread = new std::thread(threadfunc_intersection, this);
    if (m_enable_ocr == 1)
        ocr_thread = new std::thread(threadfunc_ocr, this);
    if (m_enable_vps == 1)
        vps_thread = new std::thread(threadfunc_vps, this);
    if (m_enable_roadlr == 1)
        roadlr_thread = new std::thread(threadfunc_roadlr, this);
    if (m_enable_roadtheta == 1)
        roadtheta_thread = new std::thread(threadfunc_roadtheta, this);
    if (m_enable_exploration == 1)
        exploration_thread = new std::thread(threadfunc_exploration, this);
    if (m_enable_logo == 1)
        logo_thread = new std::thread(threadfunc_logo, this);

    // run main loop
    ros::Rate loop(m_update_hz);
    while (ros::ok())
    {
        ros::Time timestamp = ros::Time::now();
        if (!runOnce(timestamp.toSec()))
            break;
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
    if(m_video_recording) printf("\tgui recording closed\n");
    if(m_save_guidance_video)
    {
        if (m_record_robotmap) m_video_robotmap.release();
        if (m_record_robotmap_crop) m_video_robotmap_crop.release();
        m_video_guidance_crop.release();
        printf("\tguidance recording closed\n");
    }
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
    if (!ok)
        return false;

    publishSubGoal3();

    return true;
}

void DGRobot::callbackRobotPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
    // ROS_INFO("Robot: %f,%f, deg:%f", x, y, cx::cvtRad2Deg(theta));

    // Reset deepguider pose by first arrived robot pose
    if (m_first_robot_pose)
    {
        // m_localizer->setPose(cvtDx2Dg(dg::Pose2(x, y, theta)), timestamp);
        // m_localizer->setPose(dg::Pose2(x, y, theta), timestamp);
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

void DGRobot::callbackDrawRobot(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    dg::Point2 robot_dg = cvtDx2Dg(cv::Point2d(x, y));
    dg::Pose2 robot_px = m_painter.cvtValue2Pixel(robot_dg);
    cv::circle(m_map_image, robot_px, 1, cv::Vec3b(0, 150, 255));
}

void DGRobot::callbackRobotStatus(const std_msgs::String::ConstPtr &msg)
{
    const char *str = msg->data.c_str();
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

void DGRobot::callbackRobotMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    int size_x = map->info.width;
    int size_y = map->info.height;
    m_robot_origin = dg::Point2(map->info.origin.position.x, map->info.origin.position.y);

    // ROS_INFO("callbackRobotMap: Robot map size x: %d, y: %d", size_x, size_y);

    if ((size_x < 3) || (size_y < 3))
    {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    Point2 new_origin_pixel;
    // // convert new origin from robot to map coordinate
    // new_origin_pixel.x = -map->info.origin.position.x / m_dx_map_meter_per_pixel;
    // new_origin_pixel.y = -map->info.origin.position.y / m_dx_map_meter_per_pixel;
    // alternatively
    Point2 origin = dg::Point2(0, 0);
    new_origin_pixel = cvtRobottoMapcoordinate(origin);

    cv::Mat image(size_y, size_x, CV_8UC1);
    for (int i = 0; i < size_y; i++)
    {
        for (int j = 0; j < size_x; j++)
        {
            int index = i * size_x + j;
            if (map->data[index] <= 30)
            {
                image.at<uchar>(i, j) = 250 - map->data[index];
            }
            else
            {
                image.at<uchar>(i, j) = 0;
            }
        }
    }

    m_robotmap_mutex.lock();
    m_dx_map_origin_pixel = new_origin_pixel;
    m_robotmap_image = image;
    m_robotmap_mutex.unlock();

    // save image
    m_guider_mutex.lock();
    Pose2 robot_dx_metric = m_guider.m_robot_pose;
    std::vector<GuidanceManager::ExtendedPathElement> ext_path = m_guider.m_extendedPath;
    m_guider_mutex.unlock();
    Point2 robot_px = cvtMetric2Pixel(robot_dx_metric);

    cv::Mat colormap;
    double view_d = 200;
    cv::Rect roi_rc(robot_px.x - view_d, robot_px.y - view_d, 2 * view_d + 1, 2 * view_d + 1);
    roi_rc = roi_rc & cv::Rect(0, 0, image.cols, image.rows);
    cv::cvtColor(image(roi_rc), colormap, cv::COLOR_GRAY2BGR);

    Point2 offset = roi_rc.tl();
    cv::circle(colormap, robot_px - offset, 10, cv::Vec3b(0, 255, 0), 2);
    Point2 heading;
    heading.x = robot_px.x - offset.x + 20 * cos(robot_dx_metric.theta);
    heading.y = robot_px.y - offset.y + 20 * sin(robot_dx_metric.theta);
    cv::line(colormap, robot_px - offset, heading, cv::Vec3b(0, 255, 0), 2);

    Pose2 cur_node_dg;
    Point2 node_dx;
    for (size_t i = 0; i < ext_path.size(); i++)
    {
        cur_node_dg = Point2(ext_path[i]);
        node_dx = cvtMetric2Pixel(cvtDg2Dx(cur_node_dg));
        cv::circle(colormap, node_dx - offset, 10, cv::Vec3b(0, 0, 255), 2);
    }

    cv::flip(colormap, colormap, 0);
    imshow("robotmap", colormap);
    cv::waitKey(1);

    if (m_save_guidance_image) imwrite("../../../callbackRobotMap.png", colormap);
}

void DGRobot::publishSubGoal3()
{
    if (!m_guider.isGuidanceInitialized())
    {
        return;
    }

    Pose2 pub_pose;
    if (m_test_continuous_subgoal)
    {
        makeSubgoal13(pub_pose);
        return;
    }

    GuidanceManager::GuideStatus cur_guidance_status = m_guider.getGuidanceStatus();
    // ROS_INFO("[publishSubGoal3] guidance status %d", (int) cur_guidance_status);
    // ROS_INFO("[publishSubGoal3] arrived guidance status %d", (int) GuidanceManager::GuideStatus::GUIDE_ARRIVED);
    // if(cur_guidance_status != GuidanceManager::GuideStatus::GUIDE_ARRIVED){  // guidance not yet arrived. (commented out because seems when guidance status is indeed arrived, this publishsubgoal function is never been run)
    GuidanceManager::RobotStatus cur_state = m_guider.getRobotStatus();

    if (cur_state != GuidanceManager::RobotStatus::NO_PATH)
    {
        m_nopath_flag = false;
    }
    if (cur_state == GuidanceManager::RobotStatus::ARRIVED_NODE || cur_state == GuidanceManager::RobotStatus::ARRIVED_GOAL || cur_state == GuidanceManager::RobotStatus::READY || cur_state == GuidanceManager::RobotStatus::NO_PATH)
    {

        std::string str;
        switch (cur_state)
        {
            case GuidanceManager::RobotStatus::ARRIVED_NODE:
                str = "ARRIVED_NODE";
                break;
            case GuidanceManager::RobotStatus::ARRIVED_GOAL:
                str = "ARRIVED_GOAL";
                break;
            case GuidanceManager::RobotStatus::READY:
                str = "READY";
                break;
            case GuidanceManager::RobotStatus::NO_PATH:
                str = "NO_PATH";
                break;
        }
        printf("\n cur_state: %s\n\n", str.c_str());

        //if there is an obstacle in the path, robot sends no_path
        if (cur_state == GuidanceManager::RobotStatus::NO_PATH)
        {
            ROS_INFO("m_nopath_flag: %d", m_nopath_flag);   
            if (!m_nopath_flag)
            {
                m_nopath_flag = true;
                m_begin_time = ros::Time::now();
            }
                     
        }

        // ROS_INFO("cur_state: %d", (int)cur_state);
        ros::Time cur_time = ros::Time::now();
        ros::Duration duration = cur_time - m_begin_time;
        if (duration > ros::Duration(5.0))
        {
            ROS_INFO("Duration seconds: %d", duration.sec);

            // (GOOGLE DOCS - Subgoal Coordinate Calculation - NO_PATH signal handling - STEP 1)
            if (cur_state == GuidanceManager::RobotStatus::NO_PATH)
            { // if no path, add undrivable pose
                Point2 undrivable_pose = m_guider.m_subgoal_pose;
                m_undrivable_points.push_back(undrivable_pose);

                // dequeue if length > m_undrivable_points_queue_size
                if (m_undrivable_points.size() > m_undrivable_points_queue_size)
                {
                    m_undrivable_points.pop_front();
                }
            }
            else
            {                                // if no more NO PATH (means, the previous goal is successful/not NO PATH) (GOOGLE DOCS - Subgoal Coordinate Calculation - NO_PATH signal handling - STEP 2)
                m_undrivable_points.clear(); // empty the queue
            }

            //if (makeSubgoal1(pub_pose))  // Seohyun's
            //if (makeSubgoal12(pub_pose)) // Marcella's
            if (makeSubgoal13(pub_pose)) // revision of Marcella's
            {
                geometry_msgs::PoseStamped rosps = makeRosPubPoseMsg(m_cur_head_node_id, pub_pose);
                pub_subgoal.publish(rosps);
                m_guider.m_subgoal_pose = pub_pose;
                ROS_INFO("==============================================================\n");
                ROS_INFO("SubGoal published!: %f, %f<=====================", pub_pose.x, pub_pose.y);
                ROS_INFO("==============================================================\n");
                m_begin_time = ros::Time::now();
            }
        }
    }
    // }
    // else{
    //     ROS_INFO("[publishSubGoal3] ARRIVED. Don't calculate and publish anymore subgoal");
    // }
}

bool DGRobot::makeSubgoal1(Pose2 &pub_pose)
{
    ROS_INFO("[makeSubgoal1] %d", m_guider.getCurGuideIdx());

    GuidanceManager::ExtendedPathElement cur_guide = m_guider.getCurExtendedPath();
    GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();
    GuidanceManager::ExtendedPathElement next_next_guide = m_guider.getNextNextExtendedPath();

    Pose2 cur_node_dg = Point2(cur_guide);
    Pose2 next_node_dg = Point2(next_guide);
    Pose2 next_next_node_dg = Point2(next_next_guide);
    ROS_INFO("[makeSubgoal] Next Heading %zd, node_metric: <%f, %f>", next_guide.cur_node_id, next_node_dg.x, next_node_dg.y);
    ROS_INFO("[makeSubgoal] Next Next Heading %zd, node_metric: <%f, :%f>", next_next_guide.cur_node_id, next_next_node_dg.x, next_next_node_dg.y);

    // load robot info
    m_robotmap_mutex.lock();
    cv::Mat robotmap = m_robotmap_image;
    Pose2 robot_dx_metric = m_guider.m_robot_pose;
    m_robotmap_mutex.unlock();

    if (robotmap.empty()) // on robot occumap
    {
        ROS_INFO("No occumap");
        robotmap = cv::imread(m_robotmap_path, cv::IMREAD_GRAYSCALE); // if no online map, read offline map
    }

    if (robotmap.empty())
        return false;

    // convert DG metric to DX metric
    Pose2 dg_pose = m_localizer->getPose();
    Pose2 dg_dx_pose = cvtDg2Dx(dg_pose);
    Pose2 cur_node_dx = cvtDg2Dx(cur_node_dg);
    Pose2 next_node_dx = cvtDg2Dx(next_node_dg);

    pub_pose = next_node_dx;
    // if two nodes are the same, it is last node
    if (cur_guide.cur_node_id == 0)
    {
        ROS_INFO("[makeSubgoal] Last node. idx:[%d] - Heading %zd", m_guider.getCurGuideIdx(), next_guide.cur_node_id);
        ROS_INFO("[makeSubgoal] pub_pose: <%f, %f>", pub_pose.x, pub_pose.y);
        return true;
    }

    ROS_INFO("[makeSubgoal] %d", m_guider.getCurGuideIdx());

    // calculate next node
    Pose2 new_node_dx;
    double diff_dist, base_theta;
    if (m_prev_node_id == 0)
    {
        new_node_dx = next_node_dx;
    }
    else if (m_prev_node_id != 0 && (m_cur_head_node_id != next_guide.cur_node_id)) // if not initial start
    {
        ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++");
        ROS_INFO("[makeSubgoal] prev_id: %zd, cur_id: %zd, next_id: %zd", m_prev_node_id, m_cur_head_node_id, next_guide.cur_node_id);
        int diff_deg = m_guider.getDegree(m_prev_node_dg, m_cur_node_dg, next_node_dg);
        diff_dist = norm(next_node_dg - m_cur_node_dg);
        // give calculated r, theta point based on robot pose, and find near subgoal
        //  ROS_INFO("[makeSubgoal] robot_dx_metric: <%f, %f>, m_prev_robot_metric <%f, %f>", robot_dx_metric.x, robot_dx_metric.y, m_prev_robot_pose.x, m_prev_robot_pose.y);
        Pose2 robot_diff = robot_dx_metric - m_prev_robot_pose;
        double base_theta = atan2(robot_diff.y, robot_diff.x);
        // ROS_INFO("[makeSubgoal] robot.theta: %f, +theta:: %d", cx::cvtRad2Deg(base_theta2), diff_deg);
        new_node_dx.theta = base_theta + cx::cvtDeg2Rad(diff_deg);
        new_node_dx.x = robot_dx_metric.x + diff_dist * cos(new_node_dx.theta);
        new_node_dx.y = robot_dx_metric.y + diff_dist * sin(new_node_dx.theta);
    }
    else
    {
        Pose2 node_diff = next_node_dg - dg_pose;
        diff_dist = norm(node_diff);
        base_theta = atan2(node_diff.y, node_diff.x);
        ROS_INFO("[makeSubgoal] norm: %f, base_theta: %f", diff_dist, cx::cvtRad2Deg(base_theta));
        new_node_dx.theta = base_theta;
        new_node_dx.x = robot_dx_metric.x + diff_dist * cos(new_node_dx.theta);
        new_node_dx.y = robot_dx_metric.y + diff_dist * sin(new_node_dx.theta);
    }

    pub_pose = new_node_dx;
    ROS_INFO("[makeSubgoal] pub_pose: <%f, %f>", pub_pose.x, pub_pose.y);

    Point2 dg_px = cvtMetric2Pixel(dg_dx_pose);
    Point2 robot_px = cvtMetric2Pixel(robot_dx_metric);
    Point2 cur_node_px = cvtMetric2Pixel(cur_node_dx);
    Point2 next_node_px = cvtMetric2Pixel(next_node_dx);
    Point2 prev_robot_px = cvtMetric2Pixel(m_prev_robot_pose);
    Point2 prev_node_px = cvtDg2Dx(cvtMetric2Pixel(m_prev_node_dg));

    // based on the undrivable points, make the robotmap undrivable on those points
    for (int i = 0; i < m_undrivable_points.size(); i++)
    {
        Point2 node_robot = m_undrivable_points[i];
        Point2 node_pixel = cvtMetric2Pixel(node_robot);
        robotmap.at<uchar>((int)node_pixel.y, (int)node_pixel.x) = 0;
    }

    // check whether pub_pose is undrivable find drivable point
    Point2 check_node_px = cvtMetric2Pixel(pub_pose);
    ROS_INFO("check_node_px: <%f, %f>", check_node_px.x, check_node_px.y);
    cv::Mat img_erode;
    Point2 node_px_moved;
    erode(robotmap, img_erode, cv::Mat::ones(cv::Size(m_robotmap_scale, m_robotmap_scale), CV_8UC1), cv::Point(-1, -1), 1);
    int value = img_erode.at<uchar>((int)check_node_px.y, (int)check_node_px.x);
    if (value < m_drivable_threshold) // if node is in black area
    {
        ROS_INFO("Current node is in dark area. <%f, %f> value:%d", check_node_px.x, check_node_px.y, value);
        if (!findDrivableinLine(img_erode, robot_px, check_node_px, node_px_moved))
        {
            if (!findExtendedDrivablePoint(img_erode, robot_px, check_node_px, node_px_moved))
            {
                ROS_INFO("Tried all. Failed to provide drivable goal.");
                return false;
            }
        }
        pub_pose = cvtPixel2Metric(node_px_moved);
    }

    if (norm(pub_pose - Point2(0, 0)) < 1)
    {
        ROS_INFO("Not an appropriate goal. <%f, %f> Return false.", pub_pose.x, pub_pose.y);
        return false;
    }

    ROS_INFO("Found subgoal: <%f, %f>", pub_pose.x, pub_pose.y);
    // change previous value
    if (m_cur_head_node_id != next_guide.cur_node_id)
    {
        m_prev_node_dg = m_cur_node_dg;
        m_cur_node_dg = next_node_dg;
        m_prev_node_id = m_cur_head_node_id;
        m_cur_head_node_id = next_guide.cur_node_id;
        m_prev_robot_pose = robot_dx_metric;
        m_guider.m_robot_heading_node_pose = pub_pose;
    }

    /// save image
    cv::Mat colormap;
    img_erode.copyTo(colormap);
    if (colormap.channels() == 1)
        cv::cvtColor(colormap, colormap, cv::COLOR_GRAY2BGR);
    ROS_INFO("save image<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    cv::circle(colormap, robot_px, 10, cv::Vec3b(0, 255, 0), 2); // robot pose: green circle
    Point2 heading;
    heading.x = robot_px.x + 20 * cos(robot_dx_metric.theta);
    heading.y = robot_px.y + 20 * sin(robot_dx_metric.theta);
    cv::line(colormap, robot_px, heading, cv::Vec3b(0, 255, 0), 2); // green

    if (norm(prev_robot_px - m_dx_map_origin_pixel) > 1)
    {
        ROS_INFO("prev_robot_px: <%f, %f>", prev_robot_px.x, prev_robot_px.y);
        cv::circle(colormap, prev_robot_px, 1, cv::Vec3b(255, 255, 0), 2);      // prev robot: cyan circle
        cv::line(colormap, prev_robot_px, robot_px, cv::Vec3b(255, 255, 0), 2); // cyan line
    }
    cv::drawMarker(colormap, dg_px, cv::Vec3b(255, 0, 0), cv::MARKER_STAR); // dg_pose: blue star
    cv::line(colormap, dg_px, next_node_px, cv::Vec3b(255, 0, 0), 2);       //

    cv::drawMarker(colormap, next_node_px, cv::Vec3b(0, 100, 255), cv::MARKER_DIAMOND, 20, 2); // cur node: orange diamond
    cv::line(colormap, robot_px, next_node_px, cv::Vec3b(0, 100, 255), 2);

    cv::drawMarker(colormap, cur_node_px, cv::Vec3b(0, 255, 255), cv::MARKER_DIAMOND, 20, 2); // next node: yellow diamond
    cv::line(colormap, cur_node_px, dg_px, cv::Vec3b(0, 255, 255), 2);

    Point2 new_node_px = cvtMetric2Pixel(new_node_dx);
    cv::drawMarker(colormap, new_node_px, cv::Vec3b(0, 0, 255), cv::MARKER_DIAMOND, 20, 2); // new node: red diamond
    cv::line(colormap, robot_px, new_node_px, cv::Vec3b(0, 0, 255), 2);

    if (norm(node_px_moved - Point2(0, 0)) > 1)
    {
        cv::drawMarker(colormap, node_px_moved, cv::Vec3b(255, 0, 255), cv::MARKER_DIAMOND, 20, 2); // new moved node: magenta
    }

    imwrite("../../../test_makeSubgoal.png", colormap);

    // temporary
    cv::Mat nodemap;
    colormap.copyTo(nodemap);
    std::vector<GuidanceManager::ExtendedPathElement> ext_path = m_guider.m_extendedPath;
    Pose2 path_node;
    Point2 draw_node;
    for (size_t i = 0; i < ext_path.size(); i++)
    {
        path_node = Point2(ext_path[i]);
        draw_node = cvtMetric2Pixel(cvtDg2Dx(path_node));
        cv::circle(nodemap, draw_node, 10, cv::Vec3b(0, 0, 255), 2);
    }

    cv::drawMarker(nodemap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), cv::MARKER_CROSS, 50, 5); // m_dx_map_origin_pixel: magenta
    cv::drawMarker(nodemap, m_dx_map_ref_pixel, cv::Vec3b(0, 0, 255), cv::MARKER_CROSS, 50, 5);      // m_dx_map_ref_pixel: green cross

    imwrite("../../../callbackRobotMap.png", nodemap);

    // record video
    cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);
    cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
    colormap.copyTo(roi);
    m_video_robotmap << videoFrame;

    cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);
    int x = robot_px.x - m_framesize_crop.width / 2;
    int y = robot_px.y - m_framesize_crop.height / 2;
    if (x + m_framesize_crop.width >= colormap.cols)
        x = colormap.cols - m_framesize_crop.width - 1;
    if (x <= 1)
        x = 1;
    if (y + m_framesize_crop.height >= colormap.rows)
        y = colormap.rows - m_framesize_crop.height - 1;
    if (y <= 1)
        y = 1;
    cv::Mat roicrop(colormap, cv::Rect(x, y, videoFrameCrop.cols, videoFrameCrop.rows));
    roicrop.copyTo(videoFrameCrop);
    m_video_guidance_crop << videoFrameCrop;

    return true;
}

int DGRobot::findRobotCoordAngleofVectorSource2Dest(Pose2 source, Pose2 dest)
{
    Pose2 source_dummy;
    source_dummy.x = source.x - 10.0;
    source_dummy.y = source.y;
    return m_guider.getDegree(source_dummy, source, dest); // return in degree
}

bool DGRobot::findAlternativePath(Pose2 &alternative_point, Pose2 robot_pose, Pose2 next_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_targetnode, int num_alternatives, cv::Mat &colormap, double &min_dist_to_next_node)
{
    /*
    Alternative candidates sampled uniformly on a circle. Angle of each sample is based on robot origin.
    */

    // find alternative that is drivable and the closest to the goal
    min_dist_to_next_node = 1000000; // 100 km.
    Pose2 valid_point_with_min_dist;
    double new_theta = 0;

    // iterate over all alternatives
    for (int i = 0; i < num_alternatives; i++)
    {
        Pose2 alternative_pub_pose;

        // a new alternative
        alternative_pub_pose.x = robot_pose.x + sub_goal_distance * cos(cx::cvtDeg2Rad(new_theta));
        alternative_pub_pose.y = robot_pose.y + sub_goal_distance * sin(cx::cvtDeg2Rad(new_theta));
        // ROS_INFO("alternative_pub_pose x %f, y %f", alternative_pub_pose.x, alternative_pub_pose.y);
        cv::drawMarker(colormap, cvtRobottoMapcoordinate(alternative_pub_pose), cv::Vec3b(255, 255, 0), 1, 10, 2); // cyan small cross

        // distance from alternative_pub_pose to next_node_robot
        double dist_to_next_node = norm(next_node_robot - alternative_pub_pose);

        // if alternative is drivable and distance to next_node is less than the min distance
        // if (isSubPathDrivable(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node){
        // if (isSubPathDrivablev2(robotmap_erode, alternative_pub_pose) && dist_to_next_node < min_dist_to_next_node){
        if (isSubPathDrivablev3(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node)
        {
            min_dist_to_next_node = dist_to_next_node;
            valid_point_with_min_dist = alternative_pub_pose;
            // ROS_INFO("Success finding better sub goal");
        }

        // go to the next alternative
        new_theta += (360 / num_alternatives);
    }
    if (min_dist_to_next_node == 1000000)
    {
        return false; // can't find alternative :(
    }
    else
    { // can find alternative :)
        alternative_point.x = valid_point_with_min_dist.x;
        alternative_point.y = valid_point_with_min_dist.y;
        return true;
    }
}

bool DGRobot::findAlternativePathv2(Pose2 &alternative_point, Pose2 robot_pose, Pose2 next_node_robot, cv::Mat robotmap_erode, double sub_goal_distance, double dist_robot_to_targetnode, int num_alternatives, cv::Mat &colormap, double angle_range, double &min_dist_to_next_node)
{
    /*
    Alternative candidates sampled from -angle_range to angle_range. Angle_range 180 is similar (but not same) to findAlternativePath (difference: angle of sample)
    */

    // Find destination angle based on robot coordinate
    int next_node_theta = findRobotCoordAngleofVectorSource2Dest(robot_pose, next_node_robot);

    // find alternative that is drivable and the closest to the goal
    min_dist_to_next_node = 1000000; // 100 km.
    Pose2 valid_point_with_min_dist;
    double new_theta = -angle_range + (double)next_node_theta;

    double final_excluded_theta;
    if (angle_range == 180)
    { // if circle, angle_range*2: don't include the final 360 degree
        final_excluded_theta = angle_range * 2;
    }
    else
    { // if not circle, include the final angle_range*2, so (angle_range*2 + (angle_range*2/num_alternatives))
        final_excluded_theta = angle_range * 2 + (angle_range * 2 / num_alternatives);
    }

    // bool isdistlessthanmindist=false;
    // bool isanypathdrivable=false;
    // iterate over all alternatives
    for (int i = 0; i < num_alternatives; i++)
    {
        Pose2 alternative_pub_pose;

        // a new alternative
        alternative_pub_pose.x = robot_pose.x + sub_goal_distance * cos(cx::cvtDeg2Rad(new_theta));
        alternative_pub_pose.y = robot_pose.y + sub_goal_distance * sin(cx::cvtDeg2Rad(new_theta));
        // ROS_INFO("alternative_pub_pose x %f, y %f", alternative_pub_pose.x, alternative_pub_pose.y);
        cv::drawMarker(colormap, cvtRobottoMapcoordinate(alternative_pub_pose), cv::Vec3b(255, 255, 0), 1, 4, 1); // cyan small cross

        // distance from alternative_pub_pose to next_node_robot
        double dist_to_next_node = norm(next_node_robot - alternative_pub_pose);

        // if alternative is drivable and distance to next_node is less than the min distance
        // if (isSubPathDrivable(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node){
        // if (isSubPathDrivablev2(robotmap_erode, alternative_pub_pose) && dist_to_next_node < min_dist_to_next_node){
        if (isSubPathDrivablev3(robotmap_erode, alternative_pub_pose, robot_pose) && dist_to_next_node < min_dist_to_next_node)
        {
            min_dist_to_next_node = dist_to_next_node;
            valid_point_with_min_dist = alternative_pub_pose;
            //ROS_INFO("Success finding better sub goal");
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
        new_theta += (final_excluded_theta / num_alternatives);
    }
    if (min_dist_to_next_node == 1000000)
    {
        // if (!isanypathdrivable) ROS_INFO("Can't find alternative because of no path drivable");
        // if (!min_dist_to_next_node) ROS_INFO("Can't find alternative because of min dist is still same.. HAVE TO CHANGE THE INITIAL VALUE");
        return false; // can't find alternative :(
    }
    else
    { // can find alternative :)
        alternative_point.x = valid_point_with_min_dist.x;
        alternative_point.y = valid_point_with_min_dist.y;
        return true;
    }
}

void DGRobot::rotatePose(Pose2 &P, double theta_rot)
{
    Pose2 P_ = P;
    P.x = P_.x * cos(theta_rot) - P_.y * sin(theta_rot);
    P.y = P_.x * sin(theta_rot) + P_.y * cos(theta_rot);
    P.theta = P_.theta + P.theta; // TODO: don't know if this is correct
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

bool DGRobot::isSubPathDrivable(cv::Mat robotmap, Pose2 pointA, Pose2 pointB)
{
    // if ANY point between pointA and pointB is below drivable threshold, then not drivable

    // pointA and pointB in robot coordinate
    Pose2 pointA_px = cvtRobottoMapcoordinate(pointA);
    Pose2 pointB_px = cvtRobottoMapcoordinate(pointB);
    cv::LineIterator line_it(robotmap, cv::Point(pointA_px.x, pointA_px.y), cv::Point(pointB_px.x, pointB_px.y), 8);
    // ROS_INFO("from pointA <%f, %f> to pointB <%f, %f> Num of line iterator %d", pointA_px.x, pointA_px.y, pointB_px.x, pointB_px.y, line_it.count);
    for (int i = 0; i < line_it.count; i++, ++line_it)
    {
        cv::Point point_px = line_it.pos();
        int value = robotmap.at<uchar>(point_px.y, point_px.x);
        if (value < m_drivable_threshold) // any non drivable area
        {
            // ROS_INFO("non drivable value <%d>", value);
            return false;
        }
    }
    return true;
}

bool DGRobot::isSubPathDrivablev2(cv::Mat robotmap, Pose2 dest_point)
{
    // as long as the dest_point is in drivable area, then drivable

    Pose2 point_px = cvtRobottoMapcoordinate(dest_point);

    int value = robotmap.at<uchar>(point_px.y, point_px.x);
    if (value < m_drivable_threshold)
    {
        return false;
    }
    return true;
}

bool DGRobot::isSubPathDrivablev3(cv::Mat robotmap, Pose2 dest_point, Pose2 source_point)
{
    // if current position is drivable, isSubPathDrivable
    // else, ignore the black continuous to the source_point first, then after getting to drivable area (new_source_point), isSubPathDrivable

    // pointA and pointB in robot coordinate
    Pose2 dest_point_px = cvtRobottoMapcoordinate(dest_point);
    Pose2 source_point_px = cvtRobottoMapcoordinate(source_point);

    if (robotmap.at<uchar>(source_point_px.y, source_point_px.x) < m_drivable_threshold)
    { // current position is NOT drivable
        cv::LineIterator line_it(robotmap, cv::Point(dest_point_px.x, dest_point_px.y), cv::Point(source_point_px.x, source_point_px.y), 8);

        for (int i = 0; i < line_it.count; i++, ++line_it)
        {
            cv::Point point_px = line_it.pos();
            int value = robotmap.at<uchar>(point_px.y, point_px.x);
            if (value >= m_drivable_threshold) // first encounter of drivable area
            {
                Pose2 point_px_pose;
                point_px_pose.x = point_px.x;
                point_px_pose.y = point_px.y;
                Pose2 new_source_point = cvtMaptoRobotcoordinate(point_px_pose);
                return isSubPathDrivable(robotmap, dest_point, new_source_point);
            }
        }

        ROS_INFO("No encounter of the first drivable area");
        return false; // all non-drivable until the destination
    }
    else
    { // current position is drivable
        return isSubPathDrivable(robotmap, dest_point, source_point);
    }
}

Point2 DGRobot::getFarthestPoint(cv::Mat robotmap, Point2 dest_point, Point2 source_point)
{
    // return false if no drivable area (including source itself)
    // return the_farthest_point

    // pointA and pointB in robot coordinate
    Point2 dest_point_px = cvtRobottoMapcoordinate(dest_point);
    Point2 source_point_px = cvtRobottoMapcoordinate(source_point);
    Point2 the_farthest_point;
    Point2 the_farthest_point_px;

    if (robotmap.at<uchar>(source_point_px.y, source_point_px.x) < m_drivable_threshold)
    { // current position is NOT drivable
        cv::LineIterator line_it(robotmap, cv::Point(source_point_px.x, source_point_px.y), cv::Point(dest_point_px.x, dest_point_px.y), 8);
        bool is_find_drivable = false;
        for (int i = 0; i < line_it.count; i++, ++line_it)
        {
            cv::Point point_px = line_it.pos();
            int value = robotmap.at<uchar>(point_px.y, point_px.x);

            if (i == 0)
            {
                the_farthest_point_px.x = point_px.x;
                the_farthest_point_px.y = point_px.y;
            }

            if (value >= m_drivable_threshold) // first encounter of drivable area
            {
                is_find_drivable = true;
            }
            if (is_find_drivable)
            {
                // second encounter of undrivable area. Done.
                if (value < m_drivable_threshold)
                {
                    break;
                }

                the_farthest_point_px.x = point_px.x;
                the_farthest_point_px.y = point_px.y;
            }
        }
    }
    else
    { // current position is drivable
        cv::LineIterator line_it(robotmap, cv::Point(source_point_px.x, source_point_px.y), cv::Point(dest_point_px.x, dest_point_px.y), 8);
        for (int i = 0; i < line_it.count; i++, ++line_it)
        {
            cv::Point point_px = line_it.pos();
            int value = robotmap.at<uchar>(point_px.y, point_px.x);

            // first encounter of undrivable area. Done.
            if (value < m_drivable_threshold)
            {
                break;
            }

            the_farthest_point_px.x = point_px.x;
            the_farthest_point_px.y = point_px.y;
        }
    }

    the_farthest_point = cvtMaptoRobotcoordinate(the_farthest_point_px);
    return the_farthest_point;
}

bool DGRobot::makeSubgoal12(Pose2 &pub_pose) // makeSubgoal11 with offline/online map alignment (offline no alignment)
{
    // in DeepGuider coordinate
    m_guider_mutex.lock();
    GuidanceManager::ExtendedPathElement prev_guide = m_guider.getPrevExtendedPath();
    GuidanceManager::ExtendedPathElement cur_guide = m_guider.getCurExtendedPath();
    GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();
    GuidanceManager::ExtendedPathElement next_next_guide = m_guider.getNextNextExtendedPath();
    m_guider_mutex.unlock();
    Pose2 dg_pose = m_localizer->getPose();

    ROS_INFO("[makeSubgoal12] DG Pose node_robot.x: %f, y:%f", dg_pose.x, dg_pose.y);

    Pose2 prev_node_dg = Point2(prev_guide);
    Pose2 cur_node_dg = Point2(cur_guide);
    Pose2 next_node_dg = Point2(next_guide);
    Pose2 next_next_node_dg = Point2(next_next_guide);

    ROS_INFO("[makeSubgoal12] prev_node_dg.x: %f, y:%f", prev_node_dg.x, prev_node_dg.y);
    ROS_INFO("[makeSubgoal12] cur_node_dg.x: %f, y:%f", cur_node_dg.x, cur_node_dg.y);
    ROS_INFO("[makeSubgoal12] next_node_dg.x: %f, y:%f", next_node_dg.x, next_node_dg.y);
    ROS_INFO("[makeSubgoal12] next_next_node_dg.x: %f, y:%f", next_next_node_dg.x, next_next_node_dg.y);

    // load robot info
    m_robotmap_mutex.lock();
    cv::Mat onlinemap = m_robotmap_image.clone(); // <map image>
    Pose2 robot_pose = m_guider.m_robot_pose;     // robot pose in robot's coordinate
    m_robotmap_mutex.unlock();

    ROS_INFO("[makeSubgoal12] robot_pose node_robot.x: %f, y:%f", robot_pose.x, robot_pose.y);

    // Pose2 dg_pose_robot = cvtDGtoDXcoordinate(dg_pose, dg_pose, robot_pose);  // dg_pose in robot's coordinate
    // Pose2 dg_pose_robot = cvtDg2Dx(dg_pose);  // dg_pose in robot's coordinate

    cv::Mat robotmap = onlinemap.clone();
    bool use_onlinemap = true;
    if (onlinemap.empty()) // on robot occumap
    {
        ROS_INFO("No occumap");
        robotmap = cv::imread(m_robotmap_path, cv::IMREAD_GRAYSCALE); // if no online map, read offline map
        use_onlinemap = false;
    }
    cv::Size robotmap_size = robotmap.size();
    m_robot_origin.x = -m_dx_map_origin_pixel.x * m_dx_map_meter_per_pixel;
    m_robot_origin.y = -m_dx_map_origin_pixel.y * m_dx_map_meter_per_pixel;

    // imwrite("../../../robotmap.png", robotmap);

    if (robotmap.empty())
    {
        ROS_INFO("No robotmap");
        return false;
    }

    // dilate robotmap  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 1)
    cv::Mat robotmap_dilate = robotmap.clone();
    int dilate_value = 2;
    dilate(robotmap, robotmap_dilate, cv::Mat::ones(cv::Size(dilate_value, dilate_value), CV_8UC1), cv::Point(-1, -1), 1);

    // based on the undrivable points, make the robotmap undrivable on those points  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 2)
    for (int i = 0; i < m_undrivable_points.size(); i++)
    {
        Point2 node_robot = m_undrivable_points[i];
        Point2 node_pixel = cvtRobottoMapcoordinate(node_robot);
        robotmap_dilate.at<uchar>((int)node_pixel.y, (int)node_pixel.x) = 0;
    }

    // smooth robotmap
    cv::Mat robotmap_smooth = robotmap_dilate.clone();
    ///////////////////////////////////////////////////////////////////////////////////
    // // Plan A-3: comment below (except the last line)
    // // Plan B-3: uncomment below (except the last line)
    // int smoothing_filter = 3; // filter size. Must be odd
    // cv::medianBlur(robotmap, robotmap_smooth, smoothing_filter);

    // // erode robotmap  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 3)
    int erode_value = 7 + dilate_value;
    // int erode_value = 0;  // Plan A-3: uncomment. Plan B-3: comment
    ///////////////////////////////////////////////////////////////////////////////
    cv::Mat robotmap_erode = robotmap_smooth.clone();
    erode(robotmap_smooth, robotmap_erode, cv::Mat::ones(cv::Size(erode_value, erode_value), CV_8UC1), cv::Point(-1, -1), 1);
    imwrite("../../../makesubgoal12_robotmap_dilate.png", robotmap_dilate);
    imwrite("../../../makesubgoal12_robotmap.png", robotmap);

    // save image
    cv::Mat colormap = robotmap_erode.clone();
    cv::Mat clean_colormap = robotmap.clone();
    cv::cvtColor(robotmap_erode, colormap, cv::COLOR_GRAY2BGR);
    cv::cvtColor(robotmap, clean_colormap, cv::COLOR_GRAY2BGR);

    imwrite("../../../makesubgoal12_robotmap_erode.png", robotmap_erode);

    ROS_INFO("[makeSubgoal12] CurGuideIdx %d", m_guider.getCurGuideIdx());

    int diff_deg_robot;
    double diff_dist_robot, new_theta;
    new_theta = robot_pose.theta;

    // convert variables with DG coordinate to DX coordinate
    //  Pose2 dg_prev_node_robot = cvtDGtoDXcoordinate(m_prev_node_dg, dg_pose, robot_pose);
    //  Pose2 dg_cur_node_robot = cvtDGtoDXcoordinate(cur_node_dg, dg_pose, robot_pose);  // Note: cur_node_dg is same with m_cur_node_dg except when m_cur_node_dg hasn't been assigned for the first time
    //  Pose2 dg_next_node_robot = cvtDGtoDXcoordinate(next_node_dg, dg_pose, robot_pose);
    //  Pose2 dg_next_next_node_robot = cvtDGtoDXcoordinate(next_next_node_dg, dg_pose, robot_pose);
    Pose2 dg_prev_node_robot = cvtDg2Dx(prev_node_dg);
    Pose2 dg_cur_node_robot = cvtDg2Dx(cur_node_dg);
    Pose2 dg_next_node_robot = cvtDg2Dx(next_node_dg);
    Pose2 dg_next_next_node_robot = cvtDg2Dx(next_next_node_dg);
    // m_prev_robot_pose;  //in robot coordinate
    // m_cur_robot_pose;  //in robot coordinate
    ROS_INFO("[makeSubgoal12] m_prev_robot_pose: <%f, %f>", m_prev_robot_pose.x, m_prev_robot_pose.y);
    ROS_INFO("[makeSubgoal12] m_cur_robot_pose: <%f, %f>", m_cur_robot_pose.x, m_cur_robot_pose.y);

    Pose2 target_node_dx;

    // ARRIVED  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 0)
    double dist_robot_to_next = norm(dg_next_node_robot - robot_pose);
    ROS_INFO("[makeSubgoal12] dist robot to next: %f", dist_robot_to_next);
    if (dg_next_next_node_robot.x == dg_next_node_robot.x && dg_next_next_node_robot.y == dg_next_node_robot.y && dist_robot_to_next < 2)
    {
        ROS_INFO("[makeSubgoal12] ARRIVED. use previous published pub_pose.");
        pub_pose = m_guider.m_subgoal_pose;
        return true;
    }

    // here, select new node goal (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 4)
    if (m_cur_head_node_id != next_guide.cur_node_id) // new dg_next_node_robot
    {
        if (m_prev_node_id != 0) // NOT initial start (when m_prev_node_robot is not 0 anymore)
        {
            ROS_INFO("[makeSubgoal12] prev_id: %zd, cur_id: %zd, next_id: %zd", m_prev_node_id, m_cur_head_node_id, next_guide.cur_node_id);

            // find theta of dg node in robot coordinate
            diff_deg_robot = m_guider.getDegree(dg_prev_node_robot, dg_cur_node_robot, dg_next_node_robot);

            // display theta and
            // string disp = "degree = " + std::to_string(diff_deg_robot);
            // cv::putText(colormap, disp, cur_node_robot_px, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 255, 0),5);
            // disp = "   rp = " + std::to_string(cx::cvtRad2Deg(robot_pose.theta));
            // cv::putText(colormap, disp, cv::Point(500, 50) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 255, 0),5);
            // disp = "   dp = " + std::to_string(cx::cvtRad2Deg(dg_pose.theta));
            // cv::putText(colormap, disp, cv::Point(500, 120) , cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 255, 0),5);
            // imwrite("../../../test_image_withtext.png", colormap);

            // distance (in meter) between dg current node and dg next node
            double dist_dgcur_to_dgnext = norm(dg_cur_node_robot - dg_next_node_robot);

            // error vector between dg node and current pose (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 4b)
            if (use_onlinemap && m_align_dg_nodes)
            {
                // consider current dx node is current robot position
                m_cur_node_dx = robot_pose;
                // calculate the error
                m_errorvec_dgnode_curpose = robot_pose - dg_cur_node_robot;
            }
            else
            { // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 4a)
                m_errorvec_dgnode_curpose.x = 0;
                m_errorvec_dgnode_curpose.y = 0;
                m_cur_node_dx.x = dg_cur_node_robot.x;
                m_cur_node_dx.y = dg_cur_node_robot.y;
            }

            // the next dx node is based on dg next node and the error. Same with prev node and next next node
            m_next_node_dx.x = dg_next_node_robot.x + m_errorvec_dgnode_curpose.x;
            m_next_node_dx.y = dg_next_node_robot.y + m_errorvec_dgnode_curpose.y;
            m_prev_node_dx.x = dg_prev_node_robot.x + m_errorvec_dgnode_curpose.x;
            m_prev_node_dx.y = dg_prev_node_robot.y + m_errorvec_dgnode_curpose.y;
            m_next_next_node_dx.x = dg_next_next_node_robot.x + m_errorvec_dgnode_curpose.x;
            m_next_next_node_dx.y = dg_next_next_node_robot.y + m_errorvec_dgnode_curpose.y;

            // is the new updated node problematic?? (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 5)
            Pose2 farthest_point_to_m_next_node_dx = getFarthestPoint(robotmap_erode, m_next_node_dx, robot_pose);
            Pose2 farthest_point_to_m_cur_node_dx = getFarthestPoint(robotmap_erode, m_cur_node_dx, robot_pose);

            double drivable_dist_robot_to_nextnode = norm(farthest_point_to_m_next_node_dx - robot_pose);
            double drivable_dist_robot_to_curnode = norm(farthest_point_to_m_cur_node_dx - robot_pose);

            if (drivable_dist_robot_to_nextnode >= drivable_dist_robot_to_curnode)
            {                                          // not problematic. Use default: next
                target_node_dx = m_next_node_dx;       // already shifted
                m_nodeupdated_but_problematic = false; // maybe not needed but just in case... just make it false again
            }
            else
            {                                   // problematic. Use current
                target_node_dx = m_cur_node_dx; // already shifted
                m_nodeupdated_but_problematic = true;
            }

            // visualize prev, cur, next dg nodes
            Point2 dg_prev_node_robot_px = cvtRobottoMapcoordinate(dg_prev_node_robot);           // red star
            Point2 dg_cur_node_robot_px = cvtRobottoMapcoordinate(dg_cur_node_robot);             // green star
            Point2 dg_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_node_robot);           // blue star
            Point2 dg_next_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_next_node_robot); // cyan star
            cv::drawMarker(colormap, dg_prev_node_robot_px, cv::Vec3b(0, 0, 255), 2, 40, 5);
            cv::drawMarker(colormap, dg_cur_node_robot_px, cv::Vec3b(0, 255, 0), 2, 20, 5);
            cv::drawMarker(colormap, dg_next_node_robot_px, cv::Vec3b(255, 0, 0), 2, 40, 5);
            cv::drawMarker(colormap, dg_next_next_node_robot_px, cv::Vec3b(255, 255, 0), 2, 40, 5);

            // visualize prev, cur, next (shifted) dx nodes
            Point2 dx_prev_node_robot_px = cvtRobottoMapcoordinate(m_prev_node_dx);           // red diamond
            Point2 dx_cur_node_robot_px = cvtRobottoMapcoordinate(m_cur_node_dx);             // green diamond
            Point2 dx_next_node_robot_px = cvtRobottoMapcoordinate(m_next_node_dx);           // blue diamond
            Point2 dx_next_next_node_robot_px = cvtRobottoMapcoordinate(m_next_next_node_dx); // cyan diamond
            cv::drawMarker(colormap, dx_prev_node_robot_px, cv::Vec3b(0, 0, 255), 3, 40, 5);
            cv::drawMarker(colormap, dx_cur_node_robot_px, cv::Vec3b(0, 255, 0), 3, 20, 5);
            cv::drawMarker(colormap, dx_next_node_robot_px, cv::Vec3b(255, 0, 0), 3, 40, 5);
            cv::drawMarker(colormap, dx_next_next_node_robot_px, cv::Vec3b(255, 255, 0), 3, 40, 5);
        }
        else // initial start (assume: not shifted at all and not problematic to go to next node)
        {
            m_prev_node_dx = dg_prev_node_robot; // no align with error
            m_cur_node_dx = dg_cur_node_robot;
            m_next_node_dx = dg_next_node_robot;
            m_next_next_node_dx = dg_next_next_node_robot;

            target_node_dx = m_next_node_dx;       // target as default. To next node
            m_nodeupdated_but_problematic = false; // maybe not needed but just in case... just make it false again
            // imwrite("../../../initial_start.png", colormap);

            // visualize cur, next dg nodes
            Point2 dg_prev_node_robot_px = cvtRobottoMapcoordinate(dg_prev_node_robot);           // red star
            Point2 dg_cur_node_robot_px = cvtRobottoMapcoordinate(dg_cur_node_robot);             // green star
            Point2 dg_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_node_robot);           // blue star
            Point2 dg_next_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_next_node_robot); // cyan star
            cv::drawMarker(colormap, dg_prev_node_robot_px, cv::Vec3b(0, 0, 255), 2, 40, 5);
            cv::drawMarker(colormap, dg_cur_node_robot_px, cv::Vec3b(0, 255, 0), 2, 20, 5);
            cv::drawMarker(colormap, dg_next_node_robot_px, cv::Vec3b(255, 0, 0), 2, 40, 5);
            cv::drawMarker(colormap, dg_next_next_node_robot_px, cv::Vec3b(255, 255, 0), 2, 40, 5);

            // visualize cur, next (shifted) dx nodes
            Point2 dx_prev_node_robot_px = cvtRobottoMapcoordinate(m_prev_node_dx);           // red diamond
            Point2 dx_cur_node_robot_px = cvtRobottoMapcoordinate(m_cur_node_dx);             // green diamond
            Point2 dx_next_node_robot_px = cvtRobottoMapcoordinate(m_next_node_dx);           // blue diamond
            Point2 dx_next_next_node_robot_px = cvtRobottoMapcoordinate(m_next_next_node_dx); // cyan diamond
            cv::drawMarker(colormap, dx_prev_node_robot_px, cv::Vec3b(0, 0, 255), 3, 40, 5);
            cv::drawMarker(colormap, dx_cur_node_robot_px, cv::Vec3b(0, 255, 0), 3, 20, 5);
            cv::drawMarker(colormap, dx_next_node_robot_px, cv::Vec3b(255, 0, 0), 3, 40, 5);
            cv::drawMarker(colormap, dx_next_next_node_robot_px, cv::Vec3b(255, 255, 0), 3, 40, 5);
        }

        // update global variable
        // m_prev_node_dg = m_cur_node_dg;
        // m_cur_node_dg = next_node_dg;
        m_prev_node_id = m_cur_head_node_id;
        m_cur_head_node_id = next_guide.cur_node_id;
        m_prev_robot_pose = m_cur_robot_pose;
        m_cur_robot_pose = robot_pose;

        m_guider.m_robot_heading_node_pose = dg_next_node_robot;
    }
    else
    { // if not new node
        // visualize cur, next (not yet shifted) dg nodes
        Point2 dg_prev_node_robot_px = cvtRobottoMapcoordinate(dg_prev_node_robot);           // red star
        Point2 dg_cur_node_robot_px = cvtRobottoMapcoordinate(dg_cur_node_robot);             // green star
        Point2 dg_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_node_robot);           // blue star
        Point2 dg_next_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_next_node_robot); // cyan star
        cv::drawMarker(colormap, dg_prev_node_robot_px, cv::Vec3b(0, 0, 255), 2, 40, 5);
        cv::drawMarker(colormap, dg_cur_node_robot_px, cv::Vec3b(0, 255, 0), 2, 20, 5);
        cv::drawMarker(colormap, dg_next_node_robot_px, cv::Vec3b(255, 0, 0), 2, 40, 5);
        cv::drawMarker(colormap, dg_next_next_node_robot_px, cv::Vec3b(255, 255, 0), 2, 40, 5);

        // visualize cur, next (shifted) dx nodes
        Point2 dx_prev_node_robot_px = cvtRobottoMapcoordinate(m_prev_node_dx);           // red diamond
        Point2 dx_cur_node_robot_px = cvtRobottoMapcoordinate(m_cur_node_dx);             // green diamond
        Point2 dx_next_node_robot_px = cvtRobottoMapcoordinate(m_next_node_dx);           // blue diamond
        Point2 dx_next_next_node_robot_px = cvtRobottoMapcoordinate(m_next_next_node_dx); // cyan diamond
        cv::drawMarker(colormap, dx_prev_node_robot_px, cv::Vec3b(0, 0, 255), 3, 40, 5);
        cv::drawMarker(colormap, dx_cur_node_robot_px, cv::Vec3b(0, 255, 0), 3, 20, 5);
        cv::drawMarker(colormap, dx_next_node_robot_px, cv::Vec3b(255, 0, 0), 3, 40, 5);
        cv::drawMarker(colormap, dx_next_next_node_robot_px, cv::Vec3b(255, 255, 0), 3, 40, 5);

        // imwrite("../../../not_initial_start.png", colormap);
        if (m_nodeupdated_but_problematic)
        {

            Pose2 farthest_point_to_m_next_node_dx = getFarthestPoint(robotmap_erode, m_next_node_dx, robot_pose);
            Pose2 farthest_point_to_m_cur_node_dx = getFarthestPoint(robotmap_erode, m_cur_node_dx, robot_pose);

            double drivable_dist_robot_to_nextnode = norm(farthest_point_to_m_next_node_dx - robot_pose);
            double drivable_dist_robot_to_curnode = norm(farthest_point_to_m_cur_node_dx - robot_pose);

            if (drivable_dist_robot_to_nextnode >= drivable_dist_robot_to_curnode)
            { // no more problematic (drivable distance to next node > drivable distance to cur node)
                m_nodeupdated_but_problematic = false;
                target_node_dx = m_next_node_dx; // already shifted
            }
            else
            {                                   // still problematic
                target_node_dx = m_cur_node_dx; // already shifted
            }
        }
        else
        {                                    // by default, go to the shifted next node
            target_node_dx = m_next_node_dx; // already shifted
        }
    }

    // distance from robot to the (aligned) next node
    double dist_robot_to_targetnode = norm(target_node_dx - robot_pose);

    // // new_theta for robot pose (facing the next node). Note: probably not used as for subgoal, we only need coordinate.
    // new_theta = robot_pose.theta + cx::cvtDeg2Rad(diff_deg_robot);  // current pose + how much to turn based on diff_deg_robot
    ROS_INFO("[makeSubgoal12] dist_robot_to_targetnode: %f", dist_robot_to_targetnode);

    // if not too close to the next node
    // get a point between robot pose and target_node_dx. 1 meter from the robot position to the direction of next_node_robot
    double sub_goal_distance = 2.0; // in meter. Acceptable distance
    double acceptable_error = 1.5;  // acceptable error between optimal and notsooptimal pub pose
    Pose2 optimal_pub_pose;
    optimal_pub_pose = getFarthestPoint(robotmap_erode, target_node_dx, robot_pose); // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 6a)
    double dist_optimalpubpose_robot = norm(robot_pose - optimal_pub_pose);

    // draw the pub_pose from the first step (regardless drivable or not)
    cv::drawMarker(colormap, cvtRobottoMapcoordinate(optimal_pub_pose), cv::Vec3b(255, 0, 255), 1, 10, 2); // purple small cross

    // pub pose if considering robot theta. But good to remove zigzag  // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 6b)
    Pose2 notsooptimal_target_node;
    notsooptimal_target_node.x = robot_pose.x + dist_robot_to_targetnode * cos(robot_pose.theta);
    notsooptimal_target_node.y = robot_pose.y + dist_robot_to_targetnode * sin(robot_pose.theta);
    Pose2 notsooptimal_pub_pose;
    notsooptimal_pub_pose = getFarthestPoint(robotmap_erode, notsooptimal_target_node, robot_pose);
    double dist_notsooptimalpubpose_robot = norm(robot_pose - notsooptimal_pub_pose);

    // draw the the notsooptimal_pub_pose
    cv::drawMarker(colormap, cvtRobottoMapcoordinate(notsooptimal_pub_pose), cv::Vec3b(255, 0, 255), 4, 10, 2);    // purple small cross
    cv::drawMarker(colormap, cvtRobottoMapcoordinate(notsooptimal_target_node), cv::Vec3b(255, 0, 255), 4, 40, 2); // purple big cross

    double error_nextnode_notsooptimalnextnode = norm(target_node_dx - notsooptimal_target_node);

    // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 6c)
    if (dist_optimalpubpose_robot > sub_goal_distance && dist_notsooptimalpubpose_robot <= sub_goal_distance)
    { // if not so optimal subgoal is less than 5 but the optimal is more than 5
        pub_pose.x = optimal_pub_pose.x;
        pub_pose.y = optimal_pub_pose.y;
    }
    else if (dist_optimalpubpose_robot <= sub_goal_distance && dist_notsooptimalpubpose_robot > sub_goal_distance && error_nextnode_notsooptimalnextnode < acceptable_error)
    {
        pub_pose.x = notsooptimal_pub_pose.x;
        pub_pose.y = notsooptimal_pub_pose.y;
    }
    else if (dist_optimalpubpose_robot > sub_goal_distance && dist_notsooptimalpubpose_robot > sub_goal_distance)
    { // both dist_optimalpubpose_robot and dist_notsooptimalpubpose_robot are > sub_goal_distance
        // if no big difference between notsooptimal pub pose and optimal pubpose, use the notsooptimal pub pose
        double error_optimal_notsooptimal = norm(notsooptimal_pub_pose - optimal_pub_pose);
        if (error_optimal_notsooptimal > acceptable_error)
        {
            pub_pose.x = optimal_pub_pose.x;
            pub_pose.y = optimal_pub_pose.y;
        }
        else
        {
            pub_pose.x = notsooptimal_pub_pose.x;
            pub_pose.y = notsooptimal_pub_pose.y;
        }
    }
    else
    {                                                                                        // both dist_optimalpubpose_robot and dist_notsooptimalpubpose_robot are <= sub_goal_distance
        std::vector<double> alternative_sub_goal_distances = {7.0, 6.0, 5.0, 4.0, 3.0, 2.0}; // {3.0} for visualization
        std::vector<int> num_alternatives = {128, 128, 128, 128, 96, 64};                    // {4} for visualization
        double angle_range = 90;                                                             // 180;  // max 180 (whole circle)
        bool isAlternativeFound;
        double min_dist_temp = 1000000; // 100 km

        /////////////////////////////////////////////////////////////////////////////////////////////
        Pose2 pub_pose_temp;
        double min_dist_best = 1000000; // 100 km

        // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 7)
        for (int i = 0; i < num_alternatives.size(); i++)
        {
            double min_dist_temp;
            isAlternativeFound = findAlternativePathv2(pub_pose_temp, robot_pose, target_node_dx, robotmap_erode, alternative_sub_goal_distances.at(i), dist_robot_to_targetnode, num_alternatives.at(i), colormap, angle_range, min_dist_temp);
            if (isAlternativeFound && min_dist_temp < min_dist_best)
            {
                pub_pose = pub_pose_temp;
                min_dist_best = min_dist_temp;
            }
        }
        if (min_dist_best == 1000000)
        {
            isAlternativeFound = false;
        }
        else
        {
            isAlternativeFound = true;
        }
        // /////////////////////////////////////////////////////////////////////////////////////////////

        if (!isAlternativeFound)
        {
            ROS_INFO("can't find sub goal :(");
            if (m_save_guidance_video)
            {
                Pose2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose);
                cv::putText(colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255), 5);
                cv::putText(clean_colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255), 5);

                // record image
                //  ///save image
                //  Point2 dg_pose_robot_px = cvtRobottoMapcoordinate(dg_pose_robot);
                //  Point2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose);  // dx_pose_robot_px = dg_pose_robot_px

                cv::circle(colormap, robot_pose_px, 20, cv::Vec3b(0, 255, 0), 5);
                cv::circle(colormap, robot_pose_px, 5, cv::Vec3b(0, 255, 0), 2);       // with robot real size
                cv::circle(clean_colormap, robot_pose_px, 5, cv::Vec3b(0, 255, 0), 2); // with robot real size
                // cv::circle(colormap, dx_pose_robot_px, 20, cv::Vec3b(0, 0, 255), 5);

                Point2 robot_heading;
                robot_heading.x = robot_pose_px.x + 20 * cos(robot_pose.theta);
                robot_heading.y = robot_pose_px.y + 20 * sin(robot_pose.theta);
                cv::line(colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 255, 0), 5);
                robot_heading.x = robot_pose_px.x + 5 * cos(robot_pose.theta);
                robot_heading.y = robot_pose_px.y + 5 * sin(robot_pose.theta);
                cv::line(clean_colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 255, 0), 2);
                ROS_INFO("robot_pose theta %f", robot_pose.theta);

                cv::drawMarker(colormap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), 0, 50, 10);
                imwrite("../../../makesubgoal12_onlinemap.png", colormap);

                // record video
                cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);
                cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
                colormap.copyTo(roi);
                if (m_record_robotmap) m_video_robotmap << videoFrame;

                cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);
                cv::Rect roi_rc(robot_pose_px.x - m_crop_radius, robot_pose_px.y - m_crop_radius, 2 * m_crop_radius + 1, 2 * m_crop_radius + 1);
                roi_rc = roi_rc & cv::Rect(0, 0, colormap.cols, colormap.rows);
                colormap(roi_rc).copyTo(videoFrameCrop(cv::Rect(0, 0, roi_rc.width, roi_rc.height)));
                m_video_guidance_crop << videoFrameCrop;
                imwrite("../../../makesubgoal12_onlinemapcrop.png", videoFrameCrop);

                clean_colormap(roi_rc).copyTo(videoFrameCrop(cv::Rect(0, 0, roi_rc.width, roi_rc.height)));
                if (m_record_robotmap_crop) m_video_robotmap_crop << videoFrameCrop;
            }
            // // Plan B-6
            // // make current pose undrivable. To prevent repeated position
            // m_undrivable_points.push_back(robot_pose);

            return false; // can't find alternative :(
        }
    }

    ////////////////////////////////////////////////////////////
    // CALCULATE THETA (GOOGLE DOCS - Subgoal Theta Calculation)

    // theta if going from cur to next

    ROS_INFO("[makeSubgoal12] before theta m_prev_node_dx.x: %f, y:%f", m_prev_node_dx.x, m_prev_node_dx.y);
    ROS_INFO("[makeSubgoal12] before theta m_cur_node_dx.x: %f, y:%f", m_cur_node_dx.x, m_cur_node_dx.y);
    ROS_INFO("[makeSubgoal12] before theta m_next_node_dx.x: %f, y:%f", m_next_node_dx.x, m_next_node_dx.y);
    ROS_INFO("[makeSubgoal12] before theta m_next_next_node_dx.x: %f, y:%f", m_next_next_node_dx.x, m_next_next_node_dx.y);

    double theta_to_next_next = cx::cvtDeg2Rad(findRobotCoordAngleofVectorSource2Dest(m_next_node_dx, m_next_next_node_dx));
    double theta_to_next = cx::cvtDeg2Rad(findRobotCoordAngleofVectorSource2Dest(m_cur_node_dx, m_next_node_dx));
    double theta_to_cur = cx::cvtDeg2Rad(findRobotCoordAngleofVectorSource2Dest(m_prev_node_dx, m_cur_node_dx));
    double dist_pubpose_to_targetnode = norm(target_node_dx - pub_pose);                                     // (GOOGLE DOCS - Subgoal Theta Calculation - STEP 1)
    double ratio_for_theta = std::min(dist_pubpose_to_targetnode / m_guider.m_uncertain_dist_public(), 1.0); // (GOOGLE DOCS - Subgoal Theta Calculation - STEP 2)

    if (m_nodeupdated_but_problematic)
    { // (GOOGLE DOCS - Subgoal Theta Calculation - STEP 4)
        // use ratio of subgoal-target node distance : m_guider.m_uncertain_dist to decide the direction. The smaller the distance, head to next more.
        pub_pose.theta = (1 - ratio_for_theta) * theta_to_next + ratio_for_theta * theta_to_cur;
        ROS_INFO("Find theta case 1: <%f>", cx::cvtRad2Deg(pub_pose.theta));
    }
    else
    { // if not problematic (GOOGLE DOCS - Subgoal Theta Calculation - STEP 3)
        if (dist_pubpose_to_targetnode < m_guider.m_uncertain_dist_public())
        {                                                                                                  // usually happen when target node (next node) is on vicinity but far away so the node hasn't been updated
            pub_pose.theta = (1 - ratio_for_theta) * theta_to_next_next + ratio_for_theta * theta_to_next; // (GOOGLE DOCS - Subgoal Theta Calculation - STEP 3a)
            ROS_INFO("Find theta case 2: <%f>", cx::cvtRad2Deg(pub_pose.theta));
        }
        else
        {
            pub_pose.theta = theta_to_next; // by default, go to next (GOOGLE DOCS - Subgoal Theta Calculation - STEP 3b)
            ROS_INFO("Find theta case 3: <%f>", cx::cvtRad2Deg(pub_pose.theta));
        }
    }

    // CALCULATE THETA END
    ////////////////////////////////////////////////////////////

    ROS_INFO("Found subgoal: <%f, %f, %f>", pub_pose.x, pub_pose.y, cx::cvtRad2Deg(pub_pose.theta)); // OUTPUT.. care about pub_pose in robot's coordinate
    Pose2 pub_pose_px = cvtRobottoMapcoordinate(pub_pose);
    cv::circle(colormap, pub_pose_px, 20, cv::Vec3b(255, 0, 255), 5);      // small purple circle
    cv::circle(colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2);       // with robot real size
    cv::circle(clean_colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2); // with robot real size
    Pose2 pubpose_heading;
    pubpose_heading.x = pub_pose_px.x + 20 * cos(pub_pose.theta);
    pubpose_heading.y = pub_pose_px.y + 20 * sin(pub_pose.theta);
    cv::line(colormap, pub_pose_px, pubpose_heading, cv::Vec3b(255, 0, 255), 5);
    pubpose_heading.x = pub_pose_px.x + 5 * cos(pub_pose.theta);
    pubpose_heading.y = pub_pose_px.y + 5 * sin(pub_pose.theta);
    cv::line(clean_colormap, pub_pose_px, pubpose_heading, cv::Vec3b(255, 0, 255), 2);

    ///////////////////////////////////////////////////
    if (m_save_guidance_video)
    {
        // record image
        //  ///save image
        //  Point2 dg_pose_robot_px = cvtRobottoMapcoordinate(dg_pose_robot);
        Point2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose); // robot_pose_px = dg_pose_robot_px

        cv::circle(colormap, robot_pose_px, 20, cv::Vec3b(0, 255, 0), 5);
        cv::circle(colormap, robot_pose_px, 5, cv::Vec3b(0, 255, 0), 2);       // with robot real size
        cv::circle(clean_colormap, robot_pose_px, 5, cv::Vec3b(0, 255, 0), 2); // with robot real size
        // cv::circle(colormap, dx_pose_robot_px, 20, cv::Vec3b(0, 0, 255), 5);

        // //draw prev and current robot pose
        // Point2 prev_robot_px = cvtRobottoMapcoordinate(m_prev_robot_pose);
        // Point2 cur_robot_px = cvtRobottoMapcoordinate(m_cur_robot_pose);
        // cv::circle(clean_colormap, prev_robot_px, 10, cv::Vec3b(100, 100, 255), 2);  // with robot real size
        // cv::circle(clean_colormap, cur_robot_px, 10, cv::Vec3b(100, 255, 100), 2);  // with robot real size
        // cv::circle(colormap, prev_robot_px, 10, cv::Vec3b(100, 100, 255), 2);  // with robot real size
        // cv::circle(colormap, cur_robot_px, 10, cv::Vec3b(100, 255, 100), 2);  // with robot real size

        Point2 robot_heading;
        robot_heading.x = robot_pose_px.x + 20 * cos(robot_pose.theta);
        robot_heading.y = robot_pose_px.y + 20 * sin(robot_pose.theta);
        cv::line(colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 255, 0), 5);
        robot_heading.x = robot_pose_px.x + 5 * cos(robot_pose.theta);
        robot_heading.y = robot_pose_px.y + 5 * sin(robot_pose.theta);
        cv::line(clean_colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 255, 0), 2);
        ROS_INFO("robot_pose theta %f", robot_pose.theta);

        cv::drawMarker(colormap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), 0, 50, 10);
        imwrite("../../../makesubgoal12_onlinemap.png", colormap);

        // record video
        cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);
        cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
        colormap.copyTo(roi);
        if (m_record_robotmap) m_video_robotmap << videoFrame;

        cv::Mat videoFrameCrop = cv::Mat::zeros(m_framesize_crop, CV_8UC3);
        cv::Rect roi_rc(robot_pose_px.x - m_crop_radius, robot_pose_px.y - m_crop_radius, 2 * m_crop_radius + 1, 2 * m_crop_radius + 1);
        roi_rc = roi_rc & cv::Rect(0, 0, colormap.cols, colormap.rows);
        colormap(roi_rc).copyTo(videoFrameCrop(cv::Rect(0, 0, roi_rc.width, roi_rc.height)));
        m_video_guidance_crop << videoFrameCrop;
        imwrite("../../../makesubgoal12_onlinemapcrop.png", videoFrameCrop);

        clean_colormap(roi_rc).copyTo(videoFrameCrop(cv::Rect(0, 0, roi_rc.width, roi_rc.height)));
        if (m_record_robotmap_crop) m_video_robotmap_crop << videoFrameCrop;
    }

    // adjust subgoal (prevent approach to obstacles too close)
    Point2 delta_v = pub_pose - robot_pose;
    double subgoal_d = norm(delta_v);
    printf("[subgoal] d = %lf\n", subgoal_d);
    double subgoal_theta = pub_pose.theta;
    double new_delta = subgoal_d - 1.0;
    if(subgoal_d<5) new_delta = subgoal_d * 0.8;
    pub_pose = robot_pose + delta_v * new_delta / subgoal_d;
    pub_pose.theta = subgoal_theta;

    // draw robot
    Point2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose); // robot_pose_px = dg_pose_robot_px
    cv::circle(colormap, robot_pose_px, 20, cv::Vec3b(0, 200, 0), 4);
    Point2 robot_heading;
    robot_heading.x = robot_pose_px.x + 20 * cos(robot_pose.theta);
    robot_heading.y = robot_pose_px.y + 20 * sin(robot_pose.theta);
    cv::line(colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 200, 0), 4);

    cv::Mat crop_flip;
    cv::Rect roi_rc(robot_pose_px.x - m_crop_radius, robot_pose_px.y - m_crop_radius, 2 * m_crop_radius + 1, 2 * m_crop_radius + 1);
    roi_rc = roi_rc & cv::Rect(0, 0, colormap.cols, colormap.rows);
    cv::flip(colormap(roi_rc), crop_flip, 0);

    // regend annotations
    cv::Point px(crop_flip.cols - 150, 20);
    int dy = 25;
    cv::putText(crop_flip, "subgoal", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    cv::putText(crop_flip, "x subA", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    cv::putText(crop_flip, "o subB", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    cv::putText(crop_flip, "[]target", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    cv::putText(crop_flip, "prevN", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(0, 0, 255), 1); px.y+=dy;
    cv::putText(crop_flip, "curN", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(0, 255, 0), 1); px.y+=dy;
    cv::putText(crop_flip, "nextN", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 0), 1); px.y+=dy;
    cv::putText(crop_flip, "nnextN", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 255, 0), 1); px.y+=dy;
    cv::putText(crop_flip, "*: dg", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(128, 128, 128), 1); px.y+=dy;
    cv::putText(crop_flip, "<>: dx", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(128, 128, 128), 1); px.y+=dy;

    cv::imshow("guidance_map", crop_flip);
    cv::waitKey(1);

    // // Plan B-5
    // // make current pose undrivable. To prevent repeated position
    // m_undrivable_points.push_back(robot_pose);

    return true;
}

bool DGRobot::findDrivableinLine(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2 &result_px)
{
    if (image.empty())
        return false;

    ROS_INFO("robot_px:<%f, %f>, node_px:<%f, %f>", robot_px.x, robot_px.y, node_px.x, node_px.y);
    cv::LineIterator line_it(image, robot_px, node_px, 8);
    Point2 point_px;
    Point2 prev_px = robot_px;
    for (int i = 0; i < line_it.count; i++, ++line_it)
    {
        point_px = line_it.pos();
        int value = image.at<uchar>(point_px.y, point_px.x);
        ROS_INFO("Drivable value: %d <%f, %f>", value, point_px.x, point_px.y);
        if (value < m_drivable_threshold)
        {
            ROS_INFO("Meet undrivable value: %d <%f, %f>", value, point_px.x, point_px.y);
            break;
        }
        prev_px = point_px;
    }

    double dist = norm(prev_px - robot_px) / m_robotmap_scale;
    if (dist < m_min_goal_dist) // no drivble area on the line
    {
        ROS_INFO("subgoal is too near <%f, %f>. dist: %f", prev_px.x, prev_px.y, dist);
        return false;
    }

    result_px = prev_px;

    // save to image
    cv::Mat img_color;
    image.copyTo(img_color);
    if (img_color.channels() == 1)
        cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2BGR);

    cv::line(img_color, robot_px, node_px, cv::Vec3b(0, 0, 255), 2);
    cv::circle(img_color, node_px, 10, cv::Vec3b(0, 0, 255), 2);
    cv::circle(img_color, robot_px, 10, cv::Vec3b(0, 255, 0), 2);
    cv::circle(img_color, result_px, 10, cv::Vec3b(0, 255, 255), 2);
    cv::imwrite("../../../finddrivableLine.png", img_color);

    return true;
}

bool DGRobot::findExtendedDrivablePoint(cv::Mat &image, Point2 robot_px, Point2 node_px, Point2 &result_px)
{
    if (image.empty())
        return false;

    // save to image
    cv::Mat img_color;
    image.copyTo(img_color);
    if (img_color.channels() == 1)
    {
        cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2BGR);
    }

    Point2 px_diff = node_px - robot_px;
    double base_rad = atan2(px_diff.y, px_diff.x);
    double max_dist = norm(px_diff);

    std::map<int, Point2, greater<double>> candidate1;
    std::map<double, int, greater<double>> candidate2;
    std::map<int, int> candidate3;

    // search -30~30 drivable point
    double jump_rad, dist;
    Point2 max_px, prev_px;
    for (int deg = -30; deg < 30; deg++)
    {
        jump_rad = base_rad + cx::cvtDeg2Rad(deg);
        ROS_INFO("Searching %f deg, base_deg: %f", cx::cvtRad2Deg(jump_rad), cx::cvtRad2Deg(base_rad));

        max_px.x = robot_px.x + max_dist * cos(jump_rad);
        max_px.y = robot_px.y + max_dist * sin(jump_rad);

        cv::LineIterator line_it(image, robot_px, max_px, 8);
        Point2 point_px;
        Point2 prev_px = robot_px;

        for (int i = 0; i < line_it.count; i++, ++line_it)
        {
            point_px = line_it.pos();
            int value = image.at<uchar>(point_px.y, point_px.x);

            // find drivable area on the based on robot_px
            if (value < m_drivable_threshold)
            {
                dist = norm(prev_px - robot_px) / m_robotmap_scale;
                if (dist > 1)
                {
                    cv::line(img_color, robot_px, prev_px, cv::Vec3b(255, 255, 0), 1);
                    candidate1.insert(make_pair(deg, prev_px));
                    candidate2.insert(make_pair(dist, deg));
                    ROS_INFO("Find drivable point dist: %d, %f, <%f, %f>", deg, dist, prev_px.x, prev_px.y);
                    break;
                }
            }
            prev_px = point_px;
        }
    }

    if (candidate1.size() == 0)
    {
        ROS_INFO("Cannot find goal. all dark. Provide drivable map.");
        cv::circle(img_color, node_px, 10, cv::Vec3b(0, 0, 255), 2);
        cv::circle(img_color, robot_px, 10, cv::Vec3b(0, 255, 0), 2);
        cv::imwrite("../../../finddrivablepoint.png", img_color);
        return false;
    }

    for (auto itr = candidate2.begin(); itr != candidate2.end(); itr++)
    {
        dist = itr->first;
        int deg = itr->second;
        if (dist > m_min_goal_dist)
        {
            candidate3.insert(make_pair(abs(deg), deg));
            ROS_INFO("Find drivable point abs(deg)%d, deg: %d, dist: %f", abs(deg), deg, dist);
        }
    }

    int deg;
    // if there is no over 3m, select longest
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

    // find corresponding deg-pixel;
    Point2 jump_px;
    for (auto itr1 = candidate1.begin(); itr1 != candidate1.end(); itr1++)
    {
        if (itr1->first == deg)
        {
            jump_px = itr1->second;
            ROS_INFO("Found drivable point: <%f, %f>", jump_px.x, jump_px.y);
            result_px = jump_px;

            cv::circle(img_color, jump_px, 10, cv::Vec3b(255, 0, 255), 2);
            cv::imwrite("../../../finddrivablepoint.png", img_color);

            return true;
        }
    }

    return false;
}

geometry_msgs::PoseStamped DGRobot::makeRosPubPoseMsg(ID nid, dg::Pose2 pose)
{
    // check publish time
    geometry_msgs::PoseStamped rosps;
    rosps.header.stamp = ros::Time::now();
    rosps.header.frame_id = to_string(nid);
    rosps.pose.position.x = pose.x;
    rosps.pose.position.y = pose.y;
    rosps.pose.position.z = 0;

    cv::Vec4d q = cx::cvtEulerAng2Quat(0, 0, pose.theta);
    rosps.pose.orientation.w = q(0);
    rosps.pose.orientation.x = q(1);
    rosps.pose.orientation.y = q(2);
    rosps.pose.orientation.z = q(3);

    return rosps;
}

void DGRobot::initialize_DG_DX_conversion()
{
    // DX 로봇 부천 지도 origin 계산
    double ori_x_rotated = (m_dx_map_origin_pixel.x - m_dx_map_ref_pixel.x) * m_dx_map_meter_per_pixel;
    double ori_y_rotated = (m_dx_map_origin_pixel.y - m_dx_map_ref_pixel.y) * m_dx_map_meter_per_pixel;

    // rotate ori_rotated to robot's map
    double ori_x = ori_x_rotated * cos(m_dx_map_rotation_radian) - ori_y_rotated * sin(m_dx_map_rotation_radian);
    double ori_y = ori_x_rotated * sin(m_dx_map_rotation_radian) + ori_y_rotated * cos(m_dx_map_rotation_radian);

    // calculate m_dx_map_origin_utm
    dg::UTMConverter converter;
    dg::Point2UTM dx_map_ref_utm = converter.cvtLatLon2UTM(m_dx_map_ref_latlon);
    printf("Robot map ref lat = %lf, lon = %lf\n", m_dx_map_ref_latlon.lat, m_dx_map_ref_latlon.lon);
    m_dx_map_origin_utm.x = dx_map_ref_utm.x + ori_x;
    m_dx_map_origin_utm.y = dx_map_ref_utm.y + ori_y;
    m_dx_map_origin_utm.zone = dx_map_ref_utm.zone;
    m_dx_map_origin_latlon = converter.cvtUTM2LatLon(m_dx_map_origin_utm);
    printf("Robot map origin lat = %lf, lon = %lf\n", m_dx_map_origin_latlon.lat, m_dx_map_origin_latlon.lon);

    m_dx_converter.setReference(m_dx_map_origin_latlon);
    m_dg_converter.setReference(m_dg_map_origin_latlon);
    m_dg_map_origin_utm = converter.cvtLatLon2UTM(m_dg_map_origin_latlon);
    printf("dg map origin utm: x = %f, y = %f\n", m_dg_map_origin_utm.x, m_dg_map_origin_utm.y);

    // display origin in gui
    Pose2 robot_origin_metric = m_map.toMetric(m_dx_map_origin_latlon);
    Point2 robot_origin = m_painter.cvtValue2Pixel(robot_origin_metric);
    cv::circle(m_map_image, robot_origin, 10, cv::Vec3b(0, 255, 255), 10);

    Pose2 robot_ref_metric = m_map.toMetric(m_dx_map_ref_latlon);
    Point2 robot_ref = m_painter.cvtValue2Pixel(robot_ref_metric);
    cv::circle(m_map_image, robot_ref, 10, cv::Vec3b(255, 0, 0), 10);
}

Point2 DGRobot::cvtDg2Dx(const Point2 &dg_metric) const
{
    // from dg metric to utm
    dg::Point2UTM dg_utm;
    dg_utm.x = m_dg_map_origin_utm.x + dg_metric.x;
    dg_utm.y = m_dg_map_origin_utm.y + dg_metric.y;

    // from dg_utm to dx map. but it's rotated.
    Point2 dg_dx_rotated;
    dg_dx_rotated.x = dg_utm.x - m_dx_map_origin_utm.x;
    dg_dx_rotated.y = dg_utm.y - m_dx_map_origin_utm.y;

    // rotate utm to robot's coordinate
    Point2 dg_dx;
    double theta = -m_dx_map_rotation_radian;
    dg_dx.x = dg_dx_rotated.x * cos(theta) - dg_dx_rotated.y * sin(theta);
    dg_dx.y = dg_dx_rotated.x * sin(theta) + dg_dx_rotated.y * cos(theta);

    return dg_dx;
}

Point2 DGRobot::cvtDx2Dg(const Point2 &dx_metric) const
{
    // rotate robot's coordinate to utm
    Point2 dx_rotated;
    double theta = m_dx_map_rotation_radian;
    dx_rotated.x = dx_metric.x * cos(theta) - dx_metric.y * sin(theta);
    dx_rotated.y = dx_metric.x * sin(theta) + dx_metric.y * cos(theta);

    // from dx_utm to dg utm.
    Point2 dx_utm;
    dx_utm.x = dx_rotated.x + m_dx_map_origin_utm.x;
    dx_utm.y = dx_rotated.y + m_dx_map_origin_utm.y;

    // from dx utm to dg_metric
    dg::Point2UTM dg_metric;
    dg_metric.x = dx_utm.x - m_dg_map_origin_utm.x;
    dg_metric.y = dx_utm.y - m_dg_map_origin_utm.y;

    return dg_metric;
}

Point2 DGRobot::cvtMetric2Pixel(const Point2 &val) const
{
    Point2 px;
    px.x = m_dx_map_origin_pixel.x + val.x / m_dx_map_meter_per_pixel;
    px.y = m_dx_map_origin_pixel.y + val.y / m_dx_map_meter_per_pixel;

    return px;
}

Point2 DGRobot::cvtPixel2Metric(const Point2 &px) const
{
    CV_DbgAssert(m_px_per_val.x > 0 && m_px_per_val.y > 0);

    Point2 val;
    val.x = (px.x - m_dx_map_origin_pixel.x) * m_dx_map_meter_per_pixel;
    val.y = (px.y - m_dx_map_origin_pixel.y) * m_dx_map_meter_per_pixel;

    return val;
}

void DGRobot::record2Video(cv::Mat colormap, cv::Mat clean_colormap, Pose2 robot_pose)
{
    Pose2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose);

    cv::circle(colormap, robot_pose_px, 20, cv::Vec3b(0, 255, 0), 5);
    cv::circle(colormap, robot_pose_px, 5, cv::Vec3b(0, 255, 0), 2);       // with robot real size
    cv::circle(clean_colormap, robot_pose_px, 5, cv::Vec3b(0, 255, 0), 2); // with robot real size

    Point2 robot_heading;
    robot_heading.x = robot_pose_px.x + 20 * cos(robot_pose.theta);
    robot_heading.y = robot_pose_px.y + 20 * sin(robot_pose.theta);
    cv::line(colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 255, 0), 5);
    robot_heading.x = robot_pose_px.x + 5 * cos(robot_pose.theta);
    robot_heading.y = robot_pose_px.y + 5 * sin(robot_pose.theta);
    cv::line(clean_colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 255, 0), 2);
    ROS_INFO("robot_pose theta %f", robot_pose.theta);

    cv::drawMarker(colormap, m_dx_map_origin_pixel, cv::Vec3b(0, 255, 255), 0, 50, 10);
    if (m_save_guidance_image) imwrite("../../../makesubgoal13_onlinemap.png", colormap);

    // record video
    m_frame_crop = 0;
    cv::Rect roi_rc(robot_pose_px.x - m_crop_radius, robot_pose_px.y - m_crop_radius, 2 * m_crop_radius + 1, 2 * m_crop_radius + 1);
    roi_rc = roi_rc & cv::Rect(0, 0, colormap.cols, colormap.rows);
    colormap(roi_rc).copyTo(m_frame_crop(cv::Rect(0, 0, roi_rc.width, roi_rc.height)));
    m_video_guidance_crop << m_frame_crop;
    if (m_save_guidance_image) imwrite("../../../makesubgoal13_onlinemapcrop.png", m_frame_crop);

    if (m_record_robotmap)
    {
        cv::Mat videoFrame = cv::Mat::zeros(m_framesize, CV_8UC3);
        cv::Mat roi(videoFrame, cv::Rect(0, 0, colormap.cols, colormap.rows));
        colormap.copyTo(roi);
        m_video_robotmap << videoFrame;
    }
    if (m_record_robotmap_crop)
    {
        clean_colormap(roi_rc).copyTo(m_frame_crop(cv::Rect(0, 0, roi_rc.width, roi_rc.height)));
        m_video_robotmap_crop << m_frame_crop;
    }
}

void DGRobot::record2VideoCropped(cv::Mat crop)
{
    m_frame_crop = 0;
    cv::Rect roi_rc(0, 0, crop.cols, crop.rows);
    roi_rc &= cv::Rect(0, 0, m_frame_crop.cols, m_frame_crop.rows);
    crop.copyTo(m_frame_crop(roi_rc));
    m_video_guidance_crop << m_frame_crop;
    if (m_save_guidance_image) imwrite("../../../makesubgoal13_onlinemapcrop.png", m_frame_crop);
}

Point2 DGRobot::getFarthestPointSafe(cv::Mat robotmap, Point2 dest_point, Point2 source_point, double min_space_to_obstacle)
{
    // return false if no drivable area (including source itself)
    // return the_farthest_point

    // pointA and pointB in robot coordinate
    Point2 dest_point_px = cvtRobottoMapcoordinate(dest_point);
    Point2 source_point_px = cvtRobottoMapcoordinate(source_point);
    Point2 the_farthest_point;
    Point2 the_farthest_point_px;

    cv::Rect robotmap_rc(0, 0, robotmap.cols, robotmap.rows);

    if (robotmap.at<uchar>(source_point_px.y, source_point_px.x) < m_drivable_threshold)
    { // current position is NOT drivable
        bool boundary_crossed = false;
        Point2 extended_dest = source_point_px + 2*(dest_point_px - source_point_px);
        Point2 first_drivable_pt = source_point_px;
        cv::LineIterator line_it(robotmap, cv::Point(source_point_px.x, source_point_px.y), cv::Point(extended_dest.x, extended_dest.y), 8);
        bool is_find_drivable = false;
        for (int i = 0; i < line_it.count; i++, ++line_it)
        {
            cv::Point point_px = line_it.pos();
            if (!robotmap_rc.contains(point_px))
            {
                boundary_crossed = true;
                break;
            }
            int value = robotmap.at<uchar>(point_px.y, point_px.x);

            if (i == 0)
            {
                the_farthest_point_px.x = point_px.x;
                the_farthest_point_px.y = point_px.y;
            }

            if (!is_find_drivable && value >= m_drivable_threshold) // first encounter of drivable area
            {
                is_find_drivable = true;
                first_drivable_pt.x = point_px.x;
                first_drivable_pt.y = point_px.y;
            }
            if (is_find_drivable)
            {
                // second encounter of undrivable area. Done.
                if (value < m_drivable_threshold)
                {
                    break;
                }
                the_farthest_point_px = point_px;
            }
        }

        double first_d = norm(first_drivable_pt - source_point_px);
        double dest_d = norm(dest_point_px - source_point_px);
        double farthest_d = norm(the_farthest_point_px - source_point_px);
        if (!is_find_drivable)
        {
            the_farthest_point_px = source_point_px;    // error
        }
        else if (first_d >= dest_d)
        {
            the_farthest_point_px = first_drivable_pt;
        }
        else if (farthest_d > dest_d + min_space_to_obstacle)
        {
            the_farthest_point_px = dest_point_px;
        }
        else if (farthest_d <= min_space_to_obstacle)
        {
            if (dest_d < farthest_d)
            {
                the_farthest_point_px = dest_point_px;
            }
        }
        else
        {
            double alpha = min_space_to_obstacle;
            double beta = farthest_d - min_space_to_obstacle;
            Point2 old_farthest_point = the_farthest_point_px;
            the_farthest_point_px = (the_farthest_point_px * beta + source_point_px * alpha) / (alpha + beta);
            if (norm(the_farthest_point_px - source_point_px) <= first_d)
            {
                the_farthest_point_px = (first_drivable_pt + old_farthest_point) / 2.0;
            }
        }
    }
    else
    { // current position is drivable
        bool boundary_crossed = false;
        Point2 extended_dest = source_point_px + 2*(dest_point_px - source_point_px);
        cv::LineIterator line_it(robotmap, cv::Point(source_point_px.x, source_point_px.y), cv::Point(extended_dest.x, extended_dest.y), 8);
        for (int i = 0; i < line_it.count; i++, ++line_it)
        {
            cv::Point point_px = line_it.pos();
            if (!robotmap_rc.contains(point_px))
            {
                boundary_crossed = true;
                break;
            }
            int value = robotmap.at<uchar>(point_px.y, point_px.x);

            // first encounter of undrivable area. Done.
            if (value < m_drivable_threshold)
            {
                break;
            }

            the_farthest_point_px.x = point_px.x;
            the_farthest_point_px.y = point_px.y;
        }

        double dest_d = norm(dest_point_px - source_point_px);
        double farthest_d = norm(the_farthest_point_px - source_point_px);
        if (farthest_d > dest_d + min_space_to_obstacle)
        {
            the_farthest_point_px = dest_point_px;
        }
        else if (farthest_d <= min_space_to_obstacle)
        {
            if (dest_d < farthest_d)
            {
                the_farthest_point_px = dest_point_px;
            }
        }
        else
        {
            double alpha = min_space_to_obstacle;
            double beta = farthest_d - min_space_to_obstacle;
            the_farthest_point_px = (the_farthest_point_px * beta + source_point_px * alpha) / (alpha + beta);
        }
    }

    the_farthest_point = cvtMaptoRobotcoordinate(the_farthest_point_px);
    return the_farthest_point;
}


bool DGRobot::makeSubgoal13(Pose2 &pub_pose) // makeSubgoal12 with code revision
{
    //ROS_INFO("[makeSubgoal13] CurGuideIdx %d", m_guider.getCurGuideIdx());
    //ROS_INFO("[makeSubgoal13] m_prev_robot_pose: <%f, %f>", m_prev_robot_pose.x, m_prev_robot_pose.y);
    //ROS_INFO("[makeSubgoal13] m_cur_robot_pose: <%f, %f>", m_cur_robot_pose.x, m_cur_robot_pose.y);

    // current pose in DeepGuider coordinate
    Pose2 dg_pose = m_localizer->getPose();

    // current nodes in DeepGuider coordinate
    m_guider_mutex.lock();
    Pose2 robot_pose = m_guider.m_robot_pose;     // robot pose in robot's coordinate
    GuidanceManager::ExtendedPathElement prev_guide = m_guider.getPrevExtendedPath();
    GuidanceManager::ExtendedPathElement cur_guide = m_guider.getCurExtendedPath();
    GuidanceManager::ExtendedPathElement next_guide = m_guider.getNextExtendedPath();
    GuidanceManager::ExtendedPathElement next_next_guide = m_guider.getNextNextExtendedPath();
    m_guider_mutex.unlock();

    Pose2 prev_node_dg = Point2(prev_guide);
    Pose2 cur_node_dg = Point2(cur_guide);
    Pose2 next_node_dg = Point2(next_guide);
    Pose2 next_next_node_dg = Point2(next_next_guide);

    // load robot info
    m_robotmap_mutex.lock();
    cv::Mat robotmap = m_robotmap_image.clone(); // <map image>
    m_robotmap_mutex.unlock();

    bool use_onlinemap = true;
    if (robotmap.empty()) // on robot occumap
    {
        ROS_INFO("No occumap");
        
        robotmap = m_offline_robotmap.clone(); // if no online map, use offline map
        use_onlinemap = false;
    }
    if (robotmap.empty())
    {
        ROS_INFO("No robotmap");
        return false;
    }
    cv::Size robotmap_size = robotmap.size();
    m_robot_origin.x = -m_dx_map_origin_pixel.x * m_dx_map_meter_per_pixel;
    m_robot_origin.y = -m_dx_map_origin_pixel.y * m_dx_map_meter_per_pixel;

    // dilate robotmap  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 1)
    cv::Mat robotmap_dilate;
    int dilate_value = 2;
    dilate(robotmap, robotmap_dilate, cv::Mat::ones(cv::Size(dilate_value, dilate_value), CV_8UC1), cv::Point(-1, -1), 1);

    // based on the undrivable points, make the robotmap undrivable on those points  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 2)
    for (int i = 0; i < m_undrivable_points.size(); i++)
    {
        Point2 node_robot = m_undrivable_points[i];
        Point2 node_pixel = cvtRobottoMapcoordinate(node_robot);
        robotmap_dilate.at<uchar>((int)node_pixel.y, (int)node_pixel.x) = 0;
    }

    // // erode robotmap  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 3)
    int erode_value = 5 + dilate_value;
    cv::Mat robotmap_erode;
    erode(robotmap_dilate, robotmap_erode, cv::Mat::ones(cv::Size(erode_value, erode_value), CV_8UC1), cv::Point(-1, -1), 1);

    // save image
    if (m_save_guidance_image)
    {
        imwrite("../../../makesubgoal13_robotmap.png", robotmap);
        imwrite("../../../makesubgoal13_robotmap_dilate.png", robotmap_dilate);
        imwrite("../../../makesubgoal13_robotmap_erode.png", robotmap_erode);
    }
    cv::Mat colormap;
    cv::Mat clean_colormap;
    cv::cvtColor(robotmap_erode, colormap, cv::COLOR_GRAY2BGR);
    cv::cvtColor(robotmap, clean_colormap, cv::COLOR_GRAY2BGR);

    // convert variables with DG coordinate to DX coordinate
    Pose2 dg_prev_node_robot = cvtDg2Dx(prev_node_dg);
    Pose2 dg_cur_node_robot = cvtDg2Dx(cur_node_dg);
    Pose2 dg_next_node_robot = cvtDg2Dx(next_node_dg);
    Pose2 dg_next_next_node_robot = cvtDg2Dx(next_next_node_dg);

    // ARRIVED  (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 0)
    bool is_final = (dg_next_next_node_robot.x == dg_next_node_robot.x && dg_next_next_node_robot.y == dg_next_node_robot.y);
    double dist_robot_to_next = norm(dg_next_node_robot - robot_pose);
    //ROS_INFO("[makeSubgoal13] dist robot to next: %f", dist_robot_to_next);
    if ( is_final && dist_robot_to_next < 2)
    {
        ROS_INFO("[makeSubgoal13] ARRIVED. use previous published pub_pose.");
        pub_pose = m_guider.m_subgoal_pose;
        return true;
    }

    dg::Pose2 target_node_dx;

    // here, select new node goal (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 4)
    if (m_cur_head_node_id != next_guide.cur_node_id) // new dg_next_node_robot
    {
        m_prev_node_dx = dg_prev_node_robot;
        m_cur_node_dx = dg_cur_node_robot;
        m_next_node_dx = dg_next_node_robot;
        m_next_next_node_dx = dg_next_next_node_robot;

        if (m_prev_node_id != 0) // NOT initial start (when m_prev_node_robot is not 0 anymore)
        {
            ROS_INFO("[makeSubgoal13] prev_id: %zd, cur_id: %zd, next_id: %zd", m_prev_node_id, m_cur_head_node_id, next_guide.cur_node_id);

            // is the new updated node problematic?? (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 5)
            Pose2 farthest_point_to_m_next_node_dx = getFarthestPoint(robotmap_erode, m_next_node_dx, robot_pose);
            Pose2 farthest_point_to_m_cur_node_dx = getFarthestPoint(robotmap_erode, m_cur_node_dx, robot_pose);

            double drivable_dist_robot_to_nextnode = norm(farthest_point_to_m_next_node_dx - robot_pose);
            double drivable_dist_robot_to_curnode = norm(farthest_point_to_m_cur_node_dx - robot_pose);

            if (drivable_dist_robot_to_nextnode >= drivable_dist_robot_to_curnode)
            {                                          // not problematic. Use default: next
                target_node_dx = m_next_node_dx;       // already shifted
                m_nodeupdated_but_problematic = false; // maybe not needed but just in case... just make it false again
            }
            else
            {                                   // problematic. Use current
                target_node_dx = m_cur_node_dx; // already shifted
                m_nodeupdated_but_problematic = true;
            }
        }
        else // initial start (assume: not shifted at all and not problematic to go to next node)
        {
            target_node_dx = m_next_node_dx;       // target as default. To next node
            m_nodeupdated_but_problematic = false; // maybe not needed but just in case... just make it false again
        }

        // update global variable
        m_prev_node_id = m_cur_head_node_id;
        m_cur_head_node_id = next_guide.cur_node_id;
        m_prev_robot_pose = m_cur_robot_pose;
        m_cur_robot_pose = robot_pose;

        m_guider.m_robot_heading_node_pose = dg_next_node_robot;
    }
    else
    { // if not new node
        if (m_nodeupdated_but_problematic)
        {
            Pose2 farthest_point_to_m_next_node_dx = getFarthestPoint(robotmap_erode, m_next_node_dx, robot_pose);
            Pose2 farthest_point_to_m_cur_node_dx = getFarthestPoint(robotmap_erode, m_cur_node_dx, robot_pose);

            double drivable_dist_robot_to_nextnode = norm(farthest_point_to_m_next_node_dx - robot_pose);
            double drivable_dist_robot_to_curnode = norm(farthest_point_to_m_cur_node_dx - robot_pose);

            if (drivable_dist_robot_to_nextnode >= drivable_dist_robot_to_curnode)
            { // no more problematic (drivable distance to next node > drivable distance to cur node)
                m_nodeupdated_but_problematic = false;
                target_node_dx = m_next_node_dx; // already shifted
            }
            else
            {                                   // still problematic
                target_node_dx = m_cur_node_dx; // already shifted
            }
        }
        else
        {                                    // by default, go to the shifted next node
            target_node_dx = m_next_node_dx; // already shifted
        }
    }

    // check distance to target node & skip too close target
    bool next_node_jumped = false;
    if (!is_final && m_auto_jump_tooclose_target)
    {
        Point2 v = m_next_node_dx - m_cur_node_dx;
        Point2 s = target_node_dx - v;
        Point2 rs = robot_pose - s;
        double projected_rd = rs.ddot(v) / norm(v);
        if(projected_rd > norm(v) - 2.0)
        {
            target_node_dx = m_next_next_node_dx;
            next_node_jumped = true;
        }
    }

    // visualize prev, cur, next dg nodes
    Point2 dg_prev_node_robot_px = cvtRobottoMapcoordinate(dg_prev_node_robot);           // red star
    Point2 dg_cur_node_robot_px = cvtRobottoMapcoordinate(dg_cur_node_robot);             // green star
    Point2 dg_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_node_robot);           // blue star
    Point2 dg_next_next_node_robot_px = cvtRobottoMapcoordinate(dg_next_next_node_robot); // cyan star
    int markerType = cv::MARKER_STAR;
    int markerSize = 20;
    int markerThickness = 4;
    cv::drawMarker(colormap, dg_prev_node_robot_px, cv::Vec3b(128, 128, 128), markerType, markerSize, markerThickness);
    cv::drawMarker(colormap, dg_cur_node_robot_px, cv::Vec3b(0, 255, 0), markerType, markerSize, markerThickness);
    cv::drawMarker(colormap, dg_next_node_robot_px, cv::Vec3b(255, 0, 0), markerType, markerSize, markerThickness);
    cv::drawMarker(colormap, dg_next_next_node_robot_px, cv::Vec3b(255, 255, 0), markerType, markerSize, markerThickness);

    // distance from robot to the (aligned) next node
    double dist_robot_to_targetnode = norm(target_node_dx - robot_pose);
    //ROS_INFO("[makeSubgoal13] dist_robot_to_targetnode: %f", dist_robot_to_targetnode);

    // // new_theta for robot pose (facing the next node). Note: probably not used as for subgoal, we only need coordinate.
    // new_theta = robot_pose.theta + cx::cvtDeg2Rad(diff_deg_robot);  // current pose + how much to turn based on diff_deg_robot

    // compute safe margin in pixels
    double safe_pixel_margin = m_min_subgoalspace_to_obstacle * 10;
    if (dist_robot_to_targetnode < 3)
    {
        safe_pixel_margin = safe_pixel_margin * dist_robot_to_targetnode / 3.0;
    }

    // if not too close to the next node
    // get a point between robot pose and target_node_dx. 1 meter from the robot position to the direction of next_node_robot
    double min_subgoal_distance = 2.0; // in meter. Acceptable distance
    Pose2 optimal_pub_pose;
    optimal_pub_pose = getFarthestPointSafe(robotmap_erode, target_node_dx, robot_pose, safe_pixel_margin); // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 6a)
    double dist_optimalpubpose_robot = norm(robot_pose - optimal_pub_pose);

    if (dist_optimalpubpose_robot > min_subgoal_distance)
    {
        pub_pose.x = optimal_pub_pose.x;
        pub_pose.y = optimal_pub_pose.y;
    }
    else
    {                                                                                        // both dist_optimalpubpose_robot and dist_notsooptimalpubpose_robot are <= sub_goal_distance
        std::vector<double> alternative_sub_goal_distances = {7.0, 6.0, 5.0, 4.0, 3.0, 2.0}; // {3.0} for visualization
        std::vector<int> num_alternatives = {128, 128, 128, 128, 96, 64};                    // {4} for visualization
        double angle_range = 90;                                                             // 180;  // max 180 (whole circle)
        bool isAlternativeFound;
        double min_dist_temp = 1000000; // 100 km

        /////////////////////////////////////////////////////////////////////////////////////////////
        Pose2 pub_pose_temp;
        double min_dist_best = 1000000; // 100 km

        // (GOOGLE DOCS - Subgoal Coordinate Calculation - Main Algorithm - STEP 7)
        for (int i = 0; i < num_alternatives.size(); i++)
        {
            double min_dist_temp;
            isAlternativeFound = findAlternativePathv2(pub_pose_temp, robot_pose, target_node_dx, robotmap_erode, alternative_sub_goal_distances.at(i), dist_robot_to_targetnode, num_alternatives.at(i), colormap, angle_range, min_dist_temp);
            if (isAlternativeFound && min_dist_temp < min_dist_best)
            {
                pub_pose = pub_pose_temp;
                min_dist_best = min_dist_temp;
            }
        }
        if (min_dist_best == 1000000)
        {
            isAlternativeFound = false;
        }
        else
        {
            isAlternativeFound = true;
        }
        // /////////////////////////////////////////////////////////////////////////////////////////////

        if (!isAlternativeFound)
        {
            ROS_INFO("can't find sub goal :(");
            if (m_save_guidance_video)
            {
                Pose2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose);
                cv::putText(colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255), 5);
                cv::putText(clean_colormap, "FAIL", cv::Point(robot_pose_px.x + 50, robot_pose_px.y + 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Vec3b(0, 0, 255), 5);
                record2Video(colormap, clean_colormap, robot_pose);
            }
            return false; // can't find alternative :(
        }
    }

    ////////////////////////////////////////////////////////////
    // CALCULATE THETA (GOOGLE DOCS - Subgoal Theta Calculation)

    double theta_to_next_next = cx::cvtDeg2Rad(findRobotCoordAngleofVectorSource2Dest(m_next_node_dx, m_next_next_node_dx));
    double theta_to_next = cx::cvtDeg2Rad(findRobotCoordAngleofVectorSource2Dest(m_cur_node_dx, m_next_node_dx));
    double dist_pubpose_to_targetnode = norm(target_node_dx - pub_pose);                                     // (GOOGLE DOCS - Subgoal Theta Calculation - STEP 1)
    pub_pose.theta = theta_to_next;
    if (next_node_jumped)
    {
        pub_pose.theta = theta_to_next_next;
    }
    else if (dist_pubpose_to_targetnode < 2)
    {
        // compute angle average by using unit circle points 
        double ratio_for_theta = std::min(dist_pubpose_to_targetnode / 2, 1.0);
        double avg_y = ratio_for_theta * sin(theta_to_next) + (1 - ratio_for_theta) * sin(theta_to_next_next);
        double avg_x = ratio_for_theta * cos(theta_to_next) + (1 - ratio_for_theta) * cos(theta_to_next_next);
        pub_pose.theta = atan2(avg_y, avg_x);
    }

    printf("[THETA] jumped=%d, dist=%.2lf, next=%.1lf, nnext=%.1lf, theta=%.1lf\n", next_node_jumped, dist_pubpose_to_targetnode, cx::cvtRad2Deg(theta_to_next), cx::cvtRad2Deg(theta_to_next_next), cx::cvtRad2Deg(pub_pose.theta));

    // CALCULATE THETA END
    ////////////////////////////////////////////////////////////

    // draw subgoal
    ROS_INFO("Found subgoal: <%f, %f, %f>", pub_pose.x, pub_pose.y, cx::cvtRad2Deg(pub_pose.theta)); // OUTPUT.. care about pub_pose in robot's coordinate
    Pose2 pub_pose_px = cvtRobottoMapcoordinate(pub_pose);
    cv::circle(colormap, pub_pose_px, 20, cv::Vec3b(255, 0, 255), 5);      // small purple circle
    cv::circle(colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), -1);       // with robot real size
    cv::circle(clean_colormap, pub_pose_px, 5, cv::Vec3b(255, 0, 255), 2); // with robot real size
    Pose2 pubpose_heading;
    pubpose_heading.x = pub_pose_px.x + 20 * cos(pub_pose.theta);
    pubpose_heading.y = pub_pose_px.y + 20 * sin(pub_pose.theta);
    cv::line(colormap, pub_pose_px, pubpose_heading, cv::Vec3b(255, 0, 255), 5);
    pubpose_heading.x = pub_pose_px.x + 5 * cos(pub_pose.theta);
    pubpose_heading.y = pub_pose_px.y + 5 * sin(pub_pose.theta);
    cv::line(clean_colormap, pub_pose_px, pubpose_heading, cv::Vec3b(255, 0, 255), 2);

    // draw deepguider pose
    if (m_show_dg_pose)
    {
        Point2 dg_xy = cvtDg2Dx(dg_pose);
        Point2 dg_px = cvtRobottoMapcoordinate(dg_xy);
        cv::circle(colormap, dg_px, 10, cv::Vec3b(0, 0, 255), -1);
    }

    // draw robot
    Point2 robot_pose_px = cvtRobottoMapcoordinate(robot_pose); // robot_pose_px = dg_pose_robot_px
    cv::circle(colormap, robot_pose_px, 20, cv::Vec3b(0, 200, 0), 4);
    Point2 robot_heading;
    robot_heading.x = robot_pose_px.x + 20 * cos(robot_pose.theta);
    robot_heading.y = robot_pose_px.y + 20 * sin(robot_pose.theta);
    cv::line(colormap, robot_pose_px, robot_heading, cv::Vec3b(0, 200, 0), 4);

    cv::Mat crop_flip;
    cv::Rect roi_rc(robot_pose_px.x - m_crop_radius, robot_pose_px.y - m_crop_radius, 2 * m_crop_radius + 1, 2 * m_crop_radius + 1);
    roi_rc = roi_rc & cv::Rect(0, 0, colormap.cols, colormap.rows);
    cv::flip(colormap(roi_rc), crop_flip, 0);

    // regend annotations
    cv::Point px(crop_flip.cols - 130, 20);
    int dy = 25;
    cv::putText(crop_flip, "O subgoal", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    //cv::putText(crop_flip, "x subA", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    //cv::putText(crop_flip, "o subB", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    //cv::putText(crop_flip, "[] target", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 255), 1); px.y+=dy;
    cv::putText(crop_flip, "* Nprev", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(128, 128, 128), 1); px.y+=dy;
    cv::putText(crop_flip, "* Ncur", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(0, 255, 0), 1); px.y+=dy;
    cv::putText(crop_flip, "* Nnext", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 0, 0), 1); px.y+=dy;
    cv::putText(crop_flip, "* Nnnext", px, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Vec3b(255, 255, 0), 1); px.y+=dy;

    cv::imshow("guidance_map", crop_flip);
    cv::waitKey(1);

    if (m_save_guidance_video) record2VideoCropped(crop_flip);

    // // Plan B-5
    // // make current pose undrivable. To prevent repeated position
    // m_undrivable_points.push_back(robot_pose);

    return true;
}

// The main function
int main(int argc, char **argv)
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
