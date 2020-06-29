#ifndef __TEST_LOCALIZER_SIMPLE__
#define __TEST_LOCALIZER_SIMPLE__

#include "vvs.h"
#include "dg_core.hpp"
#include "dg_localizer.hpp"

int testLocBaseDist2()
{
    dg::Point2 from(1, 1), to(2, 2);
    std::pair<double, dg::Point2> result;

    result = dg::BaseLocalizer::calcDist2FromLineSeg(from, to, dg::Pose2(1, 2, 0));
    VVS_CHECK_NEAR(result.first, 0.5);
    VVS_CHECK_NEAR(result.second.x, 1.5);
    VVS_CHECK_NEAR(result.second.y, 1.5);

    result = dg::BaseLocalizer::calcDist2FromLineSeg(from, to, dg::Pose2(1, 1, 0));
    VVS_CHECK_NEAR(result.first, 0);
    VVS_CHECK_NEAR(result.second.x, 1);
    VVS_CHECK_NEAR(result.second.y, 1);

    result = dg::BaseLocalizer::calcDist2FromLineSeg(from, to, dg::Pose2(2, 2, 0));
    VVS_CHECK_NEAR(result.first, 0);
    VVS_CHECK_NEAR(result.second.x, 2);
    VVS_CHECK_NEAR(result.second.y, 2);

    result = dg::BaseLocalizer::calcDist2FromLineSeg(from, to, dg::Pose2(0, 0, 0));
    VVS_CHECK_NEAR(result.first, 2);
    VVS_CHECK_NEAR(result.second.x, 1);
    VVS_CHECK_NEAR(result.second.y, 1);

    result = dg::BaseLocalizer::calcDist2FromLineSeg(from, to, dg::Pose2(3, 3, 0));
    VVS_CHECK_NEAR(result.first, 2);
    VVS_CHECK_NEAR(result.second.x, 2);
    VVS_CHECK_NEAR(result.second.y, 2);

    result = dg::BaseLocalizer::calcDist2FromLineSeg(from, to, dg::Pose2(1, 2, 0), 3);
    VVS_CHECK_NEAR(result.first, 0.5 + 3 * CV_PI * CV_PI / 16);
    VVS_CHECK_NEAR(result.second.x, 1.5);
    VVS_CHECK_NEAR(result.second.y, 1.5);

    return 0;
}

dg::RoadMap getSimpleRoadMap()
{
    // An example road map ('+' represents direction of edges)
    // 2 --+ 3 +-+ 5 +-- 6
    // +     |     +     |
    // |     +     |     +
    // 1 +-- 4     7 +-- 8

    dg::RoadMap map;
    map.addNode(dg::Point2ID(1, 0, 0)); // ID, x, y
    map.addNode(dg::Point2ID(2, 0, 1));
    map.addNode(dg::Point2ID(3, 1, 1));
    map.addNode(dg::Point2ID(4, 1, 0));
    map.addNode(dg::Point2ID(5, 2, 1));
    map.addNode(dg::Point2ID(6, 3, 1));
    map.addNode(dg::Point2ID(7, 2, 0));
    map.addNode(dg::Point2ID(8, 3, 0));
    map.addEdge(1, 2);
    map.addEdge(2, 3);
    map.addEdge(3, 4);
    map.addEdge(4, 1);
    map.addRoad(3, 5); // Add a bi-directional edge
    map.addEdge(6, 5);
    map.addEdge(6, 8);
    map.addEdge(7, 5);
    map.addEdge(8, 7);
    return map;
}

int testLocBaseNearest()
{
    dg::SimpleLocalizer localizer;
    dg::RoadMap map = getSimpleRoadMap();
    VVS_CHECK_TRUE(localizer.loadMap(map));

    dg::Pose2 pose_m = localizer.cvtTopmetric2Metric(dg::TopometricPose(1, 0, 0.5, -CV_PI / 2));
    VVS_CHECK_NEAR(pose_m.x, 0);
    VVS_CHECK_NEAR(pose_m.y, 0.5);
    VVS_CHECK_NEAR(pose_m.theta, 0);

    dg::TopometricPose pose_t1 = localizer.findNearestTopoPose(pose_m);
    VVS_CHECK_EQUL(pose_t1.node_id, 1);
    VVS_CHECK_EQUL(pose_t1.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t1.dist, 0.5);
    VVS_CHECK_NEAR(pose_t1.head, -CV_PI / 2);

    dg::TopometricPose pose_t2 = localizer.findNearestTopoPose(pose_m, 10);
    VVS_CHECK_EQUL(pose_t2.node_id, 2);
    VVS_CHECK_EQUL(pose_t2.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t2.dist, 0);
    VVS_CHECK_NEAR(pose_t2.head, 0);

    dg::TopometricPose pose_t3 = localizer.findNearestTopoPose(dg::Pose2(1.5, 1.5, CV_PI / 4), 1);
    VVS_CHECK_EQUL(pose_t3.node_id, 3);
    VVS_CHECK_EQUL(pose_t3.edge_idx, 1);
    VVS_CHECK_NEAR(pose_t3.dist, 0.5);
    VVS_CHECK_NEAR(pose_t3.head, CV_PI / 4);

    dg::TopometricPose pose_t4 = localizer.findNearestTopoPose(dg::Pose2(1.5, 1.5, CV_PI * 3 / 4), 1);
    VVS_CHECK_EQUL(pose_t4.node_id, 5);
    VVS_CHECK_EQUL(pose_t4.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t4.dist, 0.5);
    VVS_CHECK_NEAR(pose_t4.head, -CV_PI / 4);

    return 0;
}

int testLocBaseTrack()
{
    dg::SimpleLocalizer localizer;
    dg::RoadMap map = getSimpleRoadMap();
    VVS_CHECK_TRUE(localizer.loadMap(map));

    dg::TopometricPose pose_t1 = localizer.trackTopoPose(dg::TopometricPose(1, 0, 0, 0), dg::Pose2(0, 0.5, CV_PI / 2));
    VVS_CHECK_EQUL(pose_t1.node_id, 1);
    VVS_CHECK_EQUL(pose_t1.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t1.dist, 0.5);
    VVS_CHECK_NEAR(pose_t1.head, 0);

    dg::TopometricPose pose_t2 = localizer.trackTopoPose(dg::TopometricPose(1, 0, 0, 0), dg::Pose2(0, 1.0, CV_PI / 2));
    VVS_CHECK_EQUL(pose_t2.node_id, 1);
    VVS_CHECK_EQUL(pose_t2.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t2.dist, 1.0);
    VVS_CHECK_NEAR(pose_t2.head, 0);

    dg::TopometricPose pose_t3 = localizer.trackTopoPose(dg::TopometricPose(1, 0, 0, 0), dg::Pose2(0, 1.0, 0), 1);
    VVS_CHECK_EQUL(pose_t3.node_id, 2);
    VVS_CHECK_EQUL(pose_t3.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t3.dist, 0);
    VVS_CHECK_NEAR(pose_t3.head, 0);

    dg::TopometricPose pose_t4 = localizer.trackTopoPose(dg::TopometricPose(3, 1, 0.1, 0), dg::Pose2(1, 1, 0), 1);
    VVS_CHECK_EQUL(pose_t4.node_id, 3);
    VVS_CHECK_EQUL(pose_t4.edge_idx, 1);
    VVS_CHECK_NEAR(pose_t4.dist, 0);
    VVS_CHECK_NEAR(pose_t4.head, 0);

    dg::TopometricPose pose_t5 = localizer.trackTopoPose(dg::TopometricPose(3, 1, 0.1, 0), dg::Pose2(1.5, 1.5, CV_PI), 1);
    VVS_CHECK_EQUL(pose_t5.node_id, 5);
    VVS_CHECK_EQUL(pose_t5.edge_idx, 0);
    VVS_CHECK_NEAR(pose_t5.dist, 0.5);
    VVS_CHECK_NEAR(pose_t5.head, 0);

    dg::TopometricPose pose_t6 = localizer.trackTopoPose(dg::TopometricPose(3, 1, 0.1, 0), dg::Pose2(1.5, 1.5, CV_PI), 1, false);
    VVS_CHECK_EQUL(pose_t6.node_id, 3);
    VVS_CHECK_EQUL(pose_t6.edge_idx, 1);
    VVS_CHECK_NEAR(pose_t6.dist, 0.5);
    VVS_CHECK_NEAR(pose_t6.head, -CV_PI);

    return 0;
}

std::vector<std::pair<std::string, cv::Vec3d>> getSimpleDataset()
{
    std::vector<std::pair<std::string, cv::Vec3d>> dataset =
    {
        std::make_pair("Pose",      cv::Vec3d(0.1, 0.1, cx::cvtDeg2Rad(95))),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("LocClue",   cv::Vec3d(2, -1, CV_PI)),

        std::make_pair("Odometry",  cv::Vec3d(0, cx::cvtDeg2Rad(-95))),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("Odometry",  cv::Vec3d(0.1, 0)),
        std::make_pair("LocClue",   cv::Vec3d(3, -1, CV_PI)),
    };
    return dataset;
}

int testLocSimple(int wait_msec = 1)
{
    // Load a map
    dg::SimpleLocalizer localizer;
    dg::RoadMap map = getSimpleRoadMap();
    VVS_CHECK_TRUE(localizer.loadMap(map));

    // Prepare visualization
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 200));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 2 * 0.5));
    cv::Mat map_image;
    VVS_CHECK_TRUE(painter.drawMap(map_image, map));
    dg::CanvasInfo map_info = painter.getCanvasInfo(map, map_image.size());

    // Run localization
    auto dataset = getSimpleDataset();
    VVS_CHECK_TRUE(!dataset.empty());
    for (size_t t = 0; t < dataset.size(); t++)
    {
        const dg::Timestamp time = static_cast<dg::Timestamp>(t);
        const cv::Vec3d& d = dataset[t].second;
        if (dataset[t].first == "Pose")        VVS_CHECK_TRUE(localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), time));
        if (dataset[t].first == "Position")    VVS_CHECK_TRUE(localizer.applyPosition(dg::Point2(d[0], d[1]), time));
        if (dataset[t].first == "Orientation") VVS_CHECK_TRUE(localizer.applyOrientation(d[0], time));
        if (dataset[t].first == "Odometry")    VVS_CHECK_TRUE(localizer.applyOdometry(dg::Polar2(d[0], d[1]), time));
        if (dataset[t].first == "LocClue")     VVS_CHECK_TRUE(localizer.applyLocClue(int(d[0]), dg::Polar2(d[1], d[2]), time));

        if (wait_msec >= 0)
        {
            cv::Mat image = map_image.clone();
            dg::TopometricPose pose_t = localizer.getPoseTopometric();
            dg::Pose2 pose_m = localizer.getPose();
            VVS_CHECK_TRUE(painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 0.1, 0, cx::COLOR_MAGENTA));
            cv::String info_topo = cv::format("Node ID: %d, Edge Idx: %d, Dist: %.3f", pose_t.node_id, pose_t.edge_idx, pose_t.dist);
            cv::putText(image, info_topo, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cx::COLOR_MAGENTA);

            cv::imshow("testLocSimpleTest", image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_ESC) return -1;
        }
    }

    return 0;
}

#endif // End of '__TEST_LOCALIZER_SIMPLE__'
