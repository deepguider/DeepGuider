#ifndef __TEST_SIMPLE_LOCALIZER__
#define __TEST_SIMPLE_LOCALIZER__

#include "vvs.h"
#include "dg_core.hpp"
#include "dg_localizer.hpp"

dg::SimpleRoadMap getExampleSimpleRoadMap()
{
    // An example road map ('+' represents direction of edges)
    // 2 --+ 3 +-+ 5 +-- 6
    // +     |     +     |
    // |     +     |     +
    // 1 +-- 4     7 +-- 8

    dg::SimpleRoadMap map;
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

dg::Map getExampleMap()
{
    // An example map
    // 2 --- 3 --- 5 --- 6
    // |     |     |     |
    // |     |     |     |
    // 1 --- 4     7 --- 8

    dg::Map map;
    map.addNode(dg::NodeInfo(1, 0, 0)); // ID, x, y
    map.addNode(dg::NodeInfo(2, 0, 1));
    map.addNode(dg::NodeInfo(3, 1, 1));
    map.addNode(dg::NodeInfo(4, 1, 0));
    map.addNode(dg::NodeInfo(5, 2, 1));
    map.addNode(dg::NodeInfo(6, 3, 1));
    map.addNode(dg::NodeInfo(7, 2, 0));
    map.addNode(dg::NodeInfo(8, 3, 0));
    map.addRoad(1, 2); // ID_1, ID_2
    map.addRoad(1, 4);
    map.addRoad(2, 3);
    map.addRoad(3, 4);
    map.addRoad(3, 5);
    map.addRoad(5, 6);
    map.addRoad(5, 7);
    map.addRoad(6, 8);
    map.addRoad(7, 8);
    return map;
}

std::vector<std::pair<std::string, cv::Vec3d>> getExampleDataset()
{
    std::vector<std::pair<std::string, cv::Vec3d>> dataset =
    {
        std::make_pair("Pose",      cv::Vec3d(0, 0, cx::cvtDeg2Rad(95))),
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

int testLocSimpleMetricLocalizer(int wait_msec = 1)
{
    // Load a map
    dg::SimpleMetricLocalizer localizer;
    dg::Map map = getExampleMap();
    VVS_CHECK_TRUE(localizer.loadMap(map));

    // Prepare visualization
    dg::MapPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 200));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 2 * 0.5));
    dg::CanvasInfo map_info = painter.getCanvasInfo(map);
    cv::Mat map_image;
    VVS_CHECK_TRUE(painter.drawMap(map_image, map));

    // Run localization
    auto dataset = getExampleDataset();
    for (size_t t = 0; t < dataset.size(); t++)
    {
        const cv::Vec3d& d = dataset[t].second;
        if (dataset[t].first == "Pose")     VVS_CHECK_TRUE(localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), t));
        if (dataset[t].first == "Odometry") VVS_CHECK_TRUE(localizer.applyOdometry(dg::Polar2(d[0], d[1]), t));
        if (dataset[t].first == "LocClue")  VVS_CHECK_TRUE(localizer.applyLocClue(int(d[0]), dg::Polar2(d[1], d[2]), t));

        if (wait_msec >= 0)
        {
            cv::Mat image = map_image.clone();
            dg::Pose2 pose = localizer.getPose();
            VVS_CHECK_TRUE(painter.drawNode(image, map_info, dg::Point2ID(0, pose.x, pose.y), 0.1, 0, cx::COLOR_MAGENTA));

            cv::imshow("testLocSimpleMetricLocalizer", image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_ESC) return -1;
        }
    }

    return 0;
}

#endif // End of '__TEST_SIMPLE_LOCALIZER__'
