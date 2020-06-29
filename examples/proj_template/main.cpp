#include "dg_core.hpp"
#include "dg_localizer.hpp"

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

std::vector<std::pair<std::string, cv::Vec3d>> getSimpleDataset()
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

int main()
{
    // Load a map
    dg::SimpleLocalizer localizer;
    dg::RoadMap map = getSimpleRoadMap();
    if (!localizer.loadMap(map)) return -1;

    // Prepare visualization
    dg::SimpleRoadPainter painter;
    if (!painter.setParamValue("pixel_per_meter", 200)) return -1;
    if (!painter.setParamValue("node_font_scale", 2 * 0.5)) return -1;
    cv::Mat map_image;
    if (!painter.drawMap(map_image, map)) return -1;
    dg::CanvasInfo map_info = painter.getCanvasInfo(map);

    // Run localization
    auto dataset = getSimpleDataset();
    for (size_t t = 0; t < dataset.size(); t++)
    {
        const dg::Timestamp time = static_cast<dg::Timestamp>(t);
        const cv::Vec3d& d = dataset[t].second;
        if (dataset[t].first == "Pose")        localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), time);
        if (dataset[t].first == "Position")    localizer.applyPosition(dg::Point2(d[0], d[1]), time);
        if (dataset[t].first == "Orientation") localizer.applyOrientation(d[0], time);
        if (dataset[t].first == "Odometry")    localizer.applyOdometry(dg::Polar2(d[0], d[1]), time);
        if (dataset[t].first == "LocClue")     localizer.applyLocClue(int(d[0]), dg::Polar2(d[1], d[2]), time);

        cv::Mat image = map_image.clone();
        dg::TopometricPose pose_t = localizer.getPoseTopometric();
        dg::Pose2 pose_m = localizer.getPose();
        painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 0.1, 0, cx::COLOR_MAGENTA);
        cv::String info_topo = cv::format("Node ID: %d, Edge Idx: %d, Dist: %.3f", pose_t.node_id, pose_t.edge_idx, pose_t.dist);
        cv::putText(image, info_topo, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cx::COLOR_MAGENTA);

        cv::imshow("Project Template", image);
        int key = cv::waitKey();
        if (key == cx::KEY_ESC) return -1;
    }

    return 0;
}