#include "dg_core.hpp"
#include "dg_localizer.hpp"

dg::SimpleRoadMap getExampleMap()
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
    map.addEdge(dg::Point2ID(1), dg::Point2ID(2));
    map.addEdge(dg::Point2ID(2), dg::Point2ID(3));
    map.addEdge(dg::Point2ID(3), dg::Point2ID(4));
    map.addEdge(dg::Point2ID(4), dg::Point2ID(1));
    map.addRoad(dg::Point2ID(3), dg::Point2ID(5)); // Add a bi-directional edge
    map.addEdge(dg::Point2ID(6), dg::Point2ID(5));
    map.addEdge(dg::Point2ID(6), dg::Point2ID(8));
    map.addEdge(dg::Point2ID(7), dg::Point2ID(5));
    map.addEdge(dg::Point2ID(8), dg::Point2ID(7));
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

int main()
{
    // Load a map
    dg::SimpleRoadMap map = getExampleMap();
    dg::SimpleMetricLocalizer localizer;
    if (!localizer.loadMap(map)) return -1;

    // Prepare visualization
    dg::MapPainter painter;
    if (!painter.setParamValue("pixel_per_meter", 200)) return -1;
    if (!painter.setParamValue("node_font_scale", 2 * 0.5)) return -1;
    dg::CanvasInfo map_info = painter.getCanvasInfo(map);
    cv::Mat map_image;
    if (!painter.drawMap(map_image, map)) return -1;

    // Run localization
    auto dataset = getExampleDataset();
    for (size_t t = 0; t < dataset.size(); t++)
    {
        const cv::Vec3d& d = dataset[t].second;
        if (dataset[t].first == "Pose")     localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), t);
        if (dataset[t].first == "Odometry") localizer.applyOdometry(dg::Polar2(d[0], d[1]), t);
        if (dataset[t].first == "LocClue")  localizer.applyLocClue(int(d[0]), dg::Polar2(d[1], d[2]), t);

        cv::Mat image = map_image.clone();
        dg::Pose2 pose = localizer.getPose();
        painter.drawNode(image, map_info, dg::Point2ID(0, pose.x, pose.y), 0.1, 0, cx::COLOR_MAGENTA);

        cv::imshow("Simple Test", image);
        int key = cv::waitKeyEx();
        if (key == cx::KEY_ESC) return -1;
    }

    return 0;
}