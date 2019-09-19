#include "dg_core.hpp"
#include "dg_localizer.hpp"
//#include "simple_map_manager_jsh.h"

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
    map.addEdge(dg::Point2ID(1), dg::Point2ID(2)); // Node#1 -> Node#2
    map.addEdge(dg::Point2ID(2), dg::Point2ID(3));
    map.addEdge(dg::Point2ID(3), dg::Point2ID(4));
    map.addEdge(dg::Point2ID(4), dg::Point2ID(1));
    map.addRoad(dg::Point2ID(3), dg::Point2ID(5)); // Add a bi-directional edge
    map.addEdge(dg::Point2ID(5), dg::Point2ID(6));
    map.addEdge(dg::Point2ID(6), dg::Point2ID(8));
    map.addEdge(dg::Point2ID(5), dg::Point2ID(7));
    map.addEdge(dg::Point2ID(7), dg::Point2ID(8));
    return map;
}

std::vector<std::pair<std::string, cv::Vec3d>> getExampleDataset()
{
    std::vector<std::pair<std::string, cv::Vec3d>> dataset =
    {
        std::make_pair("Pose",      cv::Vec3d(0, 0, cx::cvtDeg2Rad(95))),

        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.1)),  // RefNode#, Edge#(RefNode# & NextNode#, Each 4-Digit without left-most 0's), Distance from RefNode
        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.2)),
        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.3)),
        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.4)),
        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.5)),
        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.6)),
        std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.7)),
		std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.8)),
		std::make_pair("Odometry",  cv::Vec3d(1, 10002, 0.9)),
//      std::make_pair("LocClue",   cv::Vec3d(2, -1, CV_PI)),

        std::make_pair("Pose",      cv::Vec3d(0, 1, cx::cvtDeg2Rad(-5))),

		std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.1)),
		std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.2)),
        std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.3)),
        std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.4)),
        std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.5)),
        std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.6)),
		std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.7)),
		std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.8)),
		std::make_pair("Odometry",  cv::Vec3d(2, 20003, 0.9)),
//      std::make_pair("LocClue",   cv::Vec3d(3, -1, CV_PI)),

		std::make_pair("Pose",      cv::Vec3d(1, 1, cx::cvtDeg2Rad(0))),

		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.1)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.2)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.3)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.4)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.5)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.6)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.7)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.8)),
		std::make_pair("Odometry",  cv::Vec3d(3, 30005, 0.9)),
//		std::make_pair("LocClue",   cv::Vec3d(5, -1, CV_PI)),

		std::make_pair("Pose",      cv::Vec3d(2, 1, cx::cvtDeg2Rad(-85))),

		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.1)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.2)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.3)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.4)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.5)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.6)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.7)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.8)),
		std::make_pair("Odometry",  cv::Vec3d(5, 50007, 0.9)),
//		std::make_pair("LocClue",   cv::Vec3d(7, -1, CV_PI)),

		std::make_pair("Pose",      cv::Vec3d(2, 0, cx::cvtDeg2Rad(-5))),

		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.1)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.2)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.3)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.4)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.5)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.6)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.7)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.8)),
		std::make_pair("Odometry",  cv::Vec3d(7, 70008, 0.9)),
//		std::make_pair("LocClue",   cv::Vec3d(8, -1, CV_PI)),

		std::make_pair("Pose",      cv::Vec3d(3, 0, cx::cvtDeg2Rad(90))),

    };
    return dataset;
}

std::vector<dg::NodeInfo> getExamplePath()
{
	std::vector<dg::NodeInfo> path;

	dg::NodeInfo Path1(1, 0, 0, 0, 1);
	dg::NodeInfo Path2(2, 0, 1, 0, 1);
	dg::NodeInfo Path3(3, 1, 1, 1, 1);
	dg::NodeInfo Path4(5, 2, 1, 1, 1);
	dg::NodeInfo Path5(7, 2, 0, 0, 1);
	dg::NodeInfo Path6(8, 3, 0, 0, 1);
	
	path.push_back(Path1);
	path.push_back(Path2);
	path.push_back(Path3);
	path.push_back(Path4);
	path.push_back(Path5);
	path.push_back(Path6);

	return path;
}

int main()
{
    // Load a map
    dg::SimpleRoadMap map = getExampleMap();
    dg::SimpleMetricLocalizer localizer;
    if (!localizer.loadMap(map)) return -1;

	//added by seohyun
	std::vector<dg::NodeInfo> path = getExamplePath();

    // Prepare visualization
    dg::MapPainter painter;
    if (!painter.setParamValue("pixel_per_meter", 200)) return -1;
    if (!painter.setParamValue("node_font_scale", 2 * 0.5)) return -1;
    dg::CanvasInfo map_info = painter.getCanvasInfo(map);
    cv::Mat map_image;
    if (!painter.drawMap(map_image, map)) return -1;

    // Run localization
    auto dataset = getExampleDataset();
	cv::Vec3d preDist = (0, 0);
	cv::Vec3d curDist = (0, 0);

    for (size_t t = 0; t < dataset.size(); t++)
    {
        const cv::Vec3d& d = dataset[t].second;
		
//      if (dataset[t].first == "Pose")     localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), t);
//		if (dataset[t].first == "Odometry") localizer.applyOdometry(dg::Polar2(d[0], d[1]), t);
//      if (dataset[t].first == "LocClue")  localizer.applyLocClue(int(d[0]), dg::Polar2(d[1], d[2]), t);

		if (dataset[t].first == "Pose")
		{
			preDist = (0, 0);
			curDist = (0, 0);
			localizer.applyPose(dg::Pose2(d[0], d[1], d[2]), t);
		}
		if (dataset[t].first == "Odometry")
		{
			// d[0]=RefNode#, d[1]=Edge#(RefNode# & NextNode#), d[2]=Distance from RefNode
			curDist[0] = d[2] - preDist[0];
			preDist[0] = d[2];
			localizer.applyOdometry(dg::Polar2(curDist[0], curDist[1]), t);
		}

        cv::Mat image = map_image.clone();
		
        dg::Pose2 pose = localizer.getPose();
        painter.drawNode(image, map_info, dg::Point2ID(0, pose.x, pose.y), 0.1, 0, cx::COLOR_MAGENTA);

        cv::imshow("Simple Test", image);

//		cv::Size sizeImage = image.size();
//		cv::Point locText = (sizeImage.width / 2, sizeImage.height / 2);
//		cv::putText(image, "Go Straight", locText, 2, 2.0, cv::Scalar(0, 0, 255));

        int key = cv::waitKeyEx();
        if (key == cx::KEY_ESC) return -1;
    }

    return 0;
}