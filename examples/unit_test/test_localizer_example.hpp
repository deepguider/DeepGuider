#ifndef __TEST_LOCALIZER_EXAMPLE__
#define __TEST_LOCALIZER_EXAMPLE__

#include "vvs.h"
#include "dg_core.hpp"
#include "dg_localizer.hpp"

int generateExampleMap(const char* yml_file = "data/NaverMap_ETRI.yml", dg::ID id_offset = 1000, double pixel_per_meter = 1, const dg::LatLon& first_latlon = dg::LatLon(36.3838455, 127.3678567))
{
    // Read 'yml_file' generated from 'simple_picker'
    cv::FileStorage fs;
    try { if (!fs.open(yml_file, cv::FileStorage::READ)) return -1; }
    catch (cv::Exception & e) { return -1; }

    std::vector<std::vector<dg::Point2>> roads;
    cv::FileNode fn_lines = fs.root()["region_data"];
    if (!fn_lines.empty())
    {
        for (cv::FileNodeIterator fn_line = fn_lines.begin(); fn_line != fn_lines.end(); fn_line++)
        {
            std::vector<dg::Point2> pts;
            (*fn_line)["pts"] >> pts;
            if (!pts.empty()) roads.push_back(pts);
        }
    }
    if (roads.empty()) return -2;

    // Print nodes for example map
    printf("dg::Map map;\n");
    dg::UTMConverter converter;
    converter.setReference(first_latlon);
    const dg::Point2& first_node = roads.front().front();
    for (size_t r_idx = 0; r_idx < roads.size(); r_idx++)
    {
        for (size_t n_idx = 0; n_idx < roads[r_idx].size(); n_idx++)
        {
            dg::ID id = id_offset * (r_idx + 1) + n_idx;
            const dg::Point2& node = roads[r_idx][n_idx];
            dg::LatLon ll = converter.toLatLon(dg::Point2((node.x - first_node.x) / pixel_per_meter, (first_node.y - node.y) / pixel_per_meter));
            printf("map.addNode(dg::NodeInfo(%zd, %.12f, %.12f));\n", id, ll.lat, ll.lon);
        }
    }

    // Print nodes for example map
    for (size_t r_idx = 0; r_idx < roads.size(); r_idx++)
    {
        for (size_t n_idx = 1; n_idx < roads[r_idx].size(); n_idx++)
        {
            dg::ID id = id_offset * (r_idx + 1) + n_idx;
            printf("map.addRoad(%zd, %zd);\n", id - 1, id);
        }
    }

    return 0;
}

dg::Map getExampleMap()
{
    // An example map around ETRI
    // - Note: Geodesic data were roughly generated from 'generateExampleMap()' from manually selected nodes on Naver Map.
    //         The geodesic data were generated based on latitude and longitude of the first node. They are not globally accurate, but relatively accurate.
    //         1) The geodesic values of the first node can be acquired by Google Maps. To show the base point, please use the following URL, https://www.google.com/maps/@36.3838276,127.3676621,21z
    //         2) The geodesic values of the first node can be assigned by GPS data such as (36.3838455, 127.3678567).

    dg::Map map;
    map.addNode(dg::NodeInfo(1000, 36.383845499999, 127.367856699998));
    map.addNode(dg::NodeInfo(1001, 36.383980344326, 127.368489335844));
    map.addNode(dg::NodeInfo(1002, 36.384199943244, 127.369387764218));
    map.addNode(dg::NodeInfo(1003, 36.384496533926, 127.369983583772));
    map.addNode(dg::NodeInfo(1004, 36.385040239270, 127.370852962675));
    map.addNode(dg::NodeInfo(1005, 36.385246705794, 127.371450674120));
    map.addNode(dg::NodeInfo(1006, 36.385397530922, 127.372595827237));
    map.addNode(dg::NodeInfo(1007, 36.385624384793, 127.374029264149));
    map.addNode(dg::NodeInfo(1008, 36.385727548719, 127.374985894115));
    map.addNode(dg::NodeInfo(1009, 36.385791483728, 127.377035905513));
    map.addNode(dg::NodeInfo(1010, 36.385789135672, 127.377526492740));
    map.addNode(dg::NodeInfo(1011, 36.385805263210, 127.378050141951));
    map.addNode(dg::NodeInfo(1012, 36.385803516299, 127.378585311240));
    map.addNode(dg::NodeInfo(1013, 36.385574506719, 127.378980267293));
    map.addNode(dg::NodeInfo(1014, 36.384664169211, 127.378988019480));
    map.addNode(dg::NodeInfo(1015, 36.382690294056, 127.379006702550));
    map.addNode(dg::NodeInfo(1016, 36.381356555113, 127.379034390264));
    map.addNode(dg::NodeInfo(1017, 36.378977452428, 127.379083774640));
    map.addRoad(1000, 1001);
    map.addRoad(1001, 1002);
    map.addRoad(1002, 1003);
    map.addRoad(1003, 1004);
    map.addRoad(1004, 1005);
    map.addRoad(1005, 1006);
    map.addRoad(1006, 1007);
    map.addRoad(1007, 1008);
    map.addRoad(1008, 1009);
    map.addRoad(1009, 1010);
    map.addRoad(1010, 1011);
    map.addRoad(1011, 1012);
    map.addRoad(1012, 1013);
    map.addRoad(1013, 1014);
    map.addRoad(1014, 1015);
    map.addRoad(1015, 1016);
    map.addRoad(1016, 1017);
    return map;
}

std::vector<std::pair<double, dg::LatLon>> getExampleGPSData(const char* csv_file = "data/191115_ETRI_asen_fix.csv")
{
    cx::CSVReader csv;
    VVS_CHECK_TRUE(csv.open(csv_file));
    cx::CSVReader::CSVDouble csv_ext = csv.extract(1, { 2, 3, 7, 8 }); // Skip the header

    std::vector<std::pair<double, dg::LatLon>> data;
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        double timestamp = row->at(0) + 1e-9 * row->at(1);
        dg::LatLon ll(row->at(2), row->at(3));
        data.push_back(std::make_pair(timestamp, ll));
    }
    return data;
}

int testLocMap2SimpleRoadMap(int wait_msec = 1, const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png")
{
    // Load a map
    dg::Map map = getExampleMap();
    VVS_CHECK_TRUE(!map.isEmpty());

    // Convert it to 'dg::SimpleRoadMap'
    dg::UTMConverter converter;
    dg::Map::NodeItrConst refer_node = map.getHeadNodeConst(); // Select the first node as the origin
    VVS_CHECK_TRUE(converter.setReference(refer_node->data));
    dg::SimpleRoadMap simple_map = dg::SimpleLocalizer::cvtMap2SimpleRoadMap(map, converter);
    VVS_CHECK_TRUE(!simple_map.isEmpty());

    // Draw the converted map
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 5));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 0, 255 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 100, 0, 0 }));

    cv::Mat map_image = cv::imread(background_file);
    VVS_CHECK_TRUE(map_image.empty() == false);
    VVS_CHECK_TRUE(painter.drawMap(map_image, simple_map));

    if (wait_msec >= 0)
    {
        cv::imshow("testLocMap2SimpleRoadMap", map_image);
        cv::waitKey(wait_msec);
    }

    return 0;
}

int testLocExampleLocalizer(int wait_msec = 1, const char* gps_file = "data/191115_ETRI_asen_fix.csv", const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png")
{
    // Load a map
    dg::SimpleLocalizer localizer;
    dg::Map map = getExampleMap();
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_TRUE(localizer.setReference(map.getHeadNode()->data));
    VVS_CHECK_TRUE(localizer.loadMap(map, true));

    // Prepare visualization
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 5));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 0, 255 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 100, 0, 0 }));

    cv::Mat map_image = cv::imread(background_file);
    VVS_CHECK_TRUE(!map_image.empty());
    dg::SimpleRoadMap simple_map = localizer.getMap();
    VVS_CHECK_TRUE(painter.drawMap(map_image, simple_map));
    dg::CanvasInfo map_info = painter.getCanvasInfo(simple_map, map_image.size());

    // Run localization
    auto gps_data = getExampleGPSData();
    VVS_CHECK_TRUE(!gps_data.empty());
    auto gps_start = gps_data.front().first;
    for (auto gps_idx = 0; gps_idx < gps_data.size(); gps_idx++)
    {
        const dg::Timestamp time = gps_data[gps_idx].first;
        const dg::LatLon ll = gps_data[gps_idx].second;
        VVS_CHECK_TRUE(localizer.applyGPS(ll, time));

        if (wait_msec >= 0)
        {
            // Draw GPS observation
            dg::Point2 pt = localizer.toMetric(gps_data[gps_idx].second);
            VVS_CHECK_TRUE(painter.drawNode(map_image, map_info, dg::Point2ID(0, pt), 1, 0, cv::Vec3b(0, 0, 255)));

            // Draw the robot
            cv::Mat image = map_image.clone();
            dg::TopometricPose pose_t = localizer.getPoseTopometric();
            //dg::Pose2 pose_m = localizer.getPose();
            dg::Pose2 pose_m = localizer.toTopmetric2Metric(pose_t);
            VVS_CHECK_TRUE(painter.drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 10, 0, cx::COLOR_BLUE));
            cv::String info_topo = cv::format("Node ID: %d, Edge Idx: %d, Dist: %.3f", pose_t.node_id, pose_t.edge_idx, pose_t.dist);
            cv::putText(image, info_topo, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cx::COLOR_MAGENTA);

            cv::imshow("testLocExampleLocalizer", image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == cx::KEY_ESC) return -1;
        }
    }

    return 0;
}

#endif // End of '__TEST_LOCALIZER_EXAMPLE__'
