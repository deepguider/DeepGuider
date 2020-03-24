#ifndef __TEST_LOCALIZER_EXAMPLE__
#define __TEST_LOCALIZER_EXAMPLE__

#include "vvs.h"
#include "dg_core.hpp"
#include "dg_localizer.hpp"

std::vector<std::pair<double, dg::LatLon>> getETRIGPSData(const char* csv_file = "data/191115_ETRI_asen_fix.csv")
{
    cx::CSVReader csv;
    VVS_CHECK_TRUE(csv.open(csv_file));
    cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 2, 3, 7, 8 }); // Skip the header

    std::vector<std::pair<double, dg::LatLon>> data;
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        double timestamp = row->at(0) + 1e-9 * row->at(1);
        dg::LatLon ll(row->at(2), row->at(3));
        data.push_back(std::make_pair(timestamp, ll));
    }
    return data;
}

/*
int generateExampleMap(const char* yml_file = "data/NaverMap_ETRI.yml", dg::ID id_offset = 100, double pixel_per_meter = 1, const dg::LatLon& first_latlon = dg::LatLon(36.383837659737225, 127.36788082844413))
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
            printf("map.addNode(dg::Node(%zd, %.12f, %.12f));\n", id, ll.lat, ll.lon);
        }
    }

    // Print nodes for example map
    for (size_t r_idx = 0; r_idx < roads.size(); r_idx++)
    {
        for (size_t n_idx = 1; n_idx < roads[r_idx].size(); n_idx++)
        {
            dg::ID id = id_offset * (r_idx + 1) + n_idx;
            printf("map.addEdge(%zd, %zd);\n", id - 1, id);
        }
    }

    return 0;
}
*/

dg::Map getExampleMap()
{
    // An example map around ETRI
    // - Note: Geodesic data were roughly generated from 'generateExampleMap()' from manually selected nodes on Naver Map.
    //         The geodesic data were generated based on latitude and longitude of the first node. They are not globally accurate, but relatively accurate.
    //         1) The geodesic values of the first node can be acquired by Google Maps. To show the base point, please use the following URL, https://www.google.com/maps/@36.3838276,127.3676621,21z
    //         2) The geodesic values of the first node can be assigned by GPS data such as (36.3838455, 127.3678567).

    dg::Map map;
    map.addNode(dg::Node(100, 36.383837659737, 127.367880828442));
    map.addNode(dg::Node(101, 36.383972503935, 127.368513464272));
    map.addNode(dg::Node(102, 36.384192102671, 127.369411892631));
    map.addNode(dg::Node(103, 36.384488693234, 127.370007712223));
    map.addNode(dg::Node(104, 36.385032398405, 127.370877091215));
    map.addNode(dg::Node(105, 36.385238864809, 127.371474802670));
    map.addNode(dg::Node(106, 36.385389689704, 127.372619955728));
    map.addNode(dg::Node(107, 36.385616543283, 127.374053392578));
    map.addNode(dg::Node(108, 36.385719707014, 127.375010022487));
    map.addNode(dg::Node(109, 36.385783641605, 127.377060033716));
    map.addNode(dg::Node(110, 36.385781293449, 127.377550620897));
    map.addNode(dg::Node(111, 36.385797420879, 127.378074270063));
    map.addNode(dg::Node(112, 36.385795673859, 127.378609439303));
    map.addNode(dg::Node(113, 36.385566664196, 127.379004395247));
    map.addNode(dg::Node(114, 36.384656326678, 127.379012147148));
    map.addNode(dg::Node(115, 36.382682451501, 127.379030829600));
    map.addNode(dg::Node(116, 36.381348712540, 127.379058516894));
    map.addNode(dg::Node(117, 36.378969609824, 127.379107900521));
    map.addEdge(100, 101, dg::Edge(10001, -1, 0, true));
    map.addEdge(101, 102, dg::Edge(10102, -1, 0, true));
    map.addEdge(102, 103, dg::Edge(10203, -1, 0, true));
    map.addEdge(103, 104, dg::Edge(10304, -1, 0, true));
    map.addEdge(104, 105, dg::Edge(10405, -1, 0, true));
    map.addEdge(105, 106, dg::Edge(10506, -1, 0, true));
    map.addEdge(106, 107, dg::Edge(10607, -1, 0, true));
    map.addEdge(107, 108, dg::Edge(10708, -1, 0, true));
    map.addEdge(108, 109, dg::Edge(10809, -1, 0, true));
    map.addEdge(109, 110, dg::Edge(10910, -1, 0, true));
    map.addEdge(110, 111, dg::Edge(11011, -1, 0, true));
    map.addEdge(111, 112, dg::Edge(11112, -1, 0, true));
    map.addEdge(112, 113, dg::Edge(11213, -1, 0, true));
    map.addEdge(113, 114, dg::Edge(11314, -1, 0, true));
    map.addEdge(114, 115, dg::Edge(11415, -1, 0, true));
    map.addEdge(115, 116, dg::Edge(11516, -1, 0, true));
    map.addEdge(116, 117, dg::Edge(11617, -1, 0, true));
    return map;
}

/*
#include "dg_map_manager.hpp"

int saveETRIMap(const char* map_file = "data/NaverLabs_ETRI.csv", const dg::LatLon& ref_gps = dg::LatLon(36.383837659737, 127.367880828442))
{
    auto gps_data = getETRIGPSData();
    VVS_CHECK_TRUE(!gps_data.empty());
    auto gps_start = gps_data.front().second;
    auto gps_dest = gps_data.back().second;

    dg::MapManager map_manager;
    dg::Path path = map_manager.getPath(gps_start.lat, gps_start.lon, gps_dest.lat, gps_dest.lon);
    dg::Map map = map_manager.getMap();
    for (auto node = map.nodes.begin(); node != map.nodes.end(); node++)
    {
        if (node->lat > 90)
        {
            // Fix a wrong node
            double temp = node->lat;
            node->lat = node->lon;
            node->lon = temp;
        }
    }

    dg::UTMConverter converter;
    VVS_CHECK_TRUE(converter.setReference(ref_gps));
    dg::RoadMap road_map = dg::SimpleLocalizer::cvtMap2SimpleRoadMap(map, converter, false);
    VVS_CHECK_TRUE(!road_map.isEmpty());
    VVS_CHECK_TRUE(road_map.save(map_file));
    return 0;
}
*/

int runLocalizerETRIGPS(dg::SimpleLocalizer* localizer, dg::SimpleRoadPainter* painter, int wait_msec = 1, const char* gps_file = "data/191115_ETRI_asen_fix.csv", const char* video_file = "data/191115_ETRI.avi", const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png")
{
    if (localizer == nullptr || painter == nullptr) return -1;

    // Prepare visualization
    cv::Mat map_image = cv::imread(background_file);
    VVS_CHECK_TRUE(!map_image.empty());
    dg::RoadMap road_map = localizer->getMap();
    VVS_CHECK_TRUE(painter->drawMap(map_image, road_map));
    dg::CanvasInfo map_info = painter->getCanvasInfo(road_map, map_image.size());

    // Prepare the ETRI dataset
    auto gps_data = getETRIGPSData();
    VVS_CHECK_TRUE(!gps_data.empty());
    cv::VideoCapture video_data;
    VVS_CHECK_TRUE(video_data.open(video_file));
    double video_time_offset = gps_data.front().first - 0.5, video_time_scale = 1.75; // Calculated from 'bag' files
    double video_resize_scale = 0.4;
    cv::Point video_offset(32, 542);

    // Run localization
    cv::Mat video_image;
    double video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
    for (auto gps_idx = 0; gps_idx < gps_data.size(); gps_idx++)
    {
        const dg::Timestamp gps_time = gps_data[gps_idx].first;
        const dg::LatLon gps_datum = gps_data[gps_idx].second;
        VVS_CHECK_TRUE(localizer->applyGPS(gps_datum, gps_time));

        if (wait_msec >= 0)
        {
            // Draw GPS observation
            dg::Point2 pt = localizer->toMetric(gps_datum);
            VVS_CHECK_TRUE(painter->drawNode(map_image, map_info, dg::Point2ID(0, pt), 1, 0, cv::Vec3b(0, 0, 255)));
            cv::Mat image = map_image.clone();

            // Draw a video image if necessary
            while (video_time <= gps_time)
            {
                video_data >> video_image;
                if (video_image.empty()) break;
                video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
            }
            if (!video_image.empty())
            {
                cv::resize(video_image, video_image, cv::Size(), video_resize_scale, video_resize_scale);
                cv::Rect rect(video_offset, video_offset + cv::Point(video_image.cols, video_image.rows));
                if (rect.br().x < image.cols && rect.br().y < image.rows) image(rect) = video_image * 1;
            }

            // Draw the robot
            dg::TopometricPose pose_t = localizer->getPoseTopometric();
            //dg::Pose2 pose_m = localizer->getPose();
            dg::Pose2 pose_m = localizer->toTopmetric2Metric(pose_t);
            VVS_CHECK_TRUE(painter->drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), 10, 0, cx::COLOR_BLUE));
            cv::String info_topo = cv::format("Node ID: %d, Edge Idx: %d, Dist: %.3f (Lat: %.6f, Lon: %.6f)", pose_t.node_id, pose_t.edge_idx, pose_t.dist, gps_datum.lat, gps_datum.lon);
            cv::putText(image, info_topo, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cx::COLOR_MAGENTA);

            cv::imshow("runLocalizerETRIGPS", image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == cx::KEY_ESC) return -1;
        }
    }

    return 0;
}

int testLocMap2RoadMap(int wait_msec = 1, const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png")
{
    // Load a map
    dg::Map map = getExampleMap();
    VVS_CHECK_TRUE(!map.nodes.empty());

    // Convert it to 'dg::RoadMap'
    dg::UTMConverter converter;
    VVS_CHECK_TRUE(converter.setReference(map.nodes.front())); // Select the first node as the origin
    dg::RoadMap road_map = dg::SimpleLocalizer::cvtMap2SimpleRoadMap(map, converter);
    VVS_CHECK_TRUE(!road_map.isEmpty());

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
    VVS_CHECK_TRUE(painter.drawMap(map_image, road_map));

    if (wait_msec >= 0)
    {
        cv::imshow("testLocMap2RoadMap", map_image);
        cv::waitKey(wait_msec);
    }

    return 0;
}

int testLocSimpleExample(int wait_msec = 1)
{
    // Prepare the localizer
    dg::SimpleLocalizer localizer;
    dg::Map map = getExampleMap();
    VVS_CHECK_TRUE(!map.nodes.empty());
    VVS_CHECK_TRUE(localizer.setReference(map.nodes.front()));
    VVS_CHECK_TRUE(localizer.loadMap(map, true));

    // Prepare the painter 
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 5));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 0, 255 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 100, 0, 0 }));

    return runLocalizerETRIGPS(&localizer, &painter, wait_msec);
}

int testLocSimpleETRI(int wait_msec = 1, const char* map_file = "data/NaverLabs_ETRI.csv", const dg::LatLon& ref_gps = dg::LatLon(36.383837659737, 127.367880828442))
{
    // Prepare the localizer
    dg::SimpleLocalizer localizer;
    dg::RoadMap map;
    VVS_CHECK_TRUE(map.load(map_file));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_TRUE(localizer.loadMap(map));
    VVS_CHECK_TRUE(localizer.setReference(ref_gps));

    // Prepare the painter 
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1.045));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 4));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 0));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 50, 255 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 200, 100, 100 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_thickness", 2));

    return runLocalizerETRIGPS(&localizer, &painter, wait_msec);
}

#endif // End of '__TEST_LOCALIZER_EXAMPLE__'
