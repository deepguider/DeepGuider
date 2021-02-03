#ifndef __TEST_LOCALIZER_ETRI__
#define __TEST_LOCALIZER_ETRI__

#include "vvs.h"
#include "dg_core.hpp"
#include "dg_localizer.hpp"

std::vector<std::pair<double, dg::LatLon>> getGPSDataROSFix(const char* csv_file)
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

std::vector<std::pair<double, dg::LatLon>> getGPSDataAndroSen(const char* csv_file)
{
    cx::CSVReader csv;
    VVS_CHECK_TRUE(csv.open(csv_file, ';'));
    cx::CSVReader::Double2D csv_ext = csv.extDouble2D(2, { 31, 22, 23, 28 }); // Skip the header

    std::vector<std::pair<double, dg::LatLon>> data;
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        double timestamp = 1e-3 * row->at(0);
        dg::LatLon ll(row->at(1), row->at(2));
        data.push_back(std::make_pair(timestamp, ll));
    }
    return data;
}

/*
int saveETRISyntheticMap(const char* yml_file = "data/NaverMap_ETRI.yml", dg::ID id_offset = 100, double pixel_per_meter = 1, const dg::LatLon& first_latlon = dg::LatLon(36.383837659737225, 127.36788082844413))
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

dg::Map getETRISyntheticMap()
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

int saveETRINaverMap(const char* map_file = "data/NaverLabs_ETRI.csv", const dg::LatLon& ref_gps = dg::LatLon(36.383837659737, 127.367880828442), double radius = 2000)
{
    // Load a map including POIs and StreetViews
    dg::MapManager map_manager;
    if (!map_manager.setIP("129.254.87.96"))
        return -1;
    dg::Map map;
    if (!map_manager.getMap(ref_gps.lat, ref_gps.lon, radius, map))
        return -2;
    std::vector<dg::POI> pois;
    if (!map_manager.getPOI(ref_gps.lat, ref_gps.lon, radius, pois))
        return -3;
    std::vector<dg::StreetView> sviews;
    if (!map_manager.getStreetView(ref_gps.lat, ref_gps.lon, radius, sviews))
        return -4;

    // Convert the map, POIs, and StreetViews to 'dg::RoadMap' and save it
    dg::UTMConverter converter;
    VVS_CHECK_TRUE(converter.setReference(ref_gps));
    dg::RoadMap road_map = dg::BaseLocalizer::cvtMap2RoadMap(map, converter, false);
    for (auto p = pois.begin(); p != pois.end(); p++)
        road_map.addNode(dg::Point2ID(p->id, converter.toMetric(*p)));
    for (auto sv = sviews.begin(); sv != sviews.end(); sv++)
        road_map.addNode(dg::Point2ID(sv->id, converter.toMetric(*sv)));
    VVS_CHECK_TRUE(!road_map.isEmpty());
    VVS_CHECK_TRUE(road_map.save(map_file));
    return 0;
}
*/

int testLocETRIMap2RoadMap(int wait_msec = 1, const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png")
{
    // Load a map
    dg::Map map = getETRISyntheticMap();
    VVS_CHECK_TRUE(!map.nodes.empty());

    // Convert it to 'dg::RoadMap'
    dg::UTMConverter converter;
    VVS_CHECK_TRUE(converter.setReference(map.nodes.front())); // Select the first node as the origin
    dg::RoadMap road_map = dg::BaseLocalizer::cvtMap2RoadMap(map, converter);
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

int runLocalizerGPS(dg::BaseLocalizer* localizer, dg::SimpleRoadPainter* painter, const std::vector<std::pair<double, dg::LatLon>>& gps_data, int wait_msec = 1, const char* video_file = "data/191115_ETRI.avi", const char* background_file = "data/NaverMap_ETRI(Satellite)_191127.png")
{
    if (localizer == nullptr || painter == nullptr) return -1;

    // Prepare visualization
    cv::Mat map_image = cv::imread(background_file);
    VVS_CHECK_TRUE(!map_image.empty());
    dg::RoadMap* road_map = localizer->getMap();
    VVS_CHECK_TRUE(painter->drawMap(map_image, *road_map));
    dg::CanvasInfo map_info = painter->getCanvasInfo(*road_map, map_image.size());

    // Prepare the ETRI dataset
    VVS_CHECK_TRUE(!gps_data.empty());
    cv::VideoCapture video_data;
    if (strlen(video_file) > 0) video_data.open(video_file);

    const double video_time_offset = gps_data.front().first - 0.5, video_time_scale = 1.75; // Calculated from 'bag' files
    const double video_resize_scale = 0.4;
    const cv::Point video_offset(32, 542);
    const double robot_radius = 10; // Unit: [m]

    // Run localization
    cv::Mat video_image;
    double video_time = video_time_scale * video_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time_offset;
    for (size_t gps_idx = 0; gps_idx < gps_data.size(); gps_idx++)
    {
        const dg::Timestamp gps_time = gps_data[gps_idx].first;
        const dg::LatLon gps_datum = gps_data[gps_idx].second;
        VVS_CHECK_TRUE(localizer->applyGPS(gps_datum, gps_time));

        if (wait_msec >= 0)
        {
            // Draw GPS observation
            dg::Point2 pt = localizer->toMetric(gps_datum);
            cv::circle(map_image, painter->cvtMeter2Pixel(pt, map_info), 1, cx::COLOR_RED, -1);
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
            //dg::Pose2 pose_m = localizer->cvtTopmetric2Metric(pose_t);
            dg::Pose2 pose_m = localizer->getPose();
            if (!painter->drawNode(image, map_info, dg::Point2ID(0, pose_m.x, pose_m.y), robot_radius, 0, cx::COLOR_BLUE)) break;
            cv::Point pose_body = painter->cvtMeter2Pixel(pose_m, map_info);
            cv::Point pose_head = painter->cvtMeter2Pixel(pose_m + dg::Point2(robot_radius * cos(pose_m.theta), robot_radius * sin(pose_m.theta)), map_info);
            cv::line(image, pose_body, pose_head, cx::COLOR_GREEN, 2);
            cv::String info_topo = cv::format("Node ID: %zd, Edge Idx: %d, Dist: %.3f (Lat: %.6f, Lon: %.6f, Ori: %.1f)", pose_t.node_id, pose_t.edge_idx, pose_t.dist, gps_datum.lat, gps_datum.lon, cx::cvtRad2Deg(pose_m.theta));
            dg::EKFLocalizer* localizer_ekf = dynamic_cast<dg::EKFLocalizer*>(localizer);
            if (localizer_ekf != nullptr)
            {
                dg::Polar2 velocity = localizer_ekf->getVelocity();
                info_topo += cv::format(", Velocity: %.3f, %.1f", velocity.lin, cx::cvtRad2Deg(velocity.ang));
            }
            cv::putText(image, info_topo, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cx::COLOR_MAGENTA);

            cv::imshow("runLocalizerGPS", image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == cx::KEY_ESC) return -1;
        }
    }

    return 0;
}

cv::Ptr<dg::BaseLocalizer> getLocalizer(const std::string localizer_name)
{
    cv::Ptr<dg::BaseLocalizer> localizer;
    if (localizer_name == "SimpleLocalizer") localizer = cv::makePtr<dg::SimpleLocalizer>();
    else if (localizer_name == "EKFLocalizer") localizer = cv::makePtr<dg::EKFLocalizer>();
    else if (localizer_name == "EKFLocalizerZeroGyro") localizer = cv::makePtr<dg::EKFLocalizerZeroGyro>();
    else if (localizer_name == "EKFLocalizerHyperTan") localizer = cv::makePtr<dg::EKFLocalizerHyperTan>();
    else if (localizer_name == "EKFLocalizerSinTrack") localizer = cv::makePtr<dg::EKFLocalizerSinTrack>();

    cv::Ptr<dg::EKFLocalizer> localizer_ekf = localizer.dynamicCast<dg::EKFLocalizer>();
    if (!localizer_ekf.empty())
    {
        if (!localizer_ekf->setParamMotionNoise(0.1, 0.1)) return nullptr;
        if (!localizer_ekf->setParamGPSNoise(0.5)) return nullptr;
        if (!localizer_ekf->setParamValue("offset_gps", { 1, 0 })) return nullptr;
    }
    return localizer;
}

int testLocETRISyntheticMap(const std::string localizer_name = "SimpleLocalizer", int wait_msec = 1)
{
    // Prepare the localizer and its map
    cv::Ptr<dg::BaseLocalizer> localizer = getLocalizer(localizer_name);
    if (localizer.empty()) return -1;

    dg::Map map = getETRISyntheticMap();
    VVS_CHECK_TRUE(!map.nodes.empty());
    VVS_CHECK_TRUE(localizer->setReference(map.nodes.front()));
    VVS_CHECK_TRUE(localizer->loadMap(map, true));

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

    return runLocalizerGPS(localizer, &painter, getGPSDataROSFix("data/191115_ETRI_asen_fix.csv"), wait_msec);
}

int testLocETRIRealMap(const std::string localizer_name = "SimpleLocalizer", int wait_msec = 1, const char* map_file = "data/NaverLabs_ETRI.csv", const dg::LatLon& ref_gps = dg::LatLon(36.383837659737, 127.367880828442))
{
    // Prepare the localizer and its map
    cv::Ptr<dg::BaseLocalizer> localizer = getLocalizer(localizer_name);
    if (localizer.empty()) return -1;

    dg::RoadMap map;
    VVS_CHECK_TRUE(map.load(map_file));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_TRUE(localizer->loadMap(map));
    VVS_CHECK_TRUE(localizer->setReference(ref_gps));

    // Prepare the painter 
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1.045));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 344, 293 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 2));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 0));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 100, 100}));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 150, 100, 100}));
    VVS_CHECK_TRUE(painter.setParamValue("edge_thickness", 1));

    return runLocalizerGPS(localizer, &painter, getGPSDataROSFix("data/191115_ETRI_asen_fix.csv"), wait_msec);
}

int testLocCOEXRealMap(const std::string localizer_name = "SimpleLocalizer", int wait_msec = 1, const char* map_file = "data/NaverLabs_COEX.csv", const dg::LatLon& ref_gps = dg::LatLon(37.506207, 127.05482))
{
    // Prepare the localizer and its map
    cv::Ptr<dg::BaseLocalizer> localizer = getLocalizer(localizer_name);
    if (localizer.empty()) return -1;

    dg::RoadMap map;
    VVS_CHECK_TRUE(map.load(map_file));
    VVS_CHECK_TRUE(!map.isEmpty());
    VVS_CHECK_TRUE(localizer->loadMap(map));
    VVS_CHECK_TRUE(localizer->setReference(ref_gps));

    // Prepare the painter 
    dg::SimpleRoadPainter painter;
    VVS_CHECK_TRUE(painter.setParamValue("pixel_per_meter", 1.045));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_margin", 0));
    VVS_CHECK_TRUE(painter.setParamValue("canvas_offset", { 1073, 1011 }));
    VVS_CHECK_TRUE(painter.setParamValue("grid_step", 100));
    VVS_CHECK_TRUE(painter.setParamValue("grid_unit_pos", { 120, 10 }));
    VVS_CHECK_TRUE(painter.setParamValue("node_radius", 2));
    VVS_CHECK_TRUE(painter.setParamValue("node_font_scale", 0));
    VVS_CHECK_TRUE(painter.setParamValue("node_color", { 255, 100, 100 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_color", { 150, 100, 100 }));
    VVS_CHECK_TRUE(painter.setParamValue("edge_thickness", 1));

    return runLocalizerGPS(localizer, &painter, getGPSDataAndroSen("data/200925_153949_AndroSensor.csv"), wait_msec, "", "data/NaverMap_COEX(Satellite)_200929.png");
}

#endif // End of '__TEST_LOCALIZER_ETRI__'
