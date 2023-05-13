#include "test_core_type.hpp"
#include "test_localizer_gps2utm.hpp"
#include "test_localizer_map.hpp"
#include "test_localizer_ekf.hpp"
#include "test_localizer_etri.hpp"
#include "test_utility.hpp"
#include "localizer_runner.hpp"
#include "localizer/data_loader.hpp"

struct MapGUIProp
{
public:
    std::string image_file;
    cv::Point2d map_pixel_per_meter;
    double      map_image_rotation = 0; // radian
    dg::LatLon  map_ref_point_latlon;      // origin of UTM
    cv::Point2d map_ref_point_pixel;          // pixel coordinte of UTM origin at map image
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    std::string map_file;
    int         wnd_flag = cv::WindowFlags::WINDOW_AUTOSIZE;
    double      video_resize = 0;
    cv::Point   video_offset;
    bool        show_zoom = false;
    double      zoom_level = 0;
    double      zoom_radius = 0;
    cv::Point   zoom_offset;
    cv::Point   view_offset = cv::Point(0, 0);
    cv::Size    view_size = cv::Size(1920, 1080);
};

cx::CSVReader::Double2D readROSGPSFix(const std::string& gps_file, const dg::LatLon& ref_pts = dg::LatLon(-1, -1), const std::vector<size_t>& cols = { 2, 3, 5, 7, 8 })
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(gps_file))
    {
        cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, cols); // Skip the header
        if (!raw_data.empty())
        {
            dg::UTMConverter converter;
            if (ref_pts.lat >= 0 && ref_pts.lon >= 0) converter.setReference(ref_pts);
            else
            {
                dg::LatLon ll(raw_data.front().at(3), raw_data.front().at(4));
                converter.setReference(ll);
            }
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double status = row->at(2);
                if (status < 0) continue;   // skip nan data

                double timestamp = row->at(0) + 1e-9 * row->at(1);
                dg::LatLon ll(row->at(3), row->at(4));
                dg::Point2 utm = converter.toMetric(ll);
                std::vector<double> datum = { timestamp, utm.x, utm.y };
                data.push_back(datum);
            }
        }
    }
    return data;
}

int drawGPSData(const MapGUIProp& gui, const std::string& gps_file, const cv::Vec3b& color, int radius = 1, int gps_smoothing_n = 0)
{
    // Prepare an image and a painter for visualization
    cv::Mat image = cv::imread(gui.image_file);
    if (image.empty()) return -1;
    dg::MapPainter painter;
    painter.configCanvas(gui.map_ref_point_pixel, gui.map_pixel_per_meter, image.size(), 0, 0);
    painter.setImageRotation(gui.map_image_rotation);
    painter.drawGrid(image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!gui.map_file.empty())
    {
        dg::Map map;
        if (map.load(gui.map_file.c_str())) painter.drawMap(image, &map);
    }

    // Read and draw GPS data
    std::vector<std::vector<double>> gps_data = readROSGPSFix(gps_file, gui.map_ref_point_latlon);
    if (gps_data.empty()) return -1;
    if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, color, radius)) return -1;

    // Draw smoothed gps
    if (gps_smoothing_n > 0)
    {
        std::vector<std::vector<double>> tmp = gps_data;
        for (int k = 0; k < gps_smoothing_n; k++)
        {
            int n = (int)gps_data.size();
            for (int i = 1; i < n - 1; i++)
            {
                tmp[i][1] = (2 * gps_data[i][1] + gps_data[i - 1][1] + gps_data[i + 1][1]) / 4;
                tmp[i][2] = (2 * gps_data[i][2] + gps_data[i - 1][2] + gps_data[i + 1][2]) / 4;
            }
            tmp[0][1] = (1.5 * gps_data[0][1] + 2 * gps_data[1][1] - gps_data[2][1]) / 2.5;
            tmp[0][2] = (1.5 * gps_data[0][2] + 2 * gps_data[1][2] - gps_data[2][2]) / 2.5;
            tmp[n - 1][1] = (1.5 * gps_data[n - 1][1] + 2 * gps_data[n - 2][1] - gps_data[n - 3][1]) / 2.5;
            tmp[n - 1][2] = (1.5 * gps_data[n - 1][2] + 2 * gps_data[n - 2][2] - gps_data[n - 3][2]) / 2.5;
            gps_data = tmp;
        }
        if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, cx::COLOR_BLUE, radius)) return -1;
    }

    // Show the image
    cv::namedWindow("::drawGPSData()", gui.wnd_flag);
    cv::imshow("::drawGPSData()", image);
    cv::waitKey();
    return 0;
}

int drawGPSData(const MapGUIProp& gui, const std::vector<std::string>& gps_files, const std::vector<cv::Vec3b>& colors, int radius = 1)
{
    if (gps_files.size() != colors.size()) return -1;

    // Prepare an image and a painter for visualization
    cv::Mat image = cv::imread(gui.image_file);
    if (image.empty()) return -1;
    dg::MapPainter painter;
    painter.configCanvas(gui.map_ref_point_pixel, gui.map_pixel_per_meter, image.size(), 0, 0);
    painter.setImageRotation(gui.map_image_rotation);
    painter.drawGrid(image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!gui.map_file.empty())
    {
        dg::Map map;
        if (map.load(gui.map_file.c_str())) painter.drawMap(image, &map);
    }

    // Read and draw GPS data
    for (size_t i = 0; i < gps_files.size(); i++)
    {
        std::vector<std::vector<double>> gps_data = readROSGPSFix(gps_files[i], gui.map_ref_point_latlon);
        if (gps_data.empty()) return -1;
        if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, colors[i], radius)) return -1;
    }

    // Show the image
    cv::namedWindow("::drawGPSData()", gui.wnd_flag);
    cv::imshow("::drawGPSData()", image);
    cv::waitKey();
    return 0;
}

int runLocalizerReal(const MapGUIProp& gui, cv::Ptr<dg::BaseLocalizer> localizer, dg::DataLoader& data_loader, const std::string& rec_traj_file = "", const std::string& rec_video_file = "", double rec_video_fps = 10, bool use_mcl = true)
{
    if (localizer.empty()) return -1;

    // Prepare a map if given
    dg::Map map;
    map.setReference(gui.map_ref_point_latlon);
    if (!gui.map_file.empty())
    {
        if (!map.load(gui.map_file.c_str())) return -1;
    }

    // Read the given background image
    cv::Mat bg_image = cv::imread(gui.image_file, cv::ImreadModes::IMREAD_COLOR);
    cv::Mat zoom_bg_image;
    if (gui.zoom_level > 0)
        cv::resize(bg_image, zoom_bg_image, cv::Size(), gui.zoom_level, gui.zoom_level);

    // Prepare a painter for visualization
    dg::MapPainter painter;
    painter.configCanvas(gui.map_ref_point_pixel, gui.map_pixel_per_meter, bg_image.size(), 0, 0);
    painter.setImageRotation(gui.map_image_rotation);
    painter.drawGrid(bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("junction_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("sidewalk_color", { 150, 100, 100 });
    painter.setParamValue("mixedroad_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!map.isEmpty()) painter.drawMap(bg_image, &map);

    // Prepare a painter for zoomed visualization
    dg::MapPainter zoom_painter;
    if (gui.zoom_level > 0)
    {
        zoom_painter.configCanvas(gui.map_ref_point_pixel * gui.zoom_level, gui.map_pixel_per_meter * gui.zoom_level, zoom_bg_image.size(), 0, 0);
        zoom_painter.setImageRotation(gui.map_image_rotation);
        zoom_painter.drawGrid(zoom_bg_image, cv::Point2d(10, 10), cv::Vec3b(200, 200, 200), 1, 0);
        zoom_painter.drawGrid(zoom_bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 3, 0);
        zoom_painter.drawOrigin(zoom_bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
        zoom_painter.setParamValue("node_radius", 6);
        zoom_painter.setParamValue("node_font_scale", 0);
        zoom_painter.setParamValue("node_color", { 255, 100, 100 });
        zoom_painter.setParamValue("junction_color", { 255, 100, 100 });
        zoom_painter.setParamValue("edge_color", { 150, 100, 100 });
        zoom_painter.setParamValue("sidewalk_color", { 150, 100, 100 });
        zoom_painter.setParamValue("mixedroad_color", { 150, 100, 100 });
        zoom_painter.setParamValue("edge_thickness", 2);
        if (!map.isEmpty()) zoom_painter.drawMap(zoom_bg_image, &map);
    }

    // Run the localizer
    LocalizerRunner experiment;
    experiment.setMap(map);
    experiment.gui_painter = &painter;
    experiment.gui_background = bg_image;
    experiment.gui_robot_radius = 12;
    experiment.gui_robot_thickness = 2;
    experiment.gui_topo_ref_radius = 6;
    experiment.gui_topo_loc_radius = 4;
    experiment.gui_wnd_flag = gui.wnd_flag;
    experiment.m_view_size = gui.view_size;
    experiment.m_view_offset = gui.view_offset;
    experiment.video_resize = gui.video_resize;
    experiment.video_offset = gui.video_offset;
    experiment.show_zoom = gui.show_zoom;
    experiment.zoom_painter = &zoom_painter;
    experiment.zoom_background = zoom_bg_image;
    experiment.zoom_radius = gui.zoom_radius;
    experiment.zoom_offset = gui.zoom_offset;
    experiment.rec_traj_name = rec_traj_file;
    experiment.rec_video_name = rec_video_file;
    experiment.rec_video_resize = 0.5;
    experiment.rec_video_fps = rec_video_fps;
    return experiment.runLocalizer(localizer, data_loader, use_mcl);
}

int testUTMConverter()
{
    dg::UTMConverter converter;
    dg::Point2UTM p1 = converter.cvtLatLon2UTM(dg::LatLon(36.383837659737, 127.367880828442));  // etri
    dg::Point2UTM p2 = converter.cvtLatLon2UTM(dg::LatLon(37.506207, 127.05482));               // coex
    dg::Point2UTM p3 = converter.cvtLatLon2UTM(dg::LatLon(37.510928, 126.764344));              // bucheon
    dg::Point2UTM p4 = converter.cvtLatLon2UTM(dg::LatLon(33.19740, 126.10256));              // 남서단
    dg::Point2UTM p5 = converter.cvtLatLon2UTM(dg::LatLon(38.61815, 129.58138));              // 북동단
    printf("etri: zone=%d, x = %lf, y = %lf\n", p1.zone, p1.x, p1.y);
    printf("coex: zone=%d, x = %lf, y = %lf\n", p2.zone, p2.x, p2.y);
    printf("bucheon: zone=%d, x = %lf, y = %lf\n", p3.zone, p3.x, p3.y);
    printf("SW: zone=%d, x = %lf, y = %lf\n", p4.zone, p4.x, p4.y);
    printf("NE: zone=%d, x = %lf, y = %lf\n", p5.zone, p5.x, p5.y);

    dg::Point2 p(1000, 0);
    converter.setReference(converter.toLatLon(p1));
    dg::LatLon ll = converter.toLatLon(p);
    dg::Point2 q = converter.toMetric(ll);
    printf("\nx=%lf, y=%lf, lat=%lf, lon=%lf, x=%lf, y=%lf\n", p.x, p.y, ll.lat, ll.lon, q.x, q.y);

    return 0;
}

int runUnitTest()
{
    // Test 'core' module
    // 1. Test basic data structures
    VVS_RUN_TEST(testCoreLatLon());
    VVS_RUN_TEST(testCorePolar2());
    VVS_RUN_TEST(testCorePoint2ID());

    // 2. Test 'dg::Map' and its related
    VVS_RUN_TEST(testCoreNode());
    VVS_RUN_TEST(testCoreEdge());
    VVS_RUN_TEST(testCoreMap());
    VVS_RUN_TEST(testCorePath());

    // Test 'localizer' module
    // 1. Test GPS and UTM conversion
    VVS_RUN_TEST(testLocRawGPS2UTM(dg::LatLon(38, 128), dg::Point2(412201.58, 4206286.76))); // Zone: 52S
    VVS_RUN_TEST(testLocRawGPS2UTM(dg::LatLon(37, 127), dg::Point2(322037.81, 4096742.06))); // Zone: 52S
    VVS_RUN_TEST(testLocRawUTM2GPS(dg::Point2(412201.58, 4206286.76), 52, false, dg::LatLon(38, 128)));
    VVS_RUN_TEST(testLocRawUTM2GPS(dg::Point2(322037.81, 4096742.06), 52, false, dg::LatLon(37, 127)));
    VVS_RUN_TEST(testLocRawUTM2GPS(dg::Point2(0, 0), 52, false, dg::LatLon(-1, -1))); // Print the origin of the Zone 52
    VVS_RUN_TEST(testLocUTMConverter());

    // 2. Test 'dg::Map' and 'dg::MapPainter'
    VVS_RUN_TEST(testLocMap());
    VVS_RUN_TEST(testLocMapPainter());

    // 3. Test localizers
    VVS_RUN_TEST(testLocEKFGPS());
    VVS_RUN_TEST(testLocEKFGyroGPS());
    VVS_RUN_TEST(testLocEKFBacktracking());

    VVS_RUN_TEST(testLocETRIMap2RoadMap());
    VVS_RUN_TEST(testLocETRISyntheticMap());
    VVS_RUN_TEST(testLocETRIRealMap());
    VVS_RUN_TEST(testLocCOEXRealMap());

    // Test 'utility' module
    VVS_RUN_TEST(testUtilRingBuffer());

    return 0;
}

int runLocalizer()
{
    std::string rec_video_file = "";
    const std::string rec_traj_file = "";
    std::string gps_file, odo_file, imu_file, ocr_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file;
    bool enable_gps = true;
    bool enable_odometry = false;
    bool use_andro_gps = false;
    bool use_novatel = false;
    bool enable_imu = false;
    bool enable_ocr = false;
    bool enable_poi = false;
    bool enable_vps = false;
    bool enable_intersection = false;
    bool enable_lr = false;
    bool enable_roadtheta = false;
    bool draw_gps = false;
    bool use_mcl = true;
    //use_mcl = false;

    // Define GUI properties for ETRI and COEX sites
    MapGUIProp ETRI;
    ETRI.image_file = "data/NaverMap_ETRI(Satellite)_191127.png";
    ETRI.map_pixel_per_meter = cv::Point2d(1.039, 1.039);
    ETRI.map_image_rotation = cx::cvtDeg2Rad(1.);
    ETRI.map_ref_point_latlon = dg::LatLon(36.383837659737, 127.367880828442);
    ETRI.map_ref_point_pixel = cv::Point2d(347, 297);
    ETRI.map_radius = 1500; // meter
    ETRI.grid_unit_pos = cv::Point(-215, -6);
    ETRI.map_file = "data/ETRI/TopoMap_ETRI.csv";
    ETRI.wnd_flag = cv::WindowFlags::WINDOW_NORMAL;
    ETRI.video_resize = 0.25;
    ETRI.video_offset = cv::Point(270, 638);
    ETRI.show_zoom = false;
    ETRI.zoom_level = 5;
    ETRI.zoom_radius = 40;
    ETRI.zoom_offset = cv::Point(620, 400);

    MapGUIProp ETRI2;
    ETRI2.image_file = "data/ETRI/NaverMap_ETRI(Satellite)_large.png";
    ETRI2.map_pixel_per_meter = cv::Point2d(2.081, 2.081);
    ETRI2.map_image_rotation = cx::cvtDeg2Rad(0.96);
    ETRI2.map_ref_point_latlon = dg::LatLon(36.379208, 127.364585);
    ETRI2.map_ref_point_pixel = cv::Point2d(3790, 3409);
    ETRI2.map_radius = 1500; // meter
    ETRI2.grid_unit_pos = cv::Point(-215, -6);
    ETRI2.map_file = "data/ETRI/TopoMap_ETRI.csv";
    ETRI2.wnd_flag = cv::WindowFlags::WINDOW_NORMAL;
    ETRI2.video_resize = 0.25;
    ETRI2.video_offset = cv::Point(270, 638);
    ETRI2.zoom_level = 5;
    ETRI2.zoom_radius = 40;
    ETRI2.zoom_offset = cv::Point(620, 400);

    MapGUIProp COEX;
    COEX.image_file = "data/NaverMap_COEX(Satellite)_200929.png";
    COEX.map_pixel_per_meter = cv::Point2d(1.055, 1.055);
    COEX.map_image_rotation = cx::cvtDeg2Rad(1.2);
    COEX.map_ref_point_latlon = dg::LatLon(37.506207, 127.05482);
    COEX.map_ref_point_pixel = cv::Point2d(1090, 1018);
    COEX.map_radius = 1500; // meter
    COEX.grid_unit_pos = cv::Point(-230, -16);
    COEX.map_file = "data/COEX/TopoMap_COEX.csv";
    COEX.wnd_flag = cv::WindowFlags::WINDOW_NORMAL;
    COEX.video_resize = 0.4;
    COEX.video_offset = cv::Point(200, 50);
    COEX.show_zoom = false;
    COEX.zoom_level = 5;
    COEX.zoom_radius = 40;
    COEX.zoom_offset = cv::Point(450, 500);

    MapGUIProp Bucheon;
    Bucheon.image_file = "data/Bucheon/NaverMap_Bucheon(Satellite).png";
    Bucheon.map_pixel_per_meter = cv::Point2d(3.95, 3.95);
    Bucheon.map_image_rotation = cx::cvtDeg2Rad(1.3);
    Bucheon.map_ref_point_latlon = dg::LatLon(37.517337, 126.764761);
    Bucheon.map_ref_point_pixel = cv::Point2d(2225, 880);
    Bucheon.map_radius = 1500; // meter
    Bucheon.grid_unit_pos = cv::Point(-230, -16);
    Bucheon.map_file = "data/Bucheon/TopoMap_Bucheon.csv";
    Bucheon.wnd_flag = cv::WindowFlags::WINDOW_NORMAL;
    //Bucheon.view_offset = cv::Point(1289, 371);
    Bucheon.view_offset = cv::Point(1500, 6000);
    Bucheon.view_size = cv::Size(1800, 1012);
    Bucheon.video_resize = 0.25;
    Bucheon.video_offset = cv::Point(270, 638);
    Bucheon.show_zoom = false;
    Bucheon.zoom_level = 3;
    Bucheon.zoom_radius = 20;
    Bucheon.zoom_offset = cv::Point(1300, 0);

    cv::Ptr<dg::BaseLocalizer> localizer;
    //localizer = cv::makePtr<dg::EKFLocalizer>();
    //localizer = cv::makePtr<dg::EKFLocalizerHyperTan>();
    //localizer = cv::makePtr<dg::EKFLocalizerSinTrack>();
    if(use_mcl) localizer = cv::makePtr<dg::DGLocalizerMCL>("EKFLocalizerHyperTan");
    else localizer = cv::makePtr<dg::DGLocalizer>("EKFLocalizerHyperTan");

    if (!localizer->setParamMotionNoise(1, 10)) return -1;      // linear_velocity(m), angular_velocity(deg)
    if (!localizer->setParamMotionBounds(1, 20)) return -1;     // max. linear_velocity(m), max. angular_velocity(deg)
    if (!localizer->setParamGPSNoise(5)) return -1;            // position error(m)
    if (!localizer->setParamGPSOffset(1, 0)) return -1;         // displacement(lin,ang) from robot origin
    if (!localizer->setParamOdometryNoise(0.1, 2)) return -1;  // position error(m), orientation error(deg)
    if (!localizer->setParamIMUCompassNoise(1, 0)) return -1;   // angle arror(deg), angle offset(deg)
    if (!localizer->setParamPOINoise(1, 10)) return -1;         // distance error(m), orientation error(deg)
    if (!localizer->setParamVPSNoise(1, 10)) return -1;         // distance error(m), orientation error(deg)
    if (!localizer->setParamIntersectClsNoise(0.1)) return -1;  // position error(m)
    if (!localizer->setParamRoadThetaNoise(50)) return -1;      // angle arror(deg)
    if (!localizer->setParamCameraOffset(1, 0)) return -1;      // displacement(lin,ang) from robot origin
    localizer->setParamValue("max_observation_error", 20);         // meter
    localizer->setParamValue("odometry_stabilization_d", 0.5);     // meter
    localizer->setParamValue("gps_reverse_vel", -0.5);
    localizer->setParamValue("search_turn_weight", 100);
    localizer->setParamValue("track_near_radius", 20);
    localizer->setParamValue("enable_path_projection", true);
    localizer->setParamValue("enable_map_projection", false);
    localizer->setParamValue("enable_backtracking_ekf", true);
    localizer->setParamValue("enable_gps_smoothing", false);
    localizer->setParamValue("enable_stop_filtering", true);

    enable_odometry = true;
    //enable_imu = true;
    use_andro_gps = true;
    //use_novatel = true;
    //enable_ocr = true;
    //enable_poi = true;
    //enable_vps = true;
    //enable_intersection = true;
    //enable_lr = true;
    //enable_roadtheta = true;
    //draw_gps = true;

    int data_sel = 0;
    double start_time = 0;     // skip time (seconds)
    //rec_video_file = "mcl_bucheon_course.avi";
    double rec_video_fps = 10;
    std::vector<std::string> data_head[] = {
        {"data/ETRI/2022-08-08-13-38-04_etri_to_119", "avi", "0"},
        {"data/ETRI/191115_151140", "avi", "1.75"},    // 1, 11296 frames, 1976 sec, video_scale = 1.75
        {"data/ETRI/200219_150153", "avi", "1.6244"},  // 2, 23911 frames, 3884 sec, video_scale = 1.6244
        {"data/ETRI/200326_132938", "avi", "1.6694"},  // 3, 18366 frames, 3066 sec, video_scale = 1.6694
        {"data/ETRI/200429_131714", "avi", "1.6828"},  // 4, 13953 frames, 2348 sec, video_scale = 1.6828
        {"data/ETRI/200429_140025", "avi", "1.6571"},  // 5, 28369 frames, 4701 sec, video_scale = 1.6571
        {"data/COEX/2022-08-29-16-01-55", "avi", "0"}, // 6
        {"data/COEX/201007_142326", "mkv", "2.8918"},  // 7, 12435 frames, 1240 sec, video_scale = 2.8918
        {"data/COEX/201007_145022", "mkv", "2.869"},   // 8, 18730 frames, 1853 sec, video_scale = 2.869
        {"data/COEX/201007_152840", "mkv", "2.8902"},   // 9, 20931 frames, 2086 sec, video_scale = 2.8902
        {"data/Bucheon/_2022-10-06-14-40-21_subway_to_cityhall", "avi", "0"}, // 10
        {"data/Bucheon/_2022-11-08-16-05-30_keti2nonghyup_notbad", "avi", "0"}, // 11
        {"data/Bucheon/_2022-11-08-15-50-06_nonghyp2keti_notbad", "avi", "0"} // 12
    };
    const int coex_idx = 6;
    const int bucheon_idx = 10;
    MapGUIProp PROP = (data_sel < coex_idx) ? ETRI : COEX;
    if (data_sel >= bucheon_idx) PROP = Bucheon;
    std::string video_file = data_head[data_sel][0] + "_images." + data_head[data_sel][1];
    video_file = "";
    if (enable_gps)
    {
        if (use_andro_gps) gps_file = data_head[data_sel][0] + "_andro2linux_gps.csv";
        else if (use_novatel) gps_file = data_head[data_sel][0] + "_novatel_fix.csv";
        else gps_file = data_head[data_sel][0] + "_ascen_fix.csv";
    }
    if (enable_odometry) odo_file = data_head[data_sel][0] + "_odometry.csv";
    if (enable_imu) imu_file = data_head[data_sel][0] + "_imu_data.csv";
    if (enable_ocr) ocr_file = data_head[data_sel][0] + "_ocr.csv";
    if (enable_poi) poi_file = data_head[data_sel][0] + "_poi.csv";
    if (enable_vps) vps_file = data_head[data_sel][0] + "_vps.csv";
    if (enable_intersection) intersection_file = data_head[data_sel][0] + "_intersect.csv";
    if (enable_lr) lr_file = data_head[data_sel][0] + "_vps_lr.csv";
    if (enable_roadtheta) roadtheta_file = data_head[data_sel][0] + "_roadtheta.csv";

    dg::DataLoader data_loader;
    if (!data_loader.load(video_file, gps_file, odo_file, imu_file, ocr_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file))
    {
        printf("Failed to load data file\n");
        return -1;
    }
    data_loader.setStartSkipTime(start_time);

    // Draw GPS data
    if (draw_gps)
    {
        int gps_smoothing_n = 0;
        const cv::Vec3b COLOR_SKY(255, 127, 0);
        return drawGPSData(PROP, gps_file, { cx::COLOR_RED }, 1, gps_smoothing_n);
    }

    return runLocalizerReal(PROP, localizer, data_loader, rec_traj_file, rec_video_file, rec_video_fps, use_mcl);
}

int main()
{
    //cv::Ptr<dg::DGLocalizerMCL> localizer = cv::makePtr<dg::DGLocalizerMCL>("EKFLocalizerHyperTan");
    //localizer->testAlignCost(); return 0;
    //return testUTMConverter();
    //return runUnitTest();
    return runLocalizer();
}
