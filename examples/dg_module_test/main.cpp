#include "module_runner.hpp"

struct MapGUIProp
{
public:
    std::string image_file;
    std::string map_file;
    dg::LatLon  origin_latlon;      // origin of UTM
    cv::Point2d origin_px;          // pixel coordinte of UTM origin at map image
    cv::Point2d image_scale;
    double      image_rotation = 0; // radian
    cv::Point   map_view_offset = cv::Point(0, 0);
    cv::Size    map_view_size = cv::Size(1800, 1012);
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    double      video_resize = 0;
    cv::Point   video_offset;
    double      result_resize = 0;
};

int runModuleReal(cv::Ptr<dg::DGLocalizer> localizer, int module_sel, bool use_saved_testset, const std::string& site, dg::DataLoader& data_loader, const std::string& rec_video_file = "")
{
    // Define GUI properties for ETRI and COEX sites
    MapGUIProp ETRI;
    ETRI.image_file = "data/ETRI/NaverMap_ETRI(Satellite).png";
    ETRI.map_file = "data/ETRI/TopoMap_ETRI.csv";
    ETRI.origin_latlon = dg::LatLon(36.379208, 127.364585);
    ETRI.origin_px = cv::Point2d(1344, 1371);
    ETRI.image_scale = cv::Point2d(1.2474, 1.2474);
    ETRI.image_rotation = cx::cvtDeg2Rad(0.95);
    ETRI.map_view_offset = cv::Point(1289, 371);
    ETRI.map_radius = 1500; // meter
    ETRI.grid_unit_pos = cv::Point(-215, -6);
    ETRI.video_resize = 0.2;
    ETRI.video_offset = cv::Point(50, 840);
    ETRI.result_resize = 0.8;

    MapGUIProp COEX;
    COEX.image_file = "data/NaverMap_COEX(Satellite)_200929.png";
    COEX.image_scale = cv::Point2d(1.055, 1.055);
    COEX.image_rotation = cx::cvtDeg2Rad(1.2);
    COEX.origin_latlon = dg::LatLon(37.506207, 127.05482);
    COEX.origin_px = cv::Point2d(1090, 1018);
    COEX.map_radius = 1500; // meter
    COEX.grid_unit_pos = cv::Point(-230, -16);
    COEX.map_file = "data/COEX/TopoMap_COEX.csv";
    COEX.video_resize = 0.2;
    COEX.video_offset = cv::Point(220, 700);
    COEX.result_resize = 0.5;

    MapGUIProp COEX2;
    COEX2.image_file = "data/COEX/NaverMap_COEX(Satellite).png";
    COEX2.image_scale = cv::Point2d(2.536, 2.536);
    COEX2.image_rotation = cx::cvtDeg2Rad(1.0);
    COEX2.origin_latlon = dg::LatLon(37.506994, 127.056676);
    COEX2.origin_px = cv::Point2d(1373, 2484);
    COEX2.map_view_offset = cv::Point(1010, 300);
    COEX2.map_radius = 2000; // meter
    COEX2.grid_unit_pos = cv::Point(-230, -16);
    COEX2.map_file = "data/COEX/TopoMap_COEX.csv";
    COEX2.video_resize = 0.2;
    COEX2.video_offset = cv::Point(220, 740);
    COEX2.result_resize = 0.4;

    MapGUIProp Bucheon;
    Bucheon.image_file = "data/NaverMap_Bucheon(Satellite).png";
    Bucheon.image_scale = cv::Point2d(1.056, 1.056);
    Bucheon.image_rotation = cx::cvtDeg2Rad(0);
    Bucheon.origin_latlon = dg::LatLon(37.510928, 126.764344);
    Bucheon.origin_px = cv::Point2d(1535, 1157);
    Bucheon.map_radius = 1500; // meter
    Bucheon.grid_unit_pos = cv::Point(-215, -6);
    Bucheon.map_file = "data/Bucheon/TopoMap_Bucheon.csv";
    Bucheon.video_resize = 0.25;
    Bucheon.video_offset = cv::Point(270, 638);
    Bucheon.result_resize = 0.4;

    MapGUIProp guiprop = (cx::toLowerCase(site) == "coex") ? COEX2 : (cx::toLowerCase(site) == "bucheon") ? Bucheon : ETRI;

    // Prepare a map if given
    dg::Map map;
    map.setReference(guiprop.origin_latlon);
    if (!guiprop.map_file.empty())
    {
        VVS_CHECK_TRUE(map.load(guiprop.map_file.c_str()));
    }
    map.updateEdgeLR();

    // Read the given background image
    cv::Mat bg_image = cv::imread(guiprop.image_file, cv::ImreadModes::IMREAD_COLOR);
    VVS_CHECK_TRUE(!bg_image.empty());

    // Prepare a painter for visualization
    dg::MapPainter painter;
    painter.configCanvas(guiprop.origin_px, guiprop.image_scale, bg_image.size(), 0, 0);
    painter.setImageRotation(guiprop.image_rotation);
    painter.drawGrid(bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, guiprop.grid_unit_pos);
    painter.drawOrigin(bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 50, 255 });
    painter.setParamValue("junction_color", { 255, 50, 255 });
    painter.setParamValue("edge_color", { 200, 100, 100 });
    painter.setParamValue("sidewalk_color", { 200, 100, 100 });
    painter.setParamValue("crosswalk_color", { 0, 150, 50 });
    painter.setParamValue("mixedroad_color", { 200, 100, 100 });
    painter.setParamValue("edge_thickness", 2);
    if (!map.isEmpty()) painter.drawMap(bg_image, &map);

    // Run the localizer
    ModuleRunner runner;
    runner.setMap(map);
    runner.gui_painter = &painter;
    runner.gui_background = bg_image;
    runner.video_resize = guiprop.video_resize;
    runner.video_offset = guiprop.video_offset;
    runner.result_resize = guiprop.result_resize;
    runner.rec_video_name = rec_video_file;
    runner.m_view_offset = guiprop.map_view_offset;
    runner.m_view_size = guiprop.map_view_size;
    return runner.run(module_sel, use_saved_testset, localizer, data_loader);
}

int runModule()
{
    std::string rec_video_file = "";
    std::string gps_file, imu_file, ocr_file, poi_file, vps_file, intersection_file, roadlr_file, roadtheta_file;

    // Configure localizer
    cv::Ptr<dg::DGLocalizer> localizer = cv::makePtr<dg::DGLocalizer>();
    if (!localizer->setParamMotionNoise(1, 10)) return -1;      // linear_velocity(m), angular_velocity(deg)
    if (!localizer->setParamMotionBounds(1, 10)) return -1;     // max_linear_velocity(m), max_angular_velocity(deg)
    if (!localizer->setParamGPSNoise(10)) return -1;            // position error(m)
    if (!localizer->setParamGPSOffset(1, 0)) return -1;         // displacement(lin,ang) from robot origin
    if (!localizer->setParamIMUCompassNoise(1, 0)) return -1;   // angle arror(deg), angle offset(deg)
    if (!localizer->setParamPOINoise(1, 10)) return -1;         // position error(m), orientation error(deg)
    if (!localizer->setParamVPSNoise(1, 10)) return -1;         // position error(m), orientation error(deg)
    if (!localizer->setParamIntersectClsNoise(0.1)) return -1;  // position error(m)
    if (!localizer->setParamRoadThetaNoise(50)) return -1;      // angle arror(deg)
    if (!localizer->setParamCameraOffset(1, 0)) return -1;      // displacement(lin,ang) from robot origin
    localizer->setParamValue("gps_reverse_vel", -0.5);
    localizer->setParamValue("search_turn_weight", 100);
    localizer->setParamValue("track_near_radius", 20);
    localizer->setParamValue("enable_path_projection", true);
    localizer->setParamValue("enable_map_projection", false);
    localizer->setParamValue("enable_backtracking_ekf", true);
    localizer->setParamValue("enable_gps_smoothing", true);
    localizer->setParamValue("enable_debugging_display", false);
    localizer->setParamValue("lr_mismatch_cost", 50);
    localizer->setParamValue("enable_lr_reject", false);
    localizer->setParamValue("lr_reject_cost", 20);             // 20
    localizer->setParamValue("enable_discontinuity_cost", true);
    localizer->setParamValue("discontinuity_weight", 0.5);      // 0.5

    bool enable_gps = true;
    bool use_novatel = false;
    bool enable_imu = false;
    bool use_saved_testset = true;

    int module_sel = -1;
    //module_sel = DG_Intersection;
    //module_sel = DG_VPS;
    //module_sel = DG_RoadLR;
    module_sel = DG_OCR;
    //module_sel = DG_POI;
    //module_sel = DG_RoadTheta;

    int data_sel = 0;
    double start_time = 0;     // time skip (seconds)
    start_time = 1360;     // time skip (seconds), testset 1 우리은행
    //start_time = 1180;     // time skip (seconds), testset 2 상가 초입
    //start_time = 1440;     // time skip (seconds), testset 2 횡단보도 진입시점
    //rec_video_file = "module_test.avi";
    std::vector<std::string> data_head[] = {
        {"data/ETRI/191115_151140", "1.75"},    // 0, 11296 frames, 1976 sec, video_scale = 1.75
        {"data/ETRI/200219_150153", "1.6244"},  // 1, 23911 frames, 3884 sec, video_scale = 1.6244
        {"data/ETRI/200326_132938", "1.6694"},  // 2, 18366 frames, 3066 sec, video_scale = 1.6694
        {"data/ETRI/200429_131714", "1.6828"},  // 3, 13953 frames, 2348 sec, video_scale = 1.6828
        {"data/ETRI/200429_140025", "1.6571"},  // 4, 28369 frames, 4701 sec, video_scale = 1.6571
        {"data/COEX/201007_142326", "2.8918"},  // 5, 12435 frames, 1240 sec, video_scale = 2.8918
        {"data/COEX/201007_145022", "2.869"},   // 6, 18730 frames, 1853 sec, video_scale = 2.869
        {"data/COEX/201007_152840", "2.8902"},  // 7, 20931 frames, 2086 sec, video_scale = 2.8902
        {"data/COEX/211005_130940", "2.8902"}   // 8, 20931 frames, 2086 sec, video_scale = 2.8902
    };
    const int coex_idx = 5;
    const std::string site = (data_sel < coex_idx) ? "ETRI" : "COEX";
    std::string video_file = (data_sel < coex_idx) ? data_head[data_sel][0] + "_images.avi" : data_head[data_sel][0] + "_images.mkv";
    if (data_sel == 8) video_file = data_head[data_sel][0] + "_images.avi";
    if (enable_gps && !use_novatel) gps_file = data_head[data_sel][0] + "_ascen_fix.csv";
    if (enable_gps && use_novatel) gps_file = data_head[data_sel][0] + "_novatel_fix.csv";
    if (enable_imu) imu_file = data_head[data_sel][0] + "_imu_data.csv";
    if (use_saved_testset && module_sel == DG_OCR) ocr_file = data_head[data_sel][0] + "_ocr.csv";
    if (use_saved_testset && module_sel == DG_POI) poi_file = data_head[data_sel][0] + "_poi.csv";
    if (use_saved_testset && module_sel == DG_VPS) vps_file = data_head[data_sel][0] + "_vps.csv";
    if (use_saved_testset && module_sel == DG_Intersection) intersection_file = data_head[data_sel][0] + "_intersect.csv";
    if (use_saved_testset && module_sel == DG_RoadLR) roadlr_file = data_head[data_sel][0] + "_roadlr.csv";
    if (use_saved_testset && module_sel == DG_RoadTheta) roadtheta_file = data_head[data_sel][0] + "_roadtheta.csv";

    dg::DataLoader data_loader;
    if (!data_loader.load(video_file, gps_file, imu_file, ocr_file, poi_file, vps_file, intersection_file, roadlr_file, roadtheta_file))
    {
        printf("Failed to load data file\n");
        return -1;
    }
    data_loader.setStartSkipTime(start_time);

    return runModuleReal(localizer, module_sel, use_saved_testset, site, data_loader, rec_video_file);
}

int main()
{
    return runModule();
}
