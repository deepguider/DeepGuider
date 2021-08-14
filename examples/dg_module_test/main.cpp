#include "module_runner.hpp"

struct MapGUIProp
{
public:
    std::string image_file;
    cv::Point2d image_scale;
    double      image_rotation = 0; // radian
    dg::LatLon  origin_latlon;      // origin of UTM
    cv::Point2d origin_px;          // pixel coordinte of UTM origin at map image
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    std::string map_file;
    double      video_resize = 0;
    cv::Point   video_offset;
    double      result_resize = 0;
};

int runModuleReal(int module_sel, bool use_saved_testset, const std::string& site, dg::DataLoader& data_loader, const std::string& rec_video_file = "")
{
    // Configure localizer
    cv::Ptr<dg::DGLocalizer> localizer = cv::makePtr<dg::DGLocalizer>();
    if (!localizer->setParamMotionNoise(1, 10)) return -1;      // linear_velocity(m), angular_velocity(deg)
    if (!localizer->setParamGPSNoise(1)) return -1;             // position error(m)
    if (!localizer->setParamGPSOffset(1, 0)) return -1;         // displacement(lin,ang) from robot origin
    if (!localizer->setParamIMUCompassNoise(1, 0)) return -1;   // angle arror(deg), angle offset(deg)
    if (!localizer->setParamPOINoise(1, 10, 1)) return -1;      // rel. distance error(m), rel. orientation error(deg), position error of poi info (m)
    if (!localizer->setParamVPSNoise(1, 10, 1)) return -1;      // rel. distance error(m), rel. orientation error(deg), position error of poi info (m)
    if (!localizer->setParamIntersectClsNoise(0.1)) return -1;  // position error(m)
    if (!localizer->setParamRoadThetaNoise(10, 0)) return -1;   // angle arror(deg), angle offset(deg)
    if (!localizer->setParamCameraOffset(1, 0)) return -1;      // displacement(lin,ang) from robot origin
    localizer->setParamValue("gps_reverse_vel", -1);
    localizer->setParamValue("search_turn_weight", 100);
    localizer->setParamValue("track_near_radius", 20);
    localizer->setParamValue("enable_path_projection", true);
    localizer->setParamValue("enable_map_projection", false);
    localizer->setParamValue("enable_backtracking_ekf", true);
    localizer->setParamValue("enable_gps_smoothing)", true);

    // Define GUI properties for ETRI and COEX sites
    MapGUIProp ETRI;
    ETRI.image_file = "data/NaverMap_ETRI(Satellite)_191127.png";
    ETRI.image_scale = cv::Point2d(1.039, 1.039);
    ETRI.image_rotation = cx::cvtDeg2Rad(1.);
    ETRI.origin_latlon = dg::LatLon(36.383837659737, 127.367880828442);
    ETRI.origin_px = cv::Point2d(347, 297);
    ETRI.map_radius = 1500; // meter
    ETRI.grid_unit_pos = cv::Point(-215, -6);
    ETRI.map_file = "data/ETRI/TopoMap_ETRI_210803.csv";
    ETRI.video_resize = 0.25;
    ETRI.video_offset = cv::Point(270, 638);
    ETRI.result_resize = 0.4;

    MapGUIProp COEX;
    COEX.image_file = "data/NaverMap_COEX(Satellite)_200929.png";
    COEX.image_scale = cv::Point2d(1.055, 1.055);
    COEX.image_rotation = cx::cvtDeg2Rad(1.2);
    COEX.origin_latlon = dg::LatLon(37.506207, 127.05482);
    COEX.origin_px = cv::Point2d(1090, 1018);
    COEX.map_radius = 1500; // meter
    COEX.grid_unit_pos = cv::Point(-230, -16);
    COEX.map_file = "data/COEX/TopoMap_COEX_210803.csv";
    COEX.video_resize = 0.4;
    COEX.video_offset = cv::Point(10, 50);
    COEX.result_resize = 0.6;

    MapGUIProp Bucheon;
    Bucheon.image_file = "data/NaverMap_Bucheon(Satellite).png";
    Bucheon.image_scale = cv::Point2d(1.056, 1.056);
    Bucheon.image_rotation = cx::cvtDeg2Rad(0);
    Bucheon.origin_latlon = dg::LatLon(37.510928, 126.764344);
    Bucheon.origin_px = cv::Point2d(1535, 1157);
    Bucheon.map_radius = 1500; // meter
    Bucheon.grid_unit_pos = cv::Point(-215, -6);
    Bucheon.map_file = "data/Bucheon/TopoMap_Bucheon_210803.csv";
    Bucheon.video_resize = 0.25;
    Bucheon.video_offset = cv::Point(270, 638);
    Bucheon.result_resize = 0.4;

    MapGUIProp guiprop = (cx::toLowerCase(site) == "coex") ? COEX : (cx::toLowerCase(site) == "bucheon") ? Bucheon : ETRI;

    // Prepare a map if given
    dg::Map map;
    map.setReference(guiprop.origin_latlon);
    if (!guiprop.map_file.empty())
    {
        if (!map.load(guiprop.map_file.c_str())) return -1;
    }

    // Read the given background image
    cv::Mat bg_image = cv::imread(guiprop.image_file, cv::ImreadModes::IMREAD_COLOR);

    // Prepare a painter for visualization
    dg::MapPainter painter;
    painter.configCanvas(guiprop.origin_px, guiprop.image_scale, bg_image.size(), 0, 0);
    painter.setImageRotation(guiprop.image_rotation);
    painter.drawGrid(bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, guiprop.grid_unit_pos);
    painter.drawOrigin(bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
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
    return runner.run(module_sel, use_saved_testset, localizer, data_loader);
}

int runModule()
{
    std::string rec_video_file = "";
    std::string gps_file, imu_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file;

    bool enable_gps = true;
    bool use_novatel = false;
    bool enable_imu = true;

    bool use_saved_testset = false;

    //int module_sel = DG_Intersection;
    //int module_sel = DG_VPS;
    //int module_sel = DG_VPS_LR;
    //int module_sel = DG_POI;
    int module_sel = DG_RoadTheta;

    int data_sel = 0;
    double start_time = 0;     // time skip (seconds)
    //rec_video_file = "result.mkv";
    std::vector<std::string> data_head[] = {
        {"data/ETRI/191115_151140", "1.75"},    // 0, 11296 frames, 1976 sec, video_scale = 1.75
        {"data/ETRI/200219_150153", "1.6244"},  // 1, 23911 frames, 3884 sec, video_scale = 1.6244
        {"data/ETRI/200326_132938", "1.6694"},  // 2, 18366 frames, 3066 sec, video_scale = 1.6694
        {"data/ETRI/200429_131714", "1.6828"},  // 3, 13953 frames, 2348 sec, video_scale = 1.6828
        {"data/ETRI/200429_140025", "1.6571"},  // 4, 28369 frames, 4701 sec, video_scale = 1.6571
        {"data/COEX/201007_142326", "2.8918"},  // 5, 12435 frames, 1240 sec, video_scale = 2.8918
        {"data/COEX/201007_145022", "2.869"},   // 6, 18730 frames, 1853 sec, video_scale = 2.869
        {"data/COEX/201007_152840", "2.8902"}   // 7, 20931 frames, 2086 sec, video_scale = 2.8902
    };
    const int coex_idx = 5;
    const std::string site = (data_sel < coex_idx) ? "ETRI" : "COEX";
    std::string video_file = (data_sel < coex_idx) ? data_head[data_sel][0] + "_images.avi" : data_head[data_sel][0] + "_images.mkv";
    if (enable_gps && !use_novatel) gps_file = data_head[data_sel][0] + "_ascen_fix.csv";
    if (enable_gps && use_novatel) gps_file = data_head[data_sel][0] + "_novatel_fix.csv";
    if (enable_imu) imu_file = data_head[data_sel][0] + "_imu_data.csv";
    if (use_saved_testset && module_sel == DG_POI) poi_file = data_head[data_sel][0] + "_poi.csv";
    if (use_saved_testset && module_sel == DG_VPS) vps_file = data_head[data_sel][0] + "_vps.csv";
    if (use_saved_testset && module_sel == DG_Intersection) intersection_file = data_head[data_sel][0] + "_intersect.csv";
    if (use_saved_testset && module_sel == DG_VPS_LR) lr_file = data_head[data_sel][0] + "_vps_lr.csv";
    if (use_saved_testset && module_sel == DG_RoadTheta) roadtheta_file = data_head[data_sel][0] + "_roadtheta.csv";

    dg::DataLoader data_loader;
    if (!data_loader.load(video_file, gps_file, imu_file, poi_file, vps_file, intersection_file, lr_file, roadtheta_file))
    {
        printf("Failed to load data file\n");
        return -1;
    }
    data_loader.setStartSkipTime(start_time);

    return runModuleReal(module_sel, use_saved_testset, site, data_loader, rec_video_file);
}

int main()
{
    return runModule();
}