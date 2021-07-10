#include "localizer_runner.hpp"

using namespace std;

cx::CSVReader::Double2D readROSGPSFix(const string& gps_file, const dg::LatLon& ref_pts = dg::LatLon(-1, -1), const vector<size_t>& cols = { 2, 3, 7, 8 })
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
                dg::LatLon ll(raw_data.front().at(1), raw_data.front().at(2));
                converter.setReference(ll);
            }
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double timestamp = row->at(0) + 1e-9 * row->at(1);
                dg::LatLon ll(row->at(2), row->at(3));
                dg::Point2 utm = converter.toMetric(ll);
                vector<double> datum = { timestamp, utm.x, utm.y };
                data.push_back(datum);
            }
        }
    }
    return data;
}

cx::CSVReader::Double2D readROSAHRS(const string& ahrs_file)
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(ahrs_file))
    {
        cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 2, 3, 5, 6, 7, 8 }); // Skip the header
        if (!raw_data.empty())
        {
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double timestamp = row->at(0) + 1e-9 * row->at(1);
                data.push_back({timestamp, row->at(5), row->at(2), row->at(3), row->at(4)});
            }
        }
    }
    return data;
}

cx::CSVReader::Double2D readAndroGPS(const string& gps_file, const dg::LatLon& ref_pts = dg::LatLon(-1, -1))
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(gps_file, ';'))
    {
        cx::CSVReader::Double2D raw_data = csv.extDouble2D(2, { 31, 22, 23, 28 }); // Skip the header
        if (!raw_data.empty())
        {
            dg::UTMConverter converter;
            if (ref_pts.lat >= 0 && ref_pts.lon >= 0) converter.setReference(ref_pts);
            else
            {
                dg::LatLon ll(raw_data.front().at(1), raw_data.front().at(2));
                converter.setReference(ll);
            }
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double timestamp = 1e-3 * row->at(0);
                dg::LatLon ll(row->at(1), row->at(2));
                dg::Point2 utm = converter.toMetric(ll);
                double accuracy = row->at(3);
                vector<double> datum = { timestamp, utm.x, utm.y, accuracy };
                data.push_back(datum);
            }
        }
    }
    return data;
}

cx::CSVReader::Double2D readLocClues(const string& clue_file)
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(clue_file)) data = csv.extDouble2D(0, { 0, 3, 4 });
    return data;
}

struct MapGUIProp
{
public:
    string      image_file;
    cv::Point2d image_scale;
    double      image_rotation = 0; // radian
    dg::LatLon  origin_latlon;      // origin of UTM
    cv::Point2d origin_px;          // pixel coordinte of UTM origin at map image
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    string      map_file;
    int         wnd_flag = cv::WindowFlags::WINDOW_AUTOSIZE;
    double      video_resize = 0;
    cv::Point   video_offset;
    double      zoom_level = 0;
    double      zoom_radius = 0;
    cv::Point   zoom_offset;
};

int drawGPSData(const MapGUIProp& gui, const vector<string>& gps_files, const vector<cv::Vec3b>& colors, int radius = 1)
{
    if (gps_files.size() != colors.size()) return -1;

    // Prepare an image and a painter for visualization
    cv::Mat image = cv::imread(gui.image_file);
    if (image.empty()) return -1;
    dg::RoadPainter painter;
    painter.configCanvas(gui.origin_px, gui.image_scale, image.size(), 0, 0);
    painter.drawGrid(image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!gui.map_file.empty())
    {
        dg::RoadMap map;
        if (map.load(gui.map_file.c_str())) painter.drawMap(image, map);
    }

    // Read and draw GPS data
    for (size_t i = 0; i < gps_files.size(); i++)
    {
        vector<vector<double>> gps_data = readROSGPSFix(gps_files[i], gui.origin_latlon);
        if (gps_data.empty()) return -1;
        if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, colors[i], radius)) return -1;
    }

    // Show the image
    cv::imshow("::drawGPSData()", image);
    cv::waitKey();
    return 0;
}

int runLocalizerReal(const MapGUIProp& gui, const string& localizer_name, const string& gps_file, const string& ahrs_file = "", const string& clue_file = "", const string& video_file = "",
    double gps_noise = 0.5, dg::Polar2 gps_offset = dg::Polar2(1, 0), double motion_noise = 0.1, const string& rec_traj_file = "", const string& rec_video_file = "", double start_time = -1, cv::Vec2d video_time = cv::Vec2d(1, -1))
{
    // Prepare a localizer
    cv::Ptr<dg::EKFLocalizer> localizer = LocalizerRunner::getLocalizer(localizer_name).dynamicCast<dg::EKFLocalizer>();
    if (localizer.empty()) return -1;
    if (!localizer->setParamMotionNoise(motion_noise, motion_noise)) return -1;
    if (!localizer->setParamGPSNoise(gps_noise)) return -1;
    if (!localizer->setParamValue("gps_offset", { gps_offset.lin, gps_offset.ang })) return -1;
    if (!localizer->setParamValue("gps_reverse_vel", -1)) return -1;
    if (!localizer->setParamValue("compass_as_gyro", true)) return -1;
    localizer->setParamValue("search_turn_weight", 100);
    localizer->setParamValue("track_near_radius", 20);

    // Prepare a map if given
    dg::RoadMap map;
    if (!gui.map_file.empty())
    {
        if (!map.load(gui.map_file.c_str())) return -1;
        cv::Ptr<dg::EKFLocalizerSinTrack> map_loader = localizer.dynamicCast<dg::EKFLocalizerSinTrack>();
        if (!map_loader.empty())
        {
            if (!map_loader->loadMap(map)) return -1;
        }
    }

    // Read GPS, AHRS, and location clue data
    cx::CSVReader::Double2D gps_data = readROSGPSFix(gps_file, gui.origin_latlon);
    if (gps_data.empty()) return -1;
    cx::CSVReader::Double2D ahrs_data, clue_data;
    if (!ahrs_file.empty())
    {
        ahrs_data = readROSAHRS(ahrs_file);
        if (ahrs_data.empty()) return -1;
    }
    if (!clue_file.empty())
    {
        clue_data = readLocClues(clue_file);
        if (clue_data.empty()) return -1;
    }

    // Prepare camera data
    cv::VideoCapture camera_data;
    if (!video_file.empty())
    {
        if (!camera_data.open(video_file)) return -1;
    }

    // Crop data if the starting time is given
    if (start_time > 0)
    {
        double first_time = gps_data.front()[0];
        auto start_gps = gps_data.begin();
        for (auto gps = start_gps; gps != gps_data.end(); gps++)
        {
            if ((gps->at(0) - first_time) >= start_time)
            {
                start_gps = gps;
                break;
            }
        }
        if (start_gps != gps_data.begin()) gps_data.erase(gps_data.begin(), start_gps - 1);

        auto start_ahrs = ahrs_data.begin();
        for (auto ahrs = start_ahrs; ahrs != ahrs_data.end(); ahrs++)
        {
            if ((ahrs->at(0) - first_time) >= start_time)
            {
                start_ahrs = ahrs;
                break;
            }
        }
        if (start_ahrs != ahrs_data.begin()) ahrs_data.erase(ahrs_data.begin(), start_ahrs - 1);

        auto start_clue = clue_data.begin();
        for (auto clue = start_clue; clue != clue_data.end(); clue++)
        {
            if ((clue->at(0) - first_time) >= start_time)
            {
                start_clue = clue;
                break;
            }
        }
        if (start_clue != clue_data.begin()) clue_data.erase(clue_data.begin(), start_clue - 1);

        if (camera_data.isOpened())
        {
            if (video_time[1] < 0) video_time[1] = first_time;
            while (true)
            {
                double timestamp = video_time[0] * camera_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time[1];
                if ((timestamp - first_time) >= start_time) break;
                cv::Mat image;
                camera_data >> image;
                if (image.empty()) break;
            }
        }
    }

    // Read the given background image
    cv::Mat bg_image = cv::imread(gui.image_file, cv::ImreadModes::IMREAD_COLOR);
    cv::Mat zoom_bg_image;
    if (gui.zoom_level > 0)
        cv::resize(bg_image, zoom_bg_image, cv::Size(), gui.zoom_level, gui.zoom_level);

    // Prepare a painter for visualization
    dg::RoadPainter painter;
    painter.configCanvas(gui.origin_px, gui.image_scale, bg_image.size(), 0, 0);
    painter.setImageRotation(gui.image_rotation);
    painter.drawGrid(bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!map.isEmpty()) painter.drawMap(bg_image, map);

    // Prepare a painter for zoomed visualization
    dg::RoadPainter zoom_painter;
    if (gui.zoom_level > 0)
    {
        zoom_painter.configCanvas(gui.origin_px * gui.zoom_level, gui.image_scale * gui.zoom_level, zoom_bg_image.size(), 0, 0);
        zoom_painter.setImageRotation(gui.image_rotation);
        zoom_painter.drawGrid(zoom_bg_image, cv::Point2d(10, 10), cv::Vec3b(200, 200, 200), 1, 0);
        zoom_painter.drawGrid(zoom_bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 3, 0);
        zoom_painter.drawOrigin(zoom_bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
        zoom_painter.setParamValue("node_radius", 6);
        zoom_painter.setParamValue("node_font_scale", 0);
        zoom_painter.setParamValue("node_color", { 255, 100, 100 });
        zoom_painter.setParamValue("edge_color", { 150, 100, 100 });
        zoom_painter.setParamValue("edge_thickness", 2);
        if (!map.isEmpty()) zoom_painter.drawMap(zoom_bg_image, map);
    }

    // Run the localizer
    LocalizerRunner experiment;
    experiment.gui_painter = &painter;
    experiment.gui_background = bg_image;
    experiment.gui_robot_radius = 12;
    experiment.gui_robot_thickness = 2;
    experiment.gui_topo_ref_radius = 6;
    experiment.gui_topo_loc_radius = 4;
    experiment.gui_time_offset = (start_time > 0) ? start_time : 0;
    experiment.gui_wnd_flag = gui.wnd_flag;
    //experiment.gui_wnd_wait_msec = 0;
    experiment.video_resize = gui.video_resize;
    experiment.video_offset = gui.video_offset;
    experiment.video_time = video_time;
    experiment.zoom_painter = &zoom_painter;
    experiment.zoom_background = zoom_bg_image;
    experiment.zoom_radius = gui.zoom_radius;
    experiment.zoom_offset = gui.zoom_offset;
    experiment.rec_traj_name = rec_traj_file;
    experiment.rec_video_name = rec_video_file;
    experiment.rec_video_resize = 0.5;
    return experiment.runLocalizer(localizer, gps_data, ahrs_data, clue_data, camera_data);
}

int runLocalizer()
{
    // Define GUI properties for ETRI and COEX sites
    MapGUIProp ETRI;
    ETRI.image_file = "data/NaverMap_ETRI(Satellite)_191127.png";
    ETRI.image_scale = cv::Point2d(1.039, 1.039);
    ETRI.image_rotation = cx::cvtDeg2Rad(1.);
    ETRI.origin_latlon = dg::LatLon(36.383837659737, 127.367880828442);
    ETRI.origin_px = cv::Point2d(347, 297);
    ETRI.map_radius = 1500; // meter
    ETRI.grid_unit_pos = cv::Point(-215, -6);
    ETRI.map_file = "data/ETRI/TopoMap_ETRI_210607_autocost.csv";
    ETRI.wnd_flag = cv::WindowFlags::WINDOW_NORMAL;
    ETRI.video_resize = 0.25;
    ETRI.video_offset = cv::Point(270, 638);
    ETRI.zoom_level = 5;
    ETRI.zoom_radius = 40;
    ETRI.zoom_offset = cv::Point(620, 400);

    MapGUIProp COEX;
    COEX.image_file = "data/NaverMap_COEX(Satellite)_200929.png";
    COEX.image_scale = cv::Point2d(1.055, 1.055);
    COEX.image_rotation = cx::cvtDeg2Rad(1.2);
    COEX.origin_latlon = dg::LatLon(37.506207, 127.05482);
    COEX.origin_px = cv::Point2d(1090, 1018);
    COEX.map_radius = 1500; // meter
    COEX.grid_unit_pos = cv::Point(-230, -16);
    COEX.map_file = "data/COEX/TopoMap_COEX_210607.csv";
    COEX.wnd_flag = cv::WindowFlags::WINDOW_NORMAL;
    COEX.video_resize = 0.4;
    COEX.video_offset = cv::Point(10, 50);
    COEX.zoom_level = 5;
    COEX.zoom_radius = 40;
    COEX.zoom_offset = cv::Point(450, 500);


    // Draw GPS data
    const cv::Vec3b COLOR_SKY(255, 127, 0);

    //ETRI.map_file = "";
    //return drawGPSData(ETRI, { "data/ETRI/191115_ETRI_ascen_fix.csv" }, { cx::COLOR_RED });
    //return drawGPSData(ETRI, { "data/ETRI/200901_ETRI_ascen_gps-fix.csv", "data/ETRI/191115_ETRI_ascen_fix.csv" }, { cx::COLOR_BLUE, cx::COLOR_RED });
    //return drawGPSData(ETRI, { "data/ETRI/200901_ETRI_ascen_gps-fix.csv", "data/ETRI/200901_DDMS_ascen_gps-fix.csv" }, { cx::COLOR_RED, cx::COLOR_BLUE });
    //return drawGPSData(ETRI, { "data/ETRI/200901_ETRI_gps-fix.csv", "data/ETRI/200901_ETRI_ascen_gps-fix.csv" }, { cx::COLOR_BLUE, cx::COLOR_RED });
    //return drawGPSData(ETRI, { "data/ETRI/200901_ETRI_gps-fix.csv", "data/ETRI/200901_DDMS_gps-fix.csv", "data/ETRI/200901_ETRI_ascen_gps-fix.csv", "data/ETRI/200901_DDMS_ascen_gps-fix.csv" }, { cx::COLOR_BLUE, COLOR_SKY, cx::COLOR_RED, cx::COLOR_MAGENTA });

    //COEX.map_file = "";
    //return drawGPSData(COEX, { "data/COEX/201007_P2C_gps-fix.csv", "data/COEX/201007_P2C_ascen_gps-fix.csv" }, { cx::COLOR_BLUE, cx::COLOR_RED });
    //return drawGPSData(COEX, { "data/COEX/201007_P2C_gps-fix.csv", "data/COEX/201007_C2P_gps-fix.csv", "data/COEX/201007_P2C_ascen_gps-fix.csv", "data/COEX/201007_C2P_ascen_gps-fix.csv" }, { cx::COLOR_BLUE, COLOR_SKY, cx::COLOR_RED, cx::COLOR_MAGENTA });
    //return drawGPSData(COEX, { "data/COEX/201007_alley_gps-fix.csv", "data/COEX/201007_SJS2P_gps-fix.csv", "data/COEX/201007_alley_ascen_gps-fix.csv", "data/COEX/201007_SJS2P_ascen_gps-fix.csv" }, { cx::COLOR_BLUE, cx::COLOR_BLUE, cx::COLOR_RED, cx::COLOR_RED });
    //return drawGPSData(COEX, { "data/COEX/201007_P2C_gps-fix.csv", "data/COEX/201007_C2P_gps-fix.csv", "data/COEX/201007_alley_gps-fix.csv", "data/COEX/201007_SJS2P_gps-fix.csv",
    //                           "data/COEX/201007_P2C_ascen_gps-fix.csv", "data/COEX/201007_C2P_ascen_gps-fix.csv", "data/COEX/201007_alley_ascen_gps-fix.csv", "data/COEX/201007_SJS2P_ascen_gps-fix.csv" },
    //                         { cx::COLOR_BLUE, cx::COLOR_BLUE, cx::COLOR_BLUE, cx::COLOR_BLUE, cx::COLOR_RED, cx::COLOR_RED, cx::COLOR_RED, cx::COLOR_RED });


    // Run localizers
    const dg::Polar2 gps_offset(1, 0);
    const double ascen_noise = 1, novatel_noise = 1, motion_noise = 1;

    //const string localizer = "EKFLocalizer";
    //const string localizer = "EKFLocalizerHyperTan";
    const string localizer = "EKFLocalizerSinTrack";

    const string gps_file = "data/ETRI/191115_151140_ascen_fix.csv";
    const string ahrs_file = "data/ETRI/191115_151140_imu_data.csv";
    const string clue_file = "";
    const string video_file = "data/ETRI/191115_151140_images.avi";

    return runLocalizerReal(ETRI, localizer, gps_file, ahrs_file, clue_file, video_file, ascen_noise, gps_offset, motion_noise, "", "", 200, { 1.75, -1 });

    //return runLocalizerReal(ETRI, localizer, "data/ETRI/191115_ETRI_ascen_fix.csv", "", "data/ETRI/191115_ETRI_201126.txt", "data/ETRI/191115_ETRI.avi", ascen_noise, gps_offset, motion_noise, "", "", 200, { 1.75, -1 });
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_ETRI_ascen_gps-fix.csv", "", "", "data/ETRI/200901_ETRI.mkv", ascen_noise, gps_offset, 1);
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_ETRI_ascen_gps-fix.csv", "data/ETRI/200901_ETRI_xsens_imu-imu-data.csv", "", "data/ETRI/200901_ETRI.mkv", ascen_noise, gps_offset, motion_noise);
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_DDMS_ascen_gps-fix.csv", "", "", "data/ETRI/200901_DDMS.mkv", ascen_noise, gps_offset, 1);
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_DDMS_ascen_gps-fix.csv", "data/ETRI/200901_DDMS_xsens_imu-imu-data.csv", "", "data/ETRI/200901_DDMS.mkv", ascen_noise, gps_offset, motion_noise);

    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_ETRI_gps-fix.csv", "", "", "data/ETRI/200901_ETRI.mkv", novatel_noise, gps_offset, motion_noise);
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_ETRI_gps-fix.csv", "data/ETRI/200901_ETRI_xsens_imu-imu-data.csv", "", "data/ETRI/200901_ETRI.mkv", novatel_noise, gps_offset, motion_noise);
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_DDMS_gps-fix.csv", "", "", "data/ETRI/200901_DDMS.mkv", novatel_noise, gps_offset, motion_noise);
    //return runLocalizerReal(ETRI, localizer, "data/ETRI/200901_DDMS_gps-fix.csv", "data/ETRI/200901_DDMS_xsens_imu-imu-data.csv", "", "data/ETRI/200901_DDMS.mkv", novatel_noise, gps_offset, motion_noise);

    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_P2C_ascen_gps-fix.csv", "", "", "data/COEX/201007_P2C.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_P2C_ascen_gps-fix.csv", "data/COEX/201007_P2C_xsens_imu-imu-data.csv", "", "data/COEX/201007_P2C.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_C2P_ascen_gps-fix.csv", "", "", "data/COEX/201007_C2P.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_C2P_ascen_gps-fix.csv", "data/COEX/201007_C2P_xsens_imu-imu-data.csv", "", "data/COEX/201007_C2P.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_alley_ascen_gps-fix.csv", "", "", "data/COEX/201007_alley.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_alley_ascen_gps-fix.csv", "data/COEX/201007_alley_xsens_imu-imu-data.csv", "", "data/COEX/201007_alley.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_SJS2P_ascen_gps-fix.csv", "", "", "data/COEX/201007_SJS2P.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
    //return runLocalizerReal(COEX, localizer, "data/COEX/201007_SJS2P_ascen_gps-fix.csv", "data/COEX/201007_SJS2P_xsens_imu-imu-data.csv", "", "data/COEX/201007_SJS2P.mkv", ascen_noise, gps_offset, motion_noise, "", "", -1, { 2.9, -1 });
}
