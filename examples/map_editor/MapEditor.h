#ifndef __MAP_EDITOR__
#define __MAP_EDITOR__

#include "dg_core.hpp"
#include "utils/opencx.hpp"
#include "utils/utm_converter.hpp"
#include "utils/map_painter.hpp"
#include "utils/viewport.hpp"
#include "utils/vvs.h"

void onMouseEvent(int event, int x, int y, int flags, void* param);

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

class MapEditor
{
public:
    int configure(std::string site)
    {
        // Define GUI properties for ETRI and COEX sites
        MapGUIProp ETRI;
        ETRI.image_file = "data/ETRI/NaverMap_ETRI(Satellite).png";
        ETRI.map_file = "data/ETRI/TopoMap_ETRI_210803.csv";
        ETRI.origin_latlon = dg::LatLon(36.379208, 127.364585);
        ETRI.origin_px = cv::Point2d(1344, 1371);
        ETRI.image_scale = cv::Point2d(1.2474, 1.2474);
        ETRI.image_rotation = cx::cvtDeg2Rad(0.95);
        ETRI.map_view_offset = cv::Point(1289, 371);
        ETRI.map_radius = 1500; // meter
        ETRI.grid_unit_pos = cv::Point(-215, -6);
        ETRI.video_resize = 0.2;
        ETRI.video_offset = cv::Point(350, 840);
        ETRI.result_resize = 0.5;

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

        MapGUIProp guiprop = (site == "coex") ? COEX : (site == "bucheon") ? Bucheon : ETRI;
        m_guiprop = guiprop;

        // Prepare a map if given
        m_fpath = guiprop.map_file;
        m_map.setReference(guiprop.origin_latlon);
        if (!guiprop.map_file.empty())
        {
            m_map.load(guiprop.map_file.c_str());
        }
        if (m_map.isEmpty()) m_fpath.clear();

        // Read the given background image
        m_bg_image = cv::imread(guiprop.image_file, cv::ImreadModes::IMREAD_COLOR);
        if (m_bg_image.empty()) return -1;
        m_bg_image_original = m_bg_image.clone();

        // Prepare a painter for visualization
        m_painter.configCanvas(guiprop.origin_px, guiprop.image_scale, m_bg_image.size(), 0, 0);
        m_painter.setImageRotation(guiprop.image_rotation);
        m_painter.drawGrid(m_bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, guiprop.grid_unit_pos);
        m_painter.drawOrigin(m_bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
        m_painter.setParamValue("node_radius", 3);
        m_painter.setParamValue("node_font_scale", 0);
        m_painter.setParamValue("node_color", { 255, 50, 255 });
        m_painter.setParamValue("junction_color", { 255, 250, 0 });
        m_painter.setParamValue("edge_color", { 0, 255, 255 });
        m_painter.setParamValue("sidewalk_color", { 200, 100, 100 });
        m_painter.setParamValue("crosswalk_color", { 0, 150, 50 });
        m_painter.setParamValue("mixedroad_color", { 0, 0, 255 });
        m_painter.setParamValue("edge_thickness", 2);
        if (!m_map.isEmpty()) m_painter.drawMap(m_bg_image, &m_map);

        m_view_offset = guiprop.map_view_offset;
        m_view_size = guiprop.map_view_size;

        return 0;
    }

    int run()
    {
        if (m_bg_image.empty()) return -1;
        cv::AutoLock lock(m_mutex);
        m_running = true;
        m_stop_running = false;

        m_viewport.initialize(m_bg_image, m_view_size, m_view_offset);
        std::string winname = "map viewer";
        cv::namedWindow(winname, cv::WindowFlags::WINDOW_NORMAL);
        cv::resizeWindow(winname, m_viewport.size());
        cv::setMouseCallback(winname, onMouseEvent, this);

        cv::Mat view_image;
        while (!m_stop_running)
        {
            m_viewport.getViewportImage(view_image);
            cv::imshow(winname, view_image);
            int key = cv::waitKey(10);
        }
        m_running = false;

        return 0;
    }

    void stop()
    {
        m_stop_running = true;
        cv::AutoLock lock(m_mutex);
    }

    void load();
    void save();
    void saveAs();
    void procMouseEvent(int evt, int x, int y, int flags);

protected:
    dg::MapPainter  m_painter;
    cv::Mat         m_bg_image;
    cv::Mat         m_bg_image_original;
    dg::Map         m_map;
    bool            m_running = false;
    bool            m_stop_running = false;
    cv::Mutex       m_mutex;
    std::string     m_fpath;
    MapGUIProp      m_guiprop;
    bool            m_under_edit = false;

    cv::Point       m_view_offset = cv::Point(0, 0);
    cv::Size        m_view_size = cv::Size(1800, 1012);
    dg::Viewport    m_viewport;

}; // End of 'MapEditor'


#endif // End of '__MAP_EDITOR__'
