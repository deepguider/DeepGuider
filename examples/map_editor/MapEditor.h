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
    std::string server_ip = "129.254.81.204";
    std::string server_port = "10000";
    std::string image_file;
    std::string map_file;
    dg::LatLon  map_ref_point_latlon;       // origin of UTM
    cv::Point2d map_ref_point_pixel;        // pixel coordinte of UTM origin at map image
    double      map_pixel_per_meter;
    double      map_image_rotation = 0;     // radian
    cv::Point   map_view_offset = cv::Point(0, 0);
    cv::Size    map_view_size = cv::Size(1920, 1080);
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    double      video_resize = 0;
    cv::Point   video_offset;
    double      result_resize = 0;
};

class MapEditor
{
public:
    int configure(std::string site);
    int run();
    void stop();

    void procMouseEvent(int evt, int x, int y, int flags);
    void download();
    void load();
    void save();
    void saveAs();
    void showPoi(bool show);
    void showStreetView(bool show);
    void showLRSide(bool show);
    void showMapError(bool show);
    void verify();
    void fixMapError();
    void updateLRSide();
    void computeLRSide(dg::Map& map);
    void exportToJson();
    void adjustMapAlign();
    void applyMapAlign(double px, double py, double map_pixel_per_meter, double rotation);

protected:
    void drawMap(cv::Mat view_image, cv::Point2d offset, double zoom);
    dg::ID          m_next_id;
    void initializeNextMapID();
    dg::ID getNextMapID() { return m_next_id++; }

    dg::MapPainter  m_painter;
    cv::Mat         m_bg_image;
    cv::Mat         m_bg_image_original;
    dg::Map         m_map;
    bool            m_running = false;
    bool            m_stop_running = false;
    cv::Mutex       m_mutex_data;
    cv::Mutex       m_mutex_run;
    std::string     m_fpath;
    MapGUIProp      m_guiprop;
    bool            m_under_edit = false;
    std::string     m_winname = "map";
    std::string     m_site = "etri";
    bool            m_show_poi = false;
    bool            m_show_streetview = false;
    bool            m_show_lrside = false;
    double          m_node_radius = 3;
    double          m_edge_thickness = 2;

    cv::Point       m_view_offset = cv::Point(0, 0);
    cv::Size        m_view_size = cv::Size(1800, 1012);
    dg::Viewport    m_viewport;

    // interactive edit
    enum {G_NODE, G_POI, G_STREETVIEW};
    cv::Point       m_mouse_pt;
    cv::Point2d     m_mouse_xy;
    bool            m_mouse_drag = false;
    dg::ID          m_gobj_id;                  // selected object for drag or edit
    int             m_gobj_type;                // type of selected object
    dg::ID          m_gobj_highlight_nid = 0;   // nearby object 
    dg::ID          m_gobj_from = 0;            // selected first object for edge connecting

    // error check
    bool m_show_map_error = false;
    std::vector<dg::Node*> m_error_nodes;
    std::vector<dg::Edge*> m_error_edges;

}; // End of 'MapEditor'


#endif // End of '__MAP_EDITOR__'
