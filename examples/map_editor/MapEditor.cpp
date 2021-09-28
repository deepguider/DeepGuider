#include "framework.h"
#include "MapEditorApp.h"
#include "DlgNode.h"
#include "DlgEdge.h"
#include "DlgPOI.h"
#include "DlgView.h"
#include "afxdialogex.h"
#include "MapEditor.h"
#include "map_manager/map_manager.hpp"
#include <locale>
#include <codecvt>
#include <wchar.h>


#define MAX_PATH_LEN 512

void onMouseEvent(int event, int x, int y, int flags, void* param)
{
    MapEditor* editor = (MapEditor*)param;
    editor->procMouseEvent(event, x, y, flags);
}

int MapEditor::configure(std::string site)
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
    COEX.map_file = "data/COEX/TopoMap_COEX.csv";
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
    Bucheon.map_file = "data/Bucheon/TopoMap_Bucheon.csv";
    Bucheon.video_resize = 0.25;
    Bucheon.video_offset = cv::Point(270, 638);
    Bucheon.result_resize = 0.4;

    MapGUIProp guiprop = (site == "coex") ? COEX : (site == "bucheon") ? Bucheon : ETRI;
    m_guiprop = guiprop;
    m_site = site;

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

int MapEditor::run()
{
    if (m_bg_image.empty()) return -1;
    cv::AutoLock lock(m_mutex_run);
    m_running = true;
    m_stop_running = false;

    m_viewport.initialize(m_bg_image, m_view_size, m_view_offset);
    cv::namedWindow(m_winname, cv::WindowFlags::WINDOW_NORMAL);
    cv::setWindowTitle(m_winname, m_site);
    cv::resizeWindow(m_winname, m_viewport.size());
    cv::setMouseCallback(m_winname, onMouseEvent, this);

    cv::Mat view_image;
    while (!m_stop_running)
    {
        m_viewport.getViewportImage(view_image);
        cv::imshow(m_winname, view_image);
        int key = cv::waitKey(10);
    }
    m_running = false;

    return 0;
}

void MapEditor::stop()
{
    m_stop_running = true;
    cv::AutoLock lock(m_mutex_run);
}

void MapEditor::procMouseEvent(int evt, int x, int y, int flags)
{
    if (m_under_edit) return;
    bool is_shift = flags & cv::EVENT_FLAG_SHIFTKEY;
    double zoom_prev = m_viewport.zoom();
    if(!is_shift && !m_mouse_drag) m_viewport.procMouseEvent(evt, x, y, flags);
    double zoom = m_viewport.zoom();
    if (zoom != zoom_prev)
    {
        if (zoom >= 4)
        {
            m_painter.setParamValue("node_radius", 2);
            m_painter.setParamValue("edge_thickness", 1);
        }
        else
        {
            m_painter.setParamValue("node_radius", 3);
            m_painter.setParamValue("edge_thickness", 2);
        }
        drawMap();
    }

    if (evt == cv::EVENT_MOUSEMOVE && m_mouse_drag)
    {
        if (x != m_mouse_pt.x || y != m_mouse_pt.y)
        {
            cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
            cv::Point2d metric = m_painter.cvtPixel2Value(px);
            if (m_gobj_type == G_NODE)
            {
                dg::Node* node = m_map.getNode(m_gobj_id);
                if (node)
                {
                    node->x = metric.x;
                    node->y = metric.y;
                    for (auto it = node->edge_ids.begin(); it != node->edge_ids.end(); it++)
                    {
                        dg::Edge* edge = m_map.getEdge(*it);
                        if (edge)
                        {
                            dg::Node* node2 = m_map.getConnectedNode(node, edge->id);
                            if (node2) edge->length = norm(*node - *node2);
                        }
                    }
                }
            }
            else if (m_gobj_type == G_POI)
            {
                dg::POI* poi = m_map.getPOI(m_gobj_id);
                if (poi)
                {
                    poi->x = metric.x;
                    poi->y = metric.y;
                }
            }
            else if (m_gobj_type == G_STREETVIEW)
            {
                dg::StreetView* sv = m_map.getView(m_gobj_id);
                if (sv)
                {
                    sv->x = metric.x;
                    sv->y = metric.y;
                }
            }
            m_mouse_pt = cv::Point(x, y);

            drawMap();
        }
    }
    else if (evt == cv::EVENT_LBUTTONDOWN && is_shift)
    {
        cv::AutoLock lock(m_mutex_data);
        double dist_thr = 10;    // meter
        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d metric = m_painter.cvtPixel2Value(px);

        bool update_gobj = false;
        if (m_show_poi || m_show_streetview)
        {
            std::vector<dg::POI*> pois;
            std::vector<dg::StreetView*> streetviews;
            if (m_show_poi) pois = m_map.getNearPOIs(metric, dist_thr, true);
            if (m_show_streetview) streetviews = m_map.getNearViews(metric, dist_thr, true);
            if (!pois.empty() && (streetviews.empty() || norm(*(pois[0]) - metric) <= norm(*(streetviews[0]) - metric)))
            {
                dg::POI* poi = pois[0];
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawPoint(view_image, *poi, 4, cv::Vec3b(0, 0, 128), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                m_gobj_id = poi->id;
                m_gobj_type = G_POI;
                update_gobj = true;
            }
            if (!streetviews.empty() && (pois.empty() || norm(*(pois[0]) - metric) > norm(*(streetviews[0]) - metric)))
            {
                dg::StreetView* sv = streetviews[0];
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawPoint(view_image, *sv, 4, cv::Vec3b(128, 0, 0), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                m_gobj_id = sv->id;
                m_gobj_type = G_STREETVIEW;
                update_gobj = true;
            }
        }
        else
        {
            dg::Node* node = m_map.getNearestNode(metric);
            double d1 = norm(*node - metric);
            bool update_node = node && d1 <= dist_thr;
            if (update_node)
            {
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawNode(view_image, *node, 4, 0, cv::Vec3b(0, 255, 255), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                m_gobj_id = node->id;
                m_gobj_type = G_NODE;
                update_gobj = true;
            }
        }

        if (update_gobj)
        {
            m_mouse_pt = cv::Point(x, y);
            m_mouse_drag = true;
        }
    }
    else if (evt == cv::EVENT_LBUTTONDOWN)
    {
        m_mouse_drag = false;
    }
    else if (evt == cv::EVENT_LBUTTONUP)
    {
        m_mouse_drag = false;
    }
    else if (evt == cv::EVENT_LBUTTONDBLCLK)
    {
        m_under_edit = true;
        cv::AutoLock lock(m_mutex_data);

        double dist_thr = 10;    // meter
        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d metric = m_painter.cvtPixel2Value(px);

        bool update_gui = false;
        if (m_show_poi || m_show_streetview)
        {
            std::vector<dg::POI*> pois;
            std::vector<dg::StreetView*> streetviews;
            if (m_show_poi) pois = m_map.getNearPOIs(metric, dist_thr, true);
            if (m_show_streetview) streetviews = m_map.getNearViews(metric, dist_thr, true);
            if (!pois.empty() && (streetviews.empty() || norm(*(pois[0])-metric) <= norm(*(streetviews[0])-metric)))
            {
                dg::POI* poi = pois[0];
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawPoint(view_image, *poi, 4, cv::Vec3b(0, 0, 128), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                dg::LatLon ll = m_map.toLatLon(*poi);
                DlgPOI dlg;
                dlg.ID = poi->id;
                dlg.lat = ll.lat;
                dlg.lon = ll.lon;
                dlg.floor = poi->floor;
                dlg.name = poi->name.c_str();
                if (dlg.DoModal() == IDOK)
                {
                    dg::Point2 xy = m_map.toMetric(dg::LatLon(dlg.lat, dlg.lon));
                    poi->id = dlg.ID;
                    poi->x = xy.x;
                    poi->y = xy.y;
                    poi->floor = dlg.floor;
                    poi->name = CT2CW(dlg.name);
                }
            }
            if (!streetviews.empty() && (pois.empty() || norm(*(pois[0]) - metric) > norm(*(streetviews[0]) - metric)))
            {
                dg::StreetView* sv = streetviews[0];
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawPoint(view_image, *sv, 4, cv::Vec3b(128, 0, 0), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                dg::LatLon ll = m_map.toLatLon(*sv);
                DlgView dlg;
                dlg.ID = sv->id;
                dlg.lat = ll.lat;
                dlg.lon = ll.lon;
                dlg.floor = sv->floor;
                dlg.heading = sv->heading;
                dlg.date = sv->date.c_str();
                if (dlg.DoModal() == IDOK)
                {
                    dg::Point2 xy = m_map.toMetric(dg::LatLon(dlg.lat, dlg.lon));
                    sv->id = dlg.ID;
                    sv->x = xy.x;
                    sv->y = xy.y;
                    sv->floor = dlg.floor;
                    sv->heading = dlg.heading;
                    sv->date = CT2A(dlg.date);
                }
            }
        }
        else
        {
            dg::Node* node = m_map.getNearestNode(metric);
            double d1 = norm(*node - metric);
            dg::Point2 ep;
            dg::Edge* edge = m_map.getNearestEdge(metric, ep);
            double d2 = norm(ep - metric);

            bool update_node = node && d1 <= dist_thr && (d1 < dist_thr / 2 || d1 <= d2);
            if (update_node && d2 <= d1 && edge->length < dist_thr * 1.5 && d1 > edge->length / 3) update_node = false;
            if (update_node)
            {
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawNode(view_image, *node, 4, 0, cv::Vec3b(0, 255, 255), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                dg::LatLon ll = m_map.toLatLon(*node);
                DlgNode dlg;
                dlg.ID = node->id;
                dlg.type = node->type;
                dlg.lat = ll.lat;
                dlg.lon = ll.lon;
                dlg.edge_ids = node->edge_ids;
                dlg.floor = node->floor;
                if (dlg.DoModal() == IDOK)
                {
                    if (node->type != dlg.type) update_gui = true;

                    dg::Point2 xy = m_map.toMetric(dg::LatLon(dlg.lat, dlg.lon));
                    node->id = dlg.ID;
                    node->type = dlg.type;
                    node->x = xy.x;
                    node->y = xy.y;
                    node->floor = dlg.floor;
                    node->edge_ids = dlg.edge_ids;
                }
            }
            else if (edge && d2 <= dist_thr)
            {
                dg::Node* from = m_map.getNode(edge->node_id1);
                dg::Node* to = m_map.getNode(edge->node_id2);
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawEdge(view_image, *from, *to, 3, cv::Vec3b(0, 255, 255), 3, m_viewport.offset(), m_viewport.zoom(), 4);
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                DlgEdge dlg;
                dlg.ID = edge->id;
                dlg.type = edge->type;
                dlg.length = edge->length;
                dlg.lr_side = edge->lr_side;
                dlg.directed = (edge->directed == true);
                dlg.node_id1 = edge->node_id1;
                dlg.node_id2 = edge->node_id2;
                if (dlg.DoModal() == IDOK)
                {
                    if (edge->type != dlg.type || edge->lr_side != dlg.lr_side) update_gui = true;

                    edge->id = dlg.ID;
                    edge->type = dlg.type;
                    edge->length = dlg.length;
                    edge->lr_side = dlg.lr_side;
                    edge->node_id1 = dlg.node_id1;
                    edge->node_id2 = dlg.node_id2;
                    edge->directed = (dlg.directed == TRUE);
                }
            }
        }

        if (update_gui)
        {
            drawMap();
        }

        m_under_edit = false;
    }
    else if (evt == cv::EVENT_RBUTTONDOWN)
    {
        if (m_show_streetview)
        {
            m_under_edit = true;
            cv::AutoLock lock(m_mutex_data);

            double dist_thr = 10;    // meter
            cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
            cv::Point2d metric = m_painter.cvtPixel2Value(px);

            std::vector<dg::StreetView*> streetviews = m_map.getNearViews(metric, dist_thr, true);
            if (!streetviews.empty())
            {
                dg::StreetView* sv = streetviews[0];
                cv::Mat view_image;
                double zoom = m_viewport.zoom();
                m_viewport.getViewportImage(view_image);
                m_painter.drawPoint(view_image, *sv, 4, cv::Vec3b(128, 0, 0), m_viewport.offset(), zoom, (int)(2 * zoom + 0.5));
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                cv::Mat sv_front;
                dg::MapManager::getStreetViewImage(sv->id, sv_front, "f");
                if (!sv_front.empty())
                {
                    if (flags & cv::EVENT_FLAG_SHIFTKEY)
                    {
                        cv::Mat sv_image;
                        dg::MapManager::getStreetViewImage(sv->id, sv_image);
                        if (!sv_image.empty()) cv::namedWindow("streetview", cv::WINDOW_NORMAL);
                        if (!sv_image.empty()) cv::imshow("streetview", sv_image);
                    }
                    std::string str_id = cv::format("ID: %zu", sv->id);
                    std::string str_heading = cv::format("Heading: %.0f degree", sv->heading);
                    cv::putText(sv_front, str_id.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 5);
                    cv::putText(sv_front, str_id.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
                    cv::putText(sv_front, str_heading.c_str(), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 5);
                    cv::putText(sv_front, str_heading.c_str(), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
                    cv::namedWindow("streetview_front", cv::WINDOW_NORMAL);
                    cv::imshow("streetview_front", sv_front);
                    int key = cv::waitKey(1);
                }
            }
            m_under_edit = false;
        }
    }
    else if (evt == cv::EVENT_RBUTTONUP)
    {
    }
    else if (evt == cv::EVENT_MOUSEWHEEL)
    {
    }
}


void MapEditor::save()
{
    cv::AutoLock lock(m_mutex_data);
    if(!m_fpath.empty()) m_map.save(m_fpath.c_str());
}

void MapEditor::saveAs()
{
    char szFile[MAX_PATH_LEN] = "";  // buffer for file path
    CString szFilter = "CSV Files (*.csv)|*.csv|All Files (*.*)|*.*||";
    CFileDialog dlg(FALSE, 0, szFile, OFN_OVERWRITEPROMPT, szFilter, NULL);
    if (dlg.DoModal() == IDOK)
    {
        cv::AutoLock lock(m_mutex_data);
        m_fpath = CT2A(dlg.GetPathName().GetString());
        m_map.save(m_fpath.c_str());
    }
}

void MapEditor::load()
{
    char szFile[MAX_PATH_LEN] = "";  // buffer for file path
    CString szFilter = "CSV Files (*.csv)|*.csv|All Files (*.*)|*.*||";
    CFileDialog dlg(TRUE, 0, szFile, OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST, szFilter, NULL);
    if (dlg.DoModal() == IDOK)
    {
        cv::AutoLock lock(m_mutex_data);
        m_fpath = CT2A(dlg.GetPathName().GetString());
        m_map.load(m_fpath.c_str());
        if (m_map.isEmpty()) m_fpath.clear();
        drawMap();
    }
}

void MapEditor::download()
{
    cv::AutoLock lock(m_mutex_data);
    dg::MapManager manager;
    if (!manager.initialize("129.254.81.204")) return;
    dg::LatLon ll = m_guiprop.origin_latlon;
    manager.setReference(ll);
    double radius = 5000;  // 5 km
    if (manager.getMapAll(ll.lat, ll.lon, radius, m_map))
    {
        drawMap();
    }
}

void MapEditor::showPoi(bool show)
{
    cv::AutoLock lock(m_mutex_data);
    m_show_poi = show;
    drawMap();
}

void MapEditor::showStreetView(bool show)
{
    cv::AutoLock lock(m_mutex_data);
    m_show_streetview = show;
    drawMap();
}

void MapEditor::drawMap()
{
    m_bg_image_original.copyTo(m_bg_image);
    m_painter.drawGrid(m_bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, m_guiprop.grid_unit_pos);
    m_painter.drawOrigin(m_bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    if (m_map.isEmpty()) return;
    if (!m_show_poi && !m_show_streetview)
    {
        m_painter.drawMap(m_bg_image, &m_map);
        for (auto it = m_map.getHeadEdge(); it != m_map.getTailEdge(); it++)
        {
            if (it->lr_side == dg::Edge::LR_LEFT)
            {
                dg::Node* n1 = m_map.getNode(it->node_id1);
                dg::Node* n2 = m_map.getNode(it->node_id2);
                if (n1 == nullptr || n2 == nullptr) continue;
                dg::Point2 p1(n2->y - n1->y, n1->x - n2->x);    // rotate n1n2 by -90 degree
                p1 = *n1 + p1 * 4 / norm(p1);
                dg::Point2 p2 = p1 + (*n2 - *n1);
                m_painter.drawEdge(m_bg_image, p1, p2, 3, cv::Vec3b(0, 255, 0), 2);
            }
            if (it->lr_side == dg::Edge::LR_RIGHT)
            {
                dg::Node* n1 = m_map.getNode(it->node_id1);
                dg::Node* n2 = m_map.getNode(it->node_id2);
                if (n1 == nullptr || n2 == nullptr) continue;
                dg::Point2 p1(n1->y - n2->y, n2->x - n1->x);    // rotate n1n2 by 90 degree
                p1 = *n1 + p1 * 4 / norm(p1);
                dg::Point2 p2 = p1 + (*n2 - *n1);
                m_painter.drawEdge(m_bg_image, p1, p2, 3, cv::Vec3b(0, 255, 0), 2);
            }
        }
    }
    if (m_show_poi)
    {
        double zoom = m_viewport.zoom();
        int radius = (zoom >= 4) ? 2 : 3;
        for (auto it = m_map.getHeadPOI(); it != m_map.getTailPOI(); it++)
        {
            m_painter.drawPoint(m_bg_image, *it, radius, cv::Vec3b(0, 0, 255));
        }
    }
    if (m_show_streetview)
    {
        double zoom = m_viewport.zoom();
        int radius = (zoom >= 4) ? 2 : 3;
        for (auto it = m_map.getHeadView(); it != m_map.getTailView(); it++)
        {
            m_painter.drawPoint(m_bg_image, *it, radius, cv::Vec3b(255, 0, 0));
        }
    }
}

