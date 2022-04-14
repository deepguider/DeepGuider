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
    ETRI.server_port = "10000";
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
    COEX.server_port = "10001";
    COEX.image_file = "data/NaverMap_COEX(Satellite)_200929.png";
    COEX.image_scale = cv::Point2d(1.055, 1.055);
    COEX.image_rotation = cx::cvtDeg2Rad(1.2);
    COEX.origin_latlon = dg::LatLon(37.506207, 127.05482);
    COEX.origin_px = cv::Point2d(1090, 1018);
    COEX.map_radius = 2000; // meter
    COEX.grid_unit_pos = cv::Point(-230, -16);
    COEX.map_file = "data/COEX/TopoMap_COEX.csv";
    COEX.video_resize = 0.4;
    COEX.video_offset = cv::Point(10, 50);
    COEX.result_resize = 0.6;

    MapGUIProp COEX2;
    COEX2.server_port = "10001";
    COEX2.image_file = "data/COEX/NaverMap_COEX(Satellite).png";
    COEX2.image_scale = cv::Point2d(2.536, 2.536);
    COEX2.image_rotation = cx::cvtDeg2Rad(1.0);
    COEX2.origin_latlon = dg::LatLon(37.506994, 127.056676);
    COEX2.origin_px = cv::Point2d(1373, 2484);
    COEX2.map_view_offset = cv::Point(1010, 300);
    COEX2.map_radius = 2000; // meter
    COEX2.grid_unit_pos = cv::Point(-230, -16);
    COEX2.map_file = "data/COEX/TopoMap_COEX.csv";
    COEX2.video_resize = 0.4;
    COEX2.video_offset = cv::Point(10, 50);
    COEX2.result_resize = 0.6;

    MapGUIProp Bucheon;
    Bucheon.server_port = "10002";
    Bucheon.image_file = "data/NaverMap_Bucheon(Satellite).png";
    Bucheon.image_scale = cv::Point2d(1.056, 1.056);
    Bucheon.image_rotation = cx::cvtDeg2Rad(0);
    Bucheon.origin_latlon = dg::LatLon(37.510928, 126.764344);
    Bucheon.origin_px = cv::Point2d(1535, 1157);
    Bucheon.map_radius = 2000; // meter
    Bucheon.grid_unit_pos = cv::Point(-215, -6);
    Bucheon.map_file = "data/Bucheon/TopoMap_Bucheon.csv";
    Bucheon.video_resize = 0.25;
    Bucheon.video_offset = cv::Point(270, 638);
    Bucheon.result_resize = 0.4;

    MapGUIProp guiprop = (site == "coex") ? COEX2 : (site == "bucheon") ? Bucheon : ETRI;
    m_guiprop = guiprop;
    m_site = site;

    // Prepare a map if given
    m_fpath = guiprop.map_file;
    m_map.setReference(guiprop.origin_latlon);
    if (!guiprop.map_file.empty())
    {
        m_map.load(guiprop.map_file.c_str());
        initializeNextMapID();
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
    m_painter.setParamValue("node_radius", m_node_radius);
    m_painter.setParamValue("node_font_scale", 0);
    m_painter.setParamValue("node_color", { 255, 50, 255 });
    m_painter.setParamValue("junction_color", { 255, 250, 0 });
    m_painter.setParamValue("edge_color", { 0, 255, 255 });
    m_painter.setParamValue("sidewalk_color", { 200, 100, 100 });
    m_painter.setParamValue("crosswalk_color", { 0, 150, 50 });
    m_painter.setParamValue("mixedroad_color", { 0, 0, 255 });
    m_painter.setParamValue("edge_thickness", m_edge_thickness);
    m_painter.setParamValue("color_whitening", 1);

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
    m_viewport.setZoomRange(0.5, 8);
    cv::namedWindow(m_winname, cv::WindowFlags::WINDOW_NORMAL);
    cv::setWindowTitle(m_winname, m_site);
    cv::resizeWindow(m_winname, m_viewport.size());
    cv::setMouseCallback(m_winname, onMouseEvent, this);

    cv::Mat view_image;
    while (!m_stop_running)
    {
        cv::AutoLock lock(m_mutex_data);
        m_viewport.getViewportImage(view_image);
        drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
        if (m_gobj_highlight_nid > 0 || m_gobj_from > 0)
        {
            dg::Node* node = (m_gobj_highlight_nid > 0) ? m_map.getNode(m_gobj_highlight_nid) : m_map.getNode(m_gobj_from);
            double zoom = m_viewport.zoom();
            double scale_modifier = (zoom >= 4) ? zoom / 2 : 1;
            double pt_radius = (m_node_radius + zoom) / scale_modifier;
            double ln_thickness = (m_edge_thickness + 2) / scale_modifier;
            int pt_thickness = (int)(ln_thickness + 0.5);
            m_painter.drawNode(view_image, *node, pt_radius, 0, cv::Vec3b(0, 255, 255), m_viewport.offset(), zoom, pt_thickness);
        }
        cv::putText(view_image, cv::format("(%.2lf, %.2lf)", m_mouse_xy.x, m_mouse_xy.y), cv::Point(10, 35), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 0, 0), 3);
        cv::putText(view_image, cv::format("(%.2lf, %.2lf)", m_mouse_xy.x, m_mouse_xy.y), cv::Point(10, 35), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 0), 1);
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
    bool is_ctrl = flags & cv::EVENT_FLAG_CTRLKEY;
    if (!is_shift && !is_ctrl && !m_mouse_drag) m_viewport.procMouseEvent(evt, x, y, flags);

    double zoom = m_viewport.zoom();
    double scale_modifier = (zoom >= 4) ? zoom / 2 : 1;
    double ln_thickness = (m_edge_thickness + 2) / scale_modifier;
    double pt_radius = (m_node_radius + zoom) / scale_modifier;
    int pt_thickness = (int)(ln_thickness + 0.5);
    double dist_thr = (zoom >= 4) ? (2 + 24 / zoom) : 10;   // meter

    if (evt == cv::EVENT_MOUSEMOVE)
    {
        if (x != m_mouse_pt.x || y != m_mouse_pt.y)
        {
            cv::AutoLock lock(m_mutex_data);
            cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
            m_mouse_xy = m_painter.cvtPixel2Value(px);
        }
    }
    if (evt == cv::EVENT_MOUSEMOVE && m_mouse_drag)
    {
        if (x != m_mouse_pt.x || y != m_mouse_pt.y)
        {
            cv::AutoLock lock(m_mutex_data);
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

                    if (node->edge_ids.size() == 1)
                    {
                        dg::Node* node2 = m_map.getNearestNode(node->id);
                        if (norm(*node2 - metric) <= dist_thr) m_gobj_highlight_nid = node2->id;
                        else m_gobj_highlight_nid = 0;
                    }
                    else
                        m_gobj_highlight_nid = 0;
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
        }
    }
    else if (evt == cv::EVENT_LBUTTONDOWN && is_shift)
    {
        cv::AutoLock lock(m_mutex_data);
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
                m_gobj_id = pois[0]->id;
                m_gobj_type = G_POI;
                update_gobj = true;
            }
            if (!streetviews.empty() && (pois.empty() || norm(*(pois[0]) - metric) > norm(*(streetviews[0]) - metric)))
            {
                m_gobj_id = streetviews[0]->id;
                m_gobj_type = G_STREETVIEW;
                update_gobj = true;
            }
        }
        else
        {
            dg::Node* node = m_map.getNearestNode(metric);
            if (node && norm(*node - metric) <= dist_thr)
            {
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
        m_gobj_from = 0;
        m_gobj_highlight_nid = 0;
    }
    else if (evt == cv::EVENT_LBUTTONDOWN && is_ctrl && !m_show_poi && !m_show_streetview)
    {
        cv::AutoLock lock(m_mutex_data);
        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d metric = m_painter.cvtPixel2Value(px);

        dg::Node* node = m_map.getNearestNode(metric);
        if (node && norm(*node - metric) <= dist_thr)
        {
            if (m_gobj_from > 0)
            {
                dg::Edge edge;
                edge.id = getNextMapID();
                edge.type = 0;
                edge.node_id1 = m_gobj_from;
                edge.node_id2 = node->id;
                m_map.addEdge(edge, true);

                dg::Node* node1 = m_map.getNode(edge.node_id1);
                dg::Node* node2 = m_map.getNode(edge.node_id2);
                if (node1->edge_ids.size() >= 3 && node1->type == dg::Node::NODE_BASIC) node1->type = dg::Node::NODE_JUNCTION;
                if (node2->edge_ids.size() >= 3 && node2->type == dg::Node::NODE_BASIC) node2->type = dg::Node::NODE_JUNCTION;

                m_gobj_from = 0;
            }
            else
            {
                m_gobj_from = node->id;
            }
        }
        m_mouse_drag = false;
        m_gobj_highlight_nid = 0;
    }
    else if (evt == cv::EVENT_LBUTTONDOWN)
    {
        m_mouse_drag = false;
        m_gobj_highlight_nid = 0;
    }
    else if (evt == cv::EVENT_LBUTTONUP)
    {
        if (is_shift && m_mouse_drag && m_gobj_type == G_NODE)
        {
            cv::AutoLock lock(m_mutex_data);
            cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
            cv::Point2d metric = m_painter.cvtPixel2Value(px);
            dg::Node* node = m_map.getNode(m_gobj_id);
            if (node && node->edge_ids.size() == 1)
            {
                dg::Node* node2 = m_map.getNearestNode(node->id);
                double d1 = norm(*node2 - metric);
                if (d1 < dist_thr)
                {
                    node2->edge_ids.push_back(node->edge_ids[0]);
                    dg::Edge* edge = m_map.getEdge(node->edge_ids[0]);
                    if (edge->node_id1 == node->id) edge->node_id1 = node2->id;
                    else edge->node_id2 = node2->id;
                    if (edge && edge->type == dg::Edge::EDGE_CROSSWALK) node2->type = dg::Node::NODE_JUNCTION;
                    if (node2->edge_ids.size() >= 3 && node2->type == dg::Node::NODE_BASIC) node2->type = dg::Node::NODE_JUNCTION;
                    node->edge_ids.clear();
                    m_map.deleteNode(node->id);
                }
            }
        }
        m_mouse_drag = false;
        m_gobj_highlight_nid = 0;
    }
    else if (evt == cv::EVENT_LBUTTONDBLCLK && is_ctrl)
    {
        cv::AutoLock lock(m_mutex_data);
        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d metric = m_painter.cvtPixel2Value(px);

        if (!m_show_poi && !m_show_streetview)
        {
            dg::Point2 ep;
            dg::Edge* edge = m_map.getNearestEdge(metric, ep);
            if (edge && norm(ep - metric) < dist_thr)
            {
                dg::Node* node1 = m_map.getNode(edge->node_id1);
                dg::Node* node2 = m_map.getNode(edge->node_id2);
                dg::Node n(getNextMapID(), ep, 0, node1->floor);
                m_map.addNode(n);

                dg::Edge e1 = *edge;
                e1.id = getNextMapID();
                e1.node_id1 = edge->node_id1;
                e1.node_id2 = n.id;
                dg::Edge e2 = *edge;
                e2.id = getNextMapID();
                e2.node_id1 = n.id;
                e2.node_id2 = edge->node_id2;
                m_map.deleteEdge(edge->id);
                m_map.addEdge(e1, true);
                m_map.addEdge(e2, true);
            }
            else
            {
                dg::Node n(getNextMapID(), metric);
                m_map.addNode(n);
            }
        }
        if (m_show_poi)
        {
            dg::POI poi(getNextMapID(), metric, L"", 0);
            cv::Mat view_image;
            m_viewport.getViewportImage(view_image);
            drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
            double node_radius = m_node_radius / scale_modifier;
            m_painter.drawPoint(view_image, poi, node_radius, cv::Vec3b(0, 0, 255), m_viewport.offset(), zoom);
            m_painter.drawPoint(view_image, poi, pt_radius, cv::Vec3b(0, 0, 128), m_viewport.offset(), zoom, pt_thickness);
            cv::imshow(m_winname, view_image);
            int key = cv::waitKey(1);

            dg::LatLon ll = m_map.toLatLon(poi);
            DlgPOI dlg;
            dlg.ID = poi.id;
            dlg.lat = ll.lat;
            dlg.lon = ll.lon;
            dlg.floor = poi.floor;
            dlg.name = poi.name.c_str();
            if (dlg.DoModal() == IDOK && !dlg.erase && !dlg.name.IsEmpty())
            {
                dg::Point2 xy = m_map.toMetric(dg::LatLon(dlg.lat, dlg.lon));
                poi.id = dlg.ID;
                poi.x = xy.x;
                poi.y = xy.y;
                poi.floor = dlg.floor;
                poi.name = CT2CW(dlg.name);
                m_map.addPOI(poi);
            }
        }

        m_gobj_from = 0;
        m_mouse_drag = false;
        m_gobj_highlight_nid = 0;
    }
    else if (evt == cv::EVENT_LBUTTONDBLCLK)
    {
        m_under_edit = true;
        cv::AutoLock lock(m_mutex_data);

        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d metric = m_painter.cvtPixel2Value(px);

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
                m_viewport.getViewportImage(view_image);
                drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
                m_painter.drawPoint(view_image, *poi, pt_radius, cv::Vec3b(0, 0, 128), m_viewport.offset(), zoom, pt_thickness);
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                dg::LatLon ll = m_map.toLatLon(*poi);
                DlgPOI dlg;
                dlg.map = &m_map;
                dlg.ID = poi->id;
                dlg.lat = ll.lat;
                dlg.lon = ll.lon;
                dlg.floor = poi->floor;
                dlg.name = poi->name.c_str();
                if (dlg.DoModal() == IDOK)
                {
                    if (dlg.erase)
                    {
                        m_map.deletePOI(poi->id);
                    }
                    else
                    {
                        dg::Point2 xy = m_map.toMetric(dg::LatLon(dlg.lat, dlg.lon));
                        poi->id = dlg.ID;
                        poi->x = xy.x;
                        poi->y = xy.y;
                        poi->floor = dlg.floor;
                        poi->name = CT2CW(dlg.name);
                    }
                }
            }
            if (!streetviews.empty() && (pois.empty() || norm(*(pois[0]) - metric) > norm(*(streetviews[0]) - metric)))
            {
                dg::StreetView* sv = streetviews[0];
                cv::Mat view_image;
                m_viewport.getViewportImage(view_image);
                drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
                m_painter.drawPoint(view_image, *sv, pt_radius, cv::Vec3b(128, 0, 0), m_viewport.offset(), zoom, pt_thickness);
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
                    if (dlg.erase)
                    {
                        m_map.deleteView(sv->id);
                    }
                    else
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
        }
        else
        {
            dg::Node* node = m_map.getNearestNode(metric);
            double d1 = norm(*node - metric);
            dg::Point2 ep;
            dg::Edge* edge = m_map.getNearestEdge(metric, ep);
            double d2 = norm(ep - metric);
            double d1_prj = norm(*node - ep);

            bool update_node = node && d1 <= dist_thr;
            if (update_node && d2 < d1 && d2 < d1_prj)
            {
                if (d1_prj > edge->length / 3) update_node = false;
                if (d1_prj > dist_thr / 2) update_node = false;
            }
            if (update_node)
            {
                cv::Mat view_image;
                m_viewport.getViewportImage(view_image);
                drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
                m_painter.drawNode(view_image, *node, pt_radius, 0, cv::Vec3b(0, 255, 255), m_viewport.offset(), zoom, pt_thickness);
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
                    if (dlg.erase)
                    {
                        m_map.deleteNode(node->id);
                    }
                    else
                    {
                        dg::Point2 xy = m_map.toMetric(dg::LatLon(dlg.lat, dlg.lon));
                        node->id = dlg.ID;
                        node->type = dlg.type;
                        node->x = xy.x;
                        node->y = xy.y;
                        node->floor = dlg.floor;
                        node->edge_ids = dlg.edge_ids;
                    }
                }
            }
            else if (edge && d2 <= dist_thr)
            {
                dg::Node* from = m_map.getNode(edge->node_id1);
                dg::Node* to = m_map.getNode(edge->node_id2);
                cv::Mat view_image;
                m_viewport.getViewportImage(view_image);
                drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
                double node_radius = (m_node_radius + 1) / scale_modifier;
                m_painter.drawEdge(view_image, *from, *to, node_radius, cv::Vec3b(0, 255, 255), ln_thickness, m_viewport.offset(), m_viewport.zoom(), 4);
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
                    if (dlg.erase)
                    {
                        m_map.deleteEdge(edge->id);
                    }
                    else
                    {
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
        }
        m_gobj_from = 0;
        m_mouse_drag = false;
        m_gobj_highlight_nid = 0;

        m_under_edit = false;
    }
    else if (evt == cv::EVENT_RBUTTONDOWN)
    {
        if (m_show_streetview)
        {
            m_under_edit = true;
            cv::AutoLock lock(m_mutex_data);

            cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
            cv::Point2d metric = m_painter.cvtPixel2Value(px);

            std::vector<dg::StreetView*> streetviews = m_map.getNearViews(metric, dist_thr, true);
            if (!streetviews.empty())
            {
                dg::StreetView* sv = streetviews[0];
                cv::Mat view_image;
                m_viewport.getViewportImage(view_image);
                drawMap(view_image, m_viewport.offset(), m_viewport.zoom());
                m_painter.drawPoint(view_image, *sv, pt_radius, cv::Vec3b(128, 0, 0), m_viewport.offset(), zoom, pt_thickness);
                cv::imshow(m_winname, view_image);
                int key = cv::waitKey(1);

                cv::Mat sv_front;
                dg::MapManager::getStreetViewImage(sv->id, sv_front, m_guiprop.server_ip, m_guiprop.server_port, "f");
                if (!sv_front.empty())
                {
                    if (flags & cv::EVENT_FLAG_SHIFTKEY)
                    {
                        cv::Mat sv_image;
                        dg::MapManager::getStreetViewImage(sv->id, sv_image, m_guiprop.server_ip, m_guiprop.server_port);
                        if (!sv_image.empty()) cv::namedWindow("streetview", cv::WINDOW_NORMAL);
                        if (!sv_image.empty()) cv::setWindowProperty("streetview", 5, 1); // cv::WND_PROP_TOPMOST = 5
                        if (!sv_image.empty()) cv::imshow("streetview", sv_image);
                    }
                    std::string str_id = cv::format("ID: %zu", sv->id);
                    std::string str_heading = cv::format("Heading: %.0f degree", sv->heading);
                    cv::putText(sv_front, str_id.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 5);
                    cv::putText(sv_front, str_id.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
                    cv::putText(sv_front, str_heading.c_str(), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 5);
                    cv::putText(sv_front, str_heading.c_str(), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
                    cv::namedWindow("streetview_front", cv::WINDOW_NORMAL);
                    cv::setWindowProperty("streetview_front", 5, 1); // cv::WND_PROP_TOPMOST = 5
                    cv::imshow("streetview_front", sv_front);
                    int key = cv::waitKey(1);
                }
            }
            m_under_edit = false;
        }

        m_gobj_from = 0;
        m_mouse_drag = false;
        m_gobj_highlight_nid = 0;
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
    if (!m_fpath.empty()) m_map.save(m_fpath.c_str());
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
        initializeNextMapID();
        if (m_map.isEmpty()) m_fpath.clear();
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
    manager.getMapAll(ll.lat, ll.lon, radius, m_map);
    initializeNextMapID();
}

void MapEditor::showPoi(bool show)
{
    m_show_poi = show;
}

void MapEditor::showStreetView(bool show)
{
    m_show_streetview = show;
}

void MapEditor::showLRSide(bool show)
{
    m_show_lrside = show;
}

void MapEditor::showMapError(bool show)
{
    m_show_map_error = show;
}

void MapEditor::drawMap(cv::Mat view_image, cv::Point2d offset, double zoom)
{
    if (m_map.isEmpty()) return;

    double scale_modifier = (zoom >= 4) ? zoom / 2 : 1;
    double node_radius = m_node_radius / scale_modifier;
    double edge_thickness = m_edge_thickness / scale_modifier;
    m_painter.setParamValue("node_radius", node_radius);
    m_painter.setParamValue("edge_thickness", edge_thickness);
    if (!m_show_poi && !m_show_streetview)
    {
        m_painter.setParamValue("color_whitening", 1);
        m_painter.drawMap(view_image, &m_map, offset, zoom);

        if (m_show_lrside)
        {
            for (auto it = m_map.getHeadEdge(); it != m_map.getTailEdge(); it++)
            {
                if (it->lr_side == dg::Edge::LR_LEFT)
                {
                    dg::Node* n1 = m_map.getNode(it->node_id1);
                    dg::Node* n2 = m_map.getNode(it->node_id2);
                    if (n1 == nullptr || n2 == nullptr) continue;
                    dg::Point2 p1(n2->y - n1->y, n1->x - n2->x);    // rotate n1n2 by -90 degree
                    p1 = *n1 + p1 * 2 / norm(p1);
                    dg::Point2 p2 = p1 + (*n2 - *n1);
                    m_painter.drawEdge(view_image, p1, p2, node_radius, cv::Vec3b(0, 255, 0), edge_thickness, offset, zoom);
                }
                if (it->lr_side == dg::Edge::LR_RIGHT)
                {
                    dg::Node* n1 = m_map.getNode(it->node_id1);
                    dg::Node* n2 = m_map.getNode(it->node_id2);
                    if (n1 == nullptr || n2 == nullptr) continue;
                    dg::Point2 p1(n1->y - n2->y, n2->x - n1->x);    // rotate n1n2 by 90 degree
                    p1 = *n1 + p1 * 2 / norm(p1);
                    dg::Point2 p2 = p1 + (*n2 - *n1);
                    m_painter.drawEdge(view_image, p1, p2, node_radius, cv::Vec3b(0, 255, 0), edge_thickness, offset, zoom);
                }
            }
        }

        // display error data
        if (m_show_map_error)
        {
            for (auto it = m_error_nodes.begin(); it != m_error_nodes.end(); it++)
            {
                m_painter.drawPoint(view_image, **it, node_radius + 1, cv::Vec3b(0, 255, 255), offset, zoom);
            }
            for (auto it = m_error_edges.begin(); it != m_error_edges.end(); it++)
            {
                dg::Node* n1 = m_map.getNode((*it)->node_id1);
                dg::Node* n2 = m_map.getNode((*it)->node_id2);
                m_painter.drawEdge(view_image, *n1, *n2, node_radius, cv::Vec3b(0, 255, 255), edge_thickness + 1, offset, zoom);
            }
        }
    }
    else
    {
        m_painter.setParamValue("color_whitening", 3);
        m_painter.drawMap(view_image, &m_map, offset, zoom);
    }

    if (m_show_streetview)
    {
        for (auto it = m_map.getHeadView(); it != m_map.getTailView(); it++)
        {
            m_painter.drawPoint(view_image, *it, node_radius, cv::Vec3b(255, 0, 0), offset, zoom);
        }
    }
    if (m_show_poi)
    {
        for (auto it = m_map.getHeadPOI(); it != m_map.getTailPOI(); it++)
        {
            m_painter.drawPoint(view_image, *it, node_radius, cv::Vec3b(0, 0, 255), offset, zoom);
        }
    }
}


void MapEditor::verify()
{
    m_error_nodes.clear();
    m_error_edges.clear();
    if (m_map.isEmpty()) return;

    // check node
    std::string msg;
    for (auto node = m_map.getHeadNode(); node != m_map.getTailNode(); node++)
    {
        dg::Node* self = m_map.getNode(node->id);
        if (self == nullptr || self->id != node->id)
        {
            m_error_nodes.push_back(&(*node));
            if(self) msg += cv::format("node %zd: mismatch to getNode\n", node->id);
            else msg += cv::format("node %zd: nullptr returned at getNode\n", node->id);
        }
        for (auto it = node->edge_ids.begin(); it != node->edge_ids.end(); it++)
        {
            dg::Edge* edge = m_map.getEdge(*it);
            if (edge == nullptr)
            {
                m_error_nodes.push_back(&(*node));
                msg += cv::format("node %zd: edge %zd doesn't exist\n", node->id, *it);
            }
        }
        if (node->edge_ids.size() >= 3 && node->type != dg::Node::NODE_JUNCTION && node->type != dg::Node::NODE_ELEVATOR)
        {
            m_error_nodes.push_back(&(*node));
            msg += cv::format("node %zd: type error\n", node->id);
        }
    }

    // check edge
    double d_err = 3;   // meter
    for (auto edge = m_map.getHeadEdge(); edge != m_map.getTailEdge(); edge++)
    {
        dg::Node* n1 = m_map.getNode(edge->node_id1);
        dg::Node* n2 = m_map.getNode(edge->node_id2);
        if (n1 == nullptr) msg += cv::format("edge %zd: node1 %zd doesn't exist\n", edge->node_id1, n1->id);
        if (n2 == nullptr) msg += cv::format("edge %zd: node2 %zd doesn't exist\n", edge->node_id1, n2->id);
        if (n1 == nullptr || n2 == nullptr) m_error_edges.push_back(&(*edge));

        if (n1 && n2)
        {
            double d = norm(*n1 - *n2);
            if (fabs(d - edge->length) > d_err)
            {
                m_error_edges.push_back(&(*edge));
                msg += cv::format("edge %zd: d=%.1lf, e.length=%.1lf\n", edge->id, d, edge->length);
            }
        }
    }

    if (msg.empty())
    {
        msg = "No error!";
        return;
    }

    FILE* file = fopen("map_error_list.txt", "wt");
    if (file == nullptr) return;
    fprintf(file, "%s", msg.c_str());
    fclose(file);
}

void MapEditor::fixMapError()
{
    cv::AutoLock lock(m_mutex_data);
    if (m_map.isEmpty()) return;

    // fix node error
    std::vector<dg::ID> invalid_nodes;
    for (auto node = m_map.getHeadNode(); node != m_map.getTailNode(); node++)
    {
        // remove invalid edge ids
        std::vector<dg::ID> edge_ids;
        bool crosswalk = false;
        for (auto it = node->edge_ids.begin(); it != node->edge_ids.end(); it++)
        {
            dg::Edge* edge = m_map.getEdge(*it);
            if (edge == nullptr) continue;
            edge_ids.push_back(*it);
            if (edge->type == dg::Edge::EDGE_CROSSWALK) crosswalk = true;
        }
        node->edge_ids = edge_ids;

        // correct node type
        if (crosswalk) node->type = dg::Node::NODE_JUNCTION;
        if (!crosswalk && node->edge_ids.size() <= 2 && node->type == dg::Node::NODE_JUNCTION) node->type = dg::Node::NODE_BASIC;
        if (node->edge_ids.size() >= 3 && node->type == dg::Node::NODE_BASIC) node->type = dg::Node::NODE_JUNCTION;

        // delete isolated nodes
        if (node->edge_ids.empty()) invalid_nodes.push_back(node->id);
    }
    for (auto it = invalid_nodes.begin(); it != invalid_nodes.end(); it++)
    {
        m_map.deleteNode(*it);
    }

    // check edge
    std::vector<dg::ID> invalid_edges;
    for (auto edge = m_map.getHeadEdge(); edge != m_map.getTailEdge(); edge++)
    {
        dg::Node* n1 = m_map.getNode(edge->node_id1);
        dg::Node* n2 = m_map.getNode(edge->node_id2);
        if (n1 && n2) edge->length = norm(*n1 - *n2);
        if (n1 == nullptr || n2 == nullptr) invalid_edges.push_back(edge->id);
    }
    for (auto it = invalid_edges.begin(); it != invalid_edges.end(); it++)
    {
        m_map.deleteEdge(*it);
    }

    m_error_nodes.clear();
    m_error_edges.clear();
}


void MapEditor::initializeNextMapID()
{
    if (m_map.isEmpty()) return;

    dg::ID max_id = 0;
    for (auto it = m_map.getHeadNode(); it != m_map.getTailNode(); it++)
    {
        if (it->id > max_id) max_id = it->id;
    }
    for (auto it = m_map.getHeadEdge(); it != m_map.getTailEdge(); it++)
    {
        if (it->id > max_id) max_id = it->id;
    }
    m_next_id = max_id + 1;
}


void MapEditor::updateLRSide()
{
    cv::AutoLock lock(m_mutex_data);
    if (m_map.isEmpty()) return;
    computeLRSide(m_map);
}


void MapEditor::computeLRSide(dg::Map& map)
{
    map.updateEdgeLR();
}


void MapEditor::exportToJson()
{
    // TBD
}
