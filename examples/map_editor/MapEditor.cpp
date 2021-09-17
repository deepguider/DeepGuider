#include "framework.h"
#include "MapEditorApp.h"
#include "DlgNode.h"
#include "DlgEdge.h"
#include "afxdialogex.h"
#include "MapEditor.h"

#define MAX_PATH_LEN 512

void onMouseEvent(int event, int x, int y, int flags, void* param)
{
    MapEditor* editor = (MapEditor*)param;
    editor->procMouseEvent(event, x, y, flags);
}

void MapEditor::procMouseEvent(int evt, int x, int y, int flags)
{
    if (m_under_edit) return;
    m_viewport.procMouseEvent(evt, x, y, flags);

    if (evt == cv::EVENT_MOUSEMOVE)
    {
    }
    else if (evt == cv::EVENT_LBUTTONDOWN)
    {
    }
    else if (evt == cv::EVENT_LBUTTONUP)
    {
    }
    else if (evt == cv::EVENT_LBUTTONDBLCLK)
    {
        m_under_edit = true;

        cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
        cv::Point2d metric = m_painter.cvtPixel2Value(px);

        double dist_thr = 10;    // meter
        dg::Node* node = m_map.getNearestNode(metric);
        double d1 = norm(*node - metric);

        dg::Point2 ep;
        dg::Edge* edge = m_map.getNearestEdge(metric, ep);
        double d2 = norm(ep - metric);

        bool update_node = node && d1<=dist_thr && (d1 < dist_thr / 2 || d1 <= d2);
        if (update_node && d2 <= d1 && edge->length < dist_thr * 1.5 && d1 > edge->length / 3) update_node = false;

        bool update_gui = false;
        if (update_node)
        {
            cv::Mat view_image;
            double zoom = m_viewport.zoom();
            m_viewport.getViewportImage(view_image);
            m_painter.drawNode(view_image, *node, 4, 0, cv::Vec3b(0, 255, 255), m_viewport.offset(), zoom, (int)(2*zoom+0.5));
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
            m_painter.drawEdge(view_image, *from, *to, 3, cv::Vec3b(0, 255, 255), 3, m_viewport.offset(), m_viewport.zoom());
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
                if (edge->type != dlg.type) update_gui = true;

                edge->id = dlg.ID;
                edge->type = dlg.type;
                edge->length = dlg.length;
                edge->lr_side = dlg.lr_side;
                edge->node_id1 = dlg.node_id1;
                edge->node_id2 = dlg.node_id2;
                edge->directed = (dlg.directed == TRUE);
            }
        }

        if (update_gui)
        {
            m_bg_image_original.copyTo(m_bg_image);
            m_painter.drawGrid(m_bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, m_guiprop.grid_unit_pos);
            m_painter.drawOrigin(m_bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
            if (!m_map.isEmpty()) m_painter.drawMap(m_bg_image, &m_map);
        }

        m_under_edit = false;
    }
    else if (evt == cv::EVENT_RBUTTONDOWN)
    {
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
   if(!m_fpath.empty()) m_map.save(m_fpath.c_str());
}

void MapEditor::saveAs()
{
    char szFile[MAX_PATH_LEN] = "";  // buffer for file path
    CString szFilter = "CSV Files (*.csv)|*.csv|All Files (*.*)|*.*||";
    CFileDialog dlg(FALSE, 0, szFile, OFN_OVERWRITEPROMPT, szFilter, NULL);
    if (dlg.DoModal() == IDOK)
    {
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
        m_fpath = CT2A(dlg.GetPathName().GetString());
        m_map.load(m_fpath.c_str());
        if (m_map.isEmpty()) m_fpath.clear();
        m_bg_image_original.copyTo(m_bg_image);
        m_painter.drawGrid(m_bg_image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, m_guiprop.grid_unit_pos);
        m_painter.drawOrigin(m_bg_image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
        if (!m_map.isEmpty()) m_painter.drawMap(m_bg_image, &m_map);
    }
}
