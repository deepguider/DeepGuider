#ifndef __MAP_PAINTER__
#define __MAP_PAINTER__

#include "utils/opencx.hpp"

namespace dg
{

class MapPainter : public cx::Painter, public cx::Algorithm
{
public:
    MapPainter()
    {
        m_node_radius = 10;
        m_node_font_scale = 0.5;
        m_node_color = cx::COLOR_BLUE;
        m_junction_color = cx::COLOR_BLUE;
        m_door_color = cx::COLOR_BLUE;

        m_edge_color = cx::COLOR_GREEN;
        m_sidewalk_color = cv::Vec3b(0, 255, 255);
        m_crosswalk_color = cv::Vec3b(0, 50, 50);
        m_mixedroad_color = cv::Vec3b(200, 100, 100);
        m_edge_thickness = 2;
        m_color_whitening = 1;
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);

        CX_LOAD_PARAM_COUNT(fn, "node_radius", m_node_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_font_scale", m_node_font_scale, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_color", m_node_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "junction_color", m_junction_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "door_color", m_door_color, n_read);

        CX_LOAD_PARAM_COUNT(fn, "edge_color", m_edge_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "sidewalk_color", m_sidewalk_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "crosswalk_color", m_crosswalk_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "mixedroad_color", m_mixedroad_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "edge_thickness", m_edge_thickness, n_read);

        CX_LOAD_PARAM_COUNT(fn, "color_whitening", m_color_whitening, n_read);

        return n_read;
    }

    virtual bool writeParam(cv::FileStorage& fs) const
    {
        if (cx::Algorithm::writeParam(fs))
        {
            fs << "node_radius" << m_node_radius;
            fs << "node_font_scale" << m_node_font_scale;
            fs << "node_color" << m_node_color;

            fs << "edge_color" << m_edge_color;
            fs << "edge_thickness" << m_edge_thickness;
            return true;
        }
        return false;
    }

    bool drawMap(cv::Mat& image, const Map* map, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1) const
    {
        if (map == nullptr) return false;
        if (!image.empty())
        {
            for (auto n = map->getHeadNodeConst(); n != map->getTailNodeConst(); n++)
                // drawEdges(image, map, &(*n), m_node_radius, m_edge_color, m_edge_thickness, offset, zoom);
                drawEdges(image, map, &(*n), m_node_radius, m_edge_color, 10, offset, zoom);
            drawNodes(image, map, m_node_radius, m_node_font_scale, m_node_color, offset, zoom);
            return true;
        }
        return false;
    }

    bool drawNode(cv::Mat& image, const Point2ID& node, double radius, double font_scale, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = -1) const
    {
        if (image.empty()) return false;
        int iradius = (int)(radius * zoom + 0.5);
        if (iradius < 1) iradius = 1;
        if (thickness > 0) thickness = (int)(thickness * zoom + 0.5);
        cv::Point center = (cvtValue2Pixel(node) - offset) * zoom;
        cv::circle(image, center, iradius, color, thickness);
        if (font_scale > 0)
        {
            font_scale = font_scale * zoom;
            cv::Point font_offset(-iradius / 2, iradius / 2);
            cv::Vec3b font_color = color;
            if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;
            cv::putText(image, cv::format("%zd", node.id), center + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale + 0.5));
        }
        return true;
    }

    bool drawNodes(cv::Mat& image, const Map* map, double radius, double font_scale, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = -1) const
    {
        if (map == nullptr || image.empty()) return false;
        int iradius = (int)(radius * zoom + 0.5);
        if (iradius < 1) iradius = 1;
        if (thickness > 0) thickness = (int)(thickness * zoom + 0.5);
        font_scale = font_scale * zoom;

        cv::Point font_offset(-iradius / 2, iradius / 2);
        cv::Vec3b font_color = color;
        if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;

        cv::Vec3b cr = color;
        cv::Vec3b cr_junction = m_junction_color;
        cv::Vec3b cr_door = m_door_color;
        if (m_color_whitening > 1)
        {
            cv::Vec3b delta = cv::Vec3b(255, 255, 255) - cv::Vec3b(255, 255, 255) / m_color_whitening;
            cr = color / m_color_whitening + delta;
            cr_junction = m_junction_color / m_color_whitening + delta;
            cr_door = m_door_color / m_color_whitening + delta;
            font_color = font_color / m_color_whitening + delta;
        }

        for (auto n = map->getHeadNodeConst(); n != map->getTailNodeConst(); n++)
        {
            cv::Point center = (cvtValue2Pixel(*n) - offset) * zoom + cv::Point2d(0.5, 0.5);
            if(n->type == dg::Node::NODE_JUNCTION)
                cv::circle(image, center, iradius, cr_junction, thickness);
            else if (n->type == dg::Node::NODE_DOOR)
                cv::circle(image, center, iradius, cr_door, thickness);
            else
                cv::circle(image, center, iradius, cr, thickness);
            if (font_scale > 0)
                cv::putText(image, cv::format("%zd", n->id), center + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale + 0.5));
        }
        return true;
    }

    bool drawEdge(cv::Mat& image, const Point2& from, const Point2& to, double radius, const cv::Vec3b& color, double thickness, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, double arrow_length = -1) const
    {
        if (image.empty() || thickness <= 0) return false;
        int ithickness = (int)(thickness * zoom + 0.5);
        if (ithickness < 1) ithickness = 1;

        // Draw an edge
        cv::Point2d p = (cvtValue2Pixel(from) - offset) * zoom;
        cv::Point2d q = (cvtValue2Pixel(to) - offset) * zoom;
        double theta = atan2(q.y - p.y, q.x - p.x);
        cv::Point2d delta(radius * zoom * cos(theta), radius * zoom * sin(theta));
        p = p + delta;
        q = q - delta;
        cv::line(image, p, q, color, ithickness);

        // Draw its arrow
        if (arrow_length > 0)
        {
            double theta_p = theta + CV_PI / 6;
            double theta_m = theta - CV_PI / 6;
            cv::line(image, q, q - arrow_length * zoom * Point2(cos(theta_p), sin(theta_p)), color, ithickness);
            cv::line(image, q, q - arrow_length * zoom * Point2(cos(theta_m), sin(theta_m)), color, ithickness);
        }
        return true;
    }

    bool drawEdges(cv::Mat& image, const Map* map, const Node* node, double radius, const cv::Vec3b& color, double thickness, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, double arrow_length = -1) const
    {
        if (image.empty() || map == nullptr || thickness <= 0 || node == nullptr) return false;

        cv::Vec3b cr = color;
        cv::Vec3b cr_sidewalk = m_sidewalk_color;
        cv::Vec3b cr_crosswalk = m_crosswalk_color;
        cv::Vec3b cr_mixedroad = m_mixedroad_color;
        if (m_color_whitening > 1)
        {
            cv::Vec3b delta = cv::Vec3b(255, 255, 255) - cv::Vec3b(255, 255, 255) / m_color_whitening;
            cr = color / m_color_whitening + delta;
            cr_sidewalk = m_sidewalk_color / m_color_whitening + delta;
            cr_crosswalk = m_crosswalk_color / m_color_whitening + delta;
            cr_mixedroad = m_mixedroad_color / m_color_whitening + delta;
        }

        for (auto eid = node->edge_ids.begin(); eid != node->edge_ids.end(); eid++)
        {
            const Node* to = map->getConnectedNode(node, *eid);
            const Edge* edge = map->getEdge(*eid);
            if (edge->type == Edge::EDGE_CROSSWALK)
            {
                drawEdge(image, *node, *to, radius, cr_crosswalk / 2, thickness * 3, offset, zoom, arrow_length);
                drawEdge(image, *node, *to, radius, cv::Vec3b(200, 255, 255), thickness, offset, zoom, arrow_length);
            }
            else if (edge->type == Edge::EDGE_SIDEWALK)
                drawEdge(image, *node, *to, radius, cr_sidewalk, thickness, offset, zoom, arrow_length);
            else if (edge->type == Edge::EDGE_ROAD)
                drawEdge(image, *node, *to, radius, cr_mixedroad, thickness, offset, zoom, arrow_length);
            else
                drawEdge(image, *node, *to, radius, cr, thickness, offset, zoom, arrow_length);
        }
        return true;
    }

    bool drawPath(cv::Mat& image, const dg::Map* map, const dg::Path* path, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, const cv::Vec3b& ecolor = cv::Vec3b(0, 255, 0), const cv::Vec3b& ncolor = cv::Vec3b(0, 255, 255), double nradius = 4, double ethickness = 2)
    {
        if (map == nullptr || path == nullptr || path->empty()) return false;

        const Node* node_prev = nullptr;
        for (int idx = 0; idx < (int)path->pts.size(); idx++)
        {
            dg::ID node_id = path->pts[idx].node_id;
            const Node* node = map->getNode(node_id);
            if (node) {
                if (node_prev)
                {
                    drawEdge(image, *node_prev, *node, 0, ecolor, ethickness, offset, zoom);
                    int rdelta = (node_prev->type == Node::NODE_JUNCTION) ? 1 : 0;
                    drawNode(image, *node_prev, nradius + rdelta, 0, ncolor, offset, zoom);
                    if (node_prev->type == Node::NODE_JUNCTION) drawNode(image, *node_prev, nradius - 1, 0, ncolor / 3, offset, zoom);
                }
                else if (idx > 0)
                {
                    drawEdge(image, path->pts[idx - 1], path->pts[idx], 0, ecolor / 3, ethickness, offset, zoom);
                }
                int rdelta = (node->type == Node::NODE_JUNCTION) ? 1 : 0;
                drawNode(image, *node, nradius + rdelta, 0, ncolor, offset, zoom);
                if (node->type == Node::NODE_JUNCTION) drawNode(image, *node, nradius - 1, 0, ncolor / 3, offset, zoom);
                node_prev = node;
            }
            else if(node_prev)
            {
                drawEdge(image, *node_prev, path->pts[idx], 0, ecolor/3, ethickness, offset, zoom);
                int rdelta = (node_prev->type == Node::NODE_JUNCTION) ? 1 : 0;
                drawNode(image, *node_prev, nradius + rdelta, 0, ncolor, offset, zoom);
                if (node_prev->type == Node::NODE_JUNCTION) drawNode(image, *node_prev, nradius - 1, 0, ncolor / 3, offset, zoom);

                drawPoint(image, path->pts[idx], nradius + 1, ncolor, offset, zoom);
                drawPoint(image, path->pts[idx], nradius - 1, cx::COLOR_RED, offset, zoom);
            }
        }
        return true;
    }

    bool drawPoint(cv::Mat& image, const cv::Point2d& center, double radius, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = -1, int linetype = cv::LineTypes::LINE_8) const
    {
        if (image.empty()) clearCanvas(image);

        int iradius = (int)(radius * zoom + 0.5);
        if (iradius < 1) iradius = 1;
        if (thickness > 0) thickness = (int)(thickness * zoom + 0.5);
        cv::Point center_px = (cvtValue2Pixel(center) - offset) * zoom + cv::Point2d(0.5, 0.5);
        cv::circle(image, center_px, iradius, color, thickness, linetype);
        return true;
    }

    bool drawLine(cv::Mat& image, const cv::Point2d& val1, const cv::Point2d& val2, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = 1, int linetype = cv::LineTypes::LINE_8) const
    {
        if (thickness <= 0) return false;
        if (image.empty()) clearCanvas(image);
        int ithickness = (int)(thickness * zoom + 0.5);
        if (ithickness < 1) ithickness = 1;

        cv::Point px1 = (cvtValue2Pixel(val1) - offset)*zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
        cv::Point px2 = (cvtValue2Pixel(val2) - offset)*zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
        cv::line(image, px1, px2, color, ithickness, linetype);
        return true;
    }

    bool drawLine(cv::Mat& image, const std::vector<cv::Point2d>& vals, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = 1, int linetype = cv::LineTypes::LINE_8) const
    {
        if (thickness <= 0 || vals.size() < 2) return false;
        if (image.empty()) clearCanvas(image);
        int ithickness = (int)(thickness * zoom + 0.5);
        if (ithickness < 1) ithickness = 1;

        cv::Point px_prev = (cvtValue2Pixel(vals.front()) - offset)*zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
        for (size_t i = 1; i < vals.size(); i++)
        {
            cv::Point px = (cvtValue2Pixel(vals[i]) - offset)*zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::line(image, px_prev, px, color, ithickness, linetype);
            px_prev = px;
        }
        return true;
    }


protected:
    double m_node_radius;
    double m_node_thickness;
    double m_node_font_scale;
    cv::Vec3b m_node_color;
    cv::Vec3b m_junction_color;
    cv::Vec3b m_door_color;

    double m_edge_thickness;
    double m_edge_arrow_length;
    cv::Vec3b m_edge_color;
    cv::Vec3b m_sidewalk_color;
    cv::Vec3b m_crosswalk_color;
    cv::Vec3b m_mixedroad_color;

    double m_color_whitening;

}; // End of 'MapPainter'

} // End of 'dg'

#endif // End of '__MAP_PAINTER__'
