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

        m_edge_color = cx::COLOR_GREEN;
        m_crosswalk_color = cv::Vec3b(0, 50, 50);
        m_edge_thickness = 2;
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);

        CX_LOAD_PARAM_COUNT(fn, "node_radius", m_node_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_font_scale", m_node_font_scale, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_color", m_node_color, n_read);

        CX_LOAD_PARAM_COUNT(fn, "edge_color", m_edge_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "crosswalk_color", m_crosswalk_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "edge_thickness", m_edge_thickness, n_read);

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

    bool drawMap(cv::Mat& image, const Map* map) const
    {
        if (map == nullptr) return false;
        if (!image.empty())
        {
            for (auto n = map->getHeadNodeConst(); n != map->getTailNodeConst(); n++)
                drawEdges(image, map, &(*n), m_node_radius, m_edge_color, m_edge_thickness);
            drawNodes(image, map, m_node_radius, m_node_font_scale, m_node_color);
            return true;
        }
        return false;
    }

    bool drawNode(cv::Mat& image, const Point2ID& node, int radius, double font_scale, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = -1) const
    {
        if (image.empty()) return false;
        radius = (int)(radius * zoom + 0.5);
        cv::Point center = (cvtValue2Pixel(node) - offset) * zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
        cv::circle(image, center, radius, color, thickness);
        if (font_scale > 0)
        {
            font_scale = font_scale * zoom;
            cv::Point font_offset(-radius / 2, radius / 2);
            cv::Vec3b font_color = color;
            if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;
            cv::putText(image, cv::format("%zd", node.id), center + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale + 0.5));
        }
        return true;
    }

    bool drawNodes(cv::Mat& image, const Map* map, int radius, double font_scale, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = -1, int min_n_edge = 1) const
    {
        if (map == nullptr || image.empty()) return false;
        radius = (int)(radius * zoom + 0.5);
        font_scale = font_scale * zoom;

        cv::Point font_offset(-radius / 2, radius / 2);
        cv::Vec3b font_color = color;
        if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;

        for (auto n = map->getHeadNodeConst(); n != map->getTailNodeConst(); n++)
        {
            if (map->countEdges(&(*n)) < min_n_edge) continue;
            cv::Point center = (cvtValue2Pixel(*n) - offset) * zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::circle(image, center, radius, color, thickness);
            if (font_scale > 0)
                cv::putText(image, cv::format("%zd", n->id), center + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale + 0.5));
        }
        return true;
    }

    bool drawEdge(cv::Mat& image, const Point2& from, const Point2& to, int radius, const cv::Vec3b& color, int thickness, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int arrow_length = -1) const
    {
        if (image.empty() || thickness <= 0) return false;
        radius = (int)(radius * zoom + 0.5);
        thickness = (int)(thickness * zoom + 0.5);

        // Draw an edge
        cv::Point2d p = (cvtValue2Pixel(from) - offset) * zoom;
        cv::Point2d q = (cvtValue2Pixel(to) - offset) * zoom;
        double theta = atan2(q.y - p.y, q.x - p.x);
        cv::Point2d delta(radius * cos(theta), radius * sin(theta));
        p = p + delta;
        q = q - delta;
        cv::line(image, p, q, color, thickness);

        // Draw its arrow
        if (arrow_length > 0)
        {
            double theta_p = theta + CV_PI / 6;
            double theta_m = theta - CV_PI / 6;
            cv::line(image, q, q - arrow_length * Point2(cos(theta_p), sin(theta_p)), color, thickness);
            cv::line(image, q, q - arrow_length * Point2(cos(theta_m), sin(theta_m)), color, thickness);
        }
        return true;
    }

    bool drawEdges(cv::Mat& image, const Map* map, const Node* node, int radius, const cv::Vec3b& color, int thickness, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int arrow_length = -1) const
    {
        if (image.empty() || map == nullptr || thickness <= 0 || node == nullptr) return false;

        for (auto eid = node->edge_ids.begin(); eid != node->edge_ids.end(); eid++)
        {
            const Node* to = map->getConnectedNode(node, *eid);
            const Edge* edge = map->getEdge(*eid);
            if (edge->type == Edge::EDGE_CROSSWALK)
            {
                drawEdge(image, *node, *to, radius, m_crosswalk_color / 2, thickness * 3, offset, zoom, arrow_length);
                drawEdge(image, *node, *to, radius, cv::Vec3b(200, 255, 255), thickness, offset, zoom, arrow_length);
            }
            else
                drawEdge(image, *node, *to, radius, color, thickness, offset, zoom, arrow_length);
        }
        return true;
    }

    bool drawPath(cv::Mat& image, const dg::Map* map, const dg::Path* path, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, const cv::Vec3b& ecolor = cv::Vec3b(0, 255, 0), const cv::Vec3b& ncolor = cv::Vec3b(0, 255, 255), int nradius = 4, int ethickness = 2)
    {
        if (map == nullptr || path == nullptr || path->empty()) return false;

        const Node* node_prev = nullptr;
        for (int idx = 0; idx < (int)path->pts.size(); idx++)
        {
            dg::ID node_id = path->pts[idx].node_id;
            const Node* node = map->getNode(node_id);
            if (node) {
                if (node_prev) drawEdge(image, *node_prev, *node, 0, ecolor, ethickness, offset, zoom);
                if (node_prev)
                {
                    int rdelta = (node_prev->type == Node::NODE_JUNCTION) ? 1 : 0;
                    drawNode(image, *node_prev, nradius + rdelta, 0, ncolor, offset, zoom);
                    if (node_prev->type == Node::NODE_JUNCTION) drawNode(image, *node_prev, nradius - 1, 0, ncolor / 3, offset, zoom);
                }
                int rdelta = (node->type == Node::NODE_JUNCTION) ? 1 : 0;
                drawNode(image, *node, nradius + rdelta, 0, ncolor, offset, zoom);
                if (node->type == Node::NODE_JUNCTION) drawNode(image, *node, nradius - 1, 0, ncolor / 3, offset, zoom);
                node_prev = node;
            }
        }
        return true;
    }

    bool drawPoint(cv::Mat& image, const cv::Point2d& center, int radius, const cv::Vec3b& color, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1, int thickness = -1, int linetype = cv::LineTypes::LINE_8) const
    {
        if (image.empty()) clearCanvas(image);

        radius = (int)(radius * zoom + 0.5);
        cv::Point center_px = (cvtValue2Pixel(center) - offset)*zoom + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
        cv::circle(image, center_px, radius, color, thickness, linetype);
        return true;
    }

protected:
    int m_node_radius;

    double m_node_font_scale;

    cv::Vec3b m_node_color;

    int m_node_thickness;

    cv::Vec3b m_edge_color;

    cv::Vec3b m_crosswalk_color;

    int m_edge_thickness;

    int m_edge_arrow_length;
}; // End of 'MapPainter'

} // End of 'dg'

#endif // End of '__MAP_PAINTER__'
