#ifndef __MAP_PAINTER__
#define __MAP_PAINTER__

#include "dg_core.hpp"
#include "utils/opencx.hpp"
#include "localizer/utm_converter.hpp"


namespace dg
{

class MapCanvasInfo : public cv::Size
{
public:
    MapCanvasInfo() : ppm(0), margin(0) { }

    cv::Rect2d box_m;

    cv::Rect box_p;

    double ppm;

    double margin;

    Point2 offset;
};


class MapPainter : public cx::Algorithm, public UTMConverter
{
public:

    MapPainter()
    {
        m_pixel_per_meter = 100;
        m_image_rotation = 0;  // radian

        m_canvas_margin = 0.2;
        m_canvas_color = cx::COLOR_WHITE;

        m_box_color = cv::Vec3b(128, 128, 128);
        m_box_thickness = 1;

        m_grid_step = 1;
        m_grid_color = cv::Vec3b(200, 200, 200);
        m_grid_thickness = 1;
        m_grid_unit_font_scale = 0.5;
        m_grid_unit_color = cv::Vec3b(64, 64, 64);
        m_grid_unit_pos = cv::Point(100, 10);

        m_axes_length = 1;
        m_axes_x_color = cx::COLOR_RED;
        m_axes_y_color = cx::COLOR_BLUE;
        m_axes_thickness = 2;

        m_node_radius = 0.1;
        m_node_font_scale = 0.5;
        m_node_color = cx::COLOR_BLUE;
        m_node_thickness = -1;

        m_edge_color = cx::COLOR_GREEN;
        m_edge_thickness = 2;
        m_edge_arrow_length = 0.05;
    }

    virtual ~MapPainter() { }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);

        CX_LOAD_PARAM_COUNT(fn, "pixel_per_meter", m_pixel_per_meter, n_read);
        CX_LOAD_PARAM_COUNT(fn, "image_rotation", m_image_rotation, n_read);

        CX_LOAD_PARAM_COUNT(fn, "canvas_margin", m_canvas_margin, n_read);
        CX_LOAD_PARAM_COUNT(fn, "canvas_color", m_canvas_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "canvas_offset", m_canvas_offset, n_read);

        CX_LOAD_PARAM_COUNT(fn, "box_color", m_box_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "box_thickness", m_box_thickness, n_read);

        CX_LOAD_PARAM_COUNT(fn, "grid_step", m_grid_step, n_read);
        CX_LOAD_PARAM_COUNT(fn, "grid_color", m_grid_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "grid_thickness", m_grid_thickness, n_read);
        CX_LOAD_PARAM_COUNT(fn, "grid_unit_font_scale", m_grid_unit_font_scale, n_read);
        CX_LOAD_PARAM_COUNT(fn, "grid_unit_color", m_grid_unit_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "grid_unit_pos", m_grid_unit_pos, n_read);

        CX_LOAD_PARAM_COUNT(fn, "axes_length", m_axes_length, n_read);
        CX_LOAD_PARAM_COUNT(fn, "axes_x_color", m_axes_x_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "axes_y_color", m_axes_y_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "axes_thickness", m_axes_thickness, n_read);

        CX_LOAD_PARAM_COUNT(fn, "node_radius", m_node_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_font_scale", m_node_font_scale, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_color", m_node_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "node_thickness", m_node_thickness, n_read);

        CX_LOAD_PARAM_COUNT(fn, "edge_color", m_edge_color, n_read);
        CX_LOAD_PARAM_COUNT(fn, "edge_thickness", m_edge_thickness, n_read);
        CX_LOAD_PARAM_COUNT(fn, "edge_arrow_length", m_edge_arrow_length, n_read);

        return n_read;
    }

    virtual bool writeParam(cv::FileStorage& fs) const
    {
        if (cx::Algorithm::writeParam(fs))
        {
            fs << "pixel_per_meter" << m_pixel_per_meter;
            fs << "image_rotation" << m_image_rotation;

            fs << "canvas_margin" << m_canvas_margin;
            fs << "canvas_color" << m_canvas_color;
            fs << "canvas_offset" << m_canvas_offset;

            fs << "box_color" << m_box_color;
            fs << "box_thickness" << m_box_thickness;

            fs << "grid_step" << m_grid_step;
            fs << "grid_color" << m_grid_color;
            fs << "grid_thickness" << m_grid_thickness;
            fs << "grid_unit_font_scale" << m_grid_unit_font_scale;
            fs << "grid_unit_color" << m_grid_unit_color;
            fs << "grid_unit_pos" << m_grid_unit_pos;

            fs << "axes_length" << m_axes_length;
            fs << "axes_x_color" << m_axes_x_color;
            fs << "axes_y_color" << m_axes_y_color;
            fs << "axes_thickness" << m_axes_thickness;

            fs << "node_radius" << m_node_radius;
            fs << "node_font_scale" << m_node_font_scale;
            fs << "node_color" << m_node_color;
            fs << "node_thickness" << m_node_thickness;

            fs << "edge_color" << m_edge_color;
            fs << "edge_thickness" << m_edge_thickness;
            fs << "edge_arrow_length" << m_edge_arrow_length;
            return true;
        }
        return false;
    }

    bool drawMap(cv::Mat& image, const MapCanvasInfo& info, dg::Map& map)
    {
        drawGrid(image, info, m_grid_step, m_grid_color, m_grid_thickness, m_grid_unit_font_scale, m_grid_unit_color, m_grid_unit_pos);
        drawBox(image, info, m_box_color, m_box_thickness);
        drawAxes(image, info, m_axes_length, m_axes_x_color, m_axes_y_color, m_axes_thickness);

        // Draw nodes and edges
        if (!image.empty())
        {
            drawEdges(image, info, map, m_node_radius, m_edge_color, m_edge_thickness, m_edge_arrow_length);
            drawNodes(image, info, map, m_node_radius, m_node_font_scale, m_node_color, m_node_thickness);
            return true;
        }
        return false;
    }

    bool drawMap(cv::Mat& image, dg::Map& map)
    {
        MapCanvasInfo info = getCanvasInfo(image);
        return drawMap(image, info, map);
    }

    MapCanvasInfo getCanvasInfo(cv::Mat& image)
    {
        // get metric
        cv::Rect2d box(0, 0, image.cols, image.rows);
        MapCanvasInfo info = buildCanvasInfo(box, m_pixel_per_meter, m_canvas_margin);
        cv::Size sz = image.size();
        if (sz.width > 0 && sz.height > 0)
        {
            info.width = sz.width;
            info.height = sz.height;

            int margin_p = static_cast<int>(info.margin * info.ppm + 0.5); // + 0.5: Rounding
            info.box_p.x = margin_p;
            info.box_p.y = margin_p;
            info.box_p.width = info.width - 2 * margin_p;
            info.box_p.height = info.height - 2 * margin_p;

            info.offset = m_canvas_offset;

            Point2 p1 = cvtPixel2Meter(Point2(0, 0), info);
            Point2 p2 = cvtPixel2Meter(Point2(sz.width, sz.height), info);
            info.box_m.x = (p1.x < p2.x) ? p1.x : p2.x;
            info.box_m.y = (p1.y < p2.y) ? p1.y : p2.y;
            info.box_m.width = fabs(p2.x - p1.x);
            info.box_m.height = fabs(p2.y - p1.y);
        }

        return info;
    }

    MapCanvasInfo buildCanvasInfo(const cv::Rect2d& box, double ppm, double margin)
    {
        MapCanvasInfo info;
        info.box_m = box;
        info.ppm = ppm;
        info.margin = margin;
        info.width = static_cast<int>((box.width + 4 * margin) * ppm + 0.5);    // + 0.5: Rounding
        info.height = static_cast<int>((box.height + 4 * margin) * ppm + 0.5);  // + 0.5: Rounding
        int margin_p = static_cast<int>(info.margin * info.ppm + 0.5);          // + 0.5: Rounding
        info.box_p.x = margin_p;
        info.box_p.y = margin_p;
        info.box_p.width = info.width - 2 * margin_p;
        info.box_p.height = info.height - 2 * margin_p;
        info.offset.x = 2 * margin * ppm;
        info.offset.y = info.height - 2 * margin * ppm;
        return info;
    }

    bool drawBox(cv::Mat& image, const MapCanvasInfo& info, const cv::Vec3b& color, int thickness = 1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0) return false;

        cv::rectangle(image, info.box_p, color, thickness);
        return true;
    }

    bool drawAxes(cv::Mat& image, const MapCanvasInfo& info, double length, const cv::Vec3b& color_x, const cv::Vec3b& color_y, int thickness = 1)
    {
        CV_DbgAssert(!image.empty());
        if (length <= 0 || thickness <= 0) return false;

        cv::Point axes_0 = cvtMeter2Pixel(Point2(0, 0), info) + Point2(0.5, 0.5);       // + 0.5: Rounding
        cv::Point axes_x = cvtMeter2Pixel(Point2(length, 0), info) + Point2(0.5, 0.5);  // + 0.5: Rounding
        cv::Point axes_y = cvtMeter2Pixel(Point2(0, length), info) + Point2(0.5, 0.5);  // + 0.5: Rounding
        cv::line(image, axes_0, axes_x, color_x, thickness);
        cv::line(image, axes_0, axes_y, color_y, thickness);
        return true;
    }
     
    bool drawGrid(cv::Mat& image, const MapCanvasInfo& info, double grid_step, const cv::Vec3b& color, int thickness = 1, double unit_font_scale = 0.5, const cv::Vec3b& unit_color = cx::COLOR_BLACK, const cv::Point& unit_pos = cv::Point(100, 10))
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0) return false;

        if (grid_step < 0)
        {
            // Select cell size automatically
            double sz = std::max(info.box_m.width, info.box_m.height);
            grid_step = pow(10., int(log10(sz / 4)));
            if (grid_step < 1) grid_step = 1;
            else if (grid_step > 1000) grid_step = 1000;
        }

        Point2 center_map(info.box_m.x + info.box_m.width / 2, info.box_m.y + info.box_m.height / 2);
        Point2 center_grid = cvtMeter2Pixel(Point2(int(center_map.x / grid_step) * grid_step, int(center_map.y / grid_step) * grid_step), info);
        for (int i = 0;; i++)
        {
            double y = center_grid.y - grid_step * i * info.ppm;
            if (y < info.box_p.tl().y || y > info.box_p.br().y) break;
            cv::line(image, cv::Point2d(info.box_p.tl().x, y), cv::Point2d(info.box_p.br().x, y), color, thickness); // Upward X-directional grids
        }
        for (int i = 1;; i++)
        {
            double y = center_grid.y + grid_step * i * info.ppm;
            if (y < info.box_p.tl().y || y > info.box_p.br().y) break;
            cv::line(image, cv::Point2d(info.box_p.tl().x, y), cv::Point2d(info.box_p.br().x, y), color, thickness); // Downward X-directional grids
        }
        for (int i = 0;; i++)
        {
            double x = center_grid.x + grid_step * i * info.ppm;
            if (x < info.box_p.tl().x || x > info.box_p.br().x) break;
            cv::line(image, cv::Point2d(x, info.box_p.tl().y), cv::Point2d(x, info.box_p.br().y), color, thickness); // Rightward Y-directional grids
        }
        for (int i = 1;; i++)
        {
            double x = center_grid.x - grid_step * i * info.ppm;
            if (x < info.box_p.tl().x || x > info.box_p.br().x) break;
            cv::line(image, cv::Point2d(x, info.box_p.tl().y), cv::Point2d(x, info.box_p.br().y), color, thickness); // Leftward Y-directional grids
        }
        if (unit_font_scale > 0)
            cv::putText(image, cv::format("Grid: %d [m]", int(grid_step)), cv::Point(image.cols, image.rows) - unit_pos, cv::FONT_HERSHEY_DUPLEX, unit_font_scale, unit_color);
        return true;
    }

    bool drawPath(cv::Mat& image, const MapCanvasInfo& info, dg::Map& map, const dg::Path& path, const cv::Vec3b& ecolor = cv::Vec3b(255, 0, 0), const cv::Vec3b& ncolor = cv::Vec3b(0, 255, 255), int nradius = 5, int ethickness = 2)
    {
        Node* node_prev = nullptr;
        for (int idx = 0; idx < (int)path.pts.size(); idx++)
        {
            dg::ID node_id = path.pts[idx].node_id;
            Node* node = map.findNode(node_id);
            if (node) {
                if (node_prev) drawEdge(image, info, node_prev, node, 0, ecolor, ethickness);
                if (node_prev)
                {
                  int rdelta = (node_prev->type == Node::NODE_JUNCTION) ? 1 : 0;
                  drawNode(image, info, node_prev, nradius + rdelta, 0, ncolor);
                  if (node_prev->type == Node::NODE_JUNCTION) drawNode(image, info, node_prev, nradius-2, 0, ncolor/3);  
                }
                int rdelta = (node->type == Node::NODE_JUNCTION) ? 1 : 0;
                drawNode(image, info, node, nradius + rdelta, 0, ncolor);
                if (node->type == Node::NODE_JUNCTION) drawNode(image, info, node, nradius-2, 0, ncolor/3);
                node_prev = node;
            }
        }
        return true;
    }

    bool drawNode(cv::Mat& image, const MapCanvasInfo& info, dg::LatLon ll, double radius, double font_scale, const cv::Vec3b& color, int thickness = -1)
    {
        CV_DbgAssert(!image.empty());

        const int r = std::max(static_cast<int>(radius * info.ppm + 0.5), 1);
        const cv::Point p = cvtLatLon2Pixel(ll, info);
        cv::circle(image, p, r, color, thickness);
        return true;
    }

    bool drawNode(cv::Mat& image, const MapCanvasInfo& info, Node* node, double radius, double font_scale, const cv::Vec3b& color, int thickness = -1)
    {
        CV_DbgAssert(!image.empty());

        const int r = std::max(static_cast<int>(radius * info.ppm + 0.5), 1);
        const cv::Point font_offset(-r / 2, r / 2);
        cv::Vec3b font_color = color;
        if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;
        LatLon ll(node->lat, node->lon);
        const cv::Point p = cvtLatLon2Pixel(ll, info);
        cv::circle(image, p, r, color, thickness);
        if (font_scale > 0)
            cv::putText(image, cv::format("%zd", node->id), p + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale));
        return true;
    }

    bool drawNodes(cv::Mat& image, const MapCanvasInfo& info, Map& map, double radius, double font_scale, const cv::Vec3b& color, int thickness = -1)
    {
        CV_DbgAssert(!image.empty());

        const int r = std::max(static_cast<int>(radius * info.ppm + 0.5), 1);
        const cv::Point font_offset(-r / 2, r / 2);
        cv::Vec3b font_color = color;
        if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;
        for (size_t i=0; i<map.nodes.size(); i++)
        {
            LatLon ll(map.nodes[i].lat, map.nodes[i].lon);
            const cv::Point p = cvtLatLon2Pixel(ll, info);
            cv::circle(image, p, r, color, thickness);
            if (map.nodes[i].type == Node::NODE_JUNCTION)
            {
//              cv::circle(image, p, r-1, cv::Vec3b(0, 0, 0), 1);
                cv::circle(image, p, r-1, color/2, 2);
            }
            if (font_scale > 0)
                cv::putText(image, cv::format("%zd", map.nodes[i].id), p + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale));
        }
        return true;
    }

    bool drawEdge(cv::Mat& image, const MapCanvasInfo& info, Node* from, Node* to, double radius, const cv::Vec3b& color, int thickness = 1, double arrow_length = -1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0) return false;

        const double r = radius * info.ppm;
        const double a = arrow_length * info.ppm;

        // Draw an edge
        LatLon ll1(from->lat, from->lon);
        LatLon ll2(to->lat, to->lon);
        Point2 p = cvtLatLon2Pixel(ll1, info);
        Point2 q = cvtLatLon2Pixel(ll2, info);
        double theta = atan2(q.y - p.y, q.x - p.x);
        Point2 delta(r * cos(theta), r * sin(theta));
        p = p + delta;
        q = q - delta;
        cv::line(image, p, q, color, thickness);

        // Draw its arrow
        if (a > 0)
        {
            double theta_p = theta + CV_PI / 6;
            double theta_m = theta - CV_PI / 6;
            cv::line(image, q, q - a * Point2(cos(theta_p), sin(theta_p)), color, thickness);
            cv::line(image, q, q - a * Point2(cos(theta_m), sin(theta_m)), color, thickness);
        }

        return true;
    }

    bool drawEdges(cv::Mat& image, const MapCanvasInfo& info, Map& map, double radius, const cv::Vec3b& color, int thickness = 1, double arrow_length = -1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0 ) return false;

        const double r = radius * info.ppm;
        const double a = arrow_length * info.ppm;
        for (size_t i=0; i<map.edges.size(); i++)
        {
            Node* node1 = map.findNode(map.edges[i].node_id1);
            Node* node2 = map.findNode(map.edges[i].node_id2);
            if (node1 == nullptr || node2 == nullptr) continue;

            // Draw an edge
            LatLon ll1(node1->lat, node1->lon);
            LatLon ll2(node2->lat, node2->lon);
            Point2 p = cvtLatLon2Pixel(ll1, info);
            Point2 q = cvtLatLon2Pixel(ll2, info);
            double theta = atan2(q.y - p.y, q.x - p.x);
            Point2 delta(r * cos(theta), r * sin(theta));
            p = p + delta;
            q = q - delta;
            cv::line(image, p, q, color, thickness);
            if (map.edges[i].type == Edge::EDGE_CROSSWALK)
            {
                //cv::line(image, p, q, color/2, thickness*3);
                cv::line(image, p, q, cv::Vec3b(0,150,50), thickness);
            }

            // Draw its arrow
            if (a > 0)
            {
                double theta_p = theta + CV_PI / 6;
                double theta_m = theta - CV_PI / 6;
                cv::line(image, q, q - a * Point2(cos(theta_p), sin(theta_p)), color, thickness);
                cv::line(image, q, q - a * Point2(cos(theta_m), sin(theta_m)), color, thickness);
            }
        }
        return true;
    }

    Point2 cvtMeter2Pixel(const Point2& mt, const MapCanvasInfo& info)
    {
        Point2 px;
        if (m_image_rotation == 0)
        {
            px.x = mt.x * info.ppm + info.offset.x;
            px.y = info.offset.y - mt.y * info.ppm;
        }
        else
        {
            double cost = cos(-m_image_rotation);
            double sint = sin(-m_image_rotation);
            px.x = (mt.x * info.ppm) * cost - (-mt.y * info.ppm) * sint + info.offset.x;
            px.y = (mt.x * info.ppm) * sint + (-mt.y * info.ppm) * cost + info.offset.y;
        }
        return px;
    }

    Point2 cvtPixel2Meter(const Point2& px, const MapCanvasInfo& info)
    {
        Point2 mt;
        if (m_image_rotation == 0)
        {
            mt.x = (px.x - info.offset.x) / info.ppm;
            mt.y = -(px.y - info.offset.y) / info.ppm;
        }
        else
        {
            double cost = cos(-m_image_rotation);
            double sint = sin(-m_image_rotation);
            mt.x = ((px.x - info.offset.x) * cost + (px.y - info.offset.y) * sint) / info.ppm;
            mt.y = -(-(px.x - info.offset.x) * sint + (px.y - info.offset.y) * cost) / info.ppm;
        }
        return mt;
    }

    LatLon cvtPixel2LatLon(const cv::Point pt, const MapCanvasInfo& info)
    {
        Point2 mt = cvtPixel2Meter(pt, info);
        return toLatLon(mt);
    }

    cv::Point cvtLatLon2Pixel(const LatLon& ll, const MapCanvasInfo& info)
    {        
        Point2 mt = toMetric(ll);
        return cvtMeter2Pixel(mt, info);
    }

protected:
    double m_pixel_per_meter;

    double m_image_rotation = 0;  // radian

    double m_canvas_margin;

    Point2 m_canvas_offset;

    cv::Vec3b m_canvas_color;

    cv::Vec3b m_box_color;

    int m_box_thickness;

    double m_grid_step;

    cv::Vec3b m_grid_color;

    int m_grid_thickness;

    double m_grid_unit_font_scale;

    cv::Vec3b m_grid_unit_color;

    cv::Point m_grid_unit_pos;

    double m_axes_length;

    cv::Vec3b m_axes_x_color;

    cv::Vec3b m_axes_y_color;

    int m_axes_thickness;

    double m_node_radius;

    double m_node_font_scale;

    cv::Vec3b m_node_color;

    int m_node_thickness;

    cv::Vec3b m_edge_color;

    int m_edge_thickness;

    double m_edge_arrow_length;

}; // End of 'MapPainter'

} // End of 'dg'

#endif // End of '__MAP_PAINTER__'
