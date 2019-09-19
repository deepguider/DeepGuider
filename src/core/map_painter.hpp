#ifndef __MAP_PAINTER__
#define __MAP_PAINTER__

#include "opencx.hpp"
#include "core/simple_road_map.hpp"

namespace dg
{

class CanvasInfo : public cv::Size
{
public:
    CanvasInfo() : ppm(0), margin(0) { }

    cv::Rect2d box_m;

    cv::Rect box_p;

    double ppm;

    double margin;
};

class MapPainter : public cx::Algorithm
{
public:
    MapPainter()
    {
        m_pixel_per_meter = 100;

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

        m_axes_length = 0.5;
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

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);

        CX_LOAD_PARAM_COUNT(fn, "pixel_per_meter", m_pixel_per_meter, n_read);

        CX_LOAD_PARAM_COUNT(fn, "canvas_margin", m_canvas_margin, n_read);
        CX_LOAD_PARAM_COUNT(fn, "canvas_color", m_canvas_color, n_read);

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

            fs << "canvas_margin" << m_canvas_margin;
            fs << "canvas_color" << m_canvas_color;

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

    bool drawMap(cv::Mat& image, SimpleRoadMap& map) const
    {
        CanvasInfo info = getCanvasInfo(map);
        if (image.empty())
        {
            if (!clearCanvas(image, info, m_canvas_color)) return false;
            drawGrid(image, info, m_grid_step, m_grid_color, m_grid_thickness, m_grid_unit_font_scale, m_grid_unit_color);
            drawBox(image, info, m_box_color, m_box_thickness);
            drawAxes(image, info, m_axes_length, m_axes_x_color, m_axes_y_color, m_axes_thickness);
        }
        if (!image.empty())
        {
            drawNodes(image, info, map, m_node_radius, m_node_font_scale, m_node_color, m_node_thickness);
            for (SimpleRoadMap::NodeItr n = map.getHeadNode(); n != map.getTailNode(); n++)
            {
                drawEdges(image, info, map, &(*n), m_node_radius, m_edge_color, m_edge_thickness, m_edge_arrow_length);
            }
            return true;
        }
        return false;
    }

    CanvasInfo getCanvasInfo(SimpleRoadMap& map) const
    {
        return buildCanvasInfo(map, m_pixel_per_meter, m_canvas_margin);
    }

    static CanvasInfo buildCanvasInfo(SimpleRoadMap& map, double ppm, double margin)
    {
        CanvasInfo info;
        info.ppm = ppm;
        info.margin = margin;
        if (!map.isEmpty())
        {
            Point2 box_min = map.getHeadNode()->data, box_max = map.getHeadNode()->data;
            for (SimpleRoadMap::NodeItr n = map.getHeadNode(); n != map.getTailNode(); n++)
            {
                if (n->data.x < box_min.x) box_min.x = n->data.x;
                if (n->data.y < box_min.y) box_min.y = n->data.y;
                if (n->data.x > box_max.x) box_max.x = n->data.x;
                if (n->data.y > box_max.y) box_max.y = n->data.y;
            }
            info.box_m = cv::Rect2d(box_min, box_max);

            info.width = static_cast<int>((box_max.x - box_min.x + 4 * margin) * ppm + 0.5);  // + 0.5: Rounding
            info.height = static_cast<int>((box_max.y - box_min.y + 4 * margin) * ppm + 0.5); // + 0.5: Rounding

            int margin_p = static_cast<int>(info.margin * info.ppm + 0.5); // + 0.5: Rounding
            info.box_p.x = margin_p;
            info.box_p.y = margin_p;
            info.box_p.width = info.width - 2 * margin_p;
            info.box_p.height = info.height - 2 * margin_p;
        }
        return info;
    }

    static bool clearCanvas(cv::Mat& image, const CanvasInfo& info, const cv::Vec3b& color = cx::COLOR_WHITE)
    {
        if (info.area() > 0 && image.size() != info) image.create(info, CV_8UC3);
        if (image.empty()) return false;
        image = color;
        return true;
    }

    static bool drawBox(cv::Mat& image, const CanvasInfo& info, const cv::Vec3b& color, int thickness = 1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0) return false;

        cv::rectangle(image, info.box_p, color, thickness);
        return true;
    }

    static bool drawAxes(cv::Mat& image, const CanvasInfo& info, double length, const cv::Vec3b& color_x, const cv::Vec3b& color_y, int thickness = 1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0) return false;

        cv::Point axes_0 = cvtMeter2Pixel(Point2(0, 0), info) + Point2(0.5, 0.5);       // + 0.5: Rounding
        cv::Point axes_x = cvtMeter2Pixel(Point2(length, 0), info) + Point2(0.5, 0.5);  // + 0.5: Rounding
        cv::Point axes_y = cvtMeter2Pixel(Point2(0, length), info) + Point2(0.5, 0.5);  // + 0.5: Rounding
        cv::line(image, axes_0, axes_x, color_x, thickness);
        cv::line(image, axes_0, axes_y, color_y, thickness);
        return true;
    }

    static bool drawGrid(cv::Mat& image, const CanvasInfo& info, double grid_step, const cv::Vec3b& color, int thickness = 1, double unit_font_scale = 0.5, const cv::Vec3b& unit_color = cx::COLOR_BLACK, const cv::Point& unit_pos = cv::Point(100, 10))
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

    static bool drawNode(cv::Mat& image, const CanvasInfo& info, const Point2ID& node, double radius, double font_scale, const cv::Vec3b& color, int thickness = -1)
    {
        CV_DbgAssert(!image.empty());

        const int r = std::max(static_cast<int>(radius * info.ppm + 0.5), 1);
        const cv::Point font_offset(-r / 2, r / 2);
        cv::Vec3b font_color = color;
        if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;
        const cv::Point p = cvtMeter2Pixel(node, info);
        cv::circle(image, p, r, color, thickness);
        cv::putText(image, cv::format("%d", node.id), p + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale));
        return true;
    }

    static bool drawNodes(cv::Mat& image, const CanvasInfo& info, SimpleRoadMap& map, double radius, double font_scale, const cv::Vec3b& color, int thickness = -1)
    {
        CV_DbgAssert(!image.empty());

        const int r = std::max(static_cast<int>(radius * info.ppm + 0.5), 1);
        const cv::Point font_offset(-r / 2, r / 2);
        cv::Vec3b font_color = color;
        if (thickness < 0) font_color = cv::Vec3b(255, 255, 255) - color;
        for (SimpleRoadMap::NodeItr n = map.getHeadNode(); n != map.getTailNode(); n++)
        {
            const cv::Point p = cvtMeter2Pixel(n->data, info);
            cv::circle(image, p, r, color, thickness);
            cv::putText(image, cv::format("%d", n->data.id), p + font_offset, cv::FONT_HERSHEY_DUPLEX, font_scale, font_color, int(font_scale));
        }
        return true;
    }

    static bool drawEdge(cv::Mat& image, const CanvasInfo& info, const Point2ID& from, const Point2ID& to, double radius, const cv::Vec3b& color, int thickness = 1, double arrow_length = -1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0) return false;

        const double r = radius * info.ppm;
        const double a = arrow_length * info.ppm;

        // Draw an edge
        Point2 p = cvtMeter2Pixel(from, info);
        Point2 q = cvtMeter2Pixel(to, info);
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

    static bool drawEdges(cv::Mat& image, const CanvasInfo& info, SimpleRoadMap& map, SimpleRoadMap::Node* node, double radius, const cv::Vec3b& color, int thickness = 1, double arrow_length = -1)
    {
        CV_DbgAssert(!image.empty());
        if (thickness <= 0 || node == NULL) return false;

        const double r = radius * info.ppm;
        const double a = arrow_length * info.ppm;
        for (SimpleRoadMap::EdgeItr e = map.getHeadEdge(node); e != map.getTailEdge(node); e++)
        {
            // Draw an edge
            Point2 p = cvtMeter2Pixel(node->data, info);
            Point2 q = cvtMeter2Pixel(e->to->data, info);
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
        }
        return true;
    }

    static Point2 cvtMeter2Pixel(const Point2& p, const CanvasInfo& info)
    {
        Point2 m;
        m.x = (p.x + 2 * info.margin) * info.ppm;
        m.y = info.height - (p.y + 2 * info.margin) * info.ppm;
        return m;
    }

protected:
    double m_pixel_per_meter;

    double m_canvas_margin;

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

#endif // End of '__MAP_PAINTER_SIMPLE__'
