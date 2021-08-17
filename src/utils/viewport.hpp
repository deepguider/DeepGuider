#ifndef __VIEWPORT_HPP__
#define __VIEWPORT_HPP__

#include "opencv2/opencv.hpp"

namespace dg
{

class Viewport
{
protected:
    cv::Mat m_image;
    cv::Rect2d m_world;
    cv::Rect2d m_viewport = cv::Rect(0, 0, 1920, 1080);

public:
    void initialize(cv::Mat image, cv::Size viewport_size = cv::Size(1920, 1080), cv::Point viewport_offset = cv::Point(0, 0), double zoom = 1)
    {
        cv::AutoLock lock(m_mutex);

        m_image = image;
        m_world = cv::Rect(0, 0, image.cols, image.rows);
        m_viewport = cv::Rect(viewport_offset, viewport_size);

        m_zoom = zoom;
        if (m_viewport.width >= m_world.width && m_viewport.height >= m_world.height)
        {
            m_zoom_min = 1;
        }
        if (m_viewport.width >= m_world.width)
        {
            m_viewport.width = m_world.width;
            m_viewport.x = 0; 
        }
        if (m_viewport.height >= m_world.height)
        {
            m_viewport.height = m_world.height;
            m_viewport.y = 0; 
        }
    }

    cv::Mat getViewportImage(cv::Point2d& view_offset, double& view_zoom)
    {
        cv::Rect2d image_roi(m_viewport.x, m_viewport.y, m_viewport.width / m_zoom, m_viewport.height / m_zoom);
        image_roi = (image_roi & m_world);
        view_zoom = min((double)m_viewport.width / image_roi.width, (double)m_viewport.height / image_roi.height);
        view_offset = m_viewport.tl();
        cv::Mat viewport_image;
        cv::resize(m_image(image_roi), viewport_image, cv::Size(), view_zoom, view_zoom);
        return viewport_image;
    }

    cv::Rect2d getViewport()
    {
        cv::AutoLock lock(m_mutex);
        cv::Rect2d world_rect(m_viewport.x, m_viewport.y, m_viewport.width / m_zoom, m_viewport.height / m_zoom);
        return (m_world & world_rect);
    }

    cv::Point2d cvtView2World(const cv::Point2d& view_xy)
    {
        cv::AutoLock lock(m_mutex);

        double wx = m_viewport.x + view_xy.x / m_zoom;
        double wy = m_viewport.y + view_xy.y / m_zoom;
        return cv::Point2d(wx, wy);
    }

    cv::Point2d cvtWorld2View(const cv::Point2d& world_xy)
    {
        cv::AutoLock lock(m_mutex);

        double vx = (world_xy.x - m_viewport.x) * m_zoom;
        double vy = (world_xy.y - m_viewport.y) * m_zoom;
        return cv::Point2d(vx, vy);
    }

    void procMouseEvent(int evt, int x, int y, int flags)
    {
        cv::AutoLock lock(m_mutex);

        if (evt == cv::EVENT_MOUSEMOVE)
        {
            if (m_mouse_drag && (x != m_mouse_pt2.x || y != m_mouse_pt2.y))
            {
                int ix = (int)(m_view_pt1.x + (m_mouse_pt1.x - m_mouse_pt2.x) / m_zoom);
                int iy = (int)(m_view_pt1.y + (m_mouse_pt1.y - m_mouse_pt2.y) / m_zoom);
                updateViewport(ix, iy);
                m_mouse_pt2 = cv::Point(x, y);
            }
        }
        else if (evt == cv::EVENT_LBUTTONDOWN)
        {
            m_mouse_pt1 = cv::Point(x, y);
            m_mouse_pt2 = cv::Point(x, y);
            m_view_pt1 = m_viewport.tl();
            m_mouse_drag = true;
        }
        else if (evt == cv::EVENT_LBUTTONUP)
        {
            m_mouse_drag = false;
        }
        else if (evt == cv::EVENT_MOUSEWHEEL)
        {
            double ix = m_viewport.x + x / m_zoom;
            double iy = m_viewport.y + y / m_zoom;
            int delta = cv::getMouseWheelDelta(flags);
            if (delta > 0 && m_zoom < m_zoom_max) // zoom in
            {
                double zoom_old = m_zoom;
                double zoom_new = m_zoom * 2;
                int view_sx = (int)(ix - x / zoom_new);
                int view_sy = (int)(iy - y / zoom_new);
                updateViewport(view_sx, view_sy, zoom_new);
            }
            else if (delta < 0 && m_zoom > m_zoom_min)  // zoom out
            {
                double zoom_old = m_zoom;
                double zoom_new = m_zoom / 2;
                int view_sx = (int)(ix - x / zoom_new);
                int view_sy = (int)(iy - y / zoom_new);
                updateViewport(view_sx, view_sy, zoom_new);
            }
        }
    }

protected:
    void updateViewport(double offset_x, double offset_y, double zoom = -1)
    {
        if (zoom > 0) m_zoom = zoom;
        int view_w = (int)(m_viewport.width / m_zoom);
        int view_h = (int)(m_viewport.height / m_zoom);
        if (view_w < m_world.width)
        {
            if (offset_x < 0) offset_x = 0;
            else if (offset_x + view_w > m_world.width) offset_x = m_world.width - view_w;
            m_viewport.x = offset_x;
        }
        else
        {
            m_viewport.x = 0;
        }
        if (view_h < m_world.height)
        {
            if (offset_y < 0) offset_y = 0;
            if (offset_y + view_h > m_world.height) offset_y = m_world.height - view_h;
            m_viewport.y = offset_y;
        }
        else
        {
            m_viewport.y = 0;
        }
    }

    double m_zoom = 1;
    double m_zoom_max = 4;
    double m_zoom_min = 0.5;
    bool m_mouse_drag = false;
    cv::Point m_mouse_pt1;
    cv::Point m_mouse_pt2;
    cv::Point m_view_pt1;
    cv::Mutex m_mutex;

}; // End of 'Viewport'

} // End of 'dg'

#endif // End of '__VIEWPORT_HPP__'
