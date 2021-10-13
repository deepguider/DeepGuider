#ifndef __VIEWPORT_HPP__
#define __VIEWPORT_HPP__

#include "opencv2/opencv.hpp"

namespace dg
{

class Viewport
{
protected:
    cv::Mat m_image;
    cv::Rect m_image_rect;
    cv::Rect m_viewport = cv::Rect(0, 0, 1920, 1080);

public:
    void initialize(cv::Mat image, cv::Size viewport_size = cv::Size(1920, 1080), cv::Point viewport_offset = cv::Point(0, 0), double zoom = 1)
    {
        cv::AutoLock lock(m_mutex);
        m_image = image;
        m_image_rect = cv::Rect(0, 0, image.cols, image.rows);
        m_viewport = cv::Rect(viewport_offset, viewport_size);

        m_virtual_zoom = zoom;
        if (m_viewport.width >= m_image_rect.width && m_viewport.height >= m_image_rect.height)
        {
            m_virtual_zoom_min = 1;
        }
        if (m_viewport.width >= m_image_rect.width)
        {
            m_viewport.width = m_image_rect.width;
            m_viewport.x = 0; 
        }
        if (m_viewport.height >= m_image_rect.height)
        {
            m_viewport.height = m_image_rect.height;
            m_viewport.y = 0; 
        }
    }

    void getViewportImage(cv::Mat& viewport_image)
    {
        cv::AutoLock lock(m_mutex);
        cv::Rect image_roi(m_viewport.x, m_viewport.y, (int)(m_viewport.width / m_zoom + 0.5), (int)(m_viewport.height / m_zoom + 0.5));
        cv::resize(m_image(image_roi & m_image_rect), viewport_image, m_viewport.size());
    }

    cv::Point2d cvtView2Pixel(const cv::Point2d& view_xy)
    {
        cv::AutoLock lock(m_mutex);
        double ix = m_viewport.x + view_xy.x / m_zoom;
        double iy = m_viewport.y + view_xy.y / m_zoom;
        return cv::Point2d(ix, iy);
    }

    cv::Point2d cvtPixel2View(const cv::Point2d& pixel_xy)
    {
        cv::AutoLock lock(m_mutex);
        double x = (pixel_xy.x - m_viewport.x) * m_zoom;
        double y = (pixel_xy.y - m_viewport.y) * m_zoom;
        return cv::Point2d(x, y);
    }

    cv::Point offset() { return m_viewport.tl(); }

    double zoom() { return m_zoom; }

    cv::Size size() { return m_viewport.size(); }

    void procMouseEvent(int evt, int x, int y, int flags)
    {
        cv::AutoLock lock(m_mutex);
        if (evt == cv::EVENT_MOUSEMOVE)
        {
            if (m_mouse_drag && (x != m_mouse_pt2.x || y != m_mouse_pt2.y))
            {
                double ix = m_view_pt1.x + (m_mouse_pt1.x - m_mouse_pt2.x) / m_zoom;
                double iy = m_view_pt1.y + (m_mouse_pt1.y - m_mouse_pt2.y) / m_zoom;
                updateViewport(ix, iy);
                m_mouse_pt2 = cv::Point(x, y);
            }
            m_mouse_move_x = x;
            m_mouse_move_y = y;
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
            if (delta > 0 && m_virtual_zoom < m_virtual_zoom_max) // virtual_zoom in
            {
                double zoom_old = m_zoom;
                double zoom_new = m_virtual_zoom * 2;
                double view_sx = ix - x / zoom_new;
                double view_sy = iy - y / zoom_new;
                updateViewport(view_sx, view_sy, zoom_new);
            }
            else if (delta < 0 && m_virtual_zoom > m_virtual_zoom_min)  // virtual_zoom out
            {
                double zoom_old = m_zoom;
                double zoom_new = m_virtual_zoom / 2;
                double view_sx = ix - x / zoom_new;
                double view_sy = iy - y / zoom_new;
                updateViewport(view_sx, view_sy, zoom_new);
            }
        }
        else if (evt == cv::EVENT_MOUSEHWHEEL)  // for ROS run in Linux
        {
            double ix = m_viewport.x + m_mouse_move_x / m_zoom;
            double iy = m_viewport.y + m_mouse_move_y / m_zoom;
            int delta = -cv::getMouseWheelDelta(flags);
            if (delta > 0 && m_virtual_zoom < m_virtual_zoom_max) // virtual_zoom in
            {
                double zoom_old = m_zoom;
                double zoom_new = m_virtual_zoom * 2;
                double view_sx = ix - m_mouse_move_x / zoom_new;
                double view_sy = iy - m_mouse_move_y / zoom_new;
                updateViewport(view_sx, view_sy, zoom_new);
            }
            else if (delta < 0 && m_virtual_zoom > m_virtual_zoom_min)  // virtual_zoom out
            {
                double zoom_old = m_zoom;
                double zoom_new = m_virtual_zoom / 2;
                double view_sx = ix - m_mouse_move_x / zoom_new;
                double view_sy = iy - m_mouse_move_y / zoom_new;
                updateViewport(view_sx, view_sy, zoom_new);
            }
        }
    }

    void setZoomRange(double z_min, double z_max)
    {
        m_virtual_zoom_min = z_min;
        m_virtual_zoom_max = z_max;
    }

    void centerizeViewportTo(cv::Point2d px, double center_margin_ratio = 0.15)
    {
        cv::AutoLock lock(m_mutex);

        double view_iw = m_viewport.width / m_zoom;
        double view_ih = m_viewport.height / m_zoom;
        dg::Point2 vx = px - dg::Point2(m_viewport.x, m_viewport.y);

        double view_sx = m_viewport.x;
        double view_sy = m_viewport.y;
        double update_x = view_iw * center_margin_ratio;
        double update_y = view_ih * center_margin_ratio;
        bool update_view = false;
        if (vx.x < view_iw * center_margin_ratio || vx.x > view_iw * (1 - center_margin_ratio))
        {
            view_sx = (vx.x < view_iw* center_margin_ratio) ? view_sx - update_x : view_sx + update_x;
            update_view = true;
        }
        if (vx.y < view_ih * center_margin_ratio || vx.y > view_ih * (1 - center_margin_ratio))
        {
            view_sy = (vx.y < view_ih* center_margin_ratio) ? view_sy - update_y : view_sy + update_y;
            update_view = true;
        }
        if (update_view)
        {
            updateViewport(view_sx, view_sy);
        }
    }

protected:
    void updateViewport(double view_sx, double view_sy, double virtual_zoom = -1)
    {
        if (virtual_zoom > 0)
        {
            m_zoom = m_virtual_zoom = virtual_zoom;
            if (m_zoom < (double)m_viewport.width / m_image_rect.width) m_zoom = (double)m_viewport.width / m_image_rect.width;
            if (m_zoom < (double)m_viewport.height / m_image_rect.height) m_zoom = (double)m_viewport.height / m_image_rect.height;
        }

        int view_iw = (int)(m_viewport.width / m_zoom + 0.5);
        int view_ih = (int)(m_viewport.height / m_zoom + 0.5);
        if (view_iw < m_image_rect.width)
        {
            if (view_sx < 0) view_sx = 0;
            else if (view_sx + view_iw > m_image_rect.width) view_sx = m_image_rect.width - view_iw;
            m_viewport.x = (int)(view_sx + 0.5);
        }
        else
        {
            m_viewport.x = 0;
        }
        if (view_ih < m_image_rect.height)
        {
            if (view_sy < 0) view_sy = 0;
            if (view_sy + view_ih > m_image_rect.height) view_sy = m_image_rect.height - view_ih;
            m_viewport.y = (int)(view_sy + 0.5);
        }
        else
        {
            m_viewport.y = 0;
        }
    }

    double m_zoom = 1;
    double m_virtual_zoom = 1;
    double m_virtual_zoom_max = 4;
    double m_virtual_zoom_min = 0.5;
    bool m_mouse_drag = false;
    cv::Point m_mouse_pt1;
    cv::Point m_mouse_pt2;
    cv::Point m_view_pt1;
    cv::Mutex m_mutex;
    int m_mouse_move_x = 0;
    int m_mouse_move_y = 0;

}; // End of 'Viewport'

} // End of 'dg'

#endif // End of '__VIEWPORT_HPP__'
