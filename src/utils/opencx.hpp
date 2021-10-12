/**
 * OpenCX: Sunglok's OpenCV Extension
 *
 * OpenCX aims to provide extended functionality and tools to OpenCV for more convenience.
 * It consists of several header files, 'opencx.hpp' and others, which only depend on OpenCV in C++.
 * Just include the file to your project. It will work without complex configuration and dependency.
 * OpenCX is Beerware so that it is free to use and distribute.
 *
 * - Homepage: https://github.com/sunglok/opencx
 *
 * @author  Sunglok Choi (http://sites.google.com/site/sunglok)
 * @version 0.4 (05/28/2020)
 */

/**
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <sunglok@hanmail.net> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Sunglok Choi
 * ----------------------------------------------------------------------------
 */

#ifndef __OPEN_CX__
#define __OPEN_CX__

#include "opencv2/opencv.hpp"
#include "opensx.hpp"
#include "ekf.hpp"

#ifndef CX_LOAD_PARAM_TO
/** A macro function to load a value from cv::FileNode */
#   define CX_LOAD_PARAM(fn, name_cfg, name_var) \
        if (!(fn)[name_cfg].empty()) { (fn)[name_cfg] >> name_var; }
#endif

#ifndef CX_LOAD_PARAM_RESULT
/** A macro function to load a value from cv::FileNode and increase the given counter if successful */
#   define CX_LOAD_PARAM_COUNT(fn, name_cfg, name_var, success_counter) \
        if (!(fn)[name_cfg].empty()) { (fn)[name_cfg] >> name_var; success_counter++; }
#endif

namespace cx
{
    /**
     * @brief More convenient Algorithm
     *
     * This is an extension of cv::Algorithm to provide simple ways to configure various parameters if its readParam() function is defined.
     * The pre-built functions, setParam(), setParamValue(), and setParamText(), enable to set a value (scalar, vector, text) for each parameter through its name.
     * Please refer its example at example.cpp.
     */
    class Algorithm : public cv::Algorithm
    {
    public:
        /**
         * Read parameters from cv::FileNode
         * @param fs The instance of cv::FileStorage to read parameters
         * @return The number of updated parameters
         */
        virtual int readParam(const cv::FileNode& fn) { return 0; }

        /**
         * Write parameters and their values to cv::FileStorage
         * @param fs The instance of cv::FileStorage to write parameters
         * @return True if successful (false if failed)
         */
        virtual bool writeParam(cv::FileStorage& fs) const { return fs.isOpened(); }

        /**
         * Read parameters from the given file
         * @param filename The filename to read parameters
         * @return Result of success (true) or failure (false)
         */
        int loadParam(const std::string& filename)
        {
            cv::FileStorage fs(filename, cv::FileStorage::READ);
            if (!fs.isOpened()) return false;

            return readParam(fs.root());
        }

        /**
         * Write parameters and their values to he given file
         * @param filename The filename to write parameters
         * @return True if successful (false if failed)
         */
        bool saveParam(const std::string& filename)
        {
            cv::FileStorage fs(filename, cv::FileStorage::WRITE);
            if (!fs.isOpened()) return false;

            time_t rawtime;
            time(&rawtime);
            fs << "date" << asctime(localtime(&rawtime));
            return writeParam(fs);
        }

        /**
         * Set a single (or multiple) parameter from a string to describes the parameter in YAML
         * @param config A string to describes the parameter in YAML
         * @return True if successful (false if failed)
         */
        bool setParam(const std::string& config)
        {
            if (config.empty()) return false;
            std::string yaml;
            getline(std::istringstream(config), yaml);
            if (yaml == "%YAML:1.0" || yaml == "%YAML 1.0") yaml = config;
            else yaml = "%YAML:1.0\n" + config;
            cv::FileStorage fs(yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
            return readParam(fs.root()) > 0;
        }

        /**
         * Set a parameter from its name and value
         * @param param The name of the parameter
         * @param value The value of the parameter
         * @return True if successful (false if failed)
         */
        bool setParamValue(const std::string& param, double value)
        {
            if (param.empty()) return false;
            std::string yaml = param + cv::format(": %f", value);
            return setParam(yaml);
        }

        /**
         * Set a parameter from its name and value in a vector form
         * @param param The name of the parameter
         * @param values The value of the parameter in a vector form
         * @return True if successful (false if failed)
         */
        bool setParamValue(const std::string& param, const std::vector<double>& values)
        {
            if (param.empty()) return false;
            std::string yaml = param + ": [ ";
            for (size_t i = 0; i < values.size(); i++)
            {
                yaml += cv::format("%f", values[i]);
                if (i < values.size() - 1) yaml += ", ";
            }
            yaml += " ]";
            return setParam(yaml);
        }

        /**
         * Set a parameter from its name and value
         * @param param The name of the parameter
         * @param value The value of the parameter
         * @return True if successful (false if failed)
         */
        bool setParamTexts(const std::string& param, const std::string& value)
        {
            if (param.empty()) return false;
            std::string yaml = param + ": " + value;
            return setParam(yaml);
        }
    }; // End of 'Algorithm'

    /**
     * @brief Easier VideoWriter
     *
     * This is an extension of cv::VideoWriter to enable to call open() without the size and channel of images.
     * For convenience, those two informations are automatically configured when the first image is given to the video recorder.
     */
    class VideoWriter : public cv::VideoWriter
    {
    public:
        /** The default constructor */
        VideoWriter()
        {
            open("", -1);
        }

        /**
         * A constructor with initialization of member variables
         * @param filename The name of video file
         * @param fps The frame rate of video file
         * @param fourcc The codec of video file
         */
        VideoWriter(const std::string& filename, double fps = 10, int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'))
        {
            open(filename, fps, fourcc);
        }

        /**
         * Initialize to record a video
         * @param filename The name of video file
         * @param fps The frame rate of video file
         * @param fourcc The codec of video file
         * @return True if successful (false if failed)
         */
        virtual bool open(const std::string& filename, double fps = 10, int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'))
        {
            m_filename = filename;
            m_fps = fps;
            m_fourcc = fourcc;
            return true;
        }

        /**
         * Check whether VideoWriter is properly initialized
         * @return True if successfully configured (false if not)
         */
        virtual bool isConfigured() const
        {
            return !m_filename.empty() && m_fps > 0;
        }

        /**
         * Push the given image to the recording video
         * @param image An image to record
         */
        virtual void write(const cv::Mat& image)
        {
            if (isOpened()) cv::VideoWriter::write(image);
            else if (isConfigured())
            {
                if (cv::VideoWriter::open(m_filename, m_fourcc, m_fps, image.size(), image.channels() > 1))
                    cv::VideoWriter::write(image);
            }
        }

        /**
         * Terminate to record the video
         */
        virtual void release()
        {
            cv::VideoWriter::release();
            open("", -1);
        }

        /**
         * Push the given image to the recording video
         * @param vr The given video recording instance
         * @param image An image to record
         * @return The given video recording instance
         */
        friend VideoWriter& operator<<(VideoWriter& vr, const cv::Mat& image)
        {
            vr.write(image);
            return vr;
        }

    protected:
        /** The name of video file */
        std::string m_filename;

        /** The frame rate of video file */
        double m_fps;

        /** The codec of video file */
        int m_fourcc;
    }; // End of 'VideoWriter'

    class Painter
    {
    public:
        Painter()
        {
            configCanvas(cx::RangeDbl(0, 100), cx::RangeDbl(0, 100), cv::Size(640, 480), 20, 20);
        }

        bool configCanvas(const RangeDbl& range_x, const RangeDbl& range_y, const cv::Point2d& px_per_val, const cv::Point& margin, const cv::Point& padding)
        {
            m_min_val = { range_x.min, range_y.min };
            if (px_per_val.x > 0) m_px_per_val.x = px_per_val.x;
            if (px_per_val.y > 0) m_px_per_val.y = px_per_val.y;
            if (margin.x >= 0) m_img_margin.x = margin.x;
            if (margin.y >= 0) m_img_margin.y = margin.y;
            if (padding.x >= 0) m_img_padding.x = padding.x;
            if (padding.y >= 0) m_img_padding.y = padding.y;

            m_img_size.width  = static_cast<int>(range_x.length() * m_px_per_val.x + 0.5) + 2 * (m_img_padding.x + m_img_margin.x); // + 0.5: Rounding
            m_img_size.height = static_cast<int>(range_y.length() * m_px_per_val.y + 0.5) + 2 * (m_img_padding.y + m_img_margin.y); // + 0.5: Rounding
            m_img_box = getInnerBox(m_img_size, m_img_margin);
            m_img_offset = getOffset(m_min_val, m_px_per_val, m_img_size, m_img_margin, m_img_padding);
            return true;
        }

        bool configCanvas(const RangeDbl& range_x, const RangeDbl& range_y, const cv::Size& img_size, const cv::Point& margin, const cv::Point& padding)
        {
            m_min_val = { range_x.min, range_y.min };
            if (img_size.width > 0) m_img_size.width = img_size.width;
            if (img_size.height > 0) m_img_size.height = img_size.height;
            if (margin.x >= 0) m_img_margin.x = margin.x;
            if (margin.y >= 0) m_img_margin.y = margin.y;
            if (padding.x >= 0) m_img_padding.x = padding.x;
            if (padding.y >= 0) m_img_padding.y = padding.y;

            m_img_box = getInnerBox(m_img_size, m_img_margin);
            m_px_per_val.x = (m_img_box.width  - 2 * m_img_padding.x) / range_x.length();
            m_px_per_val.y = (m_img_box.height - 2 * m_img_padding.y) / range_y.length();
            m_img_offset = getOffset(m_min_val, m_px_per_val, m_img_size, m_img_margin, m_img_padding);
            return true;
        }

        bool configCanvas(const cv::Point2d& origin_px, const cv::Point2d& px_per_val, const cv::Size& img_size, const cv::Point& margin, const cv::Point& padding)
        {
            if (img_size.width > 0) m_img_size.width = img_size.width;
            if (img_size.height > 0) m_img_size.height = img_size.height;
            if (px_per_val.x > 0) m_px_per_val.x = px_per_val.x;
            if (px_per_val.y > 0) m_px_per_val.y = px_per_val.y;
            if (margin.x >= 0) m_img_margin.x = margin.x;
            if (margin.y >= 0) m_img_margin.y = margin.y;
            if (padding.x >= 0) m_img_padding.x = padding.x;
            if (padding.y >= 0) m_img_padding.y = padding.y;

            m_img_box = getInnerBox(m_img_size, m_img_margin);
            CV_DbgAssert(m_px_per_val.x > 0 && m_px_per_val.y > 0);
            m_min_val.x = (m_img_box.x + m_img_padding.x - origin_px.x) / m_px_per_val.x;
            m_min_val.y = (origin_px.y - m_img_box.y - m_img_box.height) / m_px_per_val.y;
            m_img_offset = getOffset(m_min_val, m_px_per_val, m_img_size, m_img_margin, m_img_padding);
            return true;
        }

        cv::Point2d getPixel2Value() const { return m_px_per_val; }

        bool configCanvas(const RangeDbl& range_x, const RangeDbl& range_y, const cv::Point2d& px_per_val, int margin = -1, int padding = -1)
        {
            return configCanvas(range_x, range_y, px_per_val, cv::Point(margin, margin), cv::Point(padding, padding));
        }

        bool configCanvas(const RangeDbl& range_x, const RangeDbl& range_y, const cv::Size& img_size = cv::Size(-1, -1), int margin = -1, int padding = -1)
        {
            return configCanvas(range_x, range_y, img_size, cv::Point(margin, margin), cv::Point(padding, padding));
        }

        bool configCanvas(const cv::Point2d& origin_px, const cv::Point2d& px_per_val = cv::Point2d(-1, -1), const cv::Size& img_size = cv::Size(-1, -1), int margin = -1, int padding = -1)
        {
            return configCanvas(origin_px, px_per_val, img_size, cv::Point(margin, margin), cv::Point(padding, padding));
        }

        bool clearCanvas(cv::Mat& image, const cv::Vec3b& color = cv::Vec3b(255, 255, 255)) const
        {
            if (m_img_size.width <= 0 || m_img_size.height <= 0) return false;

            image.create(m_img_size, CV_8UC3);
            if (image.empty()) return false;
            image = color;
            return true;
        }

        void setImageRotation(double rad)
        {
            m_img_rotation = rad;
        }

        double getImageRotation()
        {
            return m_img_rotation;
        }

        cv::Point2d cvtValue2Pixel(const cv::Point2d& val) const
        {
            cv::Point2d px;
            double cost = cos(-m_img_rotation);
            double sint = sin(-m_img_rotation);
            px.x = (val.x * m_px_per_val.x) * cost - (-val.y * m_px_per_val.y) * sint + m_img_offset.x;
            px.y = (val.x * m_px_per_val.x) * sint + (-val.y * m_px_per_val.y) * cost + m_img_offset.y;

            return px;
        }

        cv::Point2d cvtPixel2Value(const cv::Point2d& px) const
        {
            CV_DbgAssert(m_px_per_val.x > 0 && m_px_per_val.y > 0);

            cv::Point2d val;
            double cost = cos(-m_img_rotation);
            double sint = sin(-m_img_rotation);
            val.x = ((px.x - m_img_offset.x) * cost + (px.y - m_img_offset.y) * sint) / m_px_per_val.x;
            val.y = -(-(px.x - m_img_offset.x) * sint + (px.y - m_img_offset.y) * cost) / m_px_per_val.y;

            return val;
        }

        bool drawPoint(cv::Mat& image, const cv::Point2d& center, int radius, const cv::Vec3b& color, int thickness = -1, int linetype = cv::LineTypes::LINE_8) const
        {
            if (image.empty()) clearCanvas(image);

            cv::Point center_px = cvtValue2Pixel(center) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::circle(image, center_px, radius, color, thickness, linetype);
            return true;
        }

        bool drawPoints(cv::Mat& image, const std::vector<cv::Point2d>& pts, int radius, const cv::Vec3b& color, int thickness = -1, int linetype = cv::LineTypes::LINE_8) const
        {
            if (image.empty()) clearCanvas(image);

            for (auto pt = pts.begin(); pt != pts.end(); pt++)
            {
                cv::Point pt_px = cvtValue2Pixel(*pt) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
                cv::circle(image, pt_px, radius, color, thickness, linetype);
            }
            return true;
        }

        bool drawLine(cv::Mat& image, const cv::Point2d& val1, const cv::Point2d& val2, const cv::Vec3b& color, int thickness = 1, int linetype = cv::LineTypes::LINE_8) const
        {
            if (thickness <= 0) return false;
            if (image.empty()) clearCanvas(image);

            cv::Point px1 = cvtValue2Pixel(val1) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::Point px2 = cvtValue2Pixel(val2) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::line(image, px1, px2, color, thickness, linetype);
            return true;
        }

        bool drawLine(cv::Mat& image, const std::vector<cv::Point2d>& vals, const cv::Vec3b& color, int thickness = 1, int linetype = cv::LineTypes::LINE_8) const
        {
            if (thickness <= 0 || vals.size() < 2) return false;
            if (image.empty()) clearCanvas(image);

            cv::Point px_prev = cvtValue2Pixel(vals.front()) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            for (size_t i = 1; i < vals.size(); i++)
            {
                cv::Point px = cvtValue2Pixel(vals[i]) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
                cv::line(image, px_prev, px, color, thickness, linetype);
                px_prev = px;
            }
            return true;
        }

        bool drawBoundary(cv::Mat& image, const cv::Vec3b& color, int thickness = 1) const
        {
            if (thickness <= 0) return false;
            if (image.empty()) clearCanvas(image);

            cv::rectangle(image, m_img_box, color, thickness);
            return true;
        }

        bool drawOrigin(cv::Mat& image, const cv::Point2d& length, const cv::Vec3b& color_x, const cv::Vec3b& color_y, int thickness = 1) const
        {
            if (length.x <= 0 || length.y <= 0 || thickness <= 0) return false;
            if (image.empty()) clearCanvas(image);

            cv::Point origin = cvtValue2Pixel(cv::Point2d(0, 0)) + cv::Point2d(0.5, 0.5);        // + 0.5: Rounding
            cv::Point axes_x = cvtValue2Pixel(cv::Point2d(length.x, 0)) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::Point axes_y = cvtValue2Pixel(cv::Point2d(0, length.y)) + cv::Point2d(0.5, 0.5); // + 0.5: Rounding
            cv::line(image, origin, axes_x, color_x, thickness);
            cv::line(image, origin, axes_y, color_y, thickness);
            return true;
        }

        bool drawOrigin(cv::Mat& image, double length, const cv::Vec3b& color_x, const cv::Vec3b& color_y, int thickness = 1) const
        {
            return drawOrigin(image, cv::Point2d(length, length), color_x, color_y, thickness);
        }

        bool drawAxes(cv::Mat& image, const cv::Vec3b& color_x, const cv::Vec3b& color_y, int thickness = 1) const
        {
            if (thickness <= 0) return false;
            if (image.empty()) clearCanvas(image);

            cv::Point origin = cvtValue2Pixel(cv::Point2d(0, 0)) + cv::Point2d(0.5, 0.5);        // + 0.5: Rounding
            cv::line(image, cv::Point2d(m_img_box.tl().x, origin.y), cv::Point2d(m_img_box.br().x, origin.y), color_x, thickness); // The X axis
            cv::line(image, cv::Point2d(origin.x, m_img_box.tl().y), cv::Point2d(origin.x, m_img_box.br().y), color_y, thickness); // The Y axis
            return true;
        }

        bool drawGrid(cv::Mat& image, const cv::Point2d& grid_step, const cv::Vec3b& color, int thickness = 1, double unit_font_scale = 0.5, const cv::Vec3b& unit_color = cv::Vec3b(0, 0, 0), cv::Point unit_pos = cv::Point(20, 15)) const
        {
            if (grid_step.x <= 0 || grid_step.y <= 0 || thickness <= 0) return false;
            if (image.empty()) clearCanvas(image);

            cv::Point2d center_grid = cvtValue2Pixel(cv::Point2d(0, 0));
            for (int i = 0;; i++)
            {
                double y = center_grid.y - grid_step.y * i * m_px_per_val.y;
                if (y < m_img_box.tl().y || y > m_img_box.br().y) break;
                cv::line(image, cv::Point2d(m_img_box.tl().x, y), cv::Point2d(m_img_box.br().x, y), color, thickness); // Upward X-directional grids
            }
            for (int i = 1;; i++)
            {
                double y = center_grid.y + grid_step.y * i * m_px_per_val.y;
                if (y < m_img_box.tl().y || y > m_img_box.br().y) break;
                cv::line(image, cv::Point2d(m_img_box.tl().x, y), cv::Point2d(m_img_box.br().x, y), color, thickness); // Downward X-directional grids
            }
            for (int i = 0;; i++)
            {
                double x = center_grid.x + grid_step.x * i * m_px_per_val.x;
                if (x < m_img_box.tl().x || x > m_img_box.br().x) break;
                cv::line(image, cv::Point2d(x, m_img_box.tl().y), cv::Point2d(x, m_img_box.br().y), color, thickness); // Rightward Y-directional grids
            }
            for (int i = 1;; i++)
            {
                double x = center_grid.x - grid_step.x * i * m_px_per_val.x;
                if (x < m_img_box.tl().x || x > m_img_box.br().x) break;
                cv::line(image, cv::Point2d(x, m_img_box.tl().y), cv::Point2d(x, m_img_box.br().y), color, thickness); // Leftward Y-directional grids
            }
            if (unit_font_scale > 0)
            {
                if (unit_pos.x < 0) unit_pos.x = image.cols + unit_pos.x;
                if (unit_pos.y < 0) unit_pos.y = image.rows + unit_pos.y;
                cv::putText(image, cv::format("XGrid: %.1f, YGrid: %.1f", grid_step.x, grid_step.y), unit_pos, cv::FONT_HERSHEY_DUPLEX, unit_font_scale, unit_color);
            }
            return true;
        }

        static bool pasteImage(cv::Mat& image, const cv::Mat& stamp, const cv::Point& offset, double alpha = 1)
        {
            if (image.empty() || stamp.empty()) return false;

            cv::Mat stamp_img = stamp;
            if (image.channels() != stamp_img.channels())
            {
                int code = cv::COLOR_BGR2GRAY;
                if (image.channels() == 3) code = cv::COLOR_GRAY2BGR;
                cv::cvtColor(stamp_img, stamp_img, code);
            }

            cv::Rect stamp_box(offset.x, offset.y, stamp_img.cols, stamp_img.rows);
            cv::Rect image_box(cv::Point(), image.size());
            cv::Rect dst_box = stamp_box & image_box;
            cv::Rect src_box = (stamp_box - offset) & (image_box - offset);
            image(dst_box) = (1 - alpha) * image(dst_box) + alpha * stamp_img(src_box);
            return true;
        }

        static std::vector<cv::Point2d> getPtsVector(const cv::InputArray _data)
        {
            if (_data.empty()) return std::vector<cv::Point2d>();
            cv::Mat data = _data.getMat();
            if (data.depth() != CV_64F) data.convertTo(data, CV_64F);

            if (data.channels() == 2) return data;
            if (data.rows == 2 && data.cols != 2) data = data.t();
            if (data.cols != 2) return std::vector<cv::Point2d>();
            return data.reshape(2);
        }

        static bool getRange(const cv::InputArray data, RangeDbl& range_x, RangeDbl& range_y)
        {
            std::vector<cv::Point2d> pts = getPtsVector(data);
            if (pts.empty()) return false;

            // Find the minimum and maximum of data
            range_x.min = pts.front().x;
            range_x.max = pts.front().x;
            range_y.min = pts.front().y;
            range_y.max = pts.front().y;
            for (auto pt = pts.begin(); pt != pts.end(); pt++)
            {
                if (pt->x < range_x.min) range_x.min = pt->x;
                if (pt->x > range_x.max) range_x.max = pt->x;
                if (pt->y < range_y.min) range_y.min = pt->y;
                if (pt->y > range_y.max) range_y.max = pt->y;
            }
            return true;
        }

    protected:
        static cv::Rect getInnerBox(const cv::Size& sz, const cv::Point& margin) { return cv::Rect(margin.x, margin.y, sz.width - 2 * margin.x, sz.height - 2 * margin.y); }

        static cv::Point2d getOffset(const cv::Point2d& min_val, const cv::Point2d& ppv, const cv::Size& sz, const cv::Point& margin, const cv::Point& padding) { return cv::Point2d(padding.x + margin.x - min_val.x * ppv.x, sz.height - (padding.y + margin.y - min_val.y * ppv.y)); }

        cv::Size m_img_size;

        cv::Point m_img_padding;

        cv::Point m_img_margin;

        cv::Rect m_img_box;

        cv::Point2d m_img_offset;

        cv::Point2d m_min_val;

        cv::Point2d m_px_per_val;

        double m_img_rotation = 0;  // radian
    };

    /**
     * Convert an angle's unit from radian to degree
     * @param radian An angle in radian unit
     * @return The angle in degree unit
     */
    inline double cvtRad2Deg(double radian) { return radian * 180 / CV_PI; }

    /**
     * Convert an angle's unit from degree to radian
     * @param degree An angle in degree unit
     * @return The angle in radian unit
     */
    inline double cvtDeg2Rad(double degree) { return degree * CV_PI / 180; }

    /**
     * Make an angle within [-CV_PI, CV_PI)
     * @param radian An angle in radian unit
     * @return The trimmed angle in radian unit
     */
    inline double trimRad(double radian)
    {
        radian -= static_cast<int>(radian / (2 * CV_PI)) * (2 * CV_PI);
        if (radian >= CV_PI) radian -= 2 * CV_PI;
        if (radian < -CV_PI) radian += 2 * CV_PI;
        return radian;
    }

    /**
     * Return an x-axis rotation matrix
     * @param theta The given angle (Unit: [rad])
     * @return A 3D rotation matrix
     */
    inline cv::Matx33d Rx(double theta)
    {
        double c = cos(theta), s = sin(theta);
        return cv::Matx33d(1, 0, 0, 0, c, -s, 0, s, c);
    }

    /**
     * Return an y-axis rotation matrix
     * @param theta The given angle (Unit: [rad])
     * @return A 3D rotation matrix
     */
    inline cv::Matx33d Ry(double theta)
    {
        double c = cos(theta), s = sin(theta);
        return cv::Matx33d(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    /**
     * Return an z-axis rotation matrix
     * @param theta The given angle (Unit: [rad])
     * @return A 3D rotation matrix
     */
    inline cv::Matx33d Rz(double theta)
    {
        double c = cos(theta), s = sin(theta);
        return cv::Matx33d(c, -s, 0, s, c, 0, 0, 0, 1);
    }

    /**
     * Convert a quaternion vector to a 3D rotation matrix 
     * @param w The 1st component in the given quaternion vector
     * @param x The 2nd component in the given quaternion vector
     * @param y The 3rd component in the given quaternion vector
     * @param z The 4th component in the given quaternion vector
     * @return The converted 3D rotation matrix
     */
    inline cv::Matx33d cvtQuat2R(double w, double x, double y, double z)
    {
        // Reference) http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        double norm2 = w * w + x * x + y * y + z * z;
        double coeff = (norm2 == 0) ? 0 : 2 / norm2;
        double _2xx = coeff * x * x;
        double _2xy = coeff * x * y;
        double _2xz = coeff * x * z;
        double _2xw = coeff * w * x;
        double _2yy = coeff * y * y;
        double _2yz = coeff * y * z;
        double _2yw = coeff * w * y;
        double _2zz = coeff * z * z;
        double _2zw = coeff * w * z;

        return cv::Matx33d(
            1 - _2yy - _2zz, _2xy - _2zw,     _2xz + _2yw,
            _2xy + _2zw,     1 - _2xx - _2zz, _2yz - _2xw,
            _2xz - _2yw,     _2yz + _2xw,     1 - _2xx - _2yy);
    }

    /**
     * Convert a quaternion vector to a 3D rotation matrix 
     * @param q The given quaternion vector
     * @return The converted 3D rotation matrix
     */
    inline cv::Matx33d cvtQuat2R(const cv::Vec4d& q)
    {
        return cvtQuat2R(q(0), q(1), q(2), q(3));
    }

    /**
     * Convert a quaternion vector to Euler angles
     * @param w The 1st component in the given quaternion vector
     * @param x The 2nd component in the given quaternion vector
     * @param y The 3rd component in the given quaternion vector
     * @param z The 4th component in the given quaternion vector
     * @return The converted Euler angles
     */
    inline cv::Point3d cvtQuat2EulerAng(double w, double x, double y, double z)
    {
        // Reference) https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        cv::Point3d euler;
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler.x = atan2(sinr_cosp, cosr_cosp);
        double sinp = 2 * (w * y - z * x);
        if (sinp >=  1) euler.y =  CV_PI;
        if (sinp <= -1) euler.y = -CV_PI;
        else            euler.y = asin(sinp);
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler.z = atan2(siny_cosp, cosy_cosp);
        return euler;
    }

    /**
     * Convert a quaternion vector to Euler angles
     * @param q The given quaternion vector
     * @return The converted Euler angles
     */
    inline cv::Point3d cvtQuat2EulerAng(const cv::Vec4d& q)
    {
        return cvtQuat2EulerAng(q(0), q(1), q(2), q(3));
    }

    /**
     * Convert a 3D rotation matrix to a quaternion vector
     * @param R The given 3D rotation matrix
     * @return The converted quaternion vector
     */
    inline cv::Vec4d cvtR2Quat(const cv::Matx33d& R)
    {
        // Reference) http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
        double trace = R(0, 0) + R(1, 1) + R(2, 2), w, x, y, z;
        if (trace > 0)
        {
            double s = sqrt(trace + 1) * 2;
            w = 0.25 * s;
            x = (R(2, 1) - R(1, 2)) / s;
            y = (R(0, 2) - R(2, 0)) / s;
            z = (R(1, 0) - R(0, 1)) / s;
        }
        else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
        {
            double s = sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
            w = (R(2, 1) - R(1, 2)) / s;
            x = 0.25 * s;
            y = (R(0, 1) + R(1, 0)) / s;
            z = (R(0, 2) + R(2, 0)) / s;
        }
        else if (R(1, 1) > R(2, 2))
        {
            double s = sqrt(1 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
            w = (R(0, 2) - R(2, 0)) / s;
            x = (R(0, 1) + R(1, 0)) / s;
            y = 0.25 * s;
            z = (R(1, 2) + R(2, 1)) / s;
        }
        else
        {
            double s = sqrt(1 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
            w = (R(1, 0) - R(0, 1)) / s;
            x = (R(0, 2) + R(2, 0)) / s;
            y = (R(1, 2) + R(2, 1)) / s;
            z = 0.25 * s;
        }
        return cv::Vec4d(w, x, y, z);
    }

    /**
     * Convert an axis-angle notation embedding angular magnitude (a.k.a. Rodrigue rotation notation) to a 3D rotation matrix
     * @param axis The given Rodrigue rotation vector
     * @return The converted 3D rotation matrix
     */
    inline cv::Matx33d cvtAxisAng2R(const cv::Vec3d& axis)
    {
        cv::Matx33d R;
        cv::Rodrigues(axis, R);
        return R;
    }

    /**
     * Convert a 3D rotation matrix to an axis-angle notation embedding angular magnitude (a.k.a. Rodrigue rotation notation)
     * @param R The given 3D rotation matrix
     * @return The converted Rodrigue rotation vector
     */
    inline cv::Vec3d cvtR2AxisAng(const cv::Matx33d& R)
    {
        cv::Vec3d axis;
        cv::Rodrigues(R, axis);
        return axis;
    }

    /** A color code for red */
    const cv::Vec3b COLOR_RED(0, 0, 255);

    /** A color code for green */
    const cv::Vec3b COLOR_GREEN(0, 255, 0);

    /** A color code for blue */
    const cv::Vec3b COLOR_BLUE(255, 0, 0);

    /** A color code for cyan */
    const cv::Vec3b COLOR_CYAN(255, 255, 0);

    /** A color code for magenta */
    const cv::Vec3b COLOR_MAGENTA(255, 0, 255);

    /** A color code for yellow */
    const cv::Vec3b COLOR_YELLOW(0, 255, 255);

    /** A color code for black */
    const cv::Vec3b COLOR_BLACK(0, 0, 0);

    /** A color code for white */
    const cv::Vec3b COLOR_WHITE(255, 255, 255);

    /** A color code for gray */
    const cv::Vec3b COLOR_GRAY(127, 127, 127);

    /** A key code for _Line Feed (LF)_ */
    const int KEY_LF = '\n';

    /** A key code for _Carriage Return (CR)_, usually _Enter_ */
    const int KEY_CR = '\r';

    /** A key code for _Tab_ */
    const int KEY_TAB = '\t';

    /** A key code for _Escape (ESC)_ */
    const int KEY_ESC = 0x1B;

    /** A key code for _Space_ */
    const int KEY_SPACE = 0x20;

} // End of 'cx'

#endif // End of '__OPEN_CX__'
