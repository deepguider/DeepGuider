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
        virtual bool isConfiged() const
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
            else if (isConfiged())
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

    /**
     * Return an x-axis rotation matrix
     * @param theta The given angle (Unit: [rad])
     * @return 3D rotation matrix
     */
    inline cv::Matx33d getRx(double theta)
    {
        double c = cos(theta), s = sin(theta);
        return cv::Matx33d(1, 0, 0, 0, c, -s, 0, s, c);
    }

    /**
     * Return an y-axis rotation matrix
     * @param theta The given angle (Unit: [rad])
     * @return 3D rotation matrix
     */
    inline cv::Matx33d getRy(double theta)
    {
        double c = cos(theta), s = sin(theta);
        return cv::Matx33d(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    /**
     * Return an z-axis rotation matrix
     * @param theta The given angle (Unit: [rad])
     * @return 3D rotation matrix
     */
    inline cv::Matx33d getRz(double theta)
    {
        double c = cos(theta), s = sin(theta);
        return cv::Matx33d(c, -s, 0, s, c, 0, 0, 0, 1);
    }

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

    /** A color code for black */
    const cv::Vec3b COLOR_BLACK(0, 0, 0);

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

    /** A color code for white */
    const cv::Vec3b COLOR_WHITE(255, 255, 255);

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
