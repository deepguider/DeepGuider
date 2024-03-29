#ifndef __DATA_LOADER_HPP__
#define __DATA_LOADER_HPP__

#include "core/basic_type.hpp"
#include "utils/opencx.hpp"

namespace dg
{

/**
 * @brief Type definitions of observed data
 */
enum
{
    /** GPS data (time, lat, lon) */
    DATA_GPS = 0,

    /** IMU data (time, w, x, y, z) */
    DATA_IMU = 1,

    /** POI data */
    DATA_POI = 2,

    /** VPS data */
    DATA_VPS = 3,

    /** Intersection classifier data (time, v, confidence) */
    DATA_IntersectCls = 4,

    /** RoadLR data (time, v, confidence) */
    DATA_RoadLR = 5,

    /** RoadTheta data */
    DATA_RoadTheta = 6,

    /** OCR data */
    DATA_OCR = 7,

    /** Odometry data (time, x, y, theta_radian) */
    DATA_ODO = 8
};

/**
 * @brief DataLoader
 *
 * This implement interfaces for loading and providing timestamped sensor data
 */
class DataLoader
{
public:
    bool load(const std::string& video_file, const std::string& gps_file, const std::string& odo_file = "", const std::string& ahrs_file = "", const std::string& ocr_file = "", const std::string& poi_file = "", const std::string& vps_file = "", const std::string& intersection_file = "", const std::string& roadlr_file = "", const std::string& roadtheta_file = "")
    {
        clear();

        if (!gps_file.empty())
        {
            const std::string ANDRO_POSTFIX = "AndroSensor.csv";
            const std::string postfix = gps_file.substr(gps_file.length() - ANDRO_POSTFIX.length(), ANDRO_POSTFIX.length());
            if (postfix.compare(ANDRO_POSTFIX) == 0)
                m_gps_data = readAndroGPS(gps_file);
            else
                m_gps_data = readROSGPSFix(gps_file);
            if (m_gps_data.empty()) return false;
        }
        if (!odo_file.empty())
        {
            m_odo_data = readROSOdometry(odo_file);
            if (m_odo_data.empty()) return false;
        }
        if (!ahrs_file.empty())
        {
            m_ahrs_data = readROSAHRS(ahrs_file);
            if (m_ahrs_data.empty()) return false;
        }
        if (!ocr_file.empty())
        {
            m_ocr_vdata = readOCR(ocr_file, m_ocr_sdata);
            if (m_ocr_vdata.empty()) return false;
        }
        if (!poi_file.empty())
        {
            m_poi_data = readPOI(poi_file);
            if (m_poi_data.empty()) return false;
        }
        if (!vps_file.empty())
        {
            m_vps_data = readVPS(vps_file);
            if (m_vps_data.empty()) return false;
        }
        if (!intersection_file.empty())
        {
            m_intersection_data = readIntersection(intersection_file);
            if (m_intersection_data.empty()) return false;
        }
        if (!roadlr_file.empty())
        {
            m_roadlr_data = readRoadLR(roadlr_file);
            if (m_roadlr_data.empty()) return false;
        }
        if (!roadtheta_file.empty())
        {
            m_roadtheta_data = readRoadTheta(roadtheta_file);
            if (m_roadtheta_data.empty()) return false;
        }

        if (!m_ahrs_data.empty()) m_first_data_time = m_ahrs_data.front()[0];
        else if (!m_gps_data.empty()) m_first_data_time = m_gps_data.front()[0];
        else m_first_data_time = 0;

        if (!video_file.empty())
        {
            if (!m_camera_data.open(video_file)) return false;
            m_video_fps = m_camera_data.get(cv::VideoCaptureProperties::CAP_PROP_FPS);
            m_total_frames = (int)m_camera_data.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);

            if (!m_ahrs_data.empty())
            {
                m_first_data_time = m_ahrs_data.front()[0];
                m_video_scale = m_video_fps * (m_ahrs_data.back()[0] - m_first_data_time) / m_total_frames;
            }
            else if (!m_gps_data.empty())
            {
                m_first_data_time = m_gps_data.front()[0];
                m_video_scale = m_video_fps * (m_gps_data.back()[0] - m_first_data_time) / m_total_frames;
            }
            else
            {
                m_video_scale = 1;
            }
        }
        return true;
    }

    /**
     * Get next datum in the order of time squence
     * @param[out] type The returned data type
     * @param[out] data The returned data
     * @param[out] timestamp The timestamp of the returned data
     */
    bool getNext(int& type, std::vector<double>& vdata, std::vector<std::string>& sdata, dg::Timestamp& timestamp)
    {
        double min_time = DBL_MAX;
        cx::CSVReader::Double2D* min_data = nullptr;
        cx::CSVReader::String2D* min_sdata = nullptr;
        size_t* min_index = nullptr;
        int min_type = -1;

        if (m_gps_index < m_gps_data.size() && m_gps_data[m_gps_index][0] < min_time)
        {
            min_time = m_gps_data[m_gps_index][0];
            min_data = &m_gps_data;
            min_index = &m_gps_index;
            min_type = DATA_GPS;
        }
        if (m_odo_index < m_odo_data.size() && m_odo_data[m_odo_index][0] < min_time)
        {
            min_time = m_odo_data[m_odo_index][0];
            min_data = &m_odo_data;
            min_index = &m_odo_index;
            min_type = DATA_ODO;
        }
        if (m_ahrs_index < m_ahrs_data.size() && m_ahrs_data[m_ahrs_index][0] < min_time)
        {
            min_time = m_ahrs_data[m_ahrs_index][0];
            min_data = &m_ahrs_data;
            min_index = &m_ahrs_index;
            min_type = DATA_IMU;
        }
        if (m_ocr_index < m_ocr_vdata.size() && m_ocr_vdata[m_ocr_index][0] < min_time)
        {
            min_time = m_ocr_vdata[m_ocr_index][0];
            min_data = &m_ocr_vdata;
            min_sdata = &m_ocr_sdata;
            min_index = &m_ocr_index;
            min_type = DATA_OCR;
        }
        if (m_poi_index < m_poi_data.size() && m_poi_data[m_poi_index][0] < min_time)
        {
            min_time = m_poi_data[m_poi_index][0];
            min_data = &m_poi_data;
            min_index = &m_poi_index;
            min_type = DATA_POI;
        }
        if (m_vps_index < m_vps_data.size() && m_vps_data[m_vps_index][0] < min_time)
        {
            min_time = m_vps_data[m_vps_index][0];
            min_data = &m_vps_data;
            min_index = &m_vps_index;
            min_type = DATA_VPS;
        }
        if (m_intersection_index < m_intersection_data.size() && m_intersection_data[m_intersection_index][0] < min_time)
        {
            min_time = m_intersection_data[m_intersection_index][0];
            min_data = &m_intersection_data;
            min_index = &m_intersection_index;
            min_type = DATA_IntersectCls;
        }
        if (m_roadlr_index < m_roadlr_data.size() && m_roadlr_data[m_roadlr_index][0] < min_time)
        {
            min_time = m_roadlr_data[m_roadlr_index][0];
            min_data = &m_roadlr_data;
            min_index = &m_roadlr_index;
            min_type = DATA_RoadLR;
        }
        if (m_roadtheta_index < m_roadtheta_data.size() && m_roadtheta_data[m_roadtheta_index][0] < min_time)
        {
            min_time = m_roadtheta_data[m_roadtheta_index][0];
            min_data = &m_roadtheta_data;
            min_index = &m_roadtheta_index;
            min_type = DATA_RoadTheta;
        }

        if (min_data)
        {
            if (min_sdata) sdata = (*min_sdata)[*min_index];
            vdata = (*min_data)[*min_index];
            type = min_type;
            timestamp = min_time;
            (*min_index)++;
            return true;
        }
        return false;
    }

    /**
     * Get next datum as long as its timestamp doesn't exceed a given reference time.
     * @param[in] ref_time A givan reference time
     * @param[out] type The returned data type
     * @param[out] data The returned data
     * @param[out] timestamp The timestamp of the returned data
     */
    bool getNextUntil(dg::Timestamp ref_time, int& type, std::vector<double>& vdata, std::vector<std::string>& sdata, dg::Timestamp& timestamp)
    {
        double min_time = ref_time;
        cx::CSVReader::Double2D* min_data = nullptr;
        cx::CSVReader::String2D* min_sdata = nullptr;
        size_t* min_index = nullptr;
        int min_type = -1;

        if (m_gps_index < m_gps_data.size() && m_gps_data[m_gps_index][0] <= min_time)
        {
            min_time = m_gps_data[m_gps_index][0];
            min_data = &m_gps_data;
            min_index = &m_gps_index;
            min_type = DATA_GPS;
        }
        if (m_odo_index < m_odo_data.size() && m_odo_data[m_odo_index][0] <= min_time)
        {
            min_time = m_odo_data[m_odo_index][0];
            min_data = &m_odo_data;
            min_index = &m_odo_index;
            min_type = DATA_ODO;
        }
        if (m_ahrs_index < m_ahrs_data.size() && m_ahrs_data[m_ahrs_index][0] <= min_time)
        {
            min_time = m_ahrs_data[m_ahrs_index][0];
            min_data = &m_ahrs_data;
            min_index = &m_ahrs_index;
            min_type = DATA_IMU;
        }
        if (m_ocr_index < m_ocr_vdata.size() && m_ocr_vdata[m_ocr_index][0] <= min_time)
        {
            min_time = m_ocr_vdata[m_ocr_index][0];
            min_data = &m_ocr_vdata;
            min_sdata = &m_ocr_sdata;
            min_index = &m_ocr_index;
            min_type = DATA_OCR;
        }
        if (m_poi_index < m_poi_data.size() && m_poi_data[m_poi_index][0] <= min_time)
        {
            min_time = m_poi_data[m_poi_index][0];
            min_data = &m_poi_data;
            min_index = &m_poi_index;
            min_type = DATA_POI;
        }
        if (m_vps_index < m_vps_data.size() && m_vps_data[m_vps_index][0] <= min_time)
        {
            min_time = m_vps_data[m_vps_index][0];
            min_data = &m_vps_data;
            min_index = &m_vps_index;
            min_type = DATA_VPS;
        }
        if (m_intersection_index < m_intersection_data.size() && m_intersection_data[m_intersection_index][0] <= min_time)
        {
            min_time = m_intersection_data[m_intersection_index][0];
            min_data = &m_intersection_data;
            min_index = &m_intersection_index;
            min_type = DATA_IntersectCls;
        }
        if (m_roadlr_index < m_roadlr_data.size() && m_roadlr_data[m_roadlr_index][0] <= min_time)
        {
            min_time = m_roadlr_data[m_roadlr_index][0];
            min_data = &m_roadlr_data;
            min_index = &m_roadlr_index;
            min_type = DATA_RoadLR;
        }
        if (m_roadtheta_index < m_roadtheta_data.size() && m_roadtheta_data[m_roadtheta_index][0] <= min_time)
        {
            min_time = m_roadtheta_data[m_roadtheta_index][0];
            min_data = &m_roadtheta_data;
            min_index = &m_roadtheta_index;
            min_type = DATA_RoadTheta;
        }

        if (min_data)
        {
            if (min_sdata) sdata = (*min_sdata)[*min_index];
            vdata = (*min_data)[*min_index];
            type = min_type;
            timestamp = min_time;
            (*min_index)++;
            return true;
        }
        return false;
    }

    /**
     * Get image frame that is closest to a given timestamp
     * @param time The input timestamp
     */
    cv::Mat getFrame(const Timestamp time)
    {
        cv::Mat frame;
        if (m_camera_data.isOpened())
        {
            int frame_i = (int)((time - m_first_data_time) * m_video_fps / m_video_scale + 0.5);
            m_camera_data.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, frame_i);
            m_camera_data >> frame;
        }
        return frame;
    }

    /**
     * Get image frame that is closest to a given timestamp
     * @param time The input timestamp
     * @param fnumber The frame number of the returned image frame
     */
    cv::Mat getFrame(const Timestamp time, int& fnumber)
    {
        cv::Mat frame;
        if (m_camera_data.isOpened())
        {
            int frame_i = (int)((time - m_first_data_time) * m_video_fps / m_video_scale + 0.5);
            m_camera_data.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, frame_i);
            m_camera_data >> frame;
            fnumber = frame_i;
        }
        return frame;
    }

    /**
     * Get a next image frame from the camera data
     * @param[out] time The timestamp of the returned image frame
     * @param fnumber The frame number of the returned image frame
     */
    cv::Mat getNextFrame(Timestamp& time, int& fnumber)
    {
        cv::Mat frame;
        if (m_camera_data.isOpened() && m_frame_index < m_total_frames)
        {
            m_camera_data.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, m_frame_index);
            m_camera_data >> frame;
            time = m_frame_index * m_video_scale / m_video_fps + m_first_data_time;
            fnumber = m_frame_index;
            m_frame_index++;
        }
        return frame;
    }

    /**
     * Get a next image frame from the camera data
     * @param[out] time The timestamp of the returned image frame
     */
    cv::Mat getNextFrame(Timestamp& time)
    {
        cv::Mat frame;
        if (m_camera_data.isOpened() && m_frame_index < m_total_frames)
        {
            m_camera_data.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, m_frame_index);
            m_camera_data >> frame;
            time = m_frame_index * m_video_scale / m_video_fps + m_first_data_time;
            m_frame_index++;
        }
        return frame;
    }

    /**
     * Change the start time of data
     * @param skip_time The amount of time to skip from the begining (Unit: [sec])
     */
    void setStartSkipTime(double skip_time)
    {
        if (m_first_data_time < 0) return;  // there is no loaded data

        m_gps_index = 0;
        m_odo_index = 0;
        m_ahrs_index = 0;
        m_intersection_index = 0;
        m_vps_index = 0;
        m_roadlr_index = 0;
        m_ocr_index = 0;
        m_poi_index = 0;
        m_roadtheta_index = 0;
        m_frame_index = 0;

        if (skip_time <= 0) return;

        double start_time = m_first_data_time + skip_time;
        if (!m_gps_data.empty())
        {
            while (m_gps_index<m_gps_data.size() && m_gps_data[m_gps_index][0] < start_time) m_gps_index++;
        }
        if (!m_odo_data.empty())
        {
            while (m_odo_index < m_odo_data.size() && m_odo_data[m_odo_index][0] < start_time) m_odo_index++;
        }
        if (!m_ahrs_data.empty())
        {
            while (m_ahrs_index < m_ahrs_data.size() && m_ahrs_data[m_ahrs_index][0] < start_time) m_ahrs_index++;
        }
        if (!m_ocr_vdata.empty())
        {
            while (m_ocr_index < m_ocr_vdata.size() && m_ocr_vdata[m_ocr_index][0] < start_time) m_ocr_index++;
        }
        if (!m_poi_data.empty())
        {
            while (m_poi_index < m_poi_data.size() && m_poi_data[m_poi_index][0] < start_time) m_poi_index++;
        }
        if (!m_vps_data.empty())
        {
            while (m_vps_index < m_vps_data.size() && m_vps_data[m_vps_index][0] < start_time) m_vps_index++;
        }
        if (!m_intersection_data.empty())
        {
            while (m_intersection_index < m_intersection_data.size() && m_intersection_data[m_intersection_index][0] < start_time) m_intersection_index++;
        }
        if (!m_roadlr_data.empty())
        {
            while (m_roadlr_index < m_roadlr_data.size() && m_roadlr_data[m_roadlr_index][0] < start_time) m_roadlr_index++;
        }
        if (!m_roadtheta_data.empty())
        {
            while (m_roadtheta_index < m_roadtheta_data.size() && m_roadtheta_data[m_roadtheta_index][0] < start_time) m_roadtheta_index++;
        }
        if (m_camera_data.isOpened())
        {
            m_frame_index = (int)((start_time - m_first_data_time) * m_video_fps / m_video_scale + 0.5);
        }
    }

    double getStartTime() const { return m_first_data_time; }

    bool empty() const
    {
        return !m_camera_data.isOpened() && m_gps_data.empty() && m_odo_data.empty() && m_ahrs_data.empty() && m_ocr_sdata.empty() && m_ocr_vdata.empty() && m_poi_data.empty() && m_vps_data.empty() && m_intersection_data.empty() && m_roadlr_data.empty() && m_roadtheta_data.empty();
    }

protected:
    void clear()
    {
        m_camera_data.release();
        m_gps_data.clear();
        m_odo_data.clear();
        m_ahrs_data.clear();
        m_ocr_vdata.clear();
        m_ocr_sdata.clear();
        m_poi_data.clear();
        m_vps_data.clear();
        m_intersection_data.clear();
        m_roadlr_data.clear();
        m_roadtheta_data.clear();

        m_gps_index = 0;
        m_odo_index = 0;
        m_ahrs_index = 0;
        m_ocr_index = 0;
        m_poi_index = 0;
        m_vps_index = 0;
        m_intersection_index = 0;
        m_roadlr_index = 0;
        m_roadtheta_index = 0;
        m_video_scale = 1;
        m_video_fps = -1;
        m_first_data_time = -1;
        m_total_frames = 0;
        m_frame_index = 0;
    }

    cx::CSVReader::Double2D readROSGPSFix(const std::string& gps_file)
    {
        cx::CSVReader::Double2D data;
        cx::CSVReader csv;
        if (csv.open(gps_file))
        {
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 2, 3, 5, 7, 8 }); // Skip the header
            if (!raw_data.empty())
            {
                for (auto row = raw_data.begin(); row != raw_data.end(); row++)
                {
                    double status = row->at(2);
                    if (status < 0) continue;   // skip nan data

                    double timestamp = row->at(0) + 1e-9 * row->at(1);
                    dg::LatLon ll(row->at(3), row->at(4));
                    std::vector<double> datum = { timestamp, ll.lat, ll.lon };
                    data.push_back(datum);
                }
            }
        }
        return data;
    }

    cx::CSVReader::Double2D readAndroGPS(const std::string& gps_file)
    {
        cx::CSVReader::Double2D data;
        cx::CSVReader csv;
        if (csv.open(gps_file, ';'))
        {
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(2, { 31, 22, 23, 28 }); // Skip the header
            if (!raw_data.empty())
            {
                for (auto row = raw_data.begin(); row != raw_data.end(); row++)
                {
                    double timestamp = 1e-3 * row->at(0);
                    dg::LatLon ll(row->at(1), row->at(2));
                    double accuracy = row->at(3);
                    std::vector<double> datum = { timestamp, ll.lat, ll.lon, accuracy };
                    data.push_back(datum);
                }
            }
        }
        return data;
    }

    cx::CSVReader::Double2D readROSOdometry(const std::string& odo_file)
    {
        cx::CSVReader::Double2D data;
        cx::CSVReader csv;
        if (csv.open(odo_file))
        {
            // time,seq,secs,nsecs,frame_id,child_frame_id,pos.x,pos.y,pos.z,.orientation.x,orientation.y,orientation.z,orientation.w
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 2, 3, 6, 7, 11 }); // Skip the header
            if (!raw_data.empty())
            {
                for (auto row = raw_data.begin(); row != raw_data.end(); row++)
                {
                    double timestamp = row->at(0) + 1e-9 * row->at(1);
                    data.push_back({ timestamp, row->at(2), row->at(3), row->at(4) }); // t,x,y,theta
                }
            }
        }
        return data;
    }

    cx::CSVReader::Double2D readROSAHRS(const std::string& ahrs_file)
    {
        cx::CSVReader::Double2D data;
        cx::CSVReader csv;
        if (csv.open(ahrs_file))
        {
            // time,seq,secs,nsecs,frame_id,orientation.x,orientation.y,orientation.z,.orientation.w
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 2, 3, 5, 6, 7, 8 }); // Skip the header
            if (!raw_data.empty())
            {
                for (auto row = raw_data.begin(); row != raw_data.end(); row++)
                {
                    double timestamp = row->at(0) + 1e-9 * row->at(1);
                    data.push_back({ timestamp, row->at(5), row->at(2), row->at(3), row->at(4) }); // t, quaternion w,x,y,z
                }
            }
        }
        return data;
    }

    cx::CSVReader::Double2D readIntersection(const std::string& clue_file)
    {
        cx::CSVReader csv;
        if (csv.open(clue_file))
        {
            // framenumber,timestamp,v,confidence
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 1, 2, 3 }); // Skip the header
            return raw_data;
        }
        return cx::CSVReader::Double2D();
    }

    cx::CSVReader::Double2D readRoadLR(const std::string& clue_file)
    {
        cx::CSVReader csv;
        if (csv.open(clue_file))
        {
            // fnumber,timestamp,v,confidence,lr_side(string)
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 1, 2, 3 }); // Skip the header
            return raw_data;
        }
        return cx::CSVReader::Double2D();
    }

    cx::CSVReader::Double2D readRoadTheta(const std::string& clue_file)
    {
        cx::CSVReader csv;
        if (csv.open(clue_file))
        {
            // fnumber,timestamp,vx,vy,theta,confidence
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 1, 2, 3, 4, 5 }); // Skip the header
            return raw_data;
        }
        return cx::CSVReader::Double2D();
    }

    cx::CSVReader::Double2D readOCR(const std::string& clue_file, cx::CSVReader::String2D& sdata)
    {
        cx::CSVReader csv;
        sdata.clear();
        if (csv.open(clue_file))
        {    
            // fnumber,timestamp,dname,confidence,xmin,ymin,xmax,ymax,lat,lon
            sdata = csv.extString2D(1, { 2 }); // Skip the header
            cx::CSVReader::Double2D raw_vdata = csv.extDouble2D(1, { 1, 3, 4, 5, 6, 7 }); // Skip the header
            return raw_vdata;
        }
        return cx::CSVReader::Double2D();
    }

    cx::CSVReader::Double2D readPOI(const std::string& clue_file)
    {
        cx::CSVReader csv;
        if (csv.open(clue_file))
        {
            // fnumber,timestamp,poi_id,poi_x,poi_y,distance,angle,confidence
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 1, 2, 3, 4, 5, 6, 7 }); // Skip the header
            return raw_data;
        }
        return cx::CSVReader::Double2D();
    }

    cx::CSVReader::Double2D readVPS(const std::string& clue_file)
    {
        cx::CSVReader csv;
        if (csv.open(clue_file))
        {
            // fnumber,timestamp,svid,svidx,pred_lat,pred_lon,distance,angle,confidence,curr_lat,curr_lon,utm_err
            cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 1, 2, 3, 4, 5, 6, 7, 8 }); // Skip the header
            return raw_data;
        }
        return cx::CSVReader::Double2D();
    }

    cv::VideoCapture m_camera_data;
    cx::CSVReader::Double2D m_gps_data;
    cx::CSVReader::Double2D m_odo_data;
    cx::CSVReader::Double2D m_ahrs_data;
    cx::CSVReader::String2D m_ocr_sdata;
    cx::CSVReader::Double2D m_ocr_vdata;
    cx::CSVReader::Double2D m_poi_data;
    cx::CSVReader::Double2D m_vps_data;
    cx::CSVReader::Double2D m_intersection_data;
    cx::CSVReader::Double2D m_roadlr_data;
    cx::CSVReader::Double2D m_roadtheta_data;

    size_t m_gps_index = 0;
    size_t m_odo_index = 0;
    size_t m_ahrs_index = 0;
    size_t m_ocr_index = 0;
    size_t m_poi_index = 0;
    size_t m_vps_index = 0;
    size_t m_intersection_index = 0;
    size_t m_roadlr_index = 0;
    size_t m_roadtheta_index = 0;
    double m_first_data_time = -1;
    double m_video_scale = 1;
    double m_video_fps = -1;
    int m_total_frames = 0;
    int m_frame_index = 0;

}; // End of 'DataLoader'

} // End of 'dg'


#endif // End of '__DATA_LOADER_HPP__'
