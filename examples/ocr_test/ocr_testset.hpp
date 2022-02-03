#ifndef __OCR_TESTSET__
#define __OCR_TESTSET__

#include "dg_localizer.hpp"
#include "dg_utils.hpp"
#include "localizer/data_loader.hpp"
#include "vps/vps_localizer.hpp"
#include "ocr_recog/ocr_localizer.hpp"

using namespace dg;
using namespace std;

/**
 * @brief DataLoader4Rec
 *
 * This implement interfaces for loading and providing timestamped sensor data for recording
 */
class DataLoader4Rec : public dg::DataLoader
{
public:
    // void fixVideoScale()
    // {
    //     m_first_data_time = m_ocr_vdata.front()[0];
    //     m_video_scale = m_video_fps * (m_ocr_vdata.back()[0] - m_first_data_time) / m_total_frames;
    // }

    int getFrameNumber(const Timestamp time)
    {
        int frame_i = 0;
        if (m_camera_data.isOpened())
        {
            frame_i = (int)((time - m_first_data_time) * m_video_fps / m_video_scale + 0.5);
        }
        return frame_i;
    }

    cx::CSVReader::Double2D readOCR4Rec(const std::string& clue_file, cx::CSVReader::String2D& sdata)
    {
        cx::CSVReader csv;
        sdata.clear();
        if (csv.open(clue_file))
        {    
            // fnumber,timestamp,dname,confidence,xmin,ymin,xmax,ymax,lat,lon
            sdata = csv.extString2D(1, { 2 }); // Skip the header
            cx::CSVReader::Double2D raw_vdata = csv.extDouble2D(1, { 0, 1, 3, 4, 5, 6, 7 }); // Skip the header
            return raw_vdata;
        }
        return cx::CSVReader::Double2D();
    }

    bool load4Rec(const std::string& ocr_file = "")
    {
        clear();

        if (!ocr_file.empty())
        {
            m_ocr_vdata = readOCR4Rec(ocr_file, m_ocr_sdata);
            if (m_ocr_vdata.empty()) return false;
        }
        
        return true;
    }
}; // End of 'DataLoader4Rec'


class OCRTestset : public dg::SharedInterface
{
    cv::Ptr<dg::DGLocalizer> m_localizer;
    cv::Ptr<dg::OCRLocalizer> m_ocr_localizer;

public:
    int runOCRloc(DataLoader4Rec& data_loader, const std::string& rec_traj_file = "")
    {
        // Configure localizer
        cv::Ptr<dg::DGLocalizer> localizer = cv::makePtr<dg::DGLocalizer>();
        if (!localizer->setParamMotionNoise(1, 10)) return -1;      // linear_velocity(m), angular_velocity(deg)
        if (!localizer->setParamGPSNoise(1)) return -1;             // position error(m)
        if (!localizer->setParamGPSOffset(1, 0)) return -1;         // displacement(lin,ang) from robot origin
        if (!localizer->setParamIMUCompassNoise(1, 0)) return -1;   // angle arror(deg), angle offset(deg)
        if (!localizer->setParamPOINoise(1, 10, 1)) return -1;      // rel. distance error(m), rel. orientation error(deg), position error of poi info (m)
        if (!localizer->setParamVPSNoise(1, 10, 1)) return -1;      // rel. distance error(m), rel. orientation error(deg), position error of poi info (m)
        if (!localizer->setParamIntersectClsNoise(0.1)) return -1;  // position error(m)
        if (!localizer->setParamRoadThetaNoise(10, 0)) return -1;   // angle arror(deg), angle offset(deg)
        if (!localizer->setParamCameraOffset(1, 0)) return -1;      // displacement(lin,ang) from robot origin
        localizer->setParamValue("gps_reverse_vel", -0.5);
        localizer->setParamValue("search_turn_weight", 100);
        localizer->setParamValue("track_near_radius", 20);
        localizer->setParamValue("enable_path_projection", true);
        localizer->setParamValue("enable_map_projection", true);
        localizer->setParamValue("enable_backtracking_ekf", true);
        localizer->setParamValue("enable_gps_smoothing", true);
        localizer->setParamValue("enable_debugging_display", true);
        localizer->setParamValue("lr_mismatch_cost", 50);
        localizer->setParamValue("enable_lr_reject", false);
        localizer->setParamValue("lr_reject_cost", 20);             // 20
        localizer->setParamValue("enable_discontinuity_cost", true);
        localizer->setParamValue("discontinuity_weight", 0.5);      // 0.5

        // initialize localizer
        m_localizer = localizer;
        m_localizer->initialize(this, "EKFLocalizerHyperTan");
        // initialize module localizers
        m_ocr_localizer = cv::makePtr<dg::OCRLocalizer>();
        VVS_CHECK_TRUE(m_ocr_localizer->initialize_without_python(this));

        // Prepare the result trajectory
        FILE* out_traj = nullptr;
        
        if (!rec_traj_file.empty())
        {
            string file_name = "./result/" + rec_traj_file;
            out_traj = fopen(file_name.c_str(), "wt");//, ccs=UTF-8");
            if (out_traj == nullptr) return -1;
            fprintf(out_traj, "fnumber,timestamp,poi_id,poi_x,poi_y,distance,angle,confidence\n");
            //fprintf(out_traj, "fnumber,timestamp,dname,x,y,w,h,poi_id,poi_name,poi_x,poi_y,poi_floor,distance,angle,confidence,cam_lat,cam_lon\n");
        }
        
        int type;
        std::vector<double> vdata;
        std::vector<std::string> sdata;
        dg::Timestamp data_time;
        //std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
        
        while (1)
        {
            if (data_loader.getNext(type, vdata, sdata, data_time) == false) break;

            double timestamp = vdata[1]; 
            //int fnumber = data_loader.getFrameNumber(data_time);
            int fnumber = vdata[0];
            std::string dname = sdata[0];
            double conf = vdata[2];
            double xmin = vdata[3];
            double ymin = vdata[4];
            double xmax = vdata[5];
            double ymax = vdata[6];      
            //double cam_lat = vdata[7];
            //double cam_lon = vdata[8];     
            dg::Point2 poi_xy;
            //std::vector<POI> pois;
            Polar2 relatives;
            double poi_confidences;

            // // poi 
            // double x = (xmin + xmax) / 2.0;
            // double y = (ymin + ymax) / 2.0;
            // double w = xmax - xmin;
            // double h = ymax - ymin;

            // bool ok = localizer.applyLoc(data, pois, relatives, poi_confidences);        
            bool ok = m_ocr_localizer->applyPreprocessed(dname, xmin, ymin, xmax, ymax, conf, timestamp, poi_xy, relatives, poi_confidences);
            
            // Record the current state on the CSV file
            if (out_traj != nullptr)
            {
                if(ok) //pois.empty())
                {
                    printf("%d,%.3lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,poi_xy.x,poi_xy.y,relatives.lin,relatives.ang,poi_confidences);
                    fprintf(out_traj, "%d,%.3lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,poi_xy.x,poi_xy.y,relatives.lin,relatives.ang,poi_confidences);       
                    // printf("%d,%.3lf,%s,%.2lf,%.2lf,%.2lf,%.2lf,%d,%s,%.2lf,%.2lf,%d,%.2lf,%.2lf,%.2lf,%.7lf,%.7lf\n", fnumber,timestamp,dname.c_str(),x,y,w,h,(int)poi->id,(converter.to_bytes(poi->name)).c_str(),poi->x,poi->y,poi->floor,relatives.lin,relatives.ang,poi_confidences,cam_lat,cam_lon);
                    // fprintf(out_traj, "%d,%.3lf,%s,%.2lf,%.2lf,%.2lf,%.2lf,%d,%s,%.2lf,%.2lf,%d,%.2lf,%.2lf,%.2lf,%.7lf,%.7lf\n", fnumber,timestamp,dname.c_str(),x,y,w,h,(int)poi->id,(converter.to_bytes(poi->name)).c_str(),poi->x,poi->y,poi->floor,relatives.lin,relatives.ang,poi_confidences,cam_lat,cam_lon);
                }
                // else
                // {
                //     printf("%d,%.3lf,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,0,0.00,0.00,0.00,0.00,0.00);
                //     fprintf(out_traj, "%d,%.3lf,%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", fnumber,timestamp,0,0.00,0.00,0.00,0.00,0.00);
                // }
            }
        }
        
        if (out_traj != nullptr) fclose(out_traj);

        return 0;
    }

    dg::Pose2 getPose(dg::Timestamp* timestamp = nullptr) const
    {
        return m_localizer->getPose(timestamp);
    }
}; // End of 'OCRTestset'


#endif // End of '__OCR_TESTSET__'
