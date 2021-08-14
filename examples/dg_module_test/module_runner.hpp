#ifndef __LOCALIZER_RUNNER__
#define __LOCALIZER_RUNNER__

#include "dg_localizer.hpp"
#include "dg_utils.hpp"
#include "localizer/data_loader.hpp"
#include "intersection_cls/intersection_localizer.hpp"
#include "vps/vps_localizer.hpp"
#include "ocr_recog/ocr_localizer.hpp"
#include "roadtheta/roadtheta_localizer.hpp"

enum { DG_VPS, DG_VPS_LR, DG_POI, DG_RoadTheta, DG_Intersection };

void onMouseEvent(int event, int x, int y, int flags, void* param);

class ModuleRunner : public dg::SharedInterface
{
    cv::Ptr<dg::DGLocalizer> m_localizer;
    cv::Ptr<dg::IntersectionLocalizer> m_intersection_localizer;
    cv::Ptr<dg::OCRLocalizer> m_ocr_localizer;
    cv::Ptr<dg::VPSLocalizer> m_vps_localizer;
    cv::Ptr<dg::RoadThetaLocalizer> m_roadtheta_localizer;
    //cv::Ptr<dg::LRLocalizer> m_lr_localizer;

public:
    int run(int module_sel, bool use_saved_testset, cv::Ptr<dg::DGLocalizer> localizer, dg::DataLoader& data_loader)
    {
        // initialize localizer
        CV_DbgAssert(!localizer.empty() && !data_loader.empty());
        m_localizer = localizer;
        m_localizer->setShared(this);

        // initialize module localizers
        if (module_sel == DG_VPS) m_vps_localizer = cv::makePtr<dg::VPSLocalizer>();
        if (module_sel == DG_POI) m_ocr_localizer = cv::makePtr<dg::OCRLocalizer>();
        if (module_sel == DG_Intersection) m_intersection_localizer = cv::makePtr<dg::IntersectionLocalizer>();
        if (module_sel == DG_RoadTheta) m_roadtheta_localizer = cv::makePtr<dg::RoadThetaLocalizer>();
        //if (module_sel == DG_VPS_LR) m_lr_localizer = cv::makePtr<dg::LRLocalizer>();
        if (use_saved_testset)
        {
            if (m_vps_localizer) m_vps_localizer->initialize_without_python(this);
            if (m_ocr_localizer) m_ocr_localizer->initialize_without_python(this);
            if (m_intersection_localizer) m_intersection_localizer->initialize_without_python(this);
            if (m_roadtheta_localizer) m_roadtheta_localizer->initialize(this);
            //if (m_lr_localizer) m_lr_localizer->initialize_without_python(this);
        }
        else
        {
            if (m_vps_localizer) m_vps_localizer->initialize(this);
            if (m_ocr_localizer) m_ocr_localizer->initialize(this);
            if (m_intersection_localizer) m_intersection_localizer->initialize(this);
            if (m_roadtheta_localizer) m_roadtheta_localizer->initialize(this);
            //if (m_lr_localizer) m_lr_localizer->initialize(this);
        }

        // Prepare the video for recording
        cx::VideoWriter out_video;
        if (!rec_video_name.empty())
        {
            if (!out_video.open(rec_video_name, rec_video_fps, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'))) return -1;
        }

        // Prepare visualization
        bool show_gui = gui_wnd_wait_msec >= 0 && gui_painter != nullptr && !gui_background.empty();
        cv::Mat bg_image = gui_background.clone();

        // Run localization with GPS and other sensors
        if (show_gui) cv::namedWindow("ModuleRunner::run()", cv::WindowFlags::WINDOW_AUTOSIZE);
        cv::setMouseCallback("ModuleRunner::run()", onMouseEvent, this);

        int type;
        std::vector<double> data;
        dg::Timestamp data_time;
        cv::Mat video_image;
        double timestart = data_loader.getStartTime();
        while (1)
        {
            bool update_gui = false;
            cv::Mat result_image;
            if (use_saved_testset)
            {
                if (data_loader.getNext(type, data, data_time) == false) break;

                if (type == dg::DATA_IMU)
                {
                    auto euler = cx::cvtQuat2EulerAng(data[1], data[2], data[3], data[4]);
                    bool success = localizer->applyIMUCompass(euler.z, data_time, 1);
                    if (!success) fprintf(stderr, "applyIMUCompass() was failed.\n");
                    update_gui = true;
                }
                else if (type == dg::DATA_GPS)
                {
                    dg::LatLon gps_datum(data[1], data[2]);
                    dg::Point2 gps_xy = toMetric(gps_datum);
                    bool success = localizer->applyGPS(gps_xy, data_time, 1);
                    if (!success) fprintf(stderr, "applyGPS() was failed.\n");
                    if (show_gui && gui_gps_radius > 0) gui_painter->drawPoint(bg_image, gps_xy, gui_gps_radius, gui_gps_color);
                }
                else if (module_sel == DG_Intersection && type == dg::DATA_IntersectCls)
                {
                    double cls = data[1];
                    double cls_conf = data[2];
                    dg::Point2 xy;
                    double xy_confidence;
                    bool xy_valid = false;
                    if (m_intersection_localizer->apply(data_time, cls, cls_conf, xy, xy_confidence, xy_valid) && xy_valid)
                    {
                        bool success = localizer->applyIntersectCls(xy, data_time, xy_confidence);
                        if (!success) fprintf(stderr, "applyIntersectCls() was failed.\n");
                    }
                    video_image = data_loader.getFrame(data_time);
                    result_image = video_image.clone();
                    m_intersection_localizer->draw(result_image);
                    update_gui = true;
                }
                else if (module_sel == DG_POI && type == dg::DATA_POI)
                {
                    //bool success = localizer->applyPOI(clue_xy, relative, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyPOI() was failed.\n");
                }
                else if (module_sel == DG_VPS && type == dg::DATA_VPS)
                {
                    //bool success = localizer->applyVPS(clue_xy, relative, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyVPS() was failed.\n");
                }
                else if (module_sel == DG_VPS_LR && type == dg::DATA_LR)
                {
                    //bool success = dg_localizer->applyVPS_LR(lr_result, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyVPS_LR() was failed.\n");
                }
                else if (module_sel == DG_RoadTheta && type == dg::DATA_RoadTheta)
                {
                    //bool success = localizer->applyRoadTheta(theta, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyRoadTheta() was failed.\n");
                }
            }
            else  // apply recognizer online
            {
                double capture_time;
                video_image = data_loader.getNextFrame(capture_time);
                if (video_image.empty()) break;
                update_gui = true;

                while (data_loader.getNextUntil(capture_time, type, data, data_time))
                {
                    if (type == dg::DATA_IMU)
                    {
                        auto euler = cx::cvtQuat2EulerAng(data[1], data[2], data[3], data[4]);
                        bool success = localizer->applyIMUCompass(euler.z, data_time, 1);
                        if (!success) fprintf(stderr, "applyIMUCompass() was failed.\n");
                        update_gui = true;
                    }
                    else if (type == dg::DATA_GPS)
                    {
                        dg::LatLon gps_datum(data[1], data[2]);
                        dg::Point2 gps_xy = toMetric(gps_datum);
                        bool success = localizer->applyGPS(gps_xy, data_time, 1);
                        if (!success) fprintf(stderr, "applyGPS() was failed.\n");
                        if (show_gui && gui_gps_radius > 0) gui_painter->drawPoint(bg_image, gps_xy, gui_gps_radius, gui_gps_color);
                    }
                }

                if (module_sel == DG_Intersection)
                {
                    dg::Point2 xy;
                    double xy_confidence;
                    bool xy_valid = false;
                    if (m_intersection_localizer->apply(video_image, capture_time, xy, xy_confidence, xy_valid) && xy_valid)
                    {
                        bool success = localizer->applyIntersectCls(xy, data_time, xy_confidence);
                        if (!success) fprintf(stderr, "applyIntersectCls() was failed.\n");
                    }
                    if (show_gui)
                    {
                        result_image = video_image.clone();
                        m_intersection_localizer->draw(result_image);
                    }
                }
                else if (module_sel == DG_POI)
                {
                    std::vector<dg::Point2> poi_xys;
                    std::vector<dg::Polar2> relatives;
                    std::vector<double> poi_confidences;
                    if (m_ocr_localizer->apply(video_image, capture_time, poi_xys, relatives, poi_confidences))
                    {
                        for (size_t i = 0; i < poi_xys.size(); i++)
                        {
                            bool success = localizer->applyPOI(poi_xys[i], relatives[i], data_time, poi_confidences[i]);
                            if (!success) fprintf(stderr, "applyPOI() was failed.\n");
                        }
                    }
                    if (show_gui)
                    {
                        result_image = video_image.clone();
                        m_ocr_localizer->draw(result_image);
                    }
                }
                else if (module_sel == DG_VPS)
                {
                    dg::Point2 streetview_xy;
                    dg::Polar2 relative;
                    double streetview_confidence;
                    dg::ID sv_id;
                    cv::Mat sv_image;
                    if (m_vps_localizer->apply(video_image, capture_time, streetview_xy, relative, streetview_confidence, sv_id, sv_image))
                    {
                        bool success = localizer->applyVPS(streetview_xy, relative, data_time, streetview_confidence);
                        if (!success) fprintf(stderr, "applyVPS() was failed.\n");
                    }
                    if (show_gui)
                    {
                        result_image = sv_image;
                        m_vps_localizer->draw(result_image);
                    }
                }
                else if (module_sel == DG_VPS_LR)
                {
                    //bool success = dg_localizer->applyVPS_LR(lr_result, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyVPS_LR() was failed.\n");
                }
                else if (module_sel == DG_RoadTheta)
                {
                    double theta;
                    double confidence;
                    if (m_roadtheta_localizer->apply(video_image, capture_time, theta, confidence))
                    {
                        bool success = localizer->applyRoadTheta(theta, data_time, confidence);
                        if (!success) fprintf(stderr, "applyRoadTheta() was failed.\n");
                    }
                    if (show_gui)
                    {
                        result_image = video_image.clone();
                        m_roadtheta_localizer->draw(result_image);
                    }
                }
            }

            // Visualize and show the current state as an image
            if (show_gui && update_gui)
            {
                dg::Pose2 pose = localizer->getPose();
                if (robot_traj_radius > 0) gui_painter->drawPoint(bg_image, pose, robot_traj_radius, gui_robot_color);

                cv::Mat out_image = bg_image.clone();
                if (gui_robot_radius > 0)
                {
                    gui_painter->drawPoint(out_image, pose, gui_robot_radius, gui_robot_color);                                         // Robot body
                    gui_painter->drawPoint(out_image, pose, gui_robot_radius, cv::Vec3b(255, 255, 255) - gui_robot_color, 1);           // Robot outline
                    cv::Point2d pose_px = gui_painter->cvtValue2Pixel(pose);
                    cv::Point2d head_px(gui_robot_radius * cos(pose.theta), -gui_robot_radius * sin(pose.theta));
                    cv::line(out_image, pose_px, pose_px + head_px, cv::Vec3b(255, 255, 255) - gui_robot_color, 2); // Robot heading
                }

                // Draw the image given from the camera
                cv::Rect video_rect;
                if (!video_image.empty() && video_resize > 0)
                {
                    cv::Mat resized;
                    cv::resize(video_image, resized, cv::Size(), video_resize, video_resize);
                    cx::Painter::pasteImage(out_image, resized, video_offset);
                    video_rect = cv::Rect(video_offset, resized.size());
                }

                // Draw the result image of recognizer module
                if (!result_image.empty() && result_resize > 0)
                {
                    cv::Mat resized;
                    cv::resize(result_image, resized, cv::Size(), result_resize, result_resize);
                    cv::Point offset = video_rect.br() + cv::Point(10, -resized.rows);
                    cx::Painter::pasteImage(out_image, resized, offset);
                    video_rect = cv::Rect(offset, resized.size());
                }

                // Record the current visualization on the AVI file
                if (out_video.isConfigured())
                {
                    out_video << out_image;
                }

                cv::imshow("LocalizerRunner::runLocalizer()", out_image);
                int key = cv::waitKey(gui_wnd_wait_msec);
                if (key == cx::KEY_SPACE) key = cv::waitKey(0);
                if (key == cx::KEY_ESC) break;
            }
        }
        if (gui_wnd_wait_exit) cv::waitKey(0);
        return 0;
    }

    void procMouseEvent(int evt, int x, int y, int flags)
    {
        if (evt == cv::EVENT_MOUSEMOVE)
        {
        }
        else if (evt == cv::EVENT_LBUTTONDOWN)
        {
        }
        else if (evt == cv::EVENT_LBUTTONUP)
        {
        }
        else if (evt == cv::EVENT_LBUTTONDBLCLK)
        {
            cv::Point2d p = gui_painter->cvtPixel2Value(cv::Point(x, y));
        }
        else if (evt == cv::EVENT_RBUTTONDOWN)
        {
            cv::Point2d p = gui_painter->cvtPixel2Value(cv::Point(x, y));
            printf("x = %lf, y = %lf\n", p.x, p.y);
        }
        else if (evt == cv::EVENT_RBUTTONUP)
        {
        }
        else if (evt == cv::EVENT_MOUSEWHEEL)
        {
        }
    }

    dg::Pose2 getPose(dg::Timestamp* timestamp = nullptr) const
    {
        return m_localizer->getPose(timestamp);
    }

    dg::LatLon getPoseGPS(dg::Timestamp* timestamp = nullptr) const
    {
        return m_localizer->getPoseGPS(timestamp);
    }

    dg::TopometricPose getPoseTopometric(dg::Timestamp* timestamp = nullptr) const
    {
        return m_localizer->getPoseTopometric(timestamp);
    }

    double getPoseConfidence(dg::Timestamp* timestamp = nullptr) const
    {
        return m_localizer->getPoseConfidence(timestamp);
    }

    cx::Painter* gui_painter = nullptr;
    cv::Mat      gui_background;
    int          gui_robot_radius = 10;
    cv::Vec3b    gui_robot_color = cv::Vec3b(0, 0, 255);
    int          robot_traj_radius = 1;
    cv::Vec3b    gui_gps_color = cv::Vec3b(100, 100, 100);
    int          gui_gps_radius = 2;

    int          gui_wnd_wait_msec = 1;
    bool         gui_wnd_wait_exit = false;

    double       video_resize = 1;
    cv::Point    video_offset = cv::Vec2d(1, -1);
    double       result_resize = 1;

    std::string  rec_video_name;
    double       rec_video_fps = 10;

}; // End of 'ModuleRunner'

void onMouseEvent(int event, int x, int y, int flags, void* param)
{
    ModuleRunner* localizer = (ModuleRunner*)param;
    localizer->procMouseEvent(event, x, y, flags);
}


#endif // End of '__LOCALIZER_RUNNER__'
