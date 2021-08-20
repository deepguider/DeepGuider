#ifndef __LOCALIZER_RUNNER__
#define __LOCALIZER_RUNNER__

#include "dg_localizer.hpp"
#include "dg_utils.hpp"
#include "localizer/data_loader.hpp"
#include "intersection_cls/intersection_localizer.hpp"
#include "vps/vps_localizer.hpp"
#include "lrpose_recog/lrpose_localizer.hpp"
#include "ocr_recog/ocr_localizer.hpp"
#include "roadtheta/roadtheta_localizer.hpp"
#include "utils/viewport.hpp"

enum { DG_VPS, DG_VPS_LR, DG_POI, DG_RoadTheta, DG_Intersection };

void onMouseEvent(int event, int x, int y, int flags, void* param);

class ModuleRunner : public dg::SharedInterface
{
    cv::Ptr<dg::DGLocalizer> m_localizer;
    cv::Ptr<dg::IntersectionLocalizer> m_intersection_localizer;
    cv::Ptr<dg::OCRLocalizer> m_ocr_localizer;
    cv::Ptr<dg::VPSLocalizer> m_vps_localizer;
    cv::Ptr<dg::RoadThetaLocalizer> m_roadtheta_localizer;
    cv::Ptr<dg::LRLocalizer> m_lr_localizer;

public:
    int run(int module_sel, bool use_saved_testset, cv::Ptr<dg::DGLocalizer> localizer, dg::DataLoader& data_loader)
    {
        // initialize localizer
        m_localizer = localizer;
        m_localizer->initialize(this, "EKFLocalizerHyperTan");

        // initialize module localizers
        if (module_sel == DG_VPS) m_vps_localizer = cv::makePtr<dg::VPSLocalizer>();
        if (module_sel == DG_POI) m_ocr_localizer = cv::makePtr<dg::OCRLocalizer>();
        if (module_sel == DG_Intersection) m_intersection_localizer = cv::makePtr<dg::IntersectionLocalizer>();
        if (module_sel == DG_RoadTheta) m_roadtheta_localizer = cv::makePtr<dg::RoadThetaLocalizer>();
        if (module_sel == DG_VPS_LR) m_lr_localizer = cv::makePtr<dg::LRLocalizer>();
        if (use_saved_testset)
        {
            if (m_vps_localizer) VVS_CHECK_TRUE(m_vps_localizer->initialize_without_python(this));
            if (m_ocr_localizer) VVS_CHECK_TRUE(m_ocr_localizer->initialize_without_python(this));
            if (m_intersection_localizer) VVS_CHECK_TRUE(m_intersection_localizer->initialize_without_python(this));
            if (m_roadtheta_localizer) VVS_CHECK_TRUE(m_roadtheta_localizer->initialize(this));
            if (m_lr_localizer) VVS_CHECK_TRUE(m_lr_localizer->initialize_without_python(this));
        }
        else
        {
            // initialize python environment
            dg::init_python_environment("python3", "", false);

            if (m_vps_localizer) VVS_CHECK_TRUE(m_vps_localizer->initialize(this));
            if (m_ocr_localizer) VVS_CHECK_TRUE(m_ocr_localizer->initialize(this));
            if (m_intersection_localizer) VVS_CHECK_TRUE(m_intersection_localizer->initialize(this));
            if (m_roadtheta_localizer) VVS_CHECK_TRUE(m_roadtheta_localizer->initialize(this));
            if (m_lr_localizer) VVS_CHECK_TRUE(m_lr_localizer->initialize(this));
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
        m_viewport.initialize(bg_image, m_view_size, m_view_offset);

        // Run localization with GPS and other sensors
        if (show_gui) cv::namedWindow("ModuleRunner::run()", cv::WindowFlags::WINDOW_NORMAL);
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
                    //if (m_ocr_localizer->apply(...))
                    //{
                        //bool success = localizer->applyPOI(clue_xy, relative, data_time, confidence);
                        //if (!success) fprintf(stderr, "applyPOI() was failed.\n");
                    //}
                    video_image = data_loader.getFrame(data_time);
                    result_image = video_image.clone();
                    m_ocr_localizer->draw(result_image);
                    update_gui = true;
                }
                else if (module_sel == DG_VPS && type == dg::DATA_VPS)
                {
                    /*** Input of m_vps_localizer->apply() ***/
                    double t = data[0];
                    double svid = data[1];
                    double svidx = data[2];  // Which side db image was matched to query. 'f' means front, 'b', 'u', 'l', 'r'.
                    double pred_lat = data[3];
                    double pred_lon = data[4];                    
                    double pred_distance = data[5];
                    double pred_angle = data[6];
                    double pred_confidence = data[7];
                    dg::LatLon pred_ll;
                    /*** Output of m_vps_localizer->apply() ***/
                    dg::Point2 streetview_xy;
                    dg::Polar2 relative;
                    double streetview_confidence;
                    dg::ID sv_id;
                    cv::Mat sv_image;
                    dg::VPSResult result;
                    std::vector<dg::VPSResult> results;
   
                    pred_ll.lat = pred_lat;
                    pred_ll.lon = pred_lon;
                    video_image = data_loader.getFrame(data_time);

                    if (m_vps_localizer->apply(video_image, data_time, svid, pred_ll, pred_distance, pred_angle, pred_confidence, streetview_xy, relative, streetview_confidence, sv_id, sv_image))
                    {
                        bool success = localizer->applyVPS(streetview_xy, relative, data_time, streetview_confidence);
                        if (!success) fprintf(stderr, "applyVPS() was failed.\n");
                    }

                    result_image = sv_image;
                    result.confidence = streetview_confidence;
                    result.id = sv_id;
                    results.push_back(result);
                    m_vps_localizer->set((const std::vector<dg::VPSResult>&)results, data_time, -1);
                    m_vps_localizer->draw(result_image);
                    update_gui = true;                      
                }
                else if (module_sel == DG_VPS_LR && type == dg::DATA_LR)
                {
                    //bool success = dg_localizer->applyVPS_LR(lr_result, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyVPS_LR() was failed.\n");
                    double cls = data[1];
                    double cls_conf = data[2];
                    double lr_confidence;
                    double lr_valid = 1;  // UNKNOWN_SIDE_OF_ROAD
                    if (m_lr_localizer->apply(data_time, cls, cls_conf, lr_confidence, lr_valid))
                    {
                        bool success = localizer->applyVPS_LR(lr_valid, data_time, lr_confidence);
                        if (!success) fprintf(stderr, "applyVPS_LR() was failed.\n");
                    }
                    video_image = data_loader.getFrame(data_time);
                    result_image = video_image.clone();
                    m_lr_localizer->draw(result_image);
                    update_gui = true;                    
                }
                else if (module_sel == DG_RoadTheta && type == dg::DATA_RoadTheta)
                {
                    //bool success = localizer->applyRoadTheta(theta, data_time, confidence);
                    //if (!success) fprintf(stderr, "applyRoadTheta() was failed.\n");
                }
            }
            else  // run modules online
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

                // get viewport image
                double view_zoom;
                cv::Point2d view_offset;
                cv::Mat out_image = m_viewport.getViewportImage(view_offset, view_zoom);

                // convert to viewport point
                dg::Point2 pose_pixel = (gui_painter->cvtValue2Pixel(pose) - view_offset) * view_zoom;
                dg::Point2 pose_viewport = gui_painter->cvtPixel2Value(pose_pixel);
                int gui_robot_radius_viewport = (int)(gui_robot_radius * view_zoom + 0.5);

                // Draw path
                dg::Path* path = getPathLocked();
                if (path && !path->empty())
                {
                    // conversion
                    std::vector<dg::Point2> pts;
                    pts.resize(path->pts.size());
                    for(size_t idx=0;idx<path->pts.size(); idx++) pts[idx] = gui_painter->cvtPixel2Value((gui_painter->cvtValue2Pixel(path->pts[idx]) - view_offset) * view_zoom);

                    // draw current path
                    dg::MapPainter* painter = (dg::MapPainter*)gui_painter;
                    const cv::Vec3b ecolor = cv::Vec3b(255, 0, 0);
                    const cv::Vec3b ncolor = cv::Vec3b(0, 255, 255);
                    int nradius = 3;
                    int ethickness = 1;
                    for (int idx = 1; idx < (int)pts.size(); idx++)
                    {
                        painter->drawEdge(out_image, pts[idx - 1], pts[idx], nradius, ecolor, ethickness);
                    }
                    for (int idx = 1; idx < (int)pts.size() - 1; idx++)
                    {
                        painter->drawNode(out_image, dg::Point2ID(0, pts[idx]), nradius, 0, ncolor);
                    }
                }
                releasePathLock();

                // draw robot position
                if (gui_robot_radius > 0)
                {
                    gui_painter->drawPoint(out_image, pose_viewport, gui_robot_radius_viewport, gui_robot_color);                                         // Robot body
                    gui_painter->drawPoint(out_image, pose_viewport, gui_robot_radius_viewport, cv::Vec3b(255, 255, 255) - gui_robot_color, 1);           // Robot outline
                    cv::Point2d pose_px = gui_painter->cvtValue2Pixel(pose_viewport);
                    cv::Point2d head_px(gui_robot_radius_viewport * cos(pose.theta), -gui_robot_radius_viewport * sin(pose.theta));
                    cv::line(out_image, pose_px, pose_px + head_px, cv::Vec3b(255, 255, 255) - gui_robot_color, (int)(2 * view_zoom)); // Robot heading
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

                cv::imshow("ModuleRunner::run()", out_image);
                int key = cv::waitKey(gui_wnd_wait_msec);
                if (key == cx::KEY_SPACE) key = cv::waitKey(0);
                if (key == cx::KEY_ESC) break;
            }
        }
        if (gui_wnd_wait_exit) cv::waitKey(0);

        if (!use_saved_testset)
        {
            // Close the Python Interpreter
            dg::close_python_environment();
        }

        return 0;
    }

    void procMouseEvent(int evt, int x, int y, int flags)
    {
        m_viewport.procMouseEvent(evt, x, y, flags);

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
            dg::Pose2 p_start = getPose();
            cv::Point2d p_pixel = m_viewport.cvtView2World(cv::Point(x, y));
            cv::Point2d p_dest = gui_painter->cvtPixel2Value(p_pixel);
            dg::Path path;
            setMapLock();
            bool ok = m_map && m_map->getPath(p_start, p_dest, path);
            releaseMapLock();
            if (ok)
            {
                setPath(path);
                m_dest_xy = p_dest;
                m_dest_defined = true;
            }
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

    bool procOutOfPath(const dg::Point2& curr_pose)
    {
        if (!m_dest_defined) return false;
        dg::Path path;
        bool ok = m_map && m_map->getPath(curr_pose, m_dest_xy, path);
        if (!ok) return false;
        return setPath(path);
    }

    bool         m_dest_defined = false;
    dg::Point2   m_dest_xy;

    cx::Painter* gui_painter = nullptr;
    cv::Mat      gui_background;
    int          gui_robot_radius = 10;
    cv::Vec3b    gui_robot_color = cv::Vec3b(0, 0, 255);
    int          robot_traj_radius = 1;
    cv::Vec3b    gui_gps_color = cv::Vec3b(100, 100, 100);
    int          gui_gps_radius = 2;

    cv::Point   m_view_offset = cv::Point(0, 0);
    cv::Size    m_view_size = cv::Size(1920, 1080);
    dg::Viewport m_viewport;

    int          gui_wnd_wait_msec = 1;
    bool         gui_wnd_wait_exit = false;

    double       video_resize = 1;
    cv::Point    video_offset = cv::Point(0, 0);
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
