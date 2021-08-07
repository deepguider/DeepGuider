#ifndef __LOCALIZER_RUNNER__
#define __LOCALIZER_RUNNER__

#include "dg_localizer.hpp"

struct MapGUIProp
{
public:
    std::string image_file;
    cv::Point2d image_scale;
    double      image_rotation = 0; // radian
    dg::LatLon  origin_latlon;      // origin of UTM
    cv::Point2d origin_px;          // pixel coordinte of UTM origin at map image
    double      map_radius;         // topomap coverage from the origin (unit: meter)
    cv::Point   grid_unit_pos;
    std::string map_file;
    int         wnd_flag = cv::WindowFlags::WINDOW_AUTOSIZE;
    double      video_resize = 0;
    cv::Point   video_offset;
    double      zoom_level = 0;
    double      zoom_radius = 0;
    cv::Point   zoom_offset;
};

void onMouseEventLocalizer(int event, int x, int y, int flags, void* param);

class LocalizerRunner : public dg::SharedInterface
{
public:
    cx::Painter* gui_painter;
    cv::Mat      gui_background;
    int          gui_robot_radius;
    cv::Vec3b    gui_robot_color;
    int          gui_robot_thickness;
    int          gui_traj_radius;
    cv::Vec3b    gui_gps_color;
    int          gui_gps_radius;
    double       gui_covar_scale;
    cv::Vec3b    gui_covar_color;
    int          gui_covar_thickness;
    double       gui_text_scale;
    cv::Point    gui_text_offset;
    cv::Vec3b    gui_text_color;
    int          gui_text_thickness;
    double       gui_time_offset;
    int          gui_topo_ref_radius;
    cv::Vec3b    gui_topo_ref_color;
    int          gui_topo_ref_thickness;
    int          gui_topo_loc_radius;
    cv::Vec3b    gui_topo_loc_color;
    int          gui_topo_loc_thickness;
    std::string  gui_clue_text;
    cv::Vec3b    gui_clue_color;
    int          gui_clue_thickness;
    int          gui_wnd_flag;
    int          gui_wnd_wait_msec;
    bool         gui_wnd_wait_exit;

    double       video_resize;
    cv::Point    video_offset;
    cv::Vec2d    video_time;

    cx::Painter* zoom_painter;
    cv::Mat      zoom_background;
    double       zoom_radius;
    cv::Point    zoom_offset;
    cv::Vec3b    zoom_box_color;
    int          zoom_box_thickness;
    bool         zoom_user_drag = false;
    cv::Point    zoom_user_point;

    std::string  rec_traj_name;
    std::string  rec_video_name;
    double       rec_video_fps = 10;
    int          rec_video_fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    double       rec_video_quality = 100;
    double       rec_video_resize;

    bool         m_dest_defined = false;
    dg::Point2   m_dest_xy;

    cv::Ptr<dg::BaseLocalizer> m_localizer;

    LocalizerRunner()
    {
        gui_painter = nullptr;
        gui_robot_radius = 10;
        gui_robot_color = cv::Vec3b(0, 0, 255);
        gui_robot_thickness = 2;
        gui_traj_radius = 2;
        gui_gps_color = cv::Vec3b(100, 100, 100);
        gui_gps_radius = 2;
        gui_covar_scale = 30;
        gui_covar_color = cv::Vec3b(0, 255, 0);
        gui_covar_thickness = 1;
        gui_text_scale = 1;
        gui_text_offset = cv::Point(5, 15);
        gui_text_color = cx::COLOR_MAGENTA;
        gui_text_thickness = 1;
        gui_time_offset = 0;
        gui_topo_ref_radius = 0;
        gui_topo_ref_color = cv::Vec3b(255, 127, 255);
        gui_topo_ref_thickness = -1;
        gui_topo_loc_radius = 0;
        gui_topo_loc_color = cv::Vec3b(127, 255, 127);
        gui_topo_loc_thickness = 2;
        gui_clue_text = "Intersection (%.2f)";
        gui_clue_color = cv::Vec3b(255, 0, 0);
        gui_clue_thickness = 2;
        gui_wnd_flag = cv::WindowFlags::WINDOW_AUTOSIZE;
        gui_wnd_wait_msec = 1;
        gui_wnd_wait_exit = false;

        video_resize = 1;
        video_time = cv::Vec2d(1, -1);

        zoom_painter = nullptr;
        zoom_radius = 0;
        zoom_offset = cv::Point(5, 20);
        zoom_box_thickness = 1;

        rec_video_resize = 1;
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

    int runLocalizer(cv::Ptr<dg::BaseLocalizer> localizer, const cx::CSVReader::Double2D& gps_data)
    {
        CV_DbgAssert(!localizer.empty() && !gps_data.empty());
        m_localizer = localizer;
        m_localizer->setShared(this);

        // Prepare the result trajectory and video
        FILE* out_traj = nullptr;
        if (!rec_traj_name.empty())
        {
            out_traj = fopen(rec_traj_name.c_str(), "wt");
            if (out_traj == nullptr) return -1;
            fprintf(out_traj, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");
        }
        cx::VideoWriter out_video;
        if (!rec_video_name.empty())
        {
            if (rec_video_quality > 0) out_video.set(cv::VIDEOWRITER_PROP_QUALITY, rec_video_quality);
            if (!out_video.open(rec_video_name, rec_video_fps, rec_video_fourcc)) return -1;
            if (rec_video_quality > 0) out_video.set(cv::VIDEOWRITER_PROP_QUALITY, rec_video_quality);
        }

        // Prepare visualization
        bool show_gui = gui_wnd_wait_msec >= 0 && gui_painter != nullptr && !gui_background.empty();
        cv::Mat bg_image = gui_background.clone();
        bool show_zoom = show_gui && zoom_painter != nullptr && !zoom_background.empty();
        cv::Mat zoom_bg_image = zoom_background.clone();

        // Run GPS-only localization
        if (show_gui) cv::namedWindow("LocalizerRunner::runLocalizer()", gui_wnd_flag);
        cv::setMouseCallback("LocalizerRunner::runLocalizer()", onMouseEventLocalizer, this);

        double timestart = gps_data[0][0];
        for (size_t i = 0; i < gps_data.size(); i++)
        {
            // Apply GPS data as position observation
            double timestamp = gps_data[i][0];
            dg::Point2 gps_xy(gps_data[i][1], gps_data[i][2]);
            bool success = localizer->applyGPS(gps_xy, timestamp);
            if (!success) fprintf(stderr, "applyPosition() was failed.\n");

            // Record the current state on the CSV file
            if (out_traj != nullptr)
            {
                dg::Pose2 pose = localizer->getPose();
                fprintf(out_traj, "%f, %f, %f, %f\n", timestamp, pose.x, pose.y, pose.theta);
            }

            // Visualize and show the current state as an image
            if (show_gui)
            {
                dg::Pose2 pose = localizer->getPose();
                if (gui_traj_radius > 0) gui_painter->drawPoint(bg_image, pose, gui_traj_radius, gui_robot_color); // Robot trajectory
                if (gui_gps_radius  > 0) gui_painter->drawPoint(bg_image, gps_xy, gui_gps_radius, gui_gps_color);     // GPS point
                cv::Mat out_image = genStateImage(bg_image, localizer, timestamp - timestart);

                if (show_zoom)
                {
                    if (gui_traj_radius > 0) zoom_painter->drawPoint(zoom_bg_image, pose, gui_traj_radius, gui_robot_color); // Robot trajectory
                    if (gui_gps_radius  > 0) zoom_painter->drawPoint(zoom_bg_image, gps_xy, gui_gps_radius, gui_gps_color);     // GPS point
                    cv::Mat zoom_image = genStateImage(zoom_bg_image, localizer, timestamp - timestart, zoom_painter);
                    pasteZoomedImage(out_image, pose, zoom_image);
                }

                // Record the current visualization on the AVI file
                if (out_video.isConfigured())
                {
                    cv::Mat resize;
                    cv::resize(out_image, resize, cv::Size(), rec_video_resize, rec_video_resize);
                    out_video << out_image;
                }

                cv::imshow("LocalizerRunner::runLocalizer()", out_image);
                int key = cv::waitKey(gui_wnd_wait_msec);
                if (key == cx::KEY_SPACE) key = cv::waitKey(0);
                if (key == cx::KEY_ESC) break;
            }
        }
        if (out_traj != nullptr) fclose(out_traj);
        if (gui_wnd_wait_exit) cv::waitKey(0);
        return 0;
    }

    int runLocalizer(cv::Ptr<dg::BaseLocalizer> localizer, const cx::CSVReader::Double2D& gps_data, const cx::CSVReader::Double2D& ahrs_data, const cx::CSVReader::Double2D& clue_data, cv::VideoCapture& camera_data)
    {
        CV_DbgAssert(!localizer.empty() && !gps_data.empty());
        m_localizer = localizer;
        m_localizer->setShared(this);
        cv::Ptr<dg::PathLocalizer> path_localizer = localizer.dynamicCast<dg::PathLocalizer>();

        // Prepare the result trajectory and video
        FILE* out_traj = nullptr;
        if (!rec_traj_name.empty())
        {
            out_traj = fopen(rec_traj_name.c_str(), "wt");
            if (out_traj == nullptr) return -1;
            fprintf(out_traj, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");
        }
        cx::VideoWriter out_video;
        if (!rec_video_name.empty())
        {
            if (rec_video_quality > 0) out_video.set(cv::VIDEOWRITER_PROP_QUALITY, rec_video_quality);
            if (!out_video.open(rec_video_name, rec_video_fps, rec_video_fourcc)) return -1;
            if (rec_video_quality > 0) out_video.set(cv::VIDEOWRITER_PROP_QUALITY, rec_video_quality);
        }

        // Prepare visualization
        bool show_gui = gui_wnd_wait_msec >= 0 && gui_painter != nullptr && !gui_background.empty();
        cv::Mat bg_image = gui_background.clone();
        bool show_zoom = show_gui && zoom_painter != nullptr && !zoom_background.empty();
        cv::Mat zoom_bg_image = zoom_background.clone();

        // Run localization with GPS and other sensors
        if (show_gui) cv::namedWindow("LocalizerRunner::runLocalizer()", gui_wnd_flag);
        cv::setMouseCallback("LocalizerRunner::runLocalizer()", onMouseEventLocalizer, this);
        double timestart = gps_data[0][0];
        if (video_time[1] < 0) video_time[1] = timestart;
        size_t gps_idx = 0, ahrs_idx = 0, clue_idx = 0;
        double clue_confidence = -1;
        cv::Mat cam_image;

        //while (gps_idx < gps_data.size() || ahrs_idx < ahrs_data.size() || clue_idx < clue_data.size())
        while (gps_idx < gps_data.size())
        {
            double gps_time = DBL_MAX, ahrs_time = DBL_MAX, clue_time = DBL_MAX;
            if (gps_idx < gps_data.size()) gps_time = gps_data[gps_idx][0];
            if (ahrs_idx < ahrs_data.size()) ahrs_time = ahrs_data[ahrs_idx][0];
            if (clue_idx < clue_data.size()) clue_time = clue_data[clue_idx][0];

            if (camera_data.isOpened())
            {
                while (true)
                {
                    double camera_time = video_time[0] * camera_data.get(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC) / 1000 + video_time[1];
                    if (camera_time >= gps_time) break;
                    camera_data >> cam_image;
                    if (cam_image.empty()) break;
                }
            }

            bool update_gui = false;
            if (gps_idx < gps_data.size() && gps_time <= ahrs_time && gps_time <= clue_time)
            {
                // Apply GPS data as position observation
                dg::Point2 gps_xy(gps_data[gps_idx][1], gps_data[gps_idx][2]);
                bool success = localizer->applyGPS(gps_xy, gps_time);
                if (!success) fprintf(stderr, "applyPosition() was failed.\n");
                gps_idx++;

                if (gui_gps_radius > 0)
                {
                    if (show_gui) gui_painter->drawPoint(bg_image, gps_xy, gui_gps_radius, gui_gps_color);
                    if (show_zoom) zoom_painter->drawPoint(zoom_bg_image, gps_xy, gui_gps_radius, gui_gps_color);
                }
                update_gui = true;
            }
            else if (ahrs_idx < ahrs_data.size() && ahrs_time <= clue_time)
            {
                // Apply AHRS data as orientation observation
                auto euler = cx::cvtQuat2EulerAng(ahrs_data[ahrs_idx][1], ahrs_data[ahrs_idx][2], ahrs_data[ahrs_idx][3], ahrs_data[ahrs_idx][4]);
                bool success = localizer->applyIMUCompass(euler.z, ahrs_time);
                if (!success) fprintf(stderr, "applyIMUCompass() was failed.\n");
                ahrs_idx++;
            }
            else if (clue_idx < clue_data.size())
            {
                // Apply location clues
                if (clue_data[clue_idx][1] > 0)
                {
                    clue_confidence = clue_data[clue_idx][2];
                    //bool success = localizer->applyLocClue(0, dg::Polar2(-1, CV_PI), clue_time, clue_confidence);
                    //if (!success) fprintf(stderr, "applyLocClue() was failed.\n");
                }
                else clue_confidence = -1;
                clue_idx++;
            }

            // Record the current state on the CSV file
            if (out_traj != nullptr)
            {
                dg::Pose2 pose = localizer->getPose();
                fprintf(out_traj, "%f, %f, %f, %f\n", gps_time, pose.x, pose.y, pose.theta);
            }

            // Visualize and show the current state as an image
            if (show_gui && update_gui)
            {
                dg::Pose2 pose = localizer->getPose();
                if (gui_traj_radius > 0) gui_painter->drawPoint(bg_image, pose, gui_traj_radius, gui_robot_color);
                cv::Mat out_image = genStateImage(bg_image, localizer, gps_time - timestart);

                // Draw the current state on the bigger background
                if (show_zoom)
                {
                    if (zoom_user_drag)
                    {
                        cv::Point2d pos = gui_painter->cvtPixel2Value(zoom_user_point);
                        pasteZoomedImage(out_image, pos, zoom_background);
                    }
                    else
                    {
                        if (gui_traj_radius > 0) zoom_painter->drawPoint(zoom_bg_image, pose, gui_traj_radius, gui_robot_color);
                        cv::Mat zoom_image = genStateImage(zoom_bg_image, localizer, gps_time - timestart, zoom_painter);
                        pasteZoomedImage(out_image, pose, zoom_image);
                    }
                }

                // Draw the image given from the camera
                if (!cam_image.empty() && video_resize > 0)
                {
                    cv::Mat resize;
                    cv::resize(cam_image, resize, cv::Size(), video_resize, video_resize);
                    cx::Painter::pasteImage(out_image, resize, video_offset);
                    if (clue_confidence > 0)
                    {
                        cv::putText(out_image, cv::format(gui_clue_text.c_str(), clue_confidence), video_offset + gui_text_offset, cv::FONT_HERSHEY_PLAIN, gui_text_scale, gui_clue_color, gui_text_thickness);
                        cv::rectangle(out_image, cv::Rect(video_offset, resize.size()), gui_clue_color, gui_clue_thickness);
                    }
                }

                // Draw path
                cv::Mat path_image = out_image;
                setPathLock();
                dg::Path* path = getPath();
                if (path && !path->empty())
                {
                    path_image = out_image.clone();

                    // draw current path
                    dg::MapPainter* painter = (dg::MapPainter*)gui_painter;
                    const cv::Vec3b ecolor = cv::Vec3b(255, 0, 0);
                    const cv::Vec3b ncolor = cv::Vec3b(0, 255, 255);
                    int nradius = 3;
                    int ethickness = 1;
                    for (int idx = 1; idx < (int)path->pts.size(); idx++)
                    {
                        painter->drawEdge(path_image, path->pts[idx - 1], path->pts[idx], nradius, ecolor, ethickness);
                    }
                    for (int idx = 1; idx < (int)path->pts.size() - 1; idx++)
                    {
                        painter->drawNode(path_image, dg::Point2ID(0, path->pts[idx]), nradius, 0, ncolor);
                    }
                }
                releasePathLock();

                // Record the current visualization on the AVI file
                if (out_video.isConfigured() && rec_video_resize > 0)
                {
                    out_video << path_image;
                }

                cv::imshow("LocalizerRunner::runLocalizer()", path_image);
                int key = cv::waitKey(gui_wnd_wait_msec);
                if (key == cx::KEY_SPACE) key = cv::waitKey(0);
                if (key == cx::KEY_ESC) break;
            }
        }
        if (out_traj != nullptr) fclose(out_traj);
        if (gui_wnd_wait_exit) cv::waitKey(0);
        return 0;
    }

    void procMouseEvent(int evt, int x, int y, int flags)
    {
        if (evt == cv::EVENT_MOUSEMOVE)
        {
            if (zoom_user_drag)
            {
                zoom_user_point = cv::Point(x, y);
            }
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
            cv::Point2d p_dest = gui_painter->cvtPixel2Value(cv::Point(x, y));
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

            zoom_user_point = cv::Point(x, y);
            zoom_user_drag = true;
        }
        else if (evt == cv::EVENT_RBUTTONUP)
        {
            zoom_user_drag = false;
        }
        else if (evt == cv::EVENT_MOUSEWHEEL)
        {
        }
    }

    cv::Mat genStateImage(const cv::Mat& bg_image, const cv::Ptr<dg::BaseLocalizer> localizer, double timestamp, cx::Painter* painter = nullptr)
    {
        if (painter == nullptr) painter = gui_painter;
        CV_DbgAssert(!localizer.empty() && painter != nullptr);
        cv::Mat image = bg_image.clone();
        if (image.empty()) return cv::Mat();

        // Get pose
        dg::Pose2 pose_m = localizer->getPose();
        cv::Ptr<dg::EKFLocalizerSinTrack> localizer_topo = localizer.dynamicCast<dg::EKFLocalizerSinTrack>();
        dg::TopometricPose pose_t;
        if (!localizer_topo.empty()) pose_t = localizer_topo->getPoseTopometric();

        // Visualize pose
        if (!localizer_topo.empty() && pose_t.node_id > 0)
        {
            dg::Map* map = localizer_topo->getMap();
            if (map != nullptr)
            {
                if (gui_topo_loc_radius > 0)
                {
                    std::vector<dg::Node*> search_nodes = localizer_topo->getSearchNodes();
                    for (auto n = search_nodes.begin(); n != search_nodes.end(); n++)
                    {
                        if (*n != nullptr)
                        {
                            for (auto e = (*n)->edge_ids.begin(); e != (*n)->edge_ids.end(); e++)
                            {
                                dg::Node* to = map->getConnectedNode(*n, *e);
                                if (to != nullptr)
                                    painter->drawLine(image, *(*n), *to, gui_topo_loc_color, gui_topo_loc_thickness);
                            }
                            painter->drawPoint(image, *(*n), gui_topo_loc_radius, gui_topo_loc_color, gui_topo_loc_thickness);
                        }
                    }
                }

                if (gui_topo_ref_radius > 0)
                {
                    dg::Node* ref_node = map->getNode(pose_t.node_id);
                    if (ref_node != nullptr) painter->drawPoint(image, *ref_node, gui_topo_ref_radius, gui_topo_ref_color, gui_topo_ref_thickness);
                }
            }
        }
        if (gui_robot_radius > 0)
        {
            painter->drawPoint(image, pose_m, gui_robot_radius, gui_robot_color);                                         // Robot body
            painter->drawPoint(image, pose_m, gui_robot_radius, cv::Vec3b(255, 255, 255) - gui_robot_color, 1);           // Robot outline
            cv::Point2d pose_px = painter->cvtValue2Pixel(pose_m);
            cv::Point2d head_px(gui_robot_radius * cos(pose_m.theta), -gui_robot_radius * sin(pose_m.theta));
            cv::line(image, pose_px, pose_px + head_px, cv::Vec3b(255, 255, 255) - gui_robot_color, gui_robot_thickness); // Robot heading
        }
        if (gui_covar_scale > 0 && gui_covar_thickness > 0)
        {
            cv::Ptr<cx::EKF> ekf = localizer.dynamicCast<cx::EKF>();
            cv::Ptr<dg::PathLocalizer> path_localizer = localizer.dynamicCast<dg::PathLocalizer>();
            if (!ekf.empty() || !path_localizer.empty())
            {
                cv::Mat eval, evec;
                cv::Mat covar = (ekf) ? ekf->getStateCov() : path_localizer->getStateCov();
                cv::eigen(covar(cv::Rect(0, 0, 2, 2)), eval, evec);
                double pixel_per_value = painter->getPixel2Value().x;
                cv::RotatedRect covar_box(painter->cvtValue2Pixel(pose_m),
                    cv::Size2d(pixel_per_value * gui_covar_scale * sqrt(eval.at<double>(0)), pixel_per_value * gui_covar_scale * sqrt(eval.at<double>(1))),
                    static_cast<float>(cx::cvtRad2Deg(atan2(-evec.at<double>(1, 0), evec.at<double>(0, 0)))));
                cv::ellipse(image, covar_box, gui_covar_color, gui_covar_thickness);                                          // EKF covariance
            }
        }
        if (gui_text_scale > 0)
        {
            //dg::Polar2 velocity = localizer->getVelocity();
            dg::Polar2 velocity = dg::Polar2();
            std::string metric_text = cv::format("Time: %.2f / Pose: %.2f, %.2f, %.0f / Velocity: %.2f, %.0f",
                timestamp + gui_time_offset, pose_m.x, pose_m.y, cx::cvtRad2Deg(pose_m.theta), velocity.lin, cx::cvtRad2Deg(velocity.ang));
            cv::putText(image, metric_text, gui_text_offset, cv::FONT_HERSHEY_PLAIN, gui_text_scale, gui_text_color, gui_text_thickness);
            if (!localizer_topo.empty() && pose_t.node_id > 0)
            {
                std::string topo_text = cv::format("Node ID: %zd, Edge Idx: %d, Dist: %.2f, Head: %.0f", pose_t.node_id, pose_t.edge_idx, pose_t.dist, cx::cvtRad2Deg(pose_t.head));
                int topo_text_offset = static_cast<int>(20 * gui_text_scale);
                cv::putText(image, topo_text, gui_text_offset + cv::Point(0, topo_text_offset), cv::FONT_HERSHEY_PLAIN, gui_text_scale, gui_text_color, gui_text_thickness);
            }
        }
        return image;
    }

    bool pasteZoomedImage(cv::Mat& image, const dg::Pose2& pose, const cv::Mat& zoom_image)
    {
        if (image.empty() || zoom_image.empty()) return false;

        cv::Point tl = zoom_painter->cvtValue2Pixel(pose + dg::Pose2(-zoom_radius, -zoom_radius, 0));
        cv::Point br = zoom_painter->cvtValue2Pixel(pose + dg::Pose2(+zoom_radius, +zoom_radius, 0));
        cv::Rect zoom_crop = cv::Rect(tl, br) & cv::Rect(cv::Point(), zoom_image.size());
        cx::Painter::pasteImage(image, zoom_image(zoom_crop), zoom_offset);
        cv::rectangle(image, cv::Rect(zoom_offset.x, zoom_offset.y, zoom_crop.width, zoom_crop.height), zoom_box_color, zoom_box_thickness);
        return true;
    }

    static bool drawGPSData(cv::Mat& image, const cx::Painter* painter, const std::vector<std::vector<double>>& gps_data, const cv::Vec3b& color = cv::Vec3b(0, 0, 255), int radius = 2)
    {
        if (image.empty() || painter == nullptr || radius <= 0) return false;

        for (auto gps = gps_data.begin(); gps != gps_data.end(); gps++)
            painter->drawPoint(image, dg::Point2(gps->at(1), gps->at(2)), radius, color);
        return true;
    }

    static std::vector<std::vector<double>> readNoisyPosition(const std::string& path_file, double gps_noise = 0, const dg::Polar2& gps_offset = dg::Polar2(0, 0))
    {
        std::vector<std::vector<double>> data;

        // Load the true trajectory
        cx::CSVReader gps_reader;
        if (!gps_reader.open(path_file)) return data;
        cx::CSVReader::Double2D gps_truth = gps_reader.extDouble2D(1, { 0, 1, 2, 3 });
        if (gps_truth.empty()) return data;

        // Generate noisy GPS data
        for (size_t i = 0; i < gps_truth.size(); i++)
        {
            if (gps_truth[i].size() < 4) return data;
            double t = gps_truth[i][0];
            double x = gps_truth[i][1] + gps_offset.lin * cos(gps_truth[i][3] + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
            double y = gps_truth[i][2] + gps_offset.lin * sin(gps_truth[i][3] + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
            std::vector<double> datum = { t, x, y };
            data.push_back(datum);
        }
        return data;
    }

    static cv::Ptr<dg::BaseLocalizer> getLocalizer(const std::string& name)
    {
        cv::Ptr<dg::BaseLocalizer> localizer;
        if (name == "EKFLocalizer") localizer = cv::makePtr<dg::EKFLocalizer>();
        else if (name == "EKFLocalizerZeroGyro") localizer = cv::makePtr<dg::EKFLocalizerZeroGyro>();
        else if (name == "EKFLocalizerHyperTan") localizer = cv::makePtr<dg::EKFLocalizerHyperTan>();
        else if (name == "EKFLocalizerSinTrack") localizer = cv::makePtr<dg::EKFLocalizerSinTrack>();
        else if (name == "EKFLocalizerInterSec") localizer = cv::makePtr<dg::EKFLocalizerInterSec>();
        else if (name == "PathLocalizer") localizer = cv::makePtr<dg::PathLocalizer>();
        return localizer;
    }

}; // End of 'LocalizerRunner'

void onMouseEventLocalizer(int event, int x, int y, int flags, void* param)
{
    LocalizerRunner* localizer = (LocalizerRunner*)param;
    localizer->procMouseEvent(event, x, y, flags);
}

cx::CSVReader::Double2D readROSGPSFix(const std::string& gps_file, const dg::LatLon& ref_pts = dg::LatLon(-1, -1), const std::vector<size_t>& cols = { 2, 3, 5, 7, 8 })
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(gps_file))
    {
        cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, cols); // Skip the header
        if (!raw_data.empty())
        {
            dg::UTMConverter converter;
            if (ref_pts.lat >= 0 && ref_pts.lon >= 0) converter.setReference(ref_pts);
            else
            {
                dg::LatLon ll(raw_data.front().at(3), raw_data.front().at(4));
                converter.setReference(ll);
            }
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double status = row->at(2);
                if (status < 0) continue;   // skip nan data

                double timestamp = row->at(0) + 1e-9 * row->at(1);
                dg::LatLon ll(row->at(3), row->at(4));
                dg::Point2 utm = converter.toMetric(ll);
                std::vector<double> datum = { timestamp, utm.x, utm.y };
                data.push_back(datum);
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
        cx::CSVReader::Double2D raw_data = csv.extDouble2D(1, { 2, 3, 5, 6, 7, 8 }); // Skip the header
        if (!raw_data.empty())
        {
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double timestamp = row->at(0) + 1e-9 * row->at(1);
                data.push_back({ timestamp, row->at(5), row->at(2), row->at(3), row->at(4) });
            }
        }
    }
    return data;
}

cx::CSVReader::Double2D readAndroGPS(const std::string& gps_file, const dg::LatLon& ref_pts = dg::LatLon(-1, -1))
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(gps_file, ';'))
    {
        cx::CSVReader::Double2D raw_data = csv.extDouble2D(2, { 31, 22, 23, 28 }); // Skip the header
        if (!raw_data.empty())
        {
            dg::UTMConverter converter;
            if (ref_pts.lat >= 0 && ref_pts.lon >= 0) converter.setReference(ref_pts);
            else
            {
                dg::LatLon ll(raw_data.front().at(1), raw_data.front().at(2));
                converter.setReference(ll);
            }
            for (auto row = raw_data.begin(); row != raw_data.end(); row++)
            {
                double timestamp = 1e-3 * row->at(0);
                dg::LatLon ll(row->at(1), row->at(2));
                dg::Point2 utm = converter.toMetric(ll);
                double accuracy = row->at(3);
                std::vector<double> datum = { timestamp, utm.x, utm.y, accuracy };
                data.push_back(datum);
            }
        }
    }
    return data;
}

cx::CSVReader::Double2D readLocClues(const std::string& clue_file)
{
    cx::CSVReader::Double2D data;
    cx::CSVReader csv;
    if (csv.open(clue_file)) data = csv.extDouble2D(0, { 0, 3, 4 });
    return data;
}

int drawGPSData(const MapGUIProp& gui, const std::string& gps_file, const cv::Vec3b& color, int radius = 1, int gps_smoothing_n = 0)
{
    // Prepare an image and a painter for visualization
    cv::Mat image = cv::imread(gui.image_file);
    if (image.empty()) return -1;
    dg::MapPainter painter;
    painter.configCanvas(gui.origin_px, gui.image_scale, image.size(), 0, 0);
    painter.setImageRotation(gui.image_rotation);
    painter.drawGrid(image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!gui.map_file.empty())
    {
        dg::Map map;
        if (map.load(gui.map_file.c_str())) painter.drawMap(image, &map);
    }

    // Read and draw GPS data
    std::vector<std::vector<double>> gps_data = readROSGPSFix(gps_file, gui.origin_latlon);
    if (gps_data.empty()) return -1;
    if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, color, radius)) return -1;

    // Draw smoothed gps
    if (gps_smoothing_n > 0)
    {
        std::vector<std::vector<double>> tmp = gps_data;
        for (int k = 0; k < gps_smoothing_n; k++)
        {
            int n = (int)gps_data.size();
            for (int i = 1; i < n - 1; i++)
            {
                tmp[i][1] = (2 * gps_data[i][1] + gps_data[i - 1][1] + gps_data[i + 1][1]) / 4;
                tmp[i][2] = (2 * gps_data[i][2] + gps_data[i - 1][2] + gps_data[i + 1][2]) / 4;
            }
            tmp[0][1] = (1.5 * gps_data[0][1] + 2 * gps_data[1][1] - gps_data[2][1]) / 2.5;
            tmp[0][2] = (1.5 * gps_data[0][2] + 2 * gps_data[1][2] - gps_data[2][2]) / 2.5;
            tmp[n - 1][1] = (1.5 * gps_data[n - 1][1] + 2 * gps_data[n - 2][1] - gps_data[n - 3][1]) / 2.5;
            tmp[n - 1][2] = (1.5 * gps_data[n - 1][2] + 2 * gps_data[n - 2][2] - gps_data[n - 3][2]) / 2.5;
            gps_data = tmp;
        }
        if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, cx::COLOR_BLUE, radius)) return -1;
    }

    // Show the image
    cv::namedWindow("::drawGPSData()", gui.wnd_flag);
    cv::imshow("::drawGPSData()", image);
    cv::waitKey();
    return 0;
}

int drawGPSData(const MapGUIProp& gui, const std::vector<std::string>& gps_files, const std::vector<cv::Vec3b>& colors, int radius = 1)
{
    if (gps_files.size() != colors.size()) return -1;

    // Prepare an image and a painter for visualization
    cv::Mat image = cv::imread(gui.image_file);
    if (image.empty()) return -1;
    dg::MapPainter painter;
    painter.configCanvas(gui.origin_px, gui.image_scale, image.size(), 0, 0);
    painter.setImageRotation(gui.image_rotation);
    painter.drawGrid(image, cv::Point2d(100, 100), cv::Vec3b(200, 200, 200), 1, 0.5, cx::COLOR_BLACK, gui.grid_unit_pos);
    painter.drawOrigin(image, 20, cx::COLOR_RED, cx::COLOR_BLUE, 2);
    painter.setParamValue("node_radius", 3);
    painter.setParamValue("node_font_scale", 0);
    painter.setParamValue("node_color", { 255, 100, 100 });
    painter.setParamValue("edge_color", { 150, 100, 100 });
    painter.setParamValue("edge_thickness", 1);
    if (!gui.map_file.empty())
    {
        dg::Map map;
        if (map.load(gui.map_file.c_str())) painter.drawMap(image, &map);
    }

    // Read and draw GPS data
    for (size_t i = 0; i < gps_files.size(); i++)
    {
        std::vector<std::vector<double>> gps_data = readROSGPSFix(gps_files[i], gui.origin_latlon);
        if (gps_data.empty()) return -1;
        if (!LocalizerRunner::drawGPSData(image, &painter, gps_data, colors[i], radius)) return -1;
    }

    // Show the image
    cv::namedWindow("::drawGPSData()", gui.wnd_flag);
    cv::imshow("::drawGPSData()", image);
    cv::waitKey();
    return 0;
}

#endif // End of '__LOCALIZER_RUNNER__'
