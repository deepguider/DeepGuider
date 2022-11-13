#ifndef __LOCALIZER_RUNNER__
#define __LOCALIZER_RUNNER__

#include "dg_localizer.hpp"
#include "localizer/data_loader.hpp"
#include "intersection_cls/intersection_localizer.hpp"
#include "vps/vps_localizer.hpp"
#include "roadlr/roadlr_localizer.hpp"
#include "ocr_recog/ocr_localizer.hpp"
#include "dg_roadtheta.hpp"
#include "utils/viewport.hpp"
#include "localizer/localizer_mcl.hpp"


void onMouseEventLocalizer(int event, int x, int y, int flags, void* param);

class LocalizerRunner : public dg::SharedInterface
{
protected:
    bool         m_dest_defined = false;
    dg::Point2   m_dest_xy;

    cv::Ptr<dg::BaseLocalizer> m_localizer;
    dg::IntersectionLocalizer m_intersection_localizer;
    dg::OCRLocalizer m_ocr_localizer;
    dg::VPSLocalizer m_vps_localizer;
    dg::RoadLRLocalizer m_lr_localizer;
    dg::RoadThetaLocalizer m_roadtheta;

public:
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
        bool ok = m_map.getPath(curr_pose, m_dest_xy, path);
        if (!ok) return false;
        setPath(path);
        return true;
    }

    int runLocalizer(cv::Ptr<dg::BaseLocalizer> localizer, dg::DataLoader& data_loader, bool use_mcl)
    {
        bool show_projected_history = true; // debugging purpose

        CV_DbgAssert(!localizer.empty() && !data_loader.empty());
        m_localizer = localizer;
        m_localizer->setShared(this);
        cv::Ptr<dg::DGLocalizer> dg_localizer = localizer.dynamicCast<dg::DGLocalizer>();
        cv::Ptr<dg::DGLocalizerMCL> mcl_localizer = localizer.dynamicCast<dg::DGLocalizerMCL>();

        // initialize module localizers
        m_intersection_localizer.initialize_without_python(this);
        m_ocr_localizer.initialize_without_python(this);
        m_vps_localizer.initialize_without_python(this);
        m_lr_localizer.initialize_without_python(this);
        m_roadtheta.initialize(this);

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
        cv::Mat zoom_bg_image = zoom_background.clone();
        m_viewport.initialize(bg_image, m_view_size, m_view_offset);
        //m_viewport.setZoom(2);

        // Run localization with GPS and other sensors
        if (show_gui)
        {
            cv::namedWindow("LocalizerRunner::runLocalizer()", gui_wnd_flag);
            cv::resizeWindow("LocalizerRunner::runLocalizer()", m_view_size);
        }
        cv::setMouseCallback("LocalizerRunner::runLocalizer()", onMouseEventLocalizer, this);
        cv::Mat cam_image;

        double timestart = data_loader.getStartTime();
        cv::Mat gui_image;
        while (1)
        {
            int type;
            std::vector<double> vdata;
            std::vector<std::string> sdata;
            dg::Timestamp data_time;
            if (data_loader.getNext(type, vdata, sdata, data_time) == false) break;

            bool update_gui = false;
            if (type == dg::DATA_IMU)
            {
                auto euler = cx::cvtQuat2EulerAng(vdata[1], vdata[2], vdata[3], vdata[4]);
                bool success = localizer->applyIMUCompass(euler.z, data_time, 1);
                if (!success) fprintf(stderr, "applyIMUCompass() was failed.\n");
            }
            else if (type == dg::DATA_ODO)
            {
                dg::Pose2 odo_pose(vdata[1], vdata[2], vdata[3]);
                bool success = (apply_odo) ? localizer->applyOdometry(odo_pose, data_time, 1) : true;
                if (success) printf("applyOdometry()\n");
                if (!success) fprintf(stderr, "applyOdometry() was failed.\n");

                if (apply_odo && !apply_gps)
                {
                    cam_image = data_loader.getFrame(data_time);
                    update_gui = true;
                }
            }
            else if (type == dg::DATA_GPS)
            {
                dg::LatLon gps_datum(vdata[1], vdata[2]);
                dg::Point2 gps_xy = toMetric(gps_datum);
                bool success = (apply_gps) ? localizer->applyGPS(gps_xy, data_time, 1) : true;
                if (success) printf("applyGPS()\n"); 
                if (!success) fprintf(stderr, "applyGPS() was failed.\n");

                if (gui_gps_radius > 0)
                {
                    if (show_gui) gui_painter->drawPoint(bg_image, gps_xy, gui_gps_radius, gui_gps_color);
                    if (show_zoom) zoom_painter->drawPoint(zoom_bg_image, gps_xy, gui_gps_radius, gui_gps_color);
                }

                cam_image = data_loader.getFrame(data_time);
                update_gui = true;
            }
            else if (type == dg::DATA_OCR)
            {
                std::string name = sdata[0];
                double conf = vdata[1];
                double xmin = vdata[2];
                double ymin = vdata[3];
                double xmax = vdata[4];
                double ymax = vdata[5];
                dg::POI* poi;
                dg::Polar2 relative;
                double confidence;
                if (m_ocr_localizer.applyPreprocessed(name, xmin, ymin, xmax, ymax, conf, data_time, poi, relative, confidence))
                {
                    bool success = localizer->applyPOI(*poi, relative, data_time, confidence);
                    if (!success) fprintf(stderr, "applyOCR() was failed.\n");
                }
            }
            else if (type == dg::DATA_POI)
            {
                dg::Point2 clue_xy(vdata[2], vdata[3]);
                dg::Polar2 relative(vdata[4], vdata[5]);
                double confidence = vdata[6];
                bool success = localizer->applyPOI(clue_xy, relative, data_time, confidence);
                if (!success) fprintf(stderr, "applyPOI() was failed.\n");
            }
            else if (type == dg::DATA_VPS)
            {
                dg::Point2 clue_xy = toMetric(dg::LatLon(vdata[3], vdata[4]));
                dg::Polar2 relative(vdata[5], vdata[6]);
                double confidence = vdata[7];
                bool success = localizer->applyVPS(clue_xy, relative, data_time, confidence);
                if (!success) fprintf(stderr, "applyVPS() was failed.\n");
            }
            else if (type == dg::DATA_IntersectCls)
            {
                double cls = vdata[1];
                double cls_conf = vdata[2];
                dg::Point2 xy;
                double xy_confidence;
                bool xy_valid = false;
                if (m_intersection_localizer.applyPreprocessed(cls, cls_conf, data_time, xy, xy_confidence, xy_valid) && xy_valid)
                {
                    bool success = localizer->applyIntersectCls(xy, data_time, xy_confidence);
                    if (!success) fprintf(stderr, "applyIntersectCls() was failed.\n");
                }
            }
            else if (type == dg::DATA_RoadLR && dg_localizer)
            {
                double cls = vdata[1];
                double cls_conf = vdata[2];
                int lr_cls;
                double lr_confidence;
                if (m_lr_localizer.applyPreprocessed(cls, cls_conf, data_time, lr_cls, lr_confidence))
                {
                    bool success = dg_localizer->applyRoadLR(lr_cls, data_time, lr_confidence);
                    if (!success) fprintf(stderr, "applyLRPose() was failed.\n");
                }
            }
            else if (type == dg::DATA_RoadTheta)
            {
                double theta = vdata[3];
                double confidence = vdata[4];
                bool success = localizer->applyRoadTheta(theta, data_time, confidence);
                if (!success) fprintf(stderr, "applyRoadTheta() was failed.\n");
            }

            // Record the current state on the CSV file
            if (out_traj != nullptr)
            {
                dg::Pose2 pose = localizer->getPose();
                fprintf(out_traj, "%f, %f, %f, %f\n", data_time, pose.x, pose.y, pose.theta);
            }

            // Visualize and show the current state as an image
            if (show_gui && update_gui)
            {
                dg::Pose2 pose = localizer->getPose();
                if (gui_traj_radius > 0) gui_painter->drawPoint(bg_image, pose, gui_traj_radius, gui_robot_color);
                dg::Pose2 pose_px = gui_painter->cvtValue2Pixel(pose);
                m_viewport.centerizeViewportTo(pose_px, 0.2, 0.2);
                m_viewport.getViewportImage(gui_image);

                // Draw Particles
                if (use_mcl)
                {
                    const std::vector<dg::Particle>& particles = mcl_localizer->getParticles();
                    if (!particles.empty())
                    {
                        dg::Pose2 robot_pose = localizer->getPose();

                        // draw particles
                        int nradius = 2;
                        cv::Vec3b ncolor = cv::Vec3b(0, 255, 0);
                        dg::MapPainter* painter = (dg::MapPainter*)gui_painter;
                        dg::Map* map = dg_localizer->getMap();
                        for (auto itr = particles.begin(); itr != particles.end(); itr++)
                        {
                            dg::Node* from = itr->start_node;
                            dg::Edge* edge = itr->edge;
                            dg::Node* to = map->getConnectedNode(from, edge->id);

                            dg::Point2 v = *to - *from;
                            dg::Point2 pose_m = *from + itr->dist * v / edge->length;

                            double theta = atan2(v.y, v.x);
                            if (fabs(cx::trimRad(theta - robot_pose.theta)) < CV_PI / 2)
                                painter->drawNode(gui_image, dg::Point2ID(0, pose_m), nradius, 0, ncolor, m_viewport.offset(), m_viewport.zoom());
                            else
                                painter->drawNode(gui_image, dg::Point2ID(0, pose_m), nradius, 0, cv::Vec3b(0, 0, 0), m_viewport.offset(), m_viewport.zoom());
                        }

                        // draw best path points
                        cv::Vec3b ecolor = cv::Vec3b(255, 0, 0);
                        ncolor = cv::Vec3b(255, 0, 0);
                        nradius = 3;
                        int ethickness = 1;
                        for (int idx = 1; idx < (int)mcl_localizer->m_best_path_pts.size(); idx++)
                        {
                            painter->drawEdge(gui_image, mcl_localizer->m_best_path_pts[idx - 1], mcl_localizer->m_best_path_pts[idx], nradius, ecolor, ethickness, m_viewport.offset(), m_viewport.zoom());
                        }
                        for (int idx = 1; idx < (int)mcl_localizer->m_best_path_pts.size() - 1; idx++)
                        {
                            painter->drawNode(gui_image, dg::Point2ID(0, mcl_localizer->m_best_path_pts[idx]), nradius, 0, ncolor, m_viewport.offset(), m_viewport.zoom());
                        }

                        // draw best odo points
                        ecolor = cv::Vec3b(0, 255, 255);
                        ncolor = cv::Vec3b(0, 128, 128);
                        nradius = 3;
                        ethickness = 1;
                        for (int idx = 1; idx < (int)mcl_localizer->m_best_odo_pts.size(); idx++)
                        {
                            painter->drawEdge(gui_image, mcl_localizer->m_best_odo_pts[idx - 1], mcl_localizer->m_best_odo_pts[idx], nradius, ecolor, ethickness, m_viewport.offset(), m_viewport.zoom());
                        }
                        for (int idx = 1; idx < (int)mcl_localizer->m_best_odo_pts.size() - 1; idx++)
                        {
                            painter->drawNode(gui_image, dg::Point2ID(0, mcl_localizer->m_best_odo_pts[idx]), nradius, 0, ncolor, m_viewport.offset(), m_viewport.zoom());
                        }

                        // draw best particle
                        dg::Particle best = mcl_localizer->m_best_particle;
                        if (best.start_node != nullptr)
                        {
                            dg::Node* from = best.start_node;
                            dg::Edge* edge = best.edge;
                            dg::Node* to = map->getConnectedNode(from, edge->id);
                            dg::Point2 v = *to - *from;
                            dg::Point2 pose_best = *from + best.dist * v / edge->length;
                            painter->drawNode(gui_image, dg::Point2ID(0, pose_best), nradius * 2, 0, cv::Vec3b(255, 0, 0), m_viewport.offset(), m_viewport.zoom());
                        }
                    }
                }

                // show sensor status
                cv::Scalar color_bg(255, 255, 255);
                cv::Scalar color_active(255, 0, 0);
                cv::Scalar color_deactive(128, 128, 128);
                double gui_fscale = 0.8;
                cv::Point gui_xy(10, 50);
                std::string gui_msg = "GPS(G)";
                cv::putText(gui_image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, color_bg, 5);
                if (apply_gps) cv::putText(gui_image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, color_active, 2);
                else cv::putText(gui_image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, color_deactive, 2);
                gui_xy.y += 40;

                gui_msg = "ODO(O)";
                cv::putText(gui_image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, color_bg, 5);
                if (apply_odo) cv::putText(gui_image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, color_active, 2);
                else cv::putText(gui_image, gui_msg, gui_xy, cv::FONT_HERSHEY_SIMPLEX, gui_fscale, color_deactive, 2);
                gui_xy.y += 40;

                // Draw the image given from the camera
                if (!cam_image.empty() && video_resize > 0)
                {
                    cv::Mat resize;
                    cv::resize(cam_image, resize, cv::Size(), video_resize, video_resize);
                    cx::Painter::pasteImage(gui_image, resize, video_offset);
                }

                // Draw path
                dg::Path path = getPath();
                if (!path.empty())
                {
                    // draw current path
                    dg::MapPainter* painter = (dg::MapPainter*)gui_painter;
                    const cv::Vec3b ecolor = cv::Vec3b(255, 0, 0);
                    const cv::Vec3b ncolor = cv::Vec3b(0, 255, 255);
                    int nradius = 3;
                    int ethickness = 1;
                    for (int idx = 1; idx < (int)path.pts.size(); idx++)
                    {
                        painter->drawEdge(gui_image, path.pts[idx - 1], path.pts[idx], nradius, ecolor, ethickness, m_viewport.offset(), m_viewport.zoom());
                    }
                    for (int idx = 1; idx < (int)path.pts.size() - 1; idx++)
                    {
                        painter->drawNode(gui_image, dg::Point2ID(0, path.pts[idx]), nradius, 0, ncolor, m_viewport.offset(), m_viewport.zoom());
                    }

                    if (show_projected_history)
                    {
                        // draw projected pose history
                        const dg::RingBuffer<dg::Pose2>& pose_history = dg_localizer->getProjectedPoseHistory();
                        int j = pose_history.data_count() - 1;
                        while (j >= 0)
                        {
                            painter->drawNode(gui_image, dg::Point2ID(0, pose_history[j]), nradius, 0, cv::Vec3b(255, 255, 0), m_viewport.offset(), m_viewport.zoom());
                            j--;
                        }
                    }
                }

                // Draw Robot
                genStateImage(gui_image, localizer, data_time - timestart, gui_painter, m_viewport.offset(), m_viewport.zoom());

                // Draw the current state on the bigger background
                if (show_zoom)
                {
                    if (zoom_user_drag)
                    {
                        cv::Point2d pos = gui_painter->cvtPixel2Value(zoom_user_point);
                        pasteZoomedImage(gui_image, pos, zoom_background);
                    }
                    else
                    {
                        if (gui_traj_radius > 0) zoom_painter->drawPoint(zoom_bg_image, pose, gui_traj_radius, gui_robot_color);

                        cv::Mat zoom_image = zoom_bg_image.clone();
                        genStateImage(zoom_image, localizer, data_time - timestart, zoom_painter);
                        pasteZoomedImage(gui_image, pose, zoom_image);
                    }
                }

                // Record the current visualization on the AVI file
                if (out_video.isConfigured() && rec_video_resize > 0)
                {
                    out_video << gui_image;
                }

                cv::imshow("LocalizerRunner::runLocalizer()", gui_image);
                int key = cv::waitKey(gui_wnd_wait_msec);
                if (key == cx::KEY_SPACE) key = cv::waitKey(0);
                if (key == '1') gui_wnd_wait_msec = 1;
                if (key == '2') gui_wnd_wait_msec = 100;
                if (key == '3') gui_wnd_wait_msec = 500;
                if (key == '4') gui_wnd_wait_msec = 1000;
                if (key == 'g')
                {
                    apply_gps = !apply_gps;
                    mcl_localizer->resetGPSActivation(apply_gps);
                }
                if (key == 'o')
                {
                    apply_odo = !apply_odo;
                    mcl_localizer->resetGPSActivation(apply_odo);
                }
                if (key == 's')
                {
                    // file name
                    time_t start_t;
                    time(&start_t);
                    tm _tm;
                    localtime_s(&_tm, &start_t);
                    char szfilename[255];
                    strftime(szfilename, 255, "odo_path_%y%m%d_%H%M%S.yml", &_tm);
                    std::string fname = szfilename;

                    cv::FileStorage fs(fname, cv::FileStorage::WRITE);
                    if (fs.isOpened())
                    {
                        std::vector<cv::Point2d> path_pts;
                        for (size_t k = 0; k < mcl_localizer->m_best_path.pts.size(); k++)
                            path_pts.push_back(cv::Point2d(mcl_localizer->m_best_path.pts[k].x, mcl_localizer->m_best_path.pts[k].y));
                        fs << "odo" << mcl_localizer->m_odo_pts_original;
                        fs << "path" << path_pts;
                    }
                }
                if (key == cx::KEY_ESC) break;
            }
        }
        if (out_traj != nullptr) fclose(out_traj);
        if (gui_wnd_wait_exit) cv::waitKey(0);
        return 0;
    }

    void procMouseEvent(int evt, int x, int y, int flags)
    {
        m_viewport.procMouseEvent(evt, x, y, flags);

        if (evt == cv::EVENT_MOUSEMOVE)
        {
            if (zoom_user_drag)
            {
                zoom_user_point = m_viewport.cvtView2Pixel(cv::Point(x, y));
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
            cv::Point2d px_dest = m_viewport.cvtView2Pixel(cv::Point(x, y));
            cv::Point2d p_dest = gui_painter->cvtPixel2Value(px_dest);
            dg::Path path;
            bool ok = m_map.getPath(p_start, p_dest, path);
            if (ok)
            {
                setPath(path);
                m_dest_xy = p_dest;
                m_dest_defined = true;
            }
        }
        else if (evt == cv::EVENT_RBUTTONDOWN)
        {
            cv::Point2d px = m_viewport.cvtView2Pixel(cv::Point(x, y));
            cv::Point2d p = gui_painter->cvtPixel2Value(px);
            printf("x = %lf, y = %lf\n", p.x, p.y);

            zoom_user_point = px;
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

    void genStateImage(cv::Mat& image, const cv::Ptr<dg::BaseLocalizer> localizer, double timestamp, cx::Painter* painter = nullptr, const cv::Point2d& offset = cv::Point2d(0, 0), double zoom = 1)
    {
        if (painter == nullptr) painter = gui_painter;
        CV_DbgAssert(!localizer.empty() && painter != nullptr);

        // Get pose
        dg::Pose2 pose_m = localizer->getPose();
        cv::Ptr<dg::DGLocalizerMCL> mcl_localizer = localizer.dynamicCast<dg::DGLocalizerMCL>();
        if(mcl_localizer) pose_m = mcl_localizer->getPoseMCL();

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
                                    painter->drawLine(image, (*(*n)-offset)*zoom, (*to-offset)*zoom, gui_topo_loc_color, (int)(gui_topo_loc_thickness * zoom+0.5));
                            }
                            painter->drawPoint(image, (*(*n)-offset)*zoom, (int)(gui_topo_loc_radius*zoom+0.5), gui_topo_loc_color, (int)(gui_topo_loc_thickness * zoom+0.5));
                        }
                    }
                }

                if (gui_topo_ref_radius > 0)
                {
                    dg::Node* ref_node = map->getNode(pose_t.node_id);
                    if (ref_node != nullptr) painter->drawPoint(image, (*ref_node-offset)*zoom, gui_topo_ref_radius, gui_topo_ref_color, (int)(gui_topo_ref_thickness * zoom+0.5));
                }
            }
        }
        if (gui_robot_radius > 0)
        {
            cv::Point2d pose_px = (painter->cvtValue2Pixel(pose_m) - offset) * zoom;
            cv::circle(image, pose_px, gui_robot_radius, gui_robot_color, -1);
            cv::circle(image, pose_px, gui_robot_radius, cv::Vec3b(255, 255, 255) - gui_robot_color, 1);
            cv::Point2d head_px(gui_robot_radius * cos(pose_m.theta), -gui_robot_radius * sin(pose_m.theta));
            cv::line(image, pose_px, pose_px + head_px, cv::Vec3b(255, 255, 255) - gui_robot_color, gui_robot_thickness); // Robot heading
        }
        if (gui_covar_scale > 0 && gui_covar_thickness > 0)
        {
            cv::Ptr<cx::EKF> ekf = localizer.dynamicCast<cx::EKF>();
            cv::Ptr<dg::DGLocalizer> path_localizer = localizer.dynamicCast<dg::DGLocalizer>();
            if (!ekf.empty() || !path_localizer.empty())
            {
                cv::Mat eval, evec;
                cv::Mat covar = (ekf) ? ekf->getStateCov() : path_localizer->getStateCov();
                cv::eigen(covar(cv::Rect(0, 0, 2, 2)), eval, evec);
                double pixel_per_value = painter->getPixel2Value().x * zoom;
                cv::RotatedRect covar_box((painter->cvtValue2Pixel(pose_m)-offset)*zoom,
                    cv::Size2d(pixel_per_value * gui_covar_scale * sqrt(eval.at<double>(0)), pixel_per_value * gui_covar_scale * sqrt(eval.at<double>(1))),
                    static_cast<float>(cx::cvtRad2Deg(atan2(-evec.at<double>(1, 0), evec.at<double>(0, 0)))));
                cv::ellipse(image, covar_box, gui_covar_color, (int)(gui_covar_thickness * zoom+0.5)); // EKF covariance
            }
        }
        if (gui_text_scale > 0)
        {
            //dg::Polar2 velocity = localizer->getVelocity();
            dg::Polar2 velocity = dg::Polar2();
            std::string metric_text = cv::format("Time: %.2f / Pose: %.2f, %.2f, %.0f / Velocity: %.2f, %.0f",
                timestamp, pose_m.x, pose_m.y, cx::cvtRad2Deg(pose_m.theta), velocity.lin, cx::cvtRad2Deg(velocity.ang));
            cv::putText(image, metric_text, (gui_text_offset - offset)*zoom, cv::FONT_HERSHEY_PLAIN, gui_text_scale * zoom, gui_text_color, (int)(gui_text_thickness * zoom+0.5));
            if (!localizer_topo.empty() && pose_t.node_id > 0)
            {
                std::string topo_text = cv::format("Node ID: %zd, Edge Idx: %d, Dist: %.2f, Head: %.0f", pose_t.node_id, pose_t.edge_idx, pose_t.dist, cx::cvtRad2Deg(pose_t.head));
                int topo_text_offset = static_cast<int>(20 * gui_text_scale);
                cv::putText(image, topo_text, (gui_text_offset - offset + cv::Point2d(0, topo_text_offset))*zoom, cv::FONT_HERSHEY_PLAIN, gui_text_scale * zoom, gui_text_color, (int)(gui_text_thickness*zoom+0.5));
            }
        }
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

public:
    cx::Painter* gui_painter = nullptr;
    cv::Mat      gui_background;
    int          gui_robot_radius = 10;
    cv::Vec3b    gui_robot_color = cv::Vec3b(0, 0, 255);
    int          gui_robot_thickness = 2;
    int          gui_traj_radius = 2;
    cv::Vec3b    gui_gps_color = cv::Vec3b(100, 100, 100);
    int          gui_gps_radius = 2;
    double       gui_covar_scale = 30;
    cv::Vec3b    gui_covar_color = cv::Vec3b(0, 255, 0);;
    int          gui_covar_thickness = 1;
    double       gui_text_scale = 1;
    cv::Point2d  gui_text_offset = cv::Point2d(5, 15);
    cv::Vec3b    gui_text_color = cx::COLOR_MAGENTA;
    int          gui_text_thickness = 1;
    int          gui_topo_ref_radius = 0;
    cv::Vec3b    gui_topo_ref_color = cv::Vec3b(255, 127, 255);
    int          gui_topo_ref_thickness = -1;
    int          gui_topo_loc_radius = 0;
    cv::Vec3b    gui_topo_loc_color = cv::Vec3b(127, 255, 127);
    int          gui_topo_loc_thickness = 2;
    std::string  gui_clue_text = "Intersection (%.2f)";
    cv::Vec3b    gui_clue_color = cv::Vec3b(255, 0, 0);
    int          gui_clue_thickness = 2;
    int          gui_wnd_flag = cv::WindowFlags::WINDOW_AUTOSIZE;
    int          gui_wnd_wait_msec = 1;
    bool         gui_wnd_wait_exit = false;

    double       video_resize = 1;
    cv::Point    video_offset = cv::Point(0, 0);

    cv::Point    m_view_offset = cv::Point(0, 0);
    cv::Size     m_view_size = cv::Size(1920, 1080);
    dg::Viewport m_viewport;

    cx::Painter* zoom_painter = nullptr;
    cv::Mat      zoom_background;
    double       zoom_radius = 0;
    cv::Point    zoom_offset = cv::Point(5, 20);
    cv::Vec3b    zoom_box_color;
    int          zoom_box_thickness = 1;
    bool         zoom_user_drag = false;
    cv::Point    zoom_user_point;
    bool         show_zoom = true;

    std::string  rec_traj_name;
    std::string  rec_video_name;
    double       rec_video_fps = 10;
    int          rec_video_fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    double       rec_video_quality = 100;
    double       rec_video_resize = 1;

    bool        apply_gps = true;
    bool        apply_odo = true;

}; // End of 'LocalizerRunner'

void onMouseEventLocalizer(int event, int x, int y, int flags, void* param)
{
    LocalizerRunner* localizer = (LocalizerRunner*)param;
    localizer->procMouseEvent(event, x, y, flags);
}


#endif // End of '__LOCALIZER_RUNNER__'
