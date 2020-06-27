#include "dg_localizer.hpp"

using namespace std;

vector<cv::Vec3d> getGPSData(const string& dataset, double gps_noise = 0.5, const dg::Polar2& gps_offset = dg::Polar2(1, 0))
{
    vector<cv::Vec3d> gps_data;

    // Load the true trajectory
    cx::CSVReader gps_reader;
    if (!gps_reader.open(dataset)) return gps_data;
    cx::CSVReader::Double2D gps_truth = gps_reader.extDouble2D(1, { 0, 1, 2, 3 });
    if (gps_truth.empty()) return gps_data;

    // Generate noisy GPS data
    for (size_t i = 0; i < gps_truth.size(); i++)
    {
        if (gps_truth[i].size() < 4) return gps_data;
        double t = gps_truth[i][0];
        double x = gps_truth[i][1] + gps_offset.lin * cos(gps_truth[i][3] + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
        double y = gps_truth[i][2] + gps_offset.lin * sin(gps_truth[i][3] + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
        gps_data.push_back(cv::Vec3d(t, x, y));
    }
    return gps_data;
}

cv::Ptr<dg::EKFLocalizer> getEKFLocalizer(const string& name)
{
    cv::Ptr<dg::EKFLocalizer> localizer;
    if (name == "EKFLocalizer") localizer = cv::makePtr<dg::EKFLocalizer>();
    else if (name == "EKFLocalizerZeroGyro") localizer = cv::makePtr<dg::EKFLocalizerZeroGyro>();
    else if (name == "EKFLocalizerZeroOdom") localizer = cv::makePtr<dg::EKFLocalizerZeroOdom>();
    else if (name == "EKFLocalizerPostOdom") localizer = cv::makePtr<dg::EKFLocalizerPostOdom>();
    else if (name == "EKFLocalizerSlowGyro") localizer = cv::makePtr<dg::EKFLocalizerSlowGyro>();
    else if (name == "EKFLocalizerHyperTan") localizer = cv::makePtr<dg::EKFLocalizerHyperTan>();
    else if (name == "EKFLocalizerVelModel") localizer = cv::makePtr<dg::EKFLocalizerVelModel>();
    else if (name == "EKFLocalizerFreqPred") localizer = cv::makePtr<dg::EKFLocalizerFreqPred>();
    else if (name == "EKFLocalizerVTAdjust") localizer = cv::makePtr<dg::EKFLocalizerVTAdjust>();
    else if (name == "EKFLocalizerObsvFunc") localizer = cv::makePtr<dg::EKFLocalizerObsvFunc>();
    return localizer;
}

int runLocalizer(cv::Ptr<dg::EKFLocalizer> localizer, const vector<cv::Vec3d>& gps_data, const string& traj_name = "",
    int wait_msec = -1, dg::SimpleRoadPainter* painter = nullptr, const cv::Mat& background = cv::Mat(), double robot_radius = 1, cv::Vec3b robot_color = cv::Vec3b(0, 0, 255))
{
    CV_DbgAssert(!localizer.empty() && !gps_data.empty());

    // Prepare the result trajectory
    FILE* traj_file = nullptr;
    if (!traj_name.empty())
    {
        traj_file = fopen(traj_name.c_str(), "wt");
        if (traj_file == nullptr) return -1;
        fprintf(traj_file, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");
    }

    // Run GPS-only localization
    if (wait_msec < 0 || painter == nullptr)
    {
        for (size_t i = 0; i < gps_data.size(); i++)
        {
            // Apply noisy GPS position
            bool success = localizer->applyPosition({ gps_data[i][1], gps_data[i][2] }, gps_data[i][0]);
            if (!success) fprintf(stderr, "applyPosition() was failed.\n");

            // Record the current state
            if (traj_file != nullptr)
            {
                dg::Pose2 pose = localizer->getPose();
                dg::Polar2 velocity = localizer->getVelocity();
                fprintf(traj_file, "%f, %f, %f, %f, %f, %f\n", gps_data[i][0], pose.x, pose.y, pose.theta, velocity.lin, velocity.ang);
            }
        }
    }
    else
    {
        // Prepare visualization
        dg::CanvasInfo bg_info;
        cv::Mat bg_image = background.clone();
        if (bg_image.empty())
        {
            dg::Point2 box_min(gps_data[0][1], gps_data[0][2]), box_max(gps_data[0][1], gps_data[0][2]);
            for (size_t i = 0; i < gps_data.size(); i++)
            {
                if (gps_data[i][1] < box_min.x) box_min.x = gps_data[i][1];
                if (gps_data[i][2] < box_min.y) box_min.y = gps_data[i][2];
                if (gps_data[i][1] > box_max.x) box_max.x = gps_data[i][1];
                if (gps_data[i][2] > box_max.y) box_max.y = gps_data[i][2];
            }
            bg_info = painter->getCanvasInfo(cv::Rect2d(box_min, box_max));
        }
        else bg_info = painter->getCanvasInfo(cv::Rect2d(), bg_image.size());
        painter->drawMap(bg_image, bg_info);

        dg::Pose2 pose_prev;
        for (size_t i = 0; i < gps_data.size(); i++)
        {
            // Apply noisy GPS position
            dg::Point2 gps(gps_data[i][1], gps_data[i][2]);
            bool success = localizer->applyPosition(gps, gps_data[i][0]);
            if (!success) fprintf(stderr, "applyPosition() was failed.\n");

            // Record the current state
            dg::Pose2 pose = localizer->getPose();
            dg::Polar2 velocity = localizer->getVelocity();
            if (traj_file != nullptr)
                fprintf(traj_file, "%f, %f, %f, %f, %f, %f\n", gps_data[i][0], pose.x, pose.y, pose.theta, velocity.lin, velocity.ang);

            // Visualize the current state
            if (i > 0) cv::line(bg_image, painter->cvtMeter2Pixel(pose_prev, bg_info), painter->cvtMeter2Pixel(pose, bg_info), robot_color, 2);
            pose_prev = pose;
            cv::circle(bg_image, painter->cvtMeter2Pixel(gps, bg_info), 2, cv::Vec3b(64, 64, 64), -1);
            cv::Mat image = bg_image.clone();
            painter->drawNode(image, bg_info, dg::Point2ID(0, pose.x, pose.y), robot_radius, 0, robot_color);
            painter->drawNode(image, bg_info, dg::Point2ID(0, pose.x, pose.y), robot_radius, 0, cv::Vec3b(255, 255, 255) - robot_color, 1);
            cv::Point pose_body = painter->cvtMeter2Pixel(pose, bg_info);
            cv::Point pose_head = painter->cvtMeter2Pixel(pose + dg::Point2(robot_radius * cos(pose.theta), robot_radius * sin(pose.theta)), bg_info);
            cv::line(image, pose_body, pose_head, cv::Vec3b(255, 255, 255) - robot_color, 2);

            double confidence = localizer->getPoseConfidence();
            string state_text = cv::format("Pose: %.3f, %.3f, %.1f / Velocity: %.3f, %.1f / Confidence: %.3f", pose.x, pose.y, cx::cvtRad2Deg(pose.theta), velocity.lin, cx::cvtRad2Deg(velocity.ang), confidence);
            cv::putText(image, state_text, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cx::COLOR_MAGENTA);

            // Show the visualized image
            cv::imshow("runLocalizer", image);
            int key = cv::waitKey(wait_msec);
            if (key == cx::KEY_SPACE) key = cv::waitKey(0);
            if (key == cx::KEY_ESC) return -1;
        }
        cv::waitKey(0);
    }

    if (traj_file != nullptr) fclose(traj_file);
    return 0;
}

int runLocalizerSynthetic(const string& localizer_name, const string& gps_file, const string& traj_file = "",
    double gps_noise = 0.5, dg::Polar2 gps_offset = dg::Polar2(1, 0), double motion_noise = 0.1, const dg::Pose2& init = dg::Pose2(), int wait_msec = 1)
{
    // Read GPS data
    vector<cv::Vec3d> gps_data = getGPSData(gps_file, gps_noise, gps_offset);

    // Prepare a painter for visualization
    dg::SimpleRoadPainter painter;
    if (!painter.setParamValue("pixel_per_meter", 10)) return -2;
    if (!painter.setParamValue("canvas_margin", 2.5)) return -2;
    if (!painter.setParamValue("grid_step", 10)) return -2;
    if (!painter.setParamValue("grid_unit_pos", { 110, 10 })) return -2;
    if (!painter.setParamValue("axes_length", 2)) return -2;

    // Run a localizer
    cv::Ptr<dg::EKFLocalizer> localizer = getEKFLocalizer(localizer_name);
    if (localizer.empty()) return -3;
    if (!localizer->setParamMotionNoise(motion_noise, motion_noise)) return -3;
    if (!localizer->setParamGPSNoise(gps_noise)) return -3;
    if (!localizer->setParamValue("offset_gps", { gps_offset.lin, gps_offset.ang })) return -3;
    if (!localizer->setState(cv::Vec<double, 5>(init.x, init.y, init.theta, 0, 0))) return -2;

    return runLocalizer(localizer, gps_data, "", wait_msec, &painter);
}

int expLocalizersSynthetic(int trial_num = 100)
{
    const vector<double> gps_noise_set = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 };
    const vector<double> gps_offset_set = { 0, 1 };
    const vector<double> gps_freq_set = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    const vector<double> wait_time_set = { 0 };
    //const vector<dg::Pose2> init_set = { dg::Pose2(), dg::Pose2(100, 100, CV_PI) };
    const vector<dg::Pose2> init_set = { dg::Pose2(100, 100, cx::cvtDeg2Rad(-30)) };
    //const vector<double> motion_noise_set = { 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    const vector<double> motion_noise_set = { 0.1, 0.5 };
    const vector<string> traj_set = { "Stop", "Line", "Circle", "Sine", "Square" };
    const vector<string> localizer_name_set = { "EKFLocalizer", "EKFLocalizerHyperTan", "EKFLocalizerZeroGyro" };
    const vector<string> localizer_abbr_set = { "CV", "HT", "ZG" };

    const size_t total_num = gps_noise_set.size() * gps_offset_set.size() * gps_freq_set.size() * wait_time_set.size() * init_set.size() * motion_noise_set.size();
    size_t count = 0;
    for (auto gps_noise = gps_noise_set.begin(); gps_noise != gps_noise_set.end(); gps_noise++)
    {
        for (auto gps_offset = gps_offset_set.begin(); gps_offset != gps_offset_set.end(); gps_offset++)
        {
            for (auto gps_freq = gps_freq_set.begin(); gps_freq != gps_freq_set.end(); gps_freq++)
            {
                for (auto wait_time = wait_time_set.begin(); wait_time != wait_time_set.end(); wait_time++)
                {
                    for (auto init = init_set.begin(); init != init_set.end(); init++)
                    {
                        for (auto motion_noise = motion_noise_set.begin(); motion_noise != motion_noise_set.end(); motion_noise++)
                        {
                            string config_text = cv::format("(%02.0fHz,%02.0fs,%d)(%.1f,%.0fm,%.02f)", *gps_freq, *wait_time, *init, *gps_noise, *gps_offset, *motion_noise);
                            printf("Experiment Progress: %zd / %zd %s\n", ++count, total_num, config_text.c_str());

                            for (auto traj = traj_set.begin(); traj != traj_set.end(); traj++)
                            {
                                string dataset_file = cv::format("data_localizer/synthetic_truth/%s(%02.0fHz,%02.0fs).pose.csv", traj->c_str(), *gps_freq, *wait_time);
                                for (int trial = 0; trial < trial_num; trial++)
                                {
                                    string result_name = cv::format("data_localizer/synthetic_results/%s%s", traj->c_str(), config_text.c_str()) + ".%s" + cv::format(".%03d.csv", trial);

                                    // Generate and save GPS data
                                    dg::Polar2 gps_offset_polar(*gps_offset, 0);
                                    vector<cv::Vec3d> gps_data = getGPSData(dataset_file, *gps_noise, gps_offset_polar);
                                    if (gps_data.empty()) return -1;
                                    FILE* gps_file = fopen(cv::format(result_name.c_str(), "GPS").c_str(), "wt");
                                    if (gps_file == nullptr) return -1;
                                    fprintf(gps_file, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");
                                    for (auto gps = gps_data.begin(); gps != gps_data.end(); gps++)
                                        fprintf(gps_file, "%f, %f, %f, 0, 0, 0\n", gps->val[0], gps->val[1], gps->val[2]);
                                    fclose(gps_file);

                                    // Run three localizers
                                    for (size_t l = 0; l < localizer_name_set.size(); l++)
                                    {
                                        cv::Ptr<dg::EKFLocalizer> localizer = getEKFLocalizer(localizer_name_set[l]);
                                        if (localizer.empty()) return -2;
                                        if (!localizer->setParamMotionNoise(*motion_noise, *motion_noise)) return -2;
                                        if (!localizer->setParamGPSNoise(*gps_noise)) return -2;
                                        if (!localizer->setParamValue("offset_gps", { *gps_offset, 0 })) return -2;
                                        if (!localizer->setState(cv::Vec<double, 5>(init->x, init->y, init->theta, 0, 0))) return -2;

                                        if (runLocalizer(localizer, gps_data, cv::format(result_name.c_str(), localizer_abbr_set[l])) < 0)
                                            printf("  %s failed at %s and %d trial\n", localizer_abbr_set[l].c_str(), traj->c_str(), trial);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int cvtGPSData2UTM(const string& gps_file = "data/191115_ETRI_asen_fix.csv", const string& utm_file = "ETRI_191115.pose.csv", const dg::LatLon& ref_pts = dg::LatLon(36.383837659737, 127.367880828442))
{
    cx::CSVReader csv;
    if (!csv.open(gps_file)) return -1;
    cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 2, 3, 7, 8 }); // Skip the header
    if (csv_ext.empty()) return -1;

    dg::UTMConverter converter;
    if (!converter.setReference(ref_pts)) return -2;

    FILE* fd = fopen(utm_file.c_str(), "wt");
    if (fd == nullptr) return -3;
    fprintf(fd, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        double timestamp = row->at(0) + 1e-9 * row->at(1);
        dg::LatLon ll(row->at(2), row->at(3));
        dg::Point2 xy = converter.toMetric(ll);
        fprintf(fd, "%f, %f, %f, 0, 0, 0\n", timestamp, xy.x, xy.y);
    }
    fclose(fd);
    return 0;
}

int runLocalizerETRI(const string& localizer_name, const string& gps_file, const string& traj_file = "",
    double gps_noise = 0.5, dg::Polar2 gps_offset = dg::Polar2(1, 0), double motion_noise = 0.1, const dg::Pose2& init = dg::Pose2(), int wait_msec = 1, const string& background_file = "")
{
    // Read GPS data
    cx::CSVReader csv;
    if (!csv.open(gps_file)) return -1;
    cx::CSVReader::Double2D csv_ext = csv.extDouble2D(1, { 0, 1, 2 }); // Skip the header
    if (csv_ext.empty()) return -1;
    vector<cv::Vec3d> gps_data;
    for (auto row = csv_ext.begin(); row != csv_ext.end(); row++)
    {
        if (row->size() < 3) return -1;
        gps_data.push_back(cv::Vec3d(row->at(0), row->at(1), row->at(2)));
    }

    // Prepare a painter for visualization
    dg::SimpleRoadPainter painter;
    if (!painter.setParamValue("pixel_per_meter", 1.045)) return -2;
    if (!painter.setParamValue("canvas_margin", 0)) return -2;
    if (!painter.setParamValue("canvas_offset", { 344, 293 })) return -2;
    if (!painter.setParamValue("grid_step", 100)) return -2;
    if (!painter.setParamValue("grid_unit_pos", { 120, 10 })) return -2;
    if (!painter.setParamValue("node_radius", 2)) return -2;
    if (!painter.setParamValue("node_font_scale", 0)) return -2;
    if (!painter.setParamValue("node_color", { 255, 100, 100 })) return -2;
    if (!painter.setParamValue("edge_color", { 150, 100, 100 })) return -2;
    if (!painter.setParamValue("edge_thickness", 1)) return -2;
    cv::Mat background;
    if (!background_file.empty()) background = cv::imread(background_file);

    // Run a localizer
    cv::Ptr<dg::EKFLocalizer> localizer = getEKFLocalizer(localizer_name);
    if (localizer.empty()) return -3;
    if (!localizer->setParamMotionNoise(motion_noise, motion_noise)) return -3;
    if (!localizer->setParamGPSNoise(gps_noise)) return -3;
    if (!localizer->setParamValue("offset_gps", { gps_offset.lin, gps_offset.ang })) return -3;
    if (!localizer->setState(cv::Vec<double, 5>(init.x, init.y, init.theta, 0, 0))) return -2;
    if (!localizer->addParamGPSInaccurateBox(dg::Point2(700, 150), dg::Point2(900, 250))) return -3;

    return runLocalizer(localizer, gps_data, traj_file, wait_msec, &painter, background, 10);
}

int main()
{
    return runLocalizerSynthetic("EKFLocalizerHyperTan", "data_localizer/synthetic_truth/Square(10Hz,00s).pose.csv", "", 0.5, dg::Polar2(1, 0), 0.1);
    return runLocalizerETRI("EKFLocalizerHyperTan", "data_localizer/ETRI_191115_HT.traj.csv", "", 0.5, dg::Polar2(1, 0), 0.1, dg::Pose2(), 1, "data/NaverMap_ETRI(Satellite)_191127.png");
}
