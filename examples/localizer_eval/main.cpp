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

int runLocalizer(const string& localizer_name, const string& traj_name, const vector<cv::Vec3d>& gps_data, double gps_noise = 0.5, const dg::Polar2& gps_offset = dg::Polar2(1, 0), double motion_noise = 1, bool kidnap = true)
{
    // Instantiate the localizer
    cv::Ptr<dg::EKFLocalizer> localizer;
    if (localizer_name == "EKFLocalizer") localizer = cv::makePtr<dg::EKFLocalizer>();
    else if (localizer_name == "EKFLocalizerZeroGyro") localizer = cv::makePtr<dg::EKFLocalizerZeroGyro>();
    else if (localizer_name == "EKFLocalizerZeroOdom") localizer = cv::makePtr<dg::EKFLocalizerZeroOdom>();
    else if (localizer_name == "EKFLocalizerPositive") localizer = cv::makePtr<dg::EKFLocalizerPositive>();
    if (localizer.empty()) return -1;

    cv::Ptr<dg::EKFLocalizer> localizer_ekf = localizer.dynamicCast<dg::EKFLocalizer>();
    if (!localizer_ekf.empty())
    {
        if (!localizer_ekf->setParamMotionNoise(motion_noise, motion_noise)) return -1;
        if (!localizer_ekf->setParamGPSNoise(gps_noise, gps_noise)) return -1;
        if (!localizer_ekf->setParamValue("offset_gps", { gps_offset.lin, gps_offset.ang })) return -1;
        if (kidnap)
        {
            if (!localizer_ekf->setState(cv::Vec<double, 5>(100, 100, 180, 0, 0))) return -1;
        }
    }

    // Prepare the result trajectory
    FILE* traj_file = fopen(traj_name.c_str(), "wt");
    if (traj_file == nullptr) return -1;
    fprintf(traj_file, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");

    // Run GPS-only localization
    for (size_t i = 0; i < gps_data.size(); i++)
    {
        // Apply noisy GPS position
        bool success = localizer->applyPosition({ gps_data[i][1], gps_data[i][2] }, gps_data[i][0]);
        if (!success) fprintf(stderr, "applyPosition() was failed.\n");

        // Print the current pose
        dg::Pose2 pose = localizer->getPose();
        dg::Polar2 velocity = localizer->getVelocity();
        double confidence = localizer->getPoseConfidence();
        fprintf(traj_file, "%f, %f, %f, %f, %f, %f\n", gps_data[i][0], pose.x, pose.y, pose.theta, velocity.lin, velocity.ang);
    }

    fclose(traj_file);
    return 0;
}

int runLocalizerAll(const string& dataset, const string& result_name, double gps_noise = 0.5, const dg::Polar2& gps_offset = dg::Polar2(1, 0), double motion_noise = 1, bool kidnap = false)
{
    vector<cv::Vec3d> gps_data = getGPSData(dataset, gps_noise, gps_offset);
    if (gps_data.empty()) return -1;
    FILE* gps_file = fopen(cv::format(result_name.c_str(), "GPS").c_str(), "wt");
    if (gps_file == nullptr) return -1;
    fprintf(gps_file, "# Time[sec], X[m], Y[m], Theta[rad], LinVel[m/s], AngVel[rad/s]\n");
    for (auto gps = gps_data.begin(); gps != gps_data.end(); gps++)
        fprintf(gps_file, "%f, %f, %f, 0, 0, 0\n", gps->val[0], gps->val[1], gps->val[2]);

    if (runLocalizer("EKFLocalizer", cv::format(result_name.c_str(), "CV"), gps_data, gps_noise, gps_offset, motion_noise, kidnap) < 0) return -1;
    if (runLocalizer("EKFLocalizerZeroGyro", cv::format(result_name.c_str(), "ZG"), gps_data, gps_noise, gps_offset, motion_noise, kidnap) < 0) return -1;
    if (runLocalizer("EKFLocalizerZeroOdom", cv::format(result_name.c_str(), "ZO"), gps_data, gps_noise, gps_offset, motion_noise, kidnap) < 0) return -1;
    if (runLocalizer("EKFLocalizerPositive", cv::format(result_name.c_str(), "PO"), gps_data, gps_noise, gps_offset, motion_noise, kidnap) < 0) return -1;
    return 0;
}
int main()
{
    double gps_noise = 0.5;
    //vector<double> motion_noise_set = { 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    vector<double> motion_noise_set = { 0.1, 1 };
    for (auto motion_noise = motion_noise_set.begin(); motion_noise != motion_noise_set.end(); motion_noise++)
    {
        printf("Motion Noise: %.2f\n", *motion_noise);

        dg::Polar2 gps_offset;
        bool kidnap = false;
        string config_text;

        gps_offset.lin = 1;
        kidnap = false;
        config_text = cv::format("%.1f,%.0f,%.02f,%d", gps_noise, gps_offset.lin, *motion_noise, kidnap);
        if (runLocalizerAll("data_localizer/Stop.pose.csv",   "data_localizer/results/Stop(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Line.pose.csv",   "data_localizer/results/Line(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Sine.pose.csv",   "data_localizer/results/Sine(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Square.pose.csv", "data_localizer/results/Square(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Circle.pose.csv", "data_localizer/results/Circle(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;

        gps_offset.lin = 1;
        kidnap = true;
        config_text = cv::format("%.1f,%.0f,%.02f,%d", gps_noise, gps_offset.lin, *motion_noise, kidnap);
        if (runLocalizerAll("data_localizer/Stop.pose.csv",   "data_localizer/results/Stop(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Line.pose.csv",   "data_localizer/results/Line(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Sine.pose.csv",   "data_localizer/results/Sine(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Square.pose.csv", "data_localizer/results/Square(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Circle.pose.csv", "data_localizer/results/Circle(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;

        gps_offset.lin = 0;
        kidnap = false;
        config_text = cv::format("%.1f,%.0f,%.02f,%d", gps_noise, gps_offset.lin, *motion_noise, kidnap);
        if (runLocalizerAll("data_localizer/Stop.pose.csv",   "data_localizer/results/Stop(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Line.pose.csv",   "data_localizer/results/Line(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Sine.pose.csv",   "data_localizer/results/Sine(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Square.pose.csv", "data_localizer/results/Square(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Circle.pose.csv", "data_localizer/results/Circle(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;

        gps_offset.lin = 0;
        kidnap = true;
        config_text = cv::format("%.1f,%.0f,%.02f,%d", gps_noise, gps_offset.lin, *motion_noise, kidnap);
        if (runLocalizerAll("data_localizer/Stop.pose.csv",   "data_localizer/results/Stop(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Line.pose.csv",   "data_localizer/results/Line(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Sine.pose.csv",   "data_localizer/results/Sine(" + config_text + ",%s).000.csv",   gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Square.pose.csv", "data_localizer/results/Square(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
        if (runLocalizerAll("data_localizer/Circle.pose.csv", "data_localizer/results/Circle(" + config_text + ",%s).000.csv", gps_noise, gps_offset, *motion_noise, kidnap) < 0) return -1;
    }
    return 0;
}
