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

int runLocalizer(const string& localizer_name, const string& traj_name, const vector<cv::Vec3d>& gps_data, double gps_noise = 0.5, const dg::Polar2& gps_offset = dg::Polar2(1, 0), double motion_noise = 1, const dg::Pose2& init = dg::Pose2())
{
    // Instantiate the localizer
    cv::Ptr<dg::EKFLocalizer> localizer;
    if (localizer_name == "EKFLocalizer") localizer = cv::makePtr<dg::EKFLocalizer>();
    else if (localizer_name == "EKFLocalizerZeroGyro") localizer = cv::makePtr<dg::EKFLocalizerZeroGyro>();
    else if (localizer_name == "EKFLocalizerZeroOdom") localizer = cv::makePtr<dg::EKFLocalizerZeroOdom>();
    else if (localizer_name == "EKFLocalizerPostOdom") localizer = cv::makePtr<dg::EKFLocalizerPostOdom>();
    else if (localizer_name == "EKFLocalizerSlowGyro") localizer = cv::makePtr<dg::EKFLocalizerSlowGyro>();
    else if (localizer_name == "EKFLocalizerHyperTan") localizer = cv::makePtr<dg::EKFLocalizerHyperTan>();
    else if (localizer_name == "EKFLocalizerVelModel") localizer = cv::makePtr<dg::EKFLocalizerVelModel>();
    else if (localizer_name == "EKFLocalizerFreqPred") localizer = cv::makePtr<dg::EKFLocalizerFreqPred>();
    else if (localizer_name == "EKFLocalizerVTAdjust") localizer = cv::makePtr<dg::EKFLocalizerVTAdjust>();
    else if (localizer_name == "EKFLocalizerObsvFunc") localizer = cv::makePtr<dg::EKFLocalizerObsvFunc>();
    if (localizer.empty()) return -1;

    cv::Ptr<dg::EKFLocalizer> localizer_ekf = localizer.dynamicCast<dg::EKFLocalizer>();
    if (!localizer_ekf.empty())
    {
        if (!localizer_ekf->setParamMotionNoise(motion_noise, motion_noise)) return -1;
        if (!localizer_ekf->setParamGPSNoise(gps_noise)) return -1;
        if (!localizer_ekf->setParamValue("offset_gps", { gps_offset.lin, gps_offset.ang })) return -1;
        if (!localizer_ekf->setState(cv::Vec<double, 5>(init.x, init.y, init.theta, 0, 0))) return -1;
        //if (!localizer_ekf->addParamGPSInaccurateBox(dg::Point2(700, 150), dg::Point2(150, 250))) return -1;
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

int runLocalizersSynthetic(int trial_num = 100)
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
                                    if (runLocalizer("EKFLocalizer", cv::format(result_name.c_str(), "CV"), gps_data, *gps_noise, gps_offset_polar, *motion_noise, *init) < 0)
                                        printf("  CV failed at %s and %d trial\n", traj->c_str(), trial);
                                    if (runLocalizer("EKFLocalizerZeroGyro", cv::format(result_name.c_str(), "ZG"), gps_data, *gps_noise, gps_offset_polar, *motion_noise, *init) < 0)
                                        printf("  ZG failed at %s and %d trial\n", traj->c_str(), trial);
                                    if (runLocalizer("EKFLocalizerHyperTan", cv::format(result_name.c_str(), "HT"), gps_data, *gps_noise, gps_offset_polar, *motion_noise, *init) < 0)
                                        printf("  HT failed at %s and %d trial\n", traj->c_str(), trial);
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

int runLocalizersETRI()
{
    string localizer = "EKFLocalizerHyperTan";
    string traj_file = "data_localizer/ETRI_191115_HT.traj.csv";
    string gps_file  = "data_localizer/real_data/ETRI_191115.gps.csv";
    double gps_noise = 0.5;
    dg::Polar2 gps_offset(1, 0);
    double motion_noise = 0.1;

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

    // Run a localizer
    return runLocalizer(localizer, traj_file, gps_data, gps_noise, gps_offset, motion_noise);
}

int main()
{
    return runLocalizersETRI();
}
