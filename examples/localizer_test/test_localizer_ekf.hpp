#ifndef __TEST_LOCALIZER_EKF__
#define __TEST_LOCALIZER_EKF__

#include "dg_localizer.hpp"
#include "utils/vvs.h"

int testLocEKFGPS(double gps_noise = 0.3, const dg::Polar2& gps_offset = dg::Polar2(1, 0), double interval = 0.1, double velocity = 1)
{
    dg::EKFLocalizer localizer;
    if (!localizer.setParamGPSNoise(gps_noise)) return -1;
    if (!localizer.setParamValue("offset_gps", { gps_offset.lin, gps_offset.ang })) return -1;

    printf("| Time [sec] | GPS Data [m] | Pose [m] [deg] | Velocity [m/s] [deg/s] | Confidence |\n");
    printf("| ---------- | ------------ | -------------- | ---------------------- | ---------- |\n");
    for (double t = interval; t < 10; t += interval)
    {
        dg::Pose2 truth(velocity * t, 1, 0); // Going straight from (0, 1, 0)

        // Apply noisy GPS position
        dg::Point2 gps = truth;
        gps.x += gps_offset.lin * cos(truth.theta + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
        gps.y += gps_offset.lin * sin(truth.theta + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
        if (!localizer.applyPosition(gps, t)) return -1;

        // Print the current pose
        dg::Pose2 pose = localizer.getPose();
        dg::Polar2 velocity = localizer.getVelocity();
        double confidence = localizer.getPoseConfidence();
        printf("| %.1f | %.3f, %.3f | %.3f, %.3f, %.1f | %.3f, %.1f | %.3f |\n",
            t, gps.x, gps.y, pose.x, pose.y, cx::cvtRad2Deg(pose.theta), velocity.lin, cx::cvtRad2Deg(velocity.ang), confidence);
    }
    return 0;
}

int testLocEKFGyroGPS(double gyro_noise = 0.01, double gps_noise = 0.3, const dg::Polar2& gps_offset = dg::Polar2(1, 0), double interval = 0.1, double velocity = 1)
{
    dg::EKFLocalizer localizer;
    if (!localizer.setParamMotionNoise(1, 1, gyro_noise)) return -1;
    if (!localizer.setParamGPSNoise(gps_noise)) return -1;
    if (!localizer.setParamValue("offset_gps", { gps_offset.lin, gps_offset.ang })) return -1;

    printf("| Time [sec] | Gyro Data [deg] | GPS Data [m] | Pose [m] [deg] | Velocity [m/s] [deg/s] | Confidence |\n");
    printf("| ---------- | --------------- | ------------ | -------------- | ---------------------- | ---------- |\n");
    double gyro_prev = 0;
    for (double t = interval; t < 10; t += interval)
    {
        dg::Pose2 truth(velocity * t, 1, 0); // Going straight from (0, 1, 0)

        // Apply noisy gyroscope data
        double gyro = cv::theRNG().gaussian(gyro_noise);
        if (!localizer.applyOdometry(gyro, gyro_prev, t, t - interval)) return -1;
        gyro_prev = gyro;

        // Apply noisy GPS position
        dg::Point2 gps = truth;
        gps.x += gps_offset.lin * cos(truth.theta + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
        gps.y += gps_offset.lin * sin(truth.theta + gps_offset.ang) + cv::theRNG().gaussian(gps_noise);
        if (!localizer.applyPosition(gps, t)) return -1;

        // Print the current pose
        dg::Pose2 pose = localizer.getPose();
        dg::Polar2 velocity = localizer.getVelocity();
        double confidence = localizer.getPoseConfidence();
        printf("| %.1f | %.1f | %.3f, %.3f | %.3f, %.3f, %.1f | %.3f, %.1f | %.3f |\n",
            t, gyro, gps.x, gps.y, pose.x, pose.y, cx::cvtRad2Deg(pose.theta), velocity.lin, cx::cvtRad2Deg(velocity.ang), confidence);
    }
    return 0;
}

int configureLocalizer(cv::Ptr<dg::BaseLocalizer> localizer)
{
    if (!localizer->setParamMotionNoise(1, 10)) return -1;      // linear_velocity(m), angular_velocity(deg)
    if (!localizer->setParamMotionBounds(1, 10)) return -1;     // max. linear_velocity(m), max. angular_velocity(deg)
    if (!localizer->setParamGPSNoise(10)) return -1;            // position error(m)
    if (!localizer->setParamGPSOffset(1, 0)) return -1;         // displacement(lin,ang) from robot origin
    if (!localizer->setParamIMUCompassNoise(1, 0)) return -1;   // angle arror(deg), angle offset(deg)
    if (!localizer->setParamPOINoise(1, 10)) return -1;         // position error(m), orientation error(deg)
    if (!localizer->setParamVPSNoise(1, 10)) return -1;         // position error(m), orientation error(deg)
    if (!localizer->setParamIntersectClsNoise(0.1)) return -1;  // position error(m)
    if (!localizer->setParamRoadThetaNoise(50)) return -1;      // angle arror(deg)
    if (!localizer->setParamCameraOffset(1, 0)) return -1;      // displacement(lin,ang) from robot origin
    localizer->setParamValue("gps_reverse_vel", -1);
    localizer->setParamValue("search_turn_weight", 100);
    localizer->setParamValue("track_near_radius", 20);
    localizer->setParamValue("enable_path_projection", true);
    localizer->setParamValue("enable_map_projection", false);
    localizer->setParamValue("enable_backtracking_ekf", true);
    localizer->setParamValue("enable_gps_smoothing", false);
    return 0;
}

int testLocEKFBacktracking()
{

    class SharedTemp : public dg::SharedInterface
    {
    public:
        virtual dg::Pose2 getPose(dg::Timestamp* timestamp = nullptr) const { return dg::Pose2(); }
        virtual dg::LatLon getPoseGPS(dg::Timestamp* timestamp = nullptr) const { return dg::LatLon(); }
        virtual dg::TopometricPose getPoseTopometric(dg::Timestamp* timestamp = nullptr) const { return dg::TopometricPose(); }
        virtual double getPoseConfidence(dg::Timestamp* timestamp = nullptr) const { return 1; }
    };
    dg::Map map;
    map.setReference(dg::LatLon(36.379208, 127.364585));
    VVS_CHECK_TRUE(map.load("data/ETRI/TopoMap_ETRI_210803.csv"));
    SharedTemp shared;
    shared.setMap(map);

    cv::Ptr<dg::DGLocalizer> localizer = cv::makePtr<dg::DGLocalizer>();
    configureLocalizer(localizer);
    localizer->setShared(&shared);
    localizer->applyIMUCompass(-1.046164, 1582092620.078608);
    localizer->applyGPS(dg::Point2(243.301921, 290.833151), 1582092620.251248);
    localizer->applyGPS(dg::Point2(247.408887, 290.504789), 1582092627.254031);
    localizer->applyIMUCompass(-0.206894, 1582092627.377480);
    localizer->applyGPS(dg::Point2(251.272502, 298.780476), 1582092634.251800);
    localizer->applyIMUCompass(-0.694228 , 1582092634.477823);
    localizer->applyGPS(dg::Point2(264.538183, 305.029270), 1582092641.261015);
    localizer->applyIMUCompass(-1.387269, 1582092641.576952);
    localizer->printInternalState();
    printf("x = %lf, y = %lf, theta = %lf\n\n", localizer->getPose().x, localizer->getPose().y, localizer->getPose().theta);

    localizer = cv::makePtr<dg::DGLocalizer>();
    configureLocalizer(localizer);
    localizer->setShared(&shared);
    localizer->applyIMUCompass(-1.046164, 1582092620.078608);
    localizer->applyGPS(dg::Point2(243.301921, 290.833151), 1582092620.251248);
    localizer->applyIMUCompass(-0.206894, 1582092627.377480);
    localizer->applyGPS(dg::Point2(251.272502, 298.780476), 1582092634.251800);
    localizer->applyIMUCompass(-0.694228, 1582092634.477823);
    localizer->applyGPS(dg::Point2(264.538183, 305.029270), 1582092641.261015);
    localizer->applyGPS(dg::Point2(247.408887, 290.504789), 1582092627.254031);
    localizer->applyIMUCompass(-1.387269, 1582092641.576952);
    localizer->printInternalState();
    printf("x = %lf, y = %lf, theta = %lf\n\n", localizer->getPose().x, localizer->getPose().y, localizer->getPose().theta);

    localizer = cv::makePtr<dg::DGLocalizer>();
    configureLocalizer(localizer);
    localizer->setShared(&shared);
    localizer->applyIMUCompass(-1.046164, 1582092620.078608);
    localizer->applyGPS(dg::Point2(243.301921, 290.833151), 1582092620.251248);
    localizer->applyIMUCompass(-0.206894, 1582092627.377480);
    localizer->applyGPS(dg::Point2(247.408887, 290.504789), 1582092627.254031);
    localizer->applyIMUCompass(-1.387269, 1582092641.576952);
    localizer->applyGPS(dg::Point2(251.272502, 298.780476), 1582092634.251800);
    localizer->applyIMUCompass(-0.694228, 1582092634.477823);
    localizer->applyGPS(dg::Point2(264.538183, 305.029270), 1582092641.261015);
    localizer->printInternalState();
    printf("x = %lf, y = %lf, theta = %lf\n\n", localizer->getPose().x, localizer->getPose().y, localizer->getPose().theta);

    return 0;
}

#endif // End of '__TEST_LOCALIZER_EKF__'
