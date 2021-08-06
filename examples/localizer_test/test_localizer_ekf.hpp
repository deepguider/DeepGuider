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

#endif // End of '__TEST_LOCALIZER_EKF__'
