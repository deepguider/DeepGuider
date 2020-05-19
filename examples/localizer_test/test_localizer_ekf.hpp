#ifndef __TEST_LOCALIZER_EKF__
#define __TEST_LOCALIZER_EKF__

#include "vvs.h"
#include "dg_core.hpp"

// Ref. https://sites.google.com/view/timecontrol/tutorials/extended-kalman-filtering-with-opencv
class EKFLocalizer
{
public:
    EKFLocalizer()
    {
        initPose();
    }

    bool initPose(const dg::Pose2& pose = dg::Pose2(0, 0, 0), const dg::Pose2& cov = dg::Pose2(1, 1, 1))
    {
        m_kf.init(3, 2, 2, CV_64F); // The dimension of state, measurement, and control
        m_kf.statePre  = (cv::Mat_<double>(3, 1) << pose.x, pose.y, pose.theta);
        m_kf.statePost = (cv::Mat_<double>(3, 1) << pose.x, pose.y, pose.theta);
        m_kf.errorCovPre  = (cv::Mat_<double>(3, 3) << cov.x, 0, 0, 0, cov.y, 0, 0, 0, cov.theta);
        m_kf.errorCovPost = (cv::Mat_<double>(3, 3) << cov.x, 0, 0, 0, cov.y, 0, 0, 0, cov.theta);
        m_kf.processNoiseCov = cv::Mat::eye(3, 3, CV_64F);
        m_kf.measurementNoiseCov = cv::Mat::eye(2, 2, CV_64F);
        m_kf.transitionMatrix = cv::Mat::eye(3, 3, CV_64F);
        m_kf.measurementMatrix = cv::Mat::eye(2, 3, CV_64F);
        m_noise_control = cv::Mat::eye(2, 2, CV_64F);
        return true;
    }

    dg::Pose2 getPose() const { return m_pose; }

    bool predictPose(const dg::Polar2& delta)
    {
        double c = cos(m_pose.theta + delta.ang / 2), s = sin(m_pose.theta + delta.ang / 2);

        m_kf.transitionMatrix = (cv::Mat_<double>(3, 3) <<
            1, 0, delta.lin * -s,
            0, 1, delta.lin * c,
            0, 1, 1);
        cv::Mat W = (cv::Mat_<double>(3, 2) <<
            c, -delta.lin * s,
            s,  delta.lin * c,
            0, 1);
        m_kf.processNoiseCov = W * m_noise_control * W.t();
        m_kf.predict();
        m_pose.x += delta.lin * c;
        m_pose.y += delta.lin * s;
        m_pose.theta += delta.ang;
        m_kf.statePre = (cv::Mat_<double>(3, 1) << m_pose.x, m_pose.y, m_pose.theta);
        m_kf.statePre.copyTo(m_kf.statePost);
        m_kf.errorCovPre.copyTo(m_kf.errorCovPost);
        return true;
    }

    bool correctPose(const dg::Point2& point)
    {
        cv::Mat measure = (cv::Mat_<double>(2, 1) << point.x, point.y);
        m_kf.correct(measure);
        m_kf.temp5.at<double>(0) = point.x - m_pose.x;
        m_kf.temp5.at<double>(1) = point.y - m_pose.y;
        m_kf.statePost = m_kf.statePre + m_kf.gain * m_kf.temp5;
        m_kf.statePost.copyTo(m_kf.statePre);
        m_kf.errorCovPost.copyTo(m_kf.errorCovPre);
        m_pose.x = m_kf.statePost.at<double>(0);
        m_pose.y = m_kf.statePost.at<double>(1);
        m_pose.theta = m_kf.statePost.at<double>(2);
        return true;
    }

    bool setControlNoise(const dg::Polar2& noise)
    {
        m_noise_control = (cv::Mat_<double>(2, 2) << noise.lin, noise.ang);
        return true;
    }

    bool setMesureNoise(const dg::Point2& noise)
    {
        m_kf.measurementNoiseCov.at<double>(0, 0) = noise.x * noise.x;
        m_kf.measurementNoiseCov.at<double>(1, 1) = noise.y * noise.y;
        return true;
    }

protected:
    cv::KalmanFilter m_kf;
    cv::Mat m_noise_control;
    dg::Pose2 m_pose;
};

int testLocEKF(double gps_noise = 0.3, double interval = 0.1, double velocity = 1)
{
    cv::RNG rng;
    EKFLocalizer localizer;
    VVS_CHECK_TRUE(localizer.setMesureNoise(dg::Point2(gps_noise, gps_noise)));
    dg::Polar2 delta(velocity * interval, 0);
    for (double t = 0; t < 10; t += interval)
    {
        // Predict pose with EKF
        localizer.predictPose(delta);

        // Correct pose with EKF
        dg::Point2 gps(velocity * t, 1);
        gps.x += rng.gaussian(gps_noise);
        gps.y += rng.gaussian(gps_noise);
        localizer.correctPose(gps);

        dg::Pose2 pose = localizer.getPose();
        printf("Time: %.1f, GPS: %.3f, %.3f, Pose: %.3f, %.3f, %.0f\n", t, gps.x, gps.y, pose.x, pose.y, pose.theta);
    }
    return 0;
}

#endif // End of '__TEST_LOCALIZER_EKF__'
