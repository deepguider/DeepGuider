#ifndef __SIMPLE_LOCALIZER__
#define __SIMPLE_LOCALIZER__

#include "localizer/localizer_base.hpp"

namespace dg
{

class SimpleLocalizer : public LocalizerInterface, public UTMConverter
{
public:
    virtual Pose2 getPose()
    {
        cv::AutoLock lock(m_mutex);
        return m_pose;
    }

    virtual double getPoseConfidence()
    {
        return 0;
    }

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        const double dx = pose_curr.x - pose_prev.x;
        const double dy = pose_curr.y - pose_prev.y;
        const double c = cos(m_pose.theta - pose_prev.theta), s = sin(m_pose.theta - pose_prev.theta);
        cv::AutoLock lock(m_mutex);
        m_pose.x += c * dx - s * dy;
        m_pose.x += s * dx + c * dy;
        m_pose.theta = cx::trimRad(m_pose.theta + pose_curr.theta - pose_prev.theta);
        return true;
    }

    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose.x += delta.lin * cos(m_pose.theta + delta.ang / 2);
        m_pose.y += delta.lin * sin(m_pose.theta + delta.ang / 2);
        m_pose.theta = cx::trimRad(m_pose.theta + delta.ang);
        return true;
    }

    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose.theta = cx::trimRad(m_pose.theta + theta_curr - theta_prev);
        return true;
    }

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose = pose;
        return true;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose.x = xy.x;
        m_pose.y = xy.y;
        return true;
    }

    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1)
    {
        Point2 xy = toMetric(ll);
        return applyPosition(xy, time, confidence);
    }

    virtual bool applyOrientation(double theta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose.theta = theta;
        return true;
    }

    virtual bool applyLocClue(const Point2& clue_xy, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose = clue_xy;
        return true;
    }

    virtual bool applyLocClue(const std::vector<Point2>& clue_xy, const std::vector<Polar2>& obs, Timestamp time = -1, const std::vector<double>& confidence = std::vector<double>())
    {
        if (clue_xy.empty() || obs.empty() || clue_xy.size() != obs.size()) return false;
        return applyLocClue(clue_xy.back(), obs.back(), time);
    }

protected:
    cv::Mutex m_mutex;
    Pose2 m_pose;

}; // End of 'SimpleLocalizer'

} // End of 'dg'

#endif // End of '__SIMPLE_LOCALIZER__'
