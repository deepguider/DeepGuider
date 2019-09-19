#ifndef __SIMPLE_LOCALIZER__
#define __SIMPLE_LOCALIZER__

#include "opencx.hpp"
#include "dg_core.hpp"

namespace dg
{

class SimpleMetricLocalizer : public Localizer, public MetricLocalizer
{
public:
    virtual Pose2 getPose() const
    {
        cv::AutoLock lock(m_mutex);
        return m_pose_metric;
    }

    virtual bool loadMap(const SimpleRoadMap& map)
    {
        cv::AutoLock lock(m_mutex);
        return map.copyTo(&m_map);
    }

    virtual bool saveMap(SimpleRoadMap& map) const
    {
        cv::AutoLock lock(m_mutex);
        return m_map.copyTo(&map);
    }

    virtual const SimpleRoadMap& getMap() const
    {
        cv::AutoLock lock(m_mutex);
        return m_map;
    }

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric = pose;
        return true;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.x = xy.x;
        m_pose_metric.y = xy.y;
        return true;
    }

    virtual bool applyOrientation(double theta, Timestamp time = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.theta = theta;
        return true;
    }

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1)
    {
        cv::AutoLock lock(m_mutex);
        double dx = pose_curr.x - pose_prev.x;
        double dy = pose_curr.y - pose_prev.y;
        double c = cos(m_pose_metric.theta - pose_prev.theta), s = sin(m_pose_metric.theta - pose_prev.theta);
        m_pose_metric.x += c * dx - s * dy;
        m_pose_metric.x += s * dx + c * dy;
        m_pose_metric.theta = cx::trimRad(m_pose_metric.theta + pose_curr.theta - pose_prev.theta);
        return true;
    }

    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.theta = cx::trimRad(m_pose_metric.theta + theta_curr - theta_prev);
        return true;
    }

    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.x += delta.lin * cos(m_pose_metric.theta + delta.ang / 2);
        m_pose_metric.y += delta.lin * sin(m_pose_metric.theta + delta.ang / 2);
        m_pose_metric.theta = cx::trimRad(m_pose_metric.theta + delta.ang);
        return true;
    }

    virtual bool applyLocClue(int node_id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1)
    {
        SimpleRoadMap::Node* node = m_map.getNode(Point2ID(node_id));
        if (node == NULL) return false;
        m_pose_metric.x = node->data.x;
        m_pose_metric.y = node->data.y;
        return true;
    }

    virtual bool applyLocClue(const std::vector<int>& ids, const std::vector<Polar2>& obs, Timestamp time = -1)
    {
        if (ids.empty() || obs.empty() || ids.size() != obs.size()) return false;
        return applyLocClue(ids.back(), obs.back(), time);
    }

    virtual bool configPose(const Pose2& offset) { return false; }

    virtual bool configPosition(const Pose2& offset) { return false; }

    virtual bool configOrientation(const Pose2& offset) { return false; }

    virtual bool configOdometry(const Pose2& offset) { return false; }

    virtual bool configLocCue(const Pose2& offset) { return false; }

protected:
    SimpleRoadMap m_map;

    Pose2 m_pose_metric;

    mutable cv::Mutex m_mutex;
};

} // End of 'dg'

#endif // End of '__SIMPLE_LOCALIZER__'
