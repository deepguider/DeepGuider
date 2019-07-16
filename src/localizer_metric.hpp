#ifndef __METRIC_LOCALIZER__
#define __METRIC_LOCALIZER__

#include "localizer.hpp"

namespace dg
{

class MetricLocalizer : public Localizer
{
public:
    Pose2 getPose() const { return m_pose_metric; }

    virtual bool loadMap(const SimpleRoadMap& map) { return false; }

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1) { return false; }

    virtual bool applyPose(const Point2& xy, Timestamp time = -1) { return false; }

    virtual bool applyPose(double theta, Timestamp time = -1) { return false; }

    virtual bool applyOdom(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1) { return false; }

    virtual bool applyOdom(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1) { return false; }

    virtual bool applyOdom(const Polar2& delta, Timestamp time = -1) { return false; }

    virtual bool applyLocCue(int node_id, const Polar2& obs, Timestamp time = -1) { return false; }

    virtual bool applyLocCue(const std::vector<int>& ids, const std::vector<Polar2>& obs, Timestamp time = -1) { return false; }

protected:
    Pose2 m_pose_metric;
};

} // End of 'dg'

#endif // End of '__METRIC_LOCALIZER__'
