#ifndef __LOCALIZER__
#define __LOCALIZER__

#include "simple_road_map.hpp"
#include <vector>

namespace dg
{

class Localizer
{
public:
    virtual bool loadMap(const SimpleRoadMap& map) = 0;

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1) = 0;

    virtual bool applyPose(const Point2& xy, Timestamp time = -1) = 0;

    virtual bool applyPose(double theta, Timestamp time = -1) = 0;

    virtual bool applyOdom(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1) = 0;

    virtual bool applyOdom(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1) = 0;

    virtual bool applyOdom(const Polar2& delta, Timestamp time = -1) = 0;

    virtual bool applyLocCue(int id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1) = 0;

    virtual bool applyLocCue(const std::vector<int>& ids, const std::vector<Polar2>& obs, Timestamp time = -1) = 0;
};

} // End of 'dg'

#endif // End of '__LOCALIZER__'

