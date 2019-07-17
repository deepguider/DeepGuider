#ifndef __LOCALIZER__
#define __LOCALIZER__

#include "simple_road_map.hpp"
#include <vector>

namespace dg
{

class Localizer
{
public:
    virtual ~Localizer() { }

    virtual bool loadMap(const SimpleRoadMap& map) = 0;

    virtual bool saveMap(SimpleRoadMap& map) const = 0;

    virtual const SimpleRoadMap& getMap() const = 0;

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1) = 0;

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1) = 0;

    virtual bool applyOrientation(double theta, Timestamp time = -1) = 0;

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1) = 0;

    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1) = 0;

    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1) = 0;

    virtual bool applyLocCue(int id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1) = 0;

    virtual bool applyLocCue(const std::vector<int>& ids, const std::vector<Polar2>& obs, Timestamp time = -1) = 0;

    virtual bool configPose(const Pose2& offset) = 0;

    virtual bool configPosition(const Pose2& offset) = 0;

    virtual bool configOrientation(const Pose2& offset) = 0;

    virtual bool configOdometry(const Pose2& offset) = 0;

    virtual bool configLocCue(const Pose2& offset) = 0;
};

class MetricLocalizer
{
public:
    virtual Pose2 getPose() const = 0;
};

class TopometricPose
{
public:
    TopometricPose(int _node_id = -1, int _edge_idx = -1, double _dist = 0) : node_id(_node_id), edge_idx(_edge_idx), dist(_dist) { }

    int node_id;

    int edge_idx;

    double dist;
};

class TopometricLocalizer
{
public:
    virtual TopometricPose getPoseTopometric() const = 0;
};

} // End of 'dg'

#endif // End of '__LOCALIZER__'
