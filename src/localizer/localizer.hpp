#ifndef __LOCALIZER__
#define __LOCALIZER__

#include "core/basic_type.hpp"
#include <vector>

namespace dg
{

class Localizer
{
public:
    virtual ~Localizer() { }

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1, double confidence = -1) = 0;

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1) = 0;

    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1) = 0;

    virtual bool applyOrientation(double theta, Timestamp time = -1, double confidence = -1) = 0;

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1) = 0;

    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1, double confidence = -1) = 0;

    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1) = 0;

    virtual bool applyLocClue(int id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1) = 0;

    virtual bool applyLocClue(const std::vector<int>& ids, const std::vector<Polar2>& obs, Timestamp time = -1, double confidence = -1) = 0;
};

class MetricLocalizer
{
public:
    virtual Pose2 getPose() const = 0;

    virtual double getPoseConfidence() const = 0;
};

class TopometricLocalizer : public MetricLocalizer
{
public:
    virtual TopometricPose getPoseTopometric() const = 0;
};

} // End of 'dg'

#endif // End of '__LOCALIZER__'

