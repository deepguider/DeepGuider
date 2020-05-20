#ifndef __LOCALIZER__
#define __LOCALIZER__

#include "core/basic_type.hpp"
#include <vector>

namespace dg
{

/**
 * @brief Localizer interface for various input data
 *
 * This defines an interface for localizers to receive various types of input data.
 */
class Localizer
{
public:
    /**
     * The destructor
     */
    virtual ~Localizer() { }

    /**
     * Apply a metric pose observation<br>
     *  The data usually come from other pose estimators.
     * @param pose The observed pose (Unit: [m] and [rad])
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyPose(const Pose2& pose, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply a metric position observation<br>
     *  The data usually come from other position estimators.
     * @param xy The observed position (Unit: [m])
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply a geodesic position observation<br>
     *  The data usually come from GPS and other position estimators.
     * @param xy The observed geodesic position (Unit: [deg])
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an orientation observation<br>
     *  The data usually come from (magnetic or visual) compasses and AHRSs.
     * @param theta The observed orientation (Unit: [rad])
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOrientation(double theta, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an odometry observation as relative pose<br>
     *  The data usually come from (wheel or visual) odometry.
     * @param pose_curr The observed current pose (Unit: [m] and [rad])
     * @param pose_prev The previous pose (Unit: [m] and [rad])
     * @param time_curr The observed time (Unit: [sec])
     * @param time_prev The previous time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1) = 0;

    /**
     * Apply an odometry observation as linear and angular displacement<br>
     *  The data usually come from (wheel) odometry.
     * @param delta The observed displacement (Unit: [m] and [rad])<br>
     *  If delta.lin < 0, the linear displacement is invalid. If delta.ang >= CV_PI, the angular displacement is invalid.
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an odometry observation as angular displacement<br>
     *  The data usually come from gyroscopes and visual compasses.
     * @param theta_curr The observed current angle (Unit: [m] and [rad])
     * @param theta_prev The previous angle (Unit: [m] and [rad])
     * @param time_curr The observed time (Unit: [sec])
     * @param time_prev The previous time (Unit: [sec])
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1) = 0;

    /**
     * Apply a single localization clue<br>
     *  The data usually come from POI/structure/scene recognizers.
     * @param id The ID of observed clue
     * @param obs The relative distance and angle of the observed clue (Unit: [m] and [rad])<br>
     *  If obs.lin < 0, the relative distance is invalid. If obs.ang >= CV_PI, the relative angle is invalid.
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyLocClue(ID id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply multiple localization clues<br>
     *  The data usually come from POI/structure/scene recognizers.
     * @param ids The IDs of observed clues
     * @param obs The relative observation from each clue (Unit: [m] and [rad])
     * @param time The observed time (Unit: [sec])
     * @param The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyLocClue(const std::vector<ID>& ids, const std::vector<Polar2>& obs, Timestamp time = -1, const std::vector<double>& confidence = std::vector<double>()) = 0;
};

/**
 * @brief Localizer interface for metric outputs
 *
 * This defines an interface for localizers to provide metric pose and its confidence.
 */
class MetricLocalizer
{
public:
    /**
     * Get the current metric pose
     * @return The current metric pose
     */
    virtual Pose2 getPose() const = 0;

    /**
     * Get the current metric pose in geodesic notation
     * @return The current geodesic pose
     */
    virtual LatLon getPoseGPS() const = 0;

    /**
     * Get the current confidence of localization
     * @return The current pose confidence
     */
    virtual double getPoseConfidence() const = 0;
};

/**
 * @brief Localizer interface for topometric output
 *
 * This is an additional interface for localizers to provide topometric pose.
 */
class TopometricLocalizer : public MetricLocalizer
{
public:
    /**
     * Get the current topometric pose
     * @return The current topometric pose
     */
    virtual TopometricPose getPoseTopometric() const = 0;
};

} // End of 'dg'

#endif // End of '__LOCALIZER__'

