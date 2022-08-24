#ifndef __LOCALIZER_INTERFACE__
#define __LOCALIZER_INTERFACE__

#include "core/basic_type.hpp"
#include <vector>

namespace dg
{

/**
 * @brief Localizer interface for various sensor data
 *
 * This defines an interface for localizers to receive various types of sensor data.
 */
class SensorInterface
{
public:
    /**
     * The destructor
     */
    virtual ~SensorInterface() { }

    /**
     * Set error covariance of motion
     * @param sigma_linear_velocity Standard deviation of linear velocity (Unit: [m])
     * @param sigma_angular_velocity_deg Standard deviation of angular velocity (Unit: [deg])
     * @param cov_lin_ang Covariance of linear velocity and angular velocity
     */
    virtual bool setParamMotionNoise(double sigma_linear_velocity, double sigma_angular_velocity_deg, double cov_lin_ang = 0) = 0;

    /**
     * Set bounds of motion
     * @param max_linear_velocity Max. velocity of linear motion (Unit: [m/sec])
     * @param max_angular_velocity_deg Max. velocity of angular motion (Unit: [deg/sec])
     * @param min_linear_velocity Min. velocitiy of linear motion (-1: undefined)
     */
    virtual bool setParamMotionBounds(double max_linear_velocity, double max_angular_velocity_deg, double min_linear_velocity = -1) = 0;

    /**
     * Set error covariance of GPS
     * @param sigma_normal Standard deviation of gps position at normal (Unit: [m])
     * @param sigma_deadzone Standard deviation of gps position at deadzone (Unit: [deg])
     */
    virtual bool setParamGPSNoise(double sigma_normal, double sigma_deadzone = -1) = 0;

    /**
     * Set error covariance of Odometry
     * @param sigma_position Standard deviation of odometry position (Unit: [m])
     * @param sigma_theta_deg Standard deviation of odomery orientation (Unit: [deg])
     */
    virtual bool setParamOdometryNoise(double sigma_position, double sigma_theta_deg) = 0;

    /**
     * Set error variance of IMU theta
     * @param sigma_theta_deg Standard deviation of IMU theta error (Unit: [deg])
     * @param theta_offset Angular offset of IMU theta (Unit: [deg])
     */
    virtual bool setParamIMUCompassNoise(double sigma_theta_deg, double theta_offset = 0) = 0;

    /**
     * Set error covariance of POI observation
     * @param sigma_position Standard deviation of position error of POI localization (Unit: [m])
     * @param sigma_theta_deg Standard deviation of orientation error of POI localization (Unit: [deg])
     */
    virtual bool setParamPOINoise(double sigma_position, double sigma_theta_deg) = 0;

    /**
     * Set error covariance of VPS observation
     * @param sigma_position Standard deviation of position error of VPS localization (Unit: [m])
     * @param sigma_theta_deg Standard deviation of orientation error of VPS localization (Unit: [deg])
     */
    virtual bool setParamVPSNoise(double sigma_position, double sigma_theta_deg) = 0;

    /**
     * Set error covariance of POI observation
     * @param sigma_rel_dist Standard deviation of estimated relative distance (Unit: [m])
     * @param sigma_rel_theta_deg Standard deviation of estimated relative orientation (Unit: [deg])
     * @param sigma_position Standard deviation of position error of POI (Unit: [m])
     */
    virtual bool setParamPOINoiseRelative(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position) = 0;

    /**
     * Set error covariance of VPS observation
     * @param sigma_rel_dist Standard deviation of estimated relative distance (Unit: [m])
     * @param sigma_rel_theta_deg Standard deviation of estimated relative orientation (Unit: [deg])
     * @param sigma_position Standard deviation of position error of StreetView (Unit: [m])
     */
    virtual bool setParamVPSNoiseRelative(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position) = 0;

    /**
     * Set error covariance of Intersection-based localization
     * @param sigma_position Standard deviation of robot position estimated from Intersection observation (Unit: [m])
     */
    virtual bool setParamIntersectClsNoise(double sigma_position) = 0;

    /**
     * Set error variance of RoadTheta
     * @param sigma_theta_deg Standard deviation of RoadTheta error (Unit: [deg])
     */
    virtual bool setParamRoadThetaNoise(double sigma_theta_deg) = 0;

    /**
     * Set GPS offset
     * @param lin_offset Linear displacement from the robot origin (Unit: [m])
     * @param ang_offset_deg Angular displacement from the robot heading. CCW is positive. (Unit: [deg])
     */
    virtual bool setParamGPSOffset(double lin_offset, double ang_offset_deg = 0) = 0;

    /**
     * Set camera offset<br>
     * This setting applies for vision-based recognizers (POI, VPS, and IntersectCls).
     * @param lin_offset Linear displacement from the robot origin (Unit: [m])
     * @param ang_offset_deg Angular displacement from the robot heading. CCW is positive. (Unit: [deg])
     */
    virtual bool setParamCameraOffset(double lin_offset, double ang_offset_deg = 0) = 0;

    /**
     * Apply a geodesic position observation<br>
     *  The data usually come from GPS and other position estimators.
     * @param ll The observed geodesic position (Unit: [deg])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply a geodesic position observation<br>
     *  The data usually come from GPS and other position estimators.
     * @param xy The UTM position of an observed GPS data (Unit: [m])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyGPS(const Point2& xy, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an relative pose observation from Odometry<br>
     *  The data usually come from wheel encoder based position estimators.
     * @param odometry_pose The observed odomery pose (Unit: [m, m, rad])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOdometry(Pose2 odometry_pose, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an relative orientation observation from IMU<br>
     *  The data usually come from (magnetic or visual) compasses and AHRSs.
     * @param odometry_theta The observed odometry orientation (Unit: [rad])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyIMUCompass(double odometry_theta, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an absolute orientation observation from RoadTheta localizer<br>
     * @param theta The observed absolute orientation (Unit: [rad])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyRoadTheta(double theta, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply relative position observation from POI localizer
     * @param clue_xy The coordinate of observed clue
     * @param relative The relative distance and angle of the observed clue (Unit: [m] and [rad])<br>
     *  If relative.lin < 0, the relative distance is invalid. If relative.ang >= CV_PI, the relative angle is invalid.
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @param use_relative_model Use relative observation model in EKF if True
     * @return True if successful (false if failed)
     */
    virtual bool applyPOI(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false) = 0;

    /**
     * Apply relative position observation from VPS localizer
     * @param clue_xy The coordinate of observed clue
     * @param relative The relative distance and angle of the observed clue (Unit: [m] and [rad])<br>
     *  If relative.lin < 0, the relative distance is invalid. If relative.ang >= CV_PI, the relative angle is invalid.
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @param use_relative_model Use relative observation model in EKF if True
     * @return True if successful (false if failed)
     */
    virtual bool applyVPS(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false) = 0;

    /**
     * Apply position observation from Intersection-based localizer
     * @param xy The coordinate of robot position estimated from intsection observation
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyIntersectCls(const Point2& xy, Timestamp time = -1, double confidence = -1) = 0;
};


/**
 * @brief LocalizerInterface interface for various input data
 *
 * This defines an interface for localizers to receive various types of input data.
 */
class LocalizerInterface
{
public:
    /**
     * The destructor
     */
    virtual ~LocalizerInterface() { }

    /**
     * Apply a metric pose observation<br>
     *  The data usually come from other pose estimators.
     * @param pose The observed pose (Unit: [m] and [rad])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyPose(const Pose2& pose, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply a metric position observation<br>
     *  The data usually come from other position estimators.
     * @param xy The observed position (Unit: [m])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply an orientation observation<br>
     *  The data usually come from (magnetic or visual) compasses and AHRSs.
     * @param theta The observed orientation (Unit: [rad])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
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
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1) = 0;

    /**
     * Apply an odometry observation as linear and angular displacement<br>
     *  The data usually come from (wheel) odometry.
     * @param delta The observed displacement (Unit: [m] and [rad])<br>
     *  If delta.lin < 0, the linear displacement is invalid. If delta.ang >= CV_PI, the angular displacement is invalid.
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
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
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1) = 0;

    /**
     * Apply a single localization clue<br>
     *  The data usually come from POI/structure/scene recognizers.
     * @param clue_xy The coordinate of observed clue
     * @param relative The relative distance and angle of the observed clue (Unit: [m] and [rad])<br>
     *  If relative.lin < 0, the relative distance is invalid. If relative.ang >= CV_PI, the relative angle is invalid.
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyLocClue(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1) = 0;

    /**
     * Apply multiple localization clues<br>
     *  The data usually come from POI/structure/scene recognizers.
     * @param clue_xy The coordinates of observed clues
     * @param relative The relative observation from each clue (Unit: [m] and [rad])
     * @param time The observed time (Unit: [sec])
     * @param confidence The observation confidence
     * @return True if successful (false if failed)
     */
    virtual bool applyLocClue(const std::vector<Point2>& clue_xy, const std::vector<Polar2>& relative, Timestamp time = -1, const std::vector<double>& confidence = std::vector<double>()) = 0;
};

/**
 * @brief Localizer interface for metric outputs
 *
 * This defines an interface for localizers to provide metric pose and its confidence.
 */
class MetricInterface
{
public:
    /**
     * Get the current metric pose
     * @param[out] The timestamp of the returned pose
     * @return The current metric pose
     */
    virtual Pose2 getPose(Timestamp* timestamp = nullptr) const = 0;

    /**
     * Get the current metric pose in geodesic notation
     * @param[out] The timestamp of the returned pose
     * @return The current geodesic pose
     */
    virtual LatLon getPoseGPS(Timestamp* timestamp = nullptr) const = 0;

    /**
     * Get the current confidence of localization
     * @param[out] The timestamp of the returned pose confidence
     * @return The current pose confidence [0 ~ 1]
     */
    virtual double getPoseConfidence(Timestamp* timestamp = nullptr) const = 0;
};

/**
 * @brief Localizer interface for topometric output
 *
 * This is an additional interface for localizers to provide topometric pose.
 */
class TopometricInterface : public MetricInterface
{
public:
    /**
     * Get the current topometric pose
     * @param[out] The timestamp of the returned pose
     * @return The current topometric pose
     */
    virtual TopometricPose getPoseTopometric(Timestamp* timestamp = nullptr) const = 0;
};


} // End of 'dg'

#endif // End of '__LOCALIZER_INTERFACE__'

