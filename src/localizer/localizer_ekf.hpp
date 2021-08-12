#ifndef __EKF_LOCALIZER__
#define __EKF_LOCALIZER__

#include "localizer/localizer_base.hpp"

namespace dg
{

class EKFLocalizer : public BaseLocalizer, public LocalizerInterface, public cx::EKF
{
protected:
    // configuable parameters
    cv::Mat m_motion_noise;
    cv::Mat m_gps_noise_normal;
    cv::Mat m_gps_noise_deadzone;
    cv::Mat m_gps_noise;
    std::vector<cv::Rect2d> m_gps_dead_zones;
    Polar2 m_gps_offset;
    double m_gps_reverse_vel;
    cv::Mat m_imu_compass_noise;
    double m_imu_compass_offset;
    double m_imu_compass_prev_angle;
    double m_imu_compass_prev_time;
    cv::Mat m_roadtheta_noise;
    double m_roadtheta_offset;
    Polar2 m_camera_offset;
    cv::Mat m_poi_noise;
    cv::Mat m_vps_noise;
    cv::Mat m_intersectcls_noise;
    cv::Mat m_observation_noise;
    double m_threshold_time;
    double m_threshold_clue_dist;
    double m_norm_conf_a;
    double m_norm_conf_b;

    // Internal variables
    bool m_pose_initialized;
    double m_time_last_update;

public:
    EKFLocalizer()
    {
        // Parameters
        m_motion_noise = cv::Mat::eye(2, 2, CV_64F);
        m_gps_noise_normal = cv::Mat::eye(2, 2, CV_64F);
        m_gps_noise_deadzone = 10 * cv::Mat::eye(2, 2, CV_64F);
        m_gps_noise = m_gps_noise_normal;
        m_gps_offset = Polar2(0, 0);
        m_gps_reverse_vel = 0;
        m_imu_compass_noise = (cv::Mat_<double>(1, 1) << cx::cvtDeg2Rad(1));
        m_imu_compass_offset = 0;
        m_imu_compass_prev_angle = 0;
        m_imu_compass_prev_time = -1;
        m_roadtheta_noise = (cv::Mat_<double>(1, 1) << cx::cvtDeg2Rad(1));
        m_roadtheta_offset = 0;
        m_poi_noise = cv::Mat::eye(4, 4, CV_64F);
        m_vps_noise = cv::Mat::eye(4, 4, CV_64F);
        m_intersectcls_noise = cv::Mat::eye(2, 2, CV_64F);
        m_camera_offset = Polar2(0, 0);

        m_threshold_time = 0.01;
        m_threshold_clue_dist = 0;
        m_norm_conf_a = 1;
        m_norm_conf_b = 2;

        // Internal variables
        m_pose_initialized = false;
        m_time_last_update = -1;

        initialize(cv::Mat::zeros(5, 1, CV_64F), cv::Mat::eye(5, 5, CV_64F));
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "motion_noise", m_motion_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_noise_normal", m_gps_noise_normal, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_noise_deadzone", m_gps_noise_deadzone, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_noise", m_gps_noise, n_read);
        //CX_LOAD_PARAM_COUNT(fn, "gps_offset", m_gps_offset, n_read); // cv::read() doesn't support Polar2
        CX_LOAD_PARAM_COUNT(fn, "gps_reverse_vel", m_gps_reverse_vel, n_read);
        CX_LOAD_PARAM_COUNT(fn, "imu_compass_noise", m_imu_compass_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "imu_compass_offset", m_imu_compass_offset, n_read);
        CX_LOAD_PARAM_COUNT(fn, "imu_compass_prev_angle", m_imu_compass_prev_angle, n_read);
        CX_LOAD_PARAM_COUNT(fn, "imu_compass_prev_time", m_imu_compass_prev_time, n_read);
        CX_LOAD_PARAM_COUNT(fn, "roadtheta_noise", m_roadtheta_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "roadtheta_offset", m_roadtheta_offset, n_read);
        //CX_LOAD_PARAM_COUNT(fn, "camera_offset", m_camera_offset, n_read); // cv::read() doesn't support Polar2
        CX_LOAD_PARAM_COUNT(fn, "poi_noise", m_poi_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "vps_noise", m_vps_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "intersectcls_noise", m_intersectcls_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "threshold_time", m_threshold_time, n_read);
        CX_LOAD_PARAM_COUNT(fn, "threshold_clue_dist", m_threshold_clue_dist, n_read);
        CX_LOAD_PARAM_COUNT(fn, "norm_conf_a", m_norm_conf_a, n_read);
        CX_LOAD_PARAM_COUNT(fn, "norm_conf_b", m_norm_conf_b, n_read);
        return n_read;
    }

    bool resetEKFState(const cv::InputArray state, const cv::InputArray covariance, Timestamp time)
    {
        if (!setState(state)) return false;
        if (!setStateCov(covariance)) return false;
        m_time_last_update = time;
        m_imu_compass_prev_angle = 0;
        m_imu_compass_prev_time = -1;
        return true;
    }

    virtual bool setParamMotionNoise(double sigma_linear_velocity, double sigma_angular_velocity_deg, double cov_lin_ang = 0)
    {
        cv::AutoLock lock(m_mutex);
        double v = sigma_linear_velocity;
        double w = cx::cvtDeg2Rad(sigma_angular_velocity_deg);
        m_motion_noise = (cv::Mat_<double>(2, 2) << v * v, cov_lin_ang, cov_lin_ang, w * w);
        return true;
    }

    virtual bool setParamGPSNoise(double sigma_normal, double sigma_deadzone = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (sigma_normal > 0) m_gps_noise_normal = (cv::Mat_<double>(2, 2) << sigma_normal * sigma_normal, 0, 0, sigma_normal * sigma_normal);
        if (sigma_deadzone > 0) m_gps_noise_deadzone = (cv::Mat_<double>(2, 2) << sigma_deadzone * sigma_deadzone, 0, 0, sigma_deadzone * sigma_deadzone);
        m_gps_noise = m_gps_noise_normal;
        return true;
    }

    virtual bool setParamGPSOffset(double lin_offset, double ang_offset_deg = 0)
    {
        cv::AutoLock lock(m_mutex);
        m_gps_offset = Polar2(lin_offset, cx::cvtDeg2Rad(ang_offset_deg));
        return true;
    }

    virtual bool setParamIMUCompassNoise(double sigma_theta_deg, double offset = 0)
    {
        cv::AutoLock lock(m_mutex);
        double sigma_rad = cx::cvtDeg2Rad(sigma_theta_deg);
        m_imu_compass_noise = (cv::Mat_<double>(1, 1) << (sigma_rad * sigma_rad));
        m_imu_compass_offset = offset;
        return true;
    }

    virtual bool setParamRoadThetaNoise(double sigma_theta_deg, double offset = 0)
    {
        cv::AutoLock lock(m_mutex);
        double sigma_rad = cx::cvtDeg2Rad(sigma_theta_deg);
        m_roadtheta_noise = (cv::Mat_<double>(1, 1) << (sigma_rad * sigma_rad));
        m_roadtheta_offset = 0;
        return true;
    }

    virtual bool setParamCameraOffset(double lin_offset, double ang_offset_deg = 0)
    {
        cv::AutoLock lock(m_mutex);
        m_camera_offset = Polar2(lin_offset, cx::cvtDeg2Rad(ang_offset_deg));
        return true;
    }

    virtual bool setParamPOINoise(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position = 1)
    {
        cv::AutoLock lock(m_mutex);
        m_poi_noise = cv::Mat::zeros(4, 4, CV_64F);
        m_poi_noise.at<double>(0, 0) = sigma_rel_dist * sigma_rel_dist;
        m_poi_noise.at<double>(1, 1) = cx::cvtDeg2Rad(sigma_rel_theta_deg) * cx::cvtDeg2Rad(sigma_rel_theta_deg);
        m_poi_noise.at<double>(2, 2) = sigma_position * sigma_position;
        m_poi_noise.at<double>(3, 3) = sigma_position * sigma_position;
        return true;
    }

    virtual bool setParamVPSNoise(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position = 1)
    {
        cv::AutoLock lock(m_mutex);
        m_vps_noise = cv::Mat::zeros(4, 4, CV_64F);
        m_vps_noise.at<double>(0, 0) = sigma_rel_dist * sigma_rel_dist;
        m_vps_noise.at<double>(1, 1) = cx::cvtDeg2Rad(sigma_rel_theta_deg) * cx::cvtDeg2Rad(sigma_rel_theta_deg);
        m_vps_noise.at<double>(2, 2) = sigma_position * sigma_position;
        m_vps_noise.at<double>(3, 3) = sigma_position * sigma_position;
        return true;
    }

    virtual bool setParamIntersectClsNoise(double sigma_position)
    {
        cv::AutoLock lock(m_mutex);
        m_intersectcls_noise = (cv::Mat_<double>(2, 2) << sigma_position * sigma_position, 0, 0, sigma_position * sigma_position);
        return true;
    }

    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1)
    {
        Point2 xy = toMetric(ll);
        m_observation_noise = m_gps_noise;
        return applyPosition(xy, time, confidence);
    }

    virtual bool applyGPS(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        m_observation_noise = m_gps_noise;
        return applyPosition(xy, time, confidence);
    }

    virtual bool applyIMUCompass(double odometry_theta, Timestamp time = -1, double confidence = -1)
    {
        double odometry_theta_prev = m_imu_compass_prev_angle, time_prev = m_imu_compass_prev_time;
        m_imu_compass_prev_angle = odometry_theta;
        m_imu_compass_prev_time = time;
        if (time_prev < 0) return false;
        return applyOdometry(odometry_theta, odometry_theta_prev, time, time_prev, confidence);
    }

    virtual bool applyRoadTheta(double theta, Timestamp time = -1, double confidence = -1)
    {
        m_observation_noise = m_roadtheta_noise;
        return applyOrientation(theta, time, confidence);
    }

    virtual bool applyPOI(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        m_observation_noise = m_poi_noise;
        return applyLocClue(clue_xy, relative, time, confidence);
    }

    virtual bool applyVPS(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        m_observation_noise = m_vps_noise;
        return applyLocClue(clue_xy, relative, time, confidence);
    }

    virtual bool applyIntersectCls(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        m_observation_noise = m_intersectcls_noise;
        return applyPosition(xy, time, confidence);
    }

    bool addParamGPSDeadZone(const dg::Point2& p1, dg::Point2& p2)
    {
        cv::AutoLock lock(m_mutex);
        m_gps_dead_zones.push_back(cv::Rect2d(p1, p2));
        return true;
    }

    virtual Pose2 getPose(Timestamp* timestamp = nullptr) const
    {
        cv::AutoLock lock(m_mutex);
        if (timestamp) *timestamp = m_time_last_update;
        return Pose2(m_state_vec.at<double>(0), m_state_vec.at<double>(1), m_state_vec.at<double>(2));
    }

    virtual Polar2 getVelocity(Timestamp* timestamp = nullptr) const
    {
        cv::AutoLock lock(m_mutex);
        if (timestamp) *timestamp = m_time_last_update;
        return Polar2(m_state_vec.at<double>(3), m_state_vec.at<double>(4));
    }

    virtual double getPoseConfidence(Timestamp* timestamp = nullptr) const
    {
        cv::AutoLock lock(m_mutex);
        if (timestamp) *timestamp = m_time_last_update;
        double conf = log10(cv::determinant(m_state_cov.rowRange(0, 3).colRange(0, 3)));
        if (m_norm_conf_a > 0) conf = 1 / (1 + exp(m_norm_conf_a * conf + m_norm_conf_b));
        return conf;
    }

    Timestamp getLastUpdateTime() const
    {
        return m_time_last_update;
    }

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        double dt = time_curr - time_prev;
        if (dt > DBL_EPSILON)
        {
            double dx = pose_curr.x - pose_prev.x, dy = pose_curr.y - pose_prev.y;
            double v = sqrt(dx * dx + dy * dy) / dt, w = cx::trimRad(pose_curr.theta - pose_prev.theta) / dt;
            cv::AutoLock lock(m_mutex);
            double interval = 0;
            if (m_time_last_update > 0) interval = time_curr - m_time_last_update;
            if (interval > DBL_EPSILON && predict(cv::Vec3d(interval, v, w)))
            {
                m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
                m_time_last_update = time_curr;
                return true;
            }
        }
        return false;
    }

    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1, double confidence = -1)
    {
        double dt = 0;
        cv::AutoLock lock(m_mutex);
        if (m_time_last_update > 0) dt = time - m_time_last_update;
        if (dt > DBL_EPSILON)
        {
            double interval = time - m_time_last_update;
            if (interval > DBL_EPSILON && predict(cv::Vec3d(interval, delta.lin / dt, delta.ang / dt)))
            {
                m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
                m_time_last_update = time;
                return true;
            }
        }
        return false;
    }

    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        double dt = time_curr - time_prev;
        if (dt > DBL_EPSILON)
        {
            double w = cx::trimRad(theta_curr - theta_prev) / dt;
            cv::AutoLock lock(m_mutex);
            double interval = 0;
            if (m_time_last_update > 0) interval = time_curr - m_time_last_update;
            if (interval > DBL_EPSILON && predict(cv::Vec2d(interval, w)))
            {
                m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
                m_time_last_update = time_curr;
                return true;
            }
        }
        return false;
    }

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        // TODO: Consider pose observation if available
        return false;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (!m_pose_initialized)
        {
            m_state_vec.at<double>(0) = xy.x;
            m_state_vec.at<double>(1) = xy.y;
            m_time_last_update = time;
            m_pose_initialized = true;
            return true;
        }

        double interval = 0;
        if (m_time_last_update > 0) interval = time - m_time_last_update;
        if (interval > m_threshold_time)
        {
            predict(interval);
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
        }

        bool is_normal = true;
        for (auto zone = m_gps_dead_zones.begin(); zone != m_gps_dead_zones.end(); zone++)
        {
            if (xy.x > zone->x && xy.y > zone->y && xy.x < zone->br().x && xy.y < zone->br().y)
            {
                is_normal = false;
                m_gps_noise = m_gps_noise_deadzone;
                break;
            }
        }
        if (is_normal) m_gps_noise = m_gps_noise_normal;
        if (correct(cv::Vec2d(xy.x, xy.y)))
        {
            if (m_gps_reverse_vel < 0 && m_state_vec.at<double>(3) < m_gps_reverse_vel)
            {
                // Fix the reversed robot
                m_state_vec.at<double>(0) += 2 * m_gps_offset.lin * cos(m_state_vec.at<double>(2) + m_gps_offset.ang);
                m_state_vec.at<double>(1) += 2 * m_gps_offset.lin * sin(m_state_vec.at<double>(2) + m_gps_offset.ang);
                m_state_vec.at<double>(2) += CV_PI;
                m_state_vec.at<double>(3) *= -1;
            }
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
            m_time_last_update = time;
            return true;
        }
        return false;
    }

    virtual bool applyOrientation(double theta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        double interval = 0;
        if (m_time_last_update > 0) interval = time - m_time_last_update;
        if (interval > m_threshold_time)
        {
            predict(interval);
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
        }

        theta = m_state_vec.at<double>(2) + cx::trimRad(theta - cx::trimRad(m_state_vec.at<double>(2)));
        if (correct(theta))
        {
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
            m_time_last_update = time;
            return true;
        }
        return false;
    }

    virtual bool applyLocClue(const Point2& clue_xy, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        double interval = 0;
        if (m_time_last_update > 0) interval = time - m_time_last_update;
        if (interval > m_threshold_time) predict(interval);
        // TODO: Deal with missing observation
        if (obs.lin >= m_threshold_clue_dist && obs.ang < CV_PI && correct(cv::Vec4d(obs.lin, obs.ang, clue_xy.x, clue_xy.y)))
        {
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
            m_time_last_update = time;
            return true;
        }
        return false;
    }

    virtual bool applyLocClue(const std::vector<Point2>& clue_xy, const std::vector<Polar2>& obs, Timestamp time = -1, const std::vector<double>& confidence = std::vector<double>())
    {
        if (clue_xy.empty() || clue_xy.size() != obs.size()) return false;
        if (confidence.size() == clue_xy.size())
        {
            for (size_t i = 0; i < clue_xy.size(); i++)
                if (!applyLocClue(clue_xy[i], obs[i], time, confidence[i])) return false;
        }
        else
        {
            for (size_t i = 0; i < clue_xy.size(); i++)
                if (!applyLocClue(clue_xy[i], obs[i], time)) return false;
        }
        return true;
    }

protected:
    virtual cv::Mat getMotionNoise()
    {
        return m_motion_noise;
    }

    virtual cv::Mat getObservationNoise()
    {
        return m_observation_noise;
    }

    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double dt = control.at<double>(0);
        const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
        cv::Mat func, W;
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                w);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, -vt * dt * s / 2,
                0, 1,  vt * c, dt * s,  vt * dt * c / 2,
                0, 0,       1,      0,               dt,
                0, 0,       0,      1,                0,
                0, 0,       0,      0,                1);
            W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 1);
        }
        else if (control.rows == 2)
        {
            // The control input: [ dt, w_c ]
            const double v = state.at<double>(3), w = control.at<double>(1);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                w);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, 0,
                0, 1,  vt * c, dt * s, 0,
                0, 0,       1,      0, 0,
                0, 0,       0,      1, 0,
                0, 0,       0,      0, 0);
            W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 1);
        }
        else if (control.rows >= 3)
        {
            // The control input: [ dt, v_c, w_c ]
            const double v = control.at<double>(1), w = control.at<double>(2);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                w);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, 0, 0,
                0, 1,  vt * c, 0, 0,
                0, 0,       1, 0, 0,
                0, 0,       0, 0, 0,
                0, 0,       0, 0, 0);
            W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 1);
        }
        if (!W.empty()) noise = W * getMotionNoise() * W.t();
        return func;
    }

    virtual cv::Mat observeFunc(const cv::Mat& state, const cv::Mat& measure, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
        cv::Mat func;
        if (measure.rows == 1)
        {
            // Measurement: [ theta_{compass} ]
            func = (cv::Mat_<double>(1, 1) << theta + m_imu_compass_offset);
            jacobian = (cv::Mat_<double>(1, 5) << 0, 0, 1, 0, 0);
        }
        else if (measure.rows == 2)
        {
            // Measurement: [ x_{GPS}, y_{GPS} ]
            const double c = cos(theta + m_gps_offset.ang), s = sin(theta + m_gps_offset.ang);
            func = (cv::Mat_<double>(2, 1) <<
                x + m_gps_offset.lin * c,
                y + m_gps_offset.lin * s);
            jacobian = (cv::Mat_<double>(2, 5) <<
                1, 0, -m_gps_offset.lin * s, 0, 0,
                0, 1,  m_gps_offset.lin * c, 0, 0);
        }
        else if (measure.rows >= 4)
        {
            // Measurement: [ rho_{id}, phi_{id}, x_{id}, y_{id} ]
            const double dx = measure.at<double>(2) - x;
            const double dy = measure.at<double>(3) - y;
            const double r = sqrt(dx * dx + dy * dy);
            func = (cv::Mat_<double>(4, 1) <<
                r,
                cx::trimRad(atan2(dy, dx) - theta),
                measure.at<double>(2),
                measure.at<double>(3));
            jacobian = (cv::Mat_<double>(4, 5) <<
               -2 * dx / r, -2 * dy / r,  0, 0, 0,
                dy / r / r, -dx / r / r, -1, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0);
        }
        noise = getObservationNoise();
        return func;
    }

}; // End of 'EKFLocalizer'

} // End of 'dg'

#endif // End of '__EKF_LOCALIZER__'
