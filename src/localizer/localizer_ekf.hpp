#ifndef __EKF_LOCALIZER__
#define __EKF_LOCALIZER__

#include "localizer/localizer_base.hpp"

namespace dg
{

class EKFLocalizer : public BaseLocalizer, public cx::EKF, public cx::Algorithm
{
public:
    EKFLocalizer()
    {
        // Parameters
        m_threshold_time = 0.01;
        m_threshold_dist = 1;
        m_noise_motion = cv::Mat::eye(2, 2, CV_64F);
        m_noise_gps_normal = cv::Mat::eye(2, 2, CV_64F);
        m_noise_gps_deadzone = 10 * cv::Mat::eye(2, 2, CV_64F);
        m_noise_gps = m_noise_gps_normal;
        m_noise_loc_clue = cv::Mat::eye(4, 4, CV_64F);
        m_gps_offset = cv::Vec2d(0, 0);
        m_gps_reverse_vel = 0;
        m_gps_init_first = true;
        m_compass_noise = (cv::Mat_<double>(1, 1) << cx::cvtDeg2Rad(1));
        m_compass_offset = 0;
        m_compass_as_gyro = false;
        m_norm_conf_a = 1;
        m_norm_conf_b = 2;

        // Internal variables
        m_time_last_update = -1;
        m_compass_prev_angle = 0;
        m_compass_prev_time = -1;

        initialize(cv::Mat::zeros(5, 1, CV_64F), cv::Mat::eye(5, 5, CV_64F));
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "threshold_time", m_threshold_time, n_read);
        CX_LOAD_PARAM_COUNT(fn, "threshold_dist", m_threshold_dist, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_motion", m_noise_motion, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_gps_normal", m_noise_gps_normal, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_gps_deadzone", m_noise_gps_deadzone, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_loc_clue", m_noise_loc_clue, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_dead_zones", m_gps_dead_zones, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_offset", m_gps_offset, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_reverse_vel", m_gps_reverse_vel, n_read);
        CX_LOAD_PARAM_COUNT(fn, "gps_init_first", m_gps_init_first, n_read);
        CX_LOAD_PARAM_COUNT(fn, "compass_noise", m_compass_noise, n_read);
        CX_LOAD_PARAM_COUNT(fn, "compass_offset", m_compass_offset, n_read);
        CX_LOAD_PARAM_COUNT(fn, "compass_as_gyro", m_compass_as_gyro, n_read);
        CX_LOAD_PARAM_COUNT(fn, "norm_conf_a", m_norm_conf_a, n_read);
        CX_LOAD_PARAM_COUNT(fn, "norm_conf_b", m_norm_conf_b, n_read);        return n_read;
    }

    bool setParamMotionNoise(double vv, double ww, double vw = 0)
    {
        cv::AutoLock lock(m_mutex);
        m_noise_motion = (cv::Mat_<double>(2, 2) << vv * vv, vw * vw, vw * vw, ww * ww);
        return true;
    }

    bool setParamGPSNoise(double normal, double inaccurate = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (normal > 0) m_noise_gps_normal = (cv::Mat_<double>(2, 2) << normal * normal, 0, 0, normal * normal);
        if (inaccurate > 0) m_noise_gps_deadzone = (cv::Mat_<double>(2, 2) << inaccurate * inaccurate, 0, 0, inaccurate * inaccurate);
        return true;
    }

    bool setParamLocClueNoise(double rho, double phi)
    {
        cv::AutoLock lock(m_mutex);
        m_noise_loc_clue = cv::Mat::zeros(4, 4, CV_64F);
        m_noise_loc_clue.at<double>(0, 0) = rho * rho;
        m_noise_loc_clue.at<double>(1, 1) = phi * phi;
        return true;
    }

    bool addParamGPSDeadZone(const dg::Point2& p1, dg::Point2& p2)
    {
        cv::AutoLock lock(m_mutex);
        m_gps_dead_zones.push_back(cv::Rect2d(p1, p2));
        return true;
    }

    virtual Pose2 getPose()
    {
        cv::AutoLock lock(m_mutex);
        return Pose2(m_state_vec.at<double>(0), m_state_vec.at<double>(1), m_state_vec.at<double>(2));
    }

    virtual Polar2 getVelocity()
    {
        cv::AutoLock lock(m_mutex);
        return Polar2(m_state_vec.at<double>(3), m_state_vec.at<double>(4));
    }

    virtual LatLon getPoseGPS()
    {
        return toLatLon(getPose());
    }

    virtual TopometricPose getPoseTopometric()
    {
        Pose2 pose_m = getPose();
        cv::AutoLock lock(m_mutex);
        std::vector<RoadMap::Node*> near_nodes = findNearNodes(pose_m, 100);
        return findNearestTopoPose(pose_m, near_nodes, 1);
    }

    virtual double getPoseConfidence()
    {
        cv::AutoLock lock(m_mutex);
        double conf = log10(cv::determinant(m_state_cov.rowRange(0, 3).colRange(0, 3)));
        if (m_norm_conf_a > 0) conf = 1 / (1 + exp(m_norm_conf_a * conf + m_norm_conf_b));
        return conf;
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
        if (m_gps_init_first)
        {
            m_state_vec.at<double>(0) = xy.x;
            m_state_vec.at<double>(1) = xy.y;
            m_time_last_update = time;
            m_gps_init_first = false;
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
                m_noise_gps = m_noise_gps_deadzone;
                break;
            }
        }
        if (is_normal) m_noise_gps = m_noise_gps_normal;
        if (correct(cv::Vec2d(xy.x, xy.y)))
        {
            if (m_gps_reverse_vel < 0 && m_state_vec.at<double>(3) < m_gps_reverse_vel)
            {
                // Fix the reversed robot
                m_state_vec.at<double>(0) += 2 * m_gps_offset(0) * cos(m_state_vec.at<double>(2) + m_gps_offset(1));
                m_state_vec.at<double>(1) += 2 * m_gps_offset(0) * sin(m_state_vec.at<double>(2) + m_gps_offset(1));
                m_state_vec.at<double>(2) += CV_PI;
                m_state_vec.at<double>(3) *= -1;
            }
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
            m_time_last_update = time;
            return true;
        }
        return false;
    }

    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1)
    {
        Point2 xy = toMetric(ll);
        return applyPosition(xy, time, confidence);
    }

    virtual bool applyOrientation(double theta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (m_compass_as_gyro)
        {
            double theta_prev = m_compass_prev_angle, time_prev = m_compass_prev_time;
            m_compass_prev_angle = theta;
            m_compass_prev_time = time;
            return applyOdometry(theta, theta_prev, time, time_prev, confidence);
        }

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

    virtual bool applyLocClue(ID node_id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        RoadMap::Node* node = m_map.getNode(Point2ID(node_id));
        if (node == nullptr) return false;

        double interval = 0;
        if (m_time_last_update > 0) interval = time - m_time_last_update;
        if (interval > m_threshold_time) predict(interval);
        // TODO: Deal with missing observation
        if (obs.lin > m_threshold_dist && obs.ang < CV_PI && correct(cv::Vec4d(obs.lin, obs.ang, node->data.x, node->data.y)))
        {
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
            m_time_last_update = time;
            return true;
        }
        return false;
    }

    virtual bool applyLocClue(const std::vector<ID>& node_ids, const std::vector<Polar2>& obs, Timestamp time = -1, const std::vector<double>& confidence = std::vector<double>())
    {
        if (node_ids.empty() || node_ids.size() != obs.size()) return false;
        if (confidence.size() == node_ids.size())
        {
            for (size_t i = 0; i < node_ids.size(); i++)
                if (!applyLocClue(node_ids[i], obs[i], time, confidence[i])) return false;
        }
        else
        {
            for (size_t i = 0; i < node_ids.size(); i++)
                if (!applyLocClue(node_ids[i], obs[i], time)) return false;
        }
        return true;
    }

protected:
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
        if (!W.empty()) noise = W * m_noise_motion * W.t();
        return func;
    }

    virtual cv::Mat observeFunc(const cv::Mat& state, const cv::Mat& measure, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
        cv::Mat func;
        if (measure.rows == 1)
        {
            // Measurement: [ theta_{compass} ]
            func = (cv::Mat_<double>(1, 1) << theta + m_compass_offset);
            jacobian = (cv::Mat_<double>(1, 5) << 0, 0, 1, 0, 0);
            noise = m_compass_noise;
        }
        else if (measure.rows == 2)
        {
            // Measurement: [ x_{GPS}, y_{GPS} ]
            const double c = cos(theta + m_gps_offset(1)), s = sin(theta + m_gps_offset(1));
            func = (cv::Mat_<double>(2, 1) <<
                x + m_gps_offset(0) * c,
                y + m_gps_offset(0) * s);
            jacobian = (cv::Mat_<double>(2, 5) <<
                1, 0, -m_gps_offset(0) * s, 0, 0,
                0, 1,  m_gps_offset(0) * c, 0, 0);
            noise = m_noise_gps;
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
            noise = m_noise_loc_clue;
        }
        return func;
    }

    double m_threshold_time;

    double m_threshold_dist;

    cv::Mat m_noise_motion;

    cv::Mat m_noise_gps;

    cv::Mat m_noise_gps_normal;

    cv::Mat m_noise_gps_deadzone;

    cv::Mat m_noise_loc_clue;

    cv::Vec2d m_gps_offset;

    double m_gps_reverse_vel;

    bool m_gps_init_first;

    bool m_compass_as_gyro;

    cv::Mat m_compass_noise;

    double m_compass_offset;

    double m_norm_conf_a;

    double m_norm_conf_b;

    double m_time_last_update;

    double m_compass_prev_angle;

    double m_compass_prev_time;

    std::vector<cv::Rect2d> m_gps_dead_zones;

}; // End of 'EKFLocalizer'

} // End of 'dg'

#endif // End of '__EKF_LOCALIZER__'
