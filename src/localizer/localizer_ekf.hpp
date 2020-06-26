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
        m_noise_gps_inaccurate = 10 * cv::Mat::eye(2, 2, CV_64F);
        m_noise_gps = m_noise_gps_normal;
        m_noise_loc_clue = cv::Mat::eye(4, 4, CV_64F);
        m_offset_gps = cv::Vec2d(0, 0);

        // Internal variables
        m_time_last_update = -1;
        m_time_last_delta = -1;

        initialize(cv::Mat::zeros(5, 1, CV_64F), cv::Mat::eye(5, 5, CV_64F));
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cx::Algorithm::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "threshold_time", m_threshold_time, n_read);
        CX_LOAD_PARAM_COUNT(fn, "threshold_dist", m_threshold_dist, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_motion", m_noise_motion, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_gps_normal", m_noise_gps_normal, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_gps_inaccurate", m_noise_gps_inaccurate, n_read);
        CX_LOAD_PARAM_COUNT(fn, "noise_loc_clue", m_noise_loc_clue, n_read);
        CX_LOAD_PARAM_COUNT(fn, "offset_gps", m_offset_gps, n_read);
        CX_LOAD_PARAM_COUNT(fn, "inaccurate_boxes", m_inaccurate_boxes, n_read);
        return n_read;
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
        if (inaccurate > 0) m_noise_gps_inaccurate = (cv::Mat_<double>(2, 2) << inaccurate * inaccurate, 0, 0, inaccurate * inaccurate);
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

    bool addParamGPSInaccurateBox(const dg::Point2& p1, dg::Point2& p2)
    {
        cv::AutoLock lock(m_mutex);
        m_inaccurate_boxes.push_back(cv::Rect2d(p1, p2));
        return true;
    }

    virtual Pose2 getPose() const
    {
        cv::AutoLock lock(m_mutex);
        return Pose2(m_state_vec.at<double>(0), m_state_vec.at<double>(1), m_state_vec.at<double>(2));
    }

    virtual Polar2 getVelocity() const
    {
        cv::AutoLock lock(m_mutex);
        return Polar2(m_state_vec.at<double>(3), m_state_vec.at<double>(4));
    }

    virtual LatLon getPoseGPS() const
    {
        return toLatLon(getPose());
    }

    virtual TopometricPose getPoseTopometric() const
    {
        return toMetric2Topometric(getPose());
    }

    virtual double getPoseConfidence() const
    {
        cv::AutoLock lock(m_mutex);
        double det = cv::determinant(m_state_cov.rowRange(0, 3).colRange(0, 3));
        return det;
    }

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        double dt = time_curr - time_prev;
        if (dt > DBL_EPSILON)
        {
            double dx = pose_curr.x - pose_prev.x, dy = pose_curr.y - pose_prev.y;
            double v = sqrt(dx * dx + dy * dy) / dt, w = cx::trimRad(pose_curr.theta - pose_prev.theta) / dt;
            cv::AutoLock lock(m_mutex);
            double interval = time_curr - m_time_last_update;
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
        if (m_time_last_delta > 0) dt = time - m_time_last_delta;
        m_time_last_delta = time;
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
            double interval = time_curr - m_time_last_update;
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
        // TODO: Consider pose observation
        return false;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        double interval = 0;
        if (m_time_last_update > 0) interval = time - m_time_last_update;
        if (interval > m_threshold_time) predict(interval);

        bool is_normal = true;
        for (auto box = m_inaccurate_boxes.begin(); box != m_inaccurate_boxes.end(); box++)
        {
            if (xy.x > box->x && xy.y > box->y && xy.x < box->br().x && xy.y < box->br().y)
            {
                is_normal = false;
                m_noise_gps = m_noise_gps_inaccurate;
                break;
            }
        }
        if (is_normal) m_noise_gps = m_noise_gps_normal;
        if (correct(cv::Vec2d(xy.x, xy.y)))
        {
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
        // TODO: Consider orientation observation
        return false;
    }

    virtual bool applyLocClue(ID node_id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        RoadMap::Node* node = m_map.getNode(Point2ID(node_id));
        if (node == NULL) return false;

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
        if (measure.rows == 2)
        {
            // Measurement: [ x_{GPS}, y_{GPS} ]
            const double c = cos(theta + m_offset_gps(1)), s = sin(theta + m_offset_gps(1));
            func = (cv::Mat_<double>(2, 1) <<
                x + m_offset_gps(0) * c,
                y + m_offset_gps(0) * s);
            jacobian = (cv::Mat_<double>(2, 5) <<
                1, 0, -m_offset_gps(0) * s, 0, 0,
                0, 1,  m_offset_gps(0) * c, 0, 0);
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

    cv::Mat m_noise_gps_inaccurate;

    cv::Mat m_noise_loc_clue;

    cv::Vec2d m_offset_gps;

    double m_time_last_update;

    double m_time_last_delta;

    std::vector<cv::Rect2d> m_inaccurate_boxes;

}; // End of 'EKFLocalizer'

} // End of 'dg'

#endif // End of '__EKF_LOCALIZER__'
