#ifndef __EKF_LOCALIZER_VARIANTS__
#define __EKF_LOCALIZER_VARIANTS__

#include "localizer/localizer_ekf.hpp"

namespace dg
{

class EKFLocalizerZeroGyro : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            cv::Vec2d control_fake(control.at<double>(0), 0); // Add fake observation
            return EKFLocalizer::transitFunc(state, cv::Mat(control_fake), jacobian, noise);
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerHyperTan : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double w_op = 1;
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double dt = control.at<double>(0);
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2), th = w_op * tanh(w / w_op);
            cv::Mat func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                th);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, -vt * dt * s / 2,
                0, 1,  vt * c, dt * s,  vt * dt * c / 2,
                0, 0, 1, 0, dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1 - th * th);
            cv::Mat W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 1 - th * th);
            if (!W.empty()) noise = W * m_noise_motion * W.t();
            return func;
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerSinTrack : public EKFLocalizerHyperTan
{
public:
    EKFLocalizerSinTrack()
    {
        m_track_converge = 0;
    }

    virtual Pose2 getPose()
    {
        cv::AutoLock lock(m_mutex);
        return cvtTopmetric2Metric(getPoseTopometric());
    }

    virtual TopometricPose getPoseTopometric()
    {
        cv::AutoLock lock(m_mutex);
        return m_track_topo;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        if (EKFLocalizerHyperTan::applyPosition(xy, time, confidence))
        {
            cv::AutoLock lock(m_mutex);
            const double turn_weight = 1;
            const int converge_repeat = 5;
            const double drift_radius = 50;
            Pose2 pose_m = EKFLocalizerHyperTan::getPose();
            if (m_track_topo.node_id == 0 || m_track_converge < converge_repeat)
            {
                m_track_topo = findNearestTopoPose(pose_m, turn_weight);
                if (m_track_topo.node_id == m_track_prev.node_id && m_track_topo.edge_idx == m_track_prev.edge_idx && m_track_topo.dist > m_track_prev.dist) m_track_converge++;
                else m_track_converge = 0;
            }
            else
            {
                //auto node = m_map.getNode(m_track_topo.node_id);
                //CV_DbgAssert(node != nullptr);
                //auto edge = m_map.getEdge(node, m_track_topo.edge_idx);
                //CV_DbgAssert(edge != nullptr);
                //double progress = m_track_topo.dist / edge->cost;
                m_track_topo = trackTopoPose(m_track_topo, pose_m, turn_weight, m_track_topo.dist > 0);

                Pose2 pose_t = cvtTopmetric2Metric(m_track_topo);
                double dx = pose_m.x - pose_t.x, dy = pose_m.y - pose_t.y;
                if ((dx * dx + dy * dy) > drift_radius * drift_radius)
                    m_track_topo = findNearestTopoPose(pose_m, turn_weight, drift_radius, pose_m);
            }
            m_track_prev = m_track_topo;
            return true;
        }
        return false;
    }

protected:
    TopometricPose m_track_topo;

    TopometricPose m_track_prev;

    int m_track_converge;
};

} // End of 'dg'

#endif // End of '__EKF_LOCALIZER_VARIANTS__'
