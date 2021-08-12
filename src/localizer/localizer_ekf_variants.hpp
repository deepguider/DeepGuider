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
public:
    EKFLocalizerHyperTan() : m_max_ang_vel(1) { }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = EKFLocalizer::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "max_ang_vel", m_max_ang_vel, n_read);
        return n_read;
    }

protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double dt = control.at<double>(0);
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2), th = m_max_ang_vel * tanh(w / m_max_ang_vel);
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
            if (!W.empty()) noise = W * m_motion_noise * W.t();
            return func;
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }

    double m_max_ang_vel;
};

class EKFLocalizerSinTrack : public EKFLocalizerHyperTan
{
public:
    EKFLocalizerSinTrack()
    {
        m_search_radius = 50;
        m_search_turn_weight = 100;
        m_track_converge_threshold = 10;
        m_track_transit_dist = 10;
        m_track_drift_radius = 100;
        m_track_near_radius = 20;

        m_track_converge_count = 0;
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = EKFLocalizerHyperTan::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "search_radius", m_search_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "search_turn_weight", m_search_turn_weight, n_read);
        CX_LOAD_PARAM_COUNT(fn, "track_converge_threshold", m_track_converge_threshold, n_read);
        CX_LOAD_PARAM_COUNT(fn, "track_transit_dist", m_track_transit_dist, n_read);
        CX_LOAD_PARAM_COUNT(fn, "track_drift_radius", m_track_drift_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "track_near_radius", m_track_near_radius, n_read);
        return n_read;
    }

    virtual Pose2 getPose()
    {
        cv::AutoLock lock(m_mutex);
        return cvtTopmetric2Metric(getPoseTopometric());
    }

    virtual TopometricPose getPoseTopometric()
    {
        cv::AutoLock lock(m_mutex);
        return m_pose_topo;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        if (EKFLocalizerHyperTan::applyPosition(xy, time, confidence))
        {
            cv::AutoLock lock(m_mutex);
            return applyRoadMap();
        }
        return false;
    }

    virtual std::vector<Node*> getSearchNodes() const
    {
        cv::AutoLock lock(m_mutex);
        return m_search_nodes;
    }

protected:
    bool applyRoadMap()
    {
        cv::AutoLock lock(m_mutex);
        Map* map = getMap();
        if (map == nullptr) return false;

        Pose2 pose_m = EKFLocalizer::getPose();

        if (m_pose_topo.node_id == 0 || m_track_converge_count < m_track_converge_threshold)
        {
            // Initialize (before convergence; same with 'TopoLocalizerProjNear')
            m_search_nodes = map->getNearNodes(pose_m, m_search_radius);
            m_pose_topo = findNearestTopoPose(pose_m, m_search_nodes, m_search_turn_weight);
            if (m_pose_topo.node_id == m_track_prev.node_id && m_pose_topo.edge_idx == m_track_prev.edge_idx && m_pose_topo.dist > m_track_prev.dist) m_track_converge_count++;
            else m_track_converge_count = 0;
        }
        else
        {
            // Project only on the connected edges (after convergence)
            m_search_nodes.clear();
            Node* refer_node = map->getNode(m_track_prev.node_id);
            m_search_nodes.push_back(refer_node);
            Edge* track_edge = map->getEdge(refer_node, m_track_prev.edge_idx);
            if (track_edge != nullptr)
            {
                Node* arrival_node = map->getConnectedNode(refer_node, track_edge->id);
                Point2 d = *arrival_node - *refer_node;
                double remain = /*track_edge->cost*/ sqrt(d.x * d.x + d.y * d.y) - m_track_prev.dist;
                if (remain < m_track_transit_dist)
                {
                    m_search_nodes.push_back(arrival_node);
                    if (m_track_near_radius > 0)
                    {
                        std::vector<Node*> near_nodes = map->getNearNodes(*arrival_node, m_track_near_radius);
                        for (auto n = near_nodes.begin(); n != near_nodes.end(); n++)
                        {
                            if (std::find(m_search_nodes.begin(), m_search_nodes.end(), *n) == m_search_nodes.end())
                                m_search_nodes.push_back(*n);
                        }
                    }
                }
            }
            m_pose_topo = findNearestTopoPose(pose_m, m_search_nodes, m_search_turn_weight);

            Pose2 pose_t = cvtTopmetric2Metric(m_pose_topo);
            double dx = pose_m.x - pose_t.x, dy = pose_m.y - pose_t.y;
            if ((dx * dx + dy * dy) > m_track_drift_radius * m_track_drift_radius) m_track_converge_count = 0;
        }
        m_track_prev = m_pose_topo;
        return true;
    }

    TopometricPose m_pose_topo;

    std::vector<Node*> m_search_nodes;

    double m_search_radius;

    double m_search_turn_weight;

    int m_track_converge_threshold;

    int m_track_converge_count;

    double m_track_transit_dist;

    double m_track_drift_radius;

    double m_track_near_radius;

    TopometricPose m_track_prev;

    int m_track_converge;
};

class EKFLocalizerInterSec : public EKFLocalizerSinTrack
{
public:
    EKFLocalizerInterSec()
    {
        m_inter_appear_threshold = 5;
        m_inter_disappear_time = 2;

        m_inter_appear_count = 0;
        m_inter_appear_time = 0;
    }

    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = EKFLocalizerSinTrack::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "inter_appear_threshold", m_inter_appear_threshold, n_read);
        CX_LOAD_PARAM_COUNT(fn, "inter_disappear_time", m_inter_disappear_time, n_read);
        return n_read;
    }

    virtual bool applyLocClue(ID node_id, const Polar2& obs = Polar2(-1, CV_PI), double time = -1, double confidence = -1)
    {
        if (node_id == 0)
        {
            cv::AutoLock lock(m_mutex);
            if (m_inter_appear_threshold > 0)
            {
                m_inter_appear_count++;
                m_inter_appear_time = time;
                return true;
            }
        }
        return false;
    }

    virtual bool applyPosition(const Point2& xy, double time = -1, double confidence = -1)
    {
        if (EKFLocalizerHyperTan::applyPosition(xy, time, confidence))
        {
            cv::AutoLock lock(m_mutex);
            return applyRoadMap() && applyIntersection(time);
        }
        return false;
    }

    virtual bool applyOrientation(double theta, double time, double confidence = -1)
    {
        if (EKFLocalizerHyperTan::applyOrientation(theta, time, confidence))
        {
            cv::AutoLock lock(m_mutex);
            applyIntersection(time);
            return true;
        }
        return false;
    }

protected:
    bool applyIntersection(double time)
    {
        bool update = false;
        if (m_inter_appear_time > 0)
        {
            double interval = time - m_inter_appear_time;
            if (interval >= m_inter_disappear_time)
            {
                if (m_inter_appear_count > m_inter_appear_threshold)
                {
                    m_pose_topo.dist = m_state_vec.at<double>(3) * interval;
                    update = true;
                }
                m_inter_appear_count = 0;
                m_inter_appear_time = 0;
            }
        }
        return update;
    }

    int m_inter_appear_threshold;

    int m_inter_appear_count;

    double m_inter_appear_time;

    double m_inter_disappear_time;
};

} // End of 'dg'

#endif // End of '__EKF_LOCALIZER_VARIANTS__'
