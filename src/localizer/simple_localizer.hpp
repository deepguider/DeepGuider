#ifndef __SIMPLE_LOCALIZER__
#define __SIMPLE_LOCALIZER__

#include "localizer/localizer.hpp"
#include "opencx.hpp"

namespace dg
{

class SimpleLocalizer : public Localizer, public TopometricLocalizer, public UTMConverter
{
public:
    Pose2 toTopmetric2Metric(const TopometricPose& t_pose)
    {
        cv::AutoLock lock(m_mutex);

        // Find two nodes, 'from' and 'to'
        SimpleRoadMap::Node* from = m_map.findNode(t_pose.node_id);
        if (from == NULL) return Pose2();
        SimpleRoadMap::Node* to = NULL;
        int edge_idx = 0;
        double edge_dist = 0;
        for (auto edge = m_map.getHeadEdgeConst(from); edge != m_map.getTailEdgeConst(from); edge++, edge_idx++)
        {
            if (edge_idx == t_pose.edge_idx)
            {
                to = edge->to;
                edge_dist = edge->cost;
                break;
            }
        }
        if (to == NULL || edge_dist <= 0) return Pose2();

        // Calculate metric pose
        double progress = std::min(t_pose.dist / edge_dist, 1.);
        Pose2 m_pose = (1 - progress) * from->data + progress * to->data;
        Point2 d = to->data - from->data;
        m_pose.theta = atan2(d.y, d.x);
        return m_pose;

    }

    static std::pair<double, Point2> calcDist2FromLineSeg(const Point2& v, const Point2& w, const Point2& p)
    {
        // Ref. https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
        Point2 d = w - v;
        double l2 = d.x * d.x + d.y * d.y;
        if (l2 < DBL_EPSILON)
        {
            d = p - v;
            return std::make_pair(d.x * d.x + d.y * d.y, v);
        }
        double t = std::max(0., std::min(1., (p - v).dot(w - v) / l2));
        Point2 projection = v + t * (w - v);
        d = p - projection;
        return std::make_pair(d.x * d.x + d.y * d.y, projection);
    }

    TopometricPose toMetric2Topometric(const Pose2& pose_m) const
    {
        cv::AutoLock lock(m_mutex);

        // Find the nearest edge
        std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
        ID min_node_id = 0;
        int min_edge_idx = 0;
        for (auto from = m_map.getHeadNodeConst(); from != m_map.getTailNodeConst(); from++)
        {
            int edge_idx = 0;
            for (auto edge = m_map.getHeadEdgeConst(from); edge != m_map.getTailEdgeConst(from); edge++, edge_idx++)
            {
                const SimpleRoadMap::Node* to = edge->to;
                if (to == NULL) continue;
                auto dist2 = calcDist2FromLineSeg(from->data, to->data, pose_m);
                if (dist2.first < min_dist2.first)
                {
                    min_dist2 = dist2;
                    min_node_id = from->data.id;
                    min_edge_idx = edge_idx;
                }
            }
        }

        // Return topometric pose
        TopometricPose pose_t;
        if (min_dist2.first != DBL_MAX)
        {
            auto from = m_map.getNodeConst(min_node_id);
            if (from != m_map.getTailNodeConst())
            {
                pose_t.node_id = min_node_id;
                pose_t.edge_idx = min_edge_idx;
                double dx = min_dist2.second.x - from->data.x;
                double dy = min_dist2.second.y - from->data.y;
                pose_t.dist = sqrt(dx * dx + dy * dy);
            }
        }
        return pose_t;
    }

    virtual Pose2 getPose() const
    {
        cv::AutoLock lock(m_mutex);
        return m_pose_metric;
    }

    virtual LatLon getPoseGPS() const
    {
        return toLatLon(getPose());
    }

    virtual TopometricPose getPoseTopometric() const
    {
        return toMetric2Topometric(m_pose_metric);
    }

    virtual double getPoseConfidence() const
    {
        return -1;
    }

    static SimpleRoadMap cvtMap2SimpleRoadMap(const Map& map, const UTMConverter& converter, bool auto_cost = true)
    {
        SimpleRoadMap simple_map;

        // Copy nodes
        for (auto node = map.getHeadNodeConst(); node != map.getTailNodeConst(); node++)
        {
            Point2ID simple_node(node->data.id, converter.toMetric(node->data));
            if (simple_map.addNode(simple_node) == NULL)
            {
                // Return an empty map if failed
                simple_map.removeAll();
                return simple_map;
            }
        }

        if (auto_cost)
        {
            // Copy edges with automatic distance calculation
            for (auto node = map.getHeadNodeConst(); node != map.getTailNodeConst(); node++)
            {
                for (auto edge = map.getHeadEdgeConst(node); edge != map.getTailEdgeConst(node); edge++)
                {
                    if (simple_map.addEdge(node->data.id, edge->to->data.id, -1) == NULL)
                    {
                        // Return an empty map if failed
                        simple_map.removeAll();
                        return simple_map;
                    }
                }
            }
        }
        else
        {
            // Copy edges with automatic distance calculation
            for (auto node = map.getHeadNodeConst(); node != map.getTailNodeConst(); node++)
            {
                for (auto edge = map.getHeadEdgeConst(node); edge != map.getTailEdgeConst(node); edge++)
                {
                    if (simple_map.addEdge(node->data.id, edge->to->data.id, edge->cost.length) == NULL)
                    {
                        // Return an empty map if failed
                        simple_map.removeAll();
                        return simple_map;
                    }
                }
            }
        }
        return simple_map;
    }

    virtual bool loadMap(const Map& map, bool auto_cost = false)
    {
        cv::AutoLock lock(m_mutex);
        m_map = cvtMap2SimpleRoadMap(map, *this, auto_cost);
        return true;
    }

    virtual bool loadMap(const SimpleRoadMap& map)
    {
        cv::AutoLock lock(m_mutex);
        return map.copyTo(&m_map);
    }

    virtual SimpleRoadMap getMap() const
    {
        cv::AutoLock lock(m_mutex);
        return m_map;
    }

    virtual bool applyPose(const Pose2& pose, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric = pose;
        return true;
    }

    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.x = xy.x;
        m_pose_metric.y = xy.y;
        return true;
    }

    virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1)
    {
        Point2 xy = toMetric(ll);
        cv::AutoLock lock(m_mutex);
        m_pose_metric.x = xy.x;
        m_pose_metric.y = xy.y;
        return true;
    }

    virtual bool applyOrientation(double theta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.theta = theta;
        return true;
    }

    virtual bool applyOdometry(const Pose2& pose_curr, const Pose2& pose_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        double dx = pose_curr.x - pose_prev.x;
        double dy = pose_curr.y - pose_prev.y;
        double c = cos(m_pose_metric.theta - pose_prev.theta), s = sin(m_pose_metric.theta - pose_prev.theta);
        m_pose_metric.x += c * dx - s * dy;
        m_pose_metric.x += s * dx + c * dy;
        m_pose_metric.theta = cx::trimRad(m_pose_metric.theta + pose_curr.theta - pose_prev.theta);
        return true;
    }

    virtual bool applyOdometry(const Polar2& delta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.x += delta.lin * cos(m_pose_metric.theta + delta.ang / 2);
        m_pose_metric.y += delta.lin * sin(m_pose_metric.theta + delta.ang / 2);
        m_pose_metric.theta = cx::trimRad(m_pose_metric.theta + delta.ang);
        return true;
    }

    virtual bool applyOdometry(double theta_curr, double theta_prev, Timestamp time_curr = -1, Timestamp time_prev = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        m_pose_metric.theta = cx::trimRad(m_pose_metric.theta + theta_curr - theta_prev);
        return true;
    }

    virtual bool applyLocClue(ID node_id, const Polar2& obs = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        SimpleRoadMap::Node* node = m_map.getNode(Point2ID(node_id));
        if (node == NULL) return false;
        cv::AutoLock lock(m_mutex);
        m_pose_metric.x = node->data.x;
        m_pose_metric.y = node->data.y;
        return true;
    }

    virtual bool applyLocClue(const std::vector<ID>& node_ids, const std::vector<Polar2>& obs, Timestamp time = -1, const std::vector<double>& confidence = std::vector<double>())
    {
        if (node_ids.empty() || obs.empty() || node_ids.size() != obs.size()) return false;
        return applyLocClue(node_ids.front(), obs.front(), time);
    }

protected:
    SimpleRoadMap m_map;

    Pose2 m_pose_metric;

    mutable cv::Mutex m_mutex;
};

} // End of 'dg'

#endif // End of '__SIMPLE_LOCALIZER__'
