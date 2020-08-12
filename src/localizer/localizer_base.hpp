#ifndef __BASE_LOCALIZER__
#define __BASE_LOCALIZER__

#include "core/map.hpp"
#include "localizer/localizer.hpp"
#include "utils/opencx.hpp"
#include <set>

namespace dg
{

class BaseLocalizer : public Localizer, public TopometricLocalizer, public UTMConverter
{
public:
    virtual bool loadMap(Map& map, bool auto_cost = false)
    {
        cv::AutoLock lock(m_mutex);
        m_map = cvtMap2RoadMap(map, *this, auto_cost);
        return true;
    }

    virtual bool loadMap(const RoadMap& map)
    {
        cv::AutoLock lock(m_mutex);
        return map.copyTo(&m_map);
    }

    virtual RoadMap getMap() const
    {
        cv::AutoLock lock(m_mutex);
        return m_map;
    }

    Pose2 cvtTopmetric2Metric(const TopometricPose& pose_t)
    {
        cv::AutoLock lock(m_mutex);

        // Find two nodes, 'from' and 'to_id'
        RoadMap::Node* from = m_map.getNode(pose_t.node_id);
        if (from == nullptr) return Pose2();
        RoadMap::Node* to = nullptr;
        int edge_idx = 0;
        double edge_dist = 0;
        for (auto edge = m_map.getHeadEdgeConst(from); edge != m_map.getTailEdgeConst(from); edge++, edge_idx++)
        {
            if (edge_idx == pose_t.edge_idx)
            {
                to = edge->to;
                edge_dist = edge->cost;
                break;
            }
        }
        if (to == nullptr || edge_dist <= 0) return Pose2();

        // Calculate metric pose_m
        double progress = std::min(pose_t.dist / edge_dist, 1.);
        Pose2 pose_m = (1 - progress) * from->data + progress * to->data;
        Point2 d = to->data - from->data;
        pose_m.theta = cx::trimRad(atan2(d.y, d.x) + pose_t.head);
        return pose_m;

    }

    TopometricPose findNearestTopoPose(const Pose2& pose_m, double turn_weight = 0, double search_range = -1, const Pose2& search_pt = Pose2())
    {
        cv::AutoLock lock(m_mutex);

        // Find the nearest edge
        std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
        ID min_node_id = 0;
        int min_edge_idx = 0;
        const double range2 = search_range * search_range;
        for (auto from = m_map.getHeadNodeConst(); from != m_map.getTailNodeConst(); from++)
        {
            if (m_map.countEdges(from) == 0) continue;
            if (search_range > 0)
            {
                double dx = pose_m.x - search_pt.x, dy = pose_m.y - search_pt.y;
                if ((dx * dx + dy * dy) > range2) continue;
            }

            int edge_idx = 0;
            for (auto edge = m_map.getHeadEdgeConst(from); edge != m_map.getTailEdgeConst(from); edge++, edge_idx++)
            {
                const RoadMap::Node* to = edge->to;
                if (to == nullptr) continue;
                auto dist2 = calcDist2FromLineSeg(from->data, to->data, pose_m, turn_weight);
                if (dist2.first < min_dist2.first)
                {
                    min_dist2 = dist2;
                    min_node_id = from->data.id;
                    min_edge_idx = edge_idx;
                }
            }
        }

        // Return the nearest topometric pose
        TopometricPose pose_t;
        if (min_dist2.first != DBL_MAX)
        {
            auto from = m_map.getNode(min_node_id);
            if (from != nullptr)
            {
                pose_t.node_id = min_node_id;
                pose_t.edge_idx = min_edge_idx;
                double dx = min_dist2.second.x - from->data.x;
                double dy = min_dist2.second.y - from->data.y;
                pose_t.dist = sqrt(dx * dx + dy * dy);
                pose_t.head = cx::trimRad(pose_m.theta - atan2(dy, dx));
            }
        }
        return pose_t;
    }

    TopometricPose trackTopoPose(const TopometricPose& topo_from, const Pose2& pose_m, double turn_weight = 0, int extend_depth = 1)
    {
        cv::AutoLock lock(m_mutex);

        // Check 'pose_m' on the current edge
        TopometricPose pose_t;
        RoadMap::Node* node_from = m_map.getNode(topo_from.node_id);
        if (node_from == nullptr) return pose_t;
        RoadMap::Edge* edge_curr = m_map.getEdge(node_from, topo_from.edge_idx);
        if (edge_curr == nullptr) return pose_t;
        RoadMap::Node* node_goal = m_map.getNode(edge_curr->to->data.id);
        if (node_goal == nullptr) return pose_t;
        auto min_dist2 = calcDist2FromLineSeg(node_from->data, node_goal->data, pose_m, turn_weight);
        ID min_node_id = topo_from.node_id;
        int min_edge_idx = topo_from.edge_idx;

        // Check 'pose_m' on the connected edges
        std::set<ID> node_visit;
        std::queue<RoadMap::Node*> node_queue;
        node_queue.push(node_goal);
        for (int depth = 0; depth < extend_depth; depth++)
        {
            if (node_queue.empty()) break;
            RoadMap::Node* node_pick = node_queue.front();
            node_queue.pop();
            node_visit.insert(node_pick->data.id);
            int edge_idx = 0;
            for (auto edge = m_map.getHeadEdgeConst(node_pick); edge != m_map.getTailEdgeConst(node_pick); edge++, edge_idx++)
            {
                RoadMap::Node* to = edge->to;
                if (to == nullptr) continue;
                auto dist2 = calcDist2FromLineSeg(node_pick->data, to->data, pose_m, turn_weight);
                if (dist2.first < min_dist2.first)
                {
                    min_dist2 = dist2;
                    min_node_id = node_goal->data.id;
                    min_edge_idx = edge_idx;
                }
                if (node_visit.find(to->data.id) == node_visit.end())
                    node_queue.push(to);
            }
        }

        // Return the updated topometric pose
        auto from = m_map.getNodeConst(min_node_id);
        if (from != m_map.getTailNodeConst())
        {
            pose_t.node_id = min_node_id;
            pose_t.edge_idx = min_edge_idx;
            double dx = min_dist2.second.x - from->data.x;
            double dy = min_dist2.second.y - from->data.y;
            pose_t.dist = sqrt(dx * dx + dy * dy);
            pose_t.head = cx::trimRad(pose_m.theta - atan2(dy, dx));
        }
        return pose_t;
    }

    static RoadMap cvtMap2RoadMap(Map& map, const UTMConverter& converter, bool auto_cost = true)
    {
        RoadMap road_map;

        // Copy nodes
        for (auto node = map.nodes.begin(); node != map.nodes.end(); node++)
        {
            Point2ID road_node(node->id, converter.toMetric(*node));
            if (road_map.addNode(road_node) == nullptr)
            {
                // Return an empty map if failed
                road_map.removeAll();
                return road_map;
            }
        }

        // Copy edges
        if (auto_cost)
        {
            for (auto from = map.nodes.begin(); from != map.nodes.end(); from++)
            {
                for (auto edge_id = from->edge_ids.begin(); edge_id != from->edge_ids.end(); edge_id++)
                {
                    const Edge* edge = map.findEdge(*edge_id);
                    if (edge == nullptr) continue;
                    ID to_id = edge->node_id2;
                    if (from->id == to_id) to_id = edge->node_id1;
                    if (road_map.addEdge(from->id, to_id, -1) == nullptr)
                    {
                        // Return an empty map if failed
                        road_map.removeAll();
                        return road_map;
                    }
                }
            }
        }
        else
        {
            for (auto from = map.nodes.begin(); from != map.nodes.end(); from++)
            {
                for (auto edge_id = from->edge_ids.begin(); edge_id != from->edge_ids.end(); edge_id++)
                {
                    const Edge* edge = map.findEdge(*edge_id);
                    if (edge == nullptr) continue;
                    ID to_id = edge->node_id2;
                    if (from->id == to_id) to_id = edge->node_id1;
                    if (road_map.addEdge(from->id, to_id, edge->length) == nullptr)
                    {
                        // Return an empty map if failed
                        road_map.removeAll();
                        return road_map;
                    }
                }
            }
        }

        // Copy POIs
        for (auto poi = map.pois.begin(); poi != map.pois.end(); poi++)
        {
            Point2ID road_node(poi->id, converter.toMetric(*poi));
            if (road_map.addNode(road_node) == nullptr)
            {
                // Return an empty map if failed
                road_map.removeAll();
                return road_map;
            }
        }

        // Copy StreetViews
        for (auto view = map.views.begin(); view != map.views.end(); view++)
        {
            Point2ID road_node(view->id, converter.toMetric(*view));
            if (road_map.addNode(road_node) == nullptr)
            {
                // Return an empty map if failed
                road_map.removeAll();
                return road_map;
            }
        }

        return road_map;
    }

    static std::pair<double, Point2> calcDist2FromLineSeg(const Point2& from, const Point2& to, const Pose2& p, double turn_weight = 0)
    {
        // Ref. https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
        Point2 delta = to - from;
        double l2 = delta.x * delta.x + delta.y * delta.y;
        if (l2 < DBL_EPSILON)
        {
            Point2 dp = p - from;
            return std::make_pair(dp.x * dp.x + dp.y * dp.y, from);
        }
        double t = std::max(0., std::min(1., (p - from).dot(to - from) / l2));
        Point2 projection = from + t * (to - from);
        Point2 dp = p - projection;
        double dist2 = dp.x * dp.x + dp.y * dp.y;
        if (turn_weight > 0)
        {
            double dh = cx::trimRad(p.theta - atan2(delta.y, delta.x));
            dist2 += turn_weight * dh * dh;
        }
        return std::make_pair(dist2, projection);
    }

protected:
    RoadMap m_map;

    mutable cv::Mutex m_mutex;
}; // End of 'BaseLocalizer'

} // End of 'dg'

#endif // End of '__BASE_LOCALIZER__'
