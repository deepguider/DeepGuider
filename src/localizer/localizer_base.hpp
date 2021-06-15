#ifndef __BASE_LOCALIZER__
#define __BASE_LOCALIZER__

#include "core/map.hpp"
#include "localizer/localizer.hpp"
#include "localizer/road_map.hpp"
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

    virtual RoadMap* getMap()
    {
        cv::AutoLock lock(m_mutex);
        return &m_map;
    }

    Pose2 cvtTopmetric2Metric(const TopometricPose& pose_t)
    {
        cv::AutoLock lock(m_mutex);

        // Find two nodes, 'from' and 'to_id'
        RoadMap::Node* from = m_map.getNode(pose_t.node_id);
        if (from == nullptr) return Pose2();
        RoadMap::Edge* edge = m_map.getEdge(from, pose_t.edge_idx);
        if (edge == nullptr) return Pose2();

        // Calculate metric pose_m
        Point2 d = edge->to->data - from->data;
        double dist = sqrt(d.x * d.x + d.y * d.y);
        Pose2 pose_m = from->data + pose_t.dist * d / dist;
        pose_m.theta = cx::trimRad(atan2(d.y, d.x) + pose_t.head);
        return pose_m;
    }

    std::vector<RoadMap::Node*> findNearNodes(const Pose2& pose_m, double search_radius)
    {
        std::vector<RoadMap::Node*> nodes;
        if (search_radius <= 0)
        {
            // Return all nodes except stand-alone nodes
            for (auto n = m_map.getHeadNode(); n != m_map.getTailNode(); n++)
            {
                if (m_map.countEdges(n) == 0) continue;
                nodes.push_back(&(*n));
            }
        }
        else
        {
            // Find nodes within the given radius
            const double radius2 = search_radius * search_radius;
            for (auto n = m_map.getHeadNode(); n != m_map.getTailNode(); n++)
            {
                if (m_map.countEdges(n) == 0) continue;
                double dx = pose_m.x - n->data.x, dy = pose_m.y - n->data.y;
                if ((dx * dx + dy * dy) < radius2) nodes.push_back(&(*n));
            }
        }
        return nodes;
    }

    TopometricPose findNearestTopoPose(const Pose2& pose_m, const std::vector<RoadMap::Node*>& search_nodes, double turn_weight = 0)
    {
        // Find the nearest edge
        std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
        ID min_node_id = 0;
        int min_edge_idx = 0;
        for (auto from = search_nodes.begin(); from != search_nodes.end(); from++)
        {
            int edge_idx = 0;
            for (auto edge = m_map.getHeadEdgeConst(*from); edge != m_map.getTailEdgeConst(*from); edge++, edge_idx++)
            {
                const RoadMap::Node* to = edge->to;
                if (to == nullptr) continue;
                auto dist2 = calcDist2FromLineSeg((*from)->data, to->data, pose_m, turn_weight);
                if (dist2.first < min_dist2.first)
                {
                    min_dist2 = dist2;
                    min_node_id = (*from)->data.id;
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
