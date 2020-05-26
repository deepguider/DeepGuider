#ifndef __BASE_LOCALIZER__
#define __BASE_LOCALIZER__

#include "localizer/localizer.hpp"
#include "utils/opencx.hpp"

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

    Pose2 toTopmetric2Metric(const TopometricPose& pose_t)
    {
        cv::AutoLock lock(m_mutex);

        // Find two nodes, 'from' and 'to_id'
        RoadMap::Node* from = m_map.getNode(pose_t.node_id);
        if (from == NULL) return Pose2();
        RoadMap::Node* to = NULL;
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
        if (to == NULL || edge_dist <= 0) return Pose2();

        // Calculate metric pose_m
        double progress = std::min(pose_t.dist / edge_dist, 1.);
        Pose2 pose_m = (1 - progress) * from->data + progress * to->data;
        Point2 d = to->data - from->data;
        pose_m.theta = cx::trimRad(atan2(d.y, d.x) + pose_t.head);
        return pose_m;

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
                const RoadMap::Node* to = edge->to;
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
            if (road_map.addNode(road_node) == NULL)
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
                    if (road_map.addEdge(from->id, to_id, -1) == NULL)
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
                    if (road_map.addEdge(from->id, to_id, edge->length) == NULL)
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
            if (road_map.addNode(road_node) == NULL)
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
            if (road_map.addNode(road_node) == NULL)
            {
                // Return an empty map if failed
                road_map.removeAll();
                return road_map;
            }
        }

        return road_map;
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

protected:
    RoadMap m_map;

    mutable cv::Mutex m_mutex;
}; // End of 'BaseLocalizer'

} // End of 'dg'

#endif // End of '__BASE_LOCALIZER__'
