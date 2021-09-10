#ifndef __BASE_LOCALIZER__
#define __BASE_LOCALIZER__

#include "core/map.hpp"
#include "localizer/localizer_interface.hpp"
#include "utils/opencx.hpp"
#include <set>

namespace dg
{

class BaseLocalizer : public SensorInterface, public TopometricInterface, public cx::Algorithm
{
public:
    virtual bool setShared(SharedInterface* shared)
    {
        m_shared = shared;
        return (m_shared != nullptr);
    }

    virtual Map* getMap()
    {
        if (m_shared) return m_shared->getMap();
        return nullptr;
    }

    virtual const Map* getMap() const
    {
        if (m_shared) return m_shared->getMap();
        return nullptr;
    }

    virtual LatLon getPoseGPS(Timestamp* timestamp = nullptr) const
    {
        Pose2 pose_m = getPose(timestamp);
        return toLatLon(pose_m);
    }

    virtual Point2UTM getPoseUTM(Timestamp* timestamp = nullptr) const
    {
        Pose2 metric = getPose(timestamp);
        if (m_shared) return m_shared->cvtLatLon2UTM(toLatLon(metric));
        return Point2UTM();
    }

    virtual TopometricPose getPoseTopometric(Timestamp* timestamp = nullptr) const
    {
        Pose2 pose_m = getPose(timestamp);
        return findNearestTopoPose(pose_m);
    }

    virtual LatLon toLatLon(const Point2& metric) const
    {
        if(m_shared) return m_shared->toLatLon(metric);
        return LatLon();
    }

    virtual Point2 toMetric(const LatLon& ll) const
    {
        if(m_shared) return m_shared->toMetric(ll);
        return Point2();
    }

protected:
    SharedInterface* m_shared = nullptr;
    mutable cv::Mutex m_mutex;

    virtual Pose2 cvtTopmetric2Metric(const TopometricPose& pose_t) const
    {
        const Map* map = getMap();
        if (map == nullptr) return Pose2();

        // Find two nodes, 'from' and 'to_id'
        const Node* from = map->getNode(pose_t.node_id);
        if (from == nullptr) return Pose2();
        const Edge* edge = map->getEdge(from, pose_t.edge_idx);
        if (edge == nullptr) return Pose2();
        const Node* to = map->getConnectedNode(from, edge->id);
        if (to == nullptr) return Pose2();

        // Calculate metric pose_m
        Point2 v = *to - *from;
        Pose2 pose_m = *from + pose_t.dist * v / edge->length;
        pose_m.theta = cx::trimRad(atan2(v.y, v.x) + pose_t.head);
        return pose_m;
    }

    virtual TopometricPose findNearestTopoPose(const Pose2& pose_m, double turn_weight = 0) const
    {
        const Map* map = getMap();
        if (map == nullptr) return TopometricPose();

        // Find the nearest edge
        Point2 ep;
        const Edge* edge = map->getNearestEdge(pose_m, ep);
        if (edge == nullptr) return TopometricPose();

        // Find a starting node
        const Node* node1 = map->getNode(edge->node_id1);
        const Node* node2 = map->getNode(edge->node_id2);
        if (node1 == nullptr || node2 == nullptr) return TopometricPose();
        Point2 v1 = *node2 - *node1;
        Point2 v2 = *node1 - *node2;
        double dt1 = fabs(cx::trimRad(pose_m.theta - atan2(v1.y, v1.x)));
        double dt2 = fabs(cx::trimRad(pose_m.theta - atan2(v2.y, v2.x)));
        const Node* from = (dt1 <= dt2) ? node1 : node2;

        // Return the nearest topometric pose
        TopometricPose pose_t;
        pose_t.node_id = from->id;
        pose_t.edge_idx = map->getEdgeIndex(from, edge->id);
        Point2 v = ep - *from;
        pose_t.dist = norm(v);
        pose_t.head = cx::trimRad(pose_m.theta - atan2(v.y, v.x));
        return pose_t;
    }

    virtual TopometricPose findNearestTopoPose(const Pose2& pose_m, const std::vector<Node*>& search_nodes, double turn_weight = 0) const
    {
        const Map* map = getMap();
        if (map == nullptr) return TopometricPose();

        // Find the nearest edge
        std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
        ID min_edge_id = 0;
        for (auto it = search_nodes.begin(); it != search_nodes.end(); it++)
        {
            const Node* from = *it;
            for (size_t edge_idx = 0; edge_idx < from->edge_ids.size(); edge_idx++)
            {
                const Node* to = map->getConnectedNode(from, from->edge_ids[edge_idx]);
                if (to == nullptr) continue;
                auto dist2 = calcDist2FromLineSeg(*from, *to, pose_m, turn_weight);
                if (dist2.first < min_dist2.first)
                {
                    min_dist2 = dist2;
                    min_edge_id = from->edge_ids[edge_idx];
                }
            }
        }
        const Edge* edge = map->getEdge(min_edge_id);
        if (edge == nullptr) return TopometricPose();

        // Find a starting node
        const Node* node1 = map->getNode(edge->node_id1);
        const Node* node2 = map->getNode(edge->node_id2);
        if (node1 == nullptr || node2 == nullptr) return TopometricPose();
        Point2 v1 = *node2 - *node1;
        Point2 v2 = *node1 - *node2;
        double dt1 = fabs(cx::trimRad(pose_m.theta - atan2(v1.y, v1.x)));
        double dt2 = fabs(cx::trimRad(pose_m.theta - atan2(v2.y, v2.x)));
        const Node* from = (dt1 <= dt2) ? node1 : node2;

        // Return the nearest topometric pose
        TopometricPose pose_t;
        pose_t.node_id = from->id;
        pose_t.edge_idx = map->getEdgeIndex(from, edge->id);
        Point2 v = min_dist2.second - *from;
        pose_t.dist = norm(v);
        pose_t.head = cx::trimRad(pose_m.theta - atan2(v.y, v.x));
        return pose_t;
    }

}; // End of 'BaseLocalizer'

} // End of 'dg'

#endif // End of '__BASE_LOCALIZER__'
