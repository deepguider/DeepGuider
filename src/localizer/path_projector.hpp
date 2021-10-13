#ifndef __PATH_PROJECTION_HPP__
#define __PATH_PROJECTION_HPP__

#include "core/basic_type.hpp"
#include "core/map.hpp"
#include "core/path.hpp"
#include "utils/opencx.hpp"
#include "utils/ring_buffer.hpp"

namespace dg
{

/**
 * @brief Pose2 with LR
 *
 * Data structure for saving pose with LR info
 */
struct Pose2LR : public Pose2
{
    int lr_side = Edge::LR_NONE;
    Pose2LR(const Pose2& p = Pose2(0, 0), int lr = Edge::LR_NONE): Pose2(p), lr_side(lr) { }
};


/**
 * @brief Path Projection
 *
 * This implement interfaces and basic operations for path-based localization on a graph map.
 */
class PathProjector
{
protected:
    // configuable parameters
    double m_branchmap_search_radius = 100;     // Unit: [m]
    double m_projection_search_radius = 100;    // Unit: [m]
    double m_min_alignscore_gap = 20;           // Unit: [m]
    double m_length_align_weight = 0.5;
    double m_error_tolerance = 0.01;            // Unit: [m]
    bool m_enable_debugging_display = true;

    double m_lr_mismatch_cost = 50;             // Unit: [m]
    bool m_enable_lr_reject = false;
    int m_lr_estimation_interval = 20;
    int m_lr_continuous_n = 10;
    double m_lr_reject_cost = 20;
    bool m_enable_discontinuity_cost = true;
    double m_discontinuity_weight = 0.5;

public:
    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = 0;
        CX_LOAD_PARAM_COUNT(fn, "branchmap_search_radius", m_branchmap_search_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "projection_search_radius", m_projection_search_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "min_alignscore_gap", m_min_alignscore_gap, n_read);
        CX_LOAD_PARAM_COUNT(fn, "length_align_weight", m_length_align_weight, n_read);
        CX_LOAD_PARAM_COUNT(fn, "error_tolerance", m_error_tolerance, n_read);
        CX_LOAD_PARAM_COUNT(fn, "enable_debugging_display", m_enable_debugging_display, n_read);
        CX_LOAD_PARAM_COUNT(fn, "lr_mismatch_cost", m_lr_mismatch_cost, n_read);
        CX_LOAD_PARAM_COUNT(fn, "enable_lr_reject", m_enable_lr_reject, n_read);
        CX_LOAD_PARAM_COUNT(fn, "lr_reject_cost", m_lr_reject_cost, n_read);
        CX_LOAD_PARAM_COUNT(fn, "enable_discontinuity_cost", m_enable_discontinuity_cost, n_read);
        CX_LOAD_PARAM_COUNT(fn, "discontinuity_weight", m_discontinuity_weight, n_read);
        return n_read;
    }

    /**
     * Estimate best map-projected pose of a given pose
     * @param map Map data
     * @param pose Current pose or newly estimated pose (it is usually an output of EKF localization)
     * @param pose_history Recent trajectory of poses (it is usually history of EKF localization outputs)
     * @param projected_pose_history Recent trajectory of map-projected poses
     * @return Estimated best map-projected pose
     */
    Pose2 getMapPose(dg::Map* map, const Pose2& pose, RingBuffer<Pose2LR>& pose_history, RingBuffer<Pose2>& projected_pose_history)
    {
        bool out_of_path;
        return getPathPose(map, nullptr, pose, pose_history, projected_pose_history, out_of_path);
    }

    /**
     * Estimate best path-projected pose of a given pose and detect out of path
     * @param map Map data
     * @param path Current path to a destination
     * @param pose Current pose or newly estimated pose (it is usually an output of EKF localization)
     * @param pose_history Recent trajectory of poses (it is usually history of EKF localization outputs)
     * @param projected_pose_history Recent trajectory of map-projected poses
     * @param out_of_path True if out of path is detected
     * @return Estimated best path-projected pose (it can be out-of-path pose in case out-of-path detected)
     */
    Pose2 getPathPose(dg::Map* map, Path* path, const Pose2& pose, RingBuffer<Pose2LR>& pose_history, RingBuffer<Pose2>& projected_pose_history, bool& out_of_path)
    {
        if (map == nullptr || map->isEmpty()) return pose;
        out_of_path = false;

        // return nearest map point in case of initial pose
        bool initial_pose = projected_pose_history.empty();
        if (initial_pose)
        {
            m_localmap.removeAll();
            m_localmap_center_nid = 0;

            if (path && !path->empty()) return map->getNearestPathPose(*path, pose);
            else return map->getNearestMapPose(pose);
        }

        // find nearest node from the previous pose
        Pose2 mappose_prev = projected_pose_history.back();
        Point2 nearest_edge_point;
        Edge* edge = nullptr;
        if (m_localmap.isEmpty())
            edge = map->getNearestEdge(mappose_prev, nearest_edge_point);
        else
            edge = m_localmap.getNearestEdge(mappose_prev, nearest_edge_point);
        if (edge == nullptr) return pose;

        // build local branch map from the previous pose
        ID localmap_center_nid = edge->node_id1;
        Node* node = map->getNode(edge->node_id1);
        if (node == nullptr) return pose;
        double d1 = norm(nearest_edge_point - *node);
        if (d1 > edge->length / 2)
        {
            localmap_center_nid = edge->node_id2;
        }
        if (m_localmap.isEmpty() || localmap_center_nid != m_localmap_center_nid)
        {
            bool ok = getLocalBranchMap(map, localmap_center_nid, m_localmap, m_branchmap_search_radius);
            if (!ok) return pose;
            m_localmap_center_nid = localmap_center_nid;
        }

        // find oldest pose in the local branch map
        Node* center_node = m_localmap.getNode(localmap_center_nid);
        int pose_eval_len = 0;
        for (int i = projected_pose_history.data_count() - 1; i >= 0; i--)
        {
            if (norm(*center_node - projected_pose_history[i]) > m_branchmap_search_radius) break;
            pose_eval_len++;
        }
        if (pose_eval_len < 1) return m_localmap.getNearestMapPose(pose);

        // estimate LR decision of the latest pose interval
        int lr_decision = Edge::LR_NONE;
        int lr_cnt = 0;
        for (int i = pose_history.data_count() - 1; i >= pose_history.data_count() - m_lr_estimation_interval && i >= 0; i--)
        {
            int lr = pose_history[i].lr_side;
            if (lr == Edge::LR_NONE) continue;
            if (lr_decision == Edge::LR_NONE) lr_decision = lr;
            if (lr != lr_decision) break;
            lr_cnt++;
        }
        if (lr_cnt < m_lr_continuous_n) lr_decision = Edge::LR_NONE;
        if (m_enable_debugging_display && lr_decision == Edge::LR_LEFT) printf("LR_LEFT\n");
        if (m_enable_debugging_display && lr_decision == Edge::LR_RIGHT) printf("LR_RIGHT\n");

        // find candidate map poses
        std::vector<Pose2> map_poses = findProjectedMapPoses(&m_localmap, pose, m_projection_search_radius, m_error_tolerance);
        int path_pose_idx = -1;
        if (path && !path->empty())
        {
            Pose2 path_pose = map->getNearestPathPose(*path, pose);
            for (size_t i = 0; i < map_poses.size(); i++)
            {
                if (norm(path_pose - map_poses[i]) < m_error_tolerance)
                {
                    path_pose_idx = (int)i;
                    break;
                }
            }
            if (path_pose_idx < 0)
            {
                map_poses.push_back(path_pose);
                path_pose_idx = (int)map_poses.size() - 1;
            }
        }
        if(map_poses.empty()) return m_localmap.getNearestMapPose(pose);

        // evaluate candidate map poses
        if (m_enable_debugging_display)
        {
            m_evalMapPath.clear();
            m_evalPoseHistory.clear();
        }
        double min_align_cost = DBL_MAX;
        Pose2 best_map_pose = pose;
        int best_pose_idx = -1;
        double path_pose_cost = -1;
        Pose2 mappose_start = projected_pose_history[projected_pose_history.data_count() - pose_eval_len];
        for (int i = 0; i < (int)map_poses.size(); i++)
        {
            Pose2 map_pose = map_poses[i];

            dg::Path local_path;
            bool ok = m_localmap.getPath(mappose_start, map_pose, local_path);
            if (!ok || local_path.empty()) continue;

            int pose_start_k = pose_history.data_count() - pose_eval_len;
            int lr_match = 0;
            int lr_mismatch = 0;
            std::vector<Point2> eval_path_points;
            double align_cost = computeAlignCost(&m_localmap, local_path, 0, (int)local_path.pts.size() - 1, map_pose, pose_history, pose_start_k, -1, lr_match, lr_mismatch, eval_path_points);

            // lr reject
            bool lr_rejected = false;
            if (lr_decision != Edge::LR_NONE)
            {
                Point2 ep;
                Edge* path_edge = m_localmap.getNearestEdge(map_pose, ep);
                int path_idx = (int)local_path.pts.size() - 2;
                ID path_nid = (path_idx >= 0) ? local_path.pts[path_idx].node_id : 0;
                if (path_edge && path_edge->node_id1 == path_nid && path_edge->lr_side != Edge::LR_NONE && path_edge->lr_side != lr_decision) lr_rejected = true;
                if (path_edge && path_edge->node_id2 == path_nid && path_edge->lr_side != Edge::LR_NONE && path_edge->lr_side == lr_decision) lr_rejected = true;
            }
            if (m_enable_lr_reject && lr_rejected) align_cost += m_lr_reject_cost;

            // abrupt pose change
            double discontinuity_cost = norm(map_pose - mappose_prev) * m_discontinuity_weight;
            if (m_enable_discontinuity_cost) align_cost += discontinuity_cost;

            // save best map pose
            if (align_cost < min_align_cost)
            {
                min_align_cost = align_cost;
                best_map_pose = map_pose;
                best_pose_idx = i;

                if (m_enable_debugging_display)
                {
                    m_evalMapPath = eval_path_points;
                    m_evalPoseHistory.resize(pose_eval_len);
                    int k = 0;
                    for (int j = pose_start_k; j < pose_history.data_count(); j++, k++)
                    {
                        m_evalPoseHistory[k] = pose_history[j];
                    }
                }
            }
            if (path_pose_idx >= 0 && i == path_pose_idx) path_pose_cost = align_cost;
            if (m_enable_debugging_display)
            {
                printf("[%d] algin_cost = %.1lf (path = %d, discont = %.1lf, lr_reject = %d, match = %d, mismatch = %d)\n", i, align_cost, i==path_pose_idx, discontinuity_cost, (int)lr_rejected, lr_match, lr_mismatch);
            }
        }
        if (m_enable_debugging_display) printf("\n");

        // compare to path pose and determine out-of-path
        if (path_pose_idx >= 0 && best_pose_idx != path_pose_idx)
        {
            if ((path_pose_cost - min_align_cost) < m_min_alignscore_gap)
            {
                best_pose_idx = path_pose_idx;
                best_map_pose = map_poses[best_pose_idx];
            }
            else
            {
                out_of_path = true;
            }
        }

        // update projected_pose_history to fit into best local path (remove past pose fluctuations)
        if (best_pose_idx >= 0)
        {
            dg::Path best_path;
            int start_j = projected_pose_history.data_count() - pose_eval_len;
            Pose2 past_pose = projected_pose_history[start_j];
            Pose2 best_pose = map_poses[best_pose_idx];
            bool ok = m_localmap.getPath(past_pose, best_pose, best_path);
            if (ok && !best_path.empty())
            {
                int j = projected_pose_history.data_count() - 1;
                while (j >= start_j && j >= 0)
                {
                    Pose2 new_p = m_localmap.getNearestPathPose(best_path, projected_pose_history[j]);
                    if (norm(new_p - projected_pose_history[j]) < m_error_tolerance) break;
                    projected_pose_history[j] = new_p;
                    j--;
                }
            }
        }

        // estimate heading (현재 map edge 방향으로 수정)
        dg::Path best_path;
        bool ok = m_localmap.getPath(mappose_start, best_map_pose, best_path);
        /*
        if ((int)best_path.pts.size()>=2)
        {
            int last_i = (int)best_path.pts.size() - 1;
            Point2 v = best_path.pts[last_i] - best_path.pts[last_i - 1];
            best_map_pose.theta = atan2(v.y, v.x);
        }
        else
        {
            double theta = atan2(best_map_pose.y - mappose_prev.y, best_map_pose.x - mappose_prev.x);
            best_map_pose.theta = cx::trimRad(theta);
        }
        best_map_pose.theta = cx::trimRad((best_map_pose.theta + projected_pose_history[projected_pose_history.data_count() - 1].theta) / 2);
        */

        dg::Pose2 pose_past = mappose_prev;
        if (projected_pose_history.data_count() >= 4)
        {
            pose_past = projected_pose_history[projected_pose_history.data_count() - 4];
        }
        double theta = atan2(best_map_pose.y - pose_past.y, best_map_pose.x - pose_past.x);
        best_map_pose.theta = cx::trimRad(theta);

        return best_map_pose;
    }

    /**
     * 현재 위치와 연결된 반경 내의 local branch map을 반환 (두 Node 사이에 단방향 edge가 존재하는 경우는 동작하지 않음)
     * @param map 맵 데이터
     * @param center_nid 중심 Node ID
     * @param localmap 탐색된 local branch map
     * @param search_radius 탐색 반경
     * @return Return True if successful
     */
    static bool getLocalBranchMap(dg::Map* map, ID center_nid, dg::Map& localmap, double search_radius)
    {
        if (map == nullptr || map->isEmpty()) return false;
        Node* start = map->getNode(center_nid);
        if (start == nullptr) return false;

        localmap.removeAll();

        std::list<Node*> open;
        open.push_back(start);
        localmap.addNode(*start);

        while (!open.empty())
        {
            // pick next
            auto from = open.front();
            open.pop_front();

            for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
            {
                Node* to = map->getConnectedNode(from, *it);
                if (to == nullptr) continue;

                // check radius
                double d = norm(*to - *start);
                if (d > search_radius) continue;

                // check duplication
                auto node = localmap.getNode(to->id);
                if (node)
                {
                    Edge* edge = map->getEdge(*it);
                    localmap.addEdge(*edge);
                    continue;
                }

                open.push_back(to);
                localmap.addNode(*to);
                Edge* edge = map->getEdge(*it);
                localmap.addEdge(*edge);
            }
        }

        return true;
    }

    /**
     * 주어진 기준 위치에서 반경 내 가능한 모든 맵 투영점들을 반환 (node점 or edge점들)
     * @param map Given map
     * @param p Give reference point
     * @param search_radius Search radius
     * @param error_tolerance Tolerance for arithmetic error
     * @return Return a list of found map projection points
     */
    static std::vector<Pose2> findProjectedMapPoses(dg::Map* map, const Pose2& p, double search_radius, double error_tolerance = 0.01)
    {
        std::vector<Pose2> results;
        if (map == nullptr || map->isEmpty()) return results;

        double search_radius2 = search_radius * search_radius;
        for (auto from = map->getHeadNode(); from != map->getTailNode(); from++)
        {
            if (map->countEdges(&(*from)) == 0) continue;

            double d = norm(*from - p);
            if (d > search_radius) continue;

            // check edge projection: 해당 edge(from)가 gps의 투영점이 될 수 있는지 조사
            Point2 nv = p - *from;
            bool proj_node = true;
            for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
            {
                Node* to = map->getConnectedNode(&(*from), *it);
                if (to == nullptr) continue;
                Point2 ev = *to - *from;
                if (nv.ddot(ev) > 0)
                {
                    proj_node = false;
                    break;
                }
            }
            if (proj_node)
            {
                Point2 np = *from;
                bool already_exist = false;
                for (int k = 0; k < (int)results.size(); k++)
                {
                    if (norm(np - results[k]) <= error_tolerance)
                    {
                        already_exist = true;
                        break;
                    }
                }
                if (!already_exist) results.push_back(Pose2(np));
                //if (!already_exist) printf("prj_map: nid = %zd\n", from->id);
                continue;
            }


            // check edge projection: 해당 edge(from)가 gps의 투영점이 될 수 있는지 조사
            double min_dist2 = DBL_MAX;
            Node* min_to = nullptr;
            ID min_eid = 0;
            Point2 min_ep;
            for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
            {
                Node* to = map->getConnectedNode(&(*from), *it);
                if (to == nullptr) continue;
                auto dist2 = calcDist2FromLineSeg(*from, *to, p);
                if (dist2.first <= search_radius2 && dist2.first < min_dist2)
                {
                    min_dist2 = dist2.first;
                    min_to = to;
                    min_ep = dist2.second;
                    min_eid = *it;
                }
            }
            if (min_to)
            {
                double d1 = norm(min_ep - *from);
                double d2 = norm(min_ep - *min_to);
                if (d1 > 0 && d2 > 0)
                {
                    bool already_exist = false;
                    for (int k = 0; k < (int)results.size(); k++)
                    {
                        if (norm(min_ep - results[k]) <= error_tolerance)
                        {
                            already_exist = true;
                            break;
                        }
                    }
                    if (!already_exist) results.push_back(Pose2(min_ep));
                    //if (!already_exist) printf("prj_map: edge_id = %zd\n", min_eid);
                }
            }
        }

        return results;
    }

    static Pose2 findNearestPathPose(const Path& path, const Pose2& pose, int& path_idx, double turn_weight = 0)
    {
        path_idx = -1;
        if (path.empty()) return pose;

        // Find the nearest path point
        int min_path_idx = 0;
        std::pair<double, Point2> min_dist2 = std::make_pair(DBL_MAX, Point2());
        for (int i = 0; i < (int)path.pts.size() - 1; i++)
        {
            auto dist2 = calcDist2FromLineSeg(path.pts[i], path.pts[i + 1], pose, turn_weight);
            if (dist2.first < min_dist2.first)
            {
                min_dist2 = dist2;
                min_path_idx = i;
            }
        }

        // Set the pose to nearest path pose
        Pose2 path_pose = pose;
        if (min_path_idx < (int)path.pts.size() - 1)
        {
            path_pose.x = min_dist2.second.x;
            path_pose.y = min_dist2.second.y;
            double dx = path.pts[min_path_idx + 1].x - path.pts[min_path_idx].x;
            double dy = path.pts[min_path_idx + 1].y - path.pts[min_path_idx].y;
            path_pose.theta = cx::trimRad(atan2(dy, dx));
        }
        path_idx = min_path_idx;

        return path_pose;
    }

    void reset()
    {
        m_localmap.removeAll();
        m_localmap_center_nid = 0;
        m_evalMapPath.clear();
        m_evalPoseHistory.clear();
    }

    std::vector<Point2>& getEvalPath() { return m_evalMapPath; }

    std::vector<Point2>& getEvalPoseHistory() { return m_evalPoseHistory; }

protected:
    double computeAlignCost(dg::Map* map, const dg::Path& path, int path_idx1, int path_idx2, const Pose2& projected_path_pose, const RingBuffer<Pose2LR>& pose_history, int pose_idx1, int pose_idx2, int& n_lr_match, int& n_lr_mismatch, std::vector<Point2>& eval_path_points)
    {
        if (pose_idx2 < 0) pose_idx2 = pose_history.data_count() - 1;
        if (pose_idx1 < 0) pose_idx1 = 0;
        if (pose_idx2 > pose_history.data_count() - 1) pose_idx2 = pose_history.data_count() - 1;

        // compute total length of evaluating pose history
        double pose_len_total = 0;
        for (int k = pose_idx1 + 1; k <= pose_idx2; k++)
        {
            pose_len_total += norm(pose_history[k] - pose_history[k - 1]);
        }

        // compute path length
        Point2 p1 = path.pts[path_idx1];
        double path_len_total = 0;
        std::vector<Point2> path_node_points;
        path_node_points.push_back(p1);
        for (int k = path_idx1 + 1; k <= path_idx2; k++)
        {
            Point2 p2 = path.pts[k];
            path_len_total += norm(p2 - p1);
            p1 = p2;
            path_node_points.push_back(p1);
        }
        path_len_total += norm(projected_path_pose - p1);
        path_node_points.push_back(projected_path_pose);

        // compute align cost
        double pose_len_upto = 0;
        int path_points_idx = 0;
        double edge_len = ((int)path_node_points.size() > 1) ? norm(path_node_points[1] - path_node_points[0]) : 0;
        double path_len_upto = 0;
        double align_cost = norm(pose_history[pose_idx1] - path_node_points[0]);
        for (int k = pose_idx1 + 1; k <= pose_idx2; k++)
        {
            pose_len_upto += norm(pose_history[k] - pose_history[k - 1]);
            double len_ratio_upto = pose_len_upto / pose_len_total;
            double target_path_len = path_len_total * len_ratio_upto;
            while (target_path_len > path_len_upto + edge_len && path_points_idx < (int)path_node_points.size() - 1)
            {
                path_len_upto += edge_len;
                path_points_idx++;
                edge_len = (path_points_idx < (int)path_node_points.size() - 1) ? norm(path_node_points[path_points_idx + 1] - path_node_points[path_points_idx]) : 0;
            }
            double target_edge_len = target_path_len - path_len_upto;
            Point2 path_point = (edge_len > 0 && path_points_idx < (int)path_node_points.size() - 1) ? path_node_points[path_points_idx] + (path_node_points[path_points_idx + 1] - path_node_points[path_points_idx]) * target_edge_len / edge_len : path_node_points[path_points_idx];
            eval_path_points.push_back(path_point);

            // distance cost
            align_cost += norm(pose_history[k] - path_point);

            // lr cost
            Edge* edge = map->getEdge(path.pts[path_points_idx].edge_id);
            if (pose_history[k].lr_side != Edge::LR_NONE && edge && edge->lr_side != Edge::LR_NONE)
            {
                bool lr_mismatch = false;
                if (edge->node_id1 == path.pts[path_points_idx].node_id && edge->lr_side != pose_history[k].lr_side) lr_mismatch = true;
                if (edge->node_id2 == path.pts[path_points_idx].node_id && edge->lr_side == pose_history[k].lr_side) lr_mismatch = true;
                if (lr_mismatch)
                {
                    align_cost += m_lr_mismatch_cost;
                    n_lr_mismatch++;
                }
                else
                {
                    align_cost -= m_lr_mismatch_cost;
                    n_lr_match++;
                }
            }
        }
        double average_align_cost = align_cost / (pose_idx2 - pose_idx1 + 1);
        double length_cost = fabs(path_len_total - pose_len_total);

        return (average_align_cost + m_length_align_weight * length_cost);
    }

    // cache memory for speeding up the process
    Map m_localmap;
    ID m_localmap_center_nid = 0;

    /** temporal variables for debugging display */
    std::vector<Point2> m_evalMapPath;
    std::vector<Point2> m_evalPoseHistory;

}; // End of 'PathProjector'

} // End of 'dg'


#endif // End of '__PATH_PROJECTION_HPP__'
