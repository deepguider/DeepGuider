#ifndef __PATH_PROJECTION_HPP__
#define __PATH_PROJECTION_HPP__

#include "core/basic_type.hpp"
#include "core/map.hpp"
#include "core/path.hpp"
#include "utils/opencx.hpp"
#include "utils/utility.hpp"

namespace dg
{

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
    double m_projection_search_radius = 60;     // Unit: [m]
    double m_eval_pathlen_backward = 50;        // Unit: [m]
    double m_eval_pathlen_forward = 25;         // Unit: [m]
    double m_min_alignscore_gap = 30;           // Unit: [m]
    int m_min_eval_historylen = 10;
    double m_length_align_weight = 0.5;
    double m_error_tolerance = 0.01;            // Unit: [m]
    bool m_enable_debugging_display = false;

public:
    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = 0;
        CX_LOAD_PARAM_COUNT(fn, "branchmap_search_radius", m_branchmap_search_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "projection_search_radius", m_projection_search_radius, n_read);
        CX_LOAD_PARAM_COUNT(fn, "eval_pathlen_backward", m_eval_pathlen_backward, n_read);
        CX_LOAD_PARAM_COUNT(fn, "eval_pathlen_forward", m_eval_pathlen_forward, n_read);
        CX_LOAD_PARAM_COUNT(fn, "min_alignscore_gap", m_min_alignscore_gap, n_read);
        CX_LOAD_PARAM_COUNT(fn, "min_eval_historylen", m_min_eval_historylen, n_read);
        CX_LOAD_PARAM_COUNT(fn, "length_align_weight", m_length_align_weight, n_read);
        CX_LOAD_PARAM_COUNT(fn, "error_tolerance", m_error_tolerance, n_read);
        CX_LOAD_PARAM_COUNT(fn, "enable_debugging_display", m_enable_debugging_display, n_read);
        return n_read;
    }

    /**
     * Estimate best map-projected pose of a given pose
     * @param map Map data
     * @param pose Current pose or newly estimated robot pose (it is usually an output of EKF localization)
     * @param mappose_prev Latest map-projected pose
     * @return Estimated best map pose
     */
    Pose2 getMapPose(dg::Map* map, const Pose2& pose, const Pose2& mappose_prev, bool initial_pose = false)
    {
        if (map == nullptr || map->isEmpty()) return pose;

        // return nearest map point in case of initial pose
        if (initial_pose)
        {
            m_localmap.removeAll();
            m_localmap_center_nid = 0;

            Point2 nearest_edge_point;
            Edge* edge = map->getNearestEdge(pose, nearest_edge_point);
            if (edge == nullptr) return pose;
            return nearest_edge_point;
        }

        // find nearest node from the previous pose
        Point2 nearest_edge_point;
        Edge* edge = nullptr;
        if(m_localmap.isEmpty())
            edge = map->getNearestEdge(mappose_prev, nearest_edge_point);
        else
            edge = m_localmap.getNearestEdge(mappose_prev, nearest_edge_point);
        if (edge == nullptr) return pose;

        // build local branch map from the previous pose
        ID center_nid = edge->node_id1;
        Node* node = map->getNode(edge->node_id1);
        if (node == nullptr) return pose;
        double d1 = norm(nearest_edge_point - *node);
        if (d1 > edge->length / 2)
        {
            center_nid = edge->node_id2;
        }
        if (m_localmap.isEmpty() || center_nid != m_localmap_center_nid)
        {
            bool ok = getLocalBranchMap(map, center_nid, m_localmap, m_branchmap_search_radius);
            if (!ok) return pose;
            m_localmap_center_nid = center_nid;
        }

        // find nearest edge point from the local map
        edge = m_localmap.getNearestEdge(pose, nearest_edge_point);
        if (edge == nullptr) return pose;
        Pose2 pose_new = nearest_edge_point;
        double theta = atan2(pose_new.y - mappose_prev.y, pose_new.x - mappose_prev.x);
        pose_new.theta = cx::trimRad(theta);

        return pose_new;
    }

    /**
     * Estimate best path-projected pose of a given pose and detect out of path
     * @param map Map data
     * @param path Current path to a destination
     * @param pose Current pose or newly estimated robot pose (it is usually an output of EKF localization)
     * @param pose_history Recent trajectory of robot poses
     * @param out_of_path True if out of path is detected
     * @return Estimated best path pose (it can be out-of-path pose in case out-of-path detected)
     */
    Pose2 getPathPose(dg::Map* map, const Path& path, const Pose2& pose, const RingBuffer<Pose2T>& pose_history, bool& out_of_path)
    {
        out_of_path = false;
        if (map == nullptr || map->isEmpty() || path.empty()) return pose;

        // path projection: 현재 위치를 path에 투영
        int path_idx = -1;
        Pose2 path_pose = findNearestPathPose(path, pose, path_idx);
        if (path_idx < 0) return pose;

        // just return path-projected pose in case of initial pose
        if (pose_history.empty())
        {
            m_localmap.removeAll();
            m_localmap_center_nid = 0;
            return path_pose;
        }

        // build local map: 직전 위치의 path 투영점과 연결된 local branch map
        Pose2 pose_prev = pose_history.back();
        int path_idx_prev = -1;
        Pose2 path_pose_prev = findNearestPathPose(path, pose_prev, path_idx_prev);
        if (path_idx_prev < 0)
        {
            path_pose_prev = path_pose;
            path_idx_prev = path_idx;
        }
        dg::ID center_nid = path.pts[path_idx_prev].node_id;
        if (center_nid <= 0 && path_idx_prev < (int)path.pts.size() - 1) center_nid = path.pts[path_idx_prev + 1].node_id;
        if (m_localmap.isEmpty() || center_nid != m_localmap_center_nid)
        {
            bool ok = getLocalBranchMap(map, center_nid, m_localmap, m_branchmap_search_radius);
            if (!ok) return path_pose;
            m_localmap_center_nid = center_nid;
        }

        // localmap projection: 현재 위치에서 도달 가능한 모든 branch로의 투영점들을 탐색
        std::vector<Pose2> map_poses = findProjectedMapPoses(&m_localmap, pose, m_projection_search_radius, m_error_tolerance);

        // evaluation: 각각의 후보 map 투영점(prj_pts)들에 대한 궤적 정합도 평가
        if (m_enable_debugging_display)
        {
            m_best_align_score = 0;
            m_evalMapPath.clear();
            m_evalPoseHistory.clear();
            m_evalMapPathStartIdx = 0;
        }
        double max_align_score = -1;
        Pose2 best_map_pose;
        for (int i = 0; i < (int)map_poses.size(); i++)
        {
            Pose2 map_pose = map_poses[i];

            double align_score = -1;
            int pose_eval_len = 0;
            bool success = evaluateMapPose(&m_localmap, path, pose_history, map_pose, path_pose, path_idx, pose_eval_len, align_score);
            bool valid_pose = success && (pose_eval_len >= m_min_eval_historylen) && (align_score > m_min_alignscore_gap);
            if (valid_pose && (!out_of_path || align_score > max_align_score))
            {
                max_align_score = align_score;
                out_of_path = true;
                best_map_pose = map_pose;
            }
        }

        if (out_of_path)
        {
            dg::Path new_path;
            bool ok = map->getPath(best_map_pose, path.pts.back(), new_path);
            if (!ok || new_path.empty()) return path_pose;

            int new_path_idx = -1;
            Pose2 pose_new = findNearestPathPose(new_path, best_map_pose, new_path_idx);
            return pose_new;
        }

        return path_pose;
    }

    void reset()
    {
        m_localmap.removeAll();
        m_localmap_center_nid = 0;
        m_best_align_score = -1;
        m_evalMapPath.clear();
        m_evalMapPathStartIdx = -1;
        m_evalPoseHistory.clear();
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
                if (node) continue;

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

            // check search_radius
            double d = norm(*from - p);
            if (d > search_radius) continue;

            // check node projection: 해당 node(from)가 gps의 투영점이 될 수 있는지 조사
            Point2 nv = p - *from;
            bool proj_node = true;
            for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
            {
                Node* to = map->getConnectedNode(&(*from), *it);
                if (to == nullptr) continue;

                Point2 ev = *to - *from;
                double theta = acos(nv.ddot(ev) / (norm(nv) * norm(ev)));
                if (theta < CV_PI / 2)
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
                continue;
            }

            // check edge projection: 해당 edge(from)가 gps의 투영점이 될 수 있는지 조사
            for (auto it = from->edge_ids.begin(); it != from->edge_ids.end(); it++)
            {
                Node* to = map->getConnectedNode(&(*from), *it);
                if (to == nullptr) continue;

                auto dist2 = calcDist2FromLineSeg(*from, *to, p);
                if (dist2.first > search_radius2) continue;

                double d1 = norm(dist2.second - *from);
                double d2 = norm(dist2.second - *to);
                if (d1 > 0 && d2 > 0)
                {
                    Point2 ep = dist2.second;
                    bool already_exist = false;
                    for (int k = 0; k < (int)results.size(); k++)
                    {
                        if (norm(ep - results[k]) <= error_tolerance)
                        {
                            already_exist = true;
                            break;
                        }
                    }
                    if (!already_exist) results.push_back(Pose2(ep));
                }
            }
        }

        return results;
    }

protected:
    // cache memory for speeding up the process
    dg::Map m_localmap;
    ID m_localmap_center_nid = 0;

    /** temporal variables for debugging display */
    double m_best_align_score;
    dg::Path m_evalMapPath;
    int m_evalMapPathStartIdx;
    std::vector<Point2> m_evalPoseHistory;

    /**
     * Evaluate if a given map pose is valid pose<br>
     * A map pose is valid if a path to the map pose is aligned to the pose history better than the current path with large margin.
     * @param map Map data
     * @param path Current path
     * @param pose_history Recent trajectory of robot pose (It can be a history of EKF localization result)
     * @param map_pose A candidate map pose to be evaluated
     * @param path_pose Pose when the robot is assumed on the current path
     * @param path_idx Index of path node where the path_pose belongs to (in the current path)
     * @param pose_eval_len Length of the pose history interval used in the path evaluation
     * @param align_score Computed align score of a givien map pose (align_score = align_cost(path_pose,pose_history) - align_cost(map_pose, pose_history))
     * @return Return True if successful
     */
    bool evaluateMapPose(dg::Map* map, const dg::Path& path, const RingBuffer<Pose2T>& pose_history, const Pose2& map_pose, const Pose2& path_pose, const int path_idx, int& pose_eval_len, double& align_score)
    {
        // check if map pose is on the path
        if (map == nullptr || map->isEmpty() || path_idx < 0) return false;
        if (norm(map_pose - path_pose) <= m_error_tolerance) return false;

        int npath = (int)path.pts.size();

        // set evaluation path interval: [path_pose - m_eval_pathlen_backward, path_pose + m_eval_pathlen_forward]
        int path_idx1 = path_idx;
        double len_backward = norm(Point2(path.pts[path_idx]) - path_pose);
        while (path_idx1 > 0 && len_backward < m_eval_pathlen_backward)
        {
            path_idx1--;
            len_backward += norm(path.pts[path_idx1] - path.pts[path_idx1 + 1]);
        }
        int path_idx2 = path_idx;
        double len_forward = -norm(Point2(path.pts[path_idx]) - path_pose);
        while (path_idx2 < npath - 1 && len_forward < m_eval_pathlen_forward)
        {
            path_idx2++;
            len_forward += norm(path.pts[path_idx2 - 1] - path.pts[path_idx2]);
        }

        // expand path interval by one unit at both ends (to check path branching)
        if (path_idx1 > 0) path_idx1--;
        if (path_idx2 < npath - 1) path_idx2++;

        // find local path from path_idx1 to the map_pose
        dg::Path eval_path;
        bool ok = map->getPath(path.pts[path_idx1], map_pose, eval_path);
        if (!ok || eval_path.empty()) return false;

        // check if the eval_path is valid (it should be branched in the evaluation interval)
        int max_match_k = -1;
        for (int k = 0; k < (int)eval_path.pts.size(); k++)
        {
            if (path_idx1 + k > path_idx2) break;
            if (eval_path.pts[k].node_id != path.pts[path_idx1 + k].node_id) break;
            max_match_k = k;
        }
        bool path_valid = (path_idx1 == 0 && max_match_k == 0 || max_match_k > 0 && path_idx1 + max_match_k < path_idx2);
        if (!path_valid) return false;

        // Set evaluation interval of path (path: current path, evalpath: newly searched path to map pose)
        // - start_path_idx: original path에서의 evaluation start index
        // - start_evalpath_idx: new path에서의 evaluation start index
        int start_path_idx = path_idx1 + max_match_k;
        if (start_path_idx < 0) start_path_idx = 0;
        if (path_idx < start_path_idx) start_path_idx = path_idx; // extend eval interval so that it includes path_pose
        int start_evalpath_idx = start_path_idx - path_idx1;
        if (start_evalpath_idx < 0) start_evalpath_idx = 0;

        // Determine evaluation interval of pose_history
        // - find the latest point that belongs to the original path
        // - 기존 path의 evaluation 시작점 edge segment와 가장 가까운 pose_history 점 선택
        int pose_start_k = 0;
        double d_min = DBL_MAX;
        Point2 path_p = path.pts[start_path_idx];
        for (int k = 0; k < pose_history.data_count(); k++)
        {
            Point2 pose_p = pose_history[k];
            double d = norm(pose_p - path_p);
            if (d < d_min)
            {
                d_min = d;
                pose_start_k = k;
            }
        }

        // calc. pose_history - eval_map_path alignment score
        double align_cost_path = 0;
        double align_cost_map = 0;
        align_cost_path = computeAlignCost(path, start_path_idx, path_idx, path_pose, pose_history, pose_start_k);
        align_cost_map = computeAlignCost(eval_path, start_evalpath_idx, (int)eval_path.pts.size() - 1, map_pose, pose_history, pose_start_k);

        pose_eval_len = pose_history.data_count() - pose_start_k;
        align_score = align_cost_path - align_cost_map;

        // save current evaluation set for debugging display
        if (m_enable_debugging_display && pose_eval_len >= m_min_eval_historylen && align_score > m_best_align_score)
        {
            m_evalMapPath = eval_path;
            m_evalMapPathStartIdx = start_evalpath_idx;
            m_evalPoseHistory.resize(pose_eval_len);
            int k = 0;
            for (int i = pose_start_k; i < pose_history.data_count(); i++, k++)
            {
                m_evalPoseHistory[k] = pose_history[i];
            }
            m_best_align_score = align_score;
        }

        return true;
    }

    /**
     * Compute align cost between a path interval and a pose history
     * @param path Path to evaluate
     * @param path_idx1 Starting index of evaluation path interval
     * @param path_idx2 Ending index of evaluation path interval
     * @param projected_path_pose Projected pose on the evaluation path
     * @param pose_history Recent trajectory of robot pose
     * @param pose_idx1 Index of starting point of evaluation gps interval
     * @param pose_idx2 Index of ending point of evaluation gps interval
     * @return Computed align cost
     */
    double computeAlignCost(const dg::Path& path, int path_idx1, int path_idx2, const Pose2& projected_path_pose, const RingBuffer<Pose2T>& pose_history, int pose_idx1 = 0, int pose_idx2 = -1)
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

            align_cost += norm(pose_history[k] - path_point);
        }
        double average_align_cost = align_cost / (pose_idx2 - pose_idx1 + 1);
        double length_cost = fabs(path_len_total - pose_len_total);

        return (average_align_cost + m_length_align_weight * length_cost);
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
        Pose2 path_pose = min_dist2.second;
        if (min_path_idx < (int)path.pts.size() - 1)
        {
            path_pose = min_dist2.second;
            double dx = path.pts[min_path_idx + 1].x - path.pts[min_path_idx].x;
            double dy = path.pts[min_path_idx + 1].y - path.pts[min_path_idx].y;
            path_pose.theta = cx::trimRad(atan2(dy, dx));
        }
        path_idx = min_path_idx;

        return path_pose;
    }

}; // End of 'PathProjector'

} // End of 'dg'


#endif // End of '__PATH_PROJECTION_HPP__'
