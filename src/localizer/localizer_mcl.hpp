#ifndef __LOCALIZER_MCL_HPP__
#define __LOCALIZER_MCL_HPP__

#include "localizer/localizer.hpp"

namespace dg
{
    struct Particle
    {
        Node* start_node = nullptr;
        Edge* edge = nullptr;

        double dist = 0;    // distance from start node
        double head = 0;    // relative heading w.r.t. edge direction
        double weight = 1.0;

        int centroid_idx = -1;
    };

    struct OdometryData
    {
        double x, y, theta;         // odometry x,y,theta
        double dist, dtheta;        // odometry delta
        Timestamp t;                // timestamp

        Pose2 pose_mcl;
        double dist_accumulated = 0;
        double dtheta_accumulated = 0;

        OdometryData() { dist = dtheta = t = 0; }
        OdometryData(const Pose2& p, double _x, double _y, double _theta, double _dist, double _dtheta, Timestamp ts) : pose_mcl(p), x(_x), y(_y), theta(_theta), dist(_dist), dtheta(_dtheta), t(ts) {}
    };

    class DGLocalizerMCL : public DGLocalizer
    {
    protected:
        int m_particle_numbers = 400;
        double m_initial_error_bound = 50;      // meter
        int m_odometry_history_size = 1000;
        double m_odometry_linear_std_err = 1;
        double m_odometry_angular_std_err = cx::cvtDeg2Rad(5);
        double m_gps_pos_sigma = 30;
        double m_gps_theta_sigma = cx::cvtDeg2Rad(50);
        double m_odometry_save_interval = 1;
        double m_corner_cosine = cos(CV_PI / 4);

        Pose2 m_prev_gps_pose;
        Timestamp m_prev_gps_time = -1;

        Pose2 m_pose_mcl;
        int m_mixture_centroid_n = 5;
        std::vector<Point2> m_mixture_centroids;

        std::vector<Particle> m_particles;
        bool m_mcl_initialized = false;
        RingBuffer<OdometryData> m_odometry_history;

    public:
        DGLocalizerMCL(std::string baselocalizer_name = "EKFLocalizer")
        {
            initialize(nullptr, baselocalizer_name);

            srand((unsigned)time(NULL));
        }

        virtual bool applyGPS(const Point2& xy, Timestamp time = -1, double confidence = -1)
        {
            if (m_enable_stop_filtering && m_odometry_active && m_odometry_stabilized && m_robot_stopped) return false;

            cv::AutoLock lock(m_mutex);
            Point2 smoothed_xy = xy;
            if (m_enable_gps_smoothing) smoothed_xy = getSmoothedGPS(xy, time);
            if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (!m_ekf->applyGPS(smoothed_xy, time, confidence)) return false;
            saveObservation(ObsData::OBS_GPS, smoothed_xy, time, confidence);
            saveEKFState(m_ekf, time);
            m_pose = m_ekf->getPose();
            m_timestamp = time;

            if (!m_mcl_initialized && m_ekf->isPoseStabilized())
            {
                initialize_mcl(m_ekf->getPose());
                m_prev_gps_pose = m_ekf->getPose();
                m_prev_gps_time = time;
                return true;
            }

            if (m_mcl_initialized)
            {
                if (!m_odometry_active) predictParticles(m_ekf->getPose(), m_prev_gps_pose, time - m_prev_gps_time, m_robot_stopped);
                evaluateParticlesHistory();
                evaluateParticlesPose(m_ekf->getPose());
                resampleParticles();
                if (!m_odometry_active) m_pose_mcl = estimateMclPose();
                m_pose = m_pose_mcl;
            }
            m_prev_gps_pose = m_ekf->getPose();
            m_prev_gps_time = time;

            return true;
        }

        virtual bool applyOdometry(Pose2 odometry_pose, Timestamp time = -1, double confidence = -1)
        {
            // initialization
            if (!m_odometry_active)
            {
                m_initial_odometry_pose = odometry_pose;
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                m_odometry_active = true;
            }

            // check odometry displacement
            Point2 initial_displacement(odometry_pose.x - m_initial_odometry_pose.x, odometry_pose.y - m_initial_odometry_pose.y);
            if (!m_odometry_stabilized && norm(initial_displacement) < m_odometry_stabilization_d)
            {
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                return false;
            }
            m_odometry_stabilized = true;

            if (!isPoseStabilized()) return false;

            // check odometry velocity
            Point2 odo_delta(odometry_pose.x - m_prev_odometry_pose.x, odometry_pose.y - m_prev_odometry_pose.y);
            double odo_dt = time - m_prev_odometry_time;
            double odo_velocity = (odo_dt > 0) ? norm(odo_delta) / odo_dt : norm(odo_delta);
            if (odo_velocity <= m_stop_min_velocity)
            {
                if (m_stopped_time < 0)
                {
                    m_stopped_time = time;
                }
                double stop_period = time - m_stopped_time;
                if (!m_robot_stopped && stop_period >= m_stop_min_period)
                {
                    m_robot_stopped = true;
                }
            }
            else
            {
                m_robot_stopped = false;
                m_stopped_time = -1;
            }

            // update particle
            if (m_mcl_initialized)
            {
                predictParticles(odometry_pose, m_prev_odometry_pose, time - m_prev_odometry_time, m_robot_stopped);
                m_pose_mcl = estimateMclPose();
            }

            // save odometry
            if (m_odometry_history.empty()) m_odometry_history.push_back(OdometryData(m_pose_mcl, odometry_pose.x, odometry_pose.y, odometry_pose.theta, 0, 0, time));
            OdometryData odo = m_odometry_history.back();
            double dx = odometry_pose.x - odo.x;
            double dy = odometry_pose.y - odo.y;
            double d = sqrt(dx * dx + dy * dy);
            double dtheta = cx::trimRad(odometry_pose.theta - odo.theta);
            if (d >= m_odometry_save_interval)
            {
                OdometryData odo_new(m_pose_mcl, odometry_pose.x, odometry_pose.y, odometry_pose.theta, d, dtheta, time);
                odo_new.dist_accumulated = odo.dist + d;
                odo_new.dtheta_accumulated = odo.dtheta + dtheta;
                m_odometry_history.push_back(odo_new);
            }

            cv::AutoLock lock(m_mutex);
            if (m_ekf_pose_history.empty()) return false;
            if (!m_ekf->applyOdometry(odometry_pose, time, confidence)) return false;
            saveObservation(ObsData::OBS_ODO, odometry_pose, time, confidence);
            saveEKFState(m_ekf, time);

            m_prev_odometry_pose = odometry_pose;
            m_prev_odometry_time = time;

            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        const std::vector<Particle>& getParticles() { return m_particles; }
        Pose2 getPoseMCL() { return m_pose_mcl; }

    protected:
        void initialize_mcl(const Pose2& pose)
        {
            Point2 p(pose.x, pose.y);
            create_uniform_particles(p, m_initial_error_bound, m_particles);
            m_odometry_history.resize(m_odometry_history_size);
            m_mcl_initialized = true;
        }

        virtual bool applyPathLocalizer(Pose2 pose, Timestamp timestamp)
        {
            m_pose = pose;
            m_timestamp = timestamp;
            if (!m_enable_path_projection && !m_enable_map_projection) return true;
            if (m_shared == nullptr) return false;

            bool out_of_path = false;
            Map* map = m_shared->getMapLocked();
            if (map)
            {
                Path path = m_shared->getPath();
                if (!path.empty() && m_enable_path_projection) m_pose = getPathPose(map, &path, pose, m_ekf_pose_history, m_projected_pose_history, out_of_path);
                else if (m_enable_map_projection) m_pose = getMapPose(map, pose, m_ekf_pose_history, m_projected_pose_history);
            }
            m_shared->releaseMapLock();
            if (out_of_path) m_shared->procOutOfPath(m_pose);
            m_projected_pose_history.push_back(m_pose);

            return true;
        }

        void create_uniform_particles(const Point2 p, double radius, std::vector<Particle>& particles)
        {
            particles.resize(m_particle_numbers);
            int idx = 0;

            Map* map = m_shared->getMapLocked();
            if (map)
            {
                std::vector<Edge*> edges = map->getNearEdges(p, radius);
                double total_length = 0;
                for (size_t i = 0; i < edges.size(); i++)
                {
                    total_length += edges[i]->length;
                }

                for (size_t i = 0; i < edges.size(); i++)
                {
                    int cnt = (int)(m_particle_numbers * edges[i]->length / total_length + 0.5);
                    int k = 0;
                    while (k < cnt)
                    {
                        particles[idx].start_node = map->getNode(edges[i]->node_id1);
                        particles[idx].edge = edges[i];
                        particles[idx].dist = (double)rand() * edges[i]->length / RAND_MAX;
                        particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                        idx++;
                        k++;
                        if (k>=cnt || idx >= m_particle_numbers) break;

                        particles[idx].start_node = map->getNode(edges[i]->node_id2);
                        particles[idx].edge = edges[i];
                        particles[idx].dist = (double)rand() * edges[i]->length / RAND_MAX;
                        particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                        idx++;
                        k++;
                        if (k>=cnt || idx >= m_particle_numbers) break;

                    }
                    if (idx >= m_particle_numbers) break;
                }

                while (idx < m_particle_numbers)
                {
                    Edge* edge = edges[rand() % ((int)(edges.size()))];

                    particles[idx].start_node = map->getNode(edge->node_id1);
                    particles[idx].edge = edge;
                    particles[idx].dist = (double)rand() * edge->length / RAND_MAX;
                    particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                    idx++;
                    if (idx >= m_particle_numbers) break;

                    particles[idx].start_node = map->getNode(edge->node_id2);
                    particles[idx].edge = edge;
                    particles[idx].dist = (double)rand() * edge->length / RAND_MAX;
                    particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                    idx++;
                    if (idx >= m_particle_numbers) break;
                }
            }
            m_shared->releaseMapLock();
        }

        void predictParticles(Pose2 cur, Pose2 prev, double dtime, bool robot_stopped)
        {
            int N = (int)m_particles.size();
            std::vector<double> lin_noise = create_normal_numbers(N, m_odometry_linear_std_err);
            std::vector<double> ang_noise = create_normal_numbers(N, m_odometry_angular_std_err);

            double dx = cur.x - prev.x;
            double dy = cur.y - prev.y;
            double d = sqrt(dx * dx + dy * dy);
            double dtheta = cx::trimRad(cur.theta - prev.theta);
            double linear_velocity = (dtime > 0) ? d / dtime : 0;
            double angular_modifier = 1;
            if (linear_velocity > 0.5) angular_modifier = 0.9;
            for (int i = 0; i < N; i++)
            {
                double di = d + lin_noise[i];
                double dthi = dtheta + ang_noise[i];
                if (robot_stopped)
                {
                    di = d;
                    dthi = dtheta;
                }
                m_particles[i].dist += di*cos(m_particles[i].head + dthi/2);
                if (m_particles[i].dist < 0) m_particles[i].dist = 0;
                m_particles[i].head += dthi;
                m_particles[i].head *= angular_modifier;
                m_particles[i].head = cx::trimRad(m_particles[i].head);
                //m_particles[i].head = 0;
            }

            // edge transition
            Map* map = m_shared->getMapLocked();
            int n_added = 0;
            for (int i = 0; i < N; i++)
            {
                if (m_particles[i].dist >= m_particles[i].edge->length)
                {
                    Node* to = map->getConnectedNode(m_particles[i].start_node, m_particles[i].edge->id);
                    if (to == nullptr) continue;

                    Particle first_particle;
                    bool first_saved = false;
                    for (auto it = to->edge_ids.begin(); it != to->edge_ids.end(); it++)
                    {
                        if (*it == m_particles[i].edge->id) continue;
                        Edge* new_edge = map->getEdge(*it);
                        if (new_edge == nullptr) continue;
                        Node* node2 = map->getConnectedNode(to, new_edge->id);
                        if (node2 == nullptr) continue;

                        n_added++;

                        Node* from = m_particles[i].start_node;
                        double edge1_theta = atan2(to->y - from->y, to->x - from->x);
                        double edge2_theta = atan2(node2->y - to->y, node2->x - to->x);
                        double new_head = edge1_theta + m_particles[i].head - edge2_theta;

                        Particle particle;
                        particle.start_node = to;
                        particle.edge = new_edge;
                        particle.dist = m_particles[i].dist - m_particles[i].edge->length;
                        particle.head = cx::trimRad(new_head);
                        //particle.head = 0;
                        particle.weight = m_particles[i].weight;
                        particle.centroid_idx = m_particles[i].centroid_idx;

                        if (!first_saved)
                        {
                            first_particle = particle;
                            first_saved = true;
                        }
                        else
                        {
                            m_particles.push_back(particle);
                        }
                    }
                    if(first_saved) m_particles[i] = first_particle;
                }
            }
            m_shared->releaseMapLock();
        }

        void evaluateParticlesPose(Pose2 pose)
        {
            int N = (int)m_particles.size();

            Map* map = m_shared->getMapLocked();
            double w_sum = 0;
            double err_theta_s = 0;
            double head_s = 0;
            double d_s = 0;
            for (int i = 0; i < N; i++)
            {
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Pose2 pose_m = *from + m_particles[i].dist * v / edge->length;
                pose_m.theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);

                double dx = pose.x - pose_m.x;
                double dy = pose.y - pose_m.y;
                double err_d = sqrt(dx * dx + dy * dy);
                double err_theta = cx::trimRad(pose.theta - pose_m.theta);
                err_theta_s += fabs(err_theta);
                head_s += fabs(m_particles[i].head);
                d_s += m_particles[i].dist;
                double pdf1 = exp(-err_d * err_d / (2 * m_gps_pos_sigma * m_gps_pos_sigma)) / (m_gps_pos_sigma * sqrt(2 * CV_PI));
                double pdf2 = exp(-err_theta * err_theta / (2 * m_gps_theta_sigma * m_gps_theta_sigma)) / (m_gps_theta_sigma * sqrt(2 * CV_PI));

                m_particles[i].weight *= (pdf1 * pdf2);
                w_sum += m_particles[i].weight;
            }
            m_shared->releaseMapLock();

            for (int i = 0; i < N; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

        double alignProbPath2Odometry(const std::vector<PathNode>& path_pts, const std::vector<int>& path_corner_pts_index, const std::vector<double>& path_len_cumulated, RingBuffer<OdometryData>& odometry, Particle& particle)
        {
            // path_pts, path_len_cumulated: 출발점부터 particle이 속한 edge의 start node까지의 path
            // particle 위치는 path에 포함되지 않음에 주의!!

            //double m_odometry_save_interval = 1;

            double odo_len_total = odometry.back().dist_accumulated;
            double path_len_total = path_len_cumulated.back() + particle.dist;

            // particle pose
            Map* map = m_shared->getMap();
            Node* from = particle.start_node;
            Edge* edge = particle.edge;
            Node* to = map->getConnectedNode(from, edge->id);
            Point2 v = *to - *from;
            Point2 particle_pos = *from + particle.dist * v / edge->length;

            // theta correction: 오도메트리 시점 종점 거리가 path의 시점 종점 거리와 일치
            double path_distance = norm(path_pts[0] - particle_pos);
            Point2 odo_start(odometry[0].x, odometry[0].y);
            Point2 odo_end(odometry.back().x, odometry.back().y);
            double odo_distance = norm(odo_end - odo_start);

            for (size_t k = 0; k < path_corner_pts_index.size(); k++)
            {
                int idx = path_corner_pts_index[k];
                Point2 p = path_pts[idx];
                double ratio = path_len_cumulated[idx] / path_len_total;
                double odo_len_cumulated = ratio * odo_len_total;
            }


            return 1;
        }

        void evaluateParticlesHistory()
        {
            int N = (int)m_particles.size();
            if (m_odometry_history.data_count() < 10) return;

            // particle paths
            std::vector<ID> nid_list;
            std::vector<dg::Path> path_list;
            std::vector<std::vector<int>> corner_pts_index_list;
            std::vector<std::vector<double>> cumulated_path_length_list;

            Pose2 start = m_odometry_history[0].pose_mcl;
            Map* map = m_shared->getMap();
            for (int i = 0; i < N; i++)
            {
                ID nid = m_particles[i].start_node->id;
                bool found = false;
                for (auto it = nid_list.begin(); it != nid_list.end(); it++)
                {
                    if (*it == nid)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    dg::Path path;
                    bool ok = map->getPath(start, *m_particles[i].start_node, path);
                    if (!ok || path.empty()) continue;
                    int n_pts = (int)(path.pts.size());
                    if (n_pts < 3) continue;

                    nid_list.push_back(nid);
                    path_list.push_back(path);

                    // corner points & total path length
                    Point2 p1 = path.pts[0];
                    double path_len_total = 0;
                    std::vector<int> corner_pts_index;
                    std::vector<double> cumulated_path_length;
                    for (int k = 1; k < n_pts; k++)
                    {
                        Point2 p2 = path.pts[k];
                        double segment_len = norm(p2 - p1);
                        path_len_total += segment_len;
                        if (k < n_pts - 1)
                        {
                            Point2 v1 = p2 - p1;
                            Point2 v2 = path.pts[k + 1] - p2;
                            double cos_theta = v1.ddot(v2) / (norm(v1) * norm(v2));
                            if (cos_theta < m_corner_cosine)
                            {
                                corner_pts_index.push_back(k);
                                cumulated_path_length.push_back(path_len_total);
                            }
                        }
                        p1 = p2;
                    }
                    corner_pts_index_list.push_back(corner_pts_index);
                    cumulated_path_length_list.push_back(cumulated_path_length);
                }
            }

            double w_sum = 0;
            for (int i = 0; i < N; i++)
            {
                // particle pose
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Pose2 pose_particle = *from + m_particles[i].dist * v / edge->length;

                // path from start to particle pose
                ID nid = m_particles[i].start_node->id;
                int found_idx = -1;
                for (int k = 0; k<(int)nid_list.size(); k++)
                {
                    if (nid_list[k] == nid)
                    {
                        found_idx = k;
                        break;
                    }
                }
                if (found_idx < 0) continue;

                double w = alignProbPath2Odometry(path_list[found_idx].pts, corner_pts_index_list[found_idx], cumulated_path_length_list[found_idx], m_odometry_history, m_particles[i]);
                m_particles[i].weight *= w;
                w_sum += w;
            }

            for (int i = 0; i < N; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

        Pose2 estimateMclPose()
        {
            Map* map = m_shared->getMapLocked();

            int N = (int)m_particles.size();
            int K = m_mixture_centroid_n;
            if (m_mixture_centroids.empty())
            {
                int i = 0;
                std::vector<int> index;
                while (i < K)
                {
                    int idx = rand() % N;
                    bool duplicate = false;
                    for (auto itr = index.begin(); itr != index.end(); itr++)
                    {
                        if (idx == *itr)
                        {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate)
                    {
                        index.push_back(idx);
                        i++;
                    }
                }
                for (int i = 0; i < K; i++)
                {
                    Node* from = m_particles[index[i]].start_node;
                    Edge* edge = m_particles[index[i]].edge;
                    Node* to = map->getConnectedNode(from, edge->id);
                    Point2 v = *to - *from;
                    Point2 pos_m = *from + m_particles[index[i]].dist * v / edge->length;
                    m_mixture_centroids.push_back(pos_m);
                }
            }

            Point2 mean_p(0, 0);
            double mean_theta = 0;
            double w_sum = 0;
            for (int i = 0; i < N; i++)
            {
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Point2 pos_m = *from + m_particles[i].dist * v / edge->length;          

                mean_p += (pos_m * m_particles[i].weight);
                mean_theta += (atan2(v.y, v.x) + m_particles[i].head) * m_particles[i].weight;
                w_sum += m_particles[i].weight;
            }
            mean_p /= w_sum;
            mean_theta /= w_sum;
            mean_theta = cx::trimRad(mean_theta);
            m_shared->releaseMapLock();

            Point2 ep;
            map->getNearestEdge(mean_p, ep);

            return Pose2(ep, mean_theta);
        }

        void resampleParticles()
        {
            int N = (int)m_particles.size();
            if (N < 1) return;

            std::vector<double> cumulative_sum;
            cumulative_sum.resize(N);
            cumulative_sum[0] = m_particles[0].weight;
            for (int i = 1; i < N; i++)
            {
                cumulative_sum[i] = cumulative_sum[i - 1] + m_particles[i].weight;
            }

            // resampling
            int M = m_particle_numbers;
            if (M > N) M = N;
            std::vector<int> indexes;
            indexes.resize(M);
            for (int i = 0; i < M; i++)
                indexes[i] = 0;

            std::vector<double> positions;
            positions.resize(M);
            for (int i = 0; i < M; i++)
            {
                positions[i] = (i + (double)rand() / ((double)RAND_MAX + 1)) / M;
            }

            int i = 0;
            int j = 0;
            while (i < M && j < N)
            {
                if (positions[i] < cumulative_sum[j])
                {
                    indexes[i] = j;
                    i++;
                }
                else
                    j++;
            }
            std::vector<Particle> tmp;
            tmp.resize(M);
            double w_sum = 0;
            for (int i = 0; i < M; i++)
            {
                tmp[i] = m_particles[indexes[i]];
                w_sum += tmp[i].weight;
            }

            m_particles = tmp;
            for (int i = 0; i < M; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

        double create_normal_number(double sigma = 1)
        {
            double s = 0;
            for (int k = 0; k < 12; k++)
            {
                s += (double)rand() / RAND_MAX * 2 - 1;
            }
            return (s / 2) * sigma;
        }

        std::vector<double> create_normal_numbers(int N, double sigma = 1)
        {
            std::vector<double> samples;
            samples.resize(N);
            for (int i = 0; i < N; i++)
            {
                double s = 0;
                for (int k = 0; k < 12; k++)
                {
                    s += ((double)rand() / RAND_MAX) * 2 - 1;
                }
                samples[i] = (s / 2) * sigma;
            }
            return samples;
        }

    }; // End of 'DGLocalizerMCL'


} // End of 'dg'

#endif // End of '__LOCALIZER_MCL_HPP__'
