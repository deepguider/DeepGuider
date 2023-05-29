#ifndef __LOCALIZER_MCL_HPP__
#define __LOCALIZER_MCL_HPP__

#include "localizer/localizer.hpp"

namespace dg
{
    struct Particle
    {
        Particle() {}
        Particle(Edge* e, Node* sn, double d, double h, double w)
        : edge(e), start_node(sn), dist(d), head(h), weight(w)
        {
        }
        Edge* edge = nullptr;
        Node* start_node = nullptr;

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
        ID eid_mcl = 0;
        double dist_accumulated = 0;
        double cornerness = 0;

        OdometryData() { dist = dtheta = t = 0; }
        OdometryData(const Pose2& p, double _x, double _y, double _theta, double _dist, double _dtheta, Timestamp ts) : pose_mcl(p), x(_x), y(_y), theta(_theta), dist(_dist), dtheta(_dtheta), t(ts) {}
    };

    class DGLocalizerMCL : public DGLocalizer
    {
    protected:
        int m_particle_numbers = 500;
        double m_resample_radius = 10;      // meter
        double m_resample_delta = 2.0;      // meter
        int m_eval_history_length = 50;     // meter
        int m_odometry_history_size = 1000;
        double m_odometry_save_interval = 1;
        double m_path_particle_weight = 1.2;
        bool m_enable_icp = true;
        int m_max_icp_itr = 5;
        double m_path_corner_thr = cx::cvtDeg2Rad(40);
        bool m_enable_odo_theta_correction = true;
        double m_mcl_min_update_interval = 1;       // seconds
        double m_edge_heading_decay = 0.9;

        double m_odometry_linear_std_err = 0.5;                 // 0.5 meter
        double m_odometry_angular_std_err = cx::cvtDeg2Rad(1);  // 1 degree

        double m_gps_pos_sigma = 20;
        double m_gps_theta_sigma = cx::cvtDeg2Rad(100);
        double m_poi_max_error = 10;        // meter
        double m_poi_pos_sigma = 5;
        double m_vps_pos_sigma = 20;
        double m_odo_align_sigma = 10;
        double m_odo_align_theta_sigma = cx::cvtDeg2Rad(40);
        double m_pdf_additive_min = 0.05;

        Pose2 m_prev_gps_pose;
        Timestamp m_prev_gps_time = -1;
        Pose2 m_pose_mcl;
        ID m_eid_mcl;
        bool m_mcl_pose_valid = false;
        Timestamp m_mcl_last_update_time = -1;  // seconds

        std::vector<double> m_odo_dtheta_hist;
        std::vector<int> m_odo_dtheta_hist_cnt;
        double m_odo_dtheta_hist_angular_res = 1;   // degree
        int m_odo_dtheta_hist_bins = 3;             // should be odd number (-2~-1, -1~+1, +1~+2)

        int m_mixture_centroid_n = 5;
        std::vector<Point2> m_mixture_centroids;

        std::vector<Particle> m_particles;
        bool m_mcl_initialized = false;
        RingBuffer<OdometryData> m_odometry_history;

    public:
        // drawing data for debugging
        Particle m_best_particle;
        double m_best_pdf = -1;
        std::vector<Point2> m_best_odo_pts;
        std::vector<Point2> m_best_path_pts;

    public:
        DGLocalizerMCL(std::string baselocalizer_name = "EKFLocalizer")
        {
            srand((unsigned)time(NULL));
            initialize(nullptr, baselocalizer_name);
            m_odometry_history.resize(m_odometry_history_size);
            initialize_odo_theta_correction();
        }

        virtual void setPose(const Pose2 pose, Timestamp time = -1, bool reset_velocity = true, bool reset_cov = true)
        {
            DGLocalizer::setPose(pose, time, reset_velocity, reset_cov);
            cv::AutoLock lock(m_mutex);
            m_odometry_history.resize(m_odometry_history_size);
            initialize_odo_theta_correction();
            reset_particles(pose, m_particles, m_particle_numbers);

            Point2 ep;
            Map* map = m_shared->getMap();
            Edge* edge = map->getNearestEdge(pose, ep);
            m_pose_mcl = Pose2(ep, pose.theta);
            m_eid_mcl = edge->id;

            m_mcl_initialized = true;
        }

        virtual bool applyGPS(const Point2& xy, Timestamp time = -1, double confidence = -1)
        {
            printf("[GPS] x=%.1lf, y=%.1lf, timestamp = %.1lf\n", xy.x, xy.y, time);

            if (m_gps_deactivated) return false;
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

            if (!m_mcl_initialized)
            {
                if (m_ekf->isPoseStabilized() && m_odometry_stabilized)
                {
                    initialize_mcl(m_ekf->getPose());
                }
                m_pose_mcl = m_ekf->getPose();
                m_prev_gps_pose = m_ekf->getPose();
                m_prev_gps_time = time;
                return true;
            }

            if (m_mcl_initialized)
            {
                if (!m_odometry_active)
                {
                    predictParticles(m_ekf->getPose(), m_prev_gps_pose, time - m_prev_gps_time, m_robot_stopped);
                }
                if (!m_robot_stopped)
                {
                    evaluateParticles(m_ekf->getPose(), true, false);
                    resampleParticles();
                    estimateMclPose(m_pose_mcl, m_eid_mcl);
                    m_mcl_pose_valid = true;
                    m_mcl_last_update_time = time;
                }
                m_pose = m_pose_mcl;
            }
            m_prev_gps_pose = m_ekf->getPose();
            m_prev_gps_time = time;

            return applyPathLocalizerMCL(m_pose, time);
        }

        virtual bool applyPOI(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false)
        {
            // check poi error boundary
            double poi_theta = m_pose_mcl.theta + relative.ang;
            Point2 poi_xy;
            poi_xy.x = m_pose_mcl.x + relative.lin * cos(poi_theta);
            poi_xy.y = m_pose_mcl.y + relative.lin * sin(poi_theta);
            double err_d = norm(clue_xy - poi_xy);
            if (err_d > m_poi_max_error) return false;

            if (!m_mcl_initialized)
                return DGLocalizer::applyPOI(clue_xy, relative, time, confidence, use_relative_model);

            Pose2 before = m_pose_mcl;

            cv::AutoLock lock(m_mutex);
            evaluateParticlesPOI(clue_xy, relative);
            resampleParticles();
            estimateMclPose(m_pose_mcl, m_eid_mcl);
            m_mcl_pose_valid = true;
            m_pose = m_pose_mcl;
            m_timestamp = time;

            Pose2 after = m_pose_mcl;
            //printf("\n[MCL-PPOI] before: x=%.1lf, y=%.1lf, after: x=%.1lf, y=%.1lf\n\n", before.x, before.y, after.x, after.y);

            return applyPathLocalizerMCL(m_pose, time);
        }

        virtual bool applyVPS(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false)
        {
            if (!m_mcl_initialized)
                return DGLocalizer::applyVPS(clue_xy, relative, time, confidence, use_relative_model);

            cv::AutoLock lock(m_mutex);
            if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
            {
                if (!backtrackingEKF(time, ObsData(ObsData::OBS_VPS, clue_xy, relative, time, confidence))) return false;
                return applyPathLocalizer(m_ekf->getPose(), time);
            }
            else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (!m_ekf->applyVPS(clue_xy, relative, time, confidence, use_relative_model)) return false;
            saveObservation(ObsData::OBS_VPS, clue_xy, relative, time, confidence);
            saveEKFState(m_ekf, time);

            evaluateParticlesVPS(clue_xy, relative);
            resampleParticles();
            estimateMclPose(m_pose_mcl, m_eid_mcl);
            m_pose = m_pose_mcl;
            m_timestamp = time;

            return applyPathLocalizerMCL(m_pose, time);
        }

        virtual bool applyOdometry(Pose2 odometry_pose, Timestamp time = -1, double confidence = -1)
        {
            cv::AutoLock lock(m_mutex);
            // initialization
            if (!m_odometry_active)
            {
                m_initial_odometry_pose = odometry_pose;
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                m_odometry_active = true;
            }

            // check odometry velocity
            Point2 odo_delta(odometry_pose.x - m_prev_odometry_pose.x, odometry_pose.y - m_prev_odometry_pose.y);
            double odo_dt = time - m_prev_odometry_time;
            double odo_velocity = (odo_dt > 0) ? norm(odo_delta) / odo_dt : norm(odo_delta);
            if (odo_velocity <= m_stop_min_velocity)
            {
                if (m_stopped_time < 0) m_stopped_time = time;
                double stop_period = time - m_stopped_time;
                if (!m_robot_stopped && stop_period >= m_stop_min_period) m_robot_stopped = true;
            }
            else
            {
                m_robot_stopped = false;
                m_stopped_time = -1;
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

            if (!isPoseStabilized())
            {
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                return false;
            }

            // update particle
            if (m_mcl_initialized)
            {
                if (!m_robot_stopped)
                {
                    predictParticles(odometry_pose, m_prev_odometry_pose, time - m_prev_odometry_time, m_robot_stopped);
                    double dt = time - m_mcl_last_update_time;
                    if (dt > m_mcl_min_update_interval)
                    {
                        evaluateParticles(m_ekf->getPose(), false, false);
                        resampleParticles();
                        m_mcl_last_update_time = time;
                    }
                    estimateMclPose(m_pose_mcl, m_eid_mcl);
                    m_pose = m_pose_mcl;
                    m_mcl_pose_valid = true;
                }
            }
            else
            {
                m_pose = m_ekf->getPose();
            }
            m_timestamp = time;

            // save odometry
            if (!m_mcl_pose_valid)
            {
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                return applyPathLocalizerMCL(m_pose, time);
            }

            if (m_odometry_history.empty())
            {
                OdometryData odo_new(m_pose_mcl, odometry_pose.x, odometry_pose.y, odometry_pose.theta, 0, 0, time);
                odo_new.dist_accumulated = 0;
                odo_new.eid_mcl = m_eid_mcl;
                add_new_odometry_data(odo_new);
            }
            OdometryData odo = m_odometry_history.back();
            double dx = odometry_pose.x - odo.x;
            double dy = odometry_pose.y - odo.y;
            double d = sqrt(dx * dx + dy * dy);
            double dtheta = cx::trimRad(odometry_pose.theta - odo.theta);
            if (d >= m_odometry_save_interval)
            {
                OdometryData odo_new(m_pose_mcl, odometry_pose.x, odometry_pose.y, odometry_pose.theta, d, dtheta, time);
                odo_new.dist_accumulated = odo.dist_accumulated + d;
                odo_new.eid_mcl = m_eid_mcl;
                add_new_odometry_data(odo_new);
                if (m_enable_odo_theta_correction) update_odo_theta_hist(dtheta);
            }
            m_prev_odometry_pose = odometry_pose;
            m_prev_odometry_time = time;

            return applyPathLocalizerMCL(m_pose, time);
        }

        const std::vector<Particle>& getParticles()
        {
            return m_particles;
        }

        Pose2 getPoseMCL()
        {
            return m_pose_mcl;
        }

    protected:
        void initialize_mcl(const Pose2& pose)
        {
            create_duplicated_particles(pose, m_particles, m_particle_numbers);
            m_odometry_history.resize(m_odometry_history_size);
            m_mcl_initialized = true;
            printf("\n[Localizer] MCL initialized~~~~~~~!!!\n\n");
        }

        virtual bool applyPathLocalizerMCL(Pose2 pose, Timestamp timestamp)
        {
            if (m_shared == nullptr) return false;

            bool out_of_path = false;
            Map* map = m_shared->getMapLocked();
            if (map)
            {
                Path path = m_shared->getPath();
                if (!path.empty()) out_of_path = checkOutOfPath(map, &path, m_pose_mcl);
            }
            m_shared->releaseMapLock();
            if (out_of_path) m_shared->procOutOfPath(m_pose);

            return true;
        }

        virtual bool applyPathLocalizer(Pose2 pose, Timestamp timestamp)
        {
            return true;
        }

        void create_duplicated_particles(const Pose2 p, std::vector<Particle>& particles, int n)
        {
            particles.resize(n);
            Map* map = m_shared->getMap();
            Point2 ep;
            Edge* edge = map->getNearestEdge(p, ep);
            Node* node1 = map->getNode(edge->node_id1);
            Node* node2 = map->getNode(edge->node_id2);
            double d1 = norm(ep - *node1);
            double d2 = norm(ep - *node2);

            double theta_sigma = cx::cvtDeg2Rad(45);
            std::vector<double> ang_noise = create_normal_numbers(n, theta_sigma);

            for (int i = 0; i < n; i++)
            {
                particles[i].start_node = node1;
                particles[i].edge = edge;
                particles[i].dist = d1;
                particles[i].head = ang_noise[i];
                i++;
                if (i >= n) break;

                particles[i].start_node = node2;
                particles[i].edge = edge;
                particles[i].dist = d2;
                particles[i].head = ang_noise[i];
            }
        }

        void reset_particles(const Pose2 p, std::vector<Particle>& particles, int n)
        {
            particles.resize(n);
            Map* map = m_shared->getMap();
            Point2 ep;
            Edge* edge = map->getNearestEdge(p, ep);
            Node* node1 = map->getNode(edge->node_id1);
            Node* node2 = map->getNode(edge->node_id2);
            double d1 = norm(ep - *node1);
            double d2 = norm(ep - *node2);

            Node* start_node = node1;
            double dist = d1;

            Point2 v = *node2 - *node1;
            double edge_theta = atan2(v.y, v.x);
            double dtheta = cx::trimRad(p.theta - edge_theta);
            if (dtheta > CV_PI / 2 || dtheta < -CV_PI / 2)
            {
                start_node = node2;
                dist = d2;
                edge_theta = cx::trimRad(edge_theta + CV_PI);
                dtheta = cx::trimRad(p.theta - edge_theta);
            }

            for (int i = 0; i < n; i++)
            {
                particles[i].start_node = start_node;
                particles[i].edge = edge;
                particles[i].dist = dist;
                particles[i].head = dtheta;
            }
        }

        void create_uniform_particles(const Point2 p, double radius, std::vector<Particle>& particles, int n)
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
                    int cnt = (int)(n * edges[i]->length / total_length + 0.5);
                    int k = 0;
                    while (k < cnt)
                    {
                        particles[idx].start_node = map->getNode(edges[i]->node_id1);
                        particles[idx].edge = edges[i];
                        particles[idx].dist = (double)rand() * edges[i]->length / RAND_MAX;
                        particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                        idx++;
                        k++;
                        if (k >= cnt || idx >= n) break;

                        particles[idx].start_node = map->getNode(edges[i]->node_id2);
                        particles[idx].edge = edges[i];
                        particles[idx].dist = (double)rand() * edges[i]->length / RAND_MAX;
                        particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                        idx++;
                        k++;
                        if (k >= cnt || idx >= n) break;

                    }
                    if (idx >= n) break;
                }

                while (idx < n)
                {
                    Edge* edge = edges[rand() % ((int)(edges.size()))];

                    particles[idx].start_node = map->getNode(edge->node_id1);
                    particles[idx].edge = edge;
                    particles[idx].dist = (double)rand() * edge->length / RAND_MAX;
                    particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                    idx++;
                    if (idx >= n) break;

                    particles[idx].start_node = map->getNode(edge->node_id2);
                    particles[idx].edge = edge;
                    particles[idx].dist = (double)rand() * edge->length / RAND_MAX;
                    particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                    idx++;
                    if (idx >= n) break;
                }
            }
            m_shared->releaseMapLock();

            double w_sum = 0;
            for (int i = 0; i < (int)particles.size(); i++)
            {
                w_sum += particles[i].weight;
            }
            for (int i = 0; i < (int)particles.size(); i++)
            {
                particles[i].weight /= w_sum;
            }
        }

        int add_uniform_particles(const Pose2 pose, double d_delta, double radius, std::vector<Particle>& particles, double w)
        {
            int added = 0;

            Map* map = m_shared->getMap();
            Point2 ep;
            Edge* edge = map->getNearestEdge(pose, ep);
            if (edge == nullptr || norm(pose - ep) > radius) return added;

            Node* from = map->getNode(edge->node_id1);
            Node* to = map->getNode(edge->node_id2);
            if (from == nullptr || to == nullptr) return added;
            Point2 v = *to - *from;
            double v_theta = atan2(v.y, v.x);
            double d_theta = cx::trimRad(pose.theta - v_theta);
            if(fabs(d_theta)>CV_PI/2)
            {
                from = map->getNode(edge->node_id2);
                to = map->getNode(edge->node_id1);
            }
            double d_from = norm(ep - *from);
            double d_to = norm(ep - *to);

            double forward_acc = 0;
            while(forward_acc <= radius && forward_acc <= d_to)
            {
                particles.push_back(Particle(edge, from, d_from + forward_acc, 0, w));
                added++;
                forward_acc += d_delta;
            }
            double backward_acc = 0;
            while(backward_acc <= radius && backward_acc <= d_from)
            {
                particles.push_back(Particle(edge, from, d_from - backward_acc, 0, w));
                added++;
                backward_acc += d_delta;
            }

            std::list<Node*> open;
            std::map<ID, double> lookup_tmp;  // node_id, acc_distance fromm start
            if (forward_acc <= radius) open.push_back(to);
            if (backward_acc <= radius) open.push_back(from);
            lookup_tmp.insert(std::make_pair(to->id, d_to));
            lookup_tmp.insert(std::make_pair(from->id, -d_from));

            while (!open.empty())
            {
                // pick next node
                Node* node = open.front();
                open.pop_front();
                auto found = lookup_tmp.find(node->id);
                if(found == lookup_tmp.end()) break;
                double d_acc = found->second;

                if(d_acc>0)
                {
                    for (auto it = node->edge_ids.begin(); it != node->edge_ids.end(); it++)
                    {
                        Node* from = node;
                        double forward_acc = (int)(d_acc/d_delta + 1) * d_delta;

                        Edge* e = map->getEdge(*it);
                        if (e == nullptr) continue;
                        Node* to = map->getConnectedNode(node, e->id);
                        if (to == nullptr) continue;

                        // check duplication
                        auto found = lookup_tmp.find(to->id);
                        if (found != lookup_tmp.end()) continue;

                        while(forward_acc <= radius && forward_acc <= d_acc + e->length)
                        {
                            particles.push_back(Particle(e, from, forward_acc - d_acc, 0, w));
                            added++;
                            forward_acc += d_delta;
                        }

                        if (forward_acc <= radius)
                        {
                            open.push_back(to);
                            lookup_tmp.insert(std::make_pair(to->id, d_acc + e->length));
                        }
                    }
                }

                if(d_acc<0)
                {
                    for (auto it = node->edge_ids.begin(); it != node->edge_ids.end(); it++)
                    {
                        const Node* to = node;
                        double backward_acc = (int)(-d_acc/d_delta + 1) * d_delta;

                        Edge* e = map->getEdge(*it);
                        if (e == nullptr) continue;
                        Node* from = map->getConnectedNode(node, e->id);
                        if (from == nullptr) continue;

                        // check duplication
                        auto found = lookup_tmp.find(from->id);
                        if (found != lookup_tmp.end()) continue;

                        while(backward_acc <= radius && backward_acc <= -d_acc + e->length)
                        {
                            particles.push_back(Particle(e, from, e->length - (backward_acc + d_acc), 0, w));
                            added++;
                            backward_acc += d_delta;
                        }

                        if (backward_acc <= radius)
                        {
                            open.push_back(from);
                            lookup_tmp.insert(std::make_pair(from->id, d_acc - e->length));
                        }
                    }
                }
            }
            return added;
        }

        void predictParticles(Pose2 cur, Pose2 prev, double dtime, bool robot_stopped)
        {
            if (dtime < 1e-3) return;

            int N = (int)m_particles.size();
            std::vector<double> lin_noise = create_normal_numbers(N, m_odometry_linear_std_err);
            std::vector<double> ang_noise = create_normal_numbers(N, m_odometry_angular_std_err);

            double dx = cur.x - prev.x;
            double dy = cur.y - prev.y;
            double d = sqrt(dx * dx + dy * dy);
            double dtheta = cx::trimRad(cur.theta - prev.theta);
            double linear_velocity = (dtime > 0) ? d / dtime : 0;
            double angular_modifier = 1;
            if (linear_velocity > 0.5) angular_modifier = m_edge_heading_decay;
            for (int i = 0; i < N; i++)
            {
                double di = d + lin_noise[i];
                double dthi = dtheta + ang_noise[i];
                if (robot_stopped)
                {
                    di = d;
                    dthi = dtheta;
                }
                m_particles[i].dist += di * cos(m_particles[i].head + dthi / 2);
                if (m_particles[i].dist < 0) m_particles[i].dist = 0;
                m_particles[i].head += dthi;
                m_particles[i].head *= angular_modifier;
                m_particles[i].head = cx::trimRad(m_particles[i].head);
            }

            // edge transition
            Map* map = m_shared->getMap();
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
                    if (first_saved) m_particles[i] = first_particle;
                }
            }
        }

        bool estimateMclPose(Pose2& pose, ID& eid)
        {

            int N = (int)m_particles.size();

            // average of theta --> assume they are on unit circle --> average coordinate of unit circle position
            Point2 mean_p(0, 0);
            double mean_x = 0;
            double mean_y = 0;
            double w_sum = 0;
            Map* map = m_shared->getMap();
            for (int i = 0; i < N; i++)
            {
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                if (from==nullptr || edge==nullptr) continue;
                Node* to = map->getConnectedNode(from, edge->id);
                if (to == nullptr) continue;
                Point2 v = *to - *from;
                Point2 pos_m = *from + m_particles[i].dist * v / edge->length;
                double theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);
                mean_x += (cos(theta) * m_particles[i].weight);
                mean_y += (sin(theta) * m_particles[i].weight);
                mean_p.x += (pos_m.x * m_particles[i].weight);
                mean_p.y += (pos_m.y * m_particles[i].weight);
                w_sum += m_particles[i].weight;
            }
            mean_p /= w_sum;
            mean_x /= w_sum;
            mean_y /= w_sum;
            double mean_theta = atan2(mean_y, mean_x);

            Point2 ep;
            Edge* edge = map->getNearestEdge(mean_p, ep);
            pose = Pose2(ep, mean_theta);
            eid = edge->id;

            return true;
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
            if(m_mcl_pose_valid && m_resample_radius > 0)
            {
                int n_resample = add_uniform_particles(m_pose_mcl, m_resample_delta, m_resample_radius, tmp, w_sum / M);
                w_sum += (n_resample * w_sum / M);
            }

            m_particles = tmp;
            for (int i = 0; i < (int)m_particles.size(); i++)
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

        void add_new_odometry_data(const OdometryData& data)
        {
            m_odometry_history.push_back(data);

            // compute cornerness
            int n = m_odometry_history.data_count();
            int cornerness_delta = 5;
            int k = n - 2;
            Point2 v1, v2;
            while (k >= n - 1 - cornerness_delta && k >= n - 1 - k)
            {
                int delta = n - 1 - k;
                v1.x = m_odometry_history[k - delta].x - m_odometry_history[k].x;
                v1.y = m_odometry_history[k - delta].y - m_odometry_history[k].y;
                v2.x = m_odometry_history[k + delta].x - m_odometry_history[k].x;
                v2.y = m_odometry_history[k + delta].y - m_odometry_history[k].y;
                double cos_t = v1.ddot(v2) / (norm(v1) * norm(v2));
                if (v1.cross(v2) > 0)   // right turn
                    m_odometry_history[k].cornerness = -(1 + cos_t);
                else                    // left turn
                    m_odometry_history[k].cornerness = (1 + cos_t);
                k--;
            }
        }

        void evaluateParticles(const Pose2& pose_ekf, bool use_ekf_pose = true, bool print_debug = false)
        {
            std::vector<double> align_err_l1, aligned_theta;
            bool odo_valid = evaluateParticleHistory(align_err_l1, aligned_theta);

            Path path = m_shared->getPath();
            int path_n = (int)path.pts.size();

            int N = (int)m_particles.size();
            Map* map = m_shared->getMapLocked();
            double w_sum = 0;
            double max_pdf = 0;
            for (int i = 0; i < N; i++)
            {
                // particle pose
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Pose2 pose_particle = *from + m_particles[i].dist * v / edge->length;
                pose_particle.theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);

                // path weight
                double path_weight = 1;
                for (size_t k = 0; k < path_n; k++)
                {
                    if (edge->id == path.pts[k].edge_id)
                    {
                        path_weight = m_path_particle_weight;
                        break;
                    }
                }

                // ekf weight
                double pdf_ekf_d = 1;
                double pdf_ekf_theta = 1;
                double gd = 0;
                double gth = 0;
                if (use_ekf_pose && !m_gps_deactivated)
                {
                    double dx = pose_particle.x - pose_ekf.x;
                    double dy = pose_particle.y - pose_ekf.y;
                    gd = sqrt(dx * dx + dy * dy);
                    gth = cx::trimRad(pose_particle.theta - pose_ekf.theta);
                    pdf_ekf_d = exp(-gd * gd / (2 * m_gps_pos_sigma * m_gps_pos_sigma)) + m_pdf_additive_min;
                    pdf_ekf_theta = exp(-gth * gth / (2 * m_gps_theta_sigma * m_gps_theta_sigma)) + m_pdf_additive_min;
                }

                // odometry align weight
                double pdf_odo_d = 1;
                double pdf_odo_theta = 1;
                double od = 0;
                double oth = 0;
                double odo_aligned_theta = 0;
                if (odo_valid)
                {
                    odo_aligned_theta = aligned_theta[i];
                    od = align_err_l1[i];
                    oth = cx::trimRad(pose_particle.theta - aligned_theta[i]);
                    pdf_odo_d = exp(-od * od / (2 * m_odo_align_sigma * m_odo_align_sigma)) + m_pdf_additive_min;
                    pdf_odo_theta = exp(-oth * oth / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma)) + m_pdf_additive_min;
                }

                double pdf = path_weight * (pdf_ekf_d * pdf_ekf_theta * pdf_odo_d * pdf_odo_theta);
                if (pdf > max_pdf) max_pdf = pdf;
                if (print_debug) printf("[%d] w=%lf, gd=%.1lf(%lf), gth=%.1lf(%lf), od=%.1lf(%lf), oth=%.1lf(%lf), pth=%.1lf, alignth=%.1lf\n", i, pdf, gd, pdf_ekf_d, cx::cvtRad2Deg(gth), pdf_ekf_theta, od, pdf_odo_d, cx::cvtRad2Deg(oth), pdf_odo_theta, cx::cvtRad2Deg(pose_particle.theta), cx::cvtRad2Deg(odo_aligned_theta));

                m_particles[i].weight *= pdf;
                w_sum += m_particles[i].weight;
            }
            m_shared->releaseMapLock();
            if (print_debug) printf("[MCL] max_pdf = %lf, w_sum = %lf\n", max_pdf, w_sum);

            for (int i = 0; i < N; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

        bool evaluateParticleHistory(std::vector<double>& align_err_l1, std::vector<double>& aligned_theta)
        {
            // check odometry length
            int n_odo = m_odometry_history.data_count();
            if (n_odo < 5) return false;
            OdometryData odo = m_odometry_history.back();
            if (odo.dist_accumulated < 10) return false;

            int N = (int)m_particles.size();
            align_err_l1.resize(N);
            aligned_theta.resize(N);

            // starting point
            int odo_start_idx = n_odo - 1 - m_eval_history_length;
            if (odo_start_idx < 0) odo_start_idx = 0;
            double odo_length = odo.dist_accumulated - m_odometry_history[odo_start_idx].dist_accumulated;
            Pose2 start_p = m_odometry_history[odo_start_idx].pose_mcl;
            ID start_eid = m_odometry_history[odo_start_idx].eid_mcl;

            // odometry points
            std::vector<Point2> odo_pts;
            odo_pts.resize(n_odo - odo_start_idx);
            for (int i = odo_start_idx; i < n_odo; i++)
            {
                odo_pts[i - odo_start_idx].x = m_odometry_history[i].x;
                odo_pts[i - odo_start_idx].y = m_odometry_history[i].y;
            }

            // theta correction
            if (m_enable_odo_theta_correction) odo_theta_correction_mean(m_odometry_history, odo_start_idx, odo_pts);

            // whitening of odometry points
            Point2 mean_odo_point(0, 0);
            for (int i = odo_start_idx; i < n_odo; i++)
            {
                mean_odo_point += odo_pts[i - odo_start_idx];
            }
            mean_odo_point /= (double)odo_pts.size();
            for (int i = 0; i < (int)odo_pts.size(); i++)
            {
                odo_pts[i] -= mean_odo_point;
            }

            // find particle paths
            std::vector<ID> nid_list;
            std::vector<dg::Path> path_list;
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
                    bool ok = map->getPath(start_p, *m_particles[i].start_node, path);
                    if (!ok || path.empty()) continue;
                    int n_pts = (int)(path.pts.size());
                    if (n_pts < 2) continue;

                    nid_list.push_back(nid);
                    path_list.push_back(path);
                }
            }

            // evaluate particle paths
            double w_sum = 0;
            double best_w = -1;
            m_best_pdf = -1;
            for (int i = 0; i < N; i++)
            {
                // particle pose
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Pose2 pose_particle = *from + m_particles[i].dist * v / edge->length;
                double particle_theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);

                Path path;
                if (m_particles[i].edge->id == start_eid)
                {
                    path.pts.push_back(start_p);
                    path.pts.push_back(pose_particle);
                    double err_l1, odo_aligned_theta;
                    double w = alignCostOdometry(m_odometry_history, odo_start_idx, odo_pts, mean_odo_point, path.pts, particle_theta, err_l1, odo_aligned_theta);
                    align_err_l1[i] = err_l1;
                    aligned_theta[i] = odo_aligned_theta;

                    if (w > best_w)
                    {
                        m_best_particle = m_particles[i];
                        best_w = w;
                    }
                }
                else
                {
                    // path from start to particle pose
                    ID nid = m_particles[i].start_node->id;
                    int found_idx = -1;
                    for (int k = 0; k < (int)nid_list.size(); k++)
                    {
                        if (nid_list[k] == nid)
                        {
                            found_idx = k;
                            break;
                        }
                    }
                    if (found_idx < 0) continue;

                    path_list[found_idx].pts.push_back(pose_particle);
                    double err_l1, odo_aligned_theta;
                    double w = alignCostOdometry(m_odometry_history, odo_start_idx, odo_pts, mean_odo_point, path_list[found_idx].pts, particle_theta, err_l1, odo_aligned_theta);
                    align_err_l1[i] = err_l1;
                    aligned_theta[i] = odo_aligned_theta;

                    if (w > best_w)
                    {
                        m_best_particle = m_particles[i];
                        best_w = w;
                    }
                    path_list[found_idx].pts.pop_back();
                }
            }

            return true;
        }

        double alignCostOdometry(const RingBuffer<OdometryData>& odometry, int odo_start_idx, const std::vector<Point2>& eval_odo_pts, dg::Point2 mean_odo_point, const std::vector<PathNode>& path_pts, double particle_theta, double& err_l1, double& odo_aligned_theta)
        {
            // odometry length
            int n_odo = odometry.data_count();
            double odo_len_total = odometry[n_odo - 1].dist_accumulated - odometry[odo_start_idx].dist_accumulated;

            // path length
            int n_path = (int)path_pts.size();
            double path_len_total = 0;
            for (int i = 1; i < n_path; i++)
            {
                path_len_total += norm(path_pts[i] - path_pts[i - 1]);
            }

            // check path cornerness
            Point2 path_v = path_pts[n_path - 1] - path_pts[n_path - 2];
            double path_last_theta = atan2(path_v.y, path_v.x);
            int corner_path_idx = -1;
            double corner_path_len = norm(path_v);
            for (int i = n_path - 2; i > 0; i--)
            {
                Point2 path_v = path_pts[i] - path_pts[i - 1];
                double path_theta = atan2(path_v.y, path_v.x);
                double path_delta = cx::trimRad(path_theta - path_last_theta);
                corner_path_len += norm(path_v);

                if (fabs(path_delta) > m_path_corner_thr)
                {
                    corner_path_idx = i - 1;
                    break;
                }
            }

            // path points
            std::vector<Point2> eval_path_pts;
            eval_path_pts.push_back(path_pts[0]);
            double path_len_upto = 0;
            int path_pts_idx = 0;
            double edge_len = norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]);
            Point2 mean_path_point = path_pts[0];
            for (int i = odo_start_idx + 1; i < n_odo; i++)
            {
                double odo_len_upto = odometry[i].dist_accumulated - odometry[odo_start_idx].dist_accumulated;
                double target_path_len = path_len_total * odo_len_upto / odo_len_total;
                while (path_len_upto + edge_len < target_path_len && path_pts_idx < n_path - 1)
                {
                    path_len_upto += edge_len;
                    path_pts_idx++;
                    edge_len = (path_pts_idx < n_path - 1) ? norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]) : 0;
                }
                double target_edge_len = target_path_len - path_len_upto;
                Point2 path_p1 = path_pts[path_pts_idx];
                Point2 path_p2 = path_pts[path_pts_idx + 1];
                Point2 path_point = (edge_len > 0 && path_pts_idx < n_path - 1) ? path_p1 + (path_p2 - path_p1) * target_edge_len / edge_len : path_p1;
                eval_path_pts.push_back(path_point);
                mean_path_point += path_point;
            }
            mean_path_point /= (double)eval_path_pts.size();

            // zero-mean path points
            for (int i = 0; i < (int)eval_path_pts.size(); i++)
            {
                eval_path_pts[i] -= mean_path_point;
            }

            // estimate align theta
            double rot = estimateAlignTheta(eval_odo_pts, path_pts);

            // rotate to align theta
            cv::Mat R(2, 2, CV_64FC1);
            R.at<double>(0, 0) = cos(rot);
            R.at<double>(0, 1) = -sin(rot);
            R.at<double>(1, 0) = sin(rot);
            R.at<double>(1, 1) = cos(rot);

            int n_data = (int)eval_path_pts.size();
            cv::Mat A = cv::Mat::zeros(2, n_data, CV_64FC1);
            cv::Mat B = cv::Mat::zeros(n_data, 2, CV_64FC1);
            for (int i = 0; i < n_data; i++)
            {
                A.at<double>(0, i) = eval_odo_pts[i].x;
                A.at<double>(1, i) = eval_odo_pts[i].y;
                B.at<double>(i, 0) = eval_path_pts[i].x;
                B.at<double>(i, 1) = eval_path_pts[i].y;
            }
            cv::Mat RA = R * A;

            // compute align cost
            if (!m_enable_icp || corner_path_idx < 0)
            {
                cv::Mat E = RA - B.t();
                double dx, dy;
                double err2_s = 0;
                for (int i = 0; i < n_data; i++)
                {
                    dx = E.at<double>(0, i);
                    dy = E.at<double>(1, i);
                    err2_s += (dx * dx + dy * dy);
                }
                err_l1 = sqrt(err2_s / n_data) + fabs(odo_len_total - path_len_total);
                double pdf_d = exp(-err_l1 * err_l1 / (2 * m_odo_align_sigma * m_odo_align_sigma)) + m_pdf_additive_min;

                int i1 = (int)eval_odo_pts.size() - 2;
                int i2 = (int)eval_odo_pts.size() - 1;
                double odo_x1 = RA.at<double>(0, i1) + mean_path_point.x;
                double odo_y1 = RA.at<double>(1, i1) + mean_path_point.y;
                double odo_x2 = RA.at<double>(0, i2) + mean_path_point.x;
                double odo_y2 = RA.at<double>(1, i2) + mean_path_point.y;
                odo_aligned_theta = atan2(odo_y2 - odo_y1, odo_x2 - odo_x1);
                double err_theta = cx::trimRad(odo_aligned_theta - particle_theta);
                double pdf_theta = exp(-err_theta * err_theta / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma)) + m_pdf_additive_min;
                double pdf = pdf_d * pdf_theta;
                //printf("x1=%.1lf,y1=%.1lf,x2=%.1lf,y2=%.1lf, odo_len=%lf, path_len=%lf, err_l1=%lf, odo_theta=%lf\n", odo_x1, odo_y1, odo_x2, odo_y2, odo_len_total, path_len_total, err_l1, odo_aligned_theta);

                if (pdf > m_best_pdf)
                {
                    m_best_pdf = pdf;
                    m_best_odo_pts = eval_odo_pts;
                    m_best_path_pts = eval_path_pts;
                    cv::Mat RA = R * A;
                    for (int i = 0; i < (int)eval_odo_pts.size(); i++)
                    {
                        m_best_odo_pts[i].x = RA.at<double>(0, i) + mean_path_point.x;
                        m_best_odo_pts[i].y = RA.at<double>(1, i) + mean_path_point.y;
                        m_best_path_pts[i] = eval_path_pts[i] + mean_path_point;
                    }
                }
                return pdf;
            }

            // transform odometry points (center align)
            std::vector<Point2> odo_aligned = eval_odo_pts;
            for (int i = 0; i < (int)eval_odo_pts.size(); i++)
            {
                odo_aligned[i].x = RA.at<double>(0, i) + mean_path_point.x;
                odo_aligned[i].y = RA.at<double>(1, i) + mean_path_point.y;
            }

            // ICP
            ICP_translation_align(odo_aligned, path_pts, m_max_icp_itr);

            // compute align cost
            double icp_align_err = 0;
            if (odo_len_total < path_len_total)
            {
                std::vector<Point2> eval_path_pts2;

                // find start path point
                int path_pts_idx = 0;
                double path_len_upto = 0;
                double target_path_len = path_len_total - odo_len_total;
                double edge_len = norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]);
                while (path_len_upto + edge_len < target_path_len && path_pts_idx < n_path - 1)
                {
                    path_len_upto += edge_len;
                    path_pts_idx++;
                    edge_len = (path_pts_idx < n_path - 1) ? norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]) : 0;
                }
                double target_edge_len = target_path_len - path_len_upto;
                Point2 path_p1 = path_pts[path_pts_idx];
                Point2 path_p2 = path_pts[path_pts_idx + 1];
                Point2 path_point = (edge_len > 0 && path_pts_idx < n_path - 1) ? path_p1 + (path_p2 - path_p1) * target_edge_len / edge_len : path_p1;
                eval_path_pts2.push_back(path_point);

                // remaining path points
                double start_length = path_len_total - odo_len_total;
                for (int i = odo_start_idx + 1; i < n_odo; i++)
                {
                    double odo_len_upto = odometry[i].dist_accumulated - odometry[odo_start_idx].dist_accumulated;
                    double target_path_len = odo_len_upto + start_length;
                    while (path_len_upto + edge_len < target_path_len && path_pts_idx < n_path - 1)
                    {
                        path_len_upto += edge_len;
                        path_pts_idx++;
                        edge_len = (path_pts_idx < n_path - 1) ? norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]) : 0;
                    }
                    double target_edge_len = target_path_len - path_len_upto;
                    Point2 path_p1 = path_pts[path_pts_idx];
                    Point2 path_p2 = path_pts[path_pts_idx + 1];
                    Point2 path_point = (edge_len > 0 && path_pts_idx < n_path - 1) ? path_p1 + (path_p2 - path_p1) * target_edge_len / edge_len : path_p1;
                    eval_path_pts2.push_back(path_point);
                }

                // compute align L2 error
                double align_d2 = 0;
                for (int i = 0; i < (int)eval_path_pts2.size(); i++)
                {
                    double dx = odo_aligned[i].x - eval_path_pts2[i].x;
                    double dy = odo_aligned[i].y - eval_path_pts2[i].y;
                    align_d2 += (dx * dx + dy * dy);
                }
                icp_align_err = sqrt(align_d2 / (double)odo_aligned.size());
            }
            else // odo_len_total >= path_len_total
            {
                // find new odometry start index
                int odo_start_idx2 = odo_start_idx;
                double odo_len_total2 = odo_len_total;
                while (odo_len_total2 > path_len_total && odo_start_idx2 < n_odo - 1)
                {
                    odo_start_idx2++;
                    odo_len_total2 = odometry[n_odo - 1].dist_accumulated - odometry[odo_start_idx2].dist_accumulated;
                }

                std::vector<Point2> eval_path_pts2;
                eval_path_pts2.push_back(path_pts[0]);
                double path_len_upto = 0;
                int path_pts_idx = 0;
                double edge_len = norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]);
                for (int i = odo_start_idx2 + 1; i < n_odo; i++)
                {
                    double odo_len_upto = odometry[i].dist_accumulated - odometry[odo_start_idx2].dist_accumulated;
                    double target_path_len = path_len_total * odo_len_upto / odo_len_total2;
                    while (path_len_upto + edge_len < target_path_len && path_pts_idx < n_path - 1)
                    {
                        path_len_upto += edge_len;
                        path_pts_idx++;
                        edge_len = (path_pts_idx < n_path - 1) ? norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]) : 0;
                    }
                    double target_edge_len = target_path_len - path_len_upto;
                    Point2 path_p1 = path_pts[path_pts_idx];
                    Point2 path_p2 = path_pts[path_pts_idx + 1];
                    Point2 path_point = (edge_len > 0 && path_pts_idx < n_path - 1) ? path_p1 + (path_p2 - path_p1) * target_edge_len / edge_len : path_p1;
                    eval_path_pts2.push_back(path_point);
                }

                // compute align L2 error
                double align_d2 = 0;
                int odo_skip = odo_start_idx2 - odo_start_idx;
                for (int i = 0; i < (int)eval_path_pts2.size(); i++)
                {
                    double dx = odo_aligned[i + odo_skip].x - eval_path_pts2[i].x;
                    double dy = odo_aligned[i + odo_skip].y - eval_path_pts2[i].y;
                    align_d2 += (dx * dx + dy * dy);
                }
                icp_align_err = sqrt(align_d2 / (double)odo_aligned.size());
            }

            err_l1 = norm(odo_aligned.back() - path_pts.back()) + icp_align_err;
            Point2 odo_last_v = odo_aligned[n_data - 1] - odo_aligned[n_data - 2];
            odo_aligned_theta = atan2(odo_last_v.y, odo_last_v.x);
            double err_theta = cx::trimRad(odo_aligned_theta - particle_theta) + m_pdf_additive_min;
            double pdf_theta = exp(-err_theta * err_theta / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma)) + m_pdf_additive_min;
            double pdf_d = exp(-err_l1 * err_l1 / (2 * m_odo_align_sigma * m_odo_align_sigma));

            double pdf = pdf_d * pdf_theta;
            if (pdf > m_best_pdf)
            {
                m_best_pdf = pdf;
                m_best_odo_pts = odo_aligned;
                m_best_path_pts = eval_path_pts;
                for (int i = 0; i < (int)odo_aligned.size(); i++)
                {
                    m_best_path_pts[i] = eval_path_pts[i] + mean_path_point;
                }
            }

            return pdf;
        }

        double estimateAlignTheta(const std::vector<Point2>& p, const std::vector<PathNode>& q)
        {
            int np = (int)p.size();
            int nq = (int)q.size();

            int bins = 180;
            int blur_n = 10;
            std::vector<double> phist, qhist;
            phist.resize(bins);
            qhist.resize(bins);
            for (int i = 0; i < bins; i++)
            {
                phist[i] = 0;
                qhist[i] = 0;
            }
            for (int i = 1; i < np; i++)
            {
                double theta = atan2(p[i].y - p[i - 1].y, p[i].x - p[i - 1].x) + CV_PI;
                int idx = (int)(0.5 * theta / CV_PI * bins);
                phist[idx] += norm(p[i] - p[i - 1]);
            }

            for (int i = 1; i < nq; i++)
            {
                double theta = atan2(q[i].y - q[i - 1].y, q[i].x - q[i - 1].x) + CV_PI;
                int idx = (int)(0.5 * theta / CV_PI * bins);
                qhist[idx] += norm(q[i] - q[i - 1]);
            }

            std::vector<double> tmp;
            tmp.resize(bins);
            for (int itr = 0; itr < blur_n; itr++)
            {
                for (int i = 1; i < bins - 1; i++)
                {
                    tmp[i] = (2 * phist[i] + phist[i - 1] + phist[i + 1]) / 4;
                }
                tmp[0] = (2 * phist[0] + phist[bins - 1] + phist[1]) / 4;
                tmp[bins - 1] = (2 * phist[bins - 1] + phist[bins - 2] + phist[0]) / 4;
                phist = tmp;

                for (int i = 1; i < bins - 1; i++)
                {
                    tmp[i] = (2 * qhist[i] + qhist[i - 1] + qhist[i + 1]) / 4;
                }
                tmp[0] = (2 * qhist[0] + qhist[bins - 1] + qhist[1]) / 4;
                tmp[bins - 1] = (2 * qhist[bins - 1] + qhist[bins - 2] + qhist[0]) / 4;
                qhist = tmp;
            }

            double rot = 0;
            double max_corr = -1;
            int max_k = -1;
            for (int k = 0; k < bins; k++)
            {
                double corr = 0;
                for (int i = 0; i < bins; i++)
                {
                    corr += (phist[(i + k) % bins] * qhist[i]);
                }
                if (corr > max_corr)
                {
                    rot = k * 2 * CV_PI / bins;
                    max_corr = corr;
                    max_k = k;
                }
            }
            double align_theta = -rot;
            return align_theta;
        }

        int ICP_translation_align(vector<Point2>& p, const vector<PathNode>& q, int max_itr)
        {
            if (max_itr <= 0) return 0;

            int np = (int)p.size();
            int nq = (int)q.size();

            int itr = 0;
            std::vector<Point2> eval_q;
            eval_q.resize(np);
            while (1)
            {
                // closest p-q pair
                for (int i = 0; i < np; i++)
                {
                    Point2 min_p;
                    double min_d2 = DBL_MAX;
                    for (int j = 1; j < nq; j++)
                    {
                        Point2 closest_p;
                        double d2 = calcDist2FromLineSeg(p[i], q[j - 1], q[j], closest_p);
                        if (d2 < min_d2)
                        {
                            min_p = closest_p;
                            min_d2 = d2;
                        }
                    }
                    eval_q[i] = min_p;
                }

                // estimate t
                Point2 t;
                estimate_t(p, eval_q, t);

                // transform p
                double max_delta = 0;
                for (int i = 0; i < np; i++)
                {
                    p[i].x += t.x;
                    p[i].y += t.y;

                    double delta = norm(t);
                    if (delta > max_delta) max_delta = delta;
                }

                itr++;
                if (itr > max_itr) break;
            }

            return itr;
        }

        void estimate_t(const vector<Point2>& p, const vector<Point2>& q, Point2& t)
        {
            int n_data = (int)p.size();

            Point2 p_mean(0, 0);
            Point2 q_mean(0, 0);
            for (int i = 0; i < n_data; i++)
            {
                p_mean += p[i];
                q_mean += q[i];
            }
            p_mean /= (double)n_data;
            q_mean /= (double)n_data;

            t = q_mean - p_mean;
        }

        double alignCostOdometryLS(const RingBuffer<OdometryData>& odometry, int odo_start_idx, const std::vector<Point2>& eval_odo_pts, dg::Point2 mean_odo_point, const std::vector<PathNode>& path_pts, double particle_theta, double& err_l1, double& odo_aligned_theta)
        {
            // odometry length
            int n_odo = odometry.data_count();
            double odo_len_total = odometry[n_odo - 1].dist_accumulated - odometry[odo_start_idx].dist_accumulated;

            // path length
            int n_path = (int)path_pts.size();
            double path_len_total = 0;
            for (int i = 1; i < n_path; i++)
            {
                path_len_total += norm(path_pts[i] - path_pts[i - 1]);
            }

            // check path cornerness
            Point2 path_v = path_pts[n_path - 1] - path_pts[n_path - 2];
            double path_last_theta = atan2(path_v.y, path_v.x);
            int corner_path_idx = -1;
            double corner_path_len = norm(path_v);
            for (int i = n_path - 2; i > 0; i--)
            {
                Point2 path_v = path_pts[i] - path_pts[i - 1];
                double path_theta = atan2(path_v.y, path_v.x);
                double path_delta = cx::trimRad(path_theta - path_last_theta);
                corner_path_len += norm(path_v);

                if (fabs(path_delta) > m_path_corner_thr)
                {
                    corner_path_idx = i - 1;
                    break;
                }
            }

            // path points
            std::vector<Point2> eval_path_pts;
            eval_path_pts.push_back(path_pts[0]);
            double path_len_upto = 0;
            int path_pts_idx = 0;
            double edge_len = norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]);
            Point2 mean_path_point = path_pts[0];
            for (int i = odo_start_idx + 1; i < n_odo; i++)
            {
                double odo_len_upto = odometry[i].dist_accumulated - odometry[odo_start_idx].dist_accumulated;
                double target_path_len = path_len_total * odo_len_upto / odo_len_total;
                while (path_len_upto + edge_len < target_path_len && path_pts_idx < n_path - 1)
                {
                    path_len_upto += edge_len;
                    path_pts_idx++;
                    edge_len = (path_pts_idx < n_path - 1) ? norm(path_pts[path_pts_idx + 1] - path_pts[path_pts_idx]) : 0;
                }
                double target_edge_len = target_path_len - path_len_upto;
                Point2 path_p1 = path_pts[path_pts_idx];
                Point2 path_p2 = path_pts[path_pts_idx + 1];
                Point2 path_point = (edge_len > 0 && path_pts_idx < n_path - 1) ? path_p1 + (path_p2 - path_p1) * target_edge_len / edge_len : path_p1;
                eval_path_pts.push_back(path_point);
                mean_path_point += path_point;
            }
            mean_path_point /= (double)eval_path_pts.size();

            // zero-mean path points
            for (int i = 0; i < (int)eval_path_pts.size(); i++)
            {
                eval_path_pts[i] -= mean_path_point;
            }

            // estimate rigid transform
            int n_data = (int)eval_path_pts.size();
            cv::Mat A = cv::Mat::zeros(2, n_data, CV_64FC1);
            cv::Mat B = cv::Mat::zeros(n_data, 2, CV_64FC1);
            for (int i = 0; i < n_data; i++)
            {
                A.at<double>(0, i) = eval_odo_pts[i].x;
                A.at<double>(1, i) = eval_odo_pts[i].y;
                B.at<double>(i, 0) = eval_path_pts[i].x;
                B.at<double>(i, 1) = eval_path_pts[i].y;
            }
            cv::Mat cov = A * B;
            cv::Mat U, D, VT;
            cv::SVD::compute(cov, D, U, VT);
            cv::Mat R = VT.t() * U.t();
            double det = cv::determinant(R);
            if (det < 0)
            {
                cv::Mat W = cv::Mat::eye(2, 2, CV_64FC1);
                W.at<double>(1, 1) = -1;
                R = VT.t() * W * U.t();
            }

            // compute align cost
            if (!m_enable_icp || corner_path_idx < 0)
            {
                cv::Mat RA = R * A;
                cv::Mat E = RA - B.t();
                double dx, dy;
                double err2_s = 0;
                for (int i = 0; i < n_data; i++)
                {
                    dx = E.at<double>(0, i);
                    dy = E.at<double>(1, i);
                    err2_s += (dx * dx + dy * dy);
                }
                err_l1 = sqrt(err2_s / n_data) + fabs(odo_len_total - path_len_total);
                double pdf_d = exp(-err_l1 * err_l1 / (2 * m_odo_align_sigma * m_odo_align_sigma));

                int i1 = (int)eval_odo_pts.size() - 2;
                int i2 = (int)eval_odo_pts.size() - 1;
                double odo_x1 = RA.at<double>(0, i1) + mean_path_point.x;
                double odo_y1 = RA.at<double>(1, i1) + mean_path_point.y;
                double odo_x2 = RA.at<double>(0, i2) + mean_path_point.x;
                double odo_y2 = RA.at<double>(1, i2) + mean_path_point.y;
                odo_aligned_theta = atan2(odo_y2 - odo_y1, odo_x2 - odo_x1);
                double err_theta = cx::trimRad(odo_aligned_theta - particle_theta) + m_pdf_additive_min;
                double pdf_theta = exp(-err_theta * err_theta / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma)) + m_pdf_additive_min;
                double pdf = pdf_d * pdf_theta;
                //printf("x1=%.1lf,y1=%.1lf,x2=%.1lf,y2=%.1lf, odo_len=%lf, path_len=%lf, err_l1=%lf, odo_theta=%lf\n", odo_x1, odo_y1, odo_x2, odo_y2, odo_len_total, path_len_total, err_l1, odo_aligned_theta);

                if (pdf > m_best_pdf)
                {
                    m_best_pdf = pdf;
                    m_best_odo_pts = eval_odo_pts;
                    m_best_path_pts = eval_path_pts;
                    cv::Mat RA = R * A;
                    for (int i = 0; i < (int)eval_odo_pts.size(); i++)
                    {
                        m_best_odo_pts[i].x = RA.at<double>(0, i) + mean_path_point.x;
                        m_best_odo_pts[i].y = RA.at<double>(1, i) + mean_path_point.y;
                        m_best_path_pts[i] = eval_path_pts[i] + mean_path_point;
                    }
                }
                return pdf;
            }

            // transform odometry points
            std::vector<Point2> odo_aligned = eval_odo_pts;
            double r1 = R.at<double>(0, 0);
            double r2 = R.at<double>(0, 1);
            double r3 = R.at<double>(1, 0);
            double r4 = R.at<double>(1, 1);
            for (int i = 0; i < (int)eval_odo_pts.size(); i++)
            {
                odo_aligned[i].x = r1 * eval_odo_pts[i].x + r2 * eval_odo_pts[i].y + mean_path_point.x;
                odo_aligned[i].y = r3 * eval_odo_pts[i].x + r4 * eval_odo_pts[i].y + mean_path_point.y;
            }

            // ICP
            ICP_align(odo_aligned, path_pts, m_max_icp_itr);

            // compute align cost
            err_l1 = norm(odo_aligned.back() - path_pts.back()) + fabs(odo_len_total - path_len_total);
            Point2 odo_last_v = odo_aligned[n_data - 1] - odo_aligned[n_data - 2];
            odo_aligned_theta = atan2(odo_last_v.y, odo_last_v.x);
            double err_theta = cx::trimRad(odo_aligned_theta - particle_theta);
            double pdf_theta = exp(-err_theta * err_theta / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma)) + m_pdf_additive_min;
            double pdf_d = exp(-err_l1 * err_l1 / (2 * m_odo_align_sigma * m_odo_align_sigma)) + m_pdf_additive_min;

            double pdf = pdf_d * pdf_theta;
            if (pdf > m_best_pdf)
            {
                m_best_pdf = pdf;
                m_best_odo_pts = odo_aligned;
                m_best_path_pts = eval_path_pts;
                for (int i = 0; i < (int)odo_aligned.size(); i++)
                {
                    m_best_path_pts[i] = eval_path_pts[i] + mean_path_point;
                }
            }

            return pdf;
        }

        void initialize_odo_theta_correction()
        {
            m_odo_dtheta_hist.resize(m_odo_dtheta_hist_bins);
            m_odo_dtheta_hist_cnt.resize(m_odo_dtheta_hist_bins);
            for (int i = 0; i < m_odo_dtheta_hist_bins; i++)
            {
                m_odo_dtheta_hist[i] = 0;
                m_odo_dtheta_hist_cnt[i] = 0;
            }
        }

        void update_odo_theta_hist(double dtheta_rad)
        {
            double dtheta_deg = cx::cvtRad2Deg(dtheta_rad);
            int idx = (int)(dtheta_deg / m_odo_dtheta_hist_angular_res) + (m_odo_dtheta_hist_bins - 1) / 2;
            if (idx < 0 || idx>(m_odo_dtheta_hist_bins - 1)) return;
            m_odo_dtheta_hist[idx] += dtheta_deg;
            m_odo_dtheta_hist_cnt[idx]++;
        }

        void odo_theta_correction_mean(const RingBuffer<OdometryData>& odometry, int odo_start_idx, vector<Point2>& odo_corrected)
        {
            int nodo = odometry.data_count() - odo_start_idx;

            int max_idx = -1;
            int max_cnt = 0;
            for (int i = 0; i < m_odo_dtheta_hist_bins; i++)
            {
                if (m_odo_dtheta_hist_cnt[i] > max_cnt)
                {
                    max_cnt = m_odo_dtheta_hist_cnt[i];
                    max_idx = i;
                }
            }
            if (max_idx < 0) return;

            double dtheta_mean = m_odo_dtheta_hist[max_idx];
            int dtheta_cnt = m_odo_dtheta_hist_cnt[max_idx];
            if (max_idx < m_odo_dtheta_hist_bins - 1)
            {
                dtheta_mean += m_odo_dtheta_hist[max_idx + 1];
                dtheta_cnt += m_odo_dtheta_hist_cnt[max_idx + 1];
            }
            if (max_idx > 0)
            {
                dtheta_mean += m_odo_dtheta_hist[max_idx - 1];
                dtheta_cnt += m_odo_dtheta_hist_cnt[max_idx - 1];
            }
            dtheta_mean /= (double)dtheta_cnt;
            dtheta_mean = cx::cvtDeg2Rad(dtheta_mean);

            cv::Point2d v;
            odo_corrected.resize(nodo);
            odo_corrected[0].x = odometry[odo_start_idx].x;
            odo_corrected[0].y = odometry[odo_start_idx].y;

            for (int i = 1; i < nodo; i++)
            {
                double cos_t = cos(-dtheta_mean * i);
                double sin_t = sin(-dtheta_mean * i);
                double x_prev = odo_corrected[i - 1].x;
                double y_prev = odo_corrected[i - 1].y;
                v.x = odometry[odo_start_idx + i].x - odometry[odo_start_idx + i - 1].x;
                v.y = odometry[odo_start_idx + i].y - odometry[odo_start_idx + i - 1].y;

                odo_corrected[i].x = x_prev + (v.x * cos_t - v.y * sin_t);
                odo_corrected[i].y = y_prev + (v.x * sin_t + v.y * cos_t);
            }
        }

        void estimate_Rt(const vector<Point2>& p, const vector<Point2>& q, cv::Mat& R, Point2& t)
        {
            int n_data = (int)p.size();

            Point2 p_mean(0, 0);
            Point2 q_mean(0, 0);
            for (int i = 0; i < n_data; i++)
            {
                p_mean += p[i];
                q_mean += q[i];
            }
            p_mean /= (double)n_data;
            q_mean /= (double)n_data;

            // estimate rigid transform
            cv::Mat A = cv::Mat::zeros(2, n_data, CV_64FC1);
            cv::Mat B = cv::Mat::zeros(n_data, 2, CV_64FC1);
            for (int i = 0; i < n_data; i++)
            {
                A.at<double>(0, i) = p[i].x - p_mean.x;
                A.at<double>(1, i) = p[i].y - p_mean.y;
                B.at<double>(i, 0) = q[i].x - q_mean.x;
                B.at<double>(i, 1) = q[i].y - q_mean.y;
            }
            cv::Mat cov = A * B;
            cv::Mat U, D, VT;
            cv::SVD::compute(cov, D, U, VT);
            R = VT.t() * U.t();
            double det = cv::determinant(R);
            if (det < 0)
            {
                cv::Mat W = cv::Mat::eye(2, 2, CV_64FC1);
                W.at<double>(1, 1) = -1;
                R = VT.t() * W * U.t();
            }

            t.x = q_mean.x - (R.at<double>(0, 0) * p_mean.x + R.at<double>(0, 1) * p_mean.y);
            t.y = q_mean.y - (R.at<double>(1, 0) * p_mean.x + R.at<double>(1, 1) * p_mean.y);
        }

        double calcDist2FromLineSeg(const Point2& p, const Point2& from, const Point2& to, Point2& closest_p)
        {
            // Ref. https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
            Point2 delta = to - from;
            double l2 = delta.x * delta.x + delta.y * delta.y;
            if (l2 < DBL_EPSILON)
            {
                closest_p = from;
                Point2 dp = p - from;
                double dist2 = dp.x * dp.x + dp.y * dp.y;
                return dist2;
            }
            double t = std::max(0., std::min(1., (p - from).dot(to - from) / l2));
            closest_p = from + t * (to - from);
            Point2 dp = p - closest_p;
            double dist2 = dp.x * dp.x + dp.y * dp.y;
            return dist2;
        }

        int ICP_align(vector<Point2>& p_aligned, const vector<PathNode>& q, int max_itr)
        {
            if (max_itr <= 0) return 0;

            int np = (int)p_aligned.size();
            int nq = (int)q.size();

            // ICP
            int itr = 0;
            vector<Point2> eval_q;
            eval_q.resize(np);

            while (1)
            {
                // closest p-q pair
                for (int i = 0; i < np; i++)
                {
                    Point2 min_p;
                    double min_d2 = DBL_MAX;
                    for (int j = 1; j < nq; j++)
                    {
                        Point2 closest_p;
                        double d2 = calcDist2FromLineSeg(p_aligned[i], q[j - 1], q[j], closest_p);
                        if (d2 < min_d2)
                        {
                            min_p = closest_p;
                            min_d2 = d2;
                        }
                    }
                    eval_q[i] = min_p;
                }

                // estimate R,t
                cv::Mat R;
                Point2 t;
                estimate_Rt(p_aligned, eval_q, R, t);

                // transform p
                double r1 = R.at<double>(0, 0);
                double r2 = R.at<double>(0, 1);
                double r3 = R.at<double>(1, 0);
                double r4 = R.at<double>(1, 1);
                double max_delta2 = 0;
                for (int i = 0; i < np; i++)
                {
                    double x = r1 * p_aligned[i].x + r2 * p_aligned[i].y + t.x;
                    double y = r3 * p_aligned[i].x + r4 * p_aligned[i].y + t.y;
                    double delta2 = (p_aligned[i].x - x) * (p_aligned[i].x - x) + (p_aligned[i].y - y) * (p_aligned[i].y - y);
                    if (delta2 > max_delta2) max_delta2 = delta2;
                    p_aligned[i].x = x;
                    p_aligned[i].y = y;
                }

                itr++;
                if (itr >= max_itr || max_delta2 < 1e-8) break;
            }

            return itr;
        }

        void evaluateParticlesPOI(const Point2& clue_xy, const Polar2& relative, bool debug_print = false)
        {
            int N = (int)m_particles.size();

            double w_sum = 0;
            Map* map = m_shared->getMap();
            for (int i = 0; i < N; i++)
            {
                // particle pose
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Pose2 pose = *from + m_particles[i].dist * v / edge->length;
                pose.theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);

                double poi_theta = pose.theta + relative.ang;
                Point2 poi_xy;
                poi_xy.x = pose.x + relative.lin * cos(poi_theta);
                poi_xy.y = pose.y + relative.lin * sin(poi_theta);

                double err_d = norm(clue_xy - poi_xy);
                double pdf = exp(-err_d * err_d / (2 * m_poi_pos_sigma * m_poi_pos_sigma)) + m_pdf_additive_min;
                if (debug_print) printf("POI [%d] err_d = %lf, pdf = %lf\n", i, err_d, pdf);

                m_particles[i].weight *= pdf;
                w_sum += m_particles[i].weight;
            }

            for (int i = 0; i < N; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

        void evaluateParticlesVPS(const Point2& clue_xy, const Polar2& relative, bool debug_print = false)
        {
            int N = (int)m_particles.size();

            double w_sum = 0;
            Map* map = m_shared->getMap();
            for (int i = 0; i < N; i++)
            {
                // particle pose
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Pose2 pose = *from + m_particles[i].dist * v / edge->length;
                pose.theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);

                double err_d = norm(clue_xy - pose);
                double pdf = exp(-err_d * err_d / (2 * m_vps_pos_sigma * m_vps_pos_sigma)) + m_pdf_additive_min;
                if (debug_print) printf("VPS [%d] err_d = %lf, pdf = %lf\n", i, err_d, pdf);

                m_particles[i].weight *= pdf;
                w_sum += m_particles[i].weight;
            }

            for (int i = 0; i < N; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

    }; // End of 'DGLocalizerMCL'


} // End of 'dg'

#endif // End of '__LOCALIZER_MCL_HPP__'
