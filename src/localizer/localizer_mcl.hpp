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
        ID eid_mcl = 0;
        double dist_accumulated = 0;
        double cornerness = 0;

        OdometryData() { dist = dtheta = t = 0; }
        OdometryData(const Pose2& p, double _x, double _y, double _theta, double _dist, double _dtheta, Timestamp ts) : pose_mcl(p), x(_x), y(_y), theta(_theta), dist(_dist), dtheta(_dtheta), t(ts) {}
    };

    class DGLocalizerMCL : public DGLocalizer
    {
    protected:
        int m_particle_numbers = 400;
        int m_resample_numbers = 0;
        double m_resample_sigma = 10;       // meter
        int m_eval_history_length = 50;     // meter
        int m_odometry_history_size = 1000;
        double m_odometry_save_interval = 1;
        double m_corner_cosine = cos(CV_PI / 4);
        double m_path_particle_weight = 2;
        int m_max_icp_itr = 20;

        double m_odometry_linear_std_err = 0.5;                 // 0.5 meter
        double m_odometry_angular_std_err = cx::cvtDeg2Rad(1);  // 1 degree

        double m_gps_pos_sigma = 20;
        double m_gps_theta_sigma = cx::cvtDeg2Rad(100);
        double m_poi_pos_sigma = 5;
        double m_vps_pos_sigma = 20;
        double m_odo_align_sigma = 10;
        double m_odo_align_theta_sigma = cx::cvtDeg2Rad(50);

        Pose2 m_prev_gps_pose;
        Timestamp m_prev_gps_time = -1;
        Pose2 m_pose_mcl;
        ID m_eid_mcl;
        bool m_mcl_pose_valid = false;
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

        // saving data for test samples
        std::vector<cv::Vec6d> m_odo_pts_original;  // x,y,theta, dtheta, dist_accumulated, cornerness
        Path m_best_path;

    public:
        DGLocalizerMCL(std::string baselocalizer_name = "EKFLocalizer")
        {
            srand((unsigned)time(NULL));
            initialize(nullptr, baselocalizer_name);
            m_odometry_history.resize(m_odometry_history_size);
        }

        virtual bool applyGPS(const Point2& xy, Timestamp time = -1, double confidence = -1)
        {
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
                    evaluateParticles(m_ekf->getPose());
                    resampleParticles();
                    estimateMclPose(m_pose_mcl, m_eid_mcl);
                    m_mcl_pose_valid = true;
                }
                m_pose = m_pose_mcl;
            }
            m_prev_gps_pose = m_ekf->getPose();
            m_prev_gps_time = time;

            return applyPathLocalizerMCL(m_pose, time);
        }

        virtual bool applyPOI(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false)
        {
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
            printf("\n[MCL-PPOI] before: x=%.1lf, y=%.1lf, after: x=%.1lf, y=%.1lf\n\n", before.x, before.y, after.x, after.y);

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

            if (m_ekf_pose_history.empty())
            {
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                return false;
            }
            if (!m_ekf->applyOdometry(odometry_pose, time, confidence))
            {
                m_prev_odometry_pose = odometry_pose;
                m_prev_odometry_time = time;
                return false;
            }
            saveObservation(ObsData::OBS_ODO, odometry_pose, time, confidence);
            saveEKFState(m_ekf, time);

            // update particle
            if (m_mcl_initialized)
            {
                if(!m_robot_stopped)
                {
                    dg::Pose2 prev = m_pose_mcl;

                    predictParticles(odometry_pose, m_prev_odometry_pose, time - m_prev_odometry_time, m_robot_stopped);
                    if (m_gps_deactivated)
                    {
                        evaluateParticles(m_ekf->getPose());
                        resampleParticles();
                    }
                    estimateMclPose(m_pose_mcl, m_eid_mcl);
                    m_pose = m_pose_mcl;
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
            }
            m_prev_odometry_pose = odometry_pose;
            m_prev_odometry_time = time;

            return applyPathLocalizerMCL(m_pose, time);
        }

        virtual void setPose(const Pose2 pose, Timestamp time = -1, bool reset_velocity = true, bool reset_cov = true)
        {
            DGLocalizer::setPose(pose, time, reset_velocity, reset_cov);
            cv::AutoLock lock(m_mutex);
            reset_particles(pose, m_particles, m_particle_numbers);
            m_mcl_initialized = true;
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
            if(dtheta>CV_PI/2 || dtheta<-CV_PI/2)
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
                        if (k>=cnt || idx >= n) break;

                        particles[idx].start_node = map->getNode(edges[i]->node_id2);
                        particles[idx].edge = edges[i];
                        particles[idx].dist = (double)rand() * edges[i]->length / RAND_MAX;
                        particles[idx].head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                        idx++;
                        k++;
                        if (k>=cnt || idx >= n) break;

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

        void add_uniform_particles(const Pose2 p, double radius, std::vector<Particle>& particles, int n, double w)
        {
            Map* map = m_shared->getMap();
            Point2 ep;
            Edge* edge = map->getNearestEdge(p, ep);
            if(norm(p - ep)>=radius) return;

            //Node* node1 = map->getNode(edge->node_id1);
            //Node* node2 = map->getNode(edge->node_id2);
            //double d1 = norm(ep - *node1);
            //double d2 = norm(ep - *node2);

            // find connected edges
            std::vector<const Edge*> results;
            std::list<const Edge*> open;
            std::map<ID, int> lookup_tmp;
            int idx = 0;
            open.push_back(edge);
            lookup_tmp.insert(std::make_pair(edge->id, idx++));                
            results.push_back(edge);
            while (!open.empty())
            {
                // pick next
                auto e = open.front();
                open.pop_front();

                Node* n1 = map->getNode(e->node_id1);
                Node* n2 = map->getNode(e->node_id2);
                double d1 = norm(p - *n1);
                double d2 = norm(p - *n2);

                if(d1<radius)
                {
                    for (auto it = n1->edge_ids.begin(); it != n1->edge_ids.end(); it++)
                    {
                        const Edge* next = map->getEdge(*it);
                        if (next == nullptr) continue;

                        // check duplication
                        auto found = lookup_tmp.find(next->id);
                        if (found != lookup_tmp.end()) continue;

                        open.push_back(next);
                        results.push_back(next);
                        lookup_tmp.insert(std::make_pair(next->id, idx++));
                    }
                }
                if(d2<radius)
                {
                    for (auto it = n2->edge_ids.begin(); it != n2->edge_ids.end(); it++)
                    {
                        const Edge* next = map->getEdge(*it);
                        if (next == nullptr) continue;

                        // check duplication
                        auto found = lookup_tmp.find(next->id);
                        if (found != lookup_tmp.end()) continue;

                        open.push_back(next);
                        results.push_back(next);
                        lookup_tmp.insert(std::make_pair(next->id, idx++));
                    }
                }
            }

            int added = 0;
            if (map)
            {
                std::vector<Edge*> edges = map->getNearEdges(p, radius);
                if(edges.empty()) return;

                double total_length = 0;
                for (size_t i = 0; i < edges.size(); i++)
                {
                    total_length += edges[i]->length;
                }

                for (size_t i = 0; i < edges.size(); i++)
                {
                    int cnt = (int)(n * edges[i]->length / total_length + 0.5);

                    dg::Node* from = map->getNode(edges[i]->node_id1);
                    dg::Edge* edge = edges[i];
                    dg::Node* to = map->getNode(edges[i]->node_id2);

                    dg::Point2 v = *to - *from;
                    double theta = atan2(v.y, v.x);
                    bool use_node1 = true;
                    if (fabs(cx::trimRad(theta - p.theta)) < CV_PI / 2) use_node1 = true;
                    else use_node1 = false;

                    int k = 0;
                    while (k < cnt)
                    {
                        Particle particle;
                        if(use_node1)
                        {
                            particle.start_node = map->getNode(edges[i]->node_id1);
                            particle.edge = edges[i];
                            particle.dist = (double)rand() * edges[i]->length / RAND_MAX;
                            particle.head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                            particle.weight = w;
                            particles.push_back(particle);
                        }
                        else
                        {
                            particle.start_node = map->getNode(edges[i]->node_id2);
                            particle.edge = edges[i];
                            particle.dist = (double)rand() * edges[i]->length / RAND_MAX;
                            particle.head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                            particle.weight = w;
                            particles.push_back(particle);
                        }
                        k++;
                        added++;
                        if (k>=cnt || added >= n) break;
                    }
                    if (added >= n) break;
                }

                while (added < n)
                {
                    int ei = rand() % ((int)(edges.size()));
                    Edge* edge = edges[ei];

                    dg::Node* from = map->getNode(edge->node_id1);
                    dg::Node* to = map->getNode(edge->node_id2);

                    dg::Point2 v = *to - *from;
                    double theta = atan2(v.y, v.x);
                    bool use_node1 = true;
                    if (fabs(cx::trimRad(theta - p.theta)) < CV_PI / 2) use_node1 = true;
                    else use_node1 = false;

                        Particle particle;
                        if(use_node1)
                        {
                            particle.start_node = map->getNode(edge->node_id1);
                            particle.edge = edge;
                            particle.dist = (double)rand() * edge->length / RAND_MAX;
                            particle.head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                            particle.weight = w;
                            particles.push_back(particle);
                        }
                        else
                        {
                            particle.start_node = map->getNode(edge->node_id2);
                            particle.edge = edge;
                            particle.dist = (double)rand() * edge->length / RAND_MAX;
                            particle.head = cx::trimRad((double)rand() * CV_PI / RAND_MAX);
                            particle.weight = w;
                            particles.push_back(particle);
                        }
                    added++;
                    if (added >= n) break;
                }
            }
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

        bool estimateMclPose(Pose2& pose, ID& eid)
        {

            int N = (int)m_particles.size();

           // average of theta --> assume they are on unit circle --> average coordinate of unit circle position
            Point2 mean_p(0, 0);
            double mean_x = 0;
            double mean_y = 0;
            double w_sum = 0;
            Map* map = m_shared->getMapLocked();
            for (int i = 0; i < N; i++)
            {
                Node* from = m_particles[i].start_node;
                Edge* edge = m_particles[i].edge;
                Node* to = map->getConnectedNode(from, edge->id);
                Point2 v = *to - *from;
                Point2 pos_m = *from + m_particles[i].dist * v / edge->length;
                double theta = cx::trimRad(atan2(v.y, v.x) + m_particles[i].head);
                mean_x += (cos(theta) * m_particles[i].weight);
                mean_y += (sin(theta) * m_particles[i].weight);
                mean_p += (pos_m * m_particles[i].weight);
                w_sum += m_particles[i].weight;
            }
            mean_p /= w_sum;
            mean_x /= w_sum;
            mean_y /= w_sum;
            double mean_theta = atan2(mean_y, mean_x);
            m_shared->releaseMapLock();

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
            int M = m_particle_numbers - m_resample_numbers;
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
            add_uniform_particles(m_ekf->getPose(), m_resample_sigma, tmp, m_resample_numbers, w_sum/M);
            w_sum += (m_resample_numbers * w_sum / M);

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
            while (k >= n - 1 - cornerness_delta && k >= n-1-k)
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

        void evaluateParticles(const Pose2& pose_ekf)
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
                if (!m_gps_deactivated)
                {
                    double dx = pose_particle.x - pose_ekf.x;
                    double dy = pose_particle.y - pose_ekf.y;
                    gd = sqrt(dx * dx + dy * dy);
                    gth = cx::trimRad(pose_particle.theta - pose_ekf.theta);
                    pdf_ekf_d = exp(-gd * gd / (2 * m_gps_pos_sigma * m_gps_pos_sigma)) / (m_gps_pos_sigma * sqrt(2 * CV_PI));
                    pdf_ekf_theta = exp(-gth * gth / (2 * m_gps_theta_sigma * m_gps_theta_sigma)) / (m_gps_theta_sigma * sqrt(2 * CV_PI));
                }

                // odometry align weight
                double pdf_odo_d = 1;
                double pdf_odo_theta = 1;
                double od = 0;
                double oth = 0;
                double odo_aligned_theta = 0;
                if(odo_valid)
                {
                    odo_aligned_theta = aligned_theta[i];
                    od = align_err_l1[i];
                    oth = cx::trimRad(pose_particle.theta - aligned_theta[i]);
                    pdf_odo_d = exp(-od * od / (2 * m_odo_align_sigma * m_odo_align_sigma)) / (m_odo_align_sigma * sqrt(2 * CV_PI));
                    pdf_odo_theta = exp(-oth * oth / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma)) / (m_odo_align_theta_sigma * sqrt(2 * CV_PI));
                }

                double pdf = path_weight*(pdf_ekf_d*pdf_ekf_theta * pdf_odo_d*pdf_odo_theta);
                if(pdf>max_pdf) max_pdf = pdf;
                //printf("[%d] w=%lf, gd=%.1lf(%lf), gth=%.1lf(%lf), od=%.1lf(%lf), oth=%.1lf(%lf), pth=%.1lf, alignth=%.1lf\n", i, pdf, gd, pdf_ekf_d, cx::cvtRad2Deg(gth), pdf_ekf_theta, od, pdf_odo_d, cx::cvtRad2Deg(oth), pdf_odo_theta, cx::cvtRad2Deg(pose_particle.theta), cx::cvtRad2Deg(odo_aligned_theta));

                m_particles[i].weight *= pdf;
                w_sum += m_particles[i].weight;
            }
            m_shared->releaseMapLock();
            //printf("[MCL] max_pdf = %lf, w_sum = %lf\n", max_pdf, w_sum);

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
            if (odo.dist_accumulated < 20) return false;

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
            theta_correction_mean(m_odometry_history, odo_start_idx, odo_pts);

            // whitening of odometry points
            Point2 mean_odo_point(0, 0);
            for (int i = odo_start_idx; i < n_odo; i++)
            {
                mean_odo_point += odo_pts[i - odo_start_idx];
            }
            mean_odo_point /= (double)odo_pts.size();
            for (int i = 0; i<(int)odo_pts.size(); i++)
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
            double corner_thr = cx::cvtDeg2Rad(60);
            Point2 path_v = path_pts[n_path-1] - path_pts[n_path-2];
            double path_last_theta = atan2(path_v.y, path_v.x);
            int corner_path_idx = -1;
            double corner_path_len = norm(path_v);
            for (int i = n_path - 2; i > 0; i--)
            {
                Point2 path_v = path_pts[i] - path_pts[i - 1];
                double path_theta = atan2(path_v.y, path_v.x);
                double path_delta = cx::trimRad(path_theta - path_last_theta);
                corner_path_len += norm(path_v);

                if(fabs(path_delta) > corner_thr)
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
            if(corner_path_idx < 0)
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
                double err_theta = cx::trimRad(odo_aligned_theta - particle_theta);
                double pdf_theta = exp(-err_theta * err_theta / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma));
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
            Point2 odo_last_v = odo_aligned[n_data-1] - odo_aligned[n_data-2];
            odo_aligned_theta = atan2(odo_last_v.y, odo_last_v.x);
            double err_theta = cx::trimRad(odo_aligned_theta - particle_theta);
            double pdf_theta = exp(-err_theta * err_theta / (2 * m_odo_align_theta_sigma * m_odo_align_theta_sigma));
            double pdf_d = exp(-err_l1*err_l1 / (2 * m_odo_align_sigma * m_odo_align_sigma));

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

        void theta_correction_mean(const RingBuffer<OdometryData>& odometry, int odo_start_idx, vector<Point2>& odo_corrected)
        {
            double angular_res = 1;   // degree
            int bins = 3;             // should be odd number (-2~-1, -1~+1, +1~+2)

            int nodo = odometry.data_count() - odo_start_idx;
            vector<double> hist;
            vector<int> cnt;
            hist.resize(bins);
            cnt.resize(bins);

            for (int i = 0; i < bins; i++)
            {
                hist[i] = 0;
                cnt[i] = 0;
            }

            for (int i = 1; i < nodo; i++)
            {
                double dtheta = cx::cvtRad2Deg(odometry[odo_start_idx + i].dtheta);
                int idx = (int)(dtheta / angular_res) + (bins - 1) / 2;
                if (idx < 0 || idx>(bins - 1)) continue;
                hist[idx] += dtheta;
                cnt[idx]++;
            }

            int max_idx = -1;
            int max_cnt = 0;
            for (int i = 0; i < bins; i++)
            {
                if (cnt[i] > max_cnt)
                {
                    max_cnt = cnt[i];
                    max_idx = i;
                }
            }

            double dtheta_mean = hist[max_idx];
            int dtheta_cnt = cnt[max_idx];
            if (max_idx < bins - 1)
            {
                dtheta_mean += hist[max_idx + 1];
                dtheta_cnt += cnt[max_idx + 1];
            }
            if (max_idx > 0)
            {
                dtheta_mean += hist[max_idx - 1];
                dtheta_cnt += cnt[max_idx - 1];
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

        void evaluateParticlesPOI(const Point2& clue_xy, const Polar2& relative)
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
                double pdf = 0.0001 + exp(-err_d * err_d / (2 * m_poi_pos_sigma * m_poi_pos_sigma)) / (m_poi_pos_sigma * sqrt(2 * CV_PI));
                printf("POI [%d] err_d = %lf, pdf = %lf\n", i, err_d, pdf);

                m_particles[i].weight *= pdf;
                w_sum += m_particles[i].weight;
            }

            for (int i = 0; i < N; i++)
            {
                m_particles[i].weight /= w_sum;
            }
        }

        void evaluateParticlesVPS(const Point2& clue_xy, const Polar2& relative)
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
                double pdf = 0.0001 + exp(-err_d * err_d / (2 * m_vps_pos_sigma * m_vps_pos_sigma)) / (m_vps_pos_sigma * sqrt(2 * CV_PI));
                //printf("VPS [%d] err_d = %lf, pdf = %lf\n", i, err_d, pdf);

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
