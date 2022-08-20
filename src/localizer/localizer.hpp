#ifndef __LOCALIZER_HPP__
#define __LOCALIZER_HPP__

#include "core/basic_type.hpp"
#include "core/shared_data.hpp"
#include "localizer/localizer_ekf.hpp"
#include "localizer/localizer_ekf_variants.hpp"
#include "localizer/path_projector.hpp"
#include "utils/ring_buffer.hpp"
#include <vector>

namespace dg
{

    /**
     * @brief Localizer State
     *
     * Data structure for saving state of ekf localizer with timestamp
     */
    struct EKFState
    {
        /** The state variable */
        cv::Mat state_vec;

        /** The state covariance */
        cv::Mat state_cov;

        /** The ekf's internal time */
        double state_time;

        /** Ekf's imu-related values */
        double imu_angle, imu_time;

        /** The timestamp of this snapshot */
        Timestamp timestamp;
    };

    /**
     * @brief Observation Data
     *
     * Data structure for saving sensor observations with timestamp
     */
    struct ObsData
    {
        /** Observation type definition */
        enum
        {
            /** GPS data (v1: utm x, v2: utm y) */
            OBS_GPS = 0,

            /** Odometry data (v1: odometry x, v2: odometry y, v3: odometry theta) */
            OBS_ODO = 1,

            /** IMU data (v1: odometry theta) */
            OBS_IMU = 2,

            /** POI data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
            OBS_POI = 3,

            /** VPS data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
            OBS_VPS = 4,

            /** Intersection classifier data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
            OBS_IntersectCls = 5,

            /** RoadLR data (v1 = 0: uncertain, v1 = 1: road is left, v1 = 2: road is right) */
            OBS_RoadLR = 6,

            /** RoadTheta data (v1: theta) */
            OBS_RoadTheta = 7,

            /** Intersection3Camera classifier data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
            OBS_Intersect3CameraCls = 8
        };

        /**
         * The type of observation data
         * @see OBS_GPS, OBS_IMU, OBS_POI, OBS_VPS, OBS_VPS_LR, OBS_RoadTheta, OBS_IntersectCls, OBS_Intersect3CameraCls
         */
        int type;

        /**
         * Data values (The meaning of variable varies depending on the observation type)
         * @see Observation type definition
         */
        double v1, v2, v3, v4;

        /**
         * The timestamp
         */
        Timestamp timestamp;

        /**
         * The confidence of observation data
         */
        double confidence;

        /** The default constructor */
        ObsData() { v1 = v2 = v3 = v4 = 0; }

        /** Constructor */
        ObsData(int _type, double theta, Timestamp time, double conf) : type(_type), v1(theta), timestamp(time), confidence(conf) { v2 = v3 = v4 = 0; }

        /** Constructor */
        ObsData(int _type, Point2 xy, Timestamp time, double conf) : type(_type), v1(xy.x), v2(xy.y), timestamp(time), confidence(conf) { v3 = v4 = 0; }

        /** Constructor */
        ObsData(int _type, Pose2 pose, Timestamp time, double conf) : type(_type), v1(pose.x), v2(pose.y), v3(pose.theta), timestamp(time), confidence(conf) { v4 = 0; }

        /** Constructor */
        ObsData(int _type, Point2 xy, Polar2 relative, Timestamp time, double conf) : type(_type), v1(xy.x), v2(xy.y), v3(relative.lin), v4(relative.ang), timestamp(time), confidence(conf) {}
    };

    /**
     * @brief Path-based Localizer
     *
     * Localization module of deepguider system
     */
    class DGLocalizer : public BaseLocalizer, public PathProjector
    {
    protected:
        // configuable parameters
        int m_history_size = 2000;
        bool m_enable_path_projection = true;
        bool m_enable_map_projection = false;
        bool m_enable_backtracking_ekf = true;
        bool m_enable_gps_smoothing = false;
        double m_smoothing_alpha = 0.1;
        double m_smoothing_beta = 0.01;
        double m_smoothing_velocity_decaying = 0.95;

        /** Read parameters from cv::FileNode - Inherited from cx::Algorithm */
        virtual int readParam(const cv::FileNode& fn)
        {
            int n_read = PathProjector::readParam(fn);
            n_read += m_ekf->readParam(fn);
            CX_LOAD_PARAM_COUNT(fn, "enable_path_projection", m_enable_path_projection, n_read);
            CX_LOAD_PARAM_COUNT(fn, "enable_map_projection", m_enable_map_projection, n_read);
            CX_LOAD_PARAM_COUNT(fn, "enable_backtracking_ekf", m_enable_backtracking_ekf, n_read);
            CX_LOAD_PARAM_COUNT(fn, "enable_gps_smoothing", m_enable_gps_smoothing, n_read);
            CX_LOAD_PARAM_COUNT(fn, "smoothing_alpha", m_smoothing_alpha, n_read);
            CX_LOAD_PARAM_COUNT(fn, "smoothing_beta", m_smoothing_beta, n_read);
            CX_LOAD_PARAM_COUNT(fn, "smoothing_velocity_decaying", m_smoothing_velocity_decaying, n_read);

            int history_size = m_history_size;
            CX_LOAD_PARAM_COUNT(fn, "history_size", history_size, n_read);
            if (history_size != m_history_size)
            {
                m_history_size = history_size;
                initInternalVariables();
            }
            return n_read;
        }

    public:
        /** The constructor */
        DGLocalizer()
        {
            initialize(nullptr, "EKFLocalizer");
        }

        bool initialize(SharedInterface* shared, std::string baselocalizer_name = "")
        {
            cv::AutoLock lock(m_mutex);
            if (baselocalizer_name == "EKFLocalizer") m_ekf = cv::makePtr<dg::EKFLocalizer>();
            else if (baselocalizer_name == "EKFLocalizerZeroGyro") m_ekf = cv::makePtr<dg::EKFLocalizerZeroGyro>();
            else if (baselocalizer_name == "EKFLocalizerHyperTan") m_ekf = cv::makePtr<dg::EKFLocalizerHyperTan>();
            else if (baselocalizer_name == "EKFLocalizerSinTrack") m_ekf = cv::makePtr<dg::EKFLocalizerSinTrack>();
            if (m_ekf) m_ekf->setShared(shared);
            BaseLocalizer::setShared(shared);
            initInternalVariables();
            return (m_shared != nullptr && !m_ekf.empty());
        }

        virtual bool setShared(SharedInterface* shared)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) m_ekf->setShared(shared);
            return BaseLocalizer::setShared(shared);
        }

        virtual void setPose(const Pose2 pose, Timestamp time = -1, bool reset_velocity = true, bool reset_cov = true)

        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) m_ekf->setPose(pose, time, reset_velocity, reset_cov);
            m_state_history.resize(m_history_size);
            m_observation_history.resize(m_history_size);
            m_pose_history.resize(m_history_size);
            m_projected_pose_history.resize(m_history_size);
            m_pose = pose;
        }

        Pose2 getEkfPose()
        {
            if (m_ekf) return m_ekf->getPose();
            else return Pose2();
        }

        virtual bool setParamMotionNoise(double sigma_linear_velocity, double sigma_angular_velocity_deg, double cov_lin_ang = 0)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamMotionNoise(sigma_linear_velocity, sigma_angular_velocity_deg, cov_lin_ang);
            return false;
        }

        virtual bool setParamMotionBounds(double max_linear_velocity, double max_angular_velocity_deg, double min_linear_velocity = -1)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamMotionBounds(max_linear_velocity, max_angular_velocity_deg, min_linear_velocity);
            return false;
        }

        virtual bool setParamGPSNoise(double sigma_normal, double sigma_deadzone = -1)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamGPSNoise(sigma_normal, sigma_deadzone);
            return false;
        }

        virtual bool setParamGPSOffset(double lin_offset, double ang_offset_deg = 0)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamGPSOffset(lin_offset, ang_offset_deg);
            return false;
        }

        virtual bool setParamOdometryNoise(double sigma_position, double sigma_theta_deg)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamOdometryNoise(sigma_position, sigma_theta_deg);
            return false;
        }

        virtual bool setParamIMUCompassNoise(double sigma_theta_deg, double offset = 0)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamIMUCompassNoise(sigma_theta_deg, offset);
            return false;
        }

        virtual bool setParamRoadThetaNoise(double sigma_theta_deg)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamRoadThetaNoise(sigma_theta_deg);
            return false;
        }

        virtual bool setParamCameraOffset(double lin_offset, double ang_offset_deg = 0)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamCameraOffset(lin_offset, ang_offset_deg);
            return false;
        }

        virtual bool setParamPOINoise(double sigma_position, double sigma_theta_deg)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamPOINoise(sigma_position, sigma_theta_deg);
            return false;
        }

        virtual bool setParamVPSNoise(double sigma_position, double sigma_theta_deg)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamVPSNoise(sigma_position, sigma_theta_deg);
            return false;
        }

        virtual bool setParamPOINoiseRelative(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamPOINoiseRelative(sigma_rel_dist, sigma_rel_theta_deg, sigma_position);
            return false;
        }

        virtual bool setParamVPSNoiseRelative(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamVPSNoiseRelative(sigma_rel_dist, sigma_rel_theta_deg, sigma_position);
            return false;
        }

        virtual bool setParamIntersectClsNoise(double sigma_position)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamIntersectClsNoise(sigma_position);
            return false;
        }

        virtual bool setParamIntersect3CameraClsNoise(double sigma_position)
        {
            cv::AutoLock lock(m_mutex);
            if (m_ekf) return m_ekf->setParamIntersect3CameraClsNoise(sigma_position);
            return false;
        }


        virtual bool applyGPS(const LatLon& ll, Timestamp time = -1, double confidence = -1)
        {
            return applyGPS(toMetric(ll), time, confidence);
        }

        virtual bool applyGPS(const Point2& xy, Timestamp time = -1, double confidence = -1)
        {
            cv::AutoLock lock(m_mutex);
            Point2 smoothed_xy = xy;
            if (m_enable_gps_smoothing) smoothed_xy = getSmoothedGPS(xy, time);
            if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (!m_ekf->applyGPS(smoothed_xy, time, confidence)) return false;
            saveObservation(ObsData::OBS_GPS, smoothed_xy, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyOdometry(Pose2 odometry_pose, Timestamp time = -1, double confidence = -1)
        {
            if(!isPoseStabilized()) return false;

            cv::AutoLock lock(m_mutex);
            if (m_pose_history.empty()) return false;
            if (!m_ekf->applyOdometry(odometry_pose, time, confidence)) return false;
            saveObservation(ObsData::OBS_ODO, odometry_pose, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyIMUCompass(double odometry_theta, Timestamp time = -1, double confidence = -1)
        {
            if(!isPoseStabilized()) return false;

            cv::AutoLock lock(m_mutex);
            if (m_pose_history.empty()) return false;
            if (!m_ekf->applyIMUCompass(odometry_theta, time, confidence)) return false;
            saveObservation(ObsData::OBS_IMU, odometry_theta, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyRoadTheta(double theta, Timestamp time = -1, double confidence = -1)
        {
            cv::AutoLock lock(m_mutex);
            if (m_pose_history.empty()) return false;
            if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
            {
                if (!backtrackingEKF(time, ObsData(ObsData::OBS_RoadTheta, theta, time, confidence))) return false;
                return applyPathLocalizer(m_ekf->getPose(), time);
            }
            else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (!m_ekf->applyRoadTheta(theta, time, confidence)) return false;
            saveObservation(ObsData::OBS_RoadTheta, theta, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyPOI(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false)
        {
            // recover pose
            Pose2 pose = getPose();
            double poi_theta = pose.theta + relative.ang;
            double rx = clue_xy.x - relative.lin * cos(poi_theta);
            double ry = clue_xy.y - relative.lin * sin(poi_theta);
            pose.x = rx;
            pose.y = ry;

            cv::AutoLock lock(m_mutex);
            if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
            {
                if (!backtrackingEKF(time, ObsData(ObsData::OBS_POI, clue_xy, relative, time, confidence))) return false;
                return applyPathLocalizer(m_ekf->getPose(), time);
            }
            else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (use_relative_model && !m_ekf->applyPOI(clue_xy, relative, time, confidence, use_relative_model)) return false;
            if (!use_relative_model && !m_ekf->applyPOI(pose, time, confidence)) return false;
            saveObservation(ObsData::OBS_POI, clue_xy, relative, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyVPS(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1, bool use_relative_model = false)
        {
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
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyIntersectCls(const Point2& xy, Timestamp time = -1, double confidence = -1)
        {
            return true;
            
            cv::AutoLock lock(m_mutex);
            if (m_pose_history.empty()) return false;
            if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
            {
                if (!backtrackingEKF(time, ObsData(ObsData::OBS_IntersectCls, xy, time, confidence))) return false;
                return applyPathLocalizer(m_ekf->getPose(), time);
            }
            else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (!m_ekf->applyIntersectCls(xy, time, confidence)) return false;
            saveObservation(ObsData::OBS_IntersectCls, xy, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        virtual bool applyIntersect3CameraCls(const Point2& xy, Timestamp time = -1, double confidence = -1)
        {
            return true;
            
            cv::AutoLock lock(m_mutex);
            if (m_pose_history.empty()) return false;
            if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
            {
                if (!backtrackingEKF(time, ObsData(ObsData::OBS_Intersect3CameraCls, xy, time, confidence))) return false;
                return applyPathLocalizer(m_ekf->getPose(), time);
            }
            else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
            if (!m_ekf->applyIntersect3CameraCls(xy, time, confidence)) return false;
            saveObservation(ObsData::OBS_Intersect3CameraCls, xy, time, confidence);
            saveEKFState(m_ekf, time);
            return applyPathLocalizer(m_ekf->getPose(), time);
        }

        /**
         * Apply RoadLR result
         * @param lr_cls The lateral position of camera w.r.t. road (0: LEFT_SIDE_OF_ROAD, 1: uncertain, 2: RIGHT_SIDE_OF_ROAD)
         */
        virtual bool applyRoadLR(int lr_cls, Timestamp time = -1, double confidence = -1)
        {
            cv::AutoLock lock(m_mutex);
            if (lr_cls == Edge::LR_NONE || m_pose_history.empty()) return false;
            m_pose_history.back().lr_side = lr_cls;
            return true;
        }

        virtual Pose2 getPose(Timestamp* timestamp = nullptr) const
        {
            cv::AutoLock lock(m_mutex);
            if (timestamp) *timestamp = m_timestamp;
            return m_pose;
        }

        virtual double getPoseConfidence(Timestamp* timestamp = nullptr) const
        {
            cv::AutoLock lock(m_mutex);
            return m_ekf->getPoseConfidence(timestamp);
        }

        virtual bool isPoseInitialized()
        {
            cv::AutoLock lock(m_mutex);
            return m_ekf->isPoseInitialized();
        }

        virtual bool isPoseStabilized()
        {
            cv::AutoLock lock(m_mutex);
            return m_ekf->isPoseStabilized();
        }

        virtual const cv::Mat getState() const
        {
            cv::AutoLock lock(m_mutex);
            return m_ekf->getState();
        }

        virtual const cv::Mat getStateCov() const
        {
            cv::AutoLock lock(m_mutex);
            return m_ekf->getStateCov();
        }

        void printInternalState()
        {
            cv::AutoLock lock(m_mutex);
            printf("Observation history\n");
            for (int i = 0; i < m_observation_history.data_count(); i++)
            {
                dg::ObsData data = m_observation_history[i];
                printf("\t[%d] %lf, type=%d, v1=%lf, v2=%lf, v3=%lf, v4=%lf\n", i, data.timestamp, data.type, data.v1, data.v2, data.v3, data.v4);
            }
            printf("EKF history\n");
            for (int i = 0; i < m_state_history.data_count(); i++)
            {
                dg::EKFState s = m_state_history[i];
                printf("\t[%d] %lf, %lf, x=%lf, y=%lf, theta=%lf\n", i, s.timestamp, s.state_time, s.state_vec.at<double>(0), s.state_vec.at<double>(1), s.state_vec.at<double>(2));
            }
        }

    protected:
        bool initInternalVariables()
        {
            m_state_history.resize(m_history_size);
            m_observation_history.resize(m_history_size);
            m_pose_history.resize(m_history_size);
            m_projected_pose_history.resize(m_history_size);
            m_gps_state = Point2T(Point2(0, 0), 0);
            m_gps_velocity = Point2(0, 0);
            m_timestamp = -1;
            return true;
        }

        void saveEKFState(cv::Ptr<dg::EKFLocalizer> ekf, Timestamp timestamp)
        {
            EKFState state;
            state.state_vec = ekf->getState().clone();
            state.state_cov = ekf->getStateCov().clone();
            state.state_time = ekf->getLastUpdateTime();
            ekf->getImuState(state.imu_angle, state.imu_time);
            state.timestamp = timestamp;
            m_state_history.push_back(state);
            m_pose_history.push_back(Pose2T(ekf->getPose(), timestamp));
        }

        bool applyPathLocalizer(Pose2 pose, Timestamp timestamp)
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
                if (!path.empty() && m_enable_path_projection) m_pose = getPathPose(map, &path, pose, m_pose_history, m_projected_pose_history, out_of_path);
                else if (m_enable_map_projection) m_pose = getMapPose(map, pose, m_pose_history, m_projected_pose_history);
            }
            m_shared->releaseMapLock();
            if (out_of_path) m_shared->procOutOfPath(m_pose);
            m_projected_pose_history.push_back(m_pose);

            return true;
        }

        bool backtrackingEKF(Timestamp rollback_time, const ObsData& delayed_observation)
        {
            if (m_ekf.empty()) return false;

            // find rollback point
            int ndata = m_observation_history.data_count();
            int restart_i = ndata;
            while (restart_i > 0 && m_observation_history[restart_i - 1].timestamp > rollback_time) restart_i--;
            if (restart_i >= ndata || restart_i <= 0) return false;

            // rollback ekf state
            int rollback_point = restart_i - 1;
            EKFState state = m_state_history[rollback_point];
            bool ok = m_ekf->resetEKFState(state.state_vec, state.state_cov, state.state_time, state.imu_angle, state.imu_time);
            if (!ok) return false;

            // rollback state
            m_state_history.erase(restart_i, -1);
            m_pose_history.erase(restart_i, -1);

            // re-apply observations after rollback point
            restart_i = m_observation_history.insert(restart_i, delayed_observation);
            for (int i = restart_i; i < m_observation_history.data_count(); i++)
            {
                applyObservation(m_observation_history[i]);
                saveEKFState(m_ekf, m_observation_history[i].timestamp);
            }
            return true;
        }

        bool applyObservation(const ObsData& obs)
        {
            if (obs.type == ObsData::OBS_GPS)
            {
                return m_ekf->applyGPS(Point2(obs.v1, obs.v2), obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_ODO)
            {
                return m_ekf->applyOdometry(Pose2(obs.v1, obs.v2, obs.v3), obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_IMU)
            {
                return m_ekf->applyIMUCompass(obs.v1, obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_RoadTheta)
            {
                return m_ekf->applyRoadTheta(obs.v1, obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_POI)
            {
                return m_ekf->applyPOI(Point2(obs.v1, obs.v2), Polar2(obs.v3, obs.v4), obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_VPS)
            {
                return m_ekf->applyVPS(Point2(obs.v1, obs.v2), Polar2(obs.v3, obs.v4), obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_IntersectCls)
            {
                return m_ekf->applyIntersectCls(Point2(obs.v1, obs.v2), obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_Intersect3CameraCls)
            {
                return m_ekf->applyIntersect3CameraCls(Point2(obs.v1, obs.v2), obs.timestamp, obs.confidence);
            }
            else if (obs.type == ObsData::OBS_RoadLR)
            {
                // ignore
            }
            return false;
        }

        void saveObservation(int type, double theta, Timestamp time, double conf)
        {
            m_observation_history.push_back(ObsData(type, theta, time, conf));
        }

        void saveObservation(int type, Point2 xy, Timestamp time, double conf)
        {
            m_observation_history.push_back(ObsData(type, xy, time, conf));
        }

        void saveObservation(int type, Pose2 pose, Timestamp time, double conf)
        {
            m_observation_history.push_back(ObsData(type, pose, time, conf));
        }

        void saveObservation(int type, Point2 xy, Polar2 lin_ang, Timestamp time, double conf)
        {
            m_observation_history.push_back(ObsData(type, xy, lin_ang, time, conf));
        }

        Point2 getSmoothedGPS(const Point2& xy, Timestamp time)
        {
            if (m_gps_state.timestamp <= 0)
            {
                m_gps_state = Point2T(xy, time);
                return xy;
            }

            // alpha-beta filtering with decaying velocity
            double dt = time - m_gps_state.timestamp;
            Point2 pred = m_gps_state + m_gps_velocity * dt;
            Point2 residual = xy - pred;
            m_gps_state = pred + m_smoothing_alpha * residual;
            m_gps_state.timestamp = time;
            m_gps_velocity = m_gps_velocity * m_smoothing_velocity_decaying + m_smoothing_beta * residual;

            return m_gps_state;
        }

        Pose2 m_pose;
        Timestamp m_timestamp;
        cv::Ptr<dg::EKFLocalizer> m_ekf;
        RingBuffer<EKFState> m_state_history;
        RingBuffer<ObsData> m_observation_history;
        RingBuffer<Pose2LR> m_pose_history;
        RingBuffer<Pose2> m_projected_pose_history;
        Point2T m_gps_state;
        Point2 m_gps_velocity;

    }; // End of 'DGLocalizer'


} // End of 'dg'

#endif // End of '__LOCALIZER_HPP__'
