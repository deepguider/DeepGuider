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
struct LocState
{
    /** The state variable */
    cv::Mat state_vec;

    /** The state covariance */
    cv::Mat state_cov;

    /** The timestamp */
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

        /** IMU data (v1: odometry theta) */
        OBS_IMU = 1,

        /** POI data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
        OBS_POI = 2,

        /** VPS data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
        OBS_VPS = 3,

        /** Intersection classifier data (v1: utm x, v2: utm y, v3: lin, v4: ang) */
        OBS_IntersectCls = 4,

        /** VPS_LR data (v1 = 0: uncertain, v1 = 1: road is left, v1 = 2: road is right) */
        OBS_LR = 5,

        /** RoadTheta data (v1: theta) */
        OBS_RoadTheta = 6
    };

    /**
     * The type of observation data
     * @see OBS_GPS, OBS_IMU, OBS_POI, OBS_VPS, OBS_VPS_LR, OBS_RoadTheta, OBS_IntersectCls
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
    ObsData() {}

    /** Constructor */
    ObsData(int _type, double theta, Timestamp time, double conf) : type(_type), v1(theta), timestamp(time), confidence(conf) {}

    /** Constructor */
    ObsData(int _type, Point2 xy, Timestamp time, double conf) : type(_type), v1(xy.x), v2(xy.y), timestamp(time), confidence(conf) {}

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
    int m_history_size = 1000;
    bool m_enable_path_projection = true;
    bool m_enable_map_projection = false;
    bool m_enable_backtracking_ekf = true;
    bool m_enable_gps_smoothing = true;
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
        if (baselocalizer_name == "EKFLocalizer") m_ekf = cv::makePtr<dg::EKFLocalizer>();
        else if (baselocalizer_name == "EKFLocalizerZeroGyro") m_ekf = cv::makePtr<dg::EKFLocalizerZeroGyro>();
        else if (baselocalizer_name == "EKFLocalizerHyperTan") m_ekf = cv::makePtr<dg::EKFLocalizerHyperTan>();
        else if (baselocalizer_name == "EKFLocalizerSinTrack") m_ekf = cv::makePtr<dg::EKFLocalizerSinTrack>();
        setShared(shared);
        initInternalVariables();
        return (m_shared != nullptr && !m_ekf.empty());
    }

    virtual bool setShared(SharedInterface* shared)
    {
        if (m_ekf) m_ekf->setShared(shared);
        return BaseLocalizer::setShared(shared);
    }

    virtual bool setParamMotionNoise(double sigma_linear_velocity, double sigma_angular_velocity_deg, double cov_lin_ang = 0)
    {
        if (m_ekf) return m_ekf->setParamMotionNoise(sigma_linear_velocity, sigma_angular_velocity_deg, cov_lin_ang);
        return false;
    }

    virtual bool setParamGPSNoise(double sigma_normal, double sigma_deadzone = -1)
    {
        if (m_ekf) return m_ekf->setParamGPSNoise(sigma_normal, sigma_deadzone);
        return false;
    }

    virtual bool setParamGPSOffset(double lin_offset, double ang_offset_deg = 0)
    {
        if (m_ekf) return m_ekf->setParamGPSOffset(lin_offset, ang_offset_deg);
        return false;
    }

    virtual bool setParamIMUCompassNoise(double sigma_theta_deg, double offset = 0)
    {
        if (m_ekf) return m_ekf->setParamIMUCompassNoise(sigma_theta_deg, offset);
        return false;
    }

    virtual bool setParamRoadThetaNoise(double sigma_theta_deg, double offset = 0)
    {
        if (m_ekf) return m_ekf->setParamRoadThetaNoise(sigma_theta_deg, offset);
        return false;
    }

    virtual bool setParamCameraOffset(double lin_offset, double ang_offset_deg = 0)
    {
        if (m_ekf) return m_ekf->setParamCameraOffset(lin_offset, ang_offset_deg);
        return false;
    }

    virtual bool setParamPOINoise(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position = 1)
    {
        if (m_ekf) return m_ekf->setParamPOINoise(sigma_rel_dist, sigma_rel_theta_deg, sigma_position);
        return false;
    }

    virtual bool setParamVPSNoise(double sigma_rel_dist, double sigma_rel_theta_deg, double sigma_position = 1)
    {
        if (m_ekf) return m_ekf->setParamVPSNoise(sigma_rel_dist, sigma_rel_theta_deg, sigma_position);
        return false;
    }

    virtual bool setParamIntersectClsNoise(double sigma_position)
    {
        if (m_ekf) return m_ekf->setParamIntersectClsNoise(sigma_position);
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
        if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
        {
            if (!backtrackingEKF(time, ObsData(ObsData::OBS_GPS, xy, time, confidence))) return false;
            return applyPathLocalizer(m_ekf->getPose(), time);
        }
        else if(time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
        if (m_enable_gps_smoothing) smoothed_xy = getSmoothedGPS(xy, time);
        if (!m_ekf->applyGPS(smoothed_xy, time, confidence)) return false;
        saveObservation(ObsData::OBS_GPS, xy, time, confidence);
        saveEKFState(m_ekf, time);
        return applyPathLocalizer(m_ekf->getPose(), time);
    }

    virtual bool applyIMUCompass(double odometry_theta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
        {
            if (!backtrackingEKF(time, ObsData(ObsData::OBS_IMU, odometry_theta, time, confidence))) return false;
            return applyPathLocalizer(m_ekf->getPose(), time);
        }
        else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
        if (!m_ekf->applyIMUCompass(odometry_theta, time, confidence)) return false;
        saveObservation(ObsData::OBS_IMU, odometry_theta, time, confidence);
        saveEKFState(m_ekf, time);
        return true;
    }

    virtual bool applyRoadTheta(double theta, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
        {
            if (!backtrackingEKF(time, ObsData(ObsData::OBS_RoadTheta, theta, time, confidence))) return false;
            return applyPathLocalizer(m_ekf->getPose(), time);
        }
        else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
        if (!m_ekf->applyRoadTheta(theta, time, confidence)) return false;
        saveObservation(ObsData::OBS_RoadTheta, theta, time, confidence);
        saveEKFState(m_ekf, time);
        return true;
    }

    virtual bool applyPOI(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
        {
            if (!backtrackingEKF(time, ObsData(ObsData::OBS_POI, clue_xy, relative, time, confidence))) return false;
            return applyPathLocalizer(m_ekf->getPose(), time);
        }
        else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
        if (!m_ekf->applyPOI(clue_xy, relative, time, confidence)) return false;
        saveObservation(ObsData::OBS_POI, clue_xy, relative, time, confidence);
        saveEKFState(m_ekf, time);
        return applyPathLocalizer(m_ekf->getPose(), time);
    }

    virtual bool applyVPS(const Point2& clue_xy, const Polar2& relative = Polar2(-1, CV_PI), Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        if (m_enable_backtracking_ekf && time < m_ekf->getLastUpdateTime())
        {
            if (!backtrackingEKF(time, ObsData(ObsData::OBS_VPS, clue_xy, relative, time, confidence))) return false;
            return applyPathLocalizer(m_ekf->getPose(), time);
        }
        else if (time < m_ekf->getLastUpdateTime()) time = m_ekf->getLastUpdateTime();
        if (!m_ekf->applyVPS(clue_xy, relative, time, confidence)) return false;
        saveObservation(ObsData::OBS_VPS, clue_xy, relative, time, confidence);
        saveEKFState(m_ekf, time);
        return applyPathLocalizer(m_ekf->getPose(), time);
    }

    virtual bool applyIntersectCls(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
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

    /**
     * Apply VPS_LR result
     * @param lr_result The lateral position of camera w.r.t. road (0: left sideway, 1: uncertain, 2: right sideway)
     */
    virtual bool applyVPS_LR(double lr_result, Timestamp time = -1, double confidence = -1)
    {
        cv::AutoLock lock(m_mutex);
        saveObservation(ObsData::OBS_LR, lr_result, time, confidence);
        // TODO: apply the result to path projector
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
        return m_ekf->getPoseConfidence(timestamp);
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

protected:
    bool initInternalVariables()
    {
        m_state_history.resize(m_history_size);
        m_observation_history.resize(m_history_size);
        m_pose_history.resize(m_history_size);
        m_gps_state = Point2T(Point2(0, 0), 0);
        m_gps_velocity = Point2(0, 0);
        return true;
    }

    void saveEKFState(cv::Ptr<dg::EKFLocalizer> ekf, Timestamp timestamp)
    {
        LocState state;
        state.state_vec = ekf->getState().clone();
        state.state_cov = ekf->getStateCov().clone();
        state.timestamp = timestamp;
        m_state_history.push_back(state);
        m_pose_history.push_back(Pose2T(ekf->getPose(), timestamp));
    }

    bool applyPathLocalizer(Pose2 pose, Timestamp timestamp)
    {
        if (!m_enable_path_projection)
        {
            m_pose = pose;
            m_timestamp = timestamp;
            return true;
        }

        if (m_shared == nullptr) return false;

        Pose2 prj_pose = pose;
        Map* map = m_shared->getMapLocked();
        bool out_of_path = false;
        if (map)
        {
            Path* path = m_shared->getPathLocked();
            if (path) prj_pose = getPathPose(map, *path, pose, m_pose_history, out_of_path);
            else if (m_enable_map_projection) prj_pose = getMapPose(map, pose, m_pose, m_pose_history.empty());
            m_shared->releasePathLock();
        }
        m_shared->releaseMapLock();

        m_pose = prj_pose;
        m_timestamp = timestamp;

        if (out_of_path) m_shared->procOutOfPath(m_pose);

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
        LocState state = m_state_history[rollback_point];
        bool ok = m_ekf->resetEKFState(state.state_vec, state.state_cov, state.timestamp);
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
        else if (obs.type == ObsData::OBS_LR)
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
    RingBuffer<LocState> m_state_history;
    RingBuffer<ObsData> m_observation_history;
    RingBuffer<Pose2T> m_pose_history;
    Point2T m_gps_state;
    Point2 m_gps_velocity;

}; // End of 'DGLocalizer'


} // End of 'dg'

#endif // End of '__LOCALIZER_HPP__'

