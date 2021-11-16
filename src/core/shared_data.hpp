#ifndef __DG_SHARED_DATA__
#define __DG_SHARED_DATA__

#include "core/basic_type.hpp"
#include "core/map.hpp"
#include "core/path.hpp"

namespace dg
{

/**
 * @brief Shared Data
 * It defines interfaces for accessing and processing shared data of DeepGuider system
 */
class SharedData
{
public:
    /** Get a pointer to shared map data */
    virtual Map* getMap()
    {
        return &m_map;
    }

    /** Get a pointer to shared map data with a lock */
    virtual Map* getMapLocked()
    {
        m_mutex_map.lock();
        return &m_map;
    }

    /** Release lock for shared map data */
    void releaseMapLock()
    {
        m_mutex_map.unlock();
    }

    /** Set shared map data */
    virtual bool setMap(const Map& map)
    {
        cv::AutoLock lock(m_mutex_map);
        return map.copyTo(m_map);
    }

    /** Get a copy of shared path data */
    virtual Path getPath() 
    { 
        cv::AutoLock lock(m_mutex_path);
        return m_path;
    }

    /** Set shared path data */
    virtual void setPath(const Path& path)
    {
        cv::AutoLock lock(m_mutex_path);
        m_path = path;
    }

    /** Get gps position of a metric position by using shared converter */
    virtual LatLon toLatLon(const Point2& metric)
    {
        cv::AutoLock lock(m_mutex_map);
        return m_map.toLatLon(metric);
    }

    /** Get metric position of a gps position by using shared converter */
    virtual Point2 toMetric(const LatLon& ll)
    {
        cv::AutoLock lock(m_mutex_map);
        return m_map.toMetric(ll);
    }

    /** Get UTM position of a gps position by using shared converter */
    virtual Point2UTM cvtLatLon2UTM(const LatLon& ll)
    {
        cv::AutoLock lock(m_mutex_map);
        return m_map.cvtLatLon2UTM(ll);
    }

protected:
    dg::Map m_map;
    dg::Path m_path;

    cv::Mutex m_mutex_map;
    cv::Mutex m_mutex_path;
    
}; // End of 'SharedData'

/**
 * @brief Shared Interface
 * It defines interfaces shared over deepguider submodules to communicate with main system.
 */
class SharedInterface : public SharedData
{
public:
    /**
     * Get current metric pose
     * @param[out] Timestamp of the current pose
     * @return Return current metric pose
     */
    virtual Pose2 getPose(Timestamp* timestamp = nullptr) const { return Pose2(); }

    /**
     * Get current LatLon pose
     * @param[out] Timestamp of the current pose
     * @return Return current LatLon pose
     */
    virtual LatLon getPoseGPS(Timestamp* timestamp = nullptr) const { return LatLon(); }

    /**
     * Get current topometric pose
     * @param[out] Timestamp of the current pose
     * @return Return current topometric pose
     */
    virtual TopometricPose getPoseTopometric(Timestamp* timestamp = nullptr) const { return TopometricPose(); }

    /**
     * Get confidence of current pose
     * @return Return confidence value (0 ~ 1)
     */
    virtual double getPoseConfidence(Timestamp* timestamp = nullptr) const { return -1; }

    /**
     * Process out-of-path event
     * @param curr_pose A given current pose which is out-of-path
     * @return Return True if the processing is successful, False otherwise
     */
    virtual bool procOutOfPath(const Point2& curr_pose) { return true; }
};


} // End of 'dg'

#endif // End of '__DG_SHARED_DATA__'
