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
    /** Get a poiter to shared map data */
    virtual Map* getMap()
    {
        cv::AutoLock lock(m_mutex_map);
        return m_map;
    }

    /** Get a poiter to shared map data with a lock */
    virtual Map* getMapLocked()
    {
        setMapLock();
        return m_map;
    }

    /** Get a poiter to shared path data */
    virtual Path* getPath() 
    { 
        cv::AutoLock lock(m_mutex_path);
        return m_path;
    }

    /** Get a poiter to shared path data with a lock */
    virtual Path* getPathLocked()
    {
        setPathLock();
        return m_path;
    }

    /** Set shared map data */
    virtual bool setMap(const Map& map)
    {
        cv::AutoLock lock(m_mutex_map);
        if (m_map.empty()) m_map = new Map();
        map.copyTo(m_map);
        return true;
    }

    /** Set shared path data */
    virtual bool setPath(const Path& path)
    {
        cv::AutoLock lock(m_mutex_path);
        if (m_path.empty()) m_path = new Path();
        *m_path = path;
        return true;
    }

    /** Get gps position of a metric position by using shared converter */
    virtual LatLon toLatLon(const Point2& metric)
    {
        cv::AutoLock lock(m_mutex_map);
        if (m_map == nullptr) return LatLon();
        return m_map->toLatLon(metric);
    }

    /** Get metric position of a gps position by using shared converter */
    virtual Point2 toMetric(const LatLon& ll)
    {
        cv::AutoLock lock(m_mutex_map);
        if (m_map == nullptr) return Point2();
        return m_map->toMetric(ll);
    }

    /** Set a lock for shared map data */
    void setMapLock() 
    {
        m_mutex_map.lock();
    }

    /** Release lock for shared map data */
    void releaseMapLock()
    {
        m_mutex_map.unlock();
    }
    
    /** Set a lock for shared path data */
    void setPathLock()
    { 
        m_mutex_path.lock();
    }
     
    /** Release lock for shared path data */
    void releasePathLock() 
    {
        m_mutex_path.unlock(); 
    }
    
protected:
    cv::Ptr<Map> m_map;
    cv::Ptr<Path> m_path;

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
    virtual Pose2 getPose(Timestamp* timestamp = nullptr) const = 0;

    /**
     * Get current LatLon pose
     * @param[out] Timestamp of the current pose
     * @return Return current LatLon pose
     */
    virtual LatLon getPoseGPS(Timestamp* timestamp = nullptr) const = 0;

    /**
     * Get current topometric pose
     * @param[out] Timestamp of the current pose
     * @return Return current topometric pose
     */
    virtual TopometricPose getPoseTopometric(Timestamp* timestamp = nullptr) const = 0;

    /**
     * Get confidence of current pose
     * @return Return confidence value (0 ~ 1)
     */
    virtual double getPoseConfidence(Timestamp* timestamp = nullptr) const = 0;

    /**
     * Process out-of-path event
     * @param curr_pose A given current pose which is out-of-path
     * @return Return True if the processing is successful, False otherwise
     */
    virtual bool procOutOfPath(const Point2& curr_pose) { return true; }
};


} // End of 'dg'

#endif // End of '__DG_SHARED_DATA__'
