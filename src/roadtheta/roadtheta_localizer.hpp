#ifndef __ROAD_THETA_LOCALIZER__
#define __ROAD_THETA_LOCALIZER__

#include "dg_core.hpp"
#include "roadtheta/roadtheta.hpp"
#include "localizer/path_projector.hpp"
#include "utils/opencx.hpp"

namespace dg
{
    /**
    * @brief RoadTheta Localizer
    */
    class RoadThetaLocalizer : public RoadTheta
    {
    public:
        bool initialize(SharedInterface* shared)
        {
            if (!RoadTheta::initialize()) return false;

            cv::AutoLock lock(m_localizer_mutex);
            m_shared = shared;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, double& theta , double& confidence)
        {
            if (!RoadTheta::apply(image, image_time)) return false;

            cv::AutoLock lock(m_localizer_mutex);
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();
            double theta_relative = m_result.theta;
            if (!getAbsoluteOrientation(theta_relative, pose, theta)) return false;
            confidence = m_result.confidence;
            return true;
        }

    protected:
        bool getAbsoluteOrientation(double theta_relative, const dg::Pose2& pose, double& theta_absolute)
        {
            // path-based orientation if path defined
            Path path = m_shared->getPath();
            if (!path.empty())
            {
                Pose2 path_pose = dg::Map::getNearestPathPose(path, pose);
                theta_absolute = cx::trimRad(path_pose.theta + theta_relative);
                return true;
            }

            // map-based orientation if no path
            Map* map = m_shared->getMap();
            if (map == nullptr || map->isEmpty()) return false;
            Pose2 map_pose = map->getNearestMapPose(pose);
            theta_absolute = cx::trimRad(map_pose.theta + theta_relative);
            return true;
        }

        SharedInterface* m_shared = nullptr;
        cv::Mutex m_localizer_mutex;
    };

} // End of 'dg'

#endif // End of '__ROAD_THETA_LOCALIZER__'
