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
            double d_thr = 15;

            // discard roadtheta near map branch point
            Map* map = m_shared->getMap();
            if (map == nullptr || map->isEmpty()) return false;
            cv::Point2d ep;
            dg::Edge* edge = map->getNearestEdge(pose, ep);
            dg::Node* node1 = map->getNode(edge->node_id1);
            dg::Node* node2 = map->getNode(edge->node_id2);
            if (norm(ep - *node1) < d_thr)
            {
                if (node1->edge_ids.size() > 2) return false;
            }
            if (norm(ep - *node2) < d_thr)
            {
                if (node2->edge_ids.size() > 2) return false;
            }

            // path-based orientation if path defined
            Path path = m_shared->getPath();
            if (!path.empty())
            {
                Pose2 path_pose = dg::Map::getNearestPathPose(path, pose);
                theta_absolute = cx::trimRad(path_pose.theta + theta_relative);
                return true;
            }

            // map-based orientation if no path
            double turn_weight = 1.0;
            Pose2 map_pose = map->getNearestMapPose(pose, turn_weight);
            theta_absolute = cx::trimRad(map_pose.theta + theta_relative);
            return true;
        }

        SharedInterface* m_shared = nullptr;
        cv::Mutex m_localizer_mutex;
    };

} // End of 'dg'

#endif // End of '__ROAD_THETA_LOCALIZER__'
