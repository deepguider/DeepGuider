#ifndef __INTERSECTION_LOCALIZER__
#define __INTERSECTION_LOCALIZER__

#include "dg_core.hpp"
#include "intersection_cls/intersection_cls.hpp"

using namespace std;

namespace dg
{
    /**
    * @brief Intersection-based Localizer
    */
    class IntersectionLocalizer: public IntersectionClassifier
    {
        //configurable parameters
        double m_param_update_inc = 0.1;    // for intersection observation
        double m_param_update_dec = 0.1;    // for non-intersection observation
        double m_param_state_threshold = 0.5;

    public:
        enum {NONE_INTERSECTION = 0, INTERSECTION = 1};

        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/intersection_cls")
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            if (!IntersectionClassifier::initialize("intersection_cls", py_module_path.c_str())) return false;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, dg::Point2& xy, dg::Polar2& relative, double& confidence, bool& valid_xy)
        {
            cv::AutoLock lock(m_mutex);
            if (m_shared == nullptr) return false;
            valid_xy = false;

            if (!IntersectionClassifier::apply(image, image_time)) return false;

            // apply state filtering
            int observed_cls = m_intersect.cls;
            int state_prev = m_state;
            m_state = simpleStateFiltering(observed_cls);

            // apply classification result only at the end of intersection
            if (state_prev == INTERSECTION && m_state == NONE_INTERSECTION)
            {
                Pose2 pose = m_shared->getPose();
                Path* path = m_shared->getPathLocked();
                if (path && !path->empty()) valid_xy = findNearestPathJunction(*path, pose, xy);
                else valid_xy = findNearestMapJunction(pose, xy);
                m_shared->releasePathLock();
                relative = Polar2(-1, CV_PI);
                confidence = m_intersect.confidence;
            }
            return true;
        }

    protected:
        int simpleStateFiltering(int observation)
        {
            if (observation == NONE_INTERSECTION) m_state_score -= m_param_update_dec;
            if (observation == INTERSECTION) m_state_score += m_param_update_inc;
            if (m_state_score > 1) m_state_score = 1;
            if (m_state_score < 0) m_state_score = 0;

            if (m_state_score >= m_param_state_threshold) return INTERSECTION;
            else return NONE_INTERSECTION;
        }

        bool findNearestPathJunction(const Path& path, const dg::Pose2& pose, dg::Point2& xy)
        {
            Map* map = m_shared->getMap();
            if (map == nullptr || map->isEmpty()) return false;

            double min_d2 = DBL_MAX;
            for (auto it = path.pts.begin(); it != path.pts.end(); it++)
            {
                Node* node = map->getNode(it->node_id);
                if (node == nullptr || node->type != Node::NODE_JUNCTION) continue;
                double d2 = (pose.x - it->x) * (pose.x - it->x) + (pose.y - it->y) * (pose.y - it->y);
                if (d2 < min_d2)
                {
                    min_d2 = d2;
                    xy = *it;
                }
            }
            return (min_d2 < DBL_MAX);
        }

        bool findNearestMapJunction(const dg::Pose2& pose, dg::Point2& xy)
        {
            Map* map = m_shared->getMap();
            if (map == nullptr || map->isEmpty()) return false;

            double min_d2 = DBL_MAX;
            for (auto it = map->getHeadNode(); it != map->getTailNode(); it++)
            {
                if (it->type != Node::NODE_JUNCTION) continue;
                double d2 = (pose.x - it->x) * (pose.x - it->x) + (pose.y - it->y) * (pose.y - it->y);
                if (d2 < min_d2)
                {
                    min_d2 = d2;
                    xy = *it;
                }
            }
            return (min_d2 < DBL_MAX);
        }

        double m_state_score = 0;           // initiallly, non-intersection
        int m_state = NONE_INTERSECTION;

        SharedInterface* m_shared = nullptr;
        mutable cv::Mutex m_mutex;
    };

} // End of 'dg'

#endif // End of '__INTERSECTION_CLS__'
