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
        double m_param_update_inc = 0.05;    // for intersection observation
        double m_param_update_dec = 0.05;    // for non-intersection observation
        double m_param_state_threshold = 0.6;
        double m_param_state_upper_bound = 1.0;
        double m_param_state_lower_bound = 0;

    public:
        enum {NONE_INTERSECTION = 0, INTERSECTION = 1};

        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/intersection_cls")
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            if (!IntersectionClassifier::initialize("intersection_cls", py_module_path.c_str())) return false;
            return (m_shared != nullptr);
        }

        bool initialize_without_python(SharedInterface* shared)
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, dg::Point2& xy, double& xy_confidence, bool& xy_valid)
        {
            cv::AutoLock lock(m_mutex);
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();
            xy_valid = false;

            if (!IntersectionClassifier::apply(image, image_time)) return false;

            // apply state filtering
            int observed_cls = m_result.cls;
            int state_prev = m_state;
            m_state = simpleStateFiltering(observed_cls);

            // apply classification result only at the end of intersection
            if (state_prev == INTERSECTION && m_state == NONE_INTERSECTION)
            {
                Path path = m_shared->getPath();
                if (!path.empty()) xy_valid = findNearestPathJunction(path, pose, xy);
                else xy_valid = findNearestMapJunction(pose, xy);
                xy_confidence = m_result.confidence;
            }
            return true;
        }

        bool applyPreprocessed(double cls, double cls_conf, const dg::Timestamp data_time, dg::Point2& xy, double& xy_confidence, bool& xy_valid)
        {
            cv::AutoLock lock(m_mutex);
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();
            xy_valid = false;

            m_result.cls = (int)(cls + 0.5);
            m_result.confidence = cls_conf;
            m_timestamp = data_time;

            // apply state filtering
            int observed_cls = (int)(cls + 0.5);
            int state_prev = m_state;
            m_state = simpleStateFiltering(observed_cls);

            // apply classification result only at the end of intersection
            if (state_prev == INTERSECTION && m_state == NONE_INTERSECTION)
            {
                Path path = m_shared->getPath();
                if (!path.empty()) xy_valid = findNearestPathJunction(path, pose, xy);
                else xy_valid = findNearestMapJunction(pose, xy);
                xy_confidence = cls_conf;
            }
            return true;
        }


    protected:
        int simpleStateFiltering(int observation)
        {
            if (observation == NONE_INTERSECTION) m_state_score -= m_param_update_dec;
            if (observation == INTERSECTION) m_state_score += m_param_update_inc;
            if (m_state_score > m_param_state_upper_bound) m_state_score = m_param_state_upper_bound;
            if (m_state_score < m_param_state_lower_bound) m_state_score = m_param_state_lower_bound;

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
