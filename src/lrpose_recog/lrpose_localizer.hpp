#ifndef __LRPOSE_RECOGNIZER_LOCALIZER__
#define __LRPOSE_RECOGNIZER_LOCALIZER__

#include "dg_core.hpp"
#include "lrpose_recog/lrpose_recognizer.hpp"

using namespace std;

namespace dg
{
    /**
    * @brief Intersection-based Localizer
    */
    class LRLocalizer: public LRPoseRecognizer
    {
        //configurable parameters
        double m_param_update_inc = 0.1;    // for intersection observation
        double m_param_update_dec = 0.1;    // for non-intersection observation
        double m_param_state_threshold = 0.5;
        double m_param_state_upper_bound = 0.7;
        double m_param_state_lower_bound = 0;

    public:
        enum {LEFT_SIDE_OF_ROAD = 0, UNKNOWN_SIDE_OF_ROAD = 0, RIGHT_SIDE_OF_ROAD = 2};

        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/lrpose_recog")
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            if (!LRPoseRecognizer::initialize("lrpose_recognizer", py_module_path.c_str())) return false;
            return (m_shared != nullptr);
        }

        bool initialize_without_python(SharedInterface* shared)
        {
            cv::AutoLock lock(m_mutex);
            m_shared = shared;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, double& lr_cls, double& lr_confidence)
        {
            /***
             * Input :
             *  const cv::Mat image, const dg::Timestamp image_time
             * Output:
             *  double& lr_cls, double& lr_confidence
             * ***/
            cv::AutoLock lock(m_mutex);
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();

            if (!LRPoseRecognizer::apply(image, image_time)) return false;

            // apply state filtering
            int observed_cls = m_result.cls;  // m_result from python
            int state_prev = m_state;
            m_state = simpleStateFiltering(observed_cls);
            m_shared->releasePathLock();

			lr_cls = m_state;  // filtered cls
            lr_confidence = m_result.confidence;
            return true;
        }

        bool apply(const dg::Timestamp image_time, double cls, double cls_conf, double& lr_cls, double& lr_confidence)
        /*** 
         * Input:
         *  const dg::Timestamp image_time, double cls, double cls_conf
         * Output:
         *  double& lr_cls, double& lr_confidence
         * ***/
        {
            cv::AutoLock lock(m_mutex);
            if (m_shared == nullptr) return false;
            Pose2 pose = m_shared->getPose();

            // apply state filtering
            //int observed_cls = (int)(cls + 0.5);  // round up ?
            //int state_prev = m_state;
            //m_state = simpleStateFiltering(observed_cls);

            // Update variables for draw(). m_result needs to be set by manual when csv dataloader is enabled rather than python.
            m_result.cls = (int)(cls + 0.5);
            m_result.confidence = cls_conf;

            lr_cls = cls;
            lr_confidence = cls_conf;

            return true;
        }


    protected:
        int simpleStateFiltering(int observation)
        {
            if (observation == LEFT_SIDE_OF_ROAD) m_state_score -= m_param_update_dec;
            if (observation == RIGHT_SIDE_OF_ROAD) m_state_score += m_param_update_inc;
            if (m_state_score > m_param_state_upper_bound) m_state_score = m_param_state_upper_bound;
            if (m_state_score < m_param_state_lower_bound) m_state_score = m_param_state_lower_bound;

            if (m_state_score >= m_param_state_threshold) return RIGHT_SIDE_OF_ROAD;
            else return LEFT_SIDE_OF_ROAD;
        }

        double m_state_score = 0;
        int m_state = LEFT_SIDE_OF_ROAD;

        SharedInterface* m_shared = nullptr;
        mutable cv::Mutex m_mutex;
    };

} // End of 'dg'

#endif // End of '__LRPOSE_RECOGNIZER_LOCALIZER__'
