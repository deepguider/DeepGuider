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
        double m_param_update_inc = 0.1;     // for RIGHT_SIDE_OF_ROAD observation
        double m_param_update_dec = 0.1;     // for LEFT_SIDE_OF_ROAD observation
        double m_param_unknown_delta = 0.05; // for UNKNOWN_SIDE_OF_ROAD observation
        double m_param_state_upper_threshold = 0.8;
        double m_param_state_lower_threshold = 0.2;
        double m_param_state_upper_bound = 1;
        double m_param_state_lower_bound = 0;

    public:
        enum {LEFT_SIDE_OF_ROAD = 0, UNKNOWN_SIDE_OF_ROAD = 1, RIGHT_SIDE_OF_ROAD = 2};

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

        bool apply(const cv::Mat image, const dg::Timestamp image_time, int& lr_cls, double& lr_confidence)
        {
            cv::AutoLock lock(m_mutex);
            if (!LRPoseRecognizer::apply(image, image_time)) return false;

            // apply state filtering
            int observed_cls = m_result.cls;  // m_result from python
            m_state = simpleStateFiltering(observed_cls);

			lr_cls = m_state;  // filtered cls
            lr_confidence = m_result.confidence;
            return true;
        }

        bool applyPreprocessed(double observed_cls, double observed_conf, const dg::Timestamp data_time, int& lr_cls, double& lr_confidence)
        {
            cv::AutoLock lock(m_mutex);

            // Update internal state for draw(). m_result needs to be set by manual when csv dataloader is enabled rather than python.
            m_result.cls = (int)(observed_cls + 0.5);
            m_result.confidence = observed_conf;
            m_timestamp = data_time;

            // apply state filtering
            m_state = simpleStateFiltering((int)(observed_cls + 0.5));

            lr_cls = m_state;  // filtered cls
            lr_confidence = observed_conf;
            return true;
        }


    protected:
        int simpleStateFiltering(int observation)
        {
            if (observation == LEFT_SIDE_OF_ROAD) m_state_score -= m_param_update_dec;
            if (observation == RIGHT_SIDE_OF_ROAD) m_state_score += m_param_update_inc;
            if (observation == UNKNOWN_SIDE_OF_ROAD)
            {
                double middle_score = (m_param_state_lower_threshold + m_param_state_upper_threshold) / 2;
                if (fabs(m_state_score - middle_score) <= m_param_unknown_delta) m_state_score = middle_score;
                else if (m_state_score > middle_score) m_state_score -= m_param_unknown_delta;
                else if (m_state_score < middle_score) m_state_score += m_param_unknown_delta;
            }
            if (m_state_score > m_param_state_upper_bound) m_state_score = m_param_state_upper_bound;
            if (m_state_score < m_param_state_lower_bound) m_state_score = m_param_state_lower_bound;

            if (m_state_score <= m_param_state_lower_threshold) return LEFT_SIDE_OF_ROAD;
            else if (m_state_score >= m_param_state_upper_threshold) return RIGHT_SIDE_OF_ROAD;
            else return UNKNOWN_SIDE_OF_ROAD;
        }

        double m_state_score = 0.5;
        int m_state = UNKNOWN_SIDE_OF_ROAD;

        SharedInterface* m_shared = nullptr;
        mutable cv::Mutex m_mutex;
    };

} // End of 'dg'

#endif // End of '__LRPOSE_RECOGNIZER_LOCALIZER__'
