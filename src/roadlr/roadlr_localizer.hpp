#ifndef __ROADLR_LOCALIZER__
#define __ROADLR_LOCALIZER__

#include "dg_core.hpp"
#include "roadlr/roadlr.hpp"

using namespace std;

namespace dg
{
    /**
    * @brief Intersection-based Localizer
    */
    class RoadLRLocalizer: public RoadLRRecognizer
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

        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/roadlr")
        {
            if (!RoadLRRecognizer::initialize(py_module_path.c_str(), "roadlr", "roadlr_recognizer")) return false;

            cv::AutoLock lock(m_localizer_mutex);
            m_shared = shared;
            return (m_shared != nullptr);
        }

        bool initialize_without_python(SharedInterface* shared)
        {
            cv::AutoLock lock(m_localizer_mutex);
            m_shared = shared;
            return (m_shared != nullptr);
        }

        bool apply(const cv::Mat image, const dg::Timestamp image_time, int& lr_cls, double& lr_confidence)
        {
            if (!RoadLRRecognizer::apply(image, image_time)) return false;

            cv::AutoLock lock(m_localizer_mutex);
            // apply state filtering
            RoadLRResult roadlr = get();
            m_state = simpleStateFiltering(roadlr.cls);

			lr_cls = m_state;  // filtered cls
            lr_confidence = roadlr.confidence;
            return true;
        }

        bool applyPreprocessed(double observed_cls, double observed_conf, const dg::Timestamp data_time, int& lr_cls, double& lr_confidence)
        {
            cv::AutoLock lock(m_localizer_mutex);

            // Update internal state for draw(). m_result needs to be set by manual when csv dataloader is enabled rather than python.
            RoadLRResult roadlr;
            roadlr.cls = (int)(observed_cls + 0.5);
            roadlr.confidence = observed_conf;
            set(roadlr, data_time);

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
        cv::Mutex m_localizer_mutex;
    };

} // End of 'dg'

#endif // End of '__ROADLR_LOCALIZER__'
