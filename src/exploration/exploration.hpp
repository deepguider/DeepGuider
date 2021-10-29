#ifndef __EXPLORATION__
#define __EXPLORATION__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#include "guidance/guidance.hpp"

using namespace std;

namespace dg
{

    struct ExplorationGuidance
    {
        double theta1, d, theta2;
    };

    /**
     * @brief C++ Wrapper of python module - Active Navigation Module
     */

class ActiveNavigation : public PythonModuleWrapper
    {
    public:
        /**
        * Initialize the module
        * @return true if successful (false if failed)
        */
        bool initialize(std::string module_path = "./../src/exploration", const char* module_name = "active_navigation", const char* class_name = "ActiveNavigationModule", const char* func_name_init = "initialize", const char* func_name_apply = "getExplorationGuidance")
        {
            return PythonModuleWrapper::initialize(module_path.c_str(), module_name, class_name, func_name_init, func_name_apply);
        }


        /**
        * Run once the module for a given input (support thread run)
        * @return true if successful (false if failed)
        */
        bool apply(cv::Mat image, GuidanceManager::Guidance guidance, dg::Timestamp ts)
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            bool ret = _apply(image, guidance, ts);

            if (isThreadingEnabled()) PyGILState_Release(state);

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            m_processing_time = t2 - t1;
            m_timestamp = ts;

            return ret;
        }

        /**
        * Run once the module for a given input
        * @return true if successful (false if failed)
        */
        bool _apply(cv::Mat image, GuidanceManager::Guidance guidance, dg::Timestamp t)
        {
            // Set function arguments
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(5);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "ActiveNavigation::apply() - Cannot convert argument1\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // guidance
            if (guidance.actions.back().cmd==GuidanceManager::Motion::GO_FORWARD || guidance.actions.back().cmd==GuidanceManager::Motion::CROSS_FORWARD || 
                                 guidance.actions.back().cmd==GuidanceManager::Motion::ENTER_FORWARD || guidance.actions.back().cmd==GuidanceManager::Motion::EXIT_FORWARD)    
                pValue = PyLong_FromLong(0);
            else if (guidance.actions.back().cmd==GuidanceManager::Motion::TURN_LEFT || guidance.actions.back().cmd==GuidanceManager::Motion::CROSS_LEFT || 
                                 guidance.actions.back().cmd==GuidanceManager::Motion::ENTER_LEFT || guidance.actions.back().cmd==GuidanceManager::Motion::EXIT_LEFT)
                pValue = PyLong_FromLong(1);
            else if (guidance.actions.back().cmd==GuidanceManager::Motion::TURN_RIGHT || guidance.actions.back().cmd==GuidanceManager::Motion::CROSS_RIGHT ||
                                 guidance.actions.back().cmd==GuidanceManager::Motion::ENTER_RIGHT || guidance.actions.back().cmd==GuidanceManager::Motion::EXIT_RIGHT)
                pValue = PyLong_FromLong(2);
            else
                pValue = PyLong_FromLong(3);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // flush
            if (guidance.guide_status == GuidanceManager::GuideStatus::GUIDE_ARRIVED )
                pValue = Py_True;
            else
                pValue = Py_False;
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // exp_active
            if (guidance.guide_status == GuidanceManager::GuideStatus::GUIDE_LOST || guidance.guide_status == GuidanceManager::GuideStatus::GUIDE_EXPLORATION ||
                                     guidance.guide_status == GuidanceManager::GuideStatus::GUIDE_RECOVERY || guidance.guide_status == GuidanceManager::GuideStatus::GUIDE_OPTIMAL_VIEW)
                pValue = Py_True;
            else
                pValue = Py_False;
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // ove
            if (guidance.guide_status == GuidanceManager::GuideStatus::GUIDE_OPTIMAL_VIEW)
                pValue = Py_True;
            else
                pValue = Py_False;
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Timestamp
            // pValue = PyFloat_FromDouble(t);
            // PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Call the method
            PyObject* pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyTuple_Size(pRet);
                if (n_ret != 2)
                {
                    fprintf(stderr, "ActiveNavigation::apply() - Wrong number of returns\n");
                    return false;
                }

                cv::AutoLock lock(m_mutex);
                // list of list
                m_actions.clear();
                
                // PyObject* pList1 = PyTuple_GetItem(pRet, 1);
                // pValue = PyList_GetItem(pList1, 0);
                pValue = PyTuple_GetItem(pRet, 1);
                string stat = PyUnicode_AsUTF8(pValue);
                if (stat == "Normal")
                    m_status = GuidanceManager::GuideStatus::GUIDE_NORMAL;
                else if (stat == "Exploration")
                    m_status = GuidanceManager::GuideStatus::GUIDE_EXPLORATION;
                else if (stat == "Recovery")
                    m_status = GuidanceManager::GuideStatus::GUIDE_RECOVERY;
                else
                    m_status = GuidanceManager::GuideStatus::GUIDE_OPTIMAL_VIEW;

                PyObject* pList0 = PyTuple_GetItem(pRet, 0);
                if (pList0 != NULL)
                {
                    Py_ssize_t cnt = PyList_Size(pList0);
                    for (int i = 0; i < cnt; i++)
                    {
                        PyObject* pList = PyList_GetItem(pList0, i);
                        if(pList)
                        {
                            ExplorationGuidance action;
                            int idx = 0;
                            pValue = PyList_GetItem(pList, idx++);
                            action.theta1 = PyFloat_AsDouble(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            action.d = PyFloat_AsDouble(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            action.theta2 = PyFloat_AsDouble(pValue);
                            m_actions.push_back(action);
                        }
                    }
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "ActiveNavigation::apply() - Call failed\n");
                return false;
            }

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);            

            return true;
        }

        void get(std::vector<ExplorationGuidance>& actions, GuidanceManager::GuideStatus& status)
        {
            cv::AutoLock lock(m_mutex);
            actions = m_actions;
            status = m_status;
        }

        void get(std::vector<ExplorationGuidance>& actions,GuidanceManager::GuideStatus& status, Timestamp& t)
        {
            cv::AutoLock lock(m_mutex);
            actions = m_actions;
            status = m_status;
            t = m_timestamp;
        }

      	void draw(cv::Mat& image, cv::Scalar color = cv::Scalar(0, 255, 0), double drawing_scale = 2) const
        {
            cv::AutoLock lock(m_mutex);
            cv::Point2d pt1(image.cols / 2 - 190 * drawing_scale, 50 * drawing_scale);
            cv::Point2d pt2(image.cols / 2 - 190 * drawing_scale, 100 * drawing_scale);            
            for (size_t k = 0; k < m_actions.size(); k++)
			{
	            std::string msg1 = cv::format("exploration[Th1,D,Th2]");
            	cv::putText(image, msg1, pt1, cv::FONT_HERSHEY_PLAIN, 2.2 * drawing_scale, cv::Scalar(0, 255, 0), (int)(6 * drawing_scale));
            	cv::putText(image, msg1, pt1, cv::FONT_HERSHEY_PLAIN, 2.2 * drawing_scale, cv::Scalar(0, 0, 0), (int)(2 * drawing_scale));

	            std::string msg2 = cv::format("[%3.2f, %3.2f, %3.2f]", m_actions[k].theta1, m_actions[k].d, m_actions[k].theta2);
            	cv::putText(image, msg2, pt2, cv::FONT_HERSHEY_PLAIN, 2.2 * drawing_scale, cv::Scalar(0, 255, 0), (int)(6 * drawing_scale));
            	cv::putText(image, msg2, pt2, cv::FONT_HERSHEY_PLAIN, 2.2 * drawing_scale, cv::Scalar(0, 0, 0), (int)(2 * drawing_scale));                
                // printf("\t %s\n", msg);
			}
        }

        void print() const
        {
            cv::AutoLock lock(m_mutex);
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), m_processing_time, m_timestamp);
			printf("\t action [%lf, %lf, %lf]\n", m_actions[0].theta1, m_actions[0].d, m_actions[0].theta2);
        }

        static const char* name()
        {
            return "exploration";
        }
		
	  
    protected:
        std::vector<ExplorationGuidance> m_actions;
        GuidanceManager::GuideStatus m_status;
    };

} // End of 'dg'

#endif // End of '__ACTIVE_NAVIGATION__'
