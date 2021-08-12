#ifndef __LRPOSE_RECOGNIZER__
#define __LRPOSE_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#include "utils/utility.hpp"
#include <fstream>
#include <chrono>

using namespace std;

namespace dg
{
    struct LRPoseResult
    {
        int cls;                  // 0: Left ||road||, 1: unknown, 2: ||road|| Right
        double confidence;  // 0 ~ 1
    };

	const string PoseClassName[3] = {"Left", "Uncertain", "Right"};

    /**
    * @brief C++ Wrapper of Python module - LRPoseRecognizer
    */
    class LRPoseRecognizer: public PythonModuleWrapper
    {
    public:
        /**
        * Initialize the module
        * @return true if successful (false if failed)
        */
        bool initialize(const char* module_name = "lrpose_recognizer", const char* module_path = "./../src/lrpose_recog", const char* class_name = "lrpose_recognizer", const char* func_name_init = "initialize", const char* func_name_apply = "apply")
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            bool ret = _initialize(module_name, module_path, class_name, func_name_init, func_name_apply);

            if (isThreadingEnabled()) PyGILState_Release(state);

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            m_processing_time = t2 - t1;

            return ret;
        }

        /**
        * Reset variables and clear the memory
        */
        void clear()
        {
            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            _clear();

            if (isThreadingEnabled()) PyGILState_Release(state);

            m_timestamp = -1;
            m_processing_time = -1;
        }

        /**
        * Run once the module for a given input (support thread run)
        * @return true if successful (false if failed)
        */
        bool apply(cv::Mat image, dg::Timestamp ts)
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            bool ret = _apply(image, ts);

            if (isThreadingEnabled()) PyGILState_Release(state);

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            m_processing_time = t2 - t1;

            return true;
        }

        /**
        * Run once the module for a given input
        * @return true if successful (false if failed)
        */
        bool _apply(cv::Mat image, dg::Timestamp ts)
        {
            // Set function arguments
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(2);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "LRPoseRecognizer::apply() - Cannot convert argument1\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Timestamp
            pValue = PyFloat_FromDouble(ts);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Call the method
            PyObject* pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyTuple_Size(pRet);
                if (n_ret != 2)
                {
                    fprintf(stderr, "LRPoseRecognizer::apply() - Wrong number of returns\n");
                    return false;
                }

                // lrposenition class & confidence
                pValue = PyTuple_GetItem(pRet, 0);
                m_result.cls = PyLong_AsLong(pValue);
                pValue = PyTuple_GetItem(pRet, 1);
                m_result.confidence = PyFloat_AsDouble(pValue);
            }
            else {
                PyErr_Print();
                fprintf(stderr, "LRPoseRecognizer::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = ts;

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);            

            return true;
        }

        void get(LRPoseResult& lrpose) const
        {
            lrpose = m_result;
        }

        void get(LRPoseResult& lrpose, Timestamp& ts) const
        {
            lrpose = m_result;
            ts = m_timestamp;
        }

        void set(const LRPoseResult& lrpose, Timestamp ts, double proc_time)
        {
            m_result = lrpose;
            m_timestamp = ts;
            m_processing_time = proc_time;
        }

        dg::Timestamp timestamp() const
        {
            return m_timestamp;
        }

        double procTime() const
        {
            return m_processing_time;
        }

        void draw(cv::Mat& image, cv::Scalar color = cv::Scalar(0, 255, 0), int width = 2) const
        {
            cv::Point pt(image.cols / 2 - 170, 100);
            std::string msg = cv::format("lrpose : %s(%d), prob(%.2lf)", PoseClassName[m_result.cls].c_str(), m_result.cls, m_result.confidence);
            cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 2.2, cv::Scalar(0, 255, 0), 6);
            cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 2.2, cv::Scalar(0, 0, 0), 2);
        }

        void print() const
        {
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), procTime(), m_timestamp);
            printf("\tlrposenition: %d (%.2lf)\n", m_result.cls, m_result.confidence);

        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            std::string log = cv::format("%.3lf,%d,%s,%d,%.2lf,%.3lf", m_timestamp, cam_fnumber, name(), m_result.cls, m_result.confidence, m_processing_time);
            stream << log << std::endl;
        }

        void read(const std::vector<std::string>& stream)
        {
            for (int k = 0; k < (int)stream.size(); k++)
            {
                std::vector<std::string> elems = splitStr(stream[k].c_str(), (int)stream[k].length(), ',');
                if (elems.size() != 6)
                {
                    printf("[lrposenition] Invalid log data %s\n", stream[k].c_str());
                    return;
                }
                std::string module_name = elems[2];
                if (module_name == name())
                {
                    m_result.cls = atoi(elems[3].c_str());
                    m_result.confidence = atof(elems[4].c_str());
                    m_timestamp = atof(elems[0].c_str());
                    m_processing_time = atof(elems[5].c_str());
                }
            }
        }

        static const char* name()
        {
            return "lrposenition";
        }


    protected:
        LRPoseResult m_result;
        Timestamp m_timestamp = -1;
        double m_processing_time = -1;
    };

} // End of 'dg'

#endif // End of '__LRPOSE_RECOGNIZER__'
