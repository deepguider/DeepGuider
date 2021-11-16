#ifndef __INTERSECTION_CLS__
#define __INTERSECTION_CLS__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#include "utils/utility.hpp"
#include <fstream>
#include <chrono>

using namespace std;

namespace dg
{
    struct IntersectionResult
    {
        int cls;                  // 0: non-intersection, 1: intersection
        double confidence;  // 0 ~ 1
    };

    /**
    * @brief C++ Wrapper of Python module - IntersectionClassifier
    */
    class IntersectionClassifier: public PythonModuleWrapper
    {
    public:
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
                fprintf(stderr, "IntersectionClassifier::apply() - Cannot convert argument1\n");
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
                    fprintf(stderr, "IntersectionClassifier::apply() - Wrong number of returns\n");
                    return false;
                }

                cv::AutoLock lock(m_mutex);
                // intersection class & confidence
                pValue = PyTuple_GetItem(pRet, 0);
                m_result.cls = PyLong_AsLong(pValue);
                pValue = PyTuple_GetItem(pRet, 1);
                m_result.confidence = PyFloat_AsDouble(pValue);
            }
            else {
                PyErr_Print();
                fprintf(stderr, "IntersectionClassifier::apply() - Call failed\n");
                return false;
            }

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);            

            return true;
        }

        IntersectionResult get() const
        {
            cv::AutoLock lock(m_mutex);
            return m_result;
        }

        IntersectionResult get(Timestamp& ts) const
        {
            cv::AutoLock lock(m_mutex);
            ts = m_timestamp;
            return m_result;
        }

        void set(const IntersectionResult& intersect, Timestamp ts, double proc_time = -1)
        {
            cv::AutoLock lock(m_mutex);
            m_result = intersect;
            m_timestamp = ts;
            m_processing_time = proc_time;
        }

        void draw(cv::Mat& image, double drawing_scale = 1.0, cv::Scalar color = cv::Scalar(0, 255, 0)) const
        {
            cv::AutoLock lock(m_mutex);
            cv::Point2d pt(image.cols / 2 - 180 * drawing_scale, 100 * drawing_scale);
            std::string msg = cv::format("Intersection: %d (%.2lf)", m_result.cls, m_result.confidence);
            cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 2.2 * drawing_scale, color, (int)(6 * drawing_scale));
            cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 2.2 * drawing_scale, cv::Scalar(0, 0, 0), (int)(2 * drawing_scale));
            if(m_result.cls > 0)
            {
                cv::Rect image_rc(0, 0, image.cols, image.rows);
                cv::rectangle(image, image_rc, color, (int)(20 * drawing_scale));
            }
        }

        void print() const
        {
            cv::AutoLock lock(m_mutex);
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), m_processing_time, m_timestamp);
            printf("\tintersection: %d (%.2lf)\n", m_result.cls, m_result.confidence);
        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            cv::AutoLock lock(m_mutex);
            std::string log = cv::format("%.3lf,%d,%s,%d,%.2lf,%.3lf", m_timestamp, cam_fnumber, name(), m_result.cls, m_result.confidence, m_processing_time);
            stream << log << std::endl;
        }

        void read(const std::vector<std::string>& stream)
        {
            cv::AutoLock lock(m_mutex);
            for (int k = 0; k < (int)stream.size(); k++)
            {
                std::vector<std::string> elems = splitStr(stream[k].c_str(), (int)stream[k].length(), ',');
                if (elems.size() != 6)
                {
                    printf("[intersection] Invalid log data %s\n", stream[k].c_str());
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
            return "intersection";
        }


    protected:
        IntersectionResult m_result;
    };

} // End of 'dg'

#endif // End of '__INTERSECTION_CLS__'
