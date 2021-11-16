#ifndef __LOGO_RECOGNIZER__
#define __LOGO_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#include "utils/utility.hpp"
#include <fstream>
#include <chrono>

using namespace std;

namespace dg
{
    struct LogoResult
    {
        int xmin, ymin, xmax, ymax;
        std::string label;
        double confidence;
    };

    /**
    * @brief C++ Wrapper of Python module - LogoRecognizer
    */
    class LogoRecognizer : public PythonModuleWrapper
    {
    public:
        /**
        * Run once the module for a given input
        * @return true if successful (false if failed)
        */
        bool _apply(cv::Mat image, dg::Timestamp t)
        {
            // Set function arguments
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(2);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "LogoRecognizer::apply() - Cannot convert argument1\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Timestamp
            pValue = PyFloat_FromDouble(t);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Call the method
            PyObject* pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyTuple_Size(pRet);
                if (n_ret != 2)
                {
                    fprintf(stderr, "LogoRecognizer::apply() - Wrong number of returns\n");
                    return false;
                }

                cv::AutoLock lock(m_mutex);
                // list of list
                m_result.clear();
                PyObject* pList0 = PyTuple_GetItem(pRet, 0);
                if (pList0 != NULL)
                {
                    Py_ssize_t cnt = PyList_Size(pList0);
                    for (int i = 0; i < cnt; i++)
                    {
                        PyObject* pList = PyList_GetItem(pList0, i);
                        if (pList)
                        {
                            LogoResult logo;
                            int idx = 0;
                            pValue = PyList_GetItem(pList, idx++);
                            logo.xmin = PyLong_AsLong(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            logo.ymin = PyLong_AsLong(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            logo.xmax = PyLong_AsLong(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            logo.ymax = PyLong_AsLong(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            logo.label = PyUnicode_AsUTF8(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            logo.confidence = PyFloat_AsDouble(pValue);
                            m_result.push_back(logo);
                        }
                    }
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "LogoRecognizer::apply() - Call failed\n");
                return false;
            }

            // Clean up
            if (pRet) Py_DECREF(pRet);
            if (pArgs) Py_DECREF(pArgs);

            return true;
        }

        std::vector<LogoResult> get() const
        {
            cv::AutoLock lock(m_mutex);
            return m_result;
        }

        std::vector<LogoResult> get(Timestamp& ts) const
        {
            cv::AutoLock lock(m_mutex);
            ts = m_timestamp;
            return m_result;
        }

        void set(const std::vector<LogoResult>& logos, Timestamp ts, double proc_time = -1)
        {
            cv::AutoLock lock(m_mutex);
            m_result = logos;
            m_timestamp = ts;
            m_processing_time = proc_time;
        }

        void draw(cv::Mat& image, double drawing_scale = 1.0, cv::Scalar color = cv::Scalar(0, 255, 0)) const
        {
            cv::AutoLock lock(m_mutex);
            for (size_t i = 0; i < m_result.size(); i++)
            {
                cv::Rect rc(m_result[i].xmin, m_result[i].ymin, m_result[i].xmax - m_result[i].xmin + 1, m_result[i].ymax - m_result[i].ymin + 1);
                cv::rectangle(image, rc, color, (int)(2 * drawing_scale));
                cv::Point2d pt(m_result[i].xmin + 3 * drawing_scale, m_result[i].ymin - 5 * drawing_scale);
                std::string msg = cv::format("%s %.2lf", m_result[i].label.c_str(), m_result[i].confidence);
                cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5 * drawing_scale, cv::Scalar(0, 255, 255), (int)(6 * drawing_scale));
                cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5 * drawing_scale, cv::Scalar(255, 0, 0), (int)(2 * drawing_scale));
            }
        }

        void print() const
        {
            cv::AutoLock lock(m_mutex);
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), m_processing_time, m_timestamp);
            for (int k = 0; k < m_result.size(); k++)
            {
                printf("\t%s, %.2lf, x1=%d, y1=%d, x2=%d, y2=%d\n", m_result[k].label.c_str(), m_result[k].confidence, m_result[k].xmin, m_result[k].ymin, m_result[k].xmax, m_result[k].ymax);
            }
        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            cv::AutoLock lock(m_mutex);
            for (int k = 0; k < m_result.size(); k++)
            {
                std::string log = cv::format("%.3lf,%d,%s,%s,%.2lf,%d,%d,%d,%d,%.3lf", m_timestamp, cam_fnumber, name(), m_result[k].label.c_str(), m_result[k].confidence, m_result[k].xmin, m_result[k].ymin, m_result[k].xmax, m_result[k].ymax, m_processing_time);
                stream << log << std::endl;
            }
        }

        void read(const std::vector<std::string>& stream)
        {
            cv::AutoLock lock(m_mutex);
            m_result.clear();
            for (int k = 0; k < (int)stream.size(); k++)
            {
                std::vector<std::string> elems = splitStr(stream[k].c_str(), (int)stream[k].length(), ',');
                if (elems.size() != 10)
                {
                    printf("[logo] Invalid log data %s\n", stream[k].c_str());
                    return;
                }
                std::string module_name = elems[2];
                if (module_name == name())
                {
                    LogoResult r;
                    r.label = elems[3];
                    r.confidence = atof(elems[4].c_str());
                    r.xmin = atoi(elems[5].c_str());
                    r.ymin = atoi(elems[6].c_str());
                    r.xmax = atoi(elems[7].c_str());
                    r.ymax = atoi(elems[8].c_str());
                    m_result.push_back(r);

                    m_timestamp = atof(elems[0].c_str());
                    m_processing_time = atof(elems[9].c_str());
                }
            }
        }

        static const char* name()
        {
            return "logo";
        }


    protected:
        std::vector<LogoResult> m_result;
    };

} // End of 'dg'

#endif // End of '__LOGO_RECOGNIZER__'
