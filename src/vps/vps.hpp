#ifndef __VPS__
#define __VPS__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#include "utils/utility.hpp"
#include <fstream>
#include <chrono>


using namespace std;

namespace dg
{
    struct VPSResult
    {
        dg::ID id;
        double confidence;
    };

    /**
    * @brief C++ Wrapper of Python module - VPS
    */
    class VPS : public PythonModuleWrapper
    {
    public:

        /**
        * Run once the module for a given input (support threading run)
        * @param N number of matched images to be returned (top-N)
        * @return true if successful (false if failed)
        */
        bool apply(cv::Mat image, int N, double gps_lat, double gps_lon, double gps_accuracy, dg::Timestamp ts, const char* ipaddr)
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            bool ret = _apply(image, N, gps_lat, gps_lon, gps_accuracy, ts, ipaddr);

            if (isThreadingEnabled()) PyGILState_Release(state);

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            m_processing_time = t2 - t1;
            m_timestamp = ts;

            return ret;
        }

        /**
        * Run once the module for a given input
        * @param N number of matched images to be returned (top-N)
        * @return true if successful (false if failed)
        */
        bool _apply(cv::Mat image, int N, double gps_lat, double gps_lon, double gps_accuracy, dg::Timestamp ts, const char* ipaddr)
        {
            // Set function arguments
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(7);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "VPS::apply() - Cannot convert argument1\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // N
            pValue = PyFloat_FromDouble(N);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // GPS lat, lon, accuracy
            pValue = PyFloat_FromDouble(gps_lat);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);
            pValue = PyFloat_FromDouble(gps_lon);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);
            pValue = PyFloat_FromDouble(gps_accuracy);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Timestamp
            pValue = PyFloat_FromDouble(ts);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Image server's ip address
            pValue = PyUnicode_FromString(ipaddr);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Call the method
            PyObject* pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyList_Size(pRet);
                if (n_ret < 2)
                {
                    fprintf(stderr, "VPS::apply() - Wrong number of returns: %d\n", (int)n_ret);
                    return false;
                }

                // [[id1,...idN],[conf1,...,confN]] : matched top-N streetview ID's and Confidences

                // ID list
                std::vector<dg::ID> ids;
                PyObject* pList0 = PyList_GetItem(pRet, 0);
                if (pList0)
                {
                    Py_ssize_t cnt0 = PyList_Size(pList0);
                    for (int i = 0; i < cnt0; i++)
                    {
                        PyObject* pValue = PyList_GetItem(pList0, i);
                        if(pValue) ids.push_back(PyLong_AsLong(pValue));
                    }
                }
                
                // Confidence list
                std::vector<double> confs;
                PyObject* pList1 = PyList_GetItem(pRet, 1);
                if (pList1)
                {
                    Py_ssize_t cnt1 = PyList_Size(pList1);
                    for (int i = 0; i < cnt1; i++)
                    {
                        PyObject* pValue = PyList_GetItem(pList1, i);
                        if(pValue) confs.push_back(PyFloat_AsDouble(pValue));
                    }
                }

                // Save the result
                cv::AutoLock lock(m_mutex);
                m_result.clear();
                for (size_t i = 0; i < ids.size(); i++)
                {
                    VPSResult vps;
                    vps.id = ids[i];
                    vps.confidence = confs[i];
                    m_result.push_back(vps);
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "VPS::apply() - Call failed\n");
                return false;
            }

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);

            return true;
        }

        std::vector<VPSResult> get() const
        {
            cv::AutoLock lock(m_mutex);
            return m_result;
        }

        std::vector<VPSResult> get(Timestamp& ts) const
        {
            cv::AutoLock lock(m_mutex);
            ts = m_timestamp;
            return m_result;
        }

        void set(const std::vector<VPSResult>& result, Timestamp ts, double proc_time = -1)
        {
            cv::AutoLock lock(m_mutex);
            m_result = result;
            m_timestamp = ts;
            m_processing_time = proc_time;
        }

        void draw(cv::Mat& image, cv::Scalar color = cv::Scalar(0, 255, 0), double drawing_scale = 2) const
        {
            cv::AutoLock lock(m_mutex);
            if (m_result.empty()) return;

            cv::Point2d msg_offset = cv::Point2d(10 * drawing_scale, 30 * drawing_scale);
            double font_scale = 0.8 * drawing_scale;
            std::string str_confidence = cv::format("Confidence: %.2lf", m_result[0].confidence);
            cv::putText(image, str_confidence.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), (int)(5 * drawing_scale));
            cv::putText(image, str_confidence.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), (int)(2 * drawing_scale));
            std::string str_id = cv::format("ID: %zu", m_result[0].id);
            msg_offset.y += (30 * drawing_scale);
            cv::putText(image, str_id.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), (int)(5 * drawing_scale));
            cv::putText(image, str_id.c_str(), msg_offset, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), (int)(2 * drawing_scale));
        }

        void print() const
        {
            cv::AutoLock lock(m_mutex);
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), m_processing_time, m_timestamp);
            for (int k = 0; k < m_result.size(); k++)
            {
                printf("\ttop%d: id=%zu, confidence=%.2lf\n", k, m_result[k].id, m_result[k].confidence);
            }
        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            cv::AutoLock lock(m_mutex);
            for (int k = 0; k < m_result.size(); k++)
            {
                std::string log = cv::format("%.3lf,%d,%s,%d,%zu,%.2lf,%.3lf", m_timestamp, cam_fnumber, name(), k, m_result[k].id, m_result[k].confidence, m_processing_time);
                stream << log << std::endl;
            }
        }

        void read(const std::vector<std::string>& stream)
        {
            cv::AutoLock lock(m_mutex);
            m_result.clear();
            m_result.resize(stream.size());
            for (int k = 0; k < (int)stream.size(); k++)
            {
                std::vector<std::string> elems = splitStr(stream[k].c_str(), (int)stream[k].length(), ',');
                if (elems.size() != 7)
                {
                    printf("[vps] Invalid log data %s\n", stream[k].c_str());
                    return;
                }
                std::string module_name = elems[2];
                if (module_name == name())
                {
                    VPSResult r;
                    int top_k = atoi(elems[3].c_str());
                    r.id = (dg::ID)atof(elems[4].c_str());
                    r.confidence = atof(elems[5].c_str());
                    m_result[top_k] = r;

                    m_timestamp = atof(elems[0].c_str());
                    m_processing_time = atof(elems[6].c_str());
                }
            }
        }

        static const char* name()
        {
            return "vps";
        }


    protected:
        std::vector<VPSResult> m_result;
    };

} // End of 'dg'

#endif // End of '__VPS__'
