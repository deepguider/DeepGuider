#ifndef __VPS__
#define __VPS__

#include "dg_core.hpp"
#include "python_embedding.hpp"

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
         * Initialize the module
         * @return true if successful (false if failed)
         */
        bool initialize()
        {
            return _initialize("vps", "./../src/vps", "vps");
        }

        /**
         * Reset variables and clear the memory
         */
        void clear()
        {
            _clear();
        }

        /**
         * Run once the module for a given input
         * @param N number of matched images to be returned (top-N)
         * @return true if successful (false if failed)
         */
        bool apply(cv::Mat image, int N, double gps_lat, double gps_lon, double gps_accuracy, dg::Timestamp t, const char* ipaddr)
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
            pValue = PyFloat_FromDouble(t);
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
                    fprintf(stderr, "VPS::apply() - Wrong number of returns: %ld\n", n_ret);
                    return false;
                }

                // [[id1,...idN],[conf1,...,confN]] : matched top-N streetview ID's and Confidences
                
                // ID list
                std::vector<dg::ID> ids;
                PyObject* pList0 = PyList_GetItem(pRet, 0);
                if(pList0)
                {
                    Py_ssize_t cnt0 = PyList_Size(pList0);
                    for (int i = 0; i < cnt0; i++)
                    {
                        pValue = PyList_GetItem(pList0, i);
                        ids.push_back(PyLong_AsLong(pValue));
                    }
                }
                Py_DECREF(pList0);

                // Confidence list
                std::vector<double> confs;
                PyObject* pList1 = PyList_GetItem(pRet, 1);
                if(pList1)
                {
                    Py_ssize_t cnt1 = PyList_Size(pList1);
                    for (int i = 0; i < cnt1; i++)
                    {
                        pValue = PyList_GetItem(pList1, i);
                        confs.push_back(PyFloat_AsDouble(pValue));
                    }
                }
                Py_DECREF(pList1);

                // save the result
                m_streetviews.clear();
                for (size_t i = 0; i < ids.size(); i++)
                {
                    VPSResult vps;
                    vps.id = ids[i];
                    vps.confidence = confs[i];
                    m_streetviews.push_back(vps);
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "VPS::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = t;

            return true;
        }

        void get(std::vector<VPSResult>& streetviews)
        {
            streetviews = m_streetviews;
        }

        void get(std::vector<VPSResult>& streetviews, Timestamp& t)
        {
            streetviews = m_streetviews;
            t = m_timestamp;
        }

    protected:
        std::vector<VPSResult> m_streetviews;
        Timestamp m_timestamp = -1;
    };

} // End of 'dg'

#endif // End of '__VPS__'
