#ifndef __VPS__
#define __VPS__

#include "dg_core.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy/arrayobject.h"

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
    class VPS
    {
    public:
        /**
         * The default constructor
         */
        VPS() { }

        /**
         * The default destructor
         */
        ~VPS()
        {
            clear();
        }

        /**
         * Reset variables and clear the memory
         */
        void clear()
        {
            if (m_pFuncApply != nullptr)
            {
                Py_DECREF(m_pFuncApply);
                m_pFuncApply = nullptr;
            }
            if (m_pInstance != nullptr)
            {
                Py_DECREF(m_pInstance);
                m_pInstance = nullptr;
            }
        }

        /**
         * Initialize the module
         * @return true if successful (false if failed)
         */
        bool initialize()
        {
            std::string module_name = "vps";                    	// python module name
            std::string class_name = "vps";                 		// python class name
            std::string func_name_init = "initialize";				// python method name for initialization
            std::string func_name_apply = "apply";					// python method name for apply

            // Add module path to system path
            PyRun_SimpleString("import sys\nsys.path.append(\"./../src/vps\")");

            // Import the Python module
            PyObject* pName = PyUnicode_FromString(module_name.c_str());
            PyObject* pModule = PyImport_Import(pName);
            Py_DECREF(pName);
            if (pModule == nullptr) {
                PyErr_Print();
                fprintf(stderr, "Fails to import the module \"%s\"\n", module_name.c_str());
                return false;
            }

            // Get the class reference
            PyObject* pClass = PyObject_GetAttrString(pModule, class_name.c_str());
            Py_DECREF(pModule);
            if (pClass == nullptr) {
                PyErr_Print();
                fprintf(stderr, "Cannot find class \"%s\"\n", class_name.c_str());
                return false;
            }

            // Get the method references of the class
            m_pFuncApply = PyObject_GetAttrString(pClass, func_name_apply.c_str());
            if (m_pFuncApply == nullptr) {
                Py_DECREF(pClass);
                PyErr_Print();
                fprintf(stderr, "Cannot find function \"%s\"\n", func_name_apply.c_str());
                return false;
            }
            PyObject* pFuncInitialize = PyObject_GetAttrString(pClass, func_name_init.c_str());
            if (pFuncInitialize == nullptr) {
                PyErr_Print();
                fprintf(stderr, "Cannot find function \"%s\"\n", func_name_init.c_str());
                return false;
            }

            // Create the class instance
            m_pInstance = PyClassMethod_New(pClass);
            Py_DECREF(pClass);
            if (m_pInstance == nullptr) {
                PyErr_Print();
                fprintf(stderr, "Cannot create class instance \"%s\"\n", class_name.c_str());
                return false;
            }

            // Call the initialization method of the class instance
            PyObject* pArgs = PyTuple_New(1);
            PyTuple_SetItem(pArgs, 0, m_pInstance);
            PyObject* pRet = PyObject_CallObject(pFuncInitialize, pArgs);
            if (pRet != NULL)
            {
                if (!PyObject_IsTrue(pRet))
                {
                    fprintf(stderr, "Unsuccessful instance initialization\n");
                    return false;
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "VPS::initialize() - Call failed\n");
                return false;
            }

            return true;
        }

        /**
         * Run once the module for a given input
         * @param N number of matched images to be returned (top-N)
         * @return true if successful (false if failed)
         */
        bool apply(cv::Mat image, int N, double gps_lat, double gps_lon, double gps_accuracy, dg::Timestamp t)
        {
            // Set function arguments

            // Self
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(6);
            PyTuple_SetItem(pArgs, arg_idx++, m_pInstance);

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

            // Call the method
            PyObject* pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyTuple_Size(pRet);
                if (n_ret < 1)
                {
                    fprintf(stderr, "VPS::apply() - Wrong number of returns\n");
                    return false;
                }

                // [[id1,...idN],[conf1,...,confN]] : matched top-N streetview ID's and Confidences
                std::vector<dg::ID> ids;
                std::vector<double> confs;
                PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
                if (pValue0 != NULL)
                {
                    // ID list
                    PyObject* pList0 = PyList_GetItem(pValue0, 0);
                    Py_ssize_t cnt0 = PyList_Size(pList0);
                    for (int i = 0; i < cnt0; i++)
                    {
                        pValue = PyList_GetItem(pList0, i);
                        ids.push_back(PyLong_AsLong(pValue));
                    }
                    Py_DECREF(pList0);

                    // Confidence list
                    PyObject* pList1 = PyList_GetItem(pValue0, 1);
                    Py_ssize_t cnt1 = PyList_Size(pList1);
                    for (int i = 0; i < cnt1; i++)
                    {
                        pValue = PyList_GetItem(pList1, i);
                        confs.push_back(PyFloat_AsDouble(pValue));
                    }
                    Py_DECREF(pList1);
                }
                Py_DECREF(pValue0);

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

        PyObject* m_pFuncApply = nullptr;
        PyObject* m_pInstance = nullptr;
    };

} // End of 'dg'

#endif // End of '__VPS__'
