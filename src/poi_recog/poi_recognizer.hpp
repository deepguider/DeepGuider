#ifndef __POI_RECOGNIZER__
#define __POI_RECOGNIZER__

#include "dg_core.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy/arrayobject.h"

using namespace std;

namespace dg
{
    struct POIResult
    {
        int xmin, ymin, xmax, ymax;
        std::string label;
        double confidence;
    };

    /**
     * @brief C++ Wrapper of Python module - POIRecognizer
     */
    class POIRecognizer
    {
    public:
        /**
         * The default constructor
         */
        POIRecognizer() { }

        /**
         * The default destructor
         */
        ~POIRecognizer()
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
            std::string module_name = "poi_recognizer";            	// python module name
            std::string class_name = "POIRecognizer";          		// python class name
            std::string func_name_init = "initialize";				// python method name for initialization
            std::string func_name_apply = "apply";					// python method name for apply

            // Add module path to system path
            PyRun_SimpleString("import sys\nsys.path.append(\"./../src/poi_recog\")");

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
                fprintf(stderr, "POIRecognizer::initialize() - Call failed\n");
                return false;
            }

            return true;
        }

        /**
         * Run once the module for a given input
         * @return true if successful (false if failed)
         */
        bool apply(cv::Mat image, dg::Timestamp t)
        {
            // Set function arguments

            // Self
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(3);
            PyTuple_SetItem(pArgs, arg_idx++, m_pInstance);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "POIRecognizer::apply() - Cannot convert argument1\n");
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
                    fprintf(stderr, "POIRecognizer::apply() - Wrong number of returns\n");
                    return false;
                }

                // list of list
                m_pois.clear();
                PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
                if (pValue0 != NULL)
                {
                    Py_ssize_t cnt = PyList_Size(pValue0);
                    for (int i = 0; i < cnt; i++)
                    {
                        PyObject* pList = PyList_GetItem(pValue0, i);

                        POIResult poi;
                        int idx = 0;
                        pValue = PyList_GetItem(pList, idx++);
                        poi.xmin = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.ymin = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.xmax = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.ymax = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.label = PyUnicode_AsUTF8(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.confidence = PyFloat_AsDouble(pValue);

                        m_pois.push_back(poi);
                        Py_DECREF(pList);
                    }
                }
                Py_DECREF(pValue0);
            }
            else {
                PyErr_Print();
                fprintf(stderr, "POIRecognizer::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = t;

            return true;
        }

        void get(std::vector<POIResult>& pois)
        {
            pois = m_pois;
        }

        void get(std::vector<POIResult>& pois, Timestamp& t)
        {
            pois = m_pois;
            t = m_timestamp;
        }

    protected:
        std::vector<POIResult> m_pois;
        Timestamp m_timestamp = -1;

        PyObject* m_pFuncApply = nullptr;
        PyObject* m_pInstance = nullptr;
    };

} // End of 'dg'

#endif // End of '__POI_RECOGNIZER__'
