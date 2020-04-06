#ifndef __EXPLORATION__
#define __EXPLORATION__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"

using namespace std;

namespace dg
{
    struct ActiveNavigationGuidance
    {
        std::vector<std::string> actions;
    };

    struct OptimalViewpointGuidance
    {
        double theta1, d, theta2
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
        bool initialize(const char* module_name = "active_navigation", const char* module_path = "./../src/exploration", const char* class_name = "ActiveNavigationModule", const char* func_name_init = "initialize", const char* func_name_apply = "apply")
        {
            PyGILState_STATE state;
            bool ret;

            if (isThreadingEnabled()) state = PyGILState_Ensure();

            ret = _initialize(module_name, module_path, class_name, func_name_init, func_name_apply);

            if (isThreadingEnabled()) PyGILState_Release(state);

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
        }

        /**
        * Run once the module for a given input (support thread run)
        * @return true if successful (false if failed)
        */
        bool apply(cv::Mat image, dg::Timestamp t)
        {
            PyGILState_STATE state;
            bool ret;

            if (isThreadingEnabled()) state = PyGILState_Ensure();

            /* Call Python/C API functions here */
            ret = _apply(image, t);

            if (isThreadingEnabled()) PyGILState_Release(state);

            return ret;
        }

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
    };

} // End of 'dg'

#endif // End of '__ACTIVE_NAVIGATION__'
