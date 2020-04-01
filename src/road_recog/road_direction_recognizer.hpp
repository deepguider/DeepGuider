#ifndef __ROAD_DIRECTION_RECOGNIZER__
#define __ROAD_DIRECTION_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"

using namespace std;

namespace dg
{

    /**
    * @brief C++ Wrapper of Python module - Road direction recognizer
    */
    class RoadDirectionRecognizer : public PythonModuleWrapper
    {
    public:
        /**
        * Initialize the module
        * @return true if successful (false if failed)
        */
        bool initialize()
        {
            PyGILState_STATE state;
            bool ret;

            if (isThreadingEnabled()) state = PyGILState_Ensure();

            ret = _initialize("road_direction_recognizer", "./../src/road_recog", "RoadDirectionRecognizer");

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
        bool _apply(cv::Mat image, Timestamp t)
        {
            // Set function arguments
            int arg_idx = 0;
            PyObject *pArgs = PyTuple_New(2);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "RoadDirectionRecognizer::apply() - Cannot convert argument1\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Timestamp
            pValue = PyFloat_FromDouble(t);
            if (!pValue) {
                fprintf(stderr, "RoadDirectionRecognizer::apply() - Cannot convert argument2\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Call the method
            PyObject *pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyTuple_Size(pRet);
                if (n_ret != 2)
                {
                    fprintf(stderr, "RoadDirectionRecognizer::apply() - Wrong number of returns\n");
                    return false;
                }
                PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
                if (pValue0 != NULL) m_angle = PyLong_AsLong(pValue0);
                PyObject* pValue1 = PyTuple_GetItem(pRet, 1);
                if (pValue1 != NULL) m_prob = PyLong_AsLong(pValue1);
                Py_DECREF(pValue0);
                Py_DECREF(pValue1);
            }
            else {
                PyErr_Print();
                fprintf(stderr, "RoadDirectionRecognizer::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = t;

            return true;
        }

        void get(double& _angle, double& _prob)
        {
            _angle = m_angle;
            _prob = m_prob;
        }

        void get(double& _angle, double& _prob, Timestamp& _t)
        {
            _angle = m_angle;
            _prob = m_prob;
            _t = m_timestamp;
        }

    protected:
        double m_angle = -1;
        double m_prob = -1;
        Timestamp m_timestamp = -1;
    };

} // End of 'dg'

#endif // End of '__ROAD_DIRECTION_RECOGNIZER__'
