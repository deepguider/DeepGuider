#ifndef __LOGO_RECOGNIZER__
#define __LOGO_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"

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
        * Initialize the module
        * @return true if successful (false if failed)
        */
        bool initialize(const char* module_name = "logo_recognizer", const char* module_path = "./../src/logo_recog", const char* class_name = "LogoRecognizer", const char* func_name_init = "initialize", const char* func_name_apply = "apply")
        {
            PyGILState_STATE state;
            bool ret;

            if (isThreadingEnabled()) state = PyGILState_Ensure();

            ret = _initialize(module_name, module_path, class_name, func_name_init, func_name_apply);

            if (isThreadingEnabled()) PyGILState_Release(state);

            return ret;
        }

        /**
        * Initialize the module in fast mode (just for test purpose)
        * @return true if successful (false if failed)
        */
        bool initialize_fast(const char* module_name = "logo_recognizer", const char* module_path = "./../src/logo_recog", const char* class_name = "LogoRecognizer", const char* func_name_init = "initialize_fast", const char* func_name_apply = "apply")
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

                // list of list
                m_logos.clear();
                PyObject* pList0 = PyTuple_GetItem(pRet, 0);
                if (pList0 != NULL)
                {
                    Py_ssize_t cnt = PyList_Size(pList0);
                    for (int i = 0; i < cnt; i++)
                    {
                        PyObject* pList = PyList_GetItem(pList0, i);
                        if(pList)
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
                            m_logos.push_back(logo);
                        }
                    }
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "LogoRecognizer::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = t;

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);            

            return true;
        }

        void get(std::vector<LogoResult>& logos)
        {
            logos = m_logos;
        }

        void get(std::vector<LogoResult>& logos, Timestamp& t)
        {
            logos = m_logos;
            t = m_timestamp;
        }

    protected:
        std::vector<LogoResult> m_logos;
        Timestamp m_timestamp = -1;
    };

} // End of 'dg'

#endif // End of '__LOGO_RECOGNIZER__'
