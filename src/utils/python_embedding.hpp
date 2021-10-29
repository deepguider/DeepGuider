#ifndef __DG_PYTHON_EMBEDDING__
#define __DG_PYTHON_EMBEDDING__

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_SSIZE_T_CLEAN
#include "dg_core.hpp"
#include <Python.h>
#include "numpy/arrayobject.h"

namespace dg
{

extern PyThreadState* global_python_thread_state;

/**
* Initialize Python environment
* @param name name of python executable (select python interpreter version to run)
* @param import_path system path of the installed python packages
* @param supports_thread_run enable python thread (make it possible to call python code in c++ thread)
* @return true if successful (false if failed)
*/
bool init_python_environment(const char* name = "python3", const char* import_path = nullptr, bool support_thread_run = false);

bool close_python_environment();


class PythonModuleWrapper
{
public:
    /**
    * Initialize the module
    * @return true if successful (false if failed)
    */
    bool initialize(const char* module_path, const char* module_name, const char* class_name, const char* func_name_init = "initialize", const char* func_name_apply = "apply")
    {
        cv::AutoLock lock(m_mutex);
        dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

        PyGILState_STATE state;
        if (isThreadingEnabled()) state = PyGILState_Ensure();

        bool ret = _initialize(module_path, module_name, class_name, func_name_init, func_name_apply);

        if (isThreadingEnabled()) PyGILState_Release(state);

        dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        m_processing_time = t2 - t1;

        return ret;
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
        m_timestamp = ts;

        return ret;
    }

    /**
    * Return timestamp [sec]
    */
    dg::Timestamp timestamp() const
    {
        cv::AutoLock lock(m_mutex);
        return m_timestamp;
    }

    /**
    * Return processing time of the last operation [sec]
    */
    double procTime() const
    {
        cv::AutoLock lock(m_mutex);
        return m_processing_time;
    }

    /**
    * Reset variables and clear the memory
    */
    void clear()
    {
        cv::AutoLock lock(m_mutex);
        PyGILState_STATE state;
        if (isThreadingEnabled()) state = PyGILState_Ensure();

        _clear();

        if (isThreadingEnabled()) PyGILState_Release(state);

        m_timestamp = -1;
        m_processing_time = -1;
    }

    bool isThreadingEnabled() { return (global_python_thread_state != nullptr); }

protected:
    virtual bool _apply(cv::Mat image, dg::Timestamp ts) { return false; }
    bool _initialize(const char* module_name, const char* module_path, const char* class_name, const char* func_name_init = "initialize", const char* func_name_apply = "apply");
    void _clear();

    mutable cv::Mutex m_mutex;
    Timestamp m_timestamp = -1;
    double m_processing_time = -1;

    PyObject* m_pInstance = nullptr;
    PyObject* m_pFuncApply = nullptr;
    PyObject* m_pFuncInitialize = nullptr;
};

} // End of 'dg'

#endif // End of '__DG_PYTHON_EMBEDDING__'
