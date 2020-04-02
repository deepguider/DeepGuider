#ifndef __DG_PYTHON_EMBEDDING__
#define __DG_PYTHON_EMBEDDING__

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_SSIZE_T_CLEAN
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
    bool isThreadingEnabled() { return (global_python_thread_state != nullptr); }

protected:
    bool _initialize(const char* module_name, const char* module_path, const char* class_name, const char* func_name_init = "initialize", const char* func_name_apply = "apply");
    void _clear();

    PyObject* m_pInstance = nullptr;
    PyObject* m_pFuncApply = nullptr;
    PyObject* m_pFuncInitialize = nullptr;
};

} // End of 'dg'

#endif // End of '__DG_PYTHON_EMBEDDING__'
