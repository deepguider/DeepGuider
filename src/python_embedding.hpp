#ifndef __DG_PYTHON_EMBEDDING__
#define __DG_PYTHON_EMBEDDING__

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy/arrayobject.h"

namespace dg
{

bool init_python_environment(const char* name = "python3", const char* import_path = nullptr);


bool close_python_environment();


class PythonModuleWrapper
{
protected:
    bool _initialize(const char* module_name, const char* module_path, const char* class_name, const char* func_name_init = "initialize", const char* func_name_apply = "apply");
    void _clear();

    PyObject* m_pInstance = nullptr;
    PyObject* m_pFuncApply = nullptr;
    PyObject* m_pFuncInitialize = nullptr;
};


bool PythonModuleWrapper::_initialize(const char* module_name, const char* module_path, const char* class_name, const char* func_name_init /*= "initialize"*/, const char* func_name_apply /*= "apply"*/)
{
    // Add module path to system path
    std::string script = std::string("import sys\nsys.path.append(\"") + module_path + "\")";
    PyRun_SimpleString(script.c_str());

    // Import the Python module
    PyObject* pName = PyUnicode_FromString(module_name);
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);
    if (pModule == nullptr) {
        PyErr_Print();
        fprintf(stderr, "Fails to import the module \"%s\"\n", module_name);
        return false;
    }

    // Get the class reference
    PyObject* pClass = PyObject_GetAttrString(pModule, class_name);
    Py_DECREF(pModule);
    if (pClass == nullptr) {
        PyErr_Print();
        fprintf(stderr, "Cannot find class \"%s\"\n", class_name);
        return false;
    }

    // Create the class instance
    m_pInstance = PyObject_CallObject(pClass, nullptr);
    Py_DECREF(pClass);
    if (m_pInstance == nullptr) {
        PyErr_Print();
        fprintf(stderr, "Cannot create class instance \"%s\"\n", class_name);
        return false;
    }

    // Get the reference of class method
    m_pFuncInitialize = PyObject_GetAttrString(m_pInstance, func_name_init);
    if (m_pFuncInitialize == nullptr) {
        PyErr_Print();
        fprintf(stderr, "Cannot find function \"%s\"\n", func_name_init);
        return false;
    }
    m_pFuncApply = PyObject_GetAttrString(m_pInstance, func_name_apply);
    if (m_pFuncApply == nullptr) {
        PyErr_Print();
        fprintf(stderr, "Cannot find function \"%s\"\n", func_name_apply);
        return false;
    }

    // Call the initialize method
    PyObject* pRet = PyObject_CallObject(m_pFuncInitialize, nullptr);
    if (pRet != NULL)
    {
        PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
        if (pValue0 && !PyObject_IsTrue(pValue0))
        {
            Py_DECREF(pValue0);
            fprintf(stderr, "Unsuccessful instance initialization\n");
            return false;
        }
        Py_DECREF(pValue0);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "%s::initialize() - Call failed\n", class_name);
        return false;
    }

    return true;
}


void PythonModuleWrapper::_clear()
{
    if (m_pFuncInitialize != nullptr)
    {
        Py_DECREF(m_pFuncInitialize);
        m_pFuncInitialize = nullptr;
    }
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


#ifdef _WIN32
int setenv(const char* name, const char* value, int overwrite)
{
    int errcode = 0;
    if (!overwrite) {
        size_t envsize = 0;
        errcode = getenv_s(&envsize, NULL, 0, name);
        if (errcode || envsize) return errcode;
    }
    return _putenv_s(name, value);
}
#endif


bool init_python_environment(const char* name /*= "python3"*/, const char* import_path /*= nullptr*/)
{
    close_python_environment();

    // set the name of python executable to run (e.g. python, python3, python3.6, ...)
    wchar_t* program = Py_DecodeLocale(name, NULL);
    if (program == NULL) {
        fprintf(stderr, "Fatal error: cannot decode locale.\n");
        return false;
    }
    Py_SetProgramName(program);

    // initialize new python environment
    if (import_path != nullptr)
    {
        const char* path_old = std::getenv("PYTHONPATH");
        setenv("PYTHONPATH", import_path, 1);
        Py_Initialize();
        setenv("PYTHONPATH", path_old, 1);
    }
    else
    {
        Py_Initialize();
    }

    // Add current path to system path
    PyRun_SimpleString("import sys\nsys.path.append(\".\")");

    return true;
}


bool close_python_environment()
{
    if (Py_FinalizeEx() < 0) {
        return false;
    }
    return true;
}


} // End of 'dg'


#endif // End of '__DG_PYTHON_EMBEDDING__'
