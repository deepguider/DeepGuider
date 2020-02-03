#ifndef __DG_PYTHON_EMBEDDING__
#define __DG_PYTHON_EMBEDDING__

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdlib.h>


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

bool close_python_environment()
{
    if (Py_FinalizeEx() < 0) {
        return false;
    }
    return true;
}

bool init_python_environment(const char* name = "python3", const char* path = "")
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
    const char* path_old = std::getenv("PYTHONPATH");
    setenv("PYTHONPATH", path, 1);
    Py_Initialize();
    setenv("PYTHONPATH", path_old, 1);
    
    // Add current path to system path
    PyRun_SimpleString("import sys\nsys.path.append(\".\")"); 

    return true;
}

#endif // End of '__DG_PYTHON_EMBEDDING__'
