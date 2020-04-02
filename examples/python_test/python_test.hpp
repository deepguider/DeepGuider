#ifndef __POI_RECOGNIZER__
#define __POI_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"

using namespace std;

namespace dg
{

struct TestResult
{
    double value1;
    double value2;
};

/**
    * @brief C++ Wrapper of Python module - PythonTest
    */
class PythonTest : public PythonModuleWrapper
{
public:
    /**
        * Initialize the module
        * @return true if successful (false if failed)
        */
    bool initialize()
    {
        return _initialize("python_test", ".", "PythonTest");
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
        * @return true if successful (false if failed)
        */
    bool apply(cv::Mat image, dg::Timestamp t)
    {
        // Set function arguments
        int arg_idx = 0;
        PyObject* pArgs = PyTuple_New(2);

        // Image
        import_array();
        npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
        PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
        if (!pValue) {
            fprintf(stderr, "PythonTest::apply() - Cannot convert argument1\n");
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
                fprintf(stderr, "PythonTest::apply() - Wrong number of returns\n");
                return false;
            }
            PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
            if (pValue0 != NULL) m_result.value1 = PyLong_AsLong(pValue0);
            PyObject* pValue1 = PyTuple_GetItem(pRet, 1);
            if (pValue1 != NULL) m_result.value2 = PyLong_AsLong(pValue1);
            Py_DECREF(pValue0);
            Py_DECREF(pValue1);
        }
        else {
            PyErr_Print();
            fprintf(stderr, "PythonTest::apply() - Call failed\n");
            return false;
        }

        // Update Timestamp
        m_timestamp = t;

        return true;
    }

    void get(TestResult& result)
    {
        result = m_result;
    }

    void get(TestResult& result, Timestamp& t)
    {
        result = m_result;
        t = m_timestamp;
    }

protected:
    TestResult m_result;
    Timestamp m_timestamp = -1;
};

} // End of 'dg'

#endif // End of '__POI_RECOGNIZER__'
