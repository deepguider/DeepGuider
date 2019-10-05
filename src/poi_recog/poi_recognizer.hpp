#ifndef __POI_RECOGNIZER__
#define __POI_RECOGNIZER__

#include "../dg_core.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy/arrayobject.h"

using namespace std;

namespace dg
{

/**
 * @brief Road direction recognizer
 */
class POIRecognizer
{
	int x1 = 0;
    int y1 = 0;
    int x2 = -1;
    int y2 = -1;
	double prob = -1;

public:
	POIRecognizer()
	{
	}

	int apply(cv::Mat image, Timestamp t)
	{
		std::string src_name = "logos";
        //std::string class_name = "POIRecognizer";
		std::string func_name = "detect_logo_demo";

		PyObject* pName, * pModule, * pDict, * pClass, * pFunc;
		PyObject* pArgs, * pValue;

		wchar_t* program = Py_DecodeLocale("poi_recog_test", NULL);
		if (program == NULL) {
			fprintf(stderr, "Fatal error: cannot decode poi_recog_test\n");
			exit(-1);
		}
		Py_SetProgramName(program);
		Py_Initialize();

		// Add module path to system path
		PyRun_SimpleString("import sys\nsys.path.append(\"./../src/poi_recog\")");

		// Build the name object (python srcname)
		pName = PyUnicode_FromString(src_name.c_str());

		// Load the module object (import python src)
		pModule = PyImport_Import(pName);
		if (pModule == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Fails to import the module \"%s\"\n", src_name.c_str());
			return -1;
		}
		Py_DECREF(pName);
        
        /*
		// Get the class reference
		pClass = PyObject_GetAttrString(pModule, class_name.c_str());
		if (pClass == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find class \"%s\"\n", class_name.c_str());
			return -1;
		}
		Py_DECREF(pModule);
        */
		// Get the method reference of the class
		pFunc = PyObject_GetAttrString(pModule, func_name.c_str());
		if (pClass == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", func_name.c_str());
			return -1;
		}
		Py_DECREF(pModule); // (pClass);

		// Set function arguments
		pArgs = PyTuple_New(3);
		PyTuple_SetItem(pArgs, 0, pFunc);

		// Image
		import_array();
		npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
		pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)& dimensions, NPY_UINT8, image.data);
		PyTuple_SetItem(pArgs, 1, pValue);

		// Timestamp
		pValue = PyFloat_FromDouble(t);
		if (!pValue) {
			Py_DECREF(pArgs);
			Py_DECREF(pFunc);
			fprintf(stderr, "Cannot convert argument\n");
			return -1;
		}
		PyTuple_SetItem(pArgs, 2, pValue);

		// Call the python function
		pValue = PyObject_CallObject(pFunc, pArgs);
		Py_DECREF(pArgs);
		if (pValue != NULL) {
			int n_ret = PyTuple_Size(pValue);
			if (n_ret != 2)
			{
				Py_DECREF(pFunc);
				fprintf(stderr, "Wrong number of returns\n");
				return -1;
			}					   
			PyObject* pValue0 = PyTuple_GetItem(pValue, 0);
			if(pValue0 != NULL) angle = PyLong_AsLong(pValue0);
			PyObject* pValue1 = PyTuple_GetItem(pValue, 1);
			if (pValue1 != NULL) prob = PyLong_AsLong(pValue1);
			Py_DECREF(pValue0);
			Py_DECREF(pValue1);
			Py_DECREF(pValue);
		}
		else {
			Py_DECREF(pFunc);
			PyErr_Print();
			fprintf(stderr, "Call failed\n");
			return -1;
		}
		Py_DECREF(pFunc);

		// Finish the Python Interpreter
		if (Py_FinalizeEx() < 0) {
			return -1;
		}
		return 0;
	}

	void get(double& _angle, double& _prob) //(int& _x1, int& _y1, int& _x2, int& _y2, double& _prob)
	{
		//_x1 = x1;
        //_y1 = y1;
        //_x2 = x2;
        //_y2 = y2;
        _angle = 36.5;
		_prob = prob;
	}
};

} // End of 'dg'

#endif // End of '__POI_RECOGNIZER__'
