#ifndef __ROAD_DIRECTION_RECOGNIZER__
#define __ROAD_DIRECTION_RECOGNIZER__

#include "dg_core.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy\arrayobject.h"

using namespace std;

namespace dg
{

/**
 * @brief Road direction recognizer
 */
class RoadDirectionRecognizer
{
	double angle = -1;
	double prob = -1;

public:
	RoadDirectionRecognizer()
	{
	}

	int apply(cv::Mat image, Timestamp t)
	{
		std::string src_name = "road_direction_recognizer";
		std::string class_name = "RoadDirectionRecognizer";
		std::string func_name = "apply";

		PyObject* pName, * pModule, * pDict, * pClass, * pFunc;
		PyObject* pArgs, * pValue;

		wchar_t* program = Py_DecodeLocale("road_recog_test", NULL);
		if (program == NULL) {
			fprintf(stderr, "Fatal error: cannot decode road_recog_test\n");
			exit(-1);
		}
		Py_SetProgramName(program);
		Py_Initialize();

		// Add module path to system path
		PyRun_SimpleString("import sys\nsys.path.append(\"./../src/road_recog\")");

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

		// Get the class reference
		pClass = PyObject_GetAttrString(pModule, class_name.c_str());
		if (pClass == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find class \"%s\"\n", class_name.c_str());
			return -1;
		}
		Py_DECREF(pModule);

		// Get the method reference of the class
		pFunc = PyObject_GetAttrString(pClass, func_name.c_str());
		if (pClass == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", func_name.c_str());
			return -1;
		}
		Py_DECREF(pClass);

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

	void get(double& _angle, double& _prob)
	{
		_angle = angle;
		_prob = prob;
	}
};

} // End of 'dg'

#endif // End of '__ROAD_DIRECTION_RECOGNIZER__'
