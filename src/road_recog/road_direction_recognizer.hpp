#ifndef __ROAD_DIRECTION_RECOGNIZER__
#define __ROAD_DIRECTION_RECOGNIZER__

#include "dg_core.hpp"

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "numpy/arrayobject.h"

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
	PyObject* pFuncApply = nullptr;

public:
	/**
	 * The default constructor
	 */
	RoadDirectionRecognizer() { }

	/**
	 * The default destructor
	 */
	~RoadDirectionRecognizer()
	{
		close();
	}

	void close()
	{
		if (pFuncApply != nullptr)
		{
			Py_DECREF(pFuncApply);
		}
		pFuncApply = nullptr;
	}

	/**
	 * Initialize the module
	 * @return true if successful (false if failed)
	 */
	bool initialize()
	{
		PyObject *pName, *pModule, *pClass;

		std::string src_name = "road_direction_recognizer";
		std::string class_name = "RoadDirectionRecognizer";
		std::string func_name_initialize = "initialize";
		std::string func_name_apply = "apply";

		// Add module path to system path
		PyRun_SimpleString("import sys\nsys.path.append(\"./../src/road_recog\")");

		// Build the name object (python srcname)
		pName = PyUnicode_FromString(src_name.c_str());

		// Load the module object (import python src)
		pModule = PyImport_Import(pName);
		Py_DECREF(pName);
		if (pModule == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Fails to import the module \"%s\"\n", src_name.c_str());
			return false;
		}

		// Get the class reference
		pClass = PyObject_GetAttrString(pModule, class_name.c_str());
		Py_DECREF(pModule);
		if (pClass == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find class \"%s\"\n", class_name.c_str());
			return false;
		}

		// Get the method reference of the class
		pFuncApply = PyObject_GetAttrString(pClass, func_name_apply.c_str());
		if (pFuncApply == nullptr) {
			Py_DECREF(pClass);
			PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", func_name_apply.c_str());
			return false;
		}

		// Call the module initialization
		PyObject *pFuncInitialize = PyObject_GetAttrString(pClass, func_name_initialize.c_str());
		Py_DECREF(pClass);
		if (pFuncInitialize == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", func_name_initialize.c_str());
			return false;
		}

		PyObject* pArgs = PyTuple_New(1);
		PyTuple_SetItem(pArgs, 0, pFuncInitialize);
		PyObject_CallObject(pFuncInitialize, pArgs);

		return true;
	}

	/**
	 * Run the module for a given input
	 * @return true if successful (false if failed)
	 */
	bool apply(cv::Mat image, Timestamp t)
	{
		// Set function arguments
		PyObject *pArgs = PyTuple_New(3);
		PyTuple_SetItem(pArgs, 0, pFuncApply);

		// Image
		import_array();
		npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
		PyObject* pValue1 = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
		if (!pValue1) {
			fprintf(stderr, "RoadDirectionRecognizer::apply() - Cannot convert argument1\n");
			return false;
		}
		PyTuple_SetItem(pArgs, 1, pValue1);

		// Timestamp
		PyObject* pValue2 = PyFloat_FromDouble(t);
		if (!pValue2) {
			fprintf(stderr, "RoadDirectionRecognizer::apply() - Cannot convert argument2\n");
			return false;
		}
		PyTuple_SetItem(pArgs, 2, pValue2);

		// Call the python function
		PyObject *pRet = PyObject_CallObject(pFuncApply, pArgs);
		if (pRet != NULL) {
			int n_ret = PyTuple_Size(pRet);
			if (n_ret != 2)
			{
				fprintf(stderr, "RoadDirectionRecognizer::apply() - Wrong number of returns\n");
				return false;
			}
			PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
			if (pValue0 != NULL) angle = PyLong_AsLong(pValue0) + t;
			PyObject* pValue1 = PyTuple_GetItem(pRet, 1);
			if (pValue1 != NULL) prob = PyLong_AsLong(pValue1);
		}
		else {
			PyErr_Print();
			fprintf(stderr, "Call failed\n");
			return false;
		}

		return true;
	}

	void get(double& _angle, double& _prob)
	{
		_angle = angle;
		_prob = prob;
	}
};

} // End of 'dg'

#endif // End of '__ROAD_DIRECTION_RECOGNIZER__'
