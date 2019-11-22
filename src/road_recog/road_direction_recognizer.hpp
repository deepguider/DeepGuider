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
 * @brief C++ Wrapper of Python module - Road direction recognizer
 */
class RoadDirectionRecognizer
{
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
		clear();
	}

	/**
	 * Reset variables and clear the memory
	 */
	void clear()
	{
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

	/**
	 * Initialize the module
	 * @return true if successful (false if failed)
	 */
	bool initialize()
	{
		std::string module_name = "road_direction_recognizer";	// python module name
		std::string class_name = "RoadDirectionRecognizer";		// python class name
		std::string func_name_init = "initialize";				// python method name for initialization
		std::string func_name_apply = "apply";					// python method name for apply

		// Add module path to system path
		PyRun_SimpleString("import sys\nsys.path.append(\"./../src/road_recog\")");

		// Import the Python module
		PyObject* pName = PyUnicode_FromString(module_name.c_str());
		PyObject* pModule = PyImport_Import(pName);
		Py_DECREF(pName);
		if (pModule == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Fails to import the module \"%s\"\n", module_name.c_str());
			return false;
		}

		// Get the class reference
		PyObject* pClass = PyObject_GetAttrString(pModule, class_name.c_str());
		Py_DECREF(pModule);
		if (pClass == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find class \"%s\"\n", class_name.c_str());
			return false;
		}

		// Get the method references of the class
		m_pFuncApply = PyObject_GetAttrString(pClass, func_name_apply.c_str());
		if (m_pFuncApply == nullptr) {
			Py_DECREF(pClass);
			PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", func_name_apply.c_str());
			return false;
		}
		PyObject *pFuncInitialize = PyObject_GetAttrString(pClass, func_name_init.c_str());
		if (pFuncInitialize == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", func_name_init.c_str());
			return false;
		}

		// Create the class instance
		m_pInstance = PyClassMethod_New(pClass);
		Py_DECREF(pClass);
		if (m_pInstance == nullptr) {
			PyErr_Print();
			fprintf(stderr, "Cannot create class instance \"%s\"\n", class_name.c_str());
			return false;
		}

		// Call the initialization method of the class instance
		PyObject* pArgs = PyTuple_New(1);
		PyTuple_SetItem(pArgs, 0, m_pInstance);
		PyObject* pRet = PyObject_CallObject(pFuncInitialize, pArgs);
		if (pRet != NULL)
		{
			if (!PyObject_IsTrue(pRet))
			{
				fprintf(stderr, "Unsuccessful instance initialization\n");
				return false;
			}
		}
		else {
			PyErr_Print();
			fprintf(stderr, "RoadDirectionRecognizer::initialize() - Call failed\n");
			return false;
		}

		return true;
	}

	/**
	 * Run once the module for a given input
	 * @return true if successful (false if failed)
	 */
	bool apply(cv::Mat image, Timestamp t)
	{
		// Set function arguments
		PyObject *pArgs = PyTuple_New(3);
		PyTuple_SetItem(pArgs, 0, m_pInstance);

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

		// Call the method
		PyObject *pRet = PyObject_CallObject(m_pFuncApply, pArgs);
		if (pRet != NULL) {
			int n_ret = PyTuple_Size(pRet);
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

	PyObject* m_pFuncApply = nullptr;
	PyObject* m_pInstance = nullptr;
};

} // End of 'dg'

#endif // End of '__ROAD_DIRECTION_RECOGNIZER__'
