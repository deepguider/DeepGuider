#ifndef __POI_RECOGNIZER__
#define __POI_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"

using namespace std;

namespace dg
{
    struct POIResult
    {
        int xmin, ymin, xmax, ymax;
        std::string label;
        double confidence;
    };

    /**
     * @brief C++ Wrapper of Python module - POIRecognizer
     */
    class POIRecognizer : public PythonModuleWrapper
    {
    public:
        /**
         * Initialize the module
         * @return true if successful (false if failed)
         */
        bool initialize()
        {
            return _initialize("poi_recognizer", "./../src/poi_recog", "POIRecognizer");
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
                fprintf(stderr, "POIRecognizer::apply() - Cannot convert argument1\n");
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
                    fprintf(stderr, "POIRecognizer::apply() - Wrong number of returns\n");
                    return false;
                }

                // list of list
                m_pois.clear();
                PyObject* pValue0 = PyTuple_GetItem(pRet, 0);
                if (pValue0 != NULL)
                {
                    Py_ssize_t cnt = PyList_Size(pValue0);
                    for (int i = 0; i < cnt; i++)
                    {
                        PyObject* pList = PyList_GetItem(pValue0, i);

                        POIResult poi;
                        int idx = 0;
                        pValue = PyList_GetItem(pList, idx++);
                        poi.xmin = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.ymin = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.xmax = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.ymax = PyLong_AsLong(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.label = PyUnicode_AsUTF8(pValue);
                        pValue = PyList_GetItem(pList, idx++);
                        poi.confidence = PyFloat_AsDouble(pValue);

                        m_pois.push_back(poi);
                        Py_DECREF(pList);
                    }
                }
                Py_DECREF(pValue0);
            }
            else {
                PyErr_Print();
                fprintf(stderr, "POIRecognizer::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = t;

            return true;
        }

        void get(std::vector<POIResult>& pois)
        {
            pois = m_pois;
        }

        void get(std::vector<POIResult>& pois, Timestamp& t)
        {
            pois = m_pois;
            t = m_timestamp;
        }

    protected:
        std::vector<POIResult> m_pois;
        Timestamp m_timestamp = -1;
    };

} // End of 'dg'

#endif // End of '__POI_RECOGNIZER__'
