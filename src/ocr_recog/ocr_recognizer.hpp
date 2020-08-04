#ifndef __OCR_RECOGNIZER__
#define __OCR_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#ifdef HAVE_OPENCV_FREETYPE
#include "opencv2/freetype.hpp"
#endif
#include <fstream>
#include <chrono>


using namespace std;

namespace dg
{
    struct OCRResult
    {
        int xmin, ymin, xmax, ymax;
        std::string label;
        double confidence;
    };

    /**
    * @brief C++ Wrapper of Python module - OCRRecognizer
    */
    class OCRRecognizer : public PythonModuleWrapper
    {
    public:
        /**
        * Initialize the module
        * @return true if successful (false if failed)
        */
        bool initialize(const char* module_name = "ocr_recognizer", const char* module_path = "./../src/ocr_recog", const char* class_name = "OCRRecognizer", const char* func_name_init = "initialize", const char* func_name_apply = "apply")
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            bool ret = _initialize(module_name, module_path, class_name, func_name_init, func_name_apply);

            if (isThreadingEnabled()) PyGILState_Release(state);

#ifdef HAVE_OPENCV_FREETYPE
            m_ft2 = cv::freetype::createFreeType2();
            m_ft2->loadFontData("font/gulim.ttf", 0);
#endif

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            m_processing_time = t2 - t1;

            return ret;
        }

        /**
        * Reset variables and clear the memory
        */
        void clear()
        {
            PyGILState_STATE state;
            if (isThreadingEnabled()) state = PyGILState_Ensure();

            _clear();

            if (isThreadingEnabled()) PyGILState_Release(state);

            m_ocrs.clear();
            m_timestamp = -1;
            m_processing_time = -1;
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

            return ret;
        }

        /**
        * Run once the module for a given input
        * @return true if successful (false if failed)
        */
        bool _apply(cv::Mat image, dg::Timestamp ts)
        {
            // Set function arguments
            int arg_idx = 0;
            PyObject* pArgs = PyTuple_New(2);

            // Image
            import_array();
            npy_intp dimensions[3] = { image.rows, image.cols, image.channels() };
            PyObject* pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);
            if (!pValue) {
                fprintf(stderr, "OCRRecognizer::apply() - Cannot convert argument1\n");
                return false;
            }
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Timestamp
            pValue = PyFloat_FromDouble(ts);
            PyTuple_SetItem(pArgs, arg_idx++, pValue);

            // Call the method
            PyObject* pRet = PyObject_CallObject(m_pFuncApply, pArgs);
            if (pRet != NULL) {
                Py_ssize_t n_ret = PyTuple_Size(pRet);
                if (n_ret != 2)
                {
                    fprintf(stderr, "OCRRecognizer::apply() - Wrong number of returns\n");
                    return false;
                }

                // list of list: [[xmin:float, ymin:float, xmax:float, ymax:float]:list, label:str, confidence:float], timestamp
                m_ocrs.clear();
                PyObject* pList0 = PyTuple_GetItem(pRet, 0);
                if (pList0 != NULL)
                {
                    Py_ssize_t cnt = PyList_Size(pList0);
                    for (int i = 0; i < cnt; i++)
                    {
                        PyObject* pList = PyList_GetItem(pList0, i);
                        if(pList)
                        {
                            OCRResult ocr;
                            int idx = 0;
                            PyObject* pListCoordinate = PyList_GetItem(pList, idx++);
                            if (pListCoordinate != NULL)
                            {
                                int idx2 = 0;
                                pValue = PyList_GetItem(pListCoordinate, idx2++);
                                ocr.xmin = (int)PyFloat_AsDouble(pValue);
                                pValue = PyList_GetItem(pListCoordinate, idx2++);
                                ocr.ymin = (int)PyFloat_AsDouble(pValue);
                                pValue = PyList_GetItem(pListCoordinate, idx2++);
                                ocr.xmax = (int)PyFloat_AsDouble(pValue);
                                pValue = PyList_GetItem(pListCoordinate, idx2++);
                                ocr.ymax = (int)PyFloat_AsDouble(pValue);
                            }
                            pValue = PyList_GetItem(pList, idx++);
                            ocr.label = PyUnicode_AsUTF8(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            ocr.confidence = PyFloat_AsDouble(pValue);
                            m_ocrs.push_back(ocr);
                        }
                    }
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "OCRRecognizer::apply() - Call failed\n");
                return false;
            }

            // Update Timestamp
            m_timestamp = ts;

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);            

            return true;
        }

        void get(std::vector<OCRResult>& ocrs) const
        {
            ocrs = m_ocrs;
        }

        void get(std::vector<OCRResult>& ocrs, Timestamp& ts) const
        {
            ocrs = m_ocrs;
            ts = m_timestamp;
        }

        double procTime() const
        {
            return m_processing_time;
        }

        void draw(cv::Mat& image, cv::Scalar color = cv::Scalar(0, 255, 0), int width = 2) const
        {
            for (size_t i = 0; i < m_ocrs.size(); i++)
            {
                cv::Rect rc(m_ocrs[i].xmin, m_ocrs[i].ymin, m_ocrs[i].xmax - m_ocrs[i].xmin + 1, m_ocrs[i].ymax - m_ocrs[i].ymin + 1);
                cv::rectangle(image, rc, color, width);
                cv::Point pt(m_ocrs[i].xmin + 5, m_ocrs[i].ymin + 35);
                std::string msg = cv::format("%s %.2lf", m_ocrs[i].label.c_str(), m_ocrs[i].confidence);
#ifdef HAVE_OPENCV_FREETYPE
                if(m_ft2)
                {
                    m_ft2->putText(image, msg, pt, 30, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, true);
                    m_ft2->putText(image, msg, pt, 30, cv::Scalar(0, 0, 0), -1, cv::LINE_AA, true);
                }
                else
#endif
                {
                    cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 6);
                    cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0), 2);
                }
            }
        }

        void print() const
        {
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), procTime(), m_timestamp);
            for (int k = 0; k < m_ocrs.size(); k++)
            {
                printf("\t%s, %.2lf, x1=%d, y1=%d, x2=%d, y2=%d\n", m_ocrs[k].label.c_str(), m_ocrs[k].confidence, m_ocrs[k].xmin, m_ocrs[k].ymin, m_ocrs[k].xmax, m_ocrs[k].ymax);
            }
        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            for (int k = 0; k < m_ocrs.size(); k++)
            {
                std::string log = cv::format("%.3lf,%d,%s,%s,%.2lf,%d,%d,%d,%d,%.3lf", m_timestamp, cam_fnumber, name(), m_ocrs[k].label.c_str(), m_ocrs[k].confidence, m_ocrs[k].xmin, m_ocrs[k].ymin, m_ocrs[k].xmax, m_ocrs[k].ymax, m_processing_time);
                stream << log << std::endl;
            }
        }

        static const char* name()
        {
            return "ocr";
        }


    protected:
        std::vector<OCRResult> m_ocrs;
        Timestamp m_timestamp = -1;
        double m_processing_time = -1;
#ifdef HAVE_OPENCV_FREETYPE
        cv::Ptr<cv::freetype::FreeType2> m_ft2;
#endif
    };

} // End of 'dg'

#endif // End of '__OCR_RECOGNIZER__'
