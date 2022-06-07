#ifndef __OCR_RECOGNIZER__
#define __OCR_RECOGNIZER__

#include "dg_core.hpp"
#include "utils/python_embedding.hpp"
#include "utils/utility.hpp"
#include "utils/opencx.hpp"
#ifdef HAVE_OPENCV_FREETYPE
#include "opencv2/freetype.hpp"
#endif
#include <fstream>
#include <chrono>
#include <locale>
#include <codecvt>

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
        * Constructor
        */
        OCRRecognizer()
        {
#ifdef HAVE_OPENCV_FREETYPE
            m_ft2 = cv::freetype::createFreeType2();
            m_ft2->loadFontData("font/gulim.ttf", 0);
#endif
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
                cv::AutoLock lock(m_mutex);
                m_result.clear();
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
                            //ocr.label = cx::toLowerCase(PyUnicode_AsUTF8(pValue)); // convert to lowercase
                            ocr.label = PyUnicode_AsUTF8(pValue);
                            pValue = PyList_GetItem(pList, idx++);
                            ocr.confidence = PyFloat_AsDouble(pValue);
                            m_result.push_back(ocr);
                        }
                    }
                }
            }
            else {
                PyErr_Print();
                fprintf(stderr, "OCRRecognizer::apply() - Call failed\n");
                return false;
            }

            // Clean up
            if(pRet) Py_DECREF(pRet);
            if(pArgs) Py_DECREF(pArgs);            

            return true;
        }

        std::vector<OCRResult> get() const
        {
            cv::AutoLock lock(m_mutex);
            return m_result;
        }

        std::vector<OCRResult> get(Timestamp& ts) const
        {
            cv::AutoLock lock(m_mutex);
            ts = m_timestamp;
            return m_result;
        }

        void set(const std::vector<OCRResult>& result, Timestamp ts, double proc_time = -1)
        {
            cv::AutoLock lock(m_mutex);
            m_result = result;
            m_timestamp = ts;
            m_processing_time = proc_time;
        }

        void draw(cv::Mat& image, double drawing_scale = 1.0, int font_sz = 28, double xscale = 1, double yscale = 1, cv::Scalar color = cv::Scalar(0, 255, 0)) const
        {
            cv::AutoLock lock(m_mutex);
            for (size_t i = 0; i < m_result.size(); i++)
            {
                // bbox
                int xmin = (int)(xscale * m_result[i].xmin + 0.5);
                int ymin = (int)(yscale * m_result[i].ymin + 0.5);
                int xmax = (int)(xscale * m_result[i].xmax + 0.5);
                int ymax = (int)(yscale * m_result[i].ymax + 0.5);
                cv::Rect rc(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1);
                cv::rectangle(image, rc, color, (int)(2 * drawing_scale));

                // label
                int sz = (rc.width < rc.height) ? rc.width : rc.height;
                if(sz<10) font_sz = 10;
                cv::Point2d pt(xmin + 3 * drawing_scale, ymin - 5 * drawing_scale);
                std::string msg = cv::format("%s (%.2lf)", m_result[i].label.c_str(), m_result[i].confidence);
#ifdef HAVE_OPENCV_FREETYPE
                if(m_ft2)
                {
                    int font_height = (int)(font_sz * drawing_scale + 0.5);
                    if (pt.y - font_height < 0) pt.y = font_height;
                    if (pt.y >= image.rows) pt.y = image.rows - 1;
                    m_ft2->putText(image, msg, pt, font_height, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, true);
                    m_ft2->putText(image, msg, pt, font_height, cv::Scalar(0, 255, 255), 1, cv::LINE_AA, true);
                    m_ft2->putText(image, msg, pt, font_height, cv::Scalar(0, 255, 255), -1, cv::LINE_AA, true);
                }
                else
#endif
                {
                    cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5 * drawing_scale, cv::Scalar(0, 255, 255), (int)(6 * drawing_scale));
                    cv::putText(image, msg, pt, cv::FONT_HERSHEY_PLAIN, 1.5 * drawing_scale, cv::Scalar(255, 0, 0), (int)(2 * drawing_scale));
                }
            }
        }

        void print() const
        {
            std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
            cv::AutoLock lock(m_mutex);
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), m_processing_time, m_timestamp);
            for (int k = 0; k < m_result.size(); k++)
            {
                std::wstring wlabel = converter.from_bytes(m_result[k].label);
                wprintf(L"\t%ls, %.2lf, x1=%d, y1=%d, x2=%d, y2=%d\n", wlabel.c_str(), m_result[k].confidence, m_result[k].xmin, m_result[k].ymin, m_result[k].xmax, m_result[k].ymax);
            }
        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            cv::AutoLock lock(m_mutex);
            for (int k = 0; k < m_result.size(); k++)
            {
                std::string log = cv::format("%.3lf,%d,%s,%s,%.2lf,%d,%d,%d,%d,%.3lf", m_timestamp, cam_fnumber, name(), m_result[k].label.c_str(), m_result[k].confidence, m_result[k].xmin, m_result[k].ymin, m_result[k].xmax, m_result[k].ymax, m_processing_time);
                stream << log << std::endl;
            }
        }

        void read(const std::vector<std::string>& stream)
        {
            cv::AutoLock lock(m_mutex);
            m_result.clear();
            for (int k = 0; k < (int)stream.size(); k++)
            {
                std::vector<std::string> elems = splitStr(stream[k].c_str(), (int)stream[k].length(), ',');
                if (elems.size() != 10)
                {
                    printf("[ocr] Invalid log data %s\n", stream[k].c_str());
                    return;
                }
                std::string module_name = elems[2];
                if (module_name == name())
                {
                    OCRResult r;
                    r.label = elems[3];
                    r.confidence = atof(elems[4].c_str());
                    r.xmin = atoi(elems[5].c_str());
                    r.ymin = atoi(elems[6].c_str());
                    r.xmax = atoi(elems[7].c_str());
                    r.ymax = atoi(elems[8].c_str());
                    m_result.push_back(r);

                    m_timestamp = atof(elems[0].c_str());
                    m_processing_time = atof(elems[9].c_str());
                }
            }
        }

        static const char* name()
        {
            return "ocr";
        }

    protected:
        std::vector<OCRResult> m_result;
#ifdef HAVE_OPENCV_FREETYPE
        cv::Ptr<cv::freetype::FreeType2> m_ft2;
#endif
    };

} // End of 'dg'

#endif // End of '__OCR_RECOGNIZER__'
