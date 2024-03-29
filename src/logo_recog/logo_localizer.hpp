#ifndef __LOGO_LOCALIZER__
#define __LOGO_LOCALIZER__

#include "dg_core.hpp"
#include "logo_recog/logo_recognizer.hpp"
#include "utils/opencx.hpp"
#include <locale>
#include <codecvt>

using namespace std;

namespace dg
{
    /**
    * @brief Logo Localizer
    */
    class LogoLocalizer : public LogoRecognizer
    {
	protected:
		// camera parameters
		double m_focal_length = 772;	    // Focal length of camera, Unit: [px]
		double m_cx = 640;					// X coordinate of principal point, Unit: [px]
		double m_cy = 360;					// Y coordinate of principal point, Unit: [px]
		double m_camera_height = 0.83;		// Height of camera from ground plane, Unit: [m]
		double m_vanishing_y = 419.8;		// Y coordinate of image vanishing line, Unit: [px]

		// configuable parameters
		double m_poi_height = 3.8;			// Height of POI from ground plane, Unit: [m]
		double m_poi_search_radius = 100;	// POI search range, Unit: [m]

    public:
        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/logo_recog")
        {
			if (!LogoRecognizer::initialize(py_module_path.c_str(), "logo_recognizer", "LogoRecognizer")) return false;

			cv::AutoLock lock(m_localizer_mutex);
			m_shared = shared;
            return (m_shared != nullptr);
        }

		bool initialize_without_python(SharedInterface* shared)
		{
			cv::AutoLock lock(m_localizer_mutex);
			m_shared = shared;
			return (m_shared != nullptr);
		}

		void setParam(double f, double cx, double cy, double cam_vy, double cam_h, double poi_h = 3.8, double search_radius = 100)
		{
			cv::AutoLock lock(m_localizer_mutex);
			m_focal_length = f;
			m_cx = cx;
			m_cy = cy;
			m_vanishing_y = cam_vy;
			m_camera_height = cam_h;
			m_poi_height = poi_h;
			m_poi_search_radius = search_radius;
		}

        bool apply(const cv::Mat image, const dg::Timestamp image_time, std::vector<dg::Point2>& poi_xys, std::vector<dg::Polar2>& relatives, std::vector<double>& poi_confidences)
        {
			if (!LogoRecognizer::apply(image, image_time)) return false;

			cv::AutoLock lock(m_localizer_mutex);
			std::vector<LogoResult> logos = get();
			if (logos.empty()) return false;

			if (m_shared == nullptr) return false;
			Pose2 pose = m_shared->getPose();
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return false;

            std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			for (int k = 0; k < logos.size(); k++)
            {
                std::wstring poi_name = converter.from_bytes(logos[k].label.c_str());
                std::vector<POI*> pois = map->getPOI(poi_name, pose, m_poi_search_radius, true);
                if (!pois.empty())
                {
                    POI* poi = pois[0];
                    poi_xys.push_back(*poi);
                    Polar2 relative = computeRelative(logos[k].xmin, logos[k].ymin, logos[k].xmax, logos[k].ymax);
                    relatives.push_back(relative);
                    poi_confidences.push_back(logos[k].confidence);
                }
            }
            return true;
        }

		bool getLocClue(const Pose2& pose, std::vector<dg::Point2>& poi_xys, std::vector<dg::Polar2>& relatives, std::vector<double>& poi_confidences)
		{
			cv::AutoLock lock(m_localizer_mutex);
			std::vector<LogoResult> logos = get();
			if (logos.empty()) return false;

			if (m_shared == nullptr) return false;
			Map* map = m_shared->getMap();
			assert(map != nullptr);

			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			for (int k = 0; k < logos.size(); k++)
			{
				std::wstring poi_name = converter.from_bytes(logos[k].label.c_str());
				std::vector<POI*> pois = map->getPOI(poi_name, pose, m_poi_search_radius, true);
				if (!pois.empty())
				{
					POI* poi = pois[0];
					poi_xys.push_back(*poi);
					Polar2 relative = computeRelative(logos[k].xmin, logos[k].ymin, logos[k].xmax, logos[k].ymax);
					relatives.push_back(relative);
					poi_confidences.push_back(logos[k].confidence);
				}
			}
			return true;
		}

    protected:
        dg::Polar2 computeRelative(int x1, int y1, int x2, int y2)
        {
            dg::Polar2 relative = dg::Polar2(-1, CV_PI);

			// camera parameters
			double camera_tilt = atan((m_vanishing_y - m_cy) / m_focal_length);	// camera tilt
			double cam_pan = CV_PI / 2;

			// R|t matrix
			cv::Matx33d R = rotationFromPanTiltRoll(cam_pan, camera_tilt, 0);
			cv::Matx31d t = -R * cv::Matx31d(0, 0, m_camera_height);

			// poi top
			double poi_tx = (x1 + x2) / 2.0;
			double poi_ty = (y1 + y2) / 2.0;

			// camera coordinate of poi top
			cv::Mat pc(3, 1, CV_64FC1);
			pc.at<double>(0) = (poi_tx - m_cx) / m_focal_length;
			pc.at<double>(1) = (poi_ty - m_cy) / m_focal_length;
			pc.at<double>(2) = 1;

			// pan, tilt of poi top
			cv::Mat pw = cv::Mat(R.t()) * pc;
			double xw = pw.at<double>(0);
			double yw = pw.at<double>(1);
			double zw = pw.at<double>(2);
			double poi_pan = atan2(yw, xw);
			double poi_tilt = atan2(zw, sqrt(xw * xw + yw * yw));

			// world distance from camera to poi
			double D = (m_poi_height - m_camera_height) / tan(poi_tilt);

			// result
			relative.lin = D;
			relative.ang = poi_pan - CV_PI / 2;
			return relative;
		}

		cv::Matx33d rotationFromPanTiltRoll(double pan, double tilt, double roll)
		{
			cv::Matx33d R;
			R(0, 0) = sin(pan) * cos(roll) - cos(pan) * sin(tilt) * sin(roll);
			R(0, 1) = -cos(pan) * cos(roll) - sin(pan) * sin(tilt) * sin(roll);
			R(0, 2) = cos(tilt) * sin(roll);
			R(1, 0) = sin(pan) * sin(roll) + sin(tilt) * cos(pan) * cos(roll);
			R(1, 1) = -cos(pan) * sin(roll) + sin(tilt) * sin(pan) * cos(roll);
			R(1, 2) = -cos(tilt) * cos(roll);
			R(2, 0) = cos(tilt) * cos(pan);
			R(2, 1) = cos(tilt) * sin(pan);
			R(2, 2) = sin(tilt);

			return R;
		}

        SharedInterface* m_shared = nullptr;
		cv::Mutex m_localizer_mutex;
    };

} // End of 'dg'

#endif // End of '__LOGO_LOCALIZER__'
