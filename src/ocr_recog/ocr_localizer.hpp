#ifndef __OCR_LOCALIZER__
#define __OCR_LOCALIZER__

#include "dg_core.hpp"
#include "ocr_recog/ocr_recognizer.hpp"
#include "utils/opencx.hpp"
#include <locale>
#include <codecvt>

#define MAP_BUF_SIZE               (1024)

using namespace std;

namespace dg
{	
	bool compare_dist(std::tuple<double, double, std::wstring> a, std::tuple<double, double, std::wstring> b)
	{
		if(std::get<1>(a) == std::get<1>(b)) return std::get<0>(a) > std::get<0>(b);

		return std::get<1>(a) > std::get<1>(b);
	}

    /**
    * @brief OCR Localizer
    */
    class OCRLocalizer : public OCRRecognizer
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
        bool initialize(SharedInterface* shared, std::string py_module_path = "./../src/ocr_recog")
        {
			if (!OCRRecognizer::initialize(py_module_path.c_str(), "ocr_recognizer", "OCRRecognizer")) return false;

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
		
		bool setWeights(const char* filename = "dg_hangeul_weights.csv")
		{
			FILE* fid = fopen(filename, "rt,ccs=UTF-8");
			if (fid == nullptr) return false;

			wchar_t buffer[MAP_BUF_SIZE];
			wchar_t* context;
			while (!feof(fid))
			{
				if (fgetws(buffer, MAP_BUF_SIZE, fid) == nullptr) break;
				wchar_t* token;
				if ((token = wcstok(buffer, L",", &context)) == nullptr) goto WEIGHT_FAIL;
				if (token[0] == 'J' || token[0] == 'j')
				{
					// Read jaum
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					wchar_t c1 = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					wchar_t c2 = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					double similiraty = std::stod(cx::trimBoth(token), nullptr);

					addWeight(c1, c2, similiraty);
				}
				else if (token[0] == 'M' || token[0] == 'm')
				{
					// Read moum
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					wchar_t c1 = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					wchar_t c2 = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					double similiraty = std::stod(cx::trimBoth(token), nullptr);

					addWeight(c1, c2, similiraty);
				}
				else if (token[0] == 'S' || token[0] == 's')
				{
					// Read syllable
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					wchar_t c1 = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					wchar_t c2 = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					double similiraty = std::stod(cx::trimBoth(token), nullptr);

					addWeight(c1, c2, similiraty);
				}
			}
			fclose(fid);
			return true;

		WEIGHT_FAIL:
			fclose(fid);
			return false;
		}

        bool apply(const cv::Mat image, const dg::Timestamp image_time, std::vector<dg::Point2>& poi_xys, std::vector<dg::Polar2>& relatives, std::vector<double>& poi_confidences)
        {
			if (!OCRRecognizer::apply(image, image_time)) return false;

			cv::AutoLock lock(m_localizer_mutex);
			std::vector<OCRResult> ocrs = get();
			if (ocrs.empty()) return false;

            if (m_shared == nullptr) return false;
			Pose2 pose = m_shared->getPose();
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return false;

            std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			for (int k = 0; k < ocrs.size(); k++)
            {
                std::wstring ocr_result = converter.from_bytes(ocrs[k].label.c_str());
				std::vector<std::tuple<double, double, std::wstring>> poi_names = matchPOIName(ocr_result, pose, 300.0, 1);
				if (poi_names.empty()) return false;
				for (int n = 0; n < poi_names.size(); n++)
            	{
					std::tuple<double, double, std::wstring> poi_name = poi_names.at(n);

					std::vector<POI*> pois = map->getPOI(std::get<2>(poi_name), pose, m_poi_search_radius, true);
					if (!pois.empty())
					{
						POI* poi = pois[0];
						poi_xys.push_back(*poi);
						Polar2 relative = computeRelative(ocrs[k].xmin, ocrs[k].ymin, ocrs[k].xmax, ocrs[k].ymax);
						relatives.push_back(relative);
						poi_confidences.push_back(std::get<1>(poi_name)); //ocrs[k].confidence);
					}
				}
            }
            return true;
        }

		bool applyPreprocessed(std::string recog_name, double xmin, double ymin, double xmax, double ymax, double conf, const dg::Timestamp data_time, dg::Point2& poi_xy, dg::Polar2& relative, double& poi_confidence)
		{
			cv::AutoLock lock(m_localizer_mutex);
			if (m_shared == nullptr) return false;
			Pose2 pose = m_shared->getPose();

			std::vector<OCRResult> ocrs;
			OCRResult ocr;
			ocr.label = recog_name;
			ocr.xmin = (int)(xmin + 0.5);
			ocr.ymin = (int)(ymin + 0.5);
			ocr.xmax = (int)(xmax + 0.5);
			ocr.ymax = (int)(ymax + 0.5);
			ocr.confidence = conf;
			ocrs.push_back(ocr);
			set(ocrs, data_time);

			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return false;
			std::wstring ocr_result = converter.from_bytes(recog_name.c_str());
			std::vector<std::tuple<double, double, std::wstring>> poi_names = matchPOIName(ocr_result, pose, 100.0, 1);
			if (poi_names.empty()) return false;
			std::tuple<double, double, std::wstring> poi_name = poi_names.at(0);
			std::vector<POI*> pois = map->getPOI(std::get<2>(poi_name), pose, m_poi_search_radius, true);

			if (pois.empty()) return false;
			poi_xy = *(pois[0]);
			relative = computeRelative(xmin, ymin, xmax, ymax);
			poi_confidence = std::get<1>(poi_name); //conf;
			return true;
		}

		bool getLocClue(const Pose2& pose, std::vector<dg::Point2>& poi_xys, std::vector<dg::Polar2>& relatives, std::vector<double>& poi_confidences)
		{
			cv::AutoLock lock(m_localizer_mutex);
			std::vector<OCRResult> ocrs = get();
			if (ocrs.empty()) return false;

			if (m_shared == nullptr) return false;
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return false;

			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			for (int k = 0; k < ocrs.size(); k++)
			{
				std::wstring poi_name = converter.from_bytes(ocrs[k].label.c_str());
				std::vector<POI*> pois = map->getPOI(poi_name, pose, m_poi_search_radius, true);
				if (!pois.empty())
				{
					POI* poi = pois[0];
					poi_xys.push_back(*poi);
					Polar2 relative = computeRelative(ocrs[k].xmin, ocrs[k].ymin, ocrs[k].xmax, ocrs[k].ymax);
					relatives.push_back(relative);
					poi_confidences.push_back(ocrs[k].confidence);
				}
			}
			return true;
		}		


    protected:
		// hangeul parameters
		int kor_begin = 44032;
		int kor_end = 55203;
		int chosung_base = 588;
		int jungsung_base = 28;
		int jaum_begin = 12593;
		int jaum_end = 12622;
		int moum_begin = 12623;
		int moum_end = 12643;
		wchar_t chosung_list[19] = { L'ㄱ', L'ㄲ', L'ㄴ', L'ㄷ', L'ㄸ', L'ㄹ', L'ㅁ', L'ㅂ', L'ㅃ', L'ㅅ', L'ㅆ', L'ㅇ' , L'ㅈ', L'ㅉ', L'ㅊ', L'ㅋ', L'ㅌ', L'ㅍ', L'ㅎ' };
		wchar_t jungsung_list[21] = { L'ㅏ', L'ㅐ', L'ㅑ', L'ㅒ', L'ㅓ', L'ㅔ', L'ㅕ', L'ㅖ', L'ㅗ', L'ㅘ', L'ㅙ', L'ㅚ', L'ㅛ', L'ㅜ', L'ㅝ', L'ㅞ', L'ㅟ', L'ㅠ', L'ㅡ', L'ㅢ', L'ㅣ' };
		wchar_t jongsung_list[28] = { L' ', L'ㄱ', L'ㄲ', L'ㄳ', L'ㄴ', L'ㄵ', L'ㄶ', L'ㄷ', L'ㄹ', L'ㄺ', L'ㄻ', L'ㄼ', L'ㄽ', L'ㄾ', L'ㄿ', L'ㅀ', L'ㅁ', L'ㅂ', L'ㅄ', L'ㅅ', L'ㅆ', L'ㅇ', L'ㅈ', L'ㅊ', L'ㅋ', L'ㅌ', L'ㅍ', L'ㅎ' };
		wchar_t jaum_list[30] = { L'ㄱ', L'ㄲ', L'ㄳ', L'ㄴ', L'ㄵ', L'ㄶ', L'ㄷ', L'ㄸ', L'ㄹ', L'ㄺ', L'ㄻ', L'ㄼ', L'ㄽ', L'ㄾ', L'ㄿ', L'ㅀ', L'ㅁ', L'ㅂ', L'ㅃ', L'ㅄ', L'ㅅ', L'ㅆ', L'ㅇ', L'ㅈ', L'ㅉ', L'ㅊ', L'ㅋ', L'ㅌ', L'ㅍ', L'ㅎ' };
		wchar_t moum_list[21] = { L'ㅏ', L'ㅐ', L'ㅑ', L'ㅒ', L'ㅓ', L'ㅔ', L'ㅕ', L'ㅖ', L'ㅗ', L'ㅘ', L'ㅙ', L'ㅚ', L'ㅛ', L'ㅜ', L'ㅝ', L'ㅞ', L'ㅟ', L'ㅠ', L'ㅡ', L'ㅢ', L'ㅣ' };

		/** A hash table for finding similarity weight between characters */
		std::map<std::pair<wchar_t, wchar_t>, double> weights;

		bool character_is_korean(wchar_t c)
		{
			int i = (int)c;
			return ((kor_begin <= i && i <= kor_end) || (jaum_begin <= i && i <= jaum_end) || (moum_begin <= i && i <= moum_end));
		}

		wchar_t compose(wchar_t chosung, wchar_t jungsung, wchar_t jongsung)
		{
			setlocale(LC_ALL, "KOREAN");

			int cho_idx = 0;
			int cho_max = sizeof(chosung_list) / sizeof(wchar_t);
			while (chosung_list[cho_idx] != chosung && cho_idx < cho_max) cho_idx++;

			int jung_idx = 0;
			int jung_max = sizeof(jungsung_list) / sizeof(wchar_t);
			while (jungsung_list[jung_idx] != jungsung && jung_idx < jung_max) jung_idx++;

			int jong_idx = 0;
			int jong_max = sizeof(jongsung_list) / sizeof(wchar_t);
			while (jongsung_list[jong_idx] != jongsung && jong_idx < jong_max) jong_idx++;

			if (cho_idx >= cho_max || jung_idx >= jung_max || jong_idx >= jong_max)
			{
				printf("OCRLocalizer::compose() - invalid input character!\n");
				return 0;
			}

			wchar_t character = (wchar_t)(kor_begin + chosung_base * cho_idx + jungsung_base * jung_idx + jong_idx);

			return character;
		}

		wchar_t* decompose(wchar_t c)
		{
			setlocale(LC_ALL, "KOREAN");

			static wchar_t jamos[4] = {L'\0'};
			if (!character_is_korean(c))
			{
				jamos[0] = {c};				
				if(c == L' ')
				{
					jamos[1] = {L' '};
					jamos[2] = {L' '};
				}
				else
				{
					jamos[1] = {L'\0'};
					jamos[2] = {L'\0'};
				}
				jamos[3] = {L'\0'};
				return jamos;
			}

			int i = (int)c;
			if (jaum_begin <= i && i <= jaum_end)
			{
				jamos[0] = {c};
				jamos[1] = {L' '};
				jamos[2] = {L' '};
				jamos[3] = {L'\0'};
				return jamos;
			}
			if (moum_begin <= i && i <= moum_end)
			{
				jamos[0] = {L' '};
				jamos[1] = {c};				
				jamos[2] = {L' '};
				jamos[3] = {L'\0'};
				return jamos;
			}

			// decomposition rule
			i = i - kor_begin;
			int cho  = i / chosung_base; // chosung_base
			int jung = ( i - cho * chosung_base ) / jungsung_base; // jungsung_base 
			int jong = i - cho * chosung_base - jung * jungsung_base;    
			jamos[0] = {chosung_list[cho]};
			jamos[1] = {jungsung_list[jung]};
			jamos[2] = {jongsung_list[jong]};
			jamos[3] = {L'\0'};

			return jamos;
		}

		double substitution_cost(wchar_t c1, wchar_t c2)
		{
			if (c1 == c2)
				return 0.0;
			
			//std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			std::wstring s1(decompose(c1));
			std::wstring s2(decompose(c2));
			return levenshtein(s1, s2)/3;
		}

		double levenshtein_jamo(std::wstring s1, std::wstring s2)
		{
			if(s1.length() < s2.length())
				return levenshtein_jamo(s2, s1);

			if(s2.length() == 0)
				return s1.length();

			std::vector<double> previous_row(s2.length() + 1);
			for(int i = 0; i < int(sizeof(previous_row)/sizeof(double)); i++)
				previous_row[i] = i;

			for(int i = 0; i < s1.length(); i++)
			{
				std::vector<double> current_row = {double(i) + 1};
				for(int j = 0; j < s2.length(); j++)
				{
					double insertions = previous_row[j + 1] + 1;
					double deletions = current_row[j] + 1;
					double substitutions = previous_row[j] + substitution_cost(s1[i], s2[j]);
					current_row.push_back(std::min({insertions, deletions, substitutions}));
				}

				previous_row = current_row;
			}

			return previous_row.back();
		}
		
		bool addWeight(const wchar_t c1, const wchar_t c2, const double similarity)
		{
			std::pair pair = std::make_pair(c1, c2);
			//weights[pair] = similarity;
			weights.insert(std::make_pair(pair, similarity));

			return true;
		}
		
		double weight_similarity(const wchar_t c1, const wchar_t c2)
		{
			if (c1 == c2)
				return 1.0;

			std::pair pair = std::make_pair(c1, c2);
			//double weightd = weights[pair];
			int count = weights.count(pair);
			auto weight = weights.find(pair);
			if (weight != weights.end())
				return weight->second;
			else
				return 0.0;
		}

		double levenshtein(std::wstring s1, std::wstring s2)
		{
			if(s1.length() < s2.length())
				return levenshtein(s2, s1);

			if(s2.length() == 0)
				return s1.length();

			std::vector<double> previous_row(s2.length() + 1);
			for(int i = 0; i < int(sizeof(previous_row)/sizeof(double)); i++)
				previous_row[i] = i;

			for(int i = 0; i < s1.length(); i++)
			{
				std::vector<double> current_row = {double(i) + 1};
				for(int j = 0; j < s2.length(); j++)
				{
					double insertions = previous_row[j + 1] + 1;
					double deletions = current_row[j] + 1;
					double substitutions = previous_row[j] + (1 - weight_similarity(s1[i], s2[j]));//(s1[i] != s2[j]);
					current_row.push_back(std::min({insertions, deletions, substitutions}));
				}

				previous_row = current_row;
			}

			return previous_row.back();
		}

		/**
		 * Find Top-n POI names closest to the result of OCR
		 * @param poi_names POI names within a search radius from a point
		 * @param ocr_result A result of OCR
		 * @param num_neighbors The number of POI names to return
		 * @param jamo_mode A given levenshtein function mode
		 * @return A list of distance, confidence, and name of matched POIs (empty list if no POI found)
		 */
		std::vector<std::tuple<double, double, std::wstring>> getNeighbors(std::vector<std::wstring> poi_names, std::wstring ocr_result, int num_neighbors = 1, bool jamo_mode = true)
		{
			std::vector<std::tuple<double, double, std::wstring>> distances;
			int size = poi_names.size();
			for(int i = 0; i < poi_names.size(); i++)
			{
				double dist = 0.0;
				if(jamo_mode == true)
					dist = levenshtein_jamo(poi_names[i], ocr_result);
				else
					dist = levenshtein(poi_names[i], ocr_result);
				double dist_conf = 1.0 - (dist / std::max({ocr_result.length(), poi_names[i].length()}));
				double thre = 0.8; //0.6;
				if(dist_conf >= thre)
					distances.push_back(std::make_tuple(dist, dist_conf, poi_names[i]));
			}
			if(distances.size() > 0)
			{
				std::sort(distances.begin(), distances.end(), compare_dist);

				std::vector<std::tuple<double, double, std::wstring>> neighbors;
				for(int i = 0; i < num_neighbors; i++)
				{		
					neighbors.push_back(distances[i]);
				}

				return neighbors;
			}

			return std::vector<std::tuple<double, double, std::wstring>>();
		}
		
		/**
		 * Find POI names matched with the result of OCR
		 * @param ocr_result A result of OCR
		 * @param p A given point
		 * @param search_radius A given search radius
		 * @param num_neighbors The number of POI names to return
		 * @param jamo_mode A given levenshtein function mode
		 * @return A list of distance, confidence, and name of matched POIs (empty list if no POI found)
		 */
		std::vector<std::tuple<double, double, std::wstring>> matchPOIName(std::wstring ocr_result, const Point2& p, double search_radius = 100.0, int num_neighbors = 1, bool jamo_mode = true)		
		{
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return std::vector<std::tuple<double, double, std::wstring>>();
			std::vector<std::wstring> poi_names = map->getNearPOINames(p, search_radius);
			std::vector<std::tuple<double, double, std::wstring>> results = getNeighbors(poi_names, ocr_result, num_neighbors);

			return results;
		}

        dg::Polar2 computeRelative(double x1, double y1, double x2, double y2)
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

#endif // End of '__OCR_LOCALIZER__'
