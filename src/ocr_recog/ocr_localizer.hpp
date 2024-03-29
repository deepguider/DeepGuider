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
	// <ocr_index, poi_index, levenshtein_distance, match_score>
	bool compare_score(std::tuple<int, POI*, double, double> a, std::tuple<int, POI*, double, double> b)
	{
		if(std::get<3>(a) == std::get<3>(b)) return std::get<2>(a) < std::get<2>(b);

		return std::get<3>(a) > std::get<3>(b);
	}

    /**
    * @brief OCR Localizer
    */
    class OCRLocalizer : public OCRRecognizer, public cx::Algorithm
    {
	protected:
		// camera parameters
		int m_img_w = 1280;
		int m_img_h = 720;
		double m_hfov = 82.1;	            // Horizontal fov angle of the camera, Unit: [deg]
		double m_camera_height = 0.83;		// Height of camera from ground plane, Unit: [m]
		double m_vanishing_y = 0.583;		// normalized Y coordinate of image vanishing line (0 ~ 1)

		// configuable parameters
		double m_poi_height = 3.8;			// Height of POI from ground plane, Unit: [m]
		double m_poi_search_radius = 50;	// POI search range, Unit: [m]
		bool m_check_jungsung_type = true;	// distinguish bottom-side jungsung and right-side jungsung
		bool m_fixed_template_match = true;	// use substring template match instead of Levenshtein distance
		bool m_enable_false_filter = true;	// filter out detections detected under vanishing line
	    bool m_enable_debugging_display = false;

		double m_poi_match_thresh = 2.5;			// POI matching threshold (ratio_score + length_score)
		double m_best_to_second_match_ratio = 1.5;	// POI match ratio threshold (best_score/second_score > ratio threshod)
		double m_poi_match_ratio = 0.5;				// mimimum ratio of matched part for korean poi
		double m_poi_match_ratio_alphabet = 0.7;	// mimimum ratio of matched part for alphanumeric poi
		int m_poi_match_length_normalizer = 2; 			// minimum matched string length for korean poi
		int m_poi_match_length_normalizer_alphabet = 4;	// minimum matched string length for alphanumeric poi

        /** Read parameters from cv::FileNode - Inherited from cx::Algorithm */
        virtual int readParam(const cv::FileNode& fn)
        {
            int n_read = 0;
            CX_LOAD_PARAM_COUNT(fn, "poi_height", m_poi_height, n_read);
            CX_LOAD_PARAM_COUNT(fn, "poi_search_radius", m_poi_search_radius, n_read);
            CX_LOAD_PARAM_COUNT(fn, "poi_match_thresh", m_poi_match_thresh, n_read);
			CX_LOAD_PARAM_COUNT(fn, "enable_false_filter", m_enable_false_filter, n_read);
			CX_LOAD_PARAM_COUNT(fn, "enable_debugging_display", m_enable_debugging_display, n_read);

            return n_read;
        }
	
    public:
		OCRLocalizer()
		{
			if (!loadHangeulWeights("data/dg_hangeul_weights.csv"))
			{
				printf("[ocr_localizer] fail to load weights file\n");
			}
		}

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

		void setParam(double hfov, double cam_h, double vanishing_y, double poi_h = 3.8, double search_radius = 50)
		{
			cv::AutoLock lock(m_localizer_mutex);
			m_hfov = hfov;
			m_vanishing_y = vanishing_y;
			m_camera_height = cam_h;
			m_poi_height = poi_h;
			m_poi_search_radius = search_radius;
		}
		
		/**
		 * @return True if there is any detection, False otherwise.
		 */
        bool apply(const cv::Mat image, const dg::Timestamp image_time, std::vector<dg::POI*>& pois, std::vector<dg::Polar2>& relatives, std::vector<double>& poi_confidences)
        {
			// update image size
			m_img_w = image.cols;
			m_img_h = image.rows;

			// OCR detection (vision module)
			if (!OCRRecognizer::apply(image, image_time)) return false;
			cv::AutoLock lock(m_localizer_mutex);
			std::vector<OCRResult> ocrs = get();
			if (ocrs.empty()) return false;

			int vanishing_py = (int)(m_vanishing_y * image.rows + 0.5);

			// filter out non-POI detections
			if (m_enable_false_filter)
			{
				std::vector<OCRResult> valid_ocrs;
				for (auto it = ocrs.begin(); it != ocrs.end(); it++)
				{
					bool valid = true;
					if (it->ymax > vanishing_py) valid = false;								// ground POI
					if (valid && (it->ymax - it->ymin > it->xmax - it->xmin)) valid = false;	// vertical POI
					if (valid && it->label.length() <= 1) valid = false;						// 1-character POI
					//if (valid && isNumeric(it->label)) valid = false;					        // numeric POI
					if (valid) valid_ocrs.push_back(*it);
				}
				if (valid_ocrs.empty()) return false;
				ocrs = valid_ocrs;
			}

			// retrieve nearby POIs from the server
            if (m_shared == nullptr) return false;
			Pose2 pose = m_shared->getPose();
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return false;
			std::vector<dg::POI*> pois_near_all = map->getNearPOIs(pose, m_poi_search_radius);
			if(pois_near_all.empty()) return false;
			std::vector<dg::POI*> pois_near;
			for(int i = 0; i<(int)pois_near_all.size(); i++)
			{
				if(pois_near_all[i]->floor < 100) continue;
				pois_near.push_back(pois_near_all[i]);
			}
			/*
	        std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			for(auto itr=pois_near.begin(); itr!=pois_near.end(); itr++)
			{
				string tmp = converter.to_bytes((*itr)->name);
				printf("near pois = %s\n", tmp.c_str());
			}
			*/

			// match OCR detections with nearby POIs: <ocr_index, POI*, levenshtein_distance, match_score>
			m_match_ocrs = ocrs;
			m_matches = matchPOI(ocrs, pois_near);
			if(m_matches.empty()) return false;

			// estimate relative pose of the matched POIs
			for (size_t k = 0; k < m_matches.size(); k++)
            {
				int ocr_idx = std::get<0>(m_matches[k]);
				POI* poi = std::get<1>(m_matches[k]);
				double match_score = std::get<3>(m_matches[k]);
				pois.push_back(poi);
				Polar2 relative = computeRelative(image.cols, image.rows, ocrs[ocr_idx].xmin, ocrs[ocr_idx].ymin, ocrs[ocr_idx].xmax, ocrs[ocr_idx].ymax, poi->floor);
				relatives.push_back(relative);
				double conf = match_score * ocrs[ocr_idx].confidence;
				poi_confidences.push_back(conf);
				/*
				string tmp = converter.to_bytes(poi->name);
				printf("matched pois = %s\n", tmp.c_str());
				*/

            }
            return true;
        }

		bool applyPreprocessed(std::string recog_name, double xmin, double ymin, double xmax, double ymax, double conf, const dg::Timestamp data_time, dg::POI*& poi, dg::Polar2& relative, double& poi_confidence)
		{
			if (m_enable_false_filter && (ymax > m_vanishing_y || ymax - ymin > xmax - xmin || recog_name.length() <= 1 || isNumeric(recog_name))) return false;

			std::vector<OCRResult> ocrs;
			OCRResult ocr;
			//ocr.label = cx::toLowerCase(recog_name); // convert to lowercase
			ocr.label = recog_name;
			ocr.xmin = (int)(xmin + 0.5);
			ocr.ymin = (int)(ymin + 0.5);
			ocr.xmax = (int)(xmax + 0.5);
			ocr.ymax = (int)(ymax + 0.5);
			ocr.confidence = conf;
			ocrs.push_back(ocr);
			set(ocrs, data_time);

			// retrieve nearby POIs from the server
			cv::AutoLock lock(m_localizer_mutex);
            if (m_shared == nullptr) return false;
			Pose2 pose = m_shared->getPose();
			Map* map = m_shared->getMap();
			if (map == nullptr || map->isEmpty()) return false;
			std::vector<dg::POI*> pois = map->getNearPOIs(pose, m_poi_search_radius);
			if(pois.empty()) return false;

			// match OCR detections with nearby POIs: <ocr_index, poi_index, levenshtein_distance, match_score>
			m_match_ocrs = ocrs;
			m_matches = matchPOI(ocrs, pois);
			if(m_matches.empty()) return false;

			// estimate relative pose of the matched POIs
			int ocr_idx = std::get<0>(m_matches[0]);
			poi = std::get<1>(m_matches[0]);
			relative = computeRelative(m_img_w, m_img_h, xmin, ymin, xmax, ymax, poi->floor);
			double match_score = std::get<3>(m_matches[0]);
			poi_confidence = match_score * conf;
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
					Polar2 relative = computeRelative(m_img_w, m_img_h, ocrs[k].xmin, ocrs[k].ymin, ocrs[k].xmax, ocrs[k].ymax, poi->floor);
					relatives.push_back(relative);
					poi_confidences.push_back(ocrs[k].confidence);
				}
			}
			return true;
		}

	    void clear()
		{
			OCRRecognizer::clear();
		}

		bool loadHangeulWeights(const char* filename = "dg_hangeul_weights.csv")
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
				if (token[0] == 'P' || token[0] == 'p')
				{
					wchar_t index = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					if (index == '1') chosung_list = cx::trimBoth(token);
					else if (index == '2') jungsung_list = cx::trimBoth(token);
					else if (index == '3')
					{
						jongsung_list = cx::trimBoth(token);
						jongsung_list.insert(jongsung_list.begin(), L' ');
					}
					else if (index == '4') jaum_list = cx::trimBoth(token);
					else if (index == '5') moum_list = cx::trimBoth(token);
					else goto WEIGHT_FAIL;
				}
				if (token[0] == 'T' || token[0] == 't')
				{
					wchar_t index = token[1];
					if ((token = wcstok(nullptr, L",", &context)) == nullptr) goto WEIGHT_FAIL;
					if (index == '1') jungsung_type_list = cx::trimBoth(token);
					else goto WEIGHT_FAIL;
				}
				else if (token[0] == 'J' || token[0] == 'j')
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

		void printHangeulWeights()
		{
			setlocale(LC_ALL, "");

			// Korean Phoneme Table
			wprintf(L"Korean Phoneme Table\n");
			wprintf(L"\tchosung: %ls\n", chosung_list.c_str());
			wprintf(L"\tjungsung: %ls\n", jungsung_list.c_str());
			wprintf(L"\tjongsung: %ls\n", jongsung_list.c_str());
			wprintf(L"\tjaum: %ls\n", jaum_list.c_str());
			wprintf(L"\tmoum: %ls\n", moum_list.c_str());

			// Phoneme Type Table
			wprintf(L"Phoneme Type Table\n");
			wprintf(L"\tjungsung: %ls\n", jungsung_type_list.c_str());

			// Phoneme Similiraty Table
			wprintf(L"Phoneme Similarity Table\n");
			for(auto it = weights.begin(); it != weights.end(); it++)
			{
				wchar_t c1 = it->first.first;
				wchar_t c2 = it->first.second;
				double similarity = it->second;
				wprintf(L"\t(%lc - %lc) = %lf\n", c1, c2, similarity);
			}
		}

        void print() const
        {
			OCRRecognizer::print();

	        std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
            cv::AutoLock lock(m_mutex);
#ifdef _WIN32			
			for (size_t k = 0; k < m_matches.size(); k++)
			{
				int ocr_idx = std::get<0>(m_matches[k]);
				std::wstring ocr_name = converter.from_bytes(m_match_ocrs[ocr_idx].label);
				std::wstring poi_name = std::get<1>(m_matches[k])->name;
				double leven_dist = std::get<2>(m_matches[k]);
				double match_score = std::get<3>(m_matches[k]);
				wprintf(L"\t\b*%ls - %ls: dist = %.2lf, score = %.2lf\n", ocr_name.c_str(), poi_name.c_str(), leven_dist, match_score);
			}
			if(m_enable_debugging_display)
			{
				for(size_t k = 0; k < m_candidates.size(); k++)
				{
					int ocr_idx = std::get<0>(m_candidates[k]);
					std::wstring ocr_name = converter.from_bytes(m_match_ocrs[ocr_idx].label);
					std::wstring poi_name = std::get<1>(m_candidates[k])->name;
					double leven_dist = std::get<2>(m_candidates[k]);
					double match_score = std::get<3>(m_candidates[k]);
					wprintf(L"\t>%ls - %ls: dist = %.2lf, score = %.2lf\n", ocr_name.c_str(), poi_name.c_str(), leven_dist, match_score);
				}
			}
#else
			for (size_t k = 0; k < m_matches.size(); k++)
			{
				int ocr_idx = std::get<0>(m_matches[k]);
				std::string poi_name = converter.to_bytes(std::get<1>(m_matches[k])->name);
				double leven_dist = std::get<2>(m_matches[k]);
				double match_score = std::get<3>(m_matches[k]);
				printf("\t\b*%s - %s: dist = %.2lf, score = %.2lf\n", m_match_ocrs[ocr_idx].label.c_str(), poi_name.c_str(), leven_dist, match_score);
			}
			if(m_enable_debugging_display)
			{
				for(size_t k = 0; k < m_candidates.size(); k++)
				{
					int ocr_idx = std::get<0>(m_candidates[k]);
					std::string poi_name = converter.to_bytes(std::get<1>(m_candidates[k])->name);
					double leven_dist = std::get<2>(m_candidates[k]);
					double match_score = std::get<3>(m_candidates[k]);
					printf("\t>%s - %s: dist = %.2lf, score = %.2lf\n", m_match_ocrs[ocr_idx].label.c_str(), poi_name.c_str(), leven_dist, match_score);
				}
			}
#endif
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
		std::wstring chosung_list;
		std::wstring jungsung_list;
		std::wstring jongsung_list;
		std::wstring jaum_list;
		std::wstring moum_list;
		std::wstring jungsung_type_list;	// r: right, b: bottom, m: mixed(bottom-right)

		/** A hash table for finding similarity weight between characters */
		std::map<std::pair<wchar_t, wchar_t>, double> weights;
		std::vector<std::tuple<int, POI*, double, double>> m_matches;    // <ocr_index, POI*, levenshtein_distance, match_score>
		std::vector<std::tuple<int, POI*, double, double>> m_candidates; // <ocr_index, POI*, levenshtein_distance, match_score>
        std::vector<OCRResult> m_match_ocrs;

		bool character_is_korean(wchar_t c)
		{
			int i = (int)c;
			return ((kor_begin <= i && i <= kor_end) || (jaum_begin <= i && i <= jaum_end) || (moum_begin <= i && i <= moum_end));
		}

		bool isAlphanumeric(std::string str)
        {
            for(int i = 0; i < str.length(); i++)
            {
				char c = str[i];
				if(!('A' <= c && c <= 'Z' ) && !('a' <= c && c <= 'z' ) && !('0' <= c && c <= '9' )) return false;
			}

            return true;
        }

		bool isNumeric(std::string str)
        {
            for(int i = 0; i < str.length(); i++)
            {
				char c = str[i];
				if(!('0' <= c && c <= '9' )) return false;
			}

            return true;
        }

		wchar_t compose(wchar_t chosung, wchar_t jungsung, wchar_t jongsung)
		{
			setlocale(LC_ALL, "KOREAN");

			int cho_idx = 0;
			int cho_max = (int)chosung_list.length();
			while (cho_idx < cho_max && chosung_list[cho_idx] != chosung) cho_idx++;

			int jung_idx = 0;
			int jung_max = (int)jungsung_list.length();
			while (jung_idx < jung_max && jungsung_list[jung_idx] != jungsung) jung_idx++;

			int jong_idx = 0;
			int jong_max = (int)jongsung_list.length();
			while (jong_idx < jong_max && jongsung_list[jong_idx] != jongsung) jong_idx++;

			if (cho_idx >= cho_max || jung_idx >= jung_max || jong_idx >= jong_max)
			{
				printf("OCRLocalizer::compose() - invalid input character!\n");
				return 0;
			}

			wchar_t character = (wchar_t)(kor_begin + chosung_base * cho_idx + jungsung_base * jung_idx + jong_idx);

			return character;
		}

		wchar_t* decompose(wchar_t c, wchar_t* jungsung_type = nullptr)
		{
			setlocale(LC_ALL, "KOREAN");

			static wchar_t jamos[4] = {L'\0'};
			if (!character_is_korean(c))
			{
				jamos[0] = {c};				
				jamos[1] = {L'\0'};
				return jamos;
			}

			int i = (int)c;
			if (jaum_begin <= i && i <= jaum_end)
			{
				jamos[0] = {c};
				jamos[1] = {L'\0'};
				return jamos;
			}
			if (moum_begin <= i && i <= moum_end)
			{
				jamos[0] = {c};
				jamos[1] = {L'\0'};
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
			if(jamos[2] == L' ') jamos[2] = {L'\0'};
			if(jungsung_type != nullptr) *jungsung_type = jungsung_type_list[jung];

			return jamos;
		}

		double substitution_cost(wchar_t c1, wchar_t c2)
		{
			if (c1 == c2)
				return 0.0;
			
			wchar_t type1 = 0;
			wchar_t type2 = 0;
			std::wstring s1(decompose(c1, &type1));
			std::wstring s2(decompose(c2, &type2));
			if(m_check_jungsung_type && type1!=type2) return 1;
			
			return levenshtein(s1, s2)/std::max({s1.length(), s2.length()});
		}

		double levenshtein_jamo(std::wstring s1, std::wstring s2)
		{
			if(s1.length() < s2.length())
				return levenshtein_jamo(s2, s1);

			if(s2.length() == 0)
				return (double)s1.length();

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
			auto pair = std::make_pair(c1, c2);
			auto weight = weights.find(pair);
			if (weight != weights.end())
				weights.erase(pair);
			weights.insert(std::make_pair(pair, similarity));

			return true;
		}
		
		double weight_similarity(const wchar_t c1, const wchar_t c2)
		{
			if (c1 == c2) return 1.0;

			auto pair = std::make_pair(c1, c2);
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
				return (double)s1.length();

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

		double fixed_substitution_cost(wchar_t c1, wchar_t c2)
		{
			if (c1 == c2) return 0.0;
			
			wchar_t type1 = 0;
			wchar_t type2 = 0;
			std::wstring s1(decompose(c1, &type1));
			std::wstring s2(decompose(c2, &type2));
			if(m_check_jungsung_type && type1!=type2) return 1;

			int n_min = (s1.length() < s2.length()) ? (int)s1.length() : (int)s2.length();
			int n_max = (s1.length() > s2.length()) ? (int)s1.length() : (int)s2.length();
			double cost = 0;
			for(int i = 0; i < n_min; i++)
			{
				if(s1[i] == s2[i]) continue;
				cost += (1 - weight_similarity(s1[i], s2[i]));
			}
			cost += (n_max - n_min);

			return cost/n_max;
		}

		double fixed_levenshtein_jamo(std::wstring s1, std::wstring s2)
		{
			if(s1.length() < s2.length())
				return fixed_levenshtein_jamo(s2, s1);

			if(s2.length() == 0)
				return (double)s1.length();

			double min_cost = DBL_MAX;
			for(int i = 0; i <= s1.length() - s2.length(); i++)
			{
				double cost = 0.0;
				for(int j = 0; j < s2.length(); j++)
				{
					cost += fixed_substitution_cost(s1[i + j], s2[j]);
				}
				cost += (s1.length() - s2.length());

				if(cost < min_cost)
				{
					min_cost = cost;
				}
			}
			return min_cost;
		}

		/**
		 * Find POIs matched with the result of OCR
		 * @param ocrs A list of OCR detections
		 * @param pois A list of POIs to be matched
		 * @param jamo_mode A given levenshtein function mode
		 * @return A list of matches <OCR index, POI index, Levenshtein_distance, match_score> (empty list if no POI matched)
		 */
		std::vector<std::tuple<int, POI*, double, double>> matchPOI(const std::vector<OCRResult>& ocrs, const std::vector<dg::POI*>& pois, bool jamo_mode = true)
		{
			// preprocessing: remove spaces from POI names
			std::vector<std::wstring> poi_names;
			for (int k = 0; k < (int)pois.size(); k++)
            {
				std::wstring poi_name = pois[k]->name;
				poi_name.erase(std::remove(poi_name.begin(), poi_name.end(), ' '), poi_name.end()); // remove spaces
				poi_names.push_back(poi_name);
			}

			// match <ocr_index, POI*, levenshtein_distance, match_score>
			std::vector<std::tuple<int, POI*, double, double>> matches;
	        std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			m_candidates.clear();
			for (int i = 0; i < (int)ocrs.size(); i++)
            {
                std::wstring ocr_result = converter.from_bytes(ocrs[i].label.c_str());
				ocr_result.erase(std::remove(ocr_result.begin(), ocr_result.end(), ' '), ocr_result.end()); // remove spaces

				std::vector<std::tuple<int, POI*, double, double>> candidates;
				double best_score = -1;
				double second_best_score = -1;
				std::tuple<int, POI*, double, double> best;
				std::tuple<int, POI*, double, double> second_best;

				bool alphanumeric = isAlphanumeric(ocrs[i].label);
				size_t ocr_length = ocr_result.length();
				for(int j = 0; j < poi_names.size(); j++)
				{
					// reject detections longer than poi name
					if(poi_names[j].length() < ocr_length) continue;

					double leven_dist = 0.0;
					if(m_fixed_template_match)
						leven_dist = fixed_levenshtein_jamo(poi_names[j], ocr_result);
					else if(jamo_mode == true)
						leven_dist = levenshtein_jamo(poi_names[j], ocr_result);
					else
						leven_dist = levenshtein(poi_names[j], ocr_result);
					
					double matched_length = std::max({ocr_length, poi_names[j].length()}) - leven_dist;
					double matched_ratio1 = matched_length / ocr_length;
					double matched_ratio2 = matched_length / poi_names[j].length();
					double ratio_score = matched_ratio2;
					int length_normalizer = (alphanumeric) ? m_poi_match_length_normalizer_alphabet : m_poi_match_length_normalizer;
					double length_score = (matched_length - (ocr_length - matched_length)) / length_normalizer;
					double match_score = 2 * ratio_score + length_score;

					if (match_score > best_score)
					{
						second_best_score = best_score;
						second_best = best;
						best_score = match_score;
						best = std::make_tuple(i, pois[j], leven_dist, match_score);
					}
					else if (match_score > second_best_score)
					{
						second_best_score = match_score;
						second_best = std::make_tuple(i, pois[j], leven_dist, match_score);
					}

					double ratio_thr = (alphanumeric) ? m_poi_match_ratio_alphabet : m_poi_match_ratio;
					if(match_score >= m_poi_match_thresh && matched_ratio1 >= ratio_thr)
						candidates.push_back(std::make_tuple(i, pois[j], leven_dist, match_score));
				}
				if(candidates.size() > 0)
				{
					if (m_enable_debugging_display)
					{
						std::sort(candidates.begin(), candidates.end(), compare_score);
						m_candidates.insert(m_candidates.begin() + (int)m_candidates.size(), candidates.begin(), candidates.end());
					}

					double ratio_score = (second_best_score >= 0) ? best_score / second_best_score : m_best_to_second_match_ratio;
					if (ratio_score >= m_best_to_second_match_ratio)
					{
						matches.push_back(best);
					}
				}
            }

			return matches;
		}

        dg::Polar2 computeRelative(int img_w, int img_h, double x1, double y1, double x2, double y2, int floor)
        {
            dg::Polar2 relative = dg::Polar2(-1, CV_PI);

			// camera parameters
			double focal_length = (img_w / 2.0) / tan((m_hfov * CV_PI / 180) / 2);
			double cx = img_w/2;
			double cy = img_h/2;
			int vanishing_py = (int)(m_vanishing_y * img_w + 0.5);

			double camera_tilt = atan((vanishing_py - cy) / focal_length);	// camera tilt
			double cam_pan = CV_PI / 2;

			// R|t matrix
			cv::Matx33d R = rotationFromPanTiltRoll(cam_pan, camera_tilt, 0);
			cv::Matx31d t = -R * cv::Matx31d(0, 0, m_camera_height);

			// poi center
			double poi_tx = (x1 + x2) / 2.0;
			double poi_ty = (y1 + y2) / 2.0;

			// camera coordinate of poi center
			cv::Mat pc(3, 1, CV_64FC1);
			pc.at<double>(0) = (poi_tx - cx) / focal_length;
			pc.at<double>(1) = (poi_ty - cy) / focal_length;
			pc.at<double>(2) = 1;

			// world direction of poi ray (pc = R(pw - cw) --> pw - cw = R^Tpc)
			cv::Mat v = cv::Mat(R.t()) * pc;
			double vx = v.at<double>(0);
			double vy = v.at<double>(1);
			double vz = v.at<double>(2);

			// pan, tilt of poi center
			double poi_pan = atan2(vy, vx);
			double poi_tilt = atan2(vz, sqrt(vx * vx + vy * vy));

			// ground distance from camera to poi
			if (floor <= 0) floor = 1;
			double poi_height = m_poi_height * floor;
			if (floor>100) poi_height = (double)floor/1000.0;	// convert mm to meter
			double D = (poi_height - m_camera_height) / tan(poi_tilt);

			// result
			relative.lin = D;
			relative.ang = poi_pan - CV_PI/2;
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
