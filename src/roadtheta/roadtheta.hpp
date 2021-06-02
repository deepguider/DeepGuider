#ifndef __ROAD_THETA__
#define __ROAD_THETA__

#include "dg_core.hpp"
#include <fstream>
#include <chrono>


using namespace std;

namespace dg
{
    /**
    * @brief RoadTheta Output
    */
    struct RoadThetaResult
    {
        double theta;            // road direction in radian
        double vx, vy;           // vanishing point coordinate in pixels
        double score = -1;       // vanishing point supporting score
        double confidence = -1;  // 0 ~ 1
        bool valid = false;
    };
    
    /**
    * @brief RoadTheta Parameters
    */
    struct RoadThetaParam
    {
        int reference_image_width = 640;    // default image size(width) to normalize, unit: pixels
        double image_scale_modifier = 1;    // additional scale modifier. final image size = reference_image_size * image_scale_modifier

        int detect_method = 1;              // 0: allsac, 1: voting degree, 2: allpos
        int frame_skip = 0;
        int record_avi = 0;                 // 0: none, 1: result('r'), 2: raw('R')
        int recording_fps = 30;
        int display_delay = 1;
        bool show_fps = true;
        bool test_line_detector = false;
        bool test_grouping = false;
        bool compare_with_reference = false;
        bool show_line_image = false;

        // camera parameter check
        double camera_vanishing_y = 0.583;   // ratio w.r.t. image height
        bool check_camera_geometry = true;
        double hfov = 82.1;     // horizontal field of view angle of camera (unit: degree), logitech c930e
        int pan_range[2] = { -80, 80 };
        int tilt_range[2] = { -40, 90 };     // if tilt_range[1] < tilt_range[0], valid max is used
        bool use_vy_range = true;            // use normalized vy range
        double vy_range[2] = { 0.3, 0.8 };   // if vy_range[1] < vy_range[0], valid max is used

        // validation
        bool apply_validation = true;
        double min_vy = 0.47;               // ratio w.r.t. image height
        double max_vy = 0.694;
        double min_vx = 0;
        double max_vx = 1;

        // grouping
        bool connect_segmented_lines = true;
        double connect_dist_thr = 3;        // pixels (두 직선 사이의 수직 거리)
        double max_connect_gap = 20;        // pixels
        double min_connect_gap = -5;        // pixels
        bool group_duplicated_lines = false;
        double duplication_ang_thr = 135;   // degree
        double duplication_dist_thr = 10;   // pixels

        // filtering
        bool use_line_weight = true;
        int line_length_min = 20;
        bool filter_vert_lines = true;      // filter out vertical line segments
        double vert_line_upper_bound = 0.47;  // ratio w.r.t. image height
        bool filter_horz_lines = true;      // filter out horizontal line segments
        double filter_vert_deg = 80;
        double filter_horz_deg = 5;
        bool apply_position_filter = true;      // gaussian filter

        bool apply_tracking_filter = false;      // gaussian filter
        double tracking_filter_sx = 320;
        double tracking_filter_sy = 180;

        // check valid line
        bool check_vanishing_condition = true;  // 라인 세그먼트가 해당 소실점으로 수렴하는지 체크
        double max_vanishing_error = 10;        // pixels
        bool check_horizontal_distance = true;  // 직선과의 수평거리를 이용하여 vanishing 에러 체크
        double horizontal_distance_thr = 20;
        bool check_ground_condition = true;     // 소실점 위쪽의 라인들은 소실점 산출에서 제외할지 여부
        int max_upperline_deg = 70;

        // ransac parameters
        bool check_sampled_model = true;
        int max_itr = 2000;
        double inlier_thr = 10;     // perpendicular pixel distance
        int lscore_method = 2;      // 0: count, 1: length, 2: length^1.5, 3: lenth^2
        bool use_weighted_lscore = false;   // 소실점과 거리에 따른 패털티 적용 여부
        int refinement_method = 2;          // 0: none, 1: ls, 2: weighted ls, 3: m-estimator, 4: weighted m-estimator
        int weight_lscore_method = 1;       // 0: count, 1: length, 2: length^1.5, 3: lenth^2
        int refine_iteration_n = 1;
        bool show_ransac_vpoint = true;

        // voting method parameters (degree)
        int theta_res_per_degree_horz = 2;        // degree resolution 
        int theta_res_per_degree_vert = 4;        // degree resolution 
        bool mle_check_peak = true;
        bool print_peak_score = false;
        double min_peak_score = 700;
        bool eval_peak_ransac = true;

        // evaluation
        bool show_score = true;
        bool check_score = false;
        double score_thr = 90;

        // drawing parameters
        cv::Scalar color_line_basic = cv::Scalar(0, 255, 255);
        cv::Scalar color_line_vert = cv::Scalar(0, 0, 255);
        cv::Scalar color_line_horz = cv::Scalar(255, 0, 0);
        cv::Scalar color_line_inlier = cv::Scalar(0, 255, 0);
        cv::Scalar color_vpoint_ransac = cv::Scalar(0, 255, 255);
        cv::Scalar color_vpoint = cv::Scalar(0, 0, 255);
        cv::Scalar color_vpoint_wls = cv::Scalar(255, 255, 0);
        cv::Scalar color_vpoint_false = cv::Scalar(128, 128, 128);
        int vpoint_radius = 4;
    };

    /**
    * @brief RoadTheta Class
    */
    class RoadTheta
    {
    public:
        /**
        * Initialize the module
        * @return true if successful (false if failed)
        */
        bool initialize()
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            // initialization code here
            init_position_filter();

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            m_processing_time = t2 - t1;

            return true;
        }

        /**
        * Reset variables and clear the memory
        */
        void clear()
        {
            m_timestamp = -1;
            m_processing_time = -1;
        }

        /**
        * Run once the module for a given input (support thread run)
        * @return evaluation score if successful (-1 if failed)
        */
        bool apply(cv::Mat& image, dg::Timestamp ts)
        {
            dg::Timestamp t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;

            bool ret = _apply(image, ts);

            dg::Timestamp t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
            
            m_processing_time = t2 - t1;
            m_timestamp = ts;

            return ret;
        }

        void get(RoadThetaResult& result)
        {
            result = m_result;
        }

        void get(RoadThetaResult& result, Timestamp& ts)
        {
            result = m_result;
            ts = m_timestamp;
        }

        void set(const RoadThetaResult& result, Timestamp ts, double proc_time)
        {
            m_result = result;
            m_timestamp = ts;
            m_processing_time = proc_time;
        }

        dg::Timestamp timestamp() const
        {
            return m_timestamp;
        }

        double procTime() const
        {
            return m_processing_time;
        }

        void draw(cv::Mat& image) const;

        void print() const
        {
            printf("[%s] proctime = %.3lf, timestamp = %.3lf\n", name(), procTime(), m_timestamp);
            double theta_deg = m_result.theta * 180 / CV_PI;
            printf("\ttheta = %.1lf, confidence = %.2lf, vx = %.1lf, vy = %.1lf, score = %.1lf\n", theta_deg, m_result.confidence, m_result.vx, m_result.vy, m_result.score);
        }

        void write(std::ofstream& stream, int cam_fnumber = -1) const
        {
            std::string log = cv::format("%.3lf,%d,%s,%lf,%.2lf,%.1lf,%.1lf,%.1lf,%.3lf", m_timestamp, cam_fnumber, name(), m_result.theta, m_result.confidence, m_result.vx, m_result.vy, m_result.score, m_processing_time);
            stream << log << std::endl;
        }

        static const char* name()
        {
            return "roadtheta";
        }

        RoadThetaParam param() const
        {
            return m_param;
        }

        void set_param(RoadThetaParam param)
        {
            m_param = param;
        }


    protected:
        RoadThetaParam m_param;
        RoadThetaResult m_result;
        Timestamp m_timestamp = -1;
        double m_processing_time = -1;

        // detection result on scaled image
        int m_scaled_w;
        int m_scaled_h;
        double m_scaled_vx;
        double m_scaled_vy;
        std::vector<cv::Vec4f> m_lines;
        cv::Mat m_scoremap;
        std::vector<cv::Point2d> m_peaks;
        std::vector<double> m_peaks_score;
        int m_max_peak = -1;
        double m_score = -1;
        std::vector<double> m_peaks_score_ransac;
        int m_max_peak_ransac = -1;
        double m_score_ransac = -1;

        // filter
        cv::Mat m_position_filter;  // gaussian filter centered at image center
        void init_position_filter();
        double get_position_filter_value(cv::Size image_sz, double x, double y);
        double get_tracking_filter_value(double xprev, double yprev, double x, double y);

        bool _apply(cv::Mat& image, dg::Timestamp ts);

        void test_line_detector(cv::Mat& frame);
        void grouping_test(cv::Mat& frame);

        // Hough-like vanishing point detection
        double detect_allsac(const std::vector<cv::Vec4f>& lines, cv::Size image_sz);
        double detect_ransac(const std::vector<cv::Vec4f>& lines, cv::Size image_sz);
        double detect_voting_degree(const std::vector<cv::Vec4f>& lines, cv::Size image_sz);
        double detect_allpos(const std::vector<cv::Vec4f>& lines, cv::Size image_sz);

        // ransac (major intersection of line segments)
        double line_score(double x1, double y1, double x2, double y2, int method = 1) const;
        double line_score(double x1, double y1, double x2, double y2, double x, double y, int method = 1) const;
        double line_weight(double x1, double y1, double x2, double y2, double x, double y) const;
        bool check_inlier(double x1, double y1, double x2, double y2, double x, double y, bool check_horizontal_distance = false) const;

        bool check_valid_line(double x1, double y1, double x2, double y2, double vx, double vy) const;
        bool check_valid_model(cv::Vec4f l1, cv::Vec4f l2, double vx, double vy) const;
        bool check_lower_line(double x1, double y1, double x2, double y2, double vx, double vy);
        bool check_left_line(double x1, double y1, double x2, double y2, double vx, double vy);

        void detect_lines_fld(cv::Mat frame, std::vector<cv::Vec4f>& lines, int length_threshold);
        void detect_lines_lsd(cv::Mat frame, std::vector<cv::Vec4f>& lines);
        void remove_vertical(cv::Mat& frame, std::vector<cv::Vec4f>& lines, double vert_deg = 80, cv::Scalar color = cv::Scalar(0, 0, 255));
        void remove_horizontal(cv::Mat& frame, std::vector<cv::Vec4f>& lines, double horz_deg = 5, cv::Scalar color = cv::Scalar(255, 0, 0));
        void connect_segmented_lines(std::vector<cv::Vec4f>& lines);
        bool connect_line(cv::Vec4f l1, cv::Vec4f l2, double min_gap, double max_gap, cv::Vec4f& l);
        void remove_duplicated_lines(std::vector<cv::Vec4f>& lines);
        bool line_intersection_LS(const std::vector<cv::Vec4f>& lines, double& x, double& y) const;
        bool line_intersection_weighted_LS(const std::vector<cv::Vec4f>& lines, double& x, double& y) const;
        bool line_intersection_Mestimator(const std::vector<cv::Vec4f>& lines, double& x, double& y) const;
        bool line_intersection_weighted_Mestimator(const std::vector<cv::Vec4f>& lines, double& x, double& y) const;
        void line_2pt(double x1, double y1, double x2, double y2, double& a, double& b, double& c) const;
        void line_2pt_normalized(double x1, double y1, double x2, double y2, double& a, double& b, double& c) const;
        bool line_intersect(double a1, double b1, double c1, double a2, double b2, double c2, double& x, double& y) const;
        bool line_intersect(double a1, double b1, double c1, double x1, double y1, double x2, double y2, double& x, double& y) const;
    };

} // End of 'dg'

#endif // End of '__ROAD_THETA__'
