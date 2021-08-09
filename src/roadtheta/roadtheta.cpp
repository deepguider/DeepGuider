#include "roadtheta.hpp"
#include "opencv2/ximgproc.hpp"

#define LOAD_PARAM_VALUE(fn, name_cfg, name_var) \
    if (!(fn)[name_cfg].empty()) (fn)[name_cfg] >> name_var;

#define RAD2DEG(x) ((x)*180/3.1415926535898)
#define DEG2RAD(x) ((x)*3.1415926535898/180)

using namespace cv;

namespace dg
{

    void loadConfig(const char* cfg_fname)
    {
        cv::FileStorage fs(cfg_fname, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            printf("Error: can't find %s\n", cfg_fname);
            return;
        }
        cv::FileNode fn = fs.root();
        int param_val = 0;
        LOAD_PARAM_VALUE(fn, "param_name", param_val);
    }


    bool RoadTheta::_apply(const cv::Mat& frame_input, dg::Timestamp ts)
    {
        if (m_param.test_line_detector) { test_line_detector(frame_input); return true; };

        // scale conversion
        cv::Mat frame;
        double image_scale = m_param.reference_image_width * m_param.image_scale_modifier / frame_input.cols;
        if (image_scale != 1)
        {
            int frame_w = (int)(frame_input.cols * image_scale + 0.5);
            int frame_h = (int)(frame_input.rows * image_scale + 0.5);
            cv::resize(frame_input, frame, cv::Size(frame_w, frame_h));
        }
        else
        {
            frame = frame_input.clone();
        }

        if (m_param.test_grouping) { grouping_test(frame); return 0; }

        // gray conversion
        Mat gray;
        if (frame.channels() > 1) cvtColor(frame, gray, COLOR_BGR2GRAY);
        else gray = frame;

        // line segments
        int length_threshold = (int)(m_param.line_length_min * m_param.image_scale_modifier); // default: 10
        std::vector<cv::Vec4f> lines;
        detect_lines_fld(gray, lines, length_threshold);

        // grouping
        if (m_param.connect_segmented_lines) connect_segmented_lines(lines);
        if (m_param.group_duplicated_lines) remove_duplicated_lines(lines);

        // filtering
        if (m_param.filter_vert_lines) remove_vertical(frame, lines, m_param.filter_vert_deg, m_param.color_line_vert);
        if (m_param.filter_horz_lines) remove_horizontal(frame, lines, m_param.filter_horz_deg, m_param.color_line_horz);

        m_lines = lines;
        m_scaled_w = frame.cols;
        m_scaled_h = frame.rows;

        // detection
        double score = -1;
        cv::Size image_sz = cv::Size(frame.cols, frame.rows);
        if (m_param.detect_method == 0) score = detect_allsac(lines, image_sz);
        if (m_param.detect_method == 1) score = detect_voting_degree(lines, image_sz);
        if (m_param.detect_method == 2) score = detect_allpos(lines, image_sz);

        // result
        m_result.vx = m_scaled_vx * frame_input.cols / frame.cols;
        m_result.vy = m_scaled_vy * frame_input.rows / frame.rows;
        m_result.score = score;

        double f = frame_input.cols / 2.0 / tan(DEG2RAD(m_param.hfov) / 2);
        double cx = frame_input.cols / 2.0;
        double cy = frame_input.rows / 2.0;
        m_result.theta = atan2(m_result.vx - cx, sqrt((cy - m_result.vy)*(cy - m_result.vy) + f * f));

        bool valid = true;
        if (m_param.apply_validation)
        {
            double min_vx = frame_input.cols * m_param.min_vx;
            double max_vx = frame_input.cols * m_param.max_vx;
            double min_vy = frame_input.rows * m_param.min_vy;
            double max_vy = frame_input.rows * m_param.max_vy;
            if (m_result.vx < min_vx || m_result.vx > max_vx || m_result.vy < min_vy || m_result.vy > max_vy) valid = false;

            if (m_param.detect_method == 1 && score < m_param.min_peak_score) valid = false;
            if (m_param.detect_method == 1 && valid && m_param.eval_peak_ransac && m_max_peak != m_max_peak_ransac) valid = false;
        }
        m_result.valid = (valid && score > 0);
        m_result.confidence = (m_result.valid) ? 1 : 0;

        return (valid && score > 0);
    }


    void RoadTheta::draw(cv::Mat& image) const
    {
        double image_scale = m_param.reference_image_width * m_param.image_scale_modifier / image.cols;
        int scaled_w = (int)(image.cols * image_scale + 0.5);
        int scaled_h = (int)(image.rows * image_scale + 0.5);

        double sx = (double)image.cols / scaled_w;
        double sy = (double)image.rows / scaled_h;
        double scale = (sx + sy) / 2;
        int line_w = (int)(scale + 0.5);

        // draw lines
        for (size_t i = 0; i < m_lines.size(); i++)
        {
            Vec4f v = m_lines[i];
            cv::line(image, cv::Point2d(v(0)*sx, v(1)*sy), cv::Point2d(v(2)*sx, v(3)*sy), m_param.color_line_basic, line_w);
        }

        if (!m_result.valid) return;

        // draw inliers
        double inlier_thr = m_param.inlier_thr * m_param.image_scale_modifier;
        for (int j = 0; j < (int)m_lines.size(); j++)
        {
            Vec4f v = m_lines[j];
            bool is_inlier = check_inlier(v(0), v(1), v(2), v(3), m_scaled_vx, m_scaled_vy, false);
            if (is_inlier)
            {
                cv::line(image, cv::Point2d(v(0)*sx, v(1)*sy), cv::Point2d(v(2)*sx, v(3)*sy), m_param.color_line_inlier, line_w);
            }
        }

        // draw vanishing point
        int radius = (int)(m_param.vpoint_radius * m_param.image_scale_modifier * scale + 0.5);
        cv::circle(image, cv::Point2d(m_scaled_vx * sx, m_scaled_vy * sy), radius, m_param.color_vpoint, cv::FILLED);
    }


    void RoadTheta::init_position_filter()
    {
        int img_w = (int)(m_param.reference_image_width * m_param.image_scale_modifier + 0.5);
        int img_h = img_w * 9 / 16;

        double cx = img_w / 2;
        double cy = img_h * m_param.camera_vanishing_y;
        double sx = img_w;
        double sy = img_h / 2;

        double sx2 = sx * sx;
        double sy2 = sy * sy;

        m_position_filter = cv::Mat::zeros(img_h, img_w, CV_64FC1);
        for (int y = 0; y < img_h; y++)
        {
            for (int x = 0; x < img_w; x++)
            {
                double md = (x - cx) * (x - cx) / sx2 + (y - cy) * (y - cy) / sy2;
                m_position_filter.at<double>(y, x) = exp(-md);
            }
        }
    }


    double RoadTheta::get_position_filter_value(cv::Size image_sz, double x, double y)
    {
        int img_w = image_sz.width;
        int img_h = image_sz.height;

        double cx = img_w / 2;
        double cy = img_h * m_param.camera_vanishing_y;
        double sx = img_w;
        double sy = img_h / 2;

        double sx2 = sx * sx;
        double sy2 = sy * sy;

        double md = (x - cx) * (x - cx) / sx2 + (y - cy) * (y - cy) / sy2;
        return exp(-md);
    }


    double RoadTheta::get_tracking_filter_value(double xprev, double yprev, double x, double y)
    {
        double sx2 = m_param.tracking_filter_sx * m_param.tracking_filter_sx;
        double sy2 = m_param.tracking_filter_sy * m_param.tracking_filter_sy;

        double md = (x - xprev) * (x - xprev) / sx2 + (y - yprev) * (y - yprev) / sy2;
        return exp(-md);
    }


    double compute_vanishing_points(cv::Mat& score, const std::vector<cv::Vec4f>& lines, double vy, double cx, double cy, double f)
    {
        int w = score.cols;
        int h = score.rows;
        for (size_t i = 0; i < lines.size(); i++)
        {
            double x1 = lines[i](0);
            double y1 = lines[i](1);
            double x2 = lines[i](2);
            double y2 = lines[i](3);
            double line_score1 = 1;
            double line_score2 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            double line_score3 = line_score2 * sqrt(line_score2);

            double vx = (vy - y1) * (x2 - x1) / (y2 - y1) + x1;
            if (vy >= 0 && vy < h && vx >= 0 && vx < w)
                score.at<double>((int)vy, (int)vx) += line_score3;
        }
        return 1;
    }

    double compute_road_direction(cv::Mat& score, const std::vector<Vec4f>& lines, double vy, double cx, double cy, double f)
    {
        int w = score.cols;
        int h = score.rows;
        for (size_t i = 0; i < lines.size(); i++)
        {
            double x1 = lines[i](0);
            double y1 = lines[i](1);
            double x2 = lines[i](2);
            double y2 = lines[i](3);
            double line_score1 = 1;
            double line_score2 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            double line_score3 = line_score2 * line_score2;

            double vx = (vy - y1) * (x2 - x1) / (y2 - y1) + x1;
            double theta = -atan2(vx - cx, sqrt((cy - vy) * (cy - vy) + f * f));
            int theta_deg = (int)RAD2DEG(theta);    // -90 ~ +90
            int theta_idx = theta_deg + 90;         // 0 ~ 180
            if (vy >= 0 && vy < h && theta_idx >= 0 && theta_idx < w)
                score.at<double>((int)vy, theta_idx) += line_score3;
        }
        return 1;
    }

    void non_max_suppression(cv::Mat m, std::vector<cv::Point2d>& peaks, std::vector<double>& peak_v, std::vector<double>& peak_contrast, double minv)
    {
        peaks.clear();

        cv::Mat m2;
        cv::blur(m, m2, cv::Size(5, 5));

        // initial nms
        int border_margin = 3;
        for (int r = border_margin; r < m.rows - border_margin; r++)
        {
            for (int c = border_margin; c < m.cols - border_margin; c++)
            {
                double v = m.at<double>(r, c);
                if (v <= minv) continue;
                bool is_peak = true;
                for (int dr = -1; dr <= 1; dr++)
                {
                    for (int dc = -1; dc <= 1; dc++)
                    {
                        if (dr == 0 && dc == 0) continue;

                        if (v < m.at<double>(r + dr, c + dc))
                        {
                            is_peak = false;
                            break;
                        }
                        else if (v == m.at<double>(r + dr, c + dc))
                        {
                            double v2 = m2.at<double>(r, c);
                            if (v2 <= m2.at<double>(r + dr, c + dc))
                            {
                                is_peak = false;
                                break;
                            }
                        }
                    }
                    if (!is_peak) break;
                }
                if (is_peak)
                {
                    double pr = 0;
                    double pc = 0;
                    double ps = 0;
                    double pmax = -1;
                    for (int rr = r - border_margin; rr <= r + border_margin; rr++)
                    {
                        for (int cc = c - border_margin; cc <= c + border_margin; cc++)
                        {
                            double pv = m.at<double>(rr, cc);
                            pr += rr * pv;
                            pc += cc * pv;
                            ps += pv;
                            int d = border_margin  - 1;
                            if ((rr<r - d || rr>r + d) && (cc<c - d || cc>c + d) && pv > pmax)
                            {
                                pmax = pv;
                            }
                        }
                    }

                    peaks.push_back(cv::Point2d(pc / ps, pr / ps));
                    peak_v.push_back(v);
                    peak_contrast.push_back(v - pmax);
                }
            }
        }
    }

    double RoadTheta::detect_voting_degree(const std::vector<cv::Vec4f>& lines, cv::Size image_sz)
    {
        // camera parameters
        double f = image_sz.width / 2.0 / tan(DEG2RAD(m_param.hfov) / 2);
        double cx = image_sz.width / 2.0;
        double cy = image_sz.height / 2.0;

        // theta range
        int hmin = m_param.pan_range[0];
        int hmax = m_param.pan_range[1];
        int vmin = m_param.tilt_range[0];
        int vmax = m_param.tilt_range[1];
        if (vmax < vmin)
        {
            vmax = (int)RAD2DEG(atan2(image_sz.height - 1 - cy, f));
        }
        if (m_param.use_vy_range)
        {
            int vy1 = (int)(m_param.vy_range[0] * image_sz.height);
            int vy2 = (int)(m_param.vy_range[1] * image_sz.height);
            if (vy2 < vy1) vy2 = image_sz.height - 1;

            vmin = (int)RAD2DEG(atan2(vy1 - cy, f));
            vmax = (int)RAD2DEG(atan2(vy2 - cy, f));
        }

        // score map
        int hn = (hmax - hmin) * m_param.theta_res_per_degree_horz + 1;
        int vn = (vmax - vmin) * m_param.theta_res_per_degree_vert + 1;
        cv::Mat score = cv::Mat::zeros(vn, hn, CV_64FC1);
        double theta_delta_vert = 1.0 / m_param.theta_res_per_degree_vert;
        double theta_delta_horz = 1.0 / m_param.theta_res_per_degree_horz;
        for (double theta_v = vmin; theta_v <= vmax ; theta_v += theta_delta_vert)
        {
            double vy = cy + f * tan(DEG2RAD(theta_v));     // y coordinate of vanishing point
            for (size_t i = 0; i < lines.size(); i++)
            {
                double x1 = lines[i](0);
                double y1 = lines[i](1);
                double x2 = lines[i](2);
                double y2 = lines[i](3);
                double vx = (vy - y1) * (x2 - x1) / (y2 - y1) + x1;

                bool ok = check_valid_line(x1, y1, x2, y2, vx, vy);
                if (!ok) continue;

                int theta_h = (int)RAD2DEG(atan2(vx - cx, sqrt((cy - vy) * (cy - vy) + f * f)));
                if (theta_h >= hmin && theta_h <= hmax)
                {
                    double lsocre = line_score(x1, y1, x2, y2, m_param.lscore_method);
                    int vi = (int)((theta_v - vmin) * m_param.theta_res_per_degree_vert);
                    int hi = (int)((theta_h - hmin) * m_param.theta_res_per_degree_horz);
                    double hi_remain = (theta_h - hmin) * m_param.theta_res_per_degree_horz - hi;
                    score.at<double>(vi, hi) += (lsocre * (1 - hi_remain));
                    if (hi + 1 < score.cols) score.at<double>(vi, hi + 1) += (lsocre * hi_remain);
                }
            }
        }

        // position filtering
        if (m_param.apply_position_filter)
        {
            for (int vi = 0; vi < vn; vi++)
            {
                double vtheta = vi * theta_delta_vert + vmin;
                double vy = cy + f * tan(DEG2RAD(vtheta));
                for (int hi = 0; hi < hn; hi++)
                {
                    double htheta = hi * theta_delta_horz + hmin;
                    double vx = cx + tan(DEG2RAD(htheta)) * sqrt((cy - vy) * (cy - vy) + f * f);

                    double w = get_position_filter_value(image_sz, vx, vy);
                    score.at<double>(vi, hi) *= w;
                }
            }
        }
            
        // find peak
        int blur_n = 5;
        for (int i = 0; i < blur_n; i++)
        {
            cv::blur(score, score, cv::Size(5, 5));
        }
        double minv, maxv;
        cv::Point minp1, maxp1;
        cv::minMaxLoc(score, &minv, &maxv, &minp1, &maxp1);

        // nms
        std::vector<cv::Point2d> peaks;
        std::vector<double> peak_v;
        std::vector<double> peak_contrast;
        non_max_suppression(score, peaks, peak_v, peak_contrast, maxv/3);

        maxv = -1;
        int peak_i = -1;
        cv::Point2d maxp = maxp1;
        if(m_param.print_peak_score) printf("peak detection:\n");
        for (int i = 0; i < (int)peaks.size(); i++)
        {
            if (peak_v[i] > maxv)
            {
                maxv = peak_v[i];
                maxp = peaks[i];
                peak_i = i;
            }
            if (m_param.print_peak_score)
            {
                double vtheta = peaks[i].y / m_param.theta_res_per_degree_vert + vmin;
                double htheta = peaks[i].x / m_param.theta_res_per_degree_horz + hmin;
                double vy = cy + f * tan(DEG2RAD(vtheta));
                double vx = cx + tan(DEG2RAD(htheta)) * sqrt((cy - vy) * (cy - vy) + f * f);
                printf("\t[%d] v = %.1lf, contrast = %.1lf, vx = %.0lf, vy = %.0lf\n", (int)i, peak_v[i], peak_contrast[i], vx, vy);
            }
        }

        // ransac evaluation
        std::vector<double> peaks_score_ransac;
        peaks_score_ransac.resize(peaks.size());
        double best_score_ransac = 0;
        int max_peak_ransac = -1;
        for (int i = 0; i < (int)peaks.size(); i++)
        {
            double vtheta = peaks[i].y / m_param.theta_res_per_degree_vert + vmin;
            double htheta = peaks[i].x / m_param.theta_res_per_degree_horz + hmin;
            double vy = cy + f * tan(DEG2RAD(vtheta));
            double vx = cx + tan(DEG2RAD(htheta)) * sqrt((cy - vy) * (cy - vy) + f * f);

            double ransac_score = 0;
            for (int j = 0; j < (int)lines.size(); j++)
            {
                bool is_inlier = check_inlier(lines[j](0), lines[j](1), lines[j](2), lines[j](3), vx, vy);
                if (is_inlier)
                {
                    ransac_score += line_score(lines[j](0), lines[j](1), lines[j](2), lines[j](3), vx, vy, m_param.lscore_method);
                }
            }

            // centering filter
            if (m_param.apply_position_filter)
            {
                double w = get_position_filter_value(image_sz, vx, vy);
                ransac_score *= w;
            }

            peaks_score_ransac[i] = ransac_score;

            if (ransac_score > best_score_ransac)
            {
                best_score_ransac = ransac_score;
                max_peak_ransac = i;
            }
        }

        // save results
        double vtheta = maxp.y / m_param.theta_res_per_degree_vert + vmin;
        double htheta = maxp.x / m_param.theta_res_per_degree_horz + hmin;
        double vy = cy + f * tan(DEG2RAD(vtheta));
        double vx = cx + tan(DEG2RAD(htheta)) * sqrt((cy - vy) * (cy - vy) + f * f);

        m_scoremap = score;
        m_peaks = peaks;
        m_peaks_score = peak_v;
        m_max_peak = peak_i;
        m_scaled_vx = vx;
        m_scaled_vy = vy;
        m_score = maxv;
        m_score_ransac = best_score_ransac;
        m_max_peak_ransac = max_peak_ransac;
        m_peaks_score_ransac = peaks_score_ransac;

        return maxv;
    }


    double RoadTheta::detect_allpos(const std::vector<cv::Vec4f>& lines, cv::Size image_sz)
    {
        int vy1 = 0;
        int vy2 = image_sz.height;
        if (m_param.use_vy_range)
        {
            vy1 = (int)(m_param.vy_range[0] * image_sz.height);
            vy2 = (int)(m_param.vy_range[1] * image_sz.height);
        }
        int vx1 = 0;
        int vx2 = image_sz.width;

        // score map
        int vxn = vx2 - vx1;
        int vyn = vy2 - vy1;
        cv::Mat score = cv::Mat::zeros(vyn, vxn, CV_64FC1);
        for (int y = vy1; y < vy2; y++)
        {
            for (int x = vx1; x < vx2; x++)
            {
                double w = get_position_filter_value(image_sz, x, y);

                for (size_t i = 0; i < lines.size(); i++)
                {
                    double x1 = lines[i](0);
                    double y1 = lines[i](1);
                    double x2 = lines[i](2);
                    double y2 = lines[i](3);

                    bool ok = check_valid_line(x1, y1, x2, y2, x, y);
                    if (!ok) continue;

                    double lsocre = line_score(x1, y1, x2, y2, x, y, m_param.lscore_method);
                    score.at<double>(y - vy1, x - vx1) += lsocre;
                }

                score.at<double>(y - vy1, x - vx1) *= w;
            }
        }

        int blur_n = 5;
        for (int i = 0; i < blur_n; i++)
        {
            cv::blur(score, score, cv::Size(5, 5));
        }

        double minv, maxv;
        cv::Point minp, maxp;
        cv::minMaxLoc(score, &minv, &maxv, &minp, &maxp);

        std::vector<cv::Point2d> peaks;
        peaks.push_back(maxp);
        int peak_i = 0;

        double vx = maxp.x + vx1;
        double vy = maxp.y + vy1;

        // save results
        m_scoremap = score;
        m_peaks = peaks;
        m_max_peak = peak_i;
        m_scaled_vx = vx;
        m_scaled_vy = vy;

        return maxv;


        // find peak
        /*
        int blur_n = 5;
        for (int i = 0; i < blur_n; i++)
        {
            cv::blur(score, score, cv::Size(5, 5));
        }
        double minv, maxv;
        cv::Point minp1, maxp1;
        cv::minMaxLoc(score, &minv, &maxv, &minp1, &maxp1);

        // nms
        std::vector<cv::Point2d> peaks;
        std::vector<double> peak_v;
        std::vector<double> peak_contrast;
        non_max_suppression(score, peaks, peak_v, peak_contrast, maxv / 3);

        // position filtering
        if (m_param.apply_position_filter)
        {
            for (size_t i = 0; i < peaks.size(); i++)
            {
                double vx = peaks[i].x + vx1;
                double vy = peaks[i].y + vy1;
                double w = get_position_filter_value(image_sz, vx, vy);
                peak_v[i] = peak_v[i] * w;
                score.at<double>(peaks[i]) *= w;
            }
        }

        maxv = -1;
        int peak_i = -1;
        cv::Point2d maxp = maxp1;
        if (m_param.print_peak_score) printf("peak detection:\n");
        for (size_t i = 0; i < peaks.size(); i++)
        {
            if (peak_v[i] > maxv)
            {
                maxv = peak_v[i];
                maxp = peaks[i];
                peak_i = i;
            }
        }

        double vx = maxp.x + vx1;
        double vy = maxp.y + vy1;

        // save results
        m_scoremap = score;
        m_peaks = peaks;
        m_max_peak = peak_i;
        m_scaled_vx = vx;
        m_scaled_vy = vy;

        return maxv;
        */
    }


    void RoadTheta::line_2pt(double x1, double y1, double x2, double y2, double& a, double& b, double& c) const
    {
        a = -(y2 - y1);
        b = x2 - x1;
        c = (y2 - y1) * x1 - (x2 - x1) * y1;
    }

    void RoadTheta::line_2pt_normalized(double x1, double y1, double x2, double y2, double& a, double& b, double& c) const
    {
        a = -(y2 - y1);
        b = x2 - x1;
        c = (y2 - y1) * x1 - (x2 - x1) * y1;

        double s = sqrt(a * a + b * b);
        a = a / s;
        b = b / s;
        c = c / s;
    }

    bool RoadTheta::line_intersect(double a1, double b1, double c1, double a2, double b2, double c2, double& x, double& y) const
    {
        double d = a1 * b2 - b1 * a2;
        x = -(b2 * c1 - b1 * c2) / d;
        y = -(-a2 * c1 + a1 * c2) / d;

        double eps = 0.000001;
        if (d > -eps && d < eps) return false;
        else return true;
    }

    bool RoadTheta::line_intersect(double a1, double b1, double c1, double x1, double y1, double x2, double y2, double& x, double& y) const
    {
        double a2, b2, c2;
        line_2pt(x1, y1, x2, y2, a2, b2, c2);
        return line_intersect(a1, b1, c1, a2, b2, c2, x, y);
    }

    bool RoadTheta::line_intersection_LS(const std::vector<cv::Vec4f>& lines, double& x, double& y) const
    {
        int n = (int)lines.size();
        if (n < 2) return false;

        Mat A = Mat::ones(n, 2, CV_64FC1);
        Mat B = Mat::ones(n, 1, CV_64FC1);
        for (int i = 0; i < n; i++)
        {
            // 수직거리를 최소화할수 있도록 normalized line equation을 사용
            double a, b, c;
            line_2pt_normalized(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

            A.at<double>(i, 0) = a;
            A.at<double>(i, 1) = b;
            B.at<double>(i) = -c;
        }

        Mat pinvA = A.inv(DECOMP_SVD);
        Mat p = pinvA * B;

        x = p.at<double>(0);
        y = p.at<double>(1);

        return true;
    }

    bool RoadTheta::line_intersection_weighted_LS(const std::vector<cv::Vec4f>& lines, double& x, double& y) const
    {
        int n = (int)lines.size();
        if (n < 2) return false;

        Mat A = Mat::zeros(n, 2, CV_64FC1);
        Mat B = Mat::zeros(n, 1, CV_64FC1);
        Mat w = Mat::zeros(n, 1, CV_64FC1);
        for (int i = 0; i < n; i++)
        {
            // 수직거리를 최소화할수 있도록 normalized line equation을 사용
            double a, b, c;
            line_2pt_normalized(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

            A.at<double>(i, 0) = a;
            A.at<double>(i, 1) = b;
            B.at<double>(i) = -c;

            double score = line_score(lines[i](0), lines[i](1), lines[i](2), lines[i](3), m_param.weight_lscore_method);
            w.at<double>(i) = score;
        }

        Mat W = Mat::diag(w);
        Mat WA = A.t()* W* A;
        Mat p = WA.inv(DECOMP_SVD)* A.t()* W* B;

        x = p.at<double>(0);
        y = p.at<double>(1);

        return true;
    }

    bool RoadTheta::line_intersection_Mestimator(const std::vector<cv::Vec4f>& lines, double& x, double& y) const
    {
        int n_itr = 20;
        int n = (int)lines.size();
        if (n < 2) return false;

        Mat A = Mat::zeros(n, 2, CV_64FC1);
        Mat B = Mat::zeros(n, 1, CV_64FC1);
        for (int i = 0; i < n; i++)
        {
            // 수직거리를 최소화할수 있도록 normalized line equation을 사용
            double a, b, c;
            line_2pt_normalized(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

            A.at<double>(i, 0) = a;
            A.at<double>(i, 1) = b;
            B.at<double>(i) = -c;
        }

        Mat w = Mat::zeros(n, 1, CV_64FC1);
        Mat p = A.inv(DECOMP_SVD) * B;
        for (int itr = 0; itr < n_itr; itr++)
        {
            Mat r = A * p - B;
            for (int i = 0; i < n; i++)
            {
                w.at<double>(i) = 1.0 / (1 + fabs(r.at<double>(i)) / 1.3998);   // Fair loss
            }
            Mat W = Mat::diag(w);
            Mat AtWA = A.t() * W * A;
            p = AtWA.inv(DECOMP_SVD) * A.t() * W * B;

        }
        x = p.at<double>(0);
        y = p.at<double>(1);

        return true;
    }


    bool RoadTheta::line_intersection_weighted_Mestimator(const std::vector<cv::Vec4f>& lines, double& x, double& y) const
    {
        int n_itr = 20;
        int n = (int)lines.size();
        if (n < 2) return false;

        Mat A = Mat::zeros(n, 2, CV_64FC1);
        Mat B = Mat::zeros(n, 1, CV_64FC1);
        Mat s = Mat::zeros(n, 1, CV_64FC1);
        for (int i = 0; i < n; i++)
        {
            // 수직거리를 최소화할수 있도록 normalized line equation을 사용
            double a, b, c;
            line_2pt_normalized(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

            A.at<double>(i, 0) = a;
            A.at<double>(i, 1) = b;
            B.at<double>(i) = -c;

            double score = line_score(lines[i](0), lines[i](1), lines[i](2), lines[i](3), m_param.weight_lscore_method);
            s.at<double>(i) = score;
        }

        Mat w = Mat::zeros(n, 1, CV_64FC1);
        Mat p = A.inv(DECOMP_SVD) * B;
        for (int itr = 0; itr < n_itr; itr++)
        {
            Mat r = A * p - B;
            for (int i = 0; i < n; i++)
            {
                w.at<double>(i) = s.at<double>(i) / (1 + fabs(r.at<double>(i)) / 1.3998);   // weighted Fair loss
            }
            Mat W = Mat::diag(w);
            Mat AtWA = A.t() * W * A;
            p = AtWA.inv(DECOMP_SVD) * A.t() * W * B;

        }
        x = p.at<double>(0);
        y = p.at<double>(1);

        return true;
    }


    double RoadTheta::line_score(double x1, double y1, double x2, double y2, int method) const
    {
        if (method == 0) return 1;

        double score = 0;
        double length2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        if (method == 1) score = sqrt(length2);
        if (method == 2)
        {
            double l = sqrt(length2);
            score = l * sqrt(l);
        }
        if (method == 3) score = length2;
        return score;
    }


    double RoadTheta::line_score(double x1, double y1, double x2, double y2, double x, double y, int method) const
    {
        if (method == 0) return 1;

        double score = 0;
        double length2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        if (method == 1) score = sqrt(length2);
        if (method == 2)
        {
            double l = sqrt(length2);
            score = l * sqrt(l);
        }
        if (method == 3) score = length2;

        if (m_param.use_weighted_lscore)
        {
            double a, b, c;
            line_2pt(x1, y1, x2, y2, a, b, c);
            double lx = -(b * y + c) / a;
            double horz_d = fabs(lx - x);

            double t1 = m_param.max_vanishing_error * m_param.image_scale_modifier;
            double t2 = m_param.horizontal_distance_thr * m_param.image_scale_modifier;
            double mu = (t1 + t2) / 2;
            double s = (t2 - t1) / 5;
            double gamma = 1 / (1 + exp((horz_d - mu) / s));

            score = score * gamma;
        }

        if (m_param.use_line_weight)
        {
            double w = line_weight(x1, y1, x2, y2, x, y);
            score = score * w;
        }

        return score;
    }


    double RoadTheta::line_weight(double x1, double y1, double x2, double y2, double vx, double vy) const
    {
        // orthogonal distance
        double a, b, c;
        line_2pt(x1, y1, x2, y2, a, b, c);
        double dd = (a * vx + b * vy + c) * (a * vx + b * vy + c) / (a * a + b * b);
        double d = sqrt(dd);

        // line that pass vp and is normal to the segment: (x2-x1)(x-vx)+(y2-y1)(y-vy) = 0
        cv::Point2d vp(vx, vy);
        cv::Point2d p1(x1, y1);
        cv::Point2d p2(x2, y2);
        double f1 = (p2.x - p1.x) * (p1.x - vp.x) + (p2.y - p1.y) * (p1.y - vp.y);
        double f2 = (p2.x - p1.x) * (p2.x - vp.x) + (p2.y - p1.y) * (p2.y - vp.y);

        // horizontal distance
        double h = 0;
        if (f1 * f2 <= 0) // line cross
        {
            h = 0;
        }
        else
        {
            if (fabs(f1) < fabs(f2))   // p1 is close to vp
                h = fabs(f1) / norm(p1 - p2);
            else
                h = fabs(f2) / norm(p1 - p2);
        }

        // line weight
        double k = (fabs(h - 50) - fabs(h) + fabs(h - 100) / 10) / 55 + 1.91;
        if (h >= 100) k = 1;
        double t1 = 10 / k;
        double t2 = 20 / k;
        double m = (t1 + t2) / 2;
        double s = (t2 - t1) / 5;
        double w = 1 / (1 + exp(d - m) / s);

        return w;
    }


    bool RoadTheta::check_valid_model(cv::Vec4f l1, cv::Vec4f l2, double vx, double vy) const
    {
        cv::Point2d vp(vx, vy);
        cv::Point2d p1(l1(0), l1(1));
        cv::Point2d p2(l1(2), l1(3));
        cv::Point2d q1(l2(0), l2(1));
        cv::Point2d q2(l2(2), l2(3));

        // check camera geometry
        if (m_param.check_camera_geometry)
        {
            // camera parameters
            double f = m_scaled_w / 2.0 / tan(DEG2RAD(m_param.hfov) / 2);
            double cx = m_scaled_w / 2.0;
            double cy = m_scaled_h / 2.0;

            // theta range
            int pan_min = m_param.pan_range[0];
            int pan_max = m_param.pan_range[1];
            int tilt_min = m_param.tilt_range[0];
            int tilt_max = m_param.tilt_range[1];
            if (tilt_max < tilt_min)
            {
                tilt_max = (int)RAD2DEG(atan2(m_scaled_h - 1 - cy, f));
            }
            if (m_param.use_vy_range)
            {
                int vy1 = (int)(m_param.vy_range[0] * m_scaled_h);
                int vy2 = (int)(m_param.vy_range[1] * m_scaled_h);
                if (vy2 < vy1) vy2 = (int)(m_scaled_h * 0.9);

                tilt_min = (int)RAD2DEG(atan2(vy1 - cy, f));
                tilt_max = (int)RAD2DEG(atan2(vy2 - cy, f));
            }

            double tilt = RAD2DEG(atan2(vy - cy, f));
            if (tilt<tilt_min || tilt>tilt_max) return false;

            double pan = RAD2DEG(atan2(vx - cx, sqrt((cy - vy) * (cy - vy) + f * f)));
            if (pan<pan_min || pan>pan_max) return false;
        }

        // check vanishing(converging) condition
        if (m_param.check_vanishing_condition)
        {
            double d_thr = m_param.max_vanishing_error * m_param.image_scale_modifier;
            if (norm(p1 - vp) > d_thr && norm(p2 - vp) > d_thr)
            {
                // line that pass vp and is normal to the segment: (x2-x1)(x-vx)+(y2-y1)(y-vy) = 0
                double v1 = (p2.x - p1.x) * (p1.x - vp.x) + (p2.y - p1.y) * (p1.y - vp.y);
                double v2 = (p2.x - p1.x) * (p2.x - vp.x) + (p2.y - p1.y) * (p2.y - vp.y);

                // check line cross
                if (v1 * v2 < 0) return false;
            }
            if (norm(q1 - vp) > d_thr && norm(q2 - vp) > d_thr)
            {
                // line that pass vp and is normal to the segment: (x2-x1)(x-vx)+(y2-y1)(y-vy) = 0
                double v1 = (q2.x - q1.x) * (q1.x - vp.x) + (q2.y - q1.y) * (q1.y - vp.y);
                double v2 = (q2.x - q1.x) * (q2.x - vp.x) + (q2.y - q1.y) * (q2.y - vp.y);

                // check line cross
                if (v1 * v2 < 0) return false;
            }
        }

        // check ground condition
        if (m_param.check_ground_condition)
        {
            cv::Point2d lp = (norm(p1 - vp) < norm(p2 - vp)) ? p2 - p1 : p1 - p2;
            double theta1 = RAD2DEG(atan2(lp.y, lp.x));
            cv::Point2d lq = (norm(q1 - vp) < norm(q2 - vp)) ? q2 - q1 : q1 - q2;
            double theta2 = RAD2DEG(atan2(lq.y, lq.x));

            // caution: image y coordinate increases downwards
            if (theta1 < 0 && theta2 < 0) return false;
            if (theta1 < -m_param.max_upperline_deg && theta1 > -(180 - m_param.max_upperline_deg)) return false;
            if (theta2 < -m_param.max_upperline_deg && theta2 > -(180 - m_param.max_upperline_deg)) return false;
        }

        return true;
    }


    bool RoadTheta::check_valid_line(double x1, double y1, double x2, double y2, double vx, double vy) const
    {
        double a, b, c;
        line_2pt(x1, y1, x2, y2, a, b, c);
        double dd = (a * vx + b * vy + c)*(a * vx + b * vy + c) / (a * a + b * b);

        // check distance
        double d_thr = m_param.max_vanishing_error * m_param.image_scale_modifier;
        double dd_thr = d_thr * d_thr;
        if (dd > dd_thr) return false;

        // check vanishing(converging) condition
        double dd1 = (vx - x1) * (vx - x1) + (vy - y1) * (vy - y1);
        double dd2 = (vx - x2) * (vx - x2) + (vy - y2) * (vy - y2);
        if (m_param.check_vanishing_condition)
        {
            // segment should converge to the point (not intersect)
            if (dd1 > dd_thr && dd2 > dd_thr)
            {
                // line that pass the point and normal to the segment: (x2-x1)(x-vx)+(y2-y1)(y-vy) = 0
                double v1 = (x2 - x1) * (x1 - vx) + (y2 - y1) * (y1 - vy);
                double v2 = (x2 - x1) * (x2 - vx) + (y2 - y1) * (y2 - vy);

                // check line cross
                if (v1 * v2 < 0) return false;
            }
        }

        // check ground condition (ground lines should be under the vanishing point)
        if (m_param.check_ground_condition)
        {
            // caution: image y coordinate increases downwards
            double theta = (dd1 < dd2) ? atan2(y2 - y1, x2 - x1) : atan2(y1 - y2, x1 - x2);
            double theta_deg = RAD2DEG(theta);
            if (theta_deg < -m_param.max_upperline_deg && theta_deg > -(180 - m_param.max_upperline_deg)) return false;
        }

        return true;
    }


    bool RoadTheta::check_lower_line(double x1, double y1, double x2, double y2, double vx, double vy)
    {
        double cy = (y1 + y2) / 2;
        return (cy > vy);
    }


    bool RoadTheta::check_left_line(double x1, double y1, double x2, double y2, double vx, double vy)
    {
        double cx = (x1 + x2) / 2;
        return (cx < vx);
    }


    bool RoadTheta::check_inlier(double x1, double y1, double x2, double y2, double x, double y, bool check_horizontal_distance) const
    {
        // check perpendicular distance
        double d_thr = m_param.inlier_thr * m_param.image_scale_modifier;
        double a, b, c;
        line_2pt(x1, y1, x2, y2, a, b, c);
        double d = fabs(a * x + b * y + c) / sqrt(a * a + b * b);
        if (d > d_thr) return false;

        // check vanishing distance (horizontal distance)
        if (check_horizontal_distance)
        {
            double h_thr = m_param.horizontal_distance_thr * m_param.image_scale_modifier;
            double lx = -(b * y + c) / a;
            double horz_d = fabs(lx - x);
            if (horz_d > h_thr) return false;
        }

        // check vanishing condition
        return check_valid_line(x1, y1, x2, y2, x, y);
    }


    double RoadTheta::detect_allsac(const std::vector<cv::Vec4f>& lines, cv::Size image_sz)
    {
        if (lines.size() < 2) return -1;

        int lscore_method = m_param.lscore_method;

        int n = (int)lines.size();
        double best_score = -1;
        double best_vx, best_vy;
        for (int i1 = 0; i1 < n; i1++)
        {
            for (int i2 = i1 + 1; i2 < n; i2++)
            {
                // model estimation (line intersection)
                double a1, b1, c1;
                double a2, b2, c2;
                double vx, vy;
                line_2pt(lines[i1](0), lines[i1](1), lines[i1](2), lines[i1](3), a1, b1, c1);
                line_2pt(lines[i2](0), lines[i2](1), lines[i2](2), lines[i2](3), a2, b2, c2);
                bool ok = line_intersect(a1, b1, c1, a2, b2, c2, vx, vy);
                if (!ok) continue;

                // model validation
                if (m_param.check_sampled_model)
                {
                    bool ok = check_valid_model(lines[i1], lines[i2], vx, vy);
                    if (!ok) continue;
                }

                // model evaluation
                double score = 0;
                for (int j = 0; j < n; j++)
                {
                    bool is_inlier = check_inlier(lines[j](0), lines[j](1), lines[j](2), lines[j](3), vx, vy);
                    if (is_inlier)
                    {
                        score += line_score(lines[j](0), lines[j](1), lines[j](2), lines[j](3), vx, vy, lscore_method);
                    }
                }

                // centering filter
                if (m_param.apply_position_filter)
                {
                    double w = get_position_filter_value(image_sz, vx, vy);
                    score *= w;
                }

                // model selection
                if (score > best_score)
                {
                    best_score = score;
                    best_vx = vx;
                    best_vy = vy;
                }
            }
        }
        if (best_score < 0) return -1;

        // inliers (orthogonal distance)
        std::vector<cv::Vec4f> inliers_ransac;
        for (int j = 0; j < n; j++)
        {
            bool is_inlier = check_inlier(lines[j](0), lines[j](1), lines[j](2), lines[j](3), best_vx, best_vy);
            if (is_inlier)
                inliers_ransac.push_back(lines[j]);
        }
        if (inliers_ransac.empty()) return -1;

        // model refinement
        double vx = best_vx;
        double vy = best_vy;
        std::vector<cv::Vec4f> inliers = inliers_ransac;
        if (m_param.refinement_method == 1)     // LS
        {
            bool ok = line_intersection_LS(inliers, vx, vy);
            if (!ok) return -1;

            int itr = m_param.refine_iteration_n;
            if (m_param.check_horizontal_distance && itr < 2) itr = 2;
            for (int i = 1; i < itr; i++)
            {
                inliers.clear();
                for (int j = 0; j < n; j++)
                {
                    bool is_inlier = check_inlier(lines[j](0), lines[j](1), lines[j](2), lines[j](3), vx, vy, m_param.check_horizontal_distance);
                    if (is_inlier)
                        inliers.push_back(lines[j]);
                }
                if ((int)inliers.size() < 2) break;

                bool ok = line_intersection_LS(inliers, vx, vy);
                if (!ok) return -1;
            }
        }
        else if (m_param.refinement_method == 2)    // weighted LS
        {
            bool ok = line_intersection_weighted_LS(inliers, vx, vy);
            if (!ok) return -1;

            int itr = m_param.refine_iteration_n;
            if (m_param.check_horizontal_distance && itr < 2) itr = 2;
            for (int i = 1; i < itr; i++)
            {
                inliers.clear();
                for (int j = 0; j < n; j++)
                {
                    bool is_inlier = check_inlier(lines[j](0), lines[j](1), lines[j](2), lines[j](3), vx, vy, m_param.check_horizontal_distance);
                    if (is_inlier)
                        inliers.push_back(lines[j]);
                }
                if ((int)inliers.size() < 2) break;

                bool ok = line_intersection_weighted_LS(inliers, vx, vy);
                if (!ok) return -1;
            }
        }
        else if (m_param.refinement_method == 3)    // m-estimator
        {
            bool ok = line_intersection_Mestimator(inliers, vx, vy);
            if (!ok) return -1;
        }
        else if (m_param.refinement_method == 4)    // weighted m-estimator
        {
            bool ok = line_intersection_weighted_Mestimator(inliers, vx, vy);
            if (!ok) return -1;
        }

        // final score
        double score = 0;
        for (size_t i = 0; i < inliers.size(); i++)
        {
            score += line_score(inliers[i](0), inliers[i](1), inliers[i](2), inliers[i](3), vx, vy, lscore_method);
        }
        score = sqrt(score);

        // save results
        m_scaled_vx = vx;
        m_scaled_vy = vy;

        return score;
    }


    void RoadTheta::detect_lines_fld(cv::Mat gray, std::vector<cv::Vec4f>& lines, int _length_threshold)
    {
        // fast line detector
        float 	_distance_threshold = 3; // default: 1.414213562f;
        double 	_canny_th1 = 50.0;
        double 	_canny_th2 = 50.0;
        int 	_canny_aperture_size = 3;
        bool 	_do_merge = false;
        Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
            _length_threshold, _distance_threshold, _canny_th1, _canny_th2, _canny_aperture_size, _do_merge);

        fld->detect(gray, lines);
    }

    void RoadTheta::detect_lines_lsd(cv::Mat gray, std::vector<cv::Vec4f>& lines)
    {
        // Create and LSD detector with standard or no refinement.
        int _method = cv::LSD_REFINE_NONE;   // LSD_REFINE_ADV, LSD_REFINE_STD, LSD_REFINE_NONE
        double 	_scale = 0.8;
        double 	_sigma_scale = 0.6;
        double 	_quant = 2.0;
        double 	_ang_th = 22.5;
        double 	_log_eps = 0;
        double 	_density_th = 0.7;
        int 	_n_bins = 1024;

        cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(
            _method, _scale, _sigma_scale, _quant, _ang_th, _log_eps, _density_th, _n_bins);

        lsd->detect(gray, lines);
    }

    void RoadTheta::remove_vertical(cv::Mat& frame, std::vector<cv::Vec4f>& lines, double vert_deg, cv::Scalar color)
    {
        std::vector<Vec4f> tmp;

        double ythr = frame.rows * m_param.vert_line_upper_bound;
        float x1, y1, x2, y2;
        for (size_t i = 0; i < lines.size(); i++)
        {
            x1 = lines[i](0);
            y1 = lines[i](1);
            x2 = lines[i](2);
            y2 = lines[i](3);

            if (y1 >= ythr && y2 >= ythr)
            {
                tmp.push_back(lines[i]);
            }
            else
            {
                double theta = atan2(fabs(y2 - y1), fabs(x2 - x1));
                double theta_deg = RAD2DEG(theta);    // 0 ~ +90
                if (theta_deg <= vert_deg) tmp.push_back(lines[i]);
            }
        }
        lines = tmp;
    }

    void RoadTheta::remove_horizontal(cv::Mat& frame, std::vector<cv::Vec4f>& lines, double horz_deg, cv::Scalar color)
    {
        std::vector<Vec4f> tmp;

        float x1, y1, x2, y2;
        for (size_t i = 0; i < lines.size(); i++)
        {
            x1 = lines[i](0);
            y1 = lines[i](1);
            x2 = lines[i](2);
            y2 = lines[i](3);

            double theta = atan2(fabs(y2 - y1), fabs(x2 - x1));
            double theta_deg = RAD2DEG(theta);    // 0 ~ +90
            if (theta_deg >= horz_deg) tmp.push_back(lines[i]);
        }
        lines = tmp;
    }


    void RoadTheta::test_line_detector(const cv::Mat& frame)
    {
        std::vector<double> scales = { 0.75, 0.5, 0.39 };
        cv::namedWindow("line detector", 0);

        cv::Mat img_result;
        for (size_t i = 0; i < scales.size(); i++)
        {
            double scale = scales[i];
            cv::Mat frame_scaled;
            if (scale != 1)
            {
                int frame_w = (int)(frame.cols * scale + 0.5);
                int frame_h = (int)(frame.rows * scale + 0.5);
                cv::resize(frame, frame_scaled, cv::Size(frame_w, frame_h));
            }
            else
            {
                frame_scaled = frame;
            }

            Mat gray;
            if (frame_scaled.channels() > 1)
                cvtColor(frame_scaled, gray, COLOR_BGR2GRAY);
            else
                gray = frame_scaled;

            std::vector<cv::Vec4f> lines_fld, lines_lsd;
            detect_lines_fld(gray, lines_fld, 20);
            detect_lines_lsd(gray, lines_lsd);

            cv::Mat img_fld = frame_scaled.clone();
            cv::Mat img_lsd = frame_scaled.clone();
            for (size_t i = 0; i < lines_fld.size(); i++)
            {
                Vec4f v = lines_fld[i];
                cv::line(img_fld, cv::Point2f(v(0), v(1)), cv::Point2f(v(2), v(3)), cv::Scalar(0, 255, 0), 1);
            }
            for (size_t i = 0; i < lines_lsd.size(); i++)
            {
                Vec4f v = lines_lsd[i];
                cv::line(img_lsd, cv::Point2f(v(0), v(1)), cv::Point2f(v(2), v(3)), cv::Scalar(0, 255, 0), 1);
            }

            std::string fld_str = cv::format("FLD(scale=%.2lf)", scale);
            std::string lsd_str = cv::format("LSD(scale=%.2lf)", scale);
            putText(img_fld, fld_str, cv::Point(5, 35), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0), 2);
            putText(img_lsd, lsd_str, cv::Point(5, 35), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0), 2);

            cv::Mat img;
            cv::hconcat(img_fld, img_lsd, img);
            if (img.cols < img_result.cols)
            {
                cv::Mat blank = cv::Mat::zeros(img.rows, img_result.cols - img.cols, img.type());
                cv::hconcat(img, blank, img);
            }

            if (img_result.empty())
                img_result = img;
            else
                cv::vconcat(img_result, img, img_result);
        }

        if(img_result.rows>1440 || img_result.cols>3440) cv::namedWindow("line detector", 0);
        cv::imshow("line detector", img_result);
    }


    bool RoadTheta::connect_line(cv::Vec4f l1, cv::Vec4f l2, double min_gap, double max_gap, cv::Vec4f& combined_l)
    {
        cv::Point2d p1(l1(0), l1(1));
        cv::Point2d p2(l1(2), l1(3));
        cv::Point2d q1(l2(0), l2(1));
        cv::Point2d q2(l2(2), l2(3));

        cv::Point2d v = p2 - p1;
        cv::Point2d nv = v / norm(v);
        double pv = norm(v);

        double qv1 = (q1 - p1).ddot(nv);        // projected coordinate on v
        double qv2 = (q2 - p1).ddot(nv);        // projected coordinate on v

        bool connected = false;
        combined_l = l1;
        if (qv1 < qv2)
        {
            if (qv1 < 0 && qv2 < pv && qv2 <= -min_gap && qv2 >= -max_gap)
            {
                combined_l(0) = l2(0);
                combined_l(1) = l2(1);
                connected = true;
            }
            if (qv1 > 0 && qv2 > pv && qv1 >= pv + min_gap && qv1 <= pv + max_gap)
            {
                combined_l(2) = l2(2);
                combined_l(3) = l2(3);
                connected = true;
            }
        }
        else
        {
            if (qv2 < 0 && qv1 < pv && qv1 <= -min_gap && qv1 >= -max_gap)
            {
                combined_l(0) = l2(2);
                combined_l(1) = l2(3);
                connected = true;
            }
            if (qv2 > 0 && qv1 > pv && qv2 >= pv + min_gap && qv2 <= pv + max_gap)
            {
                combined_l(2) = l2(0);
                combined_l(3) = l2(1);
                connected = true;
            }
        }

        return connected;
    }


    void RoadTheta::connect_segmented_lines(std::vector<cv::Vec4f>& lines)
    {
        // compute line length
        std::vector<double> len2;
        for (size_t i = 0; i < lines.size(); i++)
        {
            double x1 = lines[i](0);
            double y1 = lines[i](1);
            double x2 = lines[i](2);
            double y2 = lines[i](3);
            double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

            len2.push_back(d2);
        }

        // sort in descending order w.r.t. line length
        for (size_t i = 0; i < lines.size(); i++)
        {
            int k = (int)i;
            while (k > 0 && len2[k] > len2[k - 1])
            {
                double d = len2[k - 1];
                len2[k - 1] = len2[k];
                len2[k] = d;

                cv::Vec4f l = lines[k - 1];
                lines[k - 1] = lines[k];
                lines[k] = l;
                k--;
            }
        }

        // grouping candidates
        struct match {
            int i;                       // anchor line index
            std::vector<int> j;          // matchable line index
            std::vector<double> err;     // matching error (average projection distance)
            std::vector<bool> connectable;
            std::vector<cv::Vec4f> combined_l;

            int best_j = -1;
            double best_err = -1;
            cv::Vec4f best_l;
        };
        std::vector<match> candidates;
        double connect_d_thr = m_param.connect_dist_thr;
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Point2d p1(lines[i](0), lines[i](1));
            cv::Point2d p2(lines[i](2), lines[i](3));
            double a, b, c;
            line_2pt(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

            match m;
            m.i = (int)i;
            int n_connectable = 0;
            for (size_t j = i + 1; j < lines.size(); j++)
            {
                cv::Point2d q1(lines[j](0), lines[j](1));
                cv::Point2d q2(lines[j](2), lines[j](3));

                double d1 = fabs(a * q1.x + b * q1.y + c) / sqrt(a * a + b * b);
                double d2 = fabs(a * q2.x + b * q2.y + c) / sqrt(a * a + b * b);
                if (d1 <= connect_d_thr && d2 <= connect_d_thr)
                {
                    double err = (d1 + d2) / 2;
                    cv::Vec4f combined_l;
                    bool connectable = connect_line(lines[i], lines[j], m_param.min_connect_gap, m_param.max_connect_gap, combined_l);
                    if (connectable) n_connectable++;

                    m.j.push_back((int)j);
                    m.err.push_back(err);
                    m.connectable.push_back(connectable);
                    m.combined_l.push_back(combined_l);

                    if (connectable && (m.best_err < 0 || err < m.best_err))
                    {
                        m.best_j = (int)j;
                        m.best_err = err;
                        m.best_l = combined_l;
                    }
                }
            }
            if (n_connectable > 0) candidates.push_back(m);
        }
        if (candidates.empty()) return;

        // do line grouping
        std::vector<int> lines_state;
        lines_state.resize(lines.size());
        for (size_t i = 0; i < lines.size(); i++) lines_state[i] = 1;	// 0: remove, 1: keep

        while (1)
        {
            // select best matching candidate
            int best_k = -1;
            double best_err;
            for (size_t k = 0; k < candidates.size(); k++)
            {
                if (lines_state[candidates[k].i] == 0) continue;
                if (candidates[k].best_err < 0) continue;

                if (best_k < 0 || candidates[k].best_err < best_err)
                {
                    best_k = (int)k;
                    best_err = candidates[k].best_err;
                }
            }
            if (best_k < 0) break;

            // connect best match
            match best_m = candidates[best_k];
            lines[best_m.i] = best_m.best_l;
            lines_state[best_m.best_j] = 0;          // remove matched line

            // updated candidates
            for (size_t k = 0; k < candidates.size(); k++)
            {
                match& m = candidates[k];
                int i = m.i;
                if (lines_state[i] == 0) continue;

                bool need_update = false;
                if (k == best_k) need_update = true;
                for (size_t ji = 0; ji < m.j.size(); ji++)
                {
                    int j = m.j[ji];
                    if (j == best_m.i || j == best_m.best_j) need_update = true;
                }
                if (!need_update) continue;

                m.best_j = -1;
                m.best_err = -1;

                cv::Point2d p1(lines[i](0), lines[i](1));
                cv::Point2d p2(lines[i](2), lines[i](3));
                double a, b, c;
                line_2pt(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

                for (size_t ji = 0; ji < m.j.size(); ji++)
                {
                    int j = m.j[ji];
                    if (lines_state[j] == 0) continue;

                    cv::Point2d q1(lines[j](0), lines[j](1));
                    cv::Point2d q2(lines[j](2), lines[j](3));

                    double d1 = fabs(a * q1.x + b * q1.y + c) / sqrt(a * a + b * b);
                    double d2 = fabs(a * q2.x + b * q2.y + c) / sqrt(a * a + b * b);
                    if (d1 <= connect_d_thr && d2 <= connect_d_thr)
                    {
                        double err = (d1 + d2) / 2;
                        cv::Vec4f combined_l;
                        bool connectable = connect_line(lines[i], lines[j], m_param.min_connect_gap, m_param.max_connect_gap, combined_l);

                        m.err[ji] = err;
                        m.connectable[ji] = connectable;
                        m.combined_l[ji] = combined_l;

                        if (connectable && (m.best_err < 0 || err < m.best_err))
                        {
                            m.best_j = j;
                            m.best_err = err;
                            m.best_l = combined_l;
                        }
                    }
                }
            } // updated candidates
        }

        std::vector<cv::Vec4f> lines_tmp = lines;
        lines.clear();
        for (size_t i = 0; i < lines_tmp.size(); i++)
        {
            if (lines_state[i] == 1) lines.push_back(lines_tmp[i]);
        }
    }


    void RoadTheta::remove_duplicated_lines(std::vector<cv::Vec4f>& lines)
    {
        // compute line length
        std::vector<double> len2;
        for (size_t i = 0; i < lines.size(); i++)
        {
            double x1 = lines[i](0);
            double y1 = lines[i](1);
            double x2 = lines[i](2);
            double y2 = lines[i](3);
            double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

            len2.push_back(d2);
        }

        // insertion sorting in descending order w.r.t. line length
        for (size_t i = 0; i < lines.size(); i++)
        {
            int k = (int)i;
            while (k > 0 && len2[k] > len2[k - 1])
            {
                double d = len2[k - 1];
                len2[k - 1] = len2[k];
                len2[k] = d;

                cv::Vec4f l = lines[k - 1];
                lines[k - 1] = lines[k];
                lines[k] = l;
                k--;
            }
        }

        // grouping
        std::vector<int> lines_state;
        lines_state.resize(lines.size());
        for (size_t i = 0; i < lines.size(); i++) lines_state[i] = 2;	// 0: remove, 1: keep, 2: unchecked

        double ang_thr = DEG2RAD(m_param.duplication_ang_thr);
        double dist_thr = m_param.duplication_dist_thr * m_param.image_scale_modifier;
        for (size_t i = 0; i < lines.size(); i++)
        {
            if (lines_state[i] != 2) continue;

            cv::Point2d p1(lines[i](0), lines[i](1));
            cv::Point2d p2(lines[i](2), lines[i](3));
            double a, b, c;
            line_2pt(lines[i](0), lines[i](1), lines[i](2), lines[i](3), a, b, c);

            lines_state[i] = 1;
            for (size_t j = i + 1; j < lines.size(); j++)
            {
                if (lines_state[j] != 2) continue;

                cv::Point2d q1(lines[j](0), lines[j](1));
                cv::Point2d q2(lines[j](2), lines[j](3));

                // check distance
                double d1 = fabs(a * q1.x + b * q1.y + c) / sqrt(a * a + b * b);
                double d2 = fabs(a * q2.x + b * q2.y + c) / sqrt(a * a + b * b);
                if (d1 > dist_thr || d2 > dist_thr) continue;

                // check angles
                cv::Point2d v1 = q1 - p1;
                cv::Point2d v2 = p2 - p1;
                double theta1 = acos(v1.ddot(v2) / (norm(v1) * norm(v2)));

                v1 = q2 - p1;
                v2 = p2 - p1;
                double theta2 = acos(v1.ddot(v2) / (norm(v1) * norm(v2)));

                v1 = q1 - p2;
                v2 = p1 - p2;
                double theta3 = acos(v1.ddot(v2) / (norm(v1) * norm(v2)));

                v1 = q2 - p2;
                v2 = p1 - p2;
                double theta4 = acos(v1.ddot(v2) / (norm(v1) * norm(v2)));

                int n_exteriors = 0;
                if (theta1 > ang_thr) n_exteriors++;
                if (theta2 > ang_thr) n_exteriors++;
                if (theta3 > ang_thr) n_exteriors++;
                if (theta4 > ang_thr) n_exteriors++;

                if (n_exteriors == 0)
                {
                    lines_state[j] = 0;
                }
            }
        }

        std::vector<cv::Vec4f> lines_tmp = lines;
        lines.clear();
        for (size_t i = 0; i < lines_tmp.size(); i++)
        {
            if (lines_state[i] == 1) lines.push_back(lines_tmp[i]);
        }
    }

    void RoadTheta::grouping_test(cv::Mat& frame)
    {
        // backup original frame
        cv::Mat frame_original = frame.clone();

        // convert to gray
        Mat gray;
        if (frame.channels() > 1)
            cvtColor(frame, gray, COLOR_BGR2GRAY);
        else
            gray = frame;

        // detect line segments
        std::vector<cv::Vec4f> lines;
        detect_lines_fld(gray, lines, (int)(m_param.line_length_min * m_param.image_scale_modifier));
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4f v = lines[i];
            cv::line(frame, cv::Point2f(v(0), v(1)), cv::Point2f(v(2), v(3)), m_param.color_line_inlier, 1);
        }

        // grouping
        if (m_param.connect_segmented_lines) connect_segmented_lines(lines);
        if (m_param.group_duplicated_lines) remove_duplicated_lines(lines);
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4f v = lines[i];
            cv::line(frame_original, cv::Point2f(v(0), v(1)), cv::Point2f(v(2), v(3)), m_param.color_line_inlier, 1);
        }

        hconcat(frame, frame_original, frame);
    }


} // End of 'dg'

