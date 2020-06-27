#ifndef __EKF_LOCALIZER_VARIANTS__
#define __EKF_LOCALIZER_VARIANTS__

#include "localizer/localizer_ekf.hpp"

namespace dg
{

class EKFLocalizerZeroGyro : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            cv::Vec2d control_fake(control.at<double>(0), 0); // Add fake observation
            return EKFLocalizer::transitFunc(state, cv::Mat(control_fake), jacobian, noise);
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerZeroOdom : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            cv::Vec3d control_fake(control.at<double>(0), 0, 0); // Add fake observation
            return EKFLocalizer::transitFunc(state, cv::Mat(control_fake), jacobian, noise);
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerPostOdom : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            cv::Vec3d control_fake(control.at<double>(0), 1e-3, 0); // Add fake observation
            return EKFLocalizer::transitFunc(state, cv::Mat(control_fake), jacobian, noise);
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerSlowGyro : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double rate = 0.5;
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double dt = control.at<double>(0);
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            cv::Mat func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                rate * w);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, -vt * dt * s / 2,
                0, 1,  vt * c, dt * s,  vt * dt * c / 2,
                0, 0, 1, 0, dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, rate);
            cv::Mat W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, rate);
            if (!W.empty()) noise = W * m_noise_motion * W.t();
            return func;
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerHyperTan : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double w_op = 1;
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double dt = control.at<double>(0);
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2), th = w_op * tanh(w / w_op);
            cv::Mat func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                th);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, -vt * dt * s / 2,
                0, 1,  vt * c, dt * s,  vt * dt * c / 2,
                0, 0, 1, 0, dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1 - th * th);
            cv::Mat W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 1 - th * th);
            if (!W.empty()) noise = W * m_noise_motion * W.t();
            return func;
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerZeroRate : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double dt = control.at<double>(0);
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            cv::Mat func = (cv::Mat_<double>(5, 1) <<
                x + vt * c,
                y + vt * s,
                theta + wt,
                v,
                0);
            jacobian = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, -vt * dt * s / 2,
                0, 1,  vt * c, dt * s,  vt * dt * c / 2,
                0, 0, 1, 0, dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 0);
            cv::Mat W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 0);
            if (!W.empty()) noise = W * m_noise_motion * W.t();
            return func;
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerVelModel : public EKFLocalizer
{
protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            const double v = state.at<double>(3), w = state.at<double>(4);
            if (fabs(w) > 0.001)
            {
                // The control input: [ dt ]
                const double dt = control.at<double>(0);
                const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
                const double r = v / w, wt = w * dt;
                const double ct = cos(theta), st = sin(theta);
                const double ctw = cos(theta + wt), stw = sin(theta + wt);
                const double st_stw = st - stw, ct_ctw = ct - ctw;
                cv::Mat func = (cv::Mat_<double>(5, 1) <<
                    x - r * st_stw,
                    y + r * ct_ctw,
                    theta + wt,
                    v,
                    w);
                jacobian = (cv::Mat_<double>(5, 5) <<
                    1, 0, -r * ct_ctw, -st_stw / w,  r / w * st_stw + r * ctw,
                    0, 1, -r * st_stw,  ct_ctw / w, -r / w * ct_ctw + r * stw,
                    0, 0, 1, 0, dt,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1);
                cv::Mat W = (cv::Mat_<double>(5, 2) <<
                   -st_stw / w,  r / w * st_stw + r * ctw,
                    ct_ctw / w, -r / w * ct_ctw + r * stw,
                    0, dt,
                    1, 0,
                    0, 1);
                if (!W.empty()) noise = W * m_noise_motion * W.t();
                return func;
            }
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }
};

class EKFLocalizerFreqPred : public EKFLocalizer
{
protected:
    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        const int predict_number = 10;

        cv::AutoLock lock(m_mutex);
        double interval = 0;
        if (m_time_last_update > 0) interval = time - m_time_last_update;
        if (interval > m_threshold_time)
        {
            for (int i = 0; i < predict_number; i++)
                predict(interval / predict_number);
        }
        if (correct(cv::Vec2d(xy.x, xy.y)))
        {
            m_state_vec.at<double>(2) = cx::trimRad(m_state_vec.at<double>(2));
            m_time_last_update = time;
            return true;
        }
        return false;
    }
};

class EKFLocalizerVTAdjust : public EKFLocalizer
{
public:
    virtual bool applyPosition(const Point2& xy, Timestamp time = -1, double confidence = -1)
    {
        const double ratio = 0.1;
        return EKFLocalizer::applyPosition(xy, time * ratio, confidence);
    }

    virtual Polar2 getVelocity() const
    {
        const double ratio = 0.1;
        Polar2 velocity = EKFLocalizer::getVelocity();
        velocity.lin *= ratio;
        velocity.ang *= ratio;
        return velocity;
    }
};

class EKFLocalizerObsvFunc : public EKFLocalizer
{
public:
    EKFLocalizerObsvFunc() : m_transit_dt(0) { }

protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (control.rows == 1)
        {
            // The control input: [ dt ]
            const double dt = control.at<double>(0);
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            cv::Mat func = (cv::Mat_<double>(5, 1) << x, y, theta, v, w);
            jacobian = cv::Mat::eye(5, 5, state.type());
            cv::Mat W = (cv::Mat_<double>(5, 2) <<
                dt * c, -vt * dt * s / 2,
                dt * s,  vt * dt * c / 2,
                0, dt,
                1, 0,
                0, 1);
            if (!W.empty()) noise = W * m_noise_motion * W.t();
            m_transit_dt = dt;
            return func;
        }
        return EKFLocalizer::transitFunc(state, control, jacobian, noise);
    }

    virtual cv::Mat observeFunc(const cv::Mat& state, const cv::Mat& measure, cv::Mat& jacobian, cv::Mat& noise)
    {
        if (measure.rows == 2)
        {
            // Measurement: [ x_{GPS}, y_{GPS} ]
            const double dt = m_transit_dt;
            const double x = state.at<double>(0), y = state.at<double>(1), theta = state.at<double>(2);
            const double v = state.at<double>(3), w = state.at<double>(4);
            const double vt = v * dt, wt = w * dt;
            const double c = cos(theta + wt / 2), s = sin(theta + wt / 2);
            const double co = cos(theta + wt / 2 + m_offset_gps(1)), so = sin(theta + wt / 2 + m_offset_gps(1));
            cv::Mat func = (cv::Mat_<double>(2, 1) <<
                x + vt * c + m_offset_gps(0) * co,
                y + vt * s + m_offset_gps(0) * so);
            const double J0 = -vt * s - m_offset_gps(0) * so;
            const double J1 =  vt * c + m_offset_gps(0) * co;
            jacobian = (cv::Mat_<double>(2, 5) <<
                1, 0, J0, dt * c, dt / 2 * J0,
                0, 1, J1, dt * s, dt / 2 * J1);
            noise = m_noise_gps;
            return func;
        }
        return EKFLocalizer::observeFunc(state, measure, jacobian, noise);
    }

    double m_transit_dt;
};

} // End of 'dg'

#endif // End of '__EKF_LOCALIZER_VARIANTS__'
