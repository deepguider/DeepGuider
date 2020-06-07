/**
 * This file is a part of OpenCX, especially about extended Kalman filter (EKF).
 * - Homepage: https://github.com/sunglok/opencx
 */

/**
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <sunglok@hanmail.net> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Sunglok Choi
 * ----------------------------------------------------------------------------
 */

#ifndef __EKF__
#define __EKF__

#include "opencv2/opencv.hpp"

namespace cx
{
    /**
     * @brief Extended Kalman Filter (EKF)
     *
     * The extended Kalman filter is a nonlinear extension of the Kalman filter which linearizes nonlinear state transition and observation models at the point of the current state.
     * You can make your EKF application by inheriting this class and overriding six functions: transitFunc, transitJacobian, transitNoise, observeFunc, observeJacobian, and observeNoise.
     *
     * @see Extended Kalman Filter (Wikipedia), http://en.wikipedia.org/wiki/Extended_Kalman_filter
     * @see Welch, The Kalman Filter, http://www.cs.unc.edu/~welch/kalman/
     * @see Welch and Bishop, An Introduction to the Kalman Filter, UNC-Chapel Hill, TR95-041, 2006, <a href="http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html">PDF</a>
     */
    class EKF
    {
    public:
        /**
         * The virtual destructor
         */
        virtual ~EKF() { }

        /**
         * Initialize the state variable and covariance
         * @param state_dim The dimension of state variable
         * @return True if successful (false if failed)
         */
        virtual bool initialize(int state_dim, int state_type = CV_64F)
        {
            return initialize(cv::Mat::zeros(state_dim, 1, state_type), cv::Mat::eye(state_dim, state_dim, state_type));
        }

        /**
         * Initialize the state variable and covariance with the given values
         * @param state_vec The given state variable
         * @param state_cov The given state covariance
         * @return True if successful (false if failed)
         */
        virtual bool initialize(cv::InputArray state_vec, cv::InputArray state_cov = cv::noArray())
        {
            CV_DbgAssert(!state_vec.empty());
            state_vec.copyTo(m_state_vec);
            if (m_state_vec.rows < m_state_vec.cols) m_state_vec = m_state_vec.t();
            CV_DbgAssert(m_state_vec.cols == 1);

            int dim = m_state_vec.rows;
            if (!state_cov.empty())
            {
                state_cov.copyTo(m_state_cov);
                CV_DbgAssert(m_state_cov.rows == dim && m_state_cov.cols == dim);
            }
            else m_state_cov = cv::Mat::eye(dim, dim, m_state_vec.type());
            return true;
        }

        /**
         * Predict the state variable and covariance from the given control input
         * @param control The given control input
         * @return True if successful (false if failed)
         */
        virtual bool predict(cv::InputArray control)
        {
            cv::Mat u = control.getMat();
            if (u.rows < u.cols) u = u.t();

            // Predict the state
            cv::Mat F, Q;
            m_state_vec = transitFunc(m_state_vec, u, F, Q);
            m_state_cov = F * m_state_cov * F.t() + Q;

            // Enforce the state covariance symmetric
            m_state_cov = 0.5 * m_state_cov + 0.5 * m_state_cov.t();
            return true;
        }

        /**
         * Correct the state variable and covariance with the given measurement
         * @param measure The given measurement
         * @return True if successful (false if failed)
         */
        virtual bool correct(cv::InputArray measure)
        {
            cv::Mat z = measure.getMat();
            if (z.rows < z.cols) z = z.t();

            // Calculate Kalman gain
            cv::Mat H, R;
            cv::Mat expectation = observeFunc(m_state_vec, z, H, R);
            cv::Mat S = H * m_state_cov * H.t() + R;
            cv::Mat K = m_state_cov * H.t() * S.inv(cv::DecompTypes::DECOMP_SVD);
            cv::Mat innovation = z - expectation;

            // Correct the state
            m_state_vec = m_state_vec + K * innovation;
            cv::Mat I_KH = cv::Mat::eye(m_state_cov.size(), m_state_cov.type()) - K * H;
            //m_state_cov = I_KH * m_state_cov; // The standard form
            m_state_cov = I_KH * m_state_cov * I_KH.t() + K * R * K.t(); // Joseph form

            // Enforce the state covariance symmetric
            m_state_cov = 0.5 * m_state_cov + 0.5 * cv::Mat(m_state_cov.t());
            return true;
        }

        /**
         * Calculate squared <a href="https://en.wikipedia.org/wiki/Mahalanobis_distance">Mahalanobis distance</a> of the given measurement
         * @param measure The given measurement
         * @return The squared Mahalanobis distance
         */
        virtual double checkMeasurement(cv::InputArray measure)
        {
            cv::Mat z = measure.getMat();
            if (z.rows < z.cols) z = z.t();

            cv::Mat H, R;
            cv::Mat expectation = observeFunc(m_state_vec, z, H, R);
            cv::Mat innovation = z - expectation;
            cv::Mat S = H * m_state_cov * H.t();
            cv::Mat mah_dist2 = innovation.t() * S.inv(cv::DecompTypes::DECOMP_SVD) * innovation;
            return cv::sum(mah_dist2)(0);
        }

        /**
         * Assign the state variable with the given value
         * @param state The given state variable
         * @return True if successful (false if failed)
         */
        bool setState(cv::InputArray state)
        {
            CV_DbgAssert(state.size() == m_state_vec.size());
            CV_DbgAssert(state.type() == m_state_vec.type());
            m_state_vec = state.getMat();
            return true;
        }

        /**
         * Get the current state variable
         * @return The state variable
         */
        const cv::Mat getState() { return m_state_vec; }

        /**
         * Assign the state covariance with the given value
         * @param covariance The given state covariance
         * @return True if successful (false if failed)
         */
        bool setStateCov(const cv::InputArray covariance)
        {
            CV_DbgAssert(covariance.size() == m_state_cov.size());
            CV_DbgAssert(covariance.type() == m_state_cov.type());
            m_state_cov = covariance.getMat();
            return true;
        }

        /**
         * Get the current state covariance
         * @return The state covariance
         */
        const cv::Mat getStateCov() { return m_state_cov; }

    protected:
        /**
         * The state transition function, its Jacobian, and noise
         * @param state The state variable
         * @param control The given control input
         * @param jacobian The state transition function's Jacobian (return value)
         * @param noise The state transition noise (return value)
         * @return The predicted state variable
         */
        virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise) = 0;

        /**
         * The state observation function, its Jacobian, and noise
         * @param state The state variable
         * @param measure The given measurement
         * @param jacobian The state observation function's Jacobian (return value)
         * @param noise The state observation noise (return value)
         * @return The expected measurement
         */
        virtual cv::Mat observeFunc(const cv::Mat& state, const cv::Mat& measure, cv::Mat& jacobian, cv::Mat& noise) = 0;

        /** The state variable */
        cv::Mat m_state_vec;

        /** The state covariance */
        cv::Mat m_state_cov;
    }; // End of 'EKF'

} // End of 'cx'

#endif // End of '__EKF__'
