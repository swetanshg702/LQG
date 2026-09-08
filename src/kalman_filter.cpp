#include "quadrotor_gnc/kalman_filter.hpp"
namespace quadrotor_gnc{
KalmanFilter::KalmanFilter(
    const Eigen::Matrix<double, 2, 2>& A,
    const Eigen::Matrix<double, 2, 1>& B,
    const Eigen::Matrix<double, 2, 2>& C,
    const Eigen::Matrix<double, 2, 2>& Q,
    const Eigen::Matrix<double, 2, 2>& R,
    const Eigen::Matrix<double, 2, 2>& P0,
    const Eigen::Matrix<double, 2, 1>& x0)
{
    this->A = A;
    this->B = B;
    this->C = C;
    this->Q = Q;
    this->R = R;
    this->x = x0;
    this->P = P0;
}

void KalmanFilter::predict(double u)
{
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(
    const Eigen::Matrix<double, 2, 1>& z)
{
    Eigen::Matrix<double, 2, 1> y;
    y = z - C * x;

    Eigen::Matrix<double, 2, 2> S;
    S = C * P * C.transpose() + R;

    Eigen::Matrix<double, 2, 2> K_transpose =
        S.ldlt().solve(C * P.transpose());
 
    Eigen::Matrix<double, 2, 2> K = K_transpose.transpose();
 
    x = x + K * y;
    P = (I - K * C) * P;
}

const Eigen::Matrix<double, 2, 1>&
KalmanFilter::getStateEstimate() const
{
    return x;
}

const Eigen::Matrix<double, 2, 2>&
KalmanFilter::getCovariance() const
{
    return P;
}
}