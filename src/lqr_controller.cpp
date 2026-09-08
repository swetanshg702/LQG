#include "quadrotor_gnc/lqr_controller.hpp"

LQRController::LQRController(
    const Eigen::Matrix<double, 1, 2>& K)
{
    this->K = K;
}

double LQRController::computeControl(
    const Eigen::Matrix<double, 2, 1>& x,
    const Eigen::Matrix<double, 2, 1>& x_ref) const
{
    double u = (-K * (x - x_ref)).value();

    return u;
}