#pragma once

#include <Eigen/Dense>

class LQRController
{
public:

    LQRController(
        const Eigen::Matrix<double, 1, 2>& K);

    double computeControl(
        const Eigen::Matrix<double, 2, 1>& x,
        const Eigen::Matrix<double, 2, 1>& x_ref) const;

private:

    Eigen::Matrix<double, 1, 2> K;
};