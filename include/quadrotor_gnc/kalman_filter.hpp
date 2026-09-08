#pragma once

#include <Eigen/Dense>
namespace quadrotor_gnc{
class KalmanFilter
{
public:

    KalmanFilter(
        const Eigen::Matrix<double, 2, 2>& A,
        const Eigen::Matrix<double, 2, 1>& B,
        const Eigen::Matrix<double, 2, 2>& C,
        const Eigen::Matrix<double, 2, 2>& Q,
        const Eigen::Matrix<double, 2, 2>& R,
        const Eigen::Matrix<double, 2, 2>& P0,
        const Eigen::Matrix<double, 2, 1>& x0);

    void predict(double u);

    void update(
        const Eigen::Matrix<double, 2, 1>& z);

    const Eigen::Matrix<double, 2, 1>&
    getStateEstimate() const;

    const Eigen::Matrix<double, 2, 2>&
    getCovariance() const;

private:

    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 2> C;

    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 2, 2> R;
    Eigen::Matrix<double, 2, 2> P;

    Eigen::Matrix<double, 2, 1> x;

    Eigen::Matrix<double, 2, 2> I =
        Eigen::Matrix<double, 2, 2>::Identity();
};
}