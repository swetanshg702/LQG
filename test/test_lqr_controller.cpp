#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "quadrotor_gnc/lqr_controller.hpp"

// u = -K(x - x_ref), computed by hand for each case below.

TEST(LQRControllerTest, ZeroErrorGivesZeroControl)
{
    Eigen::Matrix<double, 1, 2> K;
    K << 1.0, 2.0;

    LQRController lqr(K);

    Eigen::Matrix<double, 2, 1> x;
    x << 3.0, 5.0;

    Eigen::Matrix<double, 2, 1> x_ref;
    x_ref << 3.0, 5.0;

    // x == x_ref, so the error is zero regardless of K.
    double u = lqr.computeControl(x, x_ref);

    EXPECT_NEAR(u, 0.0, 1e-9);
}

TEST(LQRControllerTest, GeneralCaseMatchesHandComputation)
{
    Eigen::Matrix<double, 1, 2> K;
    K << 1.0, 2.0;

    LQRController lqr(K);

    Eigen::Matrix<double, 2, 1> x;
    x << 3.0, 5.0;

    Eigen::Matrix<double, 2, 1> x_ref;
    x_ref << 1.0, 1.0;

    // error = x - x_ref = [2, 4]
    // u = -K * error = -(1*2 + 2*4) = -10
    double expected_u = -10.0;

    double u = lqr.computeControl(x, x_ref);

    EXPECT_NEAR(u, expected_u, 1e-9);
}

TEST(LQRControllerTest, NegativeErrorFlipsSign)
{
    Eigen::Matrix<double, 1, 2> K;
    K << 2.0, 0.5;

    LQRController lqr(K);

    Eigen::Matrix<double, 2, 1> x;
    x << -1.0, -2.0;

    Eigen::Matrix<double, 2, 1> x_ref;
    x_ref << 1.0, 2.0;

    // error = x - x_ref = [-2, -4]
    // u = -K * error = -(2*(-2) + 0.5*(-4)) = -(-4 - 2) = 6
    double expected_u = 6.0;

    double u = lqr.computeControl(x, x_ref);

    EXPECT_NEAR(u, expected_u, 1e-9);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}