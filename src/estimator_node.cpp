#include "rclcpp/rclcpp.hpp"

#include "quadrotor_gnc/msg/measurement.hpp"
#include "quadrotor_gnc/msg/estimated_state.hpp"
#include "quadrotor_gnc/msg/control.hpp"

#include "quadrotor_gnc/kalman_filter.hpp"

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>
using quadrotor_gnc::KalmanFilter;

class EstimatorNode : public rclcpp::Node
{
public:

    EstimatorNode()
        : Node("estimator_node")
    {
        // All KF matrices are now parameters instead of hardcoded values.
        // Declared as flat, row-major double arrays since rclcpp parameters
        // don't support matrix types directly.
        Eigen::Matrix<double, 2, 2> A =
            declare_matrix2x2("A", {1.0, 0.1, 0.0, 1.0});

        Eigen::Matrix<double, 2, 1> B =
            declare_vector2("B", {0.0, 0.1});

        Eigen::Matrix<double, 2, 2> C =
            declare_matrix2x2("C", {1.0, 0.0, 0.0, 1.0});

        Eigen::Matrix<double, 2, 2> Q =
            declare_matrix2x2("Q", {0.001, 0.0, 0.0, 0.001});

        Eigen::Matrix<double, 2, 2> R =
            declare_matrix2x2("R", {0.05, 0.0, 0.0, 0.05});

        Eigen::Matrix<double, 2, 2> P0 =
            declare_matrix2x2("P0", {1.0, 0.0, 0.0, 1.0});

        Eigen::Matrix<double, 2, 1> x0 =
            declare_vector2("x0", {4.2, 0.7});

        // declare_parameter/get_parameter need a fully-constructed Node,
        // so kf_ can't be built in the member-initializer list anymore.
        // Construct it here in the body instead, once all matrices are known.
        kf_ = std::make_unique<KalmanFilter>(A, B, C, Q, R, P0, x0);

        measurement_sub_ =
            this->create_subscription<quadrotor_gnc::msg::Measurement>(
                "measurement",
                10,
                std::bind(
                    &EstimatorNode::measurement_callback,
                    this,
                    std::placeholders::_1));

        control_sub_ =
            this->create_subscription<quadrotor_gnc::msg::Control>(
                "control",
                10,
                std::bind(
                    &EstimatorNode::control_callback,
                    this,
                    std::placeholders::_1));

        estimate_pub_ =
            this->create_publisher<quadrotor_gnc::msg::EstimatedState>(
                "estimated_state",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Estimator node initialized!");
    }

private:

    // Declares a parameter as a flat 4-element row-major double array and
    // returns it as a 2x2 Eigen matrix. Falls back to `default_vals` (and
    // logs an error) if the parameter doesn't have exactly 4 elements.
    Eigen::Matrix<double, 2, 2> declare_matrix2x2(
        const std::string & name,
        const std::vector<double> & default_vals)
    {
        this->declare_parameter<std::vector<double>>(name, default_vals);

        std::vector<double> v =
            this->get_parameter(name).as_double_array();

        if (v.size() != 4)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter '%s' must have exactly 4 elements (2x2, "
                "row-major), got %zu. Falling back to default.",
                name.c_str(),
                v.size());

            v = default_vals;
        }

        Eigen::Matrix<double, 2, 2> M;
        M << v[0], v[1],
             v[2], v[3];

        return M;
    }

    // Declares a parameter as a flat 2-element double array and returns
    // it as a 2x1 Eigen vector. Falls back to `default_vals` (and logs
    // an error) if the parameter doesn't have exactly 2 elements.
    Eigen::Matrix<double, 2, 1> declare_vector2(
        const std::string & name,
        const std::vector<double> & default_vals)
    {
        this->declare_parameter<std::vector<double>>(name, default_vals);

        std::vector<double> v =
            this->get_parameter(name).as_double_array();

        if (v.size() != 2)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter '%s' must have exactly 2 elements, got %zu. "
                "Falling back to default.",
                name.c_str(),
                v.size());

            v = default_vals;
        }

        Eigen::Matrix<double, 2, 1> vec;
        vec << v[0], v[1];

        return vec;
    }

    void control_callback(
        const quadrotor_gnc::msg::Control & msg)
    {
        u_ = msg.control;
    }

    void measurement_callback(
        const quadrotor_gnc::msg::Measurement & msg)
    {
        Eigen::Matrix<double, 2, 1> z;

        z << msg.position,
             msg.velocity;

        // Prediction using the most recent control input.
        kf_->predict(u_);

        // Measurement correction.
        kf_->update(z);

        const auto & x_hat =
            kf_->getStateEstimate();

        auto estimate =
            quadrotor_gnc::msg::EstimatedState();

        estimate.position = x_hat(0);
        estimate.velocity = x_hat(1);

        estimate_pub_->publish(estimate);

        RCLCPP_INFO(
            this->get_logger(),
            "Estimate: position = %.3f, velocity = %.3f",
            estimate.position,
            estimate.velocity);
    }

    rclcpp::Subscription<
        quadrotor_gnc::msg::Measurement
    >::SharedPtr measurement_sub_;

    rclcpp::Subscription<
        quadrotor_gnc::msg::Control
    >::SharedPtr control_sub_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::EstimatedState
    >::SharedPtr estimate_pub_;

    std::unique_ptr<KalmanFilter> kf_;

    double u_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<EstimatorNode>());

    rclcpp::shutdown();

    return 0;
}