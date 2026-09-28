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
        // ------------------------------------------------------------
        // Physical parameters
        // ------------------------------------------------------------

        mass_ =
            this->declare_parameter<double>("mass", 1.0);

        gravity_ =
            this->declare_parameter<double>("gravity", 9.81);

        dt_ =
            this->declare_parameter<double>("dt", 0.1);


        // ------------------------------------------------------------
        // KF matrices
        // ------------------------------------------------------------

        Eigen::Matrix<double, 2, 2> A =
            declare_matrix2x2(
                "A",
                {
                    1.0, dt_,
                    0.0, 1.0
                });

        // Physical thrust-to-acceleration input matrix:
        //
        // B = [ 0      ]
        //     [ dt / m ]
        //
        Eigen::Matrix<double, 2, 1> B;

        B << 0.0,
             dt_ / mass_;


        Eigen::Matrix<double, 2, 2> C =
            declare_matrix2x2(
                "C",
                {
                    1.0, 0.0,
                    0.0, 1.0
                });

        Eigen::Matrix<double, 2, 2> Q =
            declare_matrix2x2(
                "Q",
                {
                    0.001, 0.0,
                    0.0, 0.001
                });

        Eigen::Matrix<double, 2, 2> R =
            declare_matrix2x2(
                "R",
                {
                    0.05, 0.0,
                    0.0, 0.05
                });

        Eigen::Matrix<double, 2, 2> P0 =
            declare_matrix2x2(
                "P0",
                {
                    1.0, 0.0,
                    0.0, 1.0
                });

        Eigen::Matrix<double, 2, 1> x0 =
            declare_vector2(
                "x0",
                {
                    4.2,
                    0.7
                });


        // ------------------------------------------------------------
        // Construct Kalman filter
        // ------------------------------------------------------------

        kf_ =
            std::make_unique<KalmanFilter>(
                A,
                B,
                C,
                Q,
                R,
                P0,
                x0);


        // Assume hover thrust until the first control message arrives.
        //
        // This avoids starting the KF with an artificial zero-thrust
        // prediction when the physical plant is expected to hover.
        u_ = mass_ * gravity_;


        // ------------------------------------------------------------
        // Measurement subscriber
        // ------------------------------------------------------------

        measurement_sub_ =
            this->create_subscription<
                quadrotor_gnc::msg::Measurement>(
                "measurement",
                10,
                std::bind(
                    &EstimatorNode::measurement_callback,
                    this,
                    std::placeholders::_1));


        // ------------------------------------------------------------
        // Control subscriber
        // ------------------------------------------------------------

        control_sub_ =
            this->create_subscription<
                quadrotor_gnc::msg::Control>(
                "control",
                10,
                std::bind(
                    &EstimatorNode::control_callback,
                    this,
                    std::placeholders::_1));


        // ------------------------------------------------------------
        // Estimated-state publisher
        // ------------------------------------------------------------

        estimate_pub_ =
            this->create_publisher<
                quadrotor_gnc::msg::EstimatedState>(
                "estimated_state",
                10);


        // ------------------------------------------------------------
        // Logging
        // ------------------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Estimator node initialized!");

        RCLCPP_INFO(
            this->get_logger(),
            "Mass = %.3f kg, Gravity = %.3f m/s^2, dt = %.3f s",
            mass_,
            gravity_,
            dt_);
    }


private:

    // ------------------------------------------------------------
    // Helper: parameter -> 2x2 Eigen matrix
    // ------------------------------------------------------------

    Eigen::Matrix<double, 2, 2> declare_matrix2x2(
        const std::string & name,
        const std::vector<double> & default_vals)
    {
        this->declare_parameter<std::vector<double>>(
            name,
            default_vals);

        std::vector<double> v =
            this->get_parameter(name).as_double_array();

        if (v.size() != 4)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter '%s' must have exactly 4 elements "
                "(2x2, row-major), got %zu. Falling back to default.",
                name.c_str(),
                v.size());

            v = default_vals;
        }

        Eigen::Matrix<double, 2, 2> M;

        M << v[0], v[1],
             v[2], v[3];

        return M;
    }


    // ------------------------------------------------------------
    // Helper: parameter -> 2x1 Eigen vector
    // ------------------------------------------------------------

    Eigen::Matrix<double, 2, 1> declare_vector2(
        const std::string & name,
        const std::vector<double> & default_vals)
    {
        this->declare_parameter<std::vector<double>>(
            name,
            default_vals);

        std::vector<double> v =
            this->get_parameter(name).as_double_array();

        if (v.size() != 2)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter '%s' must have exactly 2 elements, "
                "got %zu. Falling back to default.",
                name.c_str(),
                v.size());

            v = default_vals;
        }

        Eigen::Matrix<double, 2, 1> vec;

        vec << v[0],
               v[1];

        return vec;
    }


    // ------------------------------------------------------------
    // Receives actual thrust command from controller.
    //
    // u_ is raw thrust T [N].
    // ------------------------------------------------------------

    void control_callback(
        const quadrotor_gnc::msg::Control & msg)
    {
        u_ = msg.control;
    }


    // ------------------------------------------------------------
    // Measurement callback
    // ------------------------------------------------------------

    void measurement_callback(
        const quadrotor_gnc::msg::Measurement & msg)
    {
        Eigen::Matrix<double, 2, 1> z;

        z << msg.position,
             msg.velocity;


        // --------------------------------------------------------
        // Convert actual thrust into equivalent deviation input.
        //
        // Plant:
        //
        // x(k+1) = A*x(k) + B*T + g_vec + w
        //
        // Existing KF:
        //
        // x(k+1) = A*x(k) + B*u_KF + w
        //
        // Therefore:
        //
        // u_KF = T - m*g
        //
        // --------------------------------------------------------

        double u_kf =
            u_ - mass_ * gravity_;


        // Prediction
        kf_->predict(u_kf);


        // Measurement correction
        kf_->update(z);


        // Get state estimate
        const auto & x_hat =
            kf_->getStateEstimate();


        // --------------------------------------------------------
        // Publish estimated state
        // --------------------------------------------------------

        auto estimate =
            quadrotor_gnc::msg::EstimatedState();

        estimate.position = x_hat(0);
        estimate.velocity = x_hat(1);

        estimate_pub_->publish(estimate);


        // --------------------------------------------------------
        // Logging
        // --------------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Estimate: position = %.3f, velocity = %.3f | "
            "Thrust = %.3f N | KF input = %.3f",
            estimate.position,
            estimate.velocity,
            u_,
            u_kf);
    }


    // ------------------------------------------------------------
    // ROS2 interfaces
    // ------------------------------------------------------------

    rclcpp::Subscription<
        quadrotor_gnc::msg::Measurement
    >::SharedPtr measurement_sub_;

    rclcpp::Subscription<
        quadrotor_gnc::msg::Control
    >::SharedPtr control_sub_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::EstimatedState
    >::SharedPtr estimate_pub_;


    // ------------------------------------------------------------
    // Kalman filter
    // ------------------------------------------------------------

    std::unique_ptr<KalmanFilter> kf_;


    // ------------------------------------------------------------
    // Physical parameters
    // ------------------------------------------------------------

    double mass_;
    double gravity_;
    double dt_;


    // Most recently received actual thrust [N]
    double u_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<EstimatorNode>());

    rclcpp::shutdown();

    return 0;
}