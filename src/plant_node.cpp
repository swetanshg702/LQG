#include "rclcpp/rclcpp.hpp"
#include "quadrotor_gnc/msg/control.hpp"
#include "quadrotor_gnc/msg/measurement.hpp"
#include "quadrotor_gnc/msg/estimated_state.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <random>

using namespace std::chrono_literals;

class PlantNode : public rclcpp::Node
{
public:

    PlantNode()
        : Node("plant_node"),
          u_(0.0)
    {
        // System matrices
        Eigen::Matrix<double, 2, 2> A =
            declare_matrix2x2("A", {1.0, 0.1, 0.0, 1.0});

        Eigen::Matrix<double, 2, 1> B =
            declare_vector2("B", {0.0, 0.1});

        // Initial true state
        x_true_ << 5.0,
                   0.0;

        // Process noise covariance (must match estimator_node's Q for now)
        Eigen::Matrix<double, 2, 2> Q =
            declare_matrix2x2("Q", {0.001, 0.0, 0.0, 0.001});

        // Measurement noise covariance (must match estimator_node's R for now)
        Eigen::Matrix<double, 2, 2> R =
            declare_matrix2x2("R", {0.05, 0.0, 0.0, 0.05});

        // Assign into the members that the rest of the class actually uses
        A_ = A;
        B_ = B;
        Q_ = Q;
        R_ = R;

        // Cholesky factors: L * L^T = covariance, used to correlate
        // independent standard-normal samples into the desired covariance.
        Eigen::LLT<Eigen::Matrix2d> llt_Q(Q_);
        L_process_ = llt_Q.matrixL();

        Eigen::LLT<Eigen::Matrix2d> llt_R(R_);
        L_meas_ = llt_R.matrixL();

        std::random_device rd;
        gen_.seed(rd());

        // Publisher for measurements
        measurement_pub_ =
            this->create_publisher<quadrotor_gnc::msg::Measurement>(
                "measurement",
                10);
        truestate_pub_ =
            this->create_publisher<quadrotor_gnc::msg::EstimatedState>(
                "true_state",
                10);
        // Subscriber for control input
        control_sub_ =
            this->create_subscription<quadrotor_gnc::msg::Control>(
                "control",
                10,
                std::bind(
                    &PlantNode::control_callback,
                    this,
                    std::placeholders::_1));

        // Plant simulation runs every 0.1 s = 10 Hz
        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &PlantNode::timer_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Plant node initialized!");
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


    // Receives the latest control input.
    // IMPORTANT: this callback does NOT advance the plant.
    void control_callback(
        const quadrotor_gnc::msg::Control & msg)
    {
        u_ = msg.control;
    }

    // Draws a 2x1 vector of correlated Gaussian noise with covariance
    // L * L^T, from two independent standard-normal samples.
    Eigen::Vector2d sample_noise(const Eigen::Matrix2d & L)
    {
        Eigen::Vector2d z;
        z(0) = standard_normal_(gen_);
        z(1) = standard_normal_(gen_);
        return L * z;
    }

    // Advances the plant by one discrete time step.
    void timer_callback()
    {
        Eigen::Vector2d w = sample_noise(L_process_);

        // x(k+1) = A*x(k) + B*u(k) + w
        x_true_ = A_ * x_true_ + B_ * u_ + w;

        RCLCPP_INFO(
            this->get_logger(),
            "True state: position = %.3f, velocity = %.3f, control = %.3f",
            x_true_(0),
            x_true_(1),
            u_);

        publish_measurement();
        publish_true_state();
    }

    void publish_measurement()
    {
        Eigen::Vector2d v = sample_noise(L_meas_);
        Eigen::Vector2d z_meas = x_true_ + v;

        auto message =
            quadrotor_gnc::msg::Measurement();

        message.position = z_meas(0);
        message.velocity = z_meas(1);

        measurement_pub_->publish(message);
    }
    void publish_true_state()
    {
        auto message =
            quadrotor_gnc::msg::EstimatedState();

        message.position = x_true_(0);
        message.velocity = x_true_(1);

        truestate_pub_->publish(message);
    }
    // System matrices
    Eigen::Matrix<double, 2, 2> A_;
    Eigen::Matrix<double, 2, 1> B_;

    // Noise covariances and their Cholesky factors
    Eigen::Matrix2d Q_;
    Eigen::Matrix2d R_;
    Eigen::Matrix2d L_process_;
    Eigen::Matrix2d L_meas_;

    // RNG, persists across calls so it isn't reseeded every callback
    std::mt19937 gen_;
    std::normal_distribution<double> standard_normal_{0.0, 1.0};

    // True plant state
    Eigen::Matrix<double, 2, 1> x_true_;

    // Most recently received control input
    double u_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::Measurement
    >::SharedPtr measurement_pub_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::EstimatedState
    >::SharedPtr truestate_pub_;

    rclcpp::Subscription<
        quadrotor_gnc::msg::Control
    >::SharedPtr control_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<PlantNode>());

    rclcpp::shutdown();

    return 0;
}