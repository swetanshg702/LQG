#include "rclcpp/rclcpp.hpp"
#include "quadrotor_gnc/msg/control.hpp"
#include "quadrotor_gnc/msg/measurement.hpp"
#include "quadrotor_gnc/msg/estimated_state.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class PlantNode : public rclcpp::Node
{
public:

    PlantNode()
        : Node("plant_node")
        , dt_(0.1)
        , u_(0.0)
    {
        // Physical parameters
        mass_ = this->declare_parameter<double>("mass", 1.0);
        gravity_ = this->declare_parameter<double>("gravity", 9.81);

        // System matrix A
        Eigen::Matrix<double, 2, 2> A =
            declare_matrix2x2(
                "A",
                {
                    1.0, dt_,
                    0.0, 1.0
                });

        // For m*z_ddot = T - m*g:
        //
        // z(k+1)     = z(k) + dt*v(k)
        // v(k+1)     = v(k) + dt*(T/m - g)
        //
        // Therefore:
        //
        // B = [ 0       ]
        //     [ dt / m  ]
        //
        Eigen::Matrix<double, 2, 1> B;
        B << 0.0,
             dt_ / mass_;

        // Constant affine gravity term:
        //
        // g_vec = [ 0            ]
        //         [ -gravity*dt  ]
        //
        Eigen::Matrix<double, 2, 1> g_vec;
        g_vec << 0.0,
                 -gravity_ * dt_;

        // Initial true state
        x_true_ << 5.0,
                   0.0;

        // Process noise covariance
        Eigen::Matrix<double, 2, 2> Q =
            declare_matrix2x2(
                "Q",
                {
                    0.001, 0.0,
                    0.0, 0.001
                });

        // Measurement noise covariance
        Eigen::Matrix<double, 2, 2> R =
            declare_matrix2x2(
                "R",
                {
                    0.05, 0.0,
                    0.0, 0.05
                });

        // Store matrices
        A_ = A;
        B_ = B;
        g_vec_ = g_vec;
        Q_ = Q;
        R_ = R;

        // Cholesky factors:
        // L * L^T = covariance
        Eigen::LLT<Eigen::Matrix2d> llt_Q(Q_);
        L_process_ = llt_Q.matrixL();

        Eigen::LLT<Eigen::Matrix2d> llt_R(R_);
        L_meas_ = llt_R.matrixL();

        // Random number generator
        std::random_device rd;
        gen_.seed(rd());

        // Publisher for measurements
        measurement_pub_ =
            this->create_publisher<quadrotor_gnc::msg::Measurement>(
                "measurement",
                10);

        // Publisher for true state
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
        
        RCLCPP_INFO(
            this->get_logger(),
            "Mass = %.3f kg, Gravity = %.3f m/s^2",
            mass_,
            gravity_);

        RCLCPP_INFO(
            this->get_logger(),
            "Initial thrust = %.3f N",
            u_);
    }

private:

    // Declares a parameter as a flat 4-element row-major double array
    // and returns it as a 2x2 Eigen matrix.
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

    // Receives the latest control input.
    //
    // IMPORTANT:
    // u_ is now interpreted as raw thrust T [N].
    // This callback does NOT advance the plant.
    void control_callback(
        const quadrotor_gnc::msg::Control & msg)
    {
        u_ = msg.control;
    }

    // Draws a 2x1 vector of correlated Gaussian noise.
    Eigen::Vector2d sample_noise(
        const Eigen::Matrix2d & L)
    {
        Eigen::Vector2d z;

        z(0) = standard_normal_(gen_);
        z(1) = standard_normal_(gen_);

        return L * z;
    }

    // Advances the plant by one discrete time step.
    void timer_callback()
    {
        Eigen::Vector2d w =
            sample_noise(L_process_);

        // Affine dynamics:
        //
        // x(k+1) = A*x(k) + B*T(k) + g_vec + w
        //
        // where:
        //
        // T = u_
        //
        x_true_ =
            A_ * x_true_
            + B_ * u_
            + g_vec_
            + w;

        RCLCPP_INFO(
            this->get_logger(),
            "True state: position = %.3f, velocity = %.3f, "
            "thrust = %.3f N",
            x_true_(0),
            x_true_(1),
            u_);

        publish_measurement();
        publish_true_state();
    }

    void publish_measurement()
    {
        Eigen::Vector2d v =
            sample_noise(L_meas_);

        Eigen::Vector2d z_meas =
            x_true_ + v;

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

    // Constant affine gravity term
    Eigen::Matrix<double, 2, 1> g_vec_;

    // Physical parameters
    double mass_;
    double gravity_;
    double dt_;

    // Noise covariances and Cholesky factors
    Eigen::Matrix2d Q_;
    Eigen::Matrix2d R_;
    Eigen::Matrix2d L_process_;
    Eigen::Matrix2d L_meas_;

    // Random number generator
    std::mt19937 gen_;
    std::normal_distribution<double> standard_normal_{0.0, 1.0};

    // True plant state
    Eigen::Matrix<double, 2, 1> x_true_;

    // Most recently received raw thrust command [N]
    double u_;

    // Publishers
    rclcpp::Publisher<
        quadrotor_gnc::msg::Measurement
    >::SharedPtr measurement_pub_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::EstimatedState
    >::SharedPtr truestate_pub_;

    // Control subscriber
    rclcpp::Subscription<
        quadrotor_gnc::msg::Control
    >::SharedPtr control_sub_;

    // Simulation timer
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