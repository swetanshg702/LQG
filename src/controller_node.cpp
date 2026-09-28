#include "rclcpp/rclcpp.hpp"

#include "quadrotor_gnc/msg/estimated_state.hpp"
#include "quadrotor_gnc/msg/control.hpp"

#include "quadrotor_gnc/lqr_controller.hpp"

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <algorithm>

class ControllerNode : public rclcpp::Node
{
public:

    ControllerNode()
        : Node("controller_node")
    {
        // ------------------------------------------------------------
        // Physical parameters
        // ------------------------------------------------------------

        mass_ =
            this->declare_parameter<double>("mass", 1.0);

        gravity_ =
            this->declare_parameter<double>("gravity", 9.81);

        max_thrust_ =
            this->declare_parameter<double>("max_thrust", 20.0);

        // Hover / trim thrust:
        //
        // T_trim = m * g
        //
        trim_thrust_ =
            mass_ * gravity_;


        // ------------------------------------------------------------
        // LQR gain
        // ------------------------------------------------------------

        this->declare_parameter<std::vector<double>>(
            "K_gain",
            {2.76, 2.51});

        std::vector<double> k_gain =
            this->get_parameter("K_gain").as_double_array();

        if (k_gain.size() != 2)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "K_gain parameter must have exactly 2 elements, "
                "got %zu. Falling back to default.",
                k_gain.size());

            k_gain = {2.76, 2.51};
        }

        Eigen::Matrix<double, 1, 2> K;

        K << k_gain[0],
             k_gain[1];


        // Construct the LQR controller.
        lqr_ =
            std::make_unique<LQRController>(K);


        // ------------------------------------------------------------
        // Subscriber: estimated state
        // ------------------------------------------------------------

        state_sub_ =
            this->create_subscription<
                quadrotor_gnc::msg::EstimatedState>(
                "estimated_state",
                10,
                std::bind(
                    &ControllerNode::state_callback,
                    this,
                    std::placeholders::_1));


        // ------------------------------------------------------------
        // Publisher: thrust command
        // ------------------------------------------------------------

        control_pub_ =
            this->create_publisher<
                quadrotor_gnc::msg::Control>(
                "control",
                10);


        // ------------------------------------------------------------
        // Logging
        // ------------------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Controller node initialized!");

        RCLCPP_INFO(
            this->get_logger(),
            "Mass = %.3f kg, Gravity = %.3f m/s^2",
            mass_,
            gravity_);

        RCLCPP_INFO(
            this->get_logger(),
            "Trim thrust = %.3f N",
            trim_thrust_);

        RCLCPP_INFO(
            this->get_logger(),
            "Max thrust = %.3f N",
            max_thrust_);

        RCLCPP_INFO(
            this->get_logger(),
            "K_gain = [%.3f, %.3f]",
            k_gain[0],
            k_gain[1]);
    }


private:

    void state_callback(
        const quadrotor_gnc::msg::EstimatedState & msg)
    {
        // ------------------------------------------------------------
        // Estimated state
        // ------------------------------------------------------------

        Eigen::Matrix<double, 2, 1> x_hat;

        x_hat << msg.position,
                 msg.velocity;


        // ------------------------------------------------------------
        // Reference state
        // ------------------------------------------------------------

        Eigen::Matrix<double, 2, 1> x_ref;

        x_ref << 0.0,
                 0.0;


        // ------------------------------------------------------------
        // LQR feedback term
        // ------------------------------------------------------------

        double u_fb =
            lqr_->computeControl(
                x_hat,
                x_ref);


        // ------------------------------------------------------------
        // Add trim/feedforward thrust
        //
        // T_raw = T_trim + u_fb
        //       = m*g + u_fb
        // ------------------------------------------------------------

        double thrust_raw =
            trim_thrust_ + u_fb;


        // ------------------------------------------------------------
        // Actuator saturation
        //
        // 0 <= T_cmd <= T_max
        // ------------------------------------------------------------

        double thrust_cmd =
            std::clamp(
                thrust_raw,
                0.0,
                max_thrust_);


        // ------------------------------------------------------------
        // Publish actual actuator command
        // ------------------------------------------------------------

        auto control =
            quadrotor_gnc::msg::Control();

        control.control =
            thrust_cmd;

        control_pub_->publish(control);


        // ------------------------------------------------------------
        // Logging
        // ------------------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Position: %.3f | Velocity: %.3f | "
            "LQR feedback: %.3f | Trim: %.3f | "
            "Raw thrust: %.3f N | Commanded thrust: %.3f N",
            msg.position,
            msg.velocity,
            u_fb,
            trim_thrust_,
            thrust_raw,
            thrust_cmd);
    }


    // ------------------------------------------------------------
    // ROS2 interfaces
    // ------------------------------------------------------------

    rclcpp::Subscription<
        quadrotor_gnc::msg::EstimatedState
    >::SharedPtr state_sub_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::Control
    >::SharedPtr control_pub_;


    // ------------------------------------------------------------
    // LQR controller
    // ------------------------------------------------------------

    std::unique_ptr<LQRController> lqr_;


    // ------------------------------------------------------------
    // Physical parameters
    // ------------------------------------------------------------

    double mass_;
    double gravity_;
    double trim_thrust_;
    double max_thrust_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}