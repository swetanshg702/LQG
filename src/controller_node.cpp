#include "rclcpp/rclcpp.hpp"

#include "quadrotor_gnc/msg/estimated_state.hpp"
#include "quadrotor_gnc/msg/control.hpp"

#include "quadrotor_gnc/lqr_controller.hpp"

#include <Eigen/Dense>
#include <memory>
#include <vector>

class ControllerNode : public rclcpp::Node
{
public:

    ControllerNode()
        : Node("controller_node")
    {
        // Declare the LQR gain as a parameter, defaulting to the value
        // that used to be hardcoded. Can now be overridden from a launch
        // file's params YAML or from the CLI without recompiling.
        this->declare_parameter<std::vector<double>>(
            "K_gain",
            {2.76, 2.51});

        std::vector<double> k_gain =
            this->get_parameter("K_gain").as_double_array();

        if (k_gain.size() != 2)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "K_gain parameter must have exactly 2 elements, got %zu. "
                "Falling back to default.",
                k_gain.size());

            k_gain = {2.76, 2.51};
        }

        Eigen::Matrix<double, 1, 2> K;
        K << k_gain[0], k_gain[1];

        // declare_parameter/get_parameter need a fully-constructed Node,
        // so lqr_ can't be built in the member-initializer list anymore.
        // Construct it here in the body instead, once K is known.
        lqr_ = std::make_unique<LQRController>(K);

        state_sub_ =
            this->create_subscription<quadrotor_gnc::msg::EstimatedState>(
                "estimated_state",
                10,
                std::bind(
                    &ControllerNode::state_callback,
                    this,
                    std::placeholders::_1));

        control_pub_ =
            this->create_publisher<quadrotor_gnc::msg::Control>(
                "control",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Controller node initialized! K_gain = [%.3f, %.3f]",
            k_gain[0],
            k_gain[1]);
    }

private:

    void state_callback(
        const quadrotor_gnc::msg::EstimatedState & msg)
    {
        Eigen::Matrix<double, 2, 1> x_hat;

        x_hat << msg.position,
                 msg.velocity;

        Eigen::Matrix<double, 2, 1> x_ref;

        x_ref << 0.0,
                 0.0;

        double u =
            lqr_->computeControl(
                x_hat,
                x_ref);

        auto control =
            quadrotor_gnc::msg::Control();

        control.control = u;

        control_pub_->publish(control);

        RCLCPP_INFO(
            this->get_logger(),
            "Control: %.3f",
            u);
    }

    rclcpp::Subscription<
        quadrotor_gnc::msg::EstimatedState
    >::SharedPtr state_sub_;

    rclcpp::Publisher<
        quadrotor_gnc::msg::Control
    >::SharedPtr control_pub_;

    std::unique_ptr<LQRController> lqr_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}