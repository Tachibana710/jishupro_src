#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"
#include "my_msgs/msg/target_current.hpp"

class ControlNode : public rclcpp::Node
{
    public:
        ControlNode() : Node("control_node")
        {
            sub_ = this->create_subscription<my_msgs::msg::RobotState>(
                "/robot_state", 10, std::bind(&ControlNode::callback, this, std::placeholders::_1));
            pub_ = this->create_publisher<my_msgs::msg::TargetCurrent>("/target_current", 10);

            RCLCPP_INFO(this->get_logger(), "control_node has started.");
        }

        void callback(const my_msgs::msg::RobotState::SharedPtr msg)
        {
            my_msgs::msg::TargetCurrent command;
            constexpr double pi = 3.14159265358979323846;
            constexpr double factor = 2 * pi / 8192 / 36;
            constexpr double shoulder_gear_ratio = 44.0 / 20.0;
            constexpr double wheel_radius = 0.1;
            constexpr double shoulder_kp = 0.1;
            constexpr double shoulder_kd = 0.01;
            constexpr double elbow_kp = 0.1;
            constexpr double elbow_kd = 0.01;
            constexpr double wheel_kp = 0.1;
            constexpr double wheel_kd = 0.01;
            constexpr double y_kp = 0.1;
            constexpr double y_kd = 0.01;

            double shoulder_angle = msg->shoulder_angle;
            double shoulder_omega = msg->shoulder_omega;
            double elbow_angle = msg->elbow_angle;
            double elbow_omega = msg->elbow_omega;
            double y = msg->y;
            double y_dot = msg->y_dot;

            double shoulder_desired_angle = 0;
            double elbow_desired_angle = 0;
            double y_desired = 0;

            double shoulder_desired_omega = 0;
            double elbow_desired_omega = 0;
            double y_desired_dot = 0;

            double shoulder_error = shoulder_desired_angle - shoulder_angle;
            double elbow_error = elbow_desired_angle - elbow_angle;
            double y_error = y_desired - y;

            double shoulder_error_dot = shoulder_desired_omega - shoulder_omega;
            double elbow_error_dot = elbow_desired_omega - elbow_omega;
            double y_error_dot = y_desired_dot - y_dot;

            constexpr int shoulder_idx = 1;
            constexpr int elbow_idx = 0;
            constexpr int wheel_idx = 2;

            command.target_current[shoulder_idx] = shoulder_kp * shoulder_error + shoulder_kd * shoulder_error_dot;
            command.target_current[elbow_idx] = elbow_kp * elbow_error + elbow_kd * elbow_error_dot;
            command.target_current[wheel_idx] = wheel_kp * y_error + wheel_kd * y_error_dot;

            pub_->publish(command);
            // RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        }

    private:
        rclcpp::Subscription<my_msgs::msg::RobotState>::SharedPtr sub_;
        rclcpp::Publisher<my_msgs::msg::TargetCurrent>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}