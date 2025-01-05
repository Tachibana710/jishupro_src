#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"

class StatePublisher : public rclcpp::Node
{
    public:
        StatePublisher() : Node("state_publisher")
        {
            sub_ = this->create_subscription<my_msgs::msg::SensorData>(
                "/sensor_data", 10, std::bind(&StatePublisher::callback, this, std::placeholders::_1));
            pub_ = this->create_publisher<my_msgs::msg::RobotState>("/robot_state", 10);

            RCLCPP_INFO(this->get_logger(), "state_publisher has started.");
        }

        void callback(const my_msgs::msg::SensorData::SharedPtr msg)
        {
            my_msgs::msg::RobotState state;
            constexpr int shoulder_idx = 1;
            constexpr int elbow_idx = 0;
            constexpr int wheel_idx = 2;

            constexpr double pi = 3.14159265358979323846;
            constexpr double factor = 2 * pi / 8192 / 36;
            state.elbow_angle = msg->angle_integ[elbow_idx] * factor;
            state.elbow_omega = msg->rpm_raw[elbow_idx] * 2 * pi / 60.0 / 36;
            constexpr double shoulder_gear_ratio = 44.0 / 20.0;
            state.shoulder_angle = msg->angle_integ[1] * factor / shoulder_gear_ratio;
            state.shoulder_omega = msg->rpm_raw[shoulder_idx] * 2 * pi / 60.0 / 36 / shoulder_gear_ratio;
            constexpr double wheel_radius = 0.1;
            state.y = msg->angle_integ[2] * factor * wheel_radius;
            state.y_dot = msg->rpm_raw[wheel_idx] * 2 * pi / 60.0 / 36 * wheel_radius;

            pub_->publish(state);
            // RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        }

    private:
        rclcpp::Subscription<my_msgs::msg::SensorData>::SharedPtr sub_;
        rclcpp::Publisher<my_msgs::msg::RobotState>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}