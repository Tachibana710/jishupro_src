#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"

#include "std_srvs/srv/empty.hpp"

#define M_PI 3.14159265358979323846

struct offset_data{
    int32_t elbow_offset_raw = 0;
    int32_t shoulder_offset_raw = 0;
    int32_t wheel_offset_raw = 0;

    double elbow_offset = 0;
    double shoulder_offset = 1.5708;
    double wheel_offset = 0;
};


constexpr int shoulder_idx = 1;
constexpr int elbow_idx = 0;
constexpr int wheel_idx = 2;


class StatePublisher : public rclcpp::Node
{
    public:
        StatePublisher() : Node("state_publisher")
        {
            sub_ = this->create_subscription<my_msgs::msg::SensorData>(
                "/sensor_data", 10, std::bind(&StatePublisher::callback, this, std::placeholders::_1));
            pub_ = this->create_publisher<my_msgs::msg::RobotState>("/robot_state", 10);

            init_pose_srv_ = this->create_service<std_srvs::srv::Empty>(
                "/init_pose", std::bind(&StatePublisher::init_pose_, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "state_publisher has started.");
        }

        void callback(const my_msgs::msg::SensorData::SharedPtr msg)
        {
            sensor_data_ = *msg;
            my_msgs::msg::RobotState state;

            constexpr double pi = 3.14159265358979323846;
            constexpr double factor = 2 * pi / 8192 / 36;
            state.elbow_angle = msg->angle_integ[elbow_idx] * factor;
            state.elbow_omega = msg->rpm_raw[elbow_idx] * 2 * pi / 60.0 / 36;
            constexpr double shoulder_gear_ratio = 44.0 / 20.0;
            state.shoulder_angle = msg->angle_integ[shoulder_idx] * factor / shoulder_gear_ratio;
            state.shoulder_omega = msg->rpm_raw[shoulder_idx] * 2 * pi / 60.0 / 36 / shoulder_gear_ratio;
            constexpr double wheel_radius = 0.05 / 2;
            state.y = msg->angle_integ[wheel_idx] * factor * wheel_radius;
            state.y_dot = msg->rpm_raw[wheel_idx] * 2 * pi / 60.0 / 36 * wheel_radius;

            pub_->publish(state);
            // RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        }

        void init_pose_(const std_srvs::srv::Empty::Request::SharedPtr request,
                        std_srvs::srv::Empty::Response::SharedPtr response)
        {
            offset_.elbow_offset_raw = sensor_data_.angle_integ[elbow_idx];
            offset_.shoulder_offset_raw = sensor_data_.angle_integ[shoulder_idx];
            offset_.wheel_offset_raw = sensor_data_.angle_integ[wheel_idx];

            offset_.elbow_offset = -M_PI / 2;
            offset_.shoulder_offset = M_PI / 2;
            offset_.wheel_offset = 0;
            RCLCPP_INFO(this->get_logger(), "init_pose service has been called.");
        }

    private:
        rclcpp::Subscription<my_msgs::msg::SensorData>::SharedPtr sub_;
        rclcpp::Publisher<my_msgs::msg::RobotState>::SharedPtr pub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr init_pose_srv_;
        offset_data offset_;
        my_msgs::msg::SensorData sensor_data_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}