#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"

#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#define M_PI 3.14159265358979323846

constexpr double mm = 0.001;
constexpr double pi = 3.14159265358979323846;
constexpr double l1 = 260.719 * mm;
constexpr double l2 = 290.097 * mm;
constexpr double hand_y = 45 * mm;
constexpr double shoulder_z = 65.5 * mm;
constexpr double shoulder_x = 100.3 * mm;

struct offset_data{
    int32_t elbow_offset_raw = 0;
    int32_t shoulder_offset_raw = 0;
    int32_t wheel_offset_raw = 0;

    double elbow_offset = 0;
    double shoulder_offset = M_PI / 2;
    double wheel_offset = 150 * mm;
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

            robot_origin_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/robot_origin", 10);
            shoulder_origin_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/shoulder_origin", 10);
            elbow_origin_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/elbow_origin", 10);
            end_effector_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/end_effector", 10);
            observ_y_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "/recognition/observ_y", 10, std::bind(&StatePublisher::observ_y_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "state_publisher has started.");
        }

        void callback(const my_msgs::msg::SensorData::SharedPtr msg)
        {
            sensor_data_ = *msg;
            my_msgs::msg::RobotState state;

            constexpr double pi = 3.14159265358979323846;
            constexpr double factor = 2 * pi / 8192 / 36;
            state.elbow_angle = (msg->angle_integ[elbow_idx] - offset_.elbow_offset_raw) * factor + offset_.elbow_offset;
            state.elbow_omega = msg->rpm_raw[elbow_idx] * 2 * pi / 60.0 / 36;
            constexpr double shoulder_gear_ratio = 52.0 / 20.0;
            state.shoulder_angle = (msg->angle_integ[shoulder_idx] - offset_.shoulder_offset_raw) * factor / shoulder_gear_ratio + offset_.shoulder_offset;
            state.shoulder_omega = msg->rpm_raw[shoulder_idx] * 2 * pi / 60.0 / 36 / shoulder_gear_ratio;
            constexpr double wheel_radius = 0.05 / 2;
            state.y = (msg->angle_integ[wheel_idx] - offset_.wheel_offset_raw) * factor * wheel_radius + offset_.wheel_offset;
            state.y_dot = msg->rpm_raw[wheel_idx] * 2 * pi / 60.0 / 36 * wheel_radius;

            state_y_ = state.y;

            auto header = std_msgs::msg::Header();
            header.stamp = this->now();
            header.frame_id = "map";

            geometry_msgs::msg::PointStamped robot_origin;
            robot_origin.header = header;
            robot_origin.point.x = 0;
            robot_origin.point.y = state.y;
            robot_origin.point.z = 0;
            // state.robot_origin = robot_origin;
            robot_origin_pub_->publish(robot_origin);

            std::array<double, 3> robot_to_shoulder = {shoulder_x, hand_y, shoulder_z};
            geometry_msgs::msg::PointStamped shoulder_origin;
            shoulder_origin.header = header;
            shoulder_origin.point.x = robot_origin.point.x + robot_to_shoulder[0];
            shoulder_origin.point.y = robot_origin.point.y + robot_to_shoulder[1];
            shoulder_origin.point.z = robot_origin.point.z + robot_to_shoulder[2];
            shoulder_origin_pub_->publish(shoulder_origin);
            // state.shoulder_origin = shoulder_origin;

            std::array<double, 3> shoulder_to_elbow = {
                l1 * std::cos(state.shoulder_angle),
                0.0,
                l1 * std::sin(state.shoulder_angle)
            };
            geometry_msgs::msg::PointStamped elbow_origin;
            elbow_origin.header = header;
            elbow_origin.point.x = shoulder_origin.point.x + shoulder_to_elbow[0];
            elbow_origin.point.y = shoulder_origin.point.y + shoulder_to_elbow[1];
            elbow_origin.point.z = shoulder_origin.point.z + shoulder_to_elbow[2];
            elbow_origin_pub_->publish(elbow_origin);
            // state.elbow_origin = elbow_origin;

            std::array<double, 3> elbow_to_end_effector = {
                l2 * std::cos(state.shoulder_angle + state.elbow_angle),
                0.0,
                l2 * std::sin(state.shoulder_angle + state.elbow_angle)
            };
            geometry_msgs::msg::PointStamped end_effector;
            end_effector.header = header;
            end_effector.point.x = elbow_origin.point.x + elbow_to_end_effector[0];
            end_effector.point.y = elbow_origin.point.y + elbow_to_end_effector[1];
            end_effector.point.z = elbow_origin.point.z + elbow_to_end_effector[2];
            end_effector_pub_->publish(end_effector);

            pub_->publish(state);
            // RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        }

        void init_pose_(const std_srvs::srv::Empty::Request::SharedPtr request,
                        std_srvs::srv::Empty::Response::SharedPtr response)
        {
            (void)request;
            (void)response;

            offset_.elbow_offset_raw = sensor_data_.angle_integ[elbow_idx];
            offset_.shoulder_offset_raw = sensor_data_.angle_integ[shoulder_idx];
            offset_.wheel_offset_raw = sensor_data_.angle_integ[wheel_idx];

            offset_.elbow_offset = 0;
            offset_.shoulder_offset = M_PI / 2;
            offset_.wheel_offset = 150 * mm;
            RCLCPP_INFO(this->get_logger(), "init_pose service has been called.");
        }

        void observ_y_callback(const std_msgs::msg::Float32::SharedPtr msg){
            // RCLCPP_INFO(this->get_logger(), "y: %f", msg->data);
            offset_.wheel_offset += 0.1 * (msg->data - state_y_);
        }

    private:
        rclcpp::Subscription<my_msgs::msg::SensorData>::SharedPtr sub_;
        rclcpp::Publisher<my_msgs::msg::RobotState>::SharedPtr pub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr init_pose_srv_;

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr robot_origin_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr shoulder_origin_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr elbow_origin_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr end_effector_pub_;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr observ_y_sub_;

        offset_data offset_;
        my_msgs::msg::SensorData sensor_data_;

        double state_y_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}