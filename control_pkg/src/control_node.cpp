#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"
#include "my_msgs/msg/target_current.hpp"


struct joint_angle
{
    double shoulder_angle;
    double elbow_angle;
    double y;
};

constexpr double mm = 0.001;
constexpr double pi = 3.14159265358979323846;
constexpr double l1 = 260.719 * mm;
constexpr double l2 = 290.097 * mm;
constexpr double hand_y = 45 * mm;
constexpr double shoulder_z = 65.5 * mm;
constexpr double shoulder_x = 100.3 * mm;


class PIDRegulator
{
    public:
        PIDRegulator(std::function<double()> get_pos, std::function<double()> get_vel, double kp, double kd)
            : get_pos_(get_pos), get_vel_(get_vel), kp_(kp), kd_(kd)
        {
        }

        PIDRegulator() = default;

        double operator()(double dest_pos, double dest_vel=0)
        {
            double error = dest_pos - get_pos_();
            double error_dot = dest_vel - get_vel_();
            // double now_vel = (get_pos_() - prev_value_) / (rclcpp::Clock().now().seconds() - prev_time_);
            // prev_value_ = get_pos_();
            // prev_time_ = rclcpp::Clock().now().seconds();
            // double error_dot = dest_vel - now_vel;

            double control = kp_ * error + kd_ * error_dot;

            return control;

        }

    private:
        std::function<double()> get_pos_;
        std::function<double()> get_vel_;

        double prev_value_ = 0;
        double prev_time_ = 0;

        double kp_;
        double kd_;
};

class ControlNode : public rclcpp::Node
{
    public:
        ControlNode() : Node("control_node")
        {
            state_sub_ = this->create_subscription<my_msgs::msg::RobotState>(
                "/robot_state", 10, std::bind(&ControlNode::state_callback, this, std::placeholders::_1));
            target_sub_ = this->create_subscription<my_msgs::msg::RobotState>(
                "/target_state", 10, std::bind(&ControlNode::target_callback, this, std::placeholders::_1));
            pub_ = this->create_publisher<my_msgs::msg::TargetCurrent>("/target_current", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ControlNode::cycle_callback, this));

            state_ = my_msgs::msg::RobotState();

            shoulder_regulator_ = PIDRegulator(
                [this](){return state_.shoulder_angle;},
                [this](){return state_.shoulder_omega;},
                2, 0.3);
            elbow_regulator_ = PIDRegulator(
                [this](){return state_.elbow_angle;},
                [this](){return state_.elbow_omega;},
                2, 0.3);
            wheel_regulator_ = PIDRegulator(
                [this](){return state_.y;},
                [this](){return state_.y_dot;},
                20, 3);

            RCLCPP_INFO(this->get_logger(), "control_node has started.");
        }

        void state_callback(const my_msgs::msg::RobotState::SharedPtr msg)
        {
            state_ = *msg;
            constexpr int shoulder_idx = 1;
            constexpr int elbow_idx = 0;
            constexpr int wheel_idx = 2;
            my_msgs::msg::TargetCurrent target_current;

            target_current.target_current[shoulder_idx] = shoulder_regulator_(target_.shoulder_angle, target_.shoulder_omega) * 1000;
            target_current.target_current[elbow_idx] = elbow_regulator_(target_.elbow_angle, target_.elbow_omega) * 1000;
            // target_current.target_current[elbow_idx] = std::sin(rclcpp::Clock().now().seconds()) * 500;
            target_current.target_current[wheel_idx] = wheel_regulator_(target_.y, target_.y_dot) * 1000;

            pub_->publish(target_current);
        }

        void target_callback(const my_msgs::msg::RobotState::SharedPtr msg)
        {
            target_ = *msg;
        }

        void cycle_callback()
        {
            return;
            // constexpr int shoulder_idx = 1;
            // constexpr int elbow_idx = 0;
            // constexpr int wheel_idx = 2;
            // my_msgs::msg::TargetCurrent target_current;

            // target_current.target_current[shoulder_idx] = shoulder_regulator_(target_.shoulder_angle, target_.shoulder_omega) * 1000;
            // target_current.target_current[elbow_idx] = elbow_regulator_(target_.elbow_angle, target_.elbow_omega) * 1000;
            // target_current.target_current[wheel_idx] = wheel_regulator_(target_.y, target_.y_dot) * 1000;

            // pub_->publish(target_current);
        }

        joint_angle handpos_to_jointangle(std::array<double,3> handpos, int sign = -1)
        {
            double x = handpos[0];
            double y = handpos[1];
            double z = handpos[2];

            x -= shoulder_x;
            z -= shoulder_z;
            y -= hand_y;
            double r = std::sqrt(x * x + z * z);
            double shoulder_angle = std::atan2(z, r) + sign * std::acos((l1 * l1 + r * r - l2 * l2) / (2 * l1 * r));
            // double elbow_angle = M_PI - std::acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));
            double elbow_angle = std::atan2(z - l1 * std::sin(shoulder_angle), x - l1 * std::cos(shoulder_angle)) - shoulder_angle;
            // double theta1 = std::atan2(z, x);
            // double theta3 = std::acos((l1 * l1 + l2 * l2 - r * r - y * y) / (2 * l1 * l2));
            // double theta2 = std::atan2(y, r) - std::atan2(l2 * std::sin(theta3), l1 + l2 * std::cos(theta3));
            return joint_angle{shoulder_angle, elbow_angle, y};
        }

    private:
        my_msgs::msg::RobotState state_;
        my_msgs::msg::RobotState target_;

        rclcpp::Subscription<my_msgs::msg::RobotState>::SharedPtr state_sub_;
        rclcpp::Subscription<my_msgs::msg::RobotState>::SharedPtr target_sub_;
        rclcpp::Publisher<my_msgs::msg::TargetCurrent>::SharedPtr pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        PIDRegulator shoulder_regulator_;
        PIDRegulator elbow_regulator_;
        PIDRegulator wheel_regulator_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}