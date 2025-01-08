#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"
#include "my_msgs/msg/target_current.hpp"

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
                0.1, 0.1);
            elbow_regulator_ = PIDRegulator(
                [this](){return state_.elbow_angle;},
                [this](){return state_.elbow_omega;},
                1, 0.3);
            wheel_regulator_ = PIDRegulator(
                [this](){return state_.y;},
                [this](){return state_.y_dot;},
                0.1, 0.1);

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