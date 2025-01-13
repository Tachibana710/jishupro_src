#include <rclcpp/rclcpp.hpp>

#include "my_msgs/msg/sensor_data.hpp"
#include "my_msgs/msg/robot_state.hpp"
#include "my_msgs/msg/target_current.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "std_srvs/srv/empty.hpp"

#include <functional>
#include <cmath>
#include <array>
#include <optional>

struct joint_angle
{
    double shoulder_angle;
    double elbow_angle;
    double y;
    double shoulder_angle_dot = 0;
    double elbow_angle_dot = 0;
    double y_dot = 0;
};

constexpr double deg = M_PI / 180;
constexpr double mm = 0.001;
constexpr double gram = 0.001;
constexpr double pi = 3.14159265358979323846;
constexpr double l1 = 260.719 * mm;
constexpr double l2 = 290.097 * mm;
constexpr double hand_y = 45 * mm;
constexpr double shoulder_z = 65.5 * mm;
constexpr double shoulder_x = 100.3 * mm;

constexpr double gravity = 9.8; // m/s^2
constexpr double Kt = 0.18 / 1000; // Nm/mA
constexpr double shoulder_gear_ratio = 52.0 / 20.0;

constexpr double l1_mass = 257.5 * gram;
constexpr double l1_gpos = 120 * mm;

constexpr double l2_mass = 15 * gram;
constexpr double l2_gpos = l2 / 2;

constexpr int shoulder_idx = 1;
constexpr int elbow_idx = 0;
constexpr int wheel_idx = 2;



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

joint_angle handpos_to_jointangle(std::array<double,3> handpos, int sign = 1)
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

std::array<double, 3> operator+(const std::array<double, 3> &lhs, const std::array<double, 3> &rhs)
{
    return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}

std::array<double, 3> operator-(const std::array<double, 3> &lhs, const std::array<double, 3> &rhs)
{
    return {lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]};
}

std::array<double, 3> operator*(const std::array<double, 3> &lhs, const double &rhs)
{
    return {lhs[0] * rhs, lhs[1] * rhs, lhs[2] * rhs};
}

// class TrajectoryPlanner
// {
//     private:
//         std::array<double, 3> start_;
//         std::array<double, 3> waypoint_;
//         std::array<double, 3> end_;
//         double duration_;

//         double start_time_;

//     public:
//         std::array<double, 3> target_hand;
//         TrajectoryPlanner(std::array<double, 3> start, std::array<double, 3> end, double duration)
//             : start_(start), end_(end), duration_(duration)
//         {
//             start_time_ = rclcpp::Clock().now().seconds();
//             waypoint_ = (start_ + end_) * 0.5;
//             waypoint_[2] = 50 * mm;
//         }

//         std::array<double, 3> target_handpos()
//         {
//             double now_time = rclcpp::Clock().now().seconds();
//             double t = (now_time - start_time_) / duration_;
//             if (t > 1)
//             {
//                 return end_;
//             }
//             if (t > 0 && t < 0.5){
//                 return start_ + (waypoint_ - start_) * 2.0 * t;
//             }
//             if (t >= 0.5 && t < 1){
//                 return waypoint_ + (end_ - waypoint_) * 2.0 * (t - 0.5);
//             }
//         }

//         joint_angle operator()()
//         {
//             target_hand = target_handpos();
//             return handpos_to_jointangle(target_hand);
//         }

//         bool is_finished()
//         {
//             double now_time = rclcpp::Clock().now().seconds();
//             double t = (now_time - start_time_ - 0.5) / duration_;
//             return t > 1;
//         }
// };

class TrajectoryPlanner
{
    private:
        joint_angle start_;
        joint_angle waypoint_;
        joint_angle end_;
        double duration_;

        double start_time_;

    public:
        std::array<double, 3> target_hand;
        TrajectoryPlanner(std::array<double, 3> start, std::array<double, 3> end, double duration) : duration_(duration)
        {
            start_time_ = rclcpp::Clock().now().seconds();
            start_ = handpos_to_jointangle(start);
            end_ = handpos_to_jointangle(end);
            waypoint_ = handpos_to_jointangle((start + end) * 0.5);
            waypoint_.shoulder_angle += 15 * deg;
            waypoint_.elbow_angle += 30 * deg;
        }
        joint_angle operator()()
        {
            double now_time = rclcpp::Clock().now().seconds();
            double t = (now_time - start_time_) / duration_;
            if (t > 1)
            {
                return end_;
            }
            if (t > 0 && t < 0.5){
                joint_angle target_joint_angle;
                target_joint_angle.shoulder_angle = start_.shoulder_angle + (waypoint_.shoulder_angle - start_.shoulder_angle) * 2.0 * t;
                target_joint_angle.shoulder_angle_dot = (waypoint_.shoulder_angle - start_.shoulder_angle) * 2.0 / duration_;
                target_joint_angle.elbow_angle = start_.elbow_angle + (waypoint_.elbow_angle - start_.elbow_angle) * 2.0 * t;
                target_joint_angle.elbow_angle_dot = (waypoint_.elbow_angle - start_.elbow_angle) * 2.0 / duration_;
                target_joint_angle.y = start_.y + (waypoint_.y - start_.y) * 2.0 * t;
                target_joint_angle.y_dot = (waypoint_.y - start_.y) * 2.0 / duration_;
                return target_joint_angle;
            }
            if (t >= 0.5 && t < 1){
                joint_angle target_joint_angle;
                target_joint_angle.shoulder_angle = waypoint_.shoulder_angle + (end_.shoulder_angle - waypoint_.shoulder_angle) * 2.0 * (t - 0.5);
                target_joint_angle.shoulder_angle_dot = (end_.shoulder_angle - waypoint_.shoulder_angle) * 2.0 / duration_;
                target_joint_angle.elbow_angle = waypoint_.elbow_angle + (end_.elbow_angle - waypoint_.elbow_angle) * 2.0 * (t - 0.5);
                target_joint_angle.elbow_angle_dot = (end_.elbow_angle - waypoint_.elbow_angle) * 2.0 / duration_;
                target_joint_angle.y = waypoint_.y + (end_.y - waypoint_.y) * 2.0 * (t - 0.5);
                target_joint_angle.y_dot = (end_.y - waypoint_.y) * 2.0 / duration_;
                return target_joint_angle;
            }
        }

        bool is_finished()
        {
            double now_time = rclcpp::Clock().now().seconds();
            double t = (now_time - start_time_ - 0.5) / duration_;
            return t > 1;
        }
};

class ControlNode : public rclcpp::Node
{
    public:
        ControlNode() : Node("control_node")
        {
            state_sub_ = this->create_subscription<my_msgs::msg::RobotState>(
                "/robot_state", 10, std::bind(&ControlNode::state_callback, this, std::placeholders::_1));

            end_efector_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/end_effector", 10, 
                [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
                {
                    end_effector_ = *msg;
                });

            target_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
            // target_sub_ = this->create_subscription<my_msgs::msg::RobotState>(
            //     "/target_state", 10, std::bind(&ControlNode::target_callback, this, std::placeholders::_1));
            target_pub_ = this->create_publisher<my_msgs::msg::RobotState>("/target_state", 10);
            pub_ = this->create_publisher<my_msgs::msg::TargetCurrent>("/target_current", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&ControlNode::cycle_callback, this));

            start_control_srv_ = this->create_service<std_srvs::srv::Empty>(
                "/start_control", 
                [this](const std_srvs::srv::Empty::Request::SharedPtr request,
                       std_srvs::srv::Empty::Response::SharedPtr response)
                {
                    (void)request;
                    (void)response;
                    emergency_stop_ = false;
                    return;
                });
            emergency_stop_srv_ = this->create_service<std_srvs::srv::Empty>(
                "/emergency_stop", 
                [this](const std_srvs::srv::Empty::Request::SharedPtr request,
                       std_srvs::srv::Empty::Response::SharedPtr response)
                {
                    (void)request;
                    (void)response;
                    emergency_stop_ = true;
                    return;
                });

            state_ = my_msgs::msg::RobotState();

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            shoulder_regulator_ = PIDRegulator(
                [this](){return state_.shoulder_angle;},
                [this](){return state_.shoulder_omega;},
                6, 0.9);
            elbow_regulator_ = PIDRegulator(
                [this](){return state_.elbow_angle;},
                [this](){return state_.elbow_omega;},
                4, 0.3);
            wheel_regulator_ = PIDRegulator(
                [this](){return state_.y;},
                [this](){return state_.y_dot;},
                20, 3);

            RCLCPP_INFO(this->get_logger(), "control_node has started.");
        }

        void comp_gravity_torque(my_msgs::msg::TargetCurrent& target_current){
            double elbow_gtorque = l2_gpos * std::cos(state_.shoulder_angle + state_.elbow_angle) * l2_mass * gravity;
            double elbow_comp_current = elbow_gtorque / Kt;

            double shoulder_gtorque = 
                l1_gpos * std::cos(state_.shoulder_angle) * l1_mass * gravity
                + (l1 * std::cos(state_.shoulder_angle) + l2_gpos * std::cos(state_.shoulder_angle + state_.elbow_angle)) * l2_mass * gravity;
            double shoulder_comp_current = shoulder_gtorque / Kt / shoulder_gear_ratio;

            target_current.target_current[elbow_idx] += elbow_comp_current;
            target_current.target_current[shoulder_idx] += shoulder_comp_current;
        }

        

        void state_callback(const my_msgs::msg::RobotState::SharedPtr msg)
        {
            state_ = *msg;

            my_msgs::msg::TargetCurrent target_current;

            if (emergency_stop_)
            {
                target_current.target_current[shoulder_idx] = 0;
                target_current.target_current[elbow_idx] = 0;
                target_current.target_current[wheel_idx] = 0;
                pub_->publish(target_current);
                return;
            }

            target_current.target_current[shoulder_idx] = shoulder_regulator_(target_.shoulder_angle, target_.shoulder_omega) * 1000;
            target_current.target_current[elbow_idx] = elbow_regulator_(target_.elbow_angle, target_.elbow_omega) * 1000;
            // target_current.target_current[elbow_idx] = std::sin(rclcpp::Clock().now().seconds()) * 500;
            target_current.target_current[wheel_idx] = wheel_regulator_(target_.y, target_.y_dot) * 1000;

            comp_gravity_torque(target_current);

            for (auto &current : target_current.target_current)
            {
                if (current > 5000)
                {
                    current = 5000;
                }
                else if (current < -5000)
                {
                    current = -5000;
                }
                if (std::abs(current) > 10000)
                {
                    emergency_stop_ = true;
                }
            }
            pub_->publish(target_current);
        }

        // void target_callback(const my_msgs::msg::RobotState::SharedPtr msg)
        // {
        //     target_ = *msg;
        // }

        void cycle_callback()
        {
            try {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "object1", tf2::TimePointZero);
                object1_.point.x = transform.transform.translation.x;
                object1_.point.y = transform.transform.translation.y;
                object1_.point.z = transform.transform.translation.z;
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            }

            static double last_time = rclcpp::Clock().now().seconds();
            double now_time = rclcpp::Clock().now().seconds();
            double dt = now_time - last_time;
            if (traj_)
            {
                if (traj_->is_finished())
                {
                    traj_ = std::nullopt;
                    last_time = now_time;
                }else{
                    auto target_joint_angle = (*traj_)();
                    set_target(
                        target_joint_angle.shoulder_angle,
                        target_joint_angle.elbow_angle, 
                        target_joint_angle.y,
                        target_joint_angle.shoulder_angle_dot,
                        target_joint_angle.elbow_angle_dot,
                        target_joint_angle.y_dot);
                    // geometry_msgs::msg::PointStamped target_point;
                    // target_point.header = std_msgs::msg::Header();
                    // target_point.header.stamp = this->now();
                    // target_point.header.frame_id = "map";

                    // target_point.point.x = traj_->target_hand[0];
                    // target_point.point.y = traj_->target_hand[1];
                    // target_point.point.z = traj_->target_hand[2];
                    // target_point_pub_->publish(target_point);

                }
            }else{
                if (dt > 2.0 && !emergency_stop_){
                    traj_ = TrajectoryPlanner(
                        {end_effector_.point.x, end_effector_.point.y, end_effector_.point.z},
                        {object1_.point.x, object1_.point.y, object1_.point.z},
                        0.5);
                }
            }



            // auto target_joint_angle = handpos_to_jointangle({object1_.point.x, object1_.point.y, object1_.point.z+20 * mm});
            // // target_.shoulder_angle = target_joint_angle.shoulder_angle;
            // // target_.elbow_angle = target_joint_angle.elbow_angle;
            // // target_.y = target_joint_angle.y;
            // set_target(
            //     target_joint_angle.shoulder_angle,
            //     target_joint_angle.elbow_angle, 
            //     target_joint_angle.y);
            target_pub_->publish(target_);

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


        void set_target(const double shoulder_angle, const double elbow_angle, const double y,
                        const double shoulder_omega = 0, const double elbow_omega = 0, const double y_dot = 0)
        {
            if (shoulder_angle < 0 || shoulder_angle > M_PI / 2)
            {
                target_.shoulder_angle = state_.shoulder_angle;
                RCLCPP_INFO(this->get_logger(), "shoulder_angle is out of range: %f", shoulder_angle);
            }else{
                target_.shoulder_angle = shoulder_angle;
            }
            if (std::abs(shoulder_omega) > 5)
            {
                target_.shoulder_omega = 0;
                RCLCPP_INFO(this->get_logger(), "shoulder_omega is out of range: %f", shoulder_omega);
            }else{
                target_.shoulder_omega = shoulder_omega;
            }
            if (elbow_angle < -M_PI || elbow_angle > 90 * deg)
            {
                target_.elbow_angle = state_.elbow_angle;
                RCLCPP_INFO(this->get_logger(), "elbow_angle is out of range: %f", elbow_angle);
            }else{
                target_.elbow_angle = elbow_angle;
            }
            if (std::abs(elbow_omega) > 10)
            {
                target_.elbow_omega = 0;
                RCLCPP_INFO(this->get_logger(), "elbow_omega is out of range: %f", elbow_omega);
            }else{
                target_.elbow_omega = elbow_omega;
            }
            if (y < 0 || y > 500 * mm)
            {
                target_.y = state_.y;
                RCLCPP_INFO(this->get_logger(), "y is out of range: %f", y);
            }else{
                target_.y = y;
            }
            if (std::abs(y_dot) > 0.5)
            {
                target_.y_dot = 0;
                RCLCPP_INFO(this->get_logger(), "y_dot is out of range: %f", y_dot);
            }else{
                target_.y_dot = y_dot;
            }
            // target_.shoulder_angle = shoulder_angle;
            // target_.elbow_angle = elbow_angle;
            // target_.y = y;
        }



    private:
        std::optional<TrajectoryPlanner> traj_ = std::nullopt;

        my_msgs::msg::RobotState state_;
        my_msgs::msg::RobotState target_;
        geometry_msgs::msg::PointStamped end_effector_;

        rclcpp::Subscription<my_msgs::msg::RobotState>::SharedPtr state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr end_efector_sub;
        // rclcpp::Subscription<my_msgs::msg::RobotState>::SharedPtr target_sub_;
        rclcpp::Publisher<my_msgs::msg::RobotState>::SharedPtr target_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_pub_;
        rclcpp::Publisher<my_msgs::msg::TargetCurrent>::SharedPtr pub_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_control_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr emergency_stop_srv_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::TimerBase::SharedPtr timer_;

        PIDRegulator shoulder_regulator_;
        PIDRegulator elbow_regulator_;
        PIDRegulator wheel_regulator_;

        geometry_msgs::msg::PointStamped object1_;

        bool emergency_stop_ = true;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}