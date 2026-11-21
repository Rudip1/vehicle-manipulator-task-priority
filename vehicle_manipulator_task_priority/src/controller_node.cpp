// Task-priority controller node: one MoveTo action at a time, solved at a fixed rate by vmtp_core's Controller.
//
// In:  /odom (base pose x, y, psi), /joint_states (serial joints of the robot description)
// Out: /cmd_vel (v, omega), /arm/<joint>/cmd_vel for waist, shoulder, elbow, wrist_level, wrist, /end_effector
// Action: move_to (vehicle_manipulator_task_priority_interfaces/action/MoveTo)

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vehicle_manipulator_task_priority_interfaces/action/move_to.hpp>

#include "vehicle_manipulator_task_priority/controller.hpp"

namespace vm = vehicle_manipulator_task_priority;
using MoveTo = vehicle_manipulator_task_priority_interfaces::action::MoveTo;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveTo>;

namespace {
const std::array<std::string, 5> kSerialJoints = {"waist", "shoulder", "elbow", "wrist_level", "wrist"};
}

class ControllerNode : public rclcpp::Node {
  public:
    ControllerNode() : Node("task_priority_controller") {
        vm::ControllerConfig cfg;
        cfg.gain_position = declare_parameter("gain_position", cfg.gain_position);
        cfg.gain_yaw = declare_parameter("gain_yaw", cfg.gain_yaw);
        cfg.gain_joints = declare_parameter("gain_joints", cfg.gain_joints);
        cfg.gain_heading = declare_parameter("gain_heading", cfg.gain_heading);
        cfg.gain_limits = declare_parameter("gain_limits", cfg.gain_limits);
        cfg.limit_alpha = declare_parameter("limit_alpha", cfg.limit_alpha);
        cfg.limit_delta = declare_parameter("limit_delta", cfg.limit_delta);
        cfg.damping = declare_parameter("damping", cfg.damping);
        cfg.max_speed = declare_parameter("max_speed", cfg.max_speed);
        cfg.position_tolerance = declare_parameter("position_tolerance", cfg.position_tolerance);
        cfg.joint_tolerance = declare_parameter("joint_tolerance", cfg.joint_tolerance);
        cfg.yaw_tolerance = declare_parameter("yaw_tolerance", cfg.yaw_tolerance);
        const std::vector<double> w = declare_parameter(
            "weights", std::vector<double>(cfg.weights.data(), cfg.weights.data() + cfg.weights.size()));
        if (w.size() == 6) cfg.weights = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(w.data());
        level_gain_ = declare_parameter("wrist_level_gain", 2.0);
        default_timeout_ = declare_parameter("default_timeout", 90.0);
        const double rate = declare_parameter("rate", 50.0);
        controller_ = std::make_unique<vm::Controller>(vm::MobileSwiftPro(vm::MobileSwiftPro::default_mount()), cfg);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                const auto& o = m->pose.pose.orientation;  // yaw of a quaternion
                const double yaw = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
                base_ << m->pose.pose.position.x, m->pose.pose.position.y, yaw;
                have_odom_ = true;
                odom_frame_ = m->header.frame_id;
            });
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 20, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
                for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) serial_pos_[m->name[i]] = m->position[i];
                have_joints_ = std::all_of(kSerialJoints.begin(), kSerialJoints.end(),
                                           [this](const std::string& n) { return serial_pos_.count(n) > 0; });
            });
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        for (const std::string& j : kSerialJoints)
            joint_pubs_.push_back(create_publisher<std_msgs::msg::Float64>("arm/" + j + "/cmd_vel", 10));
        ee_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("end_effector", 10);

        server_ = rclcpp_action::create_server<MoveTo>(
            this, "move_to",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveTo::Goal> goal) {
                if (goal->type == MoveTo::Goal::POSTURE && goal->posture.size() != 4) return rclcpp_action::GoalResponse::REJECT;
                if (goal->type > MoveTo::Goal::HEADING) return rclcpp_action::GoalResponse::REJECT;
                if (goal->type == MoveTo::Goal::HEADING && !have_odom_) return rclcpp_action::GoalResponse::REJECT;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](const std::shared_ptr<GoalHandle> handle) { start_goal(handle); });

        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { step(); });
        RCLCPP_INFO(get_logger(), "task-priority controller ready (%.0f Hz)", rate);
    }

  private:
    void start_goal(const std::shared_ptr<GoalHandle>& handle) {
        if (active_) {  // a new goal replaces the old one
            auto result = std::make_shared<MoveTo::Result>();
            result->message = "preempted";
            active_->abort(result);
        }
        const auto goal = handle->get_goal();
        switch (goal->type) {
            case MoveTo::Goal::POSITION:
                controller_->set_position_goal({goal->position.x, goal->position.y, goal->position.z});
                break;
            case MoveTo::Goal::POSITION_YAW:
                controller_->set_position_yaw_goal({goal->position.x, goal->position.y, goal->position.z}, goal->yaw);
                break;
            case MoveTo::Goal::HEADING:  // face the point from where the base is now
                controller_->set_heading_goal(std::atan2(goal->position.y - base_(1), goal->position.x - base_(0)));
                break;
            default:
                controller_->set_posture_goal(Eigen::Map<const vm::Vector4d>(goal->posture.data()));
        }
        controller_->set_goal_tolerance(goal->tolerance);
        active_ = handle;
        started_ = now();
        timeout_ = goal->timeout > 0.0 ? goal->timeout : default_timeout_;
    }

    void step() {
        if (!have_odom_ || !have_joints_) return;
        vm::Vector5d s;
        for (int i = 0; i < 5; ++i) s(i) = serial_pos_[kSerialJoints[size_t(i)]];
        const vm::Vector4d t = vm::SwiftPro::from_serial(s);
        Eigen::VectorXd q(7);
        q << base_, t;
        publish_end_effector(q);

        Eigen::VectorXd u = Eigen::VectorXd::Zero(6);
        if (active_) {
            if (active_->is_canceling()) {
                finish(false, "canceled", true);
            } else {
                u = controller_->step(q);
                auto fb = std::make_shared<MoveTo::Feedback>();
                fb->error = controller_->goal_error();
                fb->active_limits = static_cast<uint8_t>(controller_->active_limits());
                active_->publish_feedback(fb);
                if (controller_->goal_reached()) {
                    u.setZero();
                    finish(true, "reached");
                } else if ((now() - started_).seconds() > timeout_) {
                    u.setZero();
                    finish(false, "timeout");
                }
            }
        }
        publish_command(u, t);
    }

    void finish(bool success, const std::string& message, bool canceled = false) {
        auto result = std::make_shared<MoveTo::Result>();
        result->success = success;
        result->final_error = controller_->goal_error();
        result->duration = (now() - started_).seconds();
        result->message = message;
        if (canceled)
            active_->canceled(result);
        else if (success)
            active_->succeed(result);
        else
            active_->abort(result);
        RCLCPP_INFO(get_logger(), "goal %s: error %.4f after %.1f s", message.c_str(), result->final_error,
                    result->duration);
        active_.reset();
        controller_->clear_goal();
    }

    void publish_command(const Eigen::VectorXd& u, const vm::Vector4d& t) {
        geometry_msgs::msg::Twist tw;
        tw.angular.z = u(0);
        tw.linear.x = u(1);
        cmd_vel_pub_->publish(tw);
        // t-rates to serial joint rates; wrist_level follows -t3 with a little feedback so the cup stays level.
        vm::Vector5d s_dot = vm::SwiftPro::to_serial(u.tail<4>());
        s_dot(3) += level_gain_ * (-t(2) - serial_pos_["wrist_level"]);
        for (int i = 0; i < 5; ++i) {
            std_msgs::msg::Float64 m;
            m.data = s_dot(i);
            joint_pubs_[size_t(i)]->publish(m);
        }
    }

    void publish_end_effector(const Eigen::VectorXd& q) {
        const Eigen::Matrix4d T = controller_->model().frame_pose(q, vm::kEndEffector);
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = now();
        p.header.frame_id = odom_frame_;
        p.pose.position.x = T(0, 3);
        p.pose.position.y = T(1, 3);
        p.pose.position.z = T(2, 3);
        tf2::Quaternion qz;
        qz.setRPY(0, 0, std::atan2(T(1, 0), T(0, 0)));
        p.pose.orientation.x = qz.x();
        p.pose.orientation.y = qz.y();
        p.pose.orientation.z = qz.z();
        p.pose.orientation.w = qz.w();
        ee_pub_->publish(p);
    }

    std::unique_ptr<vm::Controller> controller_;
    double level_gain_ = 2.0, default_timeout_ = 90.0, timeout_ = 90.0;
    Eigen::Vector3d base_ = Eigen::Vector3d::Zero();
    std::map<std::string, double> serial_pos_;
    bool have_odom_ = false, have_joints_ = false;
    std::string odom_frame_ = "odom";
    std::shared_ptr<GoalHandle> active_;
    rclcpp::Time started_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_pubs_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pub_;
    rclcpp_action::Server<MoveTo>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
