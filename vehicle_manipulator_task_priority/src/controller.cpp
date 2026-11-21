#include "vehicle_manipulator_task_priority/controller.hpp"

#include <manipulator_control/tasks.hpp>
#include <cmath>
#include <string>

namespace vehicle_manipulator_task_priority {

namespace mc = manipulator_control;

namespace {

/// Posture of the four arm joints as one task: J = [0 0 | I4], error = t_d - t.
class PostureTask : public mc::Task {
  public:
    explicit PostureTask(const Vector4d& desired) : Task("posture", 4), desired_(desired) {}
    void update(const mc::RobotModel& model, const Eigen::VectorXd& q) override {
        J_ = Eigen::MatrixXd::Zero(4, model.control_size());
        J_.rightCols<4>().setIdentity();
        error_ = desired_ - q.tail<4>();
    }

  private:
    Vector4d desired_;
};

}  // namespace

Controller::Controller(MobileSwiftPro model, ControllerConfig config) : model_(std::move(model)), config_(config) {
    const SwiftProLimits& lim = model_.arm().limits();
    for (int i = 0; i < 4; ++i) {
        auto task = std::make_shared<mc::JointLimitTask>("limit t" + std::to_string(i + 1), 2 + i, lim.lower(i),
                                                         lim.upper(i), config_.limit_alpha, config_.limit_delta);
        task->set_gain(config_.gain_limits);
        limits_.push_back(task);
    }
}

void Controller::set_position_goal(const Eigen::Vector3d& position) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = position;
    auto task = std::make_shared<mc::FrameTask>("suction cup", kEndEffector, std::vector<int>{0, 1, 2}, T);
    task->set_gain(config_.gain_position);
    goal_ = task;
    goal_type_ = GoalType::Position;
}

void Controller::set_position_yaw_goal(const Eigen::Vector3d& position, double yaw) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = std::cos(yaw);
    T(0, 1) = -std::sin(yaw);
    T(1, 0) = std::sin(yaw);
    T(1, 1) = std::cos(yaw);
    T.block<3, 1>(0, 3) = position;
    auto task = std::make_shared<mc::FrameTask>("suction cup", kEndEffector, std::vector<int>{0, 1, 2, 5}, T);
    Eigen::Vector4d k(config_.gain_position, config_.gain_position, config_.gain_position, config_.gain_yaw);
    task->set_gain(Eigen::MatrixXd(k.asDiagonal()));
    goal_ = task;
    goal_type_ = GoalType::PositionYaw;
}

void Controller::set_posture_goal(const Vector4d& posture) {
    auto task = std::make_shared<PostureTask>(posture);
    task->set_gain(config_.gain_joints);
    goal_ = task;
    goal_type_ = GoalType::Posture;
}

void Controller::clear_goal() {
    goal_.reset();
    goal_type_ = GoalType::None;
}

Eigen::VectorXd Controller::step(const Eigen::VectorXd& q) {
    mc::TaskList tasks(limits_.begin(), limits_.end());
    if (goal_) tasks.push_back(goal_);
    mc::PriorityConfig pc;
    pc.inverse = mc::InverseMethod::DampedLeastSquares;
    pc.damping = config_.damping;
    pc.max_speed = config_.max_speed;
    pc.weights = config_.weights;
    const Eigen::VectorXd u = mc::task_priority_step(model_, q, tasks, pc);
    active_limits_ = 0;
    for (int i = 0; i < 4; ++i)
        if (limits_[size_t(i)]->is_active()) active_limits_ |= 1u << i;
    goal_error_ = goal_ ? goal_->error().head(goal_type_ == GoalType::Posture ? 4 : 3).norm() : 0.0;
    yaw_error_ = goal_type_ == GoalType::PositionYaw ? std::abs(goal_->error()(3)) : 0.0;
    return u;
}

bool Controller::goal_reached() const {
    switch (goal_type_) {
        case GoalType::None:
            return true;
        case GoalType::Posture:
            return goal_error_ < config_.joint_tolerance;
        case GoalType::Position:
            return goal_error_ < config_.position_tolerance;
        case GoalType::PositionYaw:
            return goal_error_ < config_.position_tolerance && yaw_error_ < config_.yaw_tolerance;
    }
    return false;
}

}  // namespace vehicle_manipulator_task_priority
