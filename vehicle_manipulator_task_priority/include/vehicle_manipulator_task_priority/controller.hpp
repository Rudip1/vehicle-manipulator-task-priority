#pragma once
/// @file controller.hpp
/// Task-priority controller of the mobile uArm, without ROS. Every step solves the hierarchy
///
///   1. joint limits of t1..t4 (set-based, active only near a limit)
///   2. the current goal: suction-cup position (optionally with yaw), or an arm posture
///
/// with the weighted, damped recursion of learn-manipulator-control (chapters 7-9) and scales the command so
/// that no entry exceeds the speed limit. A suction-cup goal far from the arm makes the base drive: the base is
/// part of the same kinematic system, so no separate navigation is needed.

#include <Eigen/Core>
#include <manipulator_control/inequality_tasks.hpp>
#include <manipulator_control/task_priority.hpp>
#include <memory>
#include <optional>

#include "vehicle_manipulator_task_priority/mobile_swift_pro.hpp"

namespace vehicle_manipulator_task_priority {

struct ControllerConfig {
    double gain_position = 1.0;   ///< K of the suction-cup position task [1/s]
    double gain_yaw = 1.0;        ///< K of the suction-cup yaw component [1/s]
    double gain_joints = 1.0;     ///< K of the posture task [1/s]
    double gain_limits = 0.5;     ///< push of an active joint-limit task [rad/s]
    double limit_alpha = 0.05;    ///< activation margin of the joint limits [rad]
    double limit_delta = 0.09;    ///< release margin [rad]
    double damping = 0.05;        ///< lambda of the DLS inverse at every level
    double max_speed = 0.3;       ///< largest |u_j| [rad/s or m/s]
    /// Weights on (omega, v, t1dot..t4dot): larger is more expensive.
    Eigen::Matrix<double, 6, 1> weights = (Eigen::Matrix<double, 6, 1>() << 2, 2, 1, 1, 1, 1).finished();
    double position_tolerance = 0.01;  ///< goal reached below this suction-cup error [m]
    double joint_tolerance = 0.03;     ///< goal reached below this posture error [rad]
    double yaw_tolerance = 0.05;       ///< and below this suction-cup yaw error, for PositionYaw goals [rad]
};

enum class GoalType { None, Position, PositionYaw, Posture };

class Controller {
  public:
    explicit Controller(MobileSwiftPro model, ControllerConfig config = {});

    const MobileSwiftPro& model() const { return model_; }
    const ControllerConfig& config() const { return config_; }

    /// Suction cup to @p position (world frame), orientation free.
    void set_position_goal(const Eigen::Vector3d& position);
    /// Suction cup to @p position with yaw @p yaw (world frame).
    void set_position_yaw_goal(const Eigen::Vector3d& position, double yaw);
    /// Arm joints t1..t4 to @p posture; the base stays (it is not needed by this task).
    void set_posture_goal(const Vector4d& posture);
    void clear_goal();

    GoalType goal_type() const { return goal_type_; }

    /// Command u = (omega, v, t1dot..t4dot) at configuration q = (x, y, psi, t1..t4).
    Eigen::VectorXd step(const Eigen::VectorXd& q);

    /// Norm of the goal task's error at the last step: position [m] or posture [rad].
    double goal_error() const { return goal_error_; }
    /// |yaw error| of a PositionYaw goal at the last step [rad].
    double yaw_error() const { return yaw_error_; }
    bool goal_reached() const;
    /// Joint-limit tasks active at the last step (bit i for joint t_{i+1}).
    unsigned active_limits() const { return active_limits_; }

  private:
    MobileSwiftPro model_;
    ControllerConfig config_;
    std::vector<std::shared_ptr<manipulator_control::JointLimitTask>> limits_;
    std::shared_ptr<manipulator_control::Task> goal_;
    GoalType goal_type_ = GoalType::None;
    double goal_error_ = 0.0;
    double yaw_error_ = 0.0;
    unsigned active_limits_ = 0;
};

}  // namespace vehicle_manipulator_task_priority
