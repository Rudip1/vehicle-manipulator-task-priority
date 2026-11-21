#pragma once
/// @file mobile_swift_pro.hpp
/// A differential-drive base carrying a uArm Swift Pro, as a RobotModel of learn-manipulator-control (chapter 9
/// there): configuration q = (x, y, psi, t1..t4), commanded velocity u = (omega, v, t1dot..t4dot).

#include <Eigen/Core>
#include <manipulator_control/robot_model.hpp>

#include "vehicle_manipulator_task_priority/swift_pro.hpp"

namespace vehicle_manipulator_task_priority {

/// Frames of the model.
enum Frame : int {
    kEndEffector = -1,  ///< suction cup
    kBase = 0,          ///< base_footprint
    kArmBase = 1,       ///< arm base frame A
};

class MobileSwiftPro : public manipulator_control::RobotModel {
  public:
    /// @p mount: base_footprint -> arm base frame A.
    explicit MobileSwiftPro(const Eigen::Matrix4d& mount, SwiftPro arm = SwiftPro{});

    /// Mount of the arm on the Kobuki-size base used in the simulation: 0.051 m ahead, 0.198 m up.
    static Eigen::Matrix4d default_mount();

    const SwiftPro& arm() const { return arm_; }
    const Eigen::Matrix4d& mount() const { return mount_; }

    int state_size() const override { return 7; }
    int control_size() const override { return 6; }
    Eigen::Matrix4d frame_pose(const Eigen::VectorXd& q, int k) const override;
    Eigen::MatrixXd frame_jacobian(const Eigen::VectorXd& q, int k) const override;
    int state_index(int j) const override;
    Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& u, double dt) const override;

  private:
    Eigen::Matrix4d mount_;
    SwiftPro arm_;
};

}  // namespace vehicle_manipulator_task_priority
