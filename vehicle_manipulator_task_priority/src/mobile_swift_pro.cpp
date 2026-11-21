#include "vehicle_manipulator_task_priority/mobile_swift_pro.hpp"

#include <cmath>
#include <manipulator_control/mobile_manipulator.hpp>
#include <stdexcept>

namespace vehicle_manipulator_task_priority {

namespace mc = manipulator_control;

MobileSwiftPro::MobileSwiftPro(const Eigen::Matrix4d& mount, SwiftPro arm) : mount_(mount), arm_(arm) {}

Eigen::Matrix4d MobileSwiftPro::default_mount() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) << 0.051, 0.0, 0.198;
    return T;
}

Eigen::Matrix4d MobileSwiftPro::frame_pose(const Eigen::VectorXd& q, int k) const {
    if (q.size() != 7) throw std::invalid_argument("MobileSwiftPro: q must have 7 entries");
    const Eigen::Matrix4d T_base = mc::base_transform(q.head<3>());
    switch (k) {
        case kBase:
            return T_base;
        case kArmBase:
            return T_base * mount_;
        case kEndEffector:
            return T_base * mount_ * arm_.end_effector(q.tail<4>());
        default:
            throw std::out_of_range("MobileSwiftPro: unknown frame");
    }
}

Eigen::MatrixXd MobileSwiftPro::frame_jacobian(const Eigen::VectorXd& q, int k) const {
    const Eigen::Vector3d p = frame_pose(q, k).block<3, 1>(0, 3);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
    // Base columns as in learn-manipulator-control eq. (9.5): turning swings the frame about the base's
    // vertical axis; driving moves it along the heading.
    J(0, 0) = -(p.y() - q(1));
    J(1, 0) = p.x() - q(0);
    J(5, 0) = 1.0;
    J(0, 1) = std::cos(q(2));
    J(1, 1) = std::sin(q(2));
    if (k == kEndEffector) {
        const Eigen::Matrix3d R = (mc::base_transform(q.head<3>()) * mount_).topLeftCorner<3, 3>();
        const Matrix64d Ja = arm_.jacobian(q.tail<4>());
        J.block<3, 4>(0, 2) = R * Ja.topRows<3>();
        J.block<3, 4>(3, 2) = R * Ja.bottomRows<3>();
    }
    return J;
}

int MobileSwiftPro::state_index(int j) const {
    if (j == 0) return 2;
    if (j == 1) return -1;
    if (j >= 2 && j < 6) return j + 1;
    throw std::out_of_range("MobileSwiftPro: control index out of range");
}

Eigen::VectorXd MobileSwiftPro::integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& u, double dt) const {
    if (u.size() != 6) throw std::invalid_argument("MobileSwiftPro: u must have 6 entries");
    Eigen::VectorXd out(7);
    out.head<3>() = mc::integrate_unicycle(q.head<3>(), u(0), u(1), dt);
    out.tail<4>() = q.tail<4>() + dt * u.tail<4>();
    return out;
}

}  // namespace vehicle_manipulator_task_priority
