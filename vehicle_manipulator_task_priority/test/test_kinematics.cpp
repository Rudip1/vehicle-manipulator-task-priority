// uArm Swift Pro and mobile-manipulator kinematics. References: the closed-form pose at a hand-worked
// configuration, finite differences, the serial chain of the robot description built from elementary
// transforms, and the integrated motion of the base.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <manipulator_control/transforms.hpp>
#include <random>

#include "vehicle_manipulator_task_priority/mobile_swift_pro.hpp"
#include "vehicle_manipulator_task_priority/swift_pro.hpp"

namespace vm = vehicle_manipulator_task_priority;

namespace {

vm::Vector4d random_joints(std::mt19937& gen) {
    std::uniform_real_distribution<double> d(-1.5, 0.05), yaw(-1.5, 1.5);
    vm::Vector4d t;
    t << yaw(gen), d(gen), d(gen), yaw(gen);
    return t;
}

Eigen::Matrix4d rot_axis(const Eigen::Vector3d& axis, double angle) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
    return T;
}

Eigen::Matrix4d trans(double x, double y, double z) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) << x, y, z;
    return T;
}

}  // namespace

TEST(SwiftPro, HandWorkedPose) {
    // t = 0: upper arm vertical, forearm horizontal. r = 0.0132 + 0.0565 + 0.1588, z = 0.108 - 0.0722 + 0.142.
    const vm::SwiftPro arm;
    const Eigen::Matrix4d T = arm.end_effector(vm::Vector4d::Zero());
    EXPECT_NEAR(T(0, 3), 0.2285, 1e-12);
    EXPECT_NEAR(T(1, 3), 0.0, 1e-12);
    EXPECT_NEAR(T(2, 3), 0.1778, 1e-12);
    EXPECT_TRUE((T.topLeftCorner<3, 3>().isIdentity(1e-12)));
}

TEST(SwiftPro, JacobianMatchesFiniteDifferences) {
    const vm::SwiftPro arm;
    std::mt19937 gen(1);
    for (int trial = 0; trial < 20; ++trial) {
        const vm::Vector4d t = random_joints(gen);
        vm::Matrix64d fd;
        for (int i = 0; i < 4; ++i) {
            vm::Vector4d tp = t, tm = t;
            tp(i) += 1e-6;
            tm(i) -= 1e-6;
            const Eigen::Matrix4d Tp = arm.end_effector(tp), Tm = arm.end_effector(tm);
            fd.block<3, 1>(0, i) = (Tp.block<3, 1>(0, 3) - Tm.block<3, 1>(0, 3)) / 2e-6;
            fd.block<3, 1>(3, i) = manipulator_control::log_so3(Tp.topLeftCorner<3, 3>() *
                                                                Tm.topLeftCorner<3, 3>().transpose()) / 2e-6;
        }
        EXPECT_LT((arm.jacobian(t) - fd).cwiseAbs().maxCoeff(), 1e-8);
    }
}

TEST(SwiftPro, SerialChainOfTheRobotDescriptionGivesTheSamePose) {
    // The URDF models the parallelogram as a serial chain: waist (z), shoulder, elbow and wrist_level about
    // (0, -1, 0), wrist (z), with joint values to_serial(t). Built here from elementary transforms.
    const vm::SwiftProGeometry g;
    const Eigen::Vector3d down_y(0.0, -1.0, 0.0), z(0.0, 0.0, 1.0);
    std::mt19937 gen(2);
    for (int trial = 0; trial < 20; ++trial) {
        const vm::Vector4d t = random_joints(gen);
        const vm::Vector5d s = vm::SwiftPro::to_serial(t);
        const Eigen::Matrix4d T = rot_axis(z, s(0)) * trans(g.shoulder_forward, 0, g.shoulder_height) *
                                  rot_axis(down_y, s(1)) * trans(0, 0, g.upper_arm) * rot_axis(down_y, s(2)) *
                                  trans(g.forearm, 0, 0) * rot_axis(down_y, s(3)) *
                                  trans(g.bracket_forward, 0, -g.bracket_drop) * rot_axis(z, s(4));
        EXPECT_LT((T - vm::SwiftPro().end_effector(t)).cwiseAbs().maxCoeff(), 1e-12);
        EXPECT_LT((vm::SwiftPro::from_serial(s) - t).norm(), 1e-15);
    }
}

TEST(MobileSwiftPro, JacobianMatchesFiniteDifferencesOfTheMotion) {
    const vm::MobileSwiftPro robot(vm::MobileSwiftPro::default_mount());
    std::mt19937 gen(3);
    for (int trial = 0; trial < 10; ++trial) {
        Eigen::VectorXd q(7);
        q << 0.4, -0.2, 0.7, random_joints(gen);
        for (int k : {vm::kEndEffector, vm::kBase, vm::kArmBase}) {
            const Eigen::MatrixXd J = robot.frame_jacobian(q, k);
            for (int j = 0; j < 6; ++j) {
                Eigen::VectorXd du = Eigen::VectorXd::Zero(6);
                du(j) = 1e-6;
                const Eigen::Matrix4d Tp = robot.frame_pose(robot.integrate(q, du, 1.0), k);
                const Eigen::Matrix4d Tm = robot.frame_pose(robot.integrate(q, -du, 1.0), k);
                Eigen::Matrix<double, 6, 1> fd;
                fd.head<3>() = (Tp.block<3, 1>(0, 3) - Tm.block<3, 1>(0, 3)) / 2e-6;
                fd.tail<3>() = manipulator_control::log_so3(Tp.topLeftCorner<3, 3>() *
                                                            Tm.topLeftCorner<3, 3>().transpose()) / 2e-6;
                EXPECT_LT((J.col(j) - fd).cwiseAbs().maxCoeff(), 1e-7) << "frame " << k << " column " << j;
            }
        }
    }
}
