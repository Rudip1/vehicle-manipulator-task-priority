#pragma once
/// @file swift_pro.hpp
/// Kinematics of the uArm Swift Pro, a 4-DOF arm whose parallelogram keeps the end effector level.
///
/// Joint coordinates t = (t1, t2, t3, t4): t1 waist yaw, t2 angle of the upper arm from the vertical (negative
/// leans forward), t3 angle of the forearm from the horizontal (absolute, thanks to the parallelogram; negative
/// points down), t4 yaw of the end effector. In the arm base frame A (x forward at t1 = 0, z up):
///
///     r = c + f cos t3 - u sin t2,   p = (r cos t1, r sin t1, h + u cos t2 + f sin t3),   yaw = t1 + t4
///
/// with u the upper arm, f the forearm, c the sum of the horizontal offsets of the shoulder and of the suction
/// bracket and h the shoulder height minus the bracket drop.

#include <Eigen/Core>

namespace vehicle_manipulator_task_priority {

using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix64d = Eigen::Matrix<double, 6, 4>;

struct SwiftProGeometry {
    double shoulder_forward = 0.0132;  ///< horizontal offset of the shoulder axis from the waist axis [m]
    double shoulder_height = 0.108;    ///< height of the shoulder axis above the arm base [m]
    double upper_arm = 0.142;          ///< shoulder to elbow [m]
    double forearm = 0.1588;           ///< elbow to wrist [m]
    double bracket_forward = 0.0565;   ///< wrist to suction cup, horizontal [m]
    double bracket_drop = 0.0722;      ///< wrist to suction cup, downwards [m]
};

/// Joint ranges of t1..t4 [rad], as used on the real arm.
struct SwiftProLimits {
    Vector4d lower = (Vector4d() << -1.571, -1.571, -1.571, -1.571).finished();
    Vector4d upper = (Vector4d() << 1.571, 0.05, 0.05, 1.571).finished();
};

class SwiftPro {
  public:
    explicit SwiftPro(SwiftProGeometry geometry = {}, SwiftProLimits limits = {})
        : g_(geometry), limits_(limits) {}

    const SwiftProGeometry& geometry() const { return g_; }
    const SwiftProLimits& limits() const { return limits_; }

    /// Pose of the suction cup in the arm base frame A.
    Eigen::Matrix4d end_effector(const Vector4d& t) const;

    /// 6 x 4 geometric Jacobian of the suction cup in frame A, rows (v, w).
    Matrix64d jacobian(const Vector4d& t) const;

    /// The arm as a serial chain for the robot description (URDF): (waist, shoulder, elbow, wrist_level, wrist)
    /// = (t1, t2, t3 - t2, -t3, t4). The same linear map converts velocities.
    static Vector5d to_serial(const Vector4d& t);

    /// Inverse of to_serial (the wrist_level entry is ignored: it only keeps the cup level).
    static Vector4d from_serial(const Vector5d& s);

  private:
    SwiftProGeometry g_;
    SwiftProLimits limits_;
};

}  // namespace vehicle_manipulator_task_priority
