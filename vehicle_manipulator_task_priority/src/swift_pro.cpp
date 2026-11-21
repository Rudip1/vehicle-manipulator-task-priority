#include "vehicle_manipulator_task_priority/swift_pro.hpp"

#include <cmath>

namespace vehicle_manipulator_task_priority {

Eigen::Matrix4d SwiftPro::end_effector(const Vector4d& t) const {
    const double c = g_.shoulder_forward + g_.bracket_forward, h = g_.shoulder_height - g_.bracket_drop;
    const double r = c + g_.forearm * std::cos(t(2)) - g_.upper_arm * std::sin(t(1));
    const double yaw = t(0) + t(3);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = std::cos(yaw);
    T(0, 1) = -std::sin(yaw);
    T(1, 0) = std::sin(yaw);
    T(1, 1) = std::cos(yaw);
    T(0, 3) = r * std::cos(t(0));
    T(1, 3) = r * std::sin(t(0));
    T(2, 3) = h + g_.upper_arm * std::cos(t(1)) + g_.forearm * std::sin(t(2));
    return T;
}

Matrix64d SwiftPro::jacobian(const Vector4d& t) const {
    const double c = g_.shoulder_forward + g_.bracket_forward;
    const double r = c + g_.forearm * std::cos(t(2)) - g_.upper_arm * std::sin(t(1));
    const double c1 = std::cos(t(0)), s1 = std::sin(t(0));
    const double u = g_.upper_arm, f = g_.forearm;
    Matrix64d J = Matrix64d::Zero();
    J.col(0) << -r * s1, r * c1, 0.0, 0.0, 0.0, 1.0;  // waist: swings the reach radius, turns the cup
    const double dr2 = -u * std::cos(t(1)), dz2 = -u * std::sin(t(1));
    J.col(1) << dr2 * c1, dr2 * s1, dz2, 0.0, 0.0, 0.0;  // shoulder: in the vertical plane, cup stays level
    const double dr3 = -f * std::sin(t(2)), dz3 = f * std::cos(t(2));
    J.col(2) << dr3 * c1, dr3 * s1, dz3, 0.0, 0.0, 0.0;  // elbow (absolute forearm angle)
    J.col(3) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;            // wrist yaw: turns the cup in place
    return J;
}

Vector5d SwiftPro::to_serial(const Vector4d& t) {
    Vector5d s;
    s << t(0), t(1), t(2) - t(1), -t(2), t(3);
    return s;
}

Vector4d SwiftPro::from_serial(const Vector5d& s) {
    Vector4d t;
    t << s(0), s(1), s(2) + s(1), s(4);
    return t;
}

}  // namespace vehicle_manipulator_task_priority
