// The controller on the kinematic model: posture goals, suction-cup goals near and far (the base drives), joint
// limits kept, and a whole pick-and-place sequence.

#include <gtest/gtest.h>

#include <cmath>

#include "vehicle_manipulator_task_priority/controller.hpp"

namespace vm = vehicle_manipulator_task_priority;

namespace {

struct GoalRun {
    Eigen::VectorXd q;
    int steps = 0;
    bool reached = false;
    double max_speed = 0.0;
    double worst_limit_violation = -1e9;  // > 0: a joint left its range
};

GoalRun run_goal(vm::Controller& c, Eigen::VectorXd q, double dt = 0.02, int max_steps = 3000) {
    GoalRun r;
    const vm::SwiftProLimits& lim = c.model().arm().limits();
    for (r.steps = 0; r.steps < max_steps; ++r.steps) {
        const Eigen::VectorXd u = c.step(q);
        if (c.goal_reached()) {
            r.reached = true;
            break;
        }
        r.max_speed = std::max(r.max_speed, u.cwiseAbs().maxCoeff());
        q = c.model().integrate(q, u, dt);
        for (int i = 0; i < 4; ++i)
            r.worst_limit_violation = std::max({r.worst_limit_violation, q(3 + i) - lim.upper(i), lim.lower(i) - q(3 + i)});
    }
    r.q = q;
    return r;
}

Eigen::VectorXd start() {
    Eigen::VectorXd q(7);
    q << 0.0, 0.0, 0.0, 0.0, -0.3, -0.6, 0.0;  // carry posture: cup tucked in, above the base
    return q;
}

}  // namespace

TEST(Controller, PostureGoalMovesOnlyTheArm) {
    vm::Controller c(vm::MobileSwiftPro(vm::MobileSwiftPro::default_mount()));
    c.set_posture_goal((vm::Vector4d() << 0.5, -0.8, -1.0, -0.3).finished());
    const GoalRun r = run_goal(c, start());
    ASSERT_TRUE(r.reached);
    EXPECT_LT(r.q.head<3>().norm(), 1e-9);
    EXPECT_LE(r.max_speed, 0.3 + 1e-12);
}

TEST(Controller, NearGoalIsReachedWithinTheJointLimits) {
    vm::Controller c(vm::MobileSwiftPro(vm::MobileSwiftPro::default_mount()));
    c.set_position_goal(Eigen::Vector3d(0.33, 0.05, 0.13));
    const GoalRun r = run_goal(c, start());
    ASSERT_TRUE(r.reached);
    EXPECT_LT(r.worst_limit_violation, 0.0);
    EXPECT_LT((c.model().frame_pose(r.q, vm::kEndEffector).block<3, 1>(0, 3) - Eigen::Vector3d(0.33, 0.05, 0.13)).norm(), 0.01);
}

TEST(Controller, FarGoalMakesTheBaseDrive) {
    vm::Controller c(vm::MobileSwiftPro(vm::MobileSwiftPro::default_mount()));
    c.set_position_goal(Eigen::Vector3d(1.5, 1.0, 0.18));
    const GoalRun r = run_goal(c, start(), 0.02, 6000);
    ASSERT_TRUE(r.reached);
    EXPECT_GT(r.q.head<2>().norm(), 1.2);
    EXPECT_LT(r.worst_limit_violation, 0.0);
}

TEST(Controller, PickAndPlaceSequence) {
    // Box top at (1.2, 0.4, 0.13) to be placed at (-0.3, 1.2, 0.13): approach above, descend, lift, carry, set down.
    vm::Controller c(vm::MobileSwiftPro(vm::MobileSwiftPro::default_mount()));
    const Eigen::Vector3d pick(1.2, 0.4, 0.13), place(-0.3, 1.2, 0.13), up(0, 0, 0.06);
    Eigen::VectorXd q = start();
    const std::vector<Eigen::Vector3d> waypoints = {pick + up, pick, pick + up, place + up, place, place + up};
    for (const Eigen::Vector3d& w : waypoints) {
        c.set_position_goal(w);
        const GoalRun r = run_goal(c, q, 0.02, 6000);
        ASSERT_TRUE(r.reached) << "waypoint " << w.transpose();
        EXPECT_LT(r.worst_limit_violation, 0.0);
        q = r.q;
    }
    c.set_posture_goal(start().tail<4>());
    EXPECT_TRUE(run_goal(c, q).reached);
}

TEST(Controller, HeadingGoalTurnsOnlyTheBase) {
    vm::Controller c(vm::MobileSwiftPro(vm::MobileSwiftPro::default_mount()));
    c.set_heading_goal(1.2);
    const GoalRun r = run_goal(c, start());
    ASSERT_TRUE(r.reached);
    EXPECT_NEAR(r.q(2), 1.2, 0.03);
    EXPECT_LT(r.q.head<2>().norm(), 1e-9);                 // turned on the spot
    EXPECT_LT((r.q.tail<4>() - start().tail<4>()).norm(), 1e-9);  // arm untouched
}
