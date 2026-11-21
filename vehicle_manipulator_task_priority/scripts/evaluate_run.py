#!/usr/bin/env python3
"""Record a pick-and-place run from ground truth and summarise it.

Run next to the launch file (same ROS_DOMAIN_ID); it stops when the mission reports success or failure, or at the
time limit, writes a CSV and prints the summary:

    ros2 run vehicle_manipulator_task_priority evaluate_run.py --csv run.csv --time-limit 300

Box placed correctly = its centre within 3 cm of the place stand centre (horizontally), resting on the stand (centre
at 0.115 m) and upright (tilt below 10 degrees).
"""
import argparse
import csv
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

PLACE = (-0.3, 1.2)
BOX_REST_Z = 0.08 + 0.035


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tilt_of(q):
    # angle between the box's z axis and the world z axis
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


class Recorder(Node):
    def __init__(self):
        super().__init__("evaluate_run", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.gt = self.odom = self.ee = self.box = None
        self.status = None
        self.suction = "detached"
        self.rows = []
        self.start = None
        self.create_subscription(Odometry, "/ground_truth/odometry", lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 10)
        self.create_subscription(PoseStamped, "/end_effector", lambda m: setattr(self, "ee", m), 10)
        self.create_subscription(Pose, "/ground_truth/box_pose", lambda m: setattr(self, "box", m), 10)
        self.create_subscription(String, "/suction/state", lambda m: setattr(self, "suction", m.data), 10)
        qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/mission/status", self.on_status, qos)
        self.create_timer(0.1, self.sample)

    def on_status(self, m):
        self.status = m.data
        if m.data == "running" and self.start is None:
            self.start = self.get_clock().now()

    def sample(self):
        if None in (self.gt, self.odom, self.ee, self.box):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        g, o, e, b = self.gt.pose.pose, self.odom.pose.pose, self.ee.pose.position, self.box
        self.rows.append([round(t, 2), g.position.x, g.position.y, yaw_of(g.orientation), o.position.x,
                          o.position.y, yaw_of(o.orientation), e.x, e.y, e.z, b.position.x, b.position.y,
                          b.position.z, tilt_of(b.orientation), self.suction, self.status or ""])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="run.csv")
    ap.add_argument("--time-limit", type=float, default=300.0, help="wall-clock seconds")
    args = ap.parse_args()
    rclpy.init()
    node = Recorder()
    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.time_limit and node.status not in ("succeeded", "failed"):
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(20):  # a last second of samples after the end
        rclpy.spin_once(node, timeout_sec=0.05)
    node.sample()  # the final row carries the final mission status
    header = ["t", "gt_x", "gt_y", "gt_yaw", "odom_x", "odom_y", "odom_yaw", "ee_x", "ee_y", "ee_z", "box_x", "box_y",
              "box_z", "box_tilt_deg", "suction", "mission"]
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(node.rows)
    if not node.rows:
        print("RESULT no data")
        return 2
    last = node.rows[-1]
    duration = (node.get_clock().now() - node.start).nanoseconds * 1e-9 if node.start else float("nan")
    xy_err = math.hypot(last[10] - PLACE[0], last[11] - PLACE[1])
    drift = math.hypot(last[1] - last[4], last[2] - last[5])
    placed = xy_err < 0.03 and abs(last[12] - BOX_REST_Z) < 0.01 and last[13] < 10.0
    print(f"RESULT mission={node.status} duration={duration:.1f}s box_xy_error={xy_err * 100:.1f}cm "
          f"box_z={last[12]:.3f}m tilt={last[13]:.1f}deg placed={placed} odom_drift={drift * 100:.1f}cm "
          f"samples={len(node.rows)}")
    rclpy.shutdown()
    return 0 if (node.status == "succeeded" and placed) else 1


if __name__ == "__main__":
    raise SystemExit(main())
