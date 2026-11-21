#!/usr/bin/env python3
"""Figure and numbers of the pick-and-place runs from the CSV logs of evaluate_run.py.

    python3 tools/plot_run.py docs/results/aruco_run1.csv docs/results/fixed_run1.csv docs/results/pick_place.png

Each CSV is drawn as a top view: ground-truth base path, the base path the odometry believed, the suction-cup path
and the box. The table printed for every CSV is the one in the README.
"""
import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

PICK, PLACE = (1.2, 0.4, 0.3), (-0.3, 1.2, 0.4)  # stand centre and yaw [rad]


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = {k: [] for k in rows[0]}
    for r in rows:
        for k, v in r.items():
            cols[k].append(v)
    num = {k: [float(x) for x in v] for k, v in cols.items() if k not in ("suction", "mission")}
    num["suction"], num["mission"] = cols["suction"], cols["mission"]
    return num


def summary(d):
    t = d["t"]
    started = next(i for i, m in enumerate(d["mission"]) if m == "running")
    ended = next((i for i, m in enumerate(d["mission"]) if m in ("succeeded", "failed")), len(t) - 1)
    xy = math.hypot(d["box_x"][-1] - PLACE[0], d["box_y"][-1] - PLACE[1])
    drift = [math.hypot(a - c, b - e) for a, b, c, e in zip(d["gt_x"], d["gt_y"], d["odom_x"], d["odom_y"])]
    yaw = [math.degrees(abs(math.remainder(a - b, 2 * math.pi))) for a, b in zip(d["gt_yaw"], d["odom_yaw"])]
    path = sum(math.hypot(d["gt_x"][i + 1] - d["gt_x"][i], d["gt_y"][i + 1] - d["gt_y"][i]) for i in range(len(t) - 1))
    return dict(result=d["mission"][ended], duration=t[ended] - t[started], box_error_cm=100 * xy,
                box_z=d["box_z"][-1], tilt=d["box_tilt_deg"][-1], drift_cm=100 * drift[-1], max_drift_cm=100 * max(drift),
                yaw_err_deg=yaw[-1], path_m=path)


def stand(ax, c, color):
    x, y, yaw = c
    r = Rectangle((x - 0.06, y - 0.06), 0.12, 0.12, angle=math.degrees(yaw), rotation_point="center", color=color,
                  alpha=0.5)
    ax.add_patch(r)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    csvs, out = argv[:-1], argv[-1]
    fig, axes = plt.subplots(1, len(csvs), figsize=(6.5 * len(csvs), 5.5), squeeze=False)
    for ax, path in zip(axes[0], csvs):
        d = load(path)
        s = summary(d)
        print(f"{path}: " + ", ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in s.items()))
        stand(ax, PICK, "tab:blue")
        stand(ax, PLACE, "tab:orange")
        ax.plot(d["gt_x"], d["gt_y"], "k-", lw=1.5, label="base (ground truth)")
        ax.plot(d["odom_x"], d["odom_y"], "k:", lw=1.2, label="base (odometry)")
        ax.plot(d["ee_x"], d["ee_y"], "-", color="tab:green", lw=1, alpha=0.8, label="suction cup (controller's estimate)")
        ax.plot(d["box_x"], d["box_y"], "-", color="tab:red", lw=2, label="box (ground truth)")
        ax.plot(d["box_x"][-1], d["box_y"][-1], "s", color="tab:red", ms=8)
        ax.plot(0, 0, "k^", ms=9)
        ax.set_aspect("equal")
        ax.set_xlim(-0.7, 1.6)
        ax.set_ylim(-0.4, 1.8)
        ax.grid(alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        name = path.rsplit("/", 1)[-1].replace(".csv", "")
        ax.set_title(f"{name}: box {s['box_error_cm']:.1f} cm from the stand centre, odometry drift {s['drift_cm']:.1f} cm",
                     fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
