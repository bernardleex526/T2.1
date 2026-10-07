#!/usr/bin/env python3
"""Static-drift check: does a STATIONARY robot stay put?

WHY THIS EXISTS
---------------
This is the cheapest end-to-end test of an extrinsic / time-sync / scale error,
and the review lists it as the acceptance gate for all three
("静止漂移测试 < 5cm/10min").  A wrong ``ext_il``, a wrong ``imu_acc_scale`` or a
mis-scaled per-point time all show up as the published odometry sliding away
while the robot is physically still.  It needs no ground truth, no total station
and no motion - just a stationary recording.

WHAT IT MEASURES
----------------
Given the published odometry (``/fastlio2/lio_odom``) from a stationary
recording:

  * drift = displacement of the final pose from the mean of the first N seconds;
  * drift rate [m/min] and [m/h];
  * yaw drift [deg/min];
  * the largest single-sample jump, which is the signature of a clock-domain
    fault or an out-of-order flush rather than a smooth scale error.

It is deliberately framed as DRIFT OVER TIME, so the verdict does not depend on
an arbitrary recording length.

USAGE
-----
    # Odometry from a bag (stationary robot, >= 10 min)
    python3 tools/odom_static_drift.py --bag static_run --topic /fastlio2/lio_odom

    # A TUM trajectory file (the simulation scripts already write these)
    python3 tools/odom_static_drift.py --tum odom.tum

    # An .npz with times + x/y/z columns
    python3 tools/odom_static_drift.py --npz static.npz

EXIT CODES
----------
0 = within the configured drift budget
2 = exceeds it (with the numbers printed)
1 = input could not be read

See docs/test_scenarios.md and docs/calibration_procedure.md.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_io  # noqa: E402


@dataclass
class Pose:
    times: np.ndarray     # (N,)
    xyz: np.ndarray       # (N,3)
    yaw: Optional[np.ndarray]  # (N,) radians, when available


def read_tum(path: str) -> Pose:
    """TUM trajectory: 'timestamp tx ty tz qx qy qz qw' per line."""
    times, xyz, quats = [], [], []
    with open(path, "r") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                raise probe_io.ProbeInputError(
                    f"{path}:{lineno}: expected 8 TUM columns, got {len(parts)}"
                )
            try:
                vals = [float(v) for v in parts[:8]]
            except ValueError:
                raise probe_io.ProbeInputError(
                    f"{path}:{lineno}: non-numeric TUM row"
                ) from None
            times.append(vals[0])
            xyz.append(vals[1:4])
            quats.append(vals[4:8])
    if not times:
        raise probe_io.ProbeInputError(f"{path}: no pose rows")
    yaw = np.array([_quat_yaw(q) for q in quats])
    return Pose(times=np.asarray(times), xyz=np.asarray(xyz), yaw=yaw)


def _quat_yaw(q: Sequence[float]) -> float:
    """Yaw [rad] from a quaternion given as (qx, qy, qz, qw)."""
    qx, qy, qz, qw = q
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(np.arctan2(siny, cosy))


def read_npz(path: str) -> Pose:
    with np.load(path, allow_pickle=False) as z:
        keys = set(z.files)
        if "times" not in keys:
            raise probe_io.ProbeInputError(f"{path}: needs a 'times' array")
        times = np.asarray(z["times"], dtype=np.float64)
        if all(k in keys for k in ("x", "y", "z")):
            xyz = np.column_stack([z["x"], z["y"], z["z"]]).astype(np.float64)
        elif "xyz" in keys:
            xyz = np.asarray(z["xyz"], dtype=np.float64)
        else:
            raise probe_io.ProbeInputError(f"{path}: needs x/y/z or xyz")
        yaw = np.asarray(z["yaw"], dtype=np.float64) if "yaw" in keys else None
    return Pose(times=times, xyz=xyz, yaw=yaw)


def read_bag(path: str, topic: str, limit: int = 0) -> Pose:
    """Read nav_msgs/Odometry poses from a ROS 2 bag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise probe_io.ProbeInputError(
            f"reading a bag needs ROS 2 python packages: {exc}"
        ) from None

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
    except Exception as exc:
        raise probe_io.ProbeInputError(f"cannot open bag {path}: {exc}") from None

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise probe_io.ProbeInputError(
            f"{path}: topic {topic!r} not in bag. Present: {', '.join(sorted(topics))}"
        )
    if not topics[topic].endswith("nav_msgs/msg/Odometry"):
        raise probe_io.ProbeInputError(
            f"{path}:{topic} is {topics[topic]}, not nav_msgs/msg/Odometry"
        )
    msg_type = get_message(topics[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    times, xyz, yaws = [], [], []
    while reader.has_next():
        _, raw, _ = reader.read_next()
        m = deserialize_message(raw, msg_type)
        hdr = m.header.stamp
        times.append(hdr.sec + hdr.nanosec * 1e-9)
        p = m.pose.pose.position
        xyz.append((p.x, p.y, p.z))
        q = m.pose.pose.orientation
        yaws.append(_quat_yaw((q.x, q.y, q.z, q.w)))
        if limit and len(times) >= limit:
            break
    if not times:
        raise probe_io.ProbeInputError(f"{path}: no odometry on {topic!r}")
    return Pose(times=np.asarray(times), xyz=np.asarray(xyz), yaw=np.asarray(yaws))


@dataclass
class DriftReport:
    n: int
    duration_s: float
    baseline_s: float
    baseline_xyz: np.ndarray
    final_xyz: np.ndarray
    drift_m: float
    drift_rate_m_per_min: float
    drift_rate_m_per_h: float
    yaw_drift_deg: Optional[float]
    yaw_rate_deg_per_min: Optional[float]
    max_jump_m: float
    max_jump_t: float
    path_length_m: float


def analyse(pose: Pose, baseline_s: float = 10.0) -> DriftReport:
    n = pose.times.size
    if n < 2:
        raise probe_io.ProbeInputError("need at least 2 poses")

    duration = float(pose.times[-1] - pose.times[0])
    t0 = float(pose.times[0])
    base_mask = (pose.times - t0) <= baseline_s
    if base_mask.sum() < 2:
        base_mask = np.zeros(n, dtype=bool)
        base_mask[0] = True
    baseline = pose.xyz[base_mask].mean(axis=0)
    final = pose.xyz[-1]

    drift = float(np.linalg.norm(final - baseline))
    minutes = duration / 60.0 if duration > 0 else 0.0
    rate_min = drift / minutes if minutes > 0 else 0.0

    yaw_drift = yaw_rate = None
    if pose.yaw is not None and n > 1:
        base_yaw = float(np.arctan2(np.sin(pose.yaw[base_mask]).mean(),
                                    np.cos(pose.yaw[base_mask]).mean()))
        d = pose.yaw[-1] - base_yaw
        yaw_drift = float(np.degrees(np.arctan2(np.sin(d), np.cos(d))))
        yaw_rate = yaw_drift / minutes if minutes > 0 else 0.0

    # Largest single-sample jump: a clock fault looks like this, a scale error
    # does not.
    steps = np.linalg.norm(np.diff(pose.xyz, axis=0), axis=1)
    if steps.size:
        k = int(np.argmax(steps))
        max_jump, max_jump_t = float(steps[k]), float(pose.times[k + 1] - t0)
    else:
        max_jump, max_jump_t = 0.0, 0.0
    path_length = float(steps.sum())

    return DriftReport(n=n, duration_s=duration, baseline_s=baseline_s,
                       baseline_xyz=baseline, final_xyz=final, drift_m=drift,
                       drift_rate_m_per_min=rate_min,
                       drift_rate_m_per_h=rate_min * 60.0,
                       yaw_drift_deg=yaw_drift, yaw_rate_deg_per_min=yaw_rate,
                       max_jump_m=max_jump, max_jump_t=max_jump_t,
                       path_length_m=path_length)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("WHY THIS EXISTS")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="ROS 2 bag directory")
    src.add_argument("--tum", help="TUM trajectory file")
    src.add_argument("--npz", help=".npz with times + x/y/z (+ optional yaw)")
    ap.add_argument("--topic", default="/fastlio2/lio_odom",
                    help="odometry topic when reading a bag")
    ap.add_argument("--limit", type=int, default=0, help="max messages (0 = all)")
    ap.add_argument("--baseline-s", type=float, default=10.0,
                    help="seconds at the start used as the reference pose (default 10)")
    ap.add_argument("--max-drift-m", type=float, default=0.05,
                    help="drift budget in metres for the WHOLE recording "
                         "(default 0.05 = the review's 5 cm class)")
    ap.add_argument("--max-drift-per-min", type=float, default=0.005,
                    help="drift-rate budget [m/min] (default 0.005 = 5 cm per 10 min)")
    ap.add_argument("--max-jump-m", type=float, default=0.10,
                    help="largest acceptable single-sample jump [m] (default 0.10)")
    ap.add_argument("--max-yaw-deg", type=float, default=1.0,
                    help="acceptable total yaw drift over the recording [deg] "
                         "(default 1.0). Yaw is a separate failure mode: a gyro-bias or "
                         "rotation-extrinsic error can leave position perfect while the "
                         "heading rotates, which silently breaks the localizer.")
    args = ap.parse_args(argv)

    try:
        if args.tum:
            pose = read_tum(args.tum)
        elif args.npz:
            pose = read_npz(args.npz)
        else:
            pose = read_bag(args.bag, args.topic, args.limit)
    except probe_io.ProbeInputError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    try:
        r = analyse(pose, baseline_s=args.baseline_s)
    except probe_io.ProbeInputError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print("=" * 74)
    print("T2.1 static-drift check")
    print("=" * 74)
    print(f"poses            : {r.n}")
    print(f"duration         : {r.duration_s:.1f} s ({r.duration_s / 60.0:.2f} min)")
    print(f"baseline window  : first {r.baseline_s:g} s")
    print(f"baseline position: [{r.baseline_xyz[0]:+.4f} {r.baseline_xyz[1]:+.4f} "
          f"{r.baseline_xyz[2]:+.4f}] m")
    print(f"final position   : [{r.final_xyz[0]:+.4f} {r.final_xyz[1]:+.4f} "
          f"{r.final_xyz[2]:+.4f}] m")
    print()
    print(f"drift            : {r.drift_m * 100:.2f} cm "
          f"({r.drift_m:.5f} m)")
    print(f"drift rate       : {r.drift_rate_m_per_min * 100:.3f} cm/min "
          f"({r.drift_rate_m_per_h * 100:.2f} cm/h)")
    if r.yaw_drift_deg is not None:
        print(f"yaw drift        : {r.yaw_drift_deg:+.3f} deg "
              f"({r.yaw_rate_deg_per_min:+.4f} deg/min)")
    print(f"largest step     : {r.max_jump_m * 100:.2f} cm at t={r.max_jump_t:.2f} s")
    print(f"path length      : {r.path_length_m * 100:.2f} cm "
          "(a truly static robot should be ~0)")
    print()

    failures: List[str] = []
    if r.drift_m > args.max_drift_m:
        failures.append(f"drift {r.drift_m * 100:.2f} cm > budget "
                        f"{args.max_drift_m * 100:.2f} cm")
    if r.drift_rate_m_per_min > args.max_drift_per_min:
        failures.append(f"drift rate {r.drift_rate_m_per_min * 100:.3f} cm/min > budget "
                        f"{args.max_drift_per_min * 100:.3f} cm/min")
    if r.max_jump_m > args.max_jump_m:
        failures.append(f"largest step {r.max_jump_m * 100:.2f} cm > budget "
                        f"{args.max_jump_m * 100:.2f} cm")
    if r.yaw_drift_deg is not None and abs(r.yaw_drift_deg) > args.max_yaw_deg:
        failures.append(f"yaw drift {r.yaw_drift_deg:+.3f} deg > budget "
                        f"{args.max_yaw_deg:g} deg")

    print("--- verdict ---")
    if failures:
        for f in failures:
            print(f"  FAIL: {f}")
        print()
        print("Interpretation (see docs/tuning_guide.md):")
        if r.max_jump_m > args.max_jump_m:
            print("  * A large single step points at a CLOCK problem: LiDAR and IMU in")
            print("    different time domains, or a replayed bag without --clock. Check")
            print("    the time-sync path before touching ext_il.")
        print("  * Smooth drift with no jump points at the EXTRINSIC or the accel scale:")
        print("    verify ext_il, then imu_acc_scale (compare the static mean |a| with 9.81).")
        if (r.yaw_drift_deg is not None and abs(r.yaw_drift_deg) > args.max_yaw_deg
                and r.drift_m <= args.max_drift_m):
            print("  * Yaw is drifting while POSITION is fine. That is a gyro-bias or a")
            print("    ROTATION error in ext_il, not a translation error - and it is the")
            print("    more dangerous case, because the map looks correct while the")
            print("    published heading slowly rotates. Re-check the gyro bias and the")
            print("    rotational part of ext_il.")
        elif r.yaw_drift_deg is not None and abs(r.yaw_drift_deg) > args.max_yaw_deg:
            print("  * Large YAW drift with little translation is usually a gyro bias or a")
            print("    rotation error in ext_il, not a translation error.")
        return 2

    print("  PASS: within budget")
    if r.duration_s < 600.0:
        print()
        print(f"  NOTE: this recording is only {r.duration_s / 60.0:.1f} min. The review's")
        print("        acceptance target is a 10-minute static run; a short recording can")
        print("        pass while a slow drift is still present. Re-run for >= 10 min.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
