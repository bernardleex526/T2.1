#!/usr/bin/env python3
"""Generate the synthetic ROS 2 bag (sensor_msgs/PointCloud2 + sensor_msgs/Imu) used by the
FastLIO2 simulation harness.

The bag contains exactly the two topics the node subscribes to in ``pointcloud2`` mode:

  /rslidar_points  sensor_msgs/msg/PointCloud2  10 Hz   ~30k points/scan, fields x y z
                                                         intensity time(s since scan start)
  /imu/data        sensor_msgs/msg/Imu          200 Hz  specific force [m/s^2] + gyro [rad/s]

The scene is a closed 14 m x 14 m x 3 m room with four pillars; the sensor trots in place for
6 s and then walks a corner-rounded 10 m x 10 m rectangle at 0.5 m/s.  Motion, in-scan
distortion, the 10-30 Hz gait vibration and the IMU noise are all derived deterministically
from ``test_trajectory.json`` (see traj_common.py), so two runs produce VALUE-identical messages:
the decoded message content is bit-identical (verified by comparing two generations), while the
bag FILE bytes can differ in the CDR padding of the message metadata (uninitialized bytes that a
deserializer skips), so bag hashes are not a reproducibility check - compare decoded content.

The pose geometry is rendered per ray at the ray's own emission time, and the header stamp is
the scan START while the ``time`` field is seconds since that start -- the exact convention the
node expects (``Utils::pcl2_to_PCL`` + ``syncPackage``).

ROS imports are deliberately lazy: ``--check`` runs on a bare python3 (no sourced ROS).

Usage:
    python3 generate_test_bag.py                        # -> ./test_bag (next to this file)
    python3 generate_test_bag.py --out /tmp/bag --duration 20
    python3 generate_test_bag.py --check                # plan/statistics only, no ROS needed
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traj_common as tc  # noqa: E402

DEFAULT_TRAJECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_trajectory.json")
POINT_TOPIC = "/rslidar_points"
IMU_TOPIC = "/imu/data"
# FIELDS: x y z intensity time  (FLOAT32 each -> point_step 20)
PCD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                      ("intensity", "<f4"), ("time", "<f4")])


# ----------------------------------------------------------------- ROS-free core ----
def scan_starts(traj, duration_s):
    """Scan start times [s] (header stamps).  Scan k spans [k*period, (k+1)*period)."""
    period = 1.0 / float(traj["sensors"]["lidar_hz"])
    n = int(round(duration_s * float(traj["sensors"]["lidar_hz"])))
    return np.arange(n) * period


def imu_times(traj, duration_s):
    dt = 1.0 / float(traj["sensors"]["imu_hz"])
    n = int(round(duration_s * float(traj["sensors"]["imu_hz"])))
    return np.arange(n) * dt


def plan(traj, duration_s):
    """Human-readable + numeric plan for --check and for the pipeline's manifest."""
    sensors = traj["sensors"]
    scans = scan_starts(traj, duration_s)
    n_pts = int(sensors["lidar_vertical_beams"]) * int(sensors["lidar_horizontal_beams"])
    gt = tc.build_ground_truth(traj, duration_s=duration_s)
    exp = traj.get("fastlio2_init_expectation", {})
    win = float(exp.get("imu_init_window_s", 3.0))
    gyro_lim = float(exp.get("imu_init_static_gyro_std", 0.005))
    acc_lim = float(exp.get("imu_init_static_acc_dev", 0.3))
    wait_lim = float(exp.get("imu_init_max_wait_s", 5.0))
    # the init verdict is a decision about what the IMU REPORTS, so probe the measured stream
    m = gt["t"] <= win
    gyro_std = float(gt["gyro_meas"][m].std(axis=0).max())
    acc_dev = float((gt["accel_meas"][m] - gt["accel_meas"][m].mean(axis=0)).std(axis=0).max())
    acc_mean = gt["accel_meas"][m].mean(axis=0)
    return {
        "duration_s": duration_s,
        "scans": len(scans),
        "points_per_scan_nominal": n_pts,
        "imu_samples": len(imu_times(traj, duration_s)),
        "first_scan_start_s": float(scans[0]) if len(scans) else None,
        "last_scan_end_s": float(scans[-1] + 1.0 / float(sensors["lidar_hz"])) if len(scans) else None,
        "init_window_s": win,
        "init_window_gyro_std_rps": gyro_std,
        "init_window_accel_dev_mps2": acc_dev,
        "init_window_accel_mean_mps2": [float(x) for x in acc_mean],
        "init_window_accel_mean_norm": float(np.linalg.norm(acc_mean)),
        # the B3 edge case this bag is built to exercise:
        "init_static_gyro_std_limit": gyro_lim,
        "init_static_acc_dev_limit": acc_lim,
        "init_max_wait_s": wait_lim,
        "static_verdict_expected": bool(gyro_std < gyro_lim and acc_dev < acc_lim),
        "max_wait_fallback_expected": bool(not (gyro_std < gyro_lim and acc_dev < acc_lim)),
    }


def iter_messages(traj, gt, duration_s, rng, dirs, az_frac, scene, sensors):
    """Yield ``(kind, storage_stamp, header_stamp, payload)`` in READINESS order.

    A LiDAR message carries a whole sweep, so it is not *available* before its LAST point has
    been measured: the message is therefore stored/published at ``header + frame_span`` (its
    readiness), while its HEADER stays the emission time of its FIRST point (the measurement
    time the node parses).  The IMU samples are available when they are measured.  Messages are
    emitted in storage-stamp order, so the bag's own timeline is the physical availability
    timeline and an observed input-arrival -> output-arrival delay needs no span correction.

    The per-point ``time`` field is the point's emission time rebased on the first point of the
    same sweep (``pts[0, 4] == 0``), which is what ``Utils::pcl2_to_PCL`` (``t0`` = first
    range-filtered point) + ``syncPackage`` (``cloud_end_time = header + last curvature``)
    require.
    """
    period = 1.0 / float(sensors["lidar_hz"])
    imu_dt = 1.0 / float(sensors["imu_hz"])
    starts = scan_starts(traj, duration_s)
    t0 = float(traj["t0_epoch_s"])
    n_imu = len(imu_times(traj, duration_s))

    gt_t = gt["t"]
    i_imu, i_lid = 0, 0
    scan = None                       # (storage_stamp, header_stamp, payload)
    while i_imu < n_imu or scan is not None or i_lid < len(starts):
        if scan is None and i_lid < len(starts):
            t_scan = float(starts[i_lid])
            pts, t_first = tc.render_scan(gt, t_scan, period, dirs, az_frac, scene, rng, sensors)
            header = t0 + t_scan + t_first
            t_last = float(pts[-1, 4]) if pts.shape[0] else 0.0
            scan = (header + t_last, header, pts)
            i_lid += 1
        imu_stamp = t0 + i_imu * imu_dt if i_imu < n_imu else None
        if imu_stamp is not None and (scan is None or imu_stamp <= scan[0]):
            j = int(round((imu_stamp - t0) / gt["grid_dt"]))
            j = min(max(j, 0), len(gt_t) - 1)
            yield ("imu", imu_stamp, imu_stamp,
                   (gt["accel_meas"][j], gt["gyro_meas"][j]))
            i_imu += 1
        else:
            yield ("lidar", scan[0], scan[1], scan[2])
            scan = None


# ------------------------------------------------------------------- ROS writers ----
def _stamp(sec):
    from builtin_interfaces.msg import Time
    s = int(sec)
    t = Time()
    t.sec = s
    t.nanosec = int(round((sec - s) * 1e9))
    if t.nanosec >= 1000000000:
        t.sec += 1
        t.nanosec -= 1000000000
    return t


def make_pointcloud2(pts, stamp, frame_id="rslidar"):
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.stamp = _stamp(stamp)
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "intensity", "time"))]
    msg.is_bigendian = False
    msg.point_step = PCD_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    arr = np.empty(pts.shape[0], dtype=PCD_DTYPE)
    arr["x"], arr["y"], arr["z"], arr["intensity"], arr["time"] = (
        pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3], pts[:, 4])
    msg.data = arr.tobytes()
    return msg


def make_imu(acc, gyro, stamp, frame_id="imu_link"):
    from sensor_msgs.msg import Imu
    msg = Imu()
    msg.header.stamp = _stamp(stamp)
    msg.header.frame_id = frame_id
    msg.orientation.w = 1.0
    msg.orientation_covariance[0] = -1.0        # orientation unused
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    for i in range(3):
        msg.angular_velocity_covariance[i * 4] = 1e-4
        msg.linear_acceleration_covariance[i * 4] = 1e-3
    return msg


def write_bag(out_dir, traj, duration_s, overwrite=True, quiet=False):
    from rclpy.serialization import serialize_message
    from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata

    if os.path.exists(out_dir):
        if not overwrite:
            raise SystemExit("bag already exists: %s (use --overwrite)" % out_dir)
        shutil.rmtree(out_dir)
    os.makedirs(os.path.dirname(os.path.abspath(out_dir)), exist_ok=True)

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=out_dir, storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name=POINT_TOPIC, type="sensor_msgs/msg/PointCloud2",
                                      serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name=IMU_TOPIC, type="sensor_msgs/msg/Imu",
                                      serialization_format="cdr"))

    sensors, scene = traj["sensors"], traj["scene"]
    dirs, az_frac = tc.lidar_directions(sensors)
    gt = tc.build_ground_truth(traj, duration_s=duration_s)
    rng = np.random.default_rng(int(traj.get("imu_noise", {}).get("seed", 20260930)) + 17)

    n_lidar = n_imu = n_pts = 0
    period = 1.0 / float(sensors["lidar_hz"])
    t0 = float(traj["t0_epoch_s"])
    time_check = {"scans_checked": 0, "max_header_minus_first_point_s": 0.0,
                  "max_frame_span_s": 0.0, "min_frame_span_s": float("inf"),
                  "max_ready_minus_header_plus_span_s": 0.0,
                  "first_point_time_field_nonzero_n": 0, "empty_scans": 0}
    for kind, storage_stamp, header_stamp, payload in iter_messages(
            traj, gt, duration_s, rng, dirs, az_frac, scene, sensors):
        if kind == "lidar":
            if payload.shape[0] == 0:
                time_check["empty_scans"] += 1
            else:
                t_first = float(payload[0, 4])
                t_last = float(payload[-1, 4])
                scan_start = float(round((header_stamp - t0 - t_first) / period)) * period
                # header must be the scan start + the first emitted point's emission time
                err = abs(header_stamp - (t0 + scan_start + t_first))
                time_check["max_header_minus_first_point_s"] = max(
                    time_check["max_header_minus_first_point_s"], err)
                # the message is READY (and is published) when its LAST point has been measured
                ready_err = abs(storage_stamp - (header_stamp + t_last))
                time_check["max_ready_minus_header_plus_span_s"] = max(
                    time_check.get("max_ready_minus_header_plus_span_s", 0.0), ready_err)
                time_check["max_frame_span_s"] = max(time_check["max_frame_span_s"], t_last)
                time_check["min_frame_span_s"] = min(time_check["min_frame_span_s"], t_last)
                time_check["first_point_time_field_nonzero_n"] += int(t_first != 0.0)
                time_check["scans_checked"] += 1
            writer.write(POINT_TOPIC, serialize_message(make_pointcloud2(payload, header_stamp)),
                         int(round(storage_stamp * 1e9)))
            n_lidar += 1
            n_pts += int(payload.shape[0])
        else:
            acc, gyro = payload
            writer.write(IMU_TOPIC, serialize_message(make_imu(acc, gyro, storage_stamp)),
                         int(round(storage_stamp * 1e9)))
            n_imu += 1
    del writer
    if not np.isfinite(time_check["min_frame_span_s"]):
        time_check["min_frame_span_s"] = None
    if not quiet:
        size_mb = sum(os.path.getsize(os.path.join(dp, f))
                      for dp, _, fs in os.walk(out_dir) for f in fs) / 1e6
        print("[generate_test_bag] wrote %s: %d scans (%d points, %.1f pts/scan avg), %d IMU, %.1f MB"
              % (out_dir, n_lidar, n_pts, n_pts / max(1, n_lidar), n_imu, size_mb))
        print("[generate_test_bag] time contract: header=first-point measurement "
              "(max err %.3g s), per-point field rebased on index 0, frame span "
              "%.4f..%.4f s (period %.3f s); scan published at its LAST-point readiness "
              "(max err %.3g s)" % (time_check["max_header_minus_first_point_s"],
                 time_check["min_frame_span_s"] or 0.0, time_check["max_frame_span_s"], period,
                 time_check["max_ready_minus_header_plus_span_s"]))
    return {"scans": n_lidar, "points": n_pts, "imu_samples": n_imu, "path": out_dir,
            "time_consistency": time_check, "scan_period_s": period,
            "range_limits_m": [float(sensors["lidar_min_range_m"]),
                               float(sensors["lidar_max_range_m"])]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.join(os.path.dirname(DEFAULT_TRAJECTORY), "test_bag"),
                    help="output bag directory (default: ./test_bag next to this script)")
    ap.add_argument("--trajectory", default=DEFAULT_TRAJECTORY)
    ap.add_argument("--duration", type=float, default=None, help="override trajectory duration_s")
    ap.add_argument("--check", action="store_true", help="print the generation plan and exit (no ROS needed)")
    ap.add_argument("--clean-start", action="store_true",
                    help="ablation: remove the gait vibration, roll/pitch wobble and body bob, so the "
                         "trot-in-place prologue is a genuinely static init window (exercises the "
                         "static-window init path instead of the max-wait fallback, and makes "
                         "acc_normalize a no-op because |acc_mean| is exactly g)")
    ap.add_argument("--seed", type=int, default=None, help="override the IMU/map noise seed")
    ap.add_argument("--min-range", type=float, default=None,
                    help="override the sensor min range [m]; pass the effective lidar_min_range of "
                         "the node config so the emitted points cannot be re-filtered (the node "
                         "adopts t0 from the first point IT retains)")
    ap.add_argument("--max-range", type=float, default=None,
                    help="override the sensor max range [m]; see --min-range")
    ap.add_argument("--no-overwrite", dest="overwrite", action="store_false", default=True)
    args = ap.parse_args(argv)

    traj = tc.apply_variant(tc.load_trajectory(args.trajectory),
                            clean_start=args.clean_start, seed=args.seed)
    duration = float(traj["duration_s"] if args.duration is None else args.duration)
    if duration <= 0:
        raise SystemExit("duration must be positive")
    if args.min_range is not None:
        traj["sensors"]["lidar_min_range_m"] = float(args.min_range)
    if args.max_range is not None:
        traj["sensors"]["lidar_max_range_m"] = float(args.max_range)
    if float(traj["sensors"]["lidar_max_range_m"]) <= float(traj["sensors"]["lidar_min_range_m"]):
        raise SystemExit("max range must exceed min range")

    info = plan(traj, duration)
    if args.check:
        for k, v in info.items():
            print("  %-30s %s" % (k, v))
        checks = tc.self_check(traj, duration_s=min(2.0, duration), seed=0)
        print("  -- geometry/time self-consistency --")
        for k, v in checks.items():
            print("  %-30s %s" % (k, v))
        failures = []
        if checks["raycast_vs_reference_max_err_m"] > 1e-9:
            failures.append("vectorised raycast disagrees with the per-ray reference")
        if checks["rays_without_hit"] != 0:
            failures.append("rays escape the closed room box")
        if checks["frozen_scan_max_abs_diff_m"] > 0.0:
            failures.append("a frozen platform renders different geometry in different scans")
        if checks["range_within_limits_frac"] < 1.0:
            failures.append("rendered ranges violate the configured range limits")
        for key in ("rot_orthogonality_max_err", "rot_axis_invariance_max_err", "rot_angle_max_err"):
            if checks[key] > 1e-9:
                failures.append("%s too large" % key)
        if checks["time_field_first_point_s"] != 0.0:
            failures.append("per-point time field is not rebased on the first emitted point "
                            "(index 0 carries %.3g s)" % checks["time_field_first_point_s"])
        if not checks["time_field_monotonic"]:
            failures.append("per-point time field is not monotonically non-decreasing")
        if not (0.0 < checks["time_field_span_s"] <= 1.0 / float(traj["sensors"]["lidar_hz"]) + 1e-9):
            failures.append("per-point time span %.4f s is outside (0, scan period]"
                            % checks["time_field_span_s"])
        if checks.get("ray_pose_max_range_err_m") is None or \
                checks["ray_pose_max_range_err_m"] > 1e-4:
            failures.append("the rendered rays are not the actual motion at each ray's emission "
                            "time: max range error %.3g m over %s rays (a midpoint-twist "
                            "extrapolation would show ~mm)"
                            % (checks.get("ray_pose_max_range_err_m") or -1.0,
                               checks.get("ray_pose_rays_checked")))
        if checks.get("kinematic_violations"):
            env = checks.get("kinematic_envelope") or {}
            for name in checks["kinematic_violations"]:
                failures.append("derived kinematic stream violates its physical envelope: "
                                "%s = %.6g > %.6g (a model discontinuity - a closed-loop heading "
                                "wrap or a velocity step - would show up here)"
                                % (name, checks[name], env.get(name)))
        if failures:
            for f in failures:
                print("[generate_test_bag] SELF-CHECK FAIL: %s" % f, file=sys.stderr)
            return 1
        print("[generate_test_bag] --check OK (no bag written)")
        return 0

    print("[generate_test_bag] trajectory=%s duration=%.1fs scene=%s"
          % (traj.get("name", "?"), duration, os.path.basename(args.trajectory)))
    print("[generate_test_bag] init probe: gyro_std=%.4f rad/s (limit %.4f), "
          "acc_dev=%.3f m/s^2 (limit %.3f), |mean accel|=%.3f m/s^2 -> static verdict=%s"
          % (info["init_window_gyro_std_rps"], info["init_static_gyro_std_limit"],
             info["init_window_accel_dev_mps2"], info["init_static_acc_dev_limit"],
             info["init_window_accel_mean_norm"], info["static_verdict_expected"]))
    if info["max_wait_fallback_expected"]:
        print("[generate_test_bag] NOTE: the trot-in-place window is NOT judged static; the "
              "IMU-init max-wait fallback (imu_init_max_wait_s=%.1fs) is what must let "
              "initialization proceed." % info["init_max_wait_s"])
    write_bag(args.out, traj, duration, overwrite=args.overwrite)
    return 0


if __name__ == "__main__":
    sys.exit(main())
