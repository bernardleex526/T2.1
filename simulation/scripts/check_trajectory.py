#!/usr/bin/env python3
"""Trajectory accuracy DIAGNOSTIC for a FastLIO2 simulation run: ATE / RPE / yaw drift.

The estimated poses (``odom.tum`` from record_run.py) can be compared against the predefined
ground truth because the estimator and the generator share the same time base: ``lio_odom`` is
stamped with the frame end (``cloud_end_time`` = header stamp + last point curvature), and the
generator stamps the header at the first emitted point's emission time, so the frame end is the
last point's physical emission time.  Ground truth is regenerated from test_trajectory.json by
traj_common, i.e. from the same code that rendered the LiDAR and IMU, never from the estimate.

Metrics (Umeyama SE(3) alignment, scale fixed at 1 -- the estimator's world frame is
gravity-aligned but its origin/heading are arbitrary, so a rigid alignment is the standard
normalisation for this diagnostic):
  * ate_rmse_m / ate_mean_m / ate_max_m, per-axis rmse
  * rpe_trans_* over a 1 s interval (drift rate, independent of the global alignment)
  * yaw_drift_deg and final_position_error_m

THIS IS A DIAGNOSTIC, NOT AN ACCEPTANCE METRIC: the value is post-fit (SE(3)-aligned).  The
acceptance metrics are T1 (map accuracy against the scene reference) and T3 (localization
against the interpolated GT with NO alignment); the aligned number here must never be quoted
as an accuracy claim.

Exit status is always 0 unless --fail-ate-m is given, so the script is safe inside pipelines.

Usage:
    python3 check_trajectory.py --odom /tmp/fastlio2_output/odom.tum \
        --trajectory ../synthetic_data/test_trajectory.json \
        --out /tmp/fastlio2_output/trajectory_result.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "synthetic_data"))
import traj_common as tc  # noqa: E402

DEFAULT_TRAJECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  os.pardir, "synthetic_data", "test_trajectory.json")


def read_tum(path):
    rows = np.loadtxt(path, ndmin=2)
    if rows.size == 0:
        raise ValueError("empty TUM file: %s" % path)
    return rows


def align_umeyama(src, dst):
    """Rigid SE(3) transform (scale 1) mapping src -> dst, Horn/Umeyama via SVD."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(u @ vt))
    s = np.diag([1.0, 1.0, d])
    rot = u @ s @ vt
    return rot, mu_d - rot @ mu_s


def yaw_of(q):
    """Yaw [rad] from quaternion rows (x, y, z, w)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def evaluate(rows, traj, duration_s=None, rpe_interval_s=1.0):
    t = rows[:, 0]
    est_p = rows[:, 1:4]
    est_yaw = yaw_of(rows[:, 4:8])

    gt = tc.build_ground_truth(traj, duration_s=duration_s)
    gt_t = gt["t"]
    # lio_odom is stamped with absolute epochs (t0_epoch_s + scan end); the ground truth is
    # built on a relative grid starting at 0, so shift onto the same axis before matching.
    t = t - float(traj["t0_epoch_s"])
    in_range = (t >= gt_t[0]) & (t <= gt_t[-1])
    if not in_range.any():
        raise ValueError("no odom samples inside the ground-truth time span [%.3f .. %.3f]; "
                         "odom span (relative) is [%.3f .. %.3f]" % (gt_t[0], gt_t[-1], t[0], t[-1]))
    t, est_p, est_yaw = t[in_range], est_p[in_range], est_yaw[in_range]

    gt_p = np.stack([np.interp(t, gt_t, gt["pos"][:, i]) for i in range(3)], axis=1)
    gt_heading = np.interp(t, gt_t, gt["heading"])

    rot, trans = align_umeyama(est_p, gt_p)
    aligned = (rot @ est_p.T).T + trans

    err = aligned - gt_p
    dist = np.linalg.norm(err, axis=1)

    # RPE over a fixed interval: relative displacement between i and the sample ~interval later
    idx = np.searchsorted(t, t + rpe_interval_s)
    valid = idx < len(t)
    src = np.where(valid)[0]
    d_est = np.linalg.norm(aligned[idx[src]] - aligned[src], axis=1)
    d_gt = np.linalg.norm(gt_p[idx[src]] - gt_p[src], axis=1)
    rpe_err = np.abs(d_est - d_gt) if src.size else np.zeros(0)

    yaw_err = np.rad2deg(np.abs(np.arctan2(np.sin(est_yaw - gt_heading),
                                           np.cos(est_yaw - gt_heading))))

    span = float(t[-1] - t[0])
    expected_scans = int(round(float(duration_s if duration_s is not None else traj["duration_s"])
                               * float(traj["sensors"]["lidar_hz"])))
    res = {
        "status": "ok",
        "samples": int(len(t)),
        "expected_scans": expected_scans,
        "scan_delivery_ratio": float(len(t) / expected_scans) if expected_scans else None,
        "span_s": span,
        "path_length_gt_m": float(np.linalg.norm(np.diff(gt_p, axis=0), axis=1).sum()),
        "path_length_est_m": float(np.linalg.norm(np.diff(aligned, axis=0), axis=1).sum()),
        "ate_rmse_m": float(np.sqrt((dist ** 2).mean())),
        "ate_mean_m": float(dist.mean()),
        "ate_max_m": float(dist.max()),
        "ate_rmse_axis_m": [float(np.sqrt((err[:, i] ** 2).mean())) for i in range(3)],
        "rpe_trans_mean_m": float(rpe_err.mean()) if rpe_err.size else None,
        "rpe_trans_max_m": float(rpe_err.max()) if rpe_err.size else None,
        "rpe_interval_s": rpe_interval_s,
        "yaw_err_rmse_deg": float(np.sqrt((yaw_err ** 2).mean())),
        "yaw_err_final_deg": float(yaw_err[-1]),
        "final_position_error_m": float(dist[-1]),
        "alignment_rot": [[float(x) for x in r] for r in rot],
        "alignment_trans": [float(x) for x in trans],
    }
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--odom", default="/tmp/fastlio2_output/odom.tum")
    ap.add_argument("--trajectory", default=DEFAULT_TRAJECTORY)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--rpe-interval", type=float, default=1.0)
    ap.add_argument("--clean-start", action="store_true",
                    help="score against the clean-start ablation variant (must match the bag)")
    ap.add_argument("--seed", type=int, default=None, help="seed override (must match the bag)")
    ap.add_argument("--out", default="/tmp/fastlio2_output/trajectory_result.json")
    ap.add_argument("--fail-ate-m", type=float, default=None,
                    help="exit non-zero if ate_rmse_m exceeds this (for CI use)")
    args = ap.parse_args(argv)

    traj = tc.apply_variant(tc.load_trajectory(args.trajectory),
                            clean_start=args.clean_start, seed=args.seed)
    rows = read_tum(args.odom)
    res = evaluate(rows, traj, args.duration, args.rpe_interval)
    res["odom_tum"] = os.path.abspath(args.odom)
    res["trajectory"] = os.path.abspath(args.trajectory)
    res["trajectory_variant"] = traj.get("variant", "baseline")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(res, fh, indent=2)

    print("[check_trajectory] samples=%d span=%.1fs" % (res["samples"], res["span_s"]))
    if res["scan_delivery_ratio"] is not None and res["scan_delivery_ratio"] < 0.9:
        print("[check_trajectory] WARNING: only %d/%d scans produced odometry (ratio %.2f); the "
              "node is dropping scans -- re-run with rate:=0.5 for a fully deterministic sensor "
              "stream" % (res["samples"], res["expected_scans"], res["scan_delivery_ratio"]))
    if not np.isfinite(res["ate_rmse_m"]):
        print("[check_trajectory] FAIL: non-finite ATE (the estimate blew up)")
        return 1
    if res["ate_rmse_m"] > 10.0 * max(1.0, res["path_length_gt_m"]):
        print("[check_trajectory] NOTE: ATE %.1f m exceeds the entire ground-truth path length "
              "(%.1f m) -- the estimator diverged on this run rather than merely drifting."
              % (res["ate_rmse_m"], res["path_length_gt_m"]))
    print("[check_trajectory] ATE rmse=%.3f m  mean=%.3f  max=%.3f  (axis rmse %s)"
          % (res["ate_rmse_m"], res["ate_mean_m"], res["ate_max_m"],
             " ".join("%.3f" % v for v in res["ate_rmse_axis_m"])))
    if res["rpe_trans_mean_m"] is not None:
        print("[check_trajectory] RPE(%gs) mean=%.3f m max=%.3f m"
              % (res["rpe_interval_s"], res["rpe_trans_mean_m"], res["rpe_trans_max_m"]))
    print("[check_trajectory] yaw err rmse=%.2f deg final=%.2f deg | final position err=%.3f m"
          % (res["yaw_err_rmse_deg"], res["yaw_err_final_deg"], res["final_position_error_m"]))
    print("[check_trajectory] wrote %s" % args.out)
    if args.fail_ate_m is not None and res["ate_rmse_m"] > args.fail_ate_m:
        print("[check_trajectory] FAIL: ate_rmse_m %.3f > %.3f" % (res["ate_rmse_m"], args.fail_ate_m))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
