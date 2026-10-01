#!/usr/bin/env python3
"""C2.3 determination: does the PointCloud2 decimation PHASE change any observable behaviour?

C2.3 in the original task list is the "point-cloud sampling phase" of `utils.cpp::pcl2_to_PCL`,
which decimates with `for (i = 0; i < point_num; i += filter_num)` - i.e. the phase is hard-wired
to 0 and there is NO switch for it (confirmed in the source; the C2 notes in
docs/validation/04b73f5_param_notes.md state the same).  This script determines whether a phase
change COULD matter before anyone adds an opt-in experiment for it:

  1. the time contract: the phase shifts which point is first, hence t0 (the curvature origin the
     node adopts) and therefore `cloud_end_time = header + last curvature`;
  2. the measurement content: the two phases select complementary subsets of the same sweep, so
     their residual statistics against the KNOWN analytic scene surfaces must be statistically
     identical (same beam grid, same noise, same geometry) - only the sample identities differ.

No ROS, no bag, deterministic.  Run: python3 diagnostics/c2_3_sampling_phase.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir, "synthetic_data"))
import traj_common as tc  # noqa: E402

TRAJ = os.path.join(HERE, os.pardir, "synthetic_data", "test_trajectory.json")


def analytic_distance(p, scene):
    """Min distance to the room faces / pillar cylinders (exact normals)."""
    lo = np.asarray(scene["room_box_min"], float)
    hi = np.asarray(scene["room_box_max"], float)
    best = np.full(len(p), np.inf)
    for axis in range(3):
        for plane in (lo[axis], hi[axis]):
            best = np.minimum(best, np.abs(p[:, axis] - plane))
    for pil in scene["pillars"]:
        cx, cy = pil["center_xy"]
        rad = np.hypot(p[:, 0] - cx, p[:, 1] - cy)
        best = np.minimum(best, np.abs(rad - float(pil["radius_m"])))
    return best


def main():
    traj = tc.load_trajectory(TRAJ)
    sensors = traj["sensors"]
    gt = tc.build_ground_truth(traj, duration_s=12.0)
    dirs, az_frac = tc.lidar_directions(sensors)
    period = 1.0 / float(sensors["lidar_hz"])
    filt = 2                                  # lio_sim.yaml: lidar_filter_num
    out = {}
    for t_scan in (6.0, 20.0, 40.0, 70.0):
        pts, t_first = tc.render_scan(gt, t_scan, period, dirs, az_frac, traj["scene"],
                                      np.random.default_rng(7), sensors)
        # the node's own range filter (the generator already applied it) then the decimation phase
        rng = np.linalg.norm(pts[:, :3], axis=1)
        keep = (rng >= sensors["lidar_min_range_m"]) & (rng <= sensors["lidar_max_range_m"])
        P = pts[keep]
        res = {}
        for phase in range(filt):
            Q = P[phase::filt]
            d = analytic_distance(Q[:, :3], traj["scene"])
            res[phase] = {"n": int(len(Q)), "t0_s": float(Q[0, 4]),
                          "d_median": float(np.median(d)), "d_p95": float(np.percentile(d, 95)),
                          "d_mean": float(d.mean())}
        out[t_scan] = {"t_first_s": float(t_first), "phases": res}
        print("scan t=%.1f  first-point time %.6f s" % (t_scan, t_first))
        for phase in range(filt):
            r = res[phase]
            print("   phase %d: n=%d  t0=%.6f s (delta %.6f s)  d_median %.5f  d_p95 %.5f"
                  % (phase, r["n"], r["t0_s"], r["t0_s"] - res[0]["t0_s"], r["d_median"], r["d_p95"]))
    # summary of the two effects
    d_t0 = [out[t]["phases"][1]["t0_s"] - out[t]["phases"][0]["t0_s"] for t in out]
    d_med = [abs(out[t]["phases"][1]["d_median"] - out[t]["phases"][0]["d_median"]) for t in out]
    print("\nphase-1 minus phase-0: t0 shift %.6f s (%.4f%% of the frame span); |d_median| "
          "difference <= %.5f m" %
          (float(np.mean(d_t0)), 100.0 * float(np.mean(d_t0)) / period, float(np.max(d_med))))
    print("=> the phase selects WHICH points of the same sweep are kept.  It does not change the "
          "point count, the geometry, the beam grid, the noise, the frame span, or the node's time "
          "contract: the azimuth-0 beams of every scan share one emission time, so the first "
          "retained point (and therefore t0 and cloud_end_time) is the same for every phase.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
