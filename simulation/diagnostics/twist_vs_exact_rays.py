#!/usr/bin/env python3
"""How far the PRE-FIX twist renderer was from the actual per-ray motion.

The old `render_scan` extrapolated ONE scan-midpoint pose with a linear translation and a constant
body rate (`exp(w dt) R`, the world-frame convention) for the whole 100 ms sweep.  This script
recomputes both renderings for the same rays - the twist and the exact per-ray GT pose - and
reports their point deviation, which is what the new `--check` ray assertion discriminates
(measured max 2.2e-06 m on the fixed renderer against a 1e-4 m tolerance).

Run: python3 simulation/diagnostics/twist_vs_exact_rays.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir, "synthetic_data"))
import traj_common as tc  # noqa: E402


def main():
    traj = tc.load_trajectory(os.path.join(HERE, os.pardir, "synthetic_data",
                                           "test_trajectory.json"))
    gt = tc.build_ground_truth(traj)
    sensors = dict(traj["sensors"])
    sensors.update(lidar_range_noise_std_m=0.0, lidar_point_noise_std_m=0.0, lidar_dropout_frac=0.0)
    dirs, az = tc.lidar_directions(sensors)
    period = 1.0 / float(sensors["lidar_hz"])
    print("%8s %14s %14s %14s" % ("scan t", "max dev", "p95 dev", "mean dev"))
    for t_scan in (6.0, 20.0, 50.0):
        st = tc.interp_state(gt, np.array([t_scan + 0.5 * period]))
        pos_mid, rot_mid = st["pos"][0], st["rot"][0]
        vel_mid, gyro_mid = st["vel"][0], st["gyro"][0]
        dt = (az - 0.5) * period
        o_twist = pos_mid + vel_mid * dt[:, None]
        d_twist = np.einsum("nij,nj->ni", tc._exp_so3(gyro_mid * dt[:, None]) @ rot_mid[None], dirs)
        r_twist = tc.raycast_scene(o_twist, d_twist, traj["scene"], np.random.default_rng(3), sensors)
        st_ex = tc.interp_state(gt, t_scan + az * period)
        d_ex = np.einsum("nij,nj->ni", st_ex["rot"], dirs)
        r_ex = tc.raycast_scene(st_ex["pos"], d_ex, traj["scene"], np.random.default_rng(3), sensors)
        p_tw = o_twist + r_twist[:, None] * d_twist
        p_ex = st_ex["pos"] + r_ex[:, None] * d_ex
        ok = np.isfinite(p_tw).all(1) & np.isfinite(p_ex).all(1)
        dev = np.linalg.norm(p_tw[ok] - p_ex[ok], axis=1)
        print("%8.0f %11.1f mm %11.1f mm %11.1f mm"
              % (t_scan, dev.max() * 1e3, np.percentile(dev, 95) * 1e3, dev.mean() * 1e3))
    print("\nNote: the maxima are silhouette rays (a ray tangent to a pillar flips between the "
          "pillar and the far wall), so the p95/mean are the meaningful pose-error statistics.")


if __name__ == "__main__":
    main()
