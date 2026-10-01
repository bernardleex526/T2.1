#!/usr/bin/env python3
"""Generate the synthetic scene's OWN reference geometry, support ROI and ground truth.

Everything this script writes lives in the synthetic scene's own frame:

  reference_map.pcd     dense analytic sampling of the scene's static surfaces (room box
                        faces + pillar lateral surfaces).  This is the harness's reference
                        geometry -- NOT the TUHH/MCD/HILTI survey map, and no transform,
                        ROI or point of any external dataset is used anywhere in it.
  roi.json              the frozen static support region (roi_schema.json v1), derived
                        from the scene definition alone.
  gt_mapping.tum        TRUE body trajectory of the mapping route, scene frame, TUM.
  gt_localization.tum   TRUE body trajectory of the separate localization route, scene frame.
  scene_reference.json  manifest: hashes, counts, exact command, scene/route statistics.

Usage:
    python3 scene_reference.py --out-dir /tmp/sim_ref
    python3 scene_reference.py --out-dir /tmp/sim_ref --spacing 0.05 --roi-margin 0.5
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402
import traj_common as tc  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAPPING = os.path.join(HERE, "test_trajectory.json")
DEFAULT_LOCALIZATION = os.path.join(HERE, "test_trajectory_localization.json")


def build(out_dir, mapping_path, localization_path, spacing, roi_margin, duration,
          quiet=False, sample_hz=None, eval_start_s=0.0):
    traj = tc.load_trajectory(mapping_path)
    duration = float(traj["duration_s"] if duration is None else duration)
    # The reference must be at least as dense as the map it is compared with: the grazing
    # surfaces (floor/ceiling) are sampled radially by the vertical beam grid, so a low
    # ray-cast rate leaves metre-wide gaps that would look like unmapped regions.  Default to
    # the sensor's own scan rate.
    sample_hz = float(sample_hz) if sample_hz else float(traj["sensors"]["lidar_hz"])
    os.makedirs(out_dir, exist_ok=True)

    # REFERENCE GEOMETRY = the scene surfaces the sensor can actually observe from the TRUE
    # trajectory (ray-cast, no noise/dropout, scene frame).  The full analytic room shell is
    # also written, but as a DIAGNOSTIC only: it contains regions no LiDAR can observe, which
    # would be misread as "the estimate abandoned them".
    ref_pts, ref_info = tc.observed_surface_points(traj, duration_s=duration,
                                                   sample_hz=sample_hz, voxel=spacing,
                                                   t_start=eval_start_s)
    ref_path = os.path.join(out_dir, "reference_map.pcd")
    sc.write_pcd(ref_path, ref_pts)
    ref_sha = sc.sha256_file(ref_path)
    analytic = tc.scene_surface_points(traj["scene"], spacing=spacing)
    analytic_path = os.path.join(out_dir, "reference_map_full_analytic.pcd")
    sc.write_pcd(analytic_path, analytic)

    roi = tc.scene_roi(traj, margin_m=roi_margin, reference_map_sha256=ref_sha)
    roi_path = os.path.join(out_dir, "roi.json")
    sc.write_json(roi_path, roi)

    gt_map_path = os.path.join(out_dir, "gt_mapping.tum")
    gt_map = tc.write_gt_tum(traj, duration, gt_map_path)
    routes = {"mapping": tc.route_stats(traj, duration_s=duration)}

    gt_loc_path = None
    gt_loc = None
    loc_traj = None
    if localization_path and os.path.isfile(localization_path):
        loc_traj = tc.load_trajectory(localization_path)
        loc_scene = {k: loc_traj["scene"][k] for k in ("room_box_min", "room_box_max", "pillars")}
        map_scene = {k: traj["scene"][k] for k in ("room_box_min", "room_box_max", "pillars")}
        if loc_scene != map_scene:
            raise SystemExit("[error] the localization trajectory uses a different scene than the "
                             "mapping trajectory: the frozen prior map would not apply")
        gt_loc_path = os.path.join(out_dir, "gt_localization.tum")
        gt_loc = tc.write_gt_tum(loc_traj, loc_traj["duration_s"], gt_loc_path)
        routes["localization"] = tc.route_stats(loc_traj, duration_s=loc_traj["duration_s"])

    manifest = {
        "generated_utc": sc.utcnow(),
        "command": " ".join([sys.executable] + sys.argv),
        "frame": "scene",
        "reference_map": {
            "path": ref_path, "sha256": ref_sha,
            "points": int(len(ref_pts)), "voxel_m": float(spacing),
            "source": ("ray-cast of the scene definition from the TRUE trajectory (configured "
                       "beam grid, FOV and range limits, no noise/dropout): the OBSERVABLE "
                       "surface geometry in the scene frame"),
            "raycast": ref_info,
            "external_dataset_used": False,
        },
        "reference_map_full_analytic": {
            "path": analytic_path, "sha256": sc.sha256_file(analytic_path),
            "points": int(len(analytic)),
            "role": "DIAGNOSTIC ONLY: the full room shell, including surface regions no LiDAR "
                    "can observe; never used as the map-accuracy reference",
        },
        "roi": {"path": roi_path, "sha256": sc.sha256_file(roi_path), "roi_id": roi["roi_id"],
                "bounds": roi["bounds"], "type": roi["type"], "frame": roi["frame"]},
        "gt": {"mapping": dict(gt_map, sha256=sc.sha256_file(gt_map_path))},
        "routes": routes,
        "trajectories": {"mapping": os.path.abspath(mapping_path),
                         "mapping_sha256": sc.sha256_file(mapping_path),
                         "localization": os.path.abspath(localization_path) if loc_traj else None,
                         "localization_sha256": sc.sha256_file(localization_path) if loc_traj else None},
        "scene": traj["scene"],
        "notes": ["No TUHH/MCD/HILTI reference map, ROI or transform is applied to any synthetic "
                  "output; the reference geometry is the scene definition itself."],
    }
    if gt_loc:
        manifest["gt"]["localization"] = dict(gt_loc, sha256=sc.sha256_file(gt_loc_path))
    sc.write_json(os.path.join(out_dir, "scene_reference.json"), manifest)
    if not quiet:
        print("[scene_reference] %s: reference %d pts (%.3f m spacing) -> %s"
              % (out_dir, len(ref_pts), spacing, ref_path))
        print("[scene_reference] roi=%s bounds=%s" % (roi["roi_id"], roi["bounds"]))
        print("[scene_reference] gt mapping %s; localization %s"
              % (gt_map_path, gt_loc_path or "(none)"))
    return manifest


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", required=True, help="directory for the reference artifacts")
    ap.add_argument("--trajectory", default=DEFAULT_MAPPING)
    ap.add_argument("--localization-trajectory", default=DEFAULT_LOCALIZATION)
    ap.add_argument("--spacing", type=float, default=0.05,
                    help="reference surface sampling voxel in m")
    ap.add_argument("--sample-hz", type=float, default=None,
                    help="rate of ground-truth sensor poses ray-cast into the reference map "
                         "(default: the trajectory's lidar_hz, so the reference is as dense as "
                         "the map it is compared against)")
    ap.add_argument("--eval-start-s", type=float, default=0.0,
                    help="ray-cast the reference only from ground-truth poses at/after this sim "
                         "time, i.e. over the same window the evaluated map comes from")
    ap.add_argument("--roi-margin", type=float, default=0.5,
                    help="ROI expansion beyond the room box on every axis, in m")
    ap.add_argument("--duration", type=float, default=None,
                    help="override the mapping trajectory duration for the GT window")
    args = ap.parse_args(argv)
    if args.spacing <= 0.0:
        raise SystemExit("[error] --spacing must be positive")
    build(args.out_dir, args.trajectory, args.localization_trajectory, args.spacing,
          args.roi_margin, args.duration, sample_hz=args.sample_hz,
          eval_start_s=args.eval_start_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
