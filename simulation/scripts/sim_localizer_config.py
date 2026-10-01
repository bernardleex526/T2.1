#!/usr/bin/env python3
"""Write the SIMULATION-SIDE effective localizer config for T3.

The package config (src/localization/localizer/config/localizer.yaml) is never edited: this
script copies it verbatim and applies only the simulation wiring, so the ICP/gate keys always
stay identical to the deployed profile:

  cloud_topic  /fastlio2/body_cloud   (lio_node's body-frame scan)
  odom_topic   /fastlio2/lio_odom     (lio_node's odometry)
  map_frame    map                    (the frozen prior map's frame == the scene frame)
  local_frame  odom                   (overwritten at runtime by the inbound odom frame_id)
  update_hz    10.0                   (one TF per scan, so the pose stream is dense enough)
  pcd_path     <frozen prior map>

Usage:
    python3 sim_localizer_config.py --out /tmp/sim/loc_sim.yaml --pcd /tmp/sim/frozen_map.pcd
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
PACKAGE_CONFIG = os.path.join(REPO_ROOT, "src", "localization", "localizer", "config",
                              "localizer.yaml")

# Only these keys are simulation wiring; everything else is copied verbatim from the package
# config (which stays the single source of truth for the ICP and gate tuning).
SIM_OVERRIDES = {
    "cloud_topic": "/fastlio2/body_cloud",
    "odom_topic": "/fastlio2/lio_odom",
    "map_frame": "map",
    "local_frame": "odom",
    "update_hz": 10.0,
}

REQUIRED_KEYS = ("cloud_topic", "odom_topic", "map_frame", "local_frame", "update_hz",
                 "rough_scan_resolution", "rough_map_resolution", "rough_max_iteration",
                 "rough_score_thresh", "refine_scan_resolution", "refine_map_resolution",
                 "refine_max_iteration", "refine_score_thresh", "pcd_path")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--pcd", required=True, help="frozen prior PCD map the node must load")
    ap.add_argument("--package-config", default=PACKAGE_CONFIG)
    ap.add_argument("--update-hz", type=float, default=None)
    args = ap.parse_args(argv)

    with open(args.package_config, "r") as fh:
        config = yaml.safe_load(fh) or {}
    config.update(SIM_OVERRIDES)
    if args.update_hz is not None:
        config["update_hz"] = float(args.update_hz)
    config["pcd_path"] = os.path.abspath(args.pcd)
    missing = [k for k in REQUIRED_KEYS if k not in config]
    if missing:
        raise SystemExit("[error] the package localizer config is missing %s (the node raises a "
                         "YAML conversion exception without them)" % missing)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        yaml.safe_dump(config, fh, sort_keys=False)
    print("[sim_localizer_config] %s (pcd=%s, update_hz=%s)"
          % (args.out, config["pcd_path"], config["update_hz"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
