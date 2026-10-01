#!/usr/bin/env python3
"""Compose the effective lio_node config for a simulation run, BEFORE the bag is generated.

Why this exists: the bag generator must know the node's lidar_min_range/lidar_max_range,
because Utils::pcl2_to_PCL adopts its per-point time origin from the first point IT retains.
If the generator emitted points the node then re-filters, the header stamp would no longer be
the first retained point's emission time and every per-point time would shift.  So the harness
composes the effective config first, then generates a bag whose ranges match it exactly.

The composition is not re-implemented here: it imports SIM_OVERRIDES from the launch file, so
the two can never drift apart.

Usage:
    python3 sim_config.py --work-dir /tmp/sim --out /tmp/sim/lio_sim.yaml
    python3 sim_config.py --work-dir /tmp/sim --extra-config /tmp/ablation.yaml --json
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(HERE)
LAUNCH_FILE = os.path.join(SIM_DIR, "launch", "sim_fastlio2.launch.py")


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("sim_fastlio2_launch", LAUNCH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def compose(base_path, extra_path):
    mod = _load_launch_module()
    base = base_path or mod._resolve_config()
    with open(base, "r") as fh:
        config = yaml.safe_load(fh) or {}
    config.update(mod.SIM_OVERRIDES)
    if extra_path:
        with open(extra_path, "r") as fh:
            config.update(yaml.safe_load(fh) or {})
    violated = [k for k, v in mod.SIM_OVERRIDES.items() if config.get(k) != v]
    if violated:
        raise SystemExit("[error] extra_config overrode simulation-critical keys %s" % violated)
    return config, base


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--work-dir", required=True)
    ap.add_argument("--out", default=None, help="default <work-dir>/lio_sim.yaml")
    ap.add_argument("--base", default=None, help="base profile (default: the fastlio2 source config)")
    ap.add_argument("--extra-config", default=None, help="ablation overlay merged last")
    ap.add_argument("--json", action="store_true", help="print a JSON summary on stdout")
    args = ap.parse_args(argv)

    out = args.out or os.path.join(args.work_dir, "lio_sim.yaml")
    config, base = compose(args.base, args.extra_config)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as fh:
        yaml.safe_dump(config, fh, sort_keys=False)
    summary = {
        "effective_config": os.path.abspath(out),
        "base_profile": os.path.abspath(base),
        "extra_config": os.path.abspath(args.extra_config) if args.extra_config else None,
        "lidar_min_range": config.get("lidar_min_range"),
        "lidar_max_range": config.get("lidar_max_range"),
        "lidar_filter_num": config.get("lidar_filter_num"),
        "lidar_topic": config.get("lidar_topic"),
        "imu_topic": config.get("imu_topic"),
        "imu_acc_scale": config.get("imu_acc_scale"),
        "pcl2_time_field": config.get("pcl2_time_field"),
        "pcl2_time_scale": config.get("pcl2_time_scale"),
        "pcl2_filter_phase": config.get("pcl2_filter_phase"),
        "ext_il": config.get("ext_il"),
        "imu_acc_normalize": config.get("imu_acc_normalize"),
        "imu_init_mode": config.get("imu_init_mode"),
        "imu_init_min_samples": config.get("imu_init_min_samples"),
        "print_time_cost": config.get("print_time_cost"),
    }
    if args.json:
        print(json.dumps(summary))
    else:
        for k, v in summary.items():
            print("  %-22s %s" % (k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
