#!/usr/bin/env python3
"""Step-5 stability / C2 repeat evidence runner (ONE command runs all 80 mapping runs).

This runner ADDS an aggregation layer on top of the existing simulation harness.  It does not
re-implement any algorithm or metric; per run it calls exactly the frozen scripts:

  scene_reference.py    -- ONCE per route: reference geometry, ROI and the scene-frame ground
                           truth consumed by T1 (mapping_path = that route's trajectory)
  generate_test_bag.py  -- ONCE per (route, seed): the seed's input bag, generated with --seed
                           and with the range limits taken from the effective lio config
  run_mapping_sim.sh    -- the mapping run itself (--bag-dir ... --skip-bag, per-arm overlay)
  make_sim_transform.py -- the frozen frame chain (fit window disjoint from evaluated scans)
  test_t1_accuracy.py   -- map accuracy verdict (evaluator admissibility gate included)
  test_t2_speed.py      -- host-only per-frame latency verdict

Frozen design (declared here, not inferred at aggregation time):

  routes  main    : synthetic_data/test_trajectory.json                75 runs (5 arms x 5 seeds x 3)
          holdout : synthetic_data/test_trajectory_localization.json    5 runs (baseline x 5 seeds)
                    a SECOND MAPPING PATH in the same frozen scene: a cross-path holdout, never
                    a new scenario, and never evidence of cross-scenario generalization.
  arms    baseline, c2_1, c2_2, c2_3, c2_combined -- the existing simulation/ablations/*.yaml
          overlays, i.e. the same files run_eval_pipeline.sh uses, so the arms cannot drift.
  seeds   41..45.  A seed changes ONLY traj_common.apply_variant(..., seed=N) (imu_noise.seed),
          which drives the bag generator's IMU/map noise RNG.  Every seed trajectory's geometry
          fingerprint is asserted equal to the reference trajectory's (audited, not assumed), so
          the reference/ROI/ground truth are seed-independent and are generated once per route.
  inputs  every arm of a seed consumes the SAME bag directory; each run records the bag's file
          hashes AND a decoded-message content hash (CDR padding excluded), so "same input" is
          checked on decoded content, not on bag file bytes.
  stats   bootstrap RNG 20260930, 10000 draws; the sampling unit is the SEED index (with
          replacement) and ONE index draw is shared by every arm in an iteration, so paired arm
          differences are truly paired.  Per-seed value = arithmetic mean of that seed's 3
          independent node runs; overall value = mean of the 5 seed means; CI = 2.5/97.5
          percentile of the bootstrap means.

Claim discipline: a route is called stable-meets-target ONLY when every one of its runs has its
own T1 and T2 PASS with all gates green AND the overall T1 mean's 95% CI upper bound <= 0.05 m.
A paired-difference CI that crosses 0 is reported as "improvement not demonstrated"; a CI entirely
below 0 is NOT an equivalence claim and no CI difference is attributed to the algorithm.  Nothing
is retried, re-parameterised or dropped to turn a FAIL into a PASS: recorded FAIL/BLOCKED runs
stay in the aggregate and in the manifest forever.

Resume contract: the manifest and the aggregate are written after EVERY run (atomic).  `--resume`
re-opens an existing out-root and skips a planned run ONLY when its latest attempt has complete
evidence (T1+T2 verdicts present with recorded hashes that still match on disk) and its inputs
still match (bag file hashes, seed trajectory).  Every such skip is recorded explicitly together
with the verdict it froze -- including FAILs, which are NOT re-run.  A run whose latest attempt is
incomplete is executed again in a NEW `attempt<N>` subdirectory of the same base directory: an
existing attempt directory is never renamed, reused or written into, so interrupted and failed
attempts keep their evidence and every attempt stays counted (`manifest.attempts`, plus
`attempts_total` / `runs_attempted_more_than_once` / `previous_attempts`).  Old bags are never
overwritten either.

Exit codes: 0 = every planned run produced complete, gate-clean T1/T2 evidence (the verdicts
themselves may be PASS or FAIL); 1 = at least one run is incomplete, gate-failed or BLOCKED, i.e.
the frozen protocol could not be certified for it; 2 = precondition failure (environment,
out-root, non-empty ROS domain).

`--skip-holdout` exists for debugging only: the frozen step-5 delivery is the 80-run design, and a
plan built with that switch is recorded as `debug_only` in both the manifest and the aggregate.

Usage:
    python3 simulation/scripts/run_metric_repeats.py --out-root /tmp/wp2-recovery-repeats --domain-id 138
    python3 simulation/scripts/run_metric_repeats.py --out-root ... --domain-id 138 --resume
    python3 simulation/scripts/run_metric_repeats.py --plan --out-root ... --domain-id 138
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import signal
import struct
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)
sys.path.insert(0, os.path.join(SIM_DIR, "synthetic_data"))
sys.path.insert(0, HERE)
import sim_common as sc  # noqa: E402
import traj_common as tc  # noqa: E402

DEFAULT_SEEDS = (41, 42, 43, 44, 45)
REPEATS_PER_SEED = 3
BOOTSTRAP_RNG = 20260930
BOOTSTRAP_N = 10000
THRESHOLD_M = 0.05
TARGET_MPS = 0.5
MAX_MPS = 1.0
FIT_UNTIL_S = 30.0
RATE = 1.0

ARMS = (
    ("baseline", None),
    ("c2_1", "ablations/c2_1_acc_normalize.yaml"),
    ("c2_2", "ablations/c2_2_first_batch.yaml"),
    ("c2_3", "ablations/c2_3_sampling_phase.yaml"),
    ("c2_combined", "ablations/c2_combined.yaml"),
)

MAIN_ROUTE = "main"
HOLDOUT_ROUTE = "holdout"

# Trajectory keys that define the scene, the route and the sensor stream.  A seed may move
# nothing but the noise seed, so this subset must be bit-identical across a route's seeds.
GEOMETRY_KEYS = ("t0_epoch_s", "duration_s", "sensors", "body", "gait", "scene")

# The bag<->node interface: keys the launch file pins (SIM_OVERRIDES) plus the range limits the
# bag is generated with.  An ablation overlay must not move any of them; the runner re-verifies
# it on the yaml the node actually loaded.
INTERFACE_KEYS = ("lidar_type", "lidar_topic", "imu_topic", "pcl2_time_field", "pcl2_time_scale",
                  "imu_acc_scale", "body_frame", "world_frame", "ext_il",
                  "lidar_min_range", "lidar_max_range")

POINT_TOPIC = "/rslidar_points"
IMU_TOPIC = "/imu/data"

# The frozen scripts this runner drives.  Their hashes are recorded so the evidence can be bound
# to the exact bytes that produced it (they are never written to by this runner).
TOOL_FILES = {
    "run_mapping_sim.sh": os.path.join(HERE, "run_mapping_sim.sh"),
    "generate_test_bag.py": os.path.join(SIM_DIR, "synthetic_data", "generate_test_bag.py"),
    "scene_reference.py": os.path.join(SIM_DIR, "synthetic_data", "scene_reference.py"),
    "traj_common.py": os.path.join(SIM_DIR, "synthetic_data", "traj_common.py"),
    "make_sim_transform.py": os.path.join(HERE, "make_sim_transform.py"),
    "test_t1_accuracy.py": os.path.join(HERE, "test_t1_accuracy.py"),
    "test_t2_speed.py": os.path.join(HERE, "test_t2_speed.py"),
    "sim_config.py": os.path.join(HERE, "sim_config.py"),
    "sim_common.py": os.path.join(SIM_DIR, "sim_common.py"),
    "sim_fastlio2.launch.py": os.path.join(SIM_DIR, "launch", "sim_fastlio2.launch.py"),
}


# ------------------------------------------------------------------ small helpers ----
def _sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _json_sha256(obj):
    return _sha256_text(json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str))


def geometry_fingerprint(traj):
    """sha256 of the geometry/route/sensor definition only (seed-independent)."""
    sub = {k: traj[k] for k in GEOMETRY_KEYS if k in traj}
    return _json_sha256(sub)


def noise_fingerprint(traj):
    return _json_sha256(traj.get("imu_noise", {}))


def _dir_files_sha256(path):
    """sha256 over (name, size, sha256) of every file in a directory, name-sorted."""
    h = hashlib.sha256()
    for name in sorted(os.listdir(path)):
        full = os.path.join(path, name)
        if not os.path.isfile(full):
            continue
        h.update(("%s\t%d\t%s\n" % (name, os.path.getsize(full), sc.sha256_file(full))).encode())
    return h.hexdigest()


def _ros_python_paths():
    """Make the ROS 2 python packages importable when the caller did not source the setup."""
    for extra in ("/opt/ros/humble/lib/python3.10/site-packages",
                  "/opt/ros/humble/local/lib/python3.10/dist-packages"):
        if os.path.isdir(extra) and extra not in sys.path:
            sys.path.append(extra)


def _read_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def _expect(msg):
    """Precondition failure: the documented exit code 2 (never confused with run failures)."""
    sys.stderr.write("[run_metric_repeats] %s\n" % msg)
    raise SystemExit(2)


def _py(*args):
    return [sys.executable] + [str(a) for a in args]


# ----------------------------------------------------- decoded bag content hashing ----
def _pc2_canonical(msg):
    """Canonical bytes of a decoded PointCloud2: header + layout + per-point values.

    The point values are re-assembled contiguously from the declared field offsets, so CDR
    alignment padding in the serialized buffer (which a deserializer skips) cannot change the
    hash.  Returns (digest, padding_free).  If the message does not use the harness's float32
    layout the raw payload bytes are hashed instead and padding_free=False is reported.
    """
    h = hashlib.sha256()
    h.update(b"pc2\x00")
    h.update(struct.pack("<ii", int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)))
    h.update(msg.header.frame_id.encode() + b"\x00")
    h.update(struct.pack("<II", int(msg.height), int(msg.width)))
    h.update(struct.pack("<II", int(msg.point_step), int(msg.row_step)))
    h.update(bytes([int(bool(msg.is_bigendian)), int(bool(msg.is_dense))]))
    fields = [(f.name, int(f.offset), int(f.datatype), int(f.count)) for f in msg.fields]
    for name, off, dt, cnt in fields:
        h.update(struct.pack("<III", off, dt, cnt) + name.encode() + b"\x00")

    wanted = ("x", "y", "z", "intensity", "time")
    by_name = {n: (o, dt, c) for n, o, dt, c in fields}
    layout_ok = (all(n in by_name and by_name[n][1] == 7 and by_name[n][2] == 1 for n in wanted)
                 and all(by_name[n][0] == 4 * i for i, n in enumerate(wanted)))
    if not layout_ok:
        h.update(b"raw\x00")
        h.update(bytes(msg.data))
        return h.hexdigest(), False

    raw = np.frombuffer(bytes(msg.data), dtype="<f4")
    step = max(int(msg.point_step) // 4, 1)
    h.update(b"decoded\x00")
    for name in wanted:
        off = by_name[name][0] // 4
        vals = np.ascontiguousarray(raw[off::step])
        h.update(struct.pack("<I", int(vals.size)))
        h.update(vals.astype("<f4").tobytes())
    return h.hexdigest(), True


def _imu_canonical(msg):
    h = hashlib.sha256()
    h.update(b"imu\x00")
    h.update(struct.pack("<ii", int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)))
    h.update(msg.header.frame_id.encode() + b"\x00")
    vals = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    h.update(np.asarray(vals, dtype="<f8").tobytes())
    for cov in (msg.orientation_covariance, msg.angular_velocity_covariance,
                msg.linear_acceleration_covariance):
        h.update(np.asarray(list(cov), dtype="<f8").tobytes())
    return h.hexdigest()


def bag_content_hash(bag_dir):
    """Decoded-content hash of a bag: per-topic message digests, storage-order independent."""
    _ros_python_paths()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    per_topic = {POINT_TOPIC: [], IMU_TOPIC: []}
    padding_free = True
    counts = {POINT_TOPIC: 0, IMU_TOPIC: 0}
    while reader.has_next():
        topic, data, _ts = reader.read_next()
        if topic == POINT_TOPIC:
            digest, pf = _pc2_canonical(deserialize_message(data, PointCloud2))
            padding_free = padding_free and pf
        elif topic == IMU_TOPIC:
            digest = _imu_canonical(deserialize_message(data, Imu))
        else:
            continue
        per_topic[topic].append(digest)
        counts[topic] += 1
    h = hashlib.sha256()
    for topic in (POINT_TOPIC, IMU_TOPIC):
        h.update(("%s\t%d\n" % (topic, counts[topic])).encode())
        for digest in sorted(per_topic[topic]):
            h.update((digest + "\n").encode())
    return {"sha256": h.hexdigest(), "messages": counts, "padding_free": bool(padding_free),
            "scope": "decoded message content (header, layout and point/IMU values), CDR "
                     "alignment padding excluded; per-topic digests sorted, so storage order "
                     "does not matter.  A bag FILE hash is NOT a reproducibility check."}


# ------------------------------------------------------------------- subprocesses ----
def _kill_group(proc, sig, grace_s):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.send_signal(sig)
        except OSError:
            return
    try:
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        pass


def _run(cmd, log_path, timeout_s, env=None, cwd=None):
    """Run a command in its own process group, appending stdout/stderr to log_path."""
    os.makedirs(os.path.dirname(os.path.abspath(log_path)) or ".", exist_ok=True)
    started = time.time()
    timed_out = False
    with open(log_path, "a") as fh:
        fh.write("\n$ %s\n" % shlex.join(cmd))
        fh.flush()
        try:
            proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=env, cwd=cwd,
                                    text=True, start_new_session=True)
        except OSError as exc:
            fh.write("\n[runner] spawn failed: %s\n" % exc)
            return {"cmd": shlex.join(cmd), "rc": -2, "timed_out": False, "wall_s": 0.0,
                    "log": os.path.abspath(log_path)}
        try:
            rc = int(proc.wait(timeout=timeout_s))
        except subprocess.TimeoutExpired:
            timed_out = True
            fh.write("\n[runner] TIMEOUT after %.0f s: interrupting the process group\n"
                     % timeout_s)
            fh.flush()
            _kill_group(proc, signal.SIGINT, 20.0)
            _kill_group(proc, signal.SIGKILL, 10.0)
            rc = int(proc.returncode) if proc.returncode is not None else -1
    return {"cmd": shlex.join(cmd), "rc": rc, "timed_out": timed_out,
            "wall_s": round(time.time() - started, 2), "log": os.path.abspath(log_path)}


# ------------------------------------------------------------------------- spec ----
def build_spec(out_root, domain_id, seeds, args, include_holdout=True):
    main_traj = os.path.abspath(args.main_trajectory)
    holdout_traj = os.path.abspath(args.holdout_trajectory)
    checked = [main_traj] + ([holdout_traj] if include_holdout else [])
    for path in checked:
        if not os.path.isfile(path):
            _expect("trajectory not found: %s" % path)

    routes = {}
    wanted = [(MAIN_ROUTE, main_traj, [a for a, _ in ARMS], REPEATS_PER_SEED)]
    if include_holdout:
        wanted.append((HOLDOUT_ROUTE, holdout_traj, ["baseline"], 1))
    for route, traj_path, arms, repeats in wanted:
        traj = tc.load_trajectory(traj_path)
        speed = tc.route_stats(traj, duration_s=float(traj["duration_s"]),
                               target_mps=TARGET_MPS, max_mps=MAX_MPS)
        routes[route] = {
            "trajectory": os.path.abspath(traj_path),
            "trajectory_sha256": sc.sha256_file(traj_path),
            "duration_s": float(traj["duration_s"]),
            "t0_epoch_s": float(traj["t0_epoch_s"]),
            "geometry_fingerprint": geometry_fingerprint(traj),
            "seeds": list(seeds),
            "arms": arms,
            "repeats_per_seed": repeats,
            "speed": {
                "core_speed_min_mps": speed.get("core_speed_min_mps"),
                "core_speed_mean_mps": speed.get("core_speed_mean_mps"),
                "core_speed_max_mps": speed.get("core_speed_max_mps"),
                "moving_speed_median_mps": speed.get("moving_speed_median_mps"),
                "moving_speed_p05_mps": speed.get("moving_speed_p05_mps"),
                "envelope_mps": [TARGET_MPS, MAX_MPS],
                "moving_ge_target_and_le_max": bool(speed["moving_ge_target_and_le_max"]),
                "completed_full_lap": bool(speed["completed_full_lap"]),
                "basis": "tc.route_stats on the ground-truth trajectory, computed BEFORE any run",
            },
            "role": ("primary frozen synthetic mapping path" if route == MAIN_ROUTE else
                     "cross-path holdout: a second mapping route in the SAME frozen scene; not a "
                     "new scenario, and no cross-scenario generalization is claimed"),
        }

    for route, rspec in routes.items():
        # the per-route frozen definition: a run is only reusable when THIS matches, so adding or
        # removing an unrelated route (e.g. the holdout) cannot invalidate finished runs
        rspec["route_spec_sha256"] = _json_sha256(
            {k: rspec[k] for k in ("trajectory", "trajectory_sha256", "duration_s", "t0_epoch_s",
                                   "geometry_fingerprint", "seeds", "arms", "repeats_per_seed",
                                   "role")})

    spec = {
        "out_root": os.path.abspath(out_root),
        "domain_id": int(domain_id),
        "seeds": [int(s) for s in seeds],
        "repeats_per_seed": REPEATS_PER_SEED,
        "arms": [{"name": a, "overlay": (os.path.join(SIM_DIR, ov) if ov else None),
                  "overlay_sha256": (sc.sha256_file(os.path.join(SIM_DIR, ov)) if ov else None)}
                 for a, ov in ARMS],
        "threshold_m": THRESHOLD_M,
        "fit_until_s": FIT_UNTIL_S,
        "rate": RATE,
        "routes": routes,
        "bootstrap": {"rng_seed": BOOTSTRAP_RNG, "n": BOOTSTRAP_N,
                      "unit": "seed index (with replacement) out of %d seeds" % len(seeds),
                      "pairing": "one shared index draw per iteration for every arm",
                      "percentiles": [2.5, 97.5]},
        "claim_rule": ("stable-meets-target = every run of the route has T1 PASS and T2 PASS with "
                       "all gates green AND the overall T1 mean's 95% CI upper bound <= "
                       "threshold_m; a paired-difference CI crossing 0 = improvement not "
                       "demonstrated; below 0 is not an equivalence claim"),
    }
    spec["planned_runs"] = sum(len(r["arms"]) * len(r["seeds"]) * r["repeats_per_seed"]
                              for r in spec["routes"].values())
    spec["runner"] = {"path": os.path.abspath(__file__), "sha256": sc.sha256_file(__file__),
                      "version": "run_metric_repeats.py v1"}
    spec["tools"] = {name: {"path": path, "sha256": sc.sha256_file(path)}
                     for name, path in sorted(TOOL_FILES.items())}
    spec["spec_sha256"] = _json_sha256({k: spec[k] for k in
                                        ("seeds", "repeats_per_seed", "arms", "threshold_m",
                                         "fit_until_s", "rate", "routes", "bootstrap",
                                         "claim_rule")})
    return spec


def _effective_config(overlay_path, work_dir):
    """Compose the effective lio config exactly as the launch file does (single source)."""
    import sim_config
    os.makedirs(work_dir, exist_ok=True)
    config, base = sim_config.compose(None, overlay_path)
    return config, base


def _probe_ranges():
    """Compose the effective config ONCE (no overlay) to get the bag's range limits.

    The overlays cannot move these keys (the launch pins them fail-closed), so one probe covers
    every arm; the run then re-verifies the ranges on the yaml the node actually loaded.
    """
    cfg, base = _effective_config(
        None, os.path.join(tempfile.gettempdir(), "run_metric_repeats_config_probe"))
    return {"lidar_min_range": cfg.get("lidar_min_range"),
            "lidar_max_range": cfg.get("lidar_max_range")}, base


def _bag_command(spec, route, seed, seed_traj_path, bag_dir, ranges):
    """The exact bag-generation command, shared by the plan and the execution path."""
    return _py(os.path.join(SIM_DIR, "synthetic_data", "generate_test_bag.py"),
               "--out", bag_dir, "--trajectory", seed_traj_path,
               "--duration", spec["routes"][route]["duration_s"],
               "--min-range", ranges["lidar_min_range"], "--max-range", ranges["lidar_max_range"])


def prologue_commands(spec):
    """The exact commands that create the shared inputs (reference per route, bag per seed)."""
    out = []
    ranges, _base = _probe_ranges()
    for route, rspec in spec["routes"].items():
        out.append({"stage": "scene_reference", "route": route,
                    "command": shlex.join(_py(
                        os.path.join(SIM_DIR, "synthetic_data", "scene_reference.py"),
                        "--out-dir", os.path.join(spec["out_root"], "scene_ref", route),
                        "--trajectory", rspec["trajectory"], "--localization-trajectory", "",
                        "--duration", rspec["duration_s"], "--eval-start-s", FIT_UNTIL_S))})
        for seed in rspec["seeds"]:
            bag_root = os.path.join(spec["out_root"], "bags", route, "seed%02d" % seed)
            out.append({"stage": "generate_test_bag", "route": route, "seed": int(seed),
                        "command": shlex.join(_bag_command(
                            spec, route, seed, os.path.join(bag_root,
                                                            "trajectory_seed%02d.json" % seed),
                            os.path.join(bag_root, "bag"), ranges))})
    return out


def _attempt_dir(base_dir, attempt):
    """Every execution of a logical run gets its OWN attemptN directory.

    An earlier attempt is NEVER renamed, reused or written into, so a failed or interrupted run
    keeps its complete evidence (including a half-written one) and the number of attempts stays
    auditable.
    """
    return os.path.join(base_dir, "attempt%d" % int(attempt))


def _existing_attempts(base_dir):
    if not os.path.isdir(base_dir):
        return []
    found = []
    for name in os.listdir(base_dir):
        match = re.match(r"attempt(\d+)$", name)
        if match and os.path.isdir(os.path.join(base_dir, name)):
            found.append(int(match.group(1)))
    return sorted(found)


def plan_runs(spec):
    """Expand the full logical run list (attempt 1) WITHOUT executing anything.

    A logical run that needs to be (re-)executed later will run in a NEW attemptN subdirectory of
    the same base directory; the exact per-attempt commands are recorded in that attempt's record.
    """
    runs = []
    for route, rspec in spec["routes"].items():
        for seed in rspec["seeds"]:
            for arm in rspec["arms"]:
                for repeat in range(1, rspec["repeats_per_seed"] + 1):
                    base_dir = os.path.join(spec["out_root"], "runs", route, "seed%02d" % seed,
                                            arm, "repeat%d" % repeat)
                    runs.append(_run_plan_entry(spec, route, seed, arm, repeat, base_dir, 1))
    return runs


def _run_plan_entry(spec, route, seed, arm, repeat, base_dir, attempt):
    rspec = spec["routes"][route]
    run_dir = _attempt_dir(base_dir, attempt)
    bag_root = os.path.join(spec["out_root"], "bags", route, "seed%02d" % seed)
    seed_traj = os.path.join(bag_root, "trajectory_seed%02d.json" % seed)
    overlay = [a["overlay"] for a in spec["arms"] if a["name"] == arm][0]
    mapping = ["bash", os.path.join(HERE, "run_mapping_sim.sh"),
               "--run-dir", run_dir, "--work-dir", os.path.join(run_dir, "work"),
               "--bag-dir", os.path.join(bag_root, "bag"), "--skip-bag",
               "--trajectory", seed_traj, "--rate", RATE, "--fit-until-s", FIT_UNTIL_S,
               "--domain-id", spec["domain_id"]]
    if overlay:
        mapping += ["--extra-config", overlay]
    return {
        "run_id": "%s/seed%02d/%s/repeat%d" % (route, seed, arm, repeat),
        "route": route, "seed": int(seed), "arm": arm, "repeat": int(repeat),
        "attempt": int(attempt), "base_dir": base_dir, "run_dir": run_dir,
        "seed_trajectory": seed_traj,
        "bag_dir": os.path.join(bag_root, "bag"),
        "commands": {
            "mapping": shlex.join([str(a) for a in mapping]),
            "frame_chain": shlex.join(_py(
                os.path.join(HERE, "make_sim_transform.py"),
                "--gt-tum", os.path.join(spec["out_root"], "scene_ref", route, "gt_mapping.tum"),
                "--odom-tum", os.path.join(run_dir, "odom.tum"),
                "--t0-epoch", rspec["t0_epoch_s"], "--fit-until-s", FIT_UNTIL_S,
                "--apply-map", os.path.join(run_dir, "map.pcd"),
                "--out-map", os.path.join(run_dir, "frozen_map.pcd"),
                "--out", os.path.join(run_dir, "frame_chain.json"))),
            "t1": shlex.join(_py(os.path.join(HERE, "test_t1_accuracy.py"),
                                 "--run-dir", run_dir,
                                 "--ref-dir", os.path.join(spec["out_root"], "scene_ref", route),
                                 "--out", os.path.join(run_dir, "t1_result.json"))),
            "t2": shlex.join(_py(os.path.join(HERE, "test_t2_speed.py"),
                                 "--run-dir", run_dir, "--trajectory", seed_traj,
                                 "--out", os.path.join(run_dir, "t2_result.json"))),
        },
        "spec_sha256": spec["spec_sha256"],
    }


# ------------------------------------------------------------------ bag per seed ----
def ensure_seed_inputs(spec, route, seed, stats):
    """Seed trajectory + bag, generated ONCE per (route, seed) and reused by every arm."""
    rspec = spec["routes"][route]
    bag_root = os.path.join(spec["out_root"], "bags", route, "seed%02d" % seed)
    seed_traj_path = os.path.join(bag_root, "trajectory_seed%02d.json" % seed)
    bag_dir = os.path.join(bag_root, "bag")
    bag_info_path = os.path.join(bag_root, "bag_info.json")

    traj = tc.apply_variant(tc.load_trajectory(rspec["trajectory"]), seed=seed)
    geom = geometry_fingerprint(traj)
    if geom != rspec["geometry_fingerprint"]:
        _expect("seed %d changed the trajectory geometry (%s != reference %s)"
                % (seed, geom, rspec["geometry_fingerprint"]))

    os.makedirs(bag_root, exist_ok=True)
    body = json.dumps(traj, indent=1, sort_keys=True) + "\n"
    if not os.path.isfile(seed_traj_path) or open(seed_traj_path).read() != body:
        if os.path.isfile(seed_traj_path):
            os.replace(seed_traj_path, seed_traj_path + ".old-" + sc.utcnow())
        with open(seed_traj_path, "w") as fh:
            fh.write(body)

    config, base = _effective_config(None, os.path.join(bag_root, "config"))
    ranges = {"lidar_min_range": config.get("lidar_min_range"),
              "lidar_max_range": config.get("lidar_max_range")}

    info = _read_json(bag_info_path)
    if info and os.path.isdir(bag_dir):
        same_files = info.get("bag_files_sha256") == _dir_files_sha256(bag_dir)
        same_traj = info.get("seed_trajectory_sha256") == sc.sha256_file(seed_traj_path)
        if same_files and same_traj and info.get("ranges") == ranges:
            stats["seed_bags_reused"] = stats.get("seed_bags_reused", 0) + 1
            return {"seed_trajectory": seed_traj_path, "bag_dir": bag_dir, "bag_info": info,
                    "reused": True}
        if not same_files:
            # the recorded input no longer matches the bag on disk: never silently reuse it
            os.replace(bag_dir, bag_dir + ".mismatch-" + sc.utcnow())
        if not same_traj:
            os.replace(seed_traj_path, seed_traj_path + ".mismatch-" + sc.utcnow())
            with open(seed_traj_path, "w") as fh:
                fh.write(body)
    elif os.path.isdir(bag_dir):
        os.replace(bag_dir, bag_dir + ".old-" + sc.utcnow())

    cmd = _bag_command(spec, route, seed, seed_traj_path, bag_dir, ranges)
    res = _run(cmd, os.path.join(bag_root, "generate_bag.log"), 7200)
    if res["rc"] != 0 or not os.path.isdir(bag_dir):
        _expect("bag generation failed for %s seed %d (rc=%s, log %s)"
                % (route, seed, res["rc"], res["log"]))
    info = {
        "route": route, "seed": int(seed),
        "bag_dir": os.path.abspath(bag_dir),
        "seed_trajectory": os.path.abspath(seed_traj_path),
        "seed_trajectory_sha256": sc.sha256_file(seed_traj_path),
        "geometry_fingerprint": geom,
        "noise_fingerprint": noise_fingerprint(traj),
        "bag_files_sha256": _dir_files_sha256(bag_dir),
        "content_hash": bag_content_hash(bag_dir),
        "ranges": ranges,
        "base_profile": base,
        "command": res["cmd"],
        "wall_s": res["wall_s"],
        "generated_utc": sc.utcnow(),
        "note": "generated once per (route, seed) and reused by every arm via "
                "run_mapping_sim.sh --skip-bag; the bag FILE hash is not a reproducibility "
                "check, the decoded content hash is",
    }
    sc.write_json(bag_info_path, info)
    stats["seed_bags_generated"] = stats.get("seed_bags_generated", 0) + 1
    return {"seed_trajectory": seed_traj_path, "bag_dir": bag_dir, "bag_info": info,
            "reused": False}


# ------------------------------------------------------------------ one full run ----
def _attempt_record(manifest, run_id, base_dir, attempt):
    """Compact, auditable view of one attempt, from the manifest or from the disk alone."""
    key = "%s#%d" % (run_id, int(attempt))
    known = (manifest.get("attempts") or {}).get(key)
    entry = {"run_id": run_id, "attempt": int(attempt),
             "dir": _attempt_dir(base_dir, attempt)}
    if known:
        entry.update({k: known.get(k) for k in ("status", "metric_status", "gates_ok",
                                                "exit_codes", "finished_utc", "t1_verdict",
                                                "t2_verdict")})
    else:
        entry.update({"status": "unrecorded", "metric_status": None, "gates_ok": None,
                      "note": "the attempt directory exists but this manifest has no recorded "
                              "result for it (the previous session was interrupted between "
                              "creating the directory and recording the run); it is kept and "
                              "counted as an attempt, never reused or overwritten"})
    return entry


def _register_attempt(manifest, rec):
    """Append/replace this attempt in the manifest's attempt registry (all attempts counted)."""
    attempt = int(rec.get("attempt") or 1)
    key = "%s#%d" % (rec["run_id"], attempt)
    manifest.setdefault("attempts", {})[key] = {
        "run_id": rec["run_id"], "route": rec["route"], "seed": rec["seed"], "arm": rec["arm"],
        "repeat": rec["repeat"], "attempt": attempt, "run_dir": rec["run_dir"],
        "status": rec.get("status"), "metric_status": rec.get("metric_status"),
        "gates_ok": rec.get("gates_ok"), "exit_codes": rec.get("exit_codes"),
        "t1_verdict": (rec.get("t1") or {}).get("verdict"),
        "t2_verdict": (rec.get("t2") or {}).get("verdict"),
        "finished_utc": (rec.get("finished_utc") or rec.get("resumed_utc")
                         or rec.get("started_utc")),
        "recorded_utc": sc.utcnow(),
    }


def run_complete(rec, route_spec_sha256=None):
    """True when a previously recorded run still has complete, hash-matching evidence."""
    if not rec or rec.get("status") != "complete":
        return False, "no complete record"
    if route_spec_sha256 and rec.get("route_spec_sha256") != route_spec_sha256:
        return False, "recorded under a different frozen route definition"
    hashes = rec.get("hashes") or {}
    for key in ("t1_verdict", "t2_verdict", "t1_result", "t2_result", "odom", "frame_chain",
                "effective_yaml"):
        entry = hashes.get(key)
        if not entry:
            return False, "record has no %s hash" % key
        if not os.path.isfile(entry["path"]):
            return False, "missing %s" % key
        if sc.sha256_file(entry["path"]) != entry["sha256"]:
            return False, "%s changed on disk" % key
    return True, "complete evidence with matching hashes"


def execute_run(spec, entry, run_dir):
    route, seed, arm, repeat = entry["route"], entry["seed"], entry["arm"], entry["repeat"]
    rspec = spec["routes"][route]
    seed_inputs = entry["seed_inputs"]
    bag_info = seed_inputs.get("bag_info") or {}
    rec = {"run_id": entry["run_id"], "route": route, "seed": seed, "arm": arm,
           "repeat": repeat, "run_dir": os.path.abspath(run_dir),
           "base_dir": entry["base_dir"], "attempt": int(entry.get("attempt") or 1),
           "spec_sha256": spec["spec_sha256"], "started_utc": sc.utcnow(),
           "route_spec_sha256": rspec["route_spec_sha256"],
           "runner_sha256": spec["runner"]["sha256"],
           "seed_trajectory": seed_inputs["seed_trajectory"],
           "seed_trajectory_sha256": sc.sha256_file(seed_inputs["seed_trajectory"]),
           "bag_dir": seed_inputs["bag_dir"],
           "bag_files_sha256": bag_info.get("bag_files_sha256"),
           "bag_content_sha256": (bag_info.get("content_hash") or {}).get("sha256"),
           "bag_content_padding_free": (bag_info.get("content_hash") or {}).get("padding_free"),
           "geometry_fingerprint": rspec["geometry_fingerprint"],
           "noise_fingerprint": bag_info.get("noise_fingerprint"),
           "commands": entry["commands"], "gates": {}, "exit_codes": {}, "notes": []}

    if os.path.isdir(run_dir):
        # the attempt directory was selected as a NEW one; an impossible collision must fail loudly
        # instead of writing into evidence that already exists
        _expect("attempt directory %s already exists: refusing to write into it" % run_dir)
    os.makedirs(run_dir)

    env = dict(os.environ)
    env["SIM_DOMAIN_ID"] = str(spec["domain_id"])
    env["ROS_DOMAIN_ID"] = str(spec["domain_id"])
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["SIM_INSTALL_SETUP"] = spec["ros_install_setup"]

    mapping_log = os.path.join(run_dir, "run_mapping.log")
    res = _run(shlex.split(entry["commands"]["mapping"]), mapping_log, spec["run_timeout_s"],
               env=env)
    rec["exit_codes"]["mapping"] = res["rc"]
    rec["mapping"] = {"cmd": res["cmd"], "rc": res["rc"], "timed_out": res["timed_out"],
                      "wall_s": res["wall_s"], "log": res["log"]}
    log_text = open(mapping_log).read() if os.path.isfile(mapping_log) else ""
    rec["gates"]["bag_reused"] = "reusing existing bag" in log_text

    odom = os.path.join(run_dir, "odom.tum")
    rec["gates"]["odom_recorded"] = bool(os.path.isfile(odom) and os.path.getsize(odom) > 0)
    rec["gates"]["latency_evidence_present"] = os.path.isfile(
        os.path.join(run_dir, "latency_evidence.json"))

    # --- interface/range gate on the config the node actually loaded ------------------
    eff_yaml = os.path.join(run_dir, "work", "lio_sim.yaml")
    loaded = {}
    if os.path.isfile(eff_yaml):
        try:
            import yaml
            with open(eff_yaml) as fh:
                loaded = yaml.safe_load(fh) or {}
        except (OSError, ValueError):
            loaded = {}
    expected = rspec.get("interface_expectation") or {}
    deltas = {}
    for key, want in expected.items():
        got = loaded.get(key)
        if got != want:
            deltas[key] = {"expected": want, "actual": got}
    rec["gates"]["interface_keys_match"] = bool(expected and not deltas)
    if deltas:
        rec["interface_deltas"] = deltas

    # --- bag availability (T2's availability basis) -----------------------------------
    avail = _read_json(os.path.join(run_dir, "bag_availability.json")) or {}
    rec["gates"]["bag_availability_ok"] = bool(avail.get("checks")
                                               and all(avail["checks"].values()))

    # --- frozen frame chain -----------------------------------------------------------
    if rec["gates"]["odom_recorded"]:
        res = _run(shlex.split(entry["commands"]["frame_chain"]),
                   os.path.join(run_dir, "frame_chain.log"), 1800, env=env)
        rec["exit_codes"]["frame_chain"] = res["rc"]
    else:
        rec["exit_codes"]["frame_chain"] = None
        rec["notes"].append("no odometry recorded: the mapping run is invalid")
    rec["gates"]["frame_chain_ok"] = bool(
        rec["exit_codes"].get("frame_chain") == 0
        and os.path.isfile(os.path.join(run_dir, "frame_chain.json"))
        and os.path.isfile(os.path.join(run_dir, "frozen_map.pcd")))

    # --- T1 / T2 ----------------------------------------------------------------------
    if rec["gates"]["frame_chain_ok"]:
        rec["exit_codes"]["T1"] = _run(shlex.split(entry["commands"]["t1"]),
                                       os.path.join(run_dir, "t1.log"), 3600, env=env)["rc"]
    else:
        rec["exit_codes"]["T1"] = None
        rec["notes"].append("T1 skipped: no frozen frame chain")
    if rec["gates"]["latency_evidence_present"]:
        rec["exit_codes"]["T2"] = _run(shlex.split(entry["commands"]["t2"]),
                                       os.path.join(run_dir, "t2.log"), 3600, env=env)["rc"]
    else:
        rec["exit_codes"]["T2"] = None
        rec["notes"].append("T2 skipped: the mapping run produced no latency evidence")

    t1 = _read_json(os.path.join(run_dir, "t1_verdict.json"))
    t2 = _read_json(os.path.join(run_dir, "t2_verdict.json"))
    rec["t1"] = _summarize_t1(t1)
    rec["t2"] = _summarize_t2(t2)

    # --- input-provenance gates --------------------------------------------------------
    refs = rspec.get("reference") or {}
    t1_inputs = (t1 or {}).get("inputs") or {}
    frame_path = os.path.join(run_dir, "frame_chain.json")
    frame_sha = sc.sha256_file(frame_path) if os.path.isfile(frame_path) else None
    rec["gates"]["t1_inputs_match_reference"] = bool(
        t1_inputs.get("ref_map_sha256") == refs.get("reference_map_sha256")
        and t1_inputs.get("roi_sha256") == refs.get("roi_sha256")
        and frame_sha is not None
        and t1_inputs.get("transform_json_sha256") == frame_sha)
    rec["gates"]["seed_geometry_matches_reference"] = bool(
        rec["geometry_fingerprint"] == refs.get("geometry_fingerprint"))
    recorded = spec["bag_infos"].get("%s/seed%02d" % (route, seed)) or {}
    rec["gates"]["bag_files_unchanged"] = bool(recorded.get("bag_files_sha256")
                                               and recorded["bag_files_sha256"]
                                               == rec["bag_files_sha256"])
    rec["gates"]["bag_content_hash_recorded"] = bool(rec["bag_content_sha256"])

    rec["hashes"] = _hash_map(run_dir, {
        "t1_verdict": "t1_verdict.json", "t2_verdict": "t2_verdict.json",
        "t1_result": "t1_result.json", "t2_result": "t2_result.json",
        "odom": "odom.tum", "frame_chain": "frame_chain.json",
        "map": "map.pcd", "map_eval": "map_eval.pcd", "frozen_map": "frozen_map.pcd",
        "latency_evidence": "latency_evidence.json", "record_run": "record_run.json",
        "effective_yaml": os.path.join("work", "lio_sim.yaml"),
    })

    rec["status"] = "complete" if bool(rec["exit_codes"].get("T1") is not None
                                       and rec["exit_codes"].get("T2") is not None
                                       and t1 is not None and t2 is not None) else "incomplete"
    rec["gates_ok"] = all(rec["gates"].values())
    rec["metric_status"] = _metric_status(rec, rspec)
    rec["finished_utc"] = sc.utcnow()
    sc.write_json(os.path.join(run_dir, "repeat_run.json"), rec)
    return rec


def _hash_map(run_dir, files):
    out = {}
    for key, rel in files.items():
        path = os.path.join(run_dir, rel)
        if os.path.isfile(path):
            out[key] = {"path": path, "sha256": sc.sha256_file(path),
                        "bytes": os.path.getsize(path)}
    return out


def _summarize_t1(t1):
    if not t1:
        return None
    return {"verdict": t1.get("verdict"), "raw_p2pl_rmse_m": t1.get("raw_p2pl_rmse_m"),
            "threshold_m": t1.get("threshold_m"),
            "denominator_fixed_n": t1.get("denominator_fixed_n"),
            "coverage_within_5cm": t1.get("coverage_within_5cm"),
            "evaluator_status": t1.get("evaluator_status"),
            "evaluator_pass": t1.get("evaluator_pass"),
            "worst_region": t1.get("worst_region"),
            "worst_region_p2pl_p99_m": t1.get("worst_region_p2pl_p99_m"),
            "reason": t1.get("reason")}


def _summarize_t2(t2):
    if not t2:
        return None
    return {"verdict": t2.get("verdict"),
            "observed_latency_p95_s": t2.get("observed_latency_p95_s"),
            "latency_p99_s": t2.get("latency_p99_s"),
            "threshold_s": t2.get("threshold_s"),
            "delivery_ratio": t2.get("delivery_ratio"),
            "real_time_factor": t2.get("real_time_factor"),
            "failed_checks": t2.get("failed_checks"),
            "core_speed_mean_mps": ((t2.get("route") or {}).get("core_speed_mean_mps")),
            "gt_moving_speed_in_envelope": ((t2.get("checks") or {})
                                            .get("gt_moving_speed_in_envelope"))}


def _metric_status(rec, rspec):
    if rec["status"] != "complete":
        return "incomplete"
    if not rec["gates_ok"]:
        return "gates_failed"
    t1v = (rec["t1"] or {}).get("verdict")
    t2v = (rec["t2"] or {}).get("verdict")
    if t1v == "PASS" and t2v == "PASS":
        return "pass"
    if t1v == "BLOCKED" or t2v == "BLOCKED":
        return "blocked"
    if rspec.get("stress_test_only") and t1v == "PASS" and t2v == "FAIL":
        failed = (rec["t2"] or {}).get("failed_checks") or []
        if failed and set(failed) == {"gt_moving_speed_in_envelope"}:
            return "stress_only"
    return "fail"


# -------------------------------------------------------------------- aggregation ----
def _index_matrix(spec):
    """Frozen bootstrap index matrix: one shared draw per iteration for every statistic."""
    rng = np.random.default_rng(int(spec["bootstrap"]["rng_seed"]))
    n_seeds = len(spec["seeds"])
    return rng.integers(0, n_seeds, size=(int(spec["bootstrap"]["n"]), n_seeds))


def _bootstrap_stats(seed_values, seed_labels, index_matrix, threshold):
    """Per-seed means -> overall mean + 2.5/97.5 bootstrap CI over shared seed indices."""
    arr = np.array([np.nan if v is None else float(v) for v in seed_values], dtype=float)
    stats = {"per_seed_means": {("%d" % s): (None if not np.isfinite(v) else float(v))
                                for s, v in zip(seed_labels, arr)},
             "n_seeds": int(arr.size),
             "n_seeds_present": int(np.isfinite(arr).sum()),
             "mean_over_seeds": None, "ci_low": None, "ci_high": None,
             "ci_upper_le_threshold": None, "threshold_m": threshold}
    if arr.size == 0 or not np.all(np.isfinite(arr)):
        stats["blocker"] = ("at least one seed has no complete repeat set; no aggregate CI is "
                            "computed (a missing seed is never interpolated)")
        return stats
    stats["mean_over_seeds"] = float(arr.mean())
    samples = arr[index_matrix].mean(axis=1)
    stats["ci_low"] = float(np.percentile(samples, 2.5))
    stats["ci_high"] = float(np.percentile(samples, 97.5))
    if threshold is not None:
        stats["ci_upper_le_threshold"] = bool(stats["ci_high"] <= threshold)
    return stats


def _status_counts(statuses):
    out = {}
    for s in statuses:
        out[s] = out.get(s, 0) + 1
    return out


def _stable_text(arm, t1_stats, threshold):
    return ("arm %s: every run passed T1 and T2 with all gates green and the overall T1 mean "
            "95%% CI upper bound %.4f m <= %.2f m" % (arm, t1_stats["ci_high"], threshold))


def _not_stable_text(arm, all_pass, t1_stats, threshold):
    if not all_pass:
        return ("arm %s: NOT stable -- at least one run did not pass both T1 and T2, is missing, "
                "or failed a gate; failures stay in the aggregate" % arm)
    if t1_stats["ci_high"] is None:
        return "arm %s: NOT stable -- no complete aggregate CI (see blocker)" % arm
    return ("arm %s: NOT stable -- overall T1 mean 95%% CI upper bound %.4f m > %.2f m"
            % (arm, t1_stats["ci_high"], threshold))


def aggregate(spec, records, counts, index_matrix):
    boot = {"n": int(index_matrix.shape[0]), "rng_seed": spec["bootstrap"]["rng_seed"],
            "index_matrix_sha256": hashlib.sha256(
                index_matrix.astype("<i8").tobytes()).hexdigest(),
            "unit": "seed index (with replacement)",
            "pairing": "one shared index draw per iteration for every arm",
            "percentiles": [2.5, 97.5]}
    out = {"tool": "run_metric_repeats.py v1", "frozen_spec": spec, "spec_sha256":
           spec["spec_sha256"], "bootstrap": boot, "counts": counts, "routes": {},
           "debug_only": bool(spec.get("debug_only")),
           "debug_only_reason": spec.get("debug_only_reason")}

    for route, rspec in spec["routes"].items():
        recs = [r for r in records if r["route"] == route]
        planned = [e["run_id"] for e in spec["planned_index"][route]]
        route_out = {
            "trajectory": rspec["trajectory"], "trajectory_sha256": rspec["trajectory_sha256"],
            "geometry_fingerprint": rspec["geometry_fingerprint"],
            "reference": rspec.get("reference"), "speed": rspec["speed"], "role": rspec["role"],
            "stress_test_only": bool(rspec.get("stress_test_only")),
            "planned_runs": len(planned), "recorded_runs": len({r["run_id"] for r in recs}),
            "missing_runs": sorted(set(planned) - {r["run_id"] for r in recs}),
            "arms": {}, "paired_vs_baseline": {}, "per_run": [],
        }
        # every seed must be a genuinely DIFFERENT input: the decoded content hash (not the bag
        # file hash, which is not a reproducibility check) has to differ between the seeds
        bags = {k: v for k, v in (spec.get("bag_infos") or {}).items()
                if k.startswith(route + "/")}
        route_out["seed_inputs"] = {
            k: {"bag_content_sha256": (v.get("content_hash") or {}).get("sha256"),
                "bag_content_padding_free": (v.get("content_hash") or {}).get("padding_free"),
                "bag_files_sha256": v.get("bag_files_sha256"),
                "noise_fingerprint": v.get("noise_fingerprint"),
                "seed_trajectory_sha256": v.get("seed_trajectory_sha256"),
                "ranges": v.get("ranges")}
            for k, v in sorted(bags.items())}
        contents = [x["bag_content_sha256"] for x in route_out["seed_inputs"].values()]
        route_out["seed_inputs_distinct"] = bool(contents) and len(set(contents)) == len(contents)
        if not route_out["seed_inputs_distinct"]:
            route_out["seed_inputs_note"] = ("the seeds do not all have distinct decoded bag "
                                             "content hashes: the seed override may have failed")
        seed_t1, seed_t2 = {}, {}
        for arm in rspec["arms"]:
            arm_recs = [r for r in recs if r["arm"] == arm]
            per_seed_t1, per_seed_t2 = [], []
            for seed in spec["seeds"]:
                cell = [r for r in arm_recs if r["seed"] == seed]
                if len(cell) == rspec["repeats_per_seed"]:
                    per_seed_t1.append(_mean([_certified(r, "t1", "raw_p2pl_rmse_m")
                                              for r in cell]))
                    per_seed_t2.append(_mean([_certified(r, "t2", "observed_latency_p95_s")
                                              for r in cell]))
                else:
                    per_seed_t1.append(None)
                    per_seed_t2.append(None)
            seed_t1[arm], seed_t2[arm] = per_seed_t1, per_seed_t2
            statuses = [r["metric_status"] for r in arm_recs]
            n_expect = len(spec["seeds"]) * rspec["repeats_per_seed"]
            all_pass = bool(len(arm_recs) == n_expect and all(s == "pass" for s in statuses))
            t1_stats = _bootstrap_stats(per_seed_t1, spec["seeds"], index_matrix,
                                        spec["threshold_m"])
            t2_stats = _bootstrap_stats(per_seed_t2, spec["seeds"], index_matrix, None)
            stable = bool(all_pass and t1_stats["ci_high"] is not None
                          and t1_stats["ci_high"] <= spec["threshold_m"])
            route_out["arms"][arm] = {
                "runs": len(arm_recs), "runs_expected": n_expect,
                "status_counts": _status_counts(statuses), "all_runs_pass": all_pass,
                "uncertified_runs": sum(1 for r in arm_recs
                                        if _certified(r, "t1", "raw_p2pl_rmse_m") is None
                                        or _certified(r, "t2", "observed_latency_p95_s") is None),
                "t1_raw_p2pl_rmse_m": t1_stats, "t2_observed_latency_p95_s": t2_stats,
                "stable_meets_target": stable,
                "claim": (_stable_text(arm, t1_stats, spec["threshold_m"]) if stable else
                          _not_stable_text(arm, all_pass, t1_stats, spec["threshold_m"])),
            }
        base = seed_t1.get("baseline") or []
        for arm in rspec["arms"]:
            if arm == "baseline":
                continue
            diffs = []
            for i, _seed in enumerate(spec["seeds"]):
                a, b = seed_t1[arm][i], base[i]
                diffs.append(None if a is None or b is None else a - b)
            stats = _bootstrap_stats(diffs, spec["seeds"], index_matrix, None)
            stats["mean_diff_m"] = stats.pop("mean_over_seeds")
            stats["ci_crosses_zero"] = bool(stats["ci_low"] is not None
                                            and stats["ci_low"] <= 0.0 <= stats["ci_high"])
            stats["improvement"] = ("not_demonstrated"
                                    if (stats["ci_high"] is None or stats["ci_crosses_zero"])
                                    else "ci_below_zero_no_equivalence_claim")
            stats["note"] = ("paired over the 5 seed means with shared bootstrap indices; a CI "
                             "crossing 0 means the improvement is NOT demonstrated, a CI entirely "
                             "below 0 is not an equivalence claim and is not attributed to the "
                             "algorithm")
            route_out["paired_vs_baseline"][arm] = stats
        route_out["all_runs_pass"] = bool(route_out["arms"]
                                          and all(a["all_runs_pass"]
                                                  for a in route_out["arms"].values()))
        route_out["stable_meets_target"] = bool(
            route_out["arms"] and all(a["stable_meets_target"]
                                      for a in route_out["arms"].values()))
        route_out["per_run"] = [{
            "run_id": r["run_id"], "seed": r["seed"], "arm": r["arm"], "repeat": r["repeat"],
            "attempt": r.get("attempt"), "attempts_total": r.get("attempts_total"),
            "status": r["status"], "metric_status": r["metric_status"], "gates_ok": r["gates_ok"],
            "gates": r["gates"], "t1_verdict": (r["t1"] or {}).get("verdict"),
            "t1_raw_p2pl_rmse_m": (r["t1"] or {}).get("raw_p2pl_rmse_m"),
            "t2_verdict": (r["t2"] or {}).get("verdict"),
            "t2_p95_s": (r["t2"] or {}).get("observed_latency_p95_s"),
            "metric_status_source": "recorded", "run_dir": r["run_dir"],
        } for r in sorted(recs, key=lambda r: (r["arm"], r["seed"], r["repeat"]))]
        out["routes"][route] = route_out

    out["claim"] = {
        "main_path_stable_meets_target": bool(
            (out["routes"].get(MAIN_ROUTE) or {}).get("stable_meets_target")),
        "holdout_path_stable_meets_target": bool(
            (out["routes"].get(HOLDOUT_ROUTE) or {}).get("stable_meets_target")),
        "text": ("stable-meets-target requires EVERY run of the route to have its own T1 and T2 "
                 "PASS with all gates green AND the overall T1 mean's 95%% CI upper bound <= "
                 "%.2f m.  The holdout route is a second PATH in the same frozen synthetic "
                 "scene: it adds no cross-scenario generalization claim.  A CI difference is "
                 "never called an improvement of the algorithm." % spec["threshold_m"]),
        "stress_test_only_routes": [r for r, rs in spec["routes"].items()
                                    if rs.get("stress_test_only")],
        "seed_inputs_distinct": {r: bool(v.get("seed_inputs_distinct"))
                                 for r, v in out["routes"].items()},
    }
    return out


def _certified(rec, key, field):
    """The measurement only when the frozen evaluator certified it (PASS/FAIL).

    A BLOCKED verdict means the evaluator refused to certify the number (admissibility gate
    not met) or produced none: such a run is KEPT in the record and in the counts, but its
    uncertified value must never enter a mean, so the affected seed has no mean and the
    aggregate reports a blocker instead of a number.
    """
    if rec.get("status") != "complete":
        return None
    entry = rec.get(key) or {}
    if entry.get("verdict") not in ("PASS", "FAIL"):
        return None
    return entry.get(field)


def _mean(values):
    vals = [v for v in values if v is not None]
    if not vals or len(vals) != len(values):
        return None
    return float(np.mean(vals))


def _count_summary(records):
    statuses = [r["metric_status"] for r in records]
    return {
        "runs_recorded": len(records),
        "runs_complete": sum(1 for r in records if r["status"] == "complete"),
        "runs_incomplete": sum(1 for r in records if r["status"] != "complete"),
        "runs_pass": sum(1 for s in statuses if s == "pass"),
        "runs_fail": sum(1 for s in statuses if s == "fail"),
        "runs_blocked": sum(1 for s in statuses if s == "blocked"),
        "runs_gates_failed": sum(1 for s in statuses if s == "gates_failed"),
        "runs_stress_only": sum(1 for s in statuses if s == "stress_only"),
        # attempt accounting: a re-run is a NEW attempt, so every execution stays counted
        "runs_attempted_more_than_once": sum(1 for r in records if r.get("previous_attempts")),
        "attempts_of_current_runs": sum(int(r.get("attempts_total") or 1) for r in records),
    }


# --------------------------------------------------------------------------- main ----
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-root", required=True, help="evidence root for the whole repeat run")
    ap.add_argument("--domain-id", type=int, required=True,
                    help="exclusive ROS_DOMAIN_ID for every run (must have no live nodes)")
    ap.add_argument("--seeds", default=",".join(str(s) for s in DEFAULT_SEEDS))
    ap.add_argument("--main-trajectory",
                    default=os.path.join(SIM_DIR, "synthetic_data", "test_trajectory.json"))
    ap.add_argument("--holdout-trajectory",
                    default=os.path.join(SIM_DIR, "synthetic_data",
                                         "test_trajectory_localization.json"))
    ap.add_argument("--run-timeout-s", type=float, default=1800.0,
                    help="outer wall-clock guard per mapping run (the mapping script bounds "
                         "itself as well)")
    ap.add_argument("--resume", action="store_true",
                    help="continue an existing out-root: skip ONLY runs whose complete evidence "
                         "and input hashes still match; never delete, never retry a recorded FAIL")
    ap.add_argument("--plan", action="store_true",
                    help="expand and print the full run plan (with exact commands) and exit; "
                         "writes <out-root>.plan.json and does not touch the out-root")
    ap.add_argument("--skip-holdout", action="store_true",
                    help="DEBUG ONLY (never for the step-5 delivery): run only the 75 main-path "
                         "runs; the frozen design is 80 runs and the manifest is marked debug_only")
    args = ap.parse_args(argv)
    args.seeds = tuple(int(s) for s in str(args.seeds).split(",") if str(s).strip())
    if not args.seeds:
        _expect("--seeds must list at least one seed")
    return args


def domain_has_nodes(domain_id):
    cmd = ["bash", "-c",
           "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
           "export ROS_DOMAIN_ID=%d ROS_LOCALHOST_ONLY=1; ros2 node list" % int(domain_id)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError) as exc:
        return None, "could not query the ROS domain: %s" % exc
    if proc.returncode != 0:
        return None, "ros2 node list failed: %s" % (proc.stderr or proc.stdout).strip()[:300]
    return [n.strip() for n in proc.stdout.splitlines() if n.strip()], None


def main(argv=None):
    args = parse_args(argv)
    out_root = os.path.abspath(args.out_root)
    spec = build_spec(out_root, args.domain_id, args.seeds, args,
                      include_holdout=not args.skip_holdout)
    spec["run_timeout_s"] = float(args.run_timeout_s)
    setup = os.environ.get("SIM_INSTALL_SETUP") or os.path.join(REPO_ROOT, "install", "setup.bash")
    if not os.path.isfile(setup):
        _expect("ROS install setup not found: %s (export SIM_INSTALL_SETUP=<install/setup.bash>)"
                % setup)
    # every child (run_mapping_sim.sh) sources exactly this overlay; recorded so the evidence says
    # which build produced it (the default is the repository's own install)
    spec["ros_install_setup"] = os.path.abspath(setup)
    if args.skip_holdout:
        spec["debug_only"] = True
        spec["debug_only_reason"] = ("--skip-holdout was used: this plan is the 75-run main path "
                                     "only and is DEBUG evidence, not the step-5 delivery (which "
                                     "is the 80-run frozen design)")
        print("[warn] %s" % spec["debug_only_reason"])
    planned = plan_runs(spec)
    spec["planned_index"] = {}
    for entry in planned:
        spec["planned_index"].setdefault(entry["route"], []).append(entry)
    spec["planned_runs"] = len(planned)
    spec["prologue_commands"] = prologue_commands(spec)

    if args.plan:
        path = out_root.rstrip("/") + ".plan.json"
        sc.write_json(path, {"tool": "run_metric_repeats.py v1 --plan",
                             "created_utc": sc.utcnow(), "spec": spec, "runs": planned})
        print("[plan] %d runs (%s); plan written to %s"
              % (len(planned),
                 ", ".join("%s=%d" % (r, len(v)) for r, v in sorted(spec["planned_index"].items())),
                 path))
        for entry in spec["prologue_commands"]:
            print("  %s: %s" % (entry["stage"], entry["command"]))
        for entry in planned[:2]:
            print("  e.g. %s" % entry["commands"]["mapping"])
        print("[plan] a re-run of an incomplete run runs in a NEW attemptN directory (existing "
              "attempt directories are never reused or overwritten)")
        return 0

    manifest_path = os.path.join(out_root, "repeats_manifest.json")
    if os.path.exists(manifest_path) and not args.resume:
        _expect("%s exists: pass --resume to continue it (this runner never overwrites an "
                "existing evidence root)" % manifest_path)
    if os.path.isdir(out_root) and os.listdir(out_root) and not os.path.exists(manifest_path):
        _expect("out-root %s is not empty and has no repeats_manifest.json; refusing to write "
                "into it" % out_root)

    nodes, err = domain_has_nodes(args.domain_id)
    if err:
        _expect("ROS domain %d precheck failed: %s" % (args.domain_id, err))
    if nodes:
        _expect("ROS domain %d already has live nodes %s; pick a free domain" % (args.domain_id,
                                                                                nodes))
    os.makedirs(out_root, exist_ok=True)

    manifest = _read_json(manifest_path) if args.resume else None
    prior_refs = ((manifest or {}).get("spec") or {}).get("routes") or {}
    if not manifest:
        head, dirty = sc.git_head(REPO_ROOT)
        manifest = {"tool": "run_metric_repeats.py v1", "created_utc": sc.utcnow(),
                    "out_root": out_root, "domain_id": int(args.domain_id),
                    "repo": {"git_head": head, "git_dirty": dirty},
                    "run_order": [e["run_id"] for e in planned], "runs": [], "counts": {}}
    else:
        old_runner = (((manifest.get("spec") or {}).get("runner") or {}).get("sha256"))
        if old_runner and old_runner != spec["runner"]["sha256"]:
            note = ("resumed with a DIFFERENT runner revision (recorded %s, now %s): runs skipped "
                    "below were produced by the earlier revision" % (old_runner[:12],
                                                                     spec["runner"]["sha256"][:12]))
            manifest.setdefault("resume_warnings", []).append(
                {"utc": sc.utcnow(), "warning": note})
            print("[warn] %s" % note)
        if (manifest.get("spec") or {}).get("spec_sha256") != spec["spec_sha256"]:
            note = "the frozen spec changed since this out-root was created"
            manifest.setdefault("resume_warnings", []).append({"utc": sc.utcnow(),
                                                               "warning": note})
            print("[warn] %s" % note)
        old_setup = (manifest.get("spec") or {}).get("ros_install_setup")
        if old_setup and old_setup != spec["ros_install_setup"]:
            note = ("resumed with a DIFFERENT ROS install overlay (recorded %s, now %s): the "
                    "skipped runs were produced against the earlier build"
                    % (old_setup, spec["ros_install_setup"]))
            manifest.setdefault("resume_warnings", []).append({"utc": sc.utcnow(),
                                                               "warning": note})
            print("[warn] %s" % note)
    manifest["spec"] = spec
    if spec.get("debug_only"):
        logged = manifest.setdefault("debug_plans", [])
        if not logged or logged[-1].get("reason") != spec["debug_only_reason"]:
            logged.append({"utc": sc.utcnow(), "reason": spec["debug_only_reason"]})
    if manifest.get("run_order") and manifest["run_order"] != [e["run_id"] for e in planned]:
        manifest.setdefault("previous_run_orders", []).append(manifest["run_order"])
    manifest["run_order"] = [e["run_id"] for e in planned]
    manifest["domain_precheck"] = {"nodes": nodes, "checked_utc": sc.utcnow()}
    counts = manifest.setdefault("counts", {})
    for key in ("seed_bags_generated", "seed_bags_reused", "runs_executed",
                "runs_skipped_complete"):
        counts.setdefault(key, 0)

    # reference geometry/ROI/GT once per route (seed-independent by fingerprint)
    for route, rspec in spec["routes"].items():
        ref_dir = os.path.join(out_root, "scene_ref", route)
        ref_json = os.path.join(ref_dir, "scene_reference.json")
        if not os.path.isfile(ref_json):
            cmd = shlex.split([c["command"] for c in spec["prologue_commands"]
                               if c["route"] == route and c["stage"] == "scene_reference"][0])
            os.makedirs(ref_dir, exist_ok=True)
            res = _run(cmd, os.path.join(ref_dir, "scene_reference.log"), 7200)
            if res["rc"] != 0 or not os.path.isfile(ref_json):
                _expect("scene reference generation failed for %s (rc=%s, log %s)"
                        % (route, res["rc"], res["log"]))
        ref_manifest = _read_json(ref_json) or {}
        rspec["reference"] = {
            "dir": ref_dir,
            "scene_reference_json_sha256": sc.sha256_file(ref_json),
            "reference_map_sha256": sc.sha256_file(os.path.join(ref_dir, "reference_map.pcd")),
            "roi_sha256": sc.sha256_file(os.path.join(ref_dir, "roi.json")),
            "gt_mapping_sha256": sc.sha256_file(os.path.join(ref_dir, "gt_mapping.tum")),
            "trajectory_in_reference": (ref_manifest.get("trajectories") or {}).get("mapping"),
            "trajectory_sha256_in_reference": (ref_manifest.get("trajectories") or {}
                                               ).get("mapping_sha256"),
            "geometry_fingerprint": rspec["geometry_fingerprint"],
            "note": "generated once per route; every seed trajectory differs from it ONLY in "
                    "imu_noise.seed (geometry fingerprint asserted equal), so the reference, ROI "
                    "and ground truth are shared by every run of the route",
        }
        if rspec["reference"]["trajectory_in_reference"] != rspec["trajectory"] or \
                rspec["reference"]["trajectory_sha256_in_reference"] != rspec["trajectory_sha256"]:
            _expect("the existing reference in %s was built from a different trajectory (%s)"
                    % (ref_dir, rspec["reference"]["trajectory_in_reference"]))
        old = (prior_refs.get(route) or {}).get("reference") or {}
        for key in ("reference_map_sha256", "roi_sha256", "gt_mapping_sha256"):
            if old.get(key) and old[key] != rspec["reference"][key]:
                _expect("the reference artifact %s changed since the recorded run (%s: %s -> %s); "
                        "refusing to mix references" % (key, route, old[key],
                                                        rspec["reference"][key]))
        block = _effective_config(None, os.path.join(out_root, "config_ref", route))[0]
        rspec["interface_expectation"] = {k: block.get(k) for k in INTERFACE_KEYS}
        rspec["stress_test_only"] = not rspec["speed"]["moving_ge_target_and_le_max"]
        if rspec["stress_test_only"]:
            print("[warn] route %s is OUTSIDE the %.2f-%.2f m/s envelope: recorded as a stress "
                  "test only, no stability claim" % (route, TARGET_MPS, MAX_MPS))

    # recorded evidence is never deleted: prior records are looked up from both the current and
    # the superseded lists, and those outside the current plan are kept in superseded_runs
    prior_all = {}
    for r in list(manifest.get("runs", [])) + list(manifest.get("superseded_runs", [])):
        if isinstance(r, dict) and r.get("run_id"):
            prior_all.setdefault(r["run_id"], r)
    planned_ids = {e["run_id"] for e in planned}
    orphans = [r for rid, r in prior_all.items() if rid not in planned_ids]
    if orphans:
        manifest["superseded_runs"] = orphans
        manifest.setdefault("resume_warnings", []).append(
            {"utc": sc.utcnow(),
             "warning": "%d recorded run(s) are not part of the current plan and are kept in "
                        "superseded_runs" % len(orphans)})
        print("[warn] %d recorded run(s) kept in superseded_runs (not in the current plan)"
              % len(orphans))
    prior = {rid: r for rid, r in prior_all.items() if rid in planned_ids}
    records = []
    spec["bag_infos"] = {}
    seed_cache = {}
    index_matrix = _index_matrix(spec)
    t_start = time.time()
    for entry in planned:
        route, seed = entry["route"], entry["seed"]
        key = "%s/seed%02d" % (route, seed)
        if key not in seed_cache:
            # the seed's trajectory/bag are created once and then reused by every arm, so the
            # (large) bag is hashed once per seed per session, never once per run
            seed_cache[key] = ensure_seed_inputs(spec, route, seed, counts)
            spec["bag_infos"][key] = seed_cache[key].get("bag_info") or {}
        entry["seed_inputs"] = seed_cache[key]
        base_dir = entry["base_dir"]
        on_disk = _existing_attempts(base_dir)
        latest_on_disk = max(on_disk) if on_disk else 0
        history = [_attempt_record(manifest, entry["run_id"], base_dir, a) for a in on_disk]
        for item in history:
            if item.get("status") == "unrecorded":
                manifest.setdefault("attempts", {}).setdefault(
                    "%s#%d" % (entry["run_id"], item["attempt"]),
                    {"run_id": entry["run_id"], "attempt": item["attempt"],
                     "status": "unrecorded", "run_dir": item["dir"],
                     "note": item.get("note")})
        rec = prior.get(entry["run_id"])
        ok, why = run_complete(rec, spec["routes"][route]["route_spec_sha256"])
        if ok and rec.get("attempt") != latest_on_disk:
            ok, why = False, ("a later attempt (%d) exists on disk without a recorded result"
                              % latest_on_disk)
        if args.resume and ok:
            rec = dict(rec)
            rec["resumed"] = True
            rec["resume_action"] = "skipped: %s" % why
            rec["resumed_utc"] = sc.utcnow()
            counts["runs_skipped_complete"] += 1
            print("[skip] %s attempt%d (%s) metric_status=%s"
                  % (entry["run_id"], rec.get("attempt") or 1, why, rec.get("metric_status")))
        else:
            attempt = latest_on_disk + 1
            while os.path.isdir(_attempt_dir(base_dir, attempt)):
                attempt += 1
            run_entry = _run_plan_entry(spec, route, seed, entry["arm"], entry["repeat"],
                                        base_dir, attempt)
            run_entry["seed_inputs"] = seed_cache[key]
            print("[run ] %s attempt%d" % (entry["run_id"], attempt))
            rec = execute_run(spec, run_entry, run_entry["run_dir"])
            counts["runs_executed"] += 1
        rec = dict(rec)
        rec["base_dir"] = base_dir
        rec["attempts_total"] = max([latest_on_disk, int(rec.get("attempt") or 1)])
        rec["previous_attempts"] = [h for h in history if h.get("attempt") != rec.get("attempt")]
        records.append(rec)
        _register_attempt(manifest, rec)
        counts["attempts_total"] = len(manifest.get("attempts") or {})
        manifest["runs"] = records
        manifest["counts"] = dict(counts, **_count_summary(records))
        manifest["updated_utc"] = sc.utcnow()
        manifest["wall_s"] = round(time.time() - t_start, 1)
        sc.write_json(manifest_path, manifest)
        sc.write_json(os.path.join(out_root, "repeat_metrics.json"),
                      aggregate(spec, records, manifest["counts"], index_matrix))

    counts.update(_count_summary(records))
    counts["attempts_total"] = len(manifest.get("attempts") or {})
    manifest["counts"] = counts
    manifest["updated_utc"] = sc.utcnow()
    manifest["wall_s"] = round(time.time() - t_start, 1)
    sc.write_json(manifest_path, manifest)
    metrics = aggregate(spec, records, counts, index_matrix)
    sc.write_json(os.path.join(out_root, "repeat_metrics.json"), metrics)

    print("[done] %d runs: %s" % (len(records), json.dumps(counts, sort_keys=True)))
    for route, r in metrics["routes"].items():
        print("[done] %-8s all_runs_pass=%s stable_meets_target=%s"
              % (route, r["all_runs_pass"], r["stable_meets_target"]))
    print("[done] metrics : %s" % os.path.join(out_root, "repeat_metrics.json"))
    print("[done] manifest: %s" % manifest_path)
    hard = counts.get("runs_incomplete", 0) + counts.get("runs_gates_failed", 0) \
        + counts.get("runs_blocked", 0)
    if hard:
        print("[done] %d run(s) are incomplete/gate-failed/BLOCKED: their evidence is not "
              "certifiable (they are kept in the aggregate)" % hard)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
