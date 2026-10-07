#!/usr/bin/env python3
"""Preflight check of a deployment YAML before it is loaded onto the robot.

WHY THIS EXISTS
---------------
``lio_orin_nx.yaml`` is an intentionally-unvalidated starting point, and the
failure mode that motivated this repository's own policy is a placeholder value
being mistaken for a measured one.  A node that logs one warning at startup is
easy to miss in a bring-up log; a check that REFUSES to call a profile ready
unless every sensor-dependent value is either filled in or explicitly waived is
not.

This script encodes the repository's rules as executable checks:

  1. ``ext_il`` must be present and a 7-element sequence (the code silently falls
     back to r_il = I, t_il = 0 when it is absent or malformed, which is a
     placeholder, not an extrinsic).  Its quaternion must be normalised.
  2. ``pcl2_time_field`` must be non-empty AND the scale must be positive, or the
     scan gets no motion compensation at all.  An empty field is a legitimate
     choice for a STATIC platform, so it is reported as a warning that must be
     acknowledged explicitly with ``--allow-no-time-field``.
  3. IMU noise parameters (na/ng/nba/nbg) must be present and positive.
  4. ``imu_acc_scale`` must be stated explicitly; leaving it implicit is how a
     g-valued IMU silently becomes a 10x error.
  5. Gait-filter keys must be internally consistent: enabling the filter without
     frequencies is a no-op, frequencies above Nyquist are dropped, and a
     gyro/accel mask mismatch introduces a relative time skew.
  6. Every key must be one the node actually reads.  The node is a FLAT-schema
     loader, so an upstream-style nested ``mapping:`` block is silently ignored -
     the single most likely way to deploy a config that does nothing.

USAGE
-----
    python3 tools/preflight_config.py --config src/sensing/fastlio2/config/lio_orin_nx.yaml
    python3 tools/preflight_config.py --config my.yaml --strict      # warnings fail
    python3 tools/preflight_config.py --config my.yaml --allow-no-time-field

EXIT CODES
----------
0 = ready (possibly with acknowledged warnings)
2 = blocking problems found
1 = the file could not be read

See docs/hardware_deployment.md.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    print("error: PyYAML is required (pip install pyyaml)", file=sys.stderr)
    sys.exit(1)


# Keys lio_node.cpp::loadParameters() actually consumes.  Kept explicit so an
# unknown key is reported rather than silently ignored: a typo or an upstream
# group name is the difference between "configured" and "did nothing".
KNOWN_KEYS = {
    # interfaces
    "imu_topic", "lidar_topic", "image_topic", "body_frame", "world_frame",
    "print_time_cost",
    # pointcloud2 adaptation
    "pcl2_time_field", "pcl2_time_scale", "pcl2_filter_phase",
    # front end
    "lidar_filter_num", "lidar_min_range", "lidar_max_range", "scan_resolution",
    "map_resolution", "cube_len", "det_range", "move_thresh",
    "na", "ng", "nba", "nbg",
    # imu init
    "imu_init_num", "imu_init_window_s", "imu_init_static_gyro_std",
    "imu_init_static_acc_dev", "imu_init_max_wait_s",
    "imu_acc_normalize", "imu_init_mode", "imu_init_min_samples",
    # estimator
    "near_search_num", "ieskf_max_iter", "gravity_align", "esti_il",
    "point_quality_thresh", "lidar_cov_inv",
    "max_bias_gyro", "max_bias_accel", "max_velocity",
    # sensor adaptation
    "lidar_type", "lidar_max_line", "imu_acc_scale",
    # extrinsics
    "ext_il", "ext_lc",
    # camera
    "cam_width", "cam_height", "cam_fx", "cam_fy", "cam_cx", "cam_cy", "cam_d",
    # gait compensation
    "gait_filter_enable", "gait_notch_freq_hz", "gait_notch_q",
    "gait_filter_sample_rate_hz", "gait_filter_gyro_x", "gait_filter_gyro_y",
    "gait_filter_gyro_z", "gait_filter_accel_x", "gait_filter_accel_y",
    "gait_filter_accel_z",
}

# Upstream FAST-LIO group names.  The node does NOT read these; if one appears,
# the operator copied a config written for a different loader.
UPSTREAM_GROUPS = {"common", "mapping", "preprocess", "publish", "filter_size_surf"}


class Report:
    def __init__(self) -> None:
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.notes: List[str] = []

    def error(self, msg: str) -> None:
        self.errors.append(msg)

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)

    def note(self, msg: str) -> None:
        self.notes.append(msg)


def _num(cfg: Dict[str, Any], key: str, rep: Report, *, positive: bool = False,
         allow_missing: bool = False) -> Optional[float]:
    if key not in cfg:
        if not allow_missing:
            rep.error(f"{key}: missing (the node falls back to a code default that was "
                      "NOT measured on this platform)")
        return None
    v = cfg[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        rep.error(f"{key}: expected a number, got {type(v).__name__} ({v!r})")
        return None
    f = float(v)
    if not math.isfinite(f):
        rep.error(f"{key}: not finite ({v!r})")
        return None
    if positive and f <= 0.0:
        rep.error(f"{key}: must be > 0, got {f:g}")
        return None
    return f


def check_schema(cfg: Dict[str, Any], rep: Report) -> None:
    for k in cfg:
        if k in KNOWN_KEYS:
            continue
        if k in UPSTREAM_GROUPS:
            rep.error(
                f"{k}: this looks like an upstream FAST-LIO GROUP name. lio_node reads "
                "FLAT top-level keys only, so this whole block is silently ignored. "
                "Flatten it (see the mapping table in the profile header)."
            )
        else:
            rep.warn(f"{k}: not a key lio_node reads - it will be ignored. Typo?")


def check_extrinsic(cfg: Dict[str, Any], rep: Report) -> None:
    if "ext_il" not in cfg:
        rep.error(
            "ext_il: missing. loadParameters() logs 'Missing or invalid ext_il parameter, "
            "using defaults' and the estimator runs with r_il = I, t_il = 0. That is a "
            "placeholder, not a measured mounting transform. Measure it (docs/"
            "calibration_procedure.md) and paste the 7 values."
        )
        return
    v = cfg["ext_il"]
    if not isinstance(v, (list, tuple)) or len(v) != 7:
        rep.error(f"ext_il: expected a 7-element sequence [x y z qx qy qz qw], got {v!r}")
        return
    try:
        vals = [float(x) for x in v]
    except (TypeError, ValueError):
        rep.error(f"ext_il: non-numeric element in {v!r}")
        return

    t = vals[:3]
    q = vals[3:]
    qn = math.sqrt(sum(x * x for x in q))
    if qn <= 1e-9:
        rep.error("ext_il: quaternion is all zeros")
    elif abs(qn - 1.0) > 1e-3:
        # Not fatal: the loader normalises.  But it means the number pasted in is
        # not the number the calibration produced, which is worth flagging.
        rep.warn(f"ext_il: quaternion norm is {qn:.6f}, not 1.0 - the loader normalises "
                 "it, so verify the values were copied correctly")
    if all(abs(x) < 1e-12 for x in t):
        rep.warn("ext_il: translation is exactly zero. That is possible but unusual; "
                 "confirm it was measured rather than left as a placeholder")
    rep.note(f"ext_il: t=[{t[0]:+.4f} {t[1]:+.4f} {t[2]:+.4f}] m, |q|={qn:.6f}")


def check_time_field(cfg: Dict[str, Any], rep: Report, allow_none: bool) -> None:
    field = cfg.get("pcl2_time_field", None)
    scale = cfg.get("pcl2_time_scale", None)

    if field is None:
        rep.error("pcl2_time_field: missing. Without it every point gets curvature = 0 and "
                  "there is NO in-scan motion compensation.")
        return
    if not isinstance(field, str):
        rep.error(f"pcl2_time_field: expected a string, got {field!r}")
        return
    if field.strip() == "":
        msg = ("pcl2_time_field: empty. This disables in-scan motion compensation "
               "(curvature = 0 for every point), so a MOVING platform will show layered/"
               "serrated clouds. Run tools/rslidar_pcl2_probe.py to identify the field.")
        if allow_none:
            rep.warn(msg + " [acknowledged with --allow-no-time-field]")
        else:
            rep.error(msg + " Pass --allow-no-time-field to accept this deliberately.")
        return

    if scale is None:
        rep.error("pcl2_time_scale: missing while pcl2_time_field is set")
        return
    s = _num(cfg, "pcl2_time_scale", rep, positive=True)
    if s is not None:
        rep.note(f"pcl2_time_field: '{field}' with scale {s:g} "
                 f"({'seconds' if s == 1.0 else 'converted to seconds'})")
        if s > 1.0:
            rep.warn(f"pcl2_time_scale: {s:g} > 1. A scale that enlarges the offset is "
                     "unusual (the field is normally in s/ms/us/ns); verify the units")


def check_imu(cfg: Dict[str, Any], rep: Report) -> None:
    na = _num(cfg, "na", rep, positive=True)
    ng = _num(cfg, "ng", rep, positive=True)
    nba = _num(cfg, "nba", rep, positive=True)
    nbg = _num(cfg, "nbg", rep, positive=True)

    if "imu_acc_scale" not in cfg:
        rep.error(
            "imu_acc_scale: missing. The code default is 10.0 (a MID360 built-in IMU that "
            "reports ~g). If this platform's IMU reports m/s^2, the default scales every "
            "acceleration by 10x. State the value explicitly."
        )
    else:
        s = _num(cfg, "imu_acc_scale", rep, positive=True)
        if s is not None:
            rep.note(f"imu_acc_scale: {s:g} (verify: static mean |a| should be ~9.81 m/s^2)")

    # Cross-checks against the values the profile shipped with, which were copied
    # from a different sensor pair.  Equality is not an error (the target IMU may
    # genuinely match), but it must not pass unnoticed.
    defaults = {"na": 0.1, "ng": 0.01, "nba": 0.0001, "nbg": 0.0001}
    same = [k for k, d in defaults.items()
            if isinstance(cfg.get(k), (int, float)) and abs(float(cfg[k]) - d) < 1e-12]
    if len(same) == 4:
        rep.warn("na/ng/nba/nbg are all still at the profile's placeholder values, which "
                 "were copied from a MID360+VN200 pair. Run tools/imu_allan_variance.py on "
                 "the target IMU and replace them.")

    # Internal consistency: a random walk density far above the white-noise floor
    # makes the propagation covariance dominated by the walk term.
    if na is not None and nba is not None and nba > na:
        rep.warn(f"nba ({nba:g}) > na ({na:g}); check the units (nba is per sqrt(s), "
                 "not per sqrt(Hz))")
    if ng is not None and nbg is not None and nbg > ng:
        rep.warn(f"nbg ({nbg:g}) > ng ({ng:g}); check the units")


def check_gait(cfg: Dict[str, Any], rep: Report) -> None:
    enabled = cfg.get("gait_filter_enable", False)
    if not isinstance(enabled, bool):
        rep.error(f"gait_filter_enable: expected a bool, got {enabled!r}")
        return

    freqs = cfg.get("gait_notch_freq_hz", [])
    if freqs is None:
        freqs = []
    if not isinstance(freqs, (list, tuple)):
        rep.error(f"gait_notch_freq_hz: expected a list, got {freqs!r}")
        return
    try:
        freqs = [float(f) for f in freqs]
    except (TypeError, ValueError):
        rep.error(f"gait_notch_freq_hz: non-numeric element in {freqs!r}")
        return

    if not enabled:
        if freqs:
            rep.warn("gait_notch_freq_hz is set but gait_filter_enable is false, so the "
                     "filter will NOT run")
        return

    if not freqs:
        rep.error(
            "gait_filter_enable is true but gait_notch_freq_hz is empty: no notch section "
            "can be built and the compensation will NOT run. Derive the centres from a "
            "walking recording with tools/imu_gait_spectrum.py."
        )
        return

    fs = _num(cfg, "gait_filter_sample_rate_hz", rep, positive=True)
    if fs is None:
        return

    nyq = 0.5 * fs
    bad = [f for f in freqs if not (0.0 < f < nyq)]
    if bad:
        rep.error(f"gait_notch_freq_hz: {bad} outside (0, Nyquist={nyq:g}) Hz at "
                  f"fs={fs:g} Hz. These sections are dropped at runtime, so the filter "
                  "will not do what this file says.")
    good = [f for f in freqs if 0.0 < f < nyq]
    if not good:
        return

    q = _num(cfg, "gait_notch_q", rep, positive=True)
    if q is not None:
        widths = [f / q for f in good]
        for i in range(len(good)):
            for j in range(i + 1, len(good)):
                if abs(good[j] - good[i]) < 0.5 * (widths[i] + widths[j]):
                    rep.warn(f"gait_notch_freq_hz: {good[i]:g} and {good[j]:g} Hz overlap "
                             f"at q={q:g} (bandwidth ~{widths[i]:.2f} Hz); the cascade "
                             "removes a wider band than intended")
        rep.note(f"gait filter: {len(good)} section(s) at q={q:g}, fs={fs:g} Hz")

    gyro_any = any(bool(cfg.get(k, d)) for k, d in
                   (("gait_filter_gyro_x", True), ("gait_filter_gyro_y", True),
                    ("gait_filter_gyro_z", False)))
    accel_any = any(bool(cfg.get(k, False)) for k in
                    ("gait_filter_accel_x", "gait_filter_accel_y", "gait_filter_accel_z"))
    if gyro_any != accel_any:
        rep.warn("gait filter: gyro and accel are filtered differently, so they acquire a "
                 "relative time skew equal to the cascade's group delay. Prefer filtering "
                 "both triples with the same coefficients (the skew cancels).")

    rep.warn("gait filter is ENABLED. Its benefit is unmeasured on this repository's data: "
             "confirm the map/odometry did not get worse after enabling it "
             "(docs/tuning_guide.md).")


def check_misc(cfg: Dict[str, Any], rep: Report) -> None:
    lt = cfg.get("lidar_type", None)
    if lt is not None and lt not in ("livox", "pointcloud2"):
        rep.error(f"lidar_type: must be 'livox' or 'pointcloud2', got {lt!r}")
    elif lt == "pointcloud2":
        rep.note("lidar_type: pointcloud2 (the Airy/Odin1 path)")

    if lt == "livox":
        rep.warn("lidar_type is 'livox' (CustomMsg). Airy/Odin1 are not Livox products, so "
                 "this is almost certainly wrong for this platform")

    for key, lo, hi in (("scan_resolution", 0.0, None), ("map_resolution", 0.0, None),
                        ("point_quality_thresh", None, None), ("lidar_cov_inv", 0.0, None),
                        ("cube_len", 0.0, None), ("det_range", 0.0, None)):
        if key in cfg and isinstance(cfg[key], (int, float)) and not isinstance(cfg[key], bool):
            v = float(cfg[key])
            if lo is not None and v <= lo:
                rep.error(f"{key}: must be > {lo:g}, got {v:g}")
            if key == "point_quality_thresh" and not (0.0 <= v <= 1.0):
                rep.warn(f"point_quality_thresh: {v:g} is outside [0, 1]. It is the "
                         "dimensionless score s, not a distance; upstream uses 0.9")

    # A scan resolution coarser than the accuracy target makes the target
    # unreachable regardless of everything else.
    sr = cfg.get("scan_resolution")
    if isinstance(sr, (int, float)) and not isinstance(sr, bool) and float(sr) >= 0.05:
        rep.note(f"scan_resolution: {float(sr):g} m quantises map points to that grid, "
                 "which by itself precludes a sub-decimetre accuracy claim "
                 "(see the lio_highres.yaml header)")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("WHY THIS EXISTS")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, help="deployment YAML to check")
    ap.add_argument("--strict", action="store_true", help="treat warnings as failures")
    ap.add_argument("--allow-no-time-field", action="store_true",
                    help="accept an empty pcl2_time_field (static-platform deployments)")
    ap.add_argument("--quiet-notes", action="store_true", help="hide informational notes")
    args = ap.parse_args(argv)

    if not os.path.exists(args.config):
        print(f"error: config not found: {args.config}", file=sys.stderr)
        return 1
    try:
        with open(args.config, "r") as fh:
            cfg = yaml.safe_load(fh)
    except Exception as exc:
        print(f"error: cannot parse {args.config}: {exc}", file=sys.stderr)
        return 1
    if not isinstance(cfg, dict):
        print(f"error: {args.config} does not contain a YAML mapping", file=sys.stderr)
        return 1

    rep = Report()
    check_schema(cfg, rep)
    check_extrinsic(cfg, rep)
    check_time_field(cfg, rep, args.allow_no_time_field)
    check_imu(cfg, rep)
    check_gait(cfg, rep)
    check_misc(cfg, rep)

    print("=" * 74)
    print(f"T2.1 deployment preflight: {args.config}")
    print("=" * 74)
    if rep.notes and not args.quiet_notes:
        print("--- notes ---")
        for n in rep.notes:
            print(f"  . {n}")
        print()
    if rep.warnings:
        print("--- warnings ---")
        for w in rep.warnings:
            print(f"  ! {w}")
        print()
    if rep.errors:
        print("--- BLOCKING ---")
        for e in rep.errors:
            print(f"  X {e}")
        print()

    print("--- summary ---")
    print(f"  {len(rep.errors)} blocking, {len(rep.warnings)} warning(s)")
    if rep.errors:
        print("  NOT READY: resolve the blocking items above before deploying.")
        return 2
    if rep.warnings and args.strict:
        print("  NOT READY (--strict): warnings are treated as failures.")
        return 2
    if rep.warnings:
        print("  READY with warnings: every warning is a value the operator should have")
        print("  measured. Read them before trusting a metric from this run.")
        return 0
    print("  READY: no blocking items and no warnings.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
