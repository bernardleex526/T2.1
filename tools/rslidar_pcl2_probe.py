#!/usr/bin/env python3
"""Resolve ``pcl2_time_field`` / ``pcl2_time_scale`` for the target LiDAR.

WHY THIS EXISTS
---------------
``lio_orin_nx.yaml`` ships ``pcl2_time_field: ""``, which makes
``Utils::pcl2_to_PCL`` set ``curvature = 0`` for every point: no in-scan motion
compensation at all, and a moving quadruped gets layered/serrated clouds.  The
field cannot be guessed.  This probe reads ONE PointCloud2 message, reports the
real field layout, and decides the two values from measurements:

  * which field carries per-point time (by name, confirmed by datatype);
  * what its values span (must be about one scan period, not an epoch);
  * what scale converts it to seconds.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
It does not assume the field is named "time".  RoboSense's own ROS 2 SDK
(``rslidar_sdk``) only emits a per-point time field at all when the SDK is built
with ``POINT_TYPE=XYZIRT`` (its default is ``XYZI``, i.e. NO time field); when
present it is named ``timestamp`` and typed ``FLOAT64``.  A different vendor,
firmware or build gives a different answer, so the answer must be measured.

USAGE
-----
    # Against a live topic (needs ROS 2 + a running driver)
    python3 tools/rslidar_pcl2_probe.py --topic /rslidar_points --once

    # Against a recorded bag (offline, needs rosbag2_py)
    python3 tools/rslidar_pcl2_probe.py --bag walk.db3 --topic /rslidar_points

    # Against a captured layout fixture (.json), for CI and for sharing a layout
    # with someone who has no ROS install
    python3 tools/rslidar_pcl2_probe.py --fixture layout.json

    # Also emit the YAML block to paste into the deployment profile
    python3 tools/rslidar_pcl2_probe.py --bag walk.db3 --topic /rslidar_points --emit-yaml

``--once`` is accepted and is the default behaviour: the probe analyses exactly one
message and exits.  The flag exists because it is the conventional ROS idiom
(``ros2 topic echo --once``) and every document in this repository shows it; it is
kept as an explicit, accepted option rather than removed, so that the documented
commands work verbatim.

EXIT CODES
----------
0 = a usable time field was identified and verified
2 = the message was read but NO usable per-point time field exists
    (this is a real, actionable answer: the vendor SDK must be rebuilt with the
     per-point-time point type, or the profile must stay on the no-compensation
     path knowingly)
1 = the input could not be read

See docs/calibration_procedure.md and docs/hardware_deployment.md.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0].rsplit("\\", 1)[0])

# PointField datatype codes from sensor_msgs/msg/PointField.msg
DATATYPE_NAMES = {
    1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
    5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64",
}
DATATYPE_NBYTES = {
    1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8,
}
STRUCT_FMT = {
    1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d",
}

# Names that vendors actually use for a per-point sweep time.  This is a
# HINT list used to rank candidates, never a decision rule: the decision is
# made from the measured value span below.
TIME_FIELD_HINTS = (
    "timestamp", "time", "t", "offset_time", "time_offset",
    "point_time", "st", "sweep_time", "relative_time",
)
# Names that look like time but carry a per-point intensity/echo metric.
TIME_FIELD_ANTI_HINTS = ("ring", "line", "intensity", "reflectivity", "feature", "tag")


@dataclass
class Field:
    name: str
    offset: int
    datatype: int
    count: int = 1

    @property
    def type_name(self) -> str:
        return DATATYPE_NAMES.get(self.datatype, f"UNKNOWN({self.datatype})")


@dataclass
class Layout:
    """A PointCloud2 layout plus its decoded columns.

    ``columns`` maps field name -> 1-D float array of that field's first element
    per point.  Decoding lives in ``decode_message`` so the analysis below is a
    pure function of this object and can be unit-tested without ROS.
    """

    fields: List[Field]
    point_step: int
    width: int
    height: int
    columns: Dict[str, np.ndarray]
    header_stamp: Optional[float] = None
    frame_id: str = ""
    source: str = ""

    @property
    def n_points(self) -> int:
        return int(self.width * self.height)

    def field(self, name: str) -> Optional[Field]:
        for f in self.fields:
            if f.name == name:
                return f
        return None


def decode_message(msg: Any, source: str = "") -> Layout:
    """Turn a sensor_msgs/PointCloud2 into a Layout (needs ROS 2 message types)."""
    fields = [Field(name=f.name, offset=int(f.offset), datatype=int(f.datatype),
                    count=int(f.count)) for f in msg.fields]
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    point_step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    if point_step <= 0:
        raise ValueError("point_step is 0: the message has no usable layout")
    usable = min(n, raw.size // point_step)
    mat = raw[: usable * point_step].reshape(usable, point_step)

    columns: Dict[str, np.ndarray] = {}
    for f in fields:
        nbytes = DATATYPE_NBYTES.get(f.datatype)
        if nbytes is None:
            continue
        if f.offset + nbytes > point_step:
            continue
        chunk = mat[:, f.offset:f.offset + nbytes].copy()
        columns[f.name] = chunk.view(np.dtype(STRUCT_FMT[f.datatype])).reshape(-1).astype(np.float64)

    stamp = None
    try:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except AttributeError:
        pass
    return Layout(fields=fields, point_step=point_step, width=int(msg.width),
                  height=int(msg.height), columns=columns, header_stamp=stamp,
                  frame_id=getattr(msg.header, "frame_id", ""), source=source)


def load_fixture(path: str) -> Layout:
    """Load a captured layout from JSON.

    Schema (produced by ``--dump-fixture`` so a robot-side operator can capture a
    layout once and hand it to someone without ROS):

        {
          "point_step": 32, "width": 24000, "height": 1, "frame_id": "rslidar",
          "header_stamp": 1730000000.123,
          "fields": [{"name": "x", "offset": 0, "datatype": 7, "count": 1}, ...],
          "columns": {"x": [ ... ], "timestamp": [ ... ]}
        }
    """
    with open(path, "r") as fh:
        doc = json.load(fh)
    fields = [Field(name=f["name"], offset=int(f["offset"]),
                    datatype=int(f["datatype"]), count=int(f.get("count", 1)))
              for f in doc["fields"]]
    columns = {k: np.asarray(v, dtype=np.float64) for k, v in doc.get("columns", {}).items()}
    return Layout(fields=fields, point_step=int(doc["point_step"]),
                  width=int(doc["width"]), height=int(doc["height"]),
                  columns=columns, header_stamp=doc.get("header_stamp"),
                  frame_id=doc.get("frame_id", ""), source=path)


def dump_fixture(layout: Layout, path: str) -> None:
    doc = {
        "point_step": layout.point_step,
        "width": layout.width,
        "height": layout.height,
        "frame_id": layout.frame_id,
        "header_stamp": layout.header_stamp,
        "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype,
                    "count": f.count} for f in layout.fields],
        # Keep the file bounded: a layout only needs enough points to show the
        # sweep span, and a full scan is ~24k points.
        "columns": {k: np.asarray(v[:20000]).round(9).tolist()
                    for k, v in layout.columns.items()},
    }
    with open(path, "w") as fh:
        json.dump(doc, fh, indent=2)
    print(f"layout fixture written to {path}")


# --------------------------------------------------------------------------
# Analysis
# --------------------------------------------------------------------------

@dataclass
class Candidate:
    name: str
    datatype: str
    is_float: bool
    n: int
    vmin: float
    vmax: float
    span: float
    median_dt: float
    monotonic: bool
    looks_like_epoch: bool
    absolute_magnitude: bool
    score: float
    notes: List[str]


def _is_monotonic(values: np.ndarray) -> bool:
    if values.size < 3:
        return True
    d = np.diff(values)
    # Tolerate tiny negative jitter from float rounding.
    return bool(np.all(d >= -1e-9 * max(1.0, float(np.max(np.abs(values))))))


def analyse_candidates(layout: Layout, scan_period_s: float = 0.1) -> List[Candidate]:
    """Rank every numeric field by how plausibly it is a per-point sweep time.

    ``scan_period_s`` is the expected sweep duration (Airy at 10 Hz -> 0.1 s).
    It is only used to score, never to force a verdict.
    """
    out: List[Candidate] = []
    n_pts = layout.n_points

    for f in layout.fields:
        if f.count != 1:
            continue
        col = layout.columns.get(f.name)
        if col is None or col.size == 0:
            continue
        name_l = f.name.lower()
        notes: List[str] = []
        is_float = f.datatype in (7, 8)

        if name_l in TIME_FIELD_ANTI_HINTS:
            notes.append("name matches a non-time per-point attribute")

        # A time field must be finite and non-constant.
        finite = np.isfinite(col)
        if not finite.any():
            continue
        v = col[finite]
        vmin, vmax = float(v.min()), float(v.max())
        span = vmax - vmin
        d = np.diff(v)
        median_dt = float(np.median(d)) if d.size else 0.0
        mono = _is_monotonic(v)

        # An absolute epoch (seconds since 1970) has a huge MAGNITUDE but still
        # spans one sweep, which is perfectly usable: pcl2_to_PCL rebases to the
        # first retained point.  A clock whose SPAN is far larger than a sweep is
        # the unusable case (it is not a per-point sweep time at all).  These are
        # two different observations and must not be conflated.
        absolute_magnitude = bool(abs(vmin) > 1e6)
        looks_like_epoch = bool(abs(vmin) > 1e6 and span > 10.0 * scan_period_s)
        if looks_like_epoch:
            notes.append(
                f"span {span:.6g} is far larger than one scan ({scan_period_s:g}s): "
                "absolute clock, not a per-point offset"
            )
        elif absolute_magnitude:
            notes.append(
                f"magnitude {abs(vmin):.6g} looks like an absolute clock, but the span "
                f"{span:.6g} is one sweep: usable (pcl2_to_PCL rebases to the first "
                "retained point)"
            )

        score = 0.0
        if name_l in TIME_FIELD_HINTS:
            score += 3.0
            notes.append("name is a known per-point time field name")
        if is_float:
            score += 1.5
        elif f.datatype == 6:  # UINT32
            score += 0.5
            # Ranked, so it is not hidden, but Utils::pcl2_to_PCL cannot read it:
            # it accepts FLOAT32/FLOAT64 only (utils.cpp).  Flagging it here keeps
            # the eventual verdict honest instead of handing the user a config
            # value that silently disables compensation.
            notes.append(
                "UINT32 is NOT readable by Utils::pcl2_to_PCL, which accepts "
                "FLOAT32/FLOAT64 only"
            )
        else:
            score -= 2.0
            notes.append("integer type is unusual for a sweep time")

        if span > 0.0:
            ratio = span / scan_period_s
            if 0.5 <= ratio <= 2.0:
                score += 4.0
                notes.append(f"span {span:.6g} is within 2x one scan period")
            elif 0.05 <= ratio < 0.5 or 2.0 < ratio <= 20.0:
                score += 1.0
                notes.append(f"span {span:.6g} is 0.05-20x one scan period (check units)")
            else:
                score -= 1.0
        else:
            score -= 3.0
            notes.append("constant across the scan: carries no sweep information")

        if mono:
            score += 1.0
        else:
            notes.append("not monotonic in point order (ordering may be column-major)")

        if looks_like_epoch:
            score -= 5.0

        out.append(Candidate(name=f.name, datatype=f.type_name, is_float=is_float,
                             n=int(v.size), vmin=vmin, vmax=vmax, span=span,
                             median_dt=median_dt, monotonic=mono,
                             looks_like_epoch=looks_like_epoch,
                             absolute_magnitude=absolute_magnitude,
                             score=score, notes=notes))

    out.sort(key=lambda c: (-c.score, c.name))
    return out


# Scale candidates in seconds-per-unit, with the human label for each.
SCALE_CHOICES = (
    (1.0, "seconds"),
    (1e-3, "milliseconds"),
    (1e-6, "microseconds"),
    (1e-9, "nanoseconds"),
)


def decide_scale(cand: Candidate, scan_period_s: float = 0.1) -> Optional[tuple]:
    """Pick the (scale, unit) that makes ``cand.span`` land nearest one scan.

    Returns None when no power-of-1000 unit makes the span plausible, which is
    the honest answer for a field that is not a sweep time.
    """
    if cand.span <= 0.0:
        return None
    best = None
    for scale, unit in SCALE_CHOICES:
        seconds = cand.span * scale
        ratio = seconds / scan_period_s
        # Accept anything within 4x of one scan; pick the closest.
        err = abs(np.log(ratio)) if ratio > 0 else float("inf")
        if 0.25 <= ratio <= 4.0 and (best is None or err < best[0]):
            best = (err, scale, unit, seconds)
    if best is None:
        return None
    return best[1], best[2], best[3]


def render_report(layout: Layout, scan_period_s: float, top: int = 8) -> tuple:
    """Print the layout + ranked candidates; return (verdict_ok, chosen, scale)."""
    print("=" * 78)
    print("T2.1 PointCloud2 per-point time probe")
    print("=" * 78)
    print(f"source          : {layout.source}")
    print(f"frame_id        : {layout.frame_id or '(unset)'}")
    print(f"header stamp    : {layout.header_stamp if layout.header_stamp is not None else '(unset)'}")
    print(f"width x height  : {layout.width} x {layout.height}  ({layout.n_points} points)")
    print(f"point_step      : {layout.point_step} bytes")
    print(f"expected scan   : {scan_period_s:g} s  (override with --scan-period)")
    print()

    print("--- field layout (as published) ---")
    print(f"{'name':<16}{'type':<10}{'offset':>7}{'bytes':>7}  note")
    for f in layout.fields:
        nb = DATATYPE_NBYTES.get(f.datatype, 0) * f.count
        note = ""
        if f.name.lower() in TIME_FIELD_HINTS:
            note = "<- per-point time candidate"
        elif f.count != 1:
            note = f"array of {f.count}"
        print(f"{f.name:<16}{f.type_name:<10}{f.offset:>7}{nb:>7}  {note}")
    print()

    cands = analyse_candidates(layout, scan_period_s)
    if not cands:
        print("No numeric single-element fields were decoded: nothing to analyse.")
        print("VERDICT: FAIL - message has no analysable fields.")
        return False, None, None

    print("--- candidates ranked by plausibility as per-point sweep time ---")
    for c in cands[:top]:
        print(f"  {c.name:<16} type={c.datatype:<8} score={c.score:+.1f} "
              f"span={c.span:<14.6g} min={c.vmin:<14.6g} max={c.vmax:<14.6g} "
              f"monotonic={c.monotonic}")
        for note in c.notes:
            print(f"      - {note}")
    print()

    # Verdict: choose the best candidate that is USABLE as a pcl2_to_PCL time
    # field, i.e. named like a time field AND typed FLOAT32/FLOAT64.  A
    # well-named but non-float field (e.g. UINT32 nanoseconds) is a real answer
    # that requires a code change, so it is reported as such rather than being
    # silently converted into a config value that would disable compensation.
    print("--- verdict ---")
    usable = [c for c in cands
              if c.name.lower() in TIME_FIELD_HINTS and c.is_float]
    named_not_float = [c for c in cands
                       if c.name.lower() in TIME_FIELD_HINTS and not c.is_float]

    if not usable:
        if named_not_float:
            bad = named_not_float[0]
            print(f"FAIL: {bad.name!r} is a per-point time field but its type is "
                  f"{bad.datatype}.")
            print("      Utils::pcl2_to_PCL accepts FLOAT32/FLOAT64 only, so no config "
                  "value can make")
            print("      this work. Either rebuild the vendor SDK so the time field is "
                  "FLOAT64, or")
            print("      extend pcl2_to_PCL to read integer time fields.")
            return False, None, None
        best = cands[0]
        print(f"FAIL: no field is recognisable as a per-point time field "
              f"(best candidate: {best.name!r}).")
        print("      Do not configure pcl2_time_field from this field. Either the "
              "vendor SDK was")
        print("      built without per-point time (RoboSense rslidar_sdk defaults to "
              "POINT_TYPE=XYZI,")
        print("      which emits x/y/z/intensity ONLY), or the field is named "
              "something this probe")
        print("      does not know. Re-run with --dump-fixture and inspect the layout "
              "above.")
        return False, None, None

    best = usable[0]
    scale_pick = decide_scale(best, scan_period_s)

    if scale_pick is None:
        print(f"FAIL: {best.name!r} spans {best.span:.6g} units, which no "
              "power-of-1000 unit converts")
        print(f"      to about one scan period ({scan_period_s:g} s). Treating it as "
              "a sweep time would")
        print("      be a guess.")
        return False, None, None

    scale, unit, seconds = scale_pick
    print(f"PASS: per-point time field is {best.name!r} ({best.datatype}), "
          f"values in {unit}")
    print(f"      span {best.span:.6g} {unit} = {seconds:.6g} s "
          f"({seconds / scan_period_s:.2f}x one scan period)")
    print(f"      monotonic in point order: {best.monotonic}")
    if best.looks_like_epoch:
        print("      NOTE: span is far larger than one scan; this is NOT a sweep time.")
    elif best.absolute_magnitude:
        print("      NOTE: magnitude looks like an absolute clock (e.g. epoch seconds),")
        print("            but the span is one scan. That is usable: pcl2_to_PCL rebases")
        print("            each point to the first RETAINED point of the frame, so an")
        print("            absolute epoch works as long as the span is correct.")
    if not best.monotonic:
        print("      NOTE: not monotonic - the driver may publish column-major "
              "(ros_send_by_rows).")
        print("            Verify visually before trusting the compensation.")
    print()
    print("--- paste into the deployment profile ---")
    print(f'pcl2_time_field: "{best.name}"')
    print(f"pcl2_time_scale: {scale:g}   # {unit} -> seconds")
    print()
    print("Then verify (see docs/calibration_procedure.md step 4):")
    print("  1. ros2 bag play <walk.bag> --clock")
    print("  2. watch /fastlio2/body_cloud in RViz for layered/serrated points")
    print("  3. a wrong scale or field makes the cloud visibly WORSE, not better")
    return True, best.name, scale


def read_from_bag(bag: str, topic: str, timeout_s: float = 10.0) -> Layout:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise RuntimeError(f"reading a bag needs ROS 2 python packages: {exc}") from None

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
    except Exception as exc:
        raise RuntimeError(f"cannot open bag {bag}: {exc}") from None

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise RuntimeError(f"topic {topic!r} not in {bag}. Present: {', '.join(sorted(topics))}")
    msg_type = get_message(topics[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        _, raw, _ = reader.read_next()
        return decode_message(deserialize_message(raw, msg_type), source=f"{bag}:{topic}")
    raise RuntimeError(f"no message on {topic!r} in {bag}")


def read_from_topic(topic: str, timeout_s: float = 10.0) -> Layout:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2
    except Exception as exc:
        raise RuntimeError(f"reading a live topic needs ROS 2 python packages: {exc}") from None

    rclpy.init()
    node = Node("t21_pcl2_probe")
    holder: Dict[str, Any] = {}
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, topic, lambda m: holder.setdefault("msg", m), qos)

    import time
    deadline = time.time() + timeout_s
    while rclpy.ok() and "msg" not in holder and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "msg" not in holder:
        raise RuntimeError(f"no PointCloud2 arrived on {topic!r} within {timeout_s:g}s")
    return decode_message(holder["msg"], source=f"topic:{topic}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Determine pcl2_time_field/pcl2_time_scale for the target LiDAR.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("USAGE", 1)[1] if "USAGE" in __doc__ else "")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--topic", help="live PointCloud2 topic, e.g. /rslidar_points")
    src.add_argument("--bag", help="ROS 2 bag path/uri (needs --topic)")
    src.add_argument("--fixture", help="previously captured layout .json")
    ap.add_argument("--scan-period", type=float, default=0.1,
                    help="expected sweep duration in seconds (default 0.1 = 10 Hz)")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="seconds to wait for a live message (default 10)")
    ap.add_argument("--dump-fixture", metavar="PATH",
                    help="write the observed layout to PATH as JSON and continue")
    ap.add_argument("--emit-yaml", action="store_true",
                    help="print only the two YAML lines, for scripting")
    ap.add_argument("--top", type=int, default=8, help="candidates to display")
    # Accepted for compatibility with the conventional ROS idiom
    # (`ros2 topic echo --once`) and with the commands printed throughout this
    # repository's docs.  The probe always analyses exactly one message, so this
    # flag is a no-op that is kept so the documented invocations work verbatim.
    ap.add_argument("--once", action="store_true",
                    help="analyse a single message and exit (default behaviour; accepted "
                         "so the documented commands work verbatim)")
    args = ap.parse_args(argv)

    try:
        if args.fixture:
            layout = load_fixture(args.fixture)
        elif args.bag:
            if not args.topic:
                print("error: --bag requires --topic", file=sys.stderr)
                return 1
            layout = read_from_bag(args.bag, args.topic, args.timeout)
        else:
            layout = read_from_topic(args.topic, args.timeout)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if args.dump_fixture:
        dump_fixture(layout, args.dump_fixture)

    ok, name, scale = render_report(layout, args.scan_period, top=args.top)
    if args.emit_yaml and ok:
        print(f'pcl2_time_field: "{name}"')
        print(f"pcl2_time_scale: {scale:g}")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
