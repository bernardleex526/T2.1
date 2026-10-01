#!/usr/bin/env python3
"""C2.3 on the REAL bags: how does the PointCloud2 decimation phase interact with the vendor's
point ordering?

`Utils::pcl2_to_PCL` keeps the raw indices ``i % lidar_filter_num == phase`` and adopts its
per-point time origin ``t0`` from the first point it RETAINS, so the phase decides both WHICH
points survive and the curvature of every survivor.  Whether that is observable depends on the
vendor's ordering inside the message, which the synthetic generator does not reproduce:

* the synthetic bag is emitted in EMISSION-TIME order and 48 beams share each azimuth, so
  indices 0..47 all carry time 0 - phase 0/1 share t0 and the same azimuth grid, i.e. the arm is
  a null control there (``c2_3_sampling_phase.py``);
* a vendor Mid-360 PointCloud2 is layer-interleaved (``line == i % 4``), consecutive indices are
  ~4.9 us apart and the per-scan timestamps are not monotonic, so the phase selects a different
  set of rings and moves t0.

This tool measures exactly that, read-only, from any PointCloud2 bag+topic: the index-major
structure, the time-order statistics, and the retained set / ring histogram / t0 per phase.
It never changes a run, a metric or a gate.

Usage (use your own ROS domain, one agent per domain):
    ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 diagnostics/c2_3_vendor_ordering.py \\
        --bag /path/to/bags/IndoorOffice1 --topic /mid360/livox/lidar \\
        --time-field timestamp --time-scale 1e-9 --filter-num 2 --filter-num 3
"""

from __future__ import annotations

import argparse
import sys

import numpy as np


def _load_ros():
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:  # pragma: no cover - environment guard
        raise SystemExit("[error] needs a sourced ROS 2 environment (%s)" % exc)
    return rosbag2_py, deserialize_message, PointCloud2


def _field(msg, name):
    """Return (offset, datatype) of a PointField, or (-1, 0) when the field is absent."""
    for f in msg.fields:
        if f.name == name:
            return int(f.offset), int(f.datatype)
    return -1, 0


def _decode(buf, offset, datatype, count=1):
    """Decode one PointField column from a (n_points, point_step) uint8 array."""
    if datatype == 7:
        return buf[:, offset:offset + 4 * count].copy().view("<f4").ravel()
    if datatype == 8:
        return buf[:, offset:offset + 8 * count].copy().view("<f8").ravel()
    if datatype == 2:
        return buf[:, offset:offset + count].copy().view("<u1").ravel()
    if datatype == 4:
        return buf[:, offset:offset + 2 * count].copy().view("<u2").ravel()
    raise SystemExit("[error] unsupported PointField datatype %d" % datatype)


def _index_major_degree(idx, ring):
    """Largest n <= 64 with `ring == idx % n` for every point (0 = no index-major rule)."""
    best = 0
    for n in range(2, 65):
        if bool(np.all(ring == idx % n)):
            best = n
    return best


def describe(scan_id, layer, args):
    idx, t, ring, rng = layer["idx"], layer["t"], layer["ring"], layer["range"]
    dt_us = np.diff(t) * 1e6
    print("  scan %d: n=%d t_span=%.6f s  t[0]=%.9f  dt_us: min=%+.3f max=%+.3f  dt<0: %d/%d" % (
        scan_id, idx.size, float(t[-1] - t[0]), float(t[0]),
        float(dt_us.min()), float(dt_us.max()), int((dt_us < 0).sum()), dt_us.size))
    print("    range_m: [%.3f, %.3f]  below_min=%d  above_max=%d" % (
        float(rng.min()), float(rng.max()), int((rng < args.min_range).sum()),
        int((rng > args.max_range).sum())))
    if ring is not None:
        deg = _index_major_degree(idx, ring)
        print("    ring field: unique=%s  index-major degree=%d%s" % (
            np.unique(ring)[:12].tolist(), deg,
            ("  (ring == i %% %d for every point)" % deg) if deg else "  (no ring == i % n rule)"))
    for f in args.filter_num:
        for phase in range(f):
            keep = np.arange(phase, idx.size, f)
            kept = keep[(rng[keep] >= args.min_range) & (rng[keep] <= args.max_range)]
            ring_hist = ""
            if ring is not None:
                v, c = np.unique(ring[kept], return_counts=True)
                ring_hist = "ring_counts=" + str(dict(zip(v.tolist(), c.tolist())))
            print("    filter_num=%d phase=%d: raw=%d in_range=%d  t0=%.9f s (t0-t[0]=%+.3f us)"
                  "  t_last=%.9f s  %s" % (
                      f, phase, keep.size, kept.size, float(t[keep[0]]),
                      float(t[keep[0]] - t[0]) * 1e6, float(t[keep[-1]]), ring_hist))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", required=True)
    ap.add_argument("--time-field", default="", help="per-point time field (default: auto)")
    ap.add_argument("--time-scale", type=float, default=1.0, help="field -> seconds factor")
    ap.add_argument("--ring-field", default="", help="ring field (default: line|ring|laser_id)")
    ap.add_argument("--filter-num", type=int, action="append", default=[],
                    help="stride(s) to report (default: 2 and 3)")
    ap.add_argument("--min-range", type=float, default=0.5)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--max-scans", type=int, default=2)
    args = ap.parse_args(argv)
    if not args.filter_num:
        args.filter_num = [2, 3]

    rosbag2_py, deserialize_message, PointCloud2 = _load_ros()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))

    time_candidates = [args.time_field] if args.time_field else ["timestamp", "time", "t"]
    ring_candidates = [args.ring_field] if args.ring_field else ["line", "ring", "laser_id"]
    print("bag=%s topic=%s" % (args.bag, args.topic))
    scans = 0
    while reader.has_next() and scans < args.max_scans:
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        n = int(msg.width) * int(msg.height)
        if n == 0:
            continue
        step = int(msg.point_step)
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if raw.size < n * step:
            raise SystemExit("[error] truncated message: %d bytes < %d points x %d" %
                             (raw.size, n, step))
        buf = raw[:n * step].reshape(n, step)

        t = None
        for cand in time_candidates:
            off, dt = _field(msg, cand)
            if off >= 0:
                t = np.asarray(_decode(buf, off, dt), float) * args.time_scale
                used_time = cand
                break
        if t is None:
            raise SystemExit("[error] none of the time fields %s is present" % time_candidates)
        ring = None
        for cand in ring_candidates:
            off, dt = _field(msg, cand)
            if off >= 0:
                ring = _decode(buf, off, dt).astype(np.int64)
                break
        xyz = []
        for name in ("x", "y", "z"):
            off, dt = _field(msg, name)
            if off < 0:
                raise SystemExit("[error] no '%s' field" % name)
            xyz.append(_decode(buf, off, dt).astype(np.float64))

        if scans == 0:
            print("  fields=%s point_step=%d n=%d time_field=%s (scale %.3g) ring_field=%s" % (
                [f.name for f in msg.fields], step, n, used_time, args.time_scale,
                "none" if ring is None else "present"))
        describe(scans, {"idx": np.arange(n), "t": t, "ring": ring,
                         "range": np.sqrt(xyz[0] ** 2 + xyz[1] ** 2 + xyz[2] ** 2)}, args)
        scans += 1
    if scans == 0:
        raise SystemExit("[error] no message on topic %s" % args.topic)
    return 0


if __name__ == "__main__":
    sys.exit(main())
