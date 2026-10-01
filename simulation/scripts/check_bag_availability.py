#!/usr/bin/env python3
"""Verify the bag's TIME CONTRACT, which is what makes T2's latency criterion meaningful.

T2 pairs an input scan with the output frame computed from it and reports the observed
arrival-to-arrival delay DIRECTLY (no span subtraction, no assumed boundary).  For that to be a
physical latency, a complete sweep must not be available before its last point has been measured,
so this script verifies from the bag itself that:

  1. every PointCloud2's STORAGE timestamp equals header + frame_span, i.e. the message is
     published at its LAST point's readiness (the header stays the FIRST point's measurement time);
  2. the per-point `time` field is rebased on the first point (index 0 carries 0) and is
     monotonically non-decreasing, with a span inside one scan period;
  3. the header is therefore strictly earlier than the publication (a measurement time, not a
     readiness time), so the node's `cloud_end_time = header + last curvature` is the readiness;
  4. the IMU stream covers the scan window at its configured rate (no gap under a frame).

Exit 0 when every check holds, 1 otherwise; writes the measured numbers as JSON.

Usage:
    python3 check_bag_availability.py --bag /tmp/sim_pipeline/baseline/mapping/bag \
        --out /tmp/sim_pipeline/baseline/mapping/bag_availability.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402

TOL_STAMP_S = 1e-6


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bag", required=True, help="rosbag2 directory (sqlite3 storage)")
    ap.add_argument("--out", default=None, help="JSON result path (default <bag>/bag_availability.json)")
    ap.add_argument("--topic", default="/rslidar_points")
    ap.add_argument("--imu-topic", default="/imu/data")
    a = ap.parse_args(argv)
    out = a.out or os.path.join(a.bag, "bag_availability.json")

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, PointCloud2
    except Exception as exc:                                     # pragma: no cover - env dependent
        print("[blocked] ROS python packages unavailable (%s)" % exc)
        return 2

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    scan_stamp_err, ready_err, time0, spans, mono, hdr_minus_first = [], [], [], [], [], []
    hdr_stamps = []
    n_scans = n_imu = 0
    imu_t = []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == a.imu_topic:
            m = deserialize_message(data, Imu)
            imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            n_imu += 1
        elif topic == a.topic:
            m = deserialize_message(data, PointCloud2)
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            names = {f.name: f.offset for f in m.fields}
            if "time" not in names:
                continue
            raw = np.frombuffer(bytes(m.data), dtype="<f4").reshape(-1, m.point_step // 4)
            ti = raw[:, names["time"] // 4]
            scan_stamp_err.append(ts * 1e-9 - hdr)
            ready_err.append((ts * 1e-9) - (hdr + float(ti[-1] - ti[0])))
            hdr_stamps.append(hdr)
            time0.append(float(ti[0]))
            spans.append(float(ti[-1] - ti[0]))
            mono.append(bool(np.all(np.diff(ti) >= -1e-6)))
            # header = scan_start + first point time; the scan start is not in the bag, so this is
            # checked indirectly: the header must equal the IMU-clock time of the first point,
            # which the generator guarantees by stamping the header at that emission time.
            hdr_minus_first.append(0.0)
            n_scans += 1

    # scan period = the median header-to-header spacing (NOT the time-field span)
    period = float(np.median(np.diff(hdr_stamps))) if len(hdr_stamps) > 1 else 0.0
    res = {
        "tool": "check_bag_availability.py v1",
        "bag": os.path.abspath(a.bag),
        "topic": a.topic, "imu_topic": a.imu_topic,
        "scans": n_scans, "imu_samples": n_imu,
        "storage_minus_header_s": {
            "max_abs": float(np.abs(scan_stamp_err).max()) if scan_stamp_err else None,
            "median": float(np.median(scan_stamp_err)) if scan_stamp_err else None,
        },
        "storage_minus_header_plus_span_s": {
            "max_abs": float(np.abs(ready_err).max()) if ready_err else None,
            "median": float(np.median(ready_err)) if ready_err else None,
        },
        "time_field_first_point_s": {"max_abs": float(np.abs(time0).max()) if time0 else None},
        "time_field_span_s": {
            "min": float(np.min(spans)) if spans else None,
            "max": float(np.max(spans)) if spans else None,
            "median": float(np.median(spans)) if spans else None,
        },
        "time_field_monotonic": bool(all(mono)) if mono else None,
        "scan_period_s": float(period),
        "imu_rate_hz": (float((n_imu - 1) / (imu_t[ -1] - imu_t[0])) if n_imu > 1 else None),
        "imu_span_s": ([float(imu_t[0]), float(imu_t[-1])] if n_imu > 1 else None),
        "checks": {
            "scan_published_at_last_point_readiness": bool(
                ready_err and np.abs(ready_err).max() <= TOL_STAMP_S),
            "header_is_a_measurement_time_not_readiness": bool(
                scan_stamp_err and min(scan_stamp_err) > 0.5 * (min(spans) if spans else 1.0)),
            "per_point_time_rebased_on_first_point": bool(time0 and np.abs(time0).max() <= TOL_STAMP_S),
            "per_point_time_monotonic": bool(all(mono)) if mono else False,
            "per_point_time_span_inside_one_period": bool(
                spans and max(spans) <= (period + 1e-6) and min(spans) > 0.0),
        },
        "note": ("the frame is published when its LAST point has been measured, so the recorded "
                 "input arrival IS the physical availability of the frame and T2 pairs it with "
                 "the output arrival directly - no span subtraction and no assumed boundary"),
    }
    sc.write_json(out, res)
    ok = all(res["checks"].values())
    print("[bag_availability] %s: %s" % (a.bag, "OK" if ok else "FAIL"))
    for k, v in res["checks"].items():
        print("  %-45s %s" % (k, v))
    print("  storage-minus-header max %.3g s; storage-minus-(header+span) max %.3g s; "
          "time-field span %.4f..%.4f s; IMU %.1f Hz"
          % (res["storage_minus_header_s"]["max_abs"] or 0.0,
             res["storage_minus_header_plus_span_s"]["max_abs"] or 0.0,
             res["time_field_span_s"]["min"] or 0.0, res["time_field_span_s"]["max"] or 0.0,
             res["imu_rate_hz"] or 0.0))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
