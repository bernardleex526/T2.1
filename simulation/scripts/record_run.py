#!/usr/bin/env python3
"""Record one FastLIO2 simulation run: accumulated map + estimated trajectory + timing.

Subscribes to the node's outputs and writes the artifacts the metric tests consume:

  /fastlio2/lio_odom     -> <out>/odom.tum            (TUM: t tx ty tz qx qy qz qw)
  /fastlio2/world_cloud  -> <out>/map.pcd             (whole run, binary PCD x y z)
                         -> <out>/map_eval.pcd        (only stamps >= --eval-start-epoch-s)
                         -> <out>/record_run.json     (counts, span, bbox, per-message map)
  /rslidar_points        -> <out>/latency_evidence.json
                         (per-message (header stamp, receive monotonic time) for the INPUT
                          scans and the OUTPUT odom frames, plus the wall-clock span)

Why record world_cloud instead of calling the node's /mapping/save_colored_pcd service:
that service saves ``m_global_colored_cloud``, which is only ever appended to on
*camera-image* frames; with no camera in the simulation it stays empty.

Why the input scan is recorded as well: T2's latency must be a real end-to-end measurement.
The output cadence alone only bounds latency from below (a stack that falls behind still
publishes at its own pace), so the recorder timestamps the bag's input scans and the node's
output frames on one monotonic clock; T2 then pairs an odom frame with the scan it was
computed from (same header stamp - one scan period) and reports the true per-frame latency
and the accumulated backlog.  No node instrumentation is required or claimed.

Time-sliced map: ``map_eval.pcd`` holds only the scans stamped at/after
``--eval-start-epoch-s``.  T1 fits its rigid frame transform on trajectory samples from
BEFORE that instant, so the fit sample set and the evaluated map point set are provably
disjoint in time.

Stops when the publishers go quiet (bag finished) or after --duration.

Usage:
    python3 record_run.py --out-dir /tmp/fastlio2_output --duration 120
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402


MAX_ABS_COORD_M = 1.0e4      # beyond any plausible room: an estimator blow-up, not map geometry


def read_xyz(msg: PointCloud2) -> np.ndarray:
    """Read x/y/z honouring the message's OWN field offsets and point_step.

    The node publishes pcl::PointCloud<pcl::PointXYZINormal> through pcl::toROSMsg, i.e. an
    8-field, 48-byte-per-point layout (x,y,z,normal_x,normal_y,normal_z,intensity,curvature)
    whose fields are NOT packed.  Building a packed 3-float dtype silently misparses that
    layout and injects garbage coordinates into the recorded map, so the offsets and the
    itemsize are taken from the message itself.
    """
    names = [f.name for f in msg.fields]
    formats = {7: "<f4", 8: "<f8"}                     # PointField.FLOAT32 / FLOAT64
    dt = np.dtype({"names": names,
                   "formats": [formats.get(f.datatype, "<f4") for f in msg.fields],
                   "offsets": [f.offset for f in msg.fields],
                   "itemsize": msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)


def _dropped_detail(raw, max_abs=MAX_ABS_COORD_M, n=8):
    """Indices + coordinates of the rejected points, to localise structural corruption."""
    bad = ~np.isfinite(raw).all(axis=1)
    if raw.size:
        bad |= np.abs(raw).max(axis=1) > max_abs
    idx = np.where(bad)[0]
    if idx.size == 0:
        return None
    return {
        "count": int(idx.size),
        "index_min": int(idx.min()),
        "index_max": int(idx.max()),
        "index_contiguous": bool(idx.size == idx.max() - idx.min() + 1),
        "samples": [[int(i)] + [float(x) for x in raw[i]] for i in idx[:n]],
    }


def sanitize(pts, stats, max_abs=MAX_ABS_COORD_M):
    """Drop non-finite and absurdly distant points, counting why."""
    finite = np.isfinite(pts).all(axis=1)
    stats["nonfinite"] += int((~finite).sum())
    pts = pts[finite]
    if pts.size:
        sane = np.abs(pts).max(axis=1) <= max_abs
        stats["out_of_range"] += int((~sane).sum())
        pts = pts[sane]
    return pts


def voxel_unique(pts, voxel):
    if voxel > 0.0 and pts.size:
        keys = np.floor(pts / voxel).astype(np.int64)
        _, keep = np.unique(keys, axis=0, return_index=True)
        pts = pts[np.sort(keep)]
    return pts


class Recorder(Node):
    def __init__(self, out_dir, duration, idle_timeout, voxel, ascii_pcd,
                 ns="/fastlio2", scan_topic="/rslidar_points", eval_start=None):
        super().__init__("sim_recorder")
        self.out_dir = out_dir
        self.deadline = time.monotonic() + duration if duration > 0 else float("inf")
        self.idle_timeout = idle_timeout
        self.voxel = voxel
        self.ascii_pcd = ascii_pcd
        self.eval_start = eval_start
        self.scan_topic_name = scan_topic
        os.makedirs(out_dir, exist_ok=True)
        self.clouds = []
        self.cloud_stamps = []
        self.map_points = 0
        self.sanitize_stats = {"nonfinite": 0, "out_of_range": 0}
        self.per_message = []          # (stamp, kept, dropped) per world_cloud message
        self.dropped_detail = None
        self.tum = []
        self.scan_events = []          # (stamp, recv_monotonic) of the INPUT scans
        self.odom_events = []          # (stamp, recv_monotonic) of the OUTPUT odom frames
        self.wall_start = time.monotonic()
        self.wall_end = None
        self.last_msg = time.monotonic()
        self.saw_cloud = self.saw_odom = False
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, ns + "/world_cloud", self.on_cloud, qos)
        self.create_subscription(Odometry, ns + "/lio_odom", self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, scan_topic, self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self, msg):
        self.scan_events.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                 time.monotonic()))

    def on_cloud(self, msg):
        self.saw_cloud = True
        self.last_msg = time.monotonic()
        raw = read_xyz(msg)
        before = dict(self.sanitize_stats)
        pts = sanitize(raw, self.sanitize_stats)
        dropped = ((self.sanitize_stats["nonfinite"] - before["nonfinite"])
                   + (self.sanitize_stats["out_of_range"] - before["out_of_range"]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.per_message.append((stamp, int(pts.shape[0]), int(dropped)))
        if dropped and self.dropped_detail is None:
            self.dropped_detail = _dropped_detail(raw)
            self.dropped_detail["stamp"] = stamp
        self.map_points += pts.shape[0]
        self.clouds.append(voxel_unique(pts, self.voxel))
        self.cloud_stamps.append(stamp)

    def on_odom(self, msg):
        self.saw_odom = True
        self.last_msg = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tum.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.odom_events.append((t, time.monotonic()))

    def finished(self):
        if time.monotonic() > self.deadline:
            return True
        return (self.saw_cloud or self.saw_odom) and (time.monotonic() - self.last_msg) > self.idle_timeout

    # ------------------------------------------------------------------ output --
    def merge_map(self, min_stamp=None):
        sel = [c for c, s in zip(self.clouds, self.cloud_stamps)
               if min_stamp is None or s >= min_stamp]
        if not sel:
            return np.zeros((0, 3), np.float32)
        return voxel_unique(np.concatenate(sel, axis=0), self.voxel)

    def write_pcd(self, path, pts):
        sc.write_pcd(path, pts, ascii_pcd=self.ascii_pcd)

    def write_tum(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as fh:
            for row in self.tum:
                fh.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % row)

    def finalize(self, report_path=None):
        self.wall_end = time.monotonic()
        pts = self.merge_map()
        pts_eval = self.merge_map(self.eval_start) if self.eval_start is not None else pts
        self.write_pcd(os.path.join(self.out_dir, "map.pcd"), pts)
        eval_path = None
        if self.eval_start is not None:
            eval_path = os.path.join(self.out_dir, "map_eval.pcd")
            self.write_pcd(eval_path, pts_eval)
        self.write_tum(os.path.join(self.out_dir, "odom.tum"))

        n_eval_msgs = sum(1 for s in self.cloud_stamps
                          if self.eval_start is None or s >= self.eval_start)
        info = {
            "cloud_messages": len(self.clouds),
            "points_received": int(self.map_points),
            "map_points": int(pts.shape[0]),
            "points_nonfinite_dropped": self.sanitize_stats["nonfinite"],
            "points_out_of_range_dropped": self.sanitize_stats["out_of_range"],
            "max_abs_coord_m": MAX_ABS_COORD_M,
            "map_voxel_m": self.voxel,
            "odom_messages": len(self.tum),
            "odom_span_s": [self.tum[0][0], self.tum[-1][0]] if self.tum else None,
            "map_bbox": ([float(x) for x in pts.min(axis=0)], [float(x) for x in pts.max(axis=0)])
                        if pts.size else None,
            "map_pcd": os.path.join(self.out_dir, "map.pcd"),
            "map_pcd_sha256": sc.sha256_file(os.path.join(self.out_dir, "map.pcd")),
            "odom_tum": os.path.join(self.out_dir, "odom.tum"),
            "odom_tum_sha256": sc.sha256_file(os.path.join(self.out_dir, "odom.tum")),
            "eval_map": {"path": eval_path, "min_stamp_epoch_s": self.eval_start,
                         "messages": int(n_eval_msgs),
                         "points": int(pts_eval.shape[0]),
                         "sha256": sc.sha256_file(eval_path) if eval_path else None,
                         "rule": "only world_cloud messages stamped at/after min_stamp_epoch_s; "
                                 "each message is ONE scan in the world frame, so this subset is "
                                 "disjoint in time from any trajectory fitted before that stamp"},
            "messages_with_dropped_points": int(sum(1 for _, _, d in self.per_message if d)),
            "dropped_point_fraction": (float(
                (self.sanitize_stats["nonfinite"] + self.sanitize_stats["out_of_range"])
                / max(1, self.map_points + self.sanitize_stats["nonfinite"]
                      + self.sanitize_stats["out_of_range"]))),
            "per_message": [{"stamp": t, "kept": k, "dropped": d}
                            for t, k, d in self.per_message if d],
            "first_dropped_detail": self.dropped_detail,
            "wall_clock": {"start_monotonic_s": self.wall_start, "end_monotonic_s": self.wall_end,
                           "duration_s": self.wall_end - self.wall_start},
        }
        sc.write_json(report_path or os.path.join(self.out_dir, "record_run.json"), info)

        evidence = {
            "clock": "time.monotonic() in the recorder process; stamps are message header stamps",
            "scan_topic": self.scan_topic_name,
            "scan_events": [[float(t), float(r)] for t, r in self.scan_events],
            "odom_events": [[float(t), float(r)] for t, r in self.odom_events],
            "wall_clock": info["wall_clock"],
            "note": "scan_events are the INPUT scans the bag published (what the node also "
                    "received); odom_events are the node's output frames. Pairing an odom frame "
                    "with the scan whose header stamp is one scan period earlier gives the true "
                    "end-to-end per-frame latency.",
        }
        sc.write_json(os.path.join(self.out_dir, "latency_evidence.json"), evidence)
        info["latency_evidence"] = os.path.join(self.out_dir, "latency_evidence.json")
        info["latency_evidence_sha256"] = sc.sha256_file(
            os.path.join(self.out_dir, "latency_evidence.json"))

        print("[record_run] cloud_msgs=%d map_points=%d eval_points=%d odom_msgs=%d span=%s"
              % (info["cloud_messages"], info["map_points"], info["eval_map"]["points"],
                 info["odom_messages"],
                 "%.1f..%.1f" % tuple(info["odom_span_s"]) if info["odom_span_s"] else "n/a"))
        print("[record_run] input scans seen=%d, output odom frames=%d, wall %.1fs"
              % (len(self.scan_events), len(self.odom_events), info["wall_clock"]["duration_s"]))
        dropped = self.sanitize_stats["nonfinite"] + self.sanitize_stats["out_of_range"]
        if dropped:
            print("[record_run] WARNING: dropped %d points (%d non-finite, %d beyond %.0f m) -- "
                  "the estimate blew up on those scans; map.pcd holds only in-range geometry"
                  % (dropped, self.sanitize_stats["nonfinite"], self.sanitize_stats["out_of_range"],
                     MAX_ABS_COORD_M), file=sys.stderr)
        if not self.saw_cloud and not self.saw_odom:
            print("[record_run] WARNING: nothing was received -- is the node running and did "
                  "initialization succeed? (world_cloud/lio_odom stay silent until then)",
                  file=sys.stderr)
        return info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", default="/tmp/fastlio2_output")
    ap.add_argument("--duration", type=float, default=0.0, help="hard stop after N s (0 = until idle)")
    ap.add_argument("--idle-timeout", type=float, default=8.0, help="stop after N s of silence")
    ap.add_argument("--voxel", type=float, default=0.05, help="map voxel size in m (0 = keep all)")
    ap.add_argument("--ascii", action="store_true", help="write the map as ASCII PCD")
    ap.add_argument("--namespace", default="/fastlio2")
    ap.add_argument("--scan-topic", default="/rslidar_points",
                    help="the bag's INPUT LiDAR topic, timestamped for the latency evidence")
    ap.add_argument("--eval-start-epoch-s", type=float, default=None,
                    help="write map_eval.pcd from world_cloud messages stamped at/after this "
                         "epoch; the T1 frame transform is fitted strictly before it")
    args = ap.parse_args(argv)

    rclpy.init()
    node = Recorder(args.out_dir, args.duration, args.idle_timeout, args.voxel, args.ascii,
                    args.namespace, args.scan_topic, args.eval_start_epoch_s)
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.finalize()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
