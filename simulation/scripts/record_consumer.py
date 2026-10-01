#!/usr/bin/env python3
"""Record the output of the REAL localization consumer, not a surrogate.

The consumer is the robot_pose `map_pose_publisher` node.  Which semantics it runs is declared by
the publisher's own `consumer_mode` and MUST be handed to this recorder verbatim via --pose-mode:

  stamp    T_map_body resolved from the newest RECEIVED correction and odometry, published at the
           odometry's own measurement stamp;
  current  tf2 lookupTransform("map", "base_link", TimePointZero) - the chain's latest COMMON time
           - restamped with the node's current clock ("where is the robot now");
  predict  bounded causal prediction on the odometry window, published at a strictly increasing
           QUERY clock: every query emits one status carrying query_seq / query_stamp_ns /
           available / predicted / prediction_dt_s / reject_reason and the gate+chain evidence, and
           at most one PoseStamped whose header stamp EQUALS query_stamp_ns.

This recorder writes the evidence the T3 verdict needs, and nothing is backfilled:

  * every pose the node actually published (pose, the pose's own stamp, the /clock current when it
    ARRIVED, and the recorder's monotonic arrival time);
  * the FULL status stream, one JSON object per line, with the same monotonic arrival clock;
  * the /clock timeline (values, jumps, pauses) and an INDEPENDENT 20 Hz grid over /clock time, so
    a publisher that stops emitting status is detectable instead of silently scoring 0/0;
  * for the declared `stamp`/`current` semantics, the old 50 ms tick census (kept unchanged: the
    frozen T3 main verdict and the message-stamp metric are not being rewritten).

The cross-topic join is done OFFLINE by exact stamp (status query_stamp_ns == pose header stamp),
exactly once per query: /robot_pose_map and /robot_pose_map/status can arrive in either order, a
duplicate message never adds to the numerator, and a pose that arrives more than --deadline-s
(monotonic) after its query does NOT serve that query - a later pose cannot rescue it either.

`join_queries`, `clock_grid`, `gt_support_intervals` and `support_grid` are pure functions (no ROS
imports); the T3 test imports this module and scores the SAME raw events with the SAME code, so the
recorder and the evaluator can never disagree about what a query got.

Usage:
    python3 record_consumer.py --out-dir /tmp/sim_loc --duration 90 --pose-mode predict
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

try:  # the ROS imports are needed ONLY to run the recorder; the helpers below stay pure
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
except ImportError:  # analysis-only import (test_t3 reuses the join/grid functions)
    rclpy = None

    class Node:  # placeholder so the pure helpers below stay importable without ROS
        pass

HAVE_ROS = rclpy is not None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402

TICK_S = 0.05                 # the old 50 ms census tick (unchanged semantics)
DEADLINE_S = 0.05             # a query is served on time only within this monotonic window
GRID_HZ = 20.0                # the independent /clock grid period
NS = 10 ** 9
MODES = ("stamp", "current", "predict")

SEMANTICS = {
    "stamp": "T_map_body resolved from the newest RECEIVED correction and odometry, published at "
             "the odometry's own measurement stamp (default mode)",
    "current": "lookupTransform(map, base_link, TimePointZero) resolved at the chain's latest "
               "COMMON time, published with the node's current clock",
    "predict": "bounded causal prediction over the odometry window, published at a strictly "
               "increasing query clock; one status per query with query_seq/query_stamp_ns/"
               "available/predicted/prediction_dt_s/reject_reason plus gate and chain evidence, "
               "and at most one PoseStamped whose stamp equals query_stamp_ns",
}


# --------------------------------------------------------------------------- pure helpers --
def _text(value):
    return value.decode() if isinstance(value, (bytes, bytearray)) else str(value)


def status_values(values):
    """KeyValue[] (parsed JSON dicts or ROS messages) -> {key: value} of plain strings."""
    out = {}
    for kv in values or []:
        if isinstance(kv, dict):
            out[_text(kv.get("key", ""))] = _text(kv.get("value", ""))
        else:
            out[_text(kv.key)] = _text(kv.value)
    return out


def flag(values, key):
    """Tri-state read of a boolean-valued status field: True / False / None (absent or "n/a")."""
    v = (values or {}).get(key)
    if v is None:
        return None
    v = v.strip().lower()
    if v == "true":
        return True
    if v == "false":
        return False
    return None


def number(values, key):
    """Finite float from a decimal-string status field, else None ("n/a" is never a number)."""
    v = (values or {}).get(key)
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def integer(values, key):
    v = (values or {}).get(key)
    if v is None:
        return None
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return None


def join_queries(statuses, poses, deadline_s=DEADLINE_S):
    """OFFLINE cross-topic join by exact stamp, exactly once per query.

    statuses: [{"wall_rel", "clock_ns", "epoch", "values": {...}}]  poses: [{"wall_rel",
    "stamp_ns", "clock_ns", "epoch", "position", "orientation"}].  A query is served ON TIME only
    when its pose ARRIVED within `deadline_s` monotonic seconds after the query status (a pose that
    arrived earlier counts: the two topics are unordered).

    The join key is (EPOCH, stamp): a /clock that jumps backwards starts a new epoch, and the same
    stamp value recurs in it, so a pose received before the jump can never be paired with a query
    raised in the new epoch - each message is joined at most once, and duplicates are dropped, never
    added, because the numerator counts QUERIES, not messages.
    """
    keys = ("status_messages", "status_with_query_contract", "status_without_query_contract",
            "duplicate_status_dropped", "pose_messages", "duplicate_pose_dropped", "unique_poses",
            "orphan_poses", "queries", "joined", "served_on_time", "late_beyond_deadline",
            "joined_before_status", "unjoined_no_pose", "query_seq_out_of_order",
            "query_seq_missing")
    census = dict.fromkeys(keys, 0)
    pose_by_stamp = {}
    for p in poses:
        census["pose_messages"] += 1
        key = (int(p.get("epoch", 0)), int(p["stamp_ns"]))
        if key in pose_by_stamp:
            census["duplicate_pose_dropped"] += 1
            continue
        pose_by_stamp[key] = p
    census["unique_poses"] = len(pose_by_stamp)

    queries = []
    seen = set()
    query_keys = set()
    last_seq = None
    for s in statuses:
        census["status_messages"] += 1
        v = s.get("values") or {}
        if not all(k in v for k in ("query_seq", "query_stamp_ns", "available")):
            census["status_without_query_contract"] += 1
            continue
        census["status_with_query_contract"] += 1
        seq = integer(v, "query_seq")
        ns = integer(v, "query_stamp_ns")
        if seq is None or ns is None:
            census["query_seq_missing"] += 1
            continue
        epoch = int(s.get("epoch", 0))
        if (epoch, seq, ns) in seen:
            census["duplicate_status_dropped"] += 1
            continue
        seen.add((epoch, seq, ns))
        if last_seq is not None and seq <= last_seq:
            census["query_seq_out_of_order"] += 1
        else:
            last_seq = seq
        query_keys.add((epoch, ns))
        pose = pose_by_stamp.get((epoch, ns))
        status_clock_ns = s.get("clock_ns")
        if status_clock_ns is None and s.get("clock") is not None:
            status_clock_ns = now_ns(s["clock"])
        pose_clock_ns = None
        if pose is not None:
            pose_clock_ns = pose.get("clock_ns")
            if pose_clock_ns is None and pose.get("clock") is not None:
                pose_clock_ns = now_ns(pose["clock"])
        delta = (float(pose["wall_rel"]) - float(s["wall_rel"])) if pose is not None else None
        served = pose is not None and delta <= float(deadline_s)
        if pose is None:
            census["unjoined_no_pose"] += 1
        else:
            census["joined"] += 1
            if delta > float(deadline_s):
                census["late_beyond_deadline"] += 1
            if delta < 0.0:
                census["joined_before_status"] += 1
            if served:
                census["served_on_time"] += 1
        queries.append({
            "query_seq": seq, "query_stamp_ns": ns, "query_clock_ns": ns,
            "query_clock_s": ns * 1e-9, "epoch": epoch,
            "status_wall_rel": float(s["wall_rel"]),
            "status_clock_ns": None if status_clock_ns is None else int(status_clock_ns),
            "status_clock_s": None if status_clock_ns is None else int(status_clock_ns) * 1e-9,
            "available": flag(v, "available"), "predicted": flag(v, "predicted"),
            "prediction_dt_s": number(v, "prediction_dt_s"),
            "reject_reason": v.get("reject_reason", ""),
            "gate_valid": flag(v, "gate_valid"), "gate_ever_valid": flag(v, "gate_ever_valid"),
            "odom_ready": flag(v, "odom_ready"), "tf_ready": flag(v, "tf_ready"),
            "correction_age_s": number(v, "correction_age_s"),
            "odom_age_s": number(v, "odom_age_s"),
            "pose": pose, "pose_wall_rel": float(pose["wall_rel"]) if pose else None,
            "pose_clock_ns": None if pose_clock_ns is None else int(pose_clock_ns),
            "pose_clock_s": None if pose_clock_ns is None else int(pose_clock_ns) * 1e-9,
            "join_delta_s": delta, "joined": pose is not None, "served_on_time": served,
        })
    census["queries"] = len(queries)
    census["orphan_poses"] = sum(1 for k in pose_by_stamp if k not in query_keys)
    census["epochs"] = (max(int(q["epoch"]) for q in queries) + 1) if queries else 0
    census["queries_after_epoch0"] = sum(1 for q in queries if int(q["epoch"]) != 0)
    return {"queries": queries, "census": census, "deadline_s": float(deadline_s)}


def now_ns(seconds):
    """Float seconds -> integer nanoseconds.  ONLY for inputs that were not recorded as integers
    (a GT TUM); everything the recorder hears from ROS is kept in exact integer nanoseconds."""
    return int(round(float(seconds) * NS))


def cell_index_ns(t_ns, origin_ns, period_ns):
    """Half-open cell index of an EXACT nanosecond time in the grid anchored at `origin_ns`.

    Integer arithmetic on purpose: the grid, the /clock and the query stamps are all nanosecond
    integers, so a query that lands exactly ON a cell boundary (the common case when the query
    period equals the grid period) is bucketed deterministically instead of falling into the
    previous cell through a float round-trip at epoch magnitude.
    """
    if origin_ns is None or t_ns is None:
        return None
    return (int(t_ns) - int(origin_ns)) // int(period_ns)


def clock_grid(clock_events, queries, period_s=1.0 / GRID_HZ):
    """Independent 20 Hz grid over /clock time, so publisher silence is visible.

    clock_events: [[wall_rel, clock_ns, epoch]] of the /clock messages (a 2-element row is read as
    epoch 0).  cells[epoch][k] = [epoch, k, first_clock_s, last_clock_s, n_clock_msgs, n_queries,
    n_served, n_unjoined] with k in nanosecond units of the grid anchored at the FIRST clock value.
    A cell where the clock advanced but NO query arrived is the publisher having stopped emitting
    status: it is a missing request, not a silent success.  A clock that jumps backwards starts a
    NEW epoch (its cells are keyed separately, so a re-visited stamp cannot inherit the old cells),
    and a clock frozen while wall time advances is recorded as a pause.
    """
    period_ns = now_ns(period_s)
    origin_ns = int(clock_events[0][1]) if clock_events and clock_events[0][1] is not None else None
    cells = {}
    jumps = []
    pauses = []
    prev = None
    run_start = None
    for row in clock_events:
        wall, c = row[0], row[1]
        epoch = int(row[2]) if len(row) > 2 else 0
        if c is None:
            continue
        c = int(c)
        if prev is not None and c < prev:
            jumps.append([float(wall), prev * 1e-9, c * 1e-9])
        if run_start is not None and c == run_start[1]:
            if float(wall) - run_start[0] > 2.0 * float(period_s):
                pauses.append([run_start[0], float(wall), c * 1e-9])
        else:
            run_start = (float(wall), c)
        prev = c
        k = cell_index_ns(c, origin_ns, period_ns)
        cell = cells.setdefault((epoch, k), {"n_clock": 0, "first": c, "last": c,
                                             "n_queries": 0, "n_served": 0, "n_unjoined": 0})
        cell["n_clock"] += 1
        cell["last"] = c
    for q in queries:
        k = cell_index_ns(q.get("query_clock_ns"), origin_ns, period_ns)
        if k is None:
            continue
        c = int(q["query_clock_ns"])
        key = (int(q.get("epoch", 0)), k)
        cell = cells.setdefault(key, {"n_clock": 0, "first": c, "last": c, "n_queries": 0,
                                      "n_served": 0, "n_unjoined": 0})
        cell["n_queries"] += 1
        if q["served_on_time"]:
            cell["n_served"] += 1
        if not q["joined"]:
            cell["n_unjoined"] += 1
    rows = [[e, k, c["first"] * 1e-9, c["last"] * 1e-9, c["n_clock"], c["n_queries"],
             c["n_served"], c["n_unjoined"]] for (e, k), c in sorted(cells.items())]
    silent = sum(1 for r in rows if r[4] > 0 and r[5] == 0)
    return {"period_s": float(period_s), "period_ns": period_ns, "origin_clock_s": origin_ns * 1e-9
            if origin_ns is not None else None, "origin_clock_ns": origin_ns,
            "clock_messages": len(clock_events), "cells": rows,
            "cells_with_clock_no_query": int(silent),
            "clock_jumps": jumps, "clock_pauses": pauses,
            "note": "cells with clock messages but no query mean the publisher stopped emitting "
                    "status while the clock advanced; they are missing requests, never successes",
            "row_fields": ["epoch", "cell_index", "first_clock_s", "last_clock_s",
                           "clock_messages", "queries", "served_on_time", "unjoined"]}


def gt_support_intervals(gt_t, max_gap_s):
    """GT VALID SUPPORT as integer-ns runs: maximal runs of GT samples whose gaps are <=
    max_gap_s (the evaluator's own no-extrapolation-across-holes rule)."""
    ts = [now_ns(x) for x in gt_t]
    if not ts:
        return []
    gap_ns = now_ns(max_gap_s)
    iv = []
    a = b = ts[0]
    for t in ts[1:]:
        if t - b > gap_ns:
            iv.append([a, b])
            a = t
        b = t
    iv.append([a, b])
    return iv


def support_grid(gt_t, max_gap_s, period_s=1.0 / GRID_HZ):
    """The 20 Hz denominator grid over the GT support, INCLUDING startup.

    Cells are anchored at the first supported GT sample (integer nanoseconds) and only cells whose
    FULL extent lies inside one support interval count; the denominator is never redefined to
    start at the first success, and no cell is shared across a GT hole.
    """
    iv = gt_support_intervals(gt_t, max_gap_s)
    period_ns = now_ns(period_s)
    if not iv:
        return {"period_s": float(period_s), "period_ns": period_ns, "origin_clock_s": None,
                "origin_clock_ns": None, "max_gap_s": float(max_gap_s), "intervals_ns": [],
                "intervals": [], "cells": []}
    origin_ns = iv[0][0]
    cells = []
    for a, b in iv:
        k = -(-(a - origin_ns) // period_ns)          # ceil, exact in integers
        while origin_ns + (k + 1) * period_ns <= b:
            cells.append(k)
            k += 1
    return {"period_s": float(period_s), "period_ns": period_ns,
            "origin_clock_s": origin_ns * 1e-9, "origin_clock_ns": origin_ns,
            "max_gap_s": float(max_gap_s), "intervals_ns": iv,
            "intervals": [[a * 1e-9, b * 1e-9] for a, b in iv], "cells": cells,
            "rule": "half-open %.3f s cells anchored at the first supported GT sample; a cell "
                    "counts only when its FULL extent lies inside one GT support interval "
                    "(gap > %.2f s excluded), so the denominator covers the WHOLE supported span "
                    "including startup and a trailing remainder shorter than one grid period holds "
                    "no full-rate answer" % (period_s, max_gap_s)}


def post_lock_start_ns(queries):
    """First-LOCK EVIDENCE, in arrival order: the /clock nanosecond value at the moment the first
    status that already carries gate_ever_valid && odom_ready && tf_ready ARRIVED.  Never the first
    `available` true.  The post-lock denominator starts at the NEXT cell; the latch never resets."""
    for q in queries:
        if q.get("gate_ever_valid") and q.get("odom_ready") and q.get("tf_ready"):
            return q.get("status_clock_ns")
    return None


# ------------------------------------------------------------------------------- the node --
class ConsumerRecorder(Node):
    def __init__(self, out_dir, pose_topic, status_topic, mode, deadline_s, grid_hz,
                 map_frame="map", body_frame="base_link", odom_frame="odom"):
        super().__init__("sim_consumer_recorder")
        self.out_dir = out_dir
        self.mode = mode
        self.deadline_s = float(deadline_s)
        self.grid_hz = float(grid_hz)
        self.map_frame = map_frame
        self.body_frame = body_frame
        self.odom_frame = odom_frame
        os.makedirs(out_dir, exist_ok=True)
        self.wall_start = time.monotonic()
        self.outputs = []       # legacy: [arrival_wall, stamp_s, x, y, z, qx, qy, qz, qw, clock]
        self.ticks = []         # legacy: [wall, clock, has_output, common_time, common_age]
        self.pose_events = []   # [wall_rel, stamp_ns, clock_ns, x, y, z, qx, qy, qz, qw]
        self.status_events = []  # {"wall_rel", "clock_ns", "level", "message", "values"}
        self.clock_events = []  # [wall_rel, clock_ns]
        self.clock = None       # latest /clock value in seconds (legacy fields/tick census only)
        self.clock_ns = None    # latest /clock value in exact integer nanoseconds
        self.epoch = 0          # incremented on every /clock jump backwards (see on_clock)
        self.map_odom_stamp = None
        self.odom_body_stamp = None
        self.last_output_wall = None
        self.stop = {"signal": None}
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos)
        self.create_subscription(DiagnosticStatus, status_topic, self.on_status, qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        self.create_subscription(Clock, "/clock", self.on_clock,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._write_ready(topic=pose_topic, status_topic=status_topic)

    def _write_ready(self, topic, status_topic):
        sc.write_json(os.path.join(self.out_dir, "consumer_ready.json"), {
            "pid": os.getpid(), "start_monotonic_s": self.wall_start,
            "out_dir": os.path.abspath(self.out_dir), "pose_mode": self.mode,
            "pose_topic": topic, "status_topic": status_topic,
            "deadline_s": self.deadline_s, "grid_hz": self.grid_hz,
            "note": "subscriptions exist and the recorder is spinning; playback may start",
        })
        print("[record_consumer] ready (mode=%s, out=%s)" % (self.mode, self.out_dir), flush=True)

    # -------------------------------------------------------------------------- callbacks --
    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ns = int(msg.header.stamp.sec) * 10 ** 9 + int(msg.header.stamp.nanosec)
        wall = time.monotonic() - self.wall_start
        self.outputs.append([wall, stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w, self.clock])
        self.pose_events.append([wall, ns, self.clock_ns, self.epoch,
                                 p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.last_output_wall = wall

    def on_status(self, msg):
        self.status_events.append({
            "wall_rel": time.monotonic() - self.wall_start, "clock_ns": self.clock_ns,
            "epoch": self.epoch,
            "level": int(msg.level[0] if isinstance(msg.level, (bytes, bytearray))
                         else msg.level), "message": _text(msg.message),
            "values": status_values(msg.values),
        })

    def on_tf(self, msg):
        for tr in msg.transforms:
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if tr.header.frame_id == self.map_frame and tr.child_frame_id == self.odom_frame:
                self.map_odom_stamp = stamp
            elif tr.header.frame_id == self.odom_frame and tr.child_frame_id == self.body_frame:
                self.odom_body_stamp = stamp

    def on_clock(self, msg):
        ns = int(msg.clock.sec) * NS + int(msg.clock.nanosec)
        if self.clock_ns is not None and ns < self.clock_ns:
            # a /clock that goes backwards starts a NEW epoch: stamps repeat in it, so the join
            # keys and the grid cells are separated by epoch and nothing re-uses evidence from the
            # epoch before the jump
            self.epoch += 1
        self.clock_ns = ns
        self.clock = ns * 1e-9
        self.clock_events.append([time.monotonic() - self.wall_start, ns, self.epoch])

    # ---------------------------------------------------------------------- legacy census --
    def common_time(self):
        stamps = [s for s in (self.map_odom_stamp, self.odom_body_stamp) if s is not None]
        return min(stamps) if len(stamps) == 2 else None

    def tick(self):
        """The FROZEN 50 ms census row: unchanged, because the old T3 verdict reads it."""
        wall = time.monotonic() - self.wall_start
        has_output = int(self.last_output_wall is not None
                         and (wall - self.last_output_wall) < 2 * TICK_S)
        common = self.common_time()
        age = (self.clock - common) if (self.clock is not None and common is not None) else None
        self.ticks.append([wall, self.clock, has_output, common, age])

    # -------------------------------------------------------------------------- finalize --
    def finalize(self):
        status_path = os.path.join(self.out_dir, "consumer_status.jsonl")
        with open(status_path, "w") as fh:
            for ev in self.status_events:
                fh.write(json.dumps(ev) + "\n")
        poses = [{"wall_rel": r[0], "stamp_ns": r[1], "clock_ns": r[2], "epoch": r[3],
                  "position": r[4:7], "orientation": r[7:11]} for r in self.pose_events]
        joined = join_queries(self.status_events, poses, deadline_s=self.deadline_s)
        grid = clock_grid(self.clock_events, joined["queries"], period_s=1.0 / self.grid_hz)
        reasons = {}
        for q in joined["queries"]:
            tok = q["reject_reason"]
            reasons[tok] = reasons.get(tok, 0) + 1
        info = {
            "pose_topic": "/robot_pose_map",
            "terminal_status_topic": "/robot_pose_map/status",
            "node": "robot_pose/map_pose_publisher (the real consumer, not a surrogate)",
            "consumer_mode": self.mode,
            "consumer_semantics": SEMANTICS[self.mode],
            "deadline_s": self.deadline_s,
            "grid_period_s": 1.0 / self.grid_hz,
            "tick_period_s": TICK_S,
            "join_rule": "OFFLINE exact-stamp join (status.query_stamp_ns == pose header stamp), "
                         "exactly once per query; pose and status may arrive in either order; "
                         "duplicates never add to the numerator; a pose arriving later than "
                         "deadline_s after its query does not serve it, and a later pose cannot "
                         "rescue it",
            "outputs": self.outputs,
            "ticks": self.ticks,
            "pose_events": self.pose_events,
            "statuses_sample": self.status_events[:5],
            "status_count": len(self.status_events),
            "status_jsonl": status_path,
            "join_census": joined["census"],
            "reject_reason_counts": reasons,
            "clock": {"messages": len(self.clock_events), "first_clock_s": self.clock_events[0][1]
                      * 1e-9 if self.clock_events else None,
                      "last_clock_s": self.clock_events[-1][1] * 1e-9 if self.clock_events else None,
                      "epochs": self.epoch + 1, "jumps": grid["clock_jumps"],
                      "pauses": grid["clock_pauses"],
                      "note": "a /clock that goes backwards starts a new EPOCH; the join keys and "
                              "grid cells are separated by epoch, so a re-visited stamp can never "
                              "be served by evidence from before the jump"},
            "grid": grid,
            "census": {
                "ticks": len(self.ticks),
                "with_output": int(sum(t[2] for t in self.ticks)),
                "without_output": int(sum(1 - t[2] for t in self.ticks)),
                "no_clock": int(sum(1 for t in self.ticks if t[1] is None)),
                "no_common_time": int(sum(1 for t in self.ticks if t[3] is None)),
                "first_output_wall_s": (self.outputs[0][0] if self.outputs else None),
                "first_output_stamp_s": (self.outputs[0][1] if self.outputs else None),
            },
            "note": "a tick without an output is a time at which the real consumer published "
                    "nothing; those times are counted, never dropped",
        }
        path = os.path.join(self.out_dir, "consumer_queries.json")
        sc.write_json(path, info)
        cen = joined["census"]
        print("[record_consumer] mode=%s poses=%d statuses=%d queries=%d joined=%d "
              "served_on_time=%d late=%d unjoined=%d duplicates(status/pose)=%d/%d "
              "silent_grid_cells=%d -> %s"
              % (self.mode, cen["pose_messages"], cen["status_messages"], cen["queries"],
                 cen["joined"], cen["served_on_time"], cen["late_beyond_deadline"],
                 cen["unjoined_no_pose"], cen["duplicate_status_dropped"],
                 cen["duplicate_pose_dropped"], grid["cells_with_clock_no_query"], path))
        return info


def _install_stop_handlers(stop):
    def handler(signum, _frame):
        stop["signal"] = signum
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):  # non-main thread / unsupported: the loop still ends
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--duration", type=float, default=90.0)
    ap.add_argument("--pose-topic", default="/robot_pose_map")
    ap.add_argument("--status-topic", default="/robot_pose_map/status")
    ap.add_argument("--pose-mode", default="stamp", choices=list(MODES),
                    help="the publisher's consumer_mode, declared verbatim")
    ap.add_argument("--deadline-s", type=float, default=DEADLINE_S)
    ap.add_argument("--grid-hz", type=float, default=GRID_HZ)
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--body-frame", default="base_link")
    ap.add_argument("--odom-frame", default="odom")
    args = ap.parse_args(argv)

    if rclpy is None:
        print("[error] rclpy is not importable: source the ROS 2 environment before recording")
        return 2
    rclpy.init()
    node = ConsumerRecorder(args.out_dir, args.pose_topic, args.status_topic, args.pose_mode,
                            args.deadline_s, args.grid_hz, args.map_frame, args.body_frame,
                            args.odom_frame)
    _install_stop_handlers(node.stop)
    end = time.monotonic() + args.duration
    last_tick = 0.0
    try:
        while rclpy.ok() and node.stop["signal"] is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() - last_tick >= TICK_S:
                last_tick = time.monotonic()
                node.tick()
    except BaseException as exc:  # a shutdown mid-spin must still write the census
        import traceback
        print("[record_consumer] interrupted (%s); writing the census\n%s"
              % (type(exc).__name__, traceback.format_exc()))
    finally:
        if node.stop["signal"] is not None:
            print("[record_consumer] stop signal %d; finalizing" % node.stop["signal"])
        node.finalize()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # already shut down by a signal path
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
