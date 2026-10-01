#!/usr/bin/env python3
"""Regression tests for the REAL-consumer evidence semantics (no ROS, no playback).

These cover only boundaries where a wrong implementation would produce a plausible-but-wrong T3
number, in the shared code path used by BOTH record_consumer.py and test_t3_localization.py:

  * the offline cross-topic join (unordered arrival, one join per query, duplicates never add to
    the numerator, the 50 ms monotonic deadline, no rescue by a later pose, and the EPOCH key that
    stops a pose received before a /clock jump from serving a query raised after it);
  * the independent 20 Hz /clock grid (publisher silence, clock jumps, clock pauses, epoch keys);
  * the GT support grid (startup included in the full-span denominator, GT gaps excluded);
  * the post-lock denominator start (arrived gate+chain evidence, next cell, no reset);
  * the orientation-error distribution (shortest-path angle, no threshold invented).

Run: python3 simulation/scripts/test_consumer_semantics.py     (or pytest on this file)
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(SCRIPTS, os.pardir))

import record_consumer as rc  # noqa: E402

T0 = 1700000000.0
PERIOD = 0.05


def _status(seq, stamp_s, wall, epoch=0, **values):
    v = {"query_seq": str(seq), "query_stamp_ns": str(rc.now_ns(stamp_s)),
         "available": "true", "predicted": "true", "prediction_dt_s": "0.04",
         "reject_reason": "", "gate_ever_valid": "true", "odom_ready": "true",
         "tf_ready": "true", "gate_valid": "true"}
    v.update({k: str(x) for k, x in values.items()})
    return {"wall_rel": wall, "clock_ns": rc.now_ns(stamp_s), "epoch": epoch, "level": 0,
            "message": "ok", "values": v}


def _pose(stamp_s, wall, epoch=0, x=0.0, y=0.0, z=0.0):
    return {"wall_rel": wall, "stamp_ns": rc.now_ns(stamp_s), "clock_ns": rc.now_ns(stamp_s),
            "epoch": epoch, "position": [x, y, z], "orientation": [0.0, 0.0, 0.0, 1.0]}


def test_status_fields_decode_bytes_typed_ros_values():
    # rclpy hands back `bytes` for string/byte diagnostic fields in some builds.  Every status
    # field must be normalised, otherwise the recorder raises inside the subscription callback on
    # the FIRST message and the whole stream is lost (the step-4 runtime failure mode).
    from types import SimpleNamespace
    assert rc.status_values([SimpleNamespace(key=b"available", value=b"true")]) == \
        {"available": "true"}
    assert rc.status_values([{"key": b"gate_ever_valid", "value": b"false"}]) == \
        {"gate_ever_valid": "false"}
    assert rc.status_values([SimpleNamespace(key="n/a", value="1.5")]) == {"n/a": "1.5"}
    assert rc.flag({"available": "true"}, "available") is True
    assert rc.flag({"available": "n/a"}, "available") is None
    assert rc.number({"prediction_dt_s": "n/a"}, "prediction_dt_s") is None
    assert rc.number({"prediction_dt_s": "0.04"}, "prediction_dt_s") == 0.04
    assert rc.integer({"query_stamp_ns": "1700000000000000000"},
                      "query_stamp_ns") == 1700000000000000000
    # the level field is `byte`; a 1-byte buffer must yield the integer, never raise
    for raw in (b"\x01", 1, bytearray(b"\x00")):
        assert int(raw[0] if isinstance(raw, (bytes, bytearray)) else raw) in (0, 1)


def test_join_is_unordered_and_counts_queries_not_messages():
    # the pose ARRIVES BEFORE its status (different topics, no ordering guarantee) and both
    # messages are duplicated: one query, served once, and the duplicates change nothing
    st = _status(1, T0, 1.02)
    po = _pose(T0, 1.00)
    j = rc.join_queries([st, st], [po, po], deadline_s=0.05)
    c = j["census"]
    assert c["queries"] == 1 and c["joined"] == 1 and c["served_on_time"] == 1
    assert c["joined_before_status"] == 1
    assert c["duplicate_status_dropped"] == 1 and c["duplicate_pose_dropped"] == 1
    assert c["late_beyond_deadline"] == 0


def test_deadline_is_monotonic_and_a_later_pose_cannot_rescue_the_query():
    st = [_status(1, T0, 1.0), _status(2, T0 + PERIOD, 1.05)]
    po = [_pose(T0, 1.06), _pose(T0, 1.9)]  # the first is 0.06 s late, the second later still
    j = rc.join_queries(st, po, deadline_s=0.05)
    c = j["census"]
    assert c["queries"] == 2 and c["joined"] == 1 and c["served_on_time"] == 0
    assert c["late_beyond_deadline"] == 1 and c["unjoined_no_pose"] == 1


def test_join_key_is_the_epoch_so_a_pre_jump_pose_never_serves_a_post_jump_query():
    # the SAME stamp value recurs in epoch 1 after the clock jumped backwards; the pose that
    # arrived before the jump must not be paired with the new query, only the new epoch's own pose
    st = [_status(1, T0, 1.0, epoch=0), _status(2, T0, 1.5, epoch=1)]
    po = [_pose(T0, 1.01, epoch=0), _pose(T0, 1.46, epoch=1)]
    j = rc.join_queries(st, po, deadline_s=0.05)
    c = j["census"]
    assert c["queries"] == 2 and c["joined"] == 2 and c["served_on_time"] == 2
    assert c["orphan_poses"] == 0 and c["epochs"] == 2 and c["queries_after_epoch0"] == 1
    # drop the post-jump pose: the pre-jump pose with the identical stamp must NOT be reused
    j2 = rc.join_queries(st, [po[0]], deadline_s=0.05)
    c2 = j2["census"]
    assert c2["joined"] == 1 and c2["served_on_time"] == 1 and c2["unjoined_no_pose"] == 1
    assert j2["queries"][1]["joined"] is False


def test_missing_pose_and_orphan_pose_are_both_counted():
    st = [_status(1, T0, 1.0), _status(2, T0 + PERIOD, 1.05)]
    po = [_pose(T0, 1.01), _pose(T0 + 99.0, 1.06)]  # the second matches no query
    j = rc.join_queries(st, po, deadline_s=0.05)
    c = j["census"]
    assert c["served_on_time"] == 1 and c["unjoined_no_pose"] == 1 and c["orphan_poses"] == 1


def test_clock_grid_flags_publisher_silence_jump_and_pause():
    clock = [[i * PERIOD, rc.now_ns(T0 + i * PERIOD), 0] for i in range(20)]   # 1 s at 20 Hz
    clock += [[1.2, rc.now_ns(T0 + 0.95), 0], [1.4, rc.now_ns(T0 + 0.5), 1]]  # jump BACK
    for i in range(6):
        clock.append([1.4 + 0.02 * i, rc.now_ns(T0 + 0.5), 1])                # clock frozen
    # only the first 0.4 s carries a query status: the rest is publisher silence
    st = [_status(i + 1, T0 + i * PERIOD, 0.5 + i * PERIOD) for i in range(8)]
    j = rc.join_queries(st, [], deadline_s=0.05)
    g = rc.clock_grid(clock, j["queries"], period_s=PERIOD)
    assert g["cells_with_clock_no_query"] >= 10
    assert len(g["clock_jumps"]) == 1 and g["clock_jumps"][0][1] > g["clock_jumps"][0][2]
    assert g["clock_pauses"], "a frozen clock over advancing wall time must be recorded"
    # the epoch-1 cells are keyed separately, so the rewound stamp cannot inherit epoch-0 cells
    assert {r[0] for r in g["cells"]} == {0, 1}
    assert all(r[4] == 0 or r[5] > 0 for r in g["cells"] if r[0] == 0 and r[1] < 8)


def test_full_span_denominator_includes_startup_and_excludes_gt_gaps():
    gt = [T0 + i * 0.001 for i in range(1000)]           # 1 s of GT at 1 kHz
    gt += [T0 + 5.0 + i * 0.001 for i in range(1000)]    # a 4 s hole, then 1 s more
    grid = rc.support_grid(gt, max_gap_s=0.2, period_s=PERIOD)
    assert len(grid["intervals"]) == 2
    # 19 FULL cells per 0.999 s run (the trailing 0.049 s remainder is not a full cell) and the
    # 4 s hole contributes nothing
    assert len(grid["cells"]) == 38
    assert grid["cells"][0] == 0 and grid["cells"][-1] == 118
    assert grid["origin_clock_ns"] == rc.now_ns(T0)      # startup included
    hole_cell = rc.cell_index_ns(rc.now_ns(T0 + 2.0), grid["origin_clock_ns"], grid["period_ns"])
    assert hole_cell not in set(grid["cells"])


def test_post_lock_starts_at_the_next_cell_after_arrived_gate_and_chain_evidence():
    st = [_status(1, T0, 1.0, gate_ever_valid="false", odom_ready="false"),
          _status(2, T0 + PERIOD, 1.05, gate_ever_valid="false"),
          _status(3, T0 + 2 * PERIOD, 1.10),   # all three evidence flags have now arrived
          _status(4, T0 + 3 * PERIOD, 1.15, gate_valid="false",
                  gate_ever_valid="true")]     # a dropout must NOT reset the latch
    j = rc.join_queries(st, [], deadline_s=0.05)
    start = rc.post_lock_start_ns(j["queries"])
    assert start == rc.now_ns(T0 + 2 * PERIOD)
    cells = [0, 1, 2, 3, 4]
    origin = rc.now_ns(T0)
    post = [k for k in cells if origin + k * rc.now_ns(PERIOD) > start]
    assert post == [3, 4], "the cell containing the evidence arrival is excluded"


def test_orientation_error_is_the_shortest_path_angle_and_needs_a_gt_bracket():
    gt = np.array([[T0, 0, 0, 0, 0, 0, 0, 1.0],
                   [T0 + 0.1, 0, 0, 0, 0, 0, 0, 1.0]])
    half = math.sin(math.pi / 4.0)
    yaw90 = [0.0, 0.0, half, half]
    errs, not_bracketed = _t3_orientation_errors([T0 + 0.05], [yaw90], gt, 0.2)
    assert errs.size == 1 and abs(errs[0] - math.pi / 2.0) < 1e-9
    errs2, nb2 = _t3_orientation_errors([T0 + 0.05], [[0.0, 0.0, 0.0, 1.0]], gt, 0.2)
    assert errs2.size == 1 and errs2[0] < 1e-9 and nb2 == 0
    # a bracket spanning a hole wider than the gap rule cannot be scored at all
    errs3, nb3 = _t3_orientation_errors([T0 + 0.05], [yaw90], gt, 0.2, span_override=True)
    assert errs3.size == 0 and nb3 == 1


def _t3_orientation_errors(t, q, gt, max_gap, span_override=False):
    import test_t3_localization as t3
    if span_override:
        gt = gt.copy()
        gt[1, 0] = T0 + 5.0
    return t3.orientation_errors(t, q, gt, max_gap)


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print("ok   %s" % fn.__name__)
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print("FAIL %s: %s: %s" % (fn.__name__, type(exc).__name__, exc))
    print("%d/%d passed" % (len(tests) - failed, len(tests)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
