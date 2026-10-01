#!/usr/bin/env python3
"""T3: localization accuracy - real localizer on a frozen map, separate trajectory, ATE <= 5 cm.

Runs the frozen evaluation-contract evaluator (gt_time_assoc_eval.py) on the LOCALIZER's own
pose stream and reads association.ate_against_interpolated_gt_m.ate_rmse.

What makes this a localization measurement and not a re-labelled odometry run:

  * the estimate is the composition T_map_body = T_map_odom (the localizer_node's own TF
    broadcast) * T_odom_body (lio_node), recorded by record_localization.py;
  * the prior map is a FROZEN PCD, hashed in the run manifest, and is never updated;
  * the trajectory is the SEPARATE localization route (test_trajectory_localization.json),
    not the mapping route, and the recorded localization pose is never the mapping run's
    odometry -- record_localization.py stores the localizer's map->odom offset statistics and
    this test refuses to score a stream whose offset is ~identity;
  * the frames already coincide (source frame == target frame == scene), so NO transform is
    supplied and NO post-fit alignment of any kind is applied by the evaluator: the ATE is a
    direct comparison of the localization pose against the interpolated GT.

The initial pose handed to the localizer is a DECLARED start pose (the robot's known
placement) and is recorded verbatim in the run manifest; when it is GT-assisted this test
prints it as such.  It is never hidden and it is never used to post-fit the result.

TWO criteria, both required for PASS, both declared before the run:

  1. ACCURACY - ATE RMSE <= 5 cm over the poses the localizer actually delivered;
  2. CONTINUITY - ONLINE AVAILABILITY >= --min-availability over the operational window
     [first available pose, GT end], under the frozen consumer rule recorded in the run:
     latest map<-odom broadcast with stamp <= query time, zero-order hold, hold_validity_s,
     the held offset not the node's initialised identity, and the localizer's own gate valid
     within validity_stale_s.  Nothing is interpolated and no failure is filled in later.
     Startup is reported separately (first_odom / first_tf / first_lock_adopted /
     first_localization) and the FULL-span availability including startup is reported next to
     the gated number.

The evaluator's +-tolerance sample-union COVERAGE is reported as a sampling-density
diagnostic only: it is structurally capped by the stream's own output rate (a 5 Hz stream
cannot union-cover more than ~0.5 of the timeline at a 0.05 s tolerance), so it is never the
continuity criterion and its tolerance is never widened to change a verdict.

Exit codes: 0 = PASS, 1 = FAIL, 2 = BLOCKED (not measurable / no certified value).

SEPARATELY, and never an input to the frozen verdict above, this test also scores the NEW
predicted-position service the plan adds: in the publisher's `predict` mode the real node answers
every query with one status (query_seq / query_stamp_ns / available / predicted / prediction_dt_s
/ reject_reason / gate and chain evidence) and at most one PoseStamped.  record_consumer.py
records both topics with monotonic arrival clocks; this test joins them OFFLINE by exact stamp,
exactly once per query, and reports `predicted_service`:
current_clock_ate_rmse_m (the published pose vs the GT at the clock when it ARRIVED),
position_accuracy_met (<= --threshold-m), full_span_output_fraction and post_lock_output_fraction
over the pre-declared 20 Hz grid (full span = every cell of the GT support INCLUDING startup under
the GT gap rule; post-lock = the cells after the first arrived gate+chain evidence, dropouts still
counted, never re-started at the first success), post_lock_availability_met
(>= --min-post-lock-availability), the orientation-error distribution (no angular threshold is
invented), the prediction-dt distribution, rejection counts and startup.  No GT or no record ->
BLOCKED; a predict-mode consumer with no usable pose -> FAIL with ATE null.  That block has its own
verdict and never turns the frozen protocol FAIL into a PASS.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402

THRESHOLD_M = 0.05
DEFAULT_EVAL_DIR = os.path.join("artifacts", "04b73f5_evidence_audit_20260929_5V6fKb",
                                "evaluation")
DEFAULT_RUN_DIR = "/tmp/fastlio2_localization"
DEFAULT_REF_DIR = "/tmp/fastlio2_scene_ref"


def repo_root():
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run-dir", default=DEFAULT_RUN_DIR)
    ap.add_argument("--ref-dir", default=DEFAULT_REF_DIR)
    ap.add_argument("--est-tum", default=None, help="default <run-dir>/localization.tum")
    ap.add_argument("--gt-tum", default=None, help="default <ref-dir>/gt_localization.tum")
    ap.add_argument("--record", default=None, help="default <run-dir>/localization_record.json")
    ap.add_argument("--manifest", default=None, help="default <run-dir>/run_manifest.json")
    ap.add_argument("--frame", default="scene")
    ap.add_argument("--entity", default="body")
    ap.add_argument("--max-gap-s", type=float, default=0.2)
    ap.add_argument("--coverage-tolerance-s", type=float, default=0.05,
                    help="PRE-DECLARED time-coverage tolerance: half the LiDAR scan period "
                         "(0.1 s), i.e. how far a localization sample may be from a GT sample "
                         "and still count as explaining that part of the GT timeline")
    ap.add_argument("--min-coverage", type=float, default=0.95,
                    help="PROTOCOL CHOICE (not user-specified): continuous localization is only "
                         "claimed when the localization stream explains at least this fraction of "
                         "the COMPLETE ground-truth support timeline (denominator = the full GT "
                         "span, startup and unavailable time included).  Below it the ATE is "
                         "reported as a SUBSET error and the verdict is BLOCKED, never PASS.")
    ap.add_argument("--coverage-window", default="full_gt_span",
                    choices=["full_gt_span", "common"])
    ap.add_argument("--availability-file", default=None,
                    help="default <run-dir>/tf_availability.json (the localizer's own "
                         "map->odom broadcast timeline, written by record_localization.py)")
    ap.add_argument("--min-availability", type=float, default=0.95,
                    help="PROTOCOL CHOICE: the ONLINE availability gate.  Availability is the "
                         "fraction of the GT timeline at which a causal consumer could obtain a "
                         "map->body pose under the frozen consumer rule (zero-order hold, "
                         "hold_validity_s, and the localizer's own gate valid).  It is measured "
                         "at GT resolution from the broadcast timeline; it is NOT the +-tol "
                         "sample-union coverage, which is structurally capped by the output "
                         "rate and is reported as a diagnostic only.")
    ap.add_argument("--eval-dir", default=None)
    ap.add_argument("--min-post-lock-availability", type=float, default=0.95,
                    help="PRE-DECLARED gate of the NEW predicted-position service: the fraction "
                         "of the post-lock 20 Hz grid cells (first lock evidence + both chains "
                         "ready, to the GT end, dropouts still counted) in which the consumer "
                         "served the query on time.  Distinct from --min-availability, which "
                         "judges the OLD protocol over the full GT span.")
    ap.add_argument("--work-dir", default=None, help="default <run-dir>/t3_work")
    ap.add_argument("--out", default=None, help="default <run-dir>/t3_result.json")
    ap.add_argument("--threshold-m", type=float, default=THRESHOLD_M)
    return ap.parse_args(argv)


ODOM_HOLD_S = 0.25  # a query needs an odometry frame within this long before the GT sample
STAMP_RESOLUTION_S = 1e-9  # the age bound is compared at the stamp resolution (nanoseconds)


def availability_census(avail, gt_t, *, hold_validity_s, validity_stale_s=1.0,
                        odom_hold_s=ODOM_HOLD_S):
    """Apply the frozen receive-time-causal consumer rule at ground-truth resolution.

    For every ground-truth sample time t the query is raised when the odometry frame carrying t
    ARRIVES; only a map<-odom correction and only a validity answer that had already ARRIVED
    may be used for it.  Nothing is interpolated and no later evidence backfills an earlier
    query.  Returns two censuses:

      * the declared AGE CONTRACT (`hold_validity_s` on the correction age, which is
        query_stamp - tf_message_stamp, the stamp of the frame the alignment solved for);
      * the causal-available diagnostic, identical but without the age bound, so a service that
        is alive yet slower than the contract is distinguishable from one that is absent.

    Returns None when the run carries no broadcast/odometry timeline.
    """
    tf = avail.get("tf_timeline") or []
    odom = avail.get("odom_timeline") or []
    valid = avail.get("valid_timeline") or []
    if not tf or not odom or gt_t.size == 0:
        return None
    tf_arr = np.array([r[1] for r in tf], float)
    tf_stamp = np.array([r[0] for r in tf], float)
    od_stamp = np.array([r[0] for r in odom], float)
    od_arr = np.array([r[1] for r in odom], float)
    v_wall = np.array([r[0] for r in valid], float) if valid else np.zeros(0)
    v_val = np.array([bool(r[1]) for r in valid]) if valid else np.zeros(0, bool)

    keys = ("queries", "covered", "no_odom_frame", "no_tf_received_before", "too_old",
            "no_valid_answer", "gate_not_valid", "identity_offset_samples")
    age_census = dict.fromkeys(keys, 0)
    causal_census = dict.fromkeys(keys, 0)
    # odometry frames are in stamp order and their arrivals increase, so both the eligible
    # transform set and the answer cursor only ever move forward.
    eligible_stamps = []      # sorted correction stamps already received at the current anchor
    eligible_meta = {}        # stamp -> timeline row
    j = 0
    v_cursor = -1
    ages_all = []
    age_mask = np.zeros(gt_t.size, bool)
    causal_mask = np.zeros(gt_t.size, bool)
    for i, t in enumerate(gt_t):
        age_census["queries"] += 1
        causal_census["queries"] += 1
        while j < od_stamp.size and od_stamp[j] <= t:
            j += 1
        anchor = j - 1
        if anchor < 0 or (t - od_stamp[anchor]) > odom_hold_s:
            age_census["no_odom_frame"] += 1
            causal_census["no_odom_frame"] += 1
            continue
        anchor_arr = od_arr[anchor]
        k = int(np.searchsorted(tf_arr, anchor_arr, side="right"))
        while len(eligible_stamps) < k:
            row = tf[len(eligible_stamps)]
            pos = bisect.bisect(eligible_stamps, float(row[0]))
            eligible_stamps.insert(pos, float(row[0]))
            eligible_meta[float(row[0])] = row
        while v_cursor + 1 < v_wall.size and v_wall[v_cursor + 1] <= anchor_arr:
            v_cursor += 1
        p = bisect.bisect(eligible_stamps, float(t))
        if p == 0:
            age_census["no_tf_received_before"] += 1
            causal_census["no_tf_received_before"] += 1
            continue
        stamp = eligible_stamps[p - 1]
        age = float(t) - stamp
        ages_all.append(age)
        if v_cursor < 0 or (anchor_arr - v_wall[v_cursor]) > validity_stale_s:
            age_census["no_valid_answer"] += 1
            causal_census["no_valid_answer"] += 1
            continue
        if not v_val[v_cursor]:
            age_census["gate_not_valid"] += 1
            causal_census["gate_not_valid"] += 1
            continue
        causal_census["covered"] += 1
        causal_mask[i] = True
        t_mo = np.asarray(eligible_meta[stamp][2], float)
        q_mo = np.asarray(eligible_meta[stamp][3], float)
        w = min(1.0, max(-1.0, abs(float(q_mo[3]))))
        if float(np.linalg.norm(t_mo)) < 0.01 and 2.0 * math.acos(w) < math.radians(0.5):
            causal_census["identity_offset_samples"] += 1
            age_census["identity_offset_samples"] += 1
        if age <= hold_validity_s + STAMP_RESOLUTION_S:
            age_census["covered"] += 1
            age_mask[i] = True
        else:
            age_census["too_old"] += 1

    ages = np.asarray(ages_all, float)
    out = {
        "age_contract": age_census,
        "causal_available": causal_census,
        "age_s": {"n": int(ages.size),
                  "median": float(np.median(ages)) if ages.size else None,
                  "p95": float(np.percentile(ages, 95)) if ages.size else None,
                  "max": float(ages.max()) if ages.size else None},
        "hold_validity_s": float(hold_validity_s),
        "odom_hold_s": float(odom_hold_s),
        "rule": ("receive-time causal: correction and validity answer must have arrived before "
                 "the odometry frame that raises the query; correction age = query_stamp - "
                 "tf_message_stamp"),
    }
    for name, mask in (("age_contract", age_mask), ("causal_available", causal_mask)):
        cen = out[name]
        q = cen["queries"]
        out[name + "_fraction_full_span"] = (cen["covered"] / q) if q else None
        cov = np.flatnonzero(mask)
        if cov.size:
            first = int(cov[0])
            out[name + "_first_epoch_s"] = float(gt_t[first])
            out[name + "_startup_s"] = float(gt_t[first] - gt_t[0])
            out[name + "_fraction_post_lock"] = float(np.mean(mask[first:]))
        else:
            out[name + "_first_epoch_s"] = None
            out[name + "_startup_s"] = None
            out[name + "_fraction_post_lock"] = 0.0
    return out


# ---------------------------------------------- NEW predicted-position consumer service --
# The frozen verdict above judges the OLD protocol (message-stamp accuracy + the 95%/0.10 s
# availability choice).  This block is the SEPARATE, current-position service the plan adds:
# what the REAL map_pose_publisher published in predict mode, scored against the GT at the /clock
# time when each pose ARRIVED.  It never rewrites the old fields and never feeds the old verdict.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import record_consumer as rc  # the SAME pure join/grid helpers the recorder uses
except ImportError:  # pragma: no cover - the module sits next to this test
    rc = None

SERVICE_GRID_PERIOD_S = 0.05      # the declared 20 Hz grid, identical to the recorder's


def load_gt_tum(path):
    """GT as (N,8): t x y z qx qy qz qw in the scene frame, stamped on the bag clock."""
    rows = []
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if len(parts) >= 8:
                rows.append([float(x) for x in parts[:8]])
    return np.asarray(rows, float) if rows else np.zeros((0, 8))


def quat_slerp(q0, q1, a):
    """Shortest-path slerp, matching the evaluator's own interpolation between GT samples."""
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1 = -q1
        d = -d
    d = min(1.0, max(-1.0, d))
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
    else:
        th = math.acos(d)
        q = (math.sin((1.0 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)
    return q / float(np.linalg.norm(q))


def orientation_errors(est_t, est_q, gt, max_gap_s):
    """Angle between each published orientation and the interpolated GT orientation.

    The bracket rule is the evaluator's own (both GT samples must bracket the estimate and the
    bracket must not span a hole wider than max_gap_s).  The user specified NO orientation
    tolerance, so only the distribution is reported - no threshold is invented.
    """
    t = np.asarray(est_t, float)
    q = np.asarray(est_q, float)
    tg = gt[:, 0]
    gq = gt[:, 4:8]
    idx = np.clip(np.searchsorted(tg, t), 1, len(tg) - 1)
    t0, t1 = tg[idx - 1], tg[idx]
    span = t1 - t0
    alpha = np.clip((t - t0) / np.maximum(span, 1e-12), 0.0, 1.0)
    inside = (t >= t0) & (t <= t1) & (span <= float(max_gap_s))
    errs = []
    for i in np.flatnonzero(inside):
        qi = quat_slerp(gq[idx[i] - 1], gq[idx[i]], alpha[i])
        dot = abs(float(np.dot(qi, q[i])))
        errs.append(2.0 * math.acos(min(1.0, max(0.0, dot))))
    return np.asarray(errs, float), int(np.count_nonzero(~inside))


def dist_stats(values):
    a = np.asarray([v for v in values if v is not None], float)
    if a.size == 0:
        return {"n": 0, "min": None, "median": None, "p95": None, "max": None, "mean": None}
    return {"n": int(a.size), "min": float(a.min()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max()),
            "mean": float(a.mean())}


def _read_status_stream(path):
    events = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def predicted_service(evaluator, eval_dir, run, gt_path, args, ts_path, work):
    """Score the predict-mode consumer service; returns the block written into the verdict.

    BLOCKED when the record/GT/status stream is missing or the run was not in predict mode.
    FAIL (with ATE null) when the consumer was in predict mode and produced no usable pose.
    """
    period = SERVICE_GRID_PERIOD_S
    rec_path = os.path.join(run, "consumer_queries.json")
    out = {
        "role": "NEW current-position consumer service (predict mode): the poses the REAL "
                "map_pose_publisher published, scored against the GT at the /clock time when they "
                "ARRIVED. It is separate from the frozen message-stamp metric and from the old "
                "0.95 s / 0.10 s protocol, whose failure record is untouched",
        "record": os.path.abspath(rec_path),
        "consumer_mode": None, "consumer_semantics": None,
        "deadline_s": None, "grid_period_s": period,
        "position_threshold_m": float(args.threshold_m),
        "min_post_lock_availability": float(args.min_post_lock_availability),
        "current_clock_ate_rmse_m": None, "current_clock_ate_n": None,
        "current_clock_ate_ontime_rmse_m": None,
        "position_accuracy_met": None,
        "full_span_output_fraction": None, "post_lock_output_fraction": None,
        "post_lock_availability_met": False,
        "orientation_error_rad": None, "prediction_dt_s": None, "reject_reason_counts": None,
        "startup": None, "census": None, "grid": None, "clock": None,
        "evaluator_result_json": None, "est_tum": None,
        "verdict": "BLOCKED", "reason": "",
    }
    if not os.path.isfile(rec_path):
        out["reason"] = "the real consumer record %s was not written" % rec_path
        return out
    rec = sc.read_json(rec_path)
    out["record_sha256"] = sc.sha256_file(rec_path)
    out["consumer_mode"] = rec.get("consumer_mode")
    out["consumer_semantics"] = rec.get("consumer_semantics")
    out["deadline_s"] = rec.get("deadline_s")
    if rc is None:
        out["reason"] = "record_consumer.py is not importable: the join/grid rule cannot be applied"
        return out
    if rec.get("consumer_mode") != "predict":
        out["reason"] = ("the recorded consumer ran %r semantics: the predicted-service metrics "
                         "require the predict mode" % rec.get("consumer_mode"))
        return out
    st_path = rec.get("status_jsonl") or os.path.join(run, "consumer_status.jsonl")
    if not os.path.isfile(st_path):
        out["reason"] = "the full status stream %s is missing" % st_path
        return out
    out["status_jsonl"] = os.path.abspath(st_path)
    out["status_jsonl_sha256"] = sc.sha256_file(st_path)

    # the join is redone here from the RAW events with the recorder's own functions, so the metric
    # cannot depend on an in-process summary that was computed before the stream ended
    statuses = _read_status_stream(st_path)
    poses = [{"wall_rel": r[0], "stamp_ns": r[1], "clock_ns": r[2], "epoch": r[3],
              "position": r[4:7], "orientation": r[7:11]}
             for r in (rec.get("pose_events") or [])]
    joined = rc.join_queries(statuses, poses,
                             deadline_s=float(rec.get("deadline_s", rc.DEADLINE_S)))
    queries, cen = joined["queries"], joined["census"]
    out["census"] = cen
    reasons = {}
    for q in queries:
        tok = q["reject_reason"] or ("(empty)" if q["available"] else "(none)")
        reasons[tok] = reasons.get(tok, 0) + 1
    out["reject_reason_counts"] = reasons
    out["prediction_dt_s"] = {
        "predicted_outputs": dist_stats([q["prediction_dt_s"] for q in queries
                                         if q["predicted"] and q["joined"]]),
        "all_queries": dist_stats([q["prediction_dt_s"] for q in queries]),
        "definition": "query_stamp - latest odometry stamp at the query, as reported by the "
                      "publisher's own status",
    }
    out["clock"] = rec.get("clock")
    rec_grid = rec.get("grid") or {}
    if not queries:
        if cen["status_messages"]:
            # statuses arrived but none carries the predict-mode query contract: a configuration
            # mismatch (the publisher was not answering queries), not a service failure
            out["verdict"] = "BLOCKED"
            out["reason"] = ("none of the %d status messages carries the query contract "
                             "(query_seq/query_stamp_ns): the publisher did not run the predict "
                             "mode" % cen["status_messages"])
            return out
        out["verdict"] = "FAIL"
        out["reason"] = ("the consumer published nothing: %d pose messages and %d status messages "
                         "were recorded in the declared predict mode"
                         % (cen["pose_messages"], cen["status_messages"]))
        return out
    if not cen["joined"]:
        out["verdict"] = "FAIL"
        out["reason"] = ("the consumer emitted %d queries and NO pose ever joined one (unjoined "
                         "%d, all past the %.3f s deadline: %d)"
                         % (cen["queries"], cen["unjoined_no_pose"], float(out["deadline_s"]),
                            cen["late_beyond_deadline"]))
        return out

    gt = load_gt_tum(gt_path)
    if gt.shape[0] < 2:
        out["reason"] = "no usable GT at %s: the current-clock error cannot be measured" % gt_path
        return out
    grid = rc.support_grid(gt[:, 0], args.max_gap_s, period_s=period)
    cells = grid["cells"]
    if not cells:
        out["reason"] = "the GT support holds no complete %.3f s cell" % period
        return out
    origin_ns = grid["origin_clock_ns"]
    period_ns = grid["period_ns"]
    cell_set = set(cells)
    # EPOCH 0 is the reference epoch: after a /clock jump backwards the stamps repeat, so a query
    # from a later epoch may never credit a GT cell that was already lived through (and the
    # post-lock denominator may not be unlocked by evidence from a rewound clock either).
    ref_queries = [q for q in queries if int(q.get("epoch", 0)) == 0]
    out["reference_epoch"] = 0
    out["census_epochs"] = {
        "epochs": cen.get("epochs"),
        "queries_in_reference_epoch": len(ref_queries),
        "queries_after_clock_jumpback": len(queries) - len(ref_queries),
        "note": "queries after a jump-back are still joined (by epoch+stamp) and still scored for "
                "the current-clock error, but they cannot credit a GT grid cell",
    }
    served_cells = set()
    outside_support = 0
    for q in ref_queries:
        k = rc.cell_index_ns(q.get("query_clock_ns"), origin_ns, period_ns)
        if k is None or k not in cell_set:
            outside_support += 1
            continue
        if q["served_on_time"]:
            served_cells.add(k)
    out["full_span_output_fraction"] = len(served_cells) / float(len(cells))
    c_lock_ns = rc.post_lock_start_ns(ref_queries)
    post_cells = ([k for k in cells if origin_ns + k * period_ns > c_lock_ns]
                  if c_lock_ns is not None else [])
    served_post = [k for k in post_cells if k in served_cells]
    out["post_lock_output_fraction"] = (len(served_post) / float(len(post_cells))
                                       if post_cells else 0.0)
    out["post_lock_availability_met"] = bool(
        post_cells and out["post_lock_output_fraction"] >= float(args.min_post_lock_availability))
    out["grid"] = {
        "period_s": period, "origin_clock_s": grid["origin_clock_s"],
        "support_intervals_s": grid["intervals"], "max_gap_s": float(args.max_gap_s),
        "rule": grid["rule"],
        "full_span_cells": len(cells), "full_span_cells_served": len(served_cells),
        "post_lock_cells": len(post_cells), "post_lock_cells_served": len(served_post),
        "post_lock_start_clock_s": None if c_lock_ns is None else c_lock_ns * 1e-9,
        "post_lock_start_rule": "first %.3f s cell AFTER the arrival clock of the first status "
                                "carrying gate_ever_valid && odom_ready && tf_ready (never the "
                                "first `available`); the latch is not reset by a dropout, by a "
                                "clock jump-back, or by a re-visited query stamp" % period,
        "queries_outside_support": outside_support,
        "cells_with_queries_not_served": int(sum(
            1 for k in cell_set
            if k in {rc.cell_index_ns(q.get("query_clock_ns"), origin_ns, period_ns)
                     for q in ref_queries}
            and k not in served_cells)),
        "publisher_clock_grid": {
            "cells_with_clock_no_query": rec_grid.get("cells_with_clock_no_query"),
            "clock_jumps": rec_grid.get("clock_jumps"),
            "clock_pauses": rec_grid.get("clock_pauses"),
            "note": "the recorder's own independent 20 Hz /clock grid, keyed by epoch: a cell where "
                    "the clock advanced but the publisher emitted no status is a missing request, "
                    "not a success",
        },
    }

    # ---------------------------------------- current-clock accuracy of the REAL published poses
    est_path = os.path.join(work, "consumer_predict_now.tum")
    rows_now, rows_all = [], []
    for q in queries:
        if not q["joined"] or q["pose_clock_s"] is None:
            continue
        p = q["pose"]["position"]
        o = q["pose"]["orientation"]
        line = "%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f" % (
            q["pose_clock_s"], p[0], p[1], p[2], o[0], o[1], o[2], o[3])
        rows_all.append(line)
        if q["served_on_time"]:
            rows_now.append(line)
    if not rows_all:
        out["reason"] = ("every published pose arrived with no /clock value: the current-clock "
                         "error is not measurable")
        return out
    with open(est_path, "w") as fh:
        fh.write("\n".join(rows_all) + "\n")
    out["est_tum"] = os.path.abspath(est_path)

    def score(rows, tag):
        if not rows:
            return None
        path = os.path.join(work, "consumer_predict_%s.tum" % tag)
        with open(path, "w") as fh:
            fh.write("\n".join(rows) + "\n")
        res = os.path.join(run, "t3_predicted_service_%s.json" % tag)
        proc = subprocess.run(
            [sys.executable, evaluator, "--est", path, "--gt", gt_path,
             "--time-source", ts_path, "--max-gap-s", str(args.max_gap_s),
             "--source-frame", args.frame, "--target-frame", args.frame,
             "--sensor-entity", args.entity, "--gt-entity", args.entity,
             "--coverage-tolerance-s", str(args.coverage_tolerance_s),
             "--coverage-window", args.coverage_window, "--out", res],
            cwd=eval_dir, capture_output=True, text=True)
        if proc.returncode != 0 or not os.path.isfile(res):
            return None
        d = sc.read_json(res)
        a = ((d.get("association") or {}).get("ate_against_interpolated_gt_m") or {})
        return {"ate_rmse_m": a.get("ate_rmse"), "ate_n": a.get("ate_n"),
                "result_json": os.path.abspath(res), "est_tum": os.path.abspath(path),
                "status": d.get("status")}

    scored = score(rows_all, "now")
    ontime = score(rows_now, "ontime")
    if scored is None or scored.get("ate_rmse_m") is None:
        out["reason"] = ("the published pose stream could not be scored against the GT at its "
                         "arrival clock")
        return out
    out["current_clock_ate_rmse_m"] = scored["ate_rmse_m"]
    out["current_clock_ate_n"] = scored["ate_n"]
    out["evaluator_result_json"] = scored["result_json"]
    out["current_clock_ate_ontime_rmse_m"] = (ontime or {}).get("ate_rmse_m")
    out["current_clock_ate_ontime_n"] = (ontime or {}).get("ate_n")
    out["position_accuracy_met"] = bool(scored["ate_rmse_m"] <= float(args.threshold_m))

    errs, not_bracketed = orientation_errors(
        [q["pose_clock_s"] for q in queries if q["joined"] and q["pose_clock_s"] is not None],
        [q["pose"]["orientation"] for q in queries
         if q["joined"] and q["pose_clock_s"] is not None],
        gt, args.max_gap_s)
    out["orientation_error_rad"] = dict(dist_stats(errs), not_bracketed_n=not_bracketed,
                                        threshold=None,
                                        note="distribution only: the user specified no angular "
                                             "tolerance, so none is invented")

    def first_clock(pred):
        for q in ref_queries:
            if pred(q):
                return q.get("status_clock_ns")
        return None

    first_status = min([q["status_wall_rel"] for q in queries]) if queries else None
    first_served_ns = first_clock(lambda q: q["served_on_time"])
    joined_poses = [q for q in queries if q["joined"] and q["pose_clock_ns"] is not None]
    support_start_ns = rc.now_ns(gt[0, 0])
    out["startup"] = {
        "first_status_wall_rel_s": first_status,
        "first_status_clock_ns": ref_queries[0]["status_clock_ns"] if ref_queries else None,
        "first_pose_clock_ns": min([q["pose_clock_ns"] for q in joined_poses], default=None),
        "first_served_clock_ns": first_served_ns,
        "first_gate_ever_valid_clock_ns": first_clock(lambda q: bool(q["gate_ever_valid"])),
        "first_odom_ready_clock_ns": first_clock(lambda q: bool(q["odom_ready"])),
        "first_tf_ready_clock_ns": first_clock(lambda q: bool(q["tf_ready"])),
        "first_post_lock_evidence_clock_ns": c_lock_ns,
        "gt_support_start_clock_ns": support_start_ns,
        "output_startup_s": (None if first_served_ns is None
                             else (first_served_ns - support_start_ns) * 1e-9),
        "note": "startup is reported SEPARATELY but is never removed from the full-span "
                "denominator; the first-* fields use the reference epoch only",
    }

    met, post = out["position_accuracy_met"], out["post_lock_availability_met"]
    if met and post:
        out["verdict"] = "PASS"
        out["reason"] = ("current-clock ATE %.4f m <= %.2f m over %d joined poses and post-lock "
                         "output fraction %.4f >= %.2f over %d cells"
                         % (out["current_clock_ate_rmse_m"], float(args.threshold_m),
                            out["current_clock_ate_n"], out["post_lock_output_fraction"],
                            float(args.min_post_lock_availability), len(post_cells)))
    else:
        out["verdict"] = "FAIL"
        out["reason"] = ("current-clock ATE %s (<= %.2f m: %s) and post-lock output fraction "
                         "%.4f (>= %.2f: %s); full-span fraction %.4f over %d cells"
                         % (("%.4f m" % out["current_clock_ate_rmse_m"])
                            if out["current_clock_ate_rmse_m"] is not None else "not measurable",
                            float(args.threshold_m), met, out["post_lock_output_fraction"],
                            float(args.min_post_lock_availability), post,
                            out["full_span_output_fraction"], len(cells)))
    return out


def main(argv=None):
    args = parse_args(argv)
    # the frozen evaluator is run with cwd=<eval-dir>, so every path handed to it must be absolute
    run = os.path.abspath(args.run_dir)
    args.ref_dir = os.path.abspath(args.ref_dir)
    eval_dir = args.eval_dir or os.path.join(repo_root(), DEFAULT_EVAL_DIR)
    est = args.est_tum or os.path.join(run, "localization.tum")
    gt = args.gt_tum or os.path.join(args.ref_dir, "gt_localization.tum")
    record_path = args.record or os.path.join(run, "localization_record.json")
    manifest_path = args.manifest or os.path.join(run, "run_manifest.json")
    work = args.work_dir or os.path.join(run, "t3_work")
    out = args.out or os.path.join(run, "t3_result.json")
    os.makedirs(work, exist_ok=True)

    evaluator = os.path.join(eval_dir, "gt_time_assoc_eval.py")
    if not os.path.isfile(evaluator):
        print("[error] evaluator not found: %s" % evaluator)
        return 2
    for path, what in ((est, "localization estimate"), (gt, "scene-frame GT"),
                       (record_path, "localization record")):
        if not os.path.isfile(path):
            print("[blocked] %s not found: %s" % (what, path))
            return 2

    record = sc.read_json(record_path)
    if not record.get("composed_samples"):
        print("[blocked] the recorder composed 0 localization poses (map->odom %d, odom->body "
              "%d): no correction was available to a causal consumer."
              % (record.get("map_odom_messages", 0), record.get("odom_body_messages", 0)))
        return 2
    # A near-identity map<-odom correction is NOT disqualifying: a correctly localized map and
    # odometry frame may genuinely coincide.  The acceptance evidence is (a) a sample exists,
    # which the node only ever emits for an accepted ICP update, and (b) the localizer's own
    # gate answered valid - both are measured by the census below.  The offset is printed as a
    # diagnostic only.
    offset = record.get("map_odom_offset") or {}
    reloc = record.get("relocalize_result") or {}
    print("[T3] initial pose source: %s -> %s"
          % ((record.get("relocalize_request") or {}).get("source", "n/a"),
             reloc.get("message", "n/a")))
    print("[T3] published map<-odom offset (diagnostic, never an acceptance test): mean=%s "
          "span=%s max|x|=%.4f; relocalize_check valid fraction %s"
          % (offset.get("translation_mean_m"), offset.get("translation_span_m"),
             offset.get("max_abs_m") or 0.0, record.get("relocalize_valid_fraction")))

    # ---------------------------------------------------------- online availability --
    # What a RECEIVE-TIME CAUSAL consumer could actually get, at ground-truth resolution,
    # under the frozen consumer rule.  This is a different measurement from the evaluator's
    # +-tol sample-union coverage: that coverage describes sampling density and is capped by
    # the stream's own output rate, so it is never the continuity criterion.  Two availability
    # numbers are produced: the declared AGE CONTRACT (the gated one) and the causal-available
    # diagnostic that drops only the age bound, so "the service is absent" and "the service is
    # alive but slower than the contract" are distinguishable.
    avail_path = args.availability_file or os.path.join(run, "tf_availability.json")
    avail = sc.read_json(avail_path) if os.path.isfile(avail_path) else None
    proto = (record.get("frozen_protocol") or {})
    hold = float(proto.get("hold_validity_s", 0.10))
    avail_out = None
    lock_out = None
    if avail is None:
        print("[blocked] %s not found: the localizer's broadcast/odometry timeline is required "
              "to measure online availability (the +-tol coverage column is NOT an availability "
              "measure)." % avail_path)
    elif not proto:
        print("[blocked] the run record carries no frozen_protocol block: the consumer rule was "
              "not declared before the run.")
    else:
        gt_t = np.loadtxt(gt, usecols=(0,)) if os.path.isfile(gt) else np.zeros(0)
        gt_t = np.atleast_1d(gt_t)
        avail_out = availability_census(avail, gt_t, hold_validity_s=hold,
                                        validity_stale_s=float(
                                            proto.get("validity_stale_s", 1.0)))
        # the same rule with the age bound lifted: "was a live, causal, gate-valid correction
        # available at all" - a diagnostic, never the gate.
        lock_out = availability_census(avail, gt_t, hold_validity_s=float("inf"),
                                       validity_stale_s=float(
                                           proto.get("validity_stale_s", 1.0)))
        print("[T3] online availability (age contract %.2f s, receive-time causal): FULL GT span "
              "%.4f, post-lock %.4f, startup %.2f s; correction age median %s p95 %s"
              % (hold, avail_out["age_contract_fraction_full_span"] or 0.0,
                 avail_out["age_contract_fraction_post_lock"] or 0.0,
                 avail_out["age_contract_startup_s"] or 0.0,
                 avail_out["age_s"]["median"], avail_out["age_s"]["p95"]))
        print("[T3] age-contract failure census: %s" % avail_out["age_contract"])
        print("[T3] causal-available (diagnostic, no age bound): FULL %.4f, post-lock %.4f; "
              "census %s"
              % (lock_out["causal_available_fraction_full_span"] or 0.0,
                 lock_out["causal_available_fraction_post_lock"] or 0.0,
                 lock_out["causal_available"]))

    # Provenance, read from the STAGE record the runner writes (run_localization_sim.sh stores it
    # under stages.localization; reading the manifest root silently skipped this check before).
    # FAIL CLOSED: a run that does not declare its route and its frozen prior cannot be scored as
    # a separate-route localization measurement.
    if not os.path.isfile(manifest_path):
        print("[blocked] no run manifest at %s: the route/prior provenance is undeclared."
              % manifest_path)
        return 2
    man = sc.read_json(manifest_path)
    stage = ((man.get("stages") or {}).get("localization") or {})
    loc_traj = os.path.basename(str(stage.get("trajectory") or ""))
    map_traj = os.path.basename(str(stage.get("mapping_trajectory") or ""))
    prior_sha = str(stage.get("prior_map_sha256") or "")
    if not loc_traj or not map_traj or not prior_sha:
        print("[blocked] the localization stage declares no complete provenance "
              "(trajectory=%r mapping_trajectory=%r prior_map_sha256=%r): a separate-route "
              "measurement cannot be certified without it."
              % (loc_traj, map_traj, prior_sha[:16] if prior_sha else ""))
        return 2
    if loc_traj == map_traj:
        print("[blocked] the localization run reused the MAPPING trajectory (%s): a separate "
              "trajectory is required." % loc_traj)
        return 2
    print("[T3] provenance: trajectory=%s (mapping trajectory was %s), prior map sha256=%s"
          % (loc_traj, map_traj, prior_sha[:16]))

    ts_path = os.path.join(work, "time_source.json")
    sc.write_json(ts_path, {"offset_s": 0.0,
                            "source": "declared epoch-aligned stream stamps (bag clock)",
                            "method": "declared", "gt_derived": False})

    cmd = [sys.executable, evaluator,
           "--est", est, "--gt", gt,
           "--time-source", ts_path,
           "--max-gap-s", str(args.max_gap_s),
           "--source-frame", args.frame,
           "--target-frame", args.frame,
           "--sensor-entity", args.entity,
           "--gt-entity", args.entity,
           "--coverage-tolerance-s", str(args.coverage_tolerance_s),
           "--coverage-window", args.coverage_window,
           "--out", out]
    print("[T3] %s" % " ".join(cmd))
    proc = subprocess.run(cmd, cwd=eval_dir, capture_output=True, text=True)
    if proc.returncode != 0:
        print("[error] evaluator exit code %d" % proc.returncode)
        print((proc.stderr or proc.stdout or "").strip()[-2000:])
        return 2
    if not os.path.isfile(out):
        print("[error] evaluator wrote no result: %s" % out)
        return 2

    data = sc.read_json(out)
    assoc = data.get("association") or {}
    ate = (assoc.get("ate_against_interpolated_gt_m") or {}).get("ate_rmse")
    status = data.get("status")
    usable = data.get("usable_for_accuracy")
    chain = (data.get("frames") or {}).get("chain_status")

    if ate is None:
        print("[blocked] no ATE in %s (status=%s): %s"
              % (out, status, str(data.get("blocker"))[:400]))
        return 2
    coverage = assoc.get("gt_time_covered_fraction")
    cov_detail = assoc.get("gt_coverage_detail") or {}
    window = cov_detail.get("window_duration_s")
    covered_s = cov_detail.get("gt_time_covered_s")
    est_rate = assoc.get("median_est_period_s")
    est_hz = (1.0 / est_rate) if est_rate else None
    # A localization sample at t explains [t - tol, t + tol] of the GT timeline, so a stream at
    # f Hz can never cover more than f * 2 * tol of it.  Reporting that ceiling makes the
    # coverage gate diagnosable instead of arbitrary: if the measured coverage sits at the
    # ceiling, the stream's own output rate - not the map, not the tolerance - is the limit.
    # ---------------------------------------------- online (query-clock) consumer accuracy --
    # The pose the consumer actually gets is the newest RECEIVED correction composed with the
    # newest RECEIVED odometry, and it describes the robot at the time that frame was measured -
    # which lags the consumer's own clock.  The recorder therefore stamps the same poses at the
    # /clock time current when each query arrived; scoring THAT stream against the ground truth
    # at those times is the current-time consumer error (map_pose_publisher.cpp semantics:
    # lookupTransform(map, base_link, TimePointZero) restamped with now()).  The
    # timestamp-associated accuracy on the frame-stamped stream is reported separately and is
    # NOT the acceptance criterion.
    online_tum = os.path.join(run, "localization_online.tum")
    online = None
    if os.path.isfile(online_tum):
        online_out = os.path.join(run, "t3_online_result.json")
        proc2 = subprocess.run(
            [sys.executable, evaluator, "--est", online_tum, "--gt", gt,
             "--time-source", ts_path, "--max-gap-s", str(args.max_gap_s),
             "--source-frame", args.frame, "--target-frame", args.frame,
             "--sensor-entity", args.entity, "--gt-entity", args.entity,
             "--coverage-tolerance-s", str(args.coverage_tolerance_s),
             "--coverage-window", args.coverage_window, "--out", online_out],
            cwd=eval_dir, capture_output=True, text=True)
        if proc2.returncode == 0 and os.path.isfile(online_out):
            d2 = sc.read_json(online_out)
            a2 = (d2.get("association") or {})
            ate2 = (a2.get("ate_against_interpolated_gt_m") or {})
            online = {"ate_rmse_m": ate2.get("ate_rmse"), "ate_n": ate2.get("ate_n"),
                      "coverage": a2.get("gt_time_covered_fraction"),
                      "status": d2.get("status"), "usable": d2.get("usable_for_accuracy"),
                      "est_tum": os.path.abspath(online_tum),
                      "result_json": os.path.abspath(online_out)}
        else:
            online = {"error": "the online stream could not be scored",
                      "returncode": proc2.returncode}
    online_ate = (online or {}).get("ate_rmse_m")
    print("[T3] ONLINE (query-clock) consumer ATE RMSE: %s (n=%s, coverage=%s, status=%s)"
          % (("%.4f m" % online_ate) if online_ate is not None else "not measurable",
             (online or {}).get("ate_n"), (online or {}).get("coverage"),
             (online or {}).get("status")))
    print("[T3] timestamp-associated (delayed-pose) ATE RMSE: %.4f m - diagnostic, NOT the "
          "acceptance criterion" % ate)

    # ------------------------------------------------- ACTUAL consumer (real node) accuracy --
    # The verdict must rest on what the real robot_pose/map_pose_publisher node published, not on
    # a surrogate composition.  Its /robot_pose_map stream is recorded by record_consumer.py with
    # a 20 Hz query census (including the ticks where it could not resolve the chain at all).
    consumer = None
    consumer_ate = None
    consumer_path = os.path.join(run, "consumer_queries.json")
    if os.path.isfile(consumer_path):
        raw = sc.read_json(consumer_path)
        outs = raw.get("outputs") or []
        cen = raw.get("census") or {}
        consumer = {"census": cen, "pose_topic": raw.get("pose_topic"),
                    "consumer_semantics": raw.get("consumer_semantics")}
        if outs:
            est_consumer = os.path.join(work, "consumer_est.tum")
            est_now = os.path.join(work, "consumer_est_now.tum")
            with open(est_consumer, "w") as fh, open(est_now, "w") as fh_now:
                for row in outs:
                    line = ("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                            % (row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8]))
                    fh.write(line)
                    # the SAME pose stamped at the clock current when it arrived: what a consumer
                    # asking "where is the robot now" actually gets, and how old it is
                    clock = row[9] if len(row) > 9 else None
                    if clock is not None:
                        fh_now.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                                     % (clock, row[2], row[3], row[4], row[5], row[6], row[7], row[8]))
            out_consumer = os.path.join(run, "t3_consumer_result.json")
            proc3 = subprocess.run(
                [sys.executable, evaluator, "--est", est_consumer, "--gt", gt,
                 "--time-source", ts_path, "--max-gap-s", str(args.max_gap_s),
                 "--source-frame", args.frame, "--target-frame", args.frame,
                 "--sensor-entity", args.entity, "--gt-entity", args.entity,
                 "--coverage-tolerance-s", str(args.coverage_tolerance_s),
                 "--coverage-window", args.coverage_window, "--out", out_consumer],
                cwd=eval_dir, capture_output=True, text=True)
            if proc3.returncode == 0 and os.path.isfile(out_consumer):
                d3 = sc.read_json(out_consumer)
                a3 = (d3.get("association") or {})
                ate3 = (a3.get("ate_against_interpolated_gt_m") or {})
                consumer_ate = ate3.get("ate_rmse")
                consumer.update({"ate_rmse_m": consumer_ate, "ate_n": ate3.get("ate_n"),
                                 "est_tum": os.path.abspath(est_consumer),
                                 "result_json": os.path.abspath(out_consumer),
                                 "status": d3.get("status"),
                                 "usable": d3.get("usable_for_accuracy")})
            else:
                consumer["error"] = "the consumer stream could not be scored"
            if os.path.isfile(est_now) and os.path.getsize(est_now) > 0:
                out_now = os.path.join(run, "t3_consumer_now_result.json")
                proc4 = subprocess.run(
                    [sys.executable, evaluator, "--est", est_now, "--gt", gt,
                     "--time-source", ts_path, "--max-gap-s", str(args.max_gap_s),
                     "--source-frame", args.frame, "--target-frame", args.frame,
                     "--sensor-entity", args.entity, "--gt-entity", args.entity,
                     "--coverage-tolerance-s", str(args.coverage_tolerance_s),
                     "--coverage-window", args.coverage_window, "--out", out_now],
                    cwd=eval_dir, capture_output=True, text=True)
                if proc4.returncode == 0 and os.path.isfile(out_now):
                    d4 = sc.read_json(out_now)
                    a4 = ((d4.get("association") or {}).get("ate_against_interpolated_gt_m") or {})
                    consumer["current_clock_error_m"] = a4.get("ate_rmse")
                    consumer["current_clock_error_n"] = a4.get("ate_n")
                    consumer["current_clock_result_json"] = os.path.abspath(out_now)
                    ages = [row[9] - row[1] for row in outs
                            if len(row) > 9 and row[9] is not None]
                    if ages:
                        arr2 = np.asarray(ages, float)
                        consumer["published_stamp_age_s"] = {
                            "n": int(arr2.size), "median": float(np.median(arr2)),
                            "p95": float(np.percentile(arr2, 95)), "max": float(arr2.max()),
                            "note": "clock at arrival minus the published stamp: how old the "
                                    "published pose is when a subscriber receives it",
                        }
        # the age of the information the consumer actually used: the chain's common time vs the
        # clock at the tick
        ages = [t[4] for t in (raw.get("ticks") or []) if t[4] is not None]
        if ages:
            arr = np.asarray(ages, float)
            consumer["common_time_age_s"] = {"n": int(arr.size),
                                             "median": float(np.median(arr)),
                                             "p95": float(np.percentile(arr, 95)),
                                             "max": float(arr.max())}
    print("[T3] ACTUAL consumer (%s): ATE at the PUBLISHED stamp %s (n=%s); current-clock error "
          "%s; published-stamp age median %s s; census %s"
          % ((consumer or {}).get("pose_topic", "not recorded"),
             ("%.4f m" % consumer_ate) if consumer_ate is not None else "not measurable",
             (consumer or {}).get("ate_n"),
             ("%.4f m" % consumer["current_clock_error_m"])
             if (consumer or {}).get("current_clock_error_m") is not None else "n/a",
             ((consumer or {}).get("published_stamp_age_s") or {}).get("median"),
             (consumer or {}).get("census")))

    # ------------------------------------------- NEW predicted-position consumer service --
    # A SEPARATE measurement with its own verdict and its own denominator: it never feeds the
    # frozen verdict above and never rewrites the message-stamp metric or the old protocol record.
    service = predicted_service(evaluator, eval_dir, run, gt, args, ts_path, work)
    print("[T3] predicted_service (%s): verdict=%s current-clock ATE %s (n=%s) vs %.2f m -> %s; "
          "full-span %.4f, post-lock %.4f vs %.2f -> %s; orientation median %s rad; start-up %s s; "
          "rejects %s"
          % (service.get("consumer_mode"), service.get("verdict"),
             ("%.4f m" % service["current_clock_ate_rmse_m"])
             if service.get("current_clock_ate_rmse_m") is not None else "not measurable",
             service.get("current_clock_ate_n"), float(args.threshold_m),
             service.get("position_accuracy_met"),
             service.get("full_span_output_fraction") or 0.0,
             service.get("post_lock_output_fraction") or 0.0,
             float(args.min_post_lock_availability), service.get("post_lock_availability_met"),
             ((service.get("orientation_error_rad") or {}).get("median")),
             ((service.get("startup") or {}).get("output_startup_s")),
             service.get("reject_reason_counts")))
    if service.get("verdict") != "PASS":
        print("[T3] predicted_service not PASS: %s" % str(service.get("reason"))[:400])

    ceiled = (est_hz * 2.0 * args.coverage_tolerance_s) if est_hz else None
    startup = (record.get("startup") or {})
    verdict = {
        "ate_rmse_m": ate, "threshold_m": args.threshold_m,
        "coverage": coverage, "min_coverage": args.min_coverage,
        "coverage_gate_applied": False,
        "coverage_tolerance_s": args.coverage_tolerance_s,
        "coverage_window": args.coverage_window,
        "coverage_is_sampling_density_not_availability": True,
        "gt_window_s": window, "gt_covered_s": covered_s,
        "localization_rate_hz": est_hz,
        "max_achievable_coverage_at_this_rate": ceiled,
        "association_rate_est": assoc.get("association_rate_est"),
        "evaluator_status": status, "usable_for_accuracy": usable,
        "chain_status": chain,
        "startup": startup,
        "uncovered_segments_s": assoc.get("unsourced_gaps"),
        "frame_chain_applied": (data.get("frames") or {}).get("applied_to"),
        "alignment_performed_by_this_test": False,
        "initial_pose_source": (record.get("relocalize_request") or {}).get("source"),
        "map_odom_offset_diagnostic": {
            "note": "the published correction's statistics (diagnostic only; a near-identity "
                    "correction is legitimate and is not an acceptance test)",
            "mean_m": offset.get("translation_mean_m"),
            "span_m": offset.get("translation_span_m"),
            "max_abs_m": offset.get("max_abs_m"),
        },
        "evaluator": {"path": os.path.abspath(evaluator),
                      "sha256": sc.sha256_file(evaluator)},
        "evaluator_result_json": os.path.abspath(out),
        "frozen_protocol": proto,
        "min_availability": args.min_availability,
        "availability_file": os.path.abspath(avail_path) if avail is not None else None,
        # the NEW current-position service: its own verdict, its own denominator, never an input
        # to the frozen verdict above
        "predicted_service": service,
        "min_post_lock_availability": args.min_post_lock_availability,
    }
    if avail_out is not None:
        verdict.update({
            # PRIMARY (gated): the complete GT support, startup INCLUDED.  The denominator is
            # never redefined to start at the first success.
            "availability_full_span": avail_out["age_contract_fraction_full_span"],
            "availability_post_lock": avail_out["age_contract_fraction_post_lock"],
            "availability_first_available_epoch_s": avail_out["age_contract_first_epoch_s"],
            "availability_startup_s": avail_out["age_contract_startup_s"],
            "availability_age_census": avail_out["age_contract"],
            "correction_age_s": avail_out["age_s"],
            # DIAGNOSTIC: the same rule with the age bound lifted.
            "causal_available_full_span": lock_out["causal_available_fraction_full_span"],
            "causal_available_post_lock": lock_out["causal_available_fraction_post_lock"],
            "causal_available_census": lock_out["causal_available"],
            "causal_available_role": "diagnostic only (no age bound); separates an absent service "
                                     "from one that is alive but slower than the age contract",
        })
    rate = (record.get("availability") or {})
    print("T3 定位精度: ATE RMSE %.4f m (threshold %.2f m)" % (ate, args.threshold_m))
    print("  关联率 %.3f, GT 时间覆盖 %.3f (%.1f s / %.1f s), 链状态 %s, status=%s, usable=%s"
          % (assoc.get("association_rate_est") or 0.0, coverage or 0.0,
             covered_s or 0.0, window or 0.0, chain, status, usable))
    print("  定位样本中位周期 %s s (%.2f Hz); 关联容差 %.3f s (未放宽)"
          % (("%.4f" % est_rate) if est_rate else "n/a", est_hz or 0.0,
             args.coverage_tolerance_s))
    print("  实际广播速率: 不同 TF 时间戳 %s Hz (消息 %s Hz)"
          % (rate.get("distinct_tf_stamp_rate_hz"), rate.get("tf_broadcast_message_rate_hz")))
    print("  启动（单独报告，仍计入主分母）: %s"
          % {k: startup.get(k) for k in ("first_odom_epoch_s", "first_tf_epoch_s",
                                         "first_lock_adopted_epoch_s",
                                         "first_localization_epoch_s",
                                         "startup_unavailable_s")})
    print("  frame chain applied: %s (no post-fit alignment is performed by this test)"
          % (data.get("frames") or {}).get("applied_to", "n/a"))

    verdict_path = os.path.join(run, "t3_verdict.json")
    verdict["delayed_pose_accuracy"] = {
        "ate_rmse_m": ate, "ate_n": assoc.get("ate_against_interpolated_gt_m", {}).get("ate_n"),
        "role": "timestamp-associated accuracy of the pose for the frame the correction came "
                "from; a diagnostic, never the acceptance criterion",
    }
    verdict["surrogate_held_correction_accuracy"] = online
    verdict["surrogate_role"] = ("NOT the consumer: this stream composes the held map<-odom "
                                 "correction at its own stamp with the NEWEST odometry and then "
                                 "restamps it; the real consumer resolves the chain at its latest "
                                 "COMMON time.  Reported as a diagnostic only.")
    verdict["actual_consumer_accuracy"] = consumer
    verdict["user_requirement"] = ("the REAL robot_pose consumer's published /robot_pose_map pose "
                                   "is within %.2f m of the ground truth AT THE STAMP IT PUBLISHES "
                                   "(truthful timestamps, no post-fit); the current-clock error and "
                                   "the published-stamp age are reported separately as the price of "
                                   "the delayed localization, and are not hidden" % args.threshold_m)
    verdict["user_requirement_met"] = (
        None if consumer_ate is None else bool(consumer_ate <= args.threshold_m))
    verdict["added_protocol_choice"] = ("online availability >= %.2f over the COMPLETE GT "
                                        "support (startup included) under the declared %.2f s "
                                        "correction-age contract - an added protocol choice, NOT "
                                        "the user's requirement" % (args.min_availability, hold))
    if avail_out is None or lock_out is None or not proto:
        verdict["verdict"] = "BLOCKED"
        verdict["reason"] = ("online availability not measurable: the run carries no frozen "
                             "consumer-rule declaration and/or no broadcast timeline")
        sc.write_json(verdict_path, verdict)
        print("[blocked] 无法测量在线可用性（缺少 frozen_protocol / tf_availability.json）")
        return 2
    if ate is None:
        verdict["verdict"] = "BLOCKED"
        verdict["reason"] = "no ATE in %s (status=%s): %s" % (out, status,
                                                             str(data.get("blocker"))[:300])
        sc.write_json(verdict_path, verdict)
        print("[blocked] 无 ATE: status=%s" % status)
        return 2
    avail_full = avail_out["age_contract_fraction_full_span"] or 0.0
    if avail_full < args.min_availability:
        verdict["verdict"] = "FAIL"
        verdict["reason"] = ("online availability %.4f over the COMPLETE GT support (startup "
                             "%.2f s INCLUDED; post-lock %.4f; causal-available %.4f) < protocol "
                             "choice %.2f; age-contract failure census %s; correction age median "
                             "%s p95 %s s; the ACTUAL consumer ATE is %s and does NOT establish "
                             "continuous localization"
                             % (avail_full, avail_out["age_contract_startup_s"] or 0.0,
                                avail_out["age_contract_fraction_post_lock"] or 0.0,
                                lock_out["causal_available_fraction_full_span"] or 0.0,
                                args.min_availability, avail_out["age_contract"],
                                avail_out["age_s"]["median"], avail_out["age_s"]["p95"],
                                ("%.4f m" % consumer_ate) if consumer_ate is not None
                                else "not measurable (unproven)"))
        sc.write_json(verdict_path, verdict)
        print("❌ FAIL: 全支撑期在线可用性 %.4f < 协议选择 %.2f（含启动 %.2f s；门后 %.4f；"
              "因果可得 %.4f）"
              % (avail_full, args.min_availability, avail_out["age_contract_startup_s"] or 0.0,
                 avail_out["age_contract_fraction_post_lock"] or 0.0,
                 lock_out["causal_available_fraction_full_span"] or 0.0))
        print("  失败统计 %s；修正量年龄 中位 %s p95 %s s"
              % (avail_out["age_contract"], avail_out["age_s"]["median"],
                 avail_out["age_s"]["p95"]))
        print("  用户要求（真实消费者 ATE ≤5cm）: %s = %s m"
              % ({True: "满足", False: "未满足", None: "未证实"}[verdict["user_requirement_met"]],
                 ("%.4f" % consumer_ate) if consumer_ate is not None else "n/a"))
        return 1
    if consumer_ate is None:
        verdict["verdict"] = "BLOCKED"
        verdict["reason"] = ("ACTUAL consumer accuracy NOT PROVEN: the real /robot_pose_map stream "
                             "was not recorded or could not be scored (the surrogate held-correction "
                             "%.4f m and the timestamp-associated %.4f m are NOT the criterion)"
                             % (online_ate if online_ate is not None else -1.0, ate))
        sc.write_json(verdict_path, verdict)
        print("[blocked] 真实消费者精度未证实（替代流 %.4f m 不是判据）"
              % (online_ate if online_ate is not None else -1.0))
        return 2
    if consumer_ate <= args.threshold_m:
        verdict["verdict"] = "PASS"
        sc.write_json(verdict_path, verdict)
        print("✅ PASS: 全支撑期在线可用性 %.4f ≥ %.2f，真实消费者 ATE %.4f m ≤ %.2f m"
              % (avail_full, args.min_availability, consumer_ate, args.threshold_m))
        return 0
    verdict["verdict"] = "FAIL"
    verdict["reason"] = ("real consumer ATE %.4f m > %.2f m (surrogate %.4f m; timestamp-associated "
                         "%.4f m; availability %.4f)"
                         % (consumer_ate, args.threshold_m,
                            online_ate if online_ate is not None else -1.0, ate, avail_full))
    sc.write_json(verdict_path, verdict)
    print("❌ FAIL: 真实消费者 ATE %.4fm > %.2fm（可用性 %.4f）"
          % (consumer_ate, args.threshold_m, avail_full))
    return 1


if __name__ == "__main__":
    sys.exit(main())
