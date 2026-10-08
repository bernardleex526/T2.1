#!/usr/bin/env python3
"""Corrected (v2.2) GT association / time bookkeeping evaluator.

Review-driven changes (independent_review R04, R05):

  R04 the coverage column is a real TIME measurement now:
      * the tolerance is PRE-DECLARED (--coverage-tolerance-s, default half of the
        nominal GT period), never adapted to the estimate's own sampling;
      * coverage = measure of the UNION OF INTERVALS in seconds, divided by the
        duration of the explicit evaluation window;
      * the sample-count ratio is kept but is called what it is
        (`gt_samples_covered_fraction`) and is NOT time coverage.
  R05 the frame chain ACTS on the data:
      * the source->target SE(3) and the lever arm are applied to the estimate
        positions before any comparison;
      * if the source and target frames differ (or the sensor entities differ) and
        no chain is supplied, the result is BLOCKED, not silently 'usable';
      * the unaligned diagnostic value is reported separately and labelled.

Unchanged contract points: a time offset must come from a recorded source (a
GT-fitted offset is diagnostic only), interpolation is gap-limited, no
extrapolation, and no new pass thresholds are introduced -- the only lines are
measurability gates.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rsci
from scipy.spatial.transform import Slerp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_common import (  # noqa: E402
    assert_not_per_block, assert_se3, dump_json, load_json, read_tum, sha256_file, st,
)
from map_accuracy_eval import validate_transform_record  # noqa: E402  (shared admissibility gate)

TOOL_VERSION = "gt_time_assoc_eval.py v2.2.0"

ALLOWED_METHODS = {"publish_semantics", "declared", "clock_domain", "hardware_sync"}
FORBIDDEN_SUBSTRINGS = ("fit", "optimis", "optimiz", "scan_match", "scanmatch", "tuned",
                        "guess", "minim", "search")

DEFAULT_GT_NOMINAL_PERIOD_S = 0.1
DEFAULT_MIN_ASSOC_RATE = 0.50
DEFAULT_MIN_GT_COVERAGE = 0.50


class TimeSourceError(ValueError):
    pass


def validate_time_source(ts):
    if ts is None:
        return False, "no time-source record supplied", None
    rec = dict(ts)
    method = str(rec.get("method") or "").lower()
    rec["method_normalised"] = method
    if not method:
        raise TimeSourceError("time-source record has no 'method'")
    if "offset_s" not in rec:
        raise TimeSourceError("time-source record has no 'offset_s'")
    if not rec.get("source"):
        raise TimeSourceError("time-source record has no 'source' string")
    gt_derived = bool(rec.get("gt_derived", False))
    bad = [s for s in FORBIDDEN_SUBSTRINGS if s in method]
    if bad and not gt_derived:
        rec["gt_derived_implied_by_method_name"] = bad
        gt_derived = True
    rec["gt_derived"] = gt_derived
    if gt_derived:
        return False, ("offset is GT-derived (method=%r, gt_derived=%s): diagnostic only, never "
                       "for an accuracy claim" % (method, gt_derived)), rec
    if method not in ALLOWED_METHODS:
        return False, "method %r not in %s" % (method, sorted(ALLOWED_METHODS)), rec
    return True, None, rec


def frame_chain(est_pos, est_quat, *, source_frame, target_frame, sensor_entity, gt_entity,
                T_target_source, lever_arm_m):
    """Apply the DECLARED frame chain to the estimate positions (R05).

    Returns (positions, status, detail).  status is one of
    'applied', 'identity', 'blocked', 'undeclared'.
    """
    if source_frame is None or target_frame is None or sensor_entity is None or gt_entity is None:
        return None, "undeclared", ("the frame/sensor relation is not declared (source_frame=%r "
                                   "target_frame=%r sensor_entity=%r gt_entity=%r): without a "
                                   "declared chain the estimate and the GT cannot be assumed to "
                                   "refer to the same point" % (source_frame, target_frame,
                                                                sensor_entity, gt_entity))
    need_T = source_frame != target_frame
    need_lever = sensor_entity != gt_entity
    if need_T:
        if not isinstance(T_target_source, dict) or "matrix" not in T_target_source:
            return None, "blocked", ("source_frame %r != target_frame %r but no complete "
                                    "T_target_source record (with an explicit 4x4 `matrix`) was "
                                    "supplied; an empty record is NOT identity"
                                    % (source_frame, target_frame))
    if need_lever and lever_arm_m is None:
        return None, "blocked", ("sensor_entity %r != gt_entity %r but no lever arm was supplied"
                                % (sensor_entity, gt_entity))
    p = np.asarray(est_pos, float).copy()
    applied = []
    if lever_arm_m is not None:
        R = Rsci.from_quat(np.asarray(est_quat, float)).as_matrix()
        p = p + np.einsum("nij,j->ni", R, np.asarray(lever_arm_m, float))
        applied.append("lever_arm_m=%s applied in the source frame" % (np.asarray(lever_arm_m).tolist(),))
    if T_target_source is not None:
        M = np.asarray(T_target_source["matrix"], float)     # explicit, never defaulted
        assert_se3(M)
        p = (M[:3, :3] @ p.T).T + M[:3, 3]
        applied.append("T_target_source applied")
    return p, ("applied" if applied else "identity"), "; ".join(applied) or "no chain needed"


def _slerp_rows(q0, q1, alpha):
    out = np.empty_like(q0)
    for i in range(len(q0)):
        if np.allclose(q0[i], q1[i]):
            out[i] = q0[i]
            continue
        sp = Slerp([0.0, 1.0], Rsci.from_quat(np.vstack([q0[i], q1[i]])))
        out[i] = sp(alpha[i]).as_quat()
    return out


def associate(t_est, gt_t, gt_pos, gt_quat, *, offset_s, max_gap_s):
    t_est = np.asarray(t_est, float)
    tg = np.asarray(gt_t, float) + float(offset_s)
    order = np.argsort(tg)
    tg = tg[order]
    gp = np.asarray(gt_pos, float)[order]
    gq = np.asarray(gt_quat, float)[order]

    idx = np.clip(np.searchsorted(tg, t_est), 1, len(tg) - 1)
    t0, t1 = tg[idx - 1], tg[idx]
    span = t1 - t0
    alpha = np.clip((t_est - t0) / np.maximum(span, 1e-12), 0.0, 1.0)
    inside = (t_est >= t0) & (t_est <= t1)
    valid = inside & (span <= float(max_gap_s))
    est_inside_span = (t_est >= tg[0]) & (t_est <= tg[-1])
    pos = np.full((len(t_est), 3), np.nan)
    quat = np.full((len(t_est), 4), np.nan)
    if valid.any():
        p0, p1 = gp[idx[valid] - 1], gp[idx[valid]]
        pos[valid] = p0 + (p1 - p0) * alpha[valid][:, None]
        quat[valid] = _slerp_rows(gq[idx[valid] - 1], gq[idx[valid]], alpha[valid])
    return {"valid": valid, "gt_pos": pos, "gt_quat": quat, "bracket_span_s": span,
            "inside_gt_span": est_inside_span,
            "t_gt_shifted_range_s": [float(tg[0]), float(tg[-1])]}


def union_interval_measure(intervals, window):
    """Measure (seconds) of the union of [a,b] intervals clipped to `window`."""
    a0, a1 = float(window[0]), float(window[1])
    if a1 <= a0 or len(intervals) == 0:
        return 0.0, 0
    iv = np.asarray(intervals, float)
    lo = np.maximum(iv[:, 0], a0)
    hi = np.minimum(iv[:, 1], a1)
    keep = hi > lo
    if not keep.any():
        return 0.0, 0
    lo, hi = lo[keep], hi[keep]
    order = np.argsort(lo)
    lo, hi = lo[order], hi[order]
    total = 0.0
    cur_lo, cur_hi = lo[0], hi[0]
    merged = 1
    for i in range(1, len(lo)):
        if lo[i] <= cur_hi:
            cur_hi = max(cur_hi, hi[i])
        else:
            total += cur_hi - cur_lo
            cur_lo, cur_hi = lo[i], hi[i]
            merged += 1
    total += cur_hi - cur_lo
    return float(total), int(merged)


def uncovered_est_segments(t_est, valid, min_duration_s):
    t = np.asarray(t_est, float)
    v = np.asarray(valid, bool)
    segs = []
    i = 0
    while i < len(v):
        if not v[i]:
            j = i
            while j + 1 < len(v) and not v[j + 1]:
                j += 1
            dur = float(t[j] - t[i])
            if dur >= float(min_duration_s):
                segs.append({"t0_s": float(t[i]), "t1_s": float(t[j]),
                             "duration_s": dur, "est_samples": int(j - i + 1)})
            i = j + 1
        else:
            i += 1
    return segs


def gt_coverage(t_est_valid, gt_t, *, tolerance_s, window):
    """Sample-count fraction AND true time coverage on an explicit window (R04)."""
    tg = np.asarray(gt_t, float)
    te = np.sort(np.asarray(t_est_valid, float))
    out = {"tolerance_s": float(tolerance_s), "predeclared_tolerance": True,
           "window_s": [float(window[0]), float(window[1])],
           "window_definition": "common interval [max(gt_t0, est_t0), min(gt_t1, est_t1)] in the "
                                "offset-corrected domain",
           "window_duration_s": float(max(0.0, window[1] - window[0])),
           "gt_samples": int(len(tg)), "covered_samples": 0,
           "gt_samples_covered_fraction": 0.0,
           "gt_time_covered_s": 0.0, "gt_time_covered_fraction": 0.0,
           "merged_interval_n": 0, "not_covered_segments": []}
    if len(tg) == 0 or len(te) == 0:
        return out
    cov_samp = (np.searchsorted(te, tg + tolerance_s, side="right")
                - np.searchsorted(te, tg - tolerance_s, side="left")) > 0
    out["covered_samples"] = int(cov_samp.sum())
    out["gt_samples_covered_fraction"] = float(cov_samp.mean())
    out["gt_samples_covered_fraction_note"] = ("SAMPLE ratio -- reported for reference only; it is "
                                               "NOT time coverage")
    intervals = np.column_stack([te - tolerance_s, te + tolerance_s])
    meas, merged = union_interval_measure(intervals, window)
    dur = max(1e-12, window[1] - window[0])
    out["gt_time_covered_s"] = meas
    out["gt_time_covered_fraction"] = float(meas / dur)
    out["merged_interval_n"] = merged
    segs = []
    i = 0
    while i < len(tg):
        if not cov_samp[i]:
            j = i
            while j + 1 < len(tg) and not cov_samp[j + 1]:
                j += 1
            segs.append({"t0_s": float(tg[i]), "t1_s": float(tg[j]),
                         "duration_s": float(tg[j] - tg[i]), "gt_samples": int(j - i + 1)})
            i = j + 1
        else:
            i += 1
    out["not_covered_segments"] = segs
    return out


def evaluate(est_tum, gt_tum, *, time_source, max_gap_s=0.20,
             source_frame=None, target_frame=None, sensor_entity=None, gt_entity=None,
             T_target_source=None, lever_arm_m=None, lever_arm_record=None,
             coverage_tolerance_s=None, gt_nominal_period_s=DEFAULT_GT_NOMINAL_PERIOD_S,
             coverage_window="full_gt_span",
             min_assoc_rate=DEFAULT_MIN_ASSOC_RATE, min_gt_coverage=DEFAULT_MIN_GT_COVERAGE,
             min_gap_report_s=0.25, allow_gt_derived_diagnostic=False,
             est_path=None, gt_path=None, run_name=None, sequence=None):
    t_e, p_e, q_e = read_tum(est_tum) if isinstance(est_tum, str) else (
        np.asarray(est_tum[0], float), np.asarray(est_tum[1], float), np.asarray(est_tum[2], float))
    t_g, p_g, q_g = read_tum(gt_tum) if isinstance(gt_tum, str) else (
        np.asarray(gt_tum[0], float), np.asarray(gt_tum[1], float), np.asarray(gt_tum[2], float))

    tol = (float(gt_nominal_period_s) * 0.5 if coverage_tolerance_s is None
           else float(coverage_tolerance_s))
    res = {
        "tool": TOOL_VERSION,
        "protocol": "evaluation_contract.yaml",
        "run_name": run_name, "sequence": sequence,
        "est_path": est_path, "gt_path": gt_path,
        "est_poses": int(len(t_e)), "gt_poses": int(len(t_g)),
        "est_span_s": [float(t_e[0]), float(t_e[-1])] if len(t_e) else None,
        "gt_span_s": [float(t_g[0]), float(t_g[-1])] if len(t_g) else None,
        "max_gap_s": float(max_gap_s),
        "gap_policy": "a target stamp is accepted only if its bracketing GT samples are no further "
                      "apart than max_gap_s; no extrapolation, no spline across holes",
        "coverage_tolerance_s": tol,
        "coverage_tolerance_source": ("pre-declared: half the nominal GT period (%.3f s)"
                                      % gt_nominal_period_s) if coverage_tolerance_s is None
                                     else "explicit --coverage-tolerance-s",
        "no_new_pass_threshold_for_aux_metrics": True,
        "warnings": [], "unknown_states": [],
    }
    if T_target_source is not None:
        assert_not_per_block(T_target_source)

    # R39: a chain that is actually applied must carry an INDEPENDENT source record,
    # judged by the same admissibility gate as the map transform (source artifact +
    # sample/run/time provenance).  A declaration is not a proof.
    if T_target_source is not None and isinstance(T_target_source, dict) and "matrix" in T_target_source:
        if "frame_from" not in T_target_source and source_frame:
            T_target_source = dict(T_target_source, frame_from=source_frame)
        if "frame_to" not in T_target_source and target_frame:
            T_target_source = dict(T_target_source, frame_to=target_frame)
        ok_t, why_t = validate_transform_record(T_target_source, True)
        if not ok_t:
            res = {"tool": TOOL_VERSION, "protocol": "evaluation_contract.yaml",
                   "run_name": run_name, "sequence": sequence, "est_path": est_path,
                   "gt_path": gt_path, "status": "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT",
                   "association": None, "usable_for_accuracy": False, "blocker": why_t,
                   "frames": {"source_frame": source_frame, "target_frame": target_frame,
                              "sensor_entity": sensor_entity, "gt_entity": gt_entity,
                              "chain_status": "rejected"},
                   "warnings": [], "unknown_states": []}
            return res
    if sensor_entity is not None and gt_entity is not None and sensor_entity != gt_entity \
            and lever_arm_m is not None:
        la = lever_arm_record or {}
        if not (la.get("source") and (la.get("source_artifact") or la.get("source_artifact_sha256"))):
            res = {"tool": TOOL_VERSION, "protocol": "evaluation_contract.yaml",
                   "run_name": run_name, "sequence": sequence, "est_path": est_path,
                   "gt_path": gt_path,
                   "status": "BLOCKED_UNSOURCED_LEVER_ARM", "association": None,
                   "usable_for_accuracy": False,
                   "blocker": "the lever arm is applied but carries no independent source record "
                              "(source + source_artifact/sha256)",
                   "frames": {"sensor_entity": sensor_entity, "gt_entity": gt_entity,
                              "chain_status": "rejected"},
                   "warnings": [], "unknown_states": []}
            return res

    p_e_used, chain_status, chain_detail = frame_chain(
        p_e, q_e, source_frame=source_frame, target_frame=target_frame,
        sensor_entity=sensor_entity, gt_entity=gt_entity,
        T_target_source=T_target_source, lever_arm_m=lever_arm_m)
    res["frames"] = {
        "source_frame": source_frame, "target_frame": target_frame,
        "sensor_entity": sensor_entity, "gt_entity": gt_entity,
        "lever_arm_m": lever_arm_m,
        "lever_arm_record": ({k: (lever_arm_record or {}).get(k) for k in
                              ("source", "source_artifact", "source_artifact_sha256")}
                             if lever_arm_record else None),
        "T_target_source_source": (T_target_source or {}).get("source") if isinstance(T_target_source, dict) else None,
        "chain_status": chain_status, "chain_detail": chain_detail,
        "applied_to": "estimate positions (and the lever arm in the source frame)",
    }
    if T_target_source is not None:
        M = np.asarray(T_target_source.get("matrix", np.eye(4)), float)
        assert_se3(M)
        res["frames"]["T_target_source"] = [[float(v) for v in r] for r in M]
    if chain_status in ("blocked", "undeclared"):
        res.update({"status": "BLOCKED_MISSING_FRAME_CHAIN" if chain_status == "blocked"
                              else "BLOCKED_FRAME_CHAIN_UNDECLARED",
                    "association": None, "usable_for_accuracy": False, "blocker": chain_detail})
        return res

    try:
        usable, why, ts = validate_time_source(time_source)
    except TimeSourceError as exc:
        res.update({"status": "BLOCKED_TIME_SOURCE_INVALID", "association": None,
                    "usable_for_accuracy": False, "blocker": str(exc)})
        return res
    res["time_source"] = ts
    if not usable and not allow_gt_derived_diagnostic:
        res.update({"status": "BLOCKED_TIME_SOURCE_NOT_SOURCED", "association": None,
                    "usable_for_accuracy": False, "blocker": why})
        return res
    if not usable:
        res["unknown_states"].append({"code": "UNKNOWN_GT_DERIVED_OFFSET", "detail": why})
        res["usable_for_accuracy"] = False
        res["diagnostic_only_reason"] = why

    a = associate(t_e, t_g, p_g, q_g, offset_s=ts["offset_s"], max_gap_s=max_gap_s)
    valid = a["valid"]
    n_assoc = int(valid.sum())
    n_est = int(len(t_e))
    tg_shift = np.asarray(t_g, float) + float(ts["offset_s"])
    # the denominator is the FULL evaluation window (the GT timeline the estimate is
    # supposed to explain), not the intersection with the estimate -- otherwise a
    # truncated estimate would trivially cover 100 % of a shrunken window
    window_gt = (float(tg_shift[0]), float(tg_shift[-1]))
    window_common = ((max(window_gt[0], float(t_e[0])), min(window_gt[1], float(t_e[-1])))
                     if n_est else window_gt)
    window = window_gt if coverage_window == "full_gt_span" else window_common
    cov = gt_coverage(t_e[valid], tg_shift, tolerance_s=tol, window=window)
    cov["time_domain"] = "GT stamps after applying offset_s (the association domain)"
    cov["window_mode"] = coverage_window
    cov["common_window_s"] = [float(window_common[0]), float(window_common[1])]
    cov_common = gt_coverage(t_e[valid], tg_shift, tolerance_s=tol, window=window_common)
    cov["gt_time_covered_fraction_common_window"] = cov_common["gt_time_covered_fraction"]

    err = np.linalg.norm(p_e_used[valid] - a["gt_pos"][valid], axis=1) if n_assoc else np.empty(0)
    err_raw = np.linalg.norm(p_e[valid] - a["gt_pos"][valid], axis=1) if n_assoc else np.empty(0)
    large_gap = int((a["bracket_span_s"] > float(max_gap_s)).sum())

    res["association"] = {
        "n_associated": n_assoc,
        "association_rate_est": float(n_assoc) / n_est if n_est else 0.0,
        "association_rate_gt": float(n_assoc) / len(t_g) if len(t_g) else 0.0,
        "association_rate_est_definition": "fraction of ESTIMATE stamps that received a GT value",
        "gt_samples_covered_fraction": cov["gt_samples_covered_fraction"],
        "gt_time_covered_s": cov["gt_time_covered_s"],
        "gt_time_covered_fraction": cov["gt_time_covered_fraction"],
        "gt_coverage_detail": cov,
        "gt_time_coverage_definition": "union of [t_i - tol, t_i + tol] over associated estimates, "
                                       "intersected with the evaluation window, divided by the "
                                       "window duration",
        "rate_100pct_is_not_full_coverage": True,
        "est_stamps_outside_gt_span": int((~a["inside_gt_span"]).sum()),
        "est_stamps_outside_gt_span_fraction": (float((~a["inside_gt_span"]).mean()) if n_est else None),
        "rejected_because_bracket_gap_gt_max_gap_n": large_gap,
        "est_segments_without_gt": uncovered_est_segments(t_e, valid, min_gap_report_s),
        "bracket_span_stats_s": st(a["bracket_span_s"][valid], "bracket_span"),
        "median_est_period_s": (float(np.median(np.diff(t_e))) if n_est > 1 else None),
        "ate_against_interpolated_gt_m": st(err, "ate"),
        "ate_raw_unaligned_m": st(err_raw, "ate_unaligned"),
        "ate_note": "diagnostic only; the map-accuracy claim is NOT derived from this. "
                    "ate_raw_unaligned_m is the SAME comparison WITHOUT the declared frame chain.",
        "unsourced_gaps": [s for s in cov["not_covered_segments"]
                           if s["duration_s"] >= min_gap_report_s],
    }

    status, blocker = "MEASURED", None
    if n_est == 0 or len(t_g) < 3:
        status, blocker = "NOT_MEASURABLE_NO_DATA", "empty estimate or GT trajectory"
    elif res["association"]["association_rate_est"] < min_assoc_rate:
        status = "NOT_MEASURABLE_ASSOCIATION_FAILED"
        blocker = ("association rate %.3f < %.2f: the two streams cannot be compared"
                   % (res["association"]["association_rate_est"], min_assoc_rate))
    elif cov["gt_time_covered_fraction"] < min_gt_coverage:
        status = "NOT_MEASURABLE_GT_COVERAGE_LOW"
        blocker = ("time coverage %.3f of the %.1f s full evaluation window < %.2f: the estimate only "
                   "explains part of the GT timeline (an association rate of %.3f is NOT coverage)"
                   % (cov["gt_time_covered_fraction"], cov["window_duration_s"], min_gt_coverage,
                      res["association"]["association_rate_est"]))
    res.update({"status": status, "blocker": blocker})
    res["usable_for_accuracy"] = bool(res.get("usable_for_accuracy", True)
                                      and status == "MEASURED")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--est", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--time-source", required=True,
                    help="json {offset_s, source, method, gt_derived?}; offset is ADDED to GT stamps")
    ap.add_argument("--max-gap-s", type=float, default=0.20)
    ap.add_argument("--source-frame", default=None)
    ap.add_argument("--target-frame", default=None)
    ap.add_argument("--sensor-entity", default=None)
    ap.add_argument("--gt-entity", default=None)
    ap.add_argument("--t-target-source", default=None)
    ap.add_argument("--lever-arm-record", default=None,
                    help="json {source, source_artifact|sha256} for the lever arm")
    ap.add_argument("--coverage-tolerance-s", type=float, default=None)
    ap.add_argument("--coverage-window", default="full_gt_span",
                    choices=["full_gt_span", "common"])
    ap.add_argument("--gt-nominal-period-s", type=float, default=DEFAULT_GT_NOMINAL_PERIOD_S)
    ap.add_argument("--min-assoc-rate", type=float, default=DEFAULT_MIN_ASSOC_RATE)
    ap.add_argument("--min-gt-coverage", type=float, default=DEFAULT_MIN_GT_COVERAGE)
    ap.add_argument("--allow-gt-derived-diagnostic", action="store_true")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    res = evaluate(args.est, args.gt, time_source=load_json(args.time_source),
                   max_gap_s=args.max_gap_s, source_frame=args.source_frame,
                   target_frame=args.target_frame, sensor_entity=args.sensor_entity,
                   gt_entity=args.gt_entity, T_target_source=load_json(args.t_target_source),
                   lever_arm_record=load_json(args.lever_arm_record),
                   coverage_tolerance_s=args.coverage_tolerance_s,
                   coverage_window=args.coverage_window,
                   gt_nominal_period_s=args.gt_nominal_period_s,
                   min_assoc_rate=args.min_assoc_rate,
                   min_gt_coverage=args.min_gt_coverage,
                   allow_gt_derived_diagnostic=args.allow_gt_derived_diagnostic,
                   est_path=args.est, gt_path=args.gt)
    res["inputs"] = {"est_sha256": sha256_file(args.est), "gt_sha256": sha256_file(args.gt),
                     "time_source_sha256": sha256_file(args.time_source)}
    dump_json(args.out, res)
    print("status=%s assoc_rate=%s gt_time_coverage=%s"
          % (res["status"],
             None if not res.get("association") else round(res["association"]["association_rate_est"], 4),
             None if not res.get("association") else round(res["association"]["gt_time_covered_fraction"], 4)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
