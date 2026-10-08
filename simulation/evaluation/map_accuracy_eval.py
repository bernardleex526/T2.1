#!/usr/bin/env python3
"""Corrected (v2.2) T2.1 MAP-ACCURACY evaluator -- main metric.

Main metric (definition unchanged, bookkeeping corrected):
    map est->ref POINT-TO-PLANE RMSE <= 0.05 m
over a FIXED denominator inside a pre-defined support region, with the raw value
always as the headline, explicit support classification, and pass gates that an
empty / low-coverage / self-referential / fitted-registration comparison can
never pass.

Review-driven changes of v2.2 (independent_review R01-R07):
  R01 the headline value is ALWAYS the raw RMSE; --truncate-at only adds a
      diagnostic bracket and can never be the basis of a PASS.
  R02 the estimated map is sampled on the frozen grid over the WHOLE map, the
      nearest reference neighbour is searched in the FULL reference cloud (not a
      region-cropped one), and each sample is classified supported /
      drifted-out-of-support / no-reference-locally.  Region membership, the
      outside-region count and the fixed denominator are recorded, hashed and
      reported; an indistinguishable no-reference case is a decisive unknown.
  R03 a supplied SE(3) is admissible only with an independent source record:
      source + source artifact (+sha256) + independent_of_evaluation=true +
      a declared fit region that is disjoint from the evaluation region
      (cross-checked when both are boxes).  A source string alone is not
      evidence.
  R04 (see gt_time_assoc_eval.py) time coverage is a measured interval union in
      SECONDS with a pre-declared tolerance, over an explicit evaluation window.
  R05 (see gt_time_assoc_eval.py) T_target_source and the lever arm are APPLIED
      to the estimate; a missing chain is BLOCKED, not merely recorded.
  R06 exact coincidence is a 3-D point coincidence (C2C distance), never a zero
      point-to-plane residual (which only means the normal matched).
  R07 warnings and DECISIVE unknowns are separated; a decisive unknown can never
      coexist with a PASS.
  R38 sampled estimate points outside the frozen region are attributable ONLY
      when they are MEMBERS of a member set whose geometry the
      --outside-attribution record itself declares (region algebra over
      include/exclude boxes) AND whose declared frame is the frame the outside
      points are actually measured in (the frozen ROI's frame).  The membership
      is MEASURED against that declared geometry; the record's declarative flags
      (source / artifact / independence) alone attribute nothing.  Any point the
      declared geometry does not cover stays an unknown residual and blocks an
      overall PASS (fail-closed).  There is no tolerance fraction and no bound
      tuned to the estimate's own point count.

This module never writes into the legacy outputs and never runs an online
algorithm.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_common import (  # noqa: E402
    DEFAULT_COVERAGE_FLOOR, DEFAULT_MAX_DEGENERATE_FRAC, DEFAULT_MIN_DENOMINATOR,
    DEGENERACY_WARN_FRAC, E_EXACT_COINCIDENCE_M, SUPPORT_RADIUS_M, AlignmentError,
    apply_se3, assert_not_per_block, assert_se3, dump_json, estimate_normals,
    frac_within, half_voxel_offset, load_json, load_points, plane_thickness,
    rot_angle_deg, sha256_file, sha256_obj, st, truncation_bracket, voxel_grid_select,
)

TOOL_VERSION = "map_accuracy_eval.py v2.3.1"
PRIMARY_THRESHOLD_M = 0.05
REGIONAL_GAP_FACTOR = 2.0
DEFAULT_UNKNOWN_NO_REFERENCE_MAX_FRAC = 0.05
# R06: a true point coincidence is a 3-D (C2C) distance below 1 mm.  A zero
# point-to-plane residual is NOT a coincidence -- it only means the normal
# matched, which a purely tangential offset produces everywhere.
E_EXACT_COINCIDENCE_C2C_M = 0.001


# ------------------------------------------------------------------- ROI ----
def roi_mask(roi, pts):
    if roi is None:
        return np.ones(len(pts), dtype=bool)
    t = roi.get("type")
    b = roi.get("bounds") or {}
    p = np.asarray(pts, dtype=float)
    if t == "aabb":
        lo, hi = np.asarray(b["min"], float), np.asarray(b["max"], float)
        return np.all((p >= lo) & (p <= hi), axis=1)
    if t == "sphere":
        c = np.asarray(b["centre"], float)
        return np.linalg.norm(p - c, axis=1) <= float(b["radius_m"])
    if t == "polygon_2d_zrange":
        from matplotlib.path import Path
        v = np.asarray(p, float)
        inside = Path(np.asarray(b["polygon_xy_m"], float)).contains_points(v[:, :2])
        return inside & (v[:, 2] >= float(b["z_min_m"])) & (v[:, 2] <= float(b["z_max_m"]))
    raise ValueError("unsupported ROI type %r" % t)


def validate_roi(roi):
    if roi is None:
        return False, "no ROI supplied"
    if roi.get("invalid") is True:
        return False, ("the ROI artifact declares itself invalid (%s); use its superseded_by "
                       "replacement instead of a known-broken region"
                       % (roi.get("invalid_reason") or "no reason recorded"))
    prov = roi.get("provenance") or {}
    if prov.get("independent_of_estimate") is not True:
        return False, "ROI provenance does not assert independent_of_estimate=true"
    if prov.get("independent_of_error") is not True:
        return False, "ROI provenance does not assert independent_of_error=true"
    if roi.get("type") == "aabb" and not (roi.get("bounds") or {}).get("min"):
        return False, "ROI aabb has no bounds"
    return True, None


# ------------------------------------------------------- member geometry ----
# R38: the member set an outside-attribution record may claim is defined by
# GEOMETRY the record itself declares, never by a flag.  Only region algebra
# over the declared boxes enters the membership test, so relocating the estimate
# (or its point count) cannot change the member set.
MEMBER_REGION_SCHEMA = "member_region/v1"


def _member_region_box(region):
    """Normalise one declared member box to (min[3], max[3]); None if unusable."""
    if not isinstance(region, dict):
        return None
    if "min" in region and "max" in region:
        lo, hi = region["min"], region["max"]
    else:
        b = region.get("bounds") or {}
        lo, hi = b.get("min"), b.get("max")
    if lo is None or hi is None or len(lo) != 3 or len(hi) != 3:
        return None
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)) or np.any(lo > hi):
        return None
    return (lo, hi)


def parse_member_region(record, eval_frame):
    """Extract the independently declared member geometry from an attribution record.

    Returns (spec, None) on success or (None, reason) when the record carries no
    usable geometry.  `spec` is include-minus-exclude region algebra over the
    declared boxes, plus a hash of exactly the geometry used.  No estimate point,
    estimate count, voxel grid or residual is consulted anywhere in this
    function -- independence is a property of the declared boxes only.

    `eval_frame` is the frame the outside points are actually measured in (the
    frozen support ROI's frame).  The declared boxes are only meaningful there, so
    a missing or mismatched member frame is refused: the SAME numbers in another
    frame are a different region.  A declared `frame_conversion` is refused as
    well -- re-boxing a rigidly rotated AABB would INFLATE the declared region and
    over-attribute, which is the failure mode this gate exists to prevent; a
    conversion with a source must be applied upstream so the declared boxes are
    already expressed in the evaluation frame.
    """
    mr = (record or {}).get("member_region")
    if not isinstance(mr, dict):
        return None, ("the record carries no member_region geometry: declarative flags alone "
                      "attribute nothing and every outside point stays an unknown residual")
    if mr.get("independent_of_estimate") is not True or mr.get("independent_of_error") is not True:
        return None, ("member_region does not assert independent_of_estimate=true and "
                      "independent_of_error=true")
    if not (mr.get("derivation") or mr.get("source")):
        return None, "member_region carries no derivation/source string"
    frame = mr.get("frame")
    if not eval_frame:
        return None, ("the evaluation ROI declares no frame, so the member_region frame cannot be "
                      "checked against the frame the outside points are measured in")
    if not frame:
        return None, ("member_region declares no frame, so its boxes cannot be tied to the "
                      "evaluation frame %r" % (eval_frame,))
    if str(frame) != str(eval_frame):
        if mr.get("frame_conversion") is not None:
            return None, ("member_region.frame %r != evaluation frame %r and a frame_conversion is "
                          "declared: the evaluator does NOT convert member geometry (rigid re-boxing "
                          "of the declared region would inflate it and over-attribute). Apply the "
                          "sourced conversion upstream and declare boxes already in frame %r"
                          % (frame, eval_frame, eval_frame))
        return None, ("member_region.frame %r != evaluation frame %r: the same numeric bounds in "
                      "another frame are a different region, so nothing can be attributed (no "
                      "sourced conversion is applied by the evaluator)" % (frame, eval_frame))
    includes, excludes, unusable = [], [], []
    for key, sink in (("include", includes), ("exclude", excludes)):
        items = mr.get(key) or []
        if isinstance(items, dict):
            items = [items]
        elif not isinstance(items, (list, tuple)):
            unusable.append("%s (not a box or list of boxes)" % key)
            continue
        for i, item in enumerate(items):
            box = _member_region_box(item)
            if box is None:
                unusable.append("%s[%d]" % (key, i))
            else:
                sink.append(box)
    if unusable:
        return None, "member_region has unusable boxes: %s" % ", ".join(unusable)
    if not includes:
        return None, "member_region declares no include box, so the member set is empty"
    spec = {
        "schema": mr.get("schema"),
        "frame": mr.get("frame"),
        "set_operation": mr.get("set_operation"),
        "include": [([float(x) for x in lo], [float(x) for x in hi]) for lo, hi in includes],
        "exclude": [([float(x) for x in lo], [float(x) for x in hi]) for lo, hi in excludes],
    }
    spec["geometry_sha256"] = sha256_obj(spec)
    spec["n_include"] = len(spec["include"])
    spec["n_exclude"] = len(spec["exclude"])
    return spec, None


def member_region_mask(spec, pts):
    """Membership of `pts` in the DECLARED member set: inside any include box and
    outside every exclude box.  Pure region algebra over the declared geometry."""
    p = np.asarray(pts, dtype=float)
    mask = np.zeros(len(p), dtype=bool)
    for lo, hi in spec["include"]:
        mask |= np.all((p >= lo) & (p <= hi), axis=1)
    for lo, hi in spec["exclude"]:
        mask &= ~np.all((p >= lo) & (p <= hi), axis=1)
    return mask


# ----------------------------------------------------------- region algebra --
def region_box(region):
    """Normalise a region descriptor to (kind, payload) for disjointness tests."""
    if region is None:
        return None
    if isinstance(region, str):
        return ("id", region)
    if isinstance(region, (list, tuple)) and len(region) == 2:
        return ("aabb", (np.asarray(region[0], float), np.asarray(region[1], float)))
    if isinstance(region, dict):
        if "roi_id" in region:
            return ("id", region["roi_id"])
        if "min" in region and "max" in region:
            return ("aabb", (np.asarray(region["min"], float), np.asarray(region["max"], float)))
    return ("unknown", repr(region))


def regions_may_intersect(fit_region, eval_region):
    a, b = region_box(fit_region), region_box(eval_region)
    if a is None or b is None:
        return True                      # unknown -> assumed to intersect
    if a[0] == "aabb" and b[0] == "aabb":
        lo_a, hi_a = a[1]
        lo_b, hi_b = b[1]
        return bool(np.all(lo_a <= hi_b) and np.all(lo_b <= hi_a))
    if a[0] == "id" and b[0] == "id":
        return a[1] == b[1]
    return True


def _ranges_overlap(a, b):
    for x in (a or []):
        for y in (b or []):
            if len(x) == 2 and len(y) == 2 and float(x[0]) <= float(y[1]) and float(y[0]) <= float(x[1]):
                return True
    return False


def provenance_disjoint(rec):
    """Is the transform's FIT sample set provably disjoint from the EVAL set?

    Independence is a property of the SAMPLES / RUNS / TIME INDICES and their
    provenance -- NOT of spatial footprint.  Granularity order (finest first):

      1. explicit sample identity (fit_sample_ids / eval_sample_ids) -- wins over
         everything: the same run may contain pre-declared independent samples;
      2. time-index ranges, but ONLY when both sides live in the same declared
         time domain (two runs whose device-relative clocks both start at 0 are
         NOT overlapping just because the numbers coincide);
      3. run ids as a coarse fallback (a shared run id with no finer provenance is
         conservative: it cannot be shown to be independent).

    A bare `sample_disjoint` flag is not evidence.
    Returns (state, detail) with state in {'disjoint','overlapping','unknown'}.
    """
    fit_s, eval_s = rec.get("fit_sample_ids"), rec.get("eval_sample_ids")
    if fit_s and eval_s:
        shared = sorted(set(fit_s) & set(eval_s))
        if shared:
            return "overlapping", "shared fit/eval sample ids %s" % shared[:5]
        return "disjoint", "sample ids disjoint (%d fit vs %d eval)" % (len(fit_s), len(eval_s))

    dom_f = rec.get("fit_time_domain") or rec.get("time_domain")
    dom_e = rec.get("eval_time_domain") or rec.get("time_domain")
    fit_tr, eval_tr = rec.get("fit_time_ranges"), rec.get("eval_time_ranges")
    has_time = bool(fit_tr and eval_tr)
    shared_domain = bool(dom_f and dom_e and dom_f == dom_e)
    diff_domain = bool(dom_f and dom_e and dom_f != dom_e)

    fit_runs, eval_runs = rec.get("fit_run_ids"), rec.get("eval_run_ids")
    shared_runs = sorted(set(fit_runs) & set(eval_runs)) if (fit_runs and eval_runs) else []

    # If both sides are the same run, but pre-declared independent time ranges in a shared time domain -> disjoint
    if shared_runs:
        if has_time and shared_domain:
            if _ranges_overlap(fit_tr, eval_tr):
                return "overlapping", "overlapping fit/eval time-index ranges in the shared time domain"
            return "disjoint", "time-index ranges disjoint in the shared time domain %s" % dom_f
        return "overlapping", ("the transform was fitted on the evaluation samples (shared fit/eval run ids %s with no finer-grained sample or declared time-domain provenance)" % shared_runs)

    if has_time:
        if diff_domain:
            time_state, time_detail = "incomparable", ("time ranges live in different time domains (%s vs %s)" % (dom_f, dom_e))
        elif _ranges_overlap(fit_tr, eval_tr):
            return "overlapping", "overlapping fit/eval time-index ranges"
        else:
            time_state, time_detail = "disjoint", "time-index ranges disjoint"
    else:
        time_state, time_detail = "absent", "no fit/eval time-index ranges"

    if fit_runs and eval_runs:
        return "disjoint", "run ids disjoint (%s)" % time_detail
    if has_time and not diff_domain:
        return "disjoint", time_detail
    if rec.get("sample_disjoint") and not (fit_s or eval_s or fit_runs or eval_runs or has_time):
        return "unknown", "a bare sample_disjoint flag is not evidence of independence: concrete run/time/sample provenance is required"
    if time_state == "incomparable":
        return "unknown", ("cannot establish disjointness: %s, and no run ids were declared" % time_detail)
    return "unknown", ("no concrete fit/eval sample-id, run-id or time-index provenance to establish disjointness (%s)" % time_detail)


def validate_transform_record(rec, matrix_present):
    """R03 + Main correction: admissibility of a supplied SE(3).

    Required: source, a source ARTIFACT, independent_of_evaluation=true, and a
    PROVABLE sample/run/time-index disjointness between the fit and the
    evaluation sets.  Spatial overlap of fit_region and eval_region is recorded
    and warned about, but is NEVER the deciding criterion.
    """
    if not matrix_present:
        return True, None
    if not isinstance(rec, dict):
        return False, "no transform record supplied with the matrix"
    missing = [k for k in ("source", "frame_from", "frame_to", "fit_region", "eval_region")
               if rec.get(k) in (None, "")]
    if missing:
        return False, "transform record missing %s" % missing
    if not (rec.get("source_artifact") or rec.get("source_artifact_sha256")):
        return False, ("the transform has no source ARTIFACT (path or sha256): a free-text source "
                       "string is not evidence of provenance")
    if rec.get("independent_of_evaluation") is not True:
        return False, ("the transform record does not assert independent_of_evaluation=true: a "
                       "transform fitted on the evaluation data cannot certify accuracy")
    state, detail = provenance_disjoint(rec)
    if state == "overlapping":
        return False, ("the transform was fitted on the evaluation samples (%s): a registration "
                       "fitted on the evaluation data cannot certify accuracy" % detail)
    if state == "unknown":
        return False, ("transform independence cannot be established: %s" % detail)
    return True, None


# --------------------------------------------------------------- support -----
def reference_occupancy_cell_tree(ref_pts, voxel, origin):
    """Tree over the occupied reference grid CELLS (metres) inside the cloud."""
    cells = np.unique(np.floor((np.asarray(ref_pts, float) - np.asarray(origin, float))
                               / voxel).astype(np.int64), axis=0)
    centres = (cells + 0.5) * voxel + np.asarray(origin, float)
    return cKDTree(centres, compact_nodes=True, balanced_tree=True), centres


def support_classification(nn_dist, ref_cell_tree, est_points, voxel, origin, support_radius,
                           no_reference_radius_m=None):
    """Split samples into supported / drifted_out / no_reference_locally (R02).

    `nn_dist` are distances to the FULL reference cloud.  `no_reference_locally`
    means the reference has no occupied cell within 2 voxels of the sample, so
    'the estimate drifted' and 'the reference is absent here' cannot be told
    apart -- that is an explicit unknown, not silently a success.
    """
    if ref_cell_tree is None:
        unknown = np.zeros(len(est_points), dtype=bool)
    else:
        cells = (np.floor((np.asarray(est_points, float) - np.asarray(origin, float))
                          / voxel) + 0.5) * voxel + np.asarray(origin, float)
        r_noref = float(no_reference_radius_m if no_reference_radius_m is not None
                        else max(2.0 * voxel, 10.0 * support_radius))
        counts = ref_cell_tree.query_ball_point(cells, r=r_noref, return_length=True)
        unknown = np.asarray(counts) == 0
    supported = nn_dist <= support_radius
    no_ref = unknown & ~supported
    drifted = (~unknown) & (~supported)
    return supported, drifted, no_ref


# -------------------------------------------------------------- ICP (opt) ----
def conditional_icp(est_pts, ref_pts, voxels, max_dists, max_iter):
    try:
        import open3d as o3d
    except ImportError:
        return None, {"error": "open3d unavailable"}
    est = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(est_pts, float)))
    ref = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(ref_pts, float)))
    log = []
    T = np.eye(4)
    for i, vx in enumerate(voxels):
        e = est.voxel_down_sample(vx)
        r = ref.voxel_down_sample(vx)
        r.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vx * 3.0, max_nn=30))
        md = max_dists[i] if isinstance(max_dists, (list, tuple)) else max_dists
        res = o3d.pipelines.registration.registration_icp(
            e, r, md, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
        T = np.asarray(res.transformation, float)
        log.append({"voxel_m": float(vx), "max_corr_dist_m": float(md),
                    "fitness": float(res.fitness), "inlier_rmse_m": float(res.inlier_rmse)})
    return T, {"stages": log}


# ------------------------------------------------------------------ core ----
def evaluate_map(est_pts, ref_pts, *, stat_voxel=0.05,
                 grid_origin_est=(0.0, 0.0, 0.0), grid_origin_ref=None,
                 seed=0, normal_k=15, normal_radius=0.15,
                 support_radius=SUPPORT_RADIUS_M, roi=None,
                 T_manager_ref_est=None, transform_record=None,
                 truncate_at=None, icp=None, coverage_floor=DEFAULT_COVERAGE_FLOOR,
                 min_denominator=DEFAULT_MIN_DENOMINATOR,
                 max_degenerate_frac=DEFAULT_MAX_DEGENERATE_FRAC,
                 unknown_no_reference_max_frac=DEFAULT_UNKNOWN_NO_REFERENCE_MAX_FRAC,
                 outside_attribution=None,
                 est_map_path=None, ref_map_path=None, roi_path=None,
                 run_name=None, sequence=None, notes=None):
    est_pts = np.asarray(est_pts, dtype=float)
    ref_pts = np.asarray(ref_pts, dtype=float)
    grid_origin_ref = tuple(grid_origin_ref) if grid_origin_ref is not None else tuple(grid_origin_est)

    res = {
        "tool": TOOL_VERSION,
        "protocol": "evaluation_contract.yaml",
        "main_metric": "map est->ref point-to-plane RMSE on the frozen support region",
        "threshold_m": PRIMARY_THRESHOLD_M,
        "is_ate": False,
        "is_cloud_to_cloud": False,
        "inputs": {"est_map": est_map_path, "ref_map": ref_map_path, "roi": roi_path,
                   "est_map_sha256": sha256_file(est_map_path) if est_map_path and os.path.isfile(est_map_path) else None,
                   "ref_map_sha256": sha256_file(ref_map_path) if ref_map_path and os.path.isfile(ref_map_path) else None,
                   "roi_sha256": sha256_file(roi_path) if roi_path and os.path.isfile(roi_path) else None},
        "run_name": run_name, "sequence": sequence,
        "est_points_in": int(len(est_pts)), "ref_points_in": int(len(ref_pts)),
        "sampling": {
            "stat_voxel_m": float(stat_voxel),
            "grid_origin_est_m": [float(x) for x in grid_origin_est],
            "grid_origin_ref_m": [float(x) for x in grid_origin_ref],
            "est_grid_intentionally_offset_by": "half voxel (degeneracy guard)",
            "seed": int(seed),
            "sampler": "nearest input point to each occupied grid-cell centre "
                       "(deterministic; ties -> lowest index)",
            "sampled_over": "the WHOLE estimated map (not a region-cropped subset): region "
                            "membership is applied afterwards and reported",
            "weighting": "none (uniform); no distance/target weighting applied",
        },
        "normals": {
            "estimator": "PCA over k nearest reference points within a fixed radius, queried AT the "
                         "reference neighbour position",
            "k": int(normal_k), "radius_m": float(normal_radius), "seed": int(seed),
            "orientation_sensitivity": "none -- residual uses |d . n|",
            "source_cloud": "FULL reference cloud (never the estimate)",
        },
        "support_radius_m": float(support_radius),
        "coverage_floor": float(coverage_floor),
        "min_denominator": int(min_denominator),
        "unknown_no_reference_max_frac": float(unknown_no_reference_max_frac),
        "notes": notes,
        "warnings": [],
        "unknown_states": [],
    }

    # -- transform / frames ------------------------------------------------
    rec = transform_record or {}
    assert_not_per_block(rec)
    T = np.eye(4)
    if T_manager_ref_est is not None:
        T = assert_se3(np.asarray(T_manager_ref_est, float))   # rigidity FIRST
    ok_T, why_T = validate_transform_record(rec, T_manager_ref_est is not None)
    if not ok_T:
        res.update({"status": "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT", "pass": None,
                    "primary_metric": None, "blocker": why_T,
                    "alignment": {"applied": "rejected: %s" % why_T}})
        return res
    transform_spatial_overlap = None
    res["transform_record"] = None if T_manager_ref_est is None else {
        "matrix_ref_est": [[float(v) for v in row] for row in T],
        "source": rec.get("source"),
        "source_artifact": rec.get("source_artifact"),
        "source_artifact_sha256": rec.get("source_artifact_sha256"),
        "independent_of_evaluation": rec.get("independent_of_evaluation"),
        "frame_from": rec.get("frame_from"),
        "frame_to": rec.get("frame_to"),
        "fit_region": rec.get("fit_region"),
        "eval_region": rec.get("eval_region"),
        "provenance_disjointness": provenance_disjoint(rec)[1],
        "fit_region_eval_region_spatially_overlapping": regions_may_intersect(
            rec.get("fit_region"), rec.get("eval_region")),
        "spatial_overlap_is_not_a_leakage_judgement": True,
    }
    transform_spatial_overlap = (res["transform_record"] or {}).get(
        "fit_region_eval_region_spatially_overlapping")
    if T_manager_ref_est is not None and transform_spatial_overlap:
        res["warnings"].append(
            "fit_region and eval_region overlap spatially; independence was judged on sample/run/"
            "time-index provenance, not on the spatial footprint (re-visiting the same space with "
            "an independently calibrated transform is legitimate)")
    res["alignment"] = {
        "applied": "SE(3) from a validated sourced transform record" if T_manager_ref_est is not None
                   else "identity (no transform applied)",
        "translation_m": float(np.linalg.norm(T[:3, 3])),
        "rotation_deg": float(rot_angle_deg(T[:3, :3])),
        "scale": float(np.cbrt(abs(np.linalg.det(T[:3, :3])))),
        "per_block": False,
        "forbidden_ops_used": [],
    }
    assert abs(res["alignment"]["scale"] - 1.0) < 1e-4
    est_T = apply_se3(T, est_pts) if T_manager_ref_est is not None else est_pts

    # -- ROI ---------------------------------------------------------------
    roi_ok, roi_reason = validate_roi(roi)
    roi_diagnostic = bool(((roi or {}).get("provenance") or {}).get("diagnostic_only", False))
    res["roi"] = {"supplied": roi is not None, "valid": roi_ok, "invalid_reason": roi_reason,
                  "roi_id": (roi or {}).get("roi_id"), "frame": (roi or {}).get("frame"),
                  "provenance": (roi or {}).get("provenance"),
                  "diagnostic_only": roi_diagnostic}
    if roi is None:
        res.update({"status": "BLOCKED_PREDEFINED_ROI_MISSING", "pass": None,
                    "primary_metric": None,
                    "blocker": ("no pre-defined static support region was supplied. The common "
                                "support region MUST come from reference coverage / observation "
                                "conditions (roi_schema.json); it must never be picked after "
                                "seeing the residuals. Owner: DatasetQualification.")})
        return res
    if not roi_ok:
        res.update({"status": "BLOCKED_ROI_UNSOURCED", "pass": None, "primary_metric": None,
                    "blocker": "ROI rejected: %s" % roi_reason})
        return res
    if len(ref_pts) < 10:
        res.update({"status": "NOT_MEASURABLE_NO_REFERENCE_IN_SUPPORT", "pass": None,
                    "primary_metric": None,
                    "blocker": "the reference map has (almost) no points"})
        return res

    # -- fixed membership: sample the WHOLE estimate on the frozen grid ----
    org_est = (np.asarray(grid_origin_est, float)
               + np.asarray(half_voxel_offset(stat_voxel), float))
    org_ref = np.asarray(grid_origin_ref, float)
    i_all = voxel_grid_select(est_T, stat_voxel, org_est)
    est_all = est_T[i_all]
    n_all = int(len(est_all))
    in_roi = roi_mask(roi, est_all)
    res["membership"] = {
        "definition": "the estimated map sampled on the frozen grid over the WHOLE map; the "
                      "frozen region then selects the evaluation subset. The sampling is "
                      "independent of every residual and of the reference cloud.",
        "est_sampled_total_n": n_all,
        "est_in_region_n": int(in_roi.sum()),
        "est_outside_region_n": int((~in_roi).sum()),
        "est_outside_region_fraction": (float((~in_roi).mean()) if n_all else None),
        "membership_hash": sha256_obj({"voxel": float(stat_voxel),
                                       "origin_est": [float(x) for x in org_est],
                                       "n_total": n_all, "n_in_region": int(in_roi.sum())}),
    }
    est_s = est_all[in_roi]
    n_fixed = int(len(est_s))
    res["sampling"]["grid_origin_est_effective_m"] = [float(x) for x in org_est]
    res["sampling"]["grid_origin_ref_effective_m"] = [float(x) for x in org_ref]
    res["sampling"]["denominator_fixed_n"] = n_fixed
    res["sampling"]["ref_sample_n"] = int(len(voxel_grid_select(ref_pts, stat_voxel, org_ref)))

    if n_fixed < min_denominator:
        res.update({"status": "NOT_MEASURABLE_TOO_FEW_POINTS", "pass": None, "primary_metric": None,
                    "blocker": ("fixed denominator %d < min_denominator %d (of which %d sampled "
                                "estimate points fall outside the frozen region)"
                                % (n_fixed, min_denominator, int((~in_roi).sum())))})
        return res
    # R38: every sampled estimate point outside the frozen region must be a MEMBER
    # of an independently pre-declared set.  Membership is MEASURED against the
    # geometry the --outside-attribution record itself declares (include/exclude
    # region algebra); the record's declarative flags alone attribute nothing.
    # There is NO tolerance fraction and no bound tuned to the estimate's own
    # point count: any point the declared geometry does not cover stays an
    # unknown residual and blocks an overall PASS, while the raw value and the
    # exclusion counts stay reported (fail-closed).
    outside_n = int((~in_roi).sum())
    att = outside_attribution or {}
    att_flags_ok = bool(
        bool(att.get("source"))
        and bool(att.get("source_artifact") or att.get("source_artifact_sha256"))
        and att.get("independent_of_estimate") is True
        and att.get("independent_of_error") is True)
    member_spec, member_reason = parse_member_region(att, roi.get("frame")) if att else (
        None, "no --outside-attribution record supplied")
    outside_pts = est_all[~in_roi]
    covered_mask = member_region_mask(member_spec, outside_pts) if member_spec is not None \
        else np.zeros(outside_n, dtype=bool)
    covered_n = int(covered_mask.sum())
    attributed_n = covered_n if att_flags_ok else 0
    residual_n = outside_n - attributed_n
    if outside_n == 0:
        att_ok = True
    else:
        att_ok = bool(att_flags_ok and member_spec is not None and residual_n == 0)
    res["membership"].update({
        "outside_region_attribution": {
            "required": True,
            "attributed": bool(att_ok),
            "record": {k: att.get(k) for k in ("schema", "source", "source_artifact",
                                               "source_artifact_sha256",
                                               "independent_of_estimate", "independent_of_error",
                                               "member_ids", "notes")} if att else None,
            "record_flags_ok": bool(att_flags_ok),
            "member_region": None if member_spec is None else {
                "schema": member_spec.get("schema") or MEMBER_REGION_SCHEMA,
                "frame": member_spec.get("frame"),
                "eval_frame_checked_against": roi.get("frame"),
                "frame_matches_evaluation_frame": True,
                "set_operation": member_spec.get("set_operation"),
                "n_include_boxes": member_spec["n_include"],
                "n_exclude_boxes": member_spec["n_exclude"],
                "geometry_sha256": member_spec["geometry_sha256"],
                "measured_against": "the sampled estimate points outside the frozen region",
            },
            "member_region_unusable_reason": member_reason,
            "measured": {
                "outside_n": outside_n,
                "covered_by_member_region_n": covered_n,
                "covered_by_member_region_fraction": (float(covered_n) / outside_n
                                                      if outside_n else None),
                "residual_uncovered_n": residual_n,
                "residual_uncovered_fraction": (float(residual_n) / outside_n
                                                if outside_n else None),
                "residual_is_a_decisive_unknown": bool(residual_n > 0),
                "declared_member_geometry_is_measured_not_taken_on_trust": True,
            },
            "rule": "fail-closed: only sampled points inside the member geometry the record "
                    "itself declares are attributed (include-box union MINUS exclude-box union); "
                    "a record without usable geometry, or any point the declared geometry does "
                    "not cover, leaves an unknown residual and blocks an overall PASS. No "
                    "tolerance fraction and no bound tuned to the estimate's point count.",
        },
        "attributed_outside_n": int(attributed_n),
        "unattributed_outside_n": int(residual_n),
    })

    # -- support classification against the FULL reference -----------------
    tree_ref_full = cKDTree(ref_pts, compact_nodes=True, balanced_tree=True, leafsize=32)
    d_nn, idx_nn = tree_ref_full.query(est_s, workers=-1)
    cell_tree, _ = reference_occupancy_cell_tree(ref_pts, stat_voxel, org_ref)
    no_reference_radius_m = max(2.0 * stat_voxel, 10.0 * support_radius)
    supported, drifted, no_ref = support_classification(
        d_nn, cell_tree, est_s, stat_voxel, org_ref, support_radius, no_reference_radius_m)

    # ---- REFERENCE-ANCHORED membership (R38) ----------------------------
    # The frozen region + the reference cloud define a PRE-DECLARED member set that
    # no movement of the estimate can shrink: for every reference grid sample in
    # the region we ask whether an estimate sample is present.  This is the set
    # that makes "delete/relocate the offending points" unable to buy a PASS.
    ref_in_roi = ref_pts[roi_mask(roi, ref_pts)]
    ref_s = ref_in_roi[voxel_grid_select(ref_in_roi, stat_voxel, org_ref)] if len(ref_in_roi) else ref_in_roi
    ref_anchor = {
        "predeclared_membership_n": int(len(ref_s)),
        "definition": "reference grid samples inside the frozen region: fixed before any run and "
                      "independent of where the estimate went",
        "member_hash": sha256_obj({"voxel": float(stat_voxel),
                                   "origin_ref": [float(x) for x in org_ref],
                                   "n_anchor": int(len(ref_s))}),
    }
    if len(ref_s):
        est_cell_tree, _ = reference_occupancy_cell_tree(est_s, stat_voxel, org_est)
        d_r2e, i_r2e = cKDTree(est_s, compact_nodes=True, balanced_tree=True).query(ref_s, workers=-1)
        r_sup, r_drift, r_absent = support_classification(
            d_r2e, est_cell_tree, ref_s, stat_voxel, org_est, support_radius, no_reference_radius_m)
        n_ref_anchor = int(len(ref_s))
        ref_cov = float(r_sup.mean())
        n_ref_anchor_supported = int(r_sup.sum())
        n_ref_drifted = int(r_drift.sum())
        n_ref_est_absent = int(r_absent.sum())
        n_ref_est_absent_frac = float(r_absent.mean())
        n_ref_drifted_frac = float(r_drift.mean())
        n_ref_drifted_frac = float(r_drift.mean())
        n_ref_drifted_frac = float(r_drift.mean())
        n_anchor_normals, _ = estimate_normals(ref_pts, ref_s, k=normal_k,
                                               radius=normal_radius, tree=tree_ref_full)
        delta_r = est_s[i_r2e] - ref_s
        d_plane_r = np.abs(np.einsum("ij,ij->i", delta_r, n_anchor_normals))
        ref_anchor.update({
            "supported_n": n_ref_anchor_supported,
            "drifted_n": n_ref_drifted,
            "drifted_fraction": n_ref_drifted_frac,
            "estimate_absent_locally_n": n_ref_est_absent,
            "estimate_absent_locally_fraction": n_ref_est_absent_frac,
            "coverage_of_reference_members": ref_cov,
            "p2pl_stats": dict(st(d_plane_r, "p2pl_ref_anchored"),
                               p2pl_ref_anchored_p99=float(np.percentile(d_plane_r, 99)),
                               p2pl_ref_anchored_max=float(np.max(d_plane_r))),
            "note": "the reference-anchored residual is the point-to-plane distance of the NEAREST "
                    "estimate sample to the pre-declared reference member; a region the estimate "
                    "abandoned shows up as drift or as 'estimate absent', never as a deletion",
        })
        if n_ref_est_absent:
            ref_anchor["estimate_absent_note"] = ("no estimate sample within %.1f m: 'the estimate "
                                                 "drifted away' and 'the estimate never covered "
                                                 "here' cannot be distinguished"
                                                 % no_reference_radius_m)
    else:
        r_sup, r_drift, r_absent = (np.empty(0, bool),) * 3
        ref_cov = 0.0
        n_ref_est_absent_frac = 0.0
        ref_anchor.update({"supported_n": 0, "drifted_n": 0,
                           "estimate_absent_locally_n": 0,
                           "estimate_absent_locally_fraction": None,
                           "coverage_of_reference_members": 0.0, "p2pl_stats": None})
    res["reference_anchored"] = ref_anchor
    est_cov = float(supported.mean())

    res["support"] = {
        "definition": "an estimated sample has reference support when its nearest neighbour in the "
                      "FULL reference cloud is within support_radius",
        "nearest_reference_searched_in": "the FULL reference cloud (a region-cropped search would "
                                         "hide exactly the drifted-out samples)",
        "supported_n": int(supported.sum()),
        "drifted_out_of_support_n": int(drifted.sum()),
        "drifted_out_of_support_fraction": float(drifted.mean()),
        "no_reference_locally_n": int(no_ref.sum()),
        "no_reference_locally_fraction": float(no_ref.mean()),
        "no_reference_radius_m": float(no_reference_radius_m),
        "no_reference_locally_definition": "the reference has no data within no_reference_radius_m "
                                          "(= 10 x support radius, floored at 2 voxels): 'the "
                                          "estimate drifted' and 'the reference is absent here' "
                                          "cannot be distinguished -> decisive unknown",
        "drifted_note": "drifted-out samples are LARGE RESIDUALS and are kept in the primary metric",
        "no_correspondence_est_n": int((~supported).sum()),
        "no_correspondence_est_fraction": float((~supported).mean()),
        "reference_coverage_of_est": est_cov,
        "estimate_coverage_of_reference": ref_cov,
        "reference_coverage_of_est_note": "fraction of the FIXED DENOMINATOR with reference support",
        "estimate_coverage_of_reference_note": "fraction of the PRE-DECLARED reference members that "
                                               "have an estimate sample within support_radius",
    }

    # -- primary metric ----------------------------------------------------
    # normals at the REFERENCE neighbour of every evaluated sample (same query position
    # as the strict cross-check; never at the estimate position)
    ref_nn_pts = ref_pts[idx_nn]
    n_hat, n_hat_deg = estimate_normals(ref_pts, ref_nn_pts, k=normal_k,
                                        radius=normal_radius, tree=tree_ref_full)
    res["normals"]["degenerate_at_nn_n"] = int(n_hat_deg)

    delta = est_s - ref_nn_pts
    d_signed = np.einsum("ij,ij->i", delta, n_hat)
    d_plane = np.abs(d_signed)

    bracket = truncation_bracket(d_plane, "p2pl", truncate_at)
    raw_rmse = bracket["raw"]["p2pl_rmse"]
    primary_value = raw_rmse           # R01: truncation can never be the headline

    res["est_to_ref_point_to_plane"] = dict(bracket["raw"], **{
        "p2pl_frac_within_0p02": frac_within(d_plane, 0.02),
        "p2pl_frac_within_0p05": frac_within(d_plane, 0.05),
        "p2pl_signed_mean": float(np.mean(d_signed)),
        "p2pl_signed_median": float(np.median(d_signed)),
    })
    res["est_to_ref_point_to_point"] = dict(
        st(d_nn, "p2p"), p2p_frac_within_0p02=frac_within(d_nn, 0.02),
        p2p_frac_within_0p05=frac_within(d_nn, 0.05))
    res["ref_to_est_point_to_point"] = dict(
        st(d_r2e if len(ref_s) else np.empty(0), "p2p_rev"),
        p2p_rev_frac_within_0p02=frac_within(d_r2e, 0.02) if len(ref_s) else None,
        p2p_rev_frac_within_0p05=frac_within(d_r2e, 0.05) if len(ref_s) else None)

    # R06: coincidence is a 3-D point coincidence (C2C), never a zero plane residual
    n_c2c = int(np.sum(d_nn <= E_EXACT_COINCIDENCE_M))
    degeneracy_fraction = float(n_c2c) / n_fixed
    zero_plane_fraction = float(np.mean(d_plane <= E_EXACT_COINCIDENCE_M))
    # R06 parallel check: the points the plane-residual test flags as degenerate
    # are only truly coincident if their 3-D distance to the nearest reference
    # point is below E_EXACT_COINCIDENCE_C2C_M.  p2pl ~ 0 alone can be produced by
    # a purely tangential offset, so the coincidence flag is decided on C2C.
    zero_plane_mask = d_plane <= E_EXACT_COINCIDENCE_M
    cand_c2c = d_nn[zero_plane_mask]
    exact_coincidence_c2c_m = float(np.median(cand_c2c)) if cand_c2c.size else None
    exact_coincidence = bool(cand_c2c.size
                             and exact_coincidence_c2c_m < E_EXACT_COINCIDENCE_C2C_M)
    res["brackets"] = {
        "fixed_denominator_definition": "estimated-map samples on the frozen grid inside the frozen "
                                        "region, selected BEFORE any truncation / exclusion",
        "point_to_plane": bracket,
        "point_to_point": truncation_bracket(d_nn, "p2p", truncate_at),
        "raw_and_truncated_both_reported": True,
        "truncation_cannot_promote_a_pass": True,
        "truncation_default": "none (the headline number is the RAW rmse over the fixed "
                              "denominator; --truncate-at only adds a diagnostic bracket)",
        "degeneracy": {
            "definition": "3-D point coincidence: fraction of the fixed denominator whose C2C "
                          "distance to the nearest reference point is <= 1e-6 m",
            "exact_coincidence_c2c_n": n_c2c,
            "exact_coincidence_c2c_fraction": degeneracy_fraction,
            "exact_coincidence": exact_coincidence,
            "exact_coincidence_c2c_m": exact_coincidence_c2c_m,
            "exact_coincidence_c2c_basis": "median 3-D distance of the points the plane-residual "
                                           "test flags as degenerate, measured to the nearest "
                                           "reference point; the flag is set only when that "
                                           "distance is below exact_coincidence_c2c_threshold_m, "
                                           "never on p2pl ~ 0 alone",
            "exact_coincidence_c2c_threshold_m": E_EXACT_COINCIDENCE_C2C_M,
            "zero_plane_residual_fraction": zero_plane_fraction,
            "zero_plane_residual_note": "a zero point-to-plane residual only means the normal "
                                        "matched; it is NOT a coincidence and must never be used "
                                        "as a degeneracy signal",
            "hard_block_fraction": float(max_degenerate_frac),
            "warn_fraction": float(DEGENERACY_WARN_FRAC),
        },
    }

    # -- thin structure / double wall (vectorised) -------------------------
    res["thin_structure"] = {
        "note": "a 5 cm claim must survive double-wall / thin structures, which a voxel resolution "
                "floor cannot resolve",
        "reference_plane_thickness": plane_thickness(ref_in_roi, k=normal_k, seed=seed)
        if len(ref_in_roi) else None,
        "estimated_plane_thickness": plane_thickness(est_s, k=normal_k, seed=seed),
        "voxel_resolution_note": "stat voxel = %.3f m; the metric is NOT a resolution floor" % stat_voxel,
    }
    est_normals, est_normal_deg = estimate_normals(est_s, est_s, k=normal_k, radius=normal_radius)
    kk = int(min(8, len(ref_pts)))
    d_k, i_k = tree_ref_full.query(est_s, k=kk, distance_upper_bound=0.5, workers=-1)
    dw_frac = None
    if np.ndim(i_k) == 2:
        valid_k = np.isfinite(d_k) & (i_k < len(ref_pts))
        nb = ref_pts[np.where(valid_k, i_k, 0)]
        s = np.einsum("ijk,ik->ij", nb - est_s[:, None, :], est_normals)
        both = np.zeros(len(est_s), dtype=bool)
        rows = np.where(valid_k.any(axis=1))[0]
        for i in rows:
            sv = s[i][valid_k[i]]
            both[i] = bool(sv.min() <= -0.02 and sv.max() >= 0.02)
        dw_frac = float(np.mean(both))
    res["thin_structure"]["double_wall_suspect_fraction"] = dw_frac
    res["thin_structure"]["double_wall_flag"] = bool(dw_frac is not None and dw_frac > 0.05)
    res["thin_structure"]["double_wall_definition"] = (
        "fraction of estimated samples with reference neighbours on BOTH sides of the sample's own "
        "tangent plane by >= 0.02 m within 0.5 m: a single surface collapsed between two walls")
    res["thin_structure"]["estimated_normal_degenerate_n"] = int(est_normal_deg)

    # -- regional / normal breakdown ---------------------------------------
    res["regional"] = {}
    if roi.get("type") == "aabb":
        lo = np.asarray(roi["bounds"]["min"], float)
        hi = np.asarray(roi["bounds"]["max"], float)
    else:
        lo, hi = est_s.min(axis=0), est_s.max(axis=0)
    mid = 0.5 * (lo + hi)
    key = ((est_s[:, 0] >= mid[0]).astype(int) * 4 + (est_s[:, 1] >= mid[1]).astype(int) * 2
           + (est_s[:, 2] >= mid[2]).astype(int))
    for c in range(8):
        m = key == c
        if m.sum() == 0:
            continue
        res["regional"]["octant_%d%d%d" % ((c >> 2) & 1, (c >> 1) & 1, c & 1)] = dict(
            st(d_plane[m], "p2pl"), p2pl_frac_within_0p05=frac_within(d_plane[m], 0.05))
    res["regional_note"] = ("fixed 2x2x2 partition of the ROI bounds; a global RMSE can hide a "
                            "locally bent region, the per-cell max/p99 cannot")
    axis = np.argmax(np.abs(n_hat), axis=1)
    res["normal_direction"] = {}
    for a, nm in enumerate(("x", "y", "z")):
        m = axis == a
        if m.sum() == 0:
            continue
        res["normal_direction"][nm] = dict(st(d_plane[m], "p2pl_%s" % nm),
                                          n_frac=float(m.mean()))
    res["normal_direction_note"] = ("reference-normal dominant axis classes; one wall set can look "
                                    "good globally while another is off")
    worst = None
    for k, v in res["regional"].items():
        if v.get("p2pl_p99") is None:
            continue
        if worst is None or v["p2pl_p99"] > worst[1]:
            worst = (k, v["p2pl_p99"])
    res["regional"]["worst_region"] = None if worst is None else worst[0]
    res["regional"]["worst_region_p2pl_p99"] = None if worst is None else worst[1]

    # -- conditional ICP diagnostic ----------------------------------------
    res["icp"] = {"mode": (icp or {}).get("mode", "off"), "applied": False}
    registration_dependent = False
    if icp and icp.get("mode") == "conditional":
        T_icp, log = conditional_icp(est_pts, ref_pts, icp.get("voxels", [0.5, 0.2, 0.1]),
                                     icp.get("max_dists", [1.0, 0.4, 0.15]), icp.get("max_iter", 60))
        res["icp"] = {"mode": "conditional", "applied": T_icp is not None, "log": log}
        if T_icp is not None:
            assert_se3(T_icp)
            moved = (float(np.linalg.norm(T_icp[:3, 3])) > 1e-3
                     or rot_angle_deg(T_icp[:3, :3]) > 0.05)
            res["icp"].update({
                "transformation": [[float(v) for v in r] for r in T_icp],
                "translation_m": float(np.linalg.norm(T_icp[:3, 3])),
                "rotation_deg": float(rot_angle_deg(T_icp[:3, :3])),
                "identity_sufficient": not moved,
                "verdict": ("registration moved the estimate: DIAGNOSTIC ONLY, cannot PASS"
                            if moved else "identity was sufficient; registration did not move")})
            registration_dependent = moved
        else:
            res["icp"]["verdict"] = "ICP unavailable"

    # -- warnings vs DECISIVE unknowns (R07) -------------------------------
    if degeneracy_fraction > DEGENERACY_WARN_FRAC:
        if degeneracy_fraction > max_degenerate_frac:
            msg = ("%.1f%% of the fixed denominator sits exactly on a reference point (3-D): the "
                   "two sample sets are effectively the same set" % (100.0 * degeneracy_fraction))
        else:
            msg = ("%.1f%% of the fixed denominator coincides with a reference point (3-D): the "
                   "comparison is partly self-referential" % (100.0 * degeneracy_fraction))
        res["unknown_states"].append({"code": "UNKNOWN_SELF_REFERENTIAL_COMPARISON", "detail": msg})
    if worst is not None and worst[1] > REGIONAL_GAP_FACTOR * PRIMARY_THRESHOLD_M:
        res["unknown_states"].append({
            "code": "UNKNOWN_REGIONAL_GAP",
            "detail": ("regional p99 %.3f m in %s exceeds %.1fx the criterion %.2f m: a locally "
                       "bent / thin-structure region cannot be averaged away into an overall claim"
                       % (worst[1], worst[0], REGIONAL_GAP_FACTOR, PRIMARY_THRESHOLD_M))})
    if res["support"]["no_reference_locally_fraction"] > unknown_no_reference_max_frac:
        res["unknown_states"].append({
            "code": "UNKNOWN_SUPPORT_CLASSIFICATION",
            "detail": ("%.1f%% of the fixed denominator has no reference cell nearby: 'drifted out' "
                       "and 'reference absent' cannot be distinguished"
                       % (100.0 * res["support"]["no_reference_locally_fraction"]))})
    if not res["unknown_states"] and res["support"]["no_reference_locally_n"]:
        res["warnings"].append("%d samples (%.2f%%) could not be classified as supported or "
                               "drifted; they are KEPT in the metric"
                               % (res["support"]["no_reference_locally_n"],
                                  100.0 * res["support"]["no_reference_locally_fraction"]))

    # -- verdict -----------------------------------------------------------
    rows_aligned = (est_cov >= coverage_floor) or (T_manager_ref_est is not None)
    res["pass_preconditions"] = {
        "predefined_roi_with_provenance": bool(roi_ok),
        "outside_region_points_attributed": bool(att_ok),
        "transform_independent_source": True,
        "transform_fit_samples_disjoint_from_eval": True,
        "frames_aligned_by_sourced_transform_or_identity": bool(rows_aligned),
        "overlap_coverage_ge_floor": bool(est_cov >= coverage_floor),
        "reference_coverage_ge_floor": bool(ref_cov >= coverage_floor),
        "denominator_ge_min": bool(n_fixed >= min_denominator),
        "sampling_not_degenerate": bool(degeneracy_fraction <= max_degenerate_frac),
        "support_classifiable": bool(
            res["support"]["no_reference_locally_fraction"] <= unknown_no_reference_max_frac),
        "reference_members_classifiable": bool(
            n_ref_est_absent_frac <= unknown_no_reference_max_frac),
        "registration_not_fitted_on_eval_data": bool(not registration_dependent),
    }
    res["verdict"] = {
        "criterion": "est->ref point-to-plane RMSE <= %.2f m on the frozen support region"
                     % PRIMARY_THRESHOLD_M,
        "threshold_m": PRIMARY_THRESHOLD_M,
        "value_m": float(primary_value),
        "raw_value_m": float(raw_rmse),
        "truncated_value_m": (bracket.get("truncated") or {}).get("p2pl_rmse"),
        "truncated_is_diagnostic_only": True,
        "denominator_fixed_n": n_fixed,
        "coverage_within_5cm": frac_within(d_plane, 0.05),
    }

    failed = [k for k, v in res["pass_preconditions"].items() if not v]
    decisive_codes = [u["code"] for u in res["unknown_states"]]
    status, blocker = None, None
    if "outside_region_points_attributed" in failed:
        status = "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
        m_att = res["membership"]["outside_region_attribution"]
        m_meas = m_att["measured"]
        if m_att["member_region"] is None:
            why = "the record declares no usable member geometry (%s)" % m_att["member_region_unusable_reason"]
        elif not m_att["record_flags_ok"]:
            why = "the record's provenance flags are not admissible"
        else:
            why = ("the declared member region covers %.2f%% of them, leaving %d uncovered"
                   % (100.0 * (m_meas["covered_by_member_region_fraction"] or 0.0),
                      m_meas["residual_uncovered_n"]))
        blocker = ("%d of %d sampled estimate points (%.2f%%) lie outside the frozen support "
                   "region and are not attributed to the independently declared member set: %s. "
                   "They cannot be distinguished from drift, so no overall PASS is possible. The "
                   "raw value and the exclusion counts are still reported."
                   % (res["membership"]["unattributed_outside_n"],
                      res["membership"]["est_sampled_total_n"],
                      100.0 * res["membership"]["est_outside_region_fraction"],
                      why))
        res["outside_region_would_be_status"] = ("PASS" if primary_value <= PRIMARY_THRESHOLD_M
                                                 else "FAIL")
    elif "registration_not_fitted_on_eval_data" in failed:
        status = "NOT_PASSABLE_REGISTRATION_FITTED_ON_EVAL_DATA"
        blocker = ("ICP moved the estimate; a registration fitted on the evaluation data cannot "
                   "certify mapping accuracy (conditional diagnostic only)")
    elif "overlap_coverage_ge_floor" in failed or "reference_coverage_ge_floor" in failed:
        if est_cov < 0.05 and T_manager_ref_est is None:
            status = "BLOCKED_FRAMES_UNALIGNED_NO_SOURCED_TRANSFORM"
            blocker = ("the estimated map is not in the reference frame (support coverage %.4f) and "
                       "no sourced transform was supplied" % est_cov)
        else:
            status = "NOT_MEASURABLE_LOW_COVERAGE"
            blocker = ("coverage below floor (est=%.3f ref=%.3f floor=%.3f): an empty / very "
                       "low-overlap comparison can never PASS" % (est_cov, ref_cov, coverage_floor))
    elif "reference_members_classifiable" in failed:
        status = "NOT_MEASURABLE_UNKNOWN_SUPPORT_CLASSIFICATION"
        blocker = ("%.1f%% of the PRE-DECLARED reference members have no estimate sample within the "
                   "nearest-neighbour radius: a region the estimate abandoned cannot be told apart "
                   "from a region it never covered"
                   % (100.0 * n_ref_est_absent_frac))
    elif "support_classifiable" in failed:
        status = "NOT_MEASURABLE_UNKNOWN_SUPPORT_CLASSIFICATION"
        blocker = next((u["detail"] for u in res["unknown_states"]
                        if u["code"] == "UNKNOWN_SUPPORT_CLASSIFICATION"),
                       "support could not be classified")
    elif "sampling_not_degenerate" in failed:
        status = "NOT_MEASURABLE_DEGENERATE_SAMPLING"
        blocker = res["unknown_states"][0]["detail"]
    elif "frames_aligned_by_sourced_transform_or_identity" in failed:
        status = "BLOCKED_FRAMES_UNALIGNED_NO_SOURCED_TRANSFORM"
        blocker = "the frames are not aligned and no sourced transform was supplied"

    would_pass = bool(primary_value <= PRIMARY_THRESHOLD_M)
    if status is None:
        # R07: a decisive unknown must block a PASS.  This check runs before the
        # positive pass gate, so an open unknown always overrides a would-be
        # acceptance.  A plain FAIL stays FAIL (a failure is not an acceptance
        # claim) and the unknowns are listed alongside it.
        if res.get("unknown_states") and would_pass:
            res["status"] = decisive_codes[0]
            res["pass"] = None
            res["blocker"] = res["unknown_states"][0]["detail"]
            res["unknown_gate_note"] = ("a decisive unknown was raised, so an overall acceptance "
                                        "cannot be claimed even though the global RMSE is inside "
                                        "the criterion")
        else:
            res["pass"] = would_pass
            res["status"] = "PASS" if would_pass else "FAIL"
    else:
        res["status"] = status
        res["pass"] = None
        res["blocker"] = blocker

    if roi_diagnostic:
        res["diagnostic_would_be_status"] = res["status"]
        res["diagnostic_would_be_pass"] = res["pass"]
        res["status"] = "DIAGNOSTIC_ONLY_ROI"
        res["pass"] = None
        res["blocker"] = ("the supplied support region is marked diagnostic_only: it is not an "
                          "observation-conditioned pre-defined region and can never carry a PASS. "
                          "The authoritative ROI must come from DatasetQualification.")

    res["primary_metric"] = {
        "name": "map est->ref point-to-plane RMSE on the frozen support region",
        "value_m": float(primary_value),
        "raw_value_m": float(raw_rmse),
        "truncated_value_m": (bracket.get("truncated") or {}).get("p2pl_rmse"),
        "truncation_cannot_promote_a_pass": True,
        "threshold_m": PRIMARY_THRESHOLD_M,
        "denominator_fixed_n": n_fixed,
        "pass": res["pass"],
        "diagnostic_only_roi": roi_diagnostic,
        "auxiliary_metrics_have_no_passing_line": True,
    }
    return res


# -------------------------------------------------------------------- CLI ----
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--est-map", required=True)
    ap.add_argument("--ref-map", required=True)
    ap.add_argument("--roi", default=None)
    ap.add_argument("--stat-voxel", type=float, default=0.05)
    ap.add_argument("--grid-origin", default="0,0,0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--normal-k", type=int, default=15)
    ap.add_argument("--normal-radius", type=float, default=0.15)
    ap.add_argument("--support-radius", type=float, default=SUPPORT_RADIUS_M)
    ap.add_argument("--coverage-floor", type=float, default=DEFAULT_COVERAGE_FLOOR)
    ap.add_argument("--min-denominator", type=int, default=DEFAULT_MIN_DENOMINATOR)
    ap.add_argument("--unknown-no-reference-max-frac", type=float,
                    default=DEFAULT_UNKNOWN_NO_REFERENCE_MAX_FRAC)
    ap.add_argument("--outside-attribution", default=None,
                    help="json {source, source_artifact|source_artifact_sha256, "
                         "independent_of_estimate:true, independent_of_error:true, member_region:"
                         "{schema, frame, derivation, independent_of_estimate:true, "
                         "independent_of_error:true, include:[{type:aabb,bounds:{min,max}}], "
                         "exclude:[...]}} attributing the estimate points outside the frozen "
                         "region to an independently declared member set. The membership is "
                         "MEASURED against the declared include-minus-exclude geometry; the "
                         "flags alone attribute nothing and any uncovered point blocks a PASS")
    ap.add_argument("--truncate-at", type=float, default=None,
                    help="diagnostic truncation threshold (m); the headline stays the raw value")
    ap.add_argument("--transform-json", default=None)
    ap.add_argument("--icp", default="off", choices=["off", "conditional"])
    ap.add_argument("--icp-voxels", default="0.5,0.2,0.1")
    ap.add_argument("--icp-max-dist", default="1.0,0.4,0.15")
    ap.add_argument("--icp-max-iter", type=int, default=60)
    ap.add_argument("--out", required=True)
    ap.add_argument("--run-name", default=None)
    ap.add_argument("--sequence", default=None)
    args = ap.parse_args(argv)

    est = load_points(args.est_map)
    ref = load_points(args.ref_map)
    origin = tuple(float(x) for x in args.grid_origin.split(","))
    roi = load_json(args.roi)
    tjson = load_json(args.transform_json)
    T = np.asarray(tjson["matrix_ref_est"], float) if tjson else None
    icp = {"mode": args.icp,
           "voxels": [float(x) for x in args.icp_voxels.split(",")],
           "max_dists": [float(x) for x in args.icp_max_dist.split(",")],
           "max_iter": args.icp_max_iter}

    res = evaluate_map(
        est, ref, stat_voxel=args.stat_voxel, grid_origin_est=origin, grid_origin_ref=origin,
        seed=args.seed, normal_k=args.normal_k, normal_radius=args.normal_radius,
        support_radius=args.support_radius, roi=roi, T_manager_ref_est=T, transform_record=tjson,
        truncate_at=args.truncate_at, icp=icp, coverage_floor=args.coverage_floor,
        min_denominator=args.min_denominator,
        unknown_no_reference_max_frac=args.unknown_no_reference_max_frac,
        outside_attribution=load_json(args.outside_attribution),
        est_map_path=args.est_map, ref_map_path=args.ref_map, roi_path=args.roi,
        run_name=args.run_name, sequence=args.sequence)
    dump_json(args.out, res)
    print("status=%s pass=%s value_m=%s denominator=%s coverage_est=%s unknown=%s"
          % (res["status"], res.get("pass"),
             None if not res.get("primary_metric") else round(res["primary_metric"]["value_m"], 4),
             res["sampling"].get("denominator_fixed_n"),
             None if "support" not in res else round(res["support"]["reference_coverage_of_est"], 4),
             [u["code"] for u in res.get("unknown_states", [])]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
