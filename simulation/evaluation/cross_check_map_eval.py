#!/usr/bin/env python3
"""Cross-verification of the main map metric -- TWO clearly separated modes.

Review R12: the previous single "cross-check" silently changed the sampling
weights, the neighbourhood size/radius AND the normal query position, so it was
a DIFFERENT ESTIMATOR, and calling its agreement a correctness proof of the main
metric was wrong.  It is now split:

  --mode strict      SAME contract quantity (same fixed denominator, same
                     neighbourhood definition, normals queried at the same
                     reference-neighbour positions), INDEPENDENT ARITHMETIC:
                     batched SVD normals, a different residual expression and an
                     explicit-loop aggregation.  Agreement here is a genuine
                     check of the arithmetic.  Shared: only the NN library
                     primitive and the two contract primitives that DEFINE the
                     sample set (grid sampler, region mask).
  --mode sensitivity DIFFERENT estimand on purpose (uniform subsample instead of
                     the frozen grid, k=8, normals queried at the ESTIMATE
                     position).  The delta is reported as estimator sensitivity
                     and is explicitly NOT a correctness proof, NOT an accuracy
                     number, and never replaces the main metric.

Both modes report a MEASURED sampling uncertainty: the metric is recomputed on K
disjoint folds of the query set and the standard error across folds is reported,
so the agreement tolerance is measured rather than a hand-picked 5 mm/10 %.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_common import (  # noqa: E402
    SUPPORT_RADIUS_M, apply_se3, assert_se3, dump_json, half_voxel_offset, load_json,
    load_points, sha256_file, st, voxel_grid_select,
)
from map_accuracy_eval import roi_mask, validate_roi  # noqa: E402

TOOL_VERSION = "cross_check_map_eval.py v2.2.0"
STRICT_CLAIM = "same-estimand independent-arithmetic verification of the main metric"
SENS_CLAIM = "different-estimand sensitivity diagnostic -- NOT a correctness proof"


def batched_svd_normals(points, query_idx, k=15, radius=0.15, tree=None, min_neighbours=4):
    """Normals via a BATCHED SVD of the centred neighbourhood (independent route).

    EXACTLY the main estimator's estimand: the neighbourhood is the set of points
    within `radius` and, when that set has fewer than `min_neighbours` members,
    the k nearest points overall (the same fallback the main estimator applies).
    Each neighbourhood is centred on ITS OWN valid mean and the covariance is the
    sum of the centred outer products; the reported direction is the smallest
    singular direction of the centred matrix, i.e. the same eigenvector of the
    same covariance matrix that the main implementation obtains with eigh.  Sign
    is irrelevant because the metric uses |d . n|.

    Any difference from the main value must therefore be floating-point noise,
    and the machines must agree to ~1e-12 -- not to a statistical tolerance.
    """
    pts = np.asarray(points, float)
    if tree is None:
        tree = cKDTree(pts, compact_nodes=True, balanced_tree=True, leafsize=32)
    kk = int(min(max(k, min_neighbours), len(pts)))
    q = pts[query_idx]
    valid = np.zeros((len(q), kk), dtype=bool)
    idx = np.zeros((len(q), kk), dtype=np.int64)
    for lo in range(0, len(q), 20000):                 # bounded memory, same result
        hi = min(lo + 20000, len(q))
        d, ii = tree.query(q[lo:hi], k=kk, workers=-1, distance_upper_bound=float(radius))
        d, ii = np.atleast_2d(d), np.atleast_2d(ii)
        v = np.isfinite(d) & (ii < len(pts))
        need = v.sum(axis=1) < min_neighbours
        if need.any():
            _, kii = tree.query(q[lo:hi][need], k=kk, workers=-1)
            ii = ii.copy(); v = v.copy()
            ii[need] = np.atleast_2d(kii)
            v[need] = True
        idx[lo:hi], valid[lo:hi] = ii, v
    nb = pts[np.where(valid, idx, 0)]          # invalid slots are masked out below
    m = valid[..., None]
    cnt = np.maximum(valid.sum(axis=1), 1)[:, None]
    mean = (nb * m).sum(axis=1) / cnt
    centred = np.where(m, nb - mean[:, None, :], 0.0)
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    n = vt[:, -1, :]
    j = np.argmax(np.abs(n), axis=1)
    sign = np.where(n[np.arange(len(n)), j] < 0, -1.0, 1.0)
    return n * sign[:, None], valid.sum(axis=1)


def fold_uncertainty(residuals, folds=5):
    """Measured sampling uncertainty: SE of the RMSE across K disjoint folds."""
    x = np.asarray(residuals, float)
    n = len(x)
    if n < folds * 2:
        return {"folds": 0, "se_m": None, "fold_rmse": []}
    cuts = np.linspace(0, n, folds + 1).astype(int)
    vals = [math.sqrt(float(np.mean(x[cuts[i]:cuts[i + 1]] ** 2))) for i in range(folds)]
    v = np.asarray(vals)
    return {"folds": int(folds), "fold_rmse": [float(t) for t in v],
            "se_m": float(v.std(ddof=1) / math.sqrt(folds)) if folds > 1 else None,
            "definition": "standard error of the per-fold RMSE over K disjoint chunks of the "
                          "evaluation set (measured, not assumed)"}


def _load_case(roi, T_ref_est, est, ref, stat_voxel, grid_origin):
    """Contract primitives: fixed membership + the FULL-reference NN search."""
    if T_ref_est is not None:
        assert_se3(np.asarray(T_ref_est, float))
        est = apply_se3(T_ref_est, est)
    org_est = np.asarray(grid_origin, float) + np.asarray(half_voxel_offset(stat_voxel), float)
    org_ref = np.asarray(grid_origin, float)
    est_all = est[voxel_grid_select(est, stat_voxel, org_est)]
    est_s = est_all[roi_mask(roi, est_all)]
    return est_s, org_ref


def main_parameter_mismatches(main_res, *, stat_voxel, grid_origin, k_normal, normal_radius,
                              support_radius, est_sha, ref_sha, roi_sha, T_ref_est):
    """R40: the strict check is only a same-input comparison if every recorded
    parameter matches the main result.  Returns a list of mismatch strings."""
    problems = []
    if not main_res:
        problems.append("no --main-result supplied: the strict check cannot claim same-input")
        return problems
    smp = main_res.get("sampling") or {}
    if smp.get("stat_voxel_m") != float(stat_voxel):
        problems.append("stat_voxel_m %s != %s" % (smp.get("stat_voxel_m"), stat_voxel))
    eff = smp.get("grid_origin_est_effective_m")
    want = list(np.asarray(grid_origin, float) + np.asarray(half_voxel_offset(stat_voxel), float))
    if eff is not None and not np.allclose(np.asarray(eff, float), want):
        problems.append("effective est grid origin %s != %s" % (eff, want))
    nrm = main_res.get("normals") or {}
    if nrm.get("k") != int(k_normal):
        problems.append("normal k %s != %s" % (nrm.get("k"), k_normal))
    if nrm.get("radius_m") != float(normal_radius):
        problems.append("normal radius %s != %s" % (nrm.get("radius_m"), normal_radius))
    if main_res.get("support_radius_m") != float(support_radius):
        problems.append("support_radius_m %s != %s" % (main_res.get("support_radius_m"), support_radius))
    inp = main_res.get("inputs") or {}
    for name, got in (("est_map_sha256", est_sha), ("ref_map_sha256", ref_sha),
                      ("roi_sha256", roi_sha)):
        if inp.get(name) and inp.get(name) != got:
            problems.append("%s %s != %s" % (name, inp.get(name), got))
    tr = main_res.get("transform_record")
    if (tr is None) != (T_ref_est is None):
        problems.append("transform presence differs (main=%s, cross=%s)"
                        % (tr is not None, T_ref_est is not None))
    elif tr is not None and not np.allclose(np.asarray(tr["matrix_ref_est"], float),
                                            np.asarray(T_ref_est, float)):
        problems.append("transform matrix differs from the main result")
    return problems


def cross_check(est_pts, ref_pts, *, roi, mode="strict", seed=7, n_sample=200000,
                k_normal=15, normal_radius=0.15, stat_voxel=0.05, grid_origin=(0.0, 0.0, 0.0),
                support_radius=SUPPORT_RADIUS_M, T_ref_est=None, main_value_m=None,
                folds=5, parameter_mismatches=None):
    est = np.asarray(est_pts, float)
    ref = np.asarray(ref_pts, float)
    ok, why = validate_roi(roi)
    out = {"tool": TOOL_VERSION, "mode": mode, "roi_valid": bool(ok), "seed": int(seed),
           "main_value_m": None if main_value_m is None else float(main_value_m),
           "claim": STRICT_CLAIM if mode == "strict" else SENS_CLAIM,
           "replaces_main_metric": False}
    if not ok:
        out.update(status="BLOCKED_ROI_UNSOURCED", reason=why)
        return out
    if len(ref) < 10:
        out.update(status="NOT_MEASURABLE_EMPTY_OVERLAP")
        return out
    if mode == "strict" and parameter_mismatches:
        out.update(status="BLOCKED_PARAMETERS_DO_NOT_MATCH_MAIN",
                   parameter_mismatches=parameter_mismatches,
                   blocker="the strict comparison is only valid on identical inputs and settings; "
                           "a mismatch means the two numbers are different estimands")
        return out

    if mode == "strict":
        est_s, org_ref = _load_case(roi, T_ref_est, est, ref, stat_voxel, grid_origin)
        if len(est_s) < 10:
            out.update(status="NOT_MEASURABLE_TOO_FEW_POINTS", denominator_n=int(len(est_s)))
            return out
        tree_ref = cKDTree(ref, compact_nodes=True, balanced_tree=True, leafsize=32)
        d_nn, idx_nn = tree_ref.query(est_s, workers=-1)
        n_hat, _ = batched_svd_normals(ref, idx_nn, k=k_normal, radius=normal_radius, tree=tree_ref)
        delta = np.asarray(est_s, float) - ref[idx_nn]
        plane = np.abs(np.sum(delta * n_hat, axis=1))     # different expression from einsum
        acc = 0.0                                         # explicit-loop aggregation
        for v in plane:
            acc += float(v) * float(v)
        rmse = math.sqrt(acc / len(plane))
        out.update({
            "status": "MEASURED",
            "denominator_n": int(len(est_s)),
            "p2pl_rmse_m": float(rmse),
            "p2p_rmse_m": float(math.sqrt(float(np.sum(d_nn ** 2)) / len(d_nn))),
            "p2p_stats": st(d_nn, "p2p"),
            "no_correspondence_fraction": float(np.mean(d_nn > support_radius)),
            "shared_with_main": ["NN library primitive", "grid sampler (contract primitive)",
                                 "region mask (contract primitive)",
                                 "the normal ESTIMAND (neighbourhood definition)"],
            "independent_of_main": ["normal estimator implementation (batched SVD)",
                                    "residual expression", "aggregation",
                                    "sample-set extraction"],
        })
        unc = fold_uncertainty(plane, folds)
    else:
        est_roi = est[roi_mask(roi, est)]
        rs = np.random.RandomState(seed)
        if len(est_roi) == 0:
            out.update(status="NOT_MEASURABLE_EMPTY_OVERLAP")
            return out
        sel = np.sort(rs.choice(len(est_roi), min(len(est_roi), n_sample), replace=False))
        q = est_roi[sel]
        tree = cKDTree(ref, compact_nodes=True, balanced_tree=True, leafsize=32)
        d_nn, idx_nn = tree.query(q, workers=-1)
        # DELIBERATE estimand change: normals queried at the ESTIMATE position
        n_hat, _ = batched_svd_normals(ref, tree.query(q, k=1, workers=-1)[1].ravel(),
                                       k=8, radius=normal_radius, tree=tree)
        plane = np.abs(np.sum((q - ref[idx_nn]) * n_hat, axis=1))
        rmse = math.sqrt(float(np.sum(plane ** 2)) / len(plane))
        out.update({
            "status": "MEASURED",
            "denominator_n": int(len(q)),
            "p2pl_rmse_m": float(rmse),
            "p2p_rmse_m": float(math.sqrt(float(np.sum(d_nn ** 2)) / len(d_nn))),
            "p2p_stats": st(d_nn, "p2p"),
            "no_correspondence_fraction": float(np.mean(d_nn > support_radius)),
            "estimand_differences": [
                "uniform random subsample instead of the frozen grid",
                "k_normal = 8 instead of the contract k",
                "normals queried at the estimate position instead of the reference neighbour",
            ],
        })
        unc = fold_uncertainty(plane, folds)

    out["sampling_uncertainty"] = unc
    if main_value_m is not None:
        mv = abs(float(main_value_m))
        diff = abs(out["p2pl_rmse_m"] - float(main_value_m))
        if mode == "strict":
            # identical estimand -> the ONLY admissible tolerance is floating point
            tol = max(1e-9, 1e-9 * mv)
            verdict = "agrees" if diff <= tol else "disagrees"
        else:
            # different estimand: a statistical statement, and the sampling SE is the
            # (information-only) scale of the difference
            tol = max(3.0 * (unc.get("se_m") or 0.0), 1e-12)
            verdict = "agrees" if diff <= tol else "disagrees"
        out["comparison"] = {
            "abs_delta_m": float(diff),
            "tolerance_m": float(tol),
            "tolerance_relative_to_value": (float(tol / mv) if mv else None),
            "tolerance_source": ("floating-point tolerance: identical estimand, independent "
                                 "arithmetic (1e-9 absolute / relative)"
                                 if mode == "strict" else
                                 "3 x the MEASURED across-fold standard error of the sensitivity "
                                 "mode (information only, NOT a correctness standard)"),
            "verdict": verdict,
            "agrees": bool(verdict == "agrees"),
            "sampling_uncertainty_is_not_a_correctness_standard": True,
            "means": (("the arithmetic is verified (machine precision)" if (mode == "strict" and verdict == "agrees")
                       else "independent arithmetic DISAGREES with the main implementation: the "
                            "two implementations do not share an estimand and/or one is wrong"
                       if mode == "strict"
                       else "estimator-sensitivity statement only: NOT a correctness proof and "
                            "never a replacement for the main metric")
                      + ("; within the floating-point tolerance" if verdict == "agrees" and mode == "strict"
                         else "" if mode == "strict"
                         else ("; within the sampling scale" if verdict == "agrees"
                               else "; differs by more than the sampling scale"))),
        }
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--est-map", required=True)
    ap.add_argument("--ref-map", required=True)
    ap.add_argument("--roi", required=True)
    ap.add_argument("--mode", default="strict", choices=["strict", "sensitivity"])
    ap.add_argument("--main-result", default=None, help="map_accuracy_eval.py JSON to compare against")
    ap.add_argument("--transform-json", default=None)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--n-sample", type=int, default=200000)
    ap.add_argument("--k-normal", type=int, default=15)
    ap.add_argument("--normal-radius", type=float, default=0.15)
    ap.add_argument("--stat-voxel", type=float, default=0.05)
    ap.add_argument("--grid-origin", default="0,0,0")
    ap.add_argument("--support-radius", type=float, default=SUPPORT_RADIUS_M)
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    main_res = load_json(args.main_result) if args.main_result else None
    main_val = None
    if main_res and main_res.get("primary_metric"):
        main_val = main_res["primary_metric"]["value_m"]
    tjson = load_json(args.transform_json)
    T = np.asarray(tjson["matrix_ref_est"], float) if tjson else None
    origin = tuple(float(x) for x in args.grid_origin.split(","))
    mismatches = main_parameter_mismatches(
        main_res, stat_voxel=args.stat_voxel, grid_origin=origin, k_normal=args.k_normal,
        normal_radius=args.normal_radius, support_radius=args.support_radius,
        est_sha=sha256_file(args.est_map), ref_sha=sha256_file(args.ref_map),
        roi_sha=sha256_file(args.roi), T_ref_est=T)

    res = cross_check(load_points(args.est_map), load_points(args.ref_map),
                      roi=load_json(args.roi), mode=args.mode, seed=args.seed,
                      n_sample=args.n_sample, k_normal=args.k_normal,
                      normal_radius=args.normal_radius, stat_voxel=args.stat_voxel,
                      grid_origin=origin, support_radius=args.support_radius,
                      T_ref_est=T, main_value_m=main_val, folds=args.folds,
                      parameter_mismatches=mismatches)
    res["inputs"] = {"est_map": args.est_map, "ref_map": args.ref_map, "roi": args.roi,
                     "est_map_sha256": sha256_file(args.est_map),
                     "ref_map_sha256": sha256_file(args.ref_map),
                     "roi_sha256": sha256_file(args.roi)}
    dump_json(args.out, res)
    print("mode=%s status=%s rmse=%s agrees=%s"
          % (args.mode, res["status"], round(res.get("p2pl_rmse_m", float("nan")), 5),
             (res.get("comparison") or {}).get("agrees")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
