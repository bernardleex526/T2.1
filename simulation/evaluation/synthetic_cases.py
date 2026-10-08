#!/usr/bin/env python3
"""Synthetic validation cases for the corrected evaluation contract.

Each case returns a completely specified problem whose right answer is known in
closed form, so the evaluator can be checked for both magnitude and *behaviour*
(PASS vs not-measurable vs not-passable):

  geometry : normal offset, tangential offset, SE(3), local bump, double wall,
             local overlap, empty overlap, registration failure
  protocol : missing ROI, unsourced ROI, sourced transform, per-block transform
  time     : constant sensor offset, GT hole, estimate hole, GT-derived offset

No algorithm code and no network access: everything is generated from a fixed
seed with numpy.  Parameter values (offsets, gaps) are the *expected* answers,
which the tests assert against.
"""
from __future__ import annotations

import numpy as np

# ------------------------------------------------------------------- helpers --
def provenance(**over):
    p = {
        "source": "synthetic_cases.py (unit-test fixture, not a dataset claim)",
        "derivation": "closed-form construction, independent of any estimate",
        "frozen_utc": "2026-09-29T00:00:00Z",
        "author": "EvalContract",
        "independent_of_estimate": True,
        "independent_of_error": True,
    }
    p.update(over)
    return p


def roi_aabb(lo, hi, roi_id="synth_aabb", frame="ref_map_frame", **prov):
    return {"schema_version": 1, "roi_id": roi_id, "frame": frame, "type": "aabb",
            "bounds": {"min": [float(x) for x in lo], "max": [float(x) for x in hi]},
            "provenance": provenance(**prov)}


def plane_grid(half=2.0, spacing=0.02, offset=0.0, axis="z", seed=0, jitter=1e-5,
               jitter_seed=None):
    """Regular grid in the plane normal to `axis`, displaced by `offset` along it."""
    g = np.arange(-half, half + 0.5 * spacing, spacing)
    a, b = np.meshgrid(g, g, indexing="ij")
    a = a.ravel()
    b = b.ravel()
    zero = np.zeros_like(a)
    if axis == "z":
        pts = np.column_stack([a, b, zero + offset])
    elif axis == "x":
        pts = np.column_stack([zero + offset, a, b])
    elif axis == "y":
        pts = np.column_stack([a, zero + offset, b])
    else:
        raise ValueError(axis)
    if jitter:
        pts = pts + np.random.RandomState(seed if jitter_seed is None
                                           else jitter_seed).normal(0.0, jitter, pts.shape)
    return pts


def random_plane(n=40000, half=2.0, offset=0.0, axis="z", seed=0):
    """Poisson-uniform points in the plane normal to `axis`.

    Used where a *regular* grid would hide a tangential slip: a dense regular
    lattice is periodic, so a shift by a multiple of the spacing is invisible to
    point-to-point as well.  Uniform noise makes the same slip measurable.
    """
    rs = np.random.RandomState(seed)
    a = rs.uniform(-half, half, n)
    b = rs.uniform(-half, half, n)
    zero = np.zeros(n)
    if axis == "z":
        return np.column_stack([a, b, zero + offset])
    if axis == "x":
        return np.column_stack([zero + offset, a, b])
    return np.column_stack([a, zero + offset, b])


def yaw_se3(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, t=(0.0, 0.0, 0.0)):
    d = np.radians
    cx, sx = np.cos(d(roll_deg)), np.sin(d(roll_deg))
    cy, sy = np.cos(d(pitch_deg)), np.sin(d(pitch_deg))
    cz, sz = np.cos(d(yaw_deg)), np.sin(d(yaw_deg))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = t
    return T


def _roi_from(pts, pad=0.5, **kw):
    return roi_aabb(pts.min(axis=0) - pad, pts.max(axis=0) + pad, **kw)


# ----------------------------------------------------------------- geometry --
def case_normal_offset(offset_m=0.03, **ev):
    ref = plane_grid(offset=0.0, seed=1)
    est = plane_grid(offset=offset_m, seed=1, jitter_seed=1001)
    return {"est": est, "ref": ref, "roi": _roi_from(ref, roi_id="synth_normal"),
            "truth": {"p2pl_m": abs(offset_m), "expect_pass": abs(offset_m) <= 0.05},
            "event_kwargs": ev}


def case_tangential_offset(offset_m=0.06, **ev):
    # coarse REGULAR lattice: a lattice neighbour is 0.2 m away, so an in-plane
    # slip of 0.06 m is genuinely measurable by point-to-point (unlike a dense
    # Poisson cloud, where the nearest neighbour is always ~0.025 m away).
    ref = plane_grid(half=4.0, spacing=0.2, seed=2)
    est = plane_grid(half=4.0, spacing=0.2, seed=2, jitter_seed=1002)
    est[:, 0] += offset_m
    return {"est": est, "ref": ref, "roi": _roi_from(ref, roi_id="synth_tangential"),
            "truth": {"p2pl_m": 0.0, "p2p_m": offset_m,
                      "expect_pass": True,
                      "note": "an in-plane slip is invisible to point-to-plane by construction; "
                              "the reference is Poisson-uniform so it is NOT invisible to "
                              "point-to-point"},
            "event_kwargs": ev}


def case_se3(pitch_deg=5.0, t=(0.10, 0.0, 0.0)):
    ref = plane_grid(seed=3)
    # the estimate is built from an INDEPENDENTLY jittered copy: a real map never
    # re-uses the reference points, and applying the inverse transform must not
    # reproduce them exactly (that would be a degenerate, self-referential set)
    src = plane_grid(seed=3, jitter_seed=1003)
    T = yaw_se3(pitch_deg=pitch_deg, t=t)
    est = (T[:3, :3] @ src.T).T + np.asarray(t, float)
    Tinv = np.linalg.inv(T)
    return {"est": est, "ref": ref, "roi": _roi_from(ref, roi_id="synth_se3"),
            "T_fix": Tinv,
            "truth": {"tilt_deg": pitch_deg,
                      "expect_pass_with_T": True,
                      "expect_pass_without_T": False},
            "event_kwargs": {}}


def case_local_bump(amp=0.15, radius_m=0.5):
    ref = plane_grid(seed=4)
    est = plane_grid(seed=4, jitter_seed=1004)
    r = np.linalg.norm(ref[:, :2] - np.array([1.0, 1.0]), axis=1)
    est[:, 2] += amp * np.exp(-0.5 * (r / radius_m) ** 2)
    return {"est": est, "ref": ref, "roi": _roi_from(ref, roi_id="synth_bump"),
            "truth": {"bump_amp_m": amp,
                      "global_rmse_upper_m": 0.05,
                      "local_max_lower_m": 0.05},
            "event_kwargs": {}}


def case_reference_hole(half=4.0, spacing=0.05, hole_half_width=1.5, seed=13):
    """The reference has a 2*hole_half_width-wide hole; the estimate is continuous.

    Inside the hole the samples are neither supported nor provably drifted: the
    nearest reference is further than the support radius but closer than the
    no-reference radius, so the classification is genuinely unknown.
    """
    ref = plane_grid(half=half, spacing=spacing, seed=seed)
    ref = ref[np.abs(ref[:, 0]) >= hole_half_width]
    est = plane_grid(half=half, spacing=spacing, seed=seed, jitter_seed=seed + 1000)
    return {"est": est, "ref": ref, "roi": _roi_from(est, roi_id="synth_ref_hole"),
            "truth": {"expect_status": "NOT_MEASURABLE_UNKNOWN_SUPPORT_CLASSIFICATION"},
            "event_kwargs": {}}


def case_sparse_blunder(frac=0.05, blunder_m=0.5, seed=11):
    """95 % perfect estimate, 5 % displaced by 0.5 m.

    Constructs exactly the situation the contract forbids exploiting: dropping
    the large residuals would turn a failing RMSE into a passing one.
    """
    ref = plane_grid(seed=seed)
    est = plane_grid(seed=seed, jitter_seed=seed + 1000)
    rs = np.random.RandomState(seed)
    n = int(round(len(est) * frac))
    idx = rs.choice(len(est), n, replace=False)
    est[idx, 2] += blunder_m
    return {"est": est, "ref": ref, "roi": _roi_from(ref, roi_id="synth_blunder"),
            "truth": {"blunder_fraction": frac, "blunder_m": blunder_m,
                      "raw_p2pl_m": blunder_m * (frac ** 0.5),
                      "expect_pass": False},
            "event_kwargs": {}}


def case_double_wall(gap_m=0.10):
    """Reference is TWO walls `gap_m` apart; the estimate collapsed them into one."""
    ref_a = plane_grid(offset=0.0, seed=5)
    ref_b = plane_grid(offset=gap_m, seed=5)
    ref = np.vstack([ref_a, ref_b])
    est = plane_grid(offset=0.5 * gap_m, seed=5, jitter_seed=1005)
    return {"est": est, "ref": ref,
            "roi": roi_aabb(ref.min(axis=0) - 0.1, ref.max(axis=0) + 0.1,
                            roi_id="synth_double_wall"),
            "truth": {"gap_m": gap_m,
                      "expect_double_wall_flag": True,
                      "p2pl_m": 0.5 * gap_m},
            "event_kwargs": {}}


def case_local_overlap(keep=(-2.0, 0.0, -2.0, 0.0)):
    ref = plane_grid(seed=6)
    m = ((ref[:, 0] >= keep[0]) & (ref[:, 0] <= keep[1])
         & (ref[:, 1] >= keep[2]) & (ref[:, 1] <= keep[3]))
    est = ref[m].copy() + np.random.RandomState(1006).normal(0.0, 1e-5, ref[m].shape)
    return {"est": est, "ref": ref, "roi": _roi_from(ref, roi_id="synth_local_overlap"),
            "truth": {"expect_status": "NOT_MEASURABLE_LOW_COVERAGE"},
            "event_kwargs": {}}


def _far_pair(shift, roi_id):
    ref = plane_grid(seed=7)
    est = plane_grid(seed=7, jitter_seed=1007)
    est[:, 0] += shift
    lo = np.minimum(ref.min(axis=0), est.min(axis=0)) - 0.5
    hi = np.maximum(ref.max(axis=0), est.max(axis=0)) + 0.5
    return {"est": est, "ref": ref, "roi": roi_aabb(lo, hi, roi_id=roi_id),
            "truth": {"expect_pass": False, "expect_status_in": [
                "NOT_MEASURABLE_LOW_COVERAGE",
                "BLOCKED_FRAMES_UNALIGNED_NO_SOURCED_TRANSFORM",
                "NOT_PASSABLE_REGISTRATION_FITTED_ON_EVAL_DATA"]},
            "event_kwargs": {}}


def case_empty_overlap(shift=20.0):
    return _far_pair(shift, "synth_empty_overlap")


def case_registration_failure(shift=5.0):
    return _far_pair(shift, "synth_registration_failure")


# ------------------------------------------------------------------ protocol --
def case_missing_roi():
    c = case_normal_offset(0.01)
    c["roi"] = None
    c["truth"] = {"expect_status": "BLOCKED_PREDEFINED_ROI_MISSING", "expect_pass": False}
    return c


def case_unsourced_roi():
    c = case_normal_offset(0.01)
    c["roi"]["provenance"]["independent_of_estimate"] = False
    c["truth"] = {"expect_status": "BLOCKED_ROI_UNSOURCED", "expect_pass": False}
    return c


# ---------------------------------------------------------------------- time --
def tum(t, xyz, yaw_deg=None):
    n = len(t)
    q = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (n, 1))
    if yaw_deg is not None:
        h = np.radians(np.asarray(yaw_deg, float)) * 0.5
        q = np.column_stack([np.zeros(n), np.zeros(n), np.sin(h), np.cos(h)])
    return np.asarray(t, float), np.asarray(xyz, float), q


def case_time_offset(clock_offset_s=3600.0, max_gap_s=0.20):
    """Estimate stamped in a different clock domain (device time), 10 Hz GT.

    Convention (legacy-compatible): `offset_s` is ADDED to the GT stamps before
    matching, so a source that says "+3600 s" realigns a device-time estimate.
    """
    t_gt = np.arange(0.0, 10.0, 0.1)
    p_gt = np.column_stack([0.1 * t_gt, np.zeros_like(t_gt), np.zeros_like(t_gt)])
    t_est = t_gt + clock_offset_s
    # positions are the PHYSICAL ones (only the stamp carries the clock offset)
    p_est = np.column_stack([0.1 * t_gt, np.zeros_like(t_gt), np.zeros_like(t_gt)])
    return {"est_tum": tum(t_est, p_est), "gt_tum": tum(t_gt, p_gt),
            "time_source": {"offset_s": clock_offset_s, "method": "clock_domain",
                            "source": "synthetic: known constant clock offset"},
            "max_gap_s": max_gap_s,
            "truth": {"expect_status": "MEASURED",
                      "assoc_rate_min": 0.98,
                      "gt_coverage_min": 0.98}}


def case_time_no_offset_supplied(max_gap_s=0.20):
    c = case_time_offset(clock_offset_s=3600.0, max_gap_s=max_gap_s)
    c["time_source"] = {"offset_s": 0.0, "method": "clock_domain",
                        "source": "synthetic: offset deliberately left at zero"}
    c["truth"] = {"expect_status": "NOT_MEASURABLE_ASSOCIATION_FAILED",
                  "assoc_rate_max": 0.01}
    return c


def case_time_gt_gap(hole=(3.0, 6.0), max_gap_s=0.20):
    """GT stream has a 3 s hole; the estimate spans the whole recording."""
    t_full = np.arange(0.0, 10.0, 0.1)
    keep = (t_full < hole[0]) | (t_full > hole[1])
    t_gt = t_full[keep]
    p_gt = np.column_stack([0.1 * t_gt, np.zeros_like(t_gt), np.zeros_like(t_gt)])
    t_est = t_full
    p_est = np.column_stack([0.1 * t_est, np.zeros_like(t_est), np.zeros_like(t_est)])
    return {"est_tum": tum(t_est, p_est), "gt_tum": tum(t_gt, p_gt),
            "time_source": {"offset_s": 0.0, "method": "clock_domain",
                            "source": "synthetic: same clock, GT hole"},
            "max_gap_s": max_gap_s,
            "truth": {"expect_status": "MEASURED",
                      "assoc_rate_max": 0.80,
                      "expect_rejected_gap_gt": 20,
                      "expect_est_gap_segment_min_s": 2.5,
                      "note": "the gap is not bridged by interpolation; the estimate interval "
                              "that lost its GT is reported explicitly"}}


def case_time_est_hole():
    """Estimate covers only the first 20 % of the GT timeline."""
    t_gt = np.arange(0.0, 10.0, 0.1)
    p_gt = np.column_stack([0.1 * t_gt, np.zeros_like(t_gt), np.zeros_like(t_gt)])
    t_est = np.arange(0.0, 2.0, 0.1)
    p_est = np.column_stack([0.1 * t_est, np.zeros_like(t_est), np.zeros_like(t_est)])
    return {"est_tum": tum(t_est, p_est), "gt_tum": tum(t_gt, p_gt),
            "time_source": {"offset_s": 0.0, "method": "declared",
                            "source": "synthetic: truncated estimate"},
            "max_gap_s": 0.20,
            "truth": {"expect_status": "NOT_MEASURABLE_GT_COVERAGE_LOW",
                      "assoc_rate_min": 0.99,
                      "gt_coverage_max": 0.30,
                      "note": "association rate ~1.0 while the GT timeline coverage is ~0.2: "
                              "100 % association is NOT full coverage"}} 


def case_time_gt_derived_offset():
    c = case_time_offset(clock_offset_s=3600.0)
    c["time_source"] = {"offset_s": 3600.0, "method": "fitted_to_ground_truth",
                        "source": "synthetic: forbidden offset", "gt_derived": True}
    c["truth"] = {"expect_status": "BLOCKED_TIME_SOURCE_NOT_SOURCED",
                  "expect_usable_for_accuracy": False}
    return c


CASES = {
    "normal_offset_2cm": case_normal_offset,
    "tangential_offset_6cm": case_tangential_offset,
    "se3_5deg": case_se3,
    "local_bump": case_local_bump,
    "reference_hole": case_reference_hole,
    "sparse_blunder": case_sparse_blunder,
    "double_wall": case_double_wall,
    "local_overlap": case_local_overlap,
    "empty_overlap": case_empty_overlap,
    "registration_failure": case_registration_failure,
    "missing_roi": case_missing_roi,
    "unsourced_roi": case_unsourced_roi,
}

TIME_CASES = {
    "time_offset_sourced": case_time_offset,
    "time_offset_missing": case_time_no_offset_supplied,
    "time_gt_gap": case_time_gt_gap,
    "time_est_hole": case_time_est_hole,
    "time_gt_derived_offset": case_time_gt_derived_offset,
}
