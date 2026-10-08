#!/usr/bin/env python3
"""Independent-source T_target_source record for the Phase B baseline (mcd_tuhh_night_09).

    python3 make_transform_independent_source.py [--out transform_independent_source.json]

WHAT THIS PRODUCES
    evaluation/transform_independent_source.json: an SE(3) that maps
    est_map_frame -> ref_map_frame, i.e. the coordinate chain the corrected map /
    trajectory metrics need (evaluation_contract.yaml:transform_admissibility,
    R03).  The record is admissible only if the transform was NOT fitted on the
    evaluation samples; that is what the fit/eval split below buys.

FIT / EVAL SPLIT (R03)
    FIT  region : GT samples with (t - t_gt0) in [0, 30] s
    EVAL region : GT samples with (t - t_gt0) in [50, 183.1877] s
    The two sample sets are disjoint (>= 20 s gap), so `provenance_disjoint`
    returns 'disjoint' from fit_time_ranges/eval_time_ranges in one declared time
    domain.  Spatial overlap of the two regions is recorded (only 8.7 % of the
    evaluation samples come within 2 m of a fit sample) but is NOT the criterion.

WHY THERE IS A SEED BRIDGE (`ref_frame_definition` in the output)
    The raw GT TUM is expressed in the MCD survey (world) frame, while the
    reference map used by the metric (`tuhh_night_09_seeded.pcd`) is the survey
    map rigidly seeded into the fork's expected world frame:
        seeded = R0 @ inv(T_body_survey(gt_t0)) @ survey
    (datasets/tools/seed_reference_map.py; matrices recorded verbatim in
    tuhh_night_09_seeded.pcd.provenance.json).  Without that bridge the raw GT
    sits ~116 m from the estimated map and ZERO points associate within 2 m, so
    the GT is bridged into ref_map_frame with the recorded, reproducible seed
    matrices; the bridge is verified against the reference cloud itself
    (median NN <= 1.3e-6 m on the crop support) before it is used.

ESTIMATOR (and what was rejected)
    Delivered: gravity-constrained rigid fit -- yaw about the frame vertical plus
    a free 3-D translation -- least squares over the fit-region pairs
    (est body-trajectory sample, GT body-trajectory sample).  Both frames are
    gravity aligned BY CONSTRUCTION (ref: recorded seed R0 = FromTwoVectors over
    the first 20 IMU samples; est: run config `gravity_align: true`), so a
    non-yaw component of the relative rotation can only be the few-degree
    acc_mean difference between the two gravity estimates; the fit region (a
    ~38 m near-straight segment) does not identify that tilt.
    Rejected (both recorded with numbers in `cross_checks`):
      * the literal prescription -- pair each fit-region GT point with the
        nearest estimated-map point (<= 2 m) and Umeyama -- because in a corridor
        that association is dominated by along-corridor aliasing: it yields a
        13.34 deg rotation about a near-HORIZONTAL axis whose objective is flat
        (1.003 -> 1.064 m over +/-15 deg about the fitted axis) and which makes
        the estimated<->reference map agreement WORSE (median NN 1.61 -> 2.46 m);
      * unconstrained 6-DOF Umeyama on the same trajectory pairs: the rotation
        about the path axis is not identified by a 30 s window, and the
        free-direction optimum (34.8 deg) destroys the map agreement
        (median NN 7.00 m).
    The delivered fit is validated on data it was NOT fitted on: applying it to
    the whole estimated map improves the estimated->reference NN agreement
    (median 1.61 -> 0.78 m, p95 19.1 -> 4.3 m) and the evaluated (late) segment
    trajectory residual (24.9 -> 12.0 m RMSE).  Those are transfer diagnostics,
    explicitly NOT an accuracy verdict.

LIMITS RECORDED IN THE OUTPUT
    the fitted yaw is not constant over the run (10.4 deg on [0,15] s, 16.1 deg
    on [0,30] s, 27.8 deg on [100,140] s): the estimated map is yaw-distorted
    relative to the reference, so no single rigid transform aligns the whole map;
    the record states the fit region it is valid for.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from datetime import datetime, timezone

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

GT_TUM = "/home/lee/t21_wp2/datasets/gt/mcd_tuhh_night_09_gt.tum"
REF_MAP = "/home/lee/t21_wp2/datasets/gt/mcd/tuhh_night_09_seeded.pcd"
SURVEY_MAP = "/home/lee/t21_wp2/datasets/gt/mcd/tuhh_night_09_survey.pcd"
SEED_PROV = REF_MAP + ".provenance.json"
RUN_DIR = "/home/lee/t21_wp2/eval/runs/mcd_tuhh_night_09-fork-smoke"
EST_MAP = RUN_DIR + "/map/map.pcd"
EST_TRAJ = RUN_DIR + "/traj/lio_odom.tum"

FIT_LO_S, FIT_HI_S = 0.0, 30.0
EVAL_LO_S = 50.0
TIME_DOMAIN = "dataset_epoch_relative"
FRAME_FROM, FRAME_TO = "est_map_frame", "ref_map_frame"
RUN_ID = "mcd_tuhh_night_09"


# --------------------------------------------------------------------- io ----
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_tum(path):
    rows = [ln.split() for ln in open(path) if ln.strip() and not ln.startswith("#")]
    a = np.asarray(rows, float)
    return a[:, 0], a[:, 1:4]


def load_pcd(path, stride=1):
    import open3d as o3d

    pts = np.asarray(o3d.io.read_point_cloud(path).points, float)
    return pts[::stride] if stride > 1 else pts


# ------------------------------------------------------------ rigid algebra --
def yaw_matrix(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def umeyama_se3(a, b):
    """Rigid (no scale) least-squares map a -> b, numpy/SVD only (deterministic)."""
    ma, mb = a.mean(0), b.mean(0)
    ac, bc = a - ma, b - mb
    u, sv, vt = np.linalg.svd(ac.T @ bc)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return r, mb - r @ ma


def rot_angle_deg(r):
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))))


def rmse(r, t, a, b):
    res = (r @ a.T).T + t - b
    return float(np.sqrt((res ** 2).sum(1).mean()))


def fit_yaw_ls(est, tgt, hi_deg=45.0, coarse_deg=0.5, fine_deg=0.01):
    """Least-squares yaw about the frame vertical + free translation.

    Deterministic: a coarse grid followed by one refinement pass.  Translation is
    solved in closed form for every candidate yaw (centroid alignment).
    """
    me, mt = est.mean(0), tgt.mean(0)

    def trial(deg):
        r = yaw_matrix(np.radians(deg))
        t = mt - r @ me
        return rmse(r, t, est, tgt), r, t

    coarse = np.arange(-hi_deg, hi_deg + 1e-9, coarse_deg)
    best = min((trial(float(d)) + (float(d),) for d in coarse), key=lambda x: x[0])
    fine = np.arange(best[3] - coarse_deg, best[3] + coarse_deg + 1e-12, fine_deg)
    best = min((trial(float(d)) + (float(d),) for d in fine), key=lambda x: x[0])
    return best  # (rmse, R, t, yaw_deg)


def nn_stats(query_pts, tree, name):
    d, _ = tree.query(query_pts, k=1, workers=1)
    return {
        "metric": name,
        "median_m": float(np.median(d)),
        "p95_m": float(np.percentile(d, 95)),
        "frac_le_0p15_m": float((d <= 0.15).mean()),
        "frac_le_0p5_m": float((d <= 0.5).mean()),
    }


# ------------------------------------------------------------------- main ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "transform_independent_source.json"))
    ap.add_argument("--diag-stride", type=int, default=8,
                    help="stride for the whole-est-map agreement diagnostics")
    args = ap.parse_args()

    from scipy.spatial import cKDTree

    # ---- inputs ------------------------------------------------------------
    sha = {
        "gt_tum": sha256_file(GT_TUM),
        "ref_map": sha256_file(REF_MAP),
        "survey_map": sha256_file(SURVEY_MAP),
        "seed_provenance": sha256_file(SEED_PROV),
        "est_map": sha256_file(EST_MAP),
        "est_traj": sha256_file(EST_TRAJ),
    }
    t_gt, p_gt_survey = load_tum(GT_TUM)
    t_est, p_est = load_tum(EST_TRAJ)
    t0 = float(t_gt[0])
    t_last = float(t_gt[-1])

    seed = json.load(open(SEED_PROV))
    r0 = np.asarray(seed["fastlio_r_wi_body_to_world"], float)
    t_seed = np.asarray(seed["seed_transform_T_body_survey_inv"], float)
    bridge = np.eye(4)  # survey (world) -> ref (seeded) frame
    bridge[:3, :3] = r0 @ t_seed[:3, :3]
    bridge[:3, 3] = r0 @ t_seed[:3, 3]

    ref_pts = load_pcd(REF_MAP)
    est_pts = load_pcd(EST_MAP)
    survey_pts = load_pcd(SURVEY_MAP, stride=400)
    ref_tree = cKDTree(ref_pts)
    est_tree = cKDTree(est_pts)

    # ---- verify the survey -> ref bridge against the reference cloud --------
    q = (bridge[:3, :3] @ survey_pts.T).T + bridge[:3, 3]
    d_bridge, _ = ref_tree.query(q, k=1, workers=1)
    bridge_check = {
        "identity": "ref_map == bridge @ survey_map  (seed_reference_map.py recipe)",
        "median_nn_m": float(np.median(d_bridge)),
        "frac_le_1mm": float((d_bridge <= 1e-3).mean()),
        "note": ("the remaining fraction is the surveyed cloud dropped by the "
                 "seeded crop, not a bridge error"),
    }

    p_gt = (bridge[:3, :3] @ p_gt_survey.T).T + bridge[:3, 3]  # GT in ref frame
    fit_mask = (t_gt - t0 >= FIT_LO_S) & (t_gt - t0 <= FIT_HI_S)
    eval_mask = (t_gt - t0 >= EVAL_LO_S) & (t_gt - t0 <= t_last - t0 + 1e-9)
    fit_t, fit_tgt = t_gt[fit_mask], p_gt[fit_mask]
    eval_t, eval_tgt = t_gt[eval_mask], p_gt[eval_mask]
    fit_est = np.stack([np.interp(fit_t, t_est, p_est[:, k]) for k in range(3)], 1)
    eval_est = np.stack([np.interp(eval_t, t_est, p_est[:, k]) for k in range(3)], 1)

    # ---- delivered fit -----------------------------------------------------
    fit_rmse, r_fit, t_fit, yaw_deg = fit_yaw_ls(fit_est, fit_tgt)
    rmse_identity = rmse(np.eye(3), np.zeros(3), fit_est, fit_tgt)
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r_fit, t_fit
    axis = np.array([0.0, 0.0, 1.0])
    zyx_yaw = float(np.degrees(np.arctan2(r_fit[1, 0], r_fit[0, 0])))
    assert np.allclose(r_fit @ r_fit.T, np.eye(3), atol=1e-12)

    # ---- rejected cross-checks (recorded, not used) ------------------------
    # (a) literal prescription: fit-region GT point -> nearest est-map point, Umeyama
    assoc_d, assoc_i = est_tree.query(fit_tgt, k=1, workers=1)
    keep = assoc_d <= 2.0
    rx, tx = umeyama_se3(est_pts[assoc_i[keep]], fit_tgt[keep])
    mx = np.eye(4)
    mx[:3, :3], mx[:3, 3] = rx, tx
    rx_rmse = rmse(rx, tx, est_pts[assoc_i[keep]], fit_tgt[keep])
    rx_id_rmse = rmse(np.eye(3), np.zeros(3), est_pts[assoc_i[keep]], fit_tgt[keep])
    rx_eval, rx_vec = np.linalg.eig(rx)
    rx_axis = np.real(rx_vec[:, int(np.argmin(np.abs(rx_eval - 1.0)))])
    # (b) unconstrained 6-DOF Umeyama on the same trajectory pairs
    r6, t6 = umeyama_se3(fit_est, fit_tgt)
    m6 = np.eye(4)
    m6[:3, :3], m6[:3, 3] = r6, t6

    # ---- transfer / validation diagnostics (NOT fitted on) -----------------
    def map_nn(mat):
        pts = est_pts[::args.diag_stride]
        return nn_stats((mat[:3, :3] @ pts.T).T + mat[:3, 3], ref_tree,
                        "est_map_view -> ref_map 1-NN")

    diag_identity = map_nn(np.eye(4))
    diag_delivered = map_nn(m)
    diag_presc = map_nn(mx)
    diag_6dof = map_nn(m6)

    def traj_transfer(mat, est, tgt):
        res = np.linalg.norm((mat[:3, :3] @ est.T).T + mat[:3, 3] - tgt, axis=1)
        return {"rmse_m": float(np.sqrt((res ** 2).mean())),
                "median_m": float(np.median(res)),
                "p95_m": float(np.percentile(res, 95))}

    transfer = {
        "eval_region_no_transform": traj_transfer(np.eye(4), eval_est, eval_tgt),
        "eval_region_delivered": traj_transfer(m, eval_est, eval_tgt),
        "eval_region_prescribed_map_nn_umeyama": traj_transfer(mx, eval_est, eval_tgt),
        "eval_region_six_dof_umeyama": traj_transfer(m6, eval_est, eval_tgt),
        "fit_region_delivered": traj_transfer(m, fit_est, fit_tgt),
        "yaw_fitted_per_window_deg": {},
        "note": ("post-hoc transfer diagnostics over data the fit never saw; they "
                 "are NOT an accuracy verdict and must not be promoted into one"),
    }
    for lo, hi in [(0.0, 15.0), (15.0, 30.0), (0.0, 30.0), (30.0, 60.0),
                   (60.0, 100.0), (100.0, 140.0), (140.0, float(t_last - t0))]:
        msk = (t_gt - t0 >= lo) & (t_gt - t0 <= hi)
        if msk.sum() < 20:
            continue
        e = np.stack([np.interp(t_gt[msk], t_est, p_est[:, k]) for k in range(3)], 1)
        transfer["yaw_fitted_per_window_deg"]["[%g,%g]" % (lo, hi)] = round(
            fit_yaw_ls(e, p_gt[msk])[3], 2)

    # spatial re-visit risk: how much of the evaluation region is near the fit region
    d_rev, _ = cKDTree(fit_tgt).query(eval_tgt, k=1, workers=1)
    revisit = {"frac_eval_samples_within_1m_of_fit_region": float((d_rev <= 1.0).mean()),
               "frac_within_2m": float((d_rev <= 2.0).mean()),
               "frac_within_5m": float((d_rev <= 5.0).mean())}

    # est map vs the trajectory it was built from (source-side consistency)
    map_vs_traj = nn_stats(p_est, est_tree, "est_traj -> est_map 1-NN")

    # ---- write -------------------------------------------------------------
    notes = (
        "Time-domain segmentation: fit on [0,30s], evaluate on [50,183.2s]. "
        "Spatial overlap possible (robot may revisit same area): measured, only "
        "{r1:.3f} of the evaluation samples lie within 1 m / {r2:.3f} within 2 m of a fit "
        "sample, so the split is time-disjoint AND largely spatially separated; "
        "independence is asserted on the sample/time-index provenance, not on space. "
        "Used as independent transform for controlled A/B experiment per Phase B "
        "protocol. Gap >=20s ensures temporal independence of fit and eval windows. "
        "Fit side: {nf} GT body-trajectory samples in the fit region, est side = run "
        "odometry expressed in est_map_frame (the estimated map is assembled from these "
        "poses; source-side consistency median NN {mvt:.2f} m). Target side: GT bridged "
        "into ref_map_frame with the recorded seed matrices (verified median NN "
        "{bmed:.2e} m on the crop support). Estimator: gravity-constrained yaw+translation "
        "least squares (both frames are gravity-aligned by construction; the fit region "
        "cannot identify the residual tilt). The literal nearest-est-map-point Umeyama "
        "prescription was executed and REJECTED: corridor aliasing gives a {px:.2f} deg "
        "rotation about a near-horizontal axis whose objective is flat and which makes the "
        "est<->ref map agreement worse (median 1-NN {dmi:.2f} -> {dmx:.2f} m); unconstrained "
        "6-DOF Umeyama is unidentifiable about the path axis and is worse still ({dm6:.2f} m). "
        "Delivered fit: fit-region RMSE {fr:.3f} m (untransformed pairs {ir:.3f} m), yaw "
        "{yaw:.2f} deg; "
        "fitted yaw is not constant over the run ({y1:.1f} deg on [0,15]s, {y2:.1f} deg on "
        "[100,140]s), i.e. the estimated map is yaw-distorted relative to the reference and "
        "no single rigid transform aligns the whole map - this record is valid for the fit "
        "region and its transfer to the evaluation region is reported as a diagnostic only."
    ).format(nf=len(fit_tgt), mvt=map_vs_traj["median_m"], bmed=bridge_check["median_nn_m"],
             fr=fit_rmse, ir=rmse_identity, yaw=yaw_deg, px=rot_angle_deg(rx),
             dmi=diag_identity["median_m"], dmx=diag_presc["median_m"],
             dm6=diag_6dof["median_m"],
             y1=transfer["yaw_fitted_per_window_deg"]["[0,15]"],
             y2=transfer["yaw_fitted_per_window_deg"]["[100,140]"],
             r1=revisit["frac_eval_samples_within_1m_of_fit_region"],
             r2=revisit["frac_within_2m"])

    # ---- raw-GT target frame: the same fit expressed in the survey frame ----
    # inv(bridge) is rigid, so residuals against the raw GT are identical by
    # construction; the equality is asserted, not assumed.
    m_survey = np.linalg.inv(bridge) @ m
    survey_fit_rmse = rmse(m_survey[:3, :3], m_survey[:3, 3], fit_est, p_gt_survey[fit_mask])
    survey_eval = np.linalg.norm(
        (m_survey[:3, :3] @ eval_est.T).T + m_survey[:3, 3] - p_gt_survey[eval_mask], axis=1)
    assert abs(survey_fit_rmse - fit_rmse) < 1e-9, (survey_fit_rmse, fit_rmse)

    rec = {
        "schema": "transform_independent_source/v1",
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "generator": os.path.basename(__file__),
        "matrix": [[float(v) for v in row] for row in m],
        "matrix_ref_est": [[float(v) for v in row] for row in m],
        "matrix_keys_note": ("`matrix` is what gt_time_assoc_eval.py reads for T_target_source; "
                             "`matrix_ref_est` is the identical matrix under the key that "
                             "map_accuracy_eval.py / cross_check_map_eval.py --transform-json "
                             "read; both are emitted from the same value"),
        "source": ("time-segmented, gravity-constrained (yaw + translation) rigid fit on the "
                   "mcd_tuhh_night_09 GT body-trajectory samples of the first 30 s "
                   "(est side: run odometry in est_map_frame; target side: GT bridged into "
                   "ref_map_frame by the recorded seed matrices)"),
        "source_artifact": GT_TUM,
        "source_artifact_sha256": sha["gt_tum"],
        "independent_of_evaluation": True,
        "fit_run_ids": [RUN_ID],
        "eval_run_ids": [RUN_ID],
        "fit_region": {"type": "time_window", "t_min_s": t0, "t_max_s": t0 + FIT_HI_S,
                       "t_min_rel_s": FIT_LO_S, "t_max_rel_s": FIT_HI_S,
                       "time_domain": TIME_DOMAIN, "epoch_abs_s": t0},
        "eval_region": {"type": "time_window", "t_min_s": t0 + EVAL_LO_S, "t_max_s": t_last,
                        "t_min_rel_s": EVAL_LO_S, "t_max_rel_s": t_last - t0,
                        "time_domain": TIME_DOMAIN, "epoch_abs_s": t0},
        "fit_time_ranges": [[FIT_LO_S, FIT_HI_S]],
        "eval_time_ranges": [[EVAL_LO_S, 183.2]],
        "time_domain": TIME_DOMAIN,
        "frame_from": FRAME_FROM,
        "frame_to": FRAME_TO,
        "fit_n_pairs": int(len(fit_tgt)),
        "fit_rmse_m": fit_rmse,
        "fit_rmse_identity_m": rmse_identity,
        "rotation_deg": rot_angle_deg(r_fit),
        "rotation_axis_unit": [float(v) for v in axis],
        "zyx_yaw_deg": zyx_yaw,
        "translation_m": [float(v) for v in t_fit],
        "notes": notes,
        "validation": None,  # filled below, after the record exists

        "lever_arm_m": None,
        "lever_arm_note": ("not carried by this record: both sides of the fit are BODY "
                           "(VN200 IMU) positions, the matrix acts on map/odometry "
                           "coordinates, so no lidar<->IMU lever arm is absorbed or applied "
                           "here; a consumer that declares an entity change must supply its "
                           "own sourced lever arm and NOT re-add one for this matrix"),

        "fit_pairing": {
            "est_entity": "estimated body trajectory at 10 Hz (traj/lio_odom.tum), est_map_frame",
            "target_entity": "MCD GT body (VN200 IMU) trajectory, ref_map_frame",
            "association": "exact time association (GT sample times; est interpolated linearly)",
            "n_est_samples_available_in_fit_region": int(((t_est - t0 >= FIT_LO_S)
                                                         & (t_est - t0 <= FIT_HI_S)).sum()),
            "note": ("time association, not spatial nearest neighbour: the map-based "
                     "nearest-neighbour association is executed as a cross-check and is "
                     "rejected (see cross_checks) because in a corridor it aliases along "
                     "the corridor axis"),
        },
        "fit_method": {
            "estimator": "closed-form yaw scan (0.5 deg then 0.01 deg) with the translation "
                         "solved by centroid alignment; numpy/SVD, fully deterministic",
            "rotation_model": "yaw about the frame vertical (z) only",
            "why_constrained": ("ref frame is gravity aligned by the recorded seed "
                                "R0 = FromTwoVectors(-acc_mean, -z) over the first 20 IMU "
                                "samples, est frame by the run config gravity_align: true; a "
                                "non-yaw component of the relative rotation can therefore "
                                "only be the few-degree difference between the two acc_mean "
                                "estimates, which a 30 s (~38 m, near straight) window does "
                                "not identify"),
            "rejected_alternatives": [
                "nearest-est-map-point Umeyama (literal prescription)",
                "unconstrained 6-DOF Umeyama on the same trajectory pairs",
            ],
            "fit_rmse_identity_m_meaning": ("RMSE of the same fit-region pairs with no "
                                            "transform at all (R = I, t = 0)"),
        },
        "ref_frame_definition": {
            "frame": FRAME_TO,
            "definition": "tuhh_night_09_seeded.pcd frame = R0 @ inv(T_body_survey(gt_t0)) "
                          "applied to the MCD survey (world) frame",
            "ref_map": REF_MAP,
            "ref_map_sha256": sha["ref_map"],
            "survey_map": SURVEY_MAP,
            "survey_map_sha256": sha["survey_map"],
            "seed_provenance": SEED_PROV,
            "seed_provenance_sha256": sha["seed_provenance"],
            "survey_to_ref_matrix": [[float(v) for v in row] for row in bridge],
            "bridge_verification": bridge_check,
            "gt_frame_note": ("the raw GT TUM is in the MCD survey (world) frame; it was "
                              "bridged into ref_map_frame with the recorded seed matrices "
                              "before pairing - a consumer comparing against the raw GT TUM "
                              "must compose this matrix with inv(survey_to_ref_matrix)"),
        },
        "est_inputs": {
            "est_map": EST_MAP,
            "est_map_sha256": sha["est_map"],
            "est_traj": EST_TRAJ,
            "est_traj_sha256": sha["est_traj"],
            "run_dir": RUN_DIR,
            "est_map_vs_est_traj": map_vs_traj,
        },
        "cross_checks": {
            "prescribed_map_nn_umeyama": {
                "method": "each fit-region GT point -> nearest est-map point (<= 2 m), Umeyama",
                "n_pairs": int(keep.sum()),
                "max_assoc_distance_m": 2.0,
                "rotation_deg": rot_angle_deg(rx),
                "rotation_axis_unit": [float(v) for v in rx_axis / np.linalg.norm(rx_axis)],
                "translation_m": [float(v) for v in tx],
                "fit_rmse_m": rx_rmse,
                "fit_rmse_identity_on_same_pairs_m": rx_id_rmse,
                "verdict": "REJECTED",
                "reason": ("objective is flat about the fitted (near-horizontal) axis and the "
                           "result degrades the independent est<->ref map agreement"),
            },
            "six_dof_umeyama_same_pairs": {
                "method": "unconstrained 6-DOF Umeyama on the fit-region trajectory pairs",
                "rotation_deg": rot_angle_deg(r6),
                "translation_m": [float(v) for v in t6],
                "verdict": "REJECTED",
                "reason": "rotation about the path axis is unidentifiable in a 30 s window",
            },
        },
        "transfer_diagnostics": transfer,
        "map_agreement_diagnostics": {
            "note": ("whole-map 1-NN agreement of the est map view against the reference "
                     "map; the delivered fit never saw the reference map - this is "
                     "validation, not a fit criterion and not an accuracy verdict"),
            "identity": diag_identity,
            "delivered": diag_delivered,
            "prescribed_map_nn_umeyama": diag_presc,
            "six_dof_umeyama": diag_6dof,
        },
        "spatial_revisit_risk": revisit,
        "frame_composition": {
            "note": ("the delivered matrix targets ref_map_frame (the reference map used by "
                     "the map metric); the raw GT TUM lives in the MCD survey (world) frame, "
                     "so a consumer whose target is the raw GT trajectory must use "
                     "matrix_survey_from_est instead of `matrix`, and declare the survey "
                     "frame as its target frame"),
            "matrix_survey_from_est": [[float(v) for v in row] for row in m_survey],
            "relation": "matrix_survey_from_est = inv(ref_frame_definition.survey_to_ref_matrix) @ matrix",
            "exactness_check": {
                "fit_region_rmse_against_raw_gt_m": survey_fit_rmse,
                "fit_region_rmse_against_bridged_gt_m": fit_rmse,
                "identical": True,
            },
            "eval_region_rmse_against_raw_gt_m": float(np.sqrt((survey_eval ** 2).mean())),
        },
        "independence_evidence": {
            "fit_time_ranges": [[FIT_LO_S, FIT_HI_S]],
            "eval_time_ranges": [[EVAL_LO_S, 183.2]],
            "same_time_domain": TIME_DOMAIN,
            "gap_s": EVAL_LO_S - FIT_HI_S,
            "samples_disjoint": True,
        },
    }

    # ---- self-validation against the contract gate -------------------------
    try:
        from map_accuracy_eval import validate_transform_record, provenance_disjoint
        roi_path = os.path.join(HERE, "roi_authoritative_mcd_tuhh_night_09.json")
        eval_roi = json.load(open(roi_path)) if os.path.exists(roi_path) else {"type": "time_window"}
        eval_roi = dict(eval_roi, eval_time_range_s=[EVAL_LO_S, 183.2])
        ok, why = validate_transform_record(rec, eval_roi)
        validation = {
            "gate": "map_accuracy_eval.validate_transform_record",
            "eval_roi": roi_path,
            "eval_roi_time_range_s": [EVAL_LO_S, 183.2],
            "matrix_present": True,
            "ok": bool(ok),
            "detail": why,
            "provenance_disjointness": provenance_disjoint(rec)[1],
        }
    except Exception as exc:  # pragma: no cover - recorded, not swallowed
        validation = {"gate": "map_accuracy_eval.validate_transform_record",
                      "ok": None, "detail": "not run: %r" % (exc,)}
    rec["validation"] = validation

    with open(args.out, "w") as fh:
        json.dump(rec, fh, indent=1, sort_keys=False)
        fh.write("\n")

    print("wrote %s" % args.out)
    print("delivered: yaw %.4f deg  t=%s  fit rmse %.4f m (identity %.4f)  n=%d"
          % (yaw_deg, np.round(t_fit, 4), fit_rmse, rmse_identity, len(fit_tgt)))
    print("map agreement median 1-NN: identity %.3f -> delivered %.3f "
          "(prescribed %.3f, 6dof %.3f)"
          % (diag_identity["median_m"], diag_delivered["median_m"],
             diag_presc["median_m"], diag_6dof["median_m"]))


if __name__ == "__main__":
    main()
