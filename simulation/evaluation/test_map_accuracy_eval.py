#!/usr/bin/env python3
"""Full validation module for map_accuracy_eval.py (corrected v2.2 contract).

Run the WHOLE module (never a hand-picked single case):

    python3 -m pytest -q test_map_accuracy_eval.py

Findings addressed by these tests (independent_review.json):
  R01 truncation can never be the headline nor buy a PASS
  R02 fixed membership + full-cloud support search + no-reference vs drifted-out
  R03 transform admissibility needs an independent, disjoint-region source record
  R06 coincidence is a 3-D point coincidence, not a zero plane residual
  R07 decisive unknowns can never coexist with a PASS
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import synthetic_cases as sc  # noqa: E402
from eval_common import AlignmentError, assert_not_per_block, assert_se3  # noqa: E402
from map_accuracy_eval import (  # noqa: E402
    PRIMARY_THRESHOLD_M, evaluate_map, main as cli_main,
    regions_may_intersect, validate_transform_record,
)

BASE = dict(stat_voxel=0.05, min_denominator=1000)


def good_transform_record(matrix, **over):
    """An ADMISSIBLE record: independent provenance, spatially overlapping regions."""
    r = {
        "matrix_ref_est": np.asarray(matrix, float).tolist(),
        "source": "synthetic analytic construction (case_se3)",
        "source_artifact": "evaluation/synthetic_cases.py::case_se3",
        "source_artifact_sha256": "0" * 64,
        "independent_of_evaluation": True,
        "frame_from": "est_map_frame",
        "frame_to": "ref_map_frame",
        "fit_region": [[-1, -1, -1], [1, 1, 1]],       # DELIBERATELY overlapping in space
        "eval_region": [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]],
        "fit_run_ids": ["calib_run_A"],
        "eval_run_ids": ["eval_run_B"],
        "fit_time_ranges": [[0.0, 10.0]],
        "eval_time_ranges": [[20.0, 30.0]],
    }
    r.update(over)
    return r


def run(case, **over):
    kw = dict(BASE)
    kw.update(case.get("event_kwargs", {}))
    kw.update(over)
    return evaluate_map(case["est"], case["ref"], roi=case.get("roi"), **kw)


# --------------------------------------------------------------- geometry --
def test_normal_offset_2cm_magnitude_and_pass():
    c = sc.case_normal_offset(0.02)
    r = run(c)
    assert r["status"] == "PASS" and r["pass"] is True
    assert abs(r["primary_metric"]["value_m"] - 0.02) < 0.003
    assert r["primary_metric"]["denominator_fixed_n"] >= 1000
    assert r["brackets"]["point_to_plane"]["truncation_applied"] is False
    assert r["brackets"]["point_to_plane"]["raw"]["p2pl_rmse"] == pytest.approx(
        r["primary_metric"]["raw_value_m"])


def test_normal_offset_6cm_fails_but_is_still_measured():
    c = sc.case_normal_offset(0.06)
    r = run(c)
    assert r["status"] == "FAIL" and r["pass"] is False
    assert abs(r["primary_metric"]["value_m"] - 0.06) < 0.005
    assert r["support"]["no_correspondence_est_n"] == 0


def test_tangential_offset_is_invisible_to_point_to_plane():
    c = sc.case_tangential_offset(0.06)
    r = run(c)
    assert r["est_to_ref_point_to_plane"]["p2pl_rmse"] < 0.005
    assert r["est_to_ref_point_to_point"]["p2p_rmse"] > 0.04
    assert r["pass"] is True
    # R06: a zero PLANE residual is not a coincidence -- the 3-D distances are 0.06 m
    d = r["brackets"]["degeneracy"]
    assert d["exact_coincidence_c2c_fraction"] < 0.01
    assert not any(u["code"] == "UNKNOWN_SELF_REFERENTIAL_COMPARISON" for u in r["unknown_states"])


def test_se3_is_recovered_only_with_a_sourced_transform():
    c = sc.case_se3(pitch_deg=5.0, t=(0.10, 0.0, 0.0))
    fixed = run(c, T_manager_ref_est=c["T_fix"], transform_record=good_transform_record(c["T_fix"]))
    assert fixed["status"] == "PASS"
    assert fixed["primary_metric"]["value_m"] < 0.005
    assert fixed["alignment"]["scale"] == pytest.approx(1.0)
    assert fixed["alignment"]["per_block"] is False
    assert fixed["transform_record"]["spatial_overlap_is_not_a_leakage_judgement"] is True
    assert "disjoint" in fixed["transform_record"]["provenance_disjointness"]

    raw = run(c)
    assert raw["pass"] is not True
    assert raw["primary_metric"]["value_m"] > 0.05


def test_local_bump_blocks_an_overall_claim_while_the_global_rmse_passes():
    c = sc.case_local_bump(amp=0.15, radius_m=0.5)
    r = run(c)
    assert r["primary_metric"]["value_m"] < PRIMARY_THRESHOLD_M
    assert r["est_to_ref_point_to_plane"]["p2pl_max"] > 0.05
    assert r["regional"]["worst_region_p2pl_p99"] > 2.0 * PRIMARY_THRESHOLD_M
    # R07: a decisive regional unknown must NOT coexist with a PASS
    assert r["status"] == "UNKNOWN_REGIONAL_GAP"
    assert r["pass"] is None
    assert r["unknown_gate_note"]


def test_double_wall_is_flagged_although_the_planar_residual_looks_small():
    c = sc.case_double_wall(gap_m=0.10)
    r = run(c)
    assert r["primary_metric"]["value_m"] == pytest.approx(0.05, abs=0.008)
    assert r["thin_structure"]["double_wall_flag"] is True
    assert r["thin_structure"]["double_wall_suspect_fraction"] > 0.5


def test_local_overlap_cannot_pass():
    c = sc.case_local_overlap()
    r = run(c)
    assert r["status"] == "NOT_MEASURABLE_LOW_COVERAGE"
    assert r["pass"] is None
    assert r["support"]["estimate_coverage_of_reference"] < r["coverage_floor"]


def test_empty_overlap_cannot_pass():
    c = sc.case_empty_overlap(shift=20.0)
    r = run(c)
    assert r["pass"] is not True
    assert r["status"] in c["truth"]["expect_status_in"]


def test_registration_failure_cannot_pass_without_and_with_icp():
    c = sc.case_registration_failure(shift=5.0)
    r = run(c)
    assert r["pass"] is not True
    assert r["status"] == "BLOCKED_FRAMES_UNALIGNED_NO_SOURCED_TRANSFORM"

    r2 = run(c, icp={"mode": "conditional", "voxels": [0.5, 0.2], "max_dists": [2.0, 0.5],
                     "max_iter": 40})
    assert r2["pass"] is not True
    if r2["icp"].get("identity_sufficient") is False:
        assert r2["status"] == "NOT_PASSABLE_REGISTRATION_FITTED_ON_EVAL_DATA"


# ------------------------------------------------- R02 membership / support --
def test_membership_is_sampled_over_the_whole_estimate_and_reported():
    c = sc.case_normal_offset(0.02)
    r = run(c)
    m = r["membership"]
    assert m["est_sampled_total_n"] == m["est_in_region_n"] + m["est_outside_region_n"]
    assert m["membership_hash"]
    assert r["sampling"]["sampled_over"].startswith("the WHOLE estimated map")


def test_estimate_that_mostly_leaves_the_region_cannot_pass():
    """R02: a drifted map must not vanish into 'not measurable' silently."""
    c = sc.case_normal_offset(0.02)
    est = c["est"].copy()
    est[: int(0.7 * len(est)), 2] += 30.0        # 70 % of it leaves the frozen region
    r = evaluate_map(est, c["ref"], roi=c["roi"], **BASE)
    assert r["membership"]["est_outside_region_fraction"] > 0.5
    assert r["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert r["pass"] is None
    assert r["membership"]["unattributed_outside_n"] > 0
    # the raw value and the exclusion counts stay reported as diagnostics
    assert r["primary_metric"]["value_m"] is not None
    assert r["outside_region_would_be_status"] in ("PASS", "FAIL")


def test_a_single_unattributed_outside_point_blocks_the_pass():
    """R38: no tolerance fraction -- 1 % displaced outside the region is enough."""
    c = sc.case_normal_offset(0.02)
    est = c["est"].copy()
    n = int(0.01 * len(est))
    est[:n, 2] += 40.0                      # 1 % pushed far outside the frozen region
    r = evaluate_map(est, c["ref"], roi=c["roi"], **BASE)
    assert 0 < r["membership"]["unattributed_outside_n"] < 0.05 * r["membership"]["est_sampled_total_n"]
    assert r["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert r["pass"] is None
    assert r["primary_metric"]["value_m"] < PRIMARY_THRESHOLD_M      # the surviving set looks fine
    assert r["outside_region_would_be_status"] == "PASS"             # ... and is still not a PASS

    # R38 (v2.3): the declarative flags alone attribute NOTHING.  The record must
    # declare the member GEOMETRY and the membership is MEASURED against it.
    declared = {"schema": "outside_attribution/v2",
                "source": "pre-declared member set (synthetic)",
                "source_artifact": "evaluation/synthetic_cases.py",
                "source_artifact_sha256": "0" * 64,
                "independent_of_estimate": True, "independent_of_error": True,
                "member_ids": ["synthetic-outside"]}
    r_flags = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=declared, **BASE)
    assert r_flags["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert r_flags["membership"]["outside_region_attribution"]["member_region"] is None
    assert (r_flags["membership"]["unattributed_outside_n"]
            == r["membership"]["unattributed_outside_n"])

    # a member geometry that actually CONTAINS the displaced points attributes them
    covered = dict(declared, member_region={
        "schema": "member_region/v1", "frame": "ref_map_frame",
        "derivation": "synthetic box spanning the displaced points",
        "independent_of_estimate": True, "independent_of_error": True,
        "include": [{"type": "aabb", "bounds": {
            "min": [float(v) for v in est.min(axis=0) - 1.0],
            "max": [float(v) for v in est.max(axis=0) + 1.0]}}]})
    r2 = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=covered, **BASE)
    assert r2["status"] == "PASS"
    att2 = r2["membership"]["outside_region_attribution"]
    assert att2["member_region"]["geometry_sha256"]
    assert att2["member_region"]["n_include_boxes"] == 1
    assert att2["measured"]["residual_uncovered_n"] == 0
    assert r2["membership"]["attributed_outside_n"] == r["membership"]["unattributed_outside_n"]

    bad = dict(covered, independent_of_error=False)
    assert evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=bad,
                        **BASE)["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"


def test_partial_member_region_attribution_cannot_pass():
    """R38 regression: a member set covering only PART of the outside points must
    not buy a PASS -- the uncovered remainder stays a decisive unknown (the
    historical declarative record attributed the whole outside set although its
    declared region covered only 75.69 % of it)."""
    c = sc.case_normal_offset(0.02)
    est = c["est"].copy()
    n = int(0.01 * len(est))
    est[:n, 2] += 40.0                      # group A: far above the frozen region
    est[n:2 * n, 2] -= 40.0                 # group B: far below it, NOT a member
    r = evaluate_map(est, c["ref"], roi=c["roi"], **BASE)
    assert r["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    outside_n = r["membership"]["outside_region_attribution"]["measured"]["outside_n"]
    assert outside_n > 0

    att = {"schema": "outside_attribution/v2",
           "source": "pre-declared member set (synthetic, only the upper blob)",
           "source_artifact": "evaluation/synthetic_cases.py",
           "source_artifact_sha256": "0" * 64,
           "independent_of_estimate": True, "independent_of_error": True,
           "member_region": {
               "schema": "member_region/v1", "frame": "ref_map_frame",
               "derivation": "only the upper blob is declared a member",
               "independent_of_estimate": True, "independent_of_error": True,
               "include": [{"type": "aabb",
                            "bounds": {"min": [-3.0, -3.0, 39.0], "max": [3.0, 3.0, 41.0]}}]}}
    rp = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=att, **BASE)
    meas = rp["membership"]["outside_region_attribution"]["measured"]
    assert meas["outside_n"] == outside_n
    assert 0 < meas["covered_by_member_region_n"] < outside_n        # genuinely PARTIAL
    assert meas["residual_uncovered_n"] == outside_n - meas["covered_by_member_region_n"] > 0
    assert 0.0 < meas["covered_by_member_region_fraction"] < 1.0
    assert meas["residual_is_a_decisive_unknown"] is True
    assert rp["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert rp["pass"] is None
    assert rp["membership"]["unattributed_outside_n"] == meas["residual_uncovered_n"]
    assert rp["primary_metric"]["value_m"] < PRIMARY_THRESHOLD_M     # the ROI subset looks fine
    assert rp["outside_region_would_be_status"] == "PASS"            # ... and is still not a PASS

    # widening the declared geometry to cover EVERYTHING does pass, so the gate is
    # not rejecting by construction
    full = dict(att, member_region=dict(
        att["member_region"],
        include=[{"type": "aabb", "bounds": {"min": [float(v) for v in est.min(axis=0)],
                                             "max": [float(v) for v in est.max(axis=0)]}}]))
    rf = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=full, **BASE)
    assert rf["status"] == "PASS"
    assert rf["membership"]["outside_region_attribution"]["measured"]["residual_uncovered_n"] == 0


def test_member_region_geometry_must_be_declared_and_independent():
    """R38: unusable member geometry is refused, never silently trusted."""
    c = sc.case_normal_offset(0.02)
    est = c["est"].copy()
    est[: int(0.01 * len(est)), 2] += 40.0
    base = {"source": "pre-declared member set", "source_artifact_sha256": "0" * 64,
            "independent_of_estimate": True, "independent_of_error": True}
    att = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=dict(base),
                       **BASE)["membership"]["outside_region_attribution"]
    assert att["member_region"] is None and "no member_region geometry" in att["member_region_unusable_reason"]

    no_include = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=dict(base, member_region={
        "frame": c["roi"]["frame"], "derivation": "d", "independent_of_estimate": True,
        "independent_of_error": True,
        "exclude": [{"bounds": {"min": [-3, -3, -3], "max": [3, 3, 3]}}]}), **BASE)
    assert no_include["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert "no include box" in no_include["membership"]["outside_region_attribution"][
        "member_region_unusable_reason"]

    not_independent = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=dict(base, member_region={
        "frame": c["roi"]["frame"], "derivation": "d", "independent_of_estimate": False,
        "independent_of_error": True,
        "include": [{"bounds": {"min": [-3, -3, 39], "max": [3, 3, 41]}}]}), **BASE)
    assert not_independent["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert not_independent["membership"]["outside_region_attribution"]["member_region"] is None

    malformed = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=dict(base, member_region={
        "frame": c["roi"]["frame"], "derivation": "d", "independent_of_estimate": True,
        "independent_of_error": True,
        "include": [{"bounds": {"min": [0.0, 0.0], "max": [1.0, 1.0]}}]}), **BASE)
    assert malformed["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert "unusable boxes" in malformed["membership"]["outside_region_attribution"][
        "member_region_unusable_reason"]


def test_member_region_frame_must_match_the_evaluation_frame():
    """R38/P1 regression: the SAME numeric bounds declared in another frame are a
    different region and must not attribute anything."""
    c = sc.case_normal_offset(0.02)
    est = c["est"].copy()
    est[: int(0.01 * len(est)), 2] += 40.0
    box = {"min": [float(v) for v in est.min(axis=0) - 1.0],
           "max": [float(v) for v in est.max(axis=0) + 1.0]}
    base = {"schema": "outside_attribution/v2",
            "source": "pre-declared member set", "source_artifact_sha256": "0" * 64,
            "independent_of_estimate": True, "independent_of_error": True}

    def att(**mr_over):
        mr = {"schema": "member_region/v1", "frame": c["roi"]["frame"],
              "derivation": "synthetic box spanning the displaced points",
              "independent_of_estimate": True, "independent_of_error": True,
              "include": [{"type": "aabb", "bounds": box}]}
        mr.update(mr_over)
        return dict(base, member_region=mr)

    # the matching frame is the only admissible one, and it attributes every point
    good = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=att(), **BASE)
    assert good["status"] == "PASS"
    assert good["membership"]["outside_region_attribution"]["member_region"][
        "frame_matches_evaluation_frame"] is True
    assert good["membership"]["outside_region_attribution"]["member_region"][
        "eval_frame_checked_against"] == c["roi"]["frame"]

    # same bounds, WRONG frame -> nothing attributed, residual preserved
    wrong = evaluate_map(est, c["ref"], roi=c["roi"],
                         outside_attribution=att(frame="mcd_survey_frame"), **BASE)
    wa = wrong["membership"]["outside_region_attribution"]
    assert wrong["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert wrong["pass"] is None and wa["member_region"] is None
    assert "same numeric bounds in another frame" in wa["member_region_unusable_reason"]
    assert wrong["membership"]["unattributed_outside_n"] == good["membership"]["attributed_outside_n"]

    # a declared frame_conversion is refused too (rigid re-boxing would over-attribute)
    conv = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=att(
        frame="mcd_survey_frame",
        frame_conversion={"from_frame": "mcd_survey_frame", "to_frame": c["roi"]["frame"],
                          "matrix": np.eye(4).tolist(), "source": "synthetic"}), **BASE)
    assert conv["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert "does NOT convert member geometry" in conv["membership"]["outside_region_attribution"][
        "member_region_unusable_reason"]

    # no declared frame -> refused
    noframe = evaluate_map(est, c["ref"], roi=c["roi"], outside_attribution=att(frame=None), **BASE)
    assert noframe["status"] == "BLOCKED_UNATTRIBUTED_OUTSIDE_REGION_POINTS"
    assert "declares no frame" in noframe["membership"]["outside_region_attribution"][
        "member_region_unusable_reason"]


def test_reference_anchored_membership_is_predeclared():
    r = run(sc.case_normal_offset(0.02))
    ra = r["reference_anchored"]
    assert ra["predeclared_membership_n"] > 0 and ra["member_hash"]
    assert ra["coverage_of_reference_members"] > 0.99
    assert ra["p2pl_stats"]["p2pl_ref_anchored_rmse"] == pytest.approx(0.02, abs=0.003)
    assert ra["estimate_absent_locally_n"] == 0


def test_drifted_out_and_no_reference_are_separated():
    """A blunder 0.5 m above a surface is DRIFTED, not 'no reference here'."""
    c = sc.case_sparse_blunder(frac=0.05, blunder_m=0.5)
    r = run(c)
    assert r["support"]["drifted_out_of_support_n"] > 0
    assert r["support"]["no_reference_locally_n"] == 0
    assert r["support"]["no_reference_radius_m"] >= 1.0
    assert r["status"] == "FAIL"
    assert r["pass"] is False


def test_unclassifiable_support_is_a_decisive_unknown_not_a_silent_pass():
    """A hole in the reference creates a band where drift is indistinguishable."""
    c = sc.case_reference_hole()
    r = run(c)
    assert r["support"]["no_reference_locally_fraction"] > 0.05
    assert r["status"] == "NOT_MEASURABLE_UNKNOWN_SUPPORT_CLASSIFICATION"
    assert r["pass"] is None
    assert r["primary_metric"]["value_m"] is not None      # the number is still reported


# ------------------------------------------------------------- R01 / R06 ----
def test_truncation_bracket_reports_raw_excluded_and_denominator():
    c = sc.case_local_bump(amp=0.15, radius_m=0.5)
    r = run(c, truncate_at=0.05)
    b = r["brackets"]["point_to_plane"]
    assert b["truncation_applied"] is True and b["excluded_n"] > 0
    assert b["excluded_fraction"] == pytest.approx(b["excluded_n"] / b["denominator_fixed_n"])
    assert r["primary_metric"]["value_m"] == pytest.approx(r["primary_metric"]["raw_value_m"])
    assert r["brackets"]["truncation_cannot_promote_a_pass"] is True
    assert r["verdict"]["truncated_value_m"] == pytest.approx(b["truncated"]["p2pl_rmse"])


def test_deleting_large_residuals_cannot_buy_a_pass():
    c = sc.case_sparse_blunder(frac=0.05, blunder_m=0.5)
    raw = run(c)
    assert raw["status"] == "FAIL" and raw["pass"] is False
    assert raw["primary_metric"]["value_m"] > 0.10
    assert raw["support"]["drifted_out_of_support_n"] > 0
    trunc = run(c, truncate_at=0.05)
    b = trunc["brackets"]["point_to_plane"]
    assert 0.05 < b["excluded_fraction"] < 0.40
    assert b["truncated"]["p2pl_rmse"] < 0.01       # looks perfect after filtering
    assert trunc["pass"] is False                    # ... and still cannot PASS
    assert trunc["primary_metric"]["value_m"] > 0.05


def test_identical_clouds_are_reported_but_cannot_claim_an_overall_pass():
    pts = sc.plane_grid(seed=9)
    r = evaluate_map(pts, pts, roi=sc.roi_aabb(pts.min(0) - 0.1, pts.max(0) + 0.1,
                                               roi_id="identical"), **BASE)
    assert r["primary_metric"]["value_m"] < 1e-4
    d = r["brackets"]["degeneracy"]
    assert d["exact_coincidence_c2c_fraction"] > 0.0
    assert d["exact_coincidence_c2c_fraction"] == pytest.approx(
        d["exact_coincidence_c2c_n"] / r["brackets"]["point_to_plane"]["denominator_fixed_n"])
    # an identical pair is either a hard sampling degeneracy or a self-referential
    # comparison; neither may carry an overall PASS
    assert r["status"] in ("UNKNOWN_SELF_REFERENTIAL_COMPARISON",
                           "NOT_MEASURABLE_DEGENERATE_SAMPLING")
    assert r["pass"] is None


def test_copied_reference_points_are_refused_as_degenerate():
    ref = sc.case_normal_offset(0.02)["ref"]
    roi = sc.roi_aabb(ref.min(0) - 0.1, ref.max(0) + 0.1, roi_id="copied")
    r = evaluate_map(ref.copy(), ref, roi=roi, stat_voxel=0.05, min_denominator=1000,
                     grid_origin_est=(0.0, 0.0, 0.0), grid_origin_ref=(0.025, 0.025, 0.025))
    assert r["brackets"]["degeneracy"]["exact_coincidence_c2c_fraction"] > 0.99
    assert r["status"] == "NOT_MEASURABLE_DEGENERATE_SAMPLING"
    assert r["pass"] is None


# ---------------------------------------------------------------- protocol --
def test_missing_roi_blocks_instead_of_picking_one():
    c = sc.case_missing_roi()
    r = run(c)
    assert r["status"] == "BLOCKED_PREDEFINED_ROI_MISSING"
    assert r["pass"] is None and r["primary_metric"] is None
    assert "DatasetQualification" in r["blocker"]


def test_unsourced_roi_blocks():
    c = sc.case_unsourced_roi()
    r = run(c)
    assert r["status"] == "BLOCKED_ROI_UNSOURCED"
    assert r["pass"] is None


def test_roi_artifact_marked_invalid_or_superseded_is_refused():
    """A region explicitly retired as frame-inconsistent must not be usable."""
    c = sc.case_normal_offset(0.02)
    roi = dict(c["roi"])
    roi["invalid"] = True
    roi["invalid_reason"] = "mixed-frame corners"
    roi["superseded_by"] = "evaluation/roi_refframe_corrected_mcd_tuhh_night_09.json"
    r = evaluate_map(c["est"], c["ref"], roi=roi, **BASE)
    assert r["status"] == "BLOCKED_ROI_UNSOURCED"
    assert r["pass"] is None
    assert "invalid" in r["blocker"]


def test_diagnostic_only_roi_can_never_pass():
    c = sc.case_normal_offset(0.02)
    roi = dict(c["roi"])
    roi["provenance"] = dict(roi["provenance"], diagnostic_only=True)
    r = evaluate_map(c["est"], c["ref"], roi=roi, **BASE)
    assert r["status"] == "DIAGNOSTIC_ONLY_ROI"
    assert r["pass"] is None
    assert r["primary_metric"]["value_m"] is not None           # diagnostic number kept
    assert r["diagnostic_would_be_status"] == "PASS"


def test_unsourced_transform_is_blocked_and_fitted_region_disqualifies():
    c = sc.case_registration_failure(shift=5.0)
    r = evaluate_map(c["est"], c["ref"], roi=c["roi"], **BASE)
    assert r["status"] == "BLOCKED_FRAMES_UNALIGNED_NO_SOURCED_TRANSFORM"
    assert r["pass"] is None

    tang = sc.case_tangential_offset(0.06)
    assert run(tang, T_manager_ref_est=np.eye(4))["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    # (a) spatially overlapping regions with INDEPENDENT runs are ADMISSIBLE
    ok = run(tang, T_manager_ref_est=np.eye(4), transform_record=good_transform_record(np.eye(4)))
    assert ok["status"] == "PASS"
    assert ok["transform_record"]["fit_region_eval_region_spatially_overlapping"] is True
    assert any("overlap spatially" in w for w in ok["warnings"])

    # (b) the SAME run on both sides is a leak, even with disjoint boxes
    leaked = good_transform_record(np.eye(4), fit_run_ids=["eval_run_B"],
                                   fit_region=[[-9] * 3, [-8] * 3])
    rb = run(tang, T_manager_ref_est=np.eye(4), transform_record=leaked)
    assert rb["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    assert "fitted on the evaluation samples" in rb["blocker"]

    # (c) overlapping time-index ranges are a leak
    overlap_t = good_transform_record(np.eye(4), fit_time_ranges=[[25.0, 35.0]])
    assert run(tang, T_manager_ref_est=np.eye(4),
               transform_record=overlap_t)["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"

    # (d) no provenance at all: independence cannot be established
    noprov = good_transform_record(np.eye(4))
    for k in ("fit_run_ids", "eval_run_ids", "fit_time_ranges", "eval_time_ranges"):
        noprov.pop(k)
    r4 = run(tang, T_manager_ref_est=np.eye(4), transform_record=noprov)
    assert r4["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    assert "cannot be established" in r4["blocker"]

    # (e) other hard requirements
    assert run(tang, T_manager_ref_est=np.eye(4),
               transform_record=good_transform_record(np.eye(4),
                                                      independent_of_evaluation=False)
               )["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    rec3 = good_transform_record(np.eye(4))
    rec3.pop("source_artifact_sha256")
    rec3.pop("source_artifact")
    assert run(tang, T_manager_ref_est=np.eye(4),
               transform_record=rec3)["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    # (f) a bare sample_disjoint flag is NOT evidence: concrete run/time provenance is required
    onlyflag = good_transform_record(np.eye(4), sample_disjoint=True)
    for k in ("fit_run_ids", "eval_run_ids", "fit_time_ranges", "eval_time_ranges"):
        onlyflag.pop(k)
    rf = run(tang, T_manager_ref_est=np.eye(4), transform_record=onlyflag)
    assert rf["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    assert "not evidence" in rf["blocker"]


def test_provenance_identity_is_sample_first_time_domain_aware():
    from map_accuracy_eval import provenance_disjoint
    # same run, but pre-declared independent sample ids -> disjoint
    assert provenance_disjoint({"fit_sample_ids": ["r1/s0", "r1/s1"],
                                "eval_sample_ids": ["r1/s9"]})[0] == "disjoint"
    # same sample ids -> the transform was fitted on the evaluation samples
    assert provenance_disjoint({"fit_sample_ids": ["r1/s9"],
                                "eval_sample_ids": ["r1/s9"]})[0] == "overlapping"
    # same run + disjoint time ranges inside the same time domain -> disjoint
    assert provenance_disjoint({"fit_run_ids": ["r1"], "eval_run_ids": ["r1"],
                                "fit_time_ranges": [[0, 1]], "eval_time_ranges": [[5, 6]],
                                "time_domain": "epoch"})[0] == "disjoint"
    # different RUNS whose device-relative clocks both start at 0: value overlap is not a leak
    assert provenance_disjoint({"fit_run_ids": ["a"], "eval_run_ids": ["b"],
                                "fit_time_ranges": [[0, 5]], "eval_time_ranges": [[0, 5]],
                                "fit_time_domain": "dev_a", "eval_time_domain": "dev_b"})[0] == "disjoint"
    # same TIME DOMAIN with overlapping ranges -> leak
    assert provenance_disjoint({"fit_time_ranges": [[0, 5]], "eval_time_ranges": [[4, 6]],
                                "time_domain": "epoch"})[0] == "overlapping"


def test_region_overlap_helper_records_but_does_not_reject():
    assert regions_may_intersect([[-1, -1, -1], [1, 1, 1]], [[-0.5] * 3, [0.5] * 3]) is True
    assert regions_may_intersect([[-3] * 3, [-2] * 3], [[-1] * 3, [1] * 3]) is False
    assert regions_may_intersect("roi_a", "roi_b") is False
    assert regions_may_intersect(None, "roi_a") is True
    ok, why = validate_transform_record(good_transform_record(np.eye(4)), True)
    assert ok and why is None                      # overlapping boxes, independent runs -> OK
    from map_accuracy_eval import provenance_disjoint
    assert provenance_disjoint({"fit_run_ids": ["a"], "eval_run_ids": ["b"],
                                "fit_time_ranges": [[0, 1]], "eval_time_ranges": [[2, 3]]})[0] == "disjoint"
    assert provenance_disjoint({"fit_run_ids": ["a"], "eval_run_ids": ["a"]})[0] == "overlapping"
    assert provenance_disjoint({})[0] == "unknown"
    assert provenance_disjoint({"sample_disjoint": True})[0] == "unknown"
    assert provenance_disjoint({"fit_run_ids": ["a"], "eval_run_ids": ["b"]})[0] == "disjoint"
    assert provenance_disjoint({"fit_time_ranges": [[0, 1]], "eval_time_ranges": [[5, 6]]})[0] == "disjoint"


def test_sampling_record_is_complete():
    c = sc.case_normal_offset(0.02)
    r = run(c, seed=3)
    s = r["sampling"]
    assert s["stat_voxel_m"] == 0.05 and s["seed"] == 3
    assert len(s["grid_origin_est_m"]) == 3 and len(s["grid_origin_ref_m"]) == 3
    assert s["grid_origin_est_effective_m"] != s["grid_origin_ref_effective_m"]
    assert r["normals"]["k"] == 15 and r["normals"]["radius_m"] == 0.15
    assert r["sampling"]["weighting"].startswith("none")


# ------------------------------------------------------------------ guards --
def test_se3_guard_rejects_scale_shear_and_reflection():
    R = sc.yaw_se3(yaw_deg=10.0)[:3, :3]
    with pytest.raises(AlignmentError):
        assert_se3(np.block([[1.1 * R, np.zeros((3, 1))], [np.zeros((1, 3)), 1.0]]))
    with pytest.raises(AlignmentError):
        assert_se3(np.array([[1.0, 0.02, 0.0, 0.0], [0, 1.0, 0.0, 0.0],
                             [0, 0, 1.0, 0.0], [0, 0, 0, 1.0]]))
    assert_se3(sc.yaw_se3(yaw_deg=10.0, t=(1, 2, 3)))


def test_per_block_alignment_is_rejected():
    with pytest.raises(AlignmentError):
        assert_not_per_block({"per_block": [{"i": 0}]})
    with pytest.raises(AlignmentError):
        assert_not_per_block({"stages": [{"patches": [1, 2, 3]}]})
    assert_not_per_block({"matrix_ref_est": np.eye(4).tolist()})


def test_evaluate_map_rejects_non_rigid_input_transform():
    c = sc.case_tangential_offset(0.06)
    bad = np.eye(4)
    bad[0, 0] = 1.2
    with pytest.raises(AlignmentError):
        run(c, T_manager_ref_est=bad)


# --------------------------------------------------------------------- CLI --
def test_cli_end_to_end_and_roi_file_io(tmp_path):
    from eval_common import dump_json, load_json, write_pcd_ascii
    c = sc.case_normal_offset(0.02)
    est_p = write_pcd_ascii(str(tmp_path / "est.pcd"), c["est"])
    ref_p = write_pcd_ascii(str(tmp_path / "ref.pcd"), c["ref"])
    roi_p = dump_json(str(tmp_path / "roi.json"), c["roi"])
    out = str(tmp_path / "map_accuracy.json")
    import subprocess, sys as _sys
    proc = subprocess.run(
        [_sys.executable, os.path.join(HERE, "map_accuracy_eval.py"),
         "--est-map", est_p, "--ref-map", ref_p, "--roi", roi_p,
         "--out", out, "--stat-voxel", "0.05", "--min-denominator", "1000"],
        capture_output=True, text=True, cwd=HERE)
    assert proc.returncode == 0, proc.stderr
    assert "status=PASS" in proc.stdout
    assert os.path.isfile(out)
    j = load_json(out)
    assert j["status"] == "PASS"
    assert abs(j["primary_metric"]["value_m"] - 0.02) < 0.003
    assert j["inputs"]["est_map_sha256"] and j["inputs"]["roi_sha256"]
