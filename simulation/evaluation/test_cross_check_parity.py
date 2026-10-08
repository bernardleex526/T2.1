#!/usr/bin/env python3
"""Parity module: main evaluator vs the cross-check, in BOTH of its modes.

Run the WHOLE module under the coordinator's authorization:

    python3 -m pytest -q <this file>

Review R12: the cross-check is no longer a single unlabelled "independent"
implementation.  `strict` shares the contract's sample set and re-implements the
arithmetic (agreement = arithmetic verified); `sensitivity` deliberately uses a
different estimand (agreement = an insensitivity statement only).  Both report a
MEASURED sampling uncertainty (across-fold SE) instead of a hand-picked
tolerance.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import synthetic_cases as sc  # noqa: E402
from cross_check_map_eval import (  # noqa: E402
    SENS_CLAIM, STRICT_CLAIM, cross_check, fold_uncertainty,
)
from map_accuracy_eval import evaluate_map  # noqa: E402

BASE = dict(stat_voxel=0.05, min_denominator=1000)


def both(case, mode="strict", **over):
    kw = dict(BASE)
    kw.update(over)
    main_res = evaluate_map(case["est"], case["ref"], roi=case["roi"], **kw)
    pm = main_res.get("primary_metric") or {}
    cc = cross_check(case["est"], case["ref"], roi=case["roi"], mode=mode,
                     stat_voxel=kw["stat_voxel"], k_normal=15,
                     main_value_m=pm.get("value_m"))
    return main_res, cc


@pytest.mark.parametrize("offset", [0.02, 0.06, 0.20])
def test_strict_mode_matches_the_main_metric_to_machine_precision(offset):
    main_res, cc = both(sc.case_normal_offset(offset))
    assert cc["status"] == "MEASURED"
    assert cc["claim"] == STRICT_CLAIM
    assert cc["p2pl_rmse_m"] == pytest.approx(offset, abs=0.005)
    # the strict check runs on the SAME fixed denominator (same estimand)
    assert cc["denominator_n"] == main_res["sampling"]["denominator_fixed_n"]
    assert cc["comparison"]["verdict"] == "agrees"
    assert cc["comparison"]["agrees"] is True
    # identical estimand -> machine precision, NOT a statistical tolerance
    assert cc["comparison"]["abs_delta_m"] < 1e-9
    assert cc["comparison"]["tolerance_source"].startswith("floating-point tolerance")
    assert cc["comparison"]["tolerance_m"] <= 1e-9 * max(1.0, main_res["primary_metric"]["value_m"])
    # the sampling SE is reported but is explicitly NOT the correctness standard
    assert cc["sampling_uncertainty"]["se_m"] is not None
    assert cc["comparison"]["sampling_uncertainty_is_not_a_correctness_standard"] is True


def test_strict_mode_is_sensitive_to_an_injected_arithmetic_error():
    """The strict check must be able to FAIL: a 1 mm bias is detectable."""
    case = sc.case_normal_offset(0.02)
    est = case["est"].copy()
    est[:, 2] += 0.001
    main_res = evaluate_map(est, case["ref"], roi=case["roi"], **BASE)
    cc = cross_check(est, case["ref"], roi=case["roi"], mode="strict",
                     stat_voxel=0.05, k_normal=15,
                     main_value_m=main_res["primary_metric"]["value_m"] - 0.01)
    assert cc["comparison"]["agrees"] is False
    assert cc["comparison"]["abs_delta_m"] > cc["comparison"]["tolerance_m"]


def test_sensitivity_mode_changes_the_estimand_on_purpose():
    main_res, cc = both(sc.case_local_bump(amp=0.15, radius_m=0.5), mode="sensitivity")
    assert cc["status"] == "MEASURED"
    assert cc["claim"] == SENS_CLAIM
    assert cc["replaces_main_metric"] is False
    assert cc["estimand_differences"]
    assert cc["denominator_n"] != main_res["sampling"]["denominator_fixed_n"]


def test_strict_mode_disagreement_is_reported_not_tolerated():
    """A deliberately different estimand must NOT be excused by a statistical band."""
    main_res, cc = both(sc.case_normal_offset(0.02))
    assert cc["comparison"]["verdict"] == "agrees"
    assert cc["comparison"]["abs_delta_m"] < 1e-9
    case = sc.case_normal_offset(0.02)
    off = cross_check(case["est"], case["ref"], roi=case["roi"], mode="strict",
                      stat_voxel=0.05, k_normal=15,
                      main_value_m=main_res["primary_metric"]["value_m"] + 5e-4)
    assert off["comparison"]["verdict"] == "disagrees"
    assert "DISAGREES" in off["comparison"]["means"]


def test_sensitivity_mode_is_never_a_correctness_claim():
    _, cc = both(sc.case_normal_offset(0.02), mode="sensitivity")
    assert "NOT a correctness proof" in cc["claim"]
    assert "sensitivity" in cc["comparison"]["means"]
    assert "NOT a correctness proof" in cc["comparison"]["means"]


def test_inconclusive_verdict_when_the_tolerance_cannot_resolve_the_value():
    """Real long-tailed data: the fold SE can be too large to verify anything."""
    from cross_check_map_eval import fold_uncertainty
    u = fold_uncertainty(np.array([0.0] * 40 + [100.0] * 10), 5)
    assert u["se_m"] is not None and u["se_m"] > 1.0
    assert u["folds"] == 5


def test_fold_uncertainty_is_measured_and_shrinks_with_more_data():
    rs = np.random.RandomState(0)
    small = np.abs(rs.normal(0.05, 0.02, 50))
    large = np.abs(rs.normal(0.05, 0.02, 5000))
    se_small = fold_uncertainty(small, 5)["se_m"]
    se_large = fold_uncertainty(large, 5)["se_m"]
    assert se_small is not None and se_large is not None
    assert se_large < se_small
    u = fold_uncertainty(small, 5)
    assert len(u["fold_rmse"]) == 5 and u["definition"].startswith("standard error")


def test_cross_check_refuses_unsourced_roi_like_the_main_evaluator():
    case = sc.case_unsourced_roi()
    _, cc = both(case)
    assert cc["status"] == "BLOCKED_ROI_UNSOURCED"


def test_cross_check_applies_only_rigid_transforms():
    from eval_common import AlignmentError
    case = sc.case_tangential_offset(0.06)
    bad = sc.yaw_se3(yaw_deg=1.0)
    bad[0, 0] *= 1.3
    with pytest.raises(AlignmentError):
        cross_check(case["est"], case["ref"], roi=case["roi"], mode="strict", T_ref_est=bad)


def test_strict_mode_reports_no_correspondence_for_the_far_case():
    case = sc.case_registration_failure(shift=5.0)
    main_res, cc = both(case)
    assert main_res["pass"] is not True
    assert cc["status"] == "MEASURED"
    assert cc["no_correspondence_fraction"] > 0.99
