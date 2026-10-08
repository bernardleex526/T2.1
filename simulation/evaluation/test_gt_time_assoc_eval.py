#!/usr/bin/env python3
"""Full validation module for gt_time_assoc_eval.py.

Run the WHOLE module under the coordinator's authorization:

    python3 -m pytest -q <this file>

Asserts the contract's time rules: a sourced offset is required (a GT-fitted
offset is diagnostic only), interpolation is gap-limited (no bridging of a hole
in the GT), and the association rate and the effective GT timeline coverage are
reported as separate columns so that "100 % associated" cannot be read as
"100 % covered".
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import synthetic_cases as sc  # noqa: E402
from gt_time_assoc_eval import TimeSourceError, evaluate, validate_time_source  # noqa: E402


FRAMES = dict(source_frame="gt_map", target_frame="gt_map",
              sensor_entity="livox_frame", gt_entity="livox_frame")


def run(case, **over):
    kw = dict(max_gap_s=case.get("max_gap_s", 0.20),
              time_source=case["time_source"], **FRAMES)
    kw.update(over)
    return evaluate(case["est_tum"], case["gt_tum"], **kw)


def test_sourced_offset_realigns_and_reports_both_columns():
    c = sc.case_time_offset(clock_offset_s=3600.0)
    r = run(c)
    assert r["status"] == "MEASURED"
    assert r["association"]["association_rate_est"] >= 0.98
    a = r["association"]
    assert a["gt_time_covered_fraction"] >= 0.98
    assert a["gt_coverage_detail"]["window_mode"] == "full_gt_span"
    assert a["gt_time_covered_s"] > 0
    assert a["gt_coverage_detail"]["window_duration_s"] > 0
    assert a["gt_samples_covered_fraction"] >= 0.98
    assert r["time_source"]["method_normalised"] == "clock_domain"
    assert r["time_source"]["gt_derived"] is False
    assert r["usable_for_accuracy"] is True
    assert r["association"]["ate_against_interpolated_gt_m"]["ate_rmse"] < 1e-9


def test_missing_offset_is_not_measurable_not_a_silent_mismatch():
    c = sc.case_time_no_offset_supplied()
    r = run(c)
    assert r["status"] == "NOT_MEASURABLE_ASSOCIATION_FAILED"
    assert r["association"]["association_rate_est"] <= 0.01
    assert r["usable_for_accuracy"] is False
    assert r["association"]["est_stamps_outside_gt_span"] > 90


def test_gt_fitted_offset_is_diagnostic_only():
    c = sc.case_time_gt_derived_offset()
    r = run(c)
    assert r["status"] == "BLOCKED_TIME_SOURCE_NOT_SOURCED"
    assert r["usable_for_accuracy"] is False
    assert r["association"] is None

    diag = run(c, allow_gt_derived_diagnostic=True)
    assert diag["status"] == "MEASURED"
    assert diag["usable_for_accuracy"] is False
    assert any("GT-derived" in u["detail"] for u in diag["unknown_states"])


def test_method_name_implying_a_fit_is_caught_even_without_the_flag():
    usable, why, rec = validate_time_source(
        {"offset_s": -0.05, "method": "minimize_ate_then_reuse", "source": "x"})
    assert usable is False and rec["gt_derived"] is True
    assert "GT-derived" in why


def test_invalid_time_source_records_are_rejected():
    with pytest.raises(TimeSourceError):
        validate_time_source({"offset_s": 0.0, "source": "x"})           # no method
    with pytest.raises(TimeSourceError):
        validate_time_source({"method": "declared", "source": "x"})      # no offset_s
    with pytest.raises(TimeSourceError):
        validate_time_source({"offset_s": 0.0, "method": "declared"})    # no source


def test_gt_hole_is_not_bridged_by_interpolation():
    c = sc.case_time_gt_gap(hole=(3.0, 6.0), max_gap_s=0.20)
    r = run(c)
    assert r["status"] == "MEASURED"
    a = r["association"]
    assert a["association_rate_est"] <= 0.80
    assert a["rejected_because_bracket_gap_gt_max_gap_n"] >= 20
    segs = a["est_segments_without_gt"]
    assert segs and segs[0]["duration_s"] >= 2.5
    # every association that WAS made used a bracket no wider than the limit
    assert a["bracket_span_stats_s"]["bracket_span_max"] <= 0.20 + 1e-9


def test_100pct_association_is_not_100pct_coverage():
    c = sc.case_time_est_hole()
    r = run(c)
    assert r["status"] == "NOT_MEASURABLE_GT_COVERAGE_LOW"
    a = r["association"]
    assert a["association_rate_est"] >= 0.99
    assert a["gt_time_covered_fraction"] <= 0.30
    # ... while the COMMON window would have looked perfect: the full-window
    # denominator is what makes the truncation visible
    assert a["gt_coverage_detail"]["gt_time_covered_fraction_common_window"] > 0.9
    assert a["rate_100pct_is_not_full_coverage"] is True
    assert r["usable_for_accuracy"] is False


def test_no_new_pass_threshold_is_introduced_for_aux_metrics():
    c = sc.case_time_offset()
    r = run(c)
    assert r["no_new_pass_threshold_for_aux_metrics"] is True
    assert "pass" not in r


def test_undeclared_frame_chain_is_blocked():
    c = sc.case_time_offset()
    r = evaluate(c["est_tum"], c["gt_tum"], time_source=c["time_source"], max_gap_s=0.20)
    assert r["status"] == "BLOCKED_FRAME_CHAIN_UNDECLARED"
    assert r["usable_for_accuracy"] is False


def test_declared_mismatched_frames_require_a_chain_and_it_is_applied():
    c = sc.case_time_offset()
    partial = dict(FRAMES, source_frame="body", target_frame="gt_map")
    r = run(c, **partial)
    assert r["status"] == "BLOCKED_MISSING_FRAME_CHAIN"

    # a 1 m offset chain: with it applied the estimate really moves, and the
    # unaligned diagnostic value stays separately labelled
    T = {"matrix": np.eye(4).tolist(), "source": "declared chain (synthetic)",
         "source_artifact": "evaluation/synthetic_cases.py", "source_artifact_sha256": "0" * 64,
         "independent_of_evaluation": True,
         "fit_run_ids": ["calib_run"], "eval_run_ids": ["eval_run"],
         "fit_region": "calibration", "eval_region": "roi"}
    T["frame_from"], T["frame_to"] = "body", "gt_map"
    T["matrix"][0][3] = 1.0
    r2 = run(c, **partial, T_target_source=T)
    assert r2["frames"]["chain_status"] == "applied"
    assert r2["association"]["ate_against_interpolated_gt_m"]["ate_rmse"] == pytest.approx(1.0, abs=0.02)
    assert r2["association"]["ate_raw_unaligned_m"]["ate_unaligned_rmse"] < 1e-6

    # a missing lever arm is also a hard block
    partial3 = dict(FRAMES, sensor_entity="imu_frame")
    assert run(c, **partial3)["status"] == "BLOCKED_MISSING_FRAME_CHAIN"
    r3 = run(c, **partial3, lever_arm_m=[0.0, 0.0, 0.0],
             lever_arm_record={"source": "synthetic calibration",
                               "source_artifact": "evaluation/synthetic_cases.py"})
    assert r3["frames"]["chain_status"] == "applied"
    # R39: the lever arm needs a source record, and an empty transform record is NOT identity
    assert run(c, **partial3, lever_arm_m=[0.0, 0.0, 0.0])["status"] == "BLOCKED_UNSOURCED_LEVER_ARM"
    r4 = run(c, **partial, T_target_source={})
    assert r4["status"] == "BLOCKED_MISSING_FRAME_CHAIN"
    r5 = run(c, **partial, T_target_source={"matrix": np.eye(4).tolist(),
                                            "source": "x", "fit_region": "a", "eval_region": "b"})
    assert r5["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"


def test_frames_record_requires_rigid_target_source():
    from eval_common import AlignmentError
    c = sc.case_time_offset()
    bad = np.eye(4)
    bad[1, 1] = 0.9
    # an incomplete record is refused before any rigidity check
    r0 = run(c, target_frame="map", sensor_entity="livox_frame",
             T_target_source={"matrix": bad, "source": "x"})
    assert r0["status"] == "NOT_PASSABLE_TRANSFORM_NOT_INDEPENDENT"
    # a complete, independent record with a non-rigid matrix raises
    badrec = {"matrix": bad, "source": "synthetic", "source_artifact": "x",
              "source_artifact_sha256": "0" * 64, "independent_of_evaluation": True,
              "fit_run_ids": ["calib"], "eval_run_ids": ["eval"],
              "frame_from": "odom", "frame_to": "map",
              "fit_region": "calibration", "eval_region": "roi"}
    with pytest.raises(AlignmentError):
        run(c, target_frame="map", sensor_entity="livox_frame", T_target_source=badrec)
    good = run(c, target_frame="map", sensor_entity="livox_frame",
               lever_arm_m=[0.02, 0.0, 0.037],
               lever_arm_record={"source": "manifest ext_il",
                                 "source_artifact": "eval/manifests/survey_gt.json"},
               T_target_source={"matrix": np.eye(4).tolist(), "source": "manifest ext_il",
                                "source_artifact": "eval/manifests/survey_gt.json",
                                "independent_of_evaluation": True,
                                "fit_run_ids": ["calib"], "eval_run_ids": ["eval"],
                                "fit_region": "calibration", "eval_region": "roi"})
    assert good["frames"]["target_frame"] == "map"
    assert good["frames"]["sensor_entity"] == "livox_frame"
    assert good["frames"]["T_target_source_source"] == "manifest ext_il"


def test_cli_end_to_end(tmp_path):
    from eval_common import dump_json, load_json
    from gt_time_assoc_eval import main as cli_main
    c = sc.case_time_est_hole()
    est_p, gt_p = str(tmp_path / "est.tum"), str(tmp_path / "gt.tum")
    for p, (t, p3, q) in ((est_p, c["est_tum"]), (gt_p, c["gt_tum"])):
        with open(p, "w") as fh:
            for i in range(len(t)):
                fh.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"
                         % (t[i], p3[i, 0], p3[i, 1], p3[i, 2], q[i, 0], q[i, 1], q[i, 2], q[i, 3]))
    ts_p = dump_json(str(tmp_path / "time_source.json"), c["time_source"])
    out = str(tmp_path / "assoc.json")
    import subprocess, sys as _sys
    proc = subprocess.run(
        [_sys.executable, os.path.join(HERE, "gt_time_assoc_eval.py"),
         "--est", est_p, "--gt", gt_p, "--time-source", ts_p,
         "--max-gap-s", "0.20", "--source-frame", "gt_map", "--target-frame", "gt_map",
         "--sensor-entity", "livox_frame", "--gt-entity", "livox_frame", "--out", out],
        capture_output=True, text=True, cwd=HERE)
    assert proc.returncode == 0, proc.stderr
    assert "status=NOT_MEASURABLE_GT_COVERAGE_LOW" in proc.stdout
    assert os.path.isfile(out)
    j = load_json(out)
    assert j["status"] == "NOT_MEASURABLE_GT_COVERAGE_LOW"
    assert j["inputs"]["gt_sha256"]
