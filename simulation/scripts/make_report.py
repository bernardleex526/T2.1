#!/usr/bin/env python3
"""Assemble the integration report from the pipeline's own evidence files.

Reads <out-root>/integration_report.json (written by run_eval_pipeline.sh) plus each variant's
verdict/record JSONs and emits a Markdown report.  Everything in the report is read from the
recorded artifacts, so the numbers cannot drift from the evidence.

Usage:
    python3 make_report.py --out-root /tmp/sim_pipeline --out .omp/reports/simulation_integration.md
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402


def fmt(value, digits=4):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return ("%%.%df" % digits) % value
    return str(value)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-root", default="/tmp/sim_pipeline")
    ap.add_argument("--out", default=".omp/reports/simulation_integration.md")
    ap.add_argument("--evidence-dir", default=None,
                    help="where the artifacts were persisted (recorded in the report)")
    args = ap.parse_args(argv)

    rep = sc.read_json(os.path.join(args.out_root, "integration_report.json"))
    variants = rep.get("variants") or {}
    lines = []
    add = lines.append
    add("# Synthetic SLAM harness - integration results")
    add("")
    add("Generated %s from `%s/integration_report.json`." % (sc.utcnow(), args.out_root))
    add("")
    add("| field | value |")
    add("|---|---|")
    add("| repo git head | `%s` (dirty=%s) |" % (rep.get("git_head"), rep.get("git_dirty")))
    add("| scene reference dir | `%s` |" % rep.get("scene_ref_dir"))
    add("| fit window (frame chain) | sim time <= %s s |" % fmt(rep.get("fit_until_s"), 1))
    add("| ROS domain | %s (ROS_LOCALHOST_ONLY=1) |" % rep.get("domain_id"))
    add("| evidence persisted to | `%s` |" % (args.evidence_dir or "(see log paths below)"))
    add("")
    add("All numbers below are measured on the synthetic scene only (no external dataset, no")
    add("hardware, no Orin claim). T1/T2/T3 exit codes: 0 PASS, 1 FAIL, 2 BLOCKED.")
    add("")

    add("## Verdict summary")
    add("")
    add("| variant | T1 map p2pl | T2 latency | T3 localization | verdict |")
    add("|---|---|---|---|---|")
    for name, e in variants.items():
        t1 = e.get("t1") or {}
        t2 = e.get("t2") or {}
        t3 = e.get("t3") or {}
        add("| %s | %s | %s | %s | %s |" % (
            name,
            "%s %s m" % (t1.get("verdict"), fmt(t1.get("raw_p2pl_rmse_m"))),
            "%s p95 %s s" % (t2.get("verdict"), fmt(t2.get("latency_p95_s"), 3)),
            "%s ATE %s m" % (t3.get("verdict"), fmt(t3.get("ate_rmse_m"))),
            e.get("verdict")))
    add("")

    for name, e in variants.items():
        add("## Variant `%s`" % name)
        add("")
        t1 = e.get("t1") or {}
        t2 = e.get("t2") or {}
        t3 = e.get("t3") or {}
        add("exit codes: T1=%s T2=%s T3=%s" % (e.get("exit_codes"), "", ""))
        add("")
        add("### T1 - map accuracy vs the synthetic scene reference")
        add("")
        add("- verdict: **%s** (%s)" % (t1.get("verdict"), t1.get("reason") or
                                       "raw value vs %.2f m threshold" % (t1.get("threshold_m") or 0.05)))
        add("- raw est->ref point-to-plane RMSE: **%s m** (threshold %s m), denominator %s,"
            % (fmt(t1.get("raw_p2pl_rmse_m")), fmt(t1.get("threshold_m"), 2),
               t1.get("denominator_fixed_n")))
        add("  fraction within 5 cm %s, stat voxel %s m" % (fmt(t1.get("coverage_within_5cm"), 3),
                                                           t1.get("stat_voxel_m")))
        add("- evaluator status `%s`, pass=%s, worst region %s (p99 %s m)"
            % (t1.get("evaluator_status"), t1.get("evaluator_pass"), t1.get("worst_region"),
               fmt(t1.get("worst_region_p2pl_p99_m"))))
        add("- support: reference coverage of estimate %s, no-reference-locally %s, drifted-out %s"
            % (fmt((t1.get("support") or {}).get("reference_coverage_of_est"), 4),
               fmt((t1.get("support") or {}).get("no_reference_locally_fraction"), 4),
               fmt((t1.get("support") or {}).get("drifted_out_of_support_fraction"), 4)))
        add("- inputs: est map sha256 `%s`, reference sha256 `%s`, ROI sha256 `%s`, frame chain sha256 `%s`"
            % (str((t1.get("inputs") or {}).get("est_map_sha256"))[:16],
               str((t1.get("inputs") or {}).get("ref_map_sha256"))[:16],
               str((t1.get("inputs") or {}).get("roi_sha256"))[:16],
               str((t1.get("inputs") or {}).get("transform_json_sha256"))[:16]))
        add("- evaluator `%s` sha256 `%s`"
            % ((t1.get("evaluator") or {}).get("path"),
               str((t1.get("evaluator") or {}).get("sha256"))[:16]))
        add("")
        add("### T2 - real end-to-end latency / backlog / replay (host only)")
        add("")
        add("- verdict: **%s**, failed checks: %s" % (t2.get("verdict"), t2.get("failed_checks")))
        add("- observed input-ready -> output latency (criterion, no span subtraction): "
            "p95 **%s s**, p99 %s s, median %s s (n=%s, scan period %s s, frame span %s s)"
            % (fmt(t2.get("latency_p95_s"), 3), fmt(t2.get("latency_p99_s"), 3),
               fmt(t2.get("latency_median_s"), 3), t2.get("n_paired"),
               fmt(t2.get("scan_period_s"), 3), fmt(t2.get("frame_span_s"), 3)))
        add("- input arrival jitter (informational, not an availability boundary): p95 %s s"
            % fmt(t2.get("input_arrival_jitter_p95_s"), 4))
        add("- output staleness vs the sensor timeline: sim_lag p95 %s s, max %s s"
            % (fmt(t2.get("sim_lag_p95_s"), 3), fmt(t2.get("sim_lag_max_s"), 3)))
        add("- backlog: final %s s, max %s s; replay wall %s s for %s s simulated (RTF %s)"
            % (fmt(t2.get("backlog_final_s"), 3), fmt(t2.get("backlog_max_s"), 3),
               fmt(t2.get("wall_span_s"), 1), fmt(t2.get("sim_span_s"), 1),
               fmt(t2.get("real_time_factor"), 3)))
        add("- delivered frames / covered input scans: %s / %s = %s (input scans received %s, "
            "%s before the first output frame)"
            % (t2.get("output_frames"), t2.get("covered_scans"), fmt(t2.get("delivery_ratio"), 4),
               t2.get("input_scans"), t2.get("scans_before_first_frame")))
        add("- availability basis: %s" % t2.get("availability_basis"))
        route = t2.get("route") or {}
        add("- GT route: full lap %s (closure %s m, loop %s m), core walking speed min %s / "
            "mean %s / max %s m/s (envelope satisfied: %s)"
            % (route.get("completed_full_lap"), fmt(route.get("lap_closure_error_m"), 4),
               fmt(route.get("loop_length_m"), 1), fmt(route.get("core_speed_min_mps"), 3),
               fmt(route.get("core_speed_mean_mps"), 3), fmt(route.get("core_speed_max_mps"), 3),
               route.get("moving_ge_target_and_le_max")))
        add("- evidence sha256 `%s`" % str((t2.get("evidence") or {}).get("sha256"))[:16])
        add("")
        add("### T3 - separate localizer on the frozen map (no alignment)")
        add("")
        add("- verdict: **%s** (%s)" % (t3.get("verdict"), t3.get("reason")))
        add("- ATE RMSE %s m (threshold %s m); measurement coverage %s of the COMPLETE GT timeline"
            % (fmt(t3.get("ate_rmse_m")), fmt(t3.get("threshold_m"), 2), fmt(t3.get("coverage"), 3)))
        add("- coverage gate %s (protocol choice), tolerance %s s, localization rate %s Hz,"
            % (fmt(t3.get("min_coverage"), 2), fmt(t3.get("coverage_tolerance_s"), 3),
               fmt(t3.get("localization_rate_hz"), 2)))
        add("  coverage ceiling implied by that rate %s" % fmt(t3.get("max_achievable_coverage_at_this_rate"), 3))
        add("- evaluator status `%s`, usable=%s, chain `%s`, association rate %s"
            % (t3.get("evaluator_status"), t3.get("usable_for_accuracy"), t3.get("chain_status"),
               fmt(t3.get("association_rate_est"), 3)))
        add("- startup/unavailable recorded separately: %s" % (t3.get("startup"),))
        add("- initial pose source: %s" % t3.get("initial_pose_source"))
        add("- map<-odom offset non-identity: %s (alignment performed by the test: %s)"
            % (t3.get("map_odom_offset_non_identity"), t3.get("alignment_performed_by_this_test")))
        add("- evaluator sha256 `%s`" % str((t3.get("evaluator") or {}).get("sha256"))[:16])
        add("")
        add("artifacts / logs:")
        for k, v in (e.get("logs") or {}).items():
            add("- %s: `%s`" % (k, v))
        for k, v in (e.get("hashes") or {}).items():
            add("- %s sha256 `%s`" % (k, str(v)[:32]))
        add("")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    with open(args.out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("[make_report] wrote %s (%d lines)" % (args.out, len(lines)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
