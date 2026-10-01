#!/usr/bin/env python3
"""T1: map accuracy - estimated map -> SYNTHETIC SCENE reference geometry p2pl RMSE <= 5 cm.

Runs the frozen evaluation-contract evaluator (map_accuracy_eval.py) on a map produced by
the simulation harness and reads the HEADLINE raw point-to-plane value from its result JSON.

Everything compared here lives in the synthetic scene's own frame:

  * reference map : scene_reference.py's analytic sampling of the scene surfaces
                    (room box faces + pillars).  NOT the TUHH/MCD/HILTI survey map.
  * support ROI   : scene_reference.py's roi.json, derived from the scene definition only.
  * transform     : make_sim_transform.py's frame_chain.json -- a rigid SE(3) measured over a
                    PRE-DECLARED fit window of the run's odometry; the evaluated map
                    (map_eval.pcd) contains only scans stamped AFTER that window, so the fit
                    and evaluation sample sets are disjoint in time.

Exit codes: 0 = PASS, 1 = FAIL, 2 = BLOCKED (not measurable / no certified value).
A raw value inside the criterion is NOT enough: the evaluator's admissibility gate
(sourced transform, pre-defined ROI, attributed outside-region points, coverage floor,
classifiable support) must also return pass=true.  When it refuses, this test reports
BLOCKED with the evaluator's blocker instead of claiming an accuracy.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402

THRESHOLD_M = 0.05
DEFAULT_EVAL_DIR = os.path.join("artifacts", "04b73f5_evidence_audit_20260929_5V6fKb",
                                "evaluation")
DEFAULT_RUN_DIR = "/tmp/fastlio2_output"
DEFAULT_REF_DIR = "/tmp/fastlio2_scene_ref"


def repo_root():
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def eval_dir_default():
    return os.path.join(repo_root(), DEFAULT_EVAL_DIR)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run-dir", default=DEFAULT_RUN_DIR,
                    help="simulation run directory (map_eval.pcd, frame_chain.json)")
    ap.add_argument("--ref-dir", default=DEFAULT_REF_DIR,
                    help="scene_reference.py output directory (reference_map.pcd, roi.json)")
    ap.add_argument("--est-map", default=None, help="default <run-dir>/map_eval.pcd")
    ap.add_argument("--ref-map", default=None, help="default <ref-dir>/reference_map.pcd")
    ap.add_argument("--roi", default=None, help="default <ref-dir>/roi.json")
    ap.add_argument("--transform-json", default=None, help="default <run-dir>/frame_chain.json")
    ap.add_argument("--outside-attribution", default=None,
                    help="default <run-dir>/outside_attribution.json when that file exists")
    ap.add_argument("--stat-voxel", type=float, default=0.05,
                    help="frozen sampling grid; the evaluator's own default.  A coarser grid "
                         "(e.g. 0.25) offsets the reference and estimate sample sets by half a "
                         "cell and makes the support classification an artefact of that offset")
    ap.add_argument("--sequence", default=None)
    ap.add_argument("--run-name", default="sim_t1")
    ap.add_argument("--eval-dir", default=eval_dir_default())
    ap.add_argument("--out", default=None, help="default <run-dir>/t1_result.json")
    ap.add_argument("--threshold-m", type=float, default=THRESHOLD_M)
    return ap.parse_args(argv)


def check_file(path, what):
    if path and os.path.isfile(path):
        return True
    print("[error] %s not found: %s" % (what, path))
    return False


def main(argv=None):
    args = parse_args(argv)
    run = args.run_dir
    ref = args.ref_dir
    est_map = args.est_map or os.path.join(run, "map_eval.pcd")
    if not os.path.isfile(est_map):
        est_map = args.est_map or os.path.join(run, "map.pcd")
    ref_map = args.ref_map or os.path.join(ref, "reference_map.pcd")
    roi = args.roi or os.path.join(ref, "roi.json")
    transform = args.transform_json or os.path.join(run, "frame_chain.json")
    attribution = args.outside_attribution
    if attribution is None:
        cand = os.path.join(run, "outside_attribution.json")
        attribution = cand if os.path.isfile(cand) else None
    out = args.out or os.path.join(run, "t1_result.json")
    sequence = args.sequence or os.path.basename(os.path.abspath(run))

    evaluator = os.path.join(args.eval_dir, "map_accuracy_eval.py")
    if not os.path.isfile(evaluator):
        print("[error] evaluator not found: %s" % evaluator)
        return 2
    missing = False
    for path, what in ((est_map, "estimated map"), (ref_map, "synthetic scene reference map"),
                       (roi, "scene support ROI"), (transform, "frozen frame chain")):
        missing |= not check_file(path, what)
    if missing:
        print("[blocked] the synthetic scene reference inputs are incomplete; run "
              "simulation/synthetic_data/scene_reference.py and the mapping run first")
        return 2

    cmd = [sys.executable, evaluator,
           "--est-map", est_map,
           "--ref-map", ref_map,
           "--roi", roi,
           "--transform-json", transform,
           "--stat-voxel", str(args.stat_voxel),
           "--run-name", args.run_name,
           "--sequence", sequence,
           "--out", out]
    if attribution:
        cmd += ["--outside-attribution", attribution]
    print("[T1] %s" % " ".join(cmd))
    proc = subprocess.run(cmd, cwd=args.eval_dir, capture_output=True, text=True)
    if proc.returncode != 0:
        print("[error] evaluator exit code %d" % proc.returncode)
        print((proc.stderr or proc.stdout or "").strip()[-2000:])
        return 2
    if not os.path.isfile(out):
        print("[error] evaluator wrote no result: %s" % out)
        return 2

    data = sc.read_json(out)
    verdict = data.get("verdict") or {}
    primary = data.get("primary_metric") or {}
    raw = verdict.get("raw_value_m", data.get("raw_value_m", primary.get("raw_value_m")))
    denom = verdict.get("denominator_fixed_n", data.get("denominator_fixed_n"))
    coverage = verdict.get("coverage_within_5cm")
    pass_flag = data.get("pass")
    status = data.get("status")
    support = data.get("support") or {}
    chain = data.get("transform_record") or {}

    if raw is None:
        print("[blocked] evaluator produced no raw point-to-plane value (status=%s): %s"
              % (status, str(data.get("blocker"))[:400]))
        return 2

    print("T1 建图精度: p2pl raw RMSE %.4f m (threshold %.2f m)" % (raw, args.threshold_m))
    print("  分母(固定) %s, ≤5cm 占比 %s, status=%s, pass=%s"
          % (denom if denom is not None else "N/A",
             ("%.3f" % coverage) if coverage is not None else "N/A", status, pass_flag))
    print("  reference coverage of estimate %.4f, no-reference-locally %.4f"
          % (support.get("reference_coverage_of_est", float("nan")),
             support.get("no_reference_locally_fraction", float("nan"))))
    if chain:
        print("  frame chain: %s fit=[%s] eval=[%s]"
              % (chain.get("source"), chain.get("fit_region"), chain.get("eval_region")))

    verdict = {
        "raw_p2pl_rmse_m": raw, "threshold_m": args.threshold_m,
        "denominator_fixed_n": denom, "coverage_within_5cm": coverage,
        "evaluator_status": status, "evaluator_pass": pass_flag,
        "evaluator_blocker": data.get("blocker"),
        "evaluator": {"path": os.path.abspath(evaluator),
                      "sha256": sc.sha256_file(evaluator)},
        "evaluator_result_json": os.path.abspath(out),
        "inputs": {"est_map": os.path.abspath(est_map),
                   "est_map_sha256": sc.sha256_file(est_map),
                   "ref_map": os.path.abspath(ref_map),
                   "ref_map_sha256": sc.sha256_file(ref_map),
                   "roi": os.path.abspath(roi), "roi_sha256": sc.sha256_file(roi),
                   "transform_json": os.path.abspath(transform),
                   "transform_json_sha256": sc.sha256_file(transform)},
        "support": {"reference_coverage_of_est": support.get("reference_coverage_of_est"),
                    "no_reference_locally_fraction": support.get("no_reference_locally_fraction"),
                    "drifted_out_of_support_fraction": support.get("drifted_out_of_support_fraction")},
        "worst_region": (data.get("regional") or {}).get("worst_region"),
        "worst_region_p2pl_p99_m": (data.get("regional") or {}).get("worst_region_p2pl_p99"),
        "stat_voxel_m": args.stat_voxel,
    }
    verdict_path = os.path.join(run, "t1_verdict.json")
    if pass_flag is True:
        if raw <= args.threshold_m:
            verdict["verdict"] = "PASS"
            sc.write_json(verdict_path, verdict)
            print("✅ PASS: 建图精度 ≤5cm")
            return 0
        verdict["verdict"] = "FAIL"
        sc.write_json(verdict_path, verdict)
        print("❌ FAIL: 精度 %.4fm > %.2fm" % (raw, args.threshold_m))
        return 1
    if pass_flag is False:
        verdict["verdict"] = "FAIL"
        verdict["reason"] = "raw %.4f m > %.2f m" % (raw, args.threshold_m)
        sc.write_json(verdict_path, verdict)
        print("❌ FAIL: 精度 %.4fm > %.2fm (status=%s)" % (raw, args.threshold_m, status))
        return 1
    verdict["verdict"] = "BLOCKED"
    verdict["reason"] = ("the evaluator did not certify the raw value (status=%s): %s"
                         % (status, str(data.get("blocker"))[:300]))
    sc.write_json(verdict_path, verdict)
    print("⚠️  BLOCKED: 原始值 %.4fm 未被评测器认证 (status=%s)" % (raw, status))
    print("  blocker: %s" % str(data.get("blocker"))[:500])
    return 2


if __name__ == "__main__":
    sys.exit(main())
