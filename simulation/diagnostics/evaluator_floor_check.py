#!/usr/bin/env python3
"""Evaluator sanity check: run the frozen T1 evaluator on the REFERENCE as the estimate.

A perfect map must give a ~0 p2pl residual and pass every admissibility gate (identity
transform, same ROI, no outside points, full support).  This validates that a post-fix number
is being produced by a consistent evaluator/reference pair rather than by an artefact.
"""
import json
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
EVAL_DIR = ROOT + "/artifacts/04b73f5_evidence_audit_20260929_5V6fKb/evaluation"
EVAL = EVAL_DIR + "/map_accuracy_eval.py"

ref_dir = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/diag/self_check_result.json"

src = json.load(open(sys.argv[3])) if len(sys.argv) > 3 else None
I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
transform = dict(src) if src else {}
transform.update({"matrix": I, "matrix_ref_est": I})
transform["notes"] = ["DIAGNOSTIC ONLY: the run's sourced, pre-eval frame_chain record with the "
                      "matrix replaced by identity, to measure the evaluator floor on the "
                      "reference cloud used as its own estimate."]
tpath = "/tmp/diag/identity_frame_chain.json"
with open(tpath, "w") as fh:
    json.dump(transform, fh, indent=1)

cmd = [sys.executable, EVAL,
       "--est-map", os.path.join(ref_dir, "reference_map.pcd"),
       "--ref-map", os.path.join(ref_dir, "reference_map.pcd"),
       "--roi", os.path.join(ref_dir, "roi.json"),
       "--transform-json", tpath,
       "--stat-voxel", "0.05", "--run-name", "self_check", "--sequence", "identity",
       "--out", out]
print(" ".join(cmd))
p = subprocess.run(cmd, cwd=EVAL_DIR, capture_output=True, text=True)
print(p.stdout[-1500:]); print(p.stderr[-800:])
d = json.load(open(out))
print("status", d.get("status"), "pass", d.get("pass"))
print("raw p2pl", (d.get("primary_metric") or {}).get("raw_value_m"))
print("preconditions", d.get("pass_preconditions"))
