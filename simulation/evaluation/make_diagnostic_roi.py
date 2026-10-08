#!/usr/bin/env python3
"""Derive a DIAGNOSTIC-ONLY support region from the reference cloud extent.

Deterministic rule: axis-aligned bounding box of the reference map, expanded by
`--pad` metres.  It depends ONLY on the reference cloud -- never on the
estimated map and never on any residual -- but it is still NOT an
observation-conditioned support region (no visibility / laser-support model), so
it is written with `provenance.diagnostic_only = true`, and
map_accuracy_eval.py then refuses to emit a PASS from it.

The authoritative region must come from the dataset owner and is currently
BLOCKED (see recompute_notes in the audit).
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from eval_common import dump_json, load_points, sha256_file  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ref-map", required=True)
    ap.add_argument("--pad", type=float, default=0.0)
    ap.add_argument("--roi-id", default="diagnostic_ref_aabb")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    pts = load_points(args.ref_map)
    lo = pts.min(axis=0) - args.pad
    hi = pts.max(axis=0) + args.pad
    roi = {
        "schema_version": 1,
        "roi_id": args.roi_id,
        "frame": "reference_map_frame",
        "type": "aabb",
        "bounds": {"min": [float(x) for x in lo], "max": [float(x) for x in hi]},
        "provenance": {
            "source": "reference map extent only: %s" % args.ref_map,
            "derivation": "axis-aligned bounding box of the reference cloud, pad %.3f m; no "
                          "estimate and no residual enters this region" % args.pad,
            "reference_map_sha256": sha256_file(args.ref_map),
            "frozen_utc": None,
            "author": "EvalContract (diagnostic helper)",
            "independent_of_estimate": True,
            "independent_of_error": True,
            "diagnostic_only": True,
            "notes": "DIAGNOSTIC ONLY: not observation-conditioned, cannot carry a PASS. The "
                     "authoritative pre-defined ROI is BLOCKED pending DatasetQualification.",
        },
    }
    dump_json(args.out, roi)
    print("roi %s  min=%s  max=%s  points=%d" % (args.roi_id, np.round(lo, 2), np.round(hi, 2), len(pts)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
