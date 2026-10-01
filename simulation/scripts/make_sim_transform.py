#!/usr/bin/env python3
"""Derive the FROZEN rigid frame chain between the estimator's world frame and the scene.

The estimator (fastlio2) initialises its world frame at the body's own start pose, so the
estimated map/odometry and the synthetic scene frame differ by a rigid SE(3).  This script
measures that SE(3) by Umeyama alignment (scale fixed at 1) of the recorded estimated
odometry against the TRUE trajectory of the same run -- but ONLY over a PRE-DECLARED fit
window, and it records the fit and evaluation time ranges explicitly:

  * fit  window  : [t0, t0 + fit_until_s)          -- trajectory samples used to measure SE(3)
  * eval window  : [t0 + fit_until_s, run_end)     -- the map messages T1 evaluates

record_run.py writes ``map_eval.pcd`` from world_cloud messages stamped inside the EVAL
window only, and each such message is one scan in the world frame, so the fit sample set and
the evaluated map point set are provably disjoint in time.  The transform is applied RIGIDLY:
no ICP, no per-block registration, no map-to-map fitting.

Disclosure recorded in the output (Main correction): the frame chain is measured from the
same run's odometry, i.e. it is MAP-DERIVED in the sense that it shares the estimator with
the map it is applied to.  It is NOT fitted on the evaluation samples (disjoint time indices)
and the simulation runs no PGO/global optimisation, so the early trajectory used for the fit
is pre-optimisation (there is no later optimisation to leak).

Optionally applies the chain to a recorded map and writes the transformed map (used as the
FROZEN prior map for T3's separate localization run).

Usage:
    python3 make_sim_transform.py --gt-tum <scene gt.tum> --odom-tum <run odom.tum> \
        --t0-epoch 1700000000 --fit-until-s 30 --run-end-s 90 \
        --apply-map <run>/map.pcd --out-map <run>/frozen_map.pcd --out <run>/frame_chain.json
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402

TOOL = "make_sim_transform.py v1"


def read_tum(path):
    rows = np.loadtxt(path, ndmin=2)
    if rows.size == 0:
        raise SystemExit("[error] empty TUM file: %s" % path)
    return rows


def rigid_align(src, dst):
    """Rigid SE(3) (scale fixed at 1) mapping src -> dst, Horn/Umeyama via SVD."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(u @ vt))
    rot = u @ np.diag([1.0, 1.0, d]) @ vt
    return rot, mu_d - rot @ mu_s


def rotation_deg(rot):
    c = (np.trace(rot) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def build(gt_tum, odom_tum, t0_epoch, fit_until_s, run_end_s, frame_from, frame_to,
          fit_region_id, eval_region_id, min_fit_samples, gap_s=0.5):
    gt = read_tum(gt_tum)
    od = read_tum(odom_tum)
    tg = gt[:, 0] - float(t0_epoch)
    to = od[:, 0] - float(t0_epoch)
    fit_hi = float(fit_until_s)
    gap = float(gap_s)
    fit_hi_declared = fit_hi - gap
    run_end = float(run_end_s) if run_end_s is not None else float(max(to[-1], tg[-1]))

    def interp_gt(t):
        return np.stack([np.interp(t, tg, gt[:, 1 + k]) for k in range(3)], axis=1)

    sel = to <= fit_hi_declared
    n_fit = int(sel.sum())
    if n_fit < int(min_fit_samples):
        raise SystemExit("[error] only %d odometry samples inside the fit window [%.1f, %.1f] s "
                         "(need >= %d): the frame chain cannot be measured"
                         % (n_fit, float(to[0]), fit_hi_declared, int(min_fit_samples)))
    src = od[sel, 1:4]
    dst = interp_gt(to[sel])
    rot, trans = rigid_align(src, dst)
    resid = np.linalg.norm((rot @ src.T).T + trans - dst, axis=1)

    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans
    rec = {
        "tool": TOOL,
        "source": ("rigid SE(3) (Umeyama, scale fixed at 1) between the run's estimated "
                   "odometry and the TRUE trajectory of the same run, measured over the "
                   "pre-declared fit window only"),
        "source_artifact": os.path.abspath(odom_tum),
        "source_artifact_sha256": sc.sha256_file(odom_tum),
        "gt_artifact": os.path.abspath(gt_tum),
        "gt_artifact_sha256": sc.sha256_file(gt_tum),
        "independent_of_evaluation": True,
        "frame_from": frame_from,
        "frame_to": frame_to,
        "fit_region": {"roi_id": fit_region_id,
                       "description": "trajectory samples with sim time <= %.1f s"
                                      % fit_hi_declared},
        "eval_region": {"roi_id": eval_region_id,
                        "description": "world_cloud messages with sim time >= %.1f s" % fit_hi},
        "fit_time_ranges": [[float(to[0]), fit_hi_declared]],
        "eval_time_ranges": [[fit_hi, run_end]],
        "fit_eval_gap_s": gap,
        "time_domain": "sim_time_rel",
        "matrix": [[float(v) for v in row] for row in T],
        # map_accuracy_eval.py reads "matrix_ref_est" (ref<-est) and applies it to the estimate
        "matrix_ref_est": [[float(v) for v in row] for row in T],
        "fit_samples": n_fit,
        "fit_rmse_m": float(np.sqrt((resid ** 2).mean())),
        "fit_max_err_m": float(resid.max()),
        "fit_rotation_deg": rotation_deg(rot),
        "fit_translation_m": [float(v) for v in trans],
        "notes": [
            "Disclosure (Main correction): the chain is measured from the SAME run's odometry, "
            "so it is map-derived; it is not fitted on the evaluation samples (the fit and eval "
            "time ranges above are disjoint) and no ICP or map-to-map registration is used.",
            "The simulation runs lio_node only (no PGO/global optimisation), so the early "
            "trajectory used for the fit is pre-optimisation: there is no later optimisation "
            "whose effect could leak into the transform.",
            "The transform is applied rigidly (one SE(3), scale 1) to the whole estimated map; "
            "no per-region or per-block registration is performed.",
        ],
    }
    return rec, T


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--gt-tum", required=True, help="scene-frame GT TUM of the same run")
    ap.add_argument("--odom-tum", required=True, help="estimated odometry TUM of the run")
    ap.add_argument("--t0-epoch", type=float, required=True,
                    help="trajectory t0_epoch_s, to convert epoch stamps to sim seconds")
    ap.add_argument("--fit-until-s", type=float, required=True,
                    help="fit window is [run start, t0 + this) in sim seconds")
    ap.add_argument("--run-end-s", type=float, default=None)
    ap.add_argument("--frame-from", default="est_world_frame")
    ap.add_argument("--frame-to", default="scene")
    ap.add_argument("--fit-region-id", default="sim_fit_window")
    ap.add_argument("--eval-region-id", default="sim_eval_window")
    ap.add_argument("--min-fit-samples", type=int, default=20)
    ap.add_argument("--gap-s", type=float, default=0.5,
                    help="strict gap between the declared fit and eval time ranges, so the two "
                         "interval sets cannot even touch (the evaluator treats touching as overlap)")
    ap.add_argument("--apply-map", default=None,
                    help="PCD in the est frame to transform into the scene frame")
    ap.add_argument("--out-map", default=None)
    ap.add_argument("--out", required=True, help="transform record JSON")
    args = ap.parse_args(argv)

    rec, T = build(args.gt_tum, args.odom_tum, args.t0_epoch, args.fit_until_s,
                   args.run_end_s, args.frame_from, args.frame_to, args.fit_region_id,
                   args.eval_region_id, args.min_fit_samples, args.gap_s)
    if args.apply_map:
        if not args.out_map:
            raise SystemExit("[error] --apply-map requires --out-map")
        pts = sc.read_pcd(args.apply_map)
        out = (T[:3, :3] @ pts.T).T + T[:3, 3]
        sc.write_pcd(args.out_map, out)
        rec["applied_to_map"] = {
            "input": os.path.abspath(args.apply_map),
            "input_sha256": sc.sha256_file(args.apply_map),
            "output": os.path.abspath(args.out_map),
            "output_sha256": sc.sha256_file(args.out_map),
            "points": int(len(out)),
            "output_bbox": [[float(x) for x in out.min(axis=0)],
                            [float(x) for x in out.max(axis=0)]] if out.size else None,
        }
    sc.write_json(args.out, rec)
    print("[make_sim_transform] fit %d samples over [%.1f, %.1f) s: rmse=%.4f m max=%.4f m, "
          "rot=%.3f deg, trans=[%.3f %.3f %.3f]"
          % (rec["fit_samples"], rec["fit_time_ranges"][0][0], args.fit_until_s,
             rec["fit_rmse_m"], rec["fit_max_err_m"], rec["fit_rotation_deg"],
             rec["fit_translation_m"][0], rec["fit_translation_m"][1], rec["fit_translation_m"][2]))
    print("[make_sim_transform] wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
