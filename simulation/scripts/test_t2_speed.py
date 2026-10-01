#!/usr/bin/env python3
"""T2: mapping speed - REAL per-frame latency, backlog, replay wall time and GT speed.

The criterion is the p95 END-TO-END frame latency <= 0.1 s: a 10 Hz LiDAR leaves ~0.1 s per
scan, so a stack slower than that cannot sustain a robot moving >= 0.5 m/s.

Why the output cadence alone is NOT a latency proof
--------------------------------------------------
The interval between consecutive published frames only bounds the latency from below: a
stack that falls behind still publishes at its own pace, and a publisher queue that is
draining produces perfectly regular stamps while the robot's data ages.  So this test uses
the recorder's latency evidence (``<run>/latency_evidence.json``), which timestamps on ONE
monotonic clock both

  * the bag's INPUT scans (/rslidar_points) as the recorder received them, and
  * the node's OUTPUT frames (/fastlio2/lio_odom);

and pairs each output frame with the input scan it was computed from (the odom header stamp
equals the scan header stamp plus one scan period).  The paired difference is the true
per-frame OBSERVED latency (input-ready -> output-ready), not a cadence.  When only cadence
evidence exists this test returns BLOCKED -- never PASS.

Also measured, from the same evidence and the run's record_run.json:

  * accumulated backlog = (wall time elapsed) - (simulated time elapsed) over the run: a
    stack that keeps up has a flat backlog; a stack that falls behind accumulates it;
  * replay wall time and the real-time factor of the replay;
  * the delivered-scan ratio (input scans vs produced odometry frames);
  * the GROUND-TRUTH moving speed of the trajectory (median/p05/max over the moving phase),
    which must sit in [0.5, 1.0] m/s -- the envelope this host-only test is valid for.

This is a HOST measurement only: no Orin/Jetson latency claim is made or implied.

Exit codes: 0 = PASS, 1 = FAIL, 2 = BLOCKED (no usable evidence).
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                                "synthetic_data"))
import sim_common as sc  # noqa: E402
import traj_common as tc  # noqa: E402

DEFAULT_THRESHOLD_S = 0.1
DEFAULT_TRAJ = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                            "synthetic_data", "test_trajectory.json")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run-dir", default="/tmp/fastlio2_output")
    ap.add_argument("--evidence", default=None, help="default <run-dir>/latency_evidence.json")
    ap.add_argument("--record", default=None, help="default <run-dir>/record_run.json")
    ap.add_argument("--trajectory", default=DEFAULT_TRAJ)
    ap.add_argument("--duration", type=float, default=None,
                    help="bag duration in s (default: the trajectory's duration_s)")
    ap.add_argument("--threshold-s", type=float, default=DEFAULT_THRESHOLD_S)
    ap.add_argument("--target-mps", type=float, default=0.5)
    ap.add_argument("--max-mps", type=float, default=1.0)
    ap.add_argument("--max-delivery-loss", type=float, default=0.02,
                    help="allowed fraction of input scans without an odometry frame")
    ap.add_argument("--min-rtf", type=float, default=0.95,
                    help="minimum real-time factor of the replay: below this the measured "
                         "latency is not a real-time claim")
    ap.add_argument("--out", default=None, help="default <run-dir>/t2_result.json")
    return ap.parse_args(argv)


def pair_latency(scans, odoms):
    """Pair each output frame with the input scan it was computed from and measure the OBSERVED
    input-ready -> output-ready delay.

    The bag publishes a complete sweep at its LAST point's readiness (verified per run by
    check_bag_availability.py), so the recorded scan arrival IS the physical availability of the
    frame and no span correction is applied:

      * ``latency`` = recv_odom - recv_scan -- the quantity the ">= 0.5 m/s at 10 Hz" criterion
        is about: everything the stack adds on top of the sensor's own frame time;
      * ``frame_span`` = odom_stamp - scan_stamp -- context only: the frame the output answers
        (the odom stamp is the frame end = scan header + span);
      * ``sim_lag`` = (readiness of the newest frame already received) - odom_stamp -- how stale
        the output is with respect to the sensor timeline the bag had already published.

    The producing scan is the last one stamped before the odom stamp (causal pairing).
    """
    if len(scans) < 3 or len(odoms) < 3:
        return {}
    period = float(np.median(np.diff(scans[:, 0])))
    if not (0.0 < period < 5.0):
        return {}
    idx = np.searchsorted(scans[:, 0], odoms[:, 0], side="left") - 1
    ok = idx >= 0
    idx = idx[ok]
    od = odoms[ok]
    frame_span = float(np.median(od[:, 0] - scans[idx, 0]))
    if not (0.0 < frame_span <= period + 1e-6):
        return {}
    latency = od[:, 1] - scans[idx, 1]
    # staleness: the newest frame READY at the moment the odom arrived, measured against the
    # frame the output answers (the scan is ready at header + frame span, not at its header)
    j = np.searchsorted(scans[:, 1], od[:, 1], side="right") - 1
    j = np.clip(j, 0, len(scans) - 1)
    sim_lag = (scans[j, 0] + frame_span) - od[:, 0]
    return {"period": period, "frame_span": frame_span,
            "latency": latency, "sim_lag": sim_lag, "n": int(len(od))}


def backlog_series(scans, odoms):
    """Running (wall elapsed - sim elapsed) over the odom stream, in seconds."""
    if len(odoms) < 3:
        return np.empty(0)
    wall = odoms[:, 1] - odoms[0, 1]
    sim = odoms[:, 0] - odoms[0, 0]
    return wall - sim


def main(argv=None):
    args = parse_args(argv)
    run = args.run_dir
    evidence_path = args.evidence or os.path.join(run, "latency_evidence.json")
    record_path = args.record or os.path.join(run, "record_run.json")
    out = args.out or os.path.join(run, "t2_result.json")

    if not os.path.isfile(evidence_path):
        print("[blocked] no latency evidence: %s" % evidence_path)
        print("  T2 needs the recorder's input-scan + output-frame timestamps; an output cadence "
              "alone can never prove a latency.")
        return 2
    ev = sc.read_json(evidence_path)
    # Precondition (fail closed when the evidence exists): the bag's time contract must show that
    # each complete sweep is published at its LAST point's readiness, which is what makes the
    # recorded input arrival the physical availability of the frame (so the paired delay is an
    # observed end-to-end latency with no assumed boundary).
    avail_path = os.path.join(run, "bag_availability.json")
    avail = sc.read_json(avail_path) if os.path.isfile(avail_path) else None
    if avail is not None:
        bad = [k for k, v in (avail.get("checks") or {}).items() if not v]
        if bad:
            print("[blocked] the bag time contract is not verified (%s): %s" % (avail_path, bad))
            print("  T2 pairs the input arrival with the output arrival directly, which is only "
                  "physical when a complete sweep is published at its last point's readiness.")
            return 2
    scans = np.asarray(ev.get("scan_events") or [], float).reshape(-1, 2)
    odoms = np.asarray(ev.get("odom_events") or [], float).reshape(-1, 2)
    record = sc.read_json(record_path) if os.path.isfile(record_path) else {}
    wall = (record.get("wall_clock") or ev.get("wall_clock") or {})

    if len(scans) < 3:
        print("[blocked] the evidence contains %d input-scan timestamps: the output cadence "
              "alone is NOT a latency proof (a falling-behind stack still publishes "
              "regularly)." % len(scans))
        return 2
    if len(odoms) < 3:
        print("[blocked] the evidence contains %d output frames -- nothing was processed."
              % len(odoms))
        return 2

    lat = pair_latency(scans, odoms)
    if lat.get("n", 0) < 3:
        print("[blocked] could not pair output frames with input scans: the header stamps do not "
              "identify the producing scan (or the frame span is not inside one scan period).")
        return 2
    period = lat["period"]
    frame_span = lat["frame_span"]
    observed = lat["latency"]
    sim_lag = lat["sim_lag"]
    lat = observed

    p95 = float(np.percentile(lat, 95))
    p99 = float(np.percentile(lat, 99))
    med = float(np.median(lat))
    bl = backlog_series(scans, odoms)
    backlog_final = float(bl[-1]) if bl.size else None
    backlog_max = float(bl.max()) if bl.size else None
    backlog_growth = float(bl[-1] - bl[0]) if bl.size else None

    sim_span = float(odoms[-1, 0] - odoms[0, 0])
    wall_span = float(odoms[-1, 1] - odoms[0, 1])
    rtf = sim_span / wall_span if wall_span > 0 else None
    # Scans published before the node finished IMU initialisation cannot produce odometry;
    # the delivery ratio is measured over the scans inside the output's own time window.
    covered = scans[scans[:, 0] >= odoms[0, 0] - period]
    delivery = len(odoms) / float(len(covered)) if len(covered) else 0.0

    # Availability basis: each complete sweep is published at its LAST point's readiness, so the
    # recorded input arrival IS the physical availability of the frame and the paired delay is
    # measured directly (check_bag_availability.py verifies that contract per run).  The arrival
    # timestamps still carry the recorder's own scheduling jitter; it is reported as information
    # about the measurement, not used as an availability boundary.
    scan_lag = scans[:, 1] - scans[:, 0]
    scan_jitter_p95 = float(np.percentile(np.abs(scan_lag - np.median(scan_lag)), 95))


    traj = tc.apply_variant(tc.load_trajectory(args.trajectory))
    duration = float(traj["duration_s"] if args.duration is None else args.duration)
    routes = tc.route_stats(traj, duration_s=duration, target_mps=args.target_mps,
                            max_mps=args.max_mps)

    print("T2 建图速度(主机端，非 Orin): p95 观测时延(输入帧就绪->输出) %.3f s, p99 %.3f s, "
          "median %.3f s (n=%d, scan period %.3f s, frame span %.3f s)"
          % (p95, p99, med, lat.size, period, frame_span))
    print("  输入到达抖动(仅记录，不作为可用性边界): p95 %.4f s" % scan_jitter_p95)
    print("  输出相对传感器时间线的新鲜度 sim_lag: p95 %.3f s, max %.3f s"
          % (float(np.percentile(sim_lag, 95)), float(sim_lag.max())))
    print("  backlog: final %.3f s, max %.3f s (wall-sim over the odom stream)" %
          (backlog_final or 0.0, backlog_max or 0.0))
    print("  replay wall %.1f s for %.1f s simulated -> RTF %.3f; recorder wall %.1f s"
          % (wall_span, sim_span, rtf or 0.0, float(wall.get("duration_s") or 0.0)))
    print("  delivered frames/covered scans %d/%d = %.4f (lost %.4f; %d scans before init; "
          "input arrival jitter p95 %.4f s)"
          % (len(odoms), len(covered), delivery, 1.0 - delivery,
             int(len(scans) - len(covered)), scan_jitter_p95))
    print("  GT route: full lap=%s closure=%.4f m, loop=%.1f m; core walking speed "
          "min %.3f mean %.3f max %.3f m/s (vibration makes the instantaneous value oscillate); "
          "envelope [%.2f, %.2f] m/s satisfied=%s"
          % (routes["completed_full_lap"], routes["lap_closure_error_m"],
             routes["loop_length_m"], routes["core_speed_min_mps"] or 0.0,
             routes["core_speed_mean_mps"] or 0.0, routes["core_speed_max_mps"] or 0.0,
             args.target_mps, args.max_mps, routes["moving_ge_target_and_le_max"]))

    result = {
        "host_only": True,
        "orin_claim": False,
        "mode": "causal input-scan -> output-frame pairing; the OBSERVED input-ready -> "
                "output-ready delay is the criterion (the bag publishes each complete sweep at "
                "its last point's readiness, so no frame span is subtracted)",
        "n_paired": int(lat.size), "scan_period_s": period, "frame_span_s": frame_span,
        "latency_p95_s": p95, "latency_p99_s": p99, "latency_median_s": med,
        "latency_max_s": float(lat.max()),
        "observed_latency_p95_s": p95,
        "observed_latency_median_s": med,
        "sim_lag_p95_s": float(np.percentile(sim_lag, 95)),
        "sim_lag_max_s": float(sim_lag.max()),
        "threshold_s": args.threshold_s,
        "availability_basis": "the input scan message is published when its LAST point has been "
                             "measured (verified per run by check_bag_availability.py: storage "
                             "stamp == header + frame span), so the recorded input arrival IS the "
                             "physical availability of the frame and the paired delay is measured "
                             "directly with no span subtraction",
        "input_arrival_jitter_p95_s": scan_jitter_p95,
        "backlog_final_s": backlog_final, "backlog_max_s": backlog_max,
        "backlog_growth_s": backlog_growth,
        "sim_span_s": sim_span, "wall_span_s": wall_span, "real_time_factor": rtf,
        "recorder_wall_s": wall.get("duration_s"),
        "input_scans": int(len(scans)), "covered_scans": int(len(covered)),
        "scans_before_first_frame": int(len(scans) - len(covered)),
        "output_frames": int(len(odoms)),
        "delivery_ratio": delivery,
        "route": routes,
        "checks": {
            "p95_observed_latency_within_threshold": bool(p95 <= args.threshold_s),
            "sim_lag_bounded": bool(float(np.percentile(sim_lag, 95)) <= args.threshold_s),
            "backlog_bounded": bool(backlog_max is not None
                                    and backlog_max <= 2.0 * args.threshold_s),
            "no_scan_loss": bool(1.0 - delivery <= args.max_delivery_loss),
            "real_time_replay": bool(rtf is not None and rtf >= args.min_rtf),
            "gt_full_lap": bool(routes["completed_full_lap"]),
            "gt_moving_speed_in_envelope": bool(routes["moving_ge_target_and_le_max"]),
        },
    }
    sc.write_json(out, result)
    failed = [k for k, v in result["checks"].items() if not v]
    verdict = dict(result)
    verdict["verdict"] = "PASS" if not failed else "FAIL"
    verdict["failed_checks"] = failed
    verdict["evidence"] = {"path": os.path.abspath(evidence_path),
                           "sha256": sc.sha256_file(evidence_path)}
    sc.write_json(os.path.join(run, "t2_verdict.json"), verdict)
    if not failed:
        print("✅ PASS: p95 观测时延 %.3fs ≤ %.1fs，sim_lag 有界，backlog 有界，无丢帧，"
              "RTF %.3f，GT 步行速度均值 %.3f m/s"
              % (p95, args.threshold_s, rtf or 0.0, routes["core_speed_mean_mps"] or 0.0))
        print("  (主机端测量；不对 Orin/实机时延作任何断言)")
        return 0
    print("❌ FAIL: 未通过的检查 %s" % ", ".join(failed))
    return 1


if __name__ == "__main__":
    sys.exit(main())
