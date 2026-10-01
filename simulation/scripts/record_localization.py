#!/usr/bin/env python3
"""Record one T3 localization run: the localizer's map->body pose stream + its evidence.

The real localizer_node is a SEPARATE consumer of the mapping stack's outputs:

  lio_node   publishes  /fastlio2/body_cloud (body frame) and /fastlio2/lio_odom (odom frame)
  localizer  consumes both, aligns the body cloud against a FROZEN prior PCD map and
             broadcasts TF  map -> odom

This recorder composes that chain into the localization estimate under a FROZEN CONSUMER
RULE (the audit rule of artifacts/04b73f5_evidence_audit_20260929_5V6fKb/evaluation/
consumer_rule_frozen_v1.json):

    query(t)        = the latest map->odom BROADCAST with stamp <= t   (zero-order hold)
    hold_validity_s = HOLD_VALIDITY_S;  a held sample older than that is UNAVAILABLE
    failure         = counted separately (no_tf_at_or_before / too_old / gate_invalid)

    T_map_body(t)   = T_map_odom(held at t) * T_odom_body(t)

Nothing is ever interpolated: a query that fails stays failed and is counted, and a sample
is only emitted when the localizer's own gate (relocalize_check) said the lock was valid at
that moment.  The census of query age and failures is written next to the estimate
(`tf_availability.json`), because a sampled estimate stream alone says NOTHING about online
availability: a 5 Hz output can be caused by a CPU-bound ICP, and an 0.1 Hz output can still
be perfectly available.

The recorder never substitutes the mapping run's odometry for the localization: the composed
pose uses the localizer's own map->odom broadcast, whose non-identity statistics are recorded
as evidence that the localizer really ran and really produced an offset.  The pre-adoption
samples (the initialised identity offset the node broadcasts before it adopts its first
candidate) are dropped and counted, and so are the samples the gate declared invalid.

Initial pose: the node needs a starting guess (its ICP search radius is ~0.35 m).  This
recorder calls the node's `relocalize` service ONCE, shortly after the first odometry frame,
with the DECLARED START POSE of the trajectory definition (the robot is still trotting in
place at its known placement, so the body really is at that pose).  The pose source is
recorded verbatim in the output manifest: it is a declared placement / GT-assisted
initialisation, NOT an estimate of the run, and it is not hidden.

Usage:
    python3 record_localization.py --out-dir /tmp/sim_loc --duration 90 \
        --initial-pose-x 7 --initial-pose-y 3 --initial-pose-z 0.5 --initial-pose-yaw 0
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import sim_common as sc  # noqa: E402

# ---------------------------------------------------------------- frozen protocol --
# Pre-declared BEFORE the run and recorded in the output; none of these may be widened
# after seeing a score.  They are the audit consumer rule (consumer_rule_frozen_v1.json)
# plus the staleness rule of the localizer's own validity answer.
HOLD_VALIDITY_S = 0.10      # zero-order hold: max age of a held map<-odom sample
ASSOC_TOLERANCE_S = 0.05    # same 0.05 s the evaluator associates GT with; never widened
GAP_LIMIT_S = 0.20          # widest gap across which nothing is bridged
VALIDITY_STALE_S = 1.0      # a relocalize_check answer older than this proves nothing
STAMP_RESOLUTION_S = 1e-9   # ROS stamps are integer nanoseconds: the age bound is compared at
                            # the timestamp resolution, so an age that ROUNDS to the bound is
                            # inside it (not a widening - it is the bound's own quantisation)
VALID_POLL_S = 0.2          # how often the localizer's validity answer is sampled


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], float)


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], float)
    return q / np.linalg.norm(q)


class LocalizationRecorder(Node):
    def __init__(self, out_dir, duration, idle_timeout, ns="/fastlio2", localizer_ns="/localizer",
                 map_frame="map", odom_frame="odom", body_frame="body"):
        super().__init__("sim_localization_recorder")
        self.out_dir = out_dir
        self.deadline = time.monotonic() + duration if duration > 0 else float("inf")
        self.idle_timeout = idle_timeout
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.body_frame = body_frame
        os.makedirs(out_dir, exist_ok=True)
        self.map_odom = {}          # stamp -> (t(3,), q(4,))
        self.odom_body = {}         # stamp -> (t(3,), q(4,))
        self.odom_arrival = {}      # odom stamp -> recorder wall time it was received
        self.tf_samples = []        # (arrival_wall, stamp, t(3,), q(4,)) per broadcast
        self.tf_arrival = []        # (wall, stamp) of every map->odom broadcast, in order
        self.valid_log = []         # (response_wall, valid) samples of the localizer's gate
        self.clock_log = []         # (arrival_wall, bag clock seconds) from /clock, so a query
                                    # can be scored against the robot's state at the time the
                                    # consumer actually asked, not at the delayed frame's stamp
        self._valid_pending = None
        self._valid_pending_wall = None
        self.wall_start = time.monotonic()
        self.wall_end = None
        self.last_msg = time.monotonic()
        self.relocalize = None
        self.reloc_sent = False
        self.reloc_result = None
        self.reloc_request = None
        self.first_odom_wall = None
        self.skipped_pre_lock = 0
        self.valid_checks = []
        self.prior_map = ""
        self.initial_pose_xyz = (0.0, 0.0, 0.0)
        self.initial_pose_rpy = (0.0, 0.0, 0.0)
        self.reloc_delay_s = 1.0
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, ns + "/lio_odom", self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        try:
            from interface.srv import Relocalize, IsValid
            self.Relocalize, self.IsValid = Relocalize, IsValid
            self.relocalize = self.create_client(Relocalize, localizer_ns + "/relocalize")
            self.reloc_check = self.create_client(IsValid, localizer_ns + "/relocalize_check")
        except ImportError as exc:                                # pragma: no cover
            print("[record_localization] WARNING: interface.srv unavailable (%s); the initial "
                  "pose cannot be sent and the localizer will only lock if its ICP finds the "
                  "solution from identity" % exc, file=sys.stderr)
            self.relocalize = None
            self.reloc_check = None

    # ---------------------------------------------------------------- inputs --
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_body[t] = (np.array([p.x, p.y, p.z], float),
                             np.array([q.x, q.y, q.z, q.w], float))
        self.odom_arrival[t] = time.monotonic()
        if self.first_odom_wall is None:
            self.first_odom_wall = time.monotonic()
        self.last_msg = time.monotonic()

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id != self.map_frame or tr.child_frame_id != self.odom_frame:
                continue
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            t_mo = np.array([tr.transform.translation.x, tr.transform.translation.y,
                             tr.transform.translation.z], float)
            q_mo = np.array([tr.transform.rotation.x, tr.transform.rotation.y,
                             tr.transform.rotation.z, tr.transform.rotation.w], float)
            self.map_odom[t] = (t_mo, q_mo)
            wall = time.monotonic()
            self.tf_arrival.append((wall, t))
            self.tf_samples.append((wall, t, t_mo, q_mo))
            self.last_msg = time.monotonic()

    def on_clock(self, msg):
        # bag playback publishes /clock; the last value at or before a query is the robot's
        # current physical time at that query
        self.clock_log.append((time.monotonic(),
                               msg.clock.sec + msg.clock.nanosec * 1e-9))

    def clock_at(self, query_wall):
        """Bag clock time current at query_wall, or None when /clock was never seen."""
        best = None
        for wall, t in self.clock_log:
            if wall <= query_wall:
                best = t
            else:
                break
        return best

    def maybe_relocalize(self):
        """Send the declared initial pose once the node has a synced cloud+odom pair.

        Runs from the main loop (never from a subscription callback) so the synchronous
        service call cannot re-enter the executor.
        """
        if self.reloc_sent or self.relocalize is None or self.first_odom_wall is None:
            return
        if time.monotonic() - self.first_odom_wall < self.reloc_delay_s:
            return
        self.reloc_sent = True
        if not self.relocalize.wait_for_service(timeout_sec=10.0):
            self.reloc_result = {"success": False, "message": "relocalize service unavailable"}
            return
        req = self.Relocalize.Request()
        req.pcd_path = self.prior_map or ""
        req.x, req.y, req.z = [float(v) for v in self.initial_pose_xyz]
        req.roll, req.pitch, req.yaw = [float(v) for v in self.initial_pose_rpy]
        self.reloc_request = {"pcd_path": req.pcd_path, "x": req.x, "y": req.y, "z": req.z,
                              "roll": req.roll, "pitch": req.pitch, "yaw": req.yaw,
                              "source": "declared start pose of the localization trajectory "
                                        "(GT-assisted initialisation, recorded explicitly)"}
        fut = self.relocalize.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        res = fut.result()
        self.reloc_result = ({"success": bool(res.success), "message": str(res.message)}
                             if res is not None else {"success": False,
                                                      "message": "service call timed out"})
        print("[record_localization] relocalize -> %s" % self.reloc_result)

    def poll_valid(self):
        """Sample the localizer's own lock-validity answer; never blocks the recorder.

        The answer is only evidence together with the wall time at which it was asked, so the
        whole (wall, valid) series is recorded.  The node answers from its last gate update, so
        a localizer whose intake has stalled would otherwise keep answering "valid" forever;
        the freshness of the answer is what makes it meaningful.
        """
        if self.reloc_check is None or not self.reloc_check.service_is_ready():
            return
        if self._valid_pending is not None:
            if not self._valid_pending.done():
                return
            res = self._valid_pending.result()
            val = bool(res.valid) if res is not None else None
            # The answer describes the node's state when it ANSWERED, so it is evidence from
            # the moment the response arrived - dating it at the request time would let a slow
            # service backfill a query that was raised before the answer existed.
            responded = time.monotonic()
            self.valid_log.append((responded, val))
            self.valid_checks.append([self._valid_pending_wall - self.wall_start, val,
                                      responded - self._valid_pending_wall])
            self._valid_pending = None
        req = self.IsValid.Request()
        req.code = 0
        self._valid_pending_wall = time.monotonic()
        self._valid_pending = self.reloc_check.call_async(req)

    def finished(self):
        if time.monotonic() > self.deadline:
            return True
        return bool(self.odom_body) and (time.monotonic() - self.last_msg) > self.idle_timeout

    # ------------------------------------------------ the frozen consumer rule --
    def finite_span(self):
        """(start, end) of the odometry stamp span, or None."""
        if not self.odom_body:
            return None
        st = sorted(self.odom_body)
        return (float(st[0]), float(st[-1]))

    def valid_answer_at(self, query_wall, series):
        """The last validity answer RECEIVED by query_wall, or None.

        `series` is the (response_wall, valid) list: an answer is evidence for a query only if
        it had already been received when that query was raised.  Its own value already carries
        the localizer's accepted-update freshness bound (validity_timeout_s), so an answer that
        arrived is enough; one that did not arrive yet is not, and neither is one that is older
        than VALIDITY_STALE_S at the query.
        """
        best = None
        for wall, val in series:
            if wall <= query_wall:
                best = (wall, val)
            else:
                break
        if best is None:
            return None
        if (query_wall - best[0]) > VALIDITY_STALE_S:
            return None
        return best[1]

    def compose(self):
        """Compose the online T_map_body stream and measure two separate things.

        STREAM (what a real consumer gets): a query raised when odometry frame t ARRIVES may
        only use a map<-odom correction that had already ARRIVED, and only a validity answer
        already RECEIVED.  The accepted correction is then propagated over the odometry, which
        is what "propagate odom under the last accepted map correction" means - no
        re-stamping, no interpolation, no backfill.  The correction's age is NOT used to
        discard the sample: it is measured and reported.

        CENSUS A (declared contract, the GATED number): correction age
        `query_stamp - tf_message_stamp` <= HOLD_VALIDITY_S, with receipt causality and gate
        validity required as well.  The TF message stamp IS the stamp of the frame the ICP
        solved for (the node broadcasts exactly one sample per accepted update, stamped with
        that frame), so this age is the accepted-alignment age by construction; the recorder
        verifies the receipt side separately instead of assuming it.

        CENSUS B (diagnostic): the same without the age bound, i.e. "was a live, causal,
        gate-valid correction available at all".  Its failure reasons separate an absent/stale
        service from a service that is alive but slower than the declared age contract.
        """
        queries = sorted(self.odom_body, key=lambda t: self.odom_arrival[t])
        tf_by_arrival = sorted(self.tf_samples, key=lambda r: r[0])
        series = sorted(self.valid_log)
        keys = ("queries", "covered", "no_tf_received_before", "too_old", "no_valid_answer",
                "gate_not_valid", "identity_offset_samples")
        age_census = dict.fromkeys(keys, 0)
        causal_census = dict.fromkeys(keys, 0)
        causal_census["too_old"] = 0  # the age bound is not applied in census B
        ages, causal_ages, rows, off_mo = [], [], [], []
        rows_online, clock_lags = [], []
        for t in queries:
            w = self.odom_arrival[t]
            age_census["queries"] += 1
            causal_census["queries"] += 1
            held = None
            for arrival, stamp, t_mo, q_mo in tf_by_arrival:
                if arrival > w:
                    break  # had not arrived yet when this odometry frame arrived
                if stamp <= t:
                    held = (stamp, t_mo, q_mo)
            if held is None:
                age_census["no_tf_received_before"] += 1
                causal_census["no_tf_received_before"] += 1
                continue
            stamp, t_mo, q_mo = held
            age = float(t) - float(stamp)
            ages.append(age)
            answer = self.valid_answer_at(w, series)
            if answer is None:
                age_census["no_valid_answer"] += 1
                causal_census["no_valid_answer"] += 1
                continue
            if answer is not True:
                age_census["gate_not_valid"] += 1
                causal_census["gate_not_valid"] += 1
                continue
            causal_ages.append(age)
            causal_census["covered"] += 1
            R_mo = quat_to_mat(q_mo)
            t_ob, q_ob = self.odom_body[t]
            R = R_mo @ quat_to_mat(q_ob)
            p = t_mo + R_mo @ t_ob
            q = mat_to_quat(R)
            rows.append((t, p[0], p[1], p[2], q[0], q[1], q[2], q[3]))
            # the SAME pose, stamped at the time the consumer actually asked (/clock at the
            # query).  Scoring this stream against the ground truth is the current-time
            # consumer error, not the delayed-pose error of the frame the correction came from.
            t_now = self.clock_at(w)
            if t_now is not None:
                rows_online.append((t_now, p[0], p[1], p[2], q[0], q[1], q[2], q[3]))
                clock_lags.append(float(t_now) - float(t))
            off_mo.append(t_mo)
            ang = math.acos(max(-1.0, min(1.0, (float(np.trace(R_mo)) - 1.0) / 2.0)))
            if float(np.linalg.norm(t_mo)) < 0.01 and ang < math.radians(0.5):
                causal_census["identity_offset_samples"] += 1
                age_census["identity_offset_samples"] += 1
            if age <= HOLD_VALIDITY_S + STAMP_RESOLUTION_S:
                age_census["covered"] += 1
            else:
                age_census["too_old"] += 1
        self.query_census = age_census
        self.causal_census = causal_census
        self.hold_ages = ages
        self.causal_ages = causal_ages
        self.skipped_pre_lock = 0  # a near-identity correction is never dropped
        self.rows_online = rows_online
        self.clock_lags = clock_lags
        return rows, [float(r[0]) for r in rows], off_mo, age_census, causal_census

    def finalize(self, report_path=None):
        self.wall_end = time.monotonic()
        rows, stamps, off_mo, age_census, causal_census = self.compose()
        tum = os.path.join(self.out_dir, "localization.tum")
        with open(tum, "w") as fh:
            for r in rows:
                fh.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % r)

        online = getattr(self, "rows_online", [])
        online_path = os.path.join(self.out_dir, "localization_online.tum")
        with open(online_path, "w") as fh:
            for r in online:
                fh.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % r)
        lags = np.asarray(getattr(self, "clock_lags", []), float)

        off_t = np.array(off_mo, float).reshape(-1, 3) if off_mo else np.zeros((0, 3))
        odom_stamps = np.asarray(sorted(self.odom_body), float)
        loc_stamps = np.asarray([r[0] for r in rows], float)
        rate = None
        if len(loc_stamps) > 2:
            rate = 1.0 / float(np.median(np.diff(loc_stamps)))

        def age_stats(values, note):
            a = np.asarray(values, float)
            return {"n": int(a.size),
                    "median": float(np.median(a)) if a.size else None,
                    "p95": float(np.percentile(a, 95)) if a.size else None,
                    "max": float(a.max()) if a.size else None,
                    "note": note}

        def with_fraction(cen):
            q = int(cen.get("queries", 0))
            out = dict(cen)
            out["covered_fraction_of_odom_queries"] = (cen.get("covered", 0) / q) if q else None
            out["rule"] = ("receive-time causal: the correction and the validity answer must have "
                           "ARRIVED before the odometry frame that raised the query; the accepted "
                           "correction is then propagated over that frame's odometry; the "
                           "correction age is query_stamp - tf_message_stamp, where the TF "
                           "message stamp is the stamp of the frame the ICP solved for")
            return out

        censuses = {
            "age_contract": with_fraction(age_census),        # GATED: age <= hold_validity_s
            "causal_available": with_fraction(causal_census),  # diagnostic: no age bound
            "correction_age_s_all_queries": age_stats(getattr(self, "hold_ages", []),
                                                      "age of the held correction at every "
                                                      "odometry query that had a received sample"),
            "correction_age_s_composed": age_stats(getattr(self, "causal_ages", []),
                                                   "age of the correction actually used by the "
                                                   "composed online stream"),
        }
        # the explicit un-lock edge: the first broadcast whose offset is not the initialised
        # identity.  This is an EVENT in stamp time (no wall-clock mapping involved) and is
        # recorded as evidence, never used to reject a sample.
        adopted = None
        for ts in sorted(self.map_odom):
            t_mo, q_mo = self.map_odom[ts]
            R = quat_to_mat(q_mo)
            ang = math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))
            if float(np.linalg.norm(t_mo)) >= 0.01 or ang >= math.radians(0.5):
                adopted = float(ts)
                break
        # rate of DISTINCT broadcast stamps (the stream the consumer can hold) and of broadcast
        # messages (they are equal now: one sample per accepted update, no re-stamping)
        tf_stamps = np.asarray(sorted(self.map_odom), float)
        tf_rate = None
        if tf_stamps.size > 2:
            tf_rate = 1.0 / float(np.median(np.diff(tf_stamps)))
        wall = [w for w, _ in self.tf_arrival]
        tf_msg_rate = None
        if len(wall) > 2:
            tf_msg_rate = 1.0 / float(np.median(np.diff(wall)))
        startup = {
            "first_odom_epoch_s": float(odom_stamps[0]) if len(odom_stamps) else None,
            "first_tf_epoch_s": float(tf_stamps[0]) if tf_stamps.size else None,
            "first_lock_adopted_epoch_s": adopted,
            "first_localization_epoch_s": float(loc_stamps[0]) if len(loc_stamps) else None,
            "last_localization_epoch_s": float(loc_stamps[-1]) if len(loc_stamps) else None,
            "startup_unavailable_s": (float(loc_stamps[0] - odom_stamps[0])
                                      if len(loc_stamps) and len(odom_stamps) else None),
            "note": "startup (estimator IMU init + first accepted ICP lock) is reported SEPARATELY "
                    "and is NOT excluded from the ground-truth support denominator; the gate is "
                    "applied to the full GT span that includes it",
        }
        info = {
            "map_frame": self.map_frame, "odom_frame": self.odom_frame,
            "body_frame": self.body_frame,
            "composition": "T_map_body = T_map_odom (receive-time causally held localizer "
                           "correction) * T_odom_body (lio_odom); one sample per accepted ICP "
                           "update, never re-stamped",
            "frozen_protocol": {
                "hold_validity_s": HOLD_VALIDITY_S,
                "hold_validity_definition": "query_stamp - tf_message_stamp, i.e. the age of the "
                                            "accepted map<-odom correction; the TF message stamp "
                                            "IS the stamp of the frame the alignment solved for",
                "assoc_tolerance_s": ASSOC_TOLERANCE_S,
                "gap_limit_s": GAP_LIMIT_S,
                "validity_stale_s": VALIDITY_STALE_S,
                "note": "pre-declared before the run; identical for every variant; never widened "
                        "after seeing a score",
            },
            "map_odom_messages": len(self.map_odom),
            "map_odom_messages_received": len(self.tf_arrival),
            "odom_body_messages": len(self.odom_body),
            "composed_samples": len(rows),
            "pre_lock_samples_dropped": 0,
            "acceptance_evidence": "a map<-odom sample exists only for an ICP update the node "
                                   "accepted, and the query needs the localizer's own gate to "
                                   "have answered valid within validity_stale_s; a near-identity "
                                   "correction is legitimate and is counted, not rejected",
            "localization_tum": tum,
            "localization_tum_sha256": sc.sha256_file(tum),
            "localization_online_tum": online_path,
            "localization_online_tum_sha256": sc.sha256_file(online_path),
            "online_stream": {
                "samples": int(len(online)),
                "definition": "the same poses stamped at the /clock time current when the "
                              "consumer's query arrived: scoring this against the GT is the "
                              "current-time consumer error, not the delayed-pose error",
                "clock_lag_s": {
                    "n": int(lags.size),
                    "median": float(np.median(lags)) if lags.size else None,
                    "p95": float(np.percentile(lags, 95)) if lags.size else None,
                    "max": float(lags.max()) if lags.size else None,
                    "note": "query clock time minus the stamp of the frame the correction came "
                            "from: how old the robot state the consumer is being told about is",
                },
            },
            "span_s": [rows[0][0], rows[-1][0]] if rows else None,
            "availability": {
                "age_contract_census": censuses["age_contract"],
                "causal_available_census": censuses["causal_available"],
                "correction_age_s_all_queries": censuses["correction_age_s_all_queries"],
                "correction_age_s_composed": censuses["correction_age_s_composed"],
                "distinct_tf_stamp_rate_hz": tf_rate,
                "tf_broadcast_message_rate_hz": tf_msg_rate,
                "note": "distinct stamps are what a consumer can hold; with one sample per "
                        "accepted update they equal the broadcast count",
            },
            "map_odom_offset": {
                "translation_mean_m": [float(x) for x in off_t.mean(axis=0)] if len(off_t) else None,
                "translation_std_m": [float(x) for x in off_t.std(axis=0)] if len(off_t) else None,
                "translation_span_m": [float(x) for x in (off_t.max(axis=0) - off_t.min(axis=0))]
                                      if len(off_t) else None,
                "max_abs_m": float(np.abs(off_t).max()) if len(off_t) else None,
                "note": "the localizer's own map<-odom broadcast translation (the actual TF "
                        "translation, not the odometry one); it is a diagnostic of what the "
                        "localizer published, NOT an acceptance test",
            },
            "relocalize_request": self.reloc_request,
            "relocalize_result": self.reloc_result,
            "relocalize_check_samples": self.valid_checks,
            "relocalize_valid_fraction": (float(np.mean([v for _, v, *_ in self.valid_checks]))
                                          if self.valid_checks else None),
            "startup": startup,
            "localization_sample_rate_hz": rate,
            "localization_samples": int(len(rows)),
            "wall_clock": {"start_monotonic_s": self.wall_start, "end_monotonic_s": self.wall_end,
                           "duration_s": self.wall_end - self.wall_start},
        }
        sc.write_json(report_path or os.path.join(self.out_dir, "localization_record.json"), info)
        # Raw timelines with RECEIPT times, so the GT-side test can apply the identical rule (and
        # so nothing can be backfilled there either).  Wall times are relative to the recorder's
        # start; the GT-side test only compares them against each other.
        w0 = self.wall_start
        avail_path = os.path.join(self.out_dir, "tf_availability.json")
        sc.write_json(avail_path, {
            "frozen_protocol": info["frozen_protocol"],
            "tf_timeline": [[float(stamp), float(arrival - w0), [float(x) for x in t_mo],
                             [float(x) for x in q_mo]]
                            for arrival, stamp, t_mo, q_mo in sorted(self.tf_samples,
                                                                     key=lambda r: r[0])],
            "odom_timeline": [[float(stamp), float(self.odom_arrival[stamp] - w0)]
                              for stamp in sorted(self.odom_body,
                                                  key=lambda t: self.odom_arrival[t])],
            "valid_timeline": [[float(wall - w0), v] for wall, v in sorted(self.valid_log)],
            "validity_mapping": "relocalize_check answers are stamped at the moment the RESPONSE "
                                "arrived; an answer is evidence only for queries raised after it",
            "query_census_over_odom_frames": censuses,
            "note": "wall receipts are seconds since the recorder started; a transform or answer "
                    "that arrived after a query may not be used for it, and nothing is "
                    "interpolated",
        })
        print("[record_localization] map->odom msgs=%d (distinct stamps %d, broadcast rate %.2f Hz, "
              "distinct-stamp rate %s), odom->body msgs=%d, composed=%d; age-contract %d/%d "
              "covered (too_old=%d no_tf_received_before=%d no_valid_answer=%d gate_not_valid=%d), "
              "causal-available %d/%d"
              % (len(self.tf_arrival), len(self.map_odom), tf_msg_rate or 0.0,
                 ("%.2f Hz" % tf_rate) if tf_rate else "n/a",
                 info["odom_body_messages"], info["composed_samples"],
                 age_census["covered"], age_census["queries"], age_census["too_old"],
                 age_census["no_tf_received_before"], age_census["no_valid_answer"],
                 age_census["gate_not_valid"], causal_census["covered"], causal_census["queries"]))
        return info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", default="/tmp/fastlio2_localization")
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--idle-timeout", type=float, default=10.0)
    ap.add_argument("--namespace", default="/fastlio2")
    ap.add_argument("--localizer-namespace", default="/localizer")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--body-frame", default="body")
    ap.add_argument("--prior-map", default="", help="frozen prior PCD the localizer was given")
    ap.add_argument("--initial-pose", default="0,0,0,0,0,0",
                    help="declared initial pose x,y,z,roll,pitch,yaw for the relocalize service")
    ap.add_argument("--valid-poll-s", type=float, default=VALID_POLL_S,
                    help="cadence at which the localizer's own validity answer is sampled; it "
                         "must be finer than validity_stale_s, otherwise no answer is recent "
                         "enough to count as evidence")
    ap.add_argument("--reloc-delay-s", type=float, default=1.0,
                    help="wait this long after the first odometry frame before sending the "
                         "declared initial pose")
    args = ap.parse_args(argv)

    vals = [float(v) for v in args.initial_pose.split(",")]
    if len(vals) != 6:
        raise SystemExit("[error] --initial-pose needs 6 comma-separated values x,y,z,roll,pitch,yaw")

    rclpy.init()
    node = LocalizationRecorder(args.out_dir, args.duration, args.idle_timeout,
                                args.namespace, args.localizer_namespace, args.map_frame,
                                args.odom_frame, args.body_frame)
    node.prior_map = args.prior_map
    node.initial_pose_xyz = vals[:3]
    node.initial_pose_rpy = vals[3:]
    node.reloc_delay_s = float(args.reloc_delay_s)
    last_poll = 0.0
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.2)
            node.maybe_relocalize()
            if args.valid_poll_s > 0 and time.monotonic() - last_poll > args.valid_poll_s:
                last_poll = time.monotonic()
                node.poll_valid()
    except KeyboardInterrupt:
        pass
    node.finalize()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
