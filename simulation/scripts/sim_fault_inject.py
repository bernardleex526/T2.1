#!/usr/bin/env python3
"""Inject ONE observable fault into a RUNNING localization simulation run.

Step 4 requires that the consumer's failure behaviour be shown by the recorded evidence, not by
unit tests: after the initial lock the localizer must be taken away (or the odometry, or the
clock) and the consumer must STOP producing current poses instead of forging them.

The trigger is the simulation clock, not a sleep: the bag player publishes /clock, so the fault
fires at a chosen BAG TIME regardless of how long startup took.  Every action is logged as one
JSON object per line (wall offset, sim clock at the action, target pid, result) so the run's
evidence has the fault's own timeline next to the consumer's.

Faults (all PID-scoped; the target pid is verified to belong to THIS ROS_DOMAIN_ID first, and no
process is ever matched by pattern):

  stop_localizer   SIGINT the localizer_node  -> after the gate's validity timeout there must be
                   no new query and no new pose; status must report the correction invalid/stale.
  stop_odom        SIGINT lio_node            -> the consumer loses odometry; prediction must
                   reject once dt > max_prediction_s (0.12 s) and stop publishing.
  clock_pause      SIGSTOP the bag player for --hold-s, then SIGCONT -> ROS time (and every
                   stamp) freezes; the consumer must not re-emit a pose for a query clock that
                   did not advance.
  clock_jumpback   SIGINT the player and restart the SAME bag at --start-offset
                   (elapsed - --hold-s) -> ROS time jumps BACKWARDS; the consumer must flush its
                   odometry/gate/TF caches and re-wait for input instead of publishing from the
                   stale future.

Standalone use on an existing run (the runner does exactly this with --fault):
    python3 sim_fault_inject.py --fault stop_localizer --target-clock-s 1693500000.0 \\
        --pids-json <run>/pids.json --bag-pid-file <run>/bag_play.pid \\
        --bag-dir <run>/bag --bag-t0-s 1693490000.0 --log <run>/fault.log
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

FAULTS = ("stop_localizer", "stop_odom", "clock_pause", "clock_jumpback")
NODE_FOR_FAULT = {"stop_localizer": "localizer_node", "stop_odom": "lio_node"}


class Injector(Node):
    def __init__(self, log_path):
        super().__init__("sim_fault_inject")
        self.clock = None
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.log_path = log_path
        self.records = []

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ------------------------------------------------------------------- logging --
    def record(self, **fields):
        rec = {"wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "clock": self.clock, "pid": os.getpid()}
        rec.update(fields)
        self.records.append(rec)
        line = json.dumps(rec, sort_keys=True)
        print("[sim_fault_inject] %s" % line, flush=True)
        if self.log_path:
            with open(self.log_path, "a") as fh:
                fh.write(line + "\n")

    def finalize(self):
        if not self.log_path:
            return
        with open(self.log_path, "a") as fh:
            fh.write(json.dumps({"wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                                 "clock": self.clock, "pid": os.getpid(),
                                 "summary": True, "records": self.records}) + "\n")


# ------------------------------------------------------------------------ helpers --
def pid_domain(pid):
    """ROS_DOMAIN_ID of a process we own, or None (unreadable / gone)."""
    try:
        with open("/proc/%d/environ" % pid, "rb") as fh:
            environ = fh.read().decode("utf-8", "replace")
    except OSError:
        return None
    for chunk in environ.split("\0"):
        if chunk.startswith("ROS_DOMAIN_ID="):
            return chunk.split("=", 1)[1]
    return None


def load_pids(path):
    if not path or not os.path.isfile(path):
        return {}
    with open(path, "r") as fh:
        return json.load(fh)


def check_target(pid, expected_domain, what):
    """Refuse to signal anything that is not a live process of THIS run's domain."""
    if not pid:
        raise SystemExit("[error] no pid for %s (is the process up? pids.json written?)" % what)
    if not os.path.isdir("/proc/%d" % pid):
        raise SystemExit("[error] %s pid %d is not running" % (what, pid))
    domain = pid_domain(pid)
    if domain is None:
        raise SystemExit("[error] %s pid %d is not ours (environ unreadable)" % (what, pid))
    if expected_domain is not None and domain != str(expected_domain):
        raise SystemExit("[error] %s pid %d belongs to ROS_DOMAIN_ID=%s, not %s"
                         % (what, pid, domain, expected_domain))
    return pid


def alive(pid):
    return bool(pid) and os.path.isdir("/proc/%d" % pid)


def wait_gone(pid, limit_s):
    """True when the pid is gone within limit_s (PID-scoped; no pattern matching)."""
    end = time.monotonic() + limit_s
    while time.monotonic() < end:
        if not alive(pid):
            return True
        time.sleep(0.2)
    return not alive(pid)


# ------------------------------------------------------------------------- actions --
def act(node, args, domain):
    if args.fault in NODE_FOR_FAULT:
        name = NODE_FOR_FAULT[args.fault]
        pid = check_target(load_pids(args.pids_json).get(name), domain, name)
        os.kill(pid, signal.SIGINT)
        node.record(fault=args.fault, action="SIGINT", target=name, target_pid=pid,
                    detail="the consumer must stop producing current poses once this input is gone")
        return 0

    bag_pid = check_target(read_bag_pid(args.bag_pid_file), domain, "bag_play")

    if args.fault == "clock_pause":
        os.kill(bag_pid, signal.SIGSTOP)
        node.record(fault=args.fault, action="SIGSTOP", target="bag_play", target_pid=bag_pid,
                    hold_s=args.hold_s, detail="ROS time (and every stamp) freezes")
        time.sleep(args.hold_s)
        if alive(bag_pid):
            os.kill(bag_pid, signal.SIGCONT)
        node.record(fault=args.fault, action="SIGCONT", target="bag_play", target_pid=bag_pid,
                    detail="clock resumes; the frozen interval must not be filled with a "
                           "re-emitted pose")
        return 0

    if args.fault == "clock_jumpback":
        elapsed = max(0.0, args.target_clock_s - args.bag_t0_s)
        offset = max(0.0, elapsed - args.hold_s)
        os.kill(bag_pid, signal.SIGINT)
        if not wait_gone(bag_pid, 10.0):
            os.kill(bag_pid, signal.SIGKILL)
            wait_gone(bag_pid, 10.0)
        node.record(fault=args.fault, action="SIGINT", target="bag_play", target_pid=bag_pid,
                    detail="player stopped to restart it at an EARLIER offset")
        cmd = ["ros2", "bag", "play", args.bag_dir, "--clock", "--rate", str(args.rate),
               "--start-offset", "%.3f" % offset, "--disable-keyboard-controls"]
        out = open(args.play_log, "a") if args.play_log else subprocess.DEVNULL
        proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT)
        with open(args.bag_pid_file, "w") as fh:
            fh.write("%d\n" % proc.pid)
        node.record(fault=args.fault, action="restart", target="bag_play", target_pid=proc.pid,
                    start_offset_s=offset, cmd=" ".join(cmd),
                    detail="ROS time jumps BACKWARDS by the replayed interval")
        rc = proc.wait()
        node.record(fault=args.fault, action="restart_finished", target="bag_play",
                    target_pid=proc.pid, rc=rc)
        return 0

    raise SystemExit("[error] unknown fault %r" % args.fault)


def read_bag_pid(path):
    try:
        with open(path, "r") as fh:
            return int((fh.read().strip() or "0"))
    except (OSError, ValueError):
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--fault", required=True, choices=list(FAULTS))
    ap.add_argument("--target-clock-s", type=float, required=True,
                    help="absolute ROS time at which the fault fires")
    ap.add_argument("--hold-s", type=float, default=5.0,
                    help="clock_pause freeze time / clock_jumpback replayed interval")
    ap.add_argument("--pids-json", default="", help="run's pids.json (node PIDs)")
    ap.add_argument("--bag-pid-file", default="", help="file holding the bag player's PID")
    ap.add_argument("--bag-dir", default="", help="bag to replay for clock_jumpback")
    ap.add_argument("--bag-t0-s", type=float, default=0.0,
                    help="absolute ROS time of the bag's first stamp (for the jump-back offset)")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--play-log", default="", help="where a restarted player's output goes")
    ap.add_argument("--log", default="", help="fault.jsonl evidence path")
    ap.add_argument("--timeout-s", type=float, default=900.0,
                    help="give up if the target clock is never reached")
    args = ap.parse_args(argv)

    domain = os.environ.get("ROS_DOMAIN_ID")
    rclpy.init()
    node = Injector(args.log)
    node.record(fault=args.fault, action="armed", domain=domain,
                target_clock_s=args.target_clock_s, hold_s=args.hold_s, rate=args.rate)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.clock is not None and node.clock >= args.target_clock_s:
                break
        if node.clock is None or node.clock < args.target_clock_s:
            node.record(fault=args.fault, action="timeout",
                        detail="the target clock was never reached; NO fault was injected")
            return 2
        node.record(fault=args.fault, action="triggered",
                    detail="target clock reached; injecting")
        return act(node, args, domain)
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
