#!/usr/bin/env python3
"""Shared helpers for the simulation harness: artifact I/O, hashes, manifests, processes.

Pure standard library + NumPy so it can be imported by every harness script (generator,
recorder, runners, metric tests) without ROS on the import path.
"""

from __future__ import annotations

import datetime
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

PCD_HEADER = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n"
              "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n"
              "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA %s\n")


def utcnow():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def sha256_file(path, chunk=1 << 20):
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def sha256_obj(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def git_head(repo_root):
    """(head, dirty) of the repository; ('unknown', None) when git is unavailable."""
    try:
        head = subprocess.run(["git", "-C", repo_root, "rev-parse", "HEAD"],
                              capture_output=True, text=True, timeout=10)
        if head.returncode != 0:
            return "unknown", None
        st = subprocess.run(["git", "-C", repo_root, "status", "--porcelain"],
                            capture_output=True, text=True, timeout=10)
        dirty = bool(st.stdout.strip()) if st.returncode == 0 else None
        return head.stdout.strip(), dirty
    except (OSError, subprocess.SubprocessError):
        return "unknown", None


def write_json(path, obj):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(obj, fh, indent=1, sort_keys=False)
    os.replace(tmp, path)
    return path


def read_json(path):
    with open(path) as fh:
        return json.load(fh)


def write_pcd(path, pts, ascii_pcd=False):
    pts = np.asarray(pts, float)
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    header = PCD_HEADER % (len(pts), len(pts), "ascii" if ascii_pcd else "binary")
    if ascii_pcd:
        with open(path, "w") as fh:
            fh.write(header)
            np.savetxt(fh, pts, fmt="%.6f")
        return
    arr = np.empty(len(pts), dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")]))
    arr["x"], arr["y"], arr["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(arr.tobytes())


def read_pcd(path):
    """Read x y z from an ASCII or binary PCD (the formats this harness writes)."""
    with open(path, "rb") as fh:
        raw = fh.read()
    idx = raw.find(b"DATA ")
    if idx < 0:
        raise ValueError("not a PCD file (no DATA line): %s" % path)
    head = raw[:idx].decode("ascii", "replace")
    mode = raw[idx + 5:].split(b"\n", 1)[0].strip().decode("ascii").lower()
    body = raw[idx + 5:].split(b"\n", 1)[1]
    if mode == "ascii":
        return np.loadtxt(body.splitlines(), ndmin=2)[:, :3]
    if mode != "binary":
        raise ValueError("unsupported PCD DATA mode %r in %s" % (mode, path))
    n = 0
    for line in head.splitlines():
        if line.startswith("POINTS"):
            n = int(line.split()[1])
    arr = np.frombuffer(body, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")]),
                        count=n)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(float)


# ------------------------------------------------------------------ processes --
class ProcGroup:
    """Own-child process group with bounded waits and PID-scoped cleanup.

    The concurrency contract forbids blanket pkill: every process this harness starts is
    recorded here and killed by its own PID only.  ``stop_all`` is idempotent and always
    escalates SIGTERM -> SIGKILL within a bounded window.
    """

    def __init__(self, label="sim"):
        self.label = label
        self.procs = []          # (name, popen)

    def spawn(self, name, argv, log_path=None, env=None, cwd=None):
        fh = open(log_path, "ab") if log_path else subprocess.DEVNULL
        proc = subprocess.Popen(argv, stdout=fh, stderr=subprocess.STDOUT, env=env, cwd=cwd,
                                start_new_session=True)
        self.procs.append((name, proc))
        return proc

    def alive(self):
        return [(n, p) for n, p in self.procs if p.poll() is None]

    def wait_for(self, proc, timeout_s):
        """Wait at most timeout_s; returns True when the process exited."""
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return True
            time.sleep(0.1)
        return proc.poll() is not None

    def stop_all(self, grace_s=5.0):
        """SIGTERM each recorded child (by PID), then SIGKILL the stragglers."""
        for _, proc in self.procs:
            if proc.poll() is None:
                try:
                    proc.terminate()
                except OSError:
                    pass
        deadline = time.monotonic() + float(grace_s)
        while time.monotonic() < deadline and any(p.poll() is None for _, p in self.procs):
            time.sleep(0.1)
        for name, proc in self.procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    try:
                        proc.kill()
                    except OSError:
                        pass
                print("[%s] SIGKILLed straggler %s pid=%d" % (self.label, name, proc.pid),
                      file=sys.stderr)
        for _, proc in self.procs:
            try:
                proc.wait(timeout=5)
            except Exception:                                     # noqa: BLE001
                pass
        self.procs = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop_all()
        return False


def require(cond, msg):
    if not cond:
        raise SystemExit("[error] %s" % msg)
