"""Shared input layer for the T2.1 deployment probes.

Design constraint that drives everything here: the probes must run BOTH on the
target Orin NX (where ROS 2 and rosbag2_py exist) and on a workstation / CI
where they do not.  So ROS is an optional import, and every probe is written
against one neutral in-memory shape:

    samples: (N, C) float64 array
    times:   (N,)   float64 array, seconds
    meta:    dict with whatever the source could tell us

Supported sources, in the order a user is likely to have them:
  * ``.npz``  - ``times`` + ``samples`` (+ optional ``fields``), the offline/CI path;
  * ``.csv``  - first column time in seconds, remaining columns are channels;
  * ROS 2 bag - via ``rosbag2_py`` when available, reading one topic.

Nothing here invents data: a missing topic or an unreadable bag is an error with
a specific message, never an empty result that looks like a clean measurement.
"""

from __future__ import annotations

import csv
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


class ProbeInputError(RuntimeError):
    """Raised when an input cannot be read.

    Deliberately distinct from "read it and found nothing": callers must be able
    to tell a broken input path from a legitimately empty measurement.
    """


@dataclass
class Samples:
    """One channel-aligned time series."""

    times: np.ndarray                 # (N,) seconds
    samples: np.ndarray               # (N, C)
    fields: List[str] = field(default_factory=list)
    source: str = ""
    meta: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.times = np.asarray(self.times, dtype=np.float64).reshape(-1)
        self.samples = np.asarray(self.samples, dtype=np.float64)
        if self.samples.ndim == 1:
            self.samples = self.samples.reshape(-1, 1)
        if self.samples.shape[0] != self.times.shape[0]:
            raise ProbeInputError(
                f"{self.source}: {self.times.shape[0]} timestamps but "
                f"{self.samples.shape[0]} sample rows"
            )
        if not self.fields:
            self.fields = [f"ch{i}" for i in range(self.samples.shape[1])]

    @property
    def n(self) -> int:
        return int(self.times.shape[0])

    @property
    def channels(self) -> int:
        return int(self.samples.shape[1])

    def duration(self) -> float:
        return float(self.times[-1] - self.times[0]) if self.n > 1 else 0.0

    def median_rate(self) -> float:
        """Median sample rate [Hz] from the timestamps, 0.0 when undeterminable."""
        if self.n < 3:
            return 0.0
        dt = np.diff(self.times)
        dt = dt[dt > 0]
        if dt.size == 0:
            return 0.0
        return float(1.0 / np.median(dt))

    def column(self, name: str) -> np.ndarray:
        try:
            idx = self.fields.index(name)
        except ValueError:
            raise ProbeInputError(
                f"{self.source}: no channel named {name!r} (have: {', '.join(self.fields)})"
            ) from None
        return self.samples[:, idx]


def _load_npz(path: str) -> Samples:
    with np.load(path, allow_pickle=False) as z:
        keys = set(z.files)
        if "times" not in keys:
            raise ProbeInputError(f"{path}: .npz must contain a 'times' array (seconds)")
        times = z["times"]
        if "samples" in keys:
            samples = z["samples"]
        else:
            raise ProbeInputError(f"{path}: .npz must contain a 'samples' array")
        fields = [str(f) for f in z["fields"]] if "fields" in keys else []
    return Samples(times=times, samples=samples, fields=fields, source=path,
                   meta={"format": "npz"})


def _load_csv(path: str) -> Samples:
    with open(path, "r", newline="") as fh:
        rows = list(csv.reader(fh))
    if not rows:
        raise ProbeInputError(f"{path}: empty CSV")

    header: Optional[List[str]] = None
    first = [c.strip() for c in rows[0]]
    start = 0
    try:
        float(first[0])
    except (ValueError, IndexError):
        header = first
        start = 1

    data = []
    for lineno, row in enumerate(rows[start:], start=start + 1):
        if not row or all(not c.strip() for c in row):
            continue
        try:
            data.append([float(c) for c in row])
        except ValueError:
            raise ProbeInputError(
                f"{path}:{lineno}: non-numeric field in {row!r}"
            ) from None
    if not data:
        raise ProbeInputError(f"{path}: no numeric rows")

    arr = np.asarray(data, dtype=np.float64)
    times = arr[:, 0]
    samples = arr[:, 1:]
    fields = header[1:] if header else []
    return Samples(times=times, samples=samples, fields=fields, source=path,
                   meta={"format": "csv"})


def _rosbag_available() -> bool:
    try:
        import rosbag2_py  # noqa: F401
    except Exception:
        return False
    return True


def _load_rosbag(path: str, topic: str, limit: int = 0) -> Samples:
    """Read one topic from a ROS 2 bag.

    Kept intentionally narrow: we only understand the two message types the T2.1
    deployment actually consumes, and we say so instead of guessing at others.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:  # pragma: no cover - depends on the host install
        raise ProbeInputError(
            f"reading {path} needs ROS 2 python packages (rosbag2_py, rclpy): {exc}"
        ) from None

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr")
    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:
        raise ProbeInputError(f"cannot open bag {path}: {exc}") from None

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise ProbeInputError(
            f"{path}: topic {topic!r} not in bag. Present: "
            f"{', '.join(sorted(topics)) or '(none)'}"
        )
    msg_type = get_message(topics[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    times: List[float] = []
    rows: List[Sequence[float]] = []
    while reader.has_next():
        _, raw, stamp_ns = reader.read_next()
        msg = deserialize_message(raw, msg_type)
        name = topics[topic]
        if name.endswith("sensor_msgs/msg/Imu"):
            hdr = msg.header.stamp
            times.append(hdr.sec + hdr.nanosec * 1e-9)
            rows.append((msg.linear_acceleration.x, msg.linear_acceleration.y,
                         msg.linear_acceleration.z, msg.angular_velocity.x,
                         msg.angular_velocity.y, msg.angular_velocity.z))
        else:
            # PointCloud2-style topics are handled by the dedicated probe, which
            # needs the raw layout; refuse rather than flattening it wrongly.
            raise ProbeInputError(
                f"{path}:{topic} is {name}, which this reader does not flatten. "
                "Use rslidar_pcl2_probe.py for point clouds."
            )
        if limit and len(times) >= limit:
            break

    fields = ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
    return Samples(times=np.asarray(times), samples=np.asarray(rows) if rows else np.zeros((0, 6)),
                   fields=fields, source=f"{path}:{topic}",
                   meta={"format": "rosbag2", "topic": topic, "msg_type": topics[topic],
                         "bag_stamp_ns": stamp_ns if times else None})


def load(path: str, topic: str = "", limit: int = 0) -> Samples:
    """Load a time series from .npz, .csv or a ROS 2 bag directory."""
    if not os.path.exists(path):
        raise ProbeInputError(f"input not found: {path}")

    lower = path.lower()
    if lower.endswith(".npz"):
        return _load_npz(path)
    if lower.endswith(".csv"):
        return _load_csv(path)
    if os.path.isdir(path):
        if not topic:
            raise ProbeInputError(
                f"{path} looks like a ROS 2 bag: pass --topic to select a stream"
            )
        return _load_rosbag(path, topic, limit=limit)
    raise ProbeInputError(
        f"{path}: unrecognised input. Expected .npz, .csv, or a ROS 2 bag directory."
    )


def describe_inputs() -> str:
    """One-line capability note for probe headers and error messages."""
    ros = "available" if _rosbag_available() else "NOT available (offline mode: use .npz/.csv)"
    return f"rosbag2_py: {ros}; numpy {np.__version__}"
