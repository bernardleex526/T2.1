#!/usr/bin/env python3
"""Decode bag messages using rosbag2_py + rclpy deserialization (authoritative)."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

BAG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sim_pipeline/baseline/mapping/bag"
T0 = 1700000000.0
lo, hi = float(sys.argv[2]), float(sys.argv[3])

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
topics = {t.name: t.type for t in r.get_all_topics_and_types()}
print(topics)
n_imu = n_scan = 0
while r.has_next():
    topic, data, ts = r.read_next()
    t = ts * 1e-9 - T0
    if t < lo - 0.001 or t > hi + 0.001:
        continue
    if topic == "/imu/data":
        m = deserialize_message(data, Imu)
        g, a = m.angular_velocity, m.linear_acceleration
        print("IMU  t=%.3f hdr=%.3f gyro=[%+.5f %+.5f %+.5f] acc=[%+.4f %+.4f %+.4f]"
              % (t, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - T0,
                 g.x, g.y, g.z, a.x, a.y, a.z))
        n_imu += 1
    elif topic == "/rslidar_points":
        m = deserialize_message(data, PointCloud2)
        names = [f.name for f in m.fields]
        offs = {f.name: f.offset for f in m.fields}
        raw = np.frombuffer(bytes(m.data), dtype="<f4")
        step = m.point_step // 4
        mat = raw.reshape(-1, step)
        idx = {n: offs[n] // 4 for n in names}
        xyz = np.stack([mat[:, idx["x"]], mat[:, idx["y"]], mat[:, idx["z"]]], 1)
        ti = mat[:, idx["time"]] if "time" in idx else np.zeros(len(mat))
        rng = np.linalg.norm(xyz, axis=1)
        print("SCAN t=%.4f hdr=%.4f n=%d fields=%s time=[%.4f,%.4f] mono=%s range=[%.3f,%.3f]"
              % (t, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - T0, len(mat), names,
                 ti.min(), ti.max(), bool(np.all(np.diff(ti) >= -1e-6)), rng.min(), rng.max()))
        print("     xyz[0]=%s t[0]=%.4f | xyz[-1]=%s t[-1]=%.4f | z<0 %.3f"
              % (np.round(xyz[0], 3), ti[0], np.round(xyz[-1], 3), ti[-1], (xyz[:, 2] < 0).mean()))
        n_scan += 1
print("imu=%d scans=%d" % (n_imu, n_scan))
