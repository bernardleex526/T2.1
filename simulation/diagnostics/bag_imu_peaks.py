#!/usr/bin/env python3
"""Pre-fix kinematic peaks, measured from the ARCHIVED bag + archived GT TUMs (no code archaeology)."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

T0 = 1700000000.0


def imu_peaks(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    g, a, ts = [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic != "/imu/data":
            continue
        m = deserialize_message(data, Imu)
        g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        ts.append(t * 1e-9 - T0)
    g, a, ts = np.array(g), np.array(a), np.array(ts)
    ig = np.argmax(np.linalg.norm(g, axis=1))
    ia = np.argmax(np.linalg.norm(a, axis=1))
    print("  IMU samples %d  span %.3f..%.3f s" % (len(g), ts[0], ts[-1]))
    print("  max |gyro|  = %.3f rad/s at t=%.3f  %s" % (np.linalg.norm(g[ig]), ts[ig], np.round(g[ig], 3)))
    print("  max |accel| = %.3f m/s^2 at t=%.3f  %s" % (np.linalg.norm(a[ia]), ts[ia], np.round(a[ia], 3)))


def tum_peaks(path, label):
    r = np.loadtxt(path, ndmin=2)
    t = r[:, 0]; q = r[:, 4:8]
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                  np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                  np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)
    dR = np.einsum("nji,njk->nik", R[:-1], R[1:])
    inc = np.degrees(np.arccos(np.clip((np.trace(dR, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    dpos = np.linalg.norm(np.diff(r[:, 1:4], axis=0), axis=1)
    print("  %s: max rot inc %.4f deg/ms at t=%.3f | max pos inc %.6f m/ms at t=%.3f"
          % (label, inc.max(), t[np.argmax(inc)] - (1700000000.0 if t[0] > 1e9 else 0),
             dpos.max(), t[np.argmax(dpos)] - (1700000000.0 if t[0] > 1e9 else 0)))


if __name__ == "__main__":
    print("ARCHIVED mapping bag (/tmp/sim_pipeline/baseline/mapping/bag):")
    imu_peaks("/tmp/sim_pipeline/baseline/mapping/bag")
    A = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "artifacts/simulation_20260930/scene_ref"))
    tum_peaks(A + "/gt_mapping.tum", "gt_mapping.tum")
    tum_peaks(A + "/gt_localization.tum", "gt_localization.tum")
    import os
    lb = "/tmp/sim_pipeline/baseline/localization/bag"
    if os.path.isdir(lb):
        print("ARCHIVED localization bag:")
        imu_peaks(lb)
    for c in ("c2_1", "c2_2", "c2_combined"):
        p = "/tmp/sim_pipeline/%s/mapping/bag" % c
        if os.path.isdir(p):
            print("ARCHIVED bag %s:" % c)
            imu_peaks(p)
