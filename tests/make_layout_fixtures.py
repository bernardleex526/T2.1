#!/usr/bin/env python3
"""Generate the small PointCloud2 layout fixtures used by tests/test_probe_tools.py.

The layouts mirror the PUBLISHED RoboSense ROS 2 SDK field order
(RoboSense-LiDAR/rslidar_sdk, src/source/source_pointcloud_ros.hpp):

    x f32, y f32, z f32, intensity f32 [, ring u16, timestamp f64]

so point_step is 16 for the SDK's DEFAULT POINT_TYPE=XYZI (no time field at all)
and 26/32 for POINT_TYPE=XYZIRT (which adds ring + timestamp).

Fixtures are kept tiny (~100 points) on purpose: a layout decision needs the
field NAMES, TYPES and the time SPAN, not tens of thousands of points, and the
repository policy excludes large binary artefacts.

Run:  python3 tests/make_layout_fixtures.py
"""

import json
import os

import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
os.makedirs(OUT, exist_ok=True)

N = 100  # points per fixture


def f(name, offset, datatype, count=1):
    return {"name": name, "offset": offset, "datatype": datatype, "count": count}


def write(name, fields, columns, point_step, width=N, height=1,
          stamp=1730000000.123):
    doc = {
        "point_step": point_step,
        "width": width,
        "height": height,
        "frame_id": "rslidar",
        "header_stamp": stamp,
        "fields": fields,
        "columns": {k: [round(float(x), 9) for x in v] for k, v in columns.items()},
    }
    p = os.path.join(OUT, name)
    with open(p, "w") as fh:
        json.dump(doc, fh, indent=1)
    print(f"wrote {p}")


rng = np.random.default_rng(7)
xyz = rng.uniform(-20, 20, size=(N, 3))
inten = rng.uniform(0, 255, size=N)
ring = rng.integers(0, 128, size=N)
sweep = np.linspace(0.0, 0.1, N)  # one 10 Hz scan period, relative

# 1. XYZI - the rslidar_sdk DEFAULT. Four fields, point_step 16, NO time field.
write("airy_xyzi_default.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten},
      point_step=16)

# 2. XYZIRT with timestamps in SECONDS (relative to the sweep).
write("airy_xyzirt_seconds.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("timestamp", 18, 8)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "timestamp": sweep},
      point_step=26)

# 3. Same, NANOSECONDS.
write("airy_xyzirt_nanoseconds.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("timestamp", 18, 8)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "timestamp": sweep * 1e9},
      point_step=26)

# 4. Same, MICROSECONDS.
write("airy_xyzirt_microseconds.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("timestamp", 18, 8)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "timestamp": sweep * 1e6},
      point_step=26)

# 5. Absolute epoch clock spanning one sweep: usable, must be flagged as absolute.
write("airy_xyzirt_epoch.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("timestamp", 18, 8)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "timestamp": 1730000000.0 + sweep},
      point_step=26)

# 6. Velodyne-style field NAMED "time", FLOAT32 seconds.
write("velodyne_time_f32.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("time", 18, 7)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "time": sweep.astype(np.float32)},
      point_step=22)

# 7. UINT32 nanoseconds: a genuine per-point time that Utils::pcl2_to_PCL CANNOT
#    read (it accepts FLOAT32/FLOAT64 only), so the probe must refuse it.
write("airy_xyzirt_uint32_ns.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("timestamp", 18, 6)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "timestamp": (sweep * 1e9).astype(np.uint32)},
      point_step=22)

# 8. Nothing time-like: a constant field and a non-time name.
write("degenerate_no_time.json",
      [f("x", 0, 7), f("y", 4, 7), f("z", 8, 7), f("intensity", 12, 7),
       f("ring", 16, 4), f("foo", 18, 7)],
      {"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "intensity": inten,
       "ring": ring, "foo": np.full(N, 3.5)},
      point_step=22)

print(f"done: {len(os.listdir(OUT))} fixtures in {OUT}")
