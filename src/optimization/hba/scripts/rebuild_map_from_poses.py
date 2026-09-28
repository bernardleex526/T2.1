#!/usr/bin/env python3
"""Rebuild a merged map.pcd from a map directory's per-keyframe patches plus a
pose file.

    rebuild_map_from_poses.py --map-dir DIR --poses FILE --out map.pcd
                              [--pose-order wxyz|xyzw] [--voxel 0.0] [--max-range 0]

The pose lines must be one pose per line in the SAME ORDER as the patch lines of
<map-dir>/poses.txt (that is what /hba/save_poses writes: it appends one line per
loaded patch).  The pose file written by hba contains only "tx ty tz qw qx qy qz";
the patch file names are therefore taken positionally from <map-dir>/poses.txt.

Every patch is transformed by its pose into the map frame and accumulated.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import open3d as o3d


def read_patch_names(map_dir):
    names = []
    with open(os.path.join(map_dir, "poses.txt")) as fh:
        for line in fh:
            line = line.strip()
            if line:
                names.append(line.split()[0])
    return names


def read_tq(path, order="wxyz"):
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) == 8:  # "<name> tx ty tz qw qx qy qz"
                fields = fields[1:]
            v = [float(x) for x in fields]
            if len(v) == 7:
                rows.append(v)
            else:
                sys.exit("[rebuild] unexpected pose line (%d fields): %s" % (len(v), line))
    return np.asarray(rows, dtype=float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-dir", required=True)
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--voxel", type=float, default=0.0)
    ap.add_argument("--max-range", type=float, default=0.0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    names = read_patch_names(args.map_dir)
    poses = read_tq(args.poses)
    if len(poses) != len(names):
        sys.exit("[rebuild] pose count %d != patch count %d (positional match required)"
                 % (len(poses), len(names)))

    pts_all = []
    for name, p in zip(names, poses):
        pc = o3d.io.read_point_cloud(os.path.join(args.map_dir, "patches", name))
        pts = np.asarray(pc.points, dtype=float)
        if pts.size == 0:
            continue
        T = np.eye(4)
        T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(
            np.array([p[3], p[4], p[5], p[6]]))  # o3d wants (w, x, y, z)
        T[:3, 3] = p[:3]
        pts = pts @ T[:3, :3].T + T[:3, 3]
        if args.max_range > 0:
            r = np.linalg.norm(pts - T[:3, 3], axis=1)
            pts = pts[r <= args.max_range]
        pts_all.append(pts)

    all_pts = np.vstack(pts_all)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(all_pts))
    if args.voxel > 0:
        cloud = cloud.voxel_down_sample(args.voxel)
    o3d.io.write_point_cloud(args.out, cloud, write_ascii=False, compressed=False)
    if not args.quiet:
        print("[rebuild] %s: %d patches, %d points -> %d points after voxel=%s"
              % (args.out, len(names), len(all_pts), len(cloud.points), args.voxel))


if __name__ == "__main__":
    main()
