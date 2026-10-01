#!/usr/bin/env python3
"""Attribution of the residual map error: rigid (chain-absorbable) vs non-rigid (within-map).

Diagnostic only: the reported T1 number is the evaluator's, with the frozen pre-eval chain.
"""
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT + "/simulation")
import sim_common as sc

RUN = sys.argv[1]
REF = sys.argv[2]
est = sc.read_pcd(RUN + "/map_eval.pcd")
ref = sc.read_pcd(REF + "/reference_map.pcd")
C = np.array(json.load(open(RUN + "/frame_chain.json"))["matrix"])
est_s = est @ C[:3, :3].T + C[:3, 3]
print("est %d ref %d" % (len(est_s), len(ref)))
tree = cKDTree(ref)


def normals_at(query, k=15, radius=0.15):
    dd, ii = tree.query(query, k=k, workers=-1)
    ok = np.isfinite(dd) & (dd <= radius)
    n = np.zeros_like(query)
    for j in range(len(query)):
        idx = ii[j][ok[j]]
        if len(idx) < 3:
            n[j] = (0, 0, 1)
            continue
        P = ref[idx]
        Pm = P - P.mean(0)
        w, v = np.linalg.eigh(Pm.T @ Pm)
        n[j] = v[:, 0]
    return n


def p2pl(cur):
    d, i = tree.query(cur, workers=-1)
    n = normals_at(ref[i])
    return np.abs(np.einsum("ij,ij->i", cur - ref[i], n)), d


d0, dn0 = p2pl(est_s)
print("as-is:            p2pl rmse %.4f median %.4f p95 %.4f p99 %.4f" %
      (np.sqrt((d0 ** 2).mean()), np.median(d0), np.percentile(d0, 95), np.percentile(d0, 99)))


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (src - mu_s).T @ (dst - mu_d) / len(src)
    U, _, Vt = np.linalg.svd(S)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, mu_d - R @ mu_s


cur = est_s.copy()
print("residual rigid alignment (DIAGNOSTIC, not the metric):")
for it in range(5):
    d, i = tree.query(cur, workers=-1)
    keep = d < (0.3 if it < 2 else 0.1)
    R, t = umeyama(cur[keep], ref[i[keep]])
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    print("  it%d keep %.3f rot %.4f deg |t| %.4f p2p rmse %.4f" %
          (it, keep.mean(), ang, np.linalg.norm(t), np.sqrt((d[keep] ** 2).mean())))
    cur = cur @ R.T + t
    if ang < 1e-4 and np.linalg.norm(t) < 1e-5:
        break
d1, _ = p2pl(cur)
print("after rigid-only: p2pl rmse %.4f median %.4f p95 %.4f p99 %.4f" %
      (np.sqrt((d1 ** 2).mean()), np.median(d1), np.percentile(d1, 95), np.percentile(d1, 99)))
# what remains is non-rigid (within-map) error
print("non-rigid share: rmse %.4f m" % np.sqrt((d1 ** 2).mean()))

# p2pl vs distance to the nearest odometry sample (range from the sensor)
od = np.loadtxt(RUN + "/odom.tum", ndmin=2)[:, 1:4]
# odom positions are in the est frame -> map them to the scene frame for the range comparison
od = od @ C[:3, :3].T + C[:3, 3]
otree = cKDTree(od)
rng, _ = otree.query(est_s, workers=-1)
print("p2pl by range from the trajectory:")
for lo, hi in ((0, 2), (2, 4), (4, 6), (6, 8), (8, 10), (10, 20)):
    m = (rng >= lo) & (rng < hi)
    if m.sum():
        print("  %2d-%2d m n=%7d rmse %.4f p95 %.4f p99 %.4f" %
              (lo, hi, m.sum(), np.sqrt((d0[m] ** 2).mean()), np.percentile(d0[m], 95),
               np.percentile(d0[m], 99)))
axis = np.argmax(np.abs(normals_at(ref[tree.query(est_s, workers=-1)[1]])), axis=1)
for a, nm in enumerate("xyz"):
    m = axis == a
    print("  normal %s n=%7d rmse %.4f p99 %.4f" %
          (nm, m.sum(), np.sqrt((d0[m] ** 2).mean()), np.percentile(d0[m], 99)))
