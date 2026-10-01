#!/usr/bin/env python3
"""Is the regional p99 a MAP error or an EVALUATOR-normal artefact?

For every sampled estimate point in the worst region (and globally) this compares

  * the evaluator's own residual: |(est - nearest_ref_point) . n_pca|, where n_pca is the PCA
    normal of the k=15 reference neighbours inside 0.15 m at the reference neighbour - a rule
    that is ill-posed where the reference cloud is not locally planar (wall/floor/ceiling
    junctions, pillar tops, grazing ceiling returns), and
  * the residual against the KNOWN ANALYTIC scene surface: the smallest signed distance to the
    room box faces and the pillar cylinders, using the exact plane/cylinder normals.

If the analytic residual is much smaller than the PCA-normal one, the regional gate is being
tripped by the normal rule, not by a bent map; if they agree, the map really is that far off.

Diagnostic only: the certified T1 value is the evaluator's.
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
SCENE = json.load(open(ROOT + "/simulation/synthetic_data/test_trajectory.json"))["scene"]
lo = np.asarray(SCENE["room_box_min"], float)
hi = np.asarray(SCENE["room_box_max"], float)
roi = json.load(open(REF + "/roi.json"))
mid = 0.5 * (np.asarray(roi["bounds"]["min"], float) + np.asarray(roi["bounds"]["max"], float))

est = sc.read_pcd(RUN + "/map_eval.pcd")
ref = sc.read_pcd(REF + "/reference_map.pcd")
C = np.array(json.load(open(RUN + "/frame_chain.json"))["matrix"])
est_s = est @ C[:3, :3].T + C[:3, 3]

# evaluator-style sampling: fixed 5 cm grid over the whole estimate, inside the ROI, est grid
# offset by half a voxel (exactly what map_accuracy_eval.py does)
def voxel_grid_select(pts, voxel, origin):
    cells = np.floor((pts - origin) / voxel).astype(np.int64)
    _, idx = np.unique(cells, axis=0, return_index=True)
    return np.sort(idx)


voxel = 0.05
org_est = np.asarray([0.0, 0.0, 0.0]) + voxel / 2.0
org_ref = np.asarray([0.0, 0.0, 0.0])
i_all = voxel_grid_select(est_s, voxel, org_est)
est_a = est_s[i_all]
in_roi = np.all((est_a >= np.asarray(roi["bounds"]["min"], float))
                & (est_a <= np.asarray(roi["bounds"]["max"], float)), axis=1)
est_s = est_a[in_roi]
print("sampled est points inside ROI: %d (of %d)" % (len(est_s), len(est_a)))

tree = cKDTree(ref)
d_nn, i_nn = tree.query(est_s, workers=-1)
ref_nn = ref[i_nn]


def normals_at(query, k=15, radius=0.15):
    dd, ii = tree.query(query, k=k, workers=-1)
    ok = np.isfinite(dd) & (dd <= radius)
    n = np.zeros_like(query)
    spread = np.zeros(len(query))
    for j in range(len(query)):
        idx = ii[j][ok[j]]
        if len(idx) < 3:
            n[j] = (0, 0, 1)
            continue
        P = ref[idx]
        Pm = P - P.mean(0)
        w, v = np.linalg.eigh(Pm.T @ Pm)
        n[j] = v[:, 0]
        # planarity: the smallest eigenvalue vs the next one (0 = perfect plane)
        spread[j] = np.sqrt(max(w[0], 0.0) / max(w[1], 1e-12))
    return n, spread


n_pca, spread = normals_at(ref_nn)
p2pl_pca = np.abs(np.einsum("ij,ij->i", est_s - ref_nn, n_pca))

# ---- analytic surface distance with exact normals -------------------------
def analytic_signed(p):
    """(min |signed distance|, exact unit normal at that surface) over faces + pillars."""
    best = np.full(len(p), np.inf)
    best_n = np.zeros((len(p), 3))
    for axis in range(3):
        for plane, sgn in ((lo[axis], +1.0), (hi[axis], -1.0)):
            d = sgn * (p[:, axis] - plane)              # + = inside the room
            # no in-face bounds test: the evaluator's nearest reference point may sit on either
            # wall of a corner, so the comparison must use the nearest SURFACE PLANE distance
            m = np.abs(d) < best
            best[m] = np.abs(d[m])
            nrm = np.zeros((m.sum(), 3))
            nrm[:, axis] = sgn
            best_n[m] = nrm
    for pil in SCENE["pillars"]:
        cx, cy = pil["center_xy"]
        r = float(pil["radius_m"])
        z0, z1 = pil["z_range"]
        dx = p[:, 0] - cx
        dy = p[:, 1] - cy
        rad = np.hypot(dx, dy)
        d = np.abs(rad - r)
        ok = (p[:, 2] >= z0 - 1e-9) & (p[:, 2] <= z1 + 1e-9) & (rad > 1e-9)
        m = ok & (d < best)
        best[m] = d[m]
        with np.errstate(invalid="ignore", divide="ignore"):
            nrm = np.stack([dx / rad, dy / rad, np.zeros(len(p))], axis=1)
        best_n[m] = nrm[m]
    return best, best_n


d_an, n_an = analytic_signed(est_s)
# sign the analytic residual with the est point's own side (inside/outside the surface)
inside = np.ones(len(est_s))
sgn_an = np.sign(np.einsum("ij,ij->i", est_s - ref_nn, n_an))
d_an_signed = sgn_an * d_an

key = ((est_s[:, 0] >= mid[0]).astype(int) * 4 + (est_s[:, 1] >= mid[1]).astype(int) * 2
       + (est_s[:, 2] >= mid[2]).astype(int))


def stats(name, m):
    if m.sum() == 0:
        return
    print("  %-28s n=%7d  pca p99 %.4f rmse %.4f | analytic p99 %.4f rmse %.4f | "
          "mean(pca-an) %+.4f | normal-spread p95 %.3f"
          % (name, m.sum(), np.percentile(p2pl_pca[m], 99), np.sqrt((p2pl_pca[m] ** 2).mean()),
             np.percentile(d_an[m], 99), np.sqrt((d_an[m] ** 2).mean()),
             (p2pl_pca[m] - d_an[m]).mean(), np.percentile(spread[m], 95)))


print("\nresidual: evaluator PCA normal vs KNOWN ANALYTIC surface (metres)")
stats("ALL", np.ones(len(est_s), bool))
for c in range(8):
    stats("octant_%d%d%d" % ((c >> 2) & 1, (c >> 1) & 1, c & 1), key == c)
print("\nby reference-normal axis (evaluator's own classification):")
axis = np.argmax(np.abs(n_pca), axis=1)
for a, nm in enumerate("xyz"):
    stats("normal " + nm, axis == a)
print("\nworst-region detail (octant_011): how much of the p99 comes from non-planar reference "
      "neighbourhoods?")
m = key == 3
if m.sum():
    sp = spread[m]
    pc = p2pl_pca[m]
    an = d_an[m]
    for lim in (0.05, 0.1, 0.2, 1.0):
        mm = sp <= lim
        print("  normal-spread <= %.2f: n=%6d (%.1f%%) pca p99 %.4f | analytic p99 %.4f"
              % (lim, mm.sum(), 100.0 * mm.mean(), np.percentile(pc[mm], 99) if mm.any() else -1,
                 np.percentile(an[mm], 99) if mm.any() else -1))
    bad = sp > 0.2
    if bad.any():
        print("  non-planar reference neighbourhoods in octant_011: n=%d, their pca p99 %.4f vs "
              "analytic p99 %.4f" % (bad.sum(), np.percentile(pc[bad], 99),
                                     np.percentile(an[bad], 99)))
    print("  octant_011 signed analytic mean %+.4f (positive = outside the surface)"
          % d_an_signed[m].mean())
