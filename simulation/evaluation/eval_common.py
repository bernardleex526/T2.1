#!/usr/bin/env python3
"""Shared primitives for the corrected (v2) T2.1 evaluation contract.

Owner: EvalContract.  Frozen baseline commit
04b73f553918b1dcc751feac69c0a2834f194432.

Design rules (normative text lives in evaluation_contract.yaml):

  * every sampling decision is deterministic and fully parameterised by
    (voxel_m, grid_origin_m, seed); both are echoed into every result;
  * the point-to-plane residual uses |d . n| -- normals are *sign-invariant*,
    so normal orientation can never move the metric;
  * the query (estimated) cloud and the reference cloud are sampled on grids
    with DIFFERENT origins (half-voxel offset).  Legacy eval_map.py
    voxel-downsampled both clouds on one grid, which lets a voxel collapse two
    coincident points to a ~0 residual and inflates any "within X cm" fraction;
  * no scale, no non-rigid and no per-block transform is ever fitted.  The
    only accepted alignment is a rigid SE(3) supplied from a *sourced*
    transform record (assert_se3 + assert_not_per_block below);
  * no algorithm / front-end / parameter code lives here and nothing here
    writes into the legacy outputs.
"""
from __future__ import annotations

import hashlib
import json
import math
import os

import numpy as np
from scipy.spatial import cKDTree

# ---------------------------------------------------------------- constants --
# Measurability floors.  These gate *whether a number may be produced / may be
# claimed*, they are NOT new accuracy pass thresholds for auxiliary metrics.
DEFAULT_COVERAGE_FLOOR = 0.30      # fraction of the fixed denominator that must
                                   # have reference support inside the ROI
DEFAULT_MIN_DENOMINATOR = 1000     # minimum fixed-denominator points
DEFAULT_MAX_DEGENERATE_FRAC = 0.99  # HARD degeneracy guard: an exact-coincidence
                                   # fraction beyond this means the two sample sets
                                   # are the same set (a copy / grid collapse), not a
                                   # measurement.  Lower fractions (0.05+) are only
                                   # WARNED about -- a legitimately perfect planar fit
                                   # also produces exact coincidences.
DEGENERACY_WARN_FRAC = 0.05
SUPPORT_RADIUS_M = 0.10            # "is there a reference point near this est point"
                              # -> distinguishes no-reference from drifted-out
E_EXACT_COINCIDENCE_M = 1e-6


# ------------------------------------------------------------------- hashing --
def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(chunk), b""):
            h.update(blk)
    return h.hexdigest()


def sha256_obj(obj):
    payload = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


# ------------------------------------------------------------------------ io --
def load_json(path, default=None):
    if not path or not os.path.isfile(path):
        return default
    with open(path) as fh:
        return json.load(fh)


def dump_json(path, obj):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(obj, fh, indent=1, sort_keys=True)
        fh.write("\n")
    return path


def read_tum(path):
    """TUM -> (t[n], pos[n,3], quat_xyzw[n,4]) sorted by time."""
    rows = []
    with open(path) as fh:
        for line in fh:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            v = s.split()
            if len(v) < 8:
                continue
            rows.append([float(x) for x in v[:8]])
    a = np.asarray(rows, dtype=float)
    order = np.argsort(a[:, 0], kind="stable")
    a = a[order]
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def read_pcd_points(path):
    """Minimal deterministic ASCII/binary PCD reader (no open3d dependency)."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:11] == b"# .PCD v0.7" or data.startswith(b"#"):
        pass
    head_end = data.find(b"DATA ")
    if head_end < 0:
        raise ValueError("not a PCD file: %s" % path)
    nl = data.find(b"\n", head_end)
    header = data[:nl].decode("ascii", "replace")
    kind = data[head_end + 5:nl].strip().decode()
    fields = []
    for line in header.splitlines():
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("POINTS"):
            n = int(line.split()[1])
    idx = [fields.index(c) for c in ("x", "y", "z")]
    body = data[nl + 1:]
    if kind == "ascii":
        vals = np.fromstring(body.decode("ascii", "replace"), sep=" ").reshape(-1, len(fields))
        return np.ascontiguousarray(vals[:, idx], dtype=float)
    if kind == "binary":
        dt = np.dtype([(f, "<f4") for f in fields])
        arr = np.frombuffer(body[:n * dt.itemsize], dtype=dt, count=n)
        return np.stack([arr[c] for c in ("x", "y", "z")], axis=1).astype(float)
    raise ValueError("unsupported PCD DATA kind %r" % kind)


def load_points(path):
    """Read a point cloud; fall back to open3d for binary_compressed PCD."""
    try:
        return read_pcd_points(path)
    except Exception:
        import open3d as o3d  # only for I/O; never for the metric itself
        return np.asarray(o3d.io.read_point_cloud(path).points, dtype=float)


def write_pcd_ascii(path, pts):
    pts = np.asarray(pts, dtype=float)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as fh:
        fh.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
                 "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                 "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                 "POINTS %d\nDATA ascii\n" % (len(pts), len(pts)))
        for p in pts:
            fh.write("%.9g %.9g %.9g\n" % (p[0], p[1], p[2]))
    return path


# ------------------------------------------------------- deterministic sample --
def grid_cells(points, voxel, origin):
    return np.floor((np.asarray(points, dtype=float) - np.asarray(origin, dtype=float)) / voxel).astype(np.int64)


def voxel_grid_select(points, voxel, origin=(0.0, 0.0, 0.0)):
    """Fixed-denominator sampler.

    For every occupied cell of the frozen grid, keep the single input point
    closest to the cell centre (ties -> lowest index).  Deterministic, order
    independent, and -- unlike a centroid downsample -- cannot manufacture a
    point that sits exactly on a reference voxel centre.
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return np.empty(0, dtype=int)
    org = np.asarray(origin, dtype=float)
    cell = grid_cells(pts, voxel, org)
    centres = (cell + 0.5) * voxel + org
    d = np.linalg.norm(pts - centres, axis=1)
    order = np.lexsort((d, cell[:, 2], cell[:, 1], cell[:, 0]))
    c = cell[order]
    first = np.ones(len(c), dtype=bool)
    first[1:] = np.any(c[1:] != c[:-1], axis=1)
    return np.sort(order[first])


def half_voxel_offset(voxel):
    h = 0.5 * float(voxel)
    return (h, h, h)


# ------------------------------------------------------------------- geometry --
def estimate_normals(points, query_points, k=15, radius=0.15, tree=None,
                     min_neighbours=4):
    """PCA normals for `query_points` from their neighbours in `points`.

    Neighbourhood = points within `radius` (at least `min_neighbours`, else the
    `k` nearest).  Sign is canonicalised to the largest-|component| positive;
    the metric uses |d . n| so the sign convention is irrelevant to the value.
    """
    pts = np.asarray(points, dtype=float)
    q = np.asarray(query_points, dtype=float)
    if tree is None:
        tree = cKDTree(pts, compact_nodes=True, balanced_tree=True, leafsize=32)
    kk = int(min(max(k, min_neighbours), len(pts)))
    dist, idx = tree.query(q, k=kk, workers=-1, distance_upper_bound=float(radius))
    dist = np.atleast_2d(dist)
    idx = np.atleast_2d(idx)
    normals = np.zeros((len(q), 3), dtype=float)
    deg = 0
    for i in range(len(q)):
        row = idx[i]
        row = row[np.isfinite(dist[i]) & (row < len(pts))]
        if len(row) < min_neighbours:
            row = np.atleast_1d(tree.query(q[i], k=kk, workers=-1)[1])
        nb = pts[row]
        if len(nb) < 3:
            normals[i] = (0.0, 0.0, 1.0)
            deg += 1
            continue
        c = nb - nb.mean(axis=0)
        w, v = np.linalg.eigh(c.T @ c)
        n = v[:, 0]
        j = int(np.argmax(np.abs(n)))
        if n[j] < 0:
            n = -n
        normals[i] = n
    return normals, deg


def plane_thickness(pts, k=15, seed=0, max_samples=20000):
    """Local plane-fit RMS residual (sqrt of smallest k-NN covariance eigenvalue)."""
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    if n < k + 1:
        return {"n": n, "note": "too few points"}
    rs = np.random.RandomState(seed)
    sel = np.arange(n) if n <= max_samples else rs.choice(n, max_samples, replace=False)
    tree = cKDTree(pts, compact_nodes=True, balanced_tree=True)
    _, nn = tree.query(pts[sel], k=min(k, n), workers=-1)
    res = np.empty(len(sel))
    for i, row in enumerate(nn):
        nb = pts[row]
        c = nb - nb.mean(axis=0)
        w = np.linalg.eigvalsh(c.T @ c) / max(1, len(nb) - 1)
        res[i] = math.sqrt(max(0.0, w[0]))
    return {"n": int(len(sel)), "rmse": float(np.sqrt(np.mean(res ** 2))),
            "median": float(np.median(res)), "p95": float(np.percentile(res, 95)),
            "max": float(np.max(res))}


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ------------------------------------------------------------------ alignment --
class AlignmentError(ValueError):
    pass


def assert_se3(T, tol=1e-6):
    """Reject anything that is not a rigid SE(3): scale / shear / non-rigid."""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise AlignmentError("transform is not 4x4")
    R = T[:3, :3]
    if not np.allclose(T[3], [0, 0, 0, 1], atol=tol):
        raise AlignmentError("bottom row is not [0,0,0,1] (projective term present)")
    scale = float(np.cbrt(abs(np.linalg.det(R))))
    if abs(scale - 1.0) > 1e-4:
        raise AlignmentError("scale != 1 (det=%.8f): scale alignment is forbidden" % np.linalg.det(R))
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-4):
        raise AlignmentError("rotation block is not orthonormal: shear / non-rigid is forbidden")
    if np.linalg.det(R) < 0:
        raise AlignmentError("reflection (det<0) is forbidden")
    return T


def assert_not_per_block(payload):
    """Guard against per-block / piecewise alignment anywhere in the record."""
    if isinstance(payload, dict):
        for key in ("per_block", "per_patch", "per_chunk", "piecewise", "blocks", "patches"):
            if key in payload:
                v = payload[key]
                if isinstance(v, list) and len(v) > 0:
                    raise AlignmentError("per-block alignment present (%s): forbidden" % key)
                if v is True:
                    raise AlignmentError("per-block alignment enabled (%s): forbidden" % key)
        for v in payload.values():
            assert_not_per_block(v)
    elif isinstance(payload, list):
        for v in payload:
            assert_not_per_block(v)
    return True


def apply_se3(T, pts):
    T = np.asarray(T, dtype=float)
    p = np.asarray(pts, dtype=float)
    return (T[:3, :3] @ p.T).T + T[:3, 3]


# ------------------------------------------------------------------- statistics --
def st(a, prefix="x"):
    a = np.asarray(a, dtype=float)
    if a.size == 0:
        return {"%s_n" % prefix: 0}
    return {
        "%s_n" % prefix: int(a.size),
        "%s_rmse" % prefix: float(np.sqrt(np.mean(a ** 2))),
        "%s_mean" % prefix: float(np.mean(a)),
        "%s_median" % prefix: float(np.median(a)),
        "%s_p95" % prefix: float(np.percentile(a, 95)),
        "%s_p99" % prefix: float(np.percentile(a, 99)),
        "%s_max" % prefix: float(np.max(a)),
        "%s_min" % prefix: float(np.min(a)),
    }


def frac_within(a, th):
    a = np.asarray(a, dtype=float)
    return float(np.mean(a <= th)) if a.size else None


def truncation_bracket(x, prefix, truncate_at=None):
    """Pre/post-truncation statistics for a FIXED denominator.

    Returns the raw stats (always), the truncated stats (only when a threshold
    was requested) and the exclusion bookkeeping that the contract requires:
    denominator, excluded count, excluded fraction.  Nothing is ever dropped
    silently.
    """
    x = np.asarray(x, dtype=float)
    denom = int(x.size)
    # uniform sign-invariance: one legitimately non-extra statistic is how many
    # correspondences are exact coincidences (a sampling degeneracy, not accuracy)
    n_exact = int(np.sum(x <= E_EXACT_COINCIDENCE_M))
    out = {
        "denominator_fixed_n": denom,
        "truncation_threshold_m": (None if truncate_at is None else float(truncate_at)),
        "truncation_applied": truncate_at is not None,
        "raw": st(x, prefix),
        "exact_coincidence_n": n_exact,
        "exact_coincidence_fraction": (float(n_exact) / denom) if denom else None,
    }
    if truncate_at is not None:
        keep = x <= float(truncate_at)
        out["truncated"] = st(x[keep], prefix)
        out["excluded_n"] = int(denom - keep.sum())
        out["excluded_fraction"] = float(1.0 - keep.mean())
        out["excluded_stats"] = st(x[~keep], prefix + "_excluded")
    return out
