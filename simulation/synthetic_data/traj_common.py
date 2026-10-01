#!/usr/bin/env python3
"""Shared ground-truth trajectory model for the FastLIO2 simulation harness.

Pure NumPy (no ROS): usable from a plain python3 shell, from the bag generator and
from the trajectory checker, so all three agree on one trajectory definition.

The motion is analytic on a 1 kHz grid:
  * heading follows a closed polygon whose corners are rounded by a circular boxcar
    smoothing of the tangent *direction* over ``corner_smooth_m`` metres (so the path is C1
    and the differentiated accelerations stay bounded);
  * position = start + the loop's cumulative arc length at the commanded speed + gait bob;
  * the stand->walk transition ramps the commanded speed with a smoothstep
    (``WALK_RAMP_S``), so the motion is C1 in speed and its 1 kHz finite-difference
    acceleration stays bounded (a step would be a 25 g spike that only the IMU sees);
  * velocity / acceleration / angular rate are finite differences of that same grid,
    hence the IMU specific force ``R^T (a_world - g)`` is consistent with the poses the
    LiDAR scans are rendered from.  The 10-30 Hz gait vibration is REAL body-frame motion
    (double-integrated displacement, integrated angle), not a measurement-only artefact:
    the LiDAR is rendered from exactly the motion the IMU reports, so the node's per-point
    undistortion corrects a distortion that is actually there.
  * the closed loop is handled so its seams are physically continuous: the heading table is
    extended by the loop's total turning at the arc-length wrap (a cumulative angle must not
    be interpolated modulo the table), and the sampled steps are adjusted to close the polygon
    exactly (otherwise the wrap blend becomes a position spike once per lap).  Both were
    real defects: they injected a ~300 rad/s gyro spike and a 157 m/s^2 accel spike once per
    lap, which the estimator integrated into a ~9 deg map tilt.  ``self_check`` bounds the
    derived stream so this cannot regress silently.

Frame conventions: world is Z-up (gravity (0, 0, -9.81)), body is FLU, yaw about Z.
"""

from __future__ import annotations

import copy
import json
import math
import os

import numpy as np

GRID_DT = 1e-3
GRAVITY_WORLD = np.array([0.0, 0.0, -9.81])

# Standstill -> cruise transition.  A step from 0 to speed_mps is not a realizable motion: its
# 1 kHz finite-difference acceleration is ~25 g for one sample (measured: 251 m/s^2), the IMU
# stream carries that spike and the estimator integrates it through its acceleration-bias and
# attitude states.  The commanded speed is therefore ramped with a smoothstep over
# WALK_RAMP_S seconds (commanded |a| <= 1.5 * speed_mps / WALK_RAMP_S = 1.5 m/s^2 here); the
# walking phase, the route geometry and the lap closure are unchanged.
WALK_RAMP_S = 0.5

# Physical sanity envelopes for the derived 1 kHz IMU / pose stream (kinematic self-check).
# Measured on this route: |gyro| <= ~1.8 rad/s, |accel| <= ~13 m/s^2 (gravity + gait vibration
# + spin-up), rotation increment <= ~0.10 deg/ms, position increment <= ~0.6 mm/ms.  The
# envelopes are deliberately generous (~2-5x): they are a fail-closed guard against a model
# DISCONTINUITY (a cumulative-angle wrap or a velocity step is 10-300x above them), not a
# precision claim.
KINEMATIC_MAX_GYRO_RPS = 10.0
KINEMATIC_MAX_ACCEL_MPS2 = 30.0
KINEMATIC_MAX_ROT_INCREMENT_DEG_PER_MS = 0.5
KINEMATIC_MAX_POS_INCREMENT_M_PER_MS = 0.002


def walk_arc_length(t, hold, v, ramp=WALK_RAMP_S):
    """Arc length of the walk phase: 0 during the hold, smoothstep spin-up, then constant speed.

    ``v * ramp * (u^3 - u^4/2)`` is the exact integral of the smoothstep speed profile
    ``v * u^2 * (3 - 2u)``, so the speed is C1 (bounded acceleration) and the cruise phase
    resumes exactly at ``v``.  A trajectory may set ``"walk_ramp_s": 0`` to keep a hard step
    (used by nothing in this harness; kept so the profile stays configurable).
    """
    t = np.asarray(t, float)
    ramp = float(ramp)
    if ramp <= 0.0:
        return float(v) * np.maximum(0.0, t - float(hold))
    u = np.clip((t - float(hold)) / ramp, 0.0, 1.0)
    return (float(v) * ramp * (u ** 3 - 0.5 * u ** 4)
            + float(v) * np.maximum(t - (float(hold) + ramp), 0.0))


def load_trajectory(path):
    """Load and sanity-check a trajectory definition (see test_trajectory.json)."""
    with open(path, "r") as fh:
        traj = json.load(fh)
    for key in ("t0_epoch_s", "duration_s", "sensors", "body", "gait", "scene"):
        if key not in traj:
            raise ValueError("trajectory %s: missing key %r" % (path, key))
    body = traj["body"]
    if len(body["rectangle_corners_xy"]) < 3:
        raise ValueError("trajectory %s: rectangle_corners_xy needs >= 3 corners" % path)
    if traj["sensors"]["lidar_hz"] <= 0 or traj["sensors"]["imu_hz"] <= 0:
        raise ValueError("trajectory %s: sensor rates must be positive" % path)
    return traj


def apply_variant(traj, clean_start=False, seed=None):
    """Return a deep-copied trajectory with an ablation variant applied.

    ``clean_start`` removes the gait vibration, the roll/pitch wobble and the body bob, so the
    trot-in-place prologue becomes a genuinely static init window: the static-window init path
    then validates (instead of the max-wait fallback) and acc_normalize becomes a no-op because
    the window mean is exactly -g.  The bag generator and the trajectory checker must apply the
    SAME variant or the ground truth would not match the data.
    """
    if not clean_start and seed is None:
        return copy.deepcopy(traj)
    out = copy.deepcopy(traj)
    if clean_start:
        out["gait"]["vibration"] = []
        for key in ("bob_amp_m", "roll_amp_deg", "pitch_amp_deg", "yaw_wobble_amp_deg"):
            out["gait"][key] = 0.0
    if seed is not None:
        out.setdefault("imu_noise", {})["seed"] = int(seed)
    variant = "clean_start" if clean_start else ""
    if seed is not None:
        variant += ("_" if variant else "") + "seed%d" % seed
    if variant:
        out["name"] = "%s_%s" % (traj.get("name", "trajectory"), variant)
        out["variant"] = variant
    return out


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    o, l = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, -s, o], -1), np.stack([s, c, o], -1), np.stack([o, o, l], -1)], -2)


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    o, l = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, o, s], -1), np.stack([o, l, o], -1), np.stack([-s, o, c], -1)], -2)


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    o, l = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([l, o, o], -1), np.stack([o, c, -s], -1), np.stack([o, s, c], -1)], -2)


def _circular_boxcar(x, win):
    """Circular moving average over ``win`` samples (win odd, clamped to len(x))."""
    n = len(x)
    win = int(max(1, min(win, n if n % 2 else n - 1)))
    if win <= 1 or win >= n:
        return x.copy()
    pad = win // 2
    ext = np.concatenate([x[-pad:], x, x[:pad]])
    kernel = np.ones(win) / win
    return np.convolve(ext, kernel, mode="valid")


def _closed_polyline(corners_xy, spacing):
    """Uniform-arc-length samples of a closed polygon, plus the loop length."""
    pts = np.asarray(corners_xy, float)
    edges = np.roll(pts, -1, axis=0) - pts
    elen = np.linalg.norm(edges, axis=1)
    loop_len = float(elen.sum())
    n = max(int(round(loop_len / spacing)), 8)
    ds = loop_len / n
    out = np.empty((n, 2))
    s = 0.0
    for i in range(n):
        d = s
        for k in range(len(pts)):
            if d <= elen[k] or k == len(pts) - 1:
                out[i] = pts[k] + edges[k] * (d / elen[k] if elen[k] > 0 else 0.0)
                break
            d -= elen[k]
        s += ds
    return out, loop_len, ds


def build_ground_truth(traj, duration_s=None, grid_dt=GRID_DT):
    """Sample the trajectory on a fine grid; returns a dict of arrays.

    Keys (TRUE motion, no sensor artefacts): t (n,), pos (n,3), rot (n,3,3), vel (n,3),
    acc (n,3), gyro (n,3), accel_body (n,3), heading (n,), arc_length (n,).
    Keys (what the IMU WOULD report): gyro_meas (n,3), accel_meas (n,3) = true + gait
    vibration + deterministic noise.  Only the *_meas streams may be written into the bag, and
    only the true streams may drive the LiDAR scan poses.
    """
    body = traj["body"]
    gait = traj["gait"]
    noise = traj.get("imu_noise", {})

    duration = float(traj["duration_s"] if duration_s is None else duration_s)
    n = int(round(duration / grid_dt)) + 1
    t = np.arange(n) * grid_dt

    # --- closed, corner-rounded path in the xy plane -------------------------
    path, loop_len, ds = _closed_polyline(body["rectangle_corners_xy"], 0.01)
    tangent = np.roll(path, -1, axis=0) - np.roll(path, 1, axis=0)
    head_ideal = np.arctan2(tangent[:, 1], tangent[:, 0])
    win = int(round(float(body["corner_smooth_m"]) / ds)) | 1
    head_smooth = np.arctan2(_circular_boxcar(np.sin(head_ideal), win),
                             _circular_boxcar(np.cos(head_ideal), win))
    # position of the loop from the smoothed heading (arc-length preserving)
    # The n steps of a CLOSED loop must sum to exactly zero.  Chaining the sampled heading
    # leaves a small residual (the chord/arc discrepancy of the discretisation), and the
    # arc-length interpolation blends the last sample with sample 0 across the seam, so any
    # residual appears once per lap as a position/velocity spike (measured: 157 m/s^2 on the
    # triangle route).  Distribute the residual uniformly over the samples: each step moves by
    # residual/n (~2.5e-6 m here), the loop closes exactly, and the seam blend becomes the same
    # chord approximation as every other sample.
    step = ds * np.stack([np.cos(head_smooth), np.sin(head_smooth)], 1)
    step = step - step.sum(axis=0) / len(step)
    loop_xy = np.cumsum(step, axis=0)
    loop_xy -= loop_xy[0]                      # start at origin
    head_unwrap = np.unwrap(head_smooth)
    head_unwrap = head_unwrap - head_unwrap[0]

    # --- arc-length profile: trot in place, then constant-speed walk ---------
    # The heading is a CUMULATIVE angle: going once around the loop it advances by the total
    # turning `turn` (2*pi for this route).  Sampling a cumulative angle modulo the array
    # length and then linearly blending the two ends of the array mixes head_unwrap[-1] (==turn)
    # with head_unwrap[0] (==0) and injects a spurious ~turn-wide yaw sweep into the LAST sample
    # interval of every lap (which the 1 kHz finite differences turn into a ~300 rad/s gyro and
    # a ~15 m/s^2 accelerometer spike ~20 ms per lap; the estimator integrates it and its map
    # tilts by ~9 deg for the rest of the run).  Extend the heading table by ONE entry holding
    # the continuation of the loop (the value at arc length loop_len, i.e. `turn`), so the
    # interpolation never straddles the seam.
    turn = head_unwrap[-1] + (head_smooth[0] - head_smooth[-1])
    head_ext = np.append(head_unwrap, turn)               # head_ext[len] = continuation
    arc = walk_arc_length(t, float(body["static_hold_s"]), float(body["speed_mps"]),
                          float(body.get("walk_ramp_s", WALK_RAMP_S)))
    frac = (arc / ds) % len(path)
    idx0 = np.floor(frac).astype(int)
    idx1 = (idx0 + 1) % len(path)
    w = (frac - idx0)[:, None]
    xy = loop_xy[idx0] * (1.0 - w) + loop_xy[idx1] * w
    head = head_ext[idx0] * (1.0 - w[:, 0]) + head_ext[idx0 + 1] * w[:, 0]
    # closed loop => heading only defined modulo the total turning; keep it continuous
    lap = np.floor(arc / loop_len)
    head = head + lap * turn

    start = np.asarray(body["start_xyz"], float)
    pos_base = np.empty((n, 3))
    pos_base[:, :2] = start[:2] + xy
    pos_base[:, 2] = start[2] + float(gait["bob_amp_m"]) * np.sin(2 * math.pi * float(gait["bob_hz"]) * t)

    # --- attitude: yaw = path heading, plus small gait roll/pitch/yaw wobble --
    roll = np.radians(float(gait["roll_amp_deg"])) * np.sin(2 * math.pi * float(gait["attitude_hz"]) * t)
    pitch = np.radians(float(gait["pitch_amp_deg"])) * np.sin(
        2 * math.pi * float(gait["attitude_hz"]) * t + 0.9)
    yaw = head + np.radians(float(gait["yaw_wobble_amp_deg"])) * np.sin(
        2 * math.pi * float(gait["yaw_wobble_hz"]) * t + 0.4)
    rot_base = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)

    # --- gait vibration as REAL body-frame motion ----------------------------
    # The 10-30 Hz vibration is a physical gait artefact of the quadruped, so it belongs to
    # the TRUE motion: the body really oscillates.  Each entry gives a body-frame specific
    # force amplitude [m/s^2] and a body angular-rate amplitude [rad/s] at frequency hz; the
    # corresponding displacement is the double integral (-A/w^2 * sin) and the corresponding
    # angle is the integral (-G/w * sin).  Putting it in the motion (instead of only in the
    # measured IMU) is what makes the sensor stream PHYSICALLY CONSISTENT: the LiDAR is
    # rendered from exactly the motion the IMU reports, so the node's per-point undistortion
    # corrects the distortion that is actually there.  Feeding a vibrating gyro while the
    # scans do not vibrate makes the estimator "correct" a distortion that does not exist and
    # smears the map (cm-level tails on far surfaces).
    vib_pos_body = np.zeros((n, 3))
    vib_theta = np.zeros((n, 3))
    for k, vib in enumerate(gait.get("vibration", [])):
        f = float(vib["hz"])
        w = 2.0 * math.pi * f
        amp = 1.0 + 0.3 * np.sin(2 * math.pi * 0.7 * t + k)      # slow envelope
        osc = amp * np.sin(w * t + 0.7 * k)
        vib_pos_body += np.outer(-osc / (w * w), np.asarray(vib["acc_amp_mps2"], float))
        vib_theta += np.outer(-osc / w, np.asarray(vib["gyro_amp_rps"], float))
    rot = rot_base @ _exp_so3(vib_theta)
    pos = pos_base + np.einsum("nij,nj->ni", rot, vib_pos_body)

    # --- derivatives: TRUE motion (also what renders the LiDAR) ---------------
    vel = np.gradient(pos, grid_dt, axis=0, edge_order=2)
    acc = np.gradient(vel, grid_dt, axis=0, edge_order=2)
    rdot = np.gradient(rot, grid_dt, axis=0, edge_order=2)
    skew = np.einsum("nji,njk->nik", rot, rdot)          # R^T Rdot
    gyro = np.stack([skew[:, 2, 1], skew[:, 0, 2], skew[:, 1, 0]], axis=1)
    accel_body = np.einsum("nji,nj->ni", rot, acc - GRAVITY_WORLD[None, :])

    # --- MEASURED IMU = true specific force + deterministic white noise --------
    # The gait vibration is already part of `accel_body`/`gyro` above (it is real motion), so
    # only the sensor noise is added here.  Nothing structured may be added to the measurement
    # that is not in the motion the LiDAR is rendered from.
    seed = int(noise.get("seed", 20260930))
    rng = np.random.default_rng(seed)
    accel_meas = accel_body + rng.normal(0.0, float(noise.get("accel_std_mps2", 0.0)), (n, 3))
    gyro_meas = gyro + rng.normal(0.0, float(noise.get("gyro_std_rps", 0.0)), (n, 3))

    return {"t": t, "pos": pos, "rot": rot, "vel": vel, "acc": acc, "gyro": gyro,
            "accel_body": accel_body, "gyro_meas": gyro_meas, "accel_meas": accel_meas,
            "heading": head, "arc_length": arc, "grid_dt": grid_dt}


def lidar_scan_times(traj, duration_s=None):
    """Scan stamps (scan END, matching the node's treat-every-point-at-stamp-time model)."""
    hz = float(traj["sensors"]["lidar_hz"])
    duration = float(traj["duration_s"] if duration_s is None else duration_s)
    k = np.arange(1, int(round(duration * hz)) + 1)
    return k / hz


def interp_state(gt, times):
    """Interpolate pose / velocity / angular rate at arbitrary times [s]."""
    t = gt["t"]
    out = {}
    for key in ("pos", "vel"):
        out[key] = np.stack([np.interp(times, t, gt[key][:, i]) for i in range(3)], axis=1)
    # rotations: interpolate the Euler-consistent matrix entries then re-orthonormalise
    mats = np.stack([np.stack([np.interp(times, t, gt["rot"][:, i, j]) for j in range(3)], 1)
                     for i in range(3)], 1)
    u, _, vt = np.linalg.svd(mats)
    out["rot"] = u @ vt
    out["rot"] = out["rot"] * np.sign(np.linalg.det(out["rot"]))[:, None, None]
    out["gyro"] = np.stack([np.interp(times, t, gt["gyro"][:, i]) for i in range(3)], axis=1)
    return out


def raycast_scene(origin, dirs, scene, rng, sensors):
    """Range of the first scene hit along each ray; NaN where nothing was hit.

    Scene = axis-aligned room box + vertical cylinders (pillars).  Vectorised over rays;
    ``origin`` may be a single (3,) point or one row per ray (N,3).
    """
    o = np.asarray(origin, float)
    d = np.asarray(dirs, float)
    if o.ndim == 1:
        o = np.broadcast_to(o, d.shape)
    lo = np.asarray(scene["room_box_min"], float)
    hi = np.asarray(scene["room_box_max"], float)
    cands = []
    for axis in range(3):
        other = [a for a in range(3) if a != axis]
        for plane in (lo[axis], hi[axis]):
            with np.errstate(divide="ignore", invalid="ignore"):
                tt = (plane - o[:, axis]) / d[:, axis]
                hit = o + tt[:, None] * d          # inf*0 -> NaN for parallel rays, masked by ok
            ok = (tt > 0) & np.isfinite(tt)
            for a in other:
                ok &= (hit[:, a] >= lo[a]) & (hit[:, a] <= hi[a])
            cands.append(np.where(ok, tt, np.inf))
    for pillar in scene.get("pillars", []):
        cx, cy = pillar["center_xy"]
        r = float(pillar["radius_m"])
        z0, z1 = pillar["z_range"]
        a = d[:, 0] ** 2 + d[:, 1] ** 2
        b = 2.0 * (d[:, 0] * (o[:, 0] - cx) + d[:, 1] * (o[:, 1] - cy))
        c = (o[:, 0] - cx) ** 2 + (o[:, 1] - cy) ** 2 - r * r
        disc = b * b - 4 * a * c
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (-b - np.sqrt(np.maximum(disc, 0.0))) / (2 * a)
        hit_z = o[:, 2] + tt * d[:, 2]
        ok = (disc > 0) & (a > 1e-12) & (tt > 0) & (hit_z >= z0) & (hit_z <= z1)
        cands.append(np.where(ok, tt, np.inf))
    rng_arr = np.min(np.stack(cands, 0), axis=0)
    rng_arr = np.where(np.isfinite(rng_arr), rng_arr, np.nan)
    rng_arr = rng_arr + rng.normal(0.0, float(sensors.get("lidar_range_noise_std_m", 0.0)), rng_arr.shape)
    return rng_arr


def lidar_directions(sensors):
    """Unit ray directions for the configured beam grid (elevation-major, azimuth fastest)."""
    v_lo, v_hi = [math.radians(float(x)) for x in sensors["lidar_vertical_fov_deg"]]
    n_v = int(sensors["lidar_vertical_beams"])
    n_h = int(sensors["lidar_horizontal_beams"])
    elev = np.linspace(v_lo, v_hi, n_v)
    azim = np.linspace(0.0, 2 * math.pi, n_h, endpoint=False)
    el = np.repeat(elev, n_h)
    az = np.tile(azim, n_v)
    ce, se = np.cos(el), np.sin(el)
    dirs = np.stack([ce * np.cos(az), ce * np.sin(az), se], axis=1)
    return dirs, az / (2 * math.pi)          # directions + normalised sweep fraction


def self_check(traj, duration_s=2.0, seed=0):
    """Numeric self-consistency check of the generator's geometry/time model.

    Guards the failure modes that are invisible in the bag but fatal to the estimator: a wrong
    per-ray pose (e.g. a Rodrigues coefficient that scales with the angle) distorts each scan
    differently, so a static platform looks like it is moving; and a discontinuity in the
    DERIVED kinematic stream (a cumulative-angle wrap at the closed-loop seam, or a stand->walk
    velocity step) puts a non-physical gyro/accel spike into the IMU that the estimator
    integrates into its own state and therefore into the map.  Returns residuals and the
    measured kinematic peaks; the caller decides pass/fail.  No ROS, no scipy.
    """
    out = {}
    rng = np.random.default_rng(seed)

    # (1) SO(3) sanity of the per-ray rotation used for in-scan distortion
    th = rng.normal(0.0, 0.2, (64, 3))
    R = _exp_so3(th)
    out["rot_orthogonality_max_err"] = float(
        np.abs(np.einsum("nij,nkj->nik", R, R) - np.eye(3)).max())
    ang = np.linalg.norm(th, axis=1)
    u = th / ang[:, None]
    out["rot_axis_invariance_max_err"] = float(np.abs(np.einsum("nij,nj->ni", R, u) - u).max())
    ref = np.tile(np.array([0.0, 0.0, 1.0]), (len(u), 1))
    perp = np.cross(u, ref)
    perp /= np.linalg.norm(perp, axis=1)[:, None]
    rv = np.einsum("nij,nj->ni", R, perp)
    out["rot_angle_max_err"] = float(np.abs(np.arccos(np.clip(np.einsum("ni,ni->n", rv, perp), -1.0, 1.0))
                                            - ang).max())
    out["rot_identity_max_err"] = float(np.abs(_exp_so3(np.zeros((1, 3)))[0] - np.eye(3)).max())

    # (2) the vectorised ray caster must agree with a naive per-ray reference
    scene = traj["scene"]
    lo = np.asarray(scene["room_box_min"], float)
    hi = np.asarray(scene["room_box_max"], float)
    dirs, az_frac = lidar_directions(traj["sensors"])
    quiet = dict(traj["sensors"])
    quiet.update(lidar_range_noise_std_m=0.0, lidar_point_noise_std_m=0.0, lidar_dropout_frac=0.0)
    origin = np.asarray(traj["body"]["start_xyz"], float)
    pick = rng.choice(len(dirs), size=min(200, len(dirs)), replace=False)
    vec_ranges = raycast_scene(origin, dirs[pick], scene, np.random.default_rng(seed + 3), quiet)

    def brute(o, d):
        best = np.inf
        for axis in range(3):
            other = [a for a in range(3) if a != axis]
            for plane in (lo[axis], hi[axis]):
                if abs(d[axis]) < 1e-12:
                    continue
                t = (plane - o[axis]) / d[axis]
                if t <= 0:
                    continue
                h = o + t * d
                if all(lo[a] - 1e-9 <= h[a] <= hi[a] + 1e-9 for a in other):
                    best = min(best, t)
        for pillar in scene.get("pillars", []):
            cx, cy = pillar["center_xy"]
            r, (z0, z1) = float(pillar["radius_m"]), pillar["z_range"]
            a = d[0] ** 2 + d[1] ** 2
            if a < 1e-12:
                continue
            b = 2.0 * (d[0] * (o[0] - cx) + d[1] * (o[1] - cy))
            c = (o[0] - cx) ** 2 + (o[1] - cy) ** 2 - r * r
            disc = b * b - 4 * a * c
            if disc <= 0:
                continue
            t = (-b - np.sqrt(disc)) / (2 * a)
            if t > 0 and z0 - 1e-9 <= o[2] + t * d[2] <= z1 + 1e-9:
                best = min(best, t)
        return best

    ref_ranges = np.array([brute(origin, d) for d in dirs[pick]])
    finite = np.isfinite(vec_ranges) & np.isfinite(ref_ranges)
    out["raycast_vs_reference_max_err_m"] = float(np.abs(vec_ranges[finite] - ref_ranges[finite]).max()) \
        if finite.any() else float("inf")
    out["rays_without_hit"] = int((~np.isfinite(ref_ranges)).sum())

    # (3) a frozen platform must produce byte-identical scans when noise is off: any per-scan
    #     pose/frame error shows up here as a non-zero difference
    frozen = apply_variant(traj, clean_start=True)
    frozen["body"]["speed_mps"] = 0.0
    frozen["body"]["static_hold_s"] = duration_s
    gt = build_ground_truth(frozen, duration_s=duration_s)
    period = 1.0 / float(quiet["lidar_hz"])
    a = render_scan(gt, 0.0, period, dirs, az_frac, scene, np.random.default_rng(seed + 1), quiet)[0]
    b = render_scan(gt, 1.0, period, dirs, az_frac, scene, np.random.default_rng(seed + 2), quiet)[0]
    out["points_per_scan"] = int(a.shape[0])
    out["frozen_scan_max_abs_diff_m"] = float(np.abs(a[:, :3] - b[:, :3]).max()) if a.shape == b.shape \
        else float("inf")
    ranges = np.linalg.norm(a[:, :3], axis=1)
    out["range_min_m"] = float(ranges.min())
    out["range_max_m"] = float(ranges.max())
    out["range_within_limits_frac"] = float(
        ((ranges >= quiet["lidar_min_range_m"] - 1e-6) & (ranges <= quiet["lidar_max_range_m"] + 1e-6)).mean())

    # (4) the emitted per-point time field must be the node's own parsing contract: index 0
    #     carries dt = 0 and the values are the physical emission times rebased on it, so the
    #     node's t0 (first range-filtered point) and cloud_end_time are exact.
    out["time_field_first_point_s"] = float(a[0, 4])
    out["time_field_monotonic"] = bool(np.all(np.diff(a[:, 4]) >= -1e-7))
    out["time_field_span_s"] = float(a[-1, 4])

    # (5) the DERIVED 1 kHz IMU / pose stream must be physically realizable over the WHOLE route.
    #     This is the failure mode that is invisible in the scan geometry and in the route stats
    #     but fatal to the estimator: a model DISCONTINUITY (a cumulative-angle wrap at the
    #     closed-loop seam, or a stand->walk velocity step) shows up only in the finite
    #     differences, as a gyro/accel spike or a rotation/position increment spike that the
    #     estimator integrates into its map.  Measured and bounded here, not assumed.
    full = build_ground_truth(traj, duration_s=float(traj["duration_s"]))
    f_rot, f_pos, f_t = full["rot"], full["pos"], full["t"]
    f_gyro = np.linalg.norm(full["gyro_meas"], axis=1)
    f_acc = np.linalg.norm(full["accel_meas"], axis=1)
    dR = np.einsum("nji,njk->nik", f_rot[:-1], f_rot[1:])
    rot_inc = np.degrees(np.arccos(np.clip((np.trace(dR, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    pos_inc = np.linalg.norm(np.diff(f_pos, axis=0), axis=1)
    out["kinematic_route_s"] = float(f_t[-1]) if len(f_t) else 0.0
    out["max_gyro_meas_rps"] = float(f_gyro.max())
    out["max_accel_meas_mps2"] = float(f_acc.max())
    out["max_rot_increment_deg_per_ms"] = float(rot_inc.max())
    out["max_pos_increment_m_per_ms"] = float(pos_inc.max())
    out["kinematic_envelope"] = {
        "max_gyro_meas_rps": KINEMATIC_MAX_GYRO_RPS,
        "max_accel_meas_mps2": KINEMATIC_MAX_ACCEL_MPS2,
        "max_rot_increment_deg_per_ms": KINEMATIC_MAX_ROT_INCREMENT_DEG_PER_MS,
        "max_pos_increment_m_per_ms": KINEMATIC_MAX_POS_INCREMENT_M_PER_MS,
    }
    # (6) the rendered sweep must be the ACTUAL motion: re-render one scan and check every kept
    #     ray against an independent computation of the same ray/scene intersection, using the
    #     interpolated GT pose AT THAT RAY'S OWN EMISSION TIME (the direction is recoverable
    #     because every point lies on its ray, i.e. p_body / |p_body| is the ray direction in the
    #     sensor frame at that time).  A midpoint twist extrapolation - the pre-fix renderer -
    #     deviates from this by up to ~mm and would fail the tight tolerance.
    quiet_scan = dict(traj["sensors"])
    quiet_scan.update(lidar_range_noise_std_m=0.0, lidar_point_noise_std_m=0.0, lidar_dropout_frac=0.0)
    dirs_q, az_q = lidar_directions(quiet_scan)
    pts_q, t_first_q = render_scan(full, 0.0, 1.0 / float(quiet_scan["lidar_hz"]), dirs_q, az_q,
                                   traj["scene"], np.random.default_rng(seed + 5), quiet_scan)
    t_ray = t_first_q + pts_q[:, 4]
    st_ray = interp_state(full, t_ray)
    body = pts_q[:, :3].astype(float)
    r_body = np.linalg.norm(body, axis=1)
    d_body = body / r_body[:, None]
    d_world = np.einsum("nij,nj->ni", st_ray["rot"], d_body)
    r_ref = raycast_scene(st_ray["pos"], d_world, traj["scene"],
                          np.random.default_rng(seed + 6), quiet_scan)
    ok = np.isfinite(r_ref)
    out["ray_pose_max_range_err_m"] = float(np.abs(r_body[ok] - r_ref[ok]).max()) if ok.any() else None
    out["ray_pose_rays_checked"] = int(ok.sum())

    out["kinematic_violations"] = [
        name for name, val, lim in (
            ("max_gyro_meas_rps", out["max_gyro_meas_rps"], KINEMATIC_MAX_GYRO_RPS),
            ("max_accel_meas_mps2", out["max_accel_meas_mps2"], KINEMATIC_MAX_ACCEL_MPS2),
            ("max_rot_increment_deg_per_ms", out["max_rot_increment_deg_per_ms"],
             KINEMATIC_MAX_ROT_INCREMENT_DEG_PER_MS),
            ("max_pos_increment_m_per_ms", out["max_pos_increment_m_per_ms"],
             KINEMATIC_MAX_POS_INCREMENT_M_PER_MS))
        if not (val <= lim)]
    return out


def _exp_so3(theta):
    """Rodrigues exp of so(3) vectors, (N,3) -> (N,3,3).

    R = I + sin(a) K + (1 - cos(a)) K^2, with a = |theta| and K = skew(theta/|theta|) the
    skew matrix of the UNIT axis.  (Scaling the sin/cos coefficients by 1/a or 1/a^2 is only
    correct when K is built from the unnormalised theta -- mixing the two conventions silently
    produces rotations of ~a instead of ~a/3, which corrupts every per-ray pose.)
    """
    theta = np.atleast_2d(np.asarray(theta, float))
    ang = np.linalg.norm(theta, axis=1)
    safe = np.where(ang > 0.0, ang, 1.0)
    k = theta / safe[:, None]
    zero = np.zeros_like(ang)
    kx = np.stack([np.stack([zero, -k[:, 2], k[:, 1]], -1),
                   np.stack([k[:, 2], zero, -k[:, 0]], -1),
                   np.stack([-k[:, 1], k[:, 0], zero], -1)], -2)
    s = np.where(ang > 0.0, np.sin(ang), 0.0)
    c = np.where(ang > 0.0, 1.0 - np.cos(ang), 0.0)
    eye = np.broadcast_to(np.eye(3), (len(theta), 3, 3))
    return eye + s[:, None, None] * kx + c[:, None, None] * (kx @ kx)


def render_scan(gt, t_scan_start, period, dirs, az_frac, scene, rng, sensors):
    """Ray-cast one spinning scan from the ACTUAL motion at each ray's emission time.

    Every ray is rendered from the ground-truth pose INTERPOLATED AT THAT RAY'S OWN emission
    time (``t_scan_start + az_frac * period``), i.e. from exactly the motion the IMU reports and
    the GT TUM records.  An earlier version extrapolated a single midpoint pose with a linear
    translation and a constant body rate over the whole 100 ms sweep; that is only a first-order
    approximation of the 10-30 Hz gait motion (and it composed the body-frame rate with a LEFT
    multiplication, ``exp(w dt) R``, which is the world-frame convention, not the body-frame
    ``R exp(w dt)``), so away from the midpoint the rendered poses were not the realized poses.
    Physical sensor/GT consistency is the point of this harness, so the exact poses are used.

    Returns ``(pts, t_first_s)``: ``pts`` is (N,5) float32 [x, y, z, intensity, dt] where
    ``dt`` is the emission time of the point RELATIVE TO THE FIRST EMITTED POINT (so
    ``pts[0, 4] == 0``), and ``t_first_s`` is that first point's emission time relative to
    the scan start.  This is the node's own parsing contract: ``Utils::pcl2_to_PCL`` takes
    ``t0`` from the first RANGE-FILTERED point and emits ``curvature = (t - t0) * 1000``
    ms, and ``syncPackage`` sets ``cloud_end_time = header_stamp + last curvature``.  The
    caller must therefore stamp the message HEADER with ``scan_start + t_first_s`` so that the
    header really is the first point's emission time and the reconstructed frame end is
    the last point's emission time (a header at the scan start instead would shift every
    per-point time by the first retained azimuth and distort the undistortion model).  The
    message's STORAGE stamp is a different thing: it is the frame's readiness, i.e. the header
    plus the last point's time field, which is what ``iter_messages``/``write_bag`` publish.
    """
    t_ray = t_scan_start + az_frac * period           # each ray's own emission time
    st = interp_state(gt, t_ray)
    o_ray, rot_ray = st["pos"], st["rot"]             # exact GT pose at each ray's time
    d_world = np.einsum("nij,nj->ni", rot_ray, dirs)

    rng_arr = raycast_scene(o_ray, d_world, scene, rng, sensors)
    keep = np.isfinite(rng_arr)
    keep &= rng_arr >= float(sensors.get("lidar_min_range_m", 0.0))
    keep &= rng_arr <= float(sensors.get("lidar_max_range_m", np.inf))
    rng_arr, d_keep, o_keep, rot_keep, az_keep = (
        rng_arr[keep], d_world[keep], o_ray[keep], rot_ray[keep], az_frac[keep])
    if float(sensors.get("lidar_dropout_frac", 0.0)) > 0.0:
        keep2 = rng.random(rng_arr.shape[0]) >= float(sensors["lidar_dropout_frac"])
        rng_arr, d_keep, o_keep, rot_keep, az_keep = (
            rng_arr[keep2], d_keep[keep2], o_keep[keep2], rot_keep[keep2], az_keep[keep2])

    pts_world = o_keep + rng_arr[:, None] * d_keep
    pts_world = pts_world + rng.normal(0.0, float(sensors.get("lidar_point_noise_std_m", 0.0)), pts_world.shape)
    pts_body = np.einsum("nji,nj->ni", rot_keep, pts_world - o_keep)   # world -> sensor frame
    inten = np.clip(1.0 - rng_arr / float(sensors["lidar_max_range_m"]), 0.05, 1.0)
    # Emit the sweep in EMISSION-TIME order.  Utils::pcl2_to_PCL rebases every per-point time
    # on the FIRST point it retains, so the message must start with the globally earliest
    # point: otherwise the earliest points would get a NEGATIVE curvature and the node's
    # cloud_end_time (header + max curvature) would no longer be the last emission time.
    t_rel = az_keep * period
    order = np.argsort(t_rel, kind="stable")
    pts_body, inten, t_rel = pts_body[order], inten[order], t_rel[order]
    t_first = float(t_rel[0]) if t_rel.size else 0.0
    pts = np.concatenate([pts_body, inten[:, None], (t_rel - t_first)[:, None]], axis=1)
    return pts.astype(np.float32), t_first



# --------------------------------------------------------------------- scene ----
def scene_surface_points(scene, spacing=0.03):
    """Dense point sampling of the scene's static surfaces, in the SCENE frame.

    This is the harness's own REFERENCE GEOMETRY: the room box's six faces plus every
    pillar's lateral surface, sampled on a uniform ``spacing`` grid.  It is generated
    analytically from ``scene`` alone -- no estimated map, no dataset reference, no fitted
    transform enters it -- so it can serve as the ground-truth surface cloud of the
    synthetic scene's own frame.
    """
    lo = np.asarray(scene["room_box_min"], float)
    hi = np.asarray(scene["room_box_max"], float)
    step = float(spacing)
    if step <= 0.0:
        raise ValueError("spacing must be positive")
    out = []
    for axis in range(3):
        other = [a for a in range(3) if a != axis]
        grids = [np.arange(lo[a], hi[a] + 0.5 * step, step) for a in other]
        A, B = np.meshgrid(*grids, indexing="ij")
        for plane in (lo[axis], hi[axis]):
            pts = np.empty((A.size, 3), float)
            pts[:, other[0]] = A.ravel()
            pts[:, other[1]] = B.ravel()
            pts[:, axis] = plane
            out.append(pts)
    for pillar in scene.get("pillars", []):
        cx, cy = [float(x) for x in pillar["center_xy"]]
        r = float(pillar["radius_m"])
        z0, z1 = [float(z) for z in pillar["z_range"]]
        n_az = max(8, int(round(2.0 * math.pi * r / step)))
        n_z = max(2, int(round((z1 - z0) / step)) + 1)
        az = np.linspace(0.0, 2.0 * math.pi, n_az, endpoint=False)
        zs = np.linspace(z0, z1, n_z)
        AZ, ZZ = np.meshgrid(az, zs, indexing="ij")
        pts = np.stack([cx + r * np.cos(AZ.ravel()), cy + r * np.sin(AZ.ravel()),
                        ZZ.ravel()], axis=1)
        out.append(pts)
    return np.concatenate(out, axis=0)


def scene_roi(traj, margin_m=0.5, roi_id=None, frozen_utc=None, reference_map_sha256=None,
              author="SimulationIntegration"):
    """The frozen static support region of the synthetic scene (roi_schema.json v1).

    An axis-aligned box around the scene's room box, expanded by ``margin_m`` so that the
    LiDAR returns that lie exactly ON the surfaces stay inside.  The bounds come from the
    scene definition only: neither the estimated map nor any residual of a run enters the
    derivation, which is what the evaluator's R38/provenance gate requires.
    """
    lo = np.asarray(traj["scene"]["room_box_min"], float) - float(margin_m)
    hi = np.asarray(traj["scene"]["room_box_max"], float) + float(margin_m)
    return {
        "schema_version": 1,
        "roi_id": roi_id or ("synthetic_scene_box_%s" % str(traj.get("name", "traj"))),
        "frame": "scene",
        "type": "aabb",
        "bounds": {"min": [float(x) for x in lo], "max": [float(x) for x in hi]},
        "provenance": {
            "source": "synthetic scene definition (test_trajectory.json: scene.room_box_min / "
                      "scene.room_box_max)",
            "derivation": "axis-aligned box of the scene room box expanded by %.3f m on every "
                          "axis; derived from the scene definition alone, never from an "
                          "estimated map or from this run's residuals" % float(margin_m),
            "reference_map_sha256": reference_map_sha256,
            "frozen_utc": frozen_utc or _utcnow(),
            "author": author,
            "independent_of_estimate": True,
            "independent_of_error": True,
            "notes": "the synthetic scene's own support region; no external (TUHH/MCD/HILTI) "
                     "reference map, ROI or transform is used anywhere in this harness",
        },
    }


def _utcnow():
    import datetime
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def quat_from_rot(rot):
    """(N,3,3) rotation matrices -> (N,4) quaternions in ROS order (x, y, z, w)."""
    R = np.asarray(rot, float)
    n = len(R)
    q = np.empty((n, 4), float)
    m00, m11, m22 = R[:, 0, 0], R[:, 1, 1], R[:, 2, 2]
    tr = m00 + m11 + m22
    for i in range(n):
        r = R[i]
        t = tr[i]
        if t > 0.0:
            s = math.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
        q[i] = (x, y, z, w)
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def write_gt_tum(traj, duration_s, path, epoch_stamps=True, grid_dt=GRID_DT):
    """Write the TRUE body trajectory as TUM (t x y z qx qy qz qw) in the scene frame.

    The pose is the rigid-body pose the LiDAR scans are rendered from, so a localization
    estimate expressed in the scene frame can be compared with it directly -- with no
    post-fit alignment of any kind.
    """
    gt = build_ground_truth(traj, duration_s=duration_s, grid_dt=grid_dt)
    t = gt["t"] + (float(traj["t0_epoch_s"]) if epoch_stamps else 0.0)
    q = quat_from_rot(gt["rot"])
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w") as fh:
        for i in range(len(t)):
            fh.write("%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                     % (t[i], gt["pos"][i, 0], gt["pos"][i, 1], gt["pos"][i, 2],
                        q[i, 0], q[i, 1], q[i, 2], q[i, 3]))
    return {"samples": int(len(t)), "path": path,
            "span_s": [float(t[0]), float(t[-1])] if len(t) else None,
            "frame": "scene", "epoch_stamps": bool(epoch_stamps)}


def route_stats(traj, duration_s=None, moving_eps=0.05, target_mps=0.5, max_mps=1.0,
                speed_tol_mps=0.005):
    """Closed-route completion and moving-segment speed statistics of the TRUE motion.

    ``completed`` means the sampled window covers a full lap of the closed route, so the
    body really returns to its start.  ``moving_*`` are the 3-D speed statistics over the
    segments whose speed exceeds ``moving_eps`` (the walking phase); they are the ground
    truth against which the required ``>= 0.5 m/s`` moving speed is checked.
    """
    duration = float(traj["duration_s"] if duration_s is None else duration_s)
    gt = build_ground_truth(traj, duration_s=duration)
    pos, vel, t = gt["pos"], gt["vel"], gt["t"]
    speed = np.linalg.norm(vel, axis=1)
    body = traj["body"]
    _, loop_len, _ = _closed_polyline(body["rectangle_corners_xy"], 0.01)
    v = float(body["speed_mps"])
    hold = float(body["static_hold_s"])
    # The walk completes the loop where the model's own ARC LENGTH reaches loop_len.  With the
    # C1 stand->walk spin-up that is slightly later than hold + loop_len/v, so the lap end is
    # read from the arc-length profile (not assumed) and the closure is measured at that time by
    # interpolation, i.e. it is the geometric closure of the route rather than a grid artefact.
    lap_end = float(np.interp(loop_len, gt["arc_length"], t))
    moving = speed > float(moving_eps)
    # "core" moving phase = the walk proper, from 1 s after the hold to the end of the first
    # lap: it excludes the stand-to-walk transition, where the speed legitimately ramps.
    core = (t >= hold + 1.0) & (t <= lap_end)
    start_xy = np.asarray(body["start_xyz"], float)[:2]
    p_lap = np.array([np.interp(lap_end, t, pos[:, k]) for k in range(3)])
    p_hold = np.array([np.interp(hold, t, pos[:, k]) for k in range(3)])
    closure = float(np.linalg.norm(p_lap[:2] - p_hold[:2]))
    return {
        "duration_s": duration,
        "loop_length_m": float(loop_len),
        "planned_lap_end_s": float(lap_end),
        "completed_full_lap": bool(duration >= lap_end - 1e-9),
        "lap_closure_error_m": closure,
        "path_length_m": float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
        "moving_fraction": float(moving.mean()),
        "moving_speed_min_mps": float(speed[moving].min()) if moving.any() else None,
        "moving_speed_median_mps": float(np.median(speed[moving])) if moving.any() else None,
        "moving_speed_max_mps": float(speed[moving].max()) if moving.any() else None,
        "moving_speed_p05_mps": float(np.percentile(speed[moving], 5)) if moving.any() else None,
        "target_mps": float(target_mps),
        "max_mps": float(max_mps),
        "core_moving_window_s": [float(hold + 1.0), float(lap_end)],
        "frac_moving_ge_target": float((speed[moving] >= float(target_mps)).mean())
                                 if moving.any() else None,
        "core_speed_min_mps": float(speed[core].min()) if core.any() else None,
        "core_speed_max_mps": float(speed[core].max()) if core.any() else None,
        "speed_tol_mps": float(speed_tol_mps),
        "core_speed_mean_mps": float(speed[core].mean()) if core.any() else None,
        # The envelope is checked on the MEAN walking speed, not on the instantaneous minimum:
        # the gait vibration is real motion and makes the instantaneous speed oscillate by
        # ~+/-0.02 m/s around the commanded value, which is a gait artefact and not a violation
        # of "the robot moves at >= target".  min/median/max are all reported so the oscillation
        # is visible.
        "moving_ge_target_and_le_max": bool(
            core.any()
            and (speed[core].mean() >= float(target_mps) - float(speed_tol_mps))
            and (speed[core].max() <= float(max_mps) + float(speed_tol_mps))),
        "start_xy_m": [float(x) for x in start_xy],
    }


def voxel_unique(pts, voxel):
    """Keep one point per occupied ``voxel`` cell (lowest original index wins)."""
    if voxel <= 0.0 or not len(pts):
        return np.asarray(pts, float)
    keys = np.floor(np.asarray(pts, float) / float(voxel)).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return np.asarray(pts, float)[np.sort(keep)]


def observed_surface_points(traj, duration_s=None, sample_hz=1.0, voxel=0.05, sensors=None,
                            t_start=0.0):
    """REFERENCE GEOMETRY as the sensor can actually observe it from the TRUE trajectory.

    The room shell alone is the wrong reference for a map-accuracy metric: a LiDAR cannot
    observe the floor inside its own blind cone or the ceiling beyond its vertical field of
    view, so those surface regions would look like "the estimate abandoned them".  This
    function ray-casts the scene from the GROUND-TRUTH sensor poses (with the configured beam
    grid, field of view and range limits, and no noise or dropout) and returns the union of
    the hit points in the SCENE frame -- i.e. the ground-truth surface geometry restricted to
    the observable support.

    Its inputs are the scene definition and the trajectory definition only: no estimated map,
    no residual and no external dataset enters it, so it satisfies the evaluator's
    pre-defined-support provenance rule.  ``t_start`` restricts the ray-cast poses to the
    EVALUATION window, so the reference contains exactly the geometry that window can observe
    (regions observed only outside it would otherwise look abandoned).
    """
    sensors = dict(sensors or traj["sensors"])
    quiet = dict(sensors)
    quiet.update(lidar_range_noise_std_m=0.0, lidar_point_noise_std_m=0.0, lidar_dropout_frac=0.0)
    dirs, _ = lidar_directions(quiet)
    duration = float(traj["duration_s"] if duration_s is None else duration_s)
    gt = build_ground_truth(traj, duration_s=duration)
    times = np.arange(float(t_start), duration, 1.0 / float(sample_hz))
    st = interp_state(gt, times)
    rng = np.random.default_rng(0)
    chunks = []
    for k in range(len(times)):
        d_world = np.einsum("ij,nj->ni", st["rot"][k], dirs)
        r = raycast_scene(st["pos"][k], d_world, traj["scene"], rng, quiet)
        keep = np.isfinite(r)
        keep &= r >= float(quiet["lidar_min_range_m"])
        keep &= r <= float(quiet["lidar_max_range_m"])
        chunks.append(st["pos"][k][None, :] + r[keep, None] * d_world[keep])
    pts = np.concatenate(chunks, axis=0) if chunks else np.zeros((0, 3))
    return voxel_unique(pts, voxel), {"poses": int(len(times)), "sample_hz": float(sample_hz),
                                      "voxel_m": float(voxel), "points_raw": int(len(pts)),
                                      "t_start_s": float(t_start), "t_end_s": float(duration)}
