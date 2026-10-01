#!/usr/bin/env python3
"""Chain-fit conditioning: position-only (current) vs position+orientation SE(3) fit.

Both use ONLY the run's pre-declared fit window (no evaluation samples).
"""
import os
import sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
REF = ROOT + "/artifacts/simulation_20260930/scene_ref"
T0 = 1700000000.0


def read_tum(p):
    r = np.loadtxt(p, ndmin=2)
    q = r[:, 4:8]
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                  np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                  np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)
    return r[:, 0], r[:, 1:4], R


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def fit(gt_p, gt_R, es_p, es_R, w_R=0.0):
    """SE(3) mapping est -> gt; w_R = weight of the orientation term (in metres per radian)."""
    mu_g, mu_e = gt_p.mean(0), es_p.mean(0)
    H = (es_p - mu_e).T @ (gt_p - mu_g) / len(gt_p)
    if w_R > 0.0:
        H = H + w_R * np.einsum("nij,nkj->ik", es_R, gt_R) / len(gt_p)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    t = mu_g - R @ mu_e
    return R, t, H


def main():
    run = sys.argv[1] if len(sys.argv) > 1 else ROOT + "/artifacts/simulation_20260930/baseline/mapping"
    g_t, g_p, g_R = read_tum(REF + "/gt_mapping.tum")
    e_t, e_p, e_R = read_tum(run + "/odom.tum")
    # fit window: sim time <= 29.5 s (the tool's pre-declared window)
    m = (g_t - T0 <= 29.5)
    print("gt samples in fit window: %d" % m.sum())
    # pair by exact stamp (odom stamps are bag stamps present in the 1 kHz GT? use nearest)
    idx = np.searchsorted(g_t, e_t, side="left")
    idx = np.clip(idx, 0, len(g_t) - 1)
    inw = (e_t - T0) <= 29.5
    gp, gR = g_p[idx], g_R[idx]
    ep, eR = e_p[inw], e_R[inw]
    gp, gR = gp[inw], gR[inw]
    print("paired samples: %d  (t %.2f..%.2f)" % (len(ep), e_t[inw][0] - T0, e_t[inw][-1] - T0))
    for w_R in (0.0, 0.5, 1.0, 3.0, 10.0):
        R, t, H = fit(gp, gR, ep, eR, w_R=w_R)
        resid_p = np.linalg.norm((ep @ R.T + t) - gp, axis=1)
        resid_R = np.array([rot_deg(R @ eR[i].T @ gp[i] * 0 + (eR[i] @ R.T).T) for i in range(0)])
        ang = np.array([rot_deg((R.T @ eR[i]) @ gR[i].T) if False else rot_deg(R @ eR[i] @ gR[i].T)
                        for i in range(len(ep))])
        print("w_R=%4.1f  rot %7.4f deg  t=%s  pos rmse %.5f m  orient rmse %.5f deg"
              % (w_R, rot_deg(R), np.round(t, 5), np.sqrt((resid_p ** 2).mean()),
                 np.sqrt((ang ** 2).mean())))
        if w_R in (0.0, 3.0):
            euler = R
            print("        R=\n%s" % np.round(R, 6))


if __name__ == "__main__":
    main()
