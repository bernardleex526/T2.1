#!/usr/bin/env python3
"""Allan-variance calibration of the IMU noise parameters na / ng / nba / nbg.

WHY THIS EXISTS
---------------
``lio_orin_nx.yaml`` carries na/ng/nba/nbg copied from a different sensor pair
(MID360 + VN200).  The IESKF weights the IMU against the LiDAR through exactly
these numbers, so on a different IMU the filter is mis-weighted by construction.
The correct values come from an Allan deviation of a long static recording.

WHAT IT COMPUTES
----------------
Overlapping Allan deviation of each axis of the gyro and accel triples:

  * ``ng``  [rad/s/√Hz] : white-noise floor, read from the -1/2 slope region,
                          ``sigma(tau) = ng / sqrt(tau)``  =>  ``ng = sigma(tau)*sqrt(tau)``
  * ``na``  [m/s²/√Hz]  : same, from the accel axes
  * ``nbg`` [rad/s^2/√Hz] and ``nba`` [m/s^3/√Hz] : bias-instability random
                          walk, read from the +1/2 slope region,
                          ``sigma(tau) = nbg * sqrt(tau/3)`` => ``nbg = sigma/sqrt(tau/3)``
  * a bias-instability minimum, reported for information

HOW IT DECIDES (no hidden constants)
------------------------------------
The two regions are found by fitting the local log-log slope and selecting the
window where it is closest to -0.5 (white noise) / +0.5 (random walk).  If a
region is absent the tool says so and prints nothing for that parameter: a
fabricated number here propagates straight into the estimator.

USAGE
-----
    # 6-12 h of a STATIONARY recording (see docs/calibration_procedure.md)
    python3 tools/imu_allan_variance.py --bag imu_static --topic /imu/data --emit-yaml
    python3 tools/imu_allan_variance.py --csv imu_static.csv --plot allan.png

Input must be STATIONARY.  The tool refuses a recording whose own motion
statistics say otherwise, because an Allan deviation of moving data is
meaningless.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import probe_io  # noqa: E402


# --------------------------------------------------------------------------
# Core estimator
# --------------------------------------------------------------------------

def overlapping_allan_deviation(x: np.ndarray, rate: float,
                                taus: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of ``x`` sampled at ``rate`` [Hz].

    Returns (taus, adev).  The overlapping estimator uses every possible
    averaging window, which is why it needs far less data than the disjoint one
    for the same confidence - the standard choice for IMU calibration.

    The estimator is the standard OADEV:

        sigma^2(tau) = 1/(2*(N-2m+1)) * sum_j ( xbar_{j+m,m} - xbar_{j,m} )^2

    where xbar_{j,m} is the mean of x[j : j+m] and m = tau*rate.  The two blocks
    being differenced must be separated by **m** samples, not 1: comparing
    adjacent sliding windows (separation 1) differences two means that share
    m-1 samples, which collapses the white-noise slope from -1/2 to -1 and
    under-reports the noise density by a factor of sqrt(m).  That error is
    invisible without a known-parameter test, so it is asserted in
    tests/test_probe_tools.py.
    """
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    n = x.size
    if n < 4 or rate <= 0:
        return np.zeros(0), np.zeros(0)

    dt = 1.0 / rate
    # Octave-spaced averaging factors.  Each tau needs at least a few
    # independent block pairs, so cap m at n/4.
    max_m = max(1, n // 4)
    ms = np.unique(np.floor(np.logspace(0, np.log10(max_m), 60)).astype(int))
    ms = ms[ms >= 1]

    if taus is None:
        taus = ms * dt

    # Cumulative sums give each window mean in O(1).
    c = np.concatenate(([0.0], np.cumsum(x)))
    out = np.zeros(ms.size)
    for i, m in enumerate(ms):
        if 2 * m > n:
            out[i] = np.nan
            continue
        sums = c[m:] - c[:-m]          # sums[j] = sum(x[j : j+m]), length n-m+1
        means = sums / m               # xbar_{j,m}
        d = means[m:] - means[:-m]     # separated by m: length n-2m+1
        if d.size == 0:
            out[i] = np.nan
            continue
        out[i] = np.sqrt(0.5 * np.mean(d * d))
    return taus, out


def _local_slope(taus: np.ndarray, adev: np.ndarray) -> np.ndarray:
    """Central-difference log-log slope at each tau."""
    lt, la = np.log10(taus), np.log10(adev)
    slope = np.full(taus.size, np.nan)
    slope[1:-1] = (la[2:] - la[:-2]) / (lt[2:] - lt[:-2])
    return slope


def fit_noise_model(taus: np.ndarray, adev: np.ndarray) -> Dict[str, Optional[float]]:
    """Fit the standard Allan-variance noise model by linear least squares.

    The Allan variance of the three processes that matter for an IMU is a LINEAR
    model in the unknown coefficients once written in terms of sigma^2:

        sigma^2(tau) = A * (1/tau)  +  B * tau  +  C
                       \\_________/    \\____/    \\_/
                        white noise   random    bias-instability
                                      walk      floor (constant)

    with

        N (white noise density) = sqrt(A)
        K (random walk density) = sqrt(3 * B)

    Why a model fit instead of "find the region whose local slope is -1/2":
    slope hunting is unreliable exactly where it matters.  The random-walk region
    is short and sits next to a steep transition, and at long tau each Allan
    point has few independent samples, so the local slope is noisy and any
    slope-window rule either misses a real region or invents one (both were
    observed while building this tool).  Fitting the full model uses every tau at
    once and, crucially, yields a STANDARD ERROR for each coefficient, so the
    tool can say "this random walk is not distinguishable from zero in this
    recording" instead of reporting a fitted number that is pure noise.

    Returns a dict with N, K, C, their standard errors, and the fit quality.
    A coefficient is reported as None when it is not statistically significant,
    which is the honest answer for a random walk whose crossover lies beyond the
    recording length.
    """
    finite = np.isfinite(adev) & (adev > 0) & np.isfinite(taus) & (taus > 0)
    out: Dict[str, Optional[float]] = {
        "N": None, "K": None, "C": None,
        "N_err": None, "K_err": None, "C_err": None,
        "n_pts": float(finite.sum()), "r2": None, "tau_min": None, "tau_max": None,
    }
    if finite.sum() < 4:
        return out

    t = taus[finite]
    y = (adev[finite]) ** 2
    out["tau_min"] = float(t.min())
    out["tau_max"] = float(t.max())

    # sigma^2 = A/tau + B*tau + C  ->  linear in (A, B, C)
    X = np.column_stack([1.0 / t, t, np.ones_like(t)])
    # Unweighted least squares.  The Allan points are correlated and heteroscedastic,
    # but the coefficient standard errors below are still the right order of
    # magnitude for the significance test they are used for.
    try:
        beta, residuals, rank, _ = np.linalg.lstsq(X, y, rcond=None)
    except np.linalg.LinAlgError:
        return out
    if rank < 3:
        return out

    A, B, C = (float(beta[0]), float(beta[1]), float(beta[2]))

    # Residual variance -> parameter covariance.
    dof = max(1, int(finite.sum()) - 3)
    resid = y - X @ beta
    s2 = float(np.sum(resid ** 2) / dof)
    try:
        cov = s2 * np.linalg.inv(X.T @ X)
    except np.linalg.LinAlgError:
        return out
    se = np.sqrt(np.maximum(np.diag(cov), 0.0))

    ss_tot = float(np.sum((y - y.mean()) ** 2))
    out["r2"] = float(1.0 - np.sum(resid ** 2) / ss_tot) if ss_tot > 0 else None

    # A coefficient is reported only if it is positive AND at least 2 standard
    # errors away from zero.  Below that it is indistinguishable from "absent",
    # and printing it would dress up noise as a sensor parameter.
    def significant(value: float, err: float) -> bool:
        return value > 0.0 and err > 0.0 and value / err >= 2.0

    if A > 0.0:
        out["N"] = float(np.sqrt(A))
        # d(sqrt(A))/dA = 1/(2 sqrt(A))
        out["N_err"] = float(se[0] / (2.0 * np.sqrt(A))) if A > 0 else None
    if significant(B, se[1]):
        out["K"] = float(np.sqrt(3.0 * B))
        out["K_err"] = float(se[1] * 3.0 / (2.0 * np.sqrt(3.0 * B)))
    if significant(C, se[2]):
        out["C"] = float(np.sqrt(C))
        out["C_err"] = float(se[2] / (2.0 * np.sqrt(C)))
    return out


def local_slopes(taus: np.ndarray, adev: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(tau, log-log slope) as a diagnostic of where each region lies."""
    finite = np.isfinite(adev) & (adev > 0)
    t, a = taus[finite], adev[finite]
    if t.size < 3:
        return np.zeros(0), np.zeros(0)
    lt, la = np.log10(t), np.log10(a)
    slope = (la[2:] - la[:-2]) / (lt[2:] - lt[:-2])
    return t[1:-1], slope


@dataclass
class AxisResult:
    axis: str
    white_noise: Optional[float]        # na or ng
    white_err: Optional[float]          # standard error of the above
    random_walk: Optional[float]        # nba or nbg
    rw_err: Optional[float]
    bias_instability: Optional[float]   # sqrt(C), for information
    bi_err: Optional[float]
    r2: Optional[float]
    n_taus: int
    tau_max: Optional[float]


def analyse_axis(x: np.ndarray, rate: float, axis: str) -> AxisResult:
    """Fit the Allan noise model to one axis and return the parameters."""
    taus, adev = overlapping_allan_deviation(x, rate)
    if taus.size == 0:
        return AxisResult(axis, None, None, None, None, None, None, None, 0, None)

    fit = fit_noise_model(taus, adev)
    return AxisResult(
        axis=axis,
        white_noise=fit["N"], white_err=fit["N_err"],
        random_walk=fit["K"], rw_err=fit["K_err"],
        bias_instability=fit["C"], bi_err=fit["C_err"],
        r2=fit["r2"], n_taus=int(fit["n_pts"]), tau_max=fit["tau_max"],
    )


# --------------------------------------------------------------------------
# Stationarity gate
# --------------------------------------------------------------------------

def check_stationary(s: probe_io.Samples, gyro_cols: Sequence[str],
                     accel_cols: Sequence[str], block_s: float = 1.0,
                     max_ratio: float = 6.0) -> List[str]:
    """Return a list of complaints; empty means the data is plausibly static.

    An Allan deviation assumes the process is stationary and the only inputs are
    noise.  Walking data violates that completely, so this is a hard gate rather
    than a note in the docs.

    The test is a RATIO, not a fixed threshold, because any fixed threshold is
    scale-blind: a sensor with a large noise density produces large block means
    from white noise alone, and a fixed limit would reject a perfectly good
    static recording of a noisy IMU (and, worse, accept a quiet IMU that is
    actually being carried around).

      * observed spread = std of block means over the recording (low-frequency
        content, where motion lives);
      * expected spread = (per-sample std) / sqrt(block length), i.e. what white
        noise alone would leave in a block mean.

    Motion makes the observed spread far exceed the expected one.  Both sides are
    measured from the data, so the test carries no assumption about the IMU's
    units or noise level.
    """
    problems: List[str] = []
    if s.duration() < 60.0:
        problems.append(
            f"recording is {s.duration():.1f}s; Allan variance needs minutes to hours "
            "(white-noise region needs short taus, random-walk region needs long ones)"
        )

    rate = s.median_rate()
    if rate <= 0:
        return problems + ["cannot determine the sample rate"]

    per_block = max(1, int(round(rate * block_s)))
    n_blocks = s.n // per_block
    if n_blocks < 10:
        problems.append(
            f"only {n_blocks} x {block_s:g}s blocks fit in the recording; too few to "
            "judge stationarity"
        )
        return problems

    def spread_ratio(cols: Sequence[str]) -> Tuple[float, float, float]:
        """(observed block-mean spread, expected from white noise, ratio)."""
        arr = s.samples[:, [s.fields.index(c) for c in cols]]
        # Per-sample std from first differences: immune to slow drift, so it
        # measures the white-noise content only.
        d = np.diff(arr, axis=0)
        sample_std = float(np.max(d.std(axis=0) / np.sqrt(2.0)))
        trimmed = arr[: n_blocks * per_block].reshape(n_blocks, per_block, len(cols))
        means = trimmed.mean(axis=1)
        observed = float(np.max(means.std(axis=0)))
        expected = sample_std / np.sqrt(per_block) if per_block > 0 else 0.0
        if expected <= 0.0:
            return observed, expected, 0.0
        return observed, expected, observed / expected

    g_obs, g_exp, g_ratio = spread_ratio(gyro_cols)
    if g_ratio > max_ratio:
        problems.append(
            f"gyro block-mean spread {g_obs:.4g} rad/s is {g_ratio:.1f}x what this "
            f"recording's own noise level predicts ({g_exp:.4g} rad/s): that is platform "
            "motion, not sensor noise"
        )

    a = s.samples[:, [s.fields.index(c) for c in accel_cols]]
    a_mean = a.mean(axis=0)
    a_norm = float(np.linalg.norm(a_mean))
    if not (8.0 < a_norm < 11.0):
        problems.append(
            f"mean |accel| = {a_norm:.3f} m/s^2 is not near 9.81: either the IMU is not "
            "reporting m/s^2 (set imu_acc_scale first) or the recording is not static"
        )

    a_obs, a_exp, a_ratio = spread_ratio(accel_cols)
    if a_ratio > max_ratio:
        problems.append(
            f"accel block-mean spread {a_obs:.4g} m/s^2 is {a_ratio:.1f}x what this "
            f"recording's own noise level predicts ({a_exp:.4g} m/s^2): that is motion, "
            "not sensor noise"
        )
    return problems


# --------------------------------------------------------------------------
# Plot
# --------------------------------------------------------------------------

def save_plot(path: str, curves: Dict[str, Tuple[np.ndarray, np.ndarray]]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"note: cannot plot ({exc}); continuing without a figure", file=sys.stderr)
        return
    fig, ax = plt.subplots(figsize=(9, 6))
    for label, (taus, adev) in curves.items():
        m = np.isfinite(adev) & (adev > 0)
        if m.any():
            ax.loglog(taus[m], adev[m], marker=".", lw=1, label=label)
    ax.set_xlabel("tau [s]")
    ax.set_ylabel("Allan deviation")
    ax.set_title("T2.1 IMU Allan deviation (static recording)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    print(f"plot written to {path}")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("WHY THIS EXISTS")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="ROS 2 bag directory holding a static recording")
    src.add_argument("--csv", help="CSV: time_s, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z")
    src.add_argument("--npz", help=".npz with 'times' and 'samples' (+ optional 'fields')")
    ap.add_argument("--topic", default="/imu/data", help="IMU topic when reading a bag")
    ap.add_argument("--limit", type=int, default=0, help="max messages to read (0 = all)")
    ap.add_argument("--rate", type=float, default=0.0,
                    help="override sample rate [Hz]; default = measured median rate")
    ap.add_argument("--plot", metavar="PATH", help="write an Allan deviation PNG")
    ap.add_argument("--emit-yaml", action="store_true",
                    help="print only the four YAML lines")
    ap.add_argument("--force", action="store_true",
                    help="proceed even when the stationarity gate fails (results then "
                         "describe MOTION, not sensor noise - do not put them in a config)")
    args = ap.parse_args(argv)

    try:
        s = probe_io.load(args.bag or args.csv or args.npz, topic=args.topic,
                          limit=args.limit)
    except probe_io.ProbeInputError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"source        : {s.source}")
    print(f"samples       : {s.n}")
    print(f"duration      : {s.duration():.1f} s")
    rate = args.rate if args.rate > 0 else s.median_rate()
    print(f"sample rate   : {rate:.2f} Hz" + ("" if args.rate > 0 else " (measured)"))
    print(f"channels      : {', '.join(s.fields)}")
    print()

    if rate <= 0:
        print("error: cannot determine the sample rate; pass --rate", file=sys.stderr)
        return 1

    accel_cols = [c for c in ("acc_x", "acc_y", "acc_z") if c in s.fields]
    gyro_cols = [c for c in ("gyro_x", "gyro_y", "gyro_z") if c in s.fields]
    if len(accel_cols) != 3 or len(gyro_cols) != 3:
        print("error: need acc_x/acc_y/acc_z and gyro_x/gyro_y/gyro_z channels. "
              f"Found: {', '.join(s.fields)}", file=sys.stderr)
        return 1

    problems = check_stationary(s, gyro_cols, accel_cols)
    if problems:
        print("--- STATIONARITY GATE ---")
        for p in problems:
            print(f"  ! {p}")
        if not args.force:
            print()
            print("Refusing to report noise parameters from non-static data: an Allan "
                  "deviation of")
            print("a moving recording measures the motion, not the sensor. Re-record with "
                  "the robot")
            print("powered on but STANDING STILL (see docs/calibration_procedure.md).")
            print("Use --force only to inspect the curve shape.")
            return 2
        print("  (--force given: the numbers below are NOT sensor noise parameters)")
    print()

    curves: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    results: List[AxisResult] = []

    for c in gyro_cols:
        r = analyse_axis(s.column(c), rate, c)
        results.append(r)
        curves[c] = overlapping_allan_deviation(s.column(c), rate)
    for c in accel_cols:
        r = analyse_axis(s.column(c), rate, c)
        results.append(r)
        curves[c] = overlapping_allan_deviation(s.column(c), rate)

    print("--- per-axis results ---")
    print(f"{'axis':<10}{'white N':>13}{'+-':>11}{'random walk K':>15}{'+-':>11}"
          f"{'bias inst':>12}{'R2':>8}{'tau_max':>10}")
    for r in results:
        def fmt(v: Optional[float], err: Optional[float] = None) -> Tuple[str, str]:
            if v is None:
                return "n/a", "-"
            return f"{v:.4g}", (f"{err:.2g}" if err is not None else "-")
        wn, wne = fmt(r.white_noise, r.white_err)
        rw, rwe = fmt(r.random_walk, rw_err := r.rw_err)
        bi, bie = fmt(r.bias_instability, r.bi_err)
        r2 = f"{r.r2:.4f}" if r.r2 is not None else "n/a"
        tm = f"{r.tau_max:.0f}" if r.tau_max is not None else "-"
        print(f"{r.axis:<10}{wn:>13}{wne:>11}{rw:>15}{rwe:>11}{bi:>12}{r2:>8}{tm:>10}")
    print()
    print("n/a = the coefficient is not statistically distinguishable from zero in this")
    print("      recording (fewer than 2 standard errors from 0), NOT a failure to run.")
    print("      A random walk whose crossover tau_c = sqrt(3)*N/K exceeds the recording")
    print("      length is genuinely unobservable: record longer to resolve it.")

    if args.plot:
        save_plot(args.plot, curves)

    # Aggregate: the config wants one scalar per parameter.  Take the WORST
    # (largest) axis, which is conservative for a filter: it tells the estimator
    # to trust the IMU no more than the noisiest axis warrants.
    def worst(vals: List[Optional[float]]) -> Optional[float]:
        v = [x for x in vals if x is not None]
        return max(v) if v else None

    ng = worst([r.white_noise for r in results if r.axis.startswith("gyro")])
    na = worst([r.white_noise for r in results if r.axis.startswith("acc")])
    nbg = worst([r.random_walk for r in results if r.axis.startswith("gyro")])
    nba = worst([r.random_walk for r in results if r.axis.startswith("acc")])

    print()
    print("--- deployment values (worst axis of each triple) ---")
    for label, val, unit in (("ng", ng, "rad/s/sqrt(Hz)"), ("na", na, "m/s^2/sqrt(Hz)"),
                             ("nbg", nbg, "rad/s^2/sqrt(Hz)"), ("nba", nba, "m/s^3/sqrt(Hz)")):
        print(f"  {label:<5} = {val:.6g} {unit}" if val is not None
              else f"  {label:<5} = n/a  (region not identifiable in this recording)")

    if args.emit_yaml:
        print()
        for label, val in (("na", na), ("ng", ng), ("nba", nba), ("nbg", nbg)):
            if val is not None:
                print(f"{label}: {val:.6g}")
            else:
                print(f"# {label}: NOT IDENTIFIABLE - record longer and re-run")

    print()
    print("Next: paste the four values into the deployment profile, then re-run the")
    print("static-drift check (tools/imu_static_drift.py) to confirm the filter no "
          "longer has to")
    print("absorb a scale error. See docs/calibration_procedure.md.")
    return 0 if (na is not None and ng is not None) else 2


if __name__ == "__main__":
    sys.exit(main())
