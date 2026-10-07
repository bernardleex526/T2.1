#!/usr/bin/env python3
"""Find the quadruped gait frequencies to put in ``gait_notch_freq_hz``.

WHY THIS EXISTS
---------------
The gait notch filter is inert until someone supplies centre frequencies, and the
review's own example (``notch_freq_hz: [15.0, 20.0, 25.0]``) is an illustration,
not a measurement.  Filtering the wrong frequency is worse than not filtering:
it removes real motion and leaves the gait oscillation in place.  So the centres
must come from a spectrum of the actual robot walking.

WHAT IT DOES
------------
Welch PSD of the gyro (and optionally accel) channels over a walking recording,
then reports the dominant peaks with the harmonic structure that identifies a
gait fundamental:

  * peaks are ranked by prominence above the local noise floor;
  * for each candidate fundamental the tool checks whether its 2nd/3rd harmonics
    are also present - a real gait shows a harmonic series, a one-off vibration
    does not;
  * it prints the ready-to-paste ``gait_notch_freq_hz`` list and the matching
    ``gait_notch_q`` for the measured peak widths.

USAGE
-----
    python3 tools/imu_gait_spectrum.py --bag walk_imu --topic /imu/data
    python3 tools/imu_gait_spectrum.py --csv walk.csv --plot spectrum.png --emit-yaml

Record a straight walk at the robot's normal speed for at least ~30 s.  The tool
reports the frequency resolution it achieved, so a peak is never quoted finer
than the data supports.
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


@dataclass
class Peak:
    freq: float
    power: float
    prominence: float          # power / local median floor
    width_hz: float            # -3 dB width estimate


def welch_psd(x: np.ndarray, rate: float, nperseg: int = 0,
              overlap: float = 0.5) -> Tuple[np.ndarray, np.ndarray]:
    """Welch power spectral density, Hann window, mean-detrended segments.

    Implemented directly rather than via scipy so the tool has no dependency
    beyond numpy (it must run on the Orin NX image as shipped).
    """
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    n = x.size
    if n < 16 or rate <= 0:
        return np.zeros(0), np.zeros(0)

    if nperseg <= 0:
        # Aim for ~2 s segments: long enough for a few Hz of resolution, short
        # enough that the walking speed does not drift across a segment.
        nperseg = int(min(n, max(64, round(rate * 2.0))))
    step = max(1, int(nperseg * (1.0 - overlap)))
    win = np.hanning(nperseg)
    win_norm = float(np.sum(win ** 2))

    segs = []
    for start in range(0, n - nperseg + 1, step):
        seg = x[start:start + nperseg]
        seg = seg - seg.mean()
        segs.append(seg * win)
    if not segs:
        return np.zeros(0), np.zeros(0)

    arr = np.asarray(segs)
    spec = np.fft.rfft(arr, axis=1)
    psd = (np.abs(spec) ** 2).mean(axis=0) / (rate * win_norm)
    psd[1:-1] *= 2.0  # one-sided
    freqs = np.fft.rfftfreq(nperseg, d=1.0 / rate)
    return freqs, psd


def find_peaks(freqs: np.ndarray, psd: np.ndarray, f_min: float, f_max: float,
               max_peaks: int = 10, min_prominence: float = 3.0) -> List[Peak]:
    """Local maxima above a smoothed noise floor, within [f_min, f_max]."""
    m = (freqs >= f_min) & (freqs <= f_max)
    if m.sum() < 5:
        return []
    f, p = freqs[m], psd[m]

    # Local maxima.
    cand = [i for i in range(1, len(p) - 1) if p[i] > p[i - 1] and p[i] >= p[i + 1]]
    if not cand:
        return []

    # Noise floor: median of the band.  Median is robust to the very peaks we are
    # looking for, unlike a mean.
    floor = float(np.median(p))
    if floor <= 0:
        return []

    out: List[Peak] = []
    for i in cand:
        prom = p[i] / floor
        if prom < min_prominence:
            continue
        # -3 dB width around the peak (half power).
        half = p[i] / 2.0
        lo = i
        while lo > 0 and p[lo] > half:
            lo -= 1
        hi = i
        while hi < len(p) - 1 and p[hi] > half:
            hi += 1
        width = float(max(f[hi] - f[lo], f[1] - f[0]))
        out.append(Peak(freq=float(f[i]), power=float(p[i]), prominence=float(prom),
                        width_hz=width))

    out.sort(key=lambda k: -k.power)
    return out[:max_peaks]


def harmonic_score(peaks: List[Peak], f0: float, tol_rel: float = 0.06) -> Tuple[int, List[float]]:
    """How many of the 2nd..4th harmonics of f0 are present among ``peaks``.

    A gait oscillation is periodic, so it produces a harmonic series.  A single
    structural resonance usually does not.  This is the test that separates the
    two, and it is why the tool does not simply report the tallest peak.
    """
    found: List[float] = []
    for k in (2, 3, 4):
        target = k * f0
        for pk in peaks:
            if abs(pk.freq - target) <= tol_rel * target:
                found.append(pk.freq)
                break
    return len(found), found


def save_plot(path: str, curves: Dict[str, Tuple[np.ndarray, np.ndarray]],
              f_min: float, f_max: float) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"note: cannot plot ({exc}); continuing without a figure", file=sys.stderr)
        return
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for label, (f, p) in curves.items():
        m = (f >= f_min) & (f <= f_max)
        if m.any():
            ax.semilogy(f[m], p[m], lw=0.9, label=label)
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("PSD")
    ax.set_title("T2.1 quadruped gait spectrum (walking recording)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    print(f"plot written to {path}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("WHY THIS EXISTS")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="ROS 2 bag directory with a walking recording")
    src.add_argument("--csv", help="CSV: time_s, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z")
    src.add_argument("--npz", help=".npz with 'times' and 'samples'")
    ap.add_argument("--topic", default="/imu/data", help="IMU topic when reading a bag")
    ap.add_argument("--limit", type=int, default=0, help="max messages (0 = all)")
    ap.add_argument("--rate", type=float, default=0.0, help="override sample rate [Hz]")
    ap.add_argument("--fmin", type=float, default=3.0, help="band start [Hz]")
    ap.add_argument("--fmax", type=float, default=60.0, help="band end [Hz]")
    ap.add_argument("--prominence", type=float, default=4.0,
                    help="peak/noise-floor ratio to report (default 4)")
    ap.add_argument("--plot", metavar="PATH", help="write a PSD PNG")
    ap.add_argument("--emit-yaml", action="store_true",
                    help="print only the gait_notch_* YAML lines")
    args = ap.parse_args(argv)

    try:
        s = probe_io.load(args.bag or args.csv or args.npz, topic=args.topic,
                          limit=args.limit)
    except probe_io.ProbeInputError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    rate = args.rate if args.rate > 0 else s.median_rate()
    print(f"source      : {s.source}")
    print(f"samples     : {s.n}   duration: {s.duration():.1f} s")
    print(f"sample rate : {rate:.2f} Hz")
    if rate <= 0:
        print("error: cannot determine the sample rate; pass --rate", file=sys.stderr)
        return 1

    nperseg = int(min(s.n, max(64, round(rate * 2.0))))
    resolution = rate / nperseg
    print(f"resolution  : {resolution:.3f} Hz (segment {nperseg / rate:.2f} s); "
          f"Nyquist {rate / 2:.1f} Hz")
    if args.fmax > rate / 2:
        print(f"note: --fmax {args.fmax:g} Hz exceeds Nyquist; clamping to {rate / 2:g} Hz")
        args.fmax = rate / 2.0
    print()

    gyro_cols = [c for c in ("gyro_x", "gyro_y", "gyro_z") if c in s.fields]
    accel_cols = [c for c in ("acc_x", "acc_y", "acc_z") if c in s.fields]
    if not gyro_cols:
        print("error: no gyro_x/gyro_y/gyro_z channels found. "
              f"Have: {', '.join(s.fields)}", file=sys.stderr)
        return 1

    curves: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    all_peaks: List[Peak] = []
    for c in gyro_cols + accel_cols:
        f, p = welch_psd(s.column(c), rate, nperseg=nperseg)
        if f.size == 0:
            continue
        curves[c] = (f, p)
        for pk in find_peaks(f, p, args.fmin, args.fmax, min_prominence=args.prominence):
            all_peaks.append(pk)

    if not all_peaks:
        print(f"No peak exceeded {args.prominence:g}x the noise floor in "
              f"[{args.fmin:g}, {args.fmax:g}] Hz.")
        print("That is a real result: either the recording is not a walk, or the gait")
        print("content is below the band. Do NOT invent notch frequencies - leaving")
        print("gait_filter_enable false is the correct outcome here.")
        return 2

    # Merge peaks that the different channels put at the same frequency.
    merged: List[Peak] = []
    for pk in sorted(all_peaks, key=lambda k: k.freq):
        if merged and abs(pk.freq - merged[-1].freq) <= max(resolution, 0.1 * pk.freq):
            if pk.power > merged[-1].power:
                merged[-1] = pk
        else:
            merged.append(pk)

    print(f"--- peaks above {args.prominence:g}x the local noise floor ---")
    print(f"{'freq [Hz]':>11}{'prominence':>12}{'width [Hz]':>12}{'implied Q':>11}")
    for pk in merged:
        q = pk.freq / pk.width_hz if pk.width_hz > 0 else float("nan")
        print(f"{pk.freq:>11.2f}{pk.prominence:>12.1f}{pk.width_hz:>12.2f}{q:>11.1f}")
    print()

    # Identify the gait fundamental as the lowest strong peak that has harmonics.
    print("--- harmonic structure (a gait is periodic; a lone resonance is not) ---")
    best_f0: Optional[Peak] = None
    best_harm = -1
    for pk in merged:
        n_harm, found = harmonic_score(merged, pk.freq)
        marker = " <-- harmonic series" if n_harm >= 2 else ""
        print(f"  f0={pk.freq:7.2f} Hz: {n_harm}/3 harmonics present "
              f"({', '.join(f'{x:.2f}' for x in found) or 'none'}){marker}")
        if n_harm > best_harm or (n_harm == best_harm and best_f0 is not None
                                  and pk.freq < best_f0.freq):
            best_harm, best_f0 = n_harm, pk
    print()

    if best_f0 is None or best_harm < 2:
        print("No peak shows a convincing harmonic series (>= 2 of the 2nd/3rd/4th")
        print("harmonics). This does not look like a periodic gait. Consider leaving")
        print("gait_filter_enable false rather than notching a one-off resonance.")
        return 2

    print(f"--- suggested configuration (fundamental {best_f0.freq:.2f} Hz) ---")
    centres = [best_f0.freq]
    for k in (2, 3):
        target = k * best_f0.freq
        for pk in merged:
            if abs(pk.freq - target) <= 0.06 * target and pk.freq < 0.95 * rate / 2:
                centres.append(pk.freq)
                break
    # Q from the measured peak width: Q = f0 / bandwidth.
    qs = [c / best_f0.width_hz for c in centres if best_f0.width_hz > 0]
    q = float(np.median(qs)) if qs else 10.0
    q = float(min(max(q, 1.0), 100.0))

    print(f"  measured peak width at the fundamental: {best_f0.width_hz:.2f} Hz")
    print(f"  implied Q = f0/width = {q:.1f}")
    print()
    print("--- paste into the deployment profile ---")
    print("gait_filter_enable: true")
    print("gait_notch_freq_hz: [" + ", ".join(f"{c:.2f}" for c in centres) + "]")
    print(f"gait_notch_q: {q:.1f}")
    print(f"gait_filter_sample_rate_hz: {rate:.1f}")
    print()
    print("IMPORTANT: a notch removes energy at these frequencies whether or not the")
    print("gait is really there. After enabling, verify the map/odometry did not get")
    print("worse (see docs/tuning_guide.md) - the review's claim that this improves")
    print("accuracy is untested on this repository's data.")

    if args.plot:
        save_plot(args.plot, curves, args.fmin, args.fmax)
    return 0


if __name__ == "__main__":
    sys.exit(main())
