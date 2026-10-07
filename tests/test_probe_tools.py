#!/usr/bin/env python3
"""Regression tests for the T2.1 deployment probe tools (``tools/``).

Run with either:

    python3 -m pytest tests/test_probe_tools.py -v
    python3 tests/test_probe_tools.py          # no pytest needed

WHY THESE TESTS EXIST
---------------------
The probes decide configuration values that go straight onto the robot, and each
of them has a failure mode that is invisible without a known-answer test:

  * the PointCloud2 probe must say NO when the vendor SDK publishes no per-point
    time field.  RoboSense's ``rslidar_sdk`` defaults to ``POINT_TYPE=XYZI``,
    which emits x/y/z/intensity only - so the most likely real answer on the
    target is "there is no field", and a tool that always names one would send
    the operator off to configure a field that does not exist;
  * the Allan-variance estimator must recover parameters that were GENERATED
    with known values.  An Allan estimator with a wrong block-difference stride
    still produces a smooth, plausible-looking curve - it is just wrong by a
    factor of sqrt(m) - so nothing but a known-answer test catches it;
  * the gait-spectrum tool must refuse to invent notch frequencies from
    broadband noise, because filtering a frequency that is not there removes
    real motion;
  * the preflight validator must reject the specific placeholder states this
    repository's policy forbids.

The C++ side of the gait filter is covered separately by
``src/sensing/fastlio2/test/test_gait_filter.cpp``.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOOLS = os.path.join(ROOT, "tools")
FIXTURES = os.path.join(HERE, "fixtures")

sys.path.insert(0, TOOLS)

import imu_allan_variance as allan  # noqa: E402
import imu_gait_spectrum as gait  # noqa: E402
import odom_static_drift as drift  # noqa: E402
import probe_io  # noqa: E402
import rslidar_pcl2_probe as pcl2  # noqa: E402


def run_tool(script: str, *args: str):
    """Run a probe as a subprocess; return (rc, combined output)."""
    r = subprocess.run([sys.executable, os.path.join(TOOLS, script), *args],
                       capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


# ===========================================================================
# 1. PointCloud2 per-point time probe
# ===========================================================================

class TestPcl2Probe(unittest.TestCase):

    def probe(self, fixture: str, *extra: str):
        return run_tool("rslidar_pcl2_probe.py", "--fixture",
                        os.path.join(FIXTURES, fixture), *extra)

    def test_xyzi_default_has_no_time_field_and_is_refused(self):
        """The rslidar_sdk DEFAULT layout must be reported as unusable.

        This is the single most important case: XYZI is what the SDK builds
        unless the operator changed POINT_TYPE, and answering "timestamp" here
        would send them to configure a field that does not exist.
        """
        rc, out = self.probe("airy_xyzi_default.json")
        self.assertEqual(rc, 2, out)
        self.assertIn("no field is recognisable", out)

    def test_xyzirt_seconds_is_accepted_with_scale_one(self):
        rc, out = self.probe("airy_xyzirt_seconds.json", "--emit-yaml")
        self.assertEqual(rc, 0, out)
        self.assertIn('pcl2_time_field: "timestamp"', out)
        self.assertIn("pcl2_time_scale: 1", out)

    def test_xyzirt_nanoseconds_gets_nano_scale(self):
        rc, out = self.probe("airy_xyzirt_nanoseconds.json", "--emit-yaml")
        self.assertEqual(rc, 0, out)
        self.assertIn("pcl2_time_scale: 1e-09", out)

    def test_xyzirt_microseconds_gets_micro_scale(self):
        rc, out = self.probe("airy_xyzirt_microseconds.json", "--emit-yaml")
        self.assertEqual(rc, 0, out)
        self.assertIn("pcl2_time_scale: 1e-06", out)

    def test_velodyne_named_time_field_is_accepted(self):
        rc, out = self.probe("velodyne_time_f32.json", "--emit-yaml")
        self.assertEqual(rc, 0, out)
        self.assertIn('pcl2_time_field: "time"', out)

    def test_absolute_epoch_is_accepted_and_flagged(self):
        rc, out = self.probe("airy_xyzirt_epoch.json")
        self.assertEqual(rc, 0, out)
        self.assertIn("absolute clock", out)

    def test_uint32_nanoseconds_is_refused_with_a_code_change_note(self):
        """A real time field the C++ reader cannot consume must not become config.

        ``Utils::pcl2_to_PCL`` reads FLOAT32/FLOAT64 only. Emitting a config
        value here would silently disable in-scan compensation.
        """
        rc, out = self.probe("airy_xyzirt_uint32_ns.json")
        self.assertEqual(rc, 2, out)
        self.assertIn("UINT32", out)
        self.assertIn("FLOAT32/FLOAT64 only", out)

    def test_degenerate_layout_is_refused(self):
        rc, out = self.probe("degenerate_no_time.json")
        self.assertEqual(rc, 2, out)

    def test_missing_fixture_is_rc_1(self):
        rc, _ = self.probe("does_not_exist.json")
        self.assertEqual(rc, 1)

    def test_once_flag_is_accepted(self):
        """``--once`` must be accepted, because the docs use it everywhere.

        It is the conventional ROS idiom (``ros2 topic echo --once``) and the
        probe's own USAGE block, the README and five documents all print it.  A
        documented command that exits with "unrecognized arguments" is a
        documentation bug that costs the operator a bring-up cycle, so the flag
        is asserted here rather than removed from the docs.
        """
        rc, out = self.probe("airy_xyzirt_seconds.json", "--once")
        self.assertEqual(rc, 0, out)
        self.assertIn("PASS", out)

    def test_every_documented_command_line_parses(self):
        """Extract every `rslidar_pcl2_probe.py ...` invocation from the docs and
        check its flags against the tool's real argparse options.

        This catches the whole class of bug where a doc invents a flag.  It is
        deliberately a flag-level check (not a full run), so it works without
        ROS, bags or hardware.
        """
        import re
        import argparse
        import importlib.util

        spec = importlib.util.spec_from_file_location(
            "rslidar_pcl2_probe", os.path.join(TOOLS, "rslidar_pcl2_probe.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        # Collect the tool's real option strings without running main().
        parser = argparse.ArgumentParser()
        known: set = set()
        src = open(os.path.join(TOOLS, "rslidar_pcl2_probe.py")).read()
        known.update(re.findall(r'ap\.add_argument\(\s*"(--[a-z0-9-]+)"', src))
        known.update(re.findall(r'add_argument\(\s*"(--[a-z0-9-]+)"', src))
        self.assertTrue(known, "could not discover the tool's options")

        docs = [
            os.path.join(ROOT, "README.md"),
            os.path.join(ROOT, "docs", "calibration_procedure.md"),
            os.path.join(ROOT, "docs", "quadruped_adaptation.md"),
            os.path.join(ROOT, "docs", "tuning_guide.md"),
            os.path.join(ROOT, "docs", "hardware_deployment.md"),
            os.path.join(ROOT, "src", "sensing", "fastlio2", "config",
                         "TIME_SYNC_NOTES.md"),
        ]
        bad: list = []
        for path in docs:
            if not os.path.exists(path):
                continue
            for lineno, line in enumerate(open(path, encoding="utf-8"), 1):
                if "rslidar_pcl2_probe.py" not in line:
                    continue
                for flag in re.findall(r"--[a-z0-9][a-z0-9-]*", line):
                    if flag not in known:
                        bad.append(f"{os.path.relpath(path, ROOT)}:{lineno} "
                                   f"uses unknown flag {flag}")
        self.assertEqual(bad, [], "documented commands use flags the tool lacks:\n"
                                  + "\n".join(bad))

    def test_scale_decision_picks_the_closest_power_of_1000(self):
        # 0.1 s of sweep expressed in each unit must map back to 0.1 s.
        for unit_scale, expected in ((1.0, 1.0), (1e3, 1e-3), (1e6, 1e-6), (1e9, 1e-9)):
            span = 0.1 * unit_scale
            cand = pcl2.Candidate(name="timestamp", datatype="FLOAT64", is_float=True,
                                  n=100, vmin=0.0, vmax=span, span=span, median_dt=span / 99,
                                  monotonic=True, looks_like_epoch=False,
                                  absolute_magnitude=False, score=1.0, notes=[])
            pick = pcl2.decide_scale(cand, scan_period_s=0.1)
            self.assertIsNotNone(pick, f"unit scale {unit_scale}")
            self.assertAlmostEqual(pick[0], expected, places=12)

    def test_scale_decision_refuses_a_nonsensical_span(self):
        cand = pcl2.Candidate(name="timestamp", datatype="FLOAT64", is_float=True,
                              n=100, vmin=0.0, vmax=12345.0, span=12345.0,
                              median_dt=1.0, monotonic=True, looks_like_epoch=False,
                              absolute_magnitude=False, score=1.0, notes=[])
        self.assertIsNone(pcl2.decide_scale(cand, scan_period_s=0.1))

    def test_roundtrip_dump_and_load_fixture(self):
        layout = pcl2.load_fixture(os.path.join(FIXTURES, "airy_xyzirt_seconds.json"))
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "dumped.json")
            pcl2.dump_fixture(layout, p)
            again = pcl2.load_fixture(p)
        self.assertEqual(len(again.fields), len(layout.fields))
        self.assertEqual(again.point_step, layout.point_step)
        for f in layout.fields:
            self.assertIn(f.name, again.columns)
            np.testing.assert_allclose(again.columns[f.name],
                                       layout.columns[f.name][:len(again.columns[f.name])],
                                       rtol=1e-8)


# ===========================================================================
# 2. Allan variance estimator
# ===========================================================================

class TestAllanEstimator(unittest.TestCase):
    RATE = 200.0

    def test_overlapping_adev_recovers_white_noise_density(self):
        """Known-answer test for the OADEV stride.

        White noise of per-sample std s has density N = s/sqrt(rate), and
        sigma(tau) = N/sqrt(tau).  A stride bug (differencing adjacent windows
        instead of windows m apart) still yields a straight line but with slope
        -1 and a value low by sqrt(m), so this asserts the actual number.
        """
        s = 0.5
        n = 200000
        rng = np.random.default_rng(11)
        x = rng.normal(0.0, s, size=n)
        taus, adev = allan.overlapping_allan_deviation(x, self.RATE)
        self.assertGreater(taus.size, 5)
        expected = s / np.sqrt(self.RATE)
        for i in (0, 3, 8):
            got = adev[i] * np.sqrt(taus[i])
            self.assertAlmostEqual(got / expected, 1.0, delta=0.05,
                                   msg=f"tau={taus[i]}")

    def test_overlapping_adev_recovers_random_walk_density(self):
        """sigma(tau) = K*sqrt(tau/3) for an integrated random walk."""
        k_true = 2e-3
        dt = 1.0 / self.RATE
        n = 200000
        rng = np.random.default_rng(12)
        x = np.cumsum(rng.normal(0.0, k_true * np.sqrt(dt), size=n))
        taus, adev = allan.overlapping_allan_deviation(x, self.RATE)
        # Sample the region where the walk dominates and tau << n*dt.
        for target in (10.0, 100.0):
            i = int(np.argmin(np.abs(taus - target)))
            theory = k_true * np.sqrt(taus[i] / 3.0)
            self.assertAlmostEqual(adev[i] / theory, 1.0, delta=0.15,
                                   msg=f"tau={taus[i]}")

    def test_model_fit_recovers_both_processes(self):
        """A combined white+walk signal must yield both coefficients."""
        na, nba = 0.20, 2.0e-3
        dt = 1.0 / self.RATE
        n = int(self.RATE * 3600 * 2)
        rng = np.random.default_rng(13)
        x = (rng.normal(0.0, na * np.sqrt(self.RATE), size=n)
             + np.cumsum(rng.normal(0.0, nba * np.sqrt(dt), size=n)))
        taus, adev = allan.overlapping_allan_deviation(x, self.RATE)
        fit = allan.fit_noise_model(taus, adev)
        self.assertIsNotNone(fit["N"])
        self.assertIsNotNone(fit["K"])
        self.assertAlmostEqual(fit["N"] / na, 1.0, delta=0.15)
        self.assertAlmostEqual(fit["K"] / nba, 1.0, delta=0.5)

    def test_unobservable_random_walk_is_reported_as_none(self):
        """A walk whose crossover exceeds the recording must NOT be fabricated.

        With ng=0.005 and nbg=3e-6 the crossover is sqrt(3)*ng/nbg = 2887 s, so a
        2 h recording cannot resolve it.  The estimator must return None rather
        than a fitted number that is pure noise - the distinction matters because
        the value goes into a config file.
        """
        ng, nbg = 0.005, 3.0e-6
        dt = 1.0 / self.RATE
        n = int(self.RATE * 3600 * 2)
        rng = np.random.default_rng(14)
        x = (rng.normal(0.0, ng * np.sqrt(self.RATE), size=n)
             + np.cumsum(rng.normal(0.0, nbg * np.sqrt(dt), size=n)))
        taus, adev = allan.overlapping_allan_deviation(x, self.RATE)
        fit = allan.fit_noise_model(taus, adev)
        self.assertAlmostEqual(fit["N"] / ng, 1.0, delta=0.15)
        self.assertIsNone(fit["K"],
                          "an unresolvable random walk must be reported as None")

    def test_stationarity_gate_rejects_motion(self):
        rate, n = 200.0, int(200 * 60)
        t = np.arange(n) / rate
        rng = np.random.default_rng(15)
        acc = np.column_stack([1.5 * np.sin(2 * np.pi * 0.5 * t),
                               rng.normal(0, 0.05, n),
                               9.81 + rng.normal(0, 0.05, n)])
        gyro = np.column_stack([rng.normal(0, 0.01, n),
                                0.35 * np.sin(2 * np.pi * 0.5 * t),
                                rng.normal(0, 0.01, n)])
        s = probe_io.Samples(times=t, samples=np.column_stack([acc, gyro]),
                             fields=["acc_x", "acc_y", "acc_z",
                                     "gyro_x", "gyro_y", "gyro_z"], source="synthetic")
        problems = allan.check_stationary(s, ["gyro_x", "gyro_y", "gyro_z"],
                                          ["acc_x", "acc_y", "acc_z"])
        self.assertTrue(problems, "motion must be reported")

    def test_stationarity_gate_accepts_pure_noise(self):
        """The gate must be scale-aware, not a fixed threshold.

        A fixed limit rejects a long static recording of a NOISY IMU, because
        white noise alone produces large block means.
        """
        rate, n = 200.0, int(200 * 300)
        t = np.arange(n) / rate
        rng = np.random.default_rng(16)
        acc = np.column_stack([rng.normal(0, 0.2, n), rng.normal(0, 0.2, n),
                               9.81 + rng.normal(0, 0.2, n)])
        gyro = rng.normal(0, 0.02, size=(n, 3))
        s = probe_io.Samples(times=t, samples=np.column_stack([acc, gyro]),
                             fields=["acc_x", "acc_y", "acc_z",
                                     "gyro_x", "gyro_y", "gyro_z"], source="synthetic")
        problems = allan.check_stationary(s, ["gyro_x", "gyro_y", "gyro_z"],
                                          ["acc_x", "acc_y", "acc_z"])
        self.assertEqual(problems, [], f"static noise rejected: {problems}")


# ===========================================================================
# 3. Gait spectrum
# ===========================================================================

class TestGaitSpectrum(unittest.TestCase):
    RATE = 200.0
    DUR = 60.0

    def synth_walk(self, f0, harmonics=True, seed=21):
        n = int(self.RATE * self.DUR)
        t = np.arange(n) / self.RATE
        rng = np.random.default_rng(seed)
        gx = 0.25 * np.sin(2 * np.pi * f0 * t)
        gy = 0.18 * np.sin(2 * np.pi * f0 * t + 0.4)
        if harmonics:
            gx += 0.09 * np.sin(2 * np.pi * 2 * f0 * t)
            gx += 0.04 * np.sin(2 * np.pi * 3 * f0 * t)
        acc = np.column_stack([0.9 * np.sin(2 * np.pi * f0 * t),
                               0.6 * np.sin(2 * np.pi * f0 * t + 0.3),
                               9.81 + 0.4 * np.sin(2 * np.pi * 2 * f0 * t)])
        gyro = np.column_stack([gx, gy, 0.02 * np.sin(2 * np.pi * 0.7 * t)])
        return t, acc + rng.normal(0, 0.05, acc.shape), gyro + rng.normal(0, 0.01, gyro.shape)

    def test_welch_psd_finds_a_known_tone(self):
        n = int(self.RATE * 30)
        t = np.arange(n) / self.RATE
        f0 = 17.0
        x = np.sin(2 * np.pi * f0 * t)
        f, p = gait.welch_psd(x, self.RATE)
        self.assertGreater(f.size, 10)
        peak = f[int(np.argmax(p))]
        self.assertAlmostEqual(peak, f0, delta=0.5)

    def test_harmonic_score_identifies_a_series(self):
        peaks = [gait.Peak(freq=f, power=1.0, prominence=10.0, width_hz=0.5)
                 for f in (12.0, 24.0, 36.0, 48.0, 7.0)]
        n_harm, found = gait.harmonic_score(peaks, 12.0)
        self.assertEqual(n_harm, 3)
        self.assertEqual(len(found), 3)

    def test_harmonic_score_rejects_a_lone_peak(self):
        peaks = [gait.Peak(freq=f, power=1.0, prominence=10.0, width_hz=0.5)
                 for f in (12.0, 7.3, 41.9)]
        n_harm, _ = gait.harmonic_score(peaks, 12.0)
        self.assertEqual(n_harm, 0)

    def _write_csv(self, path, t, acc, gyro):
        arr = np.column_stack([t, acc, gyro])
        np.savetxt(path, arr, delimiter=",", fmt="%.9g", comments="",
                   header="time,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z")

    def test_known_gait_frequency_is_recovered(self):
        for f0 in (12.0, 20.0):
            with tempfile.TemporaryDirectory() as td:
                p = os.path.join(td, "walk.csv")
                t, acc, gyro = self.synth_walk(f0)
                self._write_csv(p, t, acc, gyro)
                rc, out = run_tool("imu_gait_spectrum.py", "--csv", p,
                                   "--fmax", str(3.5 * f0), "--emit-yaml")
            self.assertEqual(rc, 0, out)
            line = [l for l in out.splitlines()
                    if l.startswith("gait_notch_freq_hz")][0]
            vals = [float(v) for v in line.split("[")[1].split("]")[0].split(",")]
            self.assertTrue(any(abs(v - f0) <= 0.35 * f0 for v in vals),
                            f"f0={f0} not in {vals}")

    def test_broadband_noise_yields_no_notch_list(self):
        """Refusing here is the whole point: a notch removes real motion."""
        n = int(self.RATE * self.DUR)
        rng = np.random.default_rng(22)
        t = np.arange(n) / self.RATE
        acc = np.column_stack([rng.normal(0, 0.5, n), rng.normal(0, 0.5, n),
                               9.81 + rng.normal(0, 0.5, n)])
        gyro = rng.normal(0, 0.05, size=(n, 3))
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "noise.csv")
            self._write_csv(p, t, acc, gyro)
            rc, out = run_tool("imu_gait_spectrum.py", "--csv", p, "--emit-yaml")
        self.assertEqual(rc, 2, out)
        self.assertNotIn("gait_notch_freq_hz:", out)


# ===========================================================================
# 4. Static drift
# ===========================================================================

class TestStaticDrift(unittest.TestCase):

    def _tum(self, path, times, xyz, yaw=None):
        with open(path, "w") as fh:
            for i, t in enumerate(times):
                if yaw is None:
                    q = (0.0, 0.0, 0.0, 1.0)
                else:
                    q = (0.0, 0.0, float(np.sin(yaw[i] / 2)), float(np.cos(yaw[i] / 2)))
                fh.write(f"{t:.6f} {xyz[i,0]:.9f} {xyz[i,1]:.9f} {xyz[i,2]:.9f} "
                         f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")

    def setUp(self):
        self.td = tempfile.TemporaryDirectory()
        self.n = 3000
        self.rate = 10.0
        self.times = np.arange(self.n) / self.rate

    def tearDown(self):
        self.td.cleanup()

    def _run(self, path, *extra):
        return run_tool("odom_static_drift.py", "--tum", path, *extra)

    def test_static_pose_passes(self):
        p = os.path.join(self.td.name, "static.tum")
        self._tum(p, self.times, np.zeros((self.n, 3)))
        rc, out = self._run(p)
        self.assertEqual(rc, 0, out)

    def test_drift_above_budget_fails(self):
        p = os.path.join(self.td.name, "drift.tum")
        xyz = np.column_stack([np.linspace(0, 0.5, self.n), np.zeros(self.n),
                               np.zeros(self.n)])
        self._tum(p, self.times, xyz)
        rc, out = self._run(p)
        self.assertEqual(rc, 2, out)

    def test_single_jump_blames_the_clock(self):
        p = os.path.join(self.td.name, "jump.tum")
        xyz = np.zeros((self.n, 3))
        xyz[self.n // 2:, 0] = 0.30
        self._tum(p, self.times, xyz)
        rc, out = self._run(p)
        self.assertEqual(rc, 2, out)
        self.assertIn("CLOCK", out)

    def test_yaw_only_drift_is_a_failure(self):
        """Position can be perfect while the heading rotates.

        That case used to PASS, which is the dangerous outcome: the map looks
        correct while the published heading slowly turns, silently breaking the
        localizer's initial guess.
        """
        p = os.path.join(self.td.name, "yaw.tum")
        self._tum(p, self.times, np.zeros((self.n, 3)),
                  yaw=np.linspace(0, np.radians(10.0), self.n))
        rc, out = self._run(p)
        self.assertEqual(rc, 2, out)
        self.assertIn("yaw drift", out)
        self.assertIn("gyro", out)

    def test_small_yaw_drift_passes(self):
        p = os.path.join(self.td.name, "yaw_ok.tum")
        self._tum(p, self.times, np.zeros((self.n, 3)),
                  yaw=np.linspace(0, np.radians(0.2), self.n))
        rc, out = self._run(p)
        self.assertEqual(rc, 0, out)

    def test_short_recording_is_flagged(self):
        p = os.path.join(self.td.name, "short.tum")
        self._tum(p, np.arange(200) / self.rate, np.zeros((200, 3)))
        rc, out = self._run(p)
        self.assertEqual(rc, 0, out)
        self.assertIn("10 min", out)

    def test_missing_file_is_rc_1(self):
        rc, _ = self._run(os.path.join(self.td.name, "absent.tum"))
        self.assertEqual(rc, 1)

    def test_quaternion_yaw_conversion(self):
        for deg in (0.0, 30.0, -45.0, 179.0):
            rad = np.radians(deg)
            q = (0.0, 0.0, np.sin(rad / 2), np.cos(rad / 2))
            self.assertAlmostEqual(np.degrees(drift._quat_yaw(q)), deg, places=6)


# ===========================================================================
# 5. Preflight config validator
# ===========================================================================

class TestPreflight(unittest.TestCase):
    GOOD = os.path.join(FIXTURES, "calibrated_profile_example.yaml")

    def _run(self, path, *extra):
        return run_tool("preflight_config.py", "--config", path, *extra)

    def _mutated(self, **changes):
        import yaml
        with open(self.GOOD) as fh:
            cfg = yaml.safe_load(fh)
        for k, v in changes.items():
            if v is None:
                cfg.pop(k, None)
            else:
                cfg[k] = v
        fd, p = tempfile.mkstemp(suffix=".yaml")
        os.close(fd)
        with open(p, "w") as fh:
            yaml.safe_dump(cfg, fh)
        self.addCleanup(os.unlink, p)
        return p

    def test_calibrated_example_passes(self):
        rc, out = self._run(self.GOOD)
        self.assertEqual(rc, 0, out)

    def test_shipped_profile_is_not_ready(self):
        """The repository's own lio_orin_nx.yaml must fail preflight.

        It is documented as an unvalidated starting point with no extrinsic and
        no time field; a validator that called it ready would be worse than none.
        """
        shipped = os.path.join(ROOT, "src", "sensing", "fastlio2", "config",
                               "lio_orin_nx.yaml")
        rc, out = self._run(shipped)
        self.assertEqual(rc, 2, out)
        self.assertIn("ext_il", out)

    def test_missing_extrinsic_is_blocking(self):
        rc, out = self._run(self._mutated(ext_il=None))
        self.assertEqual(rc, 2, out)

    def test_wrong_extrinsic_length_is_blocking(self):
        rc, out = self._run(self._mutated(ext_il=[0, 0, 0, 0, 0, 0]))
        self.assertEqual(rc, 2, out)
        self.assertIn("7-element", out)

    def test_empty_time_field_is_blocking_without_the_flag(self):
        p = self._mutated(pcl2_time_field="")
        rc, out = self._run(p)
        self.assertEqual(rc, 2, out)
        rc2, _ = self._run(p, "--allow-no-time-field")
        self.assertEqual(rc2, 0)

    def test_upstream_group_name_is_blocking(self):
        p = self._mutated()
        import yaml
        with open(p) as fh:
            cfg = yaml.safe_load(fh)
        cfg["mapping"] = {"filter_size_surf": 0.3}
        with open(p, "w") as fh:
            yaml.safe_dump(cfg, fh)
        rc, out = self._run(p)
        self.assertEqual(rc, 2, out)
        self.assertIn("GROUP", out)

    def test_unknown_key_warns_and_strict_fails(self):
        p = self._mutated(not_a_real_key=1)
        rc, out = self._run(p)
        self.assertEqual(rc, 0, out)
        self.assertIn("ignored", out)
        rc2, _ = self._run(p, "--strict")
        self.assertEqual(rc2, 2)

    def test_missing_imu_acc_scale_is_blocking(self):
        rc, out = self._run(self._mutated(imu_acc_scale=None))
        self.assertEqual(rc, 2, out)
        self.assertIn("imu_acc_scale", out)

    def test_gait_enabled_without_frequencies_is_blocking(self):
        rc, out = self._run(self._mutated(gait_notch_freq_hz=[]))
        self.assertEqual(rc, 2, out)

    def test_gait_frequency_above_nyquist_is_blocking(self):
        rc, out = self._run(self._mutated(gait_notch_freq_hz=[18.5, 150.0]))
        self.assertEqual(rc, 2, out)
        self.assertIn("Nyquist", out)

    def test_non_positive_noise_parameters_are_blocking(self):
        for key in ("na", "ng", "nba", "nbg"):
            rc, _ = self._run(self._mutated(**{key: 0.0}))
            self.assertEqual(rc, 2, f"{key}=0 should be blocking")

    def test_placeholder_noise_values_warn(self):
        rc, out = self._run(self._mutated(na=0.1, ng=0.01, nba=0.0001, nbg=0.0001))
        self.assertEqual(rc, 0, out)
        self.assertIn("placeholder", out)

    def test_missing_config_is_rc_1(self):
        rc, _ = self._run(os.path.join(FIXTURES, "nope.yaml"))
        self.assertEqual(rc, 1)

    def test_shipped_config_and_example_share_expected_keys(self):
        """The example profile must not drift away from what the node reads."""
        import yaml
        with open(self.GOOD) as fh:
            example = yaml.safe_load(fh)
        unknown = [k for k in example if k not in TestPreflight._known_keys()]
        self.assertEqual(unknown, [], f"example profile has unread keys: {unknown}")

    @staticmethod
    def _known_keys():
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "preflight_config", os.path.join(TOOLS, "preflight_config.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.KNOWN_KEYS


# ===========================================================================
# 6. Input layer
# ===========================================================================

class TestProbeIo(unittest.TestCase):
    def test_npz_roundtrip(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "a.npz")
            times = np.arange(10) / 100.0
            samples = np.arange(20).reshape(10, 2).astype(float)
            np.savez(p, times=times, samples=samples, fields=np.array(["a", "b"]))
            s = probe_io.load(p)
        self.assertEqual(s.n, 10)
        self.assertEqual(s.fields, ["a", "b"])
        self.assertAlmostEqual(s.median_rate(), 100.0, places=6)

    def test_csv_with_header(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "a.csv")
            with open(p, "w") as fh:
                fh.write("time,acc_x,gyro_x\n")
                for i in range(5):
                    fh.write(f"{i/100.0},1.5,0.25\n")
            s = probe_io.load(p)
        self.assertEqual(s.n, 5)
        self.assertEqual(s.fields, ["acc_x", "gyro_x"])
        self.assertTrue(np.allclose(s.column("acc_x"), 1.5))

    def test_channel_count_mismatch_is_an_error(self):
        with self.assertRaises(probe_io.ProbeInputError):
            probe_io.Samples(times=np.zeros(3), samples=np.zeros((4, 2)), source="x")

    def test_missing_channel_is_an_error(self):
        s = probe_io.Samples(times=np.arange(3), samples=np.zeros((3, 2)),
                             fields=["a", "b"], source="x")
        with self.assertRaises(probe_io.ProbeInputError):
            s.column("nope")

    def test_missing_input_is_an_error(self):
        with self.assertRaises(probe_io.ProbeInputError):
            probe_io.load("definitely/not/here.npz")

    def test_bag_without_topic_is_an_error(self):
        with tempfile.TemporaryDirectory() as td:
            with self.assertRaises(probe_io.ProbeInputError):
                probe_io.load(td)


# ===========================================================================
# 7. Documentation integrity
# ===========================================================================

class TestDocs(unittest.TestCase):
    """The docs are part of the deliverable, so they are checked like code.

    A command printed in a runbook that does not run costs the operator a
    bring-up cycle, and a link to a file that does not exist sends them nowhere.
    """

    def test_doc_validation_passes(self):
        r = subprocess.run([sys.executable, os.path.join(HERE, "validate_docs.py")],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0,
                         "doc validation failed:\n" + r.stdout + r.stderr)

    def test_required_docs_exist(self):
        for name in ("hardware_deployment.md", "tuning_guide.md",
                     "calibration_procedure.md", "test_scenarios.md",
                     "quadruped_adaptation.md"):
            p = os.path.join(ROOT, "docs", name)
            self.assertTrue(os.path.exists(p), f"missing deliverable doc: {name}")
            self.assertGreater(os.path.getsize(p), 2000,
                               f"{name} looks too small to be a real document")

    def test_readme_indexes_the_new_docs(self):
        readme = open(os.path.join(ROOT, "README.md"), encoding="utf-8").read()
        for name in ("hardware_deployment.md", "tuning_guide.md",
                     "calibration_procedure.md", "test_scenarios.md",
                     "quadruped_adaptation.md"):
            self.assertIn(name, readme, f"README does not link {name}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
