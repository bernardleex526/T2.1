# Mapping-accuracy diagnostics

Tools that produced the evidence for `.omp/reports/mapping_accuracy_diagnosis.md`.  They are
read-only with respect to the harness: they consume recorded artifacts (bags, maps, odometry,
GT) and never change a metric, a gate or a run.  Each takes explicit paths, so they can be
pointed at any run root.

| tool | what it measures | command (post-fix run root = `/tmp/sim_fixed2`) |
|---|---|---|
| `bag_imu_peaks.py` | max \|gyro\| / \|accel\| and the max per-ms pose increments of the FROZEN pre-fix bags and GT TUMs - the direct pre-fix evidence | `ROS_DOMAIN_ID=99 python3 simulation/diagnostics/bag_imu_peaks.py` |
| `decode_bag_messages.py` | decodes IMU/PointCloud2 messages of one bag with `rosbag2_py` (authoritative field offsets), printing stamps, time fields and ranges | `ROS_DOMAIN_ID=99 python3 simulation/diagnostics/decode_bag_messages.py <bag> <t_lo> <t_hi>` |
| `frame_chain_conditioning.py` | the frozen est->scene SE(3): position-only fit (as used) vs an orientation-augmented fit, i.e. how well the pre-eval window determines the chain's rotation | `python3 simulation/diagnostics/frame_chain_conditioning.py <run-dir>/mapping` |
| `residual_attribution.py` | the residual T1 error split into a rigid (gauge/chain) part and a non-rigid (within-map) part, by range and by reference-normal axis | `python3 simulation/diagnostics/residual_attribution.py <run-dir>/mapping <ref-dir>` |
| `residual_vs_analytic_surface.py` | the evaluator's PCA-normal residual vs the exact distance to the KNOWN analytic scene surfaces, per octant and per normal axis, with a planarity measure of the reference neighbourhood (answers "is the regional p99 a normal-estimation artefact or a genuine tilt?") | `python3 simulation/diagnostics/residual_vs_analytic_surface.py <run-dir>/mapping <ref-dir>` |
| `evaluator_floor_check.py` | runs the frozen evaluator with the reference cloud as its own estimate (identity matrix, sourced record) to measure the evaluator's floor | `python3 simulation/diagnostics/evaluator_floor_check.py <ref-dir> /tmp/floor.json <run-dir>/mapping/frame_chain.json` |
| `c2_3_sampling_phase.py` | whether the PointCloud2 decimation phase can matter at all - phase 0 vs 1 of `lidar_filter_num = 2` on the RENDERED scans: retained count, time origin (t0), frame span and the residual against the analytic scene surfaces | `python3 simulation/diagnostics/c2_3_sampling_phase.py` |
| `c2_3_vendor_ordering.py` | the same question on the REAL vendor bags: the index-major structure of the vendor layout (`ring == i % n`?), the per-scan time-order statistics, and the retained set / ring histogram / `t0` per phase (measured: IndoorOffice1 `/mid360/livox/lidar` has `line == i % 4` for 100 % of points, so phase 0 -> lines {0,2} vs phase 1 -> {1,3} and `t0` shifts 4.768 us; HILTI `/hesai/pandar` is not index-major, so the phase only shifts `t0` by 0.95-3.10 us) | `ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 simulation/diagnostics/c2_3_vendor_ordering.py --bag datasets/bags/IndoorOffice1 --topic /mid360/livox/lidar --time-field timestamp --time-scale 1e-9 --filter-num 2 --filter-num 3` |
| `twist_vs_exact_rays.py` | how far the PRE-FIX twist renderer was from the actual per-ray motion (the quantity the `--check` ray assertion discriminates): mean 1-5 cm, p95 3-19 cm per scan, up to 9.1 m on silhouette rays | `python3 simulation/diagnostics/twist_vs_exact_rays.py` |
| `run_seam_isolation.sh` | CONTROLLED isolation: re-runs the archived PRE-FIX bag truncated just before the arc-length wrap (glitch-free) through the unchanged stages and scores it against the pre-fix reference | `bash simulation/diagnostics/run_seam_isolation.sh` (writes `/tmp/diag/run_bag859`) |

Results produced by these tools are recorded in section 1a, 5 and 6 of the report; the run
evidence itself is persisted under `artifacts/mapping_corrected_20260930/` (with
`EVIDENCE_INDEX.json` hashes) and `artifacts/mapping_corrected_20260930/controlled_prefix_bag859/`.

Note on domains: every ROS-touching tool and every run must use its own `ROS_DOMAIN_ID` with
`ROS_LOCALHOST_ONLY=1`.  Two agents sharing a domain silently cross-contaminate (one recorder
then receives another run's `/rslidar_points`, and both `lio_node`s subscribe to the same
topics); this happened once during this work and the affected run was discarded.
