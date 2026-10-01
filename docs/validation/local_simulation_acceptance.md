# Local-simulation acceptance report — WP2 T2.1 (no-hardware deliverables)

> **Scope of this document.** This is the *final acceptance report for the work that can be
> verified without hardware*. It reports measured results from the in-repo synthetic harness and
> from controlled offline interventions, the ROS 2 package/config/TF/time/extrinsics contract, the
> T1.2/T1.4 interface contracts, the IMU/deskew/debug parameters, and the FastLIVO2 interface.
> It contains **no hardware, Orin/Jetson, calibration or real-robot claim**: every hardware item is
> `【board-only】`. It is **not** the historical dataset-level acceptance verdict (MCD/HILTI on
> survey maps), which stays `BLOCKED` for its own reasons — see §2 and §14.
>
> **Headline (final).** In the local simulation: map accuracy **PASS** (T1 0.03145 m ≤ 0.05 m) and
> mapping latency **PASS** (T2 observed p95 0.0486 s < one 0.1 s frame period, host-only). For
> localization, the real `/robot_pose_map` consumer **meets the published-stamp criterion** (default
> 0.0191 m; opt-in strict mode 0.0223 m after the stamp fix; 0.1263 m before it), but the
> **current-clock requirement FAILS**: default mode **0.1625 m**, and the opt-in strict
> `current_pose_mode:=true` **0.0591 m > 5 cm** in the final run (its earlier 0.0457 m is
> **superseded** — that run predates the two review P1 fixes). The added 95 %/0.1 s availability
> protocol also **FAILs** (availability 0.0000 over the full 60 001-query span; min achievable age
> ~0.2 s). **The requirements are not all met.** See §2 and §14.
>
> - Baseline: `fork` @ `04b73f553918b1dcc751feac69c0a2834f194432` (`main`); working tree dirty with
>   the simulation/harness and localizer fixes described below (recorded per run in
>   `run_manifest.json` / `git_dirty`).
> - Evidence grading used throughout: `[measured]` (artifact or command output readable on disk),
>   `[inference]`, `[conditional]` (depends on an unverified assumption), `[board-only]`,
>   `[blocked]`. **Never** read an `[inference]`/`[conditional]` value as `[measured]`.
> - Machine-readable manifest of every deliverable cited here, with sha256:
>   `fork/artifacts/nonhardware_delivery/manifest.json`.
>   **Hash policy:** the source is **frozen** and every recorded sha256 was recomputed and verified
>   against the file on disk at freeze time; the report's hash is bound one-way in the manifest.
> - Companion reports (working evidence, not deliverables):
>   `.omp/reports/mapping_accuracy_diagnosis.md`,
>   `.omp/reports/localization_coverage_diagnosis.md`,
>   `.omp/reports/simulation_integration.md`.
> - `docs/validation/04b73f5_*.md` are the **prior drafts** (interface, params, P3 deployment,
>   three-replay, speed/accuracy, localizer regression, mapping diagnosis); this document
>   **supersedes their status statements** and keeps their source-level facts where they still hold.

---

## Executive summary

**Scope**: everything verifiable without hardware, in the local simulation. No calibration,
Orin/Jetson, real-robot or real-time claim anywhere in this document.

| # | Requirement | Verdict | Key measured number | Detail |
|---|---|---|---|---|
| 1 | Map accuracy ≤ 5 cm (T1, synthetic scene) | **PASS** | raw p2pl **0.03145 m**; all admissibility preconditions true | §4 |
| 2 | Mapping latency (T2) | **PASS** (host) | observed input-ready→output-ready p95 **0.0486 s** < one 0.1 s frame | §5 |
| 3 | Localization pose accuracy **at its own published stamp** (T3, real `/robot_pose_map`) | **PASS** | **0.0191 m** (n = 1320); 0.1263 m before the stamp fix | §6.1–6.2 |
| 4 | Localization accuracy **at the consumer's current clock** (T3) — **NOT MET** | **FAIL** | default mode **0.1625 m**; opt-in `current_pose_mode:=true` (review-passed) **0.0591 m** current-clock (**> 5 cm**) at published-stamp 0.0223 m (n = 1082); publishes 1013/1459 ticks = **69.4 %** (446 without output) | §6.1–6.2, §6.5 |
| 5 | Availability **95 % / 0.1 s age** (audit/proposed extra protocol, *not* the user's 5 cm) | **FAIL** | age-gated 0.0000; causal-available 0.8900 full / 1.0000 post-lock; min age ~0.2 s | §6.3 |
| 6 | C1 controlled revisit (internal metric only) | **MEASURED** | revisit NN median 6.196 m → 0.153 m; not GT validation | §7 |
| 7 | C2 isolated 5-arm ablation | **MEASURED — every arm PASS; no demonstrated improvement** | raw p2pl 0.0225–0.0293 m (values differ); each arm's evaluator verdict is `PASS` with all preconditions true; differences lie inside the observed same-code run-to-run variation, with **no significance test** | §8 |
| 8 | T1.2 sensor abstraction (Airy/Odin1) | **PARTIAL / BLOCKED** | config + topic adaptation delivered; P1–P7 board-only | §9.6 |
| 9 | T1.4 state-estimation dependency (legged body-velocity input) | **EXTERNAL DEPENDENCY NOT AVAILABLE / UNQUALIFIED** | interface contract documented (timestamped body-frame velocity + covariance, Q1–Q5); the input itself is not provided and not qualified; no fusion model invented | §9.6 |
| 10 | Historical dataset acceptance (MCD/HILTI on survey maps) | **BLOCKED** | coordinate chain/ROI/transform not closed; out of this document's scope | §2, §14 |
| 11 | Hardware / Orin / calibration | **NOT RUN** | `【board-only】` | §13.2 |

**Status of row 4.** Both modes are final: the **default** mode is accurate at the stamp it carries
(0.0191 m) with a **0.1625 m** current-clock error; the **opt-in strict `current_pose_mode`**
(review-passed implementation) is **MET at the published stamp** (0.0223 m final run) but its
**current-clock error is 0.0591 m > 5 cm ⇒ FAIL** in the final run (run 1 was 0.0457 m, PASS). It
never extrapolates past the odometry's own stamp. The **0.1 s continuity audit** (row 5) remains
FAILED and is a separate item; the mode's **1.0 s** gate/odometry policy is a distinct, separately
measured policy. **The requirements are not all met.**

**Deliverables / where things live**

| What | Path |
|---|---|
| This report | `docs/validation/local_simulation_acceptance.md` |
| Machine-readable index (hashes, statuses, remaining work, hardware-only list) | `artifacts/nonhardware_delivery/manifest.json` |
| Mapped evidence (T1/T2/T3, C2 arms) | `artifacts/mapping_corrected_20260930/` |
| Localization evidence (consumer, before/after stamp fix) | `artifacts/simulation_20260930/t3_availability_intake_fix/` |
| C1 intervention evidence | `artifacts/local_loop_intervention/results/provenance.json` |
| Interface / parameters / deployment contracts | `docs/validation/04b73f5_{fastlivo2_interface,param_notes,p3_deployment}.md` |
| Working diagnosis reports (owners) | `.omp/reports/{mapping_accuracy_diagnosis,localization_coverage_diagnosis,simulation_integration}.md` |

---

## 0. Explicit corrections of statements that are no longer true

Each of these was stated (or implied) in earlier reports and **must not be repeated**:

| # | Old statement | Correction (what is actually true now) |
|---|---|---|
| C-1 | "An A/B against upstream FAST-LIO was run as a controlled comparison." | The archived `artifacts/04b73f5_controlled_ab_20260929` (upstream ROS1 replay vs fork ROS2 replay) is **not a fresh controlled A/B**: different world-frame definitions, different configs, different time windows and different artifact types (per-scan `scans.pcd` vs PGO keyframe-stitched map). It is `BLOCKED` for comparison. The only genuinely controlled comparisons in this delivery are (a) the **seam-isolation** run (same renderer, same reference; glitched tail present vs absent) and (b) the **C2 single-factor arms** (one build, same corrected bag, sequential). |
| C-2 | "The shared fork transform can be used to compare against upstream / to score other stacks." | The est→scene chain (`frame_chain.json`) is a rigid SE(3) **fitted on the fork's own odometry** over a pre-declared early window. Applying that same fork-derived transform to another stack's output is invalid; it is map-derived and disclosed as such. |
| C-3 | "The drift/accuracy root causes are proven." | Two **harness** defects were found and fixed with direct measurements (the closed-route seam modulo-interpolation glitch, and the twist scan renderer), and a recorded-map parsing defect was found. The **dataset-level** causes (the `correction` gate as "decisive bottleneck", the front-end drift root cause, the world-frame mismatch) remain **hypotheses / `BLOCKED`** — single sequence, no ablation for those claims. |
| C-4 | "Hardware calibration has been performed / values are available." | No hardware calibration exists. `lio.yaml`'s `ext_il` is the repository default robot's value; `config/lio_orin_nx.yaml` **deliberately omits** `ext_il` (missing key ⇒ `WARN` and identity/zero placeholder). No `Airy`/`Odin1` calibration, no IMU unit confirmation, no time-sync measurement. |
| C-5 | "The earlier synthetic T1/T2/T3 numbers are the current ones." | The pre-fix synthetic set (`artifacts/simulation_20260930/…`, `/tmp/sim_fixed2`) is **superseded**: those runs used the twist renderer, the first-point publication timing and (earlier still) a recorder that parsed `world_cloud` with a packed 3-float dtype so ~79 % of recorded map points were garbage. T1 was `BLOCKED` at 0.1778 m; T2 used an assumed availability boundary (span subtraction); T3 was `BLOCKED`. The current numbers are in §4–§6. |
| C-6 | "The C2 switches are a validated fix." | `imu_acc_normalize` / `imu_init_mode` / `imu_init_min_samples` are **opt-in ablation switches**, absent from `lio.yaml` (legacy default) and present only in `lio_c2_experimental.yaml`. They are not a validated fix; §8 is the measured, scope-limited verdict. |
| C-7 | "The T2 delivery ratio is 846/894 = 0.9988." | That line mixed two denominators. The ratio is over `covered_scans`; the JSON was always self-consistent, only the prose was wrong (`scans_before_first_frame` are now reported separately). |
| C-8 | "293/293 = 100 % localization coverage." | Withdrawn. Span ratio, sample-association rate and per-second coverage are three different quantities and are reported separately (§6). |
| C-9 | "A 1.24 cm diagnostic proves ICP has cm-level capability." | Withdrawn: the 1.24 cm was the rigid-body image of the odometry under a frozen `map←odom`; a frozen transform is not a localization result. |
| C-10 | "The localization ATE proves the user's online ≤ 5 cm requirement (and the `/robot_pose_map` consumer)." | **Resolved into two measured quantities after a real fix.** Three successive proxies were distinguished and rejected; the criterion was then moved to the **real** `/robot_pose_map` stream (actual `robot_pose`/`map_pose_publisher` node + 20 Hz census). (1) **Before the stamp fix** (`stamp = now()`): published-stamp ATE **0.1263 m — FAIL**. (2) **After the stamp fix** (publish the **resolved transform's own header stamp**): published-stamp ATE **0.0191 m (n = 1320, MEASURED) — the 5 cm requirement is MET for the stamp the pose carries**. **Separately reported, not hidden**: the **current-clock error** of the published pose is **0.1625 m** and the published-stamp age is median **0.219 s** (p95 0.319 s) — so a subscriber that needs the pose *at its own clock* still needs an **accepted-correction propagation contract that does not exist yet** (documented as remaining work). The added 95 %/0.1 s continuity protocol still FAILS (availability 0.0000). |
| C-11 | "The localizer emits one correction sample per measured frame, and the separate-route prerequisite is enforced." | Two **real defects** were found by the acceptance review: (a) `localizer_node.cpp` called `sendBroadCastTF(frame_time)` after **every** successful ICP — including `outcome.accepted == false` / frozen-during-recovery — **re-stamping an old trusted correction as new measurement evidence** (observed in `monitor.jsonl:1786-1790`: identical transforms at `1700001056.8998437` and `1700001057.0998437`); (b) `test_t3_localization.py` read the route/prior provenance at the manifest **top level** while `run_localization_sim.sh` writes them under `stages.localization`, so the same-route rejection silently never executed. **Both are fixed in the current tree** (guarded by `if (published_moved)`, and the stage is now read explicitly with a fail-closed requirement) — see §6.4. |

---

## 1. Requirement and protocol provenance (what the user asked vs what was added)

| Quantity | Origin | Notes |
|---|---|---|
| Map accuracy **≤ 5 cm** (point-to-plane RMSE, estimated map vs reference map) | **User requirement** (WP2 T2.1 acceptance; historical handoff §5 step 3) | The headline acceptance quantity. Also quoted in `test_t1_accuracy.py`. |
| Joint goal "**speed 0.5–1.0 m/s AND accuracy ≤ 5 cm**" | **User requirement** | The speed envelope is the robot's own; the local-simulation route runs ~0.5 m/s mean (in envelope), the historical MCD sequence is ~1.56–1.71 m/s (out of envelope ⇒ stress test only). |
| **95 % coverage / availability gate** | **Audit / proposed extra protocol** — not part of the user's 5 cm requirement | Introduced by the availability protocol (`test_t3_localization.py`, `t3_verdict.json:"min_coverage": 0.95`, `"coverage_gate_applied"`). Explicitly labelled a *protocol choice*. |
| **0.1 s correction-age contract** (`hold_validity_s`) | **Audit / proposed extra protocol** — not part of the user's 5 cm requirement | The declared consumer bound on the age of the accepted `map←odom` correction. Measured as infeasible with this stack (§6). |
| 0.1 s **frame latency** criterion (T2) | Harness criterion (physical: 10 Hz scan period) | A *different* 0.1 s from the correction-age bound above. Do not conflate the two. |
| `association_tolerance_s = 0.05`, `gap_limit_s = 0.20`, `validity_stale_s = 1.0` | Pre-declared evaluator/protocol constants | Frozen before the runs, never widened after seeing a score. |

---

## 2. Verdict matrix (requirements met / failed / blocked)

| # | Item | Verdict | Scope / caveat |
|---|---|---|---|
| V-1 | **T1 map accuracy ≤ 5 cm** — local synthetic scene | **MET (PASS)** 0.0314 m | Synthetic scene only; single scene/seed; certified by the frozen evaluator with all admissibility preconditions true. Not the historical dataset verdict. |
| V-2 | **T2 latency / backlog / RTF** (host) | **MET (PASS)** observed p95 0.0486 s | Host CPU only; `【board-only】` for Orin. |
| V-3 | **T3 localization accuracy in a DELAYED/at-stamp sense** ≤ 5 cm | **MET (PASS)** 0.0223 m | Diagnostic (evaluated at the odometry stamp). Frozen map hashed; separate route; declared GT-assisted initial pose. |
| V-3b | **Published pose accuracy at its OWN stamp** — the **real** `/robot_pose_map` consumer, 20 Hz census, **after** the stamp fix | **MET (PASS)** **0.0191 m** (n = 1320, MEASURED) | The node now publishes the resolved transform's own header stamp (no `now()`), so the timestamps are truthful and the pose is accurate for the stamp it carries. Before the fix this was 0.1263 m (FAIL). |
| V-3c | **Current-clock accuracy in the DEFAULT consumer mode** | **NOT MET** **0.1625 m** | The default mode publishes the truthful resolved stamp; at the subscriber's own clock the pose is median 0.219 s old. |
| V-3d | **Current-clock accuracy in the opt-in `current_pose_mode:=true` (strict)** | **NOT MET (FAIL)** **0.0591 m** current-clock — **above the 5 cm criterion**; published-stamp 0.0223 m (n = 1082) | Final, review-passed implementation (no mustfix): gate-answer receipt time derived every query and expiring at 1.0 s even while a request is pending; full chain `T_map_odom · T_odom_child · T_child_base` with `odom.child_frame_id` respected and `child→base_link` at the exact odometry stamp, fail-closed. Publishes 1013/1459 ticks = 69.4 %; the strict gate answers nothing for the 446 without output. Not hardware-validated. |
| V-3e | **Current-clock accuracy of the NEW bounded-prediction consumer** (`consumer_mode:=predict`, `predict_current_pose:=true`; 2026-09-30 recovery revision, evidence `artifacts/software_metrics_recovery/step4_runtime/runs_final/predict/`) | **MET (PASS)** **0.0203 m** (n = 1064) | The published stamp IS the query clock (short-window prediction ≤ 0.12 s from two odometry samples), so published-stamp and current-clock ATE coincide. Post-lock 20 Hz timely-output **0.9953** (≥ 0.95 PASS); **full span 0.8858 = NOT MET** (the ~6.62 s IMU-init + first-lock startup stays in the denominator). Orientation median 0.0066 rad (p95 0.0240); `prediction_dt` median 0.103 s, max 0.1175 s; rejections: `odom_not_ready 108 / no_correction 21 / gate_not_valid 3 / prediction_dt_out_of_range 3 / odom_interval_out_of_range 2`. The old 0.1 s protocol is unchanged and still FAILs (V-4). |
| V-3f | **Current-clock accuracy in `consumer_mode:=current`** (same new revision; `runs_final/current/`) | **MET in this run** **0.0482 m** (n = 1075, published-stamp 0.0180 m) | **Not directly comparable to V-3d (0.0591 m)**: different source revision (gate/validity + publisher changes) and a different recorder tick census (this run's ticks/cells are the 20 Hz clock grid, not the old 50 ms tick). Both numbers stand, each with its own denominator; V-3d is not overwritten. |
| V-3g | **Real `/localizer/relocalize_check` lock answer, both codes** (new revision; `runs_final/service_smoke_v2/`) | **MET (PASS)** | 303 real calls: code 0 and code 1 both answer `false` with no accepted correction, both turn `true` after the operator relocalize (operator path corroborates immediately; autonomous consecutive qualification is covered separately by gate tests and startup timelines), and both turn `false` within ~1 s of the odometry/cloud freeze (`validity_timeout_s = 1.0`). No `code == 1` bypass remains. |
| V-4 | **T3 availability, 95 % + 0.1 s age protocol** | **FAIL** | Age-gated availability 0.0000 (full span and post-lock); causal-available 0.8900 full / 1.0000 post-lock; min achievable age ~0.2 s. A *separate* protocol choice, not the user's 5 cm requirement. |
| V-5 | **C1 controlled revisit internal metric** | **MEASURED (internal only)** | Offline intervention on frozen artifacts; **not** GT validation and **not** a fresh replay. |
| V-6 | **C2 isolated 5-arm ablation** | **MEASURED — every arm PASS; no demonstrated improvement** | raw p2pl 0.0225–0.0293 m (the values differ), but every arm's evaluator verdict is `PASS` with all preconditions true and the differences lie inside the observed same-code run-to-run variation, with **no significance test** ⇒ no arm shows a demonstrated improvement beyond that variation. This is not an equality/null proof. Single build, one corrected bag, sequential. Synthetic result **must not** be generalised to vendor bags. |
| V-7 | **T1.2 sensor abstraction layer** (Airy/Odin1 driver prereqs P1–P7) | **PARTIAL / BLOCKED** | Parameterised config + topic adaptation delivered; P1–P7 need board verification. No `Airy`/`Odin1` driver exists in-repo. |
| V-8 | **T1.4 state-estimation dependency** (legged body-velocity observation) | **EXTERNAL DEPENDENCY NOT AVAILABLE / UNQUALIFIED** | The named input (body-frame linear velocity with timestamp + covariance, from joint encoders/leg kinematics) does not exist and is not qualified; the **interface contract** for it is documented (§9.6, Q1–Q5). No legged-fusion observation model is invented on the SLAM side. |
| V-9 | IMU physical strategies / deskew / debug parameters | **DOCUMENTED** | Implemented strategies are real (source-level); C2 strategies unverified (§8, §10). |
| V-10 | FastLIVO2 interface contract | **DOCUMENTED** | Authoritative topic/service/TF/parameter tables (§11). |
| V-11 | Historical dataset acceptance (MCD/HILTI ≤ 5 cm on survey maps) | **BLOCKED** | Coordinate chain / independent transform / authoritative ROI not closed. Out of this document's scope; no倍数 (ratio) claim. |
| V-12 | Hardware / Orin / calibration / real-robot | **NOT RUN** | `【board-only】`; see §13.2. |

> **There is no "all goals achieved" statement.** The user's 5 cm *map-accuracy* goal is
> **verified in the local simulation** (V-1). For localization, the real consumer's pose is within
> 5 cm **for the stamp it carries** (V-3b, 0.0191 m). The **current-clock** requirement was
> **NOT MET** by the earlier revision's opt-in strict `current_pose_mode` (**0.0591 m > 5 cm**,
> V-3d; its 0.0457 m run is superseded) and is 0.1625 m in the **default** stamp mode. On the
> **2026-09-30 recovery revision** the *new* bounded-prediction consumer
> (`consumer_mode:=predict`) measures **0.0203 m current-clock ≤ 5 cm (PASS, n = 1064)** with
> post-lock timely output **0.9953**, while its **full-span** output rate is **0.8858 — NOT MET**
> (V-3e); the same revision's `current` mode measures 0.0482 m (V-3f), not comparable to V-3d
> across revisions. The historical dataset acceptance remains **BLOCKED** (V-11), T1.2 is
> partial, the T1.4 legged-velocity **input is an unavailable/unqualified external dependency**,
> the availability protocol fails (V-4), and nothing hardware-related has been run.

---

## 3. Current authoritative evidence (measured)

| Metric | Value | Source artifact (sha256 in §3.1) |
|---|---|---|
| T1 raw est→ref p2pl RMSE | **0.03145 m** (threshold 0.05) | `mapping_corrected_20260930/baseline/mapping/t1_verdict.json` |
| T1 denominator (fixed) | 214 506 | same |
| T1 within 5 cm | 92.65 % | same |
| T1 support: ref coverage of est / drifted-out | 0.9982 / 0.18 % | same |
| T1 worst region (octant_000) p99 | 0.0806 m (below the 2× gate) | same |
| T1 all admissibility preconditions | true | same |
| T2 observed input-ready→output-ready p95 | **0.0486 s** (no span subtraction) | `…/baseline/mapping/t2_verdict.json` |
| T2 p99 / median | 0.0545 s / 0.0375 s | same |
| T2 RTF / backlog max | 0.9998 / 0.0772 s | same |
| T2 bag time contract | all 5 checks true; storage−(header+span) max 2.4e-07 s | `…/baseline/mapping/bag_availability.json` |
| T3 **real consumer** `/robot_pose_map` ATE **at the published stamp** (after the stamp fix) | **0.0191 m — PASS** (n = 1320) | `…/t3_availability_intake_fix/consumer_stamp_fix_acceptance/t3_consumer_result.json` |
| T3 same consumer, **current-clock** error, **DEFAULT** mode | **0.1625 m** (n = 1320) | `…/consumer_stamp_fix_acceptance/t3_consumer_result.json` |
| T3 **opt-in strict `current_pose_mode`** (post-fix): published-stamp / **current-clock** | 0.0223 m / **0.0591 m** (n = 1082; age median 0.0930 s) | `…/t3_availability_intake_fix/current_pose_mode_acceptance/t3_verdict.json` |
| T3 mode **cancellation test** (localizer killed at wall 60 s): poses published after the last valid answer | **0** (last publication 59.61 s; next status 59.66 s: `gate_valid=false`, answer age 1.0001 s, correction age 1.268 s) | `…/current_pose_mode_cancellation/consumer_status.jsonl` |
| T3 published-stamp age | median 0.219 s / p95 0.319 s | same (`published_stamp_age_s`) |
| T3 consumer, BEFORE the stamp fix (`stamp = now()`) | 0.1263 m — FAIL | `…/consumer_stamp_fix_acceptance/t3_verdict_before_stamp_fix.json` |
| T3 query-clock surrogate / delayed ATE (diagnostics) | 0.0284 m / 0.0223 m | `…/real_consumer_acceptance/` |
| T3 distinct-stamp rate | 10.00 Hz | same |
| T3 availability, full GT span, age ≤ 0.1 s | **0.0000** (causal-available 0.8900 full / 1.0000 post-lock; correction age median 0.260 s) | `…/consumer_stamp_fix_acceptance/` + `.omp/reports/localization_coverage_diagnosis.md` |
| T3 held-out cross-route ATE (independent prior) | 0.0414 m @ 10.0 Hz, n = 832 | `…/t3_availability_intake_fix/heldout_mapping_route/t3_result.json` |
| C1 baseline revisit NN median / candidate | 6.196 m / 0.153 m | `artifacts/local_loop_intervention/results/provenance.json` |
| C2 five arms (T1 raw p2pl, all PASS) | 0.02555–0.02619 baseline / 0.02250–0.02925 across arms | `artifacts/mapping_corrected_20260930/c2_arms/c2_arm_table.json` |
| Evaluator verification | 58 passed / 0 failed | `04b73f5_evidence_audit_20260929_5V6fKb/evaluation/evaluator_verification.json` |

### 3.1 sha256 of the referenced evidence (independently computed on the files below)

```
c063f2e08d7d4497d8b24f6148ec448692e6803ffab72ddb8dd621612396288e  artifacts/mapping_corrected_20260930/EVIDENCE_INDEX.json
68e600c9838e31de1683db508e934db1ffdfb5cf99aa8d896b3fd9ab61f1d559  artifacts/mapping_corrected_20260930/baseline/mapping/t1_verdict.json
7c312aa43a8302a8188cab38b854f2fe94b8a5e8235a3448c7104912765f6c59  artifacts/mapping_corrected_20260930/baseline/mapping/t2_verdict.json
efa6f9f206e8fa69acc4a9a7959afe27a643350daa16e7d6113024455a512d07  artifacts/mapping_corrected_20260930/baseline/mapping/frame_chain.json
0008b9c613ac250dd65444adfe270aaeb2bb964a362a0dff4a7d7086c61da891  artifacts/mapping_corrected_20260930/baseline/localization/t3_verdict.json
528d9e3c84461624215b7e54ef4223b9fba63ff30acc5612a4e336ba428e618f  artifacts/simulation_20260930/t3_availability_intake_fix/final_coherent_fixed3/t3_verdict.json   # SUPERSEDED (delayed-only, no online metric)
3b4ddd36ab2a10a77cb5e87579a53724820b6eec92efd71b3aa9bfc25f5ea780  artifacts/simulation_20260930/t3_availability_intake_fix/final_acceptance/t3_verdict.json
622dee574b78a4d49beaa46396962bc315d261df75fcceb073b549f5e70128ee  artifacts/simulation_20260930/t3_availability_intake_fix/final_acceptance/t3_online_result.json
13f750d9a094433c0bc577f2d2721fe050ba84ba6e07cf78159e3e9c39e2b008  artifacts/simulation_20260930/t3_availability_intake_fix/final_acceptance/localization_online.tum
9d5f312d11b05217db5dee498fa11296224d99e06640ee6ca535529cf2045015  artifacts/simulation_20260930/t3_availability_intake_fix/final_acceptance/run_manifest.json   # SUPERSEDED (surrogate only)
19db3a0b7a173e9c8e31da4fe2e4269b5a33a575765d66aa4900a3e22ed927aa  artifacts/simulation_20260930/t3_availability_intake_fix/real_consumer_acceptance/t3_verdict.json
c2e83cbc62f406e106edc908a6247a0a587eaa7f6e98bc8da83361eef94a1162  artifacts/simulation_20260930/t3_availability_intake_fix/real_consumer_acceptance/t3_consumer_result.json
3cfe391dec7de51d5ea60a7865345f0c5ab46876fa671950b34ca0e3099c692e  artifacts/simulation_20260930/t3_availability_intake_fix/real_consumer_acceptance/consumer_queries.json
f702abc0b25c0f5ef53e35fc0390657fd0d29ba4fd4ba742ab392c88824a1663  artifacts/simulation_20260930/t3_availability_intake_fix/real_consumer_acceptance/run_manifest.json
120b838695ff7da8a5b7ff748bfd30a56ba101f20e6630057cf38838102bdbb9  artifacts/simulation_20260930/t3_availability_intake_fix/current_pose_mode_acceptance/t3_verdict.json
1b8095e119e3790cbb81c17686946628c84d5cf32412da18af8e3a64ae5a05ea  artifacts/simulation_20260930/t3_availability_intake_fix/current_pose_mode_acceptance/consumer_status.jsonl
6b6f5582c864d28810b70547358ddc26f6c436e22101a9e4a329b7468fb18d0b  artifacts/simulation_20260930/t3_availability_intake_fix/current_pose_mode_cancellation/consumer_status.jsonl
b9a1bf19ddb4cca39539d63b1334b89a145de25d3e7d79b71ae82cd7f100365e  artifacts/local_loop_intervention/results/provenance.json
1c45a1b7087892c212fef9a1e1acb13992a6e7cc2c4ce2cdc8dab3c7d27b5fec  artifacts/mapping_corrected_20260930/c2_arms/c2_arm_table.json
19fe111ca3e245693acd3c976bbc5498aa65838d1bafedda6041af2fa9884124  artifacts/mapping_corrected_20260930/c2_arms/EVIDENCE_INDEX.json
```
Code that produced these numbers (hashed in the delivery manifest): `simulation/synthetic_data/traj_common.py`,
`simulation/synthetic_data/generate_test_bag.py`, `simulation/scripts/check_bag_availability.py`,
`simulation/scripts/test_{t1_accuracy,t2_speed,t3_localization}.py`,
`simulation/scripts/run_{eval_pipeline,mapping_sim,localization_sim}.sh`,
`artifacts/04b73f5_evidence_audit_20260929_5V6fKb/evaluation/map_accuracy_eval.py` (sha256 `408c4786aefd7d54…`),
`…/gt_time_assoc_eval.py` (sha256 `a512e506562f0a70…`).

---

## 4. T1 — map accuracy, measured in the local simulation

* **Result: PASS, 0.03145 m** against the 0.05 m criterion, with the frozen evaluator
  (sha256 `408c4786aefd7d54…`), same stat voxel (0.05), same threshold, same pre-declared
  evaluation window (sim 30–90 s) and the same pre-eval frame chain as before. Nothing was relaxed
  to obtain it: the gains came from fixing the generator and the sensor model.
* **Admissibility preconditions all true**: reference coverage of the estimate 0.9982, zero
  estimate points outside the ROI, the pre-declared regional rule satisfied (worst octant p99 =
  0.0806 m < 2× criterion), and a sourced pre-eval transform whose fit samples are time-disjoint
  from the evaluation window (fit [5.20, 29.50] s, eval ≥ 30 s).
* **What was fixed (harness, not the metric)**: (1) the closed-route seam defect — the heading
  table was interpolated modulo its own length, injecting a ~360° yaw blend into the last 0.01 m
  of each lap (measured pre-fix: `gyro_meas` −309 rad/s, 17.999 °/ms rotation increment); (2) the
  stand→walk velocity step (measured 251 m/s² = 25 g); (3) the twist scan renderer, replaced by
  exact per-ray rendering from the GT pose at each ray's own emission time (pre-fix deviation mean
  1–5 cm, p95 3–19 cm per scan, up to 9.1 m on silhouette rays; post-fix max error 2.2e-06 m over
  30 720 rays); (4) the publication timing (a sweep is now published at its last point's readiness
  while its header keeps the first point's measurement time).
* **Controlled isolation of the seam defect** (same evaluator, same pre-fix reference, pre-fix bag
  truncated just before the wrap): full lap 0.1778 m / BLOCKED vs truncated 0.0501 m / FAIL(value)
  ⇒ the seam alone accounts for 0.178 → 0.050 m; the remaining model artefacts for 0.050 → 0.039 m.
* **Residual attribution** (diagnostic): as-is RMSE 0.0300 m; after one extra rigid SE(3) re-fit
  0.0181 m ⇒ ≈1.2 cm rigid (gauge/chain) + 1.81 cm non-rigid (within-map). The evaluator's
  PCA-normal residual agrees with the exact analytic scene-surface distance to 0.2 mm on the mean
  and 0.7 mm at p99 ⇒ the residual rule is not invalid here. Evaluator floor check (perfect map as
  its own estimate) gives 0.0000 m raw p2pl ⇒ the PASS is not a floor/sampling artefact.
* **Limits**: synthetic scene only (the reference geometry, ROI and GT are generated from the
  scene definition); one deterministic seed, one scene, one host; LIO without PGO; the certified
  number includes the pre-eval chain's gauge uncertainty (≤ 1.4 cm at room scale) and the
  estimator's own drift (1.81 cm non-rigid). **It is not a claim that every surface is within
  5 cm** — 92.65 % of the sampled map is, and the worst octant p99 is 0.0806 m.

---

## 5. T2 — speed / latency / backlog, measured on this host

* **Criterion**: the **observed input-ready → output-ready** p95 delay ≤ 0.1 s (one 10 Hz frame
  period). **Result: PASS, p95 0.0486 s**, p99 0.0545 s, median 0.0375 s. No frame span is
  subtracted: the bag publishes each complete sweep at its **last point's readiness**, and
  `check_bag_availability.py` verifies that contract per run from the bag itself (all five checks
  true; storage − (header + span) max 2.4e-07 s); `test_t2_speed.py` fails closed if it does not hold.
* Replay: RTF 0.9998 (84.70 s simulated in 84.72 s wall), backlog max 0.0772 s, delivered
  843/845 covered scans = 0.9976 (895 received, 50 before the first output frame); GT route
  completes a full lap (closure 0.00015 m), core walking speed min 0.480 / mean 0.508 / max
  0.531 m/s (inside the 0.5–1.0 m/s envelope; the instantaneous minimum oscillates because the
  gait vibration is real motion).
* **Host-only.** No Orin/Jetson latency is claimed `【board-only】`. The criterion is a *different*
  0.1 s from the T3 correction-age contract (§1).

---

## 6. T3 — localization on a frozen map, separate route

### 6.1 What is measured (three quantities, only the last is the user's)

Authoritative set: `artifacts/simulation_20260930/t3_availability_intake_fix/real_consumer_acceptance/`.

| Quantity | Definition | Measured | Status |
|---|---|---|---|
| **Published pose accuracy at its OWN stamp** (the **user criterion** for the pose stream) | the actual `robot_pose`/`map_pose_publisher` node launched on the sim clock, `/robot_pose_map` scored against GT **at the published stamp**; 20 Hz census; node publishes the resolved transform's own header stamp | **0.0191 m** (n = 1320) | **PASS, MEASURED, `user_requirement_met = true`** |
| **Current-clock error**, **DEFAULT mode** | the published (truthful-stamp) pose compared with GT at the **consumer's arrival clock** | **0.1625 m** (n = 1320); age median 0.219 s | **NOT MET** in the default mode — opt in to `current_pose_mode:=true` (next row) |
| **Current-clock error**, **opt-in `current_pose_mode:=true` (strict, final review-passed code)** | gate-answer receipt-derived freshness (1.0 s expiry), full chain `T_map_odom · T_odom_child · T_child_base`, stamped with the odometry's own stamp; no extrapolation past that stamp | published-stamp **0.0223 m** (n = 1082) = **MET**; **current-clock 0.0591 m = FAIL (> 5 cm)**; age median **0.0930 s**; census **1013 / 1459 ticks with output = 69.4 %** (446 without) | accepted implementation; the current-clock **requirement is not met in the final run** |
| Query-clock **surrogate** | held `T_map_odom` at its older stamp `s` ∘ newest odometry at `t`, re-stamped at `/clock` — *not* the consumer | 0.0284 m | diagnostic |
| Delayed timestamp-associated ATE | receive-causal composed stream scored at the **odometry stamp** | 0.0223 m | diagnostic |
| Same consumer **before** the stamp fix (`stamp = now()`) | published-stamp ATE | 0.1263 m | FAIL (the defect the fix removed) |

* **Census** (post-fix run): 1 434 ticks; 1 203 with output; 231 without (59 no `/clock`, 231 no
  common time); first output at GT + 6.20 s. Published-stamp age median **0.219 s** / p95 0.319 s;
  chain common-time age median 0.244 s.
* Common context: separate localization route, frozen prior map (hashed), declared **GT-assisted**
  initial pose, no alignment performed by the test, distinct-stamp rate 10.00 Hz,
  `association_rate_est = 1.0` (coverage ≠ association).
* Held-out cross-route check (fixed3 *mapping* bag, INDEPENDENT pre-fix prior, `gt_mapping.tum`):
  0.0414 m at 10.1 Hz, n = 832 — different route/prior, and **not** a consumer measurement.

> **Harness disclosure**: the sim TF tree provides `odom→body` while the node resolves
> `map→base_link`; the run therefore inserted a **declared static identity bridge `body→base_link`**
> (recorded in the run manifest). That is a sim-harness adaptation, not a hardware claim.

### 6.2 The stamp fix, and the contract that is still missing

* **What was wrong.** `map_pose_publisher` published the transform resolved at the chain's latest
  **common** time but stamped the message with `now()`. At the route's 0.5 m/s a 0.24 s stale pose
  is ~12 cm of travel, and the consumer measured **0.1263 m — FAIL**.
* **What was fixed (landed, owned by the localization owner).** The node now publishes the
  **resolved transform's own header stamp** instead of `now()` (frame preserved; no extrapolation
  and no invented future pose). The rule is factored into `map_pose_stamp()` with a regression
  (`robot_pose/test/test_map_pose_stamp.cpp`, 1 case). On the **same inputs** (same bag, GT, frozen
  prior; only the stamp changed) the published-stamp ATE goes **0.1263 m → 0.0191 m**, i.e. the
  pose is accurate **for the stamp it carries**.
* **What is still not met by default.** In the **default** mode the published pose is median
  **0.219 s** old (p95 0.319 s), so its **current-clock error is 0.1625 m**. A subscriber that needs
  a pose at *its own* clock must either opt in to the mode below or implement its own propagation.
* **Implemented resolution (opt-in, measured).** `robot_pose/map_pose_publisher` now has a public,
  documented parameter **`current_pose_mode:=true`** (default `false`): it composes the **last
  accepted `map` correction** over the **newest received odometry** and stamps the pose with the
  **odometry's own measurement stamp** (never `now()`). It is **STRICT**: it publishes only while
  the localizer's own gate answers `valid` and that answer is fresh (≤ the declared 1.0 s); stale
  odometry (> 1.0 s) is treated as unavailable; there is **no extrapolation and no future TF lookup**.
  Ages/validity are exposed on **`/robot_pose_map/status`**. Final measurement on the same fixed3
  route with the real node and a 20 Hz census at 0.5 m/s (after the two review P1 fixes):
  published-stamp ATE **0.0223 m (n = 1082) = MET**, **current-clock error 0.0591 m = FAIL (> 5 cm)**,
  published-stamp age median **0.0930 s**, publishing 1013/1459 ticks = 69.4 %. (An earlier run of
  the same route read 0.0186 / 0.0457 m; that run **predates** the P1 fixes and is **superseded**.)
  The default truthful-stamp mode is unchanged.
* Two earlier surrogate attempts were rejected by the review (attempt 1: delayed composition,
  0.0224 m; attempt 2: `/clock`-stamped surrogate, 0.0260 m); the criterion has been moved to the
  real topic, and `user_requirement_met` depends on it.

This is **independent** of the 95 %/0.1 s protocol failure below (a continuity/age property of the
correction stream, not the accuracy of a correctly-stamped pose).

### 6.3 Freshness / availability (the 95 % + 0.1 s protocol)

Two quantities are deliberately **not merged**:

| Quantity | Definition | Measured | Meaning |
|---|---|---|---|
| **Age-gated availability** (protocol, `hold_validity_s = 0.1`) | a correction that has already ARRIVED with `query_stamp − tf_message_stamp ≤ 0.10 s`, plus a validity answer that had already arrived | **0.0000** full span / **0.0000** post-lock (`availability_age_census` over 60 001 queries: 5 200 `no_odom_frame`, 1 200 `no_tf_received_before`, 53 401 `too_old`, 200 `gate_not_valid`, 0 `no_valid_answer`) | The declared 0.1 s freshness contract is **infeasible** at any output rate this ICP can reach. |
| **Causal-available (no age bound)** — diagnostic | a correction that had already arrived, age unbounded | **0.8917** full span / **1.0000** post-lock | The stream is usable; what fails is the *contract*, not the physics. |
| **Correction age** | age of the accepted `map←odom` correction (`query_stamp − tf_message_stamp`) | median **0.261 s** / p95 **0.377 s** (n = 53 501) | A ~0.26 s-old correction drifts at the odometry-drift rate ⇒ sub-mm over that interval, physically harmless — but it is what makes the consumer's common-time pose 12 cm stale at 0.5 m/s (§6.2). |

* The minimum achievable age is ~0.2 s (refine ICP 68 ms at 8 iterations + transport + one 0.1 s
  frame period), so the age-gated availability is 0 **by construction**. Nothing was widened.
* **Delayed composed ATE** is the composed, receive-causal stream (`0.0223 m`); it is *not*
  re-stamped to look fresher, and the evaluator's `median_est_period_s = 0.1000 s` "10 Hz" is a
  sampling-grid artefact — the measurement is `distinct_tf_stamp_rate_hz`. (Diagnostic, §6.1.)
* **Startup** (GT's first 5.2 s have no odometry frame at all — estimator IMU init; first
  localization pose +1.6 s later) is reported **separately** and stays in the primary denominator;
  even a perfect localizer could not exceed ~0.91 full-span coverage.
* **Coverage ≠ association**: `association_rate_gt` (fraction of GT samples matched) is reported
  independently from per-second time coverage. The old "293/293 = 100 %" is withdrawn (C-8).

### 6.4 Defects found, fixed, and still open in the localizer

Intake stopping permanently on a stalled `ApproximateTime` pairing (bounded stamp-keyed
`pair_cache.h`); a reliable keep-last-1 odometry reader freezing ~50 s in (canonical sensor-data
QoS); the intake falling behind the live stream (latest-sample intake; localizer now adds only
0.16 s); freshness fabrication in `timerCB` (one sample per accepted update, stamped with the
solved frame); `relocalize_check` answering from stale evidence (`lock_validity.h`); avoidable
per-update target KD-tree rebuild (`force_no_recompute`); and an authorised ICP-cost change
(`refine_max_iteration 15 → 8`, mean align 160 → 102 ms) that reaches the configured 10 Hz with no
accuracy loss (3.13 cm at 8 vs 3.22 cm at 15 on identical inputs; 4.14 cm held-out).

Acceptance-review findings, with status at the time of writing:

| # | Finding | Status |
|---|---|---|
| P2a | `localizer_node.cpp` re-stamped a frozen correction on gate rejection (`sendBroadCastTF(frame_time)` after every successful ICP, including `accepted == false`). | **FIXED, verified in the current tree**: the call is now guarded by `if (published_moved)` (a TF sample is emitted only when the *published* `map←odom` correction actually moved — adopt / accepted EMA update / recovery snap); a rejection or a degraded accepted update keeps the previous stamp. Two new gate regressions pin the invariant. |
| P2b | `test_t3_localization.py` read the route/prior provenance at the manifest top level while they live under `stages.localization`, so the same-route rejection silently never ran. | **FIXED, verified in the current tree**: the stage is read explicitly, `trajectory` + `mapping_trajectory` + `prior_map_sha256` are required and the check **fails closed** if absent. |
| P1 | The timestamp-associated ATE cannot certify the online consumer. | **RESOLVED — the criterion now measures the real consumer.** Surrogates rejected; `record_consumer.py` records the **real** `/robot_pose_map` + `/tf` + `/clock` with a 20 Hz census, and T3 scores **that** stream: 0.1263 m before the stamp fix, **0.0191 m at the published stamp after** it. The **current-clock** quantity is reported separately at **0.1625 m** (not claimed as met). |
| P3 | The consumer published a pose resolved at the latest **common** time with a `now()` stamp, so its output lagged the robot. | **FIXED** for stamp truthfulness: the node now publishes the **resolved transform's own header stamp** (`map_pose_stamp()` + `robot_pose/test/test_map_pose_stamp.cpp`); published-stamp ATE 0.1263 → **0.0191 m**. **Residual (OPEN, design in §6.5)**: the current-clock error is still **0.1625 m** because no accepted-correction propagation contract exists. |

> The numbers in §6.1/§6.3 come from the authoritative post-fix set
> `artifacts/simulation_20260930/t3_availability_intake_fix/consumer_stamp_fix_acceptance/`
> (P2a/P2b/P3-stamp landed; real-consumer census measured, before/after both retained). Earlier
> sets — `…/real_consumer_acceptance/` (pre-stamp-fix consumer), `…/final_acceptance/`
> (query-clock surrogate) and `…/final_coherent_fixed3/` (delayed only) — are retained for
> traceability and must not be quoted as the criterion.

**Remaining software work (documented, not hidden)**: the **continuity contract** — the 0.1 s
correction-age bound must be restated at a physically meaningful value (≥ 0.3 s) or the pipeline
latency must come down; `relocalize_check` still answers `code == 1` as an unconditional `valid`;
the first self-lock reports invalid until an operator `relocalize` (wrong for autonomous
deployment). The current-clock accuracy itself is **implemented** as the opt-in mode of §6.5.

### 6.5 The opt-in current-pose mode: review-passed; the current-clock requirement still FAILS

> **Status: implementation review-passed (no mustfix); the current-clock requirement is NOT met.**
> The two review P1s are fixed and the reviewer approved both, plus the cancellation artifacts:
> (a) the **gate-answer receipt time is derived every query** and **expires at the declared 1.0 s
> bound even while a request is pending**; (b) the composition is the full chain
> **`T_map_odom · T_odom_child · T_child_base`**, with `odom.child_frame_id` respected and
> `child→base_link` looked up **at the exact odometry stamp** (fail closed if unavailable).
> Measured on the same fixed3 route (real node, 20 Hz census, 0.5 m/s), strict
> `current_pose_mode:=true`, **final run**:
>
> * **published-stamp ATE 0.0223 m (n = 1082) ≤ 5 cm → MET** (run 1: 0.0186 m — also MET). The
>   user's published-stamp criterion is met in both runs.
> * **current-clock error 0.0591 m → FAIL (> 5 cm)** in the final run (run 1: 0.0457 m, PASS). The
>   mode never extrapolates past the odometry's own stamp, so the metric is genuinely at/over the
>   bound in the final run — it is **not** presented as passing or "at the boundary".
> * published-stamp age median **0.0930 s**; **census 1013/1459 ticks with output = 69.4 %**, 446
>   without (pre-fix comparison 1227/1461 = 84.0 %). The strict gate legitimately answers nothing
>   for the remainder.
>
> The **default** mode is unchanged (truthful resolved stamp, **0.1625 m** at the subscriber's
> clock), and the **0.1 s continuity audit remains FAILED** (§6.3): availability 0.0000 over the
> full 60 001-query startup-inclusive span, causal-available 0.8917. The opt-in **1.0 s**
> gate/odometry policy is a **distinct, separately-measured policy**, not a fix for the 0.1 s
> contract. Not hardware-validated.

What the mode does (final implementation):

1. **Inputs at query clock `t_q`**: the newest accepted `map←odom` correction with its true stamp
   `s ≤ t_q`; the odometry stream `odom←body(t)` with stamps; the published pose of the resolved
   transform (`map←base_link` at the chain's common time).
2. **Rule**: pick the newest odometry sample with stamp `t_o ≤ t_q`; propagate `odom←body` from
   `t_o` to `t_q` with the stream's own body twist; compose
   `T_map_base(t_q) = T_map_odom(s) ∘ T_odom_body(t_q)`; publish with stamp **`t_q`**.
3. **Bounded, fail-closed**: require `t_q − s ≤ hold_validity_s_contract` and a propagation horizon
   `t_q − t_o ≤ Δt_max` (a declared value; the accuracy cost of linear extrapolation is
   `½·|a|·Δt²`, e.g. ≤ 0.008 m at `Δt = 0.1 s` and `a ≤ 1.5 m/s²`). Outside either bound,
   publish **with the resolved stamp** (today's truthful behaviour) or mark invalid — never
   re-stamp without propagating, and never look up a future transform.
4. **Evidence required to claim it**: 20 Hz query-clock ATE (the same census harness,
   `record_consumer.py`), the `Δt` distribution, the rejection counts, and the fraction of queries
   served with a propagated (rather than resolved-stamp) pose.
5. **Explicitly not in scope here**: choosing `hold_validity_s_contract`/`Δt_max` is a design
   decision; the current 0.1 s age protocol is already known infeasible (~0.2 s minimum), so the
   contract value must be re-derived, not inherited.

---

## 7. C1 — controlled offline loop intervention (internal revisit metric)

* **Nature**: a **controlled OFFLINE intervention** on the frozen `replay_2` artifacts, not a fresh
  replay and **not** a controlled online A/B. It re-runs the repository's registration cascade on
  the frozen patches/poses, with the production gate config, and forces one candidate
  (`kf522 ← kf6`) to measure what the `correction` gate alone is doing.
* **Internal revisit metric [measured]**: baseline **0 accepted**, revisit NN median **6.196 m**
  (p90 10.087 m; fraction ≤ 0.1 m = 0.13 %), candidate **1 accepted**, revisit NN median
  **0.153 m** (p90 4.572 m; fraction ≤ 0.1 m = 41.1 %, ≤ 0.5 m = 67.5 %). Pose shift: kf522 =
  10.179 m, kf6 = 0.000 m, RMS 4.127 m, max 10.218 m. Baseline poses reproduce the archived poses
  to < 5e-8 m.
* **Provenance distinction [measured]**: the production log logs scalars only
  (`rel_t = 1.067 m, corr = 10.180 m (allow 2.80 m), dyaw = −4.01°, p2pl = 0.0263 m,
  overlap = 0.434`; sole fail `correction`); it does **not** log the 6-DoF matrix, so the original
  SE(3) is **not** claimed from the log. The re-registered measurement (selected seed
  `odom_prior`) gives `rel_t = 1.0665, corr = 10.1803, allow 2.8296, dyaw = −4.013°, p2pl = 0.0263,
  overlap = 0.434`, `t = [−0.84636, −0.64050, 0.10424]`, `q_xyzw = [0.023187, −0.004236, 0.305445, 0.951918]`.
* **Boundary**: `full_gt_improvement_claim = false`. Revisit-window self-consistency is **not** GT
  validation and no held-out frame-valid map-accuracy assertion is made. Held-out evaluation stays
  `BLOCKED_PENDING_EVALCONTRACT` (authoritative ROI, own gravity-constrained fit, member-region
  geometry, p2pl coverage/support preconditions).
* Source changes are production-inert (a force hook for the designated pair only + optional audit
  capture, private flags default `false/0`; not called by `pgo_node`; `test_gate_math` 6/6 and
  `test_scan_context_yaw` 3/3 pass).

---

## 8. C2 — isolated five-arm ablation (corrected bag, one build, sequential)

All five arms were re-run on the **corrected** bag from **one build** (including the
`pcl2_filter_phase` switch, default 0), sequentially, in one isolated domain, host otherwise idle:

```
SIM_DOMAIN_ID=137 bash simulation/scripts/run_eval_pipeline.sh --out-root /tmp/sim_abl_final \
     --variants baseline,c2_1,c2_2,c2_combined,c2_3 --with-trajectory-diagnostic \
     --domain-id 137 --skip-localization
```

| arm | T1 raw p2pl (m) | T1 status | within 5 cm | T2 p95 observed (s) | T2 verdict |
|---|---|---|---|---|---|
| baseline | 0.02619 | PASS | 0.9745 | 0.0499 | PASS |
| c2_1 acc-normalize | 0.02925 | PASS | 0.9484 | 0.0540 | PASS |
| c2_2 first-batch init | 0.02398 | PASS | 0.9923 | 0.0487 | PASS |
| c2_combined | 0.02250 | PASS | 0.9927 | 0.0518 | PASS |
| c2_3 sampling phase | 0.02555 | PASS | 0.9788 | 0.0567 | PASS |

Every arm: `pass_preconditions_all_true = true` and `bag_availability_all_true = true`.

**Reading (honest: no demonstrated improvement).**

* **Verdict booleans**: every arm's T1 is a genuine **`PASS`** — the frozen evaluator certified the
  value *and* every admissibility precondition (`t1_pass_preconditions_all_true = true`, evaluator
  status `PASS`), and every arm's `bag_availability_all_true = true`. The T2 column is each arm's
  own observed-latency p95, and all five T2 verdicts are `PASS`.
* **Diagnostic raw values**: the five raw p2pl values span 0.0225–0.0293 m and are *not* equal —
  that span is what the table reports. It sits **inside the observed same-code run-to-run variation**
  (0.024–0.032 m over two independent baseline runs). With **no significance test** performed, the
  correct statement is: **no arm shows a demonstrated improvement beyond that variation.** This is
  **not** an equality or null proof, and the arms did not produce identical numbers.
* **Which table is authoritative**: this **executed** batch table
  (`c2_arms/c2_arm_table.json`, run root `/tmp/sim_abl_final`, persisted) — its baseline arm is
  0.026195 m. The larger 0.03145 m is the *separately measured* corrected baseline
  `/tmp/sim_fixed3` (§4). They are two runs of the same code on the same bag; the difference is the
  same run-to-run spread, so quote each value with its run root rather than treating them as
  contradictory.

*Compact results file*: `artifacts/mapping_corrected_20260930/c2_arms/c2_arm_table.json`
(run root `/tmp/sim_abl_final`, persisted by `persist_evidence.sh`; 556 MB bags excluded by design,
their hashes in each arm's `run_manifest.json`).

Interpretation rules (bind every reading of this table):

* The archived C2 arms measured on the defective model are **invalid**; only these five arms count.
* **C2.3 is a null result on the synthetic ordering only.** The synthetic generator emits its cloud
  in emission-time order with 48 beam samples sharing each azimuth, so phase 0 and phase 1 retain
  the same first point and the same `t0` (measured t0 shift 0.000000 s) — a null is the **expected**
  outcome and is **not** evidence about vendor bags.
* On **vendor** ordering the phase IS behaviourally different: IndoorOffice1 `/mid360/livox/lidar`
  has `line == i % 4` for 100 % of points, so phase 0 keeps lines {0,2} and phase 1 keeps {1,3},
  and `t0` shifts 4.768 µs (not constant), moving the frame end with it. HILTI `/hesai/pandar` is
  not index-major and only shifts `t0` by 0.95–3.10 µs.
* T2 here is host-only and was taken with no other ROS run active.

---

## 9. ROS 2 package / config / TF / time / extrinsics contract

### 9.1 Packages (`fork/src`, 20 `package.xml`)

| Package | Path | Build type |
|---|---|---|
| `fastlio2` | `src/sensing/fastlio2` | ament_cmake |
| `localizer` | `src/localization/localizer` | ament_cmake |
| `pgo` | `src/optimization/pgo` | ament_cmake |
| `hba` | `src/optimization/hba` | ament_cmake |
| `livox_ros_driver2` | `src/sensing/livox_ros_driver2` | ament_cmake_auto |
| `lidar_preprocessor`, `pointcloud_processor`, `pcd2grid`, `octomap_ros2`, `msgs/{octomap_msgs,pcl_msgs}` | `src/sensing/…` | ament_cmake |
| `robot_pose` | `src/localization/robot_pose` | ament_cmake |
| `sensing_launch`, `universe_launch` | `src/launch/…` | ament_python |
| `interface` | `src/common/interface` | ament_cmake (+ `rosidl_default_generators`) |
| `cmd_vel_smoother` | `src/control/cmd_vel_smoother` | ament_cmake |
| `g1_movement_gate` | `src/control/g1_movement_gate` | ament_python |
| `g1_multi_goal_manager`, `robot_2d_navigation` | `src/planning/…` | ament_cmake |
| `task_server` | `src/planning/task_server` | ament_python |

### 9.2 Configuration files

`src/sensing/fastlio2/config/` — `lio.yaml` (legacy default; **no** C2 keys),
`lio_c2_experimental.yaml` (C2 switches on), `lio_highres.yaml` (also silently disables the static
IMU-init window), `lio_orin_nx.yaml` (**no** `ext_il`; unverified deployment start point).
`src/localization/localizer/config/localizer.yaml`; `src/optimization/pgo/config/pgo.yaml`;
`src/optimization/hba/config/hba.yaml`; `src/planning/robot_2d_navigation/config/*.yaml`;
`src/sensing/lidar_preprocessor/config/config.yaml`. Plus harness profiles under
`simulation/ablations/*.yaml` and `simulation/launch/*.launch.py`.

### 9.3 TF contract (source-verified)

Exactly three `TransformBroadcaster` holders:

| Node | Transform | Code | Mode |
|---|---|---|---|
| `lio_node` | `world_frame → body_frame` (`odom → body`) | `lio_node.cpp:547-569`, call `:605` | both modes |
| `pgo_node` | `map_frame → local_frame` (`map → odom`) | `pgo_node.cpp:526-543` | mapping only |
| `localizer_node` | `map_frame → local_frame` (`map → odom`) | `localizer_node.cpp:346-360` | localization only |

Contract: `map→odom` has **exactly one** publisher per period (`pgo` and `localizer` are never
launched together — verified in `sensing.launch.py` / `universe.launch.py`); `odom→body` is
published only by `lio_node`; `body→base_link` is a static publisher in `universe_launch` only (the
`sensing` mode uses `base_frame_id = body`). The localizer's child frame is overwritten by the
inbound odom `frame_id` (`localizer_node.cpp:340-341`). **There is no `body→lidar` / `body→imu`
sensor TF** — `body` IS the IMU body frame (`ext_il` semantics). A fixed defect: the old
`static_transform_publisher` positional form passed `body_base_roll/pitch` into the `pitch/roll`
slots; it now uses the Humble named parameters and a **discriminating** non-zero-angle smoke
verified the read-back quaternion exactly (`--roll 0.3 --pitch -0.2 --yaw 0.1` ⇒
`(x,y,z,w) = (0.153439,-0.091158,0.064071,0.981856)`).

### 9.4 Time contract (source-verified)

* Only a **monotonicity check** exists (four callbacks clear their buffer on `timestamp < last_*`
  and log `"... Message is out of order"`). There is **no** soft-sync threshold, no PPS/PTP status
  read, no clock-drift compensation, no cross-source validation.
* Three streams must be assumed to share one hardware clock domain. Hard sync is a **deployment
  prerequisite**, not a code capability — **no synchronisation-accuracy number is claimed**.
* Per-point time semantics: `curvature` = milliseconds relative to the frame start. For Livox
  `CustomMsg` this holds by driver contract. For generic `PointCloud2` the code's zero point is
  **the first range-filtered retained point**, which is **not necessarily the message header**; this
  must be verified per LiDAR (unverified for Airy/Odin1). Missing/non-float time field ⇒
  `curvature = 0` ⇒ **silent** loss of intra-scan deskewing.

### 9.5 Extrinsics contract

`ext_il = [t_x,t_y,t_z,q_x,q_y,q_z,q_w]`, constructed as `Eigen::Quaterniond(ext[6],ext[3],ext[4],ext[5])`
(i.e. `(w,x,y,z)`), unconditionally normalised when `norm()>0`; semantics
`p_imu = R_il·p_lidar + t_il`. `ext_lc` is stored inverted (`r_cl = R_lcᵀ`, `t_cl = −R_lcᵀ·t_lc`) and
is used **only** for colour projection — it does **not** enter the estimate. `esti_il` exists but
`initialize()` writes `r_il/t_il` from config unconditionally ⇒ treat extrinsics as **fixed**; the
calibration error goes straight into the map error. The simulation identity override is
**simulation-only** and must never be reused on hardware.

> **Deployment safety warning (Airy/Odin1 / any new LiDAR-IMU pair).** `config/lio_orin_nx.yaml`
> deliberately omits `ext_il`. On a missing key `loadParameters()` only `WARN`s and falls back to
> `r_il = I, t_il = 0` (`lio_node.cpp:266`). That **identity placeholder is unsafe for
> deployment**: it silently produces a systematically wrong map, not an error. The calibration is
> **explicitly unavailable** and is **not fabricated** anywhere in this repository — the deployment
> profile must not be run as-is without a measured `ext_il`.

### 9.6 T1.2 / T1.4 contracts and their current status

**T1.2 sensor abstraction layer** (Airy/Odin1 driver):

| Prereq | Judged from | Failure mode | Status |
|---|---|---|---|
| P1 cloud message type explicit (`CustomMsg` or `PointCloud2`) | `lio_node.cpp:114-132` | no input | `【board-only】` |
| P2 `PointCloud2` per-point relative-time field, datatype ∈ {FLOAT32,FLOAT64} | `utils.cpp:49-62` | **silent** loss of deskewing (`curvature = 0`) | `【board-only】` |
| P3 time-field scale matches `pcl2_time_scale` (must yield **seconds**) | `utils.cpp:98-110` | wrong frame-end / IMU breakpoint | `【board-only】` |
| P4 IMU acceleration unit (g vs m/s²) matches `imu_acc_scale` | `lio_node.cpp:327-329` | 10× gravity/accel scale error | `【board-only】` |
| P5 IMU topic/rate in the same clock domain as LiDAR | `lio_node.cpp:418,437-438` | init/sync block, out-of-order clears | `【board-only】` |
| P6 (Livox family) `lidar_max_line` matches line count (strict `<`) | `utils.cpp:10` | silent dropping of over-limit points | board check |
| P7 intrinsic/extrinsic recalibration (`ext_il`, and `ext_lc`+`cam_*` if camera) | `lio_node.cpp:256-291` | calibration error not absorbed by the filter | `【board-only】` / no data |

⇒ T1.2's deliverable is **parameterised config + driver topic adaptation + P1–P7 board records**;
the deployment profile is `config/lio_orin_nx.yaml` (deliberately without `ext_il`). Until P1–P5
are confirmed, **no** recommended `pcl2_time_field/scale`, `imu_acc_scale` etc. may be quoted for
Airy/Odin1. Airy/Odin1 are **not Livox products**; the Livox driver and its timestamp/sync options
must not be assumed to apply.

**T1.4 state-estimation dependency** (legged platform): this is a **named external
input/dependency, not a mandate to invent a legged fusion model on the SLAM side.** `lio_node`
subscribes **only** IMU/LiDAR/Image (`lio_node.cpp:108-137`); the 21-D state is
`(r_wi,t_wi,r_il,t_il,v,bg,ba)` with **no** wheel/foot kinematic observation; `lio_odom.twist` is an
**output**. A quadruped has no wheel odometry, so the only physically possible body-velocity source
is joint encoders + leg kinematics/contact (`unitree_sdk2_python` link). **Status: the input is not
provided and not qualified — external dependency unavailable.** What this repository does provide is
the **interface contract** the input must satisfy before any fusion is even discussable (Q1–Q5:
timestamped body-frame linear velocity with covariance; its own independent accuracy/latency/slip
qualification; time alignment in the IMU/LiDAR clock domain; zero-velocity/contact-phase handling;
body-frame extrinsics). No observation model is fabricated here, and the absence of a legged
odometry input is a **dependency gap**, not a delivered-SLAM omission.

---

## 10. IMU physical strategies, deskew and debug parameters

### 10.1 IMU initialisation (physical strategies actually implemented)

| Strategy | Parameter(s) | Default / repo value | Physical behaviour | Location |
|---|---|---|---|---|
| Static-window init | `imu_init_window_s`, `imu_init_static_gyro_std`, `imu_init_static_acc_dev` | `0.0` / **`3.0`**; `0.005 rad/s`; `0.3 m/s²` | Picks the quietest contiguous window (scored by **gyro std only**); window end state assumed `v=0` then propagated to the newest IMU sample. `>0` enables it. | `imu_init.h:125-139`, `imu_processor.cpp:124-138` |
| Timed fallback | `imu_init_max_wait_s` | `15.0` / **`5.0`** | After the wait, fall back to the quietest window **unconditionally** (time-only trigger, no extra gyro-quiet requirement) — for vibrating-but-not-rotating platforms (quadruped) | `imu_init.h:141-143` |
| Legacy count mode | `imu_init_num` | `20` / `100` | Used when `imu_init_window_s ≤ 0` (e.g. `lio_highres.yaml`, which omits it ⇒ static window **silently disabled**) | `imu_init.h:103-108` |
| Measured gravity | `gravity_align` | `true` | If `8.5 < |a_mean| < 10.5` and window `acc_dev < 0.3`, override `State::gravity` with the measured `|a|` and align `r_wi` to gravity; else keep `9.81` | `imu_processor.cpp:99-103` |
| Online extrinsic estimation | `esti_il` | `false` | Present but `initialize()` does not branch on it ⇒ extrinsics stay fixed | `commons.h:69`, `lio_node.cpp` |
| Acceleration scaling | `imu_acc_scale` | `10.0` | **Multiplicative** gain (MID360 g→m/s² = 10.0; VN200 already m/s² = 1.0) | `commons.h:39`, `lidar_processor.cpp`/`lio_node.cpp:327-329` |
| C2.1 normalisation *(unverified)* | `imu_acc_normalize` | `false` (`lio.yaml` omits; on in `lio_c2_experimental.yaml`) | Multiplies every accel sample by `9.81/|window mean acc|` | `commons.h:57-60`, `imu_processor.cpp:73-85` |
| C2.2 first-batch init *(unverified)* | `imu_init_mode`, `imu_init_min_samples` | `"static_window"`, `40` | `first_batch` = initialise from the first batch, no static window, no timeout fallback | `commons.h:61-66`, `imu_processor.cpp:33-48` |

### 10.2 Deskew (motion compensation)

* Reverse-order IMU linear interpolation + SO(3) `exp` compensation; per-point `dt` from the
  `curvature` field (milliseconds). **No spline** (an earlier draft's "spline/microsecond" wording
  is corrected).
* `pcl2_time_field` / `pcl2_time_scale`: for generic `PointCloud2` only; empty ⇒ `curvature = 0` ⇒
  **no intra-scan deskew** (silent). Zero point = first range-filtered retained point (§9.4).
* `pcl2_filter_phase` (C2.3): decimation phase `(i − phase) % lidar_filter_num == 0`; default `0`
  (== upstream); **PointCloud2 path only** (Livox `livox2PCL` has no such parameter).

### 10.3 Debug / diagnostics parameters

| Parameter | Effect | Default / repo |
|---|---|---|
| `print_time_cost` | Prints the per-`process()` milliseconds (`lio_node.cpp:575-579`) | `false` / `false` (historic replays `true`) |
| `lidar_type`, `lidar_topic`, `lidar_max_line`, `imu_topic`, `image_topic` | Input selection (see §11) | `livox`, `/livox/lidar`, `4`, `/livox/imu`, `/camera2/camera/color/image_raw` |
| Publishers | All `lio_node` publishers early-return when `get_subscription_count() <= 0` ⇒ "no data" while debugging is frequently "no subscriber" | `lio_node.cpp:482,480,493,517` |
| `lio_highres.yaml` head comment | **Corrected**: it changes far more than the two voxel parameters (`lidar_filter_num` 2→3, `det_range` 100→40, `imu_init_num` 100→20, `na/ng/nba/nbg`, and **drops** the four `imu_init_*` keys) | `lio_highres.yaml` |

### 10.4 Parameter-unit traps (source-verified)

`point_quality_thresh` is a **dimensionless score** (`s > thresh`, `s = 1 − 0.9·|pd2|/√|p_body|`),
**not** metres — the `0.1` in `esti_plane(..., 0.1, pabcd)` is the plane-fit distance in m.
`rough/refine_score_thresh` are **m²**. `curvature` is **ms**. `imu_acc_scale` is **multiplicative**.
`lidar_max_line` is a strict `<`. `move_thresh` is a **ratio**. Gate innovations are per-ICP-update
in the body frame. `na/ng/nba/nbg` dimensions are **unlabelled / undetermined** (the `G·Q·Gᵀ` form
does not prove continuous-time density) — do not classify them.

---

## 11. FastLIVO2 interface (authoritative, source-verified)

**Implemented today = LiDAR-inertial odometry + point-cloud colouring.** The estimator contains only
IMU propagation + LiDAR point-to-plane residuals (`State` 21-D, no camera quantity); the image
channel only feeds projection colouring. There is **no** visual/photometric/*fusion-weight* API — do
not invent one (`CVUtils::weightPixel` is a dead bilinear-patch weight, zero call sites).

**Topics** (relative names; namespace `mapping` under `sensing.launch.py` / `universe.launch.py`):

| Dir | Name | Type | QoS |
|---|---|---|---|
| sub | `imu_topic` (default `/livox/imu`) | `sensor_msgs/msg/Imu` | `SensorDataQoS()` |
| sub | `lidar_topic` (default `/livox/lidar`) | `livox_ros_driver2/msg/CustomMsg` if `lidar_type=="livox"`, else `sensor_msgs/msg/PointCloud2` | `SensorDataQoS()` |
| sub | `image_topic` (default `/camera2/camera/color/image_raw`) | `sensor_msgs/msg/Image` (colouring only) | `SensorDataQoS()` |
| pub | `body_cloud`, `world_cloud`, `color_world_cloud`, `lio_path`, `lio_odom` | `PointCloud2`, `Path`, `Odometry` | queue 10000 |

**Services**: `/mapping/save_colored_pcd` (`interface/srv/SaveColoredPcd`, request field `save_path`),
`/pgo/save_maps` (`SaveMaps`, fields `file_path`,`save_patches`), `/localizer/relocalize`
(`Relocalize`, flat fields `pcd_path,x,y,z,yaw,pitch,roll`), `/localizer/relocalize_check`
(`IsValid`, field `code`), `/hba/refine_map` (`RefineMap`), `/hba/save_poses` (`SavePoses`);
subscription `/initialpose` (`PoseWithCovarianceStamped`). Note the *dispatch* semantics: `refine_map`
only *loads* the map and sets a flag — the HBA iterations run in a 100 ms timer, so wait for
`END OPTIMIZE` before `save_poses`.

**Camera** is started by **no** repository launch; the README's camera command publishes under
`/vision/...` while the code default subscribes to `/camera2/...` ⇒ under the default deployment the
image branch never fires `【board-only】`.

**Upstream** is `hku-mars/FAST-LIVO2` (a different code base from `hku-mars/FAST_LIO`); this fork
does **not** correspond to the visual tightly-coupled branch.

---

## 12. Test results (current, authoritative counts)

| Suite | Count | Command | Status |
|---|---|---|---|
| Evaluator/contract tests (`evaluation/test_*.py`) | **58 passed / 0 failed** (`test_map_accuracy_eval.py` 34, `test_gt_time_assoc_eval.py` 12, `test_cross_check_parity.py` 12) | `pytest` in `artifacts/04b73f5_evidence_audit_20260929_5V6fKb/evaluation/` | `[measured]` |
| `fastlio2` `test_imu_init` | **7** cases | `colcon test --packages-select fastlio2 -R test_imu_init` | `[measured]` (source count) |
| `fastlio2` `test_utils_preprocess` | **11** cases | `colcon test --packages-select fastlio2 -R test_utils_preprocess` | `[measured]` (source count) |
| `localizer` tests | **24** cases (12 `test_localizer_gate` + 7 `test_pair_cache` + 5 `test_lock_validity`) | `colcon test --packages-select localizer` | `[measured]`; 22 before the two new gate regressions added with the P2a fix |
| `robot_pose` `test_map_pose_stamp` | **1** case | `colcon test --packages-select robot_pose` | `[measured]` (source count); pins the published stamp = the resolved transform's own stamp |
| Historic 4-target regression set | 25/25 (`test_imu_init` 6, `test_gate_math` 6, `test_localizer_gate` 10, `test_scan_context_yaw` 3) | `colcon test -R "test_imu_init\|test_gate_math\|test_localizer_gate\|test_scan_context_yaw"` | historic, from `.gtest.xml` |

> Never claim a workspace-wide `colcon test` pass: the aggregated `colcon test-result` still reports
> pre-existing lint/style failures and a stale 20260928 `Test.xml` on files that were not changed.

---

## 13. Reproduction

### 13.1a Ready-to-run bring-up with the CURRENT source (no hardware)

Two launch files bring the current source up with **no hardware**: `simulation/launch/sim_fastlio2.launch.py`
(mapping: bag playback + `lio_node`, optional RViz) and `simulation/launch/sim_localization.launch.py`
(the same + the **real** `localizer_node` against a frozen prior map). Both read the package config
and apply only the sim-relevant overrides (`lidar_type: pointcloud2`, bag topics, per-point time
field, `imu_acc_scale: 1.0`) into `<work_dir>/lio_sim.yaml`; the sim identity `ext_il` is
**simulation-only**. Arguments (from the files): `bag_path`, `work_dir`, `config_file`,
`extra_config` (merged last, for ablations), `rate`, `start_delay`, `rviz`, `log_level`; the
localization launch adds `localizer_config`, `prior_map`, **`consumer_mode`**
(`stamp` | `current` | `predict`, default `stamp`; it starts the real consumer
`robot_pose/map_pose_publisher` with `(current_pose_mode, predict_current_pose)` =
`(false,false)` / `(true,false)` / `(true,true)` and adds the simulation-only identity static TF
`body→base_link`), plus `play_bag` / `shutdown_gate` / `shutdown_gate_timeout_s` for
caller-owned playback. All four runners honour **`SIM_INSTALL_SETUP`** (path to the
`install/setup.bash` they source; default = this repository's own `install/setup.bash`), so a
build kept in another directory cannot be silently overridden by the repository install.

```bash
# 1) generate the synthetic bag (writes <out>/bag + the scene reference), no ROS needed
python3 simulation/synthetic_data/generate_test_bag.py --out /tmp/sim_bag
python3 simulation/synthetic_data/generate_test_bag.py --check          # fails closed on model defects

# 2) mapping bring-up (current source, no hardware)
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
ros2 launch simulation/launch/sim_fastlio2.launch.py bag_path:=/tmp/sim_bag \
     work_dir:=/tmp/sim_map rviz:=false

# 3) localization bring-up (real localizer_node on a frozen prior map; launches the real consumer)
ros2 launch simulation/launch/sim_localization.launch.py bag_path:=/tmp/sim_bag \
     work_dir:=/tmp/sim_loc prior_map:=/tmp/sim_ref/baseline/mapping/frozen_map.pcd \
     localizer_config:=/tmp/sim_loc/loc_sim.yaml consumer_mode:=predict
# consumer_mode:=stamp (truthful resolved stamp) | current (strict current-pose) | predict (bounded
# prediction, stamp == query clock).  The launch starts exactly ONE map_pose_publisher; do NOT
# start a second one by hand (the runner's readiness probe requires one subscriber per topic).

# 4) consumer recording for the T3 criterion (20 Hz census + query/status join)
python3 simulation/scripts/record_consumer.py --out-dir /tmp/sim_loc --duration 90 --pose-mode predict
#   --pose-mode must equal the launch's consumer_mode; the runner starts it BEFORE playback and
#   waits for <run-dir>/consumer_ready.json.  record_localization.py owns the operator relocalize
#   call (declared start pose) that seeds the first ICP.
```

The four orchestrated runners (`run_mapping_sim.sh`, `run_localization_sim.sh`,
`run_eval_pipeline.sh`, `run_all_tests.sh`) wrap these with bounded wall clocks, PID-scoped cleanup,
manifests and hashes; see the command block below.

### 13.1 Runnable now (no hardware)

Run everything from `fork/`; each ROS run needs its **own** `ROS_DOMAIN_ID` with
`ROS_LOCALHOST_ONLY=1` (two agents in one domain cross-contaminate silently).

```bash
# Full synthetic pipeline (regenerates bag + reference + T1/T2/T3); variants run SEQUENTIALLY
SIM_DOMAIN_ID=137 bash simulation/scripts/run_eval_pipeline.sh --out-root /tmp/sim_fixed3 \
     --variants baseline --with-trajectory-diagnostic --domain-id 137
# ... --dry-run prints stages only; --skip-localization / --skip-mapping narrow the run.

# Single bounded runs
bash simulation/scripts/run_mapping_sim.sh --run-dir /tmp/sim_map [--extra-config simulation/ablations/c2_1_acc_normalize.yaml]
bash simulation/scripts/run_localization_sim.sh --run-dir /tmp/sim_loc --prior-map /tmp/sim_fixed3/baseline/mapping/frozen_map.pcd \
     --consumer-mode predict --domain-id 137        # stamp|current|predict; starts the recorder BEFORE playback
#   ... --fault {stop_localizer|stop_odom|clock_pause|clock_jumpback} --fault-at-s 25 injects one fault
#   ... --skip-bag reuses an existing --bag-dir (same bag for a three-mode comparison)
#   SIM_INSTALL_SETUP=<path>/install/setup.bash selects the build overlaid by every runner

# Metric tests (0 PASS, 1 FAIL, 2 BLOCKED)
python3 simulation/scripts/test_t1_accuracy.py --run-dir <run> --ref-dir <ref>
python3 simulation/scripts/test_t2_speed.py    --run-dir <run>
python3 simulation/scripts/test_t3_localization.py --run-dir <run> --ref-dir <ref>
# or all three, using the recorded artifacts only:
bash simulation/scripts/run_all_tests.sh        # honours SIM_RUN_DIR / SIM_REF_DIR

# Bag time-contract check (behind the direct T2 measurement)
ROS_DOMAIN_ID=99 python3 simulation/scripts/check_bag_availability.py --bag <bag> --out /tmp/bag_availability.json

# Model self-check: fails closed on any kinematic discontinuity (both routes)
python3 simulation/synthetic_data/generate_test_bag.py --check
python3 simulation/synthetic_data/generate_test_bag.py --check \
     --trajectory simulation/synthetic_data/test_trajectory_localization.json

# Diagnostics (read-only; see simulation/diagnostics/README.md for the full table)
python3 simulation/diagnostics/bag_imu_peaks.py
python3 simulation/diagnostics/frame_chain_conditioning.py <run-dir>/mapping
python3 simulation/diagnostics/residual_attribution.py <run-dir>/mapping <ref-dir>
python3 simulation/diagnostics/residual_vs_analytic_surface.py <run-dir>/mapping <ref-dir>
python3 simulation/diagnostics/evaluator_floor_check.py <ref-dir> /tmp/floor.json <run-dir>/mapping/frame_chain.json
python3 simulation/diagnostics/c2_3_sampling_phase.py
bash   simulation/diagnostics/run_seam_isolation.sh

# Persist evidence into the repo / re-score a finished run
bash simulation/scripts/persist_evidence.sh --out-root /tmp/sim_pipeline --dest fork/artifacts/simulation_<utc>
bash simulation/scripts/finalize_runs.sh   --out-root /tmp/sim_pipeline --dest fork/artifacts/simulation_final

# C1 controlled offline loop intervention (already executed)
bash artifacts/local_loop_intervention/run_intervention.sh both --save-map
```

Unit tests / build (do **not** run concurrently with another agent's build):

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j3 colcon build --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
colcon test --packages-select localizer
colcon test -R "test_imu_init|test_utils_preprocess"
```

### 13.2 Reproducible only on hardware (`【board-only】`)

1. Orin NX build: `colcon build` peak RSS / parallel workers; `-mcpu=cortex-a78ae` legality;
   GTSAM 4.2 aarch64 availability; JetPack/Ubuntu/Humble combination.
2. `body→base_link` non-zero-angle TF check on the full bringup (§9.3) and `map→odom` publisher
   uniqueness including any nav2/AMCL publisher.
3. Time domain: three-source clock consistency, out-of-order/drop rate, per-point time-field
   correctness for the target LiDAR (§9.4).
4. T1.2 P1–P7 and T1.4 Q1–Q5 (§9.6), including the independent qualification of the leg-kinematics
   velocity observation before any fusion.
5. Quadruped dynamics: init duration and the `v=0` assumption error when starting in motion; static
   window reachability under vibration; deskew effectiveness at high dynamics.
6. Resources: end-to-end per-frame time distribution, RSS, temperature/throttling, the actual
   scheduling-priority binding (`chrt -p <pid>` / `/proc/<pid>/sched` policy).
7. Calibration: measured `ext_il` and `body_base_{x,y,z,roll,pitch,yaw}`.
8. Camera: `image_topic` wiring, intrinsics/extrinsics, colouring correctness.
9. End-to-end real-time on Orin — desktop replay must not stand in for whole-vehicle acceptance.

---

## 14. Limits of this delivery (no grand claim)

* **Local simulation only.** Synthetic scene/GT/reference are generated from one scene definition;
  one host, one deterministic seed, one scene. The T1 PASS, the T2 latency and the C2 arms carry
  that scope; the T3 accuracy carries the additional "separate route + frozen prior + declared
  GT-assisted initial pose" scope.
* **The historical dataset acceptance is `BLOCKED`**, for its own reasons (no independent coordinate
  chain / authoritative ROI / sourced independent transform; the `correction`-gate bottleneck and
  the front-end drift root cause remain hypotheses). No ratio ("N× better/worse") claim is made.
* **The localization current-clock requirement is NOT MET.** In the final, review-passed
  implementation the real `/robot_pose_map` consumer **meets the user's published-stamp criterion**
  (default 0.0191 m; opt-in strict mode 0.0223 m, n = 1082) but its **current-clock error is
  0.0591 m (> 5 cm)** in the opt-in mode (run 1 was 0.0457 m) and **0.1625 m** in the default mode.
  The mode never extrapolates past the odometry's own stamp, and it publishes only 1013/1459 ticks
  (69.4 %). Do not present a current-clock pass. The delayed/surrogate numbers (0.0284 m,
  0.0223 m) are diagnostics only; the GT-assisted initial pose and single route travel with every
  T3 number. Not hardware-validated.
* **The availability protocol fails** (V-4) with a measurable latency cause and its 0.1 s contract
  is infeasible with this stack (min achievable age ~0.2 s); the 95 % gate is a protocol choice,
  not the user's requirement. **This is not a hardware excuse** — it is measured on this host.
* **No hardware claim of any kind.** Every hardware item is `【board-only】`; no calibration value,
  no Orin latency/temperature, no real-robot behaviour is asserted.
* `[inference]`/`[conditional]`/`[blocked]` values are labelled as such and must not be promoted.

---

*End of local-simulation acceptance report. Machine-readable deliverable index:
`fork/artifacts/nonhardware_delivery/manifest.json`.*
