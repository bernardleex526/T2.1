#pragma once
// ---------------------------------------------------------------------------
// STEP 1+2 loop-closure front end for the pgo package.
//
//   Scan Context global descriptor retrieval  ->  coarse registration with the
//   descriptor's relative yaw as the initial guess (fast_gicp)  ->  fine
//   point-to-plane ICP  ->  multi-gate acceptance (fitness, overlap ratio,
//   eigenvalue degeneracy).
//
// Why: the previous front end proposed candidates by Euclidean radius over the
// drifted keyframe positions and registered them with plain point-to-point PCL
// ICP starting from identity.  With several metres of drift that ICP converges
// to a drifted local minimum which still passes loop_score_tresh (measured on
// mid360s_office_loop_01: the true revisit kf358<->kf61 has a 2.042 m position
// discrepancy, ICP "converged" at fitness 0.133 and the loop got *worse* after
// PGO).  The descriptor is scale/rotation-aware and yaw-aligned, so it proposes
// the revisit even when the drifted positions are far apart, and the descriptor
// yaw puts the coarse stage inside its convergence basin.
// ---------------------------------------------------------------------------

#include "commons.h"

#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

using M4D = Eigen::Matrix4d;

class SCManager; // vendored third-party descriptor (src/scan_context/Scancontext.h)

namespace pgo_loop
{

// ---------------------------------------------------------------------------
// Scan Context detector
// ---------------------------------------------------------------------------
struct ScanContextConfig
{
    bool enabled = true;
    // Descriptor geometry (Scan Context paper, IROS 2018 defaults).
    int num_ring = 20;            // radial bins
    int num_sector = 60;          // azimuth bins -> yaw quantisation = 360/60 = 6 deg
    double max_radius_m = 80.0;   // points beyond this radius are dropped from the descriptor
    double lidar_height_m = 2.0;  // constant added to z before the per-bin max (see Scancontext.h)
    double downsample_resolution_m = 0.2; // VoxelGrid applied to the keyframe body cloud first
    // Retrieval
    double dist_thresh = 0.30;    // cosine descriptor distance gate (paper eq. 6 "D"); 0.1-0.2 is
                                  // the "no false alarm" range without a downstream ICP check,
                                  // 0.3-0.6 is what upstream SC-LIO-SAM uses *with* one
    int num_candidates = 3;       // nearest ringkey neighbours to re-rank per query
    int exclude_recent = 30;      // never propose one of the last N keyframes
};

// One descriptor-database hit.
struct DescriptorHit
{
    int idx = -1;         // keyframe index
    double dist = 0.0;    // Scan Context cosine distance (unitless, 0 = identical)
    double yaw_rad = 0.0; // relative yaw of R_target_source (see relativeYawRad)
};

class ScanContextDB
{
public:
    explicit ScanContextDB(const ScanContextConfig &cfg);
    // Defined in the .cpp: SCManager is an incomplete type here (vendored header, do not leak).
    ~ScanContextDB();

    // Build + store the descriptor of one keyframe body cloud.  MUST be called once per
    // keyframe, in keyframe order, so that descriptor index == keyframe index.
    void addKeyframe(const CloudType::Ptr &body_cloud, double *build_ms = nullptr);

    size_t size() const;

    // Ringkey nearest-neighbour query.  Returns the best `num_candidates` hits ranked by
    // Scan Context descriptor distance, excluding the most recent `exclude_recent`
    // keyframes.  `query_ms` receives the wall time of tree build + search + re-ranking.
    std::vector<DescriptorHit> query(double *query_ms = nullptr);

    // Relative yaw [rad] of R_target_source for the most recently added (source) keyframe
    // against descriptor `target_idx`: the rotation that maps source body coordinates into
    // target body coordinates.  Sign convention pinned by test/test_scan_context_yaw.cpp.
    double relativeYawRad(int target_idx);

    const ScanContextConfig &config() const { return m_cfg; }

private:
    ScanContextConfig m_cfg;
    std::unique_ptr<SCManager> m_sc;
};

// ---------------------------------------------------------------------------
// Registration cascade
// ---------------------------------------------------------------------------
struct RegistrationConfig
{
    // Stage A: coarse, wide correspondence distance, aggressively downsampled, GICP.
    double coarse_voxel_resolution_m = 0.5;
    double coarse_max_corr_dist_m = 3.0;   // must exceed the residual drift left AFTER the
                                           // descriptor yaw is applied; 6 deg quantisation at a
                                           // 20 m lever arm is 2.1 m, so 3.0 m is the floor
    int coarse_max_iterations = 64;
    // STEP 3: the coarse stage runs TWO passes.  Pass 1 uses this wide correspondence
    // distance so that a seed whose translation is wrong by up to ~1/2 of it can still walk
    // to the right minimum (measured on mid360s_office_loop_01: seeding a genuine revisit
    // with the odometry relative pose puts the clouds 2-4.8 m apart, which a single 3.0 m
    // pass cannot fully absorb); pass 2 re-runs at coarse_max_corr_dist_m from pass 1's
    // result.  Values below coarse_max_corr_dist_m are ignored (pass 1 == pass 2).
    double coarse_wide_corr_dist_m = 5.0;
    double coarse_max_rmse_m = 0.35;       // sqrt() of the PCL fitness score, i.e. metres
    // Stage B: fine, point-to-plane ICP.
    double fine_voxel_resolution_m = 0.10;
    double fine_max_corr_dist_m = 0.50;
    int fine_max_iterations = 60;
    // Acceptance residuals, both in metres (PCL's fitness score is a mean SQUARED distance, so
    // sqrt() of it is the RMS nearest-neighbour distance):
    //   fine_max_rmse_m       - over ALL source points.  Loose by design, and equivalent in
    //                           meaning to the legacy loop_score_tresh = 0.15 m^2 (= 0.387 m
    //                           RMS): source points whose geometry the target submap simply does
    //                           not contain (different field of view) are expected and would
    //                           otherwise dominate the mean.  Measured on true co-located scan
    //                           pairs of the MID-360S office loop it is 0.155-0.40 m, so it
    //                           cannot discriminate on its own.
    //   fine_max_plane_rmse_m - point-to-plane RMS over the INLIER correspondences only, i.e.
    //                           "how far off the surfaces are where the two scans do overlap".
    //                           This is the discriminating residual; unlike a point-to-point
    //                           inlier residual it is not bounded by the overlap radius, and
    //                           unlike the all-points one it ignores the uncovered geometry.
    double fine_max_rmse_m = 0.35;
    double fine_max_plane_rmse_m = 0.05;
    double normal_search_radius_m = 0.30;  // normal estimation radius at fine resolution
    int correspondence_randomness = 10;    // fast_gicp knn for covariance estimation
};

struct GateConfig
{
    // Overlap: fraction of the (fine) source points that have a target point within
    // overlap_radius_m after registration.  A loop between two scans of the same place
    // shares most of its geometry; a loop between two different places does not.
    double overlap_radius_m = 0.10;
    double min_overlap_ratio = 0.35;
    // Diagnostic overlap at two extra radii, logged only (never gated).
    double overlap_radius_2_m = 0.20;
    double overlap_radius_3_m = 0.50;
    // Degeneracy: H = sum_i n_i n_i^T over the accepted correspondences, n_i the target
    // normal.  H is the information matrix of the point-to-plane residual; a direction
    // that no normal spans (a long corridor: wall normals are perpendicular to the
    // corridor axis, so its eigenvalue is ~0) is unconstrained by the geometry and the
    // registration can slide arbitrarily along it while still converging.  Gate on
    // lambda_min / lambda_max.
    bool degeneracy_gate_enabled = true;
    double min_eig_ratio = 0.003;   // measured basis: see config/pgo.yaml
    // Plausibility gate on the CORRECTION a candidate demands.  A loop exists to remove drift,
    // and the drift a platform can have accumulated between two visits is bounded by its own
    // drift budget.  Measured on this sequence: the raw LIO says the two keyframes are X apart,
    // registration measures rel_t, and |X - rel_t| is what the graph is being asked to absorb.
    // A candidate demanding far more correction than the trajectory's own end-of-loop drift
    // (5.87 m over 134 m here) is far more likely to be a perceptual alias (the same office
    // corridor repeating every few metres) than a genuine loop, so it is rejected.
    // Applied to both detectors; for the radius detector it is nearly vacuous, because those
    // candidates never assert co-location and the registration stays at the odometry prior.
    // STEP 4 CALIBRATION (after the fastlio2 point-quality fix, HILTI ATE 3.52 m -> 0.17 m):
    // the old floor (2.0 m) and ratio (5 %/m) were tuned for the DRIFTING frontend; with the
    // corrected one the measured drift is ~0.2-0.35 % of path (HILTI 0.17 m / ~50 m,
    // TIERS 2.4-2.9 cm), and enforcing six sub-noise "corrections" of 0-0.7 m on
    // tiers_indoor_office2 (all revisits of the start region) degraded ATE from 2.89 to
    // 3.50 cm.  The bound is also SYMMETRIC now: a registration that STRETCHES the pair
    // (negative correction) is as implausible as one that shrinks it.
    double max_loop_correction_m = 0.5;
    // STEP 3: the correction bound above is a FLOOR.  The bound actually applied is
    //   max(max_loop_correction_m, correction_drift_ratio * travelled_path_m)
    // where travelled_path_m is the cumulative raw-odometry path length between the two
    // keyframes of the pair.  Rationale: drift grows with distance travelled, so a fixed
    // absolute bound is either too tight for long loops (a genuine revisit after 120 m of
    // driving legitimately carries a ~2-3 m correction - measured on
    // mid360s_office_loop_01: kf357<->kf70 demands 3.26 m after 154 s / ~118 m of travel)
    // or too loose for short ones.  0.01 = 1 % of path is ~3x the measured corrected-LIO
    // drift (0.34 % on HILTI exp18) while every measured perceptual alias of
    // mid360s_office_loop_01 demands 6-25 m (7-19 % of its own inter-keyframe path).
    double correction_drift_ratio = 0.01;
    // STEP 3: revisit-consistency gate (scan_context candidates only).  A scan-context
    // candidate asserts CO-LOCATION: the registration seeds it with zero relative
    // translation, so a genuine revisit must come out with a small measured relative
    // translation (two keyframes ~0.5-0.7 m apart that revisited each other are at most a
    // few metres apart physically - measured on the genuine family of this sequence:
    // rel_t 0.38-2.43 m).  A candidate the registration instead pulled 7-10 m away
    // (kf217<->kf80: accepted by every geometric gate, rel_t 7.06 m - the coarse GICP slid
    // along repeating corridor geometry until it agreed with the odometry separation) is a
    // FALSE loop whose factor happens to coincide with odometry; reject it explicitly.
    double max_revisit_rel_t_m = 4.0;
    // STEP 3: minimum measured CORRECTION (in absolute value) for ANY accepted loop, from
    // either seed.  A candidate whose registration result agrees with the odometry to
    // within this bound is an odometry NO-OP: the factor would only re-enforce what
    // odometry already says, but at an ICP measurement error far above the loop's 1 cm
    // noise model, and enforcing that error measurably degrades the graph (measured on
    // tiers_indoor_office2_mid360, run loop-t2: six accepted no-op corrections of 0.0-0.27
    // m on mocap-grade odometry degraded ATE from 2.89 to 3.50 cm).  A loop factor is only
    // worth adding when registration claims a correction larger than the ICP measurement
    // noise of co-located submaps (measured ~0.1 m).
    double min_odo_correction_m = 0.15;
    // STEP 3: cross-seed agreement.  The two seeds are two INDEPENDENT hypotheses about
    // where the revisit is: co-located + descriptor yaw, and the odometry relative pose.
    // When BOTH registrations converge with corroborating overlap, a genuine revisit must
    // measure the SAME relative translation from both (the surfaces attract to one pose);
    // a disagreement means at least one of them is a false minimum and neither can be
    // trusted (measured on mid360s_office_loop_01, run loop-b2: kf364<->kf77 - co-located
    // seed converges to rel_t 4.49 m, odometry-prior seed to 2.68 m, 1.81 m apart; the
    // odometry-prior seed "measured a 2.17 m correction" that the other hypothesis
    // contradicts).  Applied only when both seeds are informative (fine converged AND
    // overlap >= min_overlap_ratio); a seed that did not converge (e.g. garbage descriptor
    // yaw) carries no evidence either way and does not veto the other.
    double cross_seed_max_m = 1.0;
    // STEP 3: z-plausibility of the loop factor translation.  The z component of the
    // measured relative translation must stay within this bound.  Basis: an indoor
    // revisit of a floor-mounted sensor happens at (nearly) the same height, and the z
    // drift of the odometry over a few hundred seconds is small (measured on
    // mid360s_office_loop_01: 0.51 m over the whole 266 s run) - yet ICP can slide along
    // an ambiguously-supported floor/wall and claim a > 1 m z step (measured: run loop-b4,
    // accepted loop kf368<->kf53 claimed t_offset.z = 1.13 m, deformed the end of the
    // trajectory and WORSENED the map thickness from 0.0109 to 0.0163 m mean).  0.5 m is
    // ~2x the sequence's whole-loop z drift.  Set from your platform's z-drift behaviour.
    double max_loop_z_offset_m = 0.5;
    // Yaw-consistency gate.  The MEASURED relative yaw of the accepted registration (the
    // final fine-stage rotation about Z, expressed in the target keyframe's body frame) and
    // the odometry's relative yaw are two independent estimates of the same quantity; they
    // agree up to the yaw drift accumulated between the two visits (a couple of degrees on
    // an office loop).  A perceptual alias that slides along repeating corridor geometry has
    // an arbitrary measured relative yaw.
    // STEP 3 NOTE: the pre-STEP-3 form compared the DESCRIPTOR yaw against odometry.  That
    // is unsound for revisits approached from a different direction: on this sequence the
    // descriptor yaw of the genuine end-of-loop family disagrees with odometry by 37-178 deg
    // (SC's sector argmax is ambiguous under the ~90-180 deg heading changes of an office
    // loop), so the old gate rejected exactly the loops that should close.  The descriptor
    // yaw now only SEEDS the coarse stage; the gate is applied to what the registration
    // actually measured.  Measured on this sequence: the accepted genuine loop
    // kf370<->kf55 measures 96.5 deg vs odometry 99.5 deg (3.5 deg apart); the corridor-slide
    // false loop kf217<->kf80 measures 6.5 deg vs odometry 6.5 deg (it slid back onto the
    // odometry prior and is caught by the revisit_offset gate instead).
    // ASSUMPTION: the odometry's yaw drift between the two visits stays below this threshold.
    double max_yaw_disagreement_deg = 15.0;
};

struct StageTiming
{
    double desc_build_ms = 0.0; // descriptor build for the current keyframe (once per keyframe)
    double query_ms = 0.0;      // descriptor DB query (once per detection event)
    double coarse_ms = 0.0;
    double fine_ms = 0.0;
    double gates_ms = 0.0;
};

struct RegistrationResult
{
    bool accepted = false;
    std::string reject;              // comma-separated list of failed gates, empty iff accepted
    bool coarse_converged = false;
    double coarse_rmse_m = -1.0;     // sqrt(PCL fitness score of the coarse stage)  [m]
    bool fine_converged = false;
    double fine_rmse_m = -1.0;       // sqrt(PCL fitness score of the fine stage)    [m]
    double fine_plane_rmse_m = -1.0; // point-to-plane RMSE over the in-radius correspondences [m]
    double inlier_rmse_m = -1.0;     // point-to-point RMSE over the in-radius correspondences [m]
    size_t n_source = 0;             // points in the fine source cloud after downsampling
    size_t n_corr = 0;               // source points with a target point within overlap_radius_m
    double overlap = -1.0;
    double overlap_2 = -1.0;
    double overlap_3 = -1.0;
    double eig[3] = {0.0, 0.0, 0.0}; // ascending eigenvalues of H
    double eig_ratio = -1.0;         // eig[0] / eig[2]
    double eig_min_norm = -1.0;      // eig[0] / n_corr
    M4D fine_transform = M4D::Identity();
    StageTiming ms;
};

// target_submap_world / source_cloud_world are clouds expressed in the sliding-window world
// frame the PGO uses; init_guess is the world-frame delta transform that maps the source onto
// the target (identity == "the current odometry poses are right").
RegistrationResult runRegistrationCascade(const CloudType::Ptr &target_submap_world,
                                          const CloudType::Ptr &source_cloud_world,
                                          const M4D &init_guess,
                                          const RegistrationConfig &rc,
                                          const GateConfig &gc);

} // namespace pgo_loop
