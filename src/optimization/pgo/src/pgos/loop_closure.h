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
    double coarse_max_rmse_m = 0.35;       // sqrt() of the PCL fitness score, i.e. metres
    // Stage B: fine, point-to-plane ICP.
    double fine_voxel_resolution_m = 0.10;
    double fine_max_corr_dist_m = 0.50;
    int fine_max_iterations = 60;
    double fine_max_rmse_m = 0.10;         // metres (PCL fitness score <= this^2)
    double normal_search_radius_m = 0.60;  // normal estimation radius at fine resolution
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
    double min_eig_ratio = 0.02;   // measured basis: see config/pgo.yaml
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
