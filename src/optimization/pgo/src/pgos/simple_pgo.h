#pragma once
#include "commons.h"
#include "pgos/loop_closure.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// D2 fix: noiseModel::Robust + mEstimator::Huber (LossFunctions.h is included by NoiseModel.h).
#include <gtsam/linear/NoiseModel.h>

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    // Cumulative raw-odometry path length up to this keyframe (immune to PGO corrections).
    // Used by the relative max-correction gate (GateConfig::correction_drift_ratio).
    double path_m = 0.0;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

namespace pgo_experiment
{
struct CandidateAudit
{
    size_t source_id = 0;
    size_t target_id = 0;
    std::string detector;
    std::string sel_seed;
    M3D r_offset = M3D::Identity();
    V3D t_offset = V3D::Zero();
    double rel_t = 0.0;
    double corr = 0.0;
    double corr_allowance = 0.0;
    double dyaw = 0.0;
    double prior_yaw = 0.0;
    double init_yaw = 0.0;
    double prior_t = 0.0;
    bool coarse_conv = false;
    double coarse_rmse = 0.0;
    bool fine_conv = false;
    double fine_rmse = 0.0;
    double fine_plane_rmse = 0.0;
    double inlier_rmse = 0.0;
    double overlap = 0.0;
    double overlap_2 = 0.0;
    double overlap_3 = 0.0;
    size_t n_corr = 0;
    size_t n_source = 0;
    double eig[3] = {0.0, 0.0, 0.0};
    double eig_ratio = 0.0;
    double eig_min_norm = 0.0;
    std::string seeds_summary;
    std::string fails;
    bool accepted = false;
};
} // namespace pgo_experiment

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 8.0; // D3: must exceed accumulated drift; mirrors pgo.yaml
    double loop_time_tresh = 60.0;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 2.0;

    // ===== STEP 1+2 front end =====
    // Two candidate detectors, either or both active:
    //   loop_enable_scan_context - Scan Context global descriptor retrieval. NOT bounded by
    //     loop_search_radius (that is the point: it must find revisits the drifted Euclidean
    //     search misses); bounded instead by sc.exclude_recent and loop_time_tresh.
    //   loop_enable_radius_search - the original Euclidean radius search over keyframe
    //     positions, kept as a fallback/complement.  Its candidates get no yaw prior because
    //     a drifted position carries no usable relative-yaw information.
    bool loop_enable_scan_context = true;
    bool loop_enable_radius_search = true;
    // Registrations actually attempted per detection event (after ranking by descriptor
    // distance).  Every evaluated candidate is logged; accepted loops additionally need a
    // free slot, capped by max_accepted_loops_per_query.
    int max_loop_candidates_per_query = 3;
    int max_accepted_loops_per_query = 1;
    int loop_source_submap_half_range = 2;
    pgo_loop::ScanContextConfig sc;
    pgo_loop::RegistrationConfig reg;
    pgo_loop::GateConfig gate;

    // ===== D2 fix: loop-closure noise model =====
    // Standard deviations (NOT variances) of the loop BetweenFactor, in SI units:
    //   loop_noise_xyz [m]   - translation std dev applied to x, y and z
    //   loop_noise_rpy [rad] - rotation std dev applied to roll, pitch and yaw
    // These replace the old `Vector6::Ones() * pair.score`, which mapped the ICP fitness
    // score (a mean-SQUARED point-to-point residual in m^2) directly onto a 6-DoF pose
    // variance and so was 1e4..1e5x weaker than the 1e-6 odometry variances.
    double loop_noise_xyz = 0.01;   // 1 cm std dev -> variance 1e-4
    double loop_noise_rpy = 0.005;  // 0.29 deg std dev -> variance 2.5e-5
    // Huber threshold expressed in whitened units (multiples of the loop std dev), i.e. a
    // residual larger than loop_robust_k * sigma is progressively down-weighted
    // (weight = k / |r|). This is what keeps a single wrong ICP loop from wrecking the graph.
    double loop_robust_k = 3.0;
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    // STEP 1+2 front-end accounting line (one per keyframe, see [PGO][sc] in the run log).
    void printFrontEndLine(size_t cur_idx, const char *status, size_t sc_hits, size_t sc_passing,
                           size_t proposed, size_t radius_proposed, int evaluated, int accepted,
                           double best_dist, double desc_ms, double query_ms);

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

    // ===== EXPERIMENT-ONLY INTERFACE (artifacts/local_loop_intervention) =====
    // 100% production defaults preserved when not explicitly enabled.
    void setExperimentForcedCandidate(size_t source_id, size_t target_id, bool enable = true)
    {
        m_exp_forced_source = source_id;
        m_exp_forced_target = target_id;
        m_exp_force_active = enable;
    }

    void injectLoopPairForExperiment(const LoopPair &pair)
    {
        m_cache_pairs.push_back(pair);
        m_history_pairs.emplace_back(pair.target_id, pair.source_id);
    }

    void setExperimentAuditRecording(bool enable = true)
    {
        m_exp_record_audits = enable;
    }

    const std::vector<pgo_experiment::CandidateAudit> &getExperimentAudits() const
    {
        return m_exp_audits;
    }

    void clearExperimentAudits()
    {
        m_exp_audits.clear();
    }

private:
    Config m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    // STEP 1+2: Scan Context descriptor database, index-aligned with m_key_poses.
    std::unique_ptr<pgo_loop::ScanContextDB> m_sc_db;
    // STEP 3: time of the last detection event that actually ran (event-based rate limit).
    double m_last_event_time = -1e300;
    // Cumulative front-end accounting, printed on every detection event so the run log carries
    // both the per-event decision and the run totals (see [PGO][sc] lines).
    struct FrontEndStats
    {
        long events = 0;        // detection events that passed min_loop_detect_duration
        long proposed = 0;      // loop candidates proposed by either detector (after dedup)
        long temporal = 0;      // ... of which dropped by the loop_time_tresh guard
        long evaluated = 0;     // candidates that actually went through the registration cascade
        long accepted = 0;      // ... and passed every gate
        long rejected = 0;      // ... and failed at least one gate
        double t_desc_ms = 0;   // Scan Context descriptor build (once per keyframe)
        double t_query_ms = 0;  // Scan Context DB query (once per detection event)
        double t_coarse_ms = 0;
        double t_fine_ms = 0;
        double t_gates_ms = 0;
    } m_fe;
    // Experiment-only state (inactive in production)
    bool m_exp_force_active = false;
    size_t m_exp_forced_source = 0;
    size_t m_exp_forced_target = 0;
    bool m_exp_record_audits = false;
    std::vector<pgo_experiment::CandidateAudit> m_exp_audits;
};