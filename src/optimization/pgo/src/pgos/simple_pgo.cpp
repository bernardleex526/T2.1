#include "simple_pgo.h"
#include <cstdio>
#include <memory>
#include <unordered_set>

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    if (m_config.sc.enabled)
        m_sc_db = std::make_unique<pgo_loop::ScanContextDB>(m_config.sc);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void SimplePGO::searchForLoopPairs()
{
    // ---------------------------------------------------------------------------------
    // 0. Descriptor maintenance.  Built for EVERY keyframe, unconditionally and before any
    //    early return, so that descriptor index == keyframe index: the candidate index the
    //    query returns is used directly as a keyframe index.
    // ---------------------------------------------------------------------------------
    double desc_ms = 0.0;
    if (m_sc_db && !m_key_poses.empty())
    {
        m_sc_db->addKeyframe(m_key_poses.back().body_cloud, &desc_ms);
        m_fe.t_desc_ms += desc_ms;
    }

    if (m_key_poses.size() < 10)
        return;
    const size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();

    // Unchanged temporal rate limit: at most one detection event per min_loop_detect_duration.
    bool rate_limited = false;
    if (m_config.min_loop_detect_duration > 0.0 && m_history_pairs.size() > 0)
    {
        double last_time = m_key_poses[m_history_pairs.back().second].time;
        rate_limited = (last_item.time - last_time) < m_config.min_loop_detect_duration;
    }
    if (rate_limited)
    {
        printFrontEndLine(cur_idx, "rate_limited", 0, 0, 0, 0, 0, 0, -1.0, desc_ms, 0.0);
        return;
    }
    m_fe.events++;

    // ---------------------------------------------------------------------------------
    // 1. Candidate generation
    // ---------------------------------------------------------------------------------
    struct Candidate
    {
        int idx;
        double sc_dist;
        double yaw_rad;
        bool have_yaw;
        const char *detector;
    };
    std::vector<Candidate> candidates;
    std::unordered_set<int> seen;

    // --- detector A: Scan Context global descriptor retrieval -------------------------
    size_t sc_hits = 0, sc_passing = 0;
    double sc_best = -1.0, query_ms = 0.0;
    if (m_sc_db)
    {
        std::vector<pgo_loop::DescriptorHit> hits = m_sc_db->query(&query_ms);
        m_fe.t_query_ms += query_ms;
        sc_hits = hits.size();
        for (size_t i = 0; i < hits.size(); i++)
        {
            const pgo_loop::DescriptorHit &h = hits[i];
            if (sc_best < 0.0 || h.dist < sc_best)
                sc_best = h.dist;
            if (h.dist > m_config.sc.dist_thresh)
                continue;
            sc_passing++;
            if (seen.insert(h.idx).second)
                candidates.push_back({h.idx, h.dist, h.yaw_rad, true, "scan_context"});
        }
    }
    const size_t n_sc_proposed = candidates.size();

    // --- detector B: Euclidean radius search over keyframe positions (fallback) -------
    size_t n_radius_proposed = 0;
    if (m_config.loop_enable_radius_search)
    {
        pcl::PointXYZ last_pose_pt;
        last_pose_pt.x = last_item.t_global(0);
        last_pose_pt.y = last_item.t_global(1);
        last_pose_pt.z = last_item.t_global(2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < m_key_poses.size() - 1; i++)
        {
            pcl::PointXYZ pt;
            pt.x = m_key_poses[i].t_global(0);
            pt.y = m_key_poses[i].t_global(1);
            pt.z = m_key_poses[i].t_global(2);
            key_poses_cloud->push_back(pt);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(key_poses_cloud);
        std::vector<int> ids;
        std::vector<float> sqdists;
        int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
        // Preserved legacy behaviour: take the nearest temporally-distant keyframe inside the
        // radius, and never overwrite a Scan Context candidate for the same keyframe.
        for (int i = 0; i < neighbors; i++)
        {
            int idx = ids[i];
            if (std::abs(last_item.time - m_key_poses[idx].time) <= m_config.loop_time_tresh)
                continue;
            n_radius_proposed++;
            if (seen.insert(idx).second)
                candidates.push_back({idx, -1.0, 0.0, false, "radius_search"});
            break;
        }
    }

    m_fe.proposed += candidates.size();

    // ---------------------------------------------------------------------------------
    // 2. Registration cascade over the candidates, best descriptor distance first
    // ---------------------------------------------------------------------------------
    int evaluated = 0, accepted = 0, rejected = 0, temporal = 0;
    for (size_t ci = 0; ci < candidates.size(); ci++)
    {
        const Candidate &c = candidates[ci];
        if (evaluated >= m_config.max_loop_candidates_per_query)
            break;
        if (accepted >= m_config.max_accepted_loops_per_query)
            break;
        const double dt = std::abs(last_item.time - m_key_poses[c.idx].time);
        if (!(dt > m_config.loop_time_tresh))
        {
            temporal++;
            printf("[PGO][gate] kf=%zu cand=%d det=%s sc_dist=%.4f REJECT(temporal_gap=%.1fs <= %.1fs)\n",
                   cur_idx, c.idx, c.detector, c.sc_dist, dt, m_config.loop_time_tresh);
            continue;
        }

        const KeyPoseWithCloud &target_item = m_key_poses[c.idx];
        pcl::PointCloud<PointType>::Ptr target_cloud =
            getSubMap(c.idx, m_config.loop_submap_half_range, m_config.submap_resolution);
        pcl::PointCloud<PointType>::Ptr source_cloud =
            getSubMap(static_cast<int>(cur_idx), m_config.loop_source_submap_half_range,
                      m_config.submap_resolution);

        // ---- initial guess ----
        // The descriptor yaw is a statement about the RELATIVE body rotation R_target_source
        // (verified with ground truth in test/test_scan_context_yaw.cpp).  Build the world-frame
        // delta that maps the source cloud onto the target cloud:
        //     dT_world = T_w_target * T_target_source_init * T_w_source^-1
        // with the rotation of T_target_source_init taken from the descriptor and its
        // translation from the current odometry estimate (a drifted but bounded prior).
        M4D init_guess = M4D::Identity();
        double prior_yaw_deg = 0.0, init_yaw_deg = 0.0;
        {
            const M3D R_s = last_item.r_global, R_t = target_item.r_global;
            const V3D t_s = last_item.t_global, t_t = target_item.t_global;
            const M3D R_ts_odom = R_t.transpose() * R_s;
            const V3D t_ts_odom = R_t.transpose() * (t_s - t_t);
            const Eigen::Vector3d ypr = R_ts_odom.eulerAngles(2, 1, 0); // ZYX
            prior_yaw_deg = ypr(0) * 180.0 / M_PI;
            // A radius-search candidate carries no yaw information, so it reuses the odometry
            // yaw; the reconstructed rotation is then R_ts_odom itself and the delta above
            // collapses to exactly identity, i.e. the legacy behaviour, no regression.
            const double yaw = c.have_yaw ? c.yaw_rad : ypr(0);
            init_yaw_deg = yaw * 180.0 / M_PI;
            const M3D R_ts = (Eigen::AngleAxisd(yaw, V3D::UnitZ()) *
                              Eigen::AngleAxisd(ypr(1), V3D::UnitY()) *
                              Eigen::AngleAxisd(ypr(2), V3D::UnitX())).toRotationMatrix();
            Eigen::Affine3d T_ws(R_s); T_ws.translation() = t_s;
            Eigen::Affine3d T_wt(R_t); T_wt.translation() = t_t;
            Eigen::Affine3d T_ts(R_ts); T_ts.translation() = t_ts_odom;
            init_guess = (T_wt * T_ts * T_ws.inverse()).matrix();
        }

        pgo_loop::RegistrationResult r =
            pgo_loop::runRegistrationCascade(target_cloud, source_cloud, init_guess,
                                             m_config.reg, m_config.gate);
        m_fe.t_coarse_ms += r.ms.coarse_ms;
        m_fe.t_fine_ms += r.ms.fine_ms;
        m_fe.t_gates_ms += r.ms.gates_ms;
        evaluated++;

        if (r.accepted)
        {
            LoopPair one_pair;
            one_pair.source_id = cur_idx;
            one_pair.target_id = c.idx;
            one_pair.score = r.fine_rmse_m; // metres (legacy field: used for logging only)
            const M3D R_delta = r.fine_transform.block<3, 3>(0, 0);
            const V3D t_delta = r.fine_transform.block<3, 1>(0, 3);
            M3D r_refined = R_delta * m_key_poses[cur_idx].r_global;
            V3D t_refined = R_delta * m_key_poses[cur_idx].t_global + t_delta;
            one_pair.r_offset = target_item.r_global.transpose() * r_refined;
            one_pair.t_offset = target_item.r_global.transpose() * (t_refined - target_item.t_global);
            m_cache_pairs.push_back(one_pair);
            m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
            accepted++;
            m_fe.accepted++;
        }
        else
        {
            rejected++;
            m_fe.rejected++;
        }

        char decision[128];
        if (r.accepted)
            snprintf(decision, sizeof(decision), "ACCEPT");
        else
            snprintf(decision, sizeof(decision), "REJECT(%s)", r.reject.c_str());

        printf("[PGO][gate] kf=%zu cand=%d det=%s sc_dist=%.4f sc_yaw_deg=%.2f prior_yaw_deg=%.2f "
               "init_yaw_deg=%.2f | coarse conv=%d rmse=%.4fm (max %.3f) | fine conv=%d rmse=%.4fm "
               "(max %.3f) p2pl=%.4fm | overlap=%.3f@%.2fm (%.3f@%.2fm %.3f@%.2fm) n_corr=%zu/%zu | "
               "eig=[%.4f %.4f %.4f] ratio=%.5f (min %.5f) norm=%.3e | ms[desc=%.2f query=%.2f "
               "coarse=%.1f fine=%.1f gates=%.1f] | %s\n",
               cur_idx, c.idx, c.detector, c.sc_dist,
               c.have_yaw ? c.yaw_rad * 180.0 / M_PI : 0.0, prior_yaw_deg, init_yaw_deg,
               r.coarse_converged ? 1 : 0, r.coarse_rmse_m, m_config.reg.coarse_max_rmse_m,
               r.fine_converged ? 1 : 0, r.fine_rmse_m, m_config.reg.fine_max_rmse_m,
               r.fine_plane_rmse_m,
               r.overlap, m_config.gate.overlap_radius_m,
               r.overlap_2, m_config.gate.overlap_radius_2_m,
               r.overlap_3, m_config.gate.overlap_radius_3_m,
               r.n_corr, r.n_source,
               r.eig[0], r.eig[1], r.eig[2], r.eig_ratio, m_config.gate.min_eig_ratio,
               r.eig_min_norm,
               desc_ms, query_ms, r.ms.coarse_ms, r.ms.fine_ms, r.ms.gates_ms,
               decision);
    }
    m_fe.evaluated += evaluated;
    m_fe.temporal += temporal;

    printFrontEndLine(cur_idx, "ok", sc_hits, sc_passing, candidates.size(),
                      n_radius_proposed, evaluated, accepted, sc_best, desc_ms, query_ms);
    (void)n_sc_proposed;
}

void SimplePGO::printFrontEndLine(size_t cur_idx, const char *status, size_t sc_hits,
                                  size_t sc_passing, size_t proposed, size_t radius_proposed,
                                  int evaluated, int accepted, double best_dist, double desc_ms,
                                  double query_ms)
{
    printf("[PGO][sc] kf=%zu db=%zu status=%s desc_build_ms=%.3f query_ms=%.3f hits=%zu passing=%zu "
           "proposed=%zu radius_proposed=%zu evaluated=%d accepted=%d best_dist=%.4f thresh=%.4f "
           "| totals: desc_build=%.1fms query=%.1fms coarse=%.1fms fine=%.1fms gates=%.1fms "
           "events=%ld proposed=%ld temporal=%ld evaluated=%ld accepted=%ld rejected=%ld\n",
           cur_idx, m_sc_db ? m_sc_db->size() : 0, status, desc_ms, query_ms, sc_hits, sc_passing,
           proposed, radius_proposed, evaluated, accepted, best_dist, m_config.sc.dist_thresh,
           m_fe.t_desc_ms, m_fe.t_query_ms, m_fe.t_coarse_ms, m_fe.t_fine_ms, m_fe.t_gates_ms,
           m_fe.events, m_fe.proposed, m_fe.temporal, m_fe.evaluated, m_fe.accepted, m_fe.rejected);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        // D2 fix. The old code used
        //   noiseModel::Diagonal::Variances(Vector6::Ones() * pair.score)
        // where pair.score is the PCL ICP fitness score = mean SQUARED point-to-point
        // residual [m^2] (<= loop_score_tresh = 0.15). Two problems:
        //   1) it is not a pose variance: the pose error of an ICP alignment over a whole
        //      submap is far smaller than the per-point RMS residual, so it over-estimated
        //      the loop uncertainty by orders of magnitude; and
        //   2) with the odometry factors at variance 1e-6 the resulting loop factors were
        //      1e4..1e5x weaker, so gtsam/ISAM2 effectively ignored every loop (measured:
        //      the trajectory moved by <= 0.0994 m on a 2.01 m loop inconsistency).
        // New model: a fixed variance floor from config (loop_noise_xyz / loop_noise_rpy)
        // wrapped in a Huber robust kernel, so a good loop really pulls while a single
        // grossly wrong ICP loop is down-weighted instead of dragging the whole graph.
        const double var_xyz = m_config.loop_noise_xyz * m_config.loop_noise_xyz;
        const double var_rpy = m_config.loop_noise_rpy * m_config.loop_noise_rpy;
        gtsam::noiseModel::Diagonal::shared_ptr loop_base_noise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << var_rpy, var_rpy, var_rpy, var_xyz, var_xyz, var_xyz).finished());
        gtsam::noiseModel::Robust::shared_ptr loop_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(m_config.loop_robust_k), loop_base_noise);

        for (LoopPair &pair : m_cache_pairs)
        {
            // pair.score is the FINE-stage RMSE [m] of the accepted pair (it is NOT used as a
            // variance any more; the noise model below is a fixed variance floor).
            printf("[PGO][loop] id %zu <-> %zu  fine_rmse_m %.4f  var_xyz %.3e var_rpy %.3e huber_k %.2f\n",
                   pair.source_id, pair.target_id, pair.score, var_xyz, var_rpy, m_config.loop_robust_k);
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           loop_noise));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}