#include "loop_closure.h"

#include "scan_context/Scancontext.h"
#include "scan_context/KDTreeVectorOfVectorsAdaptor.h"

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

using Clock = std::chrono::steady_clock;

static inline double msSince(const Clock::time_point &t0)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

static CloudType::Ptr voxelDownsample(const CloudType::Ptr &in, double res)
{
    CloudType::Ptr out(new CloudType);
    if (!in || in->empty())
        return out;
    if (res <= 0.0)
    {
        *out = *in;
        return out;
    }
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(static_cast<float>(res), static_cast<float>(res), static_cast<float>(res));
    vg.setInputCloud(in);
    vg.filter(*out);
    return out;
}

// Downsample and attach normals (radius search).  The normals are what makes the fine stage a
// point-to-plane ICP and what the degeneracy gate forms its information matrix from.
// NOTE: pcl::concatenateFields cannot be used here - it iterates over the SOURCE fields and
// therefore tries to copy the intensity field into pcl::PointNormal, which has none.
static pcl::PointCloud<pcl::PointNormal>::Ptr voxelWithNormals(const CloudType::Ptr &in,
                                                               double res, double normal_radius)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
    CloudType::Ptr down = voxelDownsample(in, res);
    if (down->empty())
        return out;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
    ne.setNumberOfThreads(4); // the node already runs inside a 4-thread executor
    ne.setInputCloud(down);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);
    out->reserve(down->size());
    for (size_t i = 0; i < down->size() && i < normals->size(); i++)
    {
        // Points whose neighbourhood is too small get a non-finite normal; drop them instead of
        // letting them poison the normal equations (pcl::TransformationEstimationPointToPlaneLLS
        // does the same, see impl/transformation_estimation_point_to_plane_lls.hpp:185).
        if (!std::isfinite(normals->points[i].normal_x) ||
            !std::isfinite(normals->points[i].normal_y) ||
            !std::isfinite(normals->points[i].normal_z))
            continue;
        pcl::PointNormal p;
        p.x = down->points[i].x;
        p.y = down->points[i].y;
        p.z = down->points[i].z;
        p.normal_x = normals->points[i].normal_x;
        p.normal_y = normals->points[i].normal_y;
        p.normal_z = normals->points[i].normal_z;
        p.curvature = normals->points[i].curvature;
        out->push_back(p);
    }
    out->width = out->size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

// Downsample only.  No normals: the point-to-plane objective of the fine stage is linearised
// with the TARGET normals alone (pcl::TransformationEstimationPointToPlaneLLS reads only
// cloud_tgt), so computing source normals would be pure cost.
static pcl::PointCloud<pcl::PointNormal>::Ptr voxelAsPointNormal(const CloudType::Ptr &in,
                                                                 double res)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
    CloudType::Ptr down = voxelDownsample(in, res);
    out->reserve(down->size());
    for (const auto &q : down->points)
    {
        pcl::PointNormal p;
        p.x = q.x; p.y = q.y; p.z = q.z;
        p.normal_x = p.normal_y = p.normal_z = 0.f;
        p.curvature = 0.f;
        out->push_back(p);
    }
    out->width = out->size();
    out->height = 1;
    out->is_dense = true;
    return out;
}

namespace pgo_loop
{

// ===========================================================================
// Scan Context descriptor database
// ===========================================================================

ScanContextDB::ScanContextDB(const ScanContextConfig &cfg) : m_cfg(cfg)
{
    m_sc.reset(new SCManager(cfg.lidar_height_m, cfg.num_ring, cfg.num_sector, cfg.max_radius_m));
}

ScanContextDB::~ScanContextDB() = default;

size_t ScanContextDB::size() const { return m_sc->polarcontexts_.size(); }

void ScanContextDB::addKeyframe(const CloudType::Ptr &body_cloud, double *build_ms)
{
    Clock::time_point t0 = Clock::now();
    CloudType::Ptr down = voxelDownsample(body_cloud, m_cfg.downsample_resolution_m);
    m_sc->makeAndSaveScancontextAndKeys(*down);
    if (build_ms)
        *build_ms = msSince(t0);
}

std::vector<DescriptorHit> ScanContextDB::query(double *query_ms)
{
    Clock::time_point t0 = Clock::now();
    std::vector<DescriptorHit> hits;
    const int n = static_cast<int>(m_sc->polarcontexts_.size());
    const int n_search = n - m_cfg.exclude_recent;
    if (n_search <= 0)
    {
        if (query_ms)
            *query_ms = msSince(t0);
        return hits;
    }
    const int k = std::min(m_cfg.num_candidates, n_search);

    // Ringkey index over the keyframes that are old enough to be a loop target.  Rebuilt on
    // every query: at a few hundred keyframes and 20 dimensions this is far cheaper than the
    // staleness bookkeeping a cached index would need (measured: see the [PGO][sc] log line).
    KeyMat keys(m_sc->polarcontext_invkeys_mat_.begin(),
                m_sc->polarcontext_invkeys_mat_.begin() + n_search);
    InvKeyTree tree(static_cast<size_t>(m_cfg.num_ring), keys, 10);

    std::vector<size_t> ids(k, 0);
    std::vector<float> sqdists(k, 0.f);
    nanoflann::KNNResultSet<float> knn(k);
    knn.init(&ids[0], &sqdists[0]);
    const std::vector<float> &query_key = m_sc->polarcontext_invkeys_mat_.back();
    tree.index->findNeighbors(knn, &query_key[0], nanoflann::SearchParams(10));
    const size_t found = knn.size();

    // Re-rank the ringkey neighbours with the full scan-context distance (paper eq. 6), which
    // also returns the yaw that aligns the two descriptors.
    Eigen::MatrixXd &cur_desc = m_sc->polarcontexts_.back();
    for (size_t i = 0; i < found; i++)
    {
        const int cand = static_cast<int>(ids[i]);
        Eigen::MatrixXd &cand_desc = m_sc->polarcontexts_[cand];
        std::pair<double, int> d = m_sc->distanceBtnScanContext(cur_desc, cand_desc);
        DescriptorHit h;
        h.idx = cand;
        h.dist = d.first;
        // Convention pinned by test/test_scan_context_yaw.cpp on a synthetic room with known
        // ground-truth relative yaw: shifting cand_desc right by d.second columns is what
        // aligns it to cur_desc, and the resulting R_target_source is Rz(-shift).
        h.yaw_rad = -static_cast<double>(d.second) * m_sc->PC_UNIT_SECTORANGLE * M_PI / 180.0;
        hits.push_back(h);
    }
    std::sort(hits.begin(), hits.end(),
              [](const DescriptorHit &a, const DescriptorHit &b) { return a.dist < b.dist; });
    if (query_ms)
        *query_ms = msSince(t0);
    return hits;
}

double ScanContextDB::relativeYawRad(int target_idx)
{
    if (target_idx < 0 || target_idx >= static_cast<int>(m_sc->polarcontexts_.size()))
        return 0.0;
    Eigen::MatrixXd &cur_desc = m_sc->polarcontexts_.back();
    Eigen::MatrixXd &cand_desc = m_sc->polarcontexts_[target_idx];
    std::pair<double, int> d = m_sc->distanceBtnScanContext(cur_desc, cand_desc);
    return -static_cast<double>(d.second) * m_sc->PC_UNIT_SECTORANGLE * M_PI / 180.0;
}

// ===========================================================================
// Registration cascade
// ===========================================================================

static void computeCorrespondenceGates(const pcl::PointCloud<pcl::PointNormal>::Ptr &source,
                                       const pcl::PointCloud<pcl::PointNormal>::Ptr &target,
                                       const M4D &transform, const GateConfig &gc,
                                       RegistrationResult *res)
{
    Clock::time_point t0 = Clock::now();
    pcl::PointCloud<pcl::PointNormal>::Ptr aligned(new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*source, *aligned, transform.cast<float>());

    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(target);

    const double r1 = gc.overlap_radius_m, r2 = gc.overlap_radius_2_m, r3 = gc.overlap_radius_3_m;
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    size_t n_corr = 0, n2 = 0, n3 = 0;
    double sse_plane = 0.0, sse_point = 0.0;

    std::vector<int> nn(1);
    std::vector<float> d2(1);
    for (size_t i = 0; i < aligned->size(); i++)
    {
        if (kdtree.nearestKSearch(aligned->points[i], 1, nn, d2) <= 0)
            continue;
        const double d = std::sqrt(static_cast<double>(d2[0]));
        if (d <= r3)
            n3++;
        if (d <= r2)
            n2++;
        if (d <= r1)
        {
            const Eigen::Vector3d n = target->points[nn[0]].getNormalVector3fMap().cast<double>();
            if (!n.allFinite())
                continue; // never counted: H would become NaN
            n_corr++;
            H += n * n.transpose();
            const Eigen::Vector3d diff =
                aligned->points[i].getVector3fMap().cast<double>() -
                target->points[nn[0]].getVector3fMap().cast<double>();
            sse_plane += diff.dot(n) * diff.dot(n);
            sse_point += d * d;
        }
    }

    res->n_source = aligned->size();
    res->n_corr = n_corr;
    res->overlap = aligned->size() ? static_cast<double>(n_corr) / aligned->size() : 0.0;
    res->overlap_2 = aligned->size() ? static_cast<double>(n2) / aligned->size() : 0.0;
    res->overlap_3 = aligned->size() ? static_cast<double>(n3) / aligned->size() : 0.0;
    res->fine_plane_rmse_m = n_corr ? std::sqrt(sse_plane / n_corr) : -1.0;
    res->inlier_rmse_m = n_corr ? std::sqrt(sse_point / n_corr) : -1.0;

    if (n_corr >= 3)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(H); // eigenvalues ascending
        res->eig[0] = es.eigenvalues()(0);
        res->eig[1] = es.eigenvalues()(1);
        res->eig[2] = es.eigenvalues()(2);
        res->eig_ratio = res->eig[2] > 0.0 ? res->eig[0] / res->eig[2] : 0.0;
        res->eig_min_norm = static_cast<double>(n_corr) > 0.0
                                ? res->eig[0] / static_cast<double>(n_corr)
                                : 0.0;
    }
    res->ms.gates_ms = msSince(t0);
}

RegistrationResult runRegistrationCascade(const CloudType::Ptr &target_submap_world,
                                          const CloudType::Ptr &source_cloud_world,
                                          const M4D &init_guess, const RegistrationConfig &rc,
                                          const GateConfig &gc)
{
    RegistrationResult res;
    std::string reject;

    // ---------------- stage A: coarse GICP with the descriptor-yaw initial guess ------------
    {
        Clock::time_point t0 = Clock::now();
        CloudType::Ptr tgt = voxelDownsample(target_submap_world, rc.coarse_voxel_resolution_m);
        CloudType::Ptr src = voxelDownsample(source_cloud_world, rc.coarse_voxel_resolution_m);
        if (tgt->empty() || src->empty())
        {
            reject = "empty_input";
        }
        else
        {
            fast_gicp::FastGICP<PointType, PointType> gicp;
            gicp.setNumThreads(4);
            gicp.setCorrespondenceRandomness(rc.correspondence_randomness);
            gicp.setMaximumIterations(rc.coarse_max_iterations);
            gicp.setMaxCorrespondenceDistance(rc.coarse_max_corr_dist_m);
            gicp.setTransformationEpsilon(1e-8);
            gicp.setEuclideanFitnessEpsilon(1e-8);
            gicp.setRANSACIterations(0);
            gicp.setInputTarget(tgt);
            gicp.setInputSource(src);
            CloudType::Ptr out(new CloudType);
            gicp.align(*out, init_guess.cast<float>());
            res.coarse_converged = gicp.hasConverged();
            // PCL's fitness score is the MEAN SQUARED nearest-neighbour distance [m^2]; the
            // square root is the interpretable residual in metres.
            res.coarse_rmse_m = std::sqrt(gicp.getFitnessScore());
            res.fine_transform = gicp.getFinalTransformation().cast<double>();
            if (!res.coarse_converged)
                reject = "coarse_not_converged";
            else if (!(res.coarse_rmse_m <= rc.coarse_max_rmse_m))
                reject = "coarse_rmse";
        }
        res.ms.coarse_ms = msSince(t0);
    }

    if (!reject.empty())
    {
        res.reject = reject;
        return res;
    }

    // ---------------- stage B: fine point-to-plane ICP -------------------------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr fine_tgt, fine_src;
    M4D fine = res.fine_transform;
    {
        Clock::time_point t0 = Clock::now();
        fine_tgt = voxelWithNormals(target_submap_world, rc.fine_voxel_resolution_m,
                                    rc.normal_search_radius_m);
        fine_src = voxelAsPointNormal(source_cloud_world, rc.fine_voxel_resolution_m);
        if (fine_tgt->empty() || fine_src->empty())
        {
            res.ms.fine_ms = msSince(t0);
            res.reject = "empty_fine_input";
            return res;
        }
        // IterativeClosestPointWithNormals == point-to-plane (TransformationEstimation-
        // PointToPlaneLLS) with a kd-tree nearest-neighbour correspondence estimator.
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setMaximumIterations(rc.fine_max_iterations);
        icp.setMaxCorrespondenceDistance(rc.fine_max_corr_dist_m);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-8);
        icp.setRANSACIterations(0);
        icp.setInputTarget(fine_tgt);
        icp.setInputSource(fine_src);
        pcl::PointCloud<pcl::PointNormal> out;
        icp.align(out, res.fine_transform.cast<float>());
        res.fine_converged = icp.hasConverged();
        res.fine_rmse_m = std::sqrt(icp.getFitnessScore());
        fine = icp.getFinalTransformation().cast<double>();
        res.ms.fine_ms = msSince(t0);
    }
    res.fine_transform = fine;

    // ---------------- gates -----------------------------------------------------------------
    computeCorrespondenceGates(fine_src, fine_tgt, fine, gc, &res);

    // Every gate is evaluated (no short-circuit) so the log line always carries the full set
    // of values that produced the decision.
    std::vector<std::string> failed;
    if (!res.fine_converged)
        failed.push_back("fine_not_converged");
    if (!(res.fine_rmse_m <= rc.fine_max_rmse_m))
        failed.push_back("fine_rmse");
    if (!(res.fine_plane_rmse_m <= rc.fine_max_plane_rmse_m))
        failed.push_back("fine_plane_rmse");
    if (!(res.overlap >= gc.min_overlap_ratio))
        failed.push_back("overlap");
    if (gc.degeneracy_gate_enabled && !(res.eig_ratio >= gc.min_eig_ratio))
        failed.push_back("degenerate");
    for (size_t i = 0; i < failed.size(); i++)
    {
        if (!reject.empty())
            reject += ",";
        reject += failed[i];
    }

    res.accepted = reject.empty();
    res.reject = reject;
    return res;
}

} // namespace pgo_loop
