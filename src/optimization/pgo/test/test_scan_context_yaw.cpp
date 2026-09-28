// Pins the yaw convention of the vendored Scan Context descriptor.
//
// The descriptor's returned shift is used in SimplePGO::searchForLoopPairs() to build the
// coarse-registration initial guess, and the sign of that conversion cannot be read off the
// upstream code (the upstream SC-LIO-SAM fork explicitly does NOT use it: "// not use for v1").
// This test decides it against ground truth: a synthetic room is rendered into two scans from
// the SAME sensor position with a known relative yaw, and the rotation recovered from the
// descriptor must overlay the source scan onto the target scan to a few centimetres.  Flipping
// the sign in ScanContextDB::query() makes the wrong hypothesis ~10-30x worse here, and in the
// field it silently sends the coarse stage into the wrong basin.
#include <gtest/gtest.h>

#include "scan_context/Scancontext.h"

#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>

#include <cmath>
#include <random>
#include <vector>

namespace
{

const double kDeg = M_PI / 180.0;

// Walls + floor + clutter columns: a room that is NOT rotationally symmetric, so that the
// descriptor has a single unambiguous yaw alignment.
pcl::PointCloud<SCPointType> makeRoom()
{
    pcl::PointCloud<SCPointType> world;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> jitter(-0.02, 0.02);
    auto add = [&](double x, double y, double z) {
        SCPointType p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(z);
        p.intensity = 0.f;
        world.push_back(p);
    };
    for (int i = 0; i < 4000; ++i) add(-7.0 + 14.0 * i / 3999.0, 0.0, -1.5 + jitter(rng));    // floor
    for (int i = 0; i < 4000; ++i) add(-7.0 + 14.0 * i / 3999.0, 5.0 + jitter(rng), jitter(rng));
    for (int i = 0; i < 3000; ++i) add(-7.0 + 14.0 * i / 2999.0, -5.0 + jitter(rng), jitter(rng));
    for (int i = 0; i < 2000; ++i) add(7.0 + jitter(rng), -5.0 + 10.0 * i / 1999.0, jitter(rng));
    for (int i = 0; i < 2000; ++i) add(-7.0 + jitter(rng), -5.0 + 10.0 * i / 1999.0, jitter(rng));
    for (int k = 0; k < 6; ++k)
    {
        const double cx = -4.0 + 8.0 * k / 5.0, cy = 2.0 - 0.3 * k;
        for (int i = 0; i < 400; ++i)
            add(cx + 0.15 * std::cos(i * 0.7), cy + 0.15 * std::sin(i * 0.7),
                -1.4 + 0.6 * i / 399.0);
    }
    return world;
}

// Render the room into the sensor frame of a sensor at the origin with world yaw `psi`.
pcl::PointCloud<SCPointType> render(const pcl::PointCloud<SCPointType> &world, double psi_rad)
{
    pcl::PointCloud<SCPointType> scan;
    scan.reserve(world.size());
    const Eigen::Matrix3d R = Eigen::AngleAxisd(psi_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for (const auto &p : world.points)
    {
        const Eigen::Vector3d b = R * Eigen::Vector3d(p.x, p.y, p.z);
        SCPointType q;
        q.x = static_cast<float>(b.x());
        q.y = static_cast<float>(b.y());
        q.z = static_cast<float>(b.z());
        q.intensity = 0.f;
        scan.push_back(q);
    }
    return scan;
}

double overlayRmse(const pcl::PointCloud<SCPointType> &target,
                   const pcl::PointCloud<SCPointType> &source, double rotate_source_by_deg)
{
    pcl::KdTreeFLANN<SCPointType> tree;
    tree.setInputCloud(target.makeShared());
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(rotate_source_by_deg * kDeg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    double sum = 0.0;
    int n = 0;
    std::vector<int> id(1);
    std::vector<float> d2(1);
    for (std::size_t i = 0; i < source.size(); i += 7)
    {
        const Eigen::Vector3d b(source.points[i].x, source.points[i].y, source.points[i].z);
        const Eigen::Vector3d r = R * b;
        SCPointType q;
        q.x = static_cast<float>(r.x());
        q.y = static_cast<float>(r.y());
        q.z = static_cast<float>(r.z());
        q.intensity = 0.f;
        if (tree.nearestKSearch(q, 1, id, d2) > 0)
        {
            sum += std::sqrt(static_cast<double>(d2[0]));
            n++;
        }
    }
    return n ? sum / n : 1e9;
}

} // namespace

// For every unambiguous relative yaw (0 and 180 deg make +-yaw the same rotation, so they
// cannot discriminate the sign and are skipped), the descriptor must return
// R_target_source = Rz(-shift * PC_UNIT_SECTORANGLE) as implemented in
// ScanContextDB::query()/relativeYawRad().
TEST(ScanContextYaw, SourceToTargetYawSignMatchesGroundTruth)
{
    const pcl::PointCloud<SCPointType> world = makeRoom();
    SCManager sc; // upstream defaults, same as ScanContextDB's default configuration

    const std::vector<double> rel_yaws_deg = {6.0, 30.0, 90.0, 143.0, -47.0, 204.0, 300.0};
    for (double dpsi_deg : rel_yaws_deg)
    {
        const pcl::PointCloud<SCPointType> target = render(world, 0.0);
        const pcl::PointCloud<SCPointType> source = render(world, dpsi_deg * kDeg);
        Eigen::MatrixXd d_t = sc.makeScancontext(const_cast<pcl::PointCloud<SCPointType> &>(target));
        Eigen::MatrixXd d_s = sc.makeScancontext(const_cast<pcl::PointCloud<SCPointType> &>(source));
        const std::pair<double, int> hit = sc.distanceBtnScanContext(d_s, d_t);
        // the convention under test, as used by ScanContextDB:
        const double yaw_deg = -hit.second * sc.PC_UNIT_SECTORANGLE;

        const double wrong_sign_deg = -yaw_deg;
        const double rmse_right = overlayRmse(target, source, yaw_deg);
        const double rmse_wrong = overlayRmse(target, source, wrong_sign_deg);

        EXPECT_LT(rmse_right, 0.10)
            << "relative yaw " << dpsi_deg << " deg: correct-sign overlay is not tight";
        EXPECT_GT(rmse_wrong, 10.0 * rmse_right)
            << "relative yaw " << dpsi_deg << " deg: the wrong sign is not clearly worse, the "
               "descriptor shift is not carrying the relative yaw assumed by the front end";
    }
}

// The two hypotheses coincide at 0 and 180 deg; only the overlay quality is checked there.
TEST(ScanContextYaw, DegenerateYawsStillOverlay)
{
    const pcl::PointCloud<SCPointType> world = makeRoom();
    SCManager sc;
    for (double dpsi_deg : {0.0, 180.0})
    {
        const pcl::PointCloud<SCPointType> target = render(world, 0.0);
        const pcl::PointCloud<SCPointType> source = render(world, dpsi_deg * kDeg);
        Eigen::MatrixXd d_t = sc.makeScancontext(const_cast<pcl::PointCloud<SCPointType> &>(target));
        Eigen::MatrixXd d_s = sc.makeScancontext(const_cast<pcl::PointCloud<SCPointType> &>(source));
        const std::pair<double, int> hit = sc.distanceBtnScanContext(d_s, d_t);
        const double yaw_deg = -hit.second * sc.PC_UNIT_SECTORANGLE;
        EXPECT_LT(overlayRmse(target, source, yaw_deg), 0.10) << "relative yaw " << dpsi_deg;
    }
}

// Sanity: the descriptor distance is near zero for the same place and clearly larger for a
// different place, which is what makes the SC_DIST_THRES candidate gate meaningful.
TEST(ScanContextYaw, DescriptorDistanceSeparatesSameFromDifferentPlace)
{
    const pcl::PointCloud<SCPointType> world = makeRoom();
    SCManager sc;
    pcl::PointCloud<SCPointType> same_a = render(world, 0.0);
    pcl::PointCloud<SCPointType> same_b = render(world, 37.0 * kDeg);
    // A different place: only the near half of the room (the rest is beyond the far wall).
    pcl::PointCloud<SCPointType> other;
    for (const auto &p : world.points)
        if (p.y < -3.4 && p.x > -1.0)
            other.push_back(p);
    Eigen::MatrixXd d_a = sc.makeScancontext(same_a);
    Eigen::MatrixXd d_b = sc.makeScancontext(same_b);
    Eigen::MatrixXd d_c = sc.makeScancontext(other);
    const double same = sc.distanceBtnScanContext(d_a, d_b).first;
    const double diff = sc.distanceBtnScanContext(d_a, d_c).first;
    EXPECT_LT(same, 0.30) << "same place should be inside the candidate gate";
    EXPECT_GT(diff, same) << "a different place must score worse than the same place";
}
