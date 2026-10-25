// LidarProcessor must accept a scan whose DOWNSAMPLED size exceeds any fixed capacity: an Airy at
// >=300 kHz with a 5 cm scan voxel gives >10k points per scan, which used to write past the end
// of the 10k-element per-point work buffers (segfault after the second scan).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "map_builder/ieskf.h"
#include "map_builder/lidar_processor.h"

namespace {

// Points on the floor z=0 and two walls x=6 and y=6 of a corner, spaced `step` m apart.
CloudType::Ptr cornerCloud(double step, double extent)
{
    CloudType::Ptr c(new CloudType);
    for (double a = -extent; a <= extent; a += step)
        for (double b = -extent; b <= extent; b += step) {
            PointType p;
            p.x = a; p.y = b; p.z = 0.0; c->push_back(p);                       // floor
            p.x = 6.0; p.y = a; p.z = 0.5 * (b + extent) / extent * 3.0; c->push_back(p);  // wall x
            p.x = a; p.y = 6.0; c->push_back(p);                                 // wall y
        }
    return c;
}

TEST(LidarProcessorBuffers, DenseScanAboveTenThousandPointsIsProcessed)
{
    Config cfg;
    cfg.scan_resolution = 0.0;  // keep every point: the downsampled cloud is the raw cloud
    cfg.map_resolution = 0.1;
    cfg.near_search_num = 5;
    cfg.point_quality_thresh = 0.9;
    auto kf = std::make_shared<IESKF>();
    kf->setMaxIter(3);
    LidarProcessor lp(cfg, kf);

    PointVec map_pts;
    const CloudType::Ptr map_cloud = cornerCloud(0.1, 5.0);
    for (const auto &p : *map_cloud)
        map_pts.push_back(p);
    lp.initCloudMap(map_pts);

    SyncPackage pkg;
    pkg.cloud = cornerCloud(0.05, 4.0);
    ASSERT_GT(pkg.cloud->size(), 30000u);
    for (int scan = 0; scan < 3; ++scan)
        lp.process(pkg);

    // the true pose is the identity: a sane update leaves it there (and nothing crashed)
    EXPECT_LT(kf->x().t_wi.norm(), 0.02);
}


// ---- plane fit (esti_plane) -----------------------------------------------------------------
// The world origin is the IMU init position, so a surface at the init height (table, stair landing)
// passes through it.  The old fit solved n'.p = -1, which cannot represent such a plane: five
// coplanar neighbours on z = 0 came back as a VERTICAL "plane" that still passed the 0.1 m test.
TEST(EstiPlane, PlaneThroughTheWorldOriginKeepsItsNormal)
{
    PointVec pts;
    for (const auto &xy : {std::make_pair(1.00, 1.00), std::make_pair(1.06, 1.01), std::make_pair(0.97, 1.05),
                           std::make_pair(1.03, 0.94), std::make_pair(0.95, 0.98)})
    {
        PointType p;
        p.x = static_cast<float>(xy.first);
        p.y = static_cast<float>(xy.second);
        p.z = 0.0f;
        pts.push_back(p);
    }
    V4D abcd;
    ASSERT_TRUE(esti_plane(pts, 0.1, abcd));
    EXPECT_NEAR(std::abs(abcd(2)), 1.0, 1e-9);
    EXPECT_NEAR(abcd.head<3>().norm(), 1.0, 1e-12);
    EXPECT_NEAR(abcd(3), 0.0, 1e-6);
}

TEST(EstiPlane, OffsetPlaneAndRejections)
{
    // tilted plane n.p + d = 0 far from the origin
    const V3D n = V3D(1.0, 2.0, 2.0).normalized();
    const double d = -7.0;
    const V3D a = n.cross(V3D::UnitZ()).normalized(), b = n.cross(a);
    const V3D o = -d * n;
    PointVec pts;
    for (int k = 0; k < 5; ++k)
    {
        const V3D p = o + 0.5 * std::cos(1.3 * k) * a + 0.5 * std::sin(0.7 * k + 0.4) * b;
        PointType q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        pts.push_back(q);
    }
    V4D abcd;
    ASSERT_TRUE(esti_plane(pts, 0.1, abcd));
    const double sgn = abcd.head<3>().dot(n) > 0 ? 1.0 : -1.0;
    EXPECT_NEAR((sgn * abcd.head<3>() - n).norm(), 0.0, 1e-5);
    EXPECT_NEAR(sgn * abcd(3), d, 1e-4);
    // one neighbour 0.4 m off a 1 m wide patch fails the 0.1 m residual test
    pts[2].x += 0.4f * static_cast<float>(n.x());
    pts[2].y += 0.4f * static_cast<float>(n.y());
    pts[2].z += 0.4f * static_cast<float>(n.z());
    EXPECT_FALSE(esti_plane(pts, 0.1, abcd));
    // collinear neighbours carry no plane
    PointVec line;
    for (int k = 0; k < 5; ++k)
    {
        PointType q;
        q.x = 1.0f + 0.02f * k; q.y = 2.0f; q.z = 0.5f;
        line.push_back(q);
    }
    EXPECT_FALSE(esti_plane(line, 0.1, abcd));
}

// ---- end-to-end: the point-to-plane Jacobian drives the IEKF to the true pose ---------------
// Room 10 x 8 x 3 m whose FLOOR passes through the world origin (the degenerate case of the old
// plane fit).  From 2 deg / 10 cm off, one update must land on the true pose.
TEST(LidarProcessorUpdate, ConvergesFromAPerturbedPoseInARoom)
{
    struct Plane { V3D n; double d; };
    const std::vector<Plane> planes = {{V3D(0, 0, 1), 0.0}, {V3D(0, 0, -1), 3.0}, {V3D(1, 0, 0), 5.0},
                                       {V3D(-1, 0, 0), 5.0}, {V3D(0, 1, 0), 4.0}, {V3D(0, -1, 0), 4.0}};
    auto inside = [](const V3D &p, double m) {
        return p.x() >= -5 - m && p.x() <= 5 + m && p.y() >= -4 - m && p.y() <= 4 + m && p.z() >= -m && p.z() <= 3 + m;
    };
    PointVec map_pts;
    std::vector<V3D> scan_w;
    unsigned seed = 7;
    auto uni = [&seed]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xFFFF) / 32767.5 - 1.0; };
    for (const Plane &pl : planes)
    {
        const V3D a = (std::abs(pl.n.z()) < 0.9 ? V3D::UnitZ() : V3D::UnitX()).cross(pl.n).normalized();
        const V3D b = pl.n.cross(a), o = -pl.d * pl.n;
        for (double s = -6; s <= 6; s += 0.08)
            for (double t = -6; t <= 6; t += 0.08)
            {
                const V3D p = o + s * a + t * b;
                if (!inside(p, 1e-6))
                    continue;
                PointType q;
                q.x = p.x(); q.y = p.y(); q.z = p.z();
                map_pts.push_back(q);
            }
        for (int added = 0, tries = 0; added < 200 && tries < 20000; ++tries)
        {
            const V3D p = o + 4.5 * uni() * a + 4.5 * uni() * b;
            bool ok = inside(p, 1e-3);
            for (const Plane &other : planes) // stay away from the edges: one plane per neighbourhood
                if (&other != &pl && std::abs(other.n.dot(p) + other.d) < 0.4)
                    ok = false;
            if (ok)
            {
                scan_w.push_back(p);
                ++added;
            }
        }
    }
    State truth;
    truth.r_wi = Sophus::SO3d::exp(V3D(0.05, -0.03, 0.4)).matrix();
    truth.t_wi = V3D(0.5, -0.3, 1.2);
    truth.t_il = V3D(0.05, 0.0, 0.03);
    SyncPackage pkg;
    pkg.cloud.reset(new CloudType);
    for (const V3D &pw : scan_w)
    {
        const V3D pl = truth.r_il.transpose() * (truth.r_wi.transpose() * (pw - truth.t_wi) - truth.t_il);
        PointType q;
        q.x = pl.x(); q.y = pl.y(); q.z = pl.z();
        pkg.cloud->push_back(q);
    }

    Config cfg;
    cfg.scan_resolution = 0.0;
    cfg.map_resolution = 0.08;
    cfg.point_quality_thresh = 0.9;
    auto kf = std::make_shared<IESKF>();
    kf->setMaxIter(10);
    LidarProcessor lp(cfg, kf);
    lp.initCloudMap(map_pts);
    kf->x() = truth;
    lp.process(pkg); // installs the scan (at the true pose nothing moves)

    State start = truth;
    start.r_wi = truth.r_wi * Sophus::SO3d::exp(V3D(0.01, -0.015, 0.03)).matrix();
    start.t_wi = truth.t_wi + V3D(0.08, -0.05, 0.04);
    kf->x() = start;
    M21D P = M21D::Identity() * 1e-6;
    P.block<3, 3>(0, 0) = M3D::Identity() * 0.05 * 0.05;
    P.block<3, 3>(3, 3) = M3D::Identity() * 0.3 * 0.3;
    P.block<3, 3>(12, 12) = M3D::Identity();
    kf->P() = P;
    kf->update();
    const V21D err = kf->x() - truth;
    EXPECT_LT(err.segment<3>(0).norm() * 180.0 / M_PI, 0.01) << "rotation error [deg]";
    EXPECT_LT(err.segment<3>(3).norm(), 0.002) << "position error [m]";
}

}  // namespace
