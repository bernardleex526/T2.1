// IMUProcessor::undistort regressions:
//   * a package with NO new IMU sample (stream gap) must propagate from where the previous scan
//     ended, not again from the last IMU sample (the interval [last sample, previous end] used to
//     be integrated twice);
//   * a scan with no points must not be de-skewed at all (end()-1 on an empty vector walked the
//     heap backwards).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "map_builder/ieskf.h"
#include "map_builder/imu_processor.h"

namespace
{
IMUData imu(double t, const V3D &acc)
{
    IMUData d;
    d.time = t;
    d.acc = acc;
    d.gyro = V3D::Zero();
    return d;
}

struct Rig
{
    std::shared_ptr<IESKF> kf;
    std::unique_ptr<IMUProcessor> proc;
};

// Static, level IMU for 1 s at 200 Hz; legacy init (whole cache once imu_init_num samples exist).
Rig initialized()
{
    Config cfg;
    cfg.imu_init_window_s = 0.0;
    cfg.imu_init_num = 40;
    Rig r;
    r.kf = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    r.proc = std::make_unique<IMUProcessor>(cfg, r.kf);
    SyncPackage init;
    init.lidar_end = true;
    init.cloud.reset(new CloudType);
    for (int i = 0; i < 200; ++i)
        init.imus.push_back(imu(i * 0.005, V3D(0, 0, 9.81)));
    EXPECT_TRUE(r.proc->initialize(init));
    return r;
}

SyncPackage scan(double t0, double t1, bool with_point)
{
    SyncPackage p;
    p.lidar_end = true;
    p.cloud_start_time = t0;
    p.cloud_end_time = t1;
    p.cloud.reset(new CloudType);
    if (with_point)
    {
        PointType q;
        q.x = 3.0f; q.y = 0.0f; q.z = 0.0f;
        q.curvature = 50.0f; // 50 ms into the scan
        p.cloud->push_back(q);
    }
    return p;
}
} // namespace

TEST(ImuProcessorUndistort, PackageWithoutImuIsNotIntegratedTwice)
{
    Rig rig = initialized();
    auto &kf = rig.kf;
    auto &proc = rig.proc;

    // scan A [1.0, 1.1]: level IMU accelerating at 1 m/s^2 along x, last sample at 1.095 s
    SyncPackage a = scan(1.0, 1.1, true);
    for (int i = 0; i < 20; ++i)
        a.imus.push_back(imu(1.0 + i * 0.005, V3D(1.0, 0, 9.81)));
    proc->undistort(a);
    const double vx_a = kf->x().v.x();

    // scan B [1.1, 1.2] arrives with no IMU sample: exactly 0.1 s of the held input
    SyncPackage b = scan(1.1, 1.2, true);
    proc->undistort(b);
    EXPECT_NEAR(kf->x().v.x() - vx_a, 0.1, 1e-9); // was 0.105 (the 5 ms after 1.095 s counted twice)
}

TEST(ImuProcessorUndistort, EmptyCloudIsNotDeskewed)
{
    Rig rig = initialized();
    auto &kf = rig.kf;
    auto &proc = rig.proc;
    SyncPackage p = scan(1.0, 1.1, false);
    for (int i = 0; i < 20; ++i)
        p.imus.push_back(imu(1.0 + i * 0.005, V3D(0, 0, 9.81)));
    proc->undistort(p); // must not touch memory outside the (empty) cloud
    EXPECT_TRUE(p.cloud->empty());
    // and the next, normal scan still propagates and de-skews
    SyncPackage q = scan(1.1, 1.2, true);
    for (int i = 0; i < 20; ++i)
        q.imus.push_back(imu(1.1 + i * 0.005, V3D(0, 0, 9.81)));
    proc->undistort(q);
    ASSERT_EQ(q.cloud->size(), 1u);
    EXPECT_TRUE(std::isfinite(q.cloud->points[0].x));
    EXPECT_NEAR(kf->x().v.norm(), 0.0, 1e-9);
}
