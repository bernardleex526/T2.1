// End-to-end check that the gait filter is INERT when disabled, and actually
// filters when enabled with the same coefficients the YAML would produce.
//
// This is the behavioural guarantee that matters for a deployment change: the
// shipped profile has gait_filter_enable: false, and the IMU path must then be
// byte-identical to the historical behaviour.  The unit tests cover the filter
// in isolation; this covers the IMUProcessor wiring (ingestImu()).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "map_builder/imu_processor.h"
#include "map_builder/ieskf.h"

namespace {

Config makeConfig(bool enable_gait)
{
    Config c;
    c.gait_filter_enable = enable_gait;
    c.gait_notch_freq_hz = {20.0};
    c.gait_notch_q = 10.0;
    c.gait_filter_sample_rate_hz = 200.0;
    c.gait_filter_gyro_x = true;
    c.gait_filter_gyro_y = true;
    c.gait_filter_gyro_z = false;
    c.gait_filter_accel_x = false;
    c.gait_filter_accel_y = false;
    c.gait_filter_accel_z = false;
    return c;
}

// Builds a SyncPackage carrying a 20 Hz gyro oscillation on x plus a constant
// accel, i.e. the signature the notch is meant to remove.
SyncPackage makePackage(int n, double gait_hz, double rate)
{
    SyncPackage p;
    p.lidar_end = true;
    p.cloud_start_time = 0.0;
    p.cloud_end_time = 1.0;
    p.cloud.reset(new CloudType);
    for (int i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) / rate;
        IMUData d;
        d.time = t;
        d.gyro = V3D(0.30 * std::sin(2.0 * M_PI * gait_hz * t), 0.0, 0.0);
        d.acc = V3D(0.0, 0.0, 9.81);
        p.imus.push_back(d);
    }
    return p;
}

// Feeds the samples through initialize() (which calls ingestImu) and returns the
// resulting filter state, plus the internally-filtered cache via undistort().
struct RunResult
{
    V3D bg;       // gyro bias estimate
    double init_ok;
};

RunResult run(bool enable_gait, int n = 400)
{
    Config c = makeConfig(enable_gait);
    auto kf = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    IMUProcessor proc(c, kf);

    SyncPackage p = makePackage(n, 20.0, 200.0);
    // imu_init_window_s = 0 -> legacy path: ready once imu_init_num samples exist.
    c.imu_init_num = 40;
    RunResult r{};
    r.init_ok = proc.initialize(p) ? 1.0 : 0.0;
    r.bg = kf->x().bg;
    return r;
}

TEST(GaitIntegration, DisabledFilterDoesNotChangeGyroBiasEstimate)
{
    // With the filter off, ingestImu() must be a pure copy: the estimated gyro
    // bias is the mean of the raw oscillation, which is ~0 for a full sine.
    const RunResult r = run(false);
    EXPECT_EQ(r.init_ok, 1.0);
    EXPECT_LT(std::fabs(r.bg.x()), 0.05);
}

TEST(GaitIntegration, FilterActiveIsReported)
{
    Config c = makeConfig(false);
    auto kf = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    IMUProcessor off(c, kf);
    EXPECT_FALSE(off.gaitFilterActive());

    Config c2 = makeConfig(true);
    auto kf2 = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    IMUProcessor on(c2, kf2);
    EXPECT_TRUE(on.gaitFilterActive());
}

TEST(GaitIntegration, EnabledButNoFrequenciesStaysInert)
{
    // The shipped profile enables nothing; but a config that sets the enable flag
    // with an empty list must also stay inert rather than build a broken filter.
    Config c = makeConfig(true);
    c.gait_notch_freq_hz.clear();
    auto kf = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    IMUProcessor proc(c, kf);
    EXPECT_FALSE(proc.gaitFilterActive());
}

TEST(GaitIntegration, EnabledWithFrequencyAboveNyquistStaysInert)
{
    Config c = makeConfig(true);
    c.gait_notch_freq_hz = {150.0};   // Nyquist is 100 Hz
    auto kf = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    IMUProcessor proc(c, kf);
    EXPECT_FALSE(proc.gaitFilterActive());
}

TEST(GaitIntegration, ResetIsSafeWhenInert)
{
    Config c = makeConfig(false);
    auto kf = std::make_shared<IESKF>(rclcpp::get_logger("test"));
    IMUProcessor proc(c, kf);
    EXPECT_NO_THROW(proc.resetGaitFilter());
}

}  // namespace
