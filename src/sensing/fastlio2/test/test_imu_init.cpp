// Deterministic regression tests for the static-window IMU initialization decision
// (map_builder/imu_init.h), which integration review item B3 found could never terminate.
//
// The interesting case is a platform that is vibrating but NOT rotating (or an accelerometer
// spike): the gyro std of the quietest window passes, the accel deviation does not, so
// `is_static` stays false forever.  The max-wait fallback must still fire, or initialize()
// returns false on every call while the IMU cache grows without bound and no scan is ever
// processed.  N1 (accel deviation measured over the window, not the whole cache) is pinned
// here too.
#include <gtest/gtest.h>

#include "map_builder/imu_init.h"

#include <vector>

namespace
{

// 200 Hz sample stream with a deterministic square-wave jitter: the per-axis standard
// deviation of a +-amp alternating sequence is exactly `amp`, so the thresholds below are
// exercised with no random draw at all.
IMUData sample(double t, const V3D &acc, const V3D &gyro)
{
    IMUData d;
    d.time = t;
    d.acc = acc;
    d.gyro = gyro;
    return d;
}

V3D alternating(double amp, int i)
{
    const double s = (i % 2) ? 1.0 : -1.0;
    return V3D(amp * s, amp * s, amp * s);
}

// Static window thresholds used by lio.yaml.
const double kGyroStd = 0.005;
const double kAccDev = 0.3;
const double kWindowS = 1.0;
const double kMaxWaitS = 5.0;

} // namespace

TEST(ImuInit, NoisyAccelFallsBackAfterMaxWait)
{
    std::vector<IMUData> cache;
    // 4.9 s of a vibrating but non-rotating platform: gyro std 0.001 < 0.005, accel dev
    // 0.5 > 0.3.
    for (int i = 0; i < 980; ++i)
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81) + alternating(0.5, i),
                               alternating(0.001, i)));

    ImuInitPlan plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 20);
    EXPECT_FALSE(plan.ready); // no static window yet, and the max wait has not elapsed
    EXPECT_FALSE(plan.is_static);

    // ... and the fallback MUST fire once the wait is over, even though the accel is still
    // noisy.  (Before the fix `waited_out` also required a quiet gyro window and this stayed
    // false for this input, so initialize() never returned true.)
    for (int i = 980; i < 1040; ++i)
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81) + alternating(0.5, i),
                               alternating(0.001, i)));

    plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 20);
    EXPECT_TRUE(plan.ready);
    EXPECT_TRUE(plan.waited_out);
    EXPECT_FALSE(plan.is_static); // the fallback must not claim a static window
    EXPECT_LT(plan.gyro_std, kGyroStd);
    EXPECT_GT(plan.acc_dev, kAccDev);
    EXPECT_EQ(plan.i1 - plan.i0, 200u); // 1 s at 200 Hz (the first sample at or beyond 1 s ends it)
    EXPECT_GT(plan.span, kMaxWaitS);
}

TEST(ImuInit, ShortQuietCacheWaitsForAFullWindow)
{
    std::vector<IMUData> cache;
    for (int i = 0; i < 100; ++i) // 0.5 s only
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81) + alternating(0.01, i),
                               alternating(0.0005, i)));
    const ImuInitPlan plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 20);
    EXPECT_FALSE(plan.ready); // quiet, but not a full window yet
}

TEST(ImuInit, StaticWindowIsReadyAndNotWaitedOut)
{
    std::vector<IMUData> cache;
    for (int i = 0; i < 240; ++i) // 1.2 s
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81) + alternating(0.01, i),
                               alternating(0.0005, i)));
    const ImuInitPlan plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 20);
    EXPECT_TRUE(plan.ready);
    EXPECT_TRUE(plan.is_static);
    EXPECT_FALSE(plan.waited_out);
    EXPECT_LT(plan.gyro_std, kGyroStd);
    EXPECT_LT(plan.acc_dev, kAccDev);
}

TEST(ImuInit, QuietestWindowIsChosenOverMotion)
{
    std::vector<IMUData> cache;
    // 1.5 s of fast, TURN-RATE-VARYING rotation (a constant rate would have zero gyro std and
    // the picker would happily call it quiet) ...
    for (int i = 0; i < 300; ++i)
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81) + alternating(1.0, i),
                               alternating(0.5, i)));
    const size_t first_quiet = cache.size();
    // ... then 1.5 s at rest.
    for (int i = 0; i < 300; ++i)
        cache.push_back(sample(i * 0.005 + 1.5, V3D(0, 0, -9.81) + alternating(0.01, i),
                               alternating(0.0005, i)));

    const ImuInitPlan plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 20);
    EXPECT_TRUE(plan.ready);
    EXPECT_TRUE(plan.is_static);
    EXPECT_GE(plan.i0, first_quiet - 1); // the window lies in the quiet stretch
    EXPECT_LE(plan.i1, cache.size());
    EXPECT_LT(plan.gyro_std, kGyroStd);
    EXPECT_LT(plan.acc_dev, kAccDev);
}

TEST(ImuInit, AccelDeviationIgnoresSamplesOutsideTheWindow)
{
    std::vector<IMUData> cache;
    // 1.2 s quiet at the very start (no rotation at all -> no quiet window later) ...
    for (int i = 0; i < 240; ++i)
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81) + alternating(0.01, i),
                               alternating(0.0005, i)));
    // ... then 1.0 s of violent translation-induced vibration (accel dev 0.6, gyro still
    // quiet so the picker is free to stay in the first window).
    for (int i = 0; i < 200; ++i)
        cache.push_back(sample(i * 0.005 + 1.2, V3D(0, 0, -9.81) + alternating(0.6, i),
                               alternating(0.0005, i)));

    const ImuInitPlan plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 20);
    ASSERT_TRUE(plan.ready);
    // Over the whole cache the deviation would be ~0.4 > 0.3 and the measured-gravity
    // adoption in initialize() would be vetoed for a genuinely static window (N1).
    EXPECT_LT(plan.acc_dev, kAccDev);
    EXPECT_TRUE(plan.is_static);
}

TEST(ImuInit, LegacyModeUsesWholeCacheOnceInitNumSamplesExist)
{
    std::vector<IMUData> cache;
    for (int i = 0; i < 19; ++i)
        cache.push_back(sample(i * 0.005, V3D(0, 0, -9.81), V3D::Zero()));
    EXPECT_FALSE(planImuInit(cache, 0.0, kMaxWaitS, kGyroStd, kAccDev, 20).ready);

    cache.push_back(sample(19 * 0.005, V3D(0, 0, -9.81), V3D::Zero()));
    const ImuInitPlan plan = planImuInit(cache, 0.0, kMaxWaitS, kGyroStd, kAccDev, 20);
    EXPECT_TRUE(plan.ready);
    EXPECT_FALSE(plan.use_static_window);
    EXPECT_FALSE(plan.waited_out);
    EXPECT_EQ(plan.i0, 0u);
    EXPECT_EQ(plan.i1, cache.size());
}

// C2.2: first-batch initialization (upstream FAST-LIO2 behaviour).  The platform is already
// moving and rotating, so the static-window mode would sit out the whole 5 s fallback; the
// first batch must initialise as soon as it is complete instead.
TEST(ImuInit, FirstBatchIsReadyWithoutAStaticVerdict)
{
    std::vector<IMUData> cache;
    for (int i = 0; i < 40; ++i) // 0.1 s at 400 Hz
        cache.push_back(sample(i * 0.0025, V3D(2.0, 0, -9.4) + alternating(1.5, i),
                               alternating(0.4, i)));

    const ImuInitPlan plan = planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 40,
                                         ImuInitMode::FirstBatch);
    EXPECT_TRUE(plan.ready);
    EXPECT_FALSE(plan.use_static_window); // no quiet-window verdict in this mode
    EXPECT_FALSE(plan.is_static);
    EXPECT_FALSE(plan.waited_out);
    EXPECT_EQ(plan.i0, 0u);
    EXPECT_EQ(plan.i1, cache.size());
    EXPECT_GT(plan.gyro_std, kGyroStd); // the batch is NOT static - that is the point: it is
    EXPECT_GT(plan.acc_dev, kAccDev);   // still used, without waiting for the fallback

    cache.pop_back(); // one sample short of a full batch
    EXPECT_FALSE(planImuInit(cache, kWindowS, kMaxWaitS, kGyroStd, kAccDev, 40,
                             ImuInitMode::FirstBatch).ready);
}
