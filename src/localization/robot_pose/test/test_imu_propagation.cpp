// IMU-propagated pose (imu_propagation.hpp) against closed-form motion.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>

#include "imu_propagation.hpp"

using imu_prop::ImuSample;
using imu_prop::State;
using imu_prop::Status;

namespace
{
constexpr double kG = 9.81;

// samples at rate_hz on [t0, t1] of a body with constant world acceleration a_w and constant
// body rate w (the specific force is R^T (a_w - g); R evolves with w)
std::deque<ImuSample> makeImu(double t0, double t1, double rate_hz, const Eigen::Vector3d &a_w,
                              const Eigen::Vector3d &w, const Eigen::Quaterniond &q0)
{
    std::deque<ImuSample> d;
    const Eigen::Vector3d g(0, 0, -kG);
    for (double t = t0; t <= t1 + 1e-12; t += 1.0 / rate_hz)
    {
        const Eigen::Quaterniond q = q0 * imu_prop::expQ(w * (t - t0));
        d.push_back({t, w, q.conjugate() * (a_w - g)});
    }
    return d;
}
}  // namespace

TEST(ImuPropagation, StationaryBodyStaysPut)
{
    const auto imu = makeImu(0.0, 1.0, 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                             Eigen::Quaterniond::Identity());
    State s;
    s.t = 0.3;
    ASSERT_EQ(imu_prop::propagate(s, imu, 0.55, kG), Status::Ok);
    EXPECT_LT(s.p.norm(), 1e-9);
    EXPECT_LT(s.v.norm(), 1e-9);
}

TEST(ImuPropagation, ConstantAccelerationMatchesClosedForm)
{
    const Eigen::Vector3d a(0.8, -0.3, 0.5);  // e.g. a gait heave / acceleration burst
    const auto imu = makeImu(0.0, 1.0, 200.0, a, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    State s;
    s.t = 0.2;
    s.v = Eigen::Vector3d(1.0, 0.0, 0.0);
    const double T = 0.15;
    ASSERT_EQ(imu_prop::propagate(s, imu, 0.2 + T, kG), Status::Ok);
    const Eigen::Vector3d p_ref = Eigen::Vector3d(1.0, 0, 0) * T + 0.5 * a * T * T;
    EXPECT_LT((s.p - p_ref).norm(), 1e-9);
    EXPECT_LT((s.v - (Eigen::Vector3d(1.0, 0, 0) + a * T)).norm(), 1e-9);
}

TEST(ImuPropagation, RotationRateIsIntegrated)
{
    const Eigen::Vector3d w(0.0, 0.4, 1.2);  // pitch + yaw rate
    const auto imu = makeImu(0.0, 1.0, 200.0, Eigen::Vector3d::Zero(), w, Eigen::Quaterniond::Identity());
    State s;
    s.t = 0.0;
    ASSERT_EQ(imu_prop::propagate(s, imu, 0.2, kG), Status::Ok);
    const Eigen::Quaterniond q_ref = imu_prop::expQ(w * 0.2);
    EXPECT_LT(s.q.angularDistance(q_ref), 1e-9);
    EXPECT_LT(s.p.norm(), 1e-6);  // body rotating in place, no translation
}

TEST(ImuPropagation, StartBetweenSamplesUsesThePartialInterval)
{
    const Eigen::Vector3d a(0.0, 0.0, 2.0);
    const auto imu = makeImu(0.0, 1.0, 200.0, a, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    State s;
    s.t = 0.1234;  // not on the 5 ms grid
    ASSERT_EQ(imu_prop::propagate(s, imu, 0.2577, kG), Status::Ok);
    const double T = 0.2577 - 0.1234;
    EXPECT_NEAR(s.p.z(), 0.5 * 2.0 * T * T, 1e-9);
}

TEST(ImuPropagation, MissingCoverageAndGapsAreRejected)
{
    auto imu = makeImu(0.0, 0.5, 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       Eigen::Quaterniond::Identity());
    State s;
    s.t = 0.2;
    EXPECT_EQ(imu_prop::propagate(s, imu, 0.8, kG), Status::NoCoverage);  // query past the data
    State s2;
    s2.t = 0.1;
    std::deque<ImuSample> gap;
    for (const auto &m : imu)
        if (m.t < 0.2 || m.t > 0.35)
            gap.push_back(m);
    EXPECT_EQ(imu_prop::propagate(s2, gap, 0.4, kG), Status::Gap);
}
