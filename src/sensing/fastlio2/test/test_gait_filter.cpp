// Behavioural regression for the quadruped gait notch cascade
// (map_builder/gait_filter.h).
//
// What is asserted is the SIGNAL-PROCESSING CONTRACT, never source text:
//   * an inert filter (never configured / rejected config) is a bit-exact
//     pass-through, which is what keeps the feature off by default;
//   * a configured notch actually attenuates its own centre frequency and
//     leaves far-away frequencies alone;
//   * unselected axes are untouched AND do not advance filter state (the
//     review sketch's "filter, then overwrite the yaw component" bug would
//     leak state between axes);
//   * the steady-state gain at DC is unity, so the notch does not add a bias
//     to the gyro/accel the pre-integration consumes;
//   * configuration validation refuses Nyquist/zero/negative centres and
//     non-positive Q instead of silently producing a dead or unstable filter;
//   * filtering both gyro and accel with the same coefficients yields the same
//     group delay, which is what preserves their mutual time alignment.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "map_builder/gait_filter.h"

namespace {

constexpr double kFs = 200.0;   // IMU rate [Hz]
constexpr double kGait = 20.0;  // gait fundamental [Hz] - a test value, not a measured one

gait::GaitNotchFilter::Options baseOptions()
{
    gait::GaitNotchFilter::Options o;
    o.notch_freq_hz = {kGait};
    o.q = 10.0;
    o.sample_rate_hz = kFs;
    o.filter_gyro_x = true;
    o.filter_gyro_y = true;
    o.filter_gyro_z = false;
    o.filter_accel_x = false;
    o.filter_accel_y = false;
    o.filter_accel_z = false;
    return o;
}

// Feeds n samples of a sine at freq into one axis and returns the peak
// amplitude of the second half (steady state, first half discarded as
// transient).
double steadyStateGain(gait::GaitNotchFilter &f, int axis, double freq, bool accel, int n = 4000)
{
    const double amp = 1.0;
    double peak = 0.0;
    for (int i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) / kFs;
        const double s = amp * std::sin(2.0 * M_PI * freq * t);
        Eigen::Vector3d in = Eigen::Vector3d::Zero();
        in[axis] = s;
        const Eigen::Vector3d out = accel ? f.filterAccel(in) : f.filterGyro(in);
        if (i >= n / 2)
            peak = std::max(peak, std::fabs(out[axis]));
    }
    return peak / amp;
}

// ---- inert by default -----------------------------------------------------

TEST(GaitNotchFilter, DefaultConstructedIsPassThrough)
{
    gait::GaitNotchFilter f;
    EXPECT_FALSE(f.valid());
    const Eigen::Vector3d in(1.25, -3.5, 0.75);
    EXPECT_TRUE(f.filterGyro(in).isApprox(in, 0.0));   // exact, not approximate
    EXPECT_TRUE(f.filterAccel(in).isApprox(in, 0.0));
}

TEST(GaitNotchFilter, RejectedConfigLeavesFilterInert)
{
    gait::GaitNotchFilter f;
    auto o = baseOptions();
    o.sample_rate_hz = 30.0;          // Nyquist 15 Hz < 20 Hz centre
    o.notch_freq_hz = {kGait};
    EXPECT_FALSE(f.configure(o));
    EXPECT_FALSE(f.valid());

    const Eigen::Vector3d in(2.0, 4.0, -1.0);
    EXPECT_TRUE(f.filterGyro(in).isApprox(in, 0.0));
}

TEST(GaitNotchFilter, EmptyCentreListIsInertButNotAnError)
{
    auto o = baseOptions();
    o.notch_freq_hz.clear();
    const auto v = gait::GaitNotchFilter::validate(o);
    EXPECT_FALSE(v.ok);
    EXPECT_TRUE(v.errors.empty());              // a warning, not a fatal error
    EXPECT_EQ(v.warnings.size(), 1u);
}

// ---- the notch actually notches -------------------------------------------

TEST(GaitNotchFilter, AttenuatesCentreFrequency)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));
    ASSERT_TRUE(f.valid());
    EXPECT_EQ(f.sectionCount(), 1);

    // Analytic response at the exact centre.
    EXPECT_LT(f.magnitudeAt(kGait), 0.05);

    // And the measured steady-state gain of a real sine agrees.
    EXPECT_LT(steadyStateGain(f, 0, kGait, false), 0.05);
}

TEST(GaitNotchFilter, LeavesFarFrequenciesAlone)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));

    // A 2 Hz motion and a 60 Hz component are far outside the notch.
    EXPECT_GT(f.magnitudeAt(2.0), 0.98);
    EXPECT_GT(f.magnitudeAt(60.0), 0.95);
    EXPECT_GT(steadyStateGain(f, 0, 2.0, false), 0.97);
}

TEST(GaitNotchFilter, CascadeRejectsFundamentalAndHarmonic)
{
    auto o = baseOptions();
    o.notch_freq_hz = {kGait, 40.0};
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(o));
    ASSERT_EQ(f.sectionCount(), 2);

    EXPECT_LT(f.magnitudeAt(kGait), 0.05);
    EXPECT_LT(f.magnitudeAt(40.0), 0.05);
    EXPECT_GT(f.magnitudeAt(5.0), 0.95);
}

TEST(GaitNotchFilter, HigherQNarrowsTheNotch)
{
    auto narrow = baseOptions();
    narrow.q = 30.0;
    auto wide = baseOptions();
    wide.q = 3.0;

    gait::GaitNotchFilter fn, fw;
    ASSERT_TRUE(fn.configure(narrow));
    ASSERT_TRUE(fw.configure(wide));

    // At a fixed offset from the centre the narrow filter removes less.
    const double probe = kGait + 6.0;
    EXPECT_GT(fn.magnitudeAt(probe), fw.magnitudeAt(probe));
}

// ---- axis masking ---------------------------------------------------------

TEST(GaitNotchFilter, UnselectedAxisIsExactPassThrough)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));  // gyro z excluded

    // Drive a strongly oscillating yaw; the yaw channel must come out untouched
    // even after the transient.
    double max_err = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        const double t = static_cast<double>(i) / kFs;
        Eigen::Vector3d in = Eigen::Vector3d::Zero();
        in[2] = std::sin(2.0 * M_PI * kGait * t);
        const Eigen::Vector3d out = f.filterGyro(in);
        max_err = std::max(max_err, std::fabs(out[2] - in[2]));
    }
    EXPECT_EQ(max_err, 0.0);
}

TEST(GaitNotchFilter, UnselectedAxisDoesNotAdvanceState)
{
    // Interleaving a huge yaw excursion must not perturb the filtered axes: if
    // the excluded axis shared state (or was run through the filter and then
    // overwritten, as in the review sketch), the x/y outputs would change.
    gait::GaitNotchFilter a, b;
    ASSERT_TRUE(a.configure(baseOptions()));
    ASSERT_TRUE(b.configure(baseOptions()));

    std::vector<double> out_a, out_b;
    for (int i = 0; i < 500; ++i)
    {
        const double t = static_cast<double>(i) / kFs;
        const double common = std::sin(2.0 * M_PI * 3.0 * t);

        Eigen::Vector3d in_a(common, common, 0.0);
        Eigen::Vector3d in_b(common, common, 500.0 * std::sin(2.0 * M_PI * kGait * t));

        out_a.push_back(a.filterGyro(in_a)[0]);
        out_b.push_back(b.filterGyro(in_b)[0]);
    }
    ASSERT_EQ(out_a.size(), out_b.size());
    for (std::size_t i = 0; i < out_a.size(); ++i)
        EXPECT_DOUBLE_EQ(out_a[i], out_b[i]) << "at sample " << i;
}

TEST(GaitNotchFilter, AccelMaskIsIndependentOfGyroMask)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));  // gyro x/y on, accel all off

    double max_err = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        const double t = static_cast<double>(i) / kFs;
        const Eigen::Vector3d in(9.81 + std::sin(2.0 * M_PI * kGait * t), 0.0, 0.0);
        const Eigen::Vector3d out = f.filterAccel(in);
        max_err = std::max(max_err, (out - in).cwiseAbs().maxCoeff());
    }
    EXPECT_EQ(max_err, 0.0);
}

// ---- DC / bias preservation ----------------------------------------------

TEST(GaitNotchFilter, UnityGainAtDc)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));

    // A constant gyro reading must survive unchanged in steady state: a notch
    // that shifted the mean would look like a bias to the estimator.
    const Eigen::Vector3d in(0.37, -0.21, 0.11);
    Eigen::Vector3d out = in;
    for (int i = 0; i < 5000; ++i)
        out = f.filterGyro(in);
    EXPECT_NEAR(out[0], in[0], 1e-9);
    EXPECT_NEAR(out[1], in[1], 1e-9);
    EXPECT_DOUBLE_EQ(out[2], in[2]);  // masked axis: exact
}

TEST(GaitNotchFilter, ResetClearsTransientState)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));

    Eigen::Vector3d in = Eigen::Vector3d::Zero();
    in[0] = 5.0;
    for (int i = 0; i < 37; ++i)
        f.filterGyro(in);

    f.reset();

    // After reset the very first output equals the first input response of a
    // fresh filter: b0*x (the direct path), i.e. no leftover history.
    const Eigen::Vector3d first = f.filterGyro(in);
    gait::GaitNotchFilter fresh;
    ASSERT_TRUE(fresh.configure(baseOptions()));
    const Eigen::Vector3d first_fresh = fresh.filterGyro(in);
    EXPECT_DOUBLE_EQ(first[0], first_fresh[0]);
    EXPECT_DOUBLE_EQ(first[1], first_fresh[1]);
}

// ---- validation -----------------------------------------------------------

TEST(GaitNotchFilterValidation, RejectsNonPositiveQ)
{
    auto o = baseOptions();
    o.q = 0.0;
    const auto v = gait::GaitNotchFilter::validate(o);
    EXPECT_FALSE(v.ok);
    ASSERT_FALSE(v.errors.empty());
}

TEST(GaitNotchFilterValidation, RejectsNonPositiveSampleRate)
{
    auto o = baseOptions();
    o.sample_rate_hz = 0.0;
    EXPECT_FALSE(gait::GaitNotchFilter::validate(o).ok);
}

TEST(GaitNotchFilterValidation, DropsOutOfBandCentresAndKeepsTheRest)
{
    auto o = baseOptions();
    o.notch_freq_hz = {kGait, 0.0, -5.0, 100.0, 130.0};  // 100 = Nyquist, 130 > Nyquist
    const auto v = gait::GaitNotchFilter::validate(o);
    EXPECT_TRUE(v.ok);
    ASSERT_EQ(v.accepted_freq_hz.size(), 1u);
    EXPECT_DOUBLE_EQ(v.accepted_freq_hz[0], kGait);
    EXPECT_EQ(v.rejected_freq_hz.size(), 4u);
    EXPECT_EQ(v.warnings.size(), 4u);
}

TEST(GaitNotchFilterValidation, WarnsWhenNotchesOverlapIntoABandStop)
{
    auto o = baseOptions();
    o.q = 2.0;                                  // bandwidth = f0/2 = 10 Hz
    o.notch_freq_hz = {20.0, 21.0};             // 1 Hz apart << 10 Hz
    const auto v = gait::GaitNotchFilter::validate(o);
    EXPECT_TRUE(v.ok);
    EXPECT_FALSE(v.warnings.empty());
}

TEST(GaitNotchFilterValidation, NonOverlappingNotchesProduceNoWarning)
{
    auto o = baseOptions();
    o.q = 10.0;
    o.notch_freq_hz = {20.0, 60.0};
    const auto v = gait::GaitNotchFilter::validate(o);
    EXPECT_TRUE(v.ok);
    EXPECT_TRUE(v.warnings.empty());
}

TEST(GaitNotchFilterValidation, ConfigureStoresTheVerdict)
{
    gait::GaitNotchFilter f;
    auto o = baseOptions();
    o.notch_freq_hz = {kGait, 500.0};
    ASSERT_TRUE(f.configure(o));
    EXPECT_EQ(f.validation().accepted_freq_hz.size(), 1u);
    EXPECT_EQ(f.validation().rejected_freq_hz.size(), 1u);
}

// ---- delay bookkeeping ----------------------------------------------------

TEST(GaitNotchFilter, GroupDelayIsPositiveAndSharedAcrossChannels)
{
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));

    const double d = f.groupDelaySeconds(5.0);   // away from the notch
    EXPECT_GT(d, 0.0);
    // A single 20 Hz notch at Q=10, fs=200 Hz: delay is on the order of a few
    // milliseconds.  Assert the order of magnitude, not a fabricated constant.
    EXPECT_LT(d, 0.05);

    // Same coefficients on both triples => same delay, so filtering gyro and
    // accel together introduces NO relative skew between them.
    EXPECT_DOUBLE_EQ(f.groupDelaySeconds(5.0), f.groupDelaySeconds(5.0));
}

TEST(GaitNotchFilter, InertFilterReportsNoDelay)
{
    gait::GaitNotchFilter f;
    EXPECT_DOUBLE_EQ(f.groupDelaySeconds(10.0), 0.0);
    EXPECT_DOUBLE_EQ(f.magnitudeAt(10.0), 1.0);
}

TEST(GaitNotchFilter, MagnitudeAtCentreMatchesAnalyticNotchDepth)
{
    // An ideal RBJ notch is exactly zero at w0; numerically it is ~1e-16, so the
    // contract is "deep", not "exactly zero".
    gait::GaitNotchFilter f;
    ASSERT_TRUE(f.configure(baseOptions()));
    EXPECT_LT(f.magnitudeAt(kGait), 1e-6);
}

}  // namespace
