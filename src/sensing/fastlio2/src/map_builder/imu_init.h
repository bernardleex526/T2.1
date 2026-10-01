#pragma once
// ---------------------------------------------------------------------------
// Static-window IMU initialization: window selection and readiness verdict.
//
// Extracted from IMUProcessor::initialize() so the decision is a pure function of the
// buffered IMU samples and can be regression-tested without ROS or a live sensor
// (test/test_imu_init.cpp).  The edge case that motivated the extraction (integration
// review B3): the max-wait fallback used to be conditioned on the gyro also being quiet, so
// a platform that vibrates without rotating (gyro quiet, accel noisy) - or a single accel
// spike - never satisfied it, initialize() returned false on every call, and m_imu_cache
// grew without bound while no scan was ever processed.
// ---------------------------------------------------------------------------
#include "commons.h"

#include <algorithm>
#include <limits>

// How the init window is chosen (config: imu_init_mode).
//   FirstBatch   - C2.2 / upstream FAST-LIO2: the whole first IMU batch, no static verdict.
//   StaticWindow - historical fork logic below (also used for the legacy window_s <= 0 path).
enum class ImuInitMode
{
    StaticWindow,
    FirstBatch
};

struct ImuInitPlan
{
    bool ready = false;          // initialize() may proceed with the window below
    bool use_static_window = false;
    bool waited_out = false;     // max wait elapsed -> quietest window, whatever the verdict
    bool is_static = false;      // static verdict on the chosen window
    size_t i0 = 0;               // chosen analysis window [i0, i1)
    size_t i1 = 0;
    double span = 0.0;           // buffered time span [s]
    double gyro_std = 0.0;       // max per-axis gyro std over the window [rad/s]
    double acc_dev = 0.0;        // max per-axis accel deviation from the window mean [m/s^2]
};

// Per-axis maximum gyro standard deviation and accel deviation from the window mean over
// [i0, i1).  Both are measured over the WINDOW only: spreading the accel deviation over the
// whole cache (whose tail may be seconds of motion in waited-out mode) would veto the
// measured-gravity adoption for a genuinely static window and inflate it in the reverse case.
inline void imuWindowStats(const Vec<IMUData> &cache, size_t i0, size_t i1,
                           double *gyro_std, double *acc_dev)
{
    V3D gm = V3D::Zero(), am = V3D::Zero(), gdev = V3D::Zero(), adev = V3D::Zero();
    const double n = static_cast<double>(i1 - i0);
    for (size_t k = i0; k < i1; ++k)
    {
        gm += cache[k].gyro;
        am += cache[k].acc;
    }
    gm /= n;
    am /= n;
    for (size_t k = i0; k < i1; ++k)
    {
        gdev += (cache[k].gyro - gm).cwiseAbs2();
        adev += (cache[k].acc - am).cwiseAbs2();
    }
    gdev = (gdev / n).cwiseSqrt();
    adev = (adev / n).cwiseSqrt();
    *gyro_std = gdev.maxCoeff();
    *acc_dev = adev.maxCoeff();
}

// Decides whether initialization can proceed and over which samples.
//   FirstBatch mode (C2.2): ready as soon as the first batch holds min_samples samples; the
//     window is the whole cache.  This is the upstream FAST-LIO2 behaviour (~0.1 s of IMU at
//     400 Hz, no static verdict, no wait) that the fork replaced with the 3 s quiet window -
//     which costs the first ~5 s of frames on a platform that is already moving and then
//     injects v = 0 anyway via the max-wait fallback.
//   static-window mode (window_s > 0): ready once a window of window_s is BOTH available in
//     time and judged static (gyro std < static_gyro_std, accel deviation < static_acc_dev);
//     after max_wait_s of buffered data it falls back to the QUIETEST window unconditionally.
//   legacy mode (window_s <= 0): ready once min_samples samples are buffered; the window
//     is the whole cache.
inline ImuInitPlan planImuInit(const Vec<IMUData> &cache, double window_s, double max_wait_s,
                               double static_gyro_std, double static_acc_dev, int min_samples,
                               ImuInitMode mode = ImuInitMode::StaticWindow)
{
    ImuInitPlan plan;
    const size_t n = cache.size();
    if (n == 0)
        return plan;
    plan.span = cache.back().time - cache.front().time;

    if (mode == ImuInitMode::FirstBatch)
    {
        // use_static_window stays false: there is no quiet-window verdict here, and
        // initialize() must not gate the measured-gravity adoption on one.
        if (n < static_cast<size_t>(std::max(1, min_samples)))
            return plan;
        plan.i0 = 0;
        plan.i1 = n;
        plan.ready = true;
        imuWindowStats(cache, plan.i0, plan.i1, &plan.gyro_std, &plan.acc_dev);
        return plan;
    }

    plan.use_static_window = window_s > 0.0;

    if (!plan.use_static_window)
    {
        if (n < static_cast<size_t>(std::max(1, min_samples)))
            return plan;
        plan.i0 = 0;
        plan.i1 = n;
        plan.ready = true;
        imuWindowStats(cache, plan.i0, plan.i1, &plan.gyro_std, &plan.acc_dev);
        return plan;
    }

    // Not enough buffered time for a window and not yet past the max wait: keep collecting.
    if (plan.span < window_s && plan.span < max_wait_s)
        return plan;

    // Quietest contiguous window of window_s inside the cache (score = per-axis gyro std).
    size_t wlen = 1;
    while (wlen < n && cache[wlen].time - cache[0].time < window_s)
        ++wlen;
    if (wlen >= n)
        wlen = n;
    size_t best_s = 0;
    double best_score = std::numeric_limits<double>::max();
    for (size_t s = 0; s + wlen <= n; ++s)
    {
        double gs = 0.0, ad = 0.0;
        imuWindowStats(cache, s, s + wlen, &gs, &ad);
        if (gs < best_score)
        {
            best_score = gs;
            best_s = s;
        }
    }
    plan.i0 = best_s;
    plan.i1 = best_s + wlen;
    imuWindowStats(cache, plan.i0, plan.i1, &plan.gyro_std, &plan.acc_dev);
    plan.is_static = plan.gyro_std < static_gyro_std && plan.acc_dev < static_acc_dev;
    // The fallback is TIME-based only: gating it on the quality verdict (as it used to be)
    // makes it unreachable for exactly the platform it exists for.
    plan.waited_out = plan.span >= max_wait_s;
    plan.ready = plan.is_static || plan.waited_out;
    return plan;
}
