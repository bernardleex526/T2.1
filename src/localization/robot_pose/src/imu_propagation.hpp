#pragma once
// IMU-propagated current pose: the alternative to the constant-velocity extrapolation of
// predictCurrentPose().  Starting from the newest LIO odometry state (odom<-body pose, body-frame
// velocity) at its stamp, integrate the raw IMU samples (body = IMU frame, the frame FastLIO2
// estimates) up to the query instant:
//     R <- R Exp(w dt),   a_w = R f + g,   p <- p + v dt + a_w dt^2 / 2,   v <- v + a_w dt
// with g = (0, 0, -|g|) (FastLIO2 gravity-aligns odom so gravity is -z).  Unlike the constant-
// velocity model this follows the gait heave and accelerations the IMU actually measured.
// IMU biases are NOT in the odometry message and are neglected: over a 0.1-0.25 s horizon a
// 0.03 m/s^2 accel bias contributes < 1 mm and a 0.002 rad/s gyro bias < 0.03 deg.
#include <cmath>
#include <deque>

#include <Eigen/Geometry>

namespace imu_prop
{

struct ImuSample
{
    double t = 0.0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  // [rad/s], body frame
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();   // specific force [m/s^2], body frame
};

struct State
{
    double t = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // odom <- body
    Eigen::Vector3d p = Eigen::Vector3d::Zero();            // odom frame
    Eigen::Vector3d v = Eigen::Vector3d::Zero();            // odom frame
};

enum class Status
{
    Ok,
    NoCoverage,  // the IMU buffer does not cover [state.t, query]
    Gap,         // two consecutive samples further apart than max_gap_s
};

inline Eigen::Quaterniond expQ(const Eigen::Vector3d &phi)
{
    const double a = phi.norm();
    if (a < 1e-12)
        return Eigen::Quaterniond(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()).normalized();
    return Eigen::Quaterniond(Eigen::AngleAxisd(a, phi / a));
}

// Integrate samples (sorted by time) from s.t to t_query, piecewise-constant: sample k holds on
// [t_k, t_{k+1}), the last sample holds up to t_query.  The coverage check requires a sample at or
// within max_gap_s before s.t and one within max_gap_s of t_query.
inline Status propagate(State &s, const std::deque<ImuSample> &imu, double t_query, double gravity,
                        double max_gap_s = 0.05)
{
    if (t_query < s.t)
        return Status::NoCoverage;
    if (imu.empty() || imu.front().t > s.t + max_gap_s || imu.back().t < t_query - max_gap_s)
        return Status::NoCoverage;
    const Eigen::Vector3d g(0.0, 0.0, -gravity);
    auto step = [&](const ImuSample &m, double dt) {
        const Eigen::Vector3d a_w = s.q * m.acc + g;
        s.p += s.v * dt + 0.5 * a_w * dt * dt;
        s.v += a_w * dt;
        s.q = (s.q * expQ(m.gyro * dt)).normalized();
    };
    for (size_t k = 0; k < imu.size(); ++k)
    {
        const double lo = std::max(imu[k].t, s.t);
        const double hi = std::min(k + 1 < imu.size() ? imu[k + 1].t : t_query, t_query);
        if (k + 1 < imu.size() && imu[k + 1].t - imu[k].t > max_gap_s && imu[k + 1].t > s.t && imu[k].t < t_query)
            return Status::Gap;
        // the interval before the first sample is covered by holding that first sample
        const double lo_eff = (k == 0) ? s.t : lo;
        if (hi > lo_eff)
            step(imu[k], hi - lo_eff);
    }
    s.t = t_query;
    return Status::Ok;
}

}  // namespace imu_prop
