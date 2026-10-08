#pragma once
#include "ieskf.h"
#include "commons.h"
#include "gait_filter.h"
#include <chrono>
#include <rclcpp/logging.hpp>

// 2阶 Butterworth 低通滤波器 (Direct Form II Transposed)
struct ButterworthLPF
{
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
    V3D s1 = V3D::Zero();
    V3D s2 = V3D::Zero();
    bool initialized = false;

    void setup(double fc_hz, double fs_hz)
    {
        if (fc_hz <= 0.0 || fs_hz <= 0.0 || fc_hz >= 0.5 * fs_hz)
        {
            initialized = false;
            return;
        }
        double k = std::tan(M_PI * fc_hz / fs_hz);
        double k2 = k * k;
        double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k2);
        b0 = k2 * norm;
        b1 = 2.0 * b0;
        b2 = b0;
        a1 = 2.0 * (k2 - 1.0) * norm;
        a2 = (1.0 - std::sqrt(2.0) * k + k2) * norm;
        s1.setZero();
        s2.setZero();
        initialized = true;
    }

    V3D filter(const V3D &in)
    {
        if (!initialized) return in;
        V3D out = b0 * in + s1;
        s1 = b1 * in - a1 * out + s2;
        s2 = b2 * in - a2 * out;
        return out;
    }

    void reset()
    {
        s1.setZero();
        s2.setZero();
    }
};

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

    void setLogger(rclcpp::Logger logger) { m_logger = logger; }

    // Drops the gait-notch integrator state.  The state encodes the sample
    // sequence that produced it; after a stream discontinuity (out-of-order
    // message, buffer flush) that sequence has ended and the stale state would
    // ring into the new one.  No-op when the gait filter is disabled.
    void resetGaitFilter() { m_gait_filter.reset(); }
    void resetAccLpf() { m_acc_lpf.reset(); }

    // True once a usable notch cascade is installed (config opted in AND the
    // frequencies survived validation).  Exposed for the startup log and for
    // tests; false means the node behaves exactly as before the feature.
    bool gaitFilterActive() const { return m_gait_filter.valid(); }

private:
    Config m_config;
    bool m_pushed;
    std::shared_ptr<IESKF> m_kf;
    double m_last_propagate_end_time;
    Vec<IMUData> m_imu_cache;
    Vec<Pose> m_poses_cache;
    V3D m_last_acc;
    V3D m_last_gyro;
    M12D m_Q;
    IMUData m_last_imu;
    // C2.1: scale applied to every accel sample consumed by the filter, measured once at
    // initialization as 9.81/|mean accel of the init window| (1.0 when acc_normalize is off).
    double m_acc_norm_scale = 1.0;
    // Quadruped gait compensation (opt-in; inert unless configured).  Applied at the single
    // ingestion point below so every sample reaches the estimator through exactly one filter
    // pass - filtering in both initialize() and undistort() would run the same sample twice.
    gait::GaitNotchFilter m_gait_filter;
    ButterworthLPF m_acc_lpf;
    void configureAccLpf();

    // Configures the notch cascade from m_config and logs the verdict.  Called once from the
    // constructor.  When the config is disabled or invalid the filter stays inert and the IMU
    // path is byte-identical to the historical behaviour.
    void configureGaitFilter();

    // Single entry point for new IMU samples: applies the gait compensation (if active) and
    // appends to m_imu_cache.  All cache growth goes through here.
    void ingestImu(const Vec<IMUData> &samples);

    rclcpp::Logger m_logger;
};