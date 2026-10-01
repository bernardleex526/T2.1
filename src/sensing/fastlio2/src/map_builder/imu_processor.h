#pragma once
#include "ieskf.h"
#include "commons.h"
#include <chrono>
#include <rclcpp/logging.hpp>

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

    void setLogger(rclcpp::Logger logger) { m_logger = logger; }

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

    rclcpp::Logger m_logger;
};