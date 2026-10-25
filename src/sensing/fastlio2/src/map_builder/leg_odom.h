#pragma once
// 腿式/轮式里程计观测换算到 IMU 点（刚体运动学，lio_node 的 use_leg_odom 路径使用）。
//
// T_IB = (R_ib, t_ib)：base_link 在 IMU(body) 系下的位姿，p_imu = R_ib * p_base + t_ib，
// 与 quadruped_description 的 imu_* 安装参数(IMU 在 base_link 系下)互逆。
// 刚体上两点速度关系 v_P = v_Q + w x (P - Q)；取 P = IMU 原点、Q = base 原点，在 IMU 系下
// P - Q = 0 - t_ib = -t_ib，因此
//     v_imu = R_ib * v_base - w_imu x t_ib
// （v_base 为 base_link 系下的 base 原点速度，w_imu 为 IMU 系下的去零偏角速度）。
// 曾经写成 "+ w x t_ib"：转弯时观测误差为 2|w||t_ib|（0.25 m 杆臂、1 rad/s 即 0.5 m/s）。
#include "commons.h"

inline V3D legVelocityAtImu(const M3D &r_ib, const V3D &t_ib, const V3D &v_base, const V3D &w_imu)
{
    return r_ib * v_base - w_imu.cross(t_ib);
}
