// Leg/wheel odometry -> IMU-point velocity (map_builder/leg_odom.h), checked against rigid-body
// finite differences.  The lever-arm term used to have the wrong sign (+ w x t_ib): every turn fed
// the filter a velocity off by 2|w||t_ib| (0.5 m/s for a 0.25 m lever arm at 1 rad/s).
#include <gtest/gtest.h>

#include <sophus/so3.hpp>

#include "map_builder/leg_odom.h"

namespace
{
// IMU pose in world at time t: constant body rate w_I (IMU frame) and constant IMU-point velocity.
struct Motion
{
    M3D R0;
    V3D p0, v_I_w, w_I;
    void at(double t, M3D &R, V3D &p) const
    {
        R = R0 * Sophus::SO3d::exp(w_I * t).matrix();
        p = p0 + v_I_w * t;
    }
};

// What a leg odometry reports: the base_link origin velocity, expressed in base_link.
V3D baseVelocity(const Motion &m, const M3D &R_ib, const V3D &t_ib)
{
    const double h = 1e-6;
    M3D Ra, Rb;
    V3D pa, pb;
    m.at(-h, Ra, pa);
    m.at(h, Rb, pb);
    const V3D v_base_w = ((pb + Rb * t_ib) - (pa + Ra * t_ib)) / (2 * h);
    return (m.R0 * R_ib).transpose() * v_base_w;
}
} // namespace

TEST(LegOdom, LeverArmMatchesRigidBodyKinematics)
{
    Motion m{Sophus::SO3d::exp(V3D(0.2, 0.1, -0.4)).matrix(), V3D(1, 2, 0.5), V3D(0.4, 0.3, -0.05),
             V3D(0.1, -0.2, 0.8)};
    const M3D R_ib = Eigen::AngleAxisd(0.5236, V3D::UnitZ()).toRotationMatrix();
    const V3D t_ib(-0.25, 0.02, -0.09); // base_link 0.25 m behind / 0.09 m below the IMU
    const V3D v_imu = legVelocityAtImu(R_ib, t_ib, baseVelocity(m, R_ib, t_ib), m.w_I);
    EXPECT_LT((v_imu - m.R0.transpose() * m.v_I_w).norm(), 1e-8);
}

// Pure yaw rotation about the base origin: the base is still, the IMU 0.25 m ahead moves sideways
// at w * 0.25 to the LEFT (+y) for a positive yaw rate.
TEST(LegOdom, TurningInPlaceAboutTheBase)
{
    const V3D t_ib(-0.25, 0.0, 0.0);
    const V3D v = legVelocityAtImu(M3D::Identity(), t_ib, V3D::Zero(), V3D(0, 0, 1.0));
    EXPECT_NEAR(v.x(), 0.0, 1e-12);
    EXPECT_NEAR(v.y(), 0.25, 1e-12);
    EXPECT_NEAR(v.z(), 0.0, 1e-12);
}

// Without a lever arm the measurement is only rotated into the IMU frame.
TEST(LegOdom, ZeroLeverArmOnlyRotates)
{
    const M3D R_ib = Eigen::AngleAxisd(0.3, V3D::UnitX()).toRotationMatrix();
    const V3D v_base(0.7, -0.1, 0.05);
    EXPECT_LT((legVelocityAtImu(R_ib, V3D::Zero(), v_base, V3D(0.3, 0.2, -1.0)) - R_ib * v_base).norm(), 1e-12);
}
