#pragma once
// ---------------------------------------------------------------------------
// Loop-gate geometry helpers.
//
// Free functions used by SimplePGO::searchForLoopPairs() and pinned by
// test/test_gate_math.cpp.  They are functions (rather than inline expressions) because each
// one encodes a convention that is easy to get wrong and that the field data cannot show:
//
//   * yawZ - Eigen's eulerAngles(a0, a1, a2) folds its FIRST angle into a half range, so a
//     -1 deg yaw comes back as +179 deg (with pitch/roll absorbing the branch).  Comparing
//     such angles with remainder() makes the yaw-consistency gate reject genuine same-heading
//     revisits whose two estimates straddle 0 deg, and accept some 180 deg-flipped
//     registrations - the exact alias class the gate exists to reject (integration review B1).
//   * withYawAboutZ - rebuild a rotation with a different yaw about Z while keeping the
//     original roll/pitch, without going through Euler angles at all: the naive
//     Rz(yaw) Ry(pitch) Rx(roll) reconstruction needs a pitch/roll extraction that has no
//     wrong-branch freedom.
//   * correctionMagnitudeM / crossSeedDisagreementM - relative translations are compared as
//     VECTORS.  Comparing their norms makes a purely lateral correction look like an odometry
//     no-op, and lets two mirror-image seeds "measure the same thing" (review N4).
// ---------------------------------------------------------------------------
#include "commons.h"

#include <cmath>

namespace pgo_loop
{

// Yaw about Z of a rotation matrix, in (-pi, pi].
inline double yawZ(const M3D &R)
{
    return std::atan2(R(1, 0), R(0, 0));
}

// R rebuilt with the given yaw about Z, keeping R's roll/pitch exactly.
// withYawAboutZ(R, yawZ(R)) == R; for R = Rz(y) Ry(p) Rx(r) it is Rz(yaw) Ry(p) Rx(r).
inline M3D withYawAboutZ(const M3D &R, double yaw_rad)
{
    return Eigen::AngleAxisd(yaw_rad - yawZ(R), Eigen::Vector3d::UnitZ()).toRotationMatrix() * R;
}

// Signed difference a - b of two yaw angles, wrapped to (-180, 180] degrees.
inline double yawDiffDeg(double yaw_a_rad, double yaw_b_rad)
{
    return std::remainder(yaw_a_rad - yaw_b_rad, 2.0 * M_PI) * 180.0 / M_PI;
}

// Magnitude [m] of the relative-translation correction a loop factor demands: the VECTOR
// difference between the odometry's relative translation and the measured one, both expressed
// in the target keyframe's body frame.  A correction purely lateral to the odometry separation
// yields the same magnitude as one along it, where ||priori|| - ||measured|| would report ~0.
inline double correctionMagnitudeM(const V3D &t_odom_prior, const V3D &t_measured)
{
    return (t_odom_prior - t_measured).norm();
}

// Disagreement [m] between the relative translations the two seeds measured (vector distance,
// so two mirror-image minima at the same distance do NOT agree).
inline double crossSeedDisagreementM(const V3D &t_a, const V3D &t_b)
{
    return (t_a - t_b).norm();
}

} // namespace pgo_loop
