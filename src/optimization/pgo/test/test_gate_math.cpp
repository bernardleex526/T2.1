// Pins the loop-gate geometry conventions that the integration review found broken:
//
//   B1 - the measured relative yaw is taken with pgo_loop::yawZ (atan2 of the rotation's first
//        column), NOT with Eigen's eulerAngles(2,1,0).  eulerAngles folds its first angle into a
//        half range, so a -1 deg yaw reads back as +179 deg (pitch/roll absorbing the branch):
//        the yaw-consistency gate then rejects genuine same-heading revisits whose two
//        estimates straddle 0 deg, and lets 180 deg-flipped registrations through.
//   N4 - relative translations are compared as VECTORS: comparing their norms makes a purely
//        lateral correction look like an odometry no-op and lets two mirror-image seeds agree.
#include <gtest/gtest.h>

#include "pgos/gate_math.h"

#include <cmath>

namespace
{
const double kDeg = M_PI / 180.0;
}

TEST(GateMath, YawZRecoversYawInEveryQuadrant)
{
    // eulerAngles(2,1,0) cannot return a negative yaw for any of the negative cases below.
    for (const double deg : {-179.0, -170.0, -91.0, -1.0, 0.0, 1.0, 91.0, 170.0, 179.0})
    {
        const M3D R = Eigen::AngleAxisd(deg * kDeg, V3D::UnitZ()).toRotationMatrix();
        EXPECT_NEAR(pgo_loop::yawZ(R) / kDeg, deg, 1e-9) << "deg=" << deg;
    }
    // +-180 deg is the wrap point: (0, -1) first column, atan2(0, -1) = +pi.
    const M3D R180 = Eigen::AngleAxisd(180.0 * kDeg, V3D::UnitZ()).toRotationMatrix();
    EXPECT_NEAR(std::abs(pgo_loop::yawZ(R180)) / kDeg, 180.0, 1e-9);
}

TEST(GateMath, SameHeadingRevisitAcrossZeroDegIsAccepted)
{
    // The gate's job: two estimates of a ~0 deg relative yaw straddling 0 deg.  With the folded
    // eulerAngles extraction these come out as +1 deg and +179 deg -> |dyaw| = 178 deg, and the
    // gate (15 deg) rejected exactly the most common revisit geometry.
    const double dyaw = pgo_loop::yawDiffDeg(-1.0 * kDeg, 1.0 * kDeg);
    EXPECT_NEAR(dyaw, -2.0, 1e-9);
    EXPECT_LT(std::abs(dyaw), 15.0);
}

TEST(GateMath, YawFlippedRegistrationIsRejected)
{
    // A 180 deg-flipped registration (the alias class the gate exists to catch) must show up as
    // ~180 deg, not as 0: prior +5 deg vs measured -175 deg are the same branch after folding.
    EXPECT_NEAR(std::abs(pgo_loop::yawDiffDeg(-175.0 * kDeg, 5.0 * kDeg)), 180.0, 1e-9);
    // And the difference is wrapped, so +179 deg vs -179 deg is 2 deg, not 358 deg.
    EXPECT_NEAR(pgo_loop::yawDiffDeg(179.0 * kDeg, -179.0 * kDeg), -2.0, 1e-9);
}

TEST(GateMath, WithYawAboutZReplacesYawAndKeepsRollPitch)
{
    const double yaw_odom = -30.0 * kDeg, pitch = 4.0 * kDeg, roll = -2.5 * kDeg;
    const M3D R_odom = (Eigen::AngleAxisd(yaw_odom, V3D::UnitZ()) *
                        Eigen::AngleAxisd(pitch, V3D::UnitY()) *
                        Eigen::AngleAxisd(roll, V3D::UnitX()))
                           .toRotationMatrix();
    // Substituting the same yaw is the identity.
    EXPECT_LT((pgo_loop::withYawAboutZ(R_odom, yaw_odom) - R_odom).norm(), 1e-12);

    // Substituting the descriptor yaw equals the ZYX reconstruction with that yaw.
    const double yaw_desc = 40.0 * kDeg;
    const M3D expected = (Eigen::AngleAxisd(yaw_desc, V3D::UnitZ()) *
                          Eigen::AngleAxisd(pitch, V3D::UnitY()) *
                          Eigen::AngleAxisd(roll, V3D::UnitX()))
                             .toRotationMatrix();
    EXPECT_LT((pgo_loop::withYawAboutZ(R_odom, yaw_desc) - expected).norm(), 1e-12);

    // The tilt (what is left after removing the yaw) is the odometry's tilt, unchanged - this
    // is what a folded eulerAngles decomposition got wrong for negative relative yaws, where
    // pitch/roll came back near +-pi and seeded the coarse stage with a flipped rotation.
    const M3D tilt_new = Eigen::AngleAxisd(-yaw_desc, V3D::UnitZ()).toRotationMatrix() *
                         pgo_loop::withYawAboutZ(R_odom, yaw_desc);
    const M3D tilt_old = Eigen::AngleAxisd(-yaw_odom, V3D::UnitZ()).toRotationMatrix() * R_odom;
    EXPECT_LT((tilt_new - tilt_old).norm(), 1e-12);
}

TEST(GateMath, CorrectionMagnitudeCountsLateralCorrections)
{
    const V3D prior(2.0, 0.0, 0.0);
    const V3D measured(2.0, 0.3, 0.0); // 0.30 m to the side of the odometry separation

    // The old metric: |‖prior‖ - ‖measured‖| = 0.0224 m, below the 0.15 m no-op floor, so a
    // real 30 cm correction was discarded as "odometry agrees".
    EXPECT_NEAR(std::abs(prior.norm() - measured.norm()), 0.0224, 1e-3);
    EXPECT_LT(std::abs(prior.norm() - measured.norm()), 0.15);

    // The gate now sees the correction it actually is.
    EXPECT_NEAR(pgo_loop::correctionMagnitudeM(prior, measured), 0.30, 1e-12);
    EXPECT_GE(pgo_loop::correctionMagnitudeM(prior, measured), 0.15);
    // Identical translations stay a no-op.
    EXPECT_NEAR(pgo_loop::correctionMagnitudeM(prior, prior), 0.0, 1e-12);
    // Along-track corrections are unaffected.
    EXPECT_NEAR(pgo_loop::correctionMagnitudeM(prior, V3D(2.5, 0.0, 0.0)), 0.5, 1e-12);
}

TEST(GateMath, CrossSeedAgreementUsesVectorsNotNorms)
{
    const V3D a(2.0, 0.0, 0.0), b(0.0, 2.0, 0.0); // mirror-image minima

    // Same distance from the target -> the old norm comparison reported a perfect agreement.
    EXPECT_NEAR(a.norm() - b.norm(), 0.0, 1e-12);

    // The vector distance is 2.83 m, far beyond the 1.0 m agreement bound.
    EXPECT_NEAR(pgo_loop::crossSeedDisagreementM(a, b), std::sqrt(8.0), 1e-12);
    EXPECT_FALSE(pgo_loop::crossSeedDisagreementM(a, b) <= 1.0);
    // Nearby results still agree.
    EXPECT_TRUE(pgo_loop::crossSeedDisagreementM(a, V3D(2.0, 0.1, 0.0)) <= 1.0);
}
