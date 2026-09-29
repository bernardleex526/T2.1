// Deterministic regression tests for the two localizer defects the integration review found:
//
//   B2 - the physical-consistency gate of the accepted ICP updates.  Pinned here: the
//        innovation is measured on the BODY POSE (no lever-arm growth with distance from the
//        odom origin), the reference is the last ACCEPTED RAW candidate (not the published
//        EMA, which made steady drift look like a violation and turned the gate absorbing),
//        and rejection is bounded - after N consecutive rejects the gate reports `lost` and
//        re-seeds instead of staying silently "valid" forever.
//   N3 - the fitness/inlier measurement of the scan against the map (PCL's
//        getFitnessScore(max_range) takes a SQUARED distance and averages over the in-range
//        subset only, so a half-matching scan scores like a full one).
#include <gtest/gtest.h>

#include "localizers/fitness.h"
#include "localizers/outlier_gate.h"

#include <cmath>

namespace
{
M3D rotZ(double rad) { return Eigen::AngleAxisd(rad, V3D::UnitZ()).toRotationMatrix(); }

OffsetGateConfig testConfig()
{
    OffsetGateConfig cfg;
    cfg.max_translation_m = 0.04;
    cfg.max_angle_rad = 0.03;
    cfg.max_consecutive_rejects = 3;
    cfg.ema_alpha = 0.10;
    return cfg;
}
} // namespace

// ---------------------------------------------------------------------------
// B2: the innovation must not grow with the distance from the odom origin.
// ---------------------------------------------------------------------------
TEST(OffsetGate, RotationInnovationDoesNotScaleWithLeverArm)
{
    for (const double d : {1.0, 40.0})
    {
        OffsetOutlierGate gate(testConfig());
        const M3D raw_r = M3D::Identity();
        const V3D raw_t = V3D::Zero();
        const V3D body_t(d, 0.0, 0.0);
        gate.reset(raw_r, raw_t); // engage the gate on the reference, as after a relocalize

        // Raw: the transform is the identity offset, i.e. the body pose is the odometry pose.
        // Candidate: the SAME body pose, but its map<-odom offset carries a 1 mrad rotation
        // error - which shifts the offset translation by ~d * 1 mrad (0.04 m at 40 m, half the
        // gate, purely from the lever arm).
        const double eps = 1e-3;
        const M3D cand_r = rotZ(eps);
        const V3D cand_t = body_t - cand_r * body_t;
        EXPECT_NEAR((cand_t - raw_t).norm(), eps * d, 1e-3); // what the old metric measured

        // The gate compares the implied body poses: identical, so the innovation is zero at
        // both distances and the candidate is accepted.
        const OffsetGateOutcome out = gate.update(cand_r, cand_t, body_t);
        EXPECT_TRUE(out.accepted) << "d=" << d;
        EXPECT_LE(out.innovation_m, 1e-12) << "d=" << d;
        EXPECT_NEAR(out.innovation_rad, eps, 1e-9) << "d=" << d;
    }
}

TEST(OffsetGate, BodyPoseTranslationInnovationIsGated)
{
    OffsetOutlierGate gate(testConfig());
    const V3D body_t(5.0, 0.0, 0.0);
    const OffsetGateOutcome first = gate.update(M3D::Identity(), V3D::Zero(), body_t);
    EXPECT_TRUE(first.accepted); // not engaged yet: first lock

    // 0.02 m of body-pose motion: accepted.
    const OffsetGateOutcome ok = gate.update(M3D::Identity(), V3D(0.02, 0.0, 0.0), body_t);
    EXPECT_TRUE(ok.accepted);
    EXPECT_NEAR(ok.innovation_m, 0.02, 1e-12);

    // 0.5 m at once: rejected, and the published (smoothed) transform does not move.
    const V3D published_before = gate.published_t();
    const OffsetGateOutcome bad = gate.update(M3D::Identity(), V3D(0.52, 0.0, 0.0), body_t);
    EXPECT_FALSE(bad.accepted);
    EXPECT_NEAR(bad.innovation_m, 0.5, 1e-12);
    EXPECT_LT((gate.published_t() - published_before).norm(), 1e-12);
    EXPECT_EQ(bad.consecutive_rejects, 1);
}

// ---------------------------------------------------------------------------
// B2: a steady drift must not be turned into a rejection loop by an EMA reference.
// ---------------------------------------------------------------------------
TEST(OffsetGate, SteadyDriftStaysAcceptedAgainstTheLastRawCandidate)
{
    OffsetOutlierGate gate(testConfig());
    const V3D body_t(5.0, 0.0, 0.0);
    gate.update(M3D::Identity(), V3D::Zero(), body_t); // first lock

    for (int i = 1; i <= 50; ++i)
    {
        const V3D t(0.02 * i, 0.0, 0.0); // 2 cm per update, 1 m total
        const OffsetGateOutcome out = gate.update(M3D::Identity(), t, body_t);
        EXPECT_TRUE(out.accepted) << "i=" << i;
        // Against the raw reference the innovation is one step, not the accumulated lag an EMA
        // reference would show (which is ~0.18 m here and would reject from i=3 on).
        EXPECT_NEAR(out.innovation_m, 0.02, 1e-12) << "i=" << i;
        EXPECT_EQ(out.consecutive_rejects, 0) << "i=" << i;
    }
    EXPECT_GT(gate.published_t().x(), 0.0);
    EXPECT_LT(gate.published_t().x(), 1.0); // the published transform lags the raw one
}

// ---------------------------------------------------------------------------
// B2: bounded recovery instead of an absorbing gate.
// ---------------------------------------------------------------------------
TEST(OffsetGate, ConsecutiveRejectsDeclareLostAndReseed)
{
    OffsetOutlierGate gate(testConfig());
    const V3D body_t(5.0, 0.0, 0.0);
    gate.update(M3D::Identity(), V3D::Zero(), body_t); // first lock

    const V3D jump(1.0, 0.0, 0.0); // 1 m away from the reference: rejected
    for (int i = 1; i <= 2; ++i)
    {
        const OffsetGateOutcome out = gate.update(M3D::Identity(), jump, body_t);
        EXPECT_FALSE(out.accepted) << "i=" << i;
        EXPECT_FALSE(out.lost) << "i=" << i;
        EXPECT_EQ(gate.consecutiveRejects(), i);
    }

    // The third rejection crosses max_consecutive_rejects: report lost and re-seed the internal
    // tracking REFERENCE on the candidate...
    const V3D trusted = gate.published_t();
    const OffsetGateOutcome lost = gate.update(M3D::Identity(), jump, body_t);
    EXPECT_TRUE(lost.lost);
    EXPECT_EQ(lost.consecutive_rejects, 0); // reset by the re-seed
    EXPECT_LT((gate.reference_t() - jump).norm(), 1e-12);
    // ... while the PUBLISHED transform stays at the last trusted value: a rejected candidate
    // must never reach TF (see the dedicated test below).
    EXPECT_LT((gate.published_t() - trusted).norm(), 1e-12);

    // Tracking resumes from the new reference instead of rejecting everything forever.
    const OffsetGateOutcome resumed = gate.update(M3D::Identity(), jump + V3D(0.01, 0.0, 0.0),
                                                  body_t);
    EXPECT_TRUE(resumed.accepted);
    EXPECT_TRUE(gate.awaitingRecovery());
}

TEST(OffsetGate, RejectedCandidateNeverBecomesThePublishedTransform)
{
    OffsetGateConfig cfg = testConfig();
    cfg.recovery_accepts = 2;
    OffsetOutlierGate gate(cfg);
    const V3D body_t(5.0, 0.0, 0.0);

    // Operator lock: published = the accepted candidate, valid.
    gate.update(M3D::Identity(), V3D::Zero(), body_t, true);
    EXPECT_TRUE(gate.valid());
    const V3D trusted = gate.published_t();

    // 3 consecutive rejections of a 1 m jump -> lost.  The rejected candidate becomes the
    // internal reference but NOT the published transform (TF carries no validity flag, so a
    // consumer that ignores relocalize_check would otherwise be handed a pose the gate itself
    // rejected).
    const V3D jump(1.0, 0.0, 0.0);
    for (int i = 0; i < 3; ++i)
        gate.update(M3D::Identity(), jump, body_t);
    EXPECT_FALSE(gate.valid());
    EXPECT_LT((gate.published_t() - trusted).norm(), 1e-12);
    EXPECT_LT((gate.reference_t() - jump).norm(), 1e-12);

    // Plausible updates while degraded keep following the tracker internally, but the output
    // stays frozen until the lock has recovered.
    OffsetGateOutcome out = gate.update(M3D::Identity(), jump + V3D(0.01, 0.0, 0.0), body_t);
    EXPECT_TRUE(out.accepted);
    EXPECT_FALSE(out.recovered);
    EXPECT_LT((gate.published_t() - trusted).norm(), 1e-12);

    // Recovery releases the output - snapped to the recovered pose, not blended through the
    // frozen one.
    out = gate.update(M3D::Identity(), jump + V3D(0.02, 0.0, 0.0), body_t);
    EXPECT_TRUE(out.recovered);
    EXPECT_TRUE(gate.valid());
    EXPECT_LT((gate.published_t() - (jump + V3D(0.02, 0.0, 0.0))).norm(), 1e-12);
}

// ---------------------------------------------------------------------------
// B2 / NB1: the node-level lifecycle.  These drive exactly the sequence localizer_node's
// timerCB feeds the gate, including the transition that used to break: after `lost` the lock
// is INVALID but the gate must KEEP GATING, and a run of accepted updates must restore it to
// valid without any service call.
// ---------------------------------------------------------------------------
TEST(OffsetGate, ValidOnlyAfterServiceRequestThenLostThenRecovered)
{
    OffsetGateConfig cfg = testConfig();
    cfg.recovery_accepts = 3;
    OffsetOutlierGate gate(cfg);
    const V3D body_t(5.0, 0.0, 0.0);

    // Startup: the first successful ICP result becomes the reference but the lock is NOT
    // reported valid - the operator has not relocalized it yet (upstream behaviour).
    OffsetGateOutcome out = gate.update(M3D::Identity(), V3D::Zero(), body_t, false);
    EXPECT_TRUE(out.locked);
    EXPECT_FALSE(gate.valid());

    // Operator relocalize: the next adopted candidate locks it and it is valid.
    out = gate.update(M3D::Identity(), V3D(0.01, 0.0, 0.0), body_t, true);
    EXPECT_TRUE(out.locked);
    EXPECT_TRUE(gate.valid());

    // The lock then degrades: 3 consecutive rejects (max_consecutive_rejects) -> lost.
    const V3D jump(1.0, 0.0, 0.0);
    for (int i = 1; i <= 3; ++i)
        out = gate.update(M3D::Identity(), jump, body_t, false);
    EXPECT_TRUE(out.lost);
    EXPECT_FALSE(gate.valid());      // relocalize_check now reports INVALID
    EXPECT_TRUE(gate.awaitingRecovery());

    // ... but the gate is still active: an implausible candidate is rejected, NOT adopted.
    out = gate.update(M3D::Identity(), jump + V3D(3.0, 0.0, 0.0), body_t, false);
    EXPECT_FALSE(out.accepted);
    EXPECT_FALSE(out.locked);
    EXPECT_FALSE(gate.valid());

    // 3 consecutive plausible updates re-validate the lock, with no service call.
    for (int i = 1; i <= 2; ++i)
    {
        out = gate.update(M3D::Identity(), jump + V3D(0.01 * i, 0.0, 0.0), body_t, false);
        EXPECT_TRUE(out.accepted);
        EXPECT_FALSE(out.recovered);
    }
    out = gate.update(M3D::Identity(), jump + V3D(0.03, 0.0, 0.0), body_t, false);
    EXPECT_TRUE(out.recovered);
    EXPECT_TRUE(gate.valid());

    // A later relocalize request invalidates the lock until the next adopted candidate.
    gate.requestRelock();
    EXPECT_FALSE(gate.valid());
    out = gate.update(M3D::Identity(), V3D(9.0, 9.0, 0.0), body_t, true);
    EXPECT_TRUE(out.locked);
    EXPECT_TRUE(gate.valid());
}

TEST(OffsetGate, ResetRelocksOnALargeJump)
{
    OffsetOutlierGate gate(testConfig());
    const V3D body_t(5.0, 0.0, 0.0);
    gate.update(M3D::Identity(), V3D::Zero(), body_t);

    // A relocalize request resets the gate: a candidate near the NEW reference is accepted
    // (1 cm), where the same candidate is ~10 m from the old reference and would otherwise be
    // rejected forever.
    gate.reset(rotZ(1.0), V3D(10.0, 10.0, 0.0));
    const OffsetGateOutcome out = gate.update(rotZ(1.0), V3D(10.0, 10.01, 0.0), body_t);
    EXPECT_TRUE(out.accepted);
    EXPECT_NEAR(out.innovation_m, 0.01, 1e-12);
}

// ---------------------------------------------------------------------------
// N3: fitness + inlier ratio of the scan against the map.
// ---------------------------------------------------------------------------
namespace
{
CloudType::Ptr grid(double side, double spacing, double dx, double dy, int n_side, int first, int last)
{
    CloudType::Ptr c(new CloudType);
    for (int i = 0; i < n_side * n_side; ++i)
    {
        if (i < first || i >= last)
            continue;
        PointType p;
        p.x = static_cast<float>((i % n_side) * spacing + dx);
        p.y = static_cast<float>((i / n_side) * spacing + dy);
        p.z = static_cast<float>(side);
        p.intensity = 1.0f;
        c->push_back(p);
    }
    return c;
}

// 1-D strip along x (0.01 m spacing), offset in y.
CloudType::Ptr strip(double y)
{
    CloudType::Ptr c(new CloudType);
    for (int i = 0; i < 400; ++i)
    {
        PointType p;
        p.x = static_cast<float>(i * 0.01);
        p.y = static_cast<float>(y);
        p.z = 0.0f;
        p.intensity = 1.0f;
        c->push_back(p);
    }
    return c;
}
} // namespace

TEST(FitnessAndInliers, PerfectMatchScoresZeroWithFullOverlap)
{
    CloudType::Ptr tgt = grid(0.0, 0.1, 0.0, 0.0, 20, 0, 400);
    CloudType::Ptr src = grid(0.0, 0.1, 0.0, 0.0, 20, 0, 400);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(tgt);
    double score = -1.0, ratio = -1.0;
    ASSERT_TRUE(fitnessAndInliers(tree, src, 0.35, &score, &ratio));
    EXPECT_NEAR(score, 0.0, 1e-12);
    EXPECT_NEAR(ratio, 1.0, 1e-12);
}

TEST(FitnessAndInliers, PartialMatchIsExposedByTheInlierRatio)
{
    CloudType::Ptr tgt = grid(0.0, 0.1, 0.0, 0.0, 20, 0, 400);
    // Half the scan is the mapped room, half is 5 m away (unmapped geometry / a different
    // place).  PCL's getFitnessScore(0.35) would average over the matching half only and
    // report a perfect score; the ratio exposes that only half the scan matched.
    CloudType::Ptr src = grid(0.0, 0.1, 0.0, 0.0, 20, 0, 400);
    for (int i = 200; i < 400; ++i)
        src->points[i].x += 5.0f;

    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(tgt);
    double score = -1.0, ratio = -1.0;
    ASSERT_TRUE(fitnessAndInliers(tree, src, 0.35, &score, &ratio));
    EXPECT_NEAR(score, 0.0, 1e-12); // the matched half is exact
    EXPECT_NEAR(ratio, 0.5, 1e-12); // ... but only half of the scan is there
    EXPECT_LT(ratio, 0.9);
}

TEST(FitnessAndInliers, ShiftedScanDropsOutOfTheRadius)
{
    // A strip along x: unique in the (x,y) plane, so a rigid y-shift cannot land back on the
    // cloud (a periodic lattice would - any multiple of the lattice constant keeps the nearest
    // neighbour at 0 m, which is what a wrong test fixture looked like here at first).
    CloudType::Ptr tgt = strip(0.0);
    CloudType::Ptr src = strip(0.5); // every point 0.5 m off in y
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(tgt);
    double score = -1.0, ratio = -1.0;
    ASSERT_TRUE(fitnessAndInliers(tree, src, 0.35, &score, &ratio));
    EXPECT_NEAR(ratio, 0.0, 1e-12);
    EXPECT_GT(score, 4.0); // no inlier at all: the score is the "nothing matched" sentinel

    // Inside a 0.6 m radius the same shift is a full, if poor, match: 0.5 m -> 0.25 m^2.
    ASSERT_TRUE(fitnessAndInliers(tree, src, 0.6, &score, &ratio));
    EXPECT_NEAR(ratio, 1.0, 1e-12);
    EXPECT_NEAR(score, 0.25, 1e-9);
}
