// Regressions for the opt-in CURRENT-POSE consumer mode (map_pose_current.hpp).
//
// Pinned here:
//   * the composed pose is T_map_odom * T_odom_body and its stamp is the ODOMETRY'S OWN MEASUREMENT
//     STAMP - never the node clock (restamping a propagated pose is what made the default mode
//     claim a pose the robot had already left);
//   * stale odometry is UNAVAILABLE, not extrapolated;
//   * the correction's age and validity are reported separately against the declared 1.0 s bound.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <nav_msgs/msg/odometry.hpp>

#include "map_pose_current.hpp"

namespace
{
geometry_msgs::msg::TransformStamped tr(const std::string &frame, const std::string &child,
                                        double stamp_s, double x, double y, double z,
                                        double yaw = 0.0)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = frame;
    t.child_frame_id = child;
    t.header.stamp.sec = static_cast<int32_t>(std::floor(stamp_s));
    // round, do not truncate: a truncated nanosecond field would make the stamp 1e-9 short and the
    // "stamp is preserved exactly" assertions would compare against the wrong value
    t.header.stamp.nanosec =
        static_cast<uint32_t>(std::llround((stamp_s - std::floor(stamp_s)) * 1e9));
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);
    return t;
}

// identity lever arm for the odom<-body->base_link chain used by the other cases
geometry_msgs::msg::TransformStamped identityLever()
{
    auto t = tr("body", "base_link", 0.0, 0.0, 0.0, 0.0);
    return t;
}

} // namespace

TEST(CurrentPose, ComposesAndKeepsTheOdometryMeasurementStamp)
{
    CurrentPoseConfig cfg;
    // correction: map<-odom with a 0.5 m offset and a 90 deg yaw, received for a frame at t=100
    const auto map_odom = tr("map", "odom", 100.0, 10.0, 0.0, 0.5, M_PI / 2.0);
    // odometry: odom<-body at t=100.1 (the newer measurement the consumer actually has)
    const auto odom_body = tr("odom", "body", 100.1, 1.0, 0.0, 0.0);

    const auto r = currentPose(map_odom, odom_body, identityLever(), 100.12, cfg, true, 0.02);
    ASSERT_TRUE(r.available);
    // stamp = the odometry's measurement stamp, NOT now (100.12)
    EXPECT_NEAR(stampSeconds(r.stamp), 100.1, 1e-12);
    EXPECT_NE(stampSeconds(r.stamp), 100.12);
    // the odometry translation is rotated by the correction (90 deg: +x -> +y) then translated
    EXPECT_NEAR(r.pose.position.x, 10.0, 1e-9);
    EXPECT_NEAR(r.pose.position.y, 1.0, 1e-9);
    EXPECT_NEAR(r.pose.position.z, 0.5, 1e-9);
    // ages and validity are exposed separately
    EXPECT_NEAR(r.odom_age_s, 0.02, 1e-9);
    EXPECT_NEAR(r.correction_age_s, 0.12, 1e-9);
    EXPECT_TRUE(r.correction_valid);
}

TEST(CurrentPose, StaleOdometryIsUnavailableNotExtrapolated)
{
    CurrentPoseConfig cfg;
    cfg.odom_age_limit_s = 0.25;
    const auto map_odom = tr("map", "odom", 100.0, 1.0, 2.0, 3.0);
    const auto odom_body = tr("odom", "body", 100.0, 0.0, 0.0, 0.0);
    const auto r = currentPose(map_odom, odom_body, identityLever(), 100.5, cfg, true, 0.02); // odometry is 0.5 s old
    EXPECT_FALSE(r.available);
    EXPECT_NEAR(r.odom_age_s, 0.5, 1e-9);
}

TEST(CurrentPose, CorrectionValidityUsesTheDeclaredBound)
{
    CurrentPoseConfig cfg;
    cfg.correction_validity_timeout_s = 1.0;
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    // the odometry stays fresh in both checks, so the correction age is what is being measured
    const auto fresh = tr("odom", "body", 100.9, 0.0, 0.0, 0.0);
    EXPECT_TRUE(currentPose(map_odom, fresh, identityLever(), 100.9, cfg, true, 0.02).correction_valid);

    const auto later = tr("odom", "body", 101.5, 0.0, 0.0, 0.0);
    const auto old = currentPose(map_odom, later, identityLever(), 101.5, cfg, true, 0.02);
    EXPECT_FALSE(old.correction_valid) << "a 1.5 s old correction is outside the declared 1.0 s";
    // a stale correction still yields a pose (the odometry is fresh) but the status says so
    EXPECT_TRUE(old.available);
    EXPECT_NEAR(old.odom_age_s, 0.0, 1e-9);

    // ...and stale ODOMETRY is unavailable whatever the correction says
    const auto stale = currentPose(map_odom, fresh, identityLever(), 102.5, cfg, true, 0.02);
    EXPECT_FALSE(stale.available);
}

// The strict default: a correction the gate has not vouched for (or a stale gate answer) must
// never be propagated silently, however fresh the odometry is.
TEST(CurrentPose, GateMustVouchForTheLock)
{
    CurrentPoseConfig cfg; // require_gate_valid = true by default
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto odom_body = tr("odom", "body", 100.0, 0.0, 0.0, 0.0);

    EXPECT_FALSE(currentPose(map_odom, odom_body, identityLever(), 100.05, cfg, false, 0.01).available)
        << "gate says invalid -> nothing is published";
    EXPECT_FALSE(currentPose(map_odom, odom_body, identityLever(), 100.05, cfg, true, -1.0).available)
        << "no gate answer at all -> nothing is published";
    EXPECT_FALSE(currentPose(map_odom, odom_body, identityLever(), 100.05, cfg, true, 5.0).available)
        << "a 5 s old gate answer is not evidence";
    EXPECT_TRUE(currentPose(map_odom, odom_body, identityLever(), 100.05, cfg, true, 0.01).available);

    // an explicit opt-out exists for a consumer that wants the age-based rule only
    CurrentPoseConfig loose;
    loose.require_gate_valid = false;
    EXPECT_TRUE(currentPose(map_odom, odom_body, identityLever(), 100.05, loose, false, -1.0).available);
}

// The integration rule the node now uses: trust in a gate answer EXPIRES with its receipt age, and
// a request that never completes must not refresh it.  This is the defect the acceptance review
// found (the age was pinned to 0 on every response and never recomputed).
TEST(GateTrust, AnswerExpiresWithItsReceiptAge)
{
    GateTrust trust;
    EXPECT_FALSE(trust.trusted(100.0, 1.0)) << "no answer yet -> not trusted";
    EXPECT_LT(trust.age(100.0), 0.0);

    trust.onAnswer(true, 100.0);
    EXPECT_TRUE(trust.trusted(100.5, 1.0));
    EXPECT_NEAR(trust.age(100.5), 0.5, 1e-12);

    // ...and it expires even though no new request ever completed (the pending-service case)
    EXPECT_FALSE(trust.trusted(101.5, 1.0)) << "stale answer kept trust alive";
    EXPECT_FALSE(trust.trusted(140.0, 1.0));

    // a rejected answer is never trusted, however fresh
    GateTrust rejected;
    rejected.onAnswer(false, 100.0);
    EXPECT_FALSE(rejected.trusted(100.0, 1.0));

    // a fresh answer restores trust
    trust.onAnswer(true, 102.0);
    EXPECT_TRUE(trust.trusted(102.2, 1.0));
}

// The named integration scenario: the service answered true ONCE and then never responds again,
// while the odometry keeps arriving at 10 Hz.  Publication must stop as soon as the answer's
// receipt age passes the declared bound - the pending request must never refresh the trust.  This
// drives the same decision sequence the node's callback runs (GateTrust + currentPose).
TEST(GateTrust, PendingServiceNeverRefreshesTrustAndOutputStops)
{
    CurrentPoseConfig cfg;  // require_gate_valid = true, 1.0 s bound
    GateTrust trust;
    trust.onAnswer(true, 100.0);   // one true answer, then the service goes silent
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);

    int published = 0, stopped_at = -1;
    for (int i = 0; i < 40; ++i)   // 40 callbacks at 20 Hz = 2 s, odometry fresh throughout
    {
        const double now_s = 100.0 + i * 0.05;
        const double age = now_s - 100.0;
        const auto odom_child = tr("odom", "body", now_s - 0.02, 0.0, 0.0, 0.0);
        const bool gv = trust.trusted(now_s, cfg.correction_validity_timeout_s);
        const bool avail = currentPose(map_odom, odom_child, identityLever(), now_s, cfg,
                                       gv, trust.age(now_s)).available;
        // the exact contract: publish while the answer's receipt age is within the declared bound
        EXPECT_EQ(avail, age <= cfg.correction_validity_timeout_s) << "at age " << age;
        if (avail)
            ++published;
        else if (stopped_at < 0)
            stopped_at = i;
    }
    EXPECT_EQ(published, 21) << "callbacks up to and including the 1.0 s bound publish";
    EXPECT_EQ(stopped_at, 21) << "output must stop exactly when the bound is exceeded";
}

// A real mounting extrinsic is never identity.  The composed pose must be in BASE_LINK coordinates:
// the lever arm and its orientation are applied, and the odometry's own child frame is respected.
TEST(CurrentPose, NonIdentityLeverArmAndOrientationGiveBaseLinkPose)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 10.0, 20.0, 0.0);
    const auto odom_child = tr("odom", "body", 100.4, 1.0, 0.0, 0.0);
    // body -> base_link: 90 deg about z, offset (0.3, 0, 0.4) - a calibrated quadruped mounting
    auto lever = tr("body", "base_link", 100.4, 0.3, 0.0, 0.4);
    const double s2 = std::sqrt(0.5);
    lever.transform.rotation.z = s2;
    lever.transform.rotation.w = s2;

    const auto r = currentPose(map_odom, odom_child, lever, 100.5, cfg, true, 0.1);
    ASSERT_TRUE(r.available);
    // hand-computed: rotate (0.3,0,0.4) by identity, add (1,0,0) -> (1.3,0,0.4); add (10,20,0)
    EXPECT_NEAR(r.pose.position.x, 11.3, 1e-9);
    EXPECT_NEAR(r.pose.position.y, 20.0, 1e-9);
    EXPECT_NEAR(r.pose.position.z, 0.4, 1e-9) << "the mounting height must survive the composition";
    EXPECT_NEAR(r.pose.orientation.z, s2, 1e-9);
    EXPECT_NEAR(r.pose.orientation.w, s2, 1e-9);
    // NOT the body pose (which would be (11.0, 20.0, 0.0) with no rotation)
    EXPECT_GT(std::fabs(r.pose.position.x - 11.0), 1e-6);

    // a mismatched chain is unavailable, never a silently mislabelled pose
    auto wrong = lever;
    wrong.child_frame_id = "some_other_frame";
    EXPECT_FALSE(currentPose(map_odom, odom_child, wrong, 100.5, cfg, true, 0.1).available);
    auto wrong_parent = odom_child;
    wrong_parent.header.frame_id = "not_odom";
    EXPECT_FALSE(currentPose(map_odom, wrong_parent, lever, 100.5, cfg, true, 0.1).available);
}

// ---------------------------------------------------------------------------------------------
// Bounded prediction (predictCurrentPose): the numeric position/orientation regressions the plan
// pre-registers, and the full rejection surface.  Every rejection must be fail-closed - available
// false, predicted false, a named reason, and the QUERY stamp - never a silently extrapolated pose.
// ---------------------------------------------------------------------------------------------
namespace
{
builtin_interfaces::msg::Time timeFromSeconds(double s)
{
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(std::floor(s));
    t.nanosec = static_cast<uint32_t>(std::llround((s - std::floor(s)) * 1e9));
    return t;
}

// odom<-child odometry sample: pose (x,y,z,yaw) and the CHILD-frame linear twist (vx,vy,vz)
nav_msgs::msg::Odometry odomMsg(const std::string &frame, const std::string &child, double stamp_s,
                                double x, double y, double z, double yaw,
                                double vx, double vy, double vz)
{
    nav_msgs::msg::Odometry o;
    o.header.frame_id = frame;
    o.child_frame_id = child;
    o.header.stamp = timeFromSeconds(stamp_s);
    o.pose.pose.position.x = x;
    o.pose.pose.position.y = y;
    o.pose.pose.position.z = z;
    o.pose.pose.orientation.z = std::sin(yaw / 2.0);
    o.pose.pose.orientation.w = std::cos(yaw / 2.0);
    o.twist.twist.linear.x = vx;
    o.twist.twist.linear.y = vy;
    o.twist.twist.linear.z = vz;
    return o;
}

double yawOf(const geometry_msgs::msg::Quaternion &q)
{
    return 2.0 * std::atan2(q.z, q.w);
}

// a conservative default the plan's numbers assume: a valid gate answer just received
constexpr double kRecentGate = 0.01;
} // namespace

// v = (0.5,0,0), dt = 0.08 s, correction = 90 deg yaw about z -> 0.04 m of travel lands on map +y.
TEST(PredictCurrentPose, TranslatesAcrossARotatedCorrection)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0, M_PI / 2.0);
    const auto prev = odomMsg("odom", "body", 100.00, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
    const auto latest = odomMsg("odom", "body", 100.10, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
    const auto query = timeFromSeconds(100.18); // dt = 0.08

    const auto r = predictCurrentPose(map_odom, prev, latest, identityLever(), query, cfg, true,
                                      kRecentGate);
    ASSERT_TRUE(r.available);
    EXPECT_TRUE(r.predicted);
    EXPECT_EQ(r.reject_reason, "");
    EXPECT_NEAR(r.prediction_dt_s, 0.08, 1e-12);
    // 0.5 m/s * 0.08 s = 0.04 m along odom +x; the 90 deg correction rotates it onto map +y
    EXPECT_NEAR(r.pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(r.pose.position.y, 0.04, 1e-9);
    EXPECT_NEAR(r.pose.position.z, 0.0, 1e-9);
    // the published stamp is the QUERY instant, not the odometry's own stamp
    EXPECT_NEAR(stampSeconds(r.stamp), 100.18, 1e-9);
    EXPECT_NE(r.correction_age_s, r.odom_age_s);
}

// In place: h = 0.10 s, yaw 0 -> 0.10 rad, dt = 0.05 s -> predicted yaw = 0.10 + 1.0*0.05 = 0.15.
TEST(PredictCurrentPose, PredictsInPlaceRotationAtTheMeasuredRate)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto prev = odomMsg("odom", "body", 100.00, 0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0);
    const auto latest = odomMsg("odom", "body", 100.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0);
    const auto query = timeFromSeconds(100.15); // dt = 0.05

    const auto r = predictCurrentPose(map_odom, prev, latest, identityLever(), query, cfg, true,
                                      kRecentGate);
    ASSERT_TRUE(r.available);
    EXPECT_NEAR(yawOf(r.pose.orientation), 0.15, 1e-9);
    EXPECT_NEAR(r.pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(r.pose.position.y, 0.0, 1e-9);
}

// The lever arm must be carried by the PREDICTED rotation (predicting the body pose and adding a
// fixed offset would miss it): yaw 0.15 and child->base +x 1 m -> base at (cos .15, sin .15, 0).
TEST(PredictCurrentPose, AppliesTheLeverArmAfterPredicting)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto prev = odomMsg("odom", "body", 100.00, 0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0);
    const auto latest = odomMsg("odom", "body", 100.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0);
    const auto lever = tr("body", "base_link", 100.15, 1.0, 0.0, 0.0);
    const auto query = timeFromSeconds(100.15); // dt = 0.05 -> yaw 0.15

    const auto r = predictCurrentPose(map_odom, prev, latest, lever, query, cfg, true, kRecentGate);
    ASSERT_TRUE(r.available);
    EXPECT_NEAR(r.pose.position.x, std::cos(0.15), 1e-9);
    EXPECT_NEAR(r.pose.position.y, std::sin(0.15), 1e-9);
    EXPECT_NEAR(r.pose.position.z, 0.0, 1e-9);
    EXPECT_NEAR(yawOf(r.pose.orientation), 0.15, 1e-9);
}

// Zero angular rate is the identity INCREMENT: the latest orientation is kept, not extrapolated.
TEST(PredictCurrentPose, ZeroAngularRateKeepsTheLatestOrientation)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto prev = odomMsg("odom", "body", 100.00, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0);
    const auto latest = odomMsg("odom", "body", 100.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0);
    const auto query = timeFromSeconds(100.15);

    const auto r = predictCurrentPose(map_odom, prev, latest, identityLever(), query, cfg, true,
                                      kRecentGate);
    ASSERT_TRUE(r.available);
    EXPECT_NEAR(yawOf(r.pose.orientation), 0.10, 1e-9);
}

TEST(PredictCurrentPose, DtWindowIsZeroToMaxPrediction)
{
    CurrentPoseConfig cfg; // max_prediction_s = 0.12
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto prev = odomMsg("odom", "body", 100.00, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0);
    const auto latest = odomMsg("odom", "body", 100.10, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0);

    // dt = 0 (query exactly at the newest sample) is inside the window
    EXPECT_TRUE(predictCurrentPose(map_odom, prev, latest, identityLever(), timeFromSeconds(100.10),
                                   cfg, true, kRecentGate).available);
    // dt just inside the pre-registered bound is still inside
    EXPECT_TRUE(predictCurrentPose(map_odom, prev, latest, identityLever(), timeFromSeconds(100.2199),
                                   cfg, true, kRecentGate).available);
    // ...and just past it is rejected, never extrapolated further
    const auto over = predictCurrentPose(map_odom, prev, latest, identityLever(),
                                         timeFromSeconds(100.2201), cfg, true, kRecentGate);
    EXPECT_FALSE(over.available);
    EXPECT_FALSE(over.predicted);
    EXPECT_EQ(over.reject_reason, map_pose_reason::kDtOutOfRange);
    EXPECT_NEAR(stampSeconds(over.stamp), 100.2201, 1e-6);
}

TEST(PredictCurrentPose, OdometryIntervalMustBeInsideTheDeclaredWindow)
{
    CurrentPoseConfig cfg; // [0.02, 0.20]
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto query = timeFromSeconds(100.15);

    const auto too_short = predictCurrentPose(
        map_odom, odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0),
        odomMsg("odom", "body", 100.01, 0, 0, 0, 0, 0, 0, 0), identityLever(), query, cfg, true,
        kRecentGate);
    EXPECT_FALSE(too_short.available);
    EXPECT_EQ(too_short.reject_reason, map_pose_reason::kOdomInterval);

    const auto too_long = predictCurrentPose(
        map_odom, odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0),
        odomMsg("odom", "body", 100.25, 0, 0, 0, 0, 0, 0, 0), identityLever(),
        timeFromSeconds(100.30), cfg, true, kRecentGate);
    EXPECT_FALSE(too_long.available);
    EXPECT_EQ(too_long.reject_reason, map_pose_reason::kOdomInterval);
}

TEST(PredictCurrentPose, OdometrySamplesMustStrictlyIncrease)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 0.0, 0.0, 0.0);
    const auto r = predictCurrentPose(
        map_odom, odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0),
        odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0), identityLever(),
        timeFromSeconds(100.15), cfg, true, kRecentGate);
    EXPECT_FALSE(r.available);
    EXPECT_EQ(r.reject_reason, map_pose_reason::kOdomOrder);
}

TEST(PredictCurrentPose, FutureInputsAreRejected)
{
    CurrentPoseConfig cfg;
    const auto prev = odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0);
    const auto latest = odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0);

    // a correction measured after the query is not usable evidence
    const auto future_corr = predictCurrentPose(tr("map", "odom", 100.30, 0, 0, 0), prev, latest,
                                                identityLever(), timeFromSeconds(100.15), cfg, true,
                                                kRecentGate);
    EXPECT_FALSE(future_corr.available);
    EXPECT_EQ(future_corr.reject_reason, map_pose_reason::kFutureCorrection);

    // ...and neither is odometry measured after it (dt < 0)
    const auto future_odom = predictCurrentPose(tr("map", "odom", 100.0, 0, 0, 0), prev, latest,
                                                identityLever(), timeFromSeconds(100.05), cfg, true,
                                                kRecentGate);
    EXPECT_FALSE(future_odom.available);
    EXPECT_EQ(future_odom.reject_reason, map_pose_reason::kFutureOdom);
}

TEST(PredictCurrentPose, NonFiniteAndZeroNormQuaternionsAreRejected)
{
    CurrentPoseConfig cfg;
    const auto query = timeFromSeconds(100.15);
    const auto prev = odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0);
    auto latest = odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0);

    auto nan_corr = tr("map", "odom", 100.0, 0, 0, 0);
    nan_corr.transform.translation.x = std::nan("");
    EXPECT_EQ(predictCurrentPose(nan_corr, prev, latest, identityLever(), query, cfg, true,
                                 kRecentGate).reject_reason,
              map_pose_reason::kNonFinite);

    auto inf_twist = latest;
    inf_twist.twist.twist.linear.y = std::numeric_limits<double>::infinity();
    EXPECT_EQ(predictCurrentPose(tr("map", "odom", 100.0, 0, 0, 0), prev, inf_twist,
                                 identityLever(), query, cfg, true, kRecentGate).reject_reason,
              map_pose_reason::kNonFinite);

    auto zero_quat = latest;
    zero_quat.pose.pose.orientation.z = 0.0;
    zero_quat.pose.pose.orientation.w = 0.0;
    EXPECT_EQ(predictCurrentPose(tr("map", "odom", 100.0, 0, 0, 0), prev, zero_quat,
                                 identityLever(), query, cfg, true, kRecentGate).reject_reason,
              map_pose_reason::kZeroQuaternion);
}

TEST(PredictCurrentPose, HalfTurnRotationIsAmbiguousAndRejected)
{
    CurrentPoseConfig cfg;
    const auto prev = odomMsg("odom", "body", 100.00, 0, 0, 0, 0.0, 0, 0, 0);
    const auto latest = odomMsg("odom", "body", 100.10, 0, 0, 0, M_PI, 0, 0, 0);
    const auto r = predictCurrentPose(tr("map", "odom", 100.0, 0, 0, 0), prev, latest,
                                      identityLever(), timeFromSeconds(100.15), cfg, true,
                                      kRecentGate);
    EXPECT_FALSE(r.available);
    EXPECT_EQ(r.reject_reason, map_pose_reason::kAngleAmbiguous);

    // just inside the bound the rotation is determinate (dt = 0 keeps it exactly at the sample)
    const auto nearly = odomMsg("odom", "body", 100.10, 0, 0, 0, M_PI - 1e-4, 0, 0, 0);
    const auto ok = predictCurrentPose(tr("map", "odom", 100.0, 0, 0, 0), prev, nearly,
                                       identityLever(), timeFromSeconds(100.10), cfg, true,
                                       kRecentGate);
    EXPECT_TRUE(ok.available);
    EXPECT_NEAR(yawOf(ok.pose.orientation), M_PI - 1e-4, 1e-7);
}

TEST(PredictCurrentPose, FrameChainMustLineUpExactly)
{
    CurrentPoseConfig cfg;
    const auto map_odom = tr("map", "odom", 100.0, 0, 0, 0);
    const auto prev = odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0);
    const auto latest = odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0);
    const auto query = timeFromSeconds(100.15);

    auto wrong_map = map_odom;
    wrong_map.header.frame_id = "world";
    EXPECT_EQ(predictCurrentPose(wrong_map, prev, latest, identityLever(), query, cfg, true,
                                 kRecentGate).reject_reason,
              map_pose_reason::kFrames);

    auto wrong_child = latest;
    wrong_child.child_frame_id = "imu";
    EXPECT_EQ(predictCurrentPose(map_odom, prev, wrong_child, identityLever(), query, cfg, true,
                                 kRecentGate).reject_reason,
              map_pose_reason::kFrames);

    auto wrong_lever = identityLever();
    wrong_lever.child_frame_id = "some_other_frame";
    EXPECT_EQ(predictCurrentPose(map_odom, prev, latest, wrong_lever, query, cfg, true,
                                 kRecentGate).reject_reason,
              map_pose_reason::kFrames);
}

TEST(PredictCurrentPose, AgesAndGateAreEnforced)
{
    const auto map_odom = tr("map", "odom", 100.0, 0, 0, 0);
    const auto prev = odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0);
    const auto latest = odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0);
    const auto query = timeFromSeconds(100.18); // dt = 0.08

    // odometry age in its own right (dt is 0.08 > 0.05 but still inside the prediction window)
    CurrentPoseConfig age_cfg;
    age_cfg.odom_age_limit_s = 0.05;
    EXPECT_EQ(predictCurrentPose(map_odom, prev, latest, identityLever(), query, age_cfg, true,
                                 kRecentGate).reject_reason,
              map_pose_reason::kOdomAge);

    // correction age against the declared validity bound
    CurrentPoseConfig corr_cfg; // correction_validity_timeout_s = 1.0
    corr_cfg.correction_validity_timeout_s = 0.10; // correction is 0.18 s old
    EXPECT_EQ(predictCurrentPose(map_odom, prev, latest, identityLever(), query, corr_cfg, true,
                                 kRecentGate).reject_reason,
              map_pose_reason::kCorrectionAge);

    // gate not vouching, or a stale gate answer, both fail closed
    CurrentPoseConfig cfg;
    EXPECT_EQ(predictCurrentPose(map_odom, prev, latest, identityLever(), query, cfg, false,
                                 kRecentGate).reject_reason,
              map_pose_reason::kGate);
    EXPECT_EQ(predictCurrentPose(map_odom, prev, latest, identityLever(), query, cfg, true,
                                 5.0).reject_reason,
              map_pose_reason::kGate);
    EXPECT_EQ(predictCurrentPose(map_odom, prev, latest, identityLever(), query, cfg, true,
                                 -1.0).reject_reason,
              map_pose_reason::kGate);
}

TEST(PredictCurrentPose, RejectionsKeepTheQueryStampAndNeverClaimPrediction)
{
    CurrentPoseConfig cfg;
    const auto prev = odomMsg("odom", "body", 100.00, 0, 0, 0, 0, 0, 0, 0);
    const auto latest = odomMsg("odom", "body", 100.10, 0, 0, 0, 0, 0, 0, 0);
    const auto query = timeFromSeconds(100.40); // dt = 0.30, far outside the window
    const auto r = predictCurrentPose(tr("map", "odom", 100.0, 0, 0, 0), prev, latest,
                                      identityLever(), query, cfg, true, kRecentGate);
    EXPECT_FALSE(r.available);
    EXPECT_FALSE(r.predicted);
    EXPECT_FALSE(r.reject_reason.empty());
    EXPECT_NEAR(stampSeconds(r.stamp), 100.40, 1e-9); // the query the consumer asked about
    EXPECT_NEAR(r.prediction_dt_s, 0.30, 1e-9);       // and the dt it was rejected over
}

// The cache flush that a jump-back/out-of-order sample performs must forget the gate answer too.
TEST(GateTrust, ResetForgetsTheAnswer)
{
    GateTrust trust;
    trust.onAnswer(true, 100.0);
    EXPECT_TRUE(trust.trusted(100.1, 1.0));
    trust.reset();
    EXPECT_FALSE(trust.haveAnswer());
    EXPECT_FALSE(trust.trusted(100.1, 1.0));
    EXPECT_LT(trust.age(100.1), 0.0);
}

