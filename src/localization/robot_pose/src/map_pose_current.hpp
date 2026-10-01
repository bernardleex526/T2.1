#pragma once
// Explicit CURRENT-POSE consumer mode (opt-in) for map_pose_publisher.
//
// The default mode publishes the chain tf2 resolves with TimePointZero: that pose is truthful (it
// carries the resolved stamp) but it is the robot at the chain's latest COMMON time - measured
// ~0.22-0.24 s behind the clock, i.e. ~11-12 cm of travel at 0.5 m/s.  A subscriber that wants the
// robot NOW needs the accepted map correction propagated over the newest odometry it has actually
// received.
//
// Two opt-in modes live here:
//   * currentPose(...)      - propagate the last accepted correction over the newest RECEIVED
//                             odometry and publish it at the ODOMETRY'S OWN stamp.  No
//                             extrapolation; stale odometry is UNAVAILABLE.
//   * predictCurrentPose(...) - the same composition, but the odometry is carried forward to the
//                             QUERY instant over a bounded window with a constant-velocity
//                             translation and a two-sample SO(3) angular rate (the Cartographer
//                             pose-extrapolator rule: omega = Log(R_prev^T R_latest) / h).  The
//                             window is a PRE-REGISTERED short bound (max_prediction_s = 0.12 s),
//                             not a literature guarantee, and every input that does not fit the
//                             bounded, causally-received window is REJECTED with a stable reason -
//                             never silently extrapolated further, never repaired by restamping.
//
// Both modes expose the correction's age/validity on a diagnostic status topic, with the same
// declared 1.0 s bound the localizer uses for its own lock validity.
#include <cmath>
#include <string>

#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Trust in the localizer's gate answer.  The answer is evidence only for a bounded time after it
// was RECEIVED: a request that never completes must not keep an old "valid" alive, otherwise the
// consumer keeps propagating a correction the gate stopped vouching for.  The receipt time is
// stored and the age is DERIVED on every query.
//
// The receipt time MUST be a MONOTONIC clock (the node passes steady_clock): a simulated-time jump
// would otherwise make an answer look arbitrarily fresh or stale.  The pose stamps stay in the
// message clock domain, where they belong - the two domains are never mixed.
class GateTrust
{
public:
    void onAnswer(bool valid, double received_s)
    {
        m_valid = valid;
        m_received_s = received_s;
        m_have = true;
    }

    // Forget the answer entirely.  Used when the input stream is invalidated (out-of-order
    // odometry or a ROS-time jump-back): after that reset NOTHING cached may be treated as
    // evidence, so the node must re-wait for a fresh gate answer like any other input.
    void reset()
    {
        m_have = false;
        m_valid = false;
        m_received_s = 0.0;
    }

    bool haveAnswer() const { return m_have; }

    double age(double now_s) const { return m_have ? now_s - m_received_s : -1.0; }

    bool trusted(double now_s, double timeout_s) const
    {
        if (!m_have)
            return false;
        const double a = now_s - m_received_s;
        return m_valid && a >= 0.0 && a <= timeout_s;
    }

private:
    bool m_have = false;
    bool m_valid = false;
    double m_received_s = 0.0;
};

struct CurrentPoseConfig
{
    double odom_age_limit_s = 1.0;          // beyond this the newest odometry is too old to use
    double correction_validity_timeout_s = 1.0; // the localizer's declared lock-validity bound
    // STRICT by default: a pose is only published while the localizer's own gate says the lock is
    // valid AND that answer is itself fresh.  Silently propagating a correction the gate has
    // rejected (or one that has gone stale) for up to the timeout is exactly the unsafe default
    // this flag exists to prevent.
    bool require_gate_valid = true;
    std::string map_frame = "map";
    std::string base_frame = "base_link";
    // Bounds of the bounded prediction window (predictCurrentPose only).  dt = query - latest
    // odometry stamp must lie in [0, max_prediction_s]; h = latest - previous odometry stamp must
    // lie in [min_odom_interval_s, max_odom_interval_s] so the angular rate is estimated over a
    // window that neither amplifies timestamp noise (too short) nor hides a turn (too long).
    // max_prediction_s = 0.12 s is this service's PRE-REGISTERED short-window upper bound; it is
    // not a literature guarantee and must not be widened to make an acceptance case pass.
    double max_prediction_s = 0.12;
    double min_odom_interval_s = 0.02;
    double max_odom_interval_s = 0.20;
};

struct CurrentPoseResult
{
    bool available = false;
    bool correction_valid = false;
    bool gate_valid = false;
    double gate_answer_age_s = -1.0;
    double odom_age_s = -1.0;
    double correction_age_s = -1.0;
    builtin_interfaces::msg::Time stamp;    // currentPose: the odometry's own stamp;
                                            // predictCurrentPose: the QUERY stamp
    geometry_msgs::msg::Pose pose;
    // predictCurrentPose only: dt actually predicted over, whether the pose was extrapolated, and
    // the stable machine-readable token naming the first input that made the pose unavailable.
    bool predicted = false;
    double prediction_dt_s = -1.0;
    std::string reject_reason;              // "" iff available
};

// Stable rejection vocabulary carried by CurrentPoseResult::reject_reason and the publisher's
// /robot_pose_map/status "reject_reason" field.  Downstream recorders join on these tokens; do not
// reword without bumping the consumer contract.
namespace map_pose_reason
{
inline constexpr const char *kOdomNotReady = "odom_not_ready";
inline constexpr const char *kNoCorrection = "no_correction";
inline constexpr const char *kNoChildBaseTf = "no_child_base_tf";
inline constexpr const char *kFrames = "frames_mismatch";
inline constexpr const char *kNonFinite = "non_finite_input";
inline constexpr const char *kZeroQuaternion = "zero_norm_quaternion";
inline constexpr const char *kFutureCorrection = "future_correction";
inline constexpr const char *kFutureOdom = "future_odom";
inline constexpr const char *kOdomOrder = "odom_order";
inline constexpr const char *kOdomInterval = "odom_interval_out_of_range";
inline constexpr const char *kDtOutOfRange = "prediction_dt_out_of_range";
inline constexpr const char *kAngleAmbiguous = "rotation_angle_ambiguous";
inline constexpr const char *kOdomAge = "odom_age_exceeded";
inline constexpr const char *kCorrectionAge = "correction_age_exceeded";
inline constexpr const char *kGate = "gate_not_valid";
} // namespace map_pose_reason

inline double stampSeconds(const builtin_interfaces::msg::Time &t)
{
    return double(t.sec) + double(t.nanosec) * 1e-9;
}

namespace map_pose_detail
{
struct Tf
{
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double tx = 0.0, ty = 0.0, tz = 0.0;
};

inline Tf fromMsg(const geometry_msgs::msg::Transform &t)
{
    Tf out;
    out.qx = t.rotation.x; out.qy = t.rotation.y; out.qz = t.rotation.z; out.qw = t.rotation.w;
    out.tx = t.translation.x; out.ty = t.translation.y; out.tz = t.translation.z;
    return out;
}

inline bool finiteTf(const Tf &t)
{
    return std::isfinite(t.qx) && std::isfinite(t.qy) && std::isfinite(t.qz) &&
           std::isfinite(t.qw) && std::isfinite(t.tx) && std::isfinite(t.ty) &&
           std::isfinite(t.tz);
}

// Normalize the quaternion part in place.  Returns false for a zero-norm (or non-finite)
// quaternion, which must never be silently repaired into a rotation.
inline bool normalize(Tf &t)
{
    const double n2 = t.qx * t.qx + t.qy * t.qy + t.qz * t.qz + t.qw * t.qw;
    if (!(n2 > 0.0))
        return false;
    const double n = std::sqrt(n2);
    t.qx /= n; t.qy /= n; t.qz /= n; t.qw /= n;
    return true;
}

// a * b, i.e. "apply b, then a"
inline Tf compose(const Tf &a, const Tf &b)
{
    Tf out;
    out.qx = a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy;
    out.qy = a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx;
    out.qz = a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw;
    out.qw = a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz;
    const double rx = (1 - 2 * (a.qy * a.qy + a.qz * a.qz)) * b.tx +
                      2 * (a.qx * a.qy - a.qz * a.qw) * b.ty +
                      2 * (a.qx * a.qz + a.qy * a.qw) * b.tz;
    const double ry = 2 * (a.qx * a.qy + a.qz * a.qw) * b.tx +
                      (1 - 2 * (a.qx * a.qx + a.qz * a.qz)) * b.ty +
                      2 * (a.qy * a.qz - a.qx * a.qw) * b.tz;
    const double rz = 2 * (a.qx * a.qz - a.qy * a.qw) * b.tx +
                      2 * (a.qy * a.qz + a.qx * a.qw) * b.ty +
                      (1 - 2 * (a.qx * a.qx + a.qy * a.qy)) * b.tz;
    out.tx = a.tx + rx;
    out.ty = a.ty + ry;
    out.tz = a.tz + rz;
    return out;
}
} // namespace map_pose_detail

// Compose the CURRENT pose of cfg.base_frame:
//
//     T_map_base = T_map_odom(correction) * T_odom_child(odometry) * T_child_base(lever arm)
//
// All three links are required and their frames must line up (correction map<-odom, odometry
// odom<-child, lever arm child<-base_link).  A missing link, a mismatched frame or a stale gate
// answer makes the pose UNAVAILABLE - never a silently wrong base_link pose.  The published stamp
// is the ODOMETRY'S OWN MEASUREMENT STAMP, and nothing is extrapolated.
inline CurrentPoseResult currentPose(const geometry_msgs::msg::TransformStamped &map_odom,
                                     const geometry_msgs::msg::TransformStamped &odom_child,
                                     const geometry_msgs::msg::TransformStamped &child_base,
                                     double now_s, const CurrentPoseConfig &cfg,
                                     bool gate_valid = false, double gate_answer_age_s = -1.0)
{
    using namespace map_pose_detail;
    CurrentPoseResult out;
    const double odom_s = stampSeconds(odom_child.header.stamp);
    const double corr_s = stampSeconds(map_odom.header.stamp);
    out.odom_age_s = now_s - odom_s;
    out.correction_age_s = now_s - corr_s;
    out.gate_valid = gate_valid;
    out.gate_answer_age_s = gate_answer_age_s;
    out.correction_valid = out.correction_age_s >= 0.0 &&
                           out.correction_age_s <= cfg.correction_validity_timeout_s;

    // frame chain must line up exactly; anything else is unavailable (fail closed)
    if (map_odom.header.frame_id != cfg.map_frame ||
        map_odom.child_frame_id != odom_child.header.frame_id ||
        odom_child.child_frame_id != child_base.header.frame_id ||
        child_base.child_frame_id != cfg.base_frame)
        return out;

    if (out.odom_age_s < 0.0 || out.odom_age_s > cfg.odom_age_limit_s)
        return out; // stale odometry: unavailable rather than extrapolated
    if (cfg.require_gate_valid &&
        !(gate_valid && gate_answer_age_s >= 0.0 &&
          gate_answer_age_s <= cfg.correction_validity_timeout_s))
        return out; // the gate has not vouched for the lock (or the answer is stale): unavailable

    const Tf full = compose(compose(fromMsg(map_odom.transform), fromMsg(odom_child.transform)),
                            fromMsg(child_base.transform));
    out.stamp = odom_child.header.stamp;
    out.pose.position.x = full.tx;
    out.pose.position.y = full.ty;
    out.pose.position.z = full.tz;
    out.pose.orientation.x = full.qx;
    out.pose.orientation.y = full.qy;
    out.pose.orientation.z = full.qz;
    out.pose.orientation.w = full.qw;
    out.available = true;
    return out;
}

// Carry the newest RECEIVED odometry forward from its own measurement stamp to the query instant
// and compose the base_link pose there:
//
//     v_world = R_latest * v_child
//     p_pred  = p_latest + v_world * dt
//     omega   = Log(R_previous^T * R_latest) / h
//     R_pred  = R_latest * Exp(omega * dt)
//     T_map_base = T_map_odom(latest received correction)
//                * predicted(odom<-child)
//                * T_child_base(lever arm at the query stamp)
//
// The odometry link is predicted BEFORE the lever arm is applied, so a rotating base gets its
// lever arm carried too (predicting the base_link pose directly would drop that term).
//
// Rejection is fail-closed and ordered: frames -> non-finite -> zero-norm quaternion ->
// future correction -> future odometry -> odometry order -> odometry interval -> dt window ->
// ambiguous rotation (>= pi - 1e-6) -> odometry age -> correction age -> gate.  The first failure
// names reject_reason; available stays false and no pose is produced.  Causality is preserved:
// previous/latest must precede the query, and no future transform is ever consulted.
inline CurrentPoseResult predictCurrentPose(const geometry_msgs::msg::TransformStamped &map_odom,
                                            const nav_msgs::msg::Odometry &previous_odom,
                                            const nav_msgs::msg::Odometry &latest_odom,
                                            const geometry_msgs::msg::TransformStamped &child_base,
                                            const builtin_interfaces::msg::Time &query_stamp,
                                            const CurrentPoseConfig &cfg,
                                            bool gate_valid = false,
                                            double gate_answer_age_s = -1.0)
{
    using namespace map_pose_detail;
    CurrentPoseResult out;
    out.stamp = query_stamp;
    out.predicted = false;
    out.gate_valid = gate_valid;
    out.gate_answer_age_s = gate_answer_age_s;

    // 1. every link of the chain must line up exactly, and both odometry samples must describe the
    //    same link (odom<-child); anything else is not the chain this function composes.
    const bool frames_ok =
        map_odom.header.frame_id == cfg.map_frame &&
        map_odom.child_frame_id == latest_odom.header.frame_id &&
        previous_odom.header.frame_id == latest_odom.header.frame_id &&
        previous_odom.child_frame_id == latest_odom.child_frame_id &&
        child_base.header.frame_id == latest_odom.child_frame_id &&
        child_base.child_frame_id == cfg.base_frame;
    if (!frames_ok)
    {
        out.reject_reason = map_pose_reason::kFrames;
        return out;
    }

    // 2. no non-finite value may enter the estimate (a NaN would otherwise propagate silently
    //    through the composition and be published as a pose).
    const double query_s = stampSeconds(query_stamp);
    const double t_prev = stampSeconds(previous_odom.header.stamp);
    const double t_latest = stampSeconds(latest_odom.header.stamp);
    const double corr_s = stampSeconds(map_odom.header.stamp);
    const auto &pp = latest_odom.pose.pose.position;
    const auto &po = latest_odom.pose.pose.orientation;
    const auto &pv = latest_odom.twist.twist.linear;
    const auto &qo = previous_odom.pose.pose.orientation;
    const Tf corr = fromMsg(map_odom.transform);
    const Tf lever = fromMsg(child_base.transform);
    const auto fin = [](double v) { return std::isfinite(v); };
    const bool finite = fin(query_s) && fin(t_prev) && fin(t_latest) && fin(corr_s) &&
                        finiteTf(corr) && finiteTf(lever) &&
                        fin(pp.x) && fin(pp.y) && fin(pp.z) &&
                        fin(po.x) && fin(po.y) && fin(po.z) && fin(po.w) &&
                        fin(pv.x) && fin(pv.y) && fin(pv.z) &&
                        fin(qo.x) && fin(qo.y) && fin(qo.z) && fin(qo.w);
    if (!finite)
    {
        out.reject_reason = map_pose_reason::kNonFinite;
        return out;
    }

    // 3. quaternions must be real rotations, not zero-norm placeholders
    Tf corr_n = corr;
    Tf lever_n = lever;
    const auto norm2 = [](double x, double y, double z, double w) {
        return x * x + y * y + z * z + w * w;
    };
    if (!normalize(corr_n) || !normalize(lever_n) ||
        !(norm2(po.x, po.y, po.z, po.w) > 0.0) ||
        !(norm2(qo.x, qo.y, qo.z, qo.w) > 0.0))
    {
        out.reject_reason = map_pose_reason::kZeroQuaternion;
        return out;
    }

    const double h = t_latest - t_prev;
    const double dt = query_s - t_latest;
    out.prediction_dt_s = dt;
    out.odom_age_s = dt;
    out.correction_age_s = query_s - corr_s;
    out.correction_valid = out.correction_age_s >= 0.0 &&
                           out.correction_age_s <= cfg.correction_validity_timeout_s;

    // 4. the correction must not come from the future of the query
    if (corr_s > query_s)
    {
        out.reject_reason = map_pose_reason::kFutureCorrection;
        return out;
    }
    // 5. the odometry must already have been measured at the query instant (dt >= 0)
    if (t_latest > query_s)
    {
        out.reject_reason = map_pose_reason::kFutureOdom;
        return out;
    }
    // 6. two strictly increasing samples are required for an angular rate
    if (!(h > 0.0))
    {
        out.reject_reason = map_pose_reason::kOdomOrder;
        return out;
    }
    // 7. ...estimated over the declared window
    if (h < cfg.min_odom_interval_s || h > cfg.max_odom_interval_s)
    {
        out.reject_reason = map_pose_reason::kOdomInterval;
        return out;
    }
    // 8. ...and carried forward no further than the pre-registered bound
    if (dt > cfg.max_prediction_s)
    {
        out.reject_reason = map_pose_reason::kDtOutOfRange;
        return out;
    }

    const Eigen::Quaterniond q_prev(qo.w, qo.x, qo.y, qo.z);
    const Eigen::Quaterniond q_latest(po.w, po.x, po.y, po.z);
    Eigen::Quaterniond q_rel = q_prev.conjugate() * q_latest;
    if (q_rel.w() < 0.0)
        q_rel.coeffs() *= -1.0; // shortest SO(3) branch
    const double rel_angle = 2.0 * std::atan2(q_rel.vec().norm(), q_rel.w());
    // 9. a rotation at/near pi has no determinate sign for the angular rate
    constexpr double kPi = 3.14159265358979323846;
    if (rel_angle >= kPi - 1e-6)
    {
        out.reject_reason = map_pose_reason::kAngleAmbiguous;
        return out;
    }
    // 10. the newest odometry must be fresh in its own right
    if (dt > cfg.odom_age_limit_s)
    {
        out.reject_reason = map_pose_reason::kOdomAge;
        return out;
    }
    // 11. ...and so must the accepted correction
    if (!out.correction_valid)
    {
        out.reject_reason = map_pose_reason::kCorrectionAge;
        return out;
    }
    // 12. ...and the gate answer (the same rule as the non-predicting current mode)
    if (cfg.require_gate_valid &&
        !(gate_valid && gate_answer_age_s >= 0.0 &&
          gate_answer_age_s <= cfg.correction_validity_timeout_s))
    {
        out.reject_reason = map_pose_reason::kGate;
        return out;
    }

    // constant angular rate over h, zero-rate limit -> identity increment
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    if (rel_angle > 0.0)
        omega = q_rel.vec().normalized() * (rel_angle / h);
    const double rot_angle = omega.norm() * dt;
    Eigen::Quaterniond q_pred = q_latest;
    if (rot_angle > 0.0)
    {
        Eigen::Quaterniond q_delta(Eigen::AngleAxisd(rot_angle, omega.normalized()));
        if (q_delta.w() < 0.0)
            q_delta.coeffs() *= -1.0; // shortest SO(3) branch
        q_pred = (q_latest * q_delta).normalized();
        if (q_pred.w() < 0.0)
            q_pred.coeffs() *= -1.0;
    }

    // constant-velocity translation: the body-frame linear twist rotated into the odom frame
    const Eigen::Vector3d p_latest(pp.x, pp.y, pp.z);
    const Eigen::Vector3d v_child(pv.x, pv.y, pv.z);
    const Eigen::Vector3d p_pred = p_latest + (q_latest * v_child) * dt;

    Tf odom_child_pred;
    odom_child_pred.tx = p_pred.x();
    odom_child_pred.ty = p_pred.y();
    odom_child_pred.tz = p_pred.z();
    odom_child_pred.qx = q_pred.x();
    odom_child_pred.qy = q_pred.y();
    odom_child_pred.qz = q_pred.z();
    odom_child_pred.qw = q_pred.w();

    const Tf full = compose(compose(corr_n, odom_child_pred), lever_n);
    out.pose.position.x = full.tx;
    out.pose.position.y = full.ty;
    out.pose.position.z = full.tz;
    out.pose.orientation.x = full.qx;
    out.pose.orientation.y = full.qy;
    out.pose.orientation.z = full.qz;
    out.pose.orientation.w = full.qw;
    out.available = true;
    out.predicted = true;
    return out;
}
