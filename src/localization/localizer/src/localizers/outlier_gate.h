#pragma once
// ---------------------------------------------------------------------------
// Physical-consistency gate + bounded recovery for the ICP updates of localizer_node.
//
// A relocalization update is only accepted when it does not contradict the last accepted one
// by more than a few centimetres / milliradians.  The gate exists because a single wrong ICP
// lock would otherwise teleport the published map<-odom transform.
//
// Three properties are deliberate, and each was wrong before (integration review B2):
//
//  1. The innovation is measured on the BODY POSE, i.e. on where the transform puts the
//     sensor, not on the offset translation T_map<-odom itself.  The offset lives at the odom
//     origin, so ICP rotation noise delta at an odom-frame distance d moves it by ~d*delta
//     (1 mrad at 40 m = 4 cm) and the gate would tighten with distance from the odom origin.
//     Both candidates are evaluated at the SAME body pose, so a pure rotation innovation no
//     longer masquerades as a translation innovation.
//  2. The reference is the last ACCEPTED RAW candidate, not the published EMA.  Comparing
//     against a smoothed value makes any steady drift (the normal case: the LIO odometry
//     drifts while the localizer corrects it) look like a growing innovation, so a run of
//     individually plausible updates ends up rejected - and since a rejection never updated
//     the reference, the gate became absorbing.
//  3. Rejection is bounded AND the gate stays active while the lock is degraded.  After
//     max_consecutive_rejects the gate reports `lost`, re-seeds its reference (and the
//     published transform) on the candidate, and reports the lock INVALID - but it keeps
//     gating against the new reference, and after recovery_accepts consecutive accepted
//     updates it reports `recovered` so the caller can say valid again.  The "not valid"
//     state must not be interpreted as "bypass the gate": doing that left the gate disabled
//     and the node invalid until an operator relocalized it (review NB1).  The class owns the
//     whole lifecycle (engaged / valid / awaiting recovery) because all of it is decided by
//     the same accept-reject sequence; the node only maps it onto relocalize_check.
//
// Time-discrete by construction: dt is the time between ICP updates, and the thresholds are
// per-update innovation bounds (they do not scale with dt because the odometry is the
// physical prior for the body pose, not a constant-velocity model).
// ---------------------------------------------------------------------------
#include <Eigen/Eigen>
#include <Eigen/Geometry>

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;

struct OffsetGateConfig
{
    double max_translation_m = 0.04;   // body-position innovation allowed per update
    double max_angle_rad = 0.03;       // body-orientation innovation allowed per update
    int max_consecutive_rejects = 10;  // then: report lost and re-seed the reference
    // After a re-seed the gate stays ENGAGED (it keeps rejecting implausible candidates) and
    // needs this many consecutive accepted updates before it reports the lock as recovered.
    // The caller maps lost -> invalid and recovered -> valid again; without the recovery edge
    // the node would stay invalid forever while the gate is bypassed (review NB1).
    int recovery_accepts = 10;
    double ema_alpha = 0.10;           // smoothing of the PUBLISHED offset only
};

struct OffsetGateOutcome
{
    bool accepted = false;
    bool locked = false;       // adopted unconditionally: first lock or an operator relocalize
    bool lost = false;         // bounded recovery triggered: caller must report invalid
    bool recovered = false;    // lock trustworthy again after a lost event: report valid
    double innovation_m = 0.0; // body-position innovation of this candidate [m]
    double innovation_rad = 0.0;
    int consecutive_rejects = 0;
};

class OffsetOutlierGate
{
public:
    OffsetOutlierGate() = default;
    explicit OffsetOutlierGate(const OffsetGateConfig &cfg) : m_cfg(cfg) {}

    // Adopt the candidate unconditionally as the reference (first lock, or an operator
    // relocalize).  Does NOT make the lock valid by itself: only a service_request update or a
    // completed recovery does.
    void reset(const M3D &r, const V3D &t)
    {
        m_engaged = true;
        m_raw_r = r;
        m_raw_t = t;
        m_ema_r = r;
        m_ema_t = t;
        m_awaiting_recovery = false;
        m_accepts = 0;
        m_rejects = 0;
    }

    // Operator relocalize / initial-pose request: the next adopted candidate re-locks and the
    // lock is reported INVALID until it does (relocalize_check must not keep saying valid while
    // the operator is re-seeding it).
    void requestRelock()
    {
        m_valid = false;
        m_awaiting_recovery = false;
        m_accepts = 0;
    }

    bool engaged() const { return m_engaged; }
    bool valid() const { return m_valid; }
    bool awaitingRecovery() const { return m_awaiting_recovery; }
    const M3D &published_r() const { return m_ema_r; }
    const V3D &published_t() const { return m_ema_t; }
    int consecutiveRejects() const { return m_rejects; }
    const OffsetGateConfig &config() const { return m_cfg; }

    // One ICP update.
    // cand_*: the map<-odom offset the ICP just produced.
    // body_t: odom-frame position of the sensor (from the LIO odometry) that the candidate is
    //         evaluated at.  The body ORIENTATION is not needed: the angle is measured between
    //         the two offsets directly, which is the same angle their body poses differ by
    //         (angularDistance(cand_r*body_r, raw_r*body_r) == angularDistance(cand_r, raw_r)).
    // service_request: a relocalize/convergence request is pending (operator forced a new lock).
    // The reported lifecycle is: !engaged -> first candidate locks (not valid until a service
    // request); service_request -> unconditional adopt + valid; accepted -> gated, keeps valid;
    // rejected x max_consecutive_rejects -> lost (NOT valid, but still gating on the re-seeded
    // reference); then recovery_accepts consecutive accepted updates -> recovered (valid).
    OffsetGateOutcome update(const M3D &cand_r, const V3D &cand_t, const V3D &body_t,
                             bool service_request = false)
    {
        OffsetGateOutcome out;
        if (!m_engaged || service_request)
        {
            reset(cand_r, cand_t);
            out.accepted = true;
            out.locked = true;
            if (service_request)
                m_valid = true;
            return out;
        }
        const V3D cand_body = cand_r * body_t + cand_t;
        const V3D prev_body = m_raw_r * body_t + m_raw_t;
        out.innovation_m = (cand_body - prev_body).norm();
        // Right-invariant angle: angularDistance(cand_r*body_r, raw_r*body_r) == this.
        out.innovation_rad = Eigen::Quaterniond(m_raw_r).angularDistance(Eigen::Quaterniond(cand_r));

        if (out.innovation_m <= m_cfg.max_translation_m && out.innovation_rad <= m_cfg.max_angle_rad)
        {
            m_raw_r = cand_r;
            m_raw_t = cand_t;
            const double a = m_cfg.ema_alpha;
            m_ema_t = (1.0 - a) * m_ema_t + a * cand_t;
            m_ema_r = Eigen::Quaterniond(m_ema_r)
                          .slerp(a, Eigen::Quaterniond(cand_r))
                          .toRotationMatrix();
            m_rejects = 0;
            out.accepted = true;
            if (m_awaiting_recovery && ++m_accepts >= m_cfg.recovery_accepts)
            {
                m_awaiting_recovery = false;
                m_valid = true;
                out.recovered = true;
            }
        }
        else
        {
            m_accepts = 0;
            if (++m_rejects >= m_cfg.max_consecutive_rejects)
            {
                // Bounded recovery: re-seed on the candidate so tracking can resume, report the
                // lock as INVALID, and KEEP GATING - the reference is the freshly seeded
                // candidate, so the very next update is checked against it again (the node must
                // therefore not bypass the gate while it is not valid).
                reset(cand_r, cand_t);
                m_awaiting_recovery = true;
                m_valid = false;
                out.lost = true;
            }
        }
        out.consecutive_rejects = m_rejects;
        return out;
    }

private:
    OffsetGateConfig m_cfg;
    bool m_engaged = false;
    bool m_valid = false;
    bool m_awaiting_recovery = false;
    int m_accepts = 0;
    M3D m_raw_r = M3D::Identity();
    V3D m_raw_t = V3D::Zero();
    M3D m_ema_r = M3D::Identity();
    V3D m_ema_t = V3D::Zero();
    int m_rejects = 0;
};
