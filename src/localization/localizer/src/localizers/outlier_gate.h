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
//     max_consecutive_rejects the gate reports `lost`, re-seeds its tracking reference on the
//     candidate, and reports the lock INVALID - but it keeps
//     gating against the new reference, and after recovery_accepts consecutive accepted
//     updates it reports `recovered` so the caller can say valid again.  The "not valid"
//     state must not be interpreted as "bypass the gate": doing that left the gate disabled
//     and the node invalid until an operator relocalized it (review NB1).  The class owns the
//     whole lifecycle (engaged / valid / awaiting recovery) because all of it is decided by
//     the same accept-reject sequence; the node only maps it onto relocalize_check.
//  4. A candidate this gate REJECTED is never published.  The reference used for tracking can
//     be re-seeded on such a candidate (otherwise the tracker could never recover), but the
//     published map<-odom output is an unframed transform - consumers may ignore
//     relocalize_check - so it stays at the last trusted value while the lock is degraded and
//     is released (snapped, not blended) only when the lock is valid again.  Callers that must
//     follow the tracker (e.g. to seed the next ICP) use reference_*(), not published_*().
//  5. The FIRST unattended lock owes the same corroboration as a post-loss re-seed.  It used to
//     adopt AND publish the first ICP result outright, which made one unconfirmed measurement
//     both the published map<-odom transform and - with the node renewing its freshness bound on
//     it - a "valid" answer from relocalize_check.  The first candidate now only seeds the
//     tracking reference and puts the lock in the same await-recovery state, so it is published
//     and reported valid by the same recovery_accepts consecutive accepted updates.  An operator
//     relocalize is the one exception: it is corroboration by itself and locks immediately.
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
    // needs this many consecutive accepted updates before it reports the lock as recovered and
    // releases the published transform again.  The caller maps lost -> invalid and
    // recovered -> valid; without the recovery edge the node would stay invalid forever while
    // the gate is bypassed (review NB1).
    int recovery_accepts = 10;
    double ema_alpha = 0.10;           // smoothing of the PUBLISHED offset only
};

struct OffsetGateOutcome
{
    bool accepted = false;
    bool locked = false;       // adopted unconditionally: first lock or an operator relocalize
    bool lost = false;         // bounded recovery triggered: caller must report invalid
    bool recovered = false;    // lock trustworthy again after a lost event: report valid
    bool valid = false;        // the lock state after this update (see corroborated())
    double innovation_m = 0.0; // body-position innovation of this candidate [m]
    double innovation_rad = 0.0;
    int consecutive_rejects = 0;

    // Does this update CORROBORATE the lock, i.e. may the caller renew the freshness bound of
    // its lock-validity answer with it?  True only when the gate accepted the candidate while
    // the lock is valid: an accepted candidate under a valid lock, an operator relock, or a
    // completed (first or post-loss) qualification.  A failed ICP attempt and a rejected or
    // not-yet-qualified update corroborate nothing, so they must not renew the bound - a stream
    // of them would otherwise keep a dead lock answering "valid" forever, which is the defect
    // the bound exists to prevent.
    bool corroborated() const { return accepted && valid; }
};

class OffsetOutlierGate
{
public:
    OffsetOutlierGate() = default;
    explicit OffsetOutlierGate(const OffsetGateConfig &cfg) : m_cfg(cfg) {}

    // Adopt the candidate unconditionally as reference AND published transform.  This is the
    // operator path (a relocalize / initial-pose request) and the test entry point; an
    // UNATTENDED first lock does NOT go through here (see update(): it must not publish, and it
    // must earn its validity).  Does NOT make the lock valid by itself: update() does that for a
    // service_request, and the completed recovery edge does it for everything else.
    void reset(const M3D &r, const V3D &t)
    {
        m_engaged = true;
        m_raw_r = r;
        m_raw_t = t;
        setPublished(r, t, true);
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
    // The tracking reference: the last ACCEPTED ICP offset (or the re-seeded one while the lock
    // is degraded).  This is what the next ICP should be seeded with - it is deliberately NOT
    // the published (EMA-smoothed, possibly frozen) value.
    const M3D &reference_r() const { return m_raw_r; }
    const V3D &reference_t() const { return m_raw_t; }
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
    // The reported lifecycle is: !engaged -> the first candidate seeds the tracking reference and
    // enters the recovery state (NOT valid, published transform NOT released) until
    // recovery_accepts consecutive accepted updates have corroborated it; service_request ->
    // unconditional adopt + published + valid immediately (the operator is the corroboration);
    // accepted -> gated, keeps valid; rejected x max_consecutive_rejects -> lost (NOT valid, but
    // still gating on the re-seeded reference); then recovery_accepts consecutive accepted
    // updates -> recovered (valid, published transform released).
    OffsetGateOutcome update(const M3D &cand_r, const V3D &cand_t, const V3D &body_t,
                             bool service_request = false)
    {
        OffsetGateOutcome out;
        if (!m_engaged || service_request)
        {
            if (service_request)
            {
                // Operator relocalize: the operator vouched for the pose, so adopting it and
                // publishing it is corroborated by construction - no streak is owed.
                reset(cand_r, cand_t);
                m_valid = true;
            }
            else
            {
                // FIRST unattended lock.  The candidate becomes the tracking reference, but it is
                // NOT published and NOT valid: one ICP result is exactly the evidence that used
                // to teleport the published transform, and relocalize_check must not answer valid
                // on a single unconfirmed measurement.  The lock enters the same await-recovery
                // state a lost lock re-seeds into, so the SAME rule qualifies it: recovery_accepts
                // consecutive accepted updates against this reference.
                m_engaged = true;
                m_raw_r = cand_r;
                m_raw_t = cand_t;
                m_awaiting_recovery = true;
                m_valid = false;
                m_accepts = 0;
                m_rejects = 0;
            }
            out.accepted = true;
            out.locked = true;
            out.valid = m_valid;
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
            m_rejects = 0;
            out.accepted = true;
            if (m_awaiting_recovery)
            {
                // Degraded: the tracking reference follows the accepted candidates, but the
                // PUBLISHED transform stays at the last trusted value until recovery.  A
                // consumer that ignores relocalize_check must not be handed a pose the gate has
                // not corroborated the required number of times.
                if (++m_accepts >= m_cfg.recovery_accepts)
                {
                    m_awaiting_recovery = false;
                    m_valid = true;
                    out.recovered = true;
                    // Recovered: release the frozen output.  Snapped, not EMA-blended, because
                    // blending from the deliberately frozen value would sweep the published
                    // transform through intermediate poses nobody vouched for.
                    setPublished(cand_r, cand_t, true);
                }
            }
            else
            {
                setPublished(cand_r, cand_t);
            }
        }
        else
        {
            m_accepts = 0;
            if (++m_rejects >= m_cfg.max_consecutive_rejects)
            {
                // Bounded recovery: re-seed the REFERENCE on the candidate so tracking can
                // resume, report the lock as INVALID, and KEEP GATING - the very next update is
                // checked against the new reference again (the node must therefore not bypass
                // the gate while it is not valid).  The published transform is NOT touched: a
                // rejected candidate must never become the published map<-odom, whatever the
                // reason for re-seeding, because the output carries no validity flag.
                m_raw_r = cand_r;
                m_raw_t = cand_t;
                m_rejects = 0;
                m_awaiting_recovery = true;
                m_valid = false;
                out.lost = true;
            }
        }
        out.consecutive_rejects = m_rejects;
        out.valid = m_valid;
        return out;
    }

private:
    // EMA-smooth the PUBLISHED transform (an accepted update in normal operation).
    void setPublished(const M3D &r, const V3D &t, bool snap = false)
    {
        if (snap)
        {
            m_ema_r = r;
            m_ema_t = t;
            return;
        }
        const double a = m_cfg.ema_alpha;
        m_ema_t = (1.0 - a) * m_ema_t + a * t;
        m_ema_r = Eigen::Quaterniond(m_ema_r)
                      .slerp(a, Eigen::Quaterniond(r))
                      .toRotationMatrix();
    }

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
