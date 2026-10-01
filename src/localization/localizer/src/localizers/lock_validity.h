#pragma once
// ---------------------------------------------------------------------------
// Freshness bound for the localizer's own "is the lock valid?" answer.
//
// Why this exists (measured, 2026-09-30): relocalize_check answered from the gate's state, and
// the gate only changes when an ICP update arrives.  A recorded T3 run therefore answered
// valid=true at 20.7 s .. 53.7 s wall while its map->odom stream had already stopped at 17.1 s
// - the consumer could not tell a live localizer from a dead one, and the harness's own
// availability evidence inherited that lie.  A lock is now only valid while the last
// CORROBORATED correction is younger than the declared timeout, so a stalled intake reports
// invalid instead of "valid, as of an hour ago".  Only a corroborated correction renews that
// bound (see onUpdate): a failed ICP attempt used to count as "the node is still working the
// stream" and renew it, which let a localizer whose ICP had stopped succeeding answer valid on
// the strength of its own failures.
#include <atomic>

class LockValidity
{
public:
    // Called when a CORRECTION is corroborated: an ICP candidate the gate accepted while the
    // lock was valid, an explicit operator relock, or the completed (first or post-loss)
    // qualification.  NOT called for a failed ICP attempt, a rejected candidate, or a
    // not-yet-qualified first lock: an attempt is not evidence about the lock, so a stream of
    // them must expire into "invalid" instead of renewing the bound (the node used to renew on
    // the failure path, which kept a dead localizer answering valid forever).
    void onUpdate(double now_s) { m_last_update_s.store(now_s); }

    double lastUpdateS() const { return m_last_update_s.load(); }
    bool hasUpdate() const { return m_last_update_s.load() >= 0.0; }

    // A lock answer is valid only if the gate says so AND an update happened within timeout_s.
    bool valid(bool gate_valid, double now_s, double timeout_s) const
    {
        const double last = m_last_update_s.load();
        if (last < 0.0)
            return false;
        return gate_valid && (now_s - last) <= timeout_s;
    }

private:
    std::atomic<double> m_last_update_s{-1.0};
};
