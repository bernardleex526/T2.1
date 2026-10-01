#pragma once
// ---------------------------------------------------------------------------
// Bounded per-frame pairing of a point-cloud stream with an odometry stream.
//
// Why this exists (measured, 2026-09-30): the localizer used
// message_filters::ApproximateTime(10) for this job, and its intake was measured to stop
// delivering pairs PERMANENTLY while both topics kept arriving at 10 Hz - the process-external
// monitor saw 540 clouds and 546 odometry messages with no gap > 1 s, while the synchronizer
// delivered 254 pairs in the first 33 s and then none at all for the rest of the run, which
// silently froze the map->odom output for 42.8 s in one recorded T3 run.  Its internal deques
// can end up non-empty with no candidate, after which add() never calls process() again and the
// overflow branch only drops messages (see message_filters/sync_policies/approximate_time.h).
//
// This cache cannot enter that state: every message is stored keyed by its own stamp and every
// read is answered from the stores alone.  Nothing is queued across calls, so a slow consumer
// costs missed frames - never a wedged intake - and the answer is always the newest complete
// pair, i.e. it degrades to "the freshest usable frame" instead of to "nothing at all".
//
// The two streams are stamped with the SAME scan-end time by lio_node, so the default door is
// an exact stamp match; a tolerance is allowed because a real driver may stamp the pair a few
// milliseconds apart.  Selection is by nearest stamp inside the tolerance, and only stamps
// that are genuinely present in both stores can be returned.
#include <cmath>
#include <cstddef>
#include <iterator>
#include <map>

template <typename CloudT, typename PoseT>
class FramePairCache
{
public:
    struct Pair
    {
        double stamp = 0.0;
        CloudT cloud{};
        PoseT pose{};
    };

    explicit FramePairCache(std::size_t depth = 40) : m_depth(depth ? depth : 1) {}

    void addCloud(double stamp, const CloudT &cloud)
    {
        m_clouds[stamp] = cloud;
        prune(m_clouds);
    }

    void addOdom(double stamp, const PoseT &pose)
    {
        m_odom[stamp] = pose;
        prune(m_odom);
    }

    // Newest cloud stamp that also has an odometry sample within `tolerance_s`; the odometry
    // sample returned is the one nearest that stamp.  Returns false when no complete pair is
    // stored, which is the only "unavailable" answer and is never sticky.
    bool newest(double tolerance_s, Pair &out) const
    {
        for (auto it = m_clouds.rbegin(); it != m_clouds.rend(); ++it)
        {
            const auto lo = m_odom.lower_bound(it->first - tolerance_s);
            const auto hi = m_odom.upper_bound(it->first + tolerance_s);
            if (lo == hi)
                continue;
            auto best = lo;
            for (auto o = std::next(lo); o != hi; ++o)
                if (std::abs(o->first - it->first) <= std::abs(best->first - it->first))
                    best = o;
            out.stamp = it->first;
            out.cloud = it->second;
            out.pose = best->second;
            return true;
        }
        return false;
    }

    std::size_t cloudCount() const { return m_clouds.size(); }
    std::size_t odomCount() const { return m_odom.size(); }
    std::size_t depth() const { return m_depth; }

private:
    template <typename M>
    void prune(M &store)
    {
        while (store.size() > m_depth)
            store.erase(store.begin());
    }

    std::size_t m_depth;
    std::map<double, CloudT> m_clouds;
    std::map<double, PoseT> m_odom;
};
