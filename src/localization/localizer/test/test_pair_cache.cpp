// Regression tests for the localizer's frame-pairing intake.
//
// Pinned here is the defect measured on 2026-09-30: message_filters::ApproximateTime(10)
// stopped delivering (cloud, odom) pairs PERMANENTLY while both topics kept arriving at 10 Hz
// (an external monitor saw 540 clouds and 546 odometry messages with no gap > 1 s, while the
// synchronizer delivered 254 pairs and then nothing for the rest of the run).  The map->odom
// output silently froze for 42.8 s and every downstream availability number inherited it.
//
// The behaviour these tests require of the replacement is:
//   * a read always answers with the NEWEST complete pair (freshest usable frame), never with
//     a stale one, and never with "nothing" while both streams are being fed;
//   * a slow consumer (the ICP takes ~0.2 s while frames arrive every 0.1 s) only costs
//     missed frames - the intake must recover on the very next read, with no state that can
//     wedge it;
//   * arrivals that are still incomplete are not paired, and no pair is invented from one
//     stream alone;
//   * memory is bounded regardless of how long the reader stays away.
#include <gtest/gtest.h>

#include "localizers/pair_cache.h"

#include <string>
#include <vector>

namespace
{
using Cache = FramePairCache<int, std::string>;

// One complete scan/odometry frame: lio_node stamps both with the same scan-end time.
struct Frame
{
    double t;
    int cloud;
    std::string pose;
};

std::vector<Frame> frames(int n, double dt = 0.1, double t0 = 1000.0)
{
    std::vector<Frame> out;
    for (int i = 0; i < n; ++i)
        out.push_back({t0 + i * dt, i, "p" + std::to_string(i)});
    return out;
}
} // namespace

// A reader that only looks every Nth frame must still see the newest complete pair each time.
TEST(FramePairCache, SlowConsumerAlwaysGetsTheNewestPair)
{
    Cache cache(40);
    const auto fs = frames(600); // 60 s at 10 Hz
    std::vector<int> seen;
    for (size_t i = 0; i < fs.size(); ++i)
    {
        cache.addCloud(fs[i].t, fs[i].cloud);
        cache.addOdom(fs[i].t, fs[i].pose);
        if (i % 3 == 2) // the consumer is ~3x slower than the stream
        {
            Cache::Pair p;
            ASSERT_TRUE(cache.newest(0.05, p)) << "intake wedged at frame " << i;
            seen.push_back(p.cloud);
        }
    }
    ASSERT_FALSE(seen.empty());
    // every read answered with the most RECENT frame handed over at that point
    for (size_t r = 0; r < seen.size(); ++r)
    {
        const int expected = static_cast<int>(3 * (r + 1)) - 1;
        EXPECT_EQ(seen[r], expected) << "read " << r << " was stale";
    }
    // and the stream kept being answered at the full reader rate: no permanent stop
    EXPECT_EQ(seen.size(), fs.size() / 3);
}

// The exact historical failure shape: bursts of arrivals with the reader away for a long
// stretch, then reads again.  The intake must recover immediately afterwards.
TEST(FramePairCache, BurstWithAbsentReaderRecoversImmediately)
{
    Cache cache(40);
    const auto fs = frames(400);
    for (size_t i = 0; i < fs.size(); ++i)
    {
        cache.addCloud(fs[i].t, fs[i].cloud);
        cache.addOdom(fs[i].t, fs[i].pose);
        if (i == 50)
        {
            // the reader disappears for 200 frames (20 s of arrivals)
            continue;
        }
    }
    Cache::Pair p;
    ASSERT_TRUE(cache.newest(0.05, p));
    EXPECT_EQ(p.cloud, 399) << "after the outage the reader must get the freshest frame";
}

// Interleaving must not lose a frame: a cloud that arrives before its odometry is paired as
// soon as the odometry lands, and the answer is still the newest complete pair.
TEST(FramePairCache, CloudBeforeOdomIsPairedOnceOdomArrives)
{
    Cache cache(8);
    cache.addCloud(10.0, 7);
    Cache::Pair p;
    EXPECT_FALSE(cache.newest(0.05, p)) << "an incomplete frame must not be paired";
    cache.addOdom(10.0, "p7");
    ASSERT_TRUE(cache.newest(0.05, p));
    EXPECT_EQ(p.cloud, 7);
    EXPECT_EQ(p.pose, "p7");
}

// Tolerance: stamps that differ by less than the tolerance still form a pair (a real driver may
// stamp the two messages a few milliseconds apart); further apart they do not.
TEST(FramePairCache, TolerancePairsNearbyStampsOnly)
{
    Cache cache(8);
    cache.addCloud(10.020, 1);
    cache.addOdom(10.000, "p0");
    Cache::Pair p;
    EXPECT_TRUE(cache.newest(0.05, p));
    EXPECT_EQ(p.cloud, 1);
    EXPECT_EQ(p.pose, "p0");

    Cache far(8);
    far.addCloud(10.500, 1);
    far.addOdom(10.000, "p0");
    EXPECT_FALSE(far.newest(0.05, p));
}

// The new answer is the nearest odometry sample to the cloud stamp, not merely the first inside
// the tolerance window.
TEST(FramePairCache, PicksNearestOdomStamp)
{
    Cache cache(8);
    cache.addCloud(10.000, 1);
    cache.addOdom(9.980, "early");
    cache.addOdom(10.040, "late");
    Cache::Pair p;
    ASSERT_TRUE(cache.newest(0.06, p));
    EXPECT_EQ(p.pose, "early");
    cache.addOdom(10.010, "closest");
    ASSERT_TRUE(cache.newest(0.06, p));
    EXPECT_EQ(p.pose, "closest");
}

// Memory stays bounded no matter how long nobody reads.
TEST(FramePairCache, MemoryIsBounded)
{
    Cache cache(16);
    for (const auto &f : frames(1000))
    {
        cache.addCloud(f.t, f.cloud);
        cache.addOdom(f.t, f.pose);
    }
    EXPECT_LE(cache.cloudCount(), cache.depth());
    EXPECT_LE(cache.odomCount(), cache.depth());
    Cache::Pair p;
    ASSERT_TRUE(cache.newest(0.05, p));
    EXPECT_EQ(p.cloud, 999);
}

// One stream alone never produces a pair, and an empty cache is simply "unavailable".
TEST(FramePairCache, SingleStreamNeverPairs)
{
    Cache cache(8);
    Cache::Pair p;
    EXPECT_FALSE(cache.newest(0.05, p));
    for (const auto &f : frames(20))
        cache.addCloud(f.t, f.cloud);
    EXPECT_FALSE(cache.newest(0.05, p));
    Cache other(8);
    for (const auto &f : frames(20))
        other.addOdom(f.t, f.pose);
    EXPECT_FALSE(other.newest(0.05, p));
}
