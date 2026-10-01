// Regression tests for the localizer's lock-validity answer (relocalize_check).
//
// Pinned here is the defect measured on 2026-09-30: the answer came from the gate state alone,
// and the gate only changes when an ICP update arrives, so a node whose intake had stopped
// kept answering valid=true.  In one recorded T3 run the map->odom stream stopped at 17.1 s
// while relocalize_check still answered valid at 20.7 s .. 53.7 s (valid fraction 0.80-0.87),
// which made a dead localizer indistinguishable from a live one and poisoned the availability
// evidence built on top of it.
//
// The bound is renewed ONLY by a corroborated correction - the node's call site passes an
// accepted candidate under a valid lock, an explicit operator relock, or a completed
// first/post-loss qualification (see OffsetGateOutcome::corroborated() and the coupled
// gate+validity lifecycle tests in test_localizer_gate.cpp).  A failed ICP attempt renews
// nothing: a node whose ICP keeps failing expires into "invalid" exactly like a node whose
// intake stopped.
#include <gtest/gtest.h>

#include "localizers/lock_validity.h"

namespace
{
constexpr double kTimeout = 1.0;
} // namespace

// Before the first corroborated correction there is nothing to be valid about.
TEST(LockValidity, InvalidBeforeFirstUpdate)
{
    LockValidity v;
    EXPECT_FALSE(v.hasUpdate());
    EXPECT_FALSE(v.valid(true, 100.0, kTimeout));
}

// A corroborated correction makes the answer true (when the gate agrees to the same instant).
TEST(LockValidity, ValidRightAfterAnUpdate)
{
    LockValidity v;
    v.onUpdate(100.0);
    EXPECT_TRUE(v.valid(true, 100.0, kTimeout));
    EXPECT_TRUE(v.valid(true, 100.5, kTimeout));
    EXPECT_TRUE(v.valid(true, 101.0, kTimeout));
}

// The regression: a stalled intake must stop answering valid, and nothing but a corroborated
// correction may renew the bound - so however many ICP attempts fail in the meantime, the answer
// stays invalid (the failure path used to call onUpdate and keep a dead lock alive).
TEST(LockValidity, GoesInvalidWhenUpdatesStop)
{
    LockValidity v;
    v.onUpdate(100.0);
    EXPECT_TRUE(v.valid(true, 100.9, kTimeout));
    EXPECT_FALSE(v.valid(true, 101.2, kTimeout)) << "stale lock answered valid";
    EXPECT_FALSE(v.valid(true, 140.0, kTimeout)) << "hours-old lock answered valid";
    EXPECT_FALSE(v.valid(true, 3600.0, kTimeout)) << "no renewal may come from the attempt count";

    // A corroborated correction renews it again, and it then ages out on its own.
    v.onUpdate(140.0);
    EXPECT_TRUE(v.valid(true, 140.5, kTimeout));
    EXPECT_FALSE(v.valid(true, 142.0, kTimeout));
}

// The gate's own verdict is never overridden by freshness.
TEST(LockValidity, GateVerdictIsRespected)
{
    LockValidity v;
    v.onUpdate(100.0);
    EXPECT_FALSE(v.valid(false, 100.1, kTimeout));
}

// The last-update age is exposed so the node can report it instead of guessing.
TEST(LockValidity, ReportsLastUpdateAge)
{
    LockValidity v;
    EXPECT_LT(v.lastUpdateS(), 0.0);
    v.onUpdate(12.5);
    EXPECT_DOUBLE_EQ(v.lastUpdateS(), 12.5);
    v.onUpdate(13.5);
    EXPECT_DOUBLE_EQ(v.lastUpdateS(), 13.5);
}
