#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lib/statistics.h"

namespace {

GraphEdgeStatistics MakeFresh() {
    GraphEdgeStatistics s;
    s.Clear();
    return s;
}

}  // namespace

using namespace lib;

// ---------------------------------------------------------------------------
// GraphEdgeStatistics::Record / AverageSpeed -- basics
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, EmptyAverageIsZero) {
    auto s = MakeFresh();
    EXPECT_FALSE(s.Has(0, 1));
    EXPECT_DOUBLE_EQ(s.AverageSpeed(0, 1, 0.0), 0.0);
}

TEST(GraphEdgeStatistics, SinglePassageGivesLengthOverDuration) {
    auto s = MakeFresh();
    s.Record(0, 1, /*t_enter=*/0.0, /*t_exit=*/2.0, /*length=*/10.0,
             /*t_now=*/2.0);
    ASSERT_TRUE(s.Has(0, 1));
    EXPECT_DOUBLE_EQ(s.AverageSpeed(0, 1, 2.0), 5.0);
}

TEST(GraphEdgeStatistics, UndirectedKeying) {
    auto s = MakeFresh();
    s.Record(0, 1, 0.0, 1.0, 4.0, 1.0);  // speed = 4
    s.Record(1, 0, 1.0, 3.0, 4.0, 3.0);  // speed = 2 (same edge!)
    // Both passages should populate the same window.
    EXPECT_TRUE(s.Has(0, 1));
    EXPECT_TRUE(s.Has(1, 0));
    // Harmonic mean of {4, 2} = 2 / (1/4 + 1/2) = 2 / 0.75 = 8/3.
    EXPECT_NEAR(s.AverageSpeed(0, 1, 3.0), 8.0 / 3.0, 1e-12);
    EXPECT_NEAR(s.AverageSpeed(1, 0, 3.0), 8.0 / 3.0, 1e-12);
}

TEST(GraphEdgeStatistics, IgnoresNonPositiveDurationOrLength) {
    auto s = MakeFresh();
    s.Record(0, 1, 0.0, 0.0, 1.0, 0.0);     // duration 0
    s.Record(0, 1, 1.0, 0.5, 1.0, 1.0);     // negative duration
    s.Record(0, 1, 0.0, 1.0, 0.0, 1.0);     // length 0
    s.Record(0, 1, 0.0, 1.0, -1.0, 1.0);    // negative length
    EXPECT_FALSE(s.Has(0, 1));
}

// ---------------------------------------------------------------------------
// Harmonic vs arithmetic mean: ETA-correct aggregation
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, HarmonicMeanReflectsTotalTravelTime) {
    // Two passages of length 100 over the same edge: one fast (speed 50, dt=2),
    // one slow (speed 10, dt=10). Total time = 12, total length = 200, so the
    // effective speed averaged over the two passages is 200/12 = 16.666...
    //
    // The naive arithmetic mean would report 30, which would mislead any ETA
    // consumer about how long it takes to traverse this edge "on average".
    auto s = MakeFresh();
    s.Record(0, 1, 0.0,  2.0, 100.0,  2.0);   // speed 50
    s.Record(0, 1, 5.0, 15.0, 100.0, 15.0);   // speed 10
    EXPECT_NEAR(s.AverageSpeed(0, 1, 15.0), 200.0 / 12.0, 1e-9);
}

// ---------------------------------------------------------------------------
// EWMA time-decay weights
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, FiniteTauWeightsRecentMore) {
    auto s = MakeFresh();
    s.tau = 1.0;
    // Old slow passage and new fast passage, equal speeds magnitudes 1 and 10.
    s.Record(0, 1, 0.0, 1.0, 1.0,  1.0);     // speed 1, t_exit=1
    s.Record(0, 1, 9.0, 10.0, 10.0, 10.0);   // speed 10, t_exit=10
    // At t = 10, ages are 9 (old) and 0 (new). With tau=1 the old weight
    // is e^{-9} ~= 1.23e-4, so the harmonic mean should be ~= 10.
    // The actual value is approximately 9.989 due to the small contribution
    // from the old passage.
    EXPECT_NEAR(s.AverageSpeed(0, 1, 10.0), 10.0, 1e-1);

    // Sanity check: with infinite tau (default) both contribute equally and
    // the harmonic mean is 2 / (1/1 + 1/10) = 20/11 ~= 1.818.
    s.tau = std::numeric_limits<double>::infinity();
    EXPECT_NEAR(s.AverageSpeed(0, 1, 10.0), 20.0 / 11.0, 1e-9);
}

// Regression test for the stale-high-eviction artefact.
//
// Scenario: an edge has accumulated many fast passages from long ago. Traffic
// then settles into a steady slow regime. Under the legacy size-bounded FIFO
// + arithmetic mean, each new slow passage both adds a low number *and*
// evicts a stale high one, producing a misleading "the edge is getting
// worse" signal even though the physical state is steady. With EWMA decay
// the old samples have negligible weight, so a single fresh slow passage
// should already dominate the estimate.
TEST(GraphEdgeStatistics, EwmaSuppressesStaleHighSamples) {
    auto s = MakeFresh();
    s.tau = 1.0;
    // 20 fast passages a long time ago (centered around t=0).
    for (int i = 0; i < 20; ++i) {
        const double t_exit = -100.0 + static_cast<double>(i) * 0.1;
        s.Record(0, 1, t_exit - 0.1, t_exit, 5.0, t_exit);  // speed 50
    }
    // Now traffic slows down: one fresh slow passage exiting at t=0.
    s.Record(0, 1, -1.0, 0.0, 1.0, 0.0);  // speed 1

    // At t=0, all stale samples are at age ~100 (weight ~e^{-100} = 0),
    // and the fresh slow one has weight 1. The estimate should be ~1.
    EXPECT_NEAR(s.AverageSpeed(0, 1, 0.0), 1.0, 1e-3);
}

// ---------------------------------------------------------------------------
// Free-flow Bayesian prior
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, PriorReportsFreeFlowOnEmptyEdge) {
    auto s = MakeFresh();
    s.prior_weight = 1.0;
    s.prior_speed = 10.0;
    EXPECT_FALSE(s.Has(0, 1));
    // With no observations, the estimate falls back entirely to the prior.
    EXPECT_DOUBLE_EQ(s.AverageSpeed(0, 1, 0.0), 10.0);
}

TEST(GraphEdgeStatistics, PriorBlendedWithObservation) {
    auto s = MakeFresh();
    s.prior_weight = 1.0;
    s.prior_speed = 10.0;
    s.Record(0, 1, 0.0, 1.0, 2.0, 1.0);   // speed 2, weight 1
    // sum_w = 1 (prior) + 1 (obs) = 2
    // sum_w_over_v = 1/10 + 1/2 = 0.6
    // harmonic mean = 2 / 0.6 = 10/3
    EXPECT_NEAR(s.AverageSpeed(0, 1, 1.0), 10.0 / 3.0, 1e-12);
}

TEST(GraphEdgeStatistics, PriorWeightDominatedByManyObservations) {
    auto s = MakeFresh();
    s.prior_weight = 1.0;
    s.prior_speed = 100.0;
    // Twenty slow passages should swamp a single virtual prior passage.
    for (int i = 0; i < 20; ++i) {
        const double t = static_cast<double>(i);
        s.Record(0, 1, t, t + 1.0, 1.0, t + 1.0);  // speed 1
    }
    // sum_w = 21, sum_w_over_v = 20*1 + 1/100 = 20.01
    // harmonic ~= 21 / 20.01 ~= 1.0495
    EXPECT_NEAR(s.AverageSpeed(0, 1, 30.0), 21.0 / 20.01, 1e-9);
}

// ---------------------------------------------------------------------------
// Age-based eviction (memory bound)
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, AgeBasedEvictionOnRecord) {
    auto s = MakeFresh();
    s.max_age = 5.0;
    // Insert at t=0 .. t=4, then a fresh record at t=10 should evict all.
    for (int i = 0; i < 5; ++i) {
        const double t = static_cast<double>(i);
        s.Record(0, 1, t, t + 0.5, 1.0, t + 0.5);
    }
    EXPECT_TRUE(s.Has(0, 1));
    // Now record well in the future; evicts old entries whose t_exit < 10 - 5
    s.Record(0, 1, 10.0, 11.0, 2.0, 11.0);
    const auto* window = s.Window(0, 1);
    ASSERT_NE(window, nullptr);
    // Anything with t_exit < 6 should be gone.
    for (const auto& p : *window) {
        EXPECT_GE(p.t_exit, 6.0);
    }
    EXPECT_EQ(window->back().t_exit, 11.0);
}

TEST(GraphEdgeStatistics, AgeBasedEvictionOnQuery) {
    auto s = MakeFresh();
    s.max_age = 2.0;
    s.Record(0, 1, 0.0, 1.0, 4.0, 1.0);   // exits at t=1
    s.Record(0, 1, 5.0, 6.0, 4.0, 6.0);   // exits at t=6
    // Querying at t=7 with max_age=2 should drop the first passage
    // (t_exit=1 < 7 - 2 = 5), leaving only speed = 4.
    EXPECT_DOUBLE_EQ(s.AverageSpeed(0, 1, 7.0), 4.0);
    const auto* window = s.Window(0, 1);
    ASSERT_NE(window, nullptr);
    EXPECT_EQ(window->size(), 1u);
    EXPECT_DOUBLE_EQ(window->front().t_exit, 6.0);
}

TEST(GraphEdgeStatistics, InfiniteMaxAgeKeepsEverything) {
    auto s = MakeFresh();
    s.max_age = std::numeric_limits<double>::infinity();
    s.Record(0, 1, 0.0, 1.0, 1.0, 1e9);
    s.Record(0, 1, 1e6, 1e6 + 1.0, 1.0, 1e9);
    const auto* window = s.Window(0, 1);
    ASSERT_NE(window, nullptr);
    EXPECT_EQ(window->size(), 2u);
}

TEST(GraphEdgeStatistics, NoSizeCapEvictionWithoutMaxAge) {
    // Without a max_age limit there is no implicit size cap: every passage
    // is retained. This is the dual of the test above with many records.
    auto s = MakeFresh();
    for (int i = 0; i < 1000; ++i) {
        const double t = static_cast<double>(i);
        s.Record(0, 1, t, t + 0.5, 1.0, t + 0.5);
    }
    const auto* window = s.Window(0, 1);
    ASSERT_NE(window, nullptr);
    EXPECT_EQ(window->size(), 1000u);
}

// ---------------------------------------------------------------------------
// Independent windows per edge
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, DifferentEdgesAreIndependent) {
    auto s = MakeFresh();
    s.Record(0, 1, 0.0, 1.0, 2.0, 1.0);
    s.Record(2, 3, 0.0, 1.0, 6.0, 1.0);
    EXPECT_DOUBLE_EQ(s.AverageSpeed(0, 1, 1.0), 2.0);
    EXPECT_DOUBLE_EQ(s.AverageSpeed(2, 3, 1.0), 6.0);
    EXPECT_FALSE(s.Has(0, 2));
    EXPECT_FALSE(s.Has(1, 3));
}
