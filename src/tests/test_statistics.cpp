#include <gtest/gtest.h>

#include <limits>

#include "lib/statistics.h"

namespace {

GraphEdgeStatistics MakeFresh() {
    GraphEdgeStatistics s;
    s.Clear();
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// GraphEdgeStatistics::Record / AverageSpeed
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
    EXPECT_DOUBLE_EQ(s.AverageSpeed(0, 1, 3.0), 3.0);
    EXPECT_DOUBLE_EQ(s.AverageSpeed(1, 0, 3.0), 3.0);
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
// Capacity-based eviction (sliding window of size kWindow)
// ---------------------------------------------------------------------------

TEST(GraphEdgeStatistics, EvictsOldestWhenWindowExceeded) {
    auto s = MakeFresh();
    const std::size_t n = GraphEdgeStatistics::kWindow + 2;
    for (std::size_t i = 0; i < n; ++i) {
        const double t_enter = static_cast<double>(i);
        const double t_exit  = static_cast<double>(i) + 1.0;
        // length chosen so that speed == i + 1 for easy inspection
        const double length  = static_cast<double>(i) + 1.0;
        s.Record(0, 1, t_enter, t_exit, length, t_exit);
    }
    const auto* window = s.Window(0, 1);
    ASSERT_NE(window, nullptr);
    EXPECT_EQ(window->size(), GraphEdgeStatistics::kWindow);
    // Oldest two should be gone: kept t_exit values should be n-kWindow+1 ... n
    EXPECT_DOUBLE_EQ(window->front().t_exit,
                     static_cast<double>(n - GraphEdgeStatistics::kWindow + 1));
    EXPECT_DOUBLE_EQ(window->back().t_exit, static_cast<double>(n));
}

// ---------------------------------------------------------------------------
// Age-based eviction
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
    // (t_exit=1 < 7 - 2 = 5).
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
