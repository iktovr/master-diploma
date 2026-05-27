#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include "lib/graph.h"
#include "lib/router.h"
#include "lib/statistics.h"

using ::testing::ElementsAre;

using namespace lib;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Build an AStarRouter for a graph passed by value.
static std::shared_ptr<const AStarRouter> MakeRouter(const Graph& g) {
    auto g_ptr = std::make_shared<const Graph>(g);
    return std::make_shared<const AStarRouter>(g_ptr);
}

// Compare a router-produced Linestring against an expected sequence of vertex
// ids by checking positions.
static void ExpectRoute(const Graph& g, const Linestring& route, const std::vector<int>& expected_ids) {
    ASSERT_EQ(route.size(), expected_ids.size());
    for (size_t i = 0; i < expected_ids.size(); ++i) {
        EXPECT_DOUBLE_EQ(route[i].x(), g.vertices[expected_ids[i]].pos.x());
        EXPECT_DOUBLE_EQ(route[i].y(), g.vertices[expected_ids[i]].pos.y());
    }
}

// ---------------------------------------------------------------------------
// AStarRouter::GetRoute
// ---------------------------------------------------------------------------

TEST(AStarRouterGetRoute, SimpleGraph) {
    Graph g;
    g.AddVertex(-1, 0);
    g.AddVertex(0, 1);
    g.AddVertex(1, 1);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);
    auto router = MakeRouter(g);

    for (int u = 0; u < 2; ++u) {
        for (int v = 0; v < 2; ++v) {
            if (u != v) {
                ExpectRoute(g, router->GetRoute(u, v), {u, v});
            } else {
                ExpectRoute(g, router->GetRoute(u, v), {u});
            }
        }
    }
}

TEST(AStarRouterGetRoute, SameSourceAndDestination) {
    Graph g;
    g.AddVertex(0.0, 0.0);
    g.AddVertex(1.0, 0.0);
    g.AddEdge(0, 1);
    auto router = MakeRouter(g);

    ExpectRoute(g, router->GetRoute(0, 0), {0});
    ExpectRoute(g, router->GetRoute(1, 1), {1});
}

TEST(AStarRouterGetRoute, PrefersShorterPath) {
    // 0 --10-- 1
    //  \      /
    //   --2--    (vertex 2 at x=1, direct edge 0→2 length=1, edge 2→1 length=9)
    Graph g;
    g.AddVertex(0.0, 0.0);   // 0
    g.AddVertex(10.0, 0.0);  // 1
    g.AddVertex(1.0, 0.0);   // 2
    g.AddEdge(0, 1);  // length 10
    g.AddEdge(1, 2);  // length 9
    g.AddEdge(0, 2);  // length 1
    auto router = MakeRouter(g);

    ExpectRoute(g, router->GetRoute(0, 2), {0, 2});
    ExpectRoute(g, router->GetRoute(0, 1), {0, 1});
    ExpectRoute(g, router->GetRoute(2, 1), {2, 1});
}

TEST(AStarRouterGetRoute, LinearChain) {
    // 0 - 1 - 2 - 3 - 4
    Graph g;
    for (int i = 0; i < 5; ++i) {
        g.AddVertex(static_cast<double>(i), 0.0);
    }
    for (int i = 0; i < 4; ++i) {
        g.AddEdge(i, i + 1);
    }
    auto router = MakeRouter(g);

    ExpectRoute(g, router->GetRoute(0, 4), {0, 1, 2, 3, 4});
    ExpectRoute(g, router->GetRoute(4, 0), {4, 3, 2, 1, 0});
    ExpectRoute(g, router->GetRoute(1, 3), {1, 2, 3});
}

TEST(AStarRouterGetRoute, MediumGraph) {
    Graph g;
    g.AddVertex(-1, 0);
    g.AddVertex(0, 1);
    g.AddVertex(1, 1);
    g.AddVertex(2, 0);
    g.AddVertex(1, -1);
    g.AddVertex(0, -1);
    g.AddVertex(3, 1);
    g.AddVertex(2, 1);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);
    g.AddEdge(0, 3);
    g.AddEdge(0, 5);
    g.AddEdge(4, 3);
    g.AddEdge(4, 5);
    g.AddEdge(3, 6);
    g.AddEdge(6, 7);
    auto router = MakeRouter(g);

    ExpectRoute(g, router->GetRoute(0, 2), {0, 2});
    ExpectRoute(g, router->GetRoute(0, 3), {0, 3});
    ExpectRoute(g, router->GetRoute(1, 3), {1, 0, 3});
    ExpectRoute(g, router->GetRoute(2, 3), {2, 0, 3});
    ExpectRoute(g, router->GetRoute(0, 4), {0, 5, 4});
    ExpectRoute(g, router->GetRoute(1, 4), {1, 0, 5, 4});
    ExpectRoute(g, router->GetRoute(2, 4), {2, 0, 5, 4});
    ExpectRoute(g, router->GetRoute(1, 5), {1, 0, 5});
    ExpectRoute(g, router->GetRoute(2, 5), {2, 0, 5});
    ExpectRoute(g, router->GetRoute(3, 5), {3, 4, 5});
    ExpectRoute(g, router->GetRoute(0, 6), {0, 3, 6});
    ExpectRoute(g, router->GetRoute(1, 6), {1, 0, 3, 6});
    ExpectRoute(g, router->GetRoute(2, 6), {2, 0, 3, 6});
    ExpectRoute(g, router->GetRoute(4, 6), {4, 3, 6});
    ExpectRoute(g, router->GetRoute(5, 6), {5, 4, 3, 6});
    ExpectRoute(g, router->GetRoute(0, 7), {0, 3, 6, 7});
    ExpectRoute(g, router->GetRoute(1, 7), {1, 0, 3, 6, 7});
    ExpectRoute(g, router->GetRoute(2, 7), {2, 0, 3, 6, 7});
    ExpectRoute(g, router->GetRoute(3, 7), {3, 6, 7});
    ExpectRoute(g, router->GetRoute(4, 7), {4, 3, 6, 7});
    ExpectRoute(g, router->GetRoute(5, 7), {5, 4, 3, 6, 7});
}

TEST(AStarRouterGetRoute, PositionsMatchPath) {
    Graph g;
    g.AddVertex(0.0, 0.0);
    g.AddVertex(1.0, 0.0);
    g.AddVertex(2.0, 1.0);
    g.AddEdge(0, 1);
    g.AddEdge(1, 2);
    auto router = MakeRouter(g);

    ExpectRoute(g, router->GetRoute(0, 2), {0, 1, 2});
}

TEST(AStarRouterGetRoute, SingleNodeRoute) {
    Graph g;
    g.AddVertex(5.0, 7.0);
    auto router = MakeRouter(g);

    auto route = router->GetRoute(0, 0);

    ASSERT_EQ(route.size(), 1u);
    EXPECT_DOUBLE_EQ(route[0].x(), 5.0);
    EXPECT_DOUBLE_EQ(route[0].y(), 7.0);
}

// ---------------------------------------------------------------------------
// StatAStarRouter
// ---------------------------------------------------------------------------

// Build a StatAStarRouter for a graph passed by value, with the given stats
// and max_speed. The stats pointer must outlive the returned router.
static std::shared_ptr<const StatAStarRouter> MakeStatRouter(
    const Graph& g, GraphEdgeStatistics* stats, double max_speed) {
    auto g_ptr = std::make_shared<const Graph>(g);
    return std::make_shared<const StatAStarRouter>(g_ptr, stats, max_speed);
}

TEST(StatAStarRouterGetRoute, EmptyStatsMatchesAStar) {
    // Diamond: 0 - 1 - 3 and 0 - 2 - 3, with 0-1-3 cheaper by length.
    Graph g;
    g.AddVertex(0.0, 0.0);    // 0
    g.AddVertex(1.0, 0.0);    // 1
    g.AddVertex(0.0, 5.0);    // 2
    g.AddVertex(2.0, 0.0);    // 3
    g.AddEdge(0, 1);  // length 1
    g.AddEdge(1, 3);  // length 1
    g.AddEdge(0, 2);  // length 5
    g.AddEdge(2, 3);  // ~5.385

    GraphEdgeStatistics stats;  // empty
    auto router = MakeStatRouter(g, &stats, /*max_speed=*/1.0);

    ExpectRoute(g, router->GetRoute(0, 3), {0, 1, 3});
}

TEST(StatAStarRouterGetRoute, SlowEdgeForcesDetour) {
    // Same diamond as above, but edge 1-3 is recorded as extremely slow,
    // so the longer 0-2-3 path becomes faster in travel-time.
    Graph g;
    g.AddVertex(0.0, 0.0);    // 0
    g.AddVertex(1.0, 0.0);    // 1
    g.AddVertex(0.0, 5.0);    // 2
    g.AddVertex(2.0, 0.0);    // 3
    g.AddEdge(0, 1);  // length 1
    g.AddEdge(1, 3);  // length 1
    g.AddEdge(0, 2);  // length 5
    g.AddEdge(2, 3);  // ~5.385

    GraphEdgeStatistics stats;
    // Make 1-3 traversal 1000x slower than max_speed.
    const double max_speed = 1.0;
    const double slow_speed = 0.001;
    // Record several slow passages on edge 1-3.
    for (int i = 0; i < 5; ++i) {
        stats.Record(1, 3, /*t_enter=*/0.0, /*t_exit=*/1.0 / slow_speed,
                     /*length=*/1.0, /*t_now=*/0.0);
    }
    ASSERT_TRUE(stats.Has(1, 3));

    auto router = MakeStatRouter(g, &stats, max_speed);

    // Travel time via 0-1-3: 1/max_speed + 1/slow_speed ~= 1001
    // Travel time via 0-2-3: (5 + 5.385) / max_speed ~= 10.385
    ExpectRoute(g, router->GetRoute(0, 3), {0, 2, 3});
}

TEST(StatAStarRouterGetRoute, MixedFallbackUsesMaxSpeed) {
    // Linear chain 0 - 1 - 2 - 3 where only edge 0-1 has stats. The router
    // should still produce the only available path.
    Graph g;
    for (int i = 0; i < 4; ++i) {
        g.AddVertex(static_cast<double>(i), 0.0);
    }
    for (int i = 0; i < 3; ++i) {
        g.AddEdge(i, i + 1);
    }

    GraphEdgeStatistics stats;
    stats.Record(0, 1, /*t_enter=*/0.0, /*t_exit=*/0.5,
                 /*length=*/1.0, /*t_now=*/0.0);
    ASSERT_TRUE(stats.Has(0, 1));
    ASSERT_FALSE(stats.Has(1, 2));
    ASSERT_FALSE(stats.Has(2, 3));

    auto router = MakeStatRouter(g, &stats, /*max_speed=*/1.0);
    ExpectRoute(g, router->GetRoute(0, 3), {0, 1, 2, 3});
    ExpectRoute(g, router->GetRoute(3, 0), {3, 2, 1, 0});
}

TEST(StatAStarRouterGetRoute, PrefersFasterButLongerEdge) {
    // 0 --slow,short-- 1
    //  \              /
    //   --fast,long--
    // Direct edge 0-1 has length 1 but is very slow.
    // Detour 0-2-1 has length 10 but uses max_speed.
    Graph g;
    g.AddVertex(0.0, 0.0);    // 0
    g.AddVertex(1.0, 0.0);    // 1
    g.AddVertex(0.0, 5.0);    // 2 (so 0-2 length 5, 2-1 ~5.099)
    g.AddEdge(0, 1);  // length 1
    g.AddEdge(0, 2);  // length 5
    g.AddEdge(2, 1);  // ~5.099

    GraphEdgeStatistics stats;
    // Edge 0-1 observed at speed 0.01 (so travel time ~100).
    for (int i = 0; i < 3; ++i) {
        stats.Record(0, 1, 0.0, 100.0, 1.0, 0.0);
    }

    auto router = MakeStatRouter(g, &stats, /*max_speed=*/1.0);
    // Direct: ~100s. Detour: ~10.099s. Detour should win.
    ExpectRoute(g, router->GetRoute(0, 1), {0, 2, 1});
}

// ---------------------------------------------------------------------------
// StatAStarRouter per-tick edge-speed cache
// ---------------------------------------------------------------------------
//
// These tests pin down the cross-call memoization that StatAStarRouter
// performs to avoid re-evaluating GraphEdgeStatistics::AverageSpeed on every
// edge expansion during a simulation tick. They use the public
// |query_count| counter on GraphEdgeStatistics to assert the expected number
// of underlying AverageSpeed evaluations.

namespace {

// 4-vertex diamond with stats recorded on every edge. Returns the graph and
// fully populates |stats| so that every Cost() call goes through
// AverageSpeed and is therefore observable via query_count.
Graph BuildDiamondAndRecordStats(GraphEdgeStatistics& stats) {
    Graph g;
    g.AddVertex(0.0, 0.0);    // 0
    g.AddVertex(1.0, 0.0);    // 1
    g.AddVertex(0.0, 5.0);    // 2
    g.AddVertex(2.0, 0.0);    // 3
    g.AddEdge(0, 1);
    g.AddEdge(1, 3);
    g.AddEdge(0, 2);
    g.AddEdge(2, 3);
    // Record one fast passage on each undirected edge so AverageSpeed has
    // data to fold and Has() returns true.
    const std::pair<int, int> edges[] = {{0,1},{1,3},{0,2},{2,3}};
    for (auto [u, v] : edges) {
        // Use the graph's actual edge length to keep the recorded speed
        // physically meaningful (not strictly required for the cache test).
        double len = 1.0;
        for (const auto& [n, e] : g.edges[u]) {
            if (n == v) { len = e.length; break; }
        }
        stats.Record(u, v, /*t_enter=*/0.0, /*t_exit=*/len /*speed=1*/,
                     /*length=*/len, /*t_now=*/0.0);
    }
    return g;
}

}  // namespace

TEST(StatAStarRouterCache, CollapsesRepeatedQueriesWithinSameTick) {
    GraphEdgeStatistics stats;
    Graph g = BuildDiamondAndRecordStats(stats);
    auto router = MakeStatRouter(g, &stats, /*max_speed=*/1.0);

    // Warm: a single route at t=0 forces every edge it expands to populate
    // the cache.
    router->GetRoute(0, 3, /*t=*/0.0);
    const std::uint64_t after_first = stats.query_count;
    // Repeating the *same* query at the same t must not trigger any new
    // AverageSpeed evaluation: every edge A* expands is already cached.
    router->GetRoute(0, 3, /*t=*/0.0);
    EXPECT_EQ(stats.query_count, after_first);
    router->GetRoute(0, 3, /*t=*/0.0);
    EXPECT_EQ(stats.query_count, after_first);

    // A *different* (start, finish) pair at the same t may touch edges the
    // first search didn't expand, so query_count may grow — but the total
    // is hard-bounded by the number of distinct undirected edges (4).
    router->GetRoute(3, 0, /*t=*/0.0);
    router->GetRoute(1, 2, /*t=*/0.0);
    router->GetRoute(2, 1, /*t=*/0.0);
    EXPECT_LE(stats.query_count, 4u);
}

TEST(StatAStarRouterCache, InvalidatesOnTimeChange) {
    GraphEdgeStatistics stats;
    Graph g = BuildDiamondAndRecordStats(stats);
    auto router = MakeStatRouter(g, &stats, /*max_speed=*/1.0);

    router->GetRoute(0, 3, /*t=*/0.0);
    const std::uint64_t after_t0 = stats.query_count;
    // Querying at a different t must re-populate the cache, so query_count
    // must strictly increase.
    router->GetRoute(0, 3, /*t=*/1.0);
    EXPECT_GT(stats.query_count, after_t0);
}

TEST(StatAStarRouterCache, InvalidatesOnRecord) {
    GraphEdgeStatistics stats;
    Graph g = BuildDiamondAndRecordStats(stats);
    auto router = MakeStatRouter(g, &stats, /*max_speed=*/1.0);

    router->GetRoute(0, 3, /*t=*/0.0);
    const std::uint64_t after_first = stats.query_count;

    // A mutation to GraphEdgeStatistics between two router calls at the
    // same t must invalidate the cache (otherwise the router would return
    // stale speeds and could pick a wrong path after a sudden congestion
    // event).
    stats.Record(0, 1, /*t_enter=*/0.0, /*t_exit=*/100.0, /*length=*/1.0,
                 /*t_now=*/0.0);
    router->GetRoute(0, 3, /*t=*/0.0);
    EXPECT_GT(stats.query_count, after_first);
}
