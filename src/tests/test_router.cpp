#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include "lib/graph.h"
#include "lib/router.h"

using ::testing::ElementsAre;

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
