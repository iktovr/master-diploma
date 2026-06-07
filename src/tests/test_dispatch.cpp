#include <gtest/gtest.h>

#include <memory>

#include "lib/agent.h"
#include "lib/dispatch.h"
#include "lib/graph.h"
#include "lib/geometry.h"
#include "lib/router.h"

using namespace lib;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::shared_ptr<const Graph> AsShared(const Graph& g) {
    return std::make_shared<const Graph>(g);
}

static std::shared_ptr<const IRouter> MakeRouter(const std::shared_ptr<const Graph>& g) {
    return std::make_shared<const AStarRouter>(g);
}

// ---------------------------------------------------------------------------
// Graph helpers
// ---------------------------------------------------------------------------

// Minimal graph:
//   vertex 0 = base   at (0, 0)
//   vertex 1 = delivery at (1, 0)
//   edge 0-1
static Graph MakeSimpleGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(1.0, 0.0, Graph::Vertex::delivery);
    g.AddEdge(0, 1);
    return g;
}

// Graph with two bases and one delivery:
//   vertex 0 = base   at (0, 0)
//   vertex 1 = base   at (2, 0)
//   vertex 2 = delivery at (1, 1)
//   edges 0-2, 1-2
static Graph MakeTwoBaseGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(2.0, 0.0, Graph::Vertex::base);
    g.AddVertex(1.0, 1.0, Graph::Vertex::delivery);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);
    return g;
}

// ---------------------------------------------------------------------------
// Dispatch::AssignBasePoints
// ---------------------------------------------------------------------------

TEST(DispatchAssignBasePoints, SingleAgentSingleBase) {
    auto g = AsShared(MakeSimpleGraph());
    Agents agents(1);
    Dispatch d(g, MakeRouter(g), agents);

    d.AssignBasePoints(agents);

    // The only base vertex is id=0 at (0,0)
    EXPECT_EQ(agents[0].base, 0);
    EXPECT_NEAR(agents[0].pos.x(), 0.0, 1e-9);
    EXPECT_NEAR(agents[0].pos.y(), 0.0, 1e-9);
}

TEST(DispatchAssignBasePoints, TwoAgentsTwoBases) {
    auto g = AsShared(MakeTwoBaseGraph());
    Agents agents(2);
    Dispatch d(g, MakeRouter(g), agents);

    d.AssignBasePoints(agents);

    // base_points = [0, 1] (in vertex order)
    EXPECT_EQ(agents[0].base, 0);
    EXPECT_EQ(agents[1].base, 1);

    EXPECT_NEAR(agents[0].pos.x(), g->vertices[0].pos.x(), 1e-9);
    EXPECT_NEAR(agents[0].pos.y(), g->vertices[0].pos.y(), 1e-9);
    EXPECT_NEAR(agents[1].pos.x(), g->vertices[1].pos.x(), 1e-9);
    EXPECT_NEAR(agents[1].pos.y(), g->vertices[1].pos.y(), 1e-9);
}

TEST(DispatchAssignBasePoints, RoundRobinWrap) {
    auto g = AsShared(MakeTwoBaseGraph());
    Agents agents(3);
    Dispatch d(g, MakeRouter(g), agents);

    d.AssignBasePoints(agents);

    // 3 agents, 2 bases → agent[2] wraps to base_points[0]
    EXPECT_EQ(agents[0].base, agents[2].base);
    EXPECT_NEAR(agents[0].pos.x(), agents[2].pos.x(), 1e-9);
    EXPECT_NEAR(agents[0].pos.y(), agents[2].pos.y(), 1e-9);
    // agent[1] gets a different base
    EXPECT_NE(agents[0].base, agents[1].base);
}

// ---------------------------------------------------------------------------
// Dispatch::Step
// ---------------------------------------------------------------------------

TEST(DispatchStep, SkipsAgentInMoveState) {
    auto g = AsShared(MakeSimpleGraph());
    auto router = MakeRouter(g);
    Agents agents(1);
    Dispatch d(g, router, agents);
    d.AssignBasePoints(agents);

    agents[0].state = Agent::move;
    // Give it a known route so we can detect if it was changed
    agents[0].SetRoute(router->GetRoute(0, 1));
    const Linestring original_route = agents[0].Route();

    d.Step(0.0, agents);

    // State and route must be unchanged
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_EQ(agents[0].Route().size(), original_route.size());
}

TEST(DispatchStep, SkipsAgentInWaitState) {
    auto g = AsShared(MakeSimpleGraph());
    Agents agents(1);
    Dispatch d(g, MakeRouter(g), agents);
    d.AssignBasePoints(agents);

    agents[0].state = Agent::wait;

    d.Step(0.0, agents);

    EXPECT_EQ(agents[0].state, Agent::wait);
}

TEST(DispatchStep, IdleAgentWithNoOrderGetsDeliveryRoute) {
    auto g = AsShared(MakeSimpleGraph());
    Agents agents(1);
    Dispatch d(g, MakeRouter(g), agents);
    d.AssignBasePoints(agents);

    // current_order[0] == -1 (no order yet), agent is idle
    ASSERT_EQ(agents[0].state, Agent::idle);

    d.Step(0.0, agents);

    // Agent should now be moving towards a delivery point
    EXPECT_EQ(agents[0].state, Agent::move);
    // Route must be non-empty and start at the base position
    ASSERT_FALSE(agents[0].Route().empty());
    EXPECT_NEAR(agents[0].Route().front().x(), g->vertices[agents[0].base].pos.x(), 1e-9);
    EXPECT_NEAR(agents[0].Route().front().y(), g->vertices[agents[0].base].pos.y(), 1e-9);
    // current_order must now hold a valid delivery vertex id
    EXPECT_NE(d.current_order[0], -1);
}

TEST(DispatchStep, IdleAgentWithOrderGetsReturnRouteAndOrderCleared) {
    auto g = AsShared(MakeSimpleGraph());
    Agents agents(1);
    Dispatch d(g, MakeRouter(g), agents);
    d.AssignBasePoints(agents);

    // Simulate that agent already has an order (delivery vertex = 1)
    d.current_order[0] = 1;
    agents[0].state = Agent::idle;

    d.Step(0.0, agents);

    // Agent should now be returning to base
    EXPECT_EQ(agents[0].state, Agent::move);
    // Route must end at the base position
    ASSERT_FALSE(agents[0].Route().empty());
    EXPECT_NEAR(agents[0].Route().back().x(), g->vertices[agents[0].base].pos.x(), 1e-9);
    EXPECT_NEAR(agents[0].Route().back().y(), g->vertices[agents[0].base].pos.y(), 1e-9);
    // Order must be cleared
    EXPECT_EQ(d.current_order[0], -1);
}

TEST(DispatchStep, MultipleAgentsOnlyIdleOnesAreDispatched) {
    auto g = AsShared(MakeTwoBaseGraph());
    auto router = MakeRouter(g);
    Agents agents(2);
    Dispatch d(g, router, agents);
    d.AssignBasePoints(agents);

    agents[0].state = Agent::idle;
    agents[1].state = Agent::move;
    agents[1].SetRoute(router->GetRoute(0, 2));

    d.Step(0.0, agents);

    EXPECT_EQ(agents[0].state, Agent::move);  // was idle → dispatched
    EXPECT_EQ(agents[1].state, Agent::move);  // was move → untouched
}

// ---------------------------------------------------------------------------
// Delivery-to-base ownership
// ---------------------------------------------------------------------------

// Graph with two bases and two deliveries, each delivery owned by one base:
//   vertex 0 = base A      at (0, 0)
//   vertex 1 = base B      at (10, 0)
//   vertex 2 = delivery    at (1, 0), owned by base 0
//   vertex 3 = delivery    at (9, 0), owned by base 1
static Graph MakeOwnedDeliveriesGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(10.0, 0.0, Graph::Vertex::base);
    g.AddVertex(1.0, 0.0, Graph::Vertex::delivery);
    g.AddVertex(9.0, 0.0, Graph::Vertex::delivery);
    g.vertices[2].owning_bases = {0};
    g.vertices[3].owning_bases = {1};
    g.AddEdge(0, 2);
    g.AddEdge(2, 3);
    g.AddEdge(3, 1);
    return g;
}

TEST(DispatchNewOrder, OnlyOwnedDeliveriesPerBase) {
    auto g = AsShared(MakeOwnedDeliveriesGraph());
    Agents agents(2);
    Dispatch d(g, MakeRouter(g), agents);

    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(d.NewOrder(0), 2);
        EXPECT_EQ(d.NewOrder(1), 3);
    }
}

// Graph with two bases, one owned delivery and one unbound delivery:
//   vertex 0 = base A      at (0, 0)
//   vertex 1 = base B      at (10, 0)
//   vertex 2 = delivery    at (1, 0), owned by base 0
//   vertex 3 = delivery    at (5, 5), unbound
static Graph MakeMixedDeliveriesGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(10.0, 0.0, Graph::Vertex::base);
    g.AddVertex(1.0, 0.0, Graph::Vertex::delivery);
    g.AddVertex(5.0, 5.0, Graph::Vertex::delivery);
    g.vertices[2].owning_bases = {0};
    g.AddEdge(0, 2);
    g.AddEdge(2, 3);
    g.AddEdge(3, 1);
    return g;
}

TEST(DispatchNewOrder, UnboundDeliveriesAvailableToAllBases) {
    auto g = AsShared(MakeMixedDeliveriesGraph());
    Agents agents(2);
    Dispatch d(g, MakeRouter(g), agents);

    bool base1_got_unbound = false;
    for (int i = 0; i < 200; ++i) {
        int o0 = d.NewOrder(0);
        EXPECT_TRUE(o0 == 2 || o0 == 3);
        int o1 = d.NewOrder(1);
        EXPECT_EQ(o1, 3);
        if (o1 == 3) base1_got_unbound = true;
    }
    EXPECT_TRUE(base1_got_unbound);
}

TEST(DispatchNewOrder, LegacyBehaviorWhenNoOwnership) {
    auto g = AsShared(MakeTwoBaseGraph());
    Agents agents(2);
    Dispatch d(g, MakeRouter(g), agents);

    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(d.NewOrder(0), 2);
        EXPECT_EQ(d.NewOrder(1), 2);
    }
}
