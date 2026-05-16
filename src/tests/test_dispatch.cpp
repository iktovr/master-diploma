#include <gtest/gtest.h>

#include <memory>

#include "lib/agent.h"
#include "lib/dispatch.h"
#include "lib/graph.h"
#include "lib/geometry.h"
#include "lib/router.h"

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

    d.Step(agents);

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

    d.Step(agents);

    EXPECT_EQ(agents[0].state, Agent::wait);
}

TEST(DispatchStep, IdleAgentWithNoOrderGetsDeliveryRoute) {
    auto g = AsShared(MakeSimpleGraph());
    Agents agents(1);
    Dispatch d(g, MakeRouter(g), agents);
    d.AssignBasePoints(agents);

    // current_order[0] == -1 (no order yet), agent is idle
    ASSERT_EQ(agents[0].state, Agent::idle);

    d.Step(agents);

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

    d.Step(agents);

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

    d.Step(agents);

    EXPECT_EQ(agents[0].state, Agent::move);  // was idle → dispatched
    EXPECT_EQ(agents[1].state, Agent::move);  // was move → untouched
}
