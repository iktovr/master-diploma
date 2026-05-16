#include <gtest/gtest.h>

#include <memory>

#include "lib/agent.h"
#include "lib/graph.h"
#include "lib/router.h"
#include "lib/simulation.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// base(0,0) --wide-- node(1,0) --wide-- delivery(2,0)
static Graph MakeLinearGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(1.0, 0.0);
    g.AddVertex(2.0, 0.0, Graph::Vertex::delivery);
    g.AddEdge(0, 1);
    g.AddEdge(1, 2);
    return g;
}

static Simulation MakeSimulation(const double speed, const Agents& agents, const Graph& g) {
    auto g_ptr = std::make_shared<const Graph>(g);
    auto router = std::make_shared<const AStarRouter>(g_ptr);
    return Simulation(speed, agents, g_ptr, router);
}

// ---------------------------------------------------------------------------
// Simulation::Step
// ---------------------------------------------------------------------------

TEST(SimulationStep, IdleAgentGetsDispatchedAndStartsMoving) {
    Graph g = MakeLinearGraph();
    Agents agents(1);
    Simulation sim = MakeSimulation(1.0, agents, g);

    // After construction agents are idle (AssignBasePoints sets pos but not state)
    ASSERT_EQ(sim.agents[0].state, Agent::idle);

    sim.Step(0.0, 0.1);

    // Dispatch::Step should have assigned a route → agent is now moving
    EXPECT_EQ(sim.agents[0].state, Agent::move);
}

TEST(SimulationStep, MovingAgentAdvancesPosition) {
    Graph g = MakeLinearGraph();
    Agents agents(1);
    const double speed = 2.0;
    Simulation sim = MakeSimulation(speed, agents, g);

    // First step: dispatch assigns a route, agent starts moving
    sim.Step(0.0, 0.1);
    ASSERT_EQ(sim.agents[0].state, Agent::move);

    const Point pos_before = sim.agents[0].pos;

    // Second step: agent actually moves
    sim.Step(0.1, 0.1);

    const Point pos_after = sim.agents[0].pos;

    // Position must have changed
    EXPECT_FALSE(pos_before == pos_after);
}

TEST(SimulationStep, AgentEventuallyBecomesIdleAfterFinishingRoute) {
    Graph g = MakeLinearGraph();
    Agents agents(1);
    // High speed so the 2-unit route is covered in a few steps
    const double speed = 10.0;
    const double dt = 0.5;
    Simulation sim = MakeSimulation(speed, agents, g);

    // Run enough steps to complete at least one delivery round-trip
    bool became_idle_again = false;
    for (int i = 0; i < 100; ++i) {
        sim.Step(i * dt, dt);
        if (i > 0 && sim.agents[0].state == Agent::idle) {
            became_idle_again = true;
            break;
        }
    }

    EXPECT_TRUE(became_idle_again);
}

// ---------------------------------------------------------------------------
// Simulation::Simulate
// ---------------------------------------------------------------------------

TEST(SimulationSimulate, TerminatesWithoutHanging) {
    Graph g = MakeLinearGraph();
    Agents agents(1);
    Simulation sim = MakeSimulation(5.0, agents, g);

    // Should return in finite time; if it hangs the test runner will time out
    sim.Simulate(1.0, 0.1);
}

TEST(SimulationSimulate, AgentMovesFromInitialPosition) {
    Graph g = MakeLinearGraph();
    Agents agents(1);
    const double speed = 5.0;
    Simulation sim = MakeSimulation(speed, agents, g);

    const Point initial_pos = sim.agents[0].pos;

    sim.Simulate(1.0, 0.1);

    // After 1 virtual second at speed 5 the agent must have moved
    EXPECT_FALSE(sim.agents[0].pos == initial_pos);
}

TEST(SimulationSimulate, MultipleAgentsAllMove) {
    Graph g = MakeLinearGraph();
    Agents agents(2);
    Simulation sim = MakeSimulation(3.0, agents, g);

    // Record starting positions after base assignment
    const Point p0 = sim.agents[0].pos;
    const Point p1 = sim.agents[1].pos;

    sim.Simulate(2.0, 0.1);

    // Both agents must have moved
    EXPECT_FALSE(sim.agents[0].pos == p0);
    EXPECT_FALSE(sim.agents[1].pos == p1);
}
