#include <gtest/gtest.h>

#include <memory>

#include "lib/agent.h"
#include "lib/graph.h"
#include "lib/router.h"
#include "lib/simulation.h"

using namespace lib;

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

// ---------------------------------------------------------------------------
// Co-directional follow-gap on a shared edge
// ---------------------------------------------------------------------------

// Build a route from p0 to p1 with given vertex ids, two endpoints only.
static Linestring StraightRoute(const Point& a, const Point& b) {
    Linestring r;
    r.push_back(a);
    r.push_back(b);
    return r;
}

TEST(SimulationFollowGap, FollowerCannotOvertakeLeader) {
    Graph g = MakeLinearGraph();
    Agents agents(2);
    const double speed = 1.0;
    Simulation sim = MakeSimulation(speed, agents, g);

    // Manually place both agents on the same directed edge (0 -> 2) at
    // distinct positions, both moving in the +x direction.
    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    sim.agents[0].SetRoute(route, {10, 11}, /*t_now=*/0.0);
    sim.agents[1].SetRoute(route, {10, 11}, /*t_now=*/0.0);
    // Place leader (index 1) ahead of follower (index 0).
    sim.agents[0].route_follower.Move(5.0);
    sim.agents[1].route_follower.Move(5.0 + kAgentFollowGap + 0.5);
    sim.agents[0].pos = sim.agents[0].route_follower.pos;
    sim.agents[1].pos = sim.agents[1].route_follower.pos;
    sim.agents[0].state = Agent::move;
    sim.agents[1].state = Agent::move;

    // Run many steps. Follower keeps pushing forward but should never get
    // within kAgentFollowGap of the (stationary-speed-matched) leader.
    for (int i = 0; i < 200; ++i) {
        sim.Step(i * 0.1, 0.1);
        const double xf = sim.agents[0].route_follower.x;
        const double xl = sim.agents[1].route_follower.x;
        EXPECT_LE(xf + kAgentFollowGap - 1e-6, xl)
            << "follower overtook/closed gap at iteration " << i
            << " (xf=" << xf << ", xl=" << xl << ")";
    }
}

TEST(SimulationFollowGap, WaitingLeaderDoesNotBlockFollower) {
    Graph g = MakeLinearGraph();
    Agents agents(2);
    const double speed = 2.0;
    Simulation sim = MakeSimulation(speed, agents, g);

    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    sim.agents[0].SetRoute(route, {10, 11}, /*t_now=*/0.0);
    sim.agents[1].SetRoute(route, {10, 11}, /*t_now=*/0.0);
    sim.agents[0].route_follower.Move(1.0);
    sim.agents[1].route_follower.Move(2.0);  // leader by position
    sim.agents[0].pos = sim.agents[0].route_follower.pos;
    sim.agents[1].pos = sim.agents[1].route_follower.pos;
    sim.agents[0].state = Agent::move;
    sim.agents[1].state = Agent::wait;  // waiting in front of follower

    const auto caps = sim.ComputeFollowCaps();
    EXPECT_TRUE(std::isinf(caps[0]))
        << "waiting leader should not throttle the follower";
}

TEST(SimulationFollowGap, IdleLeaderDoesNotBlockFollower) {
    Graph g = MakeLinearGraph();
    Agents agents(2);
    Simulation sim = MakeSimulation(1.0, agents, g);

    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    sim.agents[0].SetRoute(route, {10, 11}, /*t_now=*/0.0);
    sim.agents[1].SetRoute(route, {10, 11}, /*t_now=*/0.0);
    sim.agents[0].route_follower.Move(1.0);
    sim.agents[1].route_follower.Move(3.0);
    sim.agents[0].pos = sim.agents[0].route_follower.pos;
    sim.agents[1].pos = sim.agents[1].route_follower.pos;
    sim.agents[0].state = Agent::move;
    sim.agents[1].state = Agent::idle;

    const auto caps = sim.ComputeFollowCaps();
    EXPECT_TRUE(std::isinf(caps[0]));
}

TEST(SimulationFollowGap, OppositeDirectionsAreUnaffected) {
    Graph g = MakeLinearGraph();
    Agents agents(2);
    Simulation sim = MakeSimulation(1.0, agents, g);

    // Two agents on the SAME geometric edge but opposite directions
    // (different directed-edge keys 10->11 vs 11->10).
    const Linestring route_fwd = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    const Linestring route_bwd = StraightRoute(Point{100.0, 0.0}, Point{0.0, 0.0});
    sim.agents[0].SetRoute(route_fwd, {10, 11}, 0.0);
    sim.agents[1].SetRoute(route_bwd, {11, 10}, 0.0);
    sim.agents[0].route_follower.Move(5.0);
    sim.agents[1].route_follower.Move(5.0);
    sim.agents[0].pos = sim.agents[0].route_follower.pos;
    sim.agents[1].pos = sim.agents[1].route_follower.pos;
    sim.agents[0].state = Agent::move;
    sim.agents[1].state = Agent::move;

    const auto caps = sim.ComputeFollowCaps();
    EXPECT_TRUE(std::isinf(caps[0]));
    EXPECT_TRUE(std::isinf(caps[1]));
}

// Build a graph with a single edge whose narrowness is configurable.
// Returns a graph where vertex 0 (base) connects to vertex 1 (delivery)
// via an edge of the requested narrow flag.
static Graph MakeSingleEdgeGraph(bool narrow) {
    Graph g;
    g.AddVertex(0.0, 0.0, Graph::Vertex::base);
    g.AddVertex(100.0, 0.0, Graph::Vertex::delivery);
    g.AddEdge(0, 1, narrow);
    return g;
}

TEST(SimulationFollowGap, WideEdgeAllowsFiveAgentsClustered) {
    Graph g = MakeSingleEdgeGraph(/*narrow=*/false);
    Agents agents(kWideEdgeCapacity);
    Simulation sim = MakeSimulation(1.0, agents, g);

    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    for (int i = 0; i < kWideEdgeCapacity; ++i) {
        sim.agents[i].SetRoute(route, {0, 1}, 0.0);
        sim.agents[i].route_follower.Move(10.0);  // all at x=10
        sim.agents[i].pos = sim.agents[i].route_follower.pos;
        sim.agents[i].state = Agent::move;
    }

    const auto caps = sim.ComputeFollowCaps();
    for (int i = 0; i < kWideEdgeCapacity; ++i) {
        EXPECT_TRUE(std::isinf(caps[i]))
            << "agent " << i << " should be uncapped (cluster of " << kWideEdgeCapacity << " on wide edge)";
    }
}

TEST(SimulationFollowGap, WideEdgeAgentIsCapped) {
    Graph g = MakeSingleEdgeGraph(/*narrow=*/false);
    Agents agents(kWideEdgeCapacity+1);
    Simulation sim = MakeSimulation(1.0, agents, g);

    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    for (int i = 0; i < kWideEdgeCapacity; ++i) {
        sim.agents[i].SetRoute(route, {0, 1}, 0.0);
        sim.agents[i].route_follower.Move(10.0);
        sim.agents[i].pos = sim.agents[i].route_follower.pos;
        sim.agents[i].state = Agent::move;
    }
    sim.agents[kWideEdgeCapacity].SetRoute(route, {0, 1}, 0.0);
    sim.agents[kWideEdgeCapacity].route_follower.Move(5.0);
    sim.agents[kWideEdgeCapacity].pos = sim.agents[kWideEdgeCapacity].route_follower.pos;
    sim.agents[kWideEdgeCapacity].state = Agent::move;

    const auto caps = sim.ComputeFollowCaps();
    EXPECT_NEAR(caps[kWideEdgeCapacity], 4.2, 1e-9);
    for (int i = 0; i < kWideEdgeCapacity; ++i) {
        EXPECT_TRUE(std::isinf(caps[i]));
    }
}

TEST(SimulationFollowGap, NarrowEdgeStillBlocksSecondAgent) {
    Graph g = MakeSingleEdgeGraph(/*narrow=*/true);
    Agents agents(2);
    Simulation sim = MakeSimulation(1.0, agents, g);

    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{100.0, 0.0});
    sim.agents[0].SetRoute(route, {0, 1}, 0.0);
    sim.agents[1].SetRoute(route, {0, 1}, 0.0);
    sim.agents[0].route_follower.Move(5.0);
    sim.agents[1].route_follower.Move(8.0);  // leader
    for (int i = 0; i < 2; ++i) {
        sim.agents[i].pos = sim.agents[i].route_follower.pos;
        sim.agents[i].state = Agent::move;
    }

    const auto caps = sim.ComputeFollowCaps();
    // Follower: cap = 8 - 0.8 - 5 = 2.2.
    EXPECT_NEAR(caps[0], 2.2, 1e-9);
    EXPECT_TRUE(std::isinf(caps[1]));
}

TEST(SimulationFollowGap, ChainOfThreeAgentsMaintainsGaps) {
    Graph g = MakeLinearGraph();
    Agents agents(3);
    const double speed = 5.0;
    Simulation sim = MakeSimulation(speed, agents, g);

    const Linestring route = StraightRoute(Point{0.0, 0.0}, Point{1000.0, 0.0});
    for (int i = 0; i < 3; ++i) {
        sim.agents[i].SetRoute(route, {10, 11}, 0.0);
        sim.agents[i].state = Agent::move;
    }
    // Start positions: 0 (trailing), middle, leader.
    sim.agents[0].route_follower.Move(0.0);
    sim.agents[1].route_follower.Move(2.0);
    sim.agents[2].route_follower.Move(4.0);
    for (int i = 0; i < 3; ++i) {
        sim.agents[i].pos = sim.agents[i].route_follower.pos;
    }

    for (int i = 0; i < 100; ++i) {
        sim.Step(i * 0.1, 0.1);
        const double x0 = sim.agents[0].route_follower.x;
        const double x1 = sim.agents[1].route_follower.x;
        const double x2 = sim.agents[2].route_follower.x;
        EXPECT_LE(x0 + kAgentFollowGap - 1e-6, x1);
        EXPECT_LE(x1 + kAgentFollowGap - 1e-6, x2);
    }
}
