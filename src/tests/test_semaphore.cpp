#include <gtest/gtest.h>

#include <memory>

#include "lib/semaphore.h"
#include "lib/graph.h"
#include "lib/agent.h"
#include "lib/geometry.h"

using namespace lib;

static std::shared_ptr<const Graph> AsShared(const Graph& g) {
    return std::make_shared<const Graph>(g);
}

// ---------------------------------------------------------------------------
// Graph helpers
// ---------------------------------------------------------------------------

// One narrow edge: vertex 0=(0,0) <--narrow--> vertex 1=(1,0)
static Graph MakeNarrowGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0);  // 0
    g.AddVertex(1.0, 0.0);  // 1
    g.AddEdge(0, 1, /*narrow=*/true);
    return g;
}

// One wide (non-narrow) edge
static Graph MakeWideGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0);
    g.AddVertex(1.0, 0.0);
    g.AddEdge(0, 1, /*narrow=*/false);
    return g;
}

// Two narrow edges: 0=(0,0) <-> 1=(1,0) <-> 2=(2,0)
static Graph MakeTwoNarrowGraph() {
    Graph g;
    g.AddVertex(0.0, 0.0);  // 0
    g.AddVertex(1.0, 0.0);  // 1
    g.AddVertex(2.0, 0.0);  // 2
    g.AddEdge(0, 1, /*narrow=*/true);
    g.AddEdge(1, 2, /*narrow=*/true);
    return g;
}

// ---------------------------------------------------------------------------
// SemaphoreManager construction
// ---------------------------------------------------------------------------

TEST(SemaphoreManagerConstruction, NoSemaphoresForWideGraph) {
    Graph g = MakeWideGraph();
    SemaphoreManager sm(AsShared(g));
    EXPECT_TRUE(sm.semaphores.empty());
}

TEST(SemaphoreManagerConstruction, OneSemaphoreForOneNarrowEdge) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    EXPECT_EQ(sm.semaphores.size(), 1u);
}

TEST(SemaphoreManagerConstruction, TwoSemaphoresForTwoNarrowEdges) {
    Graph g = MakeTwoNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    EXPECT_EQ(sm.semaphores.size(), 2u);
}

// ---------------------------------------------------------------------------
// Semaphore::Intersection
// ---------------------------------------------------------------------------

TEST(SemaphoreIntersection, IncomingFromLeftSide) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    // route[1] == seg[0], route[2] == seg[1]  →  return 1
    Linestring route;
    route.push_back(Point{-1.0, 0.0});
    route.push_back(sem.segment[0]);
    route.push_back(sem.segment[1]);

    EXPECT_EQ(sem.Intersection(route), 1);
}

TEST(SemaphoreIntersection, IncomingFromRightSide) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    // route[1] == seg[1], route[2] == seg[0]  →  return 2
    Linestring route;
    route.push_back(Point{2.0, 0.0});
    route.push_back(sem.segment[1]);
    route.push_back(sem.segment[0]);

    EXPECT_EQ(sem.Intersection(route), 2);
}

TEST(SemaphoreIntersection, AgentInsideMovingTowardsLeftSide) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    // route[1] == seg[0], but route[2] != seg[1]  →  return -1
    Linestring route;
    route.push_back(Point{-1.0, 0.0});
    route.push_back(sem.segment[0]);
    route.push_back(Point{0.0, 99.0});  // not seg[1]

    EXPECT_EQ(sem.Intersection(route), -1);
}

TEST(SemaphoreIntersection, AgentInsideMovingTowardsRightSide) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    // route[1] == seg[1], but route[2] != seg[0]  →  return -2
    Linestring route;
    route.push_back(Point{2.0, 0.0});
    route.push_back(sem.segment[1]);
    route.push_back(Point{1.0, 99.0});  // not seg[0]

    EXPECT_EQ(sem.Intersection(route), -2);
}

TEST(SemaphoreIntersection, NoIntersection) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    // route does not touch the semaphore segment at all  →  return 0
    Linestring route;
    route.push_back(Point{5.0, 5.0});
    route.push_back(Point{6.0, 5.0});
    route.push_back(Point{7.0, 5.0});

    EXPECT_EQ(sem.Intersection(route), 0);
}

// ---------------------------------------------------------------------------
// Semaphore::Inside / Exit / Empty
// ---------------------------------------------------------------------------

TEST(SemaphoreState, InitiallyEmpty) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    EXPECT_TRUE(sem.Empty());
    EXPECT_FALSE(sem.Inside(0));
}

TEST(SemaphoreState, InsideAfterInsertIntoLeftQueue) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    sem.inner_left_q.insert(42);
    EXPECT_TRUE(sem.Inside(42));
    EXPECT_FALSE(sem.Empty());
}

TEST(SemaphoreState, InsideAfterInsertIntoRightQueue) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    sem.inner_right_q.insert(7);
    EXPECT_TRUE(sem.Inside(7));
    EXPECT_FALSE(sem.Empty());
}

TEST(SemaphoreState, ExitFromLeftQueueClearsAgent) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    sem.inner_left_q.insert(5);
    sem.Exit(5);

    EXPECT_FALSE(sem.Inside(5));
    EXPECT_TRUE(sem.Empty());
}

TEST(SemaphoreState, ExitFromRightQueueClearsAgent) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    sem.inner_right_q.insert(3);
    sem.Exit(3);

    EXPECT_FALSE(sem.Inside(3));
    EXPECT_TRUE(sem.Empty());
}

TEST(SemaphoreState, EmptyOnlyWhenBothInnerQueuesEmpty) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    sem.inner_left_q.insert(1);
    sem.inner_right_q.insert(2);
    EXPECT_FALSE(sem.Empty());

    sem.Exit(1);
    EXPECT_FALSE(sem.Empty());  // right inner queue still has agent 2

    sem.Exit(2);
    EXPECT_TRUE(sem.Empty());
}

// ---------------------------------------------------------------------------
// SemaphoreManager::Step – agent enters waiting state
// ---------------------------------------------------------------------------

// Build a route where the agent is close (< 0.25 units) to the entry node.
// Left approach: agent is just to the right of seg[0], heading seg[0]→seg[1].
static Linestring LeftApproachRoute(const SemaphoreManager::Semaphore& sem) {
    Linestring r;
    r.push_back(Point{sem.segment[0].x() + 0.1, sem.segment[0].y()});
    r.push_back(sem.segment[0]);
    r.push_back(sem.segment[1]);
    return r;
}

// Right approach: agent is just to the left of seg[1], heading seg[1]→seg[0].
static Linestring RightApproachRoute(const SemaphoreManager::Semaphore& sem) {
    Linestring r;
    r.push_back(Point{sem.segment[1].x() - 0.1, sem.segment[1].y()});
    r.push_back(sem.segment[1]);
    r.push_back(sem.segment[0]);
    return r;
}

static Agent MakeAgentWithRoute(const Linestring& route) {
    Agent a;
    a.route_follower.SetRoute(route);
    // SetRoute sets state = move implicitly via Agent::SetRoute, but here we
    // call route_follower directly; set state manually.
    a.state = Agent::move;
    return a;
}

TEST(SemaphoreManagerStep, AgentEntersWaitFromLeftSide) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    Agents agents;
    agents.push_back(MakeAgentWithRoute(LeftApproachRoute(sem)));
    ASSERT_EQ(agents[0].state, Agent::move);

    // When the semaphore inner queues are empty the agent is enqueued and
    // immediately released in the same Step call (no other agent blocks it).
    sm.Step(0.0, agents);

    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_TRUE(sem.Inside(0));
}

TEST(SemaphoreManagerStep, AgentEntersWaitFromRightSide) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    Agents agents;
    agents.push_back(MakeAgentWithRoute(RightApproachRoute(sem)));
    ASSERT_EQ(agents[0].state, Agent::move);

    // Same as left side: immediately released when inner queues are empty.
    sm.Step(0.0, agents);

    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_TRUE(sem.Inside(0));
}

TEST(SemaphoreManagerStep, AgentFarAwayDoesNotWait) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    const auto& sem = sm.semaphores[0];

    // Agent is 0.5 units away from seg[0] – beyond the 0.25 threshold
    Linestring route;
    route.push_back(Point{sem.segment[0].x() + 0.5, sem.segment[0].y()});
    route.push_back(sem.segment[0]);
    route.push_back(sem.segment[1]);

    Agents agents;
    agents.push_back(MakeAgentWithRoute(route));

    sm.Step(0.0, agents);

    EXPECT_EQ(agents[0].state, Agent::move);
}

TEST(SemaphoreManagerStep, IdleAgentIsIgnored) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));

    Agents agents;
    Agent a;
    a.state = Agent::idle;
    agents.push_back(a);

    sm.Step(0.0, agents);

    EXPECT_EQ(agents[0].state, Agent::idle);
}

// ---------------------------------------------------------------------------
// SemaphoreManager::Step – queue release logic
// ---------------------------------------------------------------------------

TEST(SemaphoreManagerStep, WaitingAgentReleasedWhenSemaphoreIsEmpty) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    Agents agents;
    agents.push_back(MakeAgentWithRoute(LeftApproachRoute(sem)));

    // Step 1: inner queues are empty → agent is enqueued and immediately
    // released in the same Step call; it enters inner_left_q.
    sm.Step(0.0, agents);
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_TRUE(sem.Inside(0));

    // Step 2: agent is inside (Inside(0)==true), Intersection returns 1 (≥0)
    // → Exit(0) is called; semaphore becomes empty again.
    sm.Step(0.1, agents);
    EXPECT_FALSE(sem.Inside(0));
    EXPECT_TRUE(sem.Empty());
}

TEST(SemaphoreManagerStep, OpposingAgentsDoNotEnterSimultaneously) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    Agents agents;
    agents.push_back(MakeAgentWithRoute(LeftApproachRoute(sem)));   // agent 0
    agents.push_back(MakeAgentWithRoute(RightApproachRoute(sem)));  // agent 1

    // Both approach at the same time (t=0.0).
    // Both are enqueued; release logic picks one direction (right wins when
    // timestamps are equal because the tie-break condition uses strict <).
    sm.Step(0.0, agents);

    bool left_inside  = sem.Inside(0);
    bool right_inside = sem.Inside(1);

    // They must NOT both be inside simultaneously
    EXPECT_FALSE(left_inside && right_inside);
    // Exactly one must have been released immediately
    EXPECT_TRUE(left_inside || right_inside);

    // The released agent is move; the other is still wait
    int inside_agent  = left_inside ? 0 : 1;
    int waiting_agent = 1 - inside_agent;
    EXPECT_EQ(agents[inside_agent].state,  Agent::move);
    EXPECT_EQ(agents[waiting_agent].state, Agent::wait);
}

TEST(SemaphoreManagerStep, SameSideAgentsAllReleasedTogether) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    Agents agents;
    agents.push_back(MakeAgentWithRoute(LeftApproachRoute(sem)));  // agent 0
    agents.push_back(MakeAgentWithRoute(LeftApproachRoute(sem)));  // agent 1

    // Both approach from the left at t=0; inner queues are empty →
    // both are enqueued and immediately released in the same Step call.
    sm.Step(0.0, agents);
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_EQ(agents[1].state, Agent::move);
    EXPECT_TRUE(sem.Inside(0));
    EXPECT_TRUE(sem.Inside(1));
}

TEST(SemaphoreManagerStep, AgentExitsAndReleasesOpposingQueue) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    Agents agents;
    agents.push_back(MakeAgentWithRoute(LeftApproachRoute(sem)));   // agent 0
    agents.push_back(MakeAgentWithRoute(RightApproachRoute(sem)));  // agent 1

    // Step 0: one direction is released immediately, the other waits.
    sm.Step(0.0, agents);

    int inside_agent  = sem.Inside(0) ? 0 : 1;
    int waiting_agent = 1 - inside_agent;

    EXPECT_EQ(agents[inside_agent].state,  Agent::move);
    EXPECT_EQ(agents[waiting_agent].state, Agent::wait);

    // Manually exit the inside agent so the semaphore becomes empty.
    sem.Exit(inside_agent);
    EXPECT_TRUE(sem.Empty());

    // Next step should release the waiting agent.
    sm.Step(0.1, agents);
    EXPECT_EQ(agents[waiting_agent].state, Agent::move);
    EXPECT_TRUE(sem.Inside(waiting_agent));
}

TEST(SemaphoreManagerStep, AgentInsideAllowsSameSideToJoin) {
    Graph g = MakeNarrowGraph();
    SemaphoreManager sm(AsShared(g));
    auto& sem = sm.semaphores[0];

    // Agent 0 is already inside from the left and is still traversing the
    // segment: its incoming_route[1] == seg[0] but incoming_route[2] is NOT
    // seg[1], so Intersection returns -1 (inside, moving towards left side)
    // which is < 0 → Exit is NOT called and agent 0 stays inside.
    Linestring agent0_route;
    agent0_route.push_back(Point{sem.segment[0].x() + 0.05, sem.segment[0].y()});
    agent0_route.push_back(sem.segment[0]);
    agent0_route.push_back(Point{sem.segment[0].x() - 0.5, sem.segment[0].y() + 1.0});
    sem.inner_left_q.insert(0);
    Agents agents(2);
    agents[0] = MakeAgentWithRoute(agent0_route);  // state = move

    // Agent 1 approaches from the left.
    agents[1] = MakeAgentWithRoute(LeftApproachRoute(sem));

    // Step 0: agent 0 is Inside, Intersection==-1 (<0) → stays inside.
    // Agent 1 is close enough → enqueued in left_q, state=wait.
    // Release: inner_left_q non-empty → left_q drained immediately →
    // agent 1 is set to move and added to inner_left_q in the same step.
    sm.Step(0.0, agents);
    EXPECT_EQ(agents[1].state, Agent::move);
    EXPECT_TRUE(sem.Inside(1));
    EXPECT_TRUE(sem.Inside(0));  // agent 0 still inside (not exited)
}
