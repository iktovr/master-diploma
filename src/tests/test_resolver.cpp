#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "lib/agent.h"
#include "lib/geometry.h"
#include "lib/graph.h"
#include "lib/resolver.h"
#include "lib/statistics.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Graph with a single narrow edge u=0 <-> v=1 of length 10.
static std::shared_ptr<const Graph> MakeNarrowEdgeGraph(double length = 10.0) {
    Graph g;
    g.AddVertex(0.0, 0.0);
    g.AddVertex(length, 0.0);
    g.AddEdge(0, 1, /*narrow=*/true);
    return std::make_shared<const Graph>(g);
}

// Graph with one wide edge.
static std::shared_ptr<const Graph> MakeWideEdgeGraph(double length = 10.0) {
    Graph g;
    g.AddVertex(0.0, 0.0);
    g.AddVertex(length, 0.0);
    g.AddEdge(0, 1, /*narrow=*/false);
    return std::make_shared<const Graph>(g);
}

static Linestring StraightRoute(double from_x, double to_x) {
    Linestring r;
    r.push_back(Point{from_x, 0.0});
    r.push_back(Point{to_x, 0.0});
    return r;
}

// Helper to position an agent on a directed edge u→v at distance `x` from u.
static void PlaceOnEdge(Agent& a, double from_x, double to_x,
                        const std::vector<int>& vids, double x_on_edge) {
    a.SetRoute(StraightRoute(from_x, to_x), vids, /*t_now=*/0.0);
    a.route_follower.Move(x_on_edge);
    a.pos = a.route_follower.pos;
    a.state = Agent::move;
}

static void ResetStats() {
    auto& s = Statistics::Get();
    s.conflicts_count = 0;
    // CumulativeStatistic has no Clear() — re-create by trick: only the
    // sum/size matters for Empty(), so leave as-is. Tests should check
    // increments via differences when needed.
}

// ---------------------------------------------------------------------------
// Head-on conflict detection
// ---------------------------------------------------------------------------

TEST(ResolverHeadOn, TwoAgentsOnNarrowEdgeOneReverses) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(2);
    ResetStats();
    const int before = Statistics::Get().conflicts_count;

    // Agent 0: forward 0→1, at x=4.5 from u.
    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    // Agent 1: forward 1→0, at x=4.5 from v (physical 5.5 from u).
    // Gap = 10 - 4.5 - 4.5 = 1.0 — equal to kAgentFollowGap; choose slightly
    // smaller to trigger conflict.
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);

    r.Step(0.0, 0.1, agents);

    EXPECT_EQ(Statistics::Get().conflicts_count, before + 1);
    EXPECT_EQ(r.conflicts.size(), 1u);

    // The loser is the agent with smaller id (=0).
    EXPECT_EQ(agents[0].state, Agent::reverse);
    EXPECT_EQ(agents[1].state, Agent::move);

    const auto& c = r.conflicts[0];
    EXPECT_TRUE(c.reversing.contains(0));
    EXPECT_EQ(c.pushers.size(), 1u);
    EXPECT_EQ(c.pushers[0], 1);
    EXPECT_TRUE(c.waiting_for.contains(1));
}

TEST(ResolverHeadOn, NoConflictWhenAgentsAreFarApart) {
    auto g = MakeNarrowEdgeGraph(20.0);
    Resolver r(g);
    Agents agents(2);
    ResetStats();
    const int before = Statistics::Get().conflicts_count;

    PlaceOnEdge(agents[0], 0.0, 20.0, {0, 1}, 2.0);
    PlaceOnEdge(agents[1], 20.0, 0.0, {1, 0}, 2.0);
    // Physical gap = 20 - 2 - 2 = 16, well above kAgentFollowGap

    r.Step(0.0, 0.1, agents);

    EXPECT_EQ(Statistics::Get().conflicts_count, before);
    EXPECT_EQ(r.conflicts.size(), 0u);
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_EQ(agents[1].state, Agent::move);
}

TEST(ResolverHeadOn, WideEdgeDoesNotTriggerConflict) {
    auto g = MakeWideEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(2);
    ResetStats();
    const int before = Statistics::Get().conflicts_count;

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);

    r.Step(0.0, 0.1, agents);

    EXPECT_EQ(Statistics::Get().conflicts_count, before);
    EXPECT_EQ(r.conflicts.size(), 0u);
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_EQ(agents[1].state, Agent::move);
}

// ---------------------------------------------------------------------------
// Reverse -> wait -> move transitions
// ---------------------------------------------------------------------------

TEST(ResolverFlow, LoserBecomesWaitAfterExitingEdge) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(2);

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);  // forward at 4.5
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);  // forward at 4.6

    // Detect conflict.
    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);

    // Move the loser backward off the edge manually (simulating ticks).
    // Loser is agent 0 on directed edge (0,1), x=4.5 from u=0.
    // It needs to reach x <= 0 to leave the edge. Force x to 0.
    while (agents[0].route_follower.x > 0.0) {
        agents[0].Move(0.1, 5.0);  // reverse at 0.75*5 = 3.75 per step
    }
    EXPECT_EQ(agents[0].state, Agent::reverse);

    // Next resolver step should detect loser left edge → wait.
    r.Step(0.1, 0.1, agents);
    EXPECT_EQ(agents[0].state, Agent::wait);
}

TEST(ResolverFlow, WaitingAgentReleasedOnlyAfterAllPushersExit) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(2);

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);

    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);

    // Send loser fully back.
    while (agents[0].route_follower.x > 0.0) {
        agents[0].Move(0.1, 5.0);
    }
    r.Step(0.1, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::wait);

    // Pusher still on the edge — loser must remain waiting.
    ASSERT_EQ(agents[1].state, Agent::move);
    ASSERT_TRUE(agents[1].route_follower.CurrentEdge().has_value());

    r.Step(0.2, 0.1, agents);
    EXPECT_EQ(agents[0].state, Agent::wait);

    // Now move pusher all the way across the edge.
    while (agents[1].route_follower.x < agents[1].route_follower.length) {
        agents[1].Move(0.1, 5.0);
    }
    // Pusher's CurrentEdge() now returns nullopt (finished).
    EXPECT_EQ(agents[1].state, Agent::idle);

    r.Step(0.3, 0.1, agents);
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_EQ(r.conflicts.size(), 0u);
}

// ---------------------------------------------------------------------------
// Cascade onto same-direction trailers
// ---------------------------------------------------------------------------

TEST(ResolverCascade, TrailerBehindReversingAgentAlsoReverses) {
    auto g = MakeNarrowEdgeGraph(20.0);
    Resolver r(g);
    Agents agents(3);
    ResetStats();
    const int before = Statistics::Get().conflicts_count;

    // Agent 1: forward 0→1, deep in the edge at x=9.5 (will be loser).
    // Agent 2: forward 1→0, at x=10.0 from v (physical 10.0 from u).
    //   physical_gap = 20 - 9.5 - 10.0 = 0.5 < 1.0 → conflict.
    // Agent 0: forward 0→1, trailing agent 1 at x=8.7 (gap 0.8 < 1.0).
    PlaceOnEdge(agents[0], 0.0, 20.0, {0, 1}, 8.7);
    PlaceOnEdge(agents[1], 0.0, 20.0, {0, 1}, 9.5);
    PlaceOnEdge(agents[2], 20.0, 0.0, {1, 0}, 10.0);

    r.Step(0.0, 0.1, agents);

    // Agent 0 is the loser (smaller id wins for "smaller id loses" rule
    // because we chose smaller id as loser).
    // Actually our rule: loser = smaller id. So between agents 0 and 2,
    // agent 0 loses on first head-on. But agent 0 is at x=8.7 forward
    // (going 0→1) and agent 2 at 1→0 — these two see physical gap:
    //   20 - 8.7 - 10.0 = 1.3 (above kAgentFollowGap=1.0) → NOT a conflict.
    // Conflict is detected between agent 1 (x=9.5) and agent 2 (x=10.0):
    //   20 - 9.5 - 10.0 = 0.5 < 1.0. Smaller id of {1,2} = 1 → loser.
    EXPECT_EQ(agents[1].state, Agent::reverse);
    EXPECT_EQ(agents[2].state, Agent::move);
    EXPECT_EQ(agents[0].state, Agent::move);
    EXPECT_EQ(Statistics::Get().conflicts_count, before + 1);

    // Now cascade: agent 0 is in same direction as agent 1 (0→1) and the
    // gap between them is 9.5 - 8.7 = 0.8 < kAgentFollowGap. The cascade
    // path will flip agent 0 to reverse.
    r.Step(0.1, 0.1, agents);

    EXPECT_EQ(agents[0].state, Agent::reverse);
    EXPECT_EQ(Statistics::Get().conflicts_count, before + 2);

    const auto& c = r.conflicts[0];
    EXPECT_TRUE(c.reversing.contains(0) || c.waiting_losers.contains(0));
    EXPECT_TRUE(c.reversing.contains(1) || c.waiting_losers.contains(1));
    // pushers should remain just {2}; cascade does NOT add to pushers.
    EXPECT_EQ(c.pushers.size(), 1u);
    EXPECT_EQ(c.pushers[0], 2);
}

// ---------------------------------------------------------------------------
// Newcomers are ignored while a conflict is active
// ---------------------------------------------------------------------------

TEST(ResolverNewcomer, NewAgentEnteringDuringConflictDoesNotExtendDependency) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(3);

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);
    // Agent 2 starts off any edge (idle).
    agents[2].state = Agent::idle;

    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);
    const std::size_t pushers_before = r.conflicts[0].pushers.size();
    const std::size_t waiting_before = r.conflicts[0].waiting_for.size();

    // Newcomer agent 2 hops onto the edge going forward 0→1 far behind.
    PlaceOnEdge(agents[2], 0.0, 10.0, {0, 1}, 0.5);
    r.Step(0.1, 0.1, agents);

    // pushers/waiting_for must NOT include agent 2.
    EXPECT_EQ(r.conflicts[0].pushers.size(), pushers_before);
    EXPECT_EQ(r.conflicts[0].waiting_for.size(), waiting_before);
    EXPECT_FALSE(r.conflicts[0].waiting_for.contains(2));
}

// A newcomer that enters the conflict edge in the pusher direction and lands
// within kAgentFollowGap of any pusher still on the edge gets absorbed into
// the pushers set. The waiting losers must then also wait for it.
TEST(ResolverNewcomer, WinnerSideNewcomerJoinsPushers) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(3);
    ResetStats();
    const int before = Statistics::Get().conflicts_count;

    // Agent 0 (loser, forward 0->1 at x=4.5) and agent 1 (pusher, forward
    // 1->0 at x=4.6).
    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);
    agents[2].state = Agent::idle;

    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);
    ASSERT_EQ(r.conflicts[0].pushers.size(), 1u);
    ASSERT_EQ(r.conflicts[0].pushers[0], 1);
    EXPECT_EQ(Statistics::Get().conflicts_count, before + 1);

    // Agent 2 enters in the pusher's direction (1->0), behind agent 1.
    // Pusher 1 has DistanceAlongEdge=4.6, newcomer at 4.0 -> gap 0.6 < 1.0.
    PlaceOnEdge(agents[2], 10.0, 0.0, {1, 0}, 4.0);
    r.Step(0.1, 0.1, agents);

    EXPECT_TRUE(r.conflicts[0].waiting_for.contains(2));
    EXPECT_NE(std::find(r.conflicts[0].pushers.begin(),
                        r.conflicts[0].pushers.end(), 2),
              r.conflicts[0].pushers.end());
    EXPECT_EQ(r.conflicts[0].pushers.size(), 2u);
    EXPECT_EQ(Statistics::Get().conflicts_count, before + 2);
}

// A newcomer in the pusher's direction but too far away from every existing
// pusher (and reverser) does NOT get absorbed into the conflict.
TEST(ResolverNewcomer, WinnerSideNewcomerFarAwayIsIgnored) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(3);
    ResetStats();
    const int before = Statistics::Get().conflicts_count;

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);
    agents[2].state = Agent::idle;

    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);
    EXPECT_EQ(Statistics::Get().conflicts_count, before + 1);

    // Agent 2 enters in pusher direction at x=0.5. Pusher 1 at x=4.6 ->
    // gap 4.1 > 1.0. Reverser 0 in pusher frame at 10-4.5=5.5 -> gap 5.0.
    // Both too far. Should NOT be absorbed.
    PlaceOnEdge(agents[2], 10.0, 0.0, {1, 0}, 0.5);
    r.Step(0.1, 0.1, agents);

    EXPECT_FALSE(r.conflicts[0].waiting_for.contains(2));
    EXPECT_EQ(r.conflicts[0].pushers.size(), 1u);
    EXPECT_EQ(Statistics::Get().conflicts_count, before + 1);
}

// ---------------------------------------------------------------------------
// Fresh conflict after a previous one is resolved
// ---------------------------------------------------------------------------

TEST(ResolverLifecycle, FreshConflictAfterReleaseCreatesNewDependencyList) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(2);

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);

    // 1st conflict.
    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);

    // Drive loser fully backward.
    while (agents[0].route_follower.x > 0.0) {
        agents[0].Move(0.1, 5.0);
    }
    r.Step(0.1, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::wait);

    // Drive pusher off the edge.
    while (agents[1].route_follower.x < agents[1].route_follower.length) {
        agents[1].Move(0.1, 5.0);
    }
    r.Step(0.2, 0.1, agents);
    ASSERT_EQ(r.conflicts.size(), 0u);
    ASSERT_EQ(agents[0].state, Agent::move);

    // Now reset the agents for a fresh head-on, with roles reversed.
    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);
    r.Step(0.3, 0.1, agents);
    EXPECT_EQ(r.conflicts.size(), 1u);
    EXPECT_EQ(r.conflicts[0].pushers.size(), 1u);
    EXPECT_EQ(r.conflicts[0].pushers[0], 1);
}

// ---------------------------------------------------------------------------
// Statistics: conflicts_count and reverse_time accumulation
// ---------------------------------------------------------------------------

TEST(ResolverStats, ReverseTimeAccumulatedPerTickPerAgent) {
    auto g = MakeNarrowEdgeGraph(10.0);
    Resolver r(g);
    Agents agents(2);
    ResetStats();
    const std::size_t before_size = Statistics::Get().reverse_time.Values().size();

    PlaceOnEdge(agents[0], 0.0, 10.0, {0, 1}, 4.5);
    PlaceOnEdge(agents[1], 10.0, 0.0, {1, 0}, 4.6);

    // Step 1: detect conflict (no reverse_time added on the conflict tick
    // — at the start of the tick agent 0 is still move).
    r.Step(0.0, 0.1, agents);
    ASSERT_EQ(agents[0].state, Agent::reverse);

    // Step 2: agent 0 is in reverse → +0.1 reverse_time.
    r.Step(0.1, 0.1, agents);
    const std::size_t after_size = Statistics::Get().reverse_time.Values().size();
    EXPECT_GT(after_size, before_size);
    // The newly added value should be 0.1 (one agent in reverse).
    EXPECT_NEAR(Statistics::Get().reverse_time.Values().back(), 0.1, 1e-9);
}
