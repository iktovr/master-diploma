#include <gtest/gtest.h>

#include <cmath>

#include "lib/agent.h"
#include "lib/geometry.h"
#include "lib/statistics.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Build a simple two-point linestring: (0,0) → (length, 0)
static Linestring HorizontalRoute(double length) {
    Linestring r;
    r.push_back(Point{0.0, 0.0});
    r.push_back(Point{length, 0.0});
    return r;
}

// Build a three-point linestring: (0,0) → (a,0) → (a+b, 0)
static Linestring TwoSegmentRoute(double a, double b) {
    Linestring r;
    r.push_back(Point{0.0, 0.0});
    r.push_back(Point{a, 0.0});
    r.push_back(Point{a + b, 0.0});
    return r;
}

// ---------------------------------------------------------------------------
// RouteFollower::Move
// ---------------------------------------------------------------------------

TEST(RouteFollowerMove, AdvancesPosition) {
    RouteFollower rf;
    rf.SetRoute(HorizontalRoute(10.0));

    rf.Move(3.0);

    EXPECT_NEAR(rf.pos.x(), 3.0, 1e-9);
    EXPECT_NEAR(rf.pos.y(), 0.0, 1e-9);
    EXPECT_NEAR(rf.x, 3.0, 1e-9);
    EXPECT_FALSE(rf.IsFinished());
}

TEST(RouteFollowerMove, ClampsAtEnd) {
    RouteFollower rf;
    rf.SetRoute(HorizontalRoute(5.0));

    // Move well past the end
    rf.Move(100.0);

    EXPECT_NEAR(rf.x, 5.0, 1e-9);
    EXPECT_NEAR(rf.pos.x(), 5.0, 1e-9);
    EXPECT_TRUE(rf.IsFinished());
}

TEST(RouteFollowerMove, ExactlyReachesEnd) {
    RouteFollower rf;
    rf.SetRoute(HorizontalRoute(4.0));

    rf.Move(4.0);

    EXPECT_NEAR(rf.x, 4.0, 1e-9);
    EXPECT_TRUE(rf.IsFinished());
}

TEST(RouteFollowerMove, IncomingRouteTrimmedWhenFirstSegmentConsumed) {
    // Route: (0,0) → (1,0) → (3,0)
    // First sub-segment length = 1.0
    // Move dx = 1.5 > 1.0 → incoming_route should lose its front point
    RouteFollower rf;
    rf.SetRoute(TwoSegmentRoute(1.0, 2.0));

    ASSERT_EQ(rf.incoming_route.size(), 3u);

    rf.Move(1.5);

    // After consuming the first segment the front is trimmed
    EXPECT_EQ(rf.incoming_route.size(), 2u);
}

TEST(RouteFollowerMove, IncomingRouteNotTrimmedForSmallMove) {
    // Move less than the first sub-segment length → no trim
    RouteFollower rf;
    rf.SetRoute(TwoSegmentRoute(2.0, 2.0));

    ASSERT_EQ(rf.incoming_route.size(), 3u);

    rf.Move(0.5);

    EXPECT_EQ(rf.incoming_route.size(), 3u);
}

TEST(RouteFollowerMove, MultiSegmentInterpolation) {
    // Route: (0,0) → (2,0) → (4,0), total length = 4
    // Move 3.0 → should land at (3, 0)
    RouteFollower rf;
    rf.SetRoute(TwoSegmentRoute(2.0, 2.0));

    rf.Move(3.0);

    EXPECT_NEAR(rf.pos.x(), 3.0, 1e-6);
    EXPECT_NEAR(rf.pos.y(), 0.0, 1e-6);
    EXPECT_FALSE(rf.IsFinished());
}

TEST(RouteFollowerMove, IncomingRouteFirstPointUpdatedToCurrentPos) {
    RouteFollower rf;
    rf.SetRoute(HorizontalRoute(10.0));

    rf.Move(4.0);

    // incoming_route[0] must always reflect the current position
    EXPECT_NEAR(rf.incoming_route[0].x(), rf.pos.x(), 1e-9);
    EXPECT_NEAR(rf.incoming_route[0].y(), rf.pos.y(), 1e-9);
}

// ---------------------------------------------------------------------------
// Agent::Move
// ---------------------------------------------------------------------------

TEST(AgentMove, SkipsWhenIdle) {
    Agent a;
    a.state = Agent::idle;
    a.pos = Point{1.0, 2.0};

    a.Move(1.0, 5.0);

    EXPECT_EQ(a.state, Agent::idle);
    EXPECT_NEAR(a.pos.x(), 1.0, 1e-9);
    EXPECT_NEAR(a.pos.y(), 2.0, 1e-9);
}

TEST(AgentMove, SkipsWhenWaiting) {
    Agent a;
    a.SetRoute(HorizontalRoute(10.0));
    a.state = Agent::wait;
    a.pos = Point{0.0, 0.0};

    a.Move(1.0, 5.0);

    EXPECT_EQ(a.state, Agent::wait);
    EXPECT_NEAR(a.pos.x(), 0.0, 1e-9);
}

TEST(AgentMove, MovesAndStaysMoveState) {
    Agent a;
    a.SetRoute(HorizontalRoute(100.0));

    // dt=1, speed=3 → dx=3
    a.Move(1.0, 3.0);

    EXPECT_EQ(a.state, Agent::move);
    EXPECT_NEAR(a.pos.x(), 3.0, 1e-6);
}

TEST(AgentMove, TransitionsToIdleWhenRouteFinished) {
    Agent a;
    a.SetRoute(HorizontalRoute(5.0));

    // dt=1, speed=10 → dx=10, overshoots the 5-unit route
    a.Move(1.0, 10.0);

    EXPECT_EQ(a.state, Agent::idle);
}

TEST(AgentMove, SpeedScalesDx) {
    Agent a1, a2;
    a1.SetRoute(HorizontalRoute(100.0));
    a2.SetRoute(HorizontalRoute(100.0));

    a1.Move(1.0, 2.0);  // dx = 2
    a2.Move(2.0, 1.0);  // dx = 2

    EXPECT_NEAR(a1.pos.x(), a2.pos.x(), 1e-9);
}

// ---------------------------------------------------------------------------
// Edge-passage statistics recorded via the time-aware Move overload
// ---------------------------------------------------------------------------

TEST(AgentMoveStats, RecordsPassageAcrossSingleEdge) {
    Statistics::Get().edges.Clear();

    // Two-vertex route of length 10 over edge {100, 101}.
    Agent a;
    a.SetRoute(HorizontalRoute(10.0), {100, 101}, /*t_now=*/0.0);

    // Two steps of dt=1 at speed=5 → covers 10 units in 2 seconds.
    a.Move(/*t=*/1.0, /*dt=*/1.0, /*speed=*/5.0);
    a.Move(/*t=*/2.0, /*dt=*/1.0, /*speed=*/5.0);

    EXPECT_EQ(a.state, Agent::idle);
    ASSERT_TRUE(Statistics::Get().edges.Has(100, 101));
    EXPECT_NEAR(Statistics::Get().edges.AverageSpeed(100, 101, 2.0), 5.0, 1e-9);

    const auto* window = Statistics::Get().edges.Window(100, 101);
    ASSERT_NE(window, nullptr);
    EXPECT_EQ(window->size(), 1u);
    EXPECT_NEAR(window->front().t_exit, 2.0, 1e-9);
}

TEST(AgentMoveStats, RecordsPassagePerSegmentWithInterpolatedExit) {
    Statistics::Get().edges.Clear();

    // (0,0)→(4,0)→(10,0): two edges {200,201} of length 4 and {201,202} of length 6.
    Agent a;
    a.SetRoute(TwoSegmentRoute(4.0, 6.0), {200, 201, 202}, /*t_now=*/0.0);

    // Single step covering the entire route at constant speed 10 over dt=1.
    // The exit time of the first segment should be linearly interpolated at
    // t = 0 + 1 * (4 / 10) = 0.4.
    a.Move(/*t=*/1.0, /*dt=*/1.0, /*speed=*/10.0);

    ASSERT_TRUE(Statistics::Get().edges.Has(200, 201));
    ASSERT_TRUE(Statistics::Get().edges.Has(201, 202));

    const auto* w1 = Statistics::Get().edges.Window(200, 201);
    ASSERT_NE(w1, nullptr);
    ASSERT_EQ(w1->size(), 1u);
    EXPECT_NEAR(w1->front().t_exit, 0.4, 1e-9);
    EXPECT_NEAR(w1->front().speed, 10.0, 1e-9);

    const auto* w2 = Statistics::Get().edges.Window(201, 202);
    ASSERT_NE(w2, nullptr);
    ASSERT_EQ(w2->size(), 1u);
    EXPECT_NEAR(w2->front().t_exit, 1.0, 1e-9);
    EXPECT_NEAR(w2->front().speed, 10.0, 1e-9);
}

TEST(AgentMoveStats, LegacySetRouteDoesNotRecordPassages) {
    Statistics::Get().edges.Clear();

    Agent a;
    a.SetRoute(HorizontalRoute(10.0));  // no vertex ids

    a.Move(/*t=*/1.0, /*dt=*/1.0, /*speed=*/10.0);

    // No edge has been recorded since vertex_ids was never set.
    const auto* w = Statistics::Get().edges.Window(0, 1);
    EXPECT_TRUE(w == nullptr || w->empty());
}
