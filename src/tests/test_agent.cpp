#include <gtest/gtest.h>

#include <cmath>

#include "lib/agent.h"
#include "lib/geometry.h"

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
