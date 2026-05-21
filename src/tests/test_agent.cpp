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

// ---------------------------------------------------------------------------
// Reverse state
// ---------------------------------------------------------------------------

TEST(AgentReverse, MovesBackwardAtReducedSpeed) {
    Agent a;
    a.SetRoute(HorizontalRoute(10.0));
    a.route_follower.Move(5.0);
    a.pos = a.route_follower.pos;
    a.state = Agent::reverse;

    // dt=1, speed=2 → forward dx would be 2; reverse uses 0.5 factor → 1.0
    a.Move(/*dt=*/1.0, /*speed=*/2.0);

    EXPECT_EQ(a.state, Agent::reverse);
    EXPECT_NEAR(a.route_follower.x, 5.0 - 1.0, 1e-6);
    EXPECT_NEAR(a.pos.x(), 4.0, 1e-6);
}

TEST(AgentReverse, DoesNotTransitionToIdleAtRouteStart) {
    Agent a;
    a.SetRoute(HorizontalRoute(10.0));
    a.route_follower.Move(0.5);
    a.pos = a.route_follower.pos;
    a.state = Agent::reverse;

    // Reverse beyond the start — x should clamp at 0 but state stays reverse.
    a.Move(/*dt=*/1.0, /*speed=*/10.0);

    EXPECT_EQ(a.state, Agent::reverse);
    EXPECT_NEAR(a.route_follower.x, 0.0, 1e-6);
}

TEST(AgentReverse, IgnoredWhenIdleOrWait) {
    Agent a;
    a.SetRoute(HorizontalRoute(10.0));
    a.route_follower.Move(5.0);
    a.pos = a.route_follower.pos;
    a.state = Agent::wait;

    a.Move(/*dt=*/1.0, /*speed=*/2.0);

    EXPECT_EQ(a.state, Agent::wait);
    EXPECT_NEAR(a.route_follower.x, 5.0, 1e-9);
}

TEST(AgentReverse, ForwardAfterReverseKeepsSingleEdgePassageInterval) {
    // Route with two segments: (0,0) → (4,0) → (10,0).
    // The agent crosses fully forward to vertex 202, reverses partially back
    // onto segment {200,201}, then continues forward to the end. The edge
    // {200,201} must yield only one recorded passage with t_enter being the
    // ORIGINAL first entry (segment start) and t_exit being the time of the
    // final forward crossing of the boundary at x = 4.
    Statistics::Get().edges.Clear();

    Agent a;
    a.SetRoute(TwoSegmentRoute(4.0, 6.0), {200, 201, 202}, /*t_now=*/0.0);

    // Forward step 1: cover the entire route at speed 10 over dt=1 → both
    // edge {200,201} and {201,202} are crossed forward.
    //   exit of {200,201} = 0 + 1*4/10 = 0.4
    //   exit of {201,202} = 1.0
    a.Move(/*t=*/1.0, /*dt=*/1.0, /*speed=*/10.0);
    EXPECT_EQ(a.state, Agent::idle);

    // Now place the agent back into reverse halfway across segment {201,202}.
    // First send it to reverse and back across the {200,201} boundary.
    a.state = Agent::reverse;
    // Reverse to x = 2 (well inside the first segment).
    // From x = 10, we need to go back 8 units. Use speed=10, dt=1, factor 0.75
    // → 7.5 units. Then another small reverse.
    a.Move(/*t=*/2.0, /*dt=*/1.0, /*speed=*/10.0);  // x: 10 → 2.5
    a.Move(/*t=*/3.0, /*dt=*/0.1, /*speed=*/10.0);  // x: 2.5 → 1.75

    // Now move forward again to the end at speed 10.
    a.state = Agent::move;
    // Need to cover 10 - 1.75 = 8.25. Use dt=1, speed=10 → 10 covered.
    a.Move(/*t=*/4.0, /*dt=*/1.0, /*speed=*/10.0);

    // Edge {200,201} must still have exactly one passage recorded with
    // t_enter at 0.0 (first forward entry) and t_exit at 0.4 (the original
    // first forward crossing). The back-and-forth must NOT create a second
    // record.
    const auto* w1 = Statistics::Get().edges.Window(200, 201);
    ASSERT_NE(w1, nullptr);
    EXPECT_EQ(w1->size(), 1u);
    EXPECT_NEAR(w1->front().t_exit, 0.4, 1e-6);
    EXPECT_NEAR(w1->front().speed, 10.0, 1e-6);

    // Same for edge {201,202}.
    const auto* w2 = Statistics::Get().edges.Window(201, 202);
    ASSERT_NE(w2, nullptr);
    EXPECT_EQ(w2->size(), 1u);
    EXPECT_NEAR(w2->front().t_exit, 1.0, 1e-6);
}

TEST(AgentReverse, ReverseRebuildsIncomingRoute) {
    Agent a;
    a.SetRoute(TwoSegmentRoute(4.0, 6.0));  // (0,0)->(4,0)->(10,0)

    // Move forward enough to consume the first vertex from incoming_route.
    a.route_follower.Move(5.0);  // now on segment 2
    ASSERT_EQ(a.route_follower.incoming_route.size(), 2u);

    // Reverse back across the boundary.
    a.state = Agent::reverse;
    a.route_follower.MoveBackward(2.0);  // x = 3, segment 1

    // incoming_route must again contain the (4,0) waypoint ahead.
    EXPECT_EQ(a.route_follower.incoming_route.size(), 3u);
}

// ---------------------------------------------------------------------------
// Scheduled-position cap (RouteFollower::segment_schedule_t)
// ---------------------------------------------------------------------------

TEST(RouteFollowerSchedule, NoScheduleMovesFreely) {
    RouteFollower rf;
    // Two-segment route (0,0)->(4,0)->(10,0), vertices id=1,2,3, no schedule.
    rf.SetRoute(TwoSegmentRoute(4.0, 6.0), {1, 2, 3}, 0.0);

    // Without a schedule the (dx, dt, t_now) overload behaves like legacy Move.
    rf.Move(/*dx=*/2.0, /*dt=*/1.0, /*t_now=*/1.0);
    EXPECT_NEAR(rf.x, 2.0, 1e-9);
}

TEST(RouteFollowerSchedule, CapsForwardProgressByScheduledPosition) {
    RouteFollower rf;
    // Single-segment route 0..10, scheduled at 1 m/s: arrival times 0 and 10.
    rf.SetRoute(HorizontalRoute(10.0), {1, 2},
                /*schedule=*/std::vector<double>{0.0, 10.0}, /*t_now=*/0.0);

    // Try to sprint 5 m in dt=1s — scheduled position at t=1 is 1.0.
    rf.Move(/*dx=*/5.0, /*dt=*/1.0, /*t_now=*/1.0);
    EXPECT_NEAR(rf.x, 1.0, 1e-9);

    // At t=2, scheduled position is 2.0; another sprint should reach 2.0.
    rf.Move(/*dx=*/5.0, /*dt=*/1.0, /*t_now=*/2.0);
    EXPECT_NEAR(rf.x, 2.0, 1e-9);
}

TEST(RouteFollowerSchedule, ExplicitWaitAtVertexHoldsPosition) {
    RouteFollower rf;
    // (0,0)->(5,0)->(10,0), same vertex id 2 duplicated to encode a wait at
    // x=5 from t=5 to t=8, then continue at 1 m/s to x=10 at t=13.
    Linestring route;
    route.push_back(Point{0.0, 0.0});
    route.push_back(Point{5.0, 0.0});   // arrive at vertex 2 at t=5
    route.push_back(Point{5.0, 0.0});   // leave vertex 2 at t=8 (wait)
    route.push_back(Point{10.0, 0.0});  // arrive at vertex 3 at t=13
    rf.SetRoute(route, /*vertex_ids=*/{1, 2, 2, 3},
                /*schedule=*/std::vector<double>{0.0, 5.0, 8.0, 13.0},
                /*t_now=*/0.0);

    // Step at 1 m/s up to t=5 — should reach x=5.
    rf.Move(/*dx=*/5.0, /*dt=*/5.0, /*t_now=*/5.0);
    EXPECT_NEAR(rf.x, 5.0, 1e-9);

    // During the wait window (t=5..8) x must stay at 5 even if we ask to move.
    rf.Move(/*dx=*/3.0, /*dt=*/2.0, /*t_now=*/7.0);
    EXPECT_NEAR(rf.x, 5.0, 1e-9);

    // After the wait, x advances on schedule.
    rf.Move(/*dx=*/3.0, /*dt=*/2.0, /*t_now=*/10.0);
    EXPECT_NEAR(rf.x, 7.0, 1e-9);

    // Final stretch reaches the end.
    rf.Move(/*dx=*/5.0, /*dt=*/3.0, /*t_now=*/13.0);
    EXPECT_NEAR(rf.x, 10.0, 1e-9);
    EXPECT_TRUE(rf.IsFinished());
}

TEST(RouteFollowerSchedule, ScheduledPositionMonotone) {
    RouteFollower rf;
    rf.SetRoute(HorizontalRoute(10.0), {1, 2},
                std::vector<double>{0.0, 10.0}, /*t_now=*/0.0);
    EXPECT_NEAR(rf.ScheduledPosition(-1.0), 0.0, 1e-9);
    EXPECT_NEAR(rf.ScheduledPosition(0.0), 0.0, 1e-9);
    EXPECT_NEAR(rf.ScheduledPosition(5.0), 5.0, 1e-9);
    EXPECT_NEAR(rf.ScheduledPosition(10.0), 10.0, 1e-9);
    EXPECT_NEAR(rf.ScheduledPosition(15.0), 10.0, 1e-9);
}
