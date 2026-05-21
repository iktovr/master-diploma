#pragma once

#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "geometry.h"

inline constexpr double kAgentFollowGap = 1.0;

inline constexpr int kNarrowEdgeCapacity = 1;
inline constexpr int kWideEdgeCapacity = 3;

inline constexpr double kReverseSpeedFactor = 0.5;
inline constexpr double kNarrowEdgeSpeedFactor = 0.75;

struct RouteFollower {
    Linestring route;
    Linestring incoming_route;
    double length = 0.0;
    double x = 0.0;
    Point pos{0.0, 0.0};

    std::vector<int> vertex_ids;
    std::vector<double> cumulative_len;
    std::size_t segment_idx = 0;
    double segment_t_enter = 0.0;

    // Per-segment first-forward-entry times. Persisted across reverse
    // excursions so that a back-and-forth motion across a narrow edge
    // contributes a single (t_enter, t_exit) interval to edge statistics.
    std::vector<double> segment_entry_time;
    std::size_t max_segment_reached = 0;

    // Optional absolute simulation time at which the agent is scheduled
    // to arrive at vertex_ids[i] (and equivalently at cumulative_len[i]).
    // Must be monotonically non-decreasing. Pairs of equal vertex_ids
    // with strictly increasing schedule entries encode an explicit
    // wait at that vertex.
    //
    // When non-empty, forward Move() caps |x| so that the agent never
    // overruns its scheduled position at the current simulation time,
    // which produces natural pauses (e.g. waits dictated by CCBS).
    // Empty vector means "no schedule" and preserves legacy behavior
    // for all non-CCBS routers and tests.
    std::vector<double> segment_schedule_t;

    void SetRoute(const Linestring& new_route);
    void SetRoute(const Linestring& new_route,
                  const std::vector<int>& new_vertex_ids,
                  double t_now);
    void SetRoute(const Linestring& new_route,
                  const std::vector<int>& new_vertex_ids,
                  const std::vector<double>& new_segment_schedule_t,
                  double t_now);

    // Returns the scheduled x along the route at absolute simulation
    // time t_now, derived from |segment_schedule_t|. Returns |length|
    // when there is no schedule or t_now is past the schedule end.
    double ScheduledPosition(double t_now) const;

    bool IsFinished() const {
        return std::abs(length - x) < 1e-3;
    }

    std::optional<std::pair<int, int>> CurrentEdge() const;

    double DistanceAlongEdge() const;

    // Length of the edge that the agent currently occupies (in segment_idx).
    double CurrentEdgeLength() const;

    Point Move(double dx);
    Point Move(double dx, double dt, double t_now);
    Point Move(double dx, double dt, double t_now, double max_dx);

    // Move backward along the route by |dx|. Decreases x and never records
    // edge-passage statistics. Segment entry times are preserved so that a
    // future forward re-crossing into a segment we already visited records a
    // single combined passage from the original entry to the final exit.
    Point MoveBackward(double dx);
    Point MoveBackward(double dx, double dt, double t_now, double max_dx);

private:
    void RebuildIncomingRoute();
};

struct Agent {
    enum State {
        idle,
        move,
        wait,
        reverse
    };

    Point pos{0.0, 0.0};
    int base = 0;
    State state = idle;
    RouteFollower route_follower;

    Agent() = default;

    Agent(const double x, const double y, const int base = 0) : pos(x, y), base(base) {}

    void SetRoute(const Linestring& route);
    void SetRoute(const Linestring& route,
                  const std::vector<int>& vertex_ids,
                  double t_now);
    void SetRoute(const Linestring& route,
                  const std::vector<int>& vertex_ids,
                  const std::vector<double>& segment_schedule_t,
                  double t_now);

    inline const Linestring& Route() const {
        return route_follower.route;
    }

    inline const Linestring& IncomingRoute() const {
        return route_follower.incoming_route;
    }

    void Move(double dt, double speed);
    void Move(double t, double dt, double speed);
    void Move(double t, double dt, double speed, double max_dx);
};

using Agents = std::vector<Agent>;
