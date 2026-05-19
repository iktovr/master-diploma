#pragma once

#include <cassert>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "geometry.h"

inline constexpr double kAgentFollowGap = 1.0;

inline constexpr int kNarrowEdgeCapacity = 1;
inline constexpr int kWideEdgeCapacity = 3;

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

    void SetRoute(const Linestring& new_route);
    void SetRoute(const Linestring& new_route,
                  const std::vector<int>& new_vertex_ids,
                  double t_now);

    bool IsFinished() {
        return std::abs(length - x) < 1e-3;
    }

    std::optional<std::pair<int, int>> CurrentEdge() const;

    double DistanceAlongEdge() const;

    Point Move(double dx);
    Point Move(double dx, double dt, double t_now);
    Point Move(double dx, double dt, double t_now, double max_dx);
};

struct Agent {
    enum State {
        idle,
        move,
        wait
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
