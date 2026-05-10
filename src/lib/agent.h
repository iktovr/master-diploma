#pragma once

#include <cassert>
#include <vector>

#include "geometry.h"

struct RouteFollower {
    Linestring route;
    Linestring incoming_route;
    double length = 0.0;
    double x = 0.0;
    Point pos{0.0, 0.0};

    void SetRoute(const Linestring& new_route);

    bool IsFinished() {
        return std::abs(length - x) < 1e-3;
    }

    Point Move(const double dx);
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

    inline const Linestring& Route() const {
        return route_follower.route;
    }

    inline const Linestring& IncomingRoute() const {
        return route_follower.incoming_route;
    }

    void Move(const double dt, const double speed);
};

using Agents = std::vector<Agent>;
