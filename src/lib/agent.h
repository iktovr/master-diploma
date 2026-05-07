#pragma once

#include <cassert>
#include <vector>

#include "geometry.h"

struct RouteFollower {
    Linestring route;
    Linestring incoming_route;    
    double length;
    double x;
    Point pos;

    void SetRoute(const Linestring& new_route) {
        assert(!new_route.empty());
        route = new_route;
        incoming_route = route;
        length = bg::length(route);
        x = 0;
        pos = route.front();
    }

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

    Point pos;
    int base = 0;
    State state = idle;
    RouteFollower route_follower;

    Agent() = default;

    Agent(const double x, const double y, const int base = 0) : pos(x, y), base(base) {}

    void SetRoute(const Linestring& route) {
        route_follower.SetRoute(route);
        state = move;
    }

    inline const Linestring& Route() const {
        return route_follower.route;
    }

    inline const Linestring& IncomingRoute() const {
        return route_follower.incoming_route;
    }

    void Move(const double dt, const double speed);
};

using Agents = std::vector<Agent>;
