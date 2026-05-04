#pragma once

#include <cassert>

#include "geometry.h"

struct RouteFollower {
    Linestring route;
    double length;
    double x;
    Point pos;

    void SetRoute(const Linestring& new_route) {
        assert(!new_route.empty());
        route = new_route;
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
    State state = idle;
    RouteFollower route_follower;

    Agent() = default;

    Agent(const double x, const double y) : pos(x, y) {}

    void SetRoute(const Linestring& route) {
        route_follower.SetRoute(route);
        state = move;
    }

    void Move(const double dt, const double speed);
};