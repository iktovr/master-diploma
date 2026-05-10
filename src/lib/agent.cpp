#include "agent.h"

#include <cassert>
#include <cmath>

#include "geometry.h"

void RouteFollower::SetRoute(const Linestring& new_route) {
    assert(!new_route.empty());
    route = new_route;
    incoming_route = route;
    length = bg::length(route);
    x = 0;
    pos = route.front();
}

Point RouteFollower::Move(const double dx) {
    x += dx;
    if (x > length) {
        x = length;
    }
    if (incoming_route.size() > 2 && bg::distance(incoming_route[0], incoming_route[1]) < dx) {
        incoming_route.erase(incoming_route.begin());
    }
    bg::line_interpolate(route, x, pos);
    incoming_route[0] = pos;
    return pos;
}

void Agent::SetRoute(const Linestring& route) {
    route_follower.SetRoute(route);
    state = move;
}

void Agent::Move(const double dt, const double speed) {
    if (state != move) {
        return;
    }

    pos = route_follower.Move(speed * dt);
    if (route_follower.IsFinished()) {
        state = idle;
    }
}
