#include "agent.h"

#include <cmath>

#include "geometry.h"

Point RouteFollower::Move(const double dx) {
    x += dx;
    if (x > length) {
        x = length;
    }
    bg::line_interpolate(route, x, pos);
    return pos;
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