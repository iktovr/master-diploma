#include "agent.h"

#include <cmath>

#include "geometry.h"

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

void Agent::Move(const double dt, const double speed) {
    if (state != move) {
        return;
    }

    pos = route_follower.Move(speed * dt);
    if (route_follower.IsFinished()) {
        state = idle;
    }
}
