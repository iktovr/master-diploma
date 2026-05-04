#include "agent.h"

#include <random>
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
    // static std::mt19937 gen;
    // static std::uniform_real_distribution<double> dist(0, 2 * std::acos(-1));

    // const double dir = dist(gen);
    // pos += Point{speed * dt * std::cos(dir), speed * dt * std::sin(dir)};

    pos = route_follower.Move(speed * dt);
    if (route_follower.IsFinished()) {
        state = idle;
    }
}