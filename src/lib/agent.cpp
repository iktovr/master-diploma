#include "agent.h"

#include <random>
#include <cmath>

#include "geometry.h"

void Agent::Move(double dt, double speed) {
    static std::mt19937 gen;
    static std::uniform_real_distribution<double> dist(0, 2 * std::acos(-1));

    const double dir = dist(gen);
    pos += Point{speed * dt * std::cos(dir), speed * dt * std::sin(dir)};
}