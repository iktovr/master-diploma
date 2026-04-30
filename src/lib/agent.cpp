#include "agent.h"

#include <random>
#include <cmath>

void Agent::Move(double dt, double speed) {
    static std::mt19937 gen;
    static std::uniform_real_distribution<double> dist(0, 2 * std::acos(-1));

    const double dir = dist(gen);
    x += speed * dt * std::cos(dir);
    y += speed * dt * std::sin(dir);
}