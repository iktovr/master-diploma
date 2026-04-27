#include "simulation.h"

void Simulation::Step(double dt) {
    for (auto& agent: agents) {
        agent.Move(dt, 2.2);
    }
}