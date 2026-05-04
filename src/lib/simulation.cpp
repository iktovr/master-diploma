#include "simulation.h"

void Simulation::Step(const double dt) {
    dispatch.Step(agents);
    for (auto& agent: agents) {
        agent.Move(dt, 2.2);
    }
}