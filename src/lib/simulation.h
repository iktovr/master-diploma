#pragma once

#include "agent.h"

#include <vector>

struct Simulation {
    std::vector<Agent> agents;

    void AddAgent(const Agent& agent) {
        agents.push_back(agent);
    }

    void Step(const double dt);
};