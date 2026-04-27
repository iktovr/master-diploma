#include "agent.h"
#include "simulation.h"

#include <iostream>

int main() {
    Simulation sim;

    for (int i = 0; i < 10; ++i) {
        sim.AddAgent(Agent());
    }

    int steps;
    std::cin >> steps;
    while (steps--) {
        sim.Step(0.2);
        for (const auto& agent: sim.agents) {
            std::cout << agent.x << ' ' << agent.y << '\n';
        }
        std::cout << '\n' << std::endl;
    }
}