#include "agent.h"
#include "simulation.h"
#include "visualizer.h"

#include <iostream>

int main() {
    Simulation sim;
    Visualizer vis(200, 200, 10, 10, "/home/iktovr/master-diploma/src/test");

    for (int i = 0; i < 10; ++i) {
        sim.AddAgent(Agent());
    }

    int steps = 10;
    for (int i = 0; i < steps; ++i) {
        sim.Step(0.2);

        vis.ClearFrame();
        for (const auto& agent: sim.agents) {
            vis.DrawAgent(agent);
        }
        vis.SaveFrame(i);
    }
}