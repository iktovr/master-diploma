#include "lib/agent.h"
#include "lib/simulation.h"
#include "lib/visualizer.h"

#include <iostream>
#include <format>

int main() {
    Simulation sim;
    Visualizer vis(500, 500, 10, 10, "/home/iktovr/master-diploma/src/demo/out");

    for (int i = 0; i < 10; ++i) {
        sim.AddAgent(Agent());
    }

    int steps = 50;
    for (int i = 0; i < steps; ++i) {
        sim.Step(0.2);

        vis.ClearFrame();
        for (const auto& agent: sim.agents) {
            vis.DrawAgent(agent);
        }
        vis.SaveFrame(i);
    }
}
