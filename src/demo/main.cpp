#include "lib/agent.h"
#include "lib/graph.h"
#include "lib/simulation.h"
#include "lib/visualizer.h"

#include <iostream>
#include <format>
#include <random>

int main() {
    Simulation sim;
    Visualizer vis(500, 500, 8, 8, "/workspaces/master_diploma/src/demo/out");
    Graph g;
    g.AddVertex(-1, 0);
    g.AddVertex(0, 1);
    g.AddVertex(1, 1);
    g.AddVertex(2, 0);
    g.AddVertex(1, -1);
    g.AddVertex(0, -1);
    g.AddVertex(3, 1);
    g.AddVertex(2, 1);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);
    g.AddEdge(0, 3);
    g.AddEdge(0, 5);
    g.AddEdge(4, 3);
    g.AddEdge(4, 5);
    g.AddEdge(3, 6);
    g.AddEdge(6, 7);

    static std::mt19937 gen;
    static std::uniform_int_distribution<int> dist(0, g.vertices.size() - 1);

    const int agents_count = 10;
    std::vector<int> last_vertex(agents_count);
    for (int i = 0; i < agents_count; ++i) {
        sim.AddAgent(Agent());
        last_vertex[i] = dist(gen);
    }

    int steps = 50;
    while (steps--) {
        for (int i = 0; i < agents_count; ++i) {
            auto& agent = sim.agents[i];
            if (agent.state == Agent::idle) {
                int u = last_vertex[i], v = dist(gen);
                while (v == u) {
                    v = dist(gen);
                }
                agent.SetRoute(g.GetRoute(u, v));
            }
        }

        sim.Step(0.1);

        vis.ClearFrame();
        vis.DrawGraph(g);
        for (const auto& agent: sim.agents) {
            vis.DrawAgent(agent);
        }
        vis.SaveFrame();
    }
}
