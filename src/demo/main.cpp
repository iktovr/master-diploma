#include "lib/agent.h"
#include "lib/graph.h"
#include "lib/simulation.h"
#include "lib/visualizer.h"

int main() {
    Visualizer vis(500, 500, 8, 8, "/workspaces/master_diploma/src/demo/out");
    Graph g;
    g.AddVertex(-1, 0, Graph::Vertex::base);
    g.AddVertex(0, 1, Graph::Vertex::delivery);
    g.AddVertex(1, 1, Graph::Vertex::delivery);
    g.AddVertex(2, 0, Graph::Vertex::delivery);
    g.AddVertex(1, -1, Graph::Vertex::delivery);
    g.AddVertex(0, -1, Graph::Vertex::delivery);
    g.AddVertex(3, 1, Graph::Vertex::delivery);
    g.AddVertex(2, 1, Graph::Vertex::delivery);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 2);
    g.AddEdge(0, 3);
    g.AddEdge(0, 5);
    g.AddEdge(4, 3);
    g.AddEdge(4, 5);
    g.AddEdge(3, 6);
    g.AddEdge(6, 7);

    Agents agents(10, Agent(-1, 0, 0));
    Simulation sim(std::move(agents), std::move(g));

    int steps = 50;
    while (steps--) {
        sim.Step(0.1);

        vis.ClearFrame();
        vis.DrawGraph(sim.graph);
        for (const auto& agent: sim.agents) {
            vis.DrawAgent(agent);
        }
        vis.SaveFrame();
    }
}
