#pragma once

#include "agent.h"
#include "dispatch.h"
#include "graph.h"
#include "visualizer.h"

#include <optional>
#include <vector>

struct Simulation {
    Agents agents;
    Graph graph;
    Dispatch dispatch;
    std::optional<Visualizer> vis;

    Simulation(const Agents& agents_, const Graph& graph_, const std::optional<Visualizer> vis_ = std::nullopt) :
        agents(agents_), graph(graph_), dispatch(graph, agents), vis(vis_) {
        if (vis) {
            vis->DrawGraph(graph);
            vis->SavePersistentPart();
        }
    }

    void AddAgent(const Agent& agent) {
        agents.push_back(agent);
    }

    template <class... Args>
    void AddAgent(Args... args) {
        agents.emplace_back(args...);
    }

    void Simulate(const double duration, const double step, const double vis_step = -1);
    void Visualize();
    void Step(const double dt);
};
