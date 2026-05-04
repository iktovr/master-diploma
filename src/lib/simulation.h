#pragma once

#include "agent.h"
#include "dispatch.h"
#include "graph.h"

#include <vector>

struct Simulation {
    Agents agents;
    Graph graph;
    Dispatch dispatch;

    Simulation(const Agents& agents_, const Graph& graph_) : agents(agents_), graph(graph_), dispatch(graph, agents) {}

    void AddAgent(const Agent& agent) {
        agents.push_back(agent);
    }

    template <class... Args>
    void AddAgent(Args... args) {
        agents.emplace_back(args...);
    }

    void Step(const double dt);
};