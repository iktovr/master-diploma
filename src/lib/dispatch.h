#pragma once

#include <vector>

#include "agent.h"
#include "graph.h"

struct Dispatch {
    std::vector<int> delivery_points;
    std::vector<int> current_order;
    const Graph& graph;

    Dispatch(const Graph& graph, const Agents& agents) : 
        delivery_points(graph.GetVertices(Graph::Vertex::delivery)), current_order(agents.size(), -1), graph(graph) {}

    void Step(Agents& agents);

    int NewOrder() const;
};
