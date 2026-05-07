#pragma once

#include <vector>

#include "agent.h"
#include "graph.h"

struct Dispatch {
    const Graph& graph;
    std::vector<int> delivery_points;
    std::vector<int> base_points;
    std::vector<int> current_order;

    Dispatch(const Graph& graph, const Agents& agents) : 
        graph(graph),
        delivery_points(graph.GetVertices(Graph::Vertex::delivery)),
        base_points(graph.GetVertices(Graph::Vertex::base)),
        current_order(agents.size(), -1)
    {
    }

    void AssignBasePoints(Agents& agents) const;

    void Step(Agents& agents);

    int NewOrder() const;
};
