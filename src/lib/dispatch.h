#pragma once

#include <memory>
#include <vector>

#include "agent.h"
#include "graph.h"
#include "router.h"

struct Dispatch {
    std::shared_ptr<const Graph> graph;
    std::shared_ptr<const IRouter> router;

    std::vector<int> delivery_points;
    std::vector<int> base_points;
    std::vector<int> current_order;

    std::vector<double> route_length;
    std::vector<double> order_start_time;

    Dispatch(
        std::shared_ptr<const Graph> graph,
        std::shared_ptr<const IRouter> router,
        const Agents& agents)
        : graph(std::move(graph))
        , router(std::move(router))
        , delivery_points(this->graph->GetVertices(Graph::Vertex::delivery))
        , base_points(this->graph->GetVertices(Graph::Vertex::base))
        , current_order(agents.size(), -1)
        , route_length(agents.size(), -1)
        , order_start_time(agents.size(), -1)
    {
    }

    void AssignBasePoints(Agents& agents) const;

    void Step(double t, Agents& agents);

    int NewOrder() const;
};
