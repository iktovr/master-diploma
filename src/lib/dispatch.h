#pragma once

#include <memory>
#include <vector>

#include "agent.h"
#include "graph.h"
#include "router.h"

namespace lib {

struct Dispatch {
    std::shared_ptr<const Graph> graph;
    std::shared_ptr<const IRouter> router;

    std::vector<int> delivery_points;
    std::vector<int> base_points;
    std::vector<int> current_order;

    std::vector<double> traveled_length;
    std::vector<Point>  last_pos;
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
        , traveled_length(agents.size(), 0.0)
        , last_pos(agents.size(), Point{0.0, 0.0})
        , order_start_time(agents.size(), -1)
    {
    }

    void AssignBasePoints(Agents& agents) const;

    void Step(double t, Agents& agents);

    int NewOrder() const;
};

}
