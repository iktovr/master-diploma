#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "graph.h"
#include "geometry.h"

class IRouter {
public:
    IRouter(std::shared_ptr<const Graph> graph) : graph_(std::move(graph)) {}
    virtual ~IRouter() = default;
    virtual Linestring GetRoute(const int u, const int v) const = 0;

protected:
    virtual double Heuristic(const int u, const int v) const = 0;
    virtual double Cost(const int u, const int v, const Graph::Edge& edge) const = 0;

    std::shared_ptr<const Graph> graph_;
};

class AStarRouter : public IRouter {
public:
    AStarRouter(std::shared_ptr<const Graph> graph) : IRouter(std::move(graph)) {}
    virtual ~AStarRouter() = default;
    Linestring GetRoute(const int u, const int v) const override;

protected:
    using SearchState = std::pair<double, int>;

    struct VertexData {
        double dist;
        double cost;
        int prev;
    };

    std::vector<int> Search(const int u, const int v) const;

    double Heuristic(const int u, const int v) const override {
        return graph_->Distance(u, v);
    }

    double Cost(const int, const int, const Graph::Edge& edge) const override {
        return edge.length;
    }
};