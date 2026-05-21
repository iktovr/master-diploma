#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "graph.h"
#include "geometry.h"
#include "statistics.h"

class IRouter {
public:
    IRouter(std::shared_ptr<const Graph> graph) : graph_(std::move(graph)) {}
    virtual ~IRouter() = default;
    // agent_id is the index of the agent for whom the route is being
    // requested. It is ignored by single-agent routers (A*, Stat) and
    // used by batch multi-agent routers (CCBS) to identify the caller
    // and re-plan its peers concurrently. Default -1 == "unknown".
    virtual Linestring GetRoute(const int u, const int v, const double t = 0.0,
                                const int agent_id = -1) const = 0;
    virtual std::pair<Linestring, std::vector<int>> GetRouteWithVertices(
        const int u, const int v, const double t = 0.0, const int agent_id = -1) const = 0;

protected:
    virtual double Heuristic(const int u, const int v) const = 0;
    virtual double Cost(const int u, const int v, const Graph::Edge& edge,
                        const double t) const = 0;

    std::shared_ptr<const Graph> graph_;
};

class AStarRouter : public IRouter {
public:
    AStarRouter(std::shared_ptr<const Graph> graph) : IRouter(std::move(graph)) {}
    virtual ~AStarRouter() = default;
    Linestring GetRoute(const int u, const int v, const double t = 0.0,
                        const int agent_id = -1) const override;
    std::pair<Linestring, std::vector<int>> GetRouteWithVertices(
        const int u, const int v, const double t = 0.0, const int agent_id = -1) const override;

protected:
    using SearchState = std::pair<double, int>;

    struct VertexData {
        double dist;
        double cost;
        int prev;
    };

    std::vector<int> Search(const int u, const int v, const double t) const;

    double Heuristic(const int u, const int v) const override {
        return graph_->Distance(u, v);
    }

    double Cost(const int, const int, const Graph::Edge& edge,
                const double /*t*/) const override {
        return edge.length;
    }
};

class StatAStarRouter : public AStarRouter {
public:
    StatAStarRouter(std::shared_ptr<const Graph> graph,
                    GraphEdgeStatistics* stats,
                    double max_speed)
        : AStarRouter(std::move(graph))
        , stats_(stats)
        , max_speed_(max_speed) {}

    ~StatAStarRouter() override = default;

protected:
    double Heuristic(const int u, const int v) const override {
        return graph_->Distance(u, v) / max_speed_;
    }

    double Cost(const int u, const int v, const Graph::Edge& edge,
                const double t) const override {
        double s = max_speed_;
        if (stats_ != nullptr && stats_->Has(u, v)) {
            const double avg = stats_->AverageSpeed(u, v, t);
            if (avg > 0.0) {
                s = avg;
            }
        }
        return edge.length / s;
    }

private:
    GraphEdgeStatistics* stats_;
    double max_speed_;
};
