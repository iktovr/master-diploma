#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "graph.h"
#include "geometry.h"
#include "statistics.h"

namespace lib {

class IRouter {
public:
    IRouter(std::shared_ptr<const Graph> graph) : graph_(std::move(graph)) {}
    virtual ~IRouter() = default;
    virtual Linestring GetRoute(const int u, const int v, const double t = 0.0,
                                const int agent_id = -1) const = 0;  // agent_id: -1=unknown, used by CCBS for peer replanning
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

    Linestring GetRoute(const int u, const int v, const double t = 0.0,
                        const int agent_id = -1) const override {
        EnsureCacheFresh(t);
        return AStarRouter::GetRoute(u, v, t, agent_id);
    }

    std::pair<Linestring, std::vector<int>> GetRouteWithVertices(
        const int u, const int v, const double t = 0.0,
        const int agent_id = -1) const override {
        EnsureCacheFresh(t);
        return AStarRouter::GetRouteWithVertices(u, v, t, agent_id);
    }

protected:
    double Heuristic(const int u, const int v) const override {
        return graph_->Distance(u, v) / max_speed_;
    }

    double Cost(const int u, const int v, const Graph::Edge& edge,
                const double t) const override {
        const double s = LookupEdgeSpeed(u, v, t);
        return edge.length / (s > 0.0 ? s : max_speed_);
    }

private:
    static std::int64_t EdgeKey(int u, int v) {
        const int a = u < v ? u : v;
        const int b = u < v ? v : u;
        return (static_cast<std::int64_t>(static_cast<std::uint32_t>(a)) << 32)
             |  static_cast<std::int64_t>(static_cast<std::uint32_t>(b));
    }

    void EnsureCacheFresh(double t) const {
        const std::uint64_t v = stats_ ? stats_->Version() : 0;
        if (!cache_valid_ || cached_t_ != t || cached_version_ != v) {
            edge_speed_cache_.clear();
            cached_t_ = t;
            cached_version_ = v;
            cache_valid_ = true;
        }
    }

    double LookupEdgeSpeed(int u, int v, double t) const {
        if (stats_ == nullptr) {
            return 0.0;
        }
        const std::int64_t k = EdgeKey(u, v);
        auto it = edge_speed_cache_.find(k);
        if (it != edge_speed_cache_.end()) {
            return it->second;
        }
        double s = 0.0;
        if (stats_->Has(u, v)) {
            const double avg = stats_->AverageSpeed(u, v, t);
            if (avg > 0.0) {
                s = avg;
            }
        }
        edge_speed_cache_.emplace(k, s);
        return s;
    }

    GraphEdgeStatistics* stats_;
    double max_speed_;

    mutable std::unordered_map<std::int64_t, double> edge_speed_cache_;
    mutable double cached_t_ = 0.0;
    mutable std::uint64_t cached_version_ = 0;
    mutable bool cache_valid_ = false;
};

}
