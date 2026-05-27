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

// StatAStarRouter
// ---------------
// Same A* as AStarRouter but with edge-traversal cost driven by recent
// passage observations (GraphEdgeStatistics). Calling AverageSpeed on every
// edge expansion is expensive (it performs a hashmap lookup, evicts stale
// samples, and reduces a window with one std::exp per sample plus the
// Bayesian prior), and within a single simulation tick:
//
//   * t is constant across all router calls issued by Dispatch::Step
//     (multiple agents replan at the same simulated time);
//   * GraphEdgeStatistics is not mutated between those calls (Record only
//     fires from agent Move(), after dispatch has finished).
//
// Therefore AverageSpeed(u, v, t) is a pure function of (u, v) for the
// duration of one tick. We memoize it in edge_speed_cache_, keyed by the
// undirected edge key. The cache is invalidated whenever the caller-supplied
// t changes or the underlying GraphEdgeStatistics is mutated, detected via
// GraphEdgeStatistics::Version().
//
// The cache stores the effective traversal speed (>= 0). A stored value of
// 0.0 means "no estimate available, fall back to max_speed_" and folds the
// Has() check into the same lookup so the Cost hot path is a single
// hashmap probe.
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

    // Invalidate the per-tick edge-speed cache if either the time argument
    // or the statistics version has changed since the last router call.
    // Called at every public entry point.
    void EnsureCacheFresh(double t) const {
        const std::uint64_t v = stats_ ? stats_->Version() : 0;
        if (!cache_valid_ || cached_t_ != t || cached_version_ != v) {
            edge_speed_cache_.clear();
            cached_t_ = t;
            cached_version_ = v;
            cache_valid_ = true;
        }
    }

    // Per-tick memoized AverageSpeed. Returns the effective speed for edge
    // (u,v); a return value of 0.0 means "no estimate, caller should use
    // max_speed_".
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
