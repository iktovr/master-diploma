#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "agent.h"
#include "geometry.h"
#include "graph.h"
#include "router.h"

// Forward declarations so we don't pull <third_party/ccbs/*> headers
// into every translation unit that includes ccbs_router.h. The CCBS
// internals are reached through the ccbs_adapter::Solver bridge
// (lib/ccbs_solver.h), which avoids leaking conflicting global
// names (`Agent`, `Point`) into our codebase.
namespace ccbs_adapter {
class Solver;
struct Stamp;
using Path = std::vector<Stamp>;
}

// CcbsRouter is an IRouter adapter on top of the third-party CCBS
// (Continuous-time Conflict-Based Search) library. Unlike the
// A*-family routers, it is a *batch* multi-agent planner: a single
// call to GetRouteWithVertices() replans not only the requesting
// agent but every other agent currently in |move| or |wait| state,
// so that all paths in the resulting schedule are jointly conflict
// free on narrow edges.
//
// Each replanned agent receives a per-vertex schedule via
// |Agent::SetRoute(route, vertex_ids, segment_schedule_t, t_now)|;
// the schedule encodes both motion times and explicit waits at
// vertices, so the simulation does not need a separate Resolver
// while this router is active.
//
// Resolvers are *not* compatible with this router (CCBS already
// resolves narrow-edge conflicts itself). Use ResolverKind::none.
class CcbsRouter : public IRouter {
public:
    CcbsRouter(std::shared_ptr<const Graph> graph,
               Agents* agents,
               double max_speed);
    ~CcbsRouter() override;

    // The router needs a pointer to the live |Agents| vector so it
    // can replan peers during a batch solve. Because |Agents| is a
    // member of |Simulation| (and is moved-from in its constructor),
    // the router is typically constructed *before* the simulation
    // with |agents=nullptr| and then back-patched here.
    void SetAgents(Agents* agents) { agents_ = agents; }

    // Bound the wall-clock budget of one batch CCBS solve. When the
    // budget is exhausted, the joint planner reports failure and we
    // transparently fall back to single-agent A*. Mostly useful for
    // tests that want to assert the fallback path is taken on
    // pathological inputs (e.g. dense head-on encounters). The
    // default (set in the constructor) is generous; pass a small
    // value (e.g. 0.05 s) to force fallback quickly.
    void SetSolverTimeLimit(double seconds);

    Linestring GetRoute(const int u, const int v, const double t = 0.0,
                        const int agent_id = -1) const override;
    std::pair<Linestring, std::vector<int>> GetRouteWithVertices(
        const int u, const int v, const double t = 0.0,
        const int agent_id = -1) const override;

protected:
    // Single-agent heuristic / edge cost (used only when |agents_| is
    // null or as fallback). Distance / max_speed (units: seconds).
    double Heuristic(const int u, const int v) const override;
    double Cost(const int u, const int v, const Graph::Edge& edge,
                const double t) const override;

private:
    // Lazily build the CCBS Map mirroring |graph_| and mark every
    // narrow edge via Map::set_narrow_edge. Idempotent.
    void EnsureMapBuilt() const;

    // Convert a CCBS path (sequence of {id, g} stamps) into our
    // representation. Consecutive stamps that share |id| become two
    // consecutive entries with identical vertex/position and
    // strictly-increasing schedule time, which RouteFollower
    // interprets as an explicit wait at that vertex.
    void PathToRoute(const ccbs_adapter::Path& path,
                     double t_start,
                     Linestring* out_route,
                     std::vector<int>* out_vertex_ids,
                     std::vector<double>* out_schedule_t) const;

    // Translate one of our Agent's current state into a (start, goal)
    // sub-task for CCBS. Returns false when the agent is idle (and
    // therefore should be excluded from the batch).
    // |start_id_out| receives the *next* graph vertex the agent will
    // arrive at (so we never replan a partially-traversed edge).
    bool BuildSubtask(const Agent& a,
                      int* start_id_out,
                      int* goal_id_out) const;

    Agents* agents_;
    double max_speed_;

    // Owned CCBS bridge, mutable because GetRoute* is logically
    // const yet we lazily build the underlying roadmap.
    mutable std::mutex mu_;
    std::unique_ptr<ccbs_adapter::Solver> solver_;
    mutable bool map_built_ = false;

    // Single-agent fallback used when CCBS fails to find a joint
    // solution (e.g. an over-constrained batch). Plain A* on edge
    // length, ignoring narrow conflicts entirely.
    AStarRouter fallback_router_;
};
