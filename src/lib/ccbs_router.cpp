#include "ccbs_router.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "agent.h"
#include "ccbs_solver.h"
#include "geometry.h"
#include "graph.h"
#include "logging.h"

CcbsRouter::CcbsRouter(std::shared_ptr<const Graph> graph,
                       Agents* agents,
                       double max_speed)
    : IRouter(graph)
    , agents_(agents)
    , max_speed_(max_speed > 0.0 ? max_speed : 1.0)
    , solver_(new ccbs_adapter::Solver())
    , fallback_router_(std::move(graph)) {
    // CCBS is exponential in the number of conflicts in pathological
    // dense head-on scenarios. Cap its wall-clock budget so that
    // Solve() returns false (triggering the A* fallback) instead of
    // blocking the caller.
    solver_->SetTimeLimit(5.0);
}

CcbsRouter::~CcbsRouter() = default;

void CcbsRouter::SetSolverTimeLimit(double seconds) {
    solver_->SetTimeLimit(seconds);
}

double CcbsRouter::Heuristic(const int u, const int v) const {
    return graph_->Distance(u, v) / max_speed_;
}

double CcbsRouter::Cost(const int, const int, const Graph::Edge& edge,
                        const double /*t*/) const {
    return edge.length / max_speed_;
}

void CcbsRouter::EnsureMapBuilt() const {
    if (map_built_) return;

    const std::size_t n = graph_->vertices.size();
    std::vector<std::pair<double, double>> node_pos;
    node_pos.reserve(n);
    for (const auto& v : graph_->vertices) {
        node_pos.push_back({v.pos.x(), v.pos.y()});
    }

    std::vector<std::vector<int>> adj(n);
    std::vector<std::pair<int, int>> narrow_edges;
    for (std::size_t u = 0; u < n; ++u) {
        adj[u].reserve(graph_->edges[u].size());
        for (const auto& kv : graph_->edges[u]) {
            const int v = kv.first;
            adj[u].push_back(v);
            // Record narrow undirected pairs once (when u < v) to
            // avoid duplicates; Map::set_narrow_edge is itself
            // direction-agnostic so duplicates are harmless.
            if (kv.second.narrow && static_cast<int>(u) < v) {
                narrow_edges.push_back({static_cast<int>(u), v});
            }
        }
    }

    solver_->BuildMap(node_pos, adj, narrow_edges, kNarrowEdgeSpeedFactor);
    map_built_ = true;
}

bool CcbsRouter::BuildSubtask(const Agent& a,
                              int* start_id_out,
                              int* goal_id_out) const {
    if (a.state == Agent::idle) return false;
    const auto& rf = a.route_follower;
    if (rf.vertex_ids.empty()) return false;
    if (rf.cumulative_len.empty()) return false;

    std::size_t next_idx = rf.segment_idx;
    if (rf.x > rf.cumulative_len[rf.segment_idx]) {
        next_idx = std::min(rf.segment_idx + 1, rf.vertex_ids.size() - 1);
    }
    *start_id_out = rf.vertex_ids[next_idx];
    *goal_id_out  = rf.vertex_ids.back();
    return *start_id_out != *goal_id_out;
}

void CcbsRouter::PathToRoute(const ccbs_adapter::Path& path,
                             double t_start,
                             Linestring* out_route,
                             std::vector<int>* out_vertex_ids,
                             std::vector<double>* out_schedule_t) const {
    out_route->clear();
    out_vertex_ids->clear();
    out_schedule_t->clear();
    if (path.empty()) return;

    // CCBS emits two consecutive sNodes with the same |id| but
    // different |g| to encode a wait. We preserve that one-to-one
    // structure: each Stamp becomes one entry in (route, vertex_ids,
    // schedule_t). RouteFollower::ScheduledPosition() interpolates
    // between identical vertices with a zero-length cumulative_len
    // delta and pins the agent at that vertex until the later
    // schedule time.
    const double g0 = path.front().g;
    for (const auto& sn : path) {
        if (sn.id < 0 || sn.id >= static_cast<int>(graph_->vertices.size())) {
            out_route->clear();
            out_vertex_ids->clear();
            out_schedule_t->clear();
            return;
        }
        out_vertex_ids->push_back(sn.id);
        out_route->push_back(graph_->vertices[sn.id].pos);
        // |g| is the CCBS time-cost. Wide-edge traversal contributes
        // |distance| (unit speed), narrow-edge traversal contributes
        // |distance|/factor (slower). Dividing by max_speed_ scales
        // unit-speed time into our absolute simulation seconds; on
        // narrow edges this naturally yields max_speed_ * factor.
        out_schedule_t->push_back(t_start + (sn.g - g0) / max_speed_);
    }
}

Linestring CcbsRouter::GetRoute(const int u, const int v, const double t,
                                const int agent_id) const {
    auto pr = GetRouteWithVertices(u, v, t, agent_id);
    return pr.first;
}

std::pair<Linestring, std::vector<int>> CcbsRouter::GetRouteWithVertices(
    const int u, const int v, const double t, const int agent_id) const {
    std::lock_guard<std::mutex> lock(mu_);
    EnsureMapBuilt();

    // Build the batch task: caller first, then every other non-idle
    // agent with a still-valid trip. The caller's sub-task index in
    // the result vector is 0.
    std::vector<std::pair<int, int>> subtasks;
    subtasks.push_back({u, v});
    const int caller_task_idx = 0;

    struct PeerEntry {
        int sim_agent_idx;
        int task_idx;
    };
    std::vector<PeerEntry> peers;
    if (agents_ != nullptr && agent_id >= 0) {
        for (std::size_t i = 0; i < agents_->size(); ++i) {
            if (static_cast<int>(i) == agent_id) continue;
            const Agent& a = (*agents_)[i];
            int s = -1, g = -1;
            if (!BuildSubtask(a, &s, &g)) continue;
            peers.push_back({static_cast<int>(i),
                             static_cast<int>(subtasks.size())});
            subtasks.push_back({s, g});
        }
    }

    std::vector<ccbs_adapter::Path> paths;
    const bool ok = solver_->Solve(subtasks, &paths);

    if (!ok) {
        // CCBS failed to find a joint conflict-free solution. Fall
        // back to plain single-agent A* so the dispatcher still gets
        // a usable route. No schedule is produced in this path, so
        // narrow-edge conflicts may briefly occur until the next
        // successful CCBS replan.
        LOG_WARNING("CCBS router failed to find joint conflict-free solution for agent {} ({} -> {}). Falling back to single-agent A* routing.",
                    agent_id, u, v);
        return fallback_router_.GetRouteWithVertices(u, v, t, agent_id);
    }

    Linestring caller_route;
    std::vector<int> caller_vids;
    std::vector<double> caller_sched;
    PathToRoute(paths[caller_task_idx], t,
                &caller_route, &caller_vids, &caller_sched);
    if (agents_ != nullptr && agent_id >= 0
        && agent_id < static_cast<int>(agents_->size())) {
        (*agents_)[agent_id].SetRoute(
            caller_route, caller_vids, caller_sched, t);
    }

    for (const auto& p : peers) {
        if (p.task_idx >= static_cast<int>(paths.size())) continue;
        Linestring route;
        std::vector<int> vids;
        std::vector<double> sched;
        PathToRoute(paths[p.task_idx], t, &route, &vids, &sched);
        if (vids.size() < 2) continue;
        (*agents_)[p.sim_agent_idx].SetRoute(route, vids, sched, t);
    }

    return {caller_route, caller_vids};
}
