#include "ccbs_router.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <unordered_set>
#include <utility>
#include <vector>

#include "agent.h"
#include "ccbs_solver.h"
#include "geometry.h"
#include "graph.h"
#include "logging.h"
#include "statistics.h"


namespace lib {

CcbsRouter::CcbsRouter(std::shared_ptr<const Graph> graph,
                       Agents* agents,
                       double max_speed)
    : IRouter(graph)
    , agents_(agents)
    , max_speed_(max_speed > 0.0 ? max_speed : 1.0)
    , solver_(new ccbs_adapter::Solver())
    , fallback_router_(std::move(graph)) {
    // Two-tier solver budget. The slow timelimit is what
    // SetSolverTimeLimit() controls (tests and external knobs). The
    // fast timelimit is used for the first attempt and falls back to
    // the slow budget only if the first attempt times out (vs. proves
    // infeasible).
    solver_->SetTimeLimit(slow_timelimit_s_);
}

CcbsRouter::~CcbsRouter() = default;

void CcbsRouter::SetSolverTimeLimit(double seconds) {
    if (seconds > 0.0) {
        slow_timelimit_s_ = seconds;
    }
    // Keep the fast budget bounded by the slow one. A test that sets
    // a very small slow budget (e.g. 0.05 s to force fallback) should
    // not have an even smaller fast budget below it.
    if (fast_timelimit_s_ > slow_timelimit_s_) {
        fast_timelimit_s_ = slow_timelimit_s_;
    }
    solver_->SetTimeLimit(slow_timelimit_s_);
}

void CcbsRouter::SetFastSolverTimeLimit(double seconds) {
    if (seconds > 0.0) {
        fast_timelimit_s_ = std::min(seconds, slow_timelimit_s_);
    }
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

bool CcbsRouter::BuildPeerPlan(const Agent& peer, double t_now,
                               ccbs_adapter::PeerPlan* out) const {
    if (peer.state == Agent::idle) return false;
    const auto& rf = peer.route_follower;
    if (rf.vertex_ids.size() < 2) return false;

    // Determine the peer's next graph vertex index (same logic as
    // BuildSubtask) and the absolute wall-clock time at which the
    // peer reaches it. We express the peer plan in CCBS time-cost
    // units (g) measured from t_now, multiplied by max_speed_ so
    // that one CCBS second equals one second of absolute time.
    //
    // The plan we emit covers all vertices from the peer's *next*
    // vertex onward, *plus* — when the peer is currently mid-edge —
    // an extra leading stamp at the previous graph vertex with a
    // negative |g|. That extra stamp makes the in-flight narrow
    // traversal visible to the constraint-synthesis loop in
    // Solver::SolveSingleAgent(), which would otherwise let the
    // caller plan a head-on reverse move through the same narrow
    // edge while the peer is still inside it.
    std::size_t next_idx = rf.segment_idx;
    const bool peer_mid_edge =
        !rf.cumulative_len.empty()
        && rf.x > rf.cumulative_len[rf.segment_idx]
        && rf.segment_idx + 1 < rf.vertex_ids.size();
    if (peer_mid_edge) {
        next_idx = rf.segment_idx + 1;
    }
    if (next_idx >= rf.vertex_ids.size()) return false;

    // Compute the in-flight edge's effective traversal speed (factoring
    // in narrow slowdown) — needed both for the prepended-stamp |g|
    // and for the |has_sched|-less synthesis below.
    double dt_finish = 0.0;      // wall-time to reach |next_idx|
    double dt_already = 0.0;     // wall-time already spent on current edge
    if (peer_mid_edge) {
        const double remaining =
            rf.CurrentEdgeLength() - rf.DistanceAlongEdge();
        const double already = rf.DistanceAlongEdge();
        const auto cur_edge = rf.CurrentEdge();
        bool is_narrow = false;
        if (cur_edge.has_value()) {
            const int eu = cur_edge->first;
            const int ev = cur_edge->second;
            if (eu >= 0 && eu < static_cast<int>(graph_->edges.size())) {
                auto it = graph_->edges[eu].find(ev);
                if (it != graph_->edges[eu].end()) {
                    is_narrow = it->second.narrow;
                }
            }
        }
        const double effective_speed =
            max_speed_ * (is_narrow ? kNarrowEdgeSpeedFactor : 1.0);
        if (effective_speed > 0.0) {
            if (remaining > 0.0) dt_finish  = remaining / effective_speed;
            if (already  > 0.0) dt_already = already  / effective_speed;
        }
    }

    out->path.clear();
    out->path.reserve(rf.vertex_ids.size() - next_idx + (peer_mid_edge ? 1 : 0));

    // Prepend the in-flight edge's *origin* vertex with a negative |g|
    // so the constraint loop in Solver::SolveSingleAgent() emits a
    // head-on reverse-edge constraint covering the window during which
    // the peer is still inside the edge. We use the origin vertex's
    // id from |segment_idx| (the agent has moved past it but is still
    // between it and |segment_idx + 1|).
    if (peer_mid_edge) {
        ccbs_adapter::Stamp st;
        st.id = rf.vertex_ids[rf.segment_idx];
        st.g  = -dt_already * max_speed_;  // negative => already in the past
        out->path.push_back(st);
    }

    // Schedule-based time anchoring (preferred): if the peer has a
    // segment_schedule_t, use it verbatim.
    const bool has_sched =
        rf.segment_schedule_t.size() == rf.vertex_ids.size();

    if (has_sched) {
        for (std::size_t i = next_idx; i < rf.vertex_ids.size(); ++i) {
            ccbs_adapter::Stamp st;
            st.id = rf.vertex_ids[i];
            // Convert absolute time to CCBS g. We do *not* clamp to 0
            // here: the schedule may legitimately be slightly in the
            // past relative to t_now (e.g. when invoked between ticks),
            // and clamping would compress consecutive stamps to the
            // same g, breaking constraint synthesis. The constraint
            // loop already handles negative-g stamps correctly.
            const double dt = rf.segment_schedule_t[i] - t_now;
            st.g = dt * max_speed_;
            out->path.push_back(st);
        }
    } else {
        // Fallback: synthesize a schedule from cumulative_len at the
        // peer's effective speed (taking narrow edges into account).
        double g_acc = dt_finish * max_speed_;
        for (std::size_t i = next_idx; i < rf.vertex_ids.size(); ++i) {
            ccbs_adapter::Stamp st;
            st.id = rf.vertex_ids[i];
            st.g  = g_acc;
            out->path.push_back(st);

            // Advance g by the duration of the *next* edge if any.
            if (i + 1 < rf.vertex_ids.size()) {
                const int eu = rf.vertex_ids[i];
                const int ev = rf.vertex_ids[i + 1];
                if (eu >= 0 && eu < static_cast<int>(graph_->edges.size())) {
                    auto it = graph_->edges[eu].find(ev);
                    if (it != graph_->edges[eu].end()) {
                        const double len = it->second.length;
                        const bool narrow = it->second.narrow;
                        const double v_eff = max_speed_ *
                            (narrow ? kNarrowEdgeSpeedFactor : 1.0);
                        if (v_eff > 0.0) {
                            // g advances in unit-speed time; one CCBS
                            // second = 1/max_speed_ wall seconds, so
                            // edge-time-cost = len / (v_eff/max_speed_)
                            //                = len*max_speed_/v_eff.
                            // For wide edges that simplifies to len.
                            // For narrow edges it inflates by
                            // 1/kNarrowEdgeSpeedFactor — matching the
                            // CCBS Map::edge_time_cost convention.
                            g_acc += len * max_speed_ / v_eff;
                        }
                    }
                }
            }
        }
    }

    return out->path.size() >= 2;
}

bool CcbsRouter::PeerRelevant(const std::vector<int>& caller_vids,
                              const std::vector<int>& peer_vids) const {
    if (caller_vids.empty() || peer_vids.empty()) return false;

    // Cheap shared-vertex test. Two routes that don't touch any
    // common graph vertex cannot generate edge or wait conflicts
    // under CCBS's narrow-edge gate (head-on conflicts require both
    // endpoints to be common, and our narrow detection is per
    // vertex). The test is conservative: we include the peer if
    // there's any vertex overlap at all.
    std::unordered_set<int> caller_set(caller_vids.begin(),
                                       caller_vids.end());
    for (int v : peer_vids) {
        if (caller_set.count(v)) return true;
    }
    return false;
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

    // ------------------------------------------------------------------
    // Tier 1 — single-agent SIPP fast path
    // ------------------------------------------------------------------
    // The dispatcher invokes the router whenever a single agent becomes
    // idle and needs a new order. In the overwhelming majority of those
    // events the *other* agents already have valid, conflict-free CCBS
    // schedules from earlier replans. Re-running a full multi-agent
    // CCBS from scratch is wasteful in that regime: we can instead
    // route the caller alone against the peers' frozen schedules using
    // SIPP with synthesized constraints.
    //
    // We only attempt this fast path when |agents_| is connected (so
    // we have access to peer schedules) and the caller is identified
    // (agent_id >= 0). It is also skipped on degenerate (u == v)
    // sub-tasks because the upstream CCBS path treats those as no-ops.
    const auto t_solve_start = std::chrono::steady_clock::now();
    const bool can_use_peers =
        agents_ != nullptr && agent_id >= 0
        && agent_id < static_cast<int>(agents_->size());

    std::vector<ccbs_adapter::PeerPlan> peer_plans;
    // Side table preserved for later joint-CCBS escalation: each entry
    // mirrors a peer that BuildPeerPlan() accepted, in the same order.
    struct PeerRef {
        int  sim_agent_idx;
        std::vector<int> peer_vids;
    };
    std::vector<PeerRef> peer_refs;

    if (can_use_peers && u != v) {
        peer_plans.reserve(agents_->size());
        peer_refs.reserve(agents_->size());
        for (std::size_t i = 0; i < agents_->size(); ++i) {
            if (static_cast<int>(i) == agent_id) continue;
            const Agent& a = (*agents_)[i];
            ccbs_adapter::PeerPlan pp;
            if (!BuildPeerPlan(a, t, &pp)) continue;
            // Extract the peer's vertex sequence (we built it just
            // now) for the spatial-relevance filter and for later
            // joint-CCBS escalation.
            std::vector<int> peer_vids;
            peer_vids.reserve(pp.path.size());
            for (const auto& st : pp.path) peer_vids.push_back(st.id);

            peer_plans.push_back(std::move(pp));
            peer_refs.push_back({static_cast<int>(i), std::move(peer_vids)});
        }

        // -----------------------------------------------------------
        // Tier 1a — try the caller-only SIPP plan against all peer
        // plans verbatim. SolveSingleAgent() internally derives
        // negative constraints from the peers' narrow traversals and
        // wait intervals at narrow-incident vertices.
        // -----------------------------------------------------------
        ccbs_adapter::Path single_path;
        if (solver_->SolveSingleAgent(u, v, peer_plans, &single_path)
            && !single_path.empty()) {
            // Convert and commit only the caller; peers keep their
            // existing schedules untouched.
            Linestring caller_route;
            std::vector<int> caller_vids;
            std::vector<double> caller_sched;
            PathToRoute(single_path, t,
                        &caller_route, &caller_vids, &caller_sched);
            if (!caller_vids.empty()) {
                (*agents_)[agent_id].SetRoute(
                    caller_route, caller_vids, caller_sched, t);
                Statistics::Get().ccbs_singleagent_success++;
                Statistics::Get().ccbs_solve_time_s.Add(
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t_solve_start)
                        .count());
                return {caller_route, caller_vids};
            }
        }
    }

    // ------------------------------------------------------------------
    // Tier 2 — joint CCBS, with spatial peer filtering and adaptive
    // time budget. Single-agent failed (or wasn't applicable): the
    // peer-plan constraints we synthesized were too tight, or there's
    // an actual cooperative manoeuvre to plan. Run multi-agent CBS.
    // ------------------------------------------------------------------
    //
    // First we need a rough caller corridor for the relevance filter.
    // We approximate it with the unconstrained single-agent SIPP path
    // (no peer constraints) — that gives a cheap, conflict-agnostic
    // upper bound on the vertices the caller might pass through.
    std::vector<int> caller_corridor_vids;
    {
        ccbs_adapter::Path raw;
        if (solver_->SolveSingleAgent(u, v, /*peer_plans=*/{}, &raw)) {
            caller_corridor_vids.reserve(raw.size());
            for (const auto& st : raw) {
                caller_corridor_vids.push_back(st.id);
            }
        }
    }

    std::vector<std::pair<int, int>> subtasks;
    subtasks.push_back({u, v});
    const int caller_task_idx = 0;

    struct PeerEntry {
        int sim_agent_idx;
        int task_idx;
    };
    std::vector<PeerEntry> peers;

    if (can_use_peers) {
        for (std::size_t i = 0; i < agents_->size(); ++i) {
            if (static_cast<int>(i) == agent_id) continue;
            const Agent& a = (*agents_)[i];
            int s = -1, g = -1;
            if (!BuildSubtask(a, &s, &g)) continue;

            // Spatial filter: include only peers whose route is
            // plausibly relevant to the caller. When we have no
            // caller corridor estimate (raw SIPP failed), fall back
            // to including all peers — better safe than sorry.
            if (!caller_corridor_vids.empty()) {
                const auto& rf = a.route_follower;
                if (!PeerRelevant(caller_corridor_vids, rf.vertex_ids)) {
                    continue;
                }
            }

            peers.push_back({static_cast<int>(i),
                             static_cast<int>(subtasks.size())});
            subtasks.push_back({s, g});
        }
    }

    Statistics::Get().ccbs_joint_task_size.Add(
        static_cast<int>(subtasks.size()));

    // Adaptive time budget. We run a fast attempt first; only if it
    // returns "no solution" within the fast budget do we re-run with
    // the full slow budget. This makes the common case cheap while
    // preserving completeness on hard scenes.
    std::vector<ccbs_adapter::Path> paths;
    bool ok = false;
    {
        const double fast = std::min(fast_timelimit_s_, slow_timelimit_s_);
        solver_->SetTimeLimit(fast);
        ok = solver_->Solve(subtasks, &paths);
        if (!ok && fast < slow_timelimit_s_ - 1e-9) {
            solver_->SetTimeLimit(slow_timelimit_s_);
            ok = solver_->Solve(subtasks, &paths);
        }
        // Restore the slow budget so any direct callers of
        // SetSolverTimeLimit() see consistent state.
        solver_->SetTimeLimit(slow_timelimit_s_);
    }

    if (!ok) {
        // ------------------------------------------------------------------
        // Tier 3 — schedule-aware fallback. Before giving up to plain
        // A* we try one more single-agent SIPP attempt with the
        // already-built peer constraints (same as Tier 1 but it's
        // worth retrying because peer plans may have been refined
        // since). If that also fails we use plain A* as a last
        // resort, accepting that narrow-edge conflicts may briefly
        // re-emerge until the next successful replan.
        // ------------------------------------------------------------------
        ccbs_adapter::Path single_path;
        if (!peer_plans.empty()
            && solver_->SolveSingleAgent(u, v, peer_plans, &single_path)
            && !single_path.empty()) {
            Linestring caller_route;
            std::vector<int> caller_vids;
            std::vector<double> caller_sched;
            PathToRoute(single_path, t,
                        &caller_route, &caller_vids, &caller_sched);
            if (!caller_vids.empty()) {
                (*agents_)[agent_id].SetRoute(
                    caller_route, caller_vids, caller_sched, t);
                Statistics::Get().ccbs_singleagent_success++;
                Statistics::Get().ccbs_solve_time_s.Add(
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t_solve_start)
                        .count());
                LOG_INFO("CCBS joint solve failed for agent {}; resolved via single-agent SIPP fallback.",
                         agent_id);
                return {caller_route, caller_vids};
            }
        }

        LOG_WARNING("CCBS router failed to find joint conflict-free solution for agent {} ({} -> {}). Falling back to single-agent A* routing.",
                    agent_id, u, v);
        Statistics::Get().ccbs_fallback++;
        Statistics::Get().ccbs_solve_time_s.Add(
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_solve_start)
                .count());
        return fallback_router_.GetRouteWithVertices(u, v, t, agent_id);
    }

    Statistics::Get().ccbs_joint_success++;
    Statistics::Get().ccbs_solve_time_s.Add(
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_solve_start)
            .count());

    // Caller branch: Dispatch::Step() only invokes the router when the
    // caller agent is idle, so |u| is always the caller's current
    // vertex — no prefix stitching is needed and the schedule's
    // first entry naturally lands at t.
    Linestring caller_route;
    std::vector<int> caller_vids;
    std::vector<double> caller_sched;
    PathToRoute(paths[caller_task_idx], t,
                &caller_route, &caller_vids, &caller_sched);
    if (can_use_peers) {
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

        // Peer agents are typically mid-edge when CCBS replans them.
        // BuildSubtask() picked the *next* graph vertex as the sub-task
        // start, so |route.front()| is the position of that next
        // vertex — handing this directly to Agent::SetRoute() would
        // teleport the agent forward (RouteFollower::SetRoute resets
        // |x = 0; pos = route.front()|). Prepend the peer's current
        // position so the route geometry starts where the agent
        // actually is, and shift the first CCBS-stamp's schedule time
        // by the time it physically takes to finish the in-flight edge
        // at the same effective speed Simulation::Step() will apply.
        Agent& peer = (*agents_)[p.sim_agent_idx];
        const auto& rf = peer.route_follower;
        const auto cur_edge = rf.CurrentEdge();
        if (cur_edge.has_value() && rf.segment_idx < rf.vertex_ids.size()) {
            const double remaining =
                rf.CurrentEdgeLength() - rf.DistanceAlongEdge();
            bool is_narrow = false;
            const int eu = cur_edge->first;
            const int ev = cur_edge->second;
            if (eu >= 0 && eu < static_cast<int>(graph_->edges.size())) {
                auto it = graph_->edges[eu].find(ev);
                if (it != graph_->edges[eu].end()) {
                    is_narrow = it->second.narrow;
                }
            }
            const double effective_speed =
                max_speed_ * (is_narrow ? kNarrowEdgeSpeedFactor : 1.0);
            const double dt_finish = (effective_speed > 0.0 && remaining > 0.0)
                ? remaining / effective_speed
                : 0.0;

            // Prepend the peer's true current pos and a matching
            // vertex_id (re-use the previous vertex id — it's already
            // behind the agent so RouteFollower::Move() never revisits
            // it under monotone schedules). Shift the existing first
            // stamp's schedule by dt_finish so the agent has time to
            // physically reach it.
            Linestring stitched;
            stitched.reserve(route.size() + 1);
            stitched.push_back(rf.pos);
            for (const auto& pt : route) stitched.push_back(pt);

            std::vector<int> stitched_vids;
            stitched_vids.reserve(vids.size() + 1);
            stitched_vids.push_back(rf.vertex_ids[rf.segment_idx]);
            for (int id : vids) stitched_vids.push_back(id);

            std::vector<double> stitched_sched;
            stitched_sched.reserve(sched.size() + 1);
            stitched_sched.push_back(t);
            // Original sched.front() == t; the next vertex is reached
            // at t + dt_finish, all subsequent stamps shift by the
            // same offset so CCBS-scheduled waits stay synchronized.
            const double shift = dt_finish;
            for (double s : sched) stitched_sched.push_back(s + shift);

            peer.SetRoute(stitched, stitched_vids, stitched_sched, t);
        } else {
            // Peer has no in-flight edge (rare; e.g. just finished a
            // segment exactly on this tick). Apply the CCBS plan as-is.
            peer.SetRoute(route, vids, sched, t);
        }
    }

    return {caller_route, caller_vids};
}

}
