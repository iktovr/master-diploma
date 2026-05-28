#include "ccbs_solver.h"

#include "third_party/ccbs/cbs.h"
#include "third_party/ccbs/config.h"
#include "third_party/ccbs/heuristic.h"
#include "third_party/ccbs/map.h"
#include "third_party/ccbs/sipp.h"
#include "third_party/ccbs/structs.h"
#include "third_party/ccbs/task.h"

#include <algorithm>
#include <cstddef>
#include <list>
#include <memory>
#include <utility>
#include <vector>


namespace lib {

namespace ccbs_adapter {

struct Solver::Impl {
    std::unique_ptr<::Map> map;
    double timelimit_s = 0.5;
};

Solver::Solver() : impl_(new Impl) {}
Solver::~Solver() = default;

void Solver::SetTimeLimit(double seconds) {
    impl_->timelimit_s = seconds > 0.0 ? seconds : 0.5;
}

void Solver::BuildMap(const std::vector<std::pair<double, double>>& node_pos,
                     const std::vector<std::vector<int>>& adj,
                     const std::vector<std::pair<int, int>>& narrow_edges,
                     double narrow_speed_factor) {
    impl_->map.reset(new ::Map(/*size=*/CN_AGENT_SIZE, /*k=*/2));

    std::vector<::gNode> nodes;
    nodes.reserve(node_pos.size());
    for (const auto& p : node_pos) {
        ::gNode n;
        n.i = p.first;
        n.j = p.second;
        nodes.push_back(n);
    }
    impl_->map->build_roadmap(nodes, adj);
    for (const auto& e : narrow_edges) {
        impl_->map->set_narrow_edge(e.first, e.second);
    }
    impl_->map->set_narrow_speed_factor(narrow_speed_factor);
}

namespace {

inline void ApplyFastConfig(::Config* cfg) {
    cfg->use_cardinal = true;
    cfg->use_disjoint_splitting = true;
    cfg->hlh_type = 1;
    cfg->focal_weight = 1.0;
}

}  // namespace

bool Solver::Solve(const std::vector<std::pair<int, int>>& subtasks,
                   std::vector<Path>* paths_out) const {
    paths_out->clear();
    if (!impl_->map) return false;

    ::Task task;
    for (const auto& s : subtasks) {
        task.add_agent(s.first, s.second, *impl_->map);
    }

    ::CBS cbs;
    ::Config cfg;
    ApplyFastConfig(&cfg);
    cfg.timelimit = impl_->timelimit_s;
    ::Solution sol = cbs.find_solution(*impl_->map, task, cfg);
    if (!sol.found || sol.paths.size() != subtasks.size()) return false;

    paths_out->resize(sol.paths.size());
    for (std::size_t i = 0; i < sol.paths.size(); ++i) {
        Path& dst = (*paths_out)[i];
        dst.reserve(sol.paths[i].nodes.size());
        for (const auto& sn : sol.paths[i].nodes) {
            Stamp st;
            st.id = sn.id;
            st.g = sn.g;
            dst.push_back(st);
        }
    }
    return true;
}

bool Solver::SolveSingleAgent(int start_id, int goal_id,
                              const std::vector<PeerPlan>& peer_plans,
                              Path* path_out) const {
    path_out->clear();
    if (!impl_->map) return false;
    if (start_id == goal_id) return false;

    const ::Map& map = *impl_->map;
    const int n = map.get_size();
    if (start_id < 0 || start_id >= n) return false;
    if (goal_id  < 0 || goal_id  >= n) return false;

    // Build the constraint list to feed SIPP. For each *narrow*
    // traversal in each peer plan we forbid the caller from
    // performing the reverse traversal during the geometric overlap
    // window. Wide-edge traversals are exempted because the
    // upstream Map::has_narrow_set() gate (see CBS::check_conflict)
    // makes wide moves conflict-free by construction in our setup.
    //
    // The forbidden window is a conservative outer bound of the
    // exact CCBS get_constraint() bisection result: we forbid
    // *starting* the reverse move during [t1 - dur, t2] where
    // dur = t2 - t1 is the peer's traversal time. CCBS's exact
    // computation always falls inside this window (continuity of
    // the collision predicate in time + identical traversal
    // durations in both directions on our undirected map).
    //
    // We also forbid the caller from waiting at the peer's
    // intermediate vertices during the peer's wait intervals on
    // narrow-incident nodes (when the peer is parked there CCBS
    // would not let the caller transit through). For peers'
    // *terminal* dwell (the goal vertex past the path's last
    // stamp) we emit an infinite-tail constraint, matching the
    // semantics CCBS uses for finished agents.
    std::list<::Constraint> cons;
    const int caller_id = 0;  // SIPP uses agent.id only to look up
                              // h_values; we pass a self-consistent
                              // id throughout.

    auto is_narrow = [&](int u, int v) {
        return u != v && map.is_narrow_edge(u, v);
    };
    auto has_narrow_at = [&](int v) {
        return map.has_narrow_at_vertex(v);
    };

    for (const PeerPlan& peer : peer_plans) {
        const Path& p = peer.path;
        if (p.size() < 2) continue;
        for (std::size_t k = 0; k + 1 < p.size(); ++k) {
            const int  u  = p[k].id;
            const int  v  = p[k + 1].id;
            const double t1 = p[k].g;
            const double t2 = p[k + 1].g;
            if (!(t2 > t1)) continue;  // safety
            const double dur = t2 - t1;

            if (u != v) {
                if (!is_narrow(u, v)) continue;
                cons.emplace_back(caller_id,
                                  t1 - dur,
                                  t2,
                                  v, u);
            } else {
                if (!has_narrow_at(u)) continue;
                cons.emplace_back(caller_id, t1, t2, u, u);
            }
        }
        const int term_id = p.back().id;
        if (has_narrow_at(term_id)) {
            cons.emplace_back(caller_id, p.back().g, CN_INFINITY,
                              term_id, term_id);
        }
    }

    ::Heuristic h;
    h.init(n, 1);
    ::Agent agent(start_id, goal_id, caller_id);
    const ::gNode gs = map.get_gNode(start_id);
    const ::gNode gg = map.get_gNode(goal_id);
    agent.start_i = gs.i; agent.start_j = gs.j;
    agent.goal_i  = gg.i; agent.goal_j  = gg.j;
    agent.size    = CN_AGENT_SIZE;
    h.count(map, agent);

    ::SIPP planner;
    ::Path sipp_path = planner.find_path(agent, map, cons, h);
    if (sipp_path.cost < 0 || sipp_path.nodes.empty()) {
        return false;
    }

    path_out->reserve(sipp_path.nodes.size());
    for (const auto& nd : sipp_path.nodes) {
        Stamp st;
        st.id = nd.id;
        st.g  = nd.g;
        path_out->push_back(st);
    }
    return true;
}

}  // namespace ccbs_adapter

}