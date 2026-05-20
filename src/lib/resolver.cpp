#include "resolver.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include "statistics.h"

Resolver::Resolver(std::shared_ptr<const Graph> graph_)
    : graph(std::move(graph_)) {}

bool Resolver::IsNarrow(int u, int v) const {
    if (u < 0 || u >= static_cast<int>(graph->edges.size())) {
        return false;
    }
    auto it = graph->edges[u].find(v);
    if (it == graph->edges[u].end()) {
        return false;
    }
    return it->second.narrow;
}

std::optional<std::pair<int, int>> Resolver::AgentCurrentEdge(const Agent& a) const {
    return a.route_follower.CurrentEdge();
}

bool Resolver::AgentOnEdge(const Agent& a, EdgeKey edge_key) const {
    // An agent that has finished its route is no longer on any edge, even if
    // its route_follower hasn't bumped segment_idx (this happens when the
    // route is advanced via the no-stats Move(dx) overload). Treat such an
    // agent — and any idle agent — as off the edge so pushers correctly
    // release the conflict once they've fully traversed it.
    if (a.state == Agent::idle || a.route_follower.IsFinished()) {
        return false;
    }
    auto e = AgentCurrentEdge(a);
    if (!e) {
        return false;
    }
    return PackEdgeKey(e->first, e->second) == edge_key;
}

void Resolver::EraseConflict(std::size_t idx) {
    // Remove agent_to_conflict entries that point to this conflict before
    // erasing it, then fix-up indices for the last-swapped conflict.
    auto& c = conflicts[idx];
    for (int a : c.reversing) {
        agent_to_conflict.erase(a);
    }
    for (int a : c.waiting_losers) {
        agent_to_conflict.erase(a);
    }
    const std::size_t last = conflicts.size() - 1;
    if (idx != last) {
        auto& moved = conflicts[last];
        for (int a : moved.reversing) {
            agent_to_conflict[a] = static_cast<int>(idx);
        }
        for (int a : moved.waiting_losers) {
            agent_to_conflict[a] = static_cast<int>(idx);
        }
        conflicts[idx] = std::move(conflicts[last]);
    }
    conflicts.pop_back();
}

void Resolver::ReleaseConflict(std::size_t idx, double t, Agents& agents) {
    auto& c = conflicts[idx];
    for (int a : c.waiting_losers) {
        if (a >= 0 && a < static_cast<int>(agents.size())) {
            agents[a].state = Agent::move;
            auto it = c.wait_start.find(a);
            if (it != c.wait_start.end() && it->second < t) {
                Statistics::Get().waiting_time.Add(t - it->second);
            }
        }
    }
    // Any agents still in reverse at this point will also be released —
    // they continue forward immediately (rare edge case if pushers leave
    // before losers manage to back off entirely, e.g. very short edges).
    for (int a : c.reversing) {
        if (a >= 0 && a < static_cast<int>(agents.size())) {
            agents[a].state = Agent::move;
        }
    }
    EraseConflict(idx);
}

void Resolver::Step(double t, double dt, Agents& agents) {
    // ----- 1. Accumulate reverse_time statistics for current reversing agents.
    auto& stats = Statistics::Get();
    for (const auto& a : agents) {
        if (a.state == Agent::reverse) {
            stats.reverse_time.Add(dt);
        }
    }

    // ----- 2. Update existing conflicts: pusher exits, loser transitions,
    //         cascade onto trailers, release when complete.
    for (std::size_t i = 0; i < conflicts.size(); /* ++i below */) {
        auto& c = conflicts[i];

        // (a) Pushers that have left the edge.
        for (auto it = c.waiting_for.begin(); it != c.waiting_for.end(); /**/) {
            const int pid = *it;
            if (pid < 0 || pid >= static_cast<int>(agents.size())
                || !AgentOnEdge(agents[pid], c.edge_key)) {
                it = c.waiting_for.erase(it);
            } else {
                ++it;
            }
        }

        // (a2) Winner-side newcomers: agents in move state that have entered
        // the conflict edge in the pushers' direction and are within
        // kAgentFollowGap of any pusher still on the edge (or any reverser
        // still on the edge). They are absorbed into the conflict's pushers
        // set so the waiting losers must wait for them too.
        if (!c.waiting_for.empty()) {
            // Collect "obstacle" positions in the pusher's directed frame
            // (DistanceAlongEdge measured from pusher_from).
            std::vector<double> obstacle_x;
            obstacle_x.reserve(c.waiting_for.size() + c.reversing.size());
            for (int pid : c.waiting_for) {
                if (pid < 0 || pid >= static_cast<int>(agents.size())) continue;
                const auto& pa = agents[pid];
                if (!AgentOnEdge(pa, c.edge_key)) continue;
                obstacle_x.push_back(pa.route_follower.DistanceAlongEdge());
            }
            for (int rid : c.reversing) {
                if (rid < 0 || rid >= static_cast<int>(agents.size())) continue;
                const auto& ra = agents[rid];
                if (!AgentOnEdge(ra, c.edge_key)) continue;
                // Reverser is on the loser-direction edge; its position in
                // the pusher frame is edge_length - x_loser.
                obstacle_x.push_back(
                    c.edge_length - ra.route_follower.DistanceAlongEdge());
            }
            if (!obstacle_x.empty()) {
                for (std::size_t j = 0; j < agents.size(); ++j) {
                    auto& ag = agents[j];
                    if (ag.state != Agent::move) continue;
                    if (c.waiting_for.contains(static_cast<int>(j))) continue;
                    if (c.reversing.contains(static_cast<int>(j))) continue;
                    if (c.waiting_losers.contains(static_cast<int>(j))) continue;
                    if (agent_to_conflict.contains(static_cast<int>(j))) continue;
                    auto e = AgentCurrentEdge(ag);
                    if (!e) continue;
                    if (PackEdgeKey(e->first, e->second) != c.edge_key) continue;
                    // Must travel in the pusher's direction.
                    if (e->first != c.pusher_from || e->second != c.pusher_to) {
                        continue;
                    }
                    const double x_new = ag.route_follower.DistanceAlongEdge();
                    bool absorb = false;
                    for (double xo : obstacle_x) {
                        if (std::abs(xo - x_new) < kAgentFollowGap) {
                            absorb = true;
                            break;
                        }
                    }
                    if (absorb) {
                        c.pushers.push_back(static_cast<int>(j));
                        c.waiting_for.insert(static_cast<int>(j));
                        agent_to_conflict[static_cast<int>(j)]
                            = static_cast<int>(i);
                        stats.conflicts_count += 1;
                    }
                }
            }
        }

        // (b) Reversing agents that have backed off the edge -> wait.
        // An agent counts as having "left the edge" either when its
        // current edge changed (segment index decremented onto a previous
        // route segment) or when it has reversed all the way to the start
        // of its current segment (no prior segment to retreat onto).
        constexpr double kEdgeExitEps = 1e-6;
        for (auto it = c.reversing.begin(); it != c.reversing.end(); /**/) {
            const int aid = *it;
            if (aid < 0 || aid >= static_cast<int>(agents.size())) {
                it = c.reversing.erase(it);
                continue;
            }
            auto& ag = agents[aid];
            const bool off_edge =
                !AgentOnEdge(ag, c.edge_key)
                || ag.route_follower.DistanceAlongEdge() <= kEdgeExitEps;
            if (off_edge) {
                // Back-tracked off the edge — start waiting.
                ag.state = Agent::wait;
                c.waiting_losers.insert(aid);
                c.wait_start[aid] = t;
                it = c.reversing.erase(it);
            } else {
                ++it;
            }
        }

        // (c) Release if no more pushers blocking.
        if (c.waiting_for.empty()) {
            ReleaseConflict(i, t, agents);
            // Don't increment i — element at i is now the swapped one.
            continue;
        }

        // (d) Cascade: same-direction (pusher_to → pusher_from) move-state
        //     trailers that have caught up to any reversing loser become
        //     reverse themselves. They join the conflict but are NOT added
        //     to the pushers/waiting_for set.
        double min_loser_x_route = std::numeric_limits<double>::infinity();
        for (int aid : c.reversing) {
            const auto& ag = agents[aid];
            const double x = ag.route_follower.DistanceAlongEdge();
            min_loser_x_route = std::min(min_loser_x_route, x);
        }
        if (min_loser_x_route < std::numeric_limits<double>::infinity()) {
            for (std::size_t j = 0; j < agents.size(); ++j) {
                auto& ag = agents[j];
                if (ag.state != Agent::move) {
                    continue;
                }
                if (c.reversing.contains(static_cast<int>(j))
                    || c.waiting_losers.contains(static_cast<int>(j))) {
                    continue;
                }
                if (agent_to_conflict.contains(static_cast<int>(j))) {
                    continue;
                }
                auto e = AgentCurrentEdge(ag);
                if (!e) continue;
                if (PackEdgeKey(e->first, e->second) != c.edge_key) continue;
                // Only trailers traveling in the loser's original direction.
                if (e->first != c.pusher_to || e->second != c.pusher_from) {
                    continue;
                }
                const double x_c = ag.route_follower.DistanceAlongEdge();
                if (x_c < min_loser_x_route
                    && (min_loser_x_route - x_c) < kAgentFollowGap) {
                    ag.state = Agent::reverse;
                    c.reversing.insert(static_cast<int>(j));
                    agent_to_conflict[static_cast<int>(j)] = static_cast<int>(i);
                    stats.conflicts_count += 1;
                }
            }
        }

        ++i;
    }

    // ----- 3. Detect new head-on conflicts on narrow edges.
    // Build a list of move-state agents grouped by undirected edge key.
    struct AgentOnNarrow {
        int agent_id;
        int u_dir;
        int v_dir;
        double x;
        double edge_length;
        EdgeKey key;
    };
    std::vector<AgentOnNarrow> per_edge;
    per_edge.reserve(agents.size());
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const auto& a = agents[i];
        if (a.state != Agent::move) continue;
        auto e = AgentCurrentEdge(a);
        if (!e) continue;
        if (!IsNarrow(e->first, e->second)) continue;
        const EdgeKey key = PackEdgeKey(e->first, e->second);
        // Skip edges where a conflict is already active.
        bool skip = false;
        for (const auto& c : conflicts) {
            if (c.edge_key == key) { skip = true; break; }
        }
        if (skip) continue;
        per_edge.push_back(AgentOnNarrow{
            static_cast<int>(i), e->first, e->second,
            a.route_follower.DistanceAlongEdge(),
            a.route_follower.CurrentEdgeLength(),
            key,
        });
    }

    std::sort(per_edge.begin(), per_edge.end(),
              [](const AgentOnNarrow& a, const AgentOnNarrow& b) {
                  return a.key < b.key;
              });

    std::size_t i = 0;
    while (i < per_edge.size()) {
        std::size_t j = i + 1;
        while (j < per_edge.size() && per_edge[j].key == per_edge[i].key) ++j;

        // Within [i, j) find an opposing pair.
        int idx_a = -1;
        int idx_b = -1;
        for (std::size_t p = i; p < j && idx_a < 0; ++p) {
            for (std::size_t q = p + 1; q < j; ++q) {
                if (per_edge[p].u_dir == per_edge[q].v_dir
                    && per_edge[p].v_dir == per_edge[q].u_dir) {
                    // Same edge, opposite directions. Check physical gap.
                    const double L = per_edge[p].edge_length;
                    const double phys_gap = L - per_edge[p].x - per_edge[q].x;
                    if (phys_gap < kAgentFollowGap) {
                        idx_a = static_cast<int>(p);
                        idx_b = static_cast<int>(q);
                        break;
                    }
                }
            }
        }

        if (idx_a >= 0) {
            // Loser: the agent farther from its own edge end (i.e. with the
            // smaller DistanceAlongEdge along its directed edge). The
            // intuition: the agent closer to its edge end is also closer
            // to clearing the conflict so we let it through. Ties are
            // broken deterministically by smaller agent_id losing.
            const AgentOnNarrow* A = &per_edge[idx_a];
            const AgentOnNarrow* B = &per_edge[idx_b];
            const AgentOnNarrow* loser;
            const AgentOnNarrow* pusher;
            if (A->x != B->x) {
                loser  = (A->x < B->x) ? A : B;
                pusher = (loser == A) ? B : A;
            } else {
                loser  = (A->agent_id < B->agent_id) ? A : B;
                pusher = (loser == A) ? B : A;
            }

            Conflict c;
            c.edge_key = loser->key;
            c.u = loser->u_dir;            // loser->v_dir = pusher->u_dir
            c.v = loser->v_dir;
            c.edge_length = loser->edge_length;
            c.pusher_from = pusher->u_dir;
            c.pusher_to   = pusher->v_dir;
            c.pushers.push_back(pusher->agent_id);
            c.waiting_for.insert(pusher->agent_id);
            c.reversing.insert(loser->agent_id);

            agents[loser->agent_id].state = Agent::reverse;
            stats.conflicts_count += 1;

            const int conflict_idx = static_cast<int>(conflicts.size());
            agent_to_conflict[loser->agent_id] = conflict_idx;
            conflicts.push_back(std::move(c));
        }

        i = j;
    }
}
