#include "simulation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "logging.h"
#include "statistics.h"

namespace {

constexpr double kTimeEps = 1e-9;

struct Event {
    double time;
    double period;
    int    priority;
    std::function<void(double)> action;

    void Evaluate() {
        action(time);
        time += period;
    }
};

bool operator>(const Event& a, const Event& b) {
    if (std::abs(a.time - b.time) > kTimeEps) {
        return a.time > b.time;
    }
    return a.priority > b.priority;
}

inline std::uint64_t PackEdgeKey(int u, int v) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(u)) << 32)
         | static_cast<std::uint64_t>(static_cast<std::uint32_t>(v));
}

}  // namespace

Simulation::Simulation(
    const double speed_,
    const Agents& agents_,
    std::shared_ptr<const Graph> graph_,
    const std::shared_ptr<const IRouter> router_,
    const std::optional<Visualizer> vis_,
    ResolverKind resolver_kind)
    : speed(speed_)
    , agents(agents_)
    , graph(std::move(graph_))
    , dispatch(graph, router_, agents)
    , vis(vis_)
{
    if (resolver_kind == ResolverKind::semaphore) {
        semaphores.emplace(graph);
    } else if (resolver_kind == ResolverKind::reverse) {
        resolver.emplace(graph);
    }

    if (vis) {
        vis->DrawGraph(*graph);
        vis->SavePersistentPart();
    }
    dispatch.AssignBasePoints(agents);

    edge_capacities_.reserve(graph->edges.size() * 2);
    edge_lengths_.reserve(graph->edges.size() * 2);
    for (int u = 0; u < static_cast<int>(graph->edges.size()); ++u) {
        for (const auto& [v, edge] : graph->edges[u]) {
            const int cap = edge.narrow ? kNarrowEdgeCapacity : kWideEdgeCapacity;
            edge_capacities_.emplace_back(PackEdgeKey(u, v), cap);
            edge_lengths_.emplace_back(PackEdgeKey(u, v), edge.length);
        }
    }
    std::sort(edge_capacities_.begin(), edge_capacities_.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    std::sort(edge_lengths_.begin(), edge_lengths_.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
}

// Follow-cap interop with CCBS-scheduled agents
// ---------------------------------------------
// The per-edge capacity caps (kNarrowEdgeCapacity / kWideEdgeCapacity)
// and the kAgentFollowGap minimum spacing exist to keep agents from
// piling up on top of each other in *unscheduled* simulations (plain
// A*, semaphore, reverse resolver). They are unnecessary — and can
// be actively harmful — for agents whose motion is dictated by a
// CCBS schedule:
//
//   * Narrow edges: CCBS guarantees at most one agent per (directed)
//     narrow edge at a time, so caps never fire there anyway.
//   * Wide edges with up to kWideEdgeCapacity (3) co-located agents:
//     |bucket_size > cap| is false; caps already don't fire.
//   * Wide edges with >3 agents: CCBS legitimately plans them this
//     way and times them so the ScheduledPosition clamp inside
//     RouteFollower::Move() prevents any overrun of a leader. Caps
//     applied here would only desynchronize the trailing agents
//     from their CCBS schedule and propagate delay downstream.
//
// We therefore keep a CCBS-scheduled agent in the bucket as a
// leader/obstacle (so unscheduled followers still respect the gap
// behind it), but never write a cap *into* a CCBS-scheduled
// follower's slot. The forward-position bound for those agents
// remains RouteFollower::ScheduledPosition().
void Simulation::ComputeFollowCaps(std::vector<double>& out) const {
    const double kInf = std::numeric_limits<double>::infinity();
    out.assign(agents.size(), kInf);

    auto& entries = follow_scratch_;
    entries.clear();
    entries.reserve(agents.size() * 2);
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const auto& a = agents[i];
        const auto edge = a.route_follower.CurrentEdge();
        if (!edge) {
            continue;
        }
        const double x = a.route_follower.DistanceAlongEdge();
        if (a.state == Agent::move) {
            entries.push_back(FollowEntry{
                PackEdgeKey(edge->first, edge->second),
                x,
                static_cast<int>(i),
            });
        } else if (a.state == Agent::reverse) {
            // Obstacle in own (directed) bucket — slows trailers in the
            // loser's original direction.
            entries.push_back(FollowEntry{
                PackEdgeKey(edge->first, edge->second),
                x,
                -1,
            });
            // Obstacle in the opposite (directed) bucket — slows the
            // pusher behind it.
            const double L = a.route_follower.CurrentEdgeLength();
            entries.push_back(FollowEntry{
                PackEdgeKey(edge->second, edge->first),
                L - x,
                -1,
            });
        }
    }

    if (entries.size() < 2) {
        return;
    }

    std::sort(entries.begin(), entries.end(),
        [](const FollowEntry& a, const FollowEntry& b) {
            if (a.edge_key != b.edge_key) {
                return a.edge_key < b.edge_key;
            }
            return a.x_on_edge < b.x_on_edge;
        });

    const std::size_t n = entries.size();
    std::size_t i = 0;
    while (i < n) {
        std::size_t j = i + 1;
        const std::uint64_t key = entries[i].edge_key;
        while (j < n && entries[j].edge_key == key) {
            ++j;
        }
        const std::size_t bucket_size = j - i;
        if (bucket_size >= 2) {
            int capacity = kNarrowEdgeCapacity;
            auto it = std::lower_bound(edge_capacities_.begin(), edge_capacities_.end(), key,
                [](const std::pair<std::uint64_t, int>& a, std::uint64_t k) {
                    return a.first < k;
                });
            if (it != edge_capacities_.end() && it->first == key) {
                capacity = it->second;
            }

            const std::size_t cap = static_cast<std::size_t>(capacity);
            if (bucket_size > cap) {
                for (std::size_t k = i; k + cap < j; ++k) {
                    const FollowEntry& follower = entries[k];
                    const FollowEntry& leader = entries[k + cap];
                    if (follower.agent_id < 0) {
                        continue;  // obstacle never receives a cap
                    }
                    // CCBS-managed followers ignore the capacity cap:
                    // their position is already bounded by the
                    // ScheduledPosition() clamp in RouteFollower::Move().
                    // See the file-level comment above ComputeFollowCaps.
                    if (!agents[follower.agent_id]
                             .route_follower.segment_schedule_t.empty()) {
                        continue;
                    }
                    const double allowed = leader.x_on_edge - kAgentFollowGap - follower.x_on_edge;
                    const double cap_val = allowed > 0.0 ? allowed : 0.0;
                    if (cap_val < out[follower.agent_id]) {
                        out[follower.agent_id] = cap_val;
                    }
                }
            }
        }
        i = j;
    }
}

void Simulation::Step(const double t, const double dt) {
    if (semaphores) {
        semaphores->Step(t, agents);
    }
    if (resolver) {
        resolver->Step(t, dt, agents);
    }
    dispatch.Step(t, agents);

    ComputeFollowCaps(follow_caps_scratch_);
    for (std::size_t i = 0; i < agents.size(); ++i) {
        double agent_speed = speed;
        const auto edge = agents[i].route_follower.CurrentEdge();
        if (edge) {
            const auto key = PackEdgeKey(edge->first, edge->second);
            auto it = std::lower_bound(edge_capacities_.begin(), edge_capacities_.end(), key,
                [](const std::pair<std::uint64_t, int>& a, std::uint64_t k) {
                    return a.first < k;
                });
            if (it != edge_capacities_.end() && it->first == key
                && it->second == kNarrowEdgeCapacity) {
                agent_speed *= kNarrowEdgeSpeedFactor;
            }
        }
        agents[i].Move(t, dt, agent_speed, follow_caps_scratch_[i]);
    }
}

void Simulation::Visualize(const double t) {
    if (!vis) {
        return;
    }
    vis->ClearFrame();
    vis->DrawGraphStatistics(*graph, Statistics::Get().edges, t);
    for (const auto& agent : agents) {
        vis->DrawAgent(agent);
    }
    vis->SaveFrame();
}

void Simulation::Simulate(const double duration, const double step, const double vis_step) {
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events;

    events.push(Event{step, step, 0,
        [this, step](double t) { Step(t, step); }});

    if (vis && vis_step > 0) {
        events.push(Event{0.0, vis_step, 1,
            [this](double t) { Visualize(t); }});
    }

    const double progress_step = duration / 10.0;
    if (progress_step > 0) {
        events.push(Event{0.0, progress_step, 2,
            [duration](double t) {
                LOG_INFO("Simulation progress: {:.1f}% (t = {:.2f} / {:.2f})",
                        100.0 * t / duration, t, duration);
                }});
    }

    auto start = std::chrono::high_resolution_clock::now();

    while (!events.empty()) {
        Event event = std::move(const_cast<Event&>(events.top()));
        events.pop();
        if (event.time > duration + kTimeEps) {
            break;
        }
        event.Evaluate();
        events.push(std::move(event));
    }

    auto end = std::chrono::high_resolution_clock::now();
    double real_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e9;
    LOG_INFO("Simulation virtual duration: {:.5f}", duration);
    LOG_INFO("Simulation real duration: {:.5f}", real_duration);
    LOG_INFO("Simulation speed: {:.3f} s(v)/s", duration / real_duration);
}
