#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "agent.h"
#include "dispatch.h"
#include "graph.h"
#include "resolver.h"
#include "router.h"
#include "semaphore.h"
#include "visualizer.h"

namespace lib {

enum class ResolverKind {
    none,
    semaphore,
    reverse,
};

struct Simulation {
    double speed;
    Agents agents;
    std::shared_ptr<const Graph> graph;
    Dispatch dispatch;
    std::optional<SemaphoreManager> semaphores;
    std::optional<Resolver> resolver;
    std::optional<Visualizer> vis;

    Simulation(
        const double speed_,
        const Agents& agents_,
        std::shared_ptr<const Graph> graph_,
        const std::shared_ptr<const IRouter> router_,
        const std::optional<Visualizer> vis_ = std::nullopt,
        ResolverKind resolver_kind = ResolverKind::none);

    void AddAgent(const Agent& agent) {
        agents.push_back(agent);
    }

    template <class... Args>
    void AddAgent(Args... args) {
        agents.emplace_back(args...);
    }

    void Simulate(const double duration, const double step, const double vis_step = -1);
    void Visualize(const double t);
    void Step(const double t, const double dt);

    void ComputeFollowCaps(std::vector<double>& out) const;

    std::vector<double> ComputeFollowCaps() const {
        std::vector<double> out;
        ComputeFollowCaps(out);
        return out;
    }

private:
    std::vector<std::pair<std::uint64_t, int>> edge_capacities_;
    std::vector<std::pair<std::uint64_t, double>> edge_lengths_;

    struct FollowEntry {
        std::uint64_t edge_key;
        double x_on_edge;
        int agent_id;     // -1 for obstacle-only (reverse agent re-entry).
    };
    mutable std::vector<FollowEntry> follow_scratch_;
    mutable std::vector<double> follow_caps_scratch_;
};

}
