#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "agent.h"
#include "graph.h"

namespace lib {

struct Resolver {
    using EdgeKey = std::uint64_t;

    struct Conflict {
        EdgeKey edge_key = 0;
        int u = -1;
        int v = -1;
        double edge_length = 0.0;

        int pusher_from = -1;
        int pusher_to = -1;
        std::vector<int> pushers;
        std::unordered_set<int> waiting_for;

        std::unordered_set<int> reversing;
        std::unordered_set<int> waiting_losers;
        std::unordered_map<int, double> wait_start;
        std::unordered_map<int, double> reverse_start;
    };

    std::shared_ptr<const Graph> graph;
    std::vector<Conflict> conflicts;
    std::unordered_map<int, int> agent_to_conflict;

    explicit Resolver(std::shared_ptr<const Graph> graph_);

    void Step(double t, double dt, Agents& agents);

    static EdgeKey PackEdgeKey(int a, int b) {
        const int lo = a < b ? a : b;
        const int hi = a < b ? b : a;
        return (static_cast<EdgeKey>(static_cast<std::uint32_t>(lo)) << 32)
             |  static_cast<EdgeKey>(static_cast<std::uint32_t>(hi));
    }

private:
    bool IsNarrow(int u, int v) const;
    bool AgentOnEdge(const Agent& a, EdgeKey edge_key) const;
    std::optional<std::pair<int, int>> AgentCurrentEdge(const Agent& a) const;
    void ReleaseConflict(std::size_t idx, double t, Agents& agents);
    void EraseConflict(std::size_t idx);
};

}
