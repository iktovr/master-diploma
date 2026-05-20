#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "agent.h"
#include "graph.h"

// Resolver — alternative narrow-edge conflict resolution mechanism.
//
// When two agents traveling in opposite directions meet on a narrow edge,
// one of them (the "loser") starts moving backward (state = reverse) at
// kReverseSpeedFactor of the nominal speed, while the other one (the
// "pusher") continues forward. After the loser exits the edge backward it
// transitions to "wait" until *every* original pusher leaves the edge.
//
// If another agent appears behind a reversing loser (same direction as the
// loser's original travel) and catches up, it is also flipped to reverse
// and joins the conflict — but only for as long as the original pushers
// stay on the edge. New agents entering the edge while losers are waiting
// are not added to the dependency list until a fresh head-on conflict is
// detected.
struct Resolver {
    using EdgeKey = std::uint64_t;

    struct Conflict {
        // Undirected edge identity (min,max packed) so both directions
        // share the same conflict.
        EdgeKey edge_key = 0;
        // The two endpoints of the narrow edge.
        int u = -1;
        int v = -1;
        double edge_length = 0.0;

        // Direction the original pushers move along: pusher_from -> pusher_to.
        // Losers move in the opposite direction (pusher_to -> pusher_from)
        // along their own route — physically they retreat toward
        // pusher_to in the global edge frame.
        int pusher_from = -1;
        int pusher_to = -1;

        // The agents that were pushing forward when the conflict was
        // detected. The waiting losers must wait until every one of these
        // pushers has left the edge.
        std::vector<int> pushers;
        std::unordered_set<int> waiting_for;

        // Agents that have been reversed by this conflict (still on the
        // edge in reverse state).
        std::unordered_set<int> reversing;
        // Agents that have already backed off the edge and are waiting in
        // front of it.
        std::unordered_set<int> waiting_losers;
        // Time at which an agent entered the wait state (for waiting-time
        // statistics).
        std::unordered_map<int, double> wait_start;
    };

    std::shared_ptr<const Graph> graph;
    std::vector<Conflict> conflicts;
    // agent_id -> conflict index (only for losers in reverse/wait).
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
    // Returns true if the agent currently occupies (in either direction)
    // the narrow edge identified by edge_key. For agents in idle state or
    // off the edge returns false.
    bool AgentOnEdge(const Agent& a, EdgeKey edge_key) const;
    // Returns the directed (u,v) pair of the agent's current segment, or
    // nullopt if not on a routed segment.
    std::optional<std::pair<int, int>> AgentCurrentEdge(const Agent& a) const;
    void ReleaseConflict(std::size_t idx, double t, Agents& agents);
    void EraseConflict(std::size_t idx);
};
