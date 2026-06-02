#pragma once

// Thin primitives-only bridge to third-party CCBS (Continuous-time CBS) planner.
// Stateful: roadmap built once via BuildMap() and reused across Solve() calls.

#include <memory>
#include <utility>
#include <vector>


namespace lib {

namespace ccbs_adapter {

struct Stamp {
    int id;     // graph vertex id
    double g;   // CCBS time-cost (distance in graph units, unit speed)
};

using Path = std::vector<Stamp>;

// Peer's committed plan in CCBS time axis (g). Path is (id,g) stamps starting at
// peer's earliest reachable time (typically arrival at next vertex, relative to caller's origin).
// Consumed by SolveSingleAgent() to build forbidden-edge constraints for caller's SIPP search.
struct PeerPlan {
    Path path;
};

class Solver {
public:
    Solver();
    ~Solver();

    void BuildMap(const std::vector<std::pair<double, double>>& node_pos,  // Build roadmap from positions, adjacency, narrow edges
                  const std::vector<std::vector<int>>& adj,
                  const std::vector<std::pair<int, int>>& narrow_edges,
                  double narrow_speed_factor);

    bool Solve(const std::vector<std::pair<int, int>>& subtasks,  // Run CCBS find_solution for batched (start,goal) tasks
               std::vector<Path>* paths_out) const;  // Returns false if no conflict-free plan (paths_out empty)

    bool SolveSingleAgent(int start_id, int goal_id,  // Single-agent fast path: SIPP with frozen peer_plans
                          const std::vector<PeerPlan>& peer_plans,
                          Path* path_out) const;  // Returns false if no feasible path (fall through to joint CBS/A*)

    void SetTimeLimit(double seconds);  // Bound wall-clock time for CCBS::find_solution() (<=0 clamped to small default)

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ccbs_adapter

}
