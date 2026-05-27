#pragma once

// Thin, primitives-only bridge in front of the third-party CCBS
// (Continuous-time CBS) planner. Exists so that other translation
// units (CcbsRouter, tests) do not have to include any
// third_party/ccbs/* header (which would conflict with our own
// |Agent| and |Point| types in the global namespace).
//
// The bridge is stateful: the roadmap is built once via BuildMap()
// and reused across Solve() calls.

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

// A peer's already-committed plan, expressed in the same CCBS time
// axis (g) the caller will be planned against. |path| is a sequence
// of (id, g) stamps starting at g = the peer's earliest reachable
// time on this plan (typically the time the peer arrives at its
// next graph vertex, relative to the caller's planning origin).
//
// Consumed by Solver::SolveSingleAgent() to build forbidden-edge
// constraints that the caller's SIPP search must respect.
struct PeerPlan {
    Path path;
};

class Solver {
public:
    Solver();
    ~Solver();

    // Build the implicit roadmap from per-vertex |i,j| positions and
    // out-neighbor lists, mark every (u,v) in |narrow_edges|
    // as narrow, and set the narrow speed factor (<= 1.0) so that
    // narrow edges incur cost |distance|/factor instead of |distance|.
    void BuildMap(const std::vector<std::pair<double, double>>& node_pos,
                  const std::vector<std::vector<int>>& adj,
                  const std::vector<std::pair<int, int>>& narrow_edges,
                  double narrow_speed_factor);

    // Runs CCBS find_solution for the batched |subtasks|
    // (sub-task i is (start_id, goal_id)). Returns false if CCBS
    // fails to find a conflict-free joint plan; |*paths_out| is
    // left empty in that case. On success, |paths_out->size() ==
    // subtasks.size()| and each entry is the SIPP path in
    // (vertex id, g) form.
    bool Solve(const std::vector<std::pair<int, int>>& subtasks,
               std::vector<Path>* paths_out) const;

    // Single-agent fast path: run SIPP for one (start, goal) task
    // while treating |peer_plans| as already-committed, immutable
    // schedules. Forbidden-edge constraints are synthesized from
    // each peer's *narrow* edge traversals (wide moves cannot
    // conflict per the CCBS narrow-edge gate, so they are skipped).
    //
    // Returns true and writes the caller's path on success. Returns
    // false if SIPP cannot find any feasible single-agent path
    // honoring the constraints (in which case the caller should
    // fall through to the full joint CBS or to A*).
    //
    // This is the primary fast-path used by CcbsRouter to avoid
    // re-running multi-agent CBS on every dispatch event.
    bool SolveSingleAgent(int start_id, int goal_id,
                          const std::vector<PeerPlan>& peer_plans,
                          Path* path_out) const;

    // Bound wall-clock time spent inside CCBS::find_solution(). When
    // CCBS cannot find a conflict-free joint plan within |seconds|,
    // Solve() returns false and |*paths_out| is left empty. Values
    // <= 0 are clamped to a small positive default.
    void SetTimeLimit(double seconds);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ccbs_adapter

}
