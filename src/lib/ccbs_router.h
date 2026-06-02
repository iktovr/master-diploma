#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "agent.h"
#include "geometry.h"
#include "graph.h"
#include "router.h"

namespace lib {

// Forward declarations to avoid pulling <third_party/ccbs/*> headers into every TU.
// CCBS internals reached through ccbs_adapter::Solver bridge (ccbs_solver.h),
// avoiding conflicting global names (Agent, Point).
namespace ccbs_adapter {
class Solver;
struct Stamp;
struct PeerPlan;
using Path = std::vector<Stamp>;
}

// IRouter adapter for CCBS (Continuous-time Conflict-Based Search). Batch multi-agent planner:
// single GetRouteWithVertices() call replans requesting agent + all agents in move/wait state
class CcbsRouter : public IRouter {
public:
    CcbsRouter(std::shared_ptr<const Graph> graph,
               Agents* agents,
               double max_speed);
    ~CcbsRouter() override;

    void SetAgents(Agents* agents) { agents_ = agents; }  // Back-patch live Agents vector for peer replanning

    void SetSolverTimeLimit(double seconds);
    void SetFastSolverTimeLimit(double seconds);

    Linestring GetRoute(const int u, const int v, const double t = 0.0,
                        const int agent_id = -1) const override;
    std::pair<Linestring, std::vector<int>> GetRouteWithVertices(
        const int u, const int v, const double t = 0.0,
        const int agent_id = -1) const override;

protected:
    double Heuristic(const int u, const int v) const override;
    double Cost(const int u, const int v, const Graph::Edge& edge,
                const double t) const override;

private:
    void EnsureMapBuilt() const;

    void PathToRoute(const ccbs_adapter::Path& path,
                     double t_start,
                     Linestring* out_route,
                     std::vector<int>* out_vertex_ids,
                     std::vector<double>* out_schedule_t) const;

    bool BuildSubtask(const Agent& a,
                      int* start_id_out,
                      int* goal_id_out) const;

    bool BuildPeerPlan(const Agent& peer, double t_now,
                       ccbs_adapter::PeerPlan* out) const;

    bool PeerRelevant(const std::vector<int>& caller_vids,
                      const std::vector<int>& peer_vids) const;

    Agents* agents_;
    double max_speed_;

    double fast_timelimit_s_ = 0.2;
    double slow_timelimit_s_ = 5.0;

    mutable std::mutex mu_; 
    std::unique_ptr<ccbs_adapter::Solver> solver_;
    mutable bool map_built_ = false;

    AStarRouter fallback_router_;
};

}
