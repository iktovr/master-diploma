#include "ccbs_solver.h"

#include "third_party/ccbs/cbs.h"
#include "third_party/ccbs/config.h"
#include "third_party/ccbs/map.h"
#include "third_party/ccbs/structs.h"
#include "third_party/ccbs/task.h"

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

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

}  // namespace ccbs_adapter
